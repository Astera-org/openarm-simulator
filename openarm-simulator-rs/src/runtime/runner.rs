//! Single-threaded simulation/CAN owner. Pausing time never pauses socket I/O.
use super::{Calls, Clock, Conflict, NotFound, Reply, Request, can};
use crate::{physics::Physics, simulation::Simulation};
use anyhow::{Context, Result, bail, ensure};
use damiao_can_rs::MotorStatus;
use openarm_simulator_core_rs::{Configuration, State, Statistics};
use serde_json::Value;
use socketcan::CanFdSocket;
use std::{
    io::{self, Read},
    os::fd::{AsRawFd, FromRawFd, OwnedFd},
    sync::{
        atomic::{AtomicBool, Ordering},
        mpsc,
    },
    time::Duration,
};

struct Timer(OwnedFd);
impl Timer {
    fn new() -> Result<Self> {
        let fd = unsafe {
            libc::timerfd_create(
                libc::CLOCK_MONOTONIC,
                libc::TFD_CLOEXEC | libc::TFD_NONBLOCK,
            )
        };
        ensure!(fd >= 0, "timerfd_create: {}", io::Error::last_os_error());
        Ok(Self(unsafe { OwnedFd::from_raw_fd(fd) }))
    }
    fn arm(&self, first_ns: u64, interval_ns: u64) -> Result<()> {
        let timespec = |ns: u64| libc::timespec {
            tv_sec: (ns / 1_000_000_000) as _,
            tv_nsec: (ns % 1_000_000_000) as _,
        };
        let spec = libc::itimerspec {
            it_interval: timespec(interval_ns),
            it_value: timespec(first_ns),
        };
        ensure!(
            unsafe { libc::timerfd_settime(self.0.as_raw_fd(), 0, &spec, std::ptr::null_mut()) }
                == 0,
            "timerfd_settime: {}",
            io::Error::last_os_error()
        );
        Ok(())
    }
    fn drain(&self) -> Result<()> {
        let mut count = 0u64;
        let n = unsafe { libc::read(self.0.as_raw_fd(), (&mut count as *mut u64).cast(), 8) };
        ensure!(n == 8, "timerfd read: {}", io::Error::last_os_error());
        Ok(())
    }
}

fn read_resource<T: serde::Serialize>(
    values: &std::collections::BTreeMap<String, T>,
    id: Option<String>,
) -> Result<Value> {
    Ok(match id {
        None => serde_json::to_value(values)?,
        Some(id) => serde_json::to_value(values.get(&id).ok_or(NotFound(id))?)?,
    })
}

fn administer(simulation: &mut Simulation, request: Request) -> Result<()> {
    match request {
        Request::Inspect => (),
        Request::Reset
        | Request::Pause
        | Request::Unpause
        | Request::Advance { .. }
        | Request::Springs(_)
        | Request::PutSpring(_, _)
        | Request::DeleteSpring(_)
        | Request::Forces(_)
        | Request::PutForce(_, _)
        | Request::DeleteForce(_)
        | Request::Configuration
        | Request::Names => {
            unreachable!()
        }
        Request::Push { payload } => simulation.push(payload.torques)?,
        Request::Fault {
            payload: (name, fault),
        } => {
            if let Some(status) = fault.status {
                ensure!(
                    status.0 <= 15 && status != MotorStatus::ENABLED,
                    "fault status must be 0 or 2..15; enable motors through CAN"
                );
            }
            let motor = simulation.motor_mut(&name)?;
            if let Some(status) = fault.status {
                motor.status = status;
            }
            if let Some(silent) = fault.silent {
                motor.silent = silent;
            }
        }
    }
    Ok(())
}

fn snapshot(
    simulation: &Simulation,
    processed_time_ns: u64,
    paused: bool,
    advancing: bool,
    stats: &Statistics,
) -> State {
    State {
        state: simulation.snapshot(),
        statistics: *stats,
        time_ns: processed_time_ns,
        paused,
        advancing,
        mujoco_version: Physics::version(),
        timestep_ns: simulation.physics.timestep_ns,
        plant: simulation.physics.parameters(),
        bodies: simulation.physics.body_states(),
        sites: simulation.physics.site_states(),
        springs: simulation.physics.spring_states(),
    }
}

fn catch_up(simulation: &mut Simulation, clock: &Clock, stats: &mut Statistics) -> Result<u64> {
    let time_ns = u64::try_from(clock.elapsed()?.as_nanos()).context("clock overflow")?;
    if !clock.paused() {
        stats.max_lag_ns = stats
            .max_lag_ns
            .max(time_ns - stats.steps * simulation.physics.timestep_ns);
    }
    let count = time_ns / simulation.physics.timestep_ns - stats.steps;
    if count > 0 {
        simulation.step(count)?;
        stats.steps += count;
        stats.max_catchup_steps = stats.max_catchup_steps.max(count);
    }
    Ok(time_ns)
}

pub fn run(
    simulation: &mut Simulation,
    mut calls: Calls,
    sockets: Vec<CanFdSocket>,
    parent: Option<OwnedFd>,
    stopped: &AtomicBool,
) -> Result<()> {
    let timer = Timer::new()?;
    let mut clock = Clock::default();
    let timestep_ns = simulation.physics.timestep_ns;
    let mut processed_time_ns = 0;
    let mut advancing: Option<(u64, mpsc::Sender<Result<Reply>>)> = None;
    let mut stats = Statistics::default();
    let mut pollers = [
        timer.0.as_raw_fd(),
        parent.as_ref().map_or(-1, AsRawFd::as_raw_fd),
        calls.wake.as_raw_fd(),
    ]
    .into_iter()
    .chain(sockets.iter().map(AsRawFd::as_raw_fd))
    .map(|fd| libc::pollfd {
        fd,
        events: libc::POLLIN,
        revents: 0,
    })
    .collect::<Vec<_>>();
    while !stopped.load(Ordering::Relaxed) {
        let timeout = if advancing.is_some() { 0 } else { -1 };
        let ready = unsafe { libc::poll(pollers.as_mut_ptr(), pollers.len() as _, timeout) };
        if ready < 0 {
            if io::Error::last_os_error().kind() == io::ErrorKind::Interrupted {
                continue;
            }
            bail!("poll: {}", io::Error::last_os_error());
        }
        if pollers[1].revents & (libc::POLLIN | libc::POLLHUP) != 0 {
            let mut byte = 0u8;
            let count = unsafe { libc::read(pollers[1].fd, (&mut byte as *mut u8).cast(), 1) };
            if count == 0 {
                break;
            }
            ensure!(
                count == 1,
                "parent descriptor: {}",
                io::Error::last_os_error()
            );
        }
        if pollers[2].revents & libc::POLLIN != 0 {
            let mut bytes = [0; 256];
            loop {
                match calls.wake.read(&mut bytes) {
                    Ok(0) => bail!("administration wake socket closed"),
                    Ok(_) => (),
                    Err(e) if e.kind() == io::ErrorKind::WouldBlock => break,
                    Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
                    Err(e) => return Err(e.into()),
                }
            }
        }
        if stopped.load(Ordering::Relaxed) {
            break;
        }
        if pollers[0].revents & libc::POLLIN != 0 {
            timer.drain()?;
        }
        if let Some((deadline_ns, _)) = &advancing {
            // Service sockets between complete physics updates. This is not a
            // synchronization barrier with an external controller's clock.
            let next_time_ns = if deadline_ns / timestep_ns > stats.steps {
                (stats.steps + 1) * timestep_ns
            } else {
                *deadline_ns
            };
            clock.advance(Duration::from_nanos(next_time_ns - processed_time_ns))?;
        }
        processed_time_ns = catch_up(simulation, &clock, &mut stats)?;
        if advancing
            .as_ref()
            .is_some_and(|(deadline_ns, _)| processed_time_ns == *deadline_ns)
        {
            let (_, reply) = advancing.take().unwrap();
            let _ = reply.send(Ok(Reply::Done));
        }
        for (bus_index, bus) in sockets.iter().enumerate() {
            if pollers[bus_index + 3].revents & libc::POLLIN != 0 {
                can::receive(bus, bus_index, simulation, &mut stats)?;
            }
        }
        // The bounded channel limits work here; no timer is needed while paused.
        for _ in 0..32 {
            let Ok((request, reply)) = calls.receiver.try_recv() else {
                break;
            };
            if advancing.is_some()
                && matches!(
                    request,
                    Request::Reset | Request::Unpause | Request::Advance { .. }
                )
            {
                let _ = reply.send(Err(Conflict("advance already in progress").into()));
                continue;
            }
            let result = match request {
                Request::Reset => {
                    simulation.reset()?;
                    timer.arm(0, 0)?;
                    clock = Clock::default();
                    processed_time_ns = 0;
                    stats = Statistics::default();
                    Ok(Reply::Done)
                }
                Request::Pause => {
                    if clock.paused() {
                        Ok(Reply::Unchanged)
                    } else {
                        timer.arm(0, 0)?;
                        clock.pause()?;
                        processed_time_ns = catch_up(simulation, &clock, &mut stats)?;
                        Ok(Reply::Done)
                    }
                }
                Request::Unpause => {
                    if clock.paused() {
                        timer.arm(timestep_ns - processed_time_ns % timestep_ns, timestep_ns)?;
                        clock.unpause();
                        Ok(Reply::Done)
                    } else {
                        Ok(Reply::Unchanged)
                    }
                }
                Request::Advance { payload } => {
                    if !clock.paused() {
                        Err(Conflict("pause the clock before advancing").into())
                    } else {
                        match processed_time_ns
                            .checked_add(payload.duration_ns)
                            .context("clock overflow")
                        {
                            Ok(deadline_ns) => {
                                advancing = Some((deadline_ns, reply));
                                continue;
                            }
                            Err(error) => Err(error),
                        }
                    }
                }
                Request::Names => Ok(Reply::Names(simulation.physics.names())),
                Request::Configuration => Ok(Reply::Configuration(Box::new(Configuration {
                    configuration: simulation.physics.configuration(),
                }))),
                Request::Springs(id) => {
                    read_resource(simulation.physics.springs(), id).map(Reply::Value)
                }
                Request::Forces(id) => {
                    read_resource(simulation.physics.forces(), id).map(Reply::Value)
                }
                Request::PutSpring(id, value) => {
                    simulation.physics.put_spring(id, value).map(|created| {
                        if created {
                            Reply::Created
                        } else {
                            Reply::Unchanged
                        }
                    })
                }
                Request::PutForce(id, value) => {
                    simulation.physics.put_force(id, value).map(|created| {
                        if created {
                            Reply::Created
                        } else {
                            Reply::Unchanged
                        }
                    })
                }
                Request::DeleteSpring(id) => {
                    simulation.physics.delete_spring(&id);
                    Ok(Reply::Unchanged)
                }
                Request::DeleteForce(id) => {
                    simulation.physics.delete_force(&id);
                    Ok(Reply::Unchanged)
                }
                request => administer(simulation, request).map(|()| {
                    Reply::State(Box::new(snapshot(
                        simulation,
                        processed_time_ns,
                        clock.paused(),
                        advancing.is_some(),
                        &stats,
                    )))
                }),
            };
            let _ = reply.send(result);
        }
        ensure!(
            pollers
                .iter()
                .all(|p| p.revents & (libc::POLLERR | libc::POLLNVAL) == 0),
            "simulator descriptor failed"
        );
    }
    Ok(())
}
