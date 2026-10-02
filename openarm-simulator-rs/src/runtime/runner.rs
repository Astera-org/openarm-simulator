//! Single-threaded simulation/CAN owner. Pausing time never pauses socket I/O.
use super::{Calls, Change, Clock, Conflict, InvalidRequest, NotFound, Request, Responder, can};
use crate::{physics::Physics, simulation::Simulation};
use anyhow::{Context, Result, bail, ensure};
use damiao_can::MotorStatus;
use openarm_simulator_core::{Configuration, FaultRequest, State, Statistics};
use polling::{Event, Events, PollMode, Poller};
use socketcan::CanFdSocket;
use std::{
    fs::File,
    io::{self, Read},
    os::fd::{AsFd, OwnedFd},
    sync::atomic::{AtomicBool, Ordering},
    time::Duration,
};

enum ClockReply {
    Advance(Responder<()>),
    Pause(Responder<Change>),
}

fn set_fault(simulation: &mut Simulation, (name, fault): FaultRequest) -> Result<()> {
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
        // Return to the event loop between updates, including wall-time catch-up.
        simulation.step(1)?;
        stats.steps += 1;
        stats.max_catchup_steps = stats.max_catchup_steps.max(1);
    }
    Ok(if count > 1 {
        stats.steps * simulation.physics.timestep_ns
    } else {
        time_ns
    })
}

pub fn run(
    simulation: &mut Simulation,
    mut calls: Calls,
    sockets: Vec<CanFdSocket>,
    parent: Option<OwnedFd>,
    stopped: &AtomicBool,
) -> Result<()> {
    const WAKE: usize = 0;
    const PARENT: usize = 1;
    const CAN: usize = 2;

    let mut parent = parent.map(File::from);
    let poller = Poller::new()?;
    for (key, fd) in [(WAKE, calls.wake.as_fd())]
        .into_iter()
        .chain(parent.as_ref().map(|fd| (PARENT, fd.as_fd())))
        .chain(
            sockets
                .iter()
                .enumerate()
                .map(|(i, fd)| (CAN + i, fd.as_fd())),
        )
    {
        // SAFETY: All descriptors outlive this local poller, including on errors.
        // Level-triggering keeps unread CAN frames ready after a bounded visit.
        unsafe { poller.add_with_mode(&fd, Event::readable(key), PollMode::Level)? };
    }
    let mut events = Events::new();
    let mut clock = Clock::default();
    let timestep_ns = simulation.physics.timestep_ns;
    let mut advancing: Option<(u64, ClockReply)> = None;
    let mut stats = Statistics::default();
    while !stopped.load(Ordering::Relaxed) {
        let timeout = if advancing.is_some() {
            Some(Duration::ZERO)
        } else if clock.paused() {
            None
        } else {
            let next =
                Duration::from_nanos(stats.steps * timestep_ns) + Duration::from_nanos(timestep_ns);
            Some(next.saturating_sub(clock.elapsed()?))
        };
        events.clear();
        poller.wait(&mut events, timeout)?;
        let ready = |key| events.iter().any(|event| event.key == key);

        if let Some(parent) = parent.as_mut().filter(|_| ready(PARENT))
            && parent.read(&mut [0])? == 0
        {
            break;
        }
        if ready(WAKE) {
            match calls.wake.read(&mut [0; 32]) {
                Ok(0) => bail!("administration wake socket closed"),
                Ok(_) => (),
                Err(e)
                    if matches!(
                        e.kind(),
                        io::ErrorKind::WouldBlock | io::ErrorKind::Interrupted
                    ) => {}
                Err(e) => return Err(e.into()),
            }
        }
        if stopped.load(Ordering::Relaxed) {
            break;
        }
        let mut processed_time_ns = catch_up(simulation, &clock, &mut stats)?;
        if advancing
            .as_ref()
            .is_some_and(|(deadline_ns, _)| processed_time_ns == *deadline_ns)
        {
            let (_, reply) = advancing.take().unwrap();
            match reply {
                ClockReply::Advance(reply) => {
                    let _ = reply.send(Ok(()));
                }
                ClockReply::Pause(reply) => {
                    let _ = reply.send(Ok(Change::Changed));
                }
            }
        }
        for (bus_index, bus) in sockets.iter().enumerate() {
            if ready(CAN + bus_index) {
                can::receive(bus, bus_index, simulation, &mut stats)?;
            }
        }
        // The bounded channel limits work here; no timer is needed while paused.
        for _ in 0..32 {
            let Ok(request) = calls.receiver.try_recv() else {
                break;
            };
            let state = |simulation: &Simulation| {
                snapshot(
                    simulation,
                    processed_time_ns,
                    clock.paused(),
                    advancing.is_some(),
                    &stats,
                )
            };
            match request {
                Request::Reset(reply) | Request::Advance(_, reply) if advancing.is_some() => {
                    let _ = reply.send(Err(Conflict("advance already in progress").into()));
                }
                Request::Unpause(reply) if advancing.is_some() => {
                    let _ = reply.send(Err(Conflict("advance already in progress").into()));
                }
                Request::Reset(reply) => {
                    simulation.reset()?;
                    clock = Clock::default();
                    processed_time_ns = 0;
                    stats = Statistics::default();
                    let _ = reply.send(Ok(()));
                }
                Request::Pause(reply) => {
                    if clock.paused() {
                        let _ = reply.send(Ok(Change::Unchanged));
                    } else {
                        clock.pause()?;
                        let deadline_ns = u64::try_from(clock.elapsed()?.as_nanos())?;
                        advancing = Some((deadline_ns, ClockReply::Pause(reply)));
                    }
                }
                Request::Unpause(reply) => {
                    let change = if clock.paused() {
                        clock.unpause();
                        Change::Changed
                    } else {
                        Change::Unchanged
                    };
                    let _ = reply.send(Ok(change));
                }
                Request::Advance(payload, reply) => {
                    if !clock.paused() {
                        let _ =
                            reply.send(Err(Conflict("pause the clock before advancing").into()));
                    } else {
                        match processed_time_ns
                            .checked_add(payload.duration_ns)
                            .context("clock overflow")
                        {
                            Ok(deadline_ns) => {
                                clock.advance(Duration::from_nanos(payload.duration_ns))?;
                                advancing = Some((deadline_ns, ClockReply::Advance(reply)));
                            }
                            Err(error) => {
                                let _ = reply.send(Err(InvalidRequest(error).into()));
                            }
                        }
                    }
                }
                Request::Inspect(reply) => {
                    let _ = reply.send(Ok(state(simulation)));
                }
                Request::Names(reply) => {
                    let _ = reply.send(Ok(simulation.physics.names()));
                }
                Request::Configuration(reply) => {
                    let _ = reply.send(Ok(Configuration {
                        configuration: simulation.physics.configuration(),
                    }));
                }
                Request::Springs(reply) => {
                    let _ = reply.send(Ok(simulation.physics.springs().clone()));
                }
                Request::Spring(id, reply) => {
                    let value = simulation.physics.springs().get(&id).cloned();
                    let _ = reply.send(value.ok_or_else(|| NotFound(id).into()));
                }
                Request::Forces(reply) => {
                    let _ = reply.send(Ok(simulation.physics.forces().clone()));
                }
                Request::Force(id, reply) => {
                    let value = simulation.physics.forces().get(&id).cloned();
                    let _ = reply.send(value.ok_or_else(|| NotFound(id).into()));
                }
                Request::PutSpring(id, value, reply) => {
                    let result = simulation.physics.put_spring(id, value);
                    let _ = reply.send(result.map_err(|e| InvalidRequest(e).into()));
                }
                Request::PutForce(id, value, reply) => {
                    let result = simulation.physics.put_force(id, value);
                    let _ = reply.send(result.map_err(|e| InvalidRequest(e).into()));
                }
                Request::DeleteSpring(id, reply) => {
                    simulation.physics.delete_spring(&id);
                    let _ = reply.send(Ok(()));
                }
                Request::DeleteForce(id, reply) => {
                    simulation.physics.delete_force(&id);
                    let _ = reply.send(Ok(()));
                }
                Request::Fault(payload, reply) => {
                    let result = set_fault(simulation, payload)
                        .map_err(|e| InvalidRequest(e).into())
                        .map(|()| state(simulation));
                    let _ = reply.send(result);
                }
                Request::Push(payload, reply) => {
                    let result = simulation
                        .push(payload.torques)
                        .map_err(|e| InvalidRequest(e).into())
                        .map(|()| state(simulation));
                    let _ = reply.send(result);
                }
            }
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn catch_up_returns_to_event_processing_after_each_update() {
        let mut simulation = Simulation::new(
            mujoco::Model::from_xml_bytes(b"<mujoco/>").unwrap(),
            crate::config::Config::default(),
        )
        .unwrap();
        let period = simulation.physics.timestep_ns;
        let mut clock = Clock::default();
        clock.advance(Duration::from_nanos(3 * period + 7)).unwrap();
        let mut stats = Statistics::default();
        for step in 1..=3 {
            let time = catch_up(&mut simulation, &clock, &mut stats).unwrap();
            assert_eq!(stats.steps, step);
            assert_eq!(time, step * period + if step == 3 { 7 } else { 0 });
        }
        assert_eq!(
            catch_up(&mut simulation, &clock, &mut stats).unwrap(),
            3 * period + 7
        );
        assert_eq!(stats.steps, 3);
    }
}
