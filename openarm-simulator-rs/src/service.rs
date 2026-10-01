//! Single-threaded physics/CAN owner. Pausing time never pauses socket I/O.
use crate::motor::V1_REPLY_ID_OFFSET;
use crate::{clock::Clock, physics::Physics};
use anyhow::{Context, Result, bail, ensure};
use openarm_can_rs::{MotorStatus, REGISTER_CAN_ID};
use openarm_simulator_core_rs::{
    Advance, Arm, Configuration, FaultRequest, Push, State, Statistics,
};
use serde_json::Value;
#[cfg(test)]
use serde_json::json;
use socketcan::id::FdFlags;
use socketcan::{
    CanFdFrame, CanFdSocket, CanFilter, CanSocket, EmbeddedFrame, Frame, Socket, SocketOptions,
    StandardId,
};
use std::{
    io::{self, Read, Write},
    os::fd::{AsRawFd, FromRawFd, OwnedFd, RawFd},
    os::unix::net::UnixStream,
    process::Command,
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
        mpsc::{self, Receiver, SyncSender},
    },
    time::Duration,
};

// Application fairness budget per socket visit; keeps clock/API work responsive.
// Not a CAN queue capacity or a controller protocol requirement.
const RECEIVE_BUDGET: usize = 64;

fn require_virtual_interfaces(interfaces: &[String; 2]) -> Result<Vec<u32>> {
    // Check BOTH interfaces before opening either socket, including direct
    // native invocation. Never trust OPENARM_SIMULATION as proof of isolation.
    ensure!(
        interfaces[0] != interfaces[1] && interfaces.iter().all(|s| !s.is_empty()),
        "expected two distinct virtual CAN interfaces"
    );
    let mut indexes = Vec::new();
    for name in interfaces {
        let result = Command::new("ip")
            .args(["-j", "-d", "link", "show", "dev", name])
            .output()?;
        ensure!(result.status.success(), "cannot inspect {name}");
        let links: Value = serde_json::from_slice(&result.stdout)?;
        ensure!(
            virtual_link(&links),
            "{name} must be CAN-FD capable vcan; refusing a physical interface"
        );
        indexes.push(u32::try_from(
            links[0]["ifindex"]
                .as_u64()
                .context("missing interface index")?,
        )?);
    }
    Ok(indexes)
}

fn netns_cookie(fd: RawFd) -> Result<u64> {
    let mut cookie = 0u64;
    let mut length = size_of::<u64>() as libc::socklen_t;
    ensure!(
        unsafe {
            libc::getsockopt(
                fd,
                libc::SOL_SOCKET,
                libc::SO_NETNS_COOKIE,
                (&mut cookie as *mut u64).cast(),
                &mut length,
            )
        } == 0,
        "cannot inspect socket network namespace: {}",
        io::Error::last_os_error()
    );
    Ok(cookie)
}

fn virtual_link(links: &Value) -> bool {
    links.as_array().is_some_and(|links| {
        links.len() == 1
            && links[0]["linkinfo"]["info_kind"] == "vcan"
            && links[0]["mtu"].as_u64().is_some_and(|mtu| mtu >= 72)
    })
}

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

// Bounded messages cross into the physics thread; socket I/O never blocks it.
type Call = (Request, mpsc::Sender<Result<Reply>>);

pub enum Reply {
    State(Box<State>),
    Configuration(Box<Configuration>),
    Done,
    Unchanged,
}

#[derive(Debug, thiserror::Error)]
#[error("{0}")]
pub struct Conflict(pub &'static str);

pub struct Calls {
    receiver: Receiver<Call>,
    wake: UnixStream,
}

#[derive(Debug, thiserror::Error)]
#[error("{0}")]
pub struct Unavailable(&'static str);

#[derive(Clone)]
pub struct Control {
    sender: SyncSender<Call>,
    wake: Arc<UnixStream>,
}
impl Control {
    pub fn channel() -> Result<(Self, Calls)> {
        let (sender, receiver) = mpsc::sync_channel(32);
        let (write, read) = UnixStream::pair()?;
        write.set_nonblocking(true)?;
        read.set_nonblocking(true)?;
        Ok((
            Self {
                sender,
                wake: Arc::new(write),
            },
            Calls {
                receiver,
                wake: read,
            },
        ))
    }
    pub fn wake_on_signal(&self, signal: i32) -> Result<()> {
        signal_hook::low_level::pipe::register(signal, self.wake.try_clone()?)?;
        Ok(())
    }
    pub fn call(&self, request: Request) -> Result<Reply> {
        let (send, receive) = mpsc::channel();
        self.sender
            .try_send((request, send))
            .map_err(|_| Unavailable("simulator busy or stopped"))?;
        if let Err(error) = (&*self.wake).write_all(&[1])
            && error.kind() != io::ErrorKind::WouldBlock
        {
            return Err(error.into());
        }
        // Do not time out accepted work here while the owner still executes it.
        // Network clients control their own wall-time timeout; no automatic retry.
        receive
            .recv()
            .map_err(|_| Unavailable("simulator stopped"))?
    }
}

pub enum Request {
    Inspect,
    Fault { payload: FaultRequest },
    Push { payload: Push },
    Reset,
    Pause,
    Unpause,
    Advance { payload: Advance },
    Configuration,
}

fn administer(physics: &mut Physics, request: Request) -> Result<()> {
    match request {
        Request::Inspect | Request::Configuration => (),
        Request::Reset | Request::Pause | Request::Unpause | Request::Advance { .. } => {
            unreachable!()
        }
        Request::Push { payload } => physics.push([
            payload.right.unwrap_or([0.; 7]),
            payload.left.unwrap_or([0.; 7]),
        ])?,
        Request::Fault {
            payload: (side, joint, fault),
        } => {
            let side = match side {
                Arm::Right => 0,
                Arm::Left => 1,
            };
            ensure!((1..=8).contains(&joint), "invalid fault joint");
            if let Some(status) = fault.status {
                ensure!(
                    status.0 <= 15 && status != MotorStatus::ENABLED,
                    "fault status must be 0 or 2..15; enable motors through CAN"
                );
            }
            let motor = &mut physics.motors[side][joint - 1];
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

fn receive(
    bus: &CanFdSocket,
    side: usize,
    physics: &mut Physics,
    stats: &mut Statistics,
) -> Result<()> {
    for _ in 0..RECEIVE_BUDGET {
        let frame = match bus.read_frame() {
            Ok(f) => f,
            Err(e) if e.kind() == io::ErrorKind::WouldBlock => break,
            Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
            Err(e) => return Err(e.into()),
        };
        stats.commands += 1;
        let (id, data) = (frame.raw_id(), frame.data());
        let joint = if id == REGISTER_CAN_ID && data.len() >= 2 {
            u16::from_le_bytes([data[0], data[1]]) as u32
        } else {
            id
        };
        if !(1..=8).contains(&joint) {
            continue;
        }
        let motor = &mut physics.motors[side][joint as usize - 1];
        let Ok(reply) = motor.receive(id, data) else {
            continue;
        };
        if let Some(reply) = reply.filter(|_| !motor.silent) {
            let frame = CanFdFrame::with_flags(
                StandardId::new(joint as u16 + V1_REPLY_ID_OFFSET).unwrap(),
                &reply,
                FdFlags::BRS,
            )
            .unwrap();
            match bus.write_frame(&frame) {
                Ok(()) => stats.replies += 1,
                Err(e)
                    if e.kind() == io::ErrorKind::WouldBlock
                        || e.raw_os_error() == Some(libc::ENOBUFS) =>
                {
                    stats.dropped += 1
                }
                Err(e) => return Err(e.into()),
            }
        }
    }
    Ok(())
}

pub fn can_sockets(interfaces: &[String; 2], fds: [Option<RawFd>; 2]) -> Result<Vec<CanFdSocket>> {
    let indexes = require_virtual_interfaces(interfaces)?;
    let current_namespace = if fds.iter().any(Option::is_some) {
        let probe =
            unsafe { libc::socket(libc::AF_INET, libc::SOCK_DGRAM | libc::SOCK_CLOEXEC, 0) };
        ensure!(probe >= 0, "socket: {}", io::Error::last_os_error());
        let probe = unsafe { OwnedFd::from_raw_fd(probe) };
        Some(netns_cookie(probe.as_raw_fd())?)
    } else {
        None
    };
    let filters: Vec<_> = std::iter::once(REGISTER_CAN_ID)
        .chain(1..=8)
        // EFF/RTR flags must be clear. CAN_ERR_FLAG is NOT part of a data filter.
        .map(|id| {
            CanFilter::new(
                id,
                libc::CAN_EFF_FLAG | libc::CAN_RTR_FLAG | libc::CAN_SFF_MASK,
            )
        })
        .collect();
    interfaces
        .iter()
        .enumerate()
        .map(|(side, name)| -> Result<_> {
            let bus = if let Some(fd) = fds[side] {
                let socket = CanSocket::from(unsafe { OwnedFd::from_raw_fd(fd) });
                let raw = socket.as_raw_socket();
                ensure!(
                    raw.domain()? == libc::AF_CAN.into() && raw.r#type()? == libc::SOCK_RAW.into(),
                    "expected a CAN_RAW socket for {name}"
                );
                // Interface indexes are namespace-local. Never validate a foreign
                // socket against an unrelated vcan with the same local index.
                ensure!(
                    Some(netns_cookie(socket.as_raw_fd())?) == current_namespace,
                    "inherited CAN sockets must belong to the simulator network namespace"
                );
                let address = raw.local_addr()?;
                ensure!(
                    address.len() as usize
                        >= std::mem::offset_of!(libc::sockaddr_can, can_ifindex)
                            + size_of::<libc::c_int>(),
                    "missing CAN socket address"
                );
                let index = unsafe { (*address.as_ptr().cast::<libc::sockaddr_can>()).can_ifindex };
                ensure!(
                    index > 0 && index as u32 == indexes[side],
                    "inherited CAN socket must be bound to {name}"
                );
                // Linux can report SO_PROTOCOL=0 for CAN_RAW. Enabling its
                // CAN_RAW_FD_FRAMES option also verifies the protocol here.
                CanFdSocket::try_from(socket)?
            } else {
                CanFdSocket::open(name).with_context(|| format!("open virtual {name}"))?
            };
            bus.set_nonblocking(true)?;
            bus.set_loopback(true)?;
            bus.set_recv_own_msgs(false)?;
            bus.set_join_filters(false)?;
            bus.set_error_filter_drop_all()?;
            bus.set_filters(&filters)?;
            Ok(bus)
        })
        .collect()
}

fn snapshot(
    physics: &Physics,
    processed_time_ns: u64,
    paused: bool,
    advancing: bool,
    stats: &Statistics,
) -> State {
    State {
        state: physics.snapshot(),
        statistics: *stats,
        time_ns: processed_time_ns,
        paused,
        advancing,
        mujoco_version: Physics::version(),
        timestep_ns: physics.timestep_ns,
        plant: physics.parameters(),
        joint_stop_solref: [0.002, 1.],
        joint_stop_solimp: [0.99, 0.999, 0.001, 0.5, 2.],
    }
}

fn catch_up(physics: &mut Physics, clock: &Clock, stats: &mut Statistics) -> Result<u64> {
    let time_ns = u64::try_from(clock.elapsed()?.as_nanos()).context("clock overflow")?;
    if !clock.paused() {
        stats.max_lag_ns = stats
            .max_lag_ns
            .max(time_ns - stats.steps * physics.timestep_ns);
    }
    let count = time_ns / physics.timestep_ns - stats.steps;
    if count > 0 {
        physics.step(count)?;
        stats.steps += count;
        stats.max_catchup_steps = stats.max_catchup_steps.max(count);
    }
    Ok(time_ns)
}

pub fn run(
    physics: &mut Physics,
    mut calls: Calls,
    sockets: Vec<CanFdSocket>,
    parent: Option<OwnedFd>,
    stopped: &AtomicBool,
) -> Result<State> {
    let timer = Timer::new()?;
    let mut clock = Clock::default();
    let timestep_ns = physics.timestep_ns;
    let mut processed_time_ns = 0;
    let mut advancing: Option<(u64, mpsc::Sender<Result<Reply>>)> = None;
    let mut stats = Statistics::default();
    let mut pollers = [
        timer.0.as_raw_fd(),
        sockets[0].as_raw_fd(),
        sockets[1].as_raw_fd(),
        parent.as_ref().map_or(-1, AsRawFd::as_raw_fd),
        calls.wake.as_raw_fd(),
    ]
    .map(|fd| libc::pollfd {
        fd,
        events: libc::POLLIN,
        revents: 0,
    });
    while !stopped.load(Ordering::Relaxed) {
        let timeout = if advancing.is_some() { 0 } else { -1 };
        let ready = unsafe { libc::poll(pollers.as_mut_ptr(), pollers.len() as _, timeout) };
        if ready < 0 {
            if io::Error::last_os_error().kind() == io::ErrorKind::Interrupted {
                continue;
            }
            bail!("poll: {}", io::Error::last_os_error());
        }
        if pollers[3].revents & (libc::POLLIN | libc::POLLHUP) != 0 {
            let mut byte = 0u8;
            let count = unsafe { libc::read(pollers[3].fd, (&mut byte as *mut u8).cast(), 1) };
            if count == 0 {
                break;
            }
            ensure!(
                count == 1,
                "parent descriptor: {}",
                io::Error::last_os_error()
            );
        }
        if pollers[4].revents & libc::POLLIN != 0 {
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
        processed_time_ns = catch_up(physics, &clock, &mut stats)?;
        if advancing
            .as_ref()
            .is_some_and(|(deadline_ns, _)| processed_time_ns == *deadline_ns)
        {
            let (_, reply) = advancing.take().unwrap();
            let _ = reply.send(Ok(Reply::Done));
        }
        for (side, bus) in sockets.iter().enumerate() {
            if pollers[side + 1].revents & libc::POLLIN != 0 {
                receive(bus, side, physics, &mut stats)?;
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
                    physics.reset()?;
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
                        processed_time_ns = catch_up(physics, &clock, &mut stats)?;
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
                Request::Configuration => Ok(Reply::Configuration(Box::new(Configuration {
                    configuration: physics.configuration(),
                }))),
                request => administer(physics, request).map(|()| {
                    Reply::State(Box::new(snapshot(
                        physics,
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
    Ok(snapshot(
        physics,
        processed_time_ns,
        clock.paused(),
        false,
        &stats,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn refuses_physical_classic_or_missing_interfaces() {
        assert!(virtual_link(
            &json!([{"mtu":72,"linkinfo":{"info_kind":"vcan"}}])
        ));
        for value in [
            json!([]),
            json!([{"mtu":72,"linkinfo":{"info_kind":"can"}}]),
            json!([{"mtu":16,"linkinfo":{"info_kind":"vcan"}}]),
            json!([{"mtu":72}]),
        ] {
            assert!(!virtual_link(&value));
        }
    }
}
