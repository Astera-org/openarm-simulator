//! Single-threaded physics/CAN owner, paced by a monotonic Linux timerfd.
use crate::physics::{Arms, Physics, Pose, SIDES, STEP};
use anyhow::{Context, Result, bail, ensure};
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use socketcan::id::FdFlags;
use socketcan::{
    CanFdFrame, CanFdSocket, CanFilter, CanSocket, EmbeddedFrame, Frame, Socket, SocketOptions,
    StandardId,
};
use std::{
    io,
    os::fd::{AsRawFd, FromRawFd, OwnedFd, RawFd},
    process::Command,
    sync::{
        atomic::{AtomicBool, Ordering},
        mpsc::{self, Receiver, SyncSender},
    },
    time::{Duration, Instant},
};

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
        let timer = Self(unsafe { OwnedFd::from_raw_fd(fd) });
        let interval = libc::timespec {
            tv_sec: 0,
            tv_nsec: (STEP * 1e9) as _,
        };
        let spec = libc::itimerspec {
            it_interval: interval,
            it_value: interval,
        };
        ensure!(
            unsafe { libc::timerfd_settime(fd, 0, &spec, std::ptr::null_mut()) } == 0,
            "timerfd_settime: {}",
            io::Error::last_os_error()
        );
        Ok(timer)
    }
    fn ticks(&self) -> Result<u64> {
        let mut count = 0u64;
        let n = unsafe { libc::read(self.0.as_raw_fd(), (&mut count as *mut u64).cast(), 8) };
        ensure!(n == 8, "timerfd read: {}", io::Error::last_os_error());
        ensure!(
            count <= (1. / STEP) as u64,
            "Simulation fell more than one second behind wall time"
        );
        Ok(count)
    }
}

// Bounded messages cross into the physics thread; socket I/O never blocks it.
type Call = (Request, mpsc::Sender<Result<Value>>);

#[derive(Debug)]
pub struct Unavailable(&'static str);
impl std::fmt::Display for Unavailable {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter.write_str(self.0)
    }
}
impl std::error::Error for Unavailable {}

#[derive(Clone)]
pub struct Control(SyncSender<Call>);
impl Control {
    pub fn channel() -> (Self, Receiver<Call>) {
        let (sender, receiver) = mpsc::sync_channel(32);
        (Self(sender), receiver)
    }
    pub fn call(&self, message: Value) -> Result<Value> {
        if matches!(message["action"].as_str(), Some("push" | "reset")) {
            ensure!(
                message["payload"].is_object(),
                "payload must be an object keyed by arm"
            );
        }
        if message["action"] == "fault" {
            ensure!(
                message["payload"][2].is_object(),
                "fault settings must be an object"
            );
        }
        let request = serde_json::from_value(message)?;
        let (send, receive) = mpsc::channel();
        self.0
            .try_send((request, send))
            .map_err(|_| Unavailable("simulator busy or stopped"))?;
        receive
            .recv_timeout(Duration::from_secs(3))
            .map_err(|_| Unavailable("simulator stopped responding"))?
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Fault {
    status: Option<u8>,
    silent: Option<bool>,
}
#[derive(Deserialize)]
#[serde(tag = "action", rename_all = "snake_case", deny_unknown_fields)]
pub enum Request {
    Inspect,
    Fault { payload: (String, usize, Fault) },
    Push { payload: Arms<[f64; 7]> },
    Reset { payload: Arms<Pose> },
    Configuration,
}

fn administer(physics: &mut Physics, request: Request) -> Result<()> {
    match request {
        Request::Inspect | Request::Configuration => (),
        Request::Reset { payload } => physics.reset(payload)?,
        Request::Push { payload } => physics.push([
            payload.right.unwrap_or([0.; 7]),
            payload.left.unwrap_or([0.; 7]),
        ])?,
        Request::Fault {
            payload: (side, joint, fault),
        } => {
            let side = SIDES
                .iter()
                .position(|s| *s == side)
                .context("invalid fault arm")?;
            ensure!((1..=8).contains(&joint), "invalid fault joint");
            if let Some(status) = fault.status {
                ensure!(
                    status <= 15 && status != 1,
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

#[derive(Default, Serialize)]
struct Statistics {
    commands: u64,
    replies: u64,
    rejected: u64,
    dropped: u64,
    steps: u64,
    max_lag_ms: f64,
    max_catchup_steps: u64,
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
            Err(e) => return Err(e.into()),
        };
        stats.commands += 1;
        let (id, data) = (frame.raw_id(), frame.data());
        let joint = if id == 0x7ff && data.len() >= 2 {
            u16::from_le_bytes([data[0], data[1]]) as u32
        } else {
            id
        };
        if !(1..=8).contains(&joint) || data.len() != 8 {
            stats.rejected += 1;
            continue;
        }
        let motor = &mut physics.motors[side][joint as usize - 1];
        let reply = match motor.receive(id, data) {
            Ok(reply) => reply,
            Err(_) => {
                stats.rejected += 1;
                continue;
            }
        };
        if let Some(reply) = reply.filter(|_| !motor.silent) {
            let frame = CanFdFrame::with_flags(
                StandardId::new(joint as u16 + 16).unwrap(),
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
    let filters: Vec<_> = std::iter::once(0x7ff)
        .chain((0..4).flat_map(|mode| (1..=8).map(move |id| mode * 0x100 + id)))
        // EFF/RTR flags must be clear. CAN_ERR_FLAG is NOT part of a data filter.
        .map(|id| CanFilter::new(id, 0xc00007ff))
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

fn snapshot(physics: &Physics, stats: &Statistics, identity: &Value) -> Value {
    json!({"state": physics.snapshot(), "statistics": stats, "time": physics.time(),
        "mujoco_version": Physics::version(), "timestep_s": STEP,
        "plant": physics.parameters(), "config_sha256": identity["config_sha256"],
        "joint_stop_solref": [0.002,1.], "joint_stop_solimp": [0.99,0.999,0.001,0.5,2.]})
}

pub fn run(
    physics: &mut Physics,
    calls: Receiver<Call>,
    sockets: Vec<CanFdSocket>,
    parent: Option<OwnedFd>,
    stopped: &AtomicBool,
    identity: &Value,
) -> Result<Value> {
    let timer = Timer::new()?;
    let started = Instant::now();
    let mut stats = Statistics::default();
    let mut pollers = [
        timer.0.as_raw_fd(),
        sockets[0].as_raw_fd(),
        sockets[1].as_raw_fd(),
        parent.as_ref().map_or(-1, AsRawFd::as_raw_fd),
    ]
    .map(|fd| libc::pollfd {
        fd,
        events: libc::POLLIN,
        revents: 0,
    });
    while !stopped.load(Ordering::Relaxed) {
        let ready = unsafe { libc::poll(pollers.as_mut_ptr(), pollers.len() as _, -1) };
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
        // Catch up BEFORE new commands, so they cannot affect past steps.
        if pollers[0].revents & libc::POLLIN != 0 {
            let count = timer.ticks()?;
            stats.max_lag_ms = stats
                .max_lag_ms
                .max((started.elapsed().as_secs_f64() - stats.steps as f64 * STEP) * 1000.);
            physics.step(count)?;
            stats.steps += count;
            stats.max_catchup_steps = stats.max_catchup_steps.max(count);
        }
        for (side, bus) in sockets.iter().enumerate() {
            if pollers[side + 1].revents & libc::POLLIN != 0 {
                receive(bus, side, physics, &mut stats)?;
            }
        }
        // At most two administrative calls per tick, even under continuous load.
        for (request, reply) in calls.try_iter().take(2) {
            let result = if matches!(request, Request::Configuration) {
                Ok(identity.clone())
            } else {
                administer(physics, request).map(|()| snapshot(physics, &stats, identity))
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
    Ok(snapshot(physics, &stats, identity))
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
