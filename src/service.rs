//! Single-threaded physics/CAN owner, paced by a monotonic Linux timerfd.
use crate::physics::{Arms, Physics, Pose, SIDES, STEP};
use anyhow::{Context, Result, bail, ensure};
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use socketcan::id::FdFlags;
use socketcan::{
    CanFdFrame, CanFdSocket, CanFilter, EmbeddedFrame, Frame, Socket, SocketOptions, StandardId,
};
use std::{
    io,
    os::fd::{AsRawFd, FromRawFd, OwnedFd, RawFd},
    process::Command,
    time::Instant,
};

const INTERFACES: [&str; 2] = ["can0", "can1"];
const RECEIVE_BUDGET: usize = 64;

pub fn require_virtual_interfaces() -> Result<()> {
    // Check BOTH interfaces before opening either socket, including direct
    // native invocation. Never trust OPENARM_SIMULATION as proof of isolation.
    for name in INTERFACES {
        let result = Command::new("ip")
            .args(["-j", "-d", "link", "show", "dev", name])
            .output()?;
        ensure!(result.status.success(), "cannot inspect {name}");
        let links: Value = serde_json::from_slice(&result.stdout)?;
        ensure!(
            virtual_link(&links),
            "{name} must be CAN-FD capable vcan; refusing a physical interface"
        );
    }
    Ok(())
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

// Inherited Unix SEQPACKET socket: message boundaries, bounded/nonblocking I/O,
// and EOF when the launcher disappears. JSON is only for administrative calls.
pub struct Admin(OwnedFd);
impl Admin {
    /// The caller transfers its inherited descriptor exactly once.
    pub unsafe fn from_fd(fd: RawFd) -> Self {
        Self(unsafe { OwnedFd::from_raw_fd(fd) })
    }
    fn send(&self, value: Value) -> Result<()> {
        let bytes = serde_json::to_vec(&value)?;
        let n = unsafe {
            libc::send(
                self.0.as_raw_fd(),
                bytes.as_ptr().cast(),
                bytes.len(),
                libc::MSG_DONTWAIT | libc::MSG_NOSIGNAL,
            )
        };
        ensure!(
            n == bytes.len() as isize,
            "admin send: {}",
            io::Error::last_os_error()
        );
        Ok(())
    }
    fn read(&self) -> Result<Option<Request>> {
        let mut bytes = [0u8; 32768];
        let n = unsafe {
            libc::recv(
                self.0.as_raw_fd(),
                bytes.as_mut_ptr().cast(),
                bytes.len(),
                libc::MSG_DONTWAIT | libc::MSG_TRUNC,
            )
        };
        ensure!(
            n >= 0 && n as usize <= bytes.len(),
            "invalid admin packet: {}",
            io::Error::last_os_error()
        );
        if n == 0 {
            return Ok(None);
        }
        Ok(Some(serde_json::from_slice(&bytes[..n as usize])?))
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Fault {
    status: Option<u8>,
    silent: Option<bool>,
}
#[derive(Deserialize)]
#[serde(tag = "action", rename_all = "snake_case", deny_unknown_fields)]
enum Request {
    Inspect,
    Fault { payload: (String, usize, Fault) },
    Push { payload: Arms<[f64; 7]> },
    Reset { payload: Arms<Pose> },
    Quit,
}

fn administer(physics: &mut Physics, request: Request) -> Result<()> {
    match request {
        Request::Inspect | Request::Quit => (),
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

pub fn run(mut physics: Physics, admin: Admin) -> Result<()> {
    require_virtual_interfaces()?;
    let filters: Vec<_> = std::iter::once(0x7ff)
        .chain((0..4).flat_map(|mode| (1..=8).map(move |id| mode * 0x100 + id)))
        // EFF/RTR flags must be clear. CAN_ERR_FLAG is NOT part of a data filter.
        .map(|id| CanFilter::new(id, 0xc00007ff))
        .collect();
    let sockets: Vec<_> = INTERFACES
        .iter()
        .map(|name| -> Result<_> {
            let bus = CanFdSocket::open(name).with_context(|| format!("open virtual {name}"))?;
            bus.set_nonblocking(true)?;
            bus.set_filters(&filters)?;
            Ok(bus)
        })
        .collect::<Result<_>>()?;
    let timer = Timer::new()?;
    let started = Instant::now();
    let mut stats = Statistics::default();
    admin.send(json!({"ready": true, "mujoco_version": Physics::version(),
                      "configuration": physics.configuration()}))?;
    let mut pollers: Vec<_> = [
        timer.0.as_raw_fd(),
        admin.0.as_raw_fd(),
        sockets[0].as_raw_fd(),
        sockets[1].as_raw_fd(),
    ]
    .map(|fd| libc::pollfd {
        fd,
        events: libc::POLLIN,
        revents: 0,
    })
    .into();
    loop {
        let ready = unsafe { libc::poll(pollers.as_mut_ptr(), pollers.len() as _, -1) };
        if ready < 0 {
            if io::Error::last_os_error().kind() == io::ErrorKind::Interrupted {
                continue;
            }
            bail!("poll: {}", io::Error::last_os_error());
        }
        // Step BEFORE taking new commands so catch-up cannot apply a freshly
        // arrived command retroactively to all overdue physics steps.
        if pollers[0].revents & libc::POLLIN != 0 {
            let count = timer.ticks()?;
            stats.max_lag_ms = stats
                .max_lag_ms
                .max((started.elapsed().as_secs_f64() - stats.steps as f64 * STEP) * 1000.);
            physics.step(count)?;
            stats.steps += count;
            stats.max_catchup_steps = stats.max_catchup_steps.max(count);
        }
        if pollers[1].revents & (libc::POLLIN | libc::POLLHUP) != 0 {
            let result = match admin.read() {
                Ok(None | Some(Request::Quit)) => break,
                Ok(Some(request)) => administer(&mut physics, request),
                Err(error) => Err(error),
            };
            if let Err(error) = result {
                admin.send(json!({"error": error.to_string()}))?;
            } else {
                admin.send(json!({"state": physics.snapshot(), "statistics": stats, "time": physics.time(),
                              "mujoco_version": Physics::version(), "timestep_s": STEP,
                              "plant": physics.parameters(),
                              "joint_stop_solref": [0.002,1.], "joint_stop_solimp": [0.99,0.999,0.001,0.5,2.]}))?;
            }
        }
        for side in 0..2 {
            if pollers[side + 2].revents & libc::POLLIN != 0 {
                receive(&sockets[side], side, &mut physics, &mut stats)?;
            }
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
