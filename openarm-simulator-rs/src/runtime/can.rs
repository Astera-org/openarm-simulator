use crate::simulation::Simulation;
use anyhow::{Context, Result, ensure};
use damiao_can::REGISTER_CAN_ID;
use openarm_simulator_core::Statistics;
use serde_json::Value;
use socketcan::{
    CanFdFrame, CanFdSocket, CanFilter, CanSocket, EmbeddedFrame, Frame, Socket, SocketOptions,
    StandardId, id::FdFlags,
};
use std::{
    io,
    os::fd::{AsRawFd, FromRawFd, OwnedFd, RawFd},
    process::Command,
};

// Application fairness budget per socket visit; keeps clock/API work responsive.
// Not a CAN queue capacity or a controller protocol requirement.
const RECEIVE_BUDGET: usize = 64;

fn require_virtual_interfaces(interfaces: &[String]) -> Result<Vec<u32>> {
    // Check all interfaces before opening any socket, including direct
    // native invocation. Never trust OPENARM_SIMULATION as proof of isolation.
    ensure!(
        interfaces.iter().all(|s| !s.is_empty())
            && interfaces
                .iter()
                .collect::<std::collections::BTreeSet<_>>()
                .len()
                == interfaces.len(),
        "expected distinct virtual CAN interfaces"
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

pub(super) fn receive(
    bus: &CanFdSocket,
    bus_index: usize,
    simulation: &mut Simulation,
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
        let address = if id == REGISTER_CAN_ID && data.len() >= 2 {
            u16::from_le_bytes([data[0], data[1]]) as u32
        } else {
            id
        };
        let Some(index) =
            simulation
                .bindings
                .iter()
                .zip(&simulation.motors)
                .position(|(binding, motor)| {
                    binding.bus == bus_index && u32::from(motor.id()) == address
                })
        else {
            continue;
        };
        let motor = &mut simulation.motors[index];
        let Ok(reply) = motor.receive(id, data) else {
            continue;
        };
        if let Some(reply) = reply.filter(|_| !motor.silent) {
            let frame = CanFdFrame::with_flags(
                StandardId::new(motor.reply_id()).unwrap(),
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

pub fn can_sockets(
    interfaces: &[String],
    fds: &[Option<RawFd>],
    simulation: &Simulation,
) -> Result<Vec<CanFdSocket>> {
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
    interfaces
        .iter()
        .enumerate()
        .map(|(bus_index, name)| -> Result<_> {
            let filters: Vec<_> = std::iter::once(REGISTER_CAN_ID)
                .chain(
                    simulation
                        .bindings
                        .iter()
                        .zip(&simulation.motors)
                        .filter(|(binding, _)| binding.bus == bus_index)
                        .map(|(_, motor)| u32::from(motor.id())),
                )
                // EFF/RTR flags must be clear. CAN_ERR_FLAG is NOT part of a data filter.
                .map(|id| {
                    CanFilter::new(
                        id,
                        libc::CAN_EFF_FLAG | libc::CAN_RTR_FLAG | libc::CAN_SFF_MASK,
                    )
                })
                .collect();
            let bus = if let Some(fd) = fds[bus_index] {
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
                    index > 0 && index as u32 == indexes[bus_index],
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

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;
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
