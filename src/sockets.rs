use anyhow::{Context, Result, ensure};
use socket2::{Domain, Socket, Type};
use std::{
    collections::HashSet,
    env, io,
    net::TcpListener,
    os::fd::{FromRawFd, OwnedFd, RawFd},
};

pub fn activation_fd(explicit: Option<RawFd>) -> Result<Option<RawFd>> {
    if explicit.is_some() {
        return Ok(explicit);
    }
    let Some(count) = env::var_os("LISTEN_FDS") else {
        return Ok(None);
    };
    if let Ok(pid) = env::var("LISTEN_PID")
        && pid.parse::<u32>()? != std::process::id()
    {
        return Ok(None);
    }
    match count
        .to_str()
        .context("invalid LISTEN_FDS")?
        .parse::<u32>()?
    {
        0 => Ok(None),
        1 => Ok(Some(
            env::var("LISTEN_FDS_FIRST_FD")
                .unwrap_or_else(|_| "3".into())
                .parse()?,
        )),
        _ => anyhow::bail!("Expected exactly one inherited HTTP listener"),
    }
}

// Validate all numbers before opening anything that could reuse a closed fd.
pub fn validate(fds: &[Option<RawFd>]) -> Result<()> {
    let mut seen = HashSet::new();
    for fd in fds.iter().flatten() {
        ensure!(
            *fd >= 3 && seen.insert(*fd),
            "Inherited descriptors must be distinct and at least 3"
        );
        let flags = unsafe { libc::fcntl(*fd, libc::F_GETFD) };
        ensure!(
            flags >= 0,
            "inherited descriptor {fd}: {}",
            io::Error::last_os_error()
        );
        ensure!(
            unsafe { libc::fcntl(*fd, libc::F_SETFD, flags | libc::FD_CLOEXEC) } == 0,
            "cannot set close-on-exec: {}",
            io::Error::last_os_error()
        );
    }
    Ok(())
}

pub fn parent(fd: RawFd) -> Result<OwnedFd> {
    let mut stat = std::mem::MaybeUninit::<libc::stat>::uninit();
    ensure!(
        unsafe { libc::fstat(fd, stat.as_mut_ptr()) } == 0,
        "cannot inspect parent fd"
    );
    let mode = unsafe { stat.assume_init() }.st_mode & libc::S_IFMT;
    let flags = unsafe { libc::fcntl(fd, libc::F_GETFL) };
    ensure!(
        (mode == libc::S_IFIFO || mode == libc::S_IFSOCK)
            && flags & libc::O_ACCMODE != libc::O_WRONLY,
        "--parent-fd must be a readable pipe or socket"
    );
    Ok(unsafe { OwnedFd::from_raw_fd(fd) })
}

pub fn listener(fd: RawFd, domain: Domain, kind: Type) -> Result<Socket> {
    let socket = Socket::from(unsafe { OwnedFd::from_raw_fd(fd) });
    let actual = socket.domain()?;
    ensure!(
        (actual == domain || domain == Domain::IPV4 && actual == Domain::IPV6)
            && socket.r#type()? == kind
            && socket.is_listener()?,
        "inherited socket must be a listening {} socket",
        if domain == Domain::UNIX {
            "Unix SEQPACKET"
        } else {
            "TCP"
        }
    );
    socket.set_nonblocking(false)?;
    Ok(socket)
}

pub fn http(host: &str, port: u16, fd: Option<RawFd>) -> Result<TcpListener> {
    match fd {
        Some(fd) => Ok(listener(fd, Domain::IPV4, Type::STREAM)?.into()),
        None => Ok(TcpListener::bind((host, port))?),
    }
}
