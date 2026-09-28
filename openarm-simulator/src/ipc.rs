//! Optional Lab SEQPACKET adapter, using the same administration channel as HTTP.
use crate::{service::Control, sockets};
use anyhow::{Result, ensure};
use serde_json::{Value, json};
use socket2::{Domain, SockAddr, Socket, Type};
use std::{
    fs,
    io::{Read, Write},
    os::{fd::RawFd, unix::fs::PermissionsExt},
    path::{Path, PathBuf},
    time::Duration,
};

pub struct Listener {
    pub socket: Socket,
    owned_path: Option<PathBuf>,
}
impl Drop for Listener {
    fn drop(&mut self) {
        if let Some(path) = &self.owned_path {
            let _ = fs::remove_file(path);
        }
    }
}

pub fn listen(runtime: Option<&Path>, fd: Option<RawFd>) -> Result<Option<Listener>> {
    if let Some(fd) = fd {
        return Ok(Some(Listener {
            socket: sockets::listener(fd, Domain::UNIX, Type::SEQPACKET)?,
            owned_path: None,
        }));
    }
    let Some(runtime) = runtime else {
        return Ok(None);
    };
    let path = runtime.join("simulator.sock");
    if path.exists() {
        fs::remove_file(&path)?;
    }
    let socket = Socket::new(Domain::UNIX, Type::SEQPACKET, None)?;
    socket.bind(&SockAddr::unix(&path)?)?;
    let listener = Listener {
        socket,
        owned_path: Some(path.clone()),
    };
    fs::set_permissions(path, fs::Permissions::from_mode(0o600))?;
    listener.socket.listen(16)?;
    Ok(Some(listener))
}

fn read(socket: &Socket) -> Result<Value> {
    let mut bytes = [0u8; 32769];
    let count = (&*socket).read(&mut bytes)?;
    ensure!(count > 0 && count < bytes.len(), "invalid IPC packet size");
    Ok(serde_json::from_slice(&bytes[..count])?)
}

fn write(socket: &Socket, value: &Value) -> Result<()> {
    let bytes = serde_json::to_vec(value)?;
    ensure!(socket.send(&bytes)? == bytes.len(), "incomplete IPC reply");
    Ok(())
}

pub fn register(
    runtime: &Path,
    listener: &Listener,
    interfaces: &[String; 2],
    identity: &Value,
) -> Result<()> {
    write_json(&runtime.join("simulator-physics.json"), identity)?;
    let socket = Socket::new(Domain::UNIX, Type::SEQPACKET, None)?;
    socket.set_read_timeout(Some(Duration::from_secs(5)))?;
    socket.set_write_timeout(Some(Duration::from_secs(5)))?;
    socket.connect(&SockAddr::unix(runtime.join("manager.sock"))?)?;
    write(
        &socket,
        &json!({"op": "register", "name": "simulator",
        "instance": std::env::var("HQ_SERVICE_INSTANCE")?,
        "endpoint": {"socket": listener.socket.local_addr()?.as_pathname(),
            "transport": "socketcan", "interfaces": interfaces, "config_sha256": identity["config_sha256"]}}),
    )?;
    let reply = read(&socket)?;
    ensure!(
        reply.get("error").is_none(),
        "Lab registration failed: {reply}"
    );
    Ok(())
}

pub fn run(socket: Socket, control: Control) {
    // Each client gets one bounded request/reply, matching the Lab protocol.
    while let Ok((client, _)) = socket.accept() {
        let result = (|| -> Result<Value> {
            client.set_read_timeout(Some(Duration::from_secs(2)))?;
            client.set_write_timeout(Some(Duration::from_secs(2)))?;
            let mut message = read(&client)?;
            ensure!(message.is_object(), "expected a message object");
            if message.get("action").is_none() {
                message["action"] = "inspect".into();
            }
            ensure!(
                matches!(
                    message["action"].as_str(),
                    Some("inspect" | "fault" | "push")
                ),
                "Unsupported Lab administration action"
            );
            control.call(message)
        })();
        let reply = result.unwrap_or_else(|e| json!({"error": e.to_string()}));
        let _ = write(&client, &reply);
    }
}

pub fn write_json(path: &Path, value: &Value) -> Result<()> {
    let temporary = path.with_extension("tmp");
    let mut file = fs::File::create(&temporary)?;
    serde_json::to_writer_pretty(&mut file, value)?;
    file.write_all(b"\n")?;
    file.sync_all()?;
    fs::rename(temporary, path)?;
    fs::File::open(path.parent().unwrap())?.sync_all()?;
    Ok(())
}
