use anyhow::Result;
use openarm_simulator_core::{
    Advance, AppliedForce, Configuration, FaultRequest, PushRequest, SceneNames, Spring, State,
};
use serde_json::Value;
use std::{
    io::{self, Write},
    os::unix::net::UnixStream,
    sync::{
        Arc,
        mpsc::{self, Receiver, SyncSender},
    },
};
use tokio::sync::oneshot;

// Bounded messages cross into the physics thread; socket I/O never blocks it.
type Call = (Request, oneshot::Sender<Result<Reply>>);

pub enum Reply {
    Names(SceneNames),
    State(Box<State>),
    Configuration(Box<Configuration>),
    Done,
    Unchanged,
    Created,
    Value(Value),
}

#[derive(Debug, thiserror::Error)]
#[error("{0}")]
pub struct Conflict(pub &'static str);

#[derive(Debug, thiserror::Error)]
#[error("{0}")]
pub struct InvalidRequest(pub anyhow::Error);

pub struct Calls {
    pub(super) receiver: Receiver<Call>,
    pub(super) wake: UnixStream,
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
    pub async fn call(&self, request: Request) -> Result<Reply> {
        let (send, receive) = oneshot::channel();
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
            .await
            .map_err(|_| Unavailable("simulator stopped"))?
    }
}

#[derive(Debug, thiserror::Error)]
#[error("no such scene resource: {0}")]
pub struct NotFound(pub(super) String);

pub enum Request {
    Inspect,
    Fault { payload: FaultRequest },
    Push { payload: PushRequest },
    Reset,
    Pause,
    Unpause,
    Advance { payload: Advance },
    Configuration,
    Names,
    Springs(Option<String>),
    PutSpring(String, Spring),
    DeleteSpring(String),
    Forces(Option<String>),
    PutForce(String, AppliedForce),
    DeleteForce(String),
}
