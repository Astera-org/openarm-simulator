use anyhow::Result;
use openarm_simulator_core::{
    Advance, AppliedForce, BodyPoint, Configuration, FaultRequest, PushRequest, SceneNames, Spring,
    State,
};
use std::{
    collections::BTreeMap,
    io::{self, Write},
    os::unix::net::UnixStream,
    sync::{
        Arc,
        mpsc::{self, Receiver, SyncSender},
    },
};
use tokio::sync::oneshot;

pub type Responder<T> = oneshot::Sender<Result<T>>;

pub enum Change {
    Changed,
    Unchanged,
}

#[derive(Debug, thiserror::Error)]
#[error("{0}")]
pub struct Conflict(pub &'static str);

#[derive(Debug, thiserror::Error)]
#[error("{0}")]
pub struct InvalidRequest(pub anyhow::Error);

pub struct Calls {
    pub(super) receiver: Receiver<Request>,
    pub(super) wake: UnixStream,
}

#[derive(Debug, thiserror::Error)]
#[error("{0}")]
pub struct Unavailable(&'static str);

#[derive(Clone)]
pub struct Control {
    sender: SyncSender<Request>,
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
    pub async fn call<T>(&self, request: impl FnOnce(Responder<T>) -> Request) -> Result<T> {
        let (send, receive) = oneshot::channel();
        self.sender
            .try_send(request(send))
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
    Inspect(Responder<State>),
    Fault(FaultRequest, Responder<State>),
    Push(PushRequest, Responder<State>),
    Reset(Responder<()>),
    Pause(Responder<Change>),
    Unpause(Responder<Change>),
    Advance(Advance, Responder<()>),
    Configuration(Responder<Configuration>),
    Names(Responder<SceneNames>),
    Springs(Responder<BTreeMap<String, Spring<BodyPoint>>>),
    Spring(String, Responder<Spring<BodyPoint>>),
    PutSpring(String, Spring, Responder<bool>),
    DeleteSpring(String, Responder<()>),
    Forces(Responder<BTreeMap<String, AppliedForce<BodyPoint>>>),
    Force(String, Responder<AppliedForce<BodyPoint>>),
    PutForce(String, AppliedForce, Responder<bool>),
    DeleteForce(String, Responder<()>),
}
