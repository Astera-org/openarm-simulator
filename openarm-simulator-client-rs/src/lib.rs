//! Asynchronous HTTP administration client for an existing simulator. Run on a
//! Tokio runtime with I/O and time enabled. No process launching or CAN control.
//! Requests are never automatically retried; redirects are not followed.
//! A timeout does not cancel work
//! already accepted by the simulator.
use http_body_util::{BodyExt, Full};
use hyper::{Method, Request, Uri, body::Bytes, header};
use hyper_util::{
    client::legacy::{Client as HttpClient, connect::HttpConnector},
    rt::{TokioExecutor, TokioTimer},
};
use serde::de::DeserializeOwned;
use std::{error::Error as StdError, time::Duration};

pub use hyper::StatusCode;
use models::{Advance, Configuration, ErrorResponse, Fault, Push, State};
pub use openarm_simulator_core_rs as models;

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("simulator HTTP {status}: {message}")]
    Api { status: StatusCode, message: String },
    #[error("simulator request timed out; accepted work may still execute")]
    Timeout,
    #[error("{0}")]
    InvalidRequest(String),
    #[error("simulator transport: {0}")]
    Transport(#[source] Box<dyn StdError + Send + Sync>),
    #[error("simulator JSON: {0}")]
    Json(#[from] serde_json::Error),
}
pub type Result<T> = std::result::Result<T, Error>;

#[derive(Clone)]
pub struct Client {
    url: String,
    http: HttpClient<HttpConnector, Full<Bytes>>,
    timeout: Option<Duration>,
}
impl Client {
    /// Connect over HTTP, as served by the simulator. Optional URL path prefixes
    /// are retained. Credentials, query strings and fragments are rejected.
    pub fn new(url: &str) -> Result<Self> {
        let uri: Uri = url
            .parse()
            .map_err(|e| Error::InvalidRequest(format!("invalid simulator URL: {e}")))?;
        if uri.scheme_str() != Some("http")
            || uri.host().is_none_or(|host| host.is_empty())
            || uri.query().is_some()
            || url.contains('#')
            || uri.authority().is_some_and(|a| a.as_str().contains('@'))
        {
            return Err(Error::InvalidRequest(
                "expected an HTTP simulator URL without credentials, query or fragment".into(),
            ));
        }
        let http = HttpClient::builder(TokioExecutor::new())
            .pool_timer(TokioTimer::new())
            .retry_canceled_requests(false)
            .build_http();
        Ok(Self {
            url: url.trim_end_matches('/').into(),
            http,
            timeout: Some(Duration::from_secs(5)),
        })
    }
    /// Wall-clock deadline covering connection, response headers and body. None
    /// waits indefinitely, useful for long advances. Defaults to five seconds.
    pub fn with_timeout(mut self, timeout: Option<Duration>) -> Self {
        self.timeout = timeout;
        self
    }
    async fn request(
        &self,
        method: Method,
        path: &str,
        body: Vec<u8>,
    ) -> Result<(StatusCode, Bytes)> {
        let request = Request::builder()
            .method(method)
            .uri(format!("{}{path}", self.url))
            .header(header::ACCEPT, "application/json")
            .header(header::CONTENT_TYPE, "application/json")
            .header(header::CONTENT_LENGTH, body.len())
            .body(Full::new(Bytes::from(body)))
            .map_err(|e| Error::InvalidRequest(e.to_string()))?;
        let perform = async {
            let response = self
                .http
                .request(request)
                .await
                .map_err(|e| Error::Transport(Box::new(e)))?;
            let status = response.status();
            let bytes = response
                .into_body()
                .collect()
                .await
                .map_err(|e| Error::Transport(Box::new(e)))?
                .to_bytes();
            if !status.is_success() {
                let message = serde_json::from_slice::<ErrorResponse>(&bytes)
                    .map(|e| e.error)
                    .unwrap_or_else(|_| String::from_utf8_lossy(&bytes).into_owned());
                return Err(Error::Api { status, message });
            }
            Ok((status, bytes))
        };
        match self.timeout {
            Some(duration) => tokio::time::timeout(duration, perform)
                .await
                .map_err(|_| Error::Timeout)?,
            None => perform.await,
        }
    }
    async fn read<T: DeserializeOwned>(
        &self,
        method: Method,
        path: &str,
        body: Vec<u8>,
    ) -> Result<T> {
        let (_, bytes) = self.request(method, path, body).await?;
        Ok(serde_json::from_slice(&bytes)?)
    }
    async fn clock(&self, path: &str, body: Vec<u8>) -> Result<StatusCode> {
        let (status, bytes) = self.request(Method::POST, path, body).await?;
        let unchanged = matches!(path, "/pause" | "/unpause") && status == StatusCode::NO_CONTENT;
        if !(status == StatusCode::OK || unchanged) || !bytes.is_empty() {
            return Err(Error::Api { status, message: "unexpected clock response (expected empty 200, or 204 for unchanged pause/unpause)".into() });
        }
        Ok(status)
    }
    pub async fn state(&self) -> Result<State> {
        self.read(Method::GET, "/state", Vec::new()).await
    }
    pub async fn configuration(&self) -> Result<Configuration> {
        self.read(Method::GET, "/configuration", Vec::new()).await
    }
    /// Restore startup state and pause the clock.
    pub async fn reset(&self) -> Result<StatusCode> {
        self.clock("/reset", Vec::new()).await
    }
    /// Returns 200 when changed, 204 when already paused.
    pub async fn pause(&self) -> Result<StatusCode> {
        self.clock("/pause", Vec::new()).await
    }
    /// Returns 200 when changed, 204 when already unpaused.
    pub async fn unpause(&self) -> Result<StatusCode> {
        self.clock("/unpause", Vec::new()).await
    }
    /// Advance the paused clock. Uses integer nanoseconds without partial physics
    /// updates. Coordinate CAN exchanges separately; this does not drain consumers.
    pub async fn advance(&self, duration: Duration) -> Result<StatusCode> {
        let duration_ns = u64::try_from(duration.as_nanos())
            .map_err(|_| Error::InvalidRequest("advance exceeds u64 nanoseconds".into()))?;
        self.clock("/advance", serde_json::to_vec(&Advance { duration_ns })?)
            .await
    }
    pub async fn fault(&self, motor: &str, settings: Fault) -> Result<State> {
        self.read(
            Method::POST,
            "/fault",
            serde_json::to_vec(&(motor, settings))?,
        )
        .await
    }
    pub async fn push(&self, torques: Push) -> Result<State> {
        if !torques.values().all(|v| v.is_finite()) {
            return Err(Error::InvalidRequest(
                "applied torques must be finite".into(),
            ));
        }
        self.read(Method::POST, "/push", serde_json::to_vec(&torques)?)
            .await
    }
}
