use crate::service::{Conflict, Control, Reply, Request as AdminRequest, Unavailable};
use anyhow::{Result, ensure};
use http_body_util::{BodyExt, Full, Limited};
use hyper::{
    Method, Request, Response, StatusCode,
    body::{Bytes, Incoming},
    header,
    server::conn::http1,
    service::service_fn,
};
use hyper_util::rt::{TokioIo, TokioTimer};
use openarm_simulator_core_rs::ErrorResponse;
use serde_json::{Value, json};
use std::{convert::Infallible, net::TcpListener, thread, time::Duration};
use tokio::time::timeout;

async fn dispatch(request: Request<Incoming>, control: Control) -> Result<(StatusCode, Value)> {
    let headers = request.headers();
    if headers.contains_key(header::ORIGIN) {
        return Ok((
            StatusCode::FORBIDDEN,
            json!({"error": "Browser-origin requests are not enabled"}),
        ));
    }
    if headers.contains_key(header::EXPECT) {
        return Ok((
            StatusCode::EXPECTATION_FAILED,
            json!({"error": "Expect is not supported"}),
        ));
    }
    let path = request.uri().path().to_owned();
    let no_args = match (request.method(), path.as_str()) {
        (&Method::GET, "/state" | "/configuration")
        | (&Method::POST, "/reset" | "/pause" | "/unpause") => true,
        (&Method::POST, "/advance" | "/fault" | "/push") => false,
        _ => {
            return Ok((
                StatusCode::NOT_FOUND,
                json!({"error": "Unknown administration endpoint"}),
            ));
        }
    };
    let mut payload = Value::Null;
    if request.method() == Method::POST {
        ensure!(
            headers.get_all(header::CONTENT_LENGTH).iter().count() <= 1
                && !headers.contains_key(header::TRANSFER_ENCODING),
            "Supply Content-Length; chunked requests are not supported"
        );
        let length: usize = headers
            .get(header::CONTENT_LENGTH)
            .map(|v| v.to_str())
            .transpose()?
            .unwrap_or("0")
            .parse()?;
        if length > 16384 {
            return Ok((
                StatusCode::PAYLOAD_TOO_LARGE,
                json!({"error": "Request body exceeds 16384 bytes"}),
            ));
        }
        ensure!(no_args || length > 0, "request requires a JSON body");
        let media = headers
            .get(header::CONTENT_TYPE)
            .map(|v| v.to_str())
            .transpose()?
            .unwrap_or("");
        if length > 0
            && !media
                .split(';')
                .next()
                .unwrap()
                .trim()
                .eq_ignore_ascii_case("application/json")
        {
            return Ok((
                StatusCode::UNSUPPORTED_MEDIA_TYPE,
                json!({"error": "Use Content-Type: application/json"}),
            ));
        }
        let body = match timeout(
            Duration::from_secs(2),
            Limited::new(request.into_body(), 16384).collect(),
        )
        .await
        {
            Ok(Ok(body)) => body.to_bytes(),
            Ok(Err(error)) => anyhow::bail!("invalid request body: {error}"),
            Err(_) => {
                return Ok((
                    StatusCode::REQUEST_TIMEOUT,
                    json!({"error": "Request body timed out"}),
                ));
            }
        };
        ensure!(body.len() == length, "incomplete request body");
        if no_args {
            ensure!(
                body.is_empty() || serde_json::from_slice::<Value>(&body)? == json!({}),
                "{path} takes no arguments"
            );
        } else {
            payload = serde_json::from_slice(&body)?;
        }
    }
    let message = match path.as_str() {
        "/state" => AdminRequest::Inspect,
        "/configuration" => AdminRequest::Configuration,
        "/reset" => AdminRequest::Reset,
        "/pause" => AdminRequest::Pause,
        "/unpause" => AdminRequest::Unpause,
        "/advance" => {
            ensure!(payload.is_object(), "advance payload must be an object");
            AdminRequest::Advance {
                payload: serde_json::from_value(payload)?,
            }
        }
        "/push" => {
            ensure!(
                payload.is_object(),
                "payload must be an object keyed by arm"
            );
            AdminRequest::Push {
                payload: serde_json::from_value(payload)?,
            }
        }
        "/fault" => {
            ensure!(payload[1].is_object(), "fault settings must be an object");
            AdminRequest::Fault {
                payload: serde_json::from_value(payload)?,
            }
        }
        _ => unreachable!("route validated above"),
    };
    // Waiting for the physics owner must not block the network event loop.
    Ok(
        match tokio::task::spawn_blocking(move || control.call(message)).await?? {
            Reply::State(value) => (StatusCode::OK, serde_json::to_value(value)?),
            Reply::Configuration(value) => (StatusCode::OK, serde_json::to_value(value)?),
            Reply::Done => (StatusCode::OK, Value::Null),
            Reply::Unchanged => (StatusCode::NO_CONTENT, Value::Null),
        },
    )
}

async fn handle(
    request: Request<Incoming>,
    control: Control,
) -> Result<Response<Full<Bytes>>, Infallible> {
    let (status, value) = dispatch(request, control).await.unwrap_or_else(|error| {
        let status = if error.is::<Unavailable>() {
            StatusCode::SERVICE_UNAVAILABLE
        } else if error.is::<Conflict>() {
            StatusCode::CONFLICT
        } else {
            StatusCode::BAD_REQUEST
        };
        (
            status,
            serde_json::to_value(ErrorResponse {
                error: error.to_string(),
            })
            .unwrap(),
        )
    });
    let mut response = Response::builder()
        .status(status)
        .header(header::CACHE_CONTROL, "no-store");
    // Errors may reject an unread body. Let Hyper close without reusing it as
    // the next request; successful requests retain normal HTTP/1.1 keep-alive.
    if !status.is_success() {
        response = response.header(header::CONNECTION, "close");
    }
    let body = if value.is_null() {
        Bytes::new()
    } else {
        response = response.header(header::CONTENT_TYPE, "application/json");
        value.to_string().into()
    };
    Ok(response.body(Full::new(body)).unwrap())
}

pub fn start(listener: TcpListener, control: Control) -> Result<()> {
    let runtime = tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()?;
    listener.set_nonblocking(true)?;
    let listener = {
        let _entered = runtime.enter();
        tokio::net::TcpListener::from_std(listener)?
    };
    thread::Builder::new()
        .name("http-admin".into())
        .spawn(move || {
            runtime.block_on(async move {
                loop {
                    let stream = match listener.accept().await {
                        Ok((stream, _)) => stream,
                        Err(error) if error.kind() == std::io::ErrorKind::Interrupted => continue,
                        Err(error) => {
                            eprintln!("HTTP accept: {error}");
                            return;
                        }
                    };
                    let control = control.clone();
                    tokio::spawn(async move {
                        let _ = http1::Builder::new()
                            .keep_alive(true)
                            .timer(TokioTimer::new())
                            .header_read_timeout(Duration::from_secs(2))
                            .max_headers(32)
                            .max_buf_size(8192)
                            .serve_connection(
                                TokioIo::new(stream),
                                service_fn(move |request| handle(request, control.clone())),
                            )
                            .await;
                    });
                }
            })
        })?;
    Ok(())
}
