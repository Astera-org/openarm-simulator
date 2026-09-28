//! Concurrent HTTP administration; only the physics thread changes simulator state.
use crate::service::{Control, Unavailable};
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
    let action = match (request.method(), request.uri().path()) {
        (&Method::GET, "/state") => "inspect",
        (&Method::GET, "/configuration") => "configuration",
        (&Method::POST, "/fault") => "fault",
        (&Method::POST, "/push") => "push",
        (&Method::POST, "/reset") => "reset",
        _ => {
            return Ok((
                StatusCode::NOT_FOUND,
                json!({"error": "Unknown administration endpoint"}),
            ));
        }
    };
    let mut message = json!({"action": action});
    if request.method() == Method::POST {
        let media = headers
            .get(header::CONTENT_TYPE)
            .map(|v| v.to_str())
            .transpose()?
            .unwrap_or("");
        if !media
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
        ensure!(
            headers.get_all(header::CONTENT_LENGTH).iter().count() == 1
                && !headers.contains_key(header::TRANSFER_ENCODING),
            "Supply Content-Length; chunked requests are not supported"
        );
        let length: usize = headers[header::CONTENT_LENGTH].to_str()?.parse()?;
        if !(1..=16384).contains(&length) {
            return Ok((
                StatusCode::PAYLOAD_TOO_LARGE,
                json!({"error": "Request body must be 1..16384 bytes"}),
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
        message["payload"] = serde_json::from_slice(&body)?;
    }
    // Waiting for the physics owner must not block the network event loop.
    let value = tokio::task::spawn_blocking(move || control.call(message)).await??;
    Ok((StatusCode::OK, value))
}

async fn handle(
    request: Request<Incoming>,
    control: Control,
) -> Result<Response<Full<Bytes>>, Infallible> {
    let (status, value) = dispatch(request, control).await.unwrap_or_else(|error| {
        let status = if error.is::<Unavailable>() {
            StatusCode::SERVICE_UNAVAILABLE
        } else {
            StatusCode::BAD_REQUEST
        };
        (status, json!({"error": error.to_string()}))
    });
    let mut response = Response::builder()
        .status(status)
        .header(header::CONTENT_TYPE, "application/json")
        .header(header::CACHE_CONTROL, "no-store");
    // Errors may reject an unread body. Let Hyper close without reusing it as
    // the next request; successful requests retain normal HTTP/1.1 keep-alive.
    if !status.is_success() {
        response = response.header(header::CONNECTION, "close");
    }
    Ok(response.body(Full::new(value.to_string().into())).unwrap())
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
