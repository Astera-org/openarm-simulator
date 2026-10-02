use super::{Conflict, Control, NotFound, Reply, Request as AdminRequest, Unavailable};
use anyhow::{Result, ensure};
use axum::{
    Json, Router,
    extract::{DefaultBodyLimit, Path, Request, State},
    http::{
        HeaderMap, Method, StatusCode,
        header::{self, HeaderValue},
    },
    middleware::{self, Next},
    response::{IntoResponse, Response},
    routing::{get, post},
};
use openarm_simulator_core::ErrorResponse;
use serde_json::{Map, Value};
use std::{net::TcpListener, thread, time::Duration};
use tower_http::{
    cors::{Any, CorsLayer},
    set_header::SetResponseHeaderLayer,
    timeout::RequestBodyTimeoutLayer,
};

// Require application/json so cross-origin browser requests need preflight
// approval before executing commands. Simple requests, including empty POSTs,
// can otherwise execute even when CORS blocks reading the response. See:
// https://developer.mozilla.org/en-US/docs/Web/HTTP/Guides/CORS#simple_requests
fn require_json(headers: &HeaderMap) -> std::result::Result<(), StatusCode> {
    let media = headers
        .get(header::CONTENT_TYPE)
        .and_then(|value| value.to_str().ok())
        .and_then(|value| value.trim().parse::<mime::Mime>().ok());
    if headers.get_all(header::CONTENT_TYPE).iter().count() != 1
        || media.is_none_or(|media| media.essence_str() != "application/json")
    {
        return Err(StatusCode::UNSUPPORTED_MEDIA_TYPE);
    }
    Ok(())
}

async fn command(control: Control, args: Map<String, Value>, request: AdminRequest) -> Response {
    let request = (|| {
        ensure!(args.is_empty(), "command takes an empty JSON object");
        Ok(request)
    })();
    call(control, request).await
}

async fn call(control: Control, request: Result<AdminRequest>) -> Response {
    // Waiting for the physics owner must not block the network event loop.
    let result = tokio::task::spawn_blocking(move || control.call(request?))
        .await
        .unwrap_or_else(|error| Err(error.into()));
    match result {
        Ok(Reply::Names(value)) => Json(value).into_response(),
        Ok(Reply::State(value)) => Json(value).into_response(),
        Ok(Reply::Configuration(value)) => Json(value).into_response(),
        Ok(Reply::Done) => StatusCode::OK.into_response(),
        Ok(Reply::Unchanged) => StatusCode::NO_CONTENT.into_response(),
        Ok(Reply::Created) => StatusCode::CREATED.into_response(),
        Ok(Reply::Value(value)) => Json(value).into_response(),
        Err(error) => {
            let status = if error.is::<Unavailable>() {
                StatusCode::SERVICE_UNAVAILABLE
            } else if error.is::<Conflict>() {
                StatusCode::CONFLICT
            } else if error.is::<NotFound>() {
                StatusCode::NOT_FOUND
            } else {
                StatusCode::BAD_REQUEST
            };
            (
                status,
                Json(ErrorResponse {
                    error: error.to_string(),
                }),
            )
                .into_response()
        }
    }
}

pub fn start_http(
    listener: TcpListener,
    control: Control,
    allowed_origins: Vec<HeaderValue>,
) -> Result<()> {
    let cors = CorsLayer::new()
        .allow_methods([Method::GET, Method::POST, Method::PUT, Method::DELETE])
        .allow_headers([header::CONTENT_TYPE]);
    let cors = if allowed_origins.iter().any(|origin| origin == "*") {
        cors.allow_origin(Any)
    } else {
        cors.allow_origin(allowed_origins)
    };
    let app = Router::new()
        .route("/state", get(|State(c)| call(c, Ok(AdminRequest::Inspect))))
        .route(
            "/configuration",
            get(|State(c)| call(c, Ok(AdminRequest::Configuration))),
        )
        .route("/names", get(|State(c)| call(c, Ok(AdminRequest::Names))))
        .route(
            "/reset",
            post(|State(c), Json(args)| command(c, args, AdminRequest::Reset)),
        )
        .route(
            "/pause",
            post(|State(c), Json(args)| command(c, args, AdminRequest::Pause)),
        )
        .route(
            "/unpause",
            post(|State(c), Json(args)| command(c, args, AdminRequest::Unpause)),
        )
        .route(
            "/advance",
            post(|State(c), Json(args): Json<Map<String, Value>>| {
                call(
                    c,
                    serde_json::from_value(Value::Object(args))
                        .map(|payload| AdminRequest::Advance { payload })
                        .map_err(Into::into),
                )
            }),
        )
        .route(
            "/fault",
            post(
                |State(c), Json((name, args)): Json<(String, Map<String, Value>)>| {
                    call(
                        c,
                        serde_json::from_value(Value::Object(args))
                            .map(|fault| AdminRequest::Fault {
                                payload: (name, fault),
                            })
                            .map_err(Into::into),
                    )
                },
            ),
        )
        .route(
            "/push",
            post(|State(c), Json(payload)| call(c, Ok(AdminRequest::Push { payload }))),
        )
        .route(
            "/springs",
            get(|State(c)| call(c, Ok(AdminRequest::Springs(None)))),
        )
        .route(
            "/springs/{id}",
            get(|State(c), Path(id)| call(c, Ok(AdminRequest::Springs(Some(id)))))
                .put(|State(c), Path(id), Json(spring)| {
                    call(c, Ok(AdminRequest::PutSpring(id, spring)))
                })
                .delete(|State(c), Path(id), Json(args)| {
                    command(c, args, AdminRequest::DeleteSpring(id))
                }),
        )
        .route(
            "/forces",
            get(|State(c)| call(c, Ok(AdminRequest::Forces(None)))),
        )
        .route(
            "/forces/{id}",
            get(|State(c), Path(id)| call(c, Ok(AdminRequest::Forces(Some(id)))))
                .put(|State(c), Path(id), Json(force)| {
                    call(c, Ok(AdminRequest::PutForce(id, force)))
                })
                .delete(|State(c), Path(id), Json(args)| {
                    command(c, args, AdminRequest::DeleteForce(id))
                }),
        )
        .layer(DefaultBodyLimit::max(16384))
        .layer(RequestBodyTimeoutLayer::new(Duration::from_secs(2)))
        .layer(middleware::from_fn(
            |request: Request, next: Next| async move {
                require_json(request.headers())?;
                Ok::<_, StatusCode>(next.run(request).await)
            },
        ))
        .layer(cors)
        .layer(SetResponseHeaderLayer::overriding(
            header::CACHE_CONTROL,
            HeaderValue::from_static("no-store"),
        ))
        .with_state(control);
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
                if let Err(error) = axum::serve(listener, app).await {
                    eprintln!("HTTP server: {error}");
                }
            })
        })?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn require_json_content_type() {
        let rejected = Err(StatusCode::UNSUPPORTED_MEDIA_TYPE);
        for (values, expected) in [
            (vec!["application/json"], Ok(())),
            (vec!["Application/JSON; charset=utf-8"], Ok(())),
            (vec![], rejected),
            (vec!["text/plain"], rejected),
            (vec!["application/x-www-form-urlencoded"], rejected),
            (vec!["multipart/form-data; boundary=test"], rejected),
            (vec!["application/json", "application/json"], rejected),
        ] {
            let mut headers = HeaderMap::new();
            for value in &values {
                headers.append(header::CONTENT_TYPE, HeaderValue::from_static(value));
            }
            assert_eq!(require_json(&headers), expected, "{values:?}");
        }
    }
}
