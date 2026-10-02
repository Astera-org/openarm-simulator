use super::{Conflict, Control, InvalidRequest, NotFound, Reply, Request as Command, Unavailable};
use axum::{
    Json, Router,
    extract::{DefaultBodyLimit, FromRequest, Path, Request, State},
    http::{HeaderValue, Method, StatusCode, header},
    middleware::{self, Next},
    response::{IntoResponse, Response},
    routing::{get, post},
};
use openarm_simulator_core::{
    Advance, AppliedForce, ErrorResponse, FaultRequest, PushRequest, Spring,
};
use serde::de::IgnoredAny;
use std::{collections::BTreeMap, net::TcpListener, thread, time::Duration};
use tower::ServiceBuilder;
use tower_http::{
    cors::{Any, CorsLayer},
    set_header::SetResponseHeaderLayer,
    timeout::RequestBodyTimeoutLayer,
};

type Result = std::result::Result<Reply, ApiError>;

#[derive(Debug, thiserror::Error)]
#[error(transparent)]
struct ApiError(#[from] anyhow::Error);

impl IntoResponse for ApiError {
    fn into_response(self) -> Response {
        let status = if self.0.is::<InvalidRequest>() {
            StatusCode::BAD_REQUEST
        } else if self.0.is::<Conflict>() {
            StatusCode::CONFLICT
        } else if self.0.is::<NotFound>() {
            StatusCode::NOT_FOUND
        } else if self.0.is::<Unavailable>() {
            StatusCode::SERVICE_UNAVAILABLE
        } else {
            eprintln!("HTTP administration: {:#}", self.0);
            return StatusCode::INTERNAL_SERVER_ERROR.into_response();
        };
        (
            status,
            Json(ErrorResponse {
                error: self.to_string(),
            }),
        )
            .into_response()
    }
}

impl IntoResponse for Reply {
    fn into_response(self) -> Response {
        match self {
            Self::Names(value) => Json(value).into_response(),
            Self::State(value) => Json(value).into_response(),
            Self::Configuration(value) => Json(value).into_response(),
            Self::Value(value) => Json(value).into_response(),
            Self::Done => StatusCode::OK.into_response(),
            Self::Unchanged => StatusCode::NO_CONTENT.into_response(),
            Self::Created => StatusCode::CREATED.into_response(),
        }
    }
}

struct EmptyObject;
impl<S: Send + Sync> FromRequest<S> for EmptyObject {
    type Rejection = Response;

    async fn from_request(request: Request, state: &S) -> std::result::Result<Self, Response> {
        let Json(fields) = Json::<BTreeMap<String, IgnoredAny>>::from_request(request, state)
            .await
            .map_err(IntoResponse::into_response)?;
        if !fields.is_empty() {
            return Err((StatusCode::BAD_REQUEST, "expected an empty JSON object").into_response());
        }
        Ok(Self)
    }
}

// Require JSON so browsers must obtain preflight approval before executing
// commands. CORS alone only hides responses to simple requests, which can still
// have side effects. See https://developer.mozilla.org/en-US/docs/Web/HTTP/Guides/CORS#simple_requests
async fn require_json(request: Request, next: Next) -> std::result::Result<Response, StatusCode> {
    let headers = request.headers();
    let media = headers
        .get(header::CONTENT_TYPE)
        .and_then(|value| value.to_str().ok())
        .and_then(|value| value.parse::<mime::Mime>().ok());
    if headers.get_all(header::CONTENT_TYPE).iter().count() != 1
        || media.is_none_or(|media| media.essence_str() != "application/json")
    {
        return Err(StatusCode::UNSUPPORTED_MEDIA_TYPE);
    }
    Ok(next.run(request).await)
}

fn router(control: Control, origins: Vec<HeaderValue>) -> Router {
    let cors = CorsLayer::new()
        .allow_methods([Method::GET, Method::POST, Method::PUT, Method::DELETE])
        .allow_headers([header::CONTENT_TYPE]);
    let cors = if origins.iter().any(|origin| origin == "*") {
        cors.allow_origin(Any)
    } else {
        cors.allow_origin(origins)
    };
    Router::new()
        .route("/state", get(state))
        .route("/configuration", get(configuration))
        .route("/names", get(names))
        .route("/reset", post(reset))
        .route("/pause", post(pause))
        .route("/unpause", post(unpause))
        .route("/advance", post(advance))
        .route("/fault", post(fault))
        .route("/push", post(push))
        .route("/forces", get(forces))
        .route(
            "/forces/{id}",
            get(force).put(put_force).delete(delete_force),
        )
        .route("/springs", get(springs))
        .route(
            "/springs/{id}",
            get(spring).put(put_spring).delete(delete_spring),
        )
        .layer(
            ServiceBuilder::new()
                .layer(SetResponseHeaderLayer::overriding(
                    header::CACHE_CONTROL,
                    HeaderValue::from_static("no-store"),
                ))
                .layer(cors)
                .layer(middleware::from_fn(require_json))
                .layer(RequestBodyTimeoutLayer::new(Duration::from_secs(2)))
                .layer(DefaultBodyLimit::max(16 * 1024)),
        )
        .with_state(control)
}

async fn state(State(c): State<Control>) -> Result {
    Ok(c.call(Command::Inspect).await?)
}
async fn configuration(State(c): State<Control>) -> Result {
    Ok(c.call(Command::Configuration).await?)
}
async fn names(State(c): State<Control>) -> Result {
    Ok(c.call(Command::Names).await?)
}
async fn reset(State(c): State<Control>, _: EmptyObject) -> Result {
    Ok(c.call(Command::Reset).await?)
}
async fn pause(State(c): State<Control>, _: EmptyObject) -> Result {
    Ok(c.call(Command::Pause).await?)
}
async fn unpause(State(c): State<Control>, _: EmptyObject) -> Result {
    Ok(c.call(Command::Unpause).await?)
}
async fn advance(State(c): State<Control>, Json(payload): Json<Advance>) -> Result {
    Ok(c.call(Command::Advance { payload }).await?)
}
async fn fault(State(c): State<Control>, Json(payload): Json<FaultRequest>) -> Result {
    Ok(c.call(Command::Fault { payload }).await?)
}
async fn push(State(c): State<Control>, Json(payload): Json<PushRequest>) -> Result {
    Ok(c.call(Command::Push { payload }).await?)
}
async fn forces(State(c): State<Control>) -> Result {
    Ok(c.call(Command::Forces(None)).await?)
}
async fn force(State(c): State<Control>, Path(id): Path<String>) -> Result {
    Ok(c.call(Command::Forces(Some(id))).await?)
}
async fn put_force(
    State(c): State<Control>,
    Path(id): Path<String>,
    Json(value): Json<AppliedForce>,
) -> Result {
    Ok(c.call(Command::PutForce(id, value)).await?)
}
async fn delete_force(State(c): State<Control>, Path(id): Path<String>, _: EmptyObject) -> Result {
    Ok(c.call(Command::DeleteForce(id)).await?)
}
async fn springs(State(c): State<Control>) -> Result {
    Ok(c.call(Command::Springs(None)).await?)
}
async fn spring(State(c): State<Control>, Path(id): Path<String>) -> Result {
    Ok(c.call(Command::Springs(Some(id))).await?)
}
async fn put_spring(
    State(c): State<Control>,
    Path(id): Path<String>,
    Json(value): Json<Spring>,
) -> Result {
    Ok(c.call(Command::PutSpring(id, value)).await?)
}
async fn delete_spring(State(c): State<Control>, Path(id): Path<String>, _: EmptyObject) -> Result {
    Ok(c.call(Command::DeleteSpring(id)).await?)
}

pub fn start_http(
    listener: TcpListener,
    control: Control,
    origins: Vec<HeaderValue>,
) -> anyhow::Result<()> {
    let app = router(control, origins);
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
    use axum::body::Body;
    use tower::ServiceExt;

    #[test]
    fn request_policy_and_cors_configuration() {
        tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .unwrap()
            .block_on(async {
                let (control, calls) = Control::channel().unwrap();
                drop(calls);
                let app = router(control.clone(), vec![]);
                // A valid request reaches the unavailable owner; rejected inputs use
                // Axum's extraction responses or our two application-specific policies.
                let oversized = " ".repeat(16 * 1024 + 1);
                for (method, path, content_type, body, status) in [
                    ("GET", "/state", "", "", 415),
                    ("POST", "/reset", "text/plain", "{}", 415),
                    ("GET", "/state", "Application/JSON; charset=utf-8", "", 503),
                    ("POST", "/reset", "application/json", "{}", 503),
                    ("POST", "/reset", "application/json", "{", 400),
                    ("POST", "/reset", "application/json", "{\"extra\":1}", 400),
                    ("POST", "/reset", "application/json", "[]", 422),
                    (
                        "POST",
                        "/advance",
                        "application/json",
                        "{\"duration_ns\":-1}",
                        422,
                    ),
                    (
                        "POST",
                        "/advance",
                        "application/json",
                        oversized.as_str(),
                        413,
                    ),
                    ("GET", "/missing", "application/json", "", 404),
                    ("PUT", "/state", "application/json", "{}", 405),
                ] {
                    let mut request = Request::builder().method(method).uri(path);
                    if !content_type.is_empty() {
                        request = request.header(header::CONTENT_TYPE, content_type);
                    }
                    let response = app
                        .clone()
                        .oneshot(request.body(Body::from(body.to_owned())).unwrap())
                        .await
                        .unwrap();
                    assert_eq!(
                        response.status().as_u16(),
                        status,
                        "{method} {path} {content_type}"
                    );
                }

                let origin = "http://localhost:5173";
                for (allowed, expected) in [
                    (vec![], None),
                    (vec![origin], Some(origin)),
                    (vec!["*"], Some("*")),
                ] {
                    let app = router(
                        control.clone(),
                        allowed.into_iter().map(HeaderValue::from_static).collect(),
                    );
                    let request = Request::builder()
                        .method(Method::OPTIONS)
                        .uri("/reset")
                        .header(header::ORIGIN, origin)
                        .header(header::ACCESS_CONTROL_REQUEST_METHOD, "POST")
                        .header(header::ACCESS_CONTROL_REQUEST_HEADERS, "content-type")
                        .body(Body::empty())
                        .unwrap();
                    let response = app.clone().oneshot(request).await.unwrap();
                    assert!(response.status().is_success());
                    assert_eq!(
                        response
                            .headers()
                            .get(header::ACCESS_CONTROL_ALLOW_ORIGIN)
                            .map(|v| v.to_str().unwrap()),
                        expected
                    );
                    if expected.is_some() {
                        assert!(
                            response.headers()[header::ACCESS_CONTROL_ALLOW_METHODS]
                                .to_str()
                                .unwrap()
                                .contains("POST")
                        );
                        assert_eq!(
                            response.headers()[header::ACCESS_CONTROL_ALLOW_HEADERS],
                            "content-type"
                        );
                    }
                    let response = app
                        .oneshot(
                            Request::builder()
                                .uri("/state")
                                .header(header::ORIGIN, origin)
                                .body(Body::empty())
                                .unwrap(),
                        )
                        .await
                        .unwrap();
                    assert_eq!(response.status(), StatusCode::UNSUPPORTED_MEDIA_TYPE);
                    assert_eq!(
                        response
                            .headers()
                            .get(header::ACCESS_CONTROL_ALLOW_ORIGIN)
                            .map(|v| v.to_str().unwrap()),
                        expected
                    );
                }
            });
    }
}
