use openarm_simulator_client::{Client, Error, StatusCode};
use std::{
    io::{BufRead, BufReader, Write},
    net::TcpListener,
    sync::mpsc,
    thread,
    time::Duration,
};

#[test]
fn errors_timeouts_and_no_retries() {
    let runtime = tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
        .unwrap();
    for url in [
        "not a URL",
        "https://localhost",
        "http://user@localhost",
        "http://localhost/?q=1",
        "http://localhost/#fragment",
    ] {
        assert!(Client::new(url).is_err());
    }
    // Validate the request on the wire, then exercise error decoding, malformed
    // JSON, a connection closing without a reply, and a stalled response body.
    for scenario in 0..4 {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let url = format!("http://{}/prefix/", listener.local_addr().unwrap());
        let (release, wait) = mpsc::channel();
        let server = thread::spawn(move || {
            let (stream, _) = listener.accept().unwrap();
            stream
                .set_read_timeout(Some(Duration::from_secs(5)))
                .unwrap();
            let mut reader = BufReader::new(stream);
            let mut request = String::new();
            loop {
                let mut line = String::new();
                assert!(reader.read_line(&mut line).unwrap() > 0);
                if line == "\r\n" {
                    break;
                }
                request.push_str(&line);
            }
            assert!(request.starts_with("GET /prefix/state HTTP/1.1\r\n"));
            match scenario {
                0 => reader
                    .get_mut()
                    .write_all(
                        b"HTTP/1.1 409 Conflict\r\nContent-Length: 16\r\n\r\n{\"error\":\"busy\"}",
                    )
                    .unwrap(),
                1 => reader
                    .get_mut()
                    .write_all(b"HTTP/1.1 200 OK\r\nContent-Length: 1\r\n\r\n?")
                    .unwrap(),
                2 => drop(reader),
                _ => reader
                    .get_mut()
                    .write_all(b"HTTP/1.1 200 OK\r\nContent-Length: 100\r\n\r\n")
                    .unwrap(),
            }
            wait.recv_timeout(Duration::from_secs(5)).unwrap();
            listener.set_nonblocking(true).unwrap();
            assert_eq!(
                listener.accept().unwrap_err().kind(),
                std::io::ErrorKind::WouldBlock,
                "request was retried"
            );
        });
        let error = runtime
            .block_on(
                Client::new(&url)
                    .unwrap()
                    .with_timeout(Some(if scenario == 3 {
                        Duration::from_millis(100)
                    } else {
                        Duration::from_secs(5)
                    }))
                    .state(),
            )
            .unwrap_err();
        match (scenario, error) {
            (
                0,
                Error::Api {
                    status: StatusCode::CONFLICT,
                    message,
                },
            ) => assert_eq!(message, "busy"),
            (1, Error::Json(_)) | (2, Error::Transport(_)) | (3, Error::Timeout) => (),
            (_, error) => panic!("scenario {scenario}: {error}"),
        }
        release.send(()).unwrap();
        server.join().unwrap();
    }
}
