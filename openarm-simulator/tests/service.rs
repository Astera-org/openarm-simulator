//! cargo test provisions pinned dependencies; CAN tests require Linux namespaces and vcan.
use serde_json::{Value, json};
use socket2::{Domain, SockAddr, Socket, Type};
use socketcan::{
    CanFdFrame, CanFdSocket, CanSocket, EmbeddedFrame, Frame, Socket as CanSocketApi, StandardId,
};
use std::{
    fs,
    io::{BufRead, BufReader, Read, Write},
    net::{SocketAddr, TcpListener, TcpStream},
    os::{
        fd::{AsRawFd, RawFd},
        unix::{net::UnixStream, process::CommandExt},
    },
    path::Path,
    process::{Child, Command, Stdio},
    sync::{Arc, Barrier},
    thread,
    time::{Duration, Instant},
};

const BINARY: &str = env!("CARGO_BIN_EXE_openarm-simulator");

fn http_response(reader: &mut BufReader<TcpStream>, status: u16) -> Value {
    let mut line = String::new();
    reader.read_line(&mut line).unwrap();
    assert_eq!(
        line.split_whitespace()
            .nth(1)
            .unwrap()
            .parse::<u16>()
            .unwrap(),
        status,
        "{line}"
    );
    let mut length = None;
    loop {
        line.clear();
        assert!(
            reader.read_line(&mut line).unwrap() > 0,
            "incomplete response headers"
        );
        if line == "\r\n" {
            break;
        }
        if let Some((name, value)) = line.split_once(':')
            && name.eq_ignore_ascii_case("Content-Length")
        {
            length = Some(value.trim().parse::<usize>().unwrap());
        }
    }
    let mut body = vec![0; length.expect("response Content-Length")];
    reader.read_exact(&mut body).unwrap();
    if body.is_empty() {
        Value::Null
    } else {
        serde_json::from_slice(&body).unwrap()
    }
}

fn command() -> Command {
    let mut command = Command::new(BINARY);
    for key in [
        "LISTEN_FDS",
        "LISTEN_PID",
        "LISTEN_FDS_FIRST_FD",
        "OPENARM_SIMULATOR_CONFIG",
    ] {
        command.env_remove(key);
    }
    command
}

fn inherit(command: &mut Command, fds: &[RawFd]) {
    let fds = fds.to_vec();
    unsafe {
        command.pre_exec(move || {
            for fd in &fds {
                let flags = libc::fcntl(*fd, libc::F_GETFD);
                if flags < 0 || libc::fcntl(*fd, libc::F_SETFD, flags & !libc::FD_CLOEXEC) < 0 {
                    return Err(std::io::Error::last_os_error());
                }
            }
            Ok(())
        });
    }
}

fn readable(fd: RawFd) {
    let mut poller = libc::pollfd {
        fd,
        events: libc::POLLIN,
        revents: 0,
    };
    assert_eq!(
        unsafe { libc::poll(&mut poller, 1, 30_000) },
        1,
        "startup/IPC timeout"
    );
}

struct Running(Child, SocketAddr);
impl Running {
    fn start(command: &mut Command) -> Self {
        let child = command
            .stdout(Stdio::piped())
            .stderr(Stdio::inherit())
            .spawn()
            .unwrap();
        let mut running = Self(child, "127.0.0.1:0".parse().unwrap());
        let stdout = running.0.stdout.take().unwrap();
        readable(stdout.as_raw_fd());
        let mut line = String::new();
        BufReader::new(stdout).read_line(&mut line).unwrap();
        running.1 = line
            .trim()
            .strip_prefix("HTTP administration: http://")
            .expect(&line)
            .parse()
            .unwrap();
        running
    }
    fn wait(&mut self) {
        let deadline = Instant::now() + Duration::from_secs(10);
        loop {
            if let Some(status) = self.0.try_wait().unwrap() {
                assert!(status.success(), "{status}");
                break;
            }
            assert!(Instant::now() < deadline, "simulator failed to stop");
            thread::sleep(Duration::from_millis(10));
        }
    }
    fn terminate(&mut self) {
        assert_eq!(unsafe { libc::kill(self.0.id() as i32, libc::SIGTERM) }, 0);
        self.wait();
    }
    fn request(&self, method: &str, path: &str, body: &str, headers: &str, status: u16) -> Value {
        let mut stream = TcpStream::connect(self.1).unwrap();
        stream
            .set_read_timeout(Some(Duration::from_secs(5)))
            .unwrap();
        stream
            .set_write_timeout(Some(Duration::from_secs(5)))
            .unwrap();
        write!(
            stream,
            "{method} {path} HTTP/1.0\r\nHost: {}\r\nContent-Length: {}\r\n{headers}\r\n{body}",
            self.1,
            body.len()
        )
        .unwrap();
        http_response(&mut BufReader::new(stream), status)
    }
    fn get(&self, path: &str) -> Value {
        self.request("GET", path, "", "", 200)
    }
    fn post(&self, path: &str, value: Value, status: u16) -> Value {
        self.request(
            "POST",
            path,
            &value.to_string(),
            "Content-Type: application/json\r\n",
            status,
        )
    }
}
impl Drop for Running {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

fn local_listener(path: &Path) -> Socket {
    let socket = Socket::new(Domain::UNIX, Type::SEQPACKET, None).unwrap();
    socket.bind(&SockAddr::unix(path).unwrap()).unwrap();
    socket.listen(16).unwrap();
    socket
}
fn read_json(socket: &Socket) -> Value {
    readable(socket.as_raw_fd());
    let mut bytes = [0; 32768];
    let count = (&*socket).read(&mut bytes).unwrap();
    serde_json::from_slice(&bytes[..count]).unwrap()
}
fn ipc_call(path: &Path, value: Value) -> Value {
    let socket = Socket::new(Domain::UNIX, Type::SEQPACKET, None).unwrap();
    socket.connect(&SockAddr::unix(path).unwrap()).unwrap();
    socket.send(value.to_string().as_bytes()).unwrap();
    read_json(&socket)
}
fn rejected(mut command: Command, fds: &[RawFd], reason: &str) {
    inherit(&mut command, fds);
    let output = command.output().unwrap();
    let error = String::from_utf8_lossy(&output.stderr);
    assert!(
        !output.status.success() && error.contains(reason),
        "expected {reason}: {error}"
    );
}

#[test]
fn cli_and_invalid_descriptors() {
    let help = command().arg("--help").output().unwrap();
    assert!(help.status.success());
    assert!(String::from_utf8_lossy(&help.stdout).contains("--left-interface"));
    for (flag, reason) in [("--http-fd", "TCP"), ("--ipc-fd", "Unix SEQPACKET")] {
        let wrong = Socket::new(Domain::IPV4, Type::DGRAM, None).unwrap();
        let fd = wrong.as_raw_fd();
        let mut cmd = command();
        cmd.args(["--model", "/unused", flag, &fd.to_string()]);
        rejected(cmd, &[fd], reason);
    }
    for fd in [0, 1, 2, 999_999] {
        let mut cmd = command();
        cmd.args(["--model", "/unused", "--http-fd", &fd.to_string()]);
        rejected(cmd, &[], "descriptor");
    }
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let fd = listener.as_raw_fd();
    let mut cmd = command();
    cmd.args([
        "--model",
        "/unused",
        "--http-fd",
        &fd.to_string(),
        "--parent-fd",
        &fd.to_string(),
    ]);
    rejected(cmd, &[fd], "distinct");
}

#[test]
fn host_listener_private_namespace() {
    let model = openarm_test_model::SCENE;
    for activation in [false, true] {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let address = listener.local_addr().unwrap();
        let (parent, lifetime) = UnixStream::pair().unwrap();
        let mut cmd = Command::new("unshare");
        cmd.args(["--user", "--map-root-user", "--net", "sh", "-ec",
            "ip link set lo up; ip link add rightbus type vcan; ip link set rightbus mtu 72 up; ip link add leftbus type vcan; ip link set leftbus mtu 72 up; exec \"$@\"", "namespace", BINARY,
            "--model", model, "--left-interface", "leftbus", "--right-interface", "rightbus", "--parent-fd", &lifetime.as_raw_fd().to_string()]);
        cmd.env_remove("LISTEN_FDS")
            .env_remove("LISTEN_PID")
            .env_remove("OPENARM_SIMULATOR_CONFIG");
        if activation {
            cmd.env("LISTEN_FDS", "1")
                .env("LISTEN_FDS_FIRST_FD", listener.as_raw_fd().to_string());
        } else {
            cmd.args(["--http-fd", &listener.as_raw_fd().to_string()]);
        }
        inherit(&mut cmd, &[listener.as_raw_fd(), lifetime.as_raw_fd()]);
        let mut service = Running::start(&mut cmd);
        drop(listener);
        drop(lifetime);
        assert_eq!(service.1, address);
        assert_ne!(
            fs::read_link(format!("/proc/{}/ns/net", service.0.id())).unwrap(),
            fs::read_link("/proc/self/ns/net").unwrap()
        );
        assert_eq!(service.get("/state")["state"].as_object().unwrap().len(), 2);
        drop(parent);
        service.wait();
    }
}

fn can_command(bus: &CanFdSocket, data: [u8; 8], id: u16, status: u8) -> [u8; 8] {
    let frame = CanFdFrame::with_flags(
        StandardId::new(id).unwrap(),
        &data,
        socketcan::id::FdFlags::BRS,
    )
    .unwrap();
    bus.write_frame(&frame).unwrap();
    let reply = bus.read_frame().unwrap();
    let joint = if id == 0x7ff {
        u16::from_le_bytes([data[0], data[1]])
    } else {
        id
    };
    assert_eq!(reply.raw_id(), u32::from(joint + 16));
    assert_eq!(reply.data().len(), 8);
    assert_eq!(u16::from(reply.data()[0] & 15), joint);
    assert_eq!(reply.data()[0] >> 4, status);
    reply.data().try_into().unwrap()
}

fn in_can_namespace(test: &str) -> bool {
    if std::env::var_os("OPENARM_TEST_NAMESPACE").is_none() {
        let status = Command::new("unshare").args(["--user", "--map-root-user", "--net", "sh", "-ec",
            "ip link set lo up; for bus in can0 can1 rightbus leftbus; do ip link add \"$bus\" type vcan; ip link set \"$bus\" mtu 72 up; done; exec \"$@\"", "namespace"])
            .arg(std::env::current_exe().unwrap()).args(["--exact", test, "--nocapture"])
            .env("OPENARM_TEST_NAMESPACE", "1").status().unwrap();
        assert!(status.success());
        return false;
    }
    true
}

#[test]
fn realtime_can_motion() {
    if !in_can_namespace("realtime_can_motion") {
        return;
    }
    let model = openarm_test_model::SCENE;
    let mut service = Running::start(command().args(["--model", model, "--port", "0"]));
    let buses = ["can1", "can0"].map(|name| {
        let bus = CanFdSocket::open(name).unwrap();
        bus.set_read_timeout(Duration::from_secs(1)).unwrap();
        bus
    });
    service.post("/reset", json!({}), 200);
    for bus in &buses {
        can_command(bus, [255, 255, 255, 255, 255, 255, 255, 0xfc], 7, 1);
    }
    // MIT wrist targets +/-0.3 rad, kp=10, kd=0.5, zero velocity/torque.
    // Fixed wire packets keep this check independent of the simulator decoder.
    let packets = [
        [131, 17, 127, 240, 81, 25, 151, 255],
        [124, 237, 127, 240, 81, 25, 151, 255],
    ];
    let started = Instant::now();
    let initial = service.get("/state");
    let after_initial = started.elapsed().as_secs_f64();
    let mut commands = 0;
    for phase in 0..2 {
        let deadline = Instant::now() + Duration::from_millis(600);
        let mut next = Instant::now();
        let mut positions = [0.; 2];
        while Instant::now() < deadline {
            for (side, bus) in buses.iter().enumerate() {
                let reply = can_command(bus, packets[side ^ phase], 7, 1);
                positions[side] =
                    f64::from(u16::from_be_bytes([reply[1], reply[2]])) * 25. / 65535. - 12.5;
                commands += 1;
            }
            // Pace at 200 Hz without sending catch-up bursts after scheduling delays.
            next = (next + Duration::from_millis(5)).max(Instant::now());
            thread::sleep(next.saturating_duration_since(Instant::now()));
        }
        for (side, position) in positions.iter().enumerate() {
            let target = if side == phase { 0.3 } else { -0.3 };
            assert!(
                (position - target).abs() < 0.05,
                "phase {phase}, arm {side}: CAN position {position}, target {target}"
            );
        }
    }
    let before_final = started.elapsed().as_secs_f64();
    let final_state = service.get("/state");
    let after_final = started.elapsed().as_secs_f64();
    let elapsed = final_state["time"].as_f64().unwrap() - initial["time"].as_f64().unwrap();
    // Bracket HTTP sampling latency and allow scheduling jitter; this is not a latency benchmark.
    assert!(
        elapsed >= before_final - after_initial - 0.05 && elapsed <= after_final + 0.05,
        "simulated {elapsed}s, wall interval {}..{after_final}s",
        before_final - after_initial
    );
    for field in ["commands", "replies"] {
        assert_eq!(
            final_state["statistics"][field].as_u64().unwrap()
                - initial["statistics"][field].as_u64().unwrap(),
            commands
        );
    }
    assert_eq!(final_state["statistics"]["rejected"], 0);
    assert_eq!(final_state["statistics"]["dropped"], 0);
    service.terminate();
}

#[test]
fn can_http_and_lifecycle() {
    if !in_can_namespace("can_http_and_lifecycle") {
        return;
    }
    let model = openarm_test_model::SCENE;
    let directory = tempfile::tempdir().unwrap();
    // Default binds, all sockets inherited with custom names, and just one CAN fd.
    for mode in 0..3 {
        let names = if mode == 0 {
            ["can0", "can1"]
        } else {
            ["rightbus", "leftbus"]
        };
        let mut cmd = command();
        cmd.args(["--model", model, "--port", "0", "--report-dir"])
            .arg(directory.path());
        if mode != 0 {
            cmd.args(["--left-interface", names[1], "--right-interface", names[0]]);
        }
        let http = TcpListener::bind("127.0.0.1:0").unwrap();
        let local_path = directory.path().join(format!("inherited-{mode}.sock"));
        let local = local_listener(&local_path);
        let buses: Vec<_> = names
            .iter()
            .map(|name| CanSocket::open(name).unwrap())
            .collect();
        if mode > 0 {
            let mut fds = vec![http.as_raw_fd(), local.as_raw_fd(), buses[1].as_raw_fd()];
            cmd.args([
                "--http-fd",
                &fds[0].to_string(),
                "--ipc-fd",
                &fds[1].to_string(),
                "--left-can-fd",
                &fds[2].to_string(),
            ]);
            if mode == 1 {
                fds.push(buses[0].as_raw_fd());
                cmd.args(["--right-can-fd", &buses[0].as_raw_fd().to_string()]);
            }
            inherit(&mut cmd, &fds);
        }
        let mut service = Running::start(&mut cmd);
        drop(http);
        drop(local);
        drop(buses);
        let identity = service.get("/configuration")["config_sha256"].clone();
        if mode > 0 {
            assert_eq!(
                ipc_call(&local_path, json!({"action":"inspect"}))["config_sha256"],
                identity
            );
            assert!(
                ipc_call(&local_path, json!({"action":"step"}))
                    .get("error")
                    .is_some()
            );
        }
        for (name, side) in names.iter().zip(["right", "left"]) {
            let bus = CanFdSocket::open(name).unwrap();
            bus.set_read_timeout(Duration::from_secs(2)).unwrap();
            can_command(&bus, [255, 255, 255, 255, 255, 255, 255, 0xfc], 1, 1);
            service.post("/fault", json!([side,1,{"status":9}]), 200);
            can_command(&bus, [1, 0, 0xcc, 0, 0, 0, 0, 0], 0x7ff, 9);
            can_command(&bus, [255, 255, 255, 255, 255, 255, 255, 0xfb], 1, 0);
            can_command(&bus, [255, 255, 255, 255, 255, 255, 255, 0xfc], 1, 1);
            can_command(&bus, [255, 255, 255, 255, 255, 255, 255, 0xfd], 1, 0);
        }
        for value in [
            json!(["right",1,{"status":1}]),
            json!(["wrong", 1, {}]),
            json!(["right", 0, {}]),
            json!(["right",1,{"status":16}]),
            json!({}),
            Value::Null,
        ] {
            service.post("/fault", value, 400);
        }
        service.post("/push", json!({"right": ([1; 8])}), 400);
        service.post("/push", json!([]), 400);
        service.post("/fault", json!(["left", 1, []]), 400);
        service.post("/reset", json!({"right": ([0; 7])}), 400);
        for headers in [
            "Content-Type: application/json\r\nContent-Length: 3\r\n",
            "Content-Type: application/json\r\nTransfer-Encoding: chunked\r\n",
        ] {
            service.request("POST", "/reset", "{}", headers, 400);
        }
        for raw in ["{\"right\":[NaN]}", "{"] {
            service.request(
                "POST",
                "/reset",
                raw,
                "Content-Type: application/json\r\n",
                400,
            );
        }
        service.request("POST", "/reset", "{}", "Content-Type: text/plain\r\n", 415);
        service.request(
            "POST",
            "/reset",
            &" ".repeat(16385),
            "Content-Type: application/json\r\n",
            413,
        );
        service.request(
            "POST",
            "/reset",
            "{}",
            "Origin: https://example.com\r\n",
            403,
        );
        service.post("/command", json!({}), 404);
        service.post("/step", json!({}), 404);
        service.post("/push", json!({"right": ([0.1; 7])}), 200);
        let state = service.get("/state");
        assert_eq!(
            state["plant"]["applied_torque_nm"]["right"],
            json!(([0.1; 7]))
        );
        assert!(state["statistics"]["commands"].as_u64().unwrap() >= 10);
        let state = service.post("/reset", json!({}), 200);
        assert_eq!(state["time"], 0.);
        assert_eq!(
            state["plant"]["applied_torque_nm"]["right"],
            json!(([0.; 7]))
        );
        assert!(
            state["state"]
                .as_object()
                .unwrap()
                .values()
                .flat_map(|a| a.as_array().unwrap())
                .all(|m| m["status"] == 0)
        );
        assert_eq!(service.get("/configuration")["config_sha256"], identity);
        assert!(service.get("/state")["time"].as_f64().unwrap() > 0.);
        if mode == 0 {
            let mut stalled = TcpStream::connect(service.1).unwrap();
            stalled.write_all(b"POST /reset HTTP/1.1\r\nHost: localhost\r\nContent-Type: application/json\r\nContent-Length: 2\r\n\r\n{").unwrap();
            stalled
                .set_read_timeout(Some(Duration::from_secs(4)))
                .unwrap();
            let start = Instant::now();
            let bus = CanFdSocket::open("can0").unwrap();
            bus.set_read_timeout(Duration::from_secs(1)).unwrap();
            can_command(&bus, [1, 0, 0xcc, 0, 0, 0, 0, 0], 0x7ff, 0);
            let barrier = Arc::new(Barrier::new(8));
            thread::scope(|scope| {
                for _ in 0..8 {
                    let barrier = Arc::clone(&barrier);
                    let address = service.1;
                    scope.spawn(move || {
                        let stream = TcpStream::connect(address).unwrap();
                        stream
                            .set_read_timeout(Some(Duration::from_secs(1)))
                            .unwrap();
                        let mut reader = BufReader::new(stream);
                        barrier.wait();
                        // Three requests on the SAME TCP connection, while
                        // other clients are active and one body is stalled.
                        for path in ["/state", "/configuration", "/state"] {
                            write!(
                                reader.get_mut(),
                                "GET {path} HTTP/1.1\r\nHost: {address}\r\n\r\n"
                            )
                            .unwrap();
                            let value = http_response(&mut reader, 200);
                            assert!(value.get("config_sha256").is_some());
                        }
                    });
                }
            });
            assert!(
                start.elapsed() < Duration::from_secs(1),
                "stalled HTTP client blocked administration"
            );
            assert!(
                http_response(&mut BufReader::new(stalled), 408)
                    .get("error")
                    .is_some()
            );
        }
        service.terminate();
        let report: Value = serde_json::from_reader(
            fs::File::open(directory.path().join("simulator.json")).unwrap(),
        )
        .unwrap();
        assert_eq!(report["config_sha256"], identity);
        assert!(local_path.exists());
    }
    // Optional Lab registration and independent parent lifetime descriptor.
    let runtime = directory.path();
    fs::write(
        runtime.join("config.json"),
        json!({"report_dir": runtime}).to_string(),
    )
    .unwrap();
    let manager = local_listener(&runtime.join("manager.sock"));
    let register = thread::spawn(move || {
        readable(manager.as_raw_fd());
        let (client, _) = manager.accept().unwrap();
        let message = read_json(&client);
        assert_eq!(message["op"], "register");
        assert_eq!(message["instance"], "test-instance");
        assert_eq!(message["endpoint"]["interfaces"], json!(["can0", "can1"]));
        client.send(b"{\"ok\":true}").unwrap();
    });
    let (parent, lifetime) = UnixStream::pair().unwrap();
    let mut cmd = command();
    cmd.args([
        "--model",
        model,
        "--port",
        "0",
        "--parent-fd",
        &lifetime.as_raw_fd().to_string(),
        "--runtime",
    ])
    .arg(runtime)
    .env("HQ_SERVICE_INSTANCE", "test-instance");
    inherit(&mut cmd, &[lifetime.as_raw_fd()]);
    let mut service = Running::start(&mut cmd);
    register.join().unwrap();
    drop(lifetime);
    assert!(
        ipc_call(&runtime.join("simulator.sock"), json!({}))
            .get("state")
            .is_some()
    );
    drop(parent);
    service.wait();
    assert!(!runtime.join("simulator.sock").exists());
    reject_bad_can(model);
}

fn reject_bad_can(model: &str) {
    let buses = [
        CanSocket::open("can0").unwrap(),
        CanSocket::open("can1").unwrap(),
    ];
    let unbound = Socket::new(
        Domain::from(libc::AF_CAN),
        Type::RAW,
        Some(socket2::Protocol::from(libc::CAN_RAW)),
    )
    .unwrap();
    let tcp = TcpListener::bind("127.0.0.1:0").unwrap();
    for (right, left, reason) in [
        (tcp.as_raw_fd(), buses[1].as_raw_fd(), "CAN_RAW"),
        (buses[1].as_raw_fd(), buses[0].as_raw_fd(), "must be bound"),
        (unbound.as_raw_fd(), buses[1].as_raw_fd(), "must be bound"),
        (buses[0].as_raw_fd(), buses[0].as_raw_fd(), "distinct"),
    ] {
        let mut cmd = command();
        cmd.args([
            "--model",
            model,
            "--port",
            "0",
            "--right-can-fd",
            &right.to_string(),
            "--left-can-fd",
            &left.to_string(),
        ]);
        rejected(cmd, &[right, left], reason);
    }
    let mut foreign = Command::new("unshare");
    foreign.args(["--user", "--map-root-user", "--net", "sh", "-ec",
        "for bus in can0 can1; do ip link add \"$bus\" type vcan; ip link set \"$bus\" mtu 72 up; done; exec \"$@\"", "namespace", BINARY,
        "--model", model, "--port", "0", "--host", "0.0.0.0", "--right-can-fd", &buses[0].as_raw_fd().to_string(), "--left-can-fd", &buses[1].as_raw_fd().to_string()]);
    rejected(
        foreign,
        &[buses[0].as_raw_fd(), buses[1].as_raw_fd()],
        "network namespace",
    );
    for names in [
        ["lo", "can1"],
        ["can0", "lo"],
        ["can0", "can0"],
        ["missing", "can1"],
    ] {
        let mut cmd = command();
        cmd.args([
            "--model",
            model,
            "--port",
            "0",
            "--right-interface",
            names[0],
            "--left-interface",
            names[1],
        ]);
        rejected(
            cmd,
            &[],
            if names[0] == names[1] {
                "distinct"
            } else if names[0] == "missing" {
                "cannot inspect"
            } else {
                "refusing a physical interface"
            },
        );
    }
}
