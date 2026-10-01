//! cargo test provisions pinned dependencies; CAN tests require Linux namespaces, vcan,
//! and iproute2. The TX-overflow fixture also uses the kernel netem qdisc via tc.
use serde_json::{Value, json};
use socket2::{Domain, Socket, Type};
use socketcan::{
    CanFdFrame, CanFdSocket, CanFilter, CanSocket, EmbeddedFrame, Frame, Socket as CanSocketApi,
    SocketOptions, StandardId,
};
use std::{
    fs,
    io::{BufRead, BufReader, Read, Write},
    net::{SocketAddr, TcpListener, TcpStream},
    os::{
        fd::{AsRawFd, RawFd},
        unix::{net::UnixStream, process::CommandExt},
    },
    process::{Child, Command, Stdio},
    sync::{Arc, Barrier},
    thread,
    time::{Duration, Instant},
};

const BINARY: &str = env!("CARGO_BIN_EXE_openarm-simulator-rs");

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
    let mut body = vec![
        0;
        if status == 204 {
            0
        } else {
            length.expect("response Content-Length")
        }
    ];
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
        "startup timeout"
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
    fn clock(&self, path: &str, status: u16) {
        assert_eq!(self.request("POST", path, "", "", status), Value::Null);
    }
    fn advance(&self, duration_ns: u64) {
        assert_eq!(
            self.post("/advance", json!({"duration_ns": duration_ns}), 200),
            Value::Null
        );
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
    {
        let wrong = Socket::new(Domain::IPV4, Type::DGRAM, None).unwrap();
        let fd = wrong.as_raw_fd();
        let mut cmd = command();
        cmd.args(["--model", "/unused", "--http-fd", &fd.to_string()]);
        rejected(cmd, &[fd], "TCP");
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
    service.clock("/reset", 200);
    service.clock("/unpause", 200);
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
    let after_initial = u64::try_from(started.elapsed().as_nanos()).unwrap();
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
    let before_final = u64::try_from(started.elapsed().as_nanos()).unwrap();
    let final_state = service.get("/state");
    let after_final = u64::try_from(started.elapsed().as_nanos()).unwrap();
    let elapsed = final_state["time_ns"].as_u64().unwrap() - initial["time_ns"].as_u64().unwrap();
    // Bracket HTTP sampling latency and allow scheduling jitter; this is not a latency benchmark.
    assert!(
        elapsed >= (before_final - after_initial).saturating_sub(50_000_000)
            && elapsed <= after_final + 50_000_000,
        "simulated {elapsed}ns, wall interval {}..{after_final}ns",
        before_final - after_initial
    );
    for field in ["commands", "replies"] {
        assert_eq!(
            final_state["statistics"][field].as_u64().unwrap()
                - initial["statistics"][field].as_u64().unwrap(),
            commands
        );
    }
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
        let buses: Vec<_> = names
            .iter()
            .map(|name| CanSocket::open(name).unwrap())
            .collect();
        if mode > 0 {
            let mut fds = vec![http.as_raw_fd(), buses[1].as_raw_fd()];
            cmd.args([
                "--http-fd",
                &fds[0].to_string(),
                "--left-can-fd",
                &fds[1].to_string(),
            ]);
            if mode == 1 {
                fds.push(buses[0].as_raw_fd());
                cmd.args(["--right-can-fd", &buses[0].as_raw_fd().to_string()]);
            }
            inherit(&mut cmd, &fds);
        }
        let mut service = Running::start(&mut cmd);
        drop(http);
        drop(buses);
        let configuration = service.get("/configuration");
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
        service.clock("/reset", 200);
        let state = service.get("/state");
        assert_eq!(state["time_ns"], 0);
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
        assert_eq!(service.get("/configuration"), configuration);
        assert_eq!(service.get("/state")["time_ns"], 0);
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
                            assert!(value.get(path.trim_start_matches('/')).is_some());
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
        assert_eq!(
            report["timestep_ns"],
            configuration["configuration"]["timestep_ns"]
        );
    }
    // Independent parent lifetime descriptor.
    let (parent, lifetime) = UnixStream::pair().unwrap();
    let mut cmd = command();
    cmd.args([
        "--model",
        model,
        "--port",
        "0",
        "--parent-fd",
        &lifetime.as_raw_fd().to_string(),
    ]);
    inherit(&mut cmd, &[lifetime.as_raw_fd()]);
    let mut service = Running::start(&mut cmd);
    drop(lifetime);
    drop(parent);
    service.wait();
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

#[test]
fn clock_start_reset_and_fixed_updates() {
    if !in_can_namespace("clock_start_reset_and_fixed_updates") {
        return;
    }
    let dir = tempfile::tempdir().unwrap();
    let config = dir.path().join("plant.json");
    let period = 250_000u64;
    fs::write(
        &config,
        json!({
            "timestep_ns": period,
            "poses": {"left": [0., 0., 0., 0., 0., 0., 0.25, -0.2]}
        })
        .to_string(),
    )
    .unwrap();
    let mut service = Running::start(command().args([
        "--model",
        openarm_test_model::SCENE,
        "--port",
        "0",
        "--config",
        config.to_str().unwrap(),
    ]));
    let initial = service.get("/state");
    assert_eq!(initial["paused"], true);
    assert_eq!(initial["advancing"], false);
    assert_eq!(initial["time_ns"], 0);
    assert_eq!(initial["timestep_ns"], period);
    assert_eq!(initial["state"]["left"][6]["q"], 0.25);
    thread::sleep(Duration::from_millis(20));
    assert_eq!(service.get("/state"), initial);
    service.clock("/pause", 204);
    service.advance(period - 1);
    let fractional = service.get("/state");
    assert_eq!(fractional["time_ns"], period - 1);
    assert_eq!(fractional["statistics"]["steps"], 0);
    assert_eq!(fractional["state"], initial["state"]);
    service.advance(2);
    assert_eq!(service.get("/state")["statistics"]["steps"], 1);
    service.advance(3 * period + 7);
    let split = service.get("/state");
    assert_eq!(split["time_ns"], 4 * period + 8);
    assert_eq!(split["statistics"]["steps"], 4);
    service.clock("/reset", 200);
    assert_eq!(service.get("/state"), initial);
    service.advance(4 * period + 8);
    assert_eq!(service.get("/state"), split);

    for payload in [
        json!({"duration_ns": -1}),
        json!({"duration_ns": 1.0}),
        json!({"duration_ns": true}),
        json!({"duration_ns": u64::MAX}),
        json!({"steps": 1}),
        json!({"duration_ns": 1, "extra": 1}),
    ] {
        service.post("/advance", payload, 400);
        assert_eq!(service.get("/state"), split);
    }
    service.post("/reset", json!({"left": ([0; 8])}), 400);
    let bus = client_bus("can1", 0x17);
    can_command(&bus, [255, 255, 255, 255, 255, 255, 255, 0xfc], 7, 1);
    service.post(
        "/fault",
        json!(["left", 7, {"status": 9, "silent": true}]),
        200,
    );
    service.post("/push", json!({"left": ([0.1; 7])}), 200);
    service.clock("/unpause", 200);
    service.clock("/unpause", 204);
    service.post("/advance", json!({"duration_ns": 1}), 409);
    thread::sleep(Duration::from_millis(20));
    service.clock("/pause", 200);
    service.clock("/pause", 204);
    let paused = service.get("/state");
    assert!(paused["time_ns"].as_u64().unwrap() > split["time_ns"].as_u64().unwrap());
    let paused_time = paused["time_ns"].as_u64().unwrap();
    assert_eq!(paused["statistics"]["steps"], paused_time / period);
    thread::sleep(Duration::from_millis(20));
    assert_eq!(service.get("/state"), paused);
    service.advance(period + 1);
    let advanced = service.get("/state");
    assert_eq!(advanced["time_ns"], paused_time + period + 1);
    assert_eq!(
        advanced["statistics"]["steps"],
        (paused_time + period + 1) / period
    );
    service.clock("/unpause", 200);
    service.clock("/reset", 200);
    assert_eq!(service.get("/state"), initial);
    service.terminate();
}

fn client_bus(name: &str, id: u32) -> CanFdSocket {
    let bus = CanFdSocket::open(name).unwrap();
    bus.set_filters(&[CanFilter::new(id, 0xc000_07ff)]).unwrap();
    // These are wall-time failure guards, not simulated motor deadlines.
    bus.set_read_timeout(Duration::from_secs(5)).unwrap();
    bus.set_write_timeout(Duration::from_secs(5)).unwrap();
    bus
}

fn frame(id: u16, data: [u8; 8]) -> CanFdFrame {
    CanFdFrame::with_flags(
        StandardId::new(id).unwrap(),
        &data,
        socketcan::id::FdFlags::BRS,
    )
    .unwrap()
}

#[test]
fn can_batches_between_advances_and_accelerated_motion() {
    if !in_can_namespace("can_batches_between_advances_and_accelerated_motion") {
        return;
    }
    let mut service =
        Running::start(command().args(["--model", openarm_test_model::SCENE, "--port", "0"]));
    let bus = client_bus("can1", 0x17);
    service.advance(0);
    let initial = service.get("/state");
    let count = 24;
    for _ in 0..3 {
        // Ordinary eight-message bursts, with replies drained between bursts.
        // Both endpoints retain their default Linux socket buffer sizes.
        for _ in 0..8 {
            bus.write_frame(&frame(0x7ff, [7, 0, 0xcc, 0, 0, 0, 0, 0]))
                .unwrap();
        }
        for _ in 0..8 {
            let reply = bus.read_frame().unwrap();
            assert_eq!(reply.raw_id(), 0x17);
            assert_eq!(reply.data()[0], 7);
        }
    }
    let state = service.get("/state");
    assert_eq!(state["time_ns"], 0);
    assert_eq!(state["state"], initial["state"]);
    assert_eq!(state["statistics"]["commands"], count);
    assert_eq!(state["statistics"]["replies"], count);
    assert_eq!(state["statistics"]["dropped"], 0);
    service.advance(1);
    println!(
        "paused simulator: {count} commands in batches of eight, all replies received with default socket buffers"
    );

    // Coordinate at controller boundaries. The command reply proves the motor
    // accepted the command BEFORE advancing; an HTTP call alone is no CAN fence.
    can_command(&bus, [255, 255, 255, 255, 255, 255, 255, 0xfc], 7, 1);
    let wall = Instant::now();
    let mut exchanges = count + 1;
    for (packet, target) in [
        ([131, 17, 127, 240, 81, 25, 151, 255], 0.3),
        ([124, 237, 127, 240, 81, 25, 151, 255], -0.3),
    ] {
        for _ in 0..120 {
            can_command(&bus, packet, 7, 1);
            exchanges += 1;
            service.advance(5_000_000);
        }
        let reply = can_command(&bus, [7, 0, 0xcc, 0, 0, 0, 0, 0], 0x7ff, 1);
        exchanges += 1;
        let position = f64::from(u16::from_be_bytes([reply[1], reply[2]])) * 25. / 65535. - 12.5;
        assert!(
            (position - target).abs() < 0.05,
            "position={position}, target={target}"
        );
    }
    let state = service.get("/state");
    assert_eq!(state["time_ns"], 1_200_000_001u64);
    assert_eq!(state["statistics"]["commands"], exchanges);
    assert_eq!(state["statistics"]["replies"], exchanges);
    assert_eq!(state["statistics"]["dropped"], 0);
    // Physics updates do not emit unsolicited motor feedback.
    bus.set_nonblocking(true).unwrap();
    assert_eq!(
        bus.read_frame().unwrap_err().kind(),
        std::io::ErrorKind::WouldBlock
    );
    // Do not make CI correctness depend on the machine's performance.
    println!(
        "coordinated CAN control: 1200000000 simulated ns in {} wall ns; all {exchanges} exchanges received",
        wall.elapsed().as_nanos()
    );
    service.terminate();
}

#[test]
fn can_and_shutdown_remain_live_during_advance() {
    if !in_can_namespace("can_and_shutdown_remain_live_during_advance") {
        return;
    }
    let mut service =
        Running::start(command().args(["--model", openarm_test_model::SCENE, "--port", "0"]));
    // Keep an advance pending, then shut down deliberately. No arbitrary maximum
    // duration, integration batch limit, or sleeping to manufacture an overlap.
    let mut advance = TcpStream::connect(service.1).unwrap();
    let body = json!({"duration_ns": 3_600_000_000_000u64}).to_string();
    write!(advance, "POST /advance HTTP/1.1\r\nHost: localhost\r\nContent-Type: application/json\r\nContent-Length: {}\r\n\r\n{body}", body.len()).unwrap();
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        let state = service.get("/state");
        if state["advancing"] == true {
            break;
        }
        assert!(Instant::now() < deadline, "advance did not start");
        thread::yield_now();
    }
    let bus = client_bus("can1", 0x17);
    for _ in 0..10 {
        can_command(&bus, [7, 0, 0xcc, 0, 0, 0, 0, 0], 0x7ff, 0);
    }
    service.post("/advance", json!({"duration_ns": 1}), 409);
    service.request("POST", "/unpause", "", "", 409);
    service.request("POST", "/reset", "", "", 409);
    assert_eq!(service.get("/state")["advancing"], true);
    service.terminate();
}

fn tc(args: &[&str]) {
    let output = Command::new("tc").args(args).output().unwrap();
    assert!(
        output.status.success(),
        "tc {args:?}: {}",
        String::from_utf8_lossy(&output.stderr)
    );
}

#[test]
fn full_can_transmit_queue_rejects_writes_and_simulator_replies() {
    if !in_can_namespace("full_can_transmit_queue_rejects_writes_and_simulator_replies") {
        return;
    }
    // Starting the simulator verifies these are vcan interfaces before we
    // change any queue. The test runs in its own disposable network namespace.
    let mut service =
        Running::start(command().args(["--model", openarm_test_model::SCENE, "--port", "0"]));
    let bus = client_bus("can1", 0x17);
    let filler = CanFdSocket::open("can1").unwrap();
    filler.set_filter_drop_all().unwrap();
    filler.set_loopback(false).unwrap();
    filler.set_nonblocking(true).unwrap();

    // Hold ten frames in the actual Linux transmit qdisc. The long delay only
    // prevents it draining during this test; deleting the qdisc removes it
    // immediately. This injects queue exhaustion, not USB/CAN bus timing.
    tc(&[
        "qdisc", "add", "dev", "can1", "root", "netem", "limit", "10", "delay", "60s",
    ]);
    for sequence in 0..9u64 {
        filler
            .write_frame(&frame(0x555, sequence.to_le_bytes()))
            .unwrap();
    }
    // vcan's local loopback delivers this accepted command to the simulator.
    // It occupies the last TX slot, so the simulator's reply cannot enqueue.
    let query = frame(0x7ff, [7, 0, 0xcc, 0, 0, 0, 0, 0]);
    bus.write_frame(&query).unwrap();
    assert_eq!(
        filler
            .write_frame(&frame(0x555, [0; 8]))
            .unwrap_err()
            .raw_os_error(),
        Some(libc::ENOBUFS)
    );

    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        let state = service.get("/state");
        if state["statistics"]["dropped"] == 1 {
            assert_eq!(state["statistics"]["commands"], 1);
            assert_eq!(state["statistics"]["replies"], 0);
            assert_eq!(state["time_ns"], 0);
            break;
        }
        assert!(
            Instant::now() < deadline,
            "simulator did not account for the failed CAN write: {state}"
        );
        thread::yield_now();
    }
    let output = Command::new("tc")
        .args(["-j", "-s", "qdisc", "show", "dev", "can1"])
        .output()
        .unwrap();
    assert!(output.status.success());
    let queues: Value = serde_json::from_slice(&output.stdout).unwrap();
    assert_eq!(queues[0]["kind"], "netem");
    assert_eq!(queues[0]["qlen"], 10);
    assert_eq!(queues[0]["drops"], 2); // Our extra write and the simulator's reply.
    bus.set_nonblocking(true).unwrap();
    assert_eq!(
        bus.read_frame().unwrap_err().kind(),
        std::io::ErrorKind::WouldBlock
    );

    tc(&["qdisc", "del", "dev", "can1", "root"]);
    bus.set_nonblocking(false).unwrap();
    can_command(&bus, [7, 0, 0xcc, 0, 0, 0, 0, 0], 0x7ff, 0);
    let state = service.get("/state");
    assert_eq!(state["statistics"]["commands"], 2);
    assert_eq!(state["statistics"]["replies"], 1);
    assert_eq!(state["statistics"]["dropped"], 1);
    println!(
        "CAN TX queue: ten frames accepted; excess client write returned ENOBUFS; simulator counted its rejected reply; communication recovered after removing the blockage. Socket buffers stayed at defaults."
    );
    service.terminate();
}

#[test]
fn can_client_codec_and_socket_errors() {
    if !in_can_namespace("can_client_codec_and_socket_errors") {
        return;
    }
    use damiao_can_rs::{
        ControlMode, DM4310_DEFAULT_MAPPING_RANGES, Feedback, MotorStatus, Request,
    };
    let mut service =
        Running::start(command().args(["--model", openarm_test_model::SCENE, "--port", "0"]));
    let bus = client_bus("can1", 0x17);
    // An empty blocking read with a timeout must be reported, not a valid frame.
    bus.set_read_timeout(Duration::from_millis(10)).unwrap();
    let error = bus.read_frame().unwrap_err();
    assert_eq!(error.kind(), std::io::ErrorKind::WouldBlock);
    let limits = DM4310_DEFAULT_MAPPING_RANGES;
    let packet = Request::Enable(ControlMode::Mit).encode(7, limits).unwrap();
    let enable_frame =
        CanFdFrame::new(StandardId::new(packet.id as u16).unwrap(), packet.data()).unwrap();
    bus.write_frame(&enable_frame).unwrap();
    bus.set_read_timeout(Duration::from_secs(5)).unwrap();
    let response = bus.read_frame().unwrap();
    let feedback = Feedback::decode(response.data(), limits).unwrap();
    assert_eq!(response.raw_id(), 0x17);
    assert_eq!(feedback.reported_id, 7);
    assert_eq!(feedback.status, MotorStatus::ENABLED);

    let before = service.get("/state");
    for (mode, id, length) in [(2, 0x107, 8), (3, 0x207, 4), (4, 0x307, 8)] {
        bus.write_frame(&frame(0x7ff, [7, 0, 0x55, 10, mode, 0, 0, 0]))
            .unwrap();
        bus.write_frame(&CanFdFrame::new(StandardId::new(id).unwrap(), &[0; 8][..length]).unwrap())
            .unwrap();
        bus.write_frame(&frame(id, [255, 255, 255, 255, 255, 255, 255, 0xfb]))
            .unwrap();
    }
    // Read back CTRL_MODE to verify unsupported requests did not switch modes.
    bus.write_frame(&frame(0x7ff, [7, 0, 0x33, 10, 0, 0, 0, 0]))
        .unwrap();
    let response = bus.read_frame().unwrap();
    assert_eq!(response.raw_id(), 0x17);
    assert_eq!(response.data(), [7, 0, 0x33, 10, 1, 0, 0, 0]);
    let after = service.get("/state");
    assert_eq!(after["state"], before["state"]);
    assert_eq!(
        after["statistics"]["replies"].as_u64().unwrap()
            - before["statistics"]["replies"].as_u64().unwrap(),
        1
    );
    bus.set_nonblocking(true).unwrap();
    assert_eq!(
        bus.read_frame().unwrap_err().kind(),
        std::io::ErrorKind::WouldBlock
    );
    service.terminate();
    // Only this test's private namespace is affected.
    assert!(
        Command::new("ip")
            .args(["link", "set", "can1", "down"])
            .status()
            .unwrap()
            .success()
    );
    assert_eq!(
        bus.write_frame(&enable_frame).unwrap_err().raw_os_error(),
        Some(libc::ENETDOWN)
    );
}
