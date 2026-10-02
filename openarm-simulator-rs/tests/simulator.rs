//! Requires unprivileged Linux namespaces, vcan, and iproute2 (including netem).
use damiao_can::{Feedback, MappingRanges, MotorStatus};
use openarm_simulator_client::{
    Client, Error, StatusCode,
    models::{AppliedForce, Fault, Spring},
};
use polling::{Event, Events, Poller};
use serde_json::json;
use socketcan::{
    CanFdFrame, CanFdSocket, CanFilter, EmbeddedFrame, Frame, Socket, SocketOptions, StandardId,
};
use std::{
    fs,
    io::{BufRead, BufReader},
    net::TcpListener,
    os::{
        fd::{AsRawFd, RawFd},
        unix::{net::UnixStream, process::CommandExt},
    },
    process::{Child, Command, Stdio},
    thread,
    time::{Duration, Instant},
};

const BINARY: &str = env!("CARGO_BIN_EXE_openarm-simulator");
const QUERY: [u8; 8] = [7, 0, 0xcc, 0, 0, 0, 0, 0];

fn runtime() -> tokio::runtime::Runtime {
    tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
        .unwrap()
}

fn namespace() -> Command {
    let mut command = Command::new("unshare");
    command.args(["--user", "--map-root-user", "--net", "sh", "-ec",
        "ip link set lo up; for bus in bench aux; do ip link add \"$bus\" type vcan; ip link set \"$bus\" mtu 72 up; done; exec \"$@\"", "namespace"]);
    command
}

fn in_namespace(test: &str) -> bool {
    if std::env::var_os("OPENARM_TEST_NAMESPACE").is_some() {
        return true;
    }
    assert!(
        namespace()
            .arg(std::env::current_exe().unwrap())
            .args(["--exact", test, "--nocapture"])
            .env("OPENARM_TEST_NAMESPACE", "1")
            .status()
            .unwrap()
            .success()
    );
    false
}

fn robot(command: &mut Command) -> &mut Command {
    command.args([
        "--model",
        openarm_test_model::SCENE,
        "--config",
        openarm_test_model::CONFIG,
        "--can-interface",
        "left=bench",
        "--can-interface",
        "right=aux",
    ])
}

struct Running {
    child: Child,
    parent: Option<UnixStream>,
    client: Client,
}

impl Running {
    fn start(command: &mut Command, mut fds: Vec<RawFd>) -> Self {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let url = format!("http://{}", listener.local_addr().unwrap());
        let (parent, lifetime) = UnixStream::pair().unwrap();
        command.args([
            "--http-fd",
            &listener.as_raw_fd().to_string(),
            "--parent-fd",
            &lifetime.as_raw_fd().to_string(),
        ]);
        for key in ["LISTEN_FDS", "LISTEN_PID", "LISTEN_FDS_FIRST_FD"] {
            command.env_remove(key);
        }
        fds.extend([listener.as_raw_fd(), lifetime.as_raw_fd()]);
        unsafe {
            command.pre_exec(move || {
                for &fd in &fds {
                    let flags = libc::fcntl(fd, libc::F_GETFD);
                    if flags < 0 || libc::fcntl(fd, libc::F_SETFD, flags & !libc::FD_CLOEXEC) < 0 {
                        return Err(std::io::Error::last_os_error());
                    }
                }
                Ok(())
            });
        }
        let child = command
            .stdout(Stdio::piped())
            .stderr(Stdio::inherit())
            .spawn()
            .unwrap();
        let mut running = Self {
            child,
            parent: Some(parent),
            client: Client::new(&url).unwrap(),
        };
        let stdout = running.child.stdout.take().unwrap();
        let poller = Poller::new().unwrap();
        // SAFETY: stdout remains open until it is removed from the poller below.
        unsafe { poller.add(&stdout, Event::readable(0)).unwrap() };
        assert_eq!(
            poller
                .wait(&mut Events::new(), Some(Duration::from_secs(30)))
                .unwrap(),
            1,
            "startup timeout"
        );
        poller.delete(&stdout).unwrap();
        let mut line = String::new();
        BufReader::new(stdout).read_line(&mut line).unwrap();
        assert_eq!(line.trim(), format!("HTTP administration: {url}"));
        running
    }

    fn stop(&mut self) {
        self.parent.take();
        let deadline = Instant::now() + Duration::from_secs(5);
        loop {
            if let Some(status) = self.child.try_wait().unwrap() {
                assert!(status.success(), "{status}");
                return;
            }
            assert!(
                Instant::now() < deadline,
                "simulator did not exit on parent EOF"
            );
            thread::sleep(Duration::from_millis(10));
        }
    }
}

impl Drop for Running {
    fn drop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}

fn bus() -> CanFdSocket {
    let bus = CanFdSocket::open("bench").unwrap();
    bus.set_filters(&[CanFilter::new(0x17, 0xc000_07ff)])
        .unwrap();
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

fn exchange(bus: &CanFdSocket, id: u16, data: [u8; 8]) -> Feedback {
    bus.write_frame(&frame(id, data)).unwrap();
    let reply = bus.read_frame().unwrap();
    assert_eq!(reply.raw_id(), 0x17);
    Feedback::decode(
        reply.data(),
        MappingRanges {
            pmax: 12.5,
            vmax: 30.,
            tmax: 10.,
        },
    )
    .unwrap()
}

#[test]
fn clock_can_and_concurrent_control() {
    if !in_namespace("clock_can_and_concurrent_control") {
        return;
    }
    // One inherited CAN socket and one interface opened by the simulator.
    let inherited = CanFdSocket::open("bench").unwrap();
    let mut command = Command::new(BINARY);
    robot(&mut command).args(["--can-fd", &format!("left={}", inherited.as_raw_fd())]);
    let mut sim = Running::start(&mut command, vec![inherited.as_raw_fd()]);
    let bus = bus();
    runtime().block_on(async {
        let c = &sim.client;
        let initial = c.state().await.unwrap();
        assert!(initial.paused && !initial.advancing);
        assert_eq!(initial.time_ns, 0);
        let period = initial.timestep_ns;
        c.advance(Duration::from_nanos(period - 1)).await.unwrap();
        assert_eq!(c.state().await.unwrap().statistics.steps, 0);
        c.advance(Duration::from_nanos(2)).await.unwrap();
        let advanced = c.state().await.unwrap();
        assert_eq!(
            (advanced.time_ns, advanced.statistics.steps),
            (period + 1, 1)
        );
        c.reset().await.unwrap();
        assert_eq!(c.state().await.unwrap(), initial);

        assert_eq!(
            exchange(&bus, 7, [255, 255, 255, 255, 255, 255, 255, 0xfc]).status,
            MotorStatus::ENABLED
        );
        // MIT target 0.3 rad, kp=10, kd=0.5. Repeating feedback queries does
        // not change the command; HTTP inspection runs throughout the motion.
        exchange(&bus, 7, [131, 17, 127, 240, 81, 25, 151, 255]);
        assert_eq!(c.unpause().await.unwrap(), StatusCode::OK);
        assert_eq!(c.unpause().await.unwrap(), StatusCode::NO_CONTENT);
        assert!(matches!(
            c.advance(Duration::ZERO).await,
            Err(Error::Api {
                status: StatusCode::CONFLICT,
                ..
            })
        ));
        let deadline = Instant::now() + Duration::from_secs(5);
        loop {
            let feedback = exchange(&bus, 0x7ff, QUERY);
            let state = c.state().await.unwrap();
            assert!(!state.paused);
            if (feedback.q - 0.3).abs() < 0.05 && state.time_ns >= 600_000_000 {
                break;
            }
            assert!(
                Instant::now() < deadline,
                "motor did not reach target: {}",
                feedback.q
            );
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        assert_eq!(c.pause().await.unwrap(), StatusCode::OK);
        let paused = c.state().await.unwrap();
        assert_eq!(paused.statistics.steps, paused.time_ns / period);
        assert_eq!(paused.statistics.max_catchup_steps, 1);
        assert_eq!(c.pause().await.unwrap(), StatusCode::NO_CONTENT);
        assert_eq!(c.state().await.unwrap(), paused);
        assert_eq!(paused.statistics.dropped, 0);
        c.fault(
            "left_joint7",
            Fault {
                status: Some(MotorStatus::UNDERVOLTAGE),
                silent: None,
            },
        )
        .await
        .unwrap();
        assert_eq!(
            exchange(&bus, 0x7ff, QUERY).status,
            MotorStatus::UNDERVOLTAGE
        );
        c.reset().await.unwrap();
        assert_eq!(c.state().await.unwrap(), initial);

        let advancing = c.clone().with_timeout(None);
        let pending =
            tokio::spawn(async move { advancing.advance(Duration::from_secs(3600)).await });
        let deadline = Instant::now() + Duration::from_secs(5);
        while !c.state().await.unwrap().advancing {
            assert!(Instant::now() < deadline, "advance did not start");
            tokio::task::yield_now().await;
        }
        assert_eq!(exchange(&bus, 0x7ff, QUERY).status, MotorStatus::DISABLED);
        assert!(matches!(
            c.reset().await,
            Err(Error::Api {
                status: StatusCode::CONFLICT,
                ..
            })
        ));
        assert!(matches!(
            c.unpause().await,
            Err(Error::Api {
                status: StatusCode::CONFLICT,
                ..
            })
        ));
        assert!(matches!(
            c.advance(Duration::ZERO).await,
            Err(Error::Api {
                status: StatusCode::CONFLICT,
                ..
            })
        ));
        sim.stop();
        assert!(
            tokio::time::timeout(Duration::from_secs(5), pending)
                .await
                .unwrap()
                .unwrap()
                .is_err()
        );
    });
}

#[test]
fn forces_and_springs_move_the_scene() {
    let dir = tempfile::tempdir().unwrap();
    let model = dir.path().join("scene.xml");
    fs::write(
        &model,
        r#"<mujoco><option gravity="0 0 0"/><worldbody>
        <body name="block"><joint type="slide" axis="1 0 0"/>
        <geom type="sphere" size="0.05" mass="1"/><site name="attachment"/>
        </body></worldbody></mujoco>"#,
    )
    .unwrap();
    let mut sim = Running::start(Command::new(BINARY).arg("--model").arg(model), vec![]);
    runtime().block_on(async {
        let c = &sim.client;
        let initial = c.state().await.unwrap();
        let names = c.names().await.unwrap();
        let body = names.bodies["block"];
        let id = "load / #α";
        let mut force: AppliedForce = serde_json::from_value(json!({
            "point": names.sites["attachment"], "force_world_n": [1,0,0], "torque_world_nm": [0,0,0]
        }))
        .unwrap();
        assert_eq!(c.put_force(id, &force).await.unwrap(), StatusCode::CREATED);
        // Edit a load while the simulation runs; read state without pausing it.
        c.unpause().await.unwrap();
        tokio::time::sleep(Duration::from_millis(100)).await;
        assert!(c.state().await.unwrap().bodies[body.0].position.x.value > 0.);
        force.force.x.value = -1.;
        assert_eq!(
            c.put_force(id, &force).await.unwrap(),
            StatusCode::NO_CONTENT
        );
        c.pause().await.unwrap();
        assert_eq!(c.force(id).await.unwrap().force, force.force);
        c.reset().await.unwrap();
        force.force.x.value = 1.;
        c.put_force(id, &force).await.unwrap();
        c.advance(Duration::from_millis(100)).await.unwrap();
        force.force.x.value = -1.;
        c.put_force(id, &force).await.unwrap();
        c.advance(Duration::from_millis(100)).await.unwrap();
        c.delete_force(id).await.unwrap();
        let x = c.state().await.unwrap().bodies[body.0].position.x.value;
        assert!((x - 0.01).abs() < 0.001);
        c.advance(Duration::from_millis(100)).await.unwrap();
        assert!((c.state().await.unwrap().bodies[body.0].position.x.value - x).abs() < 0.001);
        assert!(matches!(
            c.force(id).await,
            Err(Error::Api {
                status: StatusCode::NOT_FOUND,
                ..
            })
        ));
        c.reset().await.unwrap();

        let mut spring: Spring = serde_json::from_value(json!({
            "endpoints": [names.sites["attachment"], {"body":0,"position_local_m":[1,0,0]}],
            "rest_length_m":0, "stiffness_n_per_m":4, "damping_n_s_per_m":4
        }))
        .unwrap();
        assert_eq!(
            c.put_spring(id, &spring).await.unwrap(),
            StatusCode::CREATED
        );
        c.advance(Duration::from_secs(4)).await.unwrap();
        assert!((c.state().await.unwrap().bodies[body.0].position.x.value - 1.).abs() < 0.02);
        spring.endpoints[1] =
            serde_json::from_value(json!({"body":0,"position_local_m":[-1,0,0]})).unwrap();
        assert_eq!(
            c.put_spring(id, &spring).await.unwrap(),
            StatusCode::NO_CONTENT
        );
        c.advance(Duration::from_secs(4)).await.unwrap();
        let x = c.state().await.unwrap().bodies[body.0].position.x.value;
        assert!((x + 1.).abs() < 0.02);
        c.delete_spring(id).await.unwrap();
        c.advance(Duration::from_secs(1)).await.unwrap();
        assert!((c.state().await.unwrap().bodies[body.0].position.x.value - x).abs() < 0.03);
        force.point =
            serde_json::from_value(json!({"body":999,"position_local_m":[0,0,0]})).unwrap();
        assert!(matches!(
            c.put_force(id, &force).await,
            Err(Error::Api {
                status: StatusCode::BAD_REQUEST,
                ..
            })
        ));
        c.reset().await.unwrap();
        assert_eq!(c.state().await.unwrap(), initial);
    });
    sim.stop();
}

fn tc(args: &[&str]) {
    let output = Command::new("tc").args(args).output().unwrap();
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
}

#[test]
fn can_queue_capacity_delivery_and_recovery() {
    if !in_namespace("can_queue_capacity_delivery_and_recovery") {
        return;
    }
    let mut sim = Running::start(robot(&mut Command::new(BINARY)), vec![]);
    let bus = bus();
    runtime().block_on(async {
        // Exceed one CAN receive visit while paused, then drain all replies.
        // This exercises readiness for frames left behind by the fairness budget.
        const BURST: u64 = 80;
        for _ in 0..BURST {
            bus.write_frame(&frame(0x7ff, QUERY)).unwrap();
        }
        for _ in 0..BURST {
            assert_eq!(bus.read_frame().unwrap().raw_id(), 0x17);
        }
        tc(&[
            "qdisc", "add", "dev", "bench", "root", "netem", "limit", "10",
        ]);
        // More than the configured TX capacity between advances, using small
        // batches and draining replies. Both socket buffers keep their defaults.
        for _ in 0..3 {
            for _ in 0..8 {
                bus.write_frame(&frame(0x7ff, QUERY)).unwrap();
            }
            for _ in 0..8 {
                assert_eq!(bus.read_frame().unwrap().raw_id(), 0x17);
            }
        }
        let delivered = sim.client.state().await.unwrap();
        assert_eq!(delivered.time_ns, 0);
        assert_eq!(
            (
                delivered.statistics.commands,
                delivered.statistics.replies,
                delivered.statistics.dropped
            ),
            (BURST + 24, BURST + 24, 0)
        );

        // Hold the same queue full to inject exhaustion, not USB/bus timing.
        tc(&[
            "qdisc", "change", "dev", "bench", "root", "netem", "limit", "10", "delay", "60s",
        ]);
        let filler = CanFdSocket::open("bench").unwrap();
        filler.set_filter_drop_all().unwrap();
        filler.set_loopback(false).unwrap();
        filler.set_nonblocking(true).unwrap();
        for _ in 0..9 {
            filler.write_frame(&frame(0x555, [0; 8])).unwrap();
        }
        // vcan loops this command to the simulator, but its reply has no TX slot.
        bus.write_frame(&frame(0x7ff, QUERY)).unwrap();
        assert_eq!(
            filler
                .write_frame(&frame(0x555, [0; 8]))
                .unwrap_err()
                .raw_os_error(),
            Some(libc::ENOBUFS)
        );
        let deadline = Instant::now() + Duration::from_secs(5);
        while sim.client.state().await.unwrap().statistics.dropped == 0 {
            assert!(
                Instant::now() < deadline,
                "simulator did not encounter the full queue"
            );
            tokio::task::yield_now().await;
        }
        bus.set_nonblocking(true).unwrap();
        assert_eq!(
            bus.read_frame().unwrap_err().kind(),
            std::io::ErrorKind::WouldBlock
        );
        tc(&["qdisc", "del", "dev", "bench", "root"]);
        bus.set_nonblocking(false).unwrap();
        exchange(&bus, 0x7ff, QUERY);
        let recovered = sim.client.state().await.unwrap().statistics;
        assert_eq!(
            (recovered.commands, recovered.replies, recovered.dropped),
            (BURST + 26, BURST + 25, 1)
        );
    });
    sim.stop();
}

#[test]
fn host_http_listener_reaches_private_namespace() {
    let mut command = namespace();
    command.arg(BINARY);
    let mut sim = Running::start(robot(&mut command), vec![]);
    assert_ne!(
        fs::read_link(format!("/proc/{}/ns/net", sim.child.id())).unwrap(),
        fs::read_link("/proc/self/ns/net").unwrap()
    );
    runtime().block_on(async {
        assert_eq!(sim.client.state().await.unwrap().state.len(), 16);
    });
    sim.stop();
}
