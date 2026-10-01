//! Client-to-server integration, including the shared wire types. Requires the
//! same private user/network namespaces, vcan and iproute2 as simulator tests.
use openarm_simulator_client_rs::{
    Client, Error, StatusCode,
    models::{Fault, MappingRanges, MotorStatus, Push},
};
use std::{
    io::{BufRead, BufReader},
    net::TcpListener,
    os::{fd::AsRawFd, unix::process::CommandExt},
    path::Path,
    process::{Child, Command, Stdio},
    time::Duration,
};

struct Process(Child);
impl Drop for Process {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

#[test]
fn control_api_against_simulator() {
    // Discover the binary through Cargo rather than assuming a target directory.
    let output = Command::new("cargo")
        .args([
            "build",
            "--locked",
            "--offline",
            "--message-format=json-render-diagnostics",
            "-p",
            "openarm-simulator-rs",
        ])
        .current_dir(Path::new(env!("CARGO_MANIFEST_DIR")).parent().unwrap())
        .output()
        .unwrap();
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let binary = String::from_utf8(output.stdout)
        .unwrap()
        .lines()
        .filter_map(|line| serde_json::from_str::<serde_json::Value>(line).ok())
        .find_map(|message| {
            (message["target"]["name"] == "openarm-simulator-rs")
                .then(|| message["executable"].as_str().map(str::to_owned))
                .flatten()
        })
        .expect("Cargo did not report simulator executable");
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let fd = listener.as_raw_fd();
    let url = format!("http://{}", listener.local_addr().unwrap());
    let mut command = Command::new("unshare");
    command.args(["--user", "--map-root-user", "--net", "sh", "-ec",
        "ip link set lo up; for bus in can0 can1; do ip link add \"$bus\" type vcan; ip link set \"$bus\" mtu 72 up; done; exec \"$@\"",
        "namespace", &binary, "--model", openarm_test_model::SCENE, "--http-fd", &fd.to_string()]);
    for key in [
        "LISTEN_FDS",
        "LISTEN_PID",
        "LISTEN_FDS_FIRST_FD",
        "OPENARM_SIMULATOR_CONFIG",
    ] {
        command.env_remove(key);
    }
    unsafe {
        command.pre_exec(move || {
            let flags = libc::fcntl(fd, libc::F_GETFD);
            if flags < 0 || libc::fcntl(fd, libc::F_SETFD, flags & !libc::FD_CLOEXEC) < 0 {
                return Err(std::io::Error::last_os_error());
            }
            Ok(())
        });
    }
    let mut process = Process(
        command
            .stdout(Stdio::piped())
            .stderr(Stdio::inherit())
            .spawn()
            .unwrap(),
    );
    drop(listener);
    let stdout = process.0.stdout.take().unwrap();
    let mut poller = libc::pollfd {
        fd: stdout.as_raw_fd(),
        events: libc::POLLIN,
        revents: 0,
    };
    assert_eq!(
        unsafe { libc::poll(&mut poller, 1, 30_000) },
        1,
        "startup timeout"
    );
    let mut line = String::new();
    BufReader::new(stdout).read_line(&mut line).unwrap();
    assert_eq!(line.trim(), format!("HTTP administration: {url}"));

    tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
        .unwrap()
        .block_on(async {
            let client = Client::new(&url).unwrap();
            let initial = client.state().await.unwrap();
            assert!(initial.paused);
            assert_eq!(initial.time_ns, 0);
            assert_eq!(initial.state["left_joint1"].status, MotorStatus::DISABLED);
            assert_eq!(
                initial.state["left_joint1"].ranges,
                MappingRanges {
                    pmax: 12.5,
                    vmax: 45.,
                    tmax: 54.
                }
            );
            assert_eq!(initial.state["left_joint1"].mos_temperature, 25);
            assert_eq!(initial.state["left_joint1"].rotor_temperature, 25);
            let configuration = client.configuration().await.unwrap();
            assert_eq!(initial.timestep_ns, configuration.configuration.timestep_ns);
            assert_eq!(client.pause().await.unwrap(), StatusCode::NO_CONTENT);
            assert_eq!(
                client
                    .advance(Duration::from_nanos(initial.timestep_ns - 1))
                    .await
                    .unwrap(),
                StatusCode::OK
            );
            assert_eq!(client.state().await.unwrap().statistics.steps, 0);
            client.advance(Duration::from_nanos(1)).await.unwrap();
            assert_eq!(client.state().await.unwrap().statistics.steps, 1);
            let fault = client
                .fault(
                    "left_joint1",
                    Fault {
                        status: Some(MotorStatus::UNDERVOLTAGE),
                        silent: Some(true),
                    },
                )
                .await
                .unwrap();
            assert_eq!(fault.state["left_joint1"].status, MotorStatus::UNDERVOLTAGE);
            assert!(fault.state["left_joint1"].silent);
            assert_eq!(fault.state["right_joint1"].status, MotorStatus::DISABLED);
            let unknown = client
                .fault(
                    "left_joint1",
                    Fault {
                        status: Some(MotorStatus(2)),
                        silent: None,
                    },
                )
                .await
                .unwrap();
            assert_eq!(unknown.state["left_joint1"].status, MotorStatus(2));
            let pushed = client
                .push(Push {
                    left: Some([0.1; 7]),
                    right: None,
                })
                .await
                .unwrap();
            assert_eq!(pushed.plant.applied_torque_nm.left, [0.1; 7]);
            assert_eq!(pushed.plant.applied_torque_nm.right, [0.; 7]);
            assert_eq!(client.reset().await.unwrap(), StatusCode::OK);
            assert_eq!(client.state().await.unwrap(), initial);
            assert_eq!(client.unpause().await.unwrap(), StatusCode::OK);
            assert_eq!(client.unpause().await.unwrap(), StatusCode::NO_CONTENT);
            assert!(matches!(
                client.advance(Duration::ZERO).await,
                Err(Error::Api {
                    status: StatusCode::CONFLICT,
                    ..
                })
            ));
            assert_eq!(client.pause().await.unwrap(), StatusCode::OK);
            assert_eq!(client.pause().await.unwrap(), StatusCode::NO_CONTENT);
            assert!(matches!(
                client.fault("right_joint0", Fault::default()).await,
                Err(Error::Api {
                    status: StatusCode::BAD_REQUEST,
                    ..
                })
            ));
            assert!(matches!(
                client.advance(Duration::MAX).await,
                Err(Error::InvalidRequest(_))
            ));
            assert!(matches!(
                client
                    .push(Push {
                        left: Some([f64::NAN; 7]),
                        right: None
                    })
                    .await,
                Err(Error::InvalidRequest(_))
            ));
            client.reset().await.unwrap();
            // Independent concurrent calls through the shared connection pool.
            let second = client.clone();
            let one = tokio::spawn(async move { second.state().await.unwrap() });
            assert_eq!(client.state().await.unwrap(), initial);
            assert_eq!(one.await.unwrap(), initial);
        });
    assert_eq!(
        unsafe { libc::kill(process.0.id() as i32, libc::SIGTERM) },
        0
    );
    assert!(process.0.wait().unwrap().success());
}
