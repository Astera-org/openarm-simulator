//! Client-to-server integration, including the shared wire types. Requires the
//! same private user/network namespaces, vcan and iproute2 as simulator tests.
use openarm_simulator_client::{
    Client, Error, StatusCode,
    models::{
        AppliedForce, Attachment, BodyIndex, BodyPoint, Fault, MappingRanges, MotorStatus, Push,
        SiteIndex, Spring,
        uom::si::{
            angle::radian,
            angular_velocity::radian_per_second,
            f32::{Angle as Angle32, AngularVelocity as AngularVelocity32, Torque as Torque32},
            f64::{Force, Length, Torque, Velocity},
            force::newton,
            length::meter,
            thermodynamic_temperature::degree_celsius,
            torque::newton_meter,
            velocity::meter_per_second,
        },
    },
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
            "openarm-simulator",
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
            (message["target"]["name"] == "openarm-simulator")
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
    for key in ["LISTEN_FDS", "LISTEN_PID", "LISTEN_FDS_FIRST_FD"] {
        command.env_remove(key);
    }
    command.args(["--config", openarm_test_model::CONFIG]);
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
                    pmax: Angle32::new::<radian>(12.5),
                    vmax: AngularVelocity32::new::<radian_per_second>(45.),
                    tmax: Torque32::new::<newton_meter>(54.)
                }
            );
            assert_eq!(
                initial.state["left_joint1"]
                    .mos_temperature
                    .get::<degree_celsius>(),
                25.
            );
            assert_eq!(
                initial.state["left_joint1"]
                    .rotor_temperature
                    .get::<degree_celsius>(),
                25.
            );
            let configuration = client.configuration().await.unwrap();
            assert_eq!(
                configuration.configuration.integrator,
                openarm_simulator_client::models::Integrator::ImplicitFast
            );
            let names = client.names().await.unwrap();
            let joint = names.joints["openarm_left_joint7"];
            let point = BodyPoint {
                body: BodyIndex(0),
                position: [Length::new::<meter>(0.); 3].into(),
            };
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
                .push(Push::from([(joint, Torque::new::<newton_meter>(0.1))]))
                .await
                .unwrap();
            assert_eq!(
                pushed.plant.applied_torque,
                Push::from([(joint, Torque::new::<newton_meter>(0.1))])
            );
            assert_eq!(client.reset().await.unwrap(), StatusCode::OK);
            assert_eq!(client.names().await.unwrap(), names);
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
                    .push(Push::from([(joint, Torque::new::<newton_meter>(f64::NAN))]))
                    .await,
                Err(Error::InvalidRequest(_))
            ));
            client.reset().await.unwrap();
            let id = "load / #α";
            let mut spring = Spring {
                endpoints: [
                    Attachment::Site(names.sites["world_site"]),
                    Attachment::Body(point),
                ],
                rest_length: Length::new::<meter>(0.1),
                stiffness: Force::new::<newton>(1.) / Length::new::<meter>(1.),
                damping: Force::new::<newton>(0.2) / Velocity::new::<meter_per_second>(1.),
            };
            let mut force = AppliedForce {
                point: Attachment::Site(names.sites["world_site"]),
                force: [Force::new::<newton>(0.1); 3].into(),
                torque: [Torque::new::<newton_meter>(0.); 3].into(),
            };
            assert_eq!(
                client.put_spring(id, &spring).await.unwrap(),
                StatusCode::CREATED
            );
            spring.endpoints[0] = Attachment::Body(point);
            assert_eq!(client.springs().await.unwrap()[id], spring);
            assert_eq!(
                client.put_force(id, &force).await.unwrap(),
                StatusCode::CREATED
            );
            force.point = Attachment::Body(point);
            assert_eq!(client.forces().await.unwrap()[id], force);
            assert!(matches!(
                client
                    .push(Push::from([(
                        openarm_simulator_client::models::JointIndex(usize::MAX),
                        Torque::new::<newton_meter>(1.)
                    )]))
                    .await,
                Err(Error::Api {
                    status: StatusCode::BAD_REQUEST,
                    ..
                })
            ));
            force.point = Attachment::Body(BodyPoint {
                body: BodyIndex(initial.bodies.len()),
                ..point
            });
            assert!(matches!(
                client.put_force(id, &force).await,
                Err(Error::Api {
                    status: StatusCode::BAD_REQUEST,
                    ..
                })
            ));
            spring.endpoints[1] = Attachment::Site(SiteIndex(usize::MAX));
            assert!(matches!(
                client.put_spring(id, &spring).await,
                Err(Error::Api {
                    status: StatusCode::BAD_REQUEST,
                    ..
                })
            ));
            client.unpause().await.unwrap();
            force.force.x = Force::new::<newton>(0.2);
            force.point = Attachment::Body(BodyPoint {
                position: [0.5, 0., 0.].map(Length::new::<meter>).into(),
                ..point
            });
            spring.endpoints[1] = Attachment::Body(BodyPoint {
                position: [0.3, 0.4, 0.].map(Length::new::<meter>).into(),
                ..point
            });
            spring.rest_length = Length::new::<meter>(0.2);
            assert_eq!(
                client.put_force(id, &force).await.unwrap(),
                StatusCode::NO_CONTENT
            );
            assert_eq!(
                client.put_spring(id, &spring).await.unwrap(),
                StatusCode::NO_CONTENT
            );
            client.pause().await.unwrap();
            let state = client.state().await.unwrap();
            assert_eq!(client.force(id).await.unwrap(), force);
            assert_eq!(client.spring(id).await.unwrap(), spring);
            assert_eq!(state.springs[id].length, Length::new::<meter>(0.5));
            assert_eq!(
                state.springs[id].velocity,
                Velocity::new::<meter_per_second>(0.)
            );
            assert_eq!(
                client.delete_spring(id).await.unwrap(),
                StatusCode::NO_CONTENT
            );
            assert_eq!(
                client.delete_force(id).await.unwrap(),
                StatusCode::NO_CONTENT
            );
            assert_eq!(
                client.delete_force(id).await.unwrap(),
                StatusCode::NO_CONTENT
            );
            assert!(matches!(
                client.force(id).await,
                Err(Error::Api {
                    status: StatusCode::NOT_FOUND,
                    ..
                })
            ));
            client.reset().await.unwrap();
            assert_eq!(client.state().await.unwrap(), initial);
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
