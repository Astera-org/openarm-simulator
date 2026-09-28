mod friction;
mod http;
mod ipc;
mod physics;
mod protocol;
mod service;
mod sockets;
#[allow(
    clippy::approx_constant,
    non_camel_case_types,
    non_snake_case,
    non_upper_case_globals,
    dead_code,
    unused_imports
)]
mod ffi {
    include!(concat!(env!("OUT_DIR"), "/mujoco.rs"));
}

use anyhow::{Context, Result, ensure};
use clap::Parser;
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::{
    fs,
    io::Write,
    os::fd::RawFd,
    path::{Path, PathBuf},
    sync::{Arc, atomic::AtomicBool},
    thread,
};

#[derive(Parser)]
#[command(
    version,
    about = "OpenArm MuJoCo CAN motor simulator with HTTP administration",
    after_help = "Motor commands use CAN only. HTTP: GET /state, GET /configuration, POST /fault, /push, /reset.\nInherited descriptors must survive exec. HTTP also accepts LISTEN_FDS=1, LISTEN_PID, and LISTEN_FDS_FIRST_FD (default 3).\nBuild: cargo build --release (downloads pinned MuJoCo unless MUJOCO_DIR is set).\nTests: cargo test (also provisions the pinned model; requires Linux user/network namespaces, vcan and iproute2)."
)]
struct Args {
    /// External OpenArm v1 scene.xml, including its referenced meshes
    #[arg(long, env = "OPENARM_SIMULATOR_MODEL")]
    model: PathBuf,
    #[arg(long, default_value = "127.0.0.1")]
    host: String,
    #[arg(long, default_value_t = 8080)]
    port: u16,
    /// Inherited listening TCP socket; overrides host/port
    #[arg(long)]
    http_fd: Option<RawFd>,
    #[arg(long, default_value = "can1")]
    left_interface: String,
    #[arg(long, default_value = "can0")]
    right_interface: String,
    /// Inherited CAN_RAW socket bound to the left interface in this namespace
    #[arg(long)]
    left_can_fd: Option<RawFd>,
    /// Inherited CAN_RAW socket bound to the right interface in this namespace
    #[arg(long)]
    right_can_fd: Option<RawFd>,
    /// Inherited listening Unix SEQPACKET administration socket
    #[arg(long)]
    ipc_fd: Option<RawFd>,
    /// Readable pipe/socket; EOF stops the simulator
    #[arg(long)]
    parent_fd: Option<RawFd>,
    /// Startup plant JSON: poses, offsets, bodies, joints, friction_scale
    #[arg(long, env = "OPENARM_SIMULATOR_CONFIG")]
    config: Option<PathBuf>,
    /// Write simulator.json on shutdown
    #[arg(long)]
    report_dir: Option<PathBuf>,
    /// Optional Lab service runtime directory
    #[arg(long, requires = "parent_fd")]
    runtime: Option<PathBuf>,
}

fn read_json(path: &Path) -> Result<Value> {
    serde_json::from_reader(
        fs::File::open(path).with_context(|| format!("open {}", path.display()))?,
    )
    .with_context(|| format!("parse {}", path.display()))
}

fn configuration(runtime: &Value, path: Option<&Path>) -> Result<physics::Config> {
    let value = match path {
        Some(path) => read_json(path)?,
        None => runtime
            .get("simulator")
            .cloned()
            .unwrap_or_else(|| json!({})),
    };
    ensure!(value.is_object(), "Simulator config must be a JSON object");
    for field in ["poses", "offsets"] {
        if let Some(arms) = value.get(field) {
            ensure!(arms.is_object(), "{field} must be an object keyed by arm");
        }
    }
    Ok(serde_json::from_value(value)?)
}

fn identity(physics: &physics::Physics, model: &Path) -> Result<Value> {
    let mut configuration = physics.configuration();
    configuration["model_sha256"] = physics::model_sha256(model)?.into();
    let hash = format!("{:x}", Sha256::digest(serde_json::to_vec(&configuration)?));
    Ok(json!({"configuration": configuration, "config_sha256": hash}))
}

fn save_report(directory: &Path, model: &Path, identity: &Value, state: Value) -> Result<()> {
    let mut report = state;
    let report = report.as_object_mut().unwrap();
    let final_state = report.remove("state").unwrap();
    report.insert("final_state".into(), final_state);
    report.insert("backend".into(), "mujoco-socketcan".into());
    report.insert("engine".into(), "rust".into());
    report.insert("model".into(), json!(model));
    report.insert(
        "model_sha256".into(),
        identity["configuration"]["model_sha256"].clone(),
    );
    report.insert("configuration".into(), identity["configuration"].clone());
    report.insert(
        "assumptions".into(),
        json!([
            "Nominal XML plus recorded assembly/friction overrides; synthetic parameters",
            "Stiff numerical joint stops; compliance is not measured hardware compliance",
            "Ideal MIT current/torque response; no firmware electrical/thermal model",
            "Linear symmetric gripper transmission; overlapping finger meshes excluded",
            "TIMEOUT=0; no emulated firmware watchdog",
            "Virtual CAN has no automatic USB/arbitration delay model"
        ]),
    );
    fs::create_dir_all(directory)?;
    ipc::write_json(&directory.join("simulator.json"), &json!(report))
}

fn main() -> Result<()> {
    let args = Args::parse();
    let http_fd = sockets::activation_fd(args.http_fd)?;
    sockets::validate(&[
        http_fd,
        args.left_can_fd,
        args.right_can_fd,
        args.ipc_fd,
        args.parent_fd,
    ])?;
    let parent = args.parent_fd.map(sockets::parent).transpose()?;
    let listener = sockets::http(&args.host, args.port, http_fd)?;
    let address = listener.local_addr()?;
    let ipc = ipc::listen(args.runtime.as_deref(), args.ipc_fd)?;
    let runtime = args
        .runtime
        .as_ref()
        .map(|path| read_json(&path.join("config.json")))
        .transpose()?
        .unwrap_or_else(|| json!({}));
    let config = configuration(&runtime, args.config.as_deref())?;
    let model = args.model.canonicalize().context("external model path")?;
    ensure!(model.is_file(), "model must be an MJCF file");
    // Internal motor indexes retain their original right/left mapping.
    let interfaces = [args.right_interface, args.left_interface];
    let buses = service::can_sockets(&interfaces, [args.right_can_fd, args.left_can_fd])?;
    let mut physics = physics::Physics::load(&model, config)?;
    let identity = identity(&physics, &model)?;
    let stopped = Arc::new(AtomicBool::new(false));
    for signal in [signal_hook::consts::SIGINT, signal_hook::consts::SIGTERM] {
        signal_hook::flag::register(signal, Arc::clone(&stopped))?;
    }
    if let Some(runtime) = &args.runtime {
        ipc::register(runtime, ipc.as_ref().unwrap(), &interfaces, &identity)?;
    }
    let (control, calls) = service::Control::channel();
    if let Some(ipc) = &ipc {
        let socket = ipc.socket.try_clone()?;
        let control = control.clone();
        thread::Builder::new()
            .name("lab-ipc".into())
            .spawn(move || ipc::run(socket, control))?;
    }
    http::start(listener, control)?;
    println!("HTTP administration: http://{address}");
    std::io::stdout().flush()?;
    let state = service::run(&mut physics, calls, buses, parent, &stopped, &identity)?;
    if let Some(directory) = args
        .report_dir
        .or_else(|| runtime["report_dir"].as_str().map(PathBuf::from))
    {
        save_report(&directory, &model, &identity, state)?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn configuration_file_replaces_runtime_and_rejects_unknown_fields() {
        let runtime = json!({"simulator": {"friction_scale": 0.5}});
        assert_eq!(configuration(&runtime, None).unwrap().friction_scale, 0.5);
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("plant.json");
        fs::write(&path, "{\"joints\":{}}").unwrap();
        assert_eq!(
            configuration(&runtime, Some(&path)).unwrap().friction_scale,
            1.
        );
        for value in ["[]", "{\"unknown\":1}", "{\"friction_scale\":NaN}"] {
            fs::write(&path, value).unwrap();
            assert!(configuration(&runtime, Some(&path)).is_err());
        }
    }

    #[test]
    fn model_hash_tracks_included_geometry_and_resolved_plant_identity() {
        let dir = tempfile::tempdir().unwrap();
        let scene = dir.path().join("scene.xml");
        let body = dir.path().join("body.xml");
        fs::write(&scene, "<mujoco><include file=\"body.xml\"/></mujoco>").unwrap();
        fs::write(
            &body,
            "<mujoco><worldbody><geom type=\"sphere\" size=\"1\"/></worldbody></mujoco>",
        )
        .unwrap();
        let original = physics::model_sha256(&scene).unwrap();
        let renamed = dir.path().join("renamed.xml");
        fs::copy(&scene, &renamed).unwrap();
        assert_eq!(original, physics::model_sha256(&renamed).unwrap());
        fs::write(
            &body,
            "<mujoco><worldbody><geom type=\"sphere\" size=\"2\"/></worldbody></mujoco>",
        )
        .unwrap();
        assert_ne!(original, physics::model_sha256(&scene).unwrap());
        let model = PathBuf::from(openarm_test_model::SCENE);
        let mut physics = physics::Physics::load(&model, physics::Config::default()).unwrap();
        let original = identity(&physics, &model).unwrap();
        let mut config: physics::Config = serde_json::from_value(json!({
            "joints": original["configuration"]["joints"],
            "poses": {"right": [0.,0.,0.,0.,0.,0.,0.1,0.]}
        }))
        .unwrap();
        let equivalent = physics::Physics::load(&model, config).unwrap();
        assert_eq!(original, identity(&equivalent, &model).unwrap());
        config = serde_json::from_value(json!({"offsets": {"right": ([0.01; 8])}})).unwrap();
        let changed = physics::Physics::load(&model, config).unwrap();
        assert_ne!(original, identity(&changed, &model).unwrap());
        physics.push([[0.1; 7]; 2]).unwrap();
        physics.step(2).unwrap();
        assert_eq!(original, identity(&physics, &model).unwrap());
    }
}
