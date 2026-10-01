mod clock;
mod friction;
mod http;
mod motor;
mod physics;
mod service;
mod simulation;
mod sockets;
use anyhow::{Context, Result, ensure};
use clap::Parser;
use serde_json::{Value, json};
use std::{
    collections::BTreeMap,
    fs,
    io::Write,
    os::fd::RawFd,
    path::{Path, PathBuf},
    sync::{Arc, atomic::AtomicBool},
};

#[derive(Parser)]
#[command(
    version,
    about = "OpenArm MuJoCo CAN motor simulator with HTTP administration",
    after_help = "Motor commands use CAN only. HTTP: GET /state, GET /configuration, POST /fault, /push, /reset, /pause, /unpause, /advance.\nClock starts paused; reset restores startup state and pauses. POST /advance accepts {\"duration_ns\": <integer>} while paused. Clock mutations return empty 200 responses, or 204 for an unchanged pause/unpause; 409 means clock state conflict.\nInherited descriptors must survive exec. HTTP also accepts LISTEN_FDS=1, LISTEN_PID, and LISTEN_FDS_FIRST_FD (default 3).\nBuild: cargo build --release (downloads pinned MuJoCo unless MUJOCO_DIR is set).\nTests: cargo test (also provisions the pinned model; requires Linux user/network namespaces, vcan and iproute2)."
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
    /// Override a configured bus interface: --can-interface BUS=INTERFACE (repeatable)
    #[arg(long, value_parser = assignment)]
    can_interface: Vec<(String, String)>,
    /// Inherited CAN_RAW socket for a configured bus: --can-fd BUS=FD (repeatable)
    #[arg(long, value_parser = fd_assignment)]
    can_fd: Vec<(String, RawFd)>,
    /// Readable pipe/socket; EOF stops the simulator
    #[arg(long)]
    parent_fd: Option<RawFd>,
    /// Startup JSON: timestep_ns (default 500000), poses, offsets, bodies, joints, friction_scale
    #[arg(long, env = "OPENARM_SIMULATOR_CONFIG")]
    config: Option<PathBuf>,
    /// Write simulator.json on shutdown
    #[arg(long)]
    report_dir: Option<PathBuf>,
}

fn assignment(value: &str) -> std::result::Result<(String, String), String> {
    let (name, value) = value.split_once('=').ok_or("expected BUS=VALUE")?;
    if name.is_empty() || value.is_empty() {
        return Err("expected nonempty BUS=VALUE".into());
    }
    Ok((name.into(), value.into()))
}
fn fd_assignment(value: &str) -> std::result::Result<(String, RawFd), String> {
    let (name, value) = assignment(value)?;
    Ok((name, value.parse().map_err(|_| "expected BUS=FD")?))
}

fn read_json(path: &Path) -> Result<Value> {
    serde_json::from_reader(
        fs::File::open(path).with_context(|| format!("open {}", path.display()))?,
    )
    .with_context(|| format!("parse {}", path.display()))
}

fn configuration(path: Option<&Path>) -> Result<physics::Config> {
    let value = match path {
        Some(path) => read_json(path)?,
        None => json!({}),
    };
    ensure!(value.is_object(), "Simulator config must be a JSON object");
    for field in ["poses", "offsets"] {
        if let Some(arms) = value.get(field) {
            ensure!(arms.is_object(), "{field} must be an object keyed by arm");
        }
    }
    Ok(serde_json::from_value(value)?)
}

fn main() -> Result<()> {
    let args = Args::parse();
    let http_fd = sockets::activation_fd(args.http_fd)?;
    let mut descriptors = vec![http_fd, args.parent_fd];
    descriptors.extend(args.can_fd.iter().map(|(_, fd)| Some(*fd)));
    sockets::validate(&descriptors)?;
    let parent = args.parent_fd.map(sockets::parent).transpose()?;
    let listener = sockets::http(&args.host, args.port, http_fd)?;
    let address = listener.local_addr()?;
    let mut config = configuration(args.config.as_deref())?;
    let mut overrides = std::collections::BTreeSet::new();
    for (bus, interface) in args.can_interface {
        ensure!(
            overrides.insert(bus.clone()),
            "duplicate bus override {bus}"
        );
        *config
            .buses
            .get_mut(&bus)
            .with_context(|| format!("unknown bus {bus}"))? = interface;
    }
    let mut can_fds = BTreeMap::new();
    for (bus, fd) in args.can_fd {
        ensure!(config.buses.contains_key(&bus), "unknown bus {bus}");
        ensure!(
            can_fds.insert(bus.clone(), fd).is_none(),
            "duplicate bus descriptor {bus}"
        );
    }
    let interfaces: Vec<_> = config.buses.values().cloned().collect();
    let fds: Vec<_> = config
        .buses
        .keys()
        .map(|bus| can_fds.get(bus).copied())
        .collect();
    let model = args.model.canonicalize().context("external model path")?;
    ensure!(model.is_file(), "model must be an MJCF file");
    let mut physics = simulation::Simulation::load(&model, config)?;
    let buses = service::can_sockets(&interfaces, &fds, &physics)?;
    let (control, calls) = service::Control::channel()?;
    let stopped = Arc::new(AtomicBool::new(false));
    for signal in [signal_hook::consts::SIGINT, signal_hook::consts::SIGTERM] {
        signal_hook::flag::register(signal, Arc::clone(&stopped))?;
        control.wake_on_signal(signal)?;
    }
    http::start(listener, control)?;
    println!("HTTP administration: http://{address}");
    std::io::stdout().flush()?;
    let state = service::run(&mut physics, calls, buses, parent, &stopped)?;

    if let Some(directory) = args.report_dir {
        fs::create_dir_all(&directory)?;
        fs::write(
            directory.join("simulator.json"),
            serde_json::to_vec_pretty(&state)?,
        )?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn configuration_file_and_unknown_fields() {
        assert_eq!(configuration(None).unwrap().friction_scale, 1.);
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("plant.json");
        fs::write(&path, "{\"joints\":{}}").unwrap();
        assert_eq!(configuration(Some(&path)).unwrap().friction_scale, 1.);
        for value in ["[]", "{\"unknown\":1}", "{\"friction_scale\":NaN}"] {
            fs::write(&path, value).unwrap();
            assert!(configuration(Some(&path)).is_err());
        }
    }
}
