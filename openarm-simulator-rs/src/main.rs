mod config;
mod physics;
mod runtime;
mod simulation;
use anyhow::{Context, Result, ensure};
use clap::Parser;
use hyper::header::HeaderValue;
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
    about = "Test robot control software against simulated CAN motors and MuJoCo physics",
    after_help = r#"The simulator emulates Damiao motor controllers so your control software can
communicate over CAN as it does with a robot. The HTTP API lets you inspect state,
control simulated time, inject motor faults, and apply forces or springs.

Supply a MuJoCo scene with --model. Use --config to define the simulated motors
and their CAN interfaces.

The simulation starts paused. Use the HTTP API to unpause for real-time execution
or advance time explicitly while paused. Reset restores the startup state and
pauses again.

Rust and Python clients are included. Web applications can connect directly when
their origin is permitted with --allow-origin. See README.md for HTTP usage."#
)]
struct Args {
    /// MuJoCo scene to simulate (MJCF file)
    #[arg(long)]
    model: PathBuf,
    /// Listen address for the HTTP control API
    #[arg(long, default_value = "127.0.0.1")]
    host: String,
    /// Listen port for the HTTP control API
    #[arg(long, default_value_t = 8080)]
    port: u16,
    /// Allowed web origin (repeatable), or '*' for any origin; none by default
    #[arg(long)]
    allow_origin: Vec<HeaderValue>,
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
    /// JSON configuration for motors, CAN buses, and physics
    #[arg(long)]
    config: Option<PathBuf>,
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

fn configuration(path: Option<&Path>) -> Result<config::Config> {
    let value = match path {
        Some(path) => read_json(path)?,
        None => json!({}),
    };
    ensure!(value.is_object(), "Simulator config must be a JSON object");
    Ok(serde_json::from_value(value)?)
}

fn main() -> Result<()> {
    let args = Args::parse();
    let http_fd = runtime::activation_fd(args.http_fd)?;
    let mut descriptors = vec![http_fd, args.parent_fd];
    descriptors.extend(args.can_fd.iter().map(|(_, fd)| Some(*fd)));
    runtime::validate_fds(&descriptors)?;
    let parent = args.parent_fd.map(runtime::parent_fd).transpose()?;
    let listener = runtime::http_listener(&args.host, args.port, http_fd)?;
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
    let mut simulation = simulation::Simulation::load(&model, config)?;
    let buses = runtime::can_sockets(&interfaces, &fds, &simulation)?;
    let (control, calls) = runtime::Control::channel()?;
    let stopped = Arc::new(AtomicBool::new(false));
    for signal in [signal_hook::consts::SIGINT, signal_hook::consts::SIGTERM] {
        signal_hook::flag::register(signal, Arc::clone(&stopped))?;
        control.wake_on_signal(signal)?;
    }
    runtime::start_http(listener, control, args.allow_origin)?;
    println!("HTTP administration: http://{address}");
    std::io::stdout().flush()?;
    runtime::run(&mut simulation, calls, buses, parent, &stopped)?;

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
        for value in [
            "[]",
            "{\"unknown\":1}",
            "{\"positions\":{\"joint\":0.25}}",
            "{\"friction_scale\":NaN}",
        ] {
            fs::write(&path, value).unwrap();
            assert!(configuration(Some(&path)).is_err());
        }
    }
}
