mod codec;
mod experiment;
mod friction;
mod physics;
mod protocol;
mod service;
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
fn main() -> anyhow::Result<()> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    match args.as_slice() {
        [mode] if mode == "codec" => codec::run(),
        [mode, model, config] if mode == "experiment" => {
            let physics =
                physics::Physics::load(std::path::Path::new(model), serde_json::from_str(config)?)?;
            experiment::run(physics)
        }
        [mode, fd, model, config] if mode == "serve" => {
            service::require_virtual_interfaces()?;
            let fd: i32 = fd.parse()?;
            anyhow::ensure!(fd >= 3, "expected inherited administrative descriptor");
            let admin = unsafe { service::Admin::from_fd(fd) };
            let physics =
                physics::Physics::load(std::path::Path::new(model), serde_json::from_str(config)?)?;
            service::run(physics, admin)
        }
        _ => {
            anyhow::bail!(
                "Internal simulator executable; launch with openarm-simulator --model PATH"
            )
        }
    }
}
