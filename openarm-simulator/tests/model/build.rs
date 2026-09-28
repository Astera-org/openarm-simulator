#[path = "../../build_support.rs"]
mod build_support;

use std::{env, path::PathBuf};

fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=../../build_support.rs");
    println!("cargo:rerun-if-env-changed=OPENARM_SIMULATOR_MODEL");
    let source = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap());
    let scene = env::var_os("OPENARM_SIMULATOR_MODEL")
        .map(|path| source.join("../../..").join(path))
        .unwrap_or_else(|| {
            let output = PathBuf::from(env::var_os("OUT_DIR").unwrap());
            let revision = "56e846b34d8a5bcea1bcebf93db5dc9da467d3c8";
            let prefix = format!("openarm_mujoco-{revision}");
            build_support::fetch(
                &build_support::cache(&source, &output),
                &format!("https://codeload.github.com/enactic/openarm_mujoco/tar.gz/{revision}"),
                "bfbbfe18490bfc27ba59a03985610fdbdf30fce2b35f1d2c84c5940c34925ebb",
                &[&format!("{prefix}/v1"), &format!("{prefix}/LICENSE")],
            )
            .join("v1/scene.xml")
        })
        .canonicalize()
        .expect("test model scene.xml does not exist");
    println!("cargo:rustc-env=OPENARM_TEST_MODEL={}", scene.display());
}
