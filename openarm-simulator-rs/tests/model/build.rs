use download_cache::{cache, fetch};
use std::{env, path::PathBuf};

fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-env-changed=OPENARM_SIMULATOR_MODEL");
    let source = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap());
    let external = env::var_os("OPENARM_SIMULATOR_MODEL").is_some();
    let scene = env::var_os("OPENARM_SIMULATOR_MODEL")
        .map(|path| source.join("../../..").join(path))
        .unwrap_or_else(|| {
            let revision = "56e846b34d8a5bcea1bcebf93db5dc9da467d3c8";
            let prefix = format!("openarm_mujoco-{revision}");
            fetch(
                &cache("openarm-simulator-test-model"),
                &format!("https://codeload.github.com/enactic/openarm_mujoco/tar.gz/{revision}"),
                "bfbbfe18490bfc27ba59a03985610fdbdf30fce2b35f1d2c84c5940c34925ebb",
                &[&format!("{prefix}/v1"), &format!("{prefix}/LICENSE")],
            )
            .join("v1/scene.xml")
        })
        .canonicalize()
        .expect("test model scene.xml does not exist");
    println!("cargo:rerun-if-changed={}", scene.display());
    let definition = source.join("../../models/openarm-v1.xml");
    println!("cargo:rerun-if-changed={}", definition.display());
    let scene = if external {
        scene
    } else {
        let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
        let assets = scene.parent().unwrap().join("meshes");
        let xml = std::fs::read_to_string(&definition).unwrap().replace(
            "meshdir=\"meshes\"",
            &format!(
                "meshdir=\"{}\"",
                assets
                    .display()
                    .to_string()
                    .replace('&', "&amp;")
                    .replace('\"', "&quot;")
            ),
        );
        std::fs::write(out.join("openarm_bimanual.xml"), xml).unwrap();
        std::fs::copy(&scene, out.join("scene.xml")).unwrap();
        out.join("scene.xml")
    };
    println!("cargo:rustc-env=OPENARM_TEST_MODEL={}", scene.display());
}
