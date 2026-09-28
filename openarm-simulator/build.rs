mod build_support;

use std::{env, path::PathBuf, process::Command};

fn sdk(output: &std::path::Path) -> PathBuf {
    let platform = format!(
        "{}-{}",
        env::var("CARGO_CFG_TARGET_OS").unwrap(),
        env::var("CARGO_CFG_TARGET_ARCH").unwrap()
    );
    let checksum = match platform.as_str() {
        "linux-x86_64" => "326f0da78a7767cc18fab7205c993a771b5f19eda913648f958ee3f625240944",
        "linux-aarch64" => "1bdbc32c310c6de38664bbd3bcb01f939abc70c1e9d6402ae3442664d1e0da4c",
        _ => panic!("no pinned MuJoCo SDK for {platform}; set MUJOCO_DIR"),
    };
    let source = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap());
    build_support::fetch(
        &build_support::cache(&source, output),
        &format!(
            "https://github.com/google-deepmind/mujoco/releases/download/3.14.0/mujoco-3.14.0-{platform}.tar.gz"
        ),
        checksum,
        &[
            "mujoco-3.14.0/include",
            "mujoco-3.14.0/lib",
            "mujoco-3.14.0/LICENSE",
            "mujoco-3.14.0/THIRD_PARTY_NOTICES",
        ],
    )
}

fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=build_support.rs");
    println!("cargo:rerun-if-env-changed=MUJOCO_DIR");
    let output = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    let directory = env::var_os("MUJOCO_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| sdk(&output))
        .canonicalize()
        .expect("MuJoCo SDK does not exist");
    let include = directory.join("include");
    let library_dir = directory.join("lib");
    let library = library_dir.join("libmujoco.so");
    assert!(
        library.is_file(),
        "MuJoCo SDK must provide lib/libmujoco.so"
    );
    println!("cargo:rustc-link-search=native={}", library_dir.display());
    println!("cargo:rustc-link-lib=mujoco");
    println!("cargo:rustc-link-arg=-Wl,-rpath,{}", library_dir.display());
    println!("cargo:rerun-if-changed={}", library.display());
    // Runtime-only libclang packages may lack Clang's builtin stddef.h. The
    // system C compiler already supplies the standard headers we need.
    let cc_headers = Command::new("cc")
        .arg("-print-file-name=include")
        .output()
        .unwrap();
    let cc_headers = String::from_utf8(cc_headers.stdout).unwrap();
    bindgen::Builder::default()
        .header(include.join("mujoco/mujoco.h").to_str().unwrap())
        .clang_arg(format!("-I{}", include.display()))
        .clang_arg("-isystem")
        .clang_arg(cc_headers.trim())
        .allowlist_function("(mj_|mjs_).*")
        .allowlist_type("(mj|mjt|mjs).*")
        .allowlist_var("mj.*")
        .prepend_enum_name(false)
        .derive_debug(false)
        .layout_tests(false)
        .parse_callbacks(Box::new(bindgen::CargoCallbacks::new()))
        .generate()
        .expect("could not generate MuJoCo bindings")
        .write_to_file(output.join("mujoco.rs"))
        .unwrap();
}
