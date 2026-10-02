use download_cache::{cache, fetch};
use std::{
    env, fs, io,
    path::{Path, PathBuf},
    process::Command,
};

fn stage(library_dir: &Path, out_dir: &Path) -> io::Result<()> {
    // Cargo exposes OUT_DIR rather than the final executable directory.
    let profile_dir = out_dir
        .ancestors()
        .find(|path| path.file_name().is_some_and(|name| name == "build"))
        .and_then(Path::parent)
        .expect("Cargo OUT_DIR must be below the profile's build directory");
    println!("cargo:rerun-if-changed={}", library_dir.display());
    for entry in fs::read_dir(library_dir)? {
        let entry = entry?;
        let name = entry.file_name();
        let filename = name.to_string_lossy();
        if filename != "libmujoco.so" && !filename.starts_with("libmujoco.so.") {
            continue;
        }
        let source = entry.path().canonicalize()?;
        for directory in [profile_dir.to_path_buf(), profile_dir.join("deps")] {
            fs::create_dir_all(&directory)?;
            let destination = directory.join(&name);
            let temporary = tempfile::tempdir_in(&directory)?;
            let staged = temporary.path().join(&name);
            fs::hard_link(&source, &staged).or_else(|_| fs::copy(&source, &staged).map(|_| ()))?;
            // Replace the directory entry without changing a cached or loaded inode.
            fs::rename(staged, &destination)?;
            println!("cargo:rerun-if-changed={}", destination.display());
        }
    }
    Ok(())
}

fn sdk() -> PathBuf {
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
    fetch(
        &cache("mujoco-sys-rs-build"),
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
    println!("cargo:rerun-if-env-changed=MUJOCO_DIR");
    let out_dir = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    let directory = env::var_os("MUJOCO_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(sdk)
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
    // This rpath applies only to this package's linked targets, not its dependents.
    println!("cargo:rustc-link-arg=-Wl,-rpath,$ORIGIN");
    println!("cargo:rerun-if-changed={}", library.display());
    stage(&library_dir, &out_dir).expect("stage MuJoCo shared library");
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
        .write_to_file(out_dir.join("mujoco.rs"))
        .unwrap();
}
