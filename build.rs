use std::{env, fs, path::PathBuf, process::Command};

fn main() {
    println!("cargo:rerun-if-env-changed=MUJOCO_DIR");
    let directory = PathBuf::from(
        env::var("MUJOCO_DIR").expect("set MUJOCO_DIR to the uv MuJoCo package directory"),
    );
    let include = directory.join("include");
    let output = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    let library = fs::read_dir(&directory)
        .unwrap()
        .map(|p| p.unwrap().path())
        .find(|p| {
            p.file_name()
                .unwrap()
                .to_string_lossy()
                .starts_with("libmujoco.so.")
        })
        .expect("MuJoCo shared library missing from MUJOCO_DIR");
    let link = output.join("libmujoco.so");
    // A moved project or replaced venv can leave a dangling symlink here.
    if link.symlink_metadata().is_ok() {
        fs::remove_file(&link).unwrap();
    }
    std::os::unix::fs::symlink(&library, &link).unwrap();
    println!("cargo:rustc-link-search=native={}", output.display());
    println!("cargo:rustc-link-lib=mujoco");
    println!("cargo:rustc-link-arg=-Wl,-rpath,{}", directory.display());
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
