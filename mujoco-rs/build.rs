fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    // Cargo does not inherit the sys crate's linker args. Our test executables
    // need their own rpath to find the staged MuJoCo library beside them ($ORIGIN).
    println!("cargo:rustc-link-arg=-Wl,-rpath,$ORIGIN");
}
