fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    // Cargo does not inherit dependency linker args. The simulator needs its own
    // rpath to find the MuJoCo library staged beside the executable ($ORIGIN).
    println!("cargo:rustc-link-arg=-Wl,-rpath,$ORIGIN");
}
