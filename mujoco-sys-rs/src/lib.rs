//! Raw bindings to the MuJoCo SDK selected at build time (3.14.0 by default).
#![allow(
    non_camel_case_types,
    non_snake_case,
    non_upper_case_globals,
    clippy::approx_constant
)]
include!(concat!(env!("OUT_DIR"), "/mujoco.rs"));
