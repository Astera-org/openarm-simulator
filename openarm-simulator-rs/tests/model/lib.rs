//! Model fixture provisioned only when Cargo builds development dependencies.
pub const SCENE: &str = env!("OPENARM_TEST_MODEL");
pub const CONFIG: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../config/openarm-v1.json");
