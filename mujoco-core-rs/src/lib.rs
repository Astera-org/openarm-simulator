#![deny(unsafe_code)]
//! MuJoCo types, without a native engine dependency.
//! Indices belong to one compiled model and remain valid across data resets.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "lowercase"))]
pub enum JointKind {
    Free,
    Ball,
    Slide,
    Hinge,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Integrator {
    Euler,
    #[cfg_attr(feature = "serde", serde(rename = "RK4"))]
    Rk4,
    #[cfg_attr(feature = "serde", serde(rename = "implicit"))]
    Implicit,
    #[cfg_attr(feature = "serde", serde(rename = "implicitfast"))]
    ImplicitFast,
    #[cfg_attr(feature = "serde", serde(rename = "discrete"))]
    Discrete,
}

#[derive(Clone, Copy)]
pub enum Object {
    Body,
    Joint,
    Actuator,
    Geom,
    Site,
}

pub trait ObjectIndex: Copy + From<usize> + Into<usize> {
    const OBJECT: Object;
}

macro_rules! indices {
    ($($name:ident => $kind:ident),* $(,)?) => {$(
        #[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
        #[repr(transparent)]
        #[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
        #[cfg_attr(feature = "serde", serde(transparent))]
        pub struct $name(pub usize);

        impl ObjectIndex for $name {
            const OBJECT: Object = Object::$kind;
        }
        impl From<usize> for $name {
            fn from(index: usize) -> Self { Self(index) }
        }
        impl From<$name> for usize {
            fn from(index: $name) -> Self { index.0 }
        }
        impl std::fmt::Display for $name {
            fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                self.0.fmt(f)
            }
        }
    )*};
}

indices! {
    BodyIndex => Body,
    JointIndex => Joint,
    ActuatorIndex => Actuator,
    GeomIndex => Geom,
    SiteIndex => Site,
}
