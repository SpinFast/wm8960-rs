#[cfg(feature = "codgen")]
include!(concat!(env!("OUT_DIR"), "/common.rs"));

#[cfg(not(feature = "codegen"))]
include!(concat!("generated/common.rs"));


