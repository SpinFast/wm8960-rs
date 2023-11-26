#[cfg(feature = "codegen")]
include!(concat!(env!("OUT_DIR"), "/regmap.rs"));
#[cfg(feature = "codgen")]
include!(concat!(env!("OUT_DIR"), "/reset.rs"));

#[cfg(not(feature = "codegen"))]
include!(concat!("generated/regmap.rs"));
#[cfg(not(feature = "codgen"))]
include!(concat!("generated/reset.rs"));
