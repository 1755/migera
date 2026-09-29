//! Pure, ECS-free math for the rotation-space animation stack.
//!
//! Every function here is a plain function over plain values — no `World`,
//! no `Component`, no Bevy schedule. That is deliberate: it makes the whole
//! numerical core unit-testable in milliseconds without standing up an app,
//! which is the discipline `src/hybrid`'s own `*_ref.rs` modules already
//! established for this project's shader math.
//!
//! - [`quat_ext`] — exp/log maps and the neighbourhooding guards. Read its
//!   module doc before touching any quaternion difference anywhere.
//! - [`spring`] — closed-form damped harmonic oscillators (Stage 1).
//! - [`inertialize`] — velocity-preserving transitions, replacing the
//!   crossfade (Stage 1 transitions, Stage 3 foot locking).

pub mod ik;
pub mod inertialize;
pub mod pd;
pub mod quat_ext;
pub mod spring;

pub use inertialize::{
    decay_exponential, decay_exponential_vec3, halflife_to_decay_rate, InertializeCubic,
    Inertializer, RotationInertializer,
};
pub use quat_ext::{
    canonical, from_scaled_angle_axis, neighborhood, quat_exp, quat_log, rotation_delta,
    to_scaled_angle_axis,
};
pub use spring::{spring_scalar, spring_vec3, SpringParams};
