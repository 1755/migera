//! Third-person gameplay camera.
//!
//! A camera is one rig run as a fixed pipeline of small stages, each a pure
//! function over plain structs that owns specific degrees of freedom:
//!
//! ```text
//! input → stack (mode blend) → anchor (pivot) → orbit (yaw/pitch/zoom)
//!       → rig (eye pose) → [collision] → [effects] → write
//! ```
//!
//! - [`stack`]: the push-only stack of camera modes. Modes are data
//!   ([`rig::ModeParams`] keyed by [`stack::ModeId`]), not an enum.
//! - [`anchor`]: the pivot follows the target with an exactly-integrated
//!   spring, a leash, an air deadband, and inertialized anchor hand-offs.
//! - [`orbit`]: yaw/pitch as scalars; goals, the player and auto-recentre,
//!   one owner per axis.
//! - [`rig`]: distance/height/FOV as curves of pitch, and the eye pose.
//! - [`pipeline`]: the stages chained over plain data, for tests and replay.
//! - [`trace`] and [`harness`]: record/replay and scenario metrics.
//!
//! Conventions: yaw is about world +Y and 0 looks along −Z (the walker's);
//! pitch is elevation, positive with the camera above looking down. Look
//! input `x` positive looks right, `y` positive looks up. Follow springs run
//! on virtual time, everything else on real time ([`clock`]).
//!
//! The design and its reasons: `docs/knowledge/gameplay-camera/`.
//! The roadmap and test gates: `CAMERA_PROGRESS.md`.

pub mod anchor;
pub mod bridge;
pub mod clock;
pub mod components;
pub mod device;
pub mod harness;
pub mod input;
pub mod orbit;
pub mod pipeline;
pub mod plugin;
pub mod rig;
pub mod stack;
pub mod trace;

pub use anchor::{AnchorParams, PivotState};
pub use clock::CameraClock;
pub use components::{
    CameraDesiredPose, CameraGoals, CameraModeRequests, CameraRecorder, CameraRigState,
    CameraTarget, CameraTargetState, CameraView, ThirdPersonCamera,
};
pub use device::{CameraDeviceInput, MouseLook};
pub use plugin::{CameraSet, CameraTargetSources, ThirdPersonCameraPlugin};
pub use input::{CameraInput, CameraInputSettings};
pub use orbit::{OrbitGoal, OrbitParams, OrbitState};
pub use pipeline::{CameraConfig, CameraFrame, CameraOutput, CameraRig, TargetSample};
pub use rig::{DesiredPose, ModeParams, PitchCurve};
pub use stack::{CameraStack, ModeId, ModeRequest};
pub use trace::{CameraTrace, GoldenTrace};
