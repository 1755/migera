//! The pure camera pipeline: one frame from target sample and input to a
//! desired camera pose, with no `World`.
//!
//! Stage order is fixed: stack blend → anchor → orbit → rig. Collision and
//! effects (later phases) run after the rig, on the blended pose. The ECS
//! systems call the same stage functions; this type exists so tests, trace
//! replay and the bench can drive the whole camera from plain data.

use super::anchor::{AnchorInput, AnchorParams, PivotState};
use super::clock::CameraClock;
use super::input::{CameraInput, CameraInputSettings};
use super::orbit::{OrbitContext, OrbitGoal, OrbitParams, OrbitState};
use super::rig::{desired_pose, yaw_of, DesiredPose, ModeParams};
use super::stack::{CameraStack, ModeId, ModeRequest};
use bevy::prelude::*;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

/// What the camera reads about its target in one frame.
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize, Reflect)]
pub struct TargetSample {
    /// The target's root position, world space.
    pub position: Vec3,
    pub grounded: bool,
    /// Camera yaw that looks the way the target faces, if known.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub facing_yaw: Option<f32>,
    /// The target changed (an anchor hand-off) this frame.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub rebase: bool,
    /// This target's own pivot offset (eye height), overriding the
    /// config's.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub pivot_offset: Option<Vec3>,
}

/// One frame of everything the pipeline consumes: the unit a trace records.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct CameraFrame {
    pub clock: CameraClock,
    #[serde(default, skip_serializing_if = "is_default")]
    pub input: CameraInput,
    pub target: TargetSample,
    /// Mode switches requested this frame, applied in order.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub requests: Vec<ModeRequest>,
    /// Orbit goals active this frame (lock-on, dialog, assist).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub goals: Vec<OrbitGoal>,
}

fn is_default<T: Default + PartialEq>(value: &T) -> bool {
    *value == T::default()
}

/// Everything a designer tunes, minus per-player input settings.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Reflect)]
pub struct CameraConfig {
    pub modes: BTreeMap<ModeId, ModeParams>,
    pub base_mode: ModeId,
    pub anchor: AnchorParams,
    pub orbit: OrbitParams,
    /// The pivot's offset from the target's root, world space: eye height.
    pub pivot_offset: Vec3,
}

impl Default for CameraConfig {
    fn default() -> Self {
        let mut modes = BTreeMap::new();
        modes.insert(ModeId::new("explore"), ModeParams::exploration());
        modes.insert(ModeId::new("combat"), ModeParams::combat());
        Self {
            modes,
            base_mode: ModeId::new("explore"),
            anchor: AnchorParams::default(),
            orbit: OrbitParams::default(),
            pivot_offset: Vec3::new(0.0, 1.6, 0.0),
        }
    }
}

impl CameraConfig {
    fn mode(&self, id: &ModeId) -> ModeParams {
        self.modes.get(id).cloned().unwrap_or_else(|| {
            // An unknown mode falls back to the base so a typo in a request
            // degrades instead of panicking; P3's profile validation names it.
            self.modes.get(&self.base_mode).cloned().unwrap_or_else(ModeParams::exploration)
        })
    }
}

/// The output of one pipeline frame.
#[derive(Debug, Clone, Copy, PartialEq, Reflect)]
pub struct CameraOutput {
    pub pose: DesiredPose,
    pub yaw: f32,
    pub pitch: f32,
    /// The camera snapped this frame (first frame, teleport).
    pub cut: bool,
}

/// One camera's whole state.
#[derive(Debug, Clone, PartialEq, Reflect)]
pub struct CameraRig {
    pub pivot: PivotState,
    pub orbit: OrbitState,
    pub stack: CameraStack,
    /// Target position last frame, for its velocity.
    last_target: Option<Vec3>,
}

impl CameraRig {
    pub fn new(config: &CameraConfig, yaw: f32) -> Self {
        let base = config.mode(&config.base_mode);
        let pitch = base.orbit_shape().pitch_default;
        Self {
            pivot: PivotState::default(),
            orbit: OrbitState::new(yaw, pitch),
            stack: CameraStack::new(config.base_mode.clone(), base),
            last_target: None,
        }
    }

    pub fn step(
        &mut self,
        frame: &CameraFrame,
        settings: &CameraInputSettings,
        config: &CameraConfig,
    ) -> CameraOutput {
        let clock = frame.clock;
        // Advance existing blends over the frame, then apply this frame's
        // requests: a request arriving during a frame starts its blend at
        // the frame's end. Pushing first would credit a new mode with a frame
        // of blend it never had, by a different amount at each frame rate.
        self.stack.step(clock.real_dt);
        for request in &frame.requests {
            self.stack.push(request.id.clone(), config.mode(&request.id), request.blend);
        }

        let target = frame.target;
        let anchor = self.pivot.step(
            &AnchorInput {
                goal: target.position + target.pivot_offset.unwrap_or(config.pivot_offset),
                grounded: target.grounded,
                rebase: target.rebase,
            },
            &config.anchor,
            clock.virtual_dt,
        );

        // Heading from the target's own motion over the virtual frame.
        let velocity = match self.last_target {
            Some(last) if clock.virtual_dt > 0.0 && !anchor.cut && !target.rebase => {
                (target.position - last) / clock.virtual_dt
            }
            _ => Vec3::ZERO,
        };
        self.last_target = Some(target.position);
        let flat = Vec3::new(velocity.x, 0.0, velocity.z);
        let speed = flat.length();
        let context = OrbitContext {
            heading: (speed > 0.1).then(|| yaw_of(flat)),
            speed,
            facing: target.facing_yaw,
            goals: &frame.goals,
        };

        let orbit_shape = self.stack.orbit_shape();
        self.orbit.step(
            &frame.input,
            settings,
            &orbit_shape,
            &config.orbit,
            &context,
            clock.real_dt,
        );

        let (yaw, pitch) = (self.orbit.yaw(), self.orbit.pitch());
        let mut shape = self.stack.rig_shape(pitch);
        shape.shoulder *= self.orbit.shoulder_side;
        let pose = desired_pose(anchor.pivot, yaw, pitch, &shape, self.orbit.zoom);
        CameraOutput { pose, yaw, pitch, cut: anchor.cut }
    }
}
