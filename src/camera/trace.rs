//! Camera traces: every frame's clock, input and target, recorded so a
//! camera bug felt in play becomes a deterministic test.
//!
//! A trace replays into the pure [`CameraRig`] pipeline with no `World`, so
//! replaying a recording reproduces the camera exactly. A *golden* trace
//! also carries the poses it produced when it was recorded; replaying it and
//! comparing pins the camera's behaviour against regressions.
//!
//! Format: RON, extension `.camtrace.ron`.

use super::input::CameraInputSettings;
use super::pipeline::{CameraConfig, CameraFrame, CameraOutput, CameraRig};
use bevy::prelude::*;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct CameraTrace {
    /// The camera's yaw before the first frame.
    pub start_yaw: f32,
    pub frames: Vec<CameraFrame>,
}

/// A pose a golden trace expects at `frame`.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ExpectedPose {
    pub frame: usize,
    pub eye: Vec3,
    pub yaw: f32,
    pub pitch: f32,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GoldenTrace {
    pub trace: CameraTrace,
    /// Sparse: a pose every few frames is enough to catch a regression.
    pub expected: Vec<ExpectedPose>,
}

impl CameraTrace {
    /// Replays the trace through a fresh rig, returning every frame's output.
    pub fn replay(&self, settings: &CameraInputSettings, config: &CameraConfig) -> Vec<CameraOutput> {
        let mut rig = CameraRig::new(config, self.start_yaw);
        self.frames.iter().map(|frame| rig.step(frame, settings, config)).collect()
    }

    pub fn to_ron(&self) -> Result<String, ron::Error> {
        ron::ser::to_string_pretty(self, pretty())
    }

    pub fn from_ron(text: &str) -> Result<Self, ron::error::SpannedError> {
        ron::from_str(text)
    }

    /// Records a golden trace: this trace plus the poses it produces now,
    /// every `every` frames.
    pub fn golden(
        &self,
        settings: &CameraInputSettings,
        config: &CameraConfig,
        every: usize,
    ) -> GoldenTrace {
        let expected = self
            .replay(settings, config)
            .iter()
            .enumerate()
            .filter(|(i, _)| i % every.max(1) == 0)
            .map(|(frame, out)| ExpectedPose { frame, eye: out.pose.eye, yaw: out.yaw, pitch: out.pitch })
            .collect();
        GoldenTrace { trace: self.clone(), expected }
    }
}

/// One frame per line: diffable, and a tenth the size of fully pretty RON.
fn pretty() -> ron::ser::PrettyConfig {
    ron::ser::PrettyConfig::default().depth_limit(3)
}

impl GoldenTrace {
    pub fn to_ron(&self) -> Result<String, ron::Error> {
        ron::ser::to_string_pretty(self, pretty())
    }

    /// The largest eye distance between the replay and the recorded poses.
    pub fn worst_eye_error(&self, settings: &CameraInputSettings, config: &CameraConfig) -> f32 {
        let outputs = self.trace.replay(settings, config);
        self.expected
            .iter()
            .map(|expected| match outputs.get(expected.frame) {
                Some(out) => out.pose.eye.distance(expected.eye),
                None => f32::INFINITY,
            })
            .fold(0.0, f32::max)
    }
}
