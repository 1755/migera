//! The push-only mode stack.
//!
//! Each layer holds a mode's [`ModeParams`] and a blend progress. The stack
//! is evaluated bottom-up: start from the bottom layer, then for each layer
//! above, interpolate toward it by its weight. Pushing a mode never reorders
//! the others: a new mode enters at the top with weight 0; a mode already in
//! the stack moves to the top *keeping its current contribution*, so a quick
//! combat → explore → combat flip never pops; once the top reaches full
//! weight everything under it is dropped. This is Lyra's camera-mode stack
//! and the main layer of UE5's Gameplay Camera System; see
//! `docs/knowledge/gameplay-camera/one-rig-with-blended-layers-over-blending-virtual-cameras.md`.
//!
//! Weights follow a smoothstep of linear progress, which starts and ends
//! with zero slope, so blended scalars are velocity-continuous across a push.

use super::rig::{ModeParams, OrbitShape, RigShape};
use bevy::prelude::*;
use serde::{Deserialize, Serialize};

/// A mode's name. Modes are data (profile entries), not a Rust enum, so a
/// game adds a mode without touching the camera.
#[derive(
    Debug, Clone, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize, Reflect,
)]
pub struct ModeId(pub String);

impl ModeId {
    pub fn new(name: &str) -> Self {
        Self(name.to_owned())
    }
}

impl From<&str> for ModeId {
    fn from(name: &str) -> Self {
        Self::new(name)
    }
}

/// A request to make `id` the active mode, blending in over `blend` seconds.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Reflect)]
pub struct ModeRequest {
    pub id: ModeId,
    pub blend: f32,
}

#[derive(Debug, Clone, PartialEq, Reflect)]
pub struct Layer {
    pub id: ModeId,
    pub params: ModeParams,
    /// Linear blend progress in `0..=1`; the weight is its smoothstep.
    pub progress: f32,
    /// Seconds from 0 to full weight.
    pub blend: f32,
}

impl Layer {
    pub fn weight(&self) -> f32 {
        smoothstep(self.progress)
    }
}

#[derive(Debug, Clone, PartialEq, Reflect)]
pub struct CameraStack {
    /// Bottom first. Never empty.
    pub layers: Vec<Layer>,
}

impl CameraStack {
    /// A stack holding only `id` at full weight.
    pub fn new(id: ModeId, params: ModeParams) -> Self {
        Self { layers: vec![Layer { id, params, progress: 1.0, blend: 0.0 }] }
    }

    pub fn top(&self) -> &Layer {
        self.layers.last().expect("a camera stack is never empty")
    }

    /// Makes `id` the top mode. See the module doc for the re-push rule.
    pub fn push(&mut self, id: ModeId, params: ModeParams, blend: f32) {
        if self.top().id == id {
            // Already on top: keep blending as it was, but take new params
            // (a profile hot reload) and the requested blend time.
            let top = self.layers.last_mut().unwrap();
            top.params = params;
            top.blend = blend;
            return;
        }
        let start = match self.layers.iter().position(|layer| layer.id == id) {
            Some(index) => {
                let contribution = self.contribution(index);
                self.layers.remove(index);
                inverse_smoothstep(contribution)
            }
            None => 0.0,
        };
        let progress = if blend <= 0.0 { 1.0 } else { start };
        self.layers.push(Layer { id, params, progress, blend });
        self.drop_hidden();
    }

    /// Advances the top layer's blend by `dt` seconds (real time: blends run
    /// while the game is paused).
    pub fn step(&mut self, dt: f32) {
        let top = self.layers.last_mut().unwrap();
        if top.progress < 1.0 {
            top.progress =
                if top.blend <= 0.0 { 1.0 } else { (top.progress + dt / top.blend).min(1.0) };
        }
        self.drop_hidden();
    }

    /// The blended boom at `pitch`.
    pub fn rig_shape(&self, pitch: f32) -> RigShape {
        let mut layers = self.layers.iter();
        let mut shape = layers.next().unwrap().params.rig_shape(pitch);
        for layer in layers {
            shape = shape.lerp(layer.params.rig_shape(pitch), layer.weight());
        }
        shape
    }

    /// The blended orbit limits and recentring weight.
    pub fn orbit_shape(&self) -> OrbitShape {
        let mut layers = self.layers.iter();
        let mut shape = layers.next().unwrap().params.orbit_shape();
        for layer in layers {
            shape = shape.lerp(layer.params.orbit_shape(), layer.weight());
        }
        shape
    }

    /// How much layer `index` shows in the final blend: its own weight times
    /// what every layer above lets through.
    fn contribution(&self, index: usize) -> f32 {
        let own = if index == 0 { 1.0 } else { self.layers[index].weight() };
        self.layers[index + 1..].iter().fold(own, |c, above| c * (1.0 - above.weight()))
    }

    fn drop_hidden(&mut self) {
        if let Some(full) = self.layers.iter().rposition(|layer| layer.progress >= 1.0)
            && full > 0
        {
            self.layers.drain(..full);
        }
    }
}

/// `3t² − 2t³`, clamped: zero slope at both ends.
#[inline]
pub fn smoothstep(t: f32) -> f32 {
    let t = t.clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// The `t` in `0..=1` whose [`smoothstep`] is `y`.
#[inline]
pub fn inverse_smoothstep(y: f32) -> f32 {
    let y = y.clamp(0.0, 1.0);
    0.5 - ((1.0 - 2.0 * y).asin() / 3.0).sin()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn explore() -> ModeId {
        ModeId::new("explore")
    }

    fn combat() -> ModeId {
        ModeId::new("combat")
    }

    #[test]
    fn inverse_smoothstep_inverts_smoothstep() {
        for i in 0..=100 {
            let t = i as f32 / 100.0;
            assert!((inverse_smoothstep(smoothstep(t)) - t).abs() < 1.0e-3, "t = {t}");
        }
    }

    #[test]
    fn a_pushed_mode_blends_in_and_then_drops_the_one_below() {
        let mut stack = CameraStack::new(explore(), ModeParams::exploration());
        stack.push(combat(), ModeParams::combat(), 0.5);
        assert_eq!(stack.layers.len(), 2);
        let start = stack.rig_shape(0.3).distance;
        assert!((start - ModeParams::exploration().rig_shape(0.3).distance).abs() < 1.0e-6);
        for _ in 0..30 {
            stack.step(1.0 / 60.0);
        }
        assert_eq!(stack.layers.len(), 1, "explore must be dropped under a full-weight combat");
        assert_eq!(stack.top().id, combat());
        assert!((stack.rig_shape(0.3).distance - ModeParams::combat().rig_shape(0.3).distance).abs() < 1.0e-6);
    }

    #[test]
    fn re_pushing_a_fading_mode_keeps_its_contribution() {
        let mut stack = CameraStack::new(explore(), ModeParams::exploration());
        stack.push(combat(), ModeParams::combat(), 1.0);
        for _ in 0..24 {
            stack.step(1.0 / 60.0);
        }
        let before = stack.rig_shape(0.3).distance;
        // Back to explore mid-blend: explore is at the bottom with
        // contribution 1 − w(combat); it moves to the top with that weight.
        stack.push(explore(), ModeParams::exploration(), 1.0);
        let after = stack.rig_shape(0.3).distance;
        assert!(
            (after - before).abs() < 1.0e-4,
            "re-pushing a mode must not pop the blended rig: {before} → {after}",
        );
    }

    #[test]
    fn a_blended_scalar_has_no_velocity_spike_at_the_push() {
        // Smoothstep starts with zero slope, so the first frames after a push
        // move the blended distance less than the steady middle of the blend.
        let mut stack = CameraStack::new(explore(), ModeParams::exploration());
        let mut previous = stack.rig_shape(0.3).distance;
        stack.push(combat(), ModeParams::combat(), 0.5);
        let mut steps = Vec::new();
        for _ in 0..30 {
            stack.step(1.0 / 60.0);
            let now = stack.rig_shape(0.3).distance;
            steps.push((now - previous).abs());
            previous = now;
        }
        let peak = steps.iter().cloned().fold(0.0, f32::max);
        assert!(steps[0] < peak * 0.25, "first-frame step {} vs peak {peak}", steps[0]);
    }

    #[test]
    fn a_zero_blend_push_cuts_immediately() {
        let mut stack = CameraStack::new(explore(), ModeParams::exploration());
        stack.push(combat(), ModeParams::combat(), 0.0);
        assert_eq!(stack.layers.len(), 1);
        assert_eq!(stack.top().id, combat());
    }
}
