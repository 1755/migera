//! Point-light extraction for the compute trace's own reflection-ray
//! shading (see `sphere_prepass_trace.wgsl`'s reflection pass). Deliberately
//! NOT reusing Bevy's clustered light buffer — that's assembled deep inside
//! `bevy_pbr`'s own render-world systems in a layout private to its own
//! shaders, with no stable public bind-group our own pipeline could reuse.
//! Since this spike only ever needs a handful of lights for a hand-rolled
//! Lambertian+specular evaluation at one reflection bounce (not the general
//! clustered-forward light loop stock deferred lighting itself uses for
//! direct lighting), a small fixed-size array extracted directly from the
//! main world's `PointLight` entities is simpler and sufficient.

use bevy::prelude::*;
use bevy::render::Extract;

/// Up to this many point lights are visible to the compute trace's
/// reflection shading — matches the "two more point lights" this spike adds
/// alongside the scene's ambient/direct lighting, with headroom.
pub const MAX_PROBE_LIGHTS: usize = 4;

#[derive(Clone, Copy, Debug, Default)]
pub struct ProbeLight {
    pub position: Vec3,
    pub color: Vec3,
    pub intensity: f32,
}

#[derive(Resource, Default)]
pub struct ProbeLights {
    pub lights: Vec<ProbeLight>,
}

pub fn extract_probe_lights(
    mut probe_lights: ResMut<ProbeLights>,
    lights: Extract<Query<(&PointLight, &GlobalTransform)>>,
) {
    probe_lights.lights.clear();
    for (light, transform) in lights.iter().take(MAX_PROBE_LIGHTS) {
        probe_lights.lights.push(ProbeLight {
            position: transform.translation(),
            color: light.color.to_linear().to_vec3(),
            // Raw lumens (PointLight::intensity's own unit) — converted to
            // luminous intensity (lumens/steradian) the same way bevy_pbr's
            // own extraction does (`intensity / (4*PI)`, light.rs:537)
            // inside sphere_prepass_trace.wgsl's shade_lights, not here.
            intensity: light.intensity,
        });
    }
}
