//! Write-and-throw spike: a hardcoded sphere, ray-traced by a compute pass
//! that writes Bevy's REAL `DepthPrepass` + deferred G-buffer textures
//! (`ViewPrepassTextures.deferred`/`deferred_lighting_pass_id`) instead of
//! shading directly. Zero mesh geometry, zero custom fragment shading code —
//! Bevy's own stock deferred lighting pass (on by default with `PbrPlugin`)
//! does 100% of the shading. See `migera::prepass_probe` for the plugin and
//! its module doc comment for how this differs from `migera::hybrid`.
//!
//! Static camera, no orbit controller — see `prepass_write_sphere_orbit.rs`
//! for the interactive version with a ground-plane depth cross-check.

use bevy::pbr::DefaultOpaqueRendererMethod;
use bevy::prelude::*;
use bevy::render::view::Msaa;

use migera::prepass_probe::{PrepassProbePlugin, SdfProbeSphere};

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "prepass probe: static sphere".into(),
                ..default()
            }),
            ..default()
        }))
        // Forces every StandardMaterial (the ground plane in the orbit
        // example, none here) onto the deferred path too, so it and our
        // hand-written G-buffer fragment share one lighting pass.
        .insert_resource(DefaultOpaqueRendererMethod::deferred())
        .add_plugins(PrepassProbePlugin)
        .add_systems(Startup, setup)
        .add_systems(Update, screenshot_and_exit)
        .run();
}

/// `--shot <path>` takes a screenshot a few frames in (letting pipelines
/// finish compiling) — this spike has no `BenchConfig` harness, just enough
/// to capture proof-of-concept screenshots non-interactively. Doesn't exit
/// afterward; the caller is expected to kill the process once the file
/// lands (this is a throwaway example, not a benchmark harness).
fn screenshot_and_exit(mut commands: Commands, mut frame: Local<u32>, mut done: Local<bool>) {
    let Some(path) = std::env::args().skip_while(|a| a != "--shot").nth(1) else {
        return;
    };
    if *done {
        return;
    }
    *frame += 1;
    if *frame == 30 {
        *done = true;
        info!("prepass probe: screenshot -> {path}");
        commands
            .spawn(bevy::render::view::screenshot::Screenshot::primary_window())
            .observe(bevy::render::view::screenshot::save_to_disk(path));
    }
}

fn setup(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 1.5, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
        // Deferred rendering is structurally single-sampled — Bevy force-sets
        // this on any camera with DeferredPrepass and warns if you don't,
        // see bevy_core_pipeline::core_3d::check_msaa.
        Msaa::Off,
        bevy::core_pipeline::prepass::DepthPrepass,
        bevy::core_pipeline::prepass::DeferredPrepass,
    ));

    commands.spawn((
        DirectionalLight {
            illuminance: 6_000.0,
            shadow_maps_enabled: false,
            ..default()
        },
        Transform::from_xyz(4.0, 8.0, 4.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    commands.spawn(SdfProbeSphere {
        center: Vec3::ZERO,
        radius: 1.0,
        base_color: Vec3::new(0.8, 0.2, 0.2),
        metallic: 0.0,
        roughness: 0.4,
        reflectance: 0.5,
    });
}
