//! Write-and-throw spike, interactive version: same `SdfProbeSphere` as
//! `prepass_write_sphere.rs`, plus an orbit camera and an ordinary Bevy PBR
//! mesh (a ground plane) sharing the scene — the real end-to-end proof that
//! our compute-written depth is byte-compatible with Bevy's own: watch the
//! ground plane and the G-buffer sphere occlude each other correctly as the
//! camera orbits (if our reverse-Z reconstruction were off, one would always
//! draw over the other regardless of actual depth order).
//!
//! Extended with: two `PointLight`s (replacing the single `DirectionalLight`
//! as the scene's main illumination — a dim directional fill light stays for
//! ambient readability and casts the plane's own shadow), soft shadow edges
//! (`ShadowFilteringMethod::Gaussian`, stock/default, no feature flag
//! needed), screen-space reflections on the ground plane (tuned into SSR's
//! default roughness fade window), and a genuine ray-traced reflection baked
//! into the sphere's own G-buffer emissive channel by the compute trace (see
//! `sphere_prepass_trace.wgsl`'s `trace_reflection` — a real reflect() +
//! re-intersect against the analytic scene, not a screen-space
//! approximation, since SSR can only reflect what's already on screen and
//! this spike wants the sphere's reflection to work from any angle).
//!
//! NOTE on shadow casting: the sphere currently only RECEIVES shadows (from
//! the plane), it does not yet CAST one onto the plane — stock Bevy's shadow
//! maps are populated purely from `Mesh3d` entities (`queue_shadows` has no
//! non-mesh extension point), so a G-buffer-only object can't cast via the
//! normal path. A direct `ShadowView`/`DepthAttachment` write (the same
//! technique this plugin already uses for the deferred G-buffer) IS possible
//! and was confirmed as a viable follow-up, just not implemented in this
//! pass — flagged here rather than silently left out.
//!
//! `--stress N`: spawns N copies of the whole scene (one sphere + one plane
//! each, sharing the one set of lights/camera) on a grid with gaps between
//! cells — `--stress 1` (or omitting the flag) reproduces the exact
//! single-instance scene, since the grid-placement math (`stress_grid_dim`/
//! `stress_cell_center`, mirroring `examples/gallery.rs`'s own `--stress`
//! convention) degenerates to a single cell at the world origin for N=1, no
//! separate code path. The orbit camera is untouched by `--stress` — it
//! always orbits the same fixed radius/height around the origin regardless
//! of grid size (unlike `gallery.rs`'s stress mode, which stretches the
//! orbit to frame a growing grid; this spike keeps the camera behavior
//! identical across N so the "N copies of the same scene" comparison stays
//! apples-to-apples). Capped at `MAX_STRESS_INSTANCES` (16, see
//! `migera::prepass_probe::pipeline`) — every sphere/plane shares identical
//! material, only grid position differs.
//!
//! Controls: mouse-free — the camera auto-orbits. Close the window to exit.

use bevy::core_pipeline::prepass::{DeferredPrepass, DepthPrepass};
use bevy::light::ShadowFilteringMethod;
use bevy::pbr::{DefaultOpaqueRendererMethod, ScreenSpaceReflections};
use bevy::prelude::*;
use bevy::render::view::Msaa;

use migera::prepass_probe::pipeline::MAX_STRESS_INSTANCES;
use migera::prepass_probe::{PrepassProbePlugin, SdfProbeGroundPlane, SdfProbeSphere};

/// World-space gap between adjacent grid cells' plane edges — mirrors
/// `examples/gallery.rs`'s `STRESS_GAP` convention (a visible seam between
/// copies, not touching planes).
const STRESS_GAP: f32 = 2.0;

fn main() {
    let stress = std::env::args()
        .skip_while(|a| a != "--stress")
        .nth(1)
        .and_then(|s| s.parse::<usize>().ok())
        .unwrap_or(1)
        .clamp(1, MAX_STRESS_INSTANCES);

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "prepass probe: orbit + ground plane".into(),
                ..default()
            }),
            ..default()
        }))
        .insert_resource(DefaultOpaqueRendererMethod::deferred())
        .insert_resource(StressCount(stress))
        .add_plugins(PrepassProbePlugin)
        .add_systems(Startup, setup)
        .add_systems(Update, (orbit_camera, screenshot_at))
        .run();
}

#[derive(Resource, Clone, Copy)]
struct StressCount(usize);

/// World-space XZ center of stress cell `index` in an as-square-as-possible
/// `dim x dim` grid holding `count` cells total, centered on the world
/// origin — `count == 1` gives `dim == 1` and a single center at the origin,
/// i.e. the exact non-`--stress` placement, not a separate code path (same
/// shape as `examples/gallery.rs`'s `stress_grid_dim`/`stress_cell_center`).
fn stress_grid_dim(count: usize) -> usize {
    (count as f32).sqrt().ceil() as usize
}

fn stress_cell_center(index: usize, dim: usize, cell: f32) -> Vec2 {
    let row = (index / dim) as f32;
    let col = (index % dim) as f32;
    let half = (dim as f32 - 1.0) * 0.5;
    Vec2::new((col - half) * cell, (row - half) * cell)
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    stress: Res<StressCount>,
) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 1.5, 6.0).looking_at(Vec3::ZERO, Vec3::Y),
        Msaa::Off,
        DepthPrepass,
        DeferredPrepass,
        // Default fixed-kernel PCF-style soft shadow edges — needs no
        // feature flag (true penumbra-scaling PCSS soft shadows would need
        // `experimental_pbr_pcss`, not enabled in this project's Cargo.toml).
        ShadowFilteringMethod::Gaussian,
        // Requires DeferredPrepass (auto-required) + DefaultOpaqueRendererMethod::deferred()
        // (set above) to actually take effect; reads the deferred G-buffer
        // directly, no NormalPrepass duplication needed.
        ScreenSpaceReflections {
            // Ground plane's roughness (0.25 below) sits inside this range;
            // widen/lower if you change the plane's material.
            min_perceptual_roughness: 0.05..0.15,
            max_perceptual_roughness: 0.3..0.4,
            ..default()
        },
    ));

    // Dim directional fill light — kept mainly so the plane still casts a
    // real shadow (point lights below illuminate the scene but the sphere
    // can't cast onto the plane yet, see this file's header note; the
    // directional light's shadow at least demonstrates a real mesh-cast
    // shadow crossing the scene).
    commands.spawn((
        DirectionalLight {
            illuminance: 1_500.0,
            shadow_maps_enabled: true,
            ..default()
        },
        Transform::from_xyz(4.0, 8.0, 4.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    commands.spawn((
        PointLight {
            color: Color::srgb(1.0, 0.55, 0.35),
            intensity: 1_000_000.0, // PointLight::default()'s own value
            shadow_maps_enabled: true,
            ..default()
        },
        Transform::from_xyz(-3.0, 2.5, 2.0),
    ));
    commands.spawn((
        PointLight {
            color: Color::srgb(0.35, 0.6, 1.0),
            intensity: 1_000_000.0, // PointLight::default()'s own value
            shadow_maps_enabled: true,
            ..default()
        },
        Transform::from_xyz(3.0, 2.0, -2.5),
    ));

    // `--stress N`: N copies of the plane+sphere pair on a grid, sharing the
    // one mesh asset (a single Plane3d handle, reused per instance — only
    // the Transform differs, same as any other Bevy mesh instancing) and one
    // material. `count == 1` places the single cell at the origin, exactly
    // reproducing the pre-`--stress` layout (see `stress_cell_center`'s doc
    // comment). Roughness lowered from a plain flat plane's typical ~0.9
    // into SSR's tuned fade window above so each plane visibly reflects its
    // own sphere/the lights.
    const PLANE_HALF_SIZE: f32 = 4.0;
    const PLANE_Y: f32 = -1.0;
    const PLANE_BASE_COLOR: Vec3 = Vec3::new(0.5, 0.5, 0.55);
    const PLANE_ROUGHNESS: f32 = 0.25;
    let cell = PLANE_HALF_SIZE * 2.0 + STRESS_GAP;
    let dim = stress_grid_dim(stress.0);
    let plane_mesh = meshes.add(Plane3d::default().mesh().size(
        PLANE_HALF_SIZE * 2.0,
        PLANE_HALF_SIZE * 2.0,
    ));
    let plane_material = materials.add(StandardMaterial {
        base_color: Color::srgb(PLANE_BASE_COLOR.x, PLANE_BASE_COLOR.y, PLANE_BASE_COLOR.z),
        perceptual_roughness: PLANE_ROUGHNESS,
        ..default()
    });
    for i in 0..stress.0 {
        let center_xz = stress_cell_center(i, dim, cell);

        commands.spawn((
            Mesh3d(plane_mesh.clone()),
            MeshMaterial3d(plane_material.clone()),
            Transform::from_xyz(center_xz.x, PLANE_Y, center_xz.y),
        ));
        commands.spawn(SdfProbeGroundPlane {
            center_xz,
            y: PLANE_Y,
            half_size: PLANE_HALF_SIZE,
            base_color: PLANE_BASE_COLOR,
            roughness: PLANE_ROUGHNESS,
        });

        commands.spawn(SdfProbeSphere {
            // Lifted slightly above the plane (rather than resting exactly
            // at PLANE_Y + radius): a sphere touching the plane creates a
            // near-singularity for the reflection ray right at the contact
            // point (a ray leaving there travels nearly parallel to the
            // plane before hitting it again at very close range, blowing up
            // the inverse-square light falloff) — a small gap avoids that
            // entirely.
            center: Vec3::new(center_xz.x, 0.15, center_xz.y),
            radius: 1.0,
            base_color: Vec3::new(0.8, 0.2, 0.2),
            metallic: 0.3,
            roughness: 0.35,
            reflectance: 0.4,
        });
    }
}

fn orbit_camera(time: Res<Time>, mut cams: Query<&mut Transform, With<Camera3d>>) {
    let t = time.elapsed_secs() * 0.4;
    let r = 6.0;
    let pos = Vec3::new(r * t.cos(), 1.5 + 1.2 * (t * 0.5).sin(), r * t.sin());
    for mut transform in &mut cams {
        transform.translation = pos;
        transform.look_at(Vec3::ZERO, Vec3::Y);
    }
}

/// `--shot <path> --at <secs>` — mirrors `examples/gallery.rs`'s convention
/// closely enough for this spike's purposes: waits for `--at` seconds of
/// orbit, then captures once.
fn screenshot_at(mut commands: Commands, time: Res<Time>, mut done: Local<bool>) {
    if *done {
        return;
    }
    let args: Vec<String> = std::env::args().collect();
    let Some(path) = args
        .iter()
        .position(|a| a == "--shot")
        .and_then(|i| args.get(i + 1))
    else {
        return;
    };
    let at = args
        .iter()
        .position(|a| a == "--at")
        .and_then(|i| args.get(i + 1))
        .and_then(|s| s.parse::<f32>().ok())
        .unwrap_or(1.0);
    if time.elapsed_secs() >= at {
        *done = true;
        info!("prepass probe orbit: screenshot -> {path}");
        commands
            .spawn(bevy::render::view::screenshot::Screenshot::primary_window())
            .observe(bevy::render::view::screenshot::save_to_disk(path.clone()));
    }
}
