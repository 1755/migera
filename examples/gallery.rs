//! Gallery: ground-up rewrite of the hybrid SDF renderer showcase.
//!
//! Starting point: a bare empty Bevy scene (camera + window) plus an
//! on-screen FPS graph/stats overlay, no SDF rendering yet. See
//! `src/hybrid/mod.rs`'s doc comment for why this is a fresh start rather
//! than a continuation of `examples/gallery_legacy.rs` (whose FPS-graph
//! implementation this ports from, unchanged in approach: a fixed-size
//! ring buffer of frame times sampled once per frame, drawn as UI bars).
//!
//! Run: `cargo run --example gallery`

use std::collections::VecDeque;
use std::f32::consts::TAU;

use bevy::camera::{Exposure, Hdr};
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::diagnostic::{DiagnosticPath, DiagnosticsStore, FrameCount, FrameTimeDiagnosticsPlugin};
use bevy::math::EulerRot;
use bevy::prelude::*;
use bevy::render::diagnostic::RenderDiagnosticsPlugin;
use bevy::render::view::window::screenshot::{Screenshot, save_to_disk};
use bevy_egui::{EguiContexts, EguiPlugin, EguiPrimaryContextPass, egui};

use migera::hybrid::HybridRenderPlugin;
use migera::hybrid::extract::{
    ConeTraceConfig, DdgiConfig, DenoiseConfig, DofConfig, GiMethod, GiMethodConfig, HybridPostConfig, JitterConfig, LampLight,
    LightToggles, ProjectorLight, ReflectionConfig, RenderScaleConfig, ShadowConfig, SunLight, TemporalConfig, TransmissionConfig,
};
use migera::hybrid::material::Material;
use migera::sdf::assembly::SdfSceneRoot;
use migera::sdf::components::Shape;

/// How many past frames the graph/stats window covers.
const FPS_BARS: usize = 60;

/// `--shot PATH --at-frame N`: takes a screenshot on frame `N` and exits —
/// the same headless-verification convention
/// `examples/gallery_legacy.rs` used throughout its own development (see
/// that file's `auto_shot`), needed here for the same reason: visually
/// confirming what actually renders without a human at the keyboard.
///
/// Frame count, not elapsed wall-clock seconds: the legacy renderer's own
/// history has a documented case (see docs/knowledge/sdf-3d/rendering/
/// soft-shadows-and-ao.md) where driving a time-based value off
/// `Time::elapsed_secs()` produced a slightly different result on every
/// separate process run (machine load, startup jitter), making two "same"
/// captures not actually comparable. A frame count sidesteps that
/// entirely — frame 60 is frame 60 regardless of how long it took to get
/// there. Not yet load-bearing today (nothing on screen is time-driven
/// yet), but cheap to get right from the start rather than hit the same
/// bug again once something is.
#[derive(Resource, Clone, Default)]
struct ShotConfig {
    shot: Option<(String, u32)>,
}

impl ShotConfig {
    fn from_args() -> Self {
        let args: Vec<String> = std::env::args().collect();
        let mut cfg = Self::default();
        let mut i = 0;
        while i < args.len() {
            let val = |i: &mut usize| -> String {
                *i += 1;
                args.get(*i).cloned().unwrap_or_default()
            };
            match args[i].as_str() {
                "--shot" => cfg.shot = Some((val(&mut i), cfg.shot.map_or(120, |s| s.1))),
                "--at-frame" => {
                    cfg.shot = Some((cfg.shot.map_or("/tmp/gallery.png".into(), |s| s.0), val(&mut i).parse().unwrap_or(120)));
                }
                _ => {}
            }
            i += 1;
        }
        cfg
    }
}

/// `--camera-mode orbit|manual` (default `orbit`), plus mode-specific
/// params:
/// - orbit: `--camera-orbit-radius near|far|grid|<meters>` (default
///   `near`) and `--camera-orbit-speed <rad/s>` (default `0.35`). `grid`
///   scales to the current `--stress N` grid's actual extent (see
///   `grid_orbit_radius`), so a large stress grid doesn't need its orbit
///   radius picked by hand.
/// - manual: `--camera-pos x,y,z` and `--camera-rot pitch,yaw,roll`
///   (degrees, extrinsic YXZ — yaw around Y, then pitch around X, then
///   roll around Z), both optional and defaulting to origin/no-rotation
///   if omitted, so a manual pose can be dialed in one axis at a time.
#[derive(Resource, Clone)]
struct CameraConfig {
    mode: CameraMode,
    orbit_radius: f32,
    orbit_speed: f32,
    manual_pos: Vec3,
    manual_rot_deg: Vec3,
}

#[derive(Clone, Copy, PartialEq, Eq, Default)]
enum CameraMode {
    #[default]
    Orbit,
    Manual,
}

impl Default for CameraConfig {
    fn default() -> Self {
        Self {
            mode: CameraMode::default(),
            orbit_radius: Self::NEAR_RADIUS,
            orbit_speed: 0.35,
            manual_pos: Vec3::new(6.0, 5.0, 8.0),
            manual_rot_deg: Vec3::ZERO,
        }
    }
}

impl CameraConfig {
    const NEAR_RADIUS: f32 = 10.0;
    const FAR_RADIUS: f32 = 25.0;

    fn from_args() -> Self {
        let mut cfg = Self::default();
        let parse_vec3 = |s: &str| -> Option<Vec3> {
            let mut it = s.split(',').map(|p| p.trim().parse::<f32>());
            Some(Vec3::new(it.next()?.ok()?, it.next()?.ok()?, it.next()?.ok()?))
        };
        let args: Vec<String> = std::env::args().collect();
        let mut i = 0;
        while i < args.len() {
            let val = |i: &mut usize| -> String {
                *i += 1;
                args.get(*i).cloned().unwrap_or_default()
            };
            match args[i].as_str() {
                "--camera-mode" => {
                    cfg.mode = match val(&mut i).as_str() {
                        "manual" => CameraMode::Manual,
                        _ => CameraMode::Orbit,
                    };
                }
                "--camera-orbit-radius" => {
                    cfg.orbit_radius = match val(&mut i).as_str() {
                        "near" => Self::NEAR_RADIUS,
                        "far" => Self::FAR_RADIUS,
                        "grid" => grid_orbit_radius(StressConfig::from_args().count),
                        other => other.parse().unwrap_or(cfg.orbit_radius),
                    };
                }
                "--camera-orbit-speed" => {
                    cfg.orbit_speed = val(&mut i).parse().unwrap_or(cfg.orbit_speed);
                }
                "--camera-pos" => {
                    if let Some(v) = parse_vec3(&val(&mut i)) {
                        cfg.manual_pos = v;
                    }
                }
                "--camera-rot" => {
                    if let Some(v) = parse_vec3(&val(&mut i)) {
                        cfg.manual_rot_deg = v;
                    }
                }
                _ => {}
            }
            i += 1;
        }
        cfg
    }
}

/// `--stress N` (default `1`): spawns `N` independent copies of the scene
/// (its own ground box + its own spinning cube per copy — a real BVH leaf
/// pair, not a single object repeated via SDF-level tiling) on a
/// `ceil(sqrt(N)) x ceil(sqrt(N))` grid, `STRESS_GAP` meters between
/// adjacent ground-box edges, centered on the world origin so the orbit
/// camera still frames the whole grid. `--stress 1` (or omitting the
/// flag) reproduces today's exact single-scene layout — the grid's
/// `count == 1` case, not a separate code path — see `stress_cell_center`.
/// A real, independently BVH-culled multi-object scene to measure the
/// trace pipeline's actual scaling against, not just the 2-object case.
#[derive(Resource, Clone, Copy)]
struct StressConfig {
    count: usize,
}

impl Default for StressConfig {
    fn default() -> Self {
        Self { count: 1 }
    }
}

impl StressConfig {
    /// Matches the ground box's own footprint (`GROUND_HALF_EXTENT * 2`,
    /// see `spawn_scene`) so `--stress 1`'s single cell is pixel-identical
    /// to today's non-stress scene, and larger `N` tiles that exact same
    /// footprint with real gaps — mirrors `examples/gallery_legacy.rs`'s
    /// `STRESS_PLATFORM_SIZE`/`STRESS_GAP` convention exactly.
    const CELL_GAP: f32 = 3.0;

    fn from_args() -> Self {
        let mut cfg = Self::default();
        let args: Vec<String> = std::env::args().collect();
        let mut i = 0;
        while i < args.len() {
            let val = |i: &mut usize| -> String {
                *i += 1;
                args.get(*i).cloned().unwrap_or_default()
            };
            if args[i] == "--stress" {
                cfg.count = val(&mut i).parse().unwrap_or(cfg.count).max(1);
            }
            i += 1;
        }
        cfg
    }
}

/// `--shape <name>` (default `box`, i.e. today's `RoundedBox` cube):
/// selects which single `Shape` variant `spawn_scene` spawns in place of
/// the cube — same one-object-per-cell layout, `--stress N` grid, and
/// `--spin`/`Spinning` rotation as always, just a different shape. Exists
/// to visually verify each newly-ported primitive (`cpu_ref::
/// local_distance`'s Sphere, RoundedBox, RoundedCylinder, Capsule,
/// Ellipsoid, BoxFrame, HexPrism — RoundedCone excluded, see that
/// function's doc comment) renders, rotates, and gets a correct AABB/BVH
/// box, one at a time — not a multi-shape showcase row, which would
/// conflate "does this one shape work" with "do several shapes coexist,"
/// two different, separately-answerable questions.
#[derive(Resource, Clone, Copy, Debug, Default, PartialEq, Eq)]
enum ShapeChoice {
    #[default]
    Box,
    Sphere,
    Cylinder,
    Capsule,
    Ellipsoid,
    BoxFrame,
    HexPrism,
}

impl ShapeChoice {
    fn from_args() -> Self {
        let args: Vec<String> = std::env::args().collect();
        let mut i = 0;
        while i < args.len() {
            if args[i] == "--shape" {
                let name = args.get(i + 1).cloned().unwrap_or_default();
                return match name.as_str() {
                    "sphere" => Self::Sphere,
                    "cylinder" => Self::Cylinder,
                    "capsule" => Self::Capsule,
                    "ellipsoid" => Self::Ellipsoid,
                    "box-frame" => Self::BoxFrame,
                    "hex-prism" => Self::HexPrism,
                    // "box" and any unrecognized value fall back to the
                    // default rather than panicking on a typo.
                    _ => Self::Box,
                };
            }
            i += 1;
        }
        Self::default()
    }

    /// This choice's `Shape` at a fixed reference size (roughly comparable
    /// visual footprint across variants, not identical volume — exact
    /// matching isn't the point, "clearly visible and rotatable" is), plus
    /// its spawn height so no rotation ever dips it below the ground's top
    /// face at `y = 0`.
    ///
    /// Height is derived from `migera::hybrid::scene::local_aabb`'s own
    /// half-extents (that box's half-diagonal, `sqrt(hx^2+hy^2+hz^2)`) —
    /// NOT the shape's own tight geometric bounding radius, and this
    /// distinction is load-bearing, not a style choice. `scene::
    /// world_aabb` (what the real renderer's AABB/BVH path actually
    /// calls) computes a rotated world AABB by rotating `local_aabb`'s 8
    /// BOX corners, not the shape's true surface — so even a
    /// rotation-invariant shape like `Sphere` (whose `local_aabb` is a
    /// cube of half-extent `radius`, not a tight sphere bound) gets an
    /// AABB that grows toward the cube's `radius * sqrt(3)` diagonal at
    /// some rotations, well past the sphere's own radius. A height
    /// derived from the shape's tight bound (as an earlier version of
    /// this function did) is provably insufficient — caught by
    /// `tests::no_shape_choice_ever_dips_below_ground_at_any_rotation`,
    /// which failed for `Sphere` before this fix. Using `local_aabb`'s
    /// own half-diagonal instead guarantees clearance against exactly
    /// what the renderer computes, for every shape, uniformly.
    fn shape_and_height(self) -> (Shape, f32) {
        let shape = match self {
            Self::Box => Shape::RoundedBox { half_extents: Vec3::splat(0.8), corner_radius: 0.0 },
            Self::Sphere => Shape::Sphere { radius: 0.9 },
            Self::Cylinder => Shape::RoundedCylinder { radius: 0.7, half_height: 0.8, edge_radius: 0.1 },
            Self::Capsule => {
                Shape::Capsule { a: Vec3::new(0.0, -0.6, 0.0), b: Vec3::new(0.0, 0.6, 0.0), radius: 0.35 }
            }
            Self::Ellipsoid => Shape::Ellipsoid { radii: Vec3::new(1.1, 0.7, 0.7) },
            Self::BoxFrame => Shape::BoxFrame { half_extents: Vec3::splat(0.8), wall_thickness: 0.15 },
            Self::HexPrism => Shape::HexPrism { radius: 0.8, half_height: 0.5 },
        };
        let local = migera::hybrid::scene::local_aabb(&shape);
        // `world_aabb`'s 8-corner rotation (`Aabb::transformed`) rotates
        // each corner AROUND THE COORDINATE ORIGIN (`rotation * corner`,
        // not `rotation * (corner - aabb_center) + aabb_center`), so the
        // true worst-case distance any rotation can ever put a corner at
        // is measured from the ORIGIN to the farthest corner — NOT the
        // AABB's own half-diagonal from its own center. These coincide
        // only when the AABB happens to be centered at the origin (true
        // for every shape spawned here today — Sphere/RoundedBox/
        // RoundedCylinder/Ellipsoid/BoxFrame/HexPrism's `local_aabb` is
        // always `from_center_half(ZERO, ...)`, and this module's
        // `Capsule` choice uses symmetric endpoints) but is NOT true in
        // general (Capsule/RoundedCone's `local_aabb` spans between two
        // arbitrary endpoints, which need not be symmetric about the
        // origin) — so this deliberately measures from the origin, not
        // from `local.center()`, to stay correct even if a future choice
        // ever uses asymmetric endpoints.
        let farthest_corner = local.min.abs().max(local.max.abs());
        let object_height = farthest_corner.length() + 0.05;
        (shape, object_height)
    }
}

/// As-square-as-possible grid side length holding `count` cells total —
/// `count == 1` gives `dim == 1`, a single cell at the origin.
fn stress_grid_dim(count: usize) -> usize {
    (count as f32).sqrt().ceil() as usize
}

/// World-space XZ center of stress cell `index` in a `dim x dim` grid,
/// centered on the world origin (so the orbit camera, which always looks
/// at `Vec3::ZERO`, frames the whole grid regardless of `count`).
fn stress_cell_center(index: usize, dim: usize, cell_size: f32) -> Vec3 {
    let row = (index / dim) as f32;
    let col = (index % dim) as f32;
    let half = (dim as f32 - 1.0) * 0.5;
    Vec3::new((col - half) * cell_size, 0.0, (row - half) * cell_size)
}

/// Orbit radius that frames a `--stress count`-cell grid's full extent —
/// half the grid's diagonal footprint, times a small margin so cells at
/// the grid's edge aren't clipped right at the view frustum's boundary.
/// `count == 1` gives a `dim == 1` grid whose footprint is just one
/// ground box, so this naturally falls back to roughly `CameraConfig::
/// NEAR_RADIUS`'s own framing for the non-stress case.
fn grid_orbit_radius(count: usize) -> f32 {
    let dim = stress_grid_dim(count);
    let cell_size = GROUND_HALF_EXTENT.x * 2.0 + StressConfig::CELL_GAP;
    let footprint = dim as f32 * cell_size;
    footprint * std::f32::consts::FRAC_1_SQRT_2 * 1.15
}

/// `--gizmos on|off` (default `on`): whether the origin-axes debug gizmos
/// draw at startup. `--aabb-gizmos on|off` (default `on`): whether the
/// per-object AABB/BVH-node debug boxes draw at startup — see
/// `draw_object_aabb_gizmos`'s doc comment. Both toggleable live via the
/// egui "Controls" panel's checkboxes — this resource is the single shared
/// state both the CLI flags and the panel drive.
#[derive(Resource, Clone, Copy)]
struct DebugGizmos {
    axes: bool,
    aabbs: bool,
}

impl Default for DebugGizmos {
    fn default() -> Self {
        Self { axes: true, aabbs: false }
    }
}

impl DebugGizmos {
    fn from_args() -> Self {
        let mut cfg = Self::default();
        let args: Vec<String> = std::env::args().collect();
        let mut i = 0;
        while i < args.len() {
            let val = |i: &mut usize| -> String {
                *i += 1;
                args.get(*i).cloned().unwrap_or_default()
            };
            match args[i].as_str() {
                "--gizmos" => cfg.axes = val(&mut i) != "off",
                "--aabb-gizmos" => cfg.aabbs = val(&mut i) != "off",
                _ => {}
            }
            i += 1;
        }
        cfg
    }
}

/// `--sun on|off` / `--lamp on|off` / `--projector on|off` (each default
/// `on`): per-light enable/disable, driving `migera::hybrid::extract::
/// LightToggles` — the resource `extract_hybrid_lights` reads every frame
/// to decide which lights actually get extracted (see that resource's
/// doc comment for why a disabled light is skipped entirely, not
/// extracted at zero intensity). Defined here rather than as a
/// `LightToggles::from_args` method since CLI parsing is this example's
/// concern, not the library's — same reasoning `DebugGizmos`/
/// `StressConfig`/`ShapeChoice`'s own `from_args` methods already
/// establish for every other CLI-driven resource in this file.
fn light_toggles_from_args() -> LightToggles {
    let mut cfg = LightToggles::default();
    let args: Vec<String> = std::env::args().collect();
    let mut i = 0;
    while i < args.len() {
        let val = |i: &mut usize| -> String {
            *i += 1;
            args.get(*i).cloned().unwrap_or_default()
        };
        match args[i].as_str() {
            "--sun" => cfg.sun = val(&mut i) != "off",
            "--lamp" => cfg.lamp = val(&mut i) != "off",
            "--projector" => cfg.projector = val(&mut i) != "off",
            _ => {}
        }
        i += 1;
    }
    cfg
}

/// `--shadows on|off` (default `on`) / `--shadow-k F` (default `12.0`,
/// matching `ShadowConfig::default`'s own value): soft-shadow toggle and
/// penumbra hardness, driving `migera::hybrid::extract::ShadowConfig` —
/// same "CLI parsing is this example's concern, not the library's"
/// reasoning `light_toggles_from_args` already establishes.
fn shadow_config_from_args() -> ShadowConfig {
    let mut cfg = ShadowConfig::default();
    let args: Vec<String> = std::env::args().collect();
    let mut i = 0;
    while i < args.len() {
        let val = |i: &mut usize| -> String {
            *i += 1;
            args.get(*i).cloned().unwrap_or_default()
        };
        match args[i].as_str() {
            "--shadows" => cfg.enabled = val(&mut i) != "off",
            "--shadow-k" => {
                if let Ok(v) = val(&mut i).parse::<f32>() {
                    cfg.k = v;
                }
            }
            _ => {}
        }
        i += 1;
    }
    cfg
}

/// `--gi-method none|ddgi|cascades|conetrace` (default `ddgi` — see
/// `GiMethod`'s own doc comment for why) — selects which indirect-diffuse
/// technique is active, driving `migera::hybrid::extract::GiMethodConfig`.
/// `cascades` selects `GiMethod::RadianceCascades`, the experimental A/B
/// alternative to DDGI (see that variant's own doc comment) — this is
/// its own smoke-test/live-dispatch entry point, not meant to replace
/// `ddgi` as anyone's default. If passed multiple times, whichever
/// appears LATER on the command line wins (plain left-to-right argv scan,
/// same behavior every other `_from_args` function in this file already
/// has for repeated/conflicting flags).
fn gi_method_config_from_args() -> GiMethodConfig {
    let mut cfg = GiMethodConfig::default();
    let args: Vec<String> = std::env::args().collect();
    let mut i = 0;
    while i < args.len() {
        let val = |i: &mut usize| -> String {
            *i += 1;
            args.get(*i).cloned().unwrap_or_default()
        };
        if args[i].as_str() == "--gi-method" {
            cfg.method = match val(&mut i).as_str() {
                "none" => GiMethod::None,
                "conetrace" => GiMethod::ConeTrace,
                "cascades" => GiMethod::RadianceCascades,
                _ => GiMethod::Ddgi, // "ddgi" or unrecognized: default to DDGI
            };
        }
        i += 1;
    }
    cfg
}

/// `--ddgi-probes-per-frame N` / `--ddgi-tile-size N` / `--ddgi-history N` /
/// `--ddgi-max-t F` / `--ddgi-spacing F` / `--ddgi-layers N`: DDGI
/// probe-grid tunables, driving `migera::hybrid::extract::DdgiConfig` —
/// mirrors `shadow_config_from_args`'s exact pattern. Whether DDGI is the
/// ACTIVE indirect-diffuse technique is controlled separately, by
/// `--gi-method` (see `gi_method_config_from_args`) — these tunables
/// apply whenever DDGI is selected.
fn ddgi_config_from_args() -> DdgiConfig {
    let mut cfg = DdgiConfig::default();
    let args: Vec<String> = std::env::args().collect();
    let mut i = 0;
    while i < args.len() {
        let val = |i: &mut usize| -> String {
            *i += 1;
            args.get(*i).cloned().unwrap_or_default()
        };
        match args[i].as_str() {
            "--ddgi-probes-per-frame" => {
                if let Ok(v) = val(&mut i).parse::<u32>() {
                    cfg.probes_per_frame = v.max(1);
                }
            }
            "--ddgi-tile-size" => {
                if let Ok(v) = val(&mut i).parse::<u32>() {
                    cfg.tile_size = v.max(1);
                }
            }
            "--ddgi-history" => {
                if let Ok(v) = val(&mut i).parse::<f32>() {
                    cfg.max_history_length = v;
                }
            }
            "--ddgi-max-t" => {
                if let Ok(v) = val(&mut i).parse::<f32>() {
                    cfg.max_t = v;
                }
            }
            "--ddgi-spacing" => {
                if let Ok(v) = val(&mut i).parse::<f32>() {
                    cfg.probe_spacing = v;
                }
            }
            "--ddgi-layers" => {
                if let Ok(v) = val(&mut i).parse::<u32>() {
                    cfg.vertical_layers = v.max(1);
                }
            }
            _ => {}
        }
        i += 1;
    }
    cfg
}

/// `--cone-half-angle F` / `--cone-origin-radius F` / `--cone-max-t F` /
/// `--cone-bounces N`: SDF cone-tracing tunables, driving
/// `migera::hybrid::extract::ConeTraceConfig` — mirrors
/// `shadow_config_from_args`'s exact pattern. Whether cone tracing is
/// the ACTIVE technique is controlled by `--gi-method conetrace` (see
/// `gi_method_config_from_args`, which already accepts this string).
fn cone_trace_config_from_args() -> ConeTraceConfig {
    let mut cfg = ConeTraceConfig::default();
    let args: Vec<String> = std::env::args().collect();
    let mut i = 0;
    while i < args.len() {
        let val = |i: &mut usize| -> String {
            *i += 1;
            args.get(*i).cloned().unwrap_or_default()
        };
        match args[i].as_str() {
            "--cone-half-angle" => {
                if let Ok(v) = val(&mut i).parse::<f32>() {
                    cfg.cone_half_angle = v.max(0.0);
                }
            }
            "--cone-origin-radius" => {
                if let Ok(v) = val(&mut i).parse::<f32>() {
                    cfg.cone_origin_radius = v.max(0.0);
                }
            }
            "--cone-max-t" => {
                if let Ok(v) = val(&mut i).parse::<f32>() {
                    cfg.max_t = v;
                }
            }
            "--cone-bounces" => {
                if let Ok(v) = val(&mut i).parse::<u32>() {
                    cfg.max_bounces = v.max(1);
                }
            }
            _ => {}
        }
        i += 1;
    }
    cfg
}

/// `--reflections on|off` (default `on`) / `--reflection-bounces N` /
/// `--reflection-fresnel-cutoff F` / `--reflection-max-t F`: multi-bounce
/// specular reflection tunables, driving
/// `migera::hybrid::extract::ReflectionConfig` — mirrors
/// `cone_trace_config_from_args`'s exact pattern.
fn reflection_config_from_args() -> ReflectionConfig {
    let mut cfg = ReflectionConfig::default();
    let args: Vec<String> = std::env::args().collect();
    let mut i = 0;
    while i < args.len() {
        let val = |i: &mut usize| -> String {
            *i += 1;
            args.get(*i).cloned().unwrap_or_default()
        };
        match args[i].as_str() {
            "--reflections" => {
                cfg.enabled = val(&mut i) != "off";
            }
            "--reflection-bounces" => {
                if let Ok(v) = val(&mut i).parse::<u32>() {
                    cfg.max_bounces = v.max(1);
                }
            }
            "--reflection-fresnel-cutoff" => {
                if let Ok(v) = val(&mut i).parse::<f32>() {
                    cfg.fresnel_cutoff = v.max(0.0);
                }
            }
            "--reflection-max-t" => {
                if let Ok(v) = val(&mut i).parse::<f32>() {
                    cfg.max_t = v;
                }
            }
            _ => {}
        }
        i += 1;
    }
    cfg
}

/// `--transmission on|off` (default `on`) / `--transmission-bounces N` /
/// `--transmission-fresnel-cutoff F` / `--transmission-max-t F`:
/// multi-bounce transmission/refraction tunables, driving
/// `migera::hybrid::extract::TransmissionConfig` — mirrors
/// `reflection_config_from_args`'s exact pattern.
fn transmission_config_from_args() -> TransmissionConfig {
    let mut cfg = TransmissionConfig::default();
    let args: Vec<String> = std::env::args().collect();
    let mut i = 0;
    while i < args.len() {
        let val = |i: &mut usize| -> String {
            *i += 1;
            args.get(*i).cloned().unwrap_or_default()
        };
        match args[i].as_str() {
            "--transmission" => {
                cfg.enabled = val(&mut i) != "off";
            }
            "--transmission-bounces" => {
                if let Ok(v) = val(&mut i).parse::<u32>() {
                    cfg.max_bounces = v.max(1);
                }
            }
            "--transmission-fresnel-cutoff" => {
                if let Ok(v) = val(&mut i).parse::<f32>() {
                    cfg.fresnel_cutoff = v.max(0.0);
                }
            }
            "--transmission-max-t" => {
                if let Ok(v) = val(&mut i).parse::<f32>() {
                    cfg.max_t = v;
                }
            }
            _ => {}
        }
        i += 1;
    }
    cfg
}

/// `--grain F` / `--vignette F` / `--aberration F`: post-tonemap "lens/
/// sensor" pass tunables, driving `migera::hybrid::extract::
/// HybridPostConfig` — mirrors `transmission_config_from_args`'s exact
/// pattern (no on/off flag, unlike reflection/transmission: this pass is
/// cosmetic-only and cheap enough to always run, see
/// `assets/shaders/hybrid_post.wgsl`'s own doc comment — a strength of
/// 0.0 already fully disables any one effect's visible contribution).
fn hybrid_post_config_from_args() -> HybridPostConfig {
    let mut cfg = HybridPostConfig::default();
    let args: Vec<String> = std::env::args().collect();
    let mut i = 0;
    while i < args.len() {
        let val = |i: &mut usize| -> String {
            *i += 1;
            args.get(*i).cloned().unwrap_or_default()
        };
        match args[i].as_str() {
            "--grain" => {
                if let Ok(v) = val(&mut i).parse::<f32>() {
                    cfg.grain_strength = v.max(0.0);
                }
            }
            "--vignette" => {
                if let Ok(v) = val(&mut i).parse::<f32>() {
                    cfg.vignette_strength = v.max(0.0);
                }
            }
            "--aberration" => {
                if let Ok(v) = val(&mut i).parse::<f32>() {
                    cfg.aberration_strength = v.max(0.0);
                }
            }
            _ => {}
        }
        i += 1;
    }
    cfg
}

/// `--denoise on|off` (default `on`): kill switch for the indirect-diffuse
/// edge-aware spatial blur pass, driving
/// `migera::hybrid::extract::DenoiseConfig` — mirrors
/// `shadow_config_from_args`'s exact pattern.
fn denoise_config_from_args() -> DenoiseConfig {
    let mut cfg = DenoiseConfig::default();
    let args: Vec<String> = std::env::args().collect();
    let mut i = 0;
    while i < args.len() {
        let val = |i: &mut usize| -> String {
            *i += 1;
            args.get(*i).cloned().unwrap_or_default()
        };
        if args[i].as_str() == "--denoise" {
            cfg.enabled = val(&mut i) != "off";
        }
        i += 1;
    }
    cfg
}

/// `--no-temporal` (temporal accumulation defaults to `on`) /
/// `--temporal-max-history F` (default `24.0`, matching
/// `TemporalConfig::default`'s own value): kill switch and history-length
/// cap for indirect-diffuse temporal accumulation, driving
/// `migera::hybrid::extract::TemporalConfig` — mirrors
/// `shadow_config_from_args`'s exact pattern.
fn temporal_config_from_args() -> TemporalConfig {
    let mut cfg = TemporalConfig::default();
    let args: Vec<String> = std::env::args().collect();
    let mut i = 0;
    while i < args.len() {
        let val = |i: &mut usize| -> String {
            *i += 1;
            args.get(*i).cloned().unwrap_or_default()
        };
        match args[i].as_str() {
            "--no-temporal" => cfg.enabled = false,
            "--temporal-max-history" => {
                if let Ok(v) = val(&mut i).parse::<f32>() {
                    cfg.max_history_length = v;
                }
            }
            _ => {}
        }
        i += 1;
    }
    cfg
}

/// `--jitter` (off by default — this is an experimental A/B toggle, not
/// yet a proven improvement, see `migera::hybrid::extract::JitterConfig`'s
/// own doc comment) / `--jitter-ring N` (default `64`): enables the
/// primary ray's own per-frame Halton(2,3) sub-pixel jitter and sets how
/// many frames its index is allowed to grow across before wrapping —
/// mirrors `shadow_config_from_args`'s exact pattern.
fn jitter_config_from_args() -> JitterConfig {
    let mut cfg = JitterConfig::default();
    let args: Vec<String> = std::env::args().collect();
    let mut i = 0;
    while i < args.len() {
        let val = |i: &mut usize| -> String {
            *i += 1;
            args.get(*i).cloned().unwrap_or_default()
        };
        match args[i].as_str() {
            "--jitter" => cfg.enabled = true,
            "--jitter-ring" => {
                if let Ok(v) = val(&mut i).parse::<u32>() {
                    cfg.ring_size = v;
                }
            }
            _ => {}
        }
        i += 1;
    }
    cfg
}

/// `--render-scale F` (default `1.0`, no scaling): sets the trace pass's
/// own working resolution to `F * real viewport size`, driving
/// `migera::hybrid::extract::RenderScaleConfig` — step 1b of the same
/// temporal-upscaling experiment `--jitter`/`jitter_config_from_args` is
/// step 1a of. Mirrors `shadow_config_from_args`'s exact pattern.
fn render_scale_config_from_args() -> RenderScaleConfig {
    let mut cfg = RenderScaleConfig::default();
    let args: Vec<String> = std::env::args().collect();
    let mut i = 0;
    while i < args.len() {
        let val = |i: &mut usize| -> String {
            *i += 1;
            args.get(*i).cloned().unwrap_or_default()
        };
        if args[i].as_str() == "--render-scale"
            && let Ok(v) = val(&mut i).parse::<f32>()
        {
            cfg.scale = v;
        }
        i += 1;
    }
    cfg
}

/// `--spin x,y,z` (radians/second per axis, default `0,0,0` — no rotation
/// on any axis unless requested). Each axis is fully independent; setting
/// one to zero leaves that axis static while the others still turn.
/// Live-adjustable via the egui "Controls" panel's three sliders — this
/// resource is the single shared state both the CLI flag and the panel
/// drive, same pattern as `DebugGizmos`.
#[derive(Resource, Clone, Copy, Default)]
struct ObjectSpin {
    rad_per_sec: Vec3,
}

impl ObjectSpin {
    fn from_args() -> Self {
        let mut cfg = Self::default();
        let args: Vec<String> = std::env::args().collect();
        let mut i = 0;
        while i < args.len() {
            let val = |i: &mut usize| -> String {
                *i += 1;
                args.get(*i).cloned().unwrap_or_default()
            };
            if args[i] == "--spin" {
                let s = val(&mut i);
                let mut it = s.split(',').map(|p| p.trim().parse::<f32>());
                if let (Some(Ok(x)), Some(Ok(y)), Some(Ok(z))) = (it.next(), it.next(), it.next()) {
                    cfg.rad_per_sec = Vec3::new(x, y, z);
                }
            }
            i += 1;
        }
        cfg
    }
}

/// `--rotate x,y,z` (degrees, default `0,0,0`): a FIXED initial rotation
/// applied to every `Spinning` object at spawn time (mirrors
/// `ObjectSpin::from_args`'s exact CLI-parsing pattern) — a debugging
/// aid for reproducing rotation-angle-dependent visual bugs at a known,
/// static angle instead of hunting through a live spin's continuously
/// changing frames. Composes with `--spin`: a nonzero `--spin` still
/// animates from this starting orientation.
#[derive(Resource, Clone, Copy, Default)]
struct InitialRotation {
    degrees: Vec3,
}

impl InitialRotation {
    fn from_args() -> Self {
        let mut cfg = Self::default();
        let args: Vec<String> = std::env::args().collect();
        let mut i = 0;
        while i < args.len() {
            let val = |i: &mut usize| -> String {
                *i += 1;
                args.get(*i).cloned().unwrap_or_default()
            };
            if args[i] == "--rotate" {
                let s = val(&mut i);
                let mut it = s.split(',').map(|p| p.trim().parse::<f32>());
                if let (Some(Ok(x)), Some(Ok(y)), Some(Ok(z))) = (it.next(), it.next(), it.next()) {
                    cfg.degrees = Vec3::new(x, y, z);
                }
            }
            i += 1;
        }
        cfg
    }

    fn as_quat(self) -> Quat {
        Quat::from_euler(EulerRot::XYZ, self.degrees.x.to_radians(), self.degrees.y.to_radians(), self.degrees.z.to_radians())
    }
}

/// `--cube-color r,g,b` (linear 0.0-1.0 components, default the cube's
/// original orange `0.85,0.35,0.20`) — the spinning cube's flat material
/// color, now that `src/hybrid`'s trace step actually renders it. Same
/// hand-rolled comma-split-parse convention as `ObjectSpin::from_args`, and
/// same "single shared state both the CLI flag and the egui panel drive"
/// pattern as `DebugGizmos`/`ObjectSpin` — the panel's color picker writes
/// straight into this resource, and `apply_cube_color` copies its current
/// value onto the cube entity's `Material` every frame.
#[derive(Resource, Clone, Copy)]
struct CubeColor(Vec3);

impl Default for CubeColor {
    fn default() -> Self {
        Self(Vec3::new(0.85, 0.35, 0.20))
    }
}

impl CubeColor {
    fn from_args() -> Self {
        let mut cfg = Self::default();
        let args: Vec<String> = std::env::args().collect();
        let mut i = 0;
        while i < args.len() {
            let val = |i: &mut usize| -> String {
                *i += 1;
                args.get(*i).cloned().unwrap_or_default()
            };
            if args[i] == "--cube-color" {
                let s = val(&mut i);
                let mut it = s.split(',').map(|p| p.trim().parse::<f32>());
                if let (Some(Ok(r)), Some(Ok(g)), Some(Ok(b))) = (it.next(), it.next(), it.next()) {
                    cfg.0 = Vec3::new(r, g, b);
                }
            }
            i += 1;
        }
        cfg
    }
}

/// `--cube-metallic F --cube-roughness F --cube-reflectance F --cube-emissive
/// r,g,b`: the spinning cube's PBR material knobs beyond base color, now
/// that `hybrid_trace.wgsl`'s `shade` implements the full metallic-roughness
/// GGX Cook-Torrance BRDF (see `hybrid::material::Material`'s doc comment
/// for what each field means). Same "single shared state both the CLI flag
/// and the egui panel drive" pattern as `CubeColor` — the panel's
/// sliders/color picker write straight into this resource, and
/// `apply_cube_color` copies its current value onto the cube entity's
/// `Material` every frame, alongside `CubeColor`'s base color.
#[derive(Resource, Clone, Copy)]
struct CubeMaterialParams {
    metallic: f32,
    roughness: f32,
    reflectance: f32,
    emissive: Vec3,
    transmission: f32,
    ior: f32,
}

impl Default for CubeMaterialParams {
    fn default() -> Self {
        // Matches spawn_scene's pre-existing hardcoded cube material
        // (metallic 0.0, roughness 0.4) so turning this resource on
        // doesn't change today's default look; reflectance/emissive
        // default to `Material::new`'s own defaults (0.5, ZERO).
        // transmission=0.0/ior=1.5 match Material::new's own defaults too
        // (opaque by default — turning transmission on is opt-in, same
        // "don't change today's default look" reasoning).
        Self { metallic: 0.0, roughness: 0.4, reflectance: 0.5, emissive: Vec3::ZERO, transmission: 0.0, ior: 1.5 }
    }
}

impl CubeMaterialParams {
    fn from_args() -> Self {
        let mut cfg = Self::default();
        let args: Vec<String> = std::env::args().collect();
        let mut i = 0;
        while i < args.len() {
            let val = |i: &mut usize| -> String {
                *i += 1;
                args.get(*i).cloned().unwrap_or_default()
            };
            match args[i].as_str() {
                "--cube-metallic" => {
                    if let Ok(v) = val(&mut i).parse::<f32>() {
                        cfg.metallic = v;
                    }
                }
                "--cube-roughness" => {
                    if let Ok(v) = val(&mut i).parse::<f32>() {
                        cfg.roughness = v;
                    }
                }
                "--cube-reflectance" => {
                    if let Ok(v) = val(&mut i).parse::<f32>() {
                        cfg.reflectance = v;
                    }
                }
                "--cube-emissive" => {
                    let s = val(&mut i);
                    let mut it = s.split(',').map(|p| p.trim().parse::<f32>());
                    if let (Some(Ok(r)), Some(Ok(g)), Some(Ok(b))) = (it.next(), it.next(), it.next()) {
                        cfg.emissive = Vec3::new(r, g, b);
                    }
                }
                "--cube-transmission" => {
                    if let Ok(v) = val(&mut i).parse::<f32>() {
                        cfg.transmission = v.clamp(0.0, 1.0);
                    }
                }
                "--cube-ior" => {
                    if let Ok(v) = val(&mut i).parse::<f32>() {
                        cfg.ior = v.max(1.0);
                    }
                }
                _ => {}
            }
            i += 1;
        }
        cfg
    }
}

/// The ground plane's own material — separate from `CubeMaterialParams`
/// since the ground is the surface a REFLECTION actually bounces off of
/// (the cube is what gets reflected). Added specifically so
/// `REFLECT_ROUGHNESS_GATE`'s own temporal-accumulation cutoff
/// (`hybrid_temporal.wgsl`) can be swept visually — before this, neither
/// example scene had a live-adjustable roughness on any REFLECTING
/// surface (only on the object being reflected), making that cutoff
/// impossible to validate perceptually. Defaults match `spawn_scene`'s
/// pre-existing hardcoded ground material exactly, so omitting these
/// flags changes nothing.
#[derive(Resource, Clone, Copy)]
struct FloorMaterialParams {
    roughness: f32,
    reflectance: f32,
}

impl Default for FloorMaterialParams {
    fn default() -> Self {
        Self { roughness: 0.6, reflectance: 0.5 }
    }
}

impl FloorMaterialParams {
    fn from_args() -> Self {
        let mut cfg = Self::default();
        let args: Vec<String> = std::env::args().collect();
        let mut i = 0;
        while i < args.len() {
            let val = |i: &mut usize| -> String {
                *i += 1;
                args.get(*i).cloned().unwrap_or_default()
            };
            match args[i].as_str() {
                "--floor-roughness" => {
                    if let Ok(v) = val(&mut i).parse::<f32>() {
                        cfg.roughness = v;
                    }
                }
                "--floor-reflectance" => {
                    if let Ok(v) = val(&mut i).parse::<f32>() {
                        cfg.reflectance = v;
                    }
                }
                _ => {}
            }
            i += 1;
        }
        cfg
    }
}

fn main() {
    // Bevy's default AssetPlugin resolves `assets/` relative to the
    // executable's own directory, not the process's cwd — wrong for a
    // `target/release/examples/gallery` binary, whose sibling `assets/`
    // (if any) isn't this crate's real one. Pin it to `<cwd>/assets`
    // explicitly, same fix `examples/gallery_legacy.rs` already applies,
    // so shader loads (`shaders/hybrid_trace.wgsl` etc.) resolve correctly
    // regardless of where the binary itself lives.
    let assets = std::env::current_dir().expect("cwd").join("assets").to_string_lossy().into_owned();

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "migera gallery".into(),
                present_mode: bevy::window::PresentMode::AutoNoVsync,
                ..default()
            }),
            ..default()
        }).set(AssetPlugin { file_path: assets, ..default() }))
        .add_plugins(FrameTimeDiagnosticsPlugin::default())
        // Real per-pass GPU timestamps (not CPU wall-clock around submit) —
        // not auto-installed by DefaultPlugins (Bevy only adds it under the
        // tracing-tracy Cargo feature, which this project doesn't enable),
        // but needs no device-feature request of its own: Bevy's default
        // WgpuSettings::Functionality priority already requests every
        // feature the adapter/backend advertises, TIMESTAMP_QUERY included
        // where the backend (Vulkan/DX12) supports it. On other backends
        // (Metal/WebGPU/WebGL2) the render/*/elapsed_gpu diagnostic paths
        // this populates simply won't exist — see `update_fps_hud`'s and
        // `log_debug_stats`'s handling, which treats them as optional.
        .add_plugins(RenderDiagnosticsPlugin)
        .add_plugins(EguiPlugin::default())
        .add_plugins(HybridRenderPlugin)
        .insert_resource(FpsGraph::default())
        .insert_resource(ShotConfig::from_args())
        .insert_resource(CameraConfig::from_args())
        .insert_resource(DebugGizmos::from_args())
        .insert_resource(ObjectSpin::from_args())
        .insert_resource(InitialRotation::from_args())
        .insert_resource(CubeColor::from_args())
        .insert_resource(CubeMaterialParams::from_args())
        .insert_resource(FloorMaterialParams::from_args())
        .insert_resource(StressConfig::from_args())
        .insert_resource(ShapeChoice::from_args())
        .insert_resource(light_toggles_from_args())
        .insert_resource(shadow_config_from_args())
        .insert_resource(gi_method_config_from_args())
        .insert_resource(ddgi_config_from_args())
        .insert_resource(cone_trace_config_from_args())
        .insert_resource(reflection_config_from_args())
        .insert_resource(transmission_config_from_args())
        .insert_resource(hybrid_post_config_from_args())
        .insert_resource(denoise_config_from_args())
        .insert_resource(temporal_config_from_args())
        .insert_resource(jitter_config_from_args())
        .insert_resource(render_scale_config_from_args())
        .insert_resource(DebugLogTimer::default())
        .add_systems(
            Startup,
            (spawn_camera, spawn_scene, spawn_lights, spawn_fps_hud, spawn_camera_hud, spawn_object_light_hud),
        )
        .add_systems(
            Update,
            (
                camera_controller,
                spin_objects,
                apply_cube_color,
                draw_axis_gizmos,
                // migera::hybrid::bvh::update_persistent_bvh (registered by
                // HybridRenderPlugin itself, in this same Update schedule)
                // refits the shared PersistentBvh resource this system
                // reads — pinned explicitly so gizmos always draw this
                // frame's BVH state, not last frame's.
                draw_object_aabb_gizmos.after(migera::hybrid::bvh::update_persistent_bvh),
                sample_fps,
                update_fps_hud,
                update_camera_hud,
                update_object_light_hud,
                log_debug_stats,
                auto_shot,
            ),
        )
        .add_systems(EguiPrimaryContextPass, controls_panel)
        .run();
}

// ---------------------------------------------------------------------------
// egui controls panel: foldable window, top-right corner.
// ---------------------------------------------------------------------------

#[allow(clippy::too_many_arguments)]
fn controls_panel(
    mut contexts: EguiContexts,
    mut gizmos_cfg: ResMut<DebugGizmos>,
    mut spin: ResMut<ObjectSpin>,
    mut cube_color: ResMut<CubeColor>,
    mut material_params: ResMut<CubeMaterialParams>,
    mut light_toggles: ResMut<LightToggles>,
    mut shadow_config: ResMut<ShadowConfig>,
    mut gi_method: ResMut<GiMethodConfig>,
    mut ddgi_config: ResMut<DdgiConfig>,
    mut conetrace_config: ResMut<ConeTraceConfig>,
    mut reflection_config: ResMut<ReflectionConfig>,
    mut transmission_config: ResMut<TransmissionConfig>,
    mut hybrid_post_config: ResMut<HybridPostConfig>,
    mut dof_config: ResMut<DofConfig>,
    mut denoise_config: ResMut<DenoiseConfig>,
    mut temporal_config: ResMut<TemporalConfig>,
) -> Result {
    let ctx = contexts.ctx_mut()?;
    egui::Window::new("Controls")
        .anchor(egui::Align2::RIGHT_TOP, egui::vec2(-10.0, 10.0))
        .collapsible(true)
        .resizable(false)
        .show(ctx, |ui| {
            ui.checkbox(&mut gizmos_cfg.axes, "Axis gizmos");
            ui.checkbox(&mut gizmos_cfg.aabbs, "AABB/BVH gizmos");
            ui.separator();
            ui.label("Object spin (rad/s)");
            ui.add(egui::Slider::new(&mut spin.rad_per_sec.x, -3.0..=3.0).text("X"));
            ui.add(egui::Slider::new(&mut spin.rad_per_sec.y, -3.0..=3.0).text("Y"));
            ui.add(egui::Slider::new(&mut spin.rad_per_sec.z, -3.0..=3.0).text("Z"));
            ui.separator();
            ui.label("Cube material");
            ui.horizontal(|ui| {
                ui.label("Base color");
                let mut rgb: [f32; 3] = cube_color.0.into();
                ui.color_edit_button_rgb(&mut rgb);
                cube_color.0 = Vec3::from(rgb);
            });
            ui.add(egui::Slider::new(&mut material_params.metallic, 0.0..=1.0).text("Metallic"));
            ui.add(egui::Slider::new(&mut material_params.roughness, 0.0..=1.0).text("Roughness"));
            ui.add(egui::Slider::new(&mut material_params.reflectance, 0.0..=1.0).text("Reflectance"));
            ui.horizontal(|ui| {
                ui.label("Emissive");
                let mut rgb: [f32; 3] = material_params.emissive.into();
                ui.color_edit_button_rgb(&mut rgb);
                material_params.emissive = Vec3::from(rgb);
            });
            ui.add(egui::Slider::new(&mut material_params.transmission, 0.0..=1.0).text("Transmission"));
            ui.add_enabled(
                material_params.transmission > 0.0,
                egui::Slider::new(&mut material_params.ior, 1.0..=2.5).text("IOR"),
            );
            ui.separator();
            ui.label("Lights");
            ui.checkbox(&mut light_toggles.sun, "Sun (directional)");
            ui.checkbox(&mut light_toggles.lamp, "Lamp (point)");
            ui.checkbox(&mut light_toggles.projector, "Projector (spot)");
            ui.separator();
            ui.label("Shadows");
            ui.checkbox(&mut shadow_config.enabled, "Soft shadows");
            ui.add_enabled(
                shadow_config.enabled,
                egui::Slider::new(&mut shadow_config.k, 1.0..=64.0).text("Softness (k)"),
            );
            ui.separator();
            ui.label("GI method");
            ui.horizontal(|ui| {
                ui.radio_value(&mut gi_method.method, GiMethod::None, "None");
                ui.radio_value(&mut gi_method.method, GiMethod::Ddgi, "DDGI");
                ui.radio_value(&mut gi_method.method, GiMethod::RadianceCascades, "Radiance Cascades (experimental)");
                ui.radio_value(&mut gi_method.method, GiMethod::ConeTrace, "Cone-trace");
            });
            let gi_active = gi_method.method != GiMethod::None;
            let ddgi_active = gi_method.method == GiMethod::Ddgi;
            let conetrace_active = gi_method.method == GiMethod::ConeTrace;
            ui.add_enabled(
                ddgi_active,
                egui::Slider::new(&mut ddgi_config.probes_per_frame, 1..=4096).text("Probes relit/frame"),
            );
            ui.add_enabled(ddgi_active, egui::Slider::new(&mut ddgi_config.max_t, 1.0..=120.0).text("Probe ray reach"));
            ui.add_enabled(
                ddgi_active,
                egui::Slider::new(&mut ddgi_config.max_history_length, 1.0..=64.0).text("Probe history length"),
            );
            ui.add_enabled(
                conetrace_active,
                egui::Slider::new(&mut conetrace_config.cone_half_angle, 0.01..=0.6).text("Cone half-angle (rad)"),
            );
            ui.add_enabled(
                conetrace_active,
                egui::Slider::new(&mut conetrace_config.cone_origin_radius, 0.0..=1.0).text("Cone origin radius"),
            );
            ui.add_enabled(
                conetrace_active,
                egui::Slider::new(&mut conetrace_config.max_t, 1.0..=120.0).text("Cone reach"),
            );
            ui.add_enabled(
                conetrace_active,
                egui::Slider::new(&mut conetrace_config.max_bounces, 1..=8).text("Bounces"),
            );
            ui.add_enabled(gi_active, egui::Checkbox::new(&mut denoise_config.enabled, "Denoise (edge-aware blur)"));
            ui.add_enabled(gi_active, egui::Checkbox::new(&mut temporal_config.enabled, "Temporal accumulation"));
            ui.add_enabled(
                gi_active && temporal_config.enabled,
                egui::Slider::new(&mut temporal_config.max_history_length, 1.0..=64.0).text("History length"),
            );
            ui.separator();
            ui.label("Reflections");
            ui.checkbox(&mut reflection_config.enabled, "Multi-bounce specular reflections");
            ui.add_enabled(
                reflection_config.enabled,
                egui::Slider::new(&mut reflection_config.max_bounces, 1..=4).text("Bounces"),
            );
            ui.add_enabled(
                reflection_config.enabled,
                egui::Slider::new(&mut reflection_config.fresnel_cutoff, 0.0..=0.2).text("Fresnel cutoff"),
            );
            ui.add_enabled(
                reflection_config.enabled,
                egui::Slider::new(&mut reflection_config.max_t, 1.0..=120.0).text("Reach"),
            );
            ui.separator();
            ui.label("Transmission");
            ui.checkbox(&mut transmission_config.enabled, "Multi-bounce transmission/refraction");
            ui.add_enabled(
                transmission_config.enabled,
                egui::Slider::new(&mut transmission_config.max_bounces, 1..=4).text("Bounces"),
            );
            ui.add_enabled(
                transmission_config.enabled,
                egui::Slider::new(&mut transmission_config.fresnel_cutoff, 0.0..=0.2).text("Fresnel cutoff"),
            );
            ui.add_enabled(
                transmission_config.enabled,
                egui::Slider::new(&mut transmission_config.max_t, 1.0..=120.0).text("Reach"),
            );
            ui.separator();
            ui.label("Lens / sensor");
            ui.add(egui::Slider::new(&mut hybrid_post_config.grain_strength, 0.0..=0.3).text("Film grain"));
            ui.add(egui::Slider::new(&mut hybrid_post_config.vignette_strength, 0.0..=1.0).text("Vignette"));
            ui.add(egui::Slider::new(&mut hybrid_post_config.aberration_strength, 0.0..=0.02).text("Chromatic aberration"));
            ui.checkbox(&mut dof_config.enabled, "Depth of field (stochastic)");
            ui.add_enabled(
                dof_config.enabled,
                egui::Slider::new(&mut dof_config.focal_distance, 0.5..=40.0).text("Focal distance"),
            );
            ui.add_enabled(
                dof_config.enabled,
                egui::Slider::new(&mut dof_config.aperture_f_stops, 0.5..=16.0).text("Aperture f-stop"),
            );
            ui.add_enabled(
                dof_config.enabled,
                egui::Slider::new(&mut dof_config.max_history_length, 4.0..=64.0).text("Convergence window (frames)"),
            );
        });
    Ok(())
}

fn auto_shot(
    cfg: Res<ShotConfig>,
    frame: Res<FrameCount>,
    mut commands: Commands,
    mut fired: Local<bool>,
    mut exited: Local<bool>,
) {
    let Some((path, at)) = cfg.shot.clone() else {
        return;
    };
    if !*fired && frame.0 >= at {
        *fired = true;
        info!("gallery: screenshot -> {path}");
        commands.spawn(Screenshot::primary_window()).observe(save_to_disk(path.clone()));
    }
    if *fired && !*exited && frame.0 >= at + 60 {
        *exited = true;
        std::thread::sleep(std::time::Duration::from_millis(600));
        std::process::exit(0);
    }
}

fn spawn_camera(mut commands: Commands) {
    // Msaa::Off: the hybrid blit pipeline's RenderPipelineDescriptor uses
    // MultisampleState::default() (sample count 1) — Bevy's default camera
    // MSAA (sample count 4) makes the view target's color attachment
    // incompatible with that pipeline (a real wgpu validation error, not
    // just a warning). Same fix `examples/gallery_legacy.rs` already
    // applies for the same reason.
    commands.spawn((
        Camera3d::default(),
        Msaa::Off,
        // See gi_room.rs's spawn_camera's own doc comment for why Hdr +
        // Tonemapping are required for hybrid_pass's linear-HDR blit
        // output to ever reach Bevy's Node3d::Tonemapping graph node.
        Hdr,
        Tonemapping::default(),
        Exposure::default(),
        Transform::from_xyz(6.0, 5.0, 8.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}

// ---------------------------------------------------------------------------
// Lights: exactly 3 fixed scene-global sources (sun/lamp/projector), spawned
// ONCE regardless of `--stress N` — a stress-tested scene has many object
// cells but always exactly one of each light, not one per cell (lighting
// the whole scene, not scaling with object count is the whole point of a
// "3 fixed lights" spec). Each gets a distinct color and a different
// distance from the origin so their individual contributions are visually
// distinguishable from one another. Marker components
// (`SunLight`/`LampLight`/`ProjectorLight`, defined in `migera::hybrid::
// extract`) let both extraction and this example's own debug HUD/log find
// each specific entity.
// ---------------------------------------------------------------------------

fn spawn_lights(mut commands: Commands) {
    // Sun: warm white directional light, angled down and to the side —
    // no meaningful "distance from origin" for a directional light (it's
    // parallel rays from infinitely far away), so its Transform's
    // translation is arbitrary and only its rotation (via `forward()`)
    // matters for shading. Deliberately dimmed relative to its Bevy
    // default-daylight-scale illuminance (10,000 lux) so it doesn't wash
    // out the lamp/projector's own contributions when all three combine —
    // a directional light has no distance falloff at all, so even a
    // modest illuminance reads as "flood the whole scene" next to a
    // point/spot light's naturally localized falloff.
    commands.spawn((
        SunLight,
        DirectionalLight { color: Color::srgb(1.0, 0.95, 0.85), illuminance: 1500.0, ..default() },
        Transform::from_xyz(0.0, 10.0, 0.0).looking_at(Vec3::new(-3.0, 0.0, -2.0), Vec3::Y),
    ));

    // Lamp: cool blue-white point light, moderately close (5m). Bumped up
    // from its original intensity so its contribution reads clearly
    // alongside the now-dimmer sun.
    commands.spawn((
        LampLight,
        PointLight { color: Color::srgb(0.7, 0.85, 1.0), intensity: 350_000.0, range: 20.0, ..default() },
        Transform::from_xyz(4.0, 3.0, 0.0),
    ));

    // Projector: warm orange/pink spotlight, farther out (9m), aimed back
    // toward the origin at a steep angle — visually distinct color and
    // distance from the lamp so all three lights' individual
    // contributions are separable by eye. Bumped up alongside the lamp,
    // same reasoning.
    commands.spawn((
        ProjectorLight,
        SpotLight {
            color: Color::srgb(1.0, 0.6, 0.75),
            intensity: 1_300_000.0,
            range: 25.0,
            inner_angle: 0.3,
            outer_angle: 0.5,
            ..default()
        },
        Transform::from_xyz(-6.0, 6.0, 6.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}

// ---------------------------------------------------------------------------
// Scene: a ground box (real thickness, not a paper-thin plane) and a cube —
// no SDF marching/rendering yet, this step is only about proving the
// AABB/BVH acceleration structure (`migera::hybrid::{scene, bvh}`) is
// correct against real, potentially-rotating geometry. Uses the project's
// existing `sdf::components::Shape`/`sdf::assembly::SdfSceneRoot`
// authoring components — nothing new invented here.
// ---------------------------------------------------------------------------

/// Marks an entity whose `Transform` should spin continuously per
/// `ObjectSpin`'s per-axis rates — applied to the cube, not the ground.
#[derive(Component)]
struct Spinning;

/// Ground box half-extents (X/Z) — the footprint `StressConfig::CELL_GAP`
/// tiles with real gaps between adjacent cells. `Y` stays a real 0.4m
/// thick slab (not a paper-thin plane), top face at each cell's own
/// local `y = 0`.
const GROUND_HALF_EXTENT: Vec3 = Vec3::new(4.0, 0.2, 4.0);

/// Spawns `stress.count` independent scene copies — each its own two
/// `SdfSceneRoot`s (ground, spinning object; matches `examples/
/// gallery_legacy.rs`'s own convention of giving the ground and each
/// piece separate roots, not one root's CSG tree standing in for "the
/// scene") — on a grid, so `--stress N` is a real multi-object BVH-culled
/// scene to measure the trace pipeline against, not a single object
/// repeated via SDF-level tiling. `--stress 1` (the default) places its
/// one cell at the origin, reproducing today's exact non-stress layout.
/// The spinning object's shape is `shape_choice`'s pick (`--shape <name>`,
/// default `box` — today's `RoundedBox` cube, unchanged) — every cell
/// spawns the same chosen shape, not a mix.
fn spawn_scene(
    mut commands: Commands,
    cube_color: Res<CubeColor>,
    stress: Res<StressConfig>,
    shape_choice: Res<ShapeChoice>,
    initial_rotation: Res<InitialRotation>,
    floor_material: Res<FloorMaterialParams>,
) {
    let dim = stress_grid_dim(stress.count);
    let cell_size = GROUND_HALF_EXTENT.x * 2.0 + StressConfig::CELL_GAP;
    let (shape, object_height) = shape_choice.shape_and_height();
    let rotation = initial_rotation.as_quat();

    for i in 0..stress.count {
        let center = stress_cell_center(i, dim, cell_size);

        let ground_root =
            commands.spawn((SdfSceneRoot, Transform::from_translation(center), Visibility::default())).id();
        commands.spawn((
            ChildOf(ground_root),
            Shape::RoundedBox { half_extents: GROUND_HALF_EXTENT, corner_radius: 0.0 },
            Transform::from_xyz(0.0, -GROUND_HALF_EXTENT.y, 0.0),
            Material::new(Vec3::new(0.45, 0.46, 0.48), 0.0, floor_material.roughness).with_reflectance(floor_material.reflectance),
        ));

        let object_root =
            commands.spawn((SdfSceneRoot, Transform::from_translation(center), Visibility::default())).id();
        commands.spawn((
            ChildOf(object_root),
            shape,
            Transform::from_xyz(0.0, object_height, 0.0).with_rotation(rotation),
            Material::new(cube_color.0, 0.0, 0.4),
            Spinning,
        ));
    }
}

fn spin_objects(time: Res<Time>, spin: Res<ObjectSpin>, mut spinning: Query<&mut Transform, With<Spinning>>) {
    let d = spin.rad_per_sec * time.delta_secs();
    let delta = Quat::from_euler(EulerRot::XYZ, d.x, d.y, d.z);
    for mut transform in &mut spinning {
        transform.rotation = (delta * transform.rotation).normalize();
    }
}

/// Applies `CubeColor`'s and `CubeMaterialParams`'s current values to the
/// cube entity's `Material` every frame — cheap at one entity, and simpler
/// than change-detection-gating for a single-object demo scene. Finds the
/// cube via `Spinning`, which is already unique to it (see `spawn_scene`).
fn apply_cube_color(
    cube_color: Res<CubeColor>,
    material_params: Res<CubeMaterialParams>,
    mut cubes: Query<&mut Material, With<Spinning>>,
) {
    for mut material in &mut cubes {
        material.base_color = cube_color.0;
        material.metallic = material_params.metallic;
        material.roughness = material_params.roughness;
        material.reflectance = material_params.reflectance;
        material.emissive = material_params.emissive;
        material.transmission = material_params.transmission;
        material.ior = material_params.ior;
    }
}

// ---------------------------------------------------------------------------
// AABB/BVH debug gizmos: draws each collected object's world-space AABB
// (via `migera::hybrid::scene::collect`/`world_aabb`) plus every BVH
// internal-node box (via `migera::hybrid::bvh::Bvh::build`), so the
// acceleration structure's correctness is visually checkable every frame
// as the cube spins — a naive rotation-ignoring AABB would visibly clip
// through the cube's corners; a correct one always fully contains it.
// Object boxes and BVH-node boxes are drawn in different colors so the
// tree structure (not just per-leaf bounds) is distinguishable on screen.
// ---------------------------------------------------------------------------

fn draw_object_aabb_gizmos(
    mut gizmos: Gizmos,
    cfg: Res<DebugGizmos>,
    bvh: Res<migera::hybrid::bvh::PersistentBvh>,
    roots: Query<Entity, With<SdfSceneRoot>>,
    shapes: Query<(Entity, &Shape, &GlobalTransform, Option<&ChildOf>)>,
) {
    if !cfg.aabbs {
        return;
    }
    let objects = migera::hybrid::scene::collect(&roots, &shapes);
    if objects.is_empty() {
        return;
    }

    const OBJECT_COLOR: Color = Color::srgb(1.0, 0.85, 0.15);
    for object in &objects {
        draw_aabb_box(&mut gizmos, object.world_aabb, OBJECT_COLOR);
    }

    const NODE_COLOR: Color = Color::srgba(0.3, 0.7, 1.0, 0.5);
    for node in &bvh.0.nodes {
        if node.left_or_sentinel != migera::hybrid::bvh::LEAF_SENTINEL {
            draw_aabb_box(&mut gizmos, node.aabb, NODE_COLOR);
        }
    }
}

fn draw_aabb_box(gizmos: &mut Gizmos, aabb: migera::prim::Aabb, color: Color) {
    let center = (aabb.min + aabb.max) * 0.5;
    let size = aabb.max - aabb.min;
    gizmos.primitive_3d(&Cuboid::from_size(size), Isometry3d::from_translation(center), color);
}

// ---------------------------------------------------------------------------
// Camera controller: orbit (near/far radius presets, or an explicit meters
// value) or manual (pinned position/rotation from CLI params) — see
// `CameraConfig`'s doc comment for the exact flags.
// ---------------------------------------------------------------------------

fn camera_controller(time: Res<Time>, cfg: Res<CameraConfig>, mut cams: Query<&mut Transform, With<Camera3d>>) {
    match cfg.mode {
        CameraMode::Manual => {
            let rot = Quat::from_euler(
                EulerRot::YXZ,
                cfg.manual_rot_deg.y.to_radians(),
                cfg.manual_rot_deg.x.to_radians(),
                cfg.manual_rot_deg.z.to_radians(),
            );
            for mut transform in &mut cams {
                transform.translation = cfg.manual_pos;
                transform.rotation = rot;
            }
        }
        CameraMode::Orbit => {
            let a = (time.elapsed_secs() * cfg.orbit_speed) % TAU;
            let r = cfg.orbit_radius;
            let pos = Vec3::new(r * a.cos(), r * 0.4, r * a.sin());
            for mut transform in &mut cams {
                transform.translation = pos;
                *transform = transform.looking_at(Vec3::ZERO, Vec3::Y);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Axis gizmos: X/Y/Z lines through the origin, red/green/blue by
// convention, so the origin is visible as their crossing point. Cheap —
// three `gizmos.line()` calls per frame, no scene content to draw yet.
// ---------------------------------------------------------------------------

const AXIS_HALF_LENGTH: f32 = 1000.0;

fn draw_axis_gizmos(mut gizmos: Gizmos, cfg: Res<DebugGizmos>) {
    if !cfg.axes {
        return;
    }
    let e = Vec3::splat(AXIS_HALF_LENGTH);
    gizmos.line(-Vec3::X * e.x, Vec3::X * e.x, Color::srgb(0.9, 0.2, 0.2));
    gizmos.line(-Vec3::Y * e.y, Vec3::Y * e.y, Color::srgb(0.2, 0.9, 0.2));
    gizmos.line(-Vec3::Z * e.z, Vec3::Z * e.z, Color::srgb(0.3, 0.4, 0.95));
}

// ---------------------------------------------------------------------------
// FPS graph (on-screen bars) + stats text, top-left corner
// ---------------------------------------------------------------------------

#[derive(Resource, Default)]
struct FpsGraph {
    /// Delta time in milliseconds, newest sample last.
    samples: VecDeque<f32>,
}

fn sample_fps(time: Res<Time>, mut graph: ResMut<FpsGraph>) {
    graph.samples.push_back(time.delta_secs_f64() as f32 * 1000.0);
    while graph.samples.len() > FPS_BARS {
        graph.samples.pop_front();
    }
}

#[derive(Component)]
struct FpsStatsText;

#[derive(Component)]
struct FpsBar(usize);

fn spawn_fps_hud(mut commands: Commands) {
    commands.spawn((
        Text::new("fps —"),
        TextFont { font_size: FontSize::Px(13.0), ..default() },
        TextColor(Color::srgb(0.88, 0.92, 0.96)),
        Node {
            position_type: PositionType::Absolute,
            left: Val::Px(10.0),
            top: Val::Px(10.0),
            ..default()
        },
        FpsStatsText,
    ));

    let graph_root = commands
        .spawn((
            Node {
                position_type: PositionType::Absolute,
                left: Val::Px(10.0),
                top: Val::Px(40.0),
                width: Val::Px((FPS_BARS * 5) as f32),
                height: Val::Px(64.0),
                ..default()
            },
            BackgroundColor(Color::NONE),
        ))
        .id();
    for i in 0..FPS_BARS {
        commands.spawn((
            ChildOf(graph_root),
            Node {
                position_type: PositionType::Absolute,
                left: Val::Px((i * 5) as f32),
                bottom: Val::Px(0.0),
                width: Val::Px(4.0),
                height: Val::Px(0.0),
                ..default()
            },
            BackgroundColor(Color::srgb(0.20, 0.85, 0.45)),
            FpsBar(i),
        ));
    }
}

/// `RenderDiagnosticsPlugin`'s GPU timestamp paths for `src/hybrid/pass.rs`'s
/// three spans — `render/<name>/elapsed_gpu` is the path shape
/// `RecordDiagnostics::pass_span` produces (see that module's own comment
/// for why these two specific names). `None` (not just `0.0`) when the
/// active backend doesn't support timestamp queries (Metal/WebGPU/WebGL2 —
/// see `RenderDiagnosticsPlugin`'s own doc comment) or the diagnostic
/// hasn't smoothed a value yet, so callers can render "gpu: n/a" instead
/// of a misleadingly precise zero.
fn gpu_pass_ms(diagnostics: &DiagnosticsStore, pass_name: &str) -> Option<f32> {
    let path = DiagnosticPath::from_components(["render", pass_name, "elapsed_gpu"]);
    diagnostics.get(&path).and_then(|d| d.smoothed()).map(|v| v as f32)
}

/// FPS/frame-time stats over the current `FpsGraph` window, shared by the
/// on-screen HUD text and the once-a-second log line so both report the
/// same numbers. `gpu_trace_ms`/`gpu_blit_ms` are real per-pass GPU-only
/// time (via `RenderDiagnosticsPlugin`'s timestamp queries) — distinct
/// from `frame_ms`, which is CPU wall-clock covering the whole frame
/// (extraction + GPU submit + egui + HUD + present, all conflated). Both
/// numbers are shown side by side deliberately: neither one alone answers
/// "is this renderer close to a performance ceiling" — that question
/// needs the GPU-only number isolated from everything else sharing the
/// frame budget.
struct FpsStats {
    fps: f32,
    frame_ms: f32,
    avg_ms: f32,
    min_ms: f32,
    max_ms: f32,
    gpu_trace_ms: Option<f32>,
    gpu_temporal_ms: Option<f32>,
    gpu_reflect_temporal_ms: Option<f32>,
    gpu_transmit_temporal_ms: Option<f32>,
    gpu_denoise_ms: Option<f32>,
    gpu_blit_ms: Option<f32>,
    gpu_post_ms: Option<f32>,
    gpu_ddgi_ms: Option<f32>,
    gpu_radiance_cascades_ms: Option<f32>,
}

impl FpsStats {
    fn compute(diagnostics: &DiagnosticsStore, graph: &FpsGraph) -> Self {
        let fps = diagnostics
            .get(&FrameTimeDiagnosticsPlugin::FPS)
            .and_then(|d| d.smoothed())
            .unwrap_or(0.0);
        let (avg_ms, min_ms, max_ms) = if graph.samples.is_empty() {
            (0.0, 0.0, 0.0)
        } else {
            let avg = graph.samples.iter().sum::<f32>() / graph.samples.len() as f32;
            let min = graph.samples.iter().copied().fold(f32::MAX, f32::min);
            let max = graph.samples.iter().copied().fold(f32::MIN, f32::max);
            (avg, min, max)
        };
        let frame_ms = graph.samples.back().copied().unwrap_or(0.0);
        let gpu_trace_ms = gpu_pass_ms(diagnostics, "hybrid_trace");
        let gpu_temporal_ms = gpu_pass_ms(diagnostics, "hybrid_temporal");
        let gpu_reflect_temporal_ms = gpu_pass_ms(diagnostics, "hybrid_reflect_temporal");
        let gpu_transmit_temporal_ms = gpu_pass_ms(diagnostics, "hybrid_transmit_temporal");
        let gpu_denoise_ms = gpu_pass_ms(diagnostics, "hybrid_denoise");
        let gpu_blit_ms = gpu_pass_ms(diagnostics, "hybrid_blit");
        let gpu_post_ms = gpu_pass_ms(diagnostics, "hybrid_post");
        let gpu_ddgi_ms = gpu_pass_ms(diagnostics, "hybrid_ddgi");
        let gpu_radiance_cascades_ms = gpu_pass_ms(diagnostics, "hybrid_radiance_cascades");
        Self {
            fps: fps as f32,
            frame_ms,
            avg_ms,
            min_ms,
            max_ms,
            gpu_trace_ms,
            gpu_temporal_ms,
            gpu_reflect_temporal_ms,
            gpu_transmit_temporal_ms,
            gpu_denoise_ms,
            gpu_blit_ms,
            gpu_post_ms,
            gpu_ddgi_ms,
            gpu_radiance_cascades_ms,
        }
    }
}

impl std::fmt::Display for FpsStats {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "fps {:5.0}   frame {:.2} ms   avg {:.1}  min {:.1}  max {:.0}",
            self.fps, self.frame_ms, self.avg_ms, self.min_ms, self.max_ms
        )?;
        // Core four (trace/temporal/denoise/blit) always dispatch every
        // frame, so an absent value there means the backend has no
        // timestamp-query support at all — fall back to "gpu n/a"
        // wholesale. Cone tracing has no separate GPU pass of its own
        // (see conetrace_ref.rs's own doc comment) — its entire cost
        // shows up as a delta in the trace line below relative to
        // GiMethod::None's own baseline trace cost, not as a distinct
        // timing line here.
        match (self.gpu_trace_ms, self.gpu_temporal_ms, self.gpu_denoise_ms, self.gpu_blit_ms) {
            (Some(trace), Some(temporal), Some(denoise), Some(blit)) => {
                write!(f, "   gpu trace {trace:.2} ms  temporal {temporal:.2} ms  denoise {denoise:.2} ms  blit {blit:.2} ms")?;
            }
            _ => write!(f, "   gpu n/a")?,
        }
        // Reflection's own temporal-accumulation pass — a genuinely
        // separate dispatch (unlike cone tracing's own indirect-diffuse
        // cost, which has no pass of its own), shown as its own optional
        // suffix rather than folded into the core-four match above so
        // this HUD line's shape stays stable whether or not reflections
        // are compiled/enabled.
        if let Some(reflect_temporal) = self.gpu_reflect_temporal_ms {
            write!(f, "  reflect-temporal {reflect_temporal:.2} ms")?;
        }
        // Transmission's own temporal-accumulation pass — same "genuinely
        // separate dispatch, optional suffix" reasoning as reflect-temporal
        // above.
        if let Some(transmit_temporal) = self.gpu_transmit_temporal_ms {
            write!(f, "  transmit-temporal {transmit_temporal:.2} ms")?;
        }
        // Post-tonemap lens/sensor pass — same "genuinely separate
        // dispatch, optional suffix" reasoning as reflect/transmit-
        // temporal above. Runs even when grain/vignette/aberration are
        // all 0.0 (see HybridPostConfig's own doc comment on why), so
        // this line's presence just reflects whether Hdr+Tonemapping are
        // on the camera (see HybridRenderPlugin's own doc comment), not
        // whether any effect is currently visible.
        if let Some(post) = self.gpu_post_ms {
            write!(f, "  post {post:.2} ms")?;
        }
        // DDGI's own probe-relight pass — same "genuinely separate
        // dispatch, optional suffix" reasoning as reflect/transmit-
        // temporal above. Absent entirely (not just zero) whenever a
        // different GiMethod is active, since the dispatch itself is
        // skipped — see pass::hybrid_pass's own dispatch-level gate.
        if let Some(ddgi) = self.gpu_ddgi_ms {
            write!(f, "  ddgi {ddgi:.2} ms")?;
        }
        // Radiance Cascades' own relight pass — same "genuinely separate
        // dispatch, optional suffix" reasoning as ddgi immediately above.
        // Absent entirely (not just zero) whenever a different GiMethod
        // is active, since the dispatch itself is skipped — see
        // pass::hybrid_pass's own dispatch-level gate. This is the number
        // Stage 4's own DDGI-vs-cascades perf comparison reads.
        if let Some(radiance_cascades) = self.gpu_radiance_cascades_ms {
            write!(f, "  radiance-cascades {radiance_cascades:.2} ms")?;
        }
        Ok(())
    }
}

/// Frame-time percentiles over the current `FpsGraph` window, log-only (the
/// on-screen HUD stays avg/min/max — screen space is tight, the log has
/// room for the fuller picture).
struct FpsPercentiles {
    p25_ms: f32,
    p50_ms: f32,
    p75_ms: f32,
    p95_ms: f32,
    p99_ms: f32,
}

impl FpsPercentiles {
    fn compute(graph: &FpsGraph) -> Self {
        let mut sorted: Vec<f32> = graph.samples.iter().copied().collect();
        sorted.sort_by(|a, b| a.total_cmp(b));
        let at = |q: f32| -> f32 {
            if sorted.is_empty() {
                return 0.0;
            }
            let idx = ((sorted.len() - 1) as f32 * q).round() as usize;
            sorted[idx]
        };
        Self {
            p25_ms: at(0.25),
            p50_ms: at(0.50),
            p75_ms: at(0.75),
            p95_ms: at(0.95),
            p99_ms: at(0.99),
        }
    }

    fn iqr_ms(&self) -> f32 {
        self.p75_ms - self.p25_ms
    }
}

impl std::fmt::Display for FpsPercentiles {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "p25 {:.1}  p50 {:.1}  p75 {:.1}  p95 {:.1}  p99 {:.1}  iqr {:.1}",
            self.p25_ms,
            self.p50_ms,
            self.p75_ms,
            self.p95_ms,
            self.p99_ms,
            self.iqr_ms()
        )
    }
}

fn update_fps_hud(
    diagnostics: Res<DiagnosticsStore>,
    graph: Res<FpsGraph>,
    mut text: Query<&mut Text, With<FpsStatsText>>,
    mut bars: Query<(&FpsBar, &mut Node)>,
) {
    let stats = FpsStats::compute(&diagnostics, &graph);

    if let Ok(mut text) = text.single_mut() {
        **text = stats.to_string();
    }

    for (bar, mut node) in &mut bars {
        let idx = graph.samples.len().saturating_sub(bar.0 + 1);
        let ms = graph.samples.get(idx).copied().unwrap_or(0.0);
        node.height = Val::Px((ms / 40.0 * 64.0).clamp(2.0, 64.0));
    }
}

// ---------------------------------------------------------------------------
// Camera/frame HUD: bottom-left corner. Camera position, rotation (Euler
// degrees, same YXZ convention `CameraConfig::manual_rot_deg` uses), the
// current frame number, elapsed time since startup, and the latest frame
// time — so the exact camera pose and frame are always visible on screen,
// not just in the log.
// ---------------------------------------------------------------------------

#[derive(Component)]
struct CameraStatsText;

fn spawn_camera_hud(mut commands: Commands) {
    commands.spawn((
        Text::new("camera —"),
        TextFont { font_size: FontSize::Px(13.0), ..default() },
        TextColor(Color::srgb(0.88, 0.92, 0.96)),
        Node {
            position_type: PositionType::Absolute,
            left: Val::Px(10.0),
            bottom: Val::Px(10.0),
            ..default()
        },
        TextLayout::justify(Justify::Left),
        CameraStatsText,
    ));
}

fn camera_pose_line(transform: &Transform, frame: u32, elapsed_secs: f32, frame_ms: f32) -> String {
    let (yaw, pitch, roll) = transform.rotation.to_euler(EulerRot::YXZ);
    format!(
        "frame {frame}   t {elapsed_secs:.2}s   {frame_ms:.2} ms\n\
         pos    ({:6.2}, {:6.2}, {:6.2})\n\
         rot    (pitch {:6.1}  yaw {:6.1}  roll {:6.1})",
        transform.translation.x,
        transform.translation.y,
        transform.translation.z,
        pitch.to_degrees(),
        yaw.to_degrees(),
        roll.to_degrees(),
    )
}

fn update_camera_hud(
    time: Res<Time>,
    frame: Res<FrameCount>,
    graph: Res<FpsGraph>,
    cams: Query<&Transform, With<Camera3d>>,
    mut text: Query<&mut Text, With<CameraStatsText>>,
) {
    let Ok(transform) = cams.single() else {
        return;
    };
    let frame_ms = graph.samples.back().copied().unwrap_or(0.0);
    if let Ok(mut text) = text.single_mut() {
        **text = camera_pose_line(transform, frame.0, time.elapsed_secs(), frame_ms);
    }
}

// ---------------------------------------------------------------------------
// Object/lights HUD: bottom-right corner. The current shape's position/
// scale/rotation, plus each of the 3 fixed lights' position/rotation/
// params — requested explicitly so the exact scene state (not just camera
// pose) is always visible on screen, not just in the log.
// ---------------------------------------------------------------------------

/// This shape's own size parameter(s), compactly — this renderer's shapes
/// have no separate uniform `Transform.scale` (confirmed: `cpu_ref::
/// local_distance` takes shape params directly, never a scale factor), so
/// "scale" for debug purposes means each shape's own defining
/// dimension(s), the same fields `ShapeChoice::shape_and_height` set.
fn shape_scale_string(shape: &Shape) -> String {
    match *shape {
        Shape::Sphere { radius } => format!("radius {radius:.2}"),
        Shape::RoundedBox { half_extents, corner_radius } => {
            format!("half_extents ({:.2}, {:.2}, {:.2})  corner_r {corner_radius:.2}", half_extents.x, half_extents.y, half_extents.z)
        }
        Shape::RoundedCylinder { radius, half_height, edge_radius } => {
            format!("radius {radius:.2}  half_height {half_height:.2}  edge_r {edge_radius:.2}")
        }
        Shape::Capsule { a, b, radius } => {
            format!("a ({:.2},{:.2},{:.2})  b ({:.2},{:.2},{:.2})  radius {radius:.2}", a.x, a.y, a.z, b.x, b.y, b.z)
        }
        Shape::RoundedCone { r0, r1, .. } => format!("r0 {r0:.2}  r1 {r1:.2}"),
        Shape::Ellipsoid { radii } => format!("radii ({:.2}, {:.2}, {:.2})", radii.x, radii.y, radii.z),
        Shape::BoxFrame { half_extents, wall_thickness } => {
            format!("half_extents ({:.2}, {:.2}, {:.2})  wall {wall_thickness:.2}", half_extents.x, half_extents.y, half_extents.z)
        }
        Shape::HexPrism { radius, half_height } => format!("radius {radius:.2}  half_height {half_height:.2}"),
    }
}

/// One light's position/rotation/params line — direction-based lights
/// (sun/projector) report rotation the same Euler-degrees way the camera
/// HUD does; the lamp (point light, no meaningful orientation) reports
/// position and range/intensity only.
fn light_line(name: &str, transform: &GlobalTransform, extra: &str) -> String {
    let t = transform.translation();
    let (yaw, pitch, roll) = transform.rotation().to_euler(EulerRot::YXZ);
    format!(
        "{name}: pos ({:.2}, {:.2}, {:.2})  rot (pitch {:.1} yaw {:.1} roll {:.1})  {extra}",
        t.x, t.y, t.z, pitch.to_degrees(), yaw.to_degrees(), roll.to_degrees(),
    )
}

#[allow(clippy::type_complexity)]
fn object_and_lights_debug_text(
    objects: &Query<(&Shape, &GlobalTransform), With<Spinning>>,
    sun: &Query<(&DirectionalLight, &GlobalTransform), With<SunLight>>,
    lamp: &Query<(&PointLight, &GlobalTransform), With<LampLight>>,
    projector: &Query<(&SpotLight, &GlobalTransform), With<ProjectorLight>>,
    toggles: &LightToggles,
) -> String {
    let mut lines = Vec::new();

    if let Some((shape, transform)) = objects.iter().next() {
        let t = transform.translation();
        let (yaw, pitch, roll) = transform.rotation().to_euler(EulerRot::YXZ);
        lines.push(format!(
            "object: pos ({:.2}, {:.2}, {:.2})  rot (pitch {:.1} yaw {:.1} roll {:.1})",
            t.x, t.y, t.z, pitch.to_degrees(), yaw.to_degrees(), roll.to_degrees(),
        ));
        lines.push(format!("        scale: {}", shape_scale_string(shape)));
    }

    if let Some((light, transform)) = sun.iter().next() {
        let on = if toggles.sun { "on" } else { "off" };
        lines.push(light_line("sun", transform, &format!("illuminance {:.0}  [{on}]", light.illuminance)));
    }
    if let Some((light, transform)) = lamp.iter().next() {
        let on = if toggles.lamp { "on" } else { "off" };
        lines.push(light_line(
            "lamp",
            transform,
            &format!("intensity {:.0}  range {:.1}  [{on}]", light.intensity, light.range),
        ));
    }
    if let Some((light, transform)) = projector.iter().next() {
        let on = if toggles.projector { "on" } else { "off" };
        lines.push(light_line(
            "projector",
            transform,
            &format!(
                "intensity {:.0}  range {:.1}  inner {:.1} outer {:.1}  [{on}]",
                light.intensity,
                light.range,
                light.inner_angle.to_degrees(),
                light.outer_angle.to_degrees(),
            ),
        ));
    }

    lines.join("\n")
}

#[derive(Component)]
struct ObjectLightStatsText;

fn spawn_object_light_hud(mut commands: Commands) {
    commands.spawn((
        Text::new("object/lights —"),
        TextFont { font_size: FontSize::Px(13.0), ..default() },
        TextColor(Color::srgb(0.88, 0.92, 0.96)),
        Node {
            // Top-left, stacked below the FPS graph (300px wide, ends
            // ~y=104 — see FPS_BARS/spawn_fps_hud) rather than
            // bottom-right: this block's lines are long enough (5 lines,
            // one per light plus the object) that anchoring it opposite
            // the bottom-left camera-stats block let the two visually
            // collide in the middle of the screen at typical window
            // widths. Stacking both HUD blocks in the same corner as the
            // FPS stats avoids that entirely.
            position_type: PositionType::Absolute,
            left: Val::Px(10.0),
            top: Val::Px(115.0),
            ..default()
        },
        TextLayout::justify(Justify::Left),
        ObjectLightStatsText,
    ));
}

#[allow(clippy::type_complexity)]
fn update_object_light_hud(
    objects: Query<(&Shape, &GlobalTransform), With<Spinning>>,
    sun: Query<(&DirectionalLight, &GlobalTransform), With<SunLight>>,
    lamp: Query<(&PointLight, &GlobalTransform), With<LampLight>>,
    projector: Query<(&SpotLight, &GlobalTransform), With<ProjectorLight>>,
    toggles: Res<LightToggles>,
    mut text: Query<&mut Text, With<ObjectLightStatsText>>,
) {
    if let Ok(mut text) = text.single_mut() {
        **text = object_and_lights_debug_text(&objects, &sun, &lamp, &projector, &toggles);
    }
}

// ---------------------------------------------------------------------------
// Once-a-second debug log: same stats as the HUD, printed to the log so
// they're visible from a headless run too (CI, `--shot`, piped output).
// ---------------------------------------------------------------------------

#[derive(Resource)]
struct DebugLogTimer(Timer);

impl Default for DebugLogTimer {
    fn default() -> Self {
        Self(Timer::from_seconds(1.0, TimerMode::Repeating))
    }
}

#[allow(clippy::type_complexity, clippy::too_many_arguments)]
fn log_debug_stats(
    time: Res<Time>,
    frame: Res<FrameCount>,
    mut timer: ResMut<DebugLogTimer>,
    diagnostics: Res<DiagnosticsStore>,
    graph: Res<FpsGraph>,
    cams: Query<&Transform, With<Camera3d>>,
    objects: Query<(&Shape, &GlobalTransform), With<Spinning>>,
    sun: Query<(&DirectionalLight, &GlobalTransform), With<SunLight>>,
    lamp: Query<(&PointLight, &GlobalTransform), With<LampLight>>,
    projector: Query<(&SpotLight, &GlobalTransform), With<ProjectorLight>>,
    toggles: Res<LightToggles>,
) {
    if timer.0.tick(time.delta()).just_finished() {
        let stats = FpsStats::compute(&diagnostics, &graph);
        let percentiles = FpsPercentiles::compute(&graph);
        let camera = cams.single().ok().map(|transform| {
            let (yaw, pitch, roll) = transform.rotation.to_euler(EulerRot::YXZ);
            format!(
                "pos ({:.2}, {:.2}, {:.2})   rot (pitch {:.1} yaw {:.1} roll {:.1})",
                transform.translation.x,
                transform.translation.y,
                transform.translation.z,
                pitch.to_degrees(),
                yaw.to_degrees(),
                roll.to_degrees(),
            )
        });
        // Same object/lights info the bottom-right HUD shows, flattened
        // to one line (newlines replaced with "  |  ") so a headless run
        // still gets it in the log, not just on screen.
        let object_lights = object_and_lights_debug_text(&objects, &sun, &lamp, &projector, &toggles)
            .replace('\n', "  |  ");
        info!(
            "gallery: frame {}   t {:.2}s   {stats}   {percentiles}{}   |  {object_lights}",
            frame.0,
            time.elapsed_secs(),
            camera.map(|c| format!("   {c}")).unwrap_or_default(),
        );
    }
}

#[cfg(test)]
mod tests {
    use migera::hybrid::scene::world_aabb;

    use super::*;

    /// Every `ShapeChoice`'s `shape_and_height()` claims a spawn height
    /// tall enough that no rotation ever dips the shape below the
    /// ground's top face at world `y = 0` (see `spawn_scene`'s doc
    /// comment and the `Box` case's own comment for the original
    /// reasoning this generalizes to every shape). Proven here by
    /// actually computing `scene::world_aabb` — the same function the
    /// real renderer's AABB/BVH path calls — at many sampled rotations
    /// per shape, not just trusting each case's hand-derived bounding
    /// radius. 200 rotations per shape (uniformly spread across pitch/
    /// yaw/roll in [0, TAU)) is enough to catch a wrong-axis or
    /// off-by-a-constant bug with very high probability without being
    /// an exhaustive proof — a genuine formula error would show up as a
    /// clearance violation at a large fraction of sampled rotations, not
    /// require pinpoint luck to hit.
    #[test]
    fn no_shape_choice_ever_dips_below_ground_at_any_rotation() {
        const SAMPLES: usize = 200;
        let all_shapes = [
            ShapeChoice::Box,
            ShapeChoice::Sphere,
            ShapeChoice::Cylinder,
            ShapeChoice::Capsule,
            ShapeChoice::Ellipsoid,
            ShapeChoice::BoxFrame,
            ShapeChoice::HexPrism,
        ];
        for choice in all_shapes {
            let (shape, object_height) = choice.shape_and_height();
            for i in 0..SAMPLES {
                let f = i as f32 / SAMPLES as f32;
                let rotation = Quat::from_euler(EulerRot::XYZ, f * TAU, (f * 2.3) % 1.0 * TAU, (f * 3.7) % 1.0 * TAU);
                let transform =
                    GlobalTransform::from(Transform { translation: Vec3::new(0.0, object_height, 0.0), rotation, ..default() });
                let aabb = world_aabb(&shape, &transform);
                assert!(
                    aabb.min.y >= -1e-4,
                    "{choice:?} at rotation sample {i} (euler-derived quat {rotation:?}) dips to y={} \
                     below the ground's top face (y=0) — object_height={object_height} is too small \
                     for this shape's true bounding radius",
                    aabb.min.y
                );
            }
        }
    }

    /// `shape_and_height`'s clearance formula measures distance from the
    /// coordinate ORIGIN to `local_aabb`'s farthest corner — deliberately
    /// not from the AABB's own center, since `world_aabb`'s corner
    /// rotation pivots around the origin (see `shape_and_height`'s doc
    /// comment). Every shape this module actually spawns happens to have
    /// an origin-centered `local_aabb` today (including `ShapeChoice::
    /// Capsule`'s specific symmetric endpoint choice), so that
    /// distinction is untested by
    /// `no_shape_choice_ever_dips_below_ground_at_any_rotation` alone —
    /// this test exercises the same clearance math directly against an
    /// intentionally ASYMMETRIC capsule (endpoints not mirrored around
    /// the origin) to prove the origin-distance approach is actually
    /// correct in general, not just coincidentally correct for today's
    /// symmetric inputs.
    #[test]
    fn clearance_formula_holds_for_an_asymmetric_off_origin_capsule() {
        let shape = Shape::Capsule { a: Vec3::new(0.0, -0.2, 0.0), b: Vec3::new(0.0, 1.4, 0.0), radius: 0.3 };
        let local = migera::hybrid::scene::local_aabb(&shape);
        let farthest_corner = local.min.abs().max(local.max.abs());
        let object_height = farthest_corner.length() + 0.05;

        const SAMPLES: usize = 200;
        for i in 0..SAMPLES {
            let f = i as f32 / SAMPLES as f32;
            let rotation = Quat::from_euler(EulerRot::XYZ, f * TAU, (f * 2.3) % 1.0 * TAU, (f * 3.7) % 1.0 * TAU);
            let transform =
                GlobalTransform::from(Transform { translation: Vec3::new(0.0, object_height, 0.0), rotation, ..default() });
            let aabb = world_aabb(&shape, &transform);
            assert!(
                aabb.min.y >= -1e-4,
                "asymmetric capsule at rotation sample {i} dips to y={} below ground \
                 (object_height={object_height}) — the origin-distance clearance formula \
                 does not generalize to off-origin-centered local_aabb shapes",
                aabb.min.y
            );
        }
    }
}
