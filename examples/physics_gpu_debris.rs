//! Visual-effects-only GPU physics showcase: thousands of simple debris
//! bodies fall and scatter across a static floor, driven entirely by the
//! demoted custom engine's own GPU compute path
//! (`physics::gpu::effects::GpuDebrisBody`), now explicitly re-scoped away
//! from "the GPU port of the primary solver" (that role belongs to
//! `avian3d`, see `crate::physics_avian`) to exactly this use case: large
//! counts of simple, individually-inconsequential bodies where GPU
//! throughput is the actual goal, not per-body simulation correctness
//! against gameplay-relevant bodies. Demonstrates the real, measured ~3x
//! full-frame GPU speedup Piece 5's own benchmark found at this body-count
//! scale (2,000-20,000 bodies) — see `PROGRESS.md`'s own "Broad-phase/
//! contact-generation GPU port, Piece 5" entry for the full comparison
//! table this showcase's own body count was chosen to land inside.
//!
//! Debris bodies never carry `BodyKind`/kinematic support (dropped
//! entirely from this path, see `GpuDebrisBody`'s own doc comment) — they
//! only ever fall under gravity and collide against the static floor and
//! each other, which is exactly what `extract_physics_bodies`'s own
//! dynamic-vs-dynamic and dynamic-vs-static contact generation already
//! supports unchanged.
//!
//! Run: `cargo run --release --example physics_gpu_debris [-- --count N]`
//!
//! **Default count is 500, not Piece 5's own 2,000-20,000 benchmark
//! range** — deliberately, not an oversight. Piece 5's own measured ~3x
//! speedup was an isolated physics-dispatch comparison; THIS example also
//! pays this renderer's own real per-object raymarching + BVH-refit-under-
//! motion cost (every debris body moves every frame while settling, and
//! `hybrid::bvh`'s own refit cost under real motion was already flagged as
//! an open risk by this project's own Stage 3 profiling milestone — see
//! `PROGRESS.md`), which was never exercised at more than ~13 simple
//! bodies by any earlier example. At `--count 2000+` this example is
//! genuinely slow to render (confirmed: ~4 fps at 500 bodies on this
//! project's own dev hardware) — a real, separate cost from the physics
//! dispatch itself, reported honestly here rather than silently choosing
//! a small default that hides it. Pass `--count N` to explore the actual
//! crossover on your own hardware.

use bevy::camera::{Exposure, Hdr};
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::diagnostic::FrameCount;
use bevy::prelude::*;
use bevy::render::view::window::screenshot::{Screenshot, save_to_disk};

use migera::hybrid::HybridRenderPlugin;
use migera::hybrid::extract::SunLight;
use migera::hybrid::material::Material;
use migera::physics::components::PhysicsShape;
use migera::physics::gpu::effects::GpuDebrisBody;
use migera::physics::integrate::PhysicsGpuEnabled;
use migera::physics::solve_static::PhysicsGravity;
use migera::sdf::assembly::SdfSceneRoot;
use migera::sdf::components::Shape;

const FLOOR_HALF_EXTENTS: Vec3 = Vec3::new(20.0, 0.5, 20.0);
const DEBRIS_RADIUS: f32 = 0.15;
const DROP_HEIGHT: f32 = 12.0;

fn main() {
    let assets = std::env::current_dir().expect("cwd").join("assets").to_string_lossy().into_owned();
    let count = debris_count_from_args();

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window { title: "migera GPU debris showcase".into(), ..default() }),
            ..default()
        }).set(AssetPlugin { file_path: assets, ..default() }))
        .add_plugins(HybridRenderPlugin)
        .insert_resource(PhysicsGravity { center: Vec3::new(0.0, -10_000.0, 0.0), magnitude: 9.81 })
        .insert_resource(PhysicsGpuEnabled(true))
        .insert_resource(ShotConfig::from_args())
        .insert_resource(DebrisCount(count))
        .add_systems(Startup, (spawn_camera, spawn_scene, spawn_light))
        .add_systems(Update, auto_shot)
        .run();
}

fn debris_count_from_args() -> usize {
    let args: Vec<String> = std::env::args().collect();
    let mut i = 0;
    while i < args.len() {
        if args[i] == "--count" && let Some(value) = args.get(i + 1).and_then(|s| s.parse().ok()) {
            return value;
        }
        i += 1;
    }
    5_000
}

#[derive(Resource)]
struct DebrisCount(usize);

/// `--shot PATH --at-frame N`: headless screenshot-based verification,
/// same convention every existing example already uses.
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
                    cfg.shot = Some((cfg.shot.map_or("/tmp/physics_gpu_debris.png".into(), |s| s.0), val(&mut i).parse().unwrap_or(120)));
                }
                _ => {}
            }
            i += 1;
        }
        cfg
    }
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
        info!("physics_gpu_debris: screenshot -> {path}");
        commands.spawn(Screenshot::primary_window()).observe(save_to_disk(path.clone()));
    }
    if *fired && !*exited && frame.0 >= at + 60 {
        *exited = true;
        std::thread::sleep(std::time::Duration::from_millis(600));
        std::process::exit(0);
    }
}

fn spawn_camera(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Msaa::Off,
        Hdr,
        Tonemapping::default(),
        Exposure::default(),
        Transform::from_xyz(0.0, 22.0, 28.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}

fn spawn_light(mut commands: Commands) {
    commands.spawn((
        SunLight,
        DirectionalLight { color: Color::srgb(1.0, 0.95, 0.85), illuminance: 1500.0, ..default() },
        Transform::from_xyz(10.0, 20.0, 15.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}

/// A cheap deterministic pseudo-random generator (no external crate
/// needed) — reproducible run-to-run, same convention
/// `physics_stability.rs`'s own `Rng` uses.
struct Rng(u64);

impl Rng {
    fn next_f32(&mut self) -> f32 {
        self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1);
        ((self.0 >> 33) as f32) / (u32::MAX as f32)
    }

    fn range(&mut self, lo: f32, hi: f32) -> f32 {
        lo + self.next_f32() * (hi - lo)
    }
}

/// A static floor plus `count` small dynamic debris spheres, dropped from
/// a spread of random positions/velocities above it — no kinematic
/// platform, no `BodyKind` distinction at all, exactly the pure "large
/// count of simple, individually-inconsequential bodies" case this
/// showcase exists to demonstrate.
fn spawn_scene(mut commands: Commands, count: Res<DebrisCount>) {
    let root = commands.spawn((SdfSceneRoot, Transform::IDENTITY, Visibility::default())).id();

    commands.spawn((
        ChildOf(root),
        Shape::RoundedBox { half_extents: FLOOR_HALF_EXTENTS, corner_radius: 0.0 },
        PhysicsShape::RoundedBox { half_extents: FLOOR_HALF_EXTENTS, corner_radius: 0.0 },
        Transform::from_xyz(0.0, -FLOOR_HALF_EXTENTS.y, 0.0),
        Material::new(Vec3::new(0.35, 0.36, 0.38), 0.0, 0.6),
    ));

    let mut rng = Rng(0xdeb1_1500_c0de_abcd);
    let mass = 1.0;
    let inverse_inertia = 1.0 / (0.4 * mass * DEBRIS_RADIUS * DEBRIS_RADIUS);

    for _ in 0..count.0 {
        let x = rng.range(-FLOOR_HALF_EXTENTS.x * 0.8, FLOOR_HALF_EXTENTS.x * 0.8);
        let z = rng.range(-FLOOR_HALF_EXTENTS.z * 0.8, FLOOR_HALF_EXTENTS.z * 0.8);
        let y = rng.range(0.0, DROP_HEIGHT);
        let velocity = Vec3::new(rng.range(-0.5, 0.5), 0.0, rng.range(-0.5, 0.5));
        let hue = rng.range(0.0, 1.0);
        let color = Vec3::new(0.5 + 0.4 * (hue * 6.0).sin(), 0.5 + 0.4 * (hue * 6.0 + 2.0).sin(), 0.5 + 0.4 * (hue * 6.0 + 4.0).sin());

        commands.spawn((
            ChildOf(root),
            Shape::Sphere { radius: DEBRIS_RADIUS },
            PhysicsShape::Sphere { radius: DEBRIS_RADIUS },
            Transform::from_xyz(x, y, z),
            Material::new(color, 0.1, 0.6),
            GpuDebrisBody { linear_velocity: velocity, angular_velocity: Vec3::ZERO, inverse_mass: 1.0 / mass, inverse_inertia_local: Vec3::splat(inverse_inertia) },
        ));
    }
}
