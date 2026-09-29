//! Physics stage-by-stage verification playground — deliberately separate
//! from `examples/gallery.rs` so physics debug UI/scenes don't entangle
//! with the renderer-feature gallery (see `src/physics/mod.rs`'s doc
//! comment for the overall plan this supports).
//!
//! Stage 2 milestone (still exercised by the same `solve_world` solver
//! stage 3 introduced): a `RigidBody` sphere falls under gravity and comes
//! to rest on a static `RoundedBox` floor, resolved via
//! `physics::collision_static`/`solve_static` against the floor's own SDF
//! (distance + gradient) — including correctly against the floor's rounded
//! top corners, not just its flat top face, which is exactly the case
//! naive box-collision code gets wrong and SDF collision gets right by
//! construction.
//!
//! Stage 3 milestone: a small pyramid of `RoundedBox` crates, dropped
//! already stacked, settles under `physics::solve_world`'s Claybook-style
//! sample-point contact generation + Jacobi-averaged XPBD substep solver
//! and stays stacked — the "one distance query isn't enough for stable
//! multi-point contact" problem `physics::contacts`' own module doc
//! comment describes, exercised end-to-end through the renderer.
//!
//! Stage 1's constant-velocity-drift milestone is preserved as
//! `physics::integrate::physics_integrate_placeholder`'s own regression
//! tests, but no longer wired into this example's `App` (see
//! `physics::integrate`'s doc comment for why only one authoritative-
//! transform system runs at a time).
//!
//! Run: `cargo run --release --example physics_playground`
//!
//! **No longer has a `--gpu` flag.** This example uses the demoted CPU
//! solver's own `RigidBody`/`BodyKind`/`Inertia` components, and the GPU
//! physics path (`physics::gpu`) has been re-scoped to visual-effects-only
//! debris bodies (`physics::gpu::effects::GpuDebrisBody`), which this
//! scene's entities no longer satisfy — passing `--gpu` here would
//! silently do nothing (no bodies extracted, nothing simulated) rather
//! than erroring, so the flag was removed outright instead of leaving it
//! to silently mislead. See `crate::physics_avian` for this project's
//! current primary physics engine (`avian3d`,
//! `examples/physics_avian_playground.rs` is this example's own avian3d
//! port) and `physics::gpu::effects`'s own doc comment for the debris-only
//! GPU path's new scope.

use bevy::camera::{Exposure, Hdr};
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::diagnostic::FrameCount;
use bevy::prelude::*;
use bevy::render::view::window::screenshot::{Screenshot, save_to_disk};

use migera::hybrid::HybridRenderPlugin;
use migera::hybrid::extract::SunLight;
use migera::hybrid::material::Material;
use migera::physics::components::{BodyKind, Inertia, PhysicsShape, RigidBody};
use migera::physics::inertia::{box_inertia, sphere_inertia};
use migera::physics::integrate::PhysicsPlugin;
use migera::sdf::assembly::SdfSceneRoot;
use migera::sdf::components::Shape;

fn main() {
    let assets = std::env::current_dir().expect("cwd").join("assets").to_string_lossy().into_owned();
    if std::env::args().any(|a| a == "--gpu") {
        eprintln!(
            "physics_playground: --gpu is no longer supported here -- the GPU physics path is now visual-effects-only \
             (see physics::gpu::effects) and this example's own RigidBody/BodyKind entities aren't extracted by it. \
             Run `cargo run --release --example physics_avian_playground` for this project's current primary physics \
             engine (avian3d)."
        );
        std::process::exit(1);
    }

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window { title: "migera physics playground".into(), ..default() }),
            ..default()
        }).set(AssetPlugin { file_path: assets, ..default() }))
        .add_plugins(HybridRenderPlugin)
        .add_plugins(PhysicsPlugin)
        .insert_resource(ShotConfig::from_args())
        .add_systems(Startup, (spawn_camera, spawn_scene, spawn_light))
        .add_systems(Update, (auto_shot, drive_elevator))
        .run();
}

/// `--shot PATH --at-frame N`: headless screenshot-based verification,
/// ported verbatim from `examples/gallery.rs`'s own `ShotConfig`/`auto_shot`
/// convention — the same mechanism this project already relies on to
/// confirm what actually renders without a human at the keyboard.
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
                    cfg.shot = Some((cfg.shot.map_or("/tmp/physics_playground.png".into(), |s| s.0), val(&mut i).parse().unwrap_or(120)));
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
        info!("physics_playground: screenshot -> {path}");
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
        Transform::from_xyz(9.0, 9.0, 12.0).looking_at(Vec3::new(1.0, -0.3, 1.0), Vec3::Y),
    ));
}

fn spawn_light(mut commands: Commands) {
    commands.spawn((
        SunLight,
        DirectionalLight { color: Color::srgb(1.0, 0.95, 0.85), illuminance: 1500.0, ..default() },
        Transform::from_xyz(0.0, 10.0, 0.0).looking_at(Vec3::new(-3.0, 0.0, -2.0), Vec3::Y),
    ));
}

const FLOOR_HALF_EXTENTS: Vec3 = Vec3::new(6.0, 0.5, 6.0);
const FLOOR_CORNER_RADIUS: f32 = 0.4;
const SPHERE_RADIUS: f32 = 0.75;
const CRATE_HALF_EXTENTS: Vec3 = Vec3::new(0.6, 0.6, 0.6);
const CRATE_CORNER_RADIUS: f32 = 0.05;

/// A static `RoundedBox` floor (no `RigidBody` — `solve_static_collisions`
/// treats any `PhysicsShape` entity without one as a static collider) plus
/// a `RigidBody` sphere dropped just past the floor's rounded top corner,
/// so it visibly grazes and rolls across the curved region as it falls —
/// exercising exactly the SDF-collision case a naive box-vs-sphere check
/// gets wrong (see `collision_static`'s and `solve_static`'s own dedicated
/// corner-resolution tests) — before settling in its final resting spot.
///
/// Deliberately NOT dropped to rest balanced ON the rounded corner itself:
/// a convex corner has no local minimum for a frictionless point contact
/// (there's nothing to arrest sideways sliding, same as a marble on a
/// dome), so a sphere that lands there will always roll off given enough
/// time — correct physics for a single-contact, zero-friction solver, not
/// a bug, and not the "falls and stays resting" milestone this example
/// wants to demonstrate at a glance. Starting it above the flat face, a
/// little inward from the corner, means gravity carries it down and
/// slightly across the curved region before it settles on the flat top —
/// the corner geometry still gets genuinely exercised along the way.
fn spawn_scene(mut commands: Commands) {
    let root = commands.spawn((SdfSceneRoot, Transform::IDENTITY, Visibility::default())).id();

    commands.spawn((
        ChildOf(root),
        Shape::RoundedBox { half_extents: FLOOR_HALF_EXTENTS, corner_radius: FLOOR_CORNER_RADIUS },
        PhysicsShape::RoundedBox { half_extents: FLOOR_HALF_EXTENTS, corner_radius: FLOOR_CORNER_RADIUS },
        Transform::from_xyz(0.0, -FLOOR_HALF_EXTENTS.y, 0.0),
        Material::new(Vec3::new(0.45, 0.46, 0.48), 0.0, 0.6),
    ));

    let corner_edge = FLOOR_HALF_EXTENTS.x - FLOOR_CORNER_RADIUS;
    let start = Vec3::new(corner_edge - 0.3, 4.0, corner_edge - 0.3);
    let mass = 1.0;
    commands.spawn((
        ChildOf(root),
        Shape::Sphere { radius: SPHERE_RADIUS },
        PhysicsShape::Sphere { radius: SPHERE_RADIUS },
        Transform::from_translation(start),
        Material::new(Vec3::new(0.8, 0.25, 0.2), 0.0, 0.4),
        RigidBody::default(),
        Inertia { inverse_mass: 1.0 / mass, inverse_tensor_diag: 1.0 / sphere_inertia(mass, SPHERE_RADIUS) },
    ));

    spawn_crate_pyramid(&mut commands, root);
    spawn_elevator(&mut commands, root);
}

/// A 3-2-1 pyramid of `RoundedBox` crates, spawned already stacked (already
/// axis-aligned and lightly separated, not dropped from a height) so the
/// milestone this exercises is specifically "does the multi-point contact
/// solver hold a resting stack together," not "can bodies without any
/// angular contact response tumble into alignment on their own" —
/// `physics::solve_rigid` only applies positional (not torque) corrections
/// in this stage, so an already-toppling or badly-misaligned drop isn't a
/// scenario this solver is expected to recover from yet.
fn spawn_crate_pyramid(commands: &mut Commands, root: Entity) {
    let full = CRATE_HALF_EXTENTS * 2.0;
    let gap = 0.02;
    let mass = 1.0;
    let inverse_tensor_diag = 1.0 / box_inertia(mass, CRATE_HALF_EXTENTS);

    // Pyramid center placed away from the sphere/corner milestone so the
    // two don't interact, spawned near the floor's flat center rather than
    // its corner (the pyramid's own multi-point-manifold behavior is the
    // thing under test here, not corner resolution again).
    let base_center = Vec3::new(-2.5, 0.0, -2.5);
    let layer_counts = [3, 2, 1];
    let mut y = -FLOOR_HALF_EXTENTS.y + CRATE_HALF_EXTENTS.y;

    for &count in &layer_counts {
        let row_width = count as f32 * (full.x + gap) - gap;
        let start_x = base_center.x - row_width / 2.0 + CRATE_HALF_EXTENTS.x;
        for i in 0..count {
            let x = start_x + i as f32 * (full.x + gap);
            commands.spawn((
                ChildOf(root),
                Shape::RoundedBox { half_extents: CRATE_HALF_EXTENTS, corner_radius: CRATE_CORNER_RADIUS },
                PhysicsShape::RoundedBox { half_extents: CRATE_HALF_EXTENTS, corner_radius: CRATE_CORNER_RADIUS },
                Transform::from_xyz(x, y, base_center.z),
                Material::new(Vec3::new(0.7, 0.55, 0.35), 0.0, 0.5),
                RigidBody::default(),
                Inertia { inverse_mass: 1.0 / mass, inverse_tensor_diag },
            ));
        }
        y += full.y + gap;
    }
}

const ELEVATOR_HALF_EXTENTS: Vec3 = Vec3::new(1.2, 0.25, 1.2);
const ELEVATOR_RISE_SPEED: f32 = 0.3;
const ELEVATOR_LOW_Y: f32 = -FLOOR_HALF_EXTENTS.y + ELEVATOR_HALF_EXTENTS.y;
const ELEVATOR_HIGH_Y: f32 = ELEVATOR_LOW_Y + 3.0;

/// Marks the elevator entity so `drive_elevator` can find it without a
/// generic `BodyKind::Kinematic` query (which would also match any future
/// kinematic entity added to this example).
#[derive(Component)]
struct Elevator {
    rising: bool,
}

/// Stage 3.5's kinematic-body milestone: a `BodyKind::Kinematic` platform
/// (`RigidBody` + `Inertia::STATIC`, `Transform` driven manually by
/// `drive_elevator` below) carrying a resting dynamic crate up and down —
/// visual proof that a kinematic body imparts its externally-set velocity
/// to a dynamic body resting on it, while never being movable itself
/// (`solve_world` never writes this entity's `Transform`/`RigidBody`, see
/// `physics::components::RigidBody`'s own doc comment on the kinematic
/// velocity contract). Placed away from both the sphere/corner and the
/// crate-pyramid milestones so none of the three interact.
fn spawn_elevator(commands: &mut Commands, root: Entity) {
    let base = Vec3::new(3.0, 0.0, 3.0);

    commands.spawn((
        ChildOf(root),
        Shape::RoundedBox { half_extents: ELEVATOR_HALF_EXTENTS, corner_radius: 0.05 },
        PhysicsShape::RoundedBox { half_extents: ELEVATOR_HALF_EXTENTS, corner_radius: 0.05 },
        Transform::from_xyz(base.x, ELEVATOR_LOW_Y, base.z),
        Material::new(Vec3::new(0.3, 0.5, 0.75), 0.0, 0.4),
        RigidBody { linear_velocity: Vec3::new(0.0, ELEVATOR_RISE_SPEED, 0.0), angular_velocity: Vec3::ZERO },
        Inertia::STATIC,
        BodyKind::Kinematic,
        Elevator { rising: true },
    ));

    let mass = 1.0;
    let inverse_tensor_diag = 1.0 / box_inertia(mass, CRATE_HALF_EXTENTS);
    commands.spawn((
        ChildOf(root),
        Shape::RoundedBox { half_extents: CRATE_HALF_EXTENTS, corner_radius: CRATE_CORNER_RADIUS },
        PhysicsShape::RoundedBox { half_extents: CRATE_HALF_EXTENTS, corner_radius: CRATE_CORNER_RADIUS },
        Transform::from_xyz(base.x, ELEVATOR_LOW_Y + ELEVATOR_HALF_EXTENTS.y + CRATE_HALF_EXTENTS.y + 0.02, base.z),
        Material::new(Vec3::new(0.85, 0.7, 0.3), 0.0, 0.5),
        RigidBody::default(),
        Inertia { inverse_mass: 1.0 / mass, inverse_tensor_diag },
    ));
}

/// Manually advances the elevator's `Transform` by `velocity * dt` every
/// frame (standing in for a real animation/script system) and flips
/// direction at the high/low bounds, keeping `RigidBody.linear_velocity`
/// consistent with that motion every frame — exactly the contract
/// `physics::components::RigidBody`'s doc comment requires of whatever
/// drives a kinematic body's `Transform`.
fn drive_elevator(time: Res<Time>, mut elevators: Query<(&mut Transform, &mut RigidBody, &mut Elevator)>) {
    let dt = time.delta_secs();
    for (mut transform, mut rigid_body, mut elevator) in &mut elevators {
        if elevator.rising && transform.translation.y >= ELEVATOR_HIGH_Y {
            elevator.rising = false;
        } else if !elevator.rising && transform.translation.y <= ELEVATOR_LOW_Y {
            elevator.rising = true;
        }
        let speed = if elevator.rising { ELEVATOR_RISE_SPEED } else { -ELEVATOR_RISE_SPEED };
        rigid_body.linear_velocity = Vec3::new(0.0, speed, 0.0);
        transform.translation.y += speed * dt;
    }
}
