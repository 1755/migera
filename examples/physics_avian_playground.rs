//! avian3d parity check for `physics_playground.rs`'s own three milestones
//! (rounded-floor-corner sphere, crate pyramid, kinematic elevator) — proves
//! avian3d can reproduce every one of this project's own physics milestones
//! using its own `RigidBody`/`Collider` directly, with zero custom solver
//! code. See `src/physics_avian/mod.rs`'s own module doc comment for why
//! avian3d is now this project's primary physics engine.
//!
//! Every spawned entity carries BOTH the SDF renderer's own `Shape`
//! component (rendered exactly like every other example — zero renderer-
//! side changes needed) AND avian3d's own `RigidBody`/`Collider`
//! (simulated by avian3d, driving the SAME entity's `Transform`) — proving
//! the two systems compose with no friction.
//!
//! Run: `cargo run --release --example physics_avian_playground`

use avian3d::prelude::*;
use bevy::camera::{Exposure, Hdr};
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::diagnostic::FrameCount;
use bevy::prelude::*;
use bevy::render::view::window::screenshot::{Screenshot, save_to_disk};

use migera::hybrid::HybridRenderPlugin;
use migera::hybrid::extract::SunLight;
use migera::hybrid::material::Material;
use migera::physics_avian::shape_to_collider;
use migera::sdf::assembly::SdfSceneRoot;
use migera::sdf::components::Shape;

fn main() {
    let assets = std::env::current_dir().expect("cwd").join("assets").to_string_lossy().into_owned();

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window { title: "migera avian3d physics playground".into(), ..default() }),
            ..default()
        }).set(AssetPlugin { file_path: assets, ..default() }))
        .add_plugins(HybridRenderPlugin)
        .add_plugins(PhysicsPlugins::default())
        .insert_resource(ShotConfig::from_args())
        .add_systems(Startup, (spawn_camera, spawn_scene, spawn_light))
        .add_systems(Update, (auto_shot, drive_elevator))
        .run();
}

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
                    cfg.shot = Some((cfg.shot.map_or("/tmp/physics_avian_playground.png".into(), |s| s.0), val(&mut i).parse().unwrap_or(120)));
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
        info!("physics_avian_playground: screenshot -> {path}");
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

/// Same milestone `physics_playground.rs`'s own `spawn_scene` describes: a
/// static rounded-corner floor plus a sphere dropped just past the floor's
/// rounded top corner, so it visibly grazes the curved region before
/// settling — here resolved entirely by avian3d's own `round_cuboid`/
/// `sphere` narrow-phase, not this project's own SDF distance-field query.
fn spawn_scene(mut commands: Commands) {
    let root = commands.spawn((SdfSceneRoot, Transform::IDENTITY, Visibility::default())).id();

    let floor_shape = Shape::RoundedBox { half_extents: FLOOR_HALF_EXTENTS, corner_radius: FLOOR_CORNER_RADIUS };
    commands.spawn((
        ChildOf(root),
        floor_shape,
        shape_to_collider(&floor_shape).expect("RoundedBox always maps to a collider"),
        RigidBody::Static,
        Transform::from_xyz(0.0, -FLOOR_HALF_EXTENTS.y, 0.0),
        Material::new(Vec3::new(0.45, 0.46, 0.48), 0.0, 0.6),
    ));

    let corner_edge = FLOOR_HALF_EXTENTS.x - FLOOR_CORNER_RADIUS;
    let start = Vec3::new(corner_edge - 0.3, 4.0, corner_edge - 0.3);
    let sphere_shape = Shape::Sphere { radius: SPHERE_RADIUS };
    commands.spawn((
        ChildOf(root),
        sphere_shape,
        shape_to_collider(&sphere_shape).expect("Sphere always maps to a collider"),
        RigidBody::Dynamic,
        Transform::from_translation(start),
        Material::new(Vec3::new(0.8, 0.25, 0.2), 0.0, 0.4),
    ));

    spawn_crate_pyramid(&mut commands, root);
    spawn_elevator(&mut commands, root);
}

/// Same milestone `physics_playground.rs`'s own `spawn_crate_pyramid`
/// describes: a 3-2-1 pyramid of crates, spawned already stacked, that must
/// settle and stay stacked — here resolved by avian3d's own production
/// narrow-phase/solver instead of this project's own sample-point contact
/// generation + Jacobi substep solver.
fn spawn_crate_pyramid(commands: &mut Commands, root: Entity) {
    let full = CRATE_HALF_EXTENTS * 2.0;
    let gap = 0.02;
    let crate_shape = Shape::RoundedBox { half_extents: CRATE_HALF_EXTENTS, corner_radius: CRATE_CORNER_RADIUS };

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
                crate_shape,
                shape_to_collider(&crate_shape).expect("RoundedBox always maps to a collider"),
                RigidBody::Dynamic,
                Transform::from_xyz(x, y, base_center.z),
                Material::new(Vec3::new(0.7, 0.55, 0.35), 0.0, 0.5),
            ));
        }
        y += full.y + gap;
    }
}

const ELEVATOR_HALF_EXTENTS: Vec3 = Vec3::new(1.2, 0.25, 1.2);
const ELEVATOR_RISE_SPEED: f32 = 0.3;
const ELEVATOR_LOW_Y: f32 = -FLOOR_HALF_EXTENTS.y + ELEVATOR_HALF_EXTENTS.y;
const ELEVATOR_HIGH_Y: f32 = ELEVATOR_LOW_Y + 3.0;

/// Marks the elevator entity so `drive_elevator` can find it.
#[derive(Component)]
struct Elevator {
    rising: bool,
}

/// Same milestone `physics_playground.rs`'s own `spawn_elevator` describes:
/// a `RigidBody::Kinematic` platform carrying a resting dynamic crate up and
/// down. Unlike the demoted custom engine's own convention (drive a
/// kinematic body by writing `Transform` directly, with `RigidBody.linear_velocity`
/// kept in sync purely for the solver's own contact-response math), avian3d's
/// own documented convention is the reverse: set `LinearVelocity` and let
/// avian3d itself integrate `Transform` — writing `Transform` directly is
/// "similar to teleporting the body, which can result in unexpected
/// behavior since the body can move inside walls" (avian3d's own docs).
/// `drive_elevator` below only ever writes `LinearVelocity`.
fn spawn_elevator(commands: &mut Commands, root: Entity) {
    let base = Vec3::new(3.0, 0.0, 3.0);
    let elevator_shape = Shape::RoundedBox { half_extents: ELEVATOR_HALF_EXTENTS, corner_radius: 0.05 };

    commands.spawn((
        ChildOf(root),
        elevator_shape,
        shape_to_collider(&elevator_shape).expect("RoundedBox always maps to a collider"),
        RigidBody::Kinematic,
        LinearVelocity(Vec3::new(0.0, ELEVATOR_RISE_SPEED, 0.0)),
        Transform::from_xyz(base.x, ELEVATOR_LOW_Y, base.z),
        Material::new(Vec3::new(0.3, 0.5, 0.75), 0.0, 0.4),
        Elevator { rising: true },
    ));

    let crate_shape = Shape::RoundedBox { half_extents: CRATE_HALF_EXTENTS, corner_radius: CRATE_CORNER_RADIUS };
    commands.spawn((
        ChildOf(root),
        crate_shape,
        shape_to_collider(&crate_shape).expect("RoundedBox always maps to a collider"),
        RigidBody::Dynamic,
        Transform::from_xyz(base.x, ELEVATOR_LOW_Y + ELEVATOR_HALF_EXTENTS.y + CRATE_HALF_EXTENTS.y + 0.02, base.z),
        Material::new(Vec3::new(0.85, 0.7, 0.3), 0.0, 0.5),
    ));
}

/// Flips `LinearVelocity`'s own sign at the high/low bounds — the only
/// thing this system ever writes, per avian3d's own kinematic-body
/// convention (see `spawn_elevator`'s own doc comment). `Transform` itself
/// is left entirely to avian3d's own integration.
fn drive_elevator(mut elevators: Query<(&Transform, &mut LinearVelocity, &mut Elevator)>) {
    for (transform, mut velocity, mut elevator) in &mut elevators {
        if elevator.rising && transform.translation.y >= ELEVATOR_HIGH_Y {
            elevator.rising = false;
        } else if !elevator.rising && transform.translation.y <= ELEVATOR_LOW_Y {
            elevator.rising = true;
        }
        let speed = if elevator.rising { ELEVATOR_RISE_SPEED } else { -ELEVATOR_RISE_SPEED };
        velocity.0 = Vec3::new(0.0, speed, 0.0);
    }
}
