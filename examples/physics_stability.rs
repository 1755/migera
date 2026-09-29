//! Stability soak: many different `PhysicsShape` kinds, each with a
//! nontrivial initial rotation and a small non-radial initial velocity,
//! dropped from different angles around a static planetary sphere under
//! `PhysicsGravity`'s point-source model (`physics::solve_static::PhysicsGravity`)
//! — every body should fall toward the planet's own center (not a single
//! fixed "down" direction) and come to rest on its curved surface,
//! regardless of which side it started on.
//!
//! This exists to shake out solver-stability issues across shape variety
//! and non-trivial initial conditions that the narrower
//! `physics_playground.rs` milestones (aligned boxes, axis-aligned drops)
//! wouldn't exercise — run for an extended number of frames and confirm
//! nothing explodes, tunnels through the planet, or drifts indefinitely.
//!
//! Run: `cargo run --release --example physics_stability`
//!
//! **No longer has a `--gpu` flag.** This example uses the demoted CPU
//! solver's own `RigidBody`/`BodyKind`/`Inertia` components (including a
//! kinematic platform), and the GPU physics path (`physics::gpu`) has been
//! re-scoped to visual-effects-only debris bodies
//! (`physics::gpu::effects::GpuDebrisBody`, kinematic support dropped
//! entirely), which this scene's entities no longer satisfy — passing
//! `--gpu` here would silently do nothing (no bodies extracted, nothing
//! simulated) rather than erroring, so the flag was removed outright
//! instead of leaving it to silently mislead. See `crate::physics_avian`
//! for this project's current primary physics engine (`avian3d`) and
//! `physics::gpu::effects`'s own doc comment for the debris-only GPU
//! path's own new soak-test convention
//! (`many_dynamic_bodies_survive_a_large_dt_spike` in `physics::gpu::frame_test`
//! is this example's own soak-scale regression test, headless-only since
//! this path has no interactive/kinematic showcase left to demonstrate
//! visually the way this example's own `--gpu` flag used to).

use bevy::camera::{Exposure, Hdr};
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::diagnostic::FrameCount;
use bevy::prelude::*;
use bevy::render::view::window::screenshot::{Screenshot, save_to_disk};

use migera::hybrid::HybridRenderPlugin;
use migera::hybrid::extract::SunLight;
use migera::hybrid::material::Material;
use migera::physics::components::{BodyKind, Inertia, PhysicsShape, RigidBody};
use migera::physics::inertia::{box_inertia, capsule_inertia, cylinder_inertia, ellipsoid_inertia, sphere_inertia};
use migera::physics::integrate::PhysicsPlugin;
use migera::physics::solve_static::PhysicsGravity;
use migera::sdf::assembly::SdfSceneRoot;
use migera::sdf::components::Shape;

const PLANET_RADIUS: f32 = 8.0;
const DROP_HEIGHT: f32 = 3.0;

fn main() {
    let assets = std::env::current_dir().expect("cwd").join("assets").to_string_lossy().into_owned();
    if std::env::args().any(|a| a == "--gpu") {
        eprintln!(
            "physics_stability: --gpu is no longer supported here -- the GPU physics path is now visual-effects-only \
             (see physics::gpu::effects) and this example's own RigidBody/BodyKind entities (including its kinematic \
             platform) aren't extracted by it. Run `cargo run --release --example physics_avian_playground` for this \
             project's current primary physics engine (avian3d)."
        );
        std::process::exit(1);
    }

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window { title: "migera physics stability soak".into(), ..default() }),
            ..default()
        }).set(AssetPlugin { file_path: assets, ..default() }))
        .add_plugins(HybridRenderPlugin)
        .add_plugins(PhysicsPlugin)
        .insert_resource(PhysicsGravity { center: Vec3::ZERO, magnitude: 9.81 })
        .insert_resource(ShotConfig::from_args())
        .add_systems(Startup, (spawn_camera, spawn_scene, spawn_light))
        .add_systems(Update, (auto_shot, report_any_instability, drive_kinematic_platform))
        .run();
}

/// `--shot PATH --at-frame N`: headless screenshot-based verification,
/// ported verbatim from `examples/gallery.rs`'s own `ShotConfig`/`auto_shot`
/// convention.
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
                    cfg.shot = Some((cfg.shot.map_or("/tmp/physics_stability.png".into(), |s| s.0), val(&mut i).parse().unwrap_or(120)));
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
        info!("physics_stability: screenshot -> {path}");
        commands.spawn(Screenshot::primary_window()).observe(save_to_disk(path.clone()));
    }
    if *fired && !*exited && frame.0 >= at + 60 {
        *exited = true;
        std::thread::sleep(std::time::Duration::from_millis(600));
        std::process::exit(0);
    }
}

/// Prints a loud, impossible-to-miss warning if any body's `Transform`
/// ever goes non-finite or flies absurdly far from the planet — the
/// stability soak's actual pass/fail signal, since "the render looks
/// fine" alone wouldn't catch a body that silently NaN'd off-screen.
fn report_any_instability(bodies: Query<(Entity, &Transform), With<RigidBody>>, frame: Res<FrameCount>) {
    for (entity, transform) in &bodies {
        let p = transform.translation;
        if !p.is_finite() {
            eprintln!("INSTABILITY frame {}: entity {entity:?} has non-finite position {p:?}", frame.0);
        } else if p.length() > PLANET_RADIUS * 5.0 {
            eprintln!("INSTABILITY frame {}: entity {entity:?} drifted to {p:?} (radius {})", frame.0, p.length());
        }
    }
}

const KINEMATIC_PLATFORM_RISE_SPEED: f32 = 0.3;
const KINEMATIC_PLATFORM_LOW_Y: f32 = PLANET_RADIUS + 1.5;
const KINEMATIC_PLATFORM_HIGH_Y: f32 = KINEMATIC_PLATFORM_LOW_Y + 3.0;

/// Marks the soak scene's kinematic platform for `drive_kinematic_platform`.
#[derive(Component)]
struct KinematicPlatform {
    rising: bool,
}

/// Same manual up/down driving pattern as `physics_playground.rs`'s own
/// elevator, extended into the soak scene per Stage 3.5 Piece 2's own
/// verification requirement: the GPU kinematic-support path needs to be
/// exercised for the same extended-duration soak every other body kind
/// already gets, watching for the same `INSTABILITY` convention plus the
/// contact-generation overflow warning -- neither should fire any more
/// than under CPU with this platform added.
fn drive_kinematic_platform(time: Res<Time>, mut platforms: Query<(&mut Transform, &mut RigidBody, &mut KinematicPlatform)>) {
    let dt = time.delta_secs();
    for (mut transform, mut rigid_body, mut platform) in &mut platforms {
        if platform.rising && transform.translation.y >= KINEMATIC_PLATFORM_HIGH_Y {
            platform.rising = false;
        } else if !platform.rising && transform.translation.y <= KINEMATIC_PLATFORM_LOW_Y {
            platform.rising = true;
        }
        let speed = if platform.rising { KINEMATIC_PLATFORM_RISE_SPEED } else { -KINEMATIC_PLATFORM_RISE_SPEED };
        rigid_body.linear_velocity = Vec3::new(0.0, speed, 0.0);
        transform.translation.y += speed * dt;
    }
}

fn spawn_camera(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Msaa::Off,
        Hdr,
        Tonemapping::default(),
        Exposure::default(),
        Transform::from_xyz(22.0, 16.0, 24.0).looking_at(Vec3::ZERO, Vec3::Y),
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
/// needed) — reproducible run-to-run, which matters for a stability soak:
/// if something breaks, the exact same scene must reproduce it, not a
/// different roll each run.
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

/// One static planetary sphere at the world origin, plus a varied set of
/// dynamic bodies (every shape kind with a closed-form inertia formula:
/// `Sphere`, `RoundedBox`, `RoundedCylinder`, `Capsule`, `Ellipsoid`)
/// dropped from evenly-spread points around the planet's surface, each
/// with its own nontrivial initial rotation and a small non-radial
/// initial velocity (a sideways nudge on top of falling toward the
/// center) — the combination this stability soak specifically wants to
/// exercise: shape variety, non-axis-aligned starts, and gravity whose
/// direction genuinely differs per body (each falls toward the shared
/// center, not a single "down").
fn spawn_scene(mut commands: Commands) {
    let root = commands.spawn((SdfSceneRoot, Transform::IDENTITY, Visibility::default())).id();

    commands.spawn((
        ChildOf(root),
        Shape::Sphere { radius: PLANET_RADIUS },
        PhysicsShape::Sphere { radius: PLANET_RADIUS },
        Transform::IDENTITY,
        Material::new(Vec3::new(0.35, 0.4, 0.5), 0.0, 0.7),
    ));

    spawn_kinematic_platform(&mut commands, root);

    let mut rng = Rng(0x5eed_1234_abcd_ef01);
    let mass = 1.0;

    // Evenly-ish spread points on the planet's surface via the same
    // spherical-Fibonacci lattice `sample_points` uses internally, so
    // bodies genuinely start from many different sides of the planet
    // rather than clustering near one pole.
    const N: usize = 10;
    let golden_angle = std::f32::consts::PI * (3.0 - 5.0f32.sqrt());

    for i in 0..N {
        let y = 1.0 - 2.0 * (i as f32 + 0.5) / N as f32;
        let radius_at_y = (1.0 - y * y).max(0.0).sqrt();
        let theta = golden_angle * i as f32;
        let direction = Vec3::new(theta.cos() * radius_at_y, y, theta.sin() * radius_at_y);

        let rotation = Quat::from_euler(EulerRot::XYZ, rng.range(0.0, std::f32::consts::TAU), rng.range(0.0, std::f32::consts::TAU), rng.range(0.0, std::f32::consts::TAU));

        // A small sideways velocity component, perpendicular to the drop
        // direction, so bodies don't fall in a perfectly straight radial
        // line -- a more realistic stress on the contact solver than a
        // purely head-on approach.
        let (tangent_a, tangent_b) = direction.any_orthonormal_pair();
        let sideways = tangent_a * rng.range(-1.0, 1.0) + tangent_b * rng.range(-1.0, 1.0);
        let initial_velocity = sideways.normalize_or_zero() * rng.range(0.0, 1.5);

        let position = direction * (PLANET_RADIUS + DROP_HEIGHT);

        match i % 5 {
            0 => {
                let radius = 0.5;
                spawn_body(
                    &mut commands,
                    root,
                    Shape::Sphere { radius },
                    PhysicsShape::Sphere { radius },
                    position,
                    rotation,
                    initial_velocity,
                    1.0 / sphere_inertia(mass, radius),
                    Vec3::new(0.8, 0.25, 0.2),
                );
            }
            1 => {
                let half_extents = Vec3::new(0.5, 0.5, 0.5);
                spawn_body(
                    &mut commands,
                    root,
                    Shape::RoundedBox { half_extents, corner_radius: 0.05 },
                    PhysicsShape::RoundedBox { half_extents, corner_radius: 0.05 },
                    position,
                    rotation,
                    initial_velocity,
                    1.0 / box_inertia(mass, half_extents),
                    Vec3::new(0.7, 0.55, 0.35),
                );
            }
            2 => {
                let (radius, half_height) = (0.4, 0.5);
                spawn_body(
                    &mut commands,
                    root,
                    Shape::RoundedCylinder { radius, half_height, edge_radius: 0.08 },
                    PhysicsShape::RoundedCylinder { radius, half_height, edge_radius: 0.08 },
                    position,
                    rotation,
                    initial_velocity,
                    1.0 / cylinder_inertia(mass, radius, half_height),
                    Vec3::new(0.3, 0.6, 0.4),
                );
            }
            3 => {
                let (a, b, radius) = (Vec3::new(0.0, -0.4, 0.0), Vec3::new(0.0, 0.4, 0.0), 0.3);
                spawn_body(
                    &mut commands,
                    root,
                    Shape::Capsule { a, b, radius },
                    PhysicsShape::Capsule { a, b, radius },
                    position,
                    rotation,
                    initial_velocity,
                    1.0 / capsule_inertia(mass, radius, 0.4),
                    Vec3::new(0.5, 0.4, 0.7),
                );
            }
            _ => {
                let radii = Vec3::new(0.6, 0.4, 0.5);
                spawn_body(
                    &mut commands,
                    root,
                    Shape::Ellipsoid { radii },
                    PhysicsShape::Ellipsoid { radii },
                    position,
                    rotation,
                    initial_velocity,
                    1.0 / ellipsoid_inertia(mass, radii),
                    Vec3::new(0.8, 0.7, 0.2),
                );
            }
        }
    }
}

/// A `BodyKind::Kinematic` platform above the planet's "north pole,"
/// carrying a resting dynamic box up and down -- Stage 3.5 Piece 2's own
/// GPU soak-test extension, exercising the moving-platform GPU path
/// alongside every other shape kind this scene already stresses.
fn spawn_kinematic_platform(commands: &mut Commands, root: Entity) {
    let half_extents = Vec3::new(1.2, 0.25, 1.2);
    commands.spawn((
        ChildOf(root),
        Shape::RoundedBox { half_extents, corner_radius: 0.05 },
        PhysicsShape::RoundedBox { half_extents, corner_radius: 0.05 },
        Transform::from_xyz(0.0, KINEMATIC_PLATFORM_LOW_Y, 0.0),
        Material::new(Vec3::new(0.3, 0.5, 0.75), 0.0, 0.4),
        RigidBody { linear_velocity: Vec3::new(0.0, KINEMATIC_PLATFORM_RISE_SPEED, 0.0), angular_velocity: Vec3::ZERO },
        Inertia::STATIC,
        BodyKind::Kinematic,
        KinematicPlatform { rising: true },
    ));

    let box_half_extents = Vec3::splat(0.5);
    commands.spawn((
        ChildOf(root),
        Shape::RoundedBox { half_extents: box_half_extents, corner_radius: 0.05 },
        PhysicsShape::RoundedBox { half_extents: box_half_extents, corner_radius: 0.05 },
        Transform::from_xyz(0.0, KINEMATIC_PLATFORM_LOW_Y + half_extents.y + box_half_extents.y + 0.02, 0.0),
        Material::new(Vec3::new(0.85, 0.7, 0.3), 0.0, 0.5),
        RigidBody::default(),
        Inertia { inverse_mass: 1.0, inverse_tensor_diag: 1.0 / box_inertia(1.0, box_half_extents) },
    ));
}

#[allow(clippy::too_many_arguments)]
fn spawn_body(
    commands: &mut Commands,
    root: Entity,
    shape: Shape,
    physics_shape: PhysicsShape,
    position: Vec3,
    rotation: Quat,
    initial_velocity: Vec3,
    inverse_tensor_diag: Vec3,
    color: Vec3,
) {
    commands.spawn((
        ChildOf(root),
        shape,
        physics_shape,
        Transform { translation: position, rotation, ..default() },
        Material::new(color, 0.0, 0.5),
        RigidBody { linear_velocity: initial_velocity, angular_velocity: Vec3::ZERO },
        Inertia { inverse_mass: 1.0, inverse_tensor_diag },
    ));
}
