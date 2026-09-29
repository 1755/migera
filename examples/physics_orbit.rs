//! Orbit milestone: a small sphere launched tangent to a point-source
//! gravity center at the circular-orbit speed for its altitude
//! (`v = sqrt(g * r)`) traces a stable, roughly circular path — fully
//! resolved by `physics::solve_world`'s real gravity integration + XPBD
//! substep loop, no scripted trajectory. Deliberately has NO static
//! collider anywhere in the scene: an orbit test has nothing to rest on,
//! the entire point is staying aloft under gravity alone, so this also
//! exercises the zero-contact free-flight path continuously over many
//! frames rather than settling into a contact-dominated steady state like
//! `physics_stability.rs`'s planet-drop scenes.
//!
//! See `physics::solve_world`'s own test
//! `a_body_launched_at_orbital_velocity_completes_a_stable_orbit` for the
//! headless, assertion-based version of this same scenario.
//!
//! Run: `cargo run --release --example physics_orbit`

use bevy::camera::{Exposure, Hdr};
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::diagnostic::FrameCount;
use bevy::prelude::*;
use bevy::render::view::window::screenshot::{Screenshot, save_to_disk};

use migera::hybrid::HybridRenderPlugin;
use migera::hybrid::extract::SunLight;
use migera::hybrid::material::Material;
use migera::physics::components::{Inertia, PhysicsShape, RigidBody};
use migera::physics::integrate::PhysicsPlugin;
use migera::physics::solve_static::PhysicsGravity;
use migera::sdf::assembly::SdfSceneRoot;
use migera::sdf::components::Shape;

const GRAVITY_MAGNITUDE: f32 = 9.81;
const ORBIT_RADIUS: f32 = 10.0;
/// Purely cosmetic: a small sphere marking the gravity center itself, with
/// no `PhysicsShape` collider — it's there so the orbit has something
/// visible to orbit AROUND, not a collidable body (an orbit test must have
/// nothing for the orbiting body to rest on).
const CENTER_MARKER_RADIUS: f32 = 1.2;

fn main() {
    let assets = std::env::current_dir().expect("cwd").join("assets").to_string_lossy().into_owned();

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window { title: "migera physics orbit".into(), ..default() }),
            ..default()
        }).set(AssetPlugin { file_path: assets, ..default() }))
        .add_plugins(HybridRenderPlugin)
        .add_plugins(PhysicsPlugin)
        .insert_resource(PhysicsGravity { center: Vec3::ZERO, magnitude: GRAVITY_MAGNITUDE })
        .insert_resource(ShotConfig::from_args())
        .add_systems(Startup, (spawn_camera, spawn_scene, spawn_light))
        .add_systems(Update, (auto_shot, report_orbit_radius))
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
                    cfg.shot = Some((cfg.shot.map_or("/tmp/physics_orbit.png".into(), |s| s.0), val(&mut i).parse().unwrap_or(120)));
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
        info!("physics_orbit: screenshot -> {path}");
        commands.spawn(Screenshot::primary_window()).observe(save_to_disk(path.clone()));
    }
    if *fired && !*exited && frame.0 >= at + 60 {
        *exited = true;
        std::thread::sleep(std::time::Duration::from_millis(600));
        std::process::exit(0);
    }
}

/// Loud, impossible-to-miss signal if the orbit ever decays into the
/// center or flies off to infinity — printed periodically so a live run
/// (not just the headless test) shows the radius staying bounded over
/// time.
fn report_orbit_radius(bodies: Query<&Transform, With<RigidBody>>, frame: Res<FrameCount>) {
    if frame.0 % 60 != 0 {
        return;
    }
    for transform in &bodies {
        let r = transform.translation.length();
        if !transform.translation.is_finite() {
            eprintln!("INSTABILITY frame {}: orbit position went non-finite: {:?}", frame.0, transform.translation);
        } else if !(ORBIT_RADIUS * 0.5..ORBIT_RADIUS * 2.0).contains(&r) {
            eprintln!("INSTABILITY frame {}: orbit radius {r} drifted far from the starting radius {ORBIT_RADIUS}", frame.0);
        } else {
            info!("frame {}: orbit radius = {r:.3} (started at {ORBIT_RADIUS})", frame.0);
        }
    }
}

fn spawn_camera(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Msaa::Off,
        Hdr,
        Tonemapping::default(),
        Exposure::default(),
        // Looking down the orbit plane's normal (+Y) at a steep angle so
        // the circular path is clearly visible as a circle, not
        // foreshortened into a line.
        Transform::from_xyz(0.0, 22.0, 18.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}

fn spawn_light(mut commands: Commands) {
    commands.spawn((
        SunLight,
        DirectionalLight { color: Color::srgb(1.0, 0.95, 0.85), illuminance: 1500.0, ..default() },
        Transform::from_xyz(10.0, 20.0, 15.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}

fn spawn_scene(mut commands: Commands) {
    let root = commands.spawn((SdfSceneRoot, Transform::IDENTITY, Visibility::default())).id();

    // Purely visual gravity-center marker -- no PhysicsShape, so nothing
    // for the orbiting body to ever collide against.
    commands.spawn((
        ChildOf(root),
        Shape::Sphere { radius: CENTER_MARKER_RADIUS },
        Transform::IDENTITY,
        Material::new(Vec3::new(0.9, 0.75, 0.2), 0.0, 0.3),
    ));

    // Circular-orbit speed at this altitude: point-source gravity here has
    // CONSTANT magnitude regardless of distance
    // (`PhysicsGravity::acceleration_at`'s own doc comment), so centripetal
    // acceleration `v^2 / r = g` gives the standard `v = sqrt(g * r)`
    // directly, no inverse-square correction needed.
    let orbital_speed = (GRAVITY_MAGNITUDE * ORBIT_RADIUS).sqrt();
    let start_position = Vec3::new(ORBIT_RADIUS, 0.0, 0.0);
    let orbital_velocity = Vec3::new(0.0, 0.0, orbital_speed);

    let radius = 0.4;
    commands.spawn((
        ChildOf(root),
        Shape::Sphere { radius },
        PhysicsShape::Sphere { radius },
        Transform::from_translation(start_position),
        Material::new(Vec3::new(0.3, 0.55, 0.9), 0.0, 0.4),
        RigidBody { linear_velocity: orbital_velocity, angular_velocity: Vec3::ZERO },
        Inertia { inverse_mass: 1.0, inverse_tensor_diag: Vec3::ZERO },
    ));
}
