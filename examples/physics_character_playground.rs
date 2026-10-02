//! An animated character in a physical world.
//!
//! A 25 × 25 m room: a floor and four 3 m walls, all static colliders.
//! Characters walk straight, and when one nears a wall it turns away along
//! the wall's reflection of its heading (the mirrored ray), plus a small
//! random jitter so no two bounces repeat; one nearing another turns aside.
//! Cubes, spheres and capsules drop from 5 m around the centre and lie
//! where they land, for the characters to walk into: a foot coming down on
//! one low enough to step on stands on it (`PhysicsGround`), a taller one
//! is pushed.
//!
//! Each character is physical in one of two ways, by its distance from the
//! camera (`--physics lod`, the default):
//! - near: a pinned active ragdoll (`AnimRagdollPlugin`), every limb a
//!   body tracking the animation, so legs kick props and props knock limbs
//!   (and a hard enough knock fells it, after which it gets up);
//! - far: one kinematic capsule moving with it, pushing props aside, at a
//!   fraction of the cost.
//!
//! `--physics ragdoll` or `kinematic` holds every character to one.
//!
//! Camera: a free-flight camera (`FreeCamera`): WASD to move, Q/E down/up,
//! hold the right mouse button (or toggle with M) to look, Shift to run,
//! the scroll wheel for speed.
//!
//! Run: `cargo run --release --example physics_character_playground`
//!
//! Flags: `--characters N` (1), `--props N` (30), `--speed M_PER_S` (1.2),
//! `--jitter DEGREES` (15), `--ragdoll-distance M` (10), `--seed N`,
//! `--physics lod|ragdoll|kinematic`, `--gizmos on`,
//! `--shot PATH --at-frame N` (a screenshot, then exit),
//! `--camera X,Y,Z` and `--look X,Y,Z` (where the camera starts, and what
//! at), `--avoid off`, `--plank H` (a feet-on-props test: one plank across
//! the first character's path, no props), `--trace-feet` (prints the first
//! character's ankles and heading every frame).

use std::f32::consts::PI;

use avian3d::prelude::*;
use bevy::camera_controller::free_camera::{FreeCamera, FreeCameraPlugin};
use bevy::diagnostic::{FrameCount, FrameTimeDiagnosticsPlugin};
use bevy::prelude::*;
use bevy::render::view::window::screenshot::{save_to_disk, Screenshot};

use migera::character::anim::asset::AnimAssetPlugin;
use migera::character::anim::physics_ground::{PhysicsGround, PhysicsGroundPlugin};
use migera::character::anim::plugin::AnimFootIk;
use migera::character::anim::ragdoll_plugin::sole_blocks;
use migera::character::anim::{
    despawn_ragdoll, spawn_gltf_humanoid, spawn_ragdoll, AnimPlugin, AnimRagdollPlugin, HumanoidPlugin, Ragdoll, RagdollSpawnConfig,
    Steer, Walker, WalkerPlugin, WalkerSet, WalkerState,
};
use migera::character::{Bone, HumanoidSkeleton};

/// Half the room's side, metres.
const HALF_ROOM: f32 = 12.5;
const WALL_HEIGHT: f32 = 3.0;
const WALL_THICKNESS: f32 = 0.5;
/// The walls' own collision layer, which the bounce's ray looks for alone.
const WALL_LAYER: LayerMask = LayerMask(1 << 1);
/// How far ahead a walker looks for a wall, metres: at 1.2 m/s and a turn
/// of [`BOUNCE_TURN_RATE`] it turns within ~1 m.
const LOOK_AHEAD: f32 = 2.0;
/// The side rays' angle from the heading, radians.
const WHISKER: f32 = 0.6;
/// The least angle a walker leaves a wall at, radians.
const LEAVE_ANGLE: f32 = 0.45;
/// How fast a walker turns away from a wall, radians per second.
const BOUNCE_TURN_RATE: f32 = 2.5;
/// How much nearer than `--ragdoll-distance` a character must come to get
/// its ragdoll, and how much further to lose it, metres: so one standing
/// at the boundary does not flicker between the two.
const HYSTERESIS: f32 = 1.0;

#[derive(Resource, Clone)]
struct Config {
    characters: usize,
    props: usize,
    speed: f32,
    jitter: f32,
    ragdoll_distance: f32,
    seed: u64,
    physics: PhysicsMode,
    gizmos: bool,
    avoid: bool,
    /// `--plank H`: a test of feet on props. No props; one H m plank lying
    /// across the first character's path, which starts facing it.
    plank: Option<f32>,
    trace_feet: bool,
    /// `--bench SECS`: vsync off, 3 s to settle, then frame times for SECS
    /// seconds, printed (p50, p99), and exit.
    bench: Option<f32>,
    flat_ground: bool,
    shot: Option<(String, u32)>,
    camera: Vec3,
    /// What the camera starts looking at (`--look X,Y,Z`).
    look: Vec3,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum PhysicsMode {
    Lod,
    Ragdoll,
    Kinematic,
    /// No body at all: the animation's cost alone, for `--bench`.
    None,
}

impl Config {
    fn from_args() -> Self {
        let mut config = Self {
            characters: 1,
            props: 30,
            speed: 1.2,
            jitter: 15f32.to_radians(),
            ragdoll_distance: 10.0,
            seed: 7,
            physics: PhysicsMode::Lod,
            gizmos: false,
            avoid: true,
            plank: None,
            trace_feet: false,
            bench: None,
            flat_ground: false,
            shot: None,
            camera: Vec3::new(0.0, 9.0, 22.0),
            look: Vec3::new(0.0, 1.0, 0.0),
        };
        let mut args = std::env::args().skip(1);
        let mut at_frame = 300;
        while let Some(arg) = args.next() {
            let mut value = || args.next().unwrap_or_default();
            match arg.as_str() {
                "--characters" => config.characters = value().parse().unwrap_or(config.characters),
                "--props" => config.props = value().parse().unwrap_or(config.props),
                "--speed" => config.speed = value().parse().unwrap_or(config.speed),
                "--jitter" => config.jitter = value().parse::<f32>().map(f32::to_radians).unwrap_or(config.jitter),
                "--ragdoll-distance" => config.ragdoll_distance = value().parse().unwrap_or(config.ragdoll_distance),
                "--seed" => config.seed = value().parse().unwrap_or(config.seed),
                "--gizmos" => config.gizmos = value() != "off",
                "--avoid" => config.avoid = value() != "off",
                "--plank" => config.plank = value().parse().ok(),
                "--trace-feet" => config.trace_feet = true,
                "--bench" => config.bench = value().parse().ok(),
                "--ground" => config.flat_ground = value() == "flat",
                "--physics" => {
                    config.physics = match value().as_str() {
                        "ragdoll" => PhysicsMode::Ragdoll,
                        "kinematic" => PhysicsMode::Kinematic,
                        "none" => PhysicsMode::None,
                        _ => PhysicsMode::Lod,
                    }
                }
                "--shot" => config.shot = Some((value(), at_frame)),
                "--at-frame" => {
                    at_frame = value().parse().unwrap_or(at_frame);
                    if let Some(shot) = config.shot.as_mut() {
                        shot.1 = at_frame;
                    }
                }
                "--camera" | "--look" => {
                    let parts: Vec<f32> = value().split(',').filter_map(|p| p.trim().parse().ok()).collect();
                    if let [x, y, z] = parts[..] {
                        if arg == "--camera" {
                            config.camera = Vec3::new(x, y, z);
                        } else {
                            config.look = Vec3::new(x, y, z);
                        }
                    }
                }
                _ => {}
            }
        }
        config
    }
}

/// A walker that turns away from walls.
#[derive(Component, Default)]
struct WallBouncer {
    /// The heading it is turning onto, until it gets there; no new bounce
    /// is taken meanwhile, or the ray, still on the wall, would turn it
    /// back.
    turning_to: Option<f32>,
    /// The heading it is turning onto aside from another character. A wall
    /// overrides it: blocking the wall's check, an avoiding turn took a
    /// walker to within 0.38 m of a wall.
    avoiding_to: Option<f32>,
}

/// The kinematic capsule standing in for a far character's body.
#[derive(Component)]
struct ProxyOf(Entity);

/// A character's kinematic capsule, if it has one now.
#[derive(Component)]
struct Proxy(Entity);

/// The capsule's radius and its straight length, metres: a body's width,
/// from a step's height (`PhysicsGround::max_step`) to a 1.75 m
/// character's head. Down to the floor, it shoved every prop out from under
/// the feet before they could step on it.
const PROXY_RADIUS: f32 = 0.25;
const PROXY_BOTTOM: f32 = 0.35;
const PROXY_LENGTH: f32 = 1.75 - PROXY_BOTTOM - 2.0 * PROXY_RADIUS;
/// How near another character a walker looks, metres; how far ahead it
/// predicts their paths, seconds; how close it lets them pass, metres; and
/// how far it turns aside each time, radians.
const AVOID_RADIUS: f32 = 3.5;
const AVOID_HORIZON: f32 = 1.5;
const AVOID_CLEARANCE: f32 = 1.0;
const AVOID_TURN: f32 = 0.6;

fn main() {
    let assets = std::env::current_dir().expect("cwd").join("assets").to_string_lossy().into_owned();
    let config = Config::from_args();
    App::new()
        .add_plugins(
            DefaultPlugins
                .set(WindowPlugin {
                    primary_window: Some(Window {
                        title: "migera physics character playground".into(),
                        // Uncapped for `--bench`: at vsync every frame reads the
                        // display's 16.7 ms whatever it cost.
                        present_mode: if config.bench.is_some() { bevy::window::PresentMode::AutoNoVsync } else { default() },
                        ..default()
                    }),
                    ..default()
                })
                .set(AssetPlugin { file_path: assets, ..default() }),
        )
        .add_plugins(FrameTimeDiagnosticsPlugin::default())
        // Bevy Remote Protocol on 127.0.0.1:15702, for reading the live
        // world (positions, heights) rather than eyeballing screenshots.
        .add_plugins((bevy::remote::RemotePlugin::default(), bevy::remote::http::RemoteHttpPlugin::default()))
        // Twelve substeps, as the character gallery: at avian's six a fallen
        // ragdoll's resting contacts jittered and it crept across the floor.
        .add_plugins(PhysicsPlugins::default())
        .insert_resource(SubstepCount(12))
        .add_plugins((AnimPlugin, AnimAssetPlugin, HumanoidPlugin, WalkerPlugin, AnimRagdollPlugin, PhysicsGroundPlugin, FreeCameraPlugin))
        .insert_resource(config)
        .add_systems(Startup, (spawn_room, spawn_props, spawn_characters, spawn_camera_and_light, spawn_hud))
        .add_systems(Update, (bounce_off_walls, avoid_each_other).chain().before(WalkerSet::Drive))
        .add_systems(Update, (choose_physics, draw_bodies, update_hud, auto_shot, bench))
        .add_systems(PostUpdate, trace_feet.after(TransformSystems::Propagate))
        .add_systems(FixedUpdate, carry_proxies)
        .run();
}

fn spawn_camera_and_light(mut commands: Commands, config: Res<Config>) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_translation(config.camera).looking_at(config.look, Vec3::Y),
        FreeCamera { walk_speed: 6.0, run_speed: 18.0, ..default() },
    ));
    commands.spawn((
        DirectionalLight { illuminance: 12_000.0, shadow_maps_enabled: true, ..default() },
        Transform::from_rotation(Quat::from_euler(EulerRot::YXZ, 0.6, -0.9, 0.0)),
    ));
    commands.insert_resource(GlobalAmbientLight { brightness: 400.0, ..default() });
}

/// The floor and four walls, static, drawn and solid.
fn spawn_room(mut commands: Commands, mut meshes: ResMut<Assets<Mesh>>, mut materials: ResMut<Assets<StandardMaterial>>) {
    let side = HALF_ROOM * 2.0;
    commands.spawn((
        RigidBody::Static,
        Collider::cuboid(side, 0.2, side),
        Friction::new(1.0),
        Mesh3d(meshes.add(Cuboid::new(side, 0.2, side))),
        MeshMaterial3d(materials.add(Color::srgb(0.45, 0.47, 0.45))),
        // Its top at y = 0, where the walkers' flat ground is.
        Transform::from_xyz(0.0, -0.1, 0.0),
    ));
    let wall = materials.add(Color::srgb(0.62, 0.58, 0.52));
    let long = side + WALL_THICKNESS * 2.0;
    for (size, at) in [
        (Vec3::new(long, WALL_HEIGHT, WALL_THICKNESS), Vec3::new(0.0, 0.0, -HALF_ROOM - WALL_THICKNESS * 0.5)),
        (Vec3::new(long, WALL_HEIGHT, WALL_THICKNESS), Vec3::new(0.0, 0.0, HALF_ROOM + WALL_THICKNESS * 0.5)),
        (Vec3::new(WALL_THICKNESS, WALL_HEIGHT, side), Vec3::new(-HALF_ROOM - WALL_THICKNESS * 0.5, 0.0, 0.0)),
        (Vec3::new(WALL_THICKNESS, WALL_HEIGHT, side), Vec3::new(HALF_ROOM + WALL_THICKNESS * 0.5, 0.0, 0.0)),
    ] {
        commands.spawn((
            RigidBody::Static,
            Collider::cuboid(size.x, size.y, size.z),
            // Its own layer as well as the default, so the bounce's ray finds
            // walls alone, never a prop lying in the way.
            CollisionLayers::new(LayerMask::DEFAULT | WALL_LAYER, LayerMask::ALL),
            Mesh3d(meshes.add(Cuboid::new(size.x, size.y, size.z))),
            MeshMaterial3d(wall.clone()),
            Transform::from_translation(at + Vec3::Y * WALL_HEIGHT * 0.5),
        ));
    }
}

/// Cubes, spheres and capsules, dynamic, dropped from 5 m around the centre.
fn spawn_props(mut commands: Commands, config: Res<Config>, mut meshes: ResMut<Assets<Mesh>>, mut materials: ResMut<Assets<StandardMaterial>>) {
    if let Some(height) = config.plank {
        // A platform across the path, deep enough to walk on with both feet,
        // heavy enough not to skate away.
        commands.spawn((
            RigidBody::Dynamic,
            Collider::cuboid(2.0, height, 2.5),
            Mass(150.0),
            Friction::new(1.0),
            Mesh3d(meshes.add(Cuboid::new(2.0, height, 2.5))),
            MeshMaterial3d(materials.add(Color::srgb(0.55, 0.4, 0.25))),
            Transform::from_xyz(0.0, height * 0.5, -3.75),
        ));
        return;
    }
    let mut rng = fastrand::Rng::with_seed(config.seed);
    for i in 0..config.props {
        let angle = rng.f32() * PI * 2.0;
        // Around the centre, clear of the first character standing there.
        let reach = 1.5 + rng.f32() * 5.0;
        let at = Vec3::new(angle.cos() * reach, 5.0 + rng.f32() * 2.0, angle.sin() * reach);
        let size = 0.25 + rng.f32() * 0.35;
        let color = Color::hsl(rng.f32() * 360.0, 0.55, 0.55);
        let (collider, mesh): (Collider, Mesh) = match i % 3 {
            0 => (Collider::cuboid(size, size, size), Cuboid::new(size, size, size).into()),
            1 => (Collider::sphere(size * 0.5), Sphere::new(size * 0.5).into()),
            _ => (Collider::capsule(size * 0.3, size), Capsule3d::new(size * 0.3, size).into()),
        };
        commands.spawn((
            RigidBody::Dynamic,
            collider,
            Friction::new(0.6),
            Mesh3d(meshes.add(mesh)),
            MeshMaterial3d(materials.add(color)),
            Transform::from_translation(at).with_rotation(Quat::from_euler(EulerRot::XYZ, rng.f32() * PI, rng.f32() * PI, 0.0)),
        ));
    }
}

/// The first character at the centre, the rest scattered; each walking
/// straight on a random heading.
fn spawn_characters(mut commands: Commands, asset_server: Res<AssetServer>, config: Res<Config>) {
    let mut rng = fastrand::Rng::with_seed(config.seed ^ 0x9e37);
    for i in 0..config.characters {
        let at = if i == 0 {
            Vec3::ZERO
        } else {
            Vec3::new((rng.f32() * 2.0 - 1.0) * (HALF_ROOM - 4.0), 0.0, (rng.f32() * 2.0 - 1.0) * (HALF_ROOM - 4.0))
        };
        let yaw = rng.f32() * PI * 2.0 - PI;
        // Facing the plank (along -Z, yaw zero), for its test.
        let yaw = if i == 0 && config.plank.is_some() { 0.0 } else { yaw };
        // `puppet_base.gltf` faces +Z; this crate's forward is -Z.
        let root = spawn_gltf_humanoid(&mut commands, &asset_server, "models/puppet_base.gltf", PI, Transform::from_translation(at).with_rotation(Quat::from_rotation_y(yaw)));
        commands.entity(root).insert((Walker { speed: config.speed, steer: Steer::Toward { yaw, rate: BOUNCE_TURN_RATE }, ..default() }, WallBouncer::default()));
        // Feet on the physics world: on a prop low enough to step on
        // (`--ground flat`: the floor alone, for the bench).
        if !config.flat_ground {
            commands.entity(root).insert(PhysicsGround::default());
        }
    }
}

/// Turns each walker nearing a wall onto the wall's reflection of its
/// heading, `d − 2(d·n)n`, give or take `--jitter`.
fn bounce_off_walls(config: Res<Config>, spatial: SpatialQuery, mut walkers: Query<(&mut Walker, &mut WallBouncer, &WalkerState, &Transform)>, mut rng: Local<Option<fastrand::Rng>>) {
    let rng = rng.get_or_insert_with(|| fastrand::Rng::with_seed(config.seed ^ 0xb0b));
    let walls = SpatialQueryFilter::from_mask(WALL_LAYER);
    for (mut walker, mut bouncer, state, transform) in &mut walkers {
        let yaw = state.facing.yaw;
        if let Some(target) = bouncer.turning_to {
            if angle_between(yaw, target) < 0.05 {
                bouncer.turning_to = None;
            } else {
                continue;
            }
        }
        // Yaw zero faces -Z; the heading turns about +Y.
        let ahead = Quat::from_rotation_y(yaw) * Vec3::NEG_Z;
        let origin = transform.translation + Vec3::Y * 1.0;
        // Straight ahead and two whiskers to the sides: met at a shallow
        // angle, a wall reached the straight ray only when the walker was
        // nearly touching it (2 m × sin 5° ≈ 0.17 m; measured 0.13).
        let hit = [0.0f32, WHISKER, -WHISKER].into_iter().find_map(|angle| {
            let direction = Dir3::new(Quat::from_rotation_y(angle) * ahead).ok()?;
            spatial.cast_ray(origin, direction, LOOK_AHEAD, true, &walls)
        });
        let Some(hit) = hit else { continue };
        let normal = Vec3::new(hit.normal.x, 0.0, hit.normal.z).normalize_or_zero();
        // Only a wall it is heading into; one it already walks away from or
        // along, a whisker merely grazed.
        if normal == Vec3::ZERO || ahead.dot(normal) >= 0.0 {
            continue;
        }
        let mirrored = ahead - normal * (2.0 * ahead.dot(normal));
        let jitter = (rng.f32() * 2.0 - 1.0) * config.jitter;
        let mut leaving = Quat::from_rotation_y(jitter) * mirrored;
        // At least `LEAVE_ANGLE` off the wall: from a shallow approach the
        // reflection leaves it only shallowly, and the jitter turned it back
        // in, to walk along the wall into the next ray.
        let away = leaving.dot(normal);
        let least = LEAVE_ANGLE.sin();
        if away < least {
            let along = (leaving - normal * away).normalize_or_zero();
            leaving = along * LEAVE_ANGLE.cos() + normal * least;
        }
        let target = yaw_of(leaving);
        walker.steer = Steer::Toward { yaw: target, rate: BOUNCE_TURN_RATE };
        bouncer.turning_to = Some(target);
        bouncer.avoiding_to = None;
    }
}

/// Turns each walker aside from the other character it would come nearest,
/// predicted from both headings and speeds over [`AVOID_HORIZON`]: if
/// they would pass closer than [`AVOID_CLEARANCE`], it turns away from the
/// side the other would pass on, or to the right when dead ahead, so two
/// meeting head-on both keep right. Looking only at who stood ahead, it
/// missed characters converging from the side. A wall's turn goes first.
fn avoid_each_other(config: Res<Config>, mut walkers: Query<(Entity, &mut Walker, &mut WallBouncer, &WalkerState, &Transform)>) {
    if !config.avoid {
        return;
    }
    let moving: Vec<(Entity, Vec3, Vec3)> = walkers
        .iter()
        .map(|(entity, walker, _, state, transform)| (entity, transform.translation, Quat::from_rotation_y(state.facing.yaw) * Vec3::NEG_Z * walker.speed))
        .collect();
    for (entity, mut walker, mut bouncer, state, transform) in &mut walkers {
        let yaw = state.facing.yaw;
        if let Some(target) = bouncer.avoiding_to
            && angle_between(yaw, target) < 0.05
        {
            bouncer.avoiding_to = None;
        }
        if bouncer.turning_to.is_some() || bouncer.avoiding_to.is_some() {
            continue;
        }
        let ahead = Quat::from_rotation_y(yaw) * Vec3::NEG_Z;
        let own = ahead * walker.speed;
        // The soonest pass closer than the clearance: where the other will be,
        // relative to this one, at their closest.
        let threat = moving
            .iter()
            .filter(|(other, ..)| *other != entity)
            .filter_map(|(_, at, velocity)| {
                let rel = Vec3::new(at.x - transform.translation.x, 0.0, at.z - transform.translation.z);
                let closing = *velocity - own;
                let t = if closing.length_squared() > 1.0e-6 { (-rel.dot(closing) / closing.length_squared()).clamp(0.0, AVOID_HORIZON) } else { 0.0 };
                let closest = rel + closing * t;
                (closest.length() < AVOID_CLEARANCE && rel.length() < AVOID_RADIUS).then_some((t, closest))
            })
            .min_by(|a, b| a.0.total_cmp(&b.0));
        let Some((_, closest)) = threat else { continue };
        // Positive: it would pass on the left (yaw grows to the left).
        let side = ahead.cross(closest).y;
        let away = if side > 0.05 { -AVOID_TURN } else if side < -0.05 { AVOID_TURN } else { -AVOID_TURN };
        let target = yaw + away;
        walker.steer = Steer::Toward { yaw: target, rate: BOUNCE_TURN_RATE };
        bouncer.avoiding_to = Some(target);
    }
}

/// The yaw (about +Y, zero along -Z) a horizontal direction points along.
fn yaw_of(direction: Vec3) -> f32 {
    (-direction.x).atan2(-direction.z)
}

/// The unsigned angle between two yaws, radians.
fn angle_between(a: f32, b: f32) -> f32 {
    let d = (a - b).rem_euclid(PI * 2.0);
    d.min(PI * 2.0 - d)
}

/// Gives each character the physics its distance from the camera earns: a
/// ragdoll near, a kinematic capsule far (`--physics lod`), or one of them
/// always. A falling ragdoll keeps its body until it is up again.
#[allow(clippy::type_complexity)]
fn choose_physics(
    mut commands: Commands,
    config: Res<Config>,
    cameras: Query<&GlobalTransform, With<Camera3d>>,
    mut characters: Query<
        (Entity, &HumanoidSkeleton, &AnimFootIk, &Transform, Option<&Ragdoll>, Option<&Proxy>, Option<&mut PhysicsGround>),
        With<WalkerState>,
    >,
    global_transforms: Query<&GlobalTransform>,
) {
    let Ok(camera) = cameras.single() else { return };
    for (entity, skeleton, foot_ik, transform, ragdoll, proxy, mut ground) in &mut characters {
        // Not until the foot IK has measured the live rig (the ragdoll's
        // soles are built from it) and the bones have been placed.
        let Some(rig) = foot_ik.rig.as_ref() else { continue };
        if global_transforms.get(skeleton.entity(Bone::Hips)).is_ok_and(|hips| hips.translation() == Vec3::ZERO) {
            continue;
        }
        let distance = camera.translation().distance(transform.translation);
        // `--physics none`: the animation alone, the bench's baseline.
        if config.physics == PhysicsMode::None {
            continue;
        }
        let near = match config.physics {
            PhysicsMode::None => false,
            PhysicsMode::Ragdoll => true,
            PhysicsMode::Kinematic => false,
            PhysicsMode::Lod if ragdoll.is_some() => distance < config.ragdoll_distance + HYSTERESIS,
            PhysicsMode::Lod => distance < config.ragdoll_distance - HYSTERESIS,
        };
        match (near, ragdoll) {
            (true, None) => {
                info!("playground: {entity} {distance:.1} m away, ragdoll");
                let ragdoll = spawn_ragdoll(&mut commands, entity, skeleton, &global_transforms, &RagdollSpawnConfig { feet: Some(sole_blocks(rig)), ..default() });
                commands.entity(entity).insert(ragdoll);
                if let Some(proxy) = proxy {
                    commands.entity(proxy.0).despawn();
                    commands.entity(entity).remove::<Proxy>();
                }
                if let Some(ground) = ground.as_mut() {
                    ground.ignore.clear();
                }
            }
            (false, Some(ragdoll)) if !ragdoll.is_falling() => {
                info!("playground: {entity} {distance:.1} m away, kinematic capsule");
                despawn_ragdoll(&mut commands, entity, ragdoll);
            }
            (false, None) if proxy.is_none() => {
                let capsule = commands
                    .spawn((
                        RigidBody::Kinematic,
                        Collider::capsule(PROXY_RADIUS, PROXY_LENGTH),
                        Transform::from_translation(capsule_centre(transform.translation)),
                        ProxyOf(entity),
                    ))
                    .id();
                commands.entity(entity).insert(Proxy(capsule));
                // The feet's rays pass through the character's own capsule.
                if let Some(ground) = ground.as_mut() {
                    ground.ignore = vec![capsule];
                }
            }
            _ => {}
        }
    }
}

/// Where a character's capsule's centre is: its bottom a step above the
/// floor under it.
fn capsule_centre(feet: Vec3) -> Vec3 {
    feet + Vec3::Y * (PROXY_BOTTOM + PROXY_RADIUS + PROXY_LENGTH * 0.5)
}

/// Moves each kinematic capsule after its character by velocity, every
/// physics step, so what it pushes is pushed at the character's own pace
/// (teleported, a kinematic body hands its contacts no velocity).
fn carry_proxies(time: Res<Time>, characters: Query<&Transform, Without<ProxyOf>>, mut proxies: Query<(&ProxyOf, &Position, &mut LinearVelocity)>) {
    let dt = time.delta_secs().max(1.0e-4);
    for (of, position, mut velocity) in &mut proxies {
        let Ok(character) = characters.get(of.0) else { continue };
        velocity.0 = (capsule_centre(character.translation) - position.0) / dt;
    }
}

/// `--gizmos on`: every ragdoll body and kinematic capsule drawn, the
/// physics the animation is standing in.
fn draw_bodies(
    config: Res<Config>,
    mut gizmos: Gizmos,
    ragdolls: Query<&Ragdoll>,
    bodies: Query<(&Position, &Rotation, &Collider)>,
    proxies: Query<&Position, With<ProxyOf>>,
) {
    if !config.gizmos {
        return;
    }
    let cyan = Color::srgb(0.2, 0.9, 0.9);
    for ragdoll in &ragdolls {
        for (_, body) in ragdoll.bodies.iter() {
            let Some((position, rotation, collider)) = body.and_then(|body| bodies.get(body).ok()) else { continue };
            let Some(capsule) = collider.shape().as_capsule() else { continue };
            let (a, b) = (capsule.segment.a, capsule.segment.b);
            gizmos.line(position.0 + rotation.0 * Vec3::new(a.x, a.y, a.z), position.0 + rotation.0 * Vec3::new(b.x, b.y, b.z), cyan);
        }
    }
    for position in &proxies {
        let half = Vec3::Y * PROXY_LENGTH * 0.5;
        gizmos.line(position.0 - half, position.0 + half, Color::srgb(1.0, 0.7, 0.2));
        gizmos.circle(Isometry3d::new(position.0 - half, Quat::from_rotation_x(PI * 0.5)), PROXY_RADIUS, Color::srgb(1.0, 0.7, 0.2));
    }
}

#[derive(Component)]
struct Hud;

fn spawn_hud(mut commands: Commands) {
    commands.spawn((
        Text::new(""),
        TextFont { font_size: FontSize::Px(14.0), ..default() },
        Node { position_type: PositionType::Absolute, top: px(10), left: px(10), ..default() },
        Hud,
    ));
}

fn update_hud(
    config: Res<Config>,
    diagnostics: Res<bevy::diagnostic::DiagnosticsStore>,
    characters: Query<(Option<&Ragdoll>, Option<&Proxy>), With<WalkerState>>,
    mut hud: Query<&mut Text, With<Hud>>,
) {
    let (mut ragdolls, mut falling, mut capsules) = (0, 0, 0);
    for (ragdoll, proxy) in &characters {
        ragdolls += ragdoll.is_some() as usize;
        falling += ragdoll.is_some_and(|r| r.is_falling()) as usize;
        capsules += proxy.is_some() as usize;
    }
    let fps = diagnostics.get(&FrameTimeDiagnosticsPlugin::FPS).and_then(|d| d.smoothed()).unwrap_or(0.0);
    for mut text in &mut hud {
        text.0 = format!(
            "{fps:.0} fps | physics {:?} (ragdoll within {:.0} m) | {} characters: {ragdolls} ragdoll ({falling} falling), {capsules} capsule\n\
             WASD move, Q/E down/up, hold right mouse (or M) to look, Shift run, wheel speed",
            config.physics,
            config.ragdoll_distance,
            characters.iter().count(),
        );
    }
}

/// `--trace-feet`: the first character's ankles (world) and heading each
/// frame, as rendered: `FEET t yaw lx ly lz rx ry rz hips_y`.
fn trace_feet(config: Res<Config>, time: Res<Time>, characters: Query<(&HumanoidSkeleton, &WalkerState)>, world: Query<&GlobalTransform>) {
    if !config.trace_feet {
        return;
    }
    let Some((skeleton, state)) = characters.iter().next() else { return };
    let (Ok(left), Ok(right), Ok(hips)) =
        (world.get(skeleton.entity(Bone::LeftFoot)), world.get(skeleton.entity(Bone::RightFoot)), world.get(skeleton.entity(Bone::Hips)))
    else {
        return;
    };
    let (l, r) = (left.translation(), right.translation());
    println!(
        "FEET {:.4} {:.4} {:.4} {:.4} {:.4} {:.4} {:.4} {:.4} {:.4}",
        time.elapsed_secs(),
        state.facing.yaw,
        l.x,
        l.y,
        l.z,
        r.x,
        r.y,
        r.z,
        hips.translation().y
    );
}

/// `--bench SECS`: after 3 s to load and settle, every frame's wall time
/// for SECS seconds; then `BENCH` with the count, p50 and p99, and exit.
fn bench(config: Res<Config>, time: Res<Time<Real>>, mut frames: Local<Vec<f32>>, characters: Query<(Option<&Ragdoll>, Option<&Proxy>), With<WalkerState>>) {
    let Some(seconds) = config.bench else { return };
    let now = time.elapsed_secs();
    if now < 3.0 {
        return;
    }
    frames.push(time.delta_secs() * 1e3);
    if now >= 3.0 + seconds {
        frames.sort_by(f32::total_cmp);
        let at = |q: f32| frames[((frames.len() - 1) as f32 * q) as usize];
        let ragdolls = characters.iter().filter(|(r, _)| r.is_some()).count();
        let capsules = characters.iter().filter(|(_, p)| p.is_some()).count();
        println!(
            "BENCH physics {:?} characters {} ({ragdolls} ragdoll, {capsules} capsule) props {}: {} frames, p50 {:.2} ms, p99 {:.2} ms",
            config.physics,
            config.characters,
            config.props,
            frames.len(),
            at(0.5),
            at(0.99)
        );
        std::process::exit(0);
    }
}

/// `--shot PATH --at-frame N`: a screenshot at frame N, then exit.
fn auto_shot(config: Res<Config>, frame: Res<FrameCount>, mut commands: Commands, mut fired: Local<bool>) {
    let Some((path, at)) = config.shot.clone() else { return };
    if !*fired && frame.0 >= at {
        *fired = true;
        commands.spawn(Screenshot::primary_window()).observe(save_to_disk(path));
    }
    if *fired && frame.0 >= at + 60 {
        std::process::exit(0);
    }
}
