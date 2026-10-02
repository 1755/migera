//! An animated character in a physical world.
//!
//! A 50 × 50 m room: a floor and four 3 m walls, all static colliders.
//! Characters walk straight, and when one nears a wall it turns away along
//! the wall's reflection of its heading (the mirrored ray), plus a small
//! random jitter so no two bounces repeat. Cubes, spheres and capsules
//! drop from 5 m around the centre and lie where they land, for the
//! characters to walk into.
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
//! `--camera X,Y,Z` (where the camera starts, looking at the centre).

use std::f32::consts::PI;

use avian3d::prelude::*;
use bevy::camera_controller::free_camera::{FreeCamera, FreeCameraPlugin};
use bevy::diagnostic::{FrameCount, FrameTimeDiagnosticsPlugin};
use bevy::prelude::*;
use bevy::render::view::window::screenshot::{save_to_disk, Screenshot};

use migera::character::anim::asset::AnimAssetPlugin;
use migera::character::anim::plugin::AnimFootIk;
use migera::character::anim::ragdoll_plugin::sole_blocks;
use migera::character::anim::{
    despawn_ragdoll, spawn_gltf_humanoid, spawn_ragdoll, AnimPlugin, AnimRagdollPlugin, HumanoidPlugin, Ragdoll, RagdollSpawnConfig,
    Steer, Walker, WalkerPlugin, WalkerSet, WalkerState,
};
use migera::character::{Bone, HumanoidSkeleton};

/// Half the room's side, metres.
const HALF_ROOM: f32 = 25.0;
const WALL_HEIGHT: f32 = 3.0;
const WALL_THICKNESS: f32 = 0.5;
/// The walls' own collision layer, which the bounce's ray looks for alone.
const WALL_LAYER: LayerMask = LayerMask(1 << 1);
/// How far ahead a walker looks for a wall, metres: at 1.2 m/s and a turn
/// of [`BOUNCE_TURN_RATE`] it turns within ~1 m.
const LOOK_AHEAD: f32 = 2.0;
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
    shot: Option<(String, u32)>,
    camera: Vec3,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum PhysicsMode {
    Lod,
    Ragdoll,
    Kinematic,
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
            shot: None,
            camera: Vec3::new(0.0, 9.0, 22.0),
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
                "--physics" => {
                    config.physics = match value().as_str() {
                        "ragdoll" => PhysicsMode::Ragdoll,
                        "kinematic" => PhysicsMode::Kinematic,
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
                "--camera" => {
                    let parts: Vec<f32> = value().split(',').filter_map(|p| p.trim().parse().ok()).collect();
                    if let [x, y, z] = parts[..] {
                        config.camera = Vec3::new(x, y, z);
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
}

/// The kinematic capsule standing in for a far character's body.
#[derive(Component)]
struct ProxyOf(Entity);

/// A character's kinematic capsule, if it has one now.
#[derive(Component)]
struct Proxy(Entity);

/// The capsule's radius and its straight length, metres: a body's width and
/// a 1.7 m character's height.
const PROXY_RADIUS: f32 = 0.25;
const PROXY_LENGTH: f32 = 1.2;

fn main() {
    let assets = std::env::current_dir().expect("cwd").join("assets").to_string_lossy().into_owned();
    let config = Config::from_args();
    App::new()
        .add_plugins(
            DefaultPlugins
                .set(WindowPlugin { primary_window: Some(Window { title: "migera physics character playground".into(), ..default() }), ..default() })
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
        .add_plugins((AnimPlugin, AnimAssetPlugin, HumanoidPlugin, WalkerPlugin, AnimRagdollPlugin, FreeCameraPlugin))
        .insert_resource(config)
        .add_systems(Startup, (spawn_room, spawn_props, spawn_characters, spawn_camera_and_light, spawn_hud))
        .add_systems(Update, bounce_off_walls.before(WalkerSet::Drive))
        .add_systems(Update, (choose_physics, draw_bodies, update_hud, auto_shot))
        .add_systems(FixedUpdate, carry_proxies)
        .run();
}

fn spawn_camera_and_light(mut commands: Commands, config: Res<Config>) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_translation(config.camera).looking_at(Vec3::new(0.0, 1.0, 0.0), Vec3::Y),
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
    let mut rng = fastrand::Rng::with_seed(config.seed);
    for i in 0..config.props {
        let angle = rng.f32() * PI * 2.0;
        // Around the centre, clear of the first character standing there.
        let reach = 1.5 + rng.f32() * 6.0;
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
        // `puppet_base.gltf` faces +Z; this crate's forward is -Z.
        let root = spawn_gltf_humanoid(&mut commands, &asset_server, "models/puppet_base.gltf", PI, Transform::from_translation(at).with_rotation(Quat::from_rotation_y(yaw)));
        commands.entity(root).insert((Walker { speed: config.speed, steer: Steer::Toward { yaw, rate: BOUNCE_TURN_RATE }, ..default() }, WallBouncer::default()));
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
        let Ok(direction) = Dir3::new(ahead) else { continue };
        let origin = transform.translation + Vec3::Y * 1.0;
        let Some(hit) = spatial.cast_ray(origin, direction, LOOK_AHEAD, true, &walls) else { continue };
        let normal = Vec3::new(hit.normal.x, 0.0, hit.normal.z).normalize_or_zero();
        if normal == Vec3::ZERO {
            continue;
        }
        let mirrored = ahead - normal * (2.0 * ahead.dot(normal));
        let jitter = (rng.f32() * 2.0 - 1.0) * config.jitter;
        let target = yaw_of(mirrored) + jitter;
        walker.steer = Steer::Toward { yaw: target, rate: BOUNCE_TURN_RATE };
        bouncer.turning_to = Some(target);
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
    characters: Query<(Entity, &HumanoidSkeleton, &AnimFootIk, &Transform, Option<&Ragdoll>, Option<&Proxy>), With<WalkerState>>,
    global_transforms: Query<&GlobalTransform>,
) {
    let Ok(camera) = cameras.single() else { return };
    for (entity, skeleton, foot_ik, transform, ragdoll, proxy) in &characters {
        // Not until the foot IK has measured the live rig (the ragdoll's
        // soles are built from it) and the bones have been placed.
        let Some(rig) = foot_ik.rig.as_ref() else { continue };
        if global_transforms.get(skeleton.entity(Bone::Hips)).is_ok_and(|hips| hips.translation() == Vec3::ZERO) {
            continue;
        }
        let distance = camera.translation().distance(transform.translation);
        let near = match config.physics {
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
            }
            _ => {}
        }
    }
}

/// Where a character's capsule stands: on the floor under it.
fn capsule_centre(feet: Vec3) -> Vec3 {
    feet + Vec3::Y * (PROXY_LENGTH * 0.5 + PROXY_RADIUS)
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
