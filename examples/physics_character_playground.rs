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
//! Around the room, static terrain to climb and fall from, free-standing
//! (open on every side, to step off):
//! - in the north, five ramps of 15°, 25°, 35°, 45° and 50°, rising to
//!   platforms 1.0, 1.5, 2.0, 2.5 and 3.0 m high;
//! - in the east, a stair to a landing 5 m up: 30 risers of 167 mm on
//!   300 mm treads (within the IBC's 178 mm riser and 279 mm tread limits,
//!   2R + T = 0.63 m on Blondel's 0.63-0.65);
//! - in the south, fences 2 m wide and 1 m thick, 0.1, 0.2, 0.4, 0.8, 1.0
//!   and 1.6 m high.
//!
//! A walker climbs a rise of up to [`MAX_RISE`] and a slope of up to
//! [`MAX_SLOPE_DEGREES`], and turns from anything steeper as from a wall.
//! It does not turn from a drop: one stepping off a ledge falls (with a
//! ragdoll) and gets up below.
//!
//! Each walker heads for a goal: up a ramp or the stair to its top, across
//! a fence, or to a random place on the floor. A route goes through a
//! point lined up in front of the structure, so the walker meets it head
//! on. A turn from terrain or another walker goes first, and the walker
//! walks its new way for a moment before turning back to its goal; a goal
//! not reached in [`GOAL_SECONDS`] (the 45° ramp, the tall fences) is
//! given up for another. Arrived on top, it picks its next goal and walks
//! off the edge or back down.
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
//! Flags: `--characters N` (1), `--props N` (60), `--speed M_PER_S` (1.2),
//! `--jitter DEGREES` (15), `--ragdoll-distance M` (10), `--seed N`,
//! `--physics lod|ragdoll|kinematic`, `--gizmos on`,
//! `--shot PATH --at-frame N` (a screenshot, then exit),
//! `--camera X,Y,Z` and `--look X,Y,Z` (where the camera starts, and what
//! at), `--avoid off`, `--plank H` (a feet-on-props test: one plank across
//! the first character's path, no props), `--trace-feet` (prints the first
//! character's ankles and heading every frame), `--start X,Z,YAW` (where
//! the first character starts, and its heading in degrees, 0 along -Z: to
//! walk it at one structure; that character then has no goals),
//! `--goals off` (walk straight and bounce, no goals),
//! `--sit-at-table N` (the first character walks to the dining table's
//! chair N and sits), `--foot-obstacles off`, `--ground flat`,
//! `--bench SECS` (frame times, then exit), `--step-seconds S` (a fixed
//! step per frame, for offscreen runs).

use std::f32::consts::PI;

use avian3d::prelude::*;
use bevy::camera_controller::free_camera::{FreeCamera, FreeCameraPlugin};
use bevy::diagnostic::{FrameCount, FrameTimeDiagnosticsPlugin};
use bevy::prelude::*;
use bevy::render::view::window::screenshot::{save_to_disk, Screenshot};

use migera::character::anim::asset::AnimAssetPlugin;
use migera::character::anim::physics_ground::{PhysicsGround, PhysicsGroundPlugin};
use migera::character::anim::physics_obstacles::{Obstacle, PhysicsObstacles, PhysicsObstaclesPlugin};
use migera::character::anim::plugin::AnimFootIk;
use migera::character::anim::ragdoll_plugin::sole_blocks;
use migera::character::anim::{
    despawn_ragdoll, spawn_gltf_humanoid, spawn_ragdoll, AnimPlugin, AnimRagdollPlugin, HumanoidPlugin, Ragdoll, RagdollSpawnConfig,
    Steer, Walker, WalkerPlugin, WalkerSet, WalkerState,
};
use migera::character::anim::{approach, sitting};
use migera::character::{Bone, HumanoidSkeleton};

/// Half the room's side, metres.
const HALF_ROOM: f32 = 12.5;
const WALL_HEIGHT: f32 = 3.0;
const WALL_THICKNESS: f32 = 0.5;
/// The static terrain's own collision layer (floor, walls, ramps, stairs,
/// fences), which the steering's rays look for alone, never a prop.
const TERRAIN_LAYER: LayerMask = LayerMask(1 << 1);
/// How far ahead a walker looks for a wall, metres: at 1.2 m/s and a turn
/// of [`BOUNCE_TURN_RATE`] it turns within ~1 m.
const LOOK_AHEAD: f32 = 2.0;
/// The spacing of the ground samples along the way ahead, metres: under a
/// tread (0.3 m), so no two risers fall between two samples.
const PROBE_STEP: f32 = 0.2;
/// The tallest rise between two samples a walker steps up, metres: a stair
/// riser and the 0.2 m fence, not the 0.4 m one. Below the feet's own
/// limit (`PhysicsGround::max_step`, 0.35), so a foot stands on whatever
/// the walker walks onto.
const MAX_RISE: f32 = 0.3;
/// The steepest slope a walker walks up or down, degrees: the 35° ramp,
/// not the 45°.
const MAX_SLOPE_DEGREES: f32 = 40.0;
/// How far the ground under the soles may fall below the ground the body
/// stands on before a ragdolled walker falls, metres: one foot over a 1 m
/// ledge (half its drop), never a stair's riser or a slope's step.
const LEDGE_DROP: f32 = 0.45;
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

/// The ramps in the north: each slope's angle (degrees) and the height it
/// rises to (metres), 2 m wide, 4 m apart from x = -9, rising northward to
/// a 1.5 m deep platform. Free-standing, 2.5 m clear of the wall: a
/// walker can step off a platform's side or back, fall and get up, where
/// against the wall one fell into a 1 m gap it could not turn out of.
const RAMPS: [(f32, f32); 5] = [(15.0, 1.0), (25.0, 1.5), (35.0, 2.0), (45.0, 2.5), (50.0, 3.0)];
const RAMP_WIDTH: f32 = 2.0;
const RAMP_SPACING: f32 = 4.0;
const RAMP_FIRST_X: f32 = -9.0;
const PLATFORM_DEPTH: f32 = 1.5;
/// Where the platforms' backs stand, z.
const PLATFORM_BACK: f32 = -10.0;
/// The stair in the east, rising northward, free-standing 2 m from the
/// wall: its height, risers, tread and width, metres, the landing at the
/// top, and where the landing's back stands (x of its middle, z).
const STAIR_HEIGHT: f32 = 5.0;
const STAIR_RISERS: usize = 30;
const STAIR_TREAD: f32 = 0.3;
const STAIR_WIDTH: f32 = 2.0;
const LANDING_DEPTH: f32 = 1.5;
const STAIR_X: f32 = 9.5;
const LANDING_BACK: f32 = -3.2;
/// The fences in the south: their heights, metres; each 2 m wide (x) and
/// 1 m thick (z), 3 m apart from x = -9.5, centred on z = 9.5.
const FENCES: [f32; 6] = [0.1, 0.2, 0.4, 0.8, 1.0, 1.6];
const FENCE_WIDTH: f32 = 2.0;
const FENCE_THICKNESS: f32 = 1.0;
const FENCE_SPACING: f32 = 3.0;
const FENCE_FIRST_X: f32 = -9.5;
const FENCE_Z: f32 = 9.5;

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
    /// Walkers head for goals (`--goals off`: they walk straight and only
    /// bounce, as before).
    goals: bool,
    /// `--sit-at-table N`: the first character walks to the dining table's
    /// chair N (0-3) and sits on it, its feet kept out of the chair's and
    /// the table's legs by the physics world (`physics_obstacles`).
    sit_at_table: Option<usize>,
    /// `--plank H`: a test of feet on props. No props; one H m plank lying
    /// across the first character's path, which starts facing it.
    plank: Option<f32>,
    trace_feet: bool,
    /// `--bench SECS`: vsync off, 3 s to settle, then frame times for SECS
    /// seconds, printed (p50, p99), and exit.
    bench: Option<f32>,
    flat_ground: bool,
    /// `--foot-obstacles off`: the feet ignore the furniture, for an A/B.
    foot_obstacles: bool,
    /// `--step-seconds S`: every frame advances the clock exactly S, for
    /// runs on a software renderer (motion as at full speed, slower to
    /// watch). Never for timings.
    step_seconds: Option<f64>,
    shot: Option<(String, u32)>,
    camera: Vec3,
    /// What the camera starts looking at (`--look X,Y,Z`).
    look: Vec3,
    /// `--start X,Z,YAW`: the first character's place and heading (radians).
    start: Option<(Vec2, f32)>,
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
            props: 60,
            speed: 1.2,
            jitter: 15f32.to_radians(),
            ragdoll_distance: 10.0,
            seed: 7,
            physics: PhysicsMode::Lod,
            gizmos: false,
            avoid: true,
            goals: true,
            sit_at_table: None,
            plank: None,
            trace_feet: false,
            bench: None,
            flat_ground: false,
            foot_obstacles: true,
            step_seconds: None,
            shot: None,
            camera: Vec3::new(0.0, 9.0, 22.0),
            look: Vec3::new(0.0, 1.0, 0.0),
            start: None,
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
                "--goals" => config.goals = value() != "off",
                "--sit-at-table" => config.sit_at_table = value().parse().ok(),
                "--plank" => config.plank = value().parse().ok(),
                "--trace-feet" => config.trace_feet = true,
                "--bench" => config.bench = value().parse().ok(),
                "--ground" => config.flat_ground = value() == "flat",
                "--foot-obstacles" => config.foot_obstacles = value() != "off",
                "--step-seconds" => config.step_seconds = value().parse().ok(),
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
                "--start" => {
                    let parts: Vec<f32> = value().split(',').filter_map(|p| p.trim().parse().ok()).collect();
                    if let [x, z, yaw] = parts[..] {
                        config.start = Some((Vec2::new(x, z), yaw.to_radians()));
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

/// A walker heading for a goal: the points of its route still ahead, the
/// first next.
#[derive(Component, Default)]
struct Seeker {
    name: String,
    route: Vec<Vec2>,
    /// How long it has been after this goal, seconds.
    since: f32,
    /// How long it still walks its own way after a turn from terrain or
    /// another walker, seconds, before turning back to its goal: turned
    /// back at once, it walked straight into what it had turned from.
    detour: f32,
}

/// How long a walker pursues a goal before giving it up, seconds.
const GOAL_SECONDS: f32 = 25.0;
/// How long a walker walks its new way after a turn before heading for its
/// goal again, seconds.
const DETOUR_SECONDS: f32 = 1.5;
/// How near a route's point counts as reached, metres.
const ARRIVED: f32 = 0.6;
/// How far in front of a structure a route's entry point lies, metres: a
/// walker turning at 2.5 rad/s (a 0.5 m radius at 1.2 m/s) lines up in it.
const ENTRY: f32 = 2.0;

/// The steering's own time (terrain probes, avoidance and goals), summed
/// over the frames `--bench` measures, and the walkers it served.
#[derive(Resource, Default)]
struct SteerCost {
    time: std::time::Duration,
    walker_frames: u64,
}

/// Adds a steering system's time since `.1`, and `.2` walkers' frames, to
/// [`SteerCost`] when dropped: at the system's end, whichever way it ends.
struct TimedBy<'a>(&'a mut SteerCost, std::time::Instant, u64);

impl Drop for TimedBy<'_> {
    fn drop(&mut self) {
        self.0.time += self.1.elapsed();
        self.0.walker_frames += self.2;
    }
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
    let mut app = App::new();
    if let Some(seconds) = config.step_seconds {
        app.insert_resource(bevy::time::TimeUpdateStrategy::ManualDuration(std::time::Duration::from_secs_f64(seconds)));
    }
    app
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
        .add_plugins((AnimPlugin, AnimAssetPlugin, HumanoidPlugin, WalkerPlugin, AnimRagdollPlugin, PhysicsGroundPlugin, PhysicsObstaclesPlugin, FreeCameraPlugin))
        .insert_resource(config)
        .init_resource::<SteerCost>()
        .add_systems(Startup, (spawn_room, spawn_props, spawn_characters, spawn_camera_and_light, spawn_hud))
        .add_systems(Update, (turn_from_terrain, avoid_each_other, seek_goals).chain().before(WalkerSet::Drive))
        .add_systems(Update, (choose_physics, fall_off_ledges, draw_bodies, update_hud, auto_shot, bench))
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

/// One static solid of the room or its terrain.
struct Solid {
    shape: Shape,
    centre: Vec3,
    rotation: Quat,
    color: Color,
    /// Feet keep out of it (`Obstacle`): furniture, not terrain.
    furniture: bool,
}

enum Shape {
    /// A box of this size.
    Cuboid(Vec3),
    /// A ramp's solid wedge, rising along its local +X from its origin over
    /// `run` to `height`, `width` wide across its local Z (centred).
    Wedge { run: f32, height: f32, width: f32 },
}

impl Solid {
    /// An upright box standing on the floor, its footprint centred on `x`, `z`.
    fn standing(size: Vec3, x: f32, z: f32, color: Color) -> Self {
        Self { shape: Shape::Cuboid(size), centre: Vec3::new(x, size.y * 0.5, z), rotation: Quat::IDENTITY, color, furniture: false }
    }

    /// A box of furniture, `size`, its middle at `centre` relative to a piece
    /// at `at` turned `turn` about the vertical.
    fn furniture(size: Vec3, centre: Vec3, at: Vec3, turn: Quat, color: Color) -> Self {
        Self { shape: Shape::Cuboid(size), centre: at + turn * centre, rotation: turn, color, furniture: true }
    }

    fn collider_and_mesh(&self) -> (Collider, Mesh) {
        match self.shape {
            Shape::Cuboid(size) => (Collider::cuboid(size.x, size.y, size.z), Cuboid::new(size.x, size.y, size.z).into()),
            Shape::Wedge { run, height, width } => {
                let triangle = Triangle2d::new(Vec2::ZERO, Vec2::new(run, 0.0), Vec2::new(run, height));
                let half = width * 0.5;
                let corners = triangle.vertices.iter().flat_map(|v| [v.extend(-half), v.extend(half)]).collect();
                (Collider::convex_hull(corners).expect("a wedge is a convex solid"), Extrusion::new(triangle, width).into())
            }
        }
    }
}

/// The room and its terrain: every static box, and the footprints of the
/// structures (`x`, `z`), which no character is spawned on.
fn layout() -> (Vec<Solid>, Vec<Rect>) {
    let side = HALF_ROOM * 2.0;
    let mut solids = Vec::new();
    let mut footprints = Vec::new();
    // The floor, its top at y = 0, where the walkers' flat ground is.
    solids.push(Solid { shape: Shape::Cuboid(Vec3::new(side, 0.2, side)), centre: Vec3::new(0.0, -0.1, 0.0), rotation: Quat::IDENTITY, color: Color::srgb(0.45, 0.47, 0.45), furniture: false });
    let wall = Color::srgb(0.62, 0.58, 0.52);
    let long = side + WALL_THICKNESS * 2.0;
    let out = HALF_ROOM + WALL_THICKNESS * 0.5;
    solids.push(Solid::standing(Vec3::new(long, WALL_HEIGHT, WALL_THICKNESS), 0.0, -out, wall));
    solids.push(Solid::standing(Vec3::new(long, WALL_HEIGHT, WALL_THICKNESS), 0.0, out, wall));
    solids.push(Solid::standing(Vec3::new(WALL_THICKNESS, WALL_HEIGHT, side), -out, 0.0, wall));
    solids.push(Solid::standing(Vec3::new(WALL_THICKNESS, WALL_HEIGHT, side), out, 0.0, wall));

    // Ramps: a solid wedge from the floor up to a platform. A tilted slab
    // left a hollow under it, where the steering's ray across, just above
    // the floor, met the slab's underside and turned a walker into it.
    let platform_front = PLATFORM_BACK + PLATFORM_DEPTH;
    for (i, (degrees, height)) in RAMPS.into_iter().enumerate() {
        let x = RAMP_FIRST_X + i as f32 * RAMP_SPACING;
        let color = Color::hsl(200.0 - i as f32 * 40.0, 0.45, 0.55);
        solids.push(Solid::standing(Vec3::new(RAMP_WIDTH, height, PLATFORM_DEPTH), x, PLATFORM_BACK + PLATFORM_DEPTH * 0.5, color));
        let run = height / degrees.to_radians().tan();
        // The wedge's +X turned to the room's -Z, its origin at the foot.
        solids.push(Solid {
            shape: Shape::Wedge { run, height, width: RAMP_WIDTH },
            centre: Vec3::new(x, 0.0, platform_front + run),
            rotation: Quat::from_rotation_y(PI * 0.5),
            color,
            furniture: false,
        });
        footprints.push(Rect::new(x - RAMP_WIDTH * 0.5, PLATFORM_BACK, x + RAMP_WIDTH * 0.5, platform_front + run));
    }

    // The stair: solid steps rising northward to the landing.
    let riser = STAIR_HEIGHT / STAIR_RISERS as f32;
    let landing_front = LANDING_BACK + LANDING_DEPTH;
    let stair = Color::srgb(0.5, 0.5, 0.56);
    for k in 1..STAIR_RISERS {
        let front = landing_front + (STAIR_RISERS - k) as f32 * STAIR_TREAD;
        solids.push(Solid::standing(Vec3::new(STAIR_WIDTH, k as f32 * riser, STAIR_TREAD), STAIR_X, front - STAIR_TREAD * 0.5, stair));
    }
    solids.push(Solid::standing(Vec3::new(STAIR_WIDTH, STAIR_HEIGHT, LANDING_DEPTH), STAIR_X, LANDING_BACK + LANDING_DEPTH * 0.5, stair));
    let stair_foot = landing_front + (STAIR_RISERS - 1) as f32 * STAIR_TREAD;
    footprints.push(Rect::new(STAIR_X - STAIR_WIDTH * 0.5, LANDING_BACK, STAIR_X + STAIR_WIDTH * 0.5, stair_foot));

    // Fences.
    let fence = Color::srgb(0.72, 0.48, 0.28);
    for (i, height) in FENCES.into_iter().enumerate() {
        let x = FENCE_FIRST_X + i as f32 * FENCE_SPACING;
        solids.push(Solid::standing(Vec3::new(FENCE_WIDTH, height, FENCE_THICKNESS), x, FENCE_Z, fence));
        footprints.push(Rect::new(x - FENCE_WIDTH * 0.5, FENCE_Z - FENCE_THICKNESS * 0.5, x + FENCE_WIDTH * 0.5, FENCE_Z + FENCE_THICKNESS * 0.5));
    }

    // A dining table and four chairs, every leg its own collider, so feet
    // keep out of the legs themselves (`Obstacle`) rather than a block.
    let wood = Color::srgb(0.45, 0.30, 0.18);
    let table = TABLE_AT;
    let (length, width, height, leg) = (1.2, TABLE_WIDTH, 0.75, 0.06);
    solids.push(Solid::furniture(Vec3::new(length, 0.04, width), Vec3::new(0.0, height - 0.02, 0.0), table, Quat::IDENTITY, wood));
    for (x, z) in [(-1.0, -1.0), (1.0, -1.0), (-1.0, 1.0), (1.0, 1.0)] {
        let centre = Vec3::new(x * (length * 0.5 - 0.05), (height - 0.04) * 0.5, z * (width * 0.5 - 0.05));
        solids.push(Solid::furniture(Vec3::new(leg, height - 0.04, leg), centre, table, Quat::IDENTITY, wood));
    }
    for (at, turn) in dining_chairs() {
        for piece in chair_pieces() {
            solids.push(Solid::furniture(piece.0, piece.1, at, turn, wood));
        }
    }
    footprints.push(Rect::new(table.x - 1.3, table.z - 1.6, table.x + 1.3, table.z + 1.6));
    (solids, footprints)
}

/// Where the dining table stands, and its depth across.
const TABLE_AT: Vec3 = Vec3::new(-3.0, 0.0, 3.0);
const TABLE_WIDTH: f32 = 0.8;

/// The chairs' middles and turns: two along each long side of the table,
/// facing it (local -Z toward it), pulled out to sit on. Tucked in 0.35 m
/// from its edge, the spot to stand on to sit was inside the table; at
/// 0.65 m, where the turn onto it ends was 12 cm from the table, no room to
/// come at it.
fn dining_chairs() -> Vec<(Vec3, Quat)> {
    [(-0.3, 1.0), (0.3, 1.0), (-0.3, -1.0), (0.3, -1.0)]
        .into_iter()
        .map(|(x, side): (f32, f32)| {
            let at = TABLE_AT + Vec3::new(x, 0.0, side * (TABLE_WIDTH * 0.5 + 0.75));
            (at, Quat::from_rotation_y(if side > 0.0 { 0.0 } else { PI }))
        })
        .collect()
}

/// Chair `index` of [`dining_chairs`] as the walker sits on it: where its
/// seated hips go (0.08 behind the seat's middle, as the gallery's chair),
/// facing away from its backrest.
fn dining_chair(index: usize) -> approach::Chair {
    let (at, turn) = dining_chairs()[index % 4];
    let forward = turn * Vec3::NEG_Z;
    approach::Chair::standard(at - forward * 0.08, forward)
}

/// A standard chair's boxes (size, middle), facing local -Z: a 0.46 by 0.44
/// seat 0.45 high, a backrest, four 3.5 cm legs (the gallery's chair).
fn chair_pieces() -> Vec<(Vec3, Vec3)> {
    let (depth, width, thick, height) = (0.44, 0.46, 0.04, 0.45);
    let mut pieces = vec![(Vec3::new(width, thick, depth), Vec3::new(0.0, height - thick * 0.5, 0.0))];
    pieces.push((Vec3::new(width, 0.42, thick), Vec3::new(0.0, height + 0.25, depth * 0.5)));
    for (x, z) in [(-1.0, -1.0), (1.0, -1.0), (-1.0, 1.0), (1.0, 1.0)] {
        pieces.push((Vec3::new(0.035, height - thick, 0.035), Vec3::new(x * (width * 0.5 - 0.03), (height - thick) * 0.5, z * (depth * 0.5 - 0.03))));
    }
    pieces
}

/// The room and its terrain, static, drawn and solid.
fn spawn_room(mut commands: Commands, mut meshes: ResMut<Assets<Mesh>>, mut materials: ResMut<Assets<StandardMaterial>>) {
    for solid in layout().0 {
        let (collider, mesh) = solid.collider_and_mesh();
        let mut entity = commands.spawn((
            RigidBody::Static,
            collider,
            Friction::new(1.0),
            // Its own layer as well as the default, so the steering's rays
            // find the terrain alone, never a prop lying in the way.
            CollisionLayers::new(LayerMask::DEFAULT | TERRAIN_LAYER, LayerMask::ALL),
            Mesh3d(meshes.add(mesh)),
            MeshMaterial3d(materials.add(solid.color)),
            Transform::from_translation(solid.centre).with_rotation(solid.rotation),
        ));
        if solid.furniture {
            entity.insert(Obstacle);
        }
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

/// The first character at the centre (or `--start`), the rest scattered
/// over the floor clear of the terrain; each walking straight on a random
/// heading.
fn spawn_characters(mut commands: Commands, asset_server: Res<AssetServer>, config: Res<Config>) {
    let mut rng = fastrand::Rng::with_seed(config.seed ^ 0x9e37);
    for i in 0..config.characters {
        let mut at = Vec3::ZERO;
        if i > 0 {
            let point = free_floor_point(&mut rng);
            at = Vec3::new(point.x, 0.0, point.y);
        }
        let yaw = rng.f32() * PI * 2.0 - PI;
        // Facing the plank (along -Z, yaw zero), for its test.
        let mut yaw = if i == 0 && config.plank.is_some() { 0.0 } else { yaw };
        if i == 0
            && let Some((start, heading)) = config.start
        {
            at = Vec3::new(start.x, 0.0, start.y);
            yaw = heading;
        }
        // `puppet_base.gltf` faces +Z; this crate's forward is -Z.
        let root = spawn_gltf_humanoid(&mut commands, &asset_server, "models/puppet_base.gltf", PI, Transform::from_translation(at).with_rotation(Quat::from_rotation_y(yaw)));
        commands.entity(root).insert((Walker { speed: config.speed, steer: Steer::Toward { yaw, rate: BOUNCE_TURN_RATE }, ..default() }, WallBouncer::default()));
        // Feet on the physics world: on a prop low enough to step on
        // (`--ground flat`: the floor alone, for the bench).
        if !config.flat_ground {
            commands.entity(root).insert(PhysicsGround::default());
            // And kept out of the furniture's legs.
            if config.foot_obstacles {
                commands.entity(root).insert(PhysicsObstacles::default());
            }
        }
        // A character set up for a test (`--start`, `--plank`) keeps its
        // heading.
        let under_test = i == 0 && (config.start.is_some() || config.plank.is_some() || config.sit_at_table.is_some());
        if config.goals && !under_test {
            commands.entity(root).insert(Seeker::default());
        }
        // To the table, to sit: its own approach steers it (`approach`).
        if i == 0
            && let Some(index) = config.sit_at_table
        {
            commands.entity(root).insert(Walker {
                speed: config.speed,
                sit: Some(sitting::Sitting::Chair(sitting::ChairPose::Upright)),
                chair: Some(dining_chair(index)),
                ..default()
            });
        }
    }
}

/// A random point on the floor a metre clear of every structure.
fn free_floor_point(rng: &mut fastrand::Rng) -> Vec2 {
    let footprints: Vec<Rect> = layout().1.into_iter().map(|rect| rect.inflate(1.0)).collect();
    let mut point = Vec2::ZERO;
    for _ in 0..100 {
        point = Vec2::new((rng.f32() * 2.0 - 1.0) * (HALF_ROOM - 1.5), (rng.f32() * 2.0 - 1.0) * (HALF_ROOM - 1.5));
        if !footprints.iter().any(|rect| rect.contains(point)) {
            break;
        }
    }
    point
}

/// Every structure's route, named: a point lined up [`ENTRY`] in front of
/// it, then its top (a ramp's platform, the stair's landing) or just past
/// it (a fence, crossed southward: northward, its far side's point would
/// lie within the wall probe's reach and never be reached).
fn routes() -> Vec<(String, Vec<Vec2>)> {
    let mut routes = Vec::new();
    let platform_front = PLATFORM_BACK + PLATFORM_DEPTH;
    for (i, (degrees, height)) in RAMPS.into_iter().enumerate() {
        let x = RAMP_FIRST_X + i as f32 * RAMP_SPACING;
        let foot = platform_front + height / degrees.to_radians().tan();
        routes.push((format!("{degrees}° ramp"), vec![Vec2::new(x, foot + ENTRY), Vec2::new(x, PLATFORM_BACK + PLATFORM_DEPTH * 0.5)]));
    }
    let stair_foot = LANDING_BACK + LANDING_DEPTH + (STAIR_RISERS - 1) as f32 * STAIR_TREAD;
    routes.push(("stair".into(), vec![Vec2::new(STAIR_X, stair_foot + ENTRY), Vec2::new(STAIR_X, LANDING_BACK + LANDING_DEPTH * 0.5)]));
    for (i, height) in FENCES.into_iter().enumerate() {
        let x = FENCE_FIRST_X + i as f32 * FENCE_SPACING;
        let near = FENCE_Z - FENCE_THICKNESS * 0.5;
        routes.push((format!("{height} m fence"), vec![Vec2::new(x, near - ENTRY), Vec2::new(x, near + FENCE_THICKNESS + 0.6)]));
    }
    routes
}

/// Steers each [`Seeker`] for the next point of its route, unless a turn
/// from terrain or another walker is under way or just ended
/// ([`DETOUR_SECONDS`]). A route done or given up ([`GOAL_SECONDS`]), it
/// picks another: a structure's route seven times in ten, else a random
/// place on the floor.
#[allow(clippy::type_complexity)]
fn seek_goals(
    config: Res<Config>,
    time: Res<Time>,
    mut cost: ResMut<SteerCost>,
    mut walkers: Query<(Entity, &mut Walker, &mut Seeker, &WallBouncer, &WalkerState, &Transform, Option<&Ragdoll>)>,
    mut rng: Local<Option<fastrand::Rng>>,
    mut all_routes: Local<Vec<(String, Vec<Vec2>)>>,
) {
    let _timed = TimedBy(&mut cost, std::time::Instant::now(), 0);
    let rng = rng.get_or_insert_with(|| fastrand::Rng::with_seed(config.seed ^ 0x60a1));
    if all_routes.is_empty() {
        *all_routes = routes();
    }
    let dt = time.delta_secs();
    for (entity, mut walker, mut seeker, bouncer, state, transform, ragdoll) in &mut walkers {
        // Down or getting up: the goal waits.
        if ragdoll.is_some_and(|r| r.is_falling()) {
            continue;
        }
        let at = Vec2::new(transform.translation.x, transform.translation.z);
        seeker.since += dt;
        while seeker.route.first().is_some_and(|point| point.distance(at) < ARRIVED) {
            seeker.route.remove(0);
            if seeker.route.is_empty() {
                info!("playground: {entity} reached the {}", seeker.name);
            }
        }
        if seeker.route.is_empty() || seeker.since > GOAL_SECONDS {
            if !seeker.route.is_empty() {
                info!("playground: {entity} gave up on the {}", seeker.name);
            }
            let (name, route) = if rng.f32() < 0.7 { all_routes[rng.usize(..all_routes.len())].clone() } else { ("floor".into(), vec![free_floor_point(rng)]) };
            *seeker = Seeker { name, route, since: 0.0, detour: 0.0 };
        }
        if bouncer.turning_to.is_some() || bouncer.avoiding_to.is_some() {
            seeker.detour = DETOUR_SECONDS;
            continue;
        }
        if seeker.detour > 0.0 {
            seeker.detour -= dt;
            continue;
        }
        let toward = seeker.route[0] - at;
        let yaw = yaw_of(Vec3::new(toward.x, 0.0, toward.y));
        if angle_between(yaw, state.facing.yaw) > 0.02 {
            walker.steer = Steer::Toward { yaw, rate: BOUNCE_TURN_RATE };
        }
    }
}

/// Where the way along `direction` from `from` (on the ground) is blocked
/// within [`LOOK_AHEAD`], and the face blocking it, as its horizontal
/// normal: a wall, a rise taller than [`MAX_RISE`] between two samples, or
/// a slope steeper than [`MAX_SLOPE_DEGREES`], up or down. A drop is no
/// block.
///
/// The ground is sampled every [`PROBE_STEP`] by a ray straight down from
/// well above the last sample's height: a horizontal ray at one height, as
/// the walls had, hit a stair's riser or a gentle ramp as squarely as a
/// wall.
fn blocked_along(spatial: &SpatialQuery, terrain: &SpatialQueryFilter, from: Vec3, direction: Vec3) -> Option<Vec3> {
    let max_slope_cos = MAX_SLOPE_DEGREES.to_radians().cos();
    // Started inside something (a wall taller than this), a ray is hit at
    // its origin: a rise of the whole reach.
    let ground_at = |at: Vec3, above: f32| {
        let top = above + 4.0;
        spatial.cast_ray(Vec3::new(at.x, top, at.z), Dir3::NEG_Y, top + 1.0, true, terrain).map(|hit| (top - hit.distance, hit.normal))
    };
    // From the ground under the walker, not its height: that is eased
    // (`physics_ground::SUPPORT_SECONDS`) and the mean under both soles, and
    // lagged 0.18 m below the ground halfway up the 35° ramp, which read as
    // a riser too tall and turned the walker back off the ramp's side.
    let mut height = ground_at(from, from.y).map_or(from.y, |(ground, _)| ground);
    let mut distance = 0.0;
    while distance < LOOK_AHEAD {
        let before = from + direction * distance;
        distance += PROBE_STEP;
        let at = from + direction * distance;
        // The room's walls: past one, its inward face.
        let over = Vec2::new(at.x.abs(), at.z.abs()) - Vec2::splat(HALF_ROOM);
        if over.max_element() > 0.0 {
            return Some(if over.x > over.y { Vec3::NEG_X * at.x.signum() } else { Vec3::NEG_Z * at.z.signum() });
        }
        let Some((ground, normal)) = ground_at(at, height) else { continue };
        let rise = ground - height;
        let steep = normal.y < max_slope_cos && rise.abs() > 0.01;
        if rise > MAX_RISE || steep {
            // Up: the face between the two samples, met by a ray across
            // just above the lower one, else the slope's own lean. Down: the
            // edge, facing back up the way it came. Else straight back.
            let face = if rise > 0.0 {
                let across = Dir3::new(direction).ok().and_then(|d| spatial.cast_ray(Vec3::new(before.x, height + 0.1, before.z), d, PROBE_STEP + 0.1, true, terrain));
                across.map_or(normal, |hit| hit.normal)
            } else {
                -normal
            };
            let face = Vec3::new(face.x, 0.0, face.z).normalize_or_zero();
            return Some(if face == Vec3::ZERO { -direction } else { face });
        }
        height = ground;
    }
    None
}

/// Turns each walker nearing a wall, or ground too tall or steep to walk
/// onto, onto its face's reflection of its heading, `d − 2(d·n)n`, give
/// or take `--jitter`.
fn turn_from_terrain(
    config: Res<Config>,
    spatial: SpatialQuery,
    mut cost: ResMut<SteerCost>,
    mut walkers: Query<(&mut Walker, &mut WallBouncer, &WalkerState, &Transform)>,
    mut rng: Local<Option<fastrand::Rng>>,
) {
    let walker_count = walkers.iter().len() as u64;
    let _timed = TimedBy(&mut cost, std::time::Instant::now(), walker_count);
    let rng = rng.get_or_insert_with(|| fastrand::Rng::with_seed(config.seed ^ 0xb0b));
    let terrain = SpatialQueryFilter::from_mask(TERRAIN_LAYER);
    for (mut walker, mut bouncer, state, transform) in &mut walkers {
        // Going to sit, its approach steers it, round the chair and up to
        // the table.
        if walker.sit.is_some() {
            continue;
        }
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
        // Straight ahead and two whiskers to the sides: met at a shallow
        // angle, a wall reached the straight probe only when the walker was
        // nearly touching it (2 m × sin 5° ≈ 0.17 m; measured 0.13). Only a
        // face it is heading into; one it already walks away from or along
        // (the wall beside a stair), a whisker merely grazed.
        let blocked = [0.0f32, WHISKER, -WHISKER]
            .into_iter()
            .filter_map(|angle| blocked_along(&spatial, &terrain, transform.translation, Quat::from_rotation_y(angle) * ahead))
            .find(|normal| ahead.dot(*normal) < 0.0);
        let Some(normal) = blocked else { continue };
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
fn avoid_each_other(config: Res<Config>, mut cost: ResMut<SteerCost>, mut walkers: Query<(Entity, &mut Walker, &mut WallBouncer, &WalkerState, &Transform)>) {
    let _timed = TimedBy(&mut cost, std::time::Instant::now(), 0);
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

/// A ragdolled walker whose feet have stepped out over a drop falls
/// ([`LEDGE_DROP`]), lands below and gets up. Without a ragdoll there is no
/// body to fall: the walker steps down to the ground below within the
/// body's ease (`physics_ground::SUPPORT_SECONDS`).
fn fall_off_ledges(mut walkers: Query<(&mut Walker, &PhysicsGround, &Ragdoll)>) {
    for (mut walker, ground, ragdoll) in &mut walkers {
        if !ragdoll.is_falling() && ground.support - ground.under > LEDGE_DROP {
            info!("playground: stepped off a {:.2} m drop, falling", ground.support - ground.under);
            walker.fall_now = true;
        }
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
             north: ramps 15/25/35/45/50 deg, east: 5 m stair, south: fences 0.1-1.6 m\n\
             WASD move, Q/E down/up, hold right mouse (or M) to look, Shift run, wheel speed",
            config.physics,
            config.ragdoll_distance,
            characters.iter().count(),
        );
    }
}

/// `--trace-feet`: the first character's ankles (world) and heading each
/// frame, as rendered, then where it stands and whether it is falling:
/// `FEET t yaw lx ly lz rx ry rz hips_y x z support falling`.
#[allow(clippy::type_complexity)]
fn trace_feet(
    config: Res<Config>,
    time: Res<Time>,
    characters: Query<(&HumanoidSkeleton, &WalkerState, &Transform, Option<&PhysicsGround>, Option<&Ragdoll>)>,
    world: Query<&GlobalTransform>,
) {
    if !config.trace_feet {
        return;
    }
    let Some((skeleton, state, root, ground, ragdoll)) = characters.iter().next() else { return };
    let (Ok(left), Ok(right), Ok(hips)) =
        (world.get(skeleton.entity(Bone::LeftFoot)), world.get(skeleton.entity(Bone::RightFoot)), world.get(skeleton.entity(Bone::Hips)))
    else {
        return;
    };
    let (l, r) = (left.translation(), right.translation());
    println!(
        "FEET {:.4} {:.4} {:.4} {:.4} {:.4} {:.4} {:.4} {:.4} {:.4} {:.3} {:.3} {:.3} {}",
        time.elapsed_secs(),
        state.facing.yaw,
        l.x,
        l.y,
        l.z,
        r.x,
        r.y,
        r.z,
        hips.translation().y,
        root.translation.x,
        root.translation.z,
        ground.map_or(0.0, |g| g.support),
        ragdoll.is_some_and(|r| r.is_falling()) as u8
    );
}

/// `--bench SECS`: after 3 s to load and settle, every frame's wall time
/// for SECS seconds; then `BENCH` with the count, p50 and p99, and the
/// steering's own time per walker per frame ([`SteerCost`]), and exit.
fn bench(
    config: Res<Config>,
    time: Res<Time<Real>>,
    steer: Res<SteerCost>,
    mut frames: Local<Vec<f32>>,
    mut steer_at_start: Local<Option<(std::time::Duration, u64)>>,
    characters: Query<(Option<&Ragdoll>, Option<&Proxy>), With<WalkerState>>,
) {
    let Some(seconds) = config.bench else { return };
    let now = time.elapsed_secs();
    if now < 3.0 {
        return;
    }
    let (steer_time, steer_frames) = *steer_at_start.get_or_insert((steer.time, steer.walker_frames));
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
        let walker_frames = (steer.walker_frames - steer_frames).max(1);
        println!("BENCH steering {:.4} ms per walker per frame ({walker_frames} walker-frames)", (steer.time - steer_time).as_secs_f64() * 1e3 / walker_frames as f64);
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
