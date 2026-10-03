//! Character gallery: visual test/debug harness for migera's procedural
//! character animation work, built on the ordinary Bevy PBR pipeline (no
//! SDF/hybrid renderer — that one is being kept for effects/world-gen, see
//! `docs/knowledge/`). Mirrors `examples/gallery.rs`'s own conventions
//! (CLI-parsed `Resource` configs, an egui "Controls" panel, an on-screen
//! HUD, a once-a-second debug log, `--shot`/`--at-frame` headless
//! screenshot verification) so this example grows the same way gallery.rs
//! did: one small CLI-flag-and-Resource addition per new animation feature,
//! not a rewrite each time.
//!
//! Today this only shows the static T-pose rig (`migera::character`) — no
//! animation is applied yet. Later iterations add locomotion/IK here behind
//! their own flags, the same way `gallery.rs` grew GI methods/reflections/
//! transmission one flag at a time on top of its own original bare scene.
//!
//! Run: `cargo run --release --example character_gallery`

use std::f32::consts::TAU;

use bevy::camera::{Exposure, Hdr};
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::diagnostic::{FrameCount, FrameTimeDiagnosticsPlugin};
use bevy::math::EulerRot;
use bevy::prelude::*;
use bevy::remote::http::RemoteHttpPlugin;
use bevy::remote::RemotePlugin;
use bevy::render::view::window::screenshot::{save_to_disk, Screenshot};
use bevy_egui::{egui, EguiContexts, EguiPlugin, EguiPrimaryContextPass};

use migera::character::{Bone, BoneMarker, HumanoidSkeleton};
use migera::character::anim::asset::AnimAssetPlugin;
use migera::character::anim::poses as anim_poses;
use migera::character::anim::ground::{FlatGround, SlopedGround};
use migera::character::anim::plugin::{AnimFootIk, AnimGround};
use migera::character::anim::walker;
use migera::character::anim::{approach, sitting};
use avian3d::prelude::PhysicsPlugins;
use migera::character::anim::{
    spawn_gltf_humanoid, spawn_ragdoll, AnimPlugin, AnimRagdollPlugin, AnimSprings, AnimTarget, HumanoidPlugin,
    HumanoidProportions, HumanoidSet, Ragdoll, RagdollHit, RagdollSet, RagdollSpawnConfig, Steer, Walker, WalkerPlugin,
    WalkerSet, FALL_DAMPING,
};

/// `--shot PATH --at-frame N`: headless screenshot-based verification, same
/// convention as `examples/gallery.rs`'s own `ShotConfig`/`auto_shot`.
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
                    cfg.shot = Some((cfg.shot.map_or("/tmp/character_gallery.png".into(), |s| s.0), val(&mut i).parse().unwrap_or(120)));
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
        info!("character_gallery: screenshot -> {path}");
        commands.spawn(Screenshot::primary_window()).observe(save_to_disk(path.clone()));
    }
    if *fired && !*exited && frame.0 >= at + 60 {
        *exited = true;
        std::thread::sleep(std::time::Duration::from_millis(600));
        std::process::exit(0);
    }
}

// ---------------------------------------------------------------------------
// Camera: orbit (continuously circling the rig) or one of a small set of
// PRESET views (front/back/left/right/top) — replaces an earlier design
// where "fixed" meant manually dialing in a raw world-space position AND a
// separate rotation (pitch/yaw/roll in degrees) to match, which needed
// several trial-and-error re-shots even for a simple "look at the character
// from the front" shot, since a position and a rotation that don't actually
// point at the rig are two independently wrong things to fix instead of
// one. Every preset instead only picks a DIRECTION to view the rig from
// (`ViewPreset::eye_offset`) at a shared distance/height tuned for this
// example's ~1.8m-tall subject, then always derives its rotation via
// `Transform::looking_at` — so a preset is, by construction, never
// mis-framed the way a manually-typed rotation could be.
// ---------------------------------------------------------------------------

/// One of the camera's small set of fixed viewing angles — always looks
/// directly at [`CameraConfig::look_at_height`] on the rig's own vertical
/// centerline (`Transform::looking_at`, never a manually-authored
/// rotation), at [`CameraConfig::preset_distance`] away, so switching
/// presets always keeps the whole ~1.8m-tall rig framed in view.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum ViewPreset {
    Front,
    Back,
    Left,
    Right,
    /// Straight down from directly overhead — unlike the other four, this
    /// looks at the ground origin (`y = 0`), not `look_at_height`, since
    /// "look down at mid-torso height from above" would tilt the view off
    /// top-down; `Vec3::X` (not the usual `Vec3::Y`) is `looking_at`'s own
    /// up-axis here, since a straight-down forward vector is parallel to
    /// `Vec3::Y` and `looking_at` can't derive a stable "up" from a
    /// forward vector parallel to its own up hint.
    Top,
}

/// This rig's own approximate standing height (`Bone::Head.t_pose_world_
/// position().y` is ~1.77m, rounded up a little for headroom) — used ONLY
/// by [`ViewPreset::Top`]'s own `eye_position` to keep `preset_distance`
/// meaning "distance from the character" consistently across every preset
/// (see that function's own doc comment).
const RIG_APPROX_HEIGHT: f32 = 1.8;

impl ViewPreset {
    const ALL: [ViewPreset; 5] = [ViewPreset::Front, ViewPreset::Back, ViewPreset::Left, ViewPreset::Right, ViewPreset::Top];

    fn label(self) -> &'static str {
        match self {
            ViewPreset::Front => "Front",
            ViewPreset::Back => "Back",
            ViewPreset::Left => "Left",
            ViewPreset::Right => "Right",
            ViewPreset::Top => "Top",
        }
    }

    fn parse(s: &str) -> Option<ViewPreset> {
        match s {
            "front" => Some(ViewPreset::Front),
            "back" => Some(ViewPreset::Back),
            "left" => Some(ViewPreset::Left),
            "right" => Some(ViewPreset::Right),
            "top" => Some(ViewPreset::Top),
            _ => None,
        }
    }

    /// This preset's own camera position — always `distance` away from the
    /// world origin along one cardinal direction, at `look_at_height` (or,
    /// for `Top`, well above the rig's own ~1.8m height so it's never
    /// clipping through the head).
    ///
    /// `Front`/`Back` face along `-Z`/`+Z` matching this crate's own `-Z is
    /// forward` convention (`skeleton.rs`'s module doc, `pose::walk_step`'s
    /// own doc comment): the character's own face points toward `-Z`, so an
    /// observer seeing that face must stand further along `-Z` than the
    /// character (on the side the face-normal points toward) and look back
    /// in the `+Z` direction — NOT stand at `+Z`, which instead views the
    /// character from the same side they're already facing AWAY toward,
    /// i.e. their back/spine (a real, live-caught bug: `Front` at `+Z`
    /// rendered the character's spine, `Back` at `-Z` rendered their face,
    /// backwards from what the labels promise).
    fn eye_position(self, distance: f32, look_at_height: f32) -> Vec3 {
        match self {
            ViewPreset::Front => Vec3::new(0.0, look_at_height, -distance),
            ViewPreset::Back => Vec3::new(0.0, look_at_height, distance),
            ViewPreset::Left => Vec3::new(-distance, look_at_height, 0.0),
            ViewPreset::Right => Vec3::new(distance, look_at_height, 0.0),
            // `distance` alone (as the other four presets use it directly)
            // would put the camera only `distance` above the GROUND, not
            // `distance` above the character -- at the default 2.6m that's
            // barely 0.9m of clearance over the rig's own ~1.7m head
            // height, live-verified to render an extreme close-up on the
            // face/shoulders instead of a real top-down view. Adding
            // `RIG_APPROX_HEIGHT` keeps `distance`'s own meaning
            // consistent with the other four presets ("how far the camera
            // sits from the character"), measured from the top of the
            // head rather than from the ground.
            ViewPreset::Top => Vec3::new(0.0, distance + RIG_APPROX_HEIGHT, 0.0),
        }
    }

    fn look_at(self, look_at_height: f32) -> Vec3 {
        match self {
            ViewPreset::Top => Vec3::ZERO,
            _ => Vec3::new(0.0, look_at_height, 0.0),
        }
    }

    fn up(self) -> Vec3 {
        match self {
            ViewPreset::Top => Vec3::NEG_Z,
            _ => Vec3::Y,
        }
    }
}

/// `--character-model PATH` (default `models/puppet_base.gltf`, relative to
/// `assets/`) / `--character-yaw-correction DEGREES` (default `180`):
/// which real skinned glTF character to load and how much to rigidly yaw
/// its root to match this crate's own `-Z`-forward convention. Exists so a
/// different real-mesh asset can be dropped in and tried in the same
/// session without editing source — see [`resolve_bone_node_name`]'s own
/// doc comment for how the Bone -> glTF-node-name mapping adapts
/// automatically to a Mixamo- or UE-Mannequin-named rig, the other
/// per-asset thing that used to be hardcoded to one single table. The
/// yaw correction is NOT auto-
/// detected — it depends on which way the new asset's own mesh geometry
/// happens to face, cheapest to determine by eye via `--camera-preset
/// front`/`--camera-preset left` (see `spawn_real_mesh`'s own doc comment
/// for the live-caught bug this correction exists to fix in the first
/// place).
#[derive(Resource, Clone)]
struct CharacterModelConfig {
    path: String,
    yaw_correction_radians: f32,
}

impl Default for CharacterModelConfig {
    fn default() -> Self {
        Self { path: "models/puppet_base.gltf".to_string(), yaw_correction_radians: std::f32::consts::PI }
    }
}

impl CharacterModelConfig {
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
                "--character-model" => cfg.path = val(&mut i),
                "--character-yaw-correction" => {
                    let degrees: f32 = val(&mut i).parse().unwrap_or(180.0);
                    cfg.yaw_correction_radians = degrees.to_radians();
                }
                _ => {}
            }
            i += 1;
        }
        cfg
    }
}

/// `--camera-mode orbit|preset` (default `orbit`), plus mode-specific
/// params:
/// - orbit: `--camera-orbit-radius <meters>` (default `3.2`) and
///   `--camera-orbit-speed <rad/s>` (default `0.35`).
/// - preset: `--camera-preset front|back|left|right|top` (default `front`),
///   optionally `--camera-preset-distance <meters>` (default `2.6`).
#[derive(Resource, Clone)]
struct CameraConfig {
    mode: CameraMode,
    orbit_radius: f32,
    orbit_speed: f32,
    orbit_height: f32,
    look_at_height: f32,
    preset: ViewPreset,
    preset_distance: f32,
    /// The preset view follows the character's hips (`--camera-follow`).
    follow: bool,
}

#[derive(Clone, Copy, PartialEq, Eq, Default)]
enum CameraMode {
    #[default]
    Orbit,
    Preset,
}

impl Default for CameraConfig {
    fn default() -> Self {
        Self {
            mode: CameraMode::default(),
            orbit_radius: 3.2,
            orbit_speed: 0.35,
            orbit_height: 1.4,
            look_at_height: 0.95,
            preset: ViewPreset::Front,
            preset_distance: 2.6,
            follow: false,
        }
    }
}

impl CameraConfig {
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
                "--camera-mode" => {
                    cfg.mode = match val(&mut i).as_str() {
                        "preset" | "fixed" => CameraMode::Preset,
                        _ => CameraMode::Orbit,
                    };
                }
                "--camera-orbit-radius" => {
                    cfg.orbit_radius = val(&mut i).parse().unwrap_or(cfg.orbit_radius);
                }
                "--camera-orbit-speed" => {
                    cfg.orbit_speed = val(&mut i).parse().unwrap_or(cfg.orbit_speed);
                }
                "--camera-orbit-height" => {
                    cfg.orbit_height = val(&mut i).parse().unwrap_or(cfg.orbit_height);
                }
                "--camera-preset" => {
                    if let Some(preset) = ViewPreset::parse(val(&mut i).as_str()) {
                        cfg.preset = preset;
                        cfg.mode = CameraMode::Preset;
                    }
                }
                "--camera-preset-distance" => {
                    cfg.preset_distance = val(&mut i).parse().unwrap_or(cfg.preset_distance);
                }
                "--camera-follow" => cfg.follow = true,
                // A preset's eye and aim height, metres (default 0.95): low,
                // for what happens at the floor.
                "--camera-look-height" => {
                    cfg.look_at_height = val(&mut i).parse().unwrap_or(cfg.look_at_height);
                }
                _ => {}
            }
            i += 1;
        }
        cfg
    }

    fn preset_transform(&self) -> Transform {
        Transform::from_translation(self.preset.eye_position(self.preset_distance, self.look_at_height))
            .looking_at(self.preset.look_at(self.look_at_height), self.preset.up())
    }
}

fn camera_controller(
    time: Res<Time>,
    cfg: Res<CameraConfig>,
    mut cams: Query<&mut Transform, With<Camera3d>>,
    skeletons: Query<&HumanoidSkeleton>,
    globals: Query<&GlobalTransform>,
    idle: Res<AnimIdleConfig>,
) {
    match cfg.mode {
        CameraMode::Preset => {
            let mut transform = cfg.preset_transform();
            // `--camera-follow`: the view moves with the character's hips
            // across the floor, and up or down `--anim-slope` with the
            // ground under them, so a fall or a climb stays in frame.
            if cfg.follow
                && let Some(hips) = skeletons.iter().next().and_then(|s| globals.get(s.entity(Bone::Hips)).ok())
            {
                let (x, z) = (hips.translation().x, hips.translation().z);
                transform.translation += Vec3::new(x, idle.slope * -z, z);
            }
            for mut cam_transform in &mut cams {
                *cam_transform = transform;
            }
        }
        CameraMode::Orbit => {
            let a = (time.elapsed_secs() * cfg.orbit_speed) % TAU;
            let r = cfg.orbit_radius;
            let pos = Vec3::new(r * a.cos(), cfg.orbit_height, r * a.sin());
            let look_at = Vec3::new(0.0, cfg.look_at_height, 0.0);
            for mut transform in &mut cams {
                transform.translation = pos;
                *transform = transform.looking_at(look_at, Vec3::Y);
            }
        }
    }
}

fn spawn_camera(mut commands: Commands, cfg: Res<CameraConfig>, mut egui_global_settings: ResMut<bevy_egui::EguiGlobalSettings>) {
    // Disable bevy_egui's own auto-detected primary context -- see this
    // function's own `PrimaryEguiContext` marker below for why the
    // auto-detection heuristic (attach to whichever entity gains a
    // `Camera` component FIRST) can silently pick the wrong entity here.
    egui_global_settings.auto_create_primary_context = false;

    let initial = match cfg.mode {
        CameraMode::Preset => cfg.preset_transform(),
        CameraMode::Orbit => Transform::from_xyz(cfg.orbit_radius, cfg.orbit_height, 0.0)
            .looking_at(Vec3::new(0.0, cfg.look_at_height, 0.0), Vec3::Y),
    };
    // PrimaryEguiContext, explicit, PAIRED with the
    // `auto_create_primary_context = false` line above -- a REAL bug found
    // by the user reporting the egui Controls panel was completely
    // invisible (not merely mispositioned) despite the 3D scene and gizmos
    // rendering fine. Root cause: bevy_egui's own auto-detection attaches
    // the primary egui context to the FIRST entity it ever sees gain a
    // `Camera` component, with no check that the entity can actually
    // render anything. `spawn_light`'s own `shadow_maps_enabled: true`
    // makes Bevy's shadow-mapping machinery create an internal shadow-view
    // entity that ALSO carries a bare `Camera` component (with no render
    // graph of its own, confirmed via `bevy_render::camera`'s own "doesn't
    // have a render graph configured" warning -- a shadow-pass view is not
    // a full scene camera). That shadow-view entity happened to register
    // as `Added<Camera>` before this real one, so egui permanently latched
    // onto it instead -- the whole egui UI rendered every frame with no
    // error, just to a camera that never draws to screen. Disabling
    // auto-detection and marking this camera explicitly sidesteps the
    // heuristic entirely rather than depending on spawn/entity ordering
    // being "lucky" every run.
    commands.spawn((
        Camera3d::default(),
        Msaa::Sample4,
        Hdr,
        Tonemapping::default(),
        Exposure::default(),
        initial,
        bevy_egui::PrimaryEguiContext,
    ));
}

// ---------------------------------------------------------------------------
// Debug gizmos: per-joint local axes (so a bone's rotation state is visible
// at a glance, not just its position) plus the world origin axes — toggle
// via `--gizmos on|off` or the egui panel.
// ---------------------------------------------------------------------------

/// `--gizmos on|off` (default `on`) / `--joint-axis-length F` (default
/// `0.10`): whether debug gizmos draw at all, and how long each joint's
/// local-axis gizmo is drawn — kept short by default so nearby joints
/// (e.g. LeftArm -> LeftForeArm) don't visually overlap. `--joint-chain
/// on|off` (default `on`): a dedicated SKELETON debug layer, separate from
/// the joint-axis triads above — draws the real solved joint chain as
/// white lines plus a rest-pose marker per bone, so the rig's actual shape
/// is readable independently of whatever the skinned mesh renders. See
/// `draw_skeleton_debug_gizmos`'s own doc comment for what each color
/// means.
#[derive(Resource, Clone, Copy)]
struct DebugGizmos {
    enabled: bool,
    joint_axis_length: f32,
    world_axes: bool,
    joint_chain: bool,
    /// `--show-real-mesh on|off` (default `on`): whether the real,
    /// downloaded skinned glTF character (`assets/models/puppet_base.
    /// gltf` — a CC0 Quaternius "Superhero Male" model) renders at all.
    /// Independent of every other gizmo flag above — the thin debug
    /// skeleton (bone capsules/joint spheres from `spawn_humanoid_debug_
    /// skeleton`) always stays visible regardless of this toggle, since
    /// that's the "real bone position" ground truth this flag is
    /// specifically meant to let you see WITHOUT the mesh drawn over it.
    show_real_mesh: bool,
}

impl Default for DebugGizmos {
    fn default() -> Self {
        Self { enabled: true, joint_axis_length: 0.10, world_axes: true, joint_chain: true, show_real_mesh: true }
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
                "--gizmos" => cfg.enabled = val(&mut i) != "off",
                "--joint-axis-length" => {
                    cfg.joint_axis_length = val(&mut i).parse().unwrap_or(cfg.joint_axis_length);
                }
                "--world-axes" => cfg.world_axes = val(&mut i) != "off",
                "--joint-chain" => cfg.joint_chain = val(&mut i) != "off",
                "--show-real-mesh" => cfg.show_real_mesh = val(&mut i) != "off",
                _ => {}
            }
            i += 1;
        }
        cfg
    }
}

const WORLD_AXIS_HALF_LENGTH: f32 = 50.0;

fn draw_world_axis_gizmos(mut gizmos: Gizmos, cfg: Res<DebugGizmos>) {
    if !cfg.enabled || !cfg.world_axes {
        return;
    }
    let e = Vec3::splat(WORLD_AXIS_HALF_LENGTH);
    gizmos.line(-Vec3::X * e.x, Vec3::X * e.x, Color::srgb(0.9, 0.2, 0.2));
    gizmos.line(-Vec3::Y * e.y, Vec3::Y * e.y, Color::srgb(0.2, 0.9, 0.2));
    gizmos.line(-Vec3::Z * e.z, Vec3::Z * e.z, Color::srgb(0.3, 0.4, 0.95));
}

/// One RGB axis triad per skeleton joint, in world space — the single
/// biggest debugging win over the plain capsule rig: a capsule alone shows
/// where a bone is, but not which way it's rotated (roll around its own
/// length is completely invisible on a rotationally-symmetric capsule).
/// The gizmo triad makes every joint's full orientation, including roll,
/// readable at a glance.
fn draw_joint_axis_gizmos(mut gizmos: Gizmos, cfg: Res<DebugGizmos>, joints: Query<&GlobalTransform, With<BoneMarker>>) {
    if !cfg.enabled {
        return;
    }
    for transform in &joints {
        gizmos.axes(*transform, cfg.joint_axis_length);
    }
}

/// Skeleton debug layer — drawn from the REAL mesh joints' own
/// already-written `GlobalTransform` (the exact same state the animation
/// stack wrote this frame), rather than independently re-deriving world
/// positions from any solver's own internal synthetic-space state.
///
/// An earlier version of this function drew straight from the superseded
/// muscle solver's own `position_of`, converted into mesh-world-space via a
/// hand-rolled correction applied once at the root -- that is fundamentally
/// the wrong shape of fix, because retargeting composes each bone's
/// rotation through a REAL, per-bone parent chain, which a single rigid
/// transform applied once at the root cannot reproduce for any bone more
/// than one hop from `Hips`. Two live-caught, screenshot-confirmed failures
/// came from exactly this: a hardcoded 180°-yaw-only root correction
/// rendered the overlay floating off to one side; swapping in the real
/// ancestor rotation applied directly to each joint's raw absolute position
/// collapsed the whole overlay down near the feet. Reading straight from
/// the real joints' own `GlobalTransform` sidesteps this whole class of
/// bug: the overlay is *by construction* exactly where the mesh's own skin
/// actually is, since it comes from the SAME data Bevy's renderer uses.
///
/// This is the primary verification view the project's pose-checking rules
/// mandate: ground truth, independent of mesh skinning, occlusion and
/// foreshortening quirks that can make a correct pose LOOK wrong.
///
/// Per bone (relative to its parent joint):
/// - **white** solid line: the real joint chain, `parent`'s to `child`'s
///   own real `GlobalTransform::translation` — ground truth, identical to
///   what the mesh's own skin renders.
/// - **yellow** marker (small sphere): this child bone's own REST (T-pose/
///   bind-pose) world position, obtained by walking the SAME real parent
///   `GlobalTransform` chain but with each bone's `rest_rotation`/
///   `rest_direction` instead of its currently-solved one — a large gap
///   between this marker and the actual white-line endpoint is the direct
///   visual read of "how far this joint currently is from its own rest
///   pose."
fn draw_skeleton_debug_gizmos(
    mut gizmos: Gizmos,
    cfg: Res<DebugGizmos>,
    skeletons: Query<&HumanoidSkeleton>,
    global_transforms: Query<&GlobalTransform>,
) {
    if !cfg.enabled || !cfg.joint_chain {
        return;
    }

    let Ok(skeleton) = skeletons.single() else { return };

    for (parent_bone, child_bone) in joint_pairs() {
        let Ok(parent_transform) = global_transforms.get(skeleton.entity(parent_bone)) else { continue };
        let Ok(child_transform) = global_transforms.get(skeleton.entity(child_bone)) else { continue };
        let parent_position = parent_transform.translation();
        let child_position = child_transform.translation();

        gizmos.line(parent_position, child_position, Color::srgb(0.95, 0.95, 0.95));

        // Rest-pose marker: `parent`'s own REAL global rotation (already
        // reflecting every ancestor's real rest/solved state up to this
        // point -- same chain the white line above walks), composed with
        // `child_bone`'s own real REST local rotation/length, so this
        // marker sits exactly where `child_bone` would render if it (and
        // only it) snapped back to rest -- matching retargeting's own
        // `rest_rotation`/`rest_direction`-based rest pose, not this
        // crate's synthetic T-pose constant (which the real mesh never
        // actually holds -- see `HumanoidSkeleton::rest_direction`'s own
        // doc comment).
        let rest_offset = skeleton.rest_direction(child_bone) * child_bone.t_pose_offset().length();
        let t_pose_target = parent_position + parent_transform.rotation() * rest_offset;
        gizmos.sphere(Isometry3d::from_translation(t_pose_target), 0.015, Color::srgb(0.95, 0.85, 0.15));
    }
}

/// Every parent→child bone pair in the rig, i.e. the skeleton's own tree
/// edges. Inlined here when the superseded muscle module was deleted; it is
/// a one-liner over `Bone::ALL` and only this overlay still wanted it.
fn joint_pairs() -> impl Iterator<Item = (Bone, Bone)> {
    Bone::ALL.iter().filter_map(|&child| child.parent().map(|parent| (parent, child)))
}

// ---------------------------------------------------------------------------
// egui controls panel
// ---------------------------------------------------------------------------

fn controls_panel(
    mut contexts: EguiContexts,
    mut gizmos_cfg: ResMut<DebugGizmos>,
    mut camera_cfg: ResMut<CameraConfig>,
    mut idle_cfg: ResMut<AnimIdleConfig>,
    mut sit: ResMut<SitConfig>,
    mut characters: Query<(&mut AnimTarget, &mut AnimSprings)>,
) -> Result {
    let ctx = contexts.ctx_mut()?;
    egui::Window::new("Controls")
        .anchor(egui::Align2::RIGHT_TOP, egui::vec2(-10.0, 10.0))
        .collapsible(true)
        .resizable(false)
        .show(ctx, |ui| {
            ui.label("Camera");
            ui.horizontal(|ui| {
                ui.radio_value(&mut camera_cfg.mode, CameraMode::Orbit, "Orbit");
                ui.radio_value(&mut camera_cfg.mode, CameraMode::Preset, "Preset view");
            });
            ui.add_enabled(
                camera_cfg.mode == CameraMode::Orbit,
                egui::Slider::new(&mut camera_cfg.orbit_radius, 1.0..=10.0).text("Orbit radius"),
            );
            ui.add_enabled(
                camera_cfg.mode == CameraMode::Orbit,
                egui::Slider::new(&mut camera_cfg.orbit_speed, -2.0..=2.0).text("Orbit speed (rad/s)"),
            );
            ui.add_enabled_ui(camera_cfg.mode == CameraMode::Preset, |ui| {
                ui.horizontal(|ui| {
                    for preset in ViewPreset::ALL {
                        ui.radio_value(&mut camera_cfg.preset, preset, preset.label());
                    }
                });
                ui.add(egui::Slider::new(&mut camera_cfg.preset_distance, 1.0..=8.0).text("Preset distance"));
            });
            ui.separator();
            ui.label("Debug gizmos");
            ui.checkbox(&mut gizmos_cfg.enabled, "Joint axis gizmos");
            ui.checkbox(&mut gizmos_cfg.world_axes, "World origin axes");
            ui.checkbox(&mut gizmos_cfg.joint_chain, "Joint chain + rest markers");
            ui.add(egui::Slider::new(&mut gizmos_cfg.joint_axis_length, 0.02..=0.3).text("Joint axis length"));
            ui.checkbox(&mut gizmos_cfg.show_real_mesh, "Real skinned mesh");
            ui.separator();

            ui.label("Procedural animation (character::anim)");
            let Ok((mut target, mut springs)) = characters.single_mut() else {
                ui.label("no animated character in the scene");
                return;
            };

            // Switching the target pose does NOT snap the rig: the spring
            // stack takes it from wherever it currently is, with velocity
            // carried through. That continuity is the whole point of
            // Stage 1, and clicking between these two buttons repeatedly
            // is the quickest way to see it.
            ui.horizontal(|ui| {
                ui.label("Pose:");
                for (label, pose) in [
                    ("Rest", anim_poses::rest as fn() -> _),
                    ("Relaxed", anim_poses::relaxed_stand),
                    ("Wave", anim_poses::wave),
                ] {
                    if ui.button(label).clicked() {
                        target.pose = pose();
                    }
                }
            });

            // Sitting down, and standing up again: each button sits that
            // way (standing up first if seated another way).
            ui.horizontal_wrapped(|ui| {
                ui.label("Sit:");
                for how in sitting::Sitting::ALL {
                    if ui.button(how.name()).clicked() {
                        sit.choice = Some(how);
                        sit.live = Some(true);
                    }
                }
                if ui.button("Stand").clicked() {
                    sit.live = Some(false);
                }
            });

            ui.add(
                egui::Slider::new(&mut idle_cfg.speed, 0.0..=3.0)
                    .text("Gait speed (m/s)"),
            );
            ui.add(
                egui::Slider::new(&mut idle_cfg.slope, -0.4..=0.4)
                    .text("Ground slope (rise/run)"),
            );

            // Spring tuning, stamped across every bone at once. This is the
            // dial that turns the same pose data into a heavy brute or a
            // quick duellist, so it is worth having at hand while looking
            // at the rig: halflife is "how long to cover half the remaining
            // distance", damping below 1.0 overshoots and gives
            // follow-through.
            egui::CollapsingHeader::new("Spring tuning (all bones)").default_open(false).show(
                ui,
                |ui| {
                    let mut current = springs.0[Bone::Spine];
                    let mut changed = false;
                    changed |= ui
                        .add(
                            egui::Slider::new(&mut current.halflife, 0.01..=0.5)
                                .text("Half-life (s)"),
                        )
                        .changed();
                    changed |= ui
                        .add(
                            egui::Slider::new(&mut current.damping_ratio, 0.2..=2.0)
                                .text("Damping ratio"),
                        )
                        .changed();
                    if changed {
                        *springs = AnimSprings::uniform(current);
                    }
                    ui.label("1.0 = critically damped; below that overshoots.");
                },
            );
        });
    Ok(())
}

// ---------------------------------------------------------------------------
// Scene: ground plane, lights, the humanoid rig.
// ---------------------------------------------------------------------------

fn spawn_light(mut commands: Commands) {
    commands.spawn((
        DirectionalLight { color: Color::srgb(1.0, 0.95, 0.85), illuminance: 6000.0, shadow_maps_enabled: true, ..default() },
        Transform::from_xyz(4.0, 6.0, 3.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    commands.spawn(AmbientLight { color: Color::WHITE, brightness: 250.0, ..default() });
}

fn spawn_ground(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    idle: Res<AnimIdleConfig>,
) {
    // Tilted to `--anim-slope`, so what is drawn is what the feet (and a
    // fallen body, `spawn_physics_floor`) stand on.
    let size = if idle.slope == 0.0 { 10.0 } else { 40.0 };
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::default().mesh().size(size, size))),
        MeshMaterial3d(materials.add(StandardMaterial { base_color: Color::srgb(0.3, 0.32, 0.35), perceptual_roughness: 0.9, ..default() })),
        Transform::from_rotation(slope_rotation(idle.slope)),
    ));
}

/// The turn from flat to `SlopedGround { grade }`'s plane, `y = grade·(−z)`.
fn slope_rotation(grade: f32) -> Quat {
    Quat::from_rotation_arc(Vec3::Y, Vec3::new(0.0, 1.0, grade).normalize())
}

/// Tags the root entity of the real skinned glTF character so
/// [`apply_real_mesh_visibility`] can find it without re-loading the
/// asset or walking the scene graph.
#[derive(Component)]
struct RealMeshRoot;

/// Loads `assets/models/puppet_base.gltf` (a real, CC0-licensed skinned
/// humanoid — Quaternius "Superhero Male") as a plain Bevy world-asset
/// root. Bevy's own glTF importer handles the skin/joint-matrix machinery
/// internally once this asset is spawned; `humanoid::bind_gltf_humanoids`
/// (`HumanoidPlugin`) then binds its joint nodes to this crate's own `Bone`
/// enum so `character::anim` drives the skin live, and `WalkerPlugin`
/// gives the bound character its animation stack.
///
/// The binding is not trivial: the glTF's own bone names — `pelvis`,
/// `thigh_l`, `upperarm_l`, an Unreal-Mannequin-style convention — do not
/// match this crate's Mixamo-style names at all, hence
/// `humanoid::UE_MANNEQUIN_BONE_NAMES`, and hence
/// `HumanoidSkeleton::for_other_rig` capturing this specific rig's own
/// rest rotations rather than assuming the synthetic T-pose numbers.
///
/// No scale correction needed: the raw glTF's own `pelvis` node sits at
/// local translation `(0, 0.043, 0.949)` — already meter-scale, not
/// centimeters — with its parent `root` node carrying a `-90°` X rotation
/// (the standard Blender-Z-up -> glTF-Y-up export correction), which
/// after composing puts `pelvis` at ~0.949m world height, matching this
/// crate's own `Bone::Hips` rest height (`0.94`, see `skeleton.rs`) almost
/// exactly. Bevy's glTF importer applies this same node-transform chain
/// automatically, so the default `Transform::IDENTITY` here is already
/// correct — verified by reading the raw JSON node data directly, not
/// assumed (an earlier, unverified guess that this file needed a `0.01`
/// centimeters-to-meters scale factor was wrong: the file is already
/// meter-scale, and applying that scale would have shrunk the character
/// to 1/100th its correct size).
///
/// `bevy_world_serialization`'s `WorldAssetRoot` component (not the
/// classic `SceneRoot`/`bevy_scene` `Scene` type — this Bevy version
/// (0.19) replaced scene spawning with a `WorldAsset`/`WorldAssetRoot`
/// pair) is what actually instantiates a loaded glTF scene's entities
/// into the world.
fn spawn_real_mesh(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    model_cfg: Res<CharacterModelConfig>,
    choice: Res<AnimPoseChoice>,
    idle: Res<AnimIdleConfig>,
    proportions: Option<Res<ProportionsConfig>>,
) {
    let root = spawn_gltf_humanoid(&mut commands, &asset_server, &model_cfg.path, model_cfg.yaw_correction_radians, Transform::IDENTITY);
    let mut character = commands.entity(root);
    character.insert((
        RealMeshRoot,
        // The walk the gallery's flags and panel ask for; kept in step with
        // them each frame by `steer_the_walker`.
        Walker { pose: choice.0.clone(), speed: idle.speed, ..Default::default() },
        AnimGround(if idle.slope == 0.0 {
            Box::new(FlatGround::default())
        } else {
            Box::new(SlopedGround { height: 0.0, grade: idle.slope })
        }),
    ));
    if let Some(config) = proportions {
        character.insert(HumanoidProportions { stature: config.stature });
    }
    // The yaw correction (`spawn_gltf_humanoid`'s `yaw_correction`, default
        // 180°, `--character-yaw-correction` to override for a
        // differently-facing asset) -- this DEFAULT asset's
        // own skinned mesh geometry visibly faces +Z (live-confirmed via
        // screenshot: the face renders toward the `Front` camera preset,
        // which sits on the +Z side looking down -Z), but every OTHER
        // crate convention (`Bone::t_pose_offset`'s own doc comment,
        // `pose::walk_step`'s own forward-travel direction, `ViewPreset::
        // eye_position`'s own Front/Back/Left/Right placement) assumes
        // -Z-forward -- without this correction, `Left`/`Right` camera
        // presets show the WRONG side of the character (live-caught:
        // `--anim-pose wave`, which only moves `RightArm`, rendered the
        // raised arm prominently in-frame from the `Left` preset and
        // mostly hidden behind the torso from `Right` -- backwards) and a
        // walk cycle would move the character backward relative to its
        // own facing. A DIFFERENT asset may face a different way at
        // export time -- not auto-detectable from the glTF alone, cheapest
        // determined by eye via `--camera-preset front`/`left` and passing
        // `--character-yaw-correction 0`/`90`/`270`/etc as needed.
        //
        // This is a RIGID re-orientation of the whole mesh root, safe to
        // apply here (unlike an earlier, since-reverted attempt to rotate
        // this same root or `pelvis`'s own live `Transform.rotation`
        // directly to fix a DIFFERENT, ROTATION-retargeting bug, which
        // corrupted leg articulation because `apply_solved_sim_to_
        // skeleton` was AT THE TIME separately hardcoding `global_
        // rotation[Hips] = Quat::IDENTITY` instead of reading `HumanoidSkeleton
        // ::rest_rotation(Bone::Hips)` -- rotating the root on top of that
        // double-applied a correction the swing math wasn't using yet).
        // That bug is fixed at its actual source now (`apply_solved_sim_
        // to_skeleton` seeds `global_rotation[Hips]` from `skeleton.rest_
        // rotation(Bone::Hips)`, itself derived from `pelvis`'s own real
        // captured bind rotation via `build_real_mesh_skeleton`'s `named`
        // query, which reads whatever this root's rotation actually is at
        // spawn time) -- a rigid root rotation composes transparently
        // through that per-bone math (every swing is bone-LOCAL; nothing
        // downstream hardcodes an assumption about the root's own
        // orientation anymore), and `HumanoidSkeleton::hips_local_
        // translation_for`'s own `hips_parent_rest_world_rotation`
        // likewise already reads this root's rotation live via `Global
        // Transform`, so it stays correct automatically too.
        //
        // Recorded as well as applied (`FacingCorrection`), because root
        // motion writes this same `Transform::rotation` every frame and needs
        // something to compose ONTO.
}

/// Applies `DebugGizmos::show_real_mesh` to the real mesh's own root
/// entity — runs once per frame (cheap: only touches `Visibility`, no
/// mesh/material work) so `--show-real-mesh`/the egui checkbox both take
/// effect immediately, matching every other `DebugGizmos` flag's own
/// live-toggle convention. `Visibility::Hidden` on the root propagates to
/// every child in the loaded scene automatically via Bevy's normal
/// visibility inheritance, so this doesn't need to walk the scene graph
/// itself. The thin debug skeleton (`spawn_humanoid_debug_skeleton`'s own
/// capsules/joint spheres) is untouched by this system — it always stays
/// visible as the one canonical "real bone position" display, independent
/// of whether the real mesh is shown on top of it.
fn apply_real_mesh_visibility(cfg: Res<DebugGizmos>, mut roots: Query<&mut Visibility, With<RealMeshRoot>>) {
    if !cfg.is_changed() {
        return;
    }
    let visibility = if cfg.show_real_mesh { Visibility::Inherited } else { Visibility::Hidden };
    for mut root_visibility in &mut roots {
        *root_visibility = visibility;
    }
}

// ---------------------------------------------------------------------------
// Animation configuration
//
// There is no longer a backend switch here. `--anim-backend` existed to A/B
// the rotation-space stack against the superseded position-space `muscle`
// module during the cutover; that module is gone, so the flag has nothing
// left to select between.
// ---------------------------------------------------------------------------

/// Which named pose the `anim` backend starts in
/// (`--anim-pose rest|relaxed_stand|wave`).
#[derive(Resource, Debug, Clone)]
struct AnimPoseChoice(String);

impl AnimPoseChoice {
    fn from_args() -> Self {
        let mut args = std::env::args().skip(1);
        while let Some(arg) = args.next() {
            if arg == "--anim-pose"
                && let Some(name) = args.next()
            {
                return Self(name);
            }
        }
        Self("relaxed_stand".to_string())
    }
}

/// How fast the character is notionally travelling, driving the gait clock
/// (`--anim-speed M_PER_S`, default `0` = standing).
///
/// A stand-in for a locomotion controller: Stage 2 only reads a speed, so
/// this is enough to see cadence scale without building one.
#[derive(Resource, Debug, Clone, Copy)]
struct AnimIdleConfig {
    speed: f32,
    /// Ground rise per metre travelled forward (`--anim-slope GRADE`).
    /// Zero is flat.
    slope: f32,
    /// Turn rate in radians per second (`--anim-turn RATE`). Zero walks
    /// straight; anything else walks a steady circle, which is what makes
    /// turning visible — a character that turns once and then walks straight
    /// looks the same as one that never turned.
    turn: f32,
    /// A world-space point to look at (`--anim-look X,Y,Z`).
    look_at: Option<Vec3>,
    /// A world-space point for the left hand to reach for
    /// (`--anim-reach X,Y,Z`).
    ///
    /// Left only, deliberately: with one arm solving and one arm free, a single
    /// screenshot shows the reaching arm against its own untouched mirror
    /// image, which is a far better read on whether the solve looks right than
    /// two symmetric arms would be.
    reach: Option<Vec3>,
}

impl AnimIdleConfig {
    fn from_args() -> Self {
        let mut config =
            Self { speed: 0.0, slope: 0.0, turn: 0.0, look_at: None, reach: None };

        let mut args = std::env::args().skip(1);
        while let Some(arg) = args.next() {
            match arg.as_str() {
                "--anim-speed" => {
                    if let Some(value) = args.next()
                        && let Ok(speed) = value.parse::<f32>()
                    {
                        config.speed = speed;
                    }
                }
                "--anim-slope" => {
                    if let Some(value) = args.next()
                        && let Ok(slope) = value.parse::<f32>()
                    {
                        config.slope = slope;
                    }
                }
                "--anim-turn" => {
                    if let Some(value) = args.next()
                        && let Ok(turn) = value.parse::<f32>()
                    {
                        config.turn = turn;
                    }
                }
                "--anim-look" => {
                    if let Some(value) = args.next() {
                        let parts: Vec<f32> =
                            value.split(',').filter_map(|p| p.trim().parse().ok()).collect();
                        if parts.len() == 3 {
                            config.look_at = Some(Vec3::new(parts[0], parts[1], parts[2]));
                        }
                    }
                }
                "--anim-reach" => {
                    if let Some(value) = args.next() {
                        let parts: Vec<f32> =
                            value.split(',').filter_map(|p| p.trim().parse().ok()).collect();
                        if parts.len() == 3 {
                            config.reach = Some(Vec3::new(parts[0], parts[1], parts[2]));
                        }
                    }
                }
                _ => {}
            }
        }

        config
    }
}

/// `--anim-speed-schedule T:SPEED,T:SPEED,...`: set the gait speed to SPEED
/// at T seconds. Makes starts and stops reproducible, so they can be
/// measured over BRP and screenshotted at a known frame.
#[derive(Resource, Debug, Clone, Default)]
struct SpeedSchedule(Vec<(f32, f32)>);

impl SpeedSchedule {
    fn from_args() -> Self {
        let mut args = std::env::args().skip_while(|arg| arg != "--anim-speed-schedule").skip(1);
        let mut steps: Vec<(f32, f32)> = args
            .next()
            .unwrap_or_default()
            .split(',')
            .filter_map(|step| {
                let (at, speed) = step.split_once(':')?;
                Some((at.trim().parse().ok()?, speed.trim().parse().ok()?))
            })
            .collect();
        steps.sort_by(|a, b| a.0.total_cmp(&b.0));
        Self(steps)
    }
}

/// `--push-schedule T:FORWARD:LEFT,...`: at T seconds, shove the standing
/// character, changing its centre of mass's velocity by FORWARD and LEFT
/// m/s along its own axes. Reproducible pushes for BRP captures.
#[derive(Resource, Default)]
struct PushSchedule {
    steps: Vec<(f32, bevy::math::Vec2)>,
    /// How many steps have already been delivered.
    delivered: usize,
}

impl PushSchedule {
    fn from_args() -> Self {
        let mut args = std::env::args().skip_while(|arg| arg != "--push-schedule").skip(1);
        let mut steps: Vec<(f32, bevy::math::Vec2)> = args
            .next()
            .unwrap_or_default()
            .split(',')
            .filter_map(|step| {
                let mut parts = step.split(':').map(|part| part.trim().parse::<f32>().ok());
                Some((parts.next()??, bevy::math::Vec2::new(parts.next()??, parts.next()??)))
            })
            .collect();
        steps.sort_by(|a, b| a.0.total_cmp(&b.0));
        Self { steps, delivered: 0 }
    }

    /// The pushes due by `now` and not yet delivered.
    fn due(&mut self, now: f32) -> Vec<bevy::math::Vec2> {
        let due: Vec<_> =
            self.steps.iter().skip(self.delivered).take_while(|(at, _)| *at <= now).map(|(_, push)| *push).collect();
        self.delivered += due.len();
        due
    }
}

/// Applies [`SpeedSchedule`]: the latest step already reached wins.
fn follow_speed_schedule(time: Res<Time>, schedule: Res<SpeedSchedule>, mut idle: ResMut<AnimIdleConfig>) {
    let now = time.elapsed_secs();
    if let Some(&(_, speed)) = schedule.0.iter().rev().find(|(at, _)| *at <= now)
        && idle.speed != speed
    {
        idle.speed = speed;
    }
}

/// Keeps the gallery's character walking as its flags, panel and schedules
/// ask: the speed and turn (`--anim-speed`, the slider, the speed
/// schedule, `--anim-turn`), the look and reach, and the pushes due. The
/// walk itself is `character::anim::walker`'s.
fn steer_the_walker(
    time: Res<Time>,
    idle: Res<AnimIdleConfig>,
    mut pushes: ResMut<PushSchedule>,
    sit: Res<SitConfig>,
    mut walkers: Query<&mut Walker>,
) {
    let due = pushes.due(time.elapsed_secs());
    let sitting = sit.wanted(time.elapsed_secs());
    for mut walker in &mut walkers {
        walker.speed = idle.speed;
        walker.steer = if idle.turn != 0.0 { Steer::Circle(idle.turn) } else { Steer::Straight };
        walker.look_at = idle.look_at;
        walker.reach = idle.reach;
        walker.sit = sitting;
        walker.chair_height = sitting::CHAIR_HEIGHT;
        walker.chair = sit.chair;
        for &push in &due {
            walker.push(push);
        }
    }
}

/// Sitting: `--sit chair:upright|chair:reclined|chair:crossed|chair:forward|
/// floor:cross_legged|floor:propped|floor:hug|floor:side|floor:kneeling`
/// sits down `--sit-at SECONDS` in (default 1) and stands up again at
/// `--stand-at SECONDS`, if given. The panel's Sit buttons do it live.
///
/// A chair's way, it walks to the chair at `--chair X,Z,HEADING` (the floor
/// point under the seated hips, and the way a seated person faces, degrees
/// about +Y from -Z; default `-1.5,-1.5,180`, facing the camera) and turns
/// round to sit on it. `--chair here` sits where it stands instead, the
/// chair put under it.
#[derive(Resource, Debug, Clone)]
struct SitConfig {
    /// The way of sitting the schedule or the panel last chose.
    choice: Option<sitting::Sitting>,
    sit_at: f32,
    stand_at: Option<f32>,
    /// Set by the panel: sit `choice` now (`Some(true)`), stand (`Some(false)`),
    /// or follow the schedule (`None`).
    live: Option<bool>,
    /// The chair to walk to; `None` sits where it stands.
    chair: Option<approach::Chair>,
}

impl SitConfig {
    fn from_args() -> Self {
        let chair = |x: f32, z: f32, heading: f32| approach::Chair {
            seat: Vec3::new(x, 0.0, z),
            forward: approach::direction_of(heading.to_radians()),
            height: sitting::CHAIR_HEIGHT,
        };
        let mut config = Self { choice: None, sit_at: 1.0, stand_at: None, live: None, chair: Some(chair(-1.5, -1.5, 180.0)) };
        let mut args = std::env::args().skip(1);
        while let Some(arg) = args.next() {
            match arg.as_str() {
                "--sit" => config.choice = args.next().and_then(|name| sitting::Sitting::by_name(&name)),
                "--sit-at" => config.sit_at = args.next().and_then(|v| v.parse().ok()).unwrap_or(config.sit_at),
                "--stand-at" => config.stand_at = args.next().and_then(|v| v.parse().ok()),
                "--chair" => {
                    let value = args.next().unwrap_or_default();
                    let numbers: Vec<f32> = value.split(',').filter_map(|v| v.trim().parse().ok()).collect();
                    config.chair = match numbers[..] {
                        [x, z, heading] => Some(chair(x, z, heading)),
                        _ => None,
                    };
                }
                _ => {}
            }
        }
        config
    }

    /// How the walker should sit at `elapsed` seconds, if at all.
    fn wanted(&self, elapsed: f32) -> Option<sitting::Sitting> {
        let sitting = match self.live {
            Some(sit) => sit,
            None => elapsed >= self.sit_at && self.stand_at.is_none_or(|stand| elapsed < stand),
        };
        self.choice.filter(|_| sitting)
    }
}

/// `--step-seconds S`: every frame advances the clock exactly `S` seconds,
/// however long it took. For runs on a software renderer (an offscreen
/// display draws ~9 frames a second), whose motion is then the same as at
/// full speed, only slower to watch. Never for timing measurements.
fn step_fixed_seconds(mut commands: Commands) {
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        if arg == "--step-seconds"
            && let Some(seconds) = args.next().and_then(|v| v.parse::<f64>().ok())
        {
            commands.insert_resource(bevy::time::TimeUpdateStrategy::ManualDuration(std::time::Duration::from_secs_f64(seconds)));
        }
    }
}

/// The chair the gallery's character sits on: spawned under where its
/// seated hips land (`sitting::seat_offset`), in front of which it stands,
/// once its rig has bound and a chair pose is chosen.
#[derive(Component)]
struct GalleryChair;

fn place_chair(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    sit: Res<SitConfig>,
    chairs: Query<(), With<GalleryChair>>,
    characters: Query<(&Walker, &AnimFootIk, &HumanoidSkeleton)>,
    globals: Query<&GlobalTransform>,
) {
    if !chairs.is_empty() || !sit.choice.is_some_and(sitting::Sitting::on_chair) {
        return;
    }
    let (seat, yaw, height) = match sit.chair {
        // Where it was asked for; the character walks to it.
        Some(chair) => (chair.seat, Quat::from_rotation_y(approach::heading_of(chair.forward)), chair.height),
        // Under where the character's seated hips will land.
        None => {
            let Ok((walker, foot_ik, skeleton)) = characters.single() else { return };
            let Some(rig) = &foot_ik.rig else { return };
            let base = anim_poses::by_name(&walker.pose).unwrap_or_else(anim_poses::relaxed_stand);
            let stood = migera::character::anim::stance::stance_on_rig(&base, migera::character::anim::stance::DEFAULT_KNEE_FLEX, rig);
            let (offset, height) = sitting::seat_offset(rig, &stood, sitting::CHAIR_HEIGHT);
            let at = |bone: Bone| globals.get(skeleton.entity(bone)).ok().map(GlobalTransform::translation);
            let (Some(hips), Some(foot), Some(toe)) = (at(Bone::Hips), at(Bone::LeftFoot), at(Bone::LeftToeBase)) else { return };
            // The character's facing in the world, heel to toe.
            let forward = Vec3::new(toe.x - foot.x, 0.0, toe.z - foot.z).normalize_or_zero();
            if forward == Vec3::ZERO {
                return;
            }
            let left = Vec3::Y.cross(forward);
            (Vec3::new(hips.x, 0.0, hips.z) + forward * offset.x + left * offset.y, Quat::from_rotation_arc(Vec3::NEG_Z, forward), height)
        }
    };
    let wood = materials.add(StandardMaterial { base_color: Color::srgb(0.45, 0.30, 0.18), perceptual_roughness: 0.7, ..default() });
    let mut part = |size: Vec3, centre: Vec3| (Mesh3d(meshes.add(Cuboid::from_size(size))), MeshMaterial3d(wood.clone()), Transform::from_translation(centre));
    let (depth, width, thick) = (0.44, 0.46, 0.04);
    // The backrest just behind the buttocks (the hips joint ~0.14 m in front
    // of them), the seat reaching forward under the thighs.
    let centre = Vec3::new(0.0, height - thick * 0.5, 0.14 - depth * 0.5);
    commands
        .spawn((GalleryChair, Transform::from_translation(seat).with_rotation(yaw), Visibility::default()))
        .with_children(|chair| {
            chair.spawn(part(Vec3::new(width, thick, depth), centre));
            let back = centre.z + depth * 0.5;
            chair.spawn(part(Vec3::new(width, 0.42, thick), Vec3::new(0.0, height + 0.25, back)));
            for (x, z) in [(-1.0, -1.0), (1.0, -1.0), (-1.0, 1.0), (1.0, 1.0)] {
                let leg = Vec3::new(x * (width * 0.5 - 0.03), (height - thick) * 0.5, centre.z + z * (depth * 0.5 - 0.03));
                chair.spawn(part(Vec3::new(0.035, height - thick, 0.035), leg));
            }
        });
}

/// `--ragdoll on|off` (default `off`) plus its live strength dial.
///
/// Off by default because a ragdoll costs a rigid body and a constraint per
/// bone, and the kinematic stack is what most characters want. Stage 4 is
/// the opt-in.
#[derive(Resource, Debug, Clone, Copy)]
struct RagdollConfig {
    enabled: bool,
    /// 0 = fully limp, 1 = fully driven toward the animated pose.
    strength: f32,
    /// `--hit-at-frame N`: deliver one [`RagdollHit`] at frame N. Scheduled
    /// by frame rather than by key so a `--shot` a few frames later captures
    /// the reaction reproducibly. `H` delivers the same hit live.
    hit_at_frame: Option<u32>,
    /// `--hit-bone NAME` (default `LeftForeArm`).
    hit_bone: Bone,
    /// `--fall-at-frame N`: let the ragdoll fall at frame N, as `F` does
    /// live. A push no step can catch does it by itself
    /// (`Balance::falls`).
    fall_at_frame: Option<u32>,
    /// `--stand-on-own-feet N`: from frame N the ragdoll stands on its own
    /// feet (`Ragdoll::stand_on_own_feet`): unpinned, full gravity, its
    /// joints carrying it. The screen still shows the animation; read the
    /// bodies over BRP.
    stand_at_frame: Option<u32>,
    /// `--fall-damping PER_SECOND`: the falling joints' damping (default
    /// `FALL_DAMPING`).
    fall_damping: f32,
    /// `--hit-velocity X,Y,Z` in m/s (default straight up, `0,4,0`).
    ///
    /// Up rather than "backward" by default because it needs no facing: a
    /// hanging forearm knocked upward swings visibly in both the Front and
    /// Left views, whichever way the rig was authored to face.
    hit_velocity: Vec3,
}

impl RagdollConfig {
    fn from_args() -> Self {
        let mut config = Self {
            enabled: false,
            strength: 1.0,
            hit_at_frame: None,
            fall_at_frame: None,
            stand_at_frame: None,
            fall_damping: FALL_DAMPING,
            hit_bone: Bone::LeftForeArm,
            hit_velocity: Vec3::new(0.0, 4.0, 0.0),
        };

        let mut args = std::env::args().skip(1);
        while let Some(arg) = args.next() {
            match arg.as_str() {
                "--ragdoll" => config.enabled = args.next().as_deref() != Some("off"),
                "--ragdoll-strength" => {
                    if let Some(value) = args.next().and_then(|v| v.parse().ok()) {
                        config.strength = value;
                    }
                }
                "--hit-at-frame" => config.hit_at_frame = args.next().and_then(|v| v.parse().ok()),
                "--fall-at-frame" => config.fall_at_frame = args.next().and_then(|v| v.parse().ok()),
                "--stand-on-own-feet" => config.stand_at_frame = args.next().and_then(|v| v.parse().ok()),
                "--fall-damping" => {
                    if let Some(value) = args.next().and_then(|v| v.parse().ok()) {
                        config.fall_damping = value;
                    }
                }
                "--hit-bone" => {
                    if let Some(bone) = args.next().as_deref().and_then(Bone::from_name) {
                        config.hit_bone = bone;
                    }
                }
                "--hit-velocity" => {
                    let parts: Vec<f32> = args
                        .next()
                        .unwrap_or_default()
                        .split(',')
                        .filter_map(|part| part.trim().parse().ok())
                        .collect();
                    if let [x, y, z] = parts[..] {
                        config.hit_velocity = Vec3::new(x, y, z);
                    }
                }
                _ => {}
            }
        }

        config
    }
}

/// Strikes every ragdolled character, on `H` or at `--hit-at-frame`.
fn deliver_ragdoll_hits(
    config: Res<RagdollConfig>,
    frame: Res<FrameCount>,
    keys: Res<ButtonInput<KeyCode>>,
    characters: Query<Entity, With<Ragdoll>>,
    mut hits: MessageWriter<RagdollHit>,
) {
    let scheduled = config.hit_at_frame == Some(frame.0);
    if !scheduled && !keys.just_pressed(KeyCode::KeyH) {
        return;
    }

    for character in &characters {
        info!(
            "character_gallery: hit {} at {:?} m/s (frame {})",
            config.hit_bone.name(),
            config.hit_velocity,
            frame.0,
        );
        hits.write(RagdollHit::new(character, config.hit_bone, config.hit_velocity));
    }
}

/// Plan steps 4b and 4.5: at `--stand-on-own-feet N` every ragdoll stands
/// on its own feet; `B` toggles it live, back to a pinned root and on.
fn stand_when_asked(config: Res<RagdollConfig>, frame: Res<FrameCount>, keys: Res<ButtonInput<KeyCode>>, mut rigs: Query<&mut Ragdoll>) {
    let toggled = keys.just_pressed(KeyCode::KeyB);
    if config.stand_at_frame != Some(frame.0) && !toggled {
        return;
    }
    for mut ragdoll in &mut rigs {
        if toggled && ragdoll.self_supporting {
            info!("character_gallery: pinned again at frame {}", frame.0);
            ragdoll.stop_standing_on_own_feet();
        } else {
            info!("character_gallery: standing on its own feet at frame {}", frame.0);
            ragdoll.stand_on_own_feet();
        }
    }
}

/// H2: on `F` or at `--fall-at-frame`, asks the ragdolled walker to fall
/// (`Walker::fall_now`), with `--fall-damping`. A push no step catches makes
/// it fall by itself; the walker gets it up again (`walker::get_up_when_rested`).
fn ask_to_fall(config: Res<RagdollConfig>, frame: Res<FrameCount>, keys: Res<ButtonInput<KeyCode>>, mut walkers: Query<&mut Walker>) {
    let asked = config.fall_at_frame == Some(frame.0) || keys.just_pressed(KeyCode::KeyF);
    for mut walker in &mut walkers {
        walker.fall_damping = config.fall_damping;
        if asked {
            info!("character_gallery: asked to fall at frame {}", frame.0);
            walker.fall_now = true;
        }
    }
}

/// The floor a falling ragdoll lands on: the rendered ground is only a
/// mesh.
fn spawn_physics_floor(mut commands: Commands, idle: Res<AnimIdleConfig>) {
    use avian3d::prelude::{Collider, Friction, RigidBody};
    commands.spawn((
        RigidBody::Static,
        Collider::half_space(Vec3::Y),
        Friction::new(1.0),
        Transform::from_rotation(slope_rotation(idle.slope)),
    ));
}

/// Draws every simulated ragdoll body as a cyan line along its own axis.
///
/// Essential rather than decorative: the physics bodies are invisible
/// otherwise, because nothing writes them back onto the rendered
/// skeleton (see [`attach_ragdoll`]'s note on `RagdollSet::ReadBack`).
/// Without this overlay a ragdoll that had exploded, collapsed, or never
/// spawned at all would look exactly like one working perfectly.
fn draw_ragdoll_gizmos(
    mut gizmos: Gizmos,
    cfg: Res<DebugGizmos>,
    ragdolls: Query<&Ragdoll>,
    // avian's OWN position/rotation, not `Transform`. avian simulates into
    // `Position`/`Rotation` and syncs them onto `Transform` later in the
    // frame, so a gizmo reading `Transform` here draws a stale pose — or,
    // before the first sync, nothing at all where the bodies should be.
    bodies: Query<(
        &avian3d::prelude::Position,
        &avian3d::prelude::Rotation,
        &avian3d::prelude::Collider,
    )>,
) {
    if !cfg.enabled {
        return;
    }

    for ragdoll in &ragdolls {
        for &bone in Bone::ALL.iter() {
            let Some(body) = ragdoll.bodies[bone] else { continue };
            let Ok((position, rotation, collider)) = bodies.get(body) else { continue };

            // The capsule's REAL segment, from the collider itself. A body's
            // rotation is its bone's frame (see `spawn_bone_body`), which on
            // a real rig does not run down the bone — drawing along local +Y
            // instead put these lines at odd angles to the skeleton and read
            // as a tracking error that was not there.
            let Some(capsule) = collider.shape().as_capsule() else { continue };
            let (a, b) = (capsule.segment.a, capsule.segment.b);
            gizmos.line(
                position.0 + rotation.0 * Vec3::new(a.x, a.y, a.z),
                position.0 + rotation.0 * Vec3::new(b.x, b.y, b.z),
                Color::srgb(0.2, 0.9, 0.9),
            );
            gizmos.sphere(
                Isometry3d::from_translation(position.0),
                0.012,
                Color::srgb(0.2, 0.9, 0.9),
            );
        }
    }
}


/// Builds the simulated skeleton once the rig exists and has been through
/// transform propagation at least once.
///
/// The propagation wait is not incidental: [`spawn_ragdoll`] places every
/// body from its bone's own `GlobalTransform`, and before the first
/// propagation those all read as the origin — which would stack all 17
/// bodies on top of each other and let the solver explode them apart.
/// `Without<Ragdoll>` makes this run exactly once per character.
/// Animated rigs that do not yet have a simulated skeleton — the
/// `Without<Ragdoll>` is what makes [`attach_ragdoll`] run once per
/// character rather than every frame.
type RagdollessRigs<'w, 's> = Query<
    'w,
    's,
    (Entity, &'static HumanoidSkeleton, Option<&'static AnimFootIk>),
    (With<AnimTarget>, Without<Ragdoll>),
>;

fn attach_ragdoll(
    mut commands: Commands,
    config: Res<RagdollConfig>,
    global_transforms: Query<&GlobalTransform>,
    rigs: RagdollessRigs,
) {
    if !config.enabled {
        return;
    }

    for (entity, skeleton, foot_ik) in &rigs {
        // Skip until propagation has actually run: a rig whose hips still
        // sit at the origin has not been propagated yet.
        let Ok(hips) = global_transforms.get(skeleton.entity(Bone::Hips)) else { continue };
        if hips.translation() == Vec3::ZERO {
            continue;
        }
        // And until the foot IK has measured the live rig: the feet stand
        // on sole blocks built from it, so a fall lands on flat feet.
        let Some(rig) = foot_ik.and_then(|ik| ik.rig.as_ref()) else { continue };

        let mut ragdoll = spawn_ragdoll(
            &mut commands,
            entity,
            skeleton,
            &global_transforms,
            &RagdollSpawnConfig {
                feet: Some(migera::character::anim::ragdoll_plugin::sole_blocks(rig)),
                ..Default::default()
            },
        );
        ragdoll.set_strength(config.strength);

        let simulated = Bone::ALL.iter().filter(|&&b| ragdoll.bodies[b].is_some()).count();
        info!("character_gallery: ragdoll spawned with {simulated} simulated bones");

        commands.entity(entity).insert(ragdoll);
    }
}


// ---------------------------------------------------------------------------
// HUD: camera pose (bottom-left) + skeleton summary (bottom-right) — mirrors
// `examples/gallery.rs`'s own camera/object HUD split.
// ---------------------------------------------------------------------------

#[derive(Component)]
struct CameraStatsText;

fn spawn_camera_hud(mut commands: Commands) {
    commands.spawn((
        Text::new("camera —"),
        TextFont { font_size: FontSize::Px(13.0), ..default() },
        TextColor(Color::srgb(0.88, 0.92, 0.96)),
        Node { position_type: PositionType::Absolute, left: Val::Px(10.0), bottom: Val::Px(10.0), ..default() },
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
        transform.translation.x, transform.translation.y, transform.translation.z,
        pitch.to_degrees(), yaw.to_degrees(), roll.to_degrees(),
    )
}

fn update_camera_hud(
    time: Res<Time>,
    frame: Res<FrameCount>,
    diagnostics: Res<bevy::diagnostic::DiagnosticsStore>,
    cams: Query<&Transform, With<Camera3d>>,
    mut text: Query<&mut Text, With<CameraStatsText>>,
) {
    let Ok(transform) = cams.single() else { return };
    let frame_ms = diagnostics
        .get(&FrameTimeDiagnosticsPlugin::FRAME_TIME)
        .and_then(|d| d.smoothed())
        .unwrap_or(0.0) as f32;
    if let Ok(mut text) = text.single_mut() {
        **text = camera_pose_line(transform, frame.0, time.elapsed_secs(), frame_ms);
    }
}

/// One line per bone: local position + rotation (Euler degrees, YXZ) —
/// exactly the information needed to spot a bad offset or an unintended
/// rotation drift once procedural animation starts writing to these
/// transforms. Bones are listed in `Bone::ALL`'s parent-before-child order
/// so the block reads top-to-bottom the same way the rig's own hierarchy
/// does.
#[derive(Component)]
struct SkeletonStatsText;

fn spawn_skeleton_hud(mut commands: Commands) {
    commands.spawn((
        Text::new("skeleton —"),
        TextFont { font_size: FontSize::Px(12.0), ..default() },
        TextColor(Color::srgb(0.88, 0.92, 0.96)),
        Node { position_type: PositionType::Absolute, right: Val::Px(10.0), bottom: Val::Px(10.0), ..default() },
        TextLayout::justify(Justify::Right),
        SkeletonStatsText,
    ));
}

fn skeleton_debug_lines(bones: &Query<&Transform, With<BoneMarker>>, skeleton: &HumanoidSkeleton) -> String {
    let mut lines = Vec::with_capacity(Bone::ALL.len());
    for &bone in &Bone::ALL {
        let entity = skeleton.entity(bone);
        let Ok(transform) = bones.get(entity) else { continue };
        let (yaw, pitch, roll) = transform.rotation.to_euler(EulerRot::YXZ);
        lines.push(format!(
            "{:<14} pos ({:5.2},{:5.2},{:5.2})  rot (p{:4.0} y{:4.0} r{:4.0})",
            bone.name(),
            transform.translation.x, transform.translation.y, transform.translation.z,
            pitch.to_degrees(), yaw.to_degrees(), roll.to_degrees(),
        ));
    }
    lines.join("\n")
}

/// The worst bone-length error in the rig, in metres — live distance
/// between two joints' own real `GlobalTransform`s versus the rest length
/// that pair is bound at.
///
/// Under the rotation-space stack this should sit at float noise
/// *permanently*, and that is precisely why it is worth printing: a
/// rotation cannot stretch a bone, so any nonzero value here means
/// something is writing translations into the chain. The superseded
/// position-space solver needed this reading as a convergence check
/// (distance constraints only approach their targets); here it is a
/// structural alarm.
///
/// Reported as a single worst-case number rather than 21 lines, so it fits
/// the once-a-second log line and a `--shot` run carries the same signal.
///
/// The reference length is captured from the rig itself on the first call
/// and compared against thereafter. It deliberately is NOT
/// `Bone::t_pose_offset().length()`: a real retargeted mesh is uniformly
/// scaled relative to this crate's synthetic T-pose, so measuring against
/// the synthetic constant would report a large, permanent, entirely
/// spurious error on a perfectly correct rig. Comparing the rig against
/// its own first frame measures the only thing being claimed here — that
/// nothing stretches a bone over time.
fn worst_bone_length_error(
    skeleton: &HumanoidSkeleton,
    global_transforms: &Query<&GlobalTransform>,
    reference: &mut Option<Vec<f32>>,
) -> (f32, &'static str) {
    let live: Vec<(Bone, f32)> = joint_pairs()
        .filter_map(|(parent_bone, child_bone)| {
            let parent = global_transforms.get(skeleton.entity(parent_bone)).ok()?;
            let child = global_transforms.get(skeleton.entity(child_bone)).ok()?;
            Some((child_bone, parent.translation().distance(child.translation())))
        })
        .collect();

    let reference = reference.get_or_insert_with(|| live.iter().map(|(_, len)| *len).collect());

    // A partially-built rig on the first frame would otherwise pin a short
    // reference list and silently stop checking the rest.
    if reference.len() != live.len() {
        return (0.0, "rig still building");
    }

    live.iter()
        .zip(reference.iter())
        .map(|((bone, live_length), rest)| ((live_length - rest).abs(), bone.name()))
        .fold((0.0, "none"), |worst, current| if current.0 > worst.0 { current } else { worst })
}

fn update_skeleton_hud(
    bones: Query<&Transform, With<BoneMarker>>,
    skeletons: Query<&HumanoidSkeleton>,
    global_transforms: Query<&GlobalTransform>,
    mut reference_lengths: Local<Option<Vec<f32>>>,
    mut text: Query<&mut Text, With<SkeletonStatsText>>,
) {
    let Ok(skeleton) = skeletons.single() else { return };
    let mut lines = skeleton_debug_lines(&bones, skeleton);

    let (error, bone) =
        worst_bone_length_error(skeleton, &global_transforms, &mut reference_lengths);
    lines.push_str(&format!("\n\nworst bone-length drift: {error:.5} m ({bone})"));

    if let Ok(mut text) = text.single_mut() {
        **text = lines;
    }
}

// ---------------------------------------------------------------------------
// Once-a-second debug log — same stats the HUD shows, printed to the log so
// they're visible from a headless run too (CI, `--shot`, piped output).
// Mirrors `examples/gallery.rs`'s own `DebugLogTimer`/`log_debug_stats`.
// ---------------------------------------------------------------------------

#[derive(Resource)]
struct DebugLogTimer(Timer);

impl Default for DebugLogTimer {
    fn default() -> Self {
        Self(Timer::from_seconds(1.0, TimerMode::Repeating))
    }
}

#[allow(clippy::too_many_arguments)]
fn log_debug_stats(
    time: Res<Time>,
    frame: Res<FrameCount>,
    mut timer: ResMut<DebugLogTimer>,
    diagnostics: Res<bevy::diagnostic::DiagnosticsStore>,
    cams: Query<&Transform, With<Camera3d>>,
    bones: Query<&Transform, With<BoneMarker>>,
    skeletons: Query<&HumanoidSkeleton>,
    global_transforms: Query<&GlobalTransform>,
    mut reference_lengths: Local<Option<Vec<f32>>>,
    camera_cfg: Res<CameraConfig>,
) {
    if !timer.0.tick(time.delta()).just_finished() {
        return;
    }
    let fps = diagnostics.get(&FrameTimeDiagnosticsPlugin::FPS).and_then(|d| d.smoothed()).unwrap_or(0.0);
    let frame_ms = diagnostics
        .get(&FrameTimeDiagnosticsPlugin::FRAME_TIME)
        .and_then(|d| d.smoothed())
        .unwrap_or(0.0);
    let camera = cams.single().ok().map(|transform| {
        let (yaw, pitch, roll) = transform.rotation.to_euler(EulerRot::YXZ);
        format!(
            "pos ({:.2}, {:.2}, {:.2})   rot (pitch {:.1} yaw {:.1} roll {:.1})",
            transform.translation.x, transform.translation.y, transform.translation.z,
            pitch.to_degrees(), yaw.to_degrees(), roll.to_degrees(),
        )
    });
    let camera_mode = match camera_cfg.mode {
        CameraMode::Orbit => "orbit",
        CameraMode::Preset => camera_cfg.preset.label(),
    };
    // Flattened one-line-per-bone skeleton dump (same data the bottom-right
    // HUD shows), "|"-joined so a headless run still gets full per-bone
    // state in the log, not just on screen.
    let skeleton_line = skeletons
        .single()
        .ok()
        .map(|skeleton| skeleton_debug_lines(&bones, skeleton).replace('\n', "  |  "))
        .unwrap_or_default();
    // The structural alarm, logged so a headless run carries it: under the
    // rotation-space stack a bone's length CANNOT change, so anything
    // beyond float noise here means something is writing translations into
    // the chain. See `worst_bone_length_error`.
    let (length_error, length_bone) = skeletons
        .single()
        .ok()
        .map(|skeleton| {
            worst_bone_length_error(skeleton, &global_transforms, &mut reference_lengths)
        })
        .unwrap_or((0.0, "no rig"));
    info!(
        "character_gallery: frame {}   t {:.2}s   fps {:5.0}   frame {:.2} ms   camera[{camera_mode}] {}   \
         bone-length drift {length_error:.5} m ({length_bone})   |  {skeleton_line}",
        frame.0,
        time.elapsed_secs(),
        fps,
        frame_ms,
        camera.unwrap_or_default(),
    );
}

/// `--proportions winter [H]`: the character rescaled to Winter's fractions
/// of stature `H` metres (default: the stature its legs imply). See
/// `build_real_mesh_skeleton`.
#[derive(Resource, Clone, Copy)]
struct ProportionsConfig {
    stature: Option<f32>,
}

impl ProportionsConfig {
    fn from_args() -> Option<Self> {
        let mut args = std::env::args().skip_while(|arg| arg != "--proportions").skip(1);
        if args.next().as_deref() != Some("winter") {
            return None;
        }
        Some(Self { stature: args.next().and_then(|v| v.parse().ok()) })
    }
}

/// The body-proportion spike (`--proportion-spike move|proxy [FACTOR]`):
/// lengthens the LEFT thigh only, so the right leg stays beside it as the
/// unchanged reference.
///
/// - `move` moves the knee joint down the thigh by `FACTOR`. The thigh's
///   vertices stay where they are and the knee's blend region stretches
///   across the gap.
/// - `proxy` does that AND skins the thigh with a scale along its own +Y
///   (a Mixamo bone's axis) by a helper joint under it, so the thigh's
///   vertices stretch with the bone. The hierarchy itself carries no scale:
///   a scaled parent shears a rotated child (Bevy has no segment scale
///   compensate), so only the skinning matrix sees it.
#[derive(Resource, Clone, Copy)]
struct ProportionSpike {
    proxy: bool,
    factor: f32,
}

impl ProportionSpike {
    fn from_args() -> Option<Self> {
        let mut args = std::env::args().skip_while(|arg| arg != "--proportion-spike").skip(1);
        let proxy = match args.next().as_deref() {
            Some("move") => false,
            Some("proxy") => true,
            _ => return None,
        };
        let factor = args.next().and_then(|v| v.parse().ok()).unwrap_or(1.1);
        Some(Self { proxy, factor })
    }
}

fn apply_proportion_spike(
    mut commands: Commands,
    spike: Res<ProportionSpike>,
    mut done: Local<bool>,
    skeletons: Query<&HumanoidSkeleton>,
    mut transforms: Query<&mut Transform>,
    mut skins: Query<&mut bevy::mesh::skinning::SkinnedMesh>,
) {
    if *done {
        return;
    }
    let Ok(skeleton) = skeletons.single() else { return };
    let (thigh, knee) = (skeleton.entity(Bone::LeftUpLeg), skeleton.entity(Bone::LeftLeg));
    let Ok(mut knee_transform) = transforms.get_mut(knee) else { return };
    knee_transform.translation *= spike.factor;
    if spike.proxy {
        let helper = commands
            .spawn((Transform::from_scale(Vec3::new(1.0, spike.factor, 1.0)), ChildOf(thigh)))
            .id();
        for mut skin in &mut skins {
            for joint in skin.joints.iter_mut().filter(|joint| **joint == thigh) {
                *joint = helper;
            }
        }
    }
    *done = true;
}

fn main() {
    let assets =std::env::current_dir().expect("cwd").join("assets").to_string_lossy().into_owned();

    let mut app = App::new();

    app.add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window { title: "migera character gallery".into(), ..default() }),
            ..default()
        }).set(AssetPlugin { file_path: assets, ..default() }))
        .add_plugins(FrameTimeDiagnosticsPlugin::default())
        .add_plugins(EguiPlugin::default())
        // Bevy Remote Protocol -- exposes a JSON-RPC HTTP endpoint on
        // 127.0.0.1:15702 (BRP's own default) queryable directly via curl,
        // e.g. `curl -s localhost:15702 -H 'content-type: application/json'
        // -d '{"jsonrpc":"2.0","id":1,"method":"world.query","params":
        // {"data":{"components":["bevy_transform::components::transform::
        // Transform"]}}}'` -- lets retargeting bugs be root-caused against
        // live, queryable ECS ground truth (real `Transform`/`GlobalTransform`
        // values on actual bone entities) instead of eyeballing screenshots
        // or threading temporary `eprintln!` diagnostics through the code
        // and removing them again afterward.
        .add_plugins(RemotePlugin::default())
        .add_plugins(RemoteHttpPlugin::default())
        .insert_resource(ShotConfig::from_args())
        .insert_resource(CharacterModelConfig::from_args())
        .insert_resource(CameraConfig::from_args())
        .insert_resource(DebugGizmos::from_args())
        .insert_resource(AnimPoseChoice::from_args())
        .insert_resource(AnimIdleConfig::from_args())
        .insert_resource(SpeedSchedule::from_args())
        .insert_resource(PushSchedule::from_args())
        .insert_resource(DebugLogTimer::default())
        .add_systems(Startup, (spawn_camera, spawn_light, spawn_ground, spawn_real_mesh, spawn_camera_hud, spawn_skeleton_hud))
        .add_systems(Update, (camera_controller, draw_world_axis_gizmos, draw_joint_axis_gizmos, update_camera_hud, apply_real_mesh_visibility))
        .add_systems(
            Update,
            apply_proportion_spike
                .after(HumanoidSet::Bind)
                .run_if(resource_exists::<ProportionSpike>),
        )
        .add_systems(EguiPrimaryContextPass, controls_panel)
        // Scheduled in `PostUpdate`, after egui's own `EguiPrimaryContextPass`
        // sub-schedule, so the screenshot fires no earlier than this frame's
        // UI submission — correct regardless of `Update`-schedule ordering,
        // even though it did not resolve a separate, still-open issue: in
        // this sandbox, `--shot` headless captures were found to drop BOTH
        // the egui panel and the plain Bevy `Text` HUD entirely once any
        // additional `Resource` exists in the app (reproduced with a
        // trivial, unrelated dummy resource) — a capture-mechanism quirk in
        // this exact Bevy/bevy_egui/wgpu/driver combination, not a defect in
        // this example's own systems (which were confirmed, via direct
        // tracing, to run and complete successfully every frame regardless).
        .add_systems(PostUpdate, auto_shot);
    if let Some(spike) = ProportionSpike::from_args() {
        app.insert_resource(spike);
    }
    if let Some(proportions) = ProportionsConfig::from_args() {
        app.insert_resource(proportions);
    }

    // The rotation-space stack is now the only animation backend. The
    // superseded position-space `muscle` module — and with it the
    // `--anim-backend` A/B switch that carried the cutover — was deleted
    // once this path had been the default for a full phase.
    // Binding the glTF rig and walking it are the library's
    // (`HumanoidPlugin`, `WalkerPlugin`); the gallery only steers the walker
    // from its flags, panel and schedules.
    app.add_plugins((AnimPlugin, AnimAssetPlugin, HumanoidPlugin, WalkerPlugin))
        .add_systems(
            Update,
            (draw_skeleton_debug_gizmos, update_skeleton_hud, log_debug_stats),
        )
        .insert_resource(SitConfig::from_args())
        .add_systems(Startup, step_fixed_seconds)
        .add_systems(Update, (follow_speed_schedule, steer_the_walker).chain().before(WalkerSet::Drive))
        .add_systems(Update, place_chair.after(WalkerSet::Drive));

    // The authoring studio, compiled only under `--features anim_studio`
    // so a release consumer never links the editor UI:
    //
    //   cargo run --release --example character_gallery \
    //     --features anim_studio
    #[cfg(feature = "anim_studio")]
    app.add_plugins(migera::character::anim::studio::AnimStudioPlugin);

    // Stage 4 is opt-in (`--ragdoll on`): a simulated body and a constraint
    // per bone is real cost, and a purely kinematic character should not
    // pay it. Physics is only registered when it is actually wanted.
    let ragdoll_config = RagdollConfig::from_args();
    app.insert_resource(ragdoll_config);
    if ragdoll_config.enabled {
        // Twelve substeps, not avian's six: at six a fallen `character.glb`
        // never came to rest. Its resting contacts jittered past the sleep
        // bound (`ragdoll_plugin::FALLEN_SLEEP`) and it crept 7 mm/s across
        // the floor for as long as it lay there; at twelve it sleeps.
        app.add_plugins((PhysicsPlugins::default(), AnimRagdollPlugin))
            .insert_resource(avian3d::prelude::SubstepCount(12))
            .add_systems(Startup, spawn_physics_floor)
            .add_systems(
                Update,
                (
                    attach_ragdoll.after(walker::attach_walkers),
                    draw_ragdoll_gizmos,
                    deliver_ragdoll_hits.before(RagdollSet::Hit),
                    ask_to_fall.before(WalkerSet::Drive),
                    stand_when_asked.before(RagdollSet::Hit),
                ),
            );
    }

    app.run();
}
