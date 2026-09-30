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

use std::collections::HashMap;
use std::f32::consts::TAU;

use bevy::camera::{Exposure, Hdr};
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::diagnostic::{FrameCount, FrameTimeDiagnosticsPlugin};
use bevy::gltf::GltfAssetLabel;
use bevy::math::EulerRot;
use bevy::prelude::*;
use bevy::remote::http::RemoteHttpPlugin;
use bevy::remote::RemotePlugin;
use bevy::render::view::window::screenshot::{save_to_disk, Screenshot};
use bevy::world_serialization::WorldAssetRoot;
use bevy_egui::{egui, EguiContexts, EguiPlugin, EguiPrimaryContextPass};

use migera::character::{Bone, BoneMarker, HumanoidSkeleton};
use migera::character::anim::poses as anim_poses;
use migera::character::anim::asset::AnimAssetPlugin;
use migera::character::anim::balance;
use migera::character::anim::facing;
use migera::character::anim::gait::{cycle_of, walk_pose_on, GaitParams};
use migera::character::anim::locomotion;
use migera::character::anim::lookat;
use migera::character::anim::transition;
use migera::character::anim::rig::{LocalPose, RigGeometry};
use migera::character::anim::stance::{stance_on, stance_on_rig, DEFAULT_KNEE_FLEX};
use migera::character::anim::ground::{FlatGround, SlopedGround};
use migera::character::anim::phase::{GaitPhase, PhaseLayer};
use migera::character::anim::plugin::{AnimArmIk, AnimFootIk, AnimGround, AnimPose, AnimSet};
use avian3d::prelude::PhysicsPlugins;
use migera::character::anim::{
    spawn_ragdoll, AnimPhaseLayer, AnimPlugin, AnimRagdollPlugin, AnimSprings, AnimTarget,
    AnimTargetAsset, Ragdoll, RagdollHit, RagdollSet, RagdollSpawnConfig, FALL_DAMPING, FALL_TONE,
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

    fn look_at(self) -> Vec3 {
        match self {
            ViewPreset::Top => Vec3::ZERO,
            _ => Vec3::new(0.0, 0.95, 0.0),
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
                _ => {}
            }
            i += 1;
        }
        cfg
    }

    fn preset_transform(&self) -> Transform {
        Transform::from_translation(self.preset.eye_position(self.preset_distance, self.look_at_height))
            .looking_at(self.preset.look_at(), self.preset.up())
    }
}

fn camera_controller(time: Res<Time>, cfg: Res<CameraConfig>, mut cams: Query<&mut Transform, With<Camera3d>>) {
    match cfg.mode {
        CameraMode::Preset => {
            let transform = cfg.preset_transform();
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

fn spawn_ground(mut commands: Commands, mut meshes: ResMut<Assets<Mesh>>, mut materials: ResMut<Assets<StandardMaterial>>) {
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::default().mesh().size(10.0, 10.0))),
        MeshMaterial3d(materials.add(StandardMaterial { base_color: Color::srgb(0.3, 0.32, 0.35), perceptual_roughness: 0.9, ..default() })),
        Transform::IDENTITY,
    ));
}

/// Tags the root entity of the real skinned glTF character so
/// [`apply_real_mesh_visibility`] can find it without re-loading the
/// asset or walking the scene graph.
#[derive(Component)]
struct RealMeshRoot;

/// The asset's own facing correction, kept so the heading can be COMPOSED
/// onto it rather than overwriting it.
///
/// # Why this has to be a component and not just the spawn rotation
///
/// `spawn_real_mesh` puts the correction (default 180°) straight into this
/// root's `Transform::rotation`, which was sufficient while the rendered
/// mesh and the animated skeleton were two separate entities: root motion
/// wrote the heading onto the debug skeleton and left the mesh alone.
///
/// When the debug-capsule skeleton was removed and `HumanoidSkeleton` moved
/// onto this same mesh root, `drive_walk_cycle`'s
/// `root.rotation = facing.rotation()` silently became a CLOBBER — at yaw 0
/// `facing.rotation()` is the identity, so the correction survived exactly
/// until the first frame root motion ran. The comment defending that
/// assignment ("the model's own yaw correction lives on the mesh entity
/// this rig sits beside") went stale at the same moment and kept reading as
/// correct.
///
/// A/B measured against the unmodified parent commit, same flags
/// (`--anim-speed 1.2`), reading this root's own `Transform`:
///
/// ```text
///   before   rotation = (0, 0.0, 0, 1.0)   <- the correction, wiped
///   after    rotation = (0, 1.0, 0, ~0)    <- the 180-degree turn, kept
/// ```
///
/// In both cases the character travels toward `-Z`. With the correction
/// gone the mesh geometry faces `+Z` while travelling `-Z` — it walks
/// backward, which is what makes this visible.
///
/// # What this is NOT
///
/// It is not a knee fix. The same A/B shows the knee bending correctly
/// either way: sampled across a full stride on both builds, the knee sits
/// 0.140-0.154 m AHEAD of the hip-to-ankle line, zero backward samples.
/// A "backward-bending knee" reading that appears here is almost certainly
/// a stale `character_gallery` process still holding port 15702 — kill
/// every instance before trusting a BRP number.
#[derive(Component, Debug, Clone, Copy)]
struct FacingCorrection(Quat);

/// Loads `assets/models/puppet_base.gltf` (a real, CC0-licensed skinned
/// humanoid — Quaternius "Superhero Male") as a plain Bevy world-asset
/// root. Bevy's own glTF importer handles the skin/joint-matrix machinery
/// internally once this asset is spawned; [`build_real_mesh_skeleton`]
/// then binds its joint nodes to this crate's own `Bone` enum so
/// `character::anim` drives the skin live.
///
/// The binding is not trivial: the glTF's own bone names — `pelvis`,
/// `thigh_l`, `upperarm_l`, an Unreal-Mannequin-style convention — do not
/// match this crate's Mixamo-style names at all, hence
/// [`UE_MANNEQUIN_BONE_NAMES`] and [`resolve_bone_node_name`], and hence
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
fn spawn_real_mesh(mut commands: Commands, asset_server: Res<AssetServer>, model_cfg: Res<CharacterModelConfig>) {
    commands.spawn((
        WorldAssetRoot(asset_server.load(GltfAssetLabel::Scene(0).from_asset(model_cfg.path.clone()))),
        // A yaw correction (default 180°, `--character-yaw-correction` to
        // override for a differently-facing asset) -- this DEFAULT asset's
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
        Transform::from_rotation(Quat::from_rotation_y(model_cfg.yaw_correction_radians)),
        // Recorded as well as applied, because root motion writes this same
        // `Transform::rotation` every frame and needs something to compose
        // ONTO. See [`FacingCorrection`] for the two symptoms that appeared
        // when it had nothing.
        FacingCorrection(Quat::from_rotation_y(model_cfg.yaw_correction_radians)),
        RealMeshRoot,
    ));
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

/// This crate's own `Bone` enum name -> the standard Unreal Engine
/// Mannequin skeleton's own joint name for that same joint (`pelvis`,
/// `clavicle_l`, `upperarm_l`, `lowerarm_l`, `hand_l`, `thigh_l`,
/// `calf_l`, `foot_l`, `ball_l`, `spine_01/02/03`, `neck_01`, `Head`, ...)
/// — a REAL, documented Epic-published naming standard (used by UE4/UE5's
/// own default Mannequin and widely reused by third-party/marketplace
/// humanoid rigs that target Unreal), not specific to any one asset.
/// Originally verified DIRECTLY against `puppet_base.gltf` using the
/// `gltf` crate's own parser (a throwaway diagnostic binary dumped every
/// node's parent-child structure and each skin's own `joints` list), not
/// by eyeballing/manually counting the raw JSON (an earlier attempt at
/// that manual approach produced a WRONG guess — "`calf_l` is `pelvis`'s
/// direct child" — that the verified dump disproved: `pelvis`'s real
/// children are `spine_01`, `thigh_l`, `thigh_r`, and `calf_l` is
/// `thigh_l`'s own child, exactly as a normal humanoid rig's topology
/// would suggest); that asset happens to follow this standard exactly, so
/// this table generalizes directly to any other UE-Mannequin-rigged
/// asset, not just that one file.
///
/// Tried by [`resolve_bone_node_name`] as a proper naming CONVENTION
/// (same standing as the Mixamo convention there), not merely a per-asset
/// override — everything downstream (`character::anim`'s whole stack)
/// keeps working against this crate's own `Bone` enum unchanged
/// regardless of which convention actually matched.
const UE_MANNEQUIN_BONE_NAMES: [(Bone, &str); 22] = [
    (Bone::Hips, "pelvis"),
    (Bone::Spine, "spine_01"),
    (Bone::Spine1, "spine_02"),
    (Bone::Spine2, "spine_03"),
    (Bone::Neck, "neck_01"),
    (Bone::Head, "Head"),
    (Bone::LeftShoulder, "clavicle_l"),
    (Bone::LeftArm, "upperarm_l"),
    (Bone::LeftForeArm, "lowerarm_l"),
    (Bone::LeftHand, "hand_l"),
    (Bone::RightShoulder, "clavicle_r"),
    (Bone::RightArm, "upperarm_r"),
    (Bone::RightForeArm, "lowerarm_r"),
    (Bone::RightHand, "hand_r"),
    (Bone::LeftUpLeg, "thigh_l"),
    (Bone::LeftLeg, "calf_l"),
    (Bone::LeftFoot, "foot_l"),
    (Bone::LeftToeBase, "ball_l"),
    (Bone::RightUpLeg, "thigh_r"),
    (Bone::RightLeg, "calf_r"),
    (Bone::RightFoot, "foot_r"),
    (Bone::RightToeBase, "ball_r"),
];

/// Finds `bone`'s own real glTF node name among `descendants_by_name`'s
/// keys, trying several known real-rig naming CONVENTIONS in order — lets
/// a differently-named real mesh (a Mixamo-exported character, or a
/// UE-Mannequin-rigged asset from a different source than `puppet_base.
/// gltf`) get picked up automatically via `--character-model`, without
/// anyone first reading its raw glTF JSON and hand-writing a 22-entry
/// mapping table the way [`UE_MANNEQUIN_BONE_NAMES`] itself was
/// originally built (see that table's own doc comment on how much manual,
/// verified work that took).
///
/// Tries, in order:
/// 1. **Mixamo convention** (`mixamorig:LeftArm`, `mixamorig:Hips`, ...) —
///    this crate's own `Bone::name()` already returns exactly Mixamo's own
///    per-bone name (minus the `mixamorig:` prefix; see `skeleton.rs`'s
///    own module doc comment on why this crate's bone set was built to
///    match the Mixamo standard in the first place), so this is a direct
///    `format!("mixamorig:{}", bone.name())` lookup — the cheapest
///    possible match for the most common source of free/marketplace
///    rigged humanoids.
/// 2. **Bare Mixamo name** (`LeftArm`, `Hips`, ...) — some exporters strip
///    the `mixamorig:` prefix (or the file was re-exported through a tool
///    that does), so try `bone.name()` directly too before moving on.
/// 3. **UE Mannequin convention** ([`UE_MANNEQUIN_BONE_NAMES`],
///    `pelvis`/`upperarm_l`/`thigh_l`/...) — Epic's own standard naming,
///    reused by many third-party/marketplace humanoid rigs that target
///    Unreal, not specific to `puppet_base.gltf`.
fn resolve_bone_node_name<'a>(bone: Bone, descendants_by_name: &HashMap<&'a str, (Entity, Quat, Vec3)>) -> Option<&'a str> {
    let mixamo_prefixed = format!("mixamorig:{}", bone.name());
    if let Some((&found_name, _)) = descendants_by_name.get_key_value(mixamo_prefixed.as_str()) {
        return Some(found_name);
    }
    if let Some((&found_name, _)) = descendants_by_name.get_key_value(bone.name()) {
        return Some(found_name);
    }
    // Each convention table's own name is only a CANDIDATE -- still needs
    // confirming it's actually among `descendants_by_name`'s real keys
    // (both because the scene may not have finished spawning yet this
    // frame, the same "try again next frame" case every other branch
    // already handles, and because `get_key_value` is what hands back a
    // `&'a str` actually borrowed from `descendants_by_name` itself, never
    // a convention table's own `&'static str` -- returning a table's
    // literal directly here previously caused a real, live panic: an
    // index into `descendants_by_name` using a name this function claimed
    // to have "resolved" but never actually confirmed present).
    let ue_mannequin_name = UE_MANNEQUIN_BONE_NAMES.iter().find(|&&(b, _)| b == bone).map(|&(_, name)| name)?;
    descendants_by_name.get_key_value(ue_mannequin_name).map(|(&found_name, _)| found_name)
}

/// `true` once [`build_real_mesh_skeleton`] has already built the real
/// mesh's own [`HumanoidSkeleton`] — a plain bool `Local`, not a
/// `Without<HumanoidSkeleton>` query filter, since re-running the lookup
/// every frame before the scene finishes loading is cheap but pointless
/// churn, and the CLEANEST one-shot guard here is just "did this already
/// succeed."
///
/// Polls every frame (rather than a single `Startup` attempt) because
/// `WorldAssetRoot`'s own scene load is ASYNCHRONOUS — its child entities
/// (named `pelvis`, `thigh_l`, etc.) don't exist yet the frame
/// `spawn_real_mesh` runs, only once the glTF asset finishes loading and
/// `bevy_world_serialization` actually instantiates it, which can take
/// several frames.
fn build_real_mesh_skeleton(
    mut commands: Commands,
    mut already_built: Local<bool>,
    roots: Query<Entity, With<RealMeshRoot>>,
    named: Query<(Entity, &Name, &Transform)>,
    children_of: Query<&Children>,
    child_of: Query<&ChildOf>,
    global_transforms: Query<&GlobalTransform>,
) {
    if *already_built {
        return;
    }
    let Ok(root) = roots.single() else { return };

    // Collect every named descendant of `root` (the glTF importer names
    // spawned nodes after their own glTF node name — see `bevy_gltf`'s
    // own loader), so bones can be found by name regardless of how deep
    // Bevy nests them under `root` (there's an intermediate `Armature`
    // node above `pelvis` in this specific file, per the verified dump).
    // Captures each node's own REST LOCAL ROTATION *and* TRANSLATION
    // DIRECTION -- `HumanoidSkeleton` needs both: the rotation to seed
    // `Bone::Hips`'s own accumulated rotation correctly (this glTF's own
    // joints, unlike this crate's own T-pose-built debug skeleton, have
    // real, non-identity bind-pose rotations of their own — e.g.
    // `pelvis`'s own rest rotation is a genuine ~106.6° turn, a normal
    // Blender-glTF-export artifact, verified directly against the raw
    // glTF JSON, not assumed) and the translation direction so `apply_
    // solved_sim_to_skeleton`'s swing computation can stay entirely
    // self-consistent in THIS rig's own frame (see that function's own
    // doc comment for why reusing this crate's own T-pose direction
    // constant for a different rig's swing computation is the wrong,
    // fragile approach an earlier attempt used — live-verified to render
    // arms pointing up over the head instead of down).
    let mut descendants_by_name: HashMap<&str, (Entity, Quat, Vec3)> = HashMap::new();
    let mut stack = vec![root];
    while let Some(entity) = stack.pop() {
        if let Ok(children) = children_of.get(entity) {
            stack.extend(children.iter());
        }
        if let Ok((found_entity, name, transform)) = named.get(entity) {
            descendants_by_name.insert(name.as_str(), (found_entity, transform.rotation, transform.translation));
        }
    }

    let mut bones = HashMap::new();
    let mut rest_rotations = HashMap::new();
    let mut rest_directions = HashMap::new();
    for &bone in &Bone::ALL {
        let Some(node_name) = resolve_bone_node_name(bone, &descendants_by_name) else {
            return; // Not all joints have spawned yet this frame -- try again next frame.
        };
        let &(entity, rest_rotation, rest_translation) = &descendants_by_name[node_name];
        bones.insert(bone, entity);
        rest_rotations.insert(bone, rest_rotation);
        rest_directions.insert(bone, rest_translation.normalize_or_zero());
    }

    // `Bone::Hips`'s own mapped joint (`pelvis`) needs its OWN rest
    // local translation, its PARENT's rest world rotation, and its own
    // rest world position -- all captured HERE, before any animation
    // write ever runs, so `HumanoidSkeleton::hips_local_translation_for`
    // can correctly convert a solved world-space `Bone::Hips` position
    // into whatever local frame THIS specific rig's own hip joint
    // actually translates in (see that method's own doc comment for why
    // this crate's debug-skeleton-only "assign the world position
    // directly" assumption breaks for a real character whose root sits
    // under a non-identity parent chain).
    let hips_entity = bones[&Bone::Hips];
    let Ok(hips_local_transform) = named.get(hips_entity).map(|(_, _, t)| *t) else { return };
    let Ok(hips_parent) = child_of.get(hips_entity) else { return };
    let Ok(hips_parent_global) = global_transforms.get(hips_parent.parent()) else { return };

    // `Bone::Hips.t_pose_world_position()` -- NOT this glTF's own scene-
    // space world position -- is the ANIMATION's own rest reference: a
    // solved root translation is always expressed in this crate's own
    // synthetic T-pose coordinate convention, regardless of which
    // skeleton is being driven. The delta
    // `HumanoidSkeleton::hips_local_translation_for` needs is "how far
    // has the solved hip moved since ITS OWN rest", which is only
    // meaningful relative to that synthetic rest position -- not this
    // glTF's own, numerically-similar-but-semantically-different rest
    // position in its own scene's coordinate frame.
    let skeleton = HumanoidSkeleton::for_other_rig(
        bones,
        rest_rotations,
        rest_directions,
        hips_local_transform.translation,
        hips_parent_global.rotation(),
        hips_parent_global.scale(),
        Bone::Hips.t_pose_world_position(),
    );
    if std::env::var("MIGERA_DUMP_REST").is_ok() {
        for &b in Bone::ALL.iter() {
            eprintln!(
                "DUMP {:?} rest_rotation={:?} rest_direction={:?}",
                b,
                skeleton.rest_rotation(b),
                skeleton.rest_direction(b)
            );
        }
    }

    // `BoneMarker` on each real joint entity -- lets `draw_joint_axis_gizmos`/
    // `skeleton_debug_lines` (both `Query<..., With<BoneMarker>>`) find
    // these same real mesh joints directly, exactly like they already did
    // for the (now-removed) separate debug-capsule skeleton, with no query
    // rewrite needed beyond this one insert per bone.
    for (bone, entity) in skeleton.iter() {
        commands.entity(entity).insert(BoneMarker(bone));
    }

    commands.entity(root).insert(skeleton);
    *already_built = true;
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

/// Attaches the `anim` stack's components once the skeleton exists.
///
/// Mirrors `build_real_mesh_skeleton`'s own deferred structure: the rig is
/// spawned asynchronously from a glTF, so this waits for the skeleton to
/// appear rather than assuming it is there at startup.
fn attach_anim_backend(
    mut commands: Commands,
    choice: Res<AnimPoseChoice>,
    idle: Res<AnimIdleConfig>,
    asset_server: Res<AssetServer>,
    rigs: Query<Entity, (With<HumanoidSkeleton>, Without<AnimTarget>)>,
) {
    for entity in &rigs {
        // Seed with the compiled-in pose so the character is never a
        // T-posed mannequin for the frame or two the asset takes to load.
        let seed = anim_poses::by_name(&choice.0).unwrap_or_else(|| {
            warn!("unknown --anim-pose '{}', falling back to relaxed_stand", choice.0);
            anim_poses::relaxed_stand()
        });

        // ...then hand over to the asset, which is what makes editing
        // `assets/anim/<name>.pose.ron` update the running character.
        let handle = asset_server.load(format!("anim/{}.pose.ron", choice.0));

        commands.entity(entity).insert((
            AnimTarget::new(seed),
            AnimTargetAsset(handle),
            AnimSprings::default(),
            // Stage 2: continuous procedural motion on top of the authored
            // pose, so the character keeps breathing and shifting its
            // weight once the springs have settled.
            GaitPhase { speed: idle.speed, ..Default::default() },
            AnimPhaseLayer(if idle.speed > 0.0 {
                PhaseLayer::locomotion()
            } else {
                PhaseLayer::standing_idle()
            }),
            // Stage 3: plant the feet on whatever is underneath them.
            //
            // Enabled for walking as well as standing. It was briefly
            // disabled while the gait walked in place: the lock pins a
            // planted foot where the ANIMATION put it, and with no root
            // motion a walk cycle drags its own feet backward — measured at
            // ~0.25 m of accumulated offset, which read as the character
            // leaning forward over trailing feet.
            //
            // Root motion removes that drag by construction (the published
            // velocity cancels the stance foot's relative travel), so the
            // lock now has only the real residual to absorb, which is its
            // job.
            AnimFootIk::default(),
            GalleryLocomotion::default(),
            GalleryStride::default(),
            GalleryFacing::default(),
            GalleryTransition::default(),
            balance::Balance::default(),
            GalleryLook(match idle.look_at {
                Some(target) => lookat::LookAt::at(target),
                None => lookat::LookAt::forward(),
            }),
            GalleryReach(idle.reach),
            AnimArmIk::default(),
            AnimGround(if idle.slope == 0.0 {
                Box::new(FlatGround::default())
            } else {
                Box::new(SlopedGround { height: 0.0, grade: idle.slope })
            }),
        ));
    }
}

/// One character's root motion.
///
/// `Drive`: the gait publishes its velocity and turn, and the gallery moves
/// the body itself in [`ride_rendered_feet`], from the pose the springs
/// actually render. A game would hand the same displacement to its
/// character controller.
#[derive(Component)]
struct GalleryLocomotion(locomotion::Locomotion);

/// What [`ride_rendered_feet`] needs from this frame's gait, and the pose it
/// rendered last frame.
#[derive(Component, Default)]
struct GalleryStride {
    params: Option<GaitParams>,
    /// The gait cycle this frame, and last frame.
    cycle: f32,
    previous_cycle: f32,
    previous: Option<LocalPose>,
    /// How much of the gait is playing: 0 standing, 1 walking.
    weight: f32,
    /// How far a balance recovery step has just carried the character, in
    /// the pose's frame (`balance::Balance::travelled`): moved like root
    /// motion, then zeroed.
    stepped: Vec3,
}

/// One character's heading.
#[derive(Component, Default)]
struct GalleryFacing(facing::Facing);

/// One character's progress between standing and walking.
#[derive(Component, Default)]
struct GalleryTransition(transition::Transition);

/// What one character is looking at.
#[derive(Component, Default)]
struct GalleryLook(lookat::LookAt);

/// Where one character's left hand is reaching, in world space.
///
/// `None` leaves the arms to the animation. A world-space point rather than a
/// character-relative one so the target stays put while the character walks
/// past it, which is the case that makes a reach look like a reach.
#[derive(Component, Default)]
struct GalleryReach(Option<Vec3>);

/// Everything `drive_walk_cycle` reads and writes per character.
///
/// A named alias rather than an inline tuple: the walk needs the target
/// pose, the phase clock, the locomotion state, the heading, the
/// standing/walking blend, the foot IK (to hand it the frame's turn), the
/// transform (travel and facing), and the ground (height following).
type WalkingRig = (
    &'static mut AnimTarget,
    // Mutable so the speed reaches the leg clock — see `drive_walk_cycle`.
    &'static mut GaitPhase,
    &'static mut GalleryLocomotion,
    &'static mut GalleryFacing,
    &'static mut GalleryTransition,
    &'static mut GalleryLook,
    &'static GalleryReach,
    &'static mut AnimArmIk,
    &'static mut AnimFootIk,
    &'static mut Transform,
    // The asset's own facing correction, so the heading composes onto it
    // instead of overwriting it. `Option`, because a rig assembled without
    // `spawn_real_mesh` (the synthetic preview) has no correction to apply.
    Option<&'static FacingCorrection>,
    &'static mut GalleryStride,
    &'static mut AnimPhaseLayer,
    // Winter's standing pendulum: a push sways the body over its feet.
    &'static mut balance::Balance,
);

impl Default for GalleryLocomotion {
    fn default() -> Self {
        Self(locomotion::Locomotion { mode: locomotion::RootMotion::Drive, ..Default::default() })
    }
}

/// Moves each walking character by exactly how far its planted feet moved
/// under it in the pose just RENDERED — after the springs, before the IK.
///
/// Root motion used to integrate the gait's published velocity, measured on
/// the gait's TARGET pose, with the frame's `dt`. Two things broke that on
/// the measured walk, whose speed rises and falls through every stride: an
/// explicit step errs by `½·a·dt²`, up to 2 mm a frame; and the leg springs
/// lag the target by an amount that changes with the foot's speed — through
/// them a planted foot slid 39 mm a stance, headless. The rendered contact's
/// own displacement is exact through both (`locomotion::root_displacement_between`).
fn ride_rendered_feet(
    time: Res<Time>,
    mut rigs: Query<(
        &AnimPose,
        &mut GalleryLocomotion,
        &GalleryFacing,
        &mut GalleryStride,
        &mut AnimFootIk,
        &mut Transform,
        &AnimGround,
    )>,
) {
    for (pose, mut locomotion, facing, mut stride, mut foot_ik, mut root, ground) in &mut rigs {
        let now = pose.pose();
        let rig = foot_ik.rig.clone().unwrap_or_default();
        let moved = match (&stride.params, &stride.previous) {
            // Standing, the body sways its pelvis over feet that stay put
            // (`PhaseLayer::standing_idle`); read as root motion, that sway
            // walked the whole character — measured live, its planted feet
            // wandered 12 mm and the pelvis twice its sway.
            _ if stride.weight <= 0.0 => Vec3::ZERO,
            (Some(params), Some(previous)) => {
                // The cycle half-way through the frame decides which feet
                // are planted; the gait clock only ever runs forward.
                let middle = stride.previous_cycle
                    + 0.5 * (stride.cycle - stride.previous_cycle).rem_euclid(1.0);
                locomotion::root_displacement_between(previous, &now, middle, params, &rig)
                    .map(|moved| facing.0.rotation() * moved)
                    // No foot down — a run's flight: the body coasts.
                    .unwrap_or(locomotion.0.root_velocity * time.delta_secs())
            }
            _ => Vec3::ZERO,
        };
        // A balance recovery step carries the character too.
        let moved = moved + facing.0.rotation() * std::mem::take(&mut stride.stepped);
        locomotion.0.position += moved;
        // The foot locks keep a planted foot where it is in the WORLD only
        // if they know the body moved over it.
        foot_ik.turn.travel = moved;
        stride.previous = Some(now);
        stride.previous_cycle = stride.cycle;

        // Travel moves the ENTITY, not the pose's root translation — see
        // `drive_walk_cycle`.
        root.translation = locomotion.0.position;

        // Height comes from the ground, not from the published velocity.
        //
        // Root motion publishes travel ALONG the surface, which is all a
        // flat plane needs. On a slope, horizontal travel alone keeps the
        // character at its starting height: measured at 7.7 m underneath the
        // hillside after 26 m of a 0.3 grade. The entity's origin already
        // sits at ground level (the rig's own hips are above it), so the
        // surface height IS the origin height — `stand_height` is zero here.
        if let Some(height) =
            locomotion::ground_following_height(root.translation, 0.0, ground.0.as_ref())
        {
            root.translation.y = height;
        }
    }
}

/// Drives the walk cycle from the gait clock.
///
/// A gallery-local system rather than part of `AnimPlugin`: the plugin-side
/// wiring belongs with root motion, which is its own phase. This exists so
/// the cycle can be judged on a real rig now, which the synthetic preview
/// cannot do — `LeftLeg -> LeftFoot` is a 0.07 m stub there against a
/// 0.459 m shin on a real glTF, so knee flexion moves the sole six times
/// less and a correct walk renders with a visibly straight leg.
///
/// Runs in `AnimSet::Target`, so the phase layer composes its secondary
/// motion on top and the springs smooth the result — the same ordering the
/// plugin will use.
fn drive_walk_cycle(
    time: Res<Time>,
    idle: Res<AnimIdleConfig>,
    choice: Res<AnimPoseChoice>,
    mut pushes: ResMut<PushSchedule>,
    mut rigs: Query<WalkingRig>,
    // The stride the current speed's gait really takes, keyed by the speed
    // and whether the real rig has bound: measuring it costs a cycle of
    // root-motion samples, so it is redone only when either changes.
    mut stride: Local<Option<(u32, bool, f32)>>,
) {
    // A run above the threshold, a walk below. Real locomotion switches on
    // the same basis — a fast walk becomes a run at a speed where the
    // flight phase costs less than the cadence would.
    const RUN_ABOVE: f32 = 2.2;

    // Composed onto the AUTHORED pose, re-read every frame.
    //
    // Writing `target.pose = walk_pose(phase, &params, &target.pose)` is the
    // obvious form and it accumulates: each frame layers another cycle's
    // rotations onto the last frame's already-walked result, so the legs
    // wind up without bound. The base has to be a fixed reference.
    let base = anim_poses::by_name(&choice.0).unwrap_or_else(anim_poses::relaxed_stand);
    let due_pushes = pushes.due(time.elapsed_secs());

    for (
        mut target,
        mut phase,
        mut locomotion,
        mut facing,
        mut transition_state,
        mut look,
        reach,
        mut arm_ik,
        mut foot_ik,
        mut root,
        correction,
        mut gait,
        mut layer,
        mut balance,
    ) in &mut rigs
    {
        // `cycle_of` rather than `phase.gait`: the clock is in RADIANS and
        // `walk_pose` takes a cycle FRACTION. Passing the raw value runs the
        // legs through six cycles per stride, which shows up as both hips
        // sitting at nearly the same angle every frame — measured 11 and 12
        // degrees where they should be half a cycle apart.
        // The rig the gait is posed on: the real one once the glTF has
        // bound, the synthetic proxy for the first frames before it has.
        // Posing on anything else measures an animation the legs are not
        // playing — the gait's vertical motion is in fractions of THIS
        // rig's leg.
        let gait_rig = foot_ik.rig.clone().unwrap_or_default();

        // The transition first: it decides the speed the legs step at,
        // which through a stop's last step is the walk's, not the zero
        // asked for (Winter §11.3.3; see `transition`). While the character
        // stands it is told how the idle carries its weight, so a start
        // stands on the loaded leg.
        let idle_shift = PhaseLayer::standing_idle()
            .sway
            .map_or(0.0, |sway| sway.weight_shift(phase.elapsed));
        if transition_state.0.is_at_rest() {
            transition_state.0.idle_shift = idle_shift;
        }
        let config = transition::TransitionConfig {
            // Half the duty factor: the other leg's mid-swing, see
            // `TransitionConfig::mid_swing`.
            mid_swing: gait
                .params
                .map_or(transition::TransitionConfig::default().mid_swing, |p| p.duty_factor * 0.5),
            ..Default::default()
        };
        let event =
            transition_state.0.advance(idle.speed, cycle_of(&phase), &config, time.delta_secs());
        let speed = transition_state.0.stride_speed;

        // A walk's stride grows with its speed, scaled to this rig's leg;
        // see `GaitParams::walking_on`.
        let params = if speed >= RUN_ABOVE {
            GaitParams::running()
        } else {
            GaitParams::walking_on(speed, &gait_rig)
        };

        // The standing knee bend, applied HERE rather than baked into the
        // authored pose.
        //
        // It used to live in `relaxed_stand.pose.ron`, authored in the
        // synthetic rig's facing convention. A stored rotation cannot know
        // which rig it will drive, and `puppet_base.gltf` faces the
        // opposite way — so the baked bend rendered as a backward-bending
        // knee, and nothing downstream could correct it because the value
        // was already in the file. `stance_on_rig` derives the bend
        // direction from the rig's own measured geometry, so the knee is
        // right on any rig by construction.
        let stood = match &foot_ik.rig {
            Some(rig) => stance_on_rig(&base, DEFAULT_KNEE_FLEX, rig),
            None => stance_on(&base, DEFAULT_KNEE_FLEX),
        };
        // `--anim-pose getup:sit|squat|quadruped|half_kneel`: hold one get-up
        // key, to verify it on its own.
        let stood = match (choice.0.strip_prefix("getup:"), &foot_ik.rig) {
            (Some(name), Some(rig)) => {
                use migera::character::anim::getup::Key;
                let key = match name {
                    "sit" => Key::Sit,
                    "squat" => Key::Squat,
                    "quadruped" => Key::Quadruped,
                    _ => Key::HalfKneel,
                };
                key.pose(rig)
            }
            _ => stood,
        };

        // The speed has to reach the LEG clock, as the cadence that makes
        // this gait's stride travel at exactly this speed.
        //
        // It used to be set once, at spawn, so after the speed slider moved
        // the legs kept stepping at the launch speed's rhythm while the body
        // travelled at the new one — the planted feet slid by the
        // difference. And its `1/7 + 0.9·v` rate ignored the stride
        // entirely, which only worked while every speed shared one stride.
        let key = (speed.to_bits(), foot_ik.rig.is_some());
        let distance = match *stride {
            Some((speed, bound, distance)) if (speed, bound) == key => distance,
            _ => {
                let distance = locomotion::distance_per_cycle(&params, &stood, &gait_rig);
                *stride = Some((key.0, key.1, distance));
                distance
            }
        };
        if speed > 0.0 && distance > 1.0e-4 {
            phase.base_frequency_hz = 0.0;
            phase.speed_coefficient = 1.0 / distance;
        } else {
            // Standing: back to the clock's own idle sway.
            let defaults = GaitPhase::default();
            phase.base_frequency_hz = defaults.base_frequency_hz;
            phase.speed_coefficient = defaults.speed_coefficient;
        }
        if phase.speed != speed {
            phase.speed = speed;
        }
        match event {
            // The first step joins the walk at the swinging leg's
            // mid-swing. The gait has no weight yet, so moving its clock
            // moves nothing on screen.
            Some(transition::TransitionEvent::FirstStep { cycle }) => {
                phase.gait = cycle * TAU;
            }
            // Back at rest: the idle's weight-shift schedule starts over, so
            // the character stands square a while before shifting, rather
            // than dropping into whatever shift the clock had reached.
            Some(transition::TransitionEvent::AtRest) => phase.elapsed = 0.0,
            None => {}
        }
        let cycle = cycle_of(&phase);

        // How much of the gait applies. At zero weight the character holds
        // its standing pose — no gait, no residual stepping.
        let weight = transition_state.0.weight;

        // The standing idle sways the pelvis over planted feet; a walk's
        // body motion comes from its legs. Between the two, each layer's
        // oscillators fade with the gait's weight, and the idle's sway runs
        // only at rest: while a first step is prepared the release carries
        // the weight instead.
        let mut wanted = PhaseLayer::between(&PhaseLayer::standing_idle(), &PhaseLayer::locomotion(), weight);
        if !transition_state.0.is_at_rest() {
            wanted.sway = None;
        } else if let Some(sway) = wanted.sway.as_mut() {
            // Eased back in after coming to rest (the idle clock restarts
            // there): switched on at once it ticked the pelvis 6 mm
            // sideways in one frame, measured live at the end of a stop.
            const SETTLE_SECONDS: f32 = 1.5;
            let t = (phase.elapsed / SETTLE_SECONDS).clamp(0.0, 1.0);
            let settled = t * t * (3.0 - 2.0 * t);
            sway.lateral *= settled;
            sway.fore_aft *= settled;
        }
        if layer.0 != wanted {
            layer.0 = wanted;
        }

        // The pose the character is rendered in, as a function of phase —
        // ONE definition, used both to pose it below and to derive its root
        // motion, so the two cannot disagree. They did, three ways; see
        // `locomotion::root_velocity_of`.
        //
        // Posed on the rig the IK stage actually solves against, not the
        // synthetic proxy: the gait's vertical amplitudes are fractions of
        // leg length, and this rig's leg is nearly twice the proxy's. Falls
        // back to the proxy for the first frames, before the glTF has
        // finished binding.
        //
        // Blended from `stood`, not from the authored base: at zero weight
        // the character stands in `stood`, and blending from anything else
        // popped the standing knee bend in and out as a walk began or ended.
        //
        // The release before a first step is posed on the standing side of
        // the blend, so the walk fades it out as it fades in. The walk
        // itself is still built on `stood`: its cycle is memoized per base
        // pose, and the release changes every frame.
        let mut prepared = stood;
        transition_state.0.apply_release(&mut prepared, &gait_rig);
        // A stop's last swing is set down onto where it will stand, judged
        // by the foot IK on the rendered foot (`AnimFootIk::landing`).
        foot_ik.landing = transition_state.0.landing(&prepared, &gait_rig);
        // A push sways the standing body over its feet and it recovers
        // (Winter's inverted pendulum, `balance`), stepping if it must.
        // Posed on the standing side of the blend, so a walk starting
        // mid-sway fades it out.
        for &push in &due_pushes {
            balance.push(push);
        }
        if !balance.is_settled(1.0e-5) {
            let support = balance::Support::of(&stood, &gait_rig);
            balance.step(&support, balance::pendulum_k(&stood, &gait_rig), time.delta_secs());
            // A recovery step done: the character moves by it (with root
            // motion, in `ride_rendered_feet`), and the feet stand where
            // they are now.
            if let Some(by) = balance.travelled {
                gait.stepped += gait_rig.forward() * by.x + gait_rig.left() * by.y;
                balance.rebase();
            }
            // The stepping foot is set down onto its spot by the foot IK.
            if foot_ik.landing.is_none()
                && let Some((left, spot, strength)) = balance.landing_spot(&prepared, &gait_rig)
            {
                foot_ik.landing = Some(migera::character::anim::plugin::Landing { left, spot, strength });
            }
            balance.apply(&mut prepared, &gait_rig);
        }
        // The feet the balance has down stay locked however its sprung legs
        // lag a stumbling body (`AnimFootIk::planted`); a walk's feet are
        // the locks' own call.
        foot_ik.planted = if weight <= 0.0 && !balance.is_settled(1.0e-5) { balance.planted() } else { [false; 2] };
        let rendered = |cycle: f32| {
            if weight <= 0.0 {
                prepared
            } else {
                let walking = walk_pose_on(cycle, &params, &stood, &gait_rig);
                transition_state.0.blend(&prepared, &walking, &gait_rig)
            }
        };
        // Clippy misses the second use: root motion reads `&rendered` below.
        #[allow(clippy::redundant_closure_call)]
        {
            target.pose = rendered(cycle);
        }

        // The look is composed AFTER the gait, so a walking character can
        // also be looking somewhere — the two are independent.
        if let Some(direction) = look.0.advance(
            lookat::head_position(&target.pose, &RigGeometry::default())
                + root.translation,
            facing.0.rotation(),
            &lookat::LookAtConfig::default(),
            time.delta_secs(),
        ) {
            lookat::apply(
                &mut target.pose,
                direction,
                &lookat::LookAtConfig::default(),
                &RigGeometry::default(),
            );
        }

        // The reach is NOT applied here.
        //
        // An arm target is a point in the world, and solving for one needs the
        // character's real rig geometry — which this system does not have. It
        // would be solving against `RigGeometry::default()`, the synthetic
        // T-pose proxy, whose left shoulder sits at x = -0.300 where the real
        // rig's is at x = +0.212. The two are MIRRORED, so a world target
        // solved here and retargeted sends the hand to the wrong side of the
        // body: measured, a target at (0.45, 1.15, -0.30) put the left hand at
        // (-0.197, 1.208, +0.147).
        //
        // So the reach goes into `AnimArmIk` and the plugin solves it in
        // `AnimSet::Ik`, where the live bone transforms are available. Same
        // reason foot IK lives there rather than in a caller.
        arm_ik.left = reach.0;

        // The root velocity the gait is asking for, and — in
        // `Authoritative` mode — the travel it produces. The gallery has no
        // character controller, so it integrates the request itself.
        // `--anim-turn RADIANS_PER_SECOND` walks a steady circle, which is
        // what makes turning visible: a character that turns and then walks
        // straight looks the same as one that never turned.
        if idle.turn != 0.0 {
            facing.0.target_yaw = facing::shortest_angle(
                facing.0.yaw + idle.turn.signum() * std::f32::consts::FRAC_PI_2,
            );
            facing.0.turn_rate = idle.turn.abs();
        }

        let turn = locomotion::advance_turning_with(
            &mut locomotion.0,
            &mut facing.0,
            cycle,
            // The rate `cycle` ACTUALLY advances at — the leg clock's own.
            //
            // This used to be the transition's smoothed cadence, `0.9·v`,
            // chosen because the raw clock "steps 8.6x when a character
            // stops and never reaches zero, so root motion would lurch and
            // creep forever". Both came from pairing a rate with a pose it
            // did not describe. Measured on the RENDERED pose, a fading
            // blend shrinks the foot's travel with it, and at zero weight
            // the pose is the constant `stood`, whose velocity is exactly
            // zero — nothing creeps. Pairing the smoothed rate with legs
            // cycling at `1/7 + 0.9·v` slid the planted foot 14% at 1 m/s.
            phase.gait_frequency_hz(),
            &params,
            &rendered,
            // The same rig the gait was posed on, for the same reason.
            // Root motion is derived from how far the stance foot travels
            // under the body, so measuring a different rig's pose publishes
            // a velocity for an animation the legs are not playing.
            &gait_rig,
            time.delta_secs(),
        );

        // The foot locks need the same frame's turn, so a planted foot
        // pivots with the body rather than being dragged sideways by it.
        foot_ik.turn = turn;

        // The body itself is moved after the springs, from the pose they
        // render: see `ride_rendered_feet`.
        //
        // Travel moves the ENTITY, not the pose's root translation.
        // `LocalPose::root_translation` is routed through
        // `hips_root_rotation` and divided by the rig's parent scale, both
        // of which are right for a hip displacement expressed in the rig's
        // own bind frame — and wrong for world travel. On this Z-up rig that
        // rotation maps the pose's forward (`-Z`) onto world `-Y`, so the
        // character walked STRAIGHT DOWN: measured sinking at 0.69 m/s
        // against a published speed of 0.72.
        gait.params = Some(params);
        gait.cycle = cycle;
        gait.weight = weight;

        // The entity turns to match the heading. Without this the character
        // TRAVELS along its heading while still pointing forward — walking
        // sideways, which is a stranger failure than not turning at all.
        //
        // COMPOSED onto the asset's own facing correction, never assigned
        // over it. This line used to be a bare assignment, justified by a
        // comment saying the correction lived on a separate mesh entity —
        // true once, and stale from the moment the debug-capsule skeleton
        // was removed and `HumanoidSkeleton` moved onto the mesh root
        // itself. After that the assignment wiped a 180° correction every
        // frame, which rendered as walking backward on grasshopper knees.
        // See [`FacingCorrection`] for the measurements.
        //
        // Heading first, then the correction: the correction turns the
        // asset's own geometry onto this crate's `-Z`-forward convention
        // (a fact about the model), and the heading then turns the
        // already-corrected character in the world. Reversing the order
        // yaws about the uncorrected axis and sends a turning character
        // along a heading 180° off its facing.
        root.rotation = facing.0.rotation()
            * correction.map_or(Quat::IDENTITY, |correction| correction.0);
    }
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

/// H2: lets a ragdolled character fall when its balance finds no step
/// that catches it (`Balance::falls`), on `F`, or at `--fall-at-frame`.
/// The balance is reset: the body is the physics' now, and a stumble still
/// being posed underneath would move the character entity too.
fn fall_when_uncaught(
    config: Res<RagdollConfig>,
    frame: Res<FrameCount>,
    keys: Res<ButtonInput<KeyCode>>,
    mut rigs: Query<(&mut balance::Balance, &mut Ragdoll, &mut AnimFootIk, &GalleryFacing)>,
) {
    let asked = config.fall_at_frame == Some(frame.0) || keys.just_pressed(KeyCode::KeyF);
    for (mut balance, mut ragdoll, mut foot_ik, facing) in &mut rigs {
        if ragdoll.is_falling() || !(asked || balance.falls) {
            continue;
        }
        info!(
            "character_gallery: falling at frame {} ({})",
            frame.0,
            if balance.falls { format!("a push asked for a {:.2} m step", balance.wanted_step) } else { "asked".into() }
        );
        // The push goes with it, the part not yet delivered too: judged at
        // the push's start, the body has barely moved.
        let pushed = balance.velocity + balance.pending_push();
        let launch = foot_ik.rig.as_ref().map_or(Vec3::ZERO, |rig| {
            facing.0.rotation() * (rig.forward() * pushed.x + rig.left() * pushed.y)
        });
        ragdoll.fall_moving(FALL_TONE, config.fall_damping, launch);
        *balance = balance::Balance::default();
        foot_ik.planted = [false; 2];
        foot_ik.landing = None;
    }
}

/// H3: a fallen character that has come to rest lies for `GETUP_DELAY` (a
/// choice), then rises through the get-up keys for how it lies
/// (`Ragdoll::get_up`). The turn the rise asks for, to face the way it
/// gets up, is applied to the gallery's own heading: the character's
/// rotation is `GalleryFacing`'s to write.
fn get_up_when_rested(mut ragdolls: Query<(&mut Ragdoll, &mut GalleryFacing)>, frame: Res<FrameCount>) {
    const GETUP_DELAY: f32 = 1.0;
    for (mut ragdoll, mut facing) in &mut ragdolls {
        if ragdoll.fall.is_some_and(|fall| fall.at_rest && fall.rise.is_none()) && ragdoll.get_up(GETUP_DELAY) {
            info!("character_gallery: at rest at frame {}, getting up", frame.0);
        }
        if let Some(rise) = ragdoll.fall.as_mut().and_then(|fall| fall.rise.as_mut())
            && rise.turn_pending
        {
            facing.0.yaw += rise.turn;
            facing.0.target_yaw = facing.0.yaw;
            rise.turn_pending = false;
            info!("character_gallery: lying {:?}, turning {:.0}° to get up", rise.lying, rise.turn.to_degrees());
        }
    }
}

/// While the ragdoll falls it moves the character entity after its body
/// (`AnimRagdollPlugin`); the gallery's own position has to take that up,
/// or `ride_rendered_feet` writes the old one back the moment the body stops
/// being followed. It did: every rise slid the character 0.45 m back to
/// where it had fallen from.
fn follow_the_fallen_body(mut rigs: Query<(&Ragdoll, &Transform, &mut GalleryLocomotion)>) {
    for (ragdoll, transform, mut locomotion) in &mut rigs {
        if ragdoll.is_falling() {
            locomotion.0.position.x = transform.translation.x;
            locomotion.0.position.z = transform.translation.z;
        }
    }
}

/// The floor a falling ragdoll lands on: the rendered ground is only a
/// mesh.
fn spawn_physics_floor(mut commands: Commands) {
    use avian3d::prelude::{Collider, Friction, RigidBody};
    commands.spawn((RigidBody::Static, Collider::half_space(Vec3::Y), Friction::new(1.0), Transform::default()));
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

fn main() {
    let assets = std::env::current_dir().expect("cwd").join("assets").to_string_lossy().into_owned();

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
        .add_systems(Update, (camera_controller, draw_world_axis_gizmos, draw_joint_axis_gizmos, update_camera_hud, apply_real_mesh_visibility, build_real_mesh_skeleton))
        .add_systems(Update, attach_anim_backend.after(build_real_mesh_skeleton))
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

    // The rotation-space stack is now the only animation backend. The
    // superseded position-space `muscle` module — and with it the
    // `--anim-backend` A/B switch that carried the cutover — was deleted
    // once this path had been the default for a full phase.
    app.add_plugins((AnimPlugin, AnimAssetPlugin))
        .add_systems(
            Update,
            (draw_skeleton_debug_gizmos, update_skeleton_hud, log_debug_stats),
        )
        // In `Target`, so the phase layer composes onto the walking pose and
        // the springs smooth it — the ordering the plugin will use once
        // locomotion lands.
        .add_systems(
            Update,
            (follow_speed_schedule, drive_walk_cycle).chain().in_set(AnimSet::Target),
        )
        // After the springs have rendered this frame's pose, before the IK
        // plants the feet against the ground at the body's new position.
        .add_systems(Update, ride_rendered_feet.after(AnimSet::Spring).before(AnimSet::Ik));

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
                    attach_ragdoll.after(attach_anim_backend),
                    draw_ragdoll_gizmos,
                    deliver_ragdoll_hits.before(RagdollSet::Hit),
                    fall_when_uncaught.after(drive_walk_cycle).before(RagdollSet::Hit),
                    get_up_when_rested.before(RagdollSet::Hit),
                    follow_the_fallen_body.before(ride_rendered_feet),
                ),
            );
    }

    app.run();
}
