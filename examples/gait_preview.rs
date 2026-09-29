//! Renders the walk cycle from [`character::anim::gait`], with nothing else
//! in the way.
//!
//! Phase 1's visual gate. The gait is a pure function from a cycle position
//! to a pose, so this drives it directly from a clock and writes the result
//! onto the skeleton — no springs, no IK, no ground adaptation, no phase
//! layer. If the walk looks wrong here, it is the curves; anything further
//! up the stack can only be ruled out by removing it.
//!
//! ```text
//! cargo run --release --example gait_preview -- \
//!     --phase 0.75 --camera left --shot /tmp/swing.png
//! ```
//!
//! `--phase` freezes the cycle at one position, which is what makes a
//! screenshot comparable against the measured shape table. Without it the
//! cycle runs at `--cadence` strides per second.
//!
//! # What this preview cannot show
//!
//! It renders the **synthetic** T-pose rig, whose leg segments are shifted a
//! joint from every real rig's: `LeftLeg -> LeftFoot` is a 0.07 m stub here
//! against a 0.459 m shin on `puppet_base.gltf`. So 42 degrees of knee
//! flexion displaces the sole by 0.050 m here and the ankle by 0.329 m
//! there — **6x** — and a perfectly correct walk cycle renders with a
//! visibly straight leg.
//!
//! Use this to judge the hip swing, the foot's forward travel, the arm
//! counter-swing and the ground clearance. Do **not** conclude anything
//! about knee bend from it; that needs a real rig
//! (`character_gallery --real-mesh`).

use bevy::prelude::*;

use std::collections::HashMap;

use migera::character::anim::gait::{walk_pose, GaitParams};
use migera::character::anim::retarget::write_pose_to_skeleton;
use migera::character::anim::rig::LocalPose;
use migera::character::anim::stance::stance;
use migera::character::skeleton::{Bone, HumanoidSkeleton};

fn main() {
    let config = PreviewConfig::from_args();

    let mut app = App::new();
    app.add_plugins(DefaultPlugins.set(WindowPlugin {
        primary_window: Some(Window {
            title: "gait preview".into(),
            resolution: bevy::window::WindowResolution::new(1280, 720),
            ..default()
        }),
        ..default()
    }))
    .insert_resource(config)
    .add_systems(Startup, setup)
    .add_systems(Update, (drive_gait, draw_bones, shoot).chain());

    app.run();
}

#[derive(Resource, Clone)]
struct PreviewConfig {
    /// Freeze the cycle here instead of running it.
    frozen_phase: Option<f32>,
    /// Strides per second when not frozen.
    cadence: f32,
    /// Which way to look from.
    camera: CameraSide,
    /// Where to write a screenshot, and after how many frames.
    shot: Option<(String, usize)>,
}

#[derive(Clone, Copy, PartialEq)]
enum CameraSide {
    Front,
    Left,
}

impl PreviewConfig {
    fn from_args() -> Self {
        let mut config = Self {
            frozen_phase: None,
            cadence: 0.8,
            camera: CameraSide::Left,
            shot: None,
        };

        let mut args = std::env::args().skip(1);
        while let Some(arg) = args.next() {
            match arg.as_str() {
                "--phase" => {
                    if let Some(v) = args.next().and_then(|v| v.parse().ok()) {
                        config.frozen_phase = Some(v);
                    }
                }
                "--cadence" => {
                    if let Some(v) = args.next().and_then(|v| v.parse().ok()) {
                        config.cadence = v;
                    }
                }
                "--camera" => {
                    if let Some(v) = args.next() {
                        config.camera = match v.as_str() {
                            "front" => CameraSide::Front,
                            _ => CameraSide::Left,
                        };
                    }
                }
                "--shot" => {
                    let path = args.next().unwrap_or_default();
                    let frames = args
                        .next()
                        .and_then(|v| v.parse().ok())
                        .unwrap_or(60);
                    config.shot = Some((path, frames));
                }
                _ => {}
            }
        }

        config
    }
}

fn setup(
    mut commands: Commands,
    config: Res<PreviewConfig>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Framed on the legs: this preview exists to judge a walk cycle, and the
    // gallery's torso-height framing puts the feet off the bottom of the
    // screen at any useful zoom.
    let (eye, look_at) = match config.camera {
        // Framed on the legs, close. A walk cycle is judged on the knee and
        // the foot; the gallery's whole-body framing at torso height leaves
        // both too small to read.
        // A TRUE side view — the eye level with the look-at point, so the
        // camera looks horizontally along +X rather than down at an angle.
        //
        // This matters more than it sounds: a walk cycle's whole shape is in
        // Z (forward/back), and an eye raised above its target foreshortens
        // exactly that axis. An earlier version of this preview put the eye
        // at y=0.9 looking at y=0.45 and the legs read as straight and
        // together at a phase where the numbers said 41 degrees of knee
        // flexion and 0.13 m of foot separation.
        CameraSide::Left => (Vec3::new(-2.2, 0.45, 0.0), Vec3::new(0.0, 0.45, 0.0)),
        CameraSide::Front => (Vec3::new(0.0, 0.45, 2.2), Vec3::new(0.0, 0.45, 0.0)),
    };

    commands.spawn((
        Camera3d::default(),
        Transform::from_translation(eye).looking_at(look_at, Vec3::Y),
    ));

    commands.spawn((
        DirectionalLight { illuminance: 8_000.0, ..default() },
        Transform::from_xyz(4.0, 8.0, 4.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    spawn_preview_skeleton(&mut commands, &mut meshes, &mut materials);
}

/// Spawns the synthetic rig as a visible chain of bone capsules.
///
/// Built here rather than pulled from the library because the crate's own
/// synthetic-rig spawner is `pub(crate)` and test-only. Rendering the bones
/// as real meshes rather than gizmos is deliberate for this preview: gizmos
/// are depth-tested and a walk cycle is judged from the side, where a
/// depth-tested overlay is exactly what hides the far leg.
fn spawn_preview_skeleton(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
) {
    let root = commands.spawn((Transform::IDENTITY, Name::new("GaitPreviewRoot"))).id();

    let joint = meshes.add(Sphere::new(0.035).mesh().uv(12, 8));
    let left = materials.add(StandardMaterial {
        base_color: Color::srgb(0.95, 0.45, 0.25),
        ..default()
    });
    let right = materials.add(StandardMaterial {
        base_color: Color::srgb(0.25, 0.55, 0.95),
        ..default()
    });
    let centre = materials.add(StandardMaterial {
        base_color: Color::srgb(0.85, 0.85, 0.88),
        ..default()
    });

    let mut bones: HashMap<Bone, Entity> = HashMap::new();

    for &bone in Bone::ALL.iter() {
        let parent = match bone.parent() {
            Some(parent) => bones[&parent],
            None => root,
        };

        let material = match bone.side() {
            migera::character::skeleton::Side::Left => left.clone(),
            migera::character::skeleton::Side::Right => right.clone(),
            migera::character::skeleton::Side::Center => centre.clone(),
        };

        let entity = commands
            .spawn((
                Transform::from_translation(bone.t_pose_offset()),
                ChildOf(parent),
                Name::new(bone.name()),
                Mesh3d(joint.clone()),
                MeshMaterial3d(material),
            ))
            .id();

        bones.insert(bone, entity);
    }

    // The synthetic rig: identity rest rotations, T-pose offsets, hips at
    // their own T-pose height.
    let skeleton = HumanoidSkeleton::for_other_rig(
        bones,
        Bone::ALL.iter().map(|&b| (b, Quat::IDENTITY)).collect(),
        Bone::ALL
            .iter()
            .map(|&b| (b, b.t_pose_offset().normalize_or_zero()))
            .collect(),
        Bone::Hips.t_pose_offset(),
        Quat::IDENTITY,
        Vec3::ONE,
        Bone::Hips.t_pose_world_position(),
    );

    commands.entity(root).insert(skeleton);

    // A ground plane, so a foot sinking through it is visible.
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::default().mesh().size(6.0, 6.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.30, 0.30, 0.33),
            ..default()
        })),
        Transform::IDENTITY,
    ));
}

/// Writes the gait pose for the current instant onto the skeleton.
fn drive_gait(
    time: Res<Time>,
    config: Res<PreviewConfig>,
    skeletons: Query<&HumanoidSkeleton>,
    mut transforms: Query<&mut Transform>,
) {
    let phase = config
        .frozen_phase
        .unwrap_or_else(|| time.elapsed_secs() * config.cadence);

    let params = GaitParams::default();
    let pose = walk_pose(phase, &params, &stance(&LocalPose::REST));

    for skeleton in &skeletons {
        write_pose_to_skeleton(skeleton, &pose, &mut transforms);
    }
}

/// Draws a line along every bone.
///
/// Joint spheres alone are not a readable skeleton — the first version of
/// this preview drew only the joints, and the resulting screenshot was a
/// scatter of disconnected dots from which no claim about a knee or a leg
/// split could honestly be made. The segments are what make the chain
/// legible.
///
/// Drawn from the live `GlobalTransform`s, so this is ground truth read from
/// the same data the renderer skins with.
fn draw_bones(
    mut gizmos: Gizmos,
    skeletons: Query<&HumanoidSkeleton>,
    transforms: Query<&GlobalTransform>,
) {
    for skeleton in &skeletons {
        for &bone in Bone::ALL.iter() {
            let Some(parent) = bone.parent() else { continue };

            let (Ok(from), Ok(to)) = (
                transforms.get(skeleton.entity(parent)),
                transforms.get(skeleton.entity(bone)),
            ) else {
                continue;
            };

            let colour = match bone.side() {
                migera::character::skeleton::Side::Left => Color::srgb(0.95, 0.45, 0.25),
                migera::character::skeleton::Side::Right => Color::srgb(0.25, 0.55, 0.95),
                migera::character::skeleton::Side::Center => Color::srgb(0.8, 0.8, 0.85),
            };

            gizmos.line(from.translation(), to.translation(), colour);
        }
    }
}

/// Saves a screenshot once the scene has settled, then exits.
fn shoot(
    mut commands: Commands,
    config: Res<PreviewConfig>,
    mut frame: Local<usize>,
    mut writer: MessageWriter<AppExit>,
) {
    let Some((path, at)) = config.shot.clone() else { return };

    *frame += 1;

    if *frame == at {
        commands
            .spawn(bevy::render::view::screenshot::Screenshot::primary_window())
            .observe(bevy::render::view::screenshot::save_to_disk(path));
    }

    if *frame > at + 8 {
        writer.write(AppExit::Success);
    }
}

