//! Converts a `.positions.ron` dumped from a real animation clip into a
//! `.pose.ron` the runtime loads.
//!
//! ```text
//! # 1. read the clip (inside the model-convert dev shell)
//! blender --background --python tools/dump_animation_pose.py -- \
//!     assets/models/idle.glb --frame 0 --ron /tmp/idle.positions.ron
//!
//! # 2. convert it into a pose
//! cargo run --release --example import_reference_pose -- \
//!     /tmp/idle.positions.ron assets/anim/idle_stand.pose.ron
//! ```
//!
//! # Why this is two steps
//!
//! Blender can read the clip; only this crate knows what a pose *is*.
//!
//! Dumping Blender's local rotations straight to a `.pose.ron` was tried
//! and is wrong: a bone's local rotation there is relative to **Mixamo's
//! bind pose**, while a `LocalPose` stores a delta relative to **this
//! crate's T-pose**. The numbers transfer cleanly and mean something else
//! on arrival — measured, the left hand landed 0.54 m *above* the shoulder
//! instead of hanging below it, the same class as the once-live "arms
//! overhead" retargeting bug.
//!
//! World positions have no reference frame to get wrong, and
//! [`convert::pose_from_world_positions`] already derives rotations from
//! them correctly — it is how `relaxed_stand` was authored from this very
//! clip. So Blender reads, Rust converts.

use std::collections::HashMap;

use migera::character::anim::asset::PoseAsset;
use migera::character::anim::convert::{
    direction_error_degrees, pose_from_world_positions,
};
use migera::character::anim::rig::{forward_kinematics, BoneSet};
use migera::character::skeleton::Bone;

use bevy::math::Vec3;
use serde::Deserialize;

/// What the dump tool writes.
#[derive(Deserialize)]
struct DumpedPositions {
    positions: HashMap<String, (f32, f32, f32)>,
}

fn main() {
    let arguments: Vec<String> = std::env::args().skip(1).collect();
    let [input, output] = arguments.as_slice() else {
        eprintln!(
            "usage: import_reference_pose <input.positions.ron> <output.pose.ron>\n\
             \n\
             Produce the input with:\n  \
             blender --background --python tools/dump_animation_pose.py -- \\\n    \
             <model> --frame N --ron <input.positions.ron>",
        );
        std::process::exit(1);
    };

    let source = match std::fs::read_to_string(input) {
        Ok(source) => source,
        Err(error) => {
            eprintln!("could not read {input}: {error}");
            std::process::exit(1);
        }
    };

    let dumped: DumpedPositions = match ron::from_str(&source) {
        Ok(dumped) => dumped,
        Err(error) => {
            eprintln!("{input} is not a positions dump: {error}");
            std::process::exit(1);
        }
    };

    // Bones the dump does not mention keep their T-pose position, so a
    // partial dump converts to a partial pose rather than collapsing the
    // rig to the origin.
    let mut targets: BoneSet<Vec3> = BoneSet::from_fn(|bone| bone.t_pose_world_position());
    let mut found = 0;

    for (name, &(x, y, z)) in &dumped.positions {
        let Some(bone) = Bone::from_name(name) else {
            eprintln!("warning: '{name}' is not a bone on this rig, ignoring");
            continue;
        };
        targets[bone] = Vec3::new(x, y, z);
        found += 1;
    }

    println!("read {found} bone positions from {input}");

    // The dump is in the source rig's own scale and hip height. Shift it
    // so the hips land where this crate's rig expects them, since a pose
    // stores rotations and an absolute offset would otherwise skew every
    // derived direction near the root.
    let hip_offset = Bone::Hips.t_pose_world_position() - targets[Bone::Hips];
    for &bone in Bone::ALL.iter() {
        targets[bone] += hip_offset;
    }

    let pose = pose_from_world_positions(&targets);

    // Report how faithfully it converted, rather than assuming. Rotation
    // space cannot reproduce an authored position that violates a bone
    // length, so the honest measure is whether each bone points the way
    // the reference did.
    let errors = direction_error_degrees(&targets, &pose);
    let (worst_bone, worst) = Bone::ALL
        .iter()
        .map(|&bone| (bone, errors[bone]))
        .fold((Bone::Hips, 0.0f32), |worst, current| {
            if current.1 > worst.1 { current } else { worst }
        });

    println!("worst direction error: {} at {worst:.3} degrees", worst_bone.name());

    // A sanity check on the result's SHAPE, so an import that produced
    // something impossible fails here rather than in a screenshot.
    let positions = forward_kinematics(&pose);
    let hand_drop = positions[Bone::LeftShoulder].y - positions[Bone::LeftHand].y;
    println!("left hand sits {hand_drop:.3} m below the shoulder");

    if hand_drop < 0.0 {
        eprintln!(
            "\nrefusing to write: the converted pose puts the hand ABOVE the shoulder, \
             which means the dump's coordinate space does not match this rig's. Check \
             `to_migera_space` in the dump tool.",
        );
        std::process::exit(1);
    }

    let asset = PoseAsset::from_local_pose(&pose);
    let text = ron::ser::to_string_pretty(
        &asset,
        ron::ser::PrettyConfig::new().struct_names(false),
    )
    .expect("a pose always serializes");

    if let Some(directory) = std::path::Path::new(output).parent() {
        let _ = std::fs::create_dir_all(directory);
    }

    match std::fs::write(output, text) {
        Ok(()) => println!("wrote {output}"),
        Err(error) => {
            eprintln!("could not write {output}: {error}");
            std::process::exit(1);
        }
    }
}
