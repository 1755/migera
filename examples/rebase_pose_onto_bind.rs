//! Rebases a pose converted against this crate's straight T-pose onto the
//! bind of the rig its data came from, for its axial chain: the spine fix
//! for poses that cannot simply be re-imported (`relaxed_stand` was
//! finished by hand: knee bend baked in, neck solved for a level gaze).
//!
//! ```text
//! python3 tools/dump_bind_positions.py assets/models/idle.glb \
//!     assets/anim/idle_bind.positions.ron
//! cargo run --release --example rebase_pose_onto_bind -- \
//!     assets/anim/relaxed_stand.pose.ron assets/anim/idle_bind.positions.ron \
//!     assets/anim/relaxed_stand.pose.ron
//! ```
//!
//! See `convert::rebase_onto_bind` for what moves and what stays. Run once:
//! a rebased pose is already bind-relative, and rebasing it again would
//! remove the source's bind curvature twice.

use std::collections::HashMap;

use bevy::math::Vec3;
use migera::character::anim::asset::PoseAsset;
use migera::character::anim::convert::rebase_onto_bind;
use migera::character::anim::rig::BoneSet;
use migera::character::skeleton::Bone;
use serde::Deserialize;

#[derive(Deserialize)]
struct DumpedPositions {
    positions: HashMap<String, (f32, f32, f32)>,
}

/// The spine, whose double-counted curvature arched the back. Everything
/// hanging from it keeps its world orientation.
///
/// Not the shoulder girdle and arms: rebased too, they turned 0.1-1.1
/// degrees (the collarbones 12, landing the arm joints 4 mm from where they
/// hang anyway), so the tuned arms stay as they are. Not the neck:
/// `relaxed_stand`'s was solved for a level gaze, not converted from the
/// clip, and rebased as if it were it threw the head 23 degrees back. Not
/// the legs: their shape is the stance's and its baked knee bend, and their
/// feet were set flat on this rig.
const AXIAL: [Bone; 4] = [Bone::Hips, Bone::Spine, Bone::Spine1, Bone::Spine2];

fn main() {
    let arguments: Vec<String> = std::env::args().skip(1).collect();
    let [input, bind, output] = arguments.as_slice() else {
        eprintln!("usage: rebase_pose_onto_bind <in.pose.ron> <bind.positions.ron> <out.pose.ron>");
        std::process::exit(1);
    };
    let read = |path: &str| std::fs::read_to_string(path).unwrap_or_else(|error| panic!("could not read {path}: {error}"));
    let pose = ron::from_str::<PoseAsset>(&read(input))
        .unwrap_or_else(|error| panic!("{input} is not a pose: {error}"))
        .to_local_pose()
        .unwrap_or_else(|error| panic!("{input} names a bone this rig lacks: {error}"));
    let dumped: DumpedPositions = ron::from_str(&read(bind)).unwrap_or_else(|error| panic!("{bind} is not a positions dump: {error}"));
    let mut rest: BoneSet<Vec3> = BoneSet::from_fn(|bone| bone.t_pose_world_position());
    for (name, &(x, y, z)) in &dumped.positions {
        if let Some(bone) = Bone::from_name(name) {
            rest[bone] = Vec3::new(x, y, z);
        }
    }

    let rebased = rebase_onto_bind(&pose, &rest, &AXIAL);
    for bone in AXIAL {
        println!(
            "{:7} turned {:.2} degrees",
            bone.name(),
            pose.rotation(bone).angle_between(rebased.rotation(bone)).to_degrees()
        );
    }
    let text = ron::ser::to_string_pretty(&PoseAsset::from_local_pose(&rebased), ron::ser::PrettyConfig::new().struct_names(false))
        .expect("a pose always serializes");
    std::fs::write(output, text).unwrap_or_else(|error| panic!("could not write {output}: {error}"));
    println!("wrote {output}");
}
