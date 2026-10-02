---
title: A falling body keeps its joints' passive tone, relative to each parent, toward a relaxed pose
description: "With only damping, a fall swung limbs free into their stops: a puppet. Each joint now pulls toward a relaxed pose relative to its parent, weak mid-range and stiffening out (Riener & Edrich's knee, NASA's neutral posture), implicit per substep; joint spin fell 45-70%. Read before changing fall tone."
type: decision
status: current
tags:
  - ragdoll
  - physics
  - biomechanics
  - correctness
updated: 2026-10-03
verified: 2026-10-03
code:
  - src/character/anim/passive.rs
  - src/character/anim/getup.rs
  - src/character/anim/ragdoll_plugin.rs
  - src/character/anim/ragdoll.rs
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, §9.1 (passive force-length of the parallel elastic element) and §9.3 (viscoelastic models)"
  - "Riener & Edrich (1999), Identification of passive elastic joint moments in the lower extremities, J Biomech 32(5):539-544 — knee equation as reproduced in later papers"
  - "Mount et al. (2003), Evaluation of Neutral Body Posture on Shuttle Mission STS-57, NASA TM-2003-104805, Table 1 — https://ntrs.nasa.gov/citations/20040200967"
  - "passive joint damping: hip 1.9-4.6 N·m·s/rad, knee far lower (In vivo measurement of the passive viscoelastic properties of the human knee joint, https://www.sciencedirect.com/science/article/abs/pii/S0167945797000274)"
  - "tests passive::tests, getup::tests::the_relaxed_pose_has_the_neutral_body_postures_angles, ragdoll_plugin::tests::passive_tone_slows_a_falling_limb_about_its_joint_and_still_rests"
  - "probes (ignored): probe_fall_damping, probe_fall_shape, probe_toned_rest, probe_how_falls_lie"
aliases:
  - PassiveJoint
  - passive_tone
  - passive_gains
  - apply_passive_joints
  - relaxed pose
  - neutral body posture
  - unconscious body
  - puppet fall
  - RELAXED_KNEE_FLEXION
---

# A falling body keeps its joints' passive tone, relative to each parent, toward a relaxed pose

An unconscious body is not a puppet. Muscle and connective tissue still
resist stretch, a little in mid-range and steeply toward each end. So while
falling, each joint pulls its body toward a relaxed pose, relative to the
body it hangs from, with a stiffness that rises with the stretch.

## Context

A fall had no pose control (`FALL_TONE` = 0), only uniform joint damping
(3/s) and hard anatomical stops. Nothing resisted until a joint struck its
stop. Headless, four pushed falls ended on their stops:
- hips at −29.9° (the 30° extension stop) and 116.6° (120° flexion);
- abduction 44.4° (stop 45°);
- a backward fall's knees locked at −5°;
- every elbow at −5°.

Tone was tried before as the pose controller at strength 0.15 (see
[a fall hands the body to physics](./a-fall-hands-the-body-to-physics.md)).
That pulls each body toward a **world** target from the standing pose, like
a puppet's strings. Lying, a forearm was still moving at 0.57 m/s after 3 s.

## Decision

`passive.rs`: while falling, every body with a parent body has a
`PassiveJoint`. It is added in `release_falling_roots` and removed at the
re-pin.

- **Relative, not world:** the pull is toward the child's rotation relative
  to its parent in the relaxed pose. A turn of the whole pair is no stretch
  (`a_passive_joint_relaxes_relative_to_its_parent_not_the_world`).
- **The relaxed pose** is `getup::relaxed`, built by segment directions.
  Most of it is NASA's neutral body posture, measured in weightlessness
  where gravity loads nothing (STS-57 crew medians):
  - neck 16° forward;
  - shoulders 45° forward and 30° out;
  - elbows 70°;
  - hips 10° out;
  - waist straight.

  The legs are different. The knee's relaxed angle depends on the hip,
  because two-joint muscles cross both. Riener & Edrich's passive knee
  moment is zero at 14° of flexion with the hip straight, 26° at 30°, and
  43° at 60°. So the hips relax to 15° and the knees to 20°, Riener's zero
  for a fallen body's near-straight hips. Relaxed toward the weightless
  crew's 31° and 50°, a body lying on its back drew its knees up against
  gravity, and they toppled side to side for 8 s without coming to rest.
- **Stiffness** is `k(θ) = k₀·(1 + (θ/θs)²)`, with θ the angle from relaxed.
  The knee's 4 N·m/rad doubling at 86° gives 4.1, 9.5 and 22.7 N·m at 60°,
  90° and 130° of flexion. Riener & Edrich measured 4.5, 6.2 and 16.5
  (`the_knees_passive_moment_is_…`). Other joints are scaled from it:

  | Joint | k₀ (N·m/rad) | θs | Damping (N·m·s/rad) |
  |---|---|---|---|
  | Trunk | 30 | 30° | 4 |
  | Neck and head | 2.5 | 40° | 0.3 |
  | Hip | 10 | 70° | 3 (measured 1.9–4.6) |
  | Knee | 4 | 86° | 0.5 |
  | Shoulder | 3 | 70° | 0.5 |
  | Elbow | 2 | 70° | 0.2 |
  | Wrist | 0.4 | 50° | 0.05 |

  All of these are for 75 kg and are scaled by body mass. In mid-range they
  are weaker than gravity, so a limb lying on the floor stays down. The
  neck is under the ~4 N·m that holds a head level, so a limp head lolls.
- **Applied as a joint torque,** the parent taking the reaction. It is solved
  implicitly every substep with `joint_drive::drive_impulse`, so a stiff end
  of range cannot go unstable on a light hand.
- `Ragdoll::passive_tone` scales it: 1 is the measured tone, 0 none.

## Two traps on the way

- **A tone must not fight a constraint, or the body never rests.**
  - **Knees and elbows** are hinges while falling, so the hinge holds the
    limb's roll and sideways tilt where the fall found them. Pulled toward
    the relaxed pose in all three directions, a shin rolled against its
    hinge at 1.4–2 rad/s. Hinged joints now pull about the hinge axis
    alone (`PassiveJoint::hinge`).
  - **Ankles** have no tone. A light foot lying on the floor and pulled by
    its ankle fought the floor's friction. At 3 N·m/rad the feet crept and
    jolted at 0.05–0.5 m/s for 8 s, and pulling about the flexion axis
    alone did not stop it. At 1 N·m/rad one landing in three still crept.
    The ankle keeps its limits and the fall's damping, as before.
- **Which push lands on a side moved.** The rise test's diagonal pushes now
  land face down. Pushes straight out to the side at 1.5 m/s land on that
  side (`probe_how_falls_lie`), so the test uses them (see
  [a fall test samples one chaotic landing](./a-fall-test-samples-one-chaotic-landing.md)).

## Consequences

The same carried falls, damping 3/s, with tone off and on
(`probe_fall_damping`):

| | Off | On |
|---|---|---|
| Peak joint spin (relative to the parent) | 51.7 / 136.9 / 54.0 rad/s | 28.7 / 43.1 / 28.9 rad/s |
| Asleep by | 2.9–5.7 s | 3.9–7.6 s |

Pushed falls (`probe_fall_shape`):
- **Backward:** hip flexion up to 88.5° (was 116.6°) and abduction 2.9°
  (was 44.4°).
- **Forward:** prone, it still reaches the 30° hip-extension stop, the
  pelvis tipping onto it under the body's weight.
- **Sideways:** the top leg drops across onto the 30° adduction stop.
- **Flesh:** a light hand striking a thigh dips up to 24 mm into it at
  impact, and a hand lying on a thigh rests 8 mm in. The flesh test allows
  30 mm at impact and 10 mm at rest; parts passing through each other
  overlapped 101–180 mm.
- **Live** (`character_gallery --ragdoll on --fall-at-frame 120`):
  - The knees buckle, the body drops through a kneel and rolls back.
  - Knees and elbows stay bent in mid-range.
  - Lying, the knees are slightly flexed and an arm rests across the chest.
  - It comes to rest and rises to standing.

## Revisit when

- Knee and hip should be coupled as Riener & Edrich's two-joint terms
  couple them: the knee's relaxed angle following the hip's. Today it is
  fixed for near-straight hips.
- avian gains per-body contact stiffness or joint inverse-mass scaling
  (see
  [a falling body is hinged and fleshed](./a-falling-body-is-hinged-and-fleshed.md)):
  the foot's fight with the floor may then allow an ankle tone.

## Related

- [A fall hands the body to physics](./a-fall-hands-the-body-to-physics.md) — prerequisite: the fall, its joint damping, and why tone as a world pose failed.
- [A falling body has hinged knees and elbows and solid flesh](./a-falling-body-is-hinged-and-fleshed.md) — context: the hinges and stops this tone acts within.
- [A standing ragdoll carries its weight through joint torques](./a-standing-ragdoll-carries-its-weight-through-joint-torques.md) — prerequisite: `drive_impulse`, the implicit joint torque this reuses.
- [A fall test samples one chaotic landing](./a-fall-test-samples-one-chaotic-landing.md) — same-trap: the landings it changed, and the tests that moved with them.
