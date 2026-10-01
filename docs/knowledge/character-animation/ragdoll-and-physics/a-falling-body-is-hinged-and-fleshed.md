---
title: A falling body has hinged knees and elbows and solid flesh
description: "A fall swaps knees and elbows for avian hinges (they folded 163° backward) and makes the body's parts collide; colliders are anthropometric, the torso blocks; hip and shoulder cones lean to the middle of their range (hip 120 forward, 30 back). Read before changing ragdoll shapes, joints, limits or rest."
type: decision
status: current
tags:
  - ragdoll
  - physics
  - biomechanics
  - correctness
updated: 2026-10-01
verified: 2026-10-01
code:
  - src/character/anim/ragdoll.rs
  - src/character/anim/ragdoll_plugin.rs
sources:
  - "tests ragdoll_plugin::tests::a_limp_fall_bends_knees_and_elbows_as_hinges_and_keeps_its_flesh_apart, a_rise_moves_no_limb_far_above_where_its_keys_put_it, a_released_ragdoll_falls_with_its_momentum_and_is_drawn_where_it_lies"
  - "probe ragdoll_plugin::tests::probe_fall_shape (ignored)"
  - "AAOS normal range of motion: knee flexion 0-135, elbow 0-150 (e.g. https://goniometer.io/range-of-motion)"
  - "ANSUR II male means (Gordon et al., 2012 Anthropometric Survey of U.S. Army Personnel, NATICK/TR-15/007): thigh circumference 625 mm, calf 373, flexed biceps 358, chest breadth 289, hip breadth 354"
aliases:
  - Hinge
  - KNEE_RANGE
  - ELBOW_RANGE
  - TorsoBlock
  - default_flesh_radii
  - self-collision
  - knee bends backward
  - anatomical_cone_centre
  - tilted_parent_basis
  - anatomical_side_cone
  - LimitOnly
  - hip abduction
  - hip extension limit
---

# A falling body has hinged knees and elbows and solid flesh

Contents: [Decision](#decision) · [Alternatives](#alternatives-considered) ·
[Consequences](#consequences) · [Lessons](#lessons-on-the-way) ·
[Revisit when](#revisit-when)

Standing, the ragdoll is driven by its pose controller, and ball joints
with symmetric cones are enough. A fall runs at zero tone. Nothing then
holds a knee to its one way of bending, and a body whose parts never
touch each other is a skeleton of thin sticks. So a fall changes three
things for its duration, and the end of the rise changes them back.

## Decision

- **Knees and elbows become hinges** (`Ragdoll::hinges`,
  `release_falling_roots`). Each ball joint is disabled (`JointDisabled`)
  and an avian `RevoluteJoint` takes over.
  - **The axis** is fixed in the upper segment (femur, humerus). It runs
    across that segment and the direction the bend carries the limb:
    backward for the shin, forward for the forearm (`spawn_ragdoll`, from
    the spawn pose's heel-to-toe forward).
  - **Its frames coincide at the moment of the fall**, so nothing snaps.
    Whatever roll and sideways tilt the limb has is frozen as the hinge's
    straight line.
  - **The limits are the anatomical range less the bend it already has:**
    knee −5..140°, elbow −5..150°, from AAOS normal flexion (0–135,
    0–150), a little wider because a limp limb moves passively.
  - **The ball joint comes back** when the rise re-pins the bodies.
- **The body's parts are solid to each other while falling.** Each body's
  collision filter widens to its own layer. Jointed pairs stay exempt
  (`JointCollisionDisabled`, hinges included), so an arm still overlaps
  the chest it hangs from.
- **Flesh is anthropometric** (`default_flesh_radii`,
  `default_torso_blocks`), scaled by the rig's hips height against
  0.856 m:
  - **Limbs:** capsules of the segment's mean radius. Thigh 0.075, shank
    0.048, upper arm 0.045, forearm 0.037, head 0.09, from ANSUR II
    circumferences.
  - **Torso:** three rounded blocks wider than deep, so a fallen body lies
    on its back, front or flank instead of rolling. The pelvis is 0.34 ×
    0.22 and reaches 0.10 below the hips joint; abdomen 0.30 × 0.21; chest
    0.31 × 0.23.
  - **The torso's mass properties stay the old capsule's,** set
    explicitly, because standing stability was tuned on them.
- **Hips and shoulders lean their cones to the middle of their range**
  (`anatomical_cone_centre`, `tilted_parent_basis`), standing and falling
  alike. avian's swing limit is a symmetric cone, and a real hip flexes
  120° but extends only 30° (AAOS), so one cone centred on the bind
  cannot fit it.
  - **Hip:** the cone is centred 45° forward of straight down, with the
    same 75° half-angle, so it reaches 120 forward and 30 back.
  - **Shoulder:** centred out to the side and 0.3 forward, with a 105°
    half-angle. That reaches up, down, across the front and a hanging
    arm's 60° extension (104° from the centre), but not far behind the
    back. The run's backswing measures 94°.
  - **The twist reference turns with the cone** (shortest arc from the
    bind direction), so every pose's twist reads as before.
  - **The centre comes from the rig's bind frame,** the same in
    `spawn_ragdoll` and the pose-limits test, so the test measures the
    frames the joints have.
- **Hip abduction is a second cone on the same pivot**
  (`anatomical_side_cone`, `LimitOnly`). The tilted hip cone alone
  reaches about 68° out to the side at neutral flexion (AAOS 45; falls
  measured 52). A second `SphericalJoint` with the same anchors carries
  only a swing limit: 135° about the direction straight across the body,
  toward the other leg. That excludes just the 45° around pointing
  straight out. The allowed region is the two cones' intersection, an
  oval one cone cannot make. Abduction stops at 45° standing and flexed
  alike; flexion and extension are untouched. It is spawned with the
  joints and kept through falls, and fall damping skips it.

## Alternatives considered

- **Hinges all the time.** A revolute locks the forearm's roll, and the
  wave holds it at 90.7°, so the standing pose controller would push into
  the joint forever. The hinge is right only once nothing drives the limb.
- **One-directional limits on the ball joint.** avian 0.7's swing limit
  is a symmetric cone (see
  [avian joint limits are not cone and twist](./avian-joint-limits-are-not-cone-and-twist.md)),
  so it cannot say "forward only".
- **Self-collision while standing.** A torso contact fought the controller
  and left the neck 8.8° off (`spawn_bone_body`), so it stays off there.
- **Shaping the hip with one cone only.** Wider toward flexion, a cone
  is wider to the side too: one fitted to 120 forward and 30 back allowed
  68 of abduction. Narrowed to fit abduction, it loses flexion. Two
  intersecting cones (above) fit all three.
- **Collision hooks to filter pairs.** These are one global type per app
  that the consumer must register, too intrusive for a plugin. Layers plus
  `JointCollisionDisabled` were enough.

## Consequences

Headless, `puppet_base` drawn, relaxed stance, pushed 1.5 m/s four ways
(`probe_fall_shape`):

| | before | after |
|---|---|---|
| knee bend | −163..114° | −5.5..99° |
| knee out of plane | 88° | ≤ 1.8° |
| elbow bend | −81..89° | −5.5..146° |
| elbow out of plane | 85° | ≤ 5.3° (as stood) |
| unjointed overlap | 101–180 mm | ≤ 18 mm |
| hip extension, forward fall | 43° | 30.0° |
| hip flexion, deepest | 75° (cone) | 116° |
| hip abduction | 52° | 45.1° |

- **Lying side:** side falls lie on the flank (hips 0.18 m, the pelvis's
  half-breadth). Forward lands face down and back face up on both rigs.
  The fall test that used a +X push (sideways on that rig) for "face
  down" now uses real forward and backward pushes.
- **Rest:** a T-pose fall now rests at 5.5 s instead of 4.2: an
  outstretched forearm slides about its elbow on the floor a while.
- **Rise:**
  - A hand sweeping through the floor from lying to the propped sit made
    the clearance lift hoist the body 82 mm. Hands now tuck like feet
    (`tuck_foot`).
  - Each tuck folds about the joint's own `Hinge` axis, flexing.
    Folding about the pose's bend (upper × lower) flipped a nearly
    straight arm's axis from frame to frame, and the hips jumped 86 mm.
  - The rise keeps knees and elbows above the floor as well as the
    body's ends (`RISE_CLEARANCE_BONES`); a knee went 29 mm under it.
- **Cost:** four headless 4 s falls simulate in 1.00 s, against 0.81 s
  before, about 25% more while falling. Standing is unchanged.

## Lessons on the way

- **A hinge leaves cancelled velocity behind.** A thin forearm on the
  floor carried 1.1 rad/s of roll that its elbow hinge undid every
  substep: it turned 1.1° in half a second. Judged by velocities, the
  body never rested. Rest is now judged by how far each body moves over
  the whole `REST_SECONDS` window. Frame-to-frame differences read
  millimetre contact jitter as speed.
- **A torso's mass came from its collider.** Limbs set theirs from Table
  4.1, but the torso's came from its capsule's density, so swapping in a
  block moved its mass and centre (bodies 14° off at spawn). avian's
  `AngularInertia::from_shape` on the tilted capsule then lost the
  moments' orientation (the chest spun to 3769 rad/s). They are now built
  explicitly in the segment's frame.
- **Spawn a test rig facing the way it will stand.** Turned half round
  after spawning, the upper-arm bodies were still 28–33° off their
  targets a second later, and the elbows read asymmetric.
- **Hang a test skeleton under its character, as a game does.** In the
  get-up test it was a separate root, which never followed the fallen
  body, so the rise dragged the body back to the spawn spot: 0.40 m or
  1.17 m depending on how the fall landed. The test's 30 mm-a-frame bound
  read that slide as a jump. Hung under the character, it rises 43 mm
  from where it lay.

## Revisit when

- Hip adduction: nothing but the other leg's flesh stops a thigh
  crossing under the body while falling (the side cone's centre points
  there). AAOS allows 30.
- Another lopsided joint (the shoulder's reach behind the back, an ankle):
  a second cone on the same pivot is the tool.
- A wrist or hand body is added: it needs its own hinge-like limits.

## Related

- [A fall hands the body to physics](./a-fall-hands-the-body-to-physics.md) — prerequisite: the fall these changes apply during.
- [Getting up goes through key poses](./getting-up-is-a-timed-blend-then-a-re-pin.md) — applies: the re-pin restores the ball joints; hands now tuck.
- [avian joint limits are not cone and twist](./avian-joint-limits-are-not-cone-and-twist.md) — context: why a ball joint's limits cannot be one-directional.
- [Ragdoll body and anchor frames](./ragdoll-body-and-anchor-frames.md) — prerequisite: the body frames the hinge axes are expressed in.
