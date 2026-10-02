---
title: Getting up goes through key poses chosen by how the body lies, then the bodies are set back on their bones
description: "At rest, Ragdoll::get_up reads face up, down or on a side from the chest body, turns the character, and blends the drawn skeleton per bone in world space through solved key poses (getup.rs), palms flat, tucking dipping feet and hands, to standing; then re-pins. Read before changing get-up."
type: decision
status: current
tags:
  - ragdoll
  - physics
  - character-animation
  - correctness
updated: 2026-10-02
verified: 2026-10-02
code:
  - src/character/anim/getup.rs
  - src/character/anim/ragdoll.rs
  - src/character/anim/ragdoll_plugin.rs
  - src/character/anim/walker.rs
  - examples/character_gallery.rs
sources:
  - "VanSant (1988), Rising from a supine position to erect stance, Phys Ther 68(2):185-192, https://pubmed.ncbi.nlm.nih.gov/3340655/ — 32 young adults; most common: symmetrical push, symmetrical trunk, symmetrical squat, through sitting to squatting"
  - "Floor-to-stand studies (quadruped push-up to half-kneel), e.g. The Biomechanics of Healthy Older Adults Rising from the Floor Independently, IJERPH 20(4):3507, https://doi.org/10.3390/ijerph20043507"
  - "tests getup::tests (incl. chained_keys_keep_their_shared_contacts_in_place), ragdoll_plugin::tests::a_fallen_ragdoll_gets_up_and_is_pinned_again, a_rise_moves_no_limb_far_above_where_its_keys_put_it"
  - "character_gallery --anim-pose getup:sit|squat|quadruped|half_kneel|side_sit_left|side_sit_right (static keys, Front and Left, gizmos)"
  - "probe ragdoll_plugin::tests::probe_how_falls_lie (ignored); BRP recording of a live side rise, character.glb"
aliases:
  - H3
  - get up
  - get-up keys
  - Ragdoll::get_up
  - Rise
  - Lying
  - body_offsets
  - re-pin
  - tuck_foot
  - rise_moving
  - chained keys
---

# Getting up goes through key poses chosen by how the body lies, then the bodies are set back on their bones

A fallen body rises the way people do, by one of three documented routes:

- **Face up:** lying → sitting propped on both hands → squatting → standing.
  This is VanSant's most common adult pattern.
- **Face down:** lying → hands and knees → half-kneeling → standing. This
  is the quadruped route of floor-to-stand studies.
- **On the side** (chest within 45° of level, `getup::Lying::of`): lying →
  side-sitting on the hand underneath, legs folded to the other side →
  hands and knees → half-kneeling → standing (3.5 s). Read as face up or
  down instead, the first blend rolled the body 90° about its own length.
  The side-sit is authored by segment directions (`getup::aim`), since its
  legs fold sideways; it is solved like the others (knees, then the lean
  for the hand, iterated, since the lean moves the hips) and mirrors
  exactly left to right. Of 18 test falls pushed sideways and diagonally,
  two came to rest on a side (0.8 m/s forward and out, chest 53° from face
  down); the gallery's plain collapse does on some runs. Live, recorded
  over BRP, a side rise went side-sit → hands and knees → half-kneel →
  standing with no joint under the floor. Moving from the side-sit to
  hands and knees, the propping hand steps 0.4 m forward and arcs 128 mm
  up, as a stepping foot does.

The screen blends through these keys. The physics does nothing until the
end, when every body is set onto its bone and the root is pinned again.

## Decision

- **Keys are solved, not guessed** (`getup.rs`). Each key is sagittal joint
  angles about the rig's measured `left`: angles about `left` add down a
  chain, and a foot stays flat when pelvis + hip + knee + ankle sum to
  zero. The angles are then solved on the rig so the key's contacts meet
  the floor together, and the pose is set down on them:
  - **Sit:** thigh angle so the feet land flat, arm angle so the hands
    reach the floor behind.
  - **Squat:** trunk lean so the centre of mass is over the feet.
  - **Hands and knees:** pelvis pitch so hands and knees touch together.
  - **Half-kneel:** front hip so the front foot lands flat while the rear
    knee is down.

  Feet are grounded on the rig's own `Sole`. Knees, wrists and the seat
  use clearances (5, 3 and 10 cm), which are choices.
- **A hand that bears weight lies flat, palm down** (`getup::palm_flat`).
  Set down along its straight arm, the hand carried on from the forearm
  and pointed its fingers 18-21 cm into the floor (both rigs, live).
  Each contact hand is turned to lie along the floor, fingers forward on
  hands and knees, out and back propped behind the sit, out and forward
  under the side-sit. A real hand gets there by turning the forearm
  (pronation or supination) and bending the wrist back. Turning the hand
  alone would twist the wrist by the whole turn, so the turn about the
  arm's line is split between the shoulder and the forearm, and the hand
  only bends back, by at most 90° (it bends 75-90° on the floor). The
  rest pose's palm faces down (the bind's T-pose), and the fingertip is
  Winter's hand length on from the wrist, 0.108/0.146 of the forearm
  (`HAND_PER_FOREARM`). The rigs have no finger joints here.
- **Sitting is reclined and propped.** With the thighs level the knees
  are only seat-high and no shin reaches the floor from them; with the
  trunk upright the hands hung 27 cm short of the floor.
- **How it lies, and which way it faces.** On the rise's first frame, the
  chest body's rotation, taken relative to its standing-pose target,
  gives the chest's forward: pointing up means face up. Face down, it
  rises toward its head; face up, sitting up, toward its feet. The rise
  records that `turn`; the owner of the character's heading applies it
  once (`turn_pending`). The ragdoll doesn't write the rotation, because
  in the gallery a facing controller rewrites it every frame. The keys
  are in the character's frame, so they turn with it, and so must every
  conversion between the pose and the bodies: see
  [a pose delta's world is the character's frame](../rig-and-retargeting/a-pose-deltas-world-is-the-characters-frame.md).
  Before that fix, the turn swung the lying body 534 mm in the frame it
  was applied, and left the standing bodies with their arms out in a T.
- **Blended per bone in the world.** Each bone's world rotation takes the
  shortest path, and is then turned back into a local rotation. Blended
  locally, a limb rode its parents' swing as well as its own, and sitting
  up flung an arm out sideways, palm up.
- **Keys are chained, so shared contacts stay put.** Each key is placed
  where the next one needs it (`placed`): face up, the squat's left foot
  where the standing foot is and the sit's where the squat's is; face
  down, the half-kneel's front foot where the standing foot is and the
  hands-and-knees' knee where the half-kneel's is. Placed independently,
  a planted foot slid between keys.
- **Ground clearance.** The drawn skeleton is lifted so no toe, foot, hand
  or head joint goes below the ground, taken as the character entity's
  height.
- **A foot that dips is tucked, not lifted over.** The world-space blend
  can swing a foot through the floor between keys. Lifting the whole
  body clear of it left a leg hanging high: from lying to hands and
  knees, `LeftToeBase` lifted the body up to 209 mm. `tuck_foot` bends
  that leg's knee (about the hinge axis, thigh × shin) until the foot
  clears, and the lift then only has to clear what the tuck couldn't.
  Only feet that **move** between the two keys are tucked
  (`Ragdoll::rise_moving`, toe moving over 5 cm; all of them off the
  lying body). Tucking planted feet folded them up from squatting to
  standing, and the lift jumped 15 mm when the tuck let go. Hands tuck
  the same way, the elbow only bending further: lying with arms flat at
  its sides, a body sitting up swung a hand through the floor and was
  hoisted 82 mm. Last, each wrist bends its fingertips up to the ground
  (`turn_up_clear`): between two flat palms a turning hand dipped them
  38 mm in. Falling, the same turn keeps drawn toes and fingers out of
  the floor (see
  [a falling body is hinged and fleshed](./a-falling-body-is-hinged-and-fleshed.md)).
- **The re-pin also undoes the fall's joints and contacts:** the knees'
  and elbows' hinges go, their ball joints return, and the body's parts
  pass through each other again (see
  [a falling body is hinged and fleshed](./a-falling-body-is-hinged-and-fleshed.md)).
- **Bodies asleep during the rise; the end re-pins.** The frame after the
  last blend, each body is set onto its drawn bone at
  `Ragdoll::body_offsets` (recorded at spawn; a bone is rigid), still and
  awake. The root goes back to kinematic, the fall's `JointDamping` is
  removed, and the fall is cleared. Foot locks are kept free while rising.
- Timings are choices: 0.9 s + 0.8 s + 0.8 s, about 2.5 s from lying to
  standing.

## Alternatives considered

- **One timed blend from lying to standing** (the first stand-in). The
  in-between pose rose arched backward from kneeling.
- **Hand-authored key rotations.** Pose data has shipped broken here before;
  solving on the rig gives contacts that hold on any rig.
- **Physically drag the ragdoll through the keys.** This would show real
  bodies, but it pulls a limp chain through ground contacts. It's worth it
  once the keys are trusted.

## Consequences

- Keys, `puppet_base` and its as-rendered twin: every contact within
  15 mm of the floor and nothing under it; the COM inside its contacts'
  footprint; each key the right way round along the rig's forward
  (signed); the symmetric keys mirror exactly. Seen statically, Front and
  Left with gizmos: all four as described.
- Headless rise: its keys chosen, no frame jump over 3 cm, no end joint
  under the floor, it lasts the keys' 2.5 s, ends within 1 cm of where it
  stood, and every body is back on its bone and holds.
- Live, `puppet_base`: face up, it sits up on its hands and squats on flat
  feet; face down, it pushes up to hands and knees and then a half-kneel.
  No limb flung out after the world-space blend.
- Headless, both lying sides (`a_rise_moves_no_limb_far_above_where_its_keys_put_it`):
  hips, knees and hands never rise more than 6 cm above where the keys
  put them, feet 15 cm. The worst is 48 mm; with the tuck disabled,
  88 mm, and the test fails.
- Live, after the turn fix, both rigs: no per-frame neck move over 34 mm
  after landing, and standing bodies within 0.1–2.8° of their targets.
- **Found on the way:** the gallery rewrote the character's position from
  its own locomotion state every frame. The ragdoll's entity-follow only
  won while it wrote last, so every rise slid the character 0.45 m back to
  where it fell from. `follow_the_fallen_body` keeps the gallery's
  position in step; it now rises within 1 mm of where it lay.
- **A walker stands while it is down.** A `Walker` asked to walk kept its
  gait running through the fall and the rise. Its root motion carried the
  rising body 1.5–1.7 m forward while the hips went from 0.13 to 0.94 m,
  a visible slide (`physics_character_playground`, the same three starts
  before and after). Now `fall_when_uncaught` sets the gait to standing,
  and `drive_walkers` asks for no speed while `Ragdoll::is_falling`. The
  root moves 0.00–0.23 m through the rise, and that is the body settling
  before it starts. Once up, the walker starts from a stand.

## Revisit when

- Authored get-up motion exists: it replaces the keys, and the choice by
  lying side stays.
- The keys' floor on uneven ground: the clearance samples the character's
  `AnimGround` under each joint (since 2026-10-01, see
  [sample the ground in the world](../ik-and-locomotion/sample-the-ground-in-the-world-not-the-pose.md)),
  but the keys are still posed against a flat floor.

## Related

- [A fall hands the body to physics](./a-fall-hands-the-body-to-physics.md) — prerequisite: the fall, its rest signal and why the entity follows.
- [A falling body has hinged knees and elbows and solid flesh](./a-falling-body-is-hinged-and-fleshed.md) — context: the lying pose a rise starts from, and why hands tuck too.
- [A pose delta names a world axis](../rig-and-retargeting/a-pose-delta-names-a-world-axis.md) — prerequisite: why angles about `left` add down a chain.
- [A pose delta's world is the character's frame](../rig-and-retargeting/a-pose-deltas-world-is-the-characters-frame.md) — deeper: why the rise's turn broke the bodies, and the rule that fixed it.
- [Foot locks need the body's travel](../ik-and-locomotion/foot-locks-need-the-bodys-travel.md) — same-trap: an entity moved behind the locks' back.
- [Ragdoll body and anchor frames](./ragdoll-body-and-anchor-frames.md) — prerequisite: a body's rotation is its bone's, which is what lets it be set back on its bone.
- [A walking arm swings back, and its hand hangs curled](../ik-and-locomotion/a-walking-arm-swings-back-and-its-hand-hangs-relaxed.md) — context: the rig's fingers curl when standing and straighten while down; the rise's virtual fingertip leaves the real ones 84–90 mm under the floor for a moment.
