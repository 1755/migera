---
title: Lugaru's joint/muscle animation system
description: "Source-verified account of Lugaru's Joint/Muscle design: joints as particles, muscles as springs with a continuous animated-vs-ragdoll strength dial, rotation derived from position. Later sections log migera's deleted muscle-module port (history). Read before designing ragdoll blending or physics-driven animation."
type: research
status: current
tags:
  - character-animation
  - prior-art
  - ragdoll
  - physics
  - case-study
updated: 2026-09-25
sources:
  - https://github.com/redagito/Lugaru (Graphics/include/Animation, Graphics/source/Animation, App/source/Animation/Skeleton.cpp)
  - https://github.com/WolfireGames/lugaru (Source/Animation/*)
aliases:
  - Lugaru
  - Wolfire
  - Overgrowth predecessor
  - procedural-animation
  - muscle module
  - MuscleSim
  - strength dial
---

# Lugaru's joint/muscle animation system

Contents: [Why this exists](#why-this-exists) ·
[Structural difference from FK](#the-core-structural-difference-from-bone-hierarchy-fk) ·
[Per-frame update](#the-per-frame-update--physics-not-transform-propagation) ·
[Relevance to migera](#relevance-to-migera) ·
[Open questions](#open-questions-not-answered-by-the-files-fetched-so-far) ·
[Twist not simulated](#twistroll-not-physically-simulated-source-verified-2026-09-21) ·
[Muscles only in ragdoll](#the-mass-springmuscle-system-only-runs-during-ragdoll-source-verified-2026-09-21) ·
migera port log (history): [keyframe layer](#keyframe-driving-layer-built-and-landed-2026-09-21-same-day) ·
[twist + clips](#twist-authored-not-derived--multi-keyframe-clip-playback-2026-09-21-same-day) ·
[idle redesign](#casual-idle-redesign-research-backed-motion--a-real-relaxed-baseline-2026-09-2122) ·
[foot-planting](#foot-planting-ground-locked-dynamic-joints-replace-permanent-pins-2026-09-22) ·
[idle variety](#idle-variety-an-occasional-secondary-variant-spliced-into-the-base-loop-2026-09-22) ·
[forward walking](#real-forward-walking-locomotion-hips-becomes-ground-locked-not-permanently-pinned-2026-09-22) ·
[Related](#related)

> **Status note (Phase 7 cutover).** The design described here was
> implemented in migera as `src/character/muscle`, and that module has
> since been **deleted** and replaced by the rotation-space
> `src/character/anim` — see this tree's [INDEX](./INDEX.md) for what
> shipped instead and why. This document remains accurate as an account of
> *Lugaru's* system and is kept as prior art; it no longer describes any
> code in this repository. The one idea carried forward is the continuous
> per-constraint `strength` dial, now applied to PD torque ceilings.
> The sections from "Keyframe-driving layer" onward log that deleted port
> (see [The muscle module is deleted](./animation-core/muscle-deleted-anim-is-the-only-stack.md))
> and are history only.

Source-verified against Lugaru's actual GPLv2 C++ source (Wolfire Games,
released 2010; mirror used here: `github.com/redagito/Lugaru`, files
under `Graphics/include/Animation/{Joint,Muscle,Animation}.hpp`,
`Graphics/source/Animation/{Joint,Muscle}.cpp`,
`App/include/Animation/Skeleton.hpp`,
`App/source/Animation/Skeleton.cpp`) — not vendored locally in
`botica/references/` (only Overgrowth, Lugaru's successor by the same
studio/author, is vendored there), so this is a from-source web read, not
a repo grep. Lugaru predates Overgrowth (`research/OVERGROWTH_ANIMATION.md`
in the botica project) and is where this studio's signature
physics-driven animation approach originates — Overgrowth's own system is
a refinement of the ideas here, not a clean-slate design.

## Why this exists

migera's character work (`src/character/skeleton.rs`) currently uses a
conventional rotation-driven bone hierarchy (`Transform`/`ChildOf`, plain
forward kinematics) — the same family as Mixamo/Overgrowth/most modern
engines. Lugaru represents a genuinely different point in the design
space: **animation and ragdoll physics are not two separate systems that
hand off to each other — they are the same system, always running**, and
a keyframe-authored "animation" is really a set of *targets* that a
physics simulation is driven toward, not a set of transforms applied
directly. Given migera's own SDF/physics background (`src/physics`,
`avian3d`), this is a directly relevant prior-art data point for whether
and how far to lean into physics-driven (rather than purely kinematic)
character animation later.

## The core structural difference from bone-hierarchy FK

A conventional skeleton (migera's current one included) is a tree of
`Transform`s; a bone's *position* is a derived quantity, computed by
walking parent rotations down to it every frame (forward kinematics). No
physics is involved unless a separate ragdoll system takes over, usually
by discarding the animated transforms entirely and handing the same
bones to a physics engine's rigid-body joints.

Lugaru's `Skeleton` instead stores:

- **`Joint`** (`Graphics/include/Animation/Joint.hpp`) — a physical
  particle, not a transform node. Fields include `position`,
  `oldposition`, `realoldposition`, `startpos`, `velocity`,
  `oldvelocity`, `mass`, `length`, `velchange`, plus state flags
  (`locked`, `visible`, `hasparent`, `sametwist`) and a `bodypart label`
  (a 20-value enum: head, neck, shoulders, elbows, hands, hips, knees,
  feet, etc. — this is the closest analogue to migera's own `Bone` enum).
  A `Joint` has at most a `parent` pointer, not a full child list — the
  hierarchy is much shallower than a real bone tree; most of the
  structural connectivity actually lives in `Muscle`, not `Joint`.
- **`Muscle`** (`Graphics/include/Animation/Muscle.hpp`) — a *spring/rod
  constraint* connecting exactly two `Joint*` parents, carrying `length`,
  `targetlength`, `minlength`, `maxlength`, `strength`, and a
  `muscle_type` (`boneconnect | constraint | muscle`). It also owns the
  mesh-skinning data directly (vertex-index lists for full/low-poly/
  clothing meshes), i.e. skinning weights are attached to the
  *constraint*, not to a bone transform.
- **`Animation`** (`Graphics/include/Animation/Animation.hpp`) — a
  keyframe is `AnimationFrameJointInfo { position, twist, twist2,
  onground }` per joint, plus per-frame `forward` (facing direction),
  `label`, `weapontarget`, `speed`. Rotation is stored as two scalar
  **twist angles**, not a quaternion or matrix — a much lower-dimensional
  representation than a full 3-DOF rotation, only viable because the
  joint/muscle constraint solve (not the stored rotation) is what
  actually determines final bone orientation each frame.

## The per-frame update — physics, not transform propagation

`Skeleton::DoConstraints()` (the ~400-line core loop, called every frame
when the skeleton is in ragdoll/`free` mode, and every frame that a
partial-physics blend is active) does NOT walk the joint tree applying
rotations. It:

1. Integrates each `Joint`'s `position` from its `velocity` (real
   Newtonian integration, not a keyframe lookup).
2. Repeatedly (3 iterations, `numrepeats`) applies each `Muscle`'s
   `DoConstraint()` — see below — to pull the two joints it connects
   back toward the muscle's current target length. Iterating multiple
   times per frame is the same reason Jakobsen-style position-based
   dynamics/XPBD solvers iterate multiple substeps: a single pass over
   an interconnected constraint graph doesn't converge, iterating does.
3. Checks terrain/object collision per joint (line-segment intersection
   against level geometry), applies friction/elasticity, and locks a
   joint once it settles (velocity below a threshold) — this is where
   footplant/resting behavior emerges, not from an authored IK pass.
4. Accumulates `damage` from high-impact collisions — i.e. hit reactions
   and combat damage are a *direct readout of the physics simulation*,
   not a separate hit-reaction state machine layered on top (contrast
   with botica's own `hit_reaction` module, see
   [botica's character animation system](./botica-character-animation-system.md), which is a conventional
   separate directional-flinch system).

`Skeleton::DoGravity()` applies gravity per joint, with an explicit
exception: knee/elbow joints skip gravity when the body is inverted
(checked via `lowforward.y > -.1` or `forward.y < .3`) — a small,
specific hand-tuned hack to keep flips/inversions from immediately
buckling the limbs under gravity, called out because it's exactly the
kind of undocumented special-case that's invisible from the general
algorithm and easy to lose if this system were ever reimplemented from
a description rather than the source.

### `Muscle::DoConstraint()` — the actual spring-to-target math

The three `muscle_type` values change what the constraint solves toward:

- `boneconnect`: `strength` is forced to `1` — the muscle behaves as a
  rigid, inextensible rod (this is the "bone" in the traditional sense:
  a fixed-length link that just transmits the skeleton's shape).
- `constraint`: `strength` is forced to `0` — the muscle just enforces
  its `relaxlength` (a resting/neutral length) with no pull toward any
  animated target; this is closer to a soft-tissue/ligament link that
  keeps two joints roughly apart/together without driving a pose.
- `muscle` (the general case): `strength` is user-authored in `[0,1]`
  and both terms below are blended.

Each frame, the muscle's *current* length is relaxed toward two different
targets simultaneously, weighted by `strength`:

```
length -= (length - relaxlength) * (1 - strength) * multiplier * 10000
length -= (length - targetlength) * strength      * multiplier * 10000
```

`relaxlength` pulls toward the joint's neutral/rest separation (ragdoll-
like slack); `targetlength` pulls toward whatever the current animation
keyframe (or an active-ragdoll/player-input target) wants — so `strength`
is a **continuous, per-muscle dial between "flop like a ragdoll" and
"snap to the authored pose,"** not a binary animated-vs-physics switch.
This is the mechanism that makes Lugaru's system a real *active ragdoll*
rather than "animation with a ragdoll fallback": the same joints and the
same constraint solve are live at all times, and `strength` (which can
vary per-muscle and be scripted over time, e.g. weakened on a hit,
restored on recovery) is the only thing that changes between "fully
animated" and "fully limp."

After the length update, position correction distributes the correction
between the two parent joints proportional to the *other* joint's mass
(`midp - vel * length * (parent2.mass / (parent1.mass + parent2.mass))`)
— a heavier joint moves less, exactly the momentum-conserving behavior a
real spring-mass system should have, and notably NOT what a naive
"average the two positions" constraint would give.

### `Skeleton::FindRotationMuscle()` — deriving visual orientation from physics state

Since joints only store `position` (a physics quantity) and rotation is
only stored as two authored `twist`/`twist2` scalars per keyframe, the
actual per-muscle rendering rotation (`rotate1`/`rotate2`/`rotate3`) is
*derived* every frame from the current physics-solved joint positions:
a vertical angle between the two parent joints, a horizontal XZ-plane
angle, and a forward-facing angle relative to the body's own `forward`
vector (with a special case for `hanganim`, where hands align to the
world X-axis instead of body-forward — a ledge-grab-style pose override).
This is the inverse of migera's/a conventional rig's flow: FK computes
position *from* rotation; Lugaru computes (visual) rotation *from*
already-physics-solved position.

## Relevance to migera

- **Not a drop-in replacement.** migera's `src/character/skeleton.rs`
  bone tree is deliberately Mixamo-standard so a real skinned glTF
  character can be swapped in later (see
  [migera pivots to Bevy PBR for characters](../hybrid-architecture/migera-pivot-to-bevy-pbr-for-characters.md)); Lugaru's joint/muscle model is
  fundamentally incompatible with that goal — skinning weights live on
  muscles (constraints), not bones, and rotation isn't a first-class
  per-joint quantity at all. Porting this wholesale would mean abandoning
  the Mixamo-compatibility goal.
- **The idea worth keeping is the continuous animated<->ragdoll dial**
  (per-constraint `strength` blending `targetlength` against
  `relaxlength`), which is a materially different design from botica's
  own active-ragdoll approach (`RagdollMode::{Animated,Blending,Active,
  Passive}` as discrete named states — see
  [botica's character animation system](./botica-character-animation-system.md)) or a typical Bevy/avian3d
  approach (separate kinematic-animated vs. dynamic-ragdoll rigidbody
  modes, switched wholesale). A continuous per-joint dial is strictly
  more expressive (e.g. "arms stay physical while legs stay animated,
  blending independently and changing over time") at the cost of needing
  an iterative constraint solver in the animation update itself, not just
  at handoff time.
- **If migera ever wants hit reactions/damage response to fall out of
  physics rather than a scripted state machine** (unlike botica's
  `hit_reaction` module), Lugaru's damage-from-impact-velocity approach
  in `DoConstraints()` is the concrete precedent to study further.
- **Twist-angle keyframes are a compression trick specific to a
  physics-solved rig** — they only work because the constraint solve,
  not the stored rotation, determines final bone orientation. Not
  applicable to migera's plain-FK rig, where `Transform.rotation` is the
  authoritative per-bone quantity and must be a real quaternion/Euler
  value.

## Open questions (not answered by the files fetched so far)

- Exactly how/where `strength` is scripted per-muscle in response to
  combat hits or player input (the `.cpp` files read so far show the
  mechanism, not the call sites that drive it during gameplay).
- The full `bodypart` enum and complete muscle graph topology (which
  muscles are `boneconnect` vs `constraint` vs `muscle` for a full
  humanoid) — only the type semantics were confirmed, not the full rig
  layout.

## Twist/roll: NOT physically simulated (source-verified, 2026-09-21)

Directly relevant follow-up after hitting a real instability trying to add
a driven twist DOF to migera's own avian3d rig (see
[migera pivots to Bevy PBR for characters](../hybrid-architecture/migera-pivot-to-bevy-pbr-for-characters.md)): re-checked against the current
official mirror (`github.com/WolfireGames/lugaru`, layout since moved to
`Source/Animation/*` from the older `Graphics/App` split cited above).

**Lugaru only ever physically integrates joint `position` (3 DOF via the
mass-spring `Muscle::DoConstraint()` length solve) — rotation, including
twist/roll, is never a simulated quantity.** `Skeleton::FindRotationMuscle`
(`Source/Animation/Skeleton.cpp:520-631`) derives ALL THREE rendered
rotation components purely as a stateless trig readout of the two parent
joints' already-physics-solved positions, run strictly AFTER
`DoConstraints()`/`DoGravity()` finish each tick (call sites confirmed in
`Person.cpp:1739,4376,6445`, all downstream of the physics pass):

- `rotate2` (vertical swing angle) and `rotate1` (horizontal swing angle):
  `asin`/`acos` of the normalized vector between the two joint positions.
- `rotate3` (the twist/roll analog): a per-bodypart *reference forward
  vector* (`specialforward[0..4]`, chosen by joint label — head/arm/leg
  get different statically-chosen reference vectors) is rotated by the
  already-solved `rotate1`/`rotate2` swing, then compared (via another
  `acos`) against the rest-pose reference to read off how far it's
  twisted. This has NO spring, NO target, NO velocity term of its own —
  it cannot fight the swing solve because it isn't a second constraint, it
  has no state to converge.

The `twist`/`twist2` scalar fields on `AnimationFrameJointInfo` (described
above, loaded from the `.anim` binary format) turned out to be **dead code
in the current mainline** — parsed off disk in `Animation.cpp`'s
`loadBaseInfo`/`loadTwist2` and never read anywhere else in
`Skeleton.cpp`/`Muscle.cpp`/`Person.cpp`/`GameTick.cpp`/`GameDraw.cpp`
(confirmed by grep across all of them). Only `AnimationFrameJointInfo`'s
own `.position` field is ever used downstream (as a target for
`Joint::position`). This contradicts this doc's own earlier "Animation"
section above, which took the struct's field existence at face value
without checking whether they were load-bearing — a caution about
inferring behavior from data-format fields alone, not just source
presence.

**Why this matters for migera**: the instability found while implementing
a twist DOF as a second, independently-simulated `RevoluteJoint` in series
with the existing swing joint (two stiff spring-dampers sharing one small
intermediate body, each with its own target/state) is exactly the class of
problem Lugaru's design cannot have, by construction — it never puts a
second rotational DOF into the physics solve at all. The applicable fix,
if twist/roll is revisited: don't give twist its own simulated joint state;
derive it as a read-only function of the swing joint's already-settled
orientation each frame (a reference vector carried through the solved
swing, compared against a rest-pose reference), computed AFTER the physics
step, not co-simulated alongside it.

## The mass-spring/muscle system ONLY runs during ragdoll (source-verified, 2026-09-21)

Directly resolves a real dead end hit while porting this design to
migera as a from-scratch replacement for the earlier avian3d-based rig
(see [migera pivots to Bevy PBR for characters](../hybrid-architecture/migera-pivot-to-bevy-pbr-for-characters.md)): after building the full
20-joint/45-muscle topology (parsed straight from `Data/Skeleton/
BasicFigure`'s binary format — 45 muscles vs. the 19 a spanning tree would
need, confirming real cross-bracing: pelvis ring, shoulder ring, hip-to-
opposite-shoulder/knee diagonals, contralateral ankle/knee braces) and
correctly pinning both feet to the ground as a second anchor (fixing the
legs completely — verified: they now hold under gravity), the torso and
arms still collapsed forward even at `strength=1.0`, with no comparable
second anchor point to mirror what fixed the legs. Re-checked source
directly (own reads via `raw.githubusercontent.com` + an independent
agent's parallel read, both converging on the same conclusion):

- `Skeleton::DoConstraints` (`Source/Animation/Skeleton.cpp` — the whole
  per-joint gravity/velocity/`Muscle::DoConstraint` relaxation block) is
  gated by `if (free) { ... }`. `free` is Lugaru's own ragdoll flag. The
  function's own doc comment literally says "used for ragdolls?".
- When `!free` (the ordinary animated/standing state — i.e. exactly what
  this milestone's own "hold T-pose under gravity" test scenario actually
  is), only `boneconnect`-type muscles run, which are rigid always-
  strength-1 bone links for SKINNING, not a compliant spring resisting
  gravity.
- Non-ragdoll joint positions come directly from `Person::DoAnimations`
  (`Source/Objects/Person.cpp`): `skeleton.joints[i].position =
  currentFrame().joints[i].position * (1-target) +
  targetFrame().joints[i].position * target;` — pure keyframe
  interpolation of AUTHORED joint positions. Gravity/spring physics never
  touch these joints at all while `free == 0`.
- `grep -n targetlength` across `Person.cpp`/`Animation.cpp` returns ZERO
  hits — `targetlength` is set exactly once, at skeleton-file load time
  (`Muscle::load`), and only ever consumed inside `Muscle::DoConstraint`,
  which — per the point above — only executes during ragdoll.
- The legs specifically get EXTRA hand-written geometric knee-locking
  code (`Skeleton.cpp`, an explicit `sphere_line_intersection` loop) with
  no shoulder/arm equivalent anywhere in the file — this is exactly why
  pinning migera's own feet fixed the legs but not the arms: Lugaru's own
  legs get bespoke support code even during ragdoll that its arms never
  do.

**Conclusion**: Lugaru NEVER holds any pose — torso, arms, OR legs — via
static muscle rest-lengths resisting gravity during normal (non-ragdoll)
gameplay. "Hold a T-pose rigid via muscle strength alone, under gravity,
with no active animation driving it" is not a scenario the real engine
ever runs — the milestone's own test scenario was testing something
Lugaru's actual design never does, not chasing a real bug. The mass-
spring/`strength`/bracing system's real, intended use is ragdoll physics
(does an already-limp or actively-struck body move believably), while
holding an authored pose (T-pose, a walk cycle, an idle stance) is meant
to be driven by keyframe/target-position authoring layered on top,
exactly like `Person::DoAnimations`'s own interpolation — not by static
bracing fighting gravity. **For migera, this reframes the muscle-rig
milestone's own scope**: the mass-spring solver (`solve_muscle.rs`) is
correctly designed for ragdoll-style behavior (proven: the triangulated-
shape test holds, the two-pinned-feet legs hold under gravity), but
"T-pose under gravity at strength=1" was the wrong acceptance test for
it — holding an authored pose needs a target-position-driven layer on
top (each muscle's `target_length`/`Joint.position` continuously
re-authored toward a keyframe, exactly like Lugaru's own
`DoAnimations`), which is a different, not-yet-built feature, not a bug
in the ragdoll solver itself.

## Keyframe-driving layer built and landed (2026-09-21, same day)

Directly follows the finding above. `solve_muscle::blend_toward_target_positions`
is the analogue of `Person::DoAnimations`'s own keyframe interpolation
(`joints[i].position = currentFrame*(1-t) + targetFrame*t`), called every
frame AFTER `solve_step` (the ragdoll/bracing physics), blending each
non-pinned joint's PHYSICS-SOLVED position toward its own authored target
(currently only `Bone::t_pose_world_position` — no other pose authored
yet) by `strength`, and blending velocity toward zero by the SAME weight
(not re-derived from the position delta the way `solve_step`'s own
velocity reconciliation works — a delta-based approach would wrongly zero
real physics velocity at `strength=0`, since the blend leaves position
completely unchanged there).

Unlike Lugaru itself — which is 100% kinematic when animated and 100%
physics when ragdolling, NEVER blended — migera's own `strength` dial is
a deliberate generalization: `strength=1.0` fully snaps to the keyframe
target (matching Lugaru's kinematic mode), `strength=0.0` leaves physics
fully untouched (matching Lugaru's ragdoll mode, the part already proven
working), and values between genuinely blend "how much does gravity fight
the authored pose" — a continuous dial neither of Lugaru's own two binary
modes has.

Result, verified both numerically and visually: `--muscle-strength 1.0`
now holds an EXACT T-pose (`max T-pose deviation` reads exactly `0.0`
degrees, `muscle max speed` reads exactly `0.00` m/s, screenshot confirms
a fully rigid standing T-pose) — a genuine fix, not the "collapsed/
toppled under gravity" state every earlier attempt in this same
investigation produced. `--muscle-strength 0.0` still ragdolls correctly
(arms collapse under gravity, legs stay planted via the still-pinned
feet, matching earlier ragdoll verification). A real, separate bug was
also found and fixed along the way: `step_muscle_sim` applied gravity to
EVERY joint's velocity unconditionally, including pinned ones (`Bone::
Hips`, feet) — harmless to their POSITION (pinned joints never integrate
position from velocity regardless), but their own `velocity` field kept
accumulating `gravity * dt` forever, which is why `muscle max speed` read
a nonzero, never-decaying value even once the T-pose fix made position
genuinely exact — fixed by skipping gravity accumulation for infinite-
mass joints.

## Twist authored (not derived) + multi-keyframe clip playback (2026-09-21, same day)

Two follow-ups landed together, both building on the keyframe-driving
layer above. `Pose` gained a second sparse channel, `twists: HashMap<Bone,
f32>` (radians), alongside its existing `offsets` — swing (a direction
vector) has no notion of roll around itself, so a forearm's target
POSITION alone can never distinguish "palm up" from "palm down".
`write_muscle_sim_to_bones` applies twist strictly AFTER computing the
swing rotation (`local_rotation`), as an extra `Quat::from_axis_angle`
around the bone's own REST direction (which, composed before the swing
quaternion, is equivalent to twisting around the already-solved swing
axis) — never fed back into the position solver. This deliberately
DIVERGES from Lugaru's own `rotate3`, which is a stateless DERIVED trig
readout (a per-bodypart reference-forward vector rotated through the
solved swing, compared against its rest orientation) — this rig has no
such per-bodypart reference-vector table, so twist here is simply
AUTHORED directly per pose instead of derived. The load-bearing property
carries over unchanged either way: twist is computed strictly after swing
and has no spring/target/velocity of its own, so it structurally cannot
fight the swing solve, which is exactly why the earlier avian3d two-joint
instability doesn't recur. `wave_pose` was extended to author a
`FRAC_PI_2` twist on `RightForeArm`/`RightHand` (previously undefined/
arbitrary palm orientation), verified visually via the HUD's own
per-bone rotation readout (`RightForeArm`/`RightHand` roll now reads
~103°/84° vs. `RightArm`'s ~52° — distinctly different, proving twist is
applied per-bone and not just inherited from swing).

Separately, `AnimationClip`/`ClipPlayback` (`pose.rs`) generalize a single
`PoseTransition` into an ordered sequence of `(Pose, segment_duration)`
keyframes, optionally looping — `ClipPlayback` holds one internal
`PoseTransition` and retargets it to the next keyframe once the current
segment completes, reusing the exact same smoothstep-eased blend a manual
`switch_pose` call already gets, just automatically per segment.
`MuscleConfig::play_clip`/`play_idle_loop`/`stop_clip` wire this into the
existing `MuscleConfig` (a new `active_clip: Option<ClipPlayback>` field
takes precedence over `active_pose` whenever `Some`, with a no-jump
handoff back to a settled single pose on `stop_clip` or a manual
`switch_pose` call). `idle_loop()` is a deliberately small 4-keyframe
standing sway (T-pose / lean-right / T-pose / lean-left, looping) — NOT a
walk cycle: `GROUND_CONTACT_BONES` (see `plugin.rs`) pins all 4 foot/toe
bones completely rigid, so this rig structurally cannot lift a foot off
the ground yet, which any real walk cycle needs. Visually verified across
multiple `--shot` frames of `--muscle-clip idle`: the sway is visibly
smooth and asymmetric frame-to-frame (no snapping), confirming looped
multi-segment playback works, not just a single retarget.

## Casual idle redesign: research-backed motion + a real relaxed baseline (2026-09-21/22)

User feedback on the first `idle_loop()` (above): it "still reads as a
T-shape" — correctly diagnosed via screenshots as two separate problems,
both fixed:

**1. Motion itself was too uniform/small.** Researched real idle
construction (Mixamo/mocap-industry breakdowns — AnimSchool, MoCap
Online, garagefarm.net) and confirmed via a direct fetch of Lugaru's own
`Data/Animations/Idle` binary (`github.com/osslugaru/lugaru`, 16
joints/20 frames per its own header) that even Lugaru's idle is plain
keyframed position interpolation, not physics — the mass-spring system
never runs outside ragdoll (already established above), so there was
nothing further to port from Lugaru's OWN idle specifically; the
actionable numbers came from the general-principles research instead.
Key findings applied: **amplitude hierarchy** (weight-shift biggest,
breathing/spine ~1-2cm, head bob smallest), **layered non-synchronized
cycles** (breathing ~3-4s rides on top of a slower ~6-8s weight-shift,
not one shared clock), and **deliberate left/right asymmetry** (mirrored
motion is the single biggest tell of a robotic idle). Since `Bone::Hips`
itself is pinned (this rig cannot translate its pelvis or lift a foot —
`GROUND_CONTACT_BONES`'s own doc comment), contrapposto weight-shift is
faked via a whole-upper-body `lean_pose` (cascading offset scaled by
height above the `Spine1` pivot, approximating a rigid lean) plus a
knee/foot offset on the non-weight-bearing leg — not an actual hip
translation. `idle_loop()` now runs 7 keyframes (`settle_right` -> 2
breathing keyframes -> a center pass -> `settle_left` -> its own 2
breathing keyframes, looping) instead of the original 4, with left/right
arms never given identical offsets at any keyframe.

**2. The real fix, per direct user diagnosis**: every pose (including the
redesigned one above) was still built as a tiny delta ON TOP OF raw
`t_pose()` — arms-out-horizontal, a RIGGING reference pose, not something
any real standing character holds — so no amount of tuning the sway
itself could stop it reading as "a slightly wobbly scarecrow." Fixed by
adding `relaxed_stand()`: a new baseline pose with both arms brought down
to hang naturally at the sides (elbow softly bent, hand ending near the
thigh), authored the same way `wave_pose` computes offsets (target world
position minus each bone's own known T-pose position). `lean_pose` and
`idle_loop`'s own center-pass keyframe were reworked to build on top of
`relaxed_stand()` instead of `t_pose()` — `t_pose` itself never appears
anywhere in the idle loop anymore. Also exposed as its own selectable
pose (`--muscle-pose relaxed`, egui "Relaxed" button) alongside `t_pose`/
`wave_pose`, independent of the idle clip.

Visually verified via `--shot` at multiple frames across a full ~13s
loop: arms now hang down and stay down through breathing/weight-shift/
center-pass keyframes alike, with a visible (if subtle) leg-weighting
difference between the two settled sides — reads as a standing character
at rest, not a mannequin holding a T. 30 pose-module tests pass (7 new,
covering weight-shift being the single biggest offset in a settled
keyframe, breathing lifting the spine more than the head, left/right
asymmetry, and the loop never revisiting literal T-pose).

## Foot-planting: ground-locked dynamic joints replace permanent pins (2026-09-22)

Directly unblocks the item flagged repeatedly in earlier entries above
("feet can never lift off the ground at all with this mechanism, ruling
out locomotion"). Re-checked Lugaru's real foot-locking mechanism from
source first (`github.com/osslugaru/lugaru`,
`Source/Animation/Skeleton.cpp`'s `DoConstraints`) rather than assuming
the earlier high-level summary ("locks a joint once it settles, velocity
below a threshold") was the whole story — it wasn't:

- Lugaru's own `Joint::locked` mechanism is **ragdoll-only physics**,
  gated by `if (free)` (same gate as the mass-spring system covered
  above) — a joint auto-locks once its POST-CONTACT velocity² drops below
  `1` (gated on an actual ground-height/`LineCheckPossible` swept-collision
  test, not a bare velocity heuristic), and auto-unlocks once pushed back
  above `320`/`600`. Position is NOT frozen at a captured point while
  locked — it keeps integrating and gets re-clamped to the ground surface
  every step.
- Critically, this is **not** what drives ordinary walk-cycle foot
  planting in Lugaru — that's pure keyframe position interpolation
  (`Person::DoAnimations`, already covered above), with NO physics gate at
  all. The animation-file `onground` field this doc's own "Animation"
  section originally assumed was the walk-cycle signal turned out to be
  genuinely dead/unused data (confirmed by grepping the whole
  `Person.cpp`/`GameTick.cpp`/`GameDraw.cpp` call graph) — a second
  instance of this doc's own earlier caution ("a caution about inferring
  behavior from data-format fields alone, not just source presence").

Given that, migera's own foot-planting is a NEW design inspired by the
SHAPE of Lugaru's ragdoll lock (asymmetric lock/unlock velocity
thresholds, gated on real ground proximity), not a port of an existing
walk-cycle mechanism (none exists in the source to port). Simplified vs.
the original in two ways this project doesn't need yet: a plain height
compare against a known flat ground plane (`y = 0`) stands in for
Lugaru's own terrain/collision queries, and there's no `head`/`groin`
instant-lock-on-impact special case (landing-damage reaction, out of
scope for a standing idle).

`solve_muscle::Joint` gained a `locked: bool` field (separate from `mass
== f32::INFINITY` pinning — `Bone::Hips` stays permanently pinned and
can NEVER move; a locked foot is a normal dynamic joint that CAN unlock).
`solve_muscle::apply_ground_lock` is a new pass, run every frame AFTER
both `solve_step` and `blend_toward_target_positions` (mirroring those
two functions' own existing separate-pass convention) — running last is
what lets an actively-lifting keyframe target win: nothing about `locked`
blocks `blend_toward_target_positions` from pulling a foot upward, and
the unlock check inside `apply_ground_lock` reads the velocity that
resulted from THAT blend, so a foot a keyframe is actively lifting
unlocks itself rather than fighting a stale lock from the previous frame.
`plugin::GROUND_CONTACT_BONES` feet now spawn via `Joint::dynamic_locked`
(dynamic, but starting already ground-locked, matching a rig that spawns
already standing at rest) instead of `Joint::pinned`.

`idle_loop()` was extended with a small proof-of-concept: once per
settled side, the ALREADY non-weight-bearing foot briefly lifts a few cm
and resettles at the exact same spot (a weight-shift foot tap, not a
step — no forward stride). Verified end-to-end via a throwaway scratch
harness (`examples/_dump_foot_lift.rs`, deleted after use) driving
`MusclePlugin` headless and logging `MuscleSim::position_of(Bone::
LeftFoot).y` against elapsed time: the foot's Y genuinely rises (e.g.
`0.0000 -> 0.0167`, target `0.0417`, during the `t≈5.1-5.6s` lift window)
then returns to exactly `0.0000` on resettle, mirrored for the right foot
at `t≈12.0-12.8s` — real physics-driven lift-and-land, not just a
keyframe target that never actually moves the joint. Separately verified
the STANDING case (no idle clip) still holds perfectly after switching
from a permanent pin to ground-locking: 11 seconds in, `muscle max speed`
reads exactly `0.00` m/s and every bone length matches its own rest
length — ground-locking is a strict superset of what permanent pinning
already did for a standing rig, while also (unlike a permanent pin)
allowing an active lift. 11 new `solve_muscle` tests (lock/unlock
thresholds, height gating, partial-pull-not-instant-snap, per-joint
lockable-list scoping) plus 5 new `pose` tests (lift/resettle at the
exact same spot, weight-bearing foot staying put during the OTHER foot's
tap) — 438 total tests passing, zero regressions.

## Idle variety: an occasional secondary variant spliced into the base loop (2026-09-22)

Directly implements the research's own "secondary/randomized layer"
finding (a real idle avoids repetition by cycling one base loop with
occasional secondary variants triggered every few loop cycles, rather
than looping one clip forever unchanged — a single loop becomes
noticeably repetitive after ~90 seconds). First variant:
`pose::idle_variant_look_around()` — a short (2.1s), NON-looping clip
that turns the head/neck to look aside, holds briefly, then returns to
`relaxed_stand()`'s own center and stays there. The turn itself is
**twist**, not a position offset (`Pose::twists`, already covered above)
— a head turning left/right rotates around the neck's own near-vertical
rest axis, which a swing/position offset structurally cannot express
(swing only changes which direction a bone points, never rotation around
its own length). `Neck` gets a smaller twist than `Head` (a real turn
cascades — the head contributes more than the neck below it), the same
height-scaled-cascade idea `lean_pose` already uses for position, applied
to twist instead.

Two small additions to `pose::ClipPlayback` made the automatic cycling
possible without `AnimationClip`/`ClipPlayback` themselves knowing
anything about the splicing POLICY:
- `loops_completed()` — increments exactly once per full wrap back to
  keyframe 0 (meaningless/always `0` for a non-looping clip).
- `is_finished()` — `true` once a NON-looping clip has fully settled on
  its own final keyframe (always `false` for a looping clip, which never
  "finishes" by definition).

`MuscleConfig::play_idle_loop()` now also starts an `idle_variant_phase`
state machine (`BaseLoop { next_variant_after_loops }` /
`PlayingVariant`), driven once per frame by the new private
`advance_idle_variant_cycle` (called from the existing
`advance_pose_transition`, right after the active clip's own `advance`,
so a splice this frame takes effect immediately rather than one frame
late): once the base loop's own `loops_completed()` reaches a threshold,
splice to `idle_variant_look_around()`; once THAT clip's own
`is_finished()` becomes true, splice back to a fresh `idle_loop()` with a
NEWLY drawn random threshold (not the same fixed count reused forever —
a perfectly regular cadence would itself become a second, more subtle
repetitive tell). The threshold itself is randomized within
`IDLE_VARIANT_LOOP_RANGE` (`3..=6` loops) via `fastrand` (already a
direct dependency, `Cargo.toml`'s own `fastrand = "2"` — no new
dependency needed). Both splice directions reuse `play_clip`'s own
no-jump current-pose handoff (already proven correct by the earlier
pose-transition work above) — never a jump cut in either direction. A
manual `switch_pose`/`play_clip`/`stop_clip` call clears
`idle_variant_phase` entirely, so a caller taking explicit control never
has the automatic cycle silently resuming underneath it.

**A real test-design bug found and fixed during verification** (not a bug
in the shipped mechanism itself): an initial test asserted the variant
would appear after a single big `advance_seconds(&mut cfg, 200.0)` jump,
checking only the state AFTER that jump completed. It failed
intermittently — not because splicing was broken, but because the
variant clip only occupies a ~2.1s window inside a much longer (~50-75s
at the high end of the threshold range) base-loop cycle, so a single
end-of-jump check can alias right past the entire window without ever
observing it, the same way a low sample rate can miss a narrow signal
spike entirely. A fine-grained trace (checking state every 0.5s) proved
the mechanism itself worked exactly as designed — the variant spliced in
at t=79.5s, held, returned to center at t=81.5s, and repeated at t=134.5s
and t=189s, precisely matching the random thresholds drawn each cycle.
Fixed by rewriting the tests to check EVERY intermediate step for "was
the variant ever seen," not just the final state after a long jump — a
lesson in testing anything with a narrow/bursty state window against
continuous, not sampled, observation.

456 total tests passing (10 new: 4 `pose::ClipPlayback` tests for
`loops_completed`, 4 for `is_finished`, plus 6 `plugin::MuscleConfig`
tests for the splice cycle itself — the manual-override tests, the
eventually-splices test, and the round-trip-back-to-base-loop test all
now correctly watch continuously rather than sampling at the end of a
long jump).

## Real forward-walking locomotion: Hips becomes ground-locked, not permanently pinned (2026-09-22)

Directly unblocks the last remaining item flagged since the earliest
milestones ("feet can never lift off the ground at all... ruling out
locomotion"). The deeper reason locomotion was blocked wasn't the feet at
all (foot-planting, above, already solved that) — it was `Bone::Hips`
itself, permanently `Joint::pinned` at world origin since the very first
milestone, with no mechanism to ever move. A rig whose root can never
translate cannot walk anywhere by definition, regardless of how well its
legs swing.

**The fix reuses proven machinery rather than inventing a second root-
motion system**: `Bone::Hips` now uses the exact same `Joint::
dynamic_locked` + `solve_muscle::apply_ground_lock` mechanism the feet
already use (a second `apply_ground_lock` call in `step_muscle_sim`,
locked toward hip height `HIPS_GROUND_HEIGHT = 0.94` instead of sole
height `0.0` — `apply_ground_lock` takes one shared height per call, so
bones locking toward different heights need separate calls). This
required one more small addition nothing had needed before:
`write_muscle_sim_to_bones` now writes `Bone::Hips`'s own
`Transform.translation` from its solved position every frame (every
OTHER bone gets its position "for free" via `ChildOf`/`TransformSystems::
Propagate` off a fixed, never-changing `t_pose_offset()` translation —
only the ROOT bone has no parent to inherit a position from, so once it's
allowed to move at all, its own translation has to be written explicitly
or the rig would keep rotating in place at a fixed world position
forever).

**The walk cycle itself** (`pose::walk_step`, `pose::WalkSide`): a single
non-looping, 2-keyframe half-cycle (mid-swing lift-and-carry-forward,
then replant a full `STRIDE_LENGTH` ahead) — deliberately modest scope,
matching the agreed plan: straight-line travel only, no turning/steering,
fixed pace, reusing the idle loop's own contact-phase alternating-leg
structure and ground-lock-driven lift/replant mechanism (the SAME
mechanism the idle loop's own foot-tap already proved out — a walk step
is just a foot-tap that lands somewhere NEW instead of back at its own
starting spot). `Pose::translated(delta)` is the one new primitive this
needed: rigidly shifts every bone a pose already touches by a fixed
delta, letting `walk_step(side, base_progress)` build the exact same
relative shape at any point along an indefinitely long walk, just shifted
`base_progress` meters further forward — accumulated externally by
`plugin::WalkCyclePhase`/`MuscleConfig::advance_walk_cycle` (mirroring
`IdleVariantPhase`'s own splicing structure, just simpler: no random
variant selection, only "which side swings next" alternates
deterministically every half-cycle, re-derived each time directly from
the just-finished half-step's own settled `Bone::Hips` Z position rather
than tracking a separate redundant running total).

**Two real physics bugs found and fixed via test-driven design**, both
caught by unit tests before ever reaching the example:
1. `walk_step`'s own `base` pose only explicitly touched `Bone::Hips`
   before translating — `Pose::translated` only shifts bones ALREADY
   present in a pose's own offsets map (documented behavior, not a bug in
   `translated` itself), so the legs/feet (never touched by
   `relaxed_stand`) silently failed to receive the shared forward carry.
   Caught by `walk_step_replants_the_swinging_foot_a_full_stride_ahead_
   of_where_it_started` measuring the foot landing exactly
   `STRIDE_LENGTH * 0.5` short of its expected spot. Fixed by explicitly
   including every walked-on bone (`Hips` + both legs/feet) in `base`
   with a zero offset before translating.
2. After fixing (1), the REPLANT keyframe's own `base.translated(...)`
   call also shifted the PLANTED foot's own world position forward by
   half a stride — physically backwards: a ground-locked planted foot
   must stay exactly where it landed while the body advances OVER it, it
   does not slide forward with the hips. Caught by
   `walk_step_never_moves_the_planted_foot_beyond_the_shared_forward_
   carry`. Fixed by giving `Hips` (and only `Hips`) its own explicit
   extra offset in the replant keyframe, rather than translating the
   whole shared `base` pose (which would carry every bone, planted feet
   included).

Visually and numerically verified via a throwaway headless harness
(`examples/_dump_walk_cycle.rs`, deleted after use): over 20 seconds of
real `MusclePlugin`-driven physics (not just pose-target math), `Bone::
Hips`'s own SOLVED position traveled from `z=0.000` to `z=-7.000`
(7 real meters forward), `hips.y` stayed rock-solid at exactly `0.940`
throughout (no vertical sag/drift), both feet visibly alternated
lifting (`y` rising to `~0.02-0.04`) and planting (`y=0.000`) in lockstep
with the stride pattern, `x` never drifted sideways, and no NaN/explosion
occurred. Also re-verified via `--shot` that the existing standing
(T-pose/`relaxed_stand`) and idle-loop behavior are UNCHANGED after
swapping `Hips` from a permanent pin to ground-locking — 11+ seconds
standing still reports exactly `0.00 m/s` max speed and `Hips` still
holds exactly at `(0.00, 0.94, 0.00)`, proving ground-locking is a strict
superset of what the old permanent pin already did for a stationary rig.

11 new `pose` tests (`translated`'s own shift/no-op/twist-preservation
behavior, `walk_step`'s own forward-carry/stride-distance/lift/base-
progress/side-swap correctness) plus 5 new `plugin::MuscleConfig` tests
(walk-cycle splicing, monotonic forward progress, side alternation,
manual-override cancellation) — 470 total tests passing, zero
regressions. One more test-design lesson (distinct from the earlier
narrow-window-aliasing one): a boundary-exact `advance_seconds(&mut cfg,
0.7)` call (matching `walk_step`'s own EXACT 0.7s total duration) can
under-shoot by a hair due to `f32` summation drift across many small `dt`
steps, occasionally sampling the same still-in-progress half-step twice —
fixed by advancing slightly PAST the boundary (`0.75s`, not `0.7s`)
rather than exactly on it.

## Related

- [The muscle module is deleted](./animation-core/muscle-deleted-anim-is-the-only-stack.md) — superseded-by: the migera port logged above was deleted; read for what replaced it.
- [botica's character animation system](./botica-character-animation-system.md) — contrast: discrete ragdoll modes and a Mixamo-standard bone tree instead of particles and muscles.
- [Full-strength read-back hides the physics](./ragdoll-and-physics/full-strength-readback-hides-the-physics.md) — applies: how the strength dial carried into migera's ragdoll behaves, and how to verify it.
- [Walk-cycle IK and ground-lock bugs](./ik-and-locomotion/walk-cycle-ik-and-ground-lock-bugs.md) — example: bugs in the ground-locked walk of this port.
- [migera pivots to Bevy PBR for characters](../hybrid-architecture/migera-pivot-to-bevy-pbr-for-characters.md) — prerequisite: the rendering decision behind the Mixamo-standard rig.
