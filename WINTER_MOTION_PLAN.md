# Winter-grounded motion plan

What Winter's *Biomechanics and Motor Control of Human Movement* (and its
notes under `docs/knowledge/biomechanics-winter/`) can still ground in the
animation system, as ordered, gated steps. Progress with measured numbers
goes to [CHARACTER_PROGRESS.md](./CHARACTER_PROGRESS.md); this file only
tracks the plan and which steps are done.

Order: **1 → 2 → 3a → 4 → 3b → 5.**

## Scope findings

1. **Winter gives the walk's frontal/transverse pelvis *timing and
   direction*, not its angles.** §7.4.5 has hip moments only (abductor humps
   at ~12 % and ~45 % of stride; H1-F pelvic drop at ~10 %). No pelvic
   obliquity or rotation in degrees.
   - The lateral COM path *is* derivable: the inverted pendulum (§11.2.1,
     K ≈ 0.1 s²) with the COP switching feet every step (T ≈ 0.49 s) gives a
     periodic sway of `a · (1 − 1/cosh(T / (2√K)))`, `a` = half step width,
     i.e. ≈ ¼ of half the step width (~2–3 cm).
   - §11.3.1's "COM passes medial of each stance foot" becomes a test.
   - Any angle amplitude is a labelled non-Winter constant.
2. **The ragdoll root is pinned kinematic, and its ceilings are
   acceleration-shaped** (`ragdoll.rs`, `CEILING_SCALE = 12`, raised so the
   limbs track the walk). Winter's N·m/kg budgets must be converted through
   each body's inertia. Physiological budgets will likely not track the walk,
   so they apply only in a new balancing mode.
3. **Body proportions on a skinned mesh are a feasibility question.**
   Moving joints apart stretches the mesh at the joints; spike before
   committing.

## 1. Sideways motion in the walk (§11.3.1, §7.4.5, §11.2.1)

- [x] **1.1 Measure, no code.** On `puppet_base` and `character.glb`: the
  walk's step width; the hips' lateral track vs the stance foot's inner
  border; what the authored 0.05 rad Hips roll and 0.09 rad Spine1 twist do
  to the planted feet. Baseline trace of pelvis position/roll at 0.7, 1.2,
  1.6 m/s.

  **Results (2026-09-29, headless, `puppet_base`, target pose = gait +
  `PhaseLayer::locomotion()`; ignored probe
  `locomotion::tests::probe_frontal_walk_baseline`, delete at 1.5).**
  `character.glb` has no test fixture; it is checked live in 1.5.
  - **Step width 22.9 cm** (sole centrelines) at every speed: the feet
    track straight under the hip sockets (±11.4 cm). This is about twice a
    typical adult step width. Winter gives no number in cm. Fig. 11.7 has no
    scale bar and a stretched lateral axis, but in its ratios the COM comes
    within ~15 % of the step width of each foot's centreline.
  - **The pelvis does not move sideways**: ±1 mm. The COM moves ±0.8 cm,
    only through the roll and trunk, and peaks at the *opposite* heel
    contact (c = 0.5), a quarter step later than a pendulum's mid-single-
    support peak. The COM stays ≥ 5.6 cm medial of the stance sole's
    centreline.
  - **Pendulum prediction** (K = d/g = 0.104 s², d = 1.02 m COM above the
    ankle; step time = half of 1.30 / 1.10 / 0.99 s stride at 0.7 / 1.2 /
    1.6 m/s): peak lateral COM ±4.1 / ±3.2 / ±2.7 cm at this rig's width;
    ±1.8 / ±1.4 / ±1.2 cm at a 10 cm width.
  - **Hips roll: ±2.86°, pure roll, no yaw, mistimed.** Left socket is
    highest at left heel contact (c = 0) and level at mid single support
    (0.25). Through late left single support (0.25–0.5) the *stance* side
    is lower, the reverse of the H1-F pattern (swing side drops, peak
    ~10–12 % of stride).
  - **Pelvic yaw is 0.** Only Spine1 twists (0.09 rad).
  - **The authored layer moves each stance sole up to 45–48 mm in the
    target pose.** Both legs hang from the rolled Hips, so the foot IK
    and lock must absorb this live (live planted slide stays 1–3 mm). 1.3
    must re-solve the legs rather than leave it to the IK.
  - **Decision needed before 1.2:** keep the 22.9 cm step width (±3.2 cm
    sway at 1.2 m/s) or narrow the walk toward a human width (±1.4 cm).
    Narrowing is not Winter-grounded in cm; the book's check (COM medial of
    the stance foot's inner border) holds either way.
- [x] **1.2a Narrower step (decided 2026-09-29: "more human-like").**
  `stance::STEP_WIDTH = 0.57` of the hip-socket spacing, 13 cm on
  `puppet_base`, derived from Winter's constraint: the widest step at
  which the pendulum COM still passes medial of the stance foot's inner
  border. Measured on the rig's foot mesh: 11 cm wide, inner border 3.8 cm
  inside the sole centreline. Margin 4.4 / 8.5 / 10.9 mm at 0.7 / 1.2 /
  1.6 m/s; 12 cm would leave 1 mm at 0.7 m/s and 10 cm crosses the border.
  `stance::narrow_feet` turns each leg about its hip with the foot turned
  back; `WalkCycle` composes Winter's stride onto the narrowed legs.
  Test: `narrowed_feet_stand_the_step_width_apart_flat_and_mirrored`; the
  gait mirror test now compares true mirror images.
- [x] **1.2 Lateral pelvis path** (`phase.rs` `WalkSway`,
  `walk_sway_at`; `stance::sway_over_loaded_feet`). Done in the locomotion
  `PhaseLayer` rather than `walk.rs`: the layer has the stride clock,
  so it knows the stride time the pendulum needs. The COM is the trapezoid
  pressure path with each odd harmonic divided by `1 + K(2πk/T)²`, and
  `K = d/g` from the pose's own COM height. Faded with the gait weight in
  `PhaseLayer::between`. The legs turn about their ankles and each knee
  takes up its leg's millimetres (`keep_ankle`), so the feet stay exact.
  - Tests: `the_walking_sway_is_the_pendulums_periodic_path` (vs a direct
    finite-difference solve; a phase-shift sabotage fails it by 11.6 mm);
    `a_walking_body_sways_over_its_stance_feet_but_never_past_them`
    (sways 2.3 / 1.8 / 1.6 cm toward the stance foot; planted foot 0.02 mm
    in single support, ≤ 0.67 mm in double support; swing foot 0.01 mm).
  - Original plan text: The periodic
  pendulum solution, reversing at each heel contact, applied like
  `sway_over_feet` (legs turned about their ankles). Stored beside the bob
  as a per-cycle envelope, folded by phase.
  - Tests: `the_walking_pelvis_sways_toward_the_stance_foot` (phase;
    amplitude ±20 % of the pendulum value);
    `the_walking_com_stays_medial_of_the_stance_foot` (via
    `anthropometry::centre_of_mass`); `a_swaying_pelvis_does_not_weave_the_body`
    (root motion: the hips' `root_translation` and the leg turn must cancel).
  - Existing planted-foot (< 2 mm), mirror and velocity-jump tests stay green.
  - Live (1.2 m/s, A/B vs the previous build): step width 230 → 134 mm,
    pelvis sway 36.8 mm peak to peak, start slide unchanged. Front + Left
    gizmo views checked. `anim_bench` 2.4 → 3.8 µs per character.
  - **Open, predating this (found by the A/B):** the root weaves ~90–100 mm
    sideways in a steady walk (heading ±14°) — *fixed by 1.3: it was the
    authored roll*. The last step glides 7–9 cm near the floor after the
    root stops — still open.
- [x] **1.3 done (2026-09-29).** The roll is integrated from Winter's
  frontal hip power over moment: swing side lowest 3.9° at 17 %, 0.74° low
  at its heel contact (`phase::pelvic_obliquity_at`). The amplitude comes
  from Winter too, not a labelled constant. It rolls about the loaded hip
  socket, in one pass with the sway (`stance::move_pelvis_over_feet`).
  `Bone::Spine` is on the hips' 0.015 s spring: at 0.16 s the rendered
  trunk rolled 8.1° with the pelvis. Planted soles ≤ 0.6 mm (was
  45–48 mm); live root weave 101 → 14 mm; trunk 5.9° → 1.5°;
  `anim_bench` 4.2 µs. Tests:
  `the_pelvis_drops_on_the_swing_side_then_is_lifted_back`,
  `a_walking_pelvis_drops_on_the_swing_side_with_the_trunk_upright`,
  `the_rendered_trunk_stays_upright_while_the_pelvis_rolls`.
- **1.3 as planned:** (§7.4.5 H1-F). Replace the
  authored Hips roll oscillator (`phase.rs::locomotion`) with a gait-phase
  roll: drop from stance heel contact to a peak at ~10–12 % of stride, lift
  through the ~45 % abductor hump. Amplitude 0.05 rad, labelled non-Winter.
  Legs re-solved to their feet like `shift_weight`.
  - Tests: `the_pelvis_drops_on_the_swing_side_in_early_stance` (sign and
    peak time); swing-toe minimum clearance not below today's.
- [x] **1.4 done (2026-09-29).** Winter times the turn (H1-T: it reverses
  at heel contact) but cannot size it: the transverse hip angle includes
  femoral rotation. So ±4° (`PELVIC_ROTATION`, Perry, labelled). The turn
  is composed with the roll in `move_pelvis_over_feet`; the chest twist is
  retimed to peak at the heel contacts against the pelvis. `Spine1` is on
  the arms' 0.03 s spring: at 0.16 s the rendered chest kept ±1.8°,
  uncorrelated with the pelvis. Live: pelvis ±3.9°, chest ±4.7°,
  correlation −0.97. Tests:
  `the_pelvis_turns_with_the_stepping_leg_and_the_chest_against_it`,
  `the_rendered_chest_turns_against_the_pelvis_on_time`.
- **1.4 as planned:** (§7.4.5 hip rotators). Pelvis
  yaw in step with the stride, Spine1 counter-twist kept, thighs
  compensated so the planted foot holds. Check the sprung pose, not the
  target ([a lagging pelvis rotation slides planted feet](./docs/knowledge/character-animation/ik-and-locomotion/a-lagging-pelvis-rotation-slides-planted-feet.md)).
  - Tests: `the_pelvis_yaws_with_the_swing_leg`; planted slide unchanged.
- [x] **1.5 done (2026-09-30), on `puppet_base`.** Live, each speed from
  a standing start (BRP):

  | | 0.7 m/s | 1.2 m/s | 1.6 m/s |
  |---|---|---|---|
  | step width | 130 mm | 128 mm | 130 mm |
  | pelvis sway p-p | 47 mm | 43 mm | 34 mm |
  | roll in single support | ±2.8° | ±2.7° | ±2.6° |
  | pelvis–chest correlation | −0.98 | −0.97 | −0.97 |
  | steady planted slide | 5.5 mm (was 5.7) | 3.3 mm | 7.1 mm (was 5.0) |

  About 2 mm more slide at 1.6 m/s: a small rendered-pose cost of the
  extra pelvis motion, not yet chased. The probe is removed.
  **`character.glb`: open, predates this plan.** Its live walk is broken
  on the committed pre-plan build too (A/B, same schedule): 0.27 m/s where
  1.2 was asked, the hips 63° off the travel, planted feet sliding
  ~200 mm. On the current build it walks sideways (83°) and its legs sink
  below the floor in the Front view. Standing is fine. Cause not yet
  investigated; it needs its own step before anything is judged on that
  rig. **Fixed 2026-09-30:** the live rig geometry had centimetre offsets
  and the synthetic hips height. It now walks at 1.18 m/s with a 5 mm
  slide, and step 1's motion checks out on it. The foot IK's toe contact
  now uses the walk's `Sole` (it was 15 mm off on `puppet_base`); standing
  feet rest at the assets' bind heights. The stop's last foot is set down
  onto its spot by the foot IK on the rendered foot (glide 9–15 → ~3 mm);
  the "heel under raised ground" was the synthetic rig (real rig: whole
  sole on the plane). Check every later step on both rigs.
- **1.5 as planned:** Live check. BRP capture with `--anim-speed-schedule` (start and
  stop). Front + Left, `--gizmos on --show-real-mesh off`. Claim: "the
  pelvis moves toward each stance foot and drops on the swing side, and the
  feet do not slide." `anim_bench`, CHARACTER_PROGRESS, KB note, update the
  Winter 11.3.1 / 7.4.5 notes. Checkpoint: commit.

## 2. Push recovery while standing (§5.2.9, §11.2.1)

**Done 2026-09-30** (2.1–2.3 below, as planned, with two changes):
- A push is delivered over 0.1 s; in one frame it released the foot locks
  (feet re-planted 23 mm away).
- A push the feet cannot absorb is flagged (`needs_step`) and the COP is
  unclamped as the step's stand-in. Held at the edge, the body hung there
  forever.

Live on both rigs: pushes of 0.2–0.3 m/s peak at 2–3 cm (`puppet_base`)
and return; feet ≤ 1.3 mm (balls), ≤ 4.9 mm (ankles). The support is also
bounded to Winter's 8° per plane. Tests in `balance::tests`.

- [ ] **2.1 Balance state** (new `balance.rs`). `Balance` component: COM
  offset and velocity (A/P, M/L), pendulum `COM̈ = −(COP − COM)/K`. COP
  controller `COP = COM + gains·(offset, velocity)`, clamped to the support
  polygon from the `Sole` points less a margin. `Balance::push(impulse, mass)`.
  - Tests: `a_push_sways_and_returns` (no overshoot past the step
    threshold; settles in ~1–2 s); `the_cop_leads_the_com_with_the_opposite_sign`;
    `a_push_beyond_the_support_saturates_and_flags_a_step` (hook for a
    future stepping reaction).
- [ ] **2.2 Posing the offset.** A/P through `sway_over_feet` (whole-leg
  ankle lean, trunk upright); M/L through `shift_weight`'s load/unload,
  scaled continuously.
  - Tests: planted feet ≤ 1 mm in the target, ≤ 5 mm in the sprung pose;
    sway angle < 8° (the model's validity range).
- [ ] **2.3 Wire and verify.** Compose after the idle weight shift; gallery
  `--push-schedule T:X,Z` for reproducible BRP capture. Live check, progress,
  KB note. Checkpoint: commit.

## 3a. Per-character knee style (§11.1)

**Skipped (2026-10-01, by decision).** Not to be built.

**Premise withdrawn (2026-09-30).** §11.1 shows hip/knee *moment* splits
varying day to day at near-identical *angles* (rms s.d. < 2°). That
supports no knee-angle style for the walk; the KB note that said so was
wrong and is corrected. A standing-stance knee setting would be a plain
style choice, not Winter's. The split's real use is step 4: an active
ragdoll may trade stance hip and knee torque as long as their sum holds.

- [ ] A per-character knee-flex setting (`stance_on(base, knee_flex)`,
  `DEFAULT_KNEE_FLEX`) passed into the stance, the release and the walk's
  pelvis envelope. One test swept over 0.1–0.3 rad: feet planted, pelvis
  height consistent with the flex. Live check at two settings. Checkpoint:
  commit.

## 4. Self-balancing active ragdoll (§11.2, §7.4.5, §9.0.5, §9.2, §8.1)

**4.2–4.5 deferred (2026-10-01):** kept as a later improvement, if and
when the self-balancing ragdoll is taken up again. Not in the current work.

**Re-scoped 2026-09-30: physics as a hybrid.** Animation stands and walks;
physics takes over only for falling, stumbling and hits. The self-balancing
ragdoll (4b–4d below) is dropped: it would need pairwise joint torques with
a stable PD, research-grade (see the 4b finding). Instead:

- [x] **H1 Stumble (animation), done 2026-09-30.** A push the feet cannot
  absorb (`Balance::needs_step`) steps to the predicted capture point
  (≤ 0.4 m). The weight moves toward that foot, and the trailing foot
  joins once the capture point is inside it. Root motion moves the
  character by the distance stepped. It has its own step, not the walk's
  first-step machinery: a stumble step goes in any direction and is
  planned from the pendulum. The pelvis sinks 46/115/44 mm (forward,
  sideways, back); planted feet stay ≤ 1 mm live on both rigs. See
  [the note](./docs/knowledge/character-animation/ik-and-locomotion/a-stumble-is-a-capture-point-step-then-a-join.md).
- [x] **H2 Fall (physics), done 2026-09-30.** A push asking for a step
  longer than `MAX_CATCH` (0.8 m; now a forecast, see below), or a game call (`Ragdoll::fall`; `F`
  in the gallery), releases the pinned root with its velocity. Gravity
  acts in full, tone is joint damping (3/s), the screen shows the
  simulation with the skeleton on the hips body, and the entity follows.
  A body at rest is put to sleep and marked `Fall::at_rest`. Hits don't
  trigger a fall yet. Sideways the balance can't tell a catch from a
  fall (the clamp catches). See
  [the note](./docs/knowledge/character-animation/ragdoll-and-physics/a-fall-hands-the-body-to-physics.md).
- [x] **H3 After a fall, done 2026-09-30.** Settled is `Fall::at_rest`.
  `Ragdoll::get_up(delay)` reads face up or down, turns the character to
  rise along its body, and blends through solved key poses (`getup.rs`):
  sit → squat (VanSant 1988) or hands and knees → half-kneel. It then sets
  every body on its bone and pins the root again. See
  [the note](./docs/knowledge/character-animation/ragdoll-and-physics/getting-up-is-a-timed-blend-then-a-re-pin.md).
- [x] **H2 follow-ups, done 2026-09-30.**
  - A hit is a push on the balance (by the struck body's mass share), so a
    strong enough blow steps or falls.
  - Sideways stumbles are real catches: the stance foot's pressure is at
    its nearest point, the leg that just stepped never steps again,
    `MAX_STEP` is 0.6 m, and the far leg crosses over when that is the
    shorter step.
  - A fall carries the push's velocity.
- [x] **Second follow-ups, done 2026-10-01.** See
  [CHARACTER_PROGRESS.md](./CHARACTER_PROGRESS.md) for the numbers.
  - Falls are forecast: the balance is run 3 s ahead at the first step,
    replacing `MAX_CATCH`, so 1.2 m/s sideways is caught in four
    crossovers.
  - Touchdown is cushioned by a sprung pelvis drop (jolt 12.8 → ≤ 4.8 mm).
  - Feet that dip while rising are tucked, not lifted over; keys are
    chained so shared contacts hold.
  - Hits topple a character with no balance when its capture point
    leaves its feet.
  - The ragdoll converts poses in the character's frame, so the rise's
    turn no longer snaps the body or leaves the arms in a T.
  - The balance ticks at most 1/60 s, so long frames don't lose a catch.
- [x] **Hard side push, done 2026-10-01.** The balance tests now stand as
  the character is drawn (they had the arms overhead, limits inverted).
  `MAX_STEP` is 0.7 m and the trailing foot joins as soon as the stepped
  foot holds the body, so 1.2 m/s sideways is caught live on both rigs
  with the pelvis ≤ 138 mm down. Limits: 1.5 forward, 1.4 sideways and
  back.

Already built and kept: hits with a stun-and-recover strength dial, the
pinned ragdoll following the animation, sole feet (4a).

- [x] **4.1 spike result (2026-09-30), stopped per its own rule.** Headless,
  `puppet_base` unpinned on a friction-1 floor, full gravity (the harness
  resets `GravityScale` to 1 each step, since `support_own_weight` zeroes
  it at full strength), full-strength PD toward the bind pose:
  - **Buckles at 0.5 s** (hips −10 cm), as Winter §8.1 predicts for an
    open-loop forward solution. Then the hips drop to −34 cm and recover,
    repeatedly.
  - **Contact is unstable: the feet skate.** The left foot slides 0.8 m
    in 2.5 s and turns 67°, with the pelvis within ~4° of upright. Each
    foot is one capsule, ankle to ball: no heel, no flat sole, and it can
    roll.
  - **Structural:** the PD is acceleration-shaped per body and holds the
    weight only through gravity compensation (`GravityScale = 1 −
    strength`). A balancing ragdoll has to carry its real weight through
    its feet, so its load-bearing joints need torque-shaped control. The
    4.2 torque budgets are then the control itself, not just its limits.
  - Probe: `ragdoll_plugin::tests::probe_unpinned_ragdoll_stands`
    (ignored).
- **Revised step 4 (agreed 2026-09-30):** (a) feet, (b) torque-shaped
  control of the load-bearing joints against real gravity, within Winter's
  budgets, (c) the balance controller on the measured COM, then pushes,
  (d) muscle behaviour and mode switching.
- [x] **4a feet (2026-09-30).** `RagdollSpawnConfig::feet` / `sole_blocks`:
  flat blocks from `foot::Sole` (heel to tip, Winter's 0.362 breadth, 3 cm,
  friction 1). One foot dropped: 0.10° / 0.16 mm against the capsule's
  26.6° / 25.1 mm (`a_foot_stands_flat_on_its_sole`). The whole ragdoll's
  feet stop rolling but still crawl ~0.85 m as its legs buckle: that is
  4b.
- **4b finding (2026-09-30), before building.** The current controller
  drives each body's WORLD orientation and applies the result to that body
  alone (no reaction on its parent): an invisible hand per body, not a
  muscle. With real gravity and sole feet it cannot stand either:
  - at 8 Hz it buckles at 0.5 s;
  - at 16 Hz, near the 64 Hz explicit bound, it holds ~1.1 s (hips −6 cm),
    then the feet slide apart (the left foot 0.11 → 0.49 m out, the trunk
    upright) to hips −50 cm.

  Real standing needs pairwise internal torques (+τ on the child, −τ on
  the parent) in N·m. That is unstable when explicitly integrated at the
  needed stiffness against a light foot, unless the PD is formulated
  stably (Tan et al. 2011) and/or the physics runs faster than 64 Hz.
- **4.1 as planned:** Spike, unpinned root (in a worktree). Release the root, add
  foot colliders and ground friction, drive only the pose PD. Measure time
  to fall; Winter §8.1's null result predicts ~0.5 s. Stop and report if
  contact itself is unstable.
- [ ] **4.2 Torque budgets.** Per joint: Winter per-kg peak × body mass
  (ankle ≈ 1.6 N·m/kg), converted to an acceleration ceiling through
  `limb_mass_properties`. Only in a new `Balancing` mode. Test: ceilings
  equal the budgets within 1 %.
- [ ] **4.3 Balance controller.** Part 2's controller on the measured COM:
  A/P ankle torque = W·(COP target − ankle); M/L hip abductor load/unload.
  Log the co-contraction ratio Σ|τ|/|Στ| (§11.2.2: alternating, not
  co-contraction).
  - Tests: `an_unpinned_ragdoll_stands_60s` (no drift, no NaN);
    `a_push_is_recovered_within_budget`; `a_push_beyond_the_base_falls`;
    torque never exceeds budget.
- [ ] **4.4 Muscle behaviour**, each only if stable: activation lag as a
  critical filter, ~40–60 ms (§9.0.5); Hill force–velocity ceiling (§9.2).
  Each keeps its own stability test green.
- [ ] **4.5 Mode switching.** Pinned → balancing → fall → stun recovery, no
  pops. Live check, bench, KB notes (decision + Winter 8.1 / 11.2).
  Checkpoint: commit.

## 3b. Body proportions (§4.0.1), spike only

- [x] **Spike, done 2026-10-01.** Scale one `character.glb` segment
  (thigh +10 %) by (a) moving the joint alone and (b) bone scale with
  child compensation; inspect the knee skinning.
  - (a) stretches the knee's blend triangles: p50 1.13, max 2.71 at a 90°
    bend. Not acceptable.
  - (b) as planned can't work in Bevy: a scaled parent shears a rotated
    child. Replaced by a skinning-only scale (a helper joint under the
    thigh, scaled along +Y, swapped into `SkinnedMesh::joints`) plus the
    moved joint: blend p50 1.000, max 1.16. Acceptable.
  - The foot IK reads live translations, so the longer leg stood planted
    with no other change. `--proportion-spike move|proxy F`,
    `tools/skin_segment_stretch.py`, see
    [the note](./docs/knowledge/character-animation/rig-and-retargeting/lengthen-a-segment-by-its-joint-and-a-skinning-only-scale.md).
- [ ] Height-fraction proportions (feasible; not yet requested).

## 5. Open limits (agreed 2026-10-01)

- [x] **5.1 Pushes and hits while walking, done 2026-10-01.**
  `WalkBalance`: footfalls placed at the capture point (Hof), forward
  pushes a speed surge; hits handed over; the fall's root launches at the
  walk's pace. See
  [the note](./docs/knowledge/character-animation/ik-and-locomotion/a-push-while-walking-moves-the-next-footfalls.md).
- [x] **5.2 Uneven ground, done 2026-10-01.** Locks take the body's
  rise; foot IK samples the ground in the world; the rise keeps clear of
  the `AnimGround` under each joint. Keys still posed flat. See
  [the note](./docs/knowledge/character-animation/ik-and-locomotion/sample-the-ground-in-the-world-not-the-pose.md).
- [x] **5.3 Getting up from lying on the side, done 2026-10-01.**
  Side-sit → hands and knees → half-kneel; see the get-up note.
- [x] **5.4 Walking ragdoll lag, done 2026-10-01.** Velocity
  feedforward: median 11.5 → 9.5° (puppet_base), 10.4 → 8.1°
  (character.glb), max ≤ 15°; the arms' ~5° is shoulder coupling. See
  [the note](./docs/knowledge/character-animation/ragdoll-and-physics/a-pinned-ragdoll-tracks-its-targets-velocity.md).
- [x] **5.5 Small items, worked 2026-10-01.** Fixed: edge catches (whole
  ticks; 0/12 fall). Measured and left open, with causes: the clamped
  backward step's jolt (7.6 mm); the loaded side step (138-290 mm lunges,
  or lost catches as a shuffle); a shoulder second cone (any second arm
  joint holds the arm 6° off); hand bodies (character.glb to 100°). Kept
  by decision: elbow overshoot (within passive range). See
  CHARACTER_PROGRESS.md.

## Throughout

`--release` for all cargo commands; structural tests before screenshots;
measured numbers in CHARACTER_PROGRESS; `python3 tools/kb.py lint` at 0
errors; no `git stash` (worktrees for A/B).
