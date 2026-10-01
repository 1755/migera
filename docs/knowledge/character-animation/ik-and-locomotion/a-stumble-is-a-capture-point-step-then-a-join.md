---
title: A stumble is a capture-point step, then a join the body's momentum carries
description: "balance::Balance steps to the predicted capture point (≤ 0.7 m), never re-using the leg just stepped; sideways a quick loaded side step (≤ 0.4 m), else a crossover; the weight moves onto the step through a spring and the trailing foot joins once that foot holds the body. Read before changing stepping in balance.rs."
type: decision
status: current
tags:
  - balance
  - biomechanics
  - locomotion
  - ik
  - correctness
updated: 2026-10-02
verified: 2026-10-02
code:
  - src/character/anim/balance.rs
  - src/character/anim/stance.rs
  - examples/character_gallery.rs
sources:
  - "Winter, Biomechanics and Motor Control of Human Movement, 4th ed. (2009), §11.2.1 (Eq. 11.3); Appendix A frames 63-70 (pre-swing roll onto the toes)"
  - "tests balance::tests::a_stumble_steps_cleanly_on_the_real_rig, a_stumble_plans_the_same_steps_at_uneven_frame_times, a_catch_does_not_depend_on_frame_times"
  - "live BRP, character_gallery --push-schedule 3:0.6:0,8:0:0.7,13:-0.8:0 and 3:0:0.8, puppet_base and character.glb"
  - "Maki & McIlroy (1997), The role of limb movements in maintaining upright stance: the 'change-in-support' strategy, Phys Ther 77(5)"
  - "Postural reactions to external mediolateral perturbations: a review, Applied Sciences 13(3):1696 (2023), https://www.mdpi.com/2076-3417/13/3/1696 — loaded side step, unloaded crossover, unloaded medial step; young adults favour the loaded side step"
  - "Untangling biomechanical differences in perturbation-induced stepping strategies for lateral balance stability in older individuals, https://pmc.ncbi.nlm.nih.gov/articles/PMC7778461/"
  - "probe balance::tests::probe_catch_table (ignored)"
  - "probes balance::tests::probe_max_jolt, probe_side_steps, probe_jolt_trace (ignored); test a_sideways_shove_side_steps_then_crosses_over_when_harder"
  - "Mille et al. (2005), Clin Biomech 20:607 — young adults recover from lateral pulls mostly with one loaded side step"
aliases:
  - crossover step
  - CROSSOVER_AHEAD
  - MAX_STEP
  - stumble step
  - recovery step
  - join step
  - weight transfer
  - heel rise
  - MAX_HEEL_RISE
  - DROP_BEFORE_HEEL_RISE
  - move_pelvis_and_feet
  - Sink
  - SINK_LEAD
  - SWING_SLACK
  - touchdown V
  - MAX_TICK
  - early join
  - loaded side step
  - SIDE_STEP_SECONDS
  - SIDE_STEP_MAX
  - WEIGHT_FREQUENCY
  - CEILING_LEAD
---

# A stumble is a capture-point step, then a join the body's momentum carries

Contents: [Context](#context) · [Decision](#decision) ·
[Alternatives](#alternatives-considered) · [Consequences](#consequences) ·
[Revisit when](#revisit-when)

When a push puts the capture point `x + ẋ√K` outside the feet,
`balance::Balance` takes one recovery step to where the capture point will
be when the foot lands. The weight then moves onto that foot, and the
trailing foot joins it once the body's momentum will carry the COM onto
the stepped foot. The body does not wait until it is already over it. The
character then moves by the distance stepped (root motion), and the feet
stand side by side as before.

## Context

Pushes the feet can absorb were already handled by the pendulum sway (see
[push recovery](./push-recovery-is-winters-pendulum.md)); `needs_step` was
the hook left for this. H1 of the hybrid plan: a stumble stays kinematic,
and physics takes over only for a fall.

## Decision

- **Landing: the predicted capture point.** During the swing the COP
  stands on the other foot, and the capture point runs away from it as
  `e^{t/√K}`. So the foot goes to `p + (cp − p)·e^{T/√K}`, with
  `STEP_SECONDS` = 0.3. `p` is the stance foot's point **nearest the
  capture point**, where `step`'s law pins the COP, not its middle; the
  middle asked for steps ~2× too long (0.48 m where 0.26 m caught a
  0.5 m/s push). Travel is clamped to `MAX_STEP` = 0.7 m, about 0.76 of
  `puppet_base`'s leg length and 40% of its height: young adults' maximal
  step is 77–79% of height forward (Medell & Alexander's test), and
  lateral lunges are standardised at 60%. At 0.6 m a 1.2 m/s side push
  (0.82 m asked) ran away in 26 steps; at 0.65 it was an edge catch with
  72 mm jolts. The sideways component is kept only for a sideways push.
- **Judged and planned on the whole push**, the part not yet delivered
  too. Judged on what had landed, a frame's delay (50 ms live) put a
  0.7 m/s side push's step 17% further out, past `MAX_STEP`.
- **Which leg.** Straight ahead or back, the unloaded one. **Never the
  foot that just stepped**: stepping it again lifts it before it takes
  the weight, and a sideways push walked the stepping foot out 0.4 m at a
  time with the COM running after it.
- **Sideways: a quick loaded side step, then crossovers.** The literature
  names the loaded side step, the unloaded crossover and the unloaded
  medial step (Maki & McIlroy's change-in-support strategy). Young adults
  mostly take the loaded side step; older adults cross over more.
  - **The near leg side-steps** while it needs at most `SIDE_STEP_MAX`
    (0.4 m), swinging in `SIDE_STEP_SECONDS` (0.2 s, against 0.3 for
    other steps). It stands on the far foot, whose pressure drives the
    body on, so its length grows as `e^{T/√K}` with its time. At 0.3 s a
    0.6 m/s push needed a 0.49 m lunge that sank the pelvis 146 mm. At
    0.2 s it needs 0.30 m and sinks 44 mm, the crossover's 0.35 m and 45.
    At 0.15 s the steps were shorter still, but harder pushes jolted up to
    9 mm.
  - **Past that, the far leg crosses over** in front, standing on the
    near foot, whose pressure brakes the body. A side step's stance ends
    wide and holds the pelvis low: 317 mm down at 1.2 m/s against the
    crossover's 140. Side steps catch pushes to 0.7 m/s.
  - A crossover lands `CROSSOVER_AHEAD` (0.12 m) forward and bows out
    that far mid-swing, so the legs pass rather than through each other.
    The join uncrosses them, because the joining foot goes to the stood
    width beside the stepped one.
  - Later steps of a hard sideways catch are quick side steps too. That
    caught 1.5 m/s sideways in a crossover and four side steps, sinking
    158 mm.
- **The weight moves onto a recovery step as it lands** (`transfer`), and
  **the join starts as soon as that foot alone holds the body** and the
  capture point is not on the trailing foot's side of it. Both conditions
  and why each is needed are in
  [a foot may lift only when the other holds the body](./a-foot-may-lift-only-when-the-other-holds-the-body.md).
  Two ways of waiting both sank the pelvis. Waiting until the COM was
  over the stepped foot: 14 cm forward, 21 cm sideways (`COM_PER_PELVIS`
  puts the pelvis beyond the COM). Waiting to be caught at rest before
  transferring: the pendulum pulled the COM to the middle of a 0.7 m
  crossover's stance, and the far leg held the pelvis 213–271 mm down
  for about 2 s, the join 2.5–3.0 s in. Now it joins 0.37–0.67 s in.
- **The foot that stays is the one the weight is on**; any landing that
  sets the feet side by side ends the stumble, a recovery step too.
- **The swinging foot keeps holding the pelvis in reach.** Its leg gets
  load 0.1 in the drop, aimed at the foot's moving, lifted target. When
  it was let go, the pelvis sprang up 132 mm in the frame the trailing
  foot lifted.
- **A trailing foot rolls onto its toes instead of squatting the body.**
  `move_pelvis_and_feet(.., rise)` lets a foot whose tip trails its hip
  socket pitch up by as much as `MAX_HEEL_RISE` (0.6 rad). It does so only
  once, held flat, the foot would ask the pelvis to drop more than
  `DROP_BEFORE_HEEL_RISE` (4 cm). It rolls rigidly about the sole's
  **tip**, because `foot::Sole` is a rigid foot. About the toe joint, the
  rigid tip went into the floor. This is also Winter's pre-swing: the
  metatarsal marker climbs while the toe marker stays down. The walk's
  callers pass `rise = 0` and are unchanged.
- **The pelvis carries the sway, not the step.** Pelvis =
  `carried + (offset − carried)/COM_PER_PELVIS`, where `carried` is the
  feet's mean displacement. Scaling the whole offset put the pelvis 60 mm
  ahead of itself by the end of a step, and the hand-over popped it back.
- **One solve** (`stance::move_pelvis_and_feet`): the pelvis height accounts
  for where each loaded foot is going, and each ankle is placed once under
  the moved pelvis. Placing the stepped foot first, under a pelvis not yet
  over it, left that foot out of reach in the air.
- **Touchdown is cushioned by a sprung pelvis drop** (`balance::Sink`).
  Followed exactly, the pelvis height traced the leg's reach through a V
  at a long step's landing: its per-frame move changed by up to 12.8 mm
  (≈ 5 g at 60 Hz). A real landing decelerates over 50–100 ms of knee
  flexion. Three parts, all needed:
  1. The drop aims at the need with the swinging foot `SINK_LEAD`
     (0.08 s) ahead on its arc, so it starts down before the foot does.
     0.04 s served 0.6 m steps; on a 0.66 m crossover the late swing
     jolted 9.7 mm, the ceiling catching the lagging spring.
  2. The swinging leg's reach may be short by `SWING_SLACK` (the step's
     lift, 5 cm), fading to zero by touchdown, so the aim is not undone
     by a leg near full extension.
  3. A critically damped spring (15 rad/s, solved exactly, not
     integrated) follows the aim, never deeper than the loaded legs'
     reach plus slack.

  With the spring removed (instant follow), 6.0 mm on 0.6 m steps: most
  of that gain is the lead and slack.
- **The weight moves over through a spring** (`WEIGHT_FREQUENCY`, 15
  rad/s, critically damped), not in a frame. Which feet bear weight
  changes at a landing or a lift, and switched at once it flipped the
  pelvis's roll and the socket it pivots on together: a 3.5 mm sideways
  reversal at a backward landing, up to 5.6 mm jolts forward. Every leg's
  load comes from the sprung shift, a lifted one's too. Forced to the
  floor load the moment it lifted, the pivot still jumped (7.0 mm at a
  0.6 m/s side push's join). The landings sit ~6 mm deeper.
- **The sink sees the ceiling coming** (`CEILING_LEAD`, 0.1 s). It aims
  no higher than the legs' ceiling will be with the pelvis moved on at
  its velocity, solved again on a copy of the pose. A backward step
  clamped at `MAX_STEP` (1.3 m/s, 0.88 m asked) jolted 7.6 mm: as the
  body flew back from the front foot, that leg's reach fell 5-11 mm a
  frame and met the spring still rising toward the stepped leg's need at
  the landing. The swing's lift was not it: landing at zero vertical
  speed (`sin²`) changed nothing. Leads estimated from the ceiling's
  last change were not monotonic (0.05 s: 10.4 mm) because the ceiling
  jumps when another leg sets it. A brake against the closing speed
  reached 6.1. Swept: no lead 8.1 mm, 0.05 7.3, 0.08 4.6, 0.1 3.7
  (sinking 120 mm against 100), 0.15 3.7 (165). Only backward pushes
  moved.

  Jolt (second difference of the pelvis) ≤ 4.4 mm for every catch,
  headless on `puppet_base` (`probe_max_jolt`, `probe_side_steps`),
  clamped steps included.
- **Whole ticks of 1/60 s** (`MAX_TICK`, the forecast's step), the rest
  of a frame carried to the next. A swing lands, and the next step is
  planned, only between ticks, so every frame pattern runs the same ticks.
  Taken a whole frame at a time, a 50 ms frame landed a foot up to 50 ms
  late while the body kept falling off the old support (9 of 12 catches
  at the limit fell). Divided evenly into ticks of at most 1/60 s, uneven
  frames still shifted the landings (4 of 12). Whole ticks: 0 of 12
  (`a_catch_at_the_limit_survives_uneven_frames`). `apply` draws the
  carried-over time on from the last tick, the COM at its velocity and a
  swinging foot along its arc; drawn between the last two ticks instead,
  a landing mixed one tick's feet with the next's swing and a planted foot
  jumped 21 mm. At 60 Hz nothing carries over and nothing changed.
- **The foot IK is told which feet are down** (`AnimFootIk::planted`); see
  [a speed contact test is fooled by a lagging sprung leg](./a-speed-contact-test-is-fooled-by-a-lagging-sprung-leg.md).
  The landing hint (`Balance::landing_spot`) eases in over the first
  quarter of the swing and fades out over `LAND_HOLD` after touchdown. At
  full strength from lift-off, the ball popped up 30 mm. Held at full
  strength until the hold ended, it dropped 20 mm in one frame.

## Alternatives considered

- **Carry the COM fully over the stepped foot, then join.** This lost for
  the reason above: the rear leg's reach sets the pelvis height (14–21 cm
  sinks).
- **A longer `MAX_STEP` alone.** At 0.7 m without the early join, 1.2 m/s
  sideways was caught with the pelvis 213–271 mm down for 2 s.
- **Shorter steps, so the stance is never wide.** This would under-catch
  hard pushes. The capture-point prediction asks for about 0.4 m at
  0.6–0.8 m/s.
- **Leave the rear foot to the foot IK's toe lock**, which re-aims a foot
  it can't reach. This couldn't be tested headless, and the balance's own
  pose lifted the whole foot flat first.

## Consequences

Every catch is the step's, not the validity bound's: nothing lost to the
bound (`Balance::lost`), settled 3.2–3.5 s after a near-limit push.
Caught on `puppet_base` as drawn (`relaxed_stand`,
`puppet_base_as_rendered`): forward to 1.5 m/s, sideways to 1.5 (to 0.7
in a side step and a join, 1.2 in one crossover and a join, harder in a
crossover and up to five quick side steps), back to 1.4 (two steps)
(`a_push_past_a_catchable_step_falls`). Forward 1.6, sideways 1.6 and
back 1.5 fall (sideways 1.5 fell until 2026-10-02); see
[a fall hands the body to physics](../ragdoll-and-physics/a-fall-hands-the-body-to-physics.md).
Until 2026-10-01 these limits were measured with the arms overhead (see
[the puppet_base fixture note](../rig-and-retargeting/puppet-base-fixture-faces-away-from-the-rendered-character.md)),
and read the other way round from the live character.

Near-full-reach steps, headless (`a_stumble_steps_cleanly_on_the_real_rig`),
with the cushioned touchdown (~10 mm deeper than the bare need) and the
sprung weight shift (~6 mm more), 2026-10-02:

| push | step asked | pelvis sank | jolt | note |
|---|---|---|---|---|
| 1.2 m/s forward | 0.67 m | 106 mm | 3.1 mm | rear heel rises onto the toes |
| 1.0 m/s left | 0.66 m | 150 mm | 2.9 mm | crossover |
| 0.6 m/s left | 0.30 m | 44 mm | 2.1 mm | quick side step |
| 1.0 m/s back | 0.64 m | 72 mm | 3.2 mm | |
| 1.3 m/s back | 0.88 m (0.7 taken) | 120 mm | 3.7 mm | was 7.6 mm |
| 1.2 m/s left or right | 0.82 m (0.7 taken) | 140 mm | 3.2 mm | `a_hard_side_catch_joins_early_instead_of_lunging` |

Live, 2026-10-01, both rigs: 1.2 m/s sideways each way, and 1.2 forward
then 1.0 back, all caught. Planted balls held within 13 mm; the pelvis
sank 129–138 mm at worst. Live, 2026-10-02, both rigs (BRP): a 0.6 m/s
side push side-steps the near foot first and the far one joins, the
pelvis 32-43 mm down; 1.3 m/s back is caught, 104-119 mm down, the
pelvis's vertical acceleration ≤ 15 m/s² (a 3.7 mm jolt at 60 Hz is 13).

Until 2026-09-30 the sideways catch was the clamp's: the 8° bound, from
the stance foot alone, held the COM with its velocity zeroed, and every
side step was posed over a body the clamp had stopped.

## Revisit when

- Pushes while walking: the balance only runs on the standing side of the
  blend.
- A side step's swing time is a choice (0.2 s). If reaction-time data
  for reactive lateral steps is found, check it against that; the
  numbers that would move are in the side-step bullet above.

What failed before the quick side step (2026-10-01), so it is not tried
again: side steps with the 0.3 s swing whenever they needed at most
0.5-0.7 m (138 mm sunk at 0.6 m/s, 290 at 0.8); side steps capped at
0.4 m with joins between (catches lost from 0.8 m/s, 24-34 mm jolts); a
separate 0.1 s unloading phase before the lift (catches lost from 0.8
m/s, and above 1.0 the near leg never lifted). The last one delayed the
lift until the COM was past what the far foot holds (see
[a foot may lift only when the other holds the body](./a-foot-may-lift-only-when-the-other-holds-the-body.md));
planned at once, the side step passes that gate. For the backward jolt,
before the sprung weight and the ceiling lead: a 0.6 m backward cap
(1.3 m/s fell, 11.2 mm), landing slack (no change), a lead on the
pelvis's travel for every leg (back 7.3, others up to 8.5), a rate
limit on the hips' roll alone (sideways 8 mm).

## Related

- [Push recovery is Winter's pendulum](./push-recovery-is-winters-pendulum.md) — prerequisite: the sway, COP law and support this steps from.
- [A fall hands the body to physics](../ragdoll-and-physics/a-fall-hands-the-body-to-physics.md) — deeper: what happens when no step catches the push.
- [A foot may lift only when the other holds the body](./a-foot-may-lift-only-when-the-other-holds-the-body.md) — deeper: the gates on the join and every later step.
- [The puppet_base fixture faces away from the rendered character](../rig-and-retargeting/puppet-base-fixture-faces-away-from-the-rendered-character.md) — same-trap: why these catch limits were once inverted.
- [A speed contact test is fooled by a lagging sprung leg](./a-speed-contact-test-is-fooled-by-a-lagging-sprung-leg.md) — deeper: why the gallery passes `planted` to the foot IK.
- [Foot locks need the body's travel](./foot-locks-need-the-bodys-travel.md) — applies: the join's travelled distance becomes root motion the locks must be given.
- [Bind-pose zero leg slack is normal](./bind-pose-zero-leg-slack-is-normal.md) — context: why a wide stance leaves the legs no reach to spare.
