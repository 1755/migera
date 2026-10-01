---
title: A stumble is a capture-point step, then a join the body's momentum carries
description: "balance::Balance steps to the predicted capture point (≤ 0.7 m), never re-using the leg just stepped, crossing over sideways; the weight moves onto the step as it lands and the trailing foot joins once that foot holds the body; a sprung pelvis cushions touchdown. Read before changing stepping in balance.rs."
type: decision
status: current
tags:
  - balance
  - biomechanics
  - locomotion
  - ik
  - correctness
updated: 2026-10-01
verified: 2026-10-01
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
  time with the COM running after it. **Sideways, whichever leg needs the
  shorter step**, and in this model that is usually the far leg
  **crossing over** in front. The near leg's side step stands on the far
  foot, whose pressure drives the body on. The crossover stands on the near
  foot, whose pressure brakes it. A 0.8 m/s side-step lunge sank the
  pelvis 266 mm; the crossover sinks it 91 mm. People use both: the
  literature names the loaded side step, the unloaded crossover and the
  unloaded medial step (Maki & McIlroy's change-in-support strategy).
  **Young adults mostly take the loaded side step**, which starts faster;
  older adults cross over more. So the choice here is the model's
  dynamics, not the typical young adult's. A crossover lands `CROSSOVER_AHEAD` (0.12 m)
  forward and bows out that far mid-swing, so the legs pass rather than
  through each other. The join uncrosses them, because the joining foot
  goes to the stood width beside the stepped one.
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

  Jolt (second difference of the pelvis) ≤ 5.0 mm for every unclamped
  step, headless on `puppet_base` (`probe_max_jolt`). With the spring
  removed (instant follow), 6.0 mm on 0.6 m steps: most of the gain is
  the lead and slack. A backward step clamped at `MAX_STEP` (1.3 m/s,
  0.88 m asked) lands ball-first, overreaching, and still jolts 9.8 mm.
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
`puppet_base_as_rendered`): forward to 1.5 m/s, sideways to 1.4 (1.2 in
one crossover and a join, harder in up to three steps), back to 1.4 (two
steps) (`a_push_past_a_catchable_step_falls`). Forward 1.6, sideways 1.5
and back 1.5 fall; see
[a fall hands the body to physics](../ragdoll-and-physics/a-fall-hands-the-body-to-physics.md).
Until 2026-10-01 these limits were measured with the arms overhead (see
[the puppet_base fixture note](../rig-and-retargeting/puppet-base-fixture-faces-away-from-the-rendered-character.md)),
and read the other way round from the live character.

Near-full-reach steps, headless (`a_stumble_steps_cleanly_on_the_real_rig`),
with the cushioned touchdown (~10 mm deeper than the bare need):

| push | step asked | pelvis sank | note |
|---|---|---|---|
| 1.2 m/s forward | 0.67 m | 86 mm | rear heel rises 82 mm onto the toes |
| 1.0 m/s left | 0.66 m | 135 mm | crossover; the side-step lunge was 266 mm at 0.8 m/s |
| 1.0 m/s back | 0.64 m | 55 mm | |
| 1.2 m/s left or right | 0.82 m (0.7 taken) | 123–130 mm | `a_hard_side_catch_joins_early_instead_of_lunging` |

Live, 2026-10-01, both rigs: 1.2 m/s sideways each way, and 1.2 forward
then 1.0 back, all caught. Planted balls held within 13 mm; the pelvis
sank 129–138 mm at worst.

Until 2026-09-30 the sideways catch was the clamp's: the 8° bound, from
the stance foot alone, held the COM with its velocity zeroed, and every
side step was posed over a body the clamp had stopped.

## Revisit when

- Pushes while walking: the balance only runs on the standing side of the
  blend.
- Young-adult sideways stepping. Tried 2026-10-01, both ways the earlier
  analysis suggested, and both were worse. A side step with the loaded leg
  whenever it needs at most 0.5-0.7 m sank the pelvis 138 mm at 0.6 m/s
  (crossover 40) and 290 at 0.8 (82). Side steps capped at 0.4 m with the
  far leg joining between them lost catches from 0.8 m/s (crossovers catch
  1.4) and jolted 24-34 mm. People first unload the near leg (a quick
  weight shift onto the far foot) before stepping with it; without that
  phase, the side step stands on the far foot, whose pressure drives the
  body on. An unloading phase was then tried (2026-10-01): catches fell
  from 0.8 m/s, and above 1.0 m/s the near leg never lifted. The stance
  forbids lifting a foot while the other foot is past the 8° validity
  bound (see
  [a foot may lift only when the other holds the body](./a-foot-may-lift-only-when-the-other-holds-the-body.md)),
  and a hard side push puts the body there at once. Revisit only with a
  stance model valid past that bound.
- The backward step clamped at `MAX_STEP` (1.3 m/s) still jolts 7.6 mm
  (9.8 before the stance fixes). Located 2026-10-01: it lands with the
  stepping leg at full stretch, and the moment its 11 mm swing lift
  reaches zero the leg's reach ceiling pulls the pelvis down 7 mm in a
  frame. Closing the swing slack earlier and rate-limiting the hips' roll
  both changed nothing for it (the roll limit worsened sideways steps to
  8 mm). A fix changes the step itself (a later landing, a shorter
  backward reach) and so the catch limits. Tried 2026-10-01:
  - Capping backward travel at 0.6 m: 1.3 m/s backward pushes fell, and
    the jolt grew to 11.2 mm.
  - Landing slack: no change.
  - A travel lead: backward 7.3 mm, but other directions rose to 8.5.

## Related

- [Push recovery is Winter's pendulum](./push-recovery-is-winters-pendulum.md) — prerequisite: the sway, COP law and support this steps from.
- [A fall hands the body to physics](../ragdoll-and-physics/a-fall-hands-the-body-to-physics.md) — deeper: what happens when no step catches the push.
- [A foot may lift only when the other holds the body](./a-foot-may-lift-only-when-the-other-holds-the-body.md) — deeper: the gates on the join and every later step.
- [The puppet_base fixture faces away from the rendered character](../rig-and-retargeting/puppet-base-fixture-faces-away-from-the-rendered-character.md) — same-trap: why these catch limits were once inverted.
- [A speed contact test is fooled by a lagging sprung leg](./a-speed-contact-test-is-fooled-by-a-lagging-sprung-leg.md) — deeper: why the gallery passes `planted` to the foot IK.
- [Foot locks need the body's travel](./foot-locks-need-the-bodys-travel.md) — applies: the join's travelled distance becomes root motion the locks must be given.
- [Bind-pose zero leg slack is normal](./bind-pose-zero-leg-slack-is-normal.md) — context: why a wide stance leaves the legs no reach to spare.
