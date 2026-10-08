---
title: A jump from a run replays the run's stance under a planned COM, and hands back mid-frame
description: "A jump from a run replays the run's stance on its planted foot under a planned COM push, then runs on from the landing foot's toe-off or lands on both feet. Traps: a constant-speed carry slides the foot, cubic paths pull on the floor, a COM planned forward on one foot kicks the hips, a mid-frame hand-back loses travel."
type: decision
status: current
tags:
  - locomotion
  - ik
  - correctness
updated: 2026-10-09
verified: 2026-10-06
code:
  - src/character/anim/jump/leap.rs
  - src/character/anim/jump.rs
  - src/character/anim/walker.rs
sources:
  - "Linthorne 2008, Linthorne et al. 2011, Panoutsakopoulos et al. 2010: one-foot take-off speeds, horizontal loss, free-leg drive, take-off forces (4-10 body weights)"
  - "tests jump::leap::tests::*"
  - "live: character_gallery --anim-speed 4 --jump-at 6:0.25:0:run,9:0.1:0:run and --anim-speed-schedule 0:4,6.5:0 --jump-at 6:0.25, --step-seconds 0.0166667, Xvfb, BRP every frame"
aliases:
  - Jump::from_run
  - RunStart
  - Resume
  - JumpAsk::running
  - Walker::jump while running
  - jump stop
  - LOSS_PER_UP
  - LANDING_SPEED
  - LANDING_BRAKE
  - pushed_height
---

# A jump from a run replays the run's stance under a planned COM, and hands back mid-frame

Asked while running, a jump waits for the next foot to come down and takes
off from it (`Jump::from_run`):
- the run's own measured stance is replayed on that planted foot;
- the pelvis is solved so the centre of mass (COM) follows a planned push
  instead of the run's;
- the free thigh drives up and the arms swing harder.

From there it either runs on, or lands on both feet and stops
(`JumpAsk::keep_running`).

## Decision

**Phases** (`JumpPhase`):

| Phase | Running on | Stopping |
|---|---|---|
| Push | take-off stance on the foot just down | the same, braking to `LANDING_SPEED` (2 m/s) |
| Flight | parabola; the body turns into the run's contact pose on the other leg | parabola; into the standing jump's forefoot touchdown |
| Land | the run's stance on the landing leg, absorbing the fall | the standing jump's landing, braking at `LANDING_BRAKE` (6 m/s²) |
| Recover | none | the standing jump's |

**The take-off stance:**
- It covers the ground the run's stance covers (the leg sweeps the same
  angle), braking from the run's speed to the leap's, `LOSS_PER_UP` (0.3
  m/s per m/s up).
- So its length is the hips' travel at the mean of the two speeds.
- It leaves as high as the plant leg reaches.

**On one foot the hips are planned forward, the COM up.** The run moves
its pelvis with the root at its speed, and its legs move the COM in the
body by up to 3 mm in a frame (at a toe-off: −0.1, then 2.9, then 0.4 mm).
Planned forward on the COM, the hips took that kick instead, 3 mm in a
frame two frames before handing back. At each seam the hips' rate is the
neighbour's: the run's speed at contact and toe-off, and the flight's hips
rate (the COM's speed less its motion in the body) at take-off and
touchdown. The COM stays on its parabola in the air.

**The vertical path on one foot** (`pushed_height`) is shaped by the floor's
push, not by the path:
- the push is zero at contact and toe-off, so the COM falls at g there,
  continuous with the flights either side;
- between, it is a bump `u^p(1-u)` that never pulls: late in a take-off,
  early in a landing.
- Its size and `p` meet both ends' heights and speeds in closed form.

**Both ends are exact:**
- The jump starts on the run's pose at the contact and ends on its pose at
  the landing foot's toe-off.
- The root difference there is given to the entity (`Resume::handover`);
  stopping, the standing pose's (`Jump::settle`).

## Alternatives considered

- **Time-warp the run's own cycle**, slowing its clock through a longer
  flight: leg retraction at landing would slow with it, and the landing
  foot would skid. Past about 3× a run's flight the warp turns back on
  itself.
- **Drive the pelvis and let the foot IK place the stance leg**: no control
  of the COM, so the floor's push can't be checked or bounded.
- **A one-leg Jump with its own stance shapes**: throws away the measured
  run, and the hand-over and hand-back poses would not match it.

## Traps found

- **Carrying the planted foot at the run's speed** slides it. The run's
  foot rolls heel to toe; carried at constant speed, the heel slid 18 mm
  after it struck and the tip 17 mm before it left. Each stance now has a
  carry table: step by step, the body moves by as much as the sole points
  on the floor moved back under it. Under 1.5 mm a 1/240 s tick.
- **A cubic Hermite asks the floor to pull.** A leap leaving at 2.4 m/s up
  from 27 mm over its contact must gain much speed for little rise. A
  linear acceleration does that by dipping first: −0.4 body weights. A
  quintic held at g at both ends still pulled (−0.5) and dipped 11 cm. The
  push-shaped path never pulls; measured 0-5.4 body weights.
- **The run's body speed is its speed, not its contact velocity.** Read
  through `root_velocity_of` at a contact, the run's root speed was 2.4 m/s
  at a 4 m/s run, and started from that the pelvis braked from 4.26 to 3.69
  m/s in a frame. The run itself had followed that velocity, slowing at
  every contact (see the running note); it now moves at its speed, and the
  leap reads the run's COM rate as that speed plus the COM's motion in the
  body. Read off a different speed than the run goes at, the hop's hand-back
  stepped the pelvis 11 mm.
- **The walk's sway under a leap.** The locomotion layer is composed on
  the target after the walker poses it, so its walk sway (a walk's pivot
  about the loaded feet) moved the leap's pelvis too: a one-frame 9 mm step
  at the hop's hand-back, live only, which no headless replay could show.
  The walk sway now fades out with the run.
- **A hand-back at a frame boundary loses travel.** Ended with a frame posed
  at the toe-off and the run picked up the next frame, the jump's last
  frame showed only the time it had left: the hips moved 13 mm of 57, then
  104. The walker now hands back within the frame the landing foot leaves
  in:
  - the run posed as far past the toe-off as the frame goes;
  - the frame's whole travel given (the jump's rest, the run's after, the
    handover);
  - the run's root motion skipped once (`Stride::given`).
  The jump's start mirrors it: the run's travel up to the contact plus the
  jump's from it.
- **A two-foot landing at the run's speed brakes at 3.5 g.** Stopping 3.6
  m/s over one landing took 0.1 s. The take-off now brakes harder on the
  plant leg, which is ahead of the body and brakes easily (6 m/s² against
  2-3 body weights). The landing then brakes the rest within grip, the COM
  touching down far enough behind the feet to stop over them.
- **Pins on the frame that stands up.** The settle moved the entity while
  the foot IK still pinned the jump's last toes, and both feet went 14 mm on.
  That frame now shows the standing pose with the feet locked, not pinned.
- **A deeper take-off dip scrapes the free foot.** Lowered with the pelvis,
  the run's swinging foot went 3.8 mm into the floor. It is raised to the
  run's own swing clearance.

## Consequences

**Plans** (`puppet_base`):

| | Hop at 3 m/s, 0.08 m | Leap at 4 m/s, 0.3 m |
|---|---|---|
| Flight | 0.27 s | 0.5 s |
| Toe to toe | 1.45 m | 2.5 m |
| Speed after | 2.62 m/s | 3.27 m/s |
| Floor at take-off | 0-3.1 body weights | 0-5.4 body weights |

The free thigh drives near level for a leap, hardly for a hop.

**Live, running on at 4 m/s:**
- flights of 0.47 and 0.30 s, the pelvis falling at −10.3 and −10.2 m/s²;
- planted points on the floor within the run's own figures;
- the leap's hand-back carries the pelvis on without a jolt.

**Live, stopping:**
- the pelvis brakes over about 0.35 s;
- after landing the feet hold within 1.2 mm a frame;
- it ends standing.

**Seen** (Left and Front, gizmos on, following):
- take-off with the free knee driven up and the plant leg extended behind;
- the flight's split, landing on the lead foot and running on;
- stopping: both legs forward, a two-foot touchdown, a squat, then
  standing;
- Front: legs in their lanes, nothing crossed.

**Cost:**
- 72 µs a character a frame running on, 99 stopping, against 53 for a
  standing jump: the springs are led with whole run poses;
- the plan costs 1.0-1.8 ms, once, on the frame the foot lands.

**Hand-over and hand-back:**
- Headless, replaying the walker's frames, the hips' step changes at most
  0.33 mm at either seam (test bound 1 mm).
- Live at 4 m/s, the run's pelvis goes at exactly 4.00 m/s.
- The leap and the hop hand back smoothly (3.34, 3.40, 3.44 m/s; 3.58,
  3.65, 3.68).
- The pelvis's sharpest change through either is 8.4 m/s², against 20-60
  before.

## Revisit when

- Many characters jump at once: lead with shapes instead of whole run poses,
  or cache the run poses a frame needs.

## Related

- [A jump is planned as its centre of mass's path](./a-jump-is-planned-as-its-centre-of-mass-path.md) — prerequisite: the COM planning, pelvis solve, spring lead and landing pins this builds on.
- [A jump forward leans out over its toes, and its travel is the root's](./a-jump-forward-leans-out-over-its-toes-and-travels-as-root-motion.md) — prerequisite: travel as root motion, and the two-foot landing a stopping jump reuses.
- [Running replays measured strides at their Froude number](./running-replays-measured-strides-at-their-froude-number.md) — context: the run whose stance is replayed, and its root velocity.
- [A low obstacle is speed-vaulted as a reshaped running leap](../parkour/a-low-obstacle-is-speed-vaulted-as-a-reshaped-running-leap.md) — applies: a speed vault is this leap with its flight reshaped over the obstacle.
- [A wall is run along on two steps of a lifted leap](../parkour/a-wall-is-run-along-on-two-steps-of-a-lifted-leap.md) — applies: this leap's flight held up by pushes off a wall (`JumpAsk::lift`), its time to touchdown and landing fall solved with them.
