# Procedural Character Animation — Progress Log

Tracks the build-out of `src/character/anim`, the rotation-space
procedural animation plugin. Append new entries at the top, newest first.

Kept separate from [PROGRESS.md](./PROGRESS.md) deliberately: that file is
the `src/hybrid` renderer rewrite's own log, and interleaving two unrelated
subsystems in one chronological list makes both harder to read.

## How to use this file

- **One entry per proven-correct phase/step.** "Proven correct" means
  passing `cargo test --release --lib` cases *and* visual verification per
  [AGENTS.md](./AGENTS.md)'s mandatory pose-verification rules — Front and
  Left, `--gizmos on --show-real-mesh off`, with the specific yes/no claim
  stated before looking at the image.
- **Record real measured numbers**, via `examples/anim_bench.rs`, not
  impressions and not `character_gallery`'s frame time (which is
  vsync-capped at the display refresh and therefore says nothing about
  animation cost).
- **Note dead ends and null results too.** A measured "this did not help"
  is worth as much as a success and is far more easily forgotten.

## Baseline

`cargo run --release --example anim_bench`, measured on this machine
(Linux 6.18, release profile). One frame = advance gait phase, compose the
phase-oscillator layer, integrate all 22 bones' quaternion springs, run
forward kinematics:

| Characters | Frames | p50 | p99 | per-character p50 |
|---:|---:|---:|---:|---:|
| 1 | 2000 | 0.003 ms | 0.003 ms | 0.0029 ms |
| 100 | 600 | 0.183 ms | 0.197 ms | 0.0018 ms |
| 1000 | 300 | 1.838 ms | 1.869 ms | 0.0018 ms |

Cost is **linear in character count** at ~1.8 µs each, with no spikes
(p99/p50 ≈ 1.02). A 1000-character crowd costs ~11% of a 60 Hz frame
budget on the CPU. The single-character number is higher per-character
purely because fixed overhead is not amortized.

## Log

### A stop asked before a start is in finishes the start

- **The skid:** walking 0.25 m/s and told to stop 1 s in, the stopping
  foot skidded 20.8 mm on the floor. Stopped during the first step's fade
  (`Stage::FirstStep`), or while a restart blended in (`Stage::Blending`),
  the transition faded out on the clock, whatever the feet; headless, a
  foot down moved 9.5 mm a frame, and 23 mm stopped mid-restart.
- **Now it finishes what it started:** the first step completes (its fade
  landing at heel contact) and the stop starts from the walk; a restart's
  weight rises, only in single support, to the walk, then stops from
  there. Every fade is placed in the stride.
- **Tried first:** held at its part weight through the next footfall, the
  heel landed under the part-blended walk (7.6 mm a frame); faded out in
  single support anywhere, the swing was set down short as the weight
  reached zero near its landing (23 mm).
- **Tests** (headless, as the walker runs it: the feet the gait has down,
  their sole contacts within 1 mm of the floor in two frames, from the stop
  asked): stopped in the first step at 0.25 m/s, 1.44 mm a frame and 46 mm
  in all, against 1.28 and 58 from walking (9.55 and 100 before); stopped
  as a restart blends in at 1.0 m/s, the same as from walking (23.24 mm a
  frame before).
- **Live:** the same 0.25 m/s walk stopped 1 s in moves no foot over 2 mm
  on the floor (20.8 mm before); it takes its step and closes, 0.38 m.
- Distilled in
  [fade a gait only through single support](./docs/knowledge/character-animation/ik-and-locomotion/fade-a-gait-only-through-single-support.md).
- `cargo test --release --lib`: 1142 passed. Clippy: 0 warnings.

### Strafing diagonally; the shuffle's arms; a clean restart

- **Aside and forward at once** (`Walker::speed` and `Walker::aside`):
  mostly across (45° or more off forward), the shuffle on a diagonal
  (`LegCurves::Shuffle::ahead`), its stride along the way and the stance
  widened for the part across only; mostly forward, the walk with the body
  turned toward its way (`WalkerState::strafe`) and the head looking where
  it faced. Changing between them stops first. Live: 0.25 forward + 0.5
  left went 64° left of forward (asked 63°) at 0.53 m/s; 1.0 + 0.4 went
  21° (asked 22°) at 1.06 m/s, no foot moving on the floor.
- **Arms** (`shuffle::carry_arms`): 0.2 rad out, elbows 0.35 rad more bent,
  a 0.05 rad sway out as the opposite leg swings. Authored: no recording of
  a shuffle's arms was found.
- **The restart creep, fixed:** turning back, the standing foot crept
  2.4 cm, 8 mm up. The fade's blend sank it 2 cm in the pose and the locks'
  speed test let it go mid-stance. The shuffle now tells the locks which
  feet are down (`shuffle::planted`), never the foot a fade swings (counted
  down by the clock while still 9 mm up). A 1.7 cm flick at lift-off left
  after that was the sprung leg lagging a 7.5 cm stance widening in one
  swing: shorter, quicker strides (0.55 × speed, closest 0.12 m) widen it
  2 cm, and the pelvis sinks 16 mm instead of 32.
- **The swing sets down** across by three quarters of it, then straight
  down: the in-air arrival at each touchdown 4–9 mm, from 6–15.
- **Tests:** diagonals forward and back in the stride and feet tests; the
  hands carried out and forward alike; a start from a stand keeps its feet
  down within 8 mm (5.1 / 3 mm; 10.4 if the fade's swinging foot counts as
  down).
- **Live reversal** (0.4 left, 0.6 right, stop): 0.37 / 0.55 m/s; three
  floor-to-floor moves over 2 mm in the run (4.7 mm the stop's last foot,
  3 mm each first swing); never nearer than 0.125 m.
- **Seen**, Front, gizmos then the mesh: the arms carried a little out,
  elbows bent, alike; the turned walk's head toward where it faced.
- **Found, not fixed:** a walk stopped during its start skids the stopping
  foot 20.8 mm (0.25 m/s, stopped 1 s in), shuffle or not.
- Distilled in
  [walking sideways is a shuffle](./docs/knowledge/character-animation/ik-and-locomotion/walking-sideways-is-a-shuffle-on-the-walks-clock.md).
- `cargo test --release --lib`: 1140 passed. Clippy: 0 warnings.

### Walking sideways: the side shuffle; one step aside

- **`shuffle.rs`, `gait::LegCurves::Shuffle`:** walking sideways as a
  gait cycle on the walk's own clock and timing. Each foot's stance sweeps
  it across under the body at a constant rate, its swing carries it back
  on a 5 cm arc; the stance is widened to 0.14 m plus half a stride so the
  feet never cross; the legs are placed by `move_pelvis_and_feet`. The
  walk's cadence (`distance_per_cycle`), start and stop (`transition`) and
  root motion (`root_displacement_between`) serve it unchanged.
- **`Walker::aside`** (m/s) now shuffles. Turning the other way, or walking
  on, it stops first. Stride and width come from the speed asked, held
  through the stop and eased to a new speed (0.4 m/s a second).
- **`Walker::step_aside`** (metres): one step and close from a stand, the
  balance's (`Balance::step_aside`, which replaces the repeating
  `walk_aside`). Gallery: `--step-aside-at T:METRES,...`.
- **Traps:** the walk's feet turned across cross (a mean gap of a whole
  step spread them 0.69 m; it swings by a stride); a foot just set down
  carries no load and the leg solver skipped it, so it hovered 7–10 mm
  (every foot down now counts 0.2); a height-only "planted" test read a
  skimming swing's end as a 53 mm slide.
- **Tests:** a cycle carries the body a stride across within 1 %; the feet
  never nearer than 0.135 m; every foot down on the floor within 2 mm; both
  feet down move alike within 0.5 mm a frame (a non-linear sweep fails it
  at 3.1 mm).
- **Live** (gallery): 0.4 left then 0.6 right, then stop: 0.37 / 0.54 m/s;
  0.2 → 0.6 → 0.3 on the way: 0.18 / 0.58 / 0.29 m/s. Feet down move at
  most 5.0 mm (the standing idle alone: 5.0); never nearer than 0.143 m;
  the pelvis at most 32 mm down; stopped, standing as it stood. One step
  aside 0.25 m and back: out 0.245 m, back within 7 mm, planted feet within
  0.4 mm.
- **Seen** at 0.5 m/s, Front and Left, gizmos then the mesh: a foot lifted
  mid-swing beside one standing, legs never crossing, knees forward, trunk
  upright.
- **Not done:** strafing diagonally (walking and shuffling at once); the
  arms only hang.
- Distilled in
  [walking sideways is a shuffle](./docs/knowledge/character-animation/ik-and-locomotion/walking-sideways-is-a-shuffle-on-the-walks-clock.md).
- `cargo test --release --lib`: 1138 passed. Clippy: 0 warnings.

### Walking aside: side step and close

- **`Walker::aside`** (m/s, + to the left) walks a standing character
  sideways: `balance::Balance::walk_aside`. The weight onto the trailing
  foot, the leading foot's side step (0.1–0.3 m, `aside_step`), the body
  heading for its landing; then the stumble's own landing and join, and
  the travel handed to root motion. Only standing and asked neither to
  walk nor to sit; a walk asked for waits until the feet have closed.
- **Gallery:** `--aside-schedule T:SPEED,...`.
- **Turning back:** the body first comes to rest between the feet, then
  starts as from a stand. With the body still going the legs split (64 mm
  down); stopped over the new trailing foot, the next step jolted 5.4 mm.
- **`Landing::place`:** a balance step's foot is carried across onto its
  planned spot while it swings, before its lock sees it. Left to the
  sprung leg it landed 1.9 cm wide, was locked there, and the leg (a
  centimetre short of straight) left the ankle 18 mm up.
- **A pre-existing foot-lock bug, fixed:** the body's travel reached the
  locks through the live hips, which carry the pose's own pelvic roll. On
  one leg that is ~4°, so each 0.2 m side step's travel came out 14 mm
  vertical and the planted feet hovered 17–18 mm after a few steps. It now
  goes through what the hips hang from, as the ground and obstacles are
  sampled. Forward walking never showed it (a roll about the forward axis
  leaves forward travel level); the live A/B on a 1.0 m/s circle walk
  shows no change in planted slide (3.3/4.9 mm against 4.0/4.8 mm).
- **Tests:** the real rig headless walks aside at 0.200 / −0.187 m/s
  (asked 0.2), planted tips within 1 mm, feet never narrower than they
  stood, jolts under 4.5 mm, sinking under 30 mm, and stops standing as it
  stood (every sole point within 2 mm); turning back at four moments,
  under 35 mm down. The placed landing, and the planted foot's height as a
  rolled body moves aside (4.2 mm through the old frame, under 1 mm now),
  each fail with their fix removed.
- **Live** (gallery, 0.2 left, then 0.25 right, then stop): 0.18 and
  0.23 m/s; the pelvis at most 38 mm down; planted feet within 5.3 mm (the
  idle alone shows 5.0 on this check); the feet never narrower than they
  stood, ending side by side at their standing height.
- **Seen** mid-step, Front and Left, gizmos with the mesh off, then the
  mesh: the leading leg out to the side, the stance leg upright, knees
  bent forward, feet apart and straight.
- **Not done:** faster than ~0.24 m/s (a step and close takes ~1.0–1.25
  s), and stepping aside while walking.
- Distilled in
  [a step aside](./docs/knowledge/character-animation/ik-and-locomotion/a-step-aside-is-the-balances-side-step-and-close.md)
  and [foot locks need the body's travel](./docs/knowledge/character-animation/ik-and-locomotion/foot-locks-need-the-bodys-travel.md).
- `cargo test --release --lib`: 1136 passed. Clippy: 0 warnings.

### Stops land on the spot, in short steps round a smaller circle

- **The stride the gait takes, not `speed^0.65`:** `gait::walking_for`
  holds a walk's excursions at half the recorded ones below 0.54 m/s
  (`puppet_base`); slower, only its cadence drops. The approach assumed
  the stride kept shrinking. Paced to 0.30–0.37 m/s, four of sixteen live
  walks to the table stopped 12–20 cm past the turn's end, beyond what
  the seat takes up. Where the stop would end was predicted right every
  time (within 1–5 cm); the stride it was given was 0.52 m for 0.77.
  - `gait::stride_speeds`, and `approach::stride_at` and `Walked::stride`
    grow the stride only between them.
- **Short steps** (`gait::SHORT_STEPS`, `GaitParams::walking_with_steps`):
  walking to a chair, the stride shortens down to 0.3 of the recorded
  excursions (from 0.25 m/s), not 0.5. Pacing then lands every stop of a
  1–2 m sweep within 1 cm (9 of 100 missed by up to 8.5 cm without).
- **A 0.18 m turning circle at 0.4 m/s** (was 0.25 m at 0.5): half a circle
  in 1.4 s. In the walk's own 0.39 m steps it could not be followed
  (18–22 cm off, tried before); in short steps it can.
- **The stop:** paced only once lined up with the path; at the better of
  this footfall's stop and the next by how much of the seat's range each
  takes (`miss_cost`): past the turn's end it takes up 5 cm, short of it
  25 cm back and 8 cm across. A cost held flat past half a circle short
  stopped a walk a metre out, and is now ever worse further short.
- **Tests:** the model walk takes the gait's stride (short steps), ends
  where the seat takes it up, 11 cm clear of its chair through the turn
  (was 5 cm on 0.25 m); the test chairs stand 0.51 m behind the spot as
  live, not 0.40. Pacing blind to the floor fails five tests; pacing
  before lined up, two.
- **Live at the table,** sixteen walks (four chairs from four starts):
  - before: four past the seat's range (seat clamped at +150 mm);
  - on the stride the gait takes, own steps: all within, two at the edge
    (23.5 cm short of the turn's end; 8 cm across);
  - in short steps on 0.18 m: 9.8 cm short to 2.1 cm past the turn's end,
    2.0–4.9 cm across;
  - two chairs over BRP: the body's middle 19–34 cm clear of the table,
    feet ≥ 2.3 cm from every leg, hips 0 mm off the seat.
- **Live at the gallery's chairs,** A/B on the same input (four placements,
  one from behind): through the turn the body's middle came within
  2.8–4.8 cm of the chair on 0.25 m, 8.4–13.4 cm on 0.18 m; hips 0–2 mm
  off the seat in all eight.
- **Seen** mid-turn, Front and Left, gizmos with the mesh off, then the
  mesh: short steps beside the chair, no leg crossing or twisted foot.
- **Found, not fixed:**
  - Standing on a loose prop 0.3 m up beside the chair, a foot went
    3.8 cm into a chair leg: the feet's obstacle band follows the body's
    ground, and rose above the legs. With no props, 2.3 cm clear.
  - A steady 2–5 cm sideways miss live, ≤ 1.1 cm in the model.
- Distilled in
  [walking to a chair](./docs/knowledge/character-animation/ik-and-locomotion/walking-to-a-chair-turns-on-a-circle-and-paces-its-stop.md).
- `cargo test --release --lib`: 1132 passed. Clippy: 0 warnings.

### Walks to a chair route round tables and chairs

- **`approach::route`:** a visibility graph over every obstacle's corners
  (0.45 m out), straights kept 0.2 m clear, Dijkstra from the goal,
  re-planned each frame.
  - The corner being walked to is kept unless another way is 0.3 m
    shorter.
  - Its own chair is passed first, so a way into the spot (within the
    chair's margin by design) keeps out of the chair alone.
  - No way round: it waits, never straight on.
  - Within 0.5 m of an obstacle it walks at the turning pace.
- **`approach::entry`:** when the turn onto the spot has no room or is in
  the way, the walk comes at it from 1.2 m in front, left or right of it.
  It takes the nearest whose whole final approach is clear, re-checked
  every frame. This replaces walking out in front of the spot, which at a
  table went through the table.
- **`obstacles::RouteObstacles`:** filled by `physics_obstacles` from
  marked colliders around the character and its chair, at body height,
  legs dropped within their seat or top. The marker is renamed
  `FootObstacle` → `Obstacle`.
- **Playground:** chairs pulled out 0.75 m from the table. At 0.65 m the
  turn's end was 12 cm from the table, no room to stand in front.
- **Tests:** each of the table's four chairs from four sides, never into
  the table or another chair, within 6 cm of the spot.
- **Live at the table,** three chairs, one from behind:
  - the body's middle 9–38 cm clear of the table (was 16–39 cm inside);
  - feet ≥ 2.3 cm from every leg;
  - two sat with hips 0 mm off;
  - one stopped 20 cm past its spot, hips 15 cm off (the stop's
    quarter-stride limit in a short, slow final approach).
- `cargo test --release --lib`: 1130 passed. Clippy: 0 warnings.

### Foot obstacles from the physics world

- **`physics_obstacles.rs`:**
  - `PhysicsObstaclesPlugin` and `PhysicsObstacles` on a character;
    colliders marked `FootObstacle` are avoided (opt-in: a ramp or a
    stair is walkable ground, not an obstacle).
  - Each frame before the IK, avian's broad phase finds marked colliders
    within 0.5 m of the feet and at foot height (1–25 cm over the ground
    the body stands on). Each becomes a footprint in the character's
    `AnimObstacles`: an upright box exactly, anything else by its bounds.
- **Playground:**
  - a dining table with four legs and four chairs, each piece its own
    marked collider;
  - `--sit-at-table N` sends the test walker to sit on chair N;
  - `--foot-obstacles off` for an A/B, and `--step-seconds S` for
    offscreen runs.
- **Live A/B, same input,** walking to sit at the table: with the
  obstacles off a foot went 5.7–7.0 cm into a table leg; on, every foot
  2.2–2.5 cm clear of every leg.
- **Found, not fixed:** the walk to a chair routes round that chair only.
  At the table the body walked through the table (~6 s inside its
  footprint) before reaching its spot. That is the route-planning step.
- **Tests:** the footprint of a turned box and of a ball. 1129 passed.
  Clippy: 0 warnings.

### Foot obstacles move into the foot IK, as a general probe

- **`obstacles.rs`:**
  - `FootObstacles` (a capsule query: a foot's heel-to-tip line and a
    clearance), set on a character as `AnimObstacles`, like `AnimGround`;
  - `Footprints` (boxes on the floor) as the plain implementation;
  - `Chair::footprint` gives a chair's.
- **`solve_foot_ik`** moves each toe target, and a planted foot's lock,
  by the probe's answer. The walker's chair-only code is gone; the gallery
  gives its character the chair.
- **Clearance 12 cm → 8 cm:** the foot's half width, the ~2 cm the drawn
  heel falls short of the IK's target, and 1 cm spare. It no longer
  covers spring lag, so it doesn't depend on walking speed. At 5 cm the
  drawn foot still went 1.9 cm into a leg.
- **Points → a line:** a post under the arch had heel and toe pushed
  opposite ways and cancelling (the new IK test caught it). Crossing a box
  now takes the smallest separating move.
- **Live, four placements:** every foot 3.0–4.1 cm from every chair leg
  (was 1.7–5.8 at 12 cm in the walker), the drawn foot ≥ 6.2 cm from the
  footprint, seated hips 0–1 mm off, feet 0 mm of slide.
- See [the note](./docs/knowledge/character-animation/ik-and-locomotion/feet-keep-clear-of-obstacles-in-the-foot-ik.md).
- `cargo test --release --lib`: 1128 passed. Clippy: 0 warnings.

### Feet kept clear of the chair's legs through the turn

- **Measured first:** each foot (heel to tip, 4.5 cm half-wide) against
  the gallery chair's real leg posts, over BRP. Every placement put a foot
  ~5 cm into a front leg, and a heel 14 cm under the seat front.
- **`Chair::foot_clear`:** keeps each foot's outline 12 cm off the
  footprint.
  - Swinging feet are moved where they will land (`AnimFootIk::displaced`).
  - Planted ones are moved where they are held (new
    `FootLock::shift_anchor`), since the turn pivots a planted foot about
    the body.
  - Each point moves away from the footprint's nearest point, combined
    per axis.
- **Four wrong turns on the way:**
  - swinging feet only (planted ones still 4 cm in);
  - a 5 cm margin (the sprung leg trails the pose 4–7 cm, so 2–4 cm
    remained);
  - out by the nearest side (a 20 cm flip at the corner's diagonal broke
    the lock);
  - the deepest point only (a flip from heel to tip).
- **Live, four placements:**
  - feet at least 2.2 cm from every leg, none under the seat front;
  - seated hips 1 mm from the middle;
  - feet slid 0 mm sitting, ≤ 2 mm rising;
  - the walk's path unchanged.
- **Seen** from behind at the old contact frame: the foot beside the
  chair, floor between it and the leg.
- **Tests:** the move out, its continuity round a corner for a point and
  for a whole foot.
- `cargo test --release --lib`: 1123 passed. Clippy: 0 warnings.

### Round the chair, onto its middle, and no foot slides

- **Round the chair:** a straight to the turning circle that would cross
  the chair heads for a corner 0.45 m out (`approach::round`). It never
  goes back to a corner it has reached, and plans afresh once clear.
  - `Chair::standard` gives the footprint; turns sharper than 0.8 rad are
    walked at the turning pace.
- **The turn ends 10 cm in front of the spot**, its dip off the chair.
  - Before, the body passed 15 cm past the seat's front edge on every
    approach. Now at most 7 cm (test), by the corner.
  - Smaller circles (0.12, 0.18 m) and a pivot were tried live and lost:
    18–22 cm off the spot, or the feet swinging as far.
- **The pace** is set at most once a step, and the stop waits until the
  legs walk at it. Re-pacing every frame made the speed hop 0.39–0.66 m/s
  and stopped mid-turn 25 cm short.
- **`sitting::Seat::across`** (±8 cm) slants the shins so the hips land
  on the seat's middle. Chair poses are set on their feet's middle both
  ways (`on_feet`); `back` range ±15 cm.
- **Leg IK hinge fix** (`legik.rs`): the knee turned about the raw axis.
  A sideways-leaning leg put the ankle off target, and the foot turned 9°
  about its planted toe. See [the note](./docs/knowledge/character-animation/ik-and-locomotion/a-knee-hinge-must-be-square-to-the-line-to-the-target.md).
- **Both feet stay planted 0.3 s after standing up** (`STOOD_HOLD`): the
  rise's end slide went from 11–17 mm to 0 mm.
- **Tests:**
  - routing from behind and the gallery's chairs (each fails without the
    routing);
  - the turn's distance past the seat front;
  - seats moved back and across;
  - planted feet frame by frame on a moved seat;
  - the leg IK reaching sideways (fails with the old hinge).
- **Live, four placements** (one with the chair's back to the walker):
  - stopped 53–125 mm off the spot;
  - seated hips 0–5 mm from the seat's middle;
  - feet 0 mm of slide sitting and rising;
  - the body never inside the chair.
  - Known: a heel passes up to 8 cm under the seat's front mid-turn, and
    on the default chair touches its front leg for a moment.
- `cargo test --release --lib`: 1122 passed. Clippy: 0 warnings.
  - `physics::gpu::frame_test` failed once under the full parallel run,
    and passed alone.

### Walking to a chair, turning round, and sitting on it

- **`approach.rs`:** given `Walker::chair`, the walker walks a Dubins path
  to the spot in front of the chair.
  - Straight, then round a 0.25 m circle at 0.5 m/s: a 180° turn in
    ~1.6 s and ~2.5 steps (Robinson 2018: 1.5 s median).
  - It looks at the chair on the way, and walks out first when too close
    to turn onto the spot.
- **The stop is paced.** A stop can only end a footfall plus a last step
  on (half strides, 0.39 m apart); stopped at the nearest, it stood up to
  0.29 m off.
  - Over the last 1.5 m the speed is set so a footfall's stop lands on
    the spot (Lee, Lishman & Thomson 1982).
  - The stride is measured from whole steps walked (`walker::Walked`):
    the gait's straight-walk stride was 5–18 % off on the circle.
- **`sitting::Seat::back`:** the shins swing to put the hips on the seat
  however far (up to ±12 cm) the walk stopped short or past.
- **Gallery:**
  - `--chair X,Z,HEADING` (default `-1.5,-1.5,180`; `here` sits in place);
  - `--step-seconds S`: a fixed clock step, for offscreen runs on a
    software renderer.
- **Tests:**
  - a point walk that stops like the transition lands within 4 cm, from
    five starts at four stride phases (it fails with the pacing off);
  - a seat moved back or forward keeps the feet and the seat height.
- **Live, three placements:**
  - stopped 28–103 mm off the spot;
  - seated hips on the seat in depth, 26–43 mm off its middle across;
  - feet slid ≤ 4 mm while sitting, nothing under the floor.
- **Known:** after the rise, the right foot settles 11 mm sideways.
- **Gap:** no routing round the chair from behind.
- See [the note](./docs/knowledge/character-animation/ik-and-locomotion/walking-to-a-chair-turns-on-a-circle-and-paces-its-stop.md).
- `cargo test --release --lib`: 1117 passed. Clippy: 0 warnings.

### Sitting on a chair and on the floor, and standing up

- **`sitting.rs`:** each pose solved on its contacts:
  - chair (0.45 m): upright, reclined, legs crossed, leaning forward;
  - floor: cross-legged, propped, hugging the knees, side-sit, kneeling.
- **`Walker::sit`:** sits or stands through keys (`walker::Posture`).
  - **Chair rise:** timed to Schenkman's phases and the measured 1.9 s
    (28/18/54 %), feet where the character stood.
  - **Floor:** down by the get-up's squat and propped sit, kneeling by a
    half-kneel and a tall kneel.
  - **Gallery:** `--sit NAME --sit-at S --stand-at S`, Sit/Stand buttons,
    and a chair spawned under the seated hips.
- **Shared:** `rig::blend_in_world` (from the get-up's rise) and
  `AnimFootIk::legs_free`.
- **Fixed on the way:**
  - The leg IK flattened cross-legged knees (now free when the legs
    leave their planes).
  - Blends swept feet 253–348 mm through the floor. Now static keys laid
    and refined so feet go straight, plus a smooth lift.
  - Per-frame foot turns and leg solves popped (5.9 m/s, 35 cm, 260 mm).
    Removed.
  - Sprung toes trailed 47 mm under (the sprung pose is now lifted).
  - Rigid tucked toes went 71 mm under (bent onto the floor now).
  - A root jumped up to 3 cm between keys (held feet now move with the
    blend).
- **Tests:** contacts and floor on both rigs, the chair's feet and seat,
  the knee angle, the 1.9 s rise, and every cycle at 60 Hz (≤ 6 mm under,
  < 60 mm a frame).
- **Live, every cycle:**
  - lowest point ≥ 0 (chair ≥ +15 mm);
  - fastest joint 1.1 m/s on the chair, ≤ 3.4 m/s on the floor (the arm
    swung forward to rise);
  - chair feet 0 mm off where it stood.
- See [the note](./docs/knowledge/character-animation/ik-and-locomotion/sitting-down-and-standing-up-go-through-solved-keys.md).
- `cargo test --release --lib`: 1111 passed. Clippy: 0 warnings.

### A falling body keeps its passive joint tone

- **The fall was a puppet's:** no tone and uniform damping only, so limbs
  swung free into their stops. Four pushed falls ended there:
  - hips at −29.9° (30° extension stop) and 116.6° (120° flexion);
  - abduction 44.4° (stop 45°);
  - knees and elbows locked at −5°.
- **`passive.rs`:** each joint pulls its body toward a relaxed pose
  relative to its parent (not the world pose that failed at tone 0.15).
  - Stiffness is `k₀·(1 + (θ/θs)²)`, solved implicitly every substep with
    the parent taking the reaction (`drive_impulse`).
  - The relaxed pose (`getup::relaxed`) is NASA's neutral body posture
    (STS-57 medians), with legs at Riener & Edrich's knee zero for straight
    hips (hip 15°, knee 20°).
  - The knee's gains are fitted to Riener & Edrich: 4.1/9.5/22.7 N·m
    against their 4.5/6.2/16.5 at 60/90/130°. Other joints are scaled
    from it, and the hip's damping is the measured 1.9–4.6 N·m·s/rad.
  - `Ragdoll::passive_tone` scales it.
- **Same carried falls, tone off → on:** peak joint spin 52/137/54 →
  29/43/29 rad/s. Every fall still rests (3.9–7.6 s). The backward fall's
  hips stay off their stops (flexion 88.5°, abduction 2.9°).
- **Live:** the collapse buckles at the knees and rolls back with limbs
  bent in mid-range, rests relaxed, and rises (pelvis back to 0.94 m).
- **Dead ends on the way:**
  - A relaxed knee of 50° with hips of 31° (the weightless crew's) raised a
    supine body's knees against gravity; they never rested.
  - Pulling a hinged knee in three directions rolled the shin against its
    hinge (1.4–2 rad/s), so hinged joints pull about their axis alone.
  - Ankle tone (3, then 1 N·m/rad, all axes or flexion only) made the light
    feet fight the floor's friction and creep or jolt (0.05–0.5 m/s), so
    ankles have none.
- **Tests:**
  - new: `passive::tests`, `the_relaxed_pose_has_the_neutral_body_postures_angles`,
    and `passive_tone_slows_a_falling_limb_about_its_joint_and_still_rests`
    (the same input with tone off and on).
  - moved:
    - the flesh test allows a light hand's 24 mm impact dip and 10 mm at
      rest (passing through was 101–180 mm);
    - hip adduction may go 1° past its stop, as the elbows do;
    - the side-rise uses pushes straight out to the side, since the
      diagonals now land face down.
- See [the note](./docs/knowledge/character-animation/ragdoll-and-physics/a-falling-body-keeps-its-passive-joint-tone.md).
- `cargo test --release --lib`: 1105 passed. Clippy: 0 warnings.

### A falling and rising hand no longer pops

- **Live, a hand turned up to 163° between two samples** of a fall and
  rise. The pre-hand build `f997487` did the same. Per frame, three drawn
  floor corrections, each solved afresh every frame:
  - **The wrist turn snapped on and off:** 68-70° a frame against a body
    turning 15-19°.
  - **Its axis flipped:** the level line over a hand hanging straight down
    is undefined; the same lift swung 78°.
  - **The rising arm's elbow fold jumped:** between none and ~2 rad; the
    forearm turned 80-115° in a frame. This was found live with a
    temporary per-frame report; the pushed headless falls never hit it.
- **Fix** (`hold_clear`):
  - The wrist and a rising arm's shoulder each hold their turn as one
    rotation, eased back at 3 rad/s as far as the point stays clear, then
    turned further up only as far as needed.
  - Arms lift at the shoulder instead of folding the elbow.
  - Knee tucks deepen at once and let go at 6 rad/s.
- **Dead ends:**
  - Rate limits both ways: fingertips 25 mm under the floor, hips hoisted
    72-99 mm.
  - Angle plus axis held: a rising hand turned 134°.
  - The elbow fold rate-limited: hoisted 99 mm.
  - Eight times the wrist's damping: no change; those turns are contact
    impulses.
- **New test** `a_drawn_hand_turns_no_faster_than_its_body`, on a plain
  collapse and four pushes:
  - falling: 6-23° faster than the body (was 49-59°);
  - rising: 4-8° a frame (was 26-30°);
  - it fails at 33° with the limits off.
- **Live, 10 collapses:** no forearm over 50° in a frame (before, 4 of 6).
  Fingertips stay out of the floor.
- **What is left is physical:** a hand's body slapping the floor turns up
  to 45° a frame.
- See [the note](./docs/knowledge/character-animation/ragdoll-and-physics/a-drawn-floor-correction-is-held-between-frames.md).
- `cargo test --release --lib`: 1100 passed. Clippy: 0 warnings.

### Fingers give way on the floor one by one; no fingertip under the floor

- **The get-up's push-up put the fingertips 84–90 mm under the floor**,
  with flat fingers too. Two causes:
  - The wrist cleared an estimated fingertip (Winter's hand length), 34 mm
    short of this rig's middle finger.
  - The bind's thumb points out of the palm, so a flat palm pressed it
    76–83 mm into the floor.

  Now the wrist clears the real fingertips, and a flat hand's thumb lies in
  the palm's plane.
- **Fingers bend on contact instead of the whole hand going flat** (the
  previous commit straightened every finger while down, a board).
  - A falling hand stays relaxed.
  - A finger whose tip would enter the floor bends from where it is as
    little as clears it: straighter when its palm faces the ground, curled
    further when it faces away.
  - Bends are limited to 12/s and ease back to relaxed at 4/s.
  - Choosing the smaller bend each frame flipped a finger from 86° to 8° in
    one frame. Keeping its side left a fist under a flat palm and turned
    the wrist 60° in one frame.
- **Three falls and get-ups** (BRP):
  - no fingertip below the floor;
  - fingers move at most 38–47 mm per sample within the hand's frame
    (106–139 mm before the rate limit).
  - Mesh: lying, the fingers rest curled; in the push-up, they lie
    straight along the floor.
- **Already there, not changed:** the whole hand turns up to 87–163° per
  sample during a fall and rise, on `f997487` too (the get-up blend and the
  wrist's turn).
- `cargo test --release --lib`: 1099 passed. Clippy: 0 warnings.

### Walking arms swing back, elbows fold, hands hang relaxed

- **The arm swing was a march.** Live at 1.3 m/s, the upper arm went 28°
  forward and 10° back of vertical, and the elbow moved only 20–30°.
  Murray (1967), 30 men at 1.54 m/s: 8° forward, 24° back, elbow 17–47°.
  The upper arm swings mostly back, and the hand comes forward by the
  elbow.
  - The swing is now centred behind the shoulder (`ARM_SWING_CENTRE`
    −0.5, `arm_swing` 0.21).
  - The elbow has a new `GaitParams::elbow_carry` (walk 0.29 with
    `elbow_bend` 0.73; the run keeps 0.6).
  - At 1.54 m/s on puppet_base: +7°/−25°, elbow 17–46° (pinned by a test).
    Live at 1.3 m/s: +9°/−18°, elbow 18–46°.
- **Fingers curl** (`hand.rs`): the rig's finger joints are not `Bone`s
  and sat flat at the bind (9° over the middle finger). They now curl to
  Lee et al.'s relaxed angles as the rig binds (live: 56°). The thumb bends
  across the palm: bent toward it like a finger, it stuck out into the
  thigh.
- **Fingers straighten while the ragdoll is down**, over 0.3 s. Curled,
  the fingertips lay 45 mm into the floor; now 11–20 mm, as before (the
  old build: 11–26 mm).
- **Already there, not fixed:** the get-up's push-up puts the fingertips
  84–90 mm under the floor for a moment, on the old build too. Its palm
  uses a virtual fingertip.
- Front and Left, gizmos without the mesh, then the mesh and hand
  close-ups: the forearm leads, the upper arm trails, the fingers are
  curled with the thumb along the index finger.
- See [the note](./docs/knowledge/character-animation/ik-and-locomotion/a-walking-arm-swings-back-and-its-hand-hangs-relaxed.md).
- `cargo test --release --lib`: 1096 passed. Clippy: 0 warnings.

### Playground: walkers seek goals; no slide while getting up; steering cost

- **Goals** (`Seeker`): each walker heads up a ramp or the stair through a
  lined-up entry point, across a fence, or to a random floor point. Terrain
  and avoidance turns go first and are followed by a 1.5 s detour; a goal
  is given up after 25 s.
  - 16 ragdolled walkers among 60 props, ~2 min: 17 climbs or crossings
    (ramps 15°/25°/35° 4/5/3, stair 1, fences 0.1/0.2 m 4/4), 16 falls
    and 16 get-ups.
  - 0 of 384 pelvis samples outside the room or inside a solid.
  - About half the reachable goals are still given up in the crowd.
  - `--goals off` restores pure bouncing. A `--start` character keeps its
    heading.
- **Library fix, walker:** the gait kept walking through a fall and the
  rise, so the rising body slid 1.53–1.69 m forward. Now the gait is set
  to standing at the fall and asks no speed while the ragdoll is down. The
  root moves 0.00–0.23 m through the rise (same three starts, before and
  after), and the walker then starts from a stand.
- **Steering cost**, timed in its systems (`--bench 10 --characters 16`,
  two runs each): 0.016 ms per walker per frame without physics,
  0.029–0.032 ms with 16 ragdolls.
- Props doubled to 60 by default.
- See [the note](./docs/knowledge/character-animation/ik-and-locomotion/steer-over-terrain-by-the-ground-profile-ahead.md).
- `cargo test --release --lib`: 1091 passed. Clippy: 0 warnings.

### Playground: ramps, a 5 m stair, fences; climbing and falling off

- **Terrain, all free-standing:**
  - ramps of 15°/25°/35°/45°/50° rising to platforms 1.0-3.0 m high;
  - a 5 m stair, 30 risers × 167 mm on 300 mm treads, within IBC 1011.5.2
    (≤ 178 mm riser, ≥ 279 mm tread), 2R + T = 0.63 m;
  - fences 2 × 1 m, 0.1-1.6 m high.

  Characters still spawn at random, now clear of every structure, among
  the dynamic props. Against the walls, a walker that fell off a platform
  into the 1 m gap beside it walked out through the platform and the
  wall.
- **`PhysicsGround::max_step` is relative to the ground the body stands
  on**, not the floor, which hid every riser above the second. A new
  field, `under`, is the raw mean under the soles.
- **Steering reads the ground profile ahead** (downward rays every 0.2 m;
  rise ≤ 0.3 m, slope ≤ 40°). Ragdolled walkers fall off ledges.
  `--start X,Z,YAW` places the first character.
- Measured (ragdoll, one walker per structure): it climbs the 15°/25°/35°
  ramps to 1.0/1.5/2.0 m and the stair to 5.0 m, then steps off, falls,
  gets up and walks on. It is turned at the foot of 45°/50° and at
  fences ≥ 0.4 m, and steps over 0.1/0.2 m.
- 16 walkers for 60 s: 0 of 208 pelvis samples outside the room or under
  the terrain.
- Two traps, both fixed:
  - the eased height lagged 0.18 m on the 35° ramp and read as a riser;
  - a hollow tilted-slab ramp reflected a walker into itself; ramps are
    now solid wedges.

  See [the note](./docs/knowledge/character-animation/ik-and-locomotion/steer-over-terrain-by-the-ground-profile-ahead.md).
- `cargo test --release --lib`: 1091 passed. Clippy: 0 warnings.

### Playground: smaller room, avoidance, feet on props, the wall-turn foot flicker fixed, physics cost

- **The room is 25 × 25 m.**
- **Characters avoid each other** by predicting each pair's closest
  approach over 1.5 s from heading and speed, and turning aside 0.6 rad
  when it would be under 1 m (head-on, both keep right). Six characters,
  60 s:
  - avoidance on: closest pair 1.00 m, never under 0.8 m;
  - avoidance off (`--avoid off`): 0.14 m, 28 samples under 0.8 m.
- **Wall bounces** got side whiskers and a 25° minimum leaving angle.
  Before, the jitter could turn a shallow reflection back into the wall,
  and a walker came within 0.13 m. Now the closest is 1.17 m.
- **Feet stand on props** (`physics_ground`, `PhysicsGround`): raycast
  grids under each foot; the body rises to the mean under its soles.
  On a 0.15 m platform the stances are at +0.15 and the hips rise
  0.935 → 1.085 m. See
  [the note](./docs/knowledge/character-animation/ik-and-locomotion/feet-stand-on-the-physics-world-through-sampled-ground.md).
- **The foot jerk turning at walls was a library bug.** The foot locks
  were handed the turn about the character's world position, while their
  anchors live in the pose's frame. Through a wall turn 7 m from the
  origin, a planted foot flicked 0.55 m every frame and the hips 6 cm.
  Fixed: the worst per-frame move is now 9.9 mm (foot) and 2.9 mm (hips).
  The test fails at 340 mm before the fix. The gallery's circle walk is
  unchanged (2.35 m radius).
- **Physics cost** (`--bench 10 --characters 16`, vsync off, two runs,
  median frame), per character per frame:
  - a ragdoll, 0.24-0.31 ms (p99 15 → 25 ms at 16);
  - a capsule, within noise;
  - the foot rays, 0.09-0.11 ms;
  - animation and rendering together, about 0.4 ms.
- `cargo test --release --lib`: 1091 passed. Clippy: 0 warnings.

### Walking character in the library; physics character playground

- **Extracted from `character_gallery` into the library**, so any example
  or game drives a character the same way:
  - `character::anim::humanoid` (`HumanoidPlugin`, `spawn_gltf_humanoid`)
    loads a glTF humanoid and binds it, any number of characters;
    proportions are a `HumanoidProportions` component.
  - `character::anim::walker` (`WalkerPlugin`, `Walker`, `WalkerState`)
    holds the walk driver, root motion, and the fall/get-up glue. A
    walker is steered by `speed`, `Steer::{Straight, Circle, Toward}`,
    `push`, `fall_now`.
  - `despawn_ragdoll` removes a ragdoll's bodies and joints (unit-tested).
- **The gallery is now a consumer.** A/B against the pre-refactor build,
  same flags: the circle walk measures radius 2.35 m and 1.16 m/s on both.
  Standing, walking and ragdoll-walking screenshots match.
- **A latent binding bug, fixed:** binding captured the spawn heading as
  part of the rig's bind, so a character spawned turned stood with its
  hands overhead. The gallery never showed it, because it always binds
  at yaw 0. See
  [the note](./docs/knowledge/character-animation/rig-and-retargeting/bind-a-rig-at-its-own-facing-not-its-spawn-heading.md).
- **`examples/physics_character_playground.rs`:**
  - a 50 × 50 m room, floor and 3 m walls as static colliders;
  - 30 dynamic cubes, spheres and capsules dropped from 5 m around the
    centre;
  - walkers that turn off walls along the mirrored heading ± jitter;
  - a free-flight camera (Bevy's `FreeCamera`, the `free_camera`
    feature);
  - physics by camera distance: a pinned ragdoll within 10 m, a kinematic
    capsule beyond, with 1 m hysteresis.
- **Measured (BRP):**
  - Kinematic: 70 s at 1.22 m/s, 2 wall turns, never nearer a wall than
    0.9 m.
  - Ragdoll: 1.21 m/s, the pelvis never below 0.87 m.
  - Four characters in distance mode switched ragdoll ↔ capsule at 9 / 11
    m both ways, with no falls in 60 s.
  - Props are kicked aside.
- **Not measured:** physics cost per character in each mode (the window
  is vsync-capped).
- `cargo test --release --lib`: 1089 passed. Clippy: 0 warnings.

### Closing a stance after a step, and the sideways pelvis drop: tried, not fixed

- **The push matrix is noisy.** Delaying a push 7-25 frames (the idle's
  breathing phase) flips as many cells as a change does. Added
  `probe_own_feet_push_score`, which scores 4 directions × 0.5-0.8 m/s ×
  5 timings, standing 12 s later.
  - Committed stepping: 53 / 80 caught (25 with the feet left apart).
  - Stepping off: 15 / 80.
  - The earlier "0.7 / 0.8 / 0.7 m/s" limits were one timing's.
- **Root causes found** (commanded COP traced against the motion):
  - With dominant planted feet, the trailing leg is a tether at full
    stretch.
  - In double support the COP law cannot steer the closed chain, and the
    body moved against its command (0.29 m/s the wrong way).
  - Stepping out sideways leaves the body on the far foot as the COM
    races off it: ~150 N·m asked of 90 N·m of hip abductors.
- **Scored and dropped** (each below the baseline or within its noise):
  - crossover steps (pelvis roll 15° → 2-6°, but crossed stances 10 cm
    low that fell later: 52, or 47 with the hip hold);
  - early joins on a taut leg (45);
  - legs holding the hips at a moving set point (53);
  - a lateral lunge with the shift foot chosen once settled (45-54);
  - shuffle side steps (every side push fell);
  - legs solved ahead toward the rest point (lost forward pushes);
  - a structural knee and ankle twist (within noise).
- **Code unchanged; the attempt is kept as a patch, not committed.**
  Closing a stance needs a redesign of double support: whole-body control
  of the pelvis and COM by both legs, and feet that can unload. See the
  [stepping note](./docs/knowledge/character-animation/ragdoll-and-physics/a-step-on-its-own-feet-aims-its-swing-in-the-world.md).

### Stepping on its own feet works; forward weakness explained; speeds sourced

- **The step, on by default** (`steps_on_own_feet`). Found by tracing
  pushes frame by frame:
  - The swing thigh aims in the world (SIMBICON). Held to its pelvis, it
    followed a ~20° pelvis yaw: this was the 7-13 cm outward drift.
  - The swing is tracked implicitly (per-axis gains, the leg as a lump,
    4 Hz) with the arc's rate and acceleration fed forward. Explicit, it
    chattered; sized on the lump, the shin spun; without the feed-forward,
    it landed at 0.18-0.21 of a 0.31 m step.
  - Swing targets take their twist from the animation; from the bodies'
    own twist, the hip spun up to 9 rad/s.
  - The landing is re-aimed each frame at where the capture point will be,
    4 cm outside it sideways. A step ends when it arrives. The body rests
    where it was caught, and the other foot joins once the body has
    settled.
- **Measured** (`probe_own_feet_push_matrix`, `puppet_base`):

  | Direction | Feet only | Stepping |
  |---|---|---|
  | Forward | 0.4 m/s | 0.7 |
  | Back | 0.5 | 0.8 |
  | Sideways | 0.5 | 0.7 |

  - Forward and back recoveries end joined: hips within 15 mm, pelvis
    within 5°. Side steps end in a wide stance that does not join.
  - A step lands ~1 cm from its aim on a 22 cm step.
  - Live, Front and Left gizmo views:
    - `puppet_base`: a 3 m/s backward hip blow steps and stands in 8 of
      9 runs; 4-5 m/s fall.
    - `character.glb`: 2.5 m/s stands in 3 of 3; 3 m/s falls.
- **Forward weaker than back, explained.** The stance holds the COM 8.6 cm
  ahead of the ankles, leaving 13.5 cm of sole ahead and 17.3 behind. The
  limits are room / √k: 0.41 and 0.52 m/s, matching the measured 0.4 and
  0.5.
- **Tried and dropped** (no gain on the matrix, or a loss):
  - holding the COM at 5 cm (turned the limits round but broke side
    steps);
  - an 8 cm swing lift;
  - a structural knee twist limit;
  - capping the step at the leg's reach;
  - choosing the joining foot by where the step caught the body.
- **Speeds sourced** (none was a fitted Hill speed; these are unloaded
  peaks): wrist 20 → 23 and shoulder 20 kept (Jessop & Pain 2016); trunk
  15 → 12 (axial rotation 12.2); neck 15 kept (Hernandez & Camarillo
  2019). The trunk's flexion-extension speed is still assumed.
- [Stepping note](./docs/knowledge/character-animation/ragdoll-and-physics/a-step-on-its-own-feet-aims-its-swing-in-the-world.md).
  `cargo test --release --lib`: 1088 passed. Clippy: 0 warnings.

### Muscle gaps: strength per direction, measured speeds; stepping on its own feet tried

- **Strength per axis and direction.** Each drive caps its torque per
  axis of the character (left, forward, up) and per way, in N·m/kg
  (Harbo 2012 and others): ankle plantarflexion 1.8, dorsiflexion 0.57;
  knee and elbow side axes `HINGE` 5 (a hinge's frontal torque is the
  joint's structure, not a muscle; a 0.6 cap there let the body wander
  sideways). The right side mirrors forward and up.
- **Measured maximal speeds** (Anderson 2007): ankle 21, knee 24, hip 19,
  elbow 16.5 rad/s, the rest estimated.
- **Only the balance correction lags** by the twitch time; the static
  weight acts at once. Lagging the whole command swayed the trunk
  without end (28.9 mm of wander after a side push, now under 3).
- **Push limits, with the directions corrected** (the drawn rig faces
  −Z; earlier entries had forward and back swapped): forward 0.4 m/s
  caught, 0.5 falls; back 0.5 caught, 0.6 falls; sideways 0.5 caught,
  0.6 falls. Why forward is weakest is open.
- **Stepping on its own feet: built, off by default**
  (`Ragdoll::steps_on_own_feet`). Swing by two-bone IK with lumped-inertia
  tracking each substep, the foot carried flat, the stance hips held at
  standing height, support from the whole sole face. The swing foot
  covers ~70 % of a planned step (153 of 221 mm), but drifts 7-13 cm
  outward at any tracking stiffness, and the body then runs away in
  side steps. See the
  [standing note](./docs/knowledge/character-animation/ragdoll-and-physics/a-standing-ragdoll-carries-its-weight-through-joint-torques.md#stepping-on-its-own-feet-tried-experimental-off-by-default).
- `cargo test --release --lib`: 1087 passed, 20 ignored. Clippy: 0
  warnings.


### Steps 4.2, 4.4, 4.5: muscle strength, twitch, speed, and mode switching

- **4.2 budgets.** Each joint drive's whole torque is capped at maximal
  voluntary isometric torque per kg times body mass (Harbo et al. 2012,
  young man): ankle 1.8, knee 3.5, hip 2.5, trunk 3.0, neck 0.7, shoulder
  1.0, elbow 0.67, wrist 0.33 N·m/kg. Winter's walking peaks (knee 0.5)
  are what a walk uses; a standing knee already needs 0.49. The cap
  binds: at a tenth of its strength the body folds over 15 cm (4 mm
  without the cap). A 0.4 m/s push and a forearm blow stay within every
  budget.
- **4.4 muscle behaviour.**
  - Force-velocity on the cap: Hill's hyperbola shortening (k 0.25),
    Thelen's 1.4 plateau lengthening.
  - The commanded torque lags by Winter's twitch time (legs 75 ms, arms
    50, trunk 60 by choice), a critically damped response solved
    exactly. The drive's stiffness stays immediate.
  - The lag costs pushes: backward caught to 0.4 m/s (0.5 without it;
    first logged as forward, see the entry above), sideways under 0.6
    (0.6 without). At twice the twitch, even 0.4 falls.
- **4.5 mode switching.**
  - On its own feet the screen shows the bodies. `stop_standing_on_own_feet`
    pins the root where the body stands and eases it to the animation;
    both switches blend over 0.3 s.
  - Per frame at worst: on 1.4 mm and 0.5°, off 2.0 mm and 0.6°.
    Snapped: 3.7 mm and 1.05° on, 20.4 mm and 2.2° off.
  - A hit's stun now weakens the drives: a struck forearm swings over
    10° and is back within 5°, standing. Gallery: `B` toggles.
  - Bench: one ragdoll 0.64-0.88 ms a frame pinned, 0.71-1.01 on its own
    feet, within the noise.
  - Front and Left, both rigs: the body drawn from its bodies stands as
    the pinned one does.
- `cargo test --release --lib`: 1083 passed.

### Step 4c: the body on its own feet balances

- **Winter's law on the measured COM** (`carry_weight`):
  `COP = COM + (COM − rest)·k·ω² + v·2ζωk` with the kinematic balance's
  gains, held in the planted soles' hull. Fed as statics: the ground pushes
  each planted foot at its share of the COP with the load's weight and the
  pendulum's horizontal force. Every stance joint carries that push's
  moment, every other part its load under `g − a`. Each leg's share
  follows the COP, which is the hips' load/unload.
- **Physical limits.** Each planted ankle's torque is held to what keeps
  its pressure in its sole (`within_sole`). A capture point more than 2 cm
  outside the soles makes it fall, through the existing fall.
- **Measured** (headless, `puppet_base`):
  - Still within 1.5 s, then within 5 mm for 55 s; the 4b sway was ±2 cm,
    undamped.
  - 0.4 m/s pushes each way are caught: the COM is back within 1 cm, the
    feet move ≤ 3 mm, the ankles carry < 1.6 N·m/kg.
  - Limits without a step: back 0.5 caught, 0.6 falls; forward 0.4
    caught, 0.6 falls; sideways 0.6 caught, 0.8 falls. (First logged
    with forward and back swapped: the drawn rig faces −Z, so a +Z push
    is backward.)
  - Live, both rigs: hips within 5-6 mm over the last 5 s, feet 0.0 mm.
- **Tried and dropped:**
  - the COP on the ankles alone: the knees gave way, ±5 mm sway, never
    settled;
  - planted ankles without pose stiffness: the shins tipped 60°;
  - no ankle limit: 1.2 m/s "caught" by feet glued to the floor.
- **Each part fails a test when removed.** Without the law, the COM ended
  49 mm off after a push and wandered 27 mm in a minute. The sole limit
  has its own unit test.
- `cargo test --release --lib`: 1076 passed. Clippy: 0 warnings.

### Step 4b: a ragdoll standing on its own feet

- **`Ragdoll::stand_on_own_feet`**: the root is released, gravity acts in
  full, and every joint holds its pose with a torque between its two
  bodies (`joint_drive::JointDrive`), not the old per-body acceleration.
  Three parts, each load-bearing (removed, the standing test fails):
  - **Implicit drives every substep.** `apply_joint_drives` runs in
    avian's `SubstepSchedule` on `SolverBody`, as a soft constraint
    (`P = −(1 + cK)⁻¹(h·kp·e + c·v)`): stable at any gain.
  - **Weight fed forward** (`carry_weight`). The drives alone sagged and
    toppled in 1.5-3 s at every gain (kp 10-80 N·m/rad/kg): sized on two
    light bodies, their correction is capped. Each joint now carries the
    gravity moment of the side the ground does not hold. Without it the
    hips sank 607 mm.
  - **Planted feet `Dominance` 1, the ankle's reaction on the ground.**
    The early "sag" was both soles sinking 52 mm into the floor (joints
    solved after contacts, split by inverse mass). Without it, 837 mm.
    The ankle reaction put on the foot instead: NaN in a frame.
- **Measured.** Headless 5 s: hips ≤ 5 mm down, sway ≤ 6 cm, feet ≤ 1 mm,
  knees 2-3° and ankles ~3.4° off target. Ankles carry 38 N·m
  (0.54 N·m/kg), knees 34, hips 27-29. Live on both rigs (BRP,
  `--stand-on-own-feet 180`), ~9 s: hips within 2 mm, sway 25 / 41 mm,
  feet 0.0 mm.
- **Tried and dropped:** sizing each drive on the rigid lump of body each
  side of the joint (torques 1e8 N·m in a frame).
- **Left for 4c:** an undamped ±2 cm sway over the ankles (~2 s period),
  which only a balance controller can steer. The screen still draws the
  animation while the body carries itself.
- `cargo test --release --lib`: 1071 passed, 19 ignored. Clippy: 0
  warnings.

### Open items closed: loaded side step, backward jolt, drawn toes, palms

- **Loaded side step.** A sideways push now takes the young adult's
  loaded side step (the near leg steps out) while it needs at most
  `SIDE_STEP_MAX` (0.4 m), swinging in `SIDE_STEP_SECONDS` (0.2 s); past
  that, the far leg crosses over as before. Its length grows as
  `e^{T/√K}`, so time is what made it lose: at 0.3 s, 0.6 m/s needed a
  0.49 m lunge (146 mm sunk); at 0.2 s, 0.30 m and 44 mm (crossover
  0.35 m, 45). Planned at once it passes `holds`: the earlier unloading
  phase failed because it delayed the lift. Later steps of a hard catch
  are quick side steps too, so 1.5 m/s sideways is now caught (crossover
  and four side steps, 158 mm); 1.6 falls. Live, both rigs: a 0.6 m/s
  side push steps the near foot first, pelvis 32-43 mm down; Front view
  checked.
- **Backward jolt 7.6 → 3.7 mm; every catch ≤ 4.4 mm** (was ≤ 5.1 and
  7.6). Two causes, traced frame by frame (`probe_jolt_trace`):
  - The weight switched feet in one frame at a landing or lift, flipping
    the hip roll and its pivot socket together. It now moves over through
    a 15 rad/s critically damped spring (`WEIGHT_FREQUENCY`); landings sit
    ~6 mm deeper.
  - Backward, the front leg's reach fell 5-11 mm a frame as the body flew
    back from it and met the sink's spring still rising. The sink now
    aims no higher than the ceiling 0.1 s ahead (`CEILING_LEAD`), solved
    again with the pelvis moved on at its velocity. Only backward pushes
    changed. A `sin²` lift landing at zero speed changed nothing;
    finite-difference leads were non-monotonic (10.4 mm at 0.05 s); a
    closing-speed brake reached 6.1.
  Live, both rigs: 1.3 m/s back caught, 104-119 mm down, the pelvis's
  vertical acceleration ≤ 15 m/s².
- **Toes and fingers drawn out of the floor.** Falling or rising, the
  ankle, toes and wrists turn their tips up to the ground
  (`turn_up_clear`); the bodies are left alone. Live, both rigs, the
  lowest drawn toe joint sat at 0.0 mm through a fall. Why the bodies dip
  is now understood: avian solves joints after contacts in every substep,
  splitting by inverse mass, so a 1 kg foot takes ~98% of the
  correction. `Dominance(1)` on grounded feet removed the dip but nailed
  them: feet moved 5-119 mm in a fall instead of 154-712, bodies rested
  propped up; switched by the leg's lift, bodies were flung. Measured
  with the new hand bodies: fingers dip 12-68 mm too.
- **Palms flat in the get-up.** Hands bearing weight lie flat, palm down
  (`getup::palm_flat`): fingers forward on hands and knees, out and back
  propped behind, out and forward under a side-sit. The turn about the
  arm is split between shoulder and forearm; the wrist only bends back
  (≤ 90°). Fingertips were 18-21 cm in; now above the floor in every key
  and through the rise. Front and Left, both rigs, mesh on and a new
  `--camera-look-height` for floor-level views.
- `cargo test --release --lib`: 1066 passed, 18 ignored. Clippy: 0
  warnings.

### Small items, second pass (plan 5.6): four fixed, three measured

- **Fixed: arm joints anchored where the arm is drawn.** Where a bone's
  nearest simulated ancestor isn't its parent (the arm hangs from the
  chest, the collarbone has no body), the joint anchor was computed once
  at spawn. As the collarbone moved, the arm's bodies stood 7-9 cm off
  the drawn arm; a fallen hand rested 7 cm inside a slope.
  `publish_joint_targets` now re-anchors those joints every pinned frame.
  Bodies sit 1.0-1.5 cm from their drawn segments
  (`every_body_stands_on_its_drawn_segment`, < 2.5 cm). Live, no hand or
  elbow went under the floor or slope.
- **Fixed: the shoulder no longer reaches behind the back.** That stale
  anchor was also why a second arm joint held a still arm 6° off. With
  anchors following, a limit-only cone (135° about up-and-back,
  compliant point) limits horizontal extension to AAOS's ~45°. The
  closest arm-to-up-and-back angle across limp falls was 49.8/45.0/44.5/45.0°.
  Without the cone a backward fall reached 42.2°. The turned read-back
  was 11° before the anchor fix and is 5.0° now.
- **Fixed: hands have bodies** (`BodyEnd::Beyond`, 0.45 of the forearm
  past the wrist, mass 0.006 M, PD 7/60; at the default 20 a swing left
  the hand 54° behind). Walking body error: character.glb median 8.8°,
  max 14; puppet_base 10.3°, max 17; no spikes (was 100°). Sixteen bodies.
- **Built: body proportions** (`proportions::winter_factors`,
  `character_gallery --proportions winter [H]`). The thigh, shank, upper
  arm, forearm and hip-to-shoulder height are scaled to Winter's
  fractions. Each moved joint gets a skinning-only stretch along its
  segment, and the hips rise so the ankles stay.
  - Live, standing: puppet_base at 1.80 m and character.glb at 1.93 m,
    every limb within 0.1 %.
  - Feet: within 1 mm of their height before.
  - Walking at 1.3 m/s: planted slip 0.022-0.040 m/s, both with and
    without proportions.
  - Falls and get-ups ran on both rigs.
  - Front and Left: wrists lower on the skeleton, and the skin follows with
    no tear.
  - Winter's widths (hips 0.191 H, shoulders 0.259 H) are body breadths:
    set as joint spacing they put the hip joints 34 cm apart against a real
    ~17. Widths are left as the rig's.
- **Open: backward step clamped at 0.7 m (7.6 mm jolt).**
  - A 0.6 m back cap made 1.3 m/s pushes fall and jolted 11.2 mm.
  - Landing slack: no change.
  - A travel lead: back 7.3 mm, but other directions rose to 8.5.
- **Open: side step.** Unloading the near leg first, the only model that
  matches people, lost catches from 0.8 m/s, and above 1.0 m/s the near
  leg never lifted. `holds` forbids lifting a foot while the other is
  past the 8° validity bound, which is exactly when a side step needs it.
  It needs a stance model past that bound.
- **Open: toes dip 3-6 cm at fall impact**, on flat ground too
  (`probe_fall_floor_penetration`, ignored). Soft contacts against a
  light foot under the body's weight. Every lever cost something:
  - contacts ×3: 20-42 mm; ×10: 13-33, but rest detection broke;
  - 24 substeps: 23-48;
  - feet ×4 mass: 18-25;
  - a 20-30 mm skin: 10-19 mm, but the collapse reached 66 and feet
    floated 13-29 mm at rest.
- **Found: in a get-up the fingers point 18-21 cm into the floor**, wrist
  at y = 0, on both rigs with or without proportions. Not yet looked at.
- Clippy: zero warnings across the crate, examples and tests.
  `cargo test --release --lib`: 1064 passed, 16 ignored.

### Small items (plan 5.5): one fixed, five measured and recorded

- **Fixed: catches at the limit no longer depend on frame times.** The
  stumble balance runs whole 1/60 s ticks, carrying the rest of a frame
  over, and draws that carried time on from the last tick. Over four
  uneven frame patterns, catches at the limit now fall 0 of 12 (was 4).
  At 60 Hz nothing changes (identical jolts and catch limits:
  1.5/1.4/1.4). Live, both rigs, a 1.2 m/s side push was caught with
  planted feet ≤ 1.8 mm. Drawing between the last two ticks was tried
  first: at a landing it mixed one tick's feet with the next's swing (a
  planted foot jumped 21 mm).
- **Backward step clamped at `MAX_STEP`:** 7.6 mm jolt now (9.8 before the
  stance fixes). Located: it lands with the leg at full stretch, and when
  its 11 mm swing lift reaches zero the reach ceiling pulls the pelvis
  down 7 mm in a frame. Earlier slack closing: no change. A rate-limited
  hip roll: no change here, sideways jolts up to 8.3 mm. Left open; a fix
  changes the step and the catch limits.
- **Sideways: loaded side step vs crossover.** A side step whenever the
  loaded leg needs ≤ 0.5-0.7 m sank the pelvis 138 mm at 0.6 m/s
  (crossover 40), 290 at 0.8 (82). A 0.4 m side shuffle lost catches from
  0.8 m/s (was 1.4) and jolted 24-34 mm. Crossover kept; people unload
  the near leg first, which this model lacks.
- **Shoulder behind the back:** a second limit cone was tried. Any second
  joint on the arm, limited or not, compliant anchor or not, held a still
  arm 6° off its target. Reverted, not understood.
- **Hand bodies:** tried, reverted. puppet_base fine (hands 5.8°), but on
  `character.glb` the worst body reached 100°; and the slope's 3-5 cm
  hand dip is there with or without them.
- **Elbow 1.4-2.2° past 150° on impact:** kept; passive flexion exceeds
  AAOS's active range.
- Notes updated: the stumble note and the falling-body note.
- Process slip: one sweep edited `balance.rs` with a Python regex,
  against this repo's rule; the constant was set back with the Edit tool
  and the experiment reverted.

### The walking ragdoll's lag (plan 5.4)

Distilled in
[a pinned ragdoll tracks its targets' velocity](./docs/knowledge/character-animation/ragdoll-and-physics/a-pinned-ragdoll-tracks-its-targets-velocity.md)
and [a fall test samples one chaotic landing](./docs/knowledge/character-animation/ragdoll-and-physics/a-fall-test-samples-one-chaotic-landing.md).

- **Cause:** the PD damped each body's spin toward zero, so a body
  following a moving target trailed it by `2ζ·ω_target/ω` (0.04 s of its
  motion at 8 Hz). A unit test reproduces the predicted 0.199 rad at
  5 rad/s.
- **Fix:** `JointTargetVelocity`, each target's spin measured frame to
  frame, damped toward (`pd_torque_tracking`). Not while falling; zeroed
  over 30 rad/s (a jump) and under 1e-3 (rounding).
- **Live**, walking 1.2 m/s, the worst body per BRP sample: puppet_base
  median 11.5 → 9.5°, p90 16.8 → 12.4°, max 21 → 15°; character.glb
  10.4 → 8.1°, 16.8 → 10.3°, 25 → 13°. Feet 5.7 → 2.5°. Standing still,
  0.0° both before and after.
- **Left:** the upper arms, about 5°. That's the trunk acting on them
  through the shoulder (correlates with the target's acceleration, no
  constant part, unchanged at doubled ceilings), and the 64 Hz step bounds
  the gains that would hold it. Acceleration feedforward, taken frame to
  frame, bought ≤ 0.7° and raised the worst sample to 28°. Dropped.
- **Found on the way:** rounding-level feedforward (≈1e-7 rad/s) changed
  three get-up tests' falls, which then failed on the new landings (4 mm
  creep after rest, no rest within 10 s on a slope, a hand 1 mm over a
  bound). Fixed by the deadband; the tests pin single chaotic landings.

### Getting up from lying on a side (plan 5.3)

- **`getup::Lying::Side { left_down }`**: the chest within 45° of level.
  The route is side-sit → hands and knees → half-kneel → standing, 3.5 s.
  Before, a side-lying body was read as face up or down and the first
  blend rolled it 90° about its own length.
- **The side-sit key**: seated, leaning over the straight arm underneath
  with its hand on the floor beside the hip, legs folded to the other
  side, knees down. Authored by segment directions (`aim`) and solved so
  the seat, hand and both knees meet the floor. The knees and the lean are
  solved together, since the lean moves the hips (solved once each, a knee
  stood 16.4 mm up). The left and right versions mirror within 1e-3. It
  passes every key test: contacts, nothing under the floor, the COM over
  its support, and facing.
- **Live.** 2 of 18 test falls came to rest on a side (forward and out at
  0.8 m/s, chest 53° from face down), and so did the gallery's collapse
  on some runs. A side rise recorded over BRP on `character.glb` went
  through every key to standing, the lowest joint 0.000 m. Static key,
  Front and Left: seated, propped, legs to the far side, on both rigs.
- **Tests:** the rise test covers both sides. A hand that walks more than
  0.2 m between keys may lift 15 cm, like a stepping foot: from the
  side-sit, the propping hand goes 0.4 m forward and arcs 128 mm. The
  get-up test's collapse now lands on its side and passes the side route.

### Uneven ground (plan 5.2): walking up a slope, standing turned on one, rising from one

Distilled in
[sample the ground in the world](./docs/knowledge/character-animation/ik-and-locomotion/sample-the-ground-in-the-world-not-the-pose.md)
and an update to
[foot locks need the body's travel](./docs/knowledge/character-animation/ik-and-locomotion/foot-locks-need-the-bodys-travel.md).

- **Planted feet rose with the body uphill.** The lock dropped the
  vertical part of `Turn::travel`, and the gallery reported only the
  horizontal. Up a 0.2 grade a planted foot climbed with the entity:
  within a stance it changed height by 64 mm (median, BRP), now 13 mm,
  in line with flat ground (17 mm, the heel rising). Horizontal slide is
  unchanged (7–10 mm).
- **The IK sampled the ground in the pose's frame**, right only facing
  −Z on a straight slope. It now maps the toe into the world (the
  character's origin, the turn of the hips' parent against its bind). A
  character turned 90° across the grade stands each foot on the ground
  under it (the old sampling fails the new test by 66 mm).
- **The rise kept clear of a flat floor at the entity's height.** It now
  samples the character's `AnimGround` under each kept-clear joint and
  tucked tip. Live on a 0.2 grade the deepest joint while rising went from
  −47 mm to 0.0; in a test falling uphill, from −359 mm to clear. A first
  version of that test fell downhill and passed with the bug.
- **Gallery:** the drawn plane and the physics floor tilt to
  `--anim-slope`; `--camera-follow` follows the slope's height.
- Not done: the get-up keys are posed against a flat floor; a fallen hand
  (no body) rests up to 4 cm under a slope; toes dip 2–6 cm through the
  floor at a fall's impact (unsimulated toes, flat ground too).

### Pushes and hits while walking (plan 5.1)

Distilled in
[a push while walking moves the next footfalls](./docs/knowledge/character-animation/ik-and-locomotion/a-push-while-walking-moves-the-next-footfalls.md)
and [a pinned root's velocity is not its pace](./docs/knowledge/character-animation/ragdoll-and-physics/a-pinned-roots-velocity-is-not-its-pace.md).

- **`walk_balance::WalkBalance`**: the push's difference from the walk.
  The pendulum runs about the stance foot with a ≤ 1.5 cm sideways ankle;
  each swinging foot aims at the capture point predicted for its footfall,
  settled 300 ms before it (Hof et al. 2010); crossovers are allowed up to
  15 cm, the step at most a leg long. A forward push is a speed surge
  decaying over 1 s instead. 8 tests, including a control without
  footfalls (lost) and a landing-on-the-capture-point check.
- **Wiring.** `AnimFootIk::displaced` moves a foot's animated toe before
  the lock and ground see it; the body moves by the offset as root motion.
  Pushes go to the walking balance once the walk is fully in, and a hit
  is handed over from the standing one (`Balance::take_push`).
- **Catch limits by phase** (`probe_walking_catch_limits`): sideways
  0.25–0.50 m/s just after a footfall, 0.10–0.20 mid-swing (the push waits
  a whole step for the next placement), back 0.35–1.20; forward to a 1.5 m/s
  surge.
- **Live at 1.2 m/s, both rigs.** Each push alone (0.12 sideways,
  0.5 forward, 0.3 back) was caught 3/3. Four in a row felled
  `character.glb` once. A 6 m/s chest hit was caught (path displaced
  0.70 m), 14 m/s fell. Planted feet slid a median 3.9 mm per stance,
  ≤ 4.8 mm around pushes. Front and Left, gizmos: legs and knees normal
  through the hit.
- **A fall's launch.** The limbs already carried the walk, but the pinned
  root body launched at 0.00 m/s in 6 of 10 walking falls (velocity set
  per physics step, zero on a frame's second step). It now leaves at the
  target's per-frame pace: 1.02–1.59 m/s against a 1.13–1.16 walk.
  `a_fall_while_moving_leaves_at_the_bodys_pace` fails at −2.4 m/s with the
  fix disabled. The no-balance hit rule (topple when `Δv·√K` leaves the
  feet) was kept: it reads the feet of whatever pose is playing.
- Not modelled: changing step timing, a trunk lean (hip strategy).

### Body-proportion spike (plan 3b): a segment can be lengthened cleanly

`character.glb`'s left thigh lengthened 10%, right leg as the reference.
Distilled in
[lengthen a segment by moving its joint and scaling only its skinning](./docs/knowledge/character-animation/rig-and-retargeting/lengthen-a-segment-by-its-joint-and-a-skinning-only-scale.md).

- **Moving the knee joint alone** stretches the knee's blend triangles:
  median 1.13×, worst 1.44× straight and 2.71× at a 90° bend
  (`tools/skin_segment_stretch.py`). On screen the knee bandage turns
  into a tall band.
- **Scaling the bone with child compensation**, as planned, can't work in
  Bevy: a parent's non-uniform scale applies after the child's rotation,
  so a bent shin shears. Replaced by a scale on the skinning only: a
  helper joint under the thigh, scaled along +Y, takes the thigh's place
  in `SkinnedMesh::joints`. With the moved knee, the blend stays at median
  1.000×, worst 1.16×. The shin and the upper arm measure the same
  (move-only worst 1.99× and 3.12×; proxy 1.13× and 1.15×).
- **The anim stack needs nothing else:** the foot IK reads every bone's
  live translation each frame, so the longer leg stood with its foot
  planted, knee a little more bent.
- Live, `relaxed_stand` and `getup:half_kneel`, Front and Left: the proxy
  thigh reads as a longer trouser with a normal knee. Gallery flag
  `--proportion-spike move|proxy F`.
- Height-fraction proportions are feasible; not built.

### The over-arched back fixed: poses are bends from the source's bind

Closes the finding below. Distilled in
[a clip's world positions carry its rig's bind shape](./docs/knowledge/character-animation/rig-and-retargeting/a-clips-positions-carry-its-rigs-bind-shape.md).

- **Cause.** Clips were converted into bends from the straight synthetic
  T-pose, so Mixamo's curved bind spine (0.3 / 0.3 / 14.1 / 12.2° back)
  was stored as a bend and applied again on puppet_base's own curved
  bind. `relaxed_stand`'s chest stood 23.6° back, the neck base 79 mm
  behind the bind's.
- **Import.** `tools/dump_bind_positions.py` dumps a skin's bind
  (`assets/anim/idle_bind.positions.ron`), and `import_reference_pose`
  takes it to convert bind-relatively
  (`convert::pose_from_world_positions_against`). `idle_stand` was
  re-imported (worst direction error 0.028°; the old path reproduced the
  committed file exactly first).
- **`relaxed_stand`** was hand-finished, so its spine was rebased
  (`examples/rebase_pose_onto_bind`, `convert::rebase_onto_bind`). Every
  other bone keeps its world orientation, so arms, gaze and legs are as
  they were. The spine now stands +14.6 / −0.4 / −0.1 / −13.5° (bind
  +16.4 / +5.8 / +1.3 / −13.0).
- **Balance.** The arch had held mass back; straightened, the stance stood
  7.5 cm ahead of its ankles (Winter: 4). `stance::balance_over_feet` now
  leans every real-rig stance about its ankles to 4.0 cm, feet flat and
  gaze kept: 2.1° on puppet_base, 0.5° for its T-pose. It's skipped on the
  synthetic rig, whose 7 cm ankle stub has no foot to balance over (it
  leaned 3.9°).
- **Tests that hid it.** The reference test pinned the double-counted
  positions; it now checks the spine bind-relatively (the old pose fails
  by 14.07°). The upright test measured the trunk to the shoulder joints,
  which the rounded collarbones carried forward; measured to the neck,
  the old pose fails by 8.3°.
- **Knock-ons, re-measured.** Catch limits unchanged (1.5 / 1.4 / 1.4).
  Edge catches under uneven frames: 4 of 12 fall with ticking, 9 without.
  The fall probe now measures hips in the pelvis's bind frame (extension
  exactly 30.0°, adduction 30.2°). Sideways-fall elbows overshoot their
  150° stop by 1.4 and 2.2° on impact.
- **Seen live,** both rigs, Left and Front with gizmos and mesh, against
  064a6df: the spine line near straight where it bowed back, head level,
  arms at the sides, symmetric.

1045 tests pass.

### Hip adduction capped the same way; a camera that follows the fall

- A third limit-only joint on each hip's pivot: 120° about the
  direction straight out to the side, so a thigh crosses under the body
  by no more than AAOS's 30°. Sideways falls measured 48.7° before, 30.3°
  after. The fall test fails without it, and the pose-limits test checks
  every shipped pose against all of a hip's cones.
- `character_gallery --camera-follow`: a preset view tracks the hips
  across the floor, so a fall stays in frame for `--shot`. Front view of
  a 1.6 m/s sideways fall, both rigs: the legs land in a small V, never
  splayed.
- **Found, not fixed: the standing spine is over-arched.** Measured
  (`poses::tests::probe_spine_profile`), the segments' forward lean from
  the pelvis up is +16.4 / +1.5 / −12.0 / −23.6°, against the bind's
  +16.4 / +5.8 / +1.3 / −13.0°. `relaxed_stand` stores the idle mocap's
  absolute spine lean (straight synthetic bind) as its bends, but Mixamo's
  own bind spine already leans 0.3 / 14.1 / 12.2° back
  (`assets/models/idle.glb` inverse bind matrices). Relative to its bind,
  the idle bends only −4.1 / +0.8 / +1.6°. So the Mixamo curvature is
  applied again on top of puppet_base's own, about 11–13° of extra arch
  per segment. The import (`import_reference_pose`) should convert
  relative to the source rig's bind.

1045 tests pass.

### Hip abduction capped by a second cone on the same pivot

Closes the open item below. avian's one cone could not fit the hip:
tilted to give 120° of flexion and 30° of extension, it allowed about 68°
out to the side (AAOS 45). Each hip now also has a limit-only
`SphericalJoint` with the same anchors: a 135° cone about the direction
straight across the body. It excludes only the 45° around pointing
straight out, so the allowed region is the two cones' intersection
(`anatomical_side_cone`, `LimitOnly`).

- Falls, four ways: abduction 52° → 45.1°; flexion (to 116°) and
  extension (30.0°) unchanged. The fall test fails at 51.9° with the
  side cone off; the pose-limits test checks every shipped pose against
  it.
- Live A/B against 676dad4, walking at 1.2 m/s on puppet_base, two runs
  each: the worst body-to-target error per sample has median 11.5/12.0°
  before and 11.6/11.9° after. The two joints on one pivot do not fight.
  (That ~12° walking lag predates this work.)
- Seen from above, both rigs: sideways falls leave the legs in a
  moderate V, no splits.
- Distilled into
  [a falling body has hinged knees and elbows and solid flesh](./docs/knowledge/character-animation/ragdoll-and-physics/a-falling-body-is-hinged-and-fleshed.md).

1045 tests pass.

### A falling body: hinged knees and elbows, solid anthropometric flesh

Asked for: falls that move like a body, not a skeleton of thin sticks.
Distilled in
[a falling body has hinged knees and elbows and solid flesh](./docs/knowledge/character-animation/ragdoll-and-physics/a-falling-body-is-hinged-and-fleshed.md).

- **Hinges while falling.** At the fall, each knee's and elbow's ball
  joint is disabled and an avian `RevoluteJoint` takes over: its axis is
  fixed in the femur or humerus, its frames coincide at that instant (no
  snap, roll and sideways tilt frozen), and it is limited to knee −5..140°
  and elbow −5..150° (AAOS 0–135, 0–150). The re-pin swaps back: a hinge
  would lock the forearm roll a pose like the wave holds.
- **Solid flesh while falling.** The body's parts collide with each other
  (jointed neighbours exempt). Limbs are capsules of ANSUR II mean radii;
  the torso is three rounded blocks wider than deep (pelvis 0.34 × 0.22
  reaching 0.10 below the hips joint), so a fallen body lies on its back,
  front or flank.
- **Measured** (`probe_fall_shape`, relaxed stance, 1.5 m/s four ways):
  knees −163..114° → −5.5..99°, sideways 88° → ≤ 1.8°; elbows −81..89° →
  −5.5..146°, sideways 85° → ≤ 5.3°; overlap between unjointed parts
  180 mm → ≤ 10 mm. Pinned by
  `a_limp_fall_bends_knees_and_elbows_as_hinges_and_keeps_its_flesh_apart`,
  which fails with either change disabled.
- **Live, both rigs** (top and left views): forward lands face down,
  back face up, sideways on the back or flank. No knee folds backward or
  sideways and no limb passes through another.
- **Found on the way:**
  - Rest was judged by velocities. A thin forearm on the floor carried
    1.1 rad/s that its hinge cancelled every substep (it turned 1.1° in
    0.5 s), so the body never rested. Rest is now judged by how far each
    body moves over the whole 1 s window.
  - The torso's mass came from its collider, so a block moved its mass
    and centre. avian's `from_shape` inertia on the tilted capsule lost
    its orientation, and the chest spun to 3769 rad/s. Both are now set
    explicitly from the old capsule.
  - The face-up rise swung a hand through the floor and the clearance lift
    hoisted the body 82 mm. Hands now tuck like feet, the elbow bending
    further only.
  - The fall harness was turned after spawning, which left the arms
    28–33° off target. It now spawns facing the drawn way.
- **Cost:** four 4 s headless falls simulate in 1.00 s against 0.81 s,
  about 25% more while falling; standing is unchanged. A T-pose fall rests
  at 5.5 s (was 4.2).
- **Hips and shoulders, the same day.** avian's cone is symmetric, so
  each is tilted to the middle of its range (`anatomical_cone_centre`),
  its twist reference turned with it:
  - **Hip:** centred 45° forward of straight down, 75° half-angle, so
    120° forward and 30° back (AAOS). A forward fall's hip extension went
    43° → 30.0°, and backward and side falls flex the hips 96–115° where
    the bind-centred cone stopped them at 75.
  - **Shoulder:** centred out to the side and slightly forward, 105°
    half-angle, reaching a hanging arm's 60° extension. Walk and run
    swings fit (the run's backswing is 94°).
  - Pinned by the fall test (fails at 43.8° with the tilt off) and the
    pose-limits test, which now measures the tilted frames.
- **Found on the way:**
  - The get-up test's skeleton never followed the fallen body, so the
    rise dragged it 0.4–1.17 m back to the spawn spot, depending on the
    landing. Hung under its character as in a game, it rises 43 mm from
    where it lay.
  - A tuck folding about a nearly straight arm's bend flipped axis frame
    to frame, and the hips jumped 86 mm. Tucks now fold about the joint's
    `Hinge` axis; the test fails at 45.6 mm the old way.
  - Knees and elbows join the rise's floor clearance (a knee went 29 mm
    under).
- **Open:** a tilted cone still allows about 68° of hip abduction at
  neutral flexion (AAOS 45; falls measured up to 52).

1045 tests pass.

### Every test checked for the fixture that faces away

Audit of the tests on plain `puppet_base()`, by swapping the fixtures
for one run (`puppet_base()` returning the turned rig, so both helpers
return the other). 17 of 1044 failed:

- 8 already on the drawn rig: the balance tests, the pendulum ratio,
  `the_relaxed_stand_stands_upright_and_balanced`. Correct.
- 5 comparing against the asset or Bevy's render of the file
  (`gltf_rig`, `retarget`): correct on plain.
- 4 wrong, now fixed:
  - **Joint limits:** the ragdoll's forearm twist stop, −95..85°, had
    been fitted to the wave's −90.5° as the plain fixture reads it. On
    the drawn character it is +90.7°: live, the waving right forearm
    held 3.7° off its target against the stop. Now −85..95°; the waving
    forearm tracks within the body's usual 2.2° on `puppet_base`, 0.1° on
    `character.glb`. `every_pose_the_character_holds_sits_inside_its_joint_limits_on_a_real_rig`
    now runs on the drawn rig.
  - **Look-at (2):** the tests measured the head bone's own −Z, the back
    of the head on the drawn rig. On plain it started at −Z, so they
    passed while measuring the back of the head. The solver was right.
    They now measure the face (the rig's forward carried by the head's
    turn since rest) on the drawn rig, and the reachable look asserts it
    lands (0.000°; 180° on plain) instead of "same side".
  - **Foot on its sole:** `to_axis_angle` read a sign-flipped identity as
    360°. Now the shortest angle.

The 1027 that pass on either fixture don't depend on facing. Distilled in
[the puppet_base fixture note](./docs/knowledge/character-animation/rig-and-retargeting/puppet-base-fixture-faces-away-from-the-rendered-character.md).
1044 tests pass.

### The walk's sway pendulum measured on the character as drawn

Closes the open item below. `phase::COM_OVER_HIPS` (the COM's height
above the ankles over the hips', which sets the walk's sideways pendulum
K) was pinned on plain `puppet_base()`, arms overhead: 1.19, a COM 62% up
the body. As drawn it is 1.092 (about 54%, the usual ~55%). Now 1.09;
`locomotion::real_walk` moved to `puppet_base_as_rendered()` too. Distilled
in
[the walk's step width and sway](./docs/knowledge/character-animation/ik-and-locomotion/walk-step-width-and-sideways-sway.md).

- Headless: sway toward the stance foot 25.5 / 20.5 / 17.7 mm at
  0.7 / 1.2 / 1.6 m/s (was 23 / 18 / 16). The COM stays 4.2 / 8.6 /
  11.2 mm medial of the stance foot's inner border (was 4.3 / 9.2 /
  11.9), so the 13 cm step width stands. Planted feet in double support
  0.01 mm (0.57–0.67 on the old fixture).
- Live A/B against 8d13bbb, same schedule: pelvis sway 48.3 → 51.1 mm
  (0.7 m/s) and 38.8 → 41.4 mm (1.2 m/s) peak to peak; step width and
  speed unchanged; planted feet unchanged over two runs each.
- Cost unchanged: a constant.

1044 tests pass.

### A hard side push caught: the balance tests stood the right way, longer steps, an early join

Asked for: puppet_base catching 1.2 m/s sideways in the gallery. Distilled
in
[a foot may lift only when the other holds the body](./docs/knowledge/character-animation/ik-and-locomotion/a-foot-may-lift-only-when-the-other-holds-the-body.md)
(new),
[a stumble is a capture-point step](./docs/knowledge/character-animation/ik-and-locomotion/a-stumble-is-a-capture-point-step-then-a-join.md)
and
[the puppet_base fixture note](./docs/knowledge/character-animation/rig-and-retargeting/puppet-base-fixture-faces-away-from-the-rendered-character.md).

**The balance tests stood with their arms overhead.** `real_stood` put
`relaxed_stand` on plain `puppet_base()`, which faces away from the drawn
character. The hands were at 1.98 m, the COM 8 cm high (k 0.104 against
the live 0.095), and the soles reached back 0.18 m and forward 0.11
instead of 0.12 and 0.17. So every catch limit was inverted: forward
1.2 m/s "fell" and sideways 1.2 "was caught", the reverse of the live
character. The gallery's stance was right all along. Moved to
`puppet_base_as_rendered()`, pinned by
`the_balance_fixture_stands_as_the_character_is_drawn`.

**`MAX_STEP` 0.6 → 0.7 m.** Needed for 1.2 m/s sideways (0.82 m asked):
at 0.6 it ran away in 26 steps; at 0.65 it was an edge catch with 72 mm
jolts. 0.7 m is ~40% of height, inside young adults' maximal step
(77–79% of height forward, Medell & Alexander's test) and standard
lateral lunges (60%). Alone it caught the push with the pelvis 213 mm
(left) and 271 mm (right) down for about 2 s: after landing, the
pendulum pulled the COM to the middle of the wide stance and the join
waited 2.5–3.0 s to be "caught at rest".

**The early join**, and the three faults it exposed:
- The weight transfers onto a recovery step as it lands, and the
  trailing foot joins as soon as the stepped foot alone holds the COM
  (within the 8° validity lean) and the capture point isn't on the
  trailing side. Joins now start 0.37–0.67 s in.
- Without the hold gate, the join lifted the foot the COM leaned on and
  the validity clamp teleported the COM 130 mm. A second recovery step
  did the same under uneven frames: 1.05 m/s silently discarded.
- The join foot was "furthest displaced"; after a crossover and a side
  step it lifted the loaded foot (0.39 m/s lost, a fall). Now the foot
  the weight is on stays.
- A recovery step landing feet together never ended the stumble (back
  1.4 m/s never settled). Now any feet-together landing does.
- "Capture point inside, with a margin" made a 5 mm stance asymmetry
  delay the right side's join 0.2 s, 78 mm deeper. Now inside or past.

`SINK_LEAD` 0.04 → 0.08 s: on 0.66 m crossovers the late swing jolted
9.7 mm; now ≤ 5.0 mm for every unclamped step. A backward step clamped at
`MAX_STEP` (1.3 m/s) still lands ball-first with a 9.8 mm jolt.

**Catch limits, headless, drawn stance:** forward to 1.5 m/s, sideways
to 1.4, back to 1.4; 1.6 / 1.5 / 1.5 fall. Each gate, disabled alone,
fails a test. Ticking (`MAX_TICK`) now matters only at the limit: catches
there fell in 1 of 4 uneven frame patterns with it, 3 of 4 without
(`a_catch_at_the_limit_mostly_survives_uneven_frames`).

**Live, both rigs, 2026-10-01:** 1.2 m/s sideways each way, and 1.2
forward then 1.0 back, all caught; planted balls within 13 mm; the pelvis
sank 129–138 mm at worst. Forecast cost 42–55 µs, once per stumble.

**Open (closed in the entry above):** `phase::COM_OVER_HIPS` (1.19, the
walk's sway pendulum) is pinned by a test on the same overhead-arms
fixture; on the drawn stance the ratio is about 1.08. The walk sway test
(`locomotion::real_walk`) measures its COM on that fixture too.

1044 tests pass.

### Falls forecast, touchdown cushioned, rising feet tucked, the ragdoll turned with its character

Closes the four open items of the entry below, plus two found live.
Commits feecaa8 and the one after it. Distilled in
[a fall hands the body to physics](./docs/knowledge/character-animation/ragdoll-and-physics/a-fall-hands-the-body-to-physics.md),
[a stumble is a capture-point step](./docs/knowledge/character-animation/ik-and-locomotion/a-stumble-is-a-capture-point-step-then-a-join.md),
[getting up goes through key poses](./docs/knowledge/character-animation/ragdoll-and-physics/getting-up-is-a-timed-blend-then-a-re-pin.md)
and the new
[a pose delta's world is the character's frame](./docs/knowledge/character-animation/rig-and-retargeting/a-pose-deltas-world-is-the-characters-frame.md).

**Falls are forecast** (`Balance::catches_ahead`). At the first recovery
step a copy of the balance runs 3 s ahead at 1/60 s. It falls if the copy
loses over 0.1 m/s to the validity bound. This replaces `MAX_CATCH`:
1.2 m/s sideways is now caught in four crossovers on the test stance, and
every falling push is known within 0.2 s. Cost 43 µs worst frame, once per
stumble (`probe_forecast_cost`).

**Touchdown is cushioned** (`balance::Sink`). The drop aims 0.04 s ahead
on the swing arc, the swinging leg gets a fading 5 cm slack, and a
critically damped 15 rad/s spring, solved exactly, follows. The pelvis
jolt at a full-reach landing went from 12.8 mm to ≤ 4.8 mm. With the
spring alone the jolt stayed at 12.8 mm, because the need itself jumped
15 mm a frame. Softening the swing leg snapped the landing (18.7 mm).
Aiming at the landing sank the pelvis 128 mm. An explicitly integrated
spring blew up at ω 1000.

**Rising feet are tucked** (`tuck_foot`). From lying to hands and knees,
`LeftToeBase` dipped and the clearance lift raised the whole body up to
209 mm. Now a moving foot's knee bends until it clears. Headless the
worst overshoot is 48 mm; 88 mm with the tuck disabled, which fails the
test. Planted feet are not tucked (`rise_moving`), and the keys are
chained so shared contacts stay put. Tucking planted feet had made the
lift jump 15 mm.

**A hit topples a character with no balance** when the capture point
`Δv·√K` leaves the support under both feet.

**The ragdoll turned with its character.** Pose ↔ body conversions ran
on a rig rooted at the live facing, reading every delta about a scene
axis. They were right only facing the spawn direction. The rise's 86°
turn swung the lying body 534 mm in a frame, and the standing bodies held
their arms in a T. Now `character_frame` converts in the bind-rooted rig
and applies the turn at the boundary. The old turning test used the rest
pose and could not fail; now it uses `relaxed_stand` (fails 8.8° with
the fix disabled), and a new read-back test fails 90° without it. Live,
both rigs: bodies within 0.1–2.8° of their targets once standing, arms
hanging 82° below horizontal.

**Balance ticks at most 1/60 s** (`MAX_TICK`). Stepped a whole frame at a
time, live frames of 4–52 ms landed each foot up to a frame late, and the
1.2 m/s side catch asked ever-longer steps (0.74 → 1.39 m) and fell.
`a_catch_does_not_depend_on_frame_times` fails without it.

**Measured live, 2026-10-01:**
- Planted balls during sideways catches of 0.8–1.2 m/s, after a forward
  stumble: 12–28 mm at worst, a single frame at a landing. The 8–22 cm
  slides seen earlier did not reproduce at feecaa8 or after it (A/B in
  a worktree).
- The catch limit depends on the stance. The gallery's `puppet_base`
  (k 0.095 s²) falls on 1.2 m/s sideways; `character.glb` (0.104 s²) and
  the test stance catch it.

### Getting up through key poses; hits topple; sideways stumbles really caught

Follow-ups to H2/H3 in [WINTER_MOTION_PLAN.md](./WINTER_MOTION_PLAN.md).
Distilled in
[getting up goes through key poses](./docs/knowledge/character-animation/ragdoll-and-physics/getting-up-is-a-timed-blend-then-a-re-pin.md),
[a stumble is a capture-point step](./docs/knowledge/character-animation/ik-and-locomotion/a-stumble-is-a-capture-point-step-then-a-join.md)
and
[a fall hands the body to physics](./docs/knowledge/character-animation/ragdoll-and-physics/a-fall-hands-the-body-to-physics.md).

**Get-up keys** (`getup.rs`):
- Face up: sit → squat. This is VanSant's (1988) most common adult
  pattern.
- Face down: hands and knees → half-kneel.
- Each key is sagittal angles about the rig's measured `left`, solved so
  its contacts meet the floor together.
- Tests (both fixtures): contacts ≤ 15 mm from the floor, nothing under
  it; the centre of mass inside the contacts; facing checked signed; the
  symmetric keys mirror.
- Seen statically via `--anim-pose getup:<key>`, Front and Left, gizmos.
- Sitting had to be reclined and propped: with the thighs level no shin
  reached the floor, and upright the hands hung 27 cm short.

**The rise:**
- It reads face up or down from the chest body against its standing
  target, and asks for the turn that faces it along its body; the gallery
  applies that to its `Facing`.
- It blends per bone in world space: local blending flung an arm out
  while sitting up.
- It rises within 1 mm of where it lay. Before, the gallery's own
  locomotion position slid it 0.45 m back to where it fell from
  (`follow_the_fallen_body`).

**Hits topple.** A `RagdollHit` pushes the `Balance` by the struck body's
mass share (thorax 21.6%): a chest blow steps from about 3 m/s and falls
from about 5.5.

**Sideways, the step catches:**
- The planner stands the COP at the stance foot's nearest point (the
  middle asked ~2× too long a step).
- The leg that just stepped never steps again (re-stepping walked the
  foot away).
- The step is judged and planned on the whole push (a frame's delay made
  it 17% longer).
- `MAX_STEP` is 0.6 m.
- The far leg crosses over when that step is shorter: 91 mm of pelvis
  sink against 266 for the side lunge. Young adults mostly side-step
  instead; this is the model's choice.
- Caught on `puppet_base`: forward to 1.0 m/s, sideways to 1.0, back to
  1.2, nothing lost to the bound. First steps asking over `MAX_CATCH`
  (0.77 m) fall.

**Falls carry the push** (`Ragdoll::fall_moving`). Judged at the push's
start, all eight test falls had dropped identically onto the back. Now
forward lands face down and backward face up.

**Measured live:**
- 0.8 m/s left: a crossover, then a join; planted balls ≤ 1.7 mm on both
  rigs.
- Rise: within 1 mm of where it lay; feet ≥ 0 mm; standing at full height.

**Open:**
- A 12.8 mm touchdown V in the pelvis after a full-reach step.
- A leg once caught lifted high between lying and hands-and-knees.
- Sideways `MAX_CATCH` is conservative (1.2 m/s falls, though four
  crossovers would catch it).

1036 tests pass.

### Getting up: a timed blend from the fallen body back to the animation

H3, the last item of the re-scoped step 4 in
[WINTER_MOTION_PLAN.md](./WINTER_MOTION_PLAN.md). Distilled in
[getting up is a timed blend, then a re-pin](./docs/knowledge/character-animation/ragdoll-and-physics/getting-up-is-a-timed-blend-then-a-re-pin.md).

`Ragdoll::get_up(delay, duration)` starts once a fall is at rest; the
gallery waits 1 s and rises over 1.5 s.
- While rising, the bodies stay asleep. The drawn skeleton blends from them
  to the animation: rotations per bone, and the hips from the body to the
  animated hips.
- It's lifted so no toe, foot, hand or head joint goes below the ground
  (the entity's height). Without that, a ball went 17 cm under the floor.
- The foot locks are kept free, since the entity moved during the fall.
- The frame after the blend completes, every body is set onto its drawn
  bone (`Ragdoll::body_offsets`, recorded at spawn), still and awake. The
  root is pinned again, the fall's joint damping removed, and the fall
  cleared.

**Measured.**
- Headless, `a_fallen_ragdoll_gets_up_and_is_pinned_again`: no movement
  through the delay (< 1 mm), no drawn-hips jump over 3 cm a frame, and
  it ends within 1 cm of where it stood. Every body is within 2 cm and 3°
  of its bone at once and 2 s later. Without setting the bodies back it
  fails (Spine 7 cm, 22° off).
- Live on both rigs (`--push-schedule 3:1.5:0,11:0.6:0`): fall, rest,
  rise, and standing at full height (hips 0.94 / 1.12 m, balls 15 / 5 mm).
  Feet ≥ 0 mm through the rise, hips climbing 12–20 mm a frame. The push
  at 11 s is stumbled normally.

**Seen.** Mesh on, Left view, four runs: mid-rise half up with no foot
below the floor and nothing torn, then standing with feet flat. The
in-between pose is a blend, not a get-up (arched backward from kneeling);
authored get-up clips are the real answer. 1031 tests pass.

### A fall: the body goes to physics when no step can catch it

H2 of the re-scoped step 4 in [WINTER_MOTION_PLAN.md](./WINTER_MOTION_PLAN.md).
Distilled in
[a fall hands the body to physics](./docs/knowledge/character-animation/ragdoll-and-physics/a-fall-hands-the-body-to-physics.md)
and
[normalize what you read back from your own output](./docs/knowledge/engineering-practice/debugging/normalize-what-you-read-back-from-your-own-output.md).

**The balance could not fall.** Its 8° validity clamp held the COM at the
bound with its velocity zeroed. A 2 m/s shove was "caught" by one 0.4 m
step, with 1.98 m/s simply discarded. `Balance::falls` now fires when the
recovery step, planned from the whole push, asks for more than
`MAX_CATCH` (0.8 m). That value is the model's own verdict with the clamp
lifted: forward 0.8 m/s caught and 1.0 falls, back 1.0 caught and 1.2
falls. Sideways, every stumble was the clamp's catch, the H1 0.7 m/s one
included. The stumble note is corrected.

**The ragdoll falls.** `Ragdoll::fall(tone, damping)`:
- Release: the pinned (kinematic) root becomes dynamic with the velocity
  it was following the hips at.
- Physics: gravity in full, pose strength `tone`, and avian
  `JointDamping` on every joint.
- Display: the read-back shows only the simulation, with the hips joint
  placed on the hips body.
- Entity: the character entity follows the body across the ground.

The gallery gets a physics floor, sole-block feet, 12 substeps, and
`F` / `--fall-at-frame` / `--fall-damping`.

**What it took:**
- The read-back's hips rotation drifted off unit length through its own
  feedback (`inverse()` of the Transform it wrote last frame). Lying down
  it reached norm 1.03: skeleton scaled 6%, drawn 8–21° off its bodies.
  Now normalized, and drawn within 0.1°.
- Tone as a pose controller (0.15) kept a lying forearm pushing at
  0.57 m/s. Tone is now joint damping. The sweep over three fall
  directions at 12 substeps: only 1–3/s rested every fall; 3/s is used.
- A body on the floor didn't stop. Resting jitter sat at avian's 0.15
  rad/s sleep bound, so it never slept and crept 9 mm/s (`puppet_base`)
  and 7 mm/s (`character.glb`, at avian's 6 substeps). Three layers stop
  it: 12 substeps in the gallery, a looser
  `FALLEN_SLEEP`, and `rest_fallen_ragdolls`, which puts a body slow for
  1 s to sleep and marks `Fall::at_rest` (H3's trigger).
- Dead ends, each measured: joint damping as the creep's engine (still
  creeps at 0), joint limits (still creeps without them), a stray
  collider, and the entity-follow (A/B: the outcome varied by landing,
  not by follow).

**Measured.**
- Headless (`a_released_ragdoll_falls_with_its_momentum_and_is_drawn_where_it_lies`):
  carried at 1 m/s and let go, the hips coast 6+ cm in 0.1 s. Lying at
  hips < 0.35 m, the body is asleep within 5 s and moves 0 mm after.
  Every bone is drawn within 1° of its body, the hips on their body
  within 1 cm, and the norm stays within 1e-4 of 1 every frame.
- Forced rest (`a_fallen_body_that_never_sleeps_by_itself_is_put_to_rest`):
  fails without `SleepBody`.
- Live, five falls (1.5 m/s forward and back, 1.2 sideways, both rigs):
  all lie head-down (neck ~0.13 m). Four were at rest 2.5–3.5 s after
  the push; `puppet_base` sideways rolled slowly for ~6 s first.
- Caught stumbles with the ragdoll and floor present: planted balls
  ≤ 0.1 mm (`puppet_base`).

**Seen.** A fallen `puppet_base`, Front and Left:
- `--gizmos on --show-real-mesh off`: the body lies on the floor, and the
  drawn skeleton lies along the ragdoll bodies.
- Mesh on: curled on its side, intact, nothing sunk or stretched.

**Cost.** One `puppet_base` ragdoll frame, headless
(`probe_ragdoll_substep_cost`): 0.66–0.77 ms p50 at 6 substeps and
0.94–0.96 ms at 12. The kinematic stack is unchanged. 1030 tests pass.

### A stumble: one step to the capture point, then the trailing foot joins

H1 of the re-scoped step 4 in [WINTER_MOTION_PLAN.md](./WINTER_MOTION_PLAN.md).
Distilled in
[a stumble is a capture-point step, then a join](./docs/knowledge/character-animation/ik-and-locomotion/a-stumble-is-a-capture-point-step-then-a-join.md)
and
[a speed contact test is fooled by a lagging sprung leg](./docs/knowledge/character-animation/ik-and-locomotion/a-speed-contact-test-is-fooled-by-a-lagging-sprung-leg.md).

When the capture point leaves the feet, `balance::Balance` steps. It
plans the landing where the capture point will be (`p + (cp − p)·e^{T/√K}`,
≤ 0.4 m), moves the weight toward that foot, and brings the other foot
alongside once the capture point is inside the stepped foot. The gallery
moves the character by the distance stepped (`GalleryStride::stepped`,
root motion), so the feet end side by side as they stood.

What it took, each found by a headless replay of the gallery's loop
(`a_stumble_steps_cleanly_on_the_real_rig`) or a live BRP capture:
- `stance::move_pelvis_and_feet`, a single solve for the pelvis and both
  feet. Its drop is the root's *rise*, so the leg asking most is the
  **least**. Folding with `max` left the rear foot lifting 37 mm.
- The swinging foot keeps holding the pelvis (load 0.1). Let go, the
  pelvis sprang up 132 mm.
- The join starts when the capture point is inside the stepped foot.
  Waiting until the COM was over it sank the pelvis 14 cm (21 cm
  sideways) under a rear leg reaching 0.4 m.
- A trailing foot rolls onto its toes about the sole's tip (`MAX_HEEL_RISE`
  0.6 rad, after a flat 4 cm drop). Forward, the pelvis sinks 46 mm
  instead of 69.
- `AnimFootIk::planted`: the balance tells the foot IK which feet are
  down. Live, the planted foot's lock let go four times on the sprung
  toe's speed (0.49–0.78 m/s) and slid 16–21 mm.
- The landing hint eases in over the swing's first quarter and fades out
  over `LAND_HOLD`. Before, it made a 30 mm pop at lift-off and a 20 mm
  drop at the end of the hold.

**Measured.** Headless on `puppet_base`, the pelvis sinks 46 mm (0.6 m/s
forward), 115 mm (0.7 m/s left: a 0.4 m side step, feet 0.63 m apart) and
44 mm (0.8 m/s back; 0.6 m/s back needs no step, since the heel reaches
0.18 m behind the COM). The same two steps are planned at frame times
cycling 5–50 ms. Live (`--push-schedule 3:0.6:0,8:0:0.7,13:-0.8:0`), every
planted ball stays within ≤ 1.0 mm on both rigs. Before these fixes the
figure was 13–70 mm. `character.glb` catches the forward push without a
step.

**Seen.** Front and Left, `--gizmos on --show-real-mesh off`:
- Forward: the front foot is flat and the rear heel is raised on its
  toes; both knees bend forward; the pelvis is between the feet.
- Sideways: a wide stance with both soles on the floor and the pelvis
  lowered between them.

**Cost.** `anim_bench` (100 × 600): 4.2 µs per character, against 4.3–4.4
at HEAD (5f0a1f2) built in a worktree, so no change. The balance runs
only in the gallery. The growth from the 2.0 µs last recorded came from
the commits between, which didn't re-run the bench. 1027 tests pass.

### The ragdoll, unpinned: it buckles at 0.5 s; its feet now have soles

Step 4 of [WINTER_MOTION_PLAN.md](./WINTER_MOTION_PLAN.md) (revised).
Distilled in [an unpinned ragdoll needs soles and weight-bearing control](./docs/knowledge/character-animation/ragdoll-and-physics/an-unpinned-ragdoll-needs-soles-and-weight-bearing-control.md).
Step 3a was withdrawn first: Winter §11.1 shows the hip/knee *moment*
split varying at near-identical *angles*, so it supports no knee-angle
style. The KB note that claimed otherwise is corrected.

**Spike.** Headless: `puppet_base` unpinned, full gravity, full-strength
PD toward its bind pose, on a friction-1 floor.
- It buckles at 0.5 s (hips −10 cm), Winter §8.1's prediction.
- Its capsule feet roll and skate 0.8 m in 2.5 s.
- The PD holds weight only by zeroing gravity
  (`GravityScale = 1 − strength`); the probe had to restore it every step.

**Feet.** `RagdollSpawnConfig::feet` (`sole_blocks(rig)`) gives the foot
bodies flat blocks built from the walk's `foot::Sole`: heel to toe tip,
Winter's 0.362 breadth, 3 cm thick, friction 1, in the ankle bone's frame.
A foot dropped onto the floor turns 0.10° and slides 0.16 mm, where the
capsule turns 26.6° and slides 25.1 mm (`a_foot_stands_flat_on_its_sole`).
In the whole ragdoll the feet stop rolling (2° instead of 11°) but still
crawl ~0.85 m as the legs buckle. That is the controller, next. 1022 tests
pass.


### A pushed standing character sways over its feet and recovers

Step 2 of [WINTER_MOTION_PLAN.md](./WINTER_MOTION_PLAN.md). Distilled in
[push recovery as Winter's pendulum](./docs/knowledge/character-animation/ik-and-locomotion/push-recovery-is-winters-pendulum.md).

**The model.** `balance::Balance` is Winter's Eq. 11.3,
`COM̈ = (COM − COP)/K`, with the COP law `COP = COM + s·x + b·ẋ`: his
in-phase stiffness plus reactive damping. The COP is clamped to the feet
(`Support`, from the `Sole` contacts, bounded to his 8° validity).

**Posing it.** `Balance::apply` poses the offset over exactly-held feet:
an ankle lean front to back, the load/unload shift and roll side to side.
The pelvis moves `COM_PER_PELVIS = (0.863, 0.810)` further than the COM
should; that ratio is measured with `centre_of_mass` and pinned by a test
(the COM lands within 2 mm).

**The gallery** takes `--push-schedule T:FORWARD:LEFT,...` and runs the
balance on the standing side of the blend.

**Two designs that failed first:**
- **A one-frame push.** A 0.8 m/s push in one frame let the foot locks go:
  the unsprung pelvis ran ahead of the 0.015 s leg springs, and the feet
  re-planted 23 mm away. A push now takes 0.1 s.
- **Holding an unabsorbable push at the support's edge.** The clamped COP
  sat exactly under the COM there, an equilibrium, so the body hung
  forever, and the clamp's jump moved the feet 32 mm. It is now flagged
  (`needs_step`), and the COP is unclamped as a stand-in for the step.

| Live push | `puppet_base` peak pelvis | `character.glb` |
|---|---|---|
| 0.25 m/s forward | +22.9 mm | +28.4 mm |
| 0.2 m/s left | +21.8 mm | +26.7 mm |
| 0.3 m/s back | −31.8 mm | −75.0 mm (needs a step) |
| 0.8 m/s forward | +116.6 mm (needs a step) | +146.1 mm (needs a step) |

Every push returns. Planted balls move ≤ 1.3 mm and ankles ≤ 4.9 mm over
the whole capture. Left view: a lean from the ankles, trunk in line, feet
flat. Front: symmetric and planted. Seven balance tests, 1021 in all.


### A stop sets its last foot down; raised ground was a synthetic-rig artifact

**The stop's glide.** The last step fades a whole swing into standing: the
foot leaves ~0.66 m behind its spot and arrives ~0.4 s later. The target
brought the foot down on the way (74 mm short at 5 mm into the floor), and
live the foot IK slid it in along the floor.

A lift on the target alone did not carry to the screen. The legs' 0.015 s
springs lag a 2–3 m/s foot by over 100 mm: when the target reached its
spot, the rendered foot was still 38 mm behind at 6 mm up and crept the
rest in, measured headless by running the stop through `DhoState`.

So the landing is judged on the rendered foot:

- `Transition::landing` publishes the swinging foot and its standing spot
  (`AnimFootIk::landing`), through the fade and a 0.25 s hold after it.
- The foot IK holds that toe up by `landing_lift` of its own distance from
  the spot: 3 cm, eased out over the last 12 cm as `x(2 − x)`. A smoothstep
  first let it creep ~18 mm within 3 mm of the floor.
- The target keeps the same lift.

| Live, 1.2 m/s, last ball's travel within 2 mm of the floor | before | after |
|---|---|---|
| `puppet_base` | 9.4 mm | 2.9 mm |
| `character.glb` | 14.9 mm | 2.7 mm |

The foot comes down onto its spot (28.6 mm to go at 19 mm up, 6.9 mm at
5.2 mm), settles within 0.2 mm, and does not pop when the hold ends. Three
new tests; the two that exercise a lift fail with it disabled.

**The heel on raised ground was the synthetic rig.** Its leg joints are
shifted by one: `LeftUpLeg` is the knee, and the IK's "shin" is a 0.07 m
ankle stub. Folding 25 cm, that stub flipped 180° and pitched the foot,
heel ~13 cm under the plane. A new real-rig fixture (`app_with_real_rig`,
`puppet_base` driven by the plugin as the game does) shows heel, ball and
tip all exactly on the plane at 0 / 0.10 / 0.25 m. The synthetic test now
checks the ball and says why. 1014 tests pass.


### The foot IK plants the walk's own sole: feet stand at the asset's height

**The mismatch.** The foot IK planted the toe at "the toe joint's height
above the lower of toe joint and tip". Those are both joints, and on
`puppet_base` they are level, 15.2 mm above the asset's bind floor. So the
offset was 0 and the IK put the joint itself on the floor, 15 mm lower than
the walk's `Sole`. A flat foot's rest value had once been right (+0.0152);
it was replaced to follow the foot's roll, and lost the sole's thickness.

**The fix.** `toe_contact_offset` now measures the toe joint above the
lowest `Sole` contact in the animated pose, which gives both the thickness
and the roll.

`Sole::of` also stops assuming a rig binds its foot above `y = 0`. The
synthetic rig's ankle sits on it with its toe 2 cm below, so its sole is
the plane of its lowest joint. Real assets are unchanged by that.

**Standing, live (BRP):**

| | before | after | asset bind |
|---|---|---|---|
| `puppet_base` ankle / ball | 0.0788 / 0.0016 m | 0.0865 / 0.0152 | 0.0865 / 0.0152 |
| `character.glb` ankle / ball | 0.1183 / 0.0013 m | 0.1216 / 0.0049 | 0.1216 / 0.0049 |

**Walking, 1.2 m/s, steady planted slide:** `puppet_base` 3.4 / 4.1 mm
(was 3.4 / 3.5); `character.glb` 5.8 / 6.4 mm (was 5.1 / 3.7). Start slide
2.3 / 3.1 mm. The stop window now reads 41 mm, but the trace shows the
last step's pre-existing glide, 4 cm of it now inside the metric's "planted"
height band. It is not a planted foot.

**Tests.** Three plugin tests measured the "sole" as the joints; they now
use `Sole`:
- The contact-offset test pins a flat foot at the asset's 15.2 mm (the old
  code gives 0) and as the cycle's lowest.
- Measured over the whole sole, the raised-ground fixture showed a heel
  sinking ~13 cm where the floor rises past the legs' reach. That was
  always so and stays open; the test checks the ball the IK plants.

Left view, mesh on, standing: both rigs' soles on the floor. 1010 tests
pass.


### `character.glb` walks: the live rig geometry was in centimetres and at the wrong hips height

Distilled in [the live rig geometry must match the rendered rig](./docs/knowledge/character-animation/rig-and-retargeting/live-rig-geometry-must-match-the-rendered-rig.md).

**Two causes, both in how `solve_foot_ik` built the geometry that the
gait, foot IK and root motion solve on:**

1. **Units.** It used raw bone translations. Under the Mixamo rig's
   Blender `Armature` node (scale 0.01) those are centimetres: a 46 m
   thigh. They now go through `HumanoidSkeleton::bone_translation_scale`.
2. **Hips height.** The hips took the synthetic 0.94 m, while the renderer
   puts them at the rig's own rest (`character.glb` 1.126 m, `puppet_base`
   0.949 m, 44 mm off in all). They now use
   `HumanoidSkeleton::hips_rest_offset`.

After (1) alone the character walked at speed but crouched, feet 0.186 m
in the air: exactly the hips error. The construction is now one function,
`plugin::live_rig_geometry`. It is checked against the asset's bind, joint
by joint, for `puppet_base` and a centimetre copy of it; sabotaged, the
test fails by 11.5 m and by 44 mm.

**A third bug the fix exposed, on `puppet_base`.** `stance_on_rig` bent the
knees without lowering the hips, so the stance's feet floated 6.9 mm. The
old geometry's error had read as slack; without it, the foot IK pitched
each foot 4.5° toe-down to reach the floor. The stance now lowers the hips
by what the ankles rise, and the feet stand at the asset's bind heights
(new test; fails with the drop removed).

| Live, 1.2 m/s asked | `character.glb` before | after | `puppet_base` after |
|---|---|---|---|
| speed | 0.27 m/s | 1.18 | 1.18 |
| hips vs travel | 63° off | facing it | facing it |
| steady planted slide | ~200 mm | 5.1 / 3.7 mm | 3.4 / 3.5 mm |
| step width | — | 148 mm | 129 mm |
| pelvis roll / chest correlation | — | ±2.6° / −0.97 | ±2.7° / −0.97 |

Standing, `character.glb`'s ankle is at 0.1183 m (bind 0.1216). Front and
Left gizmo views: it walks upright toward its travel, knees forward.
`puppet_base`'s start slide is 2.5 / 2.9 mm, unchanged; its stop window
reads 3.1 / 5.5 mm, where it used to read 8–20.

**Still open (predates this):** the foot IK plants the toe *joint* on the
floor on `puppet_base` (its toe and tip are level), 15 mm below where the
walk's `Sole` puts the contact. Standing, the ball renders at 0.0016 m
against a bind 0.0152 m; it was 0.0102 m before these fixes. 1010 tests
pass.


### The frontal and transverse walk, checked live at three speeds — and a second rig that cannot walk

Step 1.5 of [WINTER_MOTION_PLAN.md](./WINTER_MOTION_PLAN.md), closing step
1: the walk's side-to-side and turning motion.

**`puppet_base`, live, from a standing start** (BRP capture script with a
speed schedule, A/B against the pre-plan build in a worktree):

| | 0.7 m/s | 1.2 m/s | 1.6 m/s |
|---|---|---|---|
| step width (pre-plan: ~230 mm) | 130 mm | 128 mm | 130 mm |
| pelvis sway, peak to peak | 47 mm | 43 mm | 34 mm |
| pelvis roll in single support, swing side low | ±2.8° | ±2.7° | ±2.6° |
| pelvis turn / chest turn | ±4.0° / ±4.8° | ±3.9° / ±4.7° | ±3.9° / ±4.7° |
| pelvis–chest correlation | −0.98 | −0.97 | −0.97 |
| trunk lean, peak to peak (pre-plan ~5.9°) | 2.3° | 2.2° | 2.2° |
| steady planted slide (pre-plan) | 5.5 mm (5.7) | 3.3 mm | 7.1 mm (5.0) |

The sway shrinks with speed, as the pendulum says it should. The fast walk
slides ~2 mm more than before: the target pose holds the feet exactly, so
this is the rendered pose following a busier pelvis. Small, recorded, not
chased. The temporary probe in `locomotion.rs` is gone. 1008 tests pass.

**`character.glb` cannot walk, and never could.** On the pre-plan build
(A/B, same schedule, `--character-model models/character.glb`):
- it travels 0.27 m/s where 1.2 was asked;
- its hips face 63° off the travel;
- its planted feet slide ~200 mm.

On the current build it walks sideways (hips 83° off) and, in the Front
view, its legs sink below the floor. Standing looks right. The rig was
never checked walking; every walk measurement in this log is
`puppet_base`'s. Cause not investigated yet.


### The walking pelvis turns with the stepping leg, and the chest against it

Step 1.4 of [WINTER_MOTION_PLAN.md](./WINTER_MOTION_PLAN.md). Distilled in
[the walking pelvis's turn](./docs/knowledge/character-animation/ik-and-locomotion/walking-pelvic-turn-and-chest-counter-twist.md)
and [a gait-timed motion cannot ride a weighty spring](./docs/knowledge/character-animation/ik-and-locomotion/a-gait-timed-motion-cannot-ride-a-weighty-spring.md).

**Measured first** (headless, `puppet_base`, 1.2 m/s):
- The pelvis did not turn about the vertical.
- The authored chest twist (`Spine1`, ±5.1°) peaked at midstance.
- The arms peak at 0.06 of the stride.

**Winter could only time it.** Integrating power over moment, as for the
roll, would give the pelvis against a stance femur that rotates in the
world, and the transverse moments are small. H1-T says the turn reverses
at heel contact. The size, ±4° (`phase::PELVIC_ROTATION`), is Perry's,
labelled outside Winter.

**Built:**
- `pelvic_rotation_at` is composed with the roll into one turn in
  `stance::move_pelvis_over_feet`: about the loaded hip, `Spine` turned
  back, feet exact.
- The `Spine1` twist is retimed a quarter cycle to peak at the heel
  contacts, against the pelvis.

**On screen the chest was still wrong: ±1.8°, uncorrelated with the pelvis
(+0.03).** Its 0.16 s spring passes 0.36 of a 0.9 Hz motion, 107° late,
which is the arms' old problem. The new
`the_rendered_chest_turns_against_the_pelvis_on_time` runs the springs and
failed at 3.67° of the target's 10.04° peak to peak. `Spine1` is now on the
arms' 0.03 s; `Spine2` keeps 0.16 s for weight.

| Live, 1.2 m/s (BRP) | before the spring change | after |
|---|---|---|
| pelvis turn | ±3.9° (+3.3° / −3.3° at left / right foot-down) | same |
| chest turn | −2.2..+1.5° | −5.1..+4.3° (−4.9° / +4.1° at foot-down) |
| pelvis–chest correlation | +0.03 | −0.97 |

Start slide 2.1 / 2.5 mm; root weave 15 mm; both unchanged. The test
fails on the old twist timing ("chest extremes at 0.25 / 0.75").

Top view: the hip line turns with the stepping leg and the shoulders the
other way. Front and Left: trunk upright, stride unchanged.

`anim_bench` ~4.3 µs per character per frame: the turn rides the existing
re-solve. 1008 tests pass.


### The walking pelvis drops on its swing side, Winter's way

Step 1.3 of [WINTER_MOTION_PLAN.md](./WINTER_MOTION_PLAN.md). Distilled in
[the walking pelvis's roll](./docs/knowledge/character-animation/ik-and-locomotion/walking-pelvic-obliquity-from-hip-abductor-power.md).

**The curve.** Winter has no pelvic angles, only frontal hip power and
moment (Figs 7.4–7.5). Power over moment is angular velocity, and
integrated through stance it gives the roll:

- H1-F: the swing side drops to 3.9° at 17 % of the stride.
- H2-F and H3-F: it is lifted back, to 0.74° low at its own heel contact.

The curve is stored as odd harmonics 1, 3 and 5 (`pelvic_obliquity_at`),
within 0.37° of the integration. It replaces the authored 0.05 rad Hips
oscillator.

**The move.** One pass, `stance::move_pelvis_over_feet`, now does both the
pendulum sway and the roll. The roll pivots on the load-weighted hip socket,
because rolling about the pelvis's centre would lift the stance socket past
this rig's leg length. The Spine counter-rolls, and each leg is re-solved
once. The target test pins the peak at 0.14–0.22 of the stride and
3.0–4.5°; a sign-flip sabotage fails it.

**A spring lag the target could not show.** `Bone::Spine` moved from 0.16 s
to 0.015 s, the hips' spring. Slow, it delivered the counter-roll late, and
the rendered trunk rolled with the pelvis. The new
`the_rendered_trunk_stays_upright_while_the_pelvis_rolls` runs the springs:
8.52° peak to peak with the old spring, under 1.5° now. `Spine1` and
`Spine2` keep 0.16 s.

| Measured | before | after |
|---|---|---|
| planted soles under the locomotion layer (headless) | 45–48 mm | ≤ 0.6 mm |
| roll in left single support, live | −0.65° (stance side low) | +2.7° (swing side low), peak 3.55° |
| root sideways weave, steady walk | 101 mm | 14–17 mm |
| trunk lateral lean, peak to peak | 5.9° | 1.5° |
| start, worst planted slide | 1.6 / 2.5 mm | 1.6 / 2.0 mm |

Live figures come from BRP captures, A/B against the pre-1.2 build.

The root weave in the previous entry, logged as predating the step-width
work, was this authored roll: it moved the rendered contacts against the
hips, and root motion followed. The last step's glide is unchanged: the
"stop slide" metric reads 8–19 mm run to run.

Front and Left gizmo views: the hip line is lower on the swinging side in
single support, the shoulders level within a pixel, and the stride
unchanged.

**Cost.** `RigGeometry::forward` accumulated the whole skeleton's bind
rotations on every call (120 ns); it now walks one chain (30 ns), which
every caller gains. `anim_bench`, per character per frame: 3.8 µs after
1.2, 4.8 µs with the roll as a second pass, 4.2 µs merged and with
`forward` fixed. The baseline is 2.2 µs. 1006 tests pass.


### A human step width, and a walk that sways over its stance feet

Steps 1.1–1.2 of [WINTER_MOTION_PLAN.md](./WINTER_MOTION_PLAN.md). Distilled
in [the walk's step width and sideways sway](./docs/knowledge/character-animation/ik-and-locomotion/walk-step-width-and-sideways-sway.md).

**Measured first (headless, `puppet_base`).**
- The walk's feet tracked under the hip sockets, 22.9 cm apart.
- The pelvis never moved sideways (±1 mm).
- The authored locomotion layer's Hips roll (±2.86°, pure roll, no yaw)
  is mistimed. It lowers the *stance* side through late single support,
  the reverse of Winter's H1-F.
- The authored layer moves each planted sole 45–48 mm in the target pose;
  the foot IK absorbs it. Step 1.3 replaces it.

**Step width 13 cm** (`stance::STEP_WIDTH`, 0.57 of the socket spacing).
This is the narrowest width at which the pendulum's COM still passes medial
of the stance foot's inner border, measured from the foot mesh. Margins
4.3 / 9.2 / 11.9 mm at 0.7 / 1.2 / 1.6 m/s; 10 cm crosses the border.
`narrow_feet` turns each leg about its hip with the foot turned back, and
`WalkCycle` composes Winter's stride onto the narrowed legs. The gait
mirror test now compares true mirror images: a turn about forward flips
sign between the sides.

**Sideways sway** (`phase::WalkSway`, `walk_sway_at`). It follows Eq. 11.3,
the pressure moving foot to foot as a trapezoid wave, with each harmonic
divided by `1 + K(2πk/T)²`. The pelvis sways 2.3 / 1.8 / 1.6 cm toward the
stance foot, peaking at ~0.31 of the stride. The formula matches a direct
finite-difference solve within 0.5 mm; a phase-shift sabotage fails it by
11.6 mm.

Feet held under the sway:

| | before | after |
|---|---|---|
| planted, single support | 0.00 mm | 0.02 mm |
| planted, double support | 1.2 mm | 0.67 mm |
| swinging toe | 0.6 mm into the floor | 0.01 mm |

This needed a load-weighted pelvis height (`sway_over_loaded_feet`) and
`keep_ankle`, which bends the knee for each leg's residual. `solve_leg_on`
was tried for that and made it worse (1.4 / 4.2 mm): it re-aims the foot.

**Live, 1.2 m/s** (BRP, schedule `0:0,4:1.2,10:0`, A/B against the
previous build in a worktree):

| | before | after |
|---|---|---|
| step width | 230 mm | 134 mm |
| pelvis sway vs root, peak to peak | 0 mm | 36.8 mm |
| start, worst planted slide | 1.6 / 2.5 mm | 1.4 / 2.3 mm |

Front and Left gizmo views at frames 110 and 128: both legs converge to feet
inside the hips; the sagittal stride is unchanged.

Two issues predate this change and are unchanged by it (A/B):
- **The root weaves ~90–100 mm sideways over a steady walk.** Its heading
  swings ±14°.
- **The last step glides.** It moves 8–10 cm near the floor after the root
  stops. `slide.py`'s "stop" figure (15.9 mm before, 20.5 after) is this
  glide, not a planted foot. The previous entry's 2.9 mm came from a
  different capture.

`anim_bench` now calls the rig-aware `apply_on` on a stance base, as the
plugin does, with the same change in the baseline. Per character per frame:
2.4 → 3.8 µs. Evaluating the COM every frame had cost another 2 µs; a
pinned height ratio (`COM_OVER_HIPS`) replaced it. 1004 tests pass.


### A first step that plants and lifts; a walk that no longer drops onto each leg

Closes the open first-step problem from the entry below, and a jerk seen
in the steady walk. Four causes, each measured:

- **Foot locks rode along with the body.** A lock pins a toe in the
  pose's frame and was never told the entity moved, so a foot locked while
  standing was carried by the first step's root motion. Its speed test
  read pose-frame speed, so it could never lock a planted foot during a
  walk. `footlock::Turn` gained `travel` (world axes, rotated into the
  pose's by the IK stage; set by `ride_rendered_feet` and by
  `Authoritative` root motion). The anchor is shifted back by it and the
  speed is judged in the world. Live stance slide at the start: 17.6 → 2.7 mm.
- **Root motion missed the hips moving.** Contacts are measured from the
  hips, and the release moves the hips through `root_translation` (4.5 cm
  sideways, 4 cm forward). Fading that out walked the planted foot 47 mm
  headless and ~13 cm live, and left the whole walk 5 cm off to one side.
  `root_displacement_between` now adds the horizontal hips motion.
- **Fading through double support.** With both feet down the walk holds
  their spacing, and a blend whose weight is changing scales it: the
  unloading foot slipped up to 12.5 mm a frame. Both fades now run through
  single support only (`TransitionConfig::fade`): the first step from
  mid-swing to heel contact, the last from the other foot's toe-off to its
  mid-swing.
- **The first swing skimmed the floor.** Mid-swing is where a walking
  foot is lowest (1.5 cm, Winter), and blended with a standing foot it slid
  94 mm before rising 2 mm. Reweighting joints could not fix it (a leading
  knee pointed the toe 2 cm into the floor). `Transition::blend` holds the
  swinging toe ≥ 5 cm up early in the fade, via leg IK.

Headless first step (`transition::tests`): stance slip is no more than the
steady walk's over the same stretch of stride. The test fails at 73 mm
without the hips term and 32 mm fading into double support. The swing ball
never travels more than 2 cm below 1 cm of lift.

**The walk dropped onto each leg.** The pelvis height the planted feet ask
for is lumpy on this rig: it fell 14 mm through late single support and was
caught at heel contact, 9.5 m per cycle² (headless). Live it was worse,
because floating feet made the foot IK drop the pelvis. The walk now rides
one sinusoid per step (`walk::BOB_HARMONICS`), fitted under the raw path
through single support: highest at midstance (0.22), lowest just before
heel contact (0.47), Winter's shape. Planted feet press up to 15/11/7 mm
into the floor in the plain pose (0.7/1.2/1.6 m/s), and the foot IK lifts
them by bending the stance knee (to ~22° at the slow walk; Winter's
midstance knee is 15–20°). Keeping the 4th harmonic too measured 2.6 per
cycle², with the lowest point still late in single support. Re-solving the
legs in the gait to keep feet exactly on the floor was tried and dropped:
the IK re-planes the leg (thigh 2.8° off Winter's, mirror broken, 0.11 m/s
velocity jump).

Live, 1.2 m/s, steady walk, same schedule:

| Pelvis | Before | After |
|---|---|---|
| Vertical range | 31.8 mm | 11.2 mm |
| Fastest fall | 0.525 m/s | 0.070 m/s |
| Vertical acceleration p95 / max | 21 / 44 m/s² | 0.86 / 1.3 m/s² |

Start/stop windows, worst planted slide: 1.5/2.2 mm (start), 2.9/1.1 mm
(stop). Also: the swing guard's ankle turn is clamped to 20° a step. On the
synthetic rig a foot 18.6 cm under the floor asked it for ~3 rad, which
broke the mirror test. 1000 tests pass. `anim_bench`: 2.2 µs per character
per frame (was 2.0).


### Weight shifts, a prepared start, a half-length last step, Winter's limb masses

Three items from Winter, plus two real bugs they exposed.

**Occasional weight shifts (§11.2.1).** `QuietSway::weight_shift` puts a
deliberate shift onto one leg now and then. Time runs in 14 s slots, the
first always square, and a fixed hash picks square, left or right per slot,
eased over 1.6 s. `stance::shift_weight` poses it: pelvis 4.5 cm over the
loaded foot, rolled 4° down on the resting side, spine rolled back level,
both legs re-solved to where the feet stood. On the rendered rig: COM
3.4 cm over, loaded knee 14.1°, resting 25.9°, shoulders level. Live over
BRP, the pelvis spans 57 mm and the feet move 0.1 mm.

**Starting (§11.3.2).** `Transition` gained a release. For 0.5 s, before any
foot moves, the weight goes onto the stance leg, which is the one the idle
already loaded, if any. The body also tips 4 cm forward about the ankles,
scaled by speed. The gait then joins at the swinging leg's mid-swing, where
the walk is closest to standing, and fades in over the rest of that swing.
Live: pelvis 4.5 cm onto the stance foot and 2.9 cm forward with the feet
still, then ~23 cm forward by the first heel contact (Winter: ~25 cm).
**Was open, fixed in the entry above:** the first swing foot lifted only
5 cm and the stance foot slid ~14 cm in the first step. The cause was not
the single weight but the foot locks, root motion and fade timing.

**Stopping (§11.3.3).** From a footfall, the gait fades over the time to the
other leg's mid-swing (half the duty factor). The legs keep the walk's
cadence (`stride_speed`) through it. The old stop dropped the clock to the
idle's 1/7 Hz and froze the step half-way. Live A/B on one schedule:

| | Before (HEAD) | After |
|---|---|---|
| Body still after the stop command | 2.4 s | 0.16 s |
| Planted-foot slide in the stop | 73 / 77 mm | 31 / 0 mm |
| Last swing | — | 41 cm (≈ half a walking swing), lands 4 mm beside the planted foot |

Winter's last foot lands half a step ahead; ours lands beside, so the
character ends in its standing pose. The standing sway eases back in over
1.5 s after a stop. Switched on at once, it ticked the pelvis 6 mm in one
frame. The phase layer is now `PhaseLayer::between(standing, locomotion,
weight)`, so the walk's 0.09 rad spinal twist no longer switches on or off
at a threshold. `--anim-speed-schedule T:SPEED,...` makes these runs
reproducible.

**Ragdoll limb masses (Table 4.1).** Arm, forearm-and-hand, thigh, shank
and foot bodies take Winter's mass, COM fraction and transverse inertia
m(ρL)² (`ragdoll_plugin::limb_mass_properties`, avian's auto properties
off). Real-rig thigh: COM within 5 mm of 43.3% hip→knee, inertia within
1%. With the properties disabled the test fails at 2.9 cm.

**Bugs found on the way:**
- *The pelvis rotation lagged the legs.* The hips were on the spine's
  0.16 s spring. The weight-shift roll then swung the rendered feet 4.2 cm
  about the hips, hidden by the standing foot lock until the first step
  released it as a slide. `Bone::Hips` now springs with the legs (0.015 s):
  ≤ 5 mm. See [a lagging pelvis rotation slides planted feet](docs/knowledge/character-animation/ik-and-locomotion/a-lagging-pelvis-rotation-slides-planted-feet.md).
- *`sway_over_feet` lifted the feet under a big lean.* It turned each leg
  rigidly but kept the pelvis level. With the hips ~5 cm ahead of the
  ankles, a 4 cm lean lifted the feet 3.3 mm. The pelvis now drops by what
  keeps each hip-to-ankle distance, an inverted pendulum about the ankles.

996 tests pass. `anim_bench`: 2.0 µs per character per frame (100 × 600).


### The relaxed stance looks ahead, and stands still the way a person does

Measured against Winter before changing anything, with a new whole-body
centre-of-mass model (`anthropometry.rs`: Table 4.1 Dempster segments on
the rig's joints; the ragdoll's masses now read the same table).

**First, a measurement trap.** On `gltf_rig::puppet_base()` the relaxed
stance's hands came out a metre ABOVE the hips, and the body looked
slouched 11° over its toes. The live character (BRP) hangs them at its
sides: the fixture faces +Z while the renderer turns the character to −Z,
and a pose authored in fixed world axes means different things on the two.
`gltf_rig::puppet_base_as_rendered()` reproduces the live arm heights to
5 mm (`the_rendered_rig_matches_the_live_character`). The gait, feet and
stance are written against the rig's own forward and were never affected.

**On the rendered rig:**

| | Before | After | Reference |
|---|---|---|---|
| Gaze | 30.5° down at the floor | level (0.0°) | head level |
| Centre of mass ahead of the ankles | 4.8 cm | 4.8 cm | ~4 cm, Winter Example 5.1 static stance |
| Trunk vs the bind | −0.5° | −0.5° | upright |
| Standing sway | spine bent over a fixed pelvis: shoulders 3.2 cm, COM 1.3 cm side / 0.16 cm fore-aft | pelvis carries the trunk over planted feet: live, pelvis 12 mm side to side and 8 mm fore-aft, head 11.5 mm with it | Winter §11.2.1: side-to-side COP within ~2 cm, hip load/unload, ankle pivot |
| Feet while standing, live | — (then 12 mm, see below) | 0.1 mm | planted |

- **The neck**: the Mixamo clip bows it 41.3°, which is the whole slouch.
  Re-solved to 10.64° about the same axis for a level gaze, over the clip's
  own spine curve, which is kept because it is what holds the COM at
  Winter's 4 cm. Removing the spine curve too levelled the gaze but pushed
  the COM to 6.5 cm.
- **The sway**: `PhaseLayer` gained `QuietSway`, applied with the rig by
  `apply_on` through `stance::sway_over_feet` (each leg turns about its
  ankle, each foot turns back, the pelvis shifts). Breathing trimmed 0.022
  → 0.012 rad: it had moved the head as far as the whole-body sway.
- **Root motion while standing**: the gallery's new rendered-contact root
  motion read the standing sway as travel and walked the whole character —
  feet wandered 12 mm live, the pelvis twice its sway. Root motion now only
  runs while the gait has weight, and the phase layer follows whether the
  character is standing or walking rather than how it spawned.

Tests passing **984 → 989**. The walk is unchanged live (ball of the foot
≤ 1.2 mm a planted run).


### The walk replays Winter's measured stride

The walk's legs are now driven by a real recorded stride — Winter,
*Biomechanics and Motor Control of Human Movement*, Appendix A (one adult,
1.43 m/s, 61% stance) — instead of hand-shaped curves. The run keeps the
authored curves (`LegCurves::Authored`); there is no measured run.

**Why.** Measured against the recording on `puppet_base`, the authored walk
was a crouch: the knee never straightened below 20.6° (recorded: ~0° at
contact, 5° mid-stance), the ankle never pushed off (+1…+41° dorsiflexion
against a recorded −20.5° push-off), and the hip swung twice the recorded
range to make up for the short, bent-knee step. RMS gaps: hip 20°, knee 17°,
ankle 23°.

**What was built.**
- `reference.rs`: the stride as 7-harmonic curves (Winter §2.2.4; fitted
  within 0.23°/0.41°/0.67° of every hip/knee/ankle sample). The data is
  `assets/anim/reference/winter_walking_stride.csv`, extracted by
  `tools/extract_winter_stride.py` from the (gitignored) PDF, and pinned
  against the digest's page-checked rows.
- `walk.rs`: the measured walk on a rig. The thigh is driven by its angle
  from vertical and the foot by its pitch on the ground, knee from the
  table; see the knowledge note *Replay a recorded gait by segment
  attitudes* for the four mappings that failed first. A per-rig thigh
  correction (≤1.4°, 3 passes, memoized) keeps double support planted; a
  clearance guard holds the swinging toe 1.5 cm up (Winter's measured toe
  clearance).
- `foot.rs`: feet touch the ground at heel, ball and toe tip, not the
  ankle, and ONE set of support weights decides pelvis height (a soft
  maximum), root motion and drift.
- `GaitParams::walking_on(speed, rig)`: Froude scaling onto the rig's leg;
  the duty factor grows toward 65% for slow walks.
- Root motion is the planted contact's displacement between poses
  (`locomotion::root_displacement_between`), in the gallery measured on the
  poses the springs actually render (`ride_rendered_feet`).

**Measured.**

| | Before | After |
|---|---|---|
| Thigh / knee / planted-foot attitude vs recording | 20° / 17° / 23° RMS | ≤2° / 2.5-3° RMS / ≤1° |
| Planted-foot slide, live, ball bone, 0.7-1.6 m/s | 4-15 mm (ankle) | ≤1.2 mm |
| Planted-foot slide per stance, headless | — | 5.9 mm (hand-overs only) |
| Speed within a stride (user chose the real rhythm) | ±0.1% (constant) | 0.77-1.48× at 1.2 m/s, 0.78-1.47× at 1.6 (Winter's pelvis: 0.73-1.36×); fastest in double support, slowest over the foot |
| Pelvis bob | 33 mm | 14 mm headless; 15-38 mm live |
| Gait cost per frame (pose + root motion) | 18.9 µs | 15.2 µs, plus 0.5 ms once per rig/speed |

Tests passing **973 → 984**.

**Dead ends, recorded.** Imposing Winter's pelvis bob on the rig (0.25-1.0×,
three anchorings) cost 7-10° of thigh and 9-13° of knee and floated feet
16 mm; the legs lead instead (knowledge note *Recorded pelvis path and
recorded leg angles cannot both be kept*). A fully converged double-support
correction held feet to 0.03 mm but bent the thigh 4.4° and inverted the
bob. A two-sided "landing" swing guard jumped the ankle at footfall.

**Not done here.** The relaxed stance bowed the head; fixed in the entry
above. (Its "6° forward lean" was a measurement on the wrong-facing
fixture.)


### A stutter every ~12.5 s: the spring rendered its substep, not the frame

Reported live as "sometimes jerky for a moment, or with a period". The
spring steps in whole 1/120 s substeps while root motion and hip height
follow real frame time, and the rendered pose was the last substep's —
trailing each frame by a varying 0–8.3 ms. With the legs near-instant
(0.015 s), that slack is a foot pop. The display is 59.96 Hz: a frame is
0.011 ms longer than two substeps, so the slack crosses a substep boundary
every ~750 frames (~12.5 s), and near each crossing vsync jitter picks 1,
2 or 3 substeps per frame.

Replayed headless (`a_walking_foot_moves_smoothly_at_any_frame_rate`), the
worst frame-to-frame change in a walking foot's velocity:

| | exact 60 Hz | 59.96 Hz ±0.25 ms | 144 Hz |
|---|---:|---:|---:|
| substep state rendered | 0.68 (the footfall) | **3.5** at t≈12 s | 4.3 |
| + state carried to frame time | | 1.27 | 0.67 |
| + substeps see the target at their own moment | | **0.71** | 0.31 |

Two changes in `DhoState::advance`, both leaving the simulation
deterministic: the rendered pose is the state stepped on by the leftover
time (derived, never fed back), and each substep chases the target
slerped to its own moment between the previous frame's and this one's,
instead of holding the frame-end target (which spread a footfall
differently across a 1- vs 3-substep frame). The test fails with either
disabled (3.5 / 1.27 against a 1.02 bound). Cost: `anim_bench` 100
characters, 1.7 → 2.0 µs per character per frame.

Not yet ruled out live: foot IK and its lock/unlock, which run after the
spring. The replay covers the gait and springs only.


### A walk that travels steadily, plants its feet, bobs, and swings its arms

Four walk defects, each measured live over BRP on the real rig and pinned
by a real-rig test. Test count **958 → 972**. Per-frame gait cost (target
pose + root velocity) **27 → 18.9 µs**, after hoisting a
`thigh_cycle_mean` that ran once per bone per call.

**The speed surge was a clock mismatch.** Root motion read the target pose
at one leg-clock rate while the pose played at another, and the speed
coupling was a guessed `base + c·speed`. Now the cadence is derived from
the walk's own `locomotion::distance_per_cycle`, measured on the same
rendered (blended) pose the character shows, via `root_velocity_of` /
`advance_turning_with`. Travel speed ±25% within a stride → **±0.1%**;
`a_walking_body_travels_at_a_steady_speed` at 0.5/1.0/1.5 m/s, ±2%.

**Speed now changes the stride, not just the cadence.** `GaitParams::walking_at`
scales stride with speed^0.65 (cadence carries the rest, ≈ speed^0.35, as
in real walkers). Live travel over the same window: 0.60 : 1 : 1.50 at
0.6/1.0/1.5 m/s.

**The planted foot slid 110–180 mm per stance, from three sources:**
1. the stance thigh angle was authored, so the ankle's path was not a
   line — `gait::stance_thigh` now solves the thigh (secant) so the ankle
   travels linearly between its authored footfall and toe-off;
2. the clock mismatch above;
3. the leg springs. A 0.12 s spring low-passes the stride, so the
   rendered foot swings less than the target the body is moved by. Sweep
   at 1 stride/s: 0.12 s 302 mm, 0.06 s 99, 0.04 s 27, 0.03 s 7.6, 0.02 s
   0.7 (9.4 at 1.6 strides/s) → legs **0.015 s**.
Live now: worst slide per planted run **4–15 mm** at 0.6/1.0/1.5 m/s
(BRP sampling-limited); `the_default_springs_keep_a_walking_foot_planted`
< 5 mm at 1.0 and 1.6 strides/s.

**The torso bobs, and the bob was going the wrong way.** `stance_hip_height`
derives hip height from the stance ankle's depth (load-weighted through
double support), so it is highest over the stance foot and lowest in
double support: 33 mm at 1 m/s (13 at 0.5, 60 at 1.5). Live, the pelvis
first moved **63.4 mm along the travel axis and 0 mm vertically** —
`retarget::hips_world_position` rotated the root translation by
`hips_root_rotation` a second time. Fixed; the tautological test that had
blessed it (it computed the expectation with the same function) is gone,
replaced by `a_root_translation_moves_the_rendered_hips_along_the_poses_own_axes`.

**The arms swing with the opposite foot, and live.** Three bugs: the swing
axis came from the synthetic rig; the timing was a quarter cycle off; and
the delta was composed in the T-pose frame (fixed with the new
`rig::delta_after_world_turn`, see *a pose delta names a world axis*).
Swing is now timed off the opposite footfall with a small lag
(`ARM_LAG` 0.02 cycle), biased forward (`ARM_FORWARD_BIAS` 0.3), with an
elbow that never straightens past 60% of its bend and folds as the arm
comes forward. Walk `arm_swing` 0.50 → 0.30, `elbow_bend` 0.35 → 0.45.
Even then, live counter-swing was **42%** — chance. The arm springs (0.12 s)
added ~88° of phase lag at walking cadence. Arms → **0.03 s** (~27° lag);
live now **93% / 93% / 88%** of leg-split samples at 0.6/1.0/1.5 m/s
have the opposite hand ahead (misses sit at the zero crossings).

**A stiff spring never settled.** At the f32 floor the error read back from
`goal⁻¹·current` is quantised, and the exact spring solution answered it
with a velocity that could not decay (a 0.03 s arm: 7.9e-6 rad/s forever,
flipping its last bit every frame). `dho::REST_SNAP_ERROR`/`_VELOCITY`
place a bone exactly on target below 1e-6 rad and 1e-4 rad/s. The test
that should have caught it could not: `1 − |dot|` of a quaternion with
itself is 6e-8 for this value, so its 1e-9 bound failed on identical
rotations and passed on the old spring only by rounding luck. It is now
bit-exact, and fails with the snap disabled.

The ragdoll arm test now asserts per-sample tracking (< 10° worst) rather
than swing range, which the smaller authored swing had made too blunt.


### Ragdoll known gaps: a root that walks, characters that collide, a head that is a head

The three gaps the previous entry listed, plus the one closing them
exposed. Test count **951 → 958**.

**The pinned root walks and turns with the character** — two bugs no
standing test could see. The kinematic hips body was never moved, and the
targets were rooted at the hips' parent's BIND-time rotation, so a turning
character's targets were off by the turn (sabotaged: exactly 90.0° after a
quarter turn). Now `rig_geometry` reads the parent's rotation live via
`TransformHelper`, and `follow_kinematic_roots` drives the root by velocity
inside the physics step. Live: the hips body travels with the walking
character (z −2.76 → −8.43 m over 6 s).

**Characters' ragdolls collide with each other.** One shared layer, filtered
by every ragdoll, also hid them from each other. Each now takes one bit of
`RAGDOLL_LAYER_POOL` (top 16 bits, round-robin) and filters only its own;
past 16 live ragdolls, bit-sharing pairs pass through each other — or set
`RagdollSpawnConfig::collision_layer`.

**The head is its own body.** A head-and-neck body owned by `Neck` rotated
with the neck (41° from bind in the relaxed stance) while the head bows 29°.
Now owned by `Head`, running up from the skull base along bind-pose
vertical carried by the head — exactly vertical at rest on any rig. Its
joint spans the neck, so its limit combines both (70° / ±80°). Two guesses
were measured and discarded on the way: the head's own `+Y` (1.7° from
bind vertical on this rig, but not in general) and the neck direction.

#### What closing gap 1 exposed: the ceilings were too low to track

With the root moving, the walking character's forearms, feet and head
flailed **20–176°** off target while the torso tracked. Not frame rate
(36–40° at 64, 60 and 144 Hz), not ground contact (the gallery has no world
colliders). A sweep settled it: the authored ceilings could not produce the
accelerations the character's own motion demands — a ball joint carries no
torque, so a forearm holds its angle against a swinging elbow by its own
controller alone, and at 40 rad/s² it saturated.

| ceilings | walking root | 1 Hz arm swing |
|---|---|---|
| x1 | 39.8° (head) | 79.7° (forearm) |
| x3 | 0.9° | 113.9° (forearm) |
| x6 | 1.2° | 10.0° (the swung arm's own lag) |

`CEILING_SCALE = 6` at first — raised to 12 by the real-rig measurement
below. Joint damping, swept alongside, changed nothing once ceilings were
adequate — so it was not added. Yielding to a blow no longer
needs a low ceiling: that is the stun's job.

**A test replaced, and why.** `the_shipped_joints_are_calm_at_their_own_ceilings`
used a one-limb, immovable-parent proxy whose documented torque/constraint
oscillation scales with authority (6.3 rad/s arm, 10.4 hip at x6). The real
character at x6 rests within 0.1° with ~0 spin, live. The progress half is
kept; the calmness claim moved to the whole rig,
`a_ragdoll_settles_quietly_into_a_mid_range_pose`. Dead end: rotating each
body about its joint (adding `α × d` at the centre of mass) DOUBLED the
proxy's oscillation, and was reverted.

#### The walking head and the post-turn ringing — both saturation, x6 was not enough

**The walking head (47–51° live) was saturation too.** Sampled live, its
TARGET moved 0–1° between samples while its BODY swung 4–51°, almost
entirely as a bend (≤2% of the spin about its own axis) — not lag, a
controller unable to hold a heavy head on a neck-length lever. A new
headless fixture on the REAL rig (`spawn_real_rig_ragdoll`, the parsed
`puppet_base` bind pose as a parented hierarchy) walking its own gait
showed the arms overshooting at x6: body swing **71.7°** against a
**57.3°** target swing. At `CEILING_SCALE = 12` it is 57.4 — pinned by
`on_the_real_rig_a_walking_arm_swings_as_far_as_its_animation`, which
fails at x6 — and live the head drops out of the worst six.

**The post-turn ringing was measured at the ORIGINAL ceilings**, not at x6
as the paragraph above first said: the walk-and-turn test with a 1 s hold
fails at x1 (13.4°) and passes from x6 up. It now holds only 1 s.

**Dead ends, each measured and reverted:** removing the head's limit (head
still up to 39°); velocity feedforward in the PD — damping `ω − ω_target`
from frame-differenced targets — made the feet worse (24° → 80°); frame
timing, twice (60/64/144 Hz, then uneven frames): no effect; a kinematic
root that follows the frame's measured pace instead of arriving in one
step: no measurable change.

**Still open, and it is not the ragdoll:** the walk's ROOT MOTION lurches.
With the ragdoll OFF, the rendered pelvis's forward speed swings
**0.3 ↔ 1.76 m/s** every step (~±70%; a human pelvis varies perhaps
±10–20%). The kinematic render hides it because its arms follow the
animation rigidly; the pinned ragdoll reproduces it faithfully, and arms
hanging from a lurching torso swing like pendulums — live, **73–77°**
against a 22° target swing. That is the physically right response to the
input; the input is the defect, in the locomotion's root velocity (derived
from the planted foot's hip-relative motion).

### Hits, and a ragdoll that actually tracks on the real rig

Item 6, ragdoll triggering. `RagdollHit::new(character, bone, velocity)`
shoves a limb, slackens the joints around it, and lets them pull back as
strength returns — no flinch clip, no state machine. Test count
**928 → 951**. Gallery: `--hit-at-frame N` (reproducible screenshots) and
`H` live.

**The bigger finding: the ragdoll had never tracked the real character.**
At full strength the read-back shows the animation by construction, so
"a fully driven one stands correctly" (Stage 4 entry, below) was vacuous.
Measured live over BRP on `puppet_base`: every body **35–178°** off its
target, with and without gravity. Now: worst body **0.1°, ~0 rad/s**, in
three independent runs. Seven defects, each bisected by measurement:

| defect | measured | fix |
|---|---|---|
| private copy of the pre-fix rotation convention | 50.6° on the real rig's hand | `joint_targets` → `rig::accumulate_world_rotations`; `delta_from_world` inverts it |
| jointed and nearby bodies collide | neck 8.8° off, spinning 3.6 rad/s from step 1 | `RAGDOLL_LAYER`: no self-collision |
| limit cones in the frame bodies stopped using | thighs 121° off, identical with gravity off | limit frames centred on the bind pose |
| shipped poses outside the limits | 49 violations (`relaxed_stand` neck 41° vs 35°) | anatomical ranges, pinned by a pose-vs-limit invariant |
| PD cannot see load | torso folded to 178° under gravity | `GravityScale = 1 − strength` |
| read-back wrote physics into the spring state | recovered forearm frozen 55.9° off | `Ragdoll::displayed` |
| ragdoll on switched foot/arm IK off on screen | 34.4° in the regression test | blend from `AnimFootIk::corrected` |

**avian 0.7's joint limits are not a cone and a twist.** Measured:
`swing_limit` bounds the tilt of `twist_axis.any_orthonormal_vector()`,
`twist_limit` the twist axes' roll about it. With the default
`twist_axis = +Y` along the bone they become two unrelated bend stops
(the live arm's 85° bend clamped at ~68.5° by its "twist" range).
`twist_axis = +X` makes the reference `+Y`, the bone, and the two become a
true cone and a true twist — verified empirically, pinned by two tests.

**Fewer, chunkier bodies (17 → 14).** One body per non-leaf bone made the
0.083 m neck and 0.106 m lower spine near-inertialess capsules: live, the
neck spun at up to **1769 rad/s**. Now pelvis, two torso, head-and-neck,
and upper arm/forearm/thigh/shin/foot per side, with anthropometric masses
(Dempster/Winter) — volume-derived masses on the same layout spun the upper
torso at 1047 rad/s — and torso-width (0.13 m) torso capsules.

**Why a hit is a velocity:** bodies are grams at avian's default density;
as an impulse, a 1.5 m/s request launched a body at 521 m/s.

**Dead ends:** length-proportional mass (a stub still tumbled), raising the
neck's ceiling 10× (saturation was not the cause), a spawn-pose-centred
limit frame (depends on when the ragdoll attaches — arms moved 71° after).

**Verified:** Front and Left, `--gizmos on --show-real-mesh off`, hit at
frame 400 against a no-hit baseline: left arm knocked out at 410, matching
the baseline at 640; cyan bodies on the white skeleton in every shot.

**Known gaps:** the pinned root does not follow root motion (a walking
ragdoll's hips stay where they spawned); ragdolls of different characters
pass through each other (`RAGDOLL_LAYER`); the head-and-neck capsule
extends along the forward-flexed neck, so its tip sits ahead of the real
head. avian 0.8-dev has no spherical motor (issue #934) and still solves
joints with XPBD (#440); bevy_rapier3d 0.36 supports Bevy 0.19 — see the
session's migration assessment.

### The knee bends forward, and the tests can finally see direction (ed176a3, 2128c5e, 18a5834, 4c7fbc9)

Reported as "knees bend in opposite to natural human angle", three times
across two sessions, while the whole suite stayed green. The defect was real
and the reason it survived is the more useful half of this entry.

**The bug.** Two-bone IK has two solutions, mirror images across the line to
the target; only one puts the knee in front, and which one depends on which
way the rig faces. `solve_leg_grounded` hardcoded the negative branch,
reasoned out for "a target ahead at `-Z`" — the synthetic rig's facing. The
rendered rig faces the other way, so the solver re-bent every knee backward,
every frame. Measured through the live pipeline:

```text
  stance pose entering the IK stage   +0.061   knee forward, correct
  pose the IK stage wrote back        -0.134   knee backward
```

Now both branches are constructed and the one whose knee lands forward wins.
Two extra quaternion multiplies, no assumption about a rig it has not seen.

**Why 30+ leg tests missed it.** Every one measured the *unsigned* angle
between thigh and shin, which is identical whichever way the knee folds. The
fix is two signed measurements on `RigGeometry`, and knowing which to use:

- `knee_forward_offset` — which side of the hip-to-ankle line the knee sits
  on. Intuitive, and right on a bent leg.
- `knee_fold_direction` — how the shin turns relative to the thigh. The one
  to trust near full extension, where the knee is *on* that line by
  definition and the offset's residual is dominated by the hip's lateral
  placement.

That distinction cost a wrong diagnosis of its own: the synthetic rig reads
`-0.017` at 99.2% extension, which was filed as a second backward-knee
defect. The fold at those same phases reads `-0.34` — solidly human. There
was no second defect.

Both anatomical invariants now use the fold and cover **both rigs, every
phase, walk and run, with no exemption and no tolerance**. Sabotage-verified:
inverting the gait's facing sign fails at phase 0.000 on the synthetic rig,
which the offset-based version could not detect at all.

**Three supporting fixes.** `hip_dip`/`vertical_bob` became fractions of leg
length (they were metres authored against a 0.49 m leg while the real one is
0.888 m); the standing knee flex moved out of `relaxed_stand.pose.ron` and
into `stance_on_rig`, because a stored rotation cannot know which rig it will
drive; and `LegIkConfig::max_knee_deviation` now refuses a solve that wanders
more than 1.75 rad from the animated bend — the ceiling measured from real
adaptation (1.117 rad legitimate) against the degenerate end (3.141 rad).

**A claimed fix that was not one.** A "5.3-degree toe drift" reported here as
a real non-idempotence was an artefact of the tip-clamp tests' `aim_foot:
false`. On the production default the drift is 0.14 mm. Caught by
sabotage-verifying the test written for it — it passed with the fix disabled,
so it was measuring nothing. The convergence pass was removed.

928 tests pass; clippy unchanged at 20 warnings. Live: fold -0.57 to -0.70
across the cycle, human on every sample.


### Root motion no longer wipes the mesh's facing correction (6844c88)

Reported as "when gait speed isn't zero the character moves, but moves
backward, and knees bend in wrong angle (like grasshopper legs)". The first
half was a real bug; the second half was not, and finding that out cost far
more than the fix.

**The bug.** `drive_walk_cycle` wrote `root.rotation = facing.rotation()` as
a bare assignment. That was correct while the rendered mesh and the animated
skeleton were separate entities — root motion drove the debug skeleton and
left the mesh alone. When the debug-capsule skeleton was removed and
`HumanoidSkeleton` moved onto the mesh root itself, the same line began
overwriting the asset's 180-degree `--character-yaw-correction` every frame.
The comment defending it went stale at that moment and kept reading as
correct.

A/B against the parent commit, same flags (`--anim-speed 1.2`), reading the
mesh root's own `Transform`:

```text
  before   rotation = (0, 0.0, 0, 1.0)   <- correction wiped
  after    rotation = (0, 1.0, 0, ~0)    <- correction kept
```

The character travels toward `-Z` in both. With the correction gone its
geometry faces `+Z` while travelling `-Z` — it walks backward. Now composed
via a `FacingCorrection` component recorded at spawn, heading first and
correction second.

**The knees were never broken.** Sampled across a full stride on both
builds, the knee sits **0.140-0.154 m ahead** of the hip-to-ankle line,
**zero backward samples**, identical before and after.

Every contrary reading came from a **stale `character_gallery` process still
holding BRP port 15702** — a second instance cannot take the port, so the
queries silently answered from an older build. That fed a confident false
diagnosis: the knee was "measured" backward, bisected across the gait
curves, retargeting, foot IK and the phase layer, and used to justify a
`RigGeometry::flexion_sign` mechanism (deriving each rig's facing from its
own `ankle -> toe`, since `puppet_base.gltf` faces `+Z` where the synthetic
T-pose faces `-Z`). All of it was reverted: in the gallery
`build_real_mesh_skeleton` already folds the yaw correction into
`hips_root_rotation`, so the live rig reports `flexion_sign = +1` and the
mechanism was a no-op in production.

Two signals that should have caught the staleness sooner, both present and
both missed: values **bit-identical across many samples** while the
character should have been animating, and a root translation stuck at `0.0`
while travel should have been advancing. What finally settled it was
building the unmodified parent commit in a `git worktree` and A/B-ing the
two binaries on the same input.

920 tests pass; clippy unchanged at 20 warnings (git-stash compared).


### Arm IK works end to end, and the toes lie flat — three more frame bugs

`hand_l` now lands at **(0.3200, 1.1500, -0.2500)** for a target at
(0.32, 1.15, -0.25), with `hand_r` untouched. The previous entry fixed the
pose-space convention and reported the feature still broken; two further
frame bugs were sitting behind it, each masking the next.

**Bug 2: the substitute hips offset.** `solve_foot_ik` cannot read `Hips`'
live translation — that is the one value `write_pose_to_skeleton` overwrites
every frame, so reading it back feeds the solve its own output. It
substituted `Bone::t_pose_offset()`, the synthetic table's **Y-up**
`(0, 0.94, 0)`, while the real rig's `root_rotation` is a **Z-up**
correction that forward kinematics applies to the hips offset (the root has
no parent to inherit one from). The Y-up value became `(0, 0, 0.94)` and laid
the character on its back inside the solver: ankle y = **-0.856**, toe
y = **-0.927**, a metre underground.

The visible symptom was nowhere near the cause. The leg IK reacted
*correctly* to that garbage — `lift_toe_end_out_of_the_ground` saw a tip
1.006 m below the floor and rotated each toe **113 degrees** to rescue it —
so what rendered was feet whose toes pointed at the sky. The user spotted
that and flagged it; it is what led to the bug.

**Bug 3: the world -> pose rotation.** Read from the character entity, which
is not the top of the correction chain. `character_gallery` spawns its mesh
under a node carrying a 180-degree yaw (`--character-yaw-correction`,
default 180) so the model faces the camera, and that node sits *below* the
character entity and *above* `pelvis`. The entity read identity while every
live bone transform carried the yaw. Measured, with it unaccounted for:

| bone | world x | pose x |
|---|---:|---:|
| `LeftArm` | +0.2106 | -0.2104 |
| `RightArm` | -0.2237 | +0.2239 |
| `LeftUpLeg` | +0.1143 | -0.1143 |
| `Head` | -0.0140 | +0.0143 |

Every bone's x negated — a 180-degree yaw exactly. Now derived from the live
**hips**, which carry every correction between world and rig whatever the
asset's nesting, with the rig's own root and hip binds divided back out.

**The loader was innocent.** It was the prime suspect for two rounds, and the
previous entry named it as the likely cause. Dumping the gallery's captured
`rest_rotation` for every bone against the file's own parse: **bit-for-bit
identical**, including the toe. Two independent bugs both produced
mirror-shaped symptoms, and each looked like the whole story in turn.

**Also corrected here:** "a left-hand target moves the RIGHT arm", reported
in the previous entry, was a misreading of a front view — a front-facing
character's left arm appears on the screen's *right*. The underlying bug was
real; that particular description of it was not.

**Bug 4: the estimated toe tip.** The toes still tilted up after the first
three fixes, by much less — a tip 0.060 m above its joint, down from 0.073 —
and the cause was separate. `RigGeometry::from_gltf` MEASURES the toe tip
from the rig's own `ball_leaf_l` joint; `from_skeleton`, which the plugin
uses, can only ESTIMATE it, because `HumanoidSkeleton` has no toe-end concept
by design (a 23rd bone would invalidate every `[T; 22]`, `Bone::ALL`, and
every RON asset, for a point that is never rendered):

```text
  measured   (0, 0.0789,  0.0000)
  estimated  (0, 0.0711, -0.0356)    ~27 degrees apart
```

So `lift_toe_end_out_of_the_ground` rescued a tip that was never
penetrating. Every in-crate path kept the tip level to within 0.8 mm; only
the live plugin's estimate diverged.

Fixed by reading the toe joint's own **child** from the ECS hierarchy, which
needs no per-rig name table — whatever hangs off the toe joint is by
construction the point the toe runs toward, and a rig without one keeps the
estimate. Live, after: tip **0.5 mm below** its joint and 0.0789 m in front,
matching the measured bone exactly, both feet symmetric. The gap itself
stays pinned by `the_estimated_toe_tip_is_a_poor_stand_in_for_the_measured_one`
so nobody simplifies the plugin back to the estimate.

**Verification.** 920 lib tests pass; no new clippy warnings (15 before and
after); `anim_bench` at 100 characters measures **0.0018 ms/character/frame**,
exactly the standing baseline. Both frame fixes sabotage-verified — reverting
the hips substitution fails with hips at `(0, 0, -0.94)`, reverting the yaw
recovery fails "180.000 degrees off". Visual: Front and Left,
`--gizmos on --show-real-mesh off`, claim stated first — the left arm reaches
with a bent elbow, the right hangs, the skeleton stands upright with two
distinct legs, and the foot gizmos run forward and level.

**The through-line.** Four bugs, and not one of them looked like what it was.
Toes pointing at the sky were a hips offset in the wrong coordinate
convention; a hand missing its target was a yaw correction on a node nobody
thought to read. Twice the glTF loader was the obvious suspect and twice it
was innocent — settled by dumping its captured binds against the file's own
parse and finding them bit-for-bit identical. What worked, every time, was
measuring one link of the chain at a time rather than reasoning about which
link was most likely.


### The pose-space convention: forward kinematics disagreed with the renderer

The blocker the previous entry reported, run to ground. Two real bugs, both
in load-bearing shared code, both invisible to the entire existing suite for
the same structural reason.

**What was wrong.** A pose's rotations are authored against this crate's
synthetic T-pose, whose bind rotations are all identity — so "turn 40 degrees
about +Y" means the **world** +Y. `retarget::write_pose_to_skeleton` honours
that, conjugating each delta into the bone's bind frame. `rig::
accumulate_world_rotations` — which every IK solver reasons about — composed
`parent * bind * delta` instead, applying the delta in the bone's *local*
frame. Right angle, wrong axis.

A comment at `rig.rs:392` asserted the two were "the same composition". They
were not.

**How it was settled.** The previous entry's probe measured 56–80 degrees of
disagreement but built its retarget side from `rig.bind_rotations` rather
than calling the real writer — evidence, not proof, and I flagged it as
such. That caution was warranted: **the 56–80° figure was wrong**, an
artifact of the model. The real measurement runs the same pose through
`write_pose_to_skeleton` into a `World` and accumulates what it actually
wrote.

The discriminator is the residual `intent⁻¹ · actual`, where *intent* is the
synthetic rig's own answer (unambiguous — every bind is identity there). A
path that honours the contract has a residual depending only on the bone's
bind pose, never on the pose. Two different poses, compared:

| bone | retarget | forward kinematics |
|---|---:|---:|
| `LeftArm` | 0.000° | 32.816° |
| `LeftForeArm` | 0.000° | 39.011° |
| `LeftLeg` | 0.000° | 44.400° |

Retarget was right; forward kinematics was wrong.

**Why nobody noticed for months.** Conjugating a delta by a bind rotation is
a no-op when the delta's axis is parallel to the bind's — parallel rotations
commute. The leg chain is bound almost entirely about X, and every leg delta
a walk cycle produces (hip pitch, knee bend) is *also* about X. Give
`LeftLeg` a delta about Y instead and the same 34-degree error appears
immediately. The legs were never immune; they were only ever asked the one
question the bug cannot get wrong. The suite's leg coverage is extensive and
none of it could have caught this.

**A second bug, found by the first fix.** With forward kinematics corrected,
`aim_bone` still missed. Deriving the frame properly rather than guessing it:
forward kinematics composes `W(b) = W(parent) · bind_local(b) · [B(b)⁻¹ d
B(b)]`, so a world-space correction `c` needs `d' = P⁻¹ c P d` with `P =
W(parent) · bind_local(b) · B(b)⁻¹`. `P` is identity exactly when the
ancestors are at rest — which is why an unconjugated pre-multiply looks right
in isolation and degrades as ancestors move. Aiming the forearm right after
swinging the shoulder is precisely that case:

| approach | error |
|---|---:|
| unconjugated, 1 pass | 0.19922 m |
| unconjugated, 3 passes | 0.14885 m |
| the derived frame, 1 pass | **0.00000 m** |

**A dead end worth recording.** I first read that residual as a
linearisation artifact and made `aim_bone` iterate. Three passes did improve
it — 0.199 → 0.168 → 0.149 — which is exactly the kind of partial
convergence that invites declaring a broken solver fixed. It was converging
to the wrong answer. Deriving the frame instead made a single pass exact, and
the iteration was deleted.

**A third thing the fix exposed: the wrong measurement.** `armik`'s module
doc contained a measured table naming `+Y` as the arm's degenerate twist axis
and `+X` as the hinge. Re-measured under the corrected convention, it is the
exact opposite — the upper arm runs along world `(0.9995, 0, -0.030)`, so
`+X` moves the wrist 0.0058 m (nowhere) and `+Y`/`+Z` move it 0.186 m. The
old table was a real observation of a broken system. "Measured, not assumed"
is necessary and not sufficient; what is measured also has to be correct.

This also broke the `arms_bent` test helper, which bent both elbows about
`+X` to escape the straight-arm singularity and had therefore been bending
them 0.0058 m — i.e. not at all. Several tests were silently running in the
singularity they existed to avoid.

**And a fourth: the hinge must mirror.** A single world-axis hinge cannot
serve both arms, since the left upper arm runs along `+X` and the right along
`-X`. The wrist cannot detect this — the two-bone geometry lands it correctly
either way — so the symptom is visible only at the elbow: with a shared
hinge, elbows at `z = -0.291` and `z = +0.055`, both wrists exactly mirrored.
Now `ArmChain::bend_sign`, and the mirror test checks elbow depth rather than
only the wrist.

**Tests, including the one whose absence let this through.** The old
`accumulated_world_rotations_match_the_composition_at_write_back` hand-
composed the same wrong convention it was checking — the "same function both
sides" failure mode, in hand-written form. Replaced with:

- `retarget::…::the_world_rotations_agree_with_what_retargeting_actually_writes`
  — forward kinematics against the real writer, not a model of it.
- `…::the_agreement_test_is_not_vacuous_on_the_axes_it_picks` — asserts each
  chosen delta axis is one the bone's bind genuinely moves, so the test above
  cannot go vacuous. It immediately caught that `Spine1` is bound only 1.29°
  from identity and can never discriminate; `Spine1` is documented as
  excluded rather than quietly contorted.
- `…::the_forward_kinematics_positions_are_the_ones_bevy_renders` — the other
  half, and the half whose absence mattered most: rotations agreeing does not
  make positions agree, and positions are what an IK solver actually aims at.
- `rig::…::a_world_axis_delta_turns_about_that_world_axis_on_a_bound_rig` —
  the contract stated directly.

**Sabotage-verified.** Reverting the forward-kinematics conjugation fails 12
tests including all four new ones; reverting `world_correction_frame` fails
5, among them the **leg's** own accuracy test. Neither fix is covered
vacuously, and the second confirms the legs needed it too.

**Simplification.** `rotation_frame_of` and `parent_frame_of` are deleted
rather than fixed — two flavours of a change of basis that was never the
right operation. `lookat` and the two foot-grounding corrections now share
the one derived frame.

**Measured result on the live rig.** `hand_l` with a target at world
(0.32, 1.15, −0.25): X error went from wrong-on-every-axis to **0.34 mm**.
916 lib tests pass, clippy adds no new warnings (15 before and after), and
`anim_bench` measures **0.0020 ms/character/frame** at 100 characters
against the 0.0018 baseline — the extra accumulation is in the noise.

**What is still not right, and is NOT the convention.** Measured with
`--anim-pose t_pose`, so nothing is animating and frame lag is excluded, the
left thigh's direction is `(0, -0.9920, -0.1262)` live against
`(0, -0.9920, +0.1262)` in forward kinematics — Z negated, 14.5° apart. Both
frames report bit-identical root corrections, so it is not the mapping.

Visual verification (Front and Left, `--gizmos on --show-real-mesh off`,
`--anim-pose t_pose`) makes it plainer than the numbers did. Claim stated
first: *"the left arm is visibly raised toward the target with a bent elbow,
the right hangs at the side."* **It is not.** The base pose with no reach is
clean and symmetric — both arms even, both legs distinct — and the moment the
arm solve runs it is the character's **right** arm that lifts, from a target
that only ever sets `arm_ik.left`.

A left-hand target moving the right arm is a whole-rig mirror, not a
mis-aimed solve. The remaining suspect is the one thing the unit tests
explicitly say they do not cover: they build the hierarchy from `gltf_rig`'s
own parse of the asset, while the live app gets it from `bevy_gltf`'s
**loader**. A loader reproducing the bind pose with a different handedness
would pass every test here and still mirror exactly like this.

Worth recording as a shape of false progress: the 0.34 mm X figure above is
real and still does not mean the feature works. The probe target sat near the
centreline, where a mirror about X is nearly the identity — the measurement
improved for a reason unrelated to the thing it appeared to confirm. Had I
stopped at BRP numbers and skipped the screenshots, this would have shipped
as "fixed". Recorded on `AnimArmIk` as a self-contained next step.


### Arm IK: correct as a solver, not yet correct on screen

Item 5 of six, and the first entry here that has to report a **partial**
result. `src/character/anim/armik.rs` is a two-bone arm solver with 21 tests,
exact on the real rig's geometry. It does not yet render correctly, and the
reason is not in it.

**Three real bugs found on the way, each measured rather than reasoned about.**

*One was in shared code the legs have been using all along.* `aim_bone`
conjugated its world-space delta by the ancestors' accumulated rotation alone,
omitting the bone's own bind rotation — but forward kinematics composes
`... * bind_rotations[bone] * pose.rotations[bone]`, so the pose rotation sits
*inside* the bind. On the legs, bound within a few degrees of identity, the
omission is nearly invisible. On `LeftArm`, bound at **92.6°**, asking the elbow
to point straight DOWN swung the arm UP and landed **0.417 m** away on a
0.251 m bone. Fixed with `legik::rotation_frame_of`, which the foot-grounding
corrections and `lookat` now share — the ankle binds at −69.8°, so they were
measurably wrong too, just inside the tolerances their own tests asserted.

*The hinge axis has to be perpendicularized.*
`Quat::from_axis_angle(hinge, θ) * direction` only yields a vector θ from
`direction` when the hinge is perpendicular to it; otherwise it sweeps a cone.
The leg gets away with the raw axis because a foot target is nearly straight
down, already almost perpendicular to `+X`. An arm reaches in every direction,
including straight out along `+X` where the construction degenerates entirely:
the raw axis left the wrist **0.167 m** from a point well inside reach, and no
choice of axis or sign got below 0.084 m.

*The bend sign is the opposite of the leg's.* A knee leads with the joint and
trails the shin; an elbow trails the joint and swings the forearm forward. With
the leg's negation the elbow landed **0.213 m behind** the shoulder-to-target
line. All three fixes were verified by sabotage — reverting each one fails
exactly the tests that name it, and reverting the frame fix fails 8 including
the leg's own accuracy test.

**Measured facts about the real arm** (`puppet_base.gltf`), none of which the
synthetic rig would have revealed:

| bone | offset | what it actually spans |
|---|---:|---|
| `LeftArm` | 0.2097 m | clavicle to shoulder joint |
| `LeftForeArm` | 0.2511 m | **the upper arm** |
| `LeftHand` | 0.2436 m | **the forearm** |

The names are shifted a joint, exactly like the legs. The rest arm is
critically extended — 0.4947 m of reach carrying **0.2 mm** of slack, elbow at
176.5° — which is why the axis must be supplied. And `+Y` is the arm's own long
axis: rotating about it moves the wrist **0.000 m**, so picking it by analogy
with "the arms lie along X so the hinge is Y" yields an elbow that silently
does nothing.

The default softening came down from 0.02 to the leg's 0.005 because the
softening zone applies to targets *inside* reach too: at 0.02 a target at the
arm's own wrist came back 0.0015 m short, which a per-frame grip would compound
into visible inward creep.

**What does not work, and why it is being left.** The solve is exact and the
render is not. With a target at world (0.32, 1.15, −0.25):

- the solved pose puts the wrist at **(0.32000005, 1.1500001, −0.24999999)**;
- the live skeleton puts `hand_l` at **(0.075, 0.933, 0.053)** — 0.36 m off.

So something between the pose and the skeleton discards a correct solve. The
suspect is a convention mismatch: `accumulate_world_rotations` composes
`parent * bind * pose` (post-multiply) while `write_pose_to_skeleton` writes
`rest_rotation * bind⁻¹ * delta * bind`. `rig.rs` asserts in a comment that
these are "the same composition"; a probe measured **56–80° of disagreement**
on all four bones tried, legs included (`LeftArm` 56.1°, `LeftFoot` 67.9°,
`LeftLeg` 80.1°).

That probe is evidence, not proof — it built its retarget side from
`rig.bind_rotations` where the real path uses
`HumanoidSkeleton::rest_rotation`, so it compares against a *model* of the
retarget, and the legs demonstrably render correctly today. Settling it means
fixing the pose-space convention across FK, retarget and every IK consumer at
once, which is a larger and riskier change than adding an arm solver.
Recorded on `AnimArmIk` so the next reader meets the limitation before the
API.

### Look-at, distributed across the spine — and why there are no eyes

Item 4 of six. Rotating only the head produces the owl: a head that swivels
independently of a body that has not noticed. So a look is **distributed** —
`Spine1`, `Spine2`, `Neck` and `Head` each take a share of the total,
clamped to their own limits, with any share a clamped joint cannot absorb
offered to the ones above it. The shares sum to 1.0, so a look inside every
limit lands exactly on target.

**Eyes were investigated and deliberately not built.** `puppet_base.gltf`
has an `Eyes` node and it is a **skinned mesh, not a joint** — `mesh=1,
skin=0`, no children, and no eye joint among the skin's 65. The eyes are
geometry weighted to the head and physically cannot move independently.
Adding them would mean rewriting `JOINTS_0`/`WEIGHTS_0` binary data in a
720 KB `.bin`, which is asset surgery on a downloaded model; that was raised
and dropped rather than attempted.

**`Head` is a LEAF on both rigs** — 0.083 m above `neck_01` on the real one,
with no children. Rotating it moves no joint, so a gizmo view shows nothing
happening while the skinned head turns. That inverts this project's usual
verification rule: this one needs `--show-real-mesh ON`, and the numbers
came from BRP rather than from a picture.

**A look now eases from forward rather than snapping.** The first version
adopted a freshly-set target outright, which is right for a character that
should BEGIN a scene looking somewhere and wrong for the common case of one
noticing something mid-scene. `LookAt::settled_on` covers the former
explicitly.

Measured live, the y-component of each bone's rotation:

| bone | no target | looking |
|---|---:|---:|
| `Head` | 0.000 | **0.208** |
| `neck_01` | -0.000 | **0.163** |
| `spine_03` | -0.004 | **0.086** |

All three turn, the head most, and the whole thing is inert with no target.

### The run, and the flight-phase path nothing had exercised

Item 3 of six. `GaitParams::running()` is mostly parameters, but the
structural difference is the **duty factor**: below 0.5 the two stance
windows stop overlapping, so there is a moment with no foot down at all.
That is what separates a run from a walk — a fast walk is still a walk.

**The flight phase reached code no walk can.** `locomotion::stance_foot`
returns `None` there, and the walk-only code returned a zero velocity: a
dead stop twice per cycle. At a 0.4 duty factor flight is 20% of the cycle,
so at 1.5 strides/s the body would stall for **67 ms, twice a second**. A
body in flight is a projectile, so it now coasts at the velocity it left the
ground with — evaluated rather than remembered, which keeps `root_velocity`
a pure function of phase and preserves determinism.

**A centred difference straddling toe-off halves the velocity.** The real
find, and it took three wrong diagnoses. `root_velocity` differentiates
across `[phase - STEP, phase + STEP]`; at the last grounded instant one side
is in stance and the other airborne and barely moving, so the estimate comes
out at **exactly 1.99x** too small — 2.746 m/s inside stance against 1.378
at its edge. It reads as the body losing half its speed the moment a foot
lifts.

A walk never showed it because its stance windows overlap, so another foot
always takes the reference. The fix is a one-sided difference near a
boundary. Bisecting onto the boundary — the first attempt — made it *worse*,
by landing deeper inside the straddling window.

**A degenerate gait could publish 156 m/s.** `leg_phase` clamps a duty
factor to a 0.01 minimum rather than rejecting it, so "no contact" is really
a 1% stance sliver, and differentiating across it produced enough velocity
to fling a character across a level in one frame. Now bounded against the
leg's own reach and cadence, so the ceiling scales with the rig. A NaN duty
factor slipped through the first version of that clamp, because `>` is false
for NaN.

**A result worth recording:** the run demands **96.8%** of the leg's
straight length against the walk's 98.4% — *more* headroom despite a 1.7x
longer stride, because its deeper stance knee (0.35 against 0.20) more than
pays for it. Pinned as a comparison rather than an inequality.

Live at `--anim-speed 3.0`: both feet off the ground at once (0.188 and
0.183, against a standing 0.099), travelling **3.09 m/s** against the walk's
0.62.

### Turning, and stopping: the first two of six items toward a game-usable stack

Six gaps were identified between "an impressive tech demo" and "something a
player can control": turning, gait transitions, a run cycle, look-at, arm
IK, and ragdoll triggering. The first two are done.

#### Turning

`facing.rs` owns the heading as a **scalar yaw**, not a `Quat`. A character
on ground turns about one axis, and storing that as a quaternion makes two
easy things hard: "shortest way round" becomes a neighbourhood problem (the
`q`/`-q` hazard this project has paid for), and "how far is left" stops
being a subtraction.

`world_root_velocity` rotates the published velocity by the heading.
`root_velocity` is derived from hip-relative foot motion, so it is a vector
in the CHARACTER's frame — integrating it into a world position works only
for a character that never turns.

**The foot lock needed its own fix, and not the obvious one.** A lock pins a
foot to a WORLD point, which is right under translation and wrong under
rotation. Nor does it rescue itself: a 90-degree pivot sweeps a stance foot
about **0.156 m** for this rig's hip width, INSIDE the 0.25 m break
distance, so it stays locked and drags the whole way. `Turn` rotates the
anchor about the body's own centre, which is what a real planted foot does.

Two sign errors, both caught by tests. `yaw_of` had `atan2(x, -z)` on the
intuition that positive-about-up turns toward `+X`; deriving it instead,
`Ry(yaw) * (0,0,-1) = (-sin, 0, -cos)`, so the inverse is `atan2(-x, -z)`
and **a positive yaw points toward `-X`**. And a direction test used a
target of exactly `PI`, where both ways are equally short — a tie-break, not
a property, now pinned as one.

Live: at 1.2 m/s and 0.6 rad/s the character traces
`(-1.79, 0.95) -> (-0.06, 0.38) -> (-1.08, -1.12)` holding `y = 0.95`,
consistent with the 1.15 m circle those rates imply.

#### Stopping — and a plan refuted by measurement

This item was planned against two guesses, and **both were wrong**:

- *"Stopping leaves a foot mid-swing."* It does not.
  `GaitPhase::gait_frequency_hz` is `base + coefficient * speed`, and the
  base term is `1/7` Hz — so a character told to stop **keeps stepping
  forever** at 0.143 Hz. Measured: a foot creeping from `y = -0.093` to
  `-0.035` over two seconds of standing still. Worse than freezing.
- *"A speed change makes the pose jump."* The phase is continuous, so the
  pose does not jump. What steps is the **cadence**: 1.2 m/s to a standstill
  takes the clock from 1.223 Hz to 0.143 in one frame, an **8.6x**
  deceleration in a single frame.

So `transition.rs` is about rate, not about cross-fading poses — there was
nothing to cross-fade, the gait already being one continuous function of
phase. A smoothed cadence removes the 8.6x step; a weight that reaches zero
removes the endless stepping.

**A deadlock in the first design**, worth recording: the fade waited for a
footfall while the cadence decayed toward zero, so the phase stopped
advancing and the footfall never came. From phase 0.2 the cadence was near
zero within half a second and the fade never began at all. The fix is also
the better behaviour — the character walks its last step OUT rather than
stalling mid-stride.

**And a guarantee that hid its own mechanism.** `advance` snapped the
cadence to exactly zero once the weight ran out. That made the stop
untestable: reintroducing an idle-frequency floor in the decay left every
test green, because the end state was ASSIGNED rather than reached. Removing
the snap is what let `a_stopped_character_actually_stops` catch it, which it
now does with the exact frequency in the message.

Live: standing, `foot_l` holds `y = 0.099, z = -0.055` to three decimals
over six seconds. Walking is unchanged at ~0.62 m/s.

### Locomotion: a walk cycle, and root motion that does not fight a controller

Four phases, commits 665fb14 / 7dba064 / e9b484e / fb8f554 plus this one.
Everything in the leg stack — IK, foot locking, normal alignment, the
toe-end clamp, the pelvis drop, the offline de-slider — previously only ran
on a **standing** character. Foot locking exists for a moving one.

**`gait.rs` is a phase-parameterised pose, deliberately not another
`PhaseOscillator`.** That model is one sinusoid about one axis, and a leg is
not a sinusoid in three ways: stance is ~60% of the cycle against swing's
40%, the knee stays near-straight under load then folds ~50 degrees, and the
knee LEADS the thigh. Adding leg oscillators is twenty lines that appear to
work and produce the mirrored-pendulum walk.

**`locomotion.rs` publishes a velocity; a controller owns the position.**
That separation is what makes "no sliding by construction" and "does not
fight a character controller" compatible — the naive foot-drives-the-root
design conflates them. Nothing writes a `Transform`, so a controller may
accept, clamp, project or ignore the request; when it refuses, the feet
slide because the character IS being dragged, and the foot lock absorbs it.

#### Measured

| Quantity | Value |
|---|---|
| Travel per cycle | 1.04 m (0.52 m per step, ~1.04 m/s at 1 Hz) |
| Tracked-foot residual over a cycle | **0.0074 m**, worst frame 0.17 mm |
| Summed both-feet toe slide | 0.312 m, reduced to 0.042 m by the offline pass |
| Reach headroom | 0.8% -> **1.6%** (2.9x the IK's softening band) |
| Live knee flexion | 36.5 deg, leg at 95.0% of straight |
| Slope following | sole on the surface within 0.01 m at a 0.3 grade |

#### Five sign and frame errors, every one found by measurement

1. **The walk ran BACKWARD.** `stance.rs` documented `KNEE_AXIS` as
   "positive swings backward"; it swings forward. Thirty-odd tests passed
   against a reversed gait because they asserted angles, symmetry,
   continuity and ordering — all of which a backward walk satisfies. Nothing
   asserted a DIRECTION.
2. **The foot's sign is opposite to the leg's.** Every other bone hangs down
   (`-Y`) where positive-about-X swings forward; the foot points forward
   (`-Z`), where the same rotation lifts the toe. Written with the leg's
   convention the toe pitched down at footfall — tip 0.02 m below ground,
   ankle 0.12 m high.
3. **Root motion through `LocalPose::root_translation` walked the character
   STRAIGHT DOWN.** That field is routed through `hips_root_rotation` and
   divided by the rig's parent scale, both right for a hip displacement in
   the bind frame and wrong for world travel: on this Z-up rig it mapped
   forward onto `-Y`, sinking at 0.69 m/s against a published 0.72. Travel
   now moves the entity.
4. **Horizontal travel alone is not locomotion.** On a 0.3 slope the
   character held its starting height and ended 7.7 m under the hillside.
   Height is now sampled from the ground rather than integrated.
5. **Foot tracking picked the wrong foot.** Following the one planted
   LONGEST selects the foot about to lift, so the reference switches
   mid-difference and measures the gap BETWEEN feet: 245 m/s spikes, with a
   110 m/s lateral component — exactly a hip width — that gave it away.

#### Two measurement traps worth remembering

**A continuity test that could not fail.** It sampled with a 1e-4 step
against `Quat::angle_between`'s ~9.8e-4 precision floor — pure quantisation
noise. Fixing it exposed a 0.05 rad knee jump at every footfall. The same
measurement then showed two more traps: too wide a step stops being local
and reports curvature as discontinuity, and `angle_between` is unsigned so a
curve through its minimum reads as exactly zero.

**The synthetic rig cannot show what the real one does.** Its lower leg is a
0.07 m stub against a 0.459 m shin, so 42 degrees of knee flexion moves the
sole 0.050 m there and the ankle 0.329 m on a real rig — 6x. A correct walk
renders with a visibly straight leg in any synthetic-rig preview. The same
gap then appeared for bind ROTATIONS: the real rig binds its foot at -69.8
degrees where the test rig left it at identity, so a foot that sat flat in
every test was visibly pitched in the game.

**That gap is now closed by parsing the asset rather than transcribing it.**
`gltf_rig.rs` reads `puppet_base.gltf` at test time — a `.gltf` keeps its
node hierarchy in plain JSON and only mesh data in the `.bin`, so 69 nodes
with their rotations and translations are readable with `serde_json`
(already in the tree via `bevy_gltf`) without Bevy, the asset server or a
GPU.

Faithfulness check: parsed rest-pose foot pitch **26.6 degrees** against
**26.1** measured on the live rig over BRP. Drift check: flattening
`foot_l`'s rotation in the asset fails two tests with exact messages. The
three modules that each carried their own hand-copied copy of the leg
lengths now share one parsed helper.

**The retargeting path is now covered too**, which an earlier version of
this entry said needed the live app. That was wrong: of the five
`HumanoidSkeleton` methods the path uses, only `entity` touches the ECS, and
a spawned entity per bone is cheap. The claim conflated "needs a Bevy
`Query`" with "needs the running game".

So `retarget`'s tests now exercise `for_other_rig`,
`hips_local_translation_for` (including a 0.01 parent scale, overridden
because `puppet_base` is unit-scaled and the test asserts that first),
`delta_in_bone_frame`'s conjugation, and `write_pose_to_skeleton` through a
headless `World` with `MinimalPlugins` + `TransformPlugin` — the rig renders
upright, human-scale and mirrored under real Bevy propagation.

Two things that had to be right for those to mean anything. The conjugation
test first used a delta about **X**, which the leg chain's own bind rotation
is also about — parallel rotations commute, so it was vacuously passing;
about **Y** it bites, verified by disabling the conjugation. And the
propagation test needs the rig's `-90` degree correction ON THE ROOT ENTITY:
without it the offsets and bind rotations are both glTF-local, so the rig
renders lying down (head y 0.017 against foot y 0.088 — correct for a Z-up
rig, wrong for a Y-up world).

What still needs the live app is the **glTF loader** itself: these parse the
asset's JSON, so they verify the maths against the bind pose the file
declares, not that `bevy_gltf` reproduces it on load.

#### Cleared, not fixed

A ~26 degree nose-down foot was reported from a screenshot. Measurement
cleared the gait: **standing still** the rig pitches the foot 26.1 degrees
nose-down and walking is 22.2 — the gait slightly improves it.
`relaxed_stand` contributes only 4.6 degrees; the rest is the artist's bind
pose, and `legik`'s normal alignment deliberately adds only the ground's
difference from level so it never flattens an authored pose.

807 tests, clippy clean.
### Cleanup: `relaxed_stand_v2` and `src/bench` deleted

Both were carried as "open items" that were really just unused code.

**`relaxed_stand_v2`** was exploratory output from testing the drag fixes —
a pose with no Mixamo provenance that nothing consumed. It served its
purpose (proving the studio could author and save a pose) and keeping it
would have left the named-pose set carrying a pose nobody intended to use.
`idle_stand` takes its place in `standing_poses()`, so the shape
assertions it was covering still run — against a pose with *reproducible*
provenance rather than hand-tuned numbers.

**`src/bench`** (530 lines) had no consumer since the examples that used
it were replaced. An earlier entry below records the decision to keep it
documented rather than revive it; deleting it is the same judgement taken
further, now that it is clear nobody is going to write the renderer
example that would bring it back. `examples/anim_bench.rs` remains the
animation harness. Removing it also freed the `image` crate dependency —
`src/bench`'s screenshot RMSE was its only user.

Both deletions left the test count unchanged (791 / 692), which is the
point: neither had coverage to lose.

### Correction: the joint-limit "chatter" was stale, and one real effect remains

**790 → 791 tests.**

This log carried an open concern that a joint driven into its rotation
limit sat in a bounded-but-noisy limit cycle at ~6.5 rad/s. **Re-measured:
it does not.** A pinned joint holds at **0.062 rad/s**, parked at exactly
30.00°, and a sweep of avian's `swing_compliance` from 0 to 1e-3 changes
nothing because there is nothing to absorb.

That number was measured *before* `PdParams::stable_damping` and the frame
fixes landed. It was then carried into the notes and repeated as an open
question long after the thing it described had been fixed — the test's own
bound stayed at `< 5.0 rad/s`, wide enough to hide the improvement.

Tightened to `< 0.5`, plus an assertion that the joint is actually *at* its
limit rather than passing by never reaching it.

**What is real**, and now documented where it belongs: a joint holding a
**mid-range** target does oscillate. A body's collider is centred on its
segment, so its centre of mass sits half a bone from the joint anchor, and
every commanded angular acceleration also demands a linear motion the point
constraint cancels — which the controller re-commands next step. It
persists with damping switched off entirely (3.2 rad/s at ζ=0) and scales
with the anchor offset (0.10 rad/s anchored at the centre of mass, 13 at
0.15 m), so it is a torque/constraint interaction rather than a control-law
defect.

A joint at its limit is quiet for a reason that follows directly: there the
*constraint* holds it, and a constraint does not re-command itself.

**At shipping ceilings it is much smaller** than the probes suggested —
2.6 rad/s on an arm, 2.2 on a neck, 4.0 on a hip, against 11 at the 2000
rad/s² used to make the mechanism measurable. The dominant failure there is
**undershoot**, not noise: the arm reaches 6.8° of a 15° target, which is a
weak joint running out of authority. `the_shipped_joints_are_calm_at_their_own_ceilings`
now pins that, because a bound chosen at test scale says nothing about the
rig anyone will actually use.

### Phase 8, part one: the pose editor, with viewport dragging

An in-engine pose editor behind the `anim_studio` feature. **674 → 722
tests** (48 new, all under the feature; CI must build both ways or the
gated code rots).

Pose data is judged by eye, and this project's history is that the judging
is where the time goes — two poses once shipped with limbs 59-77% off the
rig's real bone lengths, past a green suite, caught only by a screenshot.
Hot-reload already removed the recompile between a tweak and seeing it;
this removes the rest.

**Structure** mirrors `ragdoll`/`ragdoll_plugin`: `edit` and `drag` are the
model and the maths as plain values and plain functions, fully unit-tested
with no egui, no window and no mouse; `save` is RON I/O with real error
types; `pose_editor` and `drag_plugin` are thin wiring. The interesting
decisions are testable without standing up the machinery they run inside.

Bones are edited as **axis + degrees**, the same decomposition the RON
format stores. Nobody reads `(0.0, 0.0, 0.581, 0.814)` as "71 degrees about
Z", and an editor whose numbers are unreadable barely beats the text file.
The editor keeps its own `EditableRotation` per bone because
`to_axis_angle` always returns a non-negative angle, so a round trip can
flip the sign a user is looking at.

**Viewport dragging.** Grab a joint and pull; the bone follows. Grabbing a
joint rotates its **parent** — a joint's position is set by its parent's
rotation, so steering the elbow bends the upper arm. The aim is composed in
the parent's frame (a world-space arc onto a local rotation is the "right
angle, wrong axis" bug this project has hit before), with an explicit
antipodal guard because pulling a limb through its own pivot is a natural
drag and an undefined arc. Overlapping joints tie-break by depth, nearest
first: on a front view a hand can sit over a hip, and the one you can see
is the one you mean.

#### Three bugs, all caught by the mandatory screenshot

1. **Opening the editor destroyed the pose.** The studio's default is the
   REST pose and the panel writes its edit onto the rig, so the first frame
   replaced a `relaxed_stand` character with a T-pose. Opening an editor
   must never modify the thing being edited. The fix needed a second pass:
   the rig spawns asynchronously from a glTF, and the first version marked
   adoption done against a still-empty query.
2. **Drag handles floated off the upper body.** They were computed by
   running forward kinematics on this crate's *synthetic* T-pose
   proportions, while the character on screen is a real retargeted mesh
   with its own. The legs happened to line up and the arms did not — the
   kind of partial wrongness that reads as "close enough" at a glance. Now
   read from the rig's own `GlobalTransform`s, the same ground truth the
   skeleton gizmos use, so a handle lands on its joint by construction on
   whatever rig is loaded.
3. egui's default font has no glyph for `→`, so the mirror buttons rendered
   as replacement boxes.

#### Three more bugs, all found by actually using it

The editor was usable enough to produce `relaxed_stand_v2` — the first pose
in this project authored by dragging rather than by converting reference
data. Getting there surfaced:

4. **Dragging never ran at all.** `Query<(&Camera, &GlobalTransform)>`
   matches two entities: the scene camera and the internal view Bevy's
   shadow mapping creates. `single()` failed every frame and the system
   returned immediately — while the handles kept drawing from a *different*
   system, so nothing looked wrong. The same trap that once made the
   gallery's entire egui UI render to an invisible camera. Confirmed live
   via BRP before fixing.
5. **Clicking a joint jerked it; dragging spun it.** The drag re-read its
   inputs from the live rig each frame. Every input is now snapshotted at
   the grab, plus a `grab_offset` so a click with no mouse movement is a
   no-op. (A first diagnosis — that re-reading the joint's *position*
   compounds the arc — was tested and **disproved**; that loop converges.
   The real cause was the parent frame drifting as retargeting re-ran.)
6. **Vertical drags moved the joint the wrong way.** A pose stores a
   rig-independent *delta*, and `retarget` wraps it twice —
   `rest_rotation(bone) * (bind⁻¹ · delta · bind)` — so the frame it acts
   in is `parent_world · rest_rotation · bind`. Passing only `parent_world`
   inverted one screen axis while leaving the other correct, exactly the
   reported symptom.

   Bug 6 had been **invisible to the test suite because a test was
   validating it**: `aiming_works_in_the_parents_frame_not_the_world`
   reconstructed the world direction with the same wrong convention the
   code used, so it agreed with the bug and passed. Replaced with one that
   pushes the answer through `forward_kinematics`' own composition
   verbatim, on the real retargeted fixture — a synthetic rig has identity
   rest rotations and is structurally blind to the whole class.

### Phase 8, part two: spring tuning

Per-bone DHO tuning with a live step-response plot. **739 → 752 tests.**

The pose says *where* a character goes; the springs say *how it gets
there*, and that is the whole difference between a heavy brute and a quick
duellist reading from identical pose data. It is judged by feel, so it has
to be adjusted while the thing moves.

- **Four presets** — Heavy, Default, Quick, Floaty. Each *scales* the
  rig's existing grading rather than stamping one value across it: a
  uniform half-life is what makes a character read as a puppet, because
  every joint arriving at once is the one thing real bodies never do. A
  test enforces that every preset keeps spine slower than extremities.
- **Scoped edits** — bone, chain, or whole rig. Chain is the useful
  default, since a limb's feel comes from the whole chain rather than one
  joint.
- **A step-response plot**, which is the point. Whether a spring overshoots,
  by how much, and how long it rings is the thing a damping-ratio number
  cannot convey; choosing one without seeing its curve is guessing. The
  plot's vertical range expands to contain the peak, so overshoot is
  visible rather than clipped off the top — the failure mode that would
  make an underdamped spring look identical to a critically damped one.

Tested where it matters and not where it does not: presets are ordered by
speed, only the documented ones overshoot, a chain scope reaches the hand
but not the other arm, a critically damped curve never exceeds its target
and an underdamped one does, and the curve's shape does not change with
the plot's duration.

### Phase 8, part three: the reference-data pipeline

`tools/dump_animation_pose.py` upgraded and closed into a working loop.
**752 → 755 tests.** A real pose, `idle_stand`, now comes straight from
`assets/models/idle.glb` — the first with *reproducible* provenance, since
its source positions are committed beside it.

```
blender --background --python tools/dump_animation_pose.py -- \
    assets/models/idle.glb --frame 0 --ron /tmp/idle.positions.ron
cargo run --release --example import_reference_pose -- \
    /tmp/idle.positions.ron assets/anim/idle_stand.pose.ron
```

Also added: frame ranges (`--start/--end/--step`), local rotations
(`--rotations`, diagnostic), and per-frame toe speed (`--velocities`) for
offline contact annotation — 0.0001 m/s on a standing idle, correctly
reading as planted against `FootLockConfig`'s 0.15 m/s threshold.

#### Positions, not rotations — the first thing that was wrong

Dumping Blender's local rotations directly into a `.pose.ron` was built,
tried, and **is wrong**. A bone's local rotation there is relative to
*Mixamo's bind pose*; a `LocalPose` stores a delta relative to *this
crate's T-pose*. The numbers transfer cleanly and mean something else on
arrival — measured, the left hand landed 0.54 m **above** the shoulder.
The same class as the once-live "arms overhead" retargeting bug.

So Blender reads the clip and Rust does the maths: the dump emits world
positions, which carry no reference frame to get wrong, and
`convert::pose_from_world_positions` derives the rotations — the path that
produced `relaxed_stand` in the first place.

#### Three coordinate bugs, each caught by a screenshot

The conversion took three corrections, and **each looked right until
rendered**:

1. **Z-up to Y-up.** Caught by the numbers.
2. **The forward-axis sign.** "This crate faces −Z, Blender's +Y is
   forward, so +Y becomes −Z" is the obvious inference and is wrong. The
   character stood with its head tilted back staring at the sky — at the
   same 41.2° as `relaxed_stand`, with the axis negated. Settled against
   the data instead: `relaxed_stand`'s source had `Neck` at z = +0.0681
   where Blender reports y = +0.0731, so +Y maps to **+Z**.
3. **The X mirror.** Mixamo puts the character's left on +X; this crate
   puts it on −X. Without negating, the import arrived mirrored — shoulders
   swapped, feet splayed, body reading as turned. Caught only after the
   neck fix made everything else look plausible.

Note that mirroring X is a *reflection*, so it inverts handedness: an
axis-angle rotation keeps its reflected axis but reverses its angle. The
RON export routes through positions specifically so that subtlety cannot
reach an imported pose.

#### The test that makes the pipeline trustworthy

`relaxed_stand` and `idle_stand` come from the **same clip frame by
completely independent routes** — the first through the superseded
position-space module, the second through the new tooling. A test asserts
they describe the same posture (within 12°, loose enough for the old
pose's documented hand-editing) and that the arms are not swapped.

Two independent derivations agreeing is worth far more than either
matching a number chosen by hand — and it makes the coordinate conversion
permanently regression-tested, which is exactly what took three attempts.

### Phase 8, part four: clips and the timeline

The rewrite had **no clip type at all** until now — every pose was a single
frame, by design. Stages 1 and 2 already produce continuous, non-repeating
motion from one authored pose, which is why this arrived last rather than
first. What they cannot produce is a *sequence*: a footfall pattern, a
wind-up and release, anything whose shape over time is the thing being
authored.

**755 → 774 tests.** `src/character/anim/clip.rs` is the model (16 tests,
no egui); `studio/timeline.rs` is the panel.

- **Sparse keyframes of whole poses**, slerped per bone with
  neighbourhooding before the interpolation — the recurring quaternion bug
  in this project, and a blend is exactly where it bites. A test asserts a
  rotation blended with its own negation does not move the bone.
- **Contacts are stepped, never interpolated.** A foot is planted or it is
  not; Stage 3 cannot act on a half-planted foot. The lane drawing and
  `contacts_at` share that rule, so the picture cannot show a foot planted
  during a span the runtime treats as lifted.
- **Dragging a keyframe reports where it landed**, because a drag can
  reorder the clip and a UI holding an index would otherwise silently
  select a different keyframe.
- Interpolation cannot stretch a bone — asserted across 21 samples of a
  clip, since a blend produces rotations nobody authored and is where an
  incorrect one would surface.

`--studio-timeline` and `--studio-demo-clip` exist for verification: an
empty timeline renders its chrome and proves nothing about keyframes,
scrubbing or contact lanes, which are most of what the panel is. With the
demo clip the screenshot shows 3 keys over 1.60 s, correctly spaced, with
the L lane running full width and the R lane stopping partway — the
authored weight shift, and proof the lanes read real data rather than
drawing a fixed bar.

### Phase 8, part five: phase oscillators and IK effectors — Phase 8 complete

**774 → 790 tests.**

#### The phase-oscillator editor

Each oscillator is four numbers and none means much alone: amplitude is
radians on a bone whose visible motion depends how far down a limb it
sits, a harmonic of 2 reads as "peaks at each footfall" rather than "twice
as fast", and an offset of `PI/2` is the difference between hip sway
reinforcing spinal twist and cancelling it.

So the panel plots every oscillator on **one axis**. What is being tuned is
how the waves sit against each other, and a wave in isolation says almost
nothing. The plot scales to the largest amplitude present, so a two-degree
breath beside a twenty-degree sway stays a small wave rather than
flattening to a line. Amplitudes are edited in degrees — nobody judges
"is this sway too big" in radians.

Opening the phase panel necessarily un-suspends the procedural animation
the studio otherwise freezes while authoring; it is the one panel whose
subject *is* that motion.

#### IK effectors

Grab a hand or foot and the whole limb solves, wrapping the shipped
`solve_two_bone` rather than writing a second solver — an editor computing
poses by different code than the runtime is how the two drift apart.
Effector tips draw larger and orange, so it is visible *before* clicking
which handles pose a limb and which rotate one bone.

**One real bug, and a test that hid it.** The chain pivots at `upper`, not
at `root`: a shoulder or hip socket is a fixed attachment the solve never
rotates, sitting 0.14 m from where the limb actually pivots on this rig.
Measuring target distance from `root` added that offset to every solve and
landed exactly that far short.

It took an embarrassing number of passes to find, because *the test made
the same mistake* — computing its target from `root` too, so it was asking
for points genuinely outside the chain's reach. Two errors partly masking
each other, which is why the symptom looked like an imprecise solver and
survived several wrong fixes (an iterative refinement loop, an explicit
child lookup) before measurement showed the distance was already exact and
only the reach was wrong.

**Phase 8 is complete.** The studio covers pose editing with viewport
dragging, spring tuning, clip authoring, phase oscillators, and IK
effectors; the reference pipeline turns a Mixamo clip into a loadable
pose.

### Stage 4 completed: physics now reaches the rendered skeleton

`RagdollSet::ReadBack` had been declared and empty since Phase 6 — the
simulation ran, the PD controller worked, and nothing downstream read the
result. The character rendered its kinematic pose regardless, making Stage 4
an expensive no-op. Test count **669 → 674**.

#### Two frame bugs, found by asking the rig instead of guessing

Wiring read-back was blocked on a prior question: the driven bodies visibly
sagged instead of tracking. Reading back a wrong pose would only have made
the bug more visible.

BRP against the live ECS settled it in one query. `Hips` — the kinematic,
pinned root — matched its target **exactly**, while `Spine` one joint down
was wildly off. That proved `publish_joint_targets` was correct and moved
the search downstream. A direct comparison then showed `target-vs-BONE` at
**0.0° for every bone in the rig**: the targets were perfect, and the bodies
were in the wrong frames.

1. **`spawn_bone_body` oriented each body along its own segment**, not in
   its bone's frame. For a bone whose bind rotation differs from its
   parent's those are different things — 93° apart on the knees, 46° on the
   ankles, ~40° on the shoulders, while the spine and arms happened to
   agree. Fixed by building the capsule from explicit endpoints so the
   *collider* carries the alignment and the body's rotation means exactly
   one thing.
2. **`connect_bodies` anchored each joint in the bone entities' frames**,
   but a body sits at its segment's midpoint — half a bone away. Every
   constraint pulled toward the wrong point. Both passes of `spawn_ragdoll`
   now derive the body centre from one shared helper, so agreement is
   structural rather than a convention two call sites must remember.

Settled tracking error with gravity off: **177° → 15°**.

Three hypotheses were killed by measurement first: gravity (177° error
persisted with it disabled), torque ceiling (64× more torque only reached
93°), and joint limits (154° without any). Each was cheap to test and each
would have been a plausible-sounding wrong answer.

#### The read-back itself

`read_back_simulated_pose` walks parent-before-child inverting the world-
rotation accumulation (`local = bind⁻¹ · parent_world⁻¹ · world`), so a
bone with no body keeps its animated rotation and a partial ragdoll renders
correctly. Per-joint strength selects what is *shown*, inverted from the
torque path: `slerp(animated, simulated, 1 - strength)` displays the
simulation exactly where the controller has stopped enforcing the animation.

Verified per the mandatory protocol, Front and Left, claims stated first: a
limp ragdoll's skeleton visibly **collapses into a heap**; a fully driven one
**stands correctly**, indistinguishable from the kinematic render.

`JointTarget` and `Bone` are now `Reflect`, so the rig is queryable over
BRP — which is what made the diagnosis quick and is worth keeping.

**One test caught mid-writing:** the read-back tests initially passed
through a world where read-back never ran, because the headless harness
hand-registers systems rather than adding the plugin. Fixed, then confirmed
non-vacuous by disabling the read-back and watching the limp test fail.

### Ragdoll: joint limits, a full-rig spawn, and two real bugs

Closing the three items Phase 7 left open. Test count **654 → 669**.

**Joint limits shipped.** `JointLimits` (a swing cone plus a twist range,
the shape avian's `SphericalJoint` already solves) with a per-bone
anatomical table. Swing-twist rather than three Euler ranges because Euler
limits on a ball joint are order-dependent and gimbal-lock, so a limit that
reads right in one pose silently means something else in another.

Deliberately generous: these are *anatomical stops* that prevent a knee
bending backwards, not a pose authoring tool. Shaping motion inside the
range is the controller's job, and a tight limit means the controller lives
pressed against a constraint — the configuration Phase 6 flagged as risky.

#### The `kd · dt` finding

Phase 6 deferred limits because "a PD driving against a constraint is a mild
analogue of the two-rotational-springs instability". Building the test found
a real defect, though not that one.

A jointed body driven to a reachable target **vibrated at 12 rad/s**, and
the vibration got *worse* with more damping:

| ζ | `kd·dt` | chatter |
|---|---|---|
| 0.0 | 0.00 | 3.2 rad/s |
| 0.5 | 0.98 | 10.0 rad/s |
| 1.0 | 1.96 | 12.3 rad/s |
| 4.0 | 7.85 | 16.9 rad/s |

Damping that amplifies oscillation is not damping. This is the textbook
explicit-integration bound `kd · dt < 2`, and at avian's 64 Hz default with
ζ = 1.0 the ceiling lands at ~10 Hz — where the **shipped hip and spine
joints already sat**, at 98% of the limit. The 10 Hz hip was measurably the
worst-behaved joint in the rig, overshooting a 15° target to 29°.

Two fixes: `PdParams::stable_damping(dt)` clamps the gain at runtime, and
the heavy joints dropped from 9–10 Hz to 8 Hz. "Corrects harder" is what
`max_torque` expresses — the hip's ceiling is still 20× the neck's —
whereas frequency is how fast the correction is *integrated*, a property of
the solver. Raising it past what the step can carry makes a joint unstable,
not strong. `every_default_joint_is_well_conditioned_for_the_physics_timestep`
now fails loudly on a future edit that reaches for a higher frequency.

Several wrong hypotheses were killed on the way: limits (bit-identical
with and without), torque ceiling (present at 70 and at 2000), and substep
count (helps, does not fix). Each was measured rather than reasoned about.

#### The ragdoll was falling out of the world

`spawn_ragdoll` builds a complete simulated skeleton — 17 bodies, the 22
bones minus 5 leaves — from a live `HumanoidSkeleton`, so it works on a
retargeted glTF as well as the synthetic rig.

Wiring it into the gallery immediately exposed something no headless test
had: the bodies were at **y = −1930 m**, every joint correctly oriented,
the whole assembly in free fall. The PD controller drives *rotation only*;
nothing was driving position.

`a_full_ragdoll_holds_itself_together_under_gravity` passed throughout,
correctly — a ragdoll falling as one connected body keeps its spread
constant. It took a screenshot to see, and BRP against the live ECS to
confirm. `RagdollSpawnConfig::pin_root` (default on) fixes it, with
`a_full_ragdoll_does_not_fall_through_the_world` as the cheap version of
that screenshot and `an_unpinned_ragdoll_is_free_to_fall` proving the
switch is real in both directions.

A second bug caught before it ran: `simulated_segment_child` originally
delegated to `Bone::chain_continuation_child`, which only names a child for
the two *multi-child* bones and returns `None` for every single-child one —
a ragdoll of exactly 2 bones out of 22.

#### `src/bench` is renderer-only, and stays that way

Not revived. It is built around a windowed render loop — scripted cameras,
screenshot RMSE, per-frame wall-clock — which is the right shape for the
SDF pipeline and the wrong shape for CPU work: a windowed frame time is
vsync-capped and reports ~16.7 ms regardless of animation cost. Scope note
added to its module doc; `examples/anim_bench.rs` remains the animation
harness. Merging them would produce one that answers neither question
honestly.

**Resolved by the entry above:** `RagdollSet::ReadBack` had no system, and
the driven bodies sagged rather than tracking. Both are fixed; the sagging
turned out to be two frame bugs, not a tuning problem.

### Phase 7 — `src/character/muscle` deleted (7,158 lines)

The position-space mass-spring solver is gone. `src/character/anim` is now
the only animation stack, and the `--anim-backend` A/B switch that carried
the cutover has been removed along with it.

**Test count: 759 → 654.** The 106 removed tests were the solver's own
(`solve_muscle.rs`'s 23, plus pose/retargeting tests for code that no
longer exists). No test was deleted that covered surviving behaviour.

**What had to be preserved, and how.** Two real runtime dependencies ran
from `anim` back into `muscle`:

1. `poses.rs` *converted* `relaxed_stand` and `wave` from the
   position-space tables **at startup**, every launch. Those values trace
   back to real Mixamo `idle.glb` reference data, so they could not simply
   be re-eyeballed — this project's own history is that hand-guessed pose
   offsets ship 59-77% wrong past a green suite. The conversion output was
   frozen into `assets/anim/*.pose.ron` (the same files the asset loader
   hot-reloads) and embedded with `include_str!`, making the files the
   single source of truth for both paths.

   **Verified before deleting**, not after: a temporary test asserted the
   embedded RON reproduced the live conversion to within 1e-5 on every
   bone of both poses, and was confirmed to fail loudly (naming the exact
   bone) when a pose file was perturbed by 2°.

2. `convert.rs`'s conversion-fidelity tests read the same tables. Their
   input is now a frozen `RELAXED_STAND_TARGETS` fixture, guarded by a
   test asserting every bone appears in it exactly once.

**A new risk the deletion created, and its guard.** The pose rotations
were previously *derived*, so their provenance was enforced by
construction. Frozen into a file, they became bare literals that nothing
validated — precisely the setup that has failed here before. Added
`relaxed_stand_still_matches_its_reference_data`, pinning the pose against
the real Mixamo world positions it came from (tight 1 cm bound on the
spine chain; 15 cm on the arms, which inherit the shoulder displacement
rotation space provably cannot express). Confirmed non-vacuous: a 10°
corruption moves the hand 18 cm and fails.

**One test written and then rejected as worthless.** An initial
"compiled-in poses match the files on disk" test could not fail — both
sides parse the same bytes through the same loader, so cargo rebuilds them
in lockstep. Checked by inverting `PoseAsset::to_local_pose` and watching
it still pass. Replaced with a load test that asserts what it can actually
observe (the file parses, names only real bones, and yields a posed rig),
with the limitation written into its own doc comment.

**Gallery changes.** `draw_muscle_debug_gizmos` became
`draw_skeleton_debug_gizmos`: the white joint chain and yellow rest
markers survive (both read the bones' real `GlobalTransform`s), the
magenta velocity layer is gone with `MuscleSim`. The egui panel's dead
muscle dials were replaced with live pose switching, gait speed, ground
slope, and spring tuning. Three `MuscleSim` HUD helpers collapsed into
`worst_bone_length_error`, which measures the rig against **its own first
frame** rather than the synthetic T-pose constant — the real mesh is
uniformly scaled, so comparing against the constant would report a large
permanent error on a perfectly correct rig.

That readout is now a **structural alarm rather than a convergence
check**: a rotation cannot stretch a bone, so anything above float noise
means something is writing translations into the chain. It reads
**0.00000 m** live.

**Verification.** Front and Left, `--gizmos on --show-real-mesh off`,
claims stated first: both forearms and hands visible hanging at the sides;
both knees softly bent; both feet on the ground plane. All three hold, and
the rig is positionally bit-identical to the pre-deletion baseline (the
only diffs are 1-2° on spine/neck rotations, which is the breathing
oscillator — the baseline drifts by the same amount between its own
frames).

**A documentation trap found while doing this.** Gizmos are depth-tested,
so the skinned mesh hides the entire joint chain running inside it.
`--gizmos on` alone shows only the few markers past the silhouette, which
reads as a broken overlay and — worse — can be mistaken for a verified
skeleton view while showing almost nothing. AGENTS.md's rule 3 now
requires pairing it with `--show-real-mesh off`.

**Docs updated:** AGENTS.md's mandatory verification rules pointed at three
deleted paths and at `MuscleSim` as ground truth. The
`character-animation/` knowledge tree now records what migera actually
built (it claimed the project was "still in its first, static-T-pose
milestone"), and the Lugaru document carries a status note so it is read as
prior art rather than as current code.

**Still open:** `src/bench` remains orphaned — `examples/anim_bench.rs`
was written standalone rather than reviving it, since that revival is its
own task. Joint limits ship as `None` (Phase 6's note stands). No example
spawns a full 22-body ragdoll yet.

## Leg IK, the last three gaps against Holden's recipe (commits 66739b0, ecc3b3f)

`src/character/anim` implemented the article's leg-IK recipe in Phase 3 but
left three of its steps unbuilt: the toe-end re-orient, foot alignment to
the ground normal, and the pelvis vertical adjustment. `GroundHit::normal`
had been computed by `SlopedGround`, carried through the whole stack, and
read by **nothing** — a foot on a ramp stayed level and buried its heel.

**The toe end came from the rig, not a guess.** `puppet_base.gltf` turns
out to have the joint the plan assumed was missing: the chain is
`foot_l → ball_l → ball_leaf_l`, with the leaf 0.0789 m past a 0.1591 m
toe — a ratio of 0.496, hence `TOE_END_FRACTION = 0.5`. It stays out of the
`Bone` enum, so every `[T; 22]`, every RON asset and the glTF resolution are
untouched; `RigGeometry::with_toe_end` takes a measured offset where a rig
has one.

**Measured numbers.**

| Quantity | Value |
|---|---|
| `LeftFoot` pitch, flat ground (live) | −84° |
| `LeftFoot` pitch, 0.4 slope (live) | −60° |
| Tilt alignment adds (unit A/B) | 0.304 rad = slope × 0.8 blend |
| Toe tip vs surface, alignment off | −0.015 m (sinking) |
| Toe tip vs surface, alignment on | +0.049 m (clear) |
| Rig's permanent reach shortfall, flat | 0.0108 m |
| Pelvis drop, flat ground | 0.000 m |
| Pelvis drop, 5 cm dip | 0.060 m (at cap) |
| Pelvis drop, 0.12/0.25/3 m drop | 0.000 m (ledge) |

**Four wrong diagnoses, each killed by measurement rather than argument.**
The alignment overshoot was blamed in turn on double rotation, cross-frame
accumulation, and rig feedback; instrumenting the solver showed it
receiving *identical, correct* input every frame — the defect was in the
test's measurement, which compared sloped against flat ground and so
conflated alignment with a different leg solve. Comparing alignment-on
against alignment-off on the **same** ground gives 0.304 rad, exactly as
designed. Separately, the "0.035 m tip sink" recorded in the first commit
was measured mid-fix and no longer existed once alignment landed.

**Two bugs that only a real rig exposes.** The toe-end offset must live in
the *toe's* frame, not its parent's — identical on the synthetic rig
(identity bind rotations) but `puppet_base` binds `ball_l` at ~180°, which
points the tip back into the heel. And `reach_margin` as "treat the leg as
shorter" lowered the hips on level ground forever, because the rig is
authored at exactly critical extension and carries a permanent 0.0108 m
shortfall on every surface; it is now a deadband that gates the correction
without scaling it.

**A boundary worth knowing, opposite to the naive expectation:** a 5 cm dip
lowers the hips, while 0.12 m, 0.25 m and 3 m do not. Past
`FootLockConfig::max_contact_height` a surface is a ledge, the foot keeps
following the animation, and there is no shortfall to correct.

**Verification.** Every new behaviour was sabotage-tested — alignment
disabled fails 4 tests, the pelvis gate fails 5, the toe-end frame
conversion fails exactly 1 while the other four toe tests pass either way
(they are blind to it, which is why the targeted one exists). 726 tests,
clippy clean. Live: `LeftFoot` −84°→−60° across the slope change, hips
unchanged at 0.95 on flat ground.

**Still open at the time of this entry:** the article's offline PBD
foot-sliding removal. Built in the next entry below.

## Offline foot-sliding removal (PBD over a whole clip)

The last unbuilt piece of the article's recipe, and the only one that is not
a runtime system: `src/character/anim/slide.rs`. The runtime foot lock is
causal — it sees only frames that have happened, so it can pin a foot but
cannot know the pin will need to be elsewhere in forty frames. An authored
clip has every frame available at once, so the error can be distributed
across the whole contact and baked in at zero runtime cost.

**Three constraints, relaxed by Gauss-Seidel sweeps** over arrays of pelvis
and toe world positions:

1. **Contact coherence** (`hard_factor` 0.9) — consecutive in-contact frames
   pull toward their shared midpoint, clamped to the ground. This removes the
   slide.
2. **Motion preservation** (`soft_factor` 0.05) — everywhere else, each frame
   pulls toward reproducing the *original* frame-to-frame offset. Without it
   the solve collapses the animation to a single motionless pose, which
   satisfies constraint 1 perfectly.
3. **Limb length** (`soft_factor`) — pelvis and toe preserve their original
   separation, so pinning a foot is not satisfied by an infinitely long leg.

**Measured.**

| Quantity | Value |
|---|---|
| Toe travel, posed clip, before → after | **0.2900 m → 0.0057 m** (98% removed) |
| Solver's own slide metric | 0.2900 → 0.0033 m |
| Worst leg-length change, constraint on | 0.0043 m |
| ...constraint off | 0.0427 m (10x worse) |
| Slide after 1,000 sweeps | 0.0124 m |
| Slide after 25,000 sweeps | 0.0033 m |

**The iteration budget is mostly waste.** The article specifies 25,000
sweeps. Measured: sliding is already at 0.0124 m after **1,000**, and the
remaining 24,000 refine it to 0.0033 m — 25x the work for a 4x improvement
on a quantity invisible either way. A synthetic clip does hit the 1e-4
movement threshold at ~4,070 sweeps, but a real posed clip never does; the
tail converges asymptotically. Both numbers are recorded rather than the
budget being taken on faith.

**Every constraint has a test proven to bite.** Each was disabled in turn:
contact coherence fails 3 tests, motion preservation 1, limb length 1. Two of
those tests only bite after being rewritten —

- The motion-preservation term is a *restoring force*: it pulls toward "the
  neighbour's current position plus the original offset", which for
  unperturbed input IS the current position, so a free swing in isolation
  exercises nothing. It is only observable where contact pulls against it,
  and the effect concentrates entirely in the seam frame (measured 0.140 m
  jump with the term disabled, against an authored 0.03 m step).
- The limb-length test originally allowed 0.12 m, roughly 28x too loose to
  notice the 0.0427 m stretch its own constraint prevents.

**Reachable, not just implemented:** `remove_foot_sliding` is wired to a
"Remove foot sliding" button in the studio timeline, which reports the
before/after slide next to it rather than merely claiming success.

**Deterministic**, asserted bit-for-bit across two runs. It matters more for
a bake than for anything at runtime: a pass that drifted between runs would
make an authored clip depend on when it was exported.

845 tests with `--features anim_studio`, 746 without; clippy clean in both.

