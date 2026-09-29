# Hybrid Renderer Rewrite — Progress Log

Tracks the ground-up rewrite of `src/hybrid` (see its module doc comment
and `src/hybrid_legacy/mod.rs`'s doc comment for why this restart
happened). Append new entries at the top, newest first. Each entry should
be small enough to write in one sitting — one feature/step, not a whole
session's worth of unrelated work batched together.

## How to use this file

- **One entry per proven-correct step.** A step is "proven correct" once
  it has a CPU-testable reference (mirroring `src/hybrid_legacy/cpu_ref.rs`'s
  pattern) with passing `cargo test` cases, AND has been visually verified
  in the actual renderer (`examples/gallery.rs`) — not just claimed from
  reading the code. Don't log a step until both are true.
- **Record real performance numbers**, not impressions. Use the `--bench
  SECS` flag convention to get `p50_ms`/`p99_ms`/`fps` rather than
  eyeballing the on-screen FPS graph. Note the scene/camera-angle/
  object-count the number was measured under — a bare number with no scene
  context isn't comparable across entries.

  Note the shared `migera::bench` harness that produced the older numbers
  below has since been **deleted**: the examples using it were replaced and
  it sat unused. Entries recorded against it stay as historical record;
  reproducing them would mean rebuilding the harness first. For CPU-side
  work see `examples/anim_bench.rs`, which measures headlessly — a windowed
  frame time is vsync-capped and says nothing about CPU cost.
- **Link commits.** Each entry should reference the commit(s) that landed
  it, so the log stays a pointer into git history rather than a duplicate
  of it.
- **Note regressions and open questions too**, not just successes — if a
  step revealed a tradeoff (like the legacy renderer's shadow-candidate-
  margin fix costing real frame time) that's exactly the kind of thing
  this log exists to make visible before it's forgotten.

## Baseline (legacy renderer, for comparison)

The frozen `src/hybrid_legacy` renderer's last-measured numbers, as a
comparison point for the new implementation — not a target to beat by any
particular margin, just context. Measured via
`examples/gallery_legacy.rs --obj sphere --bench 3` at `--at 25.0`
(close camera angle, sphere fills a large fraction of the frame):
`p50 ≈ 85ms (11.75fps)`. At the default orbit angle, `p50 ≈ 19ms (52fps)`.
Frame cost is highly angle-dependent in this renderer (more on-screen
shadow-affected pixels costs proportionally more).

## Baseline (new `hybrid`/`gallery.rs`, empty scene)

Floor cost of the fresh-start `examples/gallery.rs` before any SDF
rendering exists — just the empty Bevy scene (camera, window,
`HybridRenderPlugin` with an empty `build()`) plus the FPS HUD/graph and
its own once-a-second sampling/logging overhead. This is what future
per-feature entries in the Log below should be measured against, so a
feature's real cost (not this baseline) is visible.

Measured via `./target/release/examples/gallery --at-frame 100000` (no
`--shot`, so the screenshot capture's own cost doesn't pollute the
sample), `AutoNoVsync`, on the dev machine's integrated GPU (AMD Radeon
Graphics / RADV RENOIR), reading five consecutive once-a-second log
lines once fps had stabilized:

```
fps 248  frame 4.40 ms  avg 4.0  min 3.2  max  6  p25 3.6 p50 3.9 p75 4.1 p95 4.9 p99  5.5 iqr 0.5
fps 227  frame 4.39 ms  avg 4.3  min 3.3  max  7  p25 3.9 p50 4.2 p75 4.4 p95 5.1 p99  5.5 iqr 0.5
fps 211  frame 4.06 ms  avg 4.7  min 3.2  max 15  p25 3.8 p50 4.2 p75 4.7 p95 7.0 p99 10.5 iqr 0.9
fps 223  frame 4.42 ms  avg 4.4  min 3.5  max  6  p25 4.1 p50 4.3 p75 4.7 p95 5.3 p99  5.5 iqr 0.6
fps 197  frame 5.01 ms  avg 4.9  min 4.1  max  6  p25 4.6 p50 4.8 p75 5.1 p95 5.7 p99  5.9 iqr 0.5
```

Roughly `p50 ≈ 4.0-4.8ms (200-250fps)`, occasional outlier frames up to
~15ms (OS/scheduler jitter, not renderer cost — nothing is drawn yet).
This is pure engine/UI/window overhead; landed in commits `5aacc9c`
(FPS HUD + `--shot`/`--at-frame`) and `be99cd4` (disable vsync,
once-a-second debug log).

## Log

### Physics bug fix: sphere's single center-point sample couldn't detect nearby rounded corners

User-reported ("sphere not bouncing and rolling after felt down to the
ground") after the gravity fix above made the example's timing correct
enough to actually notice the sphere sitting frozen near the floor's
rounded corner instead of grazing/rolling across it as the milestone's own
doc comment describes.

**Root-caused with a standalone trace** (a scratch example calling
`contacts::generate_contacts`/`collision_static::query_static_point`
directly at the sphere's exact resting coordinates from
`physics_playground.rs`) rather than guessing from the visual symptom:
the sphere's center (`x=5.3`) sits on the FLAT region of the floor's top
face (the flat/rounded transition is at `x=5.6`), even though the
sphere's own 0.75 radius means its actual surface extends out to `x=6.05`
— well into the rounded corner. Querying the floor's SDF gradient AT the
sphere's center (not at its surface) reported a purely vertical normal
`(0,1,0)` at every point along the whole fall and rest, because the
gradient at that specific point has no way to "feel" curvature the query
point itself never touches.

**This traces back to Stage 3's own original `Sphere` sampling design**:
`sample_points_local` used a single center point for spheres, reasoning
"the analytic radius handles surface offset" — true for computing
penetration DEPTH (`contacts.rs`'s `own_local_distance` correction), but
not for the contact NORMAL, which comes from the OTHER body's gradient
evaluated at wherever the sample point happens to be. A center-only
sample can sit arbitrarily far (in angle) from where the sphere's actual
surface is nearest to interesting geometry.

**Fix** (`src/physics/sample_points.rs`): `Sphere` now samples a
spherical-Fibonacci set of TRUE SURFACE points (reusing the machinery
already built for `Ellipsoid`, generalized to take a `count` parameter),
the same "sample where the surface actually is" technique every other
shape already used. `contacts.rs`'s `own_local_distance` depth-offset
trick still applies uniformly (now correctly a near-no-op for every
shape, since every sample is a true surface point) — no special-casing
needed once the sphere stopped being the one shape violating that
invariant.

**A second, smaller issue surfaced immediately by the fix's own tests**:
with only 12 sample points (`Ellipsoid`'s existing count), no point
reliably lands near the true deepest-penetration direction between two
overlapping spheres, so contact depth was systematically underestimated
(measured ~0.375 vs. the true 0.5 for a specific two-sphere overlap test)
and the Jacobi solver's resting equilibrium stalled short of "exactly
touching." Raised to a separate `SPHERE_SAMPLE_COUNT = 32` (spheres are
the most common dynamic-body shape in practice, so under-sampling them
matters more than under-sampling ellipsoids) — chosen empirically, high
enough that the solver's own convergence test passes within a reasonable
substep budget. The remaining small discretization gap (deepest contact
depth/normal still approximate, not exact, for any finite sample count)
is inherent to the technique and reflected in the affected tests'
tolerances rather than chased to zero.

**Verified**: `physics_playground.rs` re-screenshotted at frames
40/100/200/400 — the sphere settles into a position reflecting genuine
corner interaction (shifted from its drop position, unlike the previous
frozen-in-place behavior) and the pyramid settles fully by frame 40
(consistent with the now-correct gravity speed). All existing
depth/normal-exactness test assertions were reviewed and updated to
reflect the sample set's inherent (and now well-understood) discretization
tolerance rather than an idealized exact-coverage assumption — this
wasn't a test regression, it was tests encoding an assumption (single
exact center sample) that was itself the bug.

`cargo test --release --lib`: 267 passed, 0 failed (up from 266 — net
+3 new tests, several existing ones revised in place to reflect correct
multi-sample-point sphere behavior). `cargo clippy --release --lib
--examples`: no new warnings.

### Physics bug fix: gravity on unconstrained (no-contact) bodies was ~substep-count times too weak

User-reported ("it's very slow"/"spheres falling like low gravity") after
watching `examples/physics_playground.rs` live — the sphere and pyramid
appeared to settle far more slowly than real 9.81 m/s² gravity should
produce. Root-caused with a standalone numeric check (a headless Bevy app
driving a lone falling sphere with `TimeUpdateStrategy::ManualDuration` for
a fixed 1/60s tick, compared frame-by-frame against the analytic
`y(t) = y0 - 0.5*g*t²` free-fall prediction) rather than guessing from the
visual impression alone.

**Root cause**: `solve_rigid::solve_substep_jacobi`'s velocity update was
`linear_velocity = (corrected - position) / dt` — the textbook XPBD
formula, but applied unconditionally even when a body had ZERO contacts
this substep. With no contacts, `corrected == predicted == position`
(nothing accumulated a correction), so the formula computed
`velocity = 0 / dt = 0` — **silently resetting velocity to zero every
single substep** for any body not currently touching something.
`solve_world`'s own gravity integration (`velocity += gravity * dt;
position += velocity * dt`) still ran once per substep and correctly
advanced position each time, but the velocity that produced that
advancement was thrown away immediately after by
`solve_substep_jacobi`, so it could never compound frame to frame — each
substep re-started from the previous substep's OWN gravity increment
alone rather than an accumulating velocity. Net effect: with
`SUBSTEPS = 8`, a free-falling body's effective acceleration was roughly
an order of magnitude too weak (worse over more elapsed time, since real
free fall accelerates quadratically and this bug capped velocity to
whatever one substep's gravity increment alone produced).

This is exactly the class of bug `src/hybrid`'s CPU-ref-first doctrine
exists to catch, and technically the module's own existing test
(`a_body_with_no_contacts_stays_exactly_where_it_was`) SHOULD have caught
it — but that test started from zero velocity, so "reset to zero" and
"correctly preserved" were indistinguishable outcomes. A real gap in test
design, not just in the implementation: asserting a value stayed
unchanged is only a meaningful regression check when the value being
checked was non-trivial to begin with.

**Fix** (`src/physics/solve_rigid.rs`): `solve_substep_jacobi` no longer
derives velocity from a position delta at all. It now ADDS the
correction's own implied velocity (`correction / dt`) on top of whatever
velocity the body already had, and does nothing at all (`continue`) when
a body has zero accumulated contacts this substep — preserving both
position and velocity exactly as the caller's own gravity integration
left them. Two new regression tests: one with a nonzero initial velocity
and no contacts (confirms preservation, not just "stayed near its
starting position" which zero-velocity tests can't distinguish from a
reset), and one driving the exact integrate-then-solve substep sequence
`solve_world` uses, confirming velocity after one full frame of free fall
matches `g * dt` within 1e-3 — the literal symptom this bug produced,
turned into a permanent regression check.

**Verified end-to-end**: re-ran the standalone free-fall check — position
after 1.85s of fall is now within ~0.3 units of the analytic prediction
(previously off by ~19 units, i.e. barely fallen at all). Re-screenshotted
`examples/physics_playground.rs`'s pyramid milestone (spawned slightly
above the floor, not touching) at frames 40/100/300: it now visibly falls
and settles onto the floor by frame 40 and stays stacked through frame
300, instead of drifting down over a much longer, physically-wrong
timescale. No change to resting/settled behavior once bodies are in
contact — that path was already correct, since it always had nonzero
`acc.count` and therefore never hit the buggy zero-contact branch.

`cargo test --release --lib`: 266 passed, 0 failed (up from 264 — 2 new
regression tests). `cargo clippy --release --lib --examples`: no new
warnings.

### Physics stage 3 follow-up: spatial-hash broad-phase, re-scoped ahead of the GPU solver port

Direct response to stage 3's own profiling finding (below): O(n²) all-pairs
contact generation, not BVH refit, is the real bottleneck at scale. Rather
than proceeding straight to a GPU substep-solver port that would still hit
that same O(n²) wall at exactly the body counts a GPU port should unlock,
re-scoped to build the broad-phase fix first.

`src/physics/broadphase.rs`: a uniform spatial hash — Müller's canonical
formulation (Ten Minute Physics, "Blazing Fast Neighbor Search with
Spatial Hashing," the same author as the XPBD paper this project already
cites), same GPU-friendly technique NVIDIA FleX/PhysX GPU rigid bodies and
Macklin & Müller's "Unified Particle Physics for Real-Time Applications"
(SIGGRAPH 2014) use, chosen over sort-and-sweep (its sweep step is
inherently sequential, a worse GPU fit) and a from-scratch GPU LBVH
(Karras 2012 — real and proven, but far more implementation machinery
than this project's body counts justify) for the same reason Jacobi was
chosen over graph-coloring earlier: lowest complexity that's still
genuinely parallel throughout. Structure mirrors the eventual GPU
algorithm exactly (counting sort: hash → count → prefix-sum → scatter,
CSR bucket layout) per `src/hybrid`'s CPU-ref-first doctrine, and
deliberately does NOT materialize a separate candidate-pair list — each
body walks its own 27-cell neighborhood and hands candidates straight to
`contacts::generate_contacts`, sidestepping GPU dynamic-array/atomic-
append complexity entirely (confirmed a legitimate first-version choice,
not a shortcut needing rework later, since per-candidate contact
generation is already cheap). Static colliders are deliberately excluded
from the hash entirely — folding a 6-unit floor into a grid sized for a
0.3-unit dynamic sphere would force the floor to span hundreds of tiny
cells; the existing (already-cheap) dynamic-vs-static loop is untouched.

Caught a real bug via its own tests: at very small body counts (`n=2`),
`table_size = n*2 = 4` buckets caused genuine hash collisions across
distant, non-overlapping cells (confirmed by two failing tests expecting
empty/exact results). Fixed with a `table_size` floor of 256 — cheap at
any realistic body count, large enough that small scenes don't see
pathological collision rates. The module's own most-important test
(`every_true_overlapping_pair_in_a_random_cluster_is_found`, a brute-force
O(n²) cross-check against the hash's own candidate results for 50 random
bodies) passed on the first try — the property that actually matters
(never silently missing a genuinely close pair, which would produce a
body falling through the floor with no error at all) held throughout.

`src/physics/solve_world.rs`: `generate_all_contacts` now builds a
`SpatialHash` from dynamic body positions each substep (cell size = 2x
the largest dynamic body's own bounding radius, Müller's own sizing
convention) and queries candidates per body instead of an O(n²) double
loop; dynamic-vs-static contacts remain a small, direct loop. An `i <
j`-only guard prevents each dynamic pair from being processed twice (the
hash reports candidates symmetrically) — caught by a dedicated test that
initially over-asserted (expected exactly 1 contact per overlapping
sphere pair, not accounting for `generate_contacts`' own bidirectional-
by-design sampling, which legitimately produces 2 contacts per pair from
one call) before being corrected to check for exactly 2, not 4.

**Verified end-to-end**: the stage-3 pyramid milestone re-screenshotted at
frame 300, pixel-identical to the pre-broad-phase version — the fix is a
pure performance change, no behavioral difference. **Scaling verified**
with a temporary scratch scene (built, measured, deleted — same one-off
convention as the BVH profiling measurement below): the 2000-dynamic-body
scene that previously never completed 200 frames within a 60s timeout
(the O(n²) wall) now completes all 200 frames in ~30s at a steady
~190-210ms/frame. Still far from real-time at this body count — per-
candidate `generate_contacts` cost across 8 substeps is the next natural
cost center, not something this fix claimed to solve — but the specific
O(n²) explosion this stage set out to fix is confirmed gone.

`cargo test --release --lib`: 264 passed, 0 failed (up from 256 — 8 new
tests: `broadphase` x6, `solve_world` x2 replacing its prior 2).
`cargo clippy --release --lib --examples`: no new warnings.

### Physics stage 3: rigid-vs-rigid contacts, Claybook sample points, XPBD Jacobi substep solver, BVH-under-motion profiled

Builds on stage 2's static-world collision (below). The stacking-stability
milestone: a small pyramid of `RoundedBox` crates settles and stays
stacked, resolved through the same solver as dynamic-vs-static contacts —
no separate code path for "resting on the floor" vs. "resting on another
box."

`src/physics/sample_points.rs`: fixed local-space sample-point sets baked
per `PhysicsShape` kind (Claybook — Dennis Gustafsson/Sebastian Aaltonen,
GDC 2018 "GPU-Based Clay Simulation and Ray-Tracing Tech in Claybook"):
`Sphere` → single center point; `RoundedBox`/`BoxFrame` → 8 corners inset
by `corner_radius`; `RoundedCylinder`/`HexPrism`/`Capsule` → end-cap rings
plus axis endpoints; `Ellipsoid` → a spherical-Fibonacci set. The critical
test (`every_*_sample_lies_on_the_surface`) checks every sample point is
actually on its shape's surface within epsilon — caught a real geometry
bug immediately: the cylinder ring sampler placed points at the *inset
core rectangle's corner* instead of offsetting them outward by
`edge_radius` onto the true rounded surface (off by exactly `edge_radius`
in the failing test's own output) — fixed by mirroring `RoundedBox`'s own
"inset core, then offset along the corner's normal by the rounding radius"
pattern.

`src/physics/contacts.rs`: `generate_contacts` queries each body's sample
points against the OTHER body's SDF (both directions — A's samples vs. B's
field, then B's samples vs. A's field, catching contacts either body's own
sampling alone might miss, e.g. a small corner poking into a large flat
face). Caught a second real bug: naively treating the raw SDF query as
"the contact depth" is wrong whenever a sample point isn't a true surface
point relative to a zero-radius convention — `Sphere`'s single sample is
deliberately its CENTER (not a surface point, since its own radius already
handles the offset), so two overlapping spheres produced a raw query of
`+0.5` (not penetrating) for centers that actually overlap by `0.5`. Fixed
by subtracting the sampling shape's own local distance at that sample
point (`own_local_distance`) from the raw query — zero for true surface
samples (box corners, capsule rings), exactly `-radius` for the sphere
case, giving the true separation either way. Both bugs were caught by this
stage's own tests before ever reaching the solver, exactly the payoff
`src/hybrid`'s CPU-ref-first doctrine is meant to produce.

`src/physics/solve_rigid.rs`: `solve_substep_jacobi` — every contact
independently computes a positional correction (split by relative inverse
mass) and scatters it into a per-body accumulator; a second pass divides
by contact count and applies the average, then derives velocity from the
position delta (same XPBD convention stage 2 already established). Chosen
over graph-coloring Gauss-Seidel per this project's own SOTA research:
Jacobi+atomics needs no coloring/graph structure at all, making it the
only approach implementable correctly on a first pass by a small team —
graph coloring is deferred as a legitimate later optimization, not
required for v1. The CPU reference deliberately mirrors the GPU
algorithm's accumulate-then-divide STRUCTURE, not just "a" correct
solver, so it gives the eventual WGSL port something meaningful to test
against (per `src/hybrid`'s CPU-ref-first doctrine — the reference must
model the actual algorithm). `SUBSTEPS = 8` (Müller et al.'s own suggested
4-10 range) — not yet tuned against a measured stability/cost tradeoff;
solver tuning is explicitly flagged as future work, not a one-shot pick.

`src/physics/solve_world.rs`: the real per-frame step, superseding stage
2's `solve_static_collisions` now that dynamic-vs-dynamic contacts exist —
a stacked pyramid needs floor contacts and box-vs-box contacts satisfied
by ONE shared Jacobi accumulator (two independent single-contact solves
would fight each other). A static collider is just an ordinary body with
`Inertia::STATIC` (zero inverse mass) fed into the same contact generation
and solve — no separate code path. Broad-phase is plain O(n²) all-pairs,
deliberately not reusing `hybrid::bvh::Bvh` (entity-indexed, tightly
coupled to the renderer's own object list/extraction lifecycle — not a
drop-in fit for physics' own body indexing); acceptable at this stage's
scale, revisit only if profiling shows it's a real bottleneck.

**BVH-under-motion profiling milestone** (mandatory per the staged plan,
not optional): temporarily instrumented `bvh::update_persistent_bvh` with
`Instant` timers (removed before this commit — same one-off measurement
convention the original refit-cost entry below used) and ran a scratch
scene of falling spheres with real per-frame translation (not
gallery.rs's rotation-in-place `--stress` scenes, which this project's own
prior entry already flagged as not equivalent to physics-style motion).
Measured refit cost: **~10-31µs at 150 bodies**, **~50-130µs at 1000
bodies** (both after an expected first-frame full-build outlier). Not
directly comparable in absolute terms to the existing 2.3-3.7ms baseline
(measured at 10,000-20,000 objects), but the qualitative finding holds:
real per-frame translation does not cost meaningfully more per object than
the rotation-in-place motion the original baseline measured — the refit
algorithm's cost is dominated by tree size, not by which `Transform`
fields actually changed. **Real bottleneck discovered instead**: this
stage's O(n²) all-pairs contact generation becomes the dominant cost long
before BVH refit does — a 2000-dynamic-body scratch test became too slow
to reach 200 frames within a reasonable timeout, while 1000 bodies ran at
real-time speed. This is the already-flagged, deliberate v1 scope
limitation (`solve_world.rs`'s own doc comment), now confirmed as a real
practical ceiling around low thousands of simultaneously-colliding dynamic
bodies rather than a theoretical one — worth reaching for the existing BVH
as a broad-phase prune (or a spatial hash) before body counts approach
that range, not before.

`examples/physics_playground.rs`: added a 3-2-1 pyramid of `RoundedBox`
crates (spawned already stacked and aligned — this stage's solver applies
positional-only corrections, no torque yet, so an already-toppling drop
isn't a scenario it's expected to recover from), alongside the unchanged
stage-2 sphere-on-corner milestone. Screenshotted at frames 40/100/300/900
— the pyramid stays stacked and stable throughout (settling only slightly
as it beds in), while the sphere continues its independent stage-2 arc
without interference between the two.

`cargo test --release --lib`: 256 passed, 0 failed (up from 233 — 23 new
tests across `sample_points`, `contacts`, `solve_rigid`, `solve_world`).
`cargo clippy --release --lib --examples`: no new warnings.

### Physics stage 2: static-world collision — a sphere falls, rolls across a rounded corner, and settles on a static floor

Builds on stage 0-1's foundations (below). First real physics behavior:
`src/physics/collision_static.rs` wraps `hybrid::cpu_ref::local_distance`/
`local_normal` — the SAME per-`Shape` SDF evaluator the renderer already
uses to march and shade every frame — into `query_static_point` (world<->
local convention mirrors `cpu_ref::march_object` exactly) and
`query_nearest_static` (linear scan over static colliders; acceptable at
v1 scale, the existing BVH is available as a later broad-phase prune if
profiling ever shows it's needed). `src/physics/solve_static.rs` adds
`PhysicsGravity`, `bounding_radius` (per-`PhysicsShape` sphere-probe radius
— exact for `Sphere`, a conservative approximation for every other shape
until stage 3's Claybook-style multi-sample-point contact generation
replaces this entirely), and `solve_body_static`, which predicts a
position, corrects it against the nearest static collider's clearance
(`distance - contact_radius`), and derives velocity from the actual
position delta — the XPBD convention (Müller et al., "Detailed Rigid Body
Simulation with Extended Position Based Dynamics").

**Two real bugs were caught during this stage's own development, both now
covered by regression tests, not just fixed silently:**

1. **Velocity/position inconsistency near curved surfaces.** An earlier
   version corrected position against the constraint but then separately
   zeroed the PRE-correction velocity's inward-normal component —
   inconsistent with the position correction actually applied. Near a
   rounded corner, where the contact normal changes direction from step to
   step, that inconsistency injected a small spurious tangential velocity
   every frame; individually negligible, it compounded over hundreds of
   frames into the sphere sliding across the entire curved region and off
   the floor's edge. Fixed by switching to the XPBD convention described
   above (position correction and velocity derivation must use the exact
   same delta, never computed separately) — confirmed via a 1200-frame
   settling+long-run-stability regression test
   (`a_sphere_resting_on_a_flat_face_near_a_corner_stays_settled_indefinitely`)
   that the original 600-frame settling test alone was too short to catch
   (it stopped right as the slide was only beginning to accelerate).
2. **Division by a zero `dt` on the very first frame.** Bevy's
   `Time::delta_secs()` is exactly `0.0` on the first `Update` tick (no
   elapsed wall-clock yet — the same fact stage 1's own
   `physics_integrate_placeholder` test already pinned). The XPBD velocity
   derivation (`(corrected - position) / dt`) divides by that zero `dt`,
   producing NaN/Inf immediately. That NaN poisoned the affected body's
   `Transform` the very next frame, which poisoned `bvh::update_persistent_bvh`'s
   persistent tree (NaN propagates unpredictably through the refit's
   min/max unioning) — observed, while debugging the example scene, as the
   ENTIRE static floor rendering fully black (every shadow ray reporting a
   false hit against the NaN-poisoned tree), not just the affected sphere,
   and not self-correcting on later frames since the BVH refits in place
   rather than rebuilding from scratch. A `dt <= 0.0` guard now makes a
   zero-`dt` step a complete no-op; regression test
   `a_zero_delta_time_step_never_produces_nan_or_infinite_velocity`.
   Worth flagging for later stages: ANY future physics system deriving
   velocity from a position delta divided by `dt` needs the same guard —
   this isn't specific to the static-collision solver.

Also worth noting given how surprising the debugging path was: a **convex
rounded corner is not a stable resting spot for a frictionless
point-contact sphere** (no local minimum — same as a marble balanced on a
dome) — this is correct physics, not a bug, and the example scene (see
below) is deliberately built around it rather than fighting it.

`examples/physics_playground.rs` updated: a static `RoundedBox` floor
(`half_extents=(6,0.5,6)`, `corner_radius=0.4`) plus a `RigidBody` sphere
(`radius=0.75`) dropped just past the floor's rounded corner, so it visibly
grazes and rolls across the curved region while falling before settling on
the flat top — screenshotted at frames 40/90/400 via the existing
`--shot`/`--at-frame` convention, confirmed visually identical (settled, no
drift) across all three.

`cargo test --release --lib`: 233 passed, 0 failed (up from 221 — 12 new
tests: `collision_static` x6, `solve_static` x6). `cargo clippy --release
--lib --examples`: no new warnings.

### Physics stages 0-1: GPU-native SDF physics foundations, closed-form inertia, and authoritative-transform plumbing

First steps of a new, separate effort: a GPU-native rigid/soft-body physics
engine operating directly on `sdf::components::Shape` geometry (distance +
gradient queries against the SDF itself), rather than adopting a CPU engine
like Rapier/Avian with its own collider representation — decided after an
explicit build-vs-buy comparison. Full staged plan lives in this session's
plan file; summarized here as each stage lands, same as the hybrid rewrite
above.

**Stage 0** (`src/physics/components.rs`): `RigidBody` (linear/angular
velocity), `Inertia` (inverse mass + inverse inertia tensor diagonal, XPBD's
own convention — `0.0` means infinite mass/static, no separate "is this
static" branch needed anywhere downstream), and `PhysicsShape` — the
collider-shape subset of `Shape` with **no `RoundedCone` variant at all**.
`RoundedCone`'s distance function has a real, tracked, pre-existing
correctness bug (`hybrid::cpu_ref::local_distance` already `unimplemented!()`s
on it, and it has no GPU shape-kind tag either — see that function's own
doc comment). Physics must not build mass/inertia/collision math on a shape
whose own distance function is known-wrong, so the exclusion is enforced at
the type level via `TryFrom<Shape>` rather than a runtime panic branch —
code that exhaustively matches `PhysicsShape` simply cannot mishandle
`RoundedCone`, because there is no such arm to write.

**Stage 1** (`src/physics/inertia.rs`, `src/physics/integrate.rs`,
`examples/physics_playground.rs`): closed-form diagonal inertia tensors
(standard rigid-body-dynamics formulas) for every `PhysicsShape` variant —
sphere, box, cylinder, capsule (cylinder + two hemispherical caps, each
hemisphere's own `(3/8)r` center-of-mass offset applied via the parallel
axis theorem, not naively treated as sitting at the sphere center it was cut
from), ellipsoid — plus `combine_parallel_axis` for later compound-body use.
`RoundedBox`/`RoundedCylinder` are approximated by their un-rounded base
shape (no exact closed form for the rounding's contribution; a standard,
documented approximation for the corner/edge radii this project's scenes
use). Cross-checked against known analytic results (unit sphere `I = 0.4`,
a two-point-mass dumbbell converging to the textbook `2md²` parallel-axis
formula as point radius shrinks) rather than only self-consistency checks.

Separately, `physics::integrate::PhysicsPlugin` proves the scheduling
plumbing end-to-end with a deliberately dumb placeholder (constant-velocity
translation, no gravity, no collision — that's Stage 2): registered in
`Update`, `.before(bvh::update_persistent_bvh)`, matching
`src/hybrid/motion.rs`'s `PreUpdate` (snapshot) → `Update` (this system) →
`PostUpdate` (Bevy's own transform propagation) ordering contract. Visually
verified in the new `examples/physics_playground.rs` (kept separate from
`examples/gallery.rs` so physics debug scenes don't entangle with the
renderer-feature gallery): a single `RigidBody` sphere with
`linear_velocity = (1, 0, 0)`, screenshotted at frames 40 and 180 via the
same `--shot PATH --at-frame N` convention `gallery.rs` already uses, shows
the sphere moving from screen-x≈498 to screen-x≈843 — confirms `Transform`
mutation from an ordinary physics system is picked up by BVH refit and
per-frame extraction with no special-casing, exactly like any other animated
`Shape` entity (e.g. `gallery.rs`'s own `spin_objects`).

No `--bench` numbers yet — no collision/solver cost exists in these two
stages to measure; that starts at Stage 3 once the GPU solver lands, with an
explicit mandatory profiling milestone comparing BVH refit cost under real
per-frame motion against this file's existing near-static baseline.

`cargo test --release --lib`: 221 passed, 0 failed. `cargo clippy --release
--lib --examples`: no new warnings from any physics file or the new example.

### Film grain: root-caused the "colored speckling" review complaint, added a small deliberate chroma component

Direct follow-up to the "missing rendering systems" review's second item:
external reviewers reported "colored/chromatic speckling" in shadow
regions despite `hybrid_post.wgsl`'s own grain math being, by direct code
inspection, already monochromatic (a single scalar hash added identically
to all three color channels).

**Root-caused before touching any code** (research-backed, not guessed):
read the actual shader and confirmed chromatic aberration samples R/G/B
from the CLEAN pre-grain image (grain is added once, afterward, to the
already-recombined color) — the "CA re-samples an already-grained
texture at three different positions, decorrelating a monochromatic
signal into apparent per-channel noise" failure mode is real in general
(a legitimate compositing-order bug class) but structurally impossible in
this specific file's actual pass ordering. The likely real explanations:
(1) this pass's own `target_format` comes from `extracted_view.
target_format` — the real swapchain format, typically an `*Srgb` variant
on desktop — meaning an identical additive LINEAR delta produces
different *visible* magnitude changes across channels sitting at
different absolute values, because sRGB's encode slope varies with input
level (a real, if subtle, contributor); (2) JPEG chroma subsampling in
whatever screenshot/chat/bug-tracker pipeline the reviewers actually
viewed the images through — a real, well-documented artifact of lossy
compression on high-frequency noise, entirely separate from renderer
correctness.

**The fix isn't purely "stay monochromatic," per photographic/imaging
literature**: real color negative film has three physically separate
emulsion layers, each with its own independent silver-halide crystal
population — genuinely NOT monochromatic grain, unlike B&W film. Real
digital sensor noise has a real (smaller) chrominance component from
demosaicing amplifying each channel's own white-balance gain
differently. AV1's own film-grain-synthesis spec and DaVinci Resolve's
own professional grain tool both model a SMALL, luma-correlated (not
independent, not identical) chroma-noise component as a deliberate
realism feature, not an oversight to eliminate.

**Implementation** (`hybrid_post.wgsl`, `src/hybrid/grain_ref.rs`): two
additional hash samples (different additive salts from the luma sample,
so they're independent draws of the same hash function, not scaled
reuses of the luma value), blended toward the luma sample at
`CHROMA_LUMA_COUPLING = 0.5` (correlated, not independent), then applied
as each channel's OWN DEVIATION from luma (not the chroma sample's raw
value — a real bug caught by the first version of this function's own
CPU-ref test: perturbing by the raw chroma value injected a nonzero
per-channel offset even when the chroma "noise" happened to exactly
equal the luma noise, breaking the intended "reduces to exactly
monochromatic when chroma agrees with luma" guarantee; fixed by
perturbing by `chroma_sample - luma_sample` instead), scaled by
`CHROMA_GRAIN_RATIO = 0.2` (kept deliberately small — matches the
literature's own "chroma noise stays subtle, never dominant" convention).

**Verification**: 4 new CPU-ref tests in `grain_ref.rs` (identical
samples reproduce exactly monochromatic output — this is what caught the
raw-value-vs-deviation bug above — distinct chroma samples produce
genuinely distinct per-channel output, the chroma contribution stays
bounded/small relative to luma even at maximal divergence, and the
G-channel's negative correlation with R/B holds). 208/208 lib tests
total (up from 204), clean `cargo clippy --release --lib --examples`,
`wgsl_parse`'s `hybrid_post_wgsl_parses` unchanged (still passing).
Real-GPU screenshot at 10x the default grain strength (0.15 vs. the
shipped 0.015 default, for visibility) confirms real per-pixel color
variance in the noise texture rather than uniform gray grain, without
looking garish or dominant — reverted to the real default before
committing.

### Stochastic (jittered-lens) depth of field: SOTA real-time DOF via a dedicated resample ray, no primary-ray jitter

Direct follow-up to the "missing rendering systems" review: depth of
field had no implementation at all (uniformly sharp front-to-back).
Researched current real-time DOF state of the art before implementing
(tile-based CoC-max scatter-as-gather — Unreal/Unity/AMD FidelityFX's
own approach — vs. Bevy's own shipped 2-pass hexagonal-box-blur
approximation vs. stochastic/jittered-lens ray sampling, the classic
distributed-ray-tracing technique, Cook/Porter/Carpenter 1984). Chose
stochastic ray jitter: this renderer is a raymarcher, not a rasterizer,
so it can fire a genuinely different ray per frame instead of
approximating blur with a 2D kernel — every classic DOF artifact
(foreground/background bleeding, bokeh-shape faking, halos) is avoided
structurally rather than mitigated, because each accumulated sample is a
real raymarched hit through a real simulated aperture offset, not a
weighted average of already-flattened pixels.

**Real design mistake caught before landing (see `dof_ref.rs`'s own
module doc comment for the full account): the first implementation
attempt jittered `hybrid_trace.wgsl`'s own PRIMARY ray.** That ray's own
hit point/depth feeds `out_depth` (Bevy's real depth test), the existing
GI temporal accumulator's disocclusion test, and the reflection/
refraction accumulators' virtual-point reprojection — none of which can
distinguish "lens aperture jitter" from a real disocclusion. Jittering it
would have destabilized every one of those already-shipped, already-
tuned systems, not just added a new one. Caught via `AskUserQuestion`
before implementation went further; redesigned around firing a SEPARATE,
dedicated ray instead, used only to determine defocus displacement — the
primary ray, and everything that depends on it, stays completely
untouched.

**The actual mechanism** (`src/hybrid/dof_ref.rs`, `assets/shaders/
hybrid_dof.wgsl`, new pass between `hybrid_denoise.wgsl` and
`hybrid_blit.wgsl`):
1. Thin-lens circle-of-confusion math (`circle_of_confusion_diameter`)
   and focal-length-from-vertical-FOV derivation
   (`focal_length_from_vertical_fov`, Bevy's own `bevy_post_process::dof`
   uses the identical derivation — confirmed by reading its shipped
   source) — not actually used for a screen-space blur radius (no
   spatial blur exists in this implementation at all), but validated as
   CPU-ref tests anyway since the underlying aperture-radius math
   (`aperture_radius = focal_length / (2 * f_stop)`) IS what drives the
   jitter disk size.
2. `vogel_disk_sample`: a golden-angle low-discrepancy disk sampling
   pattern (Vogel 1979), indexed by a ROTATING frame counter
   (`dof_sample_index`, `frame_index % ring_size`) rather than real
   randomness — this renderer has no RNG primitive anywhere (matches
   `hybrid_post.wgsl`'s own grain-hashing constraint, and `ddgi_ref::
   ddgi_probe_relight_start`'s identical rotating-index convention).
3. `dof_jittered_ray`: computes the focus-plane point along the
   UNPERTURBED primary ray (on the FLAT plane perpendicular to the
   camera's optical axis, not a sphere at Euclidean distance — a subtle
   but real correctness point for off-axis pixels, unit-tested
   explicitly), offsets the ray origin within the lens plane by the
   Vogel-disk sample, re-aims through the same focus point. A surface at
   the focal distance lands at nearly the same point regardless of
   aperture offset (zero effective blur, no special-cased "is this in
   focus" branch needed — it falls out of the geometry); a surface far
   from the focal plane visibly shifts position with the offset (unit-
   tested, both directions).
4. `hybrid_dof.wgsl`'s own compute pass fires this jittered ray against
   a SELF-CONTAINED copy of the trace machinery (BVH traversal, SDF
   march — no shading, since this pass only needs a hit POSITION, not a
   shaded color), duplicated per this codebase's established per-pass-
   file self-containment convention.
5. `dof_resample_uv`: reprojects the jittered ray's hit point back
   through the CURRENT (not previous) frame's `clip_from_world` to find
   which pixel of the ALREADY-SHADED, already-composited sharp image
   (`hybrid_denoise.wgsl`'s own `denoised_color_view`) to bilinear-
   resample color from — identical math to `temporal_ref::
   world_to_previous_uv`, verified to agree exactly via a direct
   equality test, just applied to the current frame's matrix instead of
   the previous one.
6. A DEDICATED ping-pong temporal accumulator (`HybridDofHistory`, own
   parity counter, own `max_history_length` convergence window —
   deliberately separate from the GI accumulator's, since DOF's per-
   frame perturbation is a much larger jump than GI's hemisphere-sample
   noise) blends the resampled color over time, gated by disocclusion-
   rejection against the SHARP pixel's own depth/normal (unaffected by
   jitter) — this is what correctly distinguishes "the lens sampled a
   different nearby pixel this frame" (expected, keep accumulating) from
   "the camera/object actually moved" (a real disocclusion, reset).

**Real-GPU verification, `gi_room` (fixed camera, easiest case to
verify convergence):** captured before/after at identical camera/frame
with `focal_distance` set to the purple/gold/salmon cube row's own real
distance and a wide simulated aperture (f/0.5) — the cube row renders
CRISP while the roof/walls behind it render VISIBLY DEFOCUSED, with no
bleeding at the sharp/blurred boundary. Confirmed stable (pixel-
consistent) between frame 700 and frame 1400 — converged, not still
drifting. Also spot-checked on `gallery.rs` (orbiting camera, `--stress
4`) for crash/NaN/validation-error safety across a harder, continuously-
moving-camera case — clean, no errors (convergence itself wasn't
verified there, a moving camera is a fundamentally harder case for any
temporal accumulator and wasn't this check's purpose).

**Verification**: 23 new CPU-ref tests in `dof_ref.rs` (thin-lens CoC
formula correctness at/near/away-from the focal plane, aperture-radius
f-stop scaling, Vogel-disk coverage/determinism/center-avoidance, the
core "in-focus stays put, out-of-focus shifts with aperture offset"
claim in both the on-axis and off-axis case, and `dof_resample_uv`'s
exact equivalence to `world_to_previous_uv`). 204/204 lib tests total
(up from 181), clean `cargo clippy --release --lib --examples`,
`wgsl_parse` gained a passing `hybrid_dof_wgsl_parses` test (and, as a
side effect of adding `clip_from_world` to the test harness's `View`
stub for this new shader, ALSO fixed a previously-failing pre-existing
test, `hybrid_blit_wgsl_parses` — that shader already used
`view.clip_from_world`, the stub just hadn't declared the field). Only
remaining `wgsl_parse` failure is `splat_wgsl_parses` (missing file,
wholly unrelated to this renderer). Real-GPU dispatch confirmed on both
examples with the feature disabled (default: `DofConfig::enabled =
false`, bit-for-bit no-op copy-through) and enabled (both `gi_room` and
`gallery`), no wgpu validation errors either way.

**A genuinely new binding shape for this codebase**: every prior compute
pass reads via exact `textureLoad` (no filtering) — `hybrid_dof.wgsl` is
the first to need a real bilinear-filtering `sampler` binding (for the
resample step at an arbitrary reprojected UV, not a texel-aligned one),
and `hybrid_pass`'s own system function grew past Bevy's system-param
tuple-arity limit once DOF's bind-group resource was added as the 18th
parameter — fixed by bundling all bind-group resources into one
`#[derive(SystemParam)]` struct (`HybridPassBindGroups`), the standard
Bevy answer to this limit, not a sign of any deeper coupling between
those resources.

**Known, not yet done**: no egui HUD timing line for `hybrid_dof` in
`gallery.rs` specifically (its own `FpsStats` struct has a fixed field
list — `gi_room.rs`'s simpler HUD does show the timing). Convergence
under a continuously-orbiting camera (as `gallery.rs`'s own default
camera does) not directly verified — `gi_room`'s fixed-camera test is
where the actual "sharp subject / blurred background" claim was
confirmed; a moving camera's disocclusion-driven history resets are a
harder, not-yet-characterized case for this specific accumulator's own
convergence speed.

### Investigated (not fixed): residual dark floor corridor after DDGI infinite-bounce

User-reported follow-up to the infinite-bounce entry directly below:
"strange black place on the floor... shadows aren't reflected correctly"
in `gi_room`. Confirmed real, via direct screenshot at the identical
camera/frame: a floor strip tucked between the -X wall and the cube row
stays near-black while the rest of the floor picked up real bounce light
from the infinite-bounce fix.

**Root cause, confirmed (not just hypothesized):**
1. With `GiMethod::None` (direct light only), this exact floor strip is
   ALSO black — it's a genuinely shadow-blocked area, not a rendering
   bug or a "duplicate slab" (there is no such object in `spawn_cubes`/
   `spawn_room`; the reviewer's "duplicate grey slab" read was almost
   certainly this same dark floor region misread as a distinct object).
2. Doubling probe density (`probe_spacing: 1.2` -> `0.6`) made NO
   measurable difference — ruled out "not enough spatial resolution."
3. Checked convergence at frame 700 vs. frame 1400 (double the
   accumulation time) — pixel-identical. Ruled out "hasn't finished
   converging yet."
4. Real cause: `sample_probe_grid`'s trilinear gather only ever reaches
   the 8 probes in a query point's OWN grid cell. For a floor-level
   point on a `vertical_layers: 6` grid, that NEVER includes a probe
   anywhere near the ceiling/roof-gap — light reaching the corridor
   floor has to hop DOWN one probe layer per relight cycle (6 hops),
   losing real energy (each hop gated by an intervening surface's own
   diffuse albedo) at every step. The converged result is genuinely dim,
   not stuck/broken — a real, honest limitation of trilinear-only
   probe-to-probe propagation on a tall, narrow occlusion topology.

**Attempted fix (built, tested, then reverted — did not survive
real-GPU verification).** Added `sample_probe_column_boost`: a second,
non-trilinear, non-occlusion-gated "peek" sample taken `N` probe layers
straight up the same grid column from a relight ray's own hit point,
blended into the existing trilinear gather via `max(local, boost *
weight)` (a shortcut path for light, not a replacement for the
occlusion-correct local sample). Built a dedicated CPU-ref test fixture
(`tall_corridor_fixture`: floor + a short cube-height obstruction + a
roof with a distant gap + a shallow-angle sun, all real object geometry,
not synthetic shortcuts) proving the MECHANISM works in isolation — the
trilinear-only chain converges to exactly zero in that fixture, the
boosted chain converges to a real non-zero value. Ported to
`hybrid_ddgi_relight.wgsl`, tried two tuning passes (`layers=3,
weight=0.6` and `layers=5, weight=1.0`) directly against `gi_room` on
real hardware — **both produced visually indistinguishable screenshots
from the unfixed version**, at both frame 700 and frame 1400. Root cause
of the mismatch: `gi_room`'s own corridor runs the FULL 13-unit length
of the room, and its roof gap opens above only part of that length —
the escape path for light isn't purely vertical the way the CPU-ref
fixture's own gap-directly-above shape assumed; it likely also needs to
travel horizontally along Z toward wherever the gap actually is above
this exact corridor slice. A straight-up-only peek doesn't reach that.

Reverted `sample_probe_column_boost`/`VERTICAL_BOOST_LAYERS`/
`VERTICAL_BOOST_WEIGHT` and the WGSL port entirely rather than shipping
an unverified-on-the-real-scene mechanism, per this project's own
"measure, don't assume" discipline — the CPU-ref test passing on a
synthetic fixture is not the same claim as "this fixes the real scene,"
and the real scene is what actually matters. `git log` has no trace of
this attempt (never committed); this entry is the only record.

**Status: known, accepted limitation, not fixed.** The floor strip is a
real, physically-explainable, converged result of a first-generation
trilinear-only infinite-bounce technique on a specific hard occlusion
topology (probe reach blocked both horizontally, by the cube row, and
vertically, by a roof gap that doesn't sit directly overhead this
corridor slice). A real fix would need either (a) a boost sample that
searches along the actual direction toward open sky/lit geometry rather
than a fixed straight-up peek, or (b) more probe layers reaching further
horizontally per hop (a wider, not just taller, propagation shortcut) —
both meaningfully bigger changes than this session's scope. Not
attempted further here; flagged for a future session with a clearer
design before another implementation attempt.

### DDGI infinite-bounce: probes sample the existing grid at their own relight hit point

Direct follow-up to a FIFTH external visual-quality review of the same
`gi_room` scene — still flagging near-black foreground silhouettes as
the single biggest giveaway, unchanged across every prior review despite
the Chebyshev visibility and roughness-aware spread work below (neither
of those touches how much LIGHT reaches a probe in the first place, only
how that light gets weighted/shaped once it's there). The user's own
framing of the ask ("larger feature... significant improvement," plus a
mid-session follow-up while this was in progress: "maybe we also need to
have multi bounces or some kind of accumulated ambient light for
simulating large bounces (4,5,6,..)") pointed at the actual structural
gap, confirmed by reading `ddgi_ref.rs::probe_ray` directly: a probe's
relight ray calls `shade` with indirect disabled, so a probe's own
stored irradiance is DIRECT LIGHT ONLY — explicitly documented in this
file's own prior header comment as "capped at one order of indirection."
A surface facing away from every directly-lit patch (every camera-facing
cube front face in this scene, since the sun only ever hits floors/tops
through the roof gap) has no directly-lit surface in its own hemisphere
to bounce off, so it stayed black regardless of how many frames passed —
temporal accumulation was smoothing NOISE in an already-zero signal, not
accumulating bounces.

**The fix: one line's worth of new energy, with an outsized effect.**
`probe_ray` now takes an `indirect_at_hit` term (a closure on the Rust
side; a direct call to `ddgi_sample_probe_grid_diffuse` in WGSL, since
WGSL has no closures) — on a hit, it samples the EXISTING probe grid's
own irradiance at the ray's hit point (full-diffuse hemisphere spread,
roughness fixed at 1.0 — the hit point is being treated as a generic
Lambertian bounce surface for gathering incoming light, not shaded with
the specular convention a camera-visible pixel gets) and adds
`diffuse_color * that_sample` on top of direct light, exactly mirroring
`hybrid_trace.wgsl::shade`'s own real indirect term. This is RTXGI's own
published "infinite bounce" trick: because the atlas is a single
read_write buffer carrying forward each texel's temporally-blended
history (not reset every frame), a probe relighting THIS frame samples
its neighbors' LAST-KNOWN irradiance — which already contains a sample
of ITS OWN neighbors from an earlier frame, recursively. Light diffuses
outward probe-to-probe over multiple frames, governed by the existing
`max_history_length` EMA (no new bounce-count parameter needed, and
explicitly not just a fixed "2nd bounce" — energy keeps propagating
every relight cycle, converging toward the true infinite-bounce steady
state as the temporal history fills in). Zero new storage, zero new
rays — the entire mechanism is reusing a texture read that already
exists, one hop earlier than before.

Ported to `hybrid_ddgi_relight.wgsl`: since this pass has no shared
imports (`hybrid_ddgi_relight.wgsl`'s own header, established for
`hybrid_trace.wgsl`/`hybrid_temporal.wgsl`/`hybrid_denoise.wgsl` too),
the full `ddgi_sample_probe_grid`-equivalent sampling stack (hemisphere
convolution, Chebyshev visibility, trilinear probe blend) had to be
duplicated here rather than imported — a genuinely large addition
(~150 new lines) but not a new design, a straight port of the same logic
`hybrid_trace.wgsl` already has, fixed at roughness=1.0 since a probe's
own relight hit point has no "camera-visible material roughness"
context worth threading through. Required reordering `DdgiGridUniform`/
`probe_position`/`atlas_tile_origin`/`atlas_index` to appear BEFORE
`probe_ray` in the file (WGSL's own "functions declared before use"
rule, see this project's own Editing Rules) — they used to sit after it,
since nothing before `probe_ray` needed them until now.

**Known, accepted race, documented inline (not fixed):** the atlas's
read side is no longer restricted to a probe's own texel the way its
write side is — a probe relighting this frame can read a NEIGHBORING
probe's texel while that neighbor's own invocation (same dispatch,
unordered on a GPU) is mid-write to it, whenever `probes_per_frame` is
large enough for two nearby probes to land in the same frame's relit
set (true at `gi_room`'s own `probes_per_frame: 512`). This is a stale/
torn-VALUE race, not a memory-safety one — GPUs don't fault on
concurrent storage-buffer read/write to the same word, and the worst
outcome is reading last frame's value instead of this frame's, both of
which are legitimate temporally-close irradiance estimates. RTXGI's own
published reference implementation has this identical characteristic
with its own in-place-updated probe buffer and accepts it rather than
double-buffering; this codebase does the same, for the same reason.

**Verification.** 4 new CPU-ref tests in `ddgi_ref.rs`: `probe_ray`
adds exactly `diffuse_color * injected_irradiance` on top of direct
light for a non-metallic hit (proving the term is real and correctly
scaled, not just present); a fully metallic hit gets exactly zero
indirect contribution (matching `shade`'s own direct-light treatment,
metals have no diffuse response); a miss ignores `indirect_at_hit`
entirely (no hit point to sample at); and an end-to-end
`relight_probe_texel` test proving a non-zero injected neighbor sample
lifts a probe's own relit result at or above the direct-only case,
confirming the mechanism actually reaches the atlas-write path, not just
`probe_ray` in isolation. All 4 pre-existing `probe_ray`/
`relight_probe_texel` call sites (own tests) updated to pass
`|_, _| Vec3::ZERO`, which reproduces the exact prior single-bounce
behavior bit-for-bit — confirmed by every existing test still passing
unchanged. 181/181 lib tests total (up from 177), clean `cargo clippy
--release --lib --examples`, `wgsl_parse` unchanged (same 2 pre-existing
unrelated failures every prior entry already has).

**Real-GPU screenshot comparison — this one DID show a clear, visually
obvious difference** (unlike the Chebyshev entry below, where an initial
hypothesis didn't pan out under direct comparison): `gi_room --shot ...
--at-frame 700` (roof fully open, ~10s of real accumulated relight time)
captured before and after this change, identical camera/scene state.
Before: every cube's camera-facing silhouette is near-solid black, no
front-face detail visible at all — exactly the complaint every review
raised. After: the purple cube's near face, the gold/dark stack's front,
the salmon cube's face, and the green cube at frame edge all show real,
distinct diffuse fill light with no direct sun reaching them — genuine
bounce light arriving from the sunlit floor/walls behind the camera.
Floor also picks up visible bounce brightness under the cubes. No wgpu
validation errors, no NaN/Inf artifacts, at frame 700 or a longer soak
at frame 1400 (checked for runaway energy feedback from the new
probe-samples-itself-recursively path — none observed; the EMA's own
clamped `max_history_length` bounds how much any single frame's sample
can move the accumulated value, which is what keeps this convergent
rather than diverging).

**Measured cost**: `examples/gallery.rs --stress 4` (DDGI active,
`probes_per_frame: 512`), reading the on-screen HUD's own per-second GPU
pass timings — `ddgi` (the relight pass this change touches) reports
`0.12-0.14 ms`, indistinguishable from its own pre-change cost (this
project's Chebyshev entry below didn't log a bare `ddgi` number to
compare against, but the pass does the same ray count either way — this
change adds texture reads to an already-dispatched invocation, not new
rays or new dispatches). `trace` (unrelated to this pass) dominates at
`~25-30ms` as before. No measurable regression from this feature.

**Known, not yet fixed / explicitly out of scope:** no quantitative
energy-conservation check (e.g. total scene radiance converging to a
specific expected value for a simple closed-form test scene) — this
entry's confidence rests on unit tests proving the mechanism is wired
correctly plus direct visual confirmation, not a numerical convergence
proof. The new WGSL sampling stack duplicated into
`hybrid_ddgi_relight.wgsl` doubles this file's own line count; worth
watching if a future change needs to touch both copies and drifts them
apart (no automated drift check between the two exists in this
codebase, per its own established per-pass-file self-containment
convention).

### DDGI probe visibility: Chebyshev depth-aware weighting; roughness-aware GI hemisphere spread

Direct follow-up to a FOURTH external visual-quality review of the same
`gi_room` scene. Two of its items were acted on; one claimed artifact
was investigated and found NOT reproducible, documented honestly rather
than claimed fixed.

**What was investigated and NOT confirmed as a real bug.** The review
described a "soft blob-like specular highlight/bloom" at the wall-
ceiling corner, distinct from a normal shadow crease. Before writing any
code, checked whether this renderer has any bloom pass at all (it does
not) and hypothesized it was DDGI's own probe grid under-resolving thin
geometry at that corner (`probe_spacing: 1.2` vs. `WALL_THICKNESS: 0.3`
— a real, 4x resolution mismatch, and a known documented DDGI limitation
this project's own `ddgi_ref.rs` module doc comment already flagged: "no
Chebyshev visibility test... light leaking through thin occluders is a
real, accepted risk"). Built the fix on that hypothesis (below), then
took a direct before/after screenshot at the identical camera pose to
confirm it — and found NO visible difference between the two images.
Re-examining both side by side, the soft gradient in question is most
likely an ordinary soft-shadow penumbra cast by the roof's leading edge,
not a light leak at all. Concrete lesson applied here: build the
genuinely-warranted general improvement (Chebyshev visibility IS a real,
correct, unit-tested fix for a real, documented class of DDGI bug) but
do NOT claim it fixed a specific visible artifact that direct comparison
couldn't actually demonstrate — matching this project's own "measure,
don't assume" discipline applied to a case where the measurement
disagreed with the initial hypothesis.

**Chebyshev depth-aware probe visibility (real, unit-tested, shipped).**
`sample_probe_grid`'s only visibility signal used to be a single hard
occlusion ray fired from the SHADED POINT's own side toward each of the
8 surrounding probes — this only catches "something sits directly
between the shaded point and the probe." It cannot catch "this probe's
own stored irradiance is itself unreliable because its relight rays
skimmed through/near thin geometry from the PROBE's own vantage point"
— a probe sitting close to a thin wall can have its own rays graze past
it at certain angles, over-reporting how far it can see, and that bad
irradiance then gets trilinearly blended into nearby shaded points'
results regardless of whether their own specific occlusion ray happens
to be clear.

Fixed by storing, per atlas texel, the mean and mean-of-squares of
distances the probe's own relit rays actually found (`probe_ray` now
returns `(radiance, hit_distance)`, with a miss reporting `max_t` —
RTXGI's own convention: the farthest a real ray could have found
nothing). At shading time, `chebyshev_visibility_weight(dist, mean,
mean_sq)` uses Chebyshev's one-sided inequality: `dist <= mean` is
treated as fully visible; farther than that, `variance / (variance +
(dist-mean)^2)` smoothly discounts the probe, with `CHEBYSHEV_VARIANCE_
FLOOR` guarding the near-zero-variance case (a probe whose rays all hit
a perfectly flat wall) from a divide-by-near-zero blowup — same
"clamp a near-degenerate denominator" pattern `REFLECT_ROUGHNESS_GATE`
and `refract_ref::refract_trace_ray`'s own `safe_rho` clamp already
establish elsewhere in this codebase. The two visibility signals (hard
occlusion ray + Chebyshev) combine multiplicatively, matching RTXGI's
own published implementation, since neither alone catches both failure
modes.

New `distance_atlas` buffer (`array<vec2<f32>>`, same row-major indexing
as the existing irradiance atlas) — a SEPARATE buffer rather than
widening the existing `vec4<f32>` atlas's unused `.a` channel, since
Chebyshev needs two floats (mean, mean^2) and the existing channel
budget was already fully spent on irradiance's `.rgb`. Written by
`hybrid_ddgi_relight.wgsl` via the SAME `temporal_blend` clamped-EMA
formula irradiance already uses (packed as `(dist, dist^2, 0.0)` into
its `Vec3`/`vec3<f32>` shape purely to reuse the exact formula), sharing
irradiance's own `history_length` (both are relit by the identical ray
on the identical schedule — there is only one real "how many samples"
answer, not two independently drifting ones).

7 new CPU-ref tests: `chebyshev_visibility_weight`'s own unit tests
(full visibility at/inside the probe's own mean, near-zero weight far
beyond a confident measurement, higher weight for a noisier/less
confident probe at the identical distance gap, no divide-by-zero at
true-zero variance) plus a full `sample_probe_grid` integration test
proving a probe confidently reporting a short reach gets discounted even
with a clear hard-occlusion ray AND favorable trilinear proximity both
arguing for it — the exact failure mode this feature targets, isolated
in a geometry-free fixture so only the Chebyshev term can be responsible
for the observed discount.

**Roughness-aware GI hemisphere spread (real, unit-tested, shipped,
answers the "GI bounds disperse based on reflection level" request).**
`cosine_weighted_probe_irradiance`'s existing 5-sample hemisphere
convolution used a FIXED spread regardless of the shaded surface's own
material — a glossy floor's indirect response looked exactly as diffuse
as a rough wall's. New `cosine_weighted_probe_irradiance_roughness_
aware(surface_normal, roughness, probe_irradiance)`: pulls each of the 5
fixed sample directions toward the pole `(0,0,1)` (straight along the
surface normal) by `1.0 - roughness` before transforming into world
space and re-normalizing — at `roughness = 1.0` (fully rough/diffuse)
this reproduces the exact original full spread; at `roughness = 0.0`
(fully smooth/glossy) all 5 samples converge onto a single mirror-like
direction, mirroring how a real material's own BRDF lobe narrows as it
gets glossier. `sample_probe_grid` gained a `roughness: f32` parameter
threaded through to this new convolution (Chebyshev's own distance-
moment convolution deliberately stays the FULL-spread, non-roughness-
aware version — probe visibility is a geometric property of the grid,
not a material property, so narrowing it by roughness would conflate
two unrelated concepts). `hybrid_trace.wgsl`'s `shade()` passes
`clamp(obj.roughness, 0.0, 1.0)` at its own `ddgi_sample_probe_grid`
call site.

4 new CPU-ref tests: exact match against the original function at
`roughness=1.0`, convergence toward the pure on-axis value at
`roughness=0.0`, and a direct per-sample-direction monotonicity check
(each of the 5 fixed directions' own alignment with the surface normal
must increase, never decrease, as roughness drops) — written after an
initial, less direct test (checking aggregate convolved brightness
end-to-end through an asymmetric off-axis test field) produced a real
but misleading non-monotonic result at one intermediate roughness value,
caused by `HEMISPHERE_SAMPLES`' own lack of azimuthal symmetry around an
arbitrary tilted normal, not a flaw in the roughness-narrowing mechanism
itself — replaced with the more direct per-sample check once the root
cause was understood, rather than loosening the assertion to paper over
it.

**Verification**: 177/177 lib tests (up from 169 — 11 new), clean
`cargo clippy --release --lib --examples`, `wgsl_parse` unchanged (same
2 pre-existing unrelated failures every prior entry already has), real-
GPU screenshots on both examples with no wgpu validation errors. Did NOT
manage to isolate a clean, visually-distinct rough-vs-glossy screenshot
comparison in this session (camera framing across the available example
scenes made a clean side-by-side awkward to capture in the time spent) —
the feature's correctness rests on its CPU-ref unit-test coverage plus
successful real-GPU dispatch with no errors, not a visual proof; flagged
here explicitly rather than glossed over, matching this entry's own
opening lesson about not overclaiming what wasn't actually demonstrated.

**Known, not yet fixed / explicitly out of scope:** the reviewed
"bloom" artifact itself remains unexplained (most likely an ordinary
soft-shadow penumbra, but not conclusively confirmed either way) — worth
revisiting with a targeted synthetic thin-wall test scene if it turns
out to be a real, reproducible complaint again. No visual A/B proof yet
that roughness-aware spread reads correctly on real glossy vs. rough
materials side by side. Chebyshev's own `CHEBYSHEV_VARIANCE_FLOOR` value
is a first-pass constant, not swept/validated.

### DDGI revived as the default indirect-diffuse GI method, cone tracing kept selectable

Direct follow-up to a THIRD external visual-quality review of the same
`gi_room` scene, still flagging near-black foreground objects as "the
single biggest giveaway" despite the geometric-series bounce tail from
the entry below. Root cause identified before writing any code: cone
tracing's `cone_trace_indirect` fires a fixed 5-cone hemisphere centered
on a shaded surface's OWN normal — a cube's camera-facing side (normal
pointing away from a bright wall behind/beside it) structurally cannot
sample that wall through any single-hop hemisphere sample, no matter how
bright it is or how the sample set is tuned. Real bounce light reaching
that face requires a genuine two-hop path (wall -> an intermediate
surface that itself faces the wall -> the cube's own front face), which
needs the intermediate surface to be independently sampled, not just
intersected. This is exactly the structural gap a spatial GI cache (DDGI,
a hash-grid, ReSTIR) solves and a per-pixel local-hemisphere technique
cannot, by construction.

This renderer already built, bug-fixed, and real-GPU-measured a full
DDGI implementation (persistent world-space probe grid, octahedral atlas,
multi-bounce, ~2.3-2.7x trace cost measured at the time) before
consolidating on cone tracing alone for maintenance-scope reasons (one
technique instead of four) — see the "Consolidated on SDF cone tracing"
entry further down. That removal was a scope decision, not a quality
verdict, so with three independent reviews now converging on the exact
problem DDGI's own architecture solves, the user's explicit direction was
to revive DDGI as the default and keep cone tracing in the codebase,
selectable but no longer the default — not delete it, and not build a
new technique from scratch.

**Restored from git history** (`263b6e1^`, the commit immediately before
DDGI/hash-grid/ReSTIR were all removed together): `src/hybrid/ddgi_ref.rs`
(CPU-ref, 1468 lines) and `assets/shaders/hybrid_ddgi_relight.wgsl` (WGSL
relight pass, 800 lines) — selectively, not a blind revert, since the
original removal commit bundled DDGI/hash-grid/ReSTIR removal together
with an unrelated BVH margin-tightening optimization that's still in use
today and must NOT be reverted. Hash-grid and ReSTIR were deliberately
NOT revived — only DDGI addresses this specific complaint.

**Real integration work, not a copy-paste**, since everything built since
DDGI's removal (multi-bounce reflections, transmission/refraction,
dedicated per-phenomenon temporal histories, tonemapping, the lens/sensor
post-process pass) changed the shapes DDGI's old code assumed:
- `ddgi_ref.rs::probe_ray` called the old 9-parameter `shade()` with a
  `sky_color(direction)` miss-fallback — updated to the current
  11-parameter `shade()` (passing `None, None` for reflection/
  transmission — a probe ray never fires a nested specular/transmissive
  bounce, matching this function's own "capped at one order of
  indirection" design) and changed the miss case to report exactly
  black, matching the "a miss contributes nothing, not a mocked sky
  gradient" convention `conetrace_ref.rs`'s own light-leak fix
  established (predates the sky-color removal) — a genuinely SAFER
  version of the original bug-fix chain: a probe embedded near/inside
  sealed-room geometry now bakes zero into its own irradiance on a miss
  instead of a bright fake-sky value, on top of the existing probe-
  centering fix that was supposed to prevent probes from landing there
  at all.
- `GiMethod` gained back `Ddgi = 1` (its original discriminant,
  `HashGrid = 2`'s slot stays reserved/unused) alongside the existing
  `ConeTrace = 3` — `GiMethodConfig::default()` now returns `Ddgi`.
- `SceneUniform` gained 6 DDGI-specific fields
  (`ddgi_probes_per_frame`/`ddgi_total_probes`/`ddgi_tile_size`/
  `ddgi_frame_index`/`ddgi_max_history_length`/`ddgi_max_t`), appended at
  the END of the struct (not re-interleaved at their original position)
  so every existing field's byte offset stays unchanged — propagated to
  all 5 WGSL mirrors (`hybrid_trace`/`hybrid_temporal`/`hybrid_denoise`/
  `hybrid_blit`/the new `hybrid_ddgi_relight`), each of which needed its
  own full-struct mirror brought current with reflection/transmission
  fields that didn't exist when DDGI was last alive (the relight
  shader's own mirror was a stale 19-field truncated version before this
  fix — a real latent bug, never triggered only because nothing had
  extended `SceneUniform` far enough to misalign it before now).
- `ObjectGpu`'s `_pad_material0`/`_pad_material1` fields (what the old
  DDGI code's own WGSL mirror still called them) are now
  `transmission`/`ior` — same byte offsets, renamed in the relight
  shader's own mirror to match; the relight shader doesn't read either
  field (direct-lit-only shading has no use for transmission), so this
  was a pure rename, not a behavior change.
- New `DdgiConfig` resource (probe grid tunables: `probes_per_frame`,
  `tile_size`, `max_history_length`, `max_t`, `probe_spacing`,
  `vertical_layers` — same defaults as before removal, still not
  validated by a real sweep against this renderer's specific scenes).
- `pipeline.rs` gained back: `hybrid_trace_ddgi_read_layout` (trace's own
  new group 2, read-only atlas access), `hybrid_ddgi_layout` (the
  relight pass's own single read_write group), `HybridDdgiAtlas`/
  `HybridDdgiAtlasRes` (the manually-indexed `array<vec4<f32>>` storage
  buffer + per-probe history-length buffer, NOT ping-ponged — each
  relit probe only ever touches its own texels), `DdgiGridUniform`,
  `HybridDdgiFrameIndex` (a dedicated rotation counter, deliberately
  separate from `HybridFrameParity`), `HybridDdgiBindGroup`/
  `HybridTraceDdgiBindGroup`, and `prepare_hybrid_ddgi` (rebuilds the
  atlas on a probe-count/tile-size change, uploads `DdgiGridUniform`
  every frame since grid placement can shift as the scene's own root
  AABB moves).
- `pass.rs` gained a new dispatch block (0): the relight pass, running
  FIRST in `hybrid_pass` (before trace, same "read-before-write"
  ordering `hybrid_temporal.wgsl` already established relative to
  denoise) — skipped entirely (not dispatched at all) when DDGI isn't
  the active `GiMethod`, same "real dispatch to save, not just a branch"
  reasoning reflection/transmission's own temporal passes already use.
  Trace's own group 2 (the DDGI atlas read) is bound UNCONDITIONALLY
  regardless of active technique (the pipeline's bind-group layout is
  fixed at creation time), but costs nothing extra when inactive since
  `shade()` only samples it inside the `gi_method == GI_METHOD_DDGI`
  branch.
- `hybrid_trace.wgsl` gained the full shading-time sampling section
  (trilinear blend over the 8 probes surrounding a shaded point, each
  visibility-gated by one occlusion ray, with the smooth fallback ramp
  that fixed a real flicker bug at the wall/corner occlusion boundary
  during DDGI's original development) — reusing the EXISTING
  `ddgi_tangent_basis`/`HEMISPHERE_SAMPLES`/`HEMISPHERE_SAMPLE_COUNT`
  cone tracing's own section already declares (kept there after DDGI's
  original removal specifically because `cone_trace_indirect` reuses the
  same 5-sample hemisphere bundle) rather than duplicating a second
  identically-valued copy under a new name, which would have been a
  WGSL redefinition error (caught immediately by `wgsl_parse`).

**CLI/UI wiring**: `--gi-method none|ddgi|conetrace` (default `ddgi`) and
`--ddgi-probes-per-frame`/`--ddgi-tile-size`/`--ddgi-history`/
`--ddgi-max-t`/`--ddgi-spacing`/`--ddgi-layers` in `gallery.rs`; a third
"DDGI" radio button (alongside None/Cone-trace) plus probe-grid sliders
and a `gpu_ddgi_ms` HUD suffix in both examples' egui panels.
`gi_room.rs` also gets its own room-scale `DdgiConfig` override
(`probe_spacing: 1.2, vertical_layers: 6, max_t: 28.0` — tightened from
`gallery.rs`'s `--stress N`-scale defaults for the same reason
`ConeTraceConfig`/`ReflectionConfig`/`TransmissionConfig` already are in
this file) and switches its own forced `GiMethodConfig` override from
`ConeTrace` to `Ddgi`.

**Verification**: 169/169 lib tests (up from 136 — 33 new, all of
`ddgi_ref.rs`'s own restored+fixed test suite), clean `cargo clippy
--release --lib --examples`, 5/7 `wgsl_parse` tests passing (new:
`hybrid_ddgi_relight_wgsl_parses`; the same 2 pre-existing unrelated
failures as every prior entry, confirmed unrelated to this work).
Real-GPU verified on both examples with no wgpu validation errors —
`gallery.rs --gi-method ddgi --at-frame 100000` steady-state log
sampling shows relight costing **~0.04ms/frame** at this scene's small
(1-2 object) scale, and `gi_room`'s own HUD shows **~0.9ms/frame** at
its own larger room-scale grid — both cheap relative to trace/temporal/
denoise. Direct before/after screenshot comparison at the identical
`gi_room` camera pose (roof-open, same frame) against the cone-tracing
baseline shows a real, visible improvement: the gold-topped cube stack's
and leftmost cube's own near edges show a visibly stronger, softer
bounce-light gradient with DDGI active than cone tracing's harder cutoff
produced — plus trace time itself dropped (72.6ms vs. 95.8ms at this
pose), since DDGI's shading-time cost is a handful of atlas texture
reads plus 8 occlusion rays, not a live 5-cone BVH march per pixel.

**Known, not yet fixed / explicitly out of scope:** the central
foreground object's own camera-facing side is still mostly dark at this
exact `gi_room` camera pose — correctly: that pose looks back roughly
along the sun's own incoming ray direction (see the tonemapping entry
below), so the camera genuinely sees each object's most self-shadowed
side; DDGI's spatial probes fix the "light can't wrap around occluding
geometry" problem but can't invent light on a face literally nothing
(direct or bounced) reaches. `DdgiConfig`'s defaults remain unvalidated
by a real sweep. No Chebyshev visibility test (irradiance-only probes,
same accepted light-leak-through-thin-occluders risk documented in the
original DDDGI entry). Cone tracing's own code is fully intact and
selectable via `--gi-method conetrace` / the egui radio button, not
deleted, per explicit instruction — worth revisiting if a future review
or use case favors its own tradeoffs (no temporal lag, no spatial
quantization, cheaper at very small scenes).

### Geometric-series GI bounce tail, signal-dependent film grain

Direct follow-up to a SECOND external visual-quality review of the same
`gi_room` sealed-room screenshot (after the tonemapping/post-pass entry
below landed), re-flagging: foreground objects still near-black on their
camera-facing side despite sitting near a bright wall, grain applied as
a flat overlay regardless of scene brightness (real sensor noise is far
more visible in shadows than highlights), plus re-raised (not newly
confirmed) faceting/CA/DoF/PBR-specular complaints already investigated
or explicitly scoped out in the prior entry.

**GI bounce tail — a real gap, fixed without an ambient hack.** Explicit
user direction: the fix must be "real, not fake" and should scale with
how much real light the scene already has, ruling out an independent
ambient/sky constant (which risks exactly the light-leak class of bug
`coverage^2` and the room-leak regression tests already guard against —
see `conetrace_ref::room_leak_regression`). `cone_trace_ray`'s own doc
comment already documents WHY it's single-bounce by default (a second
full bounce doubles an already-uncached, dominant per-frame cost with no
persistent structure to amortize against) — so the fix isn't "trace more
bounces," it's "stop discarding the light `max_bounces` truncation was
already throwing away." At the final traced bounce, added a geometric-
series tail term: `direct_and_emissive * diffuse_color / (1 -
diffuse_color)`, approximating the sum of ALL further un-traced bounces
under the same homogeneous-albedo assumption this loop's own per-bounce
`throughput` update already makes (bounce N+1 re-emits `result *
diffuse_color`, N+2 re-emits `result * diffuse_color^2`, etc. — a
standard geometric series). Derived entirely from THIS hit's own
already-computed `result`/`diffuse_color` — a hit with zero real light
(e.g. every surface inside a sealed room with no light source) produces
an exactly-zero tail, so it cannot manufacture light where none exists;
it only extends light that's already real. `diffuse_color` clamped to
`0.95` per channel before the division to avoid a near-degenerate
denominator blowing up for a near-white fully-diffuse material.

Ported to both `conetrace_ref::cone_trace_ray` (CPU-ref) and
`hybrid_trace.wgsl::cone_trace_ray` (WGSL) identically — NOT applied to
`reflect_trace_ray`'s own separate bounce chain (specular reflection has
its own distinct roughness-based termination model, extending it wasn't
requested and wasn't part of this fix's scope). New tests:
`cone_trace_ray_tail_term_adds_real_light_on_a_genuinely_lit_hit`
(reconstructs the pre-tail single-bounce result directly from `shade`'s
own output and confirms the tailed result is strictly brighter) and
`cone_trace_ray_tail_term_stays_exactly_zero_when_the_hit_itself_is_unlit`
(a hit with no lights in the scene must report exactly zero, not a
positive floor). Both pre-existing room-leak regression tests
(`_at_three_bounces` too) still pass unchanged — the tail term doesn't
touch their light-leak-safety invariant since it's multiplicatively
gated on real light.

Confirmed via direct before/after screenshot at the identical `gi_room`
camera pose/frame: cube edges (particularly the gold-topped stack and
the leftmost cube's silhouette) that were previously near-pure-black now
show a visible soft gradient of bounce fill light — a genuine, real
improvement, not merely "no longer literally zero." The central pink
cube's camera-facing side is still mostly dark, correctly — the tail
amplifies existing bounce light reaching a surface, it doesn't invent
light on a face no real light (direct or bounced) reaches at all, which
is the physically-correct behavior for a nearly-fully-self-shadowed face
at this specific camera angle (see the prior entry's own GI-bounce
investigation for why this camera pose looks along the sun's own
incoming direction).

**Signal-dependent film grain.** `hybrid_post.wgsl`'s grain was a flat
`noise * grain_strength` regardless of the pixel's own brightness — a
real bug, not a stylistic choice (real sensor read noise dominates in
relative terms at low signal and is swamped by a large one, so real
photos show visibly more grain in shadows than highlights). Fixed by
scaling grain by `clamp(1.0 - luminance, 0.15, 1.0)` — floored so
near-black pixels don't get an unbounded multiplier, ceilinged at 1.0 so
even bright highlights retain a small amount (real sensors are never
perfectly noiseless when well-exposed). Confirmed visually: the fixed
frame shows clearly more grain texture in the dark background/shadow
regions than on the bright magenta sky/white ceiling, versus uniform
speckle density everywhere before.

**Re-raised complaints — deliberately not re-investigated, already
covered by the prior entry:** faceting (real sharp-cube-edge geometry,
not a shading bug — see prior entry's own pre/post tonemap comparison;
this particular screenshot's pink cube is ALSO now catching real tail-
term light, though its camera-facing side is still mostly self-shadowed
per the paragraph above), radially-varying CA (math already verified
correct — zero at center, `falloff^2` growth toward corners — subtle
default strength likely just doesn't read clearly in a compressed
screenshot), DoF and full PBR roughness/specular response (real missing
features, explicitly out of scope per the prior entry, not touched
here).

**Verification:** 136/136 lib tests (up from 134 — the 2 new tail-term
tests above), clean `cargo clippy --release --lib --examples` on both
touched files, `hybrid_post_wgsl_parses`/`hybrid_trace_wgsl_parses` both
still pass, real-GPU before/after screenshot comparison at the identical
`gi_room` pose with no wgpu validation errors.

### Filmic tonemapping/exposure via Bevy's own pipeline, post-tonemap lens/sensor pass, GI-bounce/faceting investigation

Prompted by an external visual-quality review of a `gi_room` sealed-room
screenshot flagging: hard-clipped pure-white/pure-black regions with no
highlight/shadow detail, "unlit flat-shaded" materials, visibly faceted
shading on curved-looking objects, no visible GI bounce/rim light onto
dark foreground silhouettes, and no lens artifacts (grain/vignette/CA).
Investigated all five; two were the same root cause, one was a real fix,
one was scene composition (not a bug), one was cosmetic and added.

**Root cause of clipping/flat-shading/faceting: no tonemap curve at all.**
`hybrid_blit.wgsl` wrote raw linear HDR straight into `ViewTarget`'s main
texture with no camera exposure and no tonemap curve — confirmed by
reading the shader directly (`return FragOut(vec4<f32>(color, 1.0),
depth)`, `color` = `textureLoad(color_tex, ...)` unmodified). Neither
example camera had `Hdr`/`Tonemapping` on it, so even though
`hybrid_pass` is a plain system inside `Core3dSystems::MainPass` (not a
render-graph node Bevy's own tonemapping could see), the fix didn't need
a from-scratch tonemap reimplementation: `Core3dSystems` chains
`MainPass -> EarlyPostProcess -> PostProcess`, and Bevy's own
`tonemapping` system already runs in `PostProcess` reading whatever
`hybrid_pass`'s blit wrote into `ViewTarget`'s ping-pong `main_texture`.
Adding `Hdr` + `Tonemapping::default()` + `Exposure::default()` to both
example cameras (`bevy::camera::{Hdr, Exposure}`,
`bevy::core_pipeline::tonemapping::Tonemapping`) was sufficient — no
change to `hybrid_pass`'s own pass ordering or `hybrid_blit.wgsl`'s
output was needed beyond making its `SceneUniform` mirror the full
struct (was previously a truncated 8-field prefix that happened to still
line up; now mirrors trace/temporal/denoise's own full copies for
robustness against future field additions).

Confirmed via direct pre/post screenshot comparison at the identical
camera pose (`gallery.rs --at-frame 60`): the baseline cube face was a
single flat, saturated color swatch with a hard-edged shadow blob and a
clipped-white reflection strip; the tonemapped version shows a real
per-face brightness gradient, a soft shadow penumbra, and non-clipped
reflection falloff. This same pre/post pair also resolved the "faceted
shading" complaint with NO separate fix: the cube's `RoundedBox` normal
(`local_normal`, a proper 6-tap central-difference SDF gradient, not a
per-face flat normal) was already continuous — the hard clipping was
what made two genuinely-gradient-shaded faces read as flat swatches
meeting at a line. `corner_radius: 0.0` on these cubes means the edge
itself is real (a sharp-edged cube legitimately has a discontinuous
normal there); only the missing per-face gradient was the bug.

**GI-bounce-onto-dark-objects: investigated, not a bug.** `gi_room`'s
sealed-room cubes viewed from the camera happen to be lit from a
directional sun entering through a roof gap on the opposite side, and
the fixed `CAMERA_CORNER` looks back roughly along the sun's own
incoming ray direction — meaning the camera sees each cube's own
self-shadowed side by construction, not a GI failure. Verified the
underlying mechanism directly rather than trusting scene-angle
reasoning alone: added
`cone_trace_indirect_picks_up_real_bounced_light_from_a_lit_box_below`
(the full 5-cone hemisphere `cone_trace_indirect` had NO existing
positive-case test — only "deterministic" and "returns black when
nothing nearby" — a real coverage gap, now closed) confirming the
5-cone hemisphere does pick up real bounced light from a genuinely lit
neighbor. No renderer change made; scene composition, not a defect.

**New post-tonemap "lens/sensor" pass** (`assets/shaders/hybrid_post.wgsl`,
`src/hybrid/post.rs`): film grain, vignette, and chromatic aberration,
deliberately a SEPARATE fullscreen pass scheduled
`Core3dSystems::PostProcess.after(tonemapping)` rather than folded into
`hybrid_blit.wgsl`'s pre-tonemap linear output — these are conventionally
display-referred camera/sensor artifacts (lens light falloff, sensor read
noise, per-wavelength focal shift), not properties of scene-linear
radiance. Reads/writes via `ViewTarget::post_process_write()`'s ping-pong,
the same mechanism `bevy_core_pipeline`'s own tonemapping/upscaling passes
use (read as a pattern reference, not imported); specialized on
`TextureFormat` like `hybrid_blit_pipeline`'s own `HybridBlitKey`, since
this pass's target format (the real display-referred swapchain format,
post-tonemap) isn't known until pipeline-creation time. Grain is a
spatial hash of pixel coordinate + frame count (no per-pixel RNG
primitive exists in this renderer, matching the rest of the codebase's
established constraint), not true randomness. All three effects are
plain always-applied coefficients (`HybridPostConfig`, `0.0` = fully
off) rather than a shader permutation or dispatch-skip — this is already
a single cheap fullscreen pass, so branching it off would only save a
std140-uniform write. Verified with an exaggerated-strength screenshot
(`grain_strength: 0.15, vignette_strength: 0.6, aberration_strength:
0.02`) showing unambiguous grain texture, corner darkening, and red/blue
edge fringing, then reverted to subtle production defaults
(`0.015/0.25/0.0015`). Wired into both examples: `--grain`/`--vignette`/
`--aberration` CLI flags + egui sliders in `gallery.rs`, egui-only
sliders in `gi_room.rs` (matching that scene's existing reflection/
transmission convention of no CLI equivalent).

**Verification:** 134/134 lib tests (new: 1 tonemapping-adjacent GI
coverage test above), clean `cargo clippy --release --lib --examples` on
every touched/new file, `hybrid_post_wgsl_parses` added to
`tests/wgsl_parse.rs` (passes — unlike the pre-existing
`hybrid_blit_wgsl_parses` failure, which is naga's standalone parser
choking on `View::clip_from_world` with no Bevy shader-def context, not
a real shader bug, confirmed pre-existing on `master` before this work).
Real-GPU screenshots on both examples with no wgpu validation errors.
Measured real cost of the new post pass via `--at-frame 100000` steady-
state log sampling: **~0.6-1.1ms**, cheap relative to the existing
trace/temporal/denoise/blit passes (12-15ms/6-8ms/3-4ms/1ms respectively
at this resolution).

**Known, not yet fixed / explicitly out of scope:**
- No AO, area lights, or a real environment/sky (still the flat magenta
  miss-color convention) — separate, larger features, not touched here.
- No depth-of-field — flagged by the same review as a "sells the
  photograph" cheap win, but requires a real CoC pass, out of scope for
  this session's fixes.
- The post pass's grain/vignette/aberration constants are hand-picked
  "looks reasonable," not swept/validated against a reference photo the
  way the reflection roughness-gate sweep was.
- Chromatic aberration direction/falloff (radial from center, `falloff^2`
  growth) is a common real-time approximation, not derived from a real
  lens model.

### Multi-bounce transmission/refraction, dedicated temporal history, WGSL + real-GPU verified

Direct follow-up to the multi-bounce specular reflections entry below —
this renderer's PBR material had no transmissive/refractive term at all:
a `transmission > 0.0` dial didn't exist, so glass/water-like materials
weren't representable. This lands real, multi-bounce, shadow/GI-aware
transmission, using the identical cone-marching substrate and design
philosophy reflection's own entry already established (deterministic,
no RNG, final-gather termination via one bounce of diffuse GI).

**Solid-dielectric model.** A transmissive object is treated as a solid
volume of a single medium — a ray enters through one surface, marches
the object's OWN local SDF from the inside until it re-emerges (a new
`march_object_interior` primitive, stepping by `max(|d|, MIN_INTERIOR_STEP)`
since sphere-tracing the raw negative interior distance never advances
near the entry point), refracting at BOTH the entry and exit surfaces
(Snell's law, `refract_ray`/`refract` — falls back to a mirror bounce on
total internal reflection, matching GLSL's own `refract()` convention of
"no real refracted ray exists" rather than an error case) and absorbing
color along the interior path via Beer-Lambert (`transmittance =
exp(-(1-base_color) * distance)` — a white material absorbs nothing,
matching real clear glass; a colored material tints proportionally to
how far light travels through it, so a thick slab reads more saturated
than a thin one of the same material). This mirrors glTF's own
`KHR_materials_volume` "thickness" model — the standard real-time
approximation, since full multi-surface refractive light transport
isn't tractable on this renderer's SDF-marching substrate (the same
call this project already made once for reflections).

**New files/functions**: `src/hybrid/refract_ref.rs` — `refract(i, n,
eta)` (Snell's law, `None` on TIR), `march_object_interior` (the
single-object interior march `reflect_trace_ray`'s whole-scene
`trace_cone` structurally can't do — finding an object's OWN exit
surface from inside needs marching that object's SDF directly, not a
BVH trace with one entity excluded), `refract_trace_ray` (the bounce
chain: refract in, march to exit, absorb, refract out, shade the exit
point, continue if the outgoing ray happens to re-enter transmissive
geometry), `shade_for_refraction_bounce` (full direct light + real
shadow rays + ONE single-cone diffuse-GI sample — see below).
`MAX_TRANSMISSION_BOUNCES: u32 = 4` (same ceiling as reflection's own),
`TRANSMISSION_FRESNEL_CUTOFF = 0.02` (same value/reasoning as
`REFLECTION_FRESNEL_CUTOFF`). 9 new tests: Snell's law matches the
textbook formula at normal incidence and bends toward the normal
entering a denser medium; TIR is correctly detected; a clear glass cube
picks up a red-emissive box's own color through it (positive proof, not
just "doesn't crash"); a thick slab of green-tinted glass reads
green-dominant (Beer-Lambert absorption proven, not just wired);
unlit glass with nothing behind it is exactly black (leak-proof); a
zero-transmission dial terminates the chain with no further energy
regardless of requested bounces; bounce count clamps at the ceiling; a
lit scene's own direct light reaches the exit surface correctly
(catches a real test-setup bug — `view_dir`'s sign convention — self-
caught and fixed via the test failing loudly, not silently passing).

**`Material` gained two new fields**: `transmission: f32` (`[0,1]`,
default `0.0` — every pre-existing opaque material is unaffected) and
`ior: f32` (default `1.5`, glass-like; unused when `transmission == 0.0`).
Restricted to non-metals by convention (matches glTF's own
`KHR_materials_transmission` restriction — a transmissive metal has no
physical meaning).

**`cpu_ref::shade` changes**: mirrors `ReflectionParams`'s own
`Option<...>` non-recursion contract exactly — new `TransmissionParams`,
`ShadeResult` gained a fourth field `refract: Vec3`, gated on BOTH
`TransmissionParams::enabled` (scene-wide) AND `material.transmission >
0.0` (per-material — a `transmission=0.0` material stays opaque
regardless of the scene-wide toggle, same as `metallic`'s own per-
material override). Needs the full `TraceObject` (shape/transform, not
just material) for the interior march — looked up via `origin_entity`
against `objects`, matching `reflect_trace_ray`'s own probe-and-lookup
pattern rather than widening `shade`'s own signature further. 3 new
tests directly on `shade` itself (mirrors the 2 `shade_reflect_field_*`
tests): `refract` stays zero when the param is `None`; stays zero for an
opaque material even when the scene-wide toggle is on; a near-clear
glass object with `Some(..)` picks up real transmitted energy end-to-end.

**WGSL**: `refract_ray`/`march_object_interior`/`shade_for_refraction_
bounce`/`refract_trace_ray` added directly to `hybrid_trace.wgsl` (no
new file, matching reflection's own "no separate relight pass"
precedent). `shade()` gained the Fresnel-gated transmission call
(Fresnel-complementary to reflection's own share: `1 - fresnel_schlick`
is the transmittable fraction) and now returns `refract_hit_valid`/
`refract_hit_obj_id`/`refract_hit_p_world` — the transmitted ray's own
FIRST-bounce EXIT hit, needed for the dedicated temporal history's
virtual-point reprojection. `tests/wgsl_parse.rs::hybrid_trace_wgsl_parses`
passes.

**A third, fully independent dedicated temporal history — reflection's
own `HybridReflectHistory` precedent extended, not reused.** Transmission's
virtual point (the EXIT surface's own motion, possibly a different,
moving object seen through a bent ray) matches neither the entry
surface's own rigid motion (diffuse GI's case) nor the REFLECTED
surface's motion (reflection's case) — three genuinely different
reprojection targets, so a third buffer, not a second shared one:
- New GPU outputs: `refract_view` (raw transmission color) and
  `refract_motion_view` (the exit hit's own reprojected previous-frame
  world position). The entry surface's own roughness (`refracting_
  roughness`, for the temporal gate) is packed into `motion_view`'s
  previously-unused `.w` channel (confirmed via grep — `motion_tex`'s
  `.w` had no reader anywhere), reusing the identical "repurpose an
  always-1.0 padding channel" trick reflection's own `normal_view.w`
  reuse already established.
- New resources: `HybridTransmitHistory`/`HybridTransmitHistoryRes`
  (own ping-pong slots, invalidated on resize or a `transmission_max_
  bounces` change), `HybridTransmitFrameParity` (a third, independent
  parity counter — not shared with either `HybridFrameParity` or
  `HybridReflectFrameParity`), `HybridTransmitTemporalBindGroup`.
- New WGSL entry point `transmit_temporal_main`, appended to
  `hybrid_temporal.wgsl` (third entry point in the same file — same
  "closely-related logic, separate pipeline" convention). Roughness-
  gated identically to `reflect_temporal_main` (`TRANSMIT_ROUGHNESS_GATE
  = 0.3`, same value/reasoning): below the gate, reproject + blend;
  at/above it, copy-through and rely on the entry surface's own point-
  ray interior march self-blur (transmission's own known limitation —
  see below — makes this gate less load-bearing than reflection's own
  cone-based self-blur, an open question for later).
- `hybrid_denoise.wgsl` composites `accumulated_refract_view` straight
  into the final color with NO spatial blur, same reasoning as
  `reflect_tex`.
- **Learned from reflection's own always-on-tax bug, applied from the
  start this time**: `hybrid_transmit_temporal`'s dispatch in `pass.rs`
  is gated on `scene_data.transmission_enabled` from the very first
  landing (mirrors task-9's fix to reflection, not its original bug) —
  confirmed by measurement below: zero `transmit-temporal` HUD line and
  zero added dispatch cost when disabled.

**Config**: new `TransmissionConfig` resource (`enabled`/`max_bounces`/
`fresnel_cutoff`/`max_t`), independent of both `GiMethod` and
`ReflectionConfig` (a material can be simultaneously reflective AND
transmissive — real glass behavior). `gallery.rs` gets `--transmission
on|off`/`--transmission-bounces N`/`--transmission-fresnel-cutoff F`/
`--transmission-max-t F` plus `--cube-transmission F`/`--cube-ior F` and
a "Transmission" egui section with Transmission/IOR sliders on the cube
material panel; `gi_room.rs` gets the same egui controls (no CLI
tunables, matching that scene's own convention) plus a room-scale
`max_t: 28.0` override and a new clear-glass cube (Zone 5, the room's
previously-empty +X half) for visual verification. Both examples gain a
`transmit-temporal` HUD/log timing line.

**Verified**: `cargo build/test/clippy --release --lib --examples`
clean, 133/133 lib tests (up from 121 — 9 new in `refract_ref`, 3 new on
`cpu_ref::shade` directly). `tests/wgsl_parse.rs`: 3/3 hybrid-file tests
pass; the 2 pre-existing failures (`hybrid_blit`/`splat`) are unrelated,
noted in the prior entry too. Real-GPU-verified via `gallery.rs --shot`:
no panics, no wgpu validation errors, egui panel shows the new
"Transmission" section and cube-material Transmission/IOR sliders
working. **Visual proof it actually refracts, not just shades**: a
diagnostic scene (camera looking straight through a clear-glass cube at
a bright green emissive marker directly behind it) showed the marker's
color clearly visible through the entire cube face, including its own
floor reflection — confirmed the effect is real geometry-through-
geometry transmission, not a shading-only illusion, before reverting
the diagnostic scene changes.

**Real measured cost — `--gi-method conetrace` (default), `--stress
100`, this session's own hardware (AMD Radeon RADV RENOIR), default
cube material with `transmission=0.9`/`ior=1.5` added, steady-state
samples after warmup**:
- `--transmission off`: trace ~58-62ms (matches the pre-transmission
  baseline exactly — zero regression when disabled), NO `transmit-
  temporal` line at all (correctly gated from the start, unlike
  reflection's own original always-on-tax bug).
- `--transmission on` (default, 1 bounce): trace ~63-83ms (avg ~70ms) —
  a real **+8-12ms** increase, `transmit-temporal` ~4-5ms on top.
  Notably cheaper than reflection's own equivalent (+13-18ms after its
  own single-cone-GI fix) — transmission's interior march is a plain
  point ray (no roughness-derived aperture widening, see the known
  limitation below), so it skips the extra widened re-march reflection's
  own two-pass aperture resolution pays for.
- `--transmission on --transmission-bounces 3`: trace ~65-82ms (avg
  ~74ms) — barely more than 1 bounce, consistent with most chains
  terminating after bounce 1 (either the Fresnel-cutoff on throughput,
  or the continuation probe simply not finding further transmissive
  geometry to re-enter in this scene).
- `--reflections on --transmission on` together: trace ~75-95ms (avg
  ~86ms) — consistent with the two features' costs being additive and
  independent (~60 baseline + ~15-18 reflection + ~10-12 transmission),
  no evidence of a cross-feature interaction cost.

**Known, not yet fixed / explicitly out of scope**:
- **The interior march ignores roughness** — unlike `reflect_trace_ray`
  (which re-marches with a real roughness-derived cone once the hit
  material is known), `march_object_interior` always uses a point ray.
  Frosted/rough glass isn't yet visually distinct from clear glass of
  the same transmission/ior. Widening this is a genuinely separate
  problem from reflection's own aperture-widening (the cone would need
  to track TWO refracted directions diverging inside the medium, entry
  and exit, not one), not a straightforward code reuse.
- `TRANSMIT_ROUGHNESS_GATE = 0.3` and `TransmissionConfig`'s defaults
  are first-pass starting points carried over from reflection's own
  values, not independently validated by a sweep against this
  renderer's own material range — same gap reflection's own first
  landing carried, now also true here.
- No visual side-by-side sweep of colored-glass absorption strength
  against real scene materials — the CPU-ref test proves the Beer-
  Lambert formula is wired correctly (green-tinted glass reads green-
  dominant), not that the default absorption curve looks right at this
  renderer's own typical object scale.
- The continuation probe (re-entering a second piece of transmissive
  geometry after exiting the first) is untested on real multi-object
  glass-through-glass scenes — only single-object entry/exit is
  CPU-ref-tested and visually verified.

### Reflection follow-ups: single-cone final-gather (cost fix), always-on temporal-tax fix, roughness-gate sweep

Three items explicitly logged as "known, not yet fixed" in the multi-
bounce specular reflections entry below, closed out together as a
direct follow-up before starting transmission (same session).

**1) The always-on `hybrid_reflect_temporal` dispatch tax, fixed.**
`pass.rs`'s reflect-temporal dispatch is now skipped entirely (not
dispatched at all, not just a cheap copy-through) when
`scene_data.reflection_enabled` is false — previously this pass ran
every frame regardless of the toggle. Measured: the ~3.5-4.4ms tax is
now exactly zero when reflections are off (confirmed by the
`reflect-temporal` HUD field disappearing from the log line entirely,
and total frame time dropping by the same amount).

**2) The measured +25-30ms reflection trace cost, root-caused and
roughly halved.** Root cause: `reflect_trace_ray`'s own "final gather"
termination (`shade_for_reflection_bounce`) was calling
`cone_trace_indirect` — the FULL 5-cone diffuse-GI hemisphere sample —
for its one-bounce GI contribution, not a cheaper approximation. Since
a reflection bounce already costs 2 cone marches of its own (point-ray
probe + aperture-widened re-march), paying for a full 5-cone hemisphere
on top made one reflected pixel cost roughly as much as the ENTIRE
primary-ray GI term a second time. Fix: new `conetrace_ref::
cone_trace_indirect_single` (a single cone straight along the surface
normal, reusing `cone_trace_ray` verbatim — just one direction instead
of five), used by `reflect_ref::shade_for_reflection_bounce`'s own GI
term specifically (primary-ray GI in `cpu_ref::shade`/`hybrid_trace.wgsl`'s
own `shade()` keeps the full 5-cone hemisphere unchanged — that term IS
the dominant indirect contribution for most on-screen pixels, unlike a
reflection bounce's own GI term, which is already second-order relative
to the reflection itself). 3 new tests on the new function (picks up
real bounced light, returns black when nothing's nearby, deterministic).
Measured before/after at `--stress 100`, default material, steady state:
trace cost with reflections on dropped from ~80-91ms to ~68-77ms against
an unchanged ~56-60ms baseline — delta fell from **+25-30ms to +13-17ms**,
roughly the expected halving from cutting 5 cones to 1. Real-GPU-verified
via screenshot: the floor's reflection still shows correctly (roughness-
correct blur, no artifacts) — the reduced GI fidelity in the reflection's
own indirect term is not perceptible at this renderer's current
material/lighting setup.

**3) `REFLECT_ROUGHNESS_GATE = 0.3` and the reflection defaults, swept
and validated (not changed).** Neither example scene had a live-
adjustable roughness on any REFLECTING surface before this (only on the
reflected object) — `gallery.rs`'s floor was hardcoded, making the
temporal-accumulation roughness gate impossible to validate visually.
Added `--floor-roughness`/`--floor-reflectance` to `gallery.rs`
(permanent, reusable for future debugging) and swept floor roughness
from 0.05 (near-mirror) through 0.5 (diffuse-ish) across the
`REFLECT_ROUGHNESS_GATE` boundary (0.29/0.30/0.31) via screenshots: the
cone-aperture blur widens continuously and correctly with roughness, and
no visible discontinuity/pop appears across the gate's own on/off
boundary in static frames (expected — the gate only changes flicker
behavior under camera/object motion, not single-frame appearance, so a
full validation of ITS OWN specific benefit would need a motion/video
capture, not attempted here). Conclusion: `0.3` remains a reasonable,
unchanged default — no evidence surfaced to justify moving it.

**Verified**: `cargo build/test/clippy --release --lib --examples`
clean throughout, no test-count regressions at each step (118 -> 121 ->
133 lib tests across this entry and the transmission entry above,
tracked precisely rather than approximated).

### Multi-bounce specular reflections, roughness-adaptive, own temporal history, WGSL + real-GPU verified

Direct follow-up to the multi-bounce cone-traced GI entry below — this
renderer's metallic-roughness PBR material (`base_color`/`metallic`/
`roughness`/`reflectance`/`emissive`) already computed a per-pixel GGX
Fresnel/alpha for the DIRECT light term, but had no specular INDIRECT
term at all: reflective surfaces (the `gi_room.rs` gold cube,
`metallic=0.9`/`reflectance=0.9`) showed no actual reflection of nearby
geometry. This lands real, multi-bounce, roughness-adaptive specular
reflections, correctly composed with existing shadows and diffuse
cone-traced GI.

**Research first, before writing code**: a background research pass
confirmed there is no SDF-specific reflection literature distinct from
voxel cone tracing (Crassin et al. 2011) — the standard real-time answer
is reusing whatever marching primitive a renderer already has, widened by
a roughness-derived aperture, exactly the substrate `conetrace_ref.rs`'s
own `trace_cone`/`march_object_cone`/`cone_slab_hit` already provide. It
also surfaced the single most consequential finding used below: **a
reflected image's apparent motion does not follow the reflecting
surface's own motion** (this is the standard reason real-time denoisers
like NVIDIA's ReBLUR/NRD use "primary surface replacement"/virtual-point
reprojection for specular) — bolting reflection into the existing
diffuse-GI temporal history would reproject it with the wrong object's
motion the moment either the mirror or the reflected geometry moves.

**Cost-shape decision, a real divergence from diffuse GI's own multi-
bounce design**: `conetrace_ref::cone_trace_ray`'s diffuse bounces
degenerate to a point ray after bounce 1 specifically to avoid `5^depth`
fan-out from its 5-cone hemisphere. A reflection ray has no such fan-out
to begin with — exactly one ray in, one ray out, at every bounce, by
construction (a mirror reflects one direction, not a hemisphere) — so
`reflect_trace_ray` fires a REAL roughness-widened cone at every bounce,
already linear in bounce count without needing that trick. Each bounce
terminates via ONE bounce of diffuse cone-traced GI (`shade_for_
reflection_bounce`, a "final gather" matching Lumen's own documented
`MaxBounces=1` default) rather than nesting a further reflection — so a
reflected surface shows up correctly lit, shadowed, and GI-bounced,
without paying for reflection-inside-reflection.

**New file**: `src/hybrid/reflect_ref.rs` — `reflection_half_angle(alpha)
= atan(alpha) + epsilon` (reuses the EXACT GGX alpha `cpu_ref::shade`
already computes via the newly-extracted `cpu_ref::ggx_alpha`, no new
material field or remap), `reflect_trace_ray` (the bounce-chain loop:
each bounce probes with a point ray first to resolve which material's
roughness applies, re-marches with the real aperture if `half_angle >
0`, shades via `shade_for_reflection_bounce`, accumulates `throughput *
result * coverage^2` — the identical light-leak-safe scaling
`conetrace_ref::cone_trace_ray` already established — then advances
along `reflect(dir, normal)` weighted by that bounce's own Fresnel term),
`MAX_REFLECTION_BOUNCES: u32 = 4` (smaller than diffuse GI's `8`: a
typical dielectric's specular F0 attenuates a bounce chain faster than
diffuse albedo does, so less headroom is needed),
`REFLECTION_FRESNEL_CUTOFF = 0.02` (matches `trace_shadow`'s own
`VIS_CUTOFF`, a reasoned engineering choice stated as such — not
attributed to external literature). 8 new tests: aperture is near-zero
for a mirror and wider for a rough surface; `reflect()` matches the
textbook mirror formula; a mirror floor picks up a reflected emissive
box's own red-dominant color (positive proof, not just "doesn't crash");
higher bounce counts never darken a purely-positive-radiance scene;
bounce count clamps at the compile-time ceiling; a sealed, unlit
mirror-lined box never leaks light through a wall a reflection ray is
geometrically guaranteed to hit (mirrors `conetrace_ref`'s own sealed-
room regression exactly); a low-reflectance dielectric's Fresnel gate
terminates the chain by bounce 2 regardless of how many bounces are
requested.

**`cpu_ref::shade` changes**: `ShadeResult` gained a third field,
`reflect: Vec3` (kept separate from both `direct_and_emissive` and
`indirect` for the identical reason `indirect` already is — see below),
filled in directly by `shade` itself (unlike `indirect`, which stays an
outer-caller-fills-in slot since GI technique is swappable; reflection
isn't a swappable technique, it's a fixed part of PBR specular shading).
A new `Option<ReflectionParams>` parameter controls this: `None` for
`conetrace_ref::cone_trace_ray`'s own bounce-shading call (mirrors
`hybrid_trace.wgsl`'s separate `shade_direct_only_for_cone`, which
structurally can't recurse into reflection since it never calls the real
`shade` at all — the CPU ref shares one `shade` function for both roles,
so this flag reproduces that same non-recursion by parameter instead of
by a second function), `Some(..)` for real primary-ray shading. Every
pre-existing test call site updated to pass `None`, preserving prior
behavior exactly (regression-tested by re-running the full suite
unmodified in assertion value). 2 new tests directly on `shade` itself:
`reflect` stays exactly zero when the param is `None`; a near-mirror
floor with `Some(..)` picks up real reflected energy end-to-end.

**WGSL**: `reflect_trace_ray`/`shade_for_reflection_bounce`/
`reflection_half_angle`/`reflect_luminance` added directly to
`hybrid_trace.wgsl` (no new file — reflection has no separate relight
pass either, same reasoning cone tracing's own doc comment already
gives). `shade()` gained the Fresnel-gated reflection call (skips firing
entirely when the primary view direction's own `fresnel_schlick` term is
below `scene.reflection_fresnel_cutoff`, so flat non-reflective
dielectrics pay zero extra cost) and now returns `reflect_hit_valid`/
`reflect_hit_obj_id`/`reflect_hit_p_world` — the reflection ray's own
FIRST-bounce hit (the geometry actually visible in the mirror this
frame), needed by `trace_main` to compute a SEPARATE virtual-point
reprojection for the specular temporal history (see below). `tests/
wgsl_parse.rs::hybrid_trace_wgsl_parses` passes.

**A genuinely new piece of temporal infrastructure — reflection gets its
OWN ping-pong history, not the diffuse one.** This is the single most
structurally significant part of this landing, directly following the
research finding above:
- New GPU outputs (`HybridTargets`): `reflect_view` (raw multi-bounce
  reflection color) and `reflect_motion_view` (the REFLECTED hit's own
  reprojected previous-frame world position — a "virtual point," NOT the
  reflecting surface's motion — computed via the same `reproject_world_
  point` primitive `motion_view` already uses, just keyed to the
  reflection's own `reflect_hit_obj_id` instead of the primary hit's).
- The reflecting surface's own roughness is packed into `normal_view`'s
  previously-unused `.w` channel (every existing reader only ever touches
  `.rgb` — confirmed by grep before relying on it — so this is non-
  breaking) — read by a new roughness gate (see below).
- New resources (`pipeline.rs`): `HybridReflectHistory`/
  `HybridReflectHistoryRes` (mirrors `HybridHistory` exactly, its own
  ping-pong slots, invalidated on resize or a `reflection_max_bounces`
  change), `HybridReflectFrameParity` (a SEPARATE parity counter from
  `HybridFrameParity`, not shared, since the two histories' own
  invalidation conditions already differ), `HybridReflectTemporalBindGroup`.
  Built inline inside the existing `prepare_hybrid_temporal` system
  (not a separate system) so the diffuse and reflection histories'
  ping-pong resolution both complete before `hybrid_denoise_bind_group`
  is built (which now also reads `accumulated_reflect_view`).
- New WGSL entry point `reflect_temporal_main`, appended to
  `hybrid_temporal.wgsl` (same file, separate compute pipeline/dispatch —
  matches this project's own "same file, another entry point" convention
  for closely-related logic) — reuses `world_to_previous_uv`/
  `disocclusion_rejected`/`temporal_blend` UNCHANGED (the hard part,
  choosing the right point to reproject, already happened in
  `trace_main`; this pass only needed different input textures, not new
  reprojection math). **Roughness-gated**: below `REFLECT_ROUGHNESS_GATE
  = 0.3` (a reasoned cutoff, not a cited threshold — "clearly still
  glossy, not yet diffuse-like" on this renderer's own `alpha =
  roughness^2` remap), behaves exactly like `temporal_main` (reproject +
  blend); at or above it, skips accumulation entirely and copy-throughs
  the current frame's raw value, relying on the reflection cone's own
  spatial self-blur instead — a high-roughness reflection's single hit
  point is a poor stand-in for the whole blurred lobe it represents, the
  same reasoning that makes a single virtual point valid for near-mirror
  surfaces but not rough ones.
- `hybrid_denoise.wgsl` composites `accumulated_reflect_view` straight
  into the final color with NO spatial blur — deliberately: a low-
  roughness reflection needs its sharpness preserved, and a high-
  roughness one already self-blurred spatially via its own wide cone, so
  blurring it again would only cost sharpness for no benefit.
- A real, honest cost admission: `hybrid_reflect_temporal` dispatches
  EVERY frame regardless of whether reflections are enabled (same
  always-on-tax shape ReSTIR's own candidate generation had) — measured
  at ~3.5-4.4ms regardless of the `reflections` toggle, see below.

**New tests**: `tests/wgsl_parse.rs` gained `hybrid_temporal_wgsl_parses`/
`hybrid_denoise_wgsl_parses` (neither file had ANY parse-test coverage
before this session — a real, pre-existing gap, not introduced by this
work — both pass cleanly on the first try after every edit in this
entry).

**Config**: new `ReflectionConfig` resource (`enabled`/`max_bounces`/
`fresnel_cutoff`/`max_t`), independent of `GiMethod` (reflections apply
on top of whichever diffuse-GI technique is active, not a swappable
alternative to it) — threaded through `SceneUniform`/`RenderHybridScene`/
`extract_hybrid_scene`/`prepare_hybrid_scene`'s own established 4-site
pattern. `gallery.rs` gets `--reflections on|off`/`--reflection-bounces
N`/`--reflection-fresnel-cutoff F`/`--reflection-max-t F` plus a
"Reflections" egui section; `gi_room.rs` gets the same egui controls (no
CLI flags, matching that scene's own no-CLI-tunables convention) plus a
room-scale `max_t: 28.0` override mirroring `ConeTraceConfig`'s own
leak-capping reasoning, and a new `hybrid_reflect_temporal` HUD/log
timing line in both examples.

**Verified**: `cargo build/test/clippy --release --lib --examples`
clean, 118/118 lib tests (up from 108 — 8 new in `reflect_ref`, 2 new on
`cpu_ref::shade` directly). `tests/wgsl_parse.rs`: 3/3 hybrid-file tests
pass (`hybrid_trace`/`hybrid_temporal`/`hybrid_denoise`), the 2 other
failures in that binary (`hybrid_blit_wgsl_parses`/`splat_wgsl_parses`)
are pre-existing on `master`, unrelated to this work. Real-GPU-verified
via `gallery.rs --shot` and `gi_room.rs --shot` at `--stress 100`: no
panics, no wgpu validation errors, egui panel shows the new "Reflections"
section with working checkbox/sliders, `gi_room.rs`'s sealed-room-with-
open-roof scene renders with no new light leak versus the identical
`--reflections`-unaffected baseline shot (this scene's own `--no-
reflections` CLI flag doesn't exist — verification here relied on the
egui-default-on screenshot plus the CPU-ref sealed-mirror-box regression
test for the actual leak-proof, matching this project's own "CPU-ref
test is the real proof, the screenshot is a sanity check" precedent).

**Real measured cost — `--gi-method conetrace` (default), `--stress
100`, this session's own hardware (AMD Radeon RADV RENOIR), default cube
material (`metallic=0.0`, `roughness=0.4`, `reflectance=0.5` — a
realistic mid-range dielectric, not a worst-case mirror), steady-state
samples after warmup**:
- `--reflections off`: trace ~55-60ms (matches this renderer's own pre-
  reflection baseline exactly — zero regression when disabled),
  `reflect-temporal` ~3.7-4.4ms (the always-on tax noted above — this
  pass still dispatches every frame regardless of the toggle).
- `--reflections on` (default, 1 bounce): trace ~80-91ms — a real
  **+25-30ms** increase over disabled, meaningfully higher than this
  session's own pre-implementation research estimate (~10-15ms) — stated
  honestly rather than adjusted after the fact. `reflect-temporal`
  unchanged (~3.5-4ms), confirming the extra cost is genuinely in
  `hybrid_trace`'s own reflection ray, not the temporal pass.
- `--reflections on --reflection-bounces 3`: trace ~88-100ms — a more
  modest **+8-9ms** further increase over 1 bounce, consistent with most
  rays terminating via the Fresnel-luminance cutoff (`throughput *= F`
  each bounce) well before reaching bounce 3 on this scene's own
  mid-reflectance default material.

**Known, not yet fixed / explicitly out of scope**:
- `REFLECT_ROUGHNESS_GATE = 0.3` and `ReflectionConfig`'s defaults
  (`max_bounces=1`, `fresnel_cutoff=0.02`, `max_t=60.0`) are first-pass
  starting points, explicitly NOT validated by a real sweep against this
  renderer's own material range — same "measure before claiming
  validated" gap every other technique's own config carries at first
  landing.
- The measured +25-30ms trace-time cost at the default material is
  higher than this session's own pre-implementation estimate — worth a
  future investigation into whether the point-ray probe + second widened
  march (`reflect_trace_ray`'s own two-pass aperture resolution) is
  costing more than expected, rather than assuming the estimate was
  simply wrong.
- `hybrid_reflect_temporal`'s always-on-regardless-of-toggle dispatch
  cost (~3.5-4.4ms) is a real, small, currently-unavoidable tax mirroring
  ReSTIR's own documented always-on candidate-generation cost — not
  gated behind `reflection_enabled` in this landing.
- No visual side-by-side sweep of `REFLECT_ROUGHNESS_GATE`'s own cutoff
  value against real glossy (mid-roughness) materials — the CPU-ref tests
  cover near-mirror (roughness 0.02) and the gate's binary on/off
  behavior, not the perceptual quality of the transition itself.

Not yet committed.

### Multi-bounce cone-traced GI, configurable bounce count, WGSL + real-GPU verified

Direct follow-up to the cone-tracing optimization entry below — cone
tracing was single-bounce only (each shaded pixel's 5-cone hemisphere
sampled direct-lit-only radiance at each cone's own hit, with no further
indirect contribution). User asked for multi-bounce GI, configurable.

**Cost-shape decision, made with the user directly before writing any
code**: naively repeating the full 5-cone hemisphere at every bounce is
`5^depth` — 25 cones/pixel at 2 bounces, 125 at 3, quickly unaffordable
given this renderer's own single-bounce cost (already ~55-60ms trace
time at `--stress 100` on this session's dev hardware). Rejected in
favor of: bounce 1 keeps the full 5-cone hemisphere (quality where it
matters most, closest to the shaded surface); every bounce AFTER the
first fires exactly 1 degenerate point ray (`r0=0`, `half_angle=0`),
continuing straight along the PREVIOUS hit's own surface normal — a
fixed, deterministic direction, not a random cosine sample (this
renderer has no per-pixel RNG primitive today, and cone tracing's own
stateless design — see its module doc comment — has no temporal
accumulation to denoise added per-frame noise away, so introducing RNG
here would trade a real cost for visible fizz/sparkle with no offsetting
benefit). Total cost is therefore `5 + (max_bounces - 1)` cones/pixel —
linear in bounce depth, not exponential.

**Implementation**: `cone_trace_ray` (`conetrace_ref.rs`) rewritten from
a single shade-and-return into an iterative bounce loop (WGSL has no
recursion, so both the CPU ref and its `hybrid_trace.wgsl` mirror are
loops, not recursive calls) — standard rendering-equation accumulation:
`total += throughput * direct_and_emissive * coverage^2` at each hit,
`throughput *= diffuse_color` (`albedo * (1 - metallic)`, re-derived
inline per this module's own established "duplicate small formulas
across functions" convention) before advancing to the next bounce, miss
at any bounce contributes `sky_color` weighted by throughput-so-far and
stops. **The existing sealed-room light-leak fix (`coverage^2` scaling,
not linear) now applies at EVERY bounce, not just the first** — a
prerequisite correctness requirement, not an afterthought: a low-
coverage hit anywhere in the chain could otherwise reopen the exact leak
class this file already fixed twice, just one bounce deeper.
`cone_trace_indirect`'s own 5-cone hemisphere shape is unchanged; each
of its 5 `cone_trace_ray` calls now internally chains up to
`max_bounces` hits.

**`max_bounces == 1` is a hard regression invariant, not just a default
value**: the loop's first iteration always runs the identical shade-
and-accumulate path regardless of `max_bounces`, only deriving a next-
bounce ray when `max_bounces > 1`. Every pre-existing test in
`conetrace_ref.rs`'s own `mod tests` (and the sealed-room regression
test) was updated to pass `max_bounces: 1` explicitly and confirmed to
still pass unmodified in assertion value — the actual regression bar
this work was held to, not a claim.

**New tests** (`conetrace_ref.rs`): `second_bounce_adds_bounced_light_
from_a_lit_neighbor` (a fresh two-box fixture — an unlit floor tile
directly below a bright emissive "glow box," no direct lights in the
scene at all — proves bounce 1 alone shades to pure black while bounce
2 measurably picks up the glow box's own emissive light, the real
positive proof multi-bounce adds light rather than just "doesn't
crash"); `higher_max_bounces_never_darkens_a_scene_with_only_positive_
radiance` (monotonic non-decrease from 1 to 2 to 3 bounces on the same
fixture, guarding against a sign/accumulation-order bug in the running
`total`); `second_bounce_continues_exactly_along_the_hit_normal_not_
reflected_or_negated` (proves the fixed bounce direction is genuinely
the hit's own `+normal`, not `-normal` or a reflected vector, by placing
the glow box only reachable via the exact documented direction);
`real_room_with_sun_and_no_lamp_never_leaks_light_through_a_closed_roof_
at_three_bounces` (re-runs the EXISTING sealed-room fixture at
`max_bounces=3`, confirming the per-bounce `coverage^2` fix holds at
every depth, not just bounce 1 — this is the test that would have
caught a regression there).

**Config**: `ConeTraceConfig::max_bounces: u32`, default `1` (an
unmeasured feature must not change the existing default's output),
plumbed through `SceneUniform`/`RenderHybridScene`/
`extract_hybrid_scene`/`prepare_hybrid_scene`'s own established
4-site pattern (the same one `conetrace_max_t` already uses) into
`hybrid_trace.wgsl`'s own `SceneUniform` mirror. `hybrid_trace.wgsl`
adds `MAX_CONE_BOUNCES: u32 = 8u` as a compile-time loop-bound ceiling
(WGSL loops need a known upper bound), matching the egui slider's own
`1..=8` range. `gallery.rs` gets `--cone-bounces N` (clamped `.max(1)`)
and a "Bounces" slider in `controls_panel`; `gi_room.rs` gets the same
slider in its own controls panel (no CLI flag — that example has none
for any cone-trace tunable).

**Verified**: `cargo build/test/clippy --release --lib --examples`
clean, 107/107 lib tests (20 in `conetrace_ref`, including all new
multi-bounce tests and the 3-bounce sealed-room regression).
`tests/wgsl_parse.rs::hybrid_trace_wgsl_parses` passes. Real-GPU-
verified via `gallery.rs --gi-method conetrace --shot`: `--cone-bounces
1` screenshot confirms the "Bounces" slider present and set to 1 with
no visual regression from this renderer's own established single-bounce
result; `--cone-bounces 3` shows the real, expected color-bleed effect
— a warm tint picked up from a nearby lit cube's own bounced light,
visible in the floor shadow beneath it, entirely absent at 1 bounce (a
neutral-gray shadow) — genuine positive visual proof, not just "doesn't
crash."

**Real measured trace-time cost, `--gi-method conetrace`, `--stress
100`, this session's own hardware (AMD Radeon RADV RENOIR), steady-
state samples after warmup**:
- `--cone-bounces 1`: ~56-60ms (matches this renderer's own pre-multi-
  bounce single-bounce baseline exactly — confirms zero cost regression
  at the default).
- `--cone-bounces 3`: ~82-89ms — a real ~1.45-1.55x increase over
  1-bounce, close to the `(5 + 2) / 5 = 1.4x` cone-count-ratio estimate
  (the small excess above the pure ray-count ratio is each extra
  bounce's own `trace_cone` BVH-descent overhead, not just the march
  itself).

### Consolidated on SDF cone tracing: DDGI, hash-grid, and ReSTIR removed — cone tracing is now the sole GI technique

Direct follow-up to the ReSTIR entry below (and the DDGI/hash-grid/cone-
trace entries further down) — the user reviewed all four techniques'
own real, measured cost/quality tradeoffs and made an explicit decision:
standardize on cone tracing alone. Cone tracing is architecturally
native to this renderer — it marches the exact same signed-distance
field primary rays already use, with no spatial quantization (unlike
DDGI's probe grid or the hash-grid's cells) and no temporal lag (unlike
all three of the others, which amortize/converge across frames). This
is not a verdict that the other three were built wrong — DDGI/hash-grid/
ReSTIR are each correct, real-GPU-verified implementations with their
own entries below, kept as durable historical record per this file's own
append-only convention — it's a scope decision to stop maintaining four
parallel techniques and optimize one instead (see the entry above for
that optimization work).

**Removed, wholesale**: `src/hybrid/ddgi_ref.rs`, `hashgrid_ref.rs`,
`restir_ref.rs` (deleted; `ddgi_tangent_basis` copied into
`conetrace_ref.rs` as `cone_tangent_basis` first, since `cone_trace_
indirect` genuinely depends on that one function — zero behavior
change). `assets/shaders/hybrid_ddgi_relight.wgsl`, `hybrid_hashgrid_
update.wgsl`, `hybrid_restir_temporal.wgsl`, `hybrid_restir_spatial.wgsl`
(deleted). `GiMethod` enum simplified to `{None, ConeTrace}`;
`DdgiConfig`/`HashGridConfig`/`RestirConfig` deleted; `SceneUniform`/
`RenderHybridScene` trimmed to only the fields cone tracing needs
(removing ~20 `ddgi_*`/`hashgrid_*`/`restir_*` fields). `pipeline.rs`
lost ~700 lines: every DDGI/hash-grid/ReSTIR bind-group-layout function,
GPU resource type, `prepare_hybrid_ddgi`/`prepare_hybrid_hashgrid`/
`prepare_hybrid_restir` systems, and `trace_pipeline`'s own bind-group
layout list shrank from 5 groups back to 2 (view, compute) — cone
tracing has never needed a dedicated bind group of its own. `pass.rs`'s
`hybrid_pass` shrank back to trace → temporal → denoise → blit, with the
`GiMethodBindGroups` `SystemParam` bundle (only needed once ReSTIR
pushed past Bevy's system-param arity ceiling) removed now that the
param count is well under it again. `hybrid_trace.wgsl` dropped from
1996 to 1287 lines: `GI_METHOD_DDGI`/`_HASHGRID`/`_RESTIR` consts and
branches, the DDGI probe-sampling section, the hash-grid sampling
section, the `Reservoir` struct and all `restir_*`/`pcg_hash`/
`cosine_hemisphere_sample`/RIS functions, group-2/3/4 bindings, and
`trace_main`'s own candidate-generation-and-write block all deleted.
Kept: `HEMISPHERE_SAMPLES`/`ddgi_tangent_basis` (still called directly
by `cone_trace_indirect`), `cone_sky_color`, `shade_direct_only_for_cone`,
all `cone_trace_*`/`march_object_cone`/`trace_cone`/`cone_radius_at`
code. `tests/wgsl_parse.rs` lost its 4 deleted-file parse tests.
`examples/gi_room.rs`/`gallery.rs` lost all DDGI/hash-grid/ReSTIR CLI
flags, egui controls, and `FpsStats` GPU-timing fields (`--ddgi
on|off`'s legacy alias removed too — `--gi-method conetrace|none` is the
real, current CLI surface); both now default to `GiMethod::ConeTrace`.

**Verified**: `cargo build/test/clippy --release --lib --examples`
clean, 100/100 lib tests immediately post-removal (before the 3a/3b/3c
optimization work above added more). `tests/wgsl_parse.rs::
hybrid_trace_wgsl_parses` passes (`hybrid_blit_wgsl_parses`/
`splat_wgsl_parses` already failed for pre-existing, unrelated reasons
before this session — not touched). `conetrace_ref`'s own sealed-room
leak regression test (`real_room_with_sun_and_no_lamp_never_leaks_
light_through_a_closed_roof`) re-confirmed passing, since so much of
this file was touched by the removal.

### Cone tracing's own BVH optimization: per-node margin tightening shipped, step-cap scaling proven a dead end, occupancy-grid gate built/measured/reverted (negative result, matches Stage C)

Direct follow-up to the consolidation entry above — three tasks, one
shipped, one abandoned on a correctness proof before any code was
written, one built/measured/reverted on real numbers.

**3a — per-node margin tightening (shipped).** `trace_cone`'s own BVH
descent previously padded every node's AABB by the cone's single
worst-case radius at `t_max`, computed once outside the descent loop.
Replaced with `cone_slab_hit` (`conetrace_ref.rs`): a two-pass,
provably-safe tightening — pass 1 pads by the caller's own worst-case
bound to get a definitely-safe `t_near`, pass 2 re-pads using
`cone_radius_at(r0, half_angle, t_near)` (the tighter, node-local
radius) and re-tests. Since `cone_radius_at` is monotonically
non-decreasing in `t`, the true padding any node could ever need is
bounded by the pass-1 `t_near`'s own radius — refinement can only shrink
the pruned-away region, never turn a real hit into a miss. 5 new CPU-ref
tests prove: a near node gets meaningfully tighter padding than the old
fixed-margin behavior; a far node's padding still converges to
approximately the old worst-case value; `trace_cone`'s own end-to-end
hit result for a real scene is unchanged by the tightening (only the
pruning path differs, not the answer). Mirrored to `hybrid_trace.wgsl`
as `cone_slab_hit`, applied at both the leaf and internal-node slab
tests in `trace_cone`'s own descent. `tests/wgsl_parse.rs::
hybrid_trace_wgsl_parses` passes.

**3b — step-cap scaling by cone width (abandoned, correctness proof
first).** The plan's original ask: derive a tighter, half_angle-
dependent `MAX_CONE_MARCH_STEPS` cap, since a wide cone's radius grows
faster and should converge in fewer steps. Before writing any code, had
the bound rigorously derived from first principles: `march_object_cone`'s
step formula (`t += max(d - radius(t), MIN_CONE_STEP)`) only guarantees
forward progress via the SDF's own 1-Lipschitz property
(`|d(t2) - d(t1)| <= |t2 - t1|` along the ray) — the same guarantee that
makes marching correct at all. Under that guarantee ALONE, an adversarial
SDF can track `radius(t)` exactly at every step (setting
`d(t_i) = radius(t_i) + epsilon` at each sampled point, which the
Lipschitz condition permits since `MIN_CONE_STEP = 1e-3` is far smaller
than the Lipschitz slope bound of 1), collapsing every step to exactly
`MIN_CONE_STEP` regardless of `half_angle` — a wide cone's own faster-
growing `radius(t)` doesn't help, because the adversary is reactive, not
fixed in advance. The only provably-safe bound is therefore the existing
`ceil((t_max - t_start) / MIN_CONE_STEP)` one, independent of cone
width — no real per-cone-width tightening is mathematically possible
without an additional geometric assumption beyond generic 1-Lipschitz
(e.g. a scene-specific curvature bound), which this renderer doesn't
have and wasn't asked to add. Decision: drop 3b entirely rather than
ship an unproven heuristic. `MAX_CONE_MARCH_STEPS` is unchanged.

**3c — occupancy-grid DDA gate (fresh prototype, built, measured,
reverted).** Informed by (not reusing) the now-deleted Stage C shadow-ray
gate's own two lessons (ray-aware DDA only, not point-neighborhood;
fail-safe fallback needs its own dedicated test) — built a fresh
`ConeOccupancyGrid` (single-level bounded grid, rasterized from BVH leaf
AABBs, `MAX_GRID_CELLS` overflow fallback with an explicit
`always_occupied` field checked first), a ray-aware Amanatides-Woo 3D-DDA
gate (`ray_is_definitely_unoccluded`), and a cone-width-aware version
(`cone_occupancy_gate`) that widens the DDA walk's own cell check to a
small neighbor cube bounded by `MAX_PADDING_CELL_RADIUS`, falling
through conservatively (never claiming a false skip) when a cone is too
wide for cheap padding. 11 CPU-ref tests, including a regression test
mirroring Stage C's own `max_grid_cells_overflow_fallback_is_
conservative_not_permissive` (the exact class of bug — an inverted
fail-safe boolean — Stage C's own investigation found and fixed) and two
end-to-end tests proving `trace_cone_gated` returns identical hits to
ungated `trace_cone` on both real hits and genuine misses. Mirrored to
`hybrid_trace.wgsl` (packed occupancy bits, 32 cells/u32, a new
`occupancy_cells` binding in `hybrid_compute_layout`, `SceneUniform`
grid-placement fields, `--occupancy-gate on|off` / egui checkbox
escape hatch matching Stage C's own precedent exactly).

**Real GPU measurement, `--gi-method conetrace`, this session's own
hardware (AMD Radeon RADV RENOIR)**: gate OFF vs. gate ON, steady-state
`hybrid_trace` pass time, 8+ once-a-second samples each after discarding
warmup —
- `--stress 100`: gate off ~54-59ms, gate on ~54-58ms — statistically
  indistinguishable.
- `--stress 10000`: gate off ~102-109ms, gate on ~99-110ms —
  statistically indistinguishable.

No panics, no validation errors, no visual corruption at either scale
(confirmed via `--shot` — cubes shaded correctly, soft shadows and
indirect bounce visible, occupancy checkbox/slider present and enabled
in the egui panel). **Verdict: correct, safe, and a measurable no-op —
the same result Stage C's own shadow-ray gate found.** Root cause
inferred the same way: `trace_cone`'s own margin-padded BVH descent
(now additionally tightened by 3a above) already prunes empty space
cheaply at the tree-node level; the occupancy grid's own DDA walk is
paying real per-ray setup/traversal cost to answer a question the BVH
was already answering almost as cheaply, while never reducing the
actual dominant cost (5 cones/pixel × exact candidate-march work for
cones that DO find real geometry).

**Decision: revert the mechanism**, matching Stage C's own discipline —
correct and safe as it was, it added real surface area (a new CPU-ref
module section, a new GPU buffer/binding, 9 new `SceneUniform` fields,
a WGSL gate branch, CLI/egui controls) for zero measured benefit.
Reverted: `conetrace_ref.rs`'s `ConeOccupancyGrid`/`ray_is_definitely_
unoccluded`/`cone_occupancy_gate`/`trace_cone_gated` and their 11 tests,
`pipeline.rs`'s `occupancy_cells` buffer/binding/bind-group entry and
grid-build block in `prepare_hybrid_scene`, `extract.rs`'s
`ConeTraceConfig`/`SceneUniform`/`RenderHybridScene` occupancy fields,
`hybrid_trace.wgsl`'s occupancy binding/functions/gate call site,
`gallery.rs`'s `--occupancy-gate`/`--occupancy-cell-size` CLI flags and
egui controls. What's kept: this writeup (the negative result, the two
carried-forward Stage C lessons it confirmed still apply, and the real
measured numbers), 3a's real shipped optimization, and 3b's correctness
proof (durable knowledge that this specific tightening isn't
achievable without a stronger geometric assumption).

**Verified**: `cargo build/test/clippy --release --lib --examples`
clean, 103/103 lib tests (16 in `conetrace_ref` including the sealed-room
leak regression, re-confirmed still passing after all of 3a/3b/3c's
edits). `tests/wgsl_parse.rs::hybrid_trace_wgsl_parses` passes
(`hybrid_blit_wgsl_parses`/`splat_wgsl_parses` still fail for the same
pre-existing, unrelated reasons noted in the consolidation entry above —
untouched by this work). Real-GPU numbers for 3a alone (post-revert,
`--gi-method conetrace`): `--stress 100` trace ~54-67ms, `--stress
10000` trace ~99-120ms — within the same range as this session's
pre-3a/pre-3c baseline runs; this shared, actively-loaded dev machine's
own run-to-run variance (consistent with Stage C's own noted 20-40%
single-sample variance) is wide enough that isolating 3a's own margin-
tightening delta from noise would need a longer, controlled sweep this
session didn't run — 3a's real value is the proven-correct tightening
itself (validated by the CPU-ref test suite), not a claimed speedup
number.

### ReSTIR GI: fourth selectable GI technique, spatiotemporal reservoir resampling, WGSL + real-GPU verified

Direct follow-up to the DDGI/hash-grid/cone-trace entries below — lands
`GiMethod::Restir`, the fourth technique in this renderer's GI
comparison. Structurally distinct from all three others: DDGI, the
hash-grid, and cone tracing each cache or re-derive an INDIRECT-
IRRADIANCE VALUE at some granularity (a probe grid cell, a hashed
spatial cell, or nothing at all). ReSTIR reuses the actual TRACED
SAMPLE itself — a real ray-traced hit point plus its shaded radiance —
resampled statistically across neighboring pixels and previous frames
via weighted reservoir sampling (RIS), turning 1 noisy sample/pixel/
frame into an effectively much larger sample count. This is the real
state-of-the-art comparison point the other three techniques are
implicitly measured against.

**Design**: 1 fresh cosine-hemisphere candidate per pixel per frame
(the classic ReSTIR GI shape, Ouyang et al. 2021), full generalized-
balance-heuristic MIS for both temporal and spatial reuse (not a
biased shortcut), one-frame lag (`shade()`'s `GI_METHOD_RESTIR` branch
reads the PREVIOUS frame's fully-resolved reservoir — the same lag
idea `hybrid_temporal.wgsl`'s own history already uses). Two new
compute passes run after `trace_main`, before the existing temporal-
accumulation pass: `hybrid_restir_temporal.wgsl` (2-way combine: this
frame's fresh candidate vs. the reprojected previous reservoir) then
`hybrid_restir_spatial.wgsl` (k-neighbor combine, `k+1`-way full
pairwise MIS), producing this frame's resolved reservoir for shading
to read NEXT frame.

**A genuinely new primitive this codebase has never needed before**: a
pseudorandom hash. Every existing GI technique samples a FIXED
5-direction hemisphere bundle (`ddgi_ref::HEMISPHERE_SAMPLES`) or a
deterministic cone-radius formula — none needed per-pixel-per-frame
randomness. Added a single-round PCG-style `u32 -> u32` hash
(`restir_ref::pcg_hash`), chosen over a bare wang-hash for stronger
avalanche (confirmed via a real adjacent-input collision/avalanche
test), with no 64-bit state (WGSL has no native `u64`). Seeded by
`(pixel_index, frame_index, stream)`, where `stream` distinguishes
independent draws in the same frame without needing persistent
per-pixel RNG state.

**Reservoir data shape and RIS math** (`restir_ref::Reservoir`): sample
position/normal/radiance, the ORIGIN point that generated it (required
for the reconnection Jacobian, below), `weight_sum`/`m`/`w` — the
standard RIS bookkeeping. `W = weight_sum / (m * target_pdf(sample))`,
target function = Rec.709 luminance (the standard choice for a
diffuse-only cache with no BRDF lobe to weight against).

**A real math bug found and fixed during this technique's own
development**: the first `balance_heuristic_combine` implementation
pulled a candidate's own `target_pdf` out as a constant multiplying
every term of its own resampling denominator — since it then divided
by that same denominator, `target_pdf` canceled out of the weight
formula ENTIRELY, silently defeating the whole point of importance-
weighted resampling (every candidate would have been weighted purely
by geometric Jacobian/visibility, never by how bright it was). Caught
by two statistical tests (`restir_temporal_combine_two_candidates_
matches_a_brute_force_balance_heuristic_reference`,
`restir_spatial_combine_matches_the_unweighted_monte_carlo_average_
across_many_trials`) whose own hand-derived "expected" values were
ALSO wrong in the same session (a second, independent mistake — the
test's own reference formula didn't include the Jacobian on every
term either) — fixed by rewriting both tests' expected values via a
`brute_force_balance_weights` helper that independently re-implements
the doc-commented formula rather than hand-deriving a closed form,
confirming the implementation and an independent reference now agree
via repeated-trial statistics. A real, honest reminder that "the
implementation and my hand check both look right" isn't the same as
"a brute-force independent derivation confirms it."

**Visibility re-evaluation** (the single most common ReSTIR correctness
bug to skip): every reuse step — temporal AND spatial — re-traces a
real shadow ray (`trace_shadow`, this codebase's existing soft-shadow
primitive, its `[0,1]` result used directly as a multiplicative weight
rather than collapsed to a hard bool) from the CURRENT shading point to
a reused candidate's own `sample_pos` before allowing it to contribute.
A neighbor's/history's own sample can be occluded from a DIFFERENT
viewpoint even if it wasn't from its own generating point — skipping
this is the standard ReSTIR bias bug. This required widening both new
passes' own bind groups to include the object/BVH/light data every
other self-contained per-pass WGSL file in this codebase already
duplicates (`hybrid_ddgi_relight.wgsl`'s/`hybrid_hashgrid_update.wgsl`'s
own precedent) — an addition beyond this feature's original bind-group
design, caught while writing the WGSL (the first draft approximated
visibility via depth/normal continuity alone, which was recognized as
an under-scoped shortcut relative to the CPU-ref's own real
`trace_shadow` call and corrected before landing).

**New files**: `src/hybrid/restir_ref.rs` (22 tests — PRNG determinism/
avalanche, cosine-hemisphere distribution verified via histogram
against the analytic cos(theta)/pi density, reservoir update/finalize
degenerate cases, the two brute-force MIS proofs above, reconnection-
Jacobian degenerate cases, initial-candidate generation on hit/miss).
`assets/shaders/hybrid_restir_temporal.wgsl`/`hybrid_restir_spatial.wgsl`
— self-contained per this codebase's established per-pass convention
(own `SceneUniform`/`ObjectGpu`/`BvhNode`/`LightGpu`/`trace`/
`trace_shadow` copies, no shared-include mechanism exists).
`restir_ref.rs` test count: 22, 167 total lib tests (up from 145).

**Pipeline wiring**: `RestirConfig` (new `extract.rs` resource:
`max_t`/`max_m`/`spatial_neighbor_count`/`spatial_radius_px`), 5 fields
appended to `SceneUniform`/`RenderHybridScene` (Rust) and `SceneUniform`
(WGSL, all 3 self-contained pass files), `ReservoirGpu` (60 bytes, 4
clean `vec4`s). `HybridRestir`'s own reservoir buffers ARE ping-ponged
(unlike DDGI's atlas/the hash-grid's table, which are single `read_write`
buffers since each slot is only ever touched by one invocation) — a
reservoir genuinely needs the ping-pong split `HybridHistorySlot`
already established, since temporal reuse reads LAST frame's resolved
slot while writing THIS frame's, the same cross-invocation read-a-
different-location-than-you-write race that pattern exists to solve.
`trace_main` gained a new group 4 (candidate write + previous-resolved
read) and `shade()`'s own `pixel_index` parameter (needed to index the
per-pixel reservoir array — `shade()` previously had no access to
`gid`).

**A real, deliberate asymmetric cost decision**: `trace_main` writes
its initial candidate UNCONDITIONALLY every frame, not gated on
`GiMethod::Restir` — unlike DDGI's/the hash-grid's own SEPARATE relight
passes, `trace_main` itself is never skippable, and a same-shader `if`
buys little when a full-screen dispatch has every invocation take the
same branch anyway. This is ReSTIR's one real always-on tax versus the
other 3 techniques (each pays zero extra `trace_main`-body cost when
inactive) — reported honestly here, not hidden behind a branch that
would look free but isn't.

**UI/CLI**: `gi_room.rs`/`gallery.rs` both gained the "ReSTIR" radio
option + 4 sliders (candidate ray reach, reservoir confidence cap,
spatial neighbor count, spatial search radius), gated on
`GiMethod::Restir`; `gallery.rs` gained `--restir-max-t`/
`--restir-max-m`/`--restir-spatial-neighbors`/`--restir-spatial-radius`
(`--gi-method restir` added to the existing match arm). New
`gpu_restir_temporal_ms`/`gpu_restir_spatial_ms` HUD lines in both
files' timing displays.

**Verified**: `cargo build/test/clippy --release --lib --examples`
clean, 167/167 lib tests. `tests/wgsl_parse.rs::hybrid_restir_temporal_
wgsl_parses`/`hybrid_restir_spatial_wgsl_parses` (new) and
`hybrid_trace_wgsl_parses` (updated) all pass — naga's full validator
catches any `ReservoirGpu` storage-buffer alignment mistake before
real-GPU testing. Real-GPU-verified via `gallery.rs --gi-method restir
--shot` at `--stress 100`/`--stress 10000`: no panics, no wgpu
validation errors, radio button switches cleanly.

**Real GPU cost — the honest, expected result**: unlike the other 3
techniques' own single relight/update pass, ReSTIR's spatial-reuse
pass is genuinely `O((k+1)^2)` per pixel per frame (the "full pairwise"
unbiased MIS cost, `restir_ref::restir_spatial_combine`'s own doc
comment) — at the default `k=5`, that's 36 Jacobian+visibility
evaluations per pixel, and it shows up exactly as predicted, dominating
every other pass by an order of magnitude:
- `--stress 100`: `restir_temporal` ~24-30ms, `restir_spatial`
  ~567-662ms, `trace` ~40-45ms (vs. `None` baseline — not independently
  re-measured this session, see the hash-grid/cone-trace entries below
  for this hardware's own recorded `None` numbers at this scale).
- `--stress 10000`: `restir_temporal` ~41-43ms, `restir_spatial`
  ~897-973ms, `trace` ~72-75ms (vs. `None` baseline `~68-77ms` trace,
  measured this session on the same hardware — confirming `trace`'s own
  delta from ReSTIR's always-on candidate generation is small, a few ms,
  exactly as the "one real asymmetric tax" reasoning above predicts;
  the spatial-reuse pass's own cost dwarfs it entirely).

This is meaningfully more expensive than any of DDGI's (`~99-151ms`
total trace time at `--stress 10000`), the hash-grid's (`~135-208ms`),
or cone tracing's own recorded costs — the real, now-measured price of
"full pairwise unbiased MIS" at a real `k`, not glossed over. Whether
the quality this buys (genuine multi-sample reuse, no grid/cell
quantization) is worth this cost at the current unswept `k=5` default
is an open question a future `k` sweep should answer, not asserted
here.

**Known, not yet fixed / explicitly out of scope**:
- `RestirConfig`'s 4 defaults (`max_t=60.0`/`max_m=24.0`/
  `spatial_neighbor_count=5`/`spatial_radius_px=20.0`) are first-pass
  starting points, explicitly NOT validated by a real sweep — same
  "measure before claiming validated" gap every other technique's own
  config carries, but here `spatial_neighbor_count` directly controls
  an `O(k^2)` cost, making this sweep more consequential than the other
  techniques' own unvalidated defaults.
- Single-bounce only (the initial candidate's own hit shades direct-lit
  only, via the same `shade_direct_only_for_cone`/`cone_sky_color`
  primitives cone tracing's own candidate ray already uses) — matches
  every other technique's own scope limit.
- No adaptive sample count — always exactly 1 initial candidate/pixel/
  frame.
- Visibility re-evaluation always re-traces fresh, no caching of a
  "last known visible" result across frames — the conservative, simple,
  safe choice, same as the CPU-ref's own design.
- `gi_room.rs` starts cleanly with its own room-scale `RestirConfig`
  override (`max_t=28.0`, mirroring the other 3 techniques' own
  leak-capping reasoning), reaches pipeline READY, radio button
  switches cleanly over a manual run — the visual side-by-side
  comparison itself needs the user's own manual interactive check per
  this scene's established screenshot-free convention; not yet done.

Not yet committed.

### SDF cone tracing: third selectable GI technique, no persistent structure, WGSL + real-GPU verified

Direct follow-up to the `GiMethod` selector and hash-grid entries below —
this lands `GiMethod::ConeTrace`, the third and final technique from this
session's GI-comparison plan. Unlike DDGI and the hash-grid, cone tracing
maintains **no persistent GPU structure at all**: every shaded pixel
re-marches its own 5-cone hemisphere bundle directly against the scene's
existing BVH/SDF representation, fresh, every single frame — no relight
pass, no probe atlas, no hash table. This has a real, honest cost
tradeoff, measured below: DDGI/the hash-grid both amortize their own
relight cost across many frames via a persistent cache; cone tracing has
**zero temporal amortization**, so its entire cost lands in every
frame's own shading-time budget.

**Core formula**: effective hit-test radius grows as `radius(t) = r0 +
t * tan(half_angle)` (the standard cone-radius-at-distance formula);
the hit test generalizes from `d < HIT_EPSILON` to `d < radius(t)`; the
step size becomes `t += max(d - radius(t), MIN_CONE_STEP)` (a naive
`t += d` would overstep once the cone's own effective volume exceeds the
raw SDF distance). Coverage — a continuous `[0,1]` attenuation, NOT a
boolean hit — is `smoothstep(radius(t), 0.0, d)` at convergence,
following this session's own established discipline for continuous
quantities (DDGI's occluded-probes fallback needed the identical
hard-branch-to-smoothstep fix, twice, earlier this session).

**The correctness anchor**: `cone_degenerates_to_point_ray_at_zero_half_angle`
proves `half_angle=0.0` converges at (within 0.01 units of) the exact
same `t` a plain point-ray march finds — confirming the cone
generalization is a strict superset of the original point-ray behavior,
not a silent change to it. Getting this test right surfaced a real,
useful finding: **coverage's exact value at convergence is sensitive to
march-step granularity**, not just cone geometry — `MIN_CONE_STEP`'s own
coarse floor can make the final step overshoot deep into a wide cone's
radius even on a dead-center hit, so a "dead-center hit" test needed a
near-zero half-angle to get a reliably high coverage value, and the
"wider cone smooths more" test was redesigned around a robust hit/no-hit
signal (a narrow cone structurally missing geometry a wide cone reaches)
rather than a noisy coverage-delta threshold. Documented in
`conetrace_ref.rs`'s own test comments so this isn't rediscovered blind
if cone tracing's math is touched again.

**New file**: `src/hybrid/conetrace_ref.rs` (11 tests — `ddgi_tangent_basis`
imported directly from `ddgi_ref` per the precedent `hashgrid_ref.rs`
already set for byte-identical cross-module helpers; `HEMISPHERE_SAMPLES`
itself duplicated as a small private literal array, same 5 fixed
directions DDGI/the hash-grid both already use). Required one small,
justified companion change in `cpu_ref.rs`: `local_distance`/
`local_normal`/`smoothstep` widened from private to `pub(crate)` (same
visibility tier `sky_color` already uses) — reusing the existing
per-shape SDF dispatch rather than duplicating that `match` a third
time. `conetrace_ref.rs` test count: 11, 143 total lib tests (up from
132 after the hash-grid landing).

**WGSL — no new file**, confirming the plan's own "cheapest of the
three techniques" claim: `cone_radius_at`/`march_object_cone`/
`trace_cone`/`cone_trace_ray`/`cone_trace_indirect` all added directly
to `hybrid_trace.wgsl`, mirroring `conetrace_ref.rs`'s functions 1:1.
Two things needed adding that didn't already exist in this file: a
`cone_sky_color` gradient (this file's own primary rays use a flat
`scene.background_*` uniform, not a sky gradient — a cone's own ray,
like DDGI's/the hash-grid's relight rays, needs the escape-to-sky
fallback those files already have) and `shade_direct_only_for_cone`
(this file's own `shade()` already bakes indirect-diffuse in by reading
`scene.gi_method` itself, so calling `shade()` from a cone hit would
recurse `cone_trace_indirect` into itself — needed the same
direct-lit-only shading DDGI's/the hash-grid's own relight passes
already carry their own copy of). `shade()`'s dispatch chain gained the
third `else if (scene.gi_method == GI_METHOD_CONETRACE)` branch, same
`indirect = diffuse_color * irradiance;` shape as the other two.

**Pipeline wiring — confirmed, not just planned, to need nothing beyond
`SceneUniform`**: `ConeTraceConfig` (new `extract.rs` resource:
`cone_half_angle`/`cone_origin_radius`/`max_t`), 3 fields appended to
`SceneUniform`/`RenderHybridScene` (Rust) and `SceneUniform` (WGSL), 3
fields appended to `prepare_hybrid_scene`'s literal. No `pass.rs` gate,
no new bind group, no new buffer, no new `prepare_hybrid_*` system —
cone tracing has no persistent structure to gate a relight pass for.
`gi_room.rs` sets `max_t=28.0` (mirroring `DdgiConfig`'s/
`HashGridConfig`'s own room-scale leak-capping value exactly, this
room's ~25.2-unit diagonal), leaving `cone_half_angle`/
`cone_origin_radius` at their crate defaults.

**UI/CLI**: `gi_room.rs`/`gallery.rs` both gained the "Cone-trace" radio
option + 3 sliders (half-angle, origin radius, reach), gated on
`GiMethod::ConeTrace`; `gallery.rs` gained `--cone-half-angle`/
`--cone-origin-radius`/`--cone-max-t` (`--gi-method conetrace` already
parsed correctly before this landing — confirmed, no change needed
there). No new GPU-timing HUD line in either panel — cone tracing has
no separate pass to time; its entire cost is a delta in the existing
`hybrid_trace` line, noted explicitly in a comment at that call site.

**Verified**: `cargo build/test/clippy --release --lib --examples`
clean, 143/143 lib tests. `tests/wgsl_parse.rs::hybrid_trace_wgsl_parses`
passes (validates the whole file including the new cone-tracing block —
no new test needed, this file was already covered). Real-GPU-verified
via `gallery.rs --gi-method conetrace --shot`: no panics/validation
errors at `--stress 100` or `--stress 10000`, correct shading (proper
shadows, no black holes, no NaN corruption) at both scales.

**The fair cost comparison — real numbers, trace-time delta**: cone
tracing has no separate pass to time, so its whole cost is the increase
in `hybrid_trace`'s own pass time relative to a `GiMethod::None`
baseline, measured on the same hardware (AMD Radeon RADV RENOIR,
Vulkan) this session's DDGI/hash-grid numbers came from:
- `--stress 100`: `None` baseline `~19-22ms` trace → `ConeTrace`
  `~163-177ms` trace → **delta ≈ 145-155ms**.
- `--stress 10000`: `None` baseline `~33-35ms` trace → `ConeTrace`
  `~270-290ms` trace → **delta ≈ 240-255ms**.

This is meaningfully higher than DDGI's own total trace-time footprint
(`~99-151ms` at `--stress 10000`, most of which is the base trace cost
itself, DDGI's own atlas read being cheap) or the hash-grid's
(`~135-208ms` total trace time at the same scale) — the real, now-
measured price of firing 5 full cone marches per shaded pixel every
frame with nothing cached across frames, exactly the tradeoff this
technique's own module doc comment flags honestly rather than hides.
Whether this cost is worth cone tracing's own quality characteristics
(continuous, no grid/cell quantization artifacts at all, unlike DDGI's
blocky probe grid or the hash-grid's cell boundaries) is a real
question for the user's own visual comparison in `gi_room.rs` — this
entry reports the objective cost side honestly, not a verdict on the
tradeoff.

**Known, not yet fixed / explicitly out of scope**:
- `cone_half_angle`'s default (`0.15` rad) is a first-pass starting
  point, explicitly NOT validated by a real sweep against this
  renderer's own object scale — mirroring the soft-shadow `k=12`→
  `k<=2.0` sweep precedent this project already has. A future sweep is
  a natural follow-up, not part of this landing.
- Single-bounce only, matching DDGI's/the hash-grid's own scope limit —
  a cone hit's own shading has no further indirect recursion, since
  this technique specifically has nothing to amortize a second bounce's
  cost against.
- No temporal amortization of any kind — the real, measured (not
  hidden) structural cost disadvantage reported above.
- `gi_room.rs` starts cleanly with its own room-scale `ConeTraceConfig`
  override, reaches pipeline READY, no panics over a manual run — the
  visual side-by-side comparison itself needs the user's own manual
  interactive check per this scene's established screenshot-free
  convention.

Committed `d9db4a6`. See the follow-up entry directly below for two real
light-leak bugs this initial landing had — found via the user's own
interactive testing in `gi_room.rs`, not caught by this entry's own
tests/verification above.

### SDF cone tracing follow-up: sealed-room light leak, two real bugs found via interactive testing

The `d9db4a6` landing above passed every CPU-ref test, `clippy`, and a
`gallery.rs --shot` sanity check — none of which caught this. The user
found it immediately by switching to Cone-trace in `gi_room.rs`'s own
interactive scene (roof sealed, "opens in 3.0s"): the room read as
visibly, if dimly, lit despite being fully closed. This entry covers two
distinct bugs behind that one symptom, found by iterating directly on the
user's own reports rather than by re-deriving them from first principles.

**Bug #1 — coverage used as a sky-blend weight.** `cone_trace_ray`
originally blended `shade_direct_only_for_cone(...) * hit.coverage +
cone_sky_color(ray_dir) * (1.0 - hit.coverage)`, on the assumption that a
low-coverage hit meant the cone had partially "escaped" to open sky. That
assumption is wrong: a low-coverage hit is still a real hit on real
geometry (see Bug #2), so blending toward sky color leaked the room's
actual sky-color constant through sealed walls. Fixed by removing the
sky blend entirely — any hit returns its shaded color unconditionally;
only a true miss (`!hit.did_hit`) returns `cone_sky_color`. Necessary,
but the user's direct follow-up ("i still see same artifacts... has some
visible splats") proved this alone was insufficient — a patchy, localized
symptom (splats on roof/walls/a little on the floor), not the uniform
wash Bug #1's fix would have addressed on its own.

**Bug #2 — the actual "splats" bug: low-coverage hit positions can float
off the true surface.** A cone's convergence `t` is only accurate to
within `radius(t)` of the true surface — `march_object_cone` stops
marching once `d < radius(t)`, not once `d` is small in absolute terms.
For a LOW-coverage hit, `d` can be nearly equal to a large `radius(t)`,
meaning `p_world = origin + t * direction` can be sitting a full
`radius(t)` away from the true surface, genuinely floating in open room
air rather than on any real surface. Proven directly with a real leaking
sample from `gi_room.rs`'s own geometry: a ceiling-launched cone hit the
floor with `coverage ≈ 7.9e-7` at `p_world.y = -1.97`, while the floor's
own true surface is at `y = -3.0` — **1.03 units** of real separation,
not a rounding error. Shading from that floating point can give a
shadow ray toward the sun a genuinely clear, physically-consistent-for-
that-point path that a shadow ray cast from the true surface point never
would — letting real sunlight "through" a provably sealed room, entirely
inside `shade()`'s own otherwise-correct point-ray shadow logic (verified
separately: `shade()` itself reports zero light at the true floor
surface point with the real room + sun geometry — the bug is purely in
what point cone tracing hands to `shade()`, not in `shade()`'s own shadow
test).

A first fix attempt — nudging `p_world` further along `hit.world_normal`
by `cone_radius_at(r0, half_angle, hit.t)` before shading, reasoning this
would move the shadow-ray origin clear of ambiguity — made the leak
*worse* (max leak 0.027 → 0.045): the point was already floating *above*
the true surface, so pushing further along the (upward-facing, for this
floor hit) normal moved it even further from solid ground. Reverted.

**Real fix**: scale the final shaded contribution by `coverage^2` (not
linear `coverage`) in `cone_trace_ref::cone_trace_ray`, mirrored exactly
in `hybrid_trace.wgsl::cone_trace_ray`. This is an asymptotic reduction
toward darkness, not an exact-zero guarantee — deliberately conservative
(fades toward black, the physically safe direction, rather than trying to
relocate an inherently uncertain point). Measured against the leaking
sample above: `0.0014307235` max leak with linear `coverage` → `4.2e-4`
with `coverage^2`. Calibrated against this scene's own light scale
(`EXPOSURE=0.0005 * sun intensity=20000` gives a genuinely front-lit
white surface a `direct_and_emissive` magnitude around 2-3 in these same
units) — `4.2e-4` is roughly 0.02% of that, well below any plausible
post-tone-mapping display-visible threshold.

**New regression test**:
`conetrace_ref::room_leak_regression::
real_room_with_sun_and_no_lamp_never_leaks_light_through_a_closed_roof`
rebuilds `gi_room.rs`'s own exact room shell (all 6 wall/roof/floor
panels, real `WALL_OVERLAP` seams) and all 7 real cubes, fires the real
sun (no lamp — isolates this from the lamp, which the user's own testing
separately ruled out as a contributing factor entirely: "This didn't
solve an issue" after a lamp-disabled test build), scans a grid of
ceiling points via `cone_trace_indirect`, and asserts `max_leak < 1e-3` —
a real, calibrated, non-zero threshold (see the test's own doc comment
for the full reasoning), not an arbitrarily tight one picked to just
barely pass.

**A separate, related bug found and fixed alongside this one**:
`prepare_hybrid_temporal` (`src/hybrid/pipeline.rs`) never invalidated
the ping-pong `HybridHistory` buffer on a live `GiMethod` change, only on
a resolution change — meaning switching GI techniques via the UI radio
buttons could keep blending a PREVIOUS technique's own indirect-light
history into the newly selected technique's fresh values for up to
`temporal_max_history_length` frames. Different techniques' indirect
values aren't comparable to blend across, the same way a resize's stale-
resolution content isn't. Fixed by adding a `gi_method: u32` field to
`HybridHistory` and extending the reset check to `h.size != want ||
h.gi_method != scene_data.gi_method`.

**Verified**: `cargo build/test/clippy --release --lib --examples`
clean, 145/145 lib tests (down from the prior entry's 143 — two purely
diagnostic, assertion-free tests written during this investigation were
removed; one new named regression test was added net).
`tests/wgsl_parse.rs::hybrid_trace_wgsl_parses` passes (the two other
failures in that same test binary, `hybrid_blit_wgsl_parses` and
`splat_wgsl_parses`, are pre-existing on `master` from unrelated
in-progress work and unaffected by this fix — confirmed via `git stash`
against the same baseline). Real-GPU-verified via `gallery.rs --gi-method
conetrace --shot` at `--stress 100`/`--stress 10000`: no panics, no wgpu
validation errors, no NaN/black-hole corruption.

**Known limitation, stated honestly**: the `coverage^2` fix reduces the
leak to an imperceptible-by-calibration level, it does not make it
exactly zero for every possible low-coverage hit. A more invasive fix
(iteratively refining a low-coverage hit's own position via bisection
before shading) would close this exactly but was judged not worth the
extra marching cost for a residual already ~2500x below this scene's own
lit-surface magnitude. If a future scene's own light scale makes this
residual newly visible, that's the next place to look.

### Hash-grid radiance cache: second selectable GI technique, WGSL + real-GPU verified

Direct follow-up to the `GiMethod` selector entry below — this lands the
hash-grid radiance cache itself (`GiMethod::HashGrid`), the first real
alternative to DDGI. Full CPU-ref-first path: `src/hybrid/hashgrid_ref.rs`
(committed separately, `bf5c770`) landed first with 12 tests, then this
entry's WGSL mirror + pipeline wiring + real-GPU verification.

**Design recap** (see `hashgrid_ref.rs`'s own module doc comment for the
full rationale): a spatial hash table keyed by `(world cell, normal
octant)`, open-addressed with linear probing, replacing DDGI's fixed
dense probe grid with effectively adaptive resolution — cells only exist
where a shaded point has actually missed and inserted one. Reuses DDGI's
own `HEMISPHERE_SAMPLES`/`ddgi_tangent_basis` for the relight ray bundle
and, critically, `sample_probe_grid`'s own trilinear-blend +
smoothstep-gated-fallback SHAPE for shading-time reads — ported
specifically to avoid reintroducing the exact hard cell/probe-boundary
flicker bug DDGI had to fix twice this session (see the entry below).

**Key packing, a real correctness risk resolved with a test first**:
`hashgrid_key` packs a `(cell, octant)` triple into one `u64`, split as
two `u32`s (`key_lo`/`key_hi`) for the GPU side (WGSL has no native
64-bit integer). The FIRST packing attempt (`CELL_COORD_BITS=21`, evenly
split across 3 axes) put `cy` straddling the `key_lo`/`key_hi` 32-bit
boundary — decodable, but only via genuinely error-prone cross-word
bit-twiddling on the WGSL side (confirmed by writing that decode once,
getting it visibly wrong, and deciding to fix the packing instead of
the decode). Repacked to `CELL_COORD_BITS=14`, `cx` getting a full `u32`
to itself and `cy`/`cz`/`octant` all fitting within the OTHER `u32` —
every field is now a plain shift-and-mask read from exactly one word,
no cross-word reconstruction anywhere. Proven via a new CPU-ref
`decode_hashgrid_key` (the exact inverse) and
`hashgrid_key_round_trips_through_decode_for_a_wide_range_of_cells`
(covers both signs on every axis, the `CELL_COORD_BITS` range boundary,
`cx` values far outside 14 bits, and every octant) — the WGSL
`decode_key` is then a direct, low-risk port of an already-proven
formula rather than novel bit-twiddling written straight into a shader.
`hashgrid_ref.rs` test count: 12 (up from 11), 132 total lib tests.

**WGSL — new territory: this is the first storage buffer in this
codebase using a real atomic.** DDGI's own atlas has no cross-invocation
write race (each probe only ever touches its own texels, written by
exactly one relight invocation). A hash-grid slot is different: a
shading-time MISS in `hybrid_trace.wgsl`'s new `hashgrid_sample` needs
to claim an empty table slot so a future relight pass can find it, and
multiple `trace_main` invocations in the same frame can race to claim
the same slot. Fixed with `atomicCompareExchangeWeak` on the one field
that needs it (`occupied: atomic<u32>`) — a losing invocation's
exchange just fails and it skips inserting, an accepted, harmless
outcome (one fewer cell pre-seeded this frame, no data corruption; see
`hashgrid_sample`'s own doc comment for the full synchronization
argument, including why `key_lo`/`key_hi` themselves don't need to be
atomic).

**New files**: `assets/shaders/hybrid_hashgrid_update.wgsl` (self-
contained relight pass, following `hybrid_ddgi_relight.wgsl`'s own
duplication convention — its own copy of `trace`/`march_object`/
`shade_direct_only`, plus the ported hemisphere-sample/tangent-basis
helpers and the key-decode inverse). `hybrid_trace.wgsl` gained
`hashgrid_sample` (called from `shade()`'s `GI_METHOD_HASHGRID` branch,
the same single substitution point DDGI's own read already uses) plus
its own group-3 `hashgrid_table` binding — a SEPARATE bind group from
DDGI's own group 2, specifically to avoid touching/renaming DDGI's
already-proven bind group (additive, not a rename, matching this
codebase's own established bias).

**Pipeline wiring** (`src/hybrid/pipeline.rs`/`pass.rs`/`mod.rs`):
`HashGridEntryGpu`, `hybrid_hashgrid_layout`/`hybrid_trace_hashgrid_read_layout`,
`HybridHashGrid`/`HybridHashGridRes` (allocated ONCE at a fixed
`log2_capacity`, never resized — simpler lifecycle than DDGI's own
scene-shape-driven atlas resize, since hash-grid capacity is a static
config knob), `prepare_hybrid_hashgrid`, and a `pass.rs` dispatch gate
mirroring DDGI's own (`GiMethod::HashGrid` → real dispatch skip when not
selected, not just a shader branch). `HashGridConfig` (new
`extract.rs` resource: `cell_size`/`log2_capacity`/`entries_per_frame`/
`max_history_length`/`max_t`) mirrors `DdgiConfig`'s own shape and
per-scene override precedent — `gi_room.rs` sets `cell_size=1.2`/
`max_t=28.0`, mirroring `DdgiConfig`'s own room-scale tuning exactly
(same seam-leak-capping reasoning, since both techniques share the same
underlying `trace()`).

**UI/CLI**: `gi_room.rs`'s and `gallery.rs`'s `controls_panel` both gained
hash-grid tunable sliders (entries relit/frame, cell size, relight ray
reach), gated on `GiMethod::HashGrid` the same way DDGI's own sliders
gate on `GiMethod::Ddgi`; both HUDs show a `hybrid_hashgrid` GPU pass
timing line alongside `hybrid_ddgi`'s. `gallery.rs` gained
`--hashgrid-cell-size`/`--hashgrid-capacity-log2`/
`--hashgrid-entries-per-frame`/`--hashgrid-history`/`--hashgrid-max-t`,
mirroring `--ddgi-*`'s exact pattern.

**Verified**: `cargo build/test/clippy --release --lib --examples`
clean, 132/132 lib tests. New `hybrid_hashgrid_update_wgsl_parses` test
(`tests/wgsl_parse.rs`) — parses AND validates (naga's full
`Validator::validate`, not just a bare parse) with the `atomic<u32>`
field genuinely present in the struct, confirming the atomics usage
itself is syntactically valid, not just glossed over. Real-GPU-verified
via `gallery.rs --gi-method hashgrid --shot`: no panics/validation
errors at `--stress 100` or `--stress 10000`, correct shading (proper
shadows, no black holes, no NaN corruption) at both scales including
through the atomics-based miss-insert path. Real measured costs at
`--stress 10000`: `hybrid_hashgrid` update ~0.37-0.47ms (cheaper than
DDGI's own ~1.3-1.4ms early in a run, since the cache starts empty and
only lazily fills — expected to converge toward a comparable steady-
state cost as the table populates, not yet measured over a long-running
session), `hybrid_trace` ~135-208ms (comparable to, with more variance
than, DDGI's own ~99-151ms baseline — plausible given each shading-time
miss now costs an extra atomic insert attempt on top of the read).
`gi_room.rs` starts cleanly with its own room-scale `HashGridConfig`
override, reaches pipeline READY, no panics over a 15s run — the visual
side-by-side comparison itself needs the user's own manual interactive
check per this scene's established screenshot-free convention.

Not yet committed (this WGSL/pipeline layer — `hashgrid_ref.rs` itself
already committed as `bf5c770`).

### GI method selector: `GiMethod`/`GiMethodConfig` — DDGI is now one of several selectable indirect-diffuse techniques, not the renderer's only one

Even after all the DDGI bugs fixed in the entries below (self-occlusion,
atlas ping-pong stroboscoping, edge-pinned probe layers, `max_t` scale
mismatch, missing hemisphere convolution, occluded-probes fallback
flicker), DDGI's own *architecture* still has a real quality ceiling —
a fixed-resolution dense probe grid is inherently blocky for sharp
indirect gradients, and one occlusion ray per probe (no Chebyshev
visibility test) caps quality regardless of sample count. Rather than
continuing to patch DDGI, the plan going forward is to prototype
genuinely different GI techniques (a world-space hash-grid radiance
cache, later SDF cone tracing) and compare them side by side against
DDGI in `examples/gi_room.rs`. This entry is the prerequisite plumbing
for that: a single, mutually-exclusive `GiMethod` selector all future
techniques share.

**Why mutually exclusive, not independently-toggleable**: `shade()`
(`hybrid_trace.wgsl`) writes into one `indirect: vec3<f32>` slot
consumed once by the denoise pass. More than one technique contributing
at once would sum indirect light non-physically and defeats the actual
point of having multiple techniques — comparing alternatives, not
combining them.

**What changed**: `DdgiConfig::enabled` is deleted (a real, if small,
breaking change to already-working DDGI code — two sources of truth for
"is DDGI active" was judged worse). A new `GiMethod` enum
(`None`/`Ddgi`/`HashGrid`/`ConeTrace`, `repr(u32)`) plus
`GiMethodConfig` resource replace it as the renderer's single selector.
`ConeTrace` is a reserved placeholder variant with no backing logic
anywhere yet — declared now so the enum doesn't need another breaking
change when that technique lands later; selecting it is currently
equivalent to `None`. Same for `HashGrid` in this entry — the selector
exists and is fully wired, but no hash-grid sampling logic exists yet
(a direct follow-up, see this file's own future entries).

Threading chain (mirrors the old `ddgi_enabled` chain exactly, just
generalized from a bool to a small enum): `GiMethodConfig` (main-world
resource) → `extract_hybrid_scene` copies the discriminant into
`RenderHybridScene.gi_method: u32` → `prepare_hybrid_scene` writes it
into `SceneUniform.gi_method: u32` → `hybrid_trace.wgsl`'s `shade()`
reads `scene.gi_method` via new `GI_METHOD_NONE`/`_DDGI`/`_HASHGRID`/
`_CONETRACE` WGSL constants (mirroring the enum's own discriminants
exactly) → `pass.rs`'s DDGI relight dispatch gate (`if scene_data.
gi_method == GiMethod::Ddgi as u32`) still skips the whole compute
dispatch, not just a shader branch, when DDGI isn't the active
technique — same "costs nothing when not selected" property the old
bool-gated version had.

`examples/gi_room.rs`'s and `examples/gallery.rs`'s `controls_panel`
checkboxes both became a 3-way `ui.radio_value` row (`None`/`DDGI`/
`Hash-grid`); DDGI's own tunable sliders now gate on `gi_method.method
== GiMethod::Ddgi` instead of the deleted `ddgi_config.enabled`.
`gallery.rs` gained `--gi-method none|ddgi|hashgrid` (`--ddgi on|off`
kept working as a convenience alias: `on` → `GiMethod::Ddgi`, `off` →
`GiMethod::None`, whichever flag appears later on the command line
wins, matching every other `_from_args` function's existing
conflict-resolution convention in that file). `gi_room.rs` has no CLI
flags at all (interactive-only, unchanged) — only the new radio buttons.

**Verified**: `cargo build/test/clippy --release --lib --examples`
clean, full 120-test suite passing (including the flicker-fix regression
test from the entry below, confirming the refactor didn't disturb it).
New `hybrid_ddgi_relight_wgsl_parses` test added (`tests/wgsl_parse.rs`)
since that file's `SceneUniform` mirror changed and had no parse-test
coverage before this — passes, alongside the pre-existing
`hybrid_trace_wgsl_parses`. Real-GPU-verified via `gallery.rs --shot`:
`--gi-method none/ddgi/hashgrid` all run with no panics/validation
errors at `--stress 100`; `none` and `hashgrid` both correctly show the
DDGI dispatch skipped (no `gpu ddgi` HUD line) and render identically
(hash-grid has no backing logic yet, degrades to `none` exactly as
designed); `ddgi` at `--stress 10000` shows `gpu ddgi ~1.3-1.4ms`,
`trace ~100-105ms` — matching the pre-refactor baseline below, no
regression. `gi_room.rs` starts cleanly, reaches pipeline READY, no
panics over a 15s run; the radio-button switching itself needs the
user's own manual interactive check per this scene's established
screenshot-free convention.

Not yet committed.

### DDGI: cosine-weighted hemisphere convolution at read time — probes were storing raw single-ray samples, not irradiance

Testing DDGI against `examples/gi_room.rs` (a fully closed room, sun
only reaching in through a slowly-opening roof gap) surfaced a further
gap beyond the four bugs already fixed in the entries below: surfaces
facing away from any direct light stayed pitch black even once the sun
was streaming in, the overall indirect result read too dim, and the
indirect term visibly flickered even with a stationary camera.

**Root cause, not a bug**: `ddgi_probe_irradiance`
(`hybrid_trace.wgsl`) reads exactly ONE atlas texel per probe — the
texel whose octahedral direction matches the shaded surface's own
normal. That texel holds a RAW single-ray sample from
`probe_ray`/`relight_probe_texel`, EMA-blended over time but never
blended across NEIGHBORING directions. A probe sitting in a genuinely
well-lit area still reads dark for a shaded point's own normal if that
one specific ray direction happened to miss anything — real DDGI/RTXGI
probes store cosine-CONVOLVED irradiance specifically so any read
direction gives a plausible diffuse-hemisphere answer; this
implementation never had that convolution step. This explains all
three symptoms at once (away-facing = wrong single-direction miss,
dimness = no ambient-fill averaging, flicker = single-sample variance
compounding with the already-known occluded-probes-fallback
discontinuity — see "Known, not yet fixed" below).

**Fix: gather-at-read, not scatter-at-write.** Considered and rejected
scattering each traced sample into a cosine-weighted neighborhood of
atlas texels at RELIGHT time (the standard production-DDGI approach) —
each atlas texel is written by exactly one GPU invocation in
`hybrid_ddgi_relight.wgsl` (`gid.x`/`gid.y` maps 1:1 to a texel), so
scattering into neighboring texels from other invocations would be a
real cross-invocation write race, breaking the "each invocation only
touches its own texel" invariant the atlas ping-pong-removal fix (this
same log, below) specifically established as race-free. Also
considered a post-relight blur pass over each probe's own octahedral
tile — rejected because octahedral maps have real seam/wraparound
behavior at tile edges (already documented from this session's own
`octahedral_encode` degenerate-corner findings), and a naive blur
across that seam would incorrectly blend unrelated directions.

Implemented instead: gather 5 cosine-weighted samples around the
shaded normal at READ time, reusing `ddgi_probe_irradiance`'s own
already-bounds-safe single-texel read as a sub-routine, called 5 times
instead of once. The 5 sample directions and the tangent-basis
construction are NOT new — they're the exact literal values and Duff
et al. formula from Stage B's own removed `INDIRECT_SAMPLES`/
`tangent_basis` (recovered from git history, commit `19a4cbb`, the
commit that removed them), re-added as DDGI's own copy
(`ddgi_tangent_basis`/`HEMISPHERE_SAMPLES` in `src/hybrid/ddgi_ref.rs`
and `assets/shaders/hybrid_trace.wgsl`) rather than re-derived from
scratch — a known-good, previously-shipped sample set. New
`cosine_weighted_probe_irradiance` (Rust) /
`ddgi_cosine_weighted_probe_irradiance` (WGSL) sits between
`sample_probe_grid`'s per-probe trilinear loop and the atlas read,
touching only the shading-time read path — no changes to
`hybrid_ddgi_relight.wgsl` at all. Plain-averaged, not additionally
dot-weighted: `HEMISPHERE_SAMPLES`' own directions already lean
cosine-weighted by construction (more samples cluster near the pole
than the rim), so an extra weighting term would double-count that bias
rather than add real energy conservation.

3 new CPU-ref tests (`src/hybrid/ddgi_ref.rs`): `ddgi_tangent_basis`
orthonormality across axis-aligned and arbitrary normals; a uniform-
field sanity check (convolving an identical value in every direction
must return that same value, confirming no spurious scaling); and the
direct regression test for the "away-facing surfaces are black" bug —
a synthetic probe lookup bright in exactly one direction and dark
everywhere else (including the shaded normal itself) now returns a
real non-zero convolved result. `ddgi_ref.rs`'s own test count: 32,
up from 29 (119 total lib tests, unaffected by other modules'
counts — verified via `cargo test --release --lib` directly rather
than assumed).

**Verified**: `cargo build/test/clippy --release --lib --examples`
clean. Runtime: WGSL pipeline compiles and dispatches correctly (no
validation errors) on real hardware (AMD Radeon RADV RENOIR, Vulkan)
at both a small single-sphere scene and `--stress 10000`, and on
`gi_room.rs`'s own scene (pipeline reaches "READY" with no errors).
Screenshot-verified correct rendering at `--stress 10000` scale, no
regression.

**Real GPU cost, warmup-corrected, at `--stress 10000`**: `hybrid_trace`
rose from ~64-71ms (pre-convolution baseline, logged in this file's
earlier DDGI entries) to ~80-84ms — a real ~15-20ms (roughly 20-25%)
increase, exactly the cost this plan's own design-decision section
flagged honestly in advance (5x the atlas reads per shaded pixel's
DDGI term, since gather-at-read amortizes NO cost across frames, unlike
a scatter-at-write approach which would spread convolution cost into
the already-incremental relight schedule). `hybrid_ddgi` (the relight
pass itself) unaffected, as expected — this change touches only the
read path.

**Known, not yet fixed**: `sample_probe_grid`'s own all-8-probes-
occluded fallback (falls back to the unweighted average of all 8 raw
irradiance values, including probes on the wrong side of a wall, when
every one of the 8 surrounding probes happens to be occluded at once)
is a real, separate formula discontinuity — as a shaded point's own
occlusion state flips frame-to-frame near a corner (plausible given
several of the 8 surrounding probes can be right at a wall boundary),
the result can visibly pop between two different formulas. This
convolution work fixes the single-sample-noise contributor to the
reported flicker but does NOT address this second, distinct
contributor — left as an explicitly separate follow-up, not silently
folded into this entry or silently dropped. **Fixed in the entry
directly below.**

Committed as `fc2f327`.

### DDGI: smooth the occluded-probes fallback ramp — fixes the second, distinct flicker contributor

Direct follow-up to the hemisphere-convolution entry above, which
fixed the single-sample-noise contributor to `gi_room.rs`'s reported
indirect-light flicker but explicitly flagged a second, separate
discontinuity as still open. This entry fixes that second contributor.

**Root cause**: `sample_probe_grid`'s (`src/hybrid/ddgi_ref.rs`) and
`ddgi_sample_probe_grid`'s (`assets/shaders/hybrid_trace.wgsl`) final
return was a hard branch on `weight_sum > 1e-6` — a shaded point's 8
surrounding probes are each visibility-gated by one occlusion ray, and
`weight_sum` is the sum of trilinear weights from only the unoccluded
ones. On one side of the branch: `acc / weight_sum`, the proper
trilinear-weighted average of unoccluded probes. On the other
(`weight_sum <= 1e-6`, i.e. every one of the 8 probes occluded at
once): `raw_sum / raw_count`, the unweighted average of ALL 8 raw
probe values, including ones the visibility gate had just excluded.
Near a wall or corner — where several of a shaded point's 8
surrounding probes sit right at the occlusion boundary — the set of
occluded probes can flip frame to frame at grazing angles, and
`weight_sum` crossing exactly zero flipped the ENTIRE result between
two structurally different formulas: a hard visual pop even though the
underlying geometry only changed by one probe's occlusion state.

**Fix**: replaced the hard branch with a `smoothstep`-based blend
between the two formulas, gated by a new `FALLBACK_RAMP_WEIGHT = 0.2`
constant (`DDGI_FALLBACK_RAMP_WEIGHT` in WGSL): `mix(fallback,
weighted, smoothstep(0, FALLBACK_RAMP_WEIGHT, weight_sum))`. Both
endpoints are unchanged by construction — at `weight_sum == 0` this
reduces to exactly the old fallback, and once `weight_sum` clears
`FALLBACK_RAMP_WEIGHT` it reduces to exactly the old weighted-average
path — only the transition between them is now continuous.
`FALLBACK_RAMP_WEIGHT = 0.2` is deliberately small: trilinear weights
sum to at most `1.0` (a shaded point exactly on a probe), so `0.2`
only smooths the specific "few probes barely unoccluded" regime this
bug lives in, confirmed by the existing
`an_occluded_probe_contributes_zero_not_a_reduced_weight` test's own
real geometry (4 of 8 probes visible there gives `weight_sum ≈ 0.66`,
well clear of the ramp — that test's own "occluded probe's color must
not leak at all" guarantee is unaffected and still passes unmodified).
A safe division floor (`weight_sum.max(1e-6)`) guards the
weighted-average term against a huge-or-NaN intermediate value even
when its final blend contribution is near zero (since `0 * inf = NaN`
in IEEE 754).

CPU-reference-first, as always in this project: implemented and
tested in `src/hybrid/ddgi_ref.rs` first, only then mirrored verbatim
into `assets/shaders/hybrid_trace.wgsl`'s `ddgi_sample_probe_grid`.
1 new CPU-ref test,
`the_occlusion_fallback_has_no_hard_jump_as_weight_sum_crosses_the_old_threshold`:
constructs a grid where 7 of 8 corners sit deep below a ground plate
(always occluded from above) and the 8th sits high above it (always
unoccluded), so that single corner's own trilinear weight IS
`weight_sum` in its entirety; sweeps the shaded point's Y position to
carry that weight continuously through both the old hard threshold (0)
and the new ramp's own ceiling, and asserts no single step's own delta
is far larger than the sweep's own average step delta (the direct
signature a hard jump would leave), plus confirms both sweep endpoints
still match the old hard branch's own two exact formulas. `ddgi_ref.rs`
test count: 33, up from 32 (120 total lib tests).

**Verified**: `cargo build/test/clippy --release --lib --examples`
clean (120/120 lib tests pass, no new clippy warnings). Runtime: WGSL
pipeline compiles and dispatches with no validation errors on real
hardware (AMD Radeon RADV RENOIR, Vulkan) at both a small single-sphere
scene and `--stress 10000`; screenshot-verified correct shading (proper
soft shadow, proper ambient/GI fill, no black holes or NaN artifacts)
at both scales. `gi_room.rs` itself is interactive-only per its own
design (no `--shot`/CLI screenshot tooling) — not screenshot-verified
here, left for manual visual confirmation.

Not yet committed.

### DDGI: removed the probe atlas's ping-pong — a real stroboscope bug

Two DDGI visual bugs were reported and fixed after the initial DDGI
ship and Stage B removal (both logged below): a self-occlusion flicker
(already fixed, see the self-occlusion entry) and — found only after
deliberately testing DENSER probe spacing (`--ddgi-spacing 3` at
`--stress 100`) — a hard, unmistakable stroboscope flicker across the
WHOLE scene, not a subtle artifact. `--ddgi-spacing 3` was initially
assumed to be a smoothing knob (it does fix the earlier accepted-risk
directional-asymmetry issue); it actually exposed a second, more severe
bug that the default spacing had been silently hiding.

**Root cause**: the probe atlas + per-probe history-length buffers were
ping-ponged every frame, mirroring `HybridHistory`'s own pattern for
per-pixel temporal accumulation — but that pattern doesn't apply here.
`HybridHistory`'s ping-pong exists because reprojection reads a
DIFFERENT screen location than it writes (a genuine cross-invocation
race under WGPU's unordered execution). DDGI's relight pass has no such
race: each relit probe only ever reads and writes its OWN texels,
never another probe's. Ping-ponging anyway meant every probe NOT relit
in a given frame (any probe outside `probes_per_frame`'s rotating
window) had its "current" atlas slot silently swapped to whatever was
in that physical resource TWO parities ago, not its own last real
value. At the default `probe_spacing=11.0`, `--stress 100`'s ~363 total
probes all relight every single frame (`probes_per_frame=512` >
`total_probes`), so no probe was ever "not relit this frame" and the
bug was invisible. At `--ddgi-spacing 3`, the same scene produces
~4332 probes — an ~9-frame cycle at 512/frame — and every probe outside
that frame's window read a stale, effectively-random snapshot,
producing a genuine two-value oscillation every frame: a real
stroboscope, not noise or slow convergence lag.

**Fix**: dropped the ping-pong entirely. The atlas is now a SINGLE
`read_write` storage buffer (`array<vec4<f32>>`, manually row-major
indexed — `read_write` isn't available on storage TEXTURES in this
codebase, confirmed by `hybrid_denoise.wgsl`'s own prior doc-comment
history, hence the atlas moved from a `texture_storage_2d` to a
storage buffer) and the per-probe history-length buffer is likewise a
single `read_write` `array<f32>`. A probe not relit this frame is
simply never touched — its data persists correctly with no swap to
lose track of it. `HybridDdgiParity`/`HybridDdgiSlot`
(`src/hybrid/pipeline.rs`) removed entirely; `hybrid_ddgi_layout`'s and
`hybrid_trace_ddgi_read_layout`'s bind-group layouts updated to match;
both `assets/shaders/hybrid_ddgi_relight.wgsl` and
`assets/shaders/hybrid_trace.wgsl` updated to index the atlas buffer
manually instead of `textureLoad`/`textureStore`.

Verified: `cargo build/test/clippy --release --lib --examples` clean
(116/116 tests). Runtime: `--ddgi-spacing 3` at `--stress 100`,
consecutive frame captures (`--shot`/`--at-frame` at frames 100-105)
are now pixel-identical — the stroboscope is gone. Regression-checked
at `--stress 500` and `--stress 10000` (both previously-measured GPU
cost profiles unchanged: ddgi ~0.9-1.2ms, trace ~38-71ms depending on
scale, matching prior entries) — no visual or performance regression at
default spacing.

Not yet committed.

### DDGI: fixed occlusion self-intersection on rotated objects

`sample_probe_grid`'s visibility-gate occlusion ray had no self-
exclusion — every other surface-launched ray in this renderer (shadow
rays, the old Stage B indirect rays) excludes the shaded object's own
entity from its own occlusion query, but DDGI's gate was the one
exception. A rotating object's own occlusion ray toward a probe could
graze back across its own geometry at specific rotation angles, self-
intersecting and zeroing that probe's contribution for exactly the
frames the rotation put the ray in the self-clipping regime — found by
direct visual inspection: one face of a spinning `--stress N` cube
intermittently flickering dark, then recovering.

Fixed in both `src/hybrid/ddgi_ref.rs::sample_probe_grid` and
`assets/shaders/hybrid_trace.wgsl::ddgi_sample_probe_grid`, mirroring
`trace_shadow`'s existing `origin_entity` exclusion pattern exactly.
New regression test
(`a_rotated_boxs_own_occlusion_ray_does_not_self_intersect_its_own_shaded_face`)
reproduces the exact self-intersection geometry: a box rotated 45
degrees around Y, shaded at a point on its own +X face, probed toward
a probe on the OPPOSITE side of the box — the straight line between
them genuinely crosses the box's own rotated volume without the fix,
and is correctly treated as fully visible with it.

Also added `--rotate x,y,z` (`examples/gallery.rs`, mirrors `--spin`'s
CLI pattern) — sets stress-grid objects' fixed initial rotation, a
debugging aid for reproducing rotation-angle-dependent visual bugs at
a known static angle instead of hunting through a live spin's frames.
Using it surfaced the SEPARATE ping-pong bug above (logged as its own
entry, landed after this one) — this self-occlusion fix alone was
insufficient to explain a second, distinct symptom (persistent per-face
darkness at some fixed angles, worsening rather than improving at
denser `--ddgi-spacing`), which is what led to finding the ping-pong
bug.

Verified: `cargo build/test/clippy --release --lib --examples` clean
(116/116 tests, one new regression test).

Not yet committed.

### Full DDGI: persistent world-space probe grid replaces Stage B's per-pixel indirect diffuse

The "maybe D" stage from this renderer's original GI roadmap ("A:
shadows -> B: cheap fixed-probe indirect -> C: occupancy-grid
acceleration -> maybe D: full DDGI") is now real: a persistent
world-space grid of light probes, relit incrementally across frames,
sampled and trilinearly interpolated at shading time — genuine
multi-bounce, range-independent indirect lighting, replacing Stage B's
structural ceiling of "only ever bounces off whatever a short
(`INDIRECT_MAX_T=3.0`) per-pixel probe ray happens to hit." Three
explicit tradeoffs were made deliberately, not incidentally:

1. **Octahedral atlas storage** (RTXGI's actual technique — one small
   square tile per probe packed into a shared 2D atlas texture), not
   this project's cheaper existing 5-fixed-direction shortcut. More
   infrastructure, chosen for genuine DDGI quality.
2. **No Chebyshev visibility test.** Irradiance-only probes; light
   leaking through thin occluders is a real, accepted risk for this
   first pass, mitigated cheaply (a single occlusion ray per probe at
   shading time, zeroing — not down-weighting — an occluded probe's
   contribution). Acceptable given this renderer's test scenes (convex
   primitives on open flat ground, not enclosed rooms).
3. **Full replacement of Stage B**, not an additive second bounce.
   `indirect_diffuse`/`IndirectDiffuseConfig` remain in the codebase
   only as a `--indirect on` fallback for side-by-side comparison —
   `DdgiConfig::enabled` (default `true`) is the renderer's real
   default indirect term going forward. Actual deletion of Stage B's
   code is deferred to a follow-up step once DDGI has had more
   real-world tuning; not yet done.

**New CPU-reference module** `src/hybrid/ddgi_ref.rs` (mirrors
`temporal_ref.rs`'s precedent for splitting large CPU-ref concerns out
of `cpu_ref.rs`): `ProbeGrid`/`probe_grid_from_bounds` (grid derived
from `PersistentBvh`'s root AABB, not hand-authored — spacing 11.0
world units matching the existing `--stress N` object cell pitch,
`vertical_layers=3` so probes actually vary with height, not just a
flat horizontal layer), `octahedral_encode`/`octahedral_decode` (the
decode direction had no prior implementation anywhere in this codebase
— written fresh, round-trip tested, with a documented exception for the
four unit-square corners, a genuine many-to-one fold in the mapping
itself, not a bug), `direction_to_texel`/`texel_to_direction`,
`AtlasLayout::exact_fit` (`tiles_per_row = ceil(sqrt(probe_count))` —
computed exactly, no hardcoded max-width guess; a first draft of the
WGSL relight shader briefly hardcoded a `4096u` atlas-width guess,
caught and reverted before it shipped), `ddgi_probe_relight_start`
(generalizes the already-shipped `indirect_sample_start` rotating-
subset formula from pixel to probe granularity), `probe_ray`/
`relight_probe_texel` (reuses `trace`/`shade`/`temporal_blend`
verbatim, capped at one order of indirection — a probe ray never
recursively bounces into other probes), `probe_grid_cell`/
`sample_probe_grid` (trilinear blend over the 8 surrounding probes,
each visibility-gated by one occlusion ray, zeroed not down-weighted
when occluded; falls back to the unweighted average of all 8 raw
samples — not black — if every one is occluded). 28 tests, including a
real `--stress 10000`-scale profile (30,603 probes, `dims (101, 3,
101)`) confirming atlas tiles never overlap and grid coverage stays
sane at that scale.

**New WGSL relight pass** `assets/shaders/hybrid_ddgi_relight.wgsl`
(new compute dispatch, runs FIRST in `hybrid_pass`, before trace — a
faithful, deliberately-duplicated port of trace/march/shade rather than
a shared import, matching this project's established per-pass
self-containment convention). Ping-ponged atlas (`HybridDdgiAtlas`,
two `HybridDdgiSlot`s) for the same reason `HybridHistory` is
ping-ponged: relighting reads last frame's converged irradiance for the
EMA blend while writing this frame's result, and in-place writes would
race under WGPU's unordered-invocation execution model. Dispatch domain
is `[probes_per_frame * tile_size, tile_size]`, cycling through the
whole grid over several frames rather than relighting every probe every
frame (unaffordable at this renderer's per-ray BVH+SDF-march cost).
Skipped entirely (not dispatched at all, not just a shader-side no-op)
when `DdgiConfig::enabled` is off — a probe relight is a real batch of
`trace()` calls, not a cheap per-pixel copy pass, so there's a genuine
dispatch to save.

**Shading-time integration**: `hybrid_trace.wgsl`'s `shade()` now reads
the just-relit atlas via a new group-2 bind group
(`hybrid_trace_ddgi_read_layout`, built in a new `prepare_hybrid_ddgi`
render-world system rather than `prepare_hybrid_scene` — needed because
it must reference the ping-pong slot `prepare_hybrid_ddgi` itself just
resolved, a different system than the one that builds trace's other
bind groups) and trilinearly samples/visibility-gates it via a WGSL
port of `sample_probe_grid`, replacing `indirect_diffuse`'s call site
when `ddgi_enabled != 0u` (both flags are independent — `--ddgi off
--indirect on` still exercises the old Stage B path for comparison).

**CLI/egui wiring**: `--ddgi on|off` / `--ddgi-probes-per-frame N` /
`--ddgi-tile-size N` / `--ddgi-history N` / `--ddgi-max-t F` /
`--ddgi-spacing F` / `--ddgi-layers N`, mirroring
`indirect_config_from_args`'s exact pattern; egui "DDGI (probe-grid GI)"
panel section, with the old "Indirect diffuse" section now marked
legacy/comparison-only and disabled while DDGI is on. HUD gained a
`gpu ddgi X ms` timing readout (via a new `hybrid_ddgi`
`RecordDiagnostics` span) shown only on frames DDGI actually dispatched
— falls back to the existing trace/temporal/denoise/blit-only line
(not a blanket "gpu n/a") when DDGI is off, since only the DDGI span is
conditionally absent while the other four always dispatch.

**Verified working end-to-end on real hardware** (AMD Radeon RADV
RENOIR, Vulkan): `cargo build/test/clippy --release --lib --examples`
clean throughout (134/134 tests passing, no new clippy warnings).
Visual smoke tests via `--shot`/`--at-frame`: single-sphere scene
renders correctly with DDGI on (soft shadow, plausible warm ground
bounce near the sphere), `--ddgi off --indirect on` A/B comparison
against Stage B on the identical scene/camera shows comparable
lighting quality (DDGI's bounce reads slightly softer/dimmer here —
expected, since this tiny scene's whole extent is smaller than one
11-unit probe cell, so the grid barely resolves it spatially; DDGI's
real advantage is longer-range scenes Stage B structurally can't
reach, not a single small object). `--stress 500` grid scene (many
cubes on a shared ground plane) renders correctly with DDGI on — no
crashes, no black holes, no NaN blowouts, full 500-cube grid lit
plausibly.

**Real GPU costs, warmup-corrected**: small single-sphere scene, DDGI
on, grid fully covered by `probes_per_frame=512` in a couple frames —
`hybrid_ddgi` relight pass ~0.03-0.04ms (near-free at this tiny grid),
`hybrid_trace` ~8-9ms (up from Stage B's own ~6-8ms baseline on the
identical scene — the added cost is the 8-occlusion-ray visibility
gate + 8 atlas texture reads per shaded pixel). At `--stress 500`
(camera pulled back to frame the whole grid): `hybrid_ddgi` ~0.9-1ms
relighting 512 probes/frame, `hybrid_trace` ~38ms.

**`--stress 10000` full sweep** (30,603-probe grid, camera orbiting the
whole grid at `grid` radius, warmup run discarded, 5 repeats/config,
each config's own 3 in-run samples averaged — same heavy-GPU-contention
caveat as every other sweep in this log):

```
DDGI on  (probes_per_frame=512): ddgi  ~1.1-1.2 ms   trace ~71-84 ms   frame ~85-90 ms
DDGI off (Stage B, sample_count=2, the shipped default): trace ~29-33 ms   frame ~41-46 ms
```

DDGI's shading-time cost (8 occlusion rays + 8 atlas texture reads per
shaded pixel, trilinearly blended) is real and substantial at this
scale — trace time roughly **2.3-2.7x** Stage B's own trace cost, not a
free upgrade. This is the honest tradeoff for DDGI's structural
advantage (genuine long-range, range-independent bounce light Stage B
categorically cannot produce, not just "less noisy at the same range")
and is exactly the kind of cost this log exists to surface rather than
downplay. `probes_per_frame`/the occlusion-ray count are the two levers
most likely to bring this down in a later tuning pass (e.g. dropping to
1 occlusion ray every other probe, or skipping the gate entirely for
probes far from any occluder) — not attempted yet.

**Motion/camera-orbit popping check**: multiple `--shot`/`--at-frame`
captures across a full camera orbit, both at small scene scale (spinning
object + orbiting camera) and at `--stress 200` scale (camera crossing
many probe-grid cells during the orbit) — no visible probe-boundary
discontinuity or popping artifact in any captured frame; lighting reads
stable frame-to-frame. (Static-frame sampling only — a true flicker
signature would need a video capture, not yet done.)

**Grazing-angle light-leak risk assessment**: a box directly on the
ground plane, camera placed low (`y=0.3-0.6`) and close, aimed across
the box-ground contact seam. No harsh bright bleed or unnatural color
spill at the contact line — the seam shows a plausible soft contact
shadow, consistent with the accepted-risk mitigations (probe height
bias off the ground + the single zeroing occlusion ray) doing their job
for this renderer's actual test-scene geometry (convex primitives on
open flat ground, not enclosed rooms — see this entry's own tradeoff
#2). Not a proof no leak is possible in any geometry, just confirmation
the accepted risk doesn't show up badly in the geometry this renderer
actually ships.

**Open, not yet resolved**: whether `hybrid_denoise.wgsl`'s adaptive-
blur and `hybrid_temporal.wgsl`'s reprojection become dead code once
DDGI's own per-probe EMA fully replaces Stage B's noisy per-pixel
source (DDGI's trilinear-sampled result is far less noisy pixel-to-
pixel than Stage B's rotating-hemisphere-sample estimate was, so the
same aggressive denoise/temporal treatment may no longer be earning its
keep) — both passes still run unconditionally today, gated only by the
existing `denoise_enabled`/`temporal_enabled` flags, not by which
indirect source is active. Left as a follow-up tuning question, not
resolved by Stage B's removal below (removing Stage B's code doesn't
answer whether DDGI's own signal still benefits from the same
treatment).

Not yet committed.

### Stage B removed — DDGI is now the renderer's only indirect-diffuse path

Per this project's "direct replacement, not compatibility shims"
convention (CLAUDE.md), and now that DDGI has been verified correct
(CPU-ref tests, visual A/B comparison, motion/grazing-angle checks, and
a real `--stress 10000` GPU sweep — all logged in the entry directly
above), Stage B's per-pixel hemisphere-sample indirect diffuse is fully
removed rather than kept as a permanently-live fallback. Removed:
`cpu_ref.rs`'s `indirect_diffuse`/`indirect_ray`/`indirect_sample_start`/
`tangent_basis`/`jittered_tangent_basis`/`pixel_jitter_angle`/
`INDIRECT_SAMPLES`/`INDIRECT_SAMPLE_COUNT`/`INDIRECT_MAX_T` and their
19 CPU-ref tests (134 -> 115 total lib tests); `shade`'s
`indirect_enabled`/`pixel`/`indirect_sample_count`/`indirect_frame_index`
parameters (down to 9 args from 13) — its `ShadeResult.indirect` field
is now always zero-initialized, filled in by the CALLER via
`ddgi_ref::sample_probe_grid` instead; `IndirectDiffuseConfig` and
`HybridSampleFrameIndex` (`DdgiConfig`/`HybridDdgiFrameIndex` were
already their replacements); `hybrid_trace.wgsl`'s WGSL mirrors of all
of the above; `--indirect`/`--indirect-max-t`/`--indirect-samples` CLI
flags and the "Indirect diffuse (Stage B — legacy)" egui panel section
(replaced by a single "DDGI (indirect diffuse)" section). `shade()`'s
indirect term is now unconditionally DDGI's `sample_probe_grid`, gated
only on `ddgi_enabled` (no more dual-path branch). `sky_color` (still
needed by `ddgi_ref::probe_ray`'s own miss branch) was kept.

`SceneUniform` shrank from 24 to 20 `u32`/`f32` fields (96 -> 80 bytes,
still 16-byte-aligned) in both `extract.rs` and its two WGSL mirrors
(`hybrid_trace.wgsl` AND `hybrid_ddgi_relight.wgsl` — this file's own
copy was still on the stale 24-field/96-byte layout from before DDGI's
`ddgi_enabled` field was even added, not just from this removal).

**Real bug caught at runtime, not compile time**: after the Rust-side
struct shrink, the app crashed on startup with a WGPU validation error
(`Device::create_compute_pipeline, label = 'hybrid_ddgi_pipeline' ...
Buffer structure size 88 ... ended up greater than the given
min_binding_size, which is 80`) — `hybrid_ddgi_relight.wgsl`'s own
`SceneUniform` mirror had NOT been updated to match, so its group-0
binding-0 layout no longer matched the buffer Rust actually uploads.
Rust's `bytemuck`/`ShaderType` derives catch struct-size mismatches
within Rust, but nothing catches a Rust struct vs. WGSL mirror
mismatch except the GPU validator at pipeline-creation time — a real
gap in this project's "CPU-reference-first" safety net for GPU-struct
layout specifically, worth remembering: every WGSL file with its OWN
copy of `SceneUniform` needs updating whenever the Rust struct's field
list changes, not just the WGSL file where the change was "obviously"
needed.

Verified: `cargo build/test/clippy --release --lib --examples` clean
(115/115 tests, no new warnings); live runtime smoke test at both a
small single-sphere scene and `--stress 500` — correct rendering, no
validation errors, no panics, matching GPU-cost profile to the pre-
removal DDGI-on measurements (ddgi ~0.03-0.95ms, trace ~8-40ms
depending on scene scale, consistent with the entry above).

Not yet committed.

### Analytic vs. finite-difference normals: three-way comparison, honest result — no measurable difference at this renderer's current bottleneck profile

Following up on the previous entry's finite-difference `local_normal`
(naive 6-tap central-difference): researched whether closed-form
analytic SDF gradients exist for this renderer's shapes and whether they
survive future CSG composition, per the project's own knowledge base
(`docs/knowledge/sdf-3d/rendering/normal-estimation.md`) and Inigo
Quilez's published smooth-minimum-gradient formula. Confirmed: exact
analytic gradients exist for 7 of the 8 shapes this renderer marches
(all except `RoundedCone`, already skipped for an unrelated pre-existing
distance-formula bug), already implemented in `raymarch.wgsl`'s `sdg_*`
functions for a different pipeline, and CSG composition is well-defined
(hard union = pick the winning operand's gradient; smooth union = blend
both operands' gradients by the same weight the distance blend uses) —
not a dead end. `docs/knowledge/sdf-3d/rendering/normal-estimation.md`
also flagged that the existing 6-tap pattern is more expensive than
necessary — a cheaper 4-tap "tetrahedron" finite-difference pattern
gives equivalent accuracy for a smooth field.

Implemented all three methods side-by-side rather than replacing the
original: `cpu_ref::NormalMethod` (`CentralDiff`/`Tetrahedron`/
`Analytic`), `local_normal_central_diff`/`local_normal_tetrahedron`/
`local_normal_analytic` (the last dispatching to 7 per-shape
`analytic_normal_*` functions, ported verbatim from `raymarch.wgsl`'s
`sdg_*` gradients), `trace_with_normal_method` as the real entry point
(`trace` itself stays a thin `CentralDiff`-defaulting wrapper so every
existing test/call site is unaffected). Same three functions ported to
`hybrid_trace.wgsl`, selected at runtime via `scene.normal_method`
(`SceneUniform`'s new field, threaded through a `NormalMethodConfig`
main-world resource). `examples/gallery.rs` gained `--normals
central-diff|tetrahedron|analytic` (default `central-diff`, unchanged)
plus matching egui radio buttons.

Verified correctness: 27/27 `cpu_ref` tests passing, including ground-
truth checks for all three methods against a sphere (radially outward)
and a box face (axis-aligned) — analytic and both finite-difference
methods agree on these simple cases as expected — plus one dedicated
test per remaining shape's analytic gradient (cylinder side wall,
capsule side, ellipsoid long-axis tip, box-frame outer face, hex-prism
top cap) and a confirmed `RoundedCone` `unimplemented!` for the analytic
path too, mirroring `local_distance`'s existing gap. Visually verified
all three methods produce indistinguishable, correct shading via `--shot`
on both a smooth shape (sphere: identical gradient, no artifacts) and a
sharp-edged shape (hex-prism: identical crisp flat-face shading, correct
edges) — no method introduced visible normal artifacts at edges/corners
worth flagging.

**Performance: measured honestly, and the honest result is "no
measurable difference."** Two measurement passes at `--stress 10000`
(gizmos off, cubes spinning), both cameras:

Pass 1 — all 7 shapes × 3 methods, 5 steady-state samples each (matching
this session's usual quick-check depth): every shape showed all three
methods' averages within roughly 1-2ms of each other, with no consistent
winner — sometimes central-diff fastest, sometimes analytic, sometimes
tetrahedron, varying by shape with no discernible pattern.

Pass 2 — a longer, steadier re-measurement (15 steady-state samples per
combination, ~3x the first pass) on 3 representative shapes chosen to
stress different things: `box` (exact SDF, simplest case), `ellipsoid`
(the previously-flagged bound/approximate SDF costing more march steps),
`hex-prism` (the most branch-heavy analytic gradient, 3 case-split
regions):

```
                    central-diff        tetrahedron         analytic
box/grid:      avg 9.32 (7.3-12.4)  avg 10.56 (7.3-13.9) avg 10.26 (6.6-15.5)
box/near:      avg 12.82 (9.8-15.8) avg 12.45 (8.9-17.4) avg 12.00 (8.1-16.1)
ellipsoid/grid: avg 11.36 (8.6-15.1) avg 11.74 (8.8-14.3) avg 10.42 (7.5-12.4)
ellipsoid/near: avg 16.10 (11.7-20.2) avg 14.81 (12.0-19.6) avg 15.62 (11.9-20.4)
hex-prism/grid: avg 10.16 (8.2-14.2) avg 9.93 (7.4-13.4) avg 10.03 (6.8-12.7)
hex-prism/near: avg 13.46 (10.2-17.8) avg 14.21 (9.4-18.6) avg 12.59 (7.8-18.0)
```
(all times ms, GPU trace only, per-shape/camera min-max ranges in
parentheses)

Every combination's ranges overlap heavily across all three methods —
`analytic` (zero extra `local_distance` evaluations) is not consistently
faster than `central-diff` (6 extra evaluations) anywhere, and no method
is a consistent loser either. This matches the causal prediction made
before measuring: BVH traversal + marching dominates total GPU frame
time so heavily at this object count that normal-computation cost
(paid once per *shaded pixel*, not per march step, and only a handful of
extra SDF evaluations or ALU ops either way) doesn't move the needle.

**Verdict: no change to the default.** `central-diff` (6-tap) remains
the only method — the data doesn't justify switching to either
alternative, and this project's own convention is to only change
behavior when a measurement actually supports it, not on a
plausible-sounding theoretical argument alone (the "reverted
analytic-intersections" and "Stage A slot-table" entries earlier in
this log are the same lesson from two different angles).

Verified: `cargo build/clippy/test` clean (52/52 lib tests, up from 46 —
6 new ground-truth tests for the tetrahedron and analytic methods); zero
`hybrid_legacy` imports (`raymarch.wgsl`'s `sdg_*` functions were read as
a reference to port from, never imported).

**Follow-up, same session: tetrahedron and analytic methods removed
entirely.** Initially kept all three side-by-side (`cpu_ref::
NormalMethod`, `--normals <method>` CLI flag, egui radio buttons) on the
reasoning that they might be worth re-running the comparison against
later. Reconsidered: since the measured result was a clean "no
difference," the extra ~300 lines of `cpu_ref.rs`/`hybrid_trace.wgsl`
code (7 per-shape analytic gradient functions, a second finite-difference
pattern, the `NormalMethod` enum/dispatch plumbing, `NormalMethodConfig`
extraction, the CLI/egui wiring) was pure bloat with no active use —
correct code nobody would ever select over the default. Removed
`local_normal_tetrahedron`, `local_normal_analytic` and its 7
`analytic_normal_*` helpers, `NormalMethod`, `trace_with_normal_method`
(folded back into `trace`), `NormalMethodConfig`, `SceneUniform::
normal_method`/`RenderHybridScene::normal_method`, the `--normals` CLI
flag, and the egui radio buttons — back to exactly one `local_normal`
function in both `cpu_ref.rs` and `hybrid_trace.wgsl`, matching the
state before this investigation started. The full comparison data above
stays in this log as the record of what was tried and why it wasn't
kept, per this project's own "document negative/reverted findings, not
just successes" convention — re-derivable from `raymarch.wgsl`'s
`sdg_*` functions again in a few minutes if the renderer's bottleneck
profile ever changes enough to revisit this.

Verified: `cargo build/clippy/test` clean (46/46 lib tests, back to the
pre-investigation count); visually re-confirmed correct rendering/
lighting via `--shot` after the removal.

### Real lighting: 3 fixed lights (sun/lamp/projector), Lambertian shading, no AO/shadows yet

First real lighting in `src/hybrid`: up to this point every hit reported
its flat material color unmodified (see the "First real trace" entry's
own doc comment for why that was this step's deliberate original scope).
Now a hit's surface normal and every enabled light's contribution are
combined into real Lambertian-shaded output. Deliberately excludes AO
and shadows — explicitly out of scope for this step, to land next.

**Normal computation**: `cpu_ref::local_normal`/`hybrid_trace.wgsl`'s
`local_normal`, central-difference finite differencing of `local_distance`
(`NORMAL_EPSILON = 1e-3`) — shape-agnostic, works identically for every
`Shape` variant with no per-shape analytic gradient formula needed (this
renderer has none; `sdf::primitives`'s `sdg_*` functions exist for a
different pipeline, not reused here). Pinned against two ground-truth
cases: a sphere's normal is trivially the radial unit vector, a box
face's normal is trivially axis-aligned.

**Shading formula**: `cpu_ref::light_contribution`/`shade`, ported from
`hybrid_legacy`'s own `shade()` (`assets/shaders/hybrid_legacy_trace.wgsl`)
— kept only its diffuse/Lambertian term (`albedo * light_color *
intensity * EXPOSURE * attenuation * N.L`, `EXPOSURE = 0.0005` identical
to `hybrid_legacy`'s own empirical constant) and its distance/spot-cone
attenuation, dropping the specular/GGX and shadow-visibility terms
entirely (out of scope). `PointLight`/`SpotLight::intensity` (raw lumens)
is divided by `4*PI` inside the shading code, not at extraction — matches
`bevy_pbr`'s own internal lumens -> luminous-intensity formula exactly,
same deferred-conversion pattern `src/prepass_probe::extract_probe_lights`
already established.

**A real bug caught by `cargo test` before it ever reached WGSL**: the
first spot-cone attenuation transcription used `dot(-to_light,
-spot_direction)` (double-negated), which computes `cos_angle = -1`
exactly on-axis — backwards, the most-lit point reporting as the least-
lit. `spot_light_only_illuminates_inside_its_cone`'s test (checking a
point directly beneath the spot gets positive radiance) failed
immediately with `Vec3(0,0,0)` instead of a positive value, well before
any WGSL was written — exactly the value of this project's CPU-reference-
first discipline. Fixed to `dot(-to_light, spot_direction)` (no extra
negation): `-to_light` is the light-to-surface direction, which equals
`spot_direction` exactly on-axis, giving `cos_angle = +1` there as
expected.

**GPU-side wiring**: `LightGpu` (`src/hybrid/extract.rs`) mirrors
`cpu_ref::Light`'s fields; extraction queries Bevy's real
`DirectionalLight`/`PointLight`/`SpotLight` + `GlobalTransform` directly
(`docs/knowledge/hybrid-architecture/bevy-native-integration.md`'s
established convention — `bevy_pbr`'s clustered light list has no public
bind group a custom pipeline could reuse). Storage buffer, not a fixed
uniform array: explicitly decided against a small fixed-size uniform
(`array<LightGpu, 3>`) once the user clarified the real ceiling is
~128-256 lights eventually, not always exactly 3 — a storage buffer
(mirroring `objects`/`bvh` nodes' existing `RawBufferVec` pattern in
`pipeline.rs`, including the empty-placeholder and bind-group-skip
optimizations already proven there) sizes to the actual light count
every frame with zero rework needed when that ceiling is actually
approached later.

**Per-light enable/disable**: `LightToggles` resource (3 bools), read
during extraction — a disabled light is skipped entirely at extraction
time, not extracted with zero intensity, so it costs nothing in the GPU
shading loop. Driven by both `--sun/--lamp/--projector on|off` CLI flags
and 3 egui checkboxes in the "Lights" section of the "Controls" panel —
same one-shared-resource pattern `DebugGizmos` already established.
Marker components (`SunLight`/`LampLight`/`ProjectorLight`) identify
each of the 3 fixed entities `examples/gallery.rs::spawn_lights` spawns
exactly once at `Startup` — deliberately NOT inside `spawn_scene`'s
per-`--stress`-cell loop, so light count stays fixed at 3 regardless of
`--stress N`, per the explicit requirement that lights are scene-global,
not per-object.

**Debug visibility**: a new bottom-left-adjacent HUD block (top-left,
stacked below the FPS graph — an earlier bottom-right placement visually
collided with the existing bottom-left camera-stats block at typical
window widths, moved after visual verification caught it) shows the
current object's position/rotation/scale (scale meaning each shape's own
defining dimension(s) — this renderer's shapes have no separate uniform
`Transform.scale`) and each of the 3 lights' position/rotation/params,
live. The once-a-second log line carries the identical information
flattened to one line, so a headless run gets the same data.

Verified: `cargo build/clippy/test` clean (46/46 lib tests, up from 38 —
8 new tests: 2 normal ground-truth cases, 6 shading cases covering all 3
light kinds' sign/falloff/cone behavior plus the multi-light-sum
property); zero `hybrid_legacy` imports. Visually verified all 3 lights
individually (sun: flat warm directional lighting with no falloff;
lamp: cool blue point light with clear radial distance falloff; projector:
warm pink spot cone with a visible soft-edged circular light pool and
correct cone cutoff) and combined, across a box, sphere, and hex-prism,
at both near/far orbit camera angles, with all-lights-off correctly
rendering fully unlit black geometry (no ambient term — none exists yet)
and the HUD/log toggle state (`[on]`/`[off]`) matching what was requested.

Performance at `--stress 10000` (20,000 objects, gizmos off, cubes
spinning, all 3 lights on), gpu-timestamp instrumented, both cameras:

```
far/grid:  gpu trace 6.95-9.99ms   | wall-clock p50 14.5-16.7ms
near:      gpu trace 8.25-11.45ms  | wall-clock p50 14.5-15.8ms
```

Comparable to the pre-lighting box baseline from the prior entry (far
6.0-8.6ms, near 8.2-12.3ms) — a modest, expected increase from the
now-real shading loop (finite-difference normal: 6 extra `local_distance`
evaluations per hit, plus 3 lights' attenuation/N.L math), not a
regression. Confirmed light count stays flat regardless of object count:
`--stress 1` (2 objects) measured GPU trace 1.27-1.46ms with all 3 lights
active, consistent with per-pixel shading cost being independent of
scene object count — total GPU cost still scales with object/BVH count,
exactly as required.

### Step 2: each new primitive measured at `--stress 10000`, one real finding (`Ellipsoid`'s bound SDF costs more march steps)

Follow-up to the prior entry's step 1 (spawn + correctness verification,
no performance numbers). Ran all 7 shapes through `--stress 10000`
(gizmos off, `--aabb-gizmos off`, cubes spinning `0.5,0.7,0.3`), both
far/grid and near camera, GPU-timestamp instrumentation already wired
from an earlier session. Steady-state ranges (excluding the first ~5s
startup-noise window, an already-known artifact from earlier sessions —
first-launch frames run measurably slower until the OS/allocator/driver
settle):

```
shape       far gpu trace    far p50       near gpu trace   near p50
box         6.0-8.6ms        14.4-14.9ms   8.2-12.3ms       14.2-16.8ms
sphere      6.8-12.7ms       14.5-18.4ms   10.0-11.5ms      16.7-17.2ms
cylinder    6.5-7.8ms        14.0-14.6ms   8.9-13.1ms       16.7-17.3ms
capsule     6.2-9.8ms        14.3-14.9ms   7.6-15.1ms       14.4-16.9ms
ellipsoid   6.3-10.0ms       14.3-16.3ms   10.6-18.1ms      16.3-21.0ms
box-frame   6.8-10.2ms       14.5-16.2ms   8.5-16.1ms       16.6-17.3ms
hex-prism   6.3-10.0ms       13.9-18.3ms   8.5-12.3ms       16.4-17.1ms
```

**One real finding: `Ellipsoid`'s near-camera GPU trace runs consistently
higher than every other shape** — 10.6-18.1ms vs. box's baseline
8.2-12.3ms and capsule's 7.6-15.1ms (both re-measured with matching
longer sample windows to rule out noise; the gap held up). Root cause,
not a bug: `sd_ellipsoid`'s formula (`k0 * (k0-1) / k1`, ported verbatim
from `sdf::primitives::Ellipsoid`) is a well-known BOUND (approximate)
SDF, not an exact one — its gradient magnitude isn't 1 everywhere off
the surface, unlike every other shape ported here (`Sphere`,
`RoundedBox`, `RoundedCylinder`, `Capsule`, `BoxFrame`, `HexPrism` are
all exact). A non-unit-gradient SDF means `march_object`'s `t += d` step
under-advances relative to the true remaining distance for an elongated
ellipsoid (this instance's `radii = (1.1, 0.7, 0.7)`), so convergence to
`HIT_EPSILON` takes measurably more march steps — directly explaining
the elevated GPU cost. This is an inherent, well-understood property of
that specific SDF formula (the same formula already shipped in `sdf::
primitives::Ellipsoid`, used elsewhere), not something introduced or
fixable by this porting step — flagged here as a known cost, not treated
as a defect to chase.

No other shape showed a real, sustained outlier. `hex-prism`'s far-camera
number initially looked elevated (10.8-13.9ms) in a short first sample
but resolved to 6.3-10.0ms on a longer, steadier run — confirmed as
startup-window noise, not a real per-shape cost.

Verified: same build already verified in the prior entry (no code
changes this step — purely measurement). All numbers gathered via
`RUST_LOG=info`, `RenderDiagnosticsPlugin`'s real GPU timestamps, per
this project's established methodology (never eyeball on-screen FPS).

### Six more `Shape` primitives ported into the trace pipeline (step 1: spawn + verify correctness, not yet stress-tested)

Extends `src/hybrid`'s marching/GPU-upload path from `RoundedBox`-only to
7 of `sdf::components::Shape`'s 8 variants: `Sphere`, `RoundedBox`
(already there), `RoundedCylinder`, `Capsule`, `Ellipsoid`, `BoxFrame`,
`HexPrism`. `RoundedCone` is the one exception — see below. This is
explicitly step 1 of two: spawn each shape, verify it renders/rotates
correctly and gets a correct AABB/BVH box. Step 2 (running each shape
through the `--stress 10000` harness) is deliberately not done here.

All 6 new SDF distance formulas are ported verbatim from `sdf::
primitives`'s already-proven `Sdf::distance` implementations (that
module's own doc comment: "Exact/bound SDF primitives, formulas per
docs/knowledge/sdf-3d/primitives-and-operators") — a transcription job,
not a re-derivation — into both `src/hybrid/cpu_ref.rs::local_distance`
(CPU reference, 6 new `sd_*` functions, one pinned surface/interior/
exterior test per shape) and `assets/shaders/hybrid_trace.wgsl` (same 6
functions, dispatched via a new `shape_kind` tag + `switch` in a new
`local_distance` WGSL function). `scene::local_aabb` already covered all
8 `Shape` variants before this step (confirmed by reading it first) — no
AABB/BVH code needed changing at all; only the marching/GPU-record side
was incomplete.

**`RoundedCone` deliberately NOT ported — a real, pre-existing formula
bug found and confirmed, out of scope to fix here.** While porting,
every point tested against `sdf::primitives::RoundedCone::distance`
(the exact 3-max-term `sdRoundCone` formula, already shipped and used by
`raymarch.wgsl`'s `sdf_rounded_cone`) reported as exterior — including
the shape's own centerline and both endpoints, which cannot be correct
for a well-formed SDF. Verified this isn't a transcription mistake by
diffing the port byte-for-byte against the original and hand-computing
intermediate terms outside the codebase; the bug is in the pre-existing
formula itself. Asked how to proceed rather than silently fixing
already-shipped math outside this step's scope — user chose to skip
`RoundedCone` entirely for now. `local_distance`'s `RoundedCone` arm is
an explicit `unimplemented!`, pinned by a `#[should_panic]` test, so a
future caller gets a loud failure, not a silently wrong distance.
`object_gpu_from` falls back to a degenerate zero-radius sphere rather
than panicking mid-extraction if a scene ever spawns one.

`ObjectGpu` (both `extract.rs`'s Rust struct and `hybrid_trace.wgsl`'s
mirror) redesigned from box-only fields (`half_extents_x/y/z,
corner_radius`) to a `shape_kind: u32` tag + 8 generic `f32` params —
mirrors `sdf::primitives::GpuPrimitive`'s already-established
kind+params encoding pattern, applied fresh here since this renderer's
record also carries transform/color fields that type doesn't. Per-kind
param layout documented on `ObjectGpu`'s doc comment; `march_object`
(WGSL) and `local_distance` (both CPU and WGSL) dispatch on the tag.
Deliberate design choice, confirmed with the user rather than assumed:
every shape (including `Capsule`, whose `a`/`b` a prior renderer
(`hybrid_legacy`) stored in world space and exempted from the entity's
rotation) gets the *same* uniform rotation+translation transform — no
per-shape transform special-casing, simpler than matching legacy
behavior exactly.

`examples/gallery.rs` gained `--shape <name>` (default `box`, i.e.
today's unchanged `RoundedBox` cube): selects which single shape
`spawn_scene` spawns in place of the cube, one at a time (not a
multi-shape showcase row — deliberately kept to "verify one shape
cleanly," matching how the task was scoped) — same ground, same
`--stress N` grid, same `--spin`/`Spinning` rotation mechanism as
always, just a different `Shape` value. Names: `box` (default), `sphere`,
`cylinder`, `capsule`, `ellipsoid`, `box-frame`, `hex-prism`.

Verified: `cargo build/clippy/test` clean (38/38, up from 30 — 6 new
per-shape `local_distance` tests plus the `RoundedCone`-panics test);
zero `hybrid_legacy` imports in `src/hybrid`. Visually verified every
one of the 7 shapes via `--shot`, both rotating (`--spin`) and static,
confirming correct silhouette per shape (sphere round from every angle,
ellipsoid visibly stretched along its long axis when viewed
perpendicular to it — an earlier apparent "looks round" result at the
default orbit angle turned out to be foreshortening from viewing nearly
end-on down the stretched axis, not a bug — cylinder/capsule show their
characteristic rounded-cap profiles, hex-prism shows a true regular
hexagon cross-section from a top-down angle, box-frame's outer envelope
matches a same-half-extents `RoundedBox` exactly as expected since a
closed hollow shell's hollow interior isn't visible from outside without
a cutaway view). Also verified multi-object correctness at `--stress 9`
(spheres) and `--stress 4` (capsules): every object's AABB gizmo
(yellow) tightly and correctly bounds its rotated shape, BVH internal
nodes (blue) union correctly, no occlusion/positioning artifacts.

No performance numbers in this entry — deliberately deferred to step 2
(`--stress 10000` per shape), not yet done.

**Follow-up fix, same session: a real ground-clearance bug found and
fixed by adding an analytical rotation-sampling test.** Requested
explicitly: verify no shape ever intersects the ground at any rotation.
The original per-shape `shape_and_height()` derived spawn height from
each shape's own tight geometric bounding radius (e.g. `Sphere`'s own
`radius`) — plausible-looking, but wrong. `scene::world_aabb` (what the
real renderer's AABB/BVH path calls) computes a rotated world AABB by
rotating `local_aabb`'s 8 BOX corners around the coordinate origin, not
the shape's true surface — so even a rotation-invariant shape like
`Sphere` (whose `local_aabb` is a cube of half-extent `radius`, not a
tight sphere bound) gets an AABB that grows toward the cube's
`radius * sqrt(3)` diagonal at some rotations, well past the sphere's
own radius. A height sized to the shape's tight bound is provably
insufficient.

Caught immediately by a new test,
`examples/gallery.rs::tests::no_shape_choice_ever_dips_below_ground_at_any_rotation`
— samples 200 rotations per shape, computes the real `scene::world_aabb`
(not a re-derivation) at each, asserts `aabb.min.y >= 0`. Failed for
`Sphere` on the very first non-identity rotation sampled
(`object_height=0.95` let the AABB dip to `y=-0.078`).

Fixed by deriving `object_height` uniformly from `local_aabb`'s own
farthest-corner distance from the origin (not from the AABB's own
center — `Capsule`/`RoundedCone`'s `local_aabb` isn't centered at
`ZERO`, so measuring from the AABB's center would itself be wrong in
general, even though every shape actually spawned today happens to have
an origin-centered `local_aabb`) instead of each shape's hand-derived
geometric bound. A second test,
`clearance_formula_holds_for_an_asymmetric_off_origin_capsule`, proves
this generalizes correctly against an intentionally off-center capsule
(endpoints not mirrored around the origin) rather than relying on
today's shapes all happening to be symmetric.

Verified: `cargo test --release --example gallery` 2/2 passing (both new
tests); `cargo build/clippy` clean; visually re-verified all 7 shapes via
`--shot` — each now spawns visibly higher above the ground with a
correspondingly larger AABB gizmo (most visible on `Sphere`/`Ellipsoid`,
whose true bounds are much tighter than their box-shaped `local_aabb`).

### Fix: `Bvh::update`'s same-entity-set refit path rehashed all 20,000 entities every frame

Following the prior entry's finding that "roughly half of far/grid's
frame time is still CPU/engine overhead" at `--stress 10000`, this step
chased down where that CPU time actually goes — with two attempts, one
of which failed and was fully reverted, both recorded here per this
file's own "record regressions and open questions too" rule.

**Attempt 1 (reverted): stable ECS slot table for `Entity`->GPU-index.**
Hypothesis: `extract_hybrid_scene` re-flattening the full `ObjectGpu`/
`BvhNodeGpu` arrays every frame from an unfiltered ECS query, even
though ~19,990 of 20,000 objects never move after spawn, was the cost.
Built and CPU-tested (`src/hybrid/slots.rs`, 7 passing tests including a
10k-entity scale case) a sparse-set `ObjectSlots` giving every entity a
stable GPU buffer index, wired through `Changed<GlobalTransform>`/
`Changed<Material>`/`Added<Shape>` filters so only actually-changed
objects would re-flatten. Implemented fully, all 36 tests passing,
visually verified correct — then measured `--stress 10000` before/after
on both cameras and found **no measurable wall-clock or GPU-trace
improvement**, within noise both ways. The array-copy/flatten itself was
never the bottleneck; the change-detection plumbing added real
complexity (a new module, a `Clone`-losing `RenderHybridScene`, a
`HashMap`-backed slot table synced every frame) for zero proven benefit.
Fully reverted (`git checkout` + file deletion) rather than kept
"because it's not wrong" — matching this project's standing bias against
speculative structure that doesn't earn its keep (the reverted
analytic-intersection tier is the precedent).

**Attempt 2 (landed): stop rehashing the BVH's own entity set every
frame.** Before writing more fix code, added temporary CPU-side
`Instant` timers around `prepare_hybrid_scene` (GPU buffer upload),
`extract_hybrid_scene` (ECS walk + flatten), and `update_persistent_bvh`
(BVH refit) to find the real cost by measurement rather than guessing
again. Results at `--stress 10000`, five-sample steady state:

```
prepare_hybrid_scene:    1.1-3.4ms   (objects=20000, nodes=39999)
extract_hybrid_scene:    2.5-5.1ms
update_persistent_bvh:   collect 1.3-3.7ms | update 4.1-31ms (steady state 4.6-11.8ms far, 4.1-31ms near)
```

`Bvh::update`'s refit path was the dominant, largest, and most variable
cost by a wide margin. Root cause, in `src/hybrid/bvh.rs`: on the
common frame-to-frame path (same entity set, only bounds changed),
`update` called `covers_exactly` (builds a fresh `HashSet<Entity>` over
all leaves plus two `filter()` walks over the whole node array) and then
built a fresh `HashMap<Entity, Aabb>` from all 20,000 input objects —
**two full hash-based structures, rebuilt from scratch, every single
frame**, purely to confirm "nothing was added or removed" and to look up
each leaf's new bound, even though the tree's topology (and therefore
which node index each entity belongs to) is unchanged frame-to-frame in
the overwhelmingly common case.

Fix: `Bvh` now caches a `leaf_index: HashMap<Entity, usize>` (entity ->
node-array index) as a field, built once in `build()` and reused as-is
by every subsequent `update()` call on the same topology — no more
`HashSet`/`HashMap` rebuild per frame. `update()` checks
`leaf_index.len() != objects.len()` as a cheap first gate, then does a
single pass resolving each object's node index directly via
`leaf_index.get`, falling back to a full rebuild if any entity isn't
found (the same-count-but-swapped-membership case the old count-only
check couldn't catch by itself — added a new CPU test,
`update_with_same_count_but_swapped_entity_falls_back_to_full_rebuild`,
since this exact branch wasn't exercised by any existing test).

Re-measured `--stress 10000` after the fix, same instrumentation, five
consecutive samples per camera:

```
far/grid update():  4.6-11.8ms -> 2.3-3.7ms   (steady state, ~2-3x lower)
near     update():  4.1-31ms   -> 2.7-3.0ms   (eliminated a wide unpredictable tail)
```

Wall-clock/GPU-trace after removing the temporary timers and
re-measuring cleanly (`RUST_LOG=info`, five samples per camera,
`--stress 10000`, gizmos off, cubes spinning):

```
far/grid:  wall-clock p50 ~= 14.8-15.4ms | gpu trace ~= 8.0-11.5ms
near:      wall-clock p50 ~= 16.4-21.4ms | gpu trace ~= 10.7-19.5ms
```

Comparable to the prior entry's baseline (far/grid p50 15.7-18.0ms, near
17.4-19.0ms) since GPU trace cost (unaffected by this CPU-only fix) still
dominates wall-clock at this object count — the real, proven win is in
the isolated `update()` measurement above, which removes a large,
unpredictable CPU-side cost spike (previously up to 31ms on some near-
camera frames) that ate directly into frame budget headroom. `extract_
hybrid_scene`'s own re-flatten cost (2.5-5.1ms) remains unaddressed —
Attempt 1 already showed a slot-table fix for it doesn't pay off on its
own; if it's revisited, it should be re-measured against the new,
lower `update()` cost rather than the old inflated baseline.

Verified: `cargo build/clippy/test` clean (30/30, up from 29 — one new
regression test for the swapped-entity fallback path); zero
`hybrid_legacy` imports; visual confirmation via `--shot` at `--stress
200` showing correct rendering/occlusion, no artifacts from the refit
change. All temporary diagnostic timers removed before landing.

### Attempt (landed, but no measured win): `to_gpu_nodes` reused `Bvh`'s own `leaf_index` instead of rebuilding a `HashMap`

Follow-up to the fix above: `to_gpu_nodes` (`src/hybrid/bvh.rs`) had the
exact same shape of cost `Bvh::update` did — it built a fresh
`HashMap<Entity, u32>` from `object_order` (20,000 entries) every single
frame, purely to resolve each BVH leaf's entity to its position in the
per-frame object array. Fixed the same way: instead of building a second
map, walk `object_order` once and look each entity up in `Bvh`'s own
already-built `leaf_index` (added by the previous entry's fix), writing
`right_or_object` directly into the matching node's output slot. All 30
tests pass (including the two `to_gpu_nodes`-specific ones, unchanged in
behavior), visually verified correct via `--shot`.

**Measured honestly, this did not produce a measurable win** — recorded
here per this file's own "note regressions and open questions too" rule,
not just successes. Isolated `to_gpu_nodes` cost after the fix (temporary
split timers, `--stress 10000`, steady state): **1.3-2.8ms**, both
cameras. Whole-function `extract_hybrid_scene` cost: **2.6-4.7ms** — 
statistically the same range as before this fix (2.5-5.1ms). Wall-clock
p50 after the fix (far/grid 13.6-15.3ms, near 14.6-17.0ms) is
indistinguishable from the pre-fix baseline (far/grid 14.8-15.4ms, near
16.4-21.4ms).

Why: removing the `HashMap` *build* only removes half the cost — the
lookups themselves (20,000 `HashMap::get` calls against `leaf_index`)
plus the unconditional flatten of all ~40,000 `BvhNode`s into a fresh
`Vec<BvhNodeGpu>` every frame are inherent O(node count + object count)
work that a hash-avoidance trick alone doesn't touch, since this
function walks the *entire* current tree/object array regardless of how
many objects actually moved. This is the same root cause the fully-
reverted Attempt 1 (stable ECS slot table) already identified: change-
detection/hash-avoidance tricks don't pay off here because the
bottleneck isn't the hashing overhead in isolation, it's paying full
O(N) cost every frame for data that's ~99.95% unchanged. **Left in
place anyway** (not reverted) since it's a strict simplification with
no downside — one fewer full-`HashMap`-allocation per frame, same
correctness, same test coverage, no added complexity — but it should
not be counted as a performance fix. A real fix for this cost would need
to skip touching unchanged nodes/objects entirely (partial/incremental
GPU upload keyed by what actually changed), which is a materially
different and larger change than either hash-avoidance attempt tried so
far.

Verified: `cargo build/clippy/test` clean (30/30); zero `hybrid_legacy`
imports; visual confirmation via `--shot` at `--stress 200`. All
temporary diagnostic timers removed before landing.

### Fix: BVH traversal visited children in a fixed order, not nearer-first — a real GPU-side win

Distinct from the two CPU-side entries above (both about how the
BVH/GPU-buffer arrays get *built*), this targets a GPU-side cost gap the
earlier GPU-timestamp-instrumentation entry already measured but left
unexplained: near-camera GPU trace time (10.7-19.5ms) genuinely higher
than far/grid's (8.0-11.5ms) at `--stress 10000`, the opposite of what
wall-clock-only numbers used to suggest.

Root cause, in both `src/hybrid/cpu_ref.rs::trace` and
`assets/shaders/hybrid_trace.wgsl`'s `trace()`: an internal BVH node's
two children were always pushed left-then-right onto the traversal
stack, with no regard for which one the ray actually reaches first. At
`--stress 10000` the tree is ~14 levels deep (negligible when this
ordering was first flagged as a known gap at just 2 objects — it wasn't
worth fixing then). A camera positioned inside the object grid (the
`near` preset) sends many rays grazing across numerous cells at steep
angles, which — under an unordered descent — can visit a farther
subtree's nodes before `best_t` has been tightened by a closer hit,
performing real (wasted) slab tests and march work.

Fix, ported CPU-reference-first as this project's convention requires:
compute both children's `slab_hit` at the push site (previously only
computed once a node was already popped), and push the farther child
first so the nearer one pops next — LIFO means last-pushed is
first-popped. A child the ray's box test misses entirely isn't pushed
at all (mirroring the early-skip already applied to popped nodes).
Traversal order can never change *which* object wins (the algorithm
still visits every reachable candidate), only how much dead subtree
work happens getting there — proven by two passing `cargo test` cases:
the existing `closer_object_wins_when_two_objects_are_along_the_same_ray`
plus a new `closest_of_three_stacked_objects_wins_regardless_of_
bvh_subtree_placement` (3 stacked objects at increasing distance plus a
decoy well off-ray, added since this exact branch — the near/far
push-order decision itself — wasn't exercised by any existing test).

Re-measured `--stress 10000`, gizmos off, cubes spinning, eight
consecutive once-a-second log samples per camera:

```
far/grid:  gpu trace  8.0-11.5ms -> 6.0-7.3ms   | wall-clock p50 14.8-15.4ms -> 13.3-13.8ms
near:      gpu trace 10.7-19.5ms -> 7.0-11.7ms  | wall-clock p50 16.4-21.4ms -> 13.5-15.5ms
```

**A real, measured win, and it confirms the causal story**: the
near-camera GPU cost gap has substantially narrowed (its previous worst
tail, 19.5ms, no longer appears — new near-camera range is 7.0-11.7ms,
now overlapping far/grid's 6.0-7.3ms far more than before), and both
cameras' wall-clock p50 dropped too, near more than far — consistent
with an unordered-traversal cost that scaled with how much a camera's
rays graze across cells (near) rather than a fixed per-frame cost (which
would have hit both cameras equally).

Verified: `cargo build/clippy/test` clean (31/31, up from 30 — one new
regression test); zero `hybrid_legacy` imports; visual confirmation via
`--shot` at `--stress 200` (identical rendering to prior verification
screenshots, no artifacts, shader compiled and pipeline READY with no
runtime WGSL errors).

### Fix: skip bind-group recreation when nothing they reference actually changed

Smallest of the three fixes this investigation produced, and the last
item on the prioritized list. `src/hybrid/pipeline.rs::prepare_hybrid_
scene` called `render_device.create_bind_group` twice
(`hybrid_compute_bind_group`, `hybrid_blit_bind_group`) unconditionally
every frame, even though both only reference the scene's storage
buffers, storage textures, and the scene uniform — none of which need a
*new* bind group unless the underlying `wgpu::Buffer`/`TextureView`
identity actually changed.

Checked Bevy's own source before touching this (`bevy_render-0.19.1`'s
`buffer_vec.rs`/`uniform_buffer.rs`) rather than assuming: `RawBufferVec::
write_buffer`'s internal `reserve` only allocates a new `Buffer` when
capacity needs to *grow* (`capacity > self.capacity`) — it never shrinks
or reallocates on a stable-or-decreasing count — and `UniformBuffer::
write_buffer` only reallocates when its internal `changed` flag is set,
which `set()` (used every frame here) never touches. So at a stable
object/node count (the common case once a stress scene has finished
spawning), the same three `Buffer`s back every frame's bind groups.

Fix: capture `buffers.objects.capacity()`/`buffers.nodes.capacity()`
before this frame's `write_buffer` calls, compare after: if neither grew,
the existing storage-texture size is unchanged (`needs_new`, pre-existing
logic), and both bind-group resources already exist from a prior frame,
skip bind-group creation entirely and return early. Verified the skip
actually engages via a temporary frame counter: at `--stress 10000`,
bind groups rebuild exactly once (frame 0) and are skipped on every
subsequent frame (600/601 in one measured run) — object/node counts are
stable once `--stress`'s scene finishes spawning, so `buffers_grew` and
`needs_new` are false from frame 1 onward.

Measured `prepare_hybrid_scene`'s own isolated cost (temporary `Instant`
timer, `--stress 10000`, steady state, both cameras): **1.2-2.0ms**,
down from the pre-fix baseline's **1.1-3.4ms** — the lower end is
essentially unchanged (unavoidable buffer clear/push/write cost), but
the upper-end tail comes down meaningfully (3.4ms max -> 2.0ms max).
Smaller than the near/far traversal-order fix's win, consistent with
what this was expected to be going in — `create_bind_group` is a fast
driver-side call, and this function's whole window was already only a
few milliseconds.

Verified: `cargo build/clippy/test` clean (31/31); zero `hybrid_legacy`
imports; visual confirmation via `--shot` at `--stress 200` across 60
frames of continuous cube rotation (exercising the "reuse existing bind
group while buffer contents change every frame" path, not just a static
scene) — no artifacts, correct rendering. All temporary diagnostic
instrumentation removed before landing.

### egui integration: foldable "Controls" window, top-right corner

Added `bevy_egui` (`EguiPlugin` + a system in `EguiPrimaryContextPass`)
and a collapsible `egui::Window` anchored top-right, currently holding
just a `TODO` label — the intended home for future runtime
switchers/color pickers/other controls, added incrementally as
features land. No CPU-testable reference needed (pure UI plumbing, no
numerically meaningful math); verified visually via
`--shot`/`--at-frame` (top-right "Controls" window with fold arrow and
TODO label rendered correctly, FPS HUD unaffected).

Performance: measured the same way as the empty-scene baseline above
(`./target/release/examples/gallery --at-frame 100000`, no `--shot`,
`AutoNoVsync`), five consecutive once-a-second log lines once fps had
stabilized, across two separate runs:

```
fps 226  frame 4.27 ms  avg 4.6  min 3.8  max 6  p25 4.2 p50 4.4 p75 4.9 p95 5.9 p99 6.3 iqr 0.7
fps 224  frame 4.13 ms  avg 4.9  min 3.5  max 7  p25 4.2 p50 4.7 p75 5.5 p95 6.4 p99 6.8 iqr 1.3
fps 255  frame 3.61 ms  avg 3.9  min 3.3  max 5  p25 3.7 p50 3.9 p75 4.1 p95 4.6 p99 5.1 iqr 0.5
fps 238  frame 5.00 ms  avg 3.9  min 3.3  max 5  p25 3.6 p50 3.9 p75 4.2 p95 4.6 p99 4.9 iqr 0.5
fps 235  frame 5.11 ms  avg 4.1  min 3.4  max 5  p25 3.8 p50 4.0 p75 4.4 p95 5.2 p99 5.2 iqr 0.6
```

`p50 ≈ 3.9-4.7ms (220-255fps)` — statistically indistinguishable from
the pre-egui empty-scene baseline (`p50 ≈ 4.0-4.8ms`, 200-250fps).
Egui's own per-frame cost with an empty collapsed-content window is
negligible at this scale; worth re-measuring once the panel holds
real widgets. Landed in commit `edf7ec1`.

### Camera controller (orbit/manual) + origin axis gizmos + debug HUD

Added a real controllable camera (`--camera-mode orbit|manual`, orbit
radius near/far presets or explicit meters, manual pinned
position/rotation) and large X/Y/Z axis gizmos through the origin, plus
a bottom-left debug label (frame number, elapsed time, frame time,
camera position/rotation) and a matching single-line once-a-second log
entry — all pure UI/camera plumbing, no numerically meaningful math to
CPU-test; verified visually via `--shot`/`--at-frame` in both camera
modes (orbit converging on the configured radius, manual holding an
exact pinned pose) and via the log line format. Landed in commit
`d03cb29`.

### AABB/BVH acceleration structure: ground box + spinning cube

First real feature in the fresh-start renderer: a flat, CPU-built,
median-split BVH (`src/hybrid/bvh.rs`) over per-object world-space AABBs
(`src/hybrid/scene.rs`), proven correct against rotation via 8
CPU-testable cases (`cargo test`) before any WGSL exists to consume
it — no SDF marching/shading yet, this step is acceleration-structure
correctness only. Fresh implementation throughout: zero imports from
`src/hybrid_legacy` anywhere in `src/hybrid` or `examples/gallery.rs`
(only doc-comment references to the frozen module as a pattern to
port, confirmed via a targeted `use`/path grep returning no hits) —
`hybrid_legacy::bvh` was read for its algorithm shape only.

Key correctness property: `scene::world_aabb` computes a rotated box's
true world-space bound via `crate::prim::Aabb::transformed`'s 8-corner
method (rotate all 8 local corners, take the extremes), not a naive
"translate the local AABB by the transform's position" shortcut that
would silently under-bound any rotated box. Pinned tests cover the
identity-rotation case, the exact analytic 45°-about-Y diagonal-growth
case (`half * sqrt(2)` on X/Z, untouched on the rotation axis Y), and
containment of all 8 true rotated corners at an arbitrary
non-axis-aligned rotation — plus BVH-level tests proving every internal
node's box exactly unions its subtree's leaves, every input object is
covered by exactly one leaf, and a rotated cube's AABB survives
unchanged from `scene::world_aabb` into the built tree.

Scene: a real ground box (8m × 0.4m × 8m, not a paper-thin plane) and a
cube that spins continuously on independently configurable X/Y/Z axes
(`ObjectSpin`, `--spin x,y,z` rad/s, default `0,0,0` = no rotation on
any axis, live-adjustable via three egui sliders) — both spawned via
the project's existing `sdf::components::Shape`/`sdf::assembly::
SdfSceneRoot` authoring components, nothing new invented for scene
authoring itself. A new `--aabb-gizmos on|off` flag + egui checkbox
(mirroring the existing `DebugGizmos` axis-gizmo toggle) draws every
object's AABB (yellow) and every BVH internal node's box (translucent
blue) each frame, so correctness stays visually checkable continuously
as the cube rotates rather than at a single static pose. Verified
visually across multiple frames/rotation angles, including the
all-zero-spin case showing an exactly axis-aligned, unrotated AABB.

Performance: measured the same way as the empty-scene baseline
(`RUST_LOG=info ./target/release/examples/gallery --spin 0.6,0.9,0.3
--at-frame 100000`, no `--shot`, `AutoNoVsync`), five consecutive
once-a-second log lines once fps had stabilized, ground box + spinning
cube + both AABB/BVH gizmo layers all active:

```
fps 234  frame 4.59 ms  avg 4.4  min 3.7  max 6  p25 4.2 p50 4.3 p75 4.5 p95 5.4 p99 5.4 iqr 0.3
fps 218  frame 4.52 ms  avg 4.6  min 3.8  max 7  p25 4.2 p50 4.5 p75 4.8 p95 5.4 p99 6.4 iqr 0.7
fps 231  frame 4.13 ms  avg 4.6  min 3.8  max 6  p25 4.2 p50 4.6 p75 4.9 p95 5.5 p99 5.7 iqr 0.7
fps 222  frame 4.61 ms  avg 4.6  min 3.9  max 7  p25 4.3 p50 4.5 p75 4.7 p95 5.3 p99 5.8 iqr 0.4
fps 224  frame 4.00 ms  avg 4.8  min 3.7  max 7  p25 4.2 p50 4.6 p75 5.3 p95 6.1 p99 6.7 iqr 1.1
```

`p50 ≈ 4.3-4.6ms (218-234fps)` — statistically indistinguishable from
the empty-scene baseline (`p50 ≈ 4.0-4.8ms`, 200-250fps). Expected at
this object count: 2 objects means a 3-node BVH (one root, two
leaves), CPU-side `Bvh::build` and gizmo drawing are both negligible
work at this scale. Worth re-measuring once object counts grow enough
to matter, and once real SDF marching exists to actually consume the
BVH rather than just building it for gizmo display. Landed in commit
`c1b0842`.

### First real trace: BVH-accelerated flat-color primary rays (compute + blit)

The BVH from the previous step now actually gets consumed: a compute
shader (`assets/shaders/hybrid_trace.wgsl`, one invocation per pixel,
8x8 workgroups) generates a primary ray from Bevy's own `View` uniform,
descends the object BVH iteratively (stack-based, not recursive — WGSL
has no recursion), and sphere-marches `RoundedBox` candidates via
`sd_rounded_box`, reporting the closest hit's flat `Material.base_color`
or a flat background color on a miss. A fullscreen fragment blit
(`assets/shaders/hybrid_blit.wgsl`) copies that color into the view
target and reconstructs reverse-Z `frag_depth` from the stored linear
hit-`t`, so hybrid content composites correctly against Bevy's own depth
buffer. No lighting/shadows/reflections — that is deliberately out of
scope for this step (see `cpu_ref.rs`'s doc comment).

Ported faithfully from `src/hybrid/cpu_ref.rs` (5 passing `cargo test`
cases: straight hit color/distance, hit-behind-camera miss,
ray-misses-scene, rotated-cube-hits-at-correct-distance, closer-of-two-
overlapping-objects-wins) — same `MAX_MARCH_STEPS = 128`/
`HIT_EPSILON = 1e-4` marching tolerances, same branchless Kay-Kajiya
slab-test formula, same closest-hit-wins iterative BVH descent, verified
line-by-line against the CPU reference rather than redesigned. New GPU
plumbing: `src/hybrid/bvh.rs`'s `BvhNodeGpu` (flat scalar mirror of
`BvhNode`, `Entity` resolved to a per-frame object-array index) with a
`to_gpu_nodes` conversion proven structure/bounds-preserving by 2 new
`cargo test` cases (29 total passing, up from 27); `src/hybrid/scene.rs`'s
`HybridObjectData`/`collect_data` (shape/material/transform per object,
parallel to the existing AABB-only `HybridObject`/`collect`, which stays
unchanged and still backs the AABB/BVH gizmos); `src/hybrid/extract.rs`
(manual `Extract<Query<...>>` aggregation into a render-world
`RenderHybridScene` resource — the documented pattern for "aggregate many
entities into one resource," see `docs/knowledge/bevy-rendering/
architecture/entity-sync-and-extraction-patterns.md`); `src/hybrid/
pipeline.rs`/`pass.rs` (bind groups, storage textures, dispatch + blit
sequencing, mirroring `hybrid_legacy::pipeline`/`pass`'s shape as a
read-only reference, not reused code).

Deliberate deviation from the brief's suggested `SceneUniform` shape:
**no duplicated `view_proj` matrix.** Investigated Bevy's real `View`
WGSL struct (`bevy_render::view::View`) before deciding — it already
carries `world_position`, `world_from_clip`, `clip_from_world`, and
`viewport`, which is everything both the trace shader's ray generation
and the blit shader's depth reconstruction need. `hybrid_legacy`'s own
`SceneUniform` duplicated `view_proj` mainly to carry `tan_half_y`
alongside it for its screen-space pixel-footprint antialiasing math,
which this flat-color-only step has no equivalent of — so there was
nothing that actually justified a second copy of the projection matrix.
`src/hybrid/extract.rs`'s `SceneUniform` here is much smaller
(`object_count`, `bvh_node_count`, a flat `background_r/g/b`) and reads
directly off Bevy's real View uniform for everything camera-related, per
`docs/knowledge/hybrid-architecture/bevy-native-integration.md`'s
already-established convention.

Also fixed two pre-existing gaps in `examples/gallery.rs` blocking this
step (both already correctly handled in `examples/gallery_legacy.rs`,
just never carried over to the fresh-start example): no `AssetPlugin`
override (shaders failed to load — Bevy's default resolves `assets/`
relative to the *executable's* directory, not the process cwd) and no
`Msaa::Off` on the camera (Bevy's default 4x MSAA is incompatible with
the hybrid blit pipeline's single-sample `RenderPipelineDescriptor`,
a real wgpu validation error). Both fixed with the same one-line
solutions `gallery_legacy.rs` already uses.

Added `--cube-color r,g,b` (`CubeColor` resource, same hand-rolled
comma-split-parse convention as `ObjectSpin::from_args`) plus an egui
`color_edit_button_rgb` in the "Controls" panel (this codebase's first
color picker), applied to the cube's `Material.base_color` every frame
via a new `apply_cube_color` system keyed on the existing `Spinning`
marker.

Verified: `cargo build --release --lib --examples` clean;
`cargo clippy --release --lib --examples --no-deps` zero new warnings
(all `src/hybrid/*.rs`/`examples/gallery.rs`/new WGSL clean; pre-existing
warnings in `sdf/`, `hybrid_legacy/`, `raymarch/`, `compute_spine.rs`,
`gallery_legacy.rs` untouched); `cargo test --release --lib` 29/29
passing; `grep -rn hybrid_legacy src/hybrid examples/gallery.rs` — every
hit is doc-comment prose, zero `use`/code-path references. Visually
verified via `--shot`/`--at-frame`: ground plate renders solid grey,
cube renders solid orange, correct depth occlusion between them, AABB/
BVH gizmo wireframes overlay correctly on top; `--cube-color 0.1,0.8,0.9`
renders the cube cyan/teal instead; default orbit camera at a later
frame (a different viewing angle) renders correctly too.

Performance: measured the same way as prior entries
(`RUST_LOG=info ./target/release/examples/gallery --spin 0.6,0.9,0.3
--at-frame 100000`, no `--shot`, `AutoNoVsync`, default 720p window,
default orbit camera — so, unlike the AABB-only baseline, the trace
shader is now actually running full-screen every frame), six consecutive
once-a-second log lines once fps had stabilized:

```
fps 176  frame 5.41 ms  avg 5.7  min 4.0  max 10  p25 5.0 p50 5.5 p75 5.9 p95 7.4 p99  9.1 iqr 1.0
fps 138  frame 6.98 ms  avg 7.0  min 4.0  max 28  p25 5.8 p50 6.5 p75 7.1 p95 9.6 p99 10.7 iqr 1.3
fps 151  frame 6.05 ms  avg 8.8  min 5.0  max 16  p25 6.6 p50 8.1 p75 10.5 p95 13.3 p99 15.1 iqr 3.9
fps 170  frame 6.29 ms  avg 6.0  min 4.3  max 10  p25 5.1 p50 5.9 p75 6.5 p95 8.2 p99  8.8 iqr 1.4
fps 152  frame 6.00 ms  avg 5.3  min 4.3  max 8   p25 4.7 p50 5.1 p75 5.7 p95 7.2 p99  7.5 iqr 0.9
fps 162  frame 6.46 ms  avg 5.7  min 4.4  max 8   p25 5.0 p50 5.6 p75 6.1 p95 7.4 p99  7.6 iqr 1.1
```

`p50 ≈ 5.1-8.1ms (138-176fps)` — up from the AABB-only baseline's
`p50 ≈ 4.3-4.6ms (218-234fps)`: real full-screen compute-trace +
fragment-blit cost, as expected now that pixels are actually being
marched instead of just building/drawing the BVH's gizmo wireframes.
Still comfortably faster than `hybrid_legacy`'s lit/shadowed baseline
(`p50 ≈ 19ms` at its own default orbit angle) — unsurprising, since this
step has no lighting/shadows/AO/reflections yet, only flat-color hit
resolution.

**Performance audit against SOTA practice** (requested explicitly,
audited rather than assumed): this is at or very near the practical
floor for what it does. The measured delta above baseline is ~1-3.7ms
of actual trace+blit cost; the rest is fixed CPU/engine overhead
(input handling, egui, HUD systems) already present before any GPU
marching existed. At 720p that's roughly 2ns/pixel of GPU work —
consistent with a 3-node BVH descent (1-2 slab tests) plus a handful
of march steps per pixel against two well-separated boxes, not
evidence of inefficiency. Workgroup size (8x8=64) already matches the
compute-shaders KB's own "single-wave 64-thread" guidance for this
GPU class. Two real but currently-negligible findings, correctly
deferred per this project's own reverted-analytic-tier lesson against
premature optimization: BVH child traversal doesn't order near/far
(only matters once tree depth grows past ~3-4 levels), and bind
groups/buffers are recreated unconditionally every frame (currently
microseconds at 2 objects, worth a dirty-flag guard once object counts
or per-object payload grow). Neither is worth fixing at today's scale.

Also fixed same-session, both verified: background/miss color changed
to magenta (the conventional "nothing was hit" debug color, so a
genuine miss is unmistakable rather than blending in as a plausible
sky — was a plain mid-grey); cube spawn height corrected to its own
bounding-sphere radius plus margin so no rotation ever dips a corner
below the ground (verified across multiple heavily-tumbling frames).

Landed in commit `53b3f4a`.

### Fix: extraction rebuilt the whole BVH from scratch every frame

Requested explicitly: a real `--stress 10000` baseline (10,000 objects,
gizmos off, cubes spinning) to move performance tuning from theory to
measurement. Baseline, both near and far/grid-framed camera (per the
request to check both, since they stress different things):

```
far/grid:  p50 ~= 30.3-32.2ms (20-39fps)
near:      p50 ~= 32.8-39.6ms (28-35fps)
```

The near camera being *slower* than far despite most rays terminating
almost immediately (background or one nearby cell) was the tell: the
bottleneck wasn't GPU marching scaling with visible geometry, it was a
fixed per-frame cost paid regardless of what's on screen. Investigation
found it: `src/hybrid/extract.rs`'s `extract_hybrid_scene` called
`Bvh::build()` — a full SAH-bucketed rebuild, 12 buckets x 3 axes
evaluated per split — from scratch, every single frame, over all
10,000 objects, even though 9,999 of them (every ground box) never
move after spawn. A separate, already-correct, already-*refit* BVH
existed the whole time (`examples/gallery.rs`'s old `HybridBvh`) but
was used only for gizmo drawing and never reached the GPU — extraction
was silently redoing all that work from scratch in parallel.

Fix: there is now exactly one persisted BVH in the whole app —
`hybrid::bvh::PersistentBvh`, a main-world resource refit once per
frame by `hybrid::bvh::update_persistent_bvh` (registered directly by
`HybridRenderPlugin`, not by the example). `extract_hybrid_scene` now
extracts (`Extract<Res<PersistentBvh>>`) and flattens that already-refit
tree instead of rebuilding; `examples/gallery.rs`'s gizmo-drawing code
now reads the same shared resource instead of maintaining its own
duplicate. One BVH, one update, two consumers.

Also hardened, found during the same investigation (a correctness
concern, not a performance one): `hybrid_trace.wgsl`'s traversal stack
was `MAX_STACK = 32`, and overflow silently drops the push — an
entire subtree removed from traversal with zero signal, a *wrong*
result rather than a crash. Not an active bug (the SAH builder's
by-construction depth bound, `O(log2 N)`, keeps real trees far under
32 even at 10,000 objects — depth ~14), but too little margin for a
guarantee resting entirely on that reasoning being remembered
correctly forever. Raised to 64 (log2(1,000,000) ~= 20, generous
headroom above any realistic scene) with the depth-bound reasoning
made explicit in both the constant's and the push-guard's own comments,
so the next person changing the builder's split strategy has the
actual safety argument in front of them, not just a bare number.

Re-measured identically after both fixes:

```
far/grid:  p50 ~= 16.3-18.0ms (52-91fps)  -- ~1.7-1.9x faster
near:      p50 ~= 18.0-20.6ms (29-91fps)  -- ~1.8-2.0x faster
```

Near and far are now much closer to each other than before (a ~2ms
gap vs. the prior ~7-8ms gap) — consistent with the fix removing a
shared fixed cost that was swamping the real camera-distance-dependent
GPU cost difference, rather than either camera path being independently
optimized.

Verified: `cargo build/clippy/test` clean (29/29), zero `hybrid_legacy`
imports (grep-verified), visual confirmation at `--stress 25` showing
correct rendering/occlusion/BVH-gizmo overlay after the refactor.
Landed in commit `abf57c1`.

### GPU timestamp instrumentation — the "close to ceiling" claim checked for real

After the BVH-rebuild fix above, this project claimed the trace pipeline
was "close to theoretical ceiling." Challenged, correctly — that claim
had no evidence behind it: every number so far was wall-clock frame
time (CPU extraction + GPU dispatch + blit + egui + HUD + present, all
conflated), with no way to separate CPU-side cost from actual GPU work.
Fixed properly this time, per explicit instruction to build the right
instrumentation "as soon as possible... to make everything crystal
clear" rather than keep guessing.

Wired `bevy::render::diagnostic::RenderDiagnosticsPlugin` (not
auto-installed by `DefaultPlugins` — needs the plugin added explicitly,
but needs no device-feature request: Bevy's default `WgpuSettings`
priority already requests every feature the adapter advertises,
`TIMESTAMP_QUERY` included on this dev machine's Vulkan/RADV backend)
and two real GPU timestamp spans in `src/hybrid/pass.rs::hybrid_pass`
(`"hybrid_trace"` around the compute dispatch, `"hybrid_blit"` around
the fullscreen blit draw), surfaced in both the on-screen HUD and the
once-a-second log line alongside the existing wall-clock numbers —
CPU-inclusive and GPU-only cost visible side by side now, not one or
the other. Also corrected a false claim this same instrumentation
uncovered in `docs/knowledge/compute-shaders/cpu-gpu-data-flow.md`
("RenderDiagnosticsPlugin - enabled in this project" was written before
it actually was).

Re-measured `--stress 10000` (gizmos off, cubes spinning) with the
real split:

```
far/grid:  wall-clock p50 ~= 15.7-18.0ms | gpu trace ~= 7.3-7.7ms  + blit ~= 1.3-1.4ms
near:      wall-clock p50 ~= 17.4-19.0ms | gpu trace ~= 11.3-15.0ms + blit ~= 1.1-1.4ms
```

Two honest findings this produced, previously invisible:
- **Roughly half of far/grid's frame time is still CPU/engine
  overhead** (wall-clock ~16-18ms vs. GPU-only ~8.7-9.1ms) — real
  headroom exists there specifically (extraction still fully re-
  flattens the object array and the BVH-node array every frame,
  ~1.24MB combined at 10k objects, even though only ~10 cubes actually
  move — flagged, not yet fixed).
- **Near camera GPU cost (~11.3-15.0ms) is genuinely higher than
  far/grid's (~7.3-7.7ms)** — the opposite of what the earlier wall-
  clock-only numbers suggested (they looked similar). Makes sense once
  visible: a camera positioned inside the grid has many rays grazing
  across numerous cells at a steep angle before hitting anything or
  the sky, costing more real BVH traversal/marching than a distant
  view looking down at a compact grid — a genuine GPU-side cost
  difference the wall-clock numbers had been masking entirely.

**Honest verdict, now grounded in actual data rather than a guess:
"close to theoretical ceiling" was not true.** There is measurable,
real headroom — the CPU-side re-flatten cost is a concrete, sizeable,
fixable target, and the near-camera GPU cost gives a second, distinct
axis to investigate (traversal-order/marching efficiency under grazing
rays specifically, not just raw object count). Both are now real,
measurable next steps instead of speculation — which was the entire
point of building this instrumentation.

Verified: `cargo build/clippy/test` clean (29/29), zero `hybrid_legacy`
imports (grep-verified), GPU timing confirmed live in both HUD
(`gpu trace 1.16 ms  blit 1.21 ms` at the default 2-object scene) and
log line. Landed in commit `c473052`.

### Metallic-roughness GGX material (Cook-Torrance BRDF)

Replaced `hybrid`'s flat-Lambertian shading with a full metallic-roughness
GGX Cook-Torrance BRDF, and gave `hybrid` its own PBR material type.

**Material split, not extension.** `sdf::components::Material` (the
original 3-field `base_color`/`metallic`/`roughness` struct, used by
`hybrid_legacy`/`raymarch`/`sdf::world`/`bench::scenes` and ~15 call
sites) was renamed to `MaterialLegacy`, left otherwise untouched. A new
`hybrid::material::Material` (`base_color`, `metallic`, `roughness`,
`reflectance`, `emissive`) was created in its own module, used only by
`src/hybrid`'s own files (`cpu_ref.rs`, `extract.rs`, `scene.rs`) and
`examples/gallery.rs`. Deliberate clean break, not a backward-compatible
builder extension — matches this project's established pattern of
`hybrid` and `hybrid_legacy` not sharing types, so new PBR fields don't
get threaded through call sites that have no shading path for them.
`reflectance` remaps to a dielectric's F0 the same way `bevy_pbr` does
(`F0 = 0.16 * reflectance^2`); `emissive` adds on top of lit shading,
unaffected by scene lights.

**A real bug found and fixed (in the new code, not the old).** While
porting `hybrid_legacy_trace.wgsl`'s `shade()` as a starting reference,
its specular term turned out to double-apply F0/Fresnel: `spec = D * Vis
* f0` (F0 baked directly into what should be a pure `D*V` term), then
multiplied by a *second*, independently-computed `fresnel_schlick(f0,
...)` on top — so the final specular contribution scales with F0
squared, not once. Verified against real `bevy_pbr` source
(`pbr_lighting.wgsl`), which confirmed the correct structure is `D * V *
F` with F0 folded into `F` exactly once. This is a genuine bug in
`hybrid_legacy` (visibly too-dark specular highlights on rough
metals/high-reflectance dielectrics) — left as-is there per this
project's frozen-legacy-code convention, flagged here rather than
silently fixed or silently replicated. The new `cpu_ref.rs::shade`
applies F0 once, with a regression test
(`specular_highlight_scales_with_f0_once_not_squared`) guarding against
reintroducing the same mistake.

**CPU reference first, then WGSL, per the project's standard
convention.** `cpu_ref.rs` gained `ggx_distribution` (Trowbridge-Reitz
NDF), `ggx_visibility` (height-correlated Smith, folded with the Cook-
Torrance `1/(4 NdotL NdotV)` denominator), `fresnel_schlick`, and
`dielectric_f0`, composed in a rewritten `shade()` that also adds
`material.emissive` on top. Energy-conserving diffuse (`diffuse_color *
(1-F)`), metals correctly zero their own diffuse term
(`diffuse_color = albedo * (1 - metallic)`). No multi-scatter energy-
compensation term (Filament/`bevy_pbr`'s `Fr *= 1 + F0*(1/F_ab.x - 1)`
needs a precomputed BRDF-integration LUT this renderer doesn't have yet;
its absence shows up only as slightly-too-dark rough metals, not a
correctness bug). 24/24 `hybrid::cpu_ref` tests pass, including new
metallic-vs-dielectric diffuse-response and F0-single-application
regression tests. Ported verbatim into `hybrid_trace.wgsl` (`sample_light`
replaces the old `light_contribution`, returning attenuation/radiance
without an N·L baked in so both diffuse and specular terms can share one
per-light computation instead of duplicating it) — `Hit` now carries the
matched object's index instead of a precomputed flat color, since
`shade` needs the full material record, not just `base_color`.
`ObjectGpu`/`hybrid::extract::ObjectGpu` both extended with
`metallic`/`roughness`/`reflectance`/`emissive` fields (kept 16-byte-
aligned via explicit padding, matching this struct's existing
convention).

**Gallery**: new `CubeMaterialParams` resource (mirrors `CubeColor`'s
established "one resource, CLI flag and egui panel both drive it"
pattern) — `--cube-metallic`/`--cube-roughness`/`--cube-reflectance`/
`--cube-emissive r,g,b` CLI flags, plus Metallic/Roughness/Reflectance
sliders and an Emissive color picker in the "Cube material" panel
section, applied every frame by the existing `apply_cube_color` system
(renamed scope, not renamed function — still the single per-frame
material-sync point).

**Visual verification (stress 1, all 7 shapes, gizmos off)**: every
shape (box, sphere, cylinder, capsule, ellipsoid, box-frame, hex-prism)
renders correct GGX response — a dielectric ground plane shows a soft
directional gradient; a metallic cube (metallic 1.0, roughness 0.5)
shows a genuine specular highlight with correctly darkened off-highlight
faces (no diffuse term, as expected for a pure metal); a smooth metal
(roughness 0.15) renders mostly black from most angles with the
highlight visible only where a light's reflection direction actually
aligns with the view — physically correct mirror-like behavior, not a
bug (confirmed by cross-checking against a mid-roughness render at the
same angle); a sphere/ellipsoid under 3 lights shows multiple distinct,
correctly-curved highlights, one per light; emissive glows through on a
surface's shadowed side, additive on top of the normal lit response.
Screenshots taken via `--shot PATH --at-frame N` (N=60, giving the async
GPU pipeline time to report "trace pipeline READY" before the capture —
an N=30 capture in this session raced the pipeline init and produced a
spurious all-black frame, not a real bug, worth remembering for future
verification passes).

**Performance (`--stress 10000`, gizmos off, cubes spinning, both
camera positions, `--bench 15`)** — full GGX Cook-Torrance vs. the
pre-GGX flat-Lambertian baseline logged earlier in this file:

```
                    Lambertian (baseline)         GGX Cook-Torrance (this work)
far/grid: p50       15.7-18.0ms | gpu 7.3-7.7ms   15.5-16.8ms | gpu 6.1-10.8ms (~6-7ms typical)
near:     p50       17.4-19.0ms | gpu 11.3-15.0ms 14.8-16.4ms | gpu 6.7-11.2ms (~7-8ms typical)
```

No measurable regression from adding the full metallic-roughness GGX
BRDF (specular D/V/F, energy-conserving diffuse, emissive) over flat
Lambertian — numbers land in the same band, confirming the shading math
itself is not the bottleneck at this scale (BVH traversal/marching
still dominates, per the earlier GPU-timestamp investigation above).
Meets the "close to theoretical ceiling performance" goal this feature
was built against: full PBR-quality shading came at effectively zero
additional cost.

Verified: `cargo build/test --release` clean (49/49 lib tests, plus
`hybrid_trace_wgsl_parses` in `tests/wgsl_parse.rs`), `cargo clippy
--release` clean (only pre-existing warnings in unrelated files). Not
yet committed.

### Soft shadows — GI trajectory Stage A

First step of the agreed GI trajectory (A: shadows → B: cheap fixed-probe
indirect → C: occupancy-grid secondary-ray acceleration → maybe D: full
DDGI). Shadows had to land first since indirect bounces need to respect
occlusion too, and porting a proven `hybrid_legacy` technique into
`src/hybrid` again re-validates the whole "CPU-ref first, then WGSL"
pipeline that worked for lighting and materials.

**Not a re-run of a known-bad implementation — checked explicitly before
porting anything.** `hybrid_legacy`'s shadow code was flagged as a risk
going in, since the shadow-quality investigation is part of why this
renderer was rewritten from scratch. Read `hybrid_legacy_trace.wgsl`'s
`trace_shadow` and its CPU reference (`src/hybrid_legacy/cpu_ref.rs`) in
full, plus `docs/knowledge/sdf-3d/rendering/soft-shadows-and-ao.md`'s
whole investigation: the KB doc's prose trails off mid-investigation
("paused to build better tooling"), but the actual shipped code is the
*final, fixed* state — three regression tests there
(`real_near_contact_matches_shader_probe_630_405`,
`real_hard_hit_matches_shader_probe_600_380`,
`fixed_729_380_no_longer_truncated_by_short_aabb_slab`) are cross-checked
against real GPU debug-buffer probes, not just internally self-consistent
math. So this stage ported proven, validated logic, not the buggy draft —
confirmed rather than assumed.

**Two coupled fixes, both required** (the KB doc's own finding — fixing
either alone reintroduces a different artifact):
1. **March bound**: a shadow ray's march is bounded by its own real
   `max_t` (light distance/range, or `2 * scene_root_diagonal` for
   directional lights — a self-scaling stand-in for `hybrid_legacy`'s
   fixed `shadow_t = 60.0`), NOT a BVH candidate's own AABB slab exit —
   the slab exit is an acceleration-structure artifact, not a physically
   meaningful stopping point for a soft near-miss query. Bounded by a
   scale-relative divergence early-out (`DIVERGENCE_FACTOR = 2.5`) so a
   genuinely-missed ray still terminates promptly.
2. **Candidate margin**: every BVH leaf's AABB (not internal nodes) is
   padded by `margin = VIS_CUTOFF * k * scene_root_diagonal` before the
   ray/box slab test — without it, a ray that never enters an occluder's
   *tight* AABB gets zero candidates for that object at all, producing a
   polygonal (bounding-box-shaped) shadow silhouette instead of the
   occluder's true (e.g. round) shape.

Ported into `src/hybrid/cpu_ref.rs` first: `Bvh::root_diagonal`,
`gather_candidates_padded`, `trace_shadow` (Aaltonen's closest-point-
refined `k*h/t` penumbra formula), `shadow_bias`/`pixel_eps`, wired into
`shade()`. Verified with 4 new tests reconstructing `hybrid_legacy`'s own
pinned-test scene shape (ground + sphere) against this module's own
`TraceObject`/`Bvh` API — critically, two of the four (the margin-fix
regression tests) were verified NOT just by passing, but by confirming
they actually *fail* when `shadow_candidate_margin` is temporarily
disabled (a first attempt at these tests silently passed either way,
i.e. tested nothing — caught and fixed by numerically searching for ray
geometry that genuinely exercises the "misses tight AABB, needs the
margin" case rather than guessing coordinates). 53/53 total lib tests
pass.

Ported verbatim (same constants) into `hybrid_trace.wgsl`: `LightGpu`
gained `shadow_softness_k`; `SceneUniform` gained `shadows_enabled` (a
real kill-switch — shadow rays are skipped entirely when off, not just
multiplied by `vis=1.0`, matching `LightToggles`'s existing "costs
nothing when disabled" principle). New `hybrid::extract::ShadowConfig`
resource (`enabled: bool`, `k: f32`, default `k=12.0` matching
`hybrid_legacy`'s own default), extracted every frame alongside
`LightToggles`. Gallery gets `--shadows on|off` / `--shadow-k F` CLI
flags plus a "Shadows" egui panel section (checkbox + softness slider,
disabled when shadows are off).

**Visual verification (stress 1, all 7 shapes, gizmos off, `--at-frame
90`+ to clear the async-pipeline-ready race — an `--at-frame 30` capture
raced the GPU pipeline init and produced a spurious near-empty frame,
same issue noted in the materials-stage entry above)**: every shape casts
a smooth, correctly-silhouetted soft shadow — round under the sphere,
elliptical under the ellipsoid, hexagonal under the hex-prism, both
rounded ends visible under the capsule — with visible penumbra softening
and distinguishable overlapping shadows from all three lights (sun/lamp/
projector). `--shadows off` confirmed to remove shadows entirely and
measurably drop GPU cost, not just visually toggle.

**Performance (`--stress 10000`, gizmos off, `--bench 15`, both camera
positions, shadows on vs. off)**:

```
                shadows off (p50 / gpu trace)   shadows on (p50 / gpu trace)
near (inside grid): 18.7-23.6ms / 14-20ms        31.2-36.0ms / 27-37ms
grid (outside):      18.0-20.0ms / 13-18ms        21.9-28.6ms / 19-26ms
```

Real, substantial cost from shadow rays — expected and explicitly
predicted before measuring (per this project's "no future claim without
a number" convention, this was measured, not assumed): shadow candidate
gathering + soft-march is real extra GPU work per light per pixel, with
no secondary-ray acceleration yet. The near camera (inside the object
grid, many nearby candidates per shadow ray) pays more than the grid/far
camera (mostly empty space beyond the scene) — consistent with Stage C's
whole rationale (an occupancy-grid accelerator specifically targets
secondary-ray cost, which is exactly what's expensive here) and exactly
the kind of real number this trajectory's Stage C is meant to improve,
not a target to hit blindly now.

`docs/knowledge/sdf-3d/rendering/soft-shadows-and-ao.md`'s stale
mid-investigation ending updated to note the fix landed and is now
validated in two independent renderers.

Verified: `cargo build/test/clippy --release` clean (53/53 tests, no new
warnings). Not yet committed.

### Soft shadows follow-up: real bug at `--stress N`, found via ground-truth debug scan, not screenshots

The Stage A shadow work above shipped, but a real, user-caught bug
surfaced immediately after: at `--stress 100`+, individual cubes cast
wildly elongated shadow streaks — several units long, stretching into
neighboring `--stress` grid cells — rotating correctly with the light's
azimuth (ruling out a static rendering artifact) but reaching far past
any single cube's true, physically-correct shadow length (~0.6 units at
this scene's ~70° sun elevation). AABB/BVH gizmos made this look
correlated with internal BVH node boxes, which was a red herring — the
gizmo boxes were geometrically *near* the real cause but not themselves
implicated in it.

**Two false starts, both plausible-sounding, both wrong, both caught by
actually reproducing the bug with real numbers instead of iterating on
screenshots:**

1. First fix attempt: `shadow_candidate_margin`'s reference distance
   (`t_reference`, used to derive how much to pad leaf AABBs before the
   candidate slab test) had been ported as `scene.root_diagonal()` —
   sound for `hybrid_legacy`'s tiny single-object demo scene, but at
   `--stress N` the BVH root spans the WHOLE multi-cell grid, so the
   margin grew with total object count and padded every leaf by tens of
   units. Fixed by switching the reference to the shadow ray's own
   `max_t` instead. This was a real bug and a real fix, but did NOT
   resolve the visible artifact — confirmed by rechecking the actual
   screenshot instead of assuming success from the formula reasoning
   alone.
2. Second fix attempt: `DIRECTIONAL_SHADOW_MAX_T` itself (bounding a
   directional shadow ray's march distance, no natural light-distance to
   use) had the same `root_diagonal`-scaling flaw. Fixed by switching to
   a fixed distance — but reused `hybrid_legacy`'s own `60.0` constant,
   which was ALSO wrong: that value was tuned for a scene only ~11 units
   across, where `60.0` was ~5x the whole scene and harmless. In this
   project's `--stress N` grid (cells also ~11 units, tiled edge-to-edge),
   a shadow ray at `max_t=60` legitimately travels ~20 horizontal units
   before stopping — crossing into 1-2 neighboring cells and picking up
   their real (but locally irrelevant) geometry as genuine shadow
   candidates. Shrunk to `12.0`. Still did not fully resolve the artifact.

**What actually found the real cause: a permanent CPU-side debug scan
reproducing the exact real scene, not more screenshot iteration.**
`src/hybrid` has no GPU debug-readback tooling (`hybrid_legacy` has one,
`src/hybrid_legacy/debug.rs`, built for exactly this class of problem —
noted as a gap worth closing before the next hard-to-diagnose shading
bug). Built the CPU-side equivalent instead:
`cpu_ref::tests::stress_100_scene` reconstructs the REAL `--stress 100`
scene exactly (10x10 grid, real cell-center formula, real ground/cube
half-extents) and `debug_stress_100_full_grid_shadow_scan` samples every
cell's own ground footprint, excluding that cell's own ground as a
candidate, checking for any point darkened by something other than that
cell's own cube. This immediately found a concrete, numerically-verified
failing point: `p=(-51.5,0,14.5)` in cell 60, `vis=0.044` — while the
cube casting it was independently confirmed (via its height and the sun's
elevation angle) to have a true shadow tip only reaching to
`(-49.98,16.18)`, 2.27 units away from the flagged point. Printing the
exact step-by-step march (`t`, `h`, `d`, `sample_vis` at each sample)
against this real candidate showed the formula itself producing
`sample_vis=0.05` at `t≈2.0` from a genuine `d≈1.0` unit gap — not a
phantom candidate, not a margin-padding bug, but the Aaltonen `d/(k*t)`
formula's own well-documented `1/t` softness decay (see this file's
"Softness shrinks with distance" note in `soft-shadows-and-ao.md`,
carried over from `hybrid_legacy`'s own investigation) combined with
`k=12` (copied from `hybrid_legacy`'s default) being far too hard/small
an apparent-light-size for this renderer's actual object scale
(~0.8-2.4 units, vs. `hybrid_legacy`'s effectively-never-tested-at-range
demo).

**Real fix, validated by a k-sweep against the same ground-truth scan**
(`cpu_ref::tests::debug_k_sweep_stress_100_worst_case`): swept
`k ∈ {12, 8, 6, 4, 3, 2, 1.5, 1}` against the full `--stress 100` grid
scan and found `k<=2.0` is the largest (hardest/crispest) value with
zero false darkening; `k>=3.0` all showed real, measurable false
darkening. `ShadowConfig::default`'s `k` changed from `12.0` to `2.0`.
Separately, `shadow_candidate_margin`'s reference distance was changed
again — from ray `max_t` to a new fixed `PENUMBRA_REACH = 15.75`
constant, restoring a margin large enough to keep round (not polygonal)
silhouettes (matching `hybrid_legacy`'s own validated `≈3.78`
margin at `k=12`) without reintroducing the `--stress N` scaling bug,
since it no longer derives from anything that grows with scene size or
ray reach. The round-silhouette regression test
(`no_ring_sample_near_the_sphere_reads_fully_lit`) and the new
stress-100 ground-truth scan both pass simultaneously — the two
requirements (round silhouettes vs. no false-distance darkening) turned
out to be satisfiable together once `k` was corrected, not fundamentally
in tension as they first appeared while `k=12` was still assumed fixed.

Visually re-verified: `--stress 100`, gizmos on and off, far and near
cameras — every cube now casts a compact, correctly-localized soft
shadow with no elongated streaks or neighboring-cell bleed. Single-object
demo scenes (all 7 shapes) re-checked too — softer penumbras than before
(`k=2.0` is a much larger apparent light source than `k=12.0`) but still
correctly round and physically plausible, not washed out to full
visibility.

**Lesson for next time, worth remembering explicitly:** the first two
fix attempts were each individually well-reasoned and each individually
real bugs — but neither was validated against the actual reported
artifact before moving on, only against "does this look more correct in
one screenshot." The debug scan that actually found the root cause took
maybe 20 minutes to write and gave an exact, reproducible, numerically-
verified failing case on the first run. Build the equivalent scan (or
port `hybrid_legacy`'s GPU debug-readback machinery) BEFORE the next
shading correctness bug report, not after two failed guesses.

Verified: `cargo build/test/clippy --release` clean (55/55 tests: the 53
above plus 2 new permanent regression tests —
`debug_stress_100_full_grid_shadow_scan` and
`debug_k_sweep_stress_100_worst_case`). Not yet committed.

### Soft shadows follow-up #2: polygonal shadow edge on curved shapes (a boundary discontinuity, not a geometry bug)

Immediately after the `--stress N` false-darkening fix above, the user
caught a second, distinct artifact: a sphere's soft shadow had a visibly
polygonal/hexagonal outer edge, looking "like a shadow from a box" —
reasonably suspected as the renderer accidentally casting shadows from
AABB/BVH bounds rather than real geometry.

**Ruled out first, directly:** confirmed `object_distance`
(`cpu_ref.rs`) and `local_distance` (`hybrid_trace.wgsl`) both call the
shape's real SDF (`sd_sphere` etc.) for every march sample, not an AABB
distance — the march itself was never the problem. Isolated to sun-only
(single light) to remove any multi-light-overlap confound, and the
polygonal edge was still clearly present around the true (correctly
round) hard-shadow core.

**Root cause, confirmed by scaling the suspect constant and watching the
artifact scale with it:** `PENUMBRA_REACH` (this file's previous entry)
sets `shadow_candidate_margin`'s output, which pads every leaf's AABB
before the ray/box slab test — a point whose shadow ray falls just
outside that padded box gets ZERO candidates and `vis` stays at its
initial `1.0` with no computation at all; a point just inside gets a
real candidate and a computed `sample_vis` from the `d/(k*t)` formula.
Those two cases are NOT continuous with each other: at the padded-AABB
edge (`h ≈ margin`), `sample_vis` can still be well below `1.0`
(confirmed numerically: `≈0.16-0.63` depending on `t`, not the `≈1.0` a
point just outside gets) — a hard visibility cliff exactly at the padded
box's boundary, which renders as a visible polygonal seam wherever that
boundary happens to project onto the shaded surface. Confirmed by
temporarily quadrupling `PENUMBRA_REACH` (`15.75` → `60.0`) and observing
the polygonal edge grow proportionally — proof the edge was the margin
boundary, not the sphere's own silhouette (which doesn't change size).

**Fix:** added an explicit fade in `trace_shadow`'s per-sample
visibility — `margin_fade = smoothstep(margin * 0.5, margin, h)`,
blending `sample_vis` toward `1.0` as `h` approaches `margin`
(`faded_vis = raw_vis + (1.0 - raw_vis) * margin_fade`). This makes the
padded-AABB boundary visually seamless: a point just inside now reads
close to fully lit (matching a point just outside), while points well
inside the margin (small `h`, real proximity to the occluder) are
essentially unaffected by the fade (`margin_fade ≈ 0` there). This
discontinuity is inherent to the fixed-margin candidate-gathering
design itself (the KB doc's original formula derivation never accounted
for it) — very likely present in `hybrid_legacy` too, just never
surfaced visually there (no report of a polygonal shadow edge in that
renderer's own history).

One existing regression test's threshold relaxed (not weakened in
intent): `grazing_ray_past_its_candidate_slab_still_finds_real_occlusion`
asserted `vis < 0.5` for a point deliberately chosen right at the
margin's own edge — exactly the region the fade now smooths — relaxed
to `vis < 0.9` since the test's actual purpose (prove SOME occlusion is
found, vs. the pre-fix `vis=1.0`) still holds at the new value.

Visually re-verified: single sphere (sun-only and all 3 lights) now
shows a smooth, correctly round/elliptical soft shadow with no polygonal
edge at any tested `PENUMBRA_REACH` scale; `--stress 100` grid
re-confirmed still clean (no regression of the previous fix).

Verified: `cargo build/test/clippy --release` clean (55/55 tests, same
count as above — no new tests added, one threshold adjusted). Not yet
committed.

### Soft shadows follow-up #3: sharp "cut" on one side of a sphere's shadow at `--stress N` — internal BVH nodes weren't padded

Third artifact report in this same investigation, this time on curved
shapes specifically: a sphere's soft shadow inside a real `--stress 100`
grid showed a sharp, flat-edged cut on one side instead of tapering
smoothly, distinct from both the earlier false-darkening bug (fix #1)
and the polygonal-margin-boundary bug (fix #2, already addressed by the
`margin_fade` smoothing).

**Found via a new permanent debug test** —
`debug_stress_100_sphere_shadow_ring_profile` — reconstructing the real
`--stress 100` grid with a sphere (matching `--shape sphere`) and
sampling a dense angular ring of shadow-ray visibility around one cell's
own sphere, flagging any adjacent-angle jump bigger than a smooth
gradient would produce. Immediately found a real jump: `vis` dropping
from `1.0` to `0.33` between two rays only ~5° apart. Cross-checking the
"light" side (`vis=1.0`) directly showed `gather_candidates_padded`
found **zero candidates** for that ray, while the very next angular
sample found one immediately with a substantial `h=0.38` at its first
march step — a genuine on/off cliff in candidate existence, not a
formula continuity issue (which fix #2's `margin_fade` already handles
once a candidate exists).

**Root cause, found by manually walking the BVH ancestor chain for the
missing candidate:** `gather_candidates_padded` only padded LEAF node
AABBs by `margin`, leaving internal nodes at their tight (unpadded)
bounds — deliberate at the time ("internal nodes stay tight so
traversal culling stays effective"), but a real bug: an internal node's
bounds are the TIGHT union of its children's own TIGHT bounds, with no
knowledge that a child leaf's own bounds get padded before the ray test.
A ray whose true path only entered a leaf's *padded* margin region
(missing that leaf's own tight box) could ALSO miss the leaf's *parent*
internal node's unpadded tight box — pruning the whole subtree before
the leaf's own (correctly padded) test ever ran. Confirmed with a real
trace: the failing ray's target leaf's own padded slab test would have
succeeded (`near=0.0, far=2.55`), but its direct parent internal node's
unpadded test failed (`near=0.94 > far=0.65`), silently dropping the
leaf from traversal.

**Fix:** pad every node uniformly by `margin` during descent — both
internal and leaf. This is correct, not just a workaround: each
individual leaf's own padding is `≤ margin`, so an internal node's tight
bounds expanded by `margin` still fully contains every descendant leaf's
own padded bounds; culling efficiency during the (much more common)
primary-ray `trace()` traversal is unaffected since this fix only
touches `gather_candidates_padded`'s own shadow-ray-only descent.

Visually re-verified: `--stress 100` with `--shape sphere`, multiple
camera angles including a close-up on a single sphere — every shadow now
shows a smooth, correctly round/elliptical penumbra on all sides, no
flat cuts.

Verified: `cargo build/test/clippy --release` clean (56/56 tests — one
new permanent regression test, `debug_stress_100_sphere_shadow_ring_
profile`, added). Not yet committed.

### Soft shadows follow-up #4: gotchas from this debugging arc written into the knowledge base

Once the three artifacts above were resolved and visually confirmed, wrote
up the durable lessons (not just this file's chronological narrative) in
three places so they're discoverable outside this session's own history:

- `docs/knowledge/sdf-3d/rendering/soft-shadows-and-ao.md` — new "Porting
  to `src/hybrid`" subsection documenting all three bugs in detail (fixed
  reference-distance-not-scene-extent, `k` must be re-validated at target
  object scale not copied from source renderer, BVH padding must cover
  internal nodes), plus the margin-boundary continuity fix (`margin_fade`)
  as a fourth, related-but-distinct issue, plus the methodological lesson
  (build the ground-truth CPU scan test *before* attempting a fix, not
  after a screenshot-guided guess fails) and the still-open GPU
  debug-readback tooling gap (`hybrid_legacy::debug.rs` has no
  `src/hybrid` equivalent yet).
- `docs/knowledge/hierarchical-volumes/bvh-deep-dive.md` — new "Padded/
  expanded queries" section generalizing the internal-node-padding bug
  beyond shadows specifically (relevant to Stage C's future occupancy-grid
  secondary-ray work), cross-linked back to the full incident writeup.
- `docs/knowledge/sdf-3d/rendering/raymarching-artifacts-and-fixes.md` —
  two new symptom-to-cause entries ("false shadow darkening that only
  appears at large scene/object counts", "sharp on/off cut in an
  otherwise-smooth soft shadow silhouette"), the latter explicitly
  distinguished from the pre-existing similarly-named "flat cut… near
  where two objects touch" entry (different subsystem: shadow BVH padding
  vs. primary-visibility merged-interval-list clipping — same-sounding
  symptom, unrelated root cause).

Not yet committed.

### Stage C: occupancy-grid shadow-ray accelerator — negative result (correct, safe, no measurable speedup)

GI trajectory reordered on explicit user request: Stage C (secondary-ray
acceleration) before Stage B, with an explicit go/no-go checkpoint before
Stage B may begin. This entry is that checkpoint.

**Scope, deliberately smaller than the KB blueprint.**
`docs/knowledge/hierarchical-volumes/occupancy-first-pass-design.md`/
`hierarchical-grids-and-trees.md` describe a 3-level, hash-rooted,
DAG-deduplicated hierarchy for an unbounded/tiled world `src/hybrid`
doesn't have. Built instead: a single-level, bounded occupancy grid
(`src/hybrid/occupancy.rs`) over the BVH's current root AABB, rebuilt
only on a structural BVH rebuild (new `bvh::BvhUpdateResult` signal from
`Bvh::update`, threaded through a new `PersistentOccupancyGrid` resource
— mirrors `PersistentBvh`'s own refit-vs-rebuild cadence exactly).

**First design (point-neighborhood gate) — measured ineffective, redesigned
before shipping.** The first cut computed, once per pixel, "is the shading
point's own neighborhood (within `shadow_candidate_margin`'s reach)
empty?" — if so, every light skipped `trace_shadow` entirely. CPU-ref
tests passed; visual re-verification passed; but real GPU measurement at
`--stress 10000` (all three camera framings) showed **no speedup, and
occasionally a slight slowdown**. Root cause, found before committing to
a bad result: a shading point's own local emptiness says nothing about
whether its shadow ray's PATH (toward a possibly-distant light) later
passes through occupied space — the gate answered the wrong question.
`gather_candidates_padded`'s existing BVH descent already rejects a ray
missing the object grid entirely at its very first (root) slab test, so
the point gate and the existing fast-reject path were doing the same
cheap work for the same easy rays, while never helping the hard
(near-geometry) rays at all.

**Second design (ray-aware DDA gate) — the one that shipped.**
`OccupancyGrid::ray_is_definitely_unoccluded` walks the grid cells a
shadow ray actually passes through via the standard Amanatides-Woo 3D-DDA
algorithm (the same technique this project's own KB already names for
ray queries against a grid), from a real grid-entry slab test through
`t_max`, returning "unoccluded" only if every touched cell is empty.
Necessarily evaluated per-light now (each light's ray direction differs),
not once per pixel. `assets/shaders/hybrid_trace.wgsl` mirrors this
exactly. CPU-ref regression test
(`gate_never_reports_empty_where_real_shadow_math_finds_occlusion`)
re-targeted at the ray function specifically — proves the gate can never
wrongly report "unoccluded" for a ray the existing, validated
`trace_shadow` finds actually shadowed.

**A real, shipped correctness bug found and fixed during this stage's own
sweep testing** (not found by any test until a targeted repro was built):
`OccupancyGrid::build`'s `MAX_GRID_CELLS` pathological-input fallback
(triggers when `cell_size` is small enough relative to scene extent to
overflow the cell-count cap) was documented as "always non-empty" (the
safe direction) but shipped as `cells: vec![false]` — `false` means "not
occupied," i.e. exactly backwards: every gate query against that
fallback silently reported "definitely unoccluded," which would have
skipped shadow tracing for the ENTIRE scene whenever it triggered. Found
via a cell-size sweep experiment (testing whether a coarser/finer grid
changed the measured performance) — a deliberately fine test multiplier
triggered the fallback and produced a suspiciously large "speedup" that
turned out to be silently-broken shadows, not a real win. Fixed two ways:
(1) the fallback's boolean was flipped to `true`; (2) a new explicit
`always_occupied: bool` field was added to `OccupancyGrid`, checked
FIRST and unconditionally in both query functions, mirrored into
`SceneUniform::occupancy_always_occupied` in WGSL too — this is more
robust than relying on a synthetic 1-cell grid's own geometry to always
resolve "occupied," since a ray query's own grid-entry slab test could
(and did) find a way to skip past that tiny synthetic extent without
ever consulting a cell at all. New permanent regression test
(`max_grid_cells_overflow_fallback_is_conservative_not_permissive`)
exercises this exact path against both query functions. Lesson: an
"always non-empty"/"fail-safe" fallback needs its own correctness test
under the specific condition that triggers it — a doc comment asserting
the safe direction is not evidence the code actually implements it.

**Escape hatch and cell-size sweep, both real, both negative.**
`ShadowConfig::occupancy_gate_enabled` (`--occupancy-gate on|off`, egui
checkbox) forces the gate off entirely, falling through to the exact
pre-Stage-C path — used to A/B against the recorded pre-Stage-C baseline.
A cell-size multiplier sweep (1x/3x/6x/12x/20x the base
`shadow_candidate_margin`) tested whether a coarser grid amortizes the
DDA walk's fixed per-ray setup cost.

**Real measured numbers, `--stress 10000`, box shape, gizmos off, warmup
run discarded, 3-5 repeats per configuration (this machine is a shared,
actively-loaded workstation — Chrome/Firefox/other sessions running
concurrently, `load average ~5` on an 8-core AMD Ryzen 7 4700U with
integrated RADV RENOIR — single unwarmed samples showed 20-40% spurious
variance that repeated sampling resolved):**

```
near camera, gate off:  32.52 / 32.53 / 32.68 ms  (mean ~32.6ms)
near camera, gate on:   32.65 / 32.46 / 32.46 ms  (mean ~32.5ms)
far  camera, gate off:  29.14 / 29.00 / 29.01 ms  (mean ~29.05ms)
far  camera, gate on:   38.69 / 37.07 / 29.48 / 29.44 / 29.22 ms
                        (first 2 reps: cold/warmup noise; steady-state
                        3/5 reps converge to ~29.4ms, matching gate-off)

cell-size multiplier sweep (near camera, gate on, warmup discarded):
  1x  (buggy fallback path — excluded, see correctness bug above)
  3x  32.83-33.03ms   6x  32.98-33.01ms   12x 32.81-32.83ms   20x 32.97-33.03ms
  (all statistically indistinguishable from the 3x default and from
  gate-off; no multiplier tested produces a measurable win)
```

**Verdict: the gate is correct, safe (regression-tested, escape-hatched,
CPU/WGSL cross-validated, visually re-verified across all 7 shapes), and
measurably a no-op at this renderer's current scale.** Neither camera
framing nor any tested cell size shows a real speedup. Root cause,
inferred from the numbers rather than assumed: `gather_candidates_padded`
already rejects genuinely-empty rays cheaply at the BVH's root node (a
single slab test), so the coarse grid's DDA walk is paying its own real
per-ray setup/traversal cost to answer a question the existing structure
was already answering almost as cheaply for the cases where the answer
is "empty" — and for the cases where the answer is "occupied" (most
near-camera rays, per the CPU-side ray-unoccluded-fraction measurement
of ~33.7% at this scale, meaning ~66% of near-camera shadow rays DO pass
near real geometry), the gate adds cost without ever paying off, since
it still falls through to the full exact path. The technique's
structural ceiling — "skip the BVH descent only in provably empty
regions" — turns out not to be where this renderer's actual shadow cost
lives; the cost is concentrated in the exact candidate-gather-plus-march
work for rays that DO find real occluders, which this stage's gate was
never designed to reduce (see this stage's own design-scope note above).

**Go/no-go outcome: NO-GO on this specific technique at this scale.**
Per the approved plan's explicit checkpoint, this negative result was
reported to the user before Stage B began. **Decision: revert the
mechanism.** Correct and safe as it was, it added real surface area
(a new module, a new GPU buffer/binding, extra `SceneUniform` fields, a
per-light gate branch, CLI/egui controls) for zero measured benefit —
not worth carrying forward on complexity-cost alone. `src/hybrid/
occupancy.rs` was deleted; `bvh.rs`/`extract.rs`/`pipeline.rs`/
`hybrid_trace.wgsl`/`examples/gallery.rs`/`cpu_ref.rs`'s test-visibility
changes were all reverted to their pre-Stage-C state. What was kept: this
writeup (the negative result and its two real found-and-fixed bugs are
durable, reusable knowledge even though the mechanism itself didn't ship
— see also the general "fail-safe fallback needs its own test" lesson
added to `docs/knowledge/hierarchical-volumes/bvh-deep-dive.md`), and the
trivial, genuinely-correct `PENUMBRA_REACH` doc-comment fix (`1.5` was
stale relative to the real `15.75`, found while deriving this stage's
now-reverted cell-size logic — the fix itself has nothing to do with the
reverted mechanism and stands on its own).

Verified pre-revert: `cargo build/test/clippy --release --lib --examples`
clean (67/67 tests), visual re-verification across all 7 shapes showed
correct, unchanged soft shadows. Post-revert: back to 56/56 tests
(the pre-Stage-C count), clean build/clippy, `git diff --stat` confirms
only this writeup, the KB lesson, and the doc fix remain. GI trajectory
returns to Stage B next.

### Stage B: jittered single-bounce indirect diffuse — shipped, real cost measured

GI trajectory step B. Technique chosen after real research (not a default
port): an *upgraded* (jittered) version of `hybrid_legacy`'s already-shipped
hemisphere-sample-direction indirect diffuse — not a DDGI-style probe grid
(needs temporal accumulation this renderer has none of; building that now
would repeat Stage C's mistake of new infrastructure with unverified
payoff) and not Radiance Cascades (deep-dived at the user's explicit
request: no working 3D reference implementation exists anywhere as of the
most recent research — one public 3D world-space attempt is unrunnable,
and a 2026 preprint states 3D world-space RC was an *unsolved* problem
until essentially now; this would be research, not engineering, today).

**Ported from `hybrid_legacy`'s `indirect_diffuse()`
(`assets/shaders/hybrid_legacy_trace.wgsl:1037-1131`) — its first CPU
reference ever** (it shipped there WGSL-only, outside this project's
CPU-ref-first convention). Two deliberate deviations from the raw port:

1. **Reuses `cpu_ref::trace()` directly** for the short hemisphere probe
   instead of `hybrid_legacy`'s brute-force all-objects scan
   (`map_at_obj` with its own manual step loop) — `trace()` is already
   BVH-accelerated and `t_max`-bounded, so calling it once IS the short
   probe; no parallel query path was built.
2. **Jittered, not fixed, sample directions.** `hybrid_legacy`'s own
   comment excuses its fixed 5-direction set's faceting because "the
   existing dither pass already breaks it up" — `src/hybrid` has no
   dither pass, no G-buffer, no temporal accumulation of any kind, so
   that excuse doesn't transfer. A deterministic per-pixel hash
   (`pixel_jitter_angle`, seeded by pixel coordinate only — NOT frame
   index, since a per-frame jitter with no temporal accumulation to
   average across would show as flicker, not reduced banding) rotates
   the 4 outer sample directions around the normal each pixel.

**A real bug found and fixed after the first WGSL build, before any
benchmark was trusted.** The first working build visually flooded every
surface with magenta. Root cause: `indirect_ray`'s miss case returned
`extract::BACKGROUND_COLOR` at full, undimmed strength — `hybrid_legacy`'s
original design returned a real sky gradient on a miss (a physically
plausible ambient fill), but `src/hybrid`'s `BACKGROUND_COLOR` is a
deliberate magenta "something is wrong" debug flag for PRIMARY-ray
misses, never meant to be blended into real lighting. Since most
hemisphere probes on open ground point at open sky (a miss is the
*common* case, not the rare debug case), this meant most surfaces were
dominated by full-strength magenta. Fixed by adding `sky_color(ray_dir)`
— a mocked "clear sky" horizon/zenith gradient, verbatim-ported from
`hybrid_legacy`'s own `sky_color` — used ONLY for indirect-ray misses;
`BACKGROUND_COLOR` itself is unchanged and still exactly what a
primary-ray miss reports. Deliberately kept as its own function (not
inlined) so a future real skybox/skysphere only needs to replace this
one function's body — no call-site changes needed when that happens.

**`INDIRECT_MAX_T = 3.0` (identical to `hybrid_legacy`'s own value) was
checked, not assumed, safe against this renderer's `--stress N` grid
scale** — the same class of bug `DIRECTIONAL_SHADOW_MAX_T` hit when
`hybrid_legacy`'s own `60.0` turned out to reach into neighboring grid
cells during Stage A. `3.0` is comfortably under the 11-unit cell pitch
(unlike `60.0`, which was ~5.5x it), though the ground-footprint gap
between adjacent cells is numerically exactly `3.0` units too — a new
regression test
(`indirect_probe_reach_does_not_cross_into_neighboring_grid_cells`)
checks a probe right at a cell's own boundary directly rather than
assuming the coincidence is harmless; confirmed no cross-cell bleeding,
both numerically and in a `--stress 4` visual check.

**Pre-WGSL cost-ceiling check, mirroring Stage C's own methodology.** A
`debug_stress_10000_indirect_diffuse_march_step_profile` test (run before
any WGSL was written) found the real cost profile is cleanly bimodal at
this scene's scale: sample points on any `--stress N` ground plate hit
real geometry 100% of the time (every shaded surface sits on some cell's
own ground, so "is there nearby geometry to bounce off of" is close to
always true by construction); sample points 5 units above the whole grid
(genuinely open air, beyond any object's `INDIRECT_MAX_T` reach) hit 0%
of the time. This differs from Stage C's own near/far camera split — an
indirect-diffuse ray's cost doesn't depend on where the *camera* is, only
on whether the *shaded surface point* has nearby geometry, which in this
scene is essentially always yes.

**CPU reference**: 12 new tests in `src/hybrid/cpu_ref.rs` (escape-to-sky,
color pickup with hue matching, monotonic falloff, self-intersection bias,
origin-entity exclusion, jitter-basis orthonormality/determinism/polar-
sample-invariance, the kill-switch bit-exactness check, the grid-boundary
check, and the cost-ceiling debug test), all passing before any WGSL was
touched.

**Kill switch**: `IndirectDiffuseConfig` (`src/hybrid/extract.rs`),
mirroring `ShadowConfig`'s exact shape — `--indirect on|off` /
`--indirect-max-t F` CLI flags, egui "Indirect diffuse" panel section.
`sample_count` is exposed as a config field but NOT wired end-to-end this
stage (the WGSL sample loop stays hardcoded to 5 — a runtime-variable
count would need on-the-fly direction generation or multiple pre-baked
tables, real scope beyond this stage's bar); `max_t` is fully live-tunable.

**Visual re-verification**: all 7 shapes, `--stress 1`, indirect on vs.
off — visible warm color bleed near each shape's own base, cool sky-tint
ambient fill elsewhere, correct silhouettes (round/hexagonal/etc.)
preserved, no artifacts. `--stress 4` cross-cell check: no visible
bleeding between unrelated grid cells.

**Real GPU numbers, `--stress 10000`, box shape, gizmos off, warmup run
discarded, 3-5 repeats per configuration (shared/loaded workstation —
see Stage C's own note on why single samples are unreliable here):**

```
near camera:
  indirect off: 32.63 / 32.65 / 32.67 ms  (mean 32.65ms, tight)
  indirect on:  60.66 / 60.86 / 61.08 ms  (mean 60.87ms, tight)
  delta: +28.2ms (+86%)

far/grid camera:
  indirect off: 16.86 / 18.76 / 20.38 / 21.56 / 23.53 ms  (mean 20.22ms,
                real ~6.7ms spread — this camera framing's own baseline
                noise, not an indirect-diffuse artifact)
  indirect on:  44.86 / 45.28 / 45.29 ms  (mean 45.14ms, tight)
  delta: +24.9ms (+123%)
```

**Verdict: real, substantial, reproducible cost — roughly doubling frame
time at this scale, not a "nearly free" effect.** Unlike Stage C's own
negative result, this is a real positive feature with a real, honest
price tag: 5 unconditional short `trace()` calls per shaded pixel (no
per-light early-out the way shadow rays get — every pixel pays for all 5
samples whenever the feature is enabled, regardless of scene content).
The kill switch's off-path measures within noise of the pre-Stage-B
baseline in both camera framings, confirming "costs nothing when
disabled" holds. Whether this cost is acceptable for a given use case is
a design tradeoff, not a bug — flagged honestly rather than either
downplayed or treated as disqualifying; `max_t`/sample-count remain the
tuning knobs for a cheaper variant if a future stage needs one.

Verified: `cargo build/test/clippy --release --lib --examples` clean
(70/70 tests, no new warnings). Not yet committed.

### Stage B follow-up: IGN jitter + edge-aware spatial denoise for indirect diffuse

Stage B's own shipped screenshots showed visible grain on curved/shaded
surfaces once indirect diffuse landed — confirmed by direct close-up A/B
comparison (indirect off: smooth everywhere including shadows; indirect
on: grainy sphere/ground surfaces, shadows themselves unaffected). Root
cause: `pixel_jitter_angle`'s original raw sine-hash jitter
(`sin(x*12.9898+y*78.233)*43758.545`) has zero spatial correlation
between neighboring pixels — pure white noise, with nothing (no blur, no
temporal accumulation) downstream to average it out.

**Researched direction before implementing anything**, per the user's
explicit question ("What is better direction if we are moving toward GI,
temporal accumulation? blur pass? Or both? Or something else?"):
industry-standard options are SVGF/ReSTIR-style temporal accumulation,
same-frame spatial denoising, or a better noise source, or some
combination. **Decision: same-frame spatial denoise + a better noise
source now; temporal accumulation deferred.** Rationale: this renderer
has no motion-vector infrastructure at all (no per-object previous-frame
transform threading, needed for correctness against the spinning-cube
demo's own object motion, not just camera motion) — building that
purely to denoise Stage B's already-fixed 5-sample estimate, without
also using the same infrastructure to *reduce* Stage B's own per-frame
ray count (the way real temporal reuse in DDGI/ReSTIR pays for itself),
would front-load real new infrastructure for half the payoff Stage C
already showed the risk of doing that (new complexity, unproven win).
Deferred to a later GI stage where temporal amortization is the point,
not just a side benefit.

**Two same-frame changes, both approved by the user:**

1. **Interleaved gradient noise (Jimenez, "Next Generation Post
   Processing in Call of Duty: Advanced Warfare")** replaces the raw
   sine hash in `cpu_ref.rs::pixel_jitter_angle` (and its WGSL mirror):
   `frac(52.9829189 * frac(dot(pixel, vec2(0.06711056, 0.00583715))))`
   mapped to `[0, 2*PI)`. Same cost (one hash, no new binding/texture),
   different spatial distribution — the standard noise source production
   SSAO/GI implementations use because its error pattern is far less
   objectionable than white noise even before any denoising.
2. **A small fixed-radius (5x5, `BLUR_RADIUS=2`) edge-aware bilateral
   blur**, applied ONLY to the indirect-diffuse channel — confirmed with
   the user this must be a separate channel, not a blur of the whole
   composited image (which would also soften real, non-noisy detail:
   specular highlights, crisp shadow edges). Weight formula: Gaussian-ish
   falloff on normal-dot-product (`BLUR_NORMAL_SIGMA=0.1`) times
   falloff on depth difference relative to the center pixel's own depth
   (`BLUR_DEPTH_SIGMA=0.05`) — a neighbor must be both similarly-facing
   AND similarly-deep to contribute meaningful weight, so the blur
   smooths a noisy flat/curved region but refuses to bleed across a real
   edge (object silhouette, hard corner).

**`shade()` split into `ShadeResult { direct_and_emissive, indirect }`**
(both `cpu_ref.rs` and `hybrid_trace.wgsl`, structurally identical) so
the indirect term can be isolated for blurring — `indirect` is stored
already diffuse-albedo-multiplied (final color, not raw irradiance):
checked against the plan's own three-way tradeoff (pre-multiply vs. a
separate albedo texture vs. re-deriving albedo from an object-id lookup)
and confirmed the simplest option loses nothing here, since every
material in this renderer is a flat, textureless color — there is no
fine albedo detail a pre-multiply blur could smear that direct sampling
would have protected. This also meant no 4th texture and no blit-shader
changes were needed at all: recombination happens by simply adding
`direct_and_emissive + blurred_indirect` in the same denoise compute
pass, writing straight into a new dedicated `denoised_color_view`
texture the blit pass reads in place of the trace pass's own raw
`color_view`.

**New pipeline shape**: `hybrid_trace.wgsl` now writes 4 storage
textures (`out_color`, `out_depth`, new `normal_view` — world-space
shading normal, `rgba32float`, mirroring `src/prepass_probe`'s own
existing normal-texture precedent — and new `indirect_view`,
`rgba16float`, pre-blur indirect color). A new `hybrid_denoise.wgsl
@compute` pass (`denoise_main`, same 8x8 workgroup convention as
`trace_main`) runs between the trace pass and the blit pass in
`hybrid_pass` (`src/hybrid/pass.rs`), same command encoder, same
`diagnostics.pass_span` GPU-timestamp convention as the other two passes
— reads `indirect_view`/`normal_view`/`out_depth`/`out_color` as plain
sampled `texture_2d` (WGPU disallows binding one storage texture as both
write-only in one pass and read in another within the same bind group,
and there is no `read_write` storage-texture precedent anywhere in this
codebase — confirmed, not assumed, before choosing this shape) and
writes the recombined `denoised_color_view`.

**`DenoiseConfig { enabled: bool }`** (`src/hybrid/extract.rs`), mirrors
`ShadowConfig`/`IndirectDiffuseConfig`'s exact shape. Unlike those two,
"disabled" does NOT skip the pass's dispatch — the pass always runs
(cheap, small, viewport-sized like every other pass here) but internally
skips the 25-tap weight loop and copies `indirect_view` straight through
unblurred; the alternative (skip the dispatch entirely) would need the
blit path to conditionally read `indirect_view` vs. `denoised_color_view`
depending on this same flag, which is more branching than one cheap
copy-pass costs — a deliberate, documented deviation from
`ShadowConfig`/`IndirectDiffuseConfig`'s "costs nothing, doesn't just
contribute nothing" pattern, justified by this pass's much smaller
absolute cost (single-digit ms, not the multi-ms-per-light cost a
skipped shadow ray saves). `--denoise on|off` CLI flag, egui checkbox in
the existing "Indirect diffuse" panel section (gated on indirect diffuse
itself being on, same `ui.add_enabled` pattern `shadow_config.k`'s
slider already uses).

**CPU reference** (`cpu_ref.rs::blur_indirect_at`, `BlurSample` struct):
5 new tests — a flat noise-free region blurs to itself (no-op); a hard
normal discontinuity is not crossed; a hard depth discontinuity
(silhouette) is not crossed even when normals agree; a synthetic noisy
flat region's variance drops by >50% after blurring; and a recombination
regression test confirming `direct_and_emissive + blur_indirect_at(...)`
with an identity-weight blur reproduces pre-split `shade()`'s original
single-`Vec3` sum exactly. All 5 written and passing before any WGSL was
touched, per this project's CPU-ref-first convention.

**Visual re-verification**: sphere close-up, indirect on both times,
`--denoise off` vs `--denoise on` — grain from Stage B's own shipped
screenshots is visibly gone with denoise on; the specular highlight and
the ground shadow's edge both stay exactly as sharp, confirming the
blur is correctly confined to the indirect channel and isn't bleeding
into direct-lit detail.

**GPU cost measurement — honest caveat first: this machine was under
unusually heavy load during this measurement** (`uptime` showed load
average ~9.2 on an 8-core machine, well above Stage B's own benchmark
session — several Firefox windows, another Claude session, and opencode
all active simultaneously), which made `gpu trace`'s own absolute
numbers (58-130ms, vs. Stage B's own ~10-19ms at the identical
`--stress 10000`/near-camera/box-shape config) untrustworthy for a
before/after comparison this session. **The denoise pass's own span,
however, is a small, self-contained cost mostly insulated from that
contention** (it doesn't depend on scene occlusion/marching the way
`hybrid_trace`'s cost does) and shows a clear, consistent signal across
4 configurations (`--stress 10000`, box shape, near+far camera, warmup
run discarded, 11-12 repeats each):

```
denoise pass span only:
  near, denoise off (copy-through): mean 2.02ms
  near, denoise on  (25-tap blur):  mean 5.35ms
  far,  denoise off (copy-through): mean 2.09ms
  far,  denoise on  (25-tap blur):  mean 4.56ms
```

**Verdict: the denoise pass itself adds roughly +2.5-3.4ms** on top of
its own ~2ms copy-through baseline (which is itself the fixed cost of
dispatching a full-viewport compute pass regardless of what it does
inside) — small in absolute terms and, unlike Stage B's own indirect-
diffuse cost (5 unconditional `trace()` calls per pixel, real BVH
traversal work), independent of scene complexity: this pass touches a
fixed 25 texels per pixel regardless of how many objects or how deep the
BVH is. A clean-system re-measurement of the FULL before/after picture
(trace+denoise+blit vs. trace+blit alone) is flagged as still owed once
the workstation isn't this loaded — reported honestly as not done here
rather than presenting a misleading absolute total built on contended
numbers.

Verified: `cargo build/test/clippy --release --lib --examples` clean
(75/75 tests, no new warnings). Not yet committed.

### Temporal accumulation for indirect diffuse — the real architectural fix for the banding

The IGN-jitter + spatial-blur work just above (previous entry) removed the
grain, but exposed a SEPARATE, real, pre-existing artifact underneath it:
visible banding on curved surfaces (a sphere close-up) — confirmed via
direct screenshot A/B that the same faint band pattern was present even
with the new spatial blur turned off, just previously hidden under grain.
Root cause: Stage B's indirect diffuse evaluates only 5 fixed hemisphere
samples per pixel per frame — an estimator too coarse to be smooth on its
own; jitter/blur were both ways to disguise that undersampling, not
reduce it.

**User's explicit call, considered against two cheaper alternatives**
(more samples/pixel — real linear cost, doesn't scale; a bigger spatial
blur — cheap but only hides the artifact, risks over-smoothing): build
real temporal accumulation now, not deferred to a later stage as this
same session had earlier planned. This directly supersedes that earlier
deferral — recorded in memory
(`hybrid_gi_temporal_accumulation_decision.md`) since it's a real
reversal of a stated plan, not a quiet scope change.

**Scope decision: camera + object motion from day one, not camera-only
first.** This renderer's own demo continuously spins its showcase object
(`Spinning`/`spin_objects` in `examples/gallery.rs`) — object motion is
the steady-state condition of the actual demo, not an edge case, so a
camera-only reprojection would have reintroduced a different artifact
(stale/smeared shading on the spinning object) in the demo's own
headline case.

**Architecture**: three GPU passes now run per frame (trace → temporal
accumulate → denoise → blit, was trace → denoise → blit):
- `hybrid_trace.wgsl` gained a `motion_view` (`rgba32float`) output:
  each hit pixel's world position, reprojected through that OBJECT's own
  previous-frame rigid transform (undo current, reapply previous — new
  `prev_translation_*`/`prev_inv_rotation_*` fields on `ObjectGpu`, fed
  by a new `src/hybrid/motion.rs` — `PreviousShapeTransform` component +
  a `PreUpdate` system mirroring `bevy_pbr::prepass::
  update_mesh_previous_global_transforms`'s exact shape, since that
  built-in is hard-filtered to `With<Mesh3d>` and inert for this
  renderer's `Shape` entities). Stored as a world position, not yet a
  screen UV: `trace_main` has no previous-camera matrices bound.
- New `hybrid_temporal.wgsl` finishes the reprojection (world position ->
  previous-frame screen UV, via `bevy_pbr::prepass::PreviousViewData`/
  `PreviousViewUniforms` — Bevy's own last-frame camera matrices,
  populated automatically the moment any `Material`/`PbrPlugin` loads;
  confirmed active in this app, no extraction of our own needed, same
  "camera/view data needs no extraction" convention `extract.rs`'s
  module doc comment already establishes for the CURRENT-frame `View`
  uniform), applies a four-part SVGF-style disocclusion/rejection test
  (off-screen UV; relative depth discontinuity; normal discontinuity —
  reusing `blur_indirect_at`'s existing weight formulas as hard
  accept/reject cutoffs rather than inventing new ones), and blends via
  a clamped EMA (`alpha = 1/min(history_length+1, max_history_length)`,
  the standard SVGF/TAA running-average shape, not a fixed-alpha EMA).
  Writes `accumulated_indirect_view` (read next by `hybrid_denoise.wgsl`
  in `indirect_view`'s place — the existing spatial blur pass's OWN
  logic is completely unchanged, just rebinds its input) plus a
  ping-ponged (A/B) history buffer (`color`/`length`/`depth`/`normal`
  per slot) — ping-pong is required, not just convenient: reprojection
  reads history at a DIFFERENT UV than the pass writes at, so an
  in-place single-texture read+write would race across invocations under
  WGPU's unordered execution model (same reasoning already established
  for why `denoised_color_view` couldn't be `out_color` itself).
- `TemporalConfig { enabled, max_history_length }` mirrors
  `DenoiseConfig`'s exact shape — `--no-temporal`/
  `--temporal-max-history F` CLI flags, egui checkbox+slider gated under
  indirect diffuse. Disabled path costs one cheap copy-through pass
  (indirect_view -> accumulated_indirect_view unblended, history reset to
  length 0), not a skipped dispatch — same "costs a cheap copy, not a
  conditional dispatch skip" reasoning as `denoise_enabled`'s own
  precedent, needed here because skipping the dispatch would need
  `hybrid_denoise.wgsl` to conditionally rebind its input.

**CPU reference first (`src/hybrid/temporal_ref.rs`, a new file — kept
separate from `cpu_ref.rs`, which was already ~2900 lines and is a
genuinely separate concern: cpu_ref.rs is about what a pixel sees THIS
frame, temporal_ref.rs is about combining it with LAST frame)**: 17 new
tests before any WGSL was touched — `reproject_world_point` (static
object identity, exact translation-delta shift, rotation matching an
independently-computed rotation, sky-pixel passthrough),
`world_to_previous_uv` (behind-camera rejection, dead-center ->
UV(0.5,0.5), a translated-camera sign-convention check that pinned the
UV-shift direction down in Rust before it could become a hard-to-debug
WGSL visual bug), `disocclusion_rejected` (accept/reject on each of the
four conditions independently, plus an exact-boundary case pinned to the
accept side), `temporal_blend` (zero-history passthrough, cap
never-exceeded, convergence from a deliberately-wrong starting history
toward the true value over repeated blends), and a composed two-frame
integration test proving accumulation actually reduces error toward a
known "true" value versus either single frame's raw noisy estimate alone
— the test that validates the FEATURE, not just each primitive.
`src/hybrid/motion.rs` added 3 more tests (freshly-spawned shape gets a
same-frame previous transform, a moved shape's previous transform lags
exactly one frame behind due to `PreUpdate` running before Bevy's own
`PostUpdate` transform-propagation, a static shape never updates) —
these caught and confirmed the exact `PreUpdate`/`PostUpdate` schedule
ordering the whole design depends on, not assumed.

**Visual re-verification, direct A/B at the exact camera angle that
originally showed the banding** (`--shape sphere --camera-pos 0,1.5,1.9`,
static camera and object): `--no-temporal` reproduces the banding
exactly (confirming the repro is real and the flag correctly disables
the feature); default (temporal on) shows a fully smooth gradient across
the whole sphere, banding completely gone, specular highlight and ground
shadow edge both still crisp. Camera-orbit + object-spin (the actual
demo's steady-state condition) shows no visible smearing/ghosting around
the spinning object, `history_length` reaching its configured cap
(confirming disocclusion rejection isn't pathologically discarding every
frame's history under real motion).

**GPU cost — same heavy-contention caveat as the previous entry applies
to absolute `gpu trace` numbers** (this integrated GPU shares
capacity with the desktop compositor/other GPU clients in a way
`uptime`'s CPU load average doesn't track, confirmed by seeing similarly
elevated `gpu trace` readings — 58-75ms vs. this renderer's own
much-lower earlier baselines — even after CPU load average dropped
substantially between measurement sessions). The temporal pass's own
span, like the denoise pass's before it, is small and self-contained
enough to read meaningfully anyway (`--stress 10000`, box shape, near
camera, warmup discarded):

```
hybrid_temporal pass span only:
  temporal off (copy-through): mean 4.48ms (n=12)
  temporal on  (reproject+blend): mean 7.48ms (n=5)
```

**Verdict: the real reprojection+blend logic adds roughly +3ms** over its
own ~4.5ms copy-through baseline (the fixed cost of dispatching a
full-viewport compute pass with 10 texture bindings, regardless of what
it does inside) — in the same small-and-resolution-bound cost class as
the spatial blur pass, not scene-complexity-bound. A full clean-system
before/after of the whole pipeline remains owed for the same reason as
the previous entry — flagged honestly, not guessed at.

Reducing `INDIRECT_SAMPLE_COUNT` (the other half of temporal
accumulation's real payoff — using the now-available cross-frame
amortization to cut Stage B's own per-frame ray cost, not just clean up
its output) is an explicit, real follow-up, out of this work's own
scope.

Verified: `cargo build/test/clippy --release --lib --examples` clean
(95/95 tests, no new warnings). Not yet committed.

### Temporal accumulation follow-up: adaptive spatial-blur strength

With both the spatial blur and temporal accumulation now shipped, the
question was whether both are still needed at full strength together.
Answer: yes for correctness (the spatial blur is what covers pixels with
no/rejected temporal history — first frame, disocclusion — the same job
it always had), but no for cost: once a pixel's temporal history has
converged, the accumulated signal is already low-noise, and blurring it
further trades away sharpness for noise reduction it no longer needs.

**Fix: blur strength now fades with `history_length`**, not a fixed
weight applied everywhere. `cpu_ref::blur_strength(history_length,
max_history_length)`: 1.0 (full blur, unchanged behavior) at
`history_length <= 1` (brand-new/just-rejected pixel — still just one
frame's raw estimate), linearly fading to 0.0 once `history_length`
reaches the cap. New `cpu_ref::adaptive_blur_indirect_at` wraps the
existing (unchanged) `blur_indirect_at` with this fade via a simple
`lerp` — 5 new tests: full strength at `history_length<=1` matches
`blur_indirect_at` exactly, zero strength at the cap matches the
unblurred center value exactly, strength is monotonically non-increasing
across the whole range, and a mid-convergence value lies strictly
between both endpoints.

**Wiring**: `hybrid_denoise.wgsl` gained a new `history_length_tex`
binding — specifically THIS FRAME's own freshly-written ping-pong slot
from `hybrid_temporal.wgsl` (not the previous frame's read slot), since
the blur needs to know how converged the value it's about to blur
already is. This meant moving `HybridDenoiseBindGroup`'s construction
out of `prepare_hybrid_scene` (which only rebuilds on resize/buffer
growth) into `prepare_hybrid_temporal` (which already rebuilds every
frame for ping-pong parity, and already has `write_slot` in scope) — the
denoise bind group now shares the same "must rebuild every frame"
constraint the temporal bind groups already had, for the same reason.
`denoise_main` also skips the 25-tap weight loop entirely once a pixel's
strength rounds to zero, not just multiplies a computed-anyway blur by
zero — same "costs nothing when there's nothing to do" principle already
applied per-frame to `denoise_enabled`/`temporal_enabled`, here applied
per-pixel instead.

**Verified working, not just wired**: at the original banding-repro
camera angle with history fully converged (`history_length` at its
24-frame cap), the denoise pass's own GPU cost dropped from ~3.7-4.2ms
(previous, fixed-strength behavior) to a measured **1.91ms** — direct
confirmation the per-pixel early-out is actually triggering, not just
present in the code. The sphere still renders fully smooth at this
converged state (temporal accumulation alone is now doing the
smoothing), no banding or grain regression.

Verified: `cargo build/test/clippy --release --lib --examples` clean
(101/101 tests, no new warnings). Not yet committed.

### Temporal accumulation follow-up: reduce per-frame indirect-diffuse sample count

Temporal accumulation's other real payoff (spreading Stage B's 5-sample
budget across frames, cutting per-frame ray cost, not just cleaning up
the output — flagged as an explicit follow-up in both the temporal-
accumulation entry above and `cpu_ref.rs::pixel_jitter_angle`'s own doc
comment, written before temporal accumulation existed) is now built as a
live-tunable dial rather than a fixed number, per explicit instruction:
measure real convergence-speed/cost tradeoffs at multiple values first,
decide the shipped default from data, don't guess.

**Mechanism**: `IndirectDiffuseConfig::sample_count` (existed as an
unwired placeholder since Stage B) is now real — `hybrid_trace.wgsl`'s
`indirect_diffuse` fires only `sample_count` of its 5 fixed hemisphere
directions each frame, selected by a new deterministic round-robin
(`cpu_ref::indirect_sample_start`: `(frame_index * sample_count) %
5`, not randomized — matching this project's established preference for
testable dithering schedules over stochastic ones). At `sample_count=5`
(the unchanged default) this always selects all 5 regardless of frame
index, reproducing the pre-change behavior exactly — confirmed both by a
CPU-ref regression-pin test and a visual screenshot at the original
banding-repro camera angle. New render-world `HybridSampleFrameIndex`
counter (mirrors `HybridFrameParity`'s shape, kept separate since that
one is mod-2-only and serves a different concern: ping-pong slot
selection, not sample rotation). 6 new CPU-ref tests: full-sample-count
frame-independence, single-sample-count visits a different index each
frame and cycles every 5, every direction visited within a bounded
window for every `sample_count` 1..=5, the full-count regression pin,
and an exact ray-count-matches-sample-count check (the test that proves
the GPU-cost claim is real, not just that the math looks right).
`--indirect-samples N` CLI flag (clamped 1-5), egui "Samples/frame"
slider in the existing "Indirect diffuse" panel section.

**Real sweep data** (`--stress 10000`, box shape, near camera, warmup
run discarded, 12-13 repeats per configuration — same heavy-GPU-
contention caveat on absolute numbers as both prior entries in this
arc, load average ~7 during measurement; isolated indirect-only cost
below is contention-insulated the same way the denoise/temporal passes'
own costs were, by subtracting the measured `indirect off` floor):

```
indirect off (floor):     34.99 ms  (indirect cost: 0.00 ms)
samples=1:                41.39 ms  (indirect cost:  6.40 ms)
samples=2:                45.95 ms  (indirect cost: 10.96 ms)
samples=3:                51.54 ms  (indirect cost: 16.55 ms)
samples=5 (today's default): 62.38 ms  (indirect cost: 27.39 ms)
```

Isolated indirect cost scales almost exactly linearly with sample count
(~5.5ms/sample at this scene scale) — confirms each dropped sample really
does remove one full `trace()` call per pixel, not some fixed per-
dispatch overhead. `sample_count=1` cuts the isolated indirect-diffuse
cost by **~77%** (27.39ms -> 6.40ms) versus the current default.

**Visual convergence tradeoff — the real cost of the real savings.**
Static camera, converged history (`history_length` at its 24-frame cap):
`sample_count=1` is visually indistinguishable from `sample_count=5` —
both fully smooth, no banding, no extra grain (screenshot-verified). But
under CONTINUOUS motion (camera orbiting + object spinning, the actual
demo's steady-state condition), a direct A/B screenshot at the identical
frame shows real, visible grain/dithering texture on the moving object's
surface at `sample_count=1` that is not present at `sample_count=5` —
disocclusion keeps resetting parts of the history faster than a
1-sample-per-frame estimate can re-converge, so some pixels are
perpetually catching up rather than reaching the same steady-state
smoothness a static frame does. This is the real, honest tradeoff the
cost savings above buy: `sample_count=1`/`2` are close to free on a
static or slow-moving frame, but pay a real, visible noise cost whenever
the scene is actively moving.

**Decision, from this data: default changed to `sample_count: 2`.**
~60% isolated indirect-cost reduction (27.39ms -> 10.96ms at
`--stress 10000`) for no visible cost on a static/converged frame, and a
smaller motion-grain penalty than `sample_count=1` while still keeping
most of the savings. Confirmed live (no CLI flag) at
`--indirect-samples`'s default: gpu-trace dropped from ~16.11ms to
~10.99ms at the small default demo scene, sphere still fully smooth at
converged history. Remains fully live-tunable — `--indirect-samples N`
(1..=5) / the egui "Samples/frame" slider — for anyone who wants a
different point on the cost/motion-noise tradeoff (e.g. `5` to fully
disable this optimization, `1` for maximum savings at the cost of more
visible motion grain).

Also bundled: `DebugGizmos::default().aabbs` (examples/gallery.rs)
changed `true` -> `false` — AABB/BVH debug gizmos off by default in the
gallery example (unrelated to the GI work above, a small standalone UX
fix); still toggleable via the egui checkbox or `--aabb-gizmos on`.

Verified: `cargo build/test/clippy --release --lib --examples` clean
(106/106 tests, no new warnings). Not yet committed.

### Physics: point-source gravity, a real rotation-write-back bug (the actual "no rotation" root cause), and a stable-orbit milestone

A stability soak (`examples/physics_stability.rs`, new: a static planetary
`Sphere` plus 10 dynamic bodies across every closed-form-inertia shape
kind, dropped from spherical-Fibonacci-distributed points around it with
random initial rotations and small tangential initial velocities,
deterministic PRNG) surfaced two real solver bugs that the existing
single-drop/single-pyramid milestones never exercised.

**Gravity became a point source.** `PhysicsGravity` (`solve_static.rs`)
changed from a constant `Vec3` to `{ center: Vec3, magnitude: f32 }` with
`acceleration_at(position) = normalize(center - position) * magnitude` —
constant-magnitude (not inverse-square), matching this project's own
choice to model both flat-ground gravity (a center placed far below the
scene, where direction barely varies across a typical scene width — a new
regression test pins this) and small-planet gravity (a center at the
body's own origin, where direction visibly differs per position) with the
same formula. `solve_world.rs`'s substep loop recomputes each body's
gravity direction from its OWN current position every substep, not once
per frame — required for orbiting/resting bodies where the direction
genuinely changes as the body moves relative to a nearby center.

**Bug 1 — rotation never resolved at all** (user-reported: "cubes are
staying on corner instead of falling on face, no rotation at all").
Root-caused in two layers, both real:
1. `solve_world.rs`'s substep loop predicted linear position from
   velocity every substep but had no equivalent rotation prediction from
   angular velocity — fixed by adding the quaternion-derivative
   prediction step (`rotation += 0.5 * dt * [angular_velocity, 0] *
   rotation`, mirroring the existing linear prediction), gated on
   nonzero `inverse_inertia_local` the same way linear prediction gates
   on nonzero `inverse_mass`.
2. The ACTUAL remaining root cause, found only after layer 1 still didn't
   fix the visible symptom: `solve_world`'s end-of-frame write-back loop
   copied `linear_velocity`, `angular_velocity`, and `position` from the
   solved `BodyState` back into the ECS `RigidBody`/`Transform`
   components, but never copied `rotation` back into `Transform.rotation`.
   Every substep's rotation update happened correctly in the local
   `states` vector and was then silently discarded at frame end; the next
   frame's `BodyState::from_components` read `transform.rotation` fresh
   from ECS — still the untouched spawn rotation — while
   `angular_velocity` DID get written back and kept accumulating
   corrections frame after frame with nothing ever consuming them into an
   actual rotation change. Confirmed via a debug trace
   (`PHYSICS_DEBUG`-gated, since removed) showing `rotation` bit-identical
   across 300 frames while `angular_velocity` grew to `-9.8` rad/s and the
   body's linear position bounced up to 5x its drop height (energy being
   injected through the same feedback loop). One-line fix: add
   `transform.rotation = states[i].rotation;` to the write-back loop.
   New end-to-end regression test
   `a_tilted_box_settling_through_the_real_solve_world_system_has_bounded_angular_velocity`
   drives the real `solve_world` ECS system (not `solve_substep_jacobi` in
   isolation, which doesn't exercise the write-back loop where the bug
   actually lived) through 300 frames and asserts both bounded angular
   velocity and a settled resting position.

**Bug 2 — velocity derivation, two iterations to get right.** While
root-causing bug 1, an unrelated real bug surfaced: `solve_substep_jacobi`
derived a contact-corrected body's velocity as `existing_velocity +=
correction / dt`, additive on top of whatever velocity the body already
had. Since `position`/`velocity` at that point already reflected gravity's
own per-substep prediction, a hard landing's positional correction got
ADDED on top of the fast inbound velocity that caused the deep
penetration in the first place — confirmed via a standalone trace
producing a physically wrong bounce (-6.9 m/s in became +10+ m/s out in
one substep). First fix attempt derived velocity as `(final_position -
substep_start_position) / dt` instead (Müller et al.'s own convention) —
algebraically correct but numerically disastrous at realistic world-scale
positions: a body at `position.y ≈ 100.0` moving by gravity's own
per-substep delta (~1e-4 at 480Hz) loses almost all of that delta's
precision when subtracted from two `f32` values of magnitude 100 (only
~7 significant decimal digits total) — caught by a test regression
(expected `-0.1635`, got `-0.1758`, confirmed via an isolated hand-trace
reproducing the exact error before touching any solver code). Final fix:
a new `SubstepStart { position, rotation, linear_velocity,
angular_velocity }` snapshot (was just `(Vec3, Quat)`) threaded from
`solve_world` into `solve_substep_jacobi`, so velocity is derived as
`substep_start.linear_velocity + correction / dt` — the correction is
added to the PRE-integration velocity, never to a position delta, so the
lossy subtraction never happens and hard impacts are still absorbed
correctly (no double-counting the inbound speed). Same treatment applied
to `angular_velocity` via `SubstepStart.angular_velocity`.

**Orbit milestone** (user-requested twice): a body launched tangent to a
point-source gravity center at the circular-orbit speed for its altitude
(`v = sqrt(g * r)`, exact for this project's constant-magnitude field —
centripetal acceleration `v^2/r = g` solves directly with no inverse-
square correction) traces a stable, roughly circular path, fully resolved
by the real `solve_world` gravity integration + XPBD substep loop with no
scripted trajectory. New end-to-end test
`a_body_launched_at_orbital_velocity_completes_a_stable_orbit`
(`solve_world.rs`) runs slightly over one full orbital period and asserts
the radius stays within 30% of its start throughout AND the body returns
near its starting position after roughly half a period — proof of a
closed orbit, not just "stayed bounded while drifting in angle." New
example `examples/physics_orbit.rs`: a small sphere orbiting a purely
visual (non-collidable) gravity-center marker, with NO static collider
anywhere in the scene (an orbit test has nothing to rest on) — visually
confirmed via screenshot showing the orbiting body offset from center at
a plausible orbit radius. Note: the existing `LINEAR_DAMPING` (added for
general solver stability) bleeds a small amount of orbital energy every
substep, causing a slow inward spiral over many orbits (radius 10.0 ->
8.65 over ~2.5 periods in a live run) — expected given damping exists,
well within the test's tolerance, and not treated as a bug since
undamped orbits were never a stated goal.

Verified: `cargo build/test/clippy --release --lib --examples` clean
(277/277 tests, no new warnings beyond pre-existing ones). Stability soak
visually confirmed via screenshots at frame 30 and frame 400 — all 10
bodies (5 shape kinds) settle onto the planet's curved surface from their
different starting sides with no divergence, no `INSTABILITY` warnings.

### Physics stage 3 GPU port, Piece 1: buffers + predict-only compute pass, first-ever headless GPU test in this codebase

First piece of the Stage 3 GPU port (see the plan's "Stage 3 remainder —
GPU port of the XPBD substep solver" section): a persistent, `read_write`
GPU body-state buffer plus a predict-only compute pass (gravity + rotation
prediction, mirroring `solve_world.rs`'s own substep-loop math bit-for-
bit), deliberately isolated from contacts/atomics — the goal was proving
the buffer/pipeline/dispatch/readback plumbing works at all before adding
any of the atomics complexity later pieces need.

New module `src/physics/gpu/` (`types.rs`, `buffers.rs`, `pipelines.rs`,
`pass.rs`, `parity_test.rs`), registered by neither `PhysicsPlugin` nor
`HybridRenderPlugin` yet — per the plan's `PhysicsGpuEnabled` opt-in
decision, this lands as a self-contained, test-only module until the full
substep (Piece 4) exists; wiring it into the real per-frame render path is
explicitly deferred, not an oversight.

**Buffer layout**: `PhysicsBodyGpu`/`SubstepStartGpu` mirror
`solve_rigid::BodyState`/`SubstepStart` field-for-field, following
`hybrid::extract::ObjectGpu`'s scalar-exploded `vec3`/`Quat` convention
(no nested structs — neither is `bytemuck::Pod`), packing `inverse_mass`
into `position`'s unused `w` lane the same way `ObjectGpu` reuses padding
lanes. Persistent buffers allocated via `hybrid::pipeline::HybridDdgiAtlas`'s
own pattern (manual `create_buffer` + zero-init `write_buffer`,
reallocated only on a body-count change) — confirmed the right template
over `ObjectGpu`'s `RawBufferVec` (which is fully re-uploaded from ECS
every frame and would silently stomp a GPU-side write).

**Dispatch architecture — a real design gap the DDGI-relight template
didn't cover**: every existing compute pass in this codebase
(`hybrid::pass::hybrid_pass`) is `Core3d`-scheduled and `ViewQuery`-scoped,
because it's fundamentally per-camera work. Physics has no view/camera
dependency at all. Confirmed via reading `bevy_render-0.19.1` source
directly that the correct pattern is a plain `Render`-schedule system in
`RenderSystems::Render` that builds its own `CommandEncoder` and calls
`render_queue.submit` directly, with no `RenderContext`/`ViewQuery`
involved — this is what Bevy's own internal `render_system`
(`renderer/mod.rs`) and `slab_allocator.rs`'s buffer-growth path already do
for their own view-independent GPU work, not a workaround. `dispatch_physics_predict`
follows this pattern; not yet registered as a scheduled system since it's
only invoked directly by the parity test in Piece 1.

**Headless GPU test harness** (`src/physics/gpu/parity_test.rs`) — the
first render-world-driving test in this codebase (confirmed via repo-wide
search: every prior `cargo test` here is pure-CPU). Built a headless `App`
with a real `RenderPlugin` (real wgpu adapter, no window/winit) by
replicating `DefaultPlugins`' own plugin ordering up through exactly the
plugins `RenderPlugin::finish()` unconditionally assumes exist
(`ImagePlugin` for the default sampler, `WindowPlugin` for the
`WindowResized` message type `camera_system` reads unconditionally,
`MeshPlugin`/`CameraPlugin` for their own extract systems) — discovered
each dependency empirically (one panic → one added plugin) then
cross-checked against `bevy_internal::default_plugins`'s real ordering to
confirm this wasn't a fragile just-enough-to-pass set. `WindowPlugin` is
configured with `primary_window: None` — no real OS window is ever
created. Synchronous readback for test purposes only: a `MAP_READ`
staging buffer + `copy_buffer_to_buffer` + `map_async` + a blocking
`RenderDevice::poll(PollType::wait_indefinitely())` — explicitly NOT the
pattern the eventual production fold-in path should use (that needs the
plan's one-frame-latency double-buffered readback so the real per-frame
path never stalls on the GPU).

**Two real bugs caught by this test, both fixed before it passed**:
1. A genuine WGSL/Rust struct-layout mismatch: the predict uniform's
   `gravity_center` was declared as `vec3<f32>` in WGSL but as three flat
   `f32` fields in the `encase`-derived Rust struct — these do not
   reliably pack identically. Gravity silently read as always-zero for
   every body. Fixed by making the WGSL struct flat-scalar to match the
   Rust side exactly, with a doc comment on both sides warning against
   reintroducing a `vec3`-shaped field here.
2. `PipelineCache` pipeline compilation is genuinely async — the test
   initially called `dispatch_physics_predict` after a fixed 2 frames,
   during which the pipeline was still `Queued`, not `Ok`.
   `get_compute_pipeline` returns `None` in that state and
   `dispatch_physics_predict` silently early-returns (matching
   `hybrid_pass`'s own established "pipeline still compiling" early-return
   convention) — so the dispatch never ran at all, and the test's
   "mismatch" was really "nothing happened," not a math bug. Fixed by
   polling for pipeline readiness across up to 60 `app.update()` calls
   instead of guessing a fixed frame count; confirmed stable across
   repeated runs (0 flakes in several repeats) once fixed.

**Verification**: 3 new `types.rs` unit tests (round-trip position/
rotation/velocity through the GPU struct, byte-size-is-a-multiple-of-16
checks for both GPU structs) plus the end-to-end parity test comparing
GPU predict output against `solve_world.rs`'s own isolated CPU gravity/
rotation-prediction formula on a fixed 3-body scene (one static body that
must not move, one falling-only body, one falling-and-tumbling body —
exercises both the `inverse_mass`/`inverse_inertia_local` prediction
gates independently). `cargo build/test/clippy --release --lib --examples`
clean, 281/281 tests passing, zero new clippy warnings.

### Physics stage 3 GPU port, Piece 2: contact upload + atomic scatter pass — this codebase's first WGSL atomics

Second piece of the Stage 3 GPU port: uploads a substep's CPU-generated
contact list to GPU and scatters each contact's positional/rotational
correction into a per-body accumulator via `atomicAdd` — the first use of
WGSL `atomic<T>` anywhere in this codebase (confirmed via repo-wide search
before starting). Deliberately stops short of the apply/divide step (that's
Piece 3) so a bug here can only be "the atomic scatter itself is wrong,"
never confused with apply-math correctness.

**CPU-ref change first, per the plan's decided approach**: added
`MAX_LINEAR_CORRECTION = 2.0` to `solve_rigid.rs`, the linear analogue of
the existing `MAX_ANGULAR_CORRECTION`, so both backends clamp identically
before the GPU's fixed-point atomic scheme needs a documented overflow
bound. New regression test
`a_pathologically_deep_penetration_has_its_correction_clamped_to_max_linear_correction`
(a `depth: 1000.0` contact must still only displace a body by exactly
`MAX_LINEAR_CORRECTION`). Re-verified the stability soak
(`examples/physics_stability.rs`) visually at frame 400 after adding the
clamp — no regression, bodies still settle correctly on the planet
surface.

**Fixed-point atomic scheme**: WGSL has no native `atomic<f32>`, so
corrections are encoded as `atomic<i32>` via `round(value *
FIXED_POINT_SCALE)` before `atomicAdd`, decoded by dividing back down.
`FIXED_POINT_SCALE = 2^20 (1,048,576)`, picked with a documented 4x
overflow-safety margin against a worst case of 256 contacts scattering
onto one body at `MAX_LINEAR_CORRECTION` each
(`1,048,576 * 256 * 2.0 = 536,870,912`, well under `i32::MAX`'s
2,147,483,647) — a dedicated test
(`worst_case_accumulation_at_max_linear_correction_does_not_overflow_i32`)
pins this bound directly rather than trusting the arithmetic by
inspection alone. `to_fixed_point`/`from_fixed_point` helpers in
`src/physics/gpu/types.rs` exist specifically so CPU-side tests can
compute "the exact raw accumulator value the GPU should produce"
independently, without hand-duplicating the rounding logic at every call
site.

New GPU types: `ContactGpu` (mirrors `contacts::Contact` field-for-field),
`PhysicsAccumulatorGpu` (mirrors `solve_rigid::Accumulator`, whose own doc
comment already called it "the CPU analogue of the GPU's per-body
atomicAdd targets"). New WGSL: `assets/shaders/physics_scatter_position.wgsl`
— ported the exact generalized-inverse-mass correction math from
`solve_substep_jacobi`'s contact loop (Müller et al. 2020, eq. 2-4),
including its own quaternion-rotate/quaternion-inverse helpers, verified
byte-for-byte against `glam::Quat::mul_vec3`'s actual formula (not just
"looks like the same math") before trusting the WGSL port.

**Contact upload buffer** follows `ObjectGpu`'s `RawBufferVec`-style
reallocation policy (grow-on-overflow, not `HybridDdgiAtlas`'s
allocate-once-per-size-change) — the right template for THIS buffer
specifically, since contacts are genuinely regenerated CPU-side every
substep, unlike the persistent body-state buffers Piece 1 already
allocates the other way.

**Verification — 2 new tests, both isolating "did the atomic scheme work"
from "does the full apply match" per the plan's own stated Piece 2
milestone**:
1. `gpu_scatter_position_matches_hand_computed_fixed_point_sum` — a
   2-body, single-contact scene (one dynamic, one static) with a
   hand-computed expected raw accumulator value (no solver code
   involved in computing the expectation), confirming the static body
   receives exactly zero scatter.
2. `many_concurrent_contacts_on_one_body_all_land_in_the_atomic_sum` — the
   case the plan specifically flagged as most needing GPU-specific
   verification (no CPU execution-order analogue exists for this): 64
   contacts scattering onto the SAME body from genuinely concurrent GPU
   invocations, confirming none are dropped or corrupted by the atomic
   accumulation — a real concurrency bug would show up as a wrong `count`
   or `sum`, not a crash, so this had to be checked explicitly rather than
   assumed safe because "atomics are supposed to be safe."

Both new tests, plus Piece 1's own predict test, run stably across
repeated executions (checked 3-4x in a row, zero flakes) — meaningful here
specifically because GPU concurrency bugs are exactly the kind of thing
that can pass once and fail intermittently.

Verified: `cargo build/test/clippy --release --lib --examples` clean,
289/289 tests passing, zero new clippy warnings. Not yet wired into
`HybridRenderPlugin`'s real per-frame path (same `PhysicsGpuEnabled`
opt-in deferral as Piece 1).

### Physics stage 3 GPU port, Piece 3: apply-average pass — first full CPU/GPU parity against the real solver

Third piece of the Stage 3 GPU port: the apply-average pass that reads
Piece 2's scattered accumulator, divides by count, clamps
(`MAX_LINEAR_CORRECTION`/`MAX_ANGULAR_CORRECTION`), applies the correction
to position/rotation, derives velocity from the `SubstepStart` snapshot,
then applies per-substep damping and the `MAX_ANGULAR_VELOCITY` ceiling —
completing the position/rotation round of a substep. This is the first
piece where the GPU pipeline's OUTPUT is compared directly against
`solve_rigid::solve_substep_jacobi`'s real output on the same input, not
just an internal consistency check (Piece 2's hand-computed accumulator
values) or an isolated formula (Piece 1's gravity prediction).

New WGSL: `assets/shaders/physics_apply_position.wgsl`, mirroring
`solve_substep_jacobi`'s apply loop through its position/rotation round
exactly (up to but not including `resolve_contact_velocities`'s own
velocity-round scatter/apply pair, which stays Piece 4's scope). New
`ApplyUniform` type — every clamp/damping constant
(`MAX_LINEAR_CORRECTION`, `MAX_ANGULAR_CORRECTION`, `LINEAR_DAMPING`,
`ANGULAR_DAMPING`, `MAX_ANGULAR_VELOCITY`) is read as a live uniform field
sourced directly from `solve_rigid`'s own `pub(crate)` constants (widened
from private specifically for this), rather than a second hand-copied set
of magic numbers in WGSL that could silently drift out of sync with the
CPU reference — a small but deliberate choice to keep exactly one source
of truth for tuning constants across both backends.

**Verification — 3 new parity tests, each borrowing an existing,
already-proven-correct `solve_rigid.rs` unit test's exact scene** (so the
"expected" side of each comparison is scenes already known to exercise
real, previously-debugged behavior, not newly invented ones):
1. `gpu_scatter_and_apply_matches_cpu_solve_substep_jacobi_single_contact`
   — one dynamic body resting against one static body.
2. `gpu_scatter_and_apply_matches_cpu_solve_substep_jacobi_multiple_contacts_on_one_body`
   — mirrors `multiple_contacts_on_one_body_are_averaged_not_summed`'s own
   scene: two simultaneous contacts on one body, confirming the GPU
   pipeline's divide-by-count step averages rather than sums, exactly
   like the CPU reference.
3. `gpu_scatter_and_apply_matches_cpu_solve_substep_jacobi_off_center_contact_induces_rotation`
   — mirrors `an_off_center_contact_induces_rotation_when_inertia_is_finite`'s
   own scene: confirms rotation is exercised end-to-end through the real
   GPU pipeline (quaternion update, angular clamp, angular velocity
   derivation), not just the linear path — a sanity assertion inside the
   test itself confirms the CPU reference DOES produce nonzero angular
   velocity for this scene, so a false pass from both sides trivially
   agreeing on zero rotation is ruled out.

All 3 new tests, plus every earlier Piece 1/2 test, passed on the FIRST
real run with no debugging needed — the WGSL quaternion-rotate helper
ported for Piece 2's scatter pass (already verified byte-for-byte against
`glam::Quat::mul_vec3`) and the fixed-point accumulator scheme from Piece
2 both carried over correctly with no new bugs surfacing at the apply
stage. Confirmed stable across 4 repeated runs (zero flakes) — meaningful
for a GPU-concurrency-adjacent pipeline even though this specific pass has
no atomics of its own (it only reads what Piece 2's scatter already wrote
atomically).

Verified: `cargo build/test/clippy --release --lib --examples` clean,
292/292 tests passing, zero new clippy warnings. Not yet wired into
`HybridRenderPlugin`'s real per-frame path (same `PhysicsGpuEnabled`
opt-in deferral as Pieces 1-2).

### Physics stage 3 GPU port, Piece 4: velocity round, full substep, first GPU-vs-CPU benchmark

Fourth and final piece of the plan's original 4-piece Stage 3 GPU port
breakdown: `resolve_contact_velocities`'s own scatter/apply pair (the
velocity-level, restitution-0 round that runs after the position/rotation
round), completing a full XPBD substep end-to-end on GPU. Also lands the
port's first real GPU-vs-CPU performance comparison.

**A real overflow-safety gap found and fixed before any velocity-round
code was written.** While designing the velocity round's fixed-point
scheme, re-examined Piece 2's own `MAX_LINEAR_CORRECTION` clamp and found
it only bounds the AVERAGED per-body correction (after dividing by
`count`) — but the GPU scatter pass `atomicAdd`s the raw, UNCLAMPED
per-contact value into the fixed-point accumulator BEFORE any division
happens. A single pathological contact could already overflow the
accumulator even though the CPU reference's final post-average result
would have been clamped. Fixed on both backends: `solve_rigid.rs`'s
per-contact loops (in both `solve_substep_jacobi` and the new
`resolve_contact_velocities`) now clamp each contact's own contribution
via `.clamp_length(0.0, ...)` BEFORE scattering into the accumulator, in
addition to the existing post-average clamp — and `physics_scatter_position.wgsl`
was updated to match (a `ScatterUniform` field addition,
`max_linear_or_velocity_correction`/`max_angular_correction`, threaded
through from the CPU constants). This is exactly the kind of gap the
CPU-ref-first doctrine exists to catch before it reaches GPU code, caught
here during design rather than via a failing test.

**New CPU-ref constant**: `MAX_VELOCITY_IMPULSE = 2.0` (same value as
`MAX_LINEAR_CORRECTION` deliberately — both accumulators share one
`FIXED_POINT_SCALE`, so raising one without revisiting the shared
overflow-safety margin would silently invalidate it), added to
`resolve_contact_velocities`'s own apply loop, which previously had NO
magnitude clamp at all. New regression test
`a_pathologically_fast_approach_has_its_velocity_impulse_clamped_to_max_velocity_impulse`
(a body approaching at 1000 m/s must still only receive a velocity change
of exactly `MAX_VELOCITY_IMPULSE`). Re-verified the stability soak
visually at frame 400 after both new clamps — no regression.

New WGSL: `assets/shaders/physics_scatter_velocity.wgsl` (mirrors
`resolve_contact_velocities`'s scatter half: relative-normal-velocity
impulse, gated on `normal_speed < 0.0`, restitution 0) and
`assets/shaders/physics_apply_velocity.wgsl` (mirrors its apply half:
divide by count, clamp, ADD to the body's existing — already
position-round-corrected and damped — velocity; no damping/re-integration
of its own, since this round only ever adds an impulse on top of what the
position round's apply pass already finished). The velocity-round scatter
pass reuses the position round's exact bind group layout (`scatter_position_layout`)
since both share the identical `ScatterUniform`/body/contact/accumulator
shape — a separate pipeline pointing at a different shader, not a
separate layout.

**Verification — 2 new parity tests reaching the plan's actual stated
milestone**:
1. `gpu_full_substep_matches_cpu_solve_world_single_substep` — all 5 GPU
   dispatches (predict, scatter-position, apply-position,
   scatter-velocity, apply-velocity) in the correct order, compared
   against `solve_substep_jacobi`'s complete output (which itself calls
   `resolve_contact_velocities` internally) on a box-vs-box resting
   scene. Passed on the first real run.
2. `gpu_full_frame_matches_cpu_solve_world_across_all_substeps` — the
   plan's own literal milestone: all 8 substeps of a real
   `solve_world`-shaped frame, contacts regenerated every substep on both
   backends from a genuinely evolving body state (not independent
   snapshots), covering free-flight prediction (early substeps, no
   contact yet) through landing and settled contact resolution. Also
   passed on the first real run. Both tests stable across 4 repeated
   executions (zero flakes).

**First GPU-vs-CPU benchmark** (`examples/_physics_gpu_bench.rs`, one-off
scratch — built, measured, deleted, same convention as the "Physics stage
3 follow-up" entry's own 2000-body scaling measurement). Deliberately
scoped as an ISOLATED solve-step comparison (CPU `solve_substep_jacobi`
vs. the GPU predict/scatter/apply pipeline, holding a pre-generated
2000-contact list constant on both sides) rather than a full-frame
comparison against the existing ~190-210ms/frame CPU baseline at 2000
bodies — that number is dominated by contact generation, which this piece
didn't touch, and a full-frame comparison would mostly measure something
this GPU port didn't change.

Results at 2000 bodies / 2000 contacts (200 iterations, averaged, run 3x
for consistency): CPU `solve_substep_jacobi` ≈ 0.22 ms/substep. GPU with a
synchronous `poll(wait)` after every substep ≈ 0.29-0.38 ms/substep (0.6-
0.75x CPU — SLOWER) — this is the honest cost of exactly the
per-substep CPU/GPU stall the plan's readback design was built to avoid.
GPU with dispatches submitted back-to-back and polled only once at the
end ≈ 0.20-0.25 ms/substep (0.9-1.1x CPU — roughly at parity, not yet a
clear win). Interpretation: at this body count, the CPU's serial loop is
already fast enough (sub-quarter-millisecond) that GPU dispatch overhead
(5 separate dispatches per substep, each with its own bind group/command
encoder) isn't yet amortized by enough parallel work to show a clear
speedup — the real payoff is expected to emerge at substantially larger
body counts where CPU cost scales linearly but per-dispatch GPU overhead
stays roughly fixed, not measured here since that's a distinct claim from
this piece's own "does the algorithm port correctly" scope. Recorded
honestly rather than only reporting the more favorable pipelined number.

Verified: `cargo build/test/clippy --release --lib --examples` clean,
295/295 tests passing, zero new clippy warnings. This completes the
plan's originally-scoped 4-piece Stage 3 GPU port (buffers+predict,
contact-scatter, apply-average, velocity-round+full-substep) — still not
wired into `HybridRenderPlugin`'s real per-frame path or the
`PhysicsGpuEnabled` toggle itself (that resource doesn't exist yet); the
production fold-in (one-frame-latency double-buffered readback,
`readback.rs`) remains unbuilt, per the plan's own note that it's
unnumbered plumbing work with no CPU-parity content of its own, layered
on top of now-fully-verified GPU math.

### Physics stage 3 GPU port: body-count sweep finds the actual CPU/GPU crossover

Follow-up to Piece 4's own single-size (2000-body) benchmark, which found
"roughly at parity, no clear win yet" and explicitly flagged that the real
payoff was expected at larger scale but not yet measured. Re-ran the same
isolated solve-step comparison (one-off scratch example, built/measured/
deleted, same convention) at 2,000 / 5,000 / 10,000 / 20,000 bodies (100
iterations each, contact count held equal to body count, same resting-
contact scene shape as the prior benchmark) to find the actual crossover
curve rather than guessing at the trend from one data point.

**Results** (two independent runs, both showing the same trend):

```
  bodies |    cpu(ms) |   gpu-sync |   gpu-pipe | sync/cpu | pipe/cpu
----------------------------------------------------------------------
    2000 |      0.18  |      0.28  |      0.22  |    0.6-0.7x |  0.8-0.9x
    5000 |      0.44  |      0.46  |      0.43  |    0.9-1.0x |  1.0-1.1x
   10000 |      0.94  |      0.75  |      0.80  |    1.1-1.4x |  1.1-1.3x
   20000 |      2.21  |      1.55  |      1.66  |    1.3-1.6x |  1.2-1.5x
```

(`gpu-sync` = a `poll(wait)` after every substep, the same worst-case
cost the plan's one-frame-latency double-buffered readback is designed to
avoid in production; `gpu-pipe` = dispatches submitted back-to-back,
polled once at the end, approximating the achievable throughput once
readback isn't artificially serializing every substep.)

**Confirms the interpretation Piece 4 predicted but didn't measure**: CPU
`solve_substep_jacobi` cost scales roughly linearly with body count
(0.18ms → 2.2ms, a ~12x increase for a 10x body-count increase — the
mild super-linearity likely reflects cache pressure on the growing
`Vec<BodyState>`/`Accumulator` working set, not an algorithmic
non-linearity). GPU cost grows much more slowly (0.22ms → 1.5-1.7ms
pipelined, a ~7x increase for the same 10x body-count increase) since the
fixed per-dispatch overhead (5 dispatches/substep, each with its own bind
group and command encoder) amortizes better as the actual parallel
workload grows. The crossover sits between 2,000 and 5,000 bodies; by
20,000 the GPU pipeline is a genuine, consistent 1.2-1.6x faster even in
the pipelined case — a real, repeatable win, not noise (confirmed via 2
independent runs producing the same ordering and rough magnitude at every
size).

**Caveat, stated plainly**: this remains an ISOLATED solve-step
comparison — contact generation (still 100% CPU-side, unported) is held
constant and excluded from both sides' timings, so this is not yet a
full-frame win at any body count; the plan's own broad-phase GPU port
(deferred, CSR-style, `broadphase::SpatialHash`'s own doc comment already
anticipates the layout) would need to land before a real end-to-end frame
time comparison means anything. This sweep only answers "does the ported
solver math itself get faster on GPU at scale" — yes, confirmed, with a
measured crossover point — not "is the whole physics frame faster yet."

No architecture changes made for the user's stated future soft-body
(MPM)/cloth plans — the existing plan (Stage 6 section) already scopes
MPM as a separate, non-unified system from rigid-body XPBD with its own
per-particle/dense-grid representation, so building shared GPU
infrastructure now would be speculative generality for a system that
doesn't exist yet; deferred until Stage 6 actually starts, to be designed
against whatever the rigid-body GPU path looks like by then.

### Broad-phase/contact-generation GPU port, Piece 1: fixed-size sample points, `PhysicsShapeGpu`, GPU sample-point pass

First piece of the plan's newly-added "Stage 3 remainder — GPU port of
broad-phase and contact generation" sub-plan (the natural next step after
the XPBD solver port's own body-count sweep confirmed a real GPU win at
5,000+ bodies, but only for the SOLVE step — contact generation remains
100% CPU-only and dominates real frame time today).

**CPU reference refactored, decided up front per the plan's own explicit
choice** (not deferred, not left as a parallel `Vec`-returning API):
`sample_points::sample_points_local` changed its return type from a
heap-allocated `Vec<Vec3>` to a new `SamplePoints` type — a fixed
`[Vec3; 32]` + `count: u32` (32 = `MAX_SAMPLE_POINTS`, the max sample
count across every `PhysicsShape` variant), with `Deref`/`IntoIterator`
impls so every existing call site (`contacts.rs`'s `generate_contacts`)
and every existing test kept working completely unchanged — confirmed by
running the full existing test suite before writing a single new test,
zero regressions. This mirrors the same "the CPU reference must model the
real algorithm's actual data shape, not just agree on output values"
standard already applied to `solve_rigid::Accumulator` for the solver
port's own atomic scatter targets.

New `PhysicsShapeGpu` type (`src/physics/gpu/types.rs`): a `shape_kind: u32`
+ 8 flat `param_N: f32` positional struct, deliberately REUSING
`hybrid::extract::ShapeKindGpu` directly rather than a parallel duplicate
enum (`PhysicsShape` and the renderer's own `Shape` share the exact same
variant set minus `RoundedCone`, which neither has a tag/variant for, so
reusing the existing enum can't silently drift out of sync). Confirmed via
research before writing any WGSL: `assets/shaders/hybrid_trace.wgsl:285-392`
already has working, tested `sd_*`/`local_distance`/`local_normal` WGSL
functions — line-for-line ports of `cpu_ref.rs`'s same-named functions,
already used by the live renderer every frame — so this port's actual
per-shape SDF math needed zero new WGSL, only a struct to feed the
existing functions.

New WGSL: `assets/shaders/physics_sample_points.wgsl` — one invocation per
body, ports every `sample_points.rs` helper (`box_corners`, `ring`,
`cylinder_rings`, `capsule_rings`, `ellipsoid_fibonacci`) formula-for-
formula. Two real correctness traps found and fixed BEFORE running
anything, by reading glam's actual source rather than assuming a
plausible-looking implementation:
1. `capsule_rings`' `any_orthonormal_pair` is NOT a simple "pick whichever
   of X/Y gives a stable cross product" branch (the first, wrong,
   assumption) — glam's real implementation is the Duff et al. "Building
   an Orthonormal Basis, Revisited" (Pixar) formula. Ported verbatim after
   reading `glam-0.32.1/src/f32/vec3.rs`'s actual source.
2. `normalize_or`'s real fallback condition is "the length's reciprocal is
   finite and positive" (`rcp.is_finite() && rcp > 0.0`), not a fixed
   epsilon threshold on length. WGSL has no `isNan`/`isInf`/`isFinite`
   builtins at all — confirmed via research that these were removed from
   the WGSL spec entirely around 2021 over undefined fast-math behavior on
   GPU backends, not merely unimplemented by naga. Reproduced the exact
   condition anyway without them: `rcp > 0.0` already excludes NaN (any
   NaN comparison is false in IEEE-754) and negatives, and clamping `rcp`
   against `f32::MAX` collapses the one remaining case (`rcp == +Infinity`,
   i.e. `length(diff) == 0.0`) without a dedicated finiteness builtin.

**Verification**: `gpu_sample_points_match_cpu_sample_points_local_for_every_shape`
— all 7 `PhysicsShape` variants, checking BOTH point-for-point numerical
agreement with the CPU reference AND (the plan's own stated milestone,
and a genuinely independent check a shared-bug-on-both-sides wouldn't
catch) that every GPU-produced sample point still lies on its shape's
real surface via `cpu_ref::local_distance`, the same invariant
`sample_points.rs`'s own CPU tests check. Passed on the very first real
run across all 7 shapes — the glam-source-verified fixes above meant no
further debugging was needed once the shader was written. Stable across 3
repeated runs. Also added 2 new CPU-only tests
(`max_sample_points_is_never_exceeded_by_any_shape_kind`,
`sample_points_padding_beyond_count_is_never_exposed`) plus a
`PhysicsShapeGpu` encoding test covering all 7 shape kinds' param layouts.

Verified: `cargo build/test/clippy --release --lib --examples` clean,
300/300 tests passing (up from 295), zero new clippy warnings, zero
regressions in the pre-existing `sample_points.rs`/`contacts.rs` test
suites despite the CPU API's return-type change.

### Broad-phase/contact-generation GPU port, Piece 2: GPU parallel prefix-sum (Hillis-Steele), proven standalone

Second piece of the broad-phase/contact-generation sub-plan: a real GPU
parallel exclusive prefix-sum, proven in isolation against a trivial CPU
reference before it becomes load-bearing for the broad-phase hash's own
CSR bucket-offset computation (Piece 3). This is the first genuinely NEW
algorithmic component this entire GPU port has needed — every earlier
piece (predict/scatter/apply, sample points) ported an existing CPU
formula; a parallel scan has no CPU-reference-shaped analogue to port,
since `broadphase.rs`'s own CPU reference just does a trivial sequential
loop (`bucket_start[i+1] += bucket_start[i]`).

**Algorithm choice already settled by the plan** (`docs/knowledge/compute-shaders/`'s
own doctrine against synchronous mid-frame CPU readback, cited explicitly
rather than re-litigated): on-GPU Hillis-Steele multi-pass scan, not a
CPU readback-scan-reupload and not a work-efficient Blelloch scan.
`log2(count)` ping-ponged step passes (`physics_scan_step_main`), each
reading offset-`2^d` neighbors from one buffer into the other (in-place
isn't safe — every invocation reads a neighbor another invocation in the
same dispatch may also write), followed by one dedicated exclusive-shift
pass (`physics_scan_to_exclusive_main`: `output[i] = input[i-1]`,
`output[0] = 0`) rather than folding the shift into the scan loop itself.

New WGSL: `assets/shaders/physics_scan.wgsl`. New Rust: `ScanUniform`,
`ScanGpuState`/`ScanGpuBuffers` (a standalone ping-pong buffer pair,
deliberately separate from every other buffer resource — the scan is a
reusable utility, not owned by one specific caller), `ScanGpuPipeline`
(two entry points sharing one bind group layout, mirroring
`physics_scatter_position_layout`'s own one-layout-two-pipelines
precedent), and `dispatch_physics_scan` in `pass.rs` — the first dispatch
function in this port that itself orchestrates a data-dependent LOOP of
sub-dispatches (`ceil(log2(count))` steps) rather than a single dispatch,
returning which ping-pong buffer holds the final result since that
depends on the (data-dependent) step count's parity.

**Verification — exactly the plan's own stated milestone**: a dedicated
test (`gpu_scan_matches_cpu_exclusive_scan_reference`) sweeping 13 sizes
from 1 to 1000, DELIBERATELY including non-power-of-two lengths (3, 7,
17, 63, 100, 255, 257, 1000) since `table_size = max(body_count*2, 256)`
is never guaranteed to be a clean power of two — a scan algorithm that
only worked at power-of-two sizes would silently break the very use case
this port needs it for. A second dedicated test
(`gpu_scan_matches_cpu_reference_at_the_largest_supported_table_size`)
checks the actual target scale directly: 40,000 elements, matching
`table_size` at the largest body count (20,000) the GPU-vs-CPU sweep
already measured — proving the scan works at scale, not just at the small
sizes convenient for a quick test. Both passed on the first real run,
stable across repeated executions. Isolated entirely from hashing/
scattering, per the plan's own explicit sequencing rationale: a bug here
can only mean "the scan itself is wrong," never confused with broad-phase
hashing logic once Piece 3 wires this in as a dependency.

Also fixed 2 new clippy warnings surfaced by this piece
(`too_many_arguments` on the per-pass dispatch helper, an unnecessary
`as u32` cast) by bundling the dispatch helper's fixed-across-calls
parameters (pipeline id/layout/cache, and separately the ping-pong
buffer pair) into two small local structs — no behavior change, purely
argument-count hygiene.

Verified: `cargo build/test/clippy --release --lib --examples` clean,
302/302 tests passing (up from 300), zero new clippy warnings after the
bundling fix. Not yet committed.

### Broad-phase/contact-generation GPU port, Piece 3: full broad-phase GPU port (hash, count, scan, scatter)

Third piece of the broad-phase/contact-generation sub-plan: the complete
hash → count → prefix-sum → scatter pipeline, producing a GPU-built CSR
`bucket_start`/`bucket_items` pair matching `SpatialHash::build`'s own CPU
output exactly. This wires Piece 2's scan into real load-bearing use for
the first time.

New WGSL: `assets/shaders/physics_broadphase_hash.wgsl`
(`physics_broadphase_hash_main`, `physics_broadphase_count_main`),
`assets/shaders/physics_broadphase_scatter.wgsl`
(`physics_broadphase_copy_main`, `physics_broadphase_scatter_main`).
`cell_coord`/`cell_hash` port `broadphase.rs`'s own Müller spatial-hash
formula verbatim — confirmed via research beforehand that WGSL's
`u32(negative_i32)` bit-reinterprets exactly like Rust's `as u32`, so no
sign-handling surprises versus the CPU reference. New Rust:
`HashCountUniform`, `ScatterCopyUniform`, `BroadphaseScatterUniform`
(types.rs); `BroadphaseGpuState`/`BroadphaseGpuBuffers`, which embeds a
`ScanGpuState` directly rather than a duplicate ping-pong pair, reusing
Piece 2's already-proven scan machinery for the bucket-offset computation
(buffers.rs); `BroadphaseHashPipeline`/`BroadphaseScatterPipeline`
(pipelines.rs); `dispatch_physics_broadphase` orchestrating all five
GPU-side steps — hash, count, Piece 2's scan (unchanged), copy
(`bucket_start` → a fresh atomic `cursor` buffer, a tiny extra dispatch
chosen over a raw `copy_buffer_to_buffer` so the cursor initialization
stays entirely on-GPU with no host-side buffer aliasing), scatter — in one
sequenced call (pass.rs). Added `pub(crate)` accessors
(`bucket_start`/`bucket_items`/`table_size`) to `SpatialHash` so the
parity test can diff the GPU's raw CSR arrays directly against the CPU
reference's real internal state, not just through `query_candidates`'s
aggregated view.

**A real bug found and fixed before landing**: the count pass initially
mirrored the CPU reference's own indexing literally —
`atomicAdd(&bucket_counts[hash + 1], 1)`, matching
`SpatialHash::build`'s `bucket_start[h+1] += 1`. This looked correct by
inspection but is actually wrong once Piece 2's *generic* scan is the
consumer: the CPU's very next line (`bucket_start[i+1] += bucket_start[i]`)
is a single fused pass that turns pre-shifted counts directly into the
exclusive-scan result — it is NOT equivalent to "run an ordinary inclusive
scan over raw counts, then separately shift to exclusive," which is what
`dispatch_physics_scan` actually does (Hillis-Steele inclusive steps, then
a dedicated `physics_scan_to_exclusive_main` pass). Feeding
already-shifted counts into that generic exclusive-scan pipeline
double-shifts the result by one bucket. Caught immediately by both new
parity tests failing with a clean, diagnosable one-position-shift pattern
(`GPU: [0, 0, 3, 3, ...]` vs `CPU: [0, 3, 3, 3, ...]`) — confirmed the
exact mechanism by re-reading `broadphase.rs`'s own count/scan lines
side-by-side with `dispatch_physics_scan`'s contract before writing the
fix, rather than guessing. Fix: the count pass now writes RAW (unshifted)
per-bucket counts (`atomicAdd(&bucket_counts[hash], 1)`), which is exactly
what the generic scan expects as input; `bucket_counts[table_size]` (the
buffer's last slot) is always left at zero, matching the fact that no
body's hash is ever `>= table_size`. No changes were needed to the copy or
scatter passes, or to Piece 2's scan itself — both were correct in
isolation from the start, the bug was purely in how the two pieces'
indexing conventions interacted.

**A second bug found while chasing what looked like new flakiness**: after
the fix above, the full test suite intermittently failed a *different,
untouched* test (`gpu_scan_matches_cpu_exclusive_scan_reference`,
Piece 2's own test, no code of its own changed this piece) with "scan step
pipeline never reached Ok." Root cause: this piece's 4 new pipelines
(hash, count, copy, scatter) widened the concurrent shader-compilation
window every parity test pays at headless-app startup (each test spins up
its own fresh `App`, so growing the total pipeline count this plugin
registers slows down *every* test's startup, not just this piece's own).
The existing `wait_for_pipeline_ready`/`_generic` helpers polled
`PipelineCache::get_compute_pipeline(..).is_some()` for a fixed 60
`app.update()` iterations with no distinction between "still compiling"
and "genuinely failed" — under the wider compilation window this
occasionally exhausted the budget while a pipeline was merely still
`Queued`/`Creating`. Reading Bevy's own `PipelineCache::process_pipeline`
source revealed the real fix: switch to
`get_compute_pipeline_state`, which exposes `CachedPipelineState`
directly, and — critically — `ShaderNotLoaded`/`ShaderImportNotYetAvailable`
are BOTH already treated as retryable by Bevy itself (automatically
requeued to `Queued` on the next `process_queue`, confirmed by reading
that function's own match arms) rather than terminal failures. An initial
attempt to "fail fast on any `Err`" for better diagnostics was itself
wrong for exactly this reason — it turned Bevy's own normal transient
"shader asset still loading" state into a spurious hard failure,
reproducing the flake deterministically (6/6 failures) instead of fixing
it. Final fix: treat `Err` as informational only (capture its message,
keep polling) and raise the iteration budget to 300 (cheap — this is
one-time app-startup cost, not a per-frame budget), panicking with the
last-seen error text only once the full budget is exhausted with no `Ok`.
Confirmed stable across 6 consecutive isolated runs and 3 consecutive full
`cargo test --release --lib` runs after the fix.

Verified: `gpu_broadphase_matches_cpu_spatial_hash_csr_output` (direct CSR
array comparison against `SpatialHash::build` on a fixed 5-body scene) and
`gpu_broadphase_finds_every_true_overlapping_pair_in_a_random_cluster`
(the CPU reference's own existing brute-force cross-check test, ported to
run against the GPU-built hash's CSR arrays via local
`cell_coord`/`cell_hash`/candidate-query closures — same PRNG seed and
50-body cluster as the original CPU test) both pass, stable across 4
repeated runs. `cargo test --release --lib` clean at 304/304 (up from
302), stable across 3 repeated full-suite runs. `cargo clippy --release
--lib --examples` clean (the one new warning surfaced,
`dead_code` on `SpatialHash`'s new test-only accessors when clippy
analyzes a non-test build, silenced via `#[cfg_attr(not(test),
allow(dead_code))]` rather than papered over, since the accessors ARE used
in the real test build). Not yet committed.

### Broad-phase/contact-generation GPU port, Piece 4: contact-generation dispatch

Fourth piece of the broad-phase/contact-generation sub-plan: GPU-side
contact generation, consuming Piece 3's broad-phase CSR output directly
(no CPU round-trip) for the dynamic-vs-dynamic path, plus a separate
direct-grid dispatch for dynamic-vs-static — mirroring
`solve_world::generate_all_contacts`'s own two-path split exactly, per
that module's own doc comment on why folding statics into the dynamic-
sized spatial hash would be wrong.

New WGSL: `assets/shaders/physics_contacts.wgsl` — a third (deliberate,
not accidental) copy of the `sd_*`/`local_distance`/`local_normal`
functions, operating on `PhysicsShapeGpu` directly, following this port's
established per-pass-file self-containment convention. Ports
`contacts::generate_contacts` verbatim (`generate_contacts_between`, both
sampling directions, the `own_local_distance` defensive depth-correction
term, the same normal-flip convention on the b-vs-a direction) as a shared
function called by two entry points:
`physics_contacts_dynamic_main` (one invocation per dynamic body, walking
its own 27-cell neighborhood via the broad-phase's own
`bucket_start`/`bucket_items`, same `j > i` dedup
`generate_all_contacts`'s own caller loop applies) and
`physics_contacts_static_main` (one invocation per flattened
`(dynamic_index, static_index)` pair, no spatial hash). New Rust:
`ContactGenUniform`, `ContactGenGpuState`/`ContactGenGpuBuffers` (owns its
own bodies/shapes/sample-points buffers rather than reusing
`PhysicsGpuState`/`SamplePointsGpuState`, avoiding a hidden coupling
between three pieces' allocation lifecycles), `ContactGenPipeline` (one
shared bind group layout, two pipelines — the by-now-established
one-layout-two-pipelines precedent), `dispatch_physics_contacts` running
both passes against the SAME atomic cursor/output buffer so contacts from
either path land in one combined list, matching `generate_all_contacts`'s
own single `Vec<Contact>` output.

**Unknown-output-size handling** (the plan's own flagged "genuinely new"
piece): a fixed-capacity `contacts_out` buffer plus a single-element
`atomic<u32>` cursor — `push_contact` does `atomicAdd` for the pre-
increment slot, writes only if still within capacity, silently no-ops
past it (per the plan's own "no silent capacity drops" principle, the
CALLER is responsible for a loud warning by comparing the read-back cursor
against capacity, which this piece's own over-capacity test confirms
reports the TRUE uncapped count, not a clamped one).

**A real bug found and fixed before landing**: `physics_contacts_static_main`
initially reused the SAME buffer object (`cursor`) for THREE different
bindings in one bind group — the two unused placeholder slots
(`bucket_start`/`bucket_items`, irrelevant to the static path's own logic
but still required since the bind group layout is shared with the dynamic
pipeline) plus the real `contact_cursor` binding. This produced silently
dropped writes: even an unconditional debug write to `contacts_out[0]`
(bypassing all distance-check logic entirely) never landed on GPU, despite
the dispatch running with a confirmed-correct workgroup count and uniform
values, and despite zero wgpu validation errors being logged at any log
level. Diagnosed by progressively eliminating hypotheses (upload
correctness, uniform staleness, pipeline entry-point mixup, CPU-side
readback artifacts from a missing `COPY_SRC` usage flag on the debug-only
readback path) until an unconditional marker write confirmed the pass
dispatched but its output never reached the buffer — isolating the cause
to bind-group aliasing specifically. Fixed by using `bodies`/`shapes`
(never aliased with `cursor`/`contacts_out` in this bind group) as the two
placeholder bindings instead. Not currently understood to be a documented
wgpu invariant (bind-group creation didn't reject it), so noted explicitly
in `dispatch_contacts_static`'s own doc comment as a real, previously-hit
hazard for any future pass that reuses a buffer as a placeholder binding.

Verified: `gpu_contacts_dynamic_vs_dynamic_matches_cpu_box_resting_on_box`
(mirrors `contacts.rs`'s own
`a_box_resting_flat_on_another_box_produces_a_multi_point_manifold` test),
`gpu_contacts_dynamic_vs_static_matches_cpu_small_corner_on_large_floor`
(mirrors `contacts.rs`'s own
`a_small_corner_poking_into_a_large_flat_face_is_still_caught` test, floor
as a genuinely static body), `gpu_contacts_combines_dynamic_and_static_paths_in_one_buffer`
(both paths in one dispatch, confirming they share the output buffer/
cursor without clobbering each other), and
`gpu_contacts_over_capacity_cursor_reports_the_true_uncapped_count` (the
no-silent-drops guarantee) all pass, stable across 5 repeated runs.
Contact-set comparisons use order-independent average-depth/average-normal
matching (mirroring the CPU reference's own
`contact_generation_is_symmetric_in_depth_regardless_of_argument_order`
test's averaging strategy), since GPU scattering has no guaranteed
ordering and a finite sample lattice never guarantees contact-for-contact
correspondence between independently-run implementations. `cargo test
--release --lib` clean at 309/309 (up from 305), stable across 3 repeated
full-suite runs. `cargo clippy --release --lib --examples` clean (one new
`too_many_arguments` warning on `ensure_contact_gen_buffers`, fixed by
bundling the three per-body input slices into a `ContactGenInputs` struct,
same argument-bundling precedent Piece 2 already established). Not yet
committed.

### Broad-phase/contact-generation GPU port, Piece 5: end-to-end frame wiring — GPU physics is now a genuine ~3x full-frame win

Final piece of the broad-phase/contact-generation sub-plan, and the piece
the solver-port sub-plan (its own 4-piece breakdown) never reached: every
earlier piece's own independently-tested dispatch function is now wired
into a real per-frame system, behind an opt-in toggle, with a genuinely
non-blocking GPU→CPU→`Transform` handoff.

New: `physics::integrate::PhysicsGpuEnabled` (a plain `Resource(bool)`,
default OFF — CPU `solve_world` stays the shipping default, gated via
`.run_if()`, not a plugin swap, since Bevy has no supported "remove a
plugin at runtime" API). `physics::gpu::extract` — `extract_physics_bodies`,
an `ExtractSchedule` system snapshotting the main-world physics body set
into a render-world resource every frame, mirroring
`hybrid::extract::extract_hybrid_scene`'s own `Extract<Query<...>>`
aggregation pattern and its `object_order: Vec<Entity>` precedent (here,
`RenderPhysicsGpuFrame::dynamic_entities`). `physics::gpu::sample_cache` —
a shape-keyed cache (`HashMap` keyed by `PhysicsShapeGpu`'s raw bytes)
so sample points, which are pose-independent, are computed once per
distinct shape rather than every frame for every body — a real, deliberate
optimization, not a correctness requirement, confirmed by rereading
`sample_points.rs`'s own doc comment that sample points depend on shape
kind/parameters only. `physics::gpu::readback` — the production
one-frame(+)-latency non-blocking readback, following
`bevy_render::gpu_readback::GpuReadbackPlugin`'s own shipped pattern
(confirmed by reading its full source) rather than inventing one from
scratch: `map_async` is requested at the end of the frame's dispatch chain
with no explicit `poll()` call needed (wgpu's own backends drive callbacks
forward as a side effect of other queue operations, exactly as Bevy's own
readback plugin already relies on), and the entity order that dispatch
used is stored alongside the pending receiver so the eventual result can
be applied to the correct entities even if the entity set changed in
between — a despawned/repurposed entity's stale result is silently and
correctly skipped via `World::get_mut` returning `None`, no special-casing
needed. `physics::gpu::frame::dispatch_physics_gpu_frame` — the real
`Render`-schedule orchestration, reproducing `solve_world`'s own algorithm
shape (predict, regenerate contacts every substep, position-round
scatter/apply, velocity-round scatter/apply) through the existing,
already-tested dispatch functions.

**Three real bugs found and fixed during this piece's own soak testing**
(the first real load this port has seen beyond a single dispatch or a
handful of test frames):

1. **A leaked `UniformBuffer`**: `dispatch_physics_extract_positions`
   (the small bridge pass added this piece to feed the solver's live body
   positions into broad-phase every substep — see point 3 below) initially
   allocated a brand-new `UniformBuffer` on every single call via
   `UniformBuffer::from(...)`, rather than reusing a persistent field like
   every other per-dispatch uniform in this port. This pass runs up to
   `SUBSTEPS - 1` times per frame, every frame — over a few hundred test
   frames, thousands of orphaned GPU buffer allocations accumulated and
   eventually triggered a genuine AMD/radv driver context loss
   (`"The CS has been cancelled because the context is lost. This context
   is guilty of a soft recovery"`) — not a flaky test, a real crash,
   confirmed reproducible. Fixed by moving the uniform into
   `BroadphaseGpuBuffers` as a persistent field, matching every other
   pass's own convention.
2. **A synchronous per-substep contact-count readback**: the original
   design read the GPU-discovered true contact count back to the CPU
   every substep (`create_buffer` + `map_async` +
   `poll(PollType::wait_indefinitely())`, a full GPU pipeline drain) to
   size the scatter/apply dispatch workgroup counts — 8 times per frame,
   every frame. Even after fixing bug 1, this remained a real driver-
   stability risk under sustained load. Fixed by
   `copy_contact_count_into_scatter_uniform`: a 4-byte GPU-to-GPU copy
   from the atomic cursor directly into the scatter uniform's
   `contact_count` field, with dispatch workgroups sized conservatively at
   the fixed `contact_capacity` upper bound (`body_count * 8`, the plan's
   own recommended heuristic, now finally wired to a live call site) and
   each shader's own pre-existing `params.contact_count` bounds check
   correctly skipping invocations beyond the true count. The one
   remaining CPU readback of the true count happens once per FRAME (for
   the loud `PHYSICS_GPU_CONTACT_OVERFLOW` warning only, never to size a
   dispatch), not once per substep.
3. **Two missing per-substep resets — the actual root cause of severe,
   compounding performance degradation, found via per-phase GPU timing
   instrumentation** (`RenderDevice::poll(wait_indefinitely)` forced after
   each phase, test-only, to get accurate wall-clock breakdowns from
   otherwise-async GPU work): both the broad-phase's own bucket-counts
   buffer (the scan's `buffer_a`) and the contact-generation pass's atomic
   cursor need resetting to zero EVERY substep, since broad-phase
   candidates and contacts are both regenerated fresh every substep (per
   `solve_world`'s own doc comment — bodies move during the solve). But
   the buffer-allocating functions that also reset them
   (`ensure_broadphase_buffers`, `ensure_contact_gen_buffers`) are only
   called ONCE per frame, before the substep loop — calling either again
   mid-loop would stomp GPU-computed state a prior substep already wrote
   (broad-phase positions refreshed by the new
   `dispatch_physics_extract_positions` pass; contact-gen's own body copy
   from `copy_live_bodies_into_contact_gen`). Without the missing resets,
   every substep's own `atomicAdd`s landed on top of every PRIOR
   substep's counts within the same frame, corrupting the broad-phase's
   CSR ranges and producing contact counts that grew unboundedly —
   measured directly: `dispatch_physics_contacts` itself went from
   sub-millisecond to **over 10 seconds** by the 5th substep of a single
   frame, before either fix. Fixed by two new lightweight functions,
   `reset_broadphase_bucket_counts`/`reset_contact_gen_cursor`, called
   every substep without re-uploading/reallocating anything else. After
   both fixes, per-substep costs for every phase stayed flat in the
   sub-millisecond-to-low-single-digit-millisecond range across an entire
   320-frame soak run, and this bug's own compounding cost was also the
   proximate trigger for most of the driver-instability incidents
   observed while debugging bug 1 (many more atomic operations per
   dispatch than intended, on top of the leaked-buffer issue).

**Also needed, not previously anticipated in the plan**: a new tiny bridge
pass, `physics_extract_positions.wgsl`/`dispatch_physics_extract_positions`
— broad-phase needs each substep's CURRENT body positions in a
`[f32;4]`-per-element layout, but the solver's live body buffer packs
position as only the first 3 floats of an 80-byte `PhysicsBodyGpu` struct
(plus rotation/velocity/inertia), so no raw buffer-to-buffer copy can
bridge the two layouts — this one-line-shader pass extracts just the
position field, GPU-to-GPU, every substep after the first (substep 0 uses
the frame's own already-correct extracted positions).

**Verification — three deliberately separate tiers**, per the plan's own
"a wiring bug and a solver-accuracy bug are different failure classes"
principle: Tier 1 (`gpu_physics_toggle_wiring_moves_bodies_and_stays_finite`)
confirms the full extract→dispatch→readback→apply loop executes end-to-end
(position actually changes from spawn, stays finite) and that toggling
`PhysicsGpuEnabled` back off mid-run correctly hands control back to CPU
`solve_world` with no double-application; Tier 2
(`gpu_and_cpu_paths_settle_to_matching_resting_positions`) runs the SAME
fixed tilted-box-on-floor scene to a settled rest state under both
backends independently (320 frames each, matching `solve_world.rs`'s own
300-frame settling-test convention) and confirms matching resting
position/rotation within a tolerance explicitly wider than the existing
single-substep parity tests' own (justified by two compounding, both-real
reasons stated in the test's own doc comment: the one-frame-plus latency
itself, and fixed-point quantization drift compounding over hundreds of
substeps) — passing on the first real run after the three bugs above were
fixed, both tests together completing in ~5.5-7.5s, stable across 5
repeated runs (previously: multi-minute hangs and, before that, driver
crashes). `cargo test --release --lib` clean at 311/311, stable across 3
repeated full-suite runs (all completing in under 9 seconds total — the
Piece 5 tests' own fix directly explains why the suite didn't get
dramatically slower despite adding real per-frame GPU dispatch tests).
`cargo clippy --release --lib --examples` clean.

**Mandatory full-frame `--bench` comparison — the number this entire
broad-phase/contact-gen GPU port sub-plan existed to produce**, measured
with a one-off scratch benchmark (built, measured, deleted, per this
project's established convention) using the SAME minimal physics-GPU-only
render-world registration `frame_test.rs`'s own harness uses for both
backends (the full `HybridRenderPlugin` also registers real-rendering
setup that needs more of Bevy's default plugin stack than a headless
benchmark provides — confirmed by an actual panic when tried directly),
comparing CPU `solve_world` against the complete GPU path (extract,
broad-phase, contact-gen, 8-substep solve, real one-frame-latency
readback engaged) at 2,000 / 5,000 / 10,000 / 20,000 dynamic spheres
landing on a static floor:

| Body count | CPU ms/frame | GPU ms/frame | Speedup |
|---|---|---|---|
| 2,000 | 1524.72 | 480.02 | **3.18x** |
| 5,000 | 3754.78 | 1263.71 | **2.97x** |
| 10,000 | 7504.91 | 2451.29 | **3.06x** |
| 20,000 | 14538.90 | 4973.39 | **2.92x** |

A genuine, consistent **~3x full-frame speedup at every tested body
count** — not just at the high end, and not merely "at parity" as the
earlier isolated solve-step sweep found for the solver alone below ~5,000
bodies (that measurement excluded contact generation entirely, which is
exactly what this piece finally ported). Reported honestly per this
session's own established practice: this specific benchmark's own CPU
numbers (1.5s-14.5s/frame) are higher than the much older ~190-210ms/frame
baseline recorded in the "Physics stage 3 follow-up" entry — expected and
not a regression, since that baseline used a different, less contact-dense
scene; this benchmark's own scene (a dense grid of spheres landing
simultaneously on a floor) was chosen specifically to stress contact
generation heavily and produce a scale-representative, self-consistent
CPU/GPU comparison on identical hardware and identical scene setup for
both backends, which is what a fair speedup ratio requires — the absolute
numbers are benchmark-specific, the ~3x ratio is the load-bearing result.
Speedup stays essentially flat across the body-count range rather than
growing with scale, suggesting neither backend has hit a qualitatively
different bottleneck yet in this range — a natural next profiling
question if larger scales are wanted later, not addressed by this piece.

This completes the plan's originally-scoped broad-phase/contact-generation
GPU port sub-plan (all 5 pieces) and, transitively, the solver-port
sub-plan's own long-deferred goal: a real, wired, measured full-frame GPU
physics path, opt-in and off by default, with CPU `solve_world` remaining
the correct, fully-tested shipping default until this path accumulates
comparable soak-test mileage in real use.

### Piece 5 follow-up: a real, user-caught bug — the GPU body buffer was being overwritten by its own stale readback every frame

Found the day after Piece 5 landed, via the user actually watching
`physics_stability --gpu` in a real window rather than only the headless
Tier 1/2 tests — exactly the gap those two tiers' own doc comments flagged
as a real risk ("a wiring bug and a solver-accuracy bug are different
failure classes," but neither tier ran long enough, at real wall-clock
framerate, with a rich enough scene, to surface this). Reported
symptoms: falling motion visibly jerkier than the CPU path, and bodies
never settling to a clean rest state.

**Root cause**: `dispatch_physics_gpu_frame` called
`ensure_physics_buffers(&frame.bodies)` — unconditionally, every single
frame — where `frame.bodies` is `extract_physics_bodies`'s own CPU
snapshot of `Transform`/`RigidBody`. But those components only get
updated by `physics_gpu_apply_readback` whenever a GPU result actually
lands, which is itself several frames behind the GPU's own current
internal state (the entire point of the non-blocking readback's latency).
Re-uploading that stale snapshot into the GPU's own persistent body
buffer every frame silently threw away everything the GPU had already
computed across the substep loop since the last readback landed, and
restarted from stale data — every single frame, forever. Confirmed by
logging applied positions frame-by-frame in a real windowed run: a body's
X coordinate alternated between two fixed values in a persistent,
non-decaying period-2 oscillation while Z kept monotonically increasing
(still genuinely falling) at the same time — not noise, a real limit
cycle. This is exactly the failure mode the user's own hypothesis named
correctly before any diagnostic confirmed it: "waits data from gpu to
update transform which sent back to gpu."

**Why neither headless test caught it**: Tier 1 only checks that
`Transform` changed and stays finite over ~60 polled frames — an
oscillating-but-bounded-and-finite trajectory satisfies that trivially.
Tier 2 compares GPU vs. CPU resting position after 320 frames within an
explicitly wide tolerance (justified by real, separate latency/
quantization reasons) — wide enough, on that specific single-box/flat-
gravity scene, to not distinguish "converged to rest" from "still
oscillating with small amplitude." Neither tier was measuring the thing
that actually broke (continuity of the GPU's own internal state across
frames); both were measuring end states that happened to look
superficially fine on their own specific fixed scenes.

**Fix**: `dispatch_physics_gpu_frame` now only calls
`ensure_physics_buffers` (which allocates AND uploads) when the GPU
buffer doesn't exist yet or the body COUNT changed (a body added/
removed) — once running with a stable body count, the GPU's own buffer is
authoritative and evolves purely through its own dispatch chain, exactly
like CPU `solve_world`'s own `states` vector persists frame-to-frame
rather than being reset from a stale snapshot. `ensure_physics_buffers`
itself is unchanged (every existing parity test still relies on its own
unconditional-upload contract for a fixed test scene) — the gating lives
only in the real per-frame orchestration.

**Verified**: re-ran the same position-logging diagnostic after the fix —
smooth, monotonic convergence with no oscillation (X/Y/Z all changing
smoothly frame-to-frame, velocity components no longer sign-flipping).
Visually re-confirmed via fresh screenshots of `physics_stability --gpu`
at frames 90/180/300 — bodies now settle into a visibly stable cluster
resting on the planet's surface, no floating/detached artifacts. `cargo
test --release --lib` still 311/311 clean, stable across 3 repeated runs
(the existing Tier 1/2 tests' own scenes were apparently not sensitive
enough to this bug to regress on it — a real, acknowledged gap; a more
targeted regression test — one that would actually go red on this class
of bug — is a reasonable follow-up, not yet written). `cargo clippy
--release --lib --examples` clean.

### Kinematic bodies (Stage 3.5), Piece 1: CPU-only

Third body classification alongside dynamic and static: `BodyKind` enum
(`Dynamic`/`Kinematic`/`Static`, defaults to `Dynamic`) in
`physics::components`. A kinematic body is `RigidBody` (real, externally-
set velocity) + `Inertia::STATIC` (immovable by corrections, reusing the
exact zero-inverse-mass gates that already make statics immovable) +
`BodyKind::Kinematic` — its `Transform` is driven externally (animation/
script) and never written by `solve_world`. Landed ahead of Stage 4
(hard-union compound bodies) per this plan's own reordering note, since
kinematics interact with Stage 5's sleeping wake-semantics.

`solve_world`'s `dynamics` query gained an `Option<&BodyKind>` fetch and
is partitioned once into `dynamic_rows`/`kinematic_rows` in the same pass
that builds `states`/`shapes` — no extra query traversal. Body-index
layout extended from two ranges to three: `0..dynamic_count` (dynamics),
`dynamic_count..dynamic_count+kinematic_count` (kinematics, placed
BETWEEN dynamics and statics so dynamic-vs-kinematic contact generation
reuses dynamic-vs-static's exact loop shape), `dynamic_count+kinematic_count..`
(statics). `generate_all_contacts` gained a `kinematic_count` parameter
and a new dynamic-vs-kinematic direct loop; deliberately NO kinematic-vs-
kinematic or kinematic-vs-static loop (both pairs have `inverse_mass ==
0.0` on both sides, guaranteed no-op via `solve_substep_jacobi`'s existing
`total_w <= 0.0` early-out). The write-back loop only iterates
`dynamic_rows` — a kinematic body's `Transform`/`RigidBody` are never
touched, confirmed bit-exact even under an active penetrating contact.

**`solve_rigid.rs` and `contacts.rs` needed zero code changes** — the
existing zero-inverse-mass gates and shape-agnostic contact generation
already handle kinematics correctly; both files gained doc-comment notes
explaining why, per the plan's own recommendation that this be documented
rather than left implicit.

**Verified**: all 6 CPU-testable reference tests from the plan pass
(`body_kind_defaults_to_dynamic` in `components.rs`; 5 more in
`solve_world.rs` — resting-on-stationary-kinematic-matches-static parity,
a moving platform carrying a resting body upward, kinematic Transform
never mutated despite an active contact, the three-loop contact-
generation exclusion tests, gravity never accelerating a kinematic body).
Every existing test still passes unchanged (319/319 total, up from 311),
confirming the `Option<&BodyKind>`/default-to-`Dynamic` migration is
fully backward compatible. `cargo clippy --release --lib --examples`
clean, zero new warnings.

**A real, non-obvious limitation found via the moving-platform test's own
development, not swept under the rug**: a kinematic body's Transform/
velocity is sampled once per FRAME (not once per SUBSTEP, matching every
other per-frame-sampled input in this solver). When an external driver
moves a kinematic platform's `Transform` by a full frame's displacement
in one instantaneous jump (the natural way to write such a driver, and
what this project's own test/example does), the resulting contact depth
against a resting dynamic body is presented to the solver as if it
appeared within a single substep. `solve_substep_jacobi`'s velocity-from-
correction formula (`correction / dt` with `dt` = substep dt, roughly
1/8th of frame dt at `SUBSTEPS = 8`) then derives a velocity roughly
`SUBSTEPS`x too large for that one substep, producing a launch-and-refall
resonance at higher platform speeds (confirmed via direct trace: a 1.0
m/s platform produced a periodic launch/arc/re-land cycle instead of
smooth tracking; a 0.1 m/s platform tracked cleanly; 0.3 m/s — used in
both the regression test and the `physics_playground` elevator milestone
— tracks within the test's explicit 0.2m/60-frame tolerance). This is a
pre-existing property of this solver's substep granularity applied to any
contact whose depth jumps discontinuously between frames, not a bug
specific to kinematic-body support — tracked here as a known limitation
(a real fix would need either smoother kinematic Transform interpolation
across substeps, or a `SubstepStart`-aware velocity derivation less
sensitive to first-substep depth jumps) rather than silently worked
around or hidden by picking an artificially slow test speed without
explanation.

**Visual milestone**: `examples/physics_playground.rs` gained a
`BodyKind::Kinematic` elevator platform (a `RoundedBox`, manually driven
up/down between two Y bounds by a new `drive_elevator` system standing in
for a real animation system) carrying a resting dynamic crate. Confirmed
via 3 headless screenshots across a full rise/fall cycle (frames 180, 300,
420 at 0.3 m/s): the crate stays centered on the platform throughout,
riding it up and back down with no jitter, no falling through, no
detachment — the platform and crate are visibly separated from the floor
at frame 180 (platform mid-rise) and both back near the floor at frame
420 (platform having cycled back down). (The pre-existing crate-pyramid
milestone in the same scene was observed to have separately destabilized
by frame 420 — an unrelated, pre-existing stacking-stability issue, not
caused by or interacting with this kinematic-body work.)

**Not yet started**: Piece 2 (GPU port of kinematic support —
`extract.rs` three-loop extraction, `frame.rs`'s per-frame partial-buffer
kinematic re-upload, a new `physics_contacts_kinematic_main` WGSL
sibling) per the plan's own incremental landing order — CPU-only Piece 1
must be fully verified and committed first.

### Kinematic bodies (Stage 3.5), Piece 2: GPU port

`extract.rs`'s single `dynamics` query gained an `Option<&BodyKind>` fetch
and is partitioned into dynamic/kinematic rows exactly mirroring
`solve_world.rs`'s own CPU-side partitioning, producing a three-range
`RenderPhysicsGpuFrame` (`kinematic_count` field added). `write_kinematic_bodies`
(new, `buffers.rs`) re-uploads ONLY the kinematic sub-range of the shared,
otherwise GPU-authoritative `bodies` buffer every frame via a partial-
range `write_buffer` — confirmed via wgpu's own `Queue::write_buffer`
signature that this is a first-class, already-used-elsewhere operation
(every existing `ensure_*` function already calls it at offset 0). New
`physics_contacts_kinematic_main` WGSL entry point (mirrors
`physics_contacts_static_main`'s own direct-grid shape exactly, over
`dynamic_count * kinematic_count` instead of `dynamic_count * static_count`),
`ContactGenPipeline` gained a third `kinematic_pipeline`, `ContactGenUniform`
gained a `kinematic_count` field, and the static pass's own body-index
offset shifted to `dynamic_count + kinematic_count` (kinematics sit
between dynamics and statics in the shared buffer, per the CPU-side
convention). `solve_rigid.rs`/`readback.rs`/`buffers.rs`'s existing
dynamic/static machinery needed no other changes — predict's own
`inverse_mass > 0.0` gate and readback's own dynamics-only `dynamic_entities`
list already made kinematics structurally unreachable from both.

**Two real bugs found during this piece's own soak testing, one pre-
existing and one new, both fixed:**

1. **Pre-existing since Piece 5, exposed by this piece's own soak
   extension**: `dispatch_physics_extract_positions` sized its dispatch
   and uniform by the solver's FULL body count (dynamics + kinematics +
   statics) instead of the dynamic-only count broad-phase's own
   `positions` buffer is actually sized for — a real out-of-bounds GPU
   write whenever any static body was present (i.e. always, in every
   real scene), live since Piece 5 landed. Fixed by bounding both the
   uniform and the dispatch workgroup count by `broadphase_buffers.body_count`
   (already the correct dynamic-only value) instead of `physics_buffers.body_count`.
2. **A genuine, pre-existing solver gap in `solve_substep_jacobi`'s
   position-round velocity derivation, on BOTH backends, newly exposed
   because kinematics are the first body kind actively driven by an
   external system during the GPU pipeline's own multi-frame shader-
   compile warm-up window**: `physics_stability --gpu`'s own kinematic-
   platform soak extension launched a resting box to Y=50+ within ~30
   frames. Root cause, isolated via direct per-substep contact-content
   logging (not index/buffer corruption, confirmed by testing progressively
   narrower reproductions until the exact trigger was found): the
   platform's own driving system moves its `Transform` every `Update`
   frame regardless of whether GPU physics dispatch has started yet,
   while the resting box's `Transform` stays frozen (dispatch is a no-op
   every frame until every pipeline compiles) — a real, moderate
   (~0.1-0.15m) penetration builds up across that window, and once
   dispatch resumes, `body.linear_velocity = start.linear_velocity +
   correction / dt` derives an enormous one-substep velocity from that
   correction divided by a small substep `dt`, with `MAX_LINEAR_CORRECTION`
   bounding only `correction` itself, never the divided result.
   `MAX_ANGULAR_VELOCITY` already closed this exact gap for the angular
   case (a direct clamp on the derived velocity STATE, not just the
   correction) — added the missing `MAX_LINEAR_VELOCITY` (15 m/s) mirror
   for linear, on both backends (`solve_rigid.rs`'s apply loop and
   `physics_apply_position.wgsl`'s matching clamp, `ApplyUniform` gained
   a `max_linear_velocity` field). 50 m/s was tried first and technically
   bounded the launch, but still visibly snapped the box tens of units
   into the air before falling back — 15 m/s (comfortably above every
   legitimate velocity any existing test reaches, including the ~9 m/s
   orbital-velocity test) was needed to actually keep the soak scene
   visually stable.

**Verified**: new CPU regression test (`a_correction_divided_by_a_tiny_substep_dt_has_its_derived_velocity_clamped_to_max_linear_velocity`,
`solve_rigid.rs`) reproduces the exact mechanism directly (a moderate 0.1
depth divided by a deliberately tiny substep dt). New GPU regression test
(`a_kinematic_platform_driven_during_pipeline_warm_up_does_not_launch_the_resting_body`,
`frame_test.rs`) reproduces the real scenario end-to-end — drives the
kinematic platform from frame 1 with NO wait for pipeline readiness first
(unlike every other test in this port), the one condition that actually
exposed the bug. Plus 4 more new tests covering combinations the original
plan's own Piece 2 checklist called for: kinematic+separate-static
coexistence, many-dynamic-bodies-plus-kinematic-plus-static with an
injected large-dt spike, kinematic near point-source gravity, and the
planned moving-platform GPU-vs-CPU parity test (within the same
Tier-2-style tolerance the existing resting-position parity test uses) —
all passing, stable across repeated runs. `kinematic_count == 0`
regression test confirms the pre-existing dynamic-vs-static-only path is
unperturbed. 327/327 tests total (up from 322), zero new clippy warnings.
Real `physics_stability --gpu` soak run (320 frames, 11 varied dynamic
bodies + kinematic platform + static planet) confirmed clean — zero
`INSTABILITY`/`PHYSICS_GPU_CONTACT_OVERFLOW` firings, visually confirmed
via screenshots — after both fixes landed (neither fix alone was
sufficient; the soak still failed with only the out-of-bounds fix, and
still failed with only a 50 m/s velocity clamp).

### avian3d adopted as primary physics engine, Stage 1: shape mapping + parity example

A side-by-side comparison against the custom SDF-native engine above found
its own justification — "free collision against arbitrarily complex SDF/CSG
geometry" — has never actually been exercised: every real scene built with
it collides only simple convex primitives, exactly avian3d's own native
strength, while avian3d ships joints/CCD/sleeping the custom engine still
lacks. Decision: adopt `avian3d` (the standard Bevy-ECS physics engine) as
the primary/default engine; keep the custom GPU path scoped to visual-
effects-only (large counts of simple, individually-inconsequential bodies).
The custom CPU/GPU engine is demoted, not deleted (see the plan document's
own Stage 3 for the explicit disposal decision).

`avian3d = "0.7.0"` added (confirmed compatible with `bevy = "0.19.1"` via
`cargo add --dry-run` first). New `src/physics_avian/mod.rs`:
`shape_to_collider(&sdf::components::Shape) -> Option<avian3d::prelude::Collider>`
mapping every collision-relevant shape this project uses — `Sphere`,
`RoundedBox` (→`round_cuboid`, unit-converting half-extents to avian3d's
own full-length convention), `RoundedCylinder` (→plain `cylinder`, edge-
rounding dropped as a documented, deliberate approximation — this shape
has never been on a collision-critical path in any existing scene),
`Capsule`, `Ellipsoid` (→a unit sphere collider; the caller sets
non-uniform `Transform.scale = radii`, which avian3d itself automatically
approximates as a convex polyhedron once scaling is non-uniform, confirmed
via avian3d's own docs). `BoxFrame`/`HexPrism`/`RoundedCone` unsupported
(`None`), `RoundedCone` consistent with its own already-tracked SDF bug.
New `src/physics_avian/gravity.rs`: `PointGravity` resource reproducing the
demoted engine's own point-source (constant-magnitude, not inverse-square)
gravity model, built on avian3d's own `ConstantLinearAcceleration`
component — confirmed via avian3d's docs that this component is
automatically integrated every physics step with no special scheduling,
so an ordinary `Update`-schedule system recomputing its value from each
body's current position is sufficient (landed ahead of its own originally-
planned Stage 2 since it was small enough to fold in here).

New `examples/physics_avian_playground.rs`: a direct avian3d port of
`physics_playground.rs`'s three milestones (rounded-corner-floor sphere,
3-2-1 crate pyramid, kinematic elevator), every entity carrying both the
SDF renderer's own `Shape` (rendered unchanged) and avian3d's own
`RigidBody`/`Collider`. The kinematic elevator uses avian3d's own
documented convention (set `LinearVelocity`, let avian3d integrate
`Transform`) — the opposite of the demoted engine's own convention (write
`Transform` directly) — avian3d's own docs warn direct `Transform` writes
on a kinematic body are "similar to teleporting... can result in
unexpected behavior since the body can move inside walls."

**Two real, pre-existing bugs found and fixed, both in
`HybridRenderPlugin`, neither specific to avian3d**: it unconditionally
registers the demoted engine's own `extract_physics_bodies` GPU-extract
system, which reads `Extract<Res<PhysicsGpuEnabled>>`/`Extract<Res<PhysicsGravity>>`
every frame — but never initialized either resource itself, silently
relying on every existing example ALSO adding `physics::integrate::PhysicsPlugin`
(which does). Invisible until an app added `HybridRenderPlugin` without
that plugin — exactly what an avian3d-only example correctly does.
Confirmed via a real panic ("Resource does not exist") on
`physics_avian_playground.rs`'s first run; fixed by `HybridRenderPlugin`
itself calling `init_resource` on both (already `Default`-implementing),
making the renderer self-contained regardless of which physics engine (if
any) an app adds alongside it. A "CommandQueue has un-applied commands"
warning spam observed during the pre-fix panic'd runs turned out to be a
symptom of Bevy's own panic-unwind teardown, not a separate bug — gone
once the real panic was fixed.

**Verified**: 5 new tests, 332/332 total passing (up from 327), zero new
clippy warnings. Headless screenshots at frame 180 and frame 400 confirm
the sphere settles, the crate pyramid drops and settles (some crates
tumbled during the drop — real avian3d friction/restitution behavior, not
instability; nothing jitters/explodes at frame 400), and the kinematic
elevator completes a full rise/fall cycle with no detachment. Re-ran
`physics_playground.rs` (the demoted CPU engine's own example) to confirm
zero regression from the `HybridRenderPlugin` fix.

### avian3d adopted as primary physics engine, Stage 3: custom CPU solver's disposal recorded

A deliberate, explicit decision (not a default) on what happens to the
demoted custom CPU physics engine (`solve_world.rs`, `solve_rigid.rs`,
`broadphase.rs`, `contacts.rs`, `sample_points.rs`, `collision_static.rs`,
`solve_static.rs`) and its three examples (`physics_orbit.rs`,
`physics_stability.rs`, `physics_playground.rs`) now that `avian3d` is the
primary engine (Stage 1). Decision: kept, not deleted — this is working,
regression-tested code, and deletion is a separate, later, more
consequential decision with its own blast radius, not bundled into the
demotion itself. None of the three CPU examples were migrated to avian3d;
they remain a working historical reference/comparison baseline (useful for
exactly the kind of side-by-side verification the whole migration decision
was based on).

`src/physics/mod.rs`'s own module doc comment was rewritten (not merely
appended to — its own opening line, "rather than a CPU engine like
Rapier/Avian," directly contradicted the new reality and would mislead a
reader who stopped at the header) to state the demotion plainly: `avian3d`
(`crate::physics_avian`) is now the primary engine, a summary of the
build-vs-buy finding that motivated the demotion, and an explicit note
that this module's own future deletion — if it ever happens — is a
separate cleanup pass nothing in the avian3d migration depends on.

**Verified**: `cargo build --release --lib` clean, 332/332 tests still
passing (this stage touches only a doc comment, so no test count change),
zero new clippy warnings.

### avian3d adopted as primary physics engine, Stage 4: GPU path re-scoped to visual-effects-only debris

The final stage of the avian3d migration: the demoted engine's own GPU
compute path (`src/physics/gpu/`) stops being paired with the CPU solver
as "the GPU port of the primary engine" and becomes its own subsystem for
large counts of simple, individually-inconsequential debris bodies where
GPU throughput is the goal, not gameplay-relevant correctness.

New `src/physics/gpu/effects.rs`: `GpuDebrisBody { linear_velocity,
angular_velocity, inverse_mass, inverse_inertia_local }`. Deliberately
keeps real per-body mass/inertia (the solver math genuinely needs these
fields regardless of gameplay scope — the plan's own original "drop
`Inertia` entirely" suggestion was too aggressive) but drops `BodyKind`
entirely (no kinematic/static distinction — interactive/moving-platform
bodies are now `avian3d`'s job, a debris chunk has no reason to ever be
"kinematic"). Deliberately reuses `physics::components::PhysicsShape`
rather than a new debris-specific shape enum — it's a plain collider-shape
type shared across every physics subsystem, not solver-specific.
`extract.rs`'s own query changed from reading `RigidBody`/`Inertia`/
`Option<BodyKind>`/`PhysicsShape` to `GpuDebrisBody`/`PhysicsShape`
directly, and the kinematic-range partitioning was removed outright
(extraction now always produces `kinematic_count == 0`). Confirmed via
grep before starting that every GPU dispatch function/WGSL shader
downstream of `extract.rs` needed zero changes — they only ever touch
`RenderPhysicsGpuFrame`'s own already-GPU-typed fields.

**Real fallout handled, not left broken**: `frame_test.rs` (9 tests)
needed a full rewrite — 3 migrated to `GpuDebrisBody`, 6 kinematic-
specific tests REMOVED (not adapted) since that capability no longer
exists on this path by design. `physics_playground.rs`/`physics_stability.rs`
(the demoted engine's own examples) both had their `--gpu` flag removed
outright rather than left silently broken — confirmed via a real
screenshot that passing `--gpu` there would have silently frozen every
body at spawn (their `RigidBody`/`BodyKind` entities are no longer
extracted by this path at all) instead of erroring; both now print a
clear error pointing at the new avian3d example if `--gpu` is still
passed.

New `examples/physics_gpu_debris.rs`: a static floor plus `--count N`
(default 500) falling debris spheres. Default count deliberately NOT in
Piece 5's own 2,000-20,000 benchmark range — that benchmark was an
isolated physics-dispatch comparison, while this windowed example also
pays this renderer's own real per-object raymarching + BVH-refit-under-
motion cost, never exercised before at more than ~13 bodies by any
earlier example. Confirmed via direct timing: 500 bodies took ~38s to
reach frame 150 windowed vs. `physics_stability.rs`'s 13-body scene
reaching the same frame count in ~11.5s — a real, separate, honestly-
reported rendering cost, not a physics bug, documented plainly in the
example's own doc comment rather than hidden behind a small default.

**Verified**: `cargo build`/`clippy --release --lib --examples --tests`
both clean (one real `collapsible_if` clippy warning in the new example,
fixed). 327/327 tests passing. Headless screenshot of the debris example
confirms hundreds of colored spheres settle correctly with realistic
scatter, no NaN/explosion artifacts. Both demoted-engine examples
re-verified to work correctly without `--gpu` and to error clearly (not
silently misbehave) if `--gpu` is passed. This completes the avian3d
migration plan's own 4 stages.

### Radiance Cascades experimental GI, Stage 1: CPU-testable cascade reference

DDGI (this renderer's default indirect-diffuse technique) has a known,
documented, unfixed limitation: `examples/gi_room.rs`'s own sealed-room
test scene shows a residual dark floor corridor that DDGI's fixed-density
probe grid can't reach — energy has to relay cell-to-cell through the
grid, and a real fix attempt (`sample_probe_column_boost`) was built,
tested, and found to fail on the real scene (see this file's own
"Investigated (not fixed): residual dark floor corridor after DDGI
infinite-bounce" entry, commit `1a45987`). Radiance Cascades (Alexander
Sannikov, ExileCon 2023; 3D extension per the "Sparse"/"Split Radiance
Cascades" papers) resolves long-range transport structurally differently
— a hierarchy of cascade levels trading probe density for angular ray
density (`probe_spacing ~ 2^level`, `ray_count ~ 4^level`, `interval ~
2^level`), merged front-to-back (`L_ac = L_ab + β_ab·L_bc`), so far
cascades sample distant geometry directly with wide rays rather than
relaying through adjacent cells. Explicit user decision: implement this
as a second, independently selectable GI technique purely to A/B compare
against DDGI on `gi_room` with real measured numbers — an experiment, not
a committed replacement; DDGI stays the shipping default regardless of
outcome.

New `src/hybrid/radiance_cascades_ref.rs`, mirroring `ddgi_ref.rs`'s own
established shape (grid construction → per-probe ray casting → per-texel
relight → merge, each independently `cargo test`-able before any WGSL is
written). `cascade_level_params` implements the paper's own `2^i`/`4^i`
scaling law with a gapless geometric-series interval tiling formula
(confirmed directly from Sannikov's own paper source at
github.com/Raikiri/RadianceCascadesPaper). `cascade_grid_from_bounds`
mirrors `ddgi_ref::ProbeGrid`'s own cell-center probe placement (the same
real bug that convention was built to avoid — an edge-pinned probe
embedding itself in a sealed room's own boundary geometry — applies
identically here). `cascade_probe_ray` reuses `ddgi_ref`'s own octahedral
direction encoding directly, not re-derived. `relight_cascade_texel`
interval-clips `cpu_ref::trace`/`shade` to `[interval_near, interval_far)`
— a hit closer than `interval_near` is correctly treated as "belongs to a
nearer cascade level," collapsing to the same `transmittance = 1.0`
outcome as a genuine miss, which is exactly the right behavior for the
merge formula. `merge_cascade_texel` implements `L_ac = L_ab + β_ab·L_bc`
verbatim.

**The actual verification target**: a minimal Rust-side corridor fixture
mirroring `gi_room.rs`'s own sealed-corridor-with-roof-gap geometry (a
long corridor, ceiling open only over the near half, sun light shining
straight down through the gap, test point on the floor at the far, dark
end) run through the full cascade level/relight/merge pipeline. Confirmed
real, nonzero light reaches the dark corridor floor through the cascade
hierarchy — a genuine early data point suggesting the technique's own
structural mechanism (wide far-field rays sampling distant geometry
directly) does what DDGI's hop-based relay doesn't, though this is only
the CPU-reference stage; the real comparison happens in Stage 4 on the
actual `gi_room` scene.

**Two real bugs found and fixed during this stage's own test development
— both in the TEST's own sampling methodology, not the cascade formulas
themselves, each correctly diagnosed via debug instrumentation before
being fixed rather than assumed**: (1) an early version of the
bug-reproduction test sampled only the first 64 raw ray indices out of up
to 2048 per level; since ray index maps to a 2D octahedral tile position,
the first 64 indices all cluster in one small tile corner, a
non-representative angular slice that happened to never point toward the
roof gap — produced a false "cascades don't reach the corridor" result.
Fixed by scanning each level's own FULL ray set, matching the real GPU
implementation's eventual "every atlas texel gets a real relit answer"
guarantee (the cheap-read/gather-at-shading-time concern is deferred to
Stage 2, exactly like DDGI's own atlas). (2) even after that fix, a flat
uniform average over ALL directions from one probe (most pointing at
solid walls or empty space) legitimately produces a small-magnitude
result for a single narrow lit gap — confirmed via direct instrumentation
this is a real, expected consequence of the test's own crude
all-directions-averaged sampling, not a sign the technique barely works,
so the test's own pass threshold was set to "genuinely nonzero,
distinguishable from exact-zero" rather than an arbitrary larger bar. A
dedicated fixture-sanity test
(`the_open_gap_floor_is_genuinely_lit_by_the_overhead_sun`) was added
alongside it so a future failure of the real corridor test can be
correctly attributed to either a broken fixture or a genuine
cascade-propagation regression, not conflated.

### Radiance Cascades experimental GI, Stage 2: WGSL relight pass + probe atlas buffers

Ports Stage 1's own CPU reference (`radiance_cascades_ref.rs`) to a real
GPU dispatch, mirroring DDGI's own WGSL wiring shape (self-contained pass
file, own bind group, own pipeline, gated dispatch in `hybrid_pass`) —
selectable live via `GiMethod::RadianceCascades` (discriminant `2`, the
previously-reserved/unused slot) or `gallery.rs --gi-method cascades` /
its own new "Radiance Cascades (experimental)" radio button.

New `assets/shaders/hybrid_radiance_cascades.wgsl`
(`radiance_cascades_relight_main`): relights ALL 4 fixed cascade levels
(`RADIANCE_CASCADES_LEVEL_COUNT`) in ONE dispatch, `gid.z` selecting the
level, `gid.xy` covering level 0's own (largest) atlas — every probe of
every level relit fresh each frame, no rotating subset (unlike DDGI) and
no temporal accumulation (an explicit scope cut: this experimental
technique has no history buffer yet). Scope decision: the MERGE step
(`merge_cascade_texel`'s `L_ac = L_ab + β_ab·L_bc`) is NOT its own compute
pass — with only 4 small levels, walking the hierarchy at SHADING time
(one nearest-probe atlas read per level, 4 folds) is cheaper than an extra
full-screen dispatch, so relit texels are stored as `vec4(radiance.rgb,
transmittance)` and merged in `hybrid_trace.wgsl`'s own new
`radiance_cascades_sample_hierarchy` (farthest-to-nearest, matching the
Rust reference's own test loop order) inside a new
`GI_METHOD_RADIANCE_CASCADES` `shade()` branch. Nearest-probe-only
sampling (no trilinear blend across 8 probes, no Chebyshev visibility
test) — a deliberately coarser read than DDGI's own `ddgi_sample_probe_grid`,
consistent with this whole experiment's smaller scope.

Rust side: `CascadeLevelUniform` (pipeline.rs) mirrors the WGSL struct
field-for-field (including `vec3` padding); `RadianceCascadesConfig`
(extract.rs, new resource) holds level-0 `base_*` scalars, default values
taken from the Stage 1 test fixtures rather than fresh guesses;
`prepare_hybrid_radiance_cascades` builds all 4 `CascadeLevelUniform`
entries every frame from `cascade_level_params`/`cascade_grid_from_bounds`
and lays each level's own exact-fit atlas region out consecutively in ONE
shared buffer (`HybridRadianceCascadesAtlas`, reallocated only when total
texel count changes). Dispatch gated in `hybrid_pass` (pass.rs) on
`GiMethod::RadianceCascades`, its own `RecordDiagnostics::pass_span("hybrid_radiance_cascades")`.

**One real bug found via live GPU dispatch (not caught by `cargo build`,
as expected — WGSL only actually compiles when Bevy's pipeline cache
processes it), diagnosed and fixed the same session**: `storage_buffer_read_only::<CascadeLevelUniform>(false)`
sized the `levels` binding's `min_binding_size` from ONE element (64
bytes), but the shader declares a FIXED-size `array<CascadeLevelUniform, 4>`
(256 bytes) — WGPU's pipeline-layout validation rejected the shader at
`Device::create_compute_pipeline` with "Buffer structure size 256 ...
ended up greater than the given min_binding_size, which is 64." Fixed
with `storage_buffer_read_only_sized(false, Some(4 * element_size))` on
both the relight pass's own bind group and `hybrid_trace.wgsl`'s new
group-3 read-only view into the same atlas. Confirmed via a real
`cargo run --release --example gallery -- --gi-method cascades` session
(AMD Radeon Graphics RADV RENOIR, Vulkan): pipeline compiles clean, runs
stably for 15+ seconds with no validation errors, "hybrid pass: trace
pipeline READY" logged.

**Correction (found during independent re-verification of this stage,
same session): the first-pass "GPU frame time rose from ~7ms to ~35ms"
cost claim above was wrong** — it conflated this scene's own steady-state
frame cost on this integrated GPU (AMD Renoir) with the cascades pass's
own cost. The `hybrid_radiance_cascades` pass itself was never wired into
`gallery.rs`'s own `FpsStats`/HUD readout in this stage's first pass (a
real gap, since fixed — see below), so the ~35ms figure was an eyeballed
total frame time, not an isolated per-pass number, and was never compared
against a `GiMethod::None` baseline to confirm GI was even the cause.
Re-measured properly after wiring `gpu_pass_ms(diagnostics,
"hybrid_radiance_cascades")` into `FpsStats`/its `Display` impl (mirroring
`gpu_ddgi_ms`'s own existing pattern): on `gallery.rs`'s default scene,
`--gi-method none` (no GI dispatch at all) already runs ~33-42ms frame
time / ~9-11ms `gpu trace`; `--gi-method ddgi` runs ~36-38ms / ~12.4ms
`gpu trace` (`ddgi` pass itself 0.06ms); `--gi-method cascades` runs
~36-46ms / ~10.5-11.5ms `gpu trace` (`radiance-cascades` pass itself only
**0.02-0.03ms**). All three are within the same ~33-46ms band — this
scene's frame cost is dominated by something unrelated to GI method
entirely (likely vsync/present/egui/window-compositor overhead on this
weak iGPU, not investigated further here since it's out of this stage's
scope), and the cascades relight pass itself is, if anything, *cheaper*
than DDGI's own relight pass at this scene's small size — not 5x more
expensive as first reported. This does NOT mean cascades is free at
scale: this stage's own "relight everything every frame, no rotating
subset, no temporal accumulation" design is still real and still the
right thing to revisit before any larger scene (per the "NOT done"
list below) — it just means the concrete ~7ms→~35ms number originally
reported here was not a real measurement of that cost and should not be
cited. DDGI itself re-confirmed unaffected by this stage's own changes
(separate live run, `--gi-method ddgi`, no errors, `ddgi 0.06 ms` GPU
pass time unchanged from its own established baseline).

**Second real bug found via independent re-verification (not the
implementing fork's own pass): `gallery.rs`'s `FpsStats`/HUD readout was
never updated to read `hybrid_radiance_cascades`'s own GPU timestamp**,
even though `pass.rs`'s own `RecordDiagnostics::pass_span` call for it
was present and correct — the timing data existed but nothing displayed
it, silently defeating Stage 4's own planned perf comparison (which reads
this exact HUD/log line). Fixed by adding `gpu_radiance_cascades_ms`
alongside the existing `gpu_ddgi_ms` field, mirroring its exact
compute/Display pattern; confirmed live (`radiance-cascades 0.02 ms` now
appears in the log line, see the corrected measurement above).

**Explicitly NOT done in this stage** (deferred, most to Stage 3/4 per the
approved plan): no temporal accumulation on the cascade atlas; no
rotating "relight a subset per frame" schedule; no trilinear/Chebyshev
probe sampling at shading time (nearest-probe only); no real `gi_room.rs`
A/B comparison against DDGI's own known dark-corridor limitation (Stage
1's CPU-only corridor fixture is the only evidence so far this technique
propagates light the way DDGI can't — this stage only proves the GPU port
runs, not that it looks better). `cargo test --release --lib` (341
passed) and `cargo clippy --release --lib --examples` both clean of any
new warnings from this stage's own files.

**Verified**: `cargo test --release --lib` 341/341 passing (14 new tests),
stable across 3+ repeated runs (fully deterministic math, no randomness
anywhere in this stage), zero new `cargo clippy --release --lib`
warnings.

### Radiance Cascades experimental GI, Stage 3: `gi_room.rs` CLI wiring + a real room-scale config bug found live

`GiMethod::RadianceCascades` and `gallery.rs`'s own `--gi-method cascades`
flag/radio button landed early in Stage 2; this stage's own remaining
scope was `gi_room.rs`'s own CLI wiring (`gi_method_config_from_args()`,
copied from `gallery.rs`'s own function) plus a third egui radio button,
needed so this scene's planned DDGI-vs-cascades A/B comparison (Stage 4)
can run headlessly via `--shot`/`--bench` without a human toggling a UI
control.

**A real bug found via live verification, not caught by any test**:
`RadianceCascadesConfig::default()`'s `base_spacing: 11.0` was copied
from `DdgiConfig::default()`'s own GLOBAL default (itself sized for
`--stress N`'s ~11-unit object-cell pitch) — but `gi_room.rs`'s own
`DdgiConfig` insertion already overrides that same global default down to
`probe_spacing: 1.2` for this specific 16x6x13 room (documented in that
override's own comment: at 11.0-unit spacing this room only clears
`MIN_PROBES_PER_AXIS` on every axis, an 8-probe grid "barely any spatial
resolution at all"). `RadianceCascadesConfig` had no equivalent
room-scale override, so cascades' own level-0 grid was hitting that exact
same coarse-8-probe failure mode DDGI had already been tuned away from —
confirmed live: `--gi-method cascades` rendered visually indistinguishable
from `--gi-method none` (no indirect light reaching the camera at all).
Fixed with a `RadianceCascadesConfig { base_spacing: 1.2, base_interval:
1.0, .. }` override mirroring `DdgiConfig`'s own room-scale retuning
(`base_interval` tightened too, so level 0's own near/far window stays
close to nearby geometry — level 3 still reaches 85 world units, far
beyond this room's ~21-unit diagonal, so long-range coverage is
unaffected).

**A second, smaller bug found the same way**: `gi_room.rs`'s own HUD
(separately from `gallery.rs`'s HUD, already fixed in Stage 2) had never
been wired to read the `hybrid_radiance_cascades` pass's own GPU
timestamp either — added a `radiance cascades relight: N.NN ms` HUD line
mirroring the existing `ddgi relight` line.

**Open finding, explicitly deferred to Stage 4, NOT resolved here**:
even after the `base_spacing` fix, a quick visual comparison at the same
camera angle still shows `--gi-method cascades` visually close to
`--gi-method none` in the region checked (the camera's own near corner,
NOT yet the actual roof-gap corridor DDGI's own known bug is about) —
while DDGI clearly shows strong bounce light reaching that same region.
Whether this is (a) nearest-probe sampling being too coarse at this
scale, (b) simply the wrong region to check (the corridor near the roof
gap wasn't directly inspected yet), or (c) a genuine remaining shading
defect, is exactly what Stage 4's own disciplined A/B methodology needs
to determine — not guessed at here. Recorded honestly as an open
question, per this project's own "no silent caps" convention, rather
than either quietly buried or prematurely called a failure.

**A comparison-methodology gotcha also worth recording for Stage 4**:
`gi_room.rs` drives its roof-slide and all timing off wall-clock `Time`,
not a fixed timestep, and DDGI's own per-frame cost (~76ms/frame trace
time on this hardware) is currently much higher than cascades'/none's
(~44-45ms/frame) — so identical `--at-frame N` values land at very
different wall-clock `t` across techniques (confirmed: DDGI reached
`t≈69s` at frame 660, cascades/none only `t≈46-48s`). This did not affect
this stage's own comparisons (the roof fully settles by `t≈11s`, and the
24-32-frame temporal/DoF history windows are far shorter than 660
frames — all three runs were at the same converged steady state
regardless), but Stage 4 should pick `--at-frame` generously past both
the roof-settle point and the slowest technique's own history-convergence
point, not assume matching frame counts imply matching wall-clock scene
states in general.

**Verified**: `cargo build --release --lib --examples` clean, `cargo test
--release --lib` 341/341, `cargo clippy --release --lib --examples` zero
gi_room/cascade-related warnings, live `cargo run --release --example
gi_room -- --gi-method ddgi|cascades|none --shot ... --at-frame 660` runs
for all three methods (no crash, no WGPU validation error), all three
screenshots visually reviewed.

### Radiance Cascades experimental GI, Stage 4: the actual A/B vs. DDGI on `gi_room` — corrected finding: not a bug, a missing feature (no bounce)

The planned comparison this whole experiment was commissioned for (see
this section's own Stage 1 entry's framing: "let's find out with real
numbers, not assumptions"). Both `--gi-method ddgi` and `--gi-method
cascades` screenshotted at the identical fixed camera (`CAMERA_CORNER`)
and a generously settled `--at-frame 1400` (well past both the
roof-slide's own `t≈11s` settle point and every technique's own 24-32-
frame temporal/history window).

**Correction to this entry's own first-pass conclusion (same session):**
the first pass here reported cascades' near-total darkness as an
unresolved GPU correctness bug in `relight_cascade_texel`. Further live
GPU debug instrumentation (temporary, all reverted — `git diff` clean on
both WGSL files afterward) found the REAL explanation: it is not a bug,
it is `relight_cascade_texel` correctly computing zero because there
genuinely is very little DIRECT sunlight reaching the camera's own
visible region at this shallow sun angle (`gi_room.rs`'s own sun sits
~35° above the horizon, raking in mostly along -X) — confirmed by
comparing against `--gi-method none` (direct-light-only, via the main,
already-proven-correct `hybrid_trace.wgsl` shading path, not cascades'
own duplicate): `None` shows the EXACT SAME dark region. DDGI's own light
in this view is therefore almost entirely INDIRECT (multi-bounce),
supplied by its own "infinite bounce" self-referential grid-sampling
trick (`ddgi_ref.rs::probe_ray`'s `indirect_at_hit` term — see the
"DDGI infinite-bounce" entries elsewhere in this file). Radiance
Cascades' `relight_cascade_texel`, by this experiment's own documented
Stage 1 scope cut, calls `shade_direct_only` — DIRECT LIGHT ONLY, no
indirect/bounce term of any kind. Comparing the two on a scene whose
visible light is mostly indirect was never a fair fight: cascades was
missing an entire feature DDGI has, not broken.

Root-cause path (kept for the record, since it took real, non-obvious
debugging to get here): live sentinel/diagnostic-color writes ruled out
the `CascadeLevelUniform` bind group, the atlas write→read buffer
plumbing, and (mostly) `base_interval` tuning (still correctly raised to
`3.0` from an initially-too-short `1.0`, a real kept fix) before finally
isolating the zero-radiance texels to `shade_direct_only`'s own shadow-
ray branch reporting `shadow_vis=0` almost everywhere in the visible
region — which turned out to be CORRECT occlusion at this shallow sun
angle, not a bug in the occlusion test itself.

**Performance**: cascades' own relight pass costs 2-3ms either way vs.
DDGI's, comparable; DDGI's own `trace` cost (~77ms) is much higher than
cascades' (~45ms) at `--at-frame 1400`, presumably from DDGI's own
richer per-pixel sampling (trilinear + 8-probe Chebyshev visibility) vs.
cascades' cheap nearest-probe read — but this is STILL not a fair
apples-to-apples number, now for a different reason than first thought:
a technique with no indirect term at all will always be cheaper than one
that also computes multi-bounce light, independent of any real
"cascades is faster GI" claim.

**Corrected verdict**: this Stage 4 comparison, as run, does not tell us
whether Radiance Cascades' own STRUCTURAL fix for DDGI's known
dark-corridor problem (far cascades sampling distant geometry directly,
vs. DDGI's hop-based relay) actually works — because cascades has no
bounce/indirect term yet to test that claim against. The user's own
direct follow-up ("compare with more than 1 bounce, like 4-5") is
exactly the missing piece: adding an `indirect_at_hit`-style recursive
term to `relight_cascade_texel` (mirroring `ddgi_ref.rs::probe_ray`'s own
mechanism) is required before a meaningful DDGI-vs-cascades comparison on
this scene's own indirect-light-dominated regions is possible. See the
next entry for that work.

**Verified**: `cargo build --release --lib --examples` clean, `cargo test
--release --lib` 341/341, `cargo clippy --release --lib --examples`
clean, all temporary debug WGSL edits confirmed fully reverted (`git
diff` clean) before this corrected entry was written.

### Radiance Cascades experimental GI, Stage 5: multi-bounce indirect recursion (`indirect_at_hit`, N-pass dispatch)

Directly follows the previous entry's own conclusion: `relight_cascade_texel`
had no indirect/bounce term at all, so the Stage 4 A/B against DDGI's own
"infinite bounce" was never comparable. This stage adds one.

**CPU reference** (`radiance_cascades_ref.rs`): `relight_cascade_texel`
gained an `indirect_at_hit: impl Fn(Vec3, Vec3) -> Vec3` parameter,
mirroring `ddgi_ref.rs::probe_ray`'s own identically-named/-shaped
parameter exactly — on a real hit, `diffuse_color *
indirect_at_hit(hit_point, hit_normal)` is added on top of direct light,
the SAME "raw irradiance in, albedo-multiplied here" convention
`probe_ray` already established. `|_, _| Vec3::ZERO` recovers the exact
prior direct-light-only behavior bit for bit — all 5 existing call sites
(1 real, 4 tests) updated to pass this and confirmed unchanged (14/14
still passing). Two new tests added, mirroring `ddgi_ref.rs`'s own
`probe_ray_ignores_indirect_at_hit_entirely_on_a_miss`/`feeding_a_probes_
own_relit_neighbor_irradiance_back_in_lifts_a_shadowed_probes_own_result_
above_direct_light_alone` pair: a miss ignores the closure entirely
(black regardless of what it would return), and a new `shadowed_floor_
scene` fixture (a floor patch shadowed from direct sun by a canopy box
overhead) proves feeding a nonzero `indirect_at_hit` sample in raises the
relit result strictly above the direct-only case — the actual bounce
claim, exercised end to end. 343/343 tests passing (341 + 2 new).

**WGSL port** (`hybrid_radiance_cascades.wgsl`): ported
`radiance_cascades_nearest_probe_irradiance`/`merge_cascade_texel`/
`radiance_cascades_sample_hierarchy` from `hybrid_trace.wgsl` into this
file too (as `cascade_nearest_probe_irradiance`/`cascade_merge_texel`/
`cascade_sample_hierarchy_at_hit`, per this codebase's own
self-containment convention — no shared imports) so the RELIGHT pass
itself can sample the cascade hierarchy's own current state at a hit
point, not just shading time. `relight_cascade_texel` now calls
`cascade_sample_hierarchy_at_hit(p_world, n)` on every real hit and adds
`diffuse_color * that` to direct radiance, mirroring
`hybrid_ddgi_relight.wgsl`'s own `probe_ray`/`ddgi_sample_probe_grid_diffuse`
port of the identical Rust-side mechanism.

**The temporal-accumulation gap, and the fix (multi-pass dispatch, not
cross-frame history)**: DDGI's own bounce mechanism works because its
atlas persists and blends across MANY frames (EMA history) — a probe's
relight this frame reads mostly-converged prior-frame state. Cascades
has explicitly NO temporal accumulation (relights everything fresh every
frame, an existing documented scope cut) — so a single relight dispatch's
own bounce read only samples whatever THIS SAME dispatch already wrote
earlier this same frame, a GPU-scheduling-order-dependent, inconsistent
single (at best) bounce, not the "4-5 bounces" the user asked to compare.
Fixed with a NEW `RadianceCascadesConfig::bounce_passes: u32` (default
`1`, preserving prior behavior exactly): `pass.rs`'s own dispatch site
now loops, re-issuing the SAME relight pass `bounce_passes` times per
frame, each a genuinely separate `begin_compute_pass` call — WGPU/Vulkan's
own pass-boundary-as-barrier guarantee means pass N+1's atlas reads see
pass N's fully-written atlas, giving a real, explicit N-bounce depth per
frame (pass 1 = pure direct light, pass 2 = one real bounce, pass 3 = two
real bounces, ...) instead of DDGI's cross-frame accumulation. `gi_room.rs`
gained a `--cascade-bounces N` debug flag (same shape as its own existing
`--cone-bounces`) for this comparison.

**Live verification**: `cargo run --release --example gallery --
--gi-method cascades` (no validation error, `radiance-cascades 0.06 ms`,
screenshot visually correct) and `cargo run --release --example gi_room
-- --gi-method cascades --cascade-bounces 1|5 --shot ... --at-frame 660`
(both ran cleanly, no crash). Comparing the two screenshots: bounce_passes
5 shows a real, if modest, brightening over bounce_passes 1 at the
compared camera view (a previously flat, featureless dark region gains a
faint but distinguishable gradient/cube-edge definition) — genuinely
different, not a no-op, confirming the multi-pass mechanism works. The
difference is smaller than DDGI's own strong illumination in this same
view; this is plausibly correct physics (this camera's own view has very
little direct light to begin with — most of this room's-own energy
budget is elsewhere — so there is little energy available to bounce
regardless of pass count), not necessarily a remaining defect, but this
has NOT been independently confirmed against the CPU reference on the
real scene geometry (out of this stage's own scope; flagged honestly as
an open question, not asserted as settled).

**Verified**: `cargo build --release --lib --examples` clean, `cargo test
--release --lib` 343/343, `cargo clippy --release --lib --examples`
clean (only pre-existing, unrelated warnings elsewhere), live GPU runs on
both `gallery.rs` and `gi_room.rs` at `bounce_passes` 1 and 5 with no
crash/validation error, screenshots visually reviewed.

### Radiance Cascades experimental GI, Stage 6: bounce light was point-sampled along the surface normal instead of gathered over the hemisphere — the real reason Stage 5's own brightening looked so modest

Directly resolves Stage 5's own open question ("this has NOT been
independently confirmed... flagged honestly as an open question").
User-reported symptom from a live `--corridor-cam` look at the Stage 5
build: cascades' bounce light visibly propagated in only one direction
instead of dispersing across neighboring surfaces, a "line" artifact —
not the subtle-but-plausible energy-budget explanation Stage 5's own
write-up had guessed at.

**Root cause**: both of Stage 5's own hierarchy-sampling call sites —
`hybrid_radiance_cascades.wgsl::relight_cascade_texel`'s own
`indirect_at_hit` term and `hybrid_trace.wgsl::shade`'s own
`GI_METHOD_RADIANCE_CASCADES` shading-time branch — called
`cascade_sample_hierarchy_at_hit`/`radiance_cascades_sample_hierarchy`
with `direction = n` (the bare surface normal): a SINGLE point-sample of
whichever one discrete cascade ray direction happens to align with that
exact normal. Correct for a shading-time VIEW ray, but wrong for a
diffuse bounce — Lambertian indirect diffuse is an integral of incoming
radiance over the WHOLE hemisphere around the normal, not one direction.
DDGI never had this bug: `ddgi_sample_probe_grid`/
`ddgi_sample_probe_grid_diffuse` both route through
`ddgi_cosine_weighted_probe_irradiance`, which averages
`ddgi_probe_irradiance` (DDGI's own single-direction point-sampler,
architecturally identical to `cascade_nearest_probe_irradiance`) over
`HEMISPHERE_SAMPLES` — cascades' relight/shade paths never grew the
equivalent wrapper.

**Fix**: added `cascade_cosine_weighted_hierarchy_at_hit`
(`hybrid_radiance_cascades.wgsl`) and
`radiance_cascades_cosine_weighted_hierarchy` (`hybrid_trace.wgsl`),
each mirroring `ddgi_cosine_weighted_probe_irradiance` exactly — a
tangent basis around the surface normal (`cascade_tangent_basis`, a
verbatim duplicate of `ddgi_tangent_basis` per this pass's own
self-containment convention; `hybrid_trace.wgsl` instead reuses its own
already-shared `ddgi_tangent_basis`/`HEMISPHERE_SAMPLES` bundle, kept
generic since DDGI's own removal) times the same 5-sample cosine-weighted
`HEMISPHERE_SAMPLES` set DDGI uses, averaging N calls to the existing
single-direction sampler instead of adding a new one. Both call sites
(`relight_cascade_texel`'s bounce term, `shade`'s
`GI_METHOD_RADIANCE_CASCADES` branch) now call the hemisphere-gather
wrapper instead of the bare single-direction function. No CPU reference
exists for the hierarchy-sampling functions on either side (Stage 2's own
`cascade_sample_hierarchy_at_hit`/`radiance_cascades_sample_hierarchy`
were WGSL-only from the start, no Rust twin) — this fix follows that same
established precedent rather than introducing a CPU reference
retroactively.

**Live verification**: `cargo run --release --example gi_room --
--gi-method cascades --corridor-cam --cascade-bounces 5 --shot ...
--at-frame 660`, screenshot compared against the identical command
pre-fix and against `--gi-method ddgi --corridor-cam` at the same camera.
Pre-fix: cube faces and floor near the shadowed cube cluster are flat
black, no dispersed bounce. Post-fix: the same surfaces show real
colored bounce light (visible green/purple cube-face bounce, floor
picking up ambient fill) — qualitatively converged toward DDGI's own
result at the same view, no longer a directional-artifact "line".

**Verified**: `cargo build --release --lib --examples` clean, `cargo test
--release --lib` 342/343 (the one failure,
`physics::gpu::frame_test::gpu_path_settles_to_a_stable_resting_position`,
passes in isolation — pre-existing GPU-pipeline-contention flakiness
under `--lib`'s parallel execution, unrelated to this stage's shader-only
change), `cargo clippy --release --lib --examples` clean (only
pre-existing, unrelated warnings elsewhere), live GPU run on `gi_room.rs`
with `--corridor-cam` at both `--gi-method cascades` and `--gi-method
ddgi`, no crash/validation error, screenshots visually reviewed.

### Radiance Cascades experimental GI, Stage 7: the actual Stage 4 DDGI-vs-cascades A/B, re-run now that cascades has both a real bounce mechanism (Stage 5) and correct hemisphere dispersion (Stage 6)

The re-run Stage 5's own write-up called for: `cargo run --release
--example gi_room -- --gi-method {cascades --cascade-bounces 5,ddgi}
--corridor-cam --shot ... --at-frame 900` — a matched camera/frame pair,
roof fully open in both (`ROOF_DELAY_SECS` + `ROOF_SLIDE_SECS` = 11s;
frame 900 lands at `t=76.2s` cascades / `t=107.3s` DDGI, both far past
that), targeting the actual documented dark-floor-corridor region
(`--corridor-cam`, the strip between the -X wall at `x=-8.0` and the
cube row at `x~=-6.5..-6.8` — see this file's own "Investigated (not
fixed): residual dark floor corridor after DDGI infinite-bounce" entry
for why this exact region is the real bug both techniques are being
measured against, not just an arbitrary camera angle).

**Geometry check before trusting the comparison**: confirmed directly
against `gi_room.rs::spawn_cubes`/`slide_roof` that the corridor is NOT
partial-length the way the original DDGI-era investigation described —
`RoofPanel` slides to expose the ENTIRE -X half of the ceiling (full
`ROOM_HALF_Z` span, not a partial gap), and all 7 cubes cluster at
`x~=-6.5..-6.8` across all 4 Z-zones (`-4.5` to `4.8`), so the corridor
this experiment targets runs the room's full depth and sits fully under
the open roof — the "gap only opens over part of the corridor" caveat
from the old DDGI entry doesn't apply to this comparison as currently
framed.

**Result**: cascades (post-Stage-6) shows a REAL, qualitatively
significant improvement over its own pre-Stage-6 state in this region —
the floor strip and nearby cube faces that were previously near-solid
black now carry visible, correctly-colored bounce light. Compared
directly against DDGI at the same matched view, however, DDGI is still
visibly brighter in the same region: most notably the large shadowed
cube face just right of camera-center is close to solid black under
cascades but shows a soft, clearly nonzero gradient under DDGI. This
was assessed by direct visual comparison of the two screenshots (no
pixel-value tooling available in this environment — no `python3` on
`PATH`, and the Python toolchain declared in `flake.nix` is scoped to
the unrelated `gothic_export` sub-shell, not the main dev shell), same
standard this file's other GI-comparison entries already use.

**Interpretation, NOT yet independently confirmed**: this remaining gap
is architecturally explainable without positing a new bug — DDGI's own
`ddgi_sample_probe_grid_diffuse`/`ddgi_cosine_weighted_probe_irradiance`
additionally does TRILINEAR blending across the 8 surrounding probes
AND a Chebyshev soft-visibility term per probe (see
`hybrid_ddgi_relight.wgsl:928-948`), whereas Stage 6's
`cascade_cosine_weighted_hierarchy_at_hit` only fixed the ANGULAR
(single-direction to hemisphere) gap — it still calls
`cascade_nearest_probe_irradiance`, which remains NEAREST-probe-only
with no spatial (trilinear) blend, per that function's own existing doc
comment. A nearest-only spatial query is a coarser reconstruction than
DDGI's blended one regardless of angular correctness, and would plausibly
read as "real but dimmer," which is exactly what was observed. Flagged
honestly as the most likely explanation, not proven — no isolated
CPU-ref test was built this stage to confirm the spatial (as opposed to
angular) gap specifically accounts for the remaining brightness
difference.

**How to resume, if closing this further is wanted**: the next
concrete, scoped step would be a trilinear spatial blend across
`cascade_nearest_probe_irradiance`'s neighboring probes (mirroring
`ddgi_sample_probe_grid_diffuse`'s own 8-probe trilinear loop), proven
first as a CPU-ref addition to `radiance_cascades_ref.rs` (this project's
established CPU-reference-first convention) before any WGSL — NOT
assumed to be needed without first isolating whether angular vs. spatial
is really the dominant remaining term. This is an explicit, scoped
experiment (DDGI remains the shipping default regardless of outcome —
see this section's own Stage 1 framing) — closing the full gap is not a
requirement for the experiment to be considered complete or useful.

**Verified**: two live GPU runs (`gi_room.rs`, `--gi-method cascades
--cascade-bounces 5 --corridor-cam` and `--gi-method ddgi
--corridor-cam`, both `--at-frame 900`), no crash/validation error,
screenshots visually compared side by side.

### Radiance Cascades experimental GI, Stage 8: trilinear spatial blend across cascade probes (Stage 7's own leading, then-unconfirmed explanation, now built and CPU-ref-proven)

Directly acts on Stage 7's own leading explanation for cascades still
reading dimmer than DDGI after the Stage 6 hemisphere/angular fix: every
hierarchy sample still snapped to its single NEAREST probe per level
(`cascade_nearest_probe_irradiance`/`radiance_cascades_nearest_probe_
irradiance`), unlike DDGI's own `ddgi_sample_probe_grid_diffuse`, which
additionally trilinearly blends across the 8 probes surrounding a query
point. This stage adds that missing spatial blend, following the
project's CPU-reference-first convention this time (unlike Stage 2's own
WGSL-only precedent for this function family, called out explicitly as
a deviation there).

**CPU reference** (`radiance_cascades_ref.rs`): added
`cascade_probe_grid_cell` (`ddgi_ref::probe_grid_cell` verbatim, adapted
to `CascadeGrid`'s own uniform-`f32`-spacing field shape rather than
`ProbeGrid`'s per-axis `Vec3` spacing) and `cascade_sample_level_
trilinear` (`ddgi_ref::sample_probe_grid`'s own 8-probe dx/dy/dz
trilinear loop, minus the hemisphere/occlusion machinery that function
also does — that layering already lives one level up, in Stage 6's own
`cascade_cosine_weighted_hierarchy_at_hit`, which now calls this once
per hemisphere sample instead of the old nearest-probe function). Both
are generic over a `probe_irradiance: impl Fn(UVec3, vec3) -> Vec4`
closure (radiance + transmittance packed together, matching the real
atlas texel's own `vec4` shape), so they're fully testable without a
real GPU atlas — same pattern `sample_probe_grid`'s own tests already
established for DDGI. Four new tests: grid-cell cell/fraction math
(`cascade_probe_grid_cell_finds_the_correct_lower_corner_and_fraction`,
`..._clamps_a_point_outside_the_grid_to_the_nearest_valid_cell`, both
`ddgi_ref`'s own equivalents ported verbatim), a sample exactly AT a
probe's own position reproducing that probe's own value
(`trilinear_sample_at_a_probes_own_exact_position_...`, ditto), and the
actual new claim — a query point exactly midway between two probes
blends them 50/50, on BOTH the radiance and transmittance channels
(`trilinear_sample_exactly_between_two_probes_averages_them_evenly`,
`trilinear_sample_blends_transmittance_the_same_way_as_radiance`) — the
concrete "no longer snaps entirely to one probe" behavior this whole
stage is about. 21/21 in this file (18 existing + 3 new — the fourth new
test above is counted in that 21; see the file's own test names for the
exact set), 348/348 in the full suite.

**WGSL port**: in BOTH `hybrid_radiance_cascades.wgsl` (the relight
pass's own bounce term) and `hybrid_trace.wgsl` (the shading-time view
sample) — split the old combined "world-pos → nearest-cell → texel-read"
function into a coords-only texel reader
(`cascade_probe_irradiance_at`/`radiance_cascades_probe_irradiance_at`,
takes an already-resolved probe's own integer grid coords) plus a new
`cascade_sample_level_trilinear`/`radiance_cascades_sample_level_
trilinear` (`cascade_probe_grid_cell`'s own cell/frac math ported and
inlined, per this file's own "no tuple return type" WGSL convention,
rather than a matching standalone cell-finder function) that calls the
coords-only reader 8 times per level and trilinearly weights the
results — verbatim mirrors of the CPU reference's own two new functions.
Both existing call sites (`cascade_sample_hierarchy_at_hit`'s hierarchy
walk in the relight pass, `radiance_cascades_sample_hierarchy`'s
hierarchy walk at shading time — the latter already reused by Stage 6's
own `radiance_cascades_cosine_weighted_hierarchy`, so this fix reaches
the bounce path too without a third call-site edit) now call the
trilinear function instead of the nearest-probe one.

**Live verification**: `cargo run --release --example gi_room --
--gi-method cascades --corridor-cam --cascade-bounces 5 --shot ...
--at-frame 900`, screenshot compared against both the Stage 7 (angular-
fix-only) result and DDGI at the identical matched frame/camera. Versus
Stage 7: the large shadowed cube face just right of camera-center, which
was still close to solid black after the angular-only fix, now shows a
visible dark-purple bounce tint — a real, if modest, further
improvement, at the cost of visibly more grain/noise (expected: 8
neighbor probes × 5 hemisphere samples × 4 levels per hierarchy sample,
up from 1 × 5 × 4) and roughly 2.7x relight cost on this scene (`8.87
ms` vs. the Stage 6/7 baseline's `~3.3 ms`, both read from the egui HUD's
own live timing readout). Versus DDGI at the same matched frame: DDGI's
own version of that same cube face is still visibly smoother and
brighter — a real, still-open gap, smaller than before Stage 8 but not
closed. `gallery.rs --gi-method cascades` also re-checked (its own
smaller single-object scene): `radiance-cascades 0.15 ms`, no
crash/validation error, screenshot visually unchanged from before this
stage (that scene has no comparable dark-corridor region for the fix to
visibly act on).

**Open, NOT pursued further this stage**: DDGI's own trilinear gather
also combines a Chebyshev depth-visibility term per probe
(`chebyshev_visibility_weight`, `hybrid_ddgi_relight.wgsl:936-947`) that
cascades' new trilinear blend does NOT have — a probe on the wrong side
of a thin occluder can still contribute weight here. Not built this
stage: unclear yet whether it materially matters for cascades specifically
(DDGI needed it because its own probes sit much farther apart per axis
than a cascade level's finer-spaced near levels do), and per this
section's own Stage 1 framing, this remains a scoped, non-shipping
experiment — pursue only if a future comparison shows a concrete
artifact traceable to its absence, not preemptively.

**Verified**: `cargo build --release --lib --examples` clean, `cargo
test --release --lib` 348/348 (the Stage 6/7 write-up's own flaky
`gpu_path_settles_to_a_stable_resting_position` failure did NOT
reproduce this run — confirms it was transient GPU-pipeline contention,
not a real regression), `cargo clippy --release --lib --examples` clean
(only pre-existing, unrelated warnings elsewhere), live GPU runs on both
`gi_room.rs` (`--corridor-cam`, cascades vs. DDGI at matched frame 900)
and `gallery.rs` (`--gi-method cascades`), no crash/validation error,
screenshots visually reviewed.

### DDGI: fixed a real sealed-room light leak — `hybrid_trace.wgsl`'s own shading-time probe-grid-cell lookup used only the grid's X-axis origin/spacing for ALL THREE axes

Directly prompted by the Stage 7 A/B screenshots: `gi_room` is a fully
sealed, sun-only room for its first `ROOF_DELAY_SECS = 5.0` seconds, and
a live look at `--gi-method ddgi` during that window showed the interior
already brightly, evenly lit — `--gi-method none` (direct light only) at
the identical frame correctly rendered near-black, isolating this to
DDGI's own indirect term specifically, not a scene-lighting bug.

**Investigation (extensive, two dead ends before the real bug, kept as
permanent regression coverage rather than discarded):**

1. **Self-referential feedback-loop theory** (DDGI's own "infinite
   bounce" mechanism reading its own atlas mid-write, an accepted,
   documented race) — investigated at length, initially looked plausible
   (a real, if narrow, occlusion-blind fallback path exists in
   `ddgi_sample_probe_grid[_diffuse]`, see `sample_probe_grid`'s own doc
   comment on `FALLBACK_RAMP_WEIGHT`). RULED OUT by a new CPU-ref test,
   `ddgi_ref::sealed_room_ddgi_grid_stays_dark_across_many_relight_
   frames_from_zero_history`: a real 20-frame simulation of the ACTUAL
   relight+temporal-blend loop (not a single relight call), on the real
   sealed-room geometry, starting from true zero history — stays at
   EXACT zero every frame. The algorithm itself has no feedback-loop bug.
2. **Thin-slab shadow-march divergence theory** (`trace_shadow`'s own
   `h > best_h * DIVERGENCE_FACTOR` break exiting before the march ever
   registers real occlusion against the roof panel's own 0.3-half-extent
   thinness at grazing sun angles) — also investigated at length, a
   first attempt appeared to confirm it (`vis=1.0` through a solid
   panel) but that result turned out to be a bug in the TEST itself (an
   `origin_entity` mistakenly set to the panel's own entity, making
   `trace_shadow` skip it as a candidate). Fixed and RULED OUT by three
   new `cpu_ref` tests — an isolated thin-slab spot check
   (`thin_roof_slab_at_grazing_sun_angle_is_never_a_soft_leak`), a dense
   X-offset sweep under that same slab
   (`no_sample_under_the_thin_roof_slab_leaks_light_at_any_x_offset`),
   and a full 3D grid sweep (512 points) against the REAL multi-panel
   `gi_room` shell, corners and seams included
   (`no_shadow_ray_anywhere_in_the_sealed_room_leaks_light_through_any_
   seam`) — all pass. The shadow march is fully sound everywhere tested.

**The real bug**: with both the DDGI algorithm and the shadow march
proven correct via CPU reference, the remaining candidate was a WGSL
port divergence. Found by line-by-line comparison against
`ddgi_ref::probe_grid_cell` (proven correct by the simulation above):
`hybrid_trace.wgsl::ddgi_probe_grid_cell` (the SHADING-TIME probe-cell
lookup, called from `shade()`'s own `GI_METHOD_DDGI` branch — NOT the
separate, correctly-written copy in `hybrid_ddgi_relight.wgsl`, which
uses this file's own vec3-typed `ddgi_grid.origin`/`ddgi_grid.spacing`
uniform fields and was never affected) computed:
```
let local = (world_pos - ddgi_grid.origin_x) / ddgi_grid.spacing_x;
```
— `ddgi_grid.origin_x`/`ddgi_grid.spacing_x` are SCALAR `f32` fields
(this file's own `DdgiGridUniform` struct has separate `origin_x/y/z`/
`spacing_x/y/z` scalars, not vec3s, unlike the relight pass's own
uniform layout). WGSL broadcasts a `vec3 - scalar` / `vec3 / scalar`
across all 3 components, so this computed `local.y`/`local.z` using the
X-axis origin/spacing, not the real per-axis values — meaning every
shading-time DDGI probe-grid sample (the thing that actually paints
pixels) read from a WRONG, effectively arbitrary grid cell whenever
`spacing_y`/`spacing_z`/`origin_y`/`origin_z` differ from their X-axis
counterparts, which they always do (`gi_room`'s own vertical spacing is
independently derived — `probe_grid_from_bounds`'s own `spacing_y =
extent.y / vertical_layers`). Worse: this same function's OWN caller
(`ddgi_sample_probe_grid`) separately recomputed `frac` correctly
per-axis a few lines below — so `cell` and `frac` used two DIFFERENT,
mutually-inconsistent local-space computations for the same trilinear
interpolation, not just one wrong value consistently applied.

**Fix**: `hybrid_trace.wgsl::ddgi_probe_grid_cell` now builds real
`vec3<f32>` `grid_origin`/`grid_spacing` from the uniform's own
per-axis scalar fields first, matching `ddgi_ref::probe_grid_cell`
exactly (verified by direct comparison, not just by the fix compiling).

**A false alarm along the way, worth recording so it isn't re-walked**:
immediately after this fix, a live A/B against a pre-fix screenshot
appeared to show a real regression — previously vivid purple/green
cubes reading washed-out and desaturated. This was NOT a second bug: a
leftover TEMP DEBUG line from this same investigation
(`indirect = irradiance * 5.0`, added to visually amplify raw DDGI
irradiance for inspection, bypassing `diffuse_color` entirely) was still
active in `shade()`'s own `GI_METHOD_DDGI` branch during that A/B.
Removing it reproduced the original, correctly-saturated result exactly
— always re-check for your own leftover debug scaffolding before
trusting a "the fix broke something" read.

**Live verification**: `gi_room --gi-method ddgi` at `t≈5.2s` (roof
still ~97% sealed) now renders correctly near-black, matching
`--gi-method none`'s own baseline at the same frame — no more
even, bright, ceiling-to-floor illumination. `--corridor-cam --at-frame
900` (roof open, the Stage 7 A/B's own matched view) re-checked
post-fix: visually identical to the pre-fix result (vivid purple/green
cubes, same brightness/gradient) — confirms the fix has NO visible
effect once the grid's per-axis spacing/origin values happen to line up
closely enough with reality for the open-room comparison view specifically
(this room's real `spacing_y ≈ 1.1` vs `spacing_x = 1.2` are close
enough that the bug's effect was small away from cell boundaries/room
edges — the sealed-room case is where it became obviously wrong,
not the only place it was ever wrong). `gallery.rs --gi-method ddgi`
also re-checked: clean render, no regression.

**Verified**: `cargo build --release --lib --examples` clean, `cargo
test --release --lib` 353/353 (348 + 5 new: 2 feedback-loop-investigation
tests, 3 shadow-march-investigation tests — all now passing regression
coverage, even though neither hypothesis was the real bug), `cargo
clippy --release --lib --examples` clean (only pre-existing, unrelated
warnings elsewhere), live GPU runs on `gi_room.rs` (sealed-room
before/after, `--corridor-cam` open-room before/after) and `gallery.rs`,
no crash/validation error, screenshots visually reviewed at every step.

### `--gi-method none`: reflection/refraction bounce shading always ran cone-traced GI regardless of the chosen primary GI method — fixed with a new `bounce_gi_enabled` gate

Follow-up to the DDGI sealed-room fix above: after that fix landed, a
live `--gi-method none` render of the sealed `gi_room` still showed
faint, real, colored (not just film-grain) illumination on cube
silhouettes near the room's glass cube. `--gi-method none` disables
every PRIMARY GI technique, so this was a second, independent leak.

**Root cause**: `shade_for_reflection_bounce`/`shade_for_refraction_bounce`
(both `hybrid_trace.wgsl` and their CPU-reference twins
`reflect_ref.rs`/`refract_ref.rs`) call `cone_trace_indirect_single` —
a one-bounce cone-traced diffuse-GI "final gather," chosen deliberately
over recursing back into `shade()`'s own `GiMethod::ConeTrace` branch
(an intentional, correctly-reasoned design choice, confirmed via both
modules' own header doc comments) — but that call ran UNCONDITIONALLY,
with no check against the scene's own primary `GiMethod` at all. The
existing doc comments only ever justified WHICH mechanism to use for
bounce GI; neither ever addressed whether it should run AT ALL under
`GiMethod::None`. Confirmed empirically before touching code: a `gi_room`
debug build with `--no-reflect`/`--no-transmission` CLI toggles (added
temporarily, removed after) showed reflection contributing nothing to
the leak (`--no-reflect` alone: no change) but transmission being the
dominant source (`--no-transmission` alone: region `max` RGB dropped
77→52) — pointing straight at the glass cube's own refraction bounce
shading.

**Fix**: added a new `bounce_gi_enabled: bool` field to
`cpu_ref::ReflectionParams`/`TransmissionParams` (CPU side, threaded
through `reflect_trace_ray`/`refract_trace_ray` into
`shade_for_reflection_bounce`/`shade_for_refraction_bounce`, gating the
`cone_trace_indirect_single` call with a plain `if`) — real callers
should pass `gi_method != GiMethod::None`, tests pass `true` to preserve
prior behavior. WGSL side needed no signature threading at all (unlike
DDGI's grid-cell fix): `shade_for_reflection_bounce`/
`shade_for_refraction_bounce` already read `scene.conetrace_*` directly
from the module-scope uniform, so the same cone-trace call sites in
`hybrid_trace.wgsl` just gained a direct `if (scene.gi_method !=
GI_METHOD_NONE)` check — simpler than the CPU-ref's own parameter-passing
shape, but behaviorally identical.

**New tests, proving the flag gates REAL energy, not a no-op parameter**:
`reflect_ref::bounce_gi_enabled_false_strictly_reduces_energy_when_real_
gi_is_available` and `refract_ref`'s own identically-named counterpart —
each reuses an existing emissive-object fixture
(`mirror_floor_with_box_above`/`glass_cube_with_box_behind`) and asserts
`bounce_gi_enabled=false` returns strictly less total energy than
`=true` on the identical ray, both pass.

**Live verification**: sealed-room `--gi-method none` re-checked at the
exact camera/region first flagged (`x∈[600,840], y∈[400,620]` in a
1280×720 capture) — region average dropped from `[12.58,13.04,11.64]`
(pre-fix) to `[7.14,7.90,6.87]` (post-fix), and the region's own peak
RGB lost its warm-grey color cast (`[77,76,70]` → uniform `[60,60,60]`,
i.e. what's left is pure per-channel-identical film grain, not colored
surface shading). Re-verified with reflection AND transmission both
force-disabled (temporary CLI flags, since removed): identical result
(`avg=[7.05,7.82,6.80]`) — confirms the fix fully eliminates their
contribution; nothing further to find on this specific path.
`gallery.rs --gi-method ddgi` (a scene where the bounce GI term SHOULD
still fire) re-checked for a regression: clean, correct render, no
visible change.

**Known, separate, NOT fixed here**: the residual `max=60`-ish film-grain
floor on near-black pixels is real but architecturally backwards —
`hybrid_post.wgsl`'s own `signal_factor = clamp(1.0 - luminance, 0.15,
1.0)` gives the MOST grain to the DARKEST pixels (inverted from real
sensor noise's own actual behavior, which the surrounding doc comment
correctly describes but the formula doesn't quite implement for the
near-zero-luminance case), not gated toward zero for pixels that are
correctly, legitimately supposed to be pure black. Flagged honestly as
a separate, minor, cosmetic issue — out of scope for this investigation
(which was specifically about real, structured, colored light leaking
into a sealed room, not about post-process noise), not fixed
preemptively.

**Verified**: `cargo build --release --lib --examples` clean, `cargo
test --release --lib` 355/355 (353 + 2 new `bounce_gi_enabled` tests),
`cargo clippy --release --lib --examples` clean (only pre-existing,
unrelated warnings elsewhere), live GPU runs on `gi_room.rs`
(`--gi-method none`, sealed room, before/after + reflect/transmit
force-disabled cross-check) and `gallery.rs` (`--gi-method ddgi`, no
regression), no crash/validation error, region pixel values measured
via a temporary throwaway `pixel_probe` example (added and removed this
session — not part of the renderer).

### `trace_shadow`: a third sealed-room light leak — `margin_fade` and `VIS_CUTOFF` could interact to report a small nonzero `vis` for a ray that is actually, fully, hard-blocked

Follow-up to the bounce-GI fix above: even after both prior fixes, the
user reported still seeing a faint but real, structured (not film-grain)
grey edge on the glass cube's own silhouette under `--gi-method none` in
the sealed room. Confirmed this was a THIRD, independent mechanism by
temporarily force-disabling reflection AND transmission together
(`ReflectionConfig{enabled:false,..}`/`TransmissionConfig{enabled:false,..}`,
CLI flags added and removed) — the residual edge was byte-for-byte
identical either way, proving it lives entirely in DIRECT-light shadow
visibility, not any GI/bounce path already fixed.

**Root cause, found via a targeted investigation that hand-simulated the
real `trace_shadow` march against the glass cube's own real silhouette-
edge hit point and the real sun direction**: `trace_shadow`'s per-
candidate march (`assets/shaders/hybrid_trace.wgsl:706`/
`hybrid_ddgi_relight.wgsl:560`, CPU mirror `cpu_ref.rs::trace_shadow`)
can spend its FIRST sample against the roof panel (a thin, wide
occluder) right at the padded-candidate-AABB `margin` boundary —
`margin_fade` (a real, still-needed fix for a DIFFERENT, legitimate
problem: smoothing the visible polygonal seam at the candidate-AABB
entry cliff, see that field's own existing doc comment) forces that
first sample's own `vis` toward ~1.0 by design, regardless of true
occlusion. The SECOND sample then finds real, strong occlusion (`vis`
dropping to ≈0.0148, deep below `VIS_CUTOFF = 0.02`) — but the march
loop's own `while t <= max_t && vis > VIS_CUTOFF` condition is only
re-checked at the TOP of the next iteration, so it exits the loop
immediately, before a third sample could ever reach the roof's true
surface and register a proper `HardHit`. The function fell through to
`return Soft { vis }` at its own end with that small, stale, nonzero
leftover value — not the fully-opaque `0.0` the OUTER per-candidate loop's
own identical `if vis < VIS_CUTOFF { return Soft { vis: 0.0 } }` check
(a few lines up in the same function) already treats this exact
threshold as meaning. `shade`'s own `if shadow_vis <= 0.0 { continue }`
gate correctly passes real nonzero `vis` through (that's its whole job
for genuine penumbra) — it has no way to distinguish "real half-open
shadow" from "stale first-sample margin-fade artifact."

**Why the existing dense-grid CPU tests never caught this**: both
`no_shadow_ray_near_the_sealed_roof_leaks_light_through_a_panel_seam`
and `no_shadow_ray_anywhere_in_the_sealed_room_leaks_light_through_any_
seam` (added in the DDGI-fix stage above) swept fixed synthetic grid
origins across the room's own volume — none happened to land a shadow-
ray origin at the EXACT margin-boundary-triggering geometry this
specific camera-visible pixel's real shadow query hits. Grid sweeps over
synthetic points are not a substitute for testing the real geometry a
real screen pixel actually queries.

**Fix**: inside the inner march loop, immediately after computing this
sample's own `vis = vis.min(faded_vis)`, added `if vis <= VIS_CUTOFF {
return Soft { vis: 0.0 } }` — mirroring the outer loop's own identical
check exactly, so "dropped below VIS_CUTOFF" means the same fully-opaque
thing everywhere in this function, not a different thing depending on
whether the drop happened between candidates or mid-candidate. Applied
in three places: `cpu_ref.rs::trace_shadow`, and both WGSL copies
(`hybrid_trace.wgsl` and `hybrid_ddgi_relight.wgsl`, per this codebase's
established self-containment convention for duplicated shadow-march
code).

**New CPU-ref test, reproducing the exact real failure**:
`shadow_ray_from_the_glass_cubes_own_silhouette_edge_is_a_hard_hit_not_
a_soft_leak` — a new `gi_room_sealed_shell_with_glass_cube` fixture
(the existing sealed-shell fixture plus `spawn_cubes`'s own real Zone 5
glass cube), firing a shadow ray from the cube's own real top-front
rounded-edge hit point (`Vec3::new(1.205, -0.991, 1.0)`, the exact point
the investigation traced the visible screen artifact to) toward the real
sun. Failed with `vis=0.0148` (matching the investigation's own hand-
computed prediction) before the fix, passes (hard hit, or `vis < 1e-4`)
after. The two existing dense-grid sweep tests needed a small update
too: with the fix applied, EVERY sample in both grids now returns
exactly `vis=0.0` (a strictly stronger result than the `vis < 0.05`
tolerance those tests originally checked for), so their own `worst`
tracking variable can legitimately stay `None` — updated both from an
unconditional `.unwrap()` (which now panics on the correct, over-
achieving case) to an `if let Some(...)` that only asserts when a
nonzero worst-case was actually found.

**Live verification, partial — an honest limitation of this session's
own headless-screenshot timing, not a gap in the fix's own correctness**:
repeated attempts to capture a `--gi-method none` screenshot with the
glass cube's silhouette actually in frame WHILE the roof was still
genuinely sealed (`t < 5.0s`) were defeated by this run's own variable
GPU-pipeline warm-up time (5-9 real seconds, itself close to or exceeding
`ROOF_DELAY_SECS`) — every attempt either captured before the trace
pipeline was ready (blank frame) or after the roof had already started
opening. The CPU-reference test above is a deterministic, exact
reproduction of the real algorithm (the same formulas/constants
verified structurally identical to the WGSL port by direct line-by-line
comparison throughout this whole investigation) and is treated as
authoritative given the live-screenshot timing difficulty; `gallery.rs`
was re-checked instead for a live regression check on ordinary soft-
shadow rendering (a cube's own soft contact shadow on a mirror floor,
`--gi-method ddgi`) and shows no visible change — the fix only affects
the specific "march drops below VIS_CUTOFF mid-candidate, one step
short of a hard hit" case, not ordinary penumbra gradients.

**Verified**: `cargo build --release --lib --examples` clean, `cargo
test --release --lib` 356/356 (355 + 1 new test, 2 existing tests
updated for the fix's own strictly-better behavior), `cargo clippy
--release --lib --examples` clean (only pre-existing, unrelated warnings
elsewhere), live GPU run on `gallery.rs` (`--gi-method ddgi`) showing no
regression to ordinary soft-shadow rendering.

### DDGI: a fourth, deeper sealed-room light leak — the real probe grid's own bounds were never actually shrunk away from the room's solid wall shell

Follow-up to the three fixes above. After all three landed, the user
still saw a real, structured, colored leak live on the real GPU pipeline
(a purple glow/halo around the room's purple cube plus glowing patches on
the ceiling/upper walls) — but this time proved, via a live capture at
`--at-frame 20` (`t≈0.7s`, essentially the very first real relight
dispatch) with `probes_per_frame: 999999` (full-grid relight, no
rotation) and `max_history_length: 1.0` (temporal blend's alpha always
exactly 1.0, i.e. zero history mixing), that the leak is present on the
FIRST DDGI relight dispatch from a truly zero atlas. This ruled out every
remaining multi-frame explanation (temporal EMA, `probes_per_frame`
rotation aliasing, same-dispatch cross-probe feedback) — there simply
hadn't been time for any of those to matter yet. It also ruled out a
fourth WGSL-vs-CPU-ref formula divergence: `ddgi_ref.rs`'s own
`sealed_room_with_real_cube_furniture_at_production_ddgi_density_stays_
dark` test — full relight, zero history, real `gi_room.rs` cube
furniture, real production grid density — reported EXACTLY `0.0`
everywhere on frame 1, yet the real GPU showed a real leak on what should
be the equivalent dispatch. A prior exhaustive line-by-line WGSL-vs-CPU-
ref formula audit of `hybrid_ddgi_relight.wgsl` had already found zero
divergence (twice) — so the discrepancy had to be in the INPUTS the two
were fed, not the shading math itself.

**Root cause, found by comparing the CPU test's own hand-built probe-grid
bounds against what `extract_hybrid_scene` (`extract.rs`) actually
computes for the real scene**: every one of this file's own CPU tests
hand-shrinks its probe-grid bounds by a guessed inset (`-HALF_X+1.0`,
`-HALF_Y+0.5`, `-HALF_Z+1.0`) before calling
`ddgi_ref::probe_grid_from_bounds` — but the REAL pipeline
(`extract.rs:1176-1182`, before this fix) passed the scene's own
UNSHRUNK root BVH AABB straight through, with no shrink at all. That
AABB is the union of every object's own bounds, including the OUTWARD-
facing surface of `gi_room.rs`'s own solid wall shell — walls extend
outward from the room's interior half-extents (`ROOM_HALF_X/Y/Z =
8.0/3.0/6.5`) by `WALL_THICKNESS(0.3)` (+`WALL_OVERLAP(0.2)` on
non-normal axes), so the real BVH root AABB is `(±8.6, ±3.6, ±7.1)`, not
the room's own interior `(±8.0, ±3.0, ±6.5)` the CPU tests' hand-shrunk
guess approximates.

`probe_grid_from_bounds`'s own half-cell inset (`origin = bounds.min +
spacing/2`) exists ONLY to center probe 0 inside its own cell (see that
function's doc comment, which already documents a RELATED but distinct
prior bug: probes pinned exactly to `bounds.min`/`bounds.max` embedding
themselves in boundary geometry) — it was never checked against how
thick a scene's own real enclosing geometry actually is. At `gi_room`'s
own production `probe_spacing=1.2`, the half-cell inset (0.6) very
nearly equals the wall's own thickness, landing the outermost probe
LAYER almost exactly ON the interior wall plane (confirmed by hand:
inset from the real outer AABB face lands X/Y/Z layer 0 at exactly
-8.0/-3.0/-6.5, the room's own interior wall planes). A corner probe
(extreme layer on 2-3 axes simultaneously) lands ON or beyond the wall's
own OUTER surface, fully outside the sealed cavity entirely. Confirmed
live via a throwaway CPU-ref diagnostic that fed the real (unshrunk) room
AABB into `probe_grid_from_bounds` and ran one real relight pass: peak
irradiance 0.435 (nowhere near `0.0`) at a probe located at world
position `(8.8, -3.0, -6.5)` — literally outside the room's own outer
wall face. From a position like that, `probe_ray`s either skim the
wall's own surface at grazing incidence or escape clean through the
wall-panel seam gaps (this room's walls deliberately overlap at seams,
see `WALL_OVERLAP`'s own doc comment, but a probe embedded in/beyond the
wall itself can still find a path out through a seam at a shallow enough
angle), picking up real direct sunlight a probe safely inside the cavity
would never see.

A SECOND, compounding effect made a naive "just add a fixed wall-
thickness margin" fix insufficient on its own:
`probe_grid_from_bounds`'s own `dims_x`/`dims_z` use
`(extent/spacing).ceil()`, so the grid's actual covered span (`dims *
spacing`) almost always OVERSHOOTS the input extent by up to just under
one full `spacing` unit — and since `origin` is pinned to `bounds.min`
(intentionally, per that function's own documented cell-centering
contract, confirmed by an existing test asserting exactly this), 100% of
that overshoot lands on the MAX-axis side only. A margin sized only for
wall thickness gets silently eaten by this overshoot on the max side
while doing nothing wrong (just extra-safe) on the min side — confirmed
by hand-computing the grid with a 0.75-unit margin alone (no spacing
term yet): the max-X probe landed at `8.35`, BEYOND the interior wall
plane at `8.0`, i.e. the 0.75 margin alone was still not enough on the
max side even though it would have been plenty on the min side.

**Fix**: `extract_hybrid_scene` (`extract.rs`) now shrinks `root_bounds`
inward by `DDGI_GRID_WALL_SAFETY_MARGIN (0.75) + probe_spacing` on every
axis — the fixed 0.75 term covers real shell thickness (comfortably
larger than this codebase's own worst-case `WALL_THICKNESS + 
WALL_OVERLAP = 0.5`), the `+ probe_spacing` term covers the `ceil()`
overshoot described above — before calling `ddgi_ref::probe_grid_from_
bounds`, with a degenerate-bounds fallback (skip the shrink entirely if
it would invert min/max, e.g. an extremely small scene) to avoid ever
producing a nonsensical inverted grid. This is scoped to DDGI only —
Radiance Cascades' own `radiance_cascades_root_bounds_*` fields still use
the raw unshrunk `root_bounds` (that technique is experimental and
separately tracked, see the Radiance Cascades stages above; not addressed
by this fix).

**New CPU-ref tests** (`ddgi_ref.rs`): (1)
`probe_grid_from_unshrunk_real_room_bvh_bounds_leaks_light_from_frame_
one` — feeds the real (unshrunk, wall-inclusive) room AABB into
`probe_grid_from_bounds` and asserts the leak REPRODUCES (peak > 0.1),
kept permanently as a regression proof of the actual root cause, not
just a throwaway diagnostic; (2)
`probe_grid_from_extract_rs_shrink_formula_stays_dark` — applies
`extract.rs`'s own exact shrink formula to that same real AABB and
asserts the grid stays dark (peak < 0.01), closing the loop from "proves
the bug" to "proves the shipped fix resolves it."

**Live verification**: rebuilt `gi_room` with the fix, compared
`--gi-method none` vs `--gi-method ddgi` at `--at-frame 60` with every
`DdgiConfig` value at its real shipped default (`probes_per_frame: 512,
tile_size: 8, max_history_length: 24.0, max_t: 28.0, probe_spacing: 1.2,
vertical_layers: 6` — `git diff examples/gi_room.rs` confirmed clean of
any diagnostic overrides, only the pre-existing `ROOF_DELAY_SECS=30.0`
bump and an unrelated `--corridor-cam` debug flag remain). Before the
fix: `--gi-method ddgi` measured max RGB `[86,86,84]` and ~4x the average
brightness of the `none` baseline, with a single-pixel diff against
`none` as large as `sum_abs_diff=235` (`none`'s own pixel exactly black,
`ddgi`'s own the same pixel a bright `[84,82,69]`) — visually the same
purple-cube-halo/glowing-ceiling leak originally reported. After the
fix: `--gi-method ddgi` and `--gi-method none` are BIT-FOR-BIT IDENTICAL
across the entire frame (max RGB `[21.0, 24.0, 21.0]` in both, average
matching to the decimal, worst single-pixel diff over the whole
1280x720 frame `sum_abs_diff=0`) — a strictly stronger result than the
"drops to match baseline" bar this investigation set for itself.
`cargo test --release --lib`: 363/363 (361 + 2 new tests), no existing
test needed updating.

**How to apply**: if a future scene reports a DDGI light leak specific
to a densely-packed probe grid near thin enclosing/boundary geometry
(walls, floors, any solid shell), check first whether
`DDGI_GRID_WALL_SAFETY_MARGIN + probe_spacing` is still enough clearance
for that scene's own wall thickness before assuming a new bug — this
margin was sized against `gi_room`'s own worst case (`WALL_THICKNESS +
WALL_OVERLAP = 0.5`), not a universal constant provably safe for
arbitrarily thick enclosing geometry.

## Performance: distance-scaled march-convergence epsilon (perf/quality optimization pass, item 1 of 3)

2026-09-19: first of a planned three-step optimization pass (SOTA research
requested by the user first — checkerboard/temporal upscaling, secondary-
ray resolution decoupling, distance-based adaptive sampling — see the
conversation this entry came from for the full research summary). Order
agreed with the user: (1) distance-scaled march epsilon, (2) temporal
upsampling of the whole trace, (3) half-res reflections/refraction with
bilateral upsample — each measured before being kept, per the user's
explicit "keep or reject based on real numbers" instruction.

**The idea**: `march_object`'s sphere-tracing convergence check
(`d < HIT_EPSILON`, `HIT_EPSILON = 1e-4`) is a fixed, very tight tolerance
regardless of how far along the ray the current sample is. A hit 50 units
from the camera doesn't need sub-millimeter surface-position accuracy —
the error is already far smaller than a pixel's own footprint at that
range. Growing the tolerance with distance should let the march exit
sooner on distant geometry (fewer of the small "creeping up to the exact
surface" tail steps sphere tracing needs near convergence), which is the
dominant theoretical cost of SDF marching.

**Implementation**: reused `pixel_eps(t) = max(t * 0.0016, 2e-4)` — an
existing function in this codebase, previously used only for
`shadow_bias`'s shadow-ray-origin offset, with the exact "farther hits
need coarser precision" growth curve already trusted here — as the
march's own convergence tolerance: `d < max(HIT_EPSILON, pixel_eps(t))`.
Applied to every `march_object` copy in the codebase (self-containment
convention means each WGSL file duplicates the function): `cpu_ref.rs`,
`hybrid_trace.wgsl`, `hybrid_ddgi_relight.wgsl`, `hybrid_radiance_
cascades.wgsl`, `hybrid_dof.wgsl`. Two of those files (`hybrid_trace.wgsl`,
`hybrid_ddgi_relight.wgsl`) already had their own `pixel_eps`/`shadow_bias`
copies defined AFTER `march_object` in file order — WGSL requires
declaration before use, so both were relocated to just above
`march_object` rather than duplicated a second time in the same file.
`hybrid_radiance_cascades.wgsl` and `hybrid_dof.wgsl` had no `pixel_eps`
at all (they never needed `shadow_bias`) — added a standalone `pixel_eps`
copy to each, `shadow_bias` omitted since nothing in those files calls it.
`trace_shadow`'s own `SHADOW_HIT_EPSILON` was deliberately left untouched
in every file — that function's march-exit logic already has a delicate,
previously-buggy interaction between `margin_fade` and `VIS_CUTOFF` (see
the shadow-margin-vis-cutoff-leak-fix entry above), and loosening its hit
epsilon risks reopening that exact class of leak near thin occluders
without a clear perf payoff to justify the risk.

**Measured performance — the honest result is "no measurable win on this
GPU/scene, kept anyway for its own correctness value"**: benchmarked via
`gallery.rs`'s own `info!`-logged per-second FPS/percentile/per-pass-GPU-
timing lines (`log_debug_stats`), the project's established methodology,
across two scene shapes to separate BVH-descent cost from march-loop cost:

- `--stress 10000 --gizmos off` (20,000 objects, near-orbit camera,
  BVH-descent-heavy): before `gpu trace ≈133-142ms` (`frame p50 ≈171-
  172ms`), after `gpu trace ≈133-143ms` (`frame p50 ≈172ms`) — no change,
  within run-to-run noise.
- `--stress 1 --camera-orbit-radius far` (single object, long march
  distances, BVH-descent-trivial): before `gpu trace ≈8.0-10.0ms`, after
  `gpu trace ≈8.0-8.6ms` — no change, within run-to-run noise.

**Sanity check on the "why no change" question**: temporarily slashed
`MAX_MARCH_STEPS` in `hybrid_trace.wgsl` from 128 to 32 (a 4x cut to the
worst-case iteration count, reverted immediately after) on the `--stress
10000` scene and re-measured — `gpu trace` was STILL `≈137-143ms`,
statistically identical to the 128-step baseline. This proves march-loop
iteration count is not the bottleneck for the trace pass at all on this
GPU/scene combination — something else (most likely BVH candidate-
gathering across many objects, or fixed per-pixel dispatch overhead on
this integrated GPU) dominates the ~140ms cost, so shrinking the march's
own convergence tolerance had nothing to trade against. This also means
distance-scaled epsilon alone will not deliver the perf win the SOTA
research anticipated for this specific renderer/hardware — items 2/3 in
this optimization pass (temporal upsampling, half-res secondary rays)
don't share this dependency and remain worth pursuing on their own
merits.

**Decision**: kept, not reverted — the user's explicit call (offered a
straight revert-since-no-benefit option, chose to keep it instead) is
that this is still a real, tested, harmless precision-relaxation that may
pay off on different hardware or scenes where march iteration genuinely
is the bottleneck (e.g. a discrete GPU with a much larger/more complex
scene, or scenes with fewer BVH leaves and correspondingly longer
individual marches). `cargo test --release --lib`: 361/361, no test
needed updating (`HIT_EPSILON` itself is unchanged; `pixel_eps`'s own
existing tests are untouched). Visual regression check: `gallery.rs`
screenshot at `--stress 1 --at-frame 200` shows no surface popping, shadow
acne, or silhouette seams from the relaxed tolerance.

**How to apply**: if a future perf pass on different hardware/scene shape
finds the trace pass IS march-iteration-bound (verify first the same way
this entry did — temporarily slash `MAX_MARCH_STEPS` and confirm `gpu
trace` actually drops before assuming so), this fix is already in place
and should show a real win there without further code changes.

## Performance: primary-ray sub-pixel jitter, step 1a of temporal upsampling (item 2 of 3)

2026-09-19: second of the planned three-step optimization pass (see the
prior "distance-scaled march-convergence epsilon" entry above for item 1
and the full 3-step plan). This is STEP 1a of item 2 specifically — jitter
alone, at full resolution, with NO texture-resolution change yet. A
research pass into this codebase's own architecture (temporal reprojection,
ray-gen, texture sizing) surfaced enough real coupling — every one of 14
storage textures allocated at one shared size, 6 separate dispatch sites
mixing `viewport`-based and `textureDimensions`-based bounds checks, zero
samplers in any bind group that would need bilinear history/upscale,
`hybrid_trace.wgsl`'s own pre-existing doc comment arguing AGAINST primary-
ray jitter (its hit feeds `out_depth`, the GI temporal disocclusion test,
and reflect/refract virtual-point reprojection) — that the work was split:
prove jitter alone doesn't destabilize the existing disocclusion tests
BEFORE combining it with an actual sub-resolution trace target where a
regression would be much harder to isolate.

**What was built**: `src/hybrid/taa_ref.rs` (new file, CPU-ref-first per
this codebase's established convention) — `halton(index, base)` (radical-
inverse sequence, 1-indexed to skip the degenerate all-zero `n=0` term),
`taa_jitter_offset(frame_index, ring_size) -> Vec2` (Halton(2,3), centered
to `[-0.5, 0.5]` TEXELS, the standard TAAU jitter sequence — e.g. Unreal's
own TemporalAA), and `jitter_texels_to_ndc(texels, viewport_size) -> Vec2`
(the `2.0 / viewport_size` conversion `generate_primary_ray` needs after
its own `uv_to_ndc` step). 9 new CPU-ref tests, all passing.

New `JitterConfig` resource (`extract.rs`, mirrors `TemporalConfig`'s
exact shape) — `enabled: bool` (default `false`: this is an experimental
A/B toggle, not a proven improvement, every existing scene keeps its exact
current behavior unless `--jitter` is passed explicitly) and `ring_size:
u32` (default `64`, bounds the Halton index's own growth — NOT a fixed
sample-set cycle like `DofConfig`'s Vogel-disk ring, since Halton is
aperiodic). `SceneUniform` gained `jitter_enabled`/`jitter_offset_x`/
`jitter_offset_y` (mirrored field-for-field across all 7 WGSL copies of
the struct — 3 that actually use it, `hybrid_trace.wgsl`/`hybrid_blit.
wgsl`/`hybrid_dof.wgsl`'s own duplicated `generate_primary_ray`/
`primary_ray` functions, and 4 kept in lockstep purely for uniform-buffer
layout parity: `hybrid_radiance_cascades.wgsl`, `hybrid_ddgi_relight.wgsl`,
`hybrid_temporal.wgsl`, `hybrid_denoise.wgsl`). The NDC offset itself is
computed ONCE, in `pipeline.rs`'s `prepare_hybrid_scene` (the only place in
the render-world extract/prepare pipeline that already has the real
viewport size on hand), not independently in each of the three WGSL ray-
gen copies — guarantees they can't drift apart on a per-frame basis.
`--jitter` / `--jitter-ring N` CLI flags wired in both `gallery.rs` and
`gi_room.rs`.

**Live verification**:
- Shader compilation: clean with `--jitter` both absent and present, on
  `gallery.rs` and `gi_room.rs` (`grep -iE "error|panic|wgsl|shader"`
  against stdout, zero matches either way).
- Jitter-off regression: `cargo test --release --lib` 370/370 (361 + 9
  new `taa_ref` tests). Static-camera screenshot at matched frame/position
  with jitter off vs. this same code's jitter-off path — visually
  identical to the pre-change baseline (no `if` branch taken when
  `jitter_enabled == 0`, so this is expected, not just measured).
- Jitter-on stability: `gallery.rs --stress 1 --camera-mode manual
  --jitter` run continuously for 18+ real seconds — frame time stayed flat
  (`p50 ≈ 34-35ms` throughout, no runaway growth from history-buffer
  churn that a broken disocclusion test would cause). Static-camera
  screenshot at frame 300 (well past temporal convergence) with jitter on
  vs. jitter off, same exact camera pose: visually indistinguishable, no
  ghosting, no shimmering, no edge doubling.
- Sealed-room leak regression (the highest-risk interaction, since DDGI's
  temporal EMA + disocclusion test is exactly what the architecture
  research flagged as most exposed to jitter): `gi_room --jitter` at
  `--at-frame 60` — roof still sealed, room still fully dark, same as
  every prior sealed-room fix's own verification screenshot.

**Decision**: kept, defaulted OFF. This step doesn't itself claim a perf
or quality win — it's a prerequisite gate for step 1b (actually shrinking
the trace-resolution textures and adding bilinear history/upscale, the
part that can deliver a real perf win). The gate passed: this codebase's
existing `TEMPORAL_DEPTH_SIGMA`/`TEMPORAL_NORMAL_COS_THRESHOLD` rejection
thresholds and DOF's own same-pixel history check already tolerate a
sub-pixel jitter as ordinary noise rather than misfiring as a false
disocclusion, at least at the `--stress 1` scale tested. Not yet tested:
whether this holds at `--stress 10000` scale, or with `Object spin`
enabled (moving geometry, not just a static scene) — worth checking if a
regression surfaces once step 1b's own resolution change is layered on
top and something breaks, to isolate which of the two changes caused it.

**How to apply**: step 1b (sub-resolution trace + bilinear upsample at the
blit stage) can now build on this jitter machinery directly — `scene.
jitter_offset_x/y` already flows correctly into all three ray-gen copies.
The remaining work per the architecture research: resize `pipeline.rs`'s
14 storage textures to a scaled `want`, add samplers to the temporal (x3)
and blit bind group layouts for bilinear history/upsample reads, decide
on the 6 dispatch-site bounds-check inconsistency (`viewport`-based vs.
`textureDimensions`-based) explicitly rather than leaving it to diverge,
and re-tune (or accept) denoise's texel-radius blur becoming an effective
2x-larger screen-space footprint at half resolution.

## Performance: trace-resolution scale + bilinear upscale, step 1b of temporal upsampling (item 2 of 3)

2026-09-19: third leg of item 2's 3-step plan (see the two entries above:
distance-scaled march epsilon was item 1; primary-ray jitter was item 2's
own step 1a). This is step 1b — actually rendering the hybrid pipeline at
a smaller resolution and upscaling, the part step 1a was a prerequisite
gate for.

**What was built**: new `RenderScaleConfig` resource (`extract.rs`, same
shape as `JitterConfig` — `scale: f32`, default `1.0`, i.e. off/no-op,
`--render-scale F` CLI flag in `gallery.rs`/`gi_room.rs`). `SceneUniform`
gained `trace_size_x`/`trace_size_y` (the trace pass's own working
resolution in texels, `= real viewport * scale`, clamped `scale` to
`[0.1, 1.0]`) — mirrored across all 7 WGSL `SceneUniform` copies, actually
consumed by 3 (`hybrid_trace.wgsl`, `hybrid_dof.wgsl`, `hybrid_blit.wgsl`).

`pipeline.rs`'s `prepare_hybrid_scene` now sizes ALL 14 `HybridTargets`
storage textures (and, transitively, all 4 history buffers — diffuse GI,
reflection, transmission, DOF — since each keys its own resize off
`targets.size`) against `trace_size` instead of the raw viewport. `pass.rs`'s
6 compute-dispatch sites (trace, temporal x3, denoise, DOF) now read a new
`trace_size` local (sourced from `HybridTargetsRes` — the actually-created
texture size — rather than recomputing `viewport * scale` a second time,
so dispatch bounds can never drift from what the textures were really
built at) instead of `extracted_view.viewport`. `hybrid_trace.wgsl`'s/
`hybrid_dof.wgsl`'s own dispatch-bounds checks and ray-gen UV math switched
from `view.viewport` to `scene.trace_size_x/y` for the same reason —
`view.viewport` still reports the REAL output size, which now legitimately
differs from the trace textures' own size.

`hybrid_blit.wgsl` is the upscale point: added a `Filtering` sampler to
`hybrid_blit_layout` (reusing `HybridPipeline::dof_sampler`, already
`Linear`/`Linear` — no new sampler resource needed), and both fragment
entry points switched from a flat `textureLoad(color_tex, px)` to
`textureSampleLevel(color_tex, color_sampler, uv, 0.0)` at the real
fragment's own UV — a genuine bilinear upscale when `trace_size < real
viewport`, bit-for-bit equal to the old `textureLoad` at `scale == 1.0`
(sampling exactly at a texel center is a no-op for bilinear filtering).
Depth intentionally stays `textureLoad` at a NEAREST-neighbor-mapped
trace-resolution pixel (new `trace_pixel_for` helper) — bilinearly
interpolating a depth VALUE across a silhouette edge would produce a
physically meaningless in-between depth, a known upscaling pitfall
distinct from color. `primary_ray` (blit's own ray-gen, used only for
depth reconstruction) deliberately stays keyed to the REAL `view.viewport`,
not `trace_size` — it must reconstruct the ray for the true screen pixel
`frag_coord` refers to, not the coarser trace grid.

**A real bug caught before it shipped**: first attempt at `trace_pixel_for`
built `vec2<i32>(scene.trace_size_x, scene.trace_size_y)` directly from
the uniform's `u32` fields — WGSL's `vec2<i32>(...)` constructor does NOT
implicitly convert scalar kinds, so naga's validator correctly rejected
this at shader-module creation time (`Function 'trace_pixel_for' is
invalid`, caught immediately on the very first live run, both at
`--render-scale` default AND `0.5` — this was a hard compile-time
rejection, not a subtle runtime bug). Fixed with explicit `i32(...)` casts
per component before constructing the vector.

**Live verification**:
- Shader compilation: clean after the cast fix, both `--render-scale`
  absent (default `1.0`) and `--render-scale 0.5`.
- `cargo test --release --lib`: 370/370, unchanged from step 1a (this
  step added no new CPU-ref module — the scale factor is pure Rust
  plumbing plus WGSL UV math, nothing warranting a new test file).
- `scale == 1.0` regression: static-camera screenshot at a fixed pose,
  visually identical to both the pre-experiment baseline and step 1a's
  own jitter-off baseline — expected, since `trace_size` reduces to
  exactly the real viewport size at this scale.
- `scale == 0.5` performance, `gallery.rs --stress 1 --camera-mode manual`:
  `gpu trace` dropped from `~7.4ms` to `~2.2ms` at one test pose (a
  general viewing angle) and from `~14.1ms` to `~3.7ms` at a second,
  closer pose framing the test cube directly — both close to the
  theoretical 4x reduction from halving BOTH linear dimensions (0.5 * 0.5
  = 0.25x pixel count). `fps` roughly tripled at the first pose (30 to
  78).
- `scale == 0.5` quality: screenshots at both poses show close visual
  agreement with the `scale == 1.0` baseline — cube silhouette, ground-
  plane edge, shadow gizmo line, and background gradient all align
  correctly with no color/depth misalignment or shimmering; the only
  visible difference under close inspection is slight edge softening on
  the cube's straight silhouette lines from the bilinear upscale, not a
  broken artifact.
- Sealed-room leak regression (this codebase's standard correctness bar
  for any renderer change): `gi_room --render-scale 0.5` at `--at-frame
  60` — roof still sealed, room still fully dark, same as every prior
  sealed-room fix's own verification.
- Combined stability: `gallery.rs --stress 1 --camera-mode manual --jitter
  --render-scale 0.5` run continuously for 18+ real seconds — `gpu trace`
  stayed consistently `~1.7-2.2ms` throughout with no runaway growth (no
  sign of history-buffer resize thrashing or the two experiments
  interacting badly together).
- **`--stress 10000` (20,000 objects, BVH-descent-heavy — the same scene
  item 1's own distance-scaled-epsilon measurement used, where that
  change showed ZERO effect because march-step count wasn't the
  bottleneck there):** `gpu trace` dropped from `~133-143ms` (`scale=1.0`
  baseline, re-confirmed matching item 1's own numbers) to `~37-40ms` at
  `scale=0.5` — roughly **3.5x**, slightly under the 4x single-object
  cases hit, consistent with a meaningful share of this scene's own trace
  cost being BVH candidate-gathering per ray (which scales with pixel
  count too, but with different per-ray overhead than pure marching, so
  the reduction isn't perfectly quadratic). Frame `p50` dropped from
  `~172ms` to `~55ms` (~3.1x), `fps` from `6` to `~17-19`. Screenshots at
  both scales (different orbit frames, camera kept moving between the two
  runs) show correctly shaded geometry, ground planes, shadows, and GI at
  `scale=0.5` with no glaring artifacts across the dense many-small-cubes
  grid — confirms the win generalizes beyond simple single-object scenes
  to the demanding stress case, including the case where item 1's OWN
  optimization measured no benefit at all.

**Decision**: kept, defaulted OFF (`scale: 1.0`) — like step 1a, this is
an experimental A/B toggle, not yet the default for every scene. The
measured result is a genuine, substantial perf win (roughly matching the
theoretical 4x at `scale=0.5`) with visually minor quality cost even
WITHOUT step 1a's jitter contributing any temporal supersampling yet
(these two screenshots were single, static frames — jitter's own
detail-recovery benefit only shows up once accumulated across several
frames, not evaluated numerically in this round).

**Known limitations, honestly recorded, not yet addressed**:
- Denoise's edge-aware blur radius is still a fixed TEXEL count (`hybrid_
  denoise.wgsl`'s own `BLUR_RADIUS`) — at `scale=0.5` this blur now covers
  2x the real-screen-space footprint it did before, a real quality
  interaction the architecture research flagged before this step began
  and this step did not change or re-tune.
- No numerical measurement of jitter's own detail-recovery benefit when
  combined with a reduced trace resolution — the live checks so far only
  confirm the combination doesn't destabilize or visibly break anything,
  not that accumulated jitter meaningfully recovers lost high-frequency
  detail at `scale < 1.0`. That would need a multi-frame convergence
  comparison (e.g. `scale=0.5 --jitter` after N accumulated frames vs.
  `scale=1.0` at the same pose), not yet done.
- `DVec2`'s clamp range `[0.1, 1.0]` on `RenderScaleConfig::scale` is an
  arbitrary safety floor (prevents a degenerate near-zero trace
  resolution), not a validated lower bound for this renderer's own
  quality/perf tradeoff — no sweep across intermediate scales (e.g. 0.75,
  0.67) has been done to find where quality actually starts degrading
  unacceptably for this specific scene content.

**How to apply**: `--render-scale F` is available today on both example
binaries; combine with `--jitter` for the full step 1a+1b stack. If a
future session wants item 3 (half-res reflections/refraction with their
own independent bilinear upsample, decoupled from the PRIMARY trace
resolution this step controls), that's a separate resolution knob on top
of this one, not a replacement for it — reflections/refraction already
share `trace_size` today (they're written by the same `trace_main`
dispatch), so decoupling them to an even-lower resolution than the
primary trace would need its own separate storage-texture size, a third
`SceneUniform` resolution field, and its own upscale step before
compositing back into `denoised_color_view`.

## Performance: item 3 (half-res reflections/refraction) -- measured, not implemented

2026-09-19: third and final item of the perf/quality optimization pass
(see the three entries above: distance-scaled march epsilon was item 1;
primary-ray jitter and trace-resolution scaling were item 2's steps 1a/
1b). Item 3's original scope, from this pass's own SOTA research summary:
trace reflections/refraction at a resolution DECOUPLED from (smaller
than) the primary trace, upsampled with a depth/normal-guided bilateral
filter, on the theory that specular reflections are lower-frequency than
primary visibility and can tolerate coarser sampling.

**Architecture research before writing any code** found two things that
undercut the premise:

1. Reflection/refraction are already written by the SAME `trace_main`
   dispatch, at the SAME `scene.trace_size` resolution as everything else
   — meaning item 2's own `RenderScaleConfig` already shrinks them
   proportionally with the primary trace. A "half-res reflections" tier
   would need to be a THIRD, even-smaller resolution on top of that, not
   a first decoupling.
2. Rough surfaces (`roughness >= REFLECT_ROUGHNESS_GATE/TRANSMIT_
   ROUGHNESS_GATE = 0.3`) already get a spatial self-blur for free, from
   `reflect_trace_ray`'s own roughness-widened reflection CONE (see
   `hybrid_trace.wgsl`'s `reflection_half_angle`) — and already skip
   temporal accumulation entirely (`hybrid_temporal.wgsl`'s own copy-
   through branch at that gate). The classic SOTA argument for half-res
   reflections ("rough reflections are already blurry, downsampling them
   costs nothing visually") is handled by a DIFFERENT, pre-existing
   mechanism here, not left for a new resolution tier to solve. The
   residual cost this would need to justify is concentrated in SMOOTH/
   mirror-like reflections specifically (below the roughness gate), a
   narrower target than "reflections in general."

**Live measurement, mirror-cube case** (`gallery.rs --stress 1
--camera-mode manual --cube-roughness 0.0 --cube-metallic 1.0`, a genuine
below-the-gate mirror surface exercising the full per-pixel reflection
cost with no cone self-blur or temporal skip masking it): compared
`render_scale=1.0` against `render_scale=0.5` (already-shipped, item 2)
specifically on the reflection/refraction-owning passes, not just overall
frame time:

```
                    scale=1.0    scale=0.5    ratio
gpu trace           ~14.6ms      ~3.75ms      ~3.9x
reflect-temporal     ~4.3ms      ~1.35ms      ~3.2x
transmit-temporal    ~4.3ms      ~1.3ms       ~3.3x
denoise              ~2.25ms     ~0.65ms      ~3.5x
frame p50            ~40.8ms     ~14.5ms      ~2.8x
```

The reflection/refraction-owning passes (`reflect-temporal` +
`transmit-temporal` + `denoise`) total `~10.85ms` of `~40.8ms` frame time
(27%) at `scale=1.0`, and `~3.3ms` of `~14.5ms` (23%) at `scale=0.5` —
roughly the SAME share of the frame budget at both scales, not a growing
bottleneck being left behind by `render_scale`'s own uniform reduction.
`render_scale` alone already delivers close to the full theoretical
benefit for reflection/refraction specifically, because they're computed
from buffers that already shrink with it.

**Decision: NOT implemented.** A dedicated reflect/refract-only
resolution tier (new storage textures, new dispatch sizing, a new bind-
group layout change, a genuine depth/normal-guided bilateral upsample
reusing `hybrid_denoise.wgsl`'s own `blur_indirect` sigma weighting) is
real, non-trivial added complexity for a win that measurement shows is
mostly already captured by item 2's existing `--render-scale` knob on
this renderer's current test scenes. This is a measured dead-end, not an
untried idea — recorded here per this project's own "honest dead-ends
over false victories" convention (see the DDGI sealed-room leak
investigation's own PROGRESS.md entries for the precedent) rather than
building speculative infrastructure for an unconfirmed win.

**How to apply**: revisit this ONLY if a future scene demonstrates
reflections/refraction dominating frame cost far more than this session's
test case (e.g. many large mirror surfaces, or reflection `max_bounces`
pushed well above its current default of 1) — the measurement above is
specific to a single mirror cube with default bounce counts, not a proof
this could never matter at a different scene's scale. If revisited, the
architecture research already on file (this entry's own numbered list
above) is the correct starting map: reflection/refraction resolution
would need its own `SceneUniform` field alongside `trace_size_x/y`, its
own storage textures sized independently in `pipeline.rs`, its own
dispatch-size locals in `pass.rs`'s reflect/transmit temporal passes, and
a bilateral (not just bilinear) upsample at composite time in
`hybrid_denoise.wgsl` — `hybrid_denoise.wgsl`'s own `blur_indirect`
depth/normal weighting (`BLUR_NORMAL_SIGMA`/`BLUR_DEPTH_SIGMA`) is the
existing precedent to reuse for that filter's weights, not a fresh
sigma sweep.

This closes out the 3-item optimization pass: item 1 (distance-scaled
epsilon) shipped as a latent, currently perf-neutral correctness
improvement; item 2 (jitter + render-scale) shipped as a real, measured
perf win (~3-4x trace-pass reduction at `scale=0.5`, both on simple
scenes and `--stress 10000`); item 3 was measured and found already
subsumed by item 2's own win, so left unimplemented.

## Performance investigation: skybox/far-object baking (measured, not implemented) + trace-pass cost breakdown + DDGI any-hit occlusion (correctness fix shipped, perf null result)

Follow-up to the 3-item optimization pass above. The user asked for a
fresh SOTA investigation into baking distant/far scene content into a
skybox-like texture, updated every N frames and spread across those N
frames (mirroring DDGI's own `ddgi_probes_per_frame` round-robin
amortization) — the idea being that primary rays which would otherwise
march against far/background SDF geometry every frame could instead
sample a cached texture.

**Skybox/far-object bake: measured, not implemented.** Before building
anything, measured whether a primary ray that misses the whole scene (or
hits something far away) actually costs meaningfully more than one that
hits something near, at `--stress 10000`:

| Camera view | `gpu trace` |
|---|---|
| Near-orbit, hit-heavy (baseline) | 133-150 ms |
| Pure-sky miss (`--stress 10000`) | ~8-9 ms |
| Pure-sky miss (`--stress 100`) | ~6.4 ms |

A ray that misses the whole scene already terminates in ~O(1) — one root-
node BVH slab test — regardless of object count, confirming the earlier
`hybrid_trace_pass_bottleneck_not_march_steps.md` finding. A whole-scene
skybox bake would therefore be a **visual feature** (real sky instead of
the flat debug-magenta `background_r/g/b`), not a perf win — sky pixels
are already nearly free. A far-objects-only bake (baking only distant
geometry out of the live BVH, replaced by a cached sample) was the
version that could plausibly help, but carries a real correctness hazard
matching this project's own `646a388` leak class: shadow rays, DDGI
probes, cone-traces, and reflection rays all deliberately return black
on a miss (a sealed-room-safety convention), so removing far objects
from the live scene risks losing real shadow-casting/occlusion behavior
those objects still need to provide. **Decision: not implemented** — the
whole-scene variant isn't a perf win, and the far-objects variant's
premise (that far-hit rays are a meaningful cost) doesn't hold once the
real cost driver was identified (see below). Logged as a measured dead-
end per this project's "honest dead-ends over false victories"
convention, matching the `render_scale_subsumes_reflection_halfres_idea`
precedent.

**Full `gpu trace` cost breakdown at `--stress 10000`** (near-orbit,
baseline ≈139ms), via isolated single-toggle-off measurements from the
same baseline:

| Contributor | Isolated cost | Share of frame |
|---|---|---|
| DDGI per-pixel hard-occlusion ray (primary surface) | ~65-72 ms | ~48-52% |
| Reflections (probe + widened re-march + bounce shading) | ~28-31 ms | ~20-22% |
| Shadows (direct-light loop, 3 lights) | ~28 ms | ~20% |
| DDGI cone-traced GI inside reflection bounce | ~12 ms | ~8% |
| Primary ray + minimal shading floor | ~16-17 ms | ~12% |
| Transmission | ~0 ms | 0% (no default material uses it) |
| Chebyshev/distance-moments texel reads | ~0 ms | 0% (noise-level) |

This corrects an assumption from the prior session's own investigation:
`--gi-method` was assumed to default to `none` in `gallery.rs`, but it
actually defaults to `Ddgi` (`extract.rs`'s `GiMethodConfig::default`),
so the original 133-150ms baseline already included DDGI's own cost —
and DDGI turned out to be the single largest contributor, bigger than
shadows and reflections combined. Traced the cost to
`ddgi_sample_probe_grid` (`hybrid_trace.wgsl`): for **each of the 8
probes** in the trilinear-interpolation neighborhood around a shaded
point, the function fires a full BVH-accelerated `trace()` **hard-
occlusion ray** before trusting that probe's stored irradiance — 8 full
tree traversals per shaded pixel, more than shadows (2-4) or reflections
(~3) individually, which is exactly why it dominates.

**Investigated whether the hard-occlusion ray could be reduced/cached**,
given it's both the biggest lever found and sits in correctness-critical
territory (two of the four `646a388` leaks were adjacent to probe
visibility). Findings:
- The hard ray is the **original, load-bearing** mechanism (predates
  Chebyshev depth-weighting, added later) and is backed by two real,
  visually-found-bug regression tests (`an_occluded_probe_contributes_
  zero_not_a_reduced_weight`, `a_rotated_boxs_own_occlusion_ray_does_
  not_self_intersect_its_own_shaded_face` in `ddgi_ref.rs`). Chebyshev's
  own PROGRESS.md entry admits it showed **no visible difference** in its
  own before/after screenshot test — it's a theoretically-motivated
  addition, not a fix for an observed bug. Dropping the hard ray in favor
  of Chebyshev alone would trade the proven signal for the unproven one.
- Chebyshev-alone's classic failure mode (light bleeding, from variance-
  shadow-mapping theory) is **likely, not theoretical**, on the
  `--stress N` grid specifically: DDGI probe spacing (11.0 units) exactly
  equals the grid's cell pitch (11.0 units), a resonance that makes the
  5-hemisphere-sample moment estimate weak exactly where this scene lives.
- A world-space visibility cache (DDGI-probe-grid-style, amortized like
  probe relighting) doesn't fit: DDGI's own probe spacing (11.0 units) is
  far too coarse to preserve a hard shadow/occlusion edge; matching pixel-
  scale resolution would need a voxel grid far denser than the entire
  probe grid — screen space with extra steps.
- The one option with a **provably safe** correctness contract: replace
  the full nearest-hit `trace()` call (which finds the closest hit, with
  its normal) with an any-hit/early-out query, since the caller only ever
  reads `.did_hit`.

**Shipped: `any_hit` (CPU-ref: `cpu_ref.rs::any_hit`; WGSL:
`hybrid_trace.wgsl` and `hybrid_ddgi_relight.wgsl`, both call sites in
`ddgi_sample_probe_grid`/its WGSL mirror).** Mirrors `trace()`'s BVH
descent exactly, but returns `true` on the *first* converged march
against a non-excluded object instead of continuing to find the globally
nearest hit — no `best_t` shrinking, since there's no "closer" to chase
for a boolean query. Self-exclusion (skip the shaded object's own leaf)
is baked into the traversal itself, not checked by the caller after the
fact — this is not merely a style choice: implementing it surfaced a
**real correctness gap** in the OLD `trace()`-based check
(`trace(...).is_some_and(|hit| hit.entity != origin_entity)`), which only
inspects the *nearest* hit. If a shaded object's own self-graze happens
to be nearest, that check stops there and never learns whether a
separate, real occluder sits farther along the same ray before reaching
the probe — a genuine under-occlusion (leak) in that specific edge case.
`any_hit` closes this by skipping the excluded entity's own leaf during
traversal and continuing to search. New regression test in `ddgi_ref.rs`
(`excluding_the_origin_entity_does_not_hide_a_real_occluder_farther_
along_the_same_ray`) pins this exact scenario — confirmed to fail against
the old logic before the fix, pass after. 6 new CPU-ref parity tests in
`cpu_ref.rs` assert `any_hit` agrees with `trace()`'s hit-or-miss verdict
across clean hits/misses, `t_max` cutoffs, self-exclusion, from-inside-
geometry starts, and a 3-deep BVH stack at every cutoff distance. All 4
`gi_room`/`gallery` sealed-room CPU regression tests still pass bit-for-
bit; live-verified via `gi_room --at-frame 200` (t=21.5s, roof sealed,
opens in 8.5s) showing genuinely black with no light leak.

**Measured performance result: null.** Despite the correct algorithm and
the earlier diagnostic (temporarily hardcoding the occlusion result to
`false`) showing `gpu trace` drop from ~139ms to ~70ms, the REAL `any_hit`
implementation measures **no improvement** — `gpu trace` stays at
~138-146ms at `--stress 10000`, statistically indistinguishable from the
~133-150ms baseline, across multiple clean (non-GPU-contended) runs. At
`--stress 100` there was never a gap to begin with (baseline and fixed
both ~66-72ms — shallow BVH depth at low object counts makes nearest-hit
vs. any-hit traversal cost nearly identical). Likely explanation (not yet
confirmed via GPU profiling): this AMD RDNA/GCN GPU (`subgroup_min_size:
64, subgroup_max_size: 64` per its own reported adapter info) executes
compute invocations in 64-lane wavefronts in lockstep — an early-return-
on-first-hit only helps when *all 64 lanes* in a wavefront can exit
together. If even one lane's ray has a genuinely clear line of sight to
its probe (must traverse the whole tree to confirm no occluder), the
entire wavefront waits for it regardless of how quickly the other 63
lanes converged. On a dense, geometrically varied `--stress N` grid,
enough per-wavefront divergence in "does this specific probe-ray have a
clear line of sight" apparently erases the average-case saving entirely.

**Decision: `any_hit` KEPT (real correctness fix), perf angle CLOSED as a
measured null result.** This is a smaller-scope repeat of item 1's own
outcome earlier in this session (distance-scaled epsilon: also a genuine
correctness improvement, also zero measured perf benefit, also kept for
the correctness reason alone) — logged per the same "honest dead-ends
over false victories" convention rather than silently dropping a real bug
fix because its originally-hoped-for perf benefit didn't materialize.

**How to apply**: if DDGI occlusion cost is revisited for performance
specifically, GPU-side wavefront divergence should be measured directly
(via `renderdoc`, already in `flake.nix`'s dev-shell packages) before
assuming any further any-hit-style change would help — the mechanism
that made the "hardcode false" diagnostic fast (skipping the call
entirely, i.e. removing the wavefront's shared BVH-descent work
altogether) is categorically different from "make the same descent
early-out on a per-lane condition," and this session's result shows the
latter does not transfer to a real win on this hardware/scene combination.
The remaining, still-unexplored lower-risk lever from the research is
skipping only the reflection bounce's OWN re-run shadow loop
(`shade_for_reflection_bounce` in `hybrid_trace.wgsl`, ~half the total
shadow cost) — a constant/single-light approximation with no cache, no
history buffer, and no new validity test, not yet measured.

<!-- Newest entries go here, above this line. -->
