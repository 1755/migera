//! Cornell-box GI demo: a sealed room (white ceiling/floor/back wall, red
//! left wall, green right wall) full of avian3d-simulated spheres with
//! varied materials, plus one emissive sphere doubling as a grabbable
//! point light — click-drag it around the scene to watch GI respond in
//! real time. Reuses `gi_room.rs`'s room-panel-with-overlap pattern and
//! `physics_avian_playground.rs`'s avian3d spawn/kinematic-drive
//! conventions; the only genuinely new piece is the mouse-drag-on-a-
//! depth-plane interaction (`drag_light` below) — nothing else in this
//! codebase does cursor-ray picking yet.
//!
//! Run: `cargo run --release --example cornell_room`

use avian3d::prelude::*;
use bevy::camera::{Exposure, Hdr};
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::diagnostic::{DiagnosticPath, DiagnosticsStore, FrameCount, FrameTimeDiagnosticsPlugin};
use bevy::input::mouse::MouseButton;
use bevy::math::primitives::InfinitePlane3d;
use bevy::prelude::*;
use bevy::render::diagnostic::RenderDiagnosticsPlugin;
use bevy::render::view::window::screenshot::{Screenshot, save_to_disk};
use bevy::window::PrimaryWindow;
use bevy_egui::input::EguiWantsInput;
use bevy_egui::{EguiContexts, EguiPlugin, EguiPrimaryContextPass, egui};

use migera::hybrid::HybridRenderPlugin;
use migera::hybrid::extract::{DdgiConfig, GiMethod, GiMethodConfig, LampLight, LightToggles, ReflectionConfig, SunLight, TransmissionConfig};
use migera::hybrid::material::Material;
use migera::physics_avian::shape_to_collider;
use migera::sdf::assembly::SdfSceneRoot;
use migera::sdf::components::Shape;

// ---------------------------------------------------------------------------
// Room geometry: same overlapping-panel construction as gi_room.rs's
// spawn_room (see that file's own doc comments for why panels overlap by
// WALL_OVERLAP instead of meeting at an exact seam — a ray grazing a
// knife-edge gap between two independently-marched SDF objects can slip
// through and leak light). Unlike gi_room.rs, every panel here also
// carries an avian3d Collider + RigidBody::Static so the spheres actually
// rest against floor/walls instead of falling through geometry the
// renderer draws but physics never sees.
// ---------------------------------------------------------------------------

const ROOM_HALF_X: f32 = 6.0;
const ROOM_HALF_Y: f32 = 4.0;
const ROOM_HALF_Z: f32 = 6.0;
const WALL_THICKNESS: f32 = 0.3;
const WALL_OVERLAP: f32 = 0.2;

fn spawn_room(mut commands: Commands) {
    let white = Material::new(Vec3::splat(0.85), 0.0, 0.4).with_reflectance(0.6);
    let red = Material::new(Vec3::new(0.75, 0.08, 0.08), 0.0, 0.5);
    let green = Material::new(Vec3::new(0.1, 0.65, 0.15), 0.0, 0.5);

    // Identity-transform root, real position on the rigid-body entity
    // itself — NOT a non-identity SdfSceneRoot transform with the body as
    // its child. avian3d treats nested rigid bodies as a flat structure
    // (its own docs: moving a parent does NOT move a child body) and its
    // Transform<->Position sync systems compound the parent's own
    // translation into the child's local Transform every physics tick
    // when the parent isn't identity — a real bug an earlier version of
    // this function hit: the six wall panels drifted outward without
    // bound (~900 world units within seconds), which fed a scene AABB
    // that large straight into DDGI's probe grid and crashed on a >2GB
    // buffer allocation. Mirrors physics_avian_playground.rs's own
    // spawn_scene pattern exactly (floor/crates spawned as ChildOf(root)
    // with root at Transform::IDENTITY).
    let root = commands.spawn((SdfSceneRoot, Transform::IDENTITY, Visibility::default())).id();

    // corner_radius: 0.05, NOT 0.0 — see src/physics_avian/mod.rs's own
    // shape_to_collider doc comment for why a plain sharp-edged box now
    // maps safely to Collider::cuboid regardless, but this stays a small
    // nonzero radius anyway (matching every other avian3d example's own
    // wall/floor/crate colliders) as defense in depth, and because a
    // barely-visible 0.05 fillet on a room this size costs nothing
    // visually.
    let mut panel = |translation: Vec3, half_extents: Vec3, material: Material| {
        let shape = Shape::RoundedBox { half_extents, corner_radius: 0.05 };
        commands.spawn((
            ChildOf(root),
            shape,
            shape_to_collider(&shape).expect("RoundedBox always maps to a collider"),
            RigidBody::Static,
            Transform::from_translation(translation),
            material,
        ));
    };

    // Floor and ceiling: white, extended by WALL_OVERLAP in X/Z to close
    // the seam against every wall (see gi_room.rs's own roof-panel doc
    // comment for the confirmed leak this guards against).
    let floor_half = Vec3::new(ROOM_HALF_X + WALL_OVERLAP, WALL_THICKNESS, ROOM_HALF_Z + WALL_OVERLAP);
    panel(Vec3::new(0.0, -ROOM_HALF_Y - WALL_THICKNESS, 0.0), floor_half, white);
    panel(Vec3::new(0.0, ROOM_HALF_Y + WALL_THICKNESS, 0.0), floor_half, white);

    // Left (-X, red) / right (+X, green) walls: full Y/Z footprint,
    // extended by WALL_OVERLAP past the floor/ceiling seam, thin on X.
    let side_wall_half = Vec3::new(WALL_THICKNESS, ROOM_HALF_Y + WALL_OVERLAP, ROOM_HALF_Z);
    panel(Vec3::new(-ROOM_HALF_X - WALL_THICKNESS, 0.0, 0.0), side_wall_half, red);
    panel(Vec3::new(ROOM_HALF_X + WALL_THICKNESS, 0.0, 0.0), side_wall_half, green);

    // Back wall (-Z, white). Extended by WALL_OVERLAP in both X and Y
    // (this panel meets FOUR neighbors: floor, ceiling, left wall, right
    // wall).
    let end_wall_half = Vec3::new(ROOM_HALF_X + WALL_OVERLAP, ROOM_HALF_Y + WALL_OVERLAP, WALL_THICKNESS);
    panel(Vec3::new(0.0, 0.0, -ROOM_HALF_Z - WALL_THICKNESS), end_wall_half, white);

    // Front wall (+Z) — glass, not white: the camera now sits OUTSIDE the
    // room on this side (see CAMERA_POS's own doc comment), so this panel
    // is the "aquarium glass" the whole room is viewed through. Same
    // glass recipe as gi_room.rs's own glass cube (clear base color ->
    // zero Beer-Lambert absorption, full transmission, glass-like ior) —
    // still real, solid, collidable geometry (the room stays sealed for
    // physics; spheres/light bounce off it exactly like any other wall),
    // just optically transmissive instead of opaque.
    let glass = Material::new(Vec3::ONE, 0.0, 0.02).with_reflectance(0.9).with_transmission(1.0).with_ior(1.5);
    panel(Vec3::new(0.0, 0.0, ROOM_HALF_Z + WALL_THICKNESS), end_wall_half, glass);
}

// ---------------------------------------------------------------------------
// Spheres: dynamic avian3d rigid bodies, dropped from a grid above the
// floor so they fall and settle into a pile. Materials vary procedurally
// via the fract-of-index trick already established in gallery.rs's own
// BVH stress fixture (multiply the loop index by a few different
// irrational-ish constants, then fract() back into [0,1)) — this
// codebase has no RNG primitive anywhere (see dof_ref.rs's own doc
// comment), so this is the established substitute for "looks varied but
// stays deterministic."
// ---------------------------------------------------------------------------

const SPHERE_COUNT: usize = 216;
const SPHERE_RADIUS: f32 = 0.52;

fn spawn_spheres(mut commands: Commands) {
    let root = commands.spawn((SdfSceneRoot, Transform::IDENTITY, Visibility::default())).id();

    let drop_half_x = ROOM_HALF_X - SPHERE_RADIUS - 0.5;
    let drop_half_z = ROOM_HALF_Z - SPHERE_RADIUS - 0.5;
    // Proper 3D grid (X columns x Z rows x Y layers), NOT a flattened
    // single-loop division — an earlier version of this function
    // conflated "row" and "layer," dividing i by per_row twice and
    // spawning the first per_row*per_row spheres all at nearly the same
    // height, overlapping in a flat plane near the ceiling instead of a
    // real drop grid.
    //
    // per_axis is DERIVED from the room footprint and sphere radius, not
    // a fixed constant — a real bug found when SPHERE_COUNT/SPHERE_RADIUS
    // were bumped a second time (216 spheres @ 0.6 radius): a hardcoded
    // per_axis=6 left cells too small for the jitter to stay safe (worst-
    // case adjacent separation dropped below 2*SPHERE_RADIUS) AND needed
    // so many vertical layers that the bottom spawn layer landed below
    // the floor. Solving for the SMALLEST per_axis whose own cell size
    // clears `2*SPHERE_RADIUS` (so even a full-strength jitter toward a
    // neighboring cell can't cause a spawn-time overlap) keeps the grid
    // self-consistent for any future radius/count change instead of
    // needing hand-tuned constants again.
    let per_axis: usize = {
        let mut n = 1usize;
        while (2.0 * drop_half_x / n as f32) > 2.0 * SPHERE_RADIUS && (2.0 * drop_half_z / n as f32) > 2.0 * SPHERE_RADIUS {
            n += 1;
        }
        (n - 1).max(1)
    };
    let cell = SPHERE_RADIUS * 2.5 + 0.1;
    let shape = Shape::Sphere { radius: SPHERE_RADIUS };

    for i in 0..SPHERE_COUNT {
        let f = i as f32;
        // Deterministic pseudo-variation, not RNG (see this fn's own doc
        // comment) — three different irrational-ish multipliers so hue,
        // metallic, and roughness don't visibly correlate with each other
        // or with drop position.
        // Each multiplier is a genuinely irrational constant (golden
        // ratio conjugate, then fract(sqrt(2))/fract(sqrt(3))/
        // fract(sqrt(5))) — NOT the previous 2.3/3.7/1.31, which are
        // exact tenths (23/10, 37/10, 131/100) and produced an exact
        // repeating cycle every 10/10/100 spheres respectively, a real
        // bug: with only 72 spheres, metallic and roughness repeated
        // their EXACT same 10 values 7.2 times over, giving far less
        // visual variety than the sphere count implied (found by direct
        // computation, not visual inspection alone — i=0 and i=10
        // produced bit-identical metallic/roughness). An irrational
        // multiplier's fractional sequence never exactly repeats.
        let hue = (f * 0.618_034) % 1.0; // fract(golden ratio conjugate)
        let metallic = (f * 0.414_214) % 1.0; // fract(sqrt(2))
        let roughness = 0.05 + 0.9 * ((f * 0.732_051) % 1.0); // fract(sqrt(3))
        let reflectance = 0.3 + 0.6 * ((f * 0.236_068) % 1.0); // fract(sqrt(5))
        let base_color = hue_to_rgb(hue);

        let col = i % per_axis;
        let row = (i / per_axis) % per_axis;
        let layer = i / (per_axis * per_axis);
        let cell_x = 2.0 * drop_half_x / per_axis as f32;
        let cell_z = 2.0 * drop_half_z / per_axis as f32;
        // Grid cell CENTER, then jittered off-lattice by a SAFE fraction
        // of the cell's own half-width — two more irrational multipliers
        // (see this fn's own doc comment on why irrational, not
        // rational, constants), offset by -0.5 so the jitter is signed.
        // The fraction itself is DERIVED from the cell size and radius,
        // not a fixed 0.8: worst case is two ADJACENT cells jittered
        // toward each other by the full amount, which must still leave
        // >= 2*SPHERE_RADIUS between them — i.e. `cell*(1-fraction) >=
        // 2*radius`, so `fraction <= 1 - 2*radius/cell`, capped at 0.8 so
        // it never exceeds the original safe bound even when cells are
        // much larger than the spheres. A FIXED 0.8 was a real bug found
        // when SPHERE_COUNT/SPHERE_RADIUS were bumped a second time: at
        // per_axis's own tight-fit cell size (barely above 2*radius),
        // an unconditional 0.8 jitter would have put adjacent spheres
        // well inside each other at spawn. This is purely a "doesn't
        // look like a rigid grid" cosmetic pass — see
        // src/physics_avian/mod.rs's own shape_to_collider doc comment
        // for the overlapping-spawn incident this guards against.
        let safe_fraction_x = (1.0 - 2.0 * SPHERE_RADIUS / cell_x).clamp(0.0, 0.8);
        let safe_fraction_z = (1.0 - 2.0 * SPHERE_RADIUS / cell_z).clamp(0.0, 0.8);
        let jitter_x = ((f * 0.303_357) % 1.0 - 0.5) * safe_fraction_x; // fract(sqrt(7)), signed
        let jitter_z = ((f * 0.582_576) % 1.0 - 0.5) * safe_fraction_z; // fract(sqrt(11)), signed
        let x = ((col as f32 + 0.5) / per_axis as f32 * 2.0 - 1.0) * drop_half_x + jitter_x * cell_x * 0.5;
        let z = ((row as f32 + 0.5) / per_axis as f32 * 2.0 - 1.0) * drop_half_z + jitter_z * cell_z * 0.5;
        // Spawned near the CEILING, not stacked on the floor — every
        // sphere is a real RigidBody::Dynamic (see this fn's own doc
        // comment) and actually falls under avian3d's own gravity at
        // spawn, landing and settling into a pile rather than starting
        // pre-arranged in its own resting layout.
        let y = ROOM_HALF_Y - SPHERE_RADIUS - 0.2 - layer as f32 * cell;

        commands.spawn((
            ChildOf(root),
            shape,
            shape_to_collider(&shape).expect("Sphere always maps to a collider"),
            RigidBody::Dynamic,
            Transform::from_xyz(x, y, z),
            Material::new(base_color, metallic, roughness).with_reflectance(reflectance),
        ));
    }
}

/// Cheap deterministic hue->RGB (no HSV crate dependency needed for a
/// visual-variety demo) — six-segment piecewise-linear hue wheel, `hue`
/// in `[0,1)`, full saturation/value.
fn hue_to_rgb(hue: f32) -> Vec3 {
    let h = hue.rem_euclid(1.0) * 6.0;
    let x = 1.0 - (h % 2.0 - 1.0).abs();
    match h as u32 {
        0 => Vec3::new(1.0, x, 0.0),
        1 => Vec3::new(x, 1.0, 0.0),
        2 => Vec3::new(0.0, 1.0, x),
        3 => Vec3::new(0.0, x, 1.0),
        4 => Vec3::new(x, 0.0, 1.0),
        _ => Vec3::new(1.0, 0.0, x),
    }
}

// ---------------------------------------------------------------------------
// The grabbable light: one entity carrying BOTH the visible emissive
// sphere (Shape + Material::with_emissive) AND the actual illumination
// (PointLight + LampLight, extracted every frame off this same entity's
// GlobalTransform — see src/hybrid/extract.rs's own extract_hybrid_lights
// doc comment) AND the physics body (RigidBody::Kinematic + Collider).
// Kinematic, not Dynamic: it must never react to falling spheres bumping
// it (a light shouldn't get flung across the room by an errant sphere),
// and per avian3d's own documented convention (see spawn_elevator's doc
// comment in physics_avian_playground.rs) a kinematic body's Transform is
// driven by setting LinearVelocity and letting avian3d integrate it, not
// by writing Transform directly — drag_light below follows that
// convention exactly.
// ---------------------------------------------------------------------------

const LIGHT_RADIUS: f32 = 0.3;
// X/Y centered, but offset toward -Z (the FAR wall from the camera's own
// vantage point — see CAMERA_POS's own doc comment: the camera sits near
// +Z looking toward -Z) rather than room-center on Z. Room-center Z put
// the light directly in the camera's near field with the sphere pile
// mostly behind/around it, out of clear view — sitting it back near the
// far wall instead puts it visibly among/beyond the pile from the
// camera's own framing, so dragging it toward the camera visibly drives
// it through the spheres on screen.
const LIGHT_START: Vec3 = Vec3::new(0.0, 0.0, -ROOM_HALF_Z * 0.5);

/// How far the ACTUAL `PointLight` sits above the visible glowing sphere's
/// own surface — see this module's own `spawn_light` doc comment for why
/// they can't be co-located. Must exceed `LIGHT_RADIUS` by a real margin
/// (not just graze the surface), since every existing shadow-ray call
/// site in `cpu_ref.rs` treats `shadow_max_t` as reaching the light's
/// exact position with no per-entity exclusion for the light's own
/// geometry — this renderer has no "exclude this shape from shadow
/// casting" flag today (confirmed: any `Shape` under an `SdfSceneRoot`
/// unconditionally enters the shared BVH every shadow ray queries).
const LIGHT_OFFSET: f32 = LIGHT_RADIUS * 2.5;

/// Marks the grabbable, visible glowing sphere so `drag_light` can find
/// and drive it, and `sync_light_to_sphere` can read its position.
#[derive(Component)]
struct GrabbableLight;

/// Marks the actual `PointLight` entity so `sync_light_to_sphere` can
/// write its position each frame — kept entirely separate from
/// `GrabbableLight`'s own entity (see `LIGHT_OFFSET`'s doc comment for
/// why) rather than a parent/child Transform relationship, since the
/// light must NOT itself be under `SdfSceneRoot` (a `PointLight` has no
/// `Shape` of its own regardless, but keeping it structurally separate
/// makes the "these are two different things co-located by convention,
/// not by hierarchy" intent explicit rather than incidental).
#[derive(Component)]
struct FollowLight;

fn spawn_light(mut commands: Commands) {
    // Identity-transform root, real position on the rigid-body entity
    // itself — same avian3d-nested-body pitfall as spawn_room's own doc
    // comment describes (this entity was the OTHER offender in the
    // buffer-overflow crash, drifting outward each physics tick).
    let root = commands.spawn((SdfSceneRoot, Transform::IDENTITY, Visibility::default())).id();
    let shape = Shape::Sphere { radius: LIGHT_RADIUS };
    commands.spawn((
        ChildOf(root),
        GrabbableLight,
        shape,
        shape_to_collider(&shape).expect("Sphere always maps to a collider"),
        RigidBody::Kinematic,
        LinearVelocity::default(),
        Transform::from_translation(LIGHT_START),
        Material::new(Vec3::ONE, 0.0, 0.4).with_emissive(Vec3::new(3.0, 2.85, 2.55)),
    ));

    // The real light, offset above the sphere's start position by
    // LIGHT_OFFSET — see this fn's own doc comment. Not a child of the
    // sphere (a PointLight needs no SdfSceneRoot ancestry at all; only
    // Shape components are scene-graph members here), position kept in
    // sync every frame by `sync_light_to_sphere` instead.
    commands.spawn((
        FollowLight,
        LampLight,
        PointLight { color: Color::srgb(1.0, 0.95, 0.85), intensity: 400_000.0, range: 25.0, ..default() },
        Transform::from_translation(LIGHT_START + Vec3::Y * LIGHT_OFFSET),
    ));
}

/// Keeps the actual `PointLight` positioned `LIGHT_OFFSET` above the
/// visible grabbable sphere every frame, so the light appears to emanate
/// from the glowing sphere without the sphere's own geometry self-
/// shadowing it (see `LIGHT_OFFSET`'s doc comment). A fixed +Y offset
/// (not, say, offset away from the camera) reads correctly from any
/// viewing angle since the offset is small relative to room scale and
/// the light's own falloff is smooth — a directional seam would only be
/// visible if something were positioned exactly at the offset gap, which
/// nothing in this scene is.
fn sync_light_to_sphere(
    sphere: Query<&GlobalTransform, With<GrabbableLight>>,
    mut light: Query<&mut Transform, With<FollowLight>>,
) {
    let Ok(sphere_transform) = sphere.single() else { return };
    let Ok(mut light_transform) = light.single_mut() else { return };
    light_transform.translation = sphere_transform.translation() + Vec3::Y * LIGHT_OFFSET;
}

// ---------------------------------------------------------------------------
// Drag interaction: click-drag on a depth plane at the light's own
// current distance from the camera. Genuinely new territory for this
// codebase (no cursor-ray picking exists anywhere else) — built from two
// framework-native primitives, no new dependency: Camera::viewport_to_world
// turns the cursor position into a world-space ray, and Ray3d::intersect_
// plane finds where that ray crosses a plane perpendicular to the
// camera's own view direction at the grab depth. The grabbed light is
// driven via LinearVelocity toward that point each frame (never via
// direct Transform writes), per avian3d's own kinematic-body convention.
// ---------------------------------------------------------------------------

/// Tracks drag state across frames — `Some(depth)` while a drag is active,
/// where `depth` is the distance from the camera to the plane the light
/// is being dragged along (fixed for the whole drag, taken from the
/// light's own position at the moment the drag started).
#[derive(Resource, Default)]
struct DragState {
    depth: Option<f32>,
}

/// The maximum speed the kinematic light chases its drag target at — a
/// HARD CAP, not an unbounded proportional gain. An earlier version of
/// this used `velocity = (target - light_pos) * DRAG_GAIN` with no cap at
/// all — a real bug found live via debug logging: any single-frame
/// overshoot (e.g. a fast mouse flick, or `GlobalTransform` propagation
/// lagging a tick behind avian3d's own kinematic integration) produces a
/// large error, which the SAME formula then amplifies into an even
/// larger corrective velocity the next frame, oscillating with growing
/// amplitude — logged velocities grew roughly 10x per frame (35 -> 76 ->
/// 163 -> 352 -> 535 -> 748 -> ... units/sec) within about a second of
/// dragging, flinging the kinematic light hundreds of units away and
/// blowing up DDGI's probe-grid size well past wgpu's 2GB buffer limit.
/// Capping the OUTPUT (not just picking a smaller gain, which would only
/// raise the error threshold at which the same runaway starts) makes the
/// controller unconditionally stable regardless of how large a single
/// frame's position error ever gets.
const DRAG_MAX_SPEED: f32 = 12.0;

/// Proportional response rate for the "close to the target already"
/// case — see `DRAG_MAX_SPEED`'s own doc comment for why the far-away
/// case is capped rather than scaled. `1/DRAG_CATCH_UP_RATE` is roughly
/// the response time constant: at rate `10.0`, a 1-unit error produces a
/// 10-unit/sec chase speed, converging in a few tenths of a second — far
/// below `DRAG_MAX_SPEED`, so this only matters once the light is
/// already near the cursor's own target point.
const DRAG_CATCH_UP_RATE: f32 = 10.0;

fn drag_light(
    mouse: Res<ButtonInput<MouseButton>>,
    windows: Query<&Window, With<PrimaryWindow>>,
    cameras: Query<(&Camera, &GlobalTransform)>,
    mut light: Query<(&GlobalTransform, &mut LinearVelocity), With<GrabbableLight>>,
    mut drag: ResMut<DragState>,
    egui_wants_input: Res<EguiWantsInput>,
) {
    let Ok(window) = windows.single() else { return };
    let Ok((camera, camera_transform)) = cameras.single() else { return };
    let Ok((light_transform, mut velocity)) = light.single_mut() else { return };

    // Don't start (or continue processing as scene input) a drag while the
    // cursor is over an egui panel — same "UI eats the click" convention
    // every egui-integrated example in this codebase already respects
    // implicitly via egui's own input capture, made explicit here since
    // this is the first system that reads raw mouse button state directly.
    let egui_wants_pointer = egui_wants_input.wants_pointer_input();

    if mouse.just_released(MouseButton::Left) {
        drag.depth = None;
    }

    if mouse.just_pressed(MouseButton::Left) && !egui_wants_pointer {
        let camera_forward = camera_transform.forward();
        let to_light = light_transform.translation() - camera_transform.translation();
        drag.depth = Some(to_light.dot(*camera_forward).max(0.1));
    }

    let Some(depth) = drag.depth else {
        velocity.0 = Vec3::ZERO;
        return;
    };
    let Some(cursor) = window.cursor_position() else {
        velocity.0 = Vec3::ZERO;
        return;
    };
    let Ok(ray) = camera.viewport_to_world(camera_transform, cursor) else {
        velocity.0 = Vec3::ZERO;
        return;
    };

    let plane_origin = camera_transform.translation() + camera_transform.forward() * depth;
    let plane = InfinitePlane3d::new(camera_transform.forward());
    let Some(t) = ray.intersect_plane(plane_origin, plane) else {
        velocity.0 = Vec3::ZERO;
        return;
    };
    let target = ray.get_point(t);

    // Move toward the target at a capped speed, not a gain proportional to
    // the (potentially large, and NOT self-correcting under this
    // formula) position error — see DRAG_MAX_SPEED's own doc comment for
    // the instability this replaces. DRAG_CATCH_UP_RATE is the "close to
    // the target" proportional response (converges smoothly rather than
    // slamming into DRAG_MAX_SPEED right up to the last instant and
    // stopping dead); clamp_length_max caps the far-away case, which is
    // the one that produced the unbounded, self-amplifying velocity
    // before.
    let to_target = target - light_transform.translation();
    velocity.0 = (to_target * DRAG_CATCH_UP_RATE).clamp_length_max(DRAG_MAX_SPEED);
}

// ---------------------------------------------------------------------------
// Camera, HUD, CLI scaffolding — mirrors gi_room.rs's own conventions.
// ---------------------------------------------------------------------------

/// Inside the room, in the +Z corner (near the front/glass wall, which
/// the camera's own back nearly touches) looking back toward -Z across
/// the whole sphere pile — the same "camera in a corner looking at the
/// room's own center" framing gi_room.rs uses, pulled 1 unit off every
/// nearby surface so the camera itself is never embedded in wall
/// geometry (a real bug that pattern's own doc comment in gi_room.rs
/// already documents hitting once).
///
/// An "aquarium" framing (camera OUTSIDE the room, low to the ground,
/// looking up and in through the glass +Z wall — see spawn_room's own
/// doc comment on that panel's own glass material, which is real,
/// working, and kept regardless of this camera's own position) was
/// tried and reverted: it correctly proved the glass/transmission fix
/// works (src/hybrid/refract_ref.rs's own opaque-object final-gather
/// addition), and correctly diagnosed one real bug along the way (an
/// earlier look-into-the-room target point put the glass panel outside
/// Bevy's own ~13-degree default half-FOV entirely), but a SEPARATE,
/// still-unexplained rendering artifact (a vertical "totem pole" of
/// stacked spheres plus a spiral trail, reproducible and NOT caused by
/// glass reflectance — tested at both 0.9 and 0.4 with zero visual
/// change) showed up specifically at that external, steep-angle camera
/// position and needs interactive investigation (headless multi-minute
/// screenshot round-trips are too slow to isolate it further) before
/// that framing can ship. Revisit via `cargo run --release --example
/// cornell_room` directly rather than more `--shot`/`--at-frame` runs.
const CAMERA_POS: Vec3 = Vec3::new(0.0, ROOM_HALF_Y - 2.0, ROOM_HALF_Z - 1.0);

fn spawn_camera(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Msaa::Off,
        Hdr,
        Tonemapping::default(),
        Exposure::default(),
        Transform::from_translation(CAMERA_POS).looking_at(Vec3::new(0.0, -ROOM_HALF_Y * 0.2, -ROOM_HALF_Z * 0.3), Vec3::Y),
    ));
}

fn spawn_sun(mut commands: Commands) {
    // A dim sun mostly for a faint ambient fill so the room isn't
    // pitch-black before the player grabs the light — the emissive
    // point light is the scene's real, intended illumination.
    commands.spawn((
        SunLight,
        DirectionalLight { color: Color::srgb(1.0, 0.98, 0.95), illuminance: 150.0, ..default() },
        Transform::from_xyz(0.0, 10.0, 0.0).looking_at(Vec3::new(0.3, 0.0, 0.2), Vec3::Y),
    ));
}

/// `--shot PATH --at-frame N`: headless screenshot-based verification,
/// same convention every existing example already uses.
#[derive(Resource, Clone, Default)]
struct ShotConfig {
    shot: Option<(String, u32)>,
}

impl ShotConfig {
    fn from_args() -> Self {
        let args: Vec<String> = std::env::args().collect();
        let mut cfg = Self::default();
        let mut i = 0;
        while i < args.len() {
            let val = |i: &mut usize| -> String {
                *i += 1;
                args.get(*i).cloned().unwrap_or_default()
            };
            match args[i].as_str() {
                "--shot" => cfg.shot = Some((val(&mut i), cfg.shot.map_or(120, |s| s.1))),
                "--at-frame" => {
                    cfg.shot = Some((cfg.shot.map_or("/tmp/cornell_room.png".into(), |s| s.0), val(&mut i).parse().unwrap_or(120)));
                }
                _ => {}
            }
            i += 1;
        }
        cfg
    }
}

fn auto_shot(cfg: Res<ShotConfig>, frame: Res<FrameCount>, mut commands: Commands, mut fired: Local<bool>, mut exited: Local<bool>) {
    let Some((path, at)) = cfg.shot.clone() else {
        return;
    };
    if !*fired && frame.0 >= at {
        *fired = true;
        info!("cornell_room: screenshot -> {path}");
        commands.spawn(Screenshot::primary_window()).observe(save_to_disk(path.clone()));
    }
    if *fired && !*exited && frame.0 >= at + 60 {
        *exited = true;
        std::thread::sleep(std::time::Duration::from_millis(600));
        std::process::exit(0);
    }
}

/// `--gi-method none|ddgi|conetrace` (default `ddgi`). Mirrors
/// `gi_room.rs::gi_method_config_from_args` (same parsing shape) so
/// headless `--shot` runs can isolate direct lighting for debugging
/// without a human toggling the egui radio button.
fn gi_method_config_from_args() -> GiMethodConfig {
    let mut cfg = GiMethodConfig { method: GiMethod::Ddgi };
    let args: Vec<String> = std::env::args().collect();
    let mut i = 0;
    while i < args.len() {
        let val = |i: &mut usize| -> String {
            *i += 1;
            args.get(*i).cloned().unwrap_or_default()
        };
        if args[i].as_str() == "--gi-method" {
            cfg.method = match val(&mut i).as_str() {
                "none" => GiMethod::None,
                "conetrace" => GiMethod::ConeTrace,
                _ => GiMethod::Ddgi,
            };
        }
        i += 1;
    }
    cfg
}

fn controls_panel(mut contexts: EguiContexts, mut gi_method: ResMut<GiMethodConfig>, diagnostics: Res<DiagnosticsStore>) -> Result {
    egui::Window::new("Cornell Room").show(contexts.ctx_mut()?, |ui| {
        ui.label("Click-drag the glowing sphere to move the light.");
        ui.separator();
        ui.label("GI method");
        ui.radio_value(&mut gi_method.method, GiMethod::None, "None");
        ui.radio_value(&mut gi_method.method, GiMethod::Ddgi, "DDGI");
        ui.radio_value(&mut gi_method.method, GiMethod::ConeTrace, "Cone-trace");
        ui.separator();
        // Per-pass GPU timing, mirroring gi_room.rs's own controls_panel —
        // added specifically to make "is it slow, and if so which pass"
        // an answerable question for this scene instead of an eyeballed
        // guess.
        if let Some(ms) = gpu_pass_ms(&diagnostics, "hybrid_trace") {
            ui.label(format!("trace: {ms:.2} ms"));
        }
        if let Some(ms) = gpu_pass_ms(&diagnostics, "hybrid_ddgi") {
            ui.label(format!("ddgi relight: {ms:.2} ms"));
        }
        if let Some(ms) = gpu_pass_ms(&diagnostics, "hybrid_temporal") {
            ui.label(format!("temporal: {ms:.2} ms"));
        }
        if let Some(ms) = gpu_pass_ms(&diagnostics, "hybrid_denoise") {
            ui.label(format!("denoise: {ms:.2} ms"));
        }
        if let Some(ms) = gpu_pass_ms(&diagnostics, "hybrid_dof") {
            ui.label(format!("dof: {ms:.2} ms"));
        }
        if let Some(ms) = gpu_pass_ms(&diagnostics, "hybrid_blit") {
            ui.label(format!("blit: {ms:.2} ms"));
        }
    });
    Ok(())
}

fn gpu_pass_ms(diagnostics: &DiagnosticsStore, pass_name: &str) -> Option<f32> {
    let path = DiagnosticPath::from_components(["render", pass_name, "elapsed_gpu"]);
    diagnostics.get(&path).and_then(|d| d.smoothed()).map(|v| v as f32)
}

fn main() {
    let assets = std::env::current_dir().expect("cwd").join("assets").to_string_lossy().into_owned();

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window { title: "migera: Cornell room".into(), ..default() }),
            ..default()
        }).set(AssetPlugin { file_path: assets, ..default() }))
        .add_plugins(FrameTimeDiagnosticsPlugin::default())
        .add_plugins(RenderDiagnosticsPlugin)
        .add_plugins(EguiPlugin::default())
        .add_plugins(HybridRenderPlugin)
        .add_plugins(PhysicsPlugins::default())
        // No lamp/projector entities in this scene beyond the grabbable
        // light itself (which IS the lamp) — sun stays on for a faint
        // ambient fill, projector unused.
        .insert_resource(LightToggles { sun: true, lamp: true, projector: false })
        .insert_resource(gi_method_config_from_args())
        // Room-scale DDGI tuning, same reasoning as gi_room.rs's own
        // override: the crate default probe_spacing (11.0) is sized for
        // gallery.rs's --stress N grid, not a single ~12x8x12 room —
        // tightened here so the probe grid actually resolves bounce light
        // between the pile of spheres and the room's own walls/corners.
        .insert_resource(DdgiConfig { probe_spacing: 1.0, vertical_layers: 8, max_t: 24.0, ..DdgiConfig::default() })
        // Room-scale reflection reach: ReflectionConfig::max_t's own
        // crate default (60.0, sized for gallery.rs's --stress N scenes)
        // has no reason to be that large in a room whose longest interior
        // diagonal is ~17 units — theorized (via cone_slab_hit/trace_cone
        // BVH-pruning analysis) to defeat slab-test pruning the same way
        // DIRECTIONAL_SHADOW_MAX_T and shadow_candidate_margin's own
        // oversized-reach bugs did elsewhere in this codebase. Kept
        // because it's unconditionally correct regardless (no legitimate
        // in-room reflection ray needs 60 units of reach, so this has no
        // visual cost), but flagging honestly: a live A/B on this scene
        // did NOT show a clear win (~79ms either way, run-to-run physics-
        // settling variance in this scene's own measurement made a clean
        // before/after comparison difficult) — this project's own
        // "measure, don't assume" convention means this should be logged
        // as unconfirmed, not claimed as a proven fix.
        .insert_resource(ReflectionConfig { max_t: 15.0, ..ReflectionConfig::default() })
        // max_bounces: 2, NOT the crate default of 1 — required for the
        // aquarium glass wall to show anything AT ALL behind it. With
        // max_bounces=1, refract_trace_ray's loop shades only the glass's
        // OWN exit surface and breaks before ever probing for what's
        // behind it (see src/hybrid/refract_ref.rs's own doc comment on
        // the opaque-object final-gather fix this scene motivated) — at
        // 1 bounce the aquarium view showed a uniform dark wash (the
        // glass's own shadowed inner face), not the room. 2 bounces lets
        // the chain reach exactly one object behind the glass (a sphere,
        // or the room's own back wall) and shade it directly; this
        // room's glass is a single pane, not a stack of panes, so no
        // higher bounce count is needed. Room-scale max_t (30.0, not the
        // crate default 60.0) for the same BVH-pruning reasoning as
        // ReflectionConfig::max_t above.
        .insert_resource(TransmissionConfig { max_bounces: 2, max_t: 30.0, ..TransmissionConfig::default() })
        .insert_resource(ShotConfig::from_args())
        .insert_resource(DragState::default())
        .add_systems(Startup, (spawn_camera, spawn_room, spawn_spheres, spawn_light, spawn_sun))
        .add_systems(Update, (drag_light, sync_light_to_sphere, auto_shot).chain())
        .add_systems(EguiPrimaryContextPass, controls_panel)
        .run();
}
