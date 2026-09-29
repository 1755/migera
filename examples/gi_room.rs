//! GI stress scene: a fully closed room, completely dark except for the
//! sun, with a heavy sliding roof panel that starts fully covering the
//! ceiling and slowly slides open after 5 seconds until it covers only
//! half — the point where real daylight first reaches the interior.
//! Built to make cone tracing's own indirect-diffuse contribution
//! visible and testable: a single-bounce or no-GI renderer would show
//! the room's far corners staying essentially black even once the sun
//! floods in through the gap, while a working multi-bounce cone-traced
//! result should show the sunlit half's bounce light gradually filling
//! the shadowed half too. No lamp/projector — the sun is the room's
//! only light source, for a stark, unambiguous "sealed room" vs. "sun
//! gets in" contrast with nothing else to confound it.
//!
//! Not normally a headless-capture example — meant to be watched
//! interactively: `cargo run --release --example gi_room`.
//! `--shot PATH --at-frame N` is a TEMPORARY debugging addition (see
//! `gallery.rs`'s own `ShotConfig`/`auto_shot` for the pattern this
//! mirrors) for tracking down a real multi-bounce light-leak report;
//! remove once that investigation concludes.

use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::diagnostic::{DiagnosticPath, DiagnosticsStore, FrameCount, FrameTimeDiagnosticsPlugin};
use bevy::prelude::*;
use bevy::camera::{Exposure, Hdr};
use bevy::render::diagnostic::RenderDiagnosticsPlugin;
use bevy::render::view::window::screenshot::{Screenshot, save_to_disk};
use bevy_egui::{EguiContexts, EguiPlugin, EguiPrimaryContextPass, egui};

use migera::hybrid::HybridRenderPlugin;
use migera::hybrid::extract::{
    ConeTraceConfig, DdgiConfig, DenoiseConfig, DofConfig, GiMethod, GiMethodConfig, HybridPostConfig, JitterConfig, LightToggles,
    RadianceCascadesConfig, ReflectionConfig, RenderScaleConfig, ShadowConfig, SunLight, TemporalConfig, TransmissionConfig,
};
use migera::hybrid::material::Material;
use migera::sdf::assembly::SdfSceneRoot;
use migera::sdf::components::Shape;

// ---------------------------------------------------------------------------
// Room geometry: interior 16 (X) x 6 (Y) x 13 (Z) world units, walls/floor/
// ceiling built from thin RoundedBox panels (half_extents sized so the
// panel's OUTER face sits exactly on the room boundary, its inner face
// recessed by WALL_THICKNESS — same "panel as a flat slab" approach
// gallery.rs's own ground plate already establishes for RoundedBox, just
// applied to all 6 sides instead of one). Widened from an original 10x6x8
// (X/Z only, Y left unchanged) so the camera — parked in a far corner —
// sees more of the room across its own real depth/distance, without
// making the room any taller.
// ---------------------------------------------------------------------------

const ROOM_HALF_X: f32 = 8.0;
const ROOM_HALF_Y: f32 = 3.0;
const ROOM_HALF_Z: f32 = 6.5;
const WALL_THICKNESS: f32 = 0.3;

/// How many seconds the room stays fully sealed before the roof starts
/// sliding open.
const ROOF_DELAY_SECS: f32 = 30.0;
/// How many seconds the slide itself takes, once it starts.
const ROOF_SLIDE_SECS: f32 = 6.0;

/// Marks the single sliding roof panel — a full-ceiling-sized slab that
/// starts centered (fully sealing the room) and slides along +X by
/// exactly `ROOM_HALF_X` (half the room's own width), ending flush with
/// the +X wall and leaving the -X half of the ceiling open to the sky.
#[derive(Component)]
struct RoofPanel;

/// TEMPORARY — see this file's own top doc comment. Mirrors
/// `gallery.rs::ShotConfig` exactly.
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
                    cfg.shot = Some((cfg.shot.map_or("/tmp/gi_room.png".into(), |s| s.0), val(&mut i).parse().unwrap_or(120)));
                }
                _ => {}
            }
            i += 1;
        }
        cfg
    }
}

/// `--gi-method none|ddgi|cascades|conetrace` (default `ddgi`, this
/// scene's own established default — see the `GiMethodConfig` insertion
/// site's own comment). Mirrors `gallery.rs::gi_method_config_from_args`
/// exactly (same parsing shape, same later-flag-wins behavior) — added so
/// headless `--shot`/`--bench` runs on this scene can select
/// `GiMethod::RadianceCascades` without a human toggling the egui radio
/// button, which the planned DDGI-vs-cascades A/B comparison on this
/// scene's own dark-corridor bug needs.
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
                "cascades" => GiMethod::RadianceCascades,
                _ => GiMethod::Ddgi, // "ddgi" or unrecognized: default to DDGI
            };
        }
        i += 1;
    }
    cfg
}

/// TEMPORARY — see this file's own top doc comment. Mirrors
/// `gallery.rs::auto_shot` exactly.
fn auto_shot(cfg: Res<ShotConfig>, frame: Res<FrameCount>, mut commands: Commands, mut fired: Local<bool>, mut exited: Local<bool>) {
    let Some((path, at)) = cfg.shot.clone() else {
        return;
    };
    if !*fired && frame.0 >= at {
        *fired = true;
        info!("gi_room: screenshot -> {path}");
        commands.spawn(Screenshot::primary_window()).observe(save_to_disk(path.clone()));
    }
    if *fired && !*exited && frame.0 >= at + 60 {
        *exited = true;
        std::thread::sleep(std::time::Duration::from_millis(600));
        std::process::exit(0);
    }
}

fn main() {
    let assets = std::env::current_dir().expect("cwd").join("assets").to_string_lossy().into_owned();

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window { title: "migera: GI room".into(), ..default() }),
            ..default()
        }).set(AssetPlugin { file_path: assets, ..default() }))
        .add_plugins(FrameTimeDiagnosticsPlugin::default())
        .add_plugins(RenderDiagnosticsPlugin)
        .add_plugins(EguiPlugin::default())
        .add_plugins(HybridRenderPlugin)
        // Lamp/projector unused in this scene (sun only) — LightToggles
        // is still a required resource for extraction, so insert one
        // with both off rather than spawning unused PointLight/SpotLight
        // entities.
        .insert_resource(LightToggles { sun: true, lamp: false, projector: false })
        .insert_resource(ShadowConfig::default())
        // TEMPORARY debug overrides — see this file's own top doc comment.
        .insert_resource(DenoiseConfig { enabled: !std::env::args().any(|a| a == "--no-denoise") })
        .insert_resource(TemporalConfig { enabled: !std::env::args().any(|a| a == "--no-temporal"), ..TemporalConfig::default() })
        .insert_resource(JitterConfig { enabled: std::env::args().any(|a| a == "--jitter"), ..JitterConfig::default() })
        .insert_resource(RenderScaleConfig {
            scale: std::env::args().skip_while(|a| a != "--render-scale").nth(1).and_then(|v| v.parse().ok()).unwrap_or(1.0),
        })
        // Room-scale cone-tracing tuning: ConeTraceConfig::max_t's own
        // crate default (60.0, sized for --stress N's much larger scene)
        // is too loose for this sealed room — a real bug, found by direct
        // visual inspection at this room's original (smaller) size: a
        // cone ray reach of 60.0 let any ray that found even a tiny
        // residual seam gap in the sealed shell (see spawn_room's own
        // roof/floor panel doc comments) escape and travel FAR past the
        // room, sampling cone_sky_color's bright mocked-sky gradient
        // outside — smeared across the whole ceiling/corners as if
        // genuinely lit, even with the roof provably fully sealed. This
        // room's own full diagonal (walls included) is ~25.2 world units
        // at its current (widened) size — 28.0 stays safely above that
        // (legitimate room-spanning bounces still work) while tightly
        // capping how far any still-undiscovered leak could reach.
        // Re-check this value against the room's own diagonal any time
        // ROOM_HALF_X/Y/Z changes; it is NOT self-scaling.
        // cone_half_angle/cone_origin_radius keep their crate defaults —
        // no room-specific angle/radius tuning done yet.
        .insert_resource(ConeTraceConfig {
            max_t: 28.0,
            // TEMPORARY debug override — see this file's own top doc comment.
            max_bounces: std::env::args().skip_while(|a| a != "--cone-bounces").nth(1).and_then(|v| v.parse().ok()).unwrap_or(1),
            ..ConeTraceConfig::default()
        })
        // Room-scale DDGI tuning: gallery.rs's own default probe_spacing
        // (11.0) and vertical_layers (3) were sized for --stress N's
        // ~11-unit object cell pitch, not a single room — at that spacing
        // this room would get a tiny probe grid (barely any spatial
        // resolution at all). Tightened here to actually resolve bounce
        // light between the room's own corners/objects. max_t=28.0
        // mirrors ConeTraceConfig::max_t's own leak-capping reasoning
        // above (this room's own ~25.2-unit diagonal) — though a probe
        // ray escaping through a seam gap can no longer bake a bright
        // fake-sky value into its own stored irradiance regardless (see
        // ddgi_ref.rs::probe_ray's own doc comment: a miss reports
        // exactly black now, not a mocked sky gradient), this cap is
        // kept anyway to avoid wasting probe-ray budget marching far
        // past the room for a guaranteed-black result.
        .insert_resource(DdgiConfig {
            probes_per_frame: 512,
            tile_size: 8,
            max_history_length: 24.0,
            max_t: 28.0,
            probe_spacing: 1.2,
            vertical_layers: 6,
        })
        // Same room-scale retuning reasoning as DdgiConfig immediately
        // above, applied to Radiance Cascades' own level-0 base_spacing:
        // RadianceCascadesConfig::default()'s base_spacing=11.0 was
        // copied from DdgiConfig::default()'s own global default (also
        // sized for --stress N's ~11-unit object cell pitch, see that
        // default's own doc comment) — at that spacing this room's own
        // 16x6x13 bounds only clear MIN_PROBES_PER_AXIS (2 per axis) on
        // every axis, an 8-probe level-0 grid far too coarse to resolve
        // anything (confirmed live: --gi-method cascades on this scene
        // rendered almost entirely black). base_spacing=1.2 matches
        // DdgiConfig::probe_spacing above for an apples-to-apples level-0
        // density.
        //
        // base_interval=3.0, NOT 1.0 (a first attempt at this retune,
        // reverted — see PROGRESS.md's own Stage 4 writeup for the full
        // debugging story): level 0's own interval_far IS the reach every
        // level-0 probe ray gets before being declared a miss
        // (relight_cascade_texel treats "no hit within interval_far" as
        // transmittance=1.0, radiance=0.0, same as a genuine empty-space
        // miss) — with 1.2-unit probe spacing in a room with up to
        // 8-unit half-extents, an interval_far of only 1.0 left most
        // level-0 probes' own rays too short to reach ANY nearby surface
        // in most directions, so level 0 (the level that matters most,
        // since the merge hierarchy's own near cascade dominates the
        // final result unless fully transparent) reported near-total
        // darkness almost everywhere — confirmed live via direct GPU
        // debug instrumentation (a sentinel-radiance write proved the
        // atlas write/read plumbing itself was correct; the zero
        // radiance traced specifically to relight_cascade_texel's own
        // interval being too short). base_interval=3.0 gives level 0 a
        // reach comparable to its own probe spacing's local neighborhood
        // (reliably hits nearby walls/objects/floor), while level 3
        // still reaches 3.0*(64-1)/3 + 3.0*64 = 255 world units, far
        // beyond this room's own ~21-unit diagonal — long-range coverage
        // remains unaffected.
        .insert_resource(RadianceCascadesConfig {
            base_spacing: 1.2,
            base_ray_count: 64,
            base_interval: 3.0,
            base_tile_size: 8,
            // TEMPORARY debug override, same shape as ConeTraceConfig's
            // own --cone-bounces above — for the DDGI-vs-cascades bounce-
            // depth A/B comparison (bounce_passes: 1 is the config's own
            // safe default elsewhere, unchanged).
            bounce_passes: std::env::args().skip_while(|a| a != "--cascade-bounces").nth(1).and_then(|v| v.parse().ok()).unwrap_or(1),
        })
        // Ddgi: see GiMethod's own doc comment for why this is now the
        // default across the renderer — three independent visual reviews
        // of THIS scene converged on the exact structural GI gap DDGI's
        // spatial probe cache solves. ConeTrace and the experimental
        // RadianceCascades kept selectable (not deleted) via the egui
        // panel's own radio buttons below and via gi_method_config_from_args
        // (mirrors gallery.rs's own function 1:1) for headless --shot/
        // --bench A/B comparison runs.
        .insert_resource(gi_method_config_from_args())
        // Same room-scale leak-capping reasoning as ConeTraceConfig::max_t
        // above, applied to reflection's own ray reach — this room's own
        // sealed-box regression test (reflect_ref.rs's own mirror-box
        // fixture) already covers the underlying coverage^2 fix; this
        // value just keeps a legitimate reflection ray from traveling
        // beyond this room's own ~25.2-unit diagonal even if some
        // still-undiscovered seam gap let one escape.
        .insert_resource(ReflectionConfig { max_t: 28.0, ..ReflectionConfig::default() })
        // Same room-scale leak-capping reasoning as ReflectionConfig::max_t
        // above, applied to transmission's own interior-march/continuation
        // reach.
        .insert_resource(TransmissionConfig { max_t: 28.0, ..TransmissionConfig::default() })
        .insert_resource(ShotConfig::from_args())
        .add_systems(Startup, (spawn_camera, spawn_room, spawn_cubes, spawn_lights, spawn_hud))
        .add_systems(Update, (slide_roof, update_hud, auto_shot))
        .add_systems(EguiPrimaryContextPass, controls_panel)
        .run();
}

// ---------------------------------------------------------------------------
// Camera: fixed in a room corner, looking at the room's own center — no
// orbit controller (this scene is about watching the light change over
// time from one stable vantage point, not surveying geometry).
// ---------------------------------------------------------------------------

/// The camera sits in the +X/+Z corner — deliberately the corner
/// FARTHEST from every cube (all of which are placed in the -X half,
/// see `spawn_cubes`), so the camera itself is never embedded inside or
/// pressed up against any object's own geometry (a real bug this file
/// shipped with once already: a camera placed inside a cube's own SDF
/// just marches outward from inside it, reading as "the whole screen is
/// one flat color"). Raised well above floor level and pulled a full
/// meter off each wall for the same reason, applied to the room shell
/// itself.
const CAMERA_CORNER: Vec3 = Vec3::new(ROOM_HALF_X - 1.0, -ROOM_HALF_Y + 1.8, ROOM_HALF_Z - 1.0);

fn spawn_camera(mut commands: Commands) {
    // Msaa::Off: the hybrid blit pipeline's own RenderPipelineDescriptor
    // is single-sample — same fix examples/gallery.rs already applies,
    // for the identical reason (Bevy's default camera MSAA would make
    // the view target's color attachment incompatible with that
    // pipeline, a real wgpu validation error).
    commands.spawn((
        Camera3d::default(),
        Msaa::Off,
        // Hdr + a non-None Tonemapping: hybrid_pass's own blit writes raw
        // linear HDR straight into ViewTarget's main texture (see
        // hybrid_blit.wgsl's own doc comment) — without these, that main
        // texture is the swapchain's own low-dynamic-range format and
        // Bevy's Node3d::Tonemapping graph node no-ops entirely (see its
        // own `if !camera.hdr { return }` early-out), so any radiance
        // above 1.0 hard-clips to white with no filmic rolloff. Exposure
        // defaults to EV100 0.0 (Bevy's own default), which is neutral;
        // tune per-scene if the sun/lamp/projector intensities below read
        // too bright or too dark once tonemapped.
        Hdr,
        Tonemapping::default(),
        Exposure::default(),
        // TEMPORARY debug override — see this file's own top doc comment.
        // --corridor-cam: repositions to look straight down the
        // documented dark floor corridor (the strip between the -X wall
        // at x=-8.0 and the cube row at x~=-6.5..-6.8, see PROGRESS.md's
        // own "residual dark floor corridor" entries) instead of the
        // default CAMERA_CORNER framing, which doesn't clearly isolate
        // this specific region — needed for an honest DDGI-vs-cascades
        // A/B comparison actually targeting the bug this whole experiment
        // is about, not just whatever CAMERA_CORNER happens to show.
        if std::env::args().any(|a| a == "--corridor-cam") {
            Transform::from_translation(Vec3::new(-2.0, -ROOM_HALF_Y + 3.0, 6.0))
                .looking_at(Vec3::new(-7.2, -ROOM_HALF_Y + 0.3, -3.0), Vec3::Y)
        } else {
            Transform::from_translation(CAMERA_CORNER).looking_at(Vec3::ZERO, Vec3::Y)
        },
    ));
}

// ---------------------------------------------------------------------------
// Room shell: floor, 4 walls, and the (separately animated) roof panel.
// Each panel is its own SdfSceneRoot (mirrors gallery.rs's own one-root-
// per-object convention) rather than one CSG-unioned root, so the roof
// panel can move independently without dragging the rest of the shell's
// Transform along with it via a shared parent.
//
// **Panels overlap at every seam by `WALL_OVERLAP`, not a knife-edge
// butt joint.** A first version sized every panel to meet its
// neighbors at an EXACT shared plane (zero gap, zero overlap) — correct
// in exact CSG-boolean terms, but a real bug in this renderer: each
// panel is a SEPARATE, non-unioned SDF object, marched independently,
// and a ray grazing a seam at a shallow angle (exactly what a
// low-elevation sun does against a ceiling/wall corner) can slip
// through the mathematical zero-width gap between two unrelated
// `march_object` calls — found by direct visual inspection (the
// "sealed" room's own roof and corners read as lit, when nothing should
// be reaching the interior at all). Extending each side/end wall's own
// half-extent into the floor/roof's Y-range, and the end walls into the
// side walls' own X-range, closes every seam with real geometric
// overlap.
// ---------------------------------------------------------------------------

const WALL_OVERLAP: f32 = 0.2;

fn spawn_room(mut commands: Commands) {
    let white_wall = Material::new(Vec3::splat(0.92), 0.0, 0.35).with_reflectance(0.7);

    let mut panel = |translation: Vec3, half_extents: Vec3| {
        commands.spawn((
            SdfSceneRoot,
            Transform::from_translation(translation),
            Visibility::default(),
        ))
        .with_children(|parent| {
            parent.spawn((Shape::RoundedBox { half_extents, corner_radius: 0.0 }, Transform::IDENTITY, white_wall));
        });
    };

    // Floor: extended by WALL_OVERLAP in X/Z, same reasoning as the
    // roof panel below (see its own doc comment for the confirmed bug
    // this guards against) — the walls' Y-extension alone doesn't close
    // the floor-to-wall corner seam, only the roof-panel case was
    // actually observed leaking in practice (nothing samples light from
    // below the floor), but there's no reason to leave the identical
    // seam shape unfixed here just because it happened not to be
    // visually caught yet.
    let floor_half = Vec3::new(ROOM_HALF_X + WALL_OVERLAP, WALL_THICKNESS, ROOM_HALF_Z + WALL_OVERLAP);
    panel(Vec3::new(0.0, -ROOM_HALF_Y - WALL_THICKNESS, 0.0), floor_half);

    // +X / -X walls: full Y/Z footprint (Y extended by WALL_OVERLAP past
    // the floor/ceiling seam on both ends), thin on X.
    let side_wall_half = Vec3::new(WALL_THICKNESS, ROOM_HALF_Y + WALL_OVERLAP, ROOM_HALF_Z);
    panel(Vec3::new(ROOM_HALF_X + WALL_THICKNESS, 0.0, 0.0), side_wall_half);
    panel(Vec3::new(-ROOM_HALF_X - WALL_THICKNESS, 0.0, 0.0), side_wall_half);

    // +Z / -Z walls: full X/Y footprint (both extended by WALL_OVERLAP:
    // X past the side walls' own seam, Y past the floor/ceiling seam —
    // this panel meets FOUR neighbors, not two), thin on Z.
    let end_wall_half = Vec3::new(ROOM_HALF_X + WALL_OVERLAP, ROOM_HALF_Y + WALL_OVERLAP, WALL_THICKNESS);
    panel(Vec3::new(0.0, 0.0, ROOM_HALF_Z + WALL_THICKNESS), end_wall_half);
    panel(Vec3::new(0.0, 0.0, -ROOM_HALF_Z - WALL_THICKNESS), end_wall_half);

    // Roof panel: extended by WALL_OVERLAP in X/Z (a real, confirmed bug
    // in an earlier version of this file: the roof was left at exactly
    // ROOM_HALF_X/Z, meeting the walls at an EXACT zero-overlap plane
    // along the wall-to-roof corner line — the walls' own Y-extension
    // alone doesn't close that seam, only the Y-axis part of it. A DDGI
    // probe ray at a shallow angle could still find that knife-edge gap
    // and escape to sample `ddgi_ref::sky_color`'s own bright mocked-sky
    // gradient (confirmed NOT `BACKGROUND_COLOR`'s dark debug magenta —
    // a genuinely bright value), which DDGI's own trilinear probe
    // interpolation then smears across the whole ceiling/corners —
    // found by direct visual inspection: the "sealed" room's roof and
    // corners read as lit even with the roof panel provably centered.
    // Starts centered (fully sealing the room, sitting just above the
    // ceiling's own thin slab so it doesn't z-fight/CSG-merge with it)
    // — slid open by `slide_roof` once ROOF_DELAY_SECS has elapsed.
    let roof_half = Vec3::new(ROOM_HALF_X + WALL_OVERLAP, WALL_THICKNESS, ROOM_HALF_Z + WALL_OVERLAP);
    commands
        .spawn((RoofPanel, SdfSceneRoot, Transform::from_xyz(0.0, ROOM_HALF_Y + WALL_THICKNESS, 0.0), Visibility::default()))
        .with_children(|parent| {
            parent.spawn((
                Shape::RoundedBox { half_extents: roof_half, corner_radius: 0.0 },
                Transform::IDENTITY,
                white_wall,
            ));
        });
}

/// Slides `RoofPanel` open: stationary (fully sealed) for the first
/// `ROOF_DELAY_SECS`, then eases along +X over `ROOF_SLIDE_SECS`,
/// covering exactly `ROOM_HALF_X` world units total — starting flush
/// over the ceiling, ending flush against the +X wall, uncovering the
/// -X half of the room to whatever's above it (the sun, once the gap is
/// wide enough to matter). Smoothstep easing (not linear) so the very
/// first sliver of light grows in gently rather than snapping open.
fn slide_roof(time: Res<Time>, mut roof: Query<&mut Transform, With<RoofPanel>>) {
    let elapsed = time.elapsed_secs();
    let t = ((elapsed - ROOF_DELAY_SECS) / ROOF_SLIDE_SECS).clamp(0.0, 1.0);
    let eased = t * t * (3.0 - 2.0 * t); // smoothstep
    let offset = eased * ROOM_HALF_X;
    for mut transform in &mut roof {
        transform.translation.x = offset;
    }
}

// ---------------------------------------------------------------------------
// Cubes: 7 boxes of varying size/material, all placed in the ROOM's -X
// half (the half the roof panel eventually uncovers first, so the sun's
// arrival is immediately visible on them) — some clustered together,
// some isolated, some stacked, testing DDGI's bounce-light response to
// a genuinely varied, non-uniform arrangement rather than a regular
// grid.
// ---------------------------------------------------------------------------

fn spawn_cubes(mut commands: Commands) {
    let floor_y = -ROOM_HALF_Y;

    let mut cube = |center: Vec3, half_extent: f32, material: Material| {
        commands.spawn((
            SdfSceneRoot,
            Transform::from_translation(center),
            Visibility::default(),
        ))
        .with_children(|parent| {
            parent.spawn((
                Shape::RoundedBox { half_extents: Vec3::splat(half_extent), corner_radius: 0.02 },
                Transform::IDENTITY,
                material,
            ));
        });
    };

    // All 7 cubes sit in the room's -X half, spread across 4 distinct
    // Z-zones (-2.9, -1.0, 1.2, 3.2) so nothing is close to anything
    // else unless deliberately paired — every placement below was
    // checked against the room's own interior bounds, against every
    // other cube (real AABB-to-AABB distance, not just per-axis), and
    // against CAMERA_CORNER (real point-to-AABB distance, all >4 units)
    // before being written here; a first draft had the camera spawned
    // INSIDE one cube's own geometry (reading on screen as "everything
    // is one flat color") and two unrelated cube pairs overlapping —
    // both real bugs, not hypothetical ones this comment is guarding
    // against preemptively.

    // Zone 1 (z ~ -4.5..-4.7): a cluster of two cubes close together,
    // one large matte red, one small glossy blue — nearest the -Z wall,
    // first to catch direct sun once the roof gap reaches this end.
    let red = Material::new(Vec3::new(0.75, 0.2, 0.15), 0.0, 0.6);
    let blue = Material::new(Vec3::new(0.2, 0.35, 0.8), 0.1, 0.2).with_reflectance(0.7);
    cube(Vec3::new(-6.8, floor_y + 0.9, -4.5), 0.9, red);
    cube(Vec3::new(-5.2, floor_y + 0.5, -4.7), 0.5, blue);

    // Zone 2 (z ~ -1.5): a stack of two small cubes, metallic gold on
    // top of a dark matte base — tests DDGI bounce onto a vertically
    // stacked surface.
    let gold = Material::new(Vec3::new(0.85, 0.65, 0.2), 0.9, 0.25).with_reflectance(0.9);
    let dark_base = Material::new(Vec3::splat(0.08), 0.0, 0.7);
    cube(Vec3::new(-6.5, floor_y + 0.6, -1.5), 0.6, dark_base);
    cube(Vec3::new(-6.5, floor_y + 1.5, -1.5), 0.3, gold);

    // Zone 3 (z ~ 2.0..2.2): a small white cube and a larger purple
    // cube, offset in X from each other — both isolated from the
    // cluster/stack above and the green cube below.
    let white_cube = Material::new(Vec3::splat(0.85), 0.0, 0.4).with_reflectance(0.6);
    cube(Vec3::new(-6.5, floor_y + 0.5, 2.0), 0.5, white_cube);
    let purple = Material::new(Vec3::new(0.5, 0.25, 0.6), 0.2, 0.45);
    cube(Vec3::new(-4.0, floor_y + 1.0, 2.2), 1.0, purple);

    // Zone 4 (z ~ 4.8): an isolated matte green cube near the +Z wall —
    // farthest from the cluster/stack, its own distinct corner of the
    // sunlit half.
    let green = Material::new(Vec3::new(0.25, 0.7, 0.3), 0.0, 0.5);
    cube(Vec3::new(-6.5, floor_y + 0.7, 4.8), 0.7, green);

    // Zone 5 (x ~ 2.0, z ~ 0.0): a clear glass cube in the room's
    // otherwise-empty +X half, isolated from every cube above (all sit at
    // x <= -4.0, real AABB distance >5 units) and from CAMERA_CORNER
    // (real point-to-AABB distance >5 units) — positioned roughly along
    // the camera's own sightline toward the gold/red cluster so a
    // straight-through refracted ray has real, colorful geometry to pick
    // up on the far side, the same "prove it actually looks through,
    // don't just prove it doesn't crash" standard reflect_ref.rs's own
    // mirror-floor fixture already set. Clear (white base_color -> zero
    // Beer-Lambert absorption), moderate ior (glass-like).
    let glass = Material::new(Vec3::ONE, 0.0, 0.02).with_reflectance(0.9).with_transmission(1.0).with_ior(1.5);
    cube(Vec3::new(2.0, floor_y + 1.0, 0.0), 1.0, glass);
}

// ---------------------------------------------------------------------------
// Lights: a directional sun at a real daylight angle/intensity, only
// able to reach the interior once the roof gap is wide enough for its
// parallel rays to clear the opening — the room's only light source.
// ---------------------------------------------------------------------------

fn spawn_lights(mut commands: Commands) {
    // Sun: 35 degrees above the horizon (verified by direct computation
    // of the resulting forward vector's own elevation angle — a first
    // draft's `looking_at` target was picked by eye and landed at 84
    // degrees, nearly straight down, not the requested 30-40; a
    // `looking_at` target's horizontal offset has to be large relative
    // to the light's own height above that target for the angle to
    // read as "low sun," which "eyeballing small numbers" doesn't
    // reliably produce), genuine daylight-scale illuminance (unlike
    // gallery.rs's own deliberately-dimmed 1500 lux, this scene has no
    // competing point/spot light to wash out — the whole point is a
    // stark, real contrast between "sealed room" and "sun gets in").
    // Aimed with its horizontal component mostly along -X (with a
    // little -Z) so its rays rake across the -X half of the room (the
    // half the roof uncovers, where every cube sits) once they clear
    // the gap, rather than falling straight down through only the
    // gap's own footprint.
    commands.spawn((
        SunLight,
        DirectionalLight { color: Color::srgb(1.0, 0.97, 0.9), illuminance: 20_000.0, ..default() },
        Transform::from_xyz(0.0, 10.0, 0.0).looking_at(Vec3::new(-7.63, 4.26, -2.97), Vec3::Y),
    ));

}

// ---------------------------------------------------------------------------
// Minimal HUD: countdown to roof-open, current gap fraction, GPU pass
// timings — enough to follow what's happening without the full
// gallery.rs control panel (this scene has no --stress/--spin/shape
// picker to expose).
// ---------------------------------------------------------------------------

#[derive(Component)]
struct HudText;

fn spawn_hud(mut commands: Commands) {
    commands.spawn((
        HudText,
        Text::new(""),
        TextFont { font_size: FontSize::Px(16.0), ..default() },
        Node { position_type: PositionType::Absolute, top: Val::Px(10.0), left: Val::Px(10.0), ..default() },
    ));
}

fn update_hud(time: Res<Time>, mut text: Query<&mut Text, With<HudText>>) {
    let Ok(mut text) = text.single_mut() else { return };
    let elapsed = time.elapsed_secs();
    let gap_fraction = ((elapsed - ROOF_DELAY_SECS) / ROOF_SLIDE_SECS).clamp(0.0, 1.0);
    let status = if elapsed < ROOF_DELAY_SECS {
        format!("roof sealed — opens in {:.1}s", ROOF_DELAY_SECS - elapsed)
    } else if gap_fraction < 1.0 {
        format!("roof opening — {:.0}% of half-gap", gap_fraction * 100.0)
    } else {
        "roof open (half ceiling exposed)".to_string()
    };
    **text = format!("t {elapsed:.1}s   {status}");
}

// ---------------------------------------------------------------------------
// egui controls panel: cone-trace/shadow tuning only — this scene's
// whole point is the room/lighting setup itself, not a knob showcase.
// ---------------------------------------------------------------------------

#[allow(clippy::too_many_arguments)]
fn controls_panel(
    mut contexts: EguiContexts,
    mut gi_method: ResMut<GiMethodConfig>,
    mut ddgi_config: ResMut<DdgiConfig>,
    mut conetrace_config: ResMut<ConeTraceConfig>,
    mut reflection_config: ResMut<ReflectionConfig>,
    mut transmission_config: ResMut<TransmissionConfig>,
    mut hybrid_post_config: ResMut<HybridPostConfig>,
    mut dof_config: ResMut<DofConfig>,
    mut shadow_config: ResMut<ShadowConfig>,
    diagnostics: Res<DiagnosticsStore>,
) -> Result {
    let ctx = contexts.ctx_mut()?;
    egui::Window::new("GI Room").anchor(egui::Align2::RIGHT_TOP, egui::vec2(-10.0, 10.0)).show(ctx, |ui| {
        ui.label("Shadows");
        ui.checkbox(&mut shadow_config.enabled, "Soft shadows");
        ui.separator();
        ui.label("GI method");
        ui.horizontal(|ui| {
            ui.radio_value(&mut gi_method.method, GiMethod::None, "None");
            ui.radio_value(&mut gi_method.method, GiMethod::Ddgi, "DDGI");
            ui.radio_value(&mut gi_method.method, GiMethod::RadianceCascades, "Radiance Cascades (experimental)");
            ui.radio_value(&mut gi_method.method, GiMethod::ConeTrace, "Cone-trace");
        });
        let ddgi_active = gi_method.method == GiMethod::Ddgi;
        ui.add_enabled(
            ddgi_active,
            egui::Slider::new(&mut ddgi_config.probes_per_frame, 1..=4096).text("Probes relit/frame"),
        );
        ui.add_enabled(ddgi_active, egui::Slider::new(&mut ddgi_config.probe_spacing, 0.5..=5.0).text("Probe spacing"));
        ui.add_enabled(ddgi_active, egui::Slider::new(&mut ddgi_config.max_t, 1.0..=60.0).text("Probe ray reach"));
        let conetrace_active = gi_method.method == GiMethod::ConeTrace;
        ui.add_enabled(
            conetrace_active,
            egui::Slider::new(&mut conetrace_config.cone_half_angle, 0.01..=0.6).text("Cone half-angle (rad)"),
        );
        ui.add_enabled(
            conetrace_active,
            egui::Slider::new(&mut conetrace_config.cone_origin_radius, 0.0..=1.0).text("Cone origin radius"),
        );
        ui.add_enabled(conetrace_active, egui::Slider::new(&mut conetrace_config.max_t, 1.0..=60.0).text("Cone reach"));
        ui.add_enabled(conetrace_active, egui::Slider::new(&mut conetrace_config.max_bounces, 1..=8).text("Bounces"));
        ui.separator();
        ui.label("Reflections");
        ui.checkbox(&mut reflection_config.enabled, "Multi-bounce specular reflections");
        ui.add_enabled(reflection_config.enabled, egui::Slider::new(&mut reflection_config.max_bounces, 1..=4).text("Bounces"));
        ui.add_enabled(
            reflection_config.enabled,
            egui::Slider::new(&mut reflection_config.fresnel_cutoff, 0.0..=0.2).text("Fresnel cutoff"),
        );
        ui.separator();
        ui.label("Transmission");
        ui.checkbox(&mut transmission_config.enabled, "Multi-bounce transmission/refraction");
        ui.add_enabled(
            transmission_config.enabled,
            egui::Slider::new(&mut transmission_config.max_bounces, 1..=4).text("Bounces"),
        );
        ui.add_enabled(
            transmission_config.enabled,
            egui::Slider::new(&mut transmission_config.fresnel_cutoff, 0.0..=0.2).text("Fresnel cutoff"),
        );
        ui.separator();
        ui.label("Lens / sensor");
        ui.add(egui::Slider::new(&mut hybrid_post_config.grain_strength, 0.0..=0.3).text("Film grain"));
        ui.add(egui::Slider::new(&mut hybrid_post_config.vignette_strength, 0.0..=1.0).text("Vignette"));
        ui.add(egui::Slider::new(&mut hybrid_post_config.aberration_strength, 0.0..=0.02).text("Chromatic aberration"));
        ui.checkbox(&mut dof_config.enabled, "Depth of field (stochastic)");
        ui.add_enabled(
            dof_config.enabled,
            egui::Slider::new(&mut dof_config.focal_distance, 0.5..=20.0).text("Focal distance"),
        );
        ui.add_enabled(
            dof_config.enabled,
            egui::Slider::new(&mut dof_config.aperture_f_stops, 0.5..=16.0).text("Aperture f-stop"),
        );
        ui.add_enabled(
            dof_config.enabled,
            egui::Slider::new(&mut dof_config.max_history_length, 4.0..=64.0).text("Convergence window (frames)"),
        );
        ui.separator();
        // Cone tracing has no separate GPU pass of its own (see
        // conetrace_ref.rs's own doc comment) — its entire cost shows up
        // as a delta in the trace line below relative to GiMethod::None's
        // own baseline trace cost, not as a distinct timing line here.
        // Reflection's/transmission's own temporal-accumulation passes DO
        // have separate dispatches (hybrid_reflect_temporal/
        // hybrid_transmit_temporal), shown below.
        if let Some(trace_ms) = gpu_pass_ms(&diagnostics, "hybrid_trace") {
            ui.label(format!("trace: {trace_ms:.2} ms"));
        }
        if let Some(ddgi_ms) = gpu_pass_ms(&diagnostics, "hybrid_ddgi") {
            ui.label(format!("ddgi relight: {ddgi_ms:.2} ms"));
        }
        if let Some(radiance_cascades_ms) = gpu_pass_ms(&diagnostics, "hybrid_radiance_cascades") {
            ui.label(format!("radiance cascades relight: {radiance_cascades_ms:.2} ms"));
        }
        if let Some(reflect_temporal_ms) = gpu_pass_ms(&diagnostics, "hybrid_reflect_temporal") {
            ui.label(format!("reflect temporal: {reflect_temporal_ms:.2} ms"));
        }
        if let Some(transmit_temporal_ms) = gpu_pass_ms(&diagnostics, "hybrid_transmit_temporal") {
            ui.label(format!("transmit temporal: {transmit_temporal_ms:.2} ms"));
        }
        if let Some(dof_ms) = gpu_pass_ms(&diagnostics, "hybrid_dof") {
            ui.label(format!("dof: {dof_ms:.2} ms"));
        }
    });
    Ok(())
}

fn gpu_pass_ms(diagnostics: &DiagnosticsStore, pass_name: &str) -> Option<f32> {
    let path = DiagnosticPath::from_components(["render", pass_name, "elapsed_gpu"]);
    diagnostics.get(&path).and_then(|d| d.smoothed()).map(|v| v as f32)
}
