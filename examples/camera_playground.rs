//! The third-person camera with a walking character you control.
//!
//! A walker on a flat field among walls, pillars, a roofed corridor, a low
//! tunnel and crates, followed by `ThirdPersonCameraPlugin`. The geometry
//! has static colliders already, for the camera collision of the next phase;
//! the character walks on flat ground and does not collide with it yet.
//!
//! Controls (click the window to grab the mouse, Esc to release):
//! - mouse or right stick: look; scroll or D-pad up/down: zoom;
//! - WASD or left stick: walk, relative to the camera; Shift or right
//!   trigger: run; Space or South: jump (from a stand);
//! - C or West: toggle combat mode; R, middle mouse or R3: recentre;
//!   Tab or L3: swap shoulder; T: teleport ahead (a cut);
//! - F9: write the recording (`--record PATH`).
//!
//! Run: `cargo run --release --example camera_playground`
//!
//! Flags:
//! - `--script orbit|walk|tour`: scripted input instead of devices, for
//!   reproducible shots;
//! - `--shot PATH --at-frame N`: a screenshot, then exit;
//! - `--step-seconds S`: a fixed step per frame (offscreen runs);
//! - `--debug-view`: a top-down inset with the rig drawn (pivot, shoulder,
//!   eye, boom);
//! - `--record PATH`: record every camera frame to a `.camtrace.ron`
//!   (written on F9 and on exit);
//! - `--replay PATH`: replay a recording through the pure pipeline, print
//!   its metrics, and exit (no window).

use std::f32::consts::PI;

use avian3d::prelude::*;
use bevy::camera::Viewport;
use bevy::diagnostic::FrameCount;
use bevy::input::gamepad::{Gamepad, GamepadButton};
use bevy::prelude::*;
use bevy::camera::visibility::RenderLayers;
use bevy::render::view::window::screenshot::{save_to_disk, Screenshot};
use bevy::window::{CursorGrabMode, CursorOptions, PrimaryWindow};

use migera::camera::harness::measure;
use migera::camera::rig::yaw_of;
use migera::camera::{
    CameraConfig, CameraDeviceInput, CameraInput, CameraInputSettings, CameraModeRequests,
    CameraRecorder, CameraTarget, CameraTrace, CameraView, ModeId, ModeRequest,
    ThirdPersonCamera, ThirdPersonCameraPlugin,
};
use migera::character::anim::asset::AnimAssetPlugin;
use migera::character::anim::jump::JumpAsk;
use migera::character::anim::{spawn_gltf_humanoid, AnimPlugin, HumanoidPlugin, Steer, Walker, WalkerPlugin, WalkerSet};

#[derive(Resource, Clone, Default)]
struct Config {
    script: Option<String>,
    shot: Option<(String, u32)>,
    step_seconds: Option<f64>,
    debug_view: bool,
    record: Option<String>,
}

impl Config {
    fn from_args() -> (Self, Option<String>) {
        let mut config = Config::default();
        let mut replay = None;
        let (mut shot, mut at) = (None, 120);
        let mut args = std::env::args().skip(1);
        while let Some(arg) = args.next() {
            let mut value = || args.next().unwrap_or_else(|| panic!("{arg} needs a value"));
            match arg.as_str() {
                "--script" => config.script = Some(value()),
                "--shot" => shot = Some(value()),
                "--at-frame" => at = value().parse().expect("--at-frame N"),
                "--step-seconds" => config.step_seconds = Some(value().parse().expect("--step-seconds S")),
                "--debug-view" => config.debug_view = true,
                "--record" => config.record = Some(value()),
                "--replay" => replay = Some(value()),
                other => panic!("unknown flag {other}; see the example's doc comment"),
            }
        }
        config.shot = shot.map(|path| (path, at));
        (config, replay)
    }
}

fn main() {
    let (config, replay) = Config::from_args();
    if let Some(path) = replay {
        replay_and_report(&path);
        return;
    }
    let assets = std::env::current_dir().expect("cwd").join("assets").to_string_lossy().into_owned();
    let mut app = App::new();
    if let Some(seconds) = config.step_seconds {
        app.insert_resource(bevy::time::TimeUpdateStrategy::ManualDuration(std::time::Duration::from_secs_f64(seconds)));
    }
    app.add_plugins(
        DefaultPlugins
            .set(WindowPlugin {
                primary_window: Some(Window { title: "migera camera playground".into(), ..default() }),
                ..default()
            })
            .set(AssetPlugin { file_path: assets, ..default() }),
    )
    .add_plugins((bevy::remote::RemotePlugin::default(), bevy::remote::http::RemoteHttpPlugin::default()))
    .add_plugins(PhysicsPlugins::default())
    .add_plugins((AnimPlugin, AnimAssetPlugin, HumanoidPlugin, WalkerPlugin, ThirdPersonCameraPlugin))
    .insert_resource(config)
    .init_resource::<Controls>()
    .init_gizmo_group::<RigGizmos>()
    .add_systems(Startup, (spawn_world, spawn_player_and_camera, rig_gizmos_on_their_layer))
    .add_systems(Update, (grab_cursor, read_controls, run_script, apply_controls).chain().before(WalkerSet::Drive))
    .add_systems(Update, (draw_rig, update_debug_view, update_hud, save_recording, auto_shot))
    .run();
}

/// Replays a recorded trace through the pure pipeline and prints what a
/// player would have felt.
fn replay_and_report(path: &str) {
    let text = std::fs::read_to_string(path).unwrap_or_else(|e| panic!("reading {path}: {e}"));
    let trace = CameraTrace::from_ron(&text).unwrap_or_else(|e| panic!("parsing {path}: {e}"));
    let outputs = trace.replay(&CameraInputSettings::default(), &CameraConfig::default());
    let m = measure(&trace, &outputs);
    // The followed target's own peak acceleration, and where the eye's
    // peaks: a camera can only be as smooth as what it follows, less what
    // its springs filter out.
    let accel = |points: &[Vec3], dt: &[f32], i: usize| {
        let v1 = (points[i] - points[i - 1]) / dt[i];
        let v0 = (points[i - 1] - points[i - 2]) / dt[i - 1];
        (v1 - v0).length() / dt[i]
    };
    let dts: Vec<f32> = trace.frames.iter().map(|f| f.clock.real_dt).collect();
    let targets: Vec<Vec3> = trace.frames.iter().map(|f| f.target.position).collect();
    let eyes: Vec<Vec3> = outputs.iter().map(|o| o.pose.eye).collect();
    let peak = |points: &[Vec3]| {
        (3..points.len()).map(|i| (i, accel(points, &dts, i))).fold((0, 0.0f32), |a, b| if b.1 > a.1 { b } else { a })
    };
    let (target_frame, target_peak) = peak(&targets);
    let (eye_frame, eye_peak) = peak(&eyes);
    println!("target peak accel {target_peak:.1} m/s² at frame {target_frame}; eye peak {eye_peak:.1} m/s² at frame {eye_frame}");
    println!(
        "{path}: {} frames, max roll {:.2e}, max eye speed {:.2} m/s, max eye accel {:.1} m/s², \
         pitch {:.1}°..{:.1}°",
        m.frames,
        m.max_roll,
        m.max_eye_speed,
        m.max_eye_acceleration,
        m.min_pitch.to_degrees(),
        m.max_pitch.to_degrees(),
    );
}

#[derive(Component)]
struct Player;

#[derive(Component)]
struct PlayerCamera;

#[derive(Component)]
struct DebugCamera;

#[derive(Component)]
struct Hud;

/// The rig's own drawing, on a render layer only the debug inset sees: drawn
/// from the gameplay camera, the eye marker sits on the camera itself and
/// smears across the whole view.
#[derive(Default, Reflect, GizmoConfigGroup)]
struct RigGizmos;

const RIG_LAYER: usize = 1;

fn rig_gizmos_on_their_layer(mut store: ResMut<GizmoConfigStore>) {
    store.config_mut::<RigGizmos>().0.render_layers = RenderLayers::layer(RIG_LAYER);
}

/// The player's intent this frame, from devices or a script.
#[derive(Resource, Default)]
struct Controls {
    /// Movement stick: x right, y forward.
    movement: Vec2,
    run: bool,
    jump: bool,
    toggle_combat: bool,
    teleport: bool,
    /// Script only: look stick and recentre, written straight into
    /// `CameraInput` (with devices, `CameraDeviceInput` does that).
    look_stick: Option<Vec2>,
    recenter: bool,
    combat: bool,
}

const GROUND: Color = Color::srgb(0.32, 0.36, 0.3);
const STONE: Color = Color::srgb(0.55, 0.53, 0.5);
const WOOD: Color = Color::srgb(0.5, 0.36, 0.22);

fn spawn_world(mut commands: Commands, mut meshes: ResMut<Assets<Mesh>>, mut materials: ResMut<Assets<StandardMaterial>>) {
    // A darker tile grid on the ground, so motion reads on screen.
    let tile = meshes.add(Cuboid::new(4.0, 0.01, 4.0));
    let tile_material = materials.add(StandardMaterial { base_color: Color::srgb(0.27, 0.31, 0.26), perceptual_roughness: 1.0, ..default() });
    for i in -8..=8 {
        for j in -8..=8 {
            if (i + j) % 2 == 0 {
                commands.spawn((
                    Mesh3d(tile.clone()),
                    MeshMaterial3d(tile_material.clone()),
                    Transform::from_xyz(i as f32 * 4.0, 0.005, j as f32 * 4.0),
                ));
            }
        }
    }
    let mut solid = |commands: &mut Commands, size: Vec3, at: Vec3, color: Color| {
        commands.spawn((
            Mesh3d(meshes.add(Cuboid::from_size(size))),
            MeshMaterial3d(materials.add(StandardMaterial { base_color: color, perceptual_roughness: 0.9, ..default() })),
            Transform::from_translation(at),
            RigidBody::Static,
            Collider::cuboid(size.x, size.y, size.z),
        ));
    };
    solid(&mut commands, Vec3::new(80.0, 0.2, 80.0), Vec3::new(0.0, -0.1, 0.0), GROUND);
    // A long wall to the west, to back the camera into.
    solid(&mut commands, Vec3::new(0.6, 3.0, 20.0), Vec3::new(-6.0, 1.5, -4.0), STONE);
    // A row of pillars to the east.
    for k in 0..6 {
        solid(&mut commands, Vec3::new(0.6, 4.0, 0.6), Vec3::new(5.0, 2.0, -2.0 - 3.0 * k as f32), STONE);
    }
    // A roofed corridor north: two walls 2.4 m apart under a 2.6 m roof.
    solid(&mut commands, Vec3::new(0.4, 2.6, 10.0), Vec3::new(-1.4, 1.3, -20.0), STONE);
    solid(&mut commands, Vec3::new(0.4, 2.6, 10.0), Vec3::new(1.4, 1.3, -20.0), STONE);
    solid(&mut commands, Vec3::new(3.2, 0.3, 10.0), Vec3::new(0.0, 2.75, -20.0), STONE);
    // A low tunnel south: 2.0 m ceiling.
    solid(&mut commands, Vec3::new(4.0, 0.3, 6.0), Vec3::new(0.0, 2.15, 12.0), WOOD);
    solid(&mut commands, Vec3::new(0.3, 2.0, 6.0), Vec3::new(-2.0, 1.0, 12.0), WOOD);
    solid(&mut commands, Vec3::new(0.3, 2.0, 6.0), Vec3::new(2.0, 1.0, 12.0), WOOD);
    // Crates.
    for (x, z, s) in [(8.0, 6.0, 1.0), (9.2, 6.4, 0.8), (8.4, 7.4, 1.2), (-9.0, 8.0, 1.5)] {
        solid(&mut commands, Vec3::splat(s), Vec3::new(x, s * 0.5, z), WOOD);
    }
    commands.spawn((
        DirectionalLight { illuminance: 11_000.0, shadow_maps_enabled: true, ..default() },
        Transform::from_rotation(Quat::from_euler(EulerRot::YXZ, 0.7, -0.95, 0.0)),
    ));
    commands.insert_resource(GlobalAmbientLight { brightness: 450.0, ..default() });
}

fn spawn_player_and_camera(mut commands: Commands, asset_server: Res<AssetServer>, config: Res<Config>) {
    // `puppet_base.gltf` faces +Z; this crate's forward is −Z.
    let player = spawn_gltf_humanoid(&mut commands, &asset_server, "models/puppet_base.gltf", PI, Transform::from_xyz(0.0, 0.0, 4.0));
    commands.entity(player).insert((Player, Walker { speed: 0.0, ..default() }, CameraTarget::default()));

    let camera = commands
        .spawn((Camera3d::default(), ThirdPersonCamera::follow(player), Transform::from_xyz(0.0, 2.5, 8.0), PlayerCamera))
        .id();
    if config.script.is_none() {
        commands.entity(camera).insert(CameraDeviceInput::default());
    }
    if config.record.is_some() {
        commands.entity(camera).insert(CameraRecorder::default());
    }
    if config.debug_view {
        commands.spawn((
            Camera3d::default(),
            Camera { order: 1, ..default() },
            Transform::from_xyz(0.0, 30.0, 0.0).looking_at(Vec3::ZERO, Vec3::NEG_Z),
            RenderLayers::from_layers(&[0, RIG_LAYER]),
            DebugCamera,
        ));
    }
    commands.spawn((
        Text::new(""),
        TextFont { font_size: bevy::text::FontSize::Px(15.0), ..default() },
        Node { position_type: PositionType::Absolute, left: px(10.0), top: px(10.0), ..default() },
        // On the player's camera, not the debug inset (UI otherwise picks
        // the highest-order camera).
        UiTargetCamera(camera),
        Hud,
    ));
}

/// Click to grab the mouse for looking, Esc to let it go.
fn grab_cursor(mouse: Res<ButtonInput<MouseButton>>, keys: Res<ButtonInput<KeyCode>>, mut cursor: Query<&mut CursorOptions, With<PrimaryWindow>>, config: Res<Config>) {
    if config.script.is_some() {
        return;
    }
    let Ok(mut cursor) = cursor.single_mut() else { return };
    if mouse.just_pressed(MouseButton::Left) {
        cursor.grab_mode = CursorGrabMode::Locked;
        cursor.visible = false;
    }
    if keys.just_pressed(KeyCode::Escape) {
        cursor.grab_mode = CursorGrabMode::None;
        cursor.visible = true;
    }
}

fn read_controls(keys: Res<ButtonInput<KeyCode>>, gamepads: Query<&Gamepad>, mut controls: ResMut<Controls>, config: Res<Config>) {
    if config.script.is_some() {
        return;
    }
    let mut movement = Vec2::ZERO;
    for (key, dir) in [(KeyCode::KeyW, Vec2::Y), (KeyCode::KeyS, Vec2::NEG_Y), (KeyCode::KeyD, Vec2::X), (KeyCode::KeyA, Vec2::NEG_X)] {
        if keys.pressed(key) {
            movement += dir;
        }
    }
    let mut run = keys.pressed(KeyCode::ShiftLeft);
    let mut jump = keys.just_pressed(KeyCode::Space);
    let mut toggle = keys.just_pressed(KeyCode::KeyC);
    if let Some(pad) = gamepads.iter().next() {
        let stick = pad.left_stick();
        if stick.length() > 0.2 {
            movement = stick;
        }
        run |= pad.pressed(GamepadButton::RightTrigger2);
        jump |= pad.just_pressed(GamepadButton::South);
        toggle |= pad.just_pressed(GamepadButton::West);
    }
    controls.movement = movement.clamp_length_max(1.0);
    controls.run = run;
    controls.jump = jump;
    controls.toggle_combat = toggle;
    controls.teleport = keys.just_pressed(KeyCode::KeyT);
    controls.look_stick = None;
    controls.recenter = false;
}

/// Scripted input, as a function of time, for reproducible shots.
fn run_script(time: Res<Time>, config: Res<Config>, mut controls: ResMut<Controls>) {
    let Some(script) = config.script.as_deref() else { return };
    let t = time.elapsed_secs();
    let (movement, look, run, combat) = match script {
        // Stand still while the camera circles: shows the orbit and pitch.
        "orbit" => (Vec2::ZERO, Vec2::new(0.5, if t > 4.0 { 0.4 } else { 0.0 }), false, false),
        // Walk forward, swing the camera right mid-walk; the character
        // turns with the camera because movement is camera-relative.
        "walk" => (
            if t > 1.0 { Vec2::Y } else { Vec2::ZERO },
            if (3.0..4.0).contains(&t) { Vec2::new(0.8, 0.0) } else { Vec2::ZERO },
            false,
            false,
        ),
        // Walk, run, combat mode, back to exploration.
        "tour" => (
            if t > 1.0 { Vec2::new(if t > 6.0 { 0.6 } else { 0.0 }, 1.0).normalize() } else { Vec2::ZERO },
            if (2.0..2.8).contains(&t) { Vec2::new(-0.7, 0.0) } else { Vec2::ZERO },
            (4.0..6.0).contains(&t),
            (7.0..10.0).contains(&t),
        ),
        other => panic!("unknown script {other}: orbit, walk or tour"),
    };
    controls.movement = movement;
    controls.run = run;
    controls.look_stick = Some(look);
    controls.toggle_combat = combat != controls.combat;
    controls.jump = false;
    controls.teleport = false;
    controls.recenter = false;
}

/// Camera-relative movement onto the walker, and the camera's own asks.
fn apply_controls(
    mut controls: ResMut<Controls>,
    mut players: Query<(&mut Walker, &mut Transform), With<Player>>,
    mut cameras: Query<(&CameraView, &mut CameraInput, &mut CameraModeRequests), With<PlayerCamera>>,
) {
    let Ok((view, mut input, mut requests)) = cameras.single_mut() else { return };
    let Ok((mut walker, mut transform)) = players.single_mut() else { return };
    let held = controls.movement.length() > 0.1;
    input.move_held = held;
    if let Some(look) = controls.look_stick {
        input.look_stick = look;
    }
    input.recenter |= controls.recenter;
    if held {
        let direction = view.control_forward() * controls.movement.y + view.control_right() * controls.movement.x;
        walker.steer = Steer::Toward { yaw: yaw_of(direction), rate: 5.0 };
        walker.speed = if controls.run { 3.5 } else { 1.4 } * controls.movement.length();
    } else {
        walker.speed = 0.0;
    }
    if controls.jump && !held {
        walker.jump = Some(JumpAsk::up(0.35));
    }
    if controls.toggle_combat {
        controls.combat = !controls.combat;
        let id = if controls.combat { "combat" } else { "explore" };
        requests.0.push(ModeRequest { id: ModeId::new(id), blend: 0.5 });
    }
    if controls.teleport {
        // Not how a game should move a character (root motion owns it);
        // here only to make a cut.
        transform.translation += view.control_forward() * 8.0;
    }
}

/// The rig drawn as gizmos: pivot, shoulder point, boom, eye.
fn draw_rig(mut gizmos: Gizmos<RigGizmos>, cameras: Query<(&CameraView, &migera::camera::CameraDesiredPose), With<PlayerCamera>>) {
    for (view, desired) in &cameras {
        let pose = desired.0;
        gizmos.sphere(Isometry3d::from_translation(pose.pivot), 0.08, Color::srgb(1.0, 0.8, 0.1));
        gizmos.line(pose.pivot, pose.shoulder, Color::srgb(1.0, 0.5, 0.1));
        gizmos.line(pose.shoulder, pose.eye, Color::srgb(0.2, 0.8, 1.0));
        gizmos.sphere(Isometry3d::from_translation(pose.eye), 0.12, Color::srgb(0.2, 0.8, 1.0));
        let flat = view.control_forward();
        gizmos.arrow(pose.pivot - Vec3::Y * 1.5, pose.pivot - Vec3::Y * 1.5 + flat * 1.2, Color::srgb(0.4, 1.0, 0.4));
    }
}

/// The debug inset: a top-down view over the pivot, bottom-right quarter.
fn update_debug_view(
    windows: Query<&Window, With<PrimaryWindow>>,
    views: Query<&CameraView, With<PlayerCamera>>,
    mut debug: Query<(&mut Camera, &mut Transform), With<DebugCamera>>,
) {
    let (Ok(window), Ok(view)) = (windows.single(), views.single()) else { return };
    for (mut camera, mut transform) in &mut debug {
        let size = UVec2::new(window.physical_width() / 3, window.physical_height() / 3);
        camera.viewport = Some(Viewport {
            physical_position: UVec2::new(window.physical_width() - size.x, window.physical_height() - size.y),
            physical_size: size,
            ..default()
        });
        *transform = Transform::from_translation(view.pivot + Vec3::Y * 14.0).looking_at(view.pivot, Vec3::NEG_Z);
    }
}

fn update_hud(cameras: Query<(&CameraView, &migera::camera::CameraRigState), With<PlayerCamera>>, mut hud: Query<&mut Text, With<Hud>>) {
    let (Ok((view, state)), Ok(mut text)) = (cameras.single(), hud.single_mut()) else { return };
    let mode = state.rig.as_ref().map(|rig| rig.stack.top().id.0.clone()).unwrap_or_default();
    let distance = state.output.map(|o| o.pose.distance).unwrap_or_default();
    text.0 = format!(
        "mode {mode}  yaw {:6.1} deg  pitch {:5.1} deg  boom {distance:.2} m  fov {:.0} deg{}\n\
         click: grab mouse | WASD/stick: walk | Shift: run | C: combat | R: recentre | Tab: shoulder | T: teleport",
        view.view_yaw.to_degrees(),
        view.pitch.to_degrees(),
        view.fov.to_degrees(),
        if view.latched { "  [controls latched]" } else { "" },
    );
}

fn save_recording(keys: Res<ButtonInput<KeyCode>>, config: Res<Config>, recorders: Query<&CameraRecorder>) {
    if keys.just_pressed(KeyCode::F9) {
        write_recording(&config, &recorders);
    }
}

fn write_recording(config: &Config, recorders: &Query<&CameraRecorder>) {
    let (Some(path), Ok(recorder)) = (&config.record, recorders.single()) else { return };
    match recorder.trace.to_ron() {
        Ok(text) => match std::fs::write(path, text) {
            Ok(()) => info!("wrote {} camera frames to {path}", recorder.trace.frames.len()),
            Err(e) => error!("writing {path}: {e}"),
        },
        Err(e) => error!("serializing the recording: {e}"),
    }
}

fn auto_shot(config: Res<Config>, frame: Res<FrameCount>, mut commands: Commands, mut fired: Local<bool>, recorders: Query<&CameraRecorder>) {
    let Some((path, at)) = config.shot.clone() else { return };
    if !*fired && frame.0 >= at {
        *fired = true;
        commands.spawn(Screenshot::primary_window()).observe(save_to_disk(path));
    }
    if *fired && frame.0 >= at + 30 {
        write_recording(&config, &recorders);
        std::process::exit(0);
    }
}
