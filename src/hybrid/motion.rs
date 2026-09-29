//! Per-object previous-frame transform tracking, for the temporal-
//! accumulation GI pass's per-object motion vectors (see `cpu_ref.rs`'s
//! temporal-reprojection doc comments for the full picture).
//!
//! Mirrors `bevy_pbr::prepass::PreviousGlobalTransform`/
//! `update_mesh_previous_global_transforms`
//! (`bevy_pbr-0.19.1/src/prepass/mod.rs:216-245`) — that component exists
//! and its populating system runs unconditionally in this app (any
//! `Material`/`PbrPlugin` load wires up `PrepassPlugin`), but its query
//! filter is hard-coded to `With<Mesh3d>`. This renderer's `Shape`
//! entities never carry `Mesh3d` (`src/hybrid` traces SDF primitives, not
//! Bevy meshes), so Bevy's own component is permanently inert for them.
//! This module reuses the PATTERN — a change-detected `PreUpdate` system
//! snapshotting last frame's transform before this frame's `Update`
//! (object animation, e.g. `examples/gallery.rs`'s `spin_objects`) and
//! `PostUpdate` (Bevy's own transform propagation, which is what actually
//! writes `GlobalTransform` from `Transform`) run — not the code.
//!
//! Stored as `translation`/`rotation` (not a raw `Affine3A`, unlike
//! Bevy's own version) because that's the exact shape
//! `extract::object_gpu_from` already consumes from the CURRENT frame's
//! `GlobalTransform` (via `.translation()`/`.rotation()`) — storing the
//! same shape for the previous frame avoids an extra decomposition step
//! at extraction time.

use bevy::prelude::*;

use crate::sdf::components::Shape;

/// Last frame's world-space translation/rotation of a `Shape` entity —
/// absent until this component has run at least once past that entity's
/// spawn frame (see `update_previous_shape_transforms`'s own doc comment
/// for the "freshly spawned, no history yet" case).
#[derive(Component, Clone, Copy, Debug, PartialEq)]
pub struct PreviousShapeTransform {
    pub translation: Vec3,
    pub rotation: Quat,
}

/// `PreUpdate` main-world system: last frame's `GlobalTransform` becomes
/// this frame's `PreviousShapeTransform`, mirroring
/// `update_mesh_previous_global_transforms`'s exact shape (new entities
/// get an initial snapshot equal to their own current transform — "no
/// motion yet" is the correct first-frame answer, not a missing/garbage
/// value; existing entities update only when `GlobalTransform` actually
/// changed since the last snapshot, via Bevy's own change-detection
/// rather than snapshotting unconditionally every frame).
///
/// Registered in `PreUpdate` specifically: Bevy's schedule order is
/// `PreUpdate -> Update -> PostUpdate` every frame, and `GlobalTransform`
/// is only written during `PostUpdate`'s transform-propagation system
/// (`bevy_transform::plugins`, confirmed via that crate's own
/// `add_systems(PostUpdate, ...)` registration) — so by the time THIS
/// system runs in frame N's `PreUpdate`, `GlobalTransform` still holds
/// frame (N-1)'s fully-propagated value, i.e. exactly "last frame's
/// transform," before frame N's own `Update`-schedule animation (e.g.
/// `spin_objects`) and `PostUpdate`-schedule propagation overwrite it.
#[allow(clippy::type_complexity)]
pub fn update_previous_shape_transforms(
    mut commands: Commands,
    new_shapes: Query<(Entity, &GlobalTransform), (With<Shape>, Without<PreviousShapeTransform>)>,
    mut shapes: Query<(Ref<GlobalTransform>, &mut PreviousShapeTransform), With<Shape>>,
) {
    for (entity, transform) in &new_shapes {
        commands
            .entity(entity)
            .try_insert(PreviousShapeTransform { translation: transform.translation(), rotation: transform.rotation() });
    }
    shapes.par_iter_mut().for_each(|(transform, mut previous)| {
        if transform.is_changed_after(previous.last_changed()) {
            *previous = PreviousShapeTransform { translation: transform.translation(), rotation: transform.rotation() };
        }
    });
}

#[cfg(test)]
mod tests {
    use bevy::prelude::*;

    use super::*;

    fn test_app() -> App {
        let mut app = App::new();
        app.add_plugins(MinimalPlugins);
        app.add_plugins(TransformPlugin);
        app.add_systems(PreUpdate, update_previous_shape_transforms);
        app
    }

    #[test]
    fn a_freshly_spawned_shape_gets_a_previous_transform_equal_to_its_own_current_one() {
        let mut app = test_app();
        let entity = app
            .world_mut()
            .spawn((Shape::Sphere { radius: 1.0 }, Transform::from_xyz(3.0, 1.0, -2.0), GlobalTransform::default()))
            .id();
        app.update();
        let previous = app.world().get::<PreviousShapeTransform>(entity).expect("expected a snapshot after one update");
        assert_eq!(previous.translation, Vec3::new(3.0, 1.0, -2.0));
        assert_eq!(previous.rotation, Quat::IDENTITY);
    }

    #[test]
    fn previous_transform_lags_one_frame_behind_a_moving_shape() {
        let mut app = test_app();
        let entity = app
            .world_mut()
            .spawn((Shape::Sphere { radius: 1.0 }, Transform::from_xyz(0.0, 0.0, 0.0), GlobalTransform::default()))
            .id();
        app.update();
        // First update: no prior GlobalTransform existed, so the initial
        // snapshot must equal frame 1's own position (see the "freshly
        // spawned" test above) — confirmed again here as this test's
        // baseline before moving the entity.
        assert_eq!(
            app.world().get::<PreviousShapeTransform>(entity).unwrap().translation,
            Vec3::new(0.0, 0.0, 0.0)
        );

        // Frame 2: move the entity. PreviousShapeTransform must NOT have
        // updated yet after this same app.update() call, because
        // PreUpdate (where our system runs) executes BEFORE PostUpdate
        // (where the Transform -> GlobalTransform propagation that would
        // make the new position visible actually happens) within this
        // same frame.
        app.world_mut().entity_mut(entity).get_mut::<Transform>().unwrap().translation = Vec3::new(5.0, 0.0, 0.0);
        app.update();
        assert_eq!(
            app.world().get::<PreviousShapeTransform>(entity).unwrap().translation,
            Vec3::new(0.0, 0.0, 0.0),
            "previous transform must still read frame 1's position: PreUpdate runs before PostUpdate's propagation"
        );

        // Frame 3: now that PostUpdate propagated frame 2's move into
        // GlobalTransform, this frame's PreUpdate snapshot should finally
        // pick up the moved position.
        app.update();
        assert_eq!(
            app.world().get::<PreviousShapeTransform>(entity).unwrap().translation,
            Vec3::new(5.0, 0.0, 0.0),
            "previous transform should now reflect the moved position, one frame after the move propagated"
        );
    }

    #[test]
    fn previous_transform_does_not_update_when_the_shape_is_unchanged() {
        let mut app = test_app();
        let entity = app
            .world_mut()
            .spawn((Shape::Sphere { radius: 1.0 }, Transform::from_xyz(1.0, 2.0, 3.0), GlobalTransform::default()))
            .id();
        app.update();
        app.update();
        app.update();
        assert_eq!(
            app.world().get::<PreviousShapeTransform>(entity).unwrap().translation,
            Vec3::new(1.0, 2.0, 3.0),
            "a static shape's previous transform should stay exactly its own unchanging position"
        );
    }
}
