//! Placeholder binary: the real demos live under `examples/` (see `compute_spine`
//! and `gallery`, both built on the hybrid renderer). This binary is kept minimal
//! on purpose — a bare window with a clear color — since the old direct-raymarch
//! technical demo it used to run has been retired in favor of the hybrid pipeline.

use bevy::prelude::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .insert_resource(ClearColor(Color::srgb(0.05, 0.06, 0.08)))
        .run();
}
