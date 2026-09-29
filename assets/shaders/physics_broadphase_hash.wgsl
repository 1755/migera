// GPU physics broad-phase port, Piece 3: hash + count passes. Ports
// src/physics/broadphase.rs's cell_coord/cell_hash exactly (Müller's own
// spatial-hash formula, "Blazing Fast Neighbor Search with Spatial
// Hashing," Ten Minute Physics) and the counting-sort's own count step
// (`bucket_start[h+1] += 1`, done here via atomicAdd since many bodies
// can hash to the same bucket concurrently).
//
// Two entry points sharing one bind group layout (mirrors
// physics_scatter_position/velocity's own one-layout-two-pipelines
// precedent from the solver port):
// - physics_broadphase_hash_main: one invocation per body, writes each
//   body's own hash into `hashes` (read by the scatter pass later to
//   avoid recomputing it).
// - physics_broadphase_count_main: one invocation per body, atomicAdd's
//   into `bucket_counts[hash]` -- RAW (unshifted) per-bucket counts, NOT
//   pre-shifted-by-one. This is deliberately NOT the same indexing
//   `SpatialHash::build`'s own CPU count step uses (`bucket_start[h+1] +=
//   1`) -- that shift is only valid there because the CPU's very next
//   line (`bucket_start[i+1] += bucket_start[i]`) is a single fused pass
//   that turns pre-shifted counts directly into the exclusive-scan
//   result. This GPU port instead reuses Piece 2's already-proven GENERIC
//   scan (`dispatch_physics_scan`: inclusive Hillis-Steele passes then a
//   separate exclusive-shift pass), which expects raw, unshifted counts
//   as input -- feeding it pre-shifted counts double-shifts the result by
//   one bucket (caught by this piece's own parity tests before landing).
//   `bucket_counts[table_size]` (the last slot of the table_size+1-length
//   buffer) is always left at zero here, since no body ever hashes to
//   `table_size` itself (hashes are always `< table_size`) -- matching
//   the CPU reference's own implicit invariant that `bucket_start[0] = 0`
//   and every real count lands in `1..=table_size`.
//   This pass's OUTPUT buffer is the exact same underlying buffer Piece
//   2's scan pass later reads as a plain (non-atomic) array<u32> input;
//   WGSL atomicity is a per-bind-group declaration, not a property of the
//   buffer itself, so binding the same bytes with a plain vs. atomic
//   struct across different passes is intentional and safe here -- the
//   exact same convention physics_scatter_position.wgsl's own doc comment
//   already establishes for PhysicsAccumulatorGpu.
//
// Self-contained (no shared imports), per this codebase's established
// per-pass-file convention.

struct HashCountUniform {
    body_count: u32,
    table_size: u32,
    cell_size: f32,
    _pad0: u32,
}

@group(0) @binding(0) var<uniform> params: HashCountUniform;
@group(0) @binding(1) var<storage, read> positions: array<vec4<f32>>; // xyz = position, w unused
@group(0) @binding(2) var<storage, read_write> hashes: array<u32>;
@group(0) @binding(3) var<storage, read_write> bucket_counts: array<atomic<u32>>;

fn cell_coord(point: vec3<f32>, cell_size: f32) -> vec3<i32> {
    return vec3<i32>(floor(point / cell_size));
}

// Müller's spatial-hash function, ported verbatim -- large odd
// multipliers chosen to decorrelate axis-aligned clustering. Matches
// cell_hash's exact wrapping-multiply/xor/modulo sequence; WGSL's i32
// multiplication already wraps on overflow (no explicit wrapping needed,
// unlike Rust's checked-by-default arithmetic).
fn cell_hash(cell: vec3<i32>, table_size: u32) -> u32 {
    let h = (cell.x * 92837111i) ^ (cell.y * 689287499i) ^ (cell.z * 283923481i);
    return u32(h) % table_size;
}

@compute @workgroup_size(64, 1, 1)
fn physics_broadphase_hash_main(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x;
    if i >= params.body_count {
        return;
    }
    let cell = cell_coord(positions[i].xyz, params.cell_size);
    hashes[i] = cell_hash(cell, params.table_size);
}

@compute @workgroup_size(64, 1, 1)
fn physics_broadphase_count_main(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x;
    if i >= params.body_count {
        return;
    }
    let h = hashes[i];
    atomicAdd(&bucket_counts[h], 1u);
}
