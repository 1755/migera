// GPU physics broad-phase port, Piece 3: the counting-sort's final
// scatter pass. Ports src/physics/broadphase.rs::SpatialHash::build's own
// scatter step exactly: each body atomically claims a distinct write slot
// within its own bucket (`cursor[h] += 1`, using the PRE-increment value
// as the write slot -- exactly what atomicAdd's own return value is) and
// writes its own body index there.
//
// `cursor` must start as a COPY of the scanned `bucket_start` (the
// exclusive-scan output from Piece 2's own passes) -- copied via a
// dedicated GPU-side copy pass (physics_broadphase_copy_main below)
// rather than a CPU round-trip, keeping the whole pipeline on-GPU. Two
// entry points sharing one bind group layout, same one-layout-two-
// pipelines precedent as physics_broadphase_hash.wgsl.
//
// Self-contained (no shared imports), per this codebase's established
// per-pass-file convention.

struct ScatterCopyUniform {
    count: u32,
    _pad0: u32,
    _pad1: u32,
    _pad2: u32,
}

@group(0) @binding(0) var<uniform> params: ScatterCopyUniform;
@group(0) @binding(1) var<storage, read> source: array<u32>;
@group(0) @binding(2) var<storage, read_write> dest: array<atomic<u32>>;

// A trivial element-wise copy -- source (bucket_start, the scanned
// output) into dest (the mutable cursor the scatter pass below
// atomically increments). Declared atomic on the dest side purely
// because the SAME buffer is bound as atomic<u32> in
// physics_broadphase_scatter_main below -- see this file's own header
// comment on why binding the same bytes with different WGSL struct
// declarations across passes is intentional, not a layout bug.
@compute @workgroup_size(64, 1, 1)
fn physics_broadphase_copy_main(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x;
    if i >= params.count {
        return;
    }
    atomicStore(&dest[i], source[i]);
}

struct ScatterUniform {
    body_count: u32,
    _pad0: u32,
    _pad1: u32,
    _pad2: u32,
}

@group(0) @binding(0) var<uniform> scatter_params: ScatterUniform;
@group(0) @binding(1) var<storage, read> hashes: array<u32>;
@group(0) @binding(2) var<storage, read_write> cursor: array<atomic<u32>>;
@group(0) @binding(3) var<storage, read_write> bucket_items: array<u32>;

@compute @workgroup_size(64, 1, 1)
fn physics_broadphase_scatter_main(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x;
    if i >= scatter_params.body_count {
        return;
    }
    let h = hashes[i];
    let slot = atomicAdd(&cursor[h], 1u);
    bucket_items[slot] = i;
}
