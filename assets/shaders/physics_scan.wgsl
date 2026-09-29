// GPU physics broad-phase port, Piece 2: parallel exclusive prefix sum
// (Hillis-Steele), proven standalone before it's load-bearing for the
// broad-phase CSR bucket-offset computation (Piece 3). See the plan's own
// "Stage 3 remainder" section for why Hillis-Steele was chosen over a
// work-efficient Blelloch scan: simpler (one algorithm shape, no separate
// block-sums-then-fixup pass), at the cost of O(n log n) total work
// instead of Blelloch's O(n) -- immaterial at the element counts this
// port needs (up to ~40,000 buckets at 20,000 bodies).
//
// Classic Hillis-Steele INCLUSIVE scan: at pass `d` (offset = 2^d), each
// element i becomes `arr[i] + arr[i - offset]` if `i >= offset`, else
// unchanged. log2(n) passes total. Ping-ponged between two buffers (NOT
// in-place) since every invocation reads a neighbor another invocation in
// the SAME dispatch may also be writing -- an in-place update would race.
//
// EXCLUSIVE conversion happens OUTSIDE this shader's own scan passes, in
// the final pass below (physics_scan_to_exclusive_main): exclusive[i] =
// inclusive[i-1], exclusive[0] = 0 -- the same shift-by-one relationship
// src/physics/broadphase.rs's own CPU reference already uses
// (`bucket_start[i+1] += bucket_start[i]`), just computed as a dedicated
// shift pass here rather than woven into the scan loop itself (keeping
// the scan passes' own logic identical regardless of which convention the
// caller ultimately wants).
//
// Self-contained (no shared imports), per this codebase's established
// per-pass-file convention.

struct ScanUniform {
    count: u32,
    offset: u32,
    _pad0: u32,
    _pad1: u32,
}

@group(0) @binding(0) var<uniform> params: ScanUniform;
@group(0) @binding(1) var<storage, read> input: array<u32>;
@group(0) @binding(2) var<storage, read_write> output: array<u32>;

@compute @workgroup_size(64, 1, 1)
fn physics_scan_step_main(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x;
    if i >= params.count {
        return;
    }
    if i >= params.offset {
        output[i] = input[i] + input[i - params.offset];
    } else {
        output[i] = input[i];
    }
}

@compute @workgroup_size(64, 1, 1)
fn physics_scan_to_exclusive_main(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x;
    if i >= params.count {
        return;
    }
    if i == 0u {
        output[i] = 0u;
    } else {
        output[i] = input[i - 1u];
    }
}
