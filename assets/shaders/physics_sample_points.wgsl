// GPU physics broad-phase/contact-gen port, Piece 1: sample-point
// generation. One invocation per BODY. Ports
// src/physics/sample_points.rs's sample_points_local exactly (same
// per-shape-kind formulas, same MAX_SAMPLE_POINTS=32 fixed-array
// convention as that module's own Rust-side SamplePoints type) — output
// is LOCAL-space points (not yet transformed to world space; that
// happens in the later contact-generation pass, mirroring
// contacts.rs's own `a.rotation * local_point + a.translation` step).
//
// Self-contained (no shared imports), per this codebase's established
// per-pass-file convention.

const MAX_SAMPLE_POINTS: u32 = 32u;
const RING_SAMPLES: u32 = 6u;
const SPHERE_SAMPLE_COUNT: u32 = 32u;
const ELLIPSOID_SAMPLE_COUNT: u32 = 12u;

const SHAPE_SPHERE: u32 = 0u;
const SHAPE_ROUNDED_BOX: u32 = 1u;
const SHAPE_ROUNDED_CYLINDER: u32 = 2u;
const SHAPE_CAPSULE: u32 = 3u;
const SHAPE_ELLIPSOID: u32 = 4u;
const SHAPE_BOX_FRAME: u32 = 5u;
const SHAPE_HEX_PRISM: u32 = 6u;

struct PhysicsShapeGpu {
    shape_kind: u32,
    _pad0: u32,
    _pad1: u32,
    _pad2: u32,
    param_0: f32,
    param_1: f32,
    param_2: f32,
    param_3: f32,
    param_4: f32,
    param_5: f32,
    param_6: f32,
    param_7: f32,
}

// One body's fixed-size sample-point output -- mirrors
// src/physics/sample_points.rs::SamplePoints exactly (points[..count] are
// real, points[count..] are unused padding).
struct SamplePointsGpu {
    points: array<vec4<f32>, 32>, // xyz = point, w unused
    count: u32,
    _pad0: u32,
    _pad1: u32,
    _pad2: u32,
}

struct SamplePointsUniform {
    shape_count: u32,
    _pad0: u32,
    _pad1: u32,
    _pad2: u32,
}

@group(0) @binding(0) var<uniform> params: SamplePointsUniform;
@group(0) @binding(1) var<storage, read> shapes: array<PhysicsShapeGpu>;
@group(0) @binding(2) var<storage, read_write> outputs: array<SamplePointsGpu>;

fn ellipsoid_fibonacci(radii: vec3<f32>, count: u32, out: ptr<function, SamplePointsGpu>) {
    let golden_angle = 3.14159265 * (3.0 - sqrt(5.0));
    for (var i: u32 = 0u; i < count; i = i + 1u) {
        let y = 1.0 - 2.0 * (f32(i) + 0.5) / f32(count);
        let radius_at_y = max(1.0 - y * y, 0.0);
        let radius_at_y_sqrt = sqrt(radius_at_y);
        let theta = golden_angle * f32(i);
        let x = cos(theta) * radius_at_y_sqrt;
        let z = sin(theta) * radius_at_y_sqrt;
        let point = vec3<f32>(x, y, z) * radii;
        (*out).points[(*out).count] = vec4<f32>(point, 0.0);
        (*out).count = (*out).count + 1u;
    }
}

fn box_corners(half_extents: vec3<f32>, corner_radius_in: f32, out: ptr<function, SamplePointsGpu>) {
    let corner_radius = max(corner_radius_in, 0.0);
    let inset = half_extents - vec3<f32>(corner_radius, corner_radius, corner_radius);
    for (var ix: u32 = 0u; ix < 2u; ix = ix + 1u) {
        let sx = select(-1.0, 1.0, ix == 1u);
        for (var iy: u32 = 0u; iy < 2u; iy = iy + 1u) {
            let sy = select(-1.0, 1.0, iy == 1u);
            for (var iz: u32 = 0u; iz < 2u; iz = iz + 1u) {
                let sz = select(-1.0, 1.0, iz == 1u);
                let corner_center = vec3<f32>(sx * inset.x, sy * inset.y, sz * inset.z);
                var offset = vec3<f32>(0.0, 0.0, 0.0);
                if corner_radius > 0.0 {
                    offset = normalize(vec3<f32>(sx, sy, sz)) * corner_radius;
                }
                (*out).points[(*out).count] = vec4<f32>(corner_center + offset, 0.0);
                (*out).count = (*out).count + 1u;
            }
        }
    }
}

fn ring(center: vec3<f32>, core_radius: f32, core_axis_half: f32, edge_radius: f32, sign: f32, out: ptr<function, SamplePointsGpu>) {
    let corner_2d = vec2<f32>(core_radius, core_axis_half);
    var offset_2d = vec2<f32>(0.0, 0.0);
    if edge_radius > 0.0 {
        offset_2d = normalize(corner_2d) * edge_radius;
    }
    let ring_radius = corner_2d.x + offset_2d.x;
    let ring_axis_half = corner_2d.y + offset_2d.y;
    for (var i: u32 = 0u; i < RING_SAMPLES; i = i + 1u) {
        let angle = (f32(i) / f32(RING_SAMPLES)) * 6.283185307;
        let point = center + vec3<f32>(ring_radius * cos(angle), sign * ring_axis_half, ring_radius * sin(angle));
        (*out).points[(*out).count] = vec4<f32>(point, 0.0);
        (*out).count = (*out).count + 1u;
    }
}

fn cylinder_rings(radius: f32, half_height: f32, edge_radius_in: f32, out: ptr<function, SamplePointsGpu>) {
    let edge_radius = max(edge_radius_in, 0.0);
    let core_radius = radius - edge_radius;
    let core_half_height = half_height - edge_radius;
    ring(vec3<f32>(0.0, 0.0, 0.0), core_radius, core_half_height, edge_radius, 1.0, out);
    ring(vec3<f32>(0.0, 0.0, 0.0), core_radius, core_half_height, edge_radius, -1.0, out);
    (*out).points[(*out).count] = vec4<f32>(0.0, half_height, 0.0, 0.0);
    (*out).count = (*out).count + 1u;
    (*out).points[(*out).count] = vec4<f32>(0.0, -half_height, 0.0, 0.0);
    (*out).count = (*out).count + 1u;
}

// glam::Vec3::any_orthonormal_pair, ported verbatim -- Duff et al.,
// "Building an Orthonormal Basis, Revisited" (Pixar,
// https://graphics.pixar.com/library/OrthonormalB/paper.pdf). Must match
// glam's exact formula bit-for-bit, not just "some" orthonormal basis,
// since this determines the ring sample points' actual world positions.
fn any_orthonormal_pair(axis: vec3<f32>) -> array<vec3<f32>, 2> {
    // glam's own `math::signum` is `copysign(1.0, f)`, matching Rust's
    // real `f32::signum` behavior -- crucially this NEVER returns 0.0
    // (signum(+0.0) == 1.0, signum(-0.0) == -1.0), unlike WGSL's builtin
    // `sign()`, which does return exactly 0.0 at 0.0. Reproduced here via
    // `sign()` with an explicit fallback to +1.0 only for the true-zero
    // case, matching copysign's actual +0.0 behavior (WGSL has no
    // negative-zero-distinguishing builtin, so -0.0 is treated the same
    // as +0.0 here -- an extremely narrow edge case no real capsule axis
    // should ever hit in practice).
    let raw_sign = sign(axis.z);
    let signed = select(raw_sign, 1.0, raw_sign == 0.0);
    let a = -1.0 / (signed + axis.z);
    let b = axis.x * axis.y * a;
    let u = vec3<f32>(1.0 + signed * axis.x * axis.x * a, signed * b, -signed * axis.x);
    let v = vec3<f32>(b, signed + axis.y * axis.y * a, -axis.y);
    return array<vec3<f32>, 2>(u, v);
}

fn capsule_rings(a: vec3<f32>, b: vec3<f32>, radius: f32, out: ptr<function, SamplePointsGpu>) {
    let diff = b - a;
    // glam::Vec3::normalize_or's exact condition: the length RECIPROCAL
    // must be finite and positive -- not a fixed epsilon threshold on
    // length itself. WGSL has no isNan/isInf/isFinite builtins (removed
    // from the spec entirely, ~2021, over undefined fast-math behavior on
    // GPU backends), but the condition reduces to something checkable
    // without them: `rcp > 0.0` already excludes NaN (any comparison
    // against NaN is false in IEEE-754) and negative values, so the only
    // remaining case to exclude is `rcp == +Infinity`, which happens
    // exactly when `length(diff) == 0.0` -- clamping rcp against f32::MAX
    // collapses +Infinity down (breaking equality) while leaving every
    // genuinely finite positive reciprocal unchanged.
    let length_diff = length(diff);
    let rcp = 1.0 / length_diff;
    let rcp_is_finite_positive = rcp > 0.0 && clamp(rcp, 0.0, 3.4e38) == rcp;
    var axis: vec3<f32>;
    if rcp_is_finite_positive {
        axis = diff * rcp;
    } else {
        axis = vec3<f32>(0.0, 1.0, 0.0);
    }
    let uv = any_orthonormal_pair(axis);
    let u = uv[0];
    let v = uv[1];

    for (var side: u32 = 0u; side < 2u; side = side + 1u) {
        var center = a;
        var sign = -1.0;
        if side == 1u {
            center = b;
            sign = 1.0;
        }
        for (var i: u32 = 0u; i < RING_SAMPLES; i = i + 1u) {
            let angle = (f32(i) / f32(RING_SAMPLES)) * 6.283185307;
            let radial = u * cos(angle) + v * sin(angle);
            (*out).points[(*out).count] = vec4<f32>(center + radial * radius, 0.0);
            (*out).count = (*out).count + 1u;
        }
        (*out).points[(*out).count] = vec4<f32>(center + axis * (radius * sign), 0.0);
        (*out).count = (*out).count + 1u;
    }
}

@compute @workgroup_size(64, 1, 1)
fn physics_sample_points_main(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x;
    if i >= params.shape_count {
        return;
    }

    let shape = shapes[i];
    var result: SamplePointsGpu;
    result.count = 0u;

    if shape.shape_kind == SHAPE_SPHERE {
        ellipsoid_fibonacci(vec3<f32>(shape.param_0, shape.param_0, shape.param_0), SPHERE_SAMPLE_COUNT, &result);
    } else if shape.shape_kind == SHAPE_ROUNDED_BOX {
        box_corners(vec3<f32>(shape.param_0, shape.param_1, shape.param_2), shape.param_3, &result);
    } else if shape.shape_kind == SHAPE_ROUNDED_CYLINDER {
        cylinder_rings(shape.param_0, shape.param_1, shape.param_2, &result);
    } else if shape.shape_kind == SHAPE_CAPSULE {
        capsule_rings(vec3<f32>(shape.param_0, shape.param_1, shape.param_2), vec3<f32>(shape.param_3, shape.param_4, shape.param_5), shape.param_6, &result);
    } else if shape.shape_kind == SHAPE_ELLIPSOID {
        ellipsoid_fibonacci(vec3<f32>(shape.param_0, shape.param_1, shape.param_2), ELLIPSOID_SAMPLE_COUNT, &result);
    } else if shape.shape_kind == SHAPE_BOX_FRAME {
        box_corners(vec3<f32>(shape.param_0, shape.param_1, shape.param_2), 0.0, &result);
    } else if shape.shape_kind == SHAPE_HEX_PRISM {
        cylinder_rings(shape.param_0, shape.param_1, 0.0, &result);
    }

    outputs[i] = result;
}
