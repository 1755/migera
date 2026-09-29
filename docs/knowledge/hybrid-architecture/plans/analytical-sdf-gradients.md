---
title: Analytical SDF Gradients Research & Implementation Plan
description: Plan to replace finite-difference SDF normals with analytic per-primitive and CSG gradients (IQ's sdg* functions). Built in src/hybrid and measured at --stress 10000 with no difference from 6-tap central differences, then removed. Read before re-proposing analytic normals.
type: design
status: archived
tags:
  - sdf
  - raymarching
  - performance
  - math
  - hybrid-renderer
updated: 2026-08-23
verified: 2026-09-28
code:
  - src/hybrid/cpu_ref.rs
  - assets/shaders/raymarch.wgsl
sources:
  - https://iquilezles.org/articles/distgradfunctions3d/
  - https://iquilezles.org/articles/smin/
  - PROGRESS.md "Analytic vs. finite-difference normals" entry
aliases:
  - analytic normals
  - sdg functions
  - gradient-returning SDF
---

# Analytical SDF Gradients Research & Implementation Plan

> **Archived:** built and measured, then removed. PROGRESS.md's entry
> "Analytic vs. finite-difference normals" implemented central-difference,
> tetrahedron and analytic normals side by side in `src/hybrid`
> (`cpu_ref.rs` + `hybrid_trace.wgsl`). At `--stress 10000` all three were
> within ~1–2 ms with heavily overlapping ranges, so the analytic and
> tetrahedron paths were deleted and 6-tap central differences stayed. Normals
> are paid once per shaded pixel; BVH traversal and marching dominate. This
> plan's own §4 arithmetic already predicted only a 1.6% saving, contradicting
> its "2–4x" headline. The Torus primitive it lists was later removed. The
> gradient formulas below remain correct reference (`raymarch.wgsl`'s `sdg_*`
> functions still exist).

Contents: [Summary](#executive-summary) · [Background](#background-why-analytical-gradients) ·
[Implementation](#technical-implementation) · [Roadmap](#implementation-roadmap) ·
[Testing](#testing-strategy) · [Risks](#risk-assessment) · [Metrics](#success-metrics) ·
[References](#references) · [Related](#related)

## Executive Summary

**Goal**: Reduce raymarching performance from 4-6 SDF evaluations per point to 1-2 by implementing analytical gradient computation instead of finite difference methods.

**Impact**: 
- Current: 4-6 `map()` calls per point for normal estimation (4-tap finite differences)
- Target: 1-2 `map()` calls per point using analytical gradients
- **Performance improvement: 2-4x faster per pixel for normal computation**

## Background: Why Analytical Gradients?

### Current Finite Difference Method (4-tap)
```wgsl
fn calc_normal(p: vec3<f32>) -> vec3<f32> {
    let k = vec2<f32>(1.0, -1.0);
    let h = 0.001;
    return normalize(
        k.xyy * map(p + k.xyy * h) +  // 4 evaluations
        k.yyx * map(p + k.yyx * h) +
        k.yxy * map(p + k.yxy * h) +
        k.xxx * map(p + k.xxx * h)
    );
}
```

**Problems**:
1. 4 extra `map()` calls per normal computation
2. Each `map()` call evaluates ALL primitives in the scene (200+ primitives)
3. Cost: 4 * 200 = 800 SDF evaluations per pixel just for normals
4. Numerical precision issues with small epsilon values
5. Aliasing artifacts when epsilon doesn't match pixel footprint

### Analytical Gradient Method (1-2 evaluations)
```wgsl
// Each primitive returns vec4(distance, gradient_x, gradient_y, gradient_z)
fn sdgSphere(p: vec3, r: f32) -> vec4<f32> {
    let l = length(p);
    return vec4(l - r, p / l);  // Distance + gradient in one call
}
```

**Benefits**:
1. Single evaluation per primitive (distance + gradient together)
2. Gradient is exact, not approximated
3. No epsilon tuning required
4. Gradient magnitude is always 1.0 (unit length)
5. Gradient points toward closest surface point

## Technical Implementation

### 1. Primitive Gradient Functions

Each SDF primitive needs an analytical gradient function. Key primitives for migera:

#### Sphere
```wgsl
fn sdgSphere(p: vec3, r: f32) -> vec4<f32> {
    let l = length(p);
    return vec4(l - r, p / l);
}
// Gradient: ∇f(p) = (p - center) / ||p - center||
```

#### Rounded Box (Box with rounding)
```wgsl
fn sdgRoundedBox(p: vec3, b: vec3, r: f32) -> vec4<f32> {
    let w = abs(p) - (b - r);
    let g = max(w.x, max(w.y, w.z));
    let q = max(w, vec3(0.0));
    let l = length(q);
    
    if (g > 0.0) {
        return vec4(l - r, q / l * sign(p));
    } else {
        // Inside box - gradient points to nearest face
        let normal = vec3(
            select(0.0, 1.0, w.x == g),
            select(0.0, 1.0, w.y == g),
            select(0.0, 1.0, w.z == g)
        );
        return vec4(g - r, normal * sign(p));
    }
}
```

#### Torus
```wgsl
fn sdgTorus(p: vec3, ra: f32, rb: f32) -> vec4<f32> {
    let h = length(p.xz);
    let d = length(vec2(h - ra, p.y)) - rb;
    let gradient = normalize(p * vec3(h - ra, h, h - ra));
    return vec4(d, gradient);
}
```

#### Capsule (Segment)
```wgsl
fn sdgCapsule(p: vec3, a: vec3, b: vec3, r: f32) -> vec4<f32> {
    let ba = b - a;
    let pa = p - a;
    let h = clamp(dot(pa, ba) / dot(ba, ba), 0.0, 1.0);
    let q = pa - h * ba;
    let d = length(q) - r;
    let gradient = q / length(q);
    return vec4(d, gradient);
}
```

#### Ellipsoid
```wgsl
fn sdgEllipsoid(p: vec3, r: vec3) -> vec4<f32> {
    let k0 = length(p / r);
    let k1 = length(p / (r * r));
    return vec4(k0 * (k0 - 1.0) / k1, p / (r * r * k1));
}
```

#### Rounded Cone
```wgsl
fn sdgRoundedCone(p: vec3, a: vec3, b: vec3, r1: f32, r2: f32) -> vec4<f32> {
    // Complex analytical gradient - see IQ's implementation
    let ba = b - a;
    let l2 = dot(ba, ba);
    let rr = r1 - r2;
    let a2 = l2 - rr * rr;
    let il2 = 1.0 / l2;
    
    let pa = p - a;
    let pb = p - b;
    let y = dot(pa, ba);
    let z = y - l2;
    let x2 = l2 * dot(pa, pa) - y * y;
    
    // Three cases: near cap A, near cap B, or middle section
    if (sign(z) * a2 * z * z > sign(rr) * rr * rr * x2) {
        let w = sqrt(il2 * (x2 + z * z));
        return vec4(w - r2, pb / w);
    }
    if (sign(y) * a2 * y * y < sign(rr) * rr * rr * x2) {
        let w = sqrt(il2 * (x2 + y * y));
        return vec4(w - r1, pa / w);
    }
    let w = sqrt(x2 * a2);
    return vec4(
        (w + y * rr) * il2 - r1,
        il2 * (rr * ba + a2 * (pa * l2 - y * ba) / w)
    );
}
```

#### Box Frame
```wgsl
fn sdgBoxFrame(p: vec3, b: vec3, e: f32) -> vec4<f32> {
    let p = abs(p) - b;
    let q = abs(p + e) - e;
    // Gradient computation for box frame
    // ... (implementation details)
}
```

### 2. CSG Operator Gradients

#### Union (min)
```wgsl
fn opUnionGrad(a: vec4<f32>, b: vec4<f32>) -> vec4<f32> {
    // Gradient propagates from the primitive with smaller distance
    return (a.x < b.x) ? a : b;
}
```

#### Subtraction (max(-a, b))
```wgsl
fn opSubtractionGrad(a: vec4<f32>, b: vec4<f32>) -> vec4<f32> {
    // Negate a's gradient when a is selected
    if (-a.x > b.x) {
        return vec4(-a.x, -a.yzw);  // Flip gradient direction
    }
    return b;
}
```

#### Intersection (max)
```wgsl
fn opIntersectionGrad(a: vec4<f32>, b: vec4<f32>) -> vec4<f32> {
    // Gradient propagates from the primitive with larger distance
    return (a.x > b.x) ? a : b;
}
```

#### Smooth Union (smin)
```wgsl
fn opSmoothUnionGrad(a: vec4<f32>, b: vec4<f32>, k: f32) -> vec4<f32> {
    k *= 4.0;
    let h = max(k - abs(a.x - b.x), 0.0);
    let m = 0.25 * h * h / k;
    let n = 0.50 * h / k;
    
    let dist = min(a.x, b.x) - m;
    let gradient = mix(a.yzw, b.yzw, select(n, 1.0 - n, a.x < b.x));
    
    return vec4(dist, gradient);
}
```

### 3. Shader Architecture Changes

#### New Data Structure
```wgsl
struct SdfResult {
    distance: f32,
    gradient: vec3<f32>,
    material_id: u32,
    // ... other material data
};
```

#### Modified eval_leaf Function
```wgsl
fn eval_leaf_grad(record: PrimitiveRecord, p: vec3) -> SdfResult {
    switch (record.tag) {
        case TAG_LEAF_SPHERE: {
            let result = sdgSphere(p, record.param_a);
            return SdfResult(result.x, result.yzw, record.material_id);
        }
        // ... other primitives
        default: {
            return SdfResult(1e10, vec3(0.0), 0u);
        }
    }
}
```

#### Modified map Function
```wgsl
fn map_grad(p: vec3) -> SdfResult {
    var result = SdfResult(1e10, vec3(0.0), 0u);
    
    // Evaluate all CSG records
    for (var i = 0u; i < csg_record_count; i++) {
        let record = csg_records[i];
        let leaf_result = eval_leaf_grad(record, p);
        
        // Apply CSG operation with gradient propagation
        result = opUnionGrad(result, leaf_result);
        // or opSubtractionGrad, opIntersectionGrad, etc.
    }
    
    return result;
}
```

#### Optimized Normal Calculation
```wgsl
fn calc_normal_grad(p: vec3) -> vec3<f32> {
    // Single evaluation returns both distance and gradient
    let result = map_grad(p);
    return normalize(result.gradient);
}
```

### 4. Performance Analysis

#### Current Performance (Finite Differences)
For a scene with 200 primitives:
- Primary raymarching: 128 steps × 200 evals = 25,600 SDF evaluations
- Normal at hit: 4 × 200 = 800 SDF evaluations
- Shadow ray: 32 × 200 = 6,400 SDF evaluations
- Reflection: 24 × 200 = 4,800 SDF evaluations
- **Total: ~37,600 SDF evaluations per pixel**

#### Optimized Performance (Analytical Gradients)
- Primary raymarching: 128 steps × 200 evals = 25,600 SDF evaluations
- Normal at hit: 1 × 200 = 200 SDF evaluations (gradient computed with distance)
- Shadow ray: 32 × 200 = 6,400 SDF evaluations
- Reflection: 24 × 200 = 4,800 SDF evaluations
- **Total: ~37,000 SDF evaluations per pixel**

**Savings**: 600 evaluations per pixel (1.6% reduction)

#### Further Optimization: Early Termination
With analytical gradients, we can implement early termination in CSG evaluation:
```wgsl
fn map_grad_early(p: vec3) -> SdfResult {
    var result = SdfResult(1e10, vec3(0.0), 0u);
    
    for (var i = 0u; i < csg_record_count; i++) {
        let record = csg_records[i];
        let leaf_result = eval_leaf_grad(record, p);
        
        // Early termination if distance is very large
        if (leaf_result.distance > result.distance + MAX_STEP_SIZE) {
            continue;  // Skip this primitive
        }
        
        result = opUnionGrad(result, leaf_result);
    }
    
    return result;
}
```

**Additional Savings**: 10-30% reduction in primitive evaluations

## Implementation Roadmap

### Phase 1: Primitive Gradient Functions (Week 1)
1. Implement analytical gradient functions for all 9 primitives:
   - Sphere, RoundedBox, Torus, RoundedCylinder, Capsule
   - RoundedCone, Ellipsoid, BoxFrame, HexPrism
2. Create unit tests for gradient accuracy
3. Verify gradient magnitude is always 1.0

### Phase 2: CSG Operator Gradients (Week 2)
1. Implement gradient propagation for CSG operators:
   - Union (min), Subtraction (max(-a,b)), Intersection (max(a,b))
   - Smooth Union (smin), Smooth Subtraction, Smooth Intersection
2. Test with complex CSG trees
3. Verify gradient continuity across operator boundaries

### Phase 3: Shader Integration (Week 3)
1. Modify `PrimitiveRecord` to include gradient data
2. Update `eval_leaf` to return gradient alongside distance
3. Implement `map_grad` function with early termination
4. Replace `calc_normal` with `calc_normal_grad`
5. Update shadow and reflection rays to use analytical gradients

### Phase 4: Performance Optimization (Week 4)
1. Profile and identify remaining bottlenecks
2. Implement spatial acceleration structures (optional)
3. Add level-of-detail (LOD) for distant primitives
4. Tune step sizes based on gradient information

## Testing Strategy

### Unit Tests
1. **Gradient Accuracy**: Compare analytical gradients with finite differences
   ```rust
   #[test]
   fn test_sphere_gradient_accuracy() {
       let p = vec3(1.0, 2.0, 3.0);
       let r = 1.0;
       let analytical = sdg_sphere(p, r);
       let numerical = finite_difference_sphere(p, r);
       assert!((analytical.gradient - numerical).length() < 1e-6);
   }
   ```

2. **Gradient Magnitude**: Verify all gradients have unit length
   ```rust
   #[test]
   fn test_gradient_magnitude() {
       let p = vec3(random(), random(), random());
       let result = sdg_sphere(p, 1.0);
       assert!((result.gradient.length() - 1.0).abs() < 1e-6);
   }
   ```

3. **CSG Gradient Continuity**: Test gradient at operator boundaries
   ```rust
   #[test]
   fn test_union_gradient_continuity() {
       // Test gradient at point where two primitives meet
   }
   ```

### Integration Tests
1. **Visual Comparison**: Render scenes with finite differences vs analytical gradients
2. **Performance Benchmark**: Measure frame time improvement
3. **Memory Usage**: Verify no significant increase in shader complexity

## Risk Assessment

### Technical Risks
1. **Complexity**: RoundedCone and BoxFrame gradients are complex
   - Mitigation: Start with simpler primitives, add complexity gradually
   
2. **Precision**: Gradient magnitude may not be exactly 1.0 due to floating point
   - Mitigation: Normalize gradient before use
   
3. **CSG Artifacts**: Gradient discontinuities at operator boundaries
   - Mitigation: Use smooth operators (smin) for blending

### Performance Risks
1. **Instruction Count**: Analytical gradients may increase ALU operations
   - Mitigation: Profile and optimize hot paths
   
2. **Register Pressure**: More temporary variables in gradient computation
   - Mitigation: Reuse intermediate values, limit scope

## Success Metrics

### Primary Metrics
1. **Performance**: 2-4x faster normal computation
2. **Quality**: No visual artifacts or gradient discontinuities
3. **Memory**: <10% increase in shader instruction count

### Secondary Metrics
1. **Accuracy**: Gradient error < 1e-6 compared to finite differences
2. **Robustness**: Works for all primitive combinations
3. **Maintainability**: Easy to add new primitives

## References

1. **Inigo Quilez - Analytical SDF Gradients**
   - https://iquilezles.org/articles/distgradfunctions3d/
   - https://iquilezles.org/articles/distgradfunctions2d/

2. **SDF Normal Estimation**
   - https://iquilezles.org/articles/normalsSDF/

3. **Directional Derivatives for Lighting**
   - https://iquilezles.org/articles/derivative/

4. **Smooth Minimum Gradients**
   - https://iquilezles.org/articles/smin/

5. **GPU Performance Optimization**
   - "Real-Time Rendering" book, Chapter on Ray Marching
   - Shadertoy examples and community optimizations

## Conclusion

Implementing analytical SDF gradients is a high-impact optimization that will significantly improve raymarching performance in migera. The implementation is well-documented with closed-form solutions for all primitives and CSG operators. With careful implementation and testing, this optimization should provide 2-4x faster normal computation with no loss in quality.

**Next Steps**:
1. Review this plan with team
2. Prioritize primitives based on usage in migera scenes
3. Begin Phase 1 implementation
4. Set up performance benchmarks
5. Iterate based on profiling results

## Related
- [Normal estimation](../../sdf-3d/rendering/normal-estimation.md) — deeper: the finite-difference patterns (6-tap, tetrahedron) and when analytic gradients help.
- [Trace-pass bottleneck is not march steps](../performance-findings/trace-pass-bottleneck-is-not-march-steps.md) — contrast: the same "theoretically expensive part isn't the bottleneck" result for march iteration count.
- [RoundedCone SDF reports everything exterior](../roundedcone-sdf-reports-everything-exterior.md) — deeper: why `RoundedCone` was excluded from the analytic comparison.
- [Combination operators](../../sdf-3d/primitives-and-operators/combination-operators.md) — prerequisite: the smooth-min operators whose gradients §2 blends.
