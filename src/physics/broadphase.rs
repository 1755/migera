//! Uniform spatial-hash broad-phase for dynamic-vs-dynamic contact
//! generation — replaces `solve_world`'s O(n²) all-pairs loop, which the
//! stage-3 BVH-under-motion profiling milestone confirmed as the real
//! bottleneck at scale (2000 dynamic bodies too slow for real time; BVH
//! refit cost itself was never the problem). Standard technique for
//! GPU-friendly rigid-body/particle broad-phase (NVIDIA FleX, PhysX GPU
//! rigid bodies, Macklin & Müller's "Unified Particle Physics for
//! Real-Time Applications," SIGGRAPH 2014; hash function and cell-sizing
//! convention from Matthias Müller's "Blazing Fast Neighbor Search with
//! Spatial Hashing," Ten Minute Physics) — chosen over sort-and-sweep
//! (its sweep step is inherently sequential-in-spirit, a worse GPU fit)
//! and a from-scratch GPU LBVH (Karras 2012 — real, proven, but far more
//! implementation machinery than this project's body counts justify) for
//! the same reason Jacobi-with-atomics was chosen over graph-coloring
//! Gauss-Seidel: the lowest-complexity approach that is still genuinely
//! parallel throughout, matching this project's own "the only approach a
//! small team can implement correctly on a first pass" discipline.
//!
//! **Static colliders are deliberately excluded from this hash entirely**
//! — a spatial hash sized for a 0.3-unit dynamic sphere and a 6-unit
//! static floor in the SAME grid would need the floor to span hundreds of
//! small cells, defeating the whole point. `solve_world`'s existing
//! dynamic-vs-static path (already O(dynamic × static), not the
//! bottleneck the profiling milestone found) is untouched; only the
//! dynamic-vs-dynamic O(n²) loop is replaced here.
//!
//! **Structure mirrors the eventual GPU algorithm's four-plus-one-pass
//! shape exactly** (per `src/hybrid`'s CPU-ref-first doctrine — a
//! reference must model the actual algorithm, not just independently
//! reach a correct answer, the same standard already applied to
//! `solve_rigid`'s Jacobi accumulator): (1) hash each body's cell, (2)
//! count bodies per bucket, (3) prefix-sum into bucket offsets, (4)
//! scatter body indices into their bucket's slot — a counting sort, not
//! an atomic-append-per-insert, so total capacity is known up front with
//! no unbounded-per-cell problem. The fifth "pass" (`query_candidates`)
//! deliberately does NOT materialize a separate pair list: each body
//! walks its own 27-cell neighborhood and hands candidates straight to
//! `contacts::generate_contacts`, sidestepping GPU dynamic-array/atomic-
//! append complexity entirely — confirmed a legitimate first-version
//! simplification (not a shortcut requiring later rework), since
//! sample-point contact generation is already cheap enough per candidate
//! that deferring it to a second pass buys nothing.

use bevy::math::IVec3;
use bevy::prelude::*;

/// Grid-cell coordinate for a world-space point, given `cell_size` (should
/// be sized to roughly 2x the largest DYNAMIC body's bounding radius —
/// see this module's own doc comment for why statics are excluded rather
/// than folded into this sizing decision).
fn cell_coord(point: Vec3, cell_size: f32) -> IVec3 {
    IVec3::new(
        (point.x / cell_size).floor() as i32,
        (point.y / cell_size).floor() as i32,
        (point.z / cell_size).floor() as i32,
    )
}

/// Müller's spatial-hash function: large odd multipliers chosen
/// specifically to decorrelate axis-aligned clustering (three
/// lattice-adjacent cells must not hash to adjacent buckets, or the
/// bucket distribution degenerates back toward the same locality problem
/// hashing exists to avoid). `table_size` should be a prime or at least
/// odd for the same declustering reason; this module doesn't enforce
/// that, matching the reference algorithm's own convention of leaving
/// table-size choice to the caller.
fn cell_hash(cell: IVec3, table_size: u32) -> u32 {
    let h = (cell.x.wrapping_mul(92_837_111)) ^ (cell.y.wrapping_mul(689_287_499)) ^ (cell.z.wrapping_mul(283_923_481));
    (h as u32) % table_size
}

/// The 27 neighboring cells (including the cell itself) in 3D — the
/// search radius a `cell_size` of ~2x the largest relevant radius
/// guarantees is sufficient to find every truly-overlapping pair.
fn neighbor_cells(cell: IVec3) -> impl Iterator<Item = IVec3> {
    (-1..=1).flat_map(move |dx| {
        (-1..=1).flat_map(move |dy| (-1..=1).map(move |dz| cell + IVec3::new(dx, dy, dz)))
    })
}

/// A built spatial hash: `bucket_start[h]..bucket_start[h + 1]` indexes
/// into `bucket_items` for every body hashing to bucket `h` — the
/// standard counting-sort CSR (compressed sparse row) layout, exactly
/// what a GPU port's own three-array buffer set (hash-per-body work array
/// is transient/discarded here since the CPU reference has no need to
/// keep it after scattering, matching the GPU pipeline's own discard of
/// intermediate per-body hashes once bucketed) would use.
pub struct SpatialHash {
    cell_size: f32,
    table_size: u32,
    bucket_start: Vec<u32>,
    bucket_items: Vec<u32>,
}

impl SpatialHash {
    /// Builds the hash from `positions` (typically each dynamic body's
    /// current world-space position) via the counting-sort structure this
    /// module's doc comment describes: count -> prefix-sum -> scatter.
    pub fn build(positions: &[Vec3], cell_size: f32) -> Self {
        // A floor on table size matters at small body counts: with too
        // few buckets, the hash's own modulo aliases genuinely distant
        // cells onto the same bucket (confirmed by this module's own
        // early test failures at `table_size = 4` for a 2-body scene) —
        // 256 is cheap at any realistic body count and large enough that
        // small scenes don't see pathological collision rates.
        let table_size = ((positions.len() as u32).max(1) * 2).max(256);
        let hashes: Vec<u32> = positions.iter().map(|&p| cell_hash(cell_coord(p, cell_size), table_size)).collect();

        let mut bucket_start = vec![0u32; table_size as usize + 1];
        for &h in &hashes {
            bucket_start[h as usize + 1] += 1;
        }
        for i in 0..table_size as usize {
            bucket_start[i + 1] += bucket_start[i];
        }

        let mut cursor = bucket_start.clone();
        let mut bucket_items = vec![0u32; positions.len()];
        for (i, &h) in hashes.iter().enumerate() {
            let slot = cursor[h as usize];
            bucket_items[slot as usize] = i as u32;
            cursor[h as usize] += 1;
        }

        Self { cell_size, table_size, bucket_start, bucket_items }
    }

    /// Raw CSR arrays — `pub(crate)` purely so `physics::gpu`'s own
    /// broad-phase parity test can diff the GPU-built hash's
    /// `bucket_start`/`bucket_items` against this CPU reference's real
    /// output directly, rather than only through `query_candidates`'s own
    /// (correct, but comparatively opaque) aggregated view. Not `pub`:
    /// nothing outside this crate needs the raw layout, only
    /// `query_candidates` is the intended external API.
    ///
    /// Only called from `#[cfg(test)]` code (`gpu/parity_test.rs`), so a
    /// plain (non-test) `cargo clippy --lib`/`cargo build --lib` sees
    /// these as genuinely unused — `allow(dead_code)` outside test builds
    /// silences that false positive without masking real dead code in an
    /// actual test build (where `#[cfg(test)]` is active and clippy/rustc
    /// see the real call sites).
    #[cfg_attr(not(test), allow(dead_code))]
    pub(crate) fn bucket_start(&self) -> &[u32] {
        &self.bucket_start
    }

    #[cfg_attr(not(test), allow(dead_code))]
    pub(crate) fn bucket_items(&self) -> &[u32] {
        &self.bucket_items
    }

    #[cfg_attr(not(test), allow(dead_code))]
    pub(crate) fn table_size(&self) -> u32 {
        self.table_size
    }

    /// Every OTHER body index sharing a neighboring cell with
    /// `positions[body_index]` — candidates only, not confirmed contacts;
    /// the caller (`solve_world`) still runs full sample-point contact
    /// generation against each one, exactly as the O(n²) loop it replaces
    /// did, just against a far smaller candidate set. May return
    /// duplicates if `table_size` is small enough that two neighboring
    /// cells alias to the same bucket — callers that need strict
    /// uniqueness should dedupe; `solve_world`'s own contact accumulation
    /// tolerates duplicate candidate pairs the same way multiple sample
    /// points already produce multiple contacts per pair, so no dedup is
    /// applied here.
    pub fn query_candidates(&self, positions: &[Vec3], body_index: usize) -> Vec<u32> {
        let cell = cell_coord(positions[body_index], self.cell_size);
        let mut candidates = Vec::new();
        for neighbor in neighbor_cells(cell) {
            let h = cell_hash(neighbor, self.table_size) as usize;
            let range = self.bucket_start[h] as usize..self.bucket_start[h + 1] as usize;
            for &candidate in &self.bucket_items[range] {
                if candidate as usize != body_index {
                    candidates.push(candidate);
                }
            }
        }
        candidates
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn two_bodies_in_the_same_cell_are_each_others_candidates() {
        let positions = vec![Vec3::new(0.1, 0.1, 0.1), Vec3::new(0.2, 0.2, 0.2)];
        let hash = SpatialHash::build(&positions, 1.0);
        // Duplicates are expected and tolerated (see `query_candidates`'s
        // own doc comment: neighboring cells can alias to the same
        // bucket) -- what must hold is that the OTHER body appears at
        // least once and no unrelated index appears at all.
        assert!(hash.query_candidates(&positions, 0).iter().all(|&c| c == 1));
        assert!(hash.query_candidates(&positions, 0).contains(&1));
        assert!(hash.query_candidates(&positions, 1).iter().all(|&c| c == 0));
        assert!(hash.query_candidates(&positions, 1).contains(&0));
    }

    #[test]
    fn two_bodies_in_adjacent_cells_are_still_found_via_the_27_cell_neighborhood() {
        // Same cell size as above, but positioned just across a cell
        // boundary from each other -- the whole point of searching all 27
        // neighbors, not just the body's own cell.
        let positions = vec![Vec3::new(0.05, 0.0, 0.0), Vec3::new(0.95, 0.0, 0.0)];
        let hash = SpatialHash::build(&positions, 1.0);
        assert!(hash.query_candidates(&positions, 0).contains(&1));
    }

    #[test]
    fn two_bodies_far_apart_are_not_candidates() {
        let positions = vec![Vec3::ZERO, Vec3::new(100.0, 100.0, 100.0)];
        let hash = SpatialHash::build(&positions, 1.0);
        assert!(hash.query_candidates(&positions, 0).is_empty());
        assert!(hash.query_candidates(&positions, 1).is_empty());
    }

    #[test]
    fn a_body_never_reports_itself_as_a_candidate() {
        let positions = vec![Vec3::ZERO];
        let hash = SpatialHash::build(&positions, 1.0);
        assert!(hash.query_candidates(&positions, 0).is_empty());
    }

    #[test]
    fn every_true_overlapping_pair_in_a_random_cluster_is_found() {
        // Cross-check against a brute-force O(n^2) reference: every pair
        // within `cell_size` of each other (a conservative proxy for
        // "close enough to plausibly need narrow-phase," matching what
        // the 27-cell neighborhood is actually guaranteeing) must appear
        // in the hash's own candidate results. This is the single most
        // important test in this module -- a broad-phase that silently
        // misses a genuinely close pair produces a body that falls
        // through the floor or through another body with no error at all.
        let cell_size = 1.0;
        let mut positions = Vec::new();
        let mut state = 12345u64;
        let mut next = || {
            state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
            ((state >> 33) as f32 / u32::MAX as f32) * 10.0 - 5.0
        };
        for _ in 0..50 {
            positions.push(Vec3::new(next(), next(), next()));
        }

        let hash = SpatialHash::build(&positions, cell_size);
        for i in 0..positions.len() {
            let candidates = hash.query_candidates(&positions, i);
            for j in 0..positions.len() {
                if i == j {
                    continue;
                }
                if (positions[i] - positions[j]).length() < cell_size {
                    assert!(
                        candidates.contains(&(j as u32)),
                        "body {i} at {:?} and body {j} at {:?} are within cell_size but body {j} was not a candidate",
                        positions[i],
                        positions[j]
                    );
                }
            }
        }
    }

    #[test]
    fn cell_hash_does_not_collapse_adjacent_cells_to_the_same_bucket() {
        // A minimal decorrelation sanity check: three lattice-adjacent
        // cells along a single axis should not all hash identically for a
        // reasonably sized table -- if they did, the hash would
        // reintroduce exactly the locality-clustering problem hashing is
        // meant to avoid.
        let table_size = 97;
        let h0 = cell_hash(IVec3::new(0, 0, 0), table_size);
        let h1 = cell_hash(IVec3::new(1, 0, 0), table_size);
        let h2 = cell_hash(IVec3::new(2, 0, 0), table_size);
        assert!(!(h0 == h1 && h1 == h2), "adjacent cells collapsed to the same bucket: {h0}, {h1}, {h2}");
    }
}
