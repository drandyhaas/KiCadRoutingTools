//! Grid-based obstacle map for PCB routing.

use pyo3::prelude::*;
use numpy::PyReadonlyArray2;
use rustc_hash::{FxHashMap, FxHashSet};

use crate::types::{pack_xy, unpack_xy};

/// Per-layer bitmap of blocked cells (S2, issue #384). A PURE CACHE of
/// "refcount > 0" for `GridObstacleMap.blocked_cells`: the refcount hashmaps
/// remain authoritative, and the bitmap is written ONLY at the 0->1 / 1->0
/// transitions inside the very same functions that touch the refcounts
/// (add_blocked_cell[s_batch], merge_blocked_from, remove_blocked_cells_batch)
/// -- never anywhere else, to keep the #208/#309 desync class closed.
///
/// The window grows lazily to cover the cells actually blocked (with a margin
/// so growth amortizes). Cells outside a size-capped window land in a per-layer
/// overflow hash set, so pathological coordinates degrade performance, never
/// correctness. `test()` is the is_blocked hot path: one bounds check + one
/// bit load (the overflow branch is a cheap is_empty() when unused).
#[derive(Clone)]
struct BlockedBitmap {
    min_x: i32,
    min_y: i32,
    width: i32,   // 0 = empty (nothing ever set)
    height: i32,
    words_per_layer: usize,
    num_layers: usize,
    bits: Vec<u64>, // num_layers * words_per_layer, layer-major
    overflow: Vec<FxHashSet<u64>>, // per-layer cells outside the capped window
}

/// Window growth margin in cells (amortizes reallocation).
const BITMAP_GROW_MARGIN: i32 = 128;
/// Hard cap on window area per layer (cells). 1<<26 cells = 8 MB of bits per
/// layer; a realistic board at 0.02-0.1 mm grid is a few thousand cells across,
/// far below this. Beyond the cap, cells go to the overflow sets.
const BITMAP_MAX_CELLS: i64 = 1 << 26;

impl BlockedBitmap {
    fn new(num_layers: usize) -> Self {
        Self {
            min_x: 0,
            min_y: 0,
            width: 0,
            height: 0,
            words_per_layer: 0,
            num_layers,
            bits: Vec::new(),
            overflow: (0..num_layers).map(|_| FxHashSet::default()).collect(),
        }
    }

    /// Linear cell index within a layer, or None if outside the window.
    #[inline]
    fn idx(&self, gx: i32, gy: i32) -> Option<usize> {
        let dx = gx.wrapping_sub(self.min_x);
        let dy = gy.wrapping_sub(self.min_y);
        if (dx as u32) < self.width as u32 && (dy as u32) < self.height as u32 {
            Some(dy as usize * self.width as usize + dx as usize)
        } else {
            None
        }
    }

    /// Hot-path test: is the cell's blocked bit set?
    #[inline]
    fn test(&self, gx: i32, gy: i32, layer: usize) -> bool {
        match self.idx(gx, gy) {
            Some(i) => {
                let w = self.bits[layer * self.words_per_layer + (i >> 6)];
                (w >> (i & 63)) & 1 != 0
            }
            None => {
                let ov = &self.overflow[layer];
                !ov.is_empty() && ov.contains(&pack_xy(gx, gy))
            }
        }
    }

    /// Mark a cell blocked (refcount transitioned 0 -> 1).
    fn set(&mut self, gx: i32, gy: i32, layer: usize) {
        if self.idx(gx, gy).is_none() {
            self.grow_to_include(gx, gy);
        }
        match self.idx(gx, gy) {
            Some(i) => {
                self.bits[layer * self.words_per_layer + (i >> 6)] |= 1u64 << (i & 63);
            }
            None => {
                self.overflow[layer].insert(pack_xy(gx, gy));
            }
        }
    }

    /// Unmark a cell (refcount transitioned 1 -> 0).
    fn clear(&mut self, gx: i32, gy: i32, layer: usize) {
        match self.idx(gx, gy) {
            Some(i) => {
                self.bits[layer * self.words_per_layer + (i >> 6)] &= !(1u64 << (i & 63));
            }
            None => {
                self.overflow[layer].remove(&pack_xy(gx, gy));
            }
        }
    }

    /// Grow the window to include (gx, gy) plus margin, remapping existing bits
    /// and migrating any overflow cells that fall inside the new window (they
    /// would otherwise be invisible to the in-window test path). Refuses to
    /// grow past BITMAP_MAX_CELLS; the caller then falls back to overflow.
    fn grow_to_include(&mut self, gx: i32, gy: i32) {
        let (new_min_x, new_min_y, new_max_x, new_max_y) = if self.width == 0 {
            (gx - BITMAP_GROW_MARGIN, gy - BITMAP_GROW_MARGIN,
             gx + BITMAP_GROW_MARGIN, gy + BITMAP_GROW_MARGIN)
        } else {
            (self.min_x.min(gx - BITMAP_GROW_MARGIN),
             self.min_y.min(gy - BITMAP_GROW_MARGIN),
             (self.min_x + self.width - 1).max(gx + BITMAP_GROW_MARGIN),
             (self.min_y + self.height - 1).max(gy + BITMAP_GROW_MARGIN))
        };
        let new_w = (new_max_x as i64 - new_min_x as i64 + 1) as i64;
        let new_h = (new_max_y as i64 - new_min_y as i64 + 1) as i64;
        if new_w * new_h > BITMAP_MAX_CELLS {
            return; // caller falls back to the overflow set
        }
        let new_w = new_w as i32;
        let new_h = new_h as i32;
        let new_words = ((new_w as usize * new_h as usize) + 63) / 64;
        let mut new_bits = vec![0u64; self.num_layers * new_words];

        // Remap old window bits (word-at-a-time is not worth it: growth is rare
        // and rows are short; bit-at-a-time keeps this obviously correct).
        if self.width > 0 {
            for layer in 0..self.num_layers {
                let old_base = layer * self.words_per_layer;
                let new_base = layer * new_words;
                for dy in 0..self.height {
                    let ny = (self.min_y + dy) - new_min_y;
                    for dx in 0..self.width {
                        let i = dy as usize * self.width as usize + dx as usize;
                        if (self.bits[old_base + (i >> 6)] >> (i & 63)) & 1 != 0 {
                            let nx = (self.min_x + dx) - new_min_x;
                            let ni = ny as usize * new_w as usize + nx as usize;
                            new_bits[new_base + (ni >> 6)] |= 1u64 << (ni & 63);
                        }
                    }
                }
            }
        }

        self.min_x = new_min_x;
        self.min_y = new_min_y;
        self.width = new_w;
        self.height = new_h;
        self.words_per_layer = new_words;
        self.bits = new_bits;

        // Migrate overflow cells that now fall inside the window.
        for layer in 0..self.num_layers {
            if self.overflow[layer].is_empty() {
                continue;
            }
            let inside: Vec<u64> = self.overflow[layer]
                .iter()
                .copied()
                .filter(|&key| {
                    let (x, y) = unpack_xy(key);
                    self.idx(x, y).is_some()
                })
                .collect();
            for key in inside {
                self.overflow[layer].remove(&key);
                let (x, y) = unpack_xy(key);
                if let Some(i) = self.idx(x, y) {
                    self.bits[layer * self.words_per_layer + (i >> 6)] |= 1u64 << (i & 63);
                }
            }
        }
    }
}

/// Grid-based obstacle map with reference counting for incremental updates.
/// Reference counting allows cells blocked by multiple nets to be correctly
/// managed when nets are added/removed.
#[pyclass]
pub struct GridObstacleMap {
    /// Blocked cells per layer: layer -> map of (gx, gy) packed as u64 -> ref count
    /// A cell is blocked if its ref count > 0
    pub blocked_cells: Vec<FxHashMap<u64, u16>>,
    /// S2: per-layer bitmap CACHE of "blocked_cells refcount > 0". Written only
    /// at 0->1 / 1->0 transitions inside the refcount-mutating functions.
    blocked_bitmap: BlockedBitmap,
    /// Blocked via positions: packed (gx, gy) -> ref count
    pub blocked_vias: FxHashMap<u64, u16>,
    /// Stub proximity costs: (gx, gy) -> cost
    pub stub_proximity: FxHashMap<u64, i32>,
    /// Layer-specific proximity costs (for track proximity on same layer)
    pub layer_proximity_costs: Vec<FxHashMap<u64, i32>>,
    /// Number of layers
    #[pyo3(get)]
    pub num_layers: usize,
    /// BGA exclusion zones (min_gx, min_gy, max_gx, max_gy) - multiple zones supported
    pub bga_zones: Vec<(i32, i32, i32, i32)>,
    /// BGA proximity radius in grid units (for vertical attraction exclusion)
    pub bga_proximity_radius: i32,
    /// Allowed cells that override BGA zone blocking (for source/target points inside BGA)
    pub allowed_cells: FxHashSet<u64>,
    /// Source/target cells that can be routed to even if near obstacles
    /// These override regular blocking but NOT BGA zone blocking
    /// Stored per-layer: layer -> set of (gx, gy) packed as u64
    pub source_target_cells: Vec<FxHashSet<u64>>,
    /// Cross-layer track positions for vertical alignment attraction
    /// Key: packed (gx, gy), Value: bitmask of layers that have tracks here
    pub cross_layer_tracks: FxHashMap<u64, u8>,
    /// Endpoint positions exempt from stub proximity costs (source and target)
    pub endpoint_exempt_positions: Vec<(i32, i32)>,
    /// Radius around endpoints to exempt from stub proximity costs
    pub endpoint_exempt_radius: i32,
    /// Free via positions: positions where layer changes have zero cost
    /// (e.g., through-hole pads on the same net - reuse existing holes instead of adding vias)
    pub free_via_positions: FxHashSet<u64>,
    /// B2 (issue #386): ledger of blocked_vias increments made by
    /// add_stub_proximity_costs_batch(block_vias=true), one entry per
    /// increment. clear_stub_proximity() drains it symmetrically; without
    /// this, per-net via bans accumulated monotonically across nets under
    /// --via-proximity-cost 0 (same refcount-leak class as #208/#309).
    stub_via_block_cells: Vec<u64>,
}

#[pymethods]
impl GridObstacleMap {
    #[new]
    pub fn new(num_layers: usize) -> Self {
        Self {
            blocked_cells: (0..num_layers).map(|_| FxHashMap::default()).collect(),
            blocked_bitmap: BlockedBitmap::new(num_layers),
            blocked_vias: FxHashMap::default(),
            stub_proximity: FxHashMap::default(),
            layer_proximity_costs: (0..num_layers).map(|_| FxHashMap::default()).collect(),
            num_layers,
            bga_zones: Vec::new(),
            bga_proximity_radius: 0,
            allowed_cells: FxHashSet::default(),
            source_target_cells: (0..num_layers).map(|_| FxHashSet::default()).collect(),
            cross_layer_tracks: FxHashMap::default(),
            endpoint_exempt_positions: Vec::new(),
            endpoint_exempt_radius: 0,
            free_via_positions: FxHashSet::default(),
            stub_via_block_cells: Vec::new(),
        }
    }

    /// Set endpoint positions exempt from stub proximity costs
    /// Call this before routing each net with source and target positions
    pub fn set_endpoint_exempt(&mut self, positions: Vec<(i32, i32)>, radius: i32) {
        self.endpoint_exempt_positions = positions;
        self.endpoint_exempt_radius = radius;
    }

    /// Clear endpoint exemptions
    pub fn clear_endpoint_exempt(&mut self) {
        self.endpoint_exempt_positions.clear();
        self.endpoint_exempt_radius = 0;
    }

    /// Set BGA proximity radius in grid units (for vertical attraction exclusion)
    pub fn set_bga_proximity_radius(&mut self, radius: i32) {
        self.bga_proximity_radius = radius;
    }

    /// Add a source/target cell that can be routed to even if near obstacles
    pub fn add_source_target_cell(&mut self, gx: i32, gy: i32, layer: usize) {
        if layer < self.num_layers {
            self.source_target_cells[layer].insert(pack_xy(gx, gy));
        }
    }

    /// Clear all source/target cells
    pub fn clear_source_target_cells(&mut self) {
        for layer_set in &mut self.source_target_cells {
            layer_set.clear();
        }
    }

    /// Create a deep copy of this obstacle map
    #[pyo3(name = "clone")]
    pub fn py_clone(&self) -> Self {
        Self {
            blocked_cells: self.blocked_cells.clone(),
            blocked_bitmap: self.blocked_bitmap.clone(),
            blocked_vias: self.blocked_vias.clone(),
            stub_proximity: self.stub_proximity.clone(),
            layer_proximity_costs: self.layer_proximity_costs.clone(),
            num_layers: self.num_layers,
            bga_zones: self.bga_zones.clone(),
            bga_proximity_radius: self.bga_proximity_radius,
            allowed_cells: self.allowed_cells.clone(),
            source_target_cells: self.source_target_cells.clone(),
            cross_layer_tracks: self.cross_layer_tracks.clone(),
            endpoint_exempt_positions: self.endpoint_exempt_positions.clone(),
            endpoint_exempt_radius: self.endpoint_exempt_radius,
            free_via_positions: self.free_via_positions.clone(),
            stub_via_block_cells: self.stub_via_block_cells.clone(),
        }
    }

    /// Create a deep copy for a fresh route (clears source_target_cells)
    ///
    /// Use this instead of clone() when starting a new route to avoid
    /// source/target cells from a previous route leaking into the new one.
    pub fn clone_fresh(&self) -> Self {
        Self {
            blocked_cells: self.blocked_cells.clone(),
            blocked_bitmap: self.blocked_bitmap.clone(),
            blocked_vias: self.blocked_vias.clone(),
            stub_proximity: self.stub_proximity.clone(),
            layer_proximity_costs: self.layer_proximity_costs.clone(),
            num_layers: self.num_layers,
            bga_zones: self.bga_zones.clone(),
            bga_proximity_radius: self.bga_proximity_radius,
            allowed_cells: self.allowed_cells.clone(),
            source_target_cells: (0..self.num_layers).map(|_| FxHashSet::default()).collect(),
            cross_layer_tracks: self.cross_layer_tracks.clone(),
            endpoint_exempt_positions: self.endpoint_exempt_positions.clone(),
            endpoint_exempt_radius: self.endpoint_exempt_radius,
            free_via_positions: self.free_via_positions.clone(),
            stub_via_block_cells: self.stub_via_block_cells.clone(),
        }
    }

    /// Get memory statistics for this obstacle map
    /// Returns (blocked_cells_count, blocked_vias_count, stub_proximity_count,
    ///          layer_proximity_count, cross_layer_count, source_target_count, free_vias_count)
    pub fn get_stats(&self) -> (usize, usize, usize, usize, usize, usize, usize) {
        let blocked_cells_count: usize = self.blocked_cells.iter().map(|m| m.len()).sum();
        let blocked_vias_count = self.blocked_vias.len();
        let stub_proximity_count = self.stub_proximity.len();
        let layer_proximity_count: usize = self.layer_proximity_costs.iter().map(|m| m.len()).sum();
        let cross_layer_count = self.cross_layer_tracks.len();
        let source_target_count: usize = self.source_target_cells.iter().map(|s| s.len()).sum();
        let free_vias_count = self.free_via_positions.len();

        (blocked_cells_count, blocked_vias_count, stub_proximity_count,
         layer_proximity_count, cross_layer_count, source_target_count, free_vias_count)
    }

    /// Clear stub proximity costs and zone centers (for reuse with different stubs).
    /// B2: also symmetrically removes the blocked_vias increments made by
    /// add_stub_proximity_costs_batch(block_vias=true) -- the per-net
    /// prepare/restore cycle already calls this, so the via bans now live
    /// exactly as long as the stub costs they came with.
    pub fn clear_stub_proximity(&mut self) {
        self.stub_proximity.clear();
        for key in std::mem::take(&mut self.stub_via_block_cells) {
            if let Some(count) = self.blocked_vias.get_mut(&key) {
                if *count > 1 {
                    *count -= 1;
                } else {
                    self.blocked_vias.remove(&key);
                }
            }
        }
    }

    /// Shrink all internal collections to fit their contents.
    /// Call this after bulk operations to release excess memory.
    pub fn shrink_to_fit(&mut self) {
        for layer_map in &mut self.blocked_cells {
            layer_map.shrink_to_fit();
        }
        self.blocked_vias.shrink_to_fit();
        self.stub_proximity.shrink_to_fit();
        for layer_map in &mut self.layer_proximity_costs {
            layer_map.shrink_to_fit();
        }
        self.allowed_cells.shrink_to_fit();
        for layer_set in &mut self.source_target_cells {
            layer_set.shrink_to_fit();
        }
        self.cross_layer_tracks.shrink_to_fit();
        self.free_via_positions.shrink_to_fit();
        self.stub_via_block_cells.shrink_to_fit();
    }

    /// Clear allowed cells (for reuse with different source/target)
    pub fn clear_allowed_cells(&mut self) {
        self.allowed_cells.clear();
    }

    /// Add an allowed cell that overrides BGA zone blocking
    pub fn add_allowed_cell(&mut self, gx: i32, gy: i32) {
        self.allowed_cells.insert(pack_xy(gx, gy));
    }

    /// Add a BGA exclusion zone (multiple zones supported)
    pub fn set_bga_zone(&mut self, min_gx: i32, min_gy: i32, max_gx: i32, max_gy: i32) {
        self.bga_zones.push((min_gx, min_gy, max_gx, max_gy));
    }

    /// Add a blocked cell (increments reference count)
    pub fn add_blocked_cell(&mut self, gx: i32, gy: i32, layer: usize) {
        if layer < self.num_layers {
            let key = pack_xy(gx, gy);
            let cnt = self.blocked_cells[layer].entry(key).or_insert(0);
            *cnt += 1;
            if *cnt == 1 {
                self.blocked_bitmap.set(gx, gy, layer); // S2: 0->1 transition
            }
        }
    }

    /// Add a blocked via position (increments reference count)
    pub fn add_blocked_via(&mut self, gx: i32, gy: i32) {
        let key = pack_xy(gx, gy);
        *self.blocked_vias.entry(key).or_insert(0) += 1;
    }

    /// Batch add blocked cells from numpy array (shape: N x 3, columns: gx, gy, layer)
    pub fn add_blocked_cells_batch(&mut self, cells: PyReadonlyArray2<i32>) {
        let arr = cells.as_array();
        for row in arr.rows() {
            let gx = row[0];
            let gy = row[1];
            let layer = row[2] as usize;
            if layer < self.num_layers {
                let key = pack_xy(gx, gy);
                let cnt = self.blocked_cells[layer].entry(key).or_insert(0);
                *cnt += 1;
                if *cnt == 1 {
                    self.blocked_bitmap.set(gx, gy, layer); // S2: 0->1 transition
                }
            }
        }
    }

    /// Batch add blocked vias from numpy array (shape: N x 2, columns: gx, gy)
    pub fn add_blocked_vias_batch(&mut self, vias: PyReadonlyArray2<i32>) {
        let arr = vias.as_array();
        for row in arr.rows() {
            let gx = row[0];
            let gy = row[1];
            let key = pack_xy(gx, gy);
            *self.blocked_vias.entry(key).or_insert(0) += 1;
        }
    }

    /// Merge blocked cells and vias from another obstacle map into this one
    /// (Adds reference counts from other map to this one)
    pub fn merge_blocked_from(&mut self, other: &GridObstacleMap) {
        for (layer, other_cells) in other.blocked_cells.iter().enumerate() {
            if layer < self.num_layers {
                for (&key, &count) in other_cells.iter() {
                    let cnt = self.blocked_cells[layer].entry(key).or_insert(0);
                    let was_zero = *cnt == 0;
                    *cnt += count;
                    if was_zero && *cnt > 0 {
                        let (gx, gy) = unpack_xy(key);
                        self.blocked_bitmap.set(gx, gy, layer); // S2: 0->1 transition
                    }
                }
            }
        }
        for (&key, &count) in other.blocked_vias.iter() {
            *self.blocked_vias.entry(key).or_insert(0) += count;
        }
    }

    /// Remove blocked cells from numpy array (decrements reference count, removes entry when count reaches 0)
    pub fn remove_blocked_cells_batch(&mut self, cells: PyReadonlyArray2<i32>) {
        let arr = cells.as_array();
        for row in arr.rows() {
            let gx = row[0];
            let gy = row[1];
            let layer = row[2] as usize;
            if layer < self.num_layers {
                let key = pack_xy(gx, gy);
                if let Some(count) = self.blocked_cells[layer].get_mut(&key) {
                    if *count > 1 {
                        *count -= 1;
                    } else {
                        self.blocked_cells[layer].remove(&key);
                        self.blocked_bitmap.clear(gx, gy, layer); // S2: 1->0 transition
                    }
                }
            }
        }
    }

    /// Remove blocked vias from numpy array (decrements reference count, removes entry when count reaches 0)
    pub fn remove_blocked_vias_batch(&mut self, vias: PyReadonlyArray2<i32>) {
        let arr = vias.as_array();
        for row in arr.rows() {
            let gx = row[0];
            let gy = row[1];
            let key = pack_xy(gx, gy);
            if let Some(count) = self.blocked_vias.get_mut(&key) {
                if *count > 1 {
                    *count -= 1;
                } else {
                    self.blocked_vias.remove(&key);
                }
            }
        }
    }

    /// Set stub proximity cost
    pub fn set_stub_proximity(&mut self, gx: i32, gy: i32, cost: i32) {
        let key = pack_xy(gx, gy);
        let existing = self.stub_proximity.get(&key).copied().unwrap_or(0);
        if cost > existing {
            self.stub_proximity.insert(key, cost);
        }
    }

    /// Batch compute and add stub proximity costs (much faster than Python iteration)
    /// stubs: Vec of (gx, gy) grid positions
    /// radius: proximity radius in grid units
    /// max_cost: maximum cost at stub center
    /// block_vias: if true, also block vias in proximity zones
    pub fn add_stub_proximity_costs_batch(
        &mut self,
        stubs: Vec<(i32, i32)>,
        radius: i32,
        max_cost: i32,
        block_vias: bool,
    ) {
        let radius_sq = radius * radius;
        let radius_f = radius as f32;

        for (gcx, gcy) in stubs {
            for dx in -radius..=radius {
                for dy in -radius..=radius {
                    let dist_sq = dx * dx + dy * dy;
                    if dist_sq <= radius_sq {
                        let dist = (dist_sq as f32).sqrt();
                        let proximity = 1.0 - (dist / radius_f);
                        let cost = (proximity * max_cost as f32) as i32;

                        let key = pack_xy(gcx + dx, gcy + dy);
                        let existing = self.stub_proximity.get(&key).copied().unwrap_or(0);
                        if cost > existing {
                            self.stub_proximity.insert(key, cost);
                        }

                        if block_vias {
                            // Increment ref count for blocked vias, and record the
                            // increment so clear_stub_proximity can undo it (B2)
                            *self.blocked_vias.entry(key).or_insert(0) += 1;
                            self.stub_via_block_cells.push(key);
                        }
                    }
                }
            }
        }
    }

    /// Check if cell is blocked
    #[inline]
    pub fn is_blocked(&self, gx: i32, gy: i32, layer: usize) -> bool {
        if layer >= self.num_layers {
            return true;
        }

        // Check if cell is in blocked_cells (tracks, stubs, pads from other nets).
        // S2: the per-layer bitmap caches "refcount > 0" (the refcount hashmaps
        // stay authoritative; the bitmap is written only at their transitions).
        if self.blocked_bitmap.test(gx, gy, layer) {
            // Blocked by other nets' obstacles - check if it's a source/target cell.
            // source_target_cells can override blocking for exact endpoint positions
            // only; this takes precedence over BGA zone allowed_cells.
            return !self.source_target_cells[layer].contains(&pack_xy(gx, gy));
        }

        // S2 hoist: with no BGA zones nothing else can block - skip the zone scan.
        if self.bga_zones.is_empty() {
            return false;
        }

        // If in BGA zone: allowed_cells overrides the zone blocking
        // (but NOT blocking from blocked_cells which was already checked above)
        let in_bga_zone = self.bga_zones.iter().any(|(min_gx, min_gy, max_gx, max_gy)| {
            gx >= *min_gx && gx <= *max_gx && gy >= *min_gy && gy <= *max_gy
        });
        if in_bga_zone {
            // Allowed cell inside BGA zone - permit routing here
            return !self.allowed_cells.contains(&pack_xy(gx, gy));
        }

        false
    }

    /// Check if cell is blocked with extra margin (for wide tracks)
    /// Checks all cells within margin radius - if any is blocked, returns true.
    /// This is O(margin^2) so only use for wide power tracks.
    #[inline]
    pub fn is_blocked_with_margin(&self, gx: i32, gy: i32, layer: usize, margin: i32) -> bool {
        if margin <= 0 {
            return self.is_blocked(gx, gy, layer);
        }

        // Check all cells within the margin square
        // Early exit on first blocked cell found
        for dx in -margin..=margin {
            for dy in -margin..=margin {
                if self.is_blocked(gx + dx, gy + dy, layer) {
                    return true;
                }
            }
        }
        false
    }

    /// Swept-capsule clearance check for wide tracks (issues #156 / #173): true if
    /// any blocked cell centre is within Euclidean distance `r` (in grid cells) of
    /// the SEGMENT (gx1,gy1)->(gx2,gy2) -- i.e. the extra-half-width `r` of a wide
    /// track swept along this A* move.
    ///
    /// Replaces is_blocked_with_margin for wide tracks. That function tested a
    /// Chebyshev SQUARE around ONLY the destination cell, so (a) it over-covered
    /// corners and (b) it never checked the swept body of a 45deg move -- a
    /// diagonal step could slip a blocked cell sub-cell between its endpoints
    /// (the residual grazes in #173, and why #156's point-disc was a no-win:
    /// a disc at the endpoint still misses the swept segment). This checks the
    /// true point-to-segment distance with an exact Euclidean radius, covering
    /// the diagonal sweep. A degenerate segment (p1==p2, used for layer-change
    /// checks) reduces to a disc of radius `r` about the point. `r <= 0` falls
    /// back to a plain destination-cell check (base-width tracks: identical to
    /// the old is_blocked_with_margin(.., 0)).
    #[inline]
    pub fn segment_blocked(&self, gx1: i32, gy1: i32, gx2: i32, gy2: i32,
                           layer: usize, r: f64) -> bool {
        if r <= 0.0 {
            return self.is_blocked(gx2, gy2, layer);
        }
        let r2 = r * r;
        let rc = r.ceil() as i32;
        let lo_x = gx1.min(gx2) - rc;
        let hi_x = gx1.max(gx2) + rc;
        let lo_y = gy1.min(gy2) - rc;
        let hi_y = gy1.max(gy2) + rc;
        let (ax, ay) = (gx1 as f64, gy1 as f64);
        let dx = (gx2 - gx1) as f64;
        let dy = (gy2 - gy1) as f64;
        let len2 = dx * dx + dy * dy;
        for cx in lo_x..=hi_x {
            for cy in lo_y..=hi_y {
                if !self.is_blocked(cx, cy, layer) {
                    continue;
                }
                let px = cx as f64;
                let py = cy as f64;
                let d2 = if len2 <= 0.0 {
                    let ex = px - ax;
                    let ey = py - ay;
                    ex * ex + ey * ey
                } else {
                    let mut t = ((px - ax) * dx + (py - ay) * dy) / len2;
                    if t < 0.0 { t = 0.0; } else if t > 1.0 { t = 1.0; }
                    let ex = px - (ax + t * dx);
                    let ey = py - (ay + t * dy);
                    ex * ex + ey * ey
                };
                if d2 <= r2 {
                    return true;
                }
            }
        }
        false
    }

    /// Check if via is blocked
    #[inline]
    pub fn is_via_blocked(&self, gx: i32, gy: i32) -> bool {
        // Check explicit via blocks (with ref counting, presence means count > 0)
        if self.blocked_vias.contains_key(&pack_xy(gx, gy)) {
            return true;
        }
        // Check BGA zones - vias blocked inside unless allowed
        let key = pack_xy(gx, gy);
        for (min_gx, min_gy, max_gx, max_gy) in &self.bga_zones {
            if gx >= *min_gx && gx <= *max_gx && gy >= *min_gy && gy <= *max_gy {
                return !self.allowed_cells.contains(&key);
            }
        }
        false
    }

    /// All open (non-via-blocked) cells within Chebyshev `radius` of (cx, cy),
    /// excluding the center, returned nearest-first (by squared Euclidean
    /// distance). Lets the Python via-site search (`find_via_position`) replace
    /// its per-cell `is_via_blocked()` spiral - one batched query across the FFI
    /// boundary instead of O(radius^2) calls. (v0.16.0)
    pub fn open_via_cells_within(&self, cx: i32, cy: i32, radius: i32) -> Vec<(i32, i32)> {
        let mut cells: Vec<(i64, i32, i32)> = Vec::new();
        for dy in -radius..=radius {
            for dx in -radius..=radius {
                if dx == 0 && dy == 0 {
                    continue;
                }
                let gx = cx + dx;
                let gy = cy + dy;
                if !self.is_via_blocked(gx, gy) {
                    let d2 = (dx as i64) * (dx as i64) + (dy as i64) * (dy as i64);
                    cells.push((d2, gx, gy));
                }
            }
        }
        cells.sort_by_key(|c| c.0);
        cells.into_iter().map(|c| (c.1, c.2)).collect()
    }

    /// Check if position is within BGA proximity radius of any BGA zone
    #[inline]
    pub fn is_in_bga_proximity(&self, gx: i32, gy: i32) -> bool {
        if self.bga_proximity_radius <= 0 {
            return false;
        }
        for (min_gx, min_gy, max_gx, max_gy) in &self.bga_zones {
            // Expand zone by proximity radius
            let expanded_min_gx = min_gx - self.bga_proximity_radius;
            let expanded_min_gy = min_gy - self.bga_proximity_radius;
            let expanded_max_gx = max_gx + self.bga_proximity_radius;
            let expanded_max_gy = max_gy + self.bga_proximity_radius;
            if gx >= expanded_min_gx && gx <= expanded_max_gx &&
               gy >= expanded_min_gy && gy <= expanded_max_gy {
                return true;
            }
        }
        false
    }

    /// Check if position is in any proximity zone (stub or BGA)
    /// Used to determine if a route endpoint requires proximity heuristic
    #[inline]
    pub fn is_in_any_proximity_zone(&self, gx: i32, gy: i32) -> bool {
        // Check stub proximity (has non-zero cost at this position)
        if self.stub_proximity.get(&pack_xy(gx, gy)).copied().unwrap_or(0) > 0 {
            return true;
        }
        // Check BGA proximity
        self.is_in_bga_proximity(gx, gy)
    }

    /// Get stub proximity cost
    /// Returns 0 if position is within endpoint_exempt_radius of any endpoint
    #[inline]
    pub fn get_stub_proximity_cost(&self, gx: i32, gy: i32) -> i32 {
        // Check if exempt due to being near an endpoint (source/target)
        if self.endpoint_exempt_radius > 0 {
            let radius_sq = self.endpoint_exempt_radius * self.endpoint_exempt_radius;
            for (ex, ey) in &self.endpoint_exempt_positions {
                let dx = gx - ex;
                let dy = gy - ey;
                if dx * dx + dy * dy <= radius_sq {
                    return 0;
                }
            }
        }
        self.stub_proximity.get(&pack_xy(gx, gy)).copied().unwrap_or(0)
    }

    /// Set layer-specific proximity cost (for track proximity on same layer)
    pub fn set_layer_proximity(&mut self, gx: i32, gy: i32, layer: usize, cost: i32) {
        if layer < self.num_layers && cost > 0 {
            let key = pack_xy(gx, gy);
            let entry = self.layer_proximity_costs[layer].entry(key).or_insert(0);
            *entry = (*entry).max(cost);
        }
    }

    /// Batch set layer proximity costs from numpy array
    /// Array should have shape (N, 4) with columns [layer, gx, gy, cost]
    pub fn set_layer_proximity_batch(&mut self, costs: PyReadonlyArray2<i32>) {
        let arr = costs.as_array();
        for row in arr.rows() {
            let layer = row[0] as usize;
            let gx = row[1];
            let gy = row[2];
            let cost = row[3];
            if layer < self.num_layers && cost > 0 {
                let key = pack_xy(gx, gy);
                let entry = self.layer_proximity_costs[layer].entry(key).or_insert(0);
                *entry = (*entry).max(cost);
            }
        }
    }

    /// Get layer-specific proximity cost
    #[inline]
    pub fn get_layer_proximity_cost(&self, gx: i32, gy: i32, layer: usize) -> i32 {
        if layer >= self.num_layers {
            return 0;
        }
        self.layer_proximity_costs[layer]
            .get(&pack_xy(gx, gy))
            .copied()
            .unwrap_or(0)
    }

    /// Clear layer-specific proximity costs
    pub fn clear_layer_proximity(&mut self) {
        for layer_map in &mut self.layer_proximity_costs {
            layer_map.clear();
        }
    }

    /// Add a track position for cross-layer attraction lookup
    pub fn add_cross_layer_track(&mut self, gx: i32, gy: i32, layer: usize) {
        if layer < self.num_layers && layer < 8 {
            // u8 bitmask supports up to 8 layers
            let key = pack_xy(gx, gy);
            let entry = self.cross_layer_tracks.entry(key).or_insert(0);
            *entry |= 1 << layer;
        }
    }

    /// Get cross-layer attraction bonus (positive = cost reduction) at position for given layer
    /// Returns a bonus if OTHER layers have tracks here (not the current layer)
    /// Returns 0 if position is in stub proximity zone or BGA exclusion zone
    #[inline]
    pub fn get_cross_layer_attraction(
        &self,
        gx: i32,
        gy: i32,
        current_layer: usize,
        attraction_radius: i32,
        attraction_bonus: i32,
    ) -> i32 {
        if attraction_radius <= 0 || attraction_bonus <= 0 {
            return 0;
        }

        // No attraction bonus in stub proximity zones (near unrouted stubs)
        let key = pack_xy(gx, gy);
        if self.stub_proximity.contains_key(&key) {
            return 0;
        }

        // No attraction bonus inside BGA exclusion zones (but allow within proximity radius)
        let in_bga_zone = self.bga_zones.iter().any(|(min_gx, min_gy, max_gx, max_gy)| {
            gx >= *min_gx && gx <= *max_gx && gy >= *min_gy && gy <= *max_gy
        });
        if in_bga_zone {
            return 0;
        }

        let radius_sq = attraction_radius * attraction_radius;
        let mut max_bonus = 0;

        for dx in -attraction_radius..=attraction_radius {
            for dy in -attraction_radius..=attraction_radius {
                let dist_sq = dx * dx + dy * dy;
                if dist_sq > radius_sq {
                    continue;
                }

                let key = pack_xy(gx + dx, gy + dy);
                if let Some(&layers_mask) = self.cross_layer_tracks.get(&key) {
                    // Check if any OTHER layer has a track here
                    let other_layers = layers_mask & !(1u8 << current_layer);
                    if other_layers != 0 {
                        // Linear falloff: full bonus at center, zero at radius edge
                        let dist = (dist_sq as f32).sqrt();
                        let falloff = 1.0 - (dist / attraction_radius as f32);
                        let bonus = (falloff * attraction_bonus as f32) as i32;
                        max_bonus = max_bonus.max(bonus);
                    }
                }
            }
        }

        max_bonus
    }

    /// Clear cross-layer track data
    pub fn clear_cross_layer_tracks(&mut self) {
        self.cross_layer_tracks.clear();
    }

    /// Add a free via position (layer change here has zero cost)
    /// Used for through-hole pads on the same net where we can reuse the existing hole
    pub fn add_free_via(&mut self, gx: i32, gy: i32) {
        self.free_via_positions.insert(pack_xy(gx, gy));
    }

    /// Check if position is a free via (zero-cost layer change)
    #[inline]
    pub fn is_free_via(&self, gx: i32, gy: i32) -> bool {
        self.free_via_positions.contains(&pack_xy(gx, gy))
    }

    /// Clear all free via positions
    pub fn clear_free_vias(&mut self) {
        self.free_via_positions.clear();
    }

    /// Batch add free via positions from a list of (gx, gy) tuples
    pub fn add_free_vias_batch(&mut self, positions: Vec<(i32, i32)>) {
        for (gx, gy) in positions {
            self.free_via_positions.insert(pack_xy(gx, gy));
        }
    }
}
