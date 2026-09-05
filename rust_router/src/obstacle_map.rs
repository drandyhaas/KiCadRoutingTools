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

    /// Total number of set bits across all layers plus overflow cells (#422:
    /// used only by get_static_stats for memory diagnostics -- popcount is O(bits)
    /// but this is off the routing hot path).
    fn population(&self) -> usize {
        let bits: usize = self.bits.iter().map(|w| w.count_ones() as usize).sum();
        let ov: usize = self.overflow.iter().map(|s| s.len()).sum();
        bits + ov
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
    /// #422: per-layer bitmap of PERMANENT keep-out cells (board-edge clearance,
    /// off-board / outside-outline area, board cutouts / switch holes). These
    /// regions never change during routing, so they need only a set bit -- NOT a
    /// refcount `blocked_cells` entry (~38 B/cell vs 1 bit). On a large sparse
    /// board (split keyboard: 73 switch-hole cutouts + outside-outline) this is
    /// ~60% of all blocked cells; storing them as bits instead of hashmap entries
    /// is the #422 memory fix. is_blocked ORs this with `blocked_bitmap`, so a
    /// statically-blocked cell behaves identically to a refcounted one; it is
    /// simply never cleared by per-net remove/restore (no coincident-cell desync,
    /// because the two structures are independent -- a net's own copper still
    /// refcounts through `blocked_cells`, and removing it leaves the static bit).
    static_blocked_bitmap: BlockedBitmap,
    /// #422: single-"layer" bitmap of PERMANENT via keep-out cells (edge/cutout/
    /// outside). Mirrors static_blocked_bitmap for is_via_blocked. Vias span all
    /// copper layers, so one layer suffices (matches `blocked_vias` being a single
    /// map, not per-layer).
    static_via_bitmap: BlockedBitmap,
    /// Blocked via positions: packed (gx, gy) -> ref count
    pub blocked_vias: FxHashMap<u64, u16>,
    /// #568 / #530: via blocking at every OTHER via geometry a search may use,
    /// one refcounted map per rung, rung r stored at index r-1 (rung 0 is
    /// `blocked_vias`, the configured via). Rung 1 was the single "small fab
    /// rung" (advanced via, e.g. 0.25/0.15); #530's per-net via sizes add a
    /// rung per distinct size a routed net resolves to, larger OR smaller
    /// than the configured via. An EMPTY map means "not populated" and a
    /// query at that rung falls back to `blocked_vias` (conservative for a
    /// smaller via; callers stamping a LARGER via must populate its map), so
    /// existing single-rung callers behave identically.
    pub blocked_vias_rungs: Vec<FxHashMap<u64, u16>>,
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
    /// (B3, issue #386: u32 -- the old u8 wrapped/panicked on layers >= 8)
    pub cross_layer_tracks: FxHashMap<u64, u32>,
    /// P3 (#424 soft-knobs review): per-layer precomputed max attraction
    /// bonus, built once per net by build_attraction_field(); empty = the
    /// O(radius^2)-per-move scan fallback. attraction_field[L] answers
    /// "max falloff bonus at this cell from tracks on any layer != L".
    pub attraction_field: Vec<FxHashMap<u64, i32>>,
    /// Endpoint positions exempt from stub proximity costs (source and target)
    pub endpoint_exempt_positions: Vec<(i32, i32)>,
    /// Radius around endpoints to exempt from stub proximity costs
    pub endpoint_exempt_radius: i32,
    /// Free via positions: positions where layer changes have zero cost
    /// (e.g., through-hole pads on the same net - reuse existing holes instead of adding vias)
    pub free_via_positions: FxHashSet<u64>,
}

#[pymethods]
impl GridObstacleMap {
    #[new]
    pub fn new(num_layers: usize) -> Self {
        Self {
            blocked_cells: (0..num_layers).map(|_| FxHashMap::default()).collect(),
            blocked_bitmap: BlockedBitmap::new(num_layers),
            static_blocked_bitmap: BlockedBitmap::new(num_layers),
            static_via_bitmap: BlockedBitmap::new(1),
            blocked_vias: FxHashMap::default(),
            blocked_vias_rungs: Vec::new(),
            stub_proximity: FxHashMap::default(),
            layer_proximity_costs: (0..num_layers).map(|_| FxHashMap::default()).collect(),
            num_layers,
            bga_zones: Vec::new(),
            bga_proximity_radius: 0,
            allowed_cells: FxHashSet::default(),
            source_target_cells: (0..num_layers).map(|_| FxHashSet::default()).collect(),
            cross_layer_tracks: FxHashMap::default(),
            attraction_field: Vec::new(),
            endpoint_exempt_positions: Vec::new(),
            endpoint_exempt_radius: 0,
            free_via_positions: FxHashSet::default(),
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
            static_blocked_bitmap: self.static_blocked_bitmap.clone(),
            static_via_bitmap: self.static_via_bitmap.clone(),
            blocked_vias: self.blocked_vias.clone(),
            blocked_vias_rungs: self.blocked_vias_rungs.clone(),
            stub_proximity: self.stub_proximity.clone(),
            layer_proximity_costs: self.layer_proximity_costs.clone(),
            num_layers: self.num_layers,
            bga_zones: self.bga_zones.clone(),
            bga_proximity_radius: self.bga_proximity_radius,
            allowed_cells: self.allowed_cells.clone(),
            source_target_cells: self.source_target_cells.clone(),
            cross_layer_tracks: self.cross_layer_tracks.clone(),
            attraction_field: self.attraction_field.clone(),
            endpoint_exempt_positions: self.endpoint_exempt_positions.clone(),
            endpoint_exempt_radius: self.endpoint_exempt_radius,
            free_via_positions: self.free_via_positions.clone(),
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
            static_blocked_bitmap: self.static_blocked_bitmap.clone(),
            static_via_bitmap: self.static_via_bitmap.clone(),
            blocked_vias: self.blocked_vias.clone(),
            blocked_vias_rungs: self.blocked_vias_rungs.clone(),
            stub_proximity: self.stub_proximity.clone(),
            layer_proximity_costs: self.layer_proximity_costs.clone(),
            num_layers: self.num_layers,
            bga_zones: self.bga_zones.clone(),
            bga_proximity_radius: self.bga_proximity_radius,
            allowed_cells: self.allowed_cells.clone(),
            source_target_cells: (0..self.num_layers).map(|_| FxHashSet::default()).collect(),
            cross_layer_tracks: self.cross_layer_tracks.clone(),
            attraction_field: self.attraction_field.clone(),
            endpoint_exempt_positions: self.endpoint_exempt_positions.clone(),
            endpoint_exempt_radius: self.endpoint_exempt_radius,
            free_via_positions: self.free_via_positions.clone(),
        }
    }

    /// Get memory statistics for this obstacle map
    /// Returns (blocked_cells_count, blocked_vias_count, stub_proximity_count,
    ///          layer_proximity_count, cross_layer_count, source_target_count, free_vias_count)
    pub fn get_stats(&self) -> (usize, usize, usize, usize, usize, usize, usize, usize) {
        let blocked_cells_count: usize = self.blocked_cells.iter().map(|m| m.len()).sum();
        let blocked_vias_count = self.blocked_vias.len();
        // #568: small-rung via map appended LAST so positional consumers of the
        // legacy 7-tuple (audit labels, dump harnesses) stay index-compatible.
        // #530: with several rungs this is the SUM over all of them.
        let blocked_vias_small_count: usize =
            self.blocked_vias_rungs.iter().map(|m| m.len()).sum();
        let stub_proximity_count = self.stub_proximity.len();
        let layer_proximity_count: usize = self.layer_proximity_costs.iter().map(|m| m.len()).sum();
        let cross_layer_count = self.cross_layer_tracks.len();
        let source_target_count: usize = self.source_target_cells.iter().map(|s| s.len()).sum();
        let free_vias_count = self.free_via_positions.len();

        (blocked_cells_count, blocked_vias_count, stub_proximity_count,
         layer_proximity_count, cross_layer_count, source_target_count, free_vias_count,
         blocked_vias_small_count)
    }

    /// #422: Promote ALL current dynamic blocked cells/vias into the permanent
    /// static bitmaps, then empty the dynamic refcount maps. Valid ONLY on a map
    /// that will never have any of these cells individually removed again -- i.e.
    /// the BASE obstacle map, which holds non-target, non-rippable copper + board
    /// geometry and is never mutated after construction (target nets and rippable
    /// pre-existing nets live in the per-net caches added on top of a CLONE, not
    /// in base). After freezing, is_blocked/is_via_blocked read these cells from
    /// the static bitmap (identical result); the working clone then carries the
    /// frozen base as cheap bits and only the mutable per-net caches as refcount
    /// entries. Cuts the working map's hashmap to just the cache cells.
    pub fn freeze_dynamic_to_static(&mut self) {
        for layer in 0..self.num_layers {
            // Take the layer map out to avoid borrowing self while mutating the
            // static bitmap; the dynamic layer map is left empty.
            let m = std::mem::take(&mut self.blocked_cells[layer]);
            for &key in m.keys() {
                let (gx, gy) = unpack_xy(key);
                self.static_blocked_bitmap.set(gx, gy, layer);
            }
        }
        // The dynamic bitmap cached exactly these (now-frozen) cells; reset it.
        self.blocked_bitmap = BlockedBitmap::new(self.num_layers);
        let mv = std::mem::take(&mut self.blocked_vias);
        for &key in mv.keys() {
            let (gx, gy) = unpack_xy(key);
            self.static_via_bitmap.set(gx, gy, 0);
        }
        // #568: the static via bitmap is consulted by EVERY rung, so freezing
        // the full-size cells over-blocks a SMALLER rung exactly as the
        // pre-rung code did -- conservative, never wrong. A LARGER rung's
        // extra cells must survive the freeze, so a rung map is only cleared
        // when it is a subset of what was frozen (#530); otherwise its cells
        // beyond the frozen set are kept as a dynamic overlay.
        for m in self.blocked_vias_rungs.iter_mut() {
            let extra: Vec<u64> = m.keys().copied()
                .filter(|k| { let (gx, gy) = unpack_xy(*k); !self.static_via_bitmap.test(gx, gy, 0) })
                .collect();
            if extra.is_empty() {
                m.clear();
            } else {
                let mut keep = FxHashMap::default();
                for k in extra { keep.insert(k, *m.get(&k).unwrap_or(&1)); }
                *m = keep;
            }
        }
    }

    /// #422 diagnostic: (distinct_cells, cells_with_refcount>=2, max_refcount,
    /// distinct_vias, vias_refcount>=2) across the DYNAMIC refcount maps. Sizes
    /// the potential "bitmap + overflow" sparse rewrite (only refcount>=2 cells
    /// truly need a hashmap entry; refcount-1 cells are redundant with the bitmap).
    pub fn dynamic_refcount_stats(&self) -> (usize, usize, usize, usize, usize) {
        let mut distinct = 0usize;
        let mut ge2 = 0usize;
        let mut maxc = 0usize;
        for layer in &self.blocked_cells {
            for &c in layer.values() {
                distinct += 1;
                if c >= 2 { ge2 += 1; }
                if c as usize > maxc { maxc = c as usize; }
            }
        }
        let vd = self.blocked_vias.len();
        let vge2 = self.blocked_vias.values().filter(|&&c| c >= 2).count();
        (distinct, ge2, maxc, vd, vge2)
    }

    /// Clear stub proximity costs and zone centers (for reuse with different stubs).
    pub fn clear_stub_proximity(&mut self) {
        self.stub_proximity.clear();
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
        self.static_blocked_bitmap.bits.shrink_to_fit();
        self.static_via_bitmap.bits.shrink_to_fit();
    }

    /// Clear allowed cells (for reuse with different source/target)
    pub fn clear_allowed_cells(&mut self) {
        self.allowed_cells.clear();
    }

    /// Add an allowed cell that overrides BGA zone blocking
    pub fn add_allowed_cell(&mut self, gx: i32, gy: i32) {
        self.allowed_cells.insert(pack_xy(gx, gy));
    }

    /// #800: add a RECTANGLE of allowed cells in one crossing, inclusive of
    /// both bounds. Exactly `add_allowed_cell` over the same cells -- same set,
    /// same idempotence -- with the loop moved across the boundary.
    ///
    /// A rect rather than the `N x 3` cell batch the issue sketched, for two
    /// reasons found in the code. First, `allowed_cells` is ONE set keyed by
    /// `pack_xy` with no layer dimension at all, so an `N x 3` array would
    /// carry a column this map cannot store. Second, every caller is already a
    /// rectangle around a terminal (5 sites: the 21x21 exemption block in
    /// `route_net_with_obstacles`, the via-unblock 11x11, the target 11x11, and
    /// two diff-pair blocks), so a rect costs 4 ints per terminal where a cell
    /// batch would have Python build a 441-row array first -- paying much of
    /// the per-cell cost the batch exists to remove.
    ///
    /// Cost, per cell, which is the part that reproduces: ~50 ns -> ~3 ns, a
    /// 15-17x reduction, against the ~2.5x an `N x 3` cell batch would have
    /// given (93 -> 37 ns/cell in the issue's own microbenchmark). One rp2350
    /// route emits 655,531,651 exemption cells in 1,487,872 blocks (441 cells
    /// each), counted directly.
    ///
    /// Deliberately no whole-route percentage: it divides by a route time that
    /// swings with board, machine and run, and a single A/B pair cannot resolve
    /// a low-single-digit effect from noise.
    ///
    /// An inverted range (min > max) inserts nothing, which is what makes the
    /// callers' bounds clipping safe to hand over verbatim: a terminal whose
    /// block lies wholly outside `bounds` clips to an empty range and must add
    /// no cells, exactly as the Python `range()` did.
    pub fn add_allowed_rect(&mut self, min_gx: i32, min_gy: i32, max_gx: i32, max_gy: i32) {
        if min_gx > max_gx || min_gy > max_gy {
            return;
        }
        let w = (max_gx - min_gx + 1) as usize;
        let h = (max_gy - min_gy + 1) as usize;
        self.allowed_cells.reserve(w.saturating_mul(h));
        for gx in min_gx..=max_gx {
            for gy in min_gy..=max_gy {
                self.allowed_cells.insert(pack_xy(gx, gy));
            }
        }
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

    /// #568: add/remove a SMALL-rung (rung 1) blocked via (refcounted, mirrors
    /// blocked_vias). Callers stamping a net's via blocking at both rungs
    /// use the same cell-set discipline as the full-size map so the
    /// obstacle audit's working==base+caches invariant holds per rung.
    pub fn add_blocked_via_small(&mut self, gx: i32, gy: i32) {
        self.add_blocked_via_rung(1, gx, gy);
    }

    pub fn remove_blocked_via_small(&mut self, gx: i32, gy: i32) {
        self.remove_blocked_via_rung(1, gx, gy);
    }

    /// Batch forms (shape: N x 2, columns gx, gy).
    pub fn add_blocked_vias_small_batch(&mut self, vias: PyReadonlyArray2<i32>) {
        self.add_blocked_vias_rung_batch(1, vias);
    }

    pub fn remove_blocked_vias_small_batch(&mut self, vias: PyReadonlyArray2<i32>) {
        self.remove_blocked_vias_rung_batch(1, vias);
    }

    /// #530: the general rung API. Rung 0 is `blocked_vias` (the configured
    /// via); rung r >= 1 is its own refcounted map, created on first use.
    pub fn rung_count(&self) -> usize {
        self.blocked_vias_rungs.len() + 1
    }

    /// Number of blocked cells at `rung` (0 when the rung is unpopulated).
    pub fn rung_len(&self, rung: usize) -> usize {
        if rung == 0 { return self.blocked_vias.len(); }
        self.blocked_vias_rungs.get(rung - 1).map(|m| m.len()).unwrap_or(0)
    }

    pub fn add_blocked_via_rung(&mut self, rung: usize, gx: i32, gy: i32) {
        if rung == 0 { self.add_blocked_via(gx, gy); return; }
        let key = pack_xy(gx, gy);
        *self.rung_map_mut(rung).entry(key).or_insert(0) += 1;
    }

    pub fn remove_blocked_via_rung(&mut self, rung: usize, gx: i32, gy: i32) {
        let key = pack_xy(gx, gy);
        if rung == 0 {
            if let Some(count) = self.blocked_vias.get_mut(&key) {
                if *count > 1 { *count -= 1; } else { self.blocked_vias.remove(&key); }
            }
            return;
        }
        let m = self.rung_map_mut(rung);
        if let Some(count) = m.get_mut(&key) {
            if *count > 1 { *count -= 1; } else { m.remove(&key); }
        }
    }

    pub fn add_blocked_vias_rung_batch(&mut self, rung: usize, vias: PyReadonlyArray2<i32>) {
        if rung == 0 { self.add_blocked_vias_batch(vias); return; }
        let arr = vias.as_array();
        let m = self.rung_map_mut(rung);
        for row in arr.rows() {
            let key = pack_xy(row[0], row[1]);
            *m.entry(key).or_insert(0) += 1;
        }
    }

    pub fn remove_blocked_vias_rung_batch(&mut self, rung: usize, vias: PyReadonlyArray2<i32>) {
        if rung == 0 { self.remove_blocked_vias_batch(vias); return; }
        let arr = vias.as_array();
        let m = self.rung_map_mut(rung);
        for row in arr.rows() {
            let key = pack_xy(row[0], row[1]);
            if let Some(count) = m.get_mut(&key) {
                if *count > 1 { *count -= 1; } else { m.remove(&key); }
            }
        }
    }

    /// Every dynamically blocked via cell at `rung` as a list of (gx, gy) --
    /// so Python can build a base map at another via geometry and copy its
    /// via cells into a rung of the working map (#530). Sorted, so two maps
    /// stamped identically export identically.
    pub fn blocked_via_cells_at_rung(&self, rung: usize) -> Vec<(i32, i32)> {
        let src: Option<&FxHashMap<u64, u16>> = if rung == 0 {
            Some(&self.blocked_vias)
        } else {
            self.blocked_vias_rungs.get(rung - 1)
        };
        let mut rows: Vec<(i32, i32)> = Vec::new();
        if let Some(m) = src {
            rows.reserve(m.len());
            for &k in m.keys() {
                rows.push(unpack_xy(k));
            }
        }
        rows.sort_unstable();
        rows
    }

    /// #568 / #530: rung-aware via legality. rung 0 = the configured via
    /// (exactly is_via_blocked); rung >= 1 = another via geometry. An
    /// UNPOPULATED rung map falls back to rung 0 -- a smaller via is never
    /// legal where the caller hasn't proven it, and a larger via's map must
    /// have been populated by the caller.
    pub fn is_via_blocked_rung(&self, gx: i32, gy: i32, rung: usize) -> bool {
        if rung >= 1 {
            if let Some(m) = self.blocked_vias_rungs.get(rung - 1) {
                if !m.is_empty() {
                    let key = pack_xy(gx, gy);
                    if m.contains_key(&key) { return true; }
                    if self.static_via_bitmap.test(gx, gy, 0) { return true; }
                    for (min_gx, min_gy, max_gx, max_gy) in &self.bga_zones {
                        if gx >= *min_gx && gx <= *max_gx && gy >= *min_gy && gy <= *max_gy {
                            return !self.allowed_cells.contains(&key);
                        }
                    }
                    return false;
                }
            }
        }
        self.is_via_blocked(gx, gy)
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

    /// SPAN form of add_blocked_cells_batch (shape: N x 4, columns:
    /// gx, y_lo, y_hi, layer; both y bounds INCLUSIVE).
    ///
    /// The capsule rasterizer's output is convex per column, so a keep-out is
    /// naturally ~8 cells per column. Sending spans instead of cells is the
    /// same cell multiset -- each span lists each of its cells exactly once,
    /// so the refcounts land identically -- at 12 bytes per column instead of
    /// 8 bytes per cell (measured 5.2x smaller over 400 representative
    /// capsules), and it is what lets the Python-side memo cache spans rather
    /// than materialised cells. Expanding in Python would cost 7.4 us per
    /// call against a 0.17 us cache hit, i.e. ~65 s per route, which is why
    /// the expansion belongs here.
    pub fn add_blocked_cell_spans_batch(&mut self, spans: PyReadonlyArray2<i32>) {
        let arr = spans.as_array();
        for row in arr.rows() {
            let gx = row[0];
            let lo = row[1];
            let hi = row[2];
            let layer = row[3] as usize;
            if layer >= self.num_layers || hi < lo {
                continue;
            }
            for gy in lo..=hi {
                let key = pack_xy(gx, gy);
                let cnt = self.blocked_cells[layer].entry(key).or_insert(0);
                *cnt += 1;
                if *cnt == 1 {
                    self.blocked_bitmap.set(gx, gy, layer); // S2: 0->1 transition
                }
            }
        }
    }

    /// SPAN form of add_blocked_vias_batch (shape: N x 3: gx, y_lo, y_hi).
    pub fn add_blocked_via_spans_batch(&mut self, spans: PyReadonlyArray2<i32>) {
        let arr = spans.as_array();
        for row in arr.rows() {
            let gx = row[0];
            let lo = row[1];
            let hi = row[2];
            if hi < lo {
                continue;
            }
            for gy in lo..=hi {
                let key = pack_xy(gx, gy);
                *self.blocked_vias.entry(key).or_insert(0) += 1;
            }
        }
    }

    /// SPAN form of add_static_blocked_cells_batch (shape: N x 4).
    pub fn add_static_blocked_cell_spans_batch(&mut self, spans: PyReadonlyArray2<i32>) {
        let arr = spans.as_array();
        for row in arr.rows() {
            let gx = row[0];
            let lo = row[1];
            let hi = row[2];
            let layer = row[3] as usize;
            if layer >= self.num_layers || hi < lo {
                continue;
            }
            for gy in lo..=hi {
                self.static_blocked_bitmap.set(gx, gy, layer);
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

    /// #422: Add a single PERMANENT (static) blocked cell (bitmap only, no
    /// refcount entry). Mirrors add_blocked_cell for board geometry.
    pub fn add_static_blocked_cell(&mut self, gx: i32, gy: i32, layer: usize) {
        if layer < self.num_layers {
            self.static_blocked_bitmap.set(gx, gy, layer);
        }
    }

    /// #422: Add a single PERMANENT (static) blocked via cell (bitmap only).
    pub fn add_static_blocked_via(&mut self, gx: i32, gy: i32) {
        self.static_via_bitmap.set(gx, gy, 0);
    }

    /// #422: Batch add PERMANENT (static) blocked cells from numpy array
    /// (shape: N x 3, columns gx, gy, layer). These cells are set in the static
    /// keep-out bitmap ONLY -- no `blocked_cells` refcount entry -- because they
    /// are board geometry (edge clearance, off-board area, cutouts) that never
    /// changes during routing. is_blocked ORs the static bitmap with the dynamic
    /// one, so routing sees them identically to refcounted blocks; they are just
    /// stored as 1 bit instead of a ~38 B hashmap entry, and are immune to the
    /// per-net remove/restore cycle (never cleared).
    pub fn add_static_blocked_cells_batch(&mut self, cells: PyReadonlyArray2<i32>) {
        let arr = cells.as_array();
        for row in arr.rows() {
            let gx = row[0];
            let gy = row[1];
            let layer = row[2] as usize;
            if layer < self.num_layers {
                self.static_blocked_bitmap.set(gx, gy, layer);
            }
        }
    }

    /// #422: Batch add PERMANENT (static) blocked via cells (shape: N x 2,
    /// columns gx, gy). Set in the static via bitmap only (no `blocked_vias`
    /// refcount entry). Mirrors add_static_blocked_cells_batch for vias.
    pub fn add_static_blocked_vias_batch(&mut self, vias: PyReadonlyArray2<i32>) {
        let arr = vias.as_array();
        for row in arr.rows() {
            let gx = row[0];
            let gy = row[1];
            self.static_via_bitmap.set(gx, gy, 0);
        }
    }

    /// SPAN form of add_static_blocked_vias_batch (shape: N x 3: gx, lo, hi).
    /// Needed so _StaticStampProxy can redirect the span path too -- without
    /// it, a base build (which stamps through that proxy) would fall back to
    /// the cell form and lose the memo density the spans exist for.
    pub fn add_static_blocked_via_spans_batch(&mut self, spans: PyReadonlyArray2<i32>) {
        let arr = spans.as_array();
        for row in arr.rows() {
            let gx = row[0];
            let lo = row[1];
            let hi = row[2];
            if hi < lo {
                continue;
            }
            for gy in lo..=hi {
                self.static_via_bitmap.set(gx, gy, 0);
            }
        }
    }

    /// #422: (static_blocked_cells, static_blocked_vias) population counts, for
    /// memory diagnostics only. Kept separate from get_stats() so its tuple arity
    /// (and every existing caller) is unchanged.
    pub fn get_static_stats(&self) -> (usize, usize) {
        (self.static_blocked_bitmap.population(),
         self.static_via_bitmap.population())
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

    /// SPAN form of remove_blocked_cells_batch (shape: N x 4: gx, y_lo, y_hi, layer).
    ///
    /// The unstamp twin of add_blocked_cell_spans_batch, and it must expand to
    /// the SAME cell multiset that add did -- the per-net obstacle cache stamps
    /// a net's capsules on entry and unstamps the identical arrays on exit, so
    /// any asymmetry leaves cells stuck blocked and the router silently loses
    /// routable space (the invariant route.py asserts:
    /// working_obstacles == base_obstacles + sum(net_obstacles_cache)).
    /// Each span lists each of its cells exactly once, so refcounts land
    /// identically whether the caller used the cell form or this one.
    pub fn remove_blocked_cell_spans_batch(&mut self, spans: PyReadonlyArray2<i32>) {
        let arr = spans.as_array();
        for row in arr.rows() {
            let gx = row[0];
            let lo = row[1];
            let hi = row[2];
            let layer = row[3] as usize;
            if layer >= self.num_layers || hi < lo {
                continue;
            }
            for gy in lo..=hi {
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

    /// SPAN form of remove_blocked_vias_batch (shape: N x 3: gx, y_lo, y_hi).
    pub fn remove_blocked_via_spans_batch(&mut self, spans: PyReadonlyArray2<i32>) {
        let arr = spans.as_array();
        for row in arr.rows() {
            let gx = row[0];
            let lo = row[1];
            let hi = row[2];
            if hi < lo {
                continue;
            }
            for gy in lo..=hi {
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
    ///
    /// (Historically also took block_vias for the via_proximity_cost=0 ban
    /// mode; 0 now means "no extra via cost" so the ban -- and its B2
    /// refcount ledger -- is gone.)
    pub fn add_stub_proximity_costs_batch(
        &mut self,
        stubs: Vec<(i32, i32)>,
        radius: i32,
        max_cost: i32,
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
                    }
                }
            }
        }
    }

    /// Sum-composition stub proximity (KICAD_PROXIMITY_SUM): each group is one
    /// SOURCE (a net's stubs, a ripped net's ghost vias) with its own
    /// (radius, max_cost) falloff. Within a group the per-cell cost is the MAX
    /// over the group's point disks (dedupe: a net's 6 connector stubs are one
    /// source, and cost stays independent of point/sampling density); across
    /// groups the per-cell costs ADD (a corridor threading 10 nets' stub
    /// fields prices 10x one net's). Adds into the existing map -- callers
    /// clear (or start from a fresh clone) before the per-route stamp
    /// sequence, exactly like the max-mode batch above.
    ///
    /// max_zone_rects ('zoned' mode): inclusive grid rectangles (BGA zones
    /// expanded by the proximity radius -- the escape fields) inside which
    /// cells compose by MAX instead of ADD: there the stacked foreign-stub
    /// fields price a net's MANDATORY approach, not an avoidable crowd.
    #[pyo3(signature = (groups, max_zone_rects=None))]
    pub fn add_stub_proximity_costs_grouped(
        &mut self,
        groups: Vec<(Vec<(i32, i32)>, i32, i32)>,
        max_zone_rects: Option<Vec<(i32, i32, i32, i32)>>,
    ) {
        let rects = max_zone_rects.unwrap_or_default();
        for (points, radius, max_cost) in groups {
            if radius <= 0 || points.is_empty() {
                continue;
            }
            let radius_sq = radius * radius;
            let radius_f = radius as f32;
            let mut field: FxHashMap<u64, i32> = FxHashMap::default();
            for (gcx, gcy) in points {
                for dx in -radius..=radius {
                    for dy in -radius..=radius {
                        let dist_sq = dx * dx + dy * dy;
                        if dist_sq <= radius_sq {
                            let dist = (dist_sq as f32).sqrt();
                            let proximity = 1.0 - (dist / radius_f);
                            let cost = (proximity * max_cost as f32) as i32;
                            if cost > 0 {
                                let entry = field.entry(pack_xy(gcx + dx, gcy + dy)).or_insert(0);
                                if cost > *entry {
                                    *entry = cost;
                                }
                            }
                        }
                    }
                }
            }
            for (key, cost) in field {
                let entry = self.stub_proximity.entry(key).or_insert(0);
                let in_max_zone = !rects.is_empty() && {
                    let (gx, gy) = unpack_xy(key);
                    rects.iter().any(|&(x0, y0, x1, y1)|
                        gx >= x0 && gx <= x1 && gy >= y0 && gy <= y1)
                };
                if in_max_zone {
                    if cost > *entry {
                        *entry = cost;
                    }
                } else {
                    *entry += cost;
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
        // #422: OR the static keep-out bitmap (board edge / off-board / cutouts).
        // A statically-blocked cell behaves identically to a refcounted one --
        // same source/target override -- it is just stored as a bit, not a
        // hashmap entry, and never cleared by per-net remove/restore.
        if self.blocked_bitmap.test(gx, gy, layer)
            || self.static_blocked_bitmap.test(gx, gy, layer) {
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
        // #422: static (permanent) via keep-outs (edge/cutout/outside) live in a
        // bitmap, not the refcount map -- check it too. Unconditional block, same
        // as a blocked_vias entry (vias never get a source/target override here).
        if self.static_via_bitmap.test(gx, gy, 0) {
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
        // Lookup FIRST, exemption scan only on a nonzero hit (soft-knobs P4:
        // the exempt loop ran per lookup even for cells with no stub cost --
        // and the C5 single-ended exemptions made the list longer).
        let cost = match self.stub_proximity.get(&pack_xy(gx, gy)) {
            Some(&c) if c != 0 => c,
            _ => return 0,
        };
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
        cost
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
        if layer < self.num_layers && layer < 32 {
            // u32 bitmask supports up to 32 layers (B3: the u8 mask silently
            // wrapped `1 << layer` for layers >= 8 in release builds)
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

        // P3: precomputed field short-circuit (exact same semantics as the
        // scan below; the field was built with identical falloff math).
        if !self.attraction_field.is_empty() {
            if current_layer < self.attraction_field.len() {
                return *self.attraction_field[current_layer]
                    .get(&pack_xy(gx, gy)).unwrap_or(&0);
            }
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
                    // Check if any OTHER layer has a track here (B3: guard the
                    // shift -- current_layer >= 32 has no bit to mask out)
                    let own_bit = if current_layer < 32 { 1u32 << current_layer } else { 0 };
                    let other_layers = layers_mask & !own_bit;
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
        self.attraction_field.clear();
    }

    /// P3: precompute the per-layer attraction field so the hot path is an
    /// O(1) lookup instead of an O(radius^2) scan per candidate move. Call
    /// after the per-net add_cross_layer_track loop; cleared with the
    /// tracks. Gated to <= 8 layers (field memory is entries x disk x
    /// layers); more layers keep the scan fallback.
    pub fn build_attraction_field(&mut self, attraction_radius: i32, attraction_bonus: i32) {
        self.attraction_field.clear();
        if attraction_radius <= 0 || attraction_bonus <= 0 || self.num_layers > 8
            || self.cross_layer_tracks.is_empty() {
            return;
        }
        let radius_sq = attraction_radius * attraction_radius;
        let mut field: Vec<FxHashMap<u64, i32>> =
            (0..self.num_layers).map(|_| FxHashMap::default()).collect();
        for (&key, &mask) in self.cross_layer_tracks.iter() {
            let (cx, cy) = unpack_xy(key);
            for dx in -attraction_radius..=attraction_radius {
                for dy in -attraction_radius..=attraction_radius {
                    let dist_sq = dx * dx + dy * dy;
                    if dist_sq > radius_sq {
                        continue;
                    }
                    let dist = (dist_sq as f32).sqrt();
                    let falloff = 1.0 - (dist / attraction_radius as f32);
                    let bonus = (falloff * attraction_bonus as f32) as i32;
                    if bonus <= 0 {
                        continue;
                    }
                    let ckey = pack_xy(cx + dx, cy + dy);
                    for l in 0..self.num_layers {
                        let own_bit = if l < 32 { 1u32 << l } else { 0 };
                        if mask & !own_bit != 0 {
                            let e = field[l].entry(ckey).or_insert(0);
                            if bonus > *e {
                                *e = bonus;
                            }
                        }
                    }
                }
            }
        }
        self.attraction_field = field;
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

// #530: plain (non-Python) helpers for the rung maps. Kept OUTSIDE the
// #[pymethods] block: pyo3 wraps every method in that block for Python, and a
// method returning `&mut FxHashMap` cannot be wrapped.
impl GridObstacleMap {
    fn rung_map_mut(&mut self, rung: usize) -> &mut FxHashMap<u64, u16> {
        // rung 0 is blocked_vias; callers route rung 0 there explicitly
        let idx = rung - 1;
        while self.blocked_vias_rungs.len() <= idx {
            self.blocked_vias_rungs.push(FxHashMap::default());
        }
        &mut self.blocked_vias_rungs[idx]
    }
}
