//! Grid-based obstacle map for PCB routing.

use pyo3::prelude::*;
use numpy::PyReadonlyArray2;
use rustc_hash::{FxHashMap, FxHashSet};

use crate::types::pack_xy;

/// Grid-based obstacle map with reference counting for incremental updates.
/// Reference counting allows cells blocked by multiple nets to be correctly
/// managed when nets are added/removed.
#[pyclass]
pub struct GridObstacleMap {
    /// Blocked cells per layer: layer -> map of (gx, gy) packed as u64 -> ref count
    /// A cell is blocked if its ref count > 0
    pub blocked_cells: Vec<FxHashMap<u64, u16>>,
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
}

#[pymethods]
impl GridObstacleMap {
    #[new]
    pub fn new(num_layers: usize) -> Self {
        Self {
            blocked_cells: (0..num_layers).map(|_| FxHashMap::default()).collect(),
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
        }
    }

    /// Create a deep copy for a fresh route (clears source_target_cells)
    ///
    /// Use this instead of clone() when starting a new route to avoid
    /// source/target cells from a previous route leaking into the new one.
    pub fn clone_fresh(&self) -> Self {
        Self {
            blocked_cells: self.blocked_cells.clone(),
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

    /// Clear stub proximity costs and zone centers (for reuse with different stubs)
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
            *self.blocked_cells[layer].entry(key).or_insert(0) += 1;
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
                *self.blocked_cells[layer].entry(key).or_insert(0) += 1;
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
                    *self.blocked_cells[layer].entry(key).or_insert(0) += count;
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
                            // Increment ref count for blocked vias
                            *self.blocked_vias.entry(key).or_insert(0) += 1;
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

        let key = pack_xy(gx, gy);

        // Check if inside any BGA zone
        let in_bga_zone = self.bga_zones.iter().any(|(min_gx, min_gy, max_gx, max_gy)| {
            gx >= *min_gx && gx <= *max_gx && gy >= *min_gy && gy <= *max_gy
        });

        // Check if cell is in blocked_cells (tracks, stubs, pads from other nets)
        // With ref counting, presence in the map means count > 0 (we remove entries at 0)
        let in_blocked_cells = self.blocked_cells[layer].contains_key(&key);

        // If cell is blocked by other nets' obstacles, check if it's a source/target cell
        // source_target_cells can override blocking for exact endpoint positions only
        if in_blocked_cells {
            if self.source_target_cells[layer].contains(&key) {
                return false;
            }
            // Blocked by other net's track/stub - this takes precedence over BGA zone allowed_cells
            return true;
        }

        // If in BGA zone: allowed_cells overrides the zone blocking
        // (but NOT blocking from blocked_cells which was already checked above)
        if in_bga_zone {
            if self.allowed_cells.contains(&key) {
                // Allowed cell inside BGA zone - permit routing here
                return false;
            }
            // Not allowed - block inside BGA zone
            return true;
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
