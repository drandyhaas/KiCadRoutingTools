//! Grid-based obstacle map for PCB routing.

use pyo3::prelude::*;
use rustc_hash::{FxHashMap, FxHashSet};

use crate::types::pack_xy;

/// Grid-based obstacle map
#[pyclass]
pub struct GridObstacleMap {
    /// Blocked cells per layer: layer -> set of (gx, gy) packed as u64
    pub blocked_cells: Vec<FxHashSet<u64>>,
    /// Blocked via positions
    pub blocked_vias: FxHashSet<u64>,
    /// Stub proximity costs: (gx, gy) -> cost
    pub stub_proximity: FxHashMap<u64, i32>,
    /// Number of layers
    #[pyo3(get)]
    pub num_layers: usize,
    /// BGA exclusion zones (min_gx, min_gy, max_gx, max_gy) - multiple zones supported
    pub bga_zones: Vec<(i32, i32, i32, i32)>,
    /// Allowed cells that override BGA zone blocking (for source/target points inside BGA)
    pub allowed_cells: FxHashSet<u64>,
    /// Source/target cells that can be routed to even if near obstacles
    /// These override regular blocking but NOT BGA zone blocking
    /// Stored per-layer: layer -> set of (gx, gy) packed as u64
    pub source_target_cells: Vec<FxHashSet<u64>>,
}

#[pymethods]
impl GridObstacleMap {
    #[new]
    pub fn new(num_layers: usize) -> Self {
        Self {
            blocked_cells: (0..num_layers).map(|_| FxHashSet::default()).collect(),
            blocked_vias: FxHashSet::default(),
            stub_proximity: FxHashMap::default(),
            num_layers,
            bga_zones: Vec::new(),
            allowed_cells: FxHashSet::default(),
            source_target_cells: (0..num_layers).map(|_| FxHashSet::default()).collect(),
        }
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
            num_layers: self.num_layers,
            bga_zones: self.bga_zones.clone(),
            allowed_cells: self.allowed_cells.clone(),
            source_target_cells: self.source_target_cells.clone(),
        }
    }

    /// Clear stub proximity costs (for reuse with different stubs)
    pub fn clear_stub_proximity(&mut self) {
        self.stub_proximity.clear();
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

    /// Add a blocked cell
    pub fn add_blocked_cell(&mut self, gx: i32, gy: i32, layer: usize) {
        if layer < self.num_layers {
            self.blocked_cells[layer].insert(pack_xy(gx, gy));
        }
    }

    /// Add a blocked via position
    pub fn add_blocked_via(&mut self, gx: i32, gy: i32) {
        self.blocked_vias.insert(pack_xy(gx, gy));
    }

    /// Set stub proximity cost
    pub fn set_stub_proximity(&mut self, gx: i32, gy: i32, cost: i32) {
        let key = pack_xy(gx, gy);
        let existing = self.stub_proximity.get(&key).copied().unwrap_or(0);
        if cost > existing {
            self.stub_proximity.insert(key, cost);
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
        let in_blocked_cells = self.blocked_cells[layer].contains(&key);

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

    /// Check if via is blocked
    #[inline]
    pub fn is_via_blocked(&self, gx: i32, gy: i32) -> bool {
        // Check explicit via blocks
        if self.blocked_vias.contains(&pack_xy(gx, gy)) {
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

    /// Get stub proximity cost
    #[inline]
    pub fn get_stub_proximity_cost(&self, gx: i32, gy: i32) -> i32 {
        self.stub_proximity.get(&pack_xy(gx, gy)).copied().unwrap_or(0)
    }
}
