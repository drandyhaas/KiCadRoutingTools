//! Grid-based A* PCB Router - Rust implementation for speed.
//!
//! This is a high-performance implementation of the grid router algorithm.
//! It's designed to be called from Python via PyO3 bindings.

use pyo3::prelude::*;
use rustc_hash::{FxHashMap, FxHashSet};
use std::cmp::Ordering;
use std::collections::BinaryHeap;

/// Grid state: (x, y, layer) packed into a single u64 for fast hashing
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
struct GridState {
    gx: i32,
    gy: i32,
    layer: u8,
}

impl GridState {
    #[inline]
    fn new(gx: i32, gy: i32, layer: u8) -> Self {
        Self { gx, gy, layer }
    }

    #[inline]
    fn as_key(&self) -> u64 {
        // Pack into u64 for fast hashing: 20 bits x, 20 bits y, 8 bits layer
        let x = (self.gx as u64) & 0xFFFFF;
        let y = (self.gy as u64) & 0xFFFFF;
        let l = self.layer as u64;
        (x << 28) | (y << 8) | l
    }
}

/// A* open set entry with reverse ordering for min-heap
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct OpenEntry {
    f_score: i32,
    g_score: i32,
    state: GridState,
    counter: u32, // Tie-breaker for deterministic ordering
}

impl Ord for OpenEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        // Reverse ordering for min-heap (lowest f_score first)
        other.f_score.cmp(&self.f_score)
            .then_with(|| other.counter.cmp(&self.counter))
    }
}

impl PartialOrd for OpenEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// 8 directions for octilinear routing
const DIRECTIONS: [(i32, i32); 8] = [
    (1, 0),   // East
    (1, -1),  // NE
    (0, -1),  // North
    (-1, -1), // NW
    (-1, 0),  // West
    (-1, 1),  // SW
    (0, 1),   // South
    (1, 1),   // SE
];

const ORTHO_COST: i32 = 1000;
const DIAG_COST: i32 = 1414; // sqrt(2) * 1000

/// Grid-based obstacle map
#[pyclass]
struct GridObstacleMap {
    /// Blocked cells per layer: layer -> set of (gx, gy) packed as u64
    blocked_cells: Vec<FxHashSet<u64>>,
    /// Blocked via positions
    blocked_vias: FxHashSet<u64>,
    /// Stub proximity costs: (gx, gy) -> cost
    stub_proximity: FxHashMap<u64, i32>,
    /// Number of layers
    #[pyo3(get)]
    num_layers: usize,
    /// BGA exclusion zones (min_gx, min_gy, max_gx, max_gy) - multiple zones supported
    bga_zones: Vec<(i32, i32, i32, i32)>,
    /// Allowed cells that override BGA zone blocking (for source/target points inside BGA)
    allowed_cells: FxHashSet<u64>,
    /// Source/target cells that can be routed to even if near obstacles
    /// These override regular blocking but NOT BGA zone blocking
    /// Stored per-layer: layer -> set of (gx, gy) packed as u64
    source_target_cells: Vec<FxHashSet<u64>>,
}

#[inline]
fn pack_xy(gx: i32, gy: i32) -> u64 {
    let x = (gx as u64) & 0xFFFFFFFF;
    let y = (gy as u64) & 0xFFFFFFFF;
    (x << 32) | y
}

#[pymethods]
impl GridObstacleMap {
    #[new]
    fn new(num_layers: usize) -> Self {
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
    fn add_source_target_cell(&mut self, gx: i32, gy: i32, layer: usize) {
        if layer < self.num_layers {
            self.source_target_cells[layer].insert(pack_xy(gx, gy));
        }
    }

    /// Clear all source/target cells
    fn clear_source_target_cells(&mut self) {
        for layer_set in &mut self.source_target_cells {
            layer_set.clear();
        }
    }

    /// Create a deep copy of this obstacle map
    fn clone(&self) -> Self {
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
    fn clear_stub_proximity(&mut self) {
        self.stub_proximity.clear();
    }

    /// Clear allowed cells (for reuse with different source/target)
    fn clear_allowed_cells(&mut self) {
        self.allowed_cells.clear();
    }

    /// Add an allowed cell that overrides BGA zone blocking
    fn add_allowed_cell(&mut self, gx: i32, gy: i32) {
        self.allowed_cells.insert(pack_xy(gx, gy));
    }

    /// Add a BGA exclusion zone (multiple zones supported)
    fn set_bga_zone(&mut self, min_gx: i32, min_gy: i32, max_gx: i32, max_gy: i32) {
        self.bga_zones.push((min_gx, min_gy, max_gx, max_gy));
    }

    /// Add a blocked cell
    fn add_blocked_cell(&mut self, gx: i32, gy: i32, layer: usize) {
        if layer < self.num_layers {
            self.blocked_cells[layer].insert(pack_xy(gx, gy));
        }
    }

    /// Add a blocked via position
    fn add_blocked_via(&mut self, gx: i32, gy: i32) {
        self.blocked_vias.insert(pack_xy(gx, gy));
    }

    /// Set stub proximity cost
    fn set_stub_proximity(&mut self, gx: i32, gy: i32, cost: i32) {
        let key = pack_xy(gx, gy);
        let existing = self.stub_proximity.get(&key).copied().unwrap_or(0);
        if cost > existing {
            self.stub_proximity.insert(key, cost);
        }
    }

    /// Check if cell is blocked
    #[inline]
    fn is_blocked(&self, gx: i32, gy: i32, layer: usize) -> bool {
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
    fn is_via_blocked(&self, gx: i32, gy: i32) -> bool {
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
    fn get_stub_proximity_cost(&self, gx: i32, gy: i32) -> i32 {
        self.stub_proximity.get(&pack_xy(gx, gy)).copied().unwrap_or(0)
    }
}

/// Grid A* Router
#[pyclass]
struct GridRouter {
    via_cost: i32,
    h_weight: f32,
}

#[pymethods]
impl GridRouter {
    #[new]
    fn new(via_cost: i32, h_weight: f32) -> Self {
        Self { via_cost, h_weight }
    }

    /// Route from multiple source points to multiple target points.
    /// Returns (path, iterations) where path is list of (gx, gy, layer) tuples,
    /// or (None, iterations) if no path found.
    ///
    /// collinear_vias: If true, after a via the route must continue in the same
    /// direction as before the via (for diff pair routing to ensure clean via geometry).
    #[pyo3(signature = (obstacles, sources, targets, max_iterations, collinear_vias=false))]
    fn route_multi(
        &self,
        obstacles: &GridObstacleMap,
        sources: Vec<(i32, i32, u8)>,
        targets: Vec<(i32, i32, u8)>,
        max_iterations: u32,
        collinear_vias: bool,
    ) -> (Option<Vec<(i32, i32, u8)>>, u32) {
        // Convert targets to set for O(1) lookup
        let target_set: FxHashSet<u64> = targets
            .iter()
            .map(|(gx, gy, layer)| GridState::new(*gx, *gy, *layer).as_key())
            .collect();

        let target_states: Vec<GridState> = targets
            .iter()
            .map(|(gx, gy, layer)| GridState::new(*gx, *gy, *layer))
            .collect();

        // Initialize open set with all sources
        let mut open_set = BinaryHeap::new();
        let mut g_costs: FxHashMap<u64, i32> = FxHashMap::default();
        let mut parents: FxHashMap<u64, u64> = FxHashMap::default();
        let mut closed: FxHashSet<u64> = FxHashSet::default();
        let mut counter: u32 = 0;

        for (gx, gy, layer) in sources {
            let state = GridState::new(gx, gy, layer);
            let key = state.as_key();
            let h = self.heuristic_to_targets(&state, &target_states);
            open_set.push(OpenEntry {
                f_score: h,
                g_score: 0,
                state,
                counter,
            });
            counter += 1;
            g_costs.insert(key, 0);
        }

        let mut iterations: u32 = 0;

        while let Some(current_entry) = open_set.pop() {
            if iterations >= max_iterations {
                break;
            }
            iterations += 1;

            let current = current_entry.state;
            let current_key = current.as_key();
            let g = current_entry.g_score;

            if closed.contains(&current_key) {
                continue;
            }
            closed.insert(current_key);

            // Check if reached target
            if target_set.contains(&current_key) {
                // Reconstruct path
                let path = self.reconstruct_path(&parents, current_key, &g_costs);
                return (Some(path), iterations);
            }

            // Check via constraints for diff pair routing (collinear_vias mode):
            // 1. If we just came through a via: must continue in exact same direction as before via
            // 2. If we're one step after a via exit: must be within ±45° of the pre-via direction
            // This ensures clean via geometry for differential pairs
            let (required_direction, allowed_45deg_from): (Option<(i32, i32)>, Option<(i32, i32)>) = if collinear_vias {
                self.get_via_direction_constraints(&parents, current_key, &current)
            } else {
                (None, None)
            };

            // Expand neighbors - 8 directions
            for (dx, dy) in DIRECTIONS {
                // If we have a required exact direction (just came through via), only allow that direction
                if let Some((req_dx, req_dy)) = required_direction {
                    if dx != req_dx || dy != req_dy {
                        continue;
                    }
                }
                // If we're one step after via, must be within ±45° of the pre-via direction
                else if let Some((base_dx, base_dy)) = allowed_45deg_from {
                    if !Self::is_within_45_degrees(dx, dy, base_dx, base_dy) {
                        continue;
                    }
                }

                let ngx = current.gx + dx;
                let ngy = current.gy + dy;

                if obstacles.is_blocked(ngx, ngy, current.layer as usize) {
                    continue;
                }

                let neighbor = GridState::new(ngx, ngy, current.layer);
                let neighbor_key = neighbor.as_key();

                if closed.contains(&neighbor_key) {
                    continue;
                }

                let move_cost = if dx != 0 && dy != 0 { DIAG_COST } else { ORTHO_COST };
                let proximity_cost = obstacles.get_stub_proximity_cost(ngx, ngy);
                let new_g = g + move_cost + proximity_cost;

                let existing_g = g_costs.get(&neighbor_key).copied().unwrap_or(i32::MAX);
                if new_g < existing_g {
                    g_costs.insert(neighbor_key, new_g);
                    parents.insert(neighbor_key, current_key);
                    let h = self.heuristic_to_targets(&neighbor, &target_states);
                    let f = new_g + h;
                    open_set.push(OpenEntry {
                        f_score: f,
                        g_score: new_g,
                        state: neighbor,
                        counter,
                    });
                    counter += 1;
                }
            }

            // Try via to other layers
            if !obstacles.is_via_blocked(current.gx, current.gy) {
                for layer in 0..obstacles.num_layers as u8 {
                    if layer == current.layer {
                        continue;
                    }

                    // Check if destination layer is blocked at this position
                    if obstacles.is_blocked(current.gx, current.gy, layer as usize) {
                        continue;
                    }

                    let neighbor = GridState::new(current.gx, current.gy, layer);
                    let neighbor_key = neighbor.as_key();

                    if closed.contains(&neighbor_key) {
                        continue;
                    }

                    let proximity_cost = obstacles.get_stub_proximity_cost(current.gx, current.gy) * 2;
                    let new_g = g + self.via_cost + proximity_cost;

                    let existing_g = g_costs.get(&neighbor_key).copied().unwrap_or(i32::MAX);
                    if new_g < existing_g {
                        g_costs.insert(neighbor_key, new_g);
                        parents.insert(neighbor_key, current_key);
                        let h = self.heuristic_to_targets(&neighbor, &target_states);
                        let f = new_g + h;
                        open_set.push(OpenEntry {
                            f_score: f,
                            g_score: new_g,
                            state: neighbor,
                            counter,
                        });
                        counter += 1;
                    }
                }
            }
        }

        (None, iterations)
    }
}

impl GridRouter {
    /// Octile distance heuristic to nearest target
    #[inline]
    fn heuristic_to_targets(&self, state: &GridState, targets: &[GridState]) -> i32 {
        let mut min_h = i32::MAX;
        for target in targets {
            let dx = (state.gx - target.gx).abs();
            let dy = (state.gy - target.gy).abs();
            let diag = dx.min(dy);
            let orth = (dx - dy).abs();
            let mut h = diag * DIAG_COST + orth * ORTHO_COST;
            if state.layer != target.layer {
                h += self.via_cost;
            }
            min_h = min_h.min(h);
        }
        (min_h as f32 * self.h_weight) as i32
    }

    /// Reconstruct path from parents map
    fn reconstruct_path(
        &self,
        parents: &FxHashMap<u64, u64>,
        goal_key: u64,
        g_costs: &FxHashMap<u64, i32>,
    ) -> Vec<(i32, i32, u8)> {
        let mut path = Vec::new();
        let mut current_key = goal_key;

        loop {
            // Unpack key back to state
            let layer = (current_key & 0xFF) as u8;
            let y = ((current_key >> 8) & 0xFFFFF) as i32;
            let x = ((current_key >> 28) & 0xFFFFF) as i32;
            // Handle negative coordinates (sign extension)
            let x = if x & 0x80000 != 0 { x | !0xFFFFF_i32 } else { x };
            let y = if y & 0x80000 != 0 { y | !0xFFFFF_i32 } else { y };

            path.push((x, y, layer));

            match parents.get(&current_key) {
                Some(&parent_key) => current_key = parent_key,
                None => break,
            }
        }

        path.reverse();
        path
    }

    /// Helper to unpack a key back to coordinates
    #[inline]
    fn unpack_key(key: u64) -> (i32, i32, u8) {
        let layer = (key & 0xFF) as u8;
        let y = ((key >> 8) & 0xFFFFF) as i32;
        let x = ((key >> 28) & 0xFFFFF) as i32;
        let x = if x & 0x80000 != 0 { x | !0xFFFFF_i32 } else { x };
        let y = if y & 0x80000 != 0 { y | !0xFFFFF_i32 } else { y };
        (x, y, layer)
    }

    /// Check if direction (dx, dy) is within ±45° of (base_dx, base_dy)
    /// For grid directions, this means the direction is either the same or one of the two adjacent directions
    #[inline]
    fn is_within_45_degrees(dx: i32, dy: i32, base_dx: i32, base_dy: i32) -> bool {
        // Same direction
        if dx == base_dx && dy == base_dy {
            return true;
        }
        // For each base direction, compute which directions are within 45°
        // The 8 directions in order: E(1,0), NE(1,-1), N(0,-1), NW(-1,-1), W(-1,0), SW(-1,1), S(0,1), SE(1,1)
        // Each direction is 45° apart, so "within 45°" means same or adjacent
        let base_idx = Self::direction_to_index(base_dx, base_dy);
        let test_idx = Self::direction_to_index(dx, dy);

        // Check if indices are adjacent (with wraparound)
        let diff = (test_idx as i32 - base_idx as i32 + 8) % 8;
        diff == 0 || diff == 1 || diff == 7
    }

    /// Convert direction to index (0-7)
    #[inline]
    fn direction_to_index(dx: i32, dy: i32) -> usize {
        match (dx, dy) {
            (1, 0) => 0,    // E
            (1, -1) => 1,   // NE
            (0, -1) => 2,   // N
            (-1, -1) => 3,  // NW
            (-1, 0) => 4,   // W
            (-1, 1) => 5,   // SW
            (0, 1) => 6,    // S
            (1, 1) => 7,    // SE
            _ => 0,
        }
    }

    /// Get via direction constraints for the current position
    /// Returns (required_exact_direction, allowed_45deg_base_direction)
    fn get_via_direction_constraints(
        &self,
        parents: &FxHashMap<u64, u64>,
        current_key: u64,
        current: &GridState,
    ) -> (Option<(i32, i32)>, Option<(i32, i32)>) {
        // Get parent
        let parent_key = match parents.get(&current_key) {
            Some(&k) => k,
            None => return (None, None),
        };
        let (parent_x, parent_y, parent_layer) = Self::unpack_key(parent_key);

        // Check if parent was a via (same position as current, different layer)
        let parent_is_via = parent_x == current.gx && parent_y == current.gy && parent_layer != current.layer;

        if parent_is_via {
            // We just came through a via - need exact same direction as before via
            if let Some(&grandparent_key) = parents.get(&parent_key) {
                let (gp_x, gp_y, _) = Self::unpack_key(grandparent_key);
                let dx = parent_x - gp_x;
                let dy = parent_y - gp_y;
                if dx != 0 || dy != 0 {
                    let norm_dx = if dx != 0 { dx / dx.abs() } else { 0 };
                    let norm_dy = if dy != 0 { dy / dy.abs() } else { 0 };
                    return (Some((norm_dx, norm_dy)), None);
                }
            }
            return (None, None);
        }

        // Check if grandparent was a via (we're one step after via exit)
        let grandparent_key = match parents.get(&parent_key) {
            Some(&k) => k,
            None => return (None, None),
        };
        let (gp_x, gp_y, gp_layer) = Self::unpack_key(grandparent_key);

        let grandparent_is_via = gp_x == parent_x && gp_y == parent_y && gp_layer != parent_layer;

        if grandparent_is_via {
            // We're one step after via exit - need direction within ±45° of pre-via direction
            if let Some(&great_grandparent_key) = parents.get(&grandparent_key) {
                let (ggp_x, ggp_y, _) = Self::unpack_key(great_grandparent_key);
                let dx = gp_x - ggp_x;
                let dy = gp_y - ggp_y;
                if dx != 0 || dy != 0 {
                    let norm_dx = if dx != 0 { dx / dx.abs() } else { 0 };
                    let norm_dy = if dy != 0 { dy / dy.abs() } else { 0 };
                    return (None, Some((norm_dx, norm_dy)));
                }
            }
        }

        (None, None)
    }
}

/// Search state snapshot for visualization
#[pyclass]
#[derive(Clone)]
struct SearchSnapshot {
    #[pyo3(get)]
    iteration: u32,
    #[pyo3(get)]
    current: Option<(i32, i32, u8)>,
    #[pyo3(get)]
    open_count: usize,
    #[pyo3(get)]
    closed_count: usize,
    #[pyo3(get)]
    found: bool,
    #[pyo3(get)]
    path: Option<Vec<(i32, i32, u8)>>,
    /// Closed set cells for visualization (sampled if too large)
    #[pyo3(get)]
    closed_cells: Vec<(i32, i32, u8)>,
    /// Open set cells for visualization (sampled if too large)
    #[pyo3(get)]
    open_cells: Vec<(i32, i32, u8)>,
}

#[pymethods]
impl SearchSnapshot {
    fn __repr__(&self) -> String {
        format!(
            "SearchSnapshot(iter={}, open={}, closed={}, found={})",
            self.iteration, self.open_count, self.closed_count, self.found
        )
    }
}

/// Iterator-based router for visualization
#[pyclass]
struct VisualRouter {
    via_cost: i32,
    h_weight: f32,
    // Search state
    open_set: BinaryHeap<OpenEntry>,
    g_costs: FxHashMap<u64, i32>,
    parents: FxHashMap<u64, u64>,
    closed: FxHashSet<u64>,
    counter: u32,
    iterations: u32,
    max_iterations: u32,
    target_set: FxHashSet<u64>,
    target_states: Vec<GridState>,
    // Result
    found: bool,
    final_path: Option<Vec<(i32, i32, u8)>>,
}

#[pymethods]
impl VisualRouter {
    #[new]
    fn new(via_cost: i32, h_weight: f32) -> Self {
        Self {
            via_cost,
            h_weight,
            open_set: BinaryHeap::new(),
            g_costs: FxHashMap::default(),
            parents: FxHashMap::default(),
            closed: FxHashSet::default(),
            counter: 0,
            iterations: 0,
            max_iterations: 0,
            target_set: FxHashSet::default(),
            target_states: Vec::new(),
            found: false,
            final_path: None,
        }
    }

    /// Initialize the search with sources and targets
    fn init(
        &mut self,
        sources: Vec<(i32, i32, u8)>,
        targets: Vec<(i32, i32, u8)>,
        max_iterations: u32,
    ) {
        // Reset state
        self.open_set.clear();
        self.g_costs.clear();
        self.parents.clear();
        self.closed.clear();
        self.counter = 0;
        self.iterations = 0;
        self.max_iterations = max_iterations;
        self.found = false;
        self.final_path = None;

        // Set up targets
        self.target_set = targets
            .iter()
            .map(|(gx, gy, layer)| GridState::new(*gx, *gy, *layer).as_key())
            .collect();
        self.target_states = targets
            .iter()
            .map(|(gx, gy, layer)| GridState::new(*gx, *gy, *layer))
            .collect();

        // Initialize open set with sources
        for (gx, gy, layer) in sources {
            let state = GridState::new(gx, gy, layer);
            let key = state.as_key();
            let h = self.heuristic_to_targets(&state);
            self.open_set.push(OpenEntry {
                f_score: h,
                g_score: 0,
                state,
                counter: self.counter,
            });
            self.counter += 1;
            self.g_costs.insert(key, 0);
        }
    }

    /// Run N iterations of the search, returns snapshot of current state
    fn step(&mut self, obstacles: &GridObstacleMap, num_iterations: u32) -> SearchSnapshot {
        let mut current_node: Option<GridState> = None;

        for _ in 0..num_iterations {
            if self.found || self.open_set.is_empty() || self.iterations >= self.max_iterations {
                break;
            }

            self.iterations += 1;

            let current_entry = match self.open_set.pop() {
                Some(e) => e,
                None => break,
            };

            let current = current_entry.state;
            let current_key = current.as_key();
            let g = current_entry.g_score;

            if self.closed.contains(&current_key) {
                continue;
            }
            self.closed.insert(current_key);
            current_node = Some(current);

            // Check if reached target
            if self.target_set.contains(&current_key) {
                self.found = true;
                self.final_path = Some(self.reconstruct_path(current_key));
                break;
            }

            // Expand neighbors - 8 directions
            for (dx, dy) in DIRECTIONS {
                let ngx = current.gx + dx;
                let ngy = current.gy + dy;

                if obstacles.is_blocked(ngx, ngy, current.layer as usize) {
                    continue;
                }

                let neighbor = GridState::new(ngx, ngy, current.layer);
                let neighbor_key = neighbor.as_key();

                if self.closed.contains(&neighbor_key) {
                    continue;
                }

                let move_cost = if dx != 0 && dy != 0 { DIAG_COST } else { ORTHO_COST };
                let proximity_cost = obstacles.get_stub_proximity_cost(ngx, ngy);
                let new_g = g + move_cost + proximity_cost;

                let existing_g = self.g_costs.get(&neighbor_key).copied().unwrap_or(i32::MAX);
                if new_g < existing_g {
                    self.g_costs.insert(neighbor_key, new_g);
                    self.parents.insert(neighbor_key, current_key);
                    let h = self.heuristic_to_targets(&neighbor);
                    let f = new_g + h;
                    self.open_set.push(OpenEntry {
                        f_score: f,
                        g_score: new_g,
                        state: neighbor,
                        counter: self.counter,
                    });
                    self.counter += 1;
                }
            }

            // Try via to other layers
            if !obstacles.is_via_blocked(current.gx, current.gy) {
                for layer in 0..obstacles.num_layers as u8 {
                    if layer == current.layer {
                        continue;
                    }

                    // Check if destination layer is blocked at this position
                    if obstacles.is_blocked(current.gx, current.gy, layer as usize) {
                        continue;
                    }

                    let neighbor = GridState::new(current.gx, current.gy, layer);
                    let neighbor_key = neighbor.as_key();

                    if self.closed.contains(&neighbor_key) {
                        continue;
                    }

                    let proximity_cost = obstacles.get_stub_proximity_cost(current.gx, current.gy) * 2;
                    let new_g = g + self.via_cost + proximity_cost;

                    let existing_g = self.g_costs.get(&neighbor_key).copied().unwrap_or(i32::MAX);
                    if new_g < existing_g {
                        self.g_costs.insert(neighbor_key, new_g);
                        self.parents.insert(neighbor_key, current_key);
                        let h = self.heuristic_to_targets(&neighbor);
                        let f = new_g + h;
                        self.open_set.push(OpenEntry {
                            f_score: f,
                            g_score: new_g,
                            state: neighbor,
                            counter: self.counter,
                        });
                        self.counter += 1;
                    }
                }
            }
        }

        // Build snapshot
        self.create_snapshot(current_node)
    }

    /// Check if search is complete (found path or exhausted)
    fn is_done(&self) -> bool {
        self.found || self.open_set.is_empty() || self.iterations >= self.max_iterations
    }

    /// Get the final path if found
    fn get_path(&self) -> Option<Vec<(i32, i32, u8)>> {
        self.final_path.clone()
    }

    /// Get iteration count
    fn get_iterations(&self) -> u32 {
        self.iterations
    }
}

impl VisualRouter {
    fn heuristic_to_targets(&self, state: &GridState) -> i32 {
        let mut min_h = i32::MAX;
        for target in &self.target_states {
            let dx = (state.gx - target.gx).abs();
            let dy = (state.gy - target.gy).abs();
            let diag = dx.min(dy);
            let orth = (dx - dy).abs();
            let mut h = diag * DIAG_COST + orth * ORTHO_COST;
            if state.layer != target.layer {
                h += self.via_cost;
            }
            min_h = min_h.min(h);
        }
        (min_h as f32 * self.h_weight) as i32
    }

    fn reconstruct_path(&self, goal_key: u64) -> Vec<(i32, i32, u8)> {
        let mut path = Vec::new();
        let mut current_key = goal_key;

        loop {
            let layer = (current_key & 0xFF) as u8;
            let y = ((current_key >> 8) & 0xFFFFF) as i32;
            let x = ((current_key >> 28) & 0xFFFFF) as i32;
            let x = if x & 0x80000 != 0 { x | !0xFFFFF_i32 } else { x };
            let y = if y & 0x80000 != 0 { y | !0xFFFFF_i32 } else { y };

            path.push((x, y, layer));

            match self.parents.get(&current_key) {
                Some(&parent_key) => current_key = parent_key,
                None => break,
            }
        }

        path.reverse();
        path
    }

    fn create_snapshot(&self, current: Option<GridState>) -> SearchSnapshot {
        // Sample closed and open sets (limit size for performance)
        const MAX_CELLS: usize = 50000;

        let closed_cells: Vec<(i32, i32, u8)> = self.closed
            .iter()
            .take(MAX_CELLS)
            .map(|&key| {
                let layer = (key & 0xFF) as u8;
                let y = ((key >> 8) & 0xFFFFF) as i32;
                let x = ((key >> 28) & 0xFFFFF) as i32;
                let x = if x & 0x80000 != 0 { x | !0xFFFFF_i32 } else { x };
                let y = if y & 0x80000 != 0 { y | !0xFFFFF_i32 } else { y };
                (x, y, layer)
            })
            .collect();

        // For open set, we need to peek at the heap without consuming
        // Since BinaryHeap doesn't allow efficient iteration, collect g_costs keys that aren't closed
        let open_cells: Vec<(i32, i32, u8)> = self.g_costs
            .keys()
            .filter(|k| !self.closed.contains(k))
            .take(MAX_CELLS)
            .map(|&key| {
                let layer = (key & 0xFF) as u8;
                let y = ((key >> 8) & 0xFFFFF) as i32;
                let x = ((key >> 28) & 0xFFFFF) as i32;
                let x = if x & 0x80000 != 0 { x | !0xFFFFF_i32 } else { x };
                let y = if y & 0x80000 != 0 { y | !0xFFFFF_i32 } else { y };
                (x, y, layer)
            })
            .collect();

        SearchSnapshot {
            iteration: self.iterations,
            current: current.map(|s| (s.gx, s.gy, s.layer)),
            open_count: self.open_set.len(),
            closed_count: self.closed.len(),
            found: self.found,
            path: self.final_path.clone(),
            closed_cells,
            open_cells,
        }
    }
}

/// Differential pair state: (center_x, center_y, layer, orientation)
/// The orientation determines how P and N traces are positioned relative to center:
/// 0 = horizontal (P above, N below) - traces at (x, y+offset) and (x, y-offset)
/// 1 = vertical (P right, N left) - traces at (x+offset, y) and (x-offset, y)
/// 2 = diagonal NE/SW (P at NE) - traces at (x+offset, y+offset) and (x-offset, y-offset)
/// 3 = diagonal NW/SE (P at NW) - traces at (x-offset, y+offset) and (x+offset, y-offset)
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
struct DiffPairState {
    gx: i32,      // Center x coordinate
    gy: i32,      // Center y coordinate
    layer: u8,
    orientation: u8,  // 0=horizontal, 1=vertical, 2=diag_ne, 3=diag_nw
}

impl DiffPairState {
    #[inline]
    fn new(gx: i32, gy: i32, layer: u8, orientation: u8) -> Self {
        Self { gx, gy, layer, orientation }
    }

    #[inline]
    fn as_key(&self) -> u64 {
        // Pack into u64: 18 bits x, 18 bits y, 8 bits layer, 2 bits orientation
        let x = (self.gx as u64) & 0x3FFFF;
        let y = (self.gy as u64) & 0x3FFFF;
        let l = self.layer as u64;
        let o = self.orientation as u64;
        (x << 28) | (y << 10) | (l << 2) | o
    }

    /// Get P trace position (half_spacing is the offset from center)
    #[inline]
    fn p_pos(&self, half_spacing: i32) -> (i32, i32) {
        match self.orientation {
            0 => (self.gx, self.gy + half_spacing),  // Horizontal: P above
            1 => (self.gx + half_spacing, self.gy),  // Vertical: P right
            2 => (self.gx + half_spacing, self.gy + half_spacing),  // Diag NE
            3 => (self.gx - half_spacing, self.gy + half_spacing),  // Diag NW
            _ => (self.gx, self.gy + half_spacing),
        }
    }

    /// Get N trace position (half_spacing is the offset from center)
    #[inline]
    fn n_pos(&self, half_spacing: i32) -> (i32, i32) {
        match self.orientation {
            0 => (self.gx, self.gy - half_spacing),  // Horizontal: N below
            1 => (self.gx - half_spacing, self.gy),  // Vertical: N left
            2 => (self.gx - half_spacing, self.gy - half_spacing),  // Diag SW
            3 => (self.gx + half_spacing, self.gy - half_spacing),  // Diag SE
            _ => (self.gx, self.gy - half_spacing),
        }
    }
}

/// A* open set entry for differential pair routing
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct DiffPairOpenEntry {
    f_score: i32,
    g_score: i32,
    state: DiffPairState,
    counter: u32,
}

impl Ord for DiffPairOpenEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        other.f_score.cmp(&self.f_score)
            .then_with(|| other.counter.cmp(&self.counter))
    }
}

impl PartialOrd for DiffPairOpenEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Differential Pair Grid A* Router
/// Routes P and N traces together as a coupled pair maintaining constant spacing
#[pyclass]
struct DiffPairRouter {
    via_cost: i32,
    h_weight: f32,
    half_spacing: i32,  // Grid units: half of center-to-center spacing
}

#[pymethods]
impl DiffPairRouter {
    /// Create a new differential pair router
    /// half_spacing_grid: Half of the center-to-center spacing in grid units
    #[new]
    fn new(via_cost: i32, h_weight: f32, half_spacing_grid: i32) -> Self {
        Self {
            via_cost,
            h_weight,
            half_spacing: half_spacing_grid,
        }
    }

    /// Route a differential pair from sources to targets.
    /// Sources and targets are (p_gx, p_gy, n_gx, n_gy, layer) tuples.
    /// Returns (p_path, n_path, iterations) where paths are lists of (gx, gy, layer) tuples,
    /// or (None, None, iterations) if no path found.
    fn route_diff_pair(
        &self,
        obstacles: &GridObstacleMap,
        sources: Vec<(i32, i32, i32, i32, u8)>,  // (p_gx, p_gy, n_gx, n_gy, layer)
        targets: Vec<(i32, i32, i32, i32, u8)>,  // (p_gx, p_gy, n_gx, n_gy, layer)
        max_iterations: u32,
    ) -> (Option<Vec<(i32, i32, u8)>>, Option<Vec<(i32, i32, u8)>>, u32) {
        // Convert source/target positions to DiffPairState using configured half_spacing
        // The actual P/N positions from stubs may have different spacing than our route
        let source_states: Vec<DiffPairState> = sources.iter()
            .filter_map(|(p_gx, p_gy, n_gx, n_gy, layer)| {
                // Calculate center and determine orientation
                let center_gx = (p_gx + n_gx) / 2;
                let center_gy = (p_gy + n_gy) / 2;
                let dx = p_gx - n_gx;
                let dy = p_gy - n_gy;

                let orientation = if dx.abs() > dy.abs() {
                    1  // Vertical orientation (P/N side by side horizontally)
                } else {
                    0  // Horizontal orientation (P/N stacked vertically)
                };

                Some(DiffPairState::new(center_gx, center_gy, *layer, orientation))
            })
            .collect();

        let target_states: Vec<DiffPairState> = targets.iter()
            .filter_map(|(p_gx, p_gy, n_gx, n_gy, layer)| {
                let center_gx = (p_gx + n_gx) / 2;
                let center_gy = (p_gy + n_gy) / 2;
                let dx = p_gx - n_gx;
                let dy = p_gy - n_gy;

                let orientation = if dx.abs() > dy.abs() {
                    1  // Vertical orientation
                } else {
                    0  // Horizontal orientation
                };

                Some(DiffPairState::new(center_gx, center_gy, *layer, orientation))
            })
            .collect();

        if source_states.is_empty() || target_states.is_empty() {
            return (None, None, 0);
        }

        // Use proximity-based target matching instead of exact match
        // A target is reached if we're within tolerance of any target center
        let target_tolerance = 5;  // Grid units

        // Initialize A* search
        let mut open_set = BinaryHeap::new();
        let mut g_costs: FxHashMap<u64, i32> = FxHashMap::default();
        let mut parents: FxHashMap<u64, u64> = FxHashMap::default();
        let mut closed: FxHashSet<u64> = FxHashSet::default();
        let mut counter: u32 = 0;

        for state in &source_states {
            let key = state.as_key();
            let h = self.heuristic_to_targets(state, &target_states);
            open_set.push(DiffPairOpenEntry {
                f_score: h,
                g_score: 0,
                state: *state,
                counter,
            });
            counter += 1;
            g_costs.insert(key, 0);
        }

        let mut iterations: u32 = 0;

        while let Some(current_entry) = open_set.pop() {
            if iterations >= max_iterations {
                break;
            }
            iterations += 1;

            let current = current_entry.state;
            let current_key = current.as_key();
            let g = current_entry.g_score;

            if closed.contains(&current_key) {
                continue;
            }
            closed.insert(current_key);

            // Check if reached target (within tolerance)
            let mut reached_target = false;
            for target in &target_states {
                if current.layer == target.layer {
                    let dx = (current.gx - target.gx).abs();
                    let dy = (current.gy - target.gy).abs();
                    if dx <= target_tolerance && dy <= target_tolerance {
                        reached_target = true;
                        break;
                    }
                }
            }
            if reached_target {
                let (p_path, n_path) = self.reconstruct_diff_pair_path(&parents, current_key);
                return (Some(p_path), Some(n_path), iterations);
            }

            // Expand neighbors - move both traces in same direction
            for (dx, dy) in DIRECTIONS {
                // Calculate new center position
                let new_gx = current.gx + dx;
                let new_gy = current.gy + dy;

                // Determine appropriate orientation for this move direction
                let new_orientations = self.get_valid_orientations(dx, dy, current.orientation);

                for new_orientation in new_orientations {
                    let neighbor = DiffPairState::new(new_gx, new_gy, current.layer, new_orientation);

                    // Check if both P and N positions are valid
                    let (p_x, p_y) = neighbor.p_pos(self.half_spacing);
                    let (n_x, n_y) = neighbor.n_pos(self.half_spacing);

                    if obstacles.is_blocked(p_x, p_y, neighbor.layer as usize) ||
                       obstacles.is_blocked(n_x, n_y, neighbor.layer as usize) {
                        continue;
                    }

                    let neighbor_key = neighbor.as_key();
                    if closed.contains(&neighbor_key) {
                        continue;
                    }

                    // Cost includes both traces moving
                    let move_cost = if dx != 0 && dy != 0 { DIAG_COST } else { ORTHO_COST };
                    // Add orientation change cost if orientation changed
                    let orientation_cost = if new_orientation != current.orientation { 500 } else { 0 };
                    let proximity_cost = obstacles.get_stub_proximity_cost(p_x, p_y)
                        + obstacles.get_stub_proximity_cost(n_x, n_y);
                    let new_g = g + move_cost + orientation_cost + proximity_cost;

                    let existing_g = g_costs.get(&neighbor_key).copied().unwrap_or(i32::MAX);
                    if new_g < existing_g {
                        g_costs.insert(neighbor_key, new_g);
                        parents.insert(neighbor_key, current_key);
                        let h = self.heuristic_to_targets(&neighbor, &target_states);
                        let f = new_g + h;
                        open_set.push(DiffPairOpenEntry {
                            f_score: f,
                            g_score: new_g,
                            state: neighbor,
                            counter,
                        });
                        counter += 1;
                    }
                }
            }

            // Try via to other layers (both traces need via)
            let (p_x, p_y) = current.p_pos(self.half_spacing);
            let (n_x, n_y) = current.n_pos(self.half_spacing);

            if !obstacles.is_via_blocked(p_x, p_y) && !obstacles.is_via_blocked(n_x, n_y) {
                for layer in 0..obstacles.num_layers as u8 {
                    if layer == current.layer {
                        continue;
                    }

                    // Check both positions on destination layer
                    if obstacles.is_blocked(p_x, p_y, layer as usize) ||
                       obstacles.is_blocked(n_x, n_y, layer as usize) {
                        continue;
                    }

                    let neighbor = DiffPairState::new(current.gx, current.gy, layer, current.orientation);
                    let neighbor_key = neighbor.as_key();

                    if closed.contains(&neighbor_key) {
                        continue;
                    }

                    // Via cost is doubled because we need two vias
                    let proximity_cost = (obstacles.get_stub_proximity_cost(p_x, p_y)
                        + obstacles.get_stub_proximity_cost(n_x, n_y)) * 2;
                    let new_g = g + self.via_cost * 2 + proximity_cost;

                    let existing_g = g_costs.get(&neighbor_key).copied().unwrap_or(i32::MAX);
                    if new_g < existing_g {
                        g_costs.insert(neighbor_key, new_g);
                        parents.insert(neighbor_key, current_key);
                        let h = self.heuristic_to_targets(&neighbor, &target_states);
                        let f = new_g + h;
                        open_set.push(DiffPairOpenEntry {
                            f_score: f,
                            g_score: new_g,
                            state: neighbor,
                            counter,
                        });
                        counter += 1;
                    }
                }
            }
        }

        (None, None, iterations)
    }
}

impl DiffPairRouter {
    /// Convert P and N positions to a DiffPairState with appropriate orientation
    fn positions_to_state(&self, p_gx: i32, p_gy: i32, n_gx: i32, n_gy: i32, layer: u8) -> Option<DiffPairState> {
        let dx = p_gx - n_gx;
        let dy = p_gy - n_gy;

        // Determine orientation based on relative positions
        let orientation = if dx == 0 && dy != 0 {
            // P and N aligned vertically - horizontal orientation (P above if dy > 0)
            0
        } else if dy == 0 && dx != 0 {
            // P and N aligned horizontally - vertical orientation (P right if dx > 0)
            1
        } else if dx > 0 && dy > 0 {
            // P is NE of N
            2
        } else if dx < 0 && dy > 0 {
            // P is NW of N
            3
        } else if dx > 0 && dy < 0 {
            // P is SE of N - same as NW with swapped roles
            3
        } else if dx < 0 && dy < 0 {
            // P is SW of N - same as NE with swapped roles
            2
        } else {
            // P and N at same position - invalid
            return None;
        };

        // Calculate center position
        let center_gx = (p_gx + n_gx) / 2;
        let center_gy = (p_gy + n_gy) / 2;

        Some(DiffPairState::new(center_gx, center_gy, layer, orientation))
    }

    /// Get valid orientations for a given move direction
    /// Returns orientations that make sense for the direction of travel
    fn get_valid_orientations(&self, dx: i32, dy: i32, current_orientation: u8) -> Vec<u8> {
        // For differential pairs, the orientation should generally be perpendicular to travel direction
        // to maintain proper spacing. However, we allow some flexibility.
        match (dx, dy) {
            (1, 0) | (-1, 0) => vec![0, 2, 3],  // Moving horizontally: prefer vertical trace arrangement
            (0, 1) | (0, -1) => vec![1, 2, 3],  // Moving vertically: prefer horizontal trace arrangement
            (1, 1) | (-1, -1) => vec![3, 0, 1], // Diagonal NE/SW: prefer NW/SE orientation
            (1, -1) | (-1, 1) => vec![2, 0, 1], // Diagonal NW/SE: prefer NE/SW orientation
            _ => vec![current_orientation],
        }
    }

    /// Heuristic to nearest target
    fn heuristic_to_targets(&self, state: &DiffPairState, targets: &[DiffPairState]) -> i32 {
        let mut min_h = i32::MAX;
        for target in targets {
            let dx = (state.gx - target.gx).abs();
            let dy = (state.gy - target.gy).abs();
            let diag = dx.min(dy);
            let orth = (dx - dy).abs();
            let mut h = diag * DIAG_COST + orth * ORTHO_COST;
            if state.layer != target.layer {
                h += self.via_cost * 2;  // Two vias needed for layer change
            }
            min_h = min_h.min(h);
        }
        (min_h as f32 * self.h_weight) as i32
    }

    /// Reconstruct P and N paths from the parent map
    fn reconstruct_diff_pair_path(&self, parents: &FxHashMap<u64, u64>, goal_key: u64) -> (Vec<(i32, i32, u8)>, Vec<(i32, i32, u8)>) {
        let mut p_path = Vec::new();
        let mut n_path = Vec::new();
        let mut current_key = goal_key;

        loop {
            // Unpack key back to state
            let o = (current_key & 0x3) as u8;
            let l = ((current_key >> 2) & 0xFF) as u8;
            let y = ((current_key >> 10) & 0x3FFFF) as i32;
            let x = ((current_key >> 28) & 0x3FFFF) as i32;
            // Handle negative coordinates (sign extension)
            let x = if x & 0x20000 != 0 { x | !0x3FFFF_i32 } else { x };
            let y = if y & 0x20000 != 0 { y | !0x3FFFF_i32 } else { y };

            let state = DiffPairState::new(x, y, l, o);
            let (p_x, p_y) = state.p_pos(self.half_spacing);
            let (n_x, n_y) = state.n_pos(self.half_spacing);

            p_path.push((p_x, p_y, l));
            n_path.push((n_x, n_y, l));

            match parents.get(&current_key) {
                Some(&parent_key) => current_key = parent_key,
                None => break,
            }
        }

        p_path.reverse();
        n_path.reverse();
        (p_path, n_path)
    }
}

/// Module version
const VERSION: &str = "0.4.0";

/// Python module
#[pymodule]
fn grid_router(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add("__version__", VERSION)?;
    m.add_class::<GridObstacleMap>()?;
    m.add_class::<GridRouter>()?;
    m.add_class::<VisualRouter>()?;
    m.add_class::<SearchSnapshot>()?;
    m.add_class::<DiffPairRouter>()?;
    Ok(())
}
