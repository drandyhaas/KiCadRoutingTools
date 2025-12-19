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
    /// via_exclusion_radius: Grid cells to exclude around placed vias during search.
    /// This prevents the route from passing too close to its own vias, which is
    /// important for diff pair routing where P/N tracks are offset from centerline.
    /// start_direction: Optional (dx, dy) direction for initial moves from source.
    /// If specified, first N moves must be within ±45° of this direction.
    /// end_direction: Optional (dx, dy) continuous direction for final moves to target.
    /// If specified, checks arrival direction is within ±60° of this direction.
    /// direction_steps: Number of steps to constrain at start (default 2).
    #[pyo3(signature = (obstacles, sources, targets, max_iterations, collinear_vias=false, via_exclusion_radius=0, start_direction=None, end_direction=None, direction_steps=2))]
    fn route_multi(
        &self,
        obstacles: &GridObstacleMap,
        sources: Vec<(i32, i32, u8)>,
        targets: Vec<(i32, i32, u8)>,
        max_iterations: u32,
        collinear_vias: bool,
        via_exclusion_radius: i32,
        start_direction: Option<(i32, i32)>,
        end_direction: Option<(f64, f64)>,
        direction_steps: i32,
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

        // Track via positions for each node's path (only used if via_exclusion_radius > 0)
        // Maps node key -> list of (gx, gy) via positions on path to that node
        let mut path_vias: FxHashMap<u64, Vec<(i32, i32)>> = FxHashMap::default();

        // Track steps from source for direction constraint
        // Maps node key -> steps from nearest source
        let mut steps_from_source: FxHashMap<u64, i32> = FxHashMap::default();

        // Normalize start direction if provided
        let norm_start_dir: Option<(i32, i32)> = start_direction.map(|(dx, dy)| {
            let ndx = if dx != 0 { dx / dx.abs() } else { 0 };
            let ndy = if dy != 0 { dy / dy.abs() } else { 0 };
            (ndx, ndy)
        });

        // Normalize end direction if provided (direction we want to arrive FROM)
        // This is a continuous (f64, f64) unit vector
        let norm_end_dir: Option<(f64, f64)> = end_direction.map(|(dx, dy)| {
            let len = (dx * dx + dy * dy).sqrt();
            if len > 0.0 { (dx / len, dy / len) } else { (0.0, 0.0) }
        });

        // Helper: check if position (nx, ny) would violate via exclusion
        // Returns true if blocked (too close to a via and not moving away from it)
        let check_via_exclusion = |nx: i32, ny: i32, current_gx: i32, current_gy: i32, vias: &[(i32, i32)], radius: i32| -> bool {
            if radius <= 0 {
                return false;
            }
            for &(vx, vy) in vias {
                let dist_to_neighbor = (nx - vx).abs().max((ny - vy).abs());
                let dist_to_current = (current_gx - vx).abs().max((current_gy - vy).abs());
                // If we're outside the radius, block re-entry
                if dist_to_neighbor <= radius && dist_to_current > radius {
                    return true;
                }
                // If we're inside the radius, only allow moves that increase distance from via
                // This prevents perpendicular drift within the exclusion zone
                if dist_to_current <= radius && dist_to_neighbor <= dist_to_current {
                    // Allow only if we're moving directly away from via (increasing distance)
                    // or staying at same distance in the escape direction
                    if dist_to_neighbor < dist_to_current {
                        // Moving closer to via - block
                        return true;
                    }
                    // Same distance - check if perpendicular movement
                    // Block perpendicular moves within the exclusion zone
                    if dist_to_neighbor == dist_to_current && dist_to_current > 0 {
                        // Check if this is a perpendicular move by seeing if both x and y changed
                        let dx_from_via = (nx - vx).abs() - (current_gx - vx).abs();
                        let dy_from_via = (ny - vy).abs() - (current_gy - vy).abs();
                        // If moving perpendicular (one axis increases while other decreases)
                        // and still within half the radius, block
                        if (dx_from_via > 0 && dy_from_via < 0) || (dx_from_via < 0 && dy_from_via > 0) {
                            if dist_to_current <= radius / 2 {
                                return true;
                            }
                        }
                    }
                }
            }
            false
        };


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
            // Source nodes start with no vias on their path
            if via_exclusion_radius > 0 {
                path_vias.insert(key, Vec::new());
            }
            // Source nodes are at step 0
            steps_from_source.insert(key, 0);
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

            // Check if reached target
            if target_set.contains(&current_key) {
                // If end_direction is specified, verify we arrived from a compatible direction
                let arrival_ok = if let Some((end_dx, end_dy)) = norm_end_dir {
                    if let Some(&parent_key) = parents.get(&current_key) {
                        let (px, py, _) = Self::unpack_key(parent_key);
                        let arrive_dx = (current.gx - px) as f64;
                        let arrive_dy = (current.gy - py) as f64;
                        let arrive_len = (arrive_dx * arrive_dx + arrive_dy * arrive_dy).sqrt();
                        if arrive_len > 0.0 {
                            // Normalize arrival direction and compute dot product
                            let norm_arrive_dx = arrive_dx / arrive_len;
                            let norm_arrive_dy = arrive_dy / arrive_len;
                            let dot = norm_arrive_dx * end_dx + norm_arrive_dy * end_dy;
                            // Check if arrival direction is within ±120° of required end direction
                            // cos(120°) = -0.5, so dot product must be >= -0.5
                            dot >= -0.5
                        } else {
                            true // Same position as parent (via), allow it
                        }
                    } else {
                        true // No parent (source is target), allow it
                    }
                } else {
                    true // No end direction constraint
                };

                if arrival_ok {
                    // Reconstruct path
                    let path = self.reconstruct_path(&parents, current_key, &g_costs);
                    return (Some(path), iterations);
                }
                // Arrival direction not ok - DON'T add to closed, allow reaching from different direction
                continue;
            }

            closed.insert(current_key);

            // Check via constraints for diff pair routing (collinear_vias mode):
            // 1. If we just came through a via: must continue in exact same direction as before via
            // 2. If we're one step after a via exit: must be within ±45° of the pre-via direction
            // This ensures clean via geometry for differential pairs
            let (required_direction, allowed_45deg_from): (Option<(i32, i32)>, Option<(i32, i32)>) = if collinear_vias {
                self.get_via_direction_constraints(&parents, current_key, &current)
            } else {
                (None, None)
            };

            // Get current node's via list for exclusion checking
            let current_vias: Vec<(i32, i32)> = if via_exclusion_radius > 0 {
                path_vias.get(&current_key).cloned().unwrap_or_default()
            } else {
                Vec::new()
            };

            // Get steps from source for direction constraint
            let current_steps = steps_from_source.get(&current_key).copied().unwrap_or(i32::MAX);

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

                // Check start direction constraint for first N steps
                if let Some((start_dx, start_dy)) = norm_start_dir {
                    if current_steps < direction_steps {
                        if !Self::is_within_45_degrees(dx, dy, start_dx, start_dy) {
                            continue;
                        }
                    }
                }

                let ngx = current.gx + dx;
                let ngy = current.gy + dy;

                if obstacles.is_blocked(ngx, ngy, current.layer as usize) {
                    continue;
                }

                // Check via exclusion - can't approach our own vias once we've moved away
                if check_via_exclusion(ngx, ngy, current.gx, current.gy, &current_vias, via_exclusion_radius) {
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
                    // Propagate via list to neighbor (same vias since no layer change)
                    if via_exclusion_radius > 0 {
                        path_vias.insert(neighbor_key, current_vias.clone());
                    }
                    // Update steps from source for neighbor
                    steps_from_source.insert(neighbor_key, current_steps + 1);
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
            // For collinear_vias mode, enforce symmetric constraint around via:
            // ±45° -> direction D -> VIA -> direction D -> ±45°
            // This requires at least 2 steps before via (great-grandparent must exist),
            // and direction into via must be within ±45° of previous direction
            let can_place_via = if collinear_vias {
                if let Some(&parent_key) = parents.get(&current_key) {
                    if let Some(&grandparent_key) = parents.get(&parent_key) {
                        // Need great-grandparent to ensure 2 full steps before via
                        if !parents.contains_key(&grandparent_key) {
                            false // Only 1 step before via - need at least 2
                        } else {
                            // Have great-grandparent - check that approach direction is within ±45° of previous
                            let (parent_x, parent_y, _) = Self::unpack_key(parent_key);
                            let (gp_x, gp_y, _) = Self::unpack_key(grandparent_key);

                            // Direction from grandparent to parent (previous direction)
                            let prev_dx = parent_x - gp_x;
                            let prev_dy = parent_y - gp_y;

                            // Direction from parent to current (approach direction)
                            let approach_dx = current.gx - parent_x;
                            let approach_dy = current.gy - parent_y;

                            if (prev_dx != 0 || prev_dy != 0) && (approach_dx != 0 || approach_dy != 0) {
                                let norm_prev_dx = if prev_dx != 0 { prev_dx / prev_dx.abs() } else { 0 };
                                let norm_prev_dy = if prev_dy != 0 { prev_dy / prev_dy.abs() } else { 0 };
                                let norm_approach_dx = if approach_dx != 0 { approach_dx / approach_dx.abs() } else { 0 };
                                let norm_approach_dy = if approach_dy != 0 { approach_dy / approach_dy.abs() } else { 0 };

                                // Approach must be same or within ±45° of previous
                                Self::is_within_45_degrees(norm_approach_dx, norm_approach_dy, norm_prev_dx, norm_prev_dy)
                            } else {
                                false
                            }
                        }
                    } else {
                        false // No grandparent - too close to start
                    }
                } else {
                    false // No parent at all - this is a source node
                }
            } else {
                true
            };

            // Check if this via would be too close to existing vias in the path
            let via_too_close = if via_exclusion_radius > 0 {
                current_vias.iter().any(|&(vx, vy)| {
                    let dist = (current.gx - vx).abs().max((current.gy - vy).abs());
                    dist <= via_exclusion_radius * 2  // Need 2x radius for via-via clearance
                })
            } else {
                false
            };

            if can_place_via && !via_too_close && !obstacles.is_via_blocked(current.gx, current.gy) {
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
                        // Add this via position to the path's via list
                        if via_exclusion_radius > 0 {
                            let mut new_vias = current_vias.clone();
                            new_vias.push((current.gx, current.gy));
                            path_vias.insert(neighbor_key, new_vias);
                        }
                        // Via doesn't count as a step for direction constraint
                        steps_from_source.insert(neighbor_key, current_steps);
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
        _g_costs: &FxHashMap<u64, i32>,
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
        // Still require exact same direction for symmetry when path is reversed
        let grandparent_key = match parents.get(&parent_key) {
            Some(&k) => k,
            None => return (None, None),
        };
        let (gp_x, gp_y, gp_layer) = Self::unpack_key(grandparent_key);

        let grandparent_is_via = gp_x == parent_x && gp_y == parent_y && gp_layer != parent_layer;

        if grandparent_is_via {
            // We're one step after via exit - still require exact same direction
            // (This ensures 2 straight steps after via for symmetric geometry when reversed)
            if let Some(&great_grandparent_key) = parents.get(&grandparent_key) {
                let (ggp_x, ggp_y, _) = Self::unpack_key(great_grandparent_key);
                let dx = gp_x - ggp_x;
                let dy = gp_y - ggp_y;
                if dx != 0 || dy != 0 {
                    let norm_dx = if dx != 0 { dx / dx.abs() } else { 0 };
                    let norm_dy = if dy != 0 { dy / dy.abs() } else { 0 };
                    return (Some((norm_dx, norm_dy)), None);
                }
            }
            return (None, None);
        }

        // Check if great-grandparent was a via (we're two steps after via exit)
        let great_grandparent_key = match parents.get(&grandparent_key) {
            Some(&k) => k,
            None => return (None, None),
        };
        let (ggp_x, ggp_y, ggp_layer) = Self::unpack_key(great_grandparent_key);

        let great_grandparent_is_via = ggp_x == gp_x && ggp_y == gp_y && ggp_layer != gp_layer;

        if great_grandparent_is_via {
            // We're two steps after via exit - allow ±45° turn
            if let Some(&gggp_key) = parents.get(&great_grandparent_key) {
                let (gggp_x, gggp_y, _) = Self::unpack_key(gggp_key);
                let dx = ggp_x - gggp_x;
                let dy = ggp_y - gggp_y;
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

// =============================================================================
// Pose-based A* Router with Dubins Heuristic
// =============================================================================
// State space: (x, y, theta_idx, layer) where theta_idx is 0-7 for 8 directions
// Uses Dubins path length as heuristic for better orientation-aware routing.

/// Pose state: (x, y, theta_idx, layer) for orientation-aware routing
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
struct PoseState {
    gx: i32,
    gy: i32,
    theta_idx: u8,  // 0-7, corresponding to DIRECTIONS indices
    layer: u8,
}

impl PoseState {
    #[inline]
    fn new(gx: i32, gy: i32, theta_idx: u8, layer: u8) -> Self {
        Self { gx, gy, theta_idx, layer }
    }

    #[inline]
    fn as_key(&self) -> u64 {
        // Pack into u64: 19 bits x, 19 bits y, 3 bits theta, 8 bits layer
        // This allows x,y in range [-262144, 262143]
        let x = (self.gx as u64) & 0x7FFFF;
        let y = (self.gy as u64) & 0x7FFFF;
        let t = (self.theta_idx as u64) & 0x7;
        let l = self.layer as u64;
        (x << 30) | (y << 11) | (t << 8) | l
    }

    /// Get the direction vector for this pose's heading
    #[inline]
    fn direction(&self) -> (i32, i32) {
        DIRECTIONS[self.theta_idx as usize]
    }

    /// Get theta in radians
    #[inline]
    fn theta_radians(&self) -> f64 {
        (self.theta_idx as f64) * std::f64::consts::FRAC_PI_4
    }
}

/// A* open set entry for pose-based search
#[derive(Clone, Copy, Debug)]
struct PoseOpenEntry {
    f_score: i32,
    g_score: i32,
    state: PoseState,
    counter: u32,
}

impl Eq for PoseOpenEntry {}

impl PartialEq for PoseOpenEntry {
    fn eq(&self, other: &Self) -> bool {
        self.f_score == other.f_score && self.counter == other.counter
    }
}

impl Ord for PoseOpenEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        // Reverse ordering for min-heap (lowest f_score first)
        other.f_score.cmp(&self.f_score)
            .then_with(|| other.counter.cmp(&self.counter))
    }
}

impl PartialOrd for PoseOpenEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Dubins path calculator for heuristic
struct DubinsCalculator {
    /// Minimum turning radius in grid units
    min_radius: f64,
}

impl DubinsCalculator {
    fn new(min_radius: f64) -> Self {
        Self { min_radius: min_radius.max(0.1) }
    }

    /// Calculate shortest Dubins path length between two poses
    /// Returns path length in grid units (scaled by 1000 for integer math)
    fn path_length(&self,
                   x1: f64, y1: f64, theta1: f64,
                   x2: f64, y2: f64, theta2: f64) -> i32 {
        let r = self.min_radius;

        // Vector from start to goal
        let dx = x2 - x1;
        let dy = y2 - y1;
        let d = (dx * dx + dy * dy).sqrt();

        // If very close, just return distance
        if d < 0.001 {
            // Just need to turn in place - approximate as arc
            let dtheta = Self::normalize_angle(theta2 - theta1).abs();
            return (dtheta * r * 1000.0) as i32;
        }

        // Normalize by turning radius
        let d_norm = d / r;

        // Angle from start to goal
        let phi = dy.atan2(dx);

        // Relative angles
        let alpha = Self::normalize_angle(theta1 - phi);
        let beta = Self::normalize_angle(theta2 - phi);

        // Try all 6 Dubins path types and return shortest
        let mut min_len = f64::MAX;

        // CSC paths (Circle-Straight-Circle)
        if let Some(len) = self.lsl_length(d_norm, alpha, beta) {
            min_len = min_len.min(len);
        }
        if let Some(len) = self.rsr_length(d_norm, alpha, beta) {
            min_len = min_len.min(len);
        }
        if let Some(len) = self.lsr_length(d_norm, alpha, beta) {
            min_len = min_len.min(len);
        }
        if let Some(len) = self.rsl_length(d_norm, alpha, beta) {
            min_len = min_len.min(len);
        }

        // CCC paths (Circle-Circle-Circle)
        if let Some(len) = self.rlr_length(d_norm, alpha, beta) {
            min_len = min_len.min(len);
        }
        if let Some(len) = self.lrl_length(d_norm, alpha, beta) {
            min_len = min_len.min(len);
        }

        // Fallback: if no valid path (shouldn't happen), use straight line + turn estimate
        if min_len == f64::MAX {
            let dtheta = Self::normalize_angle(theta2 - theta1).abs();
            min_len = d_norm + dtheta;
        }

        // Scale back by radius and convert to integer (x1000)
        (min_len * r * 1000.0) as i32
    }

    #[inline]
    fn normalize_angle(a: f64) -> f64 {
        let mut a = a % (2.0 * std::f64::consts::PI);
        if a > std::f64::consts::PI {
            a -= 2.0 * std::f64::consts::PI;
        } else if a < -std::f64::consts::PI {
            a += 2.0 * std::f64::consts::PI;
        }
        a
    }

    #[inline]
    fn mod2pi(a: f64) -> f64 {
        let mut a = a % (2.0 * std::f64::consts::PI);
        if a < 0.0 {
            a += 2.0 * std::f64::consts::PI;
        }
        a
    }

    /// LSL path: Left turn, Straight, Left turn
    fn lsl_length(&self, d: f64, alpha: f64, beta: f64) -> Option<f64> {
        let ca = alpha.cos();
        let sa = alpha.sin();
        let cb = beta.cos();
        let sb = beta.sin();

        let tmp = 2.0 + d * d - 2.0 * (ca * cb + sa * sb - d * (sa - sb));
        if tmp < 0.0 {
            return None;
        }
        let p = tmp.sqrt();
        let theta = (cb - ca).atan2(d + sa - sb);
        let t = Self::mod2pi(-alpha + theta);
        let q = Self::mod2pi(beta - theta);

        Some(t + p + q)
    }

    /// RSR path: Right turn, Straight, Right turn
    fn rsr_length(&self, d: f64, alpha: f64, beta: f64) -> Option<f64> {
        let ca = alpha.cos();
        let sa = alpha.sin();
        let cb = beta.cos();
        let sb = beta.sin();

        let tmp = 2.0 + d * d - 2.0 * (ca * cb + sa * sb - d * (sb - sa));
        if tmp < 0.0 {
            return None;
        }
        let p = tmp.sqrt();
        let theta = (ca - cb).atan2(d - sa + sb);
        let t = Self::mod2pi(alpha - theta);
        let q = Self::mod2pi(-beta + theta);

        Some(t + p + q)
    }

    /// LSR path: Left turn, Straight, Right turn
    fn lsr_length(&self, d: f64, alpha: f64, beta: f64) -> Option<f64> {
        let ca = alpha.cos();
        let sa = alpha.sin();
        let cb = beta.cos();
        let sb = beta.sin();

        let tmp = -2.0 + d * d + 2.0 * (ca * cb + sa * sb + d * (sa + sb));
        if tmp < 0.0 {
            return None;
        }
        let p = tmp.sqrt();
        let theta = (-ca - cb).atan2(d + sa + sb) - (-2.0_f64).atan2(p);
        let t = Self::mod2pi(-alpha + theta);
        let q = Self::mod2pi(-beta + theta);

        Some(t + p + q)
    }

    /// RSL path: Right turn, Straight, Left turn
    fn rsl_length(&self, d: f64, alpha: f64, beta: f64) -> Option<f64> {
        let ca = alpha.cos();
        let sa = alpha.sin();
        let cb = beta.cos();
        let sb = beta.sin();

        let tmp = -2.0 + d * d + 2.0 * (ca * cb + sa * sb - d * (sa + sb));
        if tmp < 0.0 {
            return None;
        }
        let p = tmp.sqrt();
        let theta = (ca + cb).atan2(d - sa - sb) - (2.0_f64).atan2(p);
        let t = Self::mod2pi(alpha - theta);
        let q = Self::mod2pi(beta - theta);

        Some(t + p + q)
    }

    /// RLR path: Right turn, Left turn, Right turn
    fn rlr_length(&self, d: f64, alpha: f64, beta: f64) -> Option<f64> {
        let ca = alpha.cos();
        let sa = alpha.sin();
        let cb = beta.cos();
        let sb = beta.sin();

        let tmp = (6.0 - d * d + 2.0 * (ca * cb + sa * sb + d * (sa - sb))) / 8.0;
        if tmp.abs() > 1.0 {
            return None;
        }
        let p = Self::mod2pi(2.0 * std::f64::consts::PI - tmp.acos());
        let theta = (ca - cb).atan2(d - sa + sb);
        let t = Self::mod2pi(alpha - theta + p / 2.0);
        let q = Self::mod2pi(alpha - beta - t + p);

        Some(t + p + q)
    }

    /// LRL path: Left turn, Right turn, Left turn
    fn lrl_length(&self, d: f64, alpha: f64, beta: f64) -> Option<f64> {
        let ca = alpha.cos();
        let sa = alpha.sin();
        let cb = beta.cos();
        let sb = beta.sin();

        let tmp = (6.0 - d * d + 2.0 * (ca * cb + sa * sb - d * (sa - sb))) / 8.0;
        if tmp.abs() > 1.0 {
            return None;
        }
        let p = Self::mod2pi(2.0 * std::f64::consts::PI - tmp.acos());
        let theta = (cb - ca).atan2(d + sa - sb);
        let t = Self::mod2pi(-alpha + theta + p / 2.0);
        let q = Self::mod2pi(beta - alpha - t + p);

        Some(t + p + q)
    }
}

/// Pose-based A* Router with Dubins heuristic
#[pyclass]
struct PoseRouter {
    via_cost: i32,
    h_weight: f32,
    turn_cost: i32,  // Cost per 45° turn
    min_radius_grid: f64,  // Minimum turning radius in grid units
}

#[pymethods]
impl PoseRouter {
    #[new]
    #[pyo3(signature = (via_cost, h_weight, turn_cost, min_radius_grid))]
    fn new(via_cost: i32, h_weight: f32, turn_cost: i32, min_radius_grid: f64) -> Self {
        Self { via_cost, h_weight, turn_cost, min_radius_grid }
    }

    /// Route from source pose to target pose using pose-based A* with Dubins heuristic.
    ///
    /// Args:
    ///     obstacles: The obstacle map
    ///     src_x, src_y, src_layer: Source position
    ///     src_theta_idx: Source heading (0-7, index into DIRECTIONS)
    ///     tgt_x, tgt_y, tgt_layer: Target position
    ///     tgt_theta_idx: Target heading (0-7, index into DIRECTIONS)
    ///     max_iterations: Maximum A* iterations
    ///     diff_pair_via_spacing: Optional spacing in grid units for P/N via offset check.
    ///         If provided, via placement checks that both +offset and -offset positions
    ///         perpendicular to the heading are clear.
    ///
    /// Returns:
    ///     (path, iterations) where path is list of (gx, gy, theta_idx, layer) or None
    #[pyo3(signature = (obstacles, src_x, src_y, src_layer, src_theta_idx, tgt_x, tgt_y, tgt_layer, tgt_theta_idx, max_iterations, diff_pair_via_spacing=None))]
    fn route_pose(
        &self,
        obstacles: &GridObstacleMap,
        src_x: i32, src_y: i32, src_layer: u8, src_theta_idx: u8,
        tgt_x: i32, tgt_y: i32, tgt_layer: u8, tgt_theta_idx: u8,
        max_iterations: u32,
        diff_pair_via_spacing: Option<i32>,
    ) -> (Option<Vec<(i32, i32, u8, u8)>>, u32) {
        let dubins = DubinsCalculator::new(self.min_radius_grid);

        let start = PoseState::new(src_x, src_y, src_theta_idx, src_layer);
        let goal = PoseState::new(tgt_x, tgt_y, tgt_theta_idx, tgt_layer);
        let goal_key = goal.as_key();

        let mut open_set = BinaryHeap::new();
        let mut g_costs: FxHashMap<u64, i32> = FxHashMap::default();
        let mut parents: FxHashMap<u64, u64> = FxHashMap::default();
        let mut closed: FxHashSet<u64> = FxHashSet::default();
        let mut counter: u32 = 0;

        // Track steps from source for via constraint (need 2+ steps before via)
        let mut steps_from_source: FxHashMap<u64, i32> = FxHashMap::default();

        // Track "steps since last via" - when >0, must continue straight (no turns)
        // Value: remaining straight steps required (2 after via, decrements each step)
        let mut straight_steps_remaining: FxHashMap<u64, i32> = FxHashMap::default();

        // Initialize with start pose
        let start_key = start.as_key();
        let h = self.dubins_heuristic(&dubins, &start, &goal);
        open_set.push(PoseOpenEntry {
            f_score: h,
            g_score: 0,
            state: start,
            counter,
        });
        counter += 1;
        g_costs.insert(start_key, 0);
        steps_from_source.insert(start_key, 0);
        straight_steps_remaining.insert(start_key, 0);

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

            // Goal check: position AND orientation must match
            if current_key == goal_key {
                let path = self.reconstruct_pose_path(&parents, current_key);
                return (Some(path), iterations);
            }

            closed.insert(current_key);

            // Get current constraint state
            let current_steps = steps_from_source.get(&current_key).copied().unwrap_or(0);
            let current_straight_remaining = straight_steps_remaining.get(&current_key).copied().unwrap_or(0);

            // Expand neighbors: can move forward OR turn in place
            // 1. Move forward in current direction
            let (dx, dy) = current.direction();
            let nx = current.gx + dx;
            let ny = current.gy + dy;

            if !obstacles.is_blocked(nx, ny, current.layer as usize) {
                let neighbor = PoseState::new(nx, ny, current.theta_idx, current.layer);
                let neighbor_key = neighbor.as_key();

                if !closed.contains(&neighbor_key) {
                    let move_cost = if dx != 0 && dy != 0 { DIAG_COST } else { ORTHO_COST };
                    let proximity_cost = obstacles.get_stub_proximity_cost(nx, ny);
                    let new_g = g + move_cost + proximity_cost;

                    let existing_g = g_costs.get(&neighbor_key).copied().unwrap_or(i32::MAX);
                    if new_g < existing_g {
                        g_costs.insert(neighbor_key, new_g);
                        parents.insert(neighbor_key, current_key);
                        // Update constraint tracking for straight move
                        steps_from_source.insert(neighbor_key, current_steps + 1);
                        straight_steps_remaining.insert(neighbor_key, (current_straight_remaining - 1).max(0));
                        let h = self.dubins_heuristic(&dubins, &neighbor, &goal);
                        open_set.push(PoseOpenEntry {
                            f_score: new_g + h,
                            g_score: new_g,
                            state: neighbor,
                            counter,
                        });
                        counter += 1;
                    }
                }
            }

            // 2. Move + turn by ±45°: move in the new direction while changing heading
            // With a minimum turning radius, you can't turn in place - must move along an arc
            // CONSTRAINTS:
            // - First move from start must be straight in src_theta direction
            // - After a via, must go straight for 2 steps (no turns)
            if current_key != start_key && current_straight_remaining <= 0 {
                for delta in [-1i8, 1i8] {
                    let new_theta = ((current.theta_idx as i8 + delta + 8) % 8) as u8;
                    let (dx, dy) = DIRECTIONS[new_theta as usize];
                    let nx = current.gx + dx;
                    let ny = current.gy + dy;

                    if !obstacles.is_blocked(nx, ny, current.layer as usize) {
                        let neighbor = PoseState::new(nx, ny, new_theta, current.layer);
                        let neighbor_key = neighbor.as_key();

                        if !closed.contains(&neighbor_key) {
                            // Cost = movement + turn arc cost
                            let move_cost = if dx != 0 && dy != 0 { DIAG_COST } else { ORTHO_COST };
                            let proximity_cost = obstacles.get_stub_proximity_cost(nx, ny);
                            let new_g = g + move_cost + self.turn_cost + proximity_cost;

                            let existing_g = g_costs.get(&neighbor_key).copied().unwrap_or(i32::MAX);
                            if new_g < existing_g {
                                g_costs.insert(neighbor_key, new_g);
                                parents.insert(neighbor_key, current_key);
                                // Update constraint tracking for turn move
                                steps_from_source.insert(neighbor_key, current_steps + 1);
                                straight_steps_remaining.insert(neighbor_key, 0);
                                let h = self.dubins_heuristic(&dubins, &neighbor, &goal);
                                open_set.push(PoseOpenEntry {
                                    f_score: new_g + h,
                                    g_score: new_g,
                                    state: neighbor,
                                    counter,
                                });
                                counter += 1;
                            }
                        }
                    }
                }
            }

            // 3. Via to other layer (keep same position and heading)
            // CONSTRAINTS for collinear vias:
            // - Need at least 2 steps from source before placing a via
            // - Cannot place via while still in post-via straight requirement
            // - After via, must go straight for 2 steps
            // - If diff_pair_via_spacing is set, check that offset positions are also clear
            let can_place_via = current_steps >= 2 && current_straight_remaining <= 0;

            // Check centerline via position
            let mut via_positions_clear = !obstacles.is_via_blocked(current.gx, current.gy);

            // For diff pairs, also check the perpendicular offset positions where P/N vias will go
            if via_positions_clear {
                if let Some(spacing) = diff_pair_via_spacing {
                    let (dx, dy) = current.direction();
                    // Perpendicular direction: rotate 90° -> (-dy, dx)
                    let perp_x = -dy;
                    let perp_y = dx;
                    // Check both offset positions
                    let p_via_x = current.gx + perp_x * spacing;
                    let p_via_y = current.gy + perp_y * spacing;
                    let n_via_x = current.gx - perp_x * spacing;
                    let n_via_y = current.gy - perp_y * spacing;
                    if obstacles.is_via_blocked(p_via_x, p_via_y) || obstacles.is_via_blocked(n_via_x, n_via_y) {
                        via_positions_clear = false;
                    }
                }
            }

            // Block vias within stub proximity radius (for diff pairs)
            if via_positions_clear && diff_pair_via_spacing.is_some() {
                if obstacles.get_stub_proximity_cost(current.gx, current.gy) > 0 {
                    via_positions_clear = false;
                }
            }

            if can_place_via && via_positions_clear {
                for layer in 0..obstacles.num_layers as u8 {
                    if layer == current.layer {
                        continue;
                    }

                    if obstacles.is_blocked(current.gx, current.gy, layer as usize) {
                        continue;
                    }

                    let neighbor = PoseState::new(current.gx, current.gy, current.theta_idx, layer);
                    let neighbor_key = neighbor.as_key();

                    if !closed.contains(&neighbor_key) {
                        let proximity_cost = obstacles.get_stub_proximity_cost(current.gx, current.gy) * 2;
                        let new_g = g + self.via_cost + proximity_cost;

                        let existing_g = g_costs.get(&neighbor_key).copied().unwrap_or(i32::MAX);
                        if new_g < existing_g {
                            g_costs.insert(neighbor_key, new_g);
                            parents.insert(neighbor_key, current_key);
                            // Via doesn't count as a step, but sets straight requirement
                            steps_from_source.insert(neighbor_key, current_steps);
                            straight_steps_remaining.insert(neighbor_key, 2);  // Must go straight for 2 steps after via
                            let h = self.dubins_heuristic(&dubins, &neighbor, &goal);
                            open_set.push(PoseOpenEntry {
                                f_score: new_g + h,
                                g_score: new_g,
                                state: neighbor,
                                counter,
                            });
                            counter += 1;
                        }
                    }
                }
            }
        }

        (None, iterations)
    }
}

impl PoseRouter {
    /// Dubins heuristic: estimate shortest path considering orientation
    fn dubins_heuristic(&self, dubins: &DubinsCalculator, state: &PoseState, goal: &PoseState) -> i32 {
        let theta1 = state.theta_radians();
        let theta2 = goal.theta_radians();

        let mut h = dubins.path_length(
            state.gx as f64, state.gy as f64, theta1,
            goal.gx as f64, goal.gy as f64, theta2,
        );

        // Add via cost if layers differ
        if state.layer != goal.layer {
            h += self.via_cost;
        }

        (h as f32 * self.h_weight) as i32
    }

    /// Reconstruct path from parents map
    fn reconstruct_pose_path(&self, parents: &FxHashMap<u64, u64>, goal_key: u64) -> Vec<(i32, i32, u8, u8)> {
        let mut path = Vec::new();
        let mut current_key = goal_key;

        loop {
            // Unpack key: 19 bits x, 19 bits y, 3 bits theta, 8 bits layer
            let l = (current_key & 0xFF) as u8;
            let t = ((current_key >> 8) & 0x7) as u8;
            let y = ((current_key >> 11) & 0x7FFFF) as i32;
            let x = ((current_key >> 30) & 0x7FFFF) as i32;
            // Sign extension for negative coordinates
            let x = if x & 0x40000 != 0 { x | !0x7FFFF_i32 } else { x };
            let y = if y & 0x40000 != 0 { y | !0x7FFFF_i32 } else { y };

            path.push((x, y, t, l));

            match parents.get(&current_key) {
                Some(&parent_key) => current_key = parent_key,
                None => break,
            }
        }

        path.reverse();
        path
    }
}

/// Module version
const VERSION: &str = "0.7.0";

/// Python module
#[pymodule]
fn grid_router(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add("__version__", VERSION)?;
    m.add_class::<GridObstacleMap>()?;
    m.add_class::<GridRouter>()?;
    m.add_class::<PoseRouter>()?;
    m.add_class::<VisualRouter>()?;
    m.add_class::<SearchSnapshot>()?;
    Ok(())
}
