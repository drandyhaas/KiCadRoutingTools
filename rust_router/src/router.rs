//! Grid-based A* router implementation.

use pyo3::prelude::*;
use rustc_hash::{FxHashMap, FxHashSet};
use std::collections::BinaryHeap;

use crate::obstacle_map::GridObstacleMap;
use crate::types::{GridState, OpenEntry, BlockedCellTracker, RouteStats, DIRECTIONS, ORTHO_COST, DIAG_COST, DEFAULT_TURN_COST};

/// Grid A* Router
#[pyclass]
pub struct GridRouter {
    via_cost: i32,
    h_weight: f32,
    turn_cost: i32,
    via_proximity_cost: i32,  // Multiplier for stub proximity cost when placing vias (0 = block vias near stubs)
    vertical_attraction_radius: i32,  // Grid units for cross-layer attraction lookup (0 = disabled)
    vertical_attraction_bonus: i32,   // Cost reduction for positions aligned with other-layer tracks
    layer_costs: Vec<i32>,  // Per-layer cost multipliers (1000 = 1.0x, 1500 = 1.5x penalty)
    proximity_heuristic_cost: i32,  // Expected proximity cost per grid step (added to heuristic)
    layer_direction_preferences: Vec<u8>,  // Per-layer direction preference (0=horizontal, 1=vertical, 255=none)
    direction_preference_cost: i32,  // Cost penalty for non-preferred direction moves
    // Bus routing: attraction to a previously routed path (same layer only)
    // Stores path with direction vectors: (gx, gy, layer, dx, dy) where dx,dy are normalized direction
    attraction_path: Vec<(i32, i32, u8, i8, i8)>,  // Path to attract to with direction
    attraction_radius: i32,  // Grid units for attraction (0 = disabled)
    attraction_bonus: i32,   // Cost reduction when moving parallel to path (same layer)
    // Spatial hash for efficient path distance lookup
    attraction_path_hash: FxHashMap<u64, i32>,  // cell_key -> min distance to path point in that cell
}

#[pymethods]
impl GridRouter {
    #[new]
    #[pyo3(signature = (via_cost, h_weight, turn_cost=None, via_proximity_cost=1, vertical_attraction_radius=0, vertical_attraction_bonus=0, layer_costs=None, proximity_heuristic_cost=None, layer_direction_preferences=None, direction_preference_cost=0, attraction_radius=0, attraction_bonus=0))]
    pub fn new(via_cost: i32, h_weight: f32, turn_cost: Option<i32>, via_proximity_cost: Option<i32>,
               vertical_attraction_radius: i32, vertical_attraction_bonus: i32,
               layer_costs: Option<Vec<i32>>, proximity_heuristic_cost: Option<i32>,
               layer_direction_preferences: Option<Vec<u8>>, direction_preference_cost: i32,
               attraction_radius: i32, attraction_bonus: i32) -> Self {
        Self {
            via_cost,
            h_weight,
            turn_cost: turn_cost.unwrap_or(DEFAULT_TURN_COST),
            via_proximity_cost: via_proximity_cost.unwrap_or(1),
            vertical_attraction_radius,
            vertical_attraction_bonus,
            layer_costs: layer_costs.unwrap_or_default(),
            proximity_heuristic_cost: proximity_heuristic_cost.unwrap_or(0),  // Default: no proximity estimate
            layer_direction_preferences: layer_direction_preferences.unwrap_or_default(),
            direction_preference_cost,
            attraction_path: Vec::new(),
            attraction_radius,
            attraction_bonus,
            attraction_path_hash: FxHashMap::default(),
        }
    }

    /// Set the proximity heuristic cost for subsequent routes.
    /// Call this before each route to adjust based on whether endpoints are in proximity zones.
    pub fn set_proximity_heuristic_cost(&mut self, cost: i32) {
        self.proximity_heuristic_cost = cost;
    }

    /// Set the attraction path for bus routing.
    /// The router will give a cost bonus for moving parallel to this path (same layer only).
    /// Direction-based attraction prevents spiraling by only rewarding moves in the same
    /// direction the neighbor was moving, not just proximity.
    /// Call this before routing each subsequent bus member with the previously routed path.
    /// Pass an empty Vec to clear the attraction.
    pub fn set_attraction_path(&mut self, path: Vec<(i32, i32, u8)>) {
        self.attraction_path_hash.clear();
        self.attraction_path.clear();

        if path.is_empty() || self.attraction_radius <= 0 {
            return;
        }

        // Convert path to include direction vectors
        // Direction at each point is computed from the move to the next point
        // (or from previous point for the last point)
        let path_with_directions: Vec<(i32, i32, u8, i8, i8)> = path.iter().enumerate().map(|(i, &(gx, gy, layer))| {
            let (dx, dy) = if i < path.len() - 1 {
                // Direction to next point
                let (nx, ny, _) = path[i + 1];
                let raw_dx = nx - gx;
                let raw_dy = ny - gy;
                (raw_dx.signum() as i8, raw_dy.signum() as i8)
            } else if i > 0 {
                // Last point: use direction from previous point
                let (px, py, _) = path[i - 1];
                let raw_dx = gx - px;
                let raw_dy = gy - py;
                (raw_dx.signum() as i8, raw_dy.signum() as i8)
            } else {
                // Single point path - no direction
                (0, 0)
            };
            (gx, gy, layer, dx, dy)
        }).collect();

        // Build spatial hash for efficient distance lookups
        // Key: (gx / bucket_size, gy / bucket_size, layer) packed into u64
        let bucket_size = (self.attraction_radius / 2).max(1);

        for &(px, py, layer, _, _) in &path_with_directions {
            // Add this path point to its bucket and neighboring buckets within radius
            let bx = px / bucket_size;
            let by = py / bucket_size;

            // Mark cells within attraction radius
            let cells_to_check = (self.attraction_radius / bucket_size) + 1;
            for dbx in -cells_to_check..=cells_to_check {
                for dby in -cells_to_check..=cells_to_check {
                    let cell_bx = bx + dbx;
                    let cell_by = by + dby;
                    // Pack bucket coords + layer into key
                    let key = Self::pack_bucket_key(cell_bx, cell_by, layer, bucket_size);
                    // Store that this bucket has path points nearby
                    self.attraction_path_hash.entry(key).or_insert(0);
                }
            }
        }

        self.attraction_path = path_with_directions;
    }

    /// Clear the attraction path
    pub fn clear_attraction_path(&mut self) {
        self.attraction_path.clear();
        self.attraction_path_hash.clear();
    }

    /// Route from multiple source points to multiple target points.
    /// Returns (path, iterations, stats) where path is list of (gx, gy, layer) tuples,
    /// or (None, iterations, stats) if no path found. Stats is a dict with search statistics.
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
    /// track_margin: Extra margin in grid cells for wide tracks (e.g., power nets).
    /// When > 0, checks cells within this radius for obstacles.
    #[pyo3(signature = (obstacles, sources, targets, max_iterations, collinear_vias=false, via_exclusion_radius=0, start_direction=None, end_direction=None, direction_steps=2, track_margin=0))]
    pub fn route_multi(
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
        track_margin: i32,
    ) -> (Option<Vec<(i32, i32, u8)>>, u32, std::collections::HashMap<String, f64>) {
        let mut stats = RouteStats::default();

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


        // Find minimum layer cost for initial g penalty
        let min_layer_cost = self.layer_costs.iter().copied().min().unwrap_or(1000);
        let mut best_initial_h = i32::MAX;

        for (gx, gy, layer) in sources {
            let state = GridState::new(gx, gy, layer);
            let key = state.as_key();
            // Penalize starting on expensive layers
            let layer_cost = self.layer_costs.get(layer as usize).copied().unwrap_or(1000);
            let initial_g = layer_cost - min_layer_cost;
            let h = self.heuristic_to_targets(&state, &target_states);
            best_initial_h = best_initial_h.min(h);
            open_set.push(OpenEntry {
                f_score: initial_g + h,
                g_score: initial_g,
                state,
                counter,
            });
            counter += 1;
            stats.cells_pushed += 1;
            g_costs.insert(key, initial_g);
            // Source nodes start with no vias on their path
            if via_exclusion_radius > 0 {
                path_vias.insert(key, Vec::new());
            }
            // Source nodes are at step 0
            steps_from_source.insert(key, 0);
        }
        stats.initial_h = best_initial_h;

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
                stats.duplicate_skips += 1;
                continue;
            }
            stats.cells_expanded += 1;

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
                    stats.path_length = path.len() as u32;
                    stats.path_cost = g;
                    stats.final_g = g;
                    stats.open_set_size = open_set.len() as u32;
                    stats.closed_set_size = closed.len() as u32;
                    // Count vias in path
                    for i in 1..path.len() {
                        if path[i].2 != path[i-1].2 {
                            stats.via_count += 1;
                        }
                    }
                    let stats_dict = self.stats_to_dict(&stats);
                    return (Some(path), iterations, stats_dict);
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

            // Get previous direction (parent -> current) for turn cost calculation
            let prev_direction: Option<(i32, i32)> = parents.get(&current_key).map(|&parent_key| {
                let (px, py, _) = Self::unpack_key(parent_key);
                let pdx = current.gx - px;
                let pdy = current.gy - py;
                // Normalize to unit direction (handle multi-step moves like vias)
                if pdx == 0 && pdy == 0 {
                    (0, 0) // Via (same position), no direction
                } else {
                    (pdx.signum(), pdy.signum())
                }
            });

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

                if obstacles.is_blocked_with_margin(ngx, ngy, current.layer as usize, track_margin) {
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

                let base_move_cost = if dx != 0 && dy != 0 { DIAG_COST } else { ORTHO_COST };
                // Apply layer cost multiplier (1000 = 1.0x, 1500 = 1.5x, etc.)
                let layer_multiplier = self.layer_costs.get(current.layer as usize).copied().unwrap_or(1000);
                let move_cost = (base_move_cost as i64 * layer_multiplier as i64 / 1000) as i32;
                // Add turn cost if direction changes (encourages straighter paths)
                let turn_cost = match prev_direction {
                    Some((pdx, pdy)) if pdx != 0 || pdy != 0 => {
                        if dx != pdx || dy != pdy { self.turn_cost } else { 0 }
                    }
                    _ => 0, // No previous direction (source node or via)
                };
                // Stub and layer proximity costs
                let proximity_cost = obstacles.get_stub_proximity_cost(ngx, ngy)
                    + obstacles.get_layer_proximity_cost(ngx, ngy, current.layer as usize);
                // Subtract attraction bonus for positions aligned with tracks on other layers
                let attraction_bonus = obstacles.get_cross_layer_attraction(
                    ngx, ngy, current.layer as usize,
                    self.vertical_attraction_radius, self.vertical_attraction_bonus);
                // Layer direction preference penalty (0=horizontal preferred, 1=vertical preferred)
                let direction_penalty = if self.direction_preference_cost > 0 {
                    let preferred = self.layer_direction_preferences.get(current.layer as usize).copied().unwrap_or(255);
                    match preferred {
                        0 => if dy != 0 && dx == 0 { self.direction_preference_cost } else { 0 },  // Horizontal pref, penalize pure vertical
                        1 => if dx != 0 && dy == 0 { self.direction_preference_cost } else { 0 },  // Vertical pref, penalize pure horizontal
                        _ => 0  // No preference (255 or other)
                    }
                } else { 0 };
                // Path attraction bonus for bus routing - direction-based to prevent spiraling
                let path_attraction_bonus = self.get_path_attraction_bonus(ngx, ngy, current.layer, dx, dy);
                let new_g = g + move_cost + turn_cost + proximity_cost + direction_penalty - attraction_bonus - path_attraction_bonus;

                let existing_g = g_costs.get(&neighbor_key).copied().unwrap_or(i32::MAX);
                if new_g < existing_g {
                    if existing_g != i32::MAX {
                        stats.cells_revisited += 1;
                    }
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
                    stats.cells_pushed += 1;
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
                    if obstacles.is_blocked_with_margin(current.gx, current.gy, layer as usize, track_margin) {
                        continue;
                    }

                    let neighbor = GridState::new(current.gx, current.gy, layer);
                    let neighbor_key = neighbor.as_key();

                    if closed.contains(&neighbor_key) {
                        continue;
                    }

                    // Use zero cost for free via positions (through-hole pads on same net)
                    let is_free = obstacles.is_free_via(current.gx, current.gy);
                    let via_cost = if is_free { 0 } else { self.via_cost };
                    let proximity_cost = (obstacles.get_stub_proximity_cost(current.gx, current.gy)
                        + obstacles.get_layer_proximity_cost(current.gx, current.gy, layer as usize))
                        * self.via_proximity_cost;
                    // Layer transition cost: penalize switching TO expensive layers, discount switching to cheaper
                    let current_layer_cost = self.layer_costs.get(current.layer as usize).copied().unwrap_or(1000);
                    let dest_layer_cost = self.layer_costs.get(layer as usize).copied().unwrap_or(1000);
                    let layer_transition_cost = dest_layer_cost - current_layer_cost;
                    // Combined via cost can be as low as 0 when switching to a much cheaper layer
                    let combined_via_cost = (via_cost + layer_transition_cost).max(0);
                    let new_g = g + combined_via_cost + proximity_cost;

                    let existing_g = g_costs.get(&neighbor_key).copied().unwrap_or(i32::MAX);
                    if new_g < existing_g {
                        if existing_g != i32::MAX {
                            stats.cells_revisited += 1;
                        }
                        g_costs.insert(neighbor_key, new_g);
                        parents.insert(neighbor_key, current_key);
                        // Track via positions (only real vias, not free vias at through-holes)
                        if via_exclusion_radius > 0 {
                            if is_free {
                                path_vias.insert(neighbor_key, current_vias.clone());
                            } else {
                                let mut new_vias = current_vias.clone();
                                new_vias.push((current.gx, current.gy));
                                path_vias.insert(neighbor_key, new_vias);
                            }
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
                        stats.cells_pushed += 1;
                    }
                }
            }
        }

        // No path found - fill in final stats
        stats.open_set_size = open_set.len() as u32;
        stats.closed_set_size = closed.len() as u32;
        let stats_dict = self.stats_to_dict(&stats);
        (None, iterations, stats_dict)
    }

    /// Route with frontier analysis - returns blocked cells on failure.
    ///
    /// Same as route_multi but on failure returns the set of blocked cells
    /// that were encountered during the search. This helps diagnose which
    /// obstacles are preventing the route.
    ///
    /// Returns (path, iterations, blocked_cells) where:
    /// - On success: path is Some, blocked_cells is empty
    /// - On failure: path is None, blocked_cells contains cells that blocked expansion
    #[pyo3(signature = (obstacles, sources, targets, max_iterations, collinear_vias=false, via_exclusion_radius=0, start_direction=None, end_direction=None, direction_steps=2, track_margin=0))]
    pub fn route_with_frontier(
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
        track_margin: i32,
    ) -> (Option<Vec<(i32, i32, u8)>>, u32, Vec<(i32, i32, u8)>) {
        let mut tracker = BlockedCellTracker::new();

        let target_set: FxHashSet<u64> = targets
            .iter()
            .map(|(gx, gy, layer)| GridState::new(*gx, *gy, *layer).as_key())
            .collect();

        let target_states: Vec<GridState> = targets
            .iter()
            .map(|(gx, gy, layer)| GridState::new(*gx, *gy, *layer))
            .collect();

        let mut open_set = BinaryHeap::new();
        let mut g_costs: FxHashMap<u64, i32> = FxHashMap::default();
        let mut parents: FxHashMap<u64, u64> = FxHashMap::default();
        let mut closed: FxHashSet<u64> = FxHashSet::default();
        let mut counter: u32 = 0;
        let mut path_vias: FxHashMap<u64, Vec<(i32, i32)>> = FxHashMap::default();
        let mut steps_from_source: FxHashMap<u64, i32> = FxHashMap::default();

        let norm_start_dir: Option<(i32, i32)> = start_direction.map(|(dx, dy)| {
            let ndx = if dx != 0 { dx / dx.abs() } else { 0 };
            let ndy = if dy != 0 { dy / dy.abs() } else { 0 };
            (ndx, ndy)
        });

        let norm_end_dir: Option<(f64, f64)> = end_direction.map(|(dx, dy)| {
            let len = (dx * dx + dy * dy).sqrt();
            if len > 0.0 { (dx / len, dy / len) } else { (0.0, 0.0) }
        });

        let check_via_exclusion = |nx: i32, ny: i32, current_gx: i32, current_gy: i32, vias: &[(i32, i32)], radius: i32| -> bool {
            if radius <= 0 { return false; }
            for &(vx, vy) in vias {
                let dist_to_neighbor = (nx - vx).abs().max((ny - vy).abs());
                let dist_to_current = (current_gx - vx).abs().max((current_gy - vy).abs());
                if dist_to_neighbor <= radius && dist_to_current > radius { return true; }
                if dist_to_current <= radius && dist_to_neighbor <= dist_to_current {
                    if dist_to_neighbor < dist_to_current { return true; }
                    if dist_to_neighbor == dist_to_current && dist_to_current > 0 {
                        let dx_from_via = (nx - vx).abs() - (current_gx - vx).abs();
                        let dy_from_via = (ny - vy).abs() - (current_gy - vy).abs();
                        if (dx_from_via > 0 && dy_from_via < 0) || (dx_from_via < 0 && dy_from_via > 0) {
                            if dist_to_current <= radius / 2 { return true; }
                        }
                    }
                }
            }
            false
        };

        // Find minimum layer cost for initial g penalty
        let min_layer_cost = self.layer_costs.iter().copied().min().unwrap_or(1000);

        for (gx, gy, layer) in sources {
            let state = GridState::new(gx, gy, layer);
            let key = state.as_key();
            // Penalize starting on expensive layers
            let layer_cost = self.layer_costs.get(layer as usize).copied().unwrap_or(1000);
            let initial_g = layer_cost - min_layer_cost;
            let h = self.heuristic_to_targets(&state, &target_states);
            open_set.push(OpenEntry { f_score: initial_g + h, g_score: initial_g, state, counter });
            counter += 1;
            g_costs.insert(key, initial_g);
            if via_exclusion_radius > 0 { path_vias.insert(key, Vec::new()); }
            steps_from_source.insert(key, 0);
        }

        let mut iterations: u32 = 0;

        while let Some(current_entry) = open_set.pop() {
            if iterations >= max_iterations { break; }
            iterations += 1;

            let current = current_entry.state;
            let current_key = current.as_key();
            let g = current_entry.g_score;

            if closed.contains(&current_key) { continue; }

            if target_set.contains(&current_key) {
                let arrival_ok = if let Some((end_dx, end_dy)) = norm_end_dir {
                    if let Some(&parent_key) = parents.get(&current_key) {
                        let (px, py, _) = Self::unpack_key(parent_key);
                        let arrive_dx = (current.gx - px) as f64;
                        let arrive_dy = (current.gy - py) as f64;
                        let arrive_len = (arrive_dx * arrive_dx + arrive_dy * arrive_dy).sqrt();
                        if arrive_len > 0.0 {
                            let dot = (arrive_dx / arrive_len) * end_dx + (arrive_dy / arrive_len) * end_dy;
                            dot >= -0.5
                        } else { true }
                    } else { true }
                } else { true };

                if arrival_ok {
                    let path = self.reconstruct_path(&parents, current_key, &g_costs);
                    return (Some(path), iterations, Vec::new());
                }
                continue;
            }

            closed.insert(current_key);

            let (required_direction, allowed_45deg_from) = if collinear_vias {
                self.get_via_direction_constraints(&parents, current_key, &current)
            } else { (None, None) };

            let current_vias: Vec<(i32, i32)> = if via_exclusion_radius > 0 {
                path_vias.get(&current_key).cloned().unwrap_or_default()
            } else { Vec::new() };

            let current_steps = steps_from_source.get(&current_key).copied().unwrap_or(i32::MAX);

            // Get previous direction (parent -> current) for turn cost calculation
            let prev_direction: Option<(i32, i32)> = parents.get(&current_key).map(|&parent_key| {
                let (px, py, _) = Self::unpack_key(parent_key);
                let pdx = current.gx - px;
                let pdy = current.gy - py;
                if pdx == 0 && pdy == 0 { (0, 0) } else { (pdx.signum(), pdy.signum()) }
            });

            for (dx, dy) in DIRECTIONS {
                if let Some((req_dx, req_dy)) = required_direction {
                    if dx != req_dx || dy != req_dy { continue; }
                } else if let Some((base_dx, base_dy)) = allowed_45deg_from {
                    if !Self::is_within_45_degrees(dx, dy, base_dx, base_dy) { continue; }
                }

                if let Some((start_dx, start_dy)) = norm_start_dir {
                    if current_steps < direction_steps && !Self::is_within_45_degrees(dx, dy, start_dx, start_dy) {
                        continue;
                    }
                }

                let ngx = current.gx + dx;
                let ngy = current.gy + dy;

                if obstacles.is_blocked_with_margin(ngx, ngy, current.layer as usize, track_margin) {
                    tracker.track(ngx, ngy, current.layer);
                    continue;
                }

                if check_via_exclusion(ngx, ngy, current.gx, current.gy, &current_vias, via_exclusion_radius) {
                    continue;
                }

                let neighbor = GridState::new(ngx, ngy, current.layer);
                let neighbor_key = neighbor.as_key();

                if closed.contains(&neighbor_key) { continue; }

                let base_move_cost = if dx != 0 && dy != 0 { DIAG_COST } else { ORTHO_COST };
                // Apply layer cost multiplier (1000 = 1.0x, 1500 = 1.5x, etc.)
                let layer_multiplier = self.layer_costs.get(current.layer as usize).copied().unwrap_or(1000);
                let move_cost = (base_move_cost as i64 * layer_multiplier as i64 / 1000) as i32;
                let turn_cost = match prev_direction {
                    Some((pdx, pdy)) if pdx != 0 || pdy != 0 => {
                        if dx != pdx || dy != pdy { self.turn_cost } else { 0 }
                    }
                    _ => 0,
                };
                // Stub and layer proximity costs
                let proximity_cost = obstacles.get_stub_proximity_cost(ngx, ngy)
                    + obstacles.get_layer_proximity_cost(ngx, ngy, current.layer as usize);
                // Subtract attraction bonus for positions aligned with tracks on other layers
                let attraction_bonus = obstacles.get_cross_layer_attraction(
                    ngx, ngy, current.layer as usize,
                    self.vertical_attraction_radius, self.vertical_attraction_bonus);
                // Layer direction preference penalty (0=horizontal preferred, 1=vertical preferred)
                let direction_penalty = if self.direction_preference_cost > 0 {
                    let preferred = self.layer_direction_preferences.get(current.layer as usize).copied().unwrap_or(255);
                    match preferred {
                        0 => if dy != 0 && dx == 0 { self.direction_preference_cost } else { 0 },
                        1 => if dx != 0 && dy == 0 { self.direction_preference_cost } else { 0 },
                        _ => 0
                    }
                } else { 0 };
                // Path attraction bonus for bus routing - direction-based to prevent spiraling
                let path_attraction_bonus = self.get_path_attraction_bonus(ngx, ngy, current.layer, dx, dy);
                let new_g = g + move_cost + turn_cost + proximity_cost + direction_penalty - attraction_bonus - path_attraction_bonus;

                let existing_g = g_costs.get(&neighbor_key).copied().unwrap_or(i32::MAX);
                if new_g < existing_g {
                    g_costs.insert(neighbor_key, new_g);
                    parents.insert(neighbor_key, current_key);
                    if via_exclusion_radius > 0 { path_vias.insert(neighbor_key, current_vias.clone()); }
                    steps_from_source.insert(neighbor_key, current_steps + 1);
                    let h = self.heuristic_to_targets(&neighbor, &target_states);
                    open_set.push(OpenEntry { f_score: new_g + h, g_score: new_g, state: neighbor, counter });
                    counter += 1;
                }
            }

            // Via expansion
            let can_place_via = if collinear_vias {
                if let Some(&parent_key) = parents.get(&current_key) {
                    if let Some(&grandparent_key) = parents.get(&parent_key) {
                        if !parents.contains_key(&grandparent_key) { false }
                        else {
                            let (parent_x, parent_y, _) = Self::unpack_key(parent_key);
                            let (gp_x, gp_y, _) = Self::unpack_key(grandparent_key);
                            let prev_dx = parent_x - gp_x;
                            let prev_dy = parent_y - gp_y;
                            let approach_dx = current.gx - parent_x;
                            let approach_dy = current.gy - parent_y;
                            if (prev_dx != 0 || prev_dy != 0) && (approach_dx != 0 || approach_dy != 0) {
                                let norm_prev_dx = if prev_dx != 0 { prev_dx / prev_dx.abs() } else { 0 };
                                let norm_prev_dy = if prev_dy != 0 { prev_dy / prev_dy.abs() } else { 0 };
                                let norm_approach_dx = if approach_dx != 0 { approach_dx / approach_dx.abs() } else { 0 };
                                let norm_approach_dy = if approach_dy != 0 { approach_dy / approach_dy.abs() } else { 0 };
                                Self::is_within_45_degrees(norm_approach_dx, norm_approach_dy, norm_prev_dx, norm_prev_dy)
                            } else { false }
                        }
                    } else { false }
                } else { false }
            } else { true };

            let via_too_close = if via_exclusion_radius > 0 {
                current_vias.iter().any(|&(vx, vy)| {
                    (current.gx - vx).abs().max((current.gy - vy).abs()) <= via_exclusion_radius * 2
                })
            } else { false };

            if can_place_via && !via_too_close && !obstacles.is_via_blocked(current.gx, current.gy) {
                for layer in 0..obstacles.num_layers as u8 {
                    if layer == current.layer { continue; }

                    if obstacles.is_blocked_with_margin(current.gx, current.gy, layer as usize, track_margin) {
                        tracker.track(current.gx, current.gy, layer);
                        continue;
                    }

                    let neighbor = GridState::new(current.gx, current.gy, layer);
                    let neighbor_key = neighbor.as_key();

                    if closed.contains(&neighbor_key) { continue; }

                    // Use zero cost for free via positions (through-hole pads on same net)
                    let is_free = obstacles.is_free_via(current.gx, current.gy);
                    let via_cost = if is_free { 0 } else { self.via_cost };
                    let proximity_cost = (obstacles.get_stub_proximity_cost(current.gx, current.gy)
                        + obstacles.get_layer_proximity_cost(current.gx, current.gy, layer as usize))
                        * self.via_proximity_cost;
                    // Layer transition cost: penalize switching TO expensive layers, discount switching to cheaper
                    let current_layer_cost = self.layer_costs.get(current.layer as usize).copied().unwrap_or(1000);
                    let dest_layer_cost = self.layer_costs.get(layer as usize).copied().unwrap_or(1000);
                    let layer_transition_cost = dest_layer_cost - current_layer_cost;
                    // Combined via cost can be as low as 0 when switching to a much cheaper layer
                    let combined_via_cost = (via_cost + layer_transition_cost).max(0);
                    let new_g = g + combined_via_cost + proximity_cost;

                    let existing_g = g_costs.get(&neighbor_key).copied().unwrap_or(i32::MAX);
                    if new_g < existing_g {
                        g_costs.insert(neighbor_key, new_g);
                        parents.insert(neighbor_key, current_key);
                        // Track via positions (only real vias, not free vias at through-holes)
                        if via_exclusion_radius > 0 {
                            if is_free {
                                path_vias.insert(neighbor_key, current_vias.clone());
                            } else {
                                let mut new_vias = current_vias.clone();
                                new_vias.push((current.gx, current.gy));
                                path_vias.insert(neighbor_key, new_vias);
                            }
                        }
                        steps_from_source.insert(neighbor_key, current_steps);
                        let h = self.heuristic_to_targets(&neighbor, &target_states);
                        open_set.push(OpenEntry { f_score: new_g + h, g_score: new_g, state: neighbor, counter });
                        counter += 1;
                    }
                }
            }
        }

        (None, iterations, tracker.get_blocked())
    }
}

impl GridRouter {
    /// Octile distance heuristic to nearest target
    /// Uses minimum layer cost to remain admissible (never overestimate)
    #[inline]
    fn heuristic_to_targets(&self, state: &GridState, targets: &[GridState]) -> i32 {
        // Use minimum layer cost for admissibility
        let min_layer_cost = self.layer_costs.iter().copied().min().unwrap_or(1000);

        let mut min_h = i32::MAX;
        for target in targets {
            let dx = (state.gx - target.gx).abs();
            let dy = (state.gy - target.gy).abs();
            let diag = dx.min(dy);
            let orth = (dx - dy).abs();
            let base_dist = diag * DIAG_COST + orth * ORTHO_COST;
            // Scale distance by minimum layer cost for admissibility
            let mut h = (base_dist as i64 * min_layer_cost as i64 / 1000) as i32;
            if state.layer != target.layer {
                h += self.via_cost;
            }
            // Add expected proximity cost per step (makes heuristic tighter for high-proximity boards)
            if self.proximity_heuristic_cost > 0 {
                let path_steps = diag + orth;
                h += path_steps * self.proximity_heuristic_cost;
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
    pub fn unpack_key(key: u64) -> (i32, i32, u8) {
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
    pub fn is_within_45_degrees(dx: i32, dy: i32, base_dx: i32, base_dy: i32) -> bool {
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

    /// Pack bucket coordinates into a key for spatial hash lookup
    #[inline]
    fn pack_bucket_key(bx: i32, by: i32, layer: u8, bucket_size: i32) -> u64 {
        // Use bucket_size as part of key to avoid collisions between different bucket sizes
        let _ = bucket_size; // Currently not used in key, but kept for future flexibility
        let layer_bits = layer as u64;
        let bx_bits = ((bx as u32) & 0xFFFFF) as u64;
        let by_bits = ((by as u32) & 0xFFFFF) as u64;
        layer_bits | (by_bits << 8) | (bx_bits << 28)
    }

    /// Calculate parallel-movement attraction bonus for bus routing.
    ///
    /// This rewards moving in the SAME DIRECTION as the neighbor path:
    /// 1. Find nearby path points on the same layer
    /// 2. Give bonus for moving in the same direction as the path at those points
    /// 3. No bonus for perpendicular or opposite movement
    ///
    /// This creates parallel tracks because:
    /// - Tracks are rewarded for moving the same direction as neighbor
    /// - Natural clearances keep them at appropriate spacing
    /// - No "pull toward" that would cause convergence
    ///
    /// Arguments:
    /// - x, y: Current position
    /// - layer: Current layer
    /// - move_dx, move_dy: Direction of the current move (normalized to -1, 0, 1)
    #[inline]
    fn get_path_attraction_bonus(&self, x: i32, y: i32, layer: u8, move_dx: i32, move_dy: i32) -> i32 {
        if self.attraction_bonus <= 0 || self.attraction_radius <= 0 || self.attraction_path.is_empty() {
            return 0;
        }

        // Quick check: is this position potentially near the path?
        let bucket_size = (self.attraction_radius / 2).max(1);
        let bx = x / bucket_size;
        let by = y / bucket_size;
        let key = Self::pack_bucket_key(bx, by, layer, bucket_size);

        if !self.attraction_path_hash.contains_key(&key) {
            return 0;
        }

        // Find all nearby path points and accumulate direction-matching bonus
        // Using the closest point's direction
        let mut nearest_dist = i32::MAX;
        let mut nearest_dir: Option<(i8, i8)> = None;

        for &(px, py, pl, path_dx, path_dy) in &self.attraction_path {
            if pl != layer {
                continue;
            }

            let dist = (x - px).abs() + (y - py).abs();
            if dist <= self.attraction_radius && dist < nearest_dist {
                nearest_dist = dist;
                nearest_dir = Some((path_dx, path_dy));
            }
        }

        let (path_dx, path_dy) = match nearest_dir {
            Some(d) => d,
            None => return 0,
        };

        // Check direction alignment using dot product
        let dot = move_dx * (path_dx as i32) + move_dy * (path_dy as i32);

        // Only give bonus for moves in the same general direction
        // dot >= 2: same diagonal (perfect)
        // dot == 1: same orthogonal or 45° off (good)
        // dot <= 0: perpendicular or opposite (no bonus)
        if dot < 1 {
            return 0;
        }

        // Alignment factor: 100% for perfect match, 70% for 45° off
        let alignment = if dot >= 2 { 100 } else { 70 };

        // Proximity factor with quadratic falloff (stronger near path)
        let proximity_ratio = (self.attraction_radius - nearest_dist) as f32 / self.attraction_radius as f32;
        let proximity_pct = (proximity_ratio * proximity_ratio * 100.0) as i32;

        // Calculate bonus: base * proximity% * alignment%
        (self.attraction_bonus as i64 * proximity_pct as i64 * alignment as i64 / 10000) as i32
    }

    /// Convert RouteStats to a Python-friendly dictionary
    fn stats_to_dict(&self, stats: &RouteStats) -> std::collections::HashMap<String, f64> {
        let mut dict = std::collections::HashMap::new();
        dict.insert("cells_expanded".to_string(), stats.cells_expanded as f64);
        dict.insert("cells_pushed".to_string(), stats.cells_pushed as f64);
        dict.insert("cells_revisited".to_string(), stats.cells_revisited as f64);
        dict.insert("duplicate_skips".to_string(), stats.duplicate_skips as f64);
        dict.insert("path_length".to_string(), stats.path_length as f64);
        dict.insert("path_cost".to_string(), stats.path_cost as f64);
        dict.insert("initial_h".to_string(), stats.initial_h as f64);
        dict.insert("final_g".to_string(), stats.final_g as f64);
        dict.insert("max_f_score".to_string(), stats.max_f_score as f64);
        dict.insert("open_set_size".to_string(), stats.open_set_size as f64);
        dict.insert("closed_set_size".to_string(), stats.closed_set_size as f64);
        dict.insert("via_count".to_string(), stats.via_count as f64);

        // Computed metrics
        if stats.initial_h > 0 && stats.final_g > 0 {
            // Heuristic efficiency: h/g ratio (1.0 = perfect, <1.0 = underestimate, >1.0 = inadmissible)
            dict.insert("heuristic_ratio".to_string(), stats.initial_h as f64 / stats.final_g as f64);
        }
        if stats.path_length > 0 {
            // Expansion ratio: cells expanded / path length (lower = more efficient)
            dict.insert("expansion_ratio".to_string(), stats.cells_expanded as f64 / stats.path_length as f64);
            // Revisit ratio: how often we improved paths
            dict.insert("revisit_ratio".to_string(), stats.cells_revisited as f64 / stats.cells_expanded as f64);
        }
        if stats.cells_expanded > 0 {
            // Skip ratio: duplicate pops from open set
            dict.insert("skip_ratio".to_string(), stats.duplicate_skips as f64 / (stats.cells_expanded + stats.duplicate_skips) as f64);
        }

        dict
    }
}
