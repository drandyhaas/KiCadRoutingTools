//! Grid-based A* router implementation.

use pyo3::prelude::*;
use rustc_hash::{FxHashMap, FxHashSet};
use std::collections::BinaryHeap;

use crate::obstacle_map::GridObstacleMap;
use crate::types::{GridState, OpenEntry, BlockedCellTracker, DIRECTIONS, ORTHO_COST, DIAG_COST};

/// Grid A* Router
#[pyclass]
pub struct GridRouter {
    via_cost: i32,
    h_weight: f32,
}

#[pymethods]
impl GridRouter {
    #[new]
    pub fn new(via_cost: i32, h_weight: f32) -> Self {
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

    /// Route with frontier analysis - returns blocked cells on failure.
    ///
    /// Same as route_multi but on failure returns the set of blocked cells
    /// that were encountered during the search. This helps diagnose which
    /// obstacles are preventing the route.
    ///
    /// Returns (path, iterations, blocked_cells) where:
    /// - On success: path is Some, blocked_cells is empty
    /// - On failure: path is None, blocked_cells contains cells that blocked expansion
    #[pyo3(signature = (obstacles, sources, targets, max_iterations, collinear_vias=false, via_exclusion_radius=0, start_direction=None, end_direction=None, direction_steps=2))]
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

        for (gx, gy, layer) in sources {
            let state = GridState::new(gx, gy, layer);
            let key = state.as_key();
            let h = self.heuristic_to_targets(&state, &target_states);
            open_set.push(OpenEntry { f_score: h, g_score: 0, state, counter });
            counter += 1;
            g_costs.insert(key, 0);
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

                if obstacles.is_blocked(ngx, ngy, current.layer as usize) {
                    tracker.track(ngx, ngy, current.layer);
                    continue;
                }

                if check_via_exclusion(ngx, ngy, current.gx, current.gy, &current_vias, via_exclusion_radius) {
                    continue;
                }

                let neighbor = GridState::new(ngx, ngy, current.layer);
                let neighbor_key = neighbor.as_key();

                if closed.contains(&neighbor_key) { continue; }

                let move_cost = if dx != 0 && dy != 0 { DIAG_COST } else { ORTHO_COST };
                let proximity_cost = obstacles.get_stub_proximity_cost(ngx, ngy);
                let new_g = g + move_cost + proximity_cost;

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

                    if obstacles.is_blocked(current.gx, current.gy, layer as usize) {
                        tracker.track(current.gx, current.gy, layer);
                        continue;
                    }

                    let neighbor = GridState::new(current.gx, current.gy, layer);
                    let neighbor_key = neighbor.as_key();

                    if closed.contains(&neighbor_key) { continue; }

                    let proximity_cost = obstacles.get_stub_proximity_cost(current.gx, current.gy) * 2;
                    let new_g = g + self.via_cost + proximity_cost;

                    let existing_g = g_costs.get(&neighbor_key).copied().unwrap_or(i32::MAX);
                    if new_g < existing_g {
                        g_costs.insert(neighbor_key, new_g);
                        parents.insert(neighbor_key, current_key);
                        if via_exclusion_radius > 0 {
                            let mut new_vias = current_vias.clone();
                            new_vias.push((current.gx, current.gy));
                            path_vias.insert(neighbor_key, new_vias);
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
}
