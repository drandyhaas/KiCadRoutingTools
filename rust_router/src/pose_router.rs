//! Pose-based A* Router with Dubins Heuristic.
//!
//! State space: (x, y, theta_idx, layer) where theta_idx is 0-7 for 8 directions.
//! Uses Dubins path length as heuristic for better orientation-aware routing.

use pyo3::prelude::*;
use rustc_hash::{FxHashMap, FxHashSet};
use std::collections::BinaryHeap;

use crate::dubins::DubinsCalculator;
use crate::obstacle_map::GridObstacleMap;
use crate::types::{PoseState, PoseOpenEntry, BlockedCellTracker, DIRECTIONS, ORTHO_COST, DIAG_COST};

/// Pose-based A* Router with Dubins heuristic
#[pyclass]
pub struct PoseRouter {
    via_cost: i32,
    h_weight: f32,
    turn_cost: i32,  // Cost per 45° turn
    min_radius_grid: f64,  // Minimum turning radius in grid units
    via_proximity_cost: i32,  // Multiplier for stub proximity cost when placing vias (0 = block vias near stubs)
    straight_after_via: i32,  // Required straight steps after via (derived from min_radius_grid)
}

#[pymethods]
impl PoseRouter {
    #[new]
    #[pyo3(signature = (via_cost, h_weight, turn_cost, min_radius_grid, via_proximity_cost=10))]
    pub fn new(via_cost: i32, h_weight: f32, turn_cost: i32, min_radius_grid: f64, via_proximity_cost: i32) -> Self {
        // After a via, we need enough straight distance to allow the P/N offset tracks
        // to clear the vias before turning. Use min_radius_grid + 1 for safety margin.
        let straight_after_via = (min_radius_grid.ceil() as i32 + 1).max(3);
        Self { via_cost, h_weight, turn_cost, min_radius_grid, via_proximity_cost, straight_after_via }
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
    pub fn route_pose(
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

            // Block or penalize vias within stub proximity radius (for diff pairs)
            // If via_proximity_cost is 0, block vias near stubs; otherwise add cost
            if via_positions_clear && diff_pair_via_spacing.is_some() && self.via_proximity_cost == 0 {
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
                        // Apply via proximity cost (multiplier on stub proximity cost)
                        let proximity_cost = obstacles.get_stub_proximity_cost(current.gx, current.gy) * self.via_proximity_cost;
                        let new_g = g + self.via_cost + proximity_cost;

                        let existing_g = g_costs.get(&neighbor_key).copied().unwrap_or(i32::MAX);
                        if new_g < existing_g {
                            g_costs.insert(neighbor_key, new_g);
                            parents.insert(neighbor_key, current_key);
                            // Via doesn't count as a step, but sets straight requirement
                            steps_from_source.insert(neighbor_key, current_steps);
                            straight_steps_remaining.insert(neighbor_key, self.straight_after_via);
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

    /// Route with frontier analysis - returns blocked cells on failure.
    ///
    /// Same as route_pose but on failure returns the set of blocked cells
    /// that were encountered during the search.
    ///
    /// Returns (path, iterations, blocked_cells) where:
    /// - On success: path is Some, blocked_cells is empty
    /// - On failure: path is None, blocked_cells contains cells that blocked expansion
    #[pyo3(signature = (obstacles, src_x, src_y, src_layer, src_theta_idx, tgt_x, tgt_y, tgt_layer, tgt_theta_idx, max_iterations, diff_pair_via_spacing=None))]
    pub fn route_pose_with_frontier(
        &self,
        obstacles: &GridObstacleMap,
        src_x: i32, src_y: i32, src_layer: u8, src_theta_idx: u8,
        tgt_x: i32, tgt_y: i32, tgt_layer: u8, tgt_theta_idx: u8,
        max_iterations: u32,
        diff_pair_via_spacing: Option<i32>,
    ) -> (Option<Vec<(i32, i32, u8, u8)>>, u32, Vec<(i32, i32, u8)>) {
        let mut tracker = BlockedCellTracker::new();
        let dubins = DubinsCalculator::new(self.min_radius_grid);

        let start = PoseState::new(src_x, src_y, src_theta_idx, src_layer);
        let goal = PoseState::new(tgt_x, tgt_y, tgt_theta_idx, tgt_layer);
        let goal_key = goal.as_key();

        let mut open_set = BinaryHeap::new();
        let mut g_costs: FxHashMap<u64, i32> = FxHashMap::default();
        let mut parents: FxHashMap<u64, u64> = FxHashMap::default();
        let mut closed: FxHashSet<u64> = FxHashSet::default();
        let mut counter: u32 = 0;
        let mut steps_from_source: FxHashMap<u64, i32> = FxHashMap::default();
        let mut straight_steps_remaining: FxHashMap<u64, i32> = FxHashMap::default();

        let start_key = start.as_key();
        let h = self.dubins_heuristic(&dubins, &start, &goal);
        open_set.push(PoseOpenEntry { f_score: h, g_score: 0, state: start, counter });
        counter += 1;
        g_costs.insert(start_key, 0);
        steps_from_source.insert(start_key, 0);
        straight_steps_remaining.insert(start_key, 0);

        let mut iterations: u32 = 0;

        while let Some(current_entry) = open_set.pop() {
            if iterations >= max_iterations { break; }
            iterations += 1;

            let current = current_entry.state;
            let current_key = current.as_key();
            let g = current_entry.g_score;

            if closed.contains(&current_key) { continue; }

            if current_key == goal_key {
                let path = self.reconstruct_pose_path(&parents, current_key);
                return (Some(path), iterations, Vec::new());
            }

            closed.insert(current_key);

            let current_steps = steps_from_source.get(&current_key).copied().unwrap_or(0);
            let current_straight_remaining = straight_steps_remaining.get(&current_key).copied().unwrap_or(0);

            // 1. Move forward in current direction
            let (dx, dy) = current.direction();
            let nx = current.gx + dx;
            let ny = current.gy + dy;

            if obstacles.is_blocked(nx, ny, current.layer as usize) {
                tracker.track(nx, ny, current.layer);
            } else {
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
                        steps_from_source.insert(neighbor_key, current_steps + 1);
                        straight_steps_remaining.insert(neighbor_key, (current_straight_remaining - 1).max(0));
                        let h = self.dubins_heuristic(&dubins, &neighbor, &goal);
                        open_set.push(PoseOpenEntry { f_score: new_g + h, g_score: new_g, state: neighbor, counter });
                        counter += 1;
                    }
                }
            }

            // 2. Move + turn by ±45°
            if current_key != start_key && current_straight_remaining <= 0 {
                for delta in [-1i8, 1i8] {
                    let new_theta = ((current.theta_idx as i8 + delta + 8) % 8) as u8;
                    let (dx, dy) = DIRECTIONS[new_theta as usize];
                    let nx = current.gx + dx;
                    let ny = current.gy + dy;

                    if obstacles.is_blocked(nx, ny, current.layer as usize) {
                        tracker.track(nx, ny, current.layer);
                        continue;
                    }

                    let neighbor = PoseState::new(nx, ny, new_theta, current.layer);
                    let neighbor_key = neighbor.as_key();

                    if !closed.contains(&neighbor_key) {
                        let move_cost = if dx != 0 && dy != 0 { DIAG_COST } else { ORTHO_COST };
                        let proximity_cost = obstacles.get_stub_proximity_cost(nx, ny);
                        let new_g = g + move_cost + self.turn_cost + proximity_cost;

                        let existing_g = g_costs.get(&neighbor_key).copied().unwrap_or(i32::MAX);
                        if new_g < existing_g {
                            g_costs.insert(neighbor_key, new_g);
                            parents.insert(neighbor_key, current_key);
                            steps_from_source.insert(neighbor_key, current_steps + 1);
                            straight_steps_remaining.insert(neighbor_key, 0);
                            let h = self.dubins_heuristic(&dubins, &neighbor, &goal);
                            open_set.push(PoseOpenEntry { f_score: new_g + h, g_score: new_g, state: neighbor, counter });
                            counter += 1;
                        }
                    }
                }
            }

            // 3. Via to other layer
            let can_place_via = current_steps >= 2 && current_straight_remaining <= 0;
            let mut via_positions_clear = !obstacles.is_via_blocked(current.gx, current.gy);

            // Track via blocking for all layers
            if !via_positions_clear {
                for layer in 0..obstacles.num_layers as u8 {
                    if layer != current.layer {
                        tracker.track(current.gx, current.gy, layer);
                    }
                }
            }

            if via_positions_clear {
                if let Some(spacing) = diff_pair_via_spacing {
                    let (dx, dy) = current.direction();
                    let perp_x = -dy;
                    let perp_y = dx;
                    let p_via_x = current.gx + perp_x * spacing;
                    let p_via_y = current.gy + perp_y * spacing;
                    let n_via_x = current.gx - perp_x * spacing;
                    let n_via_y = current.gy - perp_y * spacing;
                    let p_blocked = obstacles.is_via_blocked(p_via_x, p_via_y);
                    let n_blocked = obstacles.is_via_blocked(n_via_x, n_via_y);
                    if p_blocked || n_blocked {
                        via_positions_clear = false;
                        // Track the blocked via offset positions on all layers
                        for layer in 0..obstacles.num_layers as u8 {
                            if p_blocked {
                                tracker.track(p_via_x, p_via_y, layer);
                            }
                            if n_blocked {
                                tracker.track(n_via_x, n_via_y, layer);
                            }
                        }
                    }
                }
            }

            // Block or penalize vias within stub proximity radius (for diff pairs)
            if via_positions_clear && diff_pair_via_spacing.is_some() && self.via_proximity_cost == 0 {
                if obstacles.get_stub_proximity_cost(current.gx, current.gy) > 0 {
                    via_positions_clear = false;
                }
            }

            if can_place_via && via_positions_clear {
                for layer in 0..obstacles.num_layers as u8 {
                    if layer == current.layer { continue; }

                    if obstacles.is_blocked(current.gx, current.gy, layer as usize) {
                        tracker.track(current.gx, current.gy, layer);
                        continue;
                    }

                    let neighbor = PoseState::new(current.gx, current.gy, current.theta_idx, layer);
                    let neighbor_key = neighbor.as_key();

                    if !closed.contains(&neighbor_key) {
                        // Apply via proximity cost (multiplier on stub proximity cost)
                        let proximity_cost = obstacles.get_stub_proximity_cost(current.gx, current.gy) * self.via_proximity_cost;
                        let new_g = g + self.via_cost + proximity_cost;

                        let existing_g = g_costs.get(&neighbor_key).copied().unwrap_or(i32::MAX);
                        if new_g < existing_g {
                            g_costs.insert(neighbor_key, new_g);
                            parents.insert(neighbor_key, current_key);
                            steps_from_source.insert(neighbor_key, current_steps);
                            straight_steps_remaining.insert(neighbor_key, self.straight_after_via);
                            let h = self.dubins_heuristic(&dubins, &neighbor, &goal);
                            open_set.push(PoseOpenEntry { f_score: new_g + h, g_score: new_g, state: neighbor, counter });
                            counter += 1;
                        }
                    }
                }
            }
        }

        (None, iterations, tracker.get_blocked())
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
