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
    diff_pair_spacing: i32,  // P/N spacing in grid units (0 = not a diff pair)
    max_turn_units: i32,  // Max cumulative turn in 45° units before reset (default 6 = 270°)
    gnd_via_perp_offset: i32,  // GND via perpendicular offset from centerline (grid units, 0 = disabled)
    gnd_via_along_offset: i32,  // GND via along-heading offset from signal vias (grid units)
    vertical_attraction_radius: i32,  // Grid units for cross-layer attraction lookup (0 = disabled)
    vertical_attraction_bonus: i32,   // Cost reduction for positions aligned with other-layer tracks
}

#[pymethods]
impl PoseRouter {
    #[new]
    #[pyo3(signature = (via_cost, h_weight, turn_cost, min_radius_grid, via_proximity_cost=10, diff_pair_spacing=0, max_turn_units=4, gnd_via_perp_offset=0, gnd_via_along_offset=0, vertical_attraction_radius=0, vertical_attraction_bonus=0))]
    pub fn new(via_cost: i32, h_weight: f32, turn_cost: i32, min_radius_grid: f64, via_proximity_cost: i32, diff_pair_spacing: i32, max_turn_units: i32, gnd_via_perp_offset: i32, gnd_via_along_offset: i32, vertical_attraction_radius: i32, vertical_attraction_bonus: i32) -> Self {
        // After a via, we need enough straight distance to allow the P/N offset tracks
        // to clear the vias before turning. Use min_radius_grid + 1 for safety margin.
        let base_straight = (min_radius_grid.ceil() as i32 + 1).max(3);
        // When GND vias are enabled, need enough straight distance to clear them.
        // The P/N tracks curve when turning, so we need to go past the GND via's
        // along-position plus a margin (2 grid) before turning is safe.
        // Testing showed +1 causes DRC failures, +2 is the minimum that works.
        let straight_after_via = if gnd_via_along_offset > 0 {
            base_straight.max(gnd_via_along_offset + 2)
        } else {
            base_straight
        };
        Self { via_cost, h_weight, turn_cost, min_radius_grid, via_proximity_cost, straight_after_via, diff_pair_spacing, max_turn_units, gnd_via_perp_offset, gnd_via_along_offset, vertical_attraction_radius, vertical_attraction_bonus }
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
    ///     (path, iterations, gnd_via_directions) where:
    ///     - path is list of (gx, gy, theta_idx, layer) or None
    ///     - gnd_via_directions is list of i8 (1=ahead, -1=behind) for each layer change
    #[pyo3(signature = (obstacles, src_x, src_y, src_layer, src_theta_idx, tgt_x, tgt_y, tgt_layer, tgt_theta_idx, max_iterations, diff_pair_via_spacing=None))]
    pub fn route_pose(
        &self,
        obstacles: &GridObstacleMap,
        src_x: i32, src_y: i32, src_layer: u8, src_theta_idx: u8,
        tgt_x: i32, tgt_y: i32, tgt_layer: u8, tgt_theta_idx: u8,
        max_iterations: u32,
        diff_pair_via_spacing: Option<i32>,
    ) -> (Option<Vec<(i32, i32, u8, u8)>>, u32, Vec<i8>) {
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
        // Value: remaining straight steps required (straight_after_via after via, decrements each step)
        let mut straight_steps_remaining: FxHashMap<u64, i32> = FxHashMap::default();

        // Track consecutive straight steps taken (resets to 0 on turn)
        // Used to require straight approach before via (prevents P/N tracks from curving near each other's vias)
        let mut straight_steps_taken: FxHashMap<u64, i32> = FxHashMap::default();

        // Track cumulative turn angles (in 45° units) to prevent loops
        // Two counters offset by 50 steps, each resets every 100 steps
        // This provides overlapping coverage to catch loops in any ~100 step window
        let mut cumulative_turn_1: FxHashMap<u64, i32> = FxHashMap::default();  // resets at steps 0, 100, 200...
        let mut cumulative_turn_2: FxHashMap<u64, i32> = FxHashMap::default();  // resets at steps 50, 150, 250...

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
        straight_steps_taken.insert(start_key, 0);
        cumulative_turn_1.insert(start_key, 0);
        cumulative_turn_2.insert(start_key, 0);

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
                let gnd_via_dirs = self.compute_gnd_via_directions(obstacles, &path);
                return (Some(path), iterations, gnd_via_dirs);
            }

            closed.insert(current_key);

            // Get current constraint state
            let current_steps = steps_from_source.get(&current_key).copied().unwrap_or(0);
            let current_straight_remaining = straight_steps_remaining.get(&current_key).copied().unwrap_or(0);
            let current_straight_taken = straight_steps_taken.get(&current_key).copied().unwrap_or(0);
            let current_turn_1 = cumulative_turn_1.get(&current_key).copied().unwrap_or(0);
            let current_turn_2 = cumulative_turn_2.get(&current_key).copied().unwrap_or(0);

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
                    let proximity_cost = obstacles.get_stub_proximity_cost(nx, ny)
                        + obstacles.get_layer_proximity_cost(nx, ny, current.layer as usize);
                    let attraction_bonus = obstacles.get_cross_layer_attraction(
                        nx, ny, current.layer as usize,
                        self.vertical_attraction_radius, self.vertical_attraction_bonus);
                    let new_g = g + move_cost + proximity_cost - attraction_bonus;

                    let existing_g = g_costs.get(&neighbor_key).copied().unwrap_or(i32::MAX);
                    if new_g < existing_g {
                        g_costs.insert(neighbor_key, new_g);
                        parents.insert(neighbor_key, current_key);
                        // Update constraint tracking for straight move
                        let new_steps = current_steps + 1;
                        steps_from_source.insert(neighbor_key, new_steps);
                        straight_steps_remaining.insert(neighbor_key, (current_straight_remaining - 1).max(0));
                        straight_steps_taken.insert(neighbor_key, current_straight_taken + 1);
                        // Reset counters at their respective intervals (no turn delta for straight move)
                        let new_turn_1 = if new_steps % 100 == 0 { 0 } else { current_turn_1 };
                        let new_turn_2 = if new_steps % 100 == 50 { 0 } else { current_turn_2 };
                        cumulative_turn_1.insert(neighbor_key, new_turn_1);
                        cumulative_turn_2.insert(neighbor_key, new_turn_2);
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
            // - For diff pairs, limit cumulative turn to prevent loops
            if current_key != start_key && current_straight_remaining <= 0 {
                for delta in [-1i8, 1i8] {
                    let new_steps = current_steps + 1;
                    // Calculate new turn values, resetting at respective intervals
                    let new_turn_1 = if new_steps % 100 == 0 { delta as i32 } else { current_turn_1 + delta as i32 };
                    let new_turn_2 = if new_steps % 100 == 50 { delta as i32 } else { current_turn_2 + delta as i32 };

                    // For diff pairs, check both cumulative turn limits (max_turn_units * 45°)
                    if self.diff_pair_spacing > 0 && (new_turn_1.abs() > self.max_turn_units || new_turn_2.abs() > self.max_turn_units) {
                        continue;  // Would form a loop
                    }

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
                            let proximity_cost = obstacles.get_stub_proximity_cost(nx, ny)
                                + obstacles.get_layer_proximity_cost(nx, ny, current.layer as usize);
                            let attraction_bonus = obstacles.get_cross_layer_attraction(
                                nx, ny, current.layer as usize,
                                self.vertical_attraction_radius, self.vertical_attraction_bonus);
                            let new_g = g + move_cost + self.turn_cost + proximity_cost - attraction_bonus;

                            let existing_g = g_costs.get(&neighbor_key).copied().unwrap_or(i32::MAX);
                            if new_g < existing_g {
                                g_costs.insert(neighbor_key, new_g);
                                parents.insert(neighbor_key, current_key);
                                // Update constraint tracking for turn move
                                steps_from_source.insert(neighbor_key, new_steps);
                                // After a turn, require min_radius_grid straight steps before next turn
                                straight_steps_remaining.insert(neighbor_key, self.min_radius_grid.ceil() as i32);
                                // Reset to 1: this is the first step in the new direction
                                straight_steps_taken.insert(neighbor_key, 1);
                                cumulative_turn_1.insert(neighbor_key, new_turn_1);
                                cumulative_turn_2.insert(neighbor_key, new_turn_2);
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
            // - Must approach via straight for min_radius steps (prevents P/N curving near each other's vias)
            // - After via, must go straight for min_radius steps
            // - If diff_pair_via_spacing is set, check that offset positions are also clear
            let can_place_via = current_steps >= 2
                && current_straight_remaining <= 0
                && current_straight_taken >= self.straight_after_via;

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

                    // Check GND via positions if enabled (gnd_via_perp_offset > 0)
                    // GND vias are placed outside P/N tracks, offset along heading (ahead or behind)
                    // We also check that the P/N track path has sufficient clearance from GND vias.
                    if via_positions_clear && self.gnd_via_perp_offset > 0 {
                        let gnd_p_base_x = current.gx + perp_x * self.gnd_via_perp_offset;
                        let gnd_p_base_y = current.gy + perp_y * self.gnd_via_perp_offset;
                        let gnd_n_base_x = current.gx - perp_x * self.gnd_via_perp_offset;
                        let gnd_n_base_y = current.gy - perp_y * self.gnd_via_perp_offset;

                        // Try ahead first (+heading), then behind (-heading)
                        let ahead_offset_x = dx * self.gnd_via_along_offset;
                        let ahead_offset_y = dy * self.gnd_via_along_offset;

                        let gnd_p_ahead_x = gnd_p_base_x + ahead_offset_x;
                        let gnd_p_ahead_y = gnd_p_base_y + ahead_offset_y;
                        let gnd_n_ahead_x = gnd_n_base_x + ahead_offset_x;
                        let gnd_n_ahead_y = gnd_n_base_y + ahead_offset_y;

                        // For "ahead" placement: need straight continuation after via
                        // Check GND via positions aren't blocked
                        let ahead_clear = !obstacles.is_via_blocked(gnd_p_ahead_x, gnd_p_ahead_y)
                            && !obstacles.is_via_blocked(gnd_n_ahead_x, gnd_n_ahead_y);

                        // For "behind" placement: already had straight approach (enforced by can_place_via)
                        // Check GND via positions aren't blocked
                        let gnd_p_behind_x = gnd_p_base_x - ahead_offset_x;
                        let gnd_p_behind_y = gnd_p_base_y - ahead_offset_y;
                        let gnd_n_behind_x = gnd_n_base_x - ahead_offset_x;
                        let gnd_n_behind_y = gnd_n_base_y - ahead_offset_y;

                        let behind_clear = !obstacles.is_via_blocked(gnd_p_behind_x, gnd_p_behind_y)
                            && !obstacles.is_via_blocked(gnd_n_behind_x, gnd_n_behind_y);

                        // Prefer ahead, but if blocked fall back to behind
                        if !ahead_clear && !behind_clear {
                            // Both ahead and behind blocked - can't place GND vias here
                            via_positions_clear = false;
                        }
                    }
                }
            }

            // Block or penalize vias within stub proximity or BGA proximity (for diff pairs)
            // If via_proximity_cost is 0, block vias in these areas; otherwise multiply via cost
            let in_stub_proximity = obstacles.get_stub_proximity_cost(current.gx, current.gy) > 0;
            let in_bga_proximity = obstacles.is_in_bga_proximity(current.gx, current.gy);
            let in_proximity_zone = in_stub_proximity || in_bga_proximity;

            if via_positions_clear && diff_pair_via_spacing.is_some() && self.via_proximity_cost == 0 {
                if in_proximity_zone {
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
                        // Multiply via cost by via_proximity_cost in stub/BGA proximity zones
                        let via_cost = if in_proximity_zone {
                            self.via_cost * self.via_proximity_cost
                        } else {
                            self.via_cost
                        };
                        let new_g = g + via_cost;

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

        (None, iterations, Vec::new())
    }

    /// Route with frontier analysis - returns blocked cells on failure.
    ///
    /// Same as route_pose but on failure returns the set of blocked cells
    /// that were encountered during the search.
    ///
    /// Returns (path, iterations, blocked_cells, gnd_via_directions) where:
    /// - On success: path is Some, blocked_cells is empty, gnd_via_directions has entries
    /// - On failure: path is None, blocked_cells contains cells that blocked expansion
    #[pyo3(signature = (obstacles, src_x, src_y, src_layer, src_theta_idx, tgt_x, tgt_y, tgt_layer, tgt_theta_idx, max_iterations, diff_pair_via_spacing=None))]
    pub fn route_pose_with_frontier(
        &self,
        obstacles: &GridObstacleMap,
        src_x: i32, src_y: i32, src_layer: u8, src_theta_idx: u8,
        tgt_x: i32, tgt_y: i32, tgt_layer: u8, tgt_theta_idx: u8,
        max_iterations: u32,
        diff_pair_via_spacing: Option<i32>,
    ) -> (Option<Vec<(i32, i32, u8, u8)>>, u32, Vec<(i32, i32, u8)>, Vec<i8>) {
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
        let mut straight_steps_taken: FxHashMap<u64, i32> = FxHashMap::default();
        let mut cumulative_turn_1: FxHashMap<u64, i32> = FxHashMap::default();
        let mut cumulative_turn_2: FxHashMap<u64, i32> = FxHashMap::default();

        let start_key = start.as_key();
        let h = self.dubins_heuristic(&dubins, &start, &goal);
        open_set.push(PoseOpenEntry { f_score: h, g_score: 0, state: start, counter });
        counter += 1;
        g_costs.insert(start_key, 0);
        steps_from_source.insert(start_key, 0);
        straight_steps_remaining.insert(start_key, 0);
        straight_steps_taken.insert(start_key, 0);
        cumulative_turn_1.insert(start_key, 0);
        cumulative_turn_2.insert(start_key, 0);

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
                let gnd_via_dirs = self.compute_gnd_via_directions(obstacles, &path);
                return (Some(path), iterations, Vec::new(), gnd_via_dirs);
            }

            closed.insert(current_key);

            let current_steps = steps_from_source.get(&current_key).copied().unwrap_or(0);
            let current_straight_remaining = straight_steps_remaining.get(&current_key).copied().unwrap_or(0);
            let current_straight_taken = straight_steps_taken.get(&current_key).copied().unwrap_or(0);
            let current_turn_1 = cumulative_turn_1.get(&current_key).copied().unwrap_or(0);
            let current_turn_2 = cumulative_turn_2.get(&current_key).copied().unwrap_or(0);

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
                    let proximity_cost = obstacles.get_stub_proximity_cost(nx, ny)
                        + obstacles.get_layer_proximity_cost(nx, ny, current.layer as usize);
                    let attraction_bonus = obstacles.get_cross_layer_attraction(
                        nx, ny, current.layer as usize,
                        self.vertical_attraction_radius, self.vertical_attraction_bonus);
                    let new_g = g + move_cost + proximity_cost - attraction_bonus;

                    let existing_g = g_costs.get(&neighbor_key).copied().unwrap_or(i32::MAX);
                    if new_g < existing_g {
                        g_costs.insert(neighbor_key, new_g);
                        parents.insert(neighbor_key, current_key);
                        let new_steps = current_steps + 1;
                        steps_from_source.insert(neighbor_key, new_steps);
                        straight_steps_remaining.insert(neighbor_key, (current_straight_remaining - 1).max(0));
                        straight_steps_taken.insert(neighbor_key, current_straight_taken + 1);
                        let new_turn_1 = if new_steps % 100 == 0 { 0 } else { current_turn_1 };
                        let new_turn_2 = if new_steps % 100 == 50 { 0 } else { current_turn_2 };
                        cumulative_turn_1.insert(neighbor_key, new_turn_1);
                        cumulative_turn_2.insert(neighbor_key, new_turn_2);
                        let h = self.dubins_heuristic(&dubins, &neighbor, &goal);
                        open_set.push(PoseOpenEntry { f_score: new_g + h, g_score: new_g, state: neighbor, counter });
                        counter += 1;
                    }
                }
            }

            // 2. Move + turn by ±45°
            if current_key != start_key && current_straight_remaining <= 0 {
                for delta in [-1i8, 1i8] {
                    let new_steps = current_steps + 1;
                    let new_turn_1 = if new_steps % 100 == 0 { delta as i32 } else { current_turn_1 + delta as i32 };
                    let new_turn_2 = if new_steps % 100 == 50 { delta as i32 } else { current_turn_2 + delta as i32 };

                    // For diff pairs, check both cumulative turn limits (max_turn_units * 45°)
                    if self.diff_pair_spacing > 0 && (new_turn_1.abs() > self.max_turn_units || new_turn_2.abs() > self.max_turn_units) {
                        continue;  // Would form a loop
                    }

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
                        let proximity_cost = obstacles.get_stub_proximity_cost(nx, ny)
                            + obstacles.get_layer_proximity_cost(nx, ny, current.layer as usize);
                        let attraction_bonus = obstacles.get_cross_layer_attraction(
                            nx, ny, current.layer as usize,
                            self.vertical_attraction_radius, self.vertical_attraction_bonus);
                        let new_g = g + move_cost + self.turn_cost + proximity_cost - attraction_bonus;

                        let existing_g = g_costs.get(&neighbor_key).copied().unwrap_or(i32::MAX);
                        if new_g < existing_g {
                            g_costs.insert(neighbor_key, new_g);
                            parents.insert(neighbor_key, current_key);
                            steps_from_source.insert(neighbor_key, new_steps);
                            // After a turn, require min_radius_grid straight steps before next turn
                            straight_steps_remaining.insert(neighbor_key, self.min_radius_grid.ceil() as i32);
                            straight_steps_taken.insert(neighbor_key, 1);
                            cumulative_turn_1.insert(neighbor_key, new_turn_1);
                            cumulative_turn_2.insert(neighbor_key, new_turn_2);
                            let h = self.dubins_heuristic(&dubins, &neighbor, &goal);
                            open_set.push(PoseOpenEntry { f_score: new_g + h, g_score: new_g, state: neighbor, counter });
                            counter += 1;
                        }
                    }
                }
            }

            // 3. Via to other layer
            let can_place_via = current_steps >= 2
                && current_straight_remaining <= 0
                && current_straight_taken >= self.straight_after_via;
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

                    // Check GND via positions if enabled (gnd_via_perp_offset > 0)
                    // We also check that the P/N track path has sufficient clearance from GND vias.
                    if via_positions_clear && self.gnd_via_perp_offset > 0 {
                        let gnd_p_base_x = current.gx + perp_x * self.gnd_via_perp_offset;
                        let gnd_p_base_y = current.gy + perp_y * self.gnd_via_perp_offset;
                        let gnd_n_base_x = current.gx - perp_x * self.gnd_via_perp_offset;
                        let gnd_n_base_y = current.gy - perp_y * self.gnd_via_perp_offset;

                        let ahead_offset_x = dx * self.gnd_via_along_offset;
                        let ahead_offset_y = dy * self.gnd_via_along_offset;

                        // Check ahead positions
                        let gnd_p_ahead_x = gnd_p_base_x + ahead_offset_x;
                        let gnd_p_ahead_y = gnd_p_base_y + ahead_offset_y;
                        let gnd_n_ahead_x = gnd_n_base_x + ahead_offset_x;
                        let gnd_n_ahead_y = gnd_n_base_y + ahead_offset_y;

                        let ahead_clear = !obstacles.is_via_blocked(gnd_p_ahead_x, gnd_p_ahead_y)
                            && !obstacles.is_via_blocked(gnd_n_ahead_x, gnd_n_ahead_y);

                        // Check behind positions
                        let gnd_p_behind_x = gnd_p_base_x - ahead_offset_x;
                        let gnd_p_behind_y = gnd_p_base_y - ahead_offset_y;
                        let gnd_n_behind_x = gnd_n_base_x - ahead_offset_x;
                        let gnd_n_behind_y = gnd_n_base_y - ahead_offset_y;

                        let behind_clear = !obstacles.is_via_blocked(gnd_p_behind_x, gnd_p_behind_y)
                            && !obstacles.is_via_blocked(gnd_n_behind_x, gnd_n_behind_y);

                        if !ahead_clear && !behind_clear {
                            // Both blocked - track all GND via positions
                            via_positions_clear = false;
                            for layer in 0..obstacles.num_layers as u8 {
                                tracker.track(gnd_p_ahead_x, gnd_p_ahead_y, layer);
                                tracker.track(gnd_n_ahead_x, gnd_n_ahead_y, layer);
                                tracker.track(gnd_p_behind_x, gnd_p_behind_y, layer);
                                tracker.track(gnd_n_behind_x, gnd_n_behind_y, layer);
                            }
                        }
                    }
                }
            }

            // Block or penalize vias within stub proximity or BGA proximity (for diff pairs)
            let in_stub_proximity = obstacles.get_stub_proximity_cost(current.gx, current.gy) > 0;
            let in_bga_proximity = obstacles.is_in_bga_proximity(current.gx, current.gy);
            let in_proximity_zone = in_stub_proximity || in_bga_proximity;

            if via_positions_clear && diff_pair_via_spacing.is_some() && self.via_proximity_cost == 0 {
                if in_proximity_zone {
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
                        // Multiply via cost by via_proximity_cost in stub/BGA proximity zones
                        let via_cost = if in_proximity_zone {
                            self.via_cost * self.via_proximity_cost
                        } else {
                            self.via_cost
                        };
                        let new_g = g + via_cost;

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

        (None, iterations, tracker.get_blocked(), Vec::new())
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

    /// Compute GND via directions for each layer change in the path.
    /// Returns a Vec<i8> with one entry per layer change: 1 = ahead, -1 = behind.
    /// If gnd_via_perp_offset is 0, returns empty vec.
    fn compute_gnd_via_directions(&self, obstacles: &GridObstacleMap, path: &[(i32, i32, u8, u8)]) -> Vec<i8> {
        if self.gnd_via_perp_offset == 0 || path.len() < 2 {
            return Vec::new();
        }

        let mut directions = Vec::new();

        for i in 0..path.len() - 1 {
            let (gx, gy, theta_idx, layer) = path[i];
            let (_, _, _, next_layer) = path[i + 1];

            // Check for layer change
            if layer != next_layer {
                // Get heading direction from theta_idx
                let (dx, dy) = DIRECTIONS[theta_idx as usize];

                // Perpendicular direction (90° rotation: (-dy, dx))
                let perp_x = -dy;
                let perp_y = dx;

                // GND via base positions (perpendicular offset from centerline)
                let gnd_p_base_x = gx + perp_x * self.gnd_via_perp_offset;
                let gnd_p_base_y = gy + perp_y * self.gnd_via_perp_offset;
                let gnd_n_base_x = gx - perp_x * self.gnd_via_perp_offset;
                let gnd_n_base_y = gy - perp_y * self.gnd_via_perp_offset;

                // Along-heading offsets
                let ahead_offset_x = dx * self.gnd_via_along_offset;
                let ahead_offset_y = dy * self.gnd_via_along_offset;

                // Check ahead positions
                let gnd_p_ahead_x = gnd_p_base_x + ahead_offset_x;
                let gnd_p_ahead_y = gnd_p_base_y + ahead_offset_y;
                let gnd_n_ahead_x = gnd_n_base_x + ahead_offset_x;
                let gnd_n_ahead_y = gnd_n_base_y + ahead_offset_y;

                let ahead_clear = !obstacles.is_via_blocked(gnd_p_ahead_x, gnd_p_ahead_y)
                    && !obstacles.is_via_blocked(gnd_n_ahead_x, gnd_n_ahead_y);

                if ahead_clear {
                    directions.push(1);  // Use ahead
                } else {
                    // Try behind
                    let gnd_p_behind_x = gnd_p_base_x - ahead_offset_x;
                    let gnd_p_behind_y = gnd_p_base_y - ahead_offset_y;
                    let gnd_n_behind_x = gnd_n_base_x - ahead_offset_x;
                    let gnd_n_behind_y = gnd_n_base_y - ahead_offset_y;

                    let behind_clear = !obstacles.is_via_blocked(gnd_p_behind_x, gnd_p_behind_y)
                        && !obstacles.is_via_blocked(gnd_n_behind_x, gnd_n_behind_y);

                    if behind_clear {
                        directions.push(-1);  // Use behind
                    } else {
                        // Both blocked - shouldn't happen if routing succeeded, but default to ahead
                        directions.push(1);
                    }
                }
            }
        }

        directions
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
