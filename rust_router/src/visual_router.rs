//! Iterator-based router for visualization and debugging.

use pyo3::prelude::*;
use rustc_hash::{FxHashMap, FxHashSet};
use std::collections::BinaryHeap;

use crate::obstacle_map::GridObstacleMap;
use crate::types::{GridState, OpenEntry, DIRECTIONS, ORTHO_COST, DIAG_COST, DEFAULT_TURN_COST};

/// Search state snapshot for visualization
#[pyclass]
#[derive(Clone)]
pub struct SearchSnapshot {
    #[pyo3(get)]
    pub iteration: u32,
    #[pyo3(get)]
    pub current: Option<(i32, i32, u8)>,
    #[pyo3(get)]
    pub open_count: usize,
    #[pyo3(get)]
    pub closed_count: usize,
    #[pyo3(get)]
    pub found: bool,
    #[pyo3(get)]
    pub path: Option<Vec<(i32, i32, u8)>>,
    /// Closed set cells for visualization (sampled if too large)
    #[pyo3(get)]
    pub closed_cells: Vec<(i32, i32, u8)>,
    /// Open set cells for visualization (sampled if too large)
    #[pyo3(get)]
    pub open_cells: Vec<(i32, i32, u8)>,
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
pub struct VisualRouter {
    via_cost: i32,
    h_weight: f32,
    turn_cost: i32,
    via_proximity_cost: i32,  // Multiplier for stub proximity cost when placing vias
    vertical_attraction_radius: i32,  // Grid units for cross-layer attraction lookup (0 = disabled)
    vertical_attraction_bonus: i32,   // Cost reduction for positions aligned with other-layer tracks
    layer_costs: Vec<i32>,  // Per-layer cost multipliers (1000 = 1.0x, 1500 = 1.5x penalty)
    proximity_heuristic_cost: i32,  // Expected proximity cost per grid step (added to heuristic)
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
    // Stats
    cells_expanded: u32,
    duplicate_skips: u32,
    cells_pushed: u32,
}

#[pymethods]
impl VisualRouter {
    #[new]
    #[pyo3(signature = (via_cost, h_weight, turn_cost=None, via_proximity_cost=1, vertical_attraction_radius=0, vertical_attraction_bonus=0, layer_costs=None, proximity_heuristic_cost=0))]
    pub fn new(via_cost: i32, h_weight: f32, turn_cost: Option<i32>, via_proximity_cost: Option<i32>,
               vertical_attraction_radius: i32, vertical_attraction_bonus: i32,
               layer_costs: Option<Vec<i32>>, proximity_heuristic_cost: i32) -> Self {
        Self {
            via_cost,
            h_weight,
            turn_cost: turn_cost.unwrap_or(DEFAULT_TURN_COST),
            via_proximity_cost: via_proximity_cost.unwrap_or(1),
            vertical_attraction_radius,
            vertical_attraction_bonus,
            layer_costs: layer_costs.unwrap_or_default(),
            proximity_heuristic_cost,
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
            cells_expanded: 0,
            duplicate_skips: 0,
            cells_pushed: 0,
        }
    }

    /// Initialize the search with sources and targets
    pub fn init(
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
        self.cells_expanded = 0;
        self.duplicate_skips = 0;
        self.cells_pushed = 0;

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
        // Find minimum layer cost for initial g penalty
        let min_layer_cost = self.layer_costs.iter().copied().min().unwrap_or(1000);

        for (gx, gy, layer) in sources {
            let state = GridState::new(gx, gy, layer);
            let key = state.as_key();
            // Penalize starting on expensive layers
            let layer_cost = self.layer_costs.get(layer as usize).copied().unwrap_or(1000);
            let initial_g = layer_cost - min_layer_cost;
            let h = self.heuristic_to_targets(&state);
            self.open_set.push(OpenEntry {
                f_score: initial_g + h,
                g_score: initial_g,
                state,
                counter: self.counter,
            });
            self.counter += 1;
            self.g_costs.insert(key, initial_g);
        }
    }

    /// Run N iterations of the search, returns snapshot of current state
    pub fn step(&mut self, obstacles: &GridObstacleMap, num_iterations: u32) -> SearchSnapshot {
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
                self.duplicate_skips += 1;
                continue;
            }

            // Check if reached target (before adding to closed, matching GridRouter)
            if self.target_set.contains(&current_key) {
                self.cells_expanded += 1;
                current_node = Some(current);
                self.found = true;
                self.final_path = Some(self.reconstruct_path(current_key));
                break;
            }

            self.closed.insert(current_key);
            self.cells_expanded += 1;
            current_node = Some(current);

            // Get previous direction (parent -> current) for turn cost calculation
            let prev_direction: Option<(i32, i32)> = self.parents.get(&current_key).map(|&parent_key| {
                let py = ((parent_key >> 8) & 0xFFFFF) as i32;
                let px = ((parent_key >> 28) & 0xFFFFF) as i32;
                let px = if px & 0x80000 != 0 { px | !0xFFFFF_i32 } else { px };
                let py = if py & 0x80000 != 0 { py | !0xFFFFF_i32 } else { py };
                let pdx = current.gx - px;
                let pdy = current.gy - py;
                // Normalize to unit direction (handle vias where position is same)
                if pdx == 0 && pdy == 0 {
                    (0, 0) // Via (same position), no direction
                } else {
                    (pdx.signum(), pdy.signum())
                }
            });

            // Expand neighbors - 8 directions
            for (dx, dy) in DIRECTIONS {
                let ngx = current.gx + dx;
                let ngy = current.gy + dy;

                if obstacles.is_blocked_with_margin(ngx, ngy, current.layer as usize, 0) {
                    continue;
                }

                let neighbor = GridState::new(ngx, ngy, current.layer);
                let neighbor_key = neighbor.as_key();

                if self.closed.contains(&neighbor_key) {
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
                let new_g = g + move_cost + turn_cost + proximity_cost - attraction_bonus;

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
                    self.cells_pushed += 1;
                }
            }

            // Try via to other layers
            if !obstacles.is_via_blocked(current.gx, current.gy) {
                for layer in 0..obstacles.num_layers as u8 {
                    if layer == current.layer {
                        continue;
                    }

                    // Check if destination layer is blocked at this position
                    if obstacles.is_blocked_with_margin(current.gx, current.gy, layer as usize, 0) {
                        continue;
                    }

                    let neighbor = GridState::new(current.gx, current.gy, layer);
                    let neighbor_key = neighbor.as_key();

                    if self.closed.contains(&neighbor_key) {
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
                        self.cells_pushed += 1;
                    }
                }
            }
        }

        // Build snapshot
        self.create_snapshot(current_node)
    }

    /// Check if search is complete (found path or exhausted)
    pub fn is_done(&self) -> bool {
        self.found || self.open_set.is_empty() || self.iterations >= self.max_iterations
    }

    /// Get the final path if found
    pub fn get_path(&self) -> Option<Vec<(i32, i32, u8)>> {
        self.final_path.clone()
    }

    /// Get iteration count
    pub fn get_iterations(&self) -> u32 {
        self.iterations
    }

    /// Get search statistics
    pub fn get_stats(&self) -> std::collections::HashMap<String, u32> {
        let mut stats = std::collections::HashMap::new();
        stats.insert("iterations".to_string(), self.iterations);
        stats.insert("cells_expanded".to_string(), self.cells_expanded);
        stats.insert("duplicate_skips".to_string(), self.duplicate_skips);
        stats.insert("cells_pushed".to_string(), self.cells_pushed);
        stats.insert("closed_size".to_string(), self.closed.len() as u32);
        stats.insert("open_size".to_string(), self.open_set.len() as u32);
        stats
    }
}

impl VisualRouter {
    /// Octile distance heuristic to nearest target
    /// Uses minimum layer cost to remain admissible (never overestimate)
    fn heuristic_to_targets(&self, state: &GridState) -> i32 {
        // Use minimum layer cost for admissibility
        let min_layer_cost = self.layer_costs.iter().copied().min().unwrap_or(1000);

        let mut min_h = i32::MAX;
        for target in &self.target_states {
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
