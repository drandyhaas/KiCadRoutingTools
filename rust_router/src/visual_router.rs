//! Iterator-based router for visualization and debugging.
//!
//! C1/B1 (issues #387/#386): this used to be a hand-forked copy of the grid
//! A* loop that had drifted from production (missing the A1 negative-sentinel
//! `min_layer_cost` filter, the `layer_forbidden` guards, and the `free_here`
//! via override). It now drives the SAME `GridSearch` core as
//! `route_multi`/`route_with_frontier`, so a `--visualize` session replays
//! the production search instead of a subtly different one.

use pyo3::prelude::*;

use crate::obstacle_map::GridObstacleMap;
use crate::router::{GridRouter, GridSearch, SearchOptions, SearchStep};
use crate::types::{GridState, StatsSink, DEFAULT_TURN_COST};

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

/// Iterator-based router for visualization: owns a production GridRouter for
/// the cost model and steps the shared GridSearch core snapshot by snapshot.
#[pyclass]
pub struct VisualRouter {
    router: GridRouter,
    search: Option<GridSearch>,
    sink: StatsSink,
    // Result
    found: bool,
    final_path: Option<Vec<(i32, i32, u8)>>,
    last_current: Option<GridState>,
}

#[pymethods]
impl VisualRouter {
    #[new]
    #[pyo3(signature = (via_cost, h_weight, turn_cost=None, via_proximity_cost=1, vertical_attraction_radius=0, vertical_attraction_bonus=0, layer_costs=None, proximity_heuristic_cost=0))]
    pub fn new(via_cost: i32, h_weight: f32, turn_cost: Option<i32>, via_proximity_cost: Option<i32>,
               vertical_attraction_radius: i32, vertical_attraction_bonus: i32,
               layer_costs: Option<Vec<i32>>, proximity_heuristic_cost: i32) -> Self {
        // The inner GridRouter carries the same knobs the visualizer always
        // exposed; the ones it never had (direction preferences, bus
        // attraction) stay at their inert defaults.
        let router = GridRouter::new(
            via_cost,
            h_weight,
            Some(turn_cost.unwrap_or(DEFAULT_TURN_COST)),
            Some(via_proximity_cost.unwrap_or(1)),
            vertical_attraction_radius,
            vertical_attraction_bonus,
            layer_costs,
            Some(proximity_heuristic_cost),
            None, // layer_direction_preferences
            0,    // direction_preference_cost
            0,    // attraction_radius
            0,    // attraction_bonus
            0,    // attraction_cross_layer_pct
        );
        Self {
            router,
            search: None,
            sink: StatsSink::default(),
            found: false,
            final_path: None,
            last_current: None,
        }
    }

    /// Initialize the search with sources and targets
    pub fn init(
        &mut self,
        sources: Vec<(i32, i32, u8)>,
        targets: Vec<(i32, i32, u8)>,
        max_iterations: u32,
    ) {
        self.sink = StatsSink::default();
        self.found = false;
        self.final_path = None;
        self.last_current = None;
        // Production defaults: no collinear-via constraint, no via exclusion,
        // no start/end direction constraints, base track margin.
        let opts = SearchOptions::new(false, 0, None, None, 2, 0);
        self.search = Some(GridSearch::new(
            &self.router, sources, targets, max_iterations, opts, &mut self.sink));
    }

    /// Run N iterations of the search, returns snapshot of current state
    pub fn step(&mut self, obstacles: &GridObstacleMap, num_iterations: u32) -> SearchSnapshot {
        if let Some(search) = self.search.as_mut() {
            for _ in 0..num_iterations {
                if self.found || search.is_done() {
                    break;
                }
                match search.step(&self.router, obstacles, &mut self.sink) {
                    SearchStep::Found(goal_idx, _g) => {
                        self.found = true;
                        let path = search.store.reconstruct_path(goal_idx);
                        if let Some(&(x, y, l)) = path.last() {
                            self.last_current = Some(GridState::new(x, y, l));
                        }
                        self.final_path = Some(path);
                        break;
                    }
                    SearchStep::Progress(state) => {
                        self.last_current = Some(state);
                    }
                    SearchStep::Skipped => {}
                    SearchStep::IterationCap | SearchStep::Exhausted => break,
                }
            }
        }
        self.create_snapshot()
    }

    /// Check if search is complete (found path or exhausted)
    pub fn is_done(&self) -> bool {
        self.found || self.search.as_ref().map_or(true, |s| s.is_done())
    }

    /// Get the final path if found
    pub fn get_path(&self) -> Option<Vec<(i32, i32, u8)>> {
        self.final_path.clone()
    }

    /// Get iteration count
    pub fn get_iterations(&self) -> u32 {
        self.search.as_ref().map_or(0, |s| s.iterations)
    }

    /// Get search statistics
    pub fn get_stats(&self) -> std::collections::HashMap<String, u32> {
        let stats = &self.sink.stats;
        let mut dict = std::collections::HashMap::new();
        dict.insert("iterations".to_string(), self.get_iterations());
        dict.insert("cells_expanded".to_string(), stats.cells_expanded);
        dict.insert("duplicate_skips".to_string(), stats.duplicate_skips);
        dict.insert("cells_pushed".to_string(), stats.cells_pushed);
        dict.insert("closed_size".to_string(),
                    self.search.as_ref().map_or(0, |s| s.store.closed_len()));
        dict.insert("open_size".to_string(),
                    self.search.as_ref().map_or(0, |s| s.open_set.len() as u32));
        dict
    }
}

impl VisualRouter {
    fn create_snapshot(&self) -> SearchSnapshot {
        // Sample closed and open sets (limit size for performance)
        const MAX_CELLS: usize = 50000;

        let (iteration, open_count, closed_count, closed_cells, open_cells) =
            match self.search.as_ref() {
                Some(s) => (
                    s.iterations,
                    s.open_set.len(),
                    s.store.closed_len() as usize,
                    s.store.closed_cells(MAX_CELLS),
                    s.store.visited_open_cells(MAX_CELLS),
                ),
                None => (0, 0, 0, Vec::new(), Vec::new()),
            };

        SearchSnapshot {
            iteration,
            current: self.last_current.map(|s| (s.gx, s.gy, s.layer)),
            open_count,
            closed_count,
            found: self.found,
            path: self.final_path.clone(),
            closed_cells,
            open_cells,
        }
    }
}
