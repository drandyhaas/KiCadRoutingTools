//! Grid-based A* router implementation.

use pyo3::prelude::*;
use rustc_hash::{FxHashMap, FxHashSet};
use std::collections::BinaryHeap;

use crate::obstacle_map::GridObstacleMap;
use crate::types::{GridState, OpenEntry, RouteStats, SearchSink, StatsSink, FrontierSink, DIRECTIONS, ORTHO_COST, DIAG_COST, DEFAULT_TURN_COST};

/// Direction octant 0-7 for a unit step (soft-knobs N5: angle-proportional
/// turn cost -- octant delta * 45 degrees is the turn angle).
#[inline]
fn octant_index(dx: i32, dy: i32) -> i32 {
    match (dx.signum(), dy.signum()) {
        (1, 0) => 0,
        (1, 1) => 1,
        (0, 1) => 2,
        (-1, 1) => 3,
        (-1, 0) => 4,
        (-1, -1) => 5,
        (0, -1) => 6,
        (1, -1) => 7,
        _ => 0,
    }
}

/// S1-B (issue #384): tiled flat-array node store. Replaces the per-node
/// FxHashMap<u64, NodeState> (S1-A) with 64x64-cell tiles per layer: a node
/// lookup is one SMALL hashmap probe per tile plus direct array indexing, and
/// the per-node record is 12 bytes (g: i32, parent: u32 node index with the
/// S1-A closed bit folded into bit 31, steps: i32). Node index = tile index *
/// 4096 + slot, so a u32 addresses ~524k tiles (~2.1G cells) -- far beyond any
/// search. The store is fresh per search; memory scales with the explored
/// area (12B/node vs ~40-50B/node hashed) and is dropped when the call ends.
const TILE_BITS: i32 = 6;
const TILE_SIZE: i32 = 1 << TILE_BITS; // 64
const TILE_MASK: i32 = TILE_SIZE - 1;
const TILE_CELLS: u32 = (TILE_SIZE * TILE_SIZE) as u32; // 4096

/// Closed flag lives in bit 31 of NodeSlot.parent (S1-A fold, now on the u32).
const CLOSED_BIT32: u32 = 1 << 31;
const PARENT_MASK32: u32 = !CLOSED_BIT32;
/// Sentinel parent for source / unvisited nodes; max value inside PARENT_MASK32.
const NO_PARENT32: u32 = PARENT_MASK32;

#[derive(Clone, Copy)]
struct NodeSlot {
    g: i32,      // i32::MAX = unvisited
    parent: u32, // bit 31 = closed; low 31 bits = parent node index or NO_PARENT32
    steps: i32,
}

const EMPTY_SLOT: NodeSlot = NodeSlot { g: i32::MAX, parent: NO_PARENT32, steps: 0 };

struct Tile {
    base_x: i32, // cell coords of slot (0, 0)
    base_y: i32,
    layer: u8,
    slots: Box<[NodeSlot]>, // TILE_CELLS
}

/// Per-search node store. Semantics identical to the previous NodeMap:
/// g()==MAX means unvisited, parent()==None means source-or-unvisited, the
/// closed bit is only ever set on popped nodes and never relaxed away
/// (call sites gate every relax on !closed).
pub(crate) struct NodeStore {
    tiles: Vec<Tile>,
    index: FxHashMap<u64, u32>, // tile key -> index into tiles
    closed_count: u32,
}

impl NodeStore {
    fn new() -> Self {
        Self { tiles: Vec::new(), index: FxHashMap::default(), closed_count: 0 }
    }

    /// Tile hash key: the tile coordinates reuse GridState's 20/20/8 packing
    /// (tile coords are cell coords >> 6, so they always fit).
    #[inline]
    fn tile_key(gx: i32, gy: i32, layer: u8) -> u64 {
        GridState::new(gx >> TILE_BITS, gy >> TILE_BITS, layer).as_key()
    }

    /// Slot within a tile (arithmetic shift + mask handle negatives: floor
    /// division semantics match between >> and &).
    #[inline]
    fn slot_of(gx: i32, gy: i32) -> u32 {
        (((gy & TILE_MASK) << TILE_BITS) | (gx & TILE_MASK)) as u32
    }

    /// Node index for a position, allocating (and default-filling) its tile on
    /// first touch.
    #[inline]
    fn ensure_index(&mut self, gx: i32, gy: i32, layer: u8) -> u32 {
        let key = Self::tile_key(gx, gy, layer);
        let tile_idx = match self.index.get(&key) {
            Some(&t) => t,
            None => {
                let t = self.tiles.len() as u32;
                self.tiles.push(Tile {
                    base_x: (gx >> TILE_BITS) << TILE_BITS,
                    base_y: (gy >> TILE_BITS) << TILE_BITS,
                    layer,
                    slots: vec![EMPTY_SLOT; TILE_CELLS as usize].into_boxed_slice(),
                });
                self.index.insert(key, t);
                t
            }
        };
        tile_idx * TILE_CELLS + Self::slot_of(gx, gy)
    }

    /// (g, closed) at a position; (i32::MAX, false) when unvisited. The one
    /// combined read the neighbor loops need (old: closed.contains + g_costs.get).
    #[inline]
    fn g_closed_at(&self, gx: i32, gy: i32, layer: u8) -> (i32, bool) {
        match self.index.get(&Self::tile_key(gx, gy, layer)) {
            Some(&t) => {
                let s = &self.tiles[t as usize].slots[Self::slot_of(gx, gy) as usize];
                (s.g, s.parent & CLOSED_BIT32 != 0)
            }
            None => (i32::MAX, false),
        }
    }

    #[inline]
    fn slot(&self, idx: u32) -> &NodeSlot {
        &self.tiles[(idx / TILE_CELLS) as usize].slots[(idx % TILE_CELLS) as usize]
    }

    #[inline]
    fn slot_mut(&mut self, idx: u32) -> &mut NodeSlot {
        &mut self.tiles[(idx / TILE_CELLS) as usize].slots[(idx % TILE_CELLS) as usize]
    }

    /// Cell coordinates of a node index (path reconstruction / parent walks).
    #[inline]
    fn coords(&self, idx: u32) -> (i32, i32, u8) {
        let tile = &self.tiles[(idx / TILE_CELLS) as usize];
        let s = (idx % TILE_CELLS) as i32;
        (tile.base_x + (s & TILE_MASK), tile.base_y + (s >> TILE_BITS), tile.layer)
    }

    /// The node's parent index, or None for a source / unvisited node.
    #[inline]
    fn parent(&self, idx: u32) -> Option<u32> {
        let p = self.slot(idx).parent & PARENT_MASK32;
        if p != NO_PARENT32 { Some(p) } else { None }
    }

    #[inline]
    fn steps_of(&self, idx: u32) -> i32 {
        self.slot(idx).steps
    }

    #[inline]
    fn is_closed(&self, idx: u32) -> bool {
        self.slot(idx).parent & CLOSED_BIT32 != 0
    }

    /// Matches the old `closed.insert(key)`; count kept for RouteStats.
    #[inline]
    fn set_closed(&mut self, idx: u32) {
        self.slot_mut(idx).parent |= CLOSED_BIT32;
        self.closed_count += 1;
    }

    #[inline]
    pub(crate) fn closed_len(&self) -> u32 {
        self.closed_count
    }

    /// Source cell: g set, steps=0, no parent.
    #[inline]
    fn set_source(&mut self, gx: i32, gy: i32, layer: u8, g: i32) {
        let idx = self.ensure_index(gx, gy, layer);
        *self.slot_mut(idx) = NodeSlot { g, parent: NO_PARENT32, steps: 0 };
    }

    /// Relax a node: g + parent + steps written together. Never called on a
    /// closed node (call sites check the closed flag first).
    #[inline]
    fn relax(&mut self, gx: i32, gy: i32, layer: u8, g: i32, parent_idx: u32, steps: i32) {
        let idx = self.ensure_index(gx, gy, layer);
        *self.slot_mut(idx) = NodeSlot { g, parent: parent_idx, steps };
    }

    /// Reconstruct the path by following parent indices back to the source
    /// (whose parent is NO_PARENT32 -> None -> loop terminates).
    pub(crate) fn reconstruct_path(&self, goal_idx: u32) -> Vec<(i32, i32, u8)> {
        let mut path = Vec::new();
        let mut idx = goal_idx;
        loop {
            path.push(self.coords(idx));
            match self.parent(idx) {
                Some(parent_idx) => idx = parent_idx,
                None => break,
            }
        }
        path.reverse();
        path
    }

    /// All closed cells, up to `limit` (visualizer snapshots only).
    pub(crate) fn closed_cells(&self, limit: usize) -> Vec<(i32, i32, u8)> {
        let mut out = Vec::new();
        'tiles: for tile in &self.tiles {
            for (i, slot) in tile.slots.iter().enumerate() {
                if slot.parent & CLOSED_BIT32 != 0 {
                    let i = i as i32;
                    out.push((tile.base_x + (i & TILE_MASK), tile.base_y + (i >> TILE_BITS), tile.layer));
                    if out.len() >= limit {
                        break 'tiles;
                    }
                }
            }
        }
        out
    }

    /// All visited-but-not-closed cells, up to `limit` (visualizer snapshots
    /// only; the old visualizer's "open" dump was g_costs keys minus closed).
    pub(crate) fn visited_open_cells(&self, limit: usize) -> Vec<(i32, i32, u8)> {
        let mut out = Vec::new();
        'tiles: for tile in &self.tiles {
            for (i, slot) in tile.slots.iter().enumerate() {
                if slot.g != i32::MAX && slot.parent & CLOSED_BIT32 == 0 {
                    let i = i as i32;
                    out.push((tile.base_x + (i & TILE_MASK), tile.base_y + (i >> TILE_BITS), tile.layer));
                    if out.len() >= limit {
                        break 'tiles;
                    }
                }
            }
        }
        out
    }
}


/// Outcome of one GridSearch step (one open-set pop).
pub(crate) enum SearchStep {
    /// A node was expanded normally (payload: the expanded state, for the
    /// visualizer's cursor; run-to-completion callers ignore it).
    Progress(GridState),
    /// The pop was a duplicate (already closed) or a target reached from a
    /// rejected arrival direction; nothing was expanded.
    Skipped,
    /// A target was reached: (goal node index, final g).
    Found(u32, i32),
    /// The iteration budget is exhausted (the popped entry is discarded,
    /// matching the historical pop-then-check loop shape).
    IterationCap,
    /// The open set ran dry.
    Exhausted,
}

/// Wide-track search margin (issue #156): the extra half-width, in FRACTIONAL
/// grid cells, that the swept-capsule check reserves beyond what the obstacle
/// map already stamped. Accepted from Python as a scalar (int/float, uniform
/// across layers -- power nets) or a per-layer list of floats (impedance
/// routing, where the layer stackup gives each layer its own width).
#[derive(FromPyObject, Clone)]
pub enum TrackMarginArg {
    Scalar(f64),
    PerLayer(Vec<f64>),
}

impl TrackMarginArg {
    /// Margin (grid cells) to reserve on `layer`. A per-layer list that is
    /// shorter than the layer count yields 0.0 for the missing layers.
    #[inline]
    pub(crate) fn at(&self, layer: usize) -> f64 {
        match self {
            TrackMarginArg::Scalar(m) => *m,
            TrackMarginArg::PerLayer(v) => v.get(layer).copied().unwrap_or(0.0),
        }
    }
}

/// Per-search options (route_multi's optional arguments, pre-normalized).
pub(crate) struct SearchOptions {
    collinear_vias: bool,
    via_exclusion_radius: i32,
    norm_start_dir: Option<(i32, i32)>,
    norm_end_dir: Option<(f64, f64)>,
    direction_steps: i32,
    track_margin: TrackMarginArg,
}

impl SearchOptions {
    pub(crate) fn new(
        collinear_vias: bool,
        via_exclusion_radius: i32,
        start_direction: Option<(i32, i32)>,
        end_direction: Option<(f64, f64)>,
        direction_steps: i32,
        track_margin: TrackMarginArg,
    ) -> Self {
        // Normalize start direction if provided
        let norm_start_dir = start_direction.map(|(dx, dy)| (dx.signum(), dy.signum()));
        // Normalize end direction if provided (direction we want to arrive FROM);
        // this is a continuous (f64, f64) unit vector
        let norm_end_dir = end_direction.map(|(dx, dy)| {
            let len = (dx * dx + dy * dy).sqrt();
            if len > 0.0 { (dx / len, dy / len) } else { (0.0, 0.0) }
        });
        Self {
            collinear_vias,
            via_exclusion_radius,
            norm_start_dir,
            norm_end_dir,
            direction_steps,
            track_margin,
        }
    }
}

/// Check if position (nx, ny) would violate via exclusion: true if blocked
/// (too close to one of the path's own vias and not moving away from it).
/// C1: was a closure duplicated verbatim in route_multi and route_with_frontier.
fn check_via_exclusion(nx: i32, ny: i32, current_gx: i32, current_gy: i32,
                       vias: &[(i32, i32)], radius: i32) -> bool {
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
        // If we're inside the radius, only allow moves that increase distance from
        // the via - this prevents perpendicular drift within the exclusion zone
        if dist_to_current <= radius && dist_to_neighbor <= dist_to_current {
            if dist_to_neighbor < dist_to_current {
                // Moving closer to via - block
                return true;
            }
            // Same distance - block perpendicular moves deep inside the zone
            if dist_to_neighbor == dist_to_current && dist_to_current > 0 {
                let dx_from_via = (nx - vx).abs() - (current_gx - vx).abs();
                let dy_from_via = (ny - vy).abs() - (current_gy - vy).abs();
                if (dx_from_via > 0 && dy_from_via < 0) || (dx_from_via < 0 && dy_from_via > 0) {
                    if dist_to_current <= radius / 2 {
                        return true;
                    }
                }
            }
        }
    }
    false
}

/// C1 (issue #387): THE grid A* search core. route_multi, route_with_frontier
/// and the visualizer all drive this one step() loop with different sinks, so
/// there is exactly one copy of the expansion logic (the visualizer fork had
/// drifted -- B1 in #386). step() pops and processes ONE open-set entry;
/// run-to-completion callers just loop it.
/// S3-b (#385): O(1) admissible replacement for the O(targets) exact
/// heuristic when the target list is large. Precomputed once per search from
/// the target set: the target bounding box, the set of layers any target
/// lives on (bitmask), and the count. Distance-to-box <= distance-to-any-
/// target-in-box, min-via over the layer mask, and min-path-steps to the box
/// are each a lower bound on the corresponding exact-heuristic term, so the
/// summed bbox heuristic is admissible (never overestimates true cost) --
/// unlike the S3-a Python target cap it keeps the FULL goal set intact, so it
/// can never make a routable net unreachable.
#[derive(Clone)]
pub(crate) struct TargetHint {
    min_gx: i32,
    max_gx: i32,
    min_gy: i32,
    max_gy: i32,
    layer_mask: u32,
    count: usize,
}

/// Above this many targets, use the O(1) bbox heuristic; at or below, the
/// exact O(targets) min is cheap enough and tighter.
const BBOX_HEURISTIC_MIN_TARGETS: usize = 16;

impl TargetHint {
    fn from_states(targets: &[GridState]) -> Self {
        let mut min_gx = i32::MAX;
        let mut max_gx = i32::MIN;
        let mut min_gy = i32::MAX;
        let mut max_gy = i32::MIN;
        let mut layer_mask: u32 = 0;
        for t in targets {
            min_gx = min_gx.min(t.gx);
            max_gx = max_gx.max(t.gx);
            min_gy = min_gy.min(t.gy);
            max_gy = max_gy.max(t.gy);
            if (t.layer as usize) < 32 {
                layer_mask |= 1u32 << t.layer;
            }
        }
        TargetHint { min_gx, max_gx, min_gy, max_gy, layer_mask, count: targets.len() }
    }
}

pub(crate) struct GridSearch {
    pub(crate) open_set: BinaryHeap<OpenEntry>,
    pub(crate) store: NodeStore,
    path_vias: FxHashMap<u64, Vec<(i32, i32)>>,
    counter: u32,
    pub(crate) iterations: u32,
    pub(crate) max_iterations: u32,
    target_set: FxHashSet<u64>,
    target_states: Vec<GridState>,
    target_hint: TargetHint,
    opts: SearchOptions,
    /// Best heuristic over the sources (RouteStats.initial_h).
    pub(crate) initial_h: i32,
}

impl GridSearch {
    pub(crate) fn new<S: SearchSink>(
        router: &GridRouter,
        sources: Vec<(i32, i32, u8)>,
        targets: Vec<(i32, i32, u8)>,
        max_iterations: u32,
        opts: SearchOptions,
        sink: &mut S,
    ) -> Self {
        let target_set: FxHashSet<u64> = targets
            .iter()
            .map(|(gx, gy, layer)| GridState::new(*gx, *gy, *layer).as_key())
            .collect();
        let target_states: Vec<GridState> = targets
            .iter()
            .map(|(gx, gy, layer)| GridState::new(*gx, *gy, *layer))
            .collect();
        let target_hint = TargetHint::from_states(&target_states);

        let mut open_set = BinaryHeap::new();
        let mut store = NodeStore::new();
        // Track via positions for each node's path (only if via_exclusion_radius > 0):
        // node key -> list of (gx, gy) via positions on the path to that node
        let mut path_vias: FxHashMap<u64, Vec<(i32, i32)>> = FxHashMap::default();
        let mut counter: u32 = 0;

        // Find minimum layer cost for the initial g penalty (A1: FORBIDDEN
        // sentinels excluded inside min_valid_layer_cost).
        let min_layer_cost = router.min_valid_layer_cost();
        let mut initial_h = i32::MAX;

        for (gx, gy, layer) in sources {
            let state = GridState::new(gx, gy, layer);
            let key = state.as_key();
            // Penalize starting on expensive layers
            let layer_cost = router.layer_cost_or_default(layer as usize);
            let initial_g = layer_cost - min_layer_cost;
            let h = router.heuristic_to_targets(&state, &target_states, &target_hint);
            initial_h = initial_h.min(h);
            open_set.push(OpenEntry {
                f_score: initial_g.saturating_add(h),
                g_score: initial_g,
                state,
                counter,
            });
            counter += 1;
            sink.on_push();
            store.set_source(gx, gy, layer, initial_g);
            // Source nodes start with no vias on their path
            if opts.via_exclusion_radius > 0 {
                path_vias.insert(key, Vec::new());
            }
        }

        Self {
            open_set,
            store,
            path_vias,
            counter,
            iterations: 0,
            max_iterations,
            target_set,
            target_states,
            target_hint,
            opts,
            initial_h,
        }
    }

    /// True when the search cannot make further progress.
    pub(crate) fn is_done(&self) -> bool {
        self.open_set.is_empty() || self.iterations >= self.max_iterations
    }

    /// Pop and process one open-set entry.
    pub(crate) fn step<S: SearchSink>(
        &mut self,
        router: &GridRouter,
        obstacles: &GridObstacleMap,
        sink: &mut S,
    ) -> SearchStep {
        let current_entry = match self.open_set.pop() {
            Some(e) => e,
            None => return SearchStep::Exhausted,
        };
        if self.iterations >= self.max_iterations {
            return SearchStep::IterationCap;
        }
        self.iterations += 1;

        let current = current_entry.state;
        let current_key = current.as_key();
        let current_idx = self.store.ensure_index(current.gx, current.gy, current.layer);
        let g = current_entry.g_score;

        if self.store.is_closed(current_idx) {
            sink.on_duplicate_skip();
            return SearchStep::Skipped;
        }
        sink.on_expand();

        // Check if reached target
        if self.target_set.contains(&current_key) {
            // If end_direction is specified, verify we arrived from a compatible direction
            let arrival_ok = if let Some((end_dx, end_dy)) = self.opts.norm_end_dir {
                if let Some(parent_idx) = self.store.parent(current_idx) {
                    let (px, py, _) = self.store.coords(parent_idx);
                    let arrive_dx = (current.gx - px) as f64;
                    let arrive_dy = (current.gy - py) as f64;
                    let arrive_len = (arrive_dx * arrive_dx + arrive_dy * arrive_dy).sqrt();
                    if arrive_len > 0.0 {
                        // Arrival direction must be within ±120° of the required end
                        // direction: cos(120°) = -0.5, so the dot must be >= -0.5
                        let dot = (arrive_dx / arrive_len) * end_dx + (arrive_dy / arrive_len) * end_dy;
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
                return SearchStep::Found(current_idx, g);
            }
            // Arrival direction not ok - DON'T close, allow reaching from another direction
            return SearchStep::Skipped;
        }

        self.store.set_closed(current_idx);

        // Via direction constraints for diff pair routing (collinear_vias mode):
        // 1. Just came through a via: must continue in the exact pre-via direction
        // 2. One step after a via exit: must stay within ±45° of the pre-via direction
        let (required_direction, allowed_45deg_from): (Option<(i32, i32)>, Option<(i32, i32)>) =
            if self.opts.collinear_vias {
                router.get_via_direction_constraints(&self.store, current_idx, &current)
            } else {
                (None, None)
            };

        // Current node's via list for exclusion checking
        let current_vias: Vec<(i32, i32)> = if self.opts.via_exclusion_radius > 0 {
            self.path_vias.get(&current_key).cloned().unwrap_or_default()
        } else {
            Vec::new()
        };

        // Steps from source for the start-direction constraint
        let current_steps = self.store.steps_of(current_idx);

        // Previous direction (parent -> current) for turn cost calculation
        let prev_direction: Option<(i32, i32)> = self.store.parent(current_idx).map(|parent_idx| {
            let (px, py, _) = self.store.coords(parent_idx);
            let pdx = current.gx - px;
            let pdy = current.gy - py;
            // Normalize to unit direction (vias produce a zero delta)
            if pdx == 0 && pdy == 0 { (0, 0) } else { (pdx.signum(), pdy.signum()) }
        });

        // Expand neighbors - 8 directions.
        // S6: the layer_forbidden(current.layer) test is hoisted out of the
        // direction loop (G2: no track on a forbidden source layer; it was an
        // unconditional first-iteration `break` inside the loop).
        if !router.layer_forbidden(current.layer as usize) {
            for (dx, dy) in DIRECTIONS {
                // Just came through a via: only the exact pre-via direction is allowed
                if let Some((req_dx, req_dy)) = required_direction {
                    if dx != req_dx || dy != req_dy {
                        continue;
                    }
                }
                // One step after a via exit: within ±45° of the pre-via direction
                else if let Some((base_dx, base_dy)) = allowed_45deg_from {
                    if !GridRouter::is_within_45_degrees(dx, dy, base_dx, base_dy) {
                        continue;
                    }
                }

                // Start direction constraint for the first N steps
                if let Some((start_dx, start_dy)) = self.opts.norm_start_dir {
                    if current_steps < self.opts.direction_steps
                        && !GridRouter::is_within_45_degrees(dx, dy, start_dx, start_dy) {
                        continue;
                    }
                }

                let ngx = current.gx + dx;
                let ngy = current.gy + dy;

                if obstacles.segment_blocked(current.gx, current.gy, ngx, ngy,
                                             current.layer as usize,
                                             self.opts.track_margin.at(current.layer as usize)) {
                    sink.on_blocked(ngx, ngy, current.layer);
                    continue;
                }

                // Via exclusion - can't approach our own vias once we've moved away
                if check_via_exclusion(ngx, ngy, current.gx, current.gy,
                                       &current_vias, self.opts.via_exclusion_radius) {
                    continue;
                }

                let neighbor = GridState::new(ngx, ngy, current.layer);
                let neighbor_key = neighbor.as_key();

                let (existing_g, neighbor_closed) = self.store.g_closed_at(ngx, ngy, current.layer);
                if neighbor_closed {
                    continue;
                }

                let base_move_cost = if dx != 0 && dy != 0 { DIAG_COST } else { ORTHO_COST };
                // Apply layer cost multiplier (1000 = 1.0x, 1500 = 1.5x, etc.)
                let layer_multiplier = router.layer_cost_or_default(current.layer as usize);
                let move_cost = (base_move_cost as i64 * layer_multiplier as i64 / 1000) as i32;
                // Turn cost proportional to the turn ANGLE (soft-knobs N5):
                // the knob is the 90-degree anchor; a 45-degree kink costs
                // half, a 135-degree hairpin 1.5x, a reversal 2x. The old
                // flat charge priced a gentle jog like a hairpin -- part of
                // why grid paths zigzag where human routes flow.
                let turn_cost = match prev_direction {
                    Some((pdx, pdy)) if pdx != 0 || pdy != 0 => {
                        if dx != pdx || dy != pdy {
                            let di = octant_index(dx, dy);
                            let pi = octant_index(pdx, pdy);
                            let delta = (di - pi).rem_euclid(8);
                            let angle_units = delta.min(8 - delta); // 1=45deg .. 4=180deg
                            (router.turn_cost as i64 * angle_units as i64 / 2) as i32
                        } else {
                            0
                        }
                    }
                    _ => 0, // No previous direction (source node or via)
                };
                // Stub and layer proximity costs
                let proximity_cost = obstacles.get_stub_proximity_cost(ngx, ngy)
                    + obstacles.get_layer_proximity_cost(ngx, ngy, current.layer as usize);
                // Attraction bonus for positions aligned with tracks on other layers
                let attraction_bonus = obstacles.get_cross_layer_attraction(
                    ngx, ngy, current.layer as usize,
                    router.vertical_attraction_radius, router.vertical_attraction_bonus);
                // Layer direction preference penalty (0=horizontal pref, 1=vertical pref)
                let direction_penalty = if router.direction_preference_cost > 0 {
                    let preferred = router.layer_direction_preferences
                        .get(current.layer as usize).copied().unwrap_or(255);
                    // Charge progress along the NON-PREFERRED axis, diagonals
                    // included (#452). Charging only PURE axis moves
                    // (`dy != 0 && dx == 0`) left a free staircase: a 45 move
                    // satisfies neither test, so on an H-preferred layer the
                    // cheapest way to travel vertically was a chain of
                    // zero-penalty diagonals, and the preference was bypassed
                    // rather than merely weak (raising the knob could not fix
                    // it). A diagonal advances one cell of wrong-axis progress
                    // exactly like a pure axis move, so it pays the same.
                    // Weight stays gentle by design: 50 vs DIAG_COST 1414 is a
                    // ~3.5% nudge that tips ties toward the preferred axis
                    // without discouraging useful 45 routing.
                    match preferred {
                        0 => if dy != 0 { router.direction_preference_cost } else { 0 },
                        1 => if dx != 0 { router.direction_preference_cost } else { 0 },
                        _ => 0, // No preference (255 or other)
                    }
                } else { 0 };
                // Path attraction for bus routing: a multiplicative DISCOUNT
                // percent (0-90) on the whole step, direction-based, no
                // spiraling. Multiplicative + capped means a step can never
                // go free or negative (A* first-arrival stays valid) and the
                // proximity/alignment/layer gradations survive any bonus
                // setting instead of saturating a subtractive floor. The
                // vertical (cross-layer copper) attraction stays subtractive
                // but is capped so it too cannot zero the step.
                let path_discount_pct =
                    router.get_path_attraction_bonus(ngx, ngy, current.layer, dx, dy);
                let base_step = move_cost + turn_cost + proximity_cost + direction_penalty;
                let after_vert = (base_step - attraction_bonus).max(move_cost / 10);
                let step_cost = (after_vert * (100 - path_discount_pct) / 100)
                    .max(move_cost / 10);
                let new_g = g + step_cost;

                if new_g < existing_g {
                    if existing_g != i32::MAX {
                        sink.on_revisit();
                    }
                    self.store.relax(ngx, ngy, current.layer, new_g, current_idx, current_steps + 1);
                    // Propagate via list to neighbor (same vias since no layer change)
                    if self.opts.via_exclusion_radius > 0 {
                        self.path_vias.insert(neighbor_key, current_vias.clone());
                    }
                    let h = router.heuristic_to_targets(&neighbor, &self.target_states, &self.target_hint);
                    self.open_set.push(OpenEntry {
                        f_score: new_g.saturating_add(h), // B4: no i32 wrap on huge h
                        g_score: new_g,
                        state: neighbor,
                        counter: self.counter,
                    });
                    self.counter += 1;
                    sink.on_push();
                }
            }
        }

        // Try via to other layers.
        // For collinear_vias mode, enforce the symmetric constraint around the via:
        // ±45° -> direction D -> VIA -> direction D -> ±45°. Requires at least 2
        // steps before the via (great-grandparent must exist), and the approach
        // direction must be within ±45° of the previous direction.
        let can_place_via = if self.opts.collinear_vias {
            if let Some(parent_idx) = self.store.parent(current_idx) {
                if let Some(grandparent_idx) = self.store.parent(parent_idx) {
                    if self.store.parent(grandparent_idx).is_none() {
                        false // Only 1 step before via - need at least 2
                    } else {
                        let (parent_x, parent_y, _) = self.store.coords(parent_idx);
                        let (gp_x, gp_y, _) = self.store.coords(grandparent_idx);
                        // Previous direction (grandparent -> parent) vs approach
                        // direction (parent -> current)
                        let prev_dx = parent_x - gp_x;
                        let prev_dy = parent_y - gp_y;
                        let approach_dx = current.gx - parent_x;
                        let approach_dy = current.gy - parent_y;
                        if (prev_dx != 0 || prev_dy != 0) && (approach_dx != 0 || approach_dy != 0) {
                            GridRouter::is_within_45_degrees(
                                approach_dx.signum(), approach_dy.signum(),
                                prev_dx.signum(), prev_dy.signum())
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

        // Would this via be too close to existing vias in the path?
        let via_too_close = if self.opts.via_exclusion_radius > 0 {
            current_vias.iter().any(|&(vx, vy)| {
                let dist = (current.gx - vx).abs().max((current.gy - vy).abs());
                dist <= self.opts.via_exclusion_radius * 2 // Need 2x radius for via-via clearance
            })
        } else {
            false
        };

        // A free via is an existing same-net hole (via-in-pad / through-hole pad);
        // changing layers through it is always legal and adds no new drill, so it
        // overrides both the via-blocked veto and the path's via-too-close test.
        // Without this, the same cell can be flagged free_via AND via_blocked at
        // once (its own clearance ring blocks the center), and the via branch is
        // skipped before is_free_via is ever read -- forcing a redundant via to be
        // dropped a couple cells away beside a perfectly good via-in-pad.
        let free_here = obstacles.is_free_via(current.gx, current.gy);
        if can_place_via && (!via_too_close || free_here)
            && (!obstacles.is_via_blocked(current.gx, current.gy) || free_here) {
            for layer in 0..obstacles.num_layers as u8 {
                if layer == current.layer {
                    continue;
                }
                if router.layer_forbidden(layer as usize) {
                    continue; // FORBIDDEN LAYER: never end a via here
                }

                // Check if destination layer is blocked at this position
                if obstacles.segment_blocked(current.gx, current.gy, current.gx, current.gy,
                                             layer as usize,
                                             self.opts.track_margin.at(layer as usize)) {
                    sink.on_blocked(current.gx, current.gy, layer);
                    continue;
                }

                let neighbor = GridState::new(current.gx, current.gy, layer);
                let neighbor_key = neighbor.as_key();

                let (existing_g, neighbor_closed) = self.store.g_closed_at(current.gx, current.gy, layer);
                if neighbor_closed {
                    continue;
                }

                // Zero cost for free via positions (through-hole pads on same net).
                // S6: is_free_via was recomputed for every destination layer;
                // free_here is loop-invariant, reuse it.
                let via_cost = if free_here { 0 } else { router.via_cost };
                let proximity_cost = (obstacles.get_stub_proximity_cost(current.gx, current.gy)
                    + obstacles.get_layer_proximity_cost(current.gx, current.gy, layer as usize))
                    * router.via_proximity_cost;
                // Layer transition cost: penalize switching TO expensive layers,
                // discount switching to cheaper ones
                let current_layer_cost = router.layer_cost_or_default(current.layer as usize);
                let dest_layer_cost = router.layer_cost_or_default(layer as usize);
                let layer_transition_cost = dest_layer_cost - current_layer_cost;
                // Combined via cost can be as low as 0 when switching to a much cheaper layer
                let combined_via_cost = (via_cost + layer_transition_cost).max(0);
                let new_g = g + combined_via_cost + proximity_cost;

                if new_g < existing_g {
                    if existing_g != i32::MAX {
                        sink.on_revisit();
                    }
                    self.store.relax(current.gx, current.gy, layer, new_g, current_idx, current_steps);
                    // Track via positions (only real vias, not free vias at through-holes)
                    if self.opts.via_exclusion_radius > 0 {
                        if free_here {
                            self.path_vias.insert(neighbor_key, current_vias.clone());
                        } else {
                            let mut new_vias = current_vias.clone();
                            new_vias.push((current.gx, current.gy));
                            self.path_vias.insert(neighbor_key, new_vias);
                        }
                    }
                    let h = router.heuristic_to_targets(&neighbor, &self.target_states, &self.target_hint);
                    self.open_set.push(OpenEntry {
                        f_score: new_g.saturating_add(h), // B4: no i32 wrap on huge h
                        g_score: new_g,
                        state: neighbor,
                        counter: self.counter,
                    });
                    self.counter += 1;
                    sink.on_push();
                }
            }
        }

        SearchStep::Progress(current)
    }
}

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
    // Fraction (percent) of the attraction bonus granted on layers OTHER
    // than the path point's layer (issue #296 R9 phase B: a bus corridor
    // must survive a via transition -- with 0 the river loses all guidance
    // the moment a member changes layer). 0 = same-layer only (legacy).
    attraction_cross_layer_pct: i32,
    // Spatial POINT index for the attraction path (soft-knobs P2): bucket
    // (bx, by, layer=254 wildcard) -> indices into attraction_path. The old
    // membership-only hash still forced a linear scan of the WHOLE path per
    // candidate move; the index bounds each lookup to the nearby points.
    attraction_path_buckets: FxHashMap<u64, Vec<u32>>,
}

#[pymethods]
impl GridRouter {
    #[new]
    #[pyo3(signature = (via_cost, h_weight, turn_cost=None, via_proximity_cost=1, vertical_attraction_radius=0, vertical_attraction_bonus=0, layer_costs=None, proximity_heuristic_cost=None, layer_direction_preferences=None, direction_preference_cost=0, attraction_radius=0, attraction_bonus=0, attraction_cross_layer_pct=0))]
    pub fn new(via_cost: i32, h_weight: f32, turn_cost: Option<i32>, via_proximity_cost: Option<i32>,
               vertical_attraction_radius: i32, vertical_attraction_bonus: i32,
               layer_costs: Option<Vec<i32>>, proximity_heuristic_cost: Option<i32>,
               layer_direction_preferences: Option<Vec<u8>>, direction_preference_cost: i32,
               attraction_radius: i32, attraction_bonus: i32,
               attraction_cross_layer_pct: i32) -> Self {
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
            attraction_cross_layer_pct,
            attraction_path_buckets: FxHashMap::default(),
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
        self.attraction_path_buckets.clear();
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

        // Build the spatial POINT index (soft-knobs P2): each point goes in
        // its OWN bucket; lookups scan the neighborhood of buckets within the
        // radius. Layer-blind (254 wildcard key) -- the lookup filters layers
        // itself for the cross-layer fraction (#296 R9 phase B).
        let bucket_size = (self.attraction_radius / 2).max(1);
        for (i, &(px, py, _layer, _, _)) in path_with_directions.iter().enumerate() {
            let key = Self::pack_bucket_key(px / bucket_size, py / bucket_size, 254);
            self.attraction_path_buckets.entry(key).or_default().push(i as u32);
        }

        self.attraction_path = path_with_directions;
    }

    /// Clear the attraction path
    pub fn clear_attraction_path(&mut self) {
        self.attraction_path.clear();
        self.attraction_path_buckets.clear();
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
    /// track_margin: Extra margin in FRACTIONAL grid cells for wide tracks
    /// (power nets / impedance-width nets). Scalar (uniform) or per-layer
    /// list of floats (#156). When > 0 on a layer, the swept-capsule check
    /// reserves that Euclidean radius along every move on that layer.
    ///
    /// C1: thin wrapper over the shared GridSearch core with a StatsSink.
    #[pyo3(signature = (obstacles, sources, targets, max_iterations, collinear_vias=false, via_exclusion_radius=0, start_direction=None, end_direction=None, direction_steps=2, track_margin=TrackMarginArg::Scalar(0.0)))]
    #[allow(clippy::too_many_arguments)]
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
        track_margin: TrackMarginArg,
    ) -> (Option<Vec<(i32, i32, u8)>>, u32, std::collections::HashMap<String, f64>) {
        let opts = SearchOptions::new(collinear_vias, via_exclusion_radius,
                                      start_direction, end_direction,
                                      direction_steps, track_margin);
        let mut sink = StatsSink::default();
        let mut search = GridSearch::new(self, sources, targets, max_iterations, opts, &mut sink);
        sink.stats.initial_h = search.initial_h;

        loop {
            match search.step(self, obstacles, &mut sink) {
                SearchStep::Found(goal_idx, g) => {
                    let path = search.store.reconstruct_path(goal_idx);
                    let stats = &mut sink.stats;
                    stats.path_length = path.len() as u32;
                    stats.path_cost = g;
                    stats.final_g = g;
                    stats.open_set_size = search.open_set.len() as u32;
                    stats.closed_set_size = search.store.closed_len();
                    // Count vias in path
                    for i in 1..path.len() {
                        if path[i].2 != path[i - 1].2 {
                            stats.via_count += 1;
                        }
                    }
                    let stats_dict = self.stats_to_dict(&sink.stats);
                    return (Some(path), search.iterations, stats_dict);
                }
                SearchStep::IterationCap | SearchStep::Exhausted => break,
                SearchStep::Progress(_) | SearchStep::Skipped => {}
            }
        }

        // No path found - fill in final stats
        sink.stats.open_set_size = search.open_set.len() as u32;
        sink.stats.closed_set_size = search.store.closed_len();
        let stats_dict = self.stats_to_dict(&sink.stats);
        (None, search.iterations, stats_dict)
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
    ///
    /// C1: thin wrapper over the shared GridSearch core with a FrontierSink.
    #[pyo3(signature = (obstacles, sources, targets, max_iterations, collinear_vias=false, via_exclusion_radius=0, start_direction=None, end_direction=None, direction_steps=2, track_margin=TrackMarginArg::Scalar(0.0)))]
    #[allow(clippy::too_many_arguments)]
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
        track_margin: TrackMarginArg,
    ) -> (Option<Vec<(i32, i32, u8)>>, u32, Vec<(i32, i32, u8)>) {
        let opts = SearchOptions::new(collinear_vias, via_exclusion_radius,
                                      start_direction, end_direction,
                                      direction_steps, track_margin);
        let mut sink = FrontierSink::new();
        let mut search = GridSearch::new(self, sources, targets, max_iterations, opts, &mut sink);

        loop {
            match search.step(self, obstacles, &mut sink) {
                SearchStep::Found(goal_idx, _g) => {
                    let path = search.store.reconstruct_path(goal_idx);
                    return (Some(path), search.iterations, Vec::new());
                }
                SearchStep::IterationCap | SearchStep::Exhausted => break,
                SearchStep::Progress(_) | SearchStep::Skipped => {}
            }
        }

        (None, search.iterations, sink.tracker.get_blocked())
    }
}

impl GridRouter {
    /// FORBIDDEN LAYER (`--layer-costs -1` => a negative entry in `layer_costs`):
    /// the router never places a track on, nor ends a via on, such a layer. It
    /// still acts as an obstacle (its copper blocks via barrels) and through-vias
    /// may SPAN it (a via is one direct edge to its target layer).
    #[inline]
    fn layer_forbidden(&self, layer: usize) -> bool {
        self.layer_costs.get(layer).map_or(false, |&c| c < 0)
    }

    /// Cost to use in arithmetic; a forbidden/missing layer folds to the neutral
    /// 1.0x (1000) so the sentinel can never leak into a subtraction/multiply.
    #[inline]
    fn layer_cost_or_default(&self, layer: usize) -> i32 {
        match self.layer_costs.get(layer).copied() {
            Some(c) if c >= 0 => c,
            _ => 1000,
        }
    }

    /// Minimum non-forbidden layer cost (A1: exclude FORBIDDEN sentinels < 0 —
    /// else min() returns the sentinel and pollutes the start penalty + the
    /// admissibility-scaled heuristic). Shared by the source init + heuristic.
    #[inline]
    fn min_valid_layer_cost(&self) -> i32 {
        self.layer_costs.iter().copied().filter(|&c| c >= 0).min().unwrap_or(1000)
    }

    /// Octile distance heuristic to nearest target
    /// Uses minimum layer cost to remain admissible (never overestimate)
    #[inline]
    fn heuristic_to_targets(&self, state: &GridState, targets: &[GridState],
                            hint: &TargetHint) -> i32 {
        // B4: an empty target list used to fall through as i32::MAX, and the
        // f = g + h addition then wrapped in release builds, corrupting the
        // heap order into a silent max-iterations burn. No target -> h = 0.
        if targets.is_empty() {
            return 0;
        }
        let min_layer_cost = self.min_valid_layer_cost();

        // S3-b (#385): O(1) admissible bbox heuristic for large target lists.
        // The exact loop below is O(targets) per push; a multipoint/tap
        // backward probe passes hundreds of tap points, making the heuristic
        // the hot cost. Each bbox term is a per-component lower bound on the
        // exact min (distance to the clamped box point <= distance to any
        // target in the box; via_cost=0 iff some target shares this layer;
        // min path-steps to the box), so the sum is admissible -- weaker
        // (may expand a few more nodes) but never overestimates, so it can
        // never make a routable net unreachable (unlike the S3-a target cap).
        if hint.count > BBOX_HEURISTIC_MIN_TARGETS {
            let cx = state.gx.clamp(hint.min_gx, hint.max_gx);
            let cy = state.gy.clamp(hint.min_gy, hint.max_gy);
            let dx = (state.gx - cx).abs();
            let dy = (state.gy - cy).abs();
            let diag = dx.min(dy);
            let orth = (dx - dy).abs();
            let base_dist = diag * DIAG_COST + orth * ORTHO_COST;
            let mut h = (base_dist as i64 * min_layer_cost as i64 / 1000) as i32;
            // via term: 0 if any target is already on this layer, else via_cost
            let on_layer = (state.layer as usize) < 32
                && (hint.layer_mask & (1u32 << state.layer)) != 0;
            if !on_layer {
                h += self.via_cost;
            }
            if self.proximity_heuristic_cost > 0 {
                h += (diag + orth) * self.proximity_heuristic_cost;
            }
            return (h as f32 * self.h_weight) as i32;
        }

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
        // Each direction is 45° apart, so "within 45°" means same or adjacent.
        // B5 (issue #386): a zero delta used to silently alias East, turning a
        // degenerate base direction into a real E/NE/SE constraint. A
        // degenerate BASE is no constraint at all (allow); a degenerate test
        // direction can never satisfy a real constraint (reject).
        let base_idx = match Self::direction_to_index(base_dx, base_dy) {
            Some(i) => i,
            None => return true,
        };
        let test_idx = match Self::direction_to_index(dx, dy) {
            Some(i) => i,
            None => return false,
        };

        // Check if indices are adjacent (with wraparound)
        let diff = (test_idx as i32 - base_idx as i32 + 8) % 8;
        diff == 0 || diff == 1 || diff == 7
    }

    /// Convert direction to index (0-7); None for a non-unit delta (B5).
    #[inline]
    fn direction_to_index(dx: i32, dy: i32) -> Option<usize> {
        match (dx, dy) {
            (1, 0) => Some(0),    // E
            (1, -1) => Some(1),   // NE
            (0, -1) => Some(2),   // N
            (-1, -1) => Some(3),  // NW
            (-1, 0) => Some(4),   // W
            (-1, 1) => Some(5),   // SW
            (0, 1) => Some(6),    // S
            (1, 1) => Some(7),    // SE
            _ => None,
        }
    }

    /// Get via direction constraints for the current position
    /// Returns (required_exact_direction, allowed_45deg_base_direction)
    fn get_via_direction_constraints(
        &self,
        store: &NodeStore,
        current_idx: u32,
        current: &GridState,
    ) -> (Option<(i32, i32)>, Option<(i32, i32)>) {
        // Get parent
        let parent_idx = match store.parent(current_idx) {
            Some(i) => i,
            None => return (None, None),
        };
        let (parent_x, parent_y, parent_layer) = store.coords(parent_idx);

        // Check if parent was a via (same position as current, different layer)
        let parent_is_via = parent_x == current.gx && parent_y == current.gy && parent_layer != current.layer;

        if parent_is_via {
            // We just came through a via - need exact same direction as before via
            if let Some(grandparent_idx) = store.parent(parent_idx) {
                let (gp_x, gp_y, _) = store.coords(grandparent_idx);
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
        let grandparent_idx = match store.parent(parent_idx) {
            Some(i) => i,
            None => return (None, None),
        };
        let (gp_x, gp_y, gp_layer) = store.coords(grandparent_idx);

        let grandparent_is_via = gp_x == parent_x && gp_y == parent_y && gp_layer != parent_layer;

        if grandparent_is_via {
            // We're one step after via exit - still require exact same direction
            // (This ensures 2 straight steps after via for symmetric geometry when reversed)
            if let Some(great_grandparent_idx) = store.parent(grandparent_idx) {
                let (ggp_x, ggp_y, _) = store.coords(great_grandparent_idx);
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
        let great_grandparent_idx = match store.parent(grandparent_idx) {
            Some(i) => i,
            None => return (None, None),
        };
        let (ggp_x, ggp_y, ggp_layer) = store.coords(great_grandparent_idx);

        let great_grandparent_is_via = ggp_x == gp_x && ggp_y == gp_y && ggp_layer != gp_layer;

        if great_grandparent_is_via {
            // We're two steps after via exit - allow ±45° turn
            if let Some(gggp_idx) = store.parent(great_grandparent_idx) {
                let (gggp_x, gggp_y, _) = store.coords(gggp_idx);
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
    /// (C2: the unused bucket_size parameter was dropped)
    #[inline]
    fn pack_bucket_key(bx: i32, by: i32, layer: u8) -> u64 {
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

        // Scan only the point-index buckets within reach (soft-knobs P2: the
        // old membership hash still linear-scanned the WHOLE path per move).
        // Distance is EUCLIDEAN (soft-knobs N4: the old Manhattan test against
        // a Euclidean-derived radius halved the diagonal reach). Same-layer
        // points give the full bonus; when only OTHER-layer points are near,
        // grant attraction_cross_layer_pct of it (the corridor survives via
        // transitions instead of going silent, #296 R9 phase B).
        let bucket_size = (self.attraction_radius / 2).max(1);
        let r2 = self.attraction_radius * self.attraction_radius;
        let bx0 = (x - self.attraction_radius) / bucket_size - 1;
        let bx1 = (x + self.attraction_radius) / bucket_size + 1;
        let by0 = (y - self.attraction_radius) / bucket_size - 1;
        let by1 = (y + self.attraction_radius) / bucket_size + 1;

        let mut d2_same = i64::MAX;
        let mut dir_same: Option<(i8, i8)> = None;
        let mut d2_cross = i64::MAX;
        let mut dir_cross: Option<(i8, i8)> = None;

        for bx in bx0..=bx1 {
            for by in by0..=by1 {
                let Some(idxs) = self.attraction_path_buckets
                    .get(&Self::pack_bucket_key(bx, by, 254)) else { continue };
                for &i in idxs {
                    let (px, py, pl, path_dx, path_dy) = self.attraction_path[i as usize];
                    let ddx = (x - px) as i64;
                    let ddy = (y - py) as i64;
                    let d2 = ddx * ddx + ddy * ddy;
                    if d2 > r2 as i64 {
                        continue;
                    }
                    if pl == layer {
                        if d2 < d2_same {
                            d2_same = d2;
                            dir_same = Some((path_dx, path_dy));
                        }
                    } else if self.attraction_cross_layer_pct > 0 && d2 < d2_cross {
                        d2_cross = d2;
                        dir_cross = Some((path_dx, path_dy));
                    }
                }
            }
        }

        let (nearest_d2, nearest_dir, layer_pct) = if dir_same.is_some() {
            (d2_same, dir_same, 100)
        } else {
            (d2_cross, dir_cross, self.attraction_cross_layer_pct)
        };
        let (path_dx, path_dy) = match nearest_dir {
            Some(d) => d,
            None => return 0,
        };
        let nearest_dist = (nearest_d2 as f32).sqrt() as i32;

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

        // Return a DISCOUNT PERCENT of the step cost (0-90), not cost units:
        // a subtractive bonus larger than the move cost saturates into a
        // negative/floored edge and every gradation (proximity, alignment,
        // layer) collapses -- the soft-knobs review's finding #2. Strength
        // mapping keeps legacy tuning meaningful: 50 bonus units = 1% max
        // discount, so the default 5000 = full strength, capped at 90%.
        let strength_pct = (self.attraction_bonus / 50).min(90);
        (strength_pct as i64 * proximity_pct as i64 * alignment as i64
            * layer_pct as i64 / 1_000_000) as i32
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
