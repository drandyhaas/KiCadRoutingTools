//! Placement scorer — Rust implementation for fast candidate evaluation.
//!
//! Provides `find_best_candidate` which evaluates a grid of candidate positions
//! and rotations for a component, scoring each by airwire length + crossing penalty.
//! Called from Python via PyO3 bindings.

use pyo3::prelude::*;
use std::collections::{HashMap, HashSet};

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

#[inline]
fn snap_to_grid(value: f64, grid_step: f64) -> f64 {
    (value / grid_step).round() * grid_step
}

#[inline]
fn local_to_global(ox: f64, oy: f64, rot_deg: f64, lx: f64, ly: f64) -> (f64, f64) {
    let angle = (-rot_deg).to_radians();
    let cos_a = angle.cos();
    let sin_a = angle.sin();
    let gx = ox + lx * cos_a - ly * sin_a;
    let gy = oy + lx * sin_a + ly * cos_a;
    (gx, gy)
}

#[inline]
fn rects_overlap(
    a_min_x: f64, a_min_y: f64, a_max_x: f64, a_max_y: f64,
    b_min_x: f64, b_min_y: f64, b_max_x: f64, b_max_y: f64,
    clearance: f64,
) -> bool {
    !(a_max_x + clearance <= b_min_x
        || b_max_x + clearance <= a_min_x
        || a_max_y + clearance <= b_min_y
        || b_max_y + clearance <= a_min_y)
}

#[inline]
fn overlaps_any(
    gmin_x: f64, gmin_y: f64, gmax_x: f64, gmax_y: f64,
    placed_rects: &[(f64, f64, f64, f64)],
    clearance: f64,
) -> bool {
    for &(pmin_x, pmin_y, pmax_x, pmax_y) in placed_rects {
        if rects_overlap(
            gmin_x, gmin_y, gmax_x, gmax_y,
            pmin_x, pmin_y, pmax_x, pmax_y,
            clearance,
        ) {
            return true;
        }
    }
    false
}

/// Prim's MST algorithm — returns edges as (idx_a, idx_b, distance).
fn compute_mst_edges(points: &[(f64, f64)]) -> Vec<(usize, usize, f64)> {
    let n = points.len();
    if n < 2 {
        return Vec::new();
    }
    if n == 2 {
        let dx = points[0].0 - points[1].0;
        let dy = points[0].1 - points[1].1;
        return vec![(0, 1, (dx * dx + dy * dy).sqrt())];
    }

    let mut in_tree = vec![false; n];
    let mut min_dist = vec![f64::INFINITY; n];
    let mut nearest = vec![0usize; n];
    let mut edges = Vec::with_capacity(n - 1);

    in_tree[0] = true;
    for j in 1..n {
        let dx = points[0].0 - points[j].0;
        let dy = points[0].1 - points[j].1;
        min_dist[j] = (dx * dx + dy * dy).sqrt();
    }

    for _ in 0..n - 1 {
        let mut best_j = usize::MAX;
        let mut best_dist = f64::INFINITY;
        for j in 0..n {
            if !in_tree[j] && min_dist[j] < best_dist {
                best_dist = min_dist[j];
                best_j = j;
            }
        }
        if best_j == usize::MAX {
            break;
        }
        in_tree[best_j] = true;
        edges.push((nearest[best_j], best_j, best_dist));

        for j in 0..n {
            if !in_tree[j] {
                let dx = points[best_j].0 - points[j].0;
                let dy = points[best_j].1 - points[j].1;
                let d = (dx * dx + dy * dy).sqrt();
                if d < min_dist[j] {
                    min_dist[j] = d;
                    nearest[j] = best_j;
                }
            }
        }
    }
    edges
}

/// Airwire segment: (x1, y1, x2, y2, net_id)
type Airwire = (f64, f64, f64, f64, i32);

/// Build MST-based airwire segments for specified nets.
fn build_airwires(
    net_pad_positions: &HashMap<i32, Vec<(f64, f64)>>,
    involved_nets: &HashSet<i32>,
) -> Vec<Airwire> {
    let mut airwires = Vec::new();
    for &net_id in involved_nets {
        let points = match net_pad_positions.get(&net_id) {
            Some(pts) => pts,
            None => continue,
        };
        if points.len() < 2 {
            continue;
        }
        if points.len() == 2 {
            airwires.push((points[0].0, points[0].1, points[1].0, points[1].1, net_id));
        } else {
            let edges = compute_mst_edges(points);
            for (i, j, _dist) in edges {
                airwires.push((
                    points[i].0, points[i].1,
                    points[j].0, points[j].1,
                    net_id,
                ));
            }
        }
    }
    airwires
}

#[inline]
fn ccw(ax: f64, ay: f64, bx: f64, by: f64, cx: f64, cy: f64) -> bool {
    (cy - ay) * (bx - ax) > (by - ay) * (cx - ax)
}

#[inline]
fn segments_intersect(
    a1x: f64, a1y: f64, a2x: f64, a2y: f64,
    b1x: f64, b1y: f64, b2x: f64, b2y: f64,
) -> bool {
    // Check shared endpoints (not a real crossing)
    const EPS: f64 = 0.001;
    if (a1x - b1x).abs() < EPS && (a1y - b1y).abs() < EPS { return false; }
    if (a1x - b2x).abs() < EPS && (a1y - b2y).abs() < EPS { return false; }
    if (a2x - b1x).abs() < EPS && (a2y - b1y).abs() < EPS { return false; }
    if (a2x - b2x).abs() < EPS && (a2y - b2y).abs() < EPS { return false; }

    (ccw(a1x, a1y, b1x, b1y, b2x, b2y) != ccw(a2x, a2y, b1x, b1y, b2x, b2y))
        && (ccw(a1x, a1y, a2x, a2y, b1x, b1y) != ccw(a1x, a1y, a2x, a2y, b2x, b2y))
}

fn count_crossings(new_airwires: &[Airwire], existing_airwires: &[Airwire]) -> i32 {
    let mut crossings = 0i32;
    for &(a1x, a1y, a2x, a2y, net_a) in new_airwires {
        for &(b1x, b1y, b2x, b2y, net_b) in existing_airwires {
            if net_a == net_b {
                continue;
            }
            if segments_intersect(a1x, a1y, a2x, a2y, b1x, b1y, b2x, b2y) {
                crossings += 1;
            }
        }
    }
    crossings
}

fn compute_airwire_length(airwires: &[Airwire]) -> f64 {
    let mut total = 0.0f64;
    for &(x1, y1, x2, y2, _) in airwires {
        let dx = x2 - x1;
        let dy = y2 - y1;
        total += (dx * dx + dy * dy).sqrt();
    }
    total
}

// ---------------------------------------------------------------------------
// Candidate evaluation for a single position + rotation
// ---------------------------------------------------------------------------

struct CandidateResult {
    origin_x: f64,
    origin_y: f64,
    rotation: f64,
    score: f64,
    pad_globals: Vec<(f64, f64, i32)>,
    new_airwires: Vec<Airwire>,
}

fn evaluate_candidate(
    origin_x: f64,
    origin_y: f64,
    rotation: f64,
    pad_locals: &[(f64, f64, i32)],
    net_pad_positions: &HashMap<i32, Vec<(f64, f64)>>,
    existing_airwires: &[Airwire],
    crossing_penalty: f64,
) -> (f64, Vec<(f64, f64, i32)>, Vec<Airwire>) {
    // Compute global pad positions
    let mut pad_globals: Vec<(f64, f64, i32)> = Vec::with_capacity(pad_locals.len());
    let mut involved_nets: HashSet<i32> = HashSet::new();
    let mut temp_net_positions: HashMap<i32, Vec<(f64, f64)>> = HashMap::new();

    for &(lx, ly, net_id) in pad_locals {
        let (gx, gy) = local_to_global(origin_x, origin_y, rotation, lx, ly);
        pad_globals.push((gx, gy, net_id));
        involved_nets.insert(net_id);
        let entry = temp_net_positions.entry(net_id).or_insert_with(|| {
            net_pad_positions.get(&net_id).cloned().unwrap_or_default()
        });
        entry.push((gx, gy));
    }

    let new_airwires = build_airwires(&temp_net_positions, &involved_nets);
    let crossings = count_crossings(&new_airwires, existing_airwires);
    let airwire_length = compute_airwire_length(&new_airwires);
    let score = airwire_length + (crossings as f64) * crossing_penalty;

    (score, pad_globals, new_airwires)
}

// ---------------------------------------------------------------------------
// Python-exposed function
// ---------------------------------------------------------------------------

/// Find the best candidate position and rotation for placing a component.
///
/// Returns Some((origin_x, origin_y, rotation, score, pad_globals, new_airwires))
/// or None if no valid position was found.
#[pyfunction]
#[pyo3(signature = (
    pad_locals,
    bounds_by_rotation,
    target_x, target_y,
    search_radius, candidate_step, grid_step,
    usable_min_x, usable_min_y, usable_max_x, usable_max_y,
    placed_rects,
    collision_clearance,
    net_pad_positions,
    existing_airwires,
    crossing_penalty,
    is_first,
    board_center_x, board_center_y,
))]
fn find_best_candidate(
    pad_locals: Vec<(f64, f64, i32)>,
    bounds_by_rotation: Vec<(f64, f64, f64, f64)>,
    target_x: f64,
    target_y: f64,
    search_radius: f64,
    candidate_step: f64,
    grid_step: f64,
    usable_min_x: f64,
    usable_min_y: f64,
    usable_max_x: f64,
    usable_max_y: f64,
    placed_rects: Vec<(f64, f64, f64, f64)>,
    collision_clearance: f64,
    net_pad_positions: HashMap<i32, Vec<(f64, f64)>>,
    existing_airwires: Vec<Airwire>,
    crossing_penalty: f64,
    is_first: bool,
    board_center_x: f64,
    board_center_y: f64,
) -> PyResult<Option<(f64, f64, f64, f64, Vec<(f64, f64, i32)>, Vec<Airwire>)>> {
    let rotations = [0.0f64, 90.0, 180.0, 270.0];

    let mut best_score: Option<f64> = None;
    let mut best_result: Option<CandidateResult> = None;

    // First component: place at board center, only try rotations
    if is_first {
        let origin_x = snap_to_grid(board_center_x, grid_step);
        let origin_y = snap_to_grid(board_center_y, grid_step);

        for (ri, &rotation) in rotations.iter().enumerate() {
            let (rmin_x, rmin_y, rmax_x, rmax_y) = bounds_by_rotation[ri];
            let gmin_x = origin_x + rmin_x;
            let gmin_y = origin_y + rmin_y;
            let gmax_x = origin_x + rmax_x;
            let gmax_y = origin_y + rmax_y;

            if gmin_x < usable_min_x || gmax_x > usable_max_x
                || gmin_y < usable_min_y || gmax_y > usable_max_y
            {
                continue;
            }
            if overlaps_any(gmin_x, gmin_y, gmax_x, gmax_y, &placed_rects, collision_clearance) {
                continue;
            }

            let (score, pad_globals, new_airwires) = evaluate_candidate(
                origin_x, origin_y, rotation,
                &pad_locals, &net_pad_positions, &existing_airwires, crossing_penalty,
            );

            if best_score.is_none() || score < best_score.unwrap() {
                best_score = Some(score);
                best_result = Some(CandidateResult {
                    origin_x, origin_y, rotation, score, pad_globals, new_airwires,
                });
            }
        }
    }

    // Grid search (skipped for first component if a candidate was found)
    let radii = [search_radius, search_radius * 2.0, search_radius * 4.0];

    for &attempt_radius in &radii {
        if best_result.is_some() {
            break;
        }

        let n = (2.0 * attempt_radius / candidate_step) as i32 + 1;
        let half_n = n / 2;

        for (ri, &rotation) in rotations.iter().enumerate() {
            let (rmin_x, rmin_y, rmax_x, rmax_y) = bounds_by_rotation[ri];

            for ix in 0..n {
                for iy in 0..n {
                    let cx = target_x + ((ix - half_n) as f64) * candidate_step;
                    let cy = target_y + ((iy - half_n) as f64) * candidate_step;

                    let origin_x = snap_to_grid(cx, grid_step);
                    let origin_y = snap_to_grid(cy, grid_step);

                    let gmin_x = origin_x + rmin_x;
                    let gmin_y = origin_y + rmin_y;
                    let gmax_x = origin_x + rmax_x;
                    let gmax_y = origin_y + rmax_y;

                    if gmin_x < usable_min_x || gmax_x > usable_max_x
                        || gmin_y < usable_min_y || gmax_y > usable_max_y
                    {
                        continue;
                    }
                    if overlaps_any(
                        gmin_x, gmin_y, gmax_x, gmax_y,
                        &placed_rects, collision_clearance,
                    ) {
                        continue;
                    }

                    let (score, pad_globals, new_airwires) = evaluate_candidate(
                        origin_x, origin_y, rotation,
                        &pad_locals, &net_pad_positions, &existing_airwires,
                        crossing_penalty,
                    );

                    if best_score.is_none() || score < best_score.unwrap() {
                        best_score = Some(score);
                        best_result = Some(CandidateResult {
                            origin_x, origin_y, rotation, score, pad_globals, new_airwires,
                        });
                    }
                }
            }
        }
    }

    match best_result {
        Some(r) => Ok(Some((
            r.origin_x, r.origin_y, r.rotation, r.score,
            r.pad_globals, r.new_airwires,
        ))),
        None => Ok(None),
    }
}

/// Python module
#[pymodule]
fn placement_scorer(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add("__version__", env!("CARGO_PKG_VERSION"))?;
    m.add_function(wrap_pyfunction!(find_best_candidate, m)?)?;
    Ok(())
}
