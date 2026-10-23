//! Placement kernels: the airwire trees, crossings, foreign-pad hits, group
//! gaps and congestion that `py_placer/placement/quench.py` and
//! `simple.py` price for every candidate pose, done in Rust because they run
//! once per move.
//!
//! Each computes what its docstring's Python names, with the same tie rules.
//! The trees and gap trees keep the Python's float expressions in order;
//! the crossing and pad-hit kernels skip pairs whose boxes cannot meet, and
//! the congestion kernels (`net_box`, `pad_use`, `cong_delta`) sum a move's
//! change over the cells it reaches rather than the whole window, so their
//! totals can differ from a Python sum in the last bits.

use std::collections::{HashMap, HashSet};

use numpy::ndarray::Array2;
use numpy::{IntoPyArray, PyArray1, PyArray2, PyReadonlyArray1, PyReadonlyArray2};
use pyo3::prelude::*;

/// `np.minimum` / `np.maximum`: a NaN in either operand gives NaN.
#[inline]
fn np_min(a: f64, b: f64) -> f64 {
    if a.is_nan() || b.is_nan() {
        f64::NAN
    } else if b < a {
        b
    } else {
        a
    }
}

#[inline]
fn np_max(a: f64, b: f64) -> f64 {
    if a.is_nan() || b.is_nan() {
        f64::NAN
    } else if b > a {
        b
    } else {
        a
    }
}

/// quench's `ccw`: `(cy - ay) * (bx - ax) > (by - ay) * (cx - ax)`.
#[inline]
fn ccw(ax: f64, ay: f64, bx: f64, by: f64, cx: f64, cy: f64) -> bool {
    (cy - ay) * (bx - ax) > (by - ay) * (cx - ax)
}

/// Every (a row, b row) pair of airwires that cross, in row-major order --
/// `quench._crossing_hits` filtered to its hits: boxes that meet (`a`'s grown
/// by 1e-6), segments that cross by the orientation test, never two of one
/// net, never two sharing an endpoint within 1 um. Rows are
/// (x1, y1, x2, y2, net). With `net_w` (a net-id-indexed weight table) each
/// pair carries max(net_w[net_a], net_w[net_b]); without, 1.0.
///
/// Returns (ii, jj, w).
#[pyfunction]
#[pyo3(signature = (a, b, net_w=None))]
pub fn crossing_pairs<'py>(
    py: Python<'py>,
    a: PyReadonlyArray2<'py, f64>,
    b: PyReadonlyArray2<'py, f64>,
    net_w: Option<PyReadonlyArray1<'py, f64>>,
) -> (
    Bound<'py, PyArray1<i64>>,
    Bound<'py, PyArray1<i64>>,
    Bound<'py, PyArray1<f64>>,
) {
    let a = a.as_array();
    let b = b.as_array();
    let w = net_w.as_ref().map(|w| w.as_array());
    let m = 1e-6;
    let eps = 0.001;
    // b's boxes once.
    let nb = b.nrows();
    let mut bb: Vec<[f64; 4]> = Vec::with_capacity(nb);
    for j in 0..nb {
        bb.push([
            np_min(b[[j, 0]], b[[j, 2]]),
            np_max(b[[j, 0]], b[[j, 2]]),
            np_min(b[[j, 1]], b[[j, 3]]),
            np_max(b[[j, 1]], b[[j, 3]]),
        ]);
    }
    let mut ii: Vec<i64> = Vec::new();
    let mut jj: Vec<i64> = Vec::new();
    let mut ww: Vec<f64> = Vec::new();
    for i in 0..a.nrows() {
        let (a1x, a1y, a2x, a2y, an) = (a[[i, 0]], a[[i, 1]], a[[i, 2]], a[[i, 3]], a[[i, 4]]);
        let ax0 = np_min(a1x, a2x) - m;
        let ax1 = np_max(a1x, a2x) + m;
        let ay0 = np_min(a1y, a2y) - m;
        let ay1 = np_max(a1y, a2y) + m;
        for j in 0..nb {
            let [bx0, bx1, by0, by1] = bb[j];
            if !(ax1 >= bx0 && ax0 <= bx1 && ay1 >= by0 && ay0 <= by1) {
                continue;
            }
            let (b1x, b1y, b2x, b2y, bn) = (b[[j, 0]], b[[j, 1]], b[[j, 2]], b[[j, 3]], b[[j, 4]]);
            if an == bn {
                continue;
            }
            let shared = ((a1x - b1x).abs() < eps && (a1y - b1y).abs() < eps)
                || ((a1x - b2x).abs() < eps && (a1y - b2y).abs() < eps)
                || ((a2x - b1x).abs() < eps && (a2y - b1y).abs() < eps)
                || ((a2x - b2x).abs() < eps && (a2y - b2y).abs() < eps);
            if shared {
                continue;
            }
            let inter = (ccw(a1x, a1y, b1x, b1y, b2x, b2y) != ccw(a2x, a2y, b1x, b1y, b2x, b2y))
                && (ccw(a1x, a1y, a2x, a2y, b1x, b1y) != ccw(a1x, a1y, a2x, a2y, b2x, b2y));
            if !inter {
                continue;
            }
            ii.push(i as i64);
            jj.push(j as i64);
            ww.push(match &w {
                Some(t) => np_max(t[an as i64 as usize], t[bn as i64 as usize]),
                None => 1.0,
            });
        }
    }
    (
        ii.into_pyarray(py),
        jj.into_pyarray(py),
        ww.into_pyarray(py),
    )
}

/// `connectivity.compute_mst_edges(points)` (Euclidean), exactly: Prim from
/// node 0, the lowest-numbered node among equal keys, a key lowered only by a
/// strictly shorter edge, each distance `sqrt(dx*dx + dy*dy)`. Computed row by
/// row, so it holds O(n) memory where the numpy form holds the n x n matrix.
/// Returns [(i, j, d)] in the order the edges join the tree.
#[pyfunction]
pub fn mst_prim(xs: PyReadonlyArray1<f64>, ys: PyReadonlyArray1<f64>) -> Vec<(i64, i64, f64)> {
    let xs = xs.as_array();
    let ys = ys.as_array();
    let n = xs.len();
    let mut edges: Vec<(i64, i64, f64)> = Vec::new();
    if n < 2 {
        return edges;
    }
    let dist = |i: usize, j: usize| -> f64 {
        let dx = xs[j] - xs[i];
        let dy = ys[j] - ys[i];
        (dx * dx + dy * dy).sqrt()
    };
    let mut in_tree = vec![false; n];
    let mut nearest = vec![0usize; n];
    let mut min_dist: Vec<f64> = (0..n).map(|j| dist(0, j)).collect();
    in_tree[0] = true;
    min_dist[0] = f64::INFINITY;
    for _ in 0..(n - 1) {
        // np.argmin: the first index of the smallest value (a NaN wins).
        let mut best = 0usize;
        for k in 1..n {
            let (v, bv) = (min_dist[k], min_dist[best]);
            if bv.is_nan() {
                break;
            }
            if v.is_nan() || v < bv {
                best = k;
            }
        }
        let best_dist = min_dist[best];
        if !best_dist.is_finite() {
            break;
        }
        in_tree[best] = true;
        min_dist[best] = f64::INFINITY;
        edges.push((nearest[best] as i64, best as i64, best_dist));
        for k in 0..n {
            if !in_tree[k] {
                let d = dist(best, k);
                if d < min_dist[k] {
                    min_dist[k] = d;
                    nearest[k] = best;
                }
            }
        }
    }
    edges
}

// ---------------------------------------------------------------------------
// The simple path's model (`py_placer/placement/simple.py`), priced for every
// move of its annealer: a net's Manhattan tree, a group's gap tree, tree edges
// against pads and against each other, and congestion. The trees keep the
// Python's float operations in order, its tie rules and numpy's own semantics
// where numpy computed it (its min/max on signed zeros); the module header
// says which kernels sum in another order.

/// `simple.mst_l1`: the Manhattan minimum spanning tree of (xs, ys) by Prim
/// from point 0 -- the lowest index among equal keys, a key lowered only by
/// a strictly shorter edge, each distance `|xi - xj| + |yi - yj|`, the total
/// summed in the order the points join. Returns (total, [(from, to)]).
#[pyfunction]
pub fn mst_l1(xs: PyReadonlyArray1<f64>, ys: PyReadonlyArray1<f64>) -> (f64, Vec<(i64, i64)>) {
    let xs = xs.as_array();
    let ys = ys.as_array();
    let n = xs.len();
    let mut edges: Vec<(i64, i64)> = Vec::new();
    if n < 2 {
        return (0.0, edges);
    }
    let dist = |i: usize, j: usize| -> f64 { (xs[i] - xs[j]).abs() + (ys[i] - ys[j]).abs() };
    let mut done = vec![false; n];
    let mut frm = vec![0usize; n];
    let mut best: Vec<f64> = (0..n).map(|j| dist(0, j)).collect();
    done[0] = true;
    best[0] = f64::INFINITY;
    let mut total = 0.0f64;
    for _ in 0..(n - 1) {
        let mut k = usize::MAX;
        let mut bk = f64::INFINITY;
        for i in 0..n {
            if !done[i] && best[i] < bk {
                k = i;
                bk = best[i];
            }
        }
        if k == usize::MAX {
            break;
        }
        done[k] = true;
        total += bk;
        edges.push((frm[k] as i64, k as i64));
        for i in 0..n {
            if !done[i] {
                let d = dist(k, i);
                if d < best[i] {
                    best[i] = d;
                    frm[i] = k;
                }
            }
        }
    }
    (total, edges)
}

/// Python's built-in `max(a, b)` on floats: `b` only when it compares
/// strictly greater, so the first argument wins ties (and a NaN `b`).
#[inline]
fn py_max(a: f64, b: f64) -> f64 {
    if b > a {
        b
    } else {
        a
    }
}

/// Python's built-in `min(a, b)` on floats: `b` only when strictly less.
#[inline]
fn py_min(a: f64, b: f64) -> f64 {
    if b < a {
        b
    } else {
        a
    }
}

/// `simple.rect_gap`: the Manhattan gap between two rects (x0, y0, x1, y1),
/// `max(0, max(a0, b0) - min(a2, b2)) + max(0, max(a1, b1) - min(a3, b3))`
/// with Python's `max`/`min`.
#[inline]
fn rect_gap(a: [f64; 4], b: [f64; 4]) -> f64 {
    py_max(0.0, py_max(a[0], b[0]) - py_min(a[2], b[2]))
        + py_max(0.0, py_max(a[1], b[1]) - py_min(a[3], b[3]))
}

/// `simple.gap_tree`: the minimum spanning tree over rects (rows x0, y0, x1,
/// y1), each edge weighted by `rect_gap`, by Prim from rect 0 -- the next
/// rect the first one not in the tree whose key no later one beats strictly
/// (Python's `min(..., key=)`), a key lowered only by a strictly smaller gap,
/// the total summed in the order the rects join. Returns (total, [(from, to,
/// gap)]) in that order.
#[pyfunction]
pub fn gap_tree(rects: PyReadonlyArray2<f64>) -> (f64, Vec<(i64, i64, f64)>) {
    let r = rects.as_array();
    let n = r.nrows();
    let mut edges: Vec<(i64, i64, f64)> = Vec::new();
    if n < 2 {
        return (0.0, edges);
    }
    let rs: Vec<[f64; 4]> = (0..n).map(|i| [r[[i, 0]], r[[i, 1]], r[[i, 2]], r[[i, 3]]]).collect();
    let mut done = vec![false; n];
    let mut frm = vec![0usize; n];
    let mut best: Vec<f64> = (0..n).map(|i| rect_gap(rs[0], rs[i])).collect();
    done[0] = true;
    let mut total = 0.0f64;
    for _ in 0..(n - 1) {
        let mut k = usize::MAX;
        for i in 0..n {
            if !done[i] && (k == usize::MAX || best[i] < best[k]) {
                k = i;
            }
        }
        done[k] = true;
        total += best[k];
        edges.push((frm[k] as i64, k as i64, best[k]));
        for i in 0..n {
            if !done[i] {
                let g = rect_gap(rs[k], rs[i]);
                if g < best[i] {
                    best[i] = g;
                    frm[i] = k;
                }
            }
        }
    }
    (total, edges)
}

/// `simple.mst_l1` over points already in the order `sorted()` gives them,
/// with each point's owning part: the same Prim as `mst_l1` (point 0 first,
/// the lowest index among equal keys, a key lowered only by a strictly
/// shorter edge, the total summed in the order the points join). Returns
/// (total, edges m x 4 as (x0, y0, x1, y1), ends m x 2 as the two points'
/// owners) -- the arrays `simple.Model._price_net` keeps.
#[pyfunction]
pub fn mst_l1_tree<'py>(
    py: Python<'py>,
    xs: PyReadonlyArray1<'py, f64>,
    ys: PyReadonlyArray1<'py, f64>,
    owners: PyReadonlyArray1<'py, i64>,
) -> (f64, Bound<'py, PyArray2<f64>>, Bound<'py, PyArray2<i64>>) {
    let xs = xs.as_array();
    let ys = ys.as_array();
    let own = owners.as_array();
    let n = xs.len();
    let mut ev: Vec<f64> = Vec::new();
    let mut nv: Vec<i64> = Vec::new();
    let mut total = 0.0f64;
    if n > SPARSE_MST_MIN {
        // a big net (a ground with hundreds of pads): Kruskal over each
        // point's nearest neighbour per octant, O(n log n) where Prim is O(n^2)
        let (xv, yv) = (xs.to_vec(), ys.to_vec());
        for (w, a, b) in manhattan_mst(&xv, &yv) {
            total += w;
            ev.extend_from_slice(&[xv[a], yv[a], xv[b], yv[b]]);
            nv.extend_from_slice(&[own[a], own[b]]);
        }
    } else if n >= 2 {
        let dist = |i: usize, j: usize| -> f64 { (xs[i] - xs[j]).abs() + (ys[i] - ys[j]).abs() };
        let mut done = vec![false; n];
        let mut frm = vec![0usize; n];
        let mut best: Vec<f64> = (0..n).map(|j| dist(0, j)).collect();
        done[0] = true;
        best[0] = f64::INFINITY;
        for _ in 0..(n - 1) {
            let mut k = usize::MAX;
            let mut bk = f64::INFINITY;
            for i in 0..n {
                if !done[i] && best[i] < bk {
                    k = i;
                    bk = best[i];
                }
            }
            if k == usize::MAX {
                break;
            }
            done[k] = true;
            total += bk;
            let f = frm[k];
            ev.extend_from_slice(&[xs[f], ys[f], xs[k], ys[k]]);
            nv.extend_from_slice(&[own[f], own[k]]);
            for i in 0..n {
                if !done[i] {
                    let d = dist(k, i);
                    if d < best[i] {
                        best[i] = d;
                        frm[i] = k;
                    }
                }
            }
        }
    }
    let m = nv.len() / 2;
    let e = Array2::from_shape_vec((m, 4), ev).expect("m x 4");
    let o = Array2::from_shape_vec((m, 2), nv).expect("m x 2");
    (total, e.into_pyarray(py), o.into_pyarray(py))
}

/// `simple.Model._local_crossings_bd`'s tally: every pair that crosses
/// properly between `m` and `o` (row-major), keyed (mlab[i], olab[j]), then
/// every pair of `m` with itself (i < j, different labels), keyed
/// (mlab[i], mlab[j]); counted per key in the order keys are first met.
/// `o` rows whose label is in `skip` are left out. Returns (pairs, [(a, b,
/// count)]).
#[pyfunction]
#[pyo3(signature = (m, mlab, o=None, olab=None, skip=None))]
pub fn seg_cross_counts(
    m: PyReadonlyArray2<f64>,
    mlab: PyReadonlyArray1<i64>,
    o: Option<PyReadonlyArray2<f64>>,
    olab: Option<PyReadonlyArray1<i64>>,
    skip: Option<PyReadonlyArray1<i64>>,
) -> (i64, Vec<(i64, i64, i64)>) {
    let skip: HashSet<i64> = skip.map(|s| s.as_array().iter().copied().collect()).unwrap_or_default();
    let m = m.as_array();
    let ml = mlab.as_array();
    let row = |a: &numpy::ndarray::ArrayView2<f64>, i: usize| [a[[i, 0]], a[[i, 1]], a[[i, 2]], a[[i, 3]]];
    let mut out: Vec<(i64, i64, i64)> = Vec::new();
    let mut at: HashMap<(i64, i64), usize> = HashMap::new();
    let mut total = 0i64;
    let mut add = |a: i64, b: i64, out: &mut Vec<(i64, i64, i64)>| {
        total += 1;
        match at.get(&(a, b)) {
            Some(&k) => out[k].2 += 1,
            None => {
                at.insert((a, b), out.len());
                out.push((a, b, 1));
            }
        }
    };
    if let (Some(o), Some(ol)) = (o, olab) {
        let o = o.as_array();
        let ol = ol.as_array();
        // two segments whose boxes do not overlap cannot cross properly: each
        // `o` row is tested against the union box of `m`, then row by row
        let mb: Vec<[f64; 4]> = (0..m.nrows()).map(|i| seg_box(row(&m, i))).collect();
        let ub = mb.iter().fold([f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY],
                                |u, b| [u[0].min(b[0]), u[1].min(b[1]), u[2].max(b[2]), u[3].max(b[3])]);
        for j in 0..o.nrows() {
            if skip.contains(&ol[j]) {
                continue;
            }
            let rj = row(&o, j);
            let bj = seg_box(rj);
            if !boxes_meet(&ub, &bj) {
                continue;
            }
            for i in 0..m.nrows() {
                if boxes_meet(&mb[i], &bj) && proper_cross(row(&m, i), rj) {
                    add(ml[i], ol[j], &mut out);
                }
            }
        }
    }
    for i in 0..m.nrows() {
        let ri = row(&m, i);
        for j in (i + 1)..m.nrows() {
            if ml[i] != ml[j] && proper_cross(ri, row(&m, j)) {
                add(ml[i], ml[j], &mut out);
            }
        }
    }
    (total, out)
}

/// Above this many segment x rect pairs (and `GRID_MIN` rects) `pad_hit_sums`
/// buckets the rects in a `Grid`.
const GRID_PAIRS: usize = 4096;
const GRID_MIN: usize = 32;

/// Boxes bucketed on a uniform grid over their union box `ub`, about one per
/// cell: `gather` gives the boxes whose buckets a query box meets -- every box
/// it can meet -- each once, in ascending order.
struct Grid {
    x0: f64,
    y0: f64,
    cw: f64,
    ch: f64,
    g: usize,
    cells: Vec<Vec<usize>>,
}

impl Grid {
    fn new(ub: [f64; 4], boxes: impl Iterator<Item = [f64; 4]>) -> Grid {
        let boxes: Vec<[f64; 4]> = boxes.collect();
        let g = ((boxes.len() as f64).sqrt().ceil() as usize).clamp(1, 64);
        let cw = ((ub[2] - ub[0]) / g as f64).max(1e-9);
        let ch = ((ub[3] - ub[1]) / g as f64).max(1e-9);
        let mut grid = Grid { x0: ub[0], y0: ub[1], cw, ch, g, cells: vec![Vec::new(); g * g] };
        for (k, b) in boxes.iter().enumerate() {
            if let Some((c0, c1, r0, r1)) = grid.span(*b) {
                for r in r0..=r1 {
                    for c in c0..=c1 {
                        grid.cells[r * g + c].push(k);
                    }
                }
            }
        }
        grid
    }

    /// The columns and rows of buckets box `b` meets, clamped to the grid;
    /// None for a box with a NaN side, which meets nothing.
    fn span(&self, b: [f64; 4]) -> Option<(usize, usize, usize, usize)> {
        if b.iter().any(|v| v.is_nan()) {
            return None;
        }
        let top = (self.g - 1) as f64;
        let ix = |v: f64, o: f64, w: f64| ((v - o) / w).floor().clamp(0.0, top) as usize;
        Some((ix(b[0], self.x0, self.cw), ix(b[2], self.x0, self.cw), ix(b[1], self.y0, self.ch),
              ix(b[3], self.y0, self.ch)))
    }

    /// Into `out`, ascending: the boxes in the buckets `b` meets, each once
    /// (`stamp[k] == tag` marks one already taken for this query).
    fn gather(&self, b: [f64; 4], tag: usize, stamp: &mut [usize], out: &mut Vec<usize>) {
        out.clear();
        if let Some((c0, c1, r0, r1)) = self.span(b) {
            for r in r0..=r1 {
                for c in c0..=c1 {
                    for &k in &self.cells[r * self.g + c] {
                        if stamp[k] != tag {
                            stamp[k] = tag;
                            out.push(k);
                        }
                    }
                }
            }
        }
        out.sort_unstable();
    }
}

/// A segment's box (x0, y0, x1, y1 as min x, min y, max x, max y).
#[inline]
fn seg_box(a: [f64; 4]) -> [f64; 4] {
    [a[0].min(a[2]), a[1].min(a[3]), a[0].max(a[2]), a[1].max(a[3])]
}

/// Two closed boxes (min x, min y, max x, max y) share a point.
#[inline]
fn boxes_meet(a: &[f64; 4], b: &[f64; 4]) -> bool {
    a[0] <= b[2] && b[0] <= a[2] && a[1] <= b[3] && b[1] <= a[3]
}

/// `simple.Model._pad_breakdown`: the segments' hits on rects (the
/// `seg_rect_hits` test, row-major) kept when foreign -- another net than the
/// pad's and neither of the segment's two end parts the pad's owner -- with
/// each pad's weight summed per (segment's net, pad's owner), starting at 0.0
/// and in hit order, keys in the order first met. Segments of a net in
/// `skip_nets` and pads of an owner in `skip_owners` are left out. Returns
/// [(net, owner, w)].
#[pyfunction]
#[pyo3(signature = (s, ends, labs, p, w, pnets, owners, skip_nets=None, skip_owners=None))]
#[allow(clippy::too_many_arguments)]
pub fn pad_hit_sums(
    s: PyReadonlyArray2<f64>,
    ends: PyReadonlyArray2<i64>,
    labs: PyReadonlyArray1<i64>,
    p: PyReadonlyArray2<f64>,
    w: PyReadonlyArray1<f64>,
    pnets: PyReadonlyArray1<i64>,
    owners: PyReadonlyArray1<i64>,
    skip_nets: Option<PyReadonlyArray1<i64>>,
    skip_owners: Option<PyReadonlyArray1<i64>>,
) -> Vec<(i64, i64, f64)> {
    let skip_n: HashSet<i64> = skip_nets.map(|s| s.as_array().iter().copied().collect()).unwrap_or_default();
    let skip_o: HashSet<i64> = skip_owners.map(|s| s.as_array().iter().copied().collect()).unwrap_or_default();
    let s = s.as_array();
    let ends = ends.as_array();
    let labs = labs.as_array();
    let p = p.as_array();
    let w = w.as_array();
    let pn = pnets.as_array();
    let po = owners.as_array();
    let mut out: Vec<(i64, i64, f64)> = Vec::new();
    let mut at: HashMap<(i64, i64), usize> = HashMap::new();
    // the segments kept, and the rects their union box meets (NaN rows,
    // which `_index_part` leaves for rows a part no longer has, meet none)
    let segs: Vec<usize> = (0..s.nrows()).filter(|&i| !skip_n.contains(&labs[i])).collect();
    let mut ub = [f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY];
    for &i in &segs {
        let b = seg_box([s[[i, 0]], s[[i, 1]], s[[i, 2]], s[[i, 3]]]);
        ub = [ub[0].min(b[0]), ub[1].min(b[1]), ub[2].max(b[2]), ub[3].max(b[3])];
    }
    let rects: Vec<usize> = (0..p.nrows())
        .filter(|&j| {
            !skip_o.contains(&po[j]) && boxes_meet(&ub, &[p[[j, 0]], p[[j, 1]], p[[j, 2]], p[[j, 3]]])
        })
        .collect();
    // ... and the segments the rects' union box meets: one missing it misses
    // every rect
    let mut rb = [f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY];
    for &j in &rects {
        rb = [rb[0].min(p[[j, 0]]), rb[1].min(p[[j, 1]]), rb[2].max(p[[j, 2]]), rb[3].max(p[[j, 3]])];
    }
    let segs: Vec<usize> = segs
        .into_iter()
        .filter(|&i| boxes_meet(&rb, &seg_box([s[[i, 0]], s[[i, 1]], s[[i, 2]], s[[i, 3]]])))
        .collect();
    // many pairs: the rects bucketed, each segment testing those in the
    // buckets its box meets, in their order
    let grid = if segs.len() * rects.len() > GRID_PAIRS && rects.len() > GRID_MIN {
        Some(Grid::new(rb, rects.iter().map(|&j| [p[[j, 0]], p[[j, 1]], p[[j, 2]], p[[j, 3]]])))
    } else {
        None
    };
    let mut stamp = vec![usize::MAX; rects.len()];
    let mut cand: Vec<usize> = Vec::new();
    let all: Vec<usize> = (0..rects.len()).collect();
    for &i in &segs {
        let (x0, y0, x1, y1) = (s[[i, 0]], s[[i, 1]], s[[i, 2]], s[[i, 3]]);
        let (sx0, sx1) = (np_min(x0, x1), np_max(x0, x1));
        let (sy0, sy1) = (np_min(y0, y1), np_max(y0, y1));
        let dx = x1 - x0;
        let dy = y1 - y0;
        let local: &[usize] = match &grid {
            Some(g) => {
                g.gather(seg_box([x0, y0, x1, y1]), i, &mut stamp, &mut cand);
                &cand
            }
            None => &all,
        };
        for &jl in local {
            let j = rects[jl];
            let (px0, py0, px1, py1) = (p[[j, 0]], p[[j, 1]], p[[j, 2]], p[[j, 3]]);
            if !(sx0 <= px1 && sx1 >= px0 && sy0 <= py1 && sy1 >= py0) {
                continue;
            }
            let mut lo = 0.0f64;
            let mut hi = 1.0f64;
            let mut ok = true;
            for (d, q) in [(-dx, x0 - px0), (dx, px1 - x0), (-dy, y0 - py0), (dy, py1 - y0)] {
                let zero = d.abs() < 1e-12;
                if zero && q < 0.0 {
                    ok = false;
                }
                let t = q / if zero { 1.0 } else { d };
                if !zero && d < 0.0 {
                    lo = np_max(lo, t);
                }
                if !zero && d > 0.0 {
                    hi = np_min(hi, t);
                }
            }
            if !(ok && lo <= hi) {
                continue;
            }
            if labs[i] == pn[j] || ends[[i, 0]] == po[j] || ends[[i, 1]] == po[j] {
                continue;
            }
            let key = (labs[i], po[j]);
            match at.get(&key) {
                Some(&k) => out[k].2 += w[j],
                None => {
                    at.insert(key, out.len());
                    out.push((key.0, key.1, 0.0 + w[j]));
                }
            }
        }
    }
    out
}

/// numpy's `np.maximum(x, 0.0)` as `np.clip(x, 0.0, None)` gives it: x when
/// it is greater, else +0.0 (a -0.0 included).
#[inline]
fn clip0(x: f64) -> f64 {
    if x > 0.0 || x.is_nan() {
        x
    } else {
        0.0
    }
}

/// numpy's `np.minimum` / `np.maximum` on two floats: a NaN wins, and -0.0
/// is less than +0.0.
#[inline]
fn npy_min(a: f64, b: f64) -> f64 {
    if a.is_nan() || b.is_nan() {
        f64::NAN
    } else if a < b || (a == b && a.is_sign_negative()) {
        a
    } else {
        b
    }
}

#[inline]
fn npy_max(a: f64, b: f64) -> f64 {
    if a.is_nan() || b.is_nan() {
        f64::NAN
    } else if a > b || (a == b && a.is_sign_positive()) {
        a
    } else {
        b
    }
}

/// `simple.Model._seg_hits`: every (segment i, rect j) that touch -- a bounding
/// box test, then Liang-Barsky clipping of the segment to the rect, a step
/// with |d| < 1e-12 taken as parallel. Segments are (x0, y0, x1, y1), rects
/// (x0, y0, x1, y1). Returns (ii, jj) in row-major order.
#[pyfunction]
pub fn seg_rect_hits<'py>(
    py: Python<'py>,
    s: PyReadonlyArray2<'py, f64>,
    p: PyReadonlyArray2<'py, f64>,
) -> (Bound<'py, PyArray1<i64>>, Bound<'py, PyArray1<i64>>) {
    let s = s.as_array();
    let p = p.as_array();
    let mut ii: Vec<i64> = Vec::new();
    let mut jj: Vec<i64> = Vec::new();
    for i in 0..s.nrows() {
        let (x0, y0, x1, y1) = (s[[i, 0]], s[[i, 1]], s[[i, 2]], s[[i, 3]]);
        let (sx0, sx1) = (np_min(x0, x1), np_max(x0, x1));
        let (sy0, sy1) = (np_min(y0, y1), np_max(y0, y1));
        let dx = x1 - x0;
        let dy = y1 - y0;
        for j in 0..p.nrows() {
            let (px0, py0, px1, py1) = (p[[j, 0]], p[[j, 1]], p[[j, 2]], p[[j, 3]]);
            if !(sx0 <= px1 && sx1 >= px0 && sy0 <= py1 && sy1 >= py0) {
                continue;
            }
            let mut lo = 0.0f64;
            let mut hi = 1.0f64;
            let mut ok = true;
            for (d, q) in [(-dx, x0 - px0), (dx, px1 - x0), (-dy, y0 - py0), (dy, py1 - y0)] {
                let zero = d.abs() < 1e-12;
                if zero && q < 0.0 {
                    ok = false;
                }
                let t = q / if zero { 1.0 } else { d };
                if !zero && d < 0.0 {
                    lo = np_max(lo, t);
                }
                if !zero && d > 0.0 {
                    hi = np_min(hi, t);
                }
            }
            if ok && lo <= hi {
                ii.push(i as i64);
                jj.push(j as i64);
            }
        }
    }
    (ii.into_pyarray(py), jj.into_pyarray(py))
}

/// numpy's sign of `simple._orient`: (bx - ax)*(cy - ay) - (by - ay)*(cx - ax).
#[inline]
fn orient(ax: f64, ay: f64, bx: f64, by: f64, cx: f64, cy: f64) -> f64 {
    let v = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
    if v > 0.0 {
        1.0
    } else if v < 0.0 {
        -1.0
    } else {
        v // 0.0, or NaN as np.sign gives it
    }
}

#[inline]
fn proper_cross(a: [f64; 4], b: [f64; 4]) -> bool {
    let o1 = orient(a[0], a[1], a[2], a[3], b[0], b[1]);
    let o2 = orient(a[0], a[1], a[2], a[3], b[2], b[3]);
    if !(o1 * o2 < 0.0) {
        return false;
    }
    let o3 = orient(b[0], b[1], b[2], b[3], a[0], a[1]);
    let o4 = orient(b[0], b[1], b[2], b[3], a[2], a[3]);
    o3 * o4 < 0.0
}

/// `simple.crossings`: how many (a row, b row) pairs cross properly
/// (each strictly on either side of the other's line). With `labels`
/// (`b` None) the pairs are those of `a` with itself, i < j, of different
/// labels -- a set of nets' edges among themselves.
#[pyfunction]
#[pyo3(signature = (a, b=None, labels=None))]
pub fn seg_crossings(
    a: PyReadonlyArray2<f64>,
    b: Option<PyReadonlyArray2<f64>>,
    labels: Option<PyReadonlyArray1<i64>>,
) -> i64 {
    let a = a.as_array();
    let row = |m: &numpy::ndarray::ArrayView2<f64>, i: usize| [m[[i, 0]], m[[i, 1]], m[[i, 2]], m[[i, 3]]];
    let mut count = 0i64;
    match b {
        Some(b) => {
            let b = b.as_array();
            for i in 0..a.nrows() {
                let ra = row(&a, i);
                for j in 0..b.nrows() {
                    if proper_cross(ra, row(&b, j)) {
                        count += 1;
                    }
                }
            }
        }
        None => {
            let lab = labels.map(|l| l.as_array().to_vec());
            for i in 0..a.nrows() {
                let ra = row(&a, i);
                for j in (i + 1)..a.nrows() {
                    if let Some(l) = &lab {
                        if l[i] == l[j] {
                            continue;
                        }
                    }
                    if proper_cross(ra, row(&a, j)) {
                        count += 1;
                    }
                }
            }
        }
    }
    count
}

/// `seg_crossings`, returning the crossing pairs instead of their count:
/// (ii, jj) over (a row, b row) in row-major order, or with `labels` (`b`
/// None) over a's rows with i < j of different labels.
#[pyfunction]
#[pyo3(signature = (a, b=None, labels=None))]
pub fn seg_cross_pairs<'py>(
    py: Python<'py>,
    a: PyReadonlyArray2<'py, f64>,
    b: Option<PyReadonlyArray2<'py, f64>>,
    labels: Option<PyReadonlyArray1<'py, i64>>,
) -> (Bound<'py, PyArray1<i64>>, Bound<'py, PyArray1<i64>>) {
    let a = a.as_array();
    let row = |m: &numpy::ndarray::ArrayView2<f64>, i: usize| [m[[i, 0]], m[[i, 1]], m[[i, 2]], m[[i, 3]]];
    let mut ii: Vec<i64> = Vec::new();
    let mut jj: Vec<i64> = Vec::new();
    match b {
        Some(b) => {
            let b = b.as_array();
            for i in 0..a.nrows() {
                let ra = row(&a, i);
                for j in 0..b.nrows() {
                    if proper_cross(ra, row(&b, j)) {
                        ii.push(i as i64);
                        jj.push(j as i64);
                    }
                }
            }
        }
        None => {
            let lab = labels.map(|l| l.as_array().to_vec());
            for i in 0..a.nrows() {
                let ra = row(&a, i);
                for j in (i + 1)..a.nrows() {
                    if let Some(l) = &lab {
                        if l[i] == l[j] {
                            continue;
                        }
                    }
                    if proper_cross(ra, row(&a, j)) {
                        ii.push(i as i64);
                        jj.push(j as i64);
                    }
                }
            }
        }
    }
    (ii.into_pyarray(py), jj.into_pyarray(py))
}

/// The window `_win` clips the box [x0, x1] x [y0, y1] to, and each cell's
/// overlap with the box along x (`cx`) and y (`cy`), `ex` / `ey` the grid
/// lines. None when the window is empty.
#[allow(clippy::too_many_arguments)]
fn box_cover(
    x0: f64,
    x1: f64,
    y0: f64,
    y1: f64,
    ex: &numpy::ndarray::ArrayView1<f64>,
    ey: &numpy::ndarray::ArrayView1<f64>,
    gx0: f64,
    gy0: f64,
    cell: f64,
    nx: i64,
    ny: i64,
) -> Option<(i64, i64, i64, i64, Vec<f64>, Vec<f64>)> {
    let clamp = |v: f64, hi: i64| -> i64 { (v as i64).clamp(0, hi) };
    let j0 = clamp(((x0 - gx0) / cell).floor(), nx);
    let j1 = clamp(((x1 - gx0) / cell).floor() + 1.0, nx);
    let i0 = clamp(((y0 - gy0) / cell).floor(), ny);
    let i1 = clamp(((y1 - gy0) / cell).floor() + 1.0, ny);
    if i1 <= i0 || j1 <= j0 {
        return None;
    }
    let cx: Vec<f64> = (j0..j1)
        .map(|j| clip0(npy_min(ex[(j + 1) as usize], x1) - npy_max(ex[j as usize], x0)))
        .collect();
    let cy: Vec<f64> = (i0..i1)
        .map(|i| clip0(npy_min(ey[(i + 1) as usize], y1) - npy_max(ey[i as usize], y0)))
        .collect();
    Some((i0, i1, j0, j1, cy, cx))
}

/// `simple.Model._net_demand` as its two factors: a tree's share of the
/// demand is s * outer(cy, cx) over the window (i0, i1, j0, j1) -- the
/// edges' box grown by `h`, each cell's overlap with it along y and x, and
/// s = length / box area. None when the tree or the window is empty.
#[pyfunction]
#[allow(clippy::too_many_arguments)]
#[allow(clippy::type_complexity)]
pub fn net_box<'py>(
    py: Python<'py>,
    e: PyReadonlyArray2<'py, f64>,
    length: f64,
    h: f64,
    ex: PyReadonlyArray1<'py, f64>,
    ey: PyReadonlyArray1<'py, f64>,
    gx0: f64,
    gy0: f64,
    cell: f64,
    nx: i64,
    ny: i64,
) -> Option<(i64, i64, i64, i64, Bound<'py, PyArray1<f64>>, Bound<'py, PyArray1<f64>>, f64)> {
    let e = e.as_array();
    let m = e.nrows();
    if m == 0 {
        return None;
    }
    let colmin = |c: usize| (1..m).fold(e[[0, c]], |acc, i| npy_min(acc, e[[i, c]]));
    let colmax = |c: usize| (1..m).fold(e[[0, c]], |acc, i| npy_max(acc, e[[i, c]]));
    let x0 = py_min(colmin(0), colmin(2)) - h;
    let x1 = py_max(colmax(0), colmax(2)) + h;
    let y0 = py_min(colmin(1), colmin(3)) - h;
    let y1 = py_max(colmax(1), colmax(3)) + h;
    let (i0, i1, j0, j1, cy, cx) = box_cover(x0, x1, y0, y1, &ex.as_array(), &ey.as_array(), gx0, gy0, cell, nx, ny)?;
    let s = length / ((x1 - x0) * (y1 - y0));
    Some((i0, i1, j0, j1, cy.into_pyarray(py), cx.into_pyarray(py), s))
}

/// `simple.Model._part_use`: a part's pad rects `a` (rows x0, x1, y0, y1,
/// already grown) and `on` (row p, column L: pad p is on layer L), as the
/// union window of the rects and, per layer some pad is on, the sum over its
/// pads of outer(cy, cx) over that window. None when the window is empty.
#[pyfunction]
#[allow(clippy::too_many_arguments)]
#[allow(clippy::type_complexity)]
pub fn pad_use<'py>(
    py: Python<'py>,
    a: PyReadonlyArray2<'py, f64>,
    on: PyReadonlyArray2<'py, bool>,
    ex: PyReadonlyArray1<'py, f64>,
    ey: PyReadonlyArray1<'py, f64>,
    gx0: f64,
    gy0: f64,
    cell: f64,
    nx: i64,
    ny: i64,
) -> Option<(i64, i64, i64, i64, Vec<(i64, Bound<'py, PyArray2<f64>>)>)> {
    let a = a.as_array();
    let on = on.as_array();
    let k = a.nrows();
    if k == 0 {
        return None;
    }
    let ex = ex.as_array();
    let ey = ey.as_array();
    let fold = |c: usize, f: fn(f64, f64) -> f64| (1..k).fold(a[[0, c]], |acc, i| f(acc, a[[i, c]]));
    let (wx0, wx1, wy0, wy1) = (fold(0, f64::min), fold(1, f64::max), fold(2, f64::min), fold(3, f64::max));
    let (i0, i1, j0, j1, _cy, _cx) = box_cover(wx0, wx1, wy0, wy1, &ex, &ey, gx0, gy0, cell, nx, ny)?;
    let (h, w) = ((i1 - i0) as usize, (j1 - j0) as usize);
    let nl = on.ncols();
    let mut blocks: Vec<Option<Vec<f64>>> = vec![None; nl];
    for p in 0..k {
        let (x0, x1, y0, y1) = (a[[p, 0]], a[[p, 1]], a[[p, 2]], a[[p, 3]]);
        let cx: Vec<f64> = (j0..j1)
            .map(|j| clip0(npy_min(ex[(j + 1) as usize], x1) - npy_max(ex[j as usize], x0)))
            .collect();
        let cy: Vec<f64> = (i0..i1)
            .map(|i| clip0(npy_min(ey[(i + 1) as usize], y1) - npy_max(ey[i as usize], y0)))
            .collect();
        for l in 0..nl {
            if !on[[p, l]] {
                continue;
            }
            let b = blocks[l].get_or_insert_with(|| vec![0.0; h * w]);
            for (r, yv) in cy.iter().enumerate() {
                if *yv == 0.0 {
                    continue;
                }
                let row = &mut b[r * w..(r + 1) * w];
                for (q, xv) in cx.iter().enumerate() {
                    row[q] += yv * xv;
                }
            }
        }
    }
    let out = blocks
        .into_iter()
        .enumerate()
        .filter_map(|(l, b)| {
            b.map(|b| (l as i64, Array2::from_shape_vec((h, w), b).expect("block").into_pyarray(py)))
        })
        .collect();
    Some((i0, i1, j0, j1, out))
}

/// `simple.Model._cong_price`: the change a move makes to the overflow total
/// sum(clip(demand - capacity / pitch, 0)^2) over the window (i0, i1, j0,
/// j1). `d` is the committed demand and `c` the committed capacity per cell
/// (over the layers, clip(cell2 - use, 0) summed), `uses` the committed use
/// per layer. Each demand term (s, i0, j0, cy, cx) adds s * cy[a] * cx[b] at
/// cell (i0 + a, j0 + b); each use term (layer, sign, i0, j0, block) adds
/// sign * block to that layer's use, and the capacity under the use terms is
/// recomputed. Only cells a term reaches are visited.
#[pyfunction]
#[allow(clippy::too_many_arguments)]
#[allow(clippy::type_complexity)]
pub fn cong_delta(
    d: PyReadonlyArray2<f64>,
    c: PyReadonlyArray2<f64>,
    uses: Vec<PyReadonlyArray2<f64>>,
    win: (usize, usize, usize, usize),
    dem: Vec<(f64, usize, usize, PyReadonlyArray1<f64>, PyReadonlyArray1<f64>)>,
    usev: Vec<(usize, f64, usize, usize, PyReadonlyArray2<f64>)>,
    cell2: f64,
    pitch: f64,
) -> f64 {
    let d = d.as_array();
    let c = c.as_array();
    let (i0, i1, j0, j1) = win;
    if i1 <= i0 || j1 <= j0 {
        return 0.0;
    }
    let (h, w) = (i1 - i0, j1 - j0);
    let mut db = vec![0.0f64; h * w];
    for (s, ti0, tj0, cy, cx) in &dem {
        let cy = cy.as_array();
        let cx = cx.as_array();
        for (a, yv) in cy.iter().enumerate() {
            let ya = s * yv;
            if ya == 0.0 {
                continue;
            }
            let r = ti0 + a - i0;
            let row = &mut db[r * w + (tj0 - j0)..r * w + (tj0 - j0) + cx.len()];
            for (q, xv) in cx.iter().enumerate() {
                row[q] += ya * xv;
            }
        }
    }
    // the use terms' union box, its new use per layer, then its capacity
    let (mut ui0, mut ui1, mut uj0, mut uj1) = (usize::MAX, 0usize, usize::MAX, 0usize);
    for (_l, _s, ti0, tj0, b) in &usev {
        let b = b.as_array();
        ui0 = ui0.min(*ti0);
        uj0 = uj0.min(*tj0);
        ui1 = ui1.max(ti0 + b.nrows());
        uj1 = uj1.max(tj0 + b.ncols());
    }
    let mut cap_u: Vec<f64> = Vec::new();
    let (uh, uw) = if ui1 > ui0 && uj1 > uj0 { (ui1 - ui0, uj1 - uj0) } else { (0, 0) };
    if uh > 0 {
        let nl = uses.len();
        let mut du = vec![0.0f64; nl * uh * uw];
        for (l, s, ti0, tj0, b) in &usev {
            let b = b.as_array();
            for r in 0..b.nrows() {
                for q in 0..b.ncols() {
                    du[l * uh * uw + (ti0 - ui0 + r) * uw + (tj0 - uj0 + q)] += s * b[[r, q]];
                }
            }
        }
        let us: Vec<_> = uses.iter().map(|u| u.as_array()).collect();
        cap_u = vec![0.0f64; uh * uw];
        for r in 0..uh {
            for q in 0..uw {
                let mut cap = 0.0f64;
                for (l, u) in us.iter().enumerate() {
                    cap += clip0(cell2 - (u[[ui0 + r, uj0 + q]] + du[l * uh * uw + r * uw + q]));
                }
                cap_u[r * uw + q] = cap;
            }
        }
    }
    let mut acc = 0.0f64;
    for r in 0..h {
        let gi = i0 + r;
        let in_r = uh > 0 && gi >= ui0 && gi < ui1;
        for q in 0..w {
            let gj = j0 + q;
            let dd = db[r * w + q];
            let in_u = in_r && gj >= uj0 && gj < uj1;
            if dd == 0.0 && !in_u {
                continue;
            }
            let dold = d[[gi, gj]];
            let cold = c[[gi, gj]];
            let cnew = if in_u { cap_u[(gi - ui0) * uw + (gj - uj0)] } else { cold };
            let oo = clip0(dold - cold / pitch);
            let on = clip0(dold + dd - cnew / pitch);
            acc += on * on - oo * oo;
        }
    }
    acc
}

/// Above this many points `mst_l1_tree` builds its tree from octant
/// neighbours rather than by Prim: the same length, other ties.
const SPARSE_MST_MIN: usize = 160;

/// f64 to a key that sorts as the float does (no NaN expected).
#[inline]
fn ord_key(v: f64) -> u64 {
    let b = v.to_bits();
    if b >> 63 == 1 {
        !b
    } else {
        b | (1u64 << 63)
    }
}

/// The Manhattan minimum spanning tree of (xs, ys) as (length, a, b) edges,
/// lighter first: each point's nearest neighbour in four octants (the other
/// four are the same edges seen from the far end) by a Fenwick-tree sweep --
/// the candidate set contains a minimum spanning tree -- then Kruskal over
/// them. Any minimum spanning tree has the same weights, so the lengths it
/// gives, summed in this order, are the same to the bit whichever of two
/// equal edges it keeps. Points must be distinct.
fn manhattan_mst(xs: &[f64], ys: &[f64]) -> Vec<(f64, usize, usize)> {
    let n = xs.len();
    if n < 2 {
        return Vec::new();
    }
    // each coordinate's rank, equal values one rank
    let rank_of = |v: &[f64]| -> (Vec<usize>, usize) {
        let mut o: Vec<(u64, u32)> = (0..n).map(|p| (ord_key(v[p]), p as u32)).collect();
        o.sort_unstable();
        let mut rank = vec![0usize; n];
        let mut r = 0usize;
        for k in 0..n {
            if k > 0 && o[k].0 != o[k - 1].0 {
                r += 1;
            }
            rank[o[k].1 as usize] = r;
        }
        (rank, r + 1)
    };
    let (rx, mx) = rank_of(xs);
    let (ry, my) = rank_of(ys);
    // the sweep in the frame (px, py) = (x, y), (y, x), (-x, y), (y, -x): the
    // points in decreasing py - px (ties: decreasing px), each asking the
    // points already met with px no less than its own for the least px + py
    // -- its nearest in the octant dx >= 0, dy >= dx. Frames 1 and 3 take
    // frames 0 and 2's orders reversed, each run of equal keys put back.
    let dif: Vec<f64> = (0..n).map(|p| ys[p] - xs[p]).collect();
    let sum: Vec<f64> = (0..n).map(|p| xs[p] + ys[p]).collect();
    let sorted = |key: &Vec<f64>, asc_x: bool| -> Vec<u32> {
        let mut k: Vec<(u64, u64, u32)> = (0..n)
            .map(|p| (!ord_key(key[p]), if asc_x { ord_key(xs[p]) } else { !ord_key(xs[p]) }, p as u32))
            .collect();
        k.sort_unstable();
        k.iter().map(|e| e.2).collect()
    };
    let flip = |o: &Vec<u32>, key: &Vec<f64>| -> Vec<u32> {
        let mut r: Vec<u32> = o.iter().rev().copied().collect();
        let mut s = 0;
        while s < n {
            let mut e = s + 1;
            while e < n && key[r[e] as usize] == key[r[s] as usize] {
                e += 1;
            }
            r[s..e].reverse();
            s = e;
        }
        r
    };
    let o0 = sorted(&dif, false);
    let o2 = sorted(&sum, true);
    let o1 = flip(&o0, &dif);
    let o3 = flip(&o2, &sum);
    let mut cand: Vec<(u64, u32, u32)> = Vec::with_capacity(4 * n);
    let mut bv = vec![f64::INFINITY; n + 1];
    let mut bi = vec![u32::MAX; n + 1];
    for t in 0..4 {
        let (order, m) = match t {
            0 => (&o0, mx),
            1 => (&o1, my),
            2 => (&o2, mx),
            _ => (&o3, my),
        };
        for k in 0..=m {
            bv[k] = f64::INFINITY;
            bi[k] = u32::MAX;
        }
        for &i in order.iter() {
            let i = i as usize;
            // the Fenwick slot: m - rank(px), so a prefix is px no less
            let (slot, v) = match t {
                0 => (m - rx[i], xs[i] + ys[i]),
                1 => (m - ry[i], ys[i] + xs[i]),
                2 => (rx[i] + 1, -xs[i] + ys[i]),
                _ => (m - ry[i], ys[i] + -xs[i]),
            };
            let mut k = slot;
            let (mut best, mut who) = (f64::INFINITY, u32::MAX);
            while k > 0 {
                if bv[k] < best {
                    best = bv[k];
                    who = bi[k];
                }
                k &= k - 1;
            }
            if who != u32::MAX {
                let j = who as usize;
                let w = (xs[i] - xs[j]).abs() + (ys[i] - ys[j]).abs();
                cand.push((w.to_bits(), i.min(j) as u32, i.max(j) as u32));
            }
            let mut k = slot;
            while k <= m {
                if v < bv[k] {
                    bv[k] = v;
                    bi[k] = i as u32;
                }
                k += k & k.wrapping_neg();
            }
        }
    }
    // a length is never negative, so its bits sort as it does
    cand.sort_unstable_by_key(|e| e.0);
    let mut parent: Vec<u32> = (0..n as u32).collect();
    let find = |p: &mut Vec<u32>, mut x: u32| -> u32 {
        while p[x as usize] != x {
            let g = p[p[x as usize] as usize];
            p[x as usize] = g;
            x = g;
        }
        x
    };
    let mut out = Vec::with_capacity(n - 1);
    for (w, a, b) in cand {
        let (ra, rb) = (find(&mut parent, a), find(&mut parent, b));
        if ra != rb {
            parent[ra as usize] = rb;
            out.push((f64::from_bits(w), a as usize, b as usize));
            if out.len() + 1 == n {
                break;
            }
        }
    }
    out
}

/// `simple_anneal.Annealer.overlap(refs)`'s pair sum: from `total` (the
/// moving parts' off-board and intent terms), for each moving part `a` (ids
/// `mine`, ascending, with live rects `live`) and every other part `b` in id
/// order -- a moving one only when its id is above `a`'s -- sharing a face
/// (`faces[a] & faces[b]`), the area where the two courtyards grown by `h`
/// overlap, added in that order. Parts not moving are read at their
/// committed rects (`crects`, row = id). Every part is looked at, so the
/// pairs a spatial index would offer are among them: the same sum.
#[pyfunction]
pub fn overlap_sum(
    total: f64,
    h: f64,
    mine: Vec<i64>,
    live: PyReadonlyArray2<f64>,
    crects: PyReadonlyArray2<f64>,
    faces: PyReadonlyArray1<i64>,
) -> f64 {
    let live = live.as_array();
    let cr = crects.as_array();
    let faces = faces.as_array();
    let n = cr.nrows();
    let grow = |c: [f64; 4]| [c[0] - h, c[1] - h, c[2] + h, c[3] + h];
    let mut slot = vec![usize::MAX; n];
    for (k, &p) in mine.iter().enumerate() {
        slot[p as usize] = k;
    }
    let rect = |b: usize| -> [f64; 4] {
        match slot[b] {
            usize::MAX => grow([cr[[b, 0]], cr[[b, 1]], cr[[b, 2]], cr[[b, 3]]]),
            k => grow([live[[k, 0]], live[[k, 1]], live[[k, 2]], live[[k, 3]]]),
        }
    };
    // Python's min / max on two floats: the first unless the second is
    // strictly smaller / larger
    let pmin = |x: f64, y: f64| if y < x { y } else { x };
    let pmax = |x: f64, y: f64| if y > x { y } else { x };
    let mut acc = total;
    for &a in &mine {
        let a = a as usize;
        let ra = rect(a);
        let fa = faces[a];
        for b in 0..n {
            if b == a || (slot[b] != usize::MAX && b < a) || fa & faces[b] == 0 {
                continue;
            }
            let rb = rect(b);
            let ox = pmin(ra[2], rb[2]) - pmax(ra[0], rb[0]);
            if !(ox > 0.0) {
                continue;
            }
            let oy = pmin(ra[3], rb[3]) - pmax(ra[1], rb[1]);
            if oy > 0.0 {
                acc += ox * oy;
            }
        }
    }
    acc
}

/// `cong_block_slack`: over each `block` x `block` block of the grid (block
/// (bi, bj) holds cells bi*block.. and bj*block.., clipped to the grid) that
/// window (i0, i1, j0, j1) touches, the least `c / pitch - d` -- how much
/// demand its fullest cell can take before it overflows -- written into `out`
/// (the block grid). NaN when a cell's is.
#[pyfunction]
pub fn cong_block_slack(
    d: PyReadonlyArray2<f64>,
    c: PyReadonlyArray2<f64>,
    pitch: f64,
    block: usize,
    win: (usize, usize, usize, usize),
    mut out: numpy::PyReadwriteArray2<f64>,
) {
    let d = d.as_array();
    let c = c.as_array();
    let mut o = out.as_array_mut();
    let (ny, nx) = d.dim();
    let (i0, i1, j0, j1) = (win.0, win.1.min(ny), win.2, win.3.min(nx));
    if i1 <= i0 || j1 <= j0 || block == 0 {
        return;
    }
    for bi in (i0 / block)..=((i1 - 1) / block) {
        for bj in (j0 / block)..=((j1 - 1) / block) {
            let mut m = f64::INFINITY;
            'cells: for gi in bi * block..((bi + 1) * block).min(ny) {
                for gj in bj * block..((bj + 1) * block).min(nx) {
                    let v = c[[gi, gj]] / pitch - d[[gi, gj]];
                    if v.is_nan() {
                        m = f64::NAN;
                        break 'cells;
                    }
                    if v < m {
                        m = v;
                    }
                }
            }
            o[[bi, bj]] = m;
        }
    }
}

/// `cong_delta`, visiting only the cells that can change the total: the
/// window is cut into the `block` x `block` blocks `cong_block_slack` keeps
/// (`slack`, the least slack per block), and a block is skipped when its
/// least slack is more than the most the move's demand terms can add to any
/// of its cells (each term's density times its largest overlaps there) and
/// the move's pad use does not reach it -- no cell of it overflows before or
/// after, so each adds exactly nothing. The cells it does visit are summed
/// in the window's row-major order, each from the same terms in the same
/// order as `cong_delta`, so the two agree to the bit.
#[pyfunction]
#[allow(clippy::too_many_arguments)]
#[allow(clippy::type_complexity)]
pub fn cong_delta_blocks(
    d: PyReadonlyArray2<f64>,
    c: PyReadonlyArray2<f64>,
    uses: Vec<PyReadonlyArray2<f64>>,
    win: (usize, usize, usize, usize),
    dem: Vec<(f64, usize, usize, PyReadonlyArray1<f64>, PyReadonlyArray1<f64>)>,
    usev: Vec<(usize, f64, usize, usize, PyReadonlyArray2<f64>)>,
    cell2: f64,
    pitch: f64,
    slack: PyReadonlyArray2<f64>,
    block: usize,
) -> f64 {
    let d = d.as_array();
    let c = c.as_array();
    let sl = slack.as_array();
    let (i0, i1, j0, j1) = win;
    if i1 <= i0 || j1 <= j0 {
        return 0.0;
    }
    let b = block.max(1);
    let (bi0, bi1) = (i0 / b, (i1 - 1) / b + 1);
    let (bj0, bj1) = (j0 / b, (j1 - 1) / b + 1);
    let (nbh, nbw) = (bi1 - bi0, bj1 - bj0);
    let dem: Vec<(f64, usize, usize, Vec<f64>, Vec<f64>)> = dem
        .iter()
        .map(|(s, ti0, tj0, cy, cx)| (*s, *ti0, *tj0, cy.as_array().to_vec(), cx.as_array().to_vec()))
        .collect();
    // the most a cell of each block can gain: per term, its density times
    // its largest row and column overlaps in the block
    let mut gain = vec![0.0f64; nbh * nbw];
    for (s, ti0, tj0, cy, cx) in &dem {
        if !(*s > 0.0) || cy.is_empty() || cx.is_empty() {
            continue;
        }
        let (tb0, tb1) = (ti0 / b, (ti0 + cy.len() - 1) / b + 1);
        let mut ry = vec![0.0f64; tb1 - tb0];
        for (a, &v) in cy.iter().enumerate() {
            let k = (ti0 + a) / b - tb0;
            if v > ry[k] {
                ry[k] = v;
            }
        }
        let (cb0, cb1) = (tj0 / b, (tj0 + cx.len() - 1) / b + 1);
        let mut rx = vec![0.0f64; cb1 - cb0];
        for (q, &v) in cx.iter().enumerate() {
            let k = (tj0 + q) / b - cb0;
            if v > rx[k] {
                rx[k] = v;
            }
        }
        for (kr, &my) in ry.iter().enumerate() {
            if my == 0.0 {
                continue;
            }
            let row = (tb0 + kr - bi0) * nbw;
            let sy = s * my;
            for (kc, &mx) in rx.iter().enumerate() {
                gain[row + cb0 + kc - bj0] += sy * mx;
            }
        }
    }
    // the use terms' union box, its new use per layer, then its capacity
    let (mut ui0, mut ui1, mut uj0, mut uj1) = (usize::MAX, 0usize, usize::MAX, 0usize);
    for (_l, _s, ti0, tj0, bk) in &usev {
        let bk = bk.as_array();
        ui0 = ui0.min(*ti0);
        uj0 = uj0.min(*tj0);
        ui1 = ui1.max(ti0 + bk.nrows());
        uj1 = uj1.max(tj0 + bk.ncols());
    }
    let mut cap_u: Vec<f64> = Vec::new();
    let (uh, uw) = if ui1 > ui0 && uj1 > uj0 { (ui1 - ui0, uj1 - uj0) } else { (0, 0) };
    if uh > 0 {
        let nl = uses.len();
        let mut du = vec![0.0f64; nl * uh * uw];
        for (l, s, ti0, tj0, bk) in &usev {
            let bk = bk.as_array();
            for r in 0..bk.nrows() {
                for q in 0..bk.ncols() {
                    du[l * uh * uw + (ti0 - ui0 + r) * uw + (tj0 - uj0 + q)] += s * bk[[r, q]];
                }
            }
        }
        let us: Vec<_> = uses.iter().map(|u| u.as_array()).collect();
        cap_u = vec![0.0f64; uh * uw];
        for r in 0..uh {
            for q in 0..uw {
                let mut cap = 0.0f64;
                for (l, u) in us.iter().enumerate() {
                    cap += clip0(cell2 - (u[[ui0 + r, uj0 + q]] + du[l * uh * uw + r * uw + q]));
                }
                cap_u[r * uw + q] = cap;
            }
        }
    }
    // the blocks to visit, each a slot of block x block cells
    let mut slot = vec![usize::MAX; nbh * nbw];
    let mut nhot = 0usize;
    for br in 0..nbh {
        for bc in 0..nbw {
            let (gbi, gbj) = (bi0 + br, bj0 + bc);
            let mut hot = !(sl[[gbi, gbj]] > gain[br * nbw + bc] + 1e-9);
            if !hot && uh > 0 {
                let (r0, q0) = (gbi * b, gbj * b);
                hot = r0 < ui1 && ui0 < r0 + b && q0 < uj1 && uj0 < q0 + b;
            }
            if hot {
                slot[br * nbw + bc] = nhot;
                nhot += 1;
            }
        }
    }
    if nhot == 0 {
        return 0.0;
    }
    let bb = b * b;
    let mut buf = vec![0.0f64; nhot * bb];
    for (s, ti0, tj0, cy, cx) in &dem {
        let (tj0, end) = (*tj0, tj0 + cx.len());
        for (a, &yv) in cy.iter().enumerate() {
            let ya = s * yv;
            if ya == 0.0 {
                continue;
            }
            let gi = ti0 + a;
            let bgi = gi / b;
            let (row, ro) = ((bgi - bi0) * nbw, (gi - bgi * b) * b);
            let (mut gj, mut bc) = (tj0, tj0 / b);
            while gj < end {
                let bstart = bc * b;
                let stop = (bstart + b).min(end);
                let k = slot[row + bc - bj0];
                if k != usize::MAX {
                    let off = k * bb + ro + (gj - bstart);
                    let dst = &mut buf[off..off + (stop - gj)];
                    for (x, v) in dst.iter_mut().zip(&cx[gj - tj0..stop - tj0]) {
                        *x += ya * v;
                    }
                }
                gj = stop;
                bc += 1;
            }
        }
    }
    let nx = d.ncols();
    let dv = d.as_standard_layout();
    let cv = c.as_standard_layout();
    let ds = dv.as_slice().expect("a standard-layout array is one slice");
    let cs = cv.as_slice().expect("a standard-layout array is one slice");
    let mut acc = 0.0f64;
    for gi in i0..i1 {
        let bgi = gi / b;
        let (row, ro) = ((bgi - bi0) * nbw, (gi - bgi * b) * b);
        let in_r = uh > 0 && gi >= ui0 && gi < ui1;
        let (drow, crow) = (&ds[gi * nx..(gi + 1) * nx], &cs[gi * nx..(gi + 1) * nx]);
        let (mut gj, mut bc) = (j0, j0 / b);
        while gj < j1 {
            let bstart = bc * b;
            let stop = (bstart + b).min(j1);
            let k = slot[row + bc - bj0];
            if k != usize::MAX {
                let off = k * bb + ro;
                for g in gj..stop {
                    let dd = buf[off + g - bstart];
                    let in_u = in_r && g >= uj0 && g < uj1;
                    if dd == 0.0 && !in_u {
                        continue;
                    }
                    let dold = drow[g];
                    let cold = crow[g];
                    let cnew = if in_u { cap_u[(gi - ui0) * uw + (g - uj0)] } else { cold };
                    let oo = clip0(dold - cold / pitch);
                    let on = clip0(dold + dd - cnew / pitch);
                    acc += on * on - oo * oo;
                }
            }
            gj = stop;
            bc += 1;
        }
    }
    acc
}
