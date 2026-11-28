//! Geometry kernels: batch versions of pure-compute Python loops that ran
//! once per grid cell or once per union-find edge.
//!
//! Each kernel evaluates the same float expressions, in the same order, as
//! the Python it replaces, so its accept/reject decisions are bit-identical;
//! the Python keeps the policy (which polygons, which cells, which edges) and
//! hands the arrays over.

use numpy::{IntoPyArray, PyArray1, PyArray2, PyReadonlyArray1, PyReadonlyArray2};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

/// Python's `min(a, b)` on floats: `b` only when it compares strictly less.
#[inline]
fn py_min(a: f64, b: f64) -> f64 {
    if b < a {
        b
    } else {
        a
    }
}

/// Python's `max(a, b)` on floats: `b` only when it compares strictly greater.
#[inline]
fn py_max(a: f64, b: f64) -> f64 {
    if b > a {
        b
    } else {
        a
    }
}

/// `routing_utils._capsule_mask`: the lattice box around the capsule of
/// radius `margin` about (x1,y1)-(x2,y2) and, per node, whether its squared
/// distance to the segment is `< (margin - tie_eps)^2`. Same expressions and
/// order as the numpy original (elementwise IEEE ops, no FMA), so membership
/// is bit-identical. Calls `f(gx, gy)` for every member, gx-major.
#[inline]
fn capsule_for_each<F: FnMut(i64, i64)>(
    x1: f64,
    y1: f64,
    x2: f64,
    y2: f64,
    margin: f64,
    grid_step: f64,
    tie_eps: f64,
    mut f: F,
) {
    let inv = 1.0 / grid_step;
    let glo_x = ((py_min(x1, x2) - margin) * inv).floor() as i64;
    let ghi_x = ((py_max(x1, x2) + margin) * inv).ceil() as i64;
    let glo_y = ((py_min(y1, y2) - margin) * inv).floor() as i64;
    let ghi_y = ((py_max(y1, y2) + margin) * inv).ceil() as i64;
    let dx = x2 - x1;
    let dy = y2 - y1;
    let l2 = dx * dx + dy * dy;
    let m_eff = margin - tie_eps;
    let m2 = m_eff * m_eff;
    for gx in glo_x..=ghi_x {
        // numpy: int32 lattice index -> float64, times grid_step
        let cx = (gx as i32 as f64) * grid_step;
        for gy in glo_y..=ghi_y {
            let cy = (gy as i32 as f64) * grid_step;
            let t = if l2 <= 0.0 {
                0.0
            } else {
                let v = ((cx - x1) * dx + (cy - y1) * dy) / l2;
                // np.clip(t, 0, 1) on finite values
                if v < 0.0 {
                    0.0
                } else if v > 1.0 {
                    1.0
                } else {
                    v
                }
            };
            let ddx = cx - (x1 + t * dx);
            let ddy = cy - (y1 + t * dy);
            if ddx * ddx + ddy * ddy < m2 {
                f(gx, gy);
            }
        }
    }
}

/// `routing_utils.segment_blocked_spans` uncached body: (M, 3) int32
/// (gx, gy_lo, gy_hi), one row per column with any member, gx ascending.
/// Errors when a column is not one contiguous run (the Python assertion).
#[pyfunction]
#[allow(clippy::too_many_arguments)]
pub fn capsule_spans(
    py: Python<'_>,
    x1: f64,
    y1: f64,
    x2: f64,
    y2: f64,
    margin: f64,
    grid_step: f64,
    tie_eps: f64,
) -> PyResult<Py<PyArray2<i32>>> {
    let mut rows: Vec<[i32; 3]> = Vec::new();
    let mut count: i64 = 0;
    capsule_for_each(x1, y1, x2, y2, margin, grid_step, tie_eps, |gx, gy| {
        count += 1;
        match rows.last_mut() {
            Some(r) if r[0] as i64 == gx => r[2] = gy as i32,
            _ => rows.push([gx as i32, gy as i32, gy as i32]),
        }
    });
    let covered: i64 = rows.iter().map(|r| (r[2] - r[1] + 1) as i64).sum();
    if covered != count {
        return Err(pyo3::exceptions::PyAssertionError::new_err("capsule column not contiguous"));
    }
    let flat: Vec<i32> = rows.iter().flat_map(|r| r.iter().copied()).collect();
    let arr = numpy::ndarray::Array2::from_shape_vec((rows.len(), 3), flat)
        .map_err(|e| PyValueError::new_err(e.to_string()))?;
    Ok(arr.into_pyarray(py).unbind())
}

/// `routing_utils.segment_blocked_cells_array` uncached body: (N, 2) int32
/// member cells in the numpy `gxg[mask]` order (gx-major).
#[pyfunction]
#[allow(clippy::too_many_arguments)]
pub fn capsule_cells(
    py: Python<'_>,
    x1: f64,
    y1: f64,
    x2: f64,
    y2: f64,
    margin: f64,
    grid_step: f64,
    tie_eps: f64,
) -> PyResult<Py<PyArray2<i32>>> {
    let mut flat: Vec<i32> = Vec::new();
    capsule_for_each(x1, y1, x2, y2, margin, grid_step, tie_eps, |gx, gy| {
        flat.push(gx as i32);
        flat.push(gy as i32);
    });
    let n = flat.len() / 2;
    let arr = numpy::ndarray::Array2::from_shape_vec((n, 2), flat)
        .map_err(|e| PyValueError::new_err(e.to_string()))?;
    Ok(arr.into_pyarray(py).unbind())
}

/// `check_drc._point_in_poly`: even-odd ray cast over the ring `xs`/`ys`.
#[inline]
fn point_in_poly(x: f64, y: f64, xs: &[f64], ys: &[f64]) -> bool {
    let n = xs.len();
    if n == 0 {
        return false;
    }
    let mut inside = false;
    let mut j = n - 1;
    for i in 0..n {
        let (xi, yi) = (xs[i], ys[i]);
        let (xj, yj) = (xs[j], ys[j]);
        if ((yi > y) != (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi) + xi) {
            inside = !inside;
        }
        j = i;
    }
    inside
}

/// CPython's `v**2` on a float: the C library `pow(v, 2.0)`, which is not
/// always `v*v` to the last bit. The exponent goes through `black_box` so
/// LLVM cannot fold the call into a multiply.
#[inline]
fn py_sq(v: f64) -> f64 {
    v.powf(std::hint::black_box(2.0))
}

/// `geometry_utils.point_to_segment_distance`, same operation order
/// (`v**2` via `py_sq`; `max(0, min(1, t))` with Python's tie rules).
#[inline]
fn point_to_segment_distance(px: f64, py: f64, x1: f64, y1: f64, x2: f64, y2: f64) -> f64 {
    let dx = x2 - x1;
    let dy = y2 - y1;
    let length_sq = dx * dx + dy * dy;
    if length_sq < 1e-10 {
        return (py_sq(px - x1) + py_sq(py - y1)).sqrt();
    }
    let v = ((px - x1) * dx + (py - y1) * dy) / length_sq;
    let t = py_max(0.0, py_min(1.0, v));
    let proj_x = x1 + t * dx;
    let proj_y = y1 + t * dy;
    (py_sq(px - proj_x) + py_sq(py - proj_y)).sqrt()
}

/// Exposed for the randomized equality test against the Python original.
#[pyfunction]
pub fn point_segment_distances(
    p: PyReadonlyArray2<f64>,
    s: PyReadonlyArray2<f64>,
) -> PyResult<Py<PyArray1<f64>>> {
    let p = p.as_array();
    let s = s.as_array();
    if p.ncols() != 2 || s.ncols() != 4 || p.nrows() != s.nrows() {
        return Err(PyValueError::new_err("point_segment_distances: p is N x 2, s is N x 4"));
    }
    let out: Vec<f64> = (0..p.nrows())
        .map(|i| point_to_segment_distance(p[[i, 0]], p[[i, 1]], s[[i, 0]], s[[i, 1]], s[[i, 2]], s[[i, 3]]))
        .collect();
    Python::with_gil(|py| Ok(out.into_pyarray(py).unbind()))
}

/// `underpad.UnderpadGrid.block_poly`'s cell loop: every lattice cell
/// (a, b) with a in [a0, a1), b in [b0, b1), at x = x0 + a*res,
/// y = y0 + b*res, that lies inside one of the polygons or within `thr` of
/// one of their edges. Polygons arrive flattened: `pts` is N x 2 and ring k
/// is rows offs[k]..offs[k+1]. Per polygon, in order: inside test, then
/// each edge's bbox reject (bbox grown by `thr`) and the point-to-segment
/// distance `<= thr` -- the Python `_within` decision exactly.
/// Returns the flat indices a*ny + b of the hit cells, a-major.
#[pyfunction]
#[allow(clippy::too_many_arguments)]
pub fn poly_keepout_cells(
    pts: PyReadonlyArray2<f64>,
    offs: PyReadonlyArray1<i64>,
    thr: f64,
    x0: f64,
    y0: f64,
    res: f64,
    a0: i64,
    a1: i64,
    b0: i64,
    b1: i64,
    ny: i64,
) -> PyResult<Py<PyArray1<i64>>> {
    let pts = pts.as_array();
    let offs = offs.as_slice()?;
    if pts.ncols() != 2 {
        return Err(PyValueError::new_err("poly_keepout_cells: pts is N x 2"));
    }
    struct Poly {
        xs: Vec<f64>,
        ys: Vec<f64>,
        // per edge: bx0, bx1, by0, by1, x1, y1, x2, y2
        edges: Vec<[f64; 8]>,
    }
    let mut polys: Vec<Poly> = Vec::with_capacity(offs.len().saturating_sub(1));
    for k in 0..offs.len().saturating_sub(1) {
        let (s, e) = (offs[k], offs[k + 1]);
        if s < 0 || e < s || e as usize > pts.nrows() {
            return Err(PyValueError::new_err("poly_keepout_cells: bad ring offsets"));
        }
        let xs: Vec<f64> = (s as usize..e as usize).map(|i| pts[[i, 0]]).collect();
        let ys: Vec<f64> = (s as usize..e as usize).map(|i| pts[[i, 1]]).collect();
        let n = xs.len();
        let mut edges = Vec::with_capacity(n);
        for i in 0..n {
            let (x1, y1) = (xs[i], ys[i]);
            let (x2, y2) = (xs[(i + 1) % n], ys[(i + 1) % n]);
            edges.push([
                py_min(x1, x2) - thr,
                py_max(x1, x2) + thr,
                py_min(y1, y2) - thr,
                py_max(y1, y2) + thr,
                x1,
                y1,
                x2,
                y2,
            ]);
        }
        polys.push(Poly { xs, ys, edges });
    }
    let within = |x: f64, y: f64| -> bool {
        for p in &polys {
            if point_in_poly(x, y, &p.xs, &p.ys) {
                return true;
            }
            for e in &p.edges {
                if x < e[0] || x > e[1] || y < e[2] || y > e[3] {
                    continue;
                }
                if point_to_segment_distance(x, y, e[4], e[5], e[6], e[7]) <= thr {
                    return true;
                }
            }
        }
        false
    };
    let mut out: Vec<i64> = Vec::new();
    for a in a0..a1 {
        let x = x0 + (a as f64) * res;
        for b in b0..b1 {
            let y = y0 + (b as f64) * res;
            if within(x, y) {
                out.push(a * ny + b);
            }
        }
    }
    Python::with_gil(|py| Ok(out.into_pyarray(py).unbind()))
}

/// Connected-component labels for `check_connected.analyze_conn_excluding`.
/// Points are 0..num_points; `edges` is E x 2; an edge touching an
/// `excluded` point is dropped (the Python `if a in excl or b in excl:
/// continue`). Returns, for each id in `queries`, a label that is equal for
/// two queries exactly when the union-find puts them in one set. The label
/// is a root index, so only equality carries meaning -- which is all the
/// Python uses (unique counts, the main root by first-seen pad order).
#[pyfunction]
pub fn component_labels(
    num_points: i64,
    edges: PyReadonlyArray2<i64>,
    excluded: PyReadonlyArray1<i64>,
    queries: PyReadonlyArray1<i64>,
) -> PyResult<Py<PyArray1<i64>>> {
    if num_points < 0 {
        return Err(PyValueError::new_err("component_labels: num_points < 0"));
    }
    let n = num_points as usize;
    let edges = edges.as_array();
    if edges.nrows() > 0 && edges.ncols() != 2 {
        return Err(PyValueError::new_err("component_labels: edges is E x 2"));
    }
    let check = |v: i64| -> PyResult<usize> {
        if v < 0 || v as usize >= n {
            Err(PyValueError::new_err(format!("component_labels: point id {} outside 0..{}", v, n)))
        } else {
            Ok(v as usize)
        }
    };
    let mut excl = vec![false; n];
    for &v in excluded.as_slice()? {
        excl[check(v)?] = true;
    }
    let mut parent: Vec<u32> = (0..n as u32).collect();
    let mut rank: Vec<u8> = vec![0; n];
    #[inline]
    fn find(parent: &mut [u32], mut i: u32) -> u32 {
        while parent[i as usize] != i {
            let g = parent[parent[i as usize] as usize];
            parent[i as usize] = g;
            i = g;
        }
        i
    }
    for r in 0..edges.nrows() {
        let a = check(edges[[r, 0]])?;
        let b = check(edges[[r, 1]])?;
        if excl[a] || excl[b] {
            continue;
        }
        let ra = find(&mut parent, a as u32);
        let rb = find(&mut parent, b as u32);
        if ra == rb {
            continue;
        }
        let (hi, lo) = if rank[ra as usize] < rank[rb as usize] { (rb, ra) } else { (ra, rb) };
        parent[lo as usize] = hi;
        if rank[hi as usize] == rank[lo as usize] {
            rank[hi as usize] = rank[hi as usize].saturating_add(1);
        }
    }
    let q = queries.as_slice()?;
    let mut out: Vec<i64> = Vec::with_capacity(q.len());
    for &v in q {
        let i = check(v)?;
        out.push(find(&mut parent, i as u32) as i64);
    }
    Python::with_gil(|py| Ok(out.into_pyarray(py).unbind()))
}
