//! Geometry kernels: batch versions of pure-compute Python loops that ran
//! once per grid cell or once per union-find edge.
//!
//! Each kernel evaluates the same float expressions, in the same order, as
//! the Python it replaces, so its accept/reject decisions are bit-identical;
//! the Python keeps the policy (which polygons, which cells, which edges) and
//! hands the arrays over.

use numpy::{IntoPyArray, PyArray2};
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
