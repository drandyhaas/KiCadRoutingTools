//! Grid-based A* PCB Router - Rust implementation for speed.
//!
//! This is a high-performance implementation of the grid router algorithm.
//! It's designed to be called from Python via PyO3 bindings.

use pyo3::prelude::*;
use numpy::PyReadonlyArray2;
use rustc_hash::FxHashSet;
use std::collections::HashMap;

#[global_allocator]
static GLOBAL: mimalloc::MiMalloc = mimalloc::MiMalloc;  // EXPERIMENT

mod types;
mod obstacle_map;
mod router;
mod visual_router;
mod dubins;
mod pose_router;

pub use obstacle_map::GridObstacleMap;
pub use router::GridRouter;
pub use visual_router::{VisualRouter, SearchSnapshot};
pub use pose_router::PoseRouter;

/// Try to release unused memory back to the OS.
/// This is a hint to the allocator and may not have immediate effect.
#[pyfunction]
fn release_memory() {
    // On most platforms, dropping collections and calling shrink_to_fit
    // is the main way to release memory. The Rust allocator will
    // eventually return memory to the OS when possible.
    //
    // For more aggressive memory release on Linux, one could use:
    // unsafe { libc::malloc_trim(0); }
    // But this requires the libc crate and is platform-specific.
    //
    // For now, this function serves as a documentation point and
    // placeholder for future platform-specific optimizations.
}

/// Identify which nets' copper overlaps a set of blocked grid cells, returning
/// net_id -> count (how many of the net's segments/vias/pads touch the blocked
/// frontier). Rust port of single_ended_routing._identify_blocking_obstacles
/// (the rip-up blocking analysis), which was a hot Python loop.
///
/// Inputs (all grid-integer; Python pre-computes grid coords / expansions):
///   blocked:  N x 3  (gx, gy, layer)
///   segments: M x 6  (gx1, gy1, gx2, gy2, layer, net_id)   foreign only
///   vias:     V x 3  (gx, gy, net_id)                       foreign only
///   pads:     P x 6  (gx, gy, exp_x, exp_y, net_id, layer_mask)  foreign, net!=0
/// Each segment/via/pad contributes at most 1 to its net (first overlapping
/// cell), matching the Python break-on-first-hit semantics exactly.
#[pyfunction]
fn identify_blocking_obstacles(
    blocked: PyReadonlyArray2<i64>,
    segments: PyReadonlyArray2<i64>,
    vias: PyReadonlyArray2<i64>,
    pads: PyReadonlyArray2<i64>,
    expansion_grid: i64,
    via_expansion_grid: i64,
    num_layers: i64,
) -> HashMap<i64, i64> {
    let barr = blocked.as_array();
    let mut blocked_set: FxHashSet<(i64, i64, i64)> =
        FxHashSet::with_capacity_and_hasher(barr.nrows(), Default::default());
    for row in barr.rows() {
        blocked_set.insert((row[0], row[1], row[2]));
    }
    let mut counts: HashMap<i64, i64> = HashMap::new();

    // Segments: walk the Bresenham line, test each cell's clearance neighbourhood.
    let e = expansion_grid;
    for row in segments.as_array().rows() {
        let (gx1, gy1, gx2, gy2, layer, net) =
            (row[0], row[1], row[2], row[3], row[4], row[5]);
        let dx = (gx2 - gx1).abs();
        let dy = (gy2 - gy1).abs();
        let sx = if gx1 < gx2 { 1 } else { -1 };
        let sy = if gy1 < gy2 { 1 } else { -1 };
        let mut err = dx - dy;
        let (mut gx, mut gy) = (gx1, gy1);
        let mut hit = false;
        loop {
            'nb: for ex in -e..=e {
                for ey in -e..=e {
                    if blocked_set.contains(&(gx + ex, gy + ey, layer)) {
                        hit = true;
                        break 'nb;
                    }
                }
            }
            if hit || (gx == gx2 && gy == gy2) {
                break;
            }
            let e2 = 2 * err;
            if e2 > -dy {
                err -= dy;
                gx += sx;
            }
            if e2 < dx {
                err += dx;
                gy += sy;
            }
        }
        if hit {
            *counts.entry(net).or_insert(0) += 1;
        }
    }

    // Vias span all layers within their expansion radius.
    let ve = via_expansion_grid;
    for row in vias.as_array().rows() {
        let (gx, gy, net) = (row[0], row[1], row[2]);
        let mut hit = false;
        'via: for layer in 0..num_layers {
            for ex in -ve..=ve {
                for ey in -ve..=ve {
                    if blocked_set.contains(&(gx + ex, gy + ey, layer)) {
                        hit = true;
                        break 'via;
                    }
                }
            }
        }
        if hit {
            *counts.entry(net).or_insert(0) += 1;
        }
    }

    // Pads: per-pad expansion rect, on the layers in layer_mask (bit i = layer i).
    for row in pads.as_array().rows() {
        let (gx, gy, ex_x, ex_y, net, mask) =
            (row[0], row[1], row[2], row[3], row[4], row[5]);
        let mut hit = false;
        'pad: for layer in 0..num_layers {
            if (mask >> layer) & 1 == 0 {
                continue;
            }
            for ex in -ex_x..=ex_x {
                for ey in -ex_y..=ex_y {
                    if blocked_set.contains(&(gx + ex, gy + ey, layer)) {
                        hit = true;
                        break 'pad;
                    }
                }
            }
        }
        if hit {
            *counts.entry(net).or_insert(0) += 1;
        }
    }

    counts
}

/// Python module
#[pymodule]
fn grid_router(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add("__version__", env!("CARGO_PKG_VERSION"))?;
    m.add_class::<GridObstacleMap>()?;
    m.add_class::<GridRouter>()?;
    m.add_class::<PoseRouter>()?;
    m.add_class::<VisualRouter>()?;
    m.add_class::<SearchSnapshot>()?;
    m.add_function(wrap_pyfunction!(release_memory, m)?)?;
    m.add_function(wrap_pyfunction!(identify_blocking_obstacles, m)?)?;
    Ok(())
}
