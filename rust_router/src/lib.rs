//! Grid-based A* PCB Router - Rust implementation for speed.
//!
//! This is a high-performance implementation of the grid router algorithm.
//! It's designed to be called from Python via PyO3 bindings.

use pyo3::prelude::*;

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
    Ok(())
}
