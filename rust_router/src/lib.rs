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

/// Python module
#[pymodule]
fn grid_router(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add("__version__", env!("CARGO_PKG_VERSION"))?;
    m.add_class::<GridObstacleMap>()?;
    m.add_class::<GridRouter>()?;
    m.add_class::<PoseRouter>()?;
    m.add_class::<VisualRouter>()?;
    m.add_class::<SearchSnapshot>()?;
    Ok(())
}
