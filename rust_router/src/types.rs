//! Shared types and constants for the grid router.

use std::cmp::Ordering;
use rustc_hash::FxHashSet;

/// 8 directions for octilinear routing
pub const DIRECTIONS: [(i32, i32); 8] = [
    (1, 0),   // East
    (1, -1),  // NE
    (0, -1),  // North
    (-1, -1), // NW
    (-1, 0),  // West
    (-1, 1),  // SW
    (0, 1),   // South
    (1, 1),   // SE
];

pub const ORTHO_COST: i32 = 1000;
pub const DIAG_COST: i32 = 1414; // sqrt(2) * 1000

/// Pack (gx, gy) into a single u64 for fast hashing
#[inline]
pub fn pack_xy(gx: i32, gy: i32) -> u64 {
    let x = (gx as u64) & 0xFFFFFFFF;
    let y = (gy as u64) & 0xFFFFFFFF;
    (x << 32) | y
}

/// Grid state: (x, y, layer) packed into a single u64 for fast hashing
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub struct GridState {
    pub gx: i32,
    pub gy: i32,
    pub layer: u8,
}

impl GridState {
    #[inline]
    pub fn new(gx: i32, gy: i32, layer: u8) -> Self {
        Self { gx, gy, layer }
    }

    #[inline]
    pub fn as_key(&self) -> u64 {
        // Pack into u64 for fast hashing: 20 bits x, 20 bits y, 8 bits layer
        let x = (self.gx as u64) & 0xFFFFF;
        let y = (self.gy as u64) & 0xFFFFF;
        let l = self.layer as u64;
        (x << 28) | (y << 8) | l
    }
}

/// A* open set entry with reverse ordering for min-heap
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct OpenEntry {
    pub f_score: i32,
    pub g_score: i32,
    pub state: GridState,
    pub counter: u32, // Tie-breaker for deterministic ordering
}

impl Ord for OpenEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        // Reverse ordering for min-heap (lowest f_score first)
        other.f_score.cmp(&self.f_score)
            .then_with(|| other.counter.cmp(&self.counter))
    }
}

impl PartialOrd for OpenEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Pose state: (x, y, theta_idx, layer) for orientation-aware routing
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub struct PoseState {
    pub gx: i32,
    pub gy: i32,
    pub theta_idx: u8,  // 0-7, corresponding to DIRECTIONS indices
    pub layer: u8,
}

impl PoseState {
    #[inline]
    pub fn new(gx: i32, gy: i32, theta_idx: u8, layer: u8) -> Self {
        Self { gx, gy, theta_idx, layer }
    }

    #[inline]
    pub fn as_key(&self) -> u64 {
        // Pack into u64: 19 bits x, 19 bits y, 3 bits theta, 8 bits layer
        // This allows x,y in range [-262144, 262143]
        let x = (self.gx as u64) & 0x7FFFF;
        let y = (self.gy as u64) & 0x7FFFF;
        let t = (self.theta_idx as u64) & 0x7;
        let l = self.layer as u64;
        (x << 30) | (y << 11) | (t << 8) | l
    }

    /// Get the direction vector for this pose's heading
    #[inline]
    pub fn direction(&self) -> (i32, i32) {
        DIRECTIONS[self.theta_idx as usize]
    }

    /// Get theta in radians
    #[inline]
    pub fn theta_radians(&self) -> f64 {
        (self.theta_idx as f64) * std::f64::consts::FRAC_PI_4
    }
}

/// A* open set entry for pose-based search
#[derive(Clone, Copy, Debug)]
pub struct PoseOpenEntry {
    pub f_score: i32,
    pub g_score: i32,
    pub state: PoseState,
    pub counter: u32,
}

impl Eq for PoseOpenEntry {}

impl PartialEq for PoseOpenEntry {
    fn eq(&self, other: &Self) -> bool {
        self.f_score == other.f_score && self.counter == other.counter
    }
}

impl Ord for PoseOpenEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        // Reverse ordering for min-heap (lowest f_score first)
        other.f_score.cmp(&self.f_score)
            .then_with(|| other.counter.cmp(&self.counter))
    }
}

impl PartialOrd for PoseOpenEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Tracks blocked cells encountered during A* search.
/// Used by both GridRouter and PoseRouter to report frontier on failure.
pub struct BlockedCellTracker {
    blocked: FxHashSet<u64>,
}

impl BlockedCellTracker {
    pub fn new() -> Self {
        Self {
            blocked: FxHashSet::default(),
        }
    }

    /// Track a blocked cell (using GridState key format: 20 bits x, 20 bits y, 8 bits layer)
    #[inline]
    pub fn track(&mut self, gx: i32, gy: i32, layer: u8) {
        let key = GridState::new(gx, gy, layer).as_key();
        self.blocked.insert(key);
    }

    /// Get all blocked cells as (gx, gy, layer) tuples
    pub fn get_blocked(&self) -> Vec<(i32, i32, u8)> {
        self.blocked
            .iter()
            .map(|&key| {
                let layer = (key & 0xFF) as u8;
                let y = ((key >> 8) & 0xFFFFF) as i32;
                let x = ((key >> 28) & 0xFFFFF) as i32;
                // Sign extension for negative coordinates
                let x = if x & 0x80000 != 0 { x | !0xFFFFF_i32 } else { x };
                let y = if y & 0x80000 != 0 { y | !0xFFFFF_i32 } else { y };
                (x, y, layer)
            })
            .collect()
    }

}

impl Default for BlockedCellTracker {
    fn default() -> Self {
        Self::new()
    }
}
