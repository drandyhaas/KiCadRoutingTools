//! Dubins path calculator for orientation-aware heuristics.
//!
//! Dubins paths are the shortest paths for a vehicle that can only move forward
//! and has a minimum turning radius. They consist of at most 3 segments, each
//! being either a straight line (S) or a circular arc (L for left, R for right).

/// Dubins path calculator for heuristic
pub struct DubinsCalculator {
    /// Minimum turning radius in grid units
    min_radius: f64,
}

impl DubinsCalculator {
    pub fn new(min_radius: f64) -> Self {
        Self { min_radius: min_radius.max(0.1) }
    }

    /// Calculate shortest Dubins path length between two poses
    /// Returns path length in grid units (scaled by 1000 for integer math)
    pub fn path_length(&self,
                   x1: f64, y1: f64, theta1: f64,
                   x2: f64, y2: f64, theta2: f64) -> i32 {
        let r = self.min_radius;

        // Vector from start to goal
        let dx = x2 - x1;
        let dy = y2 - y1;
        let d = (dx * dx + dy * dy).sqrt();

        // If very close, just return distance
        if d < 0.001 {
            // Just need to turn in place - approximate as arc
            let dtheta = Self::normalize_angle(theta2 - theta1).abs();
            return (dtheta * r * 1000.0) as i32;
        }

        // Normalize by turning radius
        let d_norm = d / r;

        // Angle from start to goal
        let phi = dy.atan2(dx);

        // Relative angles
        let alpha = Self::normalize_angle(theta1 - phi);
        let beta = Self::normalize_angle(theta2 - phi);

        // Try all 6 Dubins path types and return shortest
        let mut min_len = f64::MAX;

        // CSC paths (Circle-Straight-Circle)
        if let Some(len) = self.lsl_length(d_norm, alpha, beta) {
            min_len = min_len.min(len);
        }
        if let Some(len) = self.rsr_length(d_norm, alpha, beta) {
            min_len = min_len.min(len);
        }
        if let Some(len) = self.lsr_length(d_norm, alpha, beta) {
            min_len = min_len.min(len);
        }
        if let Some(len) = self.rsl_length(d_norm, alpha, beta) {
            min_len = min_len.min(len);
        }

        // CCC paths (Circle-Circle-Circle)
        if let Some(len) = self.rlr_length(d_norm, alpha, beta) {
            min_len = min_len.min(len);
        }
        if let Some(len) = self.lrl_length(d_norm, alpha, beta) {
            min_len = min_len.min(len);
        }

        // Fallback: if no valid path (shouldn't happen), use straight line + turn estimate
        if min_len == f64::MAX {
            let dtheta = Self::normalize_angle(theta2 - theta1).abs();
            min_len = d_norm + dtheta;
        }

        // Scale back by radius and convert to integer (x1000)
        (min_len * r * 1000.0) as i32
    }

    #[inline]
    fn normalize_angle(a: f64) -> f64 {
        let mut a = a % (2.0 * std::f64::consts::PI);
        if a > std::f64::consts::PI {
            a -= 2.0 * std::f64::consts::PI;
        } else if a < -std::f64::consts::PI {
            a += 2.0 * std::f64::consts::PI;
        }
        a
    }

    #[inline]
    fn mod2pi(a: f64) -> f64 {
        let mut a = a % (2.0 * std::f64::consts::PI);
        if a < 0.0 {
            a += 2.0 * std::f64::consts::PI;
        }
        a
    }

    /// LSL path: Left turn, Straight, Left turn
    fn lsl_length(&self, d: f64, alpha: f64, beta: f64) -> Option<f64> {
        let ca = alpha.cos();
        let sa = alpha.sin();
        let cb = beta.cos();
        let sb = beta.sin();

        let tmp = 2.0 + d * d - 2.0 * (ca * cb + sa * sb - d * (sa - sb));
        if tmp < 0.0 {
            return None;
        }
        let p = tmp.sqrt();
        let theta = (cb - ca).atan2(d + sa - sb);
        let t = Self::mod2pi(-alpha + theta);
        let q = Self::mod2pi(beta - theta);

        Some(t + p + q)
    }

    /// RSR path: Right turn, Straight, Right turn
    fn rsr_length(&self, d: f64, alpha: f64, beta: f64) -> Option<f64> {
        let ca = alpha.cos();
        let sa = alpha.sin();
        let cb = beta.cos();
        let sb = beta.sin();

        let tmp = 2.0 + d * d - 2.0 * (ca * cb + sa * sb - d * (sb - sa));
        if tmp < 0.0 {
            return None;
        }
        let p = tmp.sqrt();
        let theta = (ca - cb).atan2(d - sa + sb);
        let t = Self::mod2pi(alpha - theta);
        let q = Self::mod2pi(-beta + theta);

        Some(t + p + q)
    }

    /// LSR path: Left turn, Straight, Right turn
    fn lsr_length(&self, d: f64, alpha: f64, beta: f64) -> Option<f64> {
        let ca = alpha.cos();
        let sa = alpha.sin();
        let cb = beta.cos();
        let sb = beta.sin();

        let tmp = -2.0 + d * d + 2.0 * (ca * cb + sa * sb + d * (sa + sb));
        if tmp < 0.0 {
            return None;
        }
        let p = tmp.sqrt();
        let theta = (-ca - cb).atan2(d + sa + sb) - (-2.0_f64).atan2(p);
        let t = Self::mod2pi(-alpha + theta);
        let q = Self::mod2pi(-beta + theta);

        Some(t + p + q)
    }

    /// RSL path: Right turn, Straight, Left turn
    fn rsl_length(&self, d: f64, alpha: f64, beta: f64) -> Option<f64> {
        let ca = alpha.cos();
        let sa = alpha.sin();
        let cb = beta.cos();
        let sb = beta.sin();

        let tmp = -2.0 + d * d + 2.0 * (ca * cb + sa * sb - d * (sa + sb));
        if tmp < 0.0 {
            return None;
        }
        let p = tmp.sqrt();
        let theta = (ca + cb).atan2(d - sa - sb) - (2.0_f64).atan2(p);
        let t = Self::mod2pi(alpha - theta);
        let q = Self::mod2pi(beta - theta);

        Some(t + p + q)
    }

    /// RLR path: Right turn, Left turn, Right turn
    fn rlr_length(&self, d: f64, alpha: f64, beta: f64) -> Option<f64> {
        let ca = alpha.cos();
        let sa = alpha.sin();
        let cb = beta.cos();
        let sb = beta.sin();

        let tmp = (6.0 - d * d + 2.0 * (ca * cb + sa * sb + d * (sa - sb))) / 8.0;
        if tmp.abs() > 1.0 {
            return None;
        }
        let p = Self::mod2pi(2.0 * std::f64::consts::PI - tmp.acos());
        let theta = (ca - cb).atan2(d - sa + sb);
        let t = Self::mod2pi(alpha - theta + p / 2.0);
        let q = Self::mod2pi(alpha - beta - t + p);

        Some(t + p + q)
    }

    /// LRL path: Left turn, Right turn, Left turn
    fn lrl_length(&self, d: f64, alpha: f64, beta: f64) -> Option<f64> {
        let ca = alpha.cos();
        let sa = alpha.sin();
        let cb = beta.cos();
        let sb = beta.sin();

        let tmp = (6.0 - d * d + 2.0 * (ca * cb + sa * sb - d * (sa - sb))) / 8.0;
        if tmp.abs() > 1.0 {
            return None;
        }
        let p = Self::mod2pi(2.0 * std::f64::consts::PI - tmp.acos());
        let theta = (cb - ca).atan2(d + sa - sb);
        let t = Self::mod2pi(-alpha + theta + p / 2.0);
        let q = Self::mod2pi(beta - alpha - t + p);

        Some(t + p + q)
    }
}
