//! The whole-route snap's grid search for a SINGLE lane (awx/whole_snap.py, route() -> search()), ported exactly.
//!
//! A* over (cell, heading, vias taken) in a lane's window: octilinear moves inside its band, a turn of at most 90
//! degrees, each move paying its length times (1 + W_DEV x its cell's distance from the smooth line) and W_BEND per
//! 45 degrees of turn; a via (the next layer) where the plan's via is within RVIA and its arc gates allow it; the first
//! move out of the tooth and the last into the berth within 90 degrees of their joins, and (strict) every move within a
//! track width of an end within 90 degrees of its stub.
//!
//! EXACT: every cost is formed from the same operations in the same order as the Python, so the same doubles; the heap
//! orders entries as Python's tuples do (estimate, then cost, then the state), so the same states are expanded in the
//! same order and the same path comes back. The heuristic and the near-end tests are hypot of two integers, which is
//! sqrt(a*a + b*b) exactly (a*a + b*b is exact and sqrt correctly rounded; math.hypot agrees on every such pair); a
//! via's distance from its planned site is not, and comes in as a table math.hypot filled.

use numpy::{PyReadonlyArray2, PyReadonlyArray3};
use pyo3::prelude::*;
use rustc_hash::FxHashMap;
use std::cmp::Ordering;
use std::collections::BinaryHeap;

const DIRS: [(i32, i32); 8] = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)];

type State = (i32, i32, u8, u8); // (i, j, heading, vias taken)

#[derive(Clone, Copy)]
struct Entry {
    f: f64,
    c: f64,
    s: State,
}

impl PartialEq for Entry {
    fn eq(&self, o: &Self) -> bool {
        self.cmp(o) == Ordering::Equal
    }
}
impl Eq for Entry {}
impl PartialOrd for Entry {
    fn partial_cmp(&self, o: &Self) -> Option<Ordering> {
        Some(self.cmp(o))
    }
}
impl Ord for Entry {
    // reversed: BinaryHeap is a max-heap, the search pops the least (f, c, state) as Python's heapq does
    fn cmp(&self, o: &Self) -> Ordering {
        o.f.partial_cmp(&self.f)
            .unwrap_or(Ordering::Equal)
            .then_with(|| o.c.partial_cmp(&self.c).unwrap_or(Ordering::Equal))
            .then_with(|| o.s.cmp(&self.s))
    }
}

fn ihypot(a: i64, b: i64) -> f64 {
    ((a * a + b * b) as f64).sqrt()
}

fn bend_of(a: u8, b: u8) -> i32 {
    let d = (a as i32 - b as i32).abs();
    d.min(8 - d)
}

/// The search. Maps are the window's (NI x NJ): band, bad[layer], vbad[heading % 4], arc, dist, and dv[k] (a via's
/// distance from the plan's k-th via site, math.hypot's, +inf out of its reach). Returns (the path's states, or
/// None; the states expanded).
#[pyfunction]
#[pyo3(signature = (band, bad, vbad, arc, dist, dv, s, e, d0, dn, lays, gate, room0, arc_hi, rvia, w_via, w_dev,
                    w_bend, g, tw, step, start_ok, end_ok, out_ok, in_ok, strict))]
#[allow(clippy::too_many_arguments)]
pub fn lane_search(
    band: PyReadonlyArray2<bool>,
    bad: PyReadonlyArray3<bool>,
    vbad: PyReadonlyArray3<bool>,
    arc: PyReadonlyArray2<f64>,
    dist: PyReadonlyArray2<f64>,
    dv: PyReadonlyArray3<f64>,
    s: (i32, i32),
    e: (i32, i32),
    d0: u8,
    dn: u8,
    lays: Vec<u8>,
    gate: Vec<(f64, f64)>,
    room0: f64,
    arc_hi: f64,
    rvia: f64,
    w_via: f64,
    w_dev: f64,
    w_bend: f64,
    g: f64,
    tw: f64,
    step: Vec<f64>,
    start_ok: Vec<bool>,
    end_ok: Vec<bool>,
    out_ok: Vec<bool>,
    in_ok: Vec<bool>,
    strict: bool,
) -> PyResult<(Option<Vec<State>>, usize)> {
    let band = band.as_array();
    let bad = bad.as_array();
    let vbad = vbad.as_array();
    let arc = arc.as_array();
    let dist = dist.as_array();
    let dv = dv.as_array();
    let (ni_, nj_) = (band.shape()[0] as i32, band.shape()[1] as i32);
    let kk = (lays.len() - 1) as u8;
    let h = |i: i32, j: i32| ihypot((i - e.0) as i64, (j - e.1) as i64) * g;

    let start: State = (s.0, s.1, d0, 0);
    let mut best: FxHashMap<State, f64> = FxHashMap::default();
    let mut prev: FxHashMap<State, State> = FxHashMap::default();
    let mut pq = BinaryHeap::new();
    best.insert(start, 0.0);
    pq.push(Entry { f: h(s.0, s.1), c: 0.0, s: start });
    let mut goal: Option<State> = None;
    let mut npop = 0usize;
    while let Some(Entry { c: c_, s: st, .. }) = pq.pop() {
        if *best.get(&st).unwrap_or(&f64::INFINITY) < c_ - 1e-12 {
            continue;
        }
        npop += 1;
        let (i, j, d, k) = st;
        if (i, j) == e && k == kk {
            goal = Some(st);
            break;
        }
        let (iu, ju) = (i as usize, j as usize);
        let l = lays[k as usize] as usize;
        // a via here: the next layer, near the plan's via, where a via clears
        if k < kk && !vbad[[(d % 4) as usize, iu, ju]] {
            let a = arc[[iu, ju]];
            if room0 <= a && a <= arc_hi {
                let dvv = dv[[k as usize, iu, ju]];
                if dvv <= rvia && !bad[[lays[k as usize + 1] as usize, iu, ju]] {
                    let nst: State = (i, j, d, k + 1);
                    let nc = c_ + w_via * dvv;
                    if nc < *best.get(&nst).unwrap_or(&f64::INFINITY) - 1e-12 {
                        best.insert(nst, nc);
                        prev.insert(nst, st);
                        pq.push(Entry { f: nc + h(i, j), c: nc, s: nst });
                    }
                }
            }
        }
        for nd in 0..8u8 {
            let bend = bend_of(nd, d);
            if bend >= 3 {
                continue;
            }
            let (di, dj) = DIRS[nd as usize];
            let (ni, nj) = (i + di, j + dj);
            if !(0 <= ni && ni < ni_ && 0 <= nj && nj < nj_) || !band[[ni as usize, nj as usize]] {
                continue;
            }
            // no fold at either end: the first move within 90 degrees of the join out of the tooth, the last within
            // 90 of the join into the berth; and (strict) every move within a track width of an end within 90 of its
            // stub's own way
            if (i, j) == s && k == 0 && !start_ok[nd as usize] {
                continue;
            }
            if (ni, nj) == e && k == kk && !end_ok[nd as usize] {
                continue;
            }
            if strict && k == 0 && ihypot((i - s.0) as i64, (j - s.1) as i64) * g <= tw && !out_ok[nd as usize] {
                continue;
            }
            if strict && k == kk && ihypot((ni - e.0) as i64, (nj - e.1) as i64) * g <= tw && !in_ok[nd as usize] {
                continue;
            }
            let (niu, nju) = (ni as usize, nj as usize);
            if (ni, nj) != e && bad[[l, niu, nju]] {
                continue;
            }
            let a = arc[[niu, nju]];
            let (glo, ghi) = gate[k as usize];
            if !(glo <= a && a <= ghi) {
                continue;
            }
            let mut nc = c_ + step[nd as usize] * (1.0 + w_dev * dist[[niu, nju]]) + w_bend * (bend as f64);
            if (ni, nj) == e {
                nc += w_bend * (bend_of(nd, dn) as f64);
            }
            let nst: State = (ni, nj, nd, k);
            if nc < *best.get(&nst).unwrap_or(&f64::INFINITY) - 1e-12 {
                best.insert(nst, nc);
                prev.insert(nst, st);
                pq.push(Entry { f: nc + h(ni, nj), c: nc, s: nst });
            }
        }
    }
    let goal = match goal {
        None => return Ok((None, npop)),
        Some(gl) => gl,
    };
    let mut path = vec![goal];
    while let Some(p) = prev.get(path.last().unwrap()) {
        path.push(*p);
    }
    path.reverse();
    Ok((Some(path), npop))
}
