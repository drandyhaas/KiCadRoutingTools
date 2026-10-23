//! The whole-route snap's grid search for a PAIR (awx/whole_snap.py, route() -> search(..., poses=True)), ported
//! exactly, as lane_search ports a single's.
//!
//! A* over (cell, heading, vias taken, straight steps owed, straight steps taken) in a lane's window, from POSE to
//! POSE: the pair moves as the pair router does (pose_router.rs) -- 45-degree turns at most, each only with no straight
//! step owed and followed by RT owed; a via only with none owed and ST taken, followed by ST owed, one heading through
//! it; an OPPOSITE-HANDS pair's first dive its crossover (XS steps taken before it, owed after it, where the crossover
//! clears: a test the Python makes, called back and kept). Each move pays its length times (1 + W_DEV x its cell's
//! distance from the smooth line), W_BEND per 45 degrees of turn, W_KEEP per mm past its cell's staircase tolerance and
//! W_SHARE on a cell in a single's share; the end pose reached on its own heading with PR straight steps taken. MULTI:
//! from every start pose to any end pose at once, with no heuristic, and with no path the farthest along the lane it
//! got.
//!
//! EXACT: every cost is formed from the same operations in the same order as the Python, so the same doubles; the heap
//! orders entries as Python's tuples do (estimate, cost, state -- the Python's state is one int in a mixed radix,
//! ordered as the tuple (i, j, heading, vias, owed, taken) is, as this one is), so the same states are expanded in the
//! same order and the same path comes back. The heuristic is hypot of two integers, sqrt(a*a + b*b) exactly; a via's
//! distance from its planned site comes in as a table math.hypot filled.

use numpy::{PyReadonlyArray2, PyReadonlyArray3};
use pyo3::prelude::*;
use rustc_hash::FxHashMap;
use std::cmp::Ordering;
use std::collections::BinaryHeap;

const DIRS: [(i32, i32); 8] = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)];

type State = (i32, i32, u8, u8, u8, u8); // (i, j, heading, vias taken, straight steps owed, straight steps taken)

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

fn bend_of(a: u8, b: u8) -> u8 {
    let d = (a as i32 - b as i32).abs();
    d.min(8 - d) as u8
}

/// The search. Maps are the window's (NI x NJ): band, bad[layer], vbad[heading % 4], arc, dist, tol (or None),
/// soft[layer] (or None), and dv[k] (a via's distance from the plan's k-th via site, math.hypot's, +inf out of its
/// reach). `starts` [(i, j, heading)]; `ends` [(i, j, heading)] -- one, or (multi) every end pose. `xs` [(taken
/// before, owed after)] per heading, and `xo(i, j, heading) -> bool` the crossover's clearance, for a crossed pair.
/// Returns (the path's states or None, the states expanded, the farthest arc reached (multi), its state).
#[pyfunction]
#[pyo3(signature = (band, bad, vbad, arc, dist, tol, soft, dv, starts, ends, multi, lays, gate, room0, room1, total,
                    rvia, w_via, w_dev, w_bend, w_keep, w_share, g, step, cap, rt, st, pr, cross, xs, xo))]
#[allow(clippy::too_many_arguments)]
pub fn pair_search(
    py: Python<'_>,
    band: PyReadonlyArray2<bool>,
    bad: PyReadonlyArray3<bool>,
    vbad: PyReadonlyArray3<bool>,
    arc: PyReadonlyArray2<f64>,
    dist: PyReadonlyArray2<f64>,
    tol: Option<PyReadonlyArray2<f64>>,
    soft: Option<PyReadonlyArray3<bool>>,
    dv: PyReadonlyArray3<f64>,
    starts: Vec<(i32, i32, u8)>,
    ends: Vec<(i32, i32, u8)>,
    multi: bool,
    lays: Vec<u8>,
    gate: Vec<(f64, f64)>,
    room0: Vec<f64>,
    room1: Vec<f64>,
    total: f64,
    rvia: f64,
    w_via: f64,
    w_dev: f64,
    w_bend: f64,
    w_keep: f64,
    w_share: f64,
    g: f64,
    step: Vec<f64>,
    cap: u8,
    rt: u8,
    st: u8,
    pr: u8,
    cross: bool,
    xs: Vec<(u8, u8)>,
    xo: Option<PyObject>,
) -> PyResult<(Option<Vec<State>>, usize, f64, Option<State>)> {
    let band = band.as_array();
    let bad = bad.as_array();
    let vbad = vbad.as_array();
    let arc = arc.as_array();
    let dist = dist.as_array();
    let tol = tol.as_ref().map(|t| t.as_array());
    let soft = soft.as_ref().map(|t| t.as_array());
    let dv = dv.as_array();
    let (ni_, nj_) = (band.shape()[0] as i32, band.shape()[1] as i32);
    let kk = (lays.len() - 1) as u8;
    let (ei, ej, edn) = ends[0];
    let h = |i: i32, j: i32| -> f64 {
        if multi {
            0.0
        } else {
            ihypot((i - ei) as i64, (j - ej) as i64) * g
        }
    };
    // (multi) the end poses: each cell's headings
    let mut eg: FxHashMap<(i32, i32), Vec<u8>> = FxHashMap::default();
    for &(i, j, d) in &ends {
        eg.entry((i, j)).or_default().push(d);
    }
    let mut xo_memo: FxHashMap<(i32, i32, u8), bool> = FxHashMap::default();

    let mut best: FxHashMap<State, f64> = FxHashMap::default();
    let mut prev: FxHashMap<State, State> = FxHashMap::default();
    let mut pq = BinaryHeap::new();
    for &(si, sj, sd) in &starts {
        let start: State = (si, sj, sd, 0, pr, 0);
        best.insert(start, 0.0);
        pq.push(Entry { f: h(si, sj), c: 0.0, s: start });
    }
    let mut goal: Option<State> = None;
    let mut npop = 0usize;
    let mut far: (f64, Option<State>) = (f64::NEG_INFINITY, None);
    while let Some(Entry { c: c_, s: stt, .. }) = pq.pop() {
        if *best.get(&stt).unwrap_or(&f64::INFINITY) < c_ - 1e-12 {
            continue;
        }
        npop += 1;
        let (i, j, d, k, sa, sb) = stt;
        let (iu, ju) = (i as usize, j as usize);
        if multi && arc[[iu, ju]] > far.0 {
            far = (arc[[iu, ju]], Some(stt));
        }
        let at_end = if multi {
            eg.get(&(i, j)).map_or(false, |ds| ds.contains(&d))
        } else {
            (i, j) == (ei, ej)
        };
        if at_end && k == kk && sb >= pr {
            goal = Some(stt);
            break;
        }
        let l = lays[k as usize] as usize;
        let du = d as usize;
        // a via here: the next layer, near the plan's via, where a via clears; an opposite-hands pair swaps its legs
        // ONCE, its first dive the crossover, any later one a plain dive
        if cross && k == 0 && kk >= 1 {
            let a = arc[[iu, ju]];
            if sa == 0 && sb >= xs[du].0 && room0[du] <= a && a <= total - room1[du]
                && !bad[[lays[1] as usize, iu, ju]]
            {
                let dvv = dv[[0, iu, ju]];
                if dvv <= rvia {
                    let ok = match xo_memo.get(&(i, j, d)) {
                        Some(v) => *v,
                        None => {
                            let v: bool = match &xo {
                                Some(f) => f.call1(py, (i, j, d))?.extract(py)?,
                                None => false,
                            };
                            xo_memo.insert((i, j, d), v);
                            v
                        }
                    };
                    if ok {
                        let nst: State = (i, j, d, 1, xs[du].1, 0);
                        let nc = c_ + w_via * dvv;
                        if nc < *best.get(&nst).unwrap_or(&f64::INFINITY) - 1e-12 {
                            best.insert(nst, nc);
                            prev.insert(nst, stt);
                            pq.push(Entry { f: nc + h(i, j), c: nc, s: nst });
                        }
                    }
                }
            }
        } else if k < kk && !vbad[[du % 4, iu, ju]] && sa == 0 && sb >= st {
            let a = arc[[iu, ju]];
            if room0[du] <= a && a <= total - room1[du] {
                let dvv = dv[[k as usize, iu, ju]];
                if dvv <= rvia && !bad[[lays[k as usize + 1] as usize, iu, ju]] {
                    let nst: State = (i, j, d, k + 1, st, 0);
                    let nc = c_ + w_via * dvv;
                    if nc < *best.get(&nst).unwrap_or(&f64::INFINITY) - 1e-12 {
                        best.insert(nst, nc);
                        prev.insert(nst, stt);
                        pq.push(Entry { f: nc + h(i, j), c: nc, s: nst });
                    }
                }
            }
        }
        for nd in 0..8u8 {
            let bend = bend_of(d, nd);
            // (a pair: 45 degrees at most, and only with no straight step owed)
            if bend >= 2 || (bend > 0 && sa > 0) {
                continue;
            }
            let (di, dj) = DIRS[nd as usize];
            let (ni, nj) = (i + di, j + dj);
            if !(0 <= ni && ni < ni_ && 0 <= nj && nj < nj_) || !band[[ni as usize, nj as usize]] {
                continue;
            }
            let (niu, nju) = (ni as usize, nj as usize);
            if bad[[l, niu, nju]] {
                continue;
            }
            let a = arc[[niu, nju]];
            let (glo, ghi) = gate[k as usize];
            if !(glo <= a && a <= ghi) {
                continue;
            }
            let stp = step[nd as usize];
            let dd = dist[[niu, nju]];
            let mut nc = c_ + stp * (1.0 + w_dev * dd) + w_bend * (bend as f64);
            if let Some(t) = &tol {
                let tt = t[[niu, nju]];
                if dd > tt {
                    nc += stp * w_keep * (dd - tt);
                }
            }
            if let Some(sm) = &soft {
                if sm[[l, niu, nju]] {
                    nc += w_share * stp;
                }
            }
            if !multi && (ni, nj) == (ei, ej) {
                let eb = bend_of(nd, edn);
                if eb > 0 {
                    continue; // (a pose is reached on its own heading)
                }
                nc += w_bend * (eb as f64);
            }
            let nst: State = if bend == 0 {
                (ni, nj, nd, k, sa.saturating_sub(1), (sb + 1).min(cap))
            } else {
                (ni, nj, nd, k, rt, 1)
            };
            if nc < *best.get(&nst).unwrap_or(&f64::INFINITY) - 1e-12 {
                best.insert(nst, nc);
                prev.insert(nst, stt);
                pq.push(Entry { f: nc + h(ni, nj), c: nc, s: nst });
            }
        }
    }
    let goal = match goal {
        None => return Ok((None, npop, far.0, far.1)),
        Some(gl) => gl,
    };
    let mut path = vec![goal];
    while let Some(p) = prev.get(path.last().unwrap()) {
        path.push(*p);
    }
    path.reverse();
    Ok((Some(path), npop, far.0, far.1))
}
