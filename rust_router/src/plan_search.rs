//! The search of the routes planned together (py_router/start_plan.py, plan_starts -> search_leg, WHOLE routes),
//! ported exactly.
//!
//! One net's way over the board's cells, layer by layer, from any cell of its own copper to any cell of the pads it
//! has yet to reach: eight moves a layer (a diagonal only between two cells free to the net), each paying its length
//! times its layer's cost, times (1 + the cell's history), times (1 + PF x the other nets' ways standing on the
//! cell); a change of layer where a via fits, paying the via's cost times (1 + the site's history) times (1 + PF x
//! the other ways within the via's reach). The estimate to go is the octile distance to the nearest pad.
//!
//! EXACT: every cost is formed from the same operations in the same order as the Python, so the same doubles; the
//! heap orders entries as Python's tuples do (estimate, then cost, then the cell), so the same cells are expanded in
//! the same order and the same way comes back.

use numpy::PyReadonlyArray1;
use pyo3::prelude::*;
use rustc_hash::{FxHashMap, FxHashSet};
use std::cmp::Ordering;
use std::collections::BinaryHeap;

#[derive(Clone, Copy)]
struct Entry {
    f: f64,
    g: f64,
    u: i64,
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
    // reversed: BinaryHeap is a max-heap, the search pops the least (f, g, cell) as Python's heapq does
    fn cmp(&self, o: &Self) -> Ordering {
        o.f.partial_cmp(&self.f)
            .unwrap_or(Ordering::Equal)
            .then_with(|| o.g.partial_cmp(&self.g).unwrap_or(Ordering::Equal))
            .then_with(|| o.u.cmp(&self.u))
    }
}

/// own[layer * N + cell]: 0 free, a net's id (its copper: free to that net alone), -1 blocked. vown[cell]: the same
/// for a via's barrel. used / vused: the other ways standing on a cell / within a via's spacing of a cell. hist /
/// vhist: the cells' and the via sites' history. inz[cell]: 1 inside the board. lc[layer]: the layer's cost (<= 0: not
/// routed on). Cells are layer * N + j * w + i. -> the way's cells from a source to a target, or None.
#[pyfunction]
#[allow(clippy::too_many_arguments)]
pub fn plan_search(
    own: PyReadonlyArray1<i32>,
    vown: PyReadonlyArray1<i32>,
    used: PyReadonlyArray1<i32>,
    vused: PyReadonlyArray1<i32>,
    hist: PyReadonlyArray1<f64>,
    vhist: PyReadonlyArray1<f64>,
    inz: PyReadonlyArray1<u8>,
    lc: Vec<f64>,
    w: i64,
    nid: i32,
    h: f64,
    cmin: f64,
    via_cost: f64,
    pf: f64,
    sources: Vec<i64>,
    targets: Vec<i64>,
    txy: Vec<(i64, i64)>,
    v_off: Vec<i64>,
) -> PyResult<Option<Vec<i64>>> {
    let own = own.as_slice()?;
    let vown = vown.as_slice()?;
    let used = used.as_slice()?;
    let vused = vused.as_slice()?;
    let hist = hist.as_slice()?;
    let vhist = vhist.as_slice()?;
    let inz = inz.as_slice()?;
    let n = inz.len() as i64;
    let nl = lc.len() as i64;
    let total = n * nl;
    let sq2 = std::f64::consts::SQRT_2;
    let tgt: FxHashSet<i64> = targets.iter().copied().collect();
    let hf = |idx: i64| -> f64 {
        let (j, i) = (idx / w, idx % w);
        let mut d = 1.0e9_f64;
        for &(ti, tj) in &txy {
            let (di, dj) = ((ti - i).abs(), (tj - j).abs());
            let e = (di.max(dj) as f64) + (sq2 - 1.0) * (di.min(dj) as f64);
            if e < d {
                d = e;
            }
        }
        d * h * cmin
    };
    // (the step, its length, the two cells a diagonal passes between)
    let moves: [(i64, f64, i64, i64); 8] = [
        (1, h, 0, 0),
        (-1, h, 0, 0),
        (w, h, 0, 0),
        (-w, h, 0, 0),
        (w + 1, h * sq2, 1, w),
        (w - 1, h * sq2, -1, w),
        (-w + 1, h * sq2, 1, -w),
        (-w - 1, h * sq2, -1, -w),
    ];
    let free = |o: i32| o == 0 || o == nid;
    let mut g: FxHashMap<i64, f64> = FxHashMap::default();
    let mut par: FxHashMap<i64, i64> = FxHashMap::default();
    let mut heap = BinaryHeap::new();
    for &u in &sources {
        if u < 0 || u >= total {
            continue;
        }
        g.insert(u, 0.0);
        heap.push(Entry { f: hf(u % n), g: 0.0, u });
    }
    let mut best = f64::INFINITY;
    let mut best_end: Option<i64> = None;
    while let Some(Entry { f, g: gu, u }) = heap.pop() {
        if f >= best {
            break;
        }
        if gu > *g.get(&u).unwrap_or(&f64::INFINITY) {
            continue;
        }
        if tgt.contains(&u) {
            best = gu;
            best_end = Some(u);
            continue;
        }
        let (li, idx) = (u / n, u % n);
        let lcu = lc[li as usize];
        for &(d, sl, o1, o2) in &moves {
            let v = u + d;
            let vi = idx + d;
            if vi < 0 || vi >= n {
                continue;
            }
            let o = own[v as usize];
            if !free(o) && !tgt.contains(&v) {
                continue;
            }
            if o1 != 0 {
                let (a, b) = (idx + o1, idx + o2);
                if a < 0 || a >= n || b < 0 || b >= n {
                    continue;
                }
                if !free(own[(u + o1) as usize]) || !free(own[(u + o2) as usize]) {
                    continue;
                }
            }
            let gv = gu + sl * lcu * (1.0 + hist[v as usize]) * (1.0 + pf * (used[v as usize] as f64));
            if inz[vi as usize] == 0 {
                continue;
            }
            if gv < *g.get(&v).unwrap_or(&f64::INFINITY) {
                g.insert(v, gv);
                par.insert(v, u);
                heap.push(Entry { f: gv + hf(vi), g: gv, u: v });
            }
        }
        if free(vown[idx as usize]) {
            let mut share: i64 = 0;
            if pf != 0.0 {
                for k in 0..nl {
                    for &o in &v_off {
                        let c = k * n + idx + o;
                        if c >= 0 && c < total {
                            share += used[c as usize] as i64;
                        }
                    }
                }
                share += vused[idx as usize] as i64;
            }
            let gv = gu + via_cost * (1.0 + vhist[idx as usize]) * (1.0 + pf * (share as f64));
            let hv = hf(idx);
            for k in 0..nl {
                if k == li || lc[k as usize] <= 0.0 {
                    continue;
                }
                let v = k * n + idx;
                if gv < *g.get(&v).unwrap_or(&f64::INFINITY) && free(own[v as usize]) {
                    g.insert(v, gv);
                    par.insert(v, u);
                    heap.push(Entry { f: gv + hv, g: gv, u: v });
                }
            }
        }
    }
    let end = match best_end {
        Some(e) => e,
        None => return Ok(None),
    };
    let mut path = vec![end];
    let mut u = end;
    while let Some(&p) = par.get(&u) {
        path.push(p);
        u = p;
    }
    path.reverse();
    Ok(Some(path))
}
