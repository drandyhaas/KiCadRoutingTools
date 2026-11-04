//! The whole route's ENDS ESTIMATE (awx/whole_ends.py: Ends.score) -- the only one: its two-layer route or, on more
//! routing layers, the crossing graph coloured on the layers (a plane's layer priced, route_layers.plane_kinds); one
//! destination or a GROUP of them (whole_frame.Group: each berth option's way in, its place on the group's walk) --
//! scored from tables awx/ends_kernel.py builds once a model: each option's point, layer, layer domain, legs, conflict
//! keys and hands; the ride of each tooth and berth together (and its wound turns and far-face ride) and the lane's
//! length; each pair of options' stacks and runs met; the plane ways each option kills; the feedback's tables. The ends
//! search scores some 400 000 states a choice. The route EXACT on a state's orders is CP-SAT, in the Python: `parts`
//! calls it back where the estimate would stand.
//!
//! The boxes, faces, cuts and places round them are whole_frame's arithmetic (grown, face, cut, cuts_tied, src_cut,
//! perim_s, perim_c, ring_side, Group.score's cut), ported exactly; what the Python computes from geometry once an
//! option (the ride, the runs met, a lane's length) comes in as its value. A Python float `%` is floor-signed, as
//! `pymod`; a Python sum() of floats is PySum.
//!
//! search: the ends search's own loop (Ends.search) -- each improving change of one lane's tooth or berth, one other
//! lane ejected where it is in the way, taken as found, sweep after sweep -- over the same lanes, ends and options in
//! the same order, its objective memoised per state, as the Python's.

use pyo3::prelude::*;
use pyo3::types::PyDict;
use rustc_hash::{FxHashMap, FxHashSet};
use std::sync::Mutex;

const E_: u8 = 0;
const N_: u8 = 1;
const W_: u8 = 2;
const S_: u8 = 3;

/// a destination cut: on the far face (its y), or WINDING (whole_frame.cut_point's face and coordinate), or a GROUP's
/// (its place on the group's walk, whole_frame.Group)
#[derive(Clone, Copy)]
enum Cut {
    Far(f64),
    Wind(u8, f64),
    Grp(f64),
}

/// a destination GROUP (whole_frame.Group, Ends.group): its walk's length and front, its box, and each berth option's
/// way in -- its place on the walk, the pins its leg passes, the pocket options it cannot be laid with -- with each
/// lane's part and each part's copper box (whole_frame.box_of)
struct GroupK {
    len: f64,
    f0: f64,
    f1: f64,
    bbox: [f64; 4],
    s: Vec<Vec<f64>>,
    pass: Vec<Vec<i64>>,
    pconf: Vec<Vec<Vec<(u32, u32)>>>,
    part: Vec<usize>,
    pbox: Vec<[f64; 4]>,
    leg_w: f64,
}

impl GroupK {
    fn in_front(&self, s: f64) -> bool {
        pymod(s - self.f0, self.len) <= pymod(self.f1 - self.f0, self.len) + 1e-9
    }

    /// Group.side: W on the front, N before it (round the north), S after it
    fn side(&self, s: f64, c: f64) -> u8 {
        if self.in_front(s) {
            return W_;
        }
        if pymod(s - c, self.len) < pymod(self.f0 - c, self.len) {
            N_
        } else {
            S_
        }
    }

    /// Group.cuts: for each gap between neighbouring places (`vs` sorted) the middle of its longest stretch off the
    /// front, [(index, s)]
    fn cuts(&self, vs: &[f64]) -> Vec<(usize, f64)> {
        let n = vs.len();
        let mut out = Vec::new();
        let gl = pymod(self.f0 - self.f1, self.len);
        for i in 0..n {
            let a = if i > 0 { vs[i - 1] } else { vs[n - 1] - self.len };
            let b = vs[i];
            let mut best: Option<(f64, f64)> = None;
            for k in [-2i64, -1, 0, 1] {
                let lo = self.f1 + k as f64 * self.len;
                let hi = self.f1 + gl + k as f64 * self.len;
                let x0 = if lo > a { lo } else { a };
                let x1 = if hi < b { hi } else { b };
                if x1 - x0 > 1e-9 && (best.is_none() || x1 - x0 > best.unwrap().1 - best.unwrap().0) {
                    best = Some((x0, x1));
                }
            }
            if let Some((x0, x1)) = best {
                out.push((i, pymod((x0 + x1) / 2.0, self.len)));
            }
        }
        out
    }

    /// Group.score's cut: the places `pl` unrolled at the cut off the front that crosses the fewest against the
    /// launch order (`rank` each lane's place in it); None where there is no cut
    fn best_cut(&self, pl: &[f64], rank: &[i64]) -> Option<f64> {
        let n = pl.len();
        let mut order: Vec<usize> = (0..n).collect();
        order.sort_by(|&a, &b| pl[a].partial_cmp(&pl[b]).unwrap().then(rank[a].cmp(&rank[b])));
        let a: Vec<i64> = order.iter().map(|&l| rank[l]).collect();
        let vs: Vec<f64> = order.iter().map(|&l| pl[l]).collect();
        let cuts = self.cuts(&vs);
        if cuts.is_empty() {
            return None;
        }
        let mut inv0: i64 = 0;
        for i in 0..n {
            for j in i + 1..n {
                if a[i] > a[j] {
                    inv0 += 1;
                }
            }
        }
        let mut cum: Vec<i64> = vec![0; n + 1];
        for i in 0..n {
            cum[i + 1] = cum[i] + (n as i64 - 1 - 2 * a[i]);
        }
        let mut best: Option<(i64, usize, f64)> = None;
        for &(i, s) in &cuts {
            let key = (inv0 + cum[i], i);
            if best.is_none() || key < (best.unwrap().0, best.unwrap().1) {
                best = Some((key.0, key.1, s));
            }
        }
        best.map(|b| b.2)
    }
}

/// the PARTS of a state (Ends.score's), the objective's terms and what the whole route reads of them
#[derive(Default, Clone)]
struct Parts {
    obj: f64,
    fan: i64,
    parity: i64,
    crossover: i64,
    cover: i64,
    settle: i64,
    couple: i64,
    route: i64,
    route_est: i64,
    over: i64,
    plane: f64,
    cong: f64,
    feedback: f64,
    ride: f64,
    chg: Vec<i64>,
    crossings: i64,
    same: i64,
    conflicts: i64,
    splits: i64,
    refused: i64,
    lane_over: Vec<i64>,
    lane_load: Vec<f64>,
    lane_x: Vec<i64>,
    stacks: i64,
    exact_failed: bool,
    cut: (u8, f64), // (0 far face, 1 'N' / 2 'S' winding, 3 a group's walk; its coordinate)
    front: f64,
    dropped: i64,
    walled: Vec<usize>,
    learned: f64,
    scut: Option<f64>,
    far: i64,
    fence: f64,
    legs: f64,
    conf_lanes: Vec<usize>,
}

/// what a state's route EXACT on its orders reads (Ends.exact_route_nl / exact_route, CP-SAT in the Python): the
/// orders, each end's layer domain (more layers than two) or layers (two), the stub vias, the crossover pairs, the
/// via ends whose runs meet, the lanes' lengths
struct ExactIn {
    kp: Vec<(f64, usize)>,
    kd: Vec<(f64, usize)>,
    ends: Vec<[(i32, i32, u32); 2]>,
    sv: Vec<i64>,
    xo: Vec<i64>,
    sep: Vec<((usize, usize), (usize, usize))>,
    tl: Vec<i32>,
    dl: Vec<i32>,
}

/// ...and its answer: (route, over, changes a lane, drops a lane, plane price); None where it found no plan
type ExactOut = Option<(i64, i64, Vec<i64>, Vec<i64>, f64)>;

#[derive(Clone)]
struct Opt {
    x: f64,
    y: f64,
    own: i32,     // its layer's index among the routing layers (-1: none)
    lid: i32,     // its layer (any name)
    fan: i64,     // the option's vias (opt[3])
    refused: bool,
    fixed: i32,   // end_domain: a surface end's layer (-1: a via end)
    neck: i32,    // ...a via end's neck (-1: none)
    allowed: u32, // ...a via end's run layers, a bit each
    via_end: bool,
    hand: i32,    // a pair's hand there (pairs.hand: +1 / -1, 0 undecided or a single)
    is_far: bool, // (a tooth) its first move leaves by the source's far face
    side_hit: bool, // (a tooth, not far) up or down past a fence across its side (Ends._far_fence)
    legs: Vec<(f64, f64, i64)>, // each leg's move: exit point, vias
    keys: Vec<u32>,             // each leg's move, as conflict keys
    kills: Vec<u32>,            // the plane ways its copper stands on
    nbrs: Vec<(u32, Vec<u32>)>, // (a via end) the other lanes' options whose runs meet its: (lane, options)
    stacks: Vec<(u32, u32, i64)>, // (lane after this one, its option, weight): exits stacked on another layer
    oconf: Vec<(u32, u32)>,       // the other lanes' options (same end) it cannot be laid with (Ends.oconf)
}

struct Ball {
    key: u32,
    dw: Vec<u32>,
    sw: Vec<(u32, i64)>, // (strap way, the ball key it joins; -1 no ball)
}

#[pyclass]
pub struct EndsKernel {
    nl: usize,
    nlanes: usize,
    npair: Vec<i64>,
    tie: Vec<i64>,
    fb_over: Vec<i64>,
    has_fb_over: bool,
    opts: [Vec<Vec<Opt>>; 2],
    conf: Vec<Vec<u32>>,
    nkeys: usize,
    meet: [FxHashSet<(u32, u32, u32, u32)>; 2],
    ride: Vec<Vec<f64>>,  // per lane: [(ti * nB + bi) * 3 + side], side 0 none, 1 'N', 2 'S'
    nturn: Vec<Vec<i64>>, // per lane: [(ti * nB + bi) * 2 + side], side 0 'N', 1 'S'
    farx: Vec<Vec<f64>>,  // per lane: the far-face ride's extra, the same
    balls: Vec<Ball>,
    nball_keys: usize,
    nways: usize,
    has_pw: bool,
    fb_avoid: Vec<(usize, usize, Vec<f64>)>,
    fb_pairs: Vec<(usize, usize, usize, Vec<f64>, Vec<f64>)>,
    front: Vec<(usize, usize, Vec<f64>)>,
    x_learned: FxHashMap<(u32, u32), f64>,
    opp_learned: Vec<f64>,
    has_x: bool,
    has_opp: bool,
    sbox: [f64; 4],
    dbox: [f64; 4],
    fence_n: usize,
    fence_s: usize,
    far_w: Vec<f64>, // per lane: its far-face tooth's price (Ends.far_w: FAR_TOOTH, or none where its every other way out
    //                  crosses another bus's laid lanes)
    escape: bool,
    neumaier: bool,
    // a routing layer's PLANE (route_layers.plane_kinds: 0 a signal layer, 1 a power plane's, 2 a ground plane's) and
    // a run's price on it, per run whatever its length (route_layers.run_price)
    pk: Vec<u8>,
    p_gnd: f64,
    p_pwr: f64,
    end_learned: Vec<f64>, // (Ends.end_learned) a lane's fixed end layers the solve could not hold, per fixed end
    has_end: bool,
    group: Option<GroupK>,
    key_lane: Vec<u32>, // each conflict key's lane
    memo: Mutex<FxHashMap<(Vec<u32>, u8, u64), f64>>, // each state scored, at its cut: its objective
    // constants (whole_ends')
    w_over: f64,
    via_pref: i64,
    stack_cost: f64,
    via_mm: f64,
    load_ok: f64,
    w_cong: f64,
    x_trunk: f64,
    fb_avoid_w: f64,
    fb_pair_w: f64,
    eps_x: f64,
    big: f64,
    walled_w: f64,
    fence_vias: f64,
    opp_dive: f64,
    lane_pitch: f64,
    chg_room: f64,
    lpitch: f64,
    tie_mm: f64,
}

/// Python's sum() of floats: since 3.12 compensated (Neumaier, CPython's builtin_sum: the compensation added at the
/// end where it is not zero and finite); before it, left to right
struct PySum {
    f: f64,
    c: f64,
    neu: bool,
}

impl PySum {
    fn new(neu: bool) -> Self {
        PySum { f: 0.0, c: 0.0, neu }
    }
    fn add(&mut self, x: f64) {
        if self.neu {
            let t = self.f + x;
            if self.f.abs() >= x.abs() {
                self.c += (self.f - t) + x;
            } else {
                self.c += (x - t) + self.f;
            }
            self.f = t;
        } else {
            self.f += x;
        }
    }
    fn get(&self) -> f64 {
        if self.neu && self.c != 0.0 && self.c.is_finite() {
            self.f + self.c
        } else {
            self.f
        }
    }
}

fn pymod(a: f64, b: f64) -> f64 {
    // Python's float %: fmod, then the divisor's sign
    let m = a % b;
    if m != 0.0 {
        if (b < 0.0) != (m < 0.0) {
            m + b
        } else {
            m
        }
    } else {
        0.0f64.copysign(b)
    }
}

fn grown(b: &[f64; 4], pts: &mut dyn Iterator<Item = (f64, f64)>) -> [f64; 4] {
    let mut d: Vec<f64> = Vec::new();
    for (x, y) in pts {
        let mut v = b[0] - x;
        for w in [x - b[2], b[1] - y, y - b[3]] {
            if w > v {
                v = w;
            }
        }
        if v > 0.0 {
            d.push(v);
        }
    }
    d.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let n = d.len();
    let m = if n == 0 {
        0.0
    } else if n % 2 == 1 {
        d[n / 2]
    } else {
        (d[n / 2 - 1] + d[n / 2]) / 2.0
    };
    [b[0] - m, b[1] - m, b[2] + m, b[3] + m]
}

fn face(x: f64, y: f64, b: &[f64; 4]) -> u8 {
    // FACES = (E, N, W, S): a tie to the first
    let d = [(x - b[2]).abs(), (y - b[1]).abs(), (x - b[0]).abs(), (y - b[3]).abs()];
    let mut k = 0;
    for i in 1..4 {
        if d[i] < d[k] {
            k = i;
        }
    }
    k as u8
}

fn gaps_of(ys: &mut Vec<f64>, lo: f64, hi: f64) -> Vec<f64> {
    ys.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let mut pts = Vec::with_capacity(ys.len() + 2);
    pts.push(lo);
    pts.extend_from_slice(ys);
    pts.push(hi);
    pts
}

fn cut_y(mut ys: Vec<f64>, lo: f64, hi: f64) -> f64 {
    let pts = gaps_of(&mut ys, lo, hi);
    let mut i0 = 0;
    for i in 1..pts.len() - 1 {
        if pts[i + 1] - pts[i] > pts[i0 + 1] - pts[i0] {
            i0 = i;
        }
    }
    (pts[i0] + pts[i0 + 1]) / 2.0
}

fn cuts_tied(mut ys: Vec<f64>, lo: f64, hi: f64, tol: f64) -> Vec<f64> {
    let pts = gaps_of(&mut ys, lo, hi);
    let gaps: Vec<(f64, f64)> = (0..pts.len() - 1).map(|i| (pts[i + 1] - pts[i], (pts[i] + pts[i + 1]) / 2.0)).collect();
    let mut i0 = 0;
    for i in 1..gaps.len() {
        if gaps[i].0 > gaps[i0].0 {
            i0 = i;
        }
    }
    let mut out = vec![gaps[i0].1];
    for (i, g) in gaps.iter().enumerate() {
        if i != i0 && g.0 > gaps[i0].0 - tol {
            out.push(g.1);
        }
    }
    out
}

fn perim_d(x: f64, y: f64, db: &[f64; 4], ycut: f64) -> f64 {
    let (x0, y0, x1, y1) = (db[0], db[1], db[2], db[3]);
    let w = x1 - x0;
    let h = y1 - y0;
    let sd = face(x, y, db);
    if sd == E_ && y <= ycut {
        return ycut - y;
    }
    if sd == N_ {
        return (ycut - y0) + (x1 - x);
    }
    if sd == W_ {
        return (ycut - y0) + w + (y - y0);
    }
    if sd == S_ {
        return (ycut - y0) + w + h + (x - x0);
    }
    (ycut - y0) + 2.0 * w + h + (y1 - y)
}

fn cut_point(c: Cut, db: &[f64; 4]) -> (f64, f64) {
    match c {
        Cut::Far(v) => (db[2], v),
        Cut::Wind(f, v) => match f {
            N_ => (v, db[1]),
            S_ => (v, db[3]),
            _ => (db[2], v),
        },
        Cut::Grp(v) => (db[2], v), // (a group's cut is a place on its walk, never round a box: no caller asks)
    }
}

fn perim_c(x: f64, y: f64, db: &[f64; 4], c: Cut) -> f64 {
    match c {
        Cut::Far(v) => perim_d(x, y, db, v),
        Cut::Wind(_, _) | Cut::Grp(_) => {
            let w = db[2] - db[0];
            let h = db[3] - db[1];
            let (cx, cy) = cut_point(c, db);
            pymod(perim_d(x, y, db, db[1]) - perim_d(cx, cy, db, db[1]), 2.0 * (w + h))
        }
    }
}

/// ring_side: 1 'N', 3 'S', 255 None (the facing face)
fn ring_side(x: f64, y: f64, db: &[f64; 4], c: Cut) -> u8 {
    let f = face(x, y, db);
    if f == W_ {
        return 255;
    }
    match c {
        Cut::Far(v) => {
            if f == E_ {
                if y <= v {
                    N_
                } else {
                    S_
                }
            } else {
                f
            }
        }
        Cut::Wind(_, _) | Cut::Grp(_) => {
            if perim_c(x, y, db, c) < perim_c(db[0], db[1], db, c) {
                N_
            } else {
                S_
            }
        }
    }
}

fn perim_s(x: f64, y: f64, sb: &[f64; 4], scut: Option<f64>) -> f64 {
    let (sx0, sy0, sx1, sy1) = (sb[0], sb[1], sb[2], sb[3]);
    let sw = sx1 - sx0;
    let sh = sy1 - sy0;
    let scut = match scut {
        None => (sy0 + sy1) / 2.0,
        Some(v) => v,
    };
    let sd = face(x, y, sb);
    if sd == W_ && y <= scut {
        return scut - y;
    }
    if sd == N_ {
        return (scut - sy0) + (x - sx0);
    }
    if sd == E_ {
        return (scut - sy0) + sw + (y - sy0);
    }
    if sd == S_ {
        return (scut - sy0) + sw + sh + (sx1 - x);
    }
    (scut - sy0) + 2.0 * sw + sh + (sy1 - y)
}

fn between(lo: f64, hi: f64, v: f64, whole: f64) -> bool {
    if hi - lo <= whole / 2.0 {
        lo < v && v < hi
    } else {
        v > hi || v < lo
    }
}

/// whole_ends._crossed: two lanes CROSS where their places round the source's box (`pa`, `pb`) and the destination's
/// (`da`, `db`) stand in STRICTLY opposite orders -- a tie at either end (within `tie`: one exit, two layers) decides
/// nothing
fn crossed(pa: f64, pb: f64, da: f64, db: f64, tie: f64) -> bool {
    (pa < pb - tie && da > db + tie) || (pa > pb + tie && da < db - tie)
}

fn lt(a: (f64, usize), b: (f64, usize)) -> bool {
    a.0 < b.0 || (a.0 == b.0 && a.1 < b.1)
}

fn get_vec<'py, T: FromPyObject<'py>>(d: &Bound<'py, PyDict>, k: &str) -> PyResult<T> {
    match d.get_item(k)? {
        Some(v) => v.extract::<T>(),
        None => Err(pyo3::exceptions::PyKeyError::new_err(k.to_string())),
    }
}

type OptRow = (
    (f64, f64, i32, i32, i64, bool),
    (i32, i32, u32, bool, i32, bool, bool),
    Vec<(f64, f64, i64)>,
    Vec<u32>,
    Vec<u32>,
    Vec<(u32, Vec<u32>)>,
    Vec<(u32, u32, i64)>,
    Vec<(u32, u32)>,
);

fn cut_key(c: Option<Cut>) -> (u8, u64) {
    match c {
        None => (0, 0),
        Some(Cut::Far(v)) => (1, v.to_bits()),
        Some(Cut::Wind(f, v)) => (2 + f, v.to_bits()),
        Some(Cut::Grp(v)) => (10, v.to_bits()),
    }
}

/// the exact route's callback (Ends: CP-SAT): an ExactIn, its answer
type ExactFn<'a> = &'a dyn Fn(&ExactIn) -> ExactOut;

impl EndsKernel {
    /// the objective of `st` at the cut, each state scored once (Ends.objective's memo)
    fn obj(&self, st: &[(usize, usize)], wcut: Option<Cut>) -> f64 {
        let (f, b) = cut_key(wcut);
        let key: (Vec<u32>, u8, u64) = (st.iter().flat_map(|&(t, b_)| [t as u32, b_ as u32]).collect(), f, b);
        if let Some(&v) = self.memo.lock().unwrap().get(&key) {
            return v;
        }
        let v = self.score(st, wcut, None, None);
        self.memo.lock().unwrap().insert(key, v);
        v
    }

    /// Ends.search: each improving change of one lane's tooth or berth (one other lane EJECTED where it alone is in
    /// the way: moved to its best option clear of the others), taken as found, sweep after sweep until a sweep finds
    /// none -- the lanes in order, the berth before the tooth, the options in order; `tabu` (lane, end, option) never
    /// taken. Returns the state, its objective, and each sweep's best and state
    fn search_(
        &self,
        state: &[(usize, usize)],
        sweeps: usize,
        wcut: Option<Cut>,
        tabu: &FxHashSet<(usize, usize, usize)>,
    ) -> (Vec<(usize, usize)>, f64, Vec<(f64, Vec<(usize, usize)>)>) {
        let cur = |s: &[(usize, usize)], l: usize, k: usize| if k == 0 { s[l].0 } else { s[l].1 };
        let set = |s: &mut Vec<(usize, usize)>, l: usize, k: usize, i: usize| {
            if k == 0 {
                s[l].0 = i;
            } else {
                s[l].1 = i;
            }
        };
        let hits = |s: &[(usize, usize)], l: usize, k: usize, i: usize| -> (usize, usize) {
            // (the number of other lanes whose chosen option conflicts, and the last of them)
            let mut n = 0;
            let mut last = 0;
            for &(o, j) in &self.opts[k][l][i].oconf {
                if cur(s, o as usize, k) == j as usize {
                    n += 1;
                    last = o as usize;
                }
            }
            (n, last)
        };
        let mut st: Vec<(usize, usize)> = state.to_vec();
        let mut best = self.obj(&st, wcut);
        let mut log: Vec<(f64, Vec<(usize, usize)>)> = Vec::new();
        for _sw in 0..sweeps {
            let mut improved = false;
            for lane in 0..self.nlanes {
                for end in [1usize, 0usize] {
                    let n = self.opts[end][lane].len();
                    for i in 0..n {
                        if i == cur(&st, lane, end) || tabu.contains(&(lane, end, i)) {
                            continue;
                        }
                        let (nh, o) = hits(&st, lane, end, i);
                        if nh > 1 {
                            continue;
                        }
                        let mut trial = st.clone();
                        set(&mut trial, lane, end, i);
                        if nh == 1 {
                            // an EJECTION: the one lane in the way to its best option clear of the others
                            let mut bc: Option<(f64, usize)> = None;
                            for j in 0..self.opts[end][o].len() {
                                if hits(&trial, o, end, j).0 != 0 || tabu.contains(&(o, end, j)) {
                                    continue;
                                }
                                let mut t2 = trial.clone();
                                set(&mut t2, o, end, j);
                                let v = self.obj(&t2, wcut);
                                if bc.is_none() || v < bc.unwrap().0 {
                                    bc = Some((v, j));
                                }
                            }
                            match bc {
                                None => continue,
                                Some((_, j)) => set(&mut trial, o, end, j),
                            }
                        }
                        let v = self.obj(&trial, wcut);
                        if v < best - 1e-9 {
                            best = v;
                            st = trial;
                            improved = true;
                        }
                    }
                }
            }
            log.push((best, st.clone()));
            if !improved {
                break;
            }
        }
        (st, best, log)
    }

    fn opt(&self, k: usize, l: usize, st: &[(usize, usize)]) -> &Opt {
        &self.opts[k][l][if k == 0 { st[l].0 } else { st[l].1 }]
    }

    /// Ends._score: at the winding cut, a GROUP's at its own, else at the far face's cut -- its tied cuts each, the
    /// best. `exact`: the route exact on the orders (the Python's CP-SAT, called back), else the estimate's
    fn score(
        &self,
        st: &[(usize, usize)],
        wcut: Option<Cut>,
        exact: Option<ExactFn>,
        mut parts: Option<&mut Parts>,
    ) -> f64 {
        if let Some(c) = wcut {
            return self.score_at(st, Some(c), true, exact, parts);
        }
        if self.group.is_some() {
            return self.score_at(st, None, false, exact, parts);
        }
        let mut pts = (0..self.nlanes).map(|l| {
            let o = self.opt(1, l, st);
            (o.x, o.y)
        });
        let db = grown(&self.dbox, &mut pts);
        let ys: Vec<f64> = (0..self.nlanes)
            .map(|l| self.opt(1, l, st))
            .filter(|o| face(o.x, o.y, &db) == E_)
            .map(|o| o.y)
            .collect();
        let cuts = cuts_tied(ys, db[1], db[3], self.lpitch);
        if cuts.len() < 2 {
            return self.score_at(st, None, false, exact, parts);
        }
        let mut best: Option<(f64, Parts)> = None;
        for c in cuts {
            let mut t = Parts::default();
            let got = self.score_at(st, Some(Cut::Far(c)), false, exact, Some(&mut t));
            if best.is_none() || got < best.as_ref().unwrap().0 - 1e-9 {
                best = Some((got, t));
            }
        }
        let (v, t) = best.unwrap();
        if let Some(tt) = parts.as_deref_mut() {
            *tt = t;
        }
        v
    }

    fn score_at(
        &self,
        st: &[(usize, usize)],
        cut_at: Option<Cut>,
        wound_on: bool,
        exact: Option<ExactFn>,
        parts: Option<&mut Parts>,
    ) -> f64 {
        let nl_ = self.nlanes;
        let to = |l: usize| &self.opts[0][l][st[l].0];
        let bo = |l: usize| &self.opts[1][l][st[l].1];
        let mut fan: i64 = 0;
        for l in 0..nl_ {
            fan += to(l).fan + bo(l).fan + self.tie[l];
        }
        // the moves chosen, for the conflicts
        let mut chosen = vec![false; self.nkeys];
        let mut ck: Vec<u32> = Vec::with_capacity(4 * nl_);
        for l in 0..nl_ {
            for &k in to(l).keys.iter().chain(bo(l).keys.iter()) {
                if !chosen[k as usize] {
                    chosen[k as usize] = true;
                    ck.push(k);
                }
            }
        }
        let mut nc: i64 = 0;
        for &c in &ck {
            for &o in &self.conf[c as usize] {
                if chosen[o as usize] {
                    nc += 1;
                }
            }
        }
        let mut nconf = nc / 2;
        if let Some(g) = &self.group {
            // (a GROUP's lanes in one POCKET out of their order along it: Ends.pconf)
            let mut pc: i64 = 0;
            for l in 0..nl_ {
                for &(o, j) in &g.pconf[l][st[l].1] {
                    if st[o as usize].1 == j as usize {
                        pc += 1;
                    }
                }
            }
            nconf += pc / 2;
        }
        let nref = (0..nl_).filter(|&l| to(l).refused || bo(l).refused).count() as i64;
        // (the lanes whose chosen moves conflict: the destination re-plan frees them with their neighbours)
        let mut conf_lanes: Vec<usize> = Vec::new();
        if nconf > 0 {
            for &c in &ck {
                if self.conf[c as usize].iter().any(|&o| chosen[o as usize]) {
                    conf_lanes.push(self.key_lane[c as usize] as usize);
                }
            }
            conf_lanes.sort();
            conf_lanes.dedup();
        }
        // the orders, as the whole frame reads them
        let sb = grown(&self.sbox, &mut (0..nl_).map(|l| (to(l).x, to(l).y)));
        let db = match &self.group {
            Some(g) => g.bbox,
            None => grown(&self.dbox, &mut (0..nl_).map(|l| (bo(l).x, bo(l).y))),
        };
        let cut = match cut_at {
            Some(c) => c,
            None => match &self.group {
                // (a GROUP's: the place on its walk where the berths' ways in cross fewest against the teeth's order
                // round the source -- Group.score)
                Some(g) => {
                    let ps0: Vec<f64> = (0..nl_).map(|l| perim_s(to(l).x, to(l).y, &sb, None)).collect();
                    let mut o: Vec<usize> = (0..nl_).collect();
                    o.sort_by(|&a, &b| ps0[a].partial_cmp(&ps0[b]).unwrap().then(a.cmp(&b)));
                    let mut rank = vec![0i64; nl_];
                    for (i, &l) in o.iter().enumerate() {
                        rank[l] = i as i64;
                    }
                    let pl: Vec<f64> = (0..nl_).map(|l| g.s[l][st[l].1]).collect();
                    Cut::Grp(g.best_cut(&pl, &rank).unwrap_or(0.0))
                }
                None => Cut::Far(cut_y(
                    (0..nl_).filter(|&l| face(bo(l).x, bo(l).y, &db) == E_).map(|l| bo(l).y).collect(),
                    db[1],
                    db[3],
                )),
            },
        };
        let pd: Vec<f64> = match (&self.group, cut) {
            (Some(g), Cut::Grp(c)) => (0..nl_).map(|l| pymod(g.s[l][st[l].1] - c, g.len)).collect(),
            _ => (0..nl_).map(|l| perim_c(bo(l).x, bo(l).y, &db, cut)).collect(),
        };
        let kd: Vec<(f64, usize)> = (0..nl_).map(|l| (pd[l], l)).collect();
        let far_l: Vec<usize> = (0..nl_).filter(|&l| to(l).is_far).collect();
        let mut scut: Option<f64> = None;
        if !far_l.is_empty() {
            // whole_frame.src_cut, its rank the crossings a cut makes and the fences the far lanes ride past
            let mut ys: Vec<f64> = far_l.iter().map(|&l| to(l).y).collect();
            let pts = gaps_of(&mut ys, sb[1], sb[3]);
            let mut best: Option<(i64, f64, f64)> = None;
            for i in 0..pts.len() - 1 {
                let g = pts[i + 1] - pts[i];
                let c = (pts[i] + pts[i + 1]) / 2.0;
                let kp: Vec<(f64, usize)> = (0..nl_).map(|l| (perim_s(to(l).x, to(l).y, &sb, Some(c)), l)).collect();
                let mut r: i64 = 0;
                for a in 0..nl_ {
                    for b in a + 1..nl_ {
                        if crossed(kp[a].0, kp[b].0, kd[a].0, kd[b].0, self.tie_mm) {
                            r += 1;
                        }
                    }
                }
                for &l in &far_l {
                    r += (if to(l).y <= c { self.fence_n } else { self.fence_s }) as i64;
                }
                let key = (r, -g, c);
                let better = match best {
                    None => true,
                    Some(bk) => key.0 < bk.0 || (key.0 == bk.0 && (key.1 < bk.1 || (key.1 == bk.1 && key.2 < bk.2))),
                };
                if better {
                    best = Some(key);
                }
            }
            scut = Some(best.unwrap().2);
        }
        let kp: Vec<(f64, usize)> = (0..nl_).map(|l| (perim_s(to(l).x, to(l).y, &sb, scut), l)).collect();
        let mut inv: Vec<(usize, usize)> = Vec::new();
        for a in 0..nl_ {
            for b in a + 1..nl_ {
                if crossed(kp[a].0, kp[b].0, kd[a].0, kd[b].0, self.tie_mm) {
                    inv.push((a, b));
                }
            }
        }
        // split pairs: another lane's end on the pair's layer between its two tips, at either end
        let mut nsplit: i64 = 0;
        if self.npair.iter().any(|&n| n > 1) {
            for k in 0..2 {
                // (at a GROUP's berths, each lane round its own part's box at that box's own cut, among its own
                // part's lanes)
                let grp = if k == 1 { self.group.as_ref() } else { None };
                let mut pbx: Vec<([f64; 4], f64)> = Vec::new();
                if let Some(g) = grp {
                    for (p, b0) in g.pbox.iter().enumerate() {
                        let ls: Vec<usize> = (0..nl_).filter(|&l| g.part[l] == p).collect();
                        let b = grown(b0, &mut ls.iter().map(|&l| (bo(l).x, bo(l).y)));
                        let c = cut_y(
                            ls.iter().filter(|&&l| face(bo(l).x, bo(l).y, &b) == E_).map(|&l| bo(l).y).collect(),
                            b[1],
                            b[3],
                        );
                        pbx.push((b, c));
                    }
                }
                let bx_of = |l: usize| -> [f64; 4] {
                    match grp {
                        Some(g) => pbx[g.part[l]].0,
                        None => {
                            if k == 0 {
                                sb
                            } else {
                                db
                            }
                        }
                    }
                };
                let pv: Vec<Vec<f64>> = (0..nl_)
                    .map(|l| {
                        let o = self.opt(k, l, st);
                        o.legs
                            .iter()
                            .map(|&(x, y, _)| {
                                if k == 0 {
                                    perim_s(x, y, &sb, scut)
                                } else if let Some(g) = grp {
                                    let (b, c) = pbx[g.part[l]];
                                    perim_c(x, y, &b, Cut::Far(c))
                                } else {
                                    perim_c(x, y, &db, cut)
                                }
                            })
                            .collect()
                    })
                    .collect();
                for l in 0..nl_ {
                    if self.npair[l] < 2 {
                        continue;
                    }
                    let bx = bx_of(l);
                    let whole = 2.0 * (bx[2] - bx[0] + bx[3] - bx[1]);
                    let ol = self.opt(k, l, st);
                    let (mut a_, mut b_) = (pv[l][0], pv[l][1]);
                    if b_ < a_ {
                        std::mem::swap(&mut a_, &mut b_);
                    }
                    for o in 0..nl_ {
                        let oo = self.opt(k, o, st);
                        if o == l || oo.lid != ol.lid {
                            continue;
                        }
                        if let Some(g) = grp {
                            if g.part[o] != g.part[l] {
                                continue;
                            }
                        }
                        if self.nl > 2 && (ol.via_end || oo.via_end) {
                            continue;
                        }
                        if pv[o].iter().any(|&v| between(a_, b_, v, whole)) {
                            nsplit += 1;
                        }
                    }
                }
            }
        }
        let sv: Vec<i64> = (0..nl_)
            .map(|l| {
                to(l).legs.iter().zip(bo(l).legs.iter()).map(|(t, b)| t.2 + b.2).max().unwrap_or(0)
            })
            .collect();
        let head = Parts {
            fan,
            conflicts: nconf,
            refused: nref,
            splits: nsplit,
            crossings: inv.len() as i64,
            conf_lanes,
            scut,
            far: far_l.len() as i64,
            ..Default::default()
        };
        if self.nl == 2 {
            // two routing layers (Ends._score_at's own route: the cover, the settling, the coupling)
            let (chg, xo, cover, settle, couple) = self.route_two(st, &kp, &kd, &inv, &sv);
            let flat = |l: usize| to(l).lid == bo(l).lid;
            let mut h = head;
            h.parity = (0..nl_).filter(|&l| !flat(l)).map(|l| self.npair[l]).sum();
            h.crossover = (0..nl_).map(|l| self.npair[l] * xo[l]).sum();
            h.cover = cover;
            h.settle = settle;
            h.couple = couple;
            h.same = inv
                .iter()
                .filter(|&&(a, b)| flat(a) && flat(b) && to(a).lid == to(b).lid && xo[a] == 0 && xo[b] == 0)
                .count() as i64;
            let opp = vec![false; nl_];
            let xin = exact.map(|f| {
                (f, ExactIn {
                    kp: kp.clone(),
                    kd: kd.clone(),
                    ends: Vec::new(),
                    sv: sv.clone(),
                    xo: xo.clone(),
                    sep: Vec::new(),
                    tl: (0..nl_).map(|l| to(l).lid).collect(),
                    dl: (0..nl_).map(|l| bo(l).lid).collect(),
                })
            });
            let zero = vec![0i64; nl_];
            return self.tail(
                st, h, &kd, &inv, sv, chg, &opp, &xo, cut, &sb, &db, scut, &far_l, wound_on, false, &zero, &zero, xin,
                parts,
            );
        }
        self.score_nl(st, head, &kp, &kd, &inv, &sv, cut, &sb, &db, scut, &far_l, wound_on, exact, parts)
    }

    #[allow(clippy::too_many_arguments)]
    fn score_nl(
        &self,
        st: &[(usize, usize)],
        mut head: Parts,
        kp: &[(f64, usize)],
        kd: &[(f64, usize)],
        inv: &[(usize, usize)],
        sv: &[i64],
        cut: Cut,
        sb: &[f64; 4],
        db: &[f64; 4],
        scut: Option<f64>,
        far_l: &[usize],
        wound_on: bool,
        exact: Option<ExactFn>,
        parts: Option<&mut Parts>,
    ) -> f64 {
        let nl_ = self.nlanes;
        let nl = self.nl as i32;
        // each end's layer domain, then the chosen ends' runs against each other (Ends.end_clashes)
        let mut ends: Vec<[(i32, i32, u32); 2]> = (0..nl_)
            .map(|l| {
                let a = self.opt(0, l, st);
                let b = self.opt(1, l, st);
                [(a.fixed, a.neck, a.allowed), (b.fixed, b.neck, b.allowed)]
            })
            .collect();
        let mut vend: Vec<(usize, usize)> = Vec::new();
        for l in 0..nl_ {
            for k in 0..2 {
                if ends[l][k].0 < 0 {
                    vend.push((l, k));
                }
            }
        }
        for &(l, k) in &vend {
            let ok = ends[l][k].2;
            let own = self.opt(k, l, st).own;
            let ia = if k == 0 { st[l].0 } else { st[l].1 } as u32;
            let mut lose: u32 = 0;
            for l2 in 0..nl_ {
                let q = ends[l2][k].0;
                if q < 0 || l2 == l || ok & (1u32 << q) == 0 || lose & (1u32 << q) != 0 || q == own {
                    continue;
                }
                let ib = if k == 0 { st[l2].0 } else { st[l2].1 } as u32;
                if self.meet[k].contains(&(l as u32, ia, l2 as u32, ib)) {
                    lose |= 1u32 << q;
                }
            }
            if lose != 0 {
                ends[l][k].2 = ok & !lose;
            }
        }
        // neighbours: the crossings, and (an escape state) two via ends whose runs meet, own layers apart -- those
        // pairs (Ends.end_clashes' sep) the exact route reads too
        let mut nb: Vec<Vec<bool>> = vec![vec![false; nl_]; nl_];
        for &(a, b) in inv {
            nb[a][b] = true;
            nb[b][a] = true;
        }
        let mut sep: Vec<(usize, usize)> = Vec::new();
        if self.escape || exact.is_some() {
            let mut at: FxHashMap<(usize, usize), usize> = FxHashMap::default();
            for (i, &e) in vend.iter().enumerate() {
                at.insert(e, i);
            }
            for (i, &(la, ka)) in vend.iter().enumerate() {
                let oa = self.opt(ka, la, st);
                for (lb, ibs) in &oa.nbrs {
                    let lb = *lb as usize;
                    if let Some(&j) = at.get(&(lb, ka)) {
                        let ib = if ka == 0 { st[lb].0 } else { st[lb].1 } as u32;
                        if j > i && ibs.contains(&ib) && oa.own != self.opt(ka, lb, st).own && la != lb {
                            sep.push((i, j));
                            if self.escape {
                                nb[la][lb] = true;
                                nb[lb][la] = true;
                            }
                        }
                    }
                }
            }
            sep.sort();
        }
        // the crossing graph coloured on the routing layers (Ends._colour)
        let deg: Vec<usize> = (0..nl_).map(|l| nb[l].iter().filter(|&&v| v).count()).collect();
        let mut order: Vec<usize> = (0..nl_).collect();
        order.sort_by(|&a, &b| deg[b].cmp(&deg[a]).then(a.cmp(&b)));
        let mut col: Vec<Option<i32>> = vec![None; nl_]; // None: not coloured yet; Some(-1): coloured None
        let mut chg: Vec<i64> = vec![0; nl_];
        let mut drp: Vec<i64> = vec![0; nl_];
        let mut pln: Vec<f64> = vec![0.0; nl_];
        // (a plane's layer priced per run: route_layers.run_price)
        let pp = |_l: usize, c: i32| -> f64 {
            match self.pk[c as usize] {
                2 => self.p_gnd,
                1 => self.p_pwr,
                _ => 0.0,
            }
        };
        for &l in &order {
            let mut used: u64 = 0;
            for o in 0..nl_ {
                if nb[l][o] {
                    if let Some(c) = col[o] {
                        if c >= 0 {
                            used |= 1u64 << c;
                        }
                    }
                }
            }
            let free: Vec<i32> = (0..nl).filter(|&c| used & (1u64 << c) == 0).collect();
            let price = |c: i32| -> (i64, i64) {
                let (mut ch, mut dr) = (0i64, 0i64);
                for e in ends[l].iter() {
                    let (f, n, ok) = *e;
                    if f >= 0 {
                        ch += (c != f) as i64;
                    } else {
                        let in_ok = ok & (1u32 << c) != 0;
                        ch += (!in_ok) as i64;
                        dr += (c == n && in_ok) as i64;
                    }
                }
                (ch, dr)
            };
            let cands: Vec<i32> = if free.is_empty() { (0..nl).collect() } else { free.clone() };
            let mut pick = cands[0];
            let mut pk = price(pick);
            let mut pv = (pk.0 - pk.1) as f64 + pp(l, pick);
            for &c in &cands[1..] {
                let p = price(c);
                let v = (p.0 - p.1) as f64 + pp(l, c);
                if v < pv {
                    pick = c;
                    pk = p;
                    pv = v;
                }
            }
            col[l] = Some(if free.is_empty() { -1 } else { pick });
            chg[l] = pk.0 + if free.is_empty() { 2 } else { 0 };
            drp[l] = pk.1;
            pln[l] = pp(l, pick);
        }
        // an OPPOSITE-HANDS pair: a change at least, at its dive
        let mut xo: Vec<i64> = vec![0; nl_];
        let mut opp: Vec<bool> = vec![false; nl_];
        for l in 0..nl_ {
            if self.npair[l] == 2 {
                let a_ = self.opt(0, l, st).hand;
                let b_ = self.opt(1, l, st).hand;
                opp[l] = a_ != 0 && b_ != 0 && a_ != b_;
                if opp[l] && chg[l] == 0 {
                    xo[l] = if ends[l][0].0 >= 0 && ends[l][1].0 >= 0 { 2 } else { 1 };
                    chg[l] += xo[l];
                }
            }
        }
        let mut dsum: i64 = 0;
        for l in 0..nl_ {
            dsum += self.npair[l] * drp[l];
        }
        head.fan -= dsum;
        head.dropped = dsum;
        let svd: Vec<i64> = (0..nl_).map(|l| (sv[l] - drp[l]).max(0)).collect();
        let mut ps = PySum::new(self.neumaier);
        for l in 0..nl_ {
            ps.add(self.npair[l] as f64 * pln[l]);
        }
        head.plane = ps.get();
        head.parity = (0..nl_)
            .filter(|&l| ends[l][0].0 >= 0 && ends[l][1].0 >= 0 && ends[l][0].0 != ends[l][1].0)
            .map(|l| self.npair[l])
            .sum();
        head.crossover = (0..nl_).map(|l| self.npair[l] * xo[l]).sum();
        head.same = inv
            .iter()
            .filter(|&&(a, b)| matches!(col[a], Some(c) if c >= 0) && col[a] == col[b])
            .count() as i64;
        let nfixed: Vec<i64> = (0..nl_).map(|l| (ends[l][0].0 >= 0) as i64 + (ends[l][1].0 >= 0) as i64).collect();
        let xin = exact.map(|f| {
            (f, ExactIn {
                kp: kp.to_vec(),
                kd: kd.to_vec(),
                ends: ends.clone(),
                sv: svd.clone(),
                xo: xo.clone(),
                sep: sep.iter().map(|&(i, j)| (vend[i], vend[j])).collect(),
                tl: Vec::new(),
                dl: Vec::new(),
            })
        });
        self.tail(
            st, head, kd, inv, svd, chg, &opp, &xo, cut, sb, db, scut, far_l, wound_on, true, &drp, &nfixed, xin, parts,
        )
    }

    /// the route on TWO routing layers (Ends._score_at's own): each lane's changes -- one for ends on two layers, two
    /// for an opposite-hands pair on one layer end to end, two for a lane of each layer's COVER (the least-weight
    /// lanes whose dives leave the rest of that layer uncrossed: the complement of its heaviest chain rising in both
    /// orders); then each lane whose ends differ SETTLES the partners its one change cannot serve (an excursion, or
    /// the fewest of them dive: a minimum cut); then the lanes whose settling shares a choice COUPLE (each conflict
    /// resolved by its cheapest lane). Returns (changes, crossover changes) a lane, and the cover's, the settling's and
    /// the coupling's changes
    fn route_two(
        &self,
        st: &[(usize, usize)],
        kp: &[(f64, usize)],
        kd: &[(f64, usize)],
        inv: &[(usize, usize)],
        sv: &[i64],
    ) -> (Vec<i64>, Vec<i64>, i64, i64, i64) {
        let n = self.nlanes;
        let to = |l: usize| &self.opts[0][l][st[l].0];
        let bo = |l: usize| &self.opts[1][l][st[l].1];
        let tl: Vec<i32> = (0..n).map(|l| to(l).lid).collect();
        let dl: Vec<i32> = (0..n).map(|l| bo(l).lid).collect();
        // ('B.Cu' if a layer is 'F.Cu' else 'F.Cu', against an end's layer: as routing-layer indices, F.Cu 0, B.Cu 1;
        // an end off both is neither)
        let town: Vec<i32> = (0..n).map(|l| to(l).own).collect();
        let down: Vec<i32> = (0..n).map(|l| bo(l).own).collect();
        let flat: Vec<bool> = (0..n).map(|l| tl[l] == dl[l]).collect();
        let mut xo: Vec<i64> = vec![0; n];
        for l in 0..n {
            if self.npair[l] == 2 && flat[l] {
                let (a_, b_) = (to(l).hand, bo(l).hand);
                xo[l] = if a_ != 0 && b_ != 0 && a_ != b_ { 2 } else { 0 };
            }
        }
        let wo = self.w_over;
        let base: Vec<i64> = (0..n).map(|l| sv[l] + if flat[l] { 0 } else { 1 } + xo[l]).collect();
        let ov = |l: usize, k: i64| -> i64 { (base[l] + k - self.via_pref).max(0) };
        let w: Vec<f64> = (0..n).map(|l| (2 * self.npair[l]) as f64 + wo * (ov(l, 2) - ov(l, 0)) as f64).collect();
        // the cover (Ends._cover): per layer, the lanes on it end to end in launch order, the heaviest chain rising in
        // the berth order too kept, the rest covered
        let mut in_cov = vec![false; n];
        let mut layers: Vec<i32> = (0..n).filter(|&l| flat[l] && xo[l] == 0).map(|l| tl[l]).collect();
        layers.sort();
        layers.dedup();
        for &lay in &layers {
            let mut ls: Vec<usize> = (0..n).filter(|&l| flat[l] && xo[l] == 0 && tl[l] == lay).collect();
            ls.sort_by(|&a, &b| if lt(kp[a], kp[b]) { std::cmp::Ordering::Less } else { std::cmp::Ordering::Greater });
            let mut best: Vec<f64> = vec![0.0; ls.len()];
            let mut prev: Vec<Option<usize>> = vec![None; ls.len()];
            for i in 0..ls.len() {
                let a = ls[i];
                best[i] = w[a];
                for j in 0..i {
                    let b = ls[j];
                    if lt(kd[b], kd[a]) && best[j] + w[a] > best[i] {
                        best[i] = best[j] + w[a];
                        prev[i] = Some(j);
                    }
                }
            }
            if ls.is_empty() {
                continue;
            }
            // (max by (best, -index in the run): the heaviest, the earliest lane on a tie)
            let mut x = 0;
            for i in 1..ls.len() {
                if best[i] > best[x] || (best[i] == best[x] && kp[ls[i]].1 < kp[ls[x]].1) {
                    x = i;
                }
            }
            let mut chain = vec![false; ls.len()];
            let mut cur = Some(x);
            while let Some(i) = cur {
                chain[i] = true;
                cur = prev[i];
            }
            for i in 0..ls.len() {
                if !chain[i] {
                    in_cov[ls[i]] = true;
                }
            }
        }
        let cover: i64 = (0..n).filter(|&l| in_cov[l]).map(|l| self.npair[l]).sum();
        let mut settle: i64 = 0;
        let mut couple: i64 = 0;
        let mut chg: Vec<i64> =
            (0..n).map(|l| (if flat[l] { 0 } else { 1 }) + xo[l] + if in_cov[l] { 2 } else { 0 }).collect();
        // the settling
        let rank = |key: &[(f64, usize)]| -> Vec<i64> {
            let mut o: Vec<usize> = (0..n).collect();
            o.sort_by(|&a, &b| if lt(key[a], key[b]) { std::cmp::Ordering::Less } else { std::cmp::Ordering::Greater });
            let mut r = vec![0i64; n];
            for (i, &l) in o.iter().enumerate() {
                r[l] = i as i64;
            }
            r
        };
        let rl = rank(kp);
        let rf = rank(kd);
        let mut xs: Vec<Vec<usize>> = vec![Vec::new(); n];
        let mut cset: FxHashSet<(usize, usize)> = FxHashSet::default();
        for &(a, b) in inv {
            xs[a].push(b);
            xs[b].push(a);
            cset.insert((a.min(b), a.max(b)));
        }
        let crossed = |a: usize, b: usize| cset.contains(&(a.min(b), a.max(b)));
        let tcr = |p: usize, m: usize| -> f64 {
            ((rl[m] - rl[p]) as f64) / (((rl[m] - rl[p]) - (rf[m] - rf[p])) as f64)
        };
        let ovk = |l: usize, k: i64| -> i64 { (sv[l] + k - self.via_pref).max(0) };
        let mut sn: Vec<usize> = (0..n).filter(|&l| !flat[l]).collect();
        sn.sort_by_key(|&l| rl[l]);
        for &p in &sn {
            let mut part: Vec<usize> = xs[p].clone();
            part.sort_by(|&a, &b| tcr(p, a).partial_cmp(&tcr(p, b)).unwrap());
            let ps: Vec<(usize, i32)> = part
                .into_iter()
                .filter(|&m| flat[m] && chg[m] == 0)
                .map(|m| (m, if town[m] == 0 { 1 } else { 0 }))
                .collect();
            let mut bad: Vec<(usize, usize)> = Vec::new();
            for i in 0..ps.len() {
                for j in i + 1..ps.len() {
                    let ((a, na), (b, nb)) = (ps[i], ps[j]);
                    if na == down[p] && nb == town[p] && !crossed(a, b) {
                        bad.push((a, b));
                    }
                }
            }
            if bad.is_empty() {
                continue;
            }
            let wt = |m: usize| wo * (ovk(m, 2) - ovk(m, 0)) as f64 + (2 * self.npair[m]) as f64;
            let div = min_cut_cover(&bad, &wt);
            let mut dsum: f64 = 0.0;
            for &m in &div {
                dsum += wt(m);
            }
            if wo * (ovk(p, chg[p] + 2) - ovk(p, chg[p])) as f64 + (2 * self.npair[p]) as f64 <= dsum {
                chg[p] += 2;
                settle += 2 * self.npair[p];
            } else {
                for &m in &div {
                    chg[m] = 2;
                    settle += 2 * self.npair[m];
                }
            }
        }
        // the coupling
        let tie = |x: usize, y: usize| (kp[x].0 - kp[y].0).abs() < 1e-3 || (kd[x].0 - kd[y].0).abs() < 1e-3;
        let mut conflicts: Vec<[usize; 4]> = Vec::new();
        for &(a, b) in inv {
            if !(flat[a] && flat[b] && chg[a] == 0 && chg[b] == 0 && tl[a] != tl[b]) || tie(a, b) {
                continue;
            }
            // (the lanes crossing both, in the run's order -- the Python sorts the set's members so)
            let mut ns: Vec<usize> = xs[a]
                .iter()
                .cloned()
                .filter(|&m| xs[b].contains(&m) && !flat[m] && chg[m] == 1 && !tie(m, a) && !tie(m, b))
                .collect();
            ns.sort();
            ns.dedup();
            if ns.len() < 2 {
                continue;
            }
            let want = |m: usize| -> bool {
                let first = if tl[m] != tl[a] { a } else { b };
                let pre = if (rl[a] - rl[m]) * (rl[b] - rl[m]) > 0 {
                    if (rl[a] - rl[m]).abs() < (rl[b] - rl[m]).abs() {
                        a
                    } else {
                        b
                    }
                } else {
                    a
                };
                first == pre
            };
            ns.sort_by(|&x, &y| tcr(a, x).partial_cmp(&tcr(a, y)).unwrap());
            for i in 0..ns.len() {
                for j in i + 1..ns.len() {
                    let (n1, n2) = (ns[i], ns[j]);
                    if !crossed(n1, n2) && !tie(n1, n2) && !want(n1) && want(n2) {
                        conflicts.push([a, b, n1, n2]);
                    }
                }
            }
        }
        while !conflicts.is_empty() {
            let fix_cost = |x: usize| -> f64 {
                if !flat[x] {
                    wo * (ovk(x, chg[x] + 2) - ovk(x, chg[x])) as f64 + (2 * self.npair[x]) as f64
                } else {
                    wo * (ovk(x, 2) - ovk(x, 0)) as f64 + (2 * self.npair[x]) as f64
                }
            };
            let mut cand: Vec<usize> = conflicts.iter().flat_map(|c| c.iter().cloned()).collect();
            cand.sort();
            cand.dedup();
            let mut pick: Option<(f64, usize)> = None;
            for &x in &cand {
                let cnt = conflicts.iter().filter(|c| c.contains(&x)).count();
                let v = fix_cost(x) / cnt as f64;
                if pick.is_none() || v < pick.unwrap().0 {
                    pick = Some((v, x));
                }
            }
            let x = pick.unwrap().1;
            chg[x] = if !flat[x] { chg[x] + 2 } else { 2 };
            couple += 2 * self.npair[x];
            conflicts.retain(|c| !c.contains(&x));
        }
        (chg, xo, cover, settle, couple)
    }

    /// the objective from a route's changes (`chg`; `svd` each lane's stub vias, its drops off) and the parts so far
    /// (`h`: the fan, the conflicts, splits and refusals, the route's own counts): the nets over two, the trunk's
    /// crossings and load, the route EXACT on the orders where asked (`exact`: the Python's CP-SAT, called back here
    /// -- after the trunk's load, which reads the estimate's changes, as the Python's did), the rides, the stacks, the
    /// feedback and fronts, the crossings LEARNED (`learned`: more routing layers than two, as the Python prices them
    /// only there; `nfixed` each lane's fixed ends, for its end lesson), the walled balls, the far teeth and fences, a
    /// group's legs. `drp` each lane's drops (more than two layers: the exact route's own replace them)
    #[allow(clippy::too_many_arguments)]
    fn tail(
        &self,
        st: &[(usize, usize)],
        mut h: Parts,
        kd: &[(f64, usize)],
        inv: &[(usize, usize)],
        svd: Vec<i64>,
        mut chg: Vec<i64>,
        opp: &[bool],
        xo: &[i64],
        cut: Cut,
        sb: &[f64; 4],
        db: &[f64; 4],
        scut: Option<f64>,
        far_l: &[usize],
        wound_on: bool,
        learned: bool,
        drp: &[i64],
        nfixed: &[i64],
        exact: Option<(ExactFn, ExactIn)>,
        parts: Option<&mut Parts>,
    ) -> f64 {
        let nl_ = self.nlanes;
        let ovk = |l: usize, k: i64| -> i64 { (svd[l] + k - self.via_pref).max(0) };
        let mut route: i64 = 0;
        let mut over: i64 = 0;
        for l in 0..nl_ {
            route += self.npair[l] * chg[l];
        }
        for l in 0..nl_ {
            over += ovk(l, chg[l]);
        }
        // the trunk: each crossing there its price, each lane's load shared among the layers
        let dcls: Vec<u8> = (0..nl_)
            .map(|l| {
                let b = self.opt(1, l, st);
                match cut {
                    Cut::Far(v) => {
                        let f = face(b.x, b.y, db);
                        if f == E_ {
                            if b.y < v {
                                N_
                            } else {
                                S_
                            }
                        } else {
                            f
                        }
                    }
                    Cut::Wind(_, _) => {
                        let r = ring_side(b.x, b.y, db, cut);
                        if r == 255 {
                            W_
                        } else {
                            r
                        }
                    }
                    // (a GROUP's lane by its way's place on the walk: Group.side)
                    Cut::Grp(c) => {
                        let g = self.group.as_ref().unwrap();
                        g.side(g.s[l][st[l].1], c)
                    }
                }
            })
            .collect();
        let gt = {
            let a = db[0] - sb[2];
            if self.lane_pitch > a {
                self.lane_pitch
            } else {
                a
            }
        };
        let mut xt: Vec<i64> = vec![0; nl_];
        for &(a, b) in inv {
            if !(dcls[a] == dcls[b] && dcls[a] != W_) {
                xt[a] += 1;
                xt[b] += 1;
            }
        }
        let xsum: i64 = xt.iter().sum();
        let mut cong = self.x_trunk * xsum as f64 / 2.0;
        let mut loads: Vec<f64> = vec![0.0; nl_];
        for l in 0..nl_ {
            let load = (xt[l] as f64 * self.lane_pitch / (self.nl as f64 - 1.0)
                + (if dcls[l] == W_ { chg[l] } else { 0 }) as f64 * self.chg_room)
                / gt;
            loads[l] = load;
            let ov = if load - self.load_ok > 0.0 { load - self.load_ok } else { 0.0 };
            cong += self.npair[l] as f64 * self.w_cong * ov * ov;
        }
        // the route EXACT on the orders: its changes, its nets over two, its drops and planes
        let route_est = route;
        let mut exact_failed = false;
        if let Some((f, xin)) = exact {
            match f(&xin) {
                Some((r, o, c, drx, p)) => {
                    route = r;
                    over = o;
                    chg = c;
                    if self.nl > 2 {
                        for l in 0..nl_ {
                            h.fan += self.npair[l] * (drp[l] - drx[l]);
                        }
                        h.plane = p;
                    }
                }
                None => exact_failed = true,
            }
        }
        if self.has_fb_over {
            over = 0;
            for l in 0..nl_ {
                over += ovk(l, chg[l]).max(self.fb_over[l]);
            }
        }
        // the wound lanes (a winding cut sends their berths round another ring than the far face's own cut would)
        let mut wound: Vec<u8> = vec![255; nl_];
        if wound_on {
            let far = cut_y(
                (0..nl_)
                    .filter(|&l| {
                        let b = self.opt(1, l, st);
                        face(b.x, b.y, db) == E_
                    })
                    .map(|l| self.opt(1, l, st).y)
                    .collect(),
                db[1],
                db[3],
            );
            for l in 0..nl_ {
                let b = self.opt(1, l, st);
                let s_w = ring_side(b.x, b.y, db, cut);
                if s_w != 255 && s_w != ring_side(b.x, b.y, db, Cut::Far(far)) {
                    wound[l] = s_w;
                }
            }
        }
        let nb_of = |l: usize| self.opts[1][l].len();
        let mut rs = PySum::new(self.neumaier);
        for l in 0..nl_ {
            let side = match wound[l] {
                N_ => 1,
                S_ => 2,
                _ => 0,
            };
            rs.add(self.ride[l][(st[l].0 * nb_of(l) + st[l].1) * 3 + side]);
        }
        let ride = rs.get();
        let mut ws: f64 = 0.0;
        for l in 0..nl_ {
            if wound[l] == 255 {
                continue;
            }
            let side = wound[l];
            let nturn = self.nturn[l][(st[l].0 * nb_of(l) + st[l].1) * 2 + if side == N_ { 0 } else { 1 }];
            let mut inside: i64 = 0;
            for o in 0..nl_ {
                if o != l
                    && dcls[o] == side
                    && (if side == N_ { lt(kd[l], kd[o]) } else { lt(kd[o], kd[l]) })
                {
                    inside += 1;
                }
            }
            ws += self.npair[l] as f64 * nturn as f64 * (std::f64::consts::PI / 2.0) * (inside as f64 / 2.0)
                * self.lane_pitch;
        }
        let mut fr: f64 = 0.0;
        if let Some(sc) = scut {
            for &l in far_l {
                let a = self.opt(0, l, st);
                let side = if a.y <= sc { 0 } else { 1 };
                fr += self.npair[l] as f64 * self.farx[l][(st[l].0 * nb_of(l) + st[l].1) * 2 + side];
            }
        }
        let ride = ride + ws + fr;
        // stacked ends
        let mut stacks: i64 = 0;
        for k in 0..2 {
            for la in 0..nl_ {
                for &(lb, ib, w) in &self.opt(k, la, st).stacks {
                    let lb = lb as usize;
                    let cur = if k == 0 { st[lb].0 } else { st[lb].1 } as u32;
                    if cur == ib && (self.escape || chg[la] != 0 || chg[lb] != 0) {
                        stacks += w;
                    }
                }
            }
        }
        // the whole route's feedback, the fronts
        let pick = |l: usize, k: usize| if k == 0 { st[l].0 } else { st[l].1 };
        let mut fa = PySum::new(self.neumaier);
        for (l, k, ix) in &self.fb_avoid {
            fa.add(ix[pick(*l, *k)]);
        }
        let mut fp = PySum::new(self.neumaier);
        for (a, b, k, ia, ib) in &self.fb_pairs {
            fp.add(ia[pick(*a, *k)] * ib[pick(*b, *k)]);
        }
        let fbk = self.fb_avoid_w * fa.get() + self.fb_pair_w * fp.get();
        let mut fs = PySum::new(self.neumaier);
        for (l, k, c) in &self.front {
            fs.add(c[pick(*l, *k)]);
        }
        let front = fs.get();
        // crossings the whole solve could not place, learned -- and pairs' hands, and fixed ends' layers
        let mut xlearn: f64 = 0.0;
        if learned && self.has_x {
            let mut s = PySum::new(self.neumaier);
            for &(a, b) in inv {
                s.add(*self.x_learned.get(&(a as u32, b as u32)).unwrap_or(&0.0));
            }
            xlearn = s.get();
        }
        if learned && self.has_opp {
            let mut s = PySum::new(self.neumaier);
            for l in 0..nl_ {
                if opp[l] {
                    s.add(self.opp_learned[l]);
                }
            }
            xlearn += s.get();
        }
        if learned && self.has_end {
            let mut s = PySum::new(self.neumaier);
            for l in 0..nl_ {
                s.add(self.end_learned[l] * nfixed[l] as f64);
            }
            xlearn += s.get();
        }
        if learned && self.opp_dive != 0.0 {
            let n = (0..nl_).filter(|&l| opp[l] && xo[l] == 0).count();
            xlearn += self.opp_dive * n as f64;
        }
        // the plane balls walled in
        let mut walled: Vec<usize> = Vec::new();
        if self.has_pw && !self.balls.is_empty() {
            let mut killed = vec![false; self.nways];
            for l in 0..nl_ {
                for k in 0..2 {
                    for &w in &self.opt(k, l, st).kills {
                        killed[w as usize] = true;
                    }
                }
            }
            let mut alive = vec![false; self.nball_keys];
            for b in &self.balls {
                alive[b.key as usize] = b.dw.iter().any(|&w| !killed[w as usize]);
            }
            for (i, b) in self.balls.iter().enumerate() {
                if !alive[b.key as usize]
                    && !b.sw.iter().any(|&(w, to)| !killed[w as usize] && to >= 0 && alive[to as usize])
                {
                    walled.push(i);
                }
            }
        }
        let nwalled = walled.len() as i64;
        // the far face's teeth and the fences
        let mut nf: f64 = 0.0;
        for l in 0..nl_ {
            let a = self.opt(0, l, st);
            if a.is_far {
                if let Some(sc) = scut {
                    if (if a.y <= sc { self.fence_n } else { self.fence_s }) > 0 {
                        nf += self.npair[l] as f64;
                    }
                }
                continue;
            }
            if a.side_hit {
                nf += self.npair[l] as f64;
            }
        }
        let fence = self.fence_vias * nf;
        // a GROUP's lanes' legs along their pockets (Ends._legs)
        let legs = match &self.group {
            Some(g) => g.leg_w * (0..nl_).map(|l| g.pass[l][st[l].1]).sum::<i64>() as f64,
            None => 0.0,
        };
        let ninv = inv.len() as i64;
        let (nconf, nsplit, nref) = (h.conflicts, h.splits, h.refused);
        let mut obj = self.w_over * over as f64;
        obj += h.fan as f64;
        obj += route as f64;
        obj += h.plane;
        obj += ride / self.via_mm;
        obj += cong;
        obj += fbk;
        obj += self.eps_x * ninv as f64;
        obj += self.big * (nconf + nsplit + nref) as f64;
        obj += self.stack_cost * stacks as f64;
        obj += front;
        obj += self.walled_w * nwalled as f64;
        if learned {
            obj += xlearn;
        }
        let mut fw: f64 = 0.0;
        for &l in far_l {
            fw += self.far_w[l];
        }
        obj += fw;
        obj += fence;
        if self.group.is_some() {
            obj += legs;
        }
        if let Some(p) = parts {
            h.obj = obj;
            h.route = route;
            h.route_est = route_est;
            h.over = over;
            h.cong = cong;
            h.feedback = fbk;
            h.ride = ride;
            h.lane_over = (0..nl_).map(|l| ovk(l, chg[l]).max(if self.has_fb_over { self.fb_over[l] } else { 0 })).collect();
            h.lane_load = loads;
            let mut lx = vec![0i64; nl_];
            for &(a, b) in inv {
                lx[a] += 1;
                lx[b] += 1;
            }
            h.lane_x = lx;
            h.chg = chg;
            h.stacks = stacks;
            h.exact_failed = exact_failed;
            h.cut = match cut {
                Cut::Far(v) => (0, v),
                Cut::Wind(f, v) => (if f == N_ { 1 } else { 2 }, v),
                Cut::Grp(v) => (3, v),
            };
            h.front = front;
            h.walled = walled;
            h.learned = xlearn;
            h.fence = fence;
            h.legs = legs;
            *p = h;
        }
        obj
    }
}

/// whole_ends.min_cut_cover: the least-weight vertex cover of the bipartite `edges` [(left, right)] (weights `wt`) --
/// a minimum s-t cut, the left lanes the source cannot reach in the residual graph and the right ones it can (that
/// set is the same for every maximum flow, so the augmenting order is free)
fn min_cut_cover(edges: &[(usize, usize)], wt: &dyn Fn(usize) -> f64) -> Vec<usize> {
    let mut left: Vec<usize> = edges.iter().map(|e| e.0).collect();
    let mut right: Vec<usize> = edges.iter().map(|e| e.1).collect();
    left.sort();
    left.dedup();
    right.sort();
    right.dedup();
    // nodes: 0 the source, 1 the sink, then the left lanes, then the right ones
    let nn = 2 + left.len() + right.len();
    let li = |a: usize| 2 + left.binary_search(&a).unwrap();
    let ri = |b: usize| 2 + left.len() + right.binary_search(&b).unwrap();
    let mut cap = vec![vec![0.0f64; nn]; nn];
    for &a in &left {
        cap[0][li(a)] += wt(a);
    }
    for &b in &right {
        cap[ri(b)][1] += wt(b);
    }
    for &(a, b) in edges {
        cap[li(a)][ri(b)] = f64::INFINITY;
    }
    loop {
        let mut prev: Vec<Option<usize>> = vec![None; nn];
        let mut seen = vec![false; nn];
        seen[0] = true;
        let mut q = std::collections::VecDeque::new();
        q.push_back(0);
        while let Some(u) = q.pop_front() {
            if seen[1] {
                break;
            }
            for v in 0..nn {
                if !seen[v] && cap[u][v] > 1e-12 {
                    seen[v] = true;
                    prev[v] = Some(u);
                    q.push_back(v);
                }
            }
        }
        if !seen[1] {
            let mut out: Vec<usize> = left.iter().cloned().filter(|&a| !seen[li(a)]).collect();
            out.extend(right.iter().cloned().filter(|&b| seen[ri(b)]));
            return out;
        }
        let mut f = f64::INFINITY;
        let mut v = 1;
        while let Some(u) = prev[v] {
            if cap[u][v] < f {
                f = cap[u][v];
            }
            v = u;
        }
        let mut v = 1;
        while let Some(u) = prev[v] {
            cap[u][v] -= f;
            cap[v][u] += f;
            v = u;
        }
    }
}

fn parse_cut(wcut: Option<(String, f64)>) -> Option<Cut> {
    wcut.map(|(f, v)| match f.as_str() {
        "N" => Cut::Wind(N_, v),
        "S" => Cut::Wind(S_, v),
        _ => Cut::Far(v),
    })
}

#[pymethods]
impl EndsKernel {
    #[new]
    fn new(d: &Bound<'_, PyDict>) -> PyResult<Self> {
        let nlanes: usize = get_vec(d, "nlanes")?;
        let mut opts: [Vec<Vec<Opt>>; 2] = [Vec::new(), Vec::new()];
        for k in 0..2 {
            let rows: Vec<Vec<OptRow>> = get_vec(d, if k == 0 { "teeth" } else { "berths" })?;
            opts[k] = rows
                .into_iter()
                .map(|lane| {
                    lane.into_iter()
                        .map(|(a, b, legs, keys, kills, nbrs, stacks, oconf)| Opt {
                            x: a.0,
                            y: a.1,
                            own: a.2,
                            lid: a.3,
                            fan: a.4,
                            refused: a.5,
                            fixed: b.0,
                            neck: b.1,
                            allowed: b.2,
                            via_end: b.3,
                            hand: b.4,
                            is_far: b.5,
                            side_hit: b.6,
                            legs,
                            keys,
                            kills,
                            nbrs,
                            stacks,
                            oconf,
                        })
                        .collect()
                })
                .collect();
        }
        let meet_rows: [Vec<(u32, u32, u32, u32)>; 2] = [get_vec(d, "meet0")?, get_vec(d, "meet1")?];
        let mut meet: [FxHashSet<(u32, u32, u32, u32)>; 2] = [FxHashSet::default(), FxHashSet::default()];
        for k in 0..2 {
            meet[k] = meet_rows[k].iter().cloned().collect();
        }
        let balls_rows: Vec<(u32, Vec<u32>, Vec<(u32, i64)>)> = get_vec(d, "balls")?;
        let c: Vec<f64> = get_vec(d, "consts")?;
        let fence: (usize, usize) = get_vec(d, "fence")?;
        let group = if get_vec::<bool>(d, "g_on")? {
            let bb: (f64, f64, f64, f64) = get_vec(d, "g_bbox")?;
            let pb: Vec<(f64, f64, f64, f64)> = get_vec(d, "g_pbox")?;
            Some(GroupK {
                len: get_vec(d, "g_len")?,
                f0: get_vec(d, "g_f0")?,
                f1: get_vec(d, "g_f1")?,
                bbox: [bb.0, bb.1, bb.2, bb.3],
                s: get_vec(d, "g_s")?,
                pass: get_vec(d, "g_pass")?,
                pconf: get_vec(d, "g_pconf")?,
                part: get_vec(d, "g_part")?,
                pbox: pb.into_iter().map(|b| [b.0, b.1, b.2, b.3]).collect(),
                leg_w: get_vec(d, "g_leg_w")?,
            })
        } else {
            None
        };
        Ok(EndsKernel {
            nl: get_vec(d, "nl")?,
            nlanes,
            npair: get_vec(d, "npair")?,
            tie: get_vec(d, "tie")?,
            fb_over: get_vec(d, "fb_over")?,
            has_fb_over: get_vec(d, "has_fb_over")?,
            opts,
            conf: get_vec(d, "conf")?,
            nkeys: get_vec(d, "nkeys")?,
            meet,
            ride: get_vec(d, "ride")?,
            nturn: get_vec(d, "nturn")?,
            farx: get_vec(d, "farx")?,
            balls: balls_rows.into_iter().map(|(key, dw, sw)| Ball { key, dw, sw }).collect(),
            nball_keys: get_vec(d, "nball_keys")?,
            nways: get_vec(d, "nways")?,
            has_pw: get_vec(d, "has_pw")?,
            fb_avoid: get_vec(d, "fb_avoid")?,
            fb_pairs: get_vec(d, "fb_pairs")?,
            front: get_vec(d, "front")?,
            x_learned: FxHashMap::default(),
            opp_learned: vec![0.0; nlanes],
            has_x: false,
            has_opp: false,
            sbox: get_vec(d, "sbox")?,
            dbox: get_vec(d, "dbox")?,
            fence_n: fence.0,
            fence_s: fence.1,
            far_w: get_vec(d, "far_w")?,
            escape: get_vec(d, "escape")?,
            neumaier: get_vec(d, "neumaier")?,
            pk: get_vec(d, "pk")?,
            p_gnd: get_vec(d, "p_gnd")?,
            p_pwr: get_vec(d, "p_pwr")?,
            end_learned: vec![0.0; nlanes],
            has_end: false,
            group,
            key_lane: get_vec(d, "key_lane")?,
            memo: Mutex::new(FxHashMap::default()),
            w_over: c[0],
            via_pref: c[1] as i64,
            stack_cost: c[2],
            via_mm: c[3],
            load_ok: c[4],
            w_cong: c[5],
            x_trunk: c[6],
            fb_avoid_w: c[7],
            fb_pair_w: c[8],
            eps_x: c[9],
            big: c[10],
            walled_w: c[11],
            fence_vias: c[12],
            opp_dive: c[13],
            lane_pitch: c[14],
            chg_room: c[15],
            lpitch: c[16],
            tie_mm: c[17],
        })
    }

    /// the crossings, pairs' hands and fixed ends the whole solve could not place (Ends.x_learned, Ends.opp_learned,
    /// Ends.end_learned): as the Python holds them now -- `x` [(lane, lane, vias)], `opp` and `end` [(lane, vias)],
    /// each dict's own emptiness
    #[allow(clippy::too_many_arguments)]
    #[pyo3(signature = (x, opp, has_x, has_opp, end=Vec::new(), has_end=false))]
    fn set_learned(
        &mut self,
        x: Vec<(u32, u32, f64)>,
        opp: Vec<(usize, f64)>,
        has_x: bool,
        has_opp: bool,
        end: Vec<(usize, f64)>,
        has_end: bool,
    ) {
        self.x_learned.clear();
        for (a, b, v) in x {
            self.x_learned.insert((a.min(b), a.max(b)), v);
        }
        self.opp_learned = vec![0.0; self.nlanes];
        for (l, v) in opp {
            self.opp_learned[l] = v;
        }
        self.end_learned = vec![0.0; self.nlanes];
        for (l, v) in end {
            self.end_learned[l] = v;
        }
        self.has_x = has_x;
        self.has_opp = has_opp;
        self.has_end = has_end;
        self.memo.lock().unwrap().clear();   // (as checked() clears the Python's: the prices moved)
    }

    /// Ends._score's objective of `state` [(tooth option, berth option)] a lane, at the winding cut `wcut`
    /// ((face, coordinate)) or the far face's
    #[pyo3(signature = (state, wcut=None))]
    fn objective(&self, py: Python<'_>, state: Vec<(usize, usize)>, wcut: Option<(String, f64)>) -> f64 {
        let c = parse_cut(wcut);
        py.allow_threads(|| self.obj(&state, c))
    }

    /// Ends.search from `state` (see search_): (state, objective, [(each sweep's best, its state)])
    #[pyo3(signature = (state, sweeps, wcut=None, tabu=Vec::new()))]
    #[allow(clippy::type_complexity)]
    fn search(
        &self,
        py: Python<'_>,
        state: Vec<(usize, usize)>,
        sweeps: usize,
        wcut: Option<(String, f64)>,
        tabu: Vec<(usize, usize, usize)>,
    ) -> (Vec<(usize, usize)>, f64, Vec<(f64, Vec<(usize, usize)>)>) {
        let c = parse_cut(wcut);
        let tb: FxHashSet<(usize, usize, usize)> = tabu.into_iter().collect();
        py.allow_threads(|| self.search_(&state, sweeps, c, &tb))
    }

    /// Ends.score's (objective, parts) of `state` at the winding cut `wcut`: every part the whole route reads, by lane
    /// index (the Python names them), and the route EXACT on the orders where `exact` is given -- the Python's CP-SAT,
    /// called back with (kp, kd, ends, sv, xo, sep, tl, dl) as each cut is scored, answering None or (route, over,
    /// changes, drops, plane)
    #[pyo3(signature = (state, wcut=None, exact=None))]
    fn parts<'py>(
        &self,
        py: Python<'py>,
        state: Vec<(usize, usize)>,
        wcut: Option<(String, f64)>,
        exact: Option<PyObject>,
    ) -> PyResult<(f64, Bound<'py, PyDict>)> {
        let err: std::cell::RefCell<Option<PyErr>> = std::cell::RefCell::new(None);
        let call = |xin: &ExactIn| -> ExactOut {
            if err.borrow().is_some() {
                return None;
            }
            let cb = exact.as_ref().unwrap();
            let ends: Vec<((i32, i32, u32), (i32, i32, u32))> = xin.ends.iter().map(|e| (e[0], e[1])).collect();
            let got = cb.call1(
                py,
                (xin.kp.clone(), xin.kd.clone(), ends, xin.sv.clone(), xin.xo.clone(), xin.sep.clone(),
                 xin.tl.clone(), xin.dl.clone()),
            );
            match got.and_then(|v| v.extract::<Option<(i64, i64, Vec<i64>, Vec<i64>, f64)>>(py)) {
                Ok(v) => v,
                Err(e) => {
                    *err.borrow_mut() = Some(e);
                    None
                }
            }
        };
        let mut p = Parts::default();
        let v = self.score(
            &state,
            parse_cut(wcut),
            if exact.is_some() { Some(&call as ExactFn) } else { None },
            Some(&mut p),
        );
        if let Some(e) = err.into_inner() {
            return Err(e);
        }
        let d = PyDict::new(py);
        d.set_item("fan", p.fan)?;
        d.set_item("parity", p.parity)?;
        d.set_item("crossover", p.crossover)?;
        d.set_item("cover", p.cover)?;
        d.set_item("settle", p.settle)?;
        d.set_item("couple", p.couple)?;
        d.set_item("route", p.route)?;
        d.set_item("route_est", p.route_est)?;
        d.set_item("over", p.over)?;
        d.set_item("plane", p.plane)?;
        d.set_item("cong", p.cong)?;
        d.set_item("feedback", p.feedback)?;
        d.set_item("ride", p.ride)?;
        d.set_item("chg", p.chg)?;
        d.set_item("crossings", p.crossings)?;
        d.set_item("same", p.same)?;
        d.set_item("conflicts", p.conflicts)?;
        d.set_item("splits", p.splits)?;
        d.set_item("refused", p.refused)?;
        d.set_item("lane_over", p.lane_over)?;
        d.set_item("lane_load", p.lane_load)?;
        d.set_item("lane_x", p.lane_x)?;
        d.set_item("stacks", p.stacks)?;
        d.set_item("exact_failed", p.exact_failed)?;
        d.set_item("cut", p.cut)?;
        d.set_item("front", p.front)?;
        d.set_item("dropped", p.dropped)?;
        d.set_item("walled", p.walled)?;
        d.set_item("learned", p.learned)?;
        d.set_item("scut", p.scut)?;
        d.set_item("far", p.far)?;
        d.set_item("fence", p.fence)?;
        d.set_item("legs", p.legs)?;
        d.set_item("conf_lanes", p.conf_lanes)?;
        Ok((v, d))
    }
}
