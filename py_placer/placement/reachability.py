"""Is a pad reachable at all, or is the router being blamed for geometry?

Across four recorded runs, **9 of 14** claims of the form *"no placement can fix
this"* were later refuted -- 64%. Every single claim that a pad was
unroutable-in-principle turned out to be wrong. They were made from renders, from
a router's failure to find a path, and from arithmetic on the wrong two edges.
None of them was made from a measurement of the actual free space.

This is that measurement. No physics, no heuristics, three classical steps:

  1. **Rasterise foreign copper, one grid per clearance class.** Clearance is
     PAIRWISE -- a KiCad netclass pair, plus `.kicad_dru` rules that scope a
     clearance to a layer or a net -- so one grid at one clearance is already
     the wrong model on any board with more than a Default class.

  2. **Euclidean distance transform per class**, then

         slack(p) = 2 * min_k( dist_k(p) - clearance_k )

     which is the widest track that legally fits at p. Linear time, exact
     Euclidean metric (not a chamfer approximation).

  3. **Widest path is a Kruskal problem, and Kruskal solves it exactly.** The
     best route from A to B is the one maximising its own minimum slack; add
     cells in descending slack under union-find, and the slack at which A and B
     first join IS the bottleneck. No search order, no grid alignment, no
     heuristic -- so it answers a question a gridded A* cannot:

         bottleneck >= track width  ->  a route EXISTS (the router may still
                                        miss it: that is a router finding)
         bottleneck <  track width  ->  no route exists at ANY grid. A real cage.

The distinction in that last block is the whole point. A router failing is
evidence about the router. This is evidence about the board.

Validated on the two cases that motivated it: a pad the router failed on at
every grid measured **PASSABLE by +5.5 um** against a hand-derived 6.4 um throat
(the "impossible" claim was false, and the hand arithmetic agreed to within a
micron), and a genuinely enclosed pad measured **CAGED**.

Needs scipy (`distance_transform_edt`, `label`). That is checked once, loudly,
because a silently-degraded reachability answer is worse than none.
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

def _open_room(view) -> float:
    """Clearance-adjusted room assigned to a cell with NO foreign copper of any
    class within reach.

    Derived from the view rather than a constant, and that is not cosmetic: a
    fixed sentinel that happens to exceed the own-copper value makes free space
    sort ABOVE own copper in the Kruskal order, so the whole open board unions
    together before the seed activates and the bottleneck comes back as the
    sentinel itself -- a number that looks like a measurement and is not. Tying
    it to the view keeps the ordering right on a sparse board and keeps the
    value geometrically meaningful (nothing in the view can be roomier than the
    view).
    """
    return max(view[2] - view[0], view[3] - view[1])


def _own_slack(view) -> float:
    """Own-net copper is a GIVEN, not a constraint: the pad and its existing
    tracks are already there and are traversable whatever the local slack says
    (a pad can score negative slack at its own centre under a rule forbidding a
    NEW track there, which says nothing about the copper already present).

    Strictly above every free-space value so own copper is always processed
    first and can never itself be the reported bottleneck.
    """
    return 2.0 * _open_room(view) + 1.0


class ScipyRequired(RuntimeError):
    """Raised when scipy is missing, rather than returning a worse answer."""


def _require_scipy():
    try:
        from scipy import ndimage
    except ImportError as exc:                    # pragma: no cover
        raise ScipyRequired(
            "reachability needs scipy (distance_transform_edt, label). It is "
            "not installed in this interpreter. Install it, or run this from "
            "an interpreter that has it -- do NOT substitute a coarser "
            "distance metric: the answer is used to say a board is "
            "unroutable, and an approximate one is worse than none.") from exc
    return ndimage


# --------------------------------------------------------------------------
# rasterisation
# --------------------------------------------------------------------------

def _capsule(occ, x0, y0, x1, y1, r, view, step):
    """Stamp a thick line segment (track, or via as a zero-length one)."""
    vx0, vy0 = view[0], view[1]
    h, w = occ.shape
    gx0 = max(0, int((min(x0, x1) - r - vx0) / step))
    gx1 = min(w, int((max(x0, x1) + r - vx0) / step) + 2)
    gy0 = max(0, int((min(y0, y1) - r - vy0) / step))
    gy1 = min(h, int((max(y0, y1) + r - vy0) / step) + 2)
    if gx0 >= gx1 or gy0 >= gy1:
        return
    ys, xs = np.mgrid[gy0:gy1, gx0:gx1]
    px = vx0 + (xs + 0.5) * step
    py = vy0 + (ys + 0.5) * step
    dx, dy = x1 - x0, y1 - y0
    L2 = dx * dx + dy * dy
    if L2 < 1e-12:
        d = np.hypot(px - x0, py - y0)
    else:
        t = np.clip(((px - x0) * dx + (py - y0) * dy) / L2, 0.0, 1.0)
        d = np.hypot(px - (x0 + t * dx), py - (y0 + t * dy))
    occ[gy0:gy1, gx0:gx1] |= (d <= r)


def _rect(occ, cx, cy, sx, sy, view, step, rot_deg=0.0):
    """Stamp a pad rectangle, honouring a residual non-orthogonal tilt."""
    vx0, vy0 = view[0], view[1]
    h, w = occ.shape
    reach = math.hypot(sx, sy) / 2.0
    gx0 = max(0, int((cx - reach - vx0) / step))
    gx1 = min(w, int((cx + reach - vx0) / step) + 2)
    gy0 = max(0, int((cy - reach - vy0) / step))
    gy1 = min(h, int((cy + reach - vy0) / step) + 2)
    if gx0 >= gx1 or gy0 >= gy1:
        return
    ys, xs = np.mgrid[gy0:gy1, gx0:gx1]
    px = vx0 + (xs + 0.5) * step - cx
    py = vy0 + (ys + 0.5) * step - cy
    if abs(rot_deg) > 1e-9:
        th = math.radians(-rot_deg)
        px, py = (px * math.cos(th) - py * math.sin(th),
                  px * math.sin(th) + py * math.cos(th))
    occ[gy0:gy1, gx0:gx1] |= ((np.abs(px) <= sx / 2) & (np.abs(py) <= sy / 2))


def _pad_on_layer(pad, layer) -> bool:
    if pad.drill and pad.drill > 0:
        return True                               # a barrel blocks every layer
    layers = pad.layers or []
    return layer in layers or any(l in ('*.Cu',) for l in layers)


def slack_field(pcb, target_net_id, layer, view, base_clearance,
                net_clearances=None, step=0.01):
    """(slack, own, D) for one layer.

    `net_clearances` is {net_id: clearance_mm} -- the per-net map the routing
    config already resolves from netclasses and `.kicad_dru`, so this grades at
    what the board would actually route at rather than at one guessed number.
    Nets absent from it use `base_clearance`.
    """
    ndimage = _require_scipy()
    w = int(math.ceil((view[2] - view[0]) / step))
    h = int(math.ceil((view[3] - view[1]) / step))
    if w <= 0 or h <= 0:
        raise ValueError(f"empty view {view}")
    per_class: Dict[float, np.ndarray] = {}
    own = np.zeros((h, w), dtype=bool)
    nc = dict(net_clearances or {})

    def grid_for(net_id):
        c = round(float(nc.get(net_id, base_clearance)), 4)
        return per_class.setdefault(c, np.zeros((h, w), dtype=bool))

    for s in pcb.segments:
        if s.layer != layer:
            continue
        g = own if s.net_id == target_net_id else grid_for(s.net_id)
        _capsule(g, s.start_x, s.start_y, s.end_x, s.end_y, s.width / 2.0,
                 view, step)
    for v in pcb.vias:                            # a via blocks every layer
        g = own if v.net_id == target_net_id else grid_for(v.net_id)
        _capsule(g, v.x, v.y, v.x, v.y, v.size / 2.0, view, step)
    for fp in pcb.footprints.values():
        for p in fp.pads:
            if getattr(p, 'pad_type', '') == 'np_thru_hole':
                # NPTH has no copper: only its hole matters, and that is a
                # drill constraint, not a clearance one.
                continue
            if not _pad_on_layer(p, layer):
                continue
            g = own if p.net_id == target_net_id else grid_for(p.net_id)
            _rect(g, p.global_x, p.global_y, p.size_x, p.size_y, view, step,
                  getattr(p, 'rect_rotation', 0.0) or 0.0)

    # D(p) = min_k(dist_k(p) - clearance_k): the clearance-adjusted room at p.
    # A track of width t fits iff D >= t/2, so track slack is 2D.
    D = np.full((h, w), np.inf)
    for c, occ in per_class.items():
        if not occ.any():
            continue
        D = np.minimum(D, ndimage.distance_transform_edt(~occ) * step - c)
    D[np.isinf(D)] = _open_room(view)
    D = np.minimum(D, _open_room(view))
    for occ in per_class.values():
        D[occ] = -1.0
    slack = 2.0 * D
    slack[own] = _own_slack(view)
    return slack, own, D


# --------------------------------------------------------------------------
# widest path
# --------------------------------------------------------------------------

def widest_path(slacks, targets, via_ok, seed_xy, seed_layer, view, step,
                return_throat=False):
    """Kruskal widest path over a multi-layer grid; returns mm or None.

    Nodes are (cell, layer). In-layer edges join adjacent ACTIVE cells; a via
    edge joins two layers at one cell and is permitted only where a via of the
    given diameter is legal on BOTH -- a legality that does not depend on the
    threshold, so it composes with Kruskal unchanged.

    Returns the largest t such that a path exists using only cells of slack
    >= t. Exact for the raster: no search order, no grid alignment, no
    heuristic. None means no positive-slack path exists at any width.

    `return_throat=True` returns `(value, (layer, x_mm, y_mm))` instead. The
    cell that CLOSED the path is the throat -- the narrowest point on the widest
    route -- and it was already in hand here (`lay`, `x`, `y` at the moment the
    seed joins the target) and thrown away at the bare `return float(val)`.
    Every consumer that wanted to say WHERE the bottleneck was had to go and
    find it again by eye. `None` throat when there is no path.
    """
    L = len(slacks)
    h, w = slacks[0].shape
    n = h * w
    sx = int((seed_xy[0] - view[0]) / step)
    sy = int((seed_xy[1] - view[1]) / step)
    if not (0 <= sx < w and 0 <= sy < h):
        raise ValueError(f"seed {seed_xy} is outside the view {view}")
    seed = seed_layer * n + sy * w + sx

    flat = np.concatenate([s.reshape(-1) for s in slacks])
    tgt_flat = np.concatenate([t.reshape(-1) for t in targets])
    via_flat = via_ok.reshape(-1)

    order = np.argsort(flat, kind='stable')[::-1]
    TARGET = L * n
    parent = np.arange(L * n + 1, dtype=np.int64)
    active = np.zeros(L * n, dtype=bool)

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb

    for idx in order:
        idx = int(idx)
        val = float(flat[idx])
        if val < 0.0:                             # never route through copper
            break
        active[idx] = True
        lay, cell = divmod(idx, n)
        if tgt_flat[idx]:
            union(idx, TARGET)
        y, x = divmod(cell, w)
        for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            ny, nx = y + dy, x + dx
            if 0 <= ny < h and 0 <= nx < w:
                j = lay * n + ny * w + nx
                if active[j]:
                    union(idx, j)
        if via_flat[cell]:
            for other in range(L):
                if other != lay and active[other * n + cell]:
                    union(idx, other * n + cell)
        if active[seed] and find(seed) == find(TARGET):
            if not return_throat:
                return float(val)
            # `lay`, `y`, `x` are this cell -- the LAST one activated, i.e. the
            # narrowest on the widest path. Cell centre, not corner.
            return float(val), (lay, view[0] + (x + 0.5) * step,
                                view[1] + (y + 0.5) * step)
    return (None, None) if return_throat else None


@dataclass
class Reachability:
    """What the field says about one pad reaching the rest of its net."""
    net: str
    net_id: int
    seed: Tuple[float, float]
    layers: Tuple[str, ...]
    step_mm: float
    track_mm: float
    via_mm: float
    view: Tuple[float, float, float, float]
    bottleneck_mm: Optional[float]        # None = no positive-slack path at all
    target_cells: int
    via_legal_fraction: float
    grid: Tuple[int, int]
    note: str = ''
    open_room_mm: float = 0.0     # the cap free space was clamped to
    # WHERE the bottleneck is: (layer_index, x_mm, y_mm), or None when there is
    # no path. Everything below exists because run 20 measured a throat, printed
    # the number, and then had to hand-assemble the coordinates, the blocking
    # refs and the relief distance into an English paragraph in a ledger string
    # -- three tools' stdout, none of it consumable by anything downstream.
    throat_cell: Optional[Tuple[int, float, float]] = None
    #: The base clearance the field was built at. Needed to convert between the
    #: two SPACES this class reports in -- see `gap_mm`.
    clearance_mm: float = 0.0
    #: Foreign copper nearest the throat, on the throat's OWN layer:
    #: [{ref, pad, kind, net, net_id, clearance, x, y, layer, dist}]. `dist` is
    #: to the object's edge; `clearance` is what converts `bottleneck_mm` into
    #: `gap_mm` for that blocker.
    near: Tuple[dict, ...] = ()

    @property
    def wide_open(self) -> bool:
        """The path never came near any foreign copper inside the view.

        Reported as such rather than as a number, because the number would be
        the view's own size -- a property of what was asked, not of the board.
        """
        return (self.bottleneck_mm is not None
                and self.bottleneck_mm >= 2.0 * self.open_room_mm - 1e-9)

    @property
    def caged(self) -> bool:
        return (self.bottleneck_mm is None
                or self.bottleneck_mm < self.track_mm)

    @property
    def measured(self) -> bool:
        """Did this run ask an answerable question at all?

        `target_cells == 0` means "no other island of this net is inside the
        view" -- the pad is already joined to everything nearby, or the margin
        is too small. That is NOT a verdict, and the CLI has always known it
        (exit 2, never 1). But `bottleneck_mm` is None in that case, so `caged`
        is True, so `to_dict()` stamped `verdict: CAGED` anyway -- and a caller
        reading the documented `--json` payload instead of the exit code got a
        confident CAGED for a question nobody answered. Measured on run 15: 7
        CAGED reported where 3 was the truth, on a classification where a single
        CAGED flips the whole loop into re-entering placement.
        """
        return self.target_cells > 0

    @property
    def margin_um(self) -> Optional[float]:
        if self.bottleneck_mm is None:
            return None
        return 1000.0 * (self.bottleneck_mm - self.track_mm)

    @property
    def binding_clearances(self) -> Tuple[float, float]:
        """The clearance in force at each of the two blockers forming the throat.

        NOT `2 * clearance_mm`. Clearance here is PAIRWISE: `slack_field` builds
        `D = min_k(dist_k - c_k)` from the per-net map the routing config
        resolves, and `check_reachability` always passes one. So the number that
        converts the bottleneck into a physical gap is the clearance of the
        copper actually at the throat, which is only the base clearance when
        every blocker happens to sit on the Default class.

        Measured, on a synthetic 1.00 mm slot at base clearance 0.05: with no
        net clearances the gap came out 1.00 (right); with the two wall nets on
        a 0.3 class it came out 0.50 for the same unchanged geometry -- off by
        `2 * (class - base)`, and in the pessimistic direction that turns a
        passable slot into a reported cage.

        Falls back to `clearance_mm` twice when no blocker is known (no `near`,
        or a caller that passed no clearance map), which is the old behaviour
        with the old assumption now stated out loud. With exactly ONE blocker
        known it duplicates that blocker's clearance -- see `gap_blockers`,
        which publishes the count so nobody reads such a `gap_mm` as an
        edge-to-edge distance between two named objects.
        """
        cs = [n['clearance'] for n in self.near
              if n.get('clearance') is not None][:2]
        if not cs:
            return (self.clearance_mm, self.clearance_mm)
        if len(cs) == 1:
            return (cs[0], cs[0])
        return (cs[0], cs[1])

    @property
    def gap_blockers(self) -> int:
        """How many blockers the gap conversion actually knew about (0, 1 or 2).

        2 is the honest case: `gap_mm` is then the distance between two named
        pieces of copper, each converted by its own clearance. With 1 the far
        side of the throat is ASSUMED to sit at the same clearance as the near
        one, and with 0 both sides are assumed at the base clearance -- the
        number is still the widest room measured at the throat, but calling it
        "the edge-to-edge distance between the two blockers" would name
        objects that were never identified. Published so a consumer can tell
        the three apart instead of trusting a label.
        """
        return min(2, sum(1 for n in self.near
                          if n.get('clearance') is not None))

    @property
    def gap_mm(self) -> Optional[float]:
        """The bottleneck in GAP space -- the physical edge-to-edge distance
        when `gap_blockers == 2`, and twice the room on the known side when it
        is less.

        `bottleneck_mm` is in SLACK space: `slack = 2 * (dist - clearance)`
        (see `slack_field`), which is the widest TRACK that fits. At the throat
        the two blockers are each exactly their own clearance beyond the widest
        track, so with `D = bottleneck / 2`::

            dist_a - c_a = dist_b - c_b = D
            gap = dist_a + dist_b = bottleneck + c_a + c_b

        which is exact when the two classes differ and collapses to the
        familiar `bottleneck + 2 * clearance` when they do not. See
        `binding_clearances` for what went wrong with the fixed `2 * clearance`.

        The two spaces are different numbers for the same throat and mixing them
        is easy: run 20's ledger recorded `0.409/0.450mm` (gap space) beside
        `-37.69um` (track space) in one sentence, as if they described the same
        measurement. Publishing both, each labelled, is why this property
        exists.

        None when there is no path, and None when the result is `wide_open`:
        there the bottleneck is the view's own size, not a distance between
        two pieces of copper, so there is no gap to report.
        """
        if self.bottleneck_mm is None or self.wide_open:
            return None
        return self.bottleneck_mm + sum(self.binding_clearances)

    @property
    def gap_need_mm(self) -> Optional[float]:
        """What the gap would have to BE for `track_mm` to fit through it.

        The partner of `gap_mm`, in the same space and converted by the same
        two clearances -- publishing one without the other leaves a reader to
        re-derive the conversion, and re-deriving it with `2 * clearance_mm` is
        exactly the bug `binding_clearances` fixes. None whenever `gap_mm` is.
        """
        if self.bottleneck_mm is None or self.wide_open:
            return None
        return self.track_mm + sum(self.binding_clearances)

    @property
    def throat(self) -> Optional[dict]:
        """The throat in world coordinates, or None."""
        if not self.throat_cell:
            return None
        lay, x, y = self.throat_cell
        return {'x': round(x, 4), 'y': round(y, 4),
                'layer': self.layers[lay] if lay < len(self.layers)
                else str(lay)}

    def to_dict(self):
        return {'net': self.net, 'seed': list(self.seed),
                'layers': list(self.layers), 'step_mm': self.step_mm,
                'track_mm': self.track_mm, 'via_mm': self.via_mm,
                'bottleneck_mm': (None if self.bottleneck_mm is None
                                  else round(self.bottleneck_mm, 5)),
                'wide_open': self.wide_open,
                # NO-TARGET is a third state, not a shade of CAGED -- see
                # `measured`. It matches the CLI's long-standing exit 2, so the
                # JSON and the exit code finally agree, and `verdict == 'CAGED'`
                # becomes safe to test directly.
                'verdict': ('CAGED' if self.caged else 'PASSABLE')
                           if self.measured else 'NO-TARGET',
                'measured': self.measured,
                'margin_um': (None if self.margin_um is None
                              else round(self.margin_um, 2)),
                'target_cells': self.target_cells,
                'via_legal_fraction': round(self.via_legal_fraction, 4),
                'grid': list(self.grid), 'view': [round(v, 3)
                                                  for v in self.view],
                'note': self.note,
                # ADDITIVE. Every key above is unchanged
                # (tests/test_defect_record_reachability.py pins the set) --
                # consumers of this payload predate the defect record.
                'throat': self.throat,
                'clearance_mm': self.clearance_mm,
                'gap_mm': (None if self.gap_mm is None
                           else round(self.gap_mm, 5)),
                # Beside `gap_mm`, because `gap_mm - gap_need_mm` is no longer
                # derivable from `clearance_mm` alone: the conversion uses the
                # clearance of each blocker at the throat, and each of those is
                # published on its own entry in `near`.
                'gap_need_mm': (None if self.gap_need_mm is None
                                else round(self.gap_need_mm, 5)),
                # How many blockers that conversion knew about. With fewer
                # than 2 the far side is assumed, and `gap_mm` is NOT a
                # distance between two named objects -- see `gap_blockers`.
                'gap_blockers': self.gap_blockers,
                'near': [dict(n) for n in self.near]}

    def defect_record(self, board=None, board_sha=None, relief=(),
                      instrument='check_reachability', floors=None):
        """This measurement as a `defect-record` document.

        Shape (version 1)::

            {kind: 'defect-record', version: 1, count: 1,
             board: <abs path, if given>, board_sha: <sha256, if given>,
             defects: [{kind: 'throat', verdict: 'CAGED', net, seed{x,y,layer},
                        at{x,y,layer}, refs[], pads[],
                        measure{space: 'track_width', have_mm, need_mm,
                                short_mm, resolution_mm, gap_mm, gap_need_mm,
                                gap_blockers, derived_from},
                        span{a, b}  (when two blockers are known; each is
                                     {x, y, layer, kind, net, clearance}),
                        view[], relief[], instrument{source, floors, step_mm,
                                                     search_view}}]}

        `measure` carries both spaces: `have_mm`/`need_mm` are track widths
        (what `bottleneck_mm` and `track_mm` are), `gap_mm`/`gap_need_mm` are
        the physical edge-to-edge distances, converted by the clearance in
        force at EACH blocker (`span.a.clearance` + `span.b.clearance`, see
        `binding_clearances`) rather than by twice the base clearance. `refs`
        and `pads` are the foreign copper nearest the throat, from `near`;
        `instrument`, `relief` and `floors` are whatever the caller passes
        through (the check_reachability CLI fills them). Everything here was
        already computed by `pad_reachability`; this only arranges it.

        Returns None when there is nothing to report: NO-TARGET (no verdict)
        or PASSABLE. The only writer at present is `check_reachability
        --defect-json`, which writes this dict to the path it is given.
        """
        if not self.measured or not self.caged:
            return None
        thr = self.throat
        # The two spaces, both stated, both labelled, with what converts them.
        measure = {'space': 'track_width',
                   'have_mm': (None if self.bottleneck_mm is None
                               else round(self.bottleneck_mm, 5)),
                   'need_mm': self.track_mm,
                   'short_mm': (None if self.bottleneck_mm is None else
                                round(self.track_mm - self.bottleneck_mm, 5)),
                   'resolution_mm': self.step_mm,
                   'gap_mm': (None if self.gap_mm is None
                              else round(self.gap_mm, 5)),
                   'gap_need_mm': (None if self.gap_need_mm is None
                                   else round(self.gap_need_mm, 5)),
                   'gap_blockers': self.gap_blockers,
                   # None, not a sentence: with no blocker found there is no
                   # gap to derive (`gap_mm` is null too). The severest CAGED
                   # -- a seed ringed by copper, `bottleneck_mm` None -- lands
                   # here, and a record that DESCRIBED a derivation nobody
                   # made would be the phantom this instrument exists to stop.
                   'derived_from': (
                       None if self.gap_mm is None else
                       'have_mm + span.a.clearance + span.b.clearance (the '
                       'clearance in force at each blocker forming the '
                       'throat, NOT 2 x the base clearance)'
                       if self.gap_blockers >= 2 else
                       'have_mm + 2 x the clearance of the ONE blocker found '
                       '(gap_blockers < 2: the far side of the throat was not '
                       'identified and is assumed at the same clearance)')}
        refs, pads = [], []
        for n in self.near:
            if n.get('ref') and n['ref'] not in refs:
                refs.append(n['ref'])
            if n.get('pad') and n['pad'] not in pads:
                pads.append(n['pad'])
        d = {'kind': 'throat', 'verdict': 'CAGED', 'net': self.net,
             'seed': {'x': round(self.seed[0], 4), 'y': round(self.seed[1], 4),
                      'layer': self.layers[0] if self.layers else None},
             'at': thr, 'refs': refs, 'pads': pads, 'measure': measure,
             'view': [round(v, 4) for v in self.view],
             'relief': [dict(r) for r in relief],
             'instrument': {'source': instrument, 'floors': dict(floors or {}),
                            'step_mm': self.step_mm,
                            'search_view': [round(v, 4) for v in self.view]}}
        if len(self.near) >= 2:
            _span = ('x', 'y', 'layer', 'kind', 'net', 'clearance')
            d['span'] = {'a': {k: self.near[0].get(k) for k in _span},
                         'b': {k: self.near[1].get(k) for k in _span}}
        doc = {'kind': 'defect-record', 'version': 1, 'count': 1,
               'defects': [d]}
        if board:
            doc['board'] = os.path.abspath(board)
        if board_sha:
            doc['board_sha'] = board_sha
        return doc

    def format_text(self):
        lines = [f"net        {self.net} from ({self.seed[0]}, "
                 f"{self.seed[1]}) on {self.layers[0]}",
                 f"layers     {list(self.layers)}   via {self.via_mm}mm legal "
                 f"at {100.0 * self.via_legal_fraction:.1f}% of cells",
                 f"grid       {self.grid[0]}x{self.grid[1]} @ {self.step_mm}mm"]
        if self.note:
            lines.append(f"note       {self.note}")
        if not self.measured:
            # Same third state the JSON carries, and the same one the CLI has
            # always signalled with exit 2. Printing CAGED here said the
            # opposite of what the exit code said.
            lines.append("BOTTLENECK not measured: nothing of this net to "
                         "reach inside the view")
            lines.append("VERDICT    NO-TARGET -- this is NOT a verdict. Widen "
                         "--margin, or this is not a reachability question.")
        elif self.bottleneck_mm is None:
            lines.append("BOTTLENECK none: no positive-slack path exists at "
                         "any grid")
            lines.append("VERDICT    CAGED for any track width")
        elif self.wide_open:
            lines.append(f"BOTTLENECK >= {2.0 * self.open_room_mm:.2f}mm -- the "
                         f"path never approached foreign copper inside the "
                         f"view, so this is bounded by the VIEW, not by the "
                         f"board. There is no throat here to measure.")
            lines.append(f"VERDICT    PASSABLE at track {self.track_mm}mm "
                         f"(nothing is in the way)")
        else:
            lines.append(f"BOTTLENECK {self.bottleneck_mm:.4f}mm (widest track "
                         f"that reaches the net)")
            lines.append(
                f"VERDICT    {'CAGED' if self.caged else 'PASSABLE'} at track "
                f"{self.track_mm}mm  (margin {self.margin_um:+.1f} um)")
        return '\n'.join(lines)


#: How many blockers `nearest_foreign` reports. TWO, and not a parameter: the
#: record's `span` is a segment between two pieces of copper and the relief
#: answer is a two-body one, so a third entry has nowhere to go. It was a
#: never-varied `k=2` argument, which read as a knob and was not one.
NEAR_K = 2
#: Search radius, in EDGE-to-edge millimetres. Also not a parameter, and this
#: one had teeth: as a never-varied `radius_mm=2.0` measured to object CENTRES
#: it silently dropped every blocker bigger than about 4 mm. A 5 mm-tall wall
#: pad 0.2 mm from the throat has its centre 2.7 mm away, so a CAGED verdict
#: shipped `near: ()`, no refs, no pads, no span and no relief -- the whole
#: point of the record. Measured edge-to-edge the same pad is 0.2 mm away and
#: is named.
NEAR_RADIUS_MM = 2.0


def _point_rect_dist(px, py, cx, cy, sx, sy, rot_deg=0.0):
    """Distance from a point to a pad RECTANGLE's edge; 0 inside.

    A rectangle, including for a round pad, because that is what `_rect`
    stamps into the field the bottleneck was measured from. Two models of the
    same pad would put the naming and the number in different geometries.
    """
    dx, dy = px - cx, py - cy
    if abs(rot_deg) > 1e-9:                       # same convention as `_rect`
        th = math.radians(-rot_deg)
        dx, dy = (dx * math.cos(th) - dy * math.sin(th),
                  dx * math.sin(th) + dy * math.cos(th))
    return math.hypot(max(abs(dx) - sx / 2.0, 0.0),
                      max(abs(dy) - sy / 2.0, 0.0))


def _point_seg_dist(px, py, x0, y0, x1, y1):
    """Distance from a point to a line SEGMENT (its centreline)."""
    dx, dy = x1 - x0, y1 - y0
    L2 = dx * dx + dy * dy
    if L2 < 1e-12:
        return math.hypot(px - x0, py - y0)
    t = max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / L2))
    return math.hypot(px - (x0 + t * dx), py - (y0 + t * dy))


def nearest_foreign(pcb, x, y, net_id, layers=None, net_clearances=None,
                    base_clearance=None):
    """The two nearest pieces of FOREIGN copper to (x, y), nearest first.

    "Which two things form this throat" is the question every reader asks
    immediately after "how wide is it", and answering it took a second tool and
    a hand-assembled sentence. Modelled on `net_forensics`' wall inventory,
    which already does exactly this inventory with `REF.PAD` naming -- this is
    the same walk with a two-nearest cut instead of a printed table.

    Each entry: {kind, ref, pad, net, net_id, clearance, x, y, layer, dist}.

    `dist` is to the object's EDGE -- the pad rectangle, the via barrel, the
    segment capsule -- not to its centre. Centre distances were the first
    version and they were wrong in the one direction that matters: a big
    object beside the throat measured FAR and was dropped, so the record that
    exists to name the blocker named nobody (see `NEAR_RADIUS_MM`).

    `x`/`y` remain the object's CENTRE: that is where the thing is, and a
    reader drawing it wants the object, not the closest point on it.

    `layers` must be the ONE layer the throat is on, not the whole analysis
    stack. Copper on another face cannot bound this throat, and naming it
    produced advice to move a part on the other side of the board. Through-hole
    pads and vias are barrels and are kept whatever the layer.

    `net_clearances` / `base_clearance`, when given, stamp each entry with the
    clearance in force for THAT blocker, which is what converts the bottleneck
    into a physical gap (see `Reachability.gap_mm`). Published per blocker so
    the conversion is auditable instead of assumed.
    """
    lay = set(layers or ())
    nc = dict(net_clearances or {})

    def _clr(nid):
        if base_clearance is None:
            return None
        return float(nc.get(nid, base_clearance))

    out = []
    for fp in pcb.footprints.values():
        for p in fp.pads:
            if p.net_id == net_id:
                continue
            if getattr(p, 'pad_type', '') == 'np_thru_hole':
                continue
            # `_pad_on_layer`, the SAME predicate `slack_field` stamps
            # with. A second, nearly-identical test re-created blocker 3
            # through another door: a pad declared on `*.Cu` is stamped into
            # the field and forms the throat, but `set(p.layers) & lay` does
            # not match it, so a CAGED result shipped near=[] refs=[] again.
            if lay and not any(_pad_on_layer(p, l) for l in lay):
                continue
            d = _point_rect_dist(x, y, p.global_x, p.global_y,
                                 p.size_x, p.size_y,
                                 getattr(p, 'rect_rotation', 0.0) or 0.0)
            if d <= NEAR_RADIUS_MM:
                out.append({'kind': 'pad', 'ref': fp.reference,
                            'pad': f'{fp.reference}.{p.pad_number}',
                            'net': (pcb.nets[p.net_id].name
                                    if p.net_id in pcb.nets else None),
                            'net_id': p.net_id, 'clearance': _clr(p.net_id),
                            'x': round(p.global_x, 4), 'y': round(p.global_y, 4),
                            'layer': (p.layers or [None])[0],
                            'dist': round(d, 5)})
    for v in pcb.vias:
        if v.net_id == net_id:
            continue
        d = max(0.0, math.hypot(v.x - x, v.y - y) - (v.size or 0.0) / 2.0)
        if d <= NEAR_RADIUS_MM:
            out.append({'kind': 'via', 'ref': None, 'pad': None,
                        'net': (pcb.nets[v.net_id].name
                                if v.net_id in pcb.nets else None),
                        'net_id': v.net_id, 'clearance': _clr(v.net_id),
                        'x': round(v.x, 4), 'y': round(v.y, 4),
                        'layer': None, 'dist': round(d, 5)})
    for s in pcb.segments:
        if s.net_id == net_id:
            continue
        if lay and s.layer not in lay:
            continue
        d = max(0.0, _point_seg_dist(x, y, s.start_x, s.start_y,
                                     s.end_x, s.end_y) - (s.width or 0.0) / 2.0)
        if d <= NEAR_RADIUS_MM:
            out.append({'kind': 'segment', 'ref': None, 'pad': None,
                        'net': (pcb.nets[s.net_id].name
                                if s.net_id in pcb.nets else None),
                        'net_id': s.net_id, 'clearance': _clr(s.net_id),
                        'x': round(s.start_x, 4), 'y': round(s.start_y, 4),
                        'layer': s.layer, 'dist': round(d, 5)})
    out.sort(key=lambda e: e['dist'])
    return tuple(out[:NEAR_K])


def pad_reachability(pcb, seed_xy, net_name=None, net_id=None, *,
                     layers=None, view=None, track_mm=0.15, via_mm=0.6,
                     base_clearance=0.15, net_clearances=None, step=0.01,
                     margin_mm=4.0) -> Reachability:
    """Can a track of `track_mm` get from `seed_xy` to the rest of its net?

    `view` defaults to a `margin_mm` box around the seed, because the answer is
    LOCAL: a throat is a property of the copper within a few millimetres, and
    rasterising a whole board at 10um to answer it costs memory for nothing. A
    view that excludes the rest of the net is reported, not silently answered --
    see `note`.
    """
    ndimage = _require_scipy()
    if net_id is None:
        net_id = next((i for i, n in pcb.nets.items() if n.name == net_name),
                      None)
    if net_id is None:
        raise ValueError(f"no net named {net_name!r} on this board")
    name = pcb.nets[net_id].name if net_id in pcb.nets else str(net_id)

    if layers is None:
        layers = tuple(pcb.board_info.copper_layers or ('F.Cu', 'B.Cu'))
    layers = tuple(layers)
    if view is None:
        view = (seed_xy[0] - margin_mm, seed_xy[1] - margin_mm,
                seed_xy[0] + margin_mm, seed_xy[1] + margin_mm)
    view = tuple(float(v) for v in view)

    slacks, owns, Ds = [], [], []
    for layer in layers:
        s, o, D = slack_field(pcb, net_id, layer, view, base_clearance,
                              net_clearances, step)
        slacks.append(s)
        owns.append(o)
        Ds.append(D)
    via_ok = np.ones_like(Ds[0], dtype=bool)
    for D in Ds:
        via_ok &= (D >= via_mm / 2.0)

    # The seed pad IS own-net copper, so "reach the net" would be trivially
    # true unless the pad's island is separated from the rest of the net.
    # Label per layer, then stitch labels through the net's own vias.
    labs, counts = [], []
    for o in owns:
        lab, k = ndimage.label(o)
        labs.append(lab)
        counts.append(k)
    off, gid = [0], 0
    for k in counts:
        gid += k
        off.append(gid)
    par = list(range(gid + 1))

    def find(i):
        while par[i] != i:
            par[i] = par[par[i]]
            i = par[i]
        return i

    def union(i, j):
        ri, rj = find(i), find(j)
        if ri != rj:
            par[ri] = rj

    for v in pcb.vias:
        if v.net_id != net_id:
            continue
        cx = int((v.x - view[0]) / step)
        cy = int((v.y - view[1]) / step)
        if not (0 <= cx < labs[0].shape[1] and 0 <= cy < labs[0].shape[0]):
            continue
        ids = [off[i] + int(labs[i][cy, cx]) for i in range(len(layers))
               if labs[i][cy, cx]]
        for k in range(1, len(ids)):
            union(ids[0], ids[k])

    sx = int((seed_xy[0] - view[0]) / step)
    sy = int((seed_xy[1] - view[1]) / step)
    if not (0 <= sx < labs[0].shape[1] and 0 <= sy < labs[0].shape[0]):
        raise ValueError(f"seed {seed_xy} is outside the view {view}")
    seed_lab = int(labs[0][sy, sx])
    if seed_lab == 0:
        raise ValueError(
            f"({seed_xy[0]}, {seed_xy[1]}) is not on {name}'s copper on "
            f"{layers[0]} -- pass the pad centre, and check the layer")
    seed_root = find(off[0] + seed_lab)

    targets = []
    for i, o in enumerate(owns):
        t = np.zeros_like(o)
        for lab in range(1, counts[i] + 1):
            if find(off[i] + lab) != seed_root:
                t |= (labs[i] == lab)
        targets.append(t)
    n_t = sum(int(t.sum()) for t in targets)

    note = ''
    if n_t == 0:
        note = ("no other island of this net is inside the view -- the pad is "
                "already joined to everything nearby. Widen --margin, or this "
                "is not a reachability question")
    b, throat = ((None, None) if n_t == 0
                 else widest_path(slacks, targets, via_ok, seed_xy, 0, view,
                                  step, return_throat=True))
    r = Reachability(
        net=name, net_id=net_id, seed=(seed_xy[0], seed_xy[1]), layers=layers,
        step_mm=step, track_mm=track_mm, via_mm=via_mm, view=view,
        bottleneck_mm=b, target_cells=n_t,
        via_legal_fraction=float(via_ok.mean()),
        grid=(slacks[0].shape[1], slacks[0].shape[0]), note=note,
        open_room_mm=_open_room(view),
        throat_cell=throat, clearance_mm=base_clearance)
    if r.wide_open:
        # The path never came near foreign copper, so the cell that closed it
        # is just the last free cell Kruskal happened to activate -- a point in
        # open board, not a throat. Reporting it (and the copper "nearest" to
        # it) would put a location and a 34 mm "gap" on a measurement that has
        # neither; `wide_open` is the whole answer.
        r.throat_cell = None
    elif throat:
        # The throat is on ONE layer. Handing `nearest_foreign` the whole
        # analysis stack let it name copper on the other face -- probed: a
        # B.Cu throat whose `near[0]` was an F.Cu SMD pad that cannot bound
        # it, which `check_reachability._relief_for` then turned into advice
        # to move a part on the other side of the board.
        r.near = nearest_foreign(pcb, throat[1], throat[2], net_id,
                                 (layers[throat[0]],),
                                 net_clearances=net_clearances,
                                 base_clearance=base_clearance)
    return r
