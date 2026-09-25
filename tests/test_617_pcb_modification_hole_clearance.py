#!/usr/bin/env python3
"""Issue #617, engine 2/3: pcb_modification's graze REPAIR must read
min_hole_clearance -- and its bridge/re-bend gates must NOT.

`check_drc` grades copper-to-hole at the board's own `min_hole_clearance`
(check_drc.py:2390) and `obstacle_map.resolve_hole_clearance` (#616) exposes it
to the engines, while every NPTH term in this module was priced at a flat
`max(clearance, NPTH_TO_TRACK_CLEARANCE)` = 0.20 mm. Five sites were candidates
for the fix; #617 changes TWO of them, and this test pins BOTH halves of that
decision, because the difference is measured, not stylistic:

  CHANGED -- passes that MOVE copper by a measured shortfall. Raising the floor
  makes them SEE a real violation they used to miss, and the move is validated
  against all foreign copper before it lands:
      _seg_worst_offender        the shortfall ranking that drives the shift
      nudge_grazing_microshift   the graze DETECTOR + the shift acceptance gate

  CHANGED LATER (#1038) -- a pass that CHOOSES where new copper goes:
      smooth_octolinear_chains   a shortcut REPLACING a staircase the router
                                 already laid clear; refusing it keeps the
                                 original copper, so nothing is abandoned.
                                 At the flat floor it collapsed spans into the
                                 declared band (run 32: J5's NPTH, 0.201 and
                                 0.212 mm against an announced 0.25).

  DELIBERATELY UNCHANGED -- passes whose only alternative to their single
  candidate is doing nothing. Raising the floor there does not move copper
  somewhere legal, it abandons the repair:
      close_soft_joints          bridges two dangling ends whose caps ALREADY
                                 overlap, so the flanking copper is already in
                                 violation (measured below: the refusal buys
                                 1.5um on a 28.5um violation)
      _connector_clear           same shape -- a stub snap spans at most
                                 1.5 track widths of EXISTING copper
      nudge_grazing_octolinear   all-or-nothing re-bend: refusing it leaves the
                                 net-to-net copper OVERLAP it was repairing

The three "unchanged" cases are kept as CHANGE DETECTORS with their measured
numbers, so a later pass at "finishing the job" has to re-measure rather than
rediscover this the hard way.

Every case is built on a board whose sibling project DECLARES
`min_hole_clearance: 0.25`, with copper at exactly 0.22 mm from the NPTH hole
wall -- legal at the flat 0.20 fab floor, a 0.03 mm violation of what the board
actually asks for. Each changed case is paired with the SAME geometry on a
board declaring nothing, pinning that the change is RAISE-ONLY.

    python3 tests/test_617_pcb_modification_hole_clearance.py
"""
import json
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_router'))  # #522
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_tools'))  # #522
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import obstacle_map
from kicad_parser import BoardInfo
from routing_config import GridRouteConfig
from single_ended_routing import (_seg_foreign_hole_dist, _seg_foreign_pad_dist,
                                  _seg_foreign_seg_dist, _seg_foreign_via_dist)
from pcb_modification import (_connector_clear, _seg_worst_offender,
                              close_soft_joints, nudge_grazing_microshift,
                              nudge_grazing_octolinear,
                              smooth_octolinear_chains)
from routing_defaults import NPTH_TO_TRACK_CLEARANCE
from synth import make_pad, make_pcb, make_seg

HOLE_X, HOLE_Y = 10.0, 10.0
DRILL = 1.0                 # -> hole wall at radius 0.5
W = 0.2                     # track width -> half 0.1
DECLARED = 0.25
BAND = 0.22                 # copper-to-hole-WALL gap under test
Y = HOLE_Y + DRILL / 2.0 + BAND + W / 2.0       # 10.82: centreline of the copper
CLEARANCE = 0.15            # routing clearance, below both floors
# A foreign-net pad above the copper under test: near enough that a shift
# toward it is measurable, far enough that a legal shift exists.
FOREIGN_Y = 11.30
FOREIGN_SIZE = 0.30


def _board_dir(tmp, name, **rules):
    """A board file whose sibling project declares `rules` (KiCad writes
    min_hole_clearance into board.design_settings.rules)."""
    d = os.path.join(tmp, name)
    os.makedirs(d, exist_ok=True)
    pcb = os.path.join(d, 'b.kicad_pcb')
    with open(pcb, 'w', encoding='utf-8') as f:
        f.write('(kicad_pcb (version 20240108))\n')
    with open(os.path.join(d, 'b.kicad_pro'), 'w', encoding='utf-8') as f:
        json.dump({'board': {'design_settings': {'rules': rules}}}, f)
    return pcb


def _npth(x, y, drill, layers=('*.Cu', '*.Mask')):
    return make_pad(net_id=0, x=x, y=y, ref='BUS1', num='H1', size_x=drill,
                    size_y=drill, shape='circle', layers=list(layers),
                    drill=drill, pad_type='np_thru_hole')


def _pcb(path, segs=(), pads=None, bounds=(0.0, 0.0, 20.0, 20.0)):
    """A board rooted at `path`, so the engines resolve the declared floor
    through PCBData.source_path (the #498 mechanism resolve_hole_clearance
    uses)."""
    bi = BoardInfo(layers={}, copper_layers=['F.Cu', 'B.Cu'],
                   board_bounds=bounds)
    return make_pcb(board_info=bi, segments=list(segs),
                    pads_by_net=(pads if pads is not None
                                 else {0: [_npth(HOLE_X, HOLE_Y, DRILL)]}),
                    source_path=path, zones=[])


def _hole_gaps(pcb, net_id=1):
    return [_seg_foreign_hole_dist(pcb, s.net_id, s.start_x, s.start_y,
                                   s.end_x, s.end_y) - s.width / 2.0
            for s in pcb.segments if s.net_id == net_id]


def _copper_gaps(pcb, net_id=1):
    """Closest approach to FOREIGN COPPER (pads, tracks, vias) -- the defect
    class these passes must never make worse."""
    out = []
    for s in pcb.segments:
        if s.net_id != net_id:
            continue
        a = (s.start_x, s.start_y, s.end_x, s.end_y)
        out.append(min(
            _seg_foreign_pad_dist(pcb, s.net_id, *a, s.layer),
            _seg_foreign_seg_dist(pcb, s.net_id, *a, s.layer),
            _seg_foreign_via_dist(pcb, s.net_id, *a, s.layer)) - s.width / 2.0)
    return out


def run():
    fails = []

    def check(name, cond):
        print(('PASS' if cond else 'FAIL') + f': {name}')
        if not cond:
            fails.append(name)

    with tempfile.TemporaryDirectory() as tmp:
        declares = _board_dir(tmp, 'declares', min_hole_clearance=DECLARED)
        silent = _board_dir(tmp, 'silent')
        obstacle_map._HOLE_CLR_CACHE.clear()

        print(f'board declares min_hole_clearance = {DECLARED}mm; the copper '
              f'under test sits {BAND}mm off the hole wall')
        print(f'  hole ({HOLE_X}, {HOLE_Y}) drill {DRILL}; track width {W} '
              f'centred at y={Y:.4g}')
        print()

        _worst_offender(check, declares, silent)
        _microshift(check, declares, silent)
        _microshift_trade(check, declares, silent)
        _smoother(check, declares, silent)
        _soft_joint_stays_flat(check, declares)
        _connector_stays_flat(check, declares)
        _octolinear_stays_flat(check, declares)

    print()
    if fails:
        print(f'{len(fails)} FAILURE(S): {fails}')
        return 1
    print('all checks passed')
    return 0


# === CHANGED site 1: _seg_worst_offender -- the shortfall ranking ===========
def _worst_offender(check, declares, silent):
    print('CHANGED site 1: _seg_worst_offender -- the graze shortfall ranking')
    out = {}
    for path, label in ((declares, 'declared 0.25'), (silent, 'nothing declared')):
        s = make_seg(8.0, Y, 12.0, Y, width=W, net_id=1)
        out[label] = _seg_worst_offender(_pcb(path, [s]), 1, s, CLEARANCE)
        got = out[label]
        print(f'        {label}: '
              + (f'shortfall {got[0]:.4f}mm' if got else 'no offender'))
    dec = out['declared 0.25']
    check('the declared floor makes the 0.22 approach a ranked offender',
          dec is not None and abs(dec[0] - (DECLARED - BAND)) < 1e-3)
    check('nothing declared -> still no offender (raise-only)',
          out['nothing declared'] is None)

    # #617/F5: the helper's documented `config.hole_clearance` override now
    # reaches this site instead of being dropped on the floor.
    cfg = GridRouteConfig()
    cfg.hole_clearance = 0.40
    s = make_seg(8.0, Y, 12.0, Y, width=W, net_id=1)
    try:
        got = _seg_worst_offender(_pcb(silent, [s]), 1, s, CLEARANCE, config=cfg)
        shown = f'shortfall {got[0]:.4f}mm' if got else 'no offender'
    except TypeError as exc:            # no `config` parameter at all (base)
        got, shown = None, f'TypeError: {exc}'
    print(f'        explicit config.hole_clearance=0.40 on the SILENT board: '
          f'{shown}')
    check('an explicit config.hole_clearance reaches this site',
          got is not None and abs(got[0] - (0.40 - BAND)) < 1e-3)
    print()


# === CHANGED site 2: nudge_grazing_microshift -- detector + acceptance ======
def _microshift(check, declares, silent):
    """The pass that MOVES copper. It must (a) see the declared-band graze,
    (b) actually clear it, and (c) not make the foreign-COPPER clearance it
    was already honouring any worse -- the paired assertion that separates a
    real repair from a refusal dressed up as one."""
    print('CHANGED site 2: nudge_grazing_microshift -- detector + shift '
          'acceptance')
    pads = {0: [_npth(HOLE_X, HOLE_Y, DRILL)],
            2: [make_pad(net_id=2, x=10.0, y=FOREIGN_Y, ref='R1', num='1',
                         size_x=FOREIGN_SIZE, size_y=FOREIGN_SIZE,
                         shape='rect', layers=['F.Cu'], pad_type='smd')]}
    for path, label, want in ((declares, 'declared 0.25', True),
                              (silent, 'nothing declared', False)):
        s = make_seg(8.0, Y, 12.0, Y, width=W, net_id=1)
        pcb = _pcb(path, [s], pads)
        hole0, cop0 = min(_hole_gaps(pcb)), min(_copper_gaps(pcb))
        res = [{'new_segments': [s], 'new_vias': []}]
        changed, nets, _rm, _add = nudge_grazing_microshift(
            res, pcb, {1}, clearance=CLEARANCE, max_shift=0.06)
        hole1, cop1 = min(_hole_gaps(pcb)), min(_copper_gaps(pcb))
        print(f'        {label}: segs_changed={changed} nets={nets}; '
              f'copper-to-hole {hole0:.4f} -> {hole1:.4f}; '
              f'foreign-copper {cop0:.4f} -> {cop1:.4f}')
        check(f'micro-shift {"fires" if want else "stays inert"} when {label}',
              (nets == 1) is want)
        if want:
            check('the shifted copper now clears the DECLARED floor',
                  hole1 >= DECLARED - 1e-4)
            check('and the foreign-COPPER clearance it was already honouring '
                  'is still honoured (no repair traded away)',
                  cop1 >= CLEARANCE - 1e-4)
        else:
            check('untouched copper keeps its exact geometry',
                  abs(hole1 - BAND) < 1e-9 and abs(cop1 - cop0) < 1e-12)
    print()


# === CHANGED site 2b: the acceptance-gate TRADE, pinned as policy ==========
def _microshift_trade(check, declares, silent):
    """The raised floor also sits in the candidate-acceptance clears(), so a
    copper-graze repair whose only escape direction points at a hole is
    REFUSED on a declaring board when every candidate would land inside the
    declared band -- the graze stays. That is deliberate (since #616,
    check_drc counts the declared band as a real violation, so 'fixing' the
    graze would manufacture a counted DRC hit), but it is a TRADE, and this
    pins BOTH arms so it can never be mistaken for a free win. Fixture: a
    net-1 track grazing a foreign pad from below, with a mask-only NPTH just
    beneath the track, so the only clearing shift direction is toward the
    hole."""
    print('CHANGED site 2b: acceptance gate -- the declaring-board trade is '
          'deliberate and pinned')
    yh = 0.86
    pads = {2: [make_pad(net_id=2, x=2.0, y=0.34, ref='R1', num='1',
                         size_x=0.3, size_y=0.3, shape='rect',
                         layers=['F.Cu'], pad_type='smd')],
            0: [_npth(2.0, -yh, 1.0, layers=('F.Mask', 'B.Mask'))]}
    for path, label, repaired in ((declares, 'declared 0.25', False),
                                  (silent, 'nothing declared', True)):
        s = make_seg(0.0, 0.0, 4.0, 0.0, width=0.2, net_id=1)
        pcb = _pcb(path, [s], pads, bounds=(-5.0, -5.0, 15.0, 15.0))
        cop0, hole0 = min(_copper_gaps(pcb)), min(_hole_gaps(pcb))
        changed, nets, _rm, _add = nudge_grazing_microshift(
            [], pcb, clearance=0.1)
        cop1, hole1 = min(_copper_gaps(pcb)), min(_hole_gaps(pcb))
        print(f'        {label}: segs_changed={changed} nets={nets}; '
              f'foreign-copper {cop0:+.4f} -> {cop1:+.4f}; '
              f'copper-to-hole {hole0:.4f} -> {hole1:.4f}')
        if repaired:
            check('silent board: the copper-graze repair PROCEEDS (the flat '
                  'floor permits the shift toward the hole)',
                  nets == 1 and cop1 > cop0)
            check('and stays legal at the flat NPTH floor',
                  hole1 >= NPTH_TO_TRACK_CLEARANCE - 1e-4)
        else:
            check('declaring board: the same repair is REFUSED rather than '
                  'moved into the declared band (the deliberate trade)',
                  nets == 0 and abs(cop1 - cop0) < 1e-12)
            check('and the incumbent copper-to-hole clearance is untouched',
                  abs(hole1 - hole0) < 1e-12)
    print()


# === CHANGED site 3 (#1038): smooth_octolinear_chains ======================
def _smoother(check, declares, silent):
    """A staircase whose jog keeps the copper 0.32 mm off an NPTH hole; the
    direct shortcut would run 0.22 mm off it -- legal at the flat 0.20 floor,
    inside the declared 0.25. On the declaring board the smoother must not
    lay that shortcut (any connector it does lay clears 0.25); on the silent
    board it still does (raise-only)."""
    print('CHANGED site 3 (#1038): smooth_octolinear_chains -- shortcut '
          'priced at the declared floor')
    hx, hy = 1.5, -(DRILL / 2.0 + BAND + W / 2.0)      # direct line 0.22 off
    for path, label, want_band in ((declares, 'declared 0.25', False),
                                   (silent, 'nothing declared', True)):
        segs = [make_seg(0.0, 0.0, 1.0, 0.0, width=W, net_id=1),
                make_seg(1.0, 0.0, 1.0, 0.1, width=W, net_id=1),
                make_seg(1.0, 0.1, 2.0, 0.1, width=W, net_id=1),
                make_seg(2.0, 0.1, 2.0, 0.0, width=W, net_id=1),
                make_seg(2.0, 0.0, 4.0, 0.0, width=W, net_id=1)]
        pads = {0: [_npth(hx, hy, DRILL, layers=('F.Mask', 'B.Mask'))],
                1: [make_pad(net_id=1, x=0.0, y=0.0, ref='U1', num='1',
                             size_x=0.3, size_y=0.3),
                    make_pad(net_id=1, x=4.0, y=0.0, ref='U2', num='1',
                             size_x=0.3, size_y=0.3)]}
        pcb = _pcb(path, segs, pads, bounds=(-5.0, -5.0, 15.0, 15.0))
        hole0 = min(_hole_gaps(pcb))
        _n, nets, _strip, _added, st = smooth_octolinear_chains(
            [], pcb, clearance=CLEARANCE)
        hole1 = min(_hole_gaps(pcb))
        print(f'        {label}: nets smoothed={nets} spans={st.get("spans")}; '
              f'copper-to-hole {hole0:.4f} -> {hole1:.4f}')
        if want_band:
            check('silent board: the smoother still lays the direct shortcut '
                  'into the 0.20-0.25 band (raise-only)',
                  nets == 1 and abs(hole1 - BAND) < 1e-3)
        else:
            check('declaring board: no smoothed copper enters the declared '
                  'band', hole1 >= DECLARED - 1e-4)
    print()


# === UNCHANGED site 1: close_soft_joints ===================================
def _soft_joint_stays_flat(check, declares):
    """Change detector. The two dangling ends' caps already overlap, so the
    bridge sits inside copper that is ALREADY in violation: gating it on the
    declared floor drops a `segment-endpoint-gap` repair and leaves the
    violation exactly where it was."""
    print('UNCHANGED site 1: close_soft_joints -- bridge still at the flat '
          'fab floor')
    segs = [make_seg(8.0, Y, 9.9, Y, width=W, net_id=1),
            make_seg(10.05, Y, 12.0, Y, width=W, net_id=1)]
    pcb = _pcb(declares, segs)
    before = min(_hole_gaps(pcb))
    cfg = GridRouteConfig()
    cfg.clearance = CLEARANCE
    n = close_soft_joints([], pcb, {1}, cfg)
    after = min(_hole_gaps(pcb))
    print(f'        declared 0.25: bridges={n}; min copper-to-hole '
          f'{before:.4f} -> {after:.4f}')
    check('the bridge is still laid on a DECLARING board (the repair is kept)',
          n == 1)
    check('and refusing it would not have helped: the flanking copper already '
          f'violates {DECLARED} at {before:.4f}mm',
          before < DECLARED - 1e-9)
    check('the bridge itself costs only what the existing copper already cost '
          '(<0.01mm)', (before - after) < 0.01)
    _bridge_sweep(check)
    print()


def _bridge_sweep(check):
    """The fixture above is one geometry; this is the whole family, exactly.

    Two collinear track pieces on y=0 whose facing ends are `gap` apart, width
    w, and an NPTH of diameter d at (hx, hy); the bridge spans the gap. Sweep
    w, d, gap (0.25x..1.5x w, covering both the soft-joint and stub-snap
    limits) and the hole position on a 5um lattice, and count how often
    refusing the bridge would actually REMOVE the violation (pre >= FLOOR >
    post) rather than merely drop the repair."""
    import numpy as np

    def gap_to(hx, hy, x1, x2, w, d):
        return np.hypot(hx - np.minimum(np.maximum(hx, x1), x2), hy) \
            - w / 2.0 - d / 2.0

    tot = refuse = sole = 0
    for w in (0.10, 0.15, 0.20, 0.25, 0.30):
        for d in (0.6, 0.8, 1.0, 1.5, 2.0):
            for gmul in (0.25, 0.5, 0.75, 1.0, 1.25, 1.5):
                g = gmul * w
                hx = np.arange(-1.0, 1.0001, 0.005)[:, None]
                hy = np.arange(0.0, 1.5001, 0.005)[None, :]
                pre = np.minimum(gap_to(hx, hy, -2.0, -g / 2, w, d),
                                 gap_to(hx, hy, g / 2, 2.0, w, d))
                post = np.minimum(pre, gap_to(hx, hy, -g / 2, g / 2, w, d))
                live = post >= -w          # bridge inside the hole = nonsense
                hit = live & (post < DECLARED)
                tot += int(live.sum())
                refuse += int(hit.sum())
                sole += int((hit & (pre >= DECLARED)).sum())
    pct = 100.0 * (refuse - sole) / refuse
    print(f'        sweep: {tot:,} bridge geometries at a declared {DECLARED}; '
          f'a raised gate would refuse {refuse:,} ({100.0 * refuse / tot:.2f}%)'
          f' and in {pct:.4f}% of those the violation is present ANYWAY')
    check('over the whole bridge family, >99% of the refusals a raised gate '
          'would add remove no violation at all', pct > 99.0)


# === UNCHANGED site 2: _connector_clear ====================================
def _connector_stays_flat(check, declares):
    """Change detector. `snap_stub_gaps` closes a dangling end with a
    connector spanning at most 1.5 track widths of EXISTING copper -- same
    shape as the soft joint, same conclusion."""
    print('UNCHANGED site 2: _connector_clear -- stub snap still at the flat '
          'fab floor')
    ok = _connector_clear(8.0, Y, 12.0, Y, W, 'F.Cu', 1, _pcb(declares),
                          CLEARANCE)
    print(f'        declared 0.25: connector {BAND}mm off the wall '
          f'accepted={ok}')
    check('the stub-snap connector is still accepted on a DECLARING board',
          ok)
    print()


# === UNCHANGED site 3: nudge_grazing_octolinear ============================
def _octolinear_stays_flat(check, declares):
    """Change detector, and the sharpest of the three. A net-1 jog
    A(0,0) -> apex(1,0.5) -> B(2,0) whose apex OVERLAPS a foreign pad by
    0.1 mm. The only clearing octolinear bend is the direct A-B line, which
    runs 0.22 mm off a mask-only NPTH at (1, -0.62). Gating the re-bend on a
    declared 0.25 does not route around the hole -- it leaves the net-to-net
    copper overlap in place."""
    print('UNCHANGED site 3: nudge_grazing_octolinear -- re-bend still at the '
          'flat fab floor')
    segs = [make_seg(0.0, 0.0, 1.0, 0.5, width=W, net_id=1),
            make_seg(1.0, 0.5, 2.0, 0.0, width=W, net_id=1)]
    pads = {
        2: [make_pad(net_id=2, x=1.0, y=0.6, ref='R1', num='1',
                     size_x=0.3, size_y=0.3, shape='rect',
                     layers=['F.Cu'], pad_type='smd')],
        # mask-only NPTH: no phantom copper, so ONLY the hole floor could stop
        # the re-bend (mirrors the #370 B2 fixture).
        0: [_npth(1.0, -0.62, 0.6, layers=('F.Mask', 'B.Mask'))],
    }
    pcb = _pcb(declares, segs, pads, bounds=(-5.0, -5.0, 15.0, 15.0))
    pad0, hole0 = min(_copper_gaps(pcb)), min(_hole_gaps(pcb))
    _changed, nets, _rm, _add = nudge_grazing_octolinear([], pcb, clearance=0.1)
    pad1, hole1 = min(_copper_gaps(pcb)), min(_hole_gaps(pcb))
    print(f'        declared 0.25: nets re-bent={nets}; foreign-pad '
          f'{pad0:+.4f} -> {pad1:+.4f} (negative = net-to-net OVERLAP); '
          f'copper-to-hole {hole0:.4f} -> {hole1:.4f}')
    check('the re-bend still fires on a DECLARING board', nets == 1)
    check('and it repairs a real net-to-net copper OVERLAP',
          pad0 < 0.0 and pad1 > 0.0)
    check('which is a bigger defect than the hole shortfall it leaves '
          f'({-pad0:.4f}mm short vs {DECLARED - hole1:.4f}mm)',
          -pad0 > (DECLARED - hole1))


if __name__ == '__main__':
    sys.exit(run())
