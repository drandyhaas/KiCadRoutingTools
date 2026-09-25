#!/usr/bin/env python3
"""#1033: power-net copper shipped below --power-nets-widths is DISCLOSED.

Run 32's bulk route asked +3V3 for 0.3 mm and shipped about a third of its
length at the 0.127 signal width; nothing in JSON_SUMMARY said so, because
the two sites that narrow a routed power connection -- the long-trunk
neck-down and the short-edge width ladder (#180) -- recorded no narrowing,
and no key measured the SHIPPED board per net.

What must hold:
  * routing_common.power_width_report measures length at / under the
    requested width per net, on the copper it is given;
  * _assign_wide_route_widths records a `design_rules` narrowing for a
    short-edge uniform width and for a long-trunk neck-down, and nothing for
    a route that shipped at full width;
  * route_summary.merge_summaries carries `power_widths` from the outermost
    summary through a reconciliation merge;
  * board_score --net-min-widths reports length_under_mm;
  * END TO END: route.py on a board whose +3V3 must pass a 0.4 mm gap in a
    pad fence writes `power_widths` into --json-out, its under_mm agrees with
    the written board measured independently, and every under-width run is
    also in `design_rules.narrowed`.

    python3 tests/test_1033_power_width_disclosure.py
"""
import json
import math
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, HERE)

from run_utils import evidence  # noqa: E402
from synth import make_seg  # noqa: E402

fails = []


def check(name, cond, detail=''):
    print(('PASS: ' if cond else 'FAIL: ') + name
          + (f'  {detail}' if detail else ''))
    if not cond:
        fails.append(name)


# --------------------------------------------------------------- the report
def t_report():
    from routing_common import power_width_report
    segs = [make_seg(0, 0, 10, 0, width=0.3, net_id=1),     # 10 mm at 0.3
            make_seg(10, 0, 14, 0, width=0.127, net_id=1),  # 4 mm at 0.127
            make_seg(14, 0, 15, 0, width=0.2999, net_id=1),  # rounding, not narrow
            make_seg(0, 5, 3, 5, width=0.2, net_id=2)]      # another net
    r = power_width_report(segs, {1: 0.3, 3: 0.4},
                           lambda n: {1: '+3V3', 3: '+5V'}[n])
    a = r['+3V3']
    check('report: length and under-length per net',
          abs(a['length_mm'] - 15.0) < 1e-6 and abs(a['under_mm'] - 4.0) < 1e-6,
          a)
    check('report: share, min and the width profile',
          abs(a['under_share'] - 4.0 / 15.0) < 1e-3 and a['min_mm'] == 0.127
          and a['by_width_mm'] == {'0.127': 4.0, '0.2999': 1.0, '0.3': 10.0},
          a)
    check('report: a requested net with no copper is listed, not dropped',
          r['+5V']['length_mm'] == 0 and r['+5V']['min_mm'] is None, r['+5V'])
    check('report: nets without a request are not reported', len(r) == 2)


# ------------------------------------------ the width-assignment disclosure
def _cfg():
    from routing_config import GridRouteConfig
    cfg = GridRouteConfig(layers=['F.Cu'], track_width=0.127, clearance=0.1,
                          grid_step=0.1)
    cfg.power_net_widths = {1: 0.3}
    return cfg


def t_assign():
    import fab_tiers
    from single_ended_routing import _assign_wide_route_widths
    from routing_config import GridCoord
    cfg = _cfg()
    coord = GridCoord(cfg.grid_step)

    def ledger():
        return [r for r in fab_tiers.escalation_summary()['narrowed']
                if r['net'] == 1]

    from obstacle_map import GridObstacleMap

    def length_at(out, pred):
        return sum(math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)
                   for s in out if pred(s.width))

    # A 9 mm short edge whose route only fit at 0.15 overall, in FREE space
    # away from its pads: the pad necks (neckdown_length each end) stay at
    # the edge's width, the middle goes back to the requested 0.3 (#1033
    # honour), and the ledger records the narrowing that remains.
    n0 = len(ledger())
    segs = [make_seg(0, 0, 4.5, 0, width=0.3, net_id=1),
            make_seg(4.5, 0, 9, 0, width=0.3, net_id=1)]
    out = _assign_wide_route_widths(segs, cfg, 1, GridObstacleMap(1), coord,
                                    ['F.Cu'], 0, False, 0.15, neck_start=True)
    new = ledger()[n0:]
    check('short edge: the pad ends keep the edge width',
          abs(out[0].width - 0.15) < 1e-9 and abs(out[-1].width - 0.15) < 1e-9,
          [round(s.width, 4) for s in out])
    check('short edge: the middle is widened back to the requested 0.3',
          length_at(out, lambda w: abs(w - 0.3) < 1e-9) > 2.0,
          [round(s.width, 4) for s in out])
    check('short edge: no copper below the edge width, length conserved',
          min(s.width for s in out) >= 0.15 - 1e-9
          and abs(length_at(out, lambda w: True) - 9.0) < 1e-6)
    check('short edge: NO per-attempt ledger row (the ledger records the '
          'SHIPPED copper after the post-route pass)', new == [], new)

    # The same edge with its whole length blocked at the wide margin: the
    # widen-back must refuse every piece.
    blocked = GridObstacleMap(1)
    for gx in range(-5, 100):
        for gy in (-2, -1, 0, 1, 2):
            blocked.add_blocked_cell(gx, gy, 0)
    segs = [make_seg(0, 0, 4.5, 0, width=0.3, net_id=1),
            make_seg(4.5, 0, 9, 0, width=0.3, net_id=1)]
    out = _assign_wide_route_widths(segs, cfg, 1, blocked, coord, ['F.Cu'],
                                    1.0, False, 0.15, neck_start=True)
    check('short edge, blocked everywhere: every piece stays at the edge width',
          all(abs(s.width - 0.15) < 1e-9 for s in out),
          [round(s.width, 4) for s in out])

    # Verifier finding: piece boundaries must be what the fit check tests.
    # _segment_fits_wide rounds endpoints to cells, so an off-grid boundary
    # was tested up to 0.49 cells from where the piece ships.
    from single_ended_routing import _widen_fitting_pieces, _segment_fits_wide
    mp = GridObstacleMap(1)
    for gx in range(40, 46):              # a pinch mid-way along a diagonal
        for gy in range(40, 46):
            mp.add_blocked_cell(gx, gy, 0)

    def real_fits(s, guard=0.0):
        return _segment_fits_wide(s, mp, coord, 0, 1.0 + guard)

    diag = make_seg(0.0, 0.0, 8.0, 8.0, width=0.3, net_id=1)   # grid to grid
    pieces = _widen_fitting_pieces(diag, real_fits, 0.127, coord)

    def on_grid(x, y):
        gx, gy = coord.to_grid(x, y)
        fx, fy = coord.to_float(gx, gy)
        return abs(fx - x) < 1e-9 and abs(fy - y) < 1e-9
    check('grid snap: an on-grid diagonal is cut at GRID POINTS only',
          len(pieces) >= 3 and all(on_grid(p.start_x, p.start_y)
                                   and on_grid(p.end_x, p.end_y)
                                   for p in pieces),
          [(round(p.start_x, 4), round(p.start_y, 4), p.width) for p in pieces])
    check('grid snap: it is both widened and necked (the pinch is local)',
          any(p.width == 0.3 for p in pieces)
          and any(p.width == 0.127 for p in pieces))
    check('grid snap: every kept-wide piece passes the fit AS SHIPPED',
          all(real_fits(p) for p in pieces if p.width == 0.3))

    import single_ended_routing as ser
    G = ser._OFFGRID_FIT_GUARD
    check('the off-grid guard covers a diagonal rounding (>= sqrt(0.5) cell)',
          G >= math.sqrt(0.5), G)

    def guards_for(seg):
        seen = []

        def rec_fits(s, guard=0.0):
            seen.append(guard)
            return False if len(seen) == 1 else True   # whole fails, pieces fit
        _widen_fitting_pieces(seg, rec_fits, 0.127, coord)
        return seen
    g_off = guards_for(make_seg(0.013, 0.0, 3.013, 0.0, width=0.3, net_id=1))
    check('off-grid segment: the WHOLE-segment check is guarded too',
          g_off[0] == G, g_off)
    check('off-grid segment: every piece is checked with the guard',
          len(g_off) > 2 and all(g == G for g in g_off[1:]), g_off)
    g_any = guards_for(make_seg(0.0, 0.0, 3.0, 1.0, width=0.3, net_id=1))
    check('on-grid ANY-angle segment: whole check exact, pieces guarded',
          g_any[0] == 0.0 and len(g_any) > 2
          and all(g == G for g in g_any[1:]), g_any)

    # The cell-centre MODEL, measured: every kept-wide piece must keep its
    # true centreline >= margin cells from every blocked cell centre. Random
    # on-grid any-angle and off-grid segments; the same sweep with the guard
    # forced to 0 is the control that shows the measurement can see a miss.
    import random

    def sweep(guard_value):
        saved = ser._OFFGRID_FIT_GUARD
        ser._OFFGRID_FIT_GUARD = guard_value
        try:
            rng = random.Random(1)
            bad = kept = 0
            for trial in range(400):
                mpx = GridObstacleMap(1)
                cells = []
                for _ in range(25):
                    cx, cy = rng.randint(0, 80), rng.randint(0, 80)
                    mpx.add_blocked_cell(cx, cy, 0)
                    cells.append((cx, cy))

                def fx(s, guard=0.0, _m=mpx):
                    return _segment_fits_wide(s, _m, coord, 0, 1.0 + guard)
                if trial % 3 == 0:
                    a = (rng.randint(0, 80) * 0.1, rng.randint(0, 80) * 0.1)
                    b = (rng.randint(0, 80) * 0.1, rng.randint(0, 80) * 0.1)
                else:
                    a = (rng.uniform(0, 8), rng.uniform(0, 8))
                    b = (rng.uniform(0, 8), rng.uniform(0, 8))
                sg = make_seg(a[0], a[1], b[0], b[1], width=0.3, net_id=1)
                for pc in _widen_fitting_pieces(sg, fx, 0.127, coord):
                    if pc.width != 0.3:
                        continue
                    kept += 1
                    ax_, ay_ = pc.start_x / 0.1, pc.start_y / 0.1
                    bx_, by_ = pc.end_x / 0.1, pc.end_y / 0.1
                    dx, dy = bx_ - ax_, by_ - ay_
                    L2 = dx * dx + dy * dy
                    dmin = 1e9
                    for cx, cy in cells:
                        t = 0.0 if L2 == 0 else max(0.0, min(1.0, (
                            (cx - ax_) * dx + (cy - ay_) * dy) / L2))
                        dmin = min(dmin, math.hypot(cx - ax_ - t * dx,
                                                    cy - ay_ - t * dy))
                    if dmin < 1.0 - 1e-6:
                        bad += 1
            return kept, bad
        finally:
            ser._OFFGRID_FIT_GUARD = saved
    k0, b0 = sweep(0.0)
    k1, b1 = sweep(G)
    check('model control: with NO guard the sweep finds pieces outside the '
          'model (the measurement can see a miss)', b0 > 0, (k0, b0))
    check('model: with the guard, 0 kept-wide pieces outside the model',
          b1 == 0 and k1 > 100, (k1, b1))

    n1 = len(ledger())
    segs = [make_seg(0, 0, 3, 0, width=0.3, net_id=1)]
    _assign_wide_route_widths(segs, cfg, 1, GridObstacleMap(1), coord,
                              ['F.Cu'], 0, False, None, neck_start=True)
    check('full width: nothing recorded', len(ledger()) == n1)


# ------------------------------- the widen pass's exact check at the pad
def t_exact_check():
    """#1033: the widen pass's pad check is the GRADER's pad copper (#1029): it must
    never call clear what check_drc calls a graze, including a tilted rect
    whose corner a circle approximation misjudges; and a piece landing in an
    own pad is capped at the pad's narrow side."""
    from kicad_parser import BoardInfo
    from synth import make_pcb, make_pad, make_net
    from routing_config import GridRouteConfig
    from power_widen import ExactWideCheck, widen_segment
    from check_drc import check_pad_segment_overlap
    cfg = GridRouteConfig(layers=['F.Cu'], track_width=0.127, clearance=0.1,
                          grid_step=0.05)
    bi = BoardInfo(layers={0: 'F.Cu'}, copper_layers=['F.Cu'],
                   board_bounds=(-5.0, -5.0, 5.0, 5.0))
    tilt = make_pad(2, 0.0, 0.6, ref='R1', num='1', size_x=0.8, size_y=0.25,
                    net_name='SIG', rect_rotation=30.0)
    own = make_pad(1, 2.0, 0.0, ref='U1', num='1', size_x=0.2, size_y=0.6,
                   net_name='+3V3')
    pcb = make_pcb(nets={1: make_net(1, '+3V3'), 2: make_net(2, 'SIG')},
                   pads_by_net={1: [own], 2: [tilt]}, board_info=bi)
    chk = ExactWideCheck(pcb, cfg, 1)
    disagree = 0
    loose = 0
    n = 0
    for k in range(60):
        yy = -0.2 + k * 0.01
        for w in (0.127, 0.2, 0.3):
            n += 1
            ok = chk.clears(-1.0, yy, 1.0, yy, 'F.Cu', w)
            graze = check_pad_segment_overlap(
                tilt, make_seg(-1.0, yy, 1.0, yy, width=w, net_id=1,
                               layer='F.Cu'), 0.1, ['F.Cu'], 0.0)[0]
            if ok and graze:
                disagree += 1
            if not ok and not graze:
                loose += 1
    check('widen exact check: never clears what check_drc grades as a pad graze '
          '(tilted rect, 180 cases)', disagree == 0, disagree)
    check('widen exact check: and is not needlessly conservative beside it',
          loose <= n * 0.1, (loose, n))
    # own-pad entry cap: a segment ending in U1's 0.2-wide pad
    s_ = make_seg(0.8, 0.0, 2.0, 0.0, width=0.127, net_id=1, layer='F.Cu')
    pieces = widen_segment(s_, 0.3, chk)
    at_pad = [p.width for p in pieces if max(p.start_x, p.end_x) > 1.9 - 1e-9]
    check('widen own-pad entry: capped at the pad narrow side (0.2), wider before',
          at_pad and max(at_pad) <= 0.2 + 1e-9
          and any(abs(p.width - 0.3) < 1e-9 for p in pieces),
          [(round(p.start_x, 3), round(p.end_x, 3), p.width) for p in pieces])


# ------------------------------------------------------------ summary merge
def t_merge():
    from route_summary import merge_summaries
    first = {'routed_single': [], 'failed_single': ['A'],
             'power_widths': {'+3V3': {'under_mm': 4.0}},
             'power_widths_measured_on': 'written board'}
    sub = {'routed_single': ['A'], 'failed_single': []}
    m = merge_summaries([first, sub])
    check('merge: power_widths survives the reconciliation merge',
          m.get('power_widths') == first['power_widths']
          and m.get('power_widths_measured_on') == 'written board',
          m.get('power_widths'))


# ---------------------------------------------------------------- end to end
# +3V3 from U1 (x=4) to U2 (x=26) on ONE layer, through a fence of foreign
# pads at x=15 whose only gap (y=10) is 0.4 mm wide: a 0.3 track at 0.1
# clearance needs 0.5, a 0.127 needs 0.327. The long trunk must neck down.
_NARROW_5V = ''' (footprint "t:P" (layer "F.Cu") (at 4 17)
  (property "Reference" "U3" (at 0 -2) (layer "F.SilkS"))
  (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 3 "+5V")))
 (footprint "t:P" (layer "F.Cu") (at 10 17)
  (property "Reference" "U4" (at 0 -2) (layer "F.SilkS"))
  (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 3 "+5V")))
 (segment (start 4 17) (end 10 17) (width 0.127) (layer "F.Cu") (net 3) (uuid "n5"))
'''


_FLANK = ''' (footprint "t:K" (layer "F.Cu") (at 24.5 10)
  (property "Reference" "J2" (at 0 -2) (layer "F.SilkS"))
  (pad "1" smd rect (at 0 -0.3) (size 0.2 0.2) (layers "F.Cu") (net 2 "SIG"))
  (pad "2" smd rect (at 0 0.3) (size 0.2 0.2) (layers "F.Cu") (net 2 "SIG")))
'''


def _board(path, gap=0.4, flank=True, narrow_5v=False):
    fence = []
    pad_h = 0.8
    y = 10.0 + gap / 2 + pad_h / 2
    k = 0
    while y < 20.5:                      # above the gap
        fence.append((15.0, y)); y += pad_h + 0.05; k += 1
    y = 10.0 - gap / 2 - pad_h / 2
    while y > -0.5:                      # below the gap
        fence.append((15.0, y)); y -= pad_h + 0.05; k += 1
    pads = ''.join(
        f'  (pad "{i + 1}" smd rect (at {fx - 15.0:.4f} {fy - 10.0:.4f}) '
        f'(size 0.8 {pad_h}) (layers "F.Cu") (net 2 "SIG"))\n'
        for i, (fx, fy) in enumerate(fence))
    flank_txt = _FLANK if flank else ''
    # an OUT-OF-SCOPE power net carrying narrow copper an earlier step laid
    net5 = ' (net 3 "+5V")\n ' if narrow_5v else ' '
    if narrow_5v:
        flank_txt += _NARROW_5V
    txt = f'''(kicad_pcb
 (version 20221018)
 (generator "test_1033")
 (general (thickness 1.6))
 (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))
 (net 0 "")
 (net 1 "+3V3")
 (net 2 "SIG")
{net5}(gr_rect (start 0 0) (end 30 20) (layer "Edge.Cuts") (width 0.1))
 (footprint "t:P" (layer "F.Cu") (at 4 10)
  (property "Reference" "U1" (at 0 -2) (layer "F.SilkS"))
  (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 1 "+3V3")))
 (footprint "t:P" (layer "F.Cu") (at 26 10)
  (property "Reference" "U2" (at 0 -2) (layer "F.SilkS"))
  (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 1 "+3V3")))
 (footprint "t:F" (layer "F.Cu") (at 15 10)
  (property "Reference" "J1" (at 0 -12) (layer "F.SilkS"))
{pads} )
{flank_txt})
'''
    with open(path, 'w', encoding='utf-8') as f:
        f.write(txt)


def t_end_to_end():
    from kicad_parser import parse_kicad_pcb
    with tempfile.TemporaryDirectory() as tmp:
        src = os.path.join(tmp, 'in.kicad_pcb')
        out = os.path.join(tmp, 'out.kicad_pcb')
        js = os.path.join(tmp, 'out.json')
        _board(src)
        env = dict(os.environ, MSYS2_ARG_CONV_EXCL='*')
        r = subprocess.run(
            [sys.executable, '-X', 'utf8',
             os.path.join(ROOT, 'py_router', 'route.py'), src, out,
             '--nets', '+3V3', '--layers', 'F.Cu', '--track-width', '0.127',
             '--clearance', '0.1', '--grid-step', '0.05',
             '--power-nets', '+3V3', '--power-nets-widths', '0.3',
             '--json-out', js],
            capture_output=True, text=True, encoding='utf-8', errors='replace',
            env=env, timeout=900)
        log = (r.stdout or '') + (r.stderr or '')
        if r.returncode != 0 or 'Traceback' in log:
            check('end to end: route.py ran', False, log[-1500:])
            return
        evidence(out, 'routed board')
        evidence(js, '--json-out summary')
        with open(js, encoding='utf-8') as f:
            doc = json.load(f)
        pw = (doc.get('power_widths') or {}).get('+3V3')
        check('end to end: power_widths present for +3V3', pw is not None,
              sorted(doc))
        if pw is None:
            return
        pcb = parse_kicad_pcb(out)
        nid = next(n for n, v in pcb.nets.items() if v.name == '+3V3')
        segs = [s for s in pcb.segments if s.net_id == nid]
        tot = sum(math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)
                  for s in segs)
        und = sum(math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)
                  for s in segs if s.width < 0.3 - 1e-3)
        check('end to end: the net routed', tot > 20.0, tot)
        check('end to end: it had to neck (the gap does not pass 0.3)',
              und > 0.0, und)
        # #1033 honour: the neck is the pad ends and the pinch, not the run.
        # Before the piecewise widen-back the whole 22 mm shipped at 0.127
        # (one straight segment crossing the gap failed the fit as a whole).
        check('end to end: no 0.127 run where 0.3 fits (under <= 10 of 22 mm)',
              und <= 10.0, (und, tot))

        # #1033 post-route widen: the PAD-NECK ZONE. U1's approach (x 4.5..6.5, free
        # space) used to be forced to the neck width for 2.5 mm; it now carries
        # 0.3 wherever that clears. J2's two SIG pads flank U2's approach at
        # x=24.5, 0.2 mm from the centreline: 0.3 does not fit between them
        # at 0.1 clearance (needs 0.25), so that stretch stays narrower.
        def _len_where(pred_x, pred_w):
            tot_ = 0.0
            for s_ in segs:
                n_ = 20
                for k_ in range(n_):
                    t_ = (k_ + 0.5) / n_
                    x_ = s_.start_x + (s_.end_x - s_.start_x) * t_
                    if pred_x(x_) and pred_w(s_.width):
                        tot_ += math.hypot(s_.end_x - s_.start_x,
                                           s_.end_y - s_.start_y) / n_
            return tot_
        free_wide = _len_where(lambda x: 4.6 < x < 6.4,
                               lambda w: abs(w - 0.3) < 1e-6)
        free_all = _len_where(lambda x: 4.6 < x < 6.4, lambda w: True)
        check('widen: the pad approach in free space is WIDE (0.3) in the neck zone',
              free_all > 1.0 and free_wide >= 0.9 * free_all,
              (round(free_wide, 3), round(free_all, 3)))
        flank_wide = _len_where(lambda x: 24.35 < x < 24.65,
                                lambda w: w > 0.25)
        flank_all = _len_where(lambda x: 24.35 < x < 24.65, lambda w: True)
        check('widen: between the flanking foreign pads it stays NECKED (< 0.3)',
              flank_all > 0.1 and flank_wide < 1e-6,
              (round(flank_wide, 3), round(flank_all, 3)))
        into_u2 = [s_.width for s_ in segs
                   if max(s_.start_x, s_.end_x) > 25.5 + 1e-6]
        check('widen: the entry into the 1 mm pad is no wider than the pad',
              into_u2 and max(into_u2) <= 1.0 + 1e-9, into_u2)
        # ...and the widened copper is legal: DRC-clean at the routed
        # clearance, and still connected (connectivity is orthogonal to DRC).
        drc = subprocess.run(
            [sys.executable, '-X', 'utf8',
             os.path.join(ROOT, 'py_router', 'check_drc.py'), out,
             '--clearance', '0.1'],
            capture_output=True, text=True, encoding='utf-8',
            errors='replace', env=env, timeout=600)
        check('end to end: the written board is DRC-clean at 0.1',
              drc.returncode == 0 and 'Traceback' not in drc.stdout,
              (drc.stdout or '')[-600:])
        con = subprocess.run(
            [sys.executable, '-X', 'utf8',
             os.path.join(ROOT, 'py_router', 'check_connected.py'), out,
             '--nets', '+3V3'],
            capture_output=True, text=True, encoding='utf-8',
            errors='replace', env=env, timeout=600)
        check('end to end: +3V3 is connected',
              con.returncode == 0 and 'Traceback' not in con.stdout,
              (con.stdout or '')[-600:])
        check('end to end: power_widths agrees with the written board',
              abs(pw['length_mm'] - tot) < 0.02
              and abs(pw['under_mm'] - und) < 0.02, (pw, tot, und))
        rows = [x for x in (doc.get('design_rules') or {}).get('narrowed', [])
                if x.get('net') == nid and x.get('kind') == 'track_width']
        check('end to end: design_rules carries ONE shipped row for the net, '
              'its length the power_widths under_mm',
              len(rows) == 1
              and rows[0]['site'] == 'power copper shipped under width'
              and abs(rows[0].get('length_mm', -1) - pw['under_mm']) < 0.02
              and rows[0]['delivered'] == pw['min_mm'], rows)
        check('end to end: the console names it',
              'Power widths: +3V3' in log, log[-800:])
        check('end to end: the summary says which copper it measured',
              doc.get('power_widths_measured_on') == 'written board',
              doc.get('power_widths_measured_on'))

        # GUI front (return_results): the same engine call, measured on the
        # change-set it hands the applier. Same board, same copper as the
        # CLI's written file (no plane pour here, so no post-apply oracle).
        import route as _route
        _ok, _f, _t, data = _route.batch_route(
            src, '', ['+3V3'], return_results=True, layers=['F.Cu'],
            track_width=0.127, clearance=0.1, grid_step=0.05,
            power_nets=['+3V3'], power_nets_widths=[0.3])
        gpw = (data.get('power_widths') or {}).get('+3V3')
        check('GUI front: power_widths reaches results_data',
              gpw is not None, sorted(data))
        if gpw is not None:
            check('GUI front: it measures the same copper as the CLI file',
                  abs(gpw['length_mm'] - pw['length_mm']) < 0.02
                  and abs(gpw['under_mm'] - pw['under_mm']) < 0.02,
                  (gpw, pw))
            check('GUI front: it says it measured the change-set',
                  str(data.get('power_widths_measured_on', '')
                      ).startswith('change-set'),
                  data.get('power_widths_measured_on'))


# ------------------------------------------------ GUI oracle payload parity
def t_gui_oracle_payload():
    """The GUI's fallback plane-finalize oracle (posted as
    results_data['plane_finalize_oracle'], run by swig_gui through
    gui_utils.run_kicad_oracle_on_live_board) must receive the per-net widths
    the CLI's oracle config (_ocfg) carries, or its weld ladder stops at a
    different width. Every payload key must be a parameter of the applier
    and be forwarded by swig_gui."""
    import ast
    import inspect
    sys.path.insert(0, ROOT)
    import kicad_routing_plugin.gui_utils as gu
    params = set(inspect.signature(gu.run_kicad_oracle_on_live_board).parameters)
    src = open(os.path.join(ROOT, 'py_router', 'route.py'),
               encoding='utf-8').read()
    tree = ast.parse(src)
    keys = set()
    for node in ast.walk(tree):
        if (isinstance(node, ast.Assign) and len(node.targets) == 1
                and isinstance(node.targets[0], ast.Subscript)
                and isinstance(node.targets[0].slice, ast.Constant)
                and node.targets[0].slice.value == 'plane_finalize_oracle'
                and isinstance(node.value, ast.Dict)):
            keys |= {k.value for k in node.value.keys
                     if isinstance(k, ast.Constant)}
    gui_src = open(os.path.join(ROOT, 'kicad_routing_plugin', 'swig_gui.py'),
                   encoding='utf-8').read()
    check('GUI oracle payload carries the per-net widths',
          {'power_net_widths', 'net_track_widths',
           'net_layer_widths'} <= keys, sorted(keys))
    for k in ('net_track_widths', 'net_layer_widths'):
        check(f'GUI applier accepts and swig_gui forwards {k}',
              k in params and f"{k}=_pfo.get('{k}')" in gui_src)


# ------------------------------------------- --strict-sizes, both ways
def t_strict_sizes():
    """--strict-sizes exits 3 only when SHIPPED copper is under width.
    A 0.5 mm gap: the router necks (its grid map refuses 0.3), the post-route
    pass widens it all back on exact geometry -> no row, exit 0. The 0.4 mm
    gap plus the flanking pads: a real neck ships -> one row, exit 3."""
    from kicad_parser import parse_kicad_pcb
    for gap, flank, want, n5 in ((0.5, False, 0, False), (0.4, True, 3, False),
                                 (0.5, False, 0, True), (0.4, True, 3, True)):
        with tempfile.TemporaryDirectory() as tmp:
            src = os.path.join(tmp, 'in.kicad_pcb')
            out = os.path.join(tmp, 'out.kicad_pcb')
            js = os.path.join(tmp, 'out.json')
            _board(src, gap=gap, flank=flank, narrow_5v=n5)
            env = dict(os.environ, MSYS2_ARG_CONV_EXCL='*')
            pnets = (['+3V3', '+5V'], ['0.3', '0.4']) if n5 else (['+3V3'], ['0.3'])
            r = subprocess.run(
                [sys.executable, '-X', 'utf8',
                 os.path.join(ROOT, 'py_router', 'route.py'), src, out,
                 '--nets', '+3V3', '--layers', 'F.Cu', '--track-width', '0.127',
                 '--clearance', '0.1', '--grid-step', '0.05',
                 '--power-nets', *pnets[0], '--power-nets-widths', *pnets[1],
                 '--json-out', js, '--strict-sizes'],
                capture_output=True, text=True, encoding='utf-8',
                errors='replace', env=env, timeout=900)
            log = (r.stdout or '') + (r.stderr or '')
            if 'Traceback' in log or not os.path.isfile(js):
                check(f'strict-sizes gap {gap}: route.py ran', False, log[-800:])
                continue
            with open(js, encoding='utf-8') as f:
                doc = json.load(f)
            pw = doc['power_widths']['+3V3']
            rows = [x for x in doc['design_rules']['narrowed']
                    if x['kind'] == 'track_width']
            if n5:
                p5 = doc['power_widths'].get('+5V') or {}
                check(f'scoped (gap {gap}): the out-of-scope +5V is still '
                      f'DISCLOSED, labelled out of scope, and has no row',
                      p5.get('under_mm', 0) > 5.0
                      and p5.get('in_run_scope') is False
                      and doc.get('power_widths_run_scope') == ['+3V3']
                      and not any(x.get('net_name') == '+5V' for x in rows),
                      (p5, doc.get('power_widths_run_scope'), rows))
            necked = 'neck-down' in log or 'short edge at' in log
            if want == 0:
                check('strict-sizes: the router DID neck this net (the case '
                      'is real)', necked)
                check(f'strict-sizes (+5V narrow copper out of scope: {n5}): '
                      'fully widened back -> 0 under, no row, exit 0', pw['under_mm'] == 0 and rows == []
                      and r.returncode == 0, (pw['under_mm'], rows, r.returncode))
                drc = subprocess.run(
                    [sys.executable, '-X', 'utf8',
                     os.path.join(ROOT, 'py_router', 'check_drc.py'), out,
                     '--clearance', '0.1'], capture_output=True, text=True,
                    encoding='utf-8', errors='replace', env=env, timeout=600)
                check('strict-sizes: the widened board is DRC-clean at 0.1',
                      drc.returncode == 0, (drc.stdout or '')[-400:])
                p = parse_kicad_pcb(out)
                nid = next(n for n, v in p.nets.items() if v.name == '+3V3')
                check('strict-sizes: every +3V3 segment ships at 0.3',
                      all(abs(sg.width - 0.3) < 1e-6 for sg in p.segments
                          if sg.net_id == nid))
            else:
                check(f'strict-sizes (+5V out of scope: {n5}): a real neck '
                      'ships -> one row, exit 3',
                      pw['under_mm'] > 0 and len(rows) == 1
                      and r.returncode == 3,
                      (pw['under_mm'], rows, r.returncode))


if __name__ == '__main__':
    t_gui_oracle_payload()
    t_report()
    t_exact_check()
    t_assign()
    t_merge()
    t_end_to_end()
    t_strict_sizes()
    if fails:
        print(f'{len(fails)} FAILURE(S): {fails}')
        sys.exit(1)
    print('all checks passed')
