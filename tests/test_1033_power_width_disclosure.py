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
    check('short edge: ONE narrowing row, site, requested, delivered',
          len(new) == 1 and new[0]['site'] == 'power short edge'
          and new[0]['requested'] == 0.3 and new[0]['delivered'] == 0.15, new)

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

    seen_guard = []

    def rec_fits(s, guard=0.0):
        seen_guard.append(guard)
        return guard != 0.0
    off = make_seg(0.013, 0.0, 3.013, 0.0, width=0.3, net_id=1)  # off grid
    _widen_fitting_pieces(off, rec_fits, 0.127, coord)
    check('off-grid segment: every piece is checked with the half-cell guard',
          seen_guard[0] == 0.0 and len(seen_guard) > 2
          and all(g == 0.5 for g in seen_guard[1:]), seen_guard)

    n1 = len(ledger())
    segs = [make_seg(0, 0, 3, 0, width=0.3, net_id=1)]
    _assign_wide_route_widths(segs, cfg, 1, GridObstacleMap(1), coord,
                              ['F.Cu'], 0, False, None, neck_start=True)
    check('full width: nothing recorded', len(ledger()) == n1)


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
def _board(path):
    fence = []
    gap = 0.4
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
    txt = f'''(kicad_pcb
 (version 20221018)
 (generator "test_1033")
 (general (thickness 1.6))
 (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))
 (net 0 "")
 (net 1 "+3V3")
 (net 2 "SIG")
 (gr_rect (start 0 0) (end 30 20) (layer "Edge.Cuts") (width 0.1))
 (footprint "t:P" (layer "F.Cu") (at 4 10)
  (property "Reference" "U1" (at 0 -2) (layer "F.SilkS"))
  (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 1 "+3V3")))
 (footprint "t:P" (layer "F.Cu") (at 26 10)
  (property "Reference" "U2" (at 0 -2) (layer "F.SilkS"))
  (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 1 "+3V3")))
 (footprint "t:F" (layer "F.Cu") (at 15 10)
  (property "Reference" "J1" (at 0 -12) (layer "F.SilkS"))
{pads} )
)
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
        check('end to end: the narrowing is in design_rules too',
              any(x['site'].startswith('power ') for x in rows), rows)
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


if __name__ == '__main__':
    t_gui_oracle_payload()
    t_report()
    t_assign()
    t_merge()
    t_end_to_end()
    if fails:
        print(f'{len(fails)} FAILURE(S): {fails}')
        sys.exit(1)
    print('all checks passed')
