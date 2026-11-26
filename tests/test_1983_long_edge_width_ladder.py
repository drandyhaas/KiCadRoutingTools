#!/usr/bin/env python3
"""#1983: a long power edge that routes at the signal width but not at its
own is tried again at the widths between, and ships at the widest that routes.

`route.py --power-nets-widths 2.0` on kicad_files/splitflap_driver shipped
52.3 mm of +12V at the 0.25 mm signal width where asking 1.0 kept it at
0.5 mm or more: an edge longer than SHORT_POWER_EDGE_MM that failed at the
requested width was retried once, at the layer width, while a short edge
stepped down through _power_width_ladder. A long edge now retries at the
layer width first, as before, and only when that routes tries the ladder's
rungs above it (net/2, net/4, ...), widest first.

Rows, on _route_main_connection with a stand-in search (which widths route
is the row's input; the calls it makes are recorded):
  1. a long edge where 1.0 of 2.0 routes ships at 1.0 (uniform_width), after
     the full width and the layer width, and the log names net, span, width;
  2. no rung routes: each rung is tried, the log names them, and (CONTROL)
     the layer-width route ships necked_down, as on main;
  3. nothing routes: the log says so, and (CONTROL) exactly the two searches
     main runs and the same union frontier for rip-up, no rung search;
  4. CONTROL: a short edge keeps the short-edge ladder and its log line;
  5. CONTROL: an edge that fits at the full width runs one search.
End to end, route.py on one-layer boards: +12V between two pads across a
fence of foreign pads with one gap:
  6. long edge (16 mm), a 2.0 mm gap (1.0 passes, 2.0 does not): the edge
     ships at 2.0 where it fits and at 1.0 in the gap, nothing at 0.25
     (main: 0.25 in the gap), and the log line says why; the GUI front
     (batch_route with return_results, as swig_gui calls it) ships the same;
  7. CONTROL: long edge, a 0.7 mm gap (only 0.25 passes): 4 mm at 0.25 and
     5 mm under the request, as on main;
  8. CONTROL: short edge (8 mm), 2.0 mm gap: the short-edge ladder's 1.0;
  9. CONTROL: long edge, no fence: all of it at 2.0, no blocked line.

    python3 tests/test_1983_long_edge_width_ladder.py      (about 3 s)
"""
import contextlib
import io
import json
import os
import subprocess
import sys
import tempfile
import types
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, HERE)

from run_utils import evidence, tool  # noqa: E402

FAILS = []


def check(name, cond, detail=''):
    print(('  [PASS] ' if cond else '  [FAIL] ') + name
          + (f'  {detail}' if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


# ------------------------------------------------- the branch, search stubbed
FULL = ('full',)
FLOOR = ('floor',)


def _w(w):
    return ('w', round(w, 4))


def _route(fits, span_cells):
    """_route_main_connection on a 2.0 mm power net (layer width 0.25) whose
    searches route exactly at the margins in `fits`. Returns (result, the
    margins searched in order, stdout)."""
    import single_ended_routing as ser
    from routing_config import GridRouteConfig
    cfg = GridRouteConfig(layers=['F.Cu'], track_width=0.25, clearance=0.1,
                          grid_step=0.1)
    cfg.power_net_widths = {1: 2.0}
    cfg.base_track_margins = lambda: FLOOR
    cfg.track_margins_for_width = _w
    calls = []

    def search(router, obstacles, config, sources, targets, track_margin,
               *a, **k):
        calls.append(track_margin)
        if track_margin in fits:
            return ([(0, 0, 0), (1, 0, 0), track_margin], 10, [], [], False,
                    5, 5)
        return (None, 100, [('fwd',) + track_margin],
                [('bwd',) + track_margin], False, 50, 50)

    pcb = types.SimpleNamespace(nets={1: types.SimpleNamespace(name='+12V')})
    saved = ser._route_connection_at_margin
    ser._route_connection_at_margin = search
    out = io.StringIO()
    try:
        with contextlib.redirect_stdout(out):
            res = ser._route_main_connection(
                None, None, cfg, [(0, 0, 0)], [(span_cells, 0, 0)], FULL,
                pcb, 1)
    finally:
        ser._route_connection_at_margin = saved
    return res, calls, out.getvalue()


def t_branch():
    import single_ended_routing as ser
    long_cells = 160        # 16 mm at 0.1 mm per cell
    short_cells = 80        # 8 mm
    check('precondition: 16 mm is a long edge, 8 mm a short one',
          short_cells * 0.1 <= ser.SHORT_POWER_EDGE_MM < long_cells * 0.1,
          ser.SHORT_POWER_EDGE_MM)

    res, calls, log = _route({FLOOR, _w(1.0), _w(0.5)}, long_cells)
    check('1. a long edge where 1.0 routes searches full, layer width, 1.0',
          calls == [FULL, FLOOR, _w(1.0)], calls)
    check('1. it ships at 1.0: uniform_width 1.0, not necked down',
          res[-2:] == (False, 1.0) and res[0][-1] == _w(1.0), res[-2:])
    check('1. the log names the net, the span and the width reached',
          'Wide power route blocked on +12V: long edge spanning 16.00mm '
          'routed at 1.0000mm (down from 2.0000)' in log, log)

    res, calls, log = _route({FLOOR}, long_cells)
    check('2. no rung routes: every rung was tried, widest first',
          calls == [FULL, FLOOR, _w(1.0), _w(0.5)], calls)
    check('2. CONTROL: the layer-width route ships necked down, as on main',
          res[0][-1] == FLOOR and res[-2:] == (True, None), res[-2:])
    check('2. the log says which widths did not route',
          'long edge spanning 16.00mm routed at the 0.2500mm neck width '
          '(down from 2.0000; 1.0000, 0.5000mm did not route), neck-down'
          in log, log)

    res, calls, log = _route(set(), long_cells)
    check('3. CONTROL: an edge no width routes runs main\'s two searches, '
          'no rung', calls == [FULL, FLOOR], calls)
    check('3. CONTROL: it fails with the union frontier main returns (#1400)',
          res[0] is None and res[1] == 200
          and res[2] == [('fwd',) + FULL, ('fwd',) + FLOOR]
          and res[3] == [('bwd',) + FULL, ('bwd',) + FLOOR]
          and res[-2:] == (False, None), res)
    check('3. the log says the neck width does not route either',
          'long edge spanning 16.00mm does not route at the 0.2500mm neck '
          'width either' in log, log)

    res, calls, log = _route({_w(1.0), _w(0.5), _w(0.25)}, short_cells)
    check('4. CONTROL: a short edge steps down its own ladder to 1.0',
          calls == [FULL, _w(1.0)] and res[-2:] == (False, 1.0), calls)
    check('4. CONTROL: with the short-edge line, and no long-edge line',
          'Wide power route blocked - routed short edge at 1.0000mm (down '
          'from 2.0000)' in log and 'long edge' not in log, log)

    res, calls, log = _route({FULL}, long_cells)
    check('5. CONTROL: an edge that fits at full width runs one search',
          calls == [FULL] and res[-2:] == (False, None) and log == '',
          (calls, log))


# ---------------------------------------------------------------- end to end
def _board(path, span, gap, fence=True, height=12.0):
    """+12V between two 3 mm pads `span` mm apart on F.Cu; with `fence`, a
    column of foreign pads across the whole board at the midpoint whose only
    gap is `gap` mm wide."""
    c = height / 2
    x1, x2 = 4.0, 4.0 + span
    ys = []
    if fence:
        y = c + gap / 2 + 0.4
        while y < height + 0.5:
            ys.append(y)
            y += 0.85
        y = c - gap / 2 - 0.4
        while y > -0.5:
            ys.append(y)
            y -= 0.85
    pads = ''.join(f'  (pad "{i + 1}" smd rect (at 0 {fy - c:.4f}) (size 0.8 '
                   f'0.8) (layers "F.Cu") (net 2 "SIG"))\n'
                   for i, fy in enumerate(ys))
    fence_txt = (f' (footprint "t:F" (layer "F.Cu") (at {(x1 + x2) / 2} {c})\n'
                 f'  (property "Reference" "J1" (at 0 -8) (layer "F.SilkS"))\n'
                 f'{pads} )\n') if fence else ''
    pad = ('  (pad "1" smd rect (at 0 0) (size 3 3) (layers "F.Cu") '
           '(net 1 "+12V")))\n')
    txt = (f'(kicad_pcb\n (version 20221018)\n (generator "test_1983")\n'
           f' (general (thickness 1.6))\n'
           f' (layers (0 "F.Cu" signal) (31 "B.Cu" signal) '
           f'(44 "Edge.Cuts" user))\n'
           f' (net 0 "")\n (net 1 "+12V")\n (net 2 "SIG")\n'
           f' (gr_rect (start 0 0) (end {x2 + 4.0} {height}) '
           f'(layer "Edge.Cuts") (width 0.1))\n'
           f' (footprint "t:P" (layer "F.Cu") (at {x1} {c})\n'
           f'  (property "Reference" "U1" (at 0 -3) (layer "F.SilkS"))\n{pad}'
           f' (footprint "t:P" (layer "F.Cu") (at {x2} {c})\n'
           f'  (property "Reference" "U2" (at 0 -3) (layer "F.SilkS"))\n{pad}'
           f'{fence_txt})\n')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(txt)


def _route_board(tmp, tag, span, gap, fence=True):
    src = os.path.join(tmp, f'{tag}.kicad_pcb')
    out = os.path.join(tmp, f'{tag}_out.kicad_pcb')
    js = os.path.join(tmp, f'{tag}.json')
    _board(src, span, gap, fence)
    r = subprocess.run(
        [sys.executable, '-X', 'utf8', tool('route.py'), src, out,
         '--nets', '+12V', '--layers', 'F.Cu', '--track-width', '0.25',
         '--clearance', '0.1', '--grid-step', '0.1',
         '--power-nets', '+12V', '--power-nets-widths', '2.0',
         '--json-out', js],
        capture_output=True, text=True, encoding='utf-8', errors='replace',
        env=dict(os.environ, MSYS2_ARG_CONV_EXCL='*'), timeout=600)
    log = (r.stdout or '') + (r.stderr or '')
    if r.returncode != 0 or 'Traceback' in log:
        return None, log
    evidence(out, f'{tag}: routed board')
    with open(evidence(js, f'{tag}: --json-out summary'), encoding='utf-8') as f:
        doc = json.load(f)
    return doc, log


def t_end_to_end():
    cases = {'fix': (16, 2.0, True), 'none': (16, 0.7, True),
             'short': (8, 2.0, True), 'fits': (16, 0.0, False)}
    with tempfile.TemporaryDirectory() as tmp:
        with ThreadPoolExecutor(len(cases)) as ex:
            futs = {k: ex.submit(_route_board, tmp, k, *v)
                    for k, v in cases.items()}
            got = {k: f.result() for k, f in futs.items()}
        # The GUI front: the same engine call swig_gui makes (return_results,
        # no output file), measured on the change-set it hands the applier.
        import route as _route
        with contextlib.redirect_stdout(io.StringIO()):
            _ok, _f, _t, gui = _route.batch_route(
                os.path.join(tmp, 'fix.kicad_pcb'), '', ['+12V'],
                return_results=True, layers=['F.Cu'], track_width=0.25,
                clearance=0.1, grid_step=0.1, power_nets=['+12V'],
                power_nets_widths=[2.0])
    gui_pw = ((gui or {}).get('power_widths') or {}).get('+12V') or {}
    pw = {}
    for k, (doc, log) in got.items():
        routed = doc is not None and doc.get('routed_single') == ['+12V']
        check(f'{k}: route.py ran and routed +12V', routed,
              log[-1500:] if doc is None else doc.get('routed_single'))
        pw[k] = ((doc or {}).get('power_widths') or {}).get('+12V') or {}

    doc, log = got['fix']
    p = pw['fix']
    check('6. the long edge ships at 1.0 in the gap: nothing narrower '
          '(main: min 0.25)', p.get('min_mm') == 1.0, p)
    check('6. no copper at the 0.25 signal width, and 4 mm at 1.0',
          '0.25' not in p.get('by_width_mm', {})
          and p.get('by_width_mm', {}).get('1') == 4.0, p.get('by_width_mm'))
    check('6. the full width is kept where it fits (11 of 16 mm at 2.0)',
          p.get('by_width_mm', {}).get('2') == 11.0, p.get('by_width_mm'))
    check('6. the log says why: routed at 1.0000mm (down from 2.0000)',
          'Wide power route blocked on +12V: long edge spanning 16.00mm '
          'routed at 1.0000mm (down from 2.0000)' in log, log[-1500:])
    check('6. the GUI front ships the same widths (min 1.0, 4 mm at 1.0)',
          gui_pw.get('min_mm') == 1.0
          and gui_pw.get('by_width_mm') == p.get('by_width_mm'),
          (gui_pw, p.get('by_width_mm')))

    doc, log = got['none']
    p = pw['none']
    check('7. CONTROL: no rung fits: 4 mm at 0.25, 5 mm under 2.0, as on main',
          p.get('min_mm') == 0.25 and p.get('under_mm') == 5.0
          and p.get('by_width_mm', {}).get('0.25') == 4.0, p)
    check('7. the log names the widths that did not route',
          '(down from 2.0000; 1.0000, 0.5000mm did not route), neck-down'
          in log, log[-1500:])

    doc, log = got['short']
    p = pw['short']
    check('8. CONTROL: a short edge keeps the short-edge ladder at 1.0',
          p.get('min_mm') == 1.0
          and 'Wide power route blocked - routed short edge at 1.0000mm '
              '(down from 2.0000)' in log
          and 'long edge' not in log, (p, log[-800:]))

    doc, log = got['fits']
    p = pw['fits']
    check('9. CONTROL: an edge that fits ships at 2.0 end to end',
          p.get('min_mm') == 2.0 and p.get('under_mm') == 0.0
          and 'route blocked' not in log, p)


def main():
    t_branch()
    t_end_to_end()
    print('FAILED: ' + ', '.join(FAILS) if FAILS else 'ALL OK')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
