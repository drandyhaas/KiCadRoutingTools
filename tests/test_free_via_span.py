#!/usr/bin/env python3
"""A route rides an existing blind/micro via only between the layers it spans
(found reviewing #1268).

Every existing same-net via was registered as a FREE via -- a zero-cost layer
change on every layer at its cell -- and route conversion emitted no via at a
layer change there. So a net routed from an F.Cu pad that carries an
F.Cu->In1.Cu microvia to a B.Cu pad changed layers AT the microvia, at no cost,
and shipped no via at all: the route's B.Cu track ends on a barrel that never
reaches B.Cu. route.py's own re-grade caught the open on the board below and a
terminal escalation drilled a through via on top of the microvia's spot; with
no re-grade behind it the net ships open.

The fix keeps the free via but closes its cell on the layers the via does not
span, takes it out of the every-layer hole sets conversion trusts, and lets a
layer change ride it only between two layers it spans
(obstacle_map.same_net_partial_via_spans).

    python3 tests/test_free_via_span.py
"""
import os
import re
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(HERE)
sys.path.insert(0, ROOT_DIR)
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_router'))  # #522
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_tools'))  # #522

from kicad_parser import parse_kicad_pcb  # noqa: E402
from routing_config import GridCoord, GridRouteConfig  # noqa: E402

FAILS = []

# Net /A: an F.Cu pad with an F.Cu->In1.Cu microvia at its centre, a B.Cu pad
# 10 mm east, and a foreign F.Cu wall between them, so the route must change
# layers before the wall -- and the microvia is the cheapest place to do it.
BOARD = '''(kicad_pcb
\t(version 20241229)
\t(generator "pcbnew")
\t(generator_version "10.0")
\t(general (thickness 1.6))
\t(layers
\t\t(0 "F.Cu" signal)
\t\t(4 "In1.Cu" signal)
\t\t(6 "In2.Cu" signal)
\t\t(2 "B.Cu" signal)
\t\t(25 "Edge.Cuts" user)
\t)
\t(net 0 "")
\t(net 1 "/A")
\t(net 2 "/W")
\t(footprint "t:pad" (layer "F.Cu") (uuid "f1") (at 10 10)
\t\t(property "Reference" "U1" (at 0 0 0) (layer "F.SilkS") (uuid "r1"))
\t\t(pad "1" smd rect (at 0 0) (size 0.8 0.8) (layers "F.Cu" "F.Paste" "F.Mask") (net 1 "/A") (uuid "p1"))
\t)
\t(footprint "t:pad" (layer "B.Cu") (uuid "f2") (at 20 10)
\t\t(property "Reference" "U2" (at 0 0 0) (layer "B.SilkS") (uuid "r2"))
\t\t(pad "1" smd rect (at 0 0) (size 0.8 0.8) (layers "B.Cu" "B.Paste" "B.Mask") (net 1 "/A") (uuid "p2"))
\t)
\t(gr_line (start 0 0) (end 30 0) (stroke (width 0.05) (type solid)) (layer "Edge.Cuts") (uuid "e1"))
\t(gr_line (start 30 0) (end 30 20) (stroke (width 0.05) (type solid)) (layer "Edge.Cuts") (uuid "e2"))
\t(gr_line (start 30 20) (end 0 20) (stroke (width 0.05) (type solid)) (layer "Edge.Cuts") (uuid "e3"))
\t(gr_line (start 0 20) (end 0 0) (stroke (width 0.05) (type solid)) (layer "Edge.Cuts") (uuid "e4"))
\t(segment (start 15 0.5) (end 15 19.5) (width 0.5) (layer "F.Cu") (net 2) (uuid "w1"))
\t(via micro (at 10 10) (size 0.3) (drill 0.1) (layers "F.Cu" "In1.Cu") (net 1) (uuid "vm"))
)
'''
LAYERS = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']


def check(label, ok, detail=''):
    print(('PASS ' if ok else 'FAIL ') + label + (f'  [{detail}]' if not ok else ''))
    if not ok:
        FAILS.append(label)


def _board(td, typed=True):
    path = os.path.join(td, 'micro.kicad_pcb' if typed else 'through.kicad_pcb')
    text = BOARD if typed else BOARD.replace(
        '(via micro (at 10 10) (size 0.3) (drill 0.1) (layers "F.Cu" "In1.Cu")',
        '(via (at 10 10) (size 0.3) (drill 0.1) (layers "F.Cu" "B.Cu")')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(text)
    return path


def _cfg():
    cfg = GridRouteConfig()
    cfg.layers = list(LAYERS)
    cfg.grid_step = 0.05
    return cfg


def test_units(td):
    try:
        from obstacle_map import same_net_partial_via_spans
    except ImportError as e:
        check("obstacle_map has same_net_partial_via_spans (a blind/micro via "
              "was an every-layer free hole)", False, e)
        return
    from obstacle_map import GridObstacleMap, get_same_net_through_hole_positions
    from routing_context import _add_free_via_positions
    from single_ended_routing import _path_to_segments_vias, _reusable_same_net_via
    cfg = _cfg()
    coord = GridCoord(cfg.grid_step)
    cell = coord.to_grid(10.0, 10.0)
    for typed in (True, False):
        pcb = parse_kicad_pcb(_board(td, typed))
        tag = 'microvia' if typed else 'through via (control)'
        holes = get_same_net_through_hole_positions(pcb, 1, cfg)
        check(f"{tag}: {'NOT ' if typed else ''}an every-layer hole for conversion",
              (cell in holes) is (not typed))
        spans = same_net_partial_via_spans(pcb, 1, cfg)
        check(f"{tag}: partial span map is "
              f"{'{F.Cu, In1.Cu}' if typed else 'empty'}",
              spans == ({cell: frozenset({'F.Cu', 'In1.Cu'})} if typed else {}), spans)
        obs = GridObstacleMap(len(LAYERS))
        stamped = _add_free_via_positions(obs, pcb, [1], cfg)
        blocked = [LAYERS[i] for i in range(len(LAYERS)) if obs.is_blocked(cell[0], cell[1], i)]
        check(f"{tag}: still a free via", obs.is_free_via(*cell))
        check(f"{tag}: its cell is closed on "
              f"{'In2.Cu and B.Cu only' if typed else 'no layer'}",
              blocked == (['In2.Cu', 'B.Cu'] if typed else []), blocked)
        check(f"{tag}: the closed cells are what the caller must remove",
              len(stamped) == len(blocked))
        # conversion: a layer change AT the via's cell
        for l2 in (1, 3):
            # A through via carries every change; the microvia only F->In1.
            want_via = typed and l2 == 3
            path = [(cell[0], cell[1], 0), (cell[0], cell[1], l2),
                    (cell[0] + 40, cell[1], l2)]
            segs, vias = _path_to_segments_vias(
                path, coord, LAYERS, 1, cfg, None, None,
                through_hole_positions=holes, pcb_data=pcb)
            check(f"{tag}: a F.Cu->{LAYERS[l2]} change at its cell "
                  f"{'drills a via' if want_via else 'rides it'}",
                  bool(vias) is want_via, len(vias))
        ride = _reusable_same_net_via(pcb, 1, 10.0, 10.0, 0.15, cfg, ('F.Cu', 'B.Cu'))
        check(f"{tag}: #671's ride-the-barrel {'refuses' if typed else 'takes'} it "
              f"for F.Cu->B.Cu", (ride is None) is typed, ride)


def test_terminals(td):
    """The route's own terminals: a via in the pad is offered as a start point
    on the layers it reaches -- every layer for a through via, but not B.Cu
    for the microvia, or the route simply STARTS on B.Cu at the microvia."""
    from connectivity import get_net_endpoints
    cfg = _cfg()
    cell = GridCoord(cfg.grid_step).to_grid(10.0, 10.0)
    for typed in (True, False):
        pcb = parse_kicad_pcb(_board(td, typed))
        srcs, tgts, _err = get_net_endpoints(pcb, 1, cfg)
        at_cell = {LAYERS[t[2]] for t in list(srcs) + list(tgts)
                   if (t[0], t[1]) == cell}
        tag = 'microvia' if typed else 'through via (control)'
        check(f"{tag}: offered as a route terminal on "
              f"{'F.Cu and In1.Cu only' if typed else 'every layer'}",
              at_cell == ({'F.Cu', 'In1.Cu'} if typed else set(LAYERS)), sorted(at_cell))


def test_inplace_balance(td):
    """prepare/restore_obstacles_inplace close and re-open the cell exactly."""
    from obstacle_map import GridObstacleMap
    from routing_context import prepare_obstacles_inplace, restore_obstacles_inplace
    cfg = _cfg()
    coord = GridCoord(cfg.grid_step)
    cell = coord.to_grid(10.0, 10.0)
    pcb = parse_kicad_pcb(_board(td, True))
    obs = GridObstacleMap(len(LAYERS))
    _stubs, snv = prepare_obstacles_inplace(obs, pcb, cfg, 1, [1], [], {},
                                            {n: i for i, n in enumerate(LAYERS)}, {})
    check("prepare_obstacles_inplace closes the microvia's B.Cu cell",
          obs.is_blocked(cell[0], cell[1], 3))
    restore_obstacles_inplace(obs, 1, {}, snv)
    check("restore_obstacles_inplace re-opens it (no leak into the next net)",
          not any(obs.is_blocked(cell[0], cell[1], i) for i in range(len(LAYERS))))


def test_route(td):
    """End to end: route.py on the board above."""
    path = _board(td, True)
    out = os.path.join(td, 'out.kicad_pcb')
    env = dict(os.environ, PYTHONUTF8='1')
    r = subprocess.run([sys.executable, '-X', 'utf8',
                        os.path.join(ROOT_DIR, 'py_router', 'route.py'), path,
                        '--output', out, '--nets', '/A', '--layers', 'F.Cu', 'B.Cu'],
                       capture_output=True, text=True, env=env, timeout=600)
    log = r.stdout + r.stderr
    check("route.py ran", r.returncode == 0 and os.path.isfile(out), log[-400:])
    m = re.search(r'SUCCESS: (\d+) segments, (\d+) vias', log)
    check("the first route drills its own via instead of riding the microvia "
          "to B.Cu", m is not None and int(m.group(2)) >= 1, m and m.group(0))
    check("...so no terminal escalation has to rescue an open net",
          'terminal escalation:' not in log)
    if os.path.isfile(out):
        from check_connected import check_net_connectivity
        pcb = parse_kicad_pcb(out)
        res = check_net_connectivity(1, [s for s in pcb.segments if s.net_id == 1],
                                     [v for v in pcb.vias if v.net_id == 1],
                                     pcb.pads_by_net.get(1, []))
        check("the shipped net is connected", res['connected'], res.get('message'))


def main():
    with tempfile.TemporaryDirectory(prefix='tfreevia_') as td:
        test_units(td)
        test_terminals(td)
        test_inplace_balance(td)
        test_route(td)
    if FAILS:
        print(f"\n{len(FAILS)} FAILED")
        return 1
    print("\nall passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
