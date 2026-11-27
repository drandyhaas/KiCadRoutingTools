#!/usr/bin/env python3
"""#1235 at its source: with GridRouteConfig.via_centre_ends (route.py
--via-centre-ends) a track that ends on, or runs through, a same-net via
meets it at the via's CENTRE, made where each such end is made.

Checks:
  1. A layer change is laid on the centre of the barrel that carries it: the
     via the conversion drills at an unblock's off-cell site, and the snap
     hands that barrel back so the conversion emits the via it centred on.
  2. The move is refused when a leg into the centre would graze foreign
     copper the grid legs clear (at a path end, the pad's bridge too), and
     never moves a merged terminal.
  3. A path whose bend sits in a via's disc passes the via's centre; a via
     without copper on the path's layer is left alone.
  4. Tap points: a sample inside a via's copper is owned by the via's centre
     with centre_vias, by the sample itself without.
  5. A path end on a #189 unblock via's cell moves onto the site the unblock
     registered, and stays when its leg would graze.
  6. nudge_grazing_vias' drag: the run's tracks ending at the via's centre
     follow it; input copper keeps the via where it is.
  6b. close_soft_joints bridges an off-centre end of any net to the centre
     with the flag, from the end itself when the rim cut would graze.
  7. Off by default (routing_defaults, the config, the CLI), and the flag
     survives manifest_to_plan.
  8. End to end on esp_prog (#1235's own repro): no off-centre via end with
     the flag (the collapse keeps the centre stubs), the nets recorded for
     later steps, every net connected and check_drc / check_weird clean in
     both arms.

    python3 tests/test_1235_via_centre_at_source.py
"""
import os
import re
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from kicad_parser import BoardInfo                             # noqa: E402
from routing_config import GridCoord, GridRouteConfig          # noqa: E402
import routing_defaults                                        # noqa: E402
import single_ended_routing as ser                             # noqa: E402
from pcb_modification import _via_drag_ends                    # noqa: E402
from run_utils import evidence                                 # noqa: E402
from synth import make_net, make_pcb, make_seg, make_via       # noqa: E402

failures = []
SITE = (1.0, 0.05)          # an unblock's fine-pass via site, off cell (10, 0)
LAYERS = ['F.Cu', 'B.Cu']


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}" + (f": {detail}" if detail and not ok else ''))
    if not ok:
        failures.append(label)


def near(p, q, tol=1e-6):
    return abs(p[0] - q[0]) <= tol and abs(p[1] - q[1]) <= tol


def cfg(clearance=0.1):
    return GridRouteConfig(clearance=clearance, track_width=0.2, via_size=0.5,
                           via_drill=0.3, layers=list(LAYERS), grid_step=0.1,
                           via_centre_ends=True)


def pcb(segments=(), vias=(), site=True, copper=None):
    info = BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'},
                     copper_layers=list(copper or LAYERS),
                     board_bounds=(-5, -5, 5, 5))
    p = make_pcb(segments=list(segments), vias=list(vias),
                 nets={1: make_net(1, '/A'), 2: make_net(2, '/B')},
                 board_info=info)
    if site:
        p._unblock_via_sites = {(1, 10, 0): SITE}
        p._unblock_via_sizes = {(1, 10, 0): (0.5, 0.3)}
    return p


def layer_change_path():
    path = [(0, 0, 0), (10, 0, 0), (10, 0, 1), (20, 0, 1)]
    coord = GridCoord(0.1)
    return path, [coord.to_float(g[0], g[1]) for g in path], coord


print("1. a layer change is laid on its barrel's centre")
path, pts, coord = layer_change_path()
board = pcb()
sites = ser._snap_layer_changes_to_barrels(path, pts, coord, LAYERS, 1, cfg(),
                                           board, set(), {}, reuse=True)
check("both legs meet the unblock site", near(pts[1], SITE) and near(pts[2], SITE),
      f"{pts[1]} {pts[2]}")
check("the snap hands back the barrel it centred on, keyed by cell",
      (10, 0, 0, 1) in sites and near(sites[(10, 0, 0, 1)][:2], SITE),
      str(sites))
check("the path's ends are not moved", near(pts[0], (0, 0)) and near(pts[3], (2, 0)))

print("2. refused where a leg into the centre would graze; merged ends stay")
# foreign copper above the via: the grid legs (y=0) clear it by 0.16, a leg
# into the centre (y=0.05) by only 0.11 -- under the 0.13 clearance
foreign = make_seg(0.8, 0.36, 1.2, 0.36, net_id=2, width=0.2)
path, pts, coord = layer_change_path()
ser._snap_layer_changes_to_barrels(path, pts, coord, LAYERS, 1, cfg(0.13),
                                   pcb([foreign]), set(), {}, reuse=True)
check("a grazing leg keeps the grid point", near(pts[1], (1.0, 0.0)), str(pts[1]))
path = [(10, 0, 0), (10, 0, 1), (20, 0, 1)]
pts = [(1.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
ser._snap_layer_changes_to_barrels(path, pts, coord, LAYERS, 1, cfg(), pcb(),
                                   set(), {}, reuse=True, frozen={0})
check("a merged terminal is not moved", near(pts[0], (1.0, 0.0)), str(pts[0]))


def end_change(foreign_items):
    """A layer change at the path's FIRST point, its pad's own point 0.2 mm
    beside the cell: the conversion lays the bridge pad -> pts[0]."""
    path = [(10, 0, 0), (10, 0, 1), (20, 0, 1)]
    pts = [(1.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    ser._snap_layer_changes_to_barrels(
        path, pts, GridCoord(0.1), LAYERS, 1, cfg(0.13), pcb(foreign_items),
        set(), {}, reuse=True, originals={0: (0.8, 0.0, 'F.Cu'), 2: None})
    return pts


# foreign copper above the pad -> via run: the bridge to the grid point (y=0)
# clears it, the bridge to the unblock site (y=0.05) does not
check("a change at a path end whose pad bridge would graze keeps the grid point",
      near(end_change([make_seg(0.7, 0.36, 0.9, 0.36, net_id=2, width=0.2)])[0],
           (1.0, 0.0)))
check("...and moves onto the barrel's centre when that bridge clears",
      near(end_change([])[0], SITE))

print("3. a path through a via's disc passes its centre")
via = make_via(*SITE, size=0.5)
path = [(0, 0, 0), (10, 0, 0), (10, 10, 0)]
pts = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]
p2, q2 = ser._route_through_via_centres(path, pts, LAYERS, 1, cfg(),
                                        pcb(vias=[via], site=False))
check("the bend in the disc is the via's centre", near(q2[1], SITE), str(q2))
inner = make_via(*SITE, size=0.5, layers=('In1.Cu', 'B.Cu'), via_type='blind')
p3, q3 = ser._route_through_via_centres(
    path, list(pts), ['F.Cu', 'In1.Cu', 'B.Cu'], 1, cfg(),
    pcb(vias=[inner], site=False, copper=['F.Cu', 'In1.Cu', 'B.Cu']))
check("a via with no copper on the path's layer is left alone",
      near(q3[1], (1.0, 0.0)), str(q3))

print("4. tap points inside a via are owned by its centre")
seg = make_seg(0.0, 0.0, 2.0, 0.0)
rows_on = ser.get_all_segment_tap_points([seg], GridCoord(0.1), LAYERS,
                                         vias=[via], centre_vias=True)
rows_off = ser.get_all_segment_tap_points([seg], GridCoord(0.1), LAYERS,
                                          vias=[via])
own_on = {(r[0], r[1], r[2]): (r[3], r[4]) for r in rows_on}
own_off = {(r[0], r[1], r[2]): (r[3], r[4]) for r in rows_off}
check("with centre_vias the disc's samples weld to the centre",
      near(own_on[(11, 0, 0)], SITE) and near(own_on[(9, 0, 0)], SITE),
      f"{own_on.get((11, 0, 0))} {own_on.get((9, 0, 0))}")
check("without it they stay the segment's own points",
      near(own_off[(11, 0, 0)], (1.1, 0.0)), str(own_off.get((11, 0, 0))))
check("a sample outside the disc is untouched",
      near(own_on[(15, 0, 0)], (1.5, 0.0)), str(own_on.get((15, 0, 0))))

print("5. a path end on an unblock via's cell moves onto its site")
path = [(10, 0, 1), (20, 0, 1)]
pts = [(1.0, 0.0), (2.0, 0.0)]
ser._snap_terminals_to_vias(path, pts, LAYERS, 1, cfg(), pcb(),
                            {0: None, 1: (2.0, 0.0, 'B.Cu')})
check("the end is the registered site", near(pts[0], SITE), str(pts[0]))
check("the far end, welded to its own row, stays", near(pts[1], (2.0, 0.0)))
pts = [(1.0, 0.0), (2.0, 0.0)]
ser._snap_terminals_to_vias(path, pts, LAYERS, 1, cfg(0.13),
                            pcb([make_seg(0.8, 0.36, 1.2, 0.36, net_id=2,
                                          width=0.2, layer='B.Cu')]),
                            {0: None, 1: None})
check("a grazing leg keeps the end on the grid point", near(pts[0], (1.0, 0.0)),
      str(pts[0]))

print("6. a nudged via takes the run's tracks at its centre along")
v = make_via(1.0, 0.0)
run_seg = make_seg(0.0, 0.0, 1.0, 0.0)
board = pcb([run_seg], [v], site=False)
drag = _via_drag_ends(v, [run_seg], 1.02, 0.0, {id(run_seg)}, board, 0.1)
check("the run's track end follows", drag == [(run_seg, 1)], str(drag))
check("input copper keeps the via put",
      _via_drag_ends(v, [run_seg], 1.02, 0.0, set(), board, 0.1) is None)

print("6b. close_soft_joints bridges every net's end to the via centre")
from pcb_modification import close_soft_joints                  # noqa: E402


def bridges(flag):
    track = make_seg(0.0, 0.0, 0.95, 0.0)          # ends 0.05 beside the centre
    foreign = make_seg(0.6, 0.24, 0.8, 0.24, net_id=2, width=0.1)  # beside the rim
    board = pcb([track, foreign], [make_via(1.0, 0.0, size=0.6)], site=False)
    c = cfg(0.1)
    c.via_centre_ends = flag
    res = [{'new_segments': [track], 'new_vias': []}]
    close_soft_joints(res, board, None, c)
    return [(round(b.start_x, 3), round(b.start_y, 3), round(b.end_x, 3), round(b.end_y, 3))
            for r in res if r.get('cleanup') == 'via_centre_bridge'
            for b in r['new_segments']], track


got, track = bridges(True)
check("with the flag the end is bridged from where it lies, inside the disc, "
      "when the rim cut would graze", got == [(0.95, 0.0, 1.0, 0.0)], str(got))
check("the track keeps its end", near((track.end_x, track.end_y), (0.95, 0.0)))
got, _ = bridges(False)
check("without it a net no length grade reads is not bridged", got == [], str(got))


def tight_bridge(foreign_items, gate=0.25):
    """An end laid tighter than the pass's clearance (an escalated rescue):
    via r 0.15, a 0.0889 track ending 0.13 beside the centre."""
    track = make_seg(0.0, 0.0, 0.87, 0.0, width=0.0889)
    segs = [track] + [x for x in foreign_items if hasattr(x, 'start_x')]
    vias = [make_via(1.0, 0.0, size=0.3, drill=0.15)] + \
        [x for x in foreign_items if not hasattr(x, 'start_x')]
    board = pcb(segs, vias, site=False)
    c = cfg(gate)
    res = [{'new_segments': [track], 'new_vias': []}]
    close_soft_joints(res, board, None, c)
    return [(round(b.start_x, 3), round(b.end_x, 3)) for r in res
            if r.get('cleanup') == 'via_centre_bridge' for b in r['new_segments']]


got = tight_bridge([make_seg(0.8, -0.25, 1.2, -0.25, net_id=2, width=0.1)])
check("a bridge no closer to the foreign track than its end is laid",
      got == [(0.87, 1.0)], str(got))
got = tight_bridge([make_via(1.05, -0.4, net_id=2, size=0.2, drill=0.1)], gate=0.3)
check("a bridge that would come closer to foreign copper than its end is not",
      got == [], str(got))

print("7. off by default, and the flag survives manifest_to_plan")
check("routing_defaults.VIA_CENTRE_ENDS is off", routing_defaults.VIA_CENTRE_ENDS is False)
check("GridRouteConfig defaults it off", GridRouteConfig().via_centre_ends is False)
sys.path.insert(0, os.path.join(TESTS_DIR, 'stress'))
import manifest_to_plan as m2p                                 # noqa: E402
check("--via-centre-ends maps onto the Route tab's checkbox",
      m2p.BOOL_FLAGS.get('--via-centre-ends') == 'via_centre_ends')
check("--no-via-centre-ends unticks it",
      m2p.INVERTED_BOOL_FLAGS.get('--no-via-centre-ends') == 'via_centre_ends')

print("8. esp_prog end to end (#1235's repro)")


def weird(board_path):
    """(off-centre via ends, check_weird's counted findings)."""
    out = subprocess.run([sys.executable, os.path.join(ROOT, 'py_router', 'check_weird.py'),
                          board_path, '--max-print', '0'],
                         capture_output=True, text=True, timeout=600).stdout
    m = re.search(r'off-centre-via-end x(\d+)', out)
    f = re.search(r'FOUND (\d+) WEIRD THINGS', out)
    return (int(m.group(1)) if m else 0), (int(f.group(1)) if f else 0)


with tempfile.TemporaryDirectory() as tmp:
    counts = {}
    for arm in ('--no-via-centre-ends', '--via-centre-ends'):
        out = os.path.join(tmp, f'esp{arm}.kicad_pcb')
        r = subprocess.run([sys.executable, os.path.join(ROOT, 'py_router', 'route.py'),
                            os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb'), out,
                            '--nets', '*', arm], capture_output=True, text=True, timeout=1200)
        check(f"route.py {arm} exits 0", r.returncode == 0, r.stderr[-400:])
        evidence(out, 'routed board')
        conn = subprocess.run([sys.executable, os.path.join(ROOT, 'py_router', 'check_connected.py'), out],
                              capture_output=True, text=True, timeout=600)
        check(f"{arm}: every net connected",
              conn.returncode == 0 and 'ALL NETS FULLY CONNECTED' in conn.stdout,
              conn.stdout[-300:])
        drc = subprocess.run([sys.executable, os.path.join(ROOT, 'py_router', 'check_drc.py'), out],
                             capture_output=True, text=True, timeout=600)
        check(f"{arm}: check_drc clean", drc.returncode == 0, drc.stdout[-300:])
        counts[arm], found = weird(out)
        check(f"{arm}: check_weird finds nothing to remove", found == 0, str(found))
        if arm == '--via-centre-ends':
            sys.path.insert(0, os.path.join(ROOT, 'py_router'))
            from kicad_parser import parse_kicad_pcb                  # noqa: E402
            from protected_nets import read_via_centre_nets_for_pcb_data  # noqa: E402
            rec = read_via_centre_nets_for_pcb_data(parse_kicad_pcb(out))
            check("the step records the nets it laid with centre stubs",
                  '/+3.3V' in rec and '/DTR' in rec, str(sorted(rec)[:6]))
    print(f"     off-centre via ends: {counts}")
    check("the flag leaves no off-centre via end on esp_prog",
          counts['--via-centre-ends'] == 0 < counts['--no-via-centre-ends'], str(counts))

if failures:
    print(f"\nFAIL: {len(failures)} check(s): {failures}")
    sys.exit(1)
print("\nPASS: test_1235_via_centre_at_source")
