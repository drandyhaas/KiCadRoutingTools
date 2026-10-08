#!/usr/bin/env python3
"""#1218: route_diff spaces a pair's own P/N transition vias at the PAIR's
class clearance.

Every intra-pair via spacing -- the via-to-via distance and the via-to-partner-
track offset, in the via placer (`_process_via_positions`, `_pair_via_offset`,
`_min_via_center_distance`) and in what the pose router is told
(`_try_route_direction`, `_route_direct_coupled_middle`) -- was priced at the
run's base clearance, the Default class. check_drc grades P against N at
max(Default, class), so a pair in a 0.25 class over a 0.13 Default landed its
vias 0.58 apart (0.45 + 0.13) where the class asks 0.70: 3 violations, all P/N.

The board is #1207's (tests/test_1207_gnd_vias_pair_class.py): a pair from an
F.Cu part to a B.Cu part, so it must change layers. Its class is set to 0.25
over a 0.13 Default, track 0.12, gap 0.154, no GND vias.

Checks:
  1. The spacing helpers price the pair: 0.70 via-to-via, not 0.58, and an
     offset of at least half of it; with no class map they are the base.
  2. route_diff routes the pair across layers, its P/N vias sit at least
     0.70 apart, and check_drc reports no violation between P and N.
  3. Control: the same board with the class AT the Default (0.13) still lands
     its vias at the base spacing, 0.58 -- a board that declares no wider
     class is unchanged.

    python3 tests/test_1218_pair_via_spacing_class.py
"""
import contextlib
import importlib.util
import io
import json
import math
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from kicad_parser import parse_kicad_pcb                       # noqa: E402

BASE, CLASS, VIA, DRILL, TRACK, GAP = 0.13, 0.25, 0.45, 0.3, 0.12, 0.154
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def _fixture():
    spec = importlib.util.spec_from_file_location(
        't1207', os.path.join(ROOT, 'tests', 'test_1207_gnd_vias_pair_class.py'))
    t = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(t)
    return t.BOARD, t.PROJECT


def _route(d, cls):
    board, proj = _fixture()
    proj = json.loads(json.dumps(proj))
    for c in proj['net_settings']['classes']:
        c.update(track_width=TRACK, diff_pair_width=TRACK, diff_pair_gap=GAP,
                 clearance=BASE if c['name'] == 'Default' else cls)
    src, out = os.path.join(d, 'b.kicad_pcb'), os.path.join(d, 'out.kicad_pcb')
    with open(src, 'w') as f:
        f.write(board)
    with open(os.path.join(d, 'b.kicad_pro'), 'w') as f:
        json.dump(proj, f, indent=2)
    r = subprocess.run(
        [sys.executable, '-X', 'utf8', 'py_router/route_diff.py', src,
         '--nets', '/D_*', '--layers', 'F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu',
         '--track-width', str(TRACK), '--diff-pair-gap', str(GAP),
         '--clearance', str(BASE), '--via-size', str(VIA),
         '--via-drill', str(DRILL), '--no-gnd-vias', '--output', out],
        cwd=ROOT, capture_output=True, text=True, encoding='utf-8',
        errors='replace')
    return out, r.stdout + r.stderr


def _pn_vias(out):
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(out)
    p = [v for v in pcb.vias if v.net_id == 2]
    n = [v for v in pcb.vias if v.net_id == 3]
    gaps = [math.hypot(a.x - b.x, a.y - b.y) for a in p for b in n]
    return p, n, gaps


def main():
    # 1. The helpers.
    from routing_config import GridRouteConfig
    from diff_pair_routing import _min_via_center_distance, _pair_via_offset
    cfg = GridRouteConfig(clearance=BASE, track_width=TRACK, via_size=VIA,
                          via_drill=DRILL, net_clearances={2: CLASS, 3: CLASS})
    c2c = _min_via_center_distance(cfg, 2, 3)
    check('via-to-via is priced at the pair class', abs(c2c - (VIA + CLASS)) < 1e-9,
          f'{c2c:.3f}, want {VIA + CLASS:.3f}')
    off = _pair_via_offset(cfg, (TRACK + GAP) / 2, 2, 3)
    check('the P/N via offset is at least half of it', off >= (VIA + CLASS) / 2 - 1e-9,
          f'{off:.3f}')
    flat = GridRouteConfig(clearance=BASE, track_width=TRACK, via_size=VIA,
                           via_drill=DRILL)
    check('with no class map the spacing is the base',
          abs(_min_via_center_distance(flat, 2, 3) - (VIA + BASE)) < 1e-9
          and abs(_min_via_center_distance(flat) - (VIA + BASE)) < 1e-9)

    # 2. The route.
    with tempfile.TemporaryDirectory(prefix='t1218_') as d:
        out, log = _route(d, CLASS)
        check('route_diff routes the pair', 'Diff pairs:    1/1 routed' in log,
              '' if 'Diff pairs:    1/1 routed' in log else log[-400:])
        if not os.path.isfile(out):
            print('FAILED: no output board')
            return 1
        p, n, gaps = _pn_vias(out)
        check('precondition: the pair changed layers through P and N vias',
              bool(p) and bool(n), f'{len(p)} P vias, {len(n)} N vias')
        check('the P/N vias sit at least via + class apart',
              bool(gaps) and min(gaps) >= VIA + CLASS - 1e-4,
              f'min {min(gaps):.4f}, want {VIA + CLASS:.3f}' if gaps else '')
        r = subprocess.run([sys.executable, '-X', 'utf8', 'py_router/check_drc.py',
                            out], cwd=ROOT, capture_output=True, text=True,
                           encoding='utf-8', errors='replace')
        bad = [l.strip() for l in r.stdout.splitlines()
               if '/D_P <-> /D_N' in l or ('/D_' in l and '<->' in l)
               or ('Via:/D_' in l and 'Seg:/D_' in l)]
        ok = not bad and r.returncode == 0
        check('check_drc reports no P <-> N violation', ok,
              '' if ok else ('; '.join(bad[:4]) or r.stdout[-300:]))

    # 3. Control: the class at the base.
    with tempfile.TemporaryDirectory(prefix='t1218c_') as d:
        out, log = _route(d, BASE)
        _p, _n, gaps = _pn_vias(out) if os.path.isfile(out) else ([], [], [])
        check('control: a class at the Default keeps the base spacing (0.58)',
              bool(gaps) and abs(min(gaps) - (VIA + BASE)) < 1e-3,
              f'min {min(gaps):.4f}' if gaps else log[-300:])

    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
