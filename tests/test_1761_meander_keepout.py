#!/usr/bin/env python3
"""Meander bumps keep out of keep-outs (#1761).

The length matcher's bump amplitude searches checked foreign copper and the
board outline, never rule areas, so `route_diff --diff-pair-intra-match` laid a
bump inside a `tracks not_allowed` area the routed pair itself kept out of.
Both searches now ask the shared `check_drc.copper_keepout_gate` (#1760) at the
bump's copper and take a smaller amplitude when it refuses:

  1. get_safe_amplitude_at_point (single-ended, and the intra-pair member
     meanders): toward the area the amplitude drops until the arm's copper
     stops at the area's edge (no clearance margin); away from it, and with
     no area, it stays at the maximum;
  2. get_safe_amplitude_for_diff_pair (a pair's centreline meander) likewise,
     judged at the pair's outer copper;
  3. the single-ended `--length-match-groups` path (apply_meanders_to_route):
     the same request lays bumps into the area without it (non-vacuity) and
     none with it, and still adds bumps;
  4. end to end, the issue's lvds_converter_dualclk /CLK repro: bumps are
     added, check_drc finds no keep-out copper, and the summary's
     `rule_area_keepouts` is [].

    python3 tests/test_1761_meander_keepout.py
"""
import contextlib
import io
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from kicad_parser import parse_kicad_pcb, Segment                # noqa: E402
from routing_config import GridRouteConfig                       # noqa: E402
from check_drc import keepout_census                             # noqa: E402
from length_matching import (apply_meanders_to_route,            # noqa: E402
                             get_safe_amplitude_at_point,
                             get_safe_amplitude_for_diff_pair)
import run_utils                                                 # noqa: E402

LVDS = os.path.join(ROOT, 'kicad_files', 'lvds_converter_dualclk.kicad_pcb')
EMPTY = ('(kicad_pcb (version 20260206) (generator "pcbnew") (generator_version "10.0") '
         '(general (thickness 1.6)) (layers (0 "F.Cu" signal) (2 "B.Cu" signal) '
         '(25 "Edge.Cuts" user)) (net 0 "") (net 1 "A") (net 2 "B") '
         '(gr_rect (start 0 0) (end 60 40) (layer "Edge.Cuts") '
         '(stroke (width 0.1) (type default)) (fill no)))\n')
EDGE = 20.6          # the area's near edge; the trace runs along y = 20
failures = []


def ok(label, cond, detail=''):
    print(f"  [{'ok' if cond else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not cond:
        failures.append(label)


def py(*argv):
    r = subprocess.run([sys.executable, '-B', '-X', 'utf8', *map(str, argv)], cwd=ROOT,
                       capture_output=True, text=True, encoding='utf-8', errors='replace')
    out = r.stdout + r.stderr
    if 'Traceback' in out:
        raise AssertionError(f'BROKEN TEST: {argv[0]} crashed:\n{out[-1500:]}')
    return r.returncode, out


def parse(path):
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        return parse_kicad_pcb(path)


def boards(tmp):
    e = os.path.join(tmp, 'e.kicad_pcb')
    open(e, 'w', encoding='utf-8').write(EMPTY)
    k = os.path.join(tmp, 'k.kicad_pcb')
    rc, o = py('py_router/add_rule_area.py', e, k, '--name', 'KO', '--layers', 'F.Cu',
               '--rect', 8, EDGE, 30, 23, '--forbid', 'tracks', 'vias')
    if rc != 0:
        raise AssertionError(f'add_rule_area failed: {o[-400:]}')
    run_utils.evidence(k, 'board with the area')
    pe, pk = parse(e), parse(k)
    if len(pk.board_info.keepouts) != 1:
        raise AssertionError('BROKEN TEST: the area did not parse')
    return pe, pk


def amplitude_checks(pe, pk, cfg):
    print('1. get_safe_amplitude_at_point')
    args = (20, 20, 1, 0, 0, 1)
    free = get_safe_amplitude_at_point(*args, 1, 2.0, 0.1, 'F.Cu', pe, 1, cfg)
    ok('no area: the maximum', abs(free - 2.0) < 1e-9, f'{free:.3f}')
    toward = get_safe_amplitude_at_point(*args, 1, 2.0, 0.1, 'F.Cu', pk, 1, cfg)
    ok('toward the area: copper stops at its edge',
       0 < toward and 20 + toward + 0.1 <= EDGE + 1e-6, f'{toward:.3f}')
    away = get_safe_amplitude_at_point(*args, -1, 2.0, 0.1, 'F.Cu', pk, 1, cfg)
    ok('away from the area: the maximum', abs(away - 2.0) < 1e-9, f'{away:.3f}')
    print('2. get_safe_amplitude_for_diff_pair')
    sp = 0.18
    free = get_safe_amplitude_for_diff_pair(*args, 1, 2.0, 0.1, 0, pe, 1, 2, cfg, sp)
    ok('no area: the maximum', abs(free - 2.0) < 1e-9, f'{free:.3f}')
    toward = get_safe_amplitude_for_diff_pair(*args, 1, 2.0, 0.1, 0, pk, 1, 2, cfg, sp)
    ok('toward the area: the outer copper stops at its edge',
       0 <= toward and 20 + toward + sp + 0.1 <= EDGE + 1e-6, f'{toward:.3f}')
    away = get_safe_amplitude_for_diff_pair(*args, -1, 2.0, 0.1, 0, pk, 1, 2, cfg, sp)
    ok('away from the area: the maximum', abs(away - 2.0) < 1e-9, f'{away:.3f}')


def single_ended_path(pe, pk, cfg):
    print('3. apply_meanders_to_route (the --length-match-groups path)')
    got = {}
    for name, pd in (('no area', pe), ('area', pk)):
        segs = [Segment(start_x=10, start_y=20, end_x=50, end_y=20, width=0.2,
                        layer='F.Cu', net_id=1)]
        out, n = apply_meanders_to_route(segs, 6.0, cfg, pcb_data=pd, net_id=1)
        got[name] = (n, len(keepout_census(pk, ['F.Cu', 'B.Cu'], segments=out)))
    ok('control: without the area the bumps cross it', got['no area'][0] > 0
       and got['no area'][1] > 0, repr(got['no area']))
    ok('with the area: bumps added, none inside it', got['area'][0] > 0
       and got['area'][1] == 0, repr(got['area']))


def end_to_end(tmp):
    print('4. end to end: route_diff --diff-pair-intra-match (lvds /CLK)')
    a = os.path.join(tmp, 'a.kicad_pcb')
    py('py_router/copy_board.py', LVDS, a)
    run_utils.evidence(a, 'copied board')
    k = os.path.join(tmp, 'lk.kicad_pcb')
    rc, o = py('py_router/add_rule_area.py', a, k, '--name', 'MOUNT_X',
               '--rect', 85.0, 59.75, 86.5, 60.6, '--forbid', 'tracks', 'vias')
    run_utils.evidence(k, 'lvds board with the area')
    out = os.path.join(tmp, 'r_match.kicad_pcb')
    rc, log = py('py_router/route_diff.py', k, out, '--nets', '/CLK*', '--track-width', 0.15,
                 '--diff-pair-gap', 0.21, '--clearance', 0.2, '--diff-pair-intra-match', '-v')
    ok('route_diff exit 0', rc == 0, f'exit {rc}')
    run_utils.evidence(out, 'routed board')
    m = re.search(r'intra-pair: (\d+) bumps added, new P=[\d.]+mm, new delta=([\d.]+)mm', log)
    ok('bumps added and the pair matched', bool(m) and int(m.group(1)) > 0
       and float(m.group(2)) < 0.05, m.group(0) if m else 'no bumps line')
    rc, o = py('py_router/check_drc.py', out, '--clearance', 0.2)
    ok('check_drc: no keep-out copper', 'KEEPOUT' not in o,
       '' if 'KEEPOUT' not in o else o[o.find('KEEPOUT'):][:300])
    m = re.search(r'^JSON_SUMMARY: (.*)$', log, re.M)
    s = json.loads(m.group(1)) if m else {}
    ok("summary: rule_area_keepouts == []", s.get('rule_area_keepouts') == [],
       repr(s.get('rule_area_keepouts', '<absent>')))


def main():
    tmp = tempfile.mkdtemp(prefix='t1761_')
    try:
        pe, pk = boards(tmp)
        cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'], track_width=0.2, clearance=0.2)
        amplitude_checks(pe, pk, cfg)
        single_ended_path(pe, pk, cfg)
        end_to_end(tmp)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    if failures:
        print(f'FAILED: {len(failures)}: {failures}')
        return 1
    print('PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
