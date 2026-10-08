#!/usr/bin/env python3
"""#1200: a tool writes a copper keep-out rule area, and the router keeps out.

One-Air-Max's ESP32-C6 antenna needs a copper keep-out on every layer. The
toolchain honours a KiCad rule area, but nothing could create one (the only
writer was route_planes' pour-only NPTH keep-out), and on a board placed from
scratch the area has to follow the module's final pose.

Checks (tracked esp_prog):
  1. kicad_writer: the pour-only keep-out still forbids copper pour alone;
     generate_rule_area_sexpr forbids what it is asked to and refuses a flag
     KiCad does not have.
  2. add_rule_area --ref U2: the area is in U2's frame as the file stores it,
     the frame of its pads -- a rect around pad 1's local position contains
     its global one. The parser reads it back on every copper layer with
     tracks, vias and copper pour not allowed. A re-run replaces it.
  3. Refusals, by reason: a --ref the board lacks, a layer it lacks.
  4. route.py routes Net-(R3-Pad2) through a 3 x 2 mm box without the area
     (non-vacuity) and around it with the area, still routed.
  5. With kicad-cli present: KiCad reports items_not_allowed for that free
     route once the area is added -- the rule area is real to KiCad.

    python3 tests/test_1200_add_rule_area.py
"""
import contextlib
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from copy_board import copy_board                              # noqa: E402
from kicad_parser import parse_kicad_pcb                       # noqa: E402
import run_utils                                               # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
NET = 'Net-(R3-Pad2)'
BOX = (132.0, 99.0, 135.0, 101.0)
KICAD_CLI = '/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli'
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def parse(path):
    with contextlib.redirect_stdout(io.StringIO()), \
            contextlib.redirect_stderr(io.StringIO()):
        return parse_kicad_pcb(path)


def tool(*argv):
    return subprocess.run([sys.executable, '-X', 'utf8', 'py_router/add_rule_area.py']
                          + [str(a) for a in argv], cwd=ROOT, capture_output=True,
                          text=True, encoding='utf-8', errors='replace')


def in_box(x, y):
    return BOX[0] < x < BOX[2] and BOX[1] < y < BOX[3]


def net_copper_in_box(path):
    pcb = parse(path)
    nid = next(i for i, n in pcb.nets.items() if n.name == NET)
    segs = [s for s in pcb.segments if s.net_id == nid]
    hits = sum(1 for s in segs if any(
        in_box(s.start_x + t / 40 * (s.end_x - s.start_x),
               s.start_y + t / 40 * (s.end_y - s.start_y)) for t in range(41)))
    return len(segs), hits


def main():
    # 1. The writer.
    from kicad_writer import generate_keepout_zone_sexpr, generate_rule_area_sexpr
    pour = generate_keepout_zone_sexpr(['F.Cu'], [(0, 0), (1, 0), (1, 1)], 'p')
    check('the NPTH keep-out still forbids copper pour alone',
          '(copperpour not_allowed)' in pour and '(tracks allowed)' in pour
          and '(vias allowed)' in pour)
    area = generate_rule_area_sexpr(['F.Cu'], [(0, 0), (1, 0), (1, 1)], 'a')
    check('a rule area forbids tracks, vias and copper pour by default',
          all(f'({k} not_allowed)' in area for k in ('tracks', 'vias', 'copperpour'))
          and '(pads allowed)' in area)
    try:
        generate_rule_area_sexpr(['F.Cu'], [(0, 0), (1, 0), (1, 1)], 'a',
                                 not_allowed=('wires',))
        check('an unknown flag is refused', False)
    except ValueError as exc:
        check('an unknown flag is refused', 'wires' in str(exc), str(exc))

    tmp = tempfile.mkdtemp(prefix='t1200_')
    try:
        # 2. --ref, read-back, replace.
        pcb = parse(BOARD)
        u2 = pcb.footprints['U2']
        p1 = next(p for p in u2.pads if p.pad_number == '1')
        out = os.path.join(tmp, 'a.kicad_pcb')
        r = tool(BOARD, out, '--name', 'ANT_KEEPOUT', '--ref', 'U2', '--rect',
                 p1.local_x - 0.2, p1.local_y - 0.2, p1.local_x + 0.2, p1.local_y + 0.2)
        check('add_rule_area says what it wrote',
              r.returncode == 0 and "wrote rule area 'ANT_KEEPOUT'" in r.stdout
              and "U2's local frame" in r.stdout, (r.stdout + r.stderr)[-300:])
        run_utils.evidence(out, 'the written board')
        kos = [k for k in parse(out).board_info.keepouts]
        k = kos[0] if kos else {}
        poly = k.get('polygon') or []
        xs, ys = [x for x, _ in poly], [y for _, y in poly]
        check("the area sits on pad 1 (the pads' frame)",
              bool(poly) and min(xs) < p1.global_x < max(xs)
              and min(ys) < p1.global_y < max(ys),
              f'pad1 ({p1.global_x}, {p1.global_y}) bbox '
              f'{(min(xs), min(ys), max(xs), max(ys)) if poly else None}')
        check('read back on every copper layer, tracks/vias/pour not allowed',
              len(kos) == 1 and set(k.get('layers') or ()) == set(pcb.board_info.copper_layers)
              and k.get('tracks_allowed') is False and k.get('vias_allowed') is False
              and k.get('copper_pour_allowed') is False, json.dumps(
                  {kk: (sorted(v) if isinstance(v, set) else v) for kk, v in k.items()
                   if kk != 'polygon' and kk != 'holes'}, default=str))
        r = tool(out, out, '--name', 'ANT_KEEPOUT', '--ref', 'U2', '--rect', -3, -3, 3, 3)
        check('a re-run replaces the area of that name',
              'replaced 1 earlier area' in r.stdout
              and len(parse(out).board_info.keepouts) == 1, r.stdout[-200:])

        # 3. Refusals.
        run_utils.check([sys.executable, '-X', 'utf8', 'py_router/add_rule_area.py',
                         BOARD, os.path.join(tmp, 'x.kicad_pcb'), '--name', 'k',
                         '--ref', 'U99', '--rect', '0', '0', '1', '1'],
                        refuse='U99 names no footprint', code=2, cwd=ROOT)
        run_utils.check([sys.executable, '-X', 'utf8', 'py_router/add_rule_area.py',
                         BOARD, os.path.join(tmp, 'x.kicad_pcb'), '--name', 'k',
                         '--layers', 'In7.Cu', '--rect', '0', '0', '1', '1'],
                        refuse='not a copper layer', code=2, cwd=ROOT)

        # 4. The router keeps out.
        res = {}
        for arm in ('free', 'ko'):
            inp = os.path.join(tmp, f'in_{arm}.kicad_pcb')
            with contextlib.redirect_stdout(io.StringIO()), \
                    contextlib.redirect_stderr(io.StringIO()):
                copy_board(BOARD, inp)
            if arm == 'ko':
                tool(inp, inp, '--name', 'KO', '--rect', *BOX)
            rout = os.path.join(tmp, f'out_{arm}.kicad_pcb')
            r = subprocess.run([sys.executable, '-X', 'utf8', 'py_router/route.py',
                                inp, rout, '--nets', NET], cwd=ROOT,
                               capture_output=True, text=True, encoding='utf-8',
                               errors='replace')
            line = next((l for l in r.stdout.splitlines()
                         if l.startswith('JSON_SUMMARY_MIN:')), '')
            routed = json.loads(line.split(':', 1)[1]).get('routed') if line else None
            res[arm] = (routed,) + net_copper_in_box(rout)
        check('without the area the route crosses the box (non-vacuity)',
              res['free'][0] == 1 and res['free'][2] >= 1, str(res['free']))
        check('with the area the net routes around it',
              res['ko'][0] == 1 and res['ko'][1] > 0 and res['ko'][2] == 0, str(res['ko']))

        # 5. KiCad agrees the area is real.
        if os.path.isfile(KICAD_CLI):
            fpk = os.path.join(tmp, 'free_ko.kicad_pcb')
            tool(os.path.join(tmp, 'out_free.kicad_pcb'), fpk, '--name', 'KO',
                 '--rect', *BOX)
            rep = os.path.join(tmp, 'drc.json')
            subprocess.run([KICAD_CLI, 'pcb', 'drc', '--format', 'json',
                            '--severity-all', '-o', rep, fpk], capture_output=True)
            n = sum(1 for v in json.load(open(rep)).get('violations', [])
                    if v.get('type') == 'items_not_allowed') if os.path.isfile(rep) else 0
            check('KiCad reports the free route inside the area as not allowed',
                  n >= 1, str(n))
        else:
            print('  [skip] kicad-cli not present: KiCad cross-check not run')
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
