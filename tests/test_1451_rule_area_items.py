#!/usr/bin/env python3
"""#1451: a pad or footprint inside a rule area that forbids it.

A rule area's `(pads not_allowed)` / `(footprints not_allowed)` -- an RF
module's own antenna keep-out, or a board-level one -- is what KiCad's DRC
reports as `items_not_allowed`. The parser never read either setting, so a cap
moved into a module's antenna area read `legal` in place_pose and `buildable`
in check_assembly, and place_simple walked parts into it.

  1. the parser reads `pads_allowed`, `footprints_allowed` and the area's
     `owner` (the footprint it sits in, '' at board level);
  2. `legality.rule_area_item_findings` counts what kicad-cli 10.0.6 counts
     on eight probes (measured on that version: 10.0.0 enforces no rule
     area at all) -- a part inside, one straddling the edge, one clear, the
     owner's own pad inside its area, a courtyard reaching in with the pads
     out, a part with no courtyard, a back-side part under a front-only area,
     and the input;
  3. place_pose refuses the issue's move and names the pair;
  4. check_assembly gates an item the --baseline did not have, reports
     without one, and does not gate an item the baseline had;
  5. place_simple keeps a part out of a locked owner's area even when its
     wiring pulls it across.

    python3 tests/test_1451_rule_area_items.py
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

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, os.path.join(REPO, 'py_placer'))
sys.path.insert(0, TESTS)

import run_utils  # noqa: E402

RUN_ALL_TIMEOUT = 600
FIXTURE = os.path.join(TESTS, 'fixtures', '1451', 'antenna_keepout.kicad_pcb')
AREA = '(xy 120 112) (xy 132 112) (xy 132 118) (xy 120 118)'
C1_AT = '(at 105 125)'
C1_CRTYD = ('\t\t(fp_rect (start -1.48 -0.73) (end 1.48 0.73) (stroke (width 0.05) '
            '(type solid)) (fill no) (layer "F.CrtYd"))\n')
#: kicad-cli 10.0.6's items_not_allowed on each probe, item by item
KICAD = {
    'inside': ['C1 footprint', 'C1 pad 1', 'C1 pad 2'],
    'edge': ['C1 footprint', 'C1 pad 1', 'C1 pad 2'],
    'clear': [],
    'ownpad': ['U1 pad 2'],
    'courtonly': ['C1 footprint'],
    'nocourt': ['C1 pad 1', 'C1 pad 2'],
    'backside': [],
    'input': [],
}
J1 = ('\t(footprint "repro:Conn_2" (locked yes) (layer "F.Cu") (at 136 115)\n'
      '\t\t(property "Reference" "J1" (at 0 -3 0) (layer "F.SilkS"))\n'
      '\t\t(fp_rect (start -1.5 -2.5) (end 1.5 2.5) (stroke (width 0.05) (type solid)) '
      '(fill no) (layer "F.CrtYd"))\n'
      '\t\t(pad "1" smd rect (at 0 -1.27) (size 1.5 1.5) (layers "F.Cu" "F.Mask" '
      '"F.Paste") (net 2 "+3V3"))\n'
      '\t\t(pad "2" smd rect (at 0 1.27) (size 1.5 1.5) (layers "F.Cu" "F.Mask" '
      '"F.Paste") (net 1 "GND")))\n')
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def parse(path):
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stdout(io.StringIO()):
        return parse_kicad_pcb(path)


def probes(tmp, base):
    """{name: path} of the eight probe boards, each one edit of the input."""
    def put(name, text):
        p = os.path.join(tmp, name + '.kicad_pcb')
        with open(p, 'w', encoding='utf-8') as f:
            f.write(text)
        return p

    def c1_at(text, x, y):
        assert text.count(C1_AT) == 1
        return text.replace(C1_AT, f'(at {x} {y})')
    back = base.replace('(31 "F.CrtYd" user "F.Courtyard"))',
                        '(31 "F.CrtYd" user "F.Courtyard") (3 "B.Mask" user) '
                        '(15 "B.Paste" user) (7 "B.SilkS" user "B.Silkscreen") '
                        '(29 "B.CrtYd" user "B.Courtyard"))')
    back = back.replace('(zone (layers "F.Cu" "B.Cu")', '(zone (layers "F.Cu")')
    i = back.index('(footprint "repro:C_0603"')
    blk = back[i:].replace('(layer "F.Cu") ' + C1_AT, '(layer "B.Cu") (at 126 115)', 1)
    blk = (blk.replace('"F.Cu" "F.Mask" "F.Paste"', '"B.Cu" "B.Mask" "B.Paste"')
           .replace('"F.CrtYd"', '"B.CrtYd"').replace('"F.SilkS"', '"B.SilkS"'))
    assert AREA in base and C1_CRTYD in base
    return {
        'inside': put('inside', c1_at(base, 126, 115)),
        'edge': put('edge', c1_at(base, 120.5, 115)),
        'clear': put('clear', c1_at(base, 105, 125.0)),
        'ownpad': put('ownpad', base.replace(AREA, AREA.replace('120', '117'))),
        'courtonly': put('courtonly', c1_at(base, 118.7, 115)),
        'nocourt': put('nocourt', c1_at(base.replace(C1_CRTYD, ''), 126, 115)),
        'backside': put('backside', back[:i] + blk),
        'input': put('input', base),
    }


def main():
    from placement.legality import rule_area_item_findings
    run_utils.evidence(FIXTURE, 'the #1451 repro board')
    tmp = tempfile.mkdtemp(prefix='t1451_')
    try:
        base = open(FIXTURE, encoding='utf-8').read()
        P = probes(tmp, base)

        # 1. the parser
        ko = parse(P['input']).board_info.keepouts
        check('one rule area, read with its pads/footprints settings and owner',
              len(ko) == 1 and ko[0].get('pads_allowed') is False
              and ko[0].get('footprints_allowed') is False and ko[0].get('owner') == 'U1',
              str([{k: a.get(k) for k in ('owner', 'pads_allowed', 'footprints_allowed')}
                   for a in ko]))
        board_level = base.replace(
            '\t(footprint "repro:Module_Antenna"', '\t(zone (layers "F.Cu") (name "B") '
            '(hatch full 0.5) (keepout (tracks allowed) (vias allowed) (pads not_allowed) '
            '(copperpour allowed) (footprints allowed)) (polygon (pts (xy 101 101) '
            '(xy 103 101) (xy 103 103))))\n\t(footprint "repro:Module_Antenna"', 1)
        bp = os.path.join(tmp, 'board_level.kicad_pcb')
        with open(bp, 'w', encoding='utf-8') as f:
            f.write(board_level)
        board = [k for k in parse(bp).board_info.keepouts if k['name'] == 'B']
        check('a board-level area: no owner, pads forbidden, footprints allowed',
              len(board) == 1 and board[0].get('owner') == ''
              and board[0].get('pads_allowed') is False
              and board[0].get('footprints_allowed') is True, str(board and {
                  k: board[0].get(k) for k in ('owner', 'pads_allowed', 'footprints_allowed')}))

        # 2. the count, against kicad-cli 10.0.6
        for name, want in KICAD.items():
            got = rule_area_item_findings(parse(P[name]), P[name])
            items = sorted(f"{r[2]} {r[3]}" for r in got['rule_area_items'])
            check(f'{name}: as kicad-cli 10.0.6 counts', items == sorted(want),
                  f'got {items}, kicad {sorted(want)}')

        # 3. place_pose refuses the move
        out = os.path.join(tmp, 'moved.kicad_pcb')
        js = os.path.join(tmp, 'pose.json')
        r = subprocess.run([sys.executable, '-X', 'utf8', 'py_placer/place_pose.py', P['input'],
                            out, 'set', 'C1', '126', '115', '--json', js], cwd=REPO,
                           capture_output=True, text=True, encoding='utf-8', errors='replace')
        d = json.load(open(js)) if os.path.isfile(js) else {}
        check('place_pose refuses a cap into the antenna area, naming the pair',
              r.returncode == 4 and not os.path.exists(out)
              and 'C1/Antenna Keep Out of U1' in (r.stdout + r.stderr)
              and d.get('rule_area_item_count_after') == 3,
              f"rc {r.returncode}, {d.get('rule_area_item_count_after')}")

        # 4. check_assembly
        def verdict(board_path, baseline=None):
            argv = [sys.executable, '-X', 'utf8', 'py_tools/check_assembly.py', board_path]
            if baseline:
                argv += ['--baseline', baseline]
            r = subprocess.run(argv, cwd=REPO, capture_output=True, text=True,
                               encoding='utf-8', errors='replace')
            m = re.search(r'VERDICT: (.*)', r.stdout)
            return r.returncode, (m.group(1) if m else r.stdout[-300:])
        rc, v = verdict(P['inside'], P['input'])
        check('check_assembly --baseline: a new item is NOT BUILDABLE',
              rc == 4 and v.startswith('NOT BUILDABLE'), f'rc {rc}: {v}')
        rc, v = verdict(P['inside'])
        check('...and without a baseline it is report-only',
              rc == 0 and v.startswith('buildable'), f'rc {rc}: {v}')
        rc, v = verdict(P['ownpad'], P['ownpad'])
        check("...and an item the baseline had (the owner's own pad) does not gate",
              rc == 0 and v.startswith('buildable'), f'rc {rc}: {v}')

        # 5. place_simple keeps the cap out though its wiring pulls it across
        i = base.index('\t(footprint "repro:C_0603"')
        pull = os.path.join(tmp, 'pull.kicad_pcb')
        with open(pull, 'w', encoding='utf-8') as f:
            f.write(base[:i] + J1 + base[i:])
        intent = os.path.join(tmp, 'intent.json')
        subprocess.run([sys.executable, '-X', 'utf8', 'py_tools/check_floorplan.py', pull,
                        '--emit-intent', intent, '--allow-unplaced'], cwd=REPO,
                       capture_output=True)
        run_utils.evidence(intent, 'the pull board intent')
        groups = os.path.join(tmp, 'groups.json')
        with open(groups, 'w') as f:
            json.dump({'schema': 1, 'groups': [{'name': 'g', 'refs': ['C1'], 'why': 'test'}]}, f)
        bad = []
        for seed in (0, 3):
            placed = os.path.join(tmp, f'placed_s{seed}.kicad_pcb')
            subprocess.run([sys.executable, '-X', 'utf8', 'py_placer/place_simple.py', pull,
                            placed, '--groups', groups, '--intent', intent,
                            '--seed', str(seed)], cwd=REPO, capture_output=True)
            run_utils.evidence(placed, f'the seed {seed} placement')
            # judged on the geometry, not the grade, so the check stands on
            # its own: C1's courtyard (+-1.48 x +-0.73) against the area
            c1 = parse(placed).footprints['C1']
            hw, hh = (1.48, 0.73) if round(c1.rotation or 0) % 180 == 0 else (0.73, 1.48)
            if c1.x + hw > 120 and c1.x - hw < 132 and c1.y + hh > 112 and c1.y - hh < 118:
                bad.append((seed, round(c1.x, 2), round(c1.y, 2)))
        check('place_simple keeps C1 out of the locked module\'s area (seeds 0, 3; '
              'main put it inside on both)', not bad, str(bad))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
