#!/usr/bin/env python3
"""#1628: two pads of ONE footprint closer than their clearance are carried.

check_drc does not grade such a pair (#1306: library geometry no placement or
routing step changes), but KiCad reports each one as a `clearance` error, and
the count stopped at one check_drc console line: board_score and
check_complete read clean, nothing said it before routing, and
kicad_drc_compare called KiCad's items check_drc false negatives. Now each
pair is an accepted `own-footprint-gap` row (named under WARNINGS with its
gap, counted in --json as `own_footprint_gaps`), board_score carries
`drc.library_pad_gaps` and prints it (Step 0 runs it before routing),
check_complete discloses it, and kicad_drc_compare gives it its own channel.
Still not graded: exit 0, `violations` 0, `blocking` unchanged.

The board is one part with pads 1|2 at a 0.150 mm gap and 3|4 at 0.145 mm,
each to a partner pad, class 0.16. Control: a project that sets `clearance`
to ignore (KiCad runs no clearance check) publishes none.

    python3 tests/test_1628_library_pad_gaps.py
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)
import run_utils  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
FAILS = []


def ok(name, cond, detail=''):
    print(('  [ok] ' if cond else '  [FAIL] ') + name
          + ('' if cond else f': {detail}'))
    if not cond:
        FAILS.append(name)


def build(d, sev=None):
    os.makedirs(d, exist_ok=True)
    pcb = os.path.join(d, 'b.kicad_pcb')

    def pad(n, x, y, net, name):
        return (f'    (pad "{n}" smd rect (at {x} {y}) (size 0.3 0.8) '
                f'(layers "F.Cu") (net {net} "{name}"))\n')

    def head(ref, x, y):
        return (f'  (footprint "T:{ref}" (layer "F.Cu") (at {x} {y})\n'
                f'    (property "Reference" "{ref}" (at 0 -2 0) (layer "F.SilkS") '
                f'(hide yes) (effects (font (size 1 1) (thickness 0.15))))\n'
                f'    (attr smd)\n')
    b = ('(kicad_pcb (version 20241229) (generator "pcbnew") (generator_version "9.0")\n'
         ' (general (thickness 1.6))\n (paper "A4")\n'
         ' (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))\n'
         ' (setup (pad_to_mask_clearance 0))\n'
         ' (net 0 "") (net 1 "/A") (net 2 "/B") (net 3 "/C") (net 4 "/D")\n')
    b += (head('U1', 120, 110) + pad(1, 0, 0, 1, '/A') + pad(2, 0.45, 0, 2, '/B')
          + pad(3, 2, 0, 3, '/C') + pad(4, 2.445, 0, 4, '/D') + '  )\n')
    for i, (n, name, px) in enumerate(((1, '/A', 110), (2, '/B', 116),
                                       (3, '/C', 124), (4, '/D', 130))):
        b += head(f'J{i + 1}', px, 115) + pad(1, 0, 0, n, name) + '  )\n'
    for x1, y1, x2, y2 in ((100, 100, 150, 100), (150, 100, 150, 125),
                           (150, 125, 100, 125), (100, 125, 100, 100)):
        b += (f'  (gr_line (start {x1} {y1}) (end {x2} {y2}) (layer "Edge.Cuts") '
              f'(stroke (width 0.05) (type solid)))\n')
    with open(pcb, 'w', encoding='utf-8') as f:
        f.write(b + ')\n')
    cl = dict(name='Default', clearance=0.16, track_width=0.15, via_diameter=0.6,
              via_drill=0.3, wire_width=6, bus_width=12, line_style=0,
              priority=2147483647)
    with open(pcb[:-len('.kicad_pcb')] + '.kicad_pro', 'w', encoding='utf-8') as f:
        json.dump({'board': {'design_settings': {
                       'rules': {'min_clearance': 0.1, 'min_track_width': 0.1},
                       'rule_severities': dict(sev or {})}},
                   'meta': {'filename': 'b.kicad_pro', 'version': 3},
                   'net_settings': {'classes': [cl], 'meta': {'version': 4}}},
                  f, indent=1)
    return pcb


def check_drc(pcb):
    js = pcb + '.drc.json'
    r = run_utils.check(PY + [os.path.join(REPO, 'py_router', 'check_drc.py'), pcb,
                              '--json', js], accept=True)
    with open(run_utils.evidence(js, 'check_drc --json'), encoding='utf-8') as f:
        return json.load(f), r.stdout


def main():
    tmp = tempfile.mkdtemp(prefix='t1628_')
    try:
        pcb = build(os.path.join(tmp, 'gaps'))
        doc, out = check_drc(pcb)
        ok('not graded: no violation', doc['violations'] == 0, doc['violations'])
        ok('--json counts both pairs', doc.get('own_footprint_gaps') == 2,
           doc.get('own_footprint_gaps'))
        rows = sorted((i['pad_ref'], i['pad_ref2'], i['gap_mm'], i['kicad_severity'])
                      for i in doc['items'] if i.get('accepted') == 'own-footprint-gap')
        ok('each pair is an accepted row with its gap and KiCad severity',
           rows == [('U1.1', 'U1.2', 0.15, 'error'), ('U1.3', 'U1.4', 0.145, 'error')],
           rows)
        ok('WARNINGS name the pairs and the gaps',
           'LIBRARY PAD GAPS: 2 pair(s)' in out
           and 'U1.1 [/A] | U1.2 [/B] on F.Cu: gap 0.1500mm < 0.1600mm' in out
           and 'U1.3 [/C] | U1.4 [/D] on F.Cu: gap 0.1450mm < 0.1600mm' in out,
           out[-1200:])

        sj = pcb + '.score.json'
        r = subprocess.run(PY + [os.path.join(REPO, 'py_tools', 'board_score.py'), pcb,
                                 '--json', sj], capture_output=True, text=True,
                           encoding='utf-8', errors='replace', cwd=REPO, timeout=600)
        acc = run_utils._accident((r.stdout or '') + (r.stderr or ''))
        if acc:
            raise AssertionError(f'BROKEN TEST: board_score died from {acc!r}\n'
                                 + (r.stdout or '')[-800:])
        with open(run_utils.evidence(sj, 'board_score --json'), encoding='utf-8') as f:
            s = json.load(f)
        ok('board_score: drc stays 0 (disclosure, not a block)',
           s['blocking_by']['drc'] == 0, s['blocking_by'])
        ok('board_score: drc.library_pad_gaps carries the count',
           s['components']['drc'].get('library_pad_gaps') == 2, s['components']['drc'])
        ok('board_score: says it on the unrouted board (Step 0)',
           'LIBRARY PAD GAPS (disclosed, not blocking): KiCad will report 2 ' in r.stdout,
           r.stdout[-800:])

        pcb_i = build(os.path.join(tmp, 'ignored'), {'clearance': 'ignore'})
        doc, _out = check_drc(pcb_i)
        ok('control: a project that ignores clearance publishes none',
           not doc.get('own_footprint_gaps'), doc.get('own_footprint_gaps'))

        from kicad_oracle import find_kicad_cli
        if find_kicad_cli(warn=False):
            sys.path.insert(0, os.path.join(REPO, 'tests', 'stress'))
            import kicad_drc_compare as kdc
            data = kdc.compare_board_data(pcb)
            ok('kicad_drc_compare: KiCad\'s two items land in their own channel',
               data and data.get('kicad_library_pad_gaps') == 2
               and data.get('checkdrc_library_pad_gaps') == 2
               and data.get('kicad_only') == 0,
               {k: (data or {}).get(k) for k in ('kicad', 'kicad_only',
                                                 'kicad_library_pad_gaps',
                                                 'checkdrc_library_pad_gaps')})
        else:
            print('  (kicad-cli not found: the comparator half is not run)')
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    if FAILS:
        print(f'FAILED: {len(FAILS)}: {FAILS}')
        return 1
    print('PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
