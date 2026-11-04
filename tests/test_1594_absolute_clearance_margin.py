#!/usr/bin/env python3
"""#1594: an absolute clearance tolerance, so check_drc can grade as KiCad does.

check_drc's tolerance was only fractional (`--clearance-margin`, default 5% of
each pair's clearance), while KiCad's is an absolute 500 nm on every pair, so
on a board with two classes no setting matched KiCad: 5% is 25 um at 0.5 mm
and 10 um at 0.2 mm. kicad_drc_compare graded its check_drc side at ONE clearance while KiCad
graded every class, so it listed pairs check_drc's own CLI reports as
KICAD-ONLY.

Gated here, on parallel track pairs each short of its class (HV 0.5 mm,
Default 0.2 mm) by a known number of nanometres:

  1. `--clearance-margin-mm 0.0005` reports exactly the pairs KiCad reports
     (short by more than 500 nm), on both classes; the 5% default is
     unchanged; the two flags refuse each other; a negative value is refused.
  2. run_drc's `clearance_margin_mm` holds for that call only.
  3. kicad_drc_compare's check_drc side grades the classes KiCad grades (with
     kicad-cli: the pairs past check_drc's 5% margin match, and only the ones
     inside it are KICAD-ONLY).

    python3 tests/test_1594_absolute_clearance_margin.py
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
sys.path.insert(0, ROOT)
sys.path.insert(0, TESTS_DIR)
sys.path.insert(0, os.path.join(TESTS_DIR, 'stress'))
for _p in ('py_router', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from run_utils import check as run_check, evidence             # noqa: E402

failures = []

ROWS = [('HV', 0.5, nm) for nm in (400, 600, 4000, 24000, 26000)] + \
       [('DEF', 0.2, nm) for nm in (400, 600, 4000, 9000, 11000)]
W = 0.2
KICAD = sorted(f'{c}_{nm}nm' for c, _, nm in ROWS if nm > 500)     # 500 nm
AT_5PCT = sorted(f'{c}_{nm}nm' for c, clr, nm in ROWS if nm * 1e-6 > clr * 0.05)


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def build(out):
    nets, tracks = [], []
    for i, (cls, clr, nm) in enumerate(ROWS):
        y = 2 + i * 3.0
        for side, yy in (('a', y), ('b', y + W + clr - nm * 1e-6)):
            nets.append(f'{cls}_{nm}nm_{side}')
            tracks.append(f' (segment (start 5 {yy:.6f}) (end 35 {yy:.6f}) (width {W})'
                          f' (layer "F.Cu") (net {len(nets)})'
                          f' (uuid "00000000-0000-0000-0000-{len(nets):012d}"))')
    head = ['(kicad_pcb (version 20241229) (generator "t") (generator_version "9.0")'
            ' (general (thickness 1.6)) (paper "A4")',
            ' (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))',
            ' (setup (pad_to_mask_clearance 0))', ' (net 0 "")']
    head += [f' (net {i + 1} "{n}")' for i, n in enumerate(nets)]
    head.append(' (gr_rect (start 0 0) (end 40 40) (stroke (width 0.1) (type default))'
                ' (fill none) (layer "Edge.Cuts") (uuid "00000000-0000-0000-0001-000000000001"))')
    with open(out, 'w', encoding='utf-8') as fh:
        fh.write('\n'.join(head + tracks + [')']) + '\n')

    def cls(name, clr):
        return {'name': name, 'clearance': clr, 'track_width': W, 'via_diameter': 0.6,
                'via_drill': 0.3, 'microvia_diameter': 0.3, 'microvia_drill': 0.1,
                'diff_pair_width': 0.2, 'diff_pair_gap': 0.25, 'diff_pair_via_gap': 0.25,
                'priority': 0 if name == 'HV' else 2147483647, 'wire_width': 6,
                'bus_width': 12, 'line_style': 0, 'schematic_color': 'rgba(0,0,0,0)',
                'pcb_color': 'rgba(0,0,0,0)'}
    with open(out[:-len('kicad_pcb')] + 'kicad_pro', 'w', encoding='utf-8') as fh:
        json.dump({'board': {'design_settings': {'rules': {
                       'min_clearance': 0.0, 'min_copper_edge_clearance': 0.0,
                       'min_track_width': 0.0}}},
                   'meta': {'filename': os.path.basename(out)[:-len('kicad_pcb')]
                            + 'kicad_pro', 'version': 3},
                   'net_settings': {'classes': [cls('Default', 0.2), cls('HV', 0.5)],
                                    'meta': {'version': 3}, 'net_colors': None,
                                    'netclass_assignments': None,
                                    'netclass_patterns': [{'netclass': 'HV',
                                                           'pattern': 'HV_*'}]}},
                  fh, indent=1)
    return out


def drc_rows(board, *flags):
    r = run_check([sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_router', 'check_drc.py'),
                   board, '--no-size-checks', '--max-print', '0', *flags],
                  refuse='DRC VIOLATIONS', code=1)
    return sorted(set(re.findall(r'^\s*((?:HV|DEF)_\d+nm)_a <->', r.stdout, re.M))), r.stdout


def main():
    tmp = tempfile.mkdtemp(prefix='t1594_')
    try:
        board = build(os.path.join(tmp, 'band.kicad_pcb'))

        print('1. the CLI')
        rows, _ = drc_rows(board)
        check('the 5% default is unchanged', rows == AT_5PCT and len(rows) == 2,
              str(rows))
        rows, out = drc_rows(board, '--clearance-margin-mm', '0.0005')
        check("--clearance-margin-mm 0.0005 reports exactly KiCad's pairs, on "
              "both classes", rows == KICAD and len(rows) == 8, str(rows))
        check('...and says which tolerance it graded at',
              'Clearance tolerance: 0.0005 mm on every pair' in out, out[:600])
        run_check([sys.executable, '-X', 'utf8',
                   os.path.join(ROOT, 'py_router', 'check_drc.py'), board,
                   '--clearance-margin', '0.1', '--clearance-margin-mm', '0.0005'],
                  refuse='not allowed with argument', code=2,
                  allow=('error: argument',))
        run_check([sys.executable, '-X', 'utf8',
                   os.path.join(ROOT, 'py_router', 'check_drc.py'), board,
                   '--clearance-margin-mm', '-1'],
                  refuse='must be a non-negative number', code=2, allow=('usage:',))
        js = os.path.join(tmp, 'drc.json')
        run_check([sys.executable, '-X', 'utf8',
                   os.path.join(ROOT, 'py_router', 'check_drc.py'), board,
                   '--no-size-checks', '--quiet', '--clearance-margin-mm', '0.0005',
                   '--json', js], refuse='FAILED', code=1)
        ga = json.load(open(evidence(js, 'check_drc --json')))['graded_at']
        check('the JSON records the absolute margin',
              ga.get('clearance_margin_mm') == 0.0005, str(ga)[:200])

        print('2. run_drc holds the margin for that call only')
        from check_drc import run_drc, _ABS_MARGIN_MM
        from list_nets import read_design_rules, net_clearance_map
        from kicad_parser import parse_kicad_pcb
        ncl = net_clearance_map(board, [n.name for n in parse_kicad_pcb(board).nets.values()],
                                rules=read_design_rules(board))
        kw = dict(clearance=0.2, quiet=True, print_summary=False, check_sizes=False,
                  net_clearances=ncl)
        with contextlib.redirect_stdout(io.StringIO()):
            a = run_drc(board, clearance_margin_mm=0.0005, **kw)
            b = run_drc(board, **kw)
        names = lambda vs: sorted({v['net1'][:-2] for v in vs})   # noqa: E731
        check('in-process: 8 rows at the absolute margin, then 2 at the fraction',
              names(a) == KICAD and names(b) == AT_5PCT,
              f'{names(a)} / {names(b)}')
        check('the margin is restored after the call', _ABS_MARGIN_MM.get() is None,
              str(_ABS_MARGIN_MM.get()))

        print('3. kicad_drc_compare')
        import kicad_drc_compare as K
        cd = K.run_check_drc(board, 0.2)
        cd_nets = sorted({sorted(v['nets'])[0][:-2] for v in cd})
        check("its check_drc side grades the classes KiCad grades",
              cd_nets == AT_5PCT, str(cd_nets))
        from kicad_oracle import find_kicad_cli
        cli = find_kicad_cli(warn=False)
        if cli:
            K.KICAD_CLI = cli
            data = K.compare_board_data(board)
            check('with kicad-cli: 8 KiCad items, the 2 past the 5% margin matched, '
                  'the 6 inside it KICAD-ONLY (no class pair lost to one clearance)',
                  data and data.get('kicad') == 8 and data.get('check_drc') == 2
                  and data.get('matched') == 2 and data.get('kicad_only') == 6,
                  str({k: (data or {}).get(k) for k in
                       ('kicad', 'check_drc', 'matched', 'kicad_only', 'verdict')}))
        else:
            print('  [skip] kicad-cli not installed: the real comparison')
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
