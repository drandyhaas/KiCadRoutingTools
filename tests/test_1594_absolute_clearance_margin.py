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

#1803 rows: a default run COUNTS the shortfalls it accepted above KiCad's
500 nm (6 here: 600, 4000, 24000 nm on HV and 600, 4000, 9000 nm on Default)
in one console line and the JSON's `within_margin`, with the grade, the exit
status and run_drc's return unchanged; at `--clearance-margin-mm 0.0005` the
count is 0 and no line is printed.

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
        rows, out = drc_rows(board)
        check('the 5% default is unchanged', rows == AT_5PCT and len(rows) == 2,
              str(rows))
        # #1803: what the 5% accepted that KiCad reports is counted, not graded
        acc = re.findall(r'^Accepted within the tolerance: (\d+) copper clearance '
                         r'shortfall\(s\) above KiCad\'s 0\.0005 mm.*the largest '
                         r'([0-9.]+) mm, (\S+) <-> (\S+)\); --clearance-margin-mm '
                         r'0\.0005 lists them$', out, re.M)
        want = len(set(KICAD) - set(AT_5PCT))
        check(f'#1803: the default run counts the {want} shortfalls it accepted above '
              f'500 nm, the largest HV 24 um', want == 6 and acc ==
              [('6', '0.0240', 'HV_24000nm_a', 'HV_24000nm_b')], str(acc))
        rows, out = drc_rows(board, '--clearance-margin-mm', '0.0005')
        check("--clearance-margin-mm 0.0005 reports exactly KiCad's pairs, on "
              "both classes", rows == KICAD and len(rows) == 8, str(rows))
        check('...and says which tolerance it graded at',
              'Clearance tolerance: 0.0005 mm on every pair' in out, out[:600])
        check('#1803: ...and accepts nothing above it, so prints no count',
              'Accepted within the tolerance' not in out)
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
        doc = json.load(open(evidence(js, 'check_drc --json')))
        ga = doc['graded_at']
        check('the JSON records the absolute margin',
              ga.get('clearance_margin_mm') == 0.0005, str(ga)[:200])
        check('#1803: ...and within_margin.count 0',
              (doc.get('within_margin') or {}).get('count') == 0,
              str(doc.get('within_margin')))
        js5 = os.path.join(tmp, 'drc5.json')
        run_check([sys.executable, '-X', 'utf8',
                   os.path.join(ROOT, 'py_router', 'check_drc.py'), board,
                   '--no-size-checks', '--quiet', '--json', js5],
                  refuse='FAILED', code=1)
        doc5 = json.load(open(evidence(js5, 'check_drc --json')))
        wm = doc5.get('within_margin') or {}
        check('#1803: the default JSON: within_margin 6, the largest 24 um on HV, '
              'and violations still 2',
              wm.get('count') == 6 and abs((wm.get('max_mm') or 0) - 0.024) < 1e-9
              and wm.get('max_nets') == ['HV_24000nm_a', 'HV_24000nm_b']
              and doc5.get('violations') == 2, f"{wm} / {doc5.get('violations')}")

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
        import check_drc as _cd
        rec, rec_abs = {}, {}
        with contextlib.redirect_stdout(io.StringIO()):
            c = run_drc(board, within_margin=rec, **kw)
            run_drc(board, clearance_margin_mm=0.0005, within_margin=rec_abs, **kw)
        check('#1803: counting changes nothing run_drc returns',
              [(v['type'], v['net1'], v['net2'], v.get('overlap_mm')) for v in c]
              == [(v['type'], v['net1'], v['net2'], v.get('overlap_mm')) for v in b],
              f'{len(c)} vs {len(b)}')
        check('#1803: in-process: 6 accepted at the fraction, 0 at the absolute margin',
              rec.get('count') == 6 and rec_abs.get('count') == 0, f'{rec} / {rec_abs}')
        check('#1803: the count is restored after the call',
              _cd._WITHIN_MARGIN.get() is None, str(_cd._WITHIN_MARGIN.get()))

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
