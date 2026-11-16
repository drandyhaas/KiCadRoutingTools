#!/usr/bin/env python3
"""#1925 piece 2: check_via_stubs.py measures the stub each via leaves.

The board is tests/test_1925_backdrill_read_keep.py's fixture: six copper
layers with a stackup whose surface depths are known, and one via per net,
each joined a different way. The expected lengths below are sums of that
stackup, worked by hand:

  /A  tracks on F.Cu and In1.Cu, no back-drill: nothing above, and the whole
      barrel below In1.Cu, 1.310 - 0.170 = 1.140
  /B  the same with a back-drill B.Cu..In3.Cu: the barrel now ends at the top
      of In3.Cu, 0.905 - 0.170 = 0.735
  /C  tracks on In2.Cu and In3.Cu, drilled from both faces: 0.370 - 0.170 and
      1.140 - 0.940, 0.200 each
  /D  tracks on F.Cu and B.Cu: no stub
  /E  joined on In3.Cu and B.Cu, which its own back-drill removes: named, and
      not measured
  /F  in an F.Cu pad of its net, leaving on In2.Cu: 1.310 - 0.405 = 0.905
  /G  a track on F.Cu and an In4.Cu pour of its net: 1.310 - 1.175 = 0.135
  /H  a blind via F.Cu-In2.Cu with tracks on F.Cu and In1.Cu: its own barrel
      ends at In2.Cu, 0.405 - 0.170 = 0.235
  /I  no copper at all: joined on no layer

  1. `via_stub_report` gives those lengths, layers and flags;
  2. removing /B's back-drill gives /A's length back (the drill is what the
     difference measures);
  3. the CLI prints them, writes the same report as JSON, names the vias over
     --max-stub and changes no byte of the board;
  4. a --nets pattern naming no net is named and the exit is 2; a board with
     no stackup is NOT GRADED in mm and still lists the layers.

Run with:  python3 tests/test_1925_via_stubs.py
"""
import json
import os
import shutil
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, 'py_router'))

import run_utils  # noqa: E402
import test_1925_backdrill_read_keep as fx  # noqa: E402

PY = sys.executable
TOOL = run_utils.tool('check_via_stubs.py')
FAILS = []

#: net: (stub_top_mm, stub_bottom_mm, joined layers)
WANT = {
    '/A': (0.0, 1.140, ['F.Cu', 'In1.Cu']),
    '/B': (0.0, 0.735, ['F.Cu', 'In1.Cu']),
    '/C': (0.200, 0.200, ['In2.Cu', 'In3.Cu']),
    '/D': (0.0, 0.0, ['F.Cu', 'B.Cu']),
    '/E': (None, None, ['In3.Cu', 'B.Cu']),
    '/F': (0.0, 0.905, ['F.Cu', 'In2.Cu']),
    '/G': (0.0, 0.135, ['F.Cu', 'In4.Cu']),
    '/H': (0.0, 0.235, ['F.Cu', 'In1.Cu']),
    '/I': (None, None, []),
}


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def near(a, b):
    return (a is None and b is None) or (a is not None and b is not None
                                         and abs(a - b) < 1e-6)


def rows_match(rows, want, label):
    by = {r['net']: r for r in rows}
    ok(set(by) == set(want), f"{label}: one row per via, {sorted(by)}")
    for net, (top, bottom, joined) in want.items():
        r = by.get(net)
        if r is None:
            continue
        ok(near(r['stub_top_mm'], top) and near(r['stub_bottom_mm'], bottom)
           and r['joined'] == joined,
           f"{label}: {net} top {r['stub_top_mm']} bottom {r['stub_bottom_mm']} "
           f"joined {r['joined']} (want {top}, {bottom}, {joined})")
    return by


def engine(td):
    print('1-2. the measure')
    from via_stubs import copper_depths, via_stub_report
    pcb, _ = fx.parse(fx.write_board(os.path.join(td, 'e.kicad_pcb')))
    depths = copper_depths(pcb)
    ok(abs(depths['In3.Cu'][0] - 0.905) < 1e-9 and abs(depths['B.Cu'][1] - 1.310) < 1e-9,
       f"the stackup gives the depths the lengths are worked from: In3.Cu "
       f"{depths['In3.Cu']}, B.Cu {depths['B.Cu']}")
    rep = via_stub_report(pcb, max_stub=0.5)
    by = rows_match(rep['vias'], WANT, 'engine')
    ok(by['/F']['joined_by'] == {'F.Cu': ['pad'], 'In2.Cu': ['track']}
       and by['/G']['joined_by'] == {'F.Cu': ['track'], 'In4.Cu': ['zone']},
       f"a pad and a pour join a via as a track does: {by['/F']['joined_by']}, "
       f"{by['/G']['joined_by']}")
    ok(by['/E']['joined_on_drilled'] == ['In3.Cu', 'B.Cu'] and not by['/E']['unjoined']
       and by['/I']['unjoined'] and not by['/I']['joined_on_drilled'],
       "a via joined on a layer its back-drill removes, and one joined on none, "
       "are told apart")
    ok(by['/B']['drilled'] == ['In3.Cu', 'In4.Cu', 'B.Cu']
       and by['/B']['stub_bottom_layers'] == ['In2.Cu'],
       f"/B's drill removes its end layer too, and the stub left passes "
       f"{by['/B']['stub_bottom_layers']}")
    ok(by['/H']['span'] == ['F.Cu', 'In2.Cu'] and by['/H']['via_type'] == 'blind',
       "a blind via is measured to its own end layer")
    over = sorted(r['net'] for r in rep['vias'] if r['over'])
    ok(over == ['/A', '/B', '/F'] and rep['over'] == 3,
       f"over 0.5 mm: {over}")
    ok(all(r['over'] is None for r in rep['vias']
           if r['net'] in ('/E', '/I')),
       'a via with no measured stub is neither over nor under')
    ok(rep['by_net']['/A'] == {'vias': 1, 'longest_mm': 1.14, 'over': 1,
                               'unjoined': 0, 'joined_on_drilled': 0},
       f"the longest stub per net: {rep['by_net']['/A']}")
    ok(rep['measured'] == 7 and rep['unmeasured'] == 2 and rep['backdrilled_vias'] == 3
       and rep['reason'] is None, 'the counts')
    ok(sorted((p['ref'], p['pad']) for p in rep['pad_backdrills']) == [('J1', '1'), ('J1', '2')],
       f"the drilled pads that declare a back-drill are listed: "
       f"{[(p['ref'], p['pad']) for p in rep['pad_backdrills']]}")
    none = via_stub_report(pcb)
    ok(none['over'] is None and all(r['over'] is None for r in none['vias']),
       'with no threshold nothing is called over')

    # Non-vacuity: the back-drill is what /B's shorter stub measures.
    vias = [v if v[0] != '/B' else ('/B', '', '', v[3], v[4]) for v in fx.VIAS]
    bare, _ = fx.parse(fx.write_board(os.path.join(td, 'e2.kicad_pcb'), vias=vias))
    b = {r['net']: r for r in via_stub_report(bare)['vias']}['/B']
    ok(near(b['stub_bottom_mm'], 1.140) and not b['backdrills'],
       f"with /B's back-drill removed its stub is /A's: {b['stub_bottom_mm']}")
    only = via_stub_report(pcb, net_ids={n for n, net in pcb.nets.items()
                                         if net.name == '/C'})
    ok([r['net'] for r in only['vias']] == ['/C'] and not only['pad_backdrills'],
       'net_ids limits the report to those nets')


def cli(td):
    print('3-4. the CLI')
    board = fx.write_board(os.path.join(td, 'c.kicad_pcb'))
    with open(board, 'rb') as fh:
        before = fh.read()
    js = os.path.join(td, 'c.json')
    r = run_utils.check([PY, TOOL, board, '--max-stub', '0.5', '--json', js], accept=True)
    with open(board, 'rb') as fh:
        ok(fh.read() == before and sorted(os.listdir(td)) == [
            'c.json', 'c.kicad_pcb', 'e.kicad_pcb', 'e2.kicad_pcb'],
           f'the board is byte for byte what it was, and only the report was '
           f'written: {sorted(os.listdir(td))}')
    with open(run_utils.evidence(js, 'check_via_stubs JSON'), encoding='utf-8') as fh:
        doc = json.load(fh)
    rows_match(doc['vias'], WANT, 'JSON')
    ok(doc['max_stub_mm'] == 0.5 and doc['over'] == 3 and doc['board'] == os.path.abspath(board)
       and doc['patterns_matching_no_net'] == [],
       'the JSON carries the threshold, the count over it and the board')
    out = r.stdout
    ok('Via stubs: 7 of 9 via(s) measured, 3 back-drilled, 3 over 0.5 mm' in out,
       'the headline: ' + out.splitlines()[0])
    ok('/A: 1 via(s), longest stub 1.140 mm, 1 over' in out
       and 'top 0.000 mm, bottom 0.735 mm  [back-drilled B.Cu..In3.Cu at 0.5; OVER]' in out
       and 'top 0.200 mm, bottom 0.200 mm  [back-drilled F.Cu..In1.Cu at 0.45, '
           'B.Cu..In4.Cu at 0.5]' in out,
       'each via is printed with both stubs and its back-drills')
    ok('JOINED ON A BACK-DRILLED LAYER: In3.Cu, B.Cu' in out and 'joined on no layer' in out
       and 'Drilled pads declaring a back-drill: 2' in out,
       'the faults and the back-drilled pads are named')
    ok(out.index('/A:') < out.index('/F:') < out.index('/B:') < out.index('/D:'),
       'nets are listed longest stub first')

    r = run_utils.check([PY, TOOL, board, '--nets', '/C', '/ZZ*'],
                        refuse="--nets pattern(s) matching no net of the board: '/ZZ*'",
                        code=2)
    ok('/C: 1 via(s)' in r.stdout and '/A:' not in r.stdout,
       'a pattern naming no net is named, exit 2, and the others still report')

    bare = fx.write_board(os.path.join(td, 'n.kicad_pcb'), stackup=False)
    js2 = os.path.join(td, 'n.json')
    r = run_utils.check([PY, TOOL, bare, '--max-stub', '0.5', '--json', js2], accept=True)
    with open(run_utils.evidence(js2, 'check_via_stubs JSON'), encoding='utf-8') as fh:
        doc = json.load(fh)
    b = {x['net']: x for x in doc['vias']}['/B']
    ok('Via stubs NOT GRADED in mm: the board has no stackup' in r.stdout
       and doc['measured'] == 0 and doc['over'] == 0 and b['stub_bottom_mm'] is None
       and b['stub_bottom_layers'] == ['In2.Cu'] and b['over'] is None,
       'a board with no stackup is NOT GRADED in mm, and the layers each stub '
       'passes are still listed')

    run_utils.check([PY, TOOL, os.path.join(td, 'absent.kicad_pcb')],
                    refuse='board not found', code=3)
    ok(True, 'a missing board is refused by name, exit 3')


def main():
    td = tempfile.mkdtemp(prefix='krt1925s_')
    try:
        engine(td)
        cli(td)
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if FAILS:
        print(f'FAIL: {len(FAILS)}: ' + '; '.join(FAILS))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
