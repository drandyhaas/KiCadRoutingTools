"""#1820: the two series parts of an AC-coupled (or series-resistor) pair.

`kicad_files/cap_chain.kicad_pcb` is one pair through C1 (P leg) and C2 (N
leg). `placement.pair_series` finds the junction the router's own
`diff_xnet.find_ac_coupled_xnets` finds and says what departs from a matched
pair: `together` (side by side, same rotation) reads matched, `split` (C2
moved 6.5 mm and turned a quarter) reads turned and ahead along the pair, and
`board_context` names it. A termination resistor across ONE pair is not a
junction.

    python3 tests/test_1820_pair_series.py
"""
import contextlib
import io
import json
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from kicad_parser import parse_kicad_pcb                      # noqa: E402
from placement import pair_series as ps                       # noqa: E402
from run_utils import check, evidence                         # noqa: E402

PY = [sys.executable, '-X', 'utf8']
FAILS = []


def ok(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}"
          + (f"  ({detail})" if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


def tool(*p):
    return os.path.join(ROOT, *p)


def parse(path):
    with contextlib.redirect_stdout(io.StringIO()):
        return parse_kicad_pcb(evidence(path, 'board'))


def main(td):
    src = os.path.join(td, 'in.kicad_pcb')
    check(PY + [tool('py_router', 'copy_board.py'), tool('kicad_files', 'cap_chain.kicad_pcb'),
                src], accept=True)
    boards = {}
    for name, ops in (('together', ['set', 'C1', '18', '15.6', 'set', 'C2', '18', '16.9']),
                      ('split', ['set', 'C1', '18', '15.6', 'set', 'C2', '13', '21',
                                 '--rot', '90'])):
        boards[name] = os.path.join(td, f'{name}.kicad_pcb')
        check(PY + [tool('py_placer', 'place_pose.py'), src, boards[name]] + ops, accept=True)

    js, _w = ps.pair_junctions(parse(boards['together']))
    ok('one junction, C1 on the P leg and C2 on the N leg',
       [(j.p_ref, j.n_ref) for j in js] == [('C1', 'C2')], js)
    if not js:
        return
    g = ps.junction_geometry(parse(boards['together']), js[0])
    ok('together: matched (one face, parallel, side by side)',
       g is not None and ps.split_reasons(g) == [], g)
    g = ps.junction_geometry(parse(boards['split']), js[0])
    why = ps.split_reasons(g) if g else []
    ok('split: turned a quarter and ahead along the pair',
       g is not None and abs(g['axis_turn_deg'] - 90.0) < 1e-6
       and g['offset_mm'] > ps.OFFSET_TOL_MM
       and any('turned' in w for w in why) and any('ahead' in w for w in why), g)

    # board_context names the split junction, and not the matched one
    for name, want in (('together', False), ('split', True)):
        out = os.path.join(td, f'ctx_{name}.json')
        check(PY + [tool('py_tools', 'board_context.py'), boards[name], '--json',
                    '-o', out, '--no-schematic'], accept=True)
        doc = json.load(open(evidence(out, 'board_context JSON')))
        rows = [r for r in (doc.get('pin_order') or {}).get('rows') or ()
                if r.get('composed')]
        poses = [s for r in rows for s in r.get('series_pose') or ()]
        ok(f'board_context {name}: one series pose, split={want}',
           len(poses) == 1 and bool(poses[0]['split']) == want, poses)

    # control: a termination resistor across one pair is no junction
    lv = parse(tool('kicad_files', 'lvds_converter_dualclk.kicad_pcb'))
    js, _w = ps.pair_junctions(lv)
    ok('control: a resistor across one pair is not a junction', js == [], js)


if __name__ == '__main__':
    with tempfile.TemporaryDirectory() as td:
        main(td)
    print('ALL PASS' if not FAILS else f'FAILED: {FAILS}')
    sys.exit(1 if FAILS else 0)
