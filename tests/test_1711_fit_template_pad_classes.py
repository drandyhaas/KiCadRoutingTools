"""#1711: fit_template seated each member against the courtyards at the board's
Default clearance and never priced a pad pair, so a member beside a pad of a
higher net class was fitted short of it -- and the seat gate, which prices
every pad pair through `legality.PadClearanceModel`, refused the fitted pose.

The fit now asks the seat gate's own pad test (`LegalityContext.pair_shortfall`
of the state it was given) of every candidate it would keep.

  A. A chip with a 0.2 mm class on its supply net, Default 0.15 mm, and two
     series resistors at the pins beside the supply pin: at every fitted
     template pose the gate's pad test finds no pair short (on the base code
     the chip / first resistor pair is 0.048 mm short).
  C. Control: the same board with the Default class only fits exactly the row
     it fitted before (the pad test changes nothing where the pads clear).

    python3 tests/test_1711_fit_template_pad_classes.py
"""
import json
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from run_utils import check, evidence, tool                   # noqa: E402

FAILS = []
SRC = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
PRO = os.path.join(ROOT, 'tests', 'fixtures', 'run25', 'esp_prog_placed.kicad_pro')
SKETCH = {"anchor": "U1", "source": "the UART series resistors at the chip's pins",
          "members": [{"ref": "R3", "pads": {"R3": ["2"], "U1": ["3"]}},
                      {"ref": "R4", "pads": {"R4": ["2"], "U1": ["4"]}}]}
#: the row the Default-only project fits (the base code fits it too)
DEFAULT_ROW = {'R3': [-0.713, 4.6815, 180.0], 'R4': [-2.383, 4.6815, 0.0]}


def ok(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}"
          + (f"  ({detail})" if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


def stage(td, tag, supply_class):
    """The board copied with its project beside it, a 0.2 mm class on the
    supply net when `supply_class`."""
    board = os.path.join(td, f'{tag}.kicad_pcb')
    check([sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_router', 'copy_board.py'),
           SRC, board], accept=True)
    pro = json.load(open(PRO, encoding='utf-8'))
    if supply_class:
        ns = pro['net_settings']
        ns['classes'].append(dict(ns['classes'][0], name='PWR', clearance=0.2, priority=0))
        ns['netclass_patterns'] = [{'netclass': 'PWR', 'pattern': '/+3.3V'}]
    json.dump(pro, open(board[:-len('.kicad_pcb')] + '.kicad_pro', 'w'), indent=2)
    return evidence(board, f'the {tag} board')


def fit_row(td, board, tag):
    row = os.path.join(td, f'{tag}_row.json')
    check([sys.executable, '-X', 'utf8', tool('fit_template.py'), board, '--sketch',
           os.path.join(td, 'sketch.json'), '-o', row], accept=True)
    evidence(row, 'the fitted row')
    return json.load(open(row))


def gate_shortfalls(board, row):
    """{(a, b): pad shortfall mm} of every pair the seat gate's pad test
    refuses at the row's template poses (the anchor at the origin, unturned)."""
    import pose_score
    from kicad_parser import parse_kicad_pcb
    from list_nets import board_floor_knobs
    pcb = parse_kicad_pcb(board)
    cl, edge, _ = board_floor_knobs(board)
    st = pose_score.make_state(pcb, board, clearance=cl, board_edge_clearance=edge,
                               estimate_courtyards=True)
    poses = {row['anchor']: (0.0, 0.0, 0.0)}
    poses.update({m['ref']: (m['at'][0], m['at'][1], m['rot']) for m in row['members']})
    refs, out = list(poses), {}
    for i, a in enumerate(refs):
        for b in refs[i + 1:]:
            sf = st.legality_ctx.pair_shortfall(a, b, pose_a=poses[a], pose_b=poses[b])
            if sf.pad > 1e-6 or sf.stack or sf.hole > 1e-6:
                out[(a, b)] = round(sf.pad, 4)
    return out


def main():
    with tempfile.TemporaryDirectory(prefix='t1711_') as td:
        json.dump(SKETCH, open(os.path.join(td, 'sketch.json'), 'w'))
        cls = stage(td, 'cls', True)
        row = fit_row(td, cls, 'cls')
        short = gate_shortfalls(cls, row)
        ok('A: with the supply class, no pair is short at the fitted poses (the gate\'s pad test)',
           short == {}, short)

        dflt = stage(td, 'dflt', False)
        row = fit_row(td, dflt, 'dflt')
        got = {m['ref']: [m['at'][0], m['at'][1], m['rot']] for m in row['members']}
        ok('C: the Default-only project fits the row it fitted before', got == DEFAULT_ROW, got)
        ok('C: ... and its pairs are clean', gate_shortfalls(dflt, row) == {})
    if FAILS:
        print(f"FAILED: {len(FAILS)} check(s)")
        return 1
    print("all checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
