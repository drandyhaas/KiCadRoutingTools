#!/usr/bin/env python3
"""The improvement gate does not trade a pour net for a signal net on a
draw, and grades the pours a rejection rests on with KiCad's exact fill.

An exact draw on both of the gate's axes (nets, disconnected pads) used to
ship whatever it traded. A draw that broke a pour net (a zone net) to connect
signals now does not: the run's in-run plane finalize already tried to mend
the cut, so the draw takes power pins off their plane to buy a signal that is
any later step's ordinary retry. But the gate reads pours through the raster
fill model, which can join fill islands KiCad's refill splits: one board's
retry step read as trading GND for /P0.17 when KiCad had GND split on its
input already (its finalize oracle left a GND link open), so with the rule
alone the gate reverted a plain gain and the board shipped 3 open nets for 2.
A rejection that rests on a pour net is therefore re-graded on KiCad's exact
fill of both boards before the gate acts on it.

Checks:
  A. gate_verdict's pour-trade rule: a draw that breaks a pour net to connect
     a signal is rejected, and says why; a signal-for-signal draw, a pour-for-
     pour draw and a draw that lowers the open pads still ship
  B. _exact_gate_pours grades the pours on KiCad's exact fill of both boards
     (CLI files, GUI staged saves, every temp dropped; no refill keeps the
     raster grade), and a retry's GND-for-/P0.17 "trade" is a plain gain
  C. batch_route's gate re-grades a pour-driven rejection before acting on it

    python3 tests/test_gate_pour_grade.py
"""
import ast
import contextlib
import io
import os
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from run_utils import evidence                                   # noqa: E402
from kicad_parser import parse_kicad_pcb                        # noqa: E402
from improvement_gate import (compare_connectivity, gate_verdict,  # noqa: E402
                              format_report)
import route                                                     # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files',
                     'lvds_converter_dualclk_gnd.kicad_pcb')
ZONE = '/GND'
failures = []


def check(label, cond, detail=''):
    print(f"  [{'ok' if cond else 'FAIL'}] {label}"
          + (f" -- {detail}" if detail and not cond else ''))
    if not cond:
        failures.append(label)


def quiet(fn, *a, **kw):
    with contextlib.redirect_stdout(io.StringIO()):
        return fn(*a, **kw)


# ------------------------------------------------------------------- A. gate
def t_gate():
    print('A. the improvement gate rejects a draw that trades a pour for a '
          'signal')
    names = {1: 'GND', 2: '/SIG', 3: '/A', 4: 'VCC'}
    nm = names.get
    # a pour trade: GND cut off one pad, /SIG connected, 2 -> 2 open pads
    c = compare_connectivity({1: (True, 0), 2: (False, 1), 3: (False, 1)},
                             {1: (False, 1), 2: (True, 0), 3: (False, 1)},
                             nm, zone_net_ids={1})
    check('GND cut for /SIG, pads 2 -> 2: a pour trade, rejected',
          c['lost_zone'] == ['GND'] and gate_verdict(c) == 'reject', str(c))
    r = format_report(c, gate_verdict(c), 'REVERTED')
    check('the report names the pour and the rule',
          'pour net(s) GND' in r and 'pours are whole' in r, r)
    c = compare_connectivity({1: (True, 0), 2: (False, 1)},
                             {1: (False, 1), 2: (True, 0)}, nm)
    check('the same trade with no pour nets named still ships',
          gate_verdict(c) == 'accept')
    c = compare_connectivity({1: (True, 0), 4: (False, 1)},
                             {1: (False, 1), 4: (True, 0)}, nm,
                             zone_net_ids={1, 4})
    check('a pour-for-pour draw ships', gate_verdict(c) == 'accept')
    c = compare_connectivity({1: (True, 0), 2: (False, 3)},
                             {1: (False, 1), 2: (True, 0)}, nm,
                             zone_net_ids={1})
    check('a pour-for-signal trade that LOWERS the open pads ships',
          gate_verdict(c) == 'accept')
    c = compare_connectivity({1: (True, 0), 2: (False, 1), 3: (False, 1)},
                             {1: (False, 1), 2: (True, 0), 3: (True, 0)}, nm,
                             zone_net_ids={1})
    check('one pour for two signals ships', gate_verdict(c) == 'accept')


# ------------------------------------------- B. the step gate's exact pours
def t_gate_exact():
    print('B. the step gate re-grades a pour-driven rejection on KiCad\'s '
          'fill')
    calls = []
    real = route._exact_pour_states

    def fake(board, board_file, names, project_from=None):
        calls.append((board_file, list(names), project_from))
        if 'none' in board_file:
            return None
        before = 'in' in os.path.basename(board_file)
        return {n: ((False, 1) if before else (False, 2)) for n in names}
    route._exact_pour_states = fake
    d = tempfile.mkdtemp(prefix='pour_gate_')
    try:
        inp = os.path.join(d, 'in.kicad_pcb')
        out = os.path.join(d, 'out.kicad_pcb')
        for p in (inp, out):
            shutil.copy(BOARD, p)
        r = route._exact_gate_pours(['GND'], False, inp, out, 'bp', 'ap',
                                    None, None, lambda t: None)
        check('CLI: both files graded, at the run\'s project',
              r == {'GND': ((False, 1), (False, 2))}
              and [c[0] for c in calls] == [inp, out]
              and all(c[2] == inp for c in calls), str((r, calls)))
        dropped = []

        def stage_in():
            p = os.path.join(d, 'in_staged.kicad_pcb')
            shutil.copy(BOARD, p)
            return p

        def stage_out():
            p = os.path.join(d, 'out_staged.kicad_pcb')
            shutil.copy(BOARD, p)
            return p, [p]
        calls.clear()
        r = quiet(route._exact_gate_pours, ['GND'], True, inp, '', None, None,
                  stage_in, stage_out, lambda t: dropped.extend(t))
        check('GUI: the live board and the staged change-set are graded, '
              'and every temp dropped',
              r == {'GND': ((False, 1), (False, 2))}
              and [os.path.basename(c[0]) for c in calls]
              == ['in_staged.kicad_pcb', 'out_staged.kicad_pcb']
              and sorted(map(os.path.basename, dropped))
              == ['in_staged.kicad_pcb', 'out_staged.kicad_pcb'],
              str((r, calls, dropped)))
        none = os.path.join(d, 'none.kicad_pcb')
        shutil.copy(BOARD, none)
        r = route._exact_gate_pours(['GND'], False, inp, none, 'bp', 'ap',
                                    None, None, lambda t: None)
        check('no refill on one board: the gate keeps its raster grade',
              r == {})
        r = route._exact_gate_pours(['GND'], True, inp, '', None, None,
                                    None, stage_out, lambda t: None)
        check('GUI without a staging callback: the raster grade stands',
              r == {})
    finally:
        route._exact_pour_states = real
        shutil.rmtree(d, ignore_errors=True)
    # A retry, as the gate now sees it: the raster says GND was
    # whole on the input and is cut on the output; KiCad says cut on both
    # (1 link -> 2), so /P0.17 is a plain gain.
    raster_b = {1: (True, 0), 2: (False, 1), 3: (False, 1)}
    raster_a = {1: (False, 1), 2: (True, 0), 3: (False, 1)}
    nm = {1: 'GND', 2: '/P0.17', 3: 'Net-(D1-Pad1)'}.get
    check('raster grade alone: a pour trade, rejected',
          gate_verdict(compare_connectivity(raster_b, raster_a, nm, {1}))
          == 'reject')
    raster_b[1], raster_a[1] = (False, 1), (False, 2)
    c = compare_connectivity(raster_b, raster_a, nm, {1})
    check('KiCad\'s grade: GND worsened, /P0.17 gained, accepted',
          c['lost'] == [] and c['gained'] == ['/P0.17']
          and gate_verdict(c) == 'accept', str(c))
    from kicad_exact_fill import find_kicad_python
    if find_kicad_python() is None:
        print('  [skip] _exact_pour_states on a real refill: no KiCad python')
        return
    pcb = quiet(parse_kicad_pcb, BOARD)
    st = quiet(route._exact_pour_states, pcb, BOARD, [ZONE, '/OUT_A'])
    check('_exact_pour_states grades the fixture\'s pour on a real refill',
          st is not None and ZONE in st and st[ZONE][0] in (True, False)
          and isinstance(st[ZONE][1], int), str(st))


# ------------------------------------------------------------- C. the wiring
def t_wiring():
    print('C. the step gate re-grades a pour-driven rejection before acting')
    with open(os.path.join(ROOT, 'py_router', 'route.py'),
              encoding='utf-8') as f:
        src = f.read()
    fn = next(n for n in ast.parse(src).body
              if isinstance(n, ast.FunctionDef) and n.name == 'batch_route')
    body = ast.get_source_segment(src, fn)
    g = body.find('_zone_ids600 = ')
    e = body.find('_exact_gate_pours(', g)
    check('the step gate re-grades a pour-driven rejection before acting',
          g >= 0 and body.find('_verdict = gate_verdict(_cmp)', g) < e
          < body.find("if _verdict == 'reject' and return_results:", g)
          and "if _verdict == 'reject' and _zhit600:" in body[g:e])


def main():
    evidence(BOARD, 'fixture board')
    t_gate()
    t_gate_exact()
    t_wiring()
    if failures:
        print(f"\nFAIL: {len(failures)} check(s): {failures}")
        return 1
    print("\nImprovement gate pour grading: all checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
