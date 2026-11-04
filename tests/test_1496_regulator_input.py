"""#1496: a regulator the agent classifies still feeds its load back to its input.

`supply_nets` on a typed LDO used to drop its typed input (it became a root
source), and an untyped regulator had no way to say what its input is, so
the input rail carried none of its load's current and nothing said so. Now
`supply_nets` names a source's outputs while its typed input pins are still
read, `input_nets` names an input the pins do not give (never taken as an
output), and a classified source that is a root while it has pads on two or
more rails is warned about.

    python3 tests/test_1496_regulator_input.py
"""
import contextlib
import io
import os
import sys
import warnings

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path[:0] = [os.path.join(REPO, 'py_tools'), os.path.join(REPO, 'py_router')]

from analyze_power_paths import (analyze_pcb, classify_component,  # noqa: E402
                                 ComponentRole as R, trace_power_paths,
                                 get_power_net_recommendations, _regulator_terminals)

failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def run(board, classes, reg):
    with contextlib.redirect_stdout(io.StringIO()):
        c, p = analyze_pcb(os.path.join(REPO, 'kicad_files', board))
    for ref, role, ma, sn, inn in classes:
        kw = {'input_nets': inn} if inn else {}
        classify_component(c, ref, role, current_rating_ma=ma, supply_nets=sn, **kw)
    with warnings.catch_warnings(record=True) as w:
        warnings.simplefilter('always')
        rec = get_power_net_recommendations(p, c, trace_power_paths(p, c))
    roots = [str(x.message) for x in w if 'root source' in str(x.message)]
    return _regulator_terminals(c[reg]), rec, roots


def main():
    base = [('J201', R.POWER_SOURCE, 1000, None, None), ('TB201', R.POWER_SOURCE, 1000, None, None)]
    sink = ('U102', R.CURRENT_SINK, 1500, None, None)
    print('1. a typed LDO named by supply_nets keeps its typed input')
    term, rec, _ = run('kit-dev-coldfire-xilinx_5213.kicad_pcb',
                       base + [('VR201', R.POWER_SOURCE, 500, ['+3.3V'], None), sink], 'VR201')
    check('VR201: outputs +3.3V, input Net-(D201-K)', term == (['+3.3V'], ['Net-(D201-K)']), f'{term}')
    check('...and its input chain is charged (2.0 mm)',
          rec.get('Net-(D201-K)') == 2.0 and rec.get('Net-(TB201-P1)') == 2.0, f'{rec}')

    print('2. an untyped buck on its switch node, with its input named')
    up = [('US2', R.POWER_SOURCE, 3000, None, None), ('D9', R.PASS_THROUGH, None, None, None),
          ('RP1', R.PASS_THROUGH, None, None, None),
          ('U4', R.POWER_SOURCE, 2000, ['/power/L2'], ['+5V']),
          ('U5', R.POWER_SOURCE, 2000, ['/power/L3'], ['+5V'])]
    load = ('U1', R.CURRENT_SINK, 1500, ['+1V1'], None)
    term, rec, roots = run('ulx3s.kicad_pcb',
                           up + [('U3', R.POWER_SOURCE, 2000, ['/power/L1'], ['+5V']), load], 'U3')
    check('U3: output its switch node, input +5V', term == (['/power/L1'], ['+5V']), f'{term}')
    check('...+5V and the USB input carry its load (1.2 mm)',
          rec.get('+5V') == 1.2 and rec.get('/usb/US2VBUS') == 1.2, f'{rec}')
    check('...and no root-source warning', not roots, f'{roots}')
    term, rec, roots = run('ulx3s.kicad_pcb',
                           up + [('U3', R.POWER_SOURCE, 2000, ['/power/L1'], None), load], 'U3')
    check('without input_nets U3 is a root, and is warned about',
          rec.get('+5V') is None and any(r.startswith('U3 is a root source') for r in roots),
          f'{roots}')

    print('3. tigard, #1378\'s own board: both LDOs declared reach /VBUS')
    tg = [('J1', R.POWER_SOURCE, 500, None, None), ('FB3', R.PASS_THROUGH, 500, None, None),
          ('U1', R.POWER_SOURCE, 300, ['+1V8'], ['+5V']),
          ('U2', R.POWER_SOURCE, 300, ['+3V3'], ['+5V']), ('U3', R.CURRENT_SINK, 150, None, None)]
    term, rec, roots = run('tigard.kicad_pcb', tg, 'U2')
    check('/VBUS carries the load (0.7 mm)', rec.get('/VBUS') == 0.7, f'{rec}')
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            c, _p = analyze_pcb(os.path.join(REPO, 'kicad_files', 'tigard.kicad_pcb'))
        classify_component(c, 'U2', R.POWER_SOURCE, input_nets=['NOPE'])
        check('an input_nets net the part has no pad on is refused', False, 'accepted')
    except ValueError as ex:
        check('an input_nets net the part has no pad on is refused', 'input_nets' in str(ex), str(ex))

    if failures:
        print(f'FAIL: {len(failures)}: ' + '; '.join(failures))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
