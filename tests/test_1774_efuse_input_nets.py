"""#1774: an eFuse with a bias pin on the raw input and its IN behind a series
FET used to be traced straight to the raw rail, so the net between the FET and
the eFuse, which carries the load, got no width -- and naming the real input
with `input_nets` changed nothing, because named inputs were ADDED to the pins'
reading.

Now `input_nets` replaces the pins' reading for that source (as `supply_nets`
replaces its outputs), and without it `get_power_net_recommendations` warns
when two inputs of one classified source are joined through PASS_THROUGH parts,
naming the net left without width and the `input_nets` that fixes it.

    python3 tests/test_1774_efuse_input_nets.py
"""
import contextlib
import io
import os
import sys
import tempfile
import warnings

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path[:0] = [os.path.join(REPO, 'py_tools'), os.path.join(REPO, 'py_router')]

from analyze_power_paths import (analyze_pcb, classify_component,  # noqa: E402
                                 ComponentRole as R, trace_power_paths,
                                 get_power_net_recommendations, _regulator_terminals)

NETS = ['GND', 'PWR_IN', 'PWR_FET', 'GATE', 'PWR_SW', '+5V']
NID = {n: i + 1 for i, n in enumerate(NETS)}
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def fp(ref, name, x, pads):   # pads: (number, net, pinfunction, pintype)
    s = (f'  (footprint "{name}" (layer "F.Cu") (at {x} 20)\n'
         f'    (property "Reference" "{ref}" (at 0 0) (layer "F.SilkS"))\n')
    for k, (num, net, fn, pt) in enumerate(pads):
        s += (f'    (pad "{num}" smd rect (at 0 {k * 1.0:.1f}) (size 0.6 0.6) (layers "F.Cu")'
              f' (net {NID[net]} "{net}") (pinfunction "{fn}") (pintype "{pt}"))\n')
    return s + '  )\n'


def board(path, vcc_net):
    """J1 (raw input) -> Q1 (reverse-blocking FET) -> U1 (eFuse, IN on PWR_FET,
    VCC bias on `vcc_net`) -> U2 (LDO) -> U3 (a 3 A load)."""
    parts = (fp('J1', 'Connector:Conn_01x02', 5,
                [('1', 'PWR_IN', 'Pin_1', 'passive'), ('2', 'GND', 'Pin_2', 'passive')])
             + fp('Q1', 'Package_TO_SOT_SMD:SOT-23', 15,
                  [('1', 'GATE', 'G', 'input'), ('2', 'PWR_FET', 'S', 'passive'),
                   ('3', 'PWR_IN', 'D', 'passive')])
             + fp('U1', 'Package_SO:MSOP-10', 25,
                  [('1', 'PWR_FET', 'IN', 'power_in'), ('2', vcc_net, 'VCC', 'power_in'),
                   ('3', 'GATE', 'DRV', 'output'), ('4', 'PWR_SW', 'OUT', 'power_out'),
                   ('5', 'GND', 'GND', 'power_in')])
             + fp('U2', 'Package_TO_SOT_SMD:SOT-223', 35,
                  [('1', 'PWR_SW', 'VI', 'power_in'), ('2', '+5V', 'VO', 'power_out'),
                   ('3', 'GND', 'GND', 'power_in')])
             + fp('U3', 'Package_SO:SOIC-8', 45,
                  [('1', '+5V', 'VCC', 'power_in'), ('2', 'GND', 'GND', 'power_in')]))
    with open(path, 'w') as f:
        f.write('(kicad_pcb (version 20240108) (generator pcbnew)\n'
                '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
                '  (net 0 "")\n' + ''.join(f'  (net {i} "{n}")\n' for n, i in NID.items())
                + '  (gr_rect (start 0 0) (end 60 40) (stroke (width 0.1) (type default))'
                  ' (layer "Edge.Cuts"))\n' + parts + ')\n')


def run(path, u1_inputs=None):
    with contextlib.redirect_stdout(io.StringIO()):
        comps, pcb = analyze_pcb(path)
    classify_component(comps, 'J1', R.POWER_SOURCE, current_rating_ma=5000, supply_nets=['PWR_IN'])
    classify_component(comps, 'Q1', R.PASS_THROUGH)
    classify_component(comps, 'U1', R.POWER_SOURCE, current_rating_ma=4000, input_nets=u1_inputs)
    classify_component(comps, 'U2', R.POWER_SOURCE, current_rating_ma=3000)
    classify_component(comps, 'U3', R.CURRENT_SINK, current_rating_ma=3000)
    paths = trace_power_paths(pcb, comps)
    with warnings.catch_warnings(record=True) as w:
        warnings.simplefilter('always')
        rec = get_power_net_recommendations(pcb, comps, paths)
    joined = [str(x.message) for x in w if 'are joined through' in str(x.message)]
    return _regulator_terminals(comps['U1']), paths, rec, joined


def main():
    with tempfile.TemporaryDirectory() as d:
        bias_raw = os.path.join(d, 'bias_on_raw.kicad_pcb')
        bias_fet = os.path.join(d, 'bias_on_fet.kicad_pcb')
        board(bias_raw, 'PWR_IN')
        board(bias_fet, 'PWR_FET')

        print('1. input_nets replaces the pins\' reading')
        term, paths, rec, joined = run(bias_raw, ['PWR_FET'])
        check('U1: output PWR_SW, input PWR_FET only (the VCC bias pin is left out)',
              term == (['PWR_SW'], ['PWR_FET']), f'{term}')
        u3 = [p for p in paths if p.sink_component == 'U3']
        check('the load is traced through the FET',
              len(u3) == 1 and u3[0].nets_in_path == ['PWR_IN', 'PWR_FET', 'PWR_SW', '+5V']
              and u3[0].components_in_path == ['Q1', 'U1', 'U2'],
              f'{[(p.nets_in_path, p.components_in_path) for p in u3]}')
        check('PWR_FET, the net carrying the 3 A, gets 2.0 mm', rec.get('PWR_FET') == 2.0, f'{rec}')
        check('...and the rest of the chain keeps it',
              all(rec.get(n) == 2.0 for n in ('PWR_IN', 'PWR_SW', '+5V')), f'{rec}')
        check('no joined-input warning once the input is named', not joined, f'{joined}')

        print('2. without input_nets the shortcut is warned about (and only warned)')
        term, paths, rec, joined = run(bias_raw)
        check('U1 reads both power_in pins as inputs',
              term == (['PWR_SW'], ['PWR_FET', 'PWR_IN']), f'{term}')
        check('PWR_FET still gets no width (warn only, no heuristic)',
              'PWR_FET' not in rec, f'{rec}')
        check('one warning, naming PWR_FET, the joining FET and the fix',
              len(joined) == 1 and joined[0].startswith("U1's inputs PWR_FET and PWR_IN")
              and 'through Q1' in joined[0] and "input_nets=['PWR_FET']" in joined[0],
              f'{joined}')
        check('the gate net is never charged', 'GATE' not in rec, f'{rec}')

        print('3. control: the bias pin behind the FET needs no warning')
        term, paths, rec, joined = run(bias_fet)
        check('U1 has the one input PWR_FET', term == (['PWR_SW'], ['PWR_FET']), f'{term}')
        check('PWR_FET gets 2.0 mm', rec.get('PWR_FET') == 2.0, f'{rec}')
        check('no warning', not joined, f'{joined}')

    if failures:
        print(f'FAIL: {len(failures)}: ' + '; '.join(failures))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
