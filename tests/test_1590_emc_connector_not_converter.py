"""#1590: a connector is never a switching-converter candidate.

`emc_roles._converters` takes the parts on an inductor's net that `big`
keeps -- three or more copper pads and no part_class -- and reads a pin-less
one alone on an undecoupled net as a switching-converter candidate. A coax
connector named by its part number has no part_class, so a bias-tee inductor
on its centre pin made it a candidate, its RF net a proposed switch node in
`emc_roles.keep_away`, and `check_groups` printed "emc_roles reads a
switching converter" for it -- while emc_roles' own `_Board.connector` said
connector (its J reference). The hint also said "reads a switching
converter" flatly for every pin-less candidate.

Pinned here, on the issue's board plus two parts:
  1. the coax J1 is no role and its RF net no switch node; nor is a
     connector whose pin is named SW on an inductor net (the pin arm);
  2. the documented heuristic stands: the pin-less eFuse U1 alone on an
     undecoupled inductor net is still a candidate, and a converter with an
     SW pin (U4) is still a switching converter;
  3. `check_groups` names no hint for J1, says "candidate" for U1, and keeps
     the plain wording for U4.

    python3 tests/test_1590_emc_connector_not_converter.py
"""
import contextlib
import io
import json
import os
import shutil
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from kicad_parser import parse_kicad_pcb                      # noqa: E402
from placement.emc_roles import _Board, emc_roles             # noqa: E402
from run_utils import check as run_check, evidence, tool      # noqa: E402

FAILS = []
NETS = ['GND', 'RF1', 'RF2', 'VPOC', 'SER1', 'SER2', 'VIN_A', 'VIN_B', 'CONN',
        'VOUT', 'Y_IN', 'Y_OUT', 'RF3', 'SWN', 'FBN', 'VIN_C', 'VOUT2']
ID = {n: i + 1 for i, n in enumerate(NETS)}


def check(name, ok, detail=''):
    print(f"  {'PASS' if ok else 'FAIL'}  {name}"
          + (f"  ({detail})" if detail and not ok else ''))
    if not ok:
        FAILS.append(name)


def _fp(ref, name, x, pads, fn=()):
    out = [f'  (footprint "{name}" (layer "F.Cu") (at {x} 20)',
           f'    (property "Reference" "{ref}" (at 0 0) (layer "F.SilkS") '
           f'(hide yes))',
           '    (fp_rect (start -2.5 -2.5) (end 2.5 2.5) (stroke (width 0.05) '
           '(type default)) (layer "F.CrtYd"))']
    for k, net in enumerate(pads):
        f = f' (pinfunction "{fn[k]}")' if fn else ''
        out.append(f'    (pad "{k + 1}" smd rect (at {-1.5 + k * 0.75} 0) '
                   f'(size 0.5 1) (layers "F.Cu") (net {ID[net]} "{net}"){f})')
    return '\n'.join(out) + '\n  )\n'


def _board(path):
    """The issue's board: J1 a coax connector named by its part number, J2
    the same pads named FAKRA, L1/L2 bias-tee inductors from each centre pin
    to VPOC, C1/C2 series DC-block caps to a serializer U2, U1 a pin-less
    eFuse fed through L3, U3 the same eFuse with named pins, Y1 the
    serializer's crystal. Added: J4, a connector whose pin is named SW on
    inductor L6's net, and U4, a buck with SW / VIN / GND / FB / EN pins."""
    L, C = 'Inductor_SMD:L_0805_2012Metric', 'Capacitor_SMD:C_0402_1005Metric'
    ef = 'Package_SO:HTSSOP-16_4.4x5mm_P0.65mm'
    parts = [
        _fp('J1', 'RF:59S2AQ-400A5-Z', 10, ['RF1', 'GND', 'GND', 'GND', 'GND']),
        _fp('J2', 'RF:FAKRA_59S2AQ-400A5-Z', 20,
            ['RF2', 'GND', 'GND', 'GND', 'GND']),
        _fp('L1', L, 30, ['RF1', 'VPOC']), _fp('L2', L, 40, ['RF2', 'VPOC']),
        _fp('C1', C, 50, ['RF1', 'SER1']), _fp('C2', C, 60, ['RF2', 'SER2']),
        _fp('C3', C, 70, ['VPOC', 'GND']),
        _fp('U1', ef, 80, ['VIN_A', 'VIN_A', 'VOUT', 'VOUT', 'GND']),
        _fp('L3', L, 90, ['VIN_A', 'CONN']),
        _fp('U3', ef, 100, ['VIN_B', 'VIN_B', 'VOUT', 'VOUT', 'GND'],
            ('VIN', 'VIN', 'VOUT', 'VOUT', 'GND')),
        _fp('L4', L, 110, ['VIN_B', 'CONN']),
        _fp('J3', 'TerminalBlock:TB_1x02', 120, ['CONN', 'GND']),
        _fp('C4', C, 130, ['VOUT', 'GND']), _fp('C5', C, 140, ['CONN', 'GND']),
        _fp('U2', 'Package_DFN_QFN:QFN-16_3x3mm_P0.5mm', 150,
            ['SER1', 'SER2', 'Y_IN', 'Y_OUT', 'GND']),
        _fp('Y1', 'Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm', 160,
            ['Y_IN', 'GND', 'Y_OUT', 'GND']),
        _fp('J4', 'house:PN-4410-03', 170, ['RF3', 'GND', 'GND'],
            ('SW', 'GND', 'GND')),
        _fp('L6', L, 180, ['RF3', 'VPOC']),
        _fp('U4', 'Package_TO_SOT_SMD:SOT-23-6', 190,
            ['SWN', 'GND', 'FBN', 'VIN_C', 'VIN_C'],
            ('SW', 'GND', 'FB', 'EN', 'VIN')),
        _fp('L5', L, 200, ['SWN', 'VOUT2']),
        _fp('C6', C, 210, ['VOUT2', 'GND']), _fp('C7', C, 220,
                                                 ['VIN_C', 'GND'])]
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write('(kicad_pcb (version 20240108) (generator pcbnew)\n'
                 '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) '
                 '(44 "Edge.Cuts" user))\n  (net 0 "")\n'
                 + ''.join(f'  (net {ID[n]} "{n}")\n' for n in NETS)
                 + '  (gr_rect (start 0 0) (end 230 40) (stroke (width 0.1) '
                 '(type default)) (layer "Edge.Cuts"))\n'
                 + ''.join(parts) + ')\n')


def main():
    tmp = tempfile.mkdtemp(prefix='t1590_')
    try:
        board = os.path.join(tmp, 'b.kicad_pcb')
        _board(board)
        with contextlib.redirect_stdout(io.StringIO()):
            pcb = parse_kicad_pcb(evidence(board, 'board'))
        b = _Board(pcb)
        check('the board is the issue\'s: J1 has no part class, is big, and '
              '_Board.connector calls it a connector',
              b.cls['J1'] is None and b.big('J1') and b.connector('J1')
              and b.connector('J4') and b.big('J4'),
              (b.cls['J1'], b.big('J1'), b.connector('J1'), b.big('J4')))
        e = emc_roles(pcb)
        roles = {r['part']: r for r in e.get('roles') or ()}

        # 1. connectors are no converter, by either arm
        check('the coax J1 is no switching-converter candidate',
              'J1' not in roles, roles.get('J1'))
        check('a connector with an SW pin on an inductor net is no converter',
              'J4' not in roles, roles.get('J4'))
        nodes = {n['net']: n['part'] for k in e.get('keep_away') or ()
                 for n in k['switch_nodes']}
        check('neither RF net is proposed as a switch node',
              'RF1' not in nodes and 'RF3' not in nodes, nodes)

        # 2. the heuristic stands
        u1 = roles.get('U1') or {}
        check('the pin-less U1 is still a switching-converter candidate',
              u1.get('role') == 'unknown'
              and u1.get('candidate') == 'switching converter', u1)
        u4 = roles.get('U4') or {}
        check('U4, with an SW pin on L5, is still a switching converter',
              u4.get('role') == 'switching converter'
              and u4.get('switch_net') == 'SWN', u4)
        check('the keep-away proposal keeps the candidate and the SW nodes',
              nodes.get('VIN_A') == 'U1' and nodes.get('SWN') == 'U4', nodes)

        # 3. the check_groups hints
        groups = os.path.join(tmp, 'groups.json')
        with open(groups, 'w', encoding='utf-8') as fh:
            json.dump({'schema': 1, 'groups': [
                {'name': 'all', 'refs': sorted(pcb.footprints)}]}, fh)
        r = run_check([sys.executable, tool('check_groups.py'), board,
                       '--groups', evidence(groups, 'groups')], accept=True)
        hints = {}
        for line in r.stdout.splitlines():
            if 'HINT' in line:
                ref = line.split('HINT', 1)[1].split()[0]
                hints[ref] = line
        check('check_groups ran its hints', 'U4' in hints, r.stdout[-800:])
        check('no hint names J1 or J4 a converter',
              not any('emc_roles' in hints.get(j, '') for j in ('J1', 'J4')),
              {j: hints.get(j) for j in ('J1', 'J4')})
        check('the hint for U1 says candidate',
              'switching-converter candidate' in hints.get('U1', ''),
              hints.get('U1'))
        check('the hint for U4 says switching converter, not candidate',
              'emc_roles reads a switching converter' in hints.get('U4', '')
              and 'candidate' not in hints.get('U4', ''), hints.get('U4'))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    if FAILS:
        print(f"FAILED: {len(FAILS)} check(s)")
        return 1
    print("all checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
