#!/usr/bin/env python3
"""autoplan.py: the plan it reads off a board, how it picks a board, and a run.

1. `_glob` gives every net a pattern that selects that net alone through the
   selectors the chain's tools use -- `expand_net_patterns` (route.py
   --nets), `matches_net_filter` with '!' exclusions (the fanouts) and
   `identify_power_nets` (--power-nets) -- and never starts with '!' or '-'.
   A net no pattern isolates on the board is left out of the analysis.
2. `fanout_parts` reads the pads: a grid array or a four-sided perimeter
   package is fanned out, at the pitch measured along its pad rows; a
   two-row part, a connector or magnetics by name, and a part whose pads
   form neither shape are not.
3. `analyze` on in-repo boards: planes on B.Cu for 2 layers and on the first
   two inner layers for 4+, plane layers priced up, fine-pitch parts fanned
   out on the fine grid.
4. `route_passes` with a scripted chain: a pass is kept only when it is no
   worse on open nets and DRC and better on one; --finish keeps the input
   unless a pass beats it; the iteration budget is a forecast from reported
   counts, and a pass that reports none starts no other.
5. --effort-factor refuses a non-finite or negative value.
6. Real runs on splitflap_driver: end to end, --finish, and --finish on a
   board inside the output's own work directory with a stale sibling at the
   output -- graded by check_connected, not by autoplan's own summary.
"""
import json
import os
import subprocess
import sys
import tempfile
from types import SimpleNamespace

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
import autoplan as AP                                        # noqa: E402
from net_queries import (expand_net_patterns, identify_power_nets,  # noqa: E402
                         matches_net_filter)
from run_utils import check, evidence                        # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')
SCRIPT = os.path.join(ROOT, 'py_router', 'autoplan.py')
FAILS = []


def expect(cond, what):
    print(('  ok   ' if cond else '  FAIL ') + what)
    if not cond:
        FAILS.append(what)


def _stub_board(names):
    nets = {i + 1: SimpleNamespace(name=n) for i, n in enumerate(names)}
    pads = {i: [SimpleNamespace(net_name=n.name)] for i, n in nets.items()}
    return SimpleNamespace(nets=nets, pads_by_net=pads)


def test_glob():
    print("_glob through the real selectors")
    names = ['GND', '/GND', '/Analog/GND', 'GNDA', '-BATT', 'BATT', '!D[0]',
             'D[0]', 'D0', '/D[1]', 'A*B', 'AxB', 'Q?', 'QQ', '-5V', ']x', '[x',
             '+5V', '/5V', 'GN', 'x]', 'x^', '!RESET', 'RESET', '/Sheet/!RESET', 'A']
    pcb = _stub_board(names)
    ids = {n.name: i for i, n in pcb.nets.items()}
    for n in names:
        pat = AP._glob(n)
        others = sorted(set(names) - {n})
        expect(not pat.startswith(('!', '-')), f"{n!r} -> {pat!r} is not an option or an exclusion")
        expect(AP._selects_exactly(pat, n, names), f"{pat!r} isolates {n!r}")
        expect(expand_net_patterns(pcb, [pat], quiet=True) == [n],
               f"--nets {pat!r} selects only {n!r}")
        expect(sorted(expand_net_patterns(pcb, ['*', '!' + pat], quiet=True)) == others,
               f"--nets '*' '!{pat}' drops only {n!r}")
        expect([m for m in names if not matches_net_filter(m, ['*', '!' + pat])] == [n],
               f"fanout filter '!{pat}' excludes only {n!r}")
        expect(set(identify_power_nets(pcb, [pat], [0.3])) == {ids[n]},
               f"--power-nets {pat!r} widens only {n!r}")
    expect(not AP._selects_exactly(AP._glob('!D[0]'), '!D[0]', ['!D[0]', ' D[0]']),
           "'!D[0]' beside ' D[0]' cannot be isolated, and that is detected")


def _pad(x, y, w=0.25, h=0.8, drill=0.0, layers=None, num='', net_id=1):
    return SimpleNamespace(local_x=x, local_y=y, size_x=w, size_y=h, drill=drill, pad_number=num,
                           pad_type='thru_hole' if drill else 'smd', net_id=net_id,
                           layers=layers or (['*.Cu'] if drill else ['F.Cu']))


def _fp(name, pads, rotation=0.0):
    return SimpleNamespace(footprint_name=name, pads=pads, rotation=rotation)


def _qfn(n_side, pitch, ep=True):
    """A four-sided package, n_side pads a side, with an exposed pad and two
    sub-pads inside it."""
    half = (n_side - 1) * pitch / 2
    edge = half + 0.6
    pads = []
    for i in range(n_side):
        o = -half + i * pitch
        pads += [_pad(-edge, o, 0.8, 0.25), _pad(edge, o, 0.8, 0.25),
                 _pad(o, -edge), _pad(o, edge)]
    if ep:
        pads += [_pad(0, 0, 2 * half, 2 * half), _pad(-0.3, 0, 0.3, 0.3), _pad(0.3, 0, 0.3, 0.3)]
    return pads


def _grid(nx, ny, px, py, drill=0.0):
    return [_pad(i * px, j * py, 0.3, 0.3, drill) for i in range(nx) for j in range(ny)]


def _two_rows(n, pitch):
    return ([_pad(i * pitch, 0) for i in range(n)]
            + [_pad(i * pitch, 2.0) for i in range(n)])


def test_fanout_parts():
    print("fanout_parts")
    parts = {
        'U1': _fp('Package_DFN_QFN:QFN-48-1EP_7x7mm_P0.5mm', _qfn(12, 0.5)),
        'U2': _fp('Package_QFP:LQFP-64_10x10mm_P0.5mm', _qfn(16, 0.5, ep=False)),
        'U3': _fp('lib:MT53D_LPDDR4', _grid(10, 20, 0.8, 0.65)),
        'U4': _fp('Package_BGA:BGA-256_17x17mm', _grid(16, 16, 1.0, 1.0)),
        'U5': _fp('Package_DFN_QFN:DFN-8-1EP_3x2mm_P0.5mm', _two_rows(4, 0.5)),
        'U6': _fp('Package_QFP:LQFP-32_7x7mm_P0.8mm', _qfn(8, 0.8, ep=False)),
        'J1': _fp('Connector_USB:USB_C_Receptacle', _qfn(12, 0.5)),
        'J2': _fp('lib:Bus_M.2_Socket_Key_M', _two_rows(35, 0.5)),
        'J3': _fp('lib:SomeEdgeThing', _two_rows(30, 0.5)),
        'T1': _fp('lib:Trans_Eth_4.7x3.3mm', _qfn(3, 0.65)),
        'RN1': _fp('Resistor_SMD:R_Array_Convex_4x0402', _two_rows(4, 0.5)),
        'U7': _fp('Package_DIP:PLCC-44_THT-Socket', _grid(11, 11, 2.54, 2.54, drill=0.9)),
        'U8': _fp('lib:scatter', [_pad(0, 0), _pad(0.37, 1.1), _pad(1.9, 0.2), _pad(2.6, 2.2),
                                  _pad(0.9, 3.1), _pad(3.3, 0.8), _pad(1.4, 1.7)]),
        'U9': _fp('lib:DDR3_FBGA-96', [p for p in _grid(9, 16, 0.8, 0.8) if not 2.0 < p.local_x < 4.4]),
        'U14': _fp('lib:AQFN-73_7x7mm_P0.5mm',
                   [_pad(x, y, 0.25, 0.25) for i in range(12)
                    for x, y in ((-2.75 + i * 0.5, -3.25), (-2.75 + i * 0.5, 3.25),
                                 (-3.25, -2.75 + i * 0.5), (3.25, -2.75 + i * 0.5))]
                   + [_pad(x, y, 0.25, 0.25) for i in range(10)
                      for x, y in ((-2.25 + i * 0.5, -2.75), (-2.25 + i * 0.5, 2.75),
                                   (-2.75, -2.25 + i * 0.5), (2.75, -2.25 + i * 0.5))]
                   + [_pad(0, 0, 4.0, 4.0)]),
        'U15': _fp('lib:WFQFN-20', [_pad(o, e, 0.3, 0.9) for o in (-0.75, -0.25, 0.25, 0.75) for e in (-1.55, 1.55)]
                   + [_pad(e, o, 0.9, 0.3) for o in (-0.75, -0.25, 0.25, 0.75) for e in (-1.55, 1.55)]
                   + [_pad(x, y, 0.35, 0.35) for x in (-1.25, 1.25) for y in (-1.25, 1.25)]
                   + [_pad(0, 0, 1.8, 1.8)]),
        'U16': _fp('lib:QFN-20-1EP_3x3mm_P0.4mm_ThermalVias',
                   [_pad(o, e, 0.2, 0.7) for o in (-0.8, -0.4, 0, 0.4, 0.8) for e in (-1.45, 1.45)]
                   + [_pad(e, o, 0.7, 0.2) for o in (-0.8, -0.4, 0, 0.4, 0.8) for e in (-1.45, 1.45)]
                   + [_pad(0, 0, 1.5, 1.5)]
                   + [_pad(x, y, 0.6, 0.6, layers=['F.Paste']) for x in (-0.41, 0.41) for y in (-0.41, 0.41)]),
        'U13': _fp('lib:IMX283', [_pad(x + (0.001 if y > 3 else 0), y, 0.9, 0.9)
                                  for x in (0, 1.5, 3.0, 4.5, 6.0, 7.5) for y in (0, 1.5, 3.0, 4.5, 6.0, 7.5)]
                   + [_pad(x, y, 2.4, 2.4) for x in (-1.5, 9.0) for y in (-1.5, 9.0)]),
        # 5 x 7 mm VQFN turned 90 degrees: its 5.7 x 3.7 exposed pad is 3.7
        # wide in the footprint's own frame, clear of the side pads at 2.4.
        'U17': _fp('lib:VQFN-40_5x7mm', [_pad(e, -2.75 + i * 0.5, 0.6, 0.25) for i in range(12) for e in (-2.4, 2.4)]
                   + [_pad(-1.75 + i * 0.5, e, 0.25, 0.6) for i in range(8) for e in (-3.4, 3.4)]
                   + [_pad(0, 0, 5.7, 3.7, num='41')], rotation=90.0),
        'U18': _fp('lib:QFN-20_BottomPad', [_pad(o, e, 0.2, 0.7) for o in (-0.8, -0.4, 0, 0.4, 0.8) for e in (-1.45, 1.45)]
                   + [_pad(e, o, 0.7, 0.2) for o in (-0.8, -0.4, 0, 0.4, 0.8) for e in (-1.45, 1.45)]
                   + [_pad(0, 0, 1.5, 1.5), _pad(0, 0, 4.0, 4.0, layers=['B.Cu'])]),
        'U19': _fp('lib:S-PWSON-N10', [_pad(e, -1.0 + i * 0.5, 0.85, 0.28, num=str(i + 1))
                                       for i in range(5) for e in (-1.475, 1.475)]
                   + [_pad(0, 0, 1.7, 2.15, num='11')]
                   + [_pad(-0.6 + i * 0.4, e, 0.23, 0.6, num='11') for i in range(4) for e in (-1.425, 1.425)]),
        # Texas RGY-24 as orangecrab's library draws it: the exposed pad is
        # numbered 12, like a real pin.
        'U21': _fp('lib:Texas_RGY_R-PVQFN-N24', [_pad(e, -2.25 + i * 0.5, 0.96, 0.28, num=str(i + (2 if e < 0 else 13)))
                                                for i in range(10) for e in (-1.795, 1.795)]
                   + [_pad(x, e, 0.28, 0.97, num=n) for x, e, n in ((-0.75, 2.795, '1'), (0.75, 2.795, '24'),
                                                                      (-0.75, -2.795, '12'), (0.75, -2.795, '13'))]
                   + [_pad(0, 0, 2.05, 3.1, num='12')]),
        'U20': _fp('lib:SON-10_doubled_lands',
                   [_pad(e * k, -1.0 + i * 0.5, w, 0.28, num=f"{i}{'LR'[e > 0]}")
                    for i in range(5) for e in (-1, 1) for k, w in ((1.76, 0.28), (1.475, 0.85))]),
        'U10': _fp('lib:Raspberry-Pi-4-Compute-Module',
                   [_pad(x, i * 0.4, 0.7, 0.2) for x in (-2.0, 1.08, 31.92, 35.0) for i in range(50)]),
        'U11': _fp('lib:UDFN-10_1.0x2.5mm_P0.5mm',
                   [_pad(x, i * 0.5, 0.25, 0.62) for x in (-0.388, 0.388) for i in (0, 1, 3, 4)]
                   + [_pad(-0.35, 1.0, 0.45, 0.7), _pad(0.412, 1.0, 0.45, 0.57)]),
        'U12': _fp('lib:GSG-LXES1TBCC2', [_pad(0, 0, 0.5, 1.1)]
                   + [_pad(x, y, 0.9, 0.3) for x in (-0.5, 0, 0.5) for y in (-0.95, 0.95)]),
        'U22': _fp('lib:BGA-381_pitch0.8mm',
                   [_pad(x * 0.8, y * 0.8, 0.4, 0.4, net_id=(x + y) % 3)
                    for x in range(-9, 10) for y in range(-9, 10)]
                   + [_pad(sx * 8.5, sy * 8.0, 0.127, 0.508, net_id=0)
                      for sx in (-1, 1) for sy in (-1, 1)]
                   + [_pad(sx * 8.0, sy * 8.5, 0.508, 0.127, net_id=0)
                      for sx in (-1, 1) for sy in (-1, 1)]),
    }
    got = {r: (t, p) for r, t, p in AP.fanout_parts(SimpleNamespace(footprints=parts))}
    expect(got.get('U1') == ('qfn', 0.5), f"QFN with an exposed pad: perimeter at 0.5 ({got.get('U1')})")
    expect(got.get('U2') == ('qfn', 0.5), f"LQFP-64: perimeter at 0.5 ({got.get('U2')})")
    expect(got.get('U3') == ('bga', 0.65), f"0.8 x 0.65 grid: each axis on its own lattice, pitch 0.65 ({got.get('U3')})")
    expect(got.get('U4') == ('bga', 1.0), f"16 x 16 grid at 1.0, depth 8 ({got.get('U4')})")
    expect('U6' not in got, "LQFP-32 at 0.8 with 32 pads: a perimeter, under the >40-pad bar")
    expect(got.get('U9') == ('bga', 0.8), f"DDR3 ball map with its empty middle columns ({got.get('U9')})")
    expect(got.get('U13') == ('bga', 1.5), f"a column jittered by 1 um is one column ({got.get('U13')})")
    expect(got.get('U22') == ('bga', 0.8), f"net-less corner markers off the ball lattice are set aside, "
                                           f"net-less balls kept ({got.get('U22')})")
    expect(got.get('U14') == ('qfn', 0.5), f"aQFN: a second staggered ring is a perimeter ({got.get('U14')})")
    expect(got.get('U15') == ('qfn', 0.5), f"corner pads set in from the sides ({got.get('U15')})")
    expect(got.get('U16') == ('qfn', 0.4), f"paste-only apertures are not pads ({got.get('U16')})")
    expect(got.get('U17') == ('qfn', 0.5), f"an exposed pad measured in the footprint's frame ({got.get('U17')})")
    expect(got.get('U18') == ('qfn', 0.4), f"a B.Cu pad does not swallow F.Cu pads ({got.get('U18')})")
    expect(got.get('U21') == ('qfn', 0.5), f"an exposed pad numbered like a pin leaves that pin a pin ({got.get('U21')})")
    expect(got.get('U10') == ('mezz', 0.4), f"a compute module's two connectors: a mezzanine at 0.4 ({got.get('U10')})")
    for r, why in (('U5', 'a two-row DFN'), ('J1', 'a connector by name, whatever its pads'),
                   ('J2', 'an M.2 socket'), ('J3', 'two rows, whatever its name'),
                   ('T1', 'magnetics by name'), ('RN1', "a resistor array's corner pads are not sides"),
                   ('U7', 'a through-hole socket'), ('U8', 'pads that form no shape'),
                   ('U11', 'two rows with an offset centre pad are not four columns'),
                   ('U12', 'two rows and a centre pad are not a 3 x 3 grid'),
                   ('U19', "an exposed pad's fingers at the package edge are not sides"),
                   ('U20', 'a two-row part whose pins are each drawn as two lands')):
        expect(r not in got, f"{r} not fanned out: {why}")


def test_analyze():
    print("analyze")
    a = AP.analyze(BOARD)
    expect(a['layers'] == ['F.Cu', 'B.Cu'], "splitflap: 2 layers")
    expect(a['planes'] == [('GND', 'B.Cu')], f"splitflap: GND on B.Cu ({a['planes']})")
    expect(a['parts'] == [] and a['grid'] == 0.1, "splitflap: no fanout, 0.1 grid")
    expect(a['plane_nets'][0] in a['power_nets'], "plane net is a power net")
    expect(all(n in a['globs'] for n in a['power_nets'] + a['pair_nets']),
           "every net the chain names has a verified pattern")
    expect(a['clearance'] >= a['fab_clearance'] > 0, "clearance at or above the fab floor")

    b = AP.analyze(os.path.join(ROOT, 'kicad_files', 'orangecrab_ext_pll.kicad_pcb'))
    L = b['layers']
    expect(len(L) == 6 and [l for _, l in b['planes']] == [L[1], L[4]],
           f"orangecrab: 6 layers, planes on In1 and In4, next to every signal layer ({b['planes']})")
    expect(b['planes'][0][0] == 'GND' and b['planes'][1][0] != 'GND',
           f"orangecrab: GND, then the dominant rail ({b['planes']})")
    expect(b['layer_costs'] == [1.0, -1.0, 1.0, 1.0, -1.0, 1.0],
           f"orangecrab: signals kept off the planes ({b['layer_costs']})")
    b3 = AP.analyze(os.path.join(ROOT, 'kicad_files', 'orangecrab_ext_pll.kicad_pcb'), plane_cost=3.0)
    expect(b3['layer_costs'] == [1.0, 3.0, 1.0, 1.0, 3.0, 1.0],
           f"orangecrab: a positive plane cost prices the planes instead ({b3['layer_costs']})")
    tools = {r: t for r, t, _ in b['parts']}
    expect(tools.get('U1') == 'bga' and 'qfn' in tools.values(),
           f"orangecrab: BGAs and fine-pitch QFNs fanned out ({tools})")
    expect(b['grid'] == 0.05 and b['congested'], "orangecrab: fine grid, congested")

    c = AP.analyze(os.path.join(ROOT, 'kicad_files', 'glasgow_revC.kicad_pcb'))
    expect({'/IO_Banks/VIOA', '/IO_Banks/VIOB'} <= set(c['power_nets']),
           f"glasgow: IO bank supplies no pattern names are rails by their power pins ({c['power_nets']})")


class ScriptedChain:
    """Chain.run and Chain.grade replaced by a script: per pass
    (grade or None, iterations or None); `input` grades the starting board."""

    def __init__(self, passes, input_grade=None):
        self.passes = list(passes)
        self.input_grade = input_grade
        self.last_iterations = None
        self.runs = 0
        self.cur = 'in'
        self.steps = []

    def grade(self, board):
        if board == 'in':
            return self.input_grade
        return self.passes[int(board[3:]) - 1][0]

    def run(self, kind, args, tag):
        self.runs += 1
        self.last_iterations = self.passes[self.runs - 1][1]
        self.steps.append({'tag': tag, 'ok': True})
        return f'out{self.runs}', {}, {}

    def accept(self, out):
        self.cur = out


def G(n_open, drc=0):
    return {'open': n_open, 'drc': drc, 'open_nets': []}


def _passes(script, effort_factor=2.0, finish=False, input_grade=None):
    a = dict(clearance=0.1, track=0.1, via=0.45, drill=0.2, grid=0.1,
             power_nets=[], congested=False, globs={})
    ch = ScriptedChain(script, input_grade)
    _, best, g = AP.route_passes(ch, a, ['F.Cu', 'B.Cu'], ['1', '1.5'],
                                 effort_factor, finish=finish)
    return ch.runs, best, g and (g['open'], g['drc'])


def test_passes():
    print("route_passes")
    expect(_passes([(G(0), 100)] * 3) == (1, 'out1', (0, 0)), "zero open: one pass")
    expect(_passes([(G(5), 100), (G(3), 50), (G(2), 40)]) == (3, 'out3', (2, 0)),
           "improving within budget: three passes")
    expect(_passes([(G(5), 100), (G(5), 50), (G(1), 10)]) == (2, 'out1', (5, 0)),
           "a pass no better than the best is not kept and ends the passes")
    expect(_passes([(G(5, 2), 100), (G(4, 3), 50), (G(1), 10)]) == (2, 'out1', (5, 2)),
           "fewer open nets bought with more DRC is not kept")
    expect(_passes([(G(5, 3), 100), (G(5, 1), 50), (G(5, 1), 10)]) == (3, 'out2', (5, 1)),
           "fewer DRC at the same open count is kept")
    expect(_passes([(G(5), 100), (G(3), 120), (G(1), 10)]) == (2, 'out2', (3, 0)),
           "100 + 120 + 120 > 2 x 100: pass 3 not started")
    expect(_passes([(G(5), 100), (G(3), 120), (G(1), 10)], effort_factor=3.4) == (3, 'out3', (1, 0)),
           "100 + 120 + 120 = 340 <= 3.4 x 100: the forecast admits equality")
    expect(_passes([(G(5), 100), (G(3), None), (G(1), 10)]) == (2, 'out2', (3, 0)),
           "a pass that reports no iterations starts no further pass")
    expect(_passes([(G(5), None), (G(3), 10)]) == (1, 'out1', (5, 0)),
           "a first pass with no iterations gives no budget")
    expect(_passes([(G(5), 0), (G(3), 0), (G(1), 0)]) == (3, 'out3', (1, 0)),
           "zero reported iterations: 0 + 0 <= 2 x 0, the passes run")
    expect(_passes([(None, 100)]) == (1, 'out1', None),
           "an ungraded first pass still ships (full chain)")

    print("route_passes --finish")
    expect(_passes([(G(30, 71), 100)], finish=True, input_grade=G(30, 53)) == (1, 'in', (30, 53)),
           "a pass adding DRC without closing a net loses to the input")
    expect(_passes([(G(31, 0), 100)], finish=True, input_grade=G(30, 5)) == (1, 'in', (30, 5)),
           "a pass opening a net loses to the input")
    expect(_passes([(G(2, 1), 100), (G(1, 1), 50), (G(1, 1), 50)], finish=True,
                   input_grade=G(4, 1)) == (3, 'out2', (1, 1)),
           "passes that beat the input are kept, a tie is not")
    expect(_passes([(G(2), 100)], finish=True, input_grade=G(0, 3)) == (0, 'in', (0, 3)),
           "a fully connected input runs no pass")
    expect(_passes([(G(2), 100)], finish=True, input_grade=None) == (0, None, None),
           "an input that cannot be graded runs no pass")


def test_plane_layers():
    print("plane_layers")
    for n, want in ((2, []), (4, [1, 2]), (6, [1, 4]), (8, [1, 3, 6]), (10, [1, 2, 5, 8])):
        got = AP.plane_layers(n)
        expect(got == want, f"{n} layers: planes at {want} ({got})")
        expect(n < 4 or all(i in got or i - 1 in got or i + 1 in got for i in range(n)),
               f"{n} layers: every layer is a plane or next to one")
    for bad in ('nan', '0'):
        check([sys.executable, '-X', 'utf8', SCRIPT, BOARD, os.devnull + '.kicad_pcb',
               '--plane-layer-cost', bad], refuse='must be a finite nonzero number', code=2,
              allow=('error: argument',))
        expect(True, f"--plane-layer-cost {bad} refused for its reason")


def test_effort_factor():
    print("--effort-factor")
    for bad in ('nan', 'inf', '-1'):
        check([sys.executable, '-X', 'utf8', SCRIPT, BOARD, os.devnull + '.kicad_pcb',
               '--effort-factor', bad], refuse='must be a finite number >= 0', code=2,
              allow=('error: argument',))
        expect(True, f"--effort-factor {bad} refused for its reason")


def _connected(board):
    r = subprocess.run([sys.executable, '-X', 'utf8',
                        os.path.join(ROOT, 'py_router', 'check_connected.py'),
                        board], capture_output=True, text=True, timeout=600)
    return r.returncode, r.stdout + r.stderr


def _autoplan(*argv):
    return subprocess.run([sys.executable, '-X', 'utf8', SCRIPT, *argv],
                          capture_output=True, text=True, timeout=1800)


def test_run():
    print("end to end on splitflap_driver")
    with tempfile.TemporaryDirectory() as tmp:
        out = os.path.join(tmp, 'routed.kicad_pcb')
        r = _autoplan(BOARD, out)
        expect(r.returncode == 0, f"autoplan exits 0 (rc={r.returncode})")
        if r.returncode:
            print(r.stdout[-3000:], r.stderr[-3000:])
            return
        evidence(out, 'routed board')
        expect(os.path.exists(out[:-len('.kicad_pcb')] + '.kicad_pro'),
               "the routed board ships with its .kicad_pro")
        plan = json.load(open(out + '.autoplan/plan.json'))
        tags = [s['tag'] for s in plan['steps']]
        expect(tags[0] == 'planes' and tags[1] == 'route',
               f"planes first, then route ({tags})")
        expect(plan['grade'] is not None and plan['grade']['open'] == 0,
               f"the shipped board's grade is recorded ({plan['grade']})")
        rc, text = _connected(out)
        expect(rc == 0, "check_connected: fully routed")
        if rc:
            print(text[-2000:])

        fin = os.path.join(tmp, 'finished.kicad_pcb')
        r = _autoplan(out, fin, '--finish')
        expect(r.returncode == 0, f"--finish exits 0 (rc={r.returncode})")
        if r.returncode:
            print(r.stdout[-3000:], r.stderr[-3000:])
            return
        evidence(fin, 'finished board')
        fplan = json.load(open(fin + '.autoplan/plan.json'))
        expect([s['tag'] for s in fplan['steps']] == ['input'],
               "--finish on a fully connected board grades it and runs no pass")
        expect(('GND', 'B.Cu') in [tuple(p) for p in fplan['analysis']['planes']],
               "--finish reads the poured plane from the board's pours")
        expect(_connected(fin)[0] == 0, "check_connected: still fully routed")

        print("input inside the output's work directory, stale sibling at the output")
        inner = [s for s in plan['steps'] if s['tag'] == 'route'][0]['log']
        inner = os.path.join(out + '.autoplan', inner[:-len('.log')] + '.kicad_pcb')
        evidence(inner, 'step board inside the work directory')
        stale = out[:-len('.kicad_pcb')] + '.kicad_dru'
        open(stale, 'w').write('(version 1)\n(rule stale (constraint clearance (min 5mm)))\n')
        r = _autoplan(inner, out, '--finish')
        expect(r.returncode == 0 and 'Traceback' not in r.stdout + r.stderr,
               f"a board inside <output>.autoplan/ is staged before the directory is cleared (rc={r.returncode})")
        if r.returncode:
            print(r.stdout[-3000:], r.stderr[-3000:])
            return
        expect(not os.path.exists(stale), "a sibling the shipped board lacks is removed")
        expect(_connected(out)[0] == 0, "check_connected: the re-shipped board is fully routed")


def main():
    test_glob()
    test_fanout_parts()
    test_analyze()
    test_passes()
    test_plane_layers()
    test_effort_factor()
    test_run()
    if FAILS:
        print(f"FAIL: {len(FAILS)} check(s)")
        return 1
    print("PASS: autoplan")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
