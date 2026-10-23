#!/usr/bin/env python3
"""#1251: a differential pair that runs through per-leg series parts.

`board_context`'s pair rows existed only for two parts that each carry BOTH
nets of a pair. A pair through AC caps or USB series resistors has no such two
parts, so ulx3s (14 pairs), rp2350_fpga_eensy_prePlane and cap_chain had no
pair row at all, and orangecrab_ext_pll had its DDR pairs and missed the
longer USB pair.

Underneath, `diff_xnet.find_ac_coupled_xnets` (#196) -- the router's junction
rule, reused here -- counted `fp.pads`, so a 0201 resistor drawn with two
paste-only windows beside its copper pads had four "pads" and never bridged:
rp2350's R4/R5 and orangecrab's R21/R29. It now counts
`kicad_parser.non_aperture_pads` (#1143/#1148's rule).

Pinned here:
  * the router's rule: the tracked boards resolve the issue's XNets, and a
    part with two copper pads plus paste windows bridges where three copper
    pads do not;
  * `board_context.composed_pair_rows`: a three-pair chain is walked end to
    end, a branch is disclosed rather than composed, and an end two parts
    carry whole is said rather than chosen;
  * `pair_order.composed_pair_metrics` IS `pair_metrics` when the two ends
    name the same nets, and reverses its verdict when one end's legs swap;
  * the tracked boards reproduce the issue's leg table (P / N / |P-N|), and
    the pair census names the pairs left out and why.

Run: python3 -X utf8 tests/test_1251_series_part_pairs.py
"""
import contextlib
import io
import os
import sys
import unittest
from types import SimpleNamespace

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from kicad_parser import parse_kicad_pcb                       # noqa: E402
from diff_xnet import find_ac_coupled_xnets                    # noqa: E402
from net_queries import find_differential_pairs                # noqa: E402
from routing_config import DiffPairNet                         # noqa: E402
from run_utils import evidence                                 # noqa: E402
import board_context                                           # noqa: E402

BOARDS = os.path.join(ROOT, 'kicad_files')
RP = os.path.join(BOARDS, 'rp2350_fpga_eensy_prePlane.kicad_pcb')
OC = os.path.join(BOARDS, 'orangecrab_ext_pll.kicad_pcb')
ULX = os.path.join(BOARDS, 'ulx3s.kicad_pcb')
CAP = os.path.join(BOARDS, 'cap_chain.kicad_pcb')
ESP = os.path.join(BOARDS, 'esp_prog.kicad_pcb')
QFN = os.path.join(BOARDS, 'qfn_diffpair_escape.kicad_pcb')

_PCB = {}
_ROWS = {}


def _pcb(path):
    if path not in _PCB:
        evidence(path, 'tracked board')
        with contextlib.redirect_stdout(io.StringIO()):
            _PCB[path] = parse_kicad_pcb(path)
    return _PCB[path]


def _xnet_refs(path):
    pcb = _pcb(path)
    dp = find_differential_pairs(pcb, ['*'])
    xnets, _warnings = find_ac_coupled_xnets(pcb, dp)
    return sorted(x.bridge_refs for x in xnets)


def _pair_rows(path):
    """`board_context.pin_order_rows`' document on a tracked board."""
    if path not in _ROWS:
        from list_nets import board_floor_knobs
        with contextlib.redirect_stdout(io.StringIO()), \
                contextlib.redirect_stderr(io.StringIO()):
            _ROWS[path] = board_context.pin_order_rows(
                _pcb(path), path, board_floor_knobs(path, clearance=None)[0])
    return _ROWS[path]


def _spanned(path):
    """The pair-scoped rows that carry a span, as `scope` `pair A/B...`."""
    return [r for r in _pair_rows(path)['rows']
            if str(r.get('scope') or '').startswith('pair ')
            and isinstance(r.get('span_mm'), (int, float))]


def _state(path):
    """The QuenchState `board_context.pin_order_rows` builds."""
    from list_nets import board_floor_knobs
    from render_placement import PlacementModel
    with contextlib.redirect_stdout(io.StringIO()), \
            contextlib.redirect_stderr(io.StringIO()):
        clr = board_floor_knobs(path, clearance=None)[0]
        return PlacementModel(_pcb(path), path,
                              quench_kwargs={'clearance': clr}).state


# ------------------------------------------------------------- the router

class TheRoutersJunctionRule(unittest.TestCase):

    def test_paste_only_windows_no_longer_hide_a_series_resistor(self):
        self.assertEqual(_xnet_refs(RP), [['R4', 'R5']])
        self.assertEqual(_xnet_refs(OC), [['R21', 'R29']])

    def test_the_boards_that_already_resolved_are_unchanged(self):
        self.assertEqual(_xnet_refs(ULX), [
            ['C38', 'C39'], ['C40', 'C41'], ['C42', 'C43'], ['C44', 'C45'],
            ['R49', 'R50'], ['R52', 'R53']])
        self.assertEqual(_xnet_refs(CAP), [['C1', 'C2']])

    def test_the_count_is_copper_pads_not_apertures(self):
        def pad(net, layers=('F.Cu', 'F.Mask')):
            return SimpleNamespace(net_id=net, layers=list(layers),
                                   pad_type='smd', drill=0.0)

        def part(*pads):
            return SimpleNamespace(pads=list(pads), dnp=False)

        paste = ('F.Paste',)
        pairs = {b: DiffPairNet(base_name=b, p_net_id=p, n_net_id=n,
                                p_net_name=b + '_P', n_net_name=b + '_N')
                 for b, p, n in (('A', 1, 2), ('B', 3, 4))}
        windows = part(pad(0, paste), pad(0, paste), pad(1), pad(3))
        board = SimpleNamespace(footprints={
            'R1': windows, 'R2': part(pad(0, paste), pad(0, paste),
                                      pad(2), pad(4))})
        xnets, _w = find_ac_coupled_xnets(board, pairs)
        self.assertEqual([x.bridge_refs for x in xnets], [['R1', 'R2']])
        # Three COPPER pads is not a series part, windows or not.
        board.footprints['R2'] = part(pad(0, paste), pad(2), pad(4), pad(9))
        xnets, _w = find_ac_coupled_xnets(board, pairs)
        self.assertEqual(xnets, [])


# ------------------------------------------------------------ the composer

_NETS = {'A_P': 1, 'A_N': 2, 'B_P': 3, 'B_N': 4, 'C_P': 5, 'C_N': 6,
         'D_P': 7, 'D_N': 8}
_FOUND = [('A_P', 'A_N'), ('B_P', 'B_N'), ('C_P', 'C_N')]


def _pad(net, x, y):
    return SimpleNamespace(net_id=_NETS[net], net_name=net, global_x=x,
                           global_y=y, layers=['F.Cu', 'F.Mask'],
                           pad_type='smd', drill=0.0, pinfunction='')


def _part(*pads):
    return SimpleNamespace(pads=list(pads), dnp=False,
                           footprint_name='t:part')


def _chain_board():
    """J1 -> C1/C2 -> C3/C4 -> J2 along x, the P leg at y=0 and N at y=1.
    Every leg is a straight line, so each is exactly its x extent."""
    return SimpleNamespace(footprints={
        'J1': _part(_pad('A_P', 0, 0), _pad('A_N', 0, 1)),
        'C1': _part(_pad('A_P', 10, 0), _pad('B_P', 11, 0)),
        'C2': _part(_pad('A_N', 10, 1), _pad('B_N', 11, 1)),
        'C3': _part(_pad('B_P', 20, 0), _pad('C_P', 21, 0)),
        'C4': _part(_pad('B_N', 20, 1), _pad('C_N', 21, 1)),
        'J2': _part(_pad('C_P', 30, 0), _pad('C_N', 30, 1))})


def _compose(board, found=_FOUND):
    no_state = SimpleNamespace(parts={})
    return board_context.composed_pair_rows(board, no_state, found, _NETS)


class TheComposer(unittest.TestCase):

    def test_a_chain_of_three_pairs_is_walked_end_to_end(self):
        rows, composed, _w, unchained = _compose(_chain_board())
        self.assertEqual(unchained, [])
        self.assertEqual(sorted(composed), ['A_P/A_N', 'B_P/B_N', 'C_P/C_N'])
        self.assertEqual(len(rows), 1)
        r = rows[0]
        self.assertEqual((r['a'], r['b']), ('J1', 'J2'))
        self.assertEqual(r['scope'], 'pair A_P/A_N + B_P/B_N + C_P/C_N')
        self.assertEqual(r['series'], {'P': ['C1', 'C3'], 'N': ['C2', 'C4']})
        self.assertEqual(r['legs_mm'], {'P': 30.0, 'N': 30.0})
        self.assertEqual((r['leg_skew_mm'], r['span_mm']), (0.0, 30.0))
        # No QuenchState part: the verdict is not measured, never guessed.
        self.assertTrue(r['verdict'].startswith('NOT MEASURED'), r['verdict'])
        self.assertIn('stops at J1', r['note'])

    def test_a_longer_leg_is_the_span_and_the_skew_is_a_number(self):
        board = _chain_board()
        board.footprints['C3'] = _part(_pad('B_P', 20, 4), _pad('C_P', 21, 4))
        rows, *_ = _compose(board)
        r = rows[0]
        self.assertEqual(r['legs_mm']['N'], 30.0)
        self.assertGreater(r['legs_mm']['P'], 30.0)
        self.assertEqual(r['span_mm'], r['legs_mm']['P'])
        self.assertAlmostEqual(r['leg_skew_mm'], r['legs_mm']['P'] - 30.0,
                               places=3)

    def test_a_branch_is_disclosed_not_composed(self):
        # B is bridged to A, C and D: three neighbours, so no single leg.
        board = _chain_board()
        board.footprints.update({
            'C5': _part(_pad('B_P', 15, 5), _pad('D_P', 16, 5)),
            'C6': _part(_pad('B_N', 15, 6), _pad('D_N', 16, 6))})
        rows, composed, _w, unchained = _compose(
            board, _FOUND + [('D_P', 'D_N')])
        self.assertEqual(rows, [])
        self.assertEqual(composed, set())
        self.assertEqual(len(unchained), 1)
        self.assertIn('branch', unchained[0]['why'])

    def test_an_end_two_parts_carry_whole_is_said_not_chosen(self):
        board = _chain_board()
        board.footprints['U9'] = _part(_pad('C_P', 40, 0), _pad('C_N', 40, 1))
        rows, *_ = _compose(board)
        r = rows[0]
        self.assertIsNone(r['span_mm'])
        self.assertEqual(r['b'], 'J2,U9')
        self.assertIn('carried whole by J2, U9', r['note'])
        self.assertTrue(r['verdict'].startswith('NOT MEASURED'))


# -------------------------------------------------------------- the verdict

class TheComposedVerdict(unittest.TestCase):

    def test_it_is_pair_metrics_when_both_ends_name_the_same_nets(self):
        from placement.pair_order import composed_pair_metrics, pair_metrics
        pcb, st = _pcb(ESP), _state(ESP)
        ids = [n.net_id for n in pcb.nets.values()
               if n.name in ('/D_P', '/D_N')]
        self.assertEqual(len(ids), 2)
        want = pair_metrics(st, 'U1', 'USB1', only_nets=ids)
        self.assertIsNotNone(want)
        self.assertEqual(
            composed_pair_metrics(st, 'U1', 'USB1', sorted(ids), sorted(ids)),
            want)

    def test_swapping_one_ends_legs_reverses_the_order(self):
        from placement.pair_order import composed_pair_metrics
        pcb, st = _pcb(CAP), _state(CAP)
        n = {x.name: x.net_id for x in pcb.nets.values()}
        a = (n['DPA_P'], n['DPA_N'])
        straight = composed_pair_metrics(st, 'J1', 'J2', a,
                                         (n['DPB_P'], n['DPB_N']))
        crossed = composed_pair_metrics(st, 'J1', 'J2', a,
                                        (n['DPB_N'], n['DPB_P']))
        self.assertEqual((straight['ties'], crossed['ties']), (0, 0))
        self.assertEqual(sorted((straight['inversions'],
                                 crossed['inversions'])), [0, 1])


# ------------------------------------------------------- the tracked boards

#: The issue's table: composed pair -> (ends, series P, series N, P, N, |P-N|).
TABLE = {
    ULX: [('GPDI1', 'U1', ['C42'], ['C43'], 10.265, 11.718, 1.453),
          ('GPDI1', 'U1', ['C40'], ['C41'], 9.923, 11.762, 1.838),
          ('U1', 'US2', ['R53'], ['R52'], 33.249, 34.872, 1.623),
          ('U6', 'US1', ['R50'], ['R49'], 48.122, 47.195, 0.927)],
    RP: [('U3', 'U6', ['R4'], ['R5'], 7.017, 6.908, 0.109)],
    OC: [('J3', 'U3', ['R21'], ['R29'], 22.022, 23.152, 1.130)],
    CAP: [('J1', 'J2', ['C1'], ['C2'], 16.523, 16.039, 0.484)],
}


class TheTrackedBoards(unittest.TestCase):

    def test_the_issues_leg_table_is_reproduced(self):
        for path, want in TABLE.items():
            got = {(r['a'], r['b'], tuple(r['series']['P']),
                    tuple(r['series']['N'])):
                   (r['legs_mm']['P'], r['legs_mm']['N'], r['leg_skew_mm'])
                   for r in _spanned(path) if r.get('composed')}
            for a, b, sp, sn, lp, ln, skew in want:
                with self.subTest(board=os.path.basename(path), series=sp):
                    self.assertEqual(got.get((a, b, tuple(sp), tuple(sn))),
                                     (lp, ln, skew), sorted(got))

    def test_every_composed_board_now_has_its_rows(self):
        want = {ULX: 6, RP: 1, CAP: 1, OC: 1}
        for path, composed in want.items():
            with self.subTest(board=os.path.basename(path)):
                self.assertEqual(
                    sum(1 for r in _spanned(path) if r.get('composed')),
                    composed)
        # orangecrab's DDR rows are still there beside the composed USB pair.
        self.assertIn(('RN6', 'U3', 18.289),
                      [(r['a'], r['b'], r['span_mm']) for r in _spanned(OC)])

    def test_the_pairs_left_out_are_named_with_why(self):
        census = _pair_rows(ULX)['pairs']
        self.assertEqual(len(census['found']), 14)
        self.assertEqual(
            [(p['pair'], p['carried_whole_by'])
             for p in census['not_measured']],
            [('/serdes/SRCLKP//serdes/SRCLKN', ['U1']),
             ('USB_FPGA_PULL_D+/USB_FPGA_PULL_D-', ['U1'])])
        qfn = _pair_rows(QFN)
        self.assertEqual(len(qfn['pairs']['found']), 2)
        self.assertEqual(_spanned(QFN), [])

    def test_rp2350s_row_says_where_it_stops(self):
        (row,) = [r for r in _spanned(RP) if r.get('composed')]
        self.assertEqual([(s['ref'], s['copper_pads'])
                          for s in row['stops_at']], [('U3', 6), ('U6', 71)])
        self.assertIn('stops at U3', row['note'])
        self.assertIn('not followed', row['note'])

    def test_the_census_names_the_composed_pairs(self):
        self.assertEqual(_pair_rows(RP)['pairs']['composed'], [
            '/Power Supply and USB/D+//Power Supply and USB/D-',
            '/RP2354A/USB_D+//RP2354A/USB_D-'])


if __name__ == '__main__':
    unittest.main()
