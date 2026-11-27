#!/usr/bin/env python3
"""make_unplaced.py --only / --group: re-place some parts of a ROUTED board.

Without them a routed board is refused (exit 3), as before. With them only the
named parts move, and only their copper goes: every track end and via on a moved
pad, then what that leaves hanging, to a fixpoint. Copper between parts that
stay is kept.

Board: R1 and R3 stay, R2 moves.
  * net A: R1.1 -- track -- R3.1                       staying pair: kept
  * net B: R1.2 -- track -- R2.1                       to the moved part: goes
  * net D: R3.2 -- F.Cu -- via -- B.Cu -- via -- F.Cu -- R2.2
           cutting R2.2's stub leaves each via joined on one layer only, so the
           whole chain unwinds back to R3.2: goes
  * net E: a lone stub that already dangled in the input and
           touches no moved pad: kept as it was (only the moved parts' copper
           is touched)
"""
import os
import subprocess
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for _p in ('py_placer', 'py_router', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from kicad_parser import parse_kicad_pcb  # noqa: E402

SCRIPT = os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-placement', 'scripts',
                      'make_unplaced.py')
_UUID = iter(f'00000000-0000-0000-0000-{i:012d}' for i in range(1, 1000))


def _fp(ref, x, y, nets):
    (n1, a), (n2, b) = nets
    return (f'  (footprint "t:R" (layer "F.Cu") (uuid "{next(_UUID)}") (at {x} {y})\n'
            f'    (property "Reference" "{ref}" (at 0 -1.5) (layer "F.SilkS"))\n'
            '    (fp_rect (start -1.2 -0.7) (end 1.2 0.7) (stroke (width 0.05)'
            ' (type default)) (layer "F.CrtYd"))\n'
            f'    (pad "1" smd rect (at -0.6 0) (size 0.6 0.8) (layers "F.Cu") (net {n1} "{a}"))\n'
            f'    (pad "2" smd rect (at 0.6 0) (size 0.6 0.8) (layers "F.Cu") (net {n2} "{b}")))')


def _seg(x1, y1, x2, y2, net, layer='F.Cu'):
    return (f'  (segment (start {x1} {y1}) (end {x2} {y2}) (width 0.2) (layer "{layer}")'
            f' (net {net}) (uuid "{next(_UUID)}"))')


def _via(x, y, net):
    return (f'  (via (at {x} {y}) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu")'
            f' (net {net}) (uuid "{next(_UUID)}"))')


def board():
    global _UUID   # the group below names R2 by its uuid: number from 1 every time
    _UUID = iter(f'00000000-0000-0000-0000-{i:012d}' for i in range(1, 1000))
    lines = ['(kicad_pcb (version 20240108) (generator pcbnew)',
             '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))',
             '  (net 0 "") (net 1 "A") (net 2 "B") (net 4 "D") (net 5 "E")',
             '  (gr_rect (start 0 0) (end 30 20) (stroke (width 0.1) (type default))'
             ' (layer "Edge.Cuts"))']
    r1 = ('R1', 5, 10, ((1, 'A'), (2, 'B')))
    r2 = ('R2', 15, 10, ((2, 'B'), (4, 'D')))
    r3 = ('R3', 5, 15, ((1, 'A'), (4, 'D')))
    lines += [_fp(*r1), _fp(*r2), _fp(*r3)]
    lines.append(f'  (group "blk" (uuid "{next(_UUID)}") (members "00000000-0000-0000-0000-000000000002"))')
    lines += [
        _seg(4.4, 10, 4.4, 15, 1),                    # A: R1.1 -> R3.1 (stays)
        _seg(5.6, 10, 14.4, 10, 2),                   # B: R1.2 -> R2.1 (goes)
        _seg(5.6, 15, 10, 15, 4), _via(10, 15, 4),     # D: R3.2 -> via1
        _seg(10, 15, 15.6, 13, 4, 'B.Cu'), _via(15.6, 13, 4),  # -> via2
        _seg(15.6, 13, 15.6, 10, 4),                   # -> R2.2
        _seg(20, 3, 22, 3, 5),                         # E: dangled already, untouched
    ]
    lines.append(')')
    return '\n'.join(lines)


def run(*args, src_text=None):
    td = tempfile.mkdtemp()
    src, dst = os.path.join(td, 'in.kicad_pcb'), os.path.join(td, 'out.kicad_pcb')
    with open(src, 'w', encoding='utf-8') as fh:
        fh.write(src_text or board())
    r = subprocess.run([sys.executable, '-X', 'utf8', SCRIPT, src, dst, *args],
                       capture_output=True, text=True, cwd=ROOT, timeout=600)
    return r, (parse_kicad_pcb(dst) if os.path.exists(dst) else None)


def copper(pcb):
    segs = {(s.net_id, round(s.start_x, 2), round(s.start_y, 2), round(s.end_x, 2),
             round(s.end_y, 2)) for s in pcb.segments if not getattr(s, 'graphic', False)}
    vias = {(v.net_id, round(v.x, 2), round(v.y, 2)) for v in pcb.vias}
    return segs, vias


class TestMakeUnplacedOnly(unittest.TestCase):
    def test_routed_board_still_refused_without_only(self):
        r, _ = run()
        self.assertEqual(r.returncode, 3, r.stdout + r.stderr)
        self.assertIn('carries routed copper', r.stderr)

    def check_r2_moved(self, r, out):
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        f = out.footprints
        self.assertEqual((f['R1'].x, f['R1'].y), (5, 10))
        self.assertEqual((f['R3'].x, f['R3'].y), (5, 15))
        self.assertGreater(f['R2'].y, 20)                 # piled below the outline
        segs, vias = copper(out)
        self.assertEqual(segs, {(1, 4.4, 10, 4.4, 15), (5, 20, 3, 22, 3)})  # A and the old E
        self.assertEqual(vias, set())                     # D's chain unwound, both vias

    def test_only_moves_the_part_and_its_copper(self):
        self.check_r2_moved(*run('--only', 'R2'))

    def test_group_names_the_same_part(self):
        self.check_r2_moved(*run('--group', 'blk'))

    def test_unknown_ref_refused(self):
        r, _ = run('--only', 'R9')
        self.assertEqual(r.returncode, 2, r.stdout + r.stderr)

    def test_locked_copper_on_a_moved_pad_refused(self):
        text = board().replace('(segment (start 5.6 10) (end 14.4 10) (width 0.2) (layer "F.Cu")',
                               '(segment (locked yes) (start 5.6 10) (end 14.4 10) (width 0.2) (layer "F.Cu")')
        r, _ = run('--only', 'R2', src_text=text)
        self.assertEqual(r.returncode, 3, r.stdout + r.stderr)
        self.assertIn('locked', r.stderr)

    def test_branch_teeing_into_a_staying_track_stays(self):
        # R1.2 -> R4.1 stays; a branch Tees into its middle and runs on to R5.1:
        # its end at the T has no other track end, but nothing was removed there
        text = board().replace('  (group', '\n'.join([
            _fp('R4', 25, 4, ((2, 'B'), (0, ''))), _fp('R5', 20, 2, ((2, 'B'), (0, ''))),
            _seg(5.6, 10, 5.6, 4, 2), _seg(5.6, 4, 24.4, 4, 2),
            _seg(12, 4, 12, 2, 2), _seg(12, 2, 19.4, 2, 2), '  (group']), 1)
        r, out = run('--only', 'R2', src_text=text)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        segs, _ = copper(out)
        self.assertLessEqual({(2, 12, 4, 12, 2), (2, 12, 2, 19.4, 2), (2, 5.6, 4, 24.4, 4)}, segs)
        self.assertNotIn((2, 5.6, 10, 14.4, 10), segs)

    def test_teardrop_on_a_moved_pad_goes(self):
        td = ('  (zone (net {}) (net_name "{}") (layer "F.Cu") (uuid "{}") (name "$teardrop_padvia$")'
              ' (hatch full 0.1) (priority 30000) (attr (teardrop (type padvia)))'
              ' (connect_pads yes (clearance 0)) (min_thickness 0.0254) (filled_areas_thickness no)'
              ' (fill yes (thermal_gap 0.5) (thermal_bridge_width 0.5) (island_removal_mode 1))'
              ' (polygon (pts (xy {} {}) (xy {} {}) (xy {} {}) (xy {} {}))))')
        # on B's track at R2.1 (goes with it); on A's track R1.1 -> R3.1 (stays)
        moved = td.format(2, 'B', next(_UUID), 13.9, 9.9, 14.4, 9.6, 14.4, 10.4, 13.9, 10.1)
        staying = td.format(1, 'A', next(_UUID), 4.3, 10.9, 4.0, 10.4, 4.8, 10.4, 4.5, 10.9)
        text = board().replace('  (group', moved + '\n' + staying + '\n  (group', 1)
        r, out = run('--only', 'R2', src_text=text)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        tds = [z for z in out.zones if getattr(z, 'teardrop', False)]
        self.assertEqual([round(z.polygon[0][0], 2) for z in tds], [4.3])

    def test_copper_dead_in_the_input_stays(self):
        # a dangling via and stub on net B (a net the move touches) were dead
        # before; only the moved part's copper goes
        text = board().replace('  (group', '\n'.join([
            _seg(5.6, 10, 5.6, 7, 2), _via(5.6, 7, 2), '  (group']), 1)
        r, out = run('--only', 'R2', src_text=text)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        segs, vias = copper(out)
        self.assertIn((2, 5.6, 10, 5.6, 7), segs)
        self.assertIn((2, 5.6, 7), vias)

    def test_join_only_through_a_moved_part_is_reported(self):
        # R1.2 -> C7.1 -> R4.1, both tracks ending in C7.1: moving C7 takes
        # both (its copper) and reports the R1 / R4 join for re-routing
        text = '\n'.join([
            '(kicad_pcb (version 20240108) (generator pcbnew)',
            '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))',
            '  (net 0 "") (net 1 "A") (net 2 "B")',
            '  (gr_rect (start 0 0) (end 30 20) (stroke (width 0.1) (type default))'
            ' (layer "Edge.Cuts"))',
            _fp('R1', 5, 10, ((1, 'A'), (2, 'B'))), _fp('C7', 15, 10, ((2, 'B'), (1, 'A'))),
            _fp('R4', 25, 10, ((2, 'B'), (1, 'A'))),
            _seg(5.6, 10, 14.4, 10, 2), _seg(14.4, 10, 24.4, 10, 2), ')'])
        r, out = run('--only', 'C7', src_text=text)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertEqual(copper(out)[0], set())
        self.assertRegex(r.stdout, r're-route +net 2: R1, R4 were joined only through moved parts')


if __name__ == '__main__':
    unittest.main()
