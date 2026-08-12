#!/usr/bin/env python3
"""Three channels that carried information nobody meant to send.

1. `fence_audit` compared POSES and never opened copper. A subject staged from
   a routed corpus board keeps the human routing while the parts move out from
   under it, so the track ends go on marking where each part BELONGS. Run 14
   shipped exactly that and the audit said CLEAN; the leak was closed later, by
   a copper strip done for an unrelated reason.

2. `placement.writer.write_placed_output` rewrites `(at)` with six decimals for
   exactly the footprints it is handed. That made the perturbed block readable
   off the output text with no tooling and no truth -- 24 six-decimal forms of
   235 on run 14's subject, against 0 on untouched corpus boards -- while
   `perturb`'s own docstring said the block was disclosed nowhere.

3. `route_planes`' summary de-duplicated `plane_nets` to a name list, so a pour
   onto two layers recorded one zone and the second vanished from the machine
   channel entirely.
"""
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'tests', 'stress'))

BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')
#: The copper fence needs a subject that CARRIES copper, which is the whole
#: scenario: a board staged from a routed corpus board and never stripped.
#: splitflap_driver is the stripped form and has none, so it cannot express the
#: leak at all -- using it here would have made the test pass by having nothing
#: to find.
ROUTED_BOARD = os.path.join(ROOT, 'kicad_files',
                            'rp2350_fpga_eensy_prePlane.kicad_pcb')


def six_decimal_ats(path):
    """How many `(at x y)` forms carry the writer's six-decimal signature."""
    with open(path, encoding='utf-8') as f:
        text = f.read()
    return sum(1 for x, _y in re.findall(r'\(at (-?\d+\.?\d*) (-?\d+\.?\d*)', text)
               if '.' in x and len(x.split('.')[-1]) == 6)


class CopperFenceTest(unittest.TestCase):
    """fence_audit must see a placement carried in copper."""

    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix='fence15_')
        self.wd = os.path.join(self.tmp, 'work')
        os.makedirs(self.wd)

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def _audit(self, mode='create', as_json=False):
        cmd = [sys.executable, '-X', 'utf8',
               os.path.join(ROOT, 'tests', 'stress', 'fence_audit.py'),
               '--control', self.control, '--workdir', self.wd, '--mode', mode]
        if as_json:
            cmd.append('--json')
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=900)
        return r.returncode, r.stdout + r.stderr

    def _stage(self, kind='translate', dose=6.0):
        """A real perturbation, with the source's copper left in place."""
        import placement.perturb as P
        self.control = os.path.join(self.tmp, 'control.kicad_pcb')
        out = os.path.join(self.wd, 'board.kicad_pcb')
        P.perturb(ROUTED_BOARD, out, kind=kind, dose_mm=dose, seed=99,
                  write_record=False, control_out=self.control)
        return out

    def test_a_stripped_board_is_clean(self):
        out = self._stage()
        # The subject as the rig SHOULD hand it over: no copper.
        subprocess.run([sys.executable, '-X', 'utf8',
                        os.path.join(ROOT, 'tests', 'stress',
                                     'strip_copper_only.py'), out, out + '.s'],
                       capture_output=True, text=True, timeout=900)
        shutil.move(out + '.s', out)
        code, txt = self._audit()
        self.assertIn('CLEAN', txt, txt[-500:])

    def test_copper_that_marks_displaced_pads_is_a_leak(self):
        """The run-14 case: the perturbed board still carrying its routing."""
        self._stage()
        code, txt = self._audit()
        self.assertIn('LEAK', txt, txt[-600:])
        self.assertIn('COPPER', txt,
                      'and it must say WHICH channel carried it, because the '
                      'fix differs: poses mean a control got copied in, copper '
                      'means the subject was never stripped')

    def test_create_mode_withholds_the_counts(self):
        """The counts ARE the block size, and the fence is still up."""
        self._stage()
        _c, txt = self._audit()
        self.assertIn('COPPER', txt)
        self.assertNotIn('displaced pads still have copper', txt,
                         'create mode must name the channel without the number')
        _c, js = self._audit(as_json=True)
        payload = json.loads(js[js.index('{'):])
        for row in payload.get('leaks', []):
            for k in ('copper_displaced_pads', 'copper_hits_on_control',
                      'copper_frac'):
                self.assertNotIn(k, row, k + ' discloses the block size')

    def test_audit_mode_does_show_them(self):
        self._stage()
        self._audit(mode='create')          # lay down the manifest
        _c, txt = self._audit(mode='audit')
        self.assertIn('displaced pads still have copper', txt,
                      'once the fence is down the number is what you need')


class AtFormattingLeakTest(unittest.TestCase):
    """The moved set must not be readable off the output text."""

    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix='atleak15_')

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def _perturb(self):
        import placement.perturb as P
        out = os.path.join(self.tmp, 'out.kicad_pcb')
        ctl = os.path.join(self.tmp, 'ctl.kicad_pcb')
        rec = P.perturb(BOARD, out, kind='translate', dose_mm=4.0, seed=1234,
                        write_record=False, control_out=ctl)
        return out, ctl, rec

    def test_the_signature_does_not_count_the_block(self):
        out, ctl, rec = self._perturb()
        from kicad_parser import parse_kicad_pcb
        total = len(parse_kicad_pcb(out).footprints)
        moved = len(rec['block']['members'])
        self.assertGreater(total, moved, 'fixture must not move every part')
        self.assertEqual(six_decimal_ats(out), total,
                         'every footprint goes through the same formatter, so '
                         'the count is the board, not the draw')
        self.assertNotEqual(six_decimal_ats(out), moved,
                            'this is the number that used to be readable')

    def test_the_control_is_formatted_the_same_way(self):
        out, ctl, _rec = self._perturb()
        self.assertEqual(six_decimal_ats(out), six_decimal_ats(ctl),
                         'a control formatted differently from its subject is '
                         'itself a comparison anyone could make')

    def test_unmoved_parts_keep_their_pose_and_rotation_exactly(self):
        _out, ctl, _rec = self._perturb()
        from kicad_parser import parse_kicad_pcb
        a, c = parse_kicad_pcb(BOARD), parse_kicad_pcb(ctl)
        for ref, fa in a.footprints.items():
            fc = c.footprints.get(ref)
            self.assertIsNotNone(fc, ref)
            self.assertAlmostEqual(fa.x, fc.x, places=6, msg=ref)
            self.assertAlmostEqual(fa.y, fc.y, places=6, msg=ref)
            # make_state normalises -90 to 270; writing that back would be a
            # 360 delta to anything comparing the numbers unnormalised.
            self.assertAlmostEqual(fa.rotation, fc.rotation, places=6, msg=ref)


class PlaneZoneChannelTest(unittest.TestCase):
    """A pour onto two layers must appear as two zones."""

    def test_summary_names_every_zone_not_the_deduplicated_net(self):
        tmp = tempfile.mkdtemp(prefix='pz15_')
        try:
            out = os.path.join(tmp, 'p.kicad_pcb')
            r = subprocess.run(
                [sys.executable, '-X', 'utf8',
                 os.path.join(ROOT, 'py_router', 'route_planes.py'), BOARD, out,
                 '--nets', 'GND', 'GND',
                 '--plane-layers', 'F.Cu', 'B.Cu', '--deadline', '240'],
                capture_output=True, text=True, timeout=900)
            line = [l for l in (r.stdout + r.stderr).splitlines()
                    if l.startswith('JSON_SUMMARY:')]
            self.assertTrue(line, (r.stdout + r.stderr)[-800:])
            d = json.loads(line[-1].split('JSON_SUMMARY: ', 1)[1])
            self.assertEqual(d.get('plane_nets'), ['GND'],
                             'the de-duplicated key stays as it was')
            zones = d.get('plane_zones')
            self.assertEqual(len(zones or []), 2,
                             'both poured zones must be in the record')
            self.assertEqual([z['layer'] for z in zones], ['F.Cu', 'B.Cu'])
            self.assertIn('pads_scope', d,
                          'and the aggregated pad tally must say it is one '
                          'tally over every zone')
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


if __name__ == '__main__':
    unittest.main(verbosity=2)
