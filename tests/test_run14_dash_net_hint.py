#!/usr/bin/env python3
"""A net whose NAME starts with '-' must not cost a lap (run 14).

castor_pollux has a net called `-12V`. `route.py --nets ... -12V ...` is
`nargs='+'`, so argparse read the name as a flag:

    route.py: error: unrecognized arguments: -12V

exit 2, no board, no JSON_SUMMARY. Both arms of run 14's plane-mapping A/B died
that way and the lap had to be re-run. The fix is one character at the call site
(`--nets=-12V`) but nothing said so, and thirty-odd sibling list flags across six
tools share the shape.

`cli_banner.install()` -- which every instrument already calls -- now patches
`ArgumentParser.error` once so the hint appears wherever it happens. These tests
pin that the hint fires, that it does not fire on unrelated errors, that it never
masks the real error, and that the workaround it recommends actually works.
"""
import argparse
import io
import contextlib
import os
import subprocess
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # #522/py_placer layout

import cli_banner  # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')


class DashHintUnitTest(unittest.TestCase):

    def setUp(self):
        cli_banner.install_dash_hint()
        self.p = argparse.ArgumentParser(prog='t', add_help=False)
        self.p.add_argument('--nets', nargs='+')

    def _err(self, argv):
        buf = io.StringIO()
        with contextlib.redirect_stderr(buf):
            with self.assertRaises(SystemExit):
                self.p.parse_args(argv)
        return buf.getvalue()

    def test_hint_fires_on_a_leading_dash_value(self):
        out = self._err(['--nets', '+12V', '-12V'])
        self.assertIn("'-12V'", out)
        self.assertIn('--nets=-12V', out)

    def test_the_real_argparse_error_still_prints(self):
        out = self._err(['--nets', '+12V', '-12V'])
        self.assertIn('unrecognized arguments', out,
                      'the hint must add to the error, never replace it')

    def test_no_hint_on_an_unrelated_error(self):
        p = argparse.ArgumentParser(prog='t', add_help=False)
        p.add_argument('--size', type=int)
        buf = io.StringIO()
        with contextlib.redirect_stderr(buf):
            with self.assertRaises(SystemExit):
                p.parse_args(['--size', 'banana'])
        self.assertNotIn('begins with', buf.getvalue())

    def test_patch_is_idempotent(self):
        before = argparse.ArgumentParser.error
        cli_banner.install_dash_hint()
        cli_banner.install_dash_hint()
        self.assertIs(argparse.ArgumentParser.error, before,
                      'install must not stack wrappers')


class DashHintCliTest(unittest.TestCase):
    """Through a real instrument, as the run actually hit it."""

    def _run(self, args):
        return subprocess.run([sys.executable, '-X', 'utf8',
                               os.path.join(ROOT, 'py_router', 'route.py')] + args,
                              capture_output=True, text=True, timeout=600)

    def test_route_py_emits_the_hint_and_still_exits_2(self):
        r = self._run([BOARD, os.path.join(ROOT, '_no_such_out.kicad_pcb'),
                       '--nets', '+12V', '-12V'])
        self.assertEqual(r.returncode, 2)
        both = r.stdout + r.stderr
        self.assertIn('--nets=-12V', both)
        self.assertIn('unrecognized arguments', both)

    def test_the_recommended_form_parses(self):
        """The hint must recommend something that actually works.

        Asserted on the MESSAGE, not the exit code: this fixture board has no
        `-12V`, so route.py rightly refuses with its own exit 2 ("none of the
        requested nets exist"). That refusal is proof the value got through
        argparse -- which is the whole claim. Checking the code alone would
        confuse two different exit 2s.
        """
        r = self._run([BOARD, os.path.join(ROOT, '_no_such_out.kicad_pcb'),
                       '--nets=-12V', '--skip-routing'])
        both = r.stdout + r.stderr
        self.assertNotIn('unrecognized arguments', both,
                         'the documented workaround must not be an argparse '
                         'error: ' + both[-300:])
        self.assertIn('-12V', both,
                      'the net name must reach the tool as a VALUE')
        self.assertNotIn('begins with', both,
                         'and the hint must not fire when nothing is wrong')


if __name__ == '__main__':
    unittest.main(verbosity=2)
