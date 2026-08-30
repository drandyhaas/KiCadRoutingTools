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
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # #522/py_placer layout

import cli_banner  # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')


def _argparse_takes_dash_values():
    """Whether a BARE ArgumentParser accepts a dash-prefixed value.

    Python 3.14's argparse does, so on those interpreters `--nets -12V` is not
    an error at all and the failure the hint annotates cannot be provoked.
    Probed, not version-gated: what matters is the behaviour, not the number.

    #718 item 2 -- this governs the UNIT arms ONLY, and the docstring now says
    so because it once did not. A `bare` parser is the thing under test there;
    it is NOT what route.py builds. Since #597 (55632d91) every shipped CLI
    pins the matcher (`cli_nets.pin_dash_digit_values`) right before
    parse_args, so a dash-digit token is a VALUE on every interpreter and the
    error can never be provoked through a CLI. Gating a CLI arm on this probe
    measures the wrong parser -- which is exactly how an arm asserting
    `SystemExit 2` from route.py survived the fix that removed it.
    """
    p = argparse.ArgumentParser(prog='probe', add_help=False)
    p.add_argument('--nets', nargs='+')
    try:
        return p.parse_args(['--nets', '+12V', '-12V']).nets == ['+12V', '-12V']
    except SystemExit:
        return False


_HINT_NOT_PROVOCABLE = _argparse_takes_dash_values()
_SKIP_REASON = ("this Python's argparse accepts dash-prefixed values in "
                "nargs='+' lists (3.14+), so the error the hint annotates "
                "never happens here; the hint is inert by design")


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

    @unittest.skipIf(_HINT_NOT_PROVOCABLE, _SKIP_REASON)
    def test_hint_fires_on_a_leading_dash_value(self):
        out = self._err(['--nets', '+12V', '-12V'])
        self.assertIn("'-12V'", out)
        self.assertIn('--nets=-12V', out)

    @unittest.skipIf(_HINT_NOT_PROVOCABLE, _SKIP_REASON)
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

    def test_route_py_reads_a_dash_net_as_a_value(self):
        """Through a SHIPPED CLI the hint is inert BY DESIGN (#718 item 2).

        This arm used to assert route.py exits 2 with `unrecognized arguments`
        on `--nets +12V -12V`. #597 (55632d91) made that impossible: route.py
        pins the dash-digit matcher before parse_args, so the token never
        reaches argparse's error path. The fix and this test were developed in
        parallel -- `git merge-base --is-ancestor 55632d91 4b4098a0` is false --
        and each passed alone, so the merge left a test pinning the behaviour
        the fix had removed.

        The skip guard could not save it: `_argparse_takes_dash_values` probes
        a BARE parser, which route.py no longer builds, so it returned False on
        <=3.13 and the arm ran anyway. Re-pinned on the shipped behaviour, which
        holds on every interpreter and therefore needs no guard at all.

        Asserted on the MESSAGE, not the exit code: this fixture board has no
        `-12V`, so route.py rightly refuses with its own exit 2 ("none of the
        requested nets exist"). That refusal is proof the value got through
        argparse. Checking the code alone would confuse two different exit 2s.

        The old arm also passed `ROOT/_no_such_out.kicad_pcb` on the assumption
        it would die in argparse first. Once it really parsed, it really routed
        -- leaving ~350 KB of untracked `_no_such_out.kicad_pcb` plus its
        `.kicad_pro` in the repo root after every suite run. Hence the tempdir.
        """
        with tempfile.TemporaryDirectory() as td:
            r = self._run([BOARD, os.path.join(td, 'out.kicad_pcb'),
                           '--nets', '+12V', '-12V'])
        both = r.stdout + r.stderr
        self.assertNotIn('unrecognized arguments', both,
                         'route.py pins the matcher (#597): a dash-digit net '
                         'name must parse as a VALUE: ' + both[-300:])
        self.assertIn('-12V', both,
                      'the net name must reach the tool as a VALUE')

    def test_route_pys_parser_is_actually_pinned(self):
        """The mechanism, directly -- so the arm above cannot go vacuous.

        `test_route_py_reads_a_dash_net_as_a_value` asserts an ABSENCE, and on
        3.14+ it would pass even if the pin were deleted, because bare argparse
        already accepts dash values there. This pins the pin: route.py must
        route its parser through `cli_nets.pin_dash_digit_values`, and that
        call must change a bare parser's behaviour on THIS interpreter or be a
        no-op only because argparse already agrees.
        """
        import cli_nets
        with open(os.path.join(ROOT, 'py_router', 'route.py'),
                  encoding='utf-8') as fh:
            src = fh.read()
        # assertTrue, not assertIn: assertIn dumps the whole CONTAINER on
        # failure, i.e. 350 KB of route.py into the test log.
        self.assertTrue('pin_dash_digit_values' in src,
                        'route.py must pin the matcher before parse_args '
                        '(#597) -- otherwise a net named -12V is read as an '
                        'unknown option and the run dies in argparse')
        p = argparse.ArgumentParser(prog='pinned', add_help=False)
        p.add_argument('--nets', nargs='+')
        cli_nets.pin_dash_digit_values(p)
        self.assertEqual(p.parse_args(['--nets', '+12V', '-12V']).nets,
                         ['+12V', '-12V'],
                         'the pinned matcher must read -12V as a value')

    def test_the_recommended_form_parses(self):
        """The hint must recommend something that actually works.

        Asserted on the MESSAGE, not the exit code: this fixture board has no
        `-12V`, so route.py rightly refuses with its own exit 2 ("none of the
        requested nets exist"). That refusal is proof the value got through
        argparse -- which is the whole claim. Checking the code alone would
        confuse two different exit 2s.
        """
        with tempfile.TemporaryDirectory() as td:
            r = self._run([BOARD, os.path.join(td, 'out.kicad_pcb'),
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
