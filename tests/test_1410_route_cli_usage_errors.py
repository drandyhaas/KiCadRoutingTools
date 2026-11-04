#!/usr/bin/env python3
"""#1410: route.py / route_diff.py answer a usage mistake with a one-line
argparse refusal (exit 2) instead of a traceback or a silent exit 0:

  * a missing input board (was: the #441 banner, then FileNotFoundError);
  * --power-nets and --power-nets-widths of different lengths (was: a
    ValueError traceback from batch_route; the engine keeps raising it for the
    GUI);
  * an option and its values fused into ONE word, which --nets swallowed as a
    net name ("--layer-costs 1.0 1.5"): the router warned the net did not
    exist, routed at the default layer costs and exited 0.

Dash-digit net names (#597) must stay legal. Every refusal asserts its REASON
(run_utils.check), so a command that died for another reason is a broken test.

    python3 tests/test_1410_route_cli_usage_errors.py
"""
import argparse
import os
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
from run_utils import check, evidence, tool        # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'cap_chain.kicad_pcb')
PY = [sys.executable, '-X', 'utf8']
#: argparse's own wording is the intended refusal here; the REASON is asserted.
ARGPARSE = ('error: argument',)


def main():
    evidence(BOARD, 'cap_chain board')
    tmp = tempfile.mkdtemp(prefix='t1410_')
    try:
        inb = os.path.join(tmp, 'in.kicad_pcb')
        shutil.copy(BOARD, inb)
        out = os.path.join(tmp, 'o.kicad_pcb')
        missing = os.path.join(tmp, 'missing.kicad_pcb')
        for script in ('route.py', 'route_diff.py'):
            check(PY + [tool(script), missing, out, '--nets', '*'],
                  refuse='does not exist', code=2, allow=ARGPARSE)
            check(PY + [tool(script), inb, out, '--nets', 'DPA_P',
                        '--layer-costs 1.0 1.5'],
                  refuse='joined into one word', code=2, allow=ARGPARSE)
            check(PY + [tool(script), inb, out, '--nets', 'DPA_*',
                        '--layers F.Cu B.Cu'],
                  refuse='joined into one word', code=2, allow=ARGPARSE)
        check(PY + [tool('route.py'), inb, out, '--nets', 'DPA_P',
                    '--power-nets', 'DPA_P', 'DPA_N',
                    '--power-nets-widths', '0.5'],
              refuse='must have the same length', code=2, allow=ARGPARSE)

        # The helper itself: a dash-digit net name and a spaced net name that
        # is not an option stay legal.
        import cli_nets
        p = argparse.ArgumentParser()
        p.add_argument('--nets', nargs='+')
        p.add_argument('--layer-costs', nargs='+', type=float)
        cli_nets.pin_dash_digit_values(p)
        for ok_names in (['-3V3', '-12V'], ['Net-(C1-Pad1)', 'my net']):
            cli_nets.refuse_joined_option_words(
                p, p.parse_args(['--nets'] + ok_names))
        # an abbreviation of a long option is the same mistake
        try:
            cli_nets.refuse_joined_option_words(
                p, argparse.Namespace(nets=['--layer-c 1.0 2.0']))
        except SystemExit as e:
            assert e.code == 2
        else:
            raise AssertionError('abbreviated joined option was accepted')
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('PASS')


if __name__ == '__main__':
    main()
