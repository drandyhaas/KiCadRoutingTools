#!/usr/bin/env python3
"""
Gate: both fanout CLIs accept BOTH --width and --track-width, and each
spelling reaches the SAME GUI control as the other.

The two sibling CLIs used to disagree on the name for one concept --
bga_fanout.py took --track-width, qfn_fanout.py took --width -- and nothing
told you which was which until argparse refused the command. That is not
hypothetical: the recorded stress manifest for openstint (set4) contains
THREE qfn_fanout attempts, and the middle one is `--track-width 0.08` being
rejected, then retried as `--width 0.08`. A later replay of that same
manifest paid the same toll again.

Aliasing is the easy half. The half worth gating is where each spelling
LANDS: qfn_fanout's width has its own GUI control (qfn_track_width, #381 D7),
so if `--track-width` fell through to the GLOBAL FLAG_PARAMS it would set the
Basic-tab track_width instead and a plan-replayed QFN fanout would route at
the wrong width -- silently, and only on the alias spelling.

Run:
    python3 tests/test_fanout_width_aliases.py
"""

import os
import subprocess
import sys

_TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS_DIR)
for _p in (_ROOT, os.path.join(_ROOT, 'py_router'),
           os.path.join(_ROOT, 'tests', 'stress')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

FAILURES = []


def check(cond, label):
    print(f"  {'PASS' if cond else 'FAIL'}  {label}")
    if not cond:
        FAILURES.append(label)


def test_both_clis_accept_both_spellings():
    """argparse must not refuse either name on either tool."""
    for tool in ('qfn_fanout.py', 'bga_fanout.py'):
        for flag in ('--width', '--track-width'):
            r = subprocess.run(
                [sys.executable, os.path.join(_ROOT, 'py_router', tool),
                 '/nonexistent.kicad_pcb', '--component', 'U1', flag, '0.08'],
                capture_output=True, text=True, timeout=120)
            # The board does not exist, so the run fails -- but it must fail
            # on the BOARD, never on the flag. Asserting the REASON, not just
            # a non-zero exit.
            check('unrecognized arguments' not in r.stderr,
                  f"{tool} accepts {flag}")


def test_each_spelling_reaches_the_same_control():
    import manifest_to_plan as m

    def dest(tool, flag):
        f = m.TOOL_FLAG_ALIASES.get(tool, {}).get(flag, flag)
        return (m.TOOL_FLAG_PARAMS.get(tool, {}).get(f)
                or m.FLAG_PARAMS.get(f) or m.LIST_FLAGS.get(f))

    q_w, q_tw = dest('qfn_fanout.py', '--width'), dest('qfn_fanout.py', '--track-width')
    b_w, b_tw = dest('bga_fanout.py', '--width'), dest('bga_fanout.py', '--track-width')
    check(q_w == q_tw == 'qfn_track_width',
          f"qfn: both spellings -> qfn_track_width (got {q_w!r} / {q_tw!r})")
    check(b_w == b_tw == 'track_width',
          f"bga: both spellings -> track_width (got {b_w!r} / {b_tw!r})")
    # The point of the per-tool override: qfn must NOT land on the Basic tab.
    check(q_tw != 'track_width',
          "qfn's --track-width does NOT fall through to the Basic-tab "
          "track_width (#381 D7)")


def main():
    print(__doc__.strip().splitlines()[0])
    for fn in (test_both_clis_accept_both_spellings,
               test_each_spelling_reaches_the_same_control):
        print(f"\n{fn.__name__}:")
        fn()
    print()
    if FAILURES:
        print(f"FAILED ({len(FAILURES)}): " + "; ".join(FAILURES))
        return 1
    print("Fanout width-alias invariants hold.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
