#!/usr/bin/env python3
"""#654: check_connected's KiCad-refill cross-check must ALWAYS say what it did.

Before this, the block printed only when it CHANGED something. "KiCad ran and
agreed", "kicad-cli was missing", "the knob disabled it" and "the board has no
zones" were one indistinguishable silence -- and all four still ended in the
same `ALL NETS FULLY CONNECTED!` / exit 0. That is the ambiguity the
cross-check exists to remove: it matters most when the copper-grading model is
under suspicion of fill-model artifacts.

Pins the wording per state (pure function, no board needed) and the invariant
that a run emits EXACTLY ONE status line.
"""
import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for p in (ROOT, os.path.join(ROOT, 'py_router'), os.path.join(ROOT, 'py_tools')):
    if p not in sys.path:
        sys.path.insert(0, p)

import fixture_boards  # noqa: E402
from check_connected import reconcile_status_line as line  # noqa: E402

# A board WITH ZONES on purpose: the whole ambiguity is zone-backed nets, and
# a zone-less board only ever exercises the "not applicable" branch -- which
# would let the two states this issue is about regress unnoticed.
#
# #716: this pointed at `kit-out-plane-connected.kicad_pcb`, which is UNTRACKED
# and has no fixture_boards recipe, so on any clone it was simply absent. The
# three board-backed checks below then degraded to an empty result rather than
# skipping -- `0 found: []` -- and the vacuity guard ("the fixture really has
# zones") was the only thing saying the other two tested nothing. Repointed at a
# TRACKED zoned board so the checks actually run. Verified: check_connected
# reaches "ran, N KiCad link(s), agrees with copper grading" on it in ~2 s, i.e.
# the real cross-check branch, not "not applicable". Do not repoint this at a
# zone-less board (esp_prog.kicad_pcb, for one, reports "no zones") -- that
# turns the file green while testing nothing, which is the state #716 found.
BOARD = fixture_boards.ensure('lvds_converter_dualclk_gnd.kicad_pcb')
passed = failed = 0


def check(name, ok, detail=""):
    global passed, failed
    passed += bool(ok); failed += not ok
    print(f"  {'OK  ' if ok else 'FAIL'} {name}{(' -- ' + detail) if detail else ''}")


def test_every_state_is_distinguishable():
    """The four silent states must produce four DIFFERENT lines -- that is the
    whole bug. A test that only checked 'something was printed' would pass on
    a single generic string."""
    states = {
        'ran_agrees':   {'state': 'ran', 'links': 0},
        'ran_reclass':  {'state': 'ran', 'links': 7, 'reclass': 2},
        'ran_flagged':  {'state': 'ran', 'links': 3, 'kicad_only': 1},
        'did_not_run':  {'state': 'did_not_run', 'detail': 'kicad-cli not found'},
        'disabled':     {'state': 'disabled', 'detail': 'KICAD_NO_GRADE_RECONCILE is set'},
        'error':        {'state': 'error', 'detail': 'boom'},
        'not_applic':   {'state': 'not_applicable', 'detail': 'board has no zones'},
    }
    out = {k: line(v) for k, v in states.items()}
    check("all states produce distinct lines",
          len(set(out.values())) == len(out), str(out))
    # The two that used to read as success must NOT read as agreement.
    for k in ('did_not_run', 'disabled', 'error'):
        check(f"{k} does not claim agreement",
              'agrees' not in out[k].lower(), out[k])
    check("ran+agrees says so", 'agrees with copper grading' in out['ran_agrees'])
    check("reclassification is reported", 'reclassified 2' in out['ran_reclass'])
    check("KiCad-only flags are reported", 'flagged 1' in out['ran_flagged'])
    # The did-not-run line must warn about what the verdict then rests on.
    check("did_not_run warns the grade is copper-only",
          'ALONE' in out['did_not_run'], out['did_not_run'])


def test_exactly_one_line_per_run():
    r = subprocess.run([sys.executable, '-X', 'utf8',
                        os.path.join(ROOT, 'py_router', 'check_connected.py'),
                        BOARD], capture_output=True, text=True, timeout=900)
    hits = re.findall(r'KiCad refill cross-check:.*', r.stdout)
    check("a run prints exactly one status line", len(hits) == 1,
          f"{len(hits)} found: {hits[:3]}")
    check("the fixture really has zones (else this tests nothing)",
          bool(hits) and 'not applicable' not in hits[0], str(hits))


def test_the_knob_is_visible_in_the_output():
    env = dict(os.environ, KICAD_NO_GRADE_RECONCILE='1')
    r = subprocess.run([sys.executable, '-X', 'utf8',
                        os.path.join(ROOT, 'py_router', 'check_connected.py'),
                        BOARD], capture_output=True, text=True, timeout=900, env=env)
    hits = re.findall(r'KiCad refill cross-check:.*', r.stdout)
    check("KICAD_NO_GRADE_RECONCILE is disclosed, not silent",
          len(hits) == 1 and 'DISABLED' in hits[0], str(hits))


if __name__ == '__main__':
    test_every_state_is_distinguishable()
    test_exactly_one_line_per_run()
    test_the_knob_is_visible_in_the_output()
    print(f"\n{passed}/{passed + failed} checks passed")
    sys.exit(1 if failed else 0)
