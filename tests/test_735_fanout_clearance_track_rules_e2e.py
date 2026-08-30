#!/usr/bin/env python3
"""#735 end-to-end: the copper the cap-repair pass draws is graded by the real
checker, at the board's own TRACK-scoped `.kicad_dru` rule.

The unit half (`test_735_fanout_clearance_track_rules.py`) proves the gate now
charges the rule. This half answers the question that actually matters and that
no in-process arm can: does `check_drc` still flag the copper this pass emits?

THE MEASUREMENT is a paired delta, never an absolute. The rig board carries its
own copper, some of which the CRIT class puts in violation of the rule all by
itself, so the interesting number is what the CONNECTOR adds:

    board + stub + foreign track                  ->  9 rule pairs
    board + stub + foreign track + the connector  -> 10 rule pairs

Measured on this tree. The delta of exactly 1 is the defect in the checker's
own currency: the pass drew one piece of copper that its own checker rejects.

WHAT THE FIX DOES ABOUT IT, in two arms, because the answer has two halves.

At this geometry NO landing in the whole sweep satisfies the rule, so the pass
does NOT abandon the via -- that would leave the pad-via graze it exists to
remove sitting on the board, which check_drc also counts. It keeps the repair
and DISCLOSES that the copper does not meet the rule. Same copper, same grade;
what changed is that the run says so.

One rung out (`D_RELOCATE`), a rule-satisfying landing does exist, and the
ruled arm takes a different one from the unruled arm with no fallback line.
That is the behaviour change; the arm above is the guarantee that it never
costs a repair.

Absolutes are deliberately not asserted -- 9 is a property of the rig board's
existing copper, and pinning it would make an unrelated board edit fail here
with a message about track rules. The steering arm is asserted on the LANDINGS
rather than on a second grade: at D_RELOCATE the unruled connector's overlap
(0.04 mm) sits close enough to check_drc's 5 % tolerance that the graded delta
is 0, so a grade there would measure the tolerance, not the fix.

This file shells out and is therefore in the integration bucket by design; it
declares no fast-path override. Its cheap half lives in the sibling.
"""
from __future__ import annotations

RUN_ALL_TIMEOUT = 2400

import os
import re
import shutil
import subprocess
import sys
import tempfile

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)
for _p in ('', 'py_router', 'py_placer', 'py_tools'):
    _d = os.path.join(_ROOT, _p)
    if _d not in sys.path:
        sys.path.insert(0, _d)
if _TESTS not in sys.path:
    sys.path.insert(0, _TESTS)

import run_utils
from kicad_writer import add_tracks_to_pcb
from test_735_fanout_clearance_track_rules import (  # noqa: E402
    CLEAR, DRU_BOTH, D_REFUSE, D_RELOCATE, RIG_LANDING, _nudge, _rig,
    _stage, _landing)

# `[^\n]*` before the colon is load-bearing: check_drc appends a contact count
# to the header. Same expression tests/test_735_dru_rule_pair_classification.py
# uses, for the same reason.
RULE_PAIRS = re.compile(
    r'^SEGMENT-SEGMENT-TRACK-RULE violations \((\d+)\)[^\n]*:', re.M)

passed = failed = 0


def check(name, ok, detail=''):
    global passed, failed
    passed += bool(ok)
    failed += not ok
    print('  %s %s%s' % ('OK  ' if ok else 'FAIL', name,
                         (' -- ' + detail) if detail else ''))


def _tracks(pcb):
    return [{'start': (s.start_x, s.start_y), 'end': (s.end_x, s.end_y),
             'width': s.width, 'layer': s.layer, 'net_id': s.net_id}
            for s in pcb.segments]


def _grade(td, src, tracks, name):
    """Write `tracks` onto a copy of `src`, attach the project AND the rules
    file, and grade with the real checker. Returns the rule-pair count."""
    out = os.path.join(td, name + '.kicad_pcb')
    if not add_tracks_to_pcb(src, out, tracks):
        raise AssertionError('the writer refused to emit %s' % name)
    stem = os.path.splitext(out)[0]
    # copyfile, not copy: the writer may already have laid down a project
    # beside the output, and copy() also copies the MODE, which on Windows
    # raises WinError 87 over an existing one. The bytes are what matter.
    shutil.copyfile(os.path.splitext(src)[0] + '.kicad_pro',
                    stem + '.kicad_pro')
    with open(stem + '.kicad_dru', 'w', encoding='utf-8') as f:
        f.write(DRU_BOTH)
    run_utils.evidence(out, 'the board to grade')
    r = subprocess.run(
        [sys.executable, '-X', 'utf8',
         os.path.join(_ROOT, 'py_router', 'check_drc.py'), out,
         '--clearance', str(CLEAR), '--max-print', '0'],
        capture_output=True, text=True, timeout=1800)
    if 'Traceback' in r.stderr:
        raise AssertionError('check_drc died on %s:\n%s' % (name, r.stderr))
    if 'Track-to-track clearance rules' not in r.stdout:
        raise AssertionError('the checker never read the rules file for %s; '
                             'the grade below would be meaningless' % name)
    m = RULE_PAIRS.search(r.stdout)
    return int(m.group(1)) if m else 0


def main():
    with tempfile.TemporaryDirectory() as td:
        # --- the UNRULED arm: it draws a connector, and the checker rejects it
        src = _stage(td, 'off')
        pcb, st, _v0 = _rig(src, dperp=D_REFUSE)
        before = _tracks(pcb)
        moves, segs, text = _nudge(st, pcb, max_shift=4.0)
        # ON THE BRANCH: without a connector there is nothing to grade, and
        # "no new violation" is what a pass that did nothing looks like too.
        check('the unruled arm relocates the via and draws a connector',
              _landing(moves) == RIG_LANDING and len(segs) == 1,
              'landing=%r segs=%d out=%r' % (_landing(moves), len(segs),
                                             text.strip()[:120]))
        after = _tracks(pcb)
        check('the connector is the only copper added',
              len(after) == len(before) + 1,
              '%d -> %d' % (len(before), len(after)))

        # NOT 'con': `CON` is a reserved DEVICE name on Windows, so
        # `con.kicad_pcb` is the console. The writer reports success, the
        # bytes go to the terminal, and `os.path.isfile` says no -- which is
        # exactly what this file's first run did. `run_utils.evidence` is what
        # turned that into a named failure instead of a grade of zero.
        n_without = _grade(td, src, before, 'without_connector')
        n_with = _grade(td, src, after, 'with_connector')
        check('the connector this pass drew is a rule violation the checker '
              'flags', n_with - n_without == 1,
              'rule pairs %d -> %d' % (n_without, n_with))
        # NEGATIVE CONTROL: the baseline must be non-zero, or the delta above
        # is being read off a checker that found nothing either way and the
        # arm would pass with the rules file unread.
        check('the rules file really is binding on this board', n_without > 0,
              'baseline rule pairs = %d' % n_without)

        # --- the RULED arm at the SAME geometry. No landing in the sweep
        # satisfies the rule here, so the pass keeps the repair rather than
        # abandoning the via -- and discloses that the copper does not meet
        # the rule. It is the SAME copper as the unruled arm, so the grade is
        # the same: what changed is that the run now says so.
        ruled = _stage(td, 'on', dru=DRU_BOTH)
        pcb2, st2, _v1 = _rig(ruled, dperp=D_REFUSE)
        moves2, segs2, text2 = _nudge(st2, pcb2, max_shift=4.0)
        check('the ruled arm KEEPS the repair rather than abandoning the via',
              _landing(moves2) == RIG_LANDING and len(segs2) == 1,
              'landing=%r segs=%d out=%r' % (_landing(moves2), len(segs2),
                                             text2.strip()[:120]))
        check('and discloses that the copper does not meet the board rule',
              'took a landing at the base clearance' in text2,
              text2.strip()[:200])

        # --- and where a rule-satisfying landing DOES exist, it takes it.
        # Same fixture, one rung out. This is the behaviour change; the arm
        # above is the guarantee that it never costs a repair.
        steer_off = _stage(td, 'steer_off')
        pcb3, st3, _v2 = _rig(steer_off, dperp=D_RELOCATE)
        moves3, _s3, text3 = _nudge(st3, pcb3, max_shift=4.0)
        steer_on = _stage(td, 'steer_on', dru=DRU_BOTH)
        pcb4, st4, _v3 = _rig(steer_on, dperp=D_RELOCATE)
        moves4, segs4, text4 = _nudge(st4, pcb4, max_shift=4.0)
        check('the rule steers the landing where one satisfies it',
              _landing(moves3) == RIG_LANDING
              and _landing(moves4) not in (None, RIG_LANDING)
              and len(segs4) == 1,
              'unruled=%r ruled=%r out=%r' % (_landing(moves3),
                                              _landing(moves4),
                                              text4.strip()[:120]))
        check('and does NOT disclose a fallback, because none was needed',
              'took a landing at the base clearance' not in text4,
              text4.strip()[:200])

    print('\n%d/%d checks passed' % (passed, passed + failed))
    return 0 if failed == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
