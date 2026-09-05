"""`blocking` counts ERROR-severity floorplan violations, not every violation.

Found while implementing #793, which proposed a new floorplan finding at WARN
"so it is loud without being fatal". It would not have been: `board_score`
summed `len(violations)` into `blocking`, severity-blind, and `converge` reports
DONE only when `blocking` is 0. So a warn was a permanent blocker -- and one
nothing could clear, because the intent schema has no `off` severity
(`floorplan.py` refuses any value but 'error'/'warn') and `--exit-zero` disables
every rule at once.

The sharp form of the defect: the two instruments DISAGREED about the same JSON.
Measured on splitflap_driver with one warn-demoted `block_unresolved`:

    check_floorplan --intent   ->  errors 0, warnings 1, pass true   EXIT 0
    board_score     --intent   ->  blocking_by.floorplan 1           EXIT 4

`check_floorplan` returns its exit code from `GradeResult.passed`, which reads
`errors` alone -- so the grader already treats warn as non-fatal, and only the
scorer disagreed.

Both arms below run the REAL `score_floorplan` against the REAL
`check_floorplan.py`, because the bug was in how the scorer reads the grader's
output: a unit test over a hand-built dict would have re-implemented the
partition it is supposed to check, and passed either way.
"""
import json
import os
import sys
import tempfile

RUN_ALL_FAST_OK = True

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(
    ROOT, '.claude', 'skills', 'plan-pcb-placement-and-routing', 'scripts'))
import board_score  # noqa: E402

SIZE = (24.0, 20.0)

passed = 0
failed = 0


def check(name, ok, detail=""):
    """Detail must read as a MEASUREMENT: it prints on OK and on FAIL alike."""
    global passed, failed
    if ok:
        passed += 1
        print(f"  OK   {name}" + (f" -- {detail}" if detail else ""))
    else:
        failed += 1
        print(f"  FAIL {name}" + (f" -- {detail}" if detail else ""))


def _board(path):
    """One SMD part on a rectangular outline. Small on purpose: this test is
    about severity bookkeeping, and a corpus board would spend minutes in the
    grader to measure the same two integers."""
    body = (
        '(kicad_pcb\n\t(version 20241229)\n\t(net 0 "")\n\t(net 1 "N1")\n'
        f'\t(gr_rect\n\t\t(start 0 0)\n\t\t(end {SIZE[0]} {SIZE[1]})\n'
        '\t\t(layer "Edge.Cuts")\n\t\t(uuid "e1")\n\t)\n'
        '\t(footprint "test:P1"\n\t\t(layer "F.Cu")\n\t\t(uuid "fp-U1")\n'
        '\t\t(at 12 10)\n\t\t(property "Reference" "U1"\n\t\t\t(at 0 0)\n\t\t)\n'
        '\t\t(fp_rect\n\t\t\t(start -1 -1)\n\t\t\t(end 1 1)\n'
        '\t\t\t(layer "F.CrtYd")\n\t\t\t(uuid "cy-U1")\n\t\t)\n'
        '\t\t(pad "1" smd rect\n\t\t\t(at 0 0)\n\t\t\t(size 0.3 0.3)\n'
        '\t\t\t(layers "F.Cu")\n\t\t\t(net 1 "N1")\n\t\t\t(uuid "p0")\n\t\t)\n'
        '\t)\n)\n')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(body)
    return path


def _intent(path, severity):
    """A block whose `refs` match nothing -> exactly one `block_unresolved`.

    Chosen because it is the one finding that needs no geometry to fire, so the
    two arms differ ONLY in the severity map -- which is the variable under
    test. A geometry-driven rule would have let a fixture difference explain a
    count difference.
    """
    doc = {"schema": 1, "kind": "floorplan-intent", "units": "mm",
           "envelope": {"rect": [0.0, 0.0, SIZE[0], SIZE[1]],
                        "tolerance_mm": 0.5},
           "blocks": [{"name": "typo", "refs": ["ZZZ999*"]}]}
    if severity:
        doc["severity"] = severity
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(doc, f)
    return path


def main():
    with tempfile.TemporaryDirectory() as wd:
        board = _board(os.path.join(wd, 'b.kicad_pcb'))
        warn = _intent(os.path.join(wd, 'warn.json'),
                       {"block_unresolved": "warn"})
        err = _intent(os.path.join(wd, 'err.json'), None)

        w = board_score.score_floorplan(ROOT, board, warn, wd)
        e = board_score.score_floorplan(ROOT, board, err, wd)

        # Neither arm may have quietly failed to run: a component that did not
        # run reports count None, and `blocking` would be `unknown` rather than
        # a number -- which is a different bug wearing this one's clothes.
        check("both arms actually graded",
              w.get('ran') is True and e.get('ran') is True,
              f"warn ran={w.get('ran')} ({w.get('reason', '-')}), "
              f"error ran={e.get('ran')} ({e.get('reason', '-')})")

        # ANTI-VACUITY: the error arm must FIRE. An arm that finds nothing makes
        # the warn arm's 0 meaningless -- both would be 0 for the same reason.
        check("CONTROL: the same finding at ERROR still counts as blocking",
              e.get('count') == 1 and e.get('violations') == ['block_unresolved'],
              f"count={e.get('count')} violations={e.get('violations')}")

        check("a WARN violation does not count toward blocking",
              w.get('count') == 0,
              f"count={w.get('count')} (was len(violations)=1 before the fix)")

        # Not counted is not the same as not reported. The whole point of warn
        # is that the author still sees it.
        check("a WARN violation is still REPORTED",
              w.get('warnings') == 1
              and w.get('warning_rules') == ['block_unresolved'],
              f"warnings={w.get('warnings')} rules={w.get('warning_rules')}")

        check("the error arm reports no warnings",
              e.get('warnings') == 0 and e.get('warning_rules') == [],
              f"warnings={e.get('warnings')} rules={e.get('warning_rules')}")

        # The keys are present on BOTH arms, so a reader cannot confuse "no
        # warnings" with "warnings were not looked at" -- the same argument
        # rules_run/rules_skipped makes one level up.
        check("both arms carry the warning keys, fired or not",
              all(k in w and k in e for k in ('warnings', 'warning_rules')),
              f"warn keys={sorted(set(w) & {'warnings', 'warning_rules'})}, "
              f"error keys={sorted(set(e) & {'warnings', 'warning_rules'})}")

        # The defect was a DISAGREEMENT with check_floorplan, so pin the
        # agreement rather than only the new number: the scorer's blocking
        # count must be zero exactly when the grader's exit code is zero.
        rc_w, _ = board_score.run_tool(ROOT, 'check_floorplan.py', board,
                                       '--intent', warn, '-q')
        rc_e, _ = board_score.run_tool(ROOT, 'check_floorplan.py', board,
                                       '--intent', err, '-q')
        check("the scorer now agrees with the grader it reads",
              (rc_w == 0) == (w.get('count') == 0)
              and (rc_e != 0) == (e.get('count') > 0),
              f"check_floorplan exit warn={rc_w} error={rc_e}; "
              f"score count warn={w.get('count')} error={e.get('count')}")

    print(f"\n{passed} passed, {failed} failed")
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
