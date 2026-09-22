#!/usr/bin/env python3
"""#857: a fab-tier escalation must say WHERE it landed, not always "advanced".

Under ``--escalation board`` every rung is raised to the board's own declared
minimums (`fab_tiers._apply_board_floors`), so a descent off rung 0 stops at
what the BOARD declared and never reaches the advanced floor for any key the
board declares. The warning and the end-of-run line claimed "advanced
(0.25/0.15 via)" regardless of policy.

Measured on eurorack_pmod (6-layer, declares min_via_diameter 0.4): a
`--escalation board` run reported "645 fab-tier escalation(s) to advanced"
while the smallest via it delivered was 0.4 -- the board's own number. The
descent is real (0.4 is below the standard tier's 0.45) and is still counted;
what was wrong is the claim about where it ended up, on the one policy whose
entire purpose is that it does NOT leave the board's declared envelope.

The `fab` arm's wording is pinned byte-for-byte: `tests/test_620_*` asserts
the 'escalated standard->advanced fab floor' string, and that arm is unchanged.
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

import fab_tiers as ft  # noqa: E402

# eurorack_pmod's declared floors -- the measured case above. via_diameter 0.4
# sits BELOW the standard tier's 0.45, which is what makes rung 1 a real
# descent that `board` nonetheless bounds at the declaration.
BOARD_RULES = {'min_clearance': 0.127, 'min_via_diameter': 0.4,
               'min_through_hole_diameter': 0.3, 'min_track_width': 0.127,
               'min_hole_to_hole': 0.25}


def _arm(policy):
    """Fresh ledger, one escalation recorded under `policy`. Returns
    (printed warning, summary, end-of-run line)."""
    import io
    import contextlib
    ft._LEDGER['fab_tier'].clear()
    ft._LEDGER['narrowed'].clear()
    ft._escalation_warned.clear()
    ft._escalation_lever_said.clear()
    ft.set_default_fab_tier('auto')
    ft.set_escalation_policy(policy,
                             board_floors=ft.board_floors_from_rules(BOARD_RULES))
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        ft.warn_fab_escalation('net rescue net_42')
    return buf.getvalue(), ft.escalation_summary(), ft.escalation_report_line()


def main():
    fails = []

    # --- the ladder really does bound the descent at the declaration, which is
    # the fact the wording has to match. Without this the test could pass on a
    # board where `board` and `fab` happen to agree.
    ft.set_default_fab_tier('auto')
    ft.set_escalation_policy('board',
                             board_floors=ft.board_floors_from_rules(BOARD_RULES))
    rungs = ft.escalation_rungs(6)
    deepest = min(r['via_diameter'] for r in rungs)
    if deepest != 0.4:
        fails.append(f"board policy's deepest via rung is {deepest}, expected the "
                     f"board's declared 0.4 -- fixture no longer exercises the case")
    if deepest >= 0.45:
        fails.append("the board rung is not below the standard 0.45, so nothing "
                     "would call warn_fab_escalation at all")

    # --- board arm: counted, but NOT described as reaching advanced
    warn, summary, line = _arm('board')
    if summary['fab_tier_escalations'] != 1:
        fails.append(f"board arm counted {summary['fab_tier_escalations']} "
                     f"escalations, expected 1 (the descent is still real)")
    if summary.get('fab_tier_target') != 'board_floors':
        fails.append(f"board arm's fab_tier_target is "
                     f"{summary.get('fab_tier_target')!r}, expected 'board_floors' "
                     f"-- a consumer cannot tell the envelope was respected")
    if '0.25/0.15 via etc' in warn:
        fails.append("board arm still claims the advanced floor's 0.25/0.15 via, "
                     "which it never reached")
    if "BOARD's own declared minimums" not in warn:
        fails.append("board arm does not say the descent stopped at the board's "
                     "own declaration")
    if '--escalation off' not in warn:
        fails.append("board arm offers no lever that actually forbids the descent")
    if 'escalation(s) to advanced' in line:
        fails.append(f"end-of-run line still says 'to advanced' under board: {line}")
    if "to the board's own declared floors" not in line:
        fails.append(f"end-of-run line does not name the real target: {line}")

    # --- fab arm: unchanged, byte-for-byte on the pinned string
    warn, summary, line = _arm('fab')
    if summary.get('fab_tier_target') != 'advanced':
        fails.append(f"fab arm's fab_tier_target is "
                     f"{summary.get('fab_tier_target')!r}, expected 'advanced'")
    if 'escalated standard->advanced fab floor' not in warn:
        fails.append("fab arm's warning changed -- test_620 pins this string")
    if '0.25/0.15 via etc' not in warn:
        fails.append("fab arm no longer names the advanced floor it does reach")
    if 'escalation(s) to advanced' not in line:
        fails.append(f"fab arm's end-of-run line changed: {line}")

    # --- off arm: no rungs at all, so nothing to mislabel
    ft.set_escalation_policy('off',
                             board_floors=ft.board_floors_from_rules(BOARD_RULES))
    if ft.escalation_rungs(6):
        fails.append("--escalation off handed out descent rungs")

    if fails:
        print("FAIL:\n  " + "\n  ".join(fails))
        return 1
    print("PASS: a board-bounded escalation is counted but reported against the "
          "board's own declared floors (warning, end-of-run line and "
          "fab_tier_target); the fab arm's wording is unchanged")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
