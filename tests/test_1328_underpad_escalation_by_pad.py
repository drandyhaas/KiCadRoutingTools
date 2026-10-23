#!/usr/bin/env python3
"""An under-pad escape that escalates discloses every pad it rescues (#1328).

When a part's under-pad escape drops pads, qfn_fanout re-runs the whole part
down qfn_clearance_ladder and keeps the run that drops fewest. The pads that
run rescued were found by NET NAME (`n not in best[2]`), so a net with two
pads that escapes one of them at the rung was still in the dropped names: the
pad went unrecorded in LAST_ESCALATED and the narrowing ledger, and the run
said "escapes 0 of them" while shipping one more escape below the requested
clearance. rp2350_fpga_eensy_prePlane U6 is that case: /RP2354A/1V1 has four
dropped pads at 0.1 mm, and pad 50 escapes at 0.09.

Checks:
  1. precondition: with escalation off, U6 drops 41 pads and /RP2354A/1V1 is
     named four times, so a name-based difference cannot see one of them
     leave;
  2. at the default policy U6 drops 40, LAST_ESCALATED names U6.50 at 0.09,
     the ledger holds one clearance row for /RP2354A/1V1 at the underpad
     site, and the console says it escapes 1;
  3. the single-pad case is unchanged: routed_output U2's Net-(U2A-DATA_6)
     (pad 46) is named at 0.095.

    python3 tests/test_1328_underpad_escalation_by_pad.py
"""
import contextlib
import io
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, HERE)

import fab_tiers                                        # noqa: E402
import qfn_fanout                                       # noqa: E402
from kicad_parser import parse_kicad_pcb                # noqa: E402
from run_utils import evidence                          # noqa: E402

RP2350 = os.path.join(ROOT, 'kicad_files', 'rp2350_fpga_eensy_prePlane.kicad_pcb')
U2BOARD = os.path.join(ROOT, 'kicad_files', 'routed_output.kicad_pcb')
RAIL = '/RP2354A/1V1'


def _run(board, ref, policy):
    prev = fab_tiers.get_escalation_policy()
    fab_tiers.set_escalation_policy(policy)
    fab_tiers.reset_ledger()
    qfn_fanout.LAST_ESCALATED.clear()
    try:
        pcb = parse_kicad_pcb(evidence(board, os.path.basename(board)))
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            _, _, dropped = qfn_fanout.generate_qfn_fanout(
                pcb.footprints[ref], pcb, layer=pcb.footprints[ref].layer,
                track_width=0.1, clearance=0.1, grid_step=0.05,
                escape_method='underpad', via_size=0.45, via_drill=0.25,
                allow_via_in_pad=False)
        rows = [r for r in fab_tiers.escalation_summary().get('narrowed', [])
                if r.get('site') == 'qfn underpad escape']
        return dropped, list(qfn_fanout.LAST_ESCALATED), rows, buf.getvalue()
    finally:
        fab_tiers.set_escalation_policy(*prev)


def main():
    dropped_off, esc_off, _, _ = _run(RP2350, 'U6', 'off')
    assert len(dropped_off) == 41 and not esc_off, (len(dropped_off), esc_off)
    assert dropped_off.count(RAIL) == 4, dropped_off.count(RAIL)
    print(f"  PASS: precondition: escalation off drops 41 pads, {RAIL} four times")

    dropped, esc, rows, text = _run(RP2350, 'U6', 'fab')
    assert len(dropped) == 40 and dropped.count(RAIL) == 3, (len(dropped), dropped.count(RAIL))
    assert esc == [{'net': RAIL, 'pad': 'U6.50', 'clearance': 0.09}], esc
    assert [(r['net_name'], r['kind'], r['delivered']) for r in rows] == [(RAIL, 'clearance', 0.09)], rows
    assert 'escapes 1 of them (40 still dropped)' in text, text
    print(f"  PASS: U6.50 ({RAIL}) rescued at 0.09 is named, ledgered and counted")

    _, esc, rows, _ = _run(U2BOARD, 'U2', 'fab')
    assert esc == [{'net': 'Net-(U2A-DATA_6)', 'pad': 'U2.46', 'clearance': 0.095}], esc
    assert len(rows) == 1, rows
    print("  PASS: a single-pad net is named as before (U2.46 at 0.095)")
    print('test_1328_underpad_escalation_by_pad: all passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
