#!/usr/bin/env python3
"""#562 GUI parity: the plane finalize's ORACLE leg must run IN-RUN under
return_results, not be deferred to the applier.

WHY THIS IS A GATE. kicad-cli computes the exact fill, so the leg needs a
real file. The GUI has none mid-run, and the first implementation deferred:
it posted results_data['plane_finalize_oracle'] and swig_gui ran the oracle
AFTER applying the copper. That is not just a byte difference -- post-apply
is past the final reconciliation, so the oracle's unroutable links can no
longer feed custody and those nets never get the reconcile retry the CLI
gives them. batch_route(stage_board_fn=...) closes it: the caller hands back
a save of its LIVE board, the engine writes this run's pending copper onto
it (== the CLI's output_file at that point) and runs the oracle there.

Needs neither wx nor pcbnew: stage_board_fn only has to return a path to a
board file standing in for the live board, so a file copy is a faithful
stand-in for pcbnew.SaveBoard.

    python3 tests/test_gui_finalize_oracle_inrun.py
"""
import os
import shutil
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_router'))  # #522
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_tools'))  # #522

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# Needs real zones AND at least one INCOMPLETE plane net: the finalize is a
# plane pass, and a board whose pours are already whole exits before it ever
# reaches the oracle leg (which is how two earlier fixture picks here made
# the gate vacuously "pass").
#
# It must ALSO be a board a fresh clone actually has. The previous pick,
# kit-out-plane.kicad_pcb, is an OUTPUT of tests/test_kit_route.py and was
# untracked + gitignored by 25de9022 (#426) on 2026-07-18 -- 17 days before
# this gate was written -- so it has never existed on any clone, and this
# file has only ever run where a stale artifact happened to be left behind.
# /kicad_files/kit-out*.kicad_pcb is still in .gitignore; do not point back
# at it. Regenerating it is not an option either: it is a full route of
# kit-dev-coldfire (minutes), which is exactly what tests/fixture_boards.py
# declines to cover.
#
# lvds_converter_dualclk_gnd.kicad_pcb is TRACKED, carries a real /GND pour
# that reads unconnected (the run's improvement gate reports the pour
# gaining /GND, 42 -> 41 disconnected pads), and the whole gate runs in
# under a second on it.
BOARD = os.path.join(REPO, 'kicad_files', 'lvds_converter_dualclk_gnd.kicad_pcb')

from kicad_parser import parse_kicad_pcb            # noqa: E402
from route import batch_route                       # noqa: E402


def _run(with_callback):
    work = tempfile.mkdtemp(prefix='finalize_oracle_')
    board = os.path.join(work, 'in.kicad_pcb')
    shutil.copy(BOARD, board)
    for ext in ('.kicad_pro', '.kicad_dru'):
        src = BOARD.replace('.kicad_pcb', ext)
        if os.path.exists(src):
            shutil.copy(src, board.replace('.kicad_pcb', ext))

    if not os.path.exists(BOARD):
        raise SystemExit(
            f"FAIL: fixture board is missing: {BOARD}\n"
            "  This gate needs a TRACKED board. A generated/gitignored one "
            "makes it unrunnable on every fresh clone (see the note above).")

    pcb = parse_kicad_pcb(board)
    zone_nets = sorted({pcb.nets[z.net_id].name for z in (pcb.zones or [])
                        if z.net_id in pcb.nets})
    if not zone_nets:
        # Deliberately NOT a skip. A fixture with no zone nets never reaches
        # the plane finalize, so the gate would report success while testing
        # nothing -- the silent non-coverage this file's own note warns about.
        raise SystemExit(
            "FAIL: fixture board has no zone nets, so the plane finalize -- "
            "and the oracle leg this gate exists to check -- never runs. "
            "Pick a fixture with a real, incomplete pour.")

    staged = []

    def stage_board_fn():
        fd, p = tempfile.mkstemp(suffix='.kicad_pcb', dir=work)
        os.close(fd)
        shutil.copy(board, p)
        staged.append(p)
        return p

    out = batch_route(
        input_file=board, output_file="", net_names=list(zone_nets),
        layers=['F.Cu', 'B.Cu'], track_width=0.2, clearance=0.2,
        via_size=0.6, via_drill=0.3, grid_step=0.2,
        return_results=True, pcb_data=pcb,
        stage_board_fn=(stage_board_fn if with_callback else None),
    )
    return {'staged': len(staged), 'results_data': out[3], 'work': work}


def main():
    failures = []

    # 1. WITH a staging callback: the leg runs in-run, and the deferred
    #    hand-off to the applier must NOT be posted (both would double-run).
    r = _run(True)
    if r['staged'] < 1:
        failures.append("stage_board_fn was never called -- the finalize "
                        "did not try to stage a board for the oracle")
    if 'plane_finalize_oracle' in r['results_data']:
        failures.append("deferred spec posted even though staging worked -- "
                        "the oracle would run twice, and the in-run verdict "
                        "is the one that can feed custody")

    # 2. WITHOUT one (headless callers, older GUI): the fallback must stay,
    #    or those fronts silently lose the leg altogether.
    r2 = _run(False)
    if r2['staged'] != 0:
        failures.append("staged a board with no callback supplied")
    if 'plane_finalize_oracle' not in r2['results_data']:
        failures.append("no staging callback AND no deferred spec -- the "
                        "oracle leg was dropped entirely on this front")

    for w in (r['work'], r2['work']):
        shutil.rmtree(w, ignore_errors=True)

    if failures:
        print("\nFAIL:")
        for f in failures:
            print(f"  - {f}")
        return 1
    print("\nGUI finalize oracle: runs in-run when a board can be staged, "
          "falls back to the post-apply hand-off when it cannot. OK")
    return 0


if __name__ == '__main__':
    sys.exit(main())
