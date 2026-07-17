#!/usr/bin/env python3
"""connection_width grading (#406): the min-copper-web class in the kicad-cli
cross-check harness.

KiCad's connection_width checker is OFF by default (min_connection 0) and
warning-severity, so routed-copper micro-webs were invisible to every
automated consumer -- the corpus gave NO signal on the class. The harness now
stages the constraint (author-set min_connection kept, else the project's
min_track_width) and grades the class SEPARATELY from the copper-clearance
classes, so items never pollute kicad_only (the check_drc-false-negative
alarm channel).

Known positive by construction: a wide-thin-wide copper bridge whose web is
narrower than the staged floor MUST be flagged (exercises the FAIL branch
before any corpus number is trusted). Requires kicad-cli; SKIPs cleanly (exit
0 with a note) when it is not installed.

    python3 tests/test_connection_width_grading.py
"""
import json
import os
import shutil
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                "tests", "stress"))

from kicad_drc_compare import KICAD_CLI, compare_board_data  # noqa: E402

BOARD_TMPL = """(kicad_pcb
	(version 20260206)
	(generator "pcbnew")
	(generator_version "10.0")
	(general
		(thickness 1.6)
		(legacy_teardrops no)
	)
	(paper "A4")
	(layers
		(0 "F.Cu" signal)
		(2 "B.Cu" signal)
		(25 "Edge.Cuts" user)
	)
	(setup
		(pad_to_mask_clearance 0)
	)
	(net 0 "")
	(net 1 "N1")
	(gr_rect
		(start 5 5)
		(end 40 20)
		(stroke
			(width 0.1)
			(type solid)
		)
		(layer "Edge.Cuts")
	)
	(segment
		(start 10 10)
		(end 18 10)
		(width 1)
		(layer "F.Cu")
		(net "N1")
		(uuid "00000000-0000-0000-0000-000000000001")
	)
	(segment
		(start 18 10)
		(end 22 10)
		(width {bridge})
		(layer "F.Cu")
		(net "N1")
		(uuid "00000000-0000-0000-0000-000000000002")
	)
	(segment
		(start 22 10)
		(end 30 10)
		(width 1)
		(layer "F.Cu")
		(net "N1")
		(uuid "00000000-0000-0000-0000-000000000003")
	)
)
"""


def _write_board(d, name, bridge, rules=None):
    """Board + sibling .kicad_pro (rules=None -> no project written)."""
    board = os.path.join(d, name + ".kicad_pcb")
    with open(board, "w") as f:
        f.write(BOARD_TMPL.format(bridge=bridge))
    if rules is not None:
        pro = {"board": {"design_settings": {"rules": rules}},
               "net_settings": {"classes": [{"name": "Default", "clearance": 0.1}]},
               "meta": {"filename": name + ".kicad_pro", "version": 3}}
        with open(os.path.join(d, name + ".kicad_pro"), "w") as f:
            json.dump(pro, f)
    return board


def run():
    if not (KICAD_CLI and os.path.exists(KICAD_CLI)):
        print("SKIP: kicad-cli not found (KICAD_CLI env / PATH / app bundle)")
        return 0

    passed = failed = 0

    def check(name, cond):
        nonlocal passed, failed
        passed += bool(cond)
        failed += not cond
        print(f"  {'PASS' if cond else 'FAIL'}  {name}")

    d = tempfile.mkdtemp(prefix="cw_test_")
    try:
        # FAIL branch, staged via min_track_width: a 0.15 web under a recorded
        # 0.2 min_track_width (no author min_connection) must be flagged.
        b = _write_board(d, "neck", bridge=0.15, rules={"min_track_width": 0.2})
        r = compare_board_data(b)
        check("thin web flagged (min_track_width staging)",
              r.get("kicad_connection_width") == 1)
        check("graded at the min_track_width floor",
              r.get("connection_width_min") == 0.2)
        check("web item carries its net",
              any("N1" in wv["nets"] for wv in r.get("connection_width_items", [])))
        check("web items never enter kicad_only",
              r.get("kicad_only") == 0 and r.get("checkdrc_only") == 0)

        # PASS branch: a web at exactly the floor is legal copper.
        b = _write_board(d, "wide", bridge=0.2, rules={"min_track_width": 0.2})
        r = compare_board_data(b)
        check("web at the floor is clean", r.get("kicad_connection_width") == 0)

        # Author rule wins: min_connection 0.3 over min_track_width 0.2 -- a
        # 0.25 web is legal by track width but violates the author's rule.
        b = _write_board(d, "author", bridge=0.25,
                         rules={"min_connection": 0.3, "min_track_width": 0.2})
        r = compare_board_data(b)
        check("author-set min_connection wins over min_track_width",
              r.get("kicad_connection_width") == 1
              and r.get("connection_width_min") == 0.3)

        # Not graded: no .kicad_pro -> no clearance, no floor -> None (visible
        # as not-graded), never a fake clean 0.
        b = _write_board(d, "nopro", bridge=0.15, rules=None)
        r = compare_board_data(b)
        check("no recorded floor grades as None, not 0",
              r.get("kicad_connection_width") is None)

        # No floor recorded (project exists, neither key): also not graded.
        b = _write_board(d, "nofloor", bridge=0.15, rules={})
        r = compare_board_data(b)
        check("project without floor keys grades as None",
              r.get("kicad_connection_width") is None)
    finally:
        shutil.rmtree(d, ignore_errors=True)

    print(f"{passed}/{passed + failed} connection_width grading tests passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(run())
