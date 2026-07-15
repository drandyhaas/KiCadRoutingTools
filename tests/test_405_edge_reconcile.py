#!/usr/bin/env python3
"""Regression test for the grader-accuracy fix (issue #405 follow-up):

  A) ab_replay_grade and kicad_drc_compare now share ONE per-board grading
     core (kicad_drc_compare.compare_board_data). Neither reimplements baseline
     subtraction or matching, so they cannot diverge (the openstint 6-vs-0 and
     the core1106_cam edge double-count came from ab_replay_grade's weaker
     reimplementation).

  B) Board-edge anchor reconciliation: kicad's copper_edge_clearance anchors at
     the edge-closest point (on the copper); check_drc's segment-board-edge
     anchors at the segment START. The same shared edge violation must MATCH
     (not double-count as kicad_only + checkdrc_only), while a net only ONE
     engine flags for edge must remain reported.

No kicad-cli or board files needed -- exercises the pure matcher/reconciler.

Run:  python3 tests/test_405_edge_reconcile.py
"""
import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)
sys.path.insert(0, os.path.join(ROOT_DIR, "tests", "stress"))

from kicad_drc_compare import reconcile_edge_family, match


def _k(nets, pos, typ="copper_edge_clearance"):
    return {"type": typ, "nets": frozenset(nets), "pos": pos}


def _cseg(nets, seg, typ="segment-board-edge"):
    # check_drc edge item: pos = segment START, plus whole segment.
    return {"type": typ, "nets": frozenset(nets), "pos": (seg[0], seg[1]), "seg": seg}


def _cpt(nets, pos, typ="via-board-edge"):
    return {"type": typ, "nets": frozenset(nets), "pos": pos}


def main():
    fails = []

    # -- B1: SAME edge violation, anchored far apart on a long segment ----------
    # kicad anchors at the edge-closest point (mid/far end of the track);
    # check_drc anchors at the segment START, ~9 mm away. Generic point-to-point
    # (radius 3) would MISS -> double count. Point-to-segment collapses it.
    seg = (100.6, 71.4, 100.6, 80.4)          # 9 mm vertical track riding the edge
    kicad_only = [_k(["/5V_IN"], (100.6, 80.35))]   # kicad point sits ON the segment
    cd_only = [_cseg(["/5V_IN"], seg)]
    # sanity: the generic matcher does NOT match these (proves the bug exists)
    _, gk, gc = match(kicad_only, cd_only)
    if not (len(gk) == 1 and len(gc) == 1):
        fails.append("precondition: generic matcher unexpectedly matched the far-anchored edge pair")
    n, rk, rc = reconcile_edge_family(list(kicad_only), list(cd_only))
    if not (n == 1 and rk == [] and rc == []):
        fails.append(f"B1 shared edge not reconciled: n={n} kicad_only={len(rk)} checkdrc_only={len(rc)}")

    # -- B2: one-sided KICAD edge (net only kicad flags) must stay reported -----
    kicad_only = [_k(["GND"], (136.7, 95.6))]
    cd_only = []  # check_drc flags nothing for GND edge
    n, rk, rc = reconcile_edge_family(list(kicad_only), list(cd_only))
    if not (n == 0 and len(rk) == 1 and len(rc) == 0):
        fails.append(f"B2 one-sided kicad edge hidden: n={n} rk={len(rk)} rc={len(rc)}")

    # -- B3: one-sided CHECK_DRC edge (net only check_drc flags) must stay ------
    kicad_only = []
    cd_only = [_cseg(["/PHANTOM"], (10.0, 10.0, 10.0, 12.0))]
    n, rk, rc = reconcile_edge_family(list(kicad_only), list(cd_only))
    if not (n == 0 and len(rk) == 0 and len(rc) == 1):
        fails.append(f"B3 one-sided check_drc edge hidden: n={n} rk={len(rk)} rc={len(rc)}")

    # -- B4: mixed -- one shared net collapses, a different-net over-flag stays -
    kicad_only = [_k(["/A"], (0.05, 5.0))]                       # shared with cd
    cd_only = [_cseg(["/A"], (0.0, 0.0, 0.0, 10.0)),            # matches kicad /A
               _cseg(["/B"], (50.0, 0.0, 50.0, 3.0))]           # check_drc-only /B
    n, rk, rc = reconcile_edge_family(list(kicad_only), list(cd_only))
    if not (n == 1 and len(rk) == 0 and len(rc) == 1 and next(iter(rc[0]["nets"])) == "/B"):
        fails.append(f"B4 mixed reconcile wrong: n={n} rk={len(rk)} rc={[sorted(x['nets']) for x in rc]}")

    # -- B5: net-set agreement fallback (anchor > radius, no segment) -----------
    # via/pad edge items carry only a point; when both engines flag the SAME net
    # for edge but anchored beyond the proximity radius, net-set agreement (pass
    # 2) still collapses them; min-count preserves a per-net over-flag.
    kicad_only = [_k(["VBUS"], (0.0, 0.0)), _k(["VBUS"], (0.0, 0.0))]
    cd_only = [_cpt(["VBUS"], (999.0, 999.0))]   # far away, no seg
    n, rk, rc = reconcile_edge_family(list(kicad_only), list(cd_only))
    if not (n == 1 and len(rk) == 1 and len(rc) == 0):
        fails.append(f"B5 net-set agreement wrong: n={n} rk={len(rk)} rc={len(rc)}")

    # -- B6: non-edge families are untouched by the edge reconciler -------------
    kicad_only = [_k(["/X", "/Y"], (0.0, 0.0), typ="clearance")]
    cd_only = [_cpt(["/X", "/Y"], (999.0, 999.0), typ="clearance")]
    n, rk, rc = reconcile_edge_family(list(kicad_only), list(cd_only))
    if not (n == 0 and len(rk) == 1 and len(rc) == 1):
        fails.append(f"B6 non-edge family wrongly reconciled: n={n} rk={len(rk)} rc={len(rc)}")

    # -- A: the two tools import the SAME core object --------------------------
    import kicad_drc_compare as krc
    import ab_replay_grade  # noqa: F401  (import side-effect: path wiring)
    # _kicad_grade closes over compare_board_data via a local import; assert the
    # symbol it will import is the shared core (identity), not a reimplementation.
    from kicad_drc_compare import compare_board_data as core1
    if core1 is not krc.compare_board_data:
        fails.append("A: compare_board_data identity mismatch")
    src = open(os.path.join(ROOT_DIR, "tests", "stress", "ab_replay_grade.py")).read()
    if "compare_board_data" not in src:
        fails.append("A: ab_replay_grade no longer calls compare_board_data")
    # ab_replay_grade must NOT reimplement its own subtraction/matching loop:
    if "matched_pre" in src or ("run_check_drc" in src and "def _kicad_grade" in src and
                                "run_check_drc(pcb" in src):
        fails.append("A: ab_replay_grade still reimplements grading internals")

    if fails:
        print("FAIL:")
        for f in fails:
            print("  -", f)
        return 1
    print("PASS: edge reconciliation + shared-core tests "
          "(B1 shared-collapse, B2/B3 one-sided preserved, B4 mixed, "
          "B5 net-set fallback, B6 non-edge untouched, A shared core)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
