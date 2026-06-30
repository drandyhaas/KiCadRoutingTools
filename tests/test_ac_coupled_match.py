#!/usr/bin/env python3
"""
Tests for end-to-end AC-coupled differential-pair length matching (issue #196).

Covers:
  * XNet detection (board-free unit tests): symmetric cap chains are stitched;
    asymmetric / DNP / decoupling / cross-polarity cases are NOT; multi-cap chains
    fold into one group.
  * Concatenated, polarity-aware measurement (_ac_coupled_member_lengths unit test).
  * Integration on the issue's repro board (kicad_files/cap_chain.kicad_pcb): with
    --ac-couple-match the XNet is detected, the concatenated P-vs-N skew is reported
    in JSON_SUMMARY, and both sides still route.
  * Control: without the flag there is no ac_coupled_xnets key (per-side behavior,
    so the routed board is unaffected).

Run with a Python that has the Rust router + numpy/scipy/shapely, e.g.:
    python3 tests/test_ac_coupled_match.py
"""
import json
import os
import re
import subprocess
import sys
from types import SimpleNamespace

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)
sys.path.insert(0, os.path.join(ROOT_DIR, "rust_router"))

from kicad_parser import Segment                       # noqa: E402
from routing_config import DiffPairNet                 # noqa: E402
from diff_xnet import find_ac_coupled_xnets            # noqa: E402
from length_matching import _ac_coupled_member_lengths  # noqa: E402

BOARD = os.path.join(ROOT_DIR, "kicad_files", "cap_chain.kicad_pcb")
GEOM = ["--layers", "F.Cu", "B.Cu", "--track-width", "0.15", "--diff-pair-gap", "0.25",
        "--diff-pair-intra-match", "--no-fix-polarity"]
NETS = ["DPA_P", "DPA_N", "DPB_P", "DPB_N"]


# ----------------------------------------------------------------------------- helpers
def _pad(net_id):
    return SimpleNamespace(net_id=net_id)


def _fp(pad_nets, dnp=False):
    return SimpleNamespace(pads=[_pad(n) for n in pad_nets], dnp=dnp)


def _pairs(*specs):
    return {b: DiffPairNet(base_name=b, p_net_id=p, n_net_id=n,
                           p_net_name=b + "_P", n_net_name=b + "_N")
            for b, p, n in specs}


def _detect(footprints, pairs):
    return find_ac_coupled_xnets(SimpleNamespace(footprints=footprints), pairs)


def _run_route(out, extra):
    cmd = [sys.executable, "route_diff.py", BOARD, out, "--nets", *NETS] + GEOM + extra
    r = subprocess.run(cmd, cwd=ROOT_DIR, capture_output=True, text=True)
    m = re.search(r"JSON_SUMMARY:\s*(\{.*\})", r.stdout)
    summary = json.loads(m.group(1)) if m else None
    return r, summary


# ----------------------------------------------------------------------------- tests
def run():
    fails = []

    # nets: DPA_P=1 DPA_N=2 DPB_P=3 DPB_N=4 DPC_P=5 DPC_N=6 GND=99
    AB = _pairs(("DPA", 1, 2), ("DPB", 3, 4))

    # 1. symmetric: C1 bridges P (1<->3), C2 bridges N (2<->4) -> one XNet, 2 members
    xn, w = _detect({"C1": _fp([1, 3]), "C2": _fp([2, 4])}, AB)
    if not (len(xn) == 1 and len(xn[0].members) == 2 and not w):
        fails.append(f"symmetric: expected 1 XNet of 2 members, got {len(xn)} xnets / warns={w}")
    elif sorted(xn[0].base_names) != ["DPA", "DPB"] or sorted(xn[0].bridge_refs) != ["C1", "C2"]:
        fails.append(f"symmetric: wrong members/bridges: {xn[0].base_names} {xn[0].bridge_refs}")

    # 2. asymmetric (only P bridged) -> no XNet, one warning
    xn, w = _detect({"C1": _fp([1, 3])}, AB)
    if len(xn) != 0 or len(w) != 1:
        fails.append(f"asymmetric: expected 0 XNets + 1 warning, got {len(xn)} / {len(w)}")

    # 3. DNP on the P bridge -> only N bridged -> asymmetric -> no XNet
    xn, w = _detect({"C1": _fp([1, 3], dnp=True), "C2": _fp([2, 4])}, AB)
    if len(xn) != 0:
        fails.append(f"DNP: a no-pop cap must not bridge; expected 0 XNets, got {len(xn)}")

    # 4. decoupling cap (a pad on GND=99, not a pair net) ignored; real chain still found
    xn, w = _detect({"C1": _fp([1, 3]), "C2": _fp([2, 4]), "C3": _fp([1, 99])}, AB)
    if not (len(xn) == 1 and len(xn[0].members) == 2):
        fails.append(f"decoupling-cap: expected 1 XNet of 2 members, got {len(xn)}")

    # 5. chain A--B--C via 4 caps -> one XNet of 3 members
    ABC = _pairs(("DPA", 1, 2), ("DPB", 3, 4), ("DPC", 5, 6))
    xn, w = _detect({"C1": _fp([1, 3]), "C2": _fp([2, 4]),
                     "C3": _fp([3, 5]), "C4": _fp([4, 6])}, ABC)
    if not (len(xn) == 1 and len(xn[0].members) == 3):
        fails.append(f"chain: expected 1 XNet of 3 members, got "
                     f"{len(xn)} / {[m.base_name for m in xn[0].members] if xn else None}")

    # 6. inert: no caps -> nothing detected, no warnings
    xn, w = _detect({}, AB)
    if xn != [] or w != []:
        fails.append(f"inert: expected nothing, got {len(xn)} xnets / {len(w)} warns")

    # 7. cross-polarity (P<->N swapped) bridge -> not supported, ignored
    xn, w = _detect({"C1": _fp([1, 4]), "C2": _fp([2, 3])}, AB)
    if len(xn) != 0:
        fails.append(f"cross-polarity: expected 0 XNets, got {len(xn)}")

    # 8. measurement: concatenated, polarity-aware per-member lengths.
    def seg(x1, y1, x2, y2, net):
        return Segment(start_x=x1, start_y=y1, end_x=x2, end_y=y2,
                       width=0.1, layer="F.Cu", net_id=net)
    fake_pcb = SimpleNamespace()
    res = {"is_diff_pair": True, "p_net_id": 1, "n_net_id": 2,
           "new_segments": [seg(0, 0, 10, 0, 1), seg(0, 1, 12, 1, 2)],  # P=10mm, N=12mm
           "new_vias": [],
           "p_src_stub_length": 0.0, "p_tgt_stub_length": 0.0,
           "n_src_stub_length": 0.0, "n_tgt_stub_length": 0.0,
           "polarity_fixed": False}
    md = _ac_coupled_member_lengths(res, 1, 2, fake_pcb)
    if not (abs(md["p_len"] - 10.0) < 1e-6 and abs(md["n_len"] - 12.0) < 1e-6):
        fails.append(f"measurement: P/N routed length wrong: {md['p_len']} / {md['n_len']}")

    # 8b. polarity swap re-routes the target stubs between P and N
    res2 = dict(res, polarity_fixed=True,
                p_src_stub_length=1.0, p_tgt_stub_length=2.0,
                n_src_stub_length=3.0, n_tgt_stub_length=4.0)
    md2 = _ac_coupled_member_lengths(res2, 1, 2, fake_pcb)
    # swapped: p_stub = p_src + n_tgt = 1+4 = 5 ; n_stub = n_src + p_tgt = 3+2 = 5
    if not (abs(md2["p_len"] - 15.0) < 1e-6 and abs(md2["n_len"] - 17.0) < 1e-6):
        fails.append(f"polarity-swap stubs wrong: p_len={md2['p_len']} n_len={md2['n_len']}")

    # 9. INTEGRATION on the issue's repro board: detection + reporting + routes.
    if not os.path.exists(BOARD):
        fails.append(f"integration: fixture missing: {BOARD}")
    else:
        out = os.path.join(TESTS_DIR, "_cap_chain_out.kicad_pcb")
        try:
            r, summary = _run_route(out, ["--ac-couple-match"])
            if summary is None:
                fails.append("integration: no JSON_SUMMARY emitted\n" + r.stdout[-800:] + r.stderr[-800:])
            elif summary.get("successful") != 2:
                fails.append(f"integration: expected 2 pairs routed, got {summary.get('successful')}")
            elif "ac_coupled_xnets" not in summary:
                fails.append("integration: --ac-couple-match did not add 'ac_coupled_xnets' to JSON_SUMMARY")
            else:
                xnets = summary["ac_coupled_xnets"]
                if len(xnets) != 1 or xnets[0].get("nets") != "DPA+DPB":
                    fails.append(f"integration: wrong XNet summary: {xnets}")
                elif sorted(xnets[0].get("bridges", [])) != ["C1", "C2"]:
                    fails.append(f"integration: wrong bridges: {xnets[0].get('bridges')}")
                elif not isinstance(xnets[0].get("skew_mm"), (int, float)):
                    fails.append(f"integration: skew_mm not reported: {xnets[0]}")

            # 10. CONTROL: without the flag, no ac_coupled_xnets key (per-side behavior).
            r2, summary2 = _run_route(out, [])
            if summary2 is None:
                fails.append("control: no JSON_SUMMARY emitted")
            elif "ac_coupled_xnets" in summary2:
                fails.append("control: ac_coupled_xnets present without --ac-couple-match (not inert)")
        finally:
            base = out[:-len(".kicad_pcb")]
            for ext in (".kicad_pcb", ".kicad_pro", ".kicad_prl"):
                if os.path.exists(base + ext):
                    os.remove(base + ext)

    # ----------------------------------------------------------------------- report
    if fails:
        for f in fails:
            print(f"  FAIL  {f}")
        print(f"\n{len(fails)} check(s) FAILED")
        return False
    print("PASS  AC-coupled XNet detection + measurement + integration")
    return True


if __name__ == "__main__":
    sys.exit(0 if run() else 1)
