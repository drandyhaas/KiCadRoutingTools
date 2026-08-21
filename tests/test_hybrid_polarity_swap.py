#!/usr/bin/env python3
"""Issue #266: the hybrid coupled-middle escape must be able to produce a
polarity (P/N pad) swap for a SIDE-FLIPPED pair, instead of always wrapping one
leg around its partner.

watchy's USB_D pair reaches the hybrid escape (`_route_direct_coupled_middle`,
via the diff_pair_loop HYBRID ESCAPE site) and is genuinely side-flipped: on the
fanned board the P pad sits on the +side of the coupled centerline at the U4
source and on the -side at the U5 target. The hybrid ALREADY computes that flip
and used to discard it ("costs one crossing either way"), and its success dict
carried no `polarity_fixed` / `swap_target_pads`, so `apply_polarity_swap` bailed
out immediately and no hybrid route could ever swap pads. One leg therefore
wrapped around its partner: 23 segments for USB_D- against 13 for USB_D+.

THREE ARMS, all required:

  ARM 1 -- INERTNESS. Polarity swaps are DENY-BY-DEFAULT (#279): without
  `--polarity-swap-nets` the pair is not permitted a swap, so the routed copper
  must be EXACTLY what it was before this feature existed. Pinned to 1um and to
  the exact segment counts. This is the regression gate that makes a corpus A/B
  unnecessary -- every stress replay that omits the flag is unchanged by
  construction. The only permitted difference is the new DISCLOSURE (the pair is
  named in JSON_SUMMARY.polarity_flip_unswapped_pairs), which touches no copper.

  ARM 2 -- THE FIX. With `--polarity-swap-nets '*'` the hybrid must assemble the
  target-swapped candidate, win on it, report the pair in
  JSON_SUMMARY.polarity_swapped_pairs, physically exchange the U5 pad nets in the
  OUTPUT board, and ship less copper than arm 1 -- still fully connected and
  DRC-clean at the routed clearance.

  ARM 3 -- THE VIA-FANNED TARGET, which is the topology the feature was actually
  written for (a BGA/QFN escape, or the fpga_sdram case in the issue) and which
  arms 1-2 do NOT cover: watchy's U5 is bare SMD pads with no via. The same
  fixture with a stub + via at each U5 target reproduces a gate that was only
  half-taught -- `_connector_grazes_foreign_copper` re-bucketed the swapped PADS
  but read every VIA at its pre-swap net, so the swapped leg was graded as
  grazing a "foreign" via the pending swap hands to its own net, and BOTH swap
  candidates were rejected ("assembled route grazes foreign via (net USB_D-) by
  375um"). With the via check taught the same pending-swap view, that candidate
  is legal (len 30.690, 4 vias).

  Note what arm 3 does and does not assert. The gate is checked DIRECTLY, at the
  function, because on this particular fixture the now-legal swap candidate still
  LOSES the selector's own comparison (30.690mm/4 vias vs 32.892mm/2 vias for the
  unswapped route), so the end-to-end outcome is the same with or without the
  fix. The end-to-end half of the arm therefore only pins that the run stays
  clean and that the side flip is disclosed. The reason the swapped route pays
  two extra vias is that its legs cannot reuse the partner's existing target via
  as a launch island -- a separate limitation, not fixed here.

NOTE on the fixture: watchy USB_D is USB D+/D-, which this repo's own
identify-diff-pairs skill classifies as polarity-CRITICAL and never to be swapped
in practice. Forcing `--polarity-swap-nets '*'` here is a legitimate way to
exercise the MECHANISM; it is not a swap a user should make on this board.

Run:
    python3 tests/test_hybrid_polarity_swap.py [-v]
"""

import argparse
import json
import math
import os
import re
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_router'))  # #522
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_tools'))  # #522

from kicad_parser import parse_kicad_pcb, Segment
from routing_config import GridRouteConfig
from routing_utils import pos_key
from diff_pair_routing import _connector_grazes_foreign_copper

BOARD = os.path.join(ROOT_DIR, "kicad_files", "watchy.kicad_pcb")
NETS = ["USB_D+", "USB_D-"]
PAIR = "USB_D"
CLEARANCE = "0.1"
FAN = ["--component", "U4", "--nets", "*", "!GND", "!+3V3", "--width", "0.1"]
DIFF_GEOM = ["--layers", "F.Cu", "In1.Cu", "In2.Cu", "B.Cu",
             "--track-width", "0.1", "--diff-pair-gap", "0.15",
             "--clearance", CLEARANCE, "--via-size", "0.3", "--via-drill", "0.2",
             "--no-gnd-vias", "--grid-step", "0.05"]

# Measured on main before the fix (and unchanged by it, without the flag).
BASE = {"USB_D+": (17.889, 13), "USB_D-": (18.821, 23)}
BASE_TOTAL_MM = round(BASE["USB_D+"][0] + BASE["USB_D-"][0], 3)   # 36.710

# ARM 3 fixture: a 0.6mm stub west out of each U5 USB_D pad, ending in a via --
# the ordinary escape topology (the pad is not the terminal; the via is).
VIA_P = (94.157, 90.942)   # end of the USB_D+ target stub
VIA_N = (94.157, 92.017)   # end of the USB_D- target stub
VIA_FIXTURE = """\t(via
\t\t(at 94.157000 90.942000)
\t\t(size 0.45)
\t\t(drill 0.25)
\t\t(layers "F.Cu" "B.Cu")
\t\t(net "USB_D+")
\t\t(uuid "bbbb1111-0000-4000-8000-000000000001")
\t)
\t(via
\t\t(at 94.157000 92.017000)
\t\t(size 0.45)
\t\t(drill 0.25)
\t\t(layers "F.Cu" "B.Cu")
\t\t(net "USB_D-")
\t\t(uuid "bbbb1111-0000-4000-8000-000000000002")
\t)
\t(segment
\t\t(start 94.757000 90.942000)
\t\t(end 94.157000 90.942000)
\t\t(width 0.1)
\t\t(layer "F.Cu")
\t\t(net "USB_D+")
\t\t(uuid "aaaa1111-0000-4000-8000-000000000001")
\t)
\t(segment
\t\t(start 94.757000 92.017000)
\t\t(end 94.157000 92.017000)
\t\t(width 0.1)
\t\t(layer "F.Cu")
\t\t(net "USB_D-")
\t\t(uuid "aaaa1111-0000-4000-8000-000000000002")
\t)
"""


def _add_target_vias(src, dst):
    """Append the ARM 3 stub+via pair to a fanned board, inside its root list."""
    txt = open(src, encoding="utf-8", newline="").read()
    extra = VIA_FIXTURE.replace("\n", "\r\n") if "\r\n" in txt else VIA_FIXTURE
    i = txt.rindex(")")
    with open(dst, "w", encoding="utf-8", newline="") as fh:
        fh.write(txt[:i] + extra + txt[i:])


def _run(args, verbose):
    r = subprocess.run([sys.executable, *args], cwd=ROOT_DIR,
                       capture_output=True, text=True)
    txt = r.stdout + r.stderr
    if verbose:
        print(txt)
    return txt


def _summary(txt):
    m = re.search(r"JSON_SUMMARY:\s*(\{.*\})", txt)
    return json.loads(m.group(1)) if m else {}


def _measure(board):
    """{net_name: (copper_mm, n_segments)} plus {pad_number: net_name} for U5."""
    pcb = parse_kicad_pcb(board)
    ids = {net.name: nid for nid, net in pcb.nets.items()}
    out = {}
    for name in NETS:
        segs = [s for s in pcb.segments if s.net_id == ids.get(name)]
        out[name] = (sum(math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)
                         for s in segs), len(segs))
    u5 = {}
    fp = pcb.footprints.get("U5")
    for pad in (fp.pads if fp else []):
        if pad.net_name in NETS:
            u5[pad.pad_number] = pad.net_name
    return out, u5


def run():
    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    if not os.path.exists(BOARD):
        print(f"FAIL: board not found: {BOARD}")
        return 1

    fails = []

    def check(ok, msg):
        print(("  ok   " if ok else "  FAIL ") + msg)
        if not ok:
            fails.append(msg)

    tmp = []

    def _tmp(prefix):
        fd, p = tempfile.mkstemp(suffix=".kicad_pcb", prefix=prefix)
        os.close(fd)
        tmp.append(p)
        # route_diff writes a sibling .kicad_pro / .kicad_dru; clean those too.
        for ext in (".kicad_pro", ".kicad_dru"):
            tmp.append(os.path.splitext(p)[0] + ext)
        return p

    try:
        # Surface fanout of the QFN -> bare F.Cu USB_D stubs; the pair then only
        # routes through the hybrid escape.
        fan = _tmp("h266_fan_")
        ftxt = _run(["py_router/qfn_fanout.py", BOARD, *FAN, "--output", fan], args.verbose)
        if not (os.path.exists(fan) and os.path.getsize(fan) > 0):
            print("FAIL: qfn_fanout produced no board\n" + ftxt)
            return 1

        # ---------------- ARM 1: inertness (no --polarity-swap-nets) --------
        print("ARM 1 - inertness (swaps deny-by-default, #279)")
        a1 = _tmp("h266_noswap_")
        t1 = _run(["py_router/route_diff.py", fan, "--nets", *NETS, *DIFF_GEOM,
                   "--output", a1], args.verbose)
        s1 = _summary(t1)
        check(s1.get("successful") == 1 and not s1.get("failed"),
              f"pair routes 1/1 (routed={s1.get('routed_diff_pairs')}, "
              f"failed={s1.get('failed_diff_pairs')})")
        check("DIRECT HYBRID: coupled middle" in t1,
              "the hybrid coupled middle is the path under test")
        check(s1.get("polarity_swapped_pairs") == [],
              f"no polarity swap without the flag "
              f"(got {s1.get('polarity_swapped_pairs')})")
        if os.path.exists(a1) and os.path.getsize(a1) > 0:
            m1, u5_1 = _measure(a1)
            for name, (mm, nseg) in BASE.items():
                got_mm, got_seg = m1[name]
                check(abs(got_mm - mm) < 1e-3,
                      f"{name} copper unchanged: {got_mm:.3f} mm (pinned {mm:.3f})")
                check(got_seg == nseg,
                      f"{name} segment count unchanged: {got_seg} (pinned {nseg})")
            check(u5_1 == {"1": "USB_D+", "3": "USB_D-"},
                  f"U5 pad nets untouched: {u5_1}")
        else:
            check(False, "arm 1 wrote an output board")
            m1 = None

        # The one deliberate arm-1 change: the side flip is DISCLOSED (plan C).
        # It moves no copper -- the pins above are what guarantee that.
        check(s1.get("polarity_flip_unswapped_pairs") == [PAIR],
              f"side flip disclosed in the summary "
              f"(got {s1.get('polarity_flip_unswapped_pairs')})")

        # ---------------- ARM 2: the fix (--polarity-swap-nets '*') ---------
        print("ARM 2 - the fix (--polarity-swap-nets '*')")
        a2 = _tmp("h266_swap_")
        t2 = _run(["py_router/route_diff.py", fan, "--nets", *NETS,
                   "--polarity-swap-nets", "*", *DIFF_GEOM, "--output", a2],
                  args.verbose)
        s2 = _summary(t2)
        check(s2.get("successful") == 1 and not s2.get("failed"),
              f"pair still routes 1/1 (routed={s2.get('routed_diff_pairs')}, "
              f"failed={s2.get('failed_diff_pairs')})")
        check("DIRECT HYBRID: coupled middle" in t2,
              "still the hybrid coupled middle (not a different path)")
        check(s2.get("polarity_swapped_pairs") == [PAIR],
              f"the hybrid produced a polarity swap "
              f"(got {s2.get('polarity_swapped_pairs')})")
        if os.path.exists(a2) and os.path.getsize(a2) > 0:
            m2, u5_2 = _measure(a2)
            tot2 = m2["USB_D+"][0] + m2["USB_D-"][0]
            seg2 = m2["USB_D+"][1] + m2["USB_D-"][1]
            check(tot2 < BASE_TOTAL_MM - 1e-3,
                  f"pair copper shrinks: {tot2:.3f} mm < {BASE_TOTAL_MM:.3f} mm "
                  f"(-{BASE_TOTAL_MM - tot2:.3f} mm, {seg2} segs vs "
                  f"{BASE['USB_D+'][1] + BASE['USB_D-'][1]})")
            # The swap is PHYSICAL: the output board's connector pads changed net.
            check(u5_2 == {"1": "USB_D-", "3": "USB_D+"},
                  f"U5 pad nets actually exchanged: {u5_2}")
            conn = _run(["py_router/check_connected.py", a2, "--nets", *NETS],
                        args.verbose)
            check("ALL NETS FULLY CONNECTED" in conn,
                  "swapped pair is fully connected")
            drc = _run(["py_router/check_drc.py", a2, "--clearance", CLEARANCE],
                       args.verbose)
            check("NO DRC VIOLATIONS" in drc,
                  f"swapped pair is DRC-clean at {CLEARANCE}mm")
        else:
            check(False, "arm 2 wrote an output board")

        # ---------------- ARM 3: via-fanned target --------------------------
        print("ARM 3 - via-fanned target (the escape topology the fix is for)")
        fanv = _tmp("h266_fanvia_")
        _add_target_vias(fan, fanv)
        # route_diff wants the sibling project (DRC floor) alongside the board.
        with open(os.path.splitext(fan)[0] + ".kicad_pro", encoding="utf-8") as fh:
            _pro = fh.read()
        with open(os.path.splitext(fanv)[0] + ".kicad_pro", "w",
                  encoding="utf-8") as fh:
            fh.write(_pro)

        pcb = parse_kicad_pcb(fanv)
        ids = {net.name: nid for nid, net in pcb.nets.items()}
        p_id, n_id = ids["USB_D+"], ids["USB_D-"]
        vnets = {(round(v.x, 3), round(v.y, 3)): v.net_id for v in pcb.vias}
        check(vnets.get(VIA_P) == p_id and vnets.get(VIA_N) == n_id,
              f"fixture has a target via on each net (got {sorted(vnets)[:4]})")

        cfg = GridRouteConfig()
        cfg.layers = ["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"]
        cfg.clearance = float(CLEARANCE)
        cfg.grid_step = 0.05
        # What a target-swapped P leg does: run to the N target via, which
        # apply_polarity_swap is about to rename to USB_D+.
        leg = Segment(start_x=93.0, start_y=VIA_N[1], end_x=VIA_N[0], end_y=VIA_N[1],
                      width=0.1, layer="F.Cu", net_id=p_id)

        def _graze(**kw):
            try:
                g = _connector_grazes_foreign_copper([leg], pcb, p_id, n_id, cfg, **kw)
            except TypeError as exc:
                return f"TypeError: {exc}"   # the gate never learned about vias
            return (g[0], round(g[3] * 1000)) if g else None

        g_plain = _graze()
        check(g_plain == ("via", 375),
              f"without the pending-swap view the leg reads a foreign via "
              f"(got {g_plain}, want ('via', 375))")
        g_swap = _graze(n_swap_positions={pos_key(*VIA_N)})
        check(g_swap is None,
              f"with it, the via the swap hands over is this leg's own "
              f"(got {g_swap}, want None)")
        # Position-scoped, not a blanket partner-via exemption: naming the OTHER
        # target's position must leave this graze standing.
        g_other = _graze(n_swap_positions={pos_key(*VIA_P)})
        check(g_other == ("via", 375),
              f"the exemption is scoped to the swapped positions "
              f"(got {g_other}, want ('via', 375))")

        # End-to-end on the same fixture. The swapped candidate is legal here but
        # loses the selector's own comparison on via count, so this half only
        # pins that the run stays clean and honest -- it is NOT a gate on the
        # via fix (see the module docstring).
        a3 = _tmp("h266_via_")
        t3 = _run(["py_router/route_diff.py", fanv, "--nets", *NETS,
                   "--polarity-swap-nets", "*", *DIFF_GEOM, "--output", a3],
                  args.verbose)
        s3 = _summary(t3)
        check(s3.get("successful") == 1 and not s3.get("failed"),
              f"via-fanned pair routes 1/1 (routed={s3.get('routed_diff_pairs')}, "
              f"failed={s3.get('failed_diff_pairs')})")
        check("DIRECT HYBRID: coupled middle" in t3,
              "still the hybrid coupled middle")
        check(s3.get("polarity_flip_unswapped_pairs") == [PAIR],
              f"the declined swap is disclosed, not silent "
              f"(got {s3.get('polarity_flip_unswapped_pairs')})")
        if os.path.exists(a3) and os.path.getsize(a3) > 0:
            conn3 = _run(["py_router/check_connected.py", a3, "--nets", *NETS],
                         args.verbose)
            check("ALL NETS FULLY CONNECTED" in conn3,
                  "via-fanned pair is fully connected")
            drc3 = _run(["py_router/check_drc.py", a3, "--clearance", CLEARANCE],
                        args.verbose)
            check("NO DRC VIOLATIONS" in drc3,
                  f"via-fanned pair is DRC-clean at {CLEARANCE}mm")
        else:
            check(False, "arm 3 wrote an output board")
    finally:
        for p in tmp:
            if os.path.exists(p):
                os.remove(p)

    if fails:
        print(f"\nFAIL ({len(fails)}): " + "; ".join(fails))
        return 1
    print("\nAll checks passed (#266: hybrid polarity swap fires under "
          "--polarity-swap-nets and is inert without it)")
    return 0


if __name__ == '__main__':
    sys.exit(run())
