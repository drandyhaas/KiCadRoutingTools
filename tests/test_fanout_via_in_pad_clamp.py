#!/usr/bin/env python3
"""Issue #202: a via dropped INSIDE a pad must never bulge past the pad edge.

The fanout can place a via-in-pad whose configured diameter exceeds the pad it
sits in (e.g. a Ø0.5 via in a 0.41 mm keks pad). The via copper then bulges past
the pad edge and a neighbouring different-net trace that legitimately cleared the
(smaller) pad grazes the (larger) via -- a real VIA-SEGMENT DRC error baked into
the board before any signal routing (163 of them on keks).

The fix sizes the via to fit its pad AT PLACEMENT (so the escape's own keep-out
uses the real, smaller via and neighbouring fanout tracks can route past it),
clamped to the fab via floor. This test covers:

  1. clamp_via_to_pad math: fits / clamped-to-pad / held-at-floor (still bulges).
  2. Every fanout method that drops a via-in-pad wires the clamp in, so no placed
     via bulges past its pad beyond the unavoidable fab floor: bga_fanout channel,
     bga_fanout underpad, qfn_fanout stub, qfn_fanout underpad.

Run:  python3 tests/test_fanout_via_in_pad_clamp.py [-v]
"""
import argparse
import contextlib
import io
import os
import sys
from types import SimpleNamespace

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from kicad_parser import parse_kicad_pcb
from list_nets import fab_floors, fab_floor_min, fab_floor_ladder
from bga_fanout import generate_bga_fanout
from bga_fanout.geometry import clamp_via_to_pad
from qfn_fanout import generate_qfn_fanout

BGA_BOARD = os.path.join(ROOT, "kicad_files", "ulx3s.kicad_pcb")            # 4-layer, 0.40 BGA pads
QFN_BOARD = os.path.join(ROOT, "kicad_files", "qfn_diffpair_escape.kicad_pcb")  # 2-layer, 0.25-wide pads
LAYERS = ["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"]


def _pad(sx, sy):
    return SimpleNamespace(size_x=sx, size_y=sy)


def test_clamp_math(verbose):
    """clamp_via_to_pad: fits / clamped / floor along the standard-tier ladder.

    The clamp walks the fab-tier floor ladder (issue #237): the multilayer standard
    tier = [standard 0.45/0.20, fine 0.30/0.15, advanced 0.25/0.15]. A pad below the
    standard via escalates to the fine rung (rung 1), then the advanced rung; a pad
    below even the deepest via is held at it.
    """
    lad = fab_floor_ladder(4)   # [0.45/0.20, 0.30/0.15, 0.25/0.15]
    deepest = len(lad) - 1
    fails = []

    def check(label, got, exp_size, exp_status, exp_rung, pad_min, configured):
        size, drill, status, rung = got
        if status != exp_status:
            fails.append(f"{label}: status {status} != {exp_status}")
        if rung != exp_rung:
            fails.append(f"{label}: rung {rung} != {exp_rung}")
        if abs(size - exp_size) > 1e-6:
            fails.append(f"{label}: size {size} != {exp_size}")
        # universal invariants: never grow, and (unless held at floor) never bulge
        if size > configured + 1e-6:
            fails.append(f"{label}: via grew {size} > configured {configured}")
        if status == 'clamped' and size > pad_min + 1e-6:
            fails.append(f"{label}: clamped via {size} still bulges past pad {pad_min}")
        if drill >= size:
            fails.append(f"{label}: drill {drill} >= size {size}")
        if verbose and not fails:
            print(f"  {label}: {size}/{drill} ({status}, rung {rung})  OK")

    # already fits -> unchanged, nominal rung
    check("fits", clamp_via_to_pad(0.3, 0.2, _pad(0.5, 0.5), lad), 0.3, 'fits', 0, 0.5, 0.3)
    # 0.46 pad still >= standard 0.45 via -> clamps at rung 0 (no escalation)
    check("0.5->0.46 std", clamp_via_to_pad(0.5, 0.3, _pad(0.46, 0.46), lad), 0.46, 'clamped', 0, 0.46, 0.5)
    # keks: Ø0.5 in 0.41 pad < 0.45 standard -> escalate to the fine 0.30 rung, shrink to 0.41
    check("keks 0.5->0.41", clamp_via_to_pad(0.5, 0.3, _pad(0.41, 0.41), lad), 0.41, 'clamped', 1, 0.41, 0.5)
    # ulx3s: Ø0.5 in 0.40 pad -> 0.40 at the fine 0.30 rung
    check("0.5->0.40", clamp_via_to_pad(0.5, 0.3, _pad(0.40, 0.40), lad), 0.40, 'clamped', 1, 0.40, 0.5)
    # rect/oval pad: clamp to the MIN dimension (fine 0.30 rung)
    check("oval min-dim", clamp_via_to_pad(0.5, 0.3, _pad(0.9, 0.42), lad), 0.42, 'clamped', 1, 0.42, 0.5)
    # pad below the deepest via floor (0.25) -> held at the deepest floor, still bulges
    check("below floor", clamp_via_to_pad(0.5, 0.3, _pad(0.22, 0.22), lad), 0.25, 'floor', deepest, 0.22, 0.5)
    # advanced tier directly = single-rung [0.25/0.15] ladder, no escalation possible
    lad_adv = fab_floor_ladder(4, 'advanced')
    check("adv 0.5->0.40", clamp_via_to_pad(0.5, 0.3, _pad(0.40, 0.40), lad_adv), 0.40, 'clamped', 0, 0.40, 0.5)
    check("adv below floor", clamp_via_to_pad(0.5, 0.3, _pad(0.20, 0.20), lad_adv), 0.25, 'floor', 0, 0.20, 0.5)
    return fails


def _via_in_pad_violations(pcb, fp, vias, configured_via, fab):
    """For every via that sits inside a same-net SMD pad, return invariant
    violations. Invariant: a placed via never bulges past its pad beyond the
    unavoidable fab floor, and is never larger than the configured via.
    Returns (n_in_pad, n_clamped_below_cfg, violations)."""
    floor_dia = fab['via_diameter']   # deepest reachable fab via (fab_floor_min)
    pads = [p for p in fp.pads if getattr(p, 'drill', 0) == 0 and p.net_id]
    # Only TRUE via-in-pad vias (centred on the pad) are governed by the clamp; an
    # off-pad staggered via with a connecting stub legitimately extends past a tiny
    # pad. Match by the code's OWN via-in-pad test (hypot <= POSITION_TOLERANCE),
    # not merely landing in the pad bbox -- a 0.45 via parked a hair off a 0.25 pad
    # is an off-pad escape, not a clamp candidate.
    from bga_fanout.constants import POSITION_TOLERANCE
    viols, n_in_pad, n_clamped = [], 0, 0
    for v in vias:
        pad = None
        for p in pads:
            if (p.net_id == v['net_id'] and
                    abs(p.global_x - v['x']) <= POSITION_TOLERANCE and
                    abs(p.global_y - v['y']) <= POSITION_TOLERANCE):
                if pad is None or min(p.size_x, p.size_y) < min(pad.size_x, pad.size_y):
                    pad = p
        if pad is None:
            continue
        n_in_pad += 1
        pad_min = min(pad.size_x, pad.size_y)
        bound = max(pad_min, floor_dia)   # fits the pad, or at worst the fab floor
        if v['size'] > bound + 1e-6:
            viols.append(f"via {v['size']:.3f} in pad {pad_min:.3f} bulges past max(pad,floor)={bound:.3f}")
        if v['size'] > configured_via + 1e-6:
            viols.append(f"via {v['size']:.3f} grew past configured {configured_via:.3f}")
        if v['size'] < configured_via - 1e-6:
            n_clamped += 1
    return n_in_pad, n_clamped, viols


def _run(fn):
    """Run a fanout entry point with its noisy stdout suppressed."""
    with contextlib.redirect_stdout(io.StringIO()):
        return fn()


def test_methods(verbose):
    """Each via-dropping fanout method clamps its via-in-pads at placement."""
    fails = []
    fab4 = fab_floor_min(4)   # deepest reachable fab floor (the advanced via)
    fab2 = fab_floor_min(2)

    if not os.path.exists(BGA_BOARD):
        print(f"  [SKIP] BGA board absent: {BGA_BOARD}")
    else:
        for method in ('channel', 'underpad'):
            pcb = parse_kicad_pcb(BGA_BOARD)
            fp = pcb.footprints["U1"]
            t, vias, _vr, _f = _run(lambda: generate_bga_fanout(
                fp, pcb, layers=LAYERS, track_width=0.12, clearance=0.1,
                via_size=0.5, via_drill=0.3, escape_method=method))
            n, nc, viols = _via_in_pad_violations(pcb, fp, vias, 0.5, fab4)
            label = f"bga_fanout/{method}"
            fails += [f"{label}: {x}" for x in viols]
            # the 0.40 pads are above the 0.30 floor, so a forced Ø0.5 MUST shrink
            if n == 0:
                fails.append(f"{label}: placed no via-in-pad (test exercises nothing)")
            elif nc != n:
                fails.append(f"{label}: only {nc}/{n} via-in-pads clamped below 0.5")
            elif verbose:
                print(f"  {label}: {n} via-in-pad, all clamped to fit (<=0.40)  OK")

    if not os.path.exists(QFN_BOARD):
        print(f"  [SKIP] QFN board absent: {QFN_BOARD}")
    else:
        # stub: surface fan (may place no via-in-pad); underpad with via-in-pad:
        # 2-layer floor (0.45) > 0.25 pad width, so the via is held at the floor
        # (still bulges) -- the invariant tolerates exactly the floor, no more.
        cases = [
            ("qfn_fanout/stub", dict(escape_method="stub", via_size=0.5, via_drill=0.3)),
            ("qfn_fanout/underpad", dict(escape_method="underpad", via_size=0.45,
                                         via_drill=0.2, allow_via_in_pad=True)),
        ]
        for label, kw in cases:
            pcb = parse_kicad_pcb(QFN_BOARD)
            fp = pcb.footprints["U1"]
            t, vias, _d = _run(lambda: generate_qfn_fanout(
                fp, pcb, net_filter=["DP1*"], layer="F.Cu", track_width=0.1,
                clearance=0.15, grid_step=0.05, **kw))
            n, _nc, viols = _via_in_pad_violations(pcb, fp, vias, kw['via_size'], fab2)
            fails += [f"{label}: {x}" for x in viols]
            if verbose and not viols:
                print(f"  {label}: {n} via-in-pad, none bulge past max(pad,floor)  OK")
    return fails


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    print("=== clamp_via_to_pad math ===")
    fails = test_clamp_math(args.verbose)
    print("=== fanout methods clamp via-in-pad at placement ===")
    fails += test_methods(args.verbose)

    if fails:
        print("\nFAIL:\n  " + "\n  ".join(fails))
        return 1
    print("\nPASS: via-in-pad clamped to fit its pad (down to the fab floor) "
          "across channel / under-pad / qfn escape methods")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
