#!/usr/bin/env python3
"""Issue #284: cross-invocation --rip-existing-nets stacked-via de-dup.

The output writer copies the input file verbatim, then APPENDS the routing
results' vias. When route.py rips an already-routed net and re-routes it, the
reroute frequently lands a via-in-pad via at the EXACT position of the net's
original via. If the original is not stripped from the verbatim copy, the output
carries two same-net vias in one hole -- and the drill hole-to-hole check is
net-independent, so this is a DRC violation whether the two vias are the same
size or not. sechzig went 7 -> 555 DRC on this class; the residual after #301 was
3 stacked drill violations.

`compute_stale_input_vias` strips an in-scope net's original via when EITHER it
is superseded on the final board (size/drill-aware -- a shrunk reroute via at the
original position no longer matches) OR the writer will re-emit a via for the
same net at the same position (covers the IDENTICAL-size stack, which a
size-aware match alone would miss). A via route.py kept unchanged (in the final
board, not re-emitted) is left alone.

    python3 tests/test_rip_existing_via_dedup.py
"""
import os
import sys
from types import SimpleNamespace

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from route import compute_stale_input_vias, compute_stale_input_segments
from kicad_writer import remove_vias_from_content, remove_segments_from_content


def _via(x, y, size, drill, net=67, layers=('F.Cu', 'B.Cu')):
    return SimpleNamespace(x=x, y=y, size=size, drill=drill, net_id=net,
                           layers=list(layers))


def _seg(x1, y1, x2, y2, width=0.15, layer='In3.Cu', net=74):
    return SimpleNamespace(start_x=x1, start_y=y1, end_x=x2, end_y=y2,
                           width=width, layer=layer, net_id=net)


def _span_key(s):
    a = (round(s.start_x, 3), round(s.start_y, 3))
    b = (round(s.end_x, 3), round(s.end_y, 3))
    return (s.net_id, min(a, b), max(a, b), s.layer)


def _final_output_vias(input_vias, stale, emitted):
    """Model the writer: verbatim input minus stale, plus the emitted (appended)
    vias. Return the list of (net, x, y) via keys actually written."""
    stale_ids = {id(v) for v in stale}
    kept = [v for v in input_vias if id(v) not in stale_ids]
    keys = [(v.net_id, round(v.x, 3), round(v.y, 3)) for v in kept + emitted]
    return keys


def run():
    fails = []
    _ALL_CHECKS = []

    def check(name, cond):
        _ALL_CHECKS.append(name)
        if not cond:
            fails.append(name)

    P = (147.6, 120.81)
    scope = {67}

    # ---- Case A: reroute lands a DIFFERENT-size via at the original position ----
    origA = _via(*P, 0.35, 0.2, net=67)          # original input via
    kept_elsewhere = _via(150.0, 125.0, 0.35, 0.2, net=67)  # untouched, on final
    rerouteA = _via(*P, 0.3, 0.15, net=67)       # shrunk reroute via (emitted)
    input_vias = [origA, kept_elsewhere]
    final_vias = [rerouteA, _via(150.0, 125.0, 0.35, 0.2, net=67)]  # pcb_data
    emitted = [rerouteA]                          # results new_vias + swap vias
    orig_by_net = {67: [origA, kept_elsewhere]}

    staleA = compute_stale_input_vias(orig_by_net, scope, final_vias, emitted)
    check("diff-size: superseded original flagged stale", origA in staleA)
    check("diff-size: untouched via left alone", kept_elsewhere not in staleA)
    keysA = _final_output_vias(input_vias, staleA, emitted)
    check("diff-size: no two vias share a (net,pos) in output",
          len(keysA) == len(set(keysA)))
    check("diff-size: reroute via present, original gone",
          (67, round(P[0], 3), round(P[1], 3)) in keysA and origA not in
          [v for v in input_vias if id(v) not in {id(s) for s in staleA}])

    # ---- Case B: reroute lands an IDENTICAL via at the original position ----
    # size-aware match alone would KEEP the original (its sig is on the final
    # board) -> the re-emit rule is what prevents the identical stack.
    origB = _via(*P, 0.35, 0.2, net=67)
    rerouteB = _via(*P, 0.35, 0.2, net=67)        # identical, but a distinct object
    final_visB = [rerouteB]                        # pcb_data holds the reroute via
    orig_by_netB = {67: [origB]}
    staleB = compute_stale_input_vias(orig_by_netB, scope, final_visB, [rerouteB])
    check("identical-size: original still flagged stale (re-emit rule)",
          origB in staleB)
    keysB = _final_output_vias([origB], staleB, [rerouteB])
    check("identical-size: no stacked (net,pos) in output",
          len(keysB) == len(set(keysB)) == 1)

    # ---- Case C: untouched rip-existing via -- on final, NOT re-emitted ----
    # (e.g. a net registered as rippable but never actually ripped) must NOT be
    # stripped, or its only copy (the verbatim one) is lost.
    origC = _via(*P, 0.35, 0.2, net=67)
    staleC = compute_stale_input_vias({67: [origC]}, scope, [origC], emitted_vias=[])
    check("untouched via (final, not emitted) is kept", origC not in staleC)

    # ---- Case D: emitted via for the net sits at a DIFFERENT position ----
    origD = _via(*P, 0.35, 0.2, net=67)
    emitD = _via(160.0, 130.0, 0.3, 0.15, net=67)
    staleD = compute_stale_input_vias({67: [origD]}, scope, [origD, emitD], [emitD])
    check("kept via untouched when re-emit is at another position",
          origD not in staleD)

    # ---- End-to-end through the real writer for both stack cases ----
    def _content(size, drill):
        return (
            '(kicad_pcb\n'
            '\t(version 20250101)\n'
            f'\t(via (at 147.6 120.81) (size {size}) (drill {drill}) '
            '(layers "F.Cu" "B.Cu") (net 67) (uuid "orig"))\n'
            '\t(via (at 150 125) (size 0.35) (drill 0.2) '
            '(layers "F.Cu" "B.Cu") (net 67) (uuid "kept"))\n'
            ')')
    outA, nA = remove_vias_from_content(_content(0.35, 0.2), staleA)
    check("writer strips original (diff-size), keeps the other",
          nA == 1 and '"orig"' not in outA and '"kept"' in outA)
    outB, nB = remove_vias_from_content(_content(0.35, 0.2), staleB)
    check("writer strips original (identical-size)",
          nB == 1 and '"orig"' not in outB)

    # ---- Segment twin: a re-emitted identical span must not double-ship ----
    SEG = (152.9, 133.4, 152.9, 133.5)
    origS = _seg(*SEG, net=74)             # original input segment
    kept_seg = _seg(140.0, 100.0, 141.0, 100.0, net=74)  # untouched elsewhere
    reemitS = _seg(*SEG, net=74)           # reroute reproduces the same span
    orig_seg_by_net = {74: [origS, kept_seg]}
    final_segs = [reemitS, _seg(140.0, 100.0, 141.0, 100.0, net=74)]
    staleS = compute_stale_input_segments(orig_seg_by_net, {74}, final_segs, [reemitS])
    check("segment: re-emitted identical span flagged stale", origS in staleS)
    check("segment: untouched span left alone", kept_seg not in staleS)
    # writer view: verbatim (minus stale) + appended re-emit -> one copy per span
    stale_seg_ids = {id(s) for s in staleS}
    written = [s for s in [origS, kept_seg] if id(s) not in stale_seg_ids] + [reemitS]
    keysS = [_span_key(s) for s in written]
    check("segment: no span shipped twice", len(keysS) == len(set(keysS)))

    # end-to-end through the real segment writer
    seg_content = (
        '(kicad_pcb\n\t(version 20250101)\n'
        '\t(segment (start 152.9 133.4) (end 152.9 133.5) (width 0.15) '
        '(layer "In3.Cu") (net 74) (uuid "s_orig"))\n'
        '\t(segment (start 140 100) (end 141 100) (width 0.15) '
        '(layer "In3.Cu") (net 74) (uuid "s_kept"))\n)')
    outS, nS = remove_segments_from_content(seg_content, staleS)
    check("writer strips the re-emitted original segment, keeps the untouched one",
          nS == 1 and '"s_orig"' not in outS and '"s_kept"' in outS)

    # ---- Segment: DIFFERENT span (rerouted away) still stripped as stale ----
    origS2 = _seg(152.9, 133.4, 152.9, 133.5, net=74)
    reemitS2 = _seg(153.5, 133.4, 153.5, 133.9, net=74)  # different path
    staleS2 = compute_stale_input_segments({74: [origS2]}, {74}, [reemitS2], [reemitS2])
    check("segment: original stripped when net rerouted to a different span",
          origS2 in staleS2)

    print("=" * 60)
    if fails:
        for f in fails:
            print(f"  FAIL  {f}")
        print(f"\n{len(fails)} failure(s)")
        return 1
    print("  PASS  #284 stacked-copper de-dup: vias (diff+identical size) and "
          "segments (re-emit + rerouted-away), kept copper untouched, "
          "end-to-end writer strip")
    print(f"\n{len(_ALL_CHECKS)}/{len(_ALL_CHECKS)} checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(run())
