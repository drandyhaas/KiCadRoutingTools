#!/usr/bin/env python3
"""Issue #164: qfn_fanout --escape-method underpad drops a clearing via per pad.

Escapes a fine-pitch QFN diff pair (tigard U3, QFN-64 0.5mm, /USB_DP+/USB_DN)
by dropping a staggered through-via just past each pad instead of the surface
45-degree fan. Asserts both halves escape and the two different-net vias clear.

Uses kicad_files/tigard.kicad_pcb; skips cleanly if absent.
"""
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # #522
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # #522
from kicad_parser import parse_kicad_pcb
from qfn_fanout import generate_qfn_fanout

BOARD = os.path.join(ROOT, "kicad_files", "tigard.kicad_pcb")

VIA_SIZE, CLEARANCE = 0.45, 0.1


def main():
    if not os.path.exists(BOARD):
        print(f"SKIP: corpus board not found ({BOARD})")
        return 0

    pcb = parse_kicad_pcb(BOARD)
    u3 = pcb.footprints["U3"]
    tracks, vias, dropped = generate_qfn_fanout(
        u3, pcb, net_filter=["/USB_DP", "/USB_DN"], layer="F.Cu",
        track_width=0.1, clearance=CLEARANCE, grid_step=0.05,
        escape_method="underpad", via_size=VIA_SIZE, via_drill=0.25)

    fails = []
    if dropped:
        fails.append(f"dropped (should escape): {dropped}")
    if len(vias) != 2:
        fails.append(f"expected 2 escape vias, got {len(vias)}")
    # every via is a through-hole on the net, with a stub feeding it
    net_ids = {v["net_id"] for v in vias}
    for v in vias:
        if v["layers"] != ["F.Cu", "B.Cu"]:
            fails.append(f"via not through-hole: {v['layers']}")
    if len(net_ids) != 2:
        fails.append(f"expected both pair halves to escape, got nets {net_ids}")
    if not any(t["net_id"] in net_ids for t in tracks):
        fails.append("no stub feeding the escape vias")
    # the two different-net vias must clear (centre-to-centre >= via + clearance)
    for i in range(len(vias)):
        for j in range(i + 1, len(vias)):
            if vias[i]["net_id"] == vias[j]["net_id"]:
                continue
            d = math.hypot(vias[i]["x"] - vias[j]["x"], vias[i]["y"] - vias[j]["y"])
            if d < VIA_SIZE + CLEARANCE - 1e-6:
                fails.append(f"vias too close: {d:.3f} < {VIA_SIZE + CLEARANCE}")

    fails += _d10_self_blindness()

    if fails:
        print("FAIL: " + "; ".join(fails))
        return 1
    print(f"PASS: pair escaped as {len(vias)} staggered through-vias, "
          f"min different-net spacing OK")
    return 0


# --- D10: the candidate test must see the run's OWN emitted copper ----------
#
# `via_clears` tested a candidate against the board it was handed -- a snapshot
# taken before anything was placed -- and against the via CENTRES emitted so
# far. It never saw the STUBS the same run emitted, so it approved vias sitting
# on copper it had just laid, and those escapes came back DRC CONTACTS.
#
# Measured: U2 (QFN56) reported 39 of 46 escapes under `--escape-method underpad
# --allow-via-in-pad` and the gate rejected every one as a contact -- while a
# hostile verifier established the geometry was fine (1.4mm moat admitting a
# 0.7mm via at all 56 pads, pad_via == 0, a real 0.700mm via placeable DRC-clean
# in U2.43, true capacity 11 of 14 lanes per side).

KN = dict(via_size=0.7, via_drill=0.4, clearance=0.2, track_width=0.15,
          hole_to_hole=0.2)


def _d10_self_blindness():
    from qfn_fanout import run_output_conflict as conflict
    bad = []

    def check(name, cond):
        print(("PASS: " if cond else "FAIL: ") + name)
        if not cond:
            bad.append(name)

    # Net 1: pad at (0,0), via at (0,2) -- so a 2mm stub runs up x=0.
    stub = [(0.0, 2.0, 1, 0.0, 0.0)]

    # A net-2 via at (0,1) sits ON that stub, yet is 1.0mm from the via centre
    # -- comfortably past the 0.9mm different-net via floor (0.7+0.2). So the
    # via-to-via test PASSES it, and ONLY the stub test can reject it. This is
    # exactly the shape that shipped contacts.
    check('a via sitting on this run\'s own stub is rejected',
          conflict(0.0, 1.0, 2, stub, **KN))
    # ...and it is ONLY the stub that rejects it: with the stub removed from
    # the entry (a bare via centre, the old 3-tuple shape) the same candidate
    # is accepted. Asserting the arithmetic instead would test literals.
    check('...the SAME candidate passes when that stub is not recorded',
          not conflict(0.0, 1.0, 2, [(0.0, 2.0, 1)], **KN))

    # Same net: its own stub is not an obstacle.
    check('a SAME-net via on the same stub is allowed',
          not conflict(0.0, 1.0, 1, stub, **KN))

    # Clear of everything.
    check('a via clear of both the stub and the via is allowed',
          not conflict(3.0, 1.0, 2, stub, **KN))

    # The candidate's OWN stub must be tested too -- here the via lands clear
    # but its stub from (0.6,1.0) sweeps across the placed via at (0,2)... use
    # a pad whose stub passes the foreign via.
    check('a candidate whose own STUB crosses a placed via is rejected',
          conflict(-2.0, 2.0, 2, stub, px=2.0, py=2.0, **KN))

    # Stub-vs-stub, ISOLATED. A parallel-stubs case would be tautological: two
    # stubs close enough to graze also put their vias inside the 0.9mm
    # different-net via floor, so the via-to-via test alone would reject it and
    # the assertion would pass with the stub logic deleted. Cross them instead,
    # with a long placed stub, so every other pair is far above its floor:
    #   cand via <-> placed via  2.50mm (floor 0.90)
    #   cand via <-> placed stub 2.00mm (floor 0.625)
    #   placed via <-> cand stub 1.50mm (floor 0.625)
    # Only the stub-vs-stub distance (0) can fire.
    long_stub = [(0.0, 3.0, 1, 0.0, 0.0)]
    check('a candidate whose STUB crosses an emitted stub is rejected',
          conflict(2.0, 1.5, 2, long_stub, px=-2.0, py=1.5, **KN))
    check('...and the same crossing on its OWN net is allowed',
          not conflict(2.0, 1.5, 1, long_stub, px=-2.0, py=1.5, **KN))

    # A via-in-pad entry has a zero-length stub and must not be read as one.
    inpad = [(5.0, 5.0, 1, 5.0, 5.0)]
    check('a zero-length (via-in-pad) stub is not treated as copper',
          not conflict(5.0, 6.0, 2, inpad, **KN))

    bad += _d10_wiring()
    bad += _q2_reuse_adds_no_via()
    return bad


# --- Q2: a REUSE adds no drill and no via copper, only the stub -------------
#
# The D10 follow-up wired the reuse branch into run_output_conflict -- correctly,
# because that branch was blind to this run's own output -- but priced the
# REUSED via as a NEW one. The reuse target is an existing board via; on
# routed_output.kicad_pcb those are 0.30, and the partners they were judged
# against are also pre-existing 0.30 vias sitting at 0.400mm: legal
# (0.15 + 0.15 + 0.10) and already on the board. Priced at config via_size 0.45
# the test demanded 0.55 and rejected all 7 reuse candidates -- judging two vias
# that already exist, at a size neither has, for a spacing this run does not
# create.
#
# Measured on kicad_files/routed_output.kicad_pcb, U2, B.Cu, track/clearance
# 0.1, via 0.45/0.25, --escape-method underpad --allow-via-in-pad:
#
#     715c821 (before D10 follow-up)  28 vias / 12 dropped / 30 tracks
#     f1dd280 (the follow-up)         29 vias / 13 dropped / 31 tracks
#
# Net-(U2A-DATA_30) lost its escape and a fresh drill appeared where a reuse
# would have served -- which is what #479 audit gap 2's reuse path exists to
# prevent.

# The numbers off that board, so the geometry under test is the measured one.
REUSE_KN = dict(via_size=0.45, via_drill=0.25, clearance=0.1, track_width=0.1,
                hole_to_hole=0.2)
U2_BOARD = os.path.join(ROOT, "kicad_files", "routed_output.kicad_pcb")


def _q2_reuse_adds_no_via():
    from qfn_fanout import run_output_conflict as conflict
    bad = []

    def check(name, cond, detail=""):
        print(("PASS: " if cond else "FAIL: ") + name + (f"  [{detail}]"
                                                         if detail else ""))
        if not cond:
            bad.append(name)

    # The BOARD first, deliberately: the checks below pass `adds_via`, which
    # does not exist before the fix, so on the unfixed engine they raise rather
    # than measure. Running the counts first makes the pre-fix failure a
    # measured 29/13/31 instead of a TypeError.
    bad += _q2_reuse_end_to_end()

    # The exact rejection the verifier isolated, in board coordinates. A
    # pre-existing 0.30 via on another net at (213.5625, 105.7) fed by a stub
    # from its pad at (213.5625, 105.5); the reuse candidate is a pre-existing
    # 0.30 via at (213.5625, 106.1), bridged from the pad at (213.5625, 106.3).
    placed = [(213.5625, 105.7, 80, 213.5625, 105.5)]
    cand = (213.5625, 106.1, 107)
    pad = dict(px=213.5625, py=106.3)
    gap = 106.1 - 105.7

    check('the measured spacing is the legal one for two 0.30 vias',
          abs(gap - 0.400) < 1e-9 and 0.400 >= 0.15 + 0.15 + 0.10 - 1e-9,
          f"gap {gap:.4f}, true floor 0.40, demanded "
          f"{REUSE_KN['via_size'] + REUSE_KN['clearance']:.2f}")
    # Pricing it as a NEW via is what rejected it: this is the f1dd280 call.
    check('priced as a NEW via, the legal reuse is rejected (the regression)',
          conflict(*cand, placed, **pad, **REUSE_KN))
    # ...and suppressing the via terms -- which is what "adds no via" means --
    # accepts it. Only `adds_via` differs between these two lines, so nothing
    # else can be what changed the verdict.
    check('a reuse adds no drill and no via copper, so it is accepted',
          not conflict(*cand, placed, **pad, adds_via=False, **REUSE_KN))

    # The stub is still tested, or the branch would be blind again -- which is
    # the D10 gap this must not reopen. A reuse whose BRIDGING STUB crosses
    # another net's stub is rejected even with adds_via=False.
    crossing = [(0.0, 3.0, 1, 0.0, 0.0)]        # placed stub up x=0, y 0..3
    check('a reuse whose stub crosses an emitted stub is still rejected',
          conflict(2.0, 1.5, 2, crossing, px=-2.0, py=1.5, adds_via=False,
                   **REUSE_KN))
    # ...and its stub against a placed VIA, the other direction.
    check('a reuse whose stub runs over a placed via is still rejected',
          conflict(2.0, 3.0, 2, [(0.0, 3.0, 1, 0.0, 0.0)], px=-2.0, py=3.0,
                   adds_via=False, **REUSE_KN))
    # Same net is still exempt.
    check('a same-net reuse crossing its own stub is allowed',
          not conflict(2.0, 1.5, 1, crossing, px=-2.0, py=1.5, adds_via=False,
                       **REUSE_KN))
    # A reuse landing ON the pad emits no track at all (the commit loop skips
    # a sub-tolerance stub), so it can conflict with nothing.
    check('a reuse with no stub to emit conflicts with nothing',
          not conflict(0.0, 1.0, 2, [(0.0, 1.0, 1, 0.0, 0.0)], px=0.0, py=1.0,
                       adds_via=False, **REUSE_KN))

    # TERM 1 IN ISOLATION -- the candidate VIA against a stub this run already
    # emitted. This is the third of the three edits the reuse fix made, and it
    # was the one nothing tested: reverting ONLY the `adds_via and` guard on
    # term 1 leaves the whole suite green and U2 at 28/12/30, because term 1
    # never fires for a reuse candidate on this corpus. Untestable-by-corpus is
    # not untestable, so the geometry is constructed instead.
    #
    # Candidate reuse via at (0,0) on net 2, bridged from its pad at (0,-0.5).
    # Placed: a net-1 via far away at (0,3), fed by a stub from its pad at
    # (0.3,0) -- so that STUB passes 0.2985mm from the candidate via.
    #   term 1  cand via   <-> placed stub  0.2985  (floor 0.375)  FIRES
    #   via-via cand via   <-> placed via   3.000   (floor 0.550)  clear
    #   term 2  placed via <-> cand stub    3.000   (floor 0.375)  clear
    #   term 3  cand stub  <-> placed stub  0.2985  (floor 0.200)  clear
    # so term 1 is the ONLY thing that can decide either call.
    t1 = [(0.0, 3.0, 1, 0.3, 0.0)]
    check('term 1 fires for a NEW via whose position this run chose',
          conflict(0.0, 0.0, 2, t1, px=0.0, py=-0.5, **REUSE_KN))
    check('...and is suppressed for a REUSE, whose position it did not choose',
          not conflict(0.0, 0.0, 2, t1, px=0.0, py=-0.5, adds_via=False,
                       **REUSE_KN))
    return bad


def _q2_reuse_end_to_end():
    """The measured board, not just the pure function.

    The pure-function checks above would all pass with `adds_via` accepted and
    ignored at the call site -- the same "tested the function, left the wiring
    uncovered" hole D10 was pulled up for. So assert the counts off the board
    the regression was measured on.
    """
    import io
    import contextlib
    from qfn_fanout import generate_qfn_fanout as fanout
    bad = []

    def check(name, cond, detail=""):
        print(("PASS: " if cond else "FAIL: ") + name + (f"  [{detail}]"
                                                         if detail else ""))
        if not cond:
            bad.append(name)

    if not os.path.exists(U2_BOARD):
        print("SKIP: corpus board not found (U2 reuse counts)")
        return bad

    pcb = parse_kicad_pcb(U2_BOARD)
    with contextlib.redirect_stdout(io.StringIO()):
        tracks, vias, dropped = fanout(
            pcb.footprints["U2"], pcb, layer="B.Cu", track_width=0.1,
            clearance=0.1, grid_step=0.05, escape_method="underpad",
            via_size=0.45, via_drill=0.25, allow_via_in_pad=True)

    got = (len(vias), len(dropped), len(tracks))
    # #619 moved this pin: the escape now tests its pad->via stub against the
    # copper `nets_to_route` erased from the obstacle map (pre-existing vias,
    # tracks and pads on the OTHER nets in this same fanout call), which it
    # never did before, so escapes that were only reachable THROUGH that copper
    # are withdrawn. On this configuration 28/12/30 -> 18/22/20. The reuse
    # behaviour this section exists to pin is unchanged -- see the named
    # DATA_30 check below, which still passes.
    check('U2 underpad reuse: 18 vias / 22 dropped / 20 tracks '
          '(was 28 / 12 / 30 before the #619 erased-copper gate)',
          got == (18, 22, 20), f"got {got[0]} / {got[1]} / {got[2]}")
    # The specific net the regression cost, named -- a count can drift back
    # into place for an unrelated reason, this cannot.
    check('Net-(U2A-DATA_30) keeps its escape (it reuses an existing via)',
          'Net-(U2A-DATA_30)' not in dropped,
          f"{len(dropped)} dropped")
    return bad


def _wiring_spy(ref, net_filter, **kw):
    """Run the underpad escape with `run_output_conflict` spied on.

    Returns what the engine FED the checker, plus the engine's own per-side pad
    grouping read back off its own output (not re-derived here -- the point is
    to observe the run, not to model it).
    """
    import io
    import re
    import contextlib
    import qfn_fanout as Q

    seen = {'entries': [], 'with_pad': 0, 'calls': 0, 'max_placed': 0}
    real = Q.run_output_conflict

    def spy(vx, vy, net_id, placed, px=None, py=None, **kwargs):
        seen['calls'] += 1
        if px is not None:
            seen['with_pad'] += 1
        seen['entries'].extend(placed)
        seen['max_placed'] = max(seen['max_placed'], len(placed))
        return real(vx, vy, net_id, placed, px, py, **kwargs)

    Q.run_output_conflict = spy
    try:
        pcb = parse_kicad_pcb(BOARD)
        with contextlib.redirect_stdout(io.StringIO()) as buf:
            Q.generate_qfn_fanout(
                pcb.footprints[ref], pcb, net_filter=net_filter, layer="F.Cu",
                track_width=0.1, clearance=CLEARANCE, grid_step=0.05,
                escape_method="underpad", via_size=VIA_SIZE, via_drill=0.25,
                **kw)
    finally:
        Q.run_output_conflict = real

    seen['sides'] = {m.group(1): int(m.group(2)) for m in
                     (re.match(r'\s+(\w+): (\d+) pads$', ln)
                      for ln in buf.getvalue().splitlines()) if m}
    return seen


def _d10_wiring():
    """The pure function is not the fix -- the WIRING is.

    Testing only `run_output_conflict` leaves the three call-site edits that
    actually connect it (passing the pad into `via_clears`, and recording the
    pad origin in `placed` / `placed_global`) completely uncovered: they can
    all be reverted with every test still green. So assert the wiring itself,
    by watching what the engine feeds the checker on a real board.
    """
    bad = []

    def check(name, cond, detail=""):
        print(("PASS: " if cond else "FAIL: ") + name + (f"  [{detail}]"
                                                         if detail else ""))
        if not cond:
            bad.append(name)

    if not os.path.exists(BOARD):
        print("SKIP: corpus board not found (wiring check)")
        return bad

    seen = _wiring_spy("U3", ["/USB_DP", "/USB_DN"])
    check('the engine actually consults the in-run conflict test',
          seen['calls'] > 0)
    # If via_clears stopped forwarding the pad, this is 0 and the candidate's
    # own stub goes untested.
    check("every candidate is offered with its OWN pad (its stub)",
          seen['calls'] > 0 and seen['with_pad'] == seen['calls'])
    # If `placed` reverted to 3-tuples, the emitted stubs become invisible.
    check('recorded placements carry the pad origin, not just the via centre',
          seen['entries'] and all(len(e) == 5 for e in seen['entries']))

    # THE CROSS-SIDE HALF, which the fixture above cannot reach. D10's commit
    # message claims the fix covers stubs "within a side AND across sides via
    # placed_global" -- but `net_filter=["/USB_DP","/USB_DN"]` yields ONE side
    # (left: 2 pads), so placed_global is still empty when trial() runs and no
    # placed_global entry ever reaches the checker. Measured per-mutation
    # against the combined revert: dropping px/py from the via_clears call
    # exits 1, a 3-tuple in `trial` exits 1, and a 3-tuple in `placed_global`
    # exits 0 -- SURVIVES. An unfiltered U3 spans four sides and kills it.
    wide = _wiring_spy("U3", None)
    biggest = max(wide['sides'].values() or [0])
    check('the cross-side fixture really does span more than one side',
          len(wide['sides']) >= 2, f"sides {wide['sides']}")
    # WHY THIS BOUND IS EXACT: inside one side's trial, `placed` starts at
    # len(placed_global) and trial() itself has appended at most (that side's
    # pads - 1) entries before any given call. So with NO cross-side carry,
    # max_placed <= biggest - 1. Seeing max_placed >= biggest therefore proves
    # placed_global was non-empty when a later side ran -- it is a direct
    # assertion about placed_global, not a proxy for one.
    check('placed_global carried committed placements INTO a later side',
          wide['max_placed'] >= biggest,
          f"max placed {wide['max_placed']}, biggest side {biggest}")
    # ...and THAT is what kills mutation C: those carried entries are the only
    # ones that come from placed_global rather than from trial's own append.
    check('...and the cross-side entries carry the pad origin too',
          wide['entries'] and all(len(e) == 5 for e in wide['entries']),
          f"{sorted(set(len(e) for e in wide['entries']))}-tuples seen")
    return bad


if __name__ == "__main__":
    raise SystemExit(main())
