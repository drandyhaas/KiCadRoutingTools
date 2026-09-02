#!/usr/bin/env python3
"""Issue #619 -- the QFN under-pad (via-drop) escape emitted its pad->via
bridging stub without ever testing it against PRE-EXISTING copper belonging to
another net in the SAME fanout call.

MECHANISM. `_underpad_via_escape` builds its obstacle map with
`build_base_obstacle_map(..., nets_to_route=list(fanned_nets))`, and
`nets_to_route` ERASES every piece of copper on those nets -- segments
(`obstacle_map.py:216`), vias (`:299`) and pads (`:312`). Right for a net's OWN
copper; wrong for every OTHER net being escaped beside it. The candidate VIA
still meets that copper, because `via_clears` scans `pcb_data` directly -- but
it tests only the via CENTRE. The STUB's one channel to the input board is
`check_line_clearance` on the holed map, so stubs shipped straight through the
CENTRES of foreign vias. The SURFACE fan has a geometric backstop for exactly
this (`_fanned_existing_segs` / `_fanned_existing_vias`, issue #257); the
under-pad path returns ~200 lines earlier and shares none of it.

THE ISSUE UNDERCOUNTS ITSELF. It names the via half only. Measured over every
footprint with >=4 pads on all 22 tracked boards (408 of them, no package-name
filter -- `tests/sweep_619_erased_copper.py`), the emitted-stub contacts with
erased copper are via=20, seg=75, pad=49. The segment half is the largest.

WHAT IS PINNED HERE, and what each check is FOR:

1. THE ERASURE ITSELF, asserted directly rather than through an outcome. The
   same obstacle map built with `nets_to_route=[A, B]` reports the stub line
   CLEAR while `nets_to_route=[A]` reports the same line BLOCKED. If that ever
   stops being true the bug is gone for a reason this file does not model, and
   every other check here is testing nothing.

2. INJECTION + COUNTER-GUARD. `kicad_files/tigard.kicad_pcb` is the only
   fixture with ZERO board vias AND ZERO board segments, so an injected
   obstacle is the only one and the baseline cannot be contaminated. Inject a
   via on net B onto net A's stub, re-run with BOTH nets in the filter (both
   must be fanned or the map would not erase it), and assert the exact emitted
   tuple -- not merely "no stub inside the floor", which any change that
   dropped every pad would satisfy vacuously.

3. NEGATIVE CONTROL. The same via injected OFF the escape ray, just outside the
   floor, must NOT block anything. This is the only check with teeth against an
   over-strict gate: multiply the floor by 5 and every other check in this file
   still passes.

4. THE GATE IS LIVE WHERE IT CLAIMS TO BE. A no-op guard on a footprint whose
   erased set is EMPTY proves nothing -- the predicate is a constant True
   there. Liveness is read from `qfn_fanout.LAST_ERASED_SETS`, published by the
   engine, NEVER re-derived: `fanned_nets` comes from `pad_infos`, which drops
   net 0, `unconnected-*`, net-filter misses and `center` pads, so the obvious
   proxy `{p.net_id for p in footprint.pads}` disagrees with it and can report
   a dead loop as live.

5. THE THREE HALVES ARE INDEPENDENT AND NOT ADDITIVE. Each is pinned alone via
   KICAD_QFN_UNDERPAD_ERASED_GATE. Neither the via half nor the segment half
   alone reaches zero on U2, and each moves the OTHER category without testing
   it -- withdrawing an escape changes which offsets the remaining pads take.
   A single tally per half would credit the wrong mechanism.

6. THE MEASURED BOARDS, with the cost pinned as well as the fix. The tallies
   are deliberately brittle change-detectors, in this repo's house style (see
   `test_qfn_underpad.py`'s own 18/22/20 pin): any future change to the escape
   ladder fails them and forces a re-measurement rather than silent drift.

7. LAYER SCOPE, as a decision pin. The via half is layer-BLIND because
   `check_drc.check_via_segment_overlap` grades a via against segments on every
   copper layer regardless of `via.layers`; the segment half is layer-FILTERED
   because segments really are single-layer. It filters on `footprint.layer`
   (where the stub lands, #195) and NOT on `layer`, and the two are not
   interchangeable: on U2 they find 25 and 17 pairs and the sets are disjoint.

8. THE PAD HALF'S FOUR EXCLUSIONS. NPTH (no copper), the `*.Cu` / `F&B.Cu`
   wildcards, per-pad `local_clearance`, and net ties. Each is check_drc's own
   rule or KiCad's; without them the pad half phantom-rejects. The surface
   fan's pad loop makes none of them, so these are not inherited -- they are
   the reason this half is safe to ship.

MUTATION EVIDENCE (`python3 tests/mutate_619.py`, from the run -- never edited
to match a prediction). 14 rows, 12 KILLED, 2 SURVIVED, 0 broken:

    over-strict-floor-x5                      KILLED    (the negative control)
    via-floor-uses-drill-not-size             KILLED
    drop-the-zero-stub-guard                  KILLED
    via-half-drops-the-own-net-skip           KILLED
    seg-half-drops-the-own-net-skip           KILLED
    pad-half-drops-the-own-net-skip           KILLED
    seg-half-filters-the-ESCAPE-layer         KILLED
    via-half-disabled                         KILLED
    seg-half-disabled                         KILLED
    pad-half-disabled                         KILLED
    unknown-knob-value-means-OFF              KILLED
    pad-half-ignores-local-clearance          KILLED
    pad-half-includes-NPTH                    SURVIVED (declared)
    stub-clearance-ignores-the-dru-layer-map  SURVIVED (declared)

FOUR of those kills exist only because the battery found them SURVIVING first,
and every one was a check that read as though it covered the thing it named:

  * the zero-stub guard was argued for in a commit message and tested nowhere;
  * check 7 asserted `check_drc`'s layer behaviour and the fixture's layer, but
    never the ENGINE's choice between them -- every board in the sweep runs
    escape layer == footprint.layer, so the distinction was invisible and
    swapping `footprint.layer` for `layer` changed nothing. It needed a
    CROSS-LAYER run (U2 mounted F.Cu, escaped to B.Cu) to become a check;
  * no pinned geometry sat between the drill-derived and copper-derived floors;
  * the knob's fail-safe direction (an unknown value means ALL, never OFF) was
    asserted only by reading the code.

The two declared survivors are real test holes, named rather than hidden: no
tracked board places a netted NPTH within stub reach, and #770 records that no
board this repo ingests carries a `.kicad_dru` layer rule, so `_stub_clr` and
`clearance` are equal on every fixture. Both are asserted against check_drc's
helpers directly (check 8) instead of pretending a board covers them.

    python3 tests/test_619_underpad_stub_vs_erased_copper.py
"""
import contextlib
import io
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # #522
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # #522

from kicad_parser import parse_kicad_pcb, Via, Segment
import qfn_fanout
from qfn_fanout import generate_qfn_fanout
import env_knobs

TIGARD = os.path.join(ROOT, "kicad_files", "tigard.kicad_pcb")
U2BOARD = os.path.join(ROOT, "kicad_files", "routed_output.kicad_pcb")
ORANGE = os.path.join(ROOT, "kicad_files", "orangecrab_ext_pll.kicad_pcb")
RP2350 = os.path.join(ROOT, "kicad_files", "rp2350_fpga_eensy_prePlane.kicad_pcb")
WATCHY = os.path.join(ROOT, "kicad_files", "watchy.kicad_pcb")
LVDS = os.path.join(ROOT, "kicad_files", "lvds_converter_dualclk_gnd.kicad_pcb")

TW, CL, VS, VD, GRID = 0.1, 0.1, 0.45, 0.25, 0.05
FLOOR = VS / 2 + TW / 2 + CL           # 0.3750 mm, the injected via's floor

CHECKS = []


def check(name, ok, detail=""):
    CHECKS.append((name, bool(ok)))
    print(("  ok   " if ok else "  FAIL ") + name + (f"  [{detail}]" if detail else ""))


def _gate(value):
    """Run the next fanout with only these halves of the gate live."""
    os.environ['KICAD_QFN_UNDERPAD_ERASED_GATE'] = value
    env_knobs.refresh()


def _fanout(pcb, ref, net_filter=None, layer=None, vip=False):
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        return generate_qfn_fanout(
            pcb.footprints[ref], pcb, net_filter=net_filter,
            layer=layer or pcb.footprints[ref].layer,
            track_width=TW, clearance=CL, grid_step=GRID,
            escape_method="underpad", via_size=VS, via_drill=VD,
            allow_via_in_pad=vip)


def _erased(pcb, ref):
    """The sets the ENGINE erased on the run that just happened."""
    return getattr(qfn_fanout, 'LAST_ERASED_SETS', {})


def _fanned(pcb, ref):
    """The net-id set `nets_to_route` erased, from the engine where it can be.

    On a build that does NOT publish `LAST_ERASED_SETS` -- i.e. the RED arm,
    with this fix reverted -- fall back to the footprint's own nets. That
    fallback is a SUPERSET of `fanned_nets` (it keeps net 0, `unconnected-*`
    and `center` pads that `pad_infos` drops), so it can only over-count. That
    is the right direction for a RED arm, where the expectation is zero, and it
    is never used on the GREEN arm. Without this the whole file dies on an
    AttributeError, which reports a BROKEN TEST as though it were a satisfied
    guard -- the exact failure mode CLAUDE.md's `run_utils.check` exists to
    prevent.
    """
    got = getattr(qfn_fanout, 'LAST_ERASED_SETS', {}).get('nets')
    if got is not None:
        return got
    return {p.net_id for p in pcb.footprints[ref].pads if p.net_id}


def _pairs(pcb, tracks, ref=None):
    """Emitted stubs inside the floor of erased copper on a DIFFERENT net.

    Graded with check_drc's own predicates, so this cannot drift away from what
    the shipped grader flags.
    """
    from check_drc import check_via_segment_overlap, check_pad_segment_overlap
    from geometry_utils import segment_to_segment_distance
    fanned = _fanned(pcb, ref) if ref else (
        getattr(qfn_fanout, 'LAST_ERASED_SETS', {}).get('nets') or set())
    cu = list(pcb.board_info.copper_layers or [])
    nv = ns = npd = 0
    for t in tracks:
        seg = Segment(start_x=t['start'][0], start_y=t['start'][1],
                      end_x=t['end'][0], end_y=t['end'][1],
                      width=t['width'], layer=t['layer'], net_id=t['net_id'])
        for v in pcb.vias:
            if v.net_id in fanned and v.net_id != t['net_id'] \
                    and check_via_segment_overlap(v, seg, CL)[0]:
                nv += 1
        for s in pcb.segments:
            if s.net_id in fanned and s.net_id != t['net_id'] \
                    and s.layer == t['layer'] \
                    and segment_to_segment_distance(
                        seg.start_x, seg.start_y, seg.end_x, seg.end_y,
                        s.start_x, s.start_y, s.end_x, s.end_y) \
                    < s.width / 2 + seg.width / 2 + CL - 1e-6:
                ns += 1
        for plist in pcb.pads_by_net.values():
            for p in plist:
                if p.net_id in fanned and p.net_id != t['net_id'] \
                        and check_pad_segment_overlap(p, seg, CL, cu)[0]:
                    npd += 1
    return nv, ns, npd


# ---------------------------------------------------------------- 1, 2, 3 ---
def test_erasure_injection_and_control():
    print("\n1-3. the erasure itself, an injected blocker, and the control "
          "(tigard U3, QFN-64)")
    _gate('all')
    pcb = parse_kicad_pcb(TIGARD)
    check("tigard carries ZERO vias and ZERO segments -- an injected obstacle "
          "is the only one, so the baseline cannot be contaminated",
          len(pcb.vias) == 0 and len(pcb.segments) == 0,
          f"{len(pcb.vias)} vias, {len(pcb.segments)} segments")

    a, b = "/BD0", "/BD1"
    nid_a = next(n for n, net in pcb.nets.items() if net.name == a)
    nid_b = next(n for n, net in pcb.nets.items() if net.name == b)

    tracks, vias, dropped = _fanout(pcb, "U3", [a, b])
    check("the engine publishes LAST_ERASED_SETS, so every check below grades "
          "against the set `nets_to_route` actually erased rather than a proxy "
          "for it (absent = this file falls back to a superset and says so, "
          "instead of dying on an AttributeError and reporting a broken test "
          "as a satisfied guard)",
          getattr(qfn_fanout, 'LAST_ERASED_SETS', {}).get('nets') is not None)
    check("baseline: both nets escape, nothing dropped",
          (len(tracks), len(vias), sorted(dropped)) == (2, 2, []),
          f"{len(tracks)}/{len(vias)}/{sorted(dropped)}")
    a_stub = [t for t in tracks if t['net_id'] == nid_a]
    if not a_stub:
        check("net A emitted a stub to inject onto", False)
        return
    (ax0, ay0), (ax1, ay1) = a_stub[0]['start'], a_stub[0]['end']
    ix, iy = (ax0 + ax1) / 2.0, (ay0 + ay1) / 2.0

    # -- 1. THE MECHANISM. Same map, same line, opposite answers.
    from obstacle_map import (build_base_obstacle_map, build_layer_map,
                              check_line_clearance)
    from routing_config import GridRouteConfig
    p2 = parse_kicad_pcb(TIGARD)
    p2.vias.append(Via(x=ix, y=iy, size=VS, drill=0.2,
                       layers=["F.Cu", "B.Cu"], net_id=nid_b))
    cfg = GridRouteConfig(layers=list(p2.board_info.copper_layers),
                          track_width=TW, clearance=CL)
    li = build_layer_map(cfg.layers)["F.Cu"]
    m_ab = build_base_obstacle_map(p2, cfg, nets_to_route=[nid_a, nid_b],
                                   extra_clearance=TW / 2)
    m_a = build_base_obstacle_map(p2, cfg, nets_to_route=[nid_a],
                                  extra_clearance=TW / 2)
    check("MECHANISM: the map the escape builds (nets_to_route=[A,B]) reports "
          "net A's stub line CLEAR -- the injected via was erased from it",
          check_line_clearance(m_ab, ax0, ay0, ax1, ay1, li, cfg) is True)
    check("MECHANISM: the SAME line on a map that keeps net B "
          "(nets_to_route=[A]) is BLOCKED -- so the erasure, not the geometry, "
          "is what hid it",
          check_line_clearance(m_a, ax0, ay0, ax1, ay1, li, cfg) is False)

    # -- 2. OUTCOME + counter-guard.
    t2, v2, d2 = _fanout(p2, "U3", [a, b])
    nv, ns, npd = _pairs(p2, t2, "U3")
    check("no emitted stub is inside the injected via's clearance floor",
          (nv, ns, npd) == (0, 0, 0), f"via/seg/pad = {nv}/{ns}/{npd}")
    check(f"counter-guard: net A ({a}) is DROPPED, not silently re-routed -- "
          f"the offset ladder only walks FURTHER OUT along the same escape "
          f"ray, so a blocker on that ray blocks every remaining candidate",
          a in d2, f"dropped={sorted(d2)}")
    check("counter-guard: exact tuple (1 track, 1 via, dropped=['/BD0']) -- a "
          "change that dropped EVERY pad would pass the violation check above",
          (len(t2), len(v2), sorted(d2)) == (1, 1, [a]),
          f"{len(t2)}/{len(v2)}/{sorted(d2)}")
    check("counter-guard: the surviving track belongs to net B",
          len(t2) == 1 and t2[0]['net_id'] == nid_b)

    # -- 3. NEGATIVE CONTROL: outside the floor must not block.
    dx, dy = ax1 - ax0, ay1 - ay0
    L = math.hypot(dx, dy)
    nx, ny = ix + (-dy / L) * 0.40, iy + (dx / L) * 0.40
    p3 = parse_kicad_pcb(TIGARD)
    p3.vias.append(Via(x=nx, y=ny, size=VS, drill=0.2,
                       layers=["F.Cu", "B.Cu"], net_id=nid_b))
    from geometry_utils import point_to_segment_distance as _p2s
    d_ctrl = _p2s(nx, ny, ax0, ay0, ax1, ay1)
    check(f"negative control sits in the band an over-strict floor would catch "
          f"({FLOOR:.4f} <= d < {5 * FLOOR:.4f})",
          FLOOR <= d_ctrl < 5 * FLOOR, f"d={d_ctrl:.4f}")
    t3, v3, d3 = _fanout(p3, "U3", [a, b])
    check("NEGATIVE CONTROL: a via OUTSIDE the floor blocks nothing -- the "
          "(2, 2, []) baseline survives. Multiplying the floor by 5 fails "
          "HERE and nowhere else in this file",
          (len(t3), len(v3), sorted(d3)) == (2, 2, []),
          f"{len(t3)}/{len(v3)}/{sorted(d3)}")

    # -- 3b. THE FLOOR IS BUILT FROM via.SIZE, NOT via.DRILL. The two differ
    #    by (size - drill)/2 = 0.125 mm on this injected via, so a blocker
    #    placed BETWEEN the two floors is caught by the shipped code and
    #    missed by a drill-based one. Found by tests/mutate_619.py: without
    #    this, `v.size / 2` -> `v.drill / 2` SURVIVED the whole file.
    f_size = VS / 2 + TW / 2 + CL            # 0.3750
    f_drill = 0.2 / 2 + TW / 2 + CL          # 0.2500 (injected drill = 0.2)
    band = (f_size + f_drill) / 2.0          # 0.3125, strictly between them
    bx, by = ix + (-dy / L) * band, iy + (dx / L) * band
    p4 = parse_kicad_pcb(TIGARD)
    p4.vias.append(Via(x=bx, y=by, size=VS, drill=0.2,
                       layers=["F.Cu", "B.Cu"], net_id=nid_b))
    t4, v4, d4 = _fanout(p4, "U3", [a, b])
    check(f"the floor is the via's COPPER radius, not its drill radius: a "
          f"blocker at d={band:.4f}, between the drill floor {f_drill:.4f} and "
          f"the copper floor {f_size:.4f}, DOES block -- a drill-based floor "
          f"would wave it through",
          a in d4, f"dropped={sorted(d4)}")

    # -- 3c. AN UNRECOGNISED KNOB VALUE MEANS ALL, NEVER OFF. A typo in a
    #    harness must not silently ship the bug back. Same injected board as
    #    check 2, so the expected outcome is exactly check 2's.
    _gate('bogus-not-a-real-arm')
    t5, v5, d5 = _fanout(p2, "U3", [a, b])
    check("an unrecognised gate value behaves as 'all', not as 'off' -- a "
          "typo in an A/B harness cannot silently disable the gate",
          (len(t5), len(v5), sorted(d5)) == (1, 1, [a]),
          f"{len(t5)}/{len(v5)}/{sorted(d5)}")
    _gate('off')
    t6, v6, d6 = _fanout(p2, "U3", [a, b])
    check("...and 'off' really does disable it, so the check above is a real "
          "discrimination and not a constant",
          (len(t6), len(v6), sorted(d6)) == (2, 2, []),
          f"{len(t6)}/{len(v6)}/{sorted(d6)}")
    _gate('all')

    # -- 3d. THE ZERO-LENGTH STUB EMITS NO COPPER, SO IT IS NOT TESTED.
    #    Inject a via on net A's OWN net at net A's pad centre: the #479 reuse
    #    path bridges to it with zero length, so the commit loop emits no track
    #    (:604). Then put a net B via within the floor of that same point.
    #    Without the guard the degenerate point-"segment" is graded against it
    #    and the reuse is refused for copper that is never emitted. Found by
    #    mutate_619: without this, removing the guard SURVIVED.
    pa = [p for p in pcb.footprints["U3"].pads if p.net_id == nid_a][0]
    p7 = parse_kicad_pcb(TIGARD)
    p7.vias.append(Via(x=pa.global_x, y=pa.global_y, size=VS, drill=0.2,
                       layers=["F.Cu", "B.Cu"], net_id=nid_a))
    p7.vias.append(Via(x=pa.global_x + 0.20, y=pa.global_y, size=VS, drill=0.2,
                       layers=["F.Cu", "B.Cu"], net_id=nid_b))
    t7, v7, d7 = _fanout(p7, "U3", [a, b])
    check("a ZERO-LENGTH stub (via reused at the pad centre) emits no track, "
          "so a foreign via beside it must not reject the reuse -- net A keeps "
          "its escape and emits no new via",
          a not in d7, f"dropped={sorted(d7)}")


# -------------------------------------------------------------------- 4 -----
def test_gate_live_but_rejects_nothing():
    print("\n4. the gate is LIVE and still rejects nothing (byte-identical)")
    _gate('all')
    for board, ref, expect in NOOP_ROWS:
        pcb = parse_kicad_pcb(board)
        tracks, vias, dropped = _fanout(pcb, ref)
        e = _erased(pcb, ref)
        live = (e.get('vias', 0), e.get('segs', 0), e.get('pads', 0))
        name = os.path.basename(board).replace('.kicad_pcb', '')
        check(f"{name} {ref}: the erased set is NON-EMPTY, so the predicate "
              f"really runs here (a board with none makes it a constant True "
              f"and guards nothing)",
              any(live), f"erased v/s/p = {live[0]}/{live[1]}/{live[2]}")
        got = (len(tracks), len(vias), len(dropped))
        check(f"{name} {ref}: escape tally unchanged at {expect} -- the gate "
              f"does not over-reject",
              got == expect, f"got {got}")


# -------------------------------------------------------------------- 5 -----
def test_halves_are_independent_and_not_additive():
    print("\n5. each half alone, on routed_output U2 -- NOT additive")
    for arm, expect, pairs in ARM_ROWS:
        _gate(arm)
        pcb = parse_kicad_pcb(U2BOARD)
        tracks, vias, dropped = _fanout(pcb, "U2")
        got = (len(tracks), len(vias), len(dropped))
        gp = _pairs(pcb, tracks, "U2")
        check(f"arm={arm:8}: tally {expect}", got == expect, f"got {got}")
        check(f"arm={arm:8}: residual via/seg/pad = "
              f"{pairs[0]}/{pairs[1]}/{pairs[2]}",
              gp == pairs, f"got {gp[0]}/{gp[1]}/{gp[2]}")
    check("NOT ADDITIVE: the via half alone leaves segment contacts and the "
          "segment half alone leaves via contacts -- neither is the whole fix, "
          "and each moves the other's count without testing it",
          ARM_ROWS[1][2][1] > 0 and ARM_ROWS[2][2][0] > 0,
          f"via-arm seg={ARM_ROWS[1][2][1]}, seg-arm via={ARM_ROWS[2][2][0]}")


# -------------------------------------------------------------------- 6 -----
def test_measured_boards():
    print("\n6. the measured boards: the fix AND its cost")
    _gate('all')
    for board, ref, expect, pre in BOARD_ROWS:
        pcb = parse_kicad_pcb(board)
        tracks, vias, dropped = _fanout(pcb, ref)
        got = (len(tracks), len(vias), len(dropped))
        gp = _pairs(pcb, tracks, ref)
        name = os.path.basename(board).replace('.kicad_pcb', '')
        check(f"{name} {ref}: no emitted stub sits inside erased copper's "
              f"floor (was via/seg/pad {pre[0]}/{pre[1]}/{pre[2]} pre-fix)",
              gp == (0, 0, 0), f"got {gp[0]}/{gp[1]}/{gp[2]}")
        check(f"{name} {ref}: escape tally pinned at {expect} -- this is the "
              f"COST, pinned so it cannot drift silently",
              got == expect, f"got {got}")


# -------------------------------------------------------------------- 7 -----
def test_layer_scope_decision():
    print("\n7. layer scope: blind for vias, filtered for segments")
    from check_drc import check_via_segment_overlap
    buried = Via(x=10.0, y=10.0, size=VS, drill=VD,
                 layers=["In1.Cu", "In2.Cu"], net_id=1)
    seg = Segment(start_x=9.0, start_y=10.0, end_x=11.0, end_y=10.0,
                  width=TW, layer="F.Cu", net_id=2)
    hit, _ = check_via_segment_overlap(buried, seg, CL)
    check("check_drc grades a via against segments on EVERY copper layer "
          "regardless of via.layers, so a layer-FILTERED via backstop would "
          "ship copper this repo's own grader flags -- the via half is "
          "deliberately layer-blind, and this fails the day that changes",
          hit is True)
    # And the segment half's filter is on the STUB's layer, not the escape
    # layer. Every board in the sweep runs escape layer == footprint.layer, so
    # the distinction is INVISIBLE there -- mutate_619 found that swapping
    # `footprint.layer` for `layer` SURVIVED the whole file. It needs a
    # CROSS-LAYER configuration to be a real check: U2 is an F.Cu part escaped
    # to B.Cu, which is what the under-pad method exists for.
    _gate('all')
    pcb = parse_kicad_pcb(U2BOARD)
    fp = pcb.footprints["U2"]
    check("the fixture is genuinely cross-layer: U2 mounts on F.Cu and is "
          "escaped to B.Cu, so the stub's layer and the escape layer differ",
          fp.layer == "F.Cu")
    t_x, v_x, d_x = _fanout(pcb, "U2", layer="B.Cu")
    e = _erased(pcb, "U2")
    check("the stub's copper layer is footprint.layer, not the escape layer "
          "(#195: putting it on the escape layer would float it above the pad)",
          e.get('layer') == "F.Cu", str(e.get('layer')))
    check("cross-layer tally pinned at 15 tracks / 13 vias / 27 dropped. "
          "Filtering erased segments by the ESCAPE layer instead of the stub's "
          "layer selects a DIFFERENT set (25 vs 17 pairs on this board) and "
          "moves this number -- which is what makes this a real check",
          (len(t_x), len(v_x), len(d_x)) == (15, 13, 27),
          f"got {(len(t_x), len(v_x), len(d_x))}")


# -------------------------------------------------------------------- 8 -----
def test_pad_half_exclusions():
    print("\n8. the pad half's four exclusions")
    from check_drc import _pad_has_no_copper, pad_copper_layers
    from kicad_parser import Pad

    def _pad(num, layers, drill, ptype, size=1.0):
        return Pad(component_ref="X1", pad_number=num,
                   global_x=0.0, global_y=0.0, local_x=0.0, local_y=0.0,
                   size_x=size, size_y=size, shape="circle", layers=layers,
                   net_id=7, net_name="GND", drill=drill, pad_type=ptype)

    npth = _pad("H1", ["*.Cu"], 2.0, "np_thru_hole", size=2.0)
    check("NPTH is excluded: KiCad lists *.Cu on it for hole keep-out but an "
          "np_thru_hole pad carries no ring, and check_drc skips it for "
          "PAD-SEGMENT (#260) -- including it would phantom-reject",
          _pad_has_no_copper(npth) is True)

    th = _pad("1", ["*.Cu"], 0.6, "thru_hole")
    cu = ["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"]
    check("the *.Cu wildcard expands: a bare `layer in pad.layers` would miss "
          "a through-hole barrel entirely, and the pcbnew parse path emits "
          "that spelling even where the text parser does not (#722)",
          "F.Cu" in pad_copper_layers(th, cu) and
          "In1.Cu" in pad_copper_layers(th, cu),
          str(sorted(pad_copper_layers(th, cu))))

    fb = _pad("2", ["F&B.Cu"], 0.0, "smd")
    got = pad_copper_layers(fb, cu)
    check("...and so does F&B.Cu, to exactly front and back",
          got == {"F.Cu", "B.Cu"}, str(sorted(got)))

    # local_clearance, tested BEHAVIOURALLY and PAIRED. An arithmetic assertion
    # that max(cl, lc) > cl would pass without the engine reading the field at
    # all. Inject a pad on net B beside net A's stub, far enough out that the
    # default floor does NOT reach it, and flip only `local_clearance`:
    #   lc = 0.0  -> stub survives, baseline (2, 2, []) intact
    #   lc = 0.5  -> stub withdrawn, net A dropped
    # Distances are chosen so `via_clears`' own pad loop cannot be what moved:
    # the pad sits at the stub's MIDPOINT, 0.673 mm from the via centre against
    # that loop's 0.425 mm reach.
    _gate('all')
    a, b = "/BD0", "/BD1"
    base = parse_kicad_pcb(TIGARD)
    nid_a = next(n for n, net in base.nets.items() if net.name == a)
    nid_b = next(n for n, net in base.nets.items() if net.name == b)
    t0, _, _ = _fanout(base, "U3", [a, b])
    astub = [t for t in t0 if t['net_id'] == nid_a][0]
    mx = (astub['start'][0] + astub['end'][0]) / 2.0
    my = (astub['start'][1] + astub['end'][1]) / 2.0

    res = {}
    for lc in (0.0, 0.5):
        p = parse_kicad_pcb(TIGARD)
        inj = _pad("LC", ["F.Cu"], 0.0, "smd", size=0.2)
        inj.component_ref, inj.net_id, inj.net_name = "INJ", nid_b, b
        inj.global_x, inj.global_y = mx, my + 0.45
        inj.local_clearance = lc
        p.pads_by_net.setdefault(nid_b, []).append(inj)
        res[lc] = _fanout(p, "U3", [a, b])
    t_lo, v_lo, d_lo = res[0.0]
    t_hi, v_hi, d_hi = res[0.5]
    check("local_clearance control: at lc=0.0 the injected pad is OUTSIDE the "
          "floor and blocks nothing -- the (2, 2, []) baseline survives",
          (len(t_lo), len(v_lo), sorted(d_lo)) == (2, 2, []),
          f"{len(t_lo)}/{len(v_lo)}/{sorted(d_lo)}")
    check("local_clearance is READ, per pad: the SAME pad at lc=0.5 raises the "
          "floor past the stub and net A is dropped. The surface fan's hoisted "
          "`margin` structurally cannot do this",
          a in d_hi and (len(t_hi), len(v_hi)) != (2, 2),
          f"{len(t_hi)}/{len(v_hi)}/{sorted(d_hi)}")

    pcb = parse_kicad_pcb(TIGARD)
    check("PCBData exposes the net-tie exemption the pad half honours "
          "(_seg_hits_pad is net-blind, and obstacle_map's own tie lift only "
          "fires when exactly ONE net is being routed, so the under-pad path "
          "never receives it)",
          callable(getattr(pcb, 'net_tie_exempt_pad_ids', None)))


# --------------------------------------------------------------------------
# Measured rows. Every number here was produced on this branch by
# `tests/sweep_619_erased_copper.py` over all 408 footprints with >=4 pads on
# the 22 tracked boards; none is carried over from PR #645, whose numbers were
# taken on an engine `98bd0fae` has since rewritten in exactly these two gates.
#
# Corpus totals, gate off -> all three halves:
#     residual stub-vs-erased pairs   via/seg/pad   20/75/49  ->  0/0/0
#     byte-identical emitted geometry               391 of 408
#     footprints changed                            17
#     escapes withdrawn                             49  (973 -> 1022 dropped)
# --------------------------------------------------------------------------

# Gate LIVE (non-empty erased sets) and output byte-identical. These are the
# guard with teeth: a footprint whose erased set is EMPTY runs a predicate that
# is a constant True and proves nothing.
NOOP_ROWS = [
    (U2BOARD, "U3", (19, 19, 59)),      # erased 209 vias / 938 segs / 486 pads
    (U2BOARD, "IC1", (33, 33, 24)),     # erased  72 / 382 / 329
    (LVDS, "IC2", (12, 11, 0)),         # erased   9 /  97 /  47, nothing dropped
]

# Each half alone on routed_output U2, and the pairs it leaves behind.
# (arm, tally t/v/d, residual pairs via/seg/pad)
ARM_ROWS = [
    ('off',     (27, 26, 15), (7, 32, 0)),
    ('via',     (20, 19, 22), (0, 13, 0)),   # via contacts gone, 13 seg remain
    ('seg',     (16, 15, 26), (2, 0, 0)),    # seg contacts gone,  2 via remain
    ('all',     (14, 13, 28), (0, 0, 0)),
]

# (board, ref, post-fix tally t/v/d, pre-fix pairs via/seg/pad)
# The last two changed ONLY because of the pad half -- their erased via and
# segment sets are both empty, so they are the evidence that half earns its
# place. It was pre-registered as a withdrawal candidate on a 5-board sample
# that showed zero yield; the full 408-footprint sweep found 49 pairs on 9
# footprints across 6 boards, so it ships.
BOARD_ROWS = [
    (U2BOARD, "U2", (14, 13, 28), (7, 32, 0)),
    (ORANGE, "U7", (2, 0, 4), (3, 0, 4)),
    (ORANGE, "U3", (33, 33, 62), (1, 14, 11)),
    (RP2350, "U6", (6, 3, 41), (2, 11, 1)),
    (TIGARD, "U3", (43, 43, 5), (0, 0, 4)),     # pad half only
    (WATCHY, "J3", (12, 12, 7), (0, 0, 9)),     # pad half only
]


def run():
    for b in (TIGARD, U2BOARD, ORANGE, RP2350, WATCHY, LVDS):
        if not os.path.exists(b):
            print(f"SKIP: missing fixture {b}")
            return 77
    prev = os.environ.get('KICAD_QFN_UNDERPAD_ERASED_GATE')
    try:
        test_erasure_injection_and_control()
        if NOOP_ROWS:
            test_gate_live_but_rejects_nothing()
        if ARM_ROWS:
            test_halves_are_independent_and_not_additive()
        if BOARD_ROWS:
            test_measured_boards()
        test_layer_scope_decision()
        test_pad_half_exclusions()
    finally:
        if prev is None:
            os.environ.pop('KICAD_QFN_UNDERPAD_ERASED_GATE', None)
        else:
            os.environ['KICAD_QFN_UNDERPAD_ERASED_GATE'] = prev
        env_knobs.refresh()
    bad = [n for n, ok in CHECKS if not ok]
    print("-" * 68)
    print(f"{len(CHECKS) - len(bad)}/{len(CHECKS)} checks passed")
    if bad:
        print("FAILED: " + "; ".join(bad))
        return 1
    print("ALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(run())
