#!/usr/bin/env python3
"""Issue #619 -- the QFN under-pad (via-drop) escape emitted its pad->via
bridging stub without ever testing it against a PRE-EXISTING via that belongs
to another net in the SAME fanout call.

Mechanism: `_underpad_via_escape` builds its obstacle map with
`build_base_obstacle_map(..., nets_to_route=list(fanned_nets))`, which ERASES
every piece of copper on a net being escaped -- including vias already on the
board for those nets (a re-run, a post-route fanout, a second part sharing the
bus). The candidate via position is still checked geometrically by
`via_clears` (it scans `pcb_data.vias` directly), but the STUB was only checked
through the obstacle map, so it could be routed straight across -- or straight
THROUGH -- a foreign via. The surface fan has a geometric backstop for exactly
this (`_fanned_existing_vias`, issue #257); the under-pad path had none.

Seven checks:

1. INJECTION REGRESSION (the pin) + NEGATIVE CONTROL. `kicad_files/tigard.kicad_pcb`
   U3 is a QFN-64 on a board with ZERO vias, so the baseline is uncontaminated.
   Learn where net A's escape stub lands, re-parse, drop a via on net B onto
   that path, and re-run with BOTH nets in the filter (both must be in
   `fanned_nets` or the map would not erase the via). Assert the MECHANISM
   directly (the same map built with `nets_to_route=[A, B]` reports the stub
   line CLEAR, while `nets_to_route=[A]` reports it BLOCKED), assert no emitted
   track comes within `v.size/2 + track_width/2 + clearance` of the injected
   via -- and assert the EXACT emitted tuple, because post-fix net A emits
   nothing at all and "no stub inside the floor" would otherwise pass vacuously
   for any change that dropped every pad.
   The NEGATIVE CONTROL is the other half: the same via injected OFF the escape
   ray, 0.4000 mm from the stub against a 0.3750 mm floor, must NOT block it --
   the baseline `(2, 2, [])` has to survive. Without it the gate could be
   arbitrarily over-strict; multiplying the floor by 5 passes every other check
   in this section and fails only here.

2. NO-OP GUARD WITH THE LOOP LIVE. The only guard worth having is a footprint
   that DOES carry pre-existing vias on its fanned nets -- so `_unmapped_vias`
   is non-empty and the new loop really runs -- and is still byte-identical.
   `orangecrab_ext_pll` U5 (7 such vias), U8 (5) and U9 (7) are those. A board
   whose fanned nets carry NO via makes `_unmapped_vias` EMPTY and
   `stub_clears_unmapped` a constant `True`; that guards nothing, so
   `rp2350_fpga_eensy_prePlane` U5 is kept only as the explicitly-labelled
   degenerate case.

3. MEASURED BOARD PIN. `kicad_files/routed_output.kicad_pcb` U2 (QFN-76, 383
   pre-existing vias, 89 of them on the fanned nets) at track 0.1 /
   clearance 0.1 / via 0.45-0.25 emitted seven stubs inside a different-net
   via's clearance floor (0.0000, 0.1000 and 0.2000x5 against a 0.300 floor;
   the 0.0000 one ran through a foreign via CENTRE -- a dead short). Assert
   zero, and pin the resulting escape tally so a collapse to zero escapes is
   a failure rather than a pass.

4. SECOND MEASURED BOARD. `kicad_files/rp2350_fpga_eensy_prePlane.kicad_pcb`
   U6 (QFN-60, 70 vias, 34 on the fanned nets) emitted two stubs at 0.3370
   and 0.2408 against a 0.375 floor. Assert zero, same pinned tally.

5. THE STARKEST CASE. `orangecrab_ext_pll` U7 is a WLCSP-20 at 0.4 mm pitch
   with 17 pre-existing vias on its fanned nets. Pre-fix it escaped all four
   GND balls -- and every one of the four stubs ran through the CENTRE of a
   foreign net's via (5 violations, all at d=0.0000). Post-fix it emits ZERO
   vias and drops all four. This is the largest single behaviour change in the
   corpus and the honest cost of the fix; nothing else in this file pins it.
   `orangecrab_ext_pll` U1 (same package) is pinned beside it.

6. THE FIX IS NOT MONOTONE. `orangecrab_ext_pll` U3 (csBGA-132, 92 vias on the
   fanned nets) drops IO_SCK and QSPI_D2 that used to escape, but RECOVERS
   SPI_CONFIG_MOSI, which the pre-fix engine dropped -- withdrawing one escape
   frees the offset another pad needed. Pinned so "the gate only ever removes
   escapes" cannot be asserted without measuring.

7. LAYER-SCOPE DECISION PIN. The new loop deliberately does NOT filter by
   `via.layers`: `check_drc.check_via_segment_overlap` grades a via against
   segments on EVERY copper layer regardless of the via's span, and
   `obstacle_map._add_via_obstacle` stamps every via on every layer. A
   layer-filtered backstop would emit copper this repo's own grader flags.
   Pinned here so that if check_drc ever becomes layer-aware, this fails and
   points at the fanout loop that should follow it.

    python3 tests/test_619_qfn_underpad_stub_vs_existing_via.py
"""
import contextlib
import io
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_router'))  # #522
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_tools'))  # #522

from kicad_parser import parse_kicad_pcb, Via, Segment
from qfn_fanout import generate_qfn_fanout
from geometry_utils import point_to_segment_distance

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TIGARD = os.path.join(REPO, "kicad_files", "tigard.kicad_pcb")
MEASURED = os.path.join(REPO, "kicad_files", "routed_output.kicad_pcb")
RP2350 = os.path.join(REPO, "kicad_files", "rp2350_fpga_eensy_prePlane.kicad_pcb")
ORANGECRAB = os.path.join(REPO, "kicad_files", "orangecrab_ext_pll.kicad_pcb")

TRACK_W, CLEARANCE, VIA_SIZE, VIA_DRILL, GRID = 0.1, 0.1, 0.45, 0.25, 0.05
FLOOR = VIA_SIZE / 2 + TRACK_W / 2 + CLEARANCE     # 0.3750 mm

fails = []


def check(name, cond):
    print(("  ok  " if cond else " FAIL ") + name)
    if not cond:
        fails.append(name)


def _fanout(pcb, ref, net_filter, layer="F.Cu"):
    """Run the under-pad escape quietly; returns (tracks, vias, dropped)."""
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        return generate_qfn_fanout(
            pcb.footprints[ref], pcb, net_filter=net_filter, layer=layer,
            track_width=TRACK_W, clearance=CLEARANCE, grid_step=GRID,
            escape_method="underpad", via_size=VIA_SIZE, via_drill=VIA_DRILL)


def _stub_via_violations(tracks, vias, clearance=CLEARANCE):
    """Emitted tracks that sit inside a DIFFERENT-net via's clearance floor."""
    out = []
    for t in tracks:
        (x0, y0), (x1, y1) = t['start'], t['end']
        for v in vias:
            if v.net_id == t['net_id']:
                continue
            d = point_to_segment_distance(v.x, v.y, x0, y0, x1, y1)
            floor = v.size / 2 + t['width'] / 2 + clearance
            if d < floor - 1e-6:
                out.append((d, floor, t['net_id'], v.net_id, (v.x, v.y)))
    out.sort()
    return out


def _fanned_via_count(pcb, ref):
    """Exactly the set `_unmapped_vias` holds: pre-existing vias on a net this
    fanout call is escaping. Zero here == the new loop is a constant True."""
    fanned = {p.net_id for p in pcb.footprints[ref].pads if p.net_id}
    return sum(1 for v in pcb.vias if v.net_id in fanned)


def _report(tag, tracks, vias, dropped, viol):
    print(f"      {tag}: tracks={len(tracks)} vias={len(vias)} "
          f"dropped={len(dropped)}")
    for d, floor, tn, vn, pos in viol[:10]:
        print(f"      d={d:.4f} need>={floor:.4f} stub net {tn} vs via net {vn} @{pos}")


# --------------------------------------------------------------------------
# 1. Injection regression + negative control.
# --------------------------------------------------------------------------
def test_injected_foreign_via_blocks_stub():
    print("\n1. injection regression + negative control (tigard U3, QFN-64)")
    pcb = parse_kicad_pcb(TIGARD)
    check("tigard has ZERO board vias (uncontaminated baseline)",
          len(pcb.vias) == 0)

    net_a, net_b = "/BD0", "/BD1"
    nid_a = next(n for n, net in pcb.nets.items() if net.name == net_a)
    nid_b = next(n for n, net in pcb.nets.items() if net.name == net_b)

    # Where does net A's stub land with nothing in the way?
    tracks, vias, dropped = _fanout(pcb, "U3", [net_a, net_b])
    a_stubs = [t for t in tracks if t['net_id'] == nid_a]
    check("baseline: both nets escape, nothing dropped",
          (len(tracks), len(vias), sorted(dropped)) == (2, 2, []))
    if not a_stubs:
        return
    (ax0, ay0), (ax1, ay1) = a_stubs[0]['start'], a_stubs[0]['end']
    ix, iy = (ax0 + ax1) / 2.0, (ay0 + ay1) / 2.0   # dead centre of the stub

    # Re-parse and inject a via on net B right on top of net A's stub.
    pcb2 = parse_kicad_pcb(TIGARD)
    pcb2.vias.append(Via(x=ix, y=iy, size=VIA_SIZE, drill=0.2,
                         layers=["F.Cu", "B.Cu"], net_id=nid_b))

    # -- MECHANISM: the injected via really is absent from the map the escape
    #    builds. nets_to_route=[A, B] erases it (the escape's own call);
    #    nets_to_route=[A] keeps it. Same map, same line, opposite answers.
    from obstacle_map import (build_base_obstacle_map, build_layer_map,
                              check_line_clearance)
    from routing_config import GridRouteConfig
    cfg = GridRouteConfig(layers=list(pcb2.board_info.copper_layers),
                          track_width=TRACK_W, clearance=CLEARANCE)
    lidx = build_layer_map(cfg.layers)["F.Cu"]
    obs_ab = build_base_obstacle_map(pcb2, cfg, nets_to_route=[nid_a, nid_b],
                                     extra_clearance=TRACK_W / 2)
    obs_a = build_base_obstacle_map(pcb2, cfg, nets_to_route=[nid_a],
                                    extra_clearance=TRACK_W / 2)
    check("mechanism: map(nets_to_route=[A,B]) reports the stub line CLEAR "
          "(injected via erased)",
          check_line_clearance(obs_ab, ax0, ay0, ax1, ay1, lidx, cfg) is True)
    check("mechanism: map(nets_to_route=[A]) reports the same line BLOCKED "
          "(injected via present)",
          check_line_clearance(obs_a, ax0, ay0, ax1, ay1, lidx, cfg) is False)

    # -- OUTCOME: no emitted stub may graze the injected via.
    tracks2, vias2, dropped2 = _fanout(pcb2, "U3", [net_a, net_b])
    print(f"      post-injection: tracks={len(tracks2)} vias={len(vias2)} "
          f"dropped={sorted(dropped2)}")
    viol = _stub_via_violations(tracks2, pcb2.vias)
    for d, floor, tn, vn, pos in viol:
        print(f"      d={d:.4f} need>={floor:.4f} stub net {tn} vs via net {vn} @{pos}")
    check("no emitted stub is inside the injected via's clearance floor",
          not viol)

    # -- COUNTER-GUARD: the check above is satisfied by emitting nothing, so
    #    pin what the fix actually does. Net A is DROPPED (the candidate_offsets
    #    ladder only walks FURTHER OUT along the same escape ray, so once a via
    #    sits on that ray every remaining offset is blocked too -- there is no
    #    recovery path). Net B keeps its escape, and it is the #479 reuse
    #    bridge to the injected via (its own net), so no new via is emitted.
    check(f"counter-guard: net A ({net_a}) is DROPPED, not silently re-routed",
          net_a in dropped2)
    check("counter-guard: exact post-injection tuple (1 track, 0 vias, "
          "dropped=['/BD0']) -- a change that dropped EVERY pad would fail here",
          (len(tracks2), len(vias2), sorted(dropped2)) == (1, 0, [net_a]))
    check("counter-guard: the surviving track belongs to net B",
          len(tracks2) == 1 and tracks2[0]['net_id'] == nid_b)

    # -- NEGATIVE CONTROL (the other direction): the same via injected OFF the
    #    escape ray, just OUTSIDE the floor, must NOT block anything. This is
    #    the only check in this file with teeth against an OVER-strict gate:
    #    multiplying the floor by 5 leaves every check above passing and fails
    #    only here.
    dx, dy = ax1 - ax0, ay1 - ay0
    L = math.hypot(dx, dy)
    perp_x, perp_y = -dy / L, dx / L                # unit normal to the stub
    off = 0.40                                      # > FLOOR (0.3750), < 5*FLOOR
    nx, ny = ix + perp_x * off, iy + perp_y * off
    pcb3 = parse_kicad_pcb(TIGARD)
    pcb3.vias.append(Via(x=nx, y=ny, size=VIA_SIZE, drill=0.2,
                         layers=["F.Cu", "B.Cu"], net_id=nid_b))
    d_ctrl = point_to_segment_distance(nx, ny, ax0, ay0, ax1, ay1)
    print(f"      negative control: via on net B at d={d_ctrl:.4f} from net A's "
          f"stub (floor {FLOOR:.4f})")
    check(f"negative control is in the band an over-strict floor would catch "
          f"({FLOOR:.4f} <= {d_ctrl:.4f} < {5 * FLOOR:.4f})",
          FLOOR <= d_ctrl < 5 * FLOOR)
    tracks3, vias3, dropped3 = _fanout(pcb3, "U3", [net_a, net_b])
    print(f"      negative control: tracks={len(tracks3)} vias={len(vias3)} "
          f"dropped={sorted(dropped3)}")
    check("negative control: a via OUTSIDE the floor does NOT block the stub "
          "-- baseline (2 tracks, 2 vias, nothing dropped) survives",
          (len(tracks3), len(vias3), sorted(dropped3)) == (2, 2, []))


# --------------------------------------------------------------------------
# 2. No-op guard, with the loop LIVE (and the degenerate case labelled).
# --------------------------------------------------------------------------
def test_no_op_when_gate_finds_nothing_to_reject():
    print("\n2. no-op guard")
    print("   2a. LOOP LIVE: pre-existing vias ON the fanned nets, and the "
          "gate rejects none of them")
    for ref, n_exp, expect in (("U5", 7, (13, 13, 3)),
                               ("U8", 5, (7, 7, 3)),
                               ("U9", 7, (15, 15, 9))):
        pcb = parse_kicad_pcb(ORANGECRAB)
        n_fanned = _fanned_via_count(pcb, ref)
        check(f"orangecrab_ext_pll {ref}: _unmapped_vias is NON-EMPTY "
              f"({n_fanned} pre-existing vias on the fanned nets, expected "
              f"{n_exp}) -- the new loop actually runs here",
              n_fanned == n_exp and n_fanned > 0)
        tracks, vias, dropped = _fanout(pcb, ref, None,
                                        layer=pcb.footprints[ref].layer)
        print(f"      orangecrab_ext_pll {ref}: tracks={len(tracks)} "
              f"vias={len(vias)} dropped={len(dropped)}")
        check(f"orangecrab_ext_pll {ref}: escape tally unchanged by the new "
              f"gate {expect} -- it does not over-reject",
              (len(tracks), len(vias), len(dropped)) == expect)

    print("   2b. DEGENERATE (labelled): board HAS vias but none on a fanned "
          "net, so _unmapped_vias is EMPTY and the gate is a constant True")
    pcb = parse_kicad_pcb(RP2350)
    n_fanned = _fanned_via_count(pcb, "U5")
    check(f"rp2350_fpga_eensy_prePlane U5: {len(pcb.vias)} board vias, "
          f"{n_fanned} on a fanned net -- this guard is INERT BY "
          f"CONSTRUCTION, not evidence the gate is well-behaved",
          len(pcb.vias) > 0 and n_fanned == 0)
    tracks, vias, dropped = _fanout(pcb, "U5", None,
                                    layer=pcb.footprints["U5"].layer)
    print(f"      rp2350_fpga_eensy_prePlane U5: tracks={len(tracks)} "
          f"vias={len(vias)} dropped={len(dropped)}")
    check("rp2350_fpga_eensy_prePlane U5: escape tally unchanged (3, 3, 1)",
          (len(tracks), len(vias), len(dropped)) == (3, 3, 1))


# --------------------------------------------------------------------------
# 3. The board the issue measured.
# --------------------------------------------------------------------------
def test_measured_board_u2():
    print("\n3. measured board (routed_output U2, QFN-76, 383 existing vias)")
    pcb = parse_kicad_pcb(MEASURED)
    check("board carries pre-existing vias on the fanned nets (the condition "
          f"under test); got {_fanned_via_count(pcb, 'U2')}",
          _fanned_via_count(pcb, "U2") > 50)
    tracks, vias, dropped = _fanout(pcb, "U2", None)
    viol = _stub_via_violations(tracks, pcb.vias)
    _report("U2", tracks, vias, dropped, viol)
    # Guard against a trivial pass: the escape must still do real work, and
    # the exact cost of the fix is pinned (26 -> 19 vias, 15 -> 22 dropped).
    check("U2 escape tally pinned at 20 tracks / 19 vias / 22 dropped "
          "(was 27 / 26 / 15 pre-fix)",
          (len(tracks), len(vias), len(dropped)) == (20, 19, 22))
    check("no emitted stub sits inside a different-net existing via's floor "
          f"(was 7 pre-fix, incl. one through a via centre); got {len(viol)}",
          not viol)


# --------------------------------------------------------------------------
# 4. The second regressing board found by the corpus sweep.
# --------------------------------------------------------------------------
def test_measured_board_rp2350_u6():
    print("\n4. second measured board (rp2350_fpga_eensy_prePlane U6, QFN-60)")
    pcb = parse_kicad_pcb(RP2350)
    check("U6 has pre-existing vias on its fanned nets; got "
          f"{_fanned_via_count(pcb, 'U6')}",
          _fanned_via_count(pcb, "U6") > 10)
    tracks, vias, dropped = _fanout(pcb, "U6", None)
    viol = _stub_via_violations(tracks, pcb.vias)
    _report("U6", tracks, vias, dropped, viol)
    check("U6 escape tally pinned at 10 tracks / 7 vias / 37 dropped "
          "(was 12 / 9 / 35 pre-fix)",
          (len(tracks), len(vias), len(dropped)) == (10, 7, 37))
    check("no emitted stub sits inside a different-net existing via's floor "
          f"(was 2 pre-fix, at d=0.3370 and d=0.2408 vs a 0.375 floor); "
          f"got {len(viol)}",
          not viol)


# --------------------------------------------------------------------------
# 5. The starkest case in the corpus: a fine-pitch WLCSP that escaped every
#    GND ball through the CENTRE of a foreign via, and now escapes none.
# --------------------------------------------------------------------------
def test_orangecrab_wlcsp_u7_and_u1():
    print("\n5. starkest case (orangecrab_ext_pll U7/U1, WLCSP-20 @ 0.4mm pitch)")
    for ref, n_exp, expect, n_viol_pre in (("U7", 17, (2, 0, 4), 5),
                                           ("U1", 17, (2, 0, 4), 4)):
        pcb = parse_kicad_pcb(ORANGECRAB)
        check(f"{ref} has pre-existing vias on its fanned nets; got "
              f"{_fanned_via_count(pcb, ref)} (expected {n_exp})",
              _fanned_via_count(pcb, ref) == n_exp)
        tracks, vias, dropped = _fanout(pcb, ref, None,
                                        layer=pcb.footprints[ref].layer)
        viol = _stub_via_violations(tracks, pcb.vias)
        _report(ref, tracks, vias, dropped, viol)
        check(f"{ref} escape tally pinned at 2 tracks / 0 vias / 4 dropped "
              f"-- EVERY GND ball is withdrawn, this is the fix's largest "
              f"single cost and it is deliberate",
              (len(tracks), len(vias), len(dropped)) == expect)
        check(f"{ref}: the four withdrawn escapes are all GND",
              sorted(dropped) == ["GND"] * 4)
        check(f"{ref}: no emitted stub sits inside a different-net existing "
              f"via's floor (was {n_viol_pre} pre-fix, every one at d=0.0000 "
              f"-- straight through a foreign via centre); got {len(viol)}",
              not viol)


# --------------------------------------------------------------------------
# 6. The gate is not monotone: it also RECOVERS an escape.
# --------------------------------------------------------------------------
def test_orangecrab_u3_recovers_an_escape():
    print("\n6. not monotone (orangecrab_ext_pll U3, csBGA-132)")
    pcb = parse_kicad_pcb(ORANGECRAB)
    check(f"U3 has pre-existing vias on its fanned nets; got "
          f"{_fanned_via_count(pcb, 'U3')}",
          _fanned_via_count(pcb, "U3") == 92)
    tracks, vias, dropped = _fanout(pcb, "U3", None,
                                    layer=pcb.footprints["U3"].layer)
    viol = _stub_via_violations(tracks, pcb.vias)
    _report("U3", tracks, vias, dropped, viol)
    check("U3 escape tally pinned at 45 tracks / 45 vias / 50 dropped "
          "(was 47 / 47 / 48 pre-fix)",
          (len(tracks), len(vias), len(dropped)) == (45, 45, 50))
    check("U3: IO_SCK and QSPI_D2 are withdrawn (they escaped pre-fix)",
          "IO_SCK" in dropped and "QSPI_D2" in dropped)
    check("U3: SPI_CONFIG_MOSI is RECOVERED -- it was DROPPED pre-fix and "
          "escapes now, so the gate is not merely subtractive",
          "SPI_CONFIG_MOSI" not in dropped)
    check(f"U3: no emitted stub sits inside a different-net existing via's "
          f"floor (was 2 pre-fix); got {len(viol)}", not viol)


# --------------------------------------------------------------------------
# 7. Layer-scope decision pin (why the new loop does NOT read via.layers).
# --------------------------------------------------------------------------
def test_backstop_layer_blindness_matches_check_drc():
    print("\n7. layer scope: the backstop is layer-blind because check_drc is")
    from check_drc import check_via_segment_overlap
    # A BURIED via that touches neither F.Cu nor B.Cu ...
    buried = Via(x=10.0, y=10.0, size=0.45, drill=0.25,
                 layers=["In1.Cu", "In2.Cu"], net_id=1)
    # ... and a track on F.Cu running straight through its centre.
    seg = Segment(start_x=9.0, start_y=10.0, end_x=11.0, end_y=10.0,
                  width=TRACK_W, layer="F.Cu", net_id=2)
    hit, overlap = check_via_segment_overlap(buried, seg, CLEARANCE)
    print(f"      check_via_segment_overlap(buried In1/In2 via, F.Cu seg) "
          f"-> hit={hit} overlap={overlap:.4f}mm")
    check("check_drc grades a via against segments on EVERY copper layer "
          "regardless of via.layers, so a layer-filtered fanout backstop "
          "would ship copper this repo's own grader flags",
          hit is True)
    # And the shipped backstop agrees: same geometry, same verdict.
    d = point_to_segment_distance(buried.x, buried.y, seg.start_x, seg.start_y,
                                  seg.end_x, seg.end_y)
    check(f"the backstop's own floor test agrees (d={d:.4f} < {FLOOR:.4f})",
          d < FLOOR - 1e-6)


def run():
    for board in (TIGARD, MEASURED, RP2350, ORANGECRAB):
        if not os.path.exists(board):
            print(f"FAIL: missing board {board}")
            return 1
    test_injected_foreign_via_blocks_stub()
    test_no_op_when_gate_finds_nothing_to_reject()
    test_measured_board_u2()
    test_measured_board_rp2350_u6()
    test_orangecrab_wlcsp_u7_and_u1()
    test_orangecrab_u3_recovers_an_escape()
    test_backstop_layer_blindness_matches_check_drc()
    print()
    if fails:
        print("FAILED: %d check(s): %s" % (len(fails), fails))
        return 1
    print("All checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(run())
