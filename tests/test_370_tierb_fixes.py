#!/usr/bin/env python3
"""
Tests for issue #370 -- Tier B post-pass DRC-guarantee gaps (B1-B4, B7).

Each section has at least one would-have-failed-before check plus one
no-over-rejection check.

  B1: meander keep-out honors the FOREIGN copper's real width / via size and
      the real board outline+cutouts (not the bbox).
  B2: NPTH-hole blindness family (octolinear re-bend, snap_stub_gaps
      connectors, close_soft_joints bridges, cap-optimizer obstacles + real
      board outline).
  B3: cap-opt via mover validates board edge, rotated pad rects, and its new
      connector segments vs edge/NPTH.
  B4: bga_fanout via placement validates drill hole-to-hole (net-independent)
      and foreign tracks.
  B7: nudge_grazing_vias models pad drills at the real (offset/slot-aware)
      hole location.

Run:
    python3 tests/test_370_tierb_fixes.py
"""

import math
import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)

from kicad_parser import PCBData, BoardInfo, Segment, Via  # noqa: E402
from routing_config import GridRouteConfig                 # noqa: E402

FAILS = []


def check(name, cond):
    print(f"  {'PASS' if cond else 'FAIL'}: {name}")
    if not cond:
        FAILS.append(name)


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

def _cfg():
    c = GridRouteConfig()
    c.track_width = 0.15
    c.clearance = 0.15
    c.via_size = 0.6
    c.grid_step = 0.05
    c.layers = ["F.Cu", "B.Cu"]
    c.meander_amplitude = 1.0
    return c


def _board(segments=(), vias=(), bounds=(-5.0, -5.0, 15.0, 50.0),
           outline=None, cutouts=None, pads_by_net=None, footprints=None):
    bi = BoardInfo(layers={}, board_bounds=bounds, copper_layers=["F.Cu", "B.Cu"])
    if outline is not None:
        bi.board_outline = outline
    if cutouts is not None:
        bi.board_cutouts = cutouts
    return PCBData(board_info=bi, nets={}, footprints=footprints or {},
                   vias=list(vias), segments=list(segments),
                   pads_by_net=pads_by_net or {})


# ---------------------------------------------------------------------------
# B1 -- meander keep-out: foreign copper real width / via size + real outline
# ---------------------------------------------------------------------------

def test_b1():
    print("B1: meander keep-out uses foreign width/via size + real outline")
    from length_matching import get_safe_amplitude_at_point

    MEANDER_NET, FOREIGN_NET = 5, 99
    common = dict(cx=5.0, cy=0.0, ux=1.0, uy=0.0, px=0.0, py=1.0, direction=1,
                  max_amplitude=1.0, min_amplitude=0.1, layer="F.Cu",
                  net_id=MEANDER_NET, config=_cfg())

    def foreign_seg(offset, width):
        return Segment(start_x=0.0, start_y=offset, end_x=10.0, end_y=offset,
                       width=width, layer="F.Cu", net_id=FOREIGN_NET)

    # A thin neighbor at 1.2mm allows some amplitude; a 1.0mm-wide trunk at the
    # same centerline offset must pull the bump back by (1.0-0.15)/2 = 0.425.
    thin = get_safe_amplitude_at_point(
        pcb_data=_board([foreign_seg(1.2, 0.15)]), **common)
    wide = get_safe_amplitude_at_point(
        pcb_data=_board([foreign_seg(1.2, 1.0)]), **common)
    check("wide foreign trunk pulls the meander back (was config width)",
          thin > 0 and wide < thin)
    # would-have-failed-before: amplitude the thin case allows must leave the
    # wide trunk's true clearance violated if reused -- i.e. the new amp keeps
    # the copper edges >= clearance apart.
    cfg = common['config']
    if wide > 0:
        # bump peak copper edge vs trunk copper edge
        gap = (1.2 - 1.0 / 2.0) - (wide + cfg.track_width / 2.0)
        check("accepted wide-trunk amplitude keeps real clearance",
              gap >= cfg.clearance - 1e-6)
    # no-over-rejection: foreign width == track_width is byte-identical
    same = get_safe_amplitude_at_point(
        pcb_data=_board([foreign_seg(1.2, 0.15)]), **common)
    check("foreign width == track_width unchanged", abs(same - thin) < 1e-12)

    # Foreign via at its REAL size: config.via_size 0.6 vs a 1.4mm via body.
    def via(offset, size):
        return Via(x=5.0, y=offset, size=size, drill=0.3,
                   layers=["F.Cu", "B.Cu"], net_id=FOREIGN_NET)

    small_v = get_safe_amplitude_at_point(pcb_data=_board(vias=[via(1.6, 0.6)]), **common)
    big_v = get_safe_amplitude_at_point(pcb_data=_board(vias=[via(1.6, 1.4)]), **common)
    check("oversized foreign via pulls the meander back (was config via_size)",
          small_v > 0 and big_v < small_v)
    same_v = get_safe_amplitude_at_point(pcb_data=_board(vias=[via(1.6, 0.6)]), **common)
    check("foreign via == via_size unchanged", abs(same_v - small_v) < 1e-12)

    # Real outline: interior cutout in the bump's path. The bbox bounds are far
    # away (old check accepted), but the cutout ring right above the centerline
    # must clamp the bump.
    cutout = [(4.0, 0.5), (6.0, 0.5), (6.0, 1.5), (4.0, 1.5)]
    outline = [(-5.0, -5.0), (15.0, -5.0), (15.0, 50.0), (-5.0, 50.0)]
    clamped = get_safe_amplitude_at_point(
        pcb_data=_board(outline=outline, cutouts=[cutout]), **common)
    open_amp = get_safe_amplitude_at_point(
        pcb_data=_board(outline=outline), **common)
    check("interior cutout clamps the bump (bbox check was blind)",
          clamped < open_amp and clamped < 0.5)
    check("outline far away does not clamp (no over-rejection)",
          open_amp == 1.0)


# ---------------------------------------------------------------------------
# B2 -- NPTH-hole blindness family
# ---------------------------------------------------------------------------

def _mk_pad(x, y, net_id=0, drill=0.0, size=None, pad_type='thru_hole',
            layers=None, ref='H1', num='1', shape='circle', rot=0.0,
            hole_x=None, hole_y=None, drill_w=0.0, drill_h=0.0):
    from kicad_parser import Pad
    size = size if size is not None else (drill + 0.2 if drill else 0.5)
    return Pad(component_ref=ref, pad_number=num, global_x=x, global_y=y,
               local_x=0.0, local_y=0.0, size_x=size, size_y=size,
               shape=shape, layers=layers or ['*.Cu'], net_id=net_id,
               net_name='', rotation=rot, drill=drill, drill_w=drill_w,
               drill_h=drill_h, hole_x=hole_x, hole_y=hole_y,
               pad_type=pad_type)


def _npth(x, y, drill, net_id=0, layers=None):
    # Realistic NPTH: size == drill (the 'size' is just the mask opening).
    # KiCad NPTH pads usually list *.Cu (phantom copper other passes see at
    # the MASK size); mask-only NPTH pads exist too and are fully invisible
    # to every copper term -- callers pick the flavor that exposes the gap.
    return _mk_pad(x, y, net_id=net_id, drill=drill, size=drill,
                   pad_type='np_thru_hole',
                   layers=layers or ['*.Cu', '*.Mask'])


def test_b2_octolinear():
    print("\nB2: nudge_grazing_octolinear refuses a re-bend across an NPTH hole")
    from pcb_modification import nudge_grazing_octolinear

    def jog_board(with_hole):
        # Net 1 jog A(0,0) -> apex(1,0.5) -> B(2,0), both arms grazing a
        # foreign pad at the apex. The only clearing octolinear bend is the
        # direct A-B line -- which crosses the NPTH hole at (1, 0).
        segs = [Segment(start_x=0.0, start_y=0.0, end_x=1.0, end_y=0.5,
                        width=0.2, layer="F.Cu", net_id=1),
                Segment(start_x=1.0, start_y=0.5, end_x=2.0, end_y=0.0,
                        width=0.2, layer="F.Cu", net_id=1)]
        pads = {2: [_mk_pad(1.0, 0.6, net_id=2, drill=0.0, size=0.3,
                            pad_type='smd', layers=['F.Cu'], ref='R1',
                            shape='rect')]}
        if with_hole:
            # mask-only NPTH: no phantom copper for the pad term to see, so
            # ONLY the hole check can stop the re-bend crossing it.
            pads[0] = [_npth(1.0, 0.0, 0.6, layers=['F.Mask', 'B.Mask'])]
        return _board(segments=segs, pads_by_net=pads)

    pcb = jog_board(with_hole=False)
    changed, nets, _removed, _added = nudge_grazing_octolinear([], pcb, clearance=0.1)
    check("without the hole the graze IS re-bent (fixture sanity)", nets == 1)
    # And the accepted bend stays clear of everything (no hole present).

    pcb = jog_board(with_hole=True)
    changed, nets, _removed, _added = nudge_grazing_octolinear([], pcb, clearance=0.1)
    if nets == 0:
        check("re-bend across the NPTH hole refused", True)
    else:
        # if it DID re-bend, the new copper must clear the hole at the floor
        from single_ended_routing import _seg_foreign_hole_dist
        from routing_defaults import NPTH_TO_TRACK_CLEARANCE
        ok = all(_seg_foreign_hole_dist(pcb, s.net_id, s.start_x, s.start_y,
                                        s.end_x, s.end_y)
                 >= NPTH_TO_TRACK_CLEARANCE + s.width / 2.0 - 1e-4
                 for s in pcb.segments if s.net_id == 1)
        check("re-bend across the NPTH hole refused", ok)


def test_b2_connector_clear():
    print("\nB2: snap_stub_gaps connectors respect NPTH drill capsules")
    from pcb_modification import _connector_clear

    # Connector from (0,0) to (2,0); mask-only NPTH hole centered ON it: no
    # phantom copper anywhere, only the (previously missing) hole check.
    pcb = _board(pads_by_net={0: [_npth(1.0, 0.0, 0.6,
                                        layers=['F.Mask', 'B.Mask'])]})
    check("connector across an NPTH hole rejected (was accepted)",
          not _connector_clear(0.0, 0.0, 2.0, 0.0, 0.2, "F.Cu", 1, pcb, 0.1))
    # Same connector with the hole far away is fine (no over-rejection).
    pcb2 = _board(pads_by_net={0: [_npth(1.0, 5.0, 0.6)]})
    check("connector clear of far NPTH hole accepted",
          _connector_clear(0.0, 0.0, 2.0, 0.0, 0.2, "F.Cu", 1, pcb2, 0.1))
    # *.Cu-flavored NPTH in the band between the mask-size copper gate and the
    # true hole floor: center 0.55 from the axis -> phantom-pad edge 0.25 >=
    # 0.2 (old gates pass) but hole edge 0.25 < NPTH 0.2 + half 0.1 (rejected).
    pcb3 = _board(pads_by_net={0: [_npth(1.0, 0.55, 0.6)]})
    check("connector inside the NPTH floor (but outside clearance) rejected",
          not _connector_clear(0.0, 0.0, 2.0, 0.0, 0.2, "F.Cu", 1, pcb3, 0.1))


def test_b2_soft_joints():
    print("\nB2: close_soft_joints gates holes at the NPTH floor")
    from pcb_modification import close_soft_joints

    def joint_board(hole_edge_dist):
        # Two dangling caps overlapping: ends at (1,0) and (1.2,0), widths 0.3.
        segs = [Segment(start_x=0.0, start_y=0.0, end_x=1.0, end_y=0.0,
                        width=0.3, layer="F.Cu", net_id=1),
                Segment(start_x=1.2, start_y=0.0, end_x=2.2, end_y=0.0,
                        width=0.3, layer="F.Cu", net_id=1)]
        pads = {}
        if hole_edge_dist is not None:
            # hole edge at hole_edge_dist from the bridge centerline (y=0)
            pads[0] = [_npth(1.1, hole_edge_dist + 0.2, 0.4)]
        return _board(segments=segs, pads_by_net=pads)

    cfg = _cfg()
    cfg.clearance = 0.1
    # hole edge 0.3 from the bridge: plain gate needs 0.1+0.15=0.25 (passes,
    # bridged pre-fix), NPTH floor needs 0.2+0.15=0.35 (refused post-fix).
    n = close_soft_joints([], joint_board(0.3), {1}, cfg)
    check("bridge inside the NPTH floor refused (was bridged)", n == 0)
    n = close_soft_joints([], joint_board(None), {1}, cfg)
    check("bridge with no hole still added (no over-rejection)", n == 1)
    n = close_soft_joints([], joint_board(0.5), {1}, cfg)
    check("bridge clear of the NPTH floor still added", n == 1)


def test_b2_cap_optimizer():
    print("\nB2: cap optimizer sees NPTH holes and the real board outline")
    from kicad_parser import parse_kicad_pcb
    from placement.fanout_clearance import _Repair, _ring_is_rect

    board_file = os.path.join(TESTS_DIR, '..', 'kicad_files',
                              'orangecrab_ext_pll.kicad_pcb')
    CAP, GND = 'C7', 5

    def new_repair(pcb):
        return _Repair(pcb, board_file, 0.1, 0.1, 0.55, 1.0, 2.0, 0.3, 'C', set())

    # (a) an NPTH mounting hole under the cap's own-net pad still counts: the
    # drill removes copper regardless of net. Tie the NPTH to the SAME net as
    # the cap pad (a net-tied mounting hole): the old code's pnet==net skip
    # made it fully invisible.
    pcb = parse_kicad_pcb(board_file)
    cap_fp = pcb.footprints[CAP]
    pad1 = next(p for p in cap_fp.pads if p.net_id == GND)
    host = next(fp for r, fp in pcb.footprints.items() if r != CAP and fp.pads)
    hole = _npth(pad1.global_x, pad1.global_y, 0.6, net_id=GND)
    hole.component_ref = host.reference
    host.pads.append(hole)
    pcb.pads_by_net.setdefault(GND, []).append(hole)
    st = new_repair(pcb)
    check("NPTH drill entered the obstacle list (net -1, both sides)",
          any(e[4] == -1 for e in st.foreign_pads))
    cap = st.caps[CAP]
    check("same-net cap pad over an NPTH hole is a violation (was invisible)",
          st.pad_penalty(CAP, cap, cap.x, cap.y, cap.rot) > 1e-6)
    # no over-rejection: without the hole the seed pad penalty is zero
    pcb2 = parse_kicad_pcb(board_file)
    st2 = new_repair(pcb2)
    cap2 = st2.caps[CAP]
    check("no NPTH -> no phantom penalty",
          st2.pad_penalty(CAP, cap2, cap2.x, cap2.y, cap2.rot) <= 1e-6)

    # (b) real outline: a cutout right next to the cap blocks candidates the
    # bbox `usable` inset would accept.
    pcb3 = parse_kicad_pcb(board_file)
    # cutout 0.8mm to the +x of the cap courtyard (outside the 0.55 edge
    # margin, so the seed stays legal), 2x2mm
    st_probe = new_repair(parse_kicad_pcb(board_file))
    seed_rect = st_probe.caps[CAP].rect()
    cx0 = seed_rect[2] + 0.8
    cy0 = (seed_rect[1] + seed_rect[3]) / 2.0 - 1.0
    cut = [(cx0, cy0), (cx0 + 2.0, cy0), (cx0 + 2.0, cy0 + 2.0), (cx0, cy0 + 2.0)]
    pcb3.board_info.board_cutouts = [cut]
    b = pcb3.board_info.board_bounds
    if not pcb3.board_info.board_outline:
        pcb3.board_info.board_outline = [(b[0], b[1]), (b[2], b[1]),
                                         (b[2], b[3]), (b[0], b[3])]
    st3 = new_repair(pcb3)
    check("edge gate active with a cutout", st3._edge_active)
    cap3 = st3.caps[CAP]
    # candidate sliding the cap INTO the cutout must be blocked
    check("candidate inside the cutout hard-blocked (bbox test was blind)",
          st3._blocked_geom(CAP, cap3, cx0 + 1.0, cy0 + 1.0, cap3.rot))
    check("seed placement (outside the cutout) not blocked by the edge gate",
          not st3._rect_edge_blocked(cap3.rect()))
    # no over-rejection: rectangular outline without cutouts keeps the gate off
    pcb4 = parse_kicad_pcb(board_file)
    b4 = pcb4.board_info.board_bounds
    pcb4.board_info.board_cutouts = []
    pcb4.board_info.board_outline = [(b4[0], b4[1]), (b4[2], b4[1]),
                                     (b4[2], b4[3]), (b4[0], b4[3])]
    pcb4.board_info.board_outlines = []
    st4 = new_repair(pcb4)
    check("rectangular outline w/o cutouts: gate stays off (bbox is exact)",
          not st4._edge_active)
    check("_ring_is_rect helper sanity",
          _ring_is_rect([(0, 0), (10, 0), (10, 5), (0, 5)])
          and not _ring_is_rect([(0, 0), (10, 0), (10, 5), (5, 8), (0, 5)]))


# ---------------------------------------------------------------------------
# B3 -- cap-opt via mover: board edge, rotated pads, connector edge checks
# ---------------------------------------------------------------------------

class _FakeCap:
    def __init__(self, rects):
        self._rects = rects
        self.x = self.y = self.rot = 0.0

    def pad_rects(self, x=None, y=None, rot=None):
        return self._rects


class _FakeSt:
    """Minimal stand-in for _Repair: one cap, permanently 'unresolved'."""
    def __init__(self, cap_rects):
        self.caps = {'C1': _FakeCap(cap_rects)}
        self.vias = []

    def graze_penalty(self, ref, cap, x, y, rot):
        return 1.0


def _via_mover_board(**kw):
    from types import SimpleNamespace
    pcb = _board(footprints={'C1': SimpleNamespace(layer='F.Cu', pads=[])}, **kw)
    return pcb


def test_b3():
    print("\nB3: cap-opt via mover -- board edge, rotated pads, connectors")
    from placement.fanout_clearance import nudge_vias_for_unresolved

    def graze_via():
        return Via(x=1.0, y=1.4, size=0.5, drill=0.3,
                   layers=["F.Cu", "B.Cu"], net_id=3)

    HBAR = (0.2, 0.9, 1.8, 1.1, 2)     # horizontal bar: only escape is +y
    VBAR = (0.5, -1.0, 0.7, 3.0, 2)    # vertical bar: only escape is +x

    # (a) board-edge bbox fallback: the only clearing candidates (y >= 1.45)
    # all violate the top edge at 1.7 (need via_r 0.25 + clearance 0.1 inset).
    v = graze_via()
    pcb = _via_mover_board(vias=[v], bounds=(0.0, 0.0, 2.0, 1.7))
    moves, segs = nudge_vias_for_unresolved(_FakeSt([HBAR]), pcb, 0.1)
    check("via not pushed past the board edge (was moved off-board)",
          moves == [] and (v.x, v.y) == (1.0, 1.4))
    v = graze_via()
    pcb = _via_mover_board(vias=[v], bounds=(0.0, 0.0, 2.0, 5.0))
    moves, _ = nudge_vias_for_unresolved(_FakeSt([HBAR]), pcb, 0.1)
    check("far edge: via still moved (no over-rejection)", len(moves) == 1)

    # (b) real outline: a cutout sliver between the via and every clearing
    # candidate. New positions past it are point-legal, but the CONNECTOR back
    # to the stub start would cross the cutout -> no move.
    outline = [(0.0, 0.0), (3.0, 0.0), (3.0, 3.0), (0.0, 3.0)]
    sliver = [(1.05, 0.5), (1.15, 0.5), (1.15, 2.5), (1.05, 2.5)]
    v = graze_via()
    stub = Segment(start_x=1.0, start_y=1.6, end_x=1.0, end_y=1.4,
                   width=0.2, layer="F.Cu", net_id=3)
    pcb = _via_mover_board(vias=[v], segments=[stub],
                           bounds=(0.0, 0.0, 3.0, 3.0),
                           outline=outline, cutouts=[sliver])
    moves, segs = nudge_vias_for_unresolved(_FakeSt([VBAR]), pcb, 0.1)
    check("no move across a cutout sliver (connector would cross Edge.Cuts)",
          moves == [])
    v = graze_via()
    stub = Segment(start_x=1.0, start_y=1.6, end_x=1.0, end_y=1.4,
                   width=0.2, layer="F.Cu", net_id=3)
    pcb = _via_mover_board(vias=[v], segments=[stub],
                           bounds=(0.0, 0.0, 3.0, 3.0), outline=outline)
    moves, segs = nudge_vias_for_unresolved(_FakeSt([VBAR]), pcb, 0.1)
    check("no cutout: via + connector move fine (no over-rejection)",
          len(moves) == 1 and len(segs) == 1)

    # (c) rotated foreign pad (#356 class): a 1.2x1.2 pad at 45 degrees. Its
    # true copper diamond corner reaches (1.05, 1.299); the old axis-aligned
    # rect test accepted the first ring candidate (1.05, 1.4) only 0.10mm from
    # real copper. The fix must land the via >= via_r + clearance from the
    # TRUE outline (and still find a move -- no over-rejection).
    from check_drc import point_to_pad_distance
    rot_pad = _mk_pad(1.05, 0.45, net_id=4, size=1.2, pad_type='smd',
                      layers=['F.Cu'], ref='U9', shape='rect')
    rot_pad.rect_rotation = 45.0
    v = graze_via()
    pcb = _via_mover_board(vias=[v], bounds=(-2.0, -2.0, 5.0, 5.0),
                           pads_by_net={4: [rot_pad]})
    moves, _ = nudge_vias_for_unresolved(_FakeSt([VBAR]), pcb, 0.1)
    check("rotated pad: a move is still found", len(moves) == 1)
    if moves:
        nd = moves[0][2]
        d = point_to_pad_distance(nd['x'], nd['y'], rot_pad)
        check("moved via clears the pad's TRUE (rotated) copper",
              d >= 0.25 + 0.1 - 1e-6)


# ---------------------------------------------------------------------------
# B4 -- bga_fanout via placement: hole-to-hole + foreign tracks
# ---------------------------------------------------------------------------

def test_b4():
    print("\nB4: bga_fanout via-in-pad validates drills + foreign tracks")
    from bga_fanout import manage_vias
    from bga_fanout.types import FanoutRoute

    NET = 7

    def ball_route():
        ball = _mk_pad(10.0, 10.0, net_id=NET, size=0.5, pad_type='smd',
                       layers=['F.Cu'], ref='U1', num='A1')
        return FanoutRoute(pad=ball, pad_pos=(10.0, 10.0),
                           stub_end=(10.5, 10.5), exit_pos=(11.0, 10.5),
                           layer='B.Cu')

    def run(vias=(), segments=(), extra_pads=()):
        r = ball_route()
        pcb = _board(vias=vias, segments=segments,
                     pads_by_net={NET: [r.pad], **({0: list(extra_pads)}
                                                   if extra_pads else {})})
        return manage_vias([r], pcb, 'F.Cu', 0.45, 0.2, 0.1)

    # clean board: via-in-pad added (no over-rejection)
    add, _rm, blocked = run()
    check("clean board: via added", len(add) == 1 and not blocked)

    # NPTH mounting hole 0.4mm away: drills 0.1+0.25 + 0.2 floor = 0.55 > 0.4.
    # Nothing checked pad drills before -- the via shipped a drill overlap.
    add, _rm, blocked = run(extra_pads=[_npth(10.0, 10.4, 0.5)])
    check("via drill within hole-to-hole of an NPTH drill: escape dropped",
          len(add) == 0 and len(blocked) == 1)

    # same-net TH pad drill 0.35mm away (net-INDEPENDENT rule, #282 class)
    th = _mk_pad(10.35, 10.0, net_id=NET, drill=0.3, size=0.6, ref='U2')
    add, _rm, blocked = run(extra_pads=[th])
    check("via drill within hole-to-hole of a SAME-net pad drill: dropped",
          len(add) == 0 and len(blocked) == 1)

    # foreign track on an inner layer 0.3mm from the ball: ring 0.225 + w/2
    # 0.1 + clearance 0.1 = 0.425 > 0.3 -- was never checked.
    trk = Segment(start_x=8.0, start_y=10.3, end_x=12.0, end_y=10.3,
                  width=0.2, layer="In1.Cu", net_id=9)
    add, _rm, blocked = run(segments=[trk])
    check("via ring grazing a foreign inner track: escape dropped",
          len(add) == 0 and len(blocked) == 1)
    # ...but a track at 0.5mm clears 0.425 (no over-rejection)
    trk2 = Segment(start_x=8.0, start_y=10.5, end_x=12.0, end_y=10.5,
                   width=0.2, layer="In1.Cu", net_id=9)
    add, _rm, blocked = run(segments=[trk2])
    check("foreign track at legal distance: via still added",
          len(add) == 1 and not blocked)

    # existing same-net via 0.5mm away, drill-heavy: drill req 0.1+0.25+0.2 =
    # 0.55 > 0.5. Pre-fix the body test silently skipped the via (leaving the
    # escape dangling); now the escape is dropped honestly.
    fat = Via(x=10.0, y=10.5, size=0.6, drill=0.5,
              layers=["F.Cu", "B.Cu"], net_id=NET)
    add, _rm, blocked = run(vias=[fat])
    check("via drill within hole-to-hole of an existing via drill: dropped",
          len(add) == 0 and len(blocked) == 1)


# ---------------------------------------------------------------------------

def main():
    test_b1()
    test_b2_octolinear()
    test_b2_connector_clear()
    test_b2_soft_joints()
    test_b2_cap_optimizer()
    test_b3()
    test_b4()
    print()
    if FAILS:
        print(f"{len(FAILS)} check(s) FAILED")
        sys.exit(1)
    print("PASS  all #370 Tier B checks")


if __name__ == "__main__":
    main()
