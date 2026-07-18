"""
QFN/QFP Fanout Strategy - Creates escape routing for QFN/QFP packages.

Generic module that analyzes actual pad geometry to determine:
- Which side each pad is on (based on position and pad orientation)
- Escape direction (perpendicular to pad's long axis)
- Stub length (based on chip size)
- Fan-out pattern (endpoints maximally separated)

Works with any QFN/QFP package regardless of pin count or size.
"""
from __future__ import annotations

import math
from typing import List, Dict, Tuple, Optional
from collections import defaultdict

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_parser import parse_kicad_pcb, Footprint, PCBData, find_components_by_type
from kicad_writer import add_tracks_and_vias_to_pcb
from qfn_fanout.types import QFNLayout, PadInfo, FanoutStub
from bga_fanout.constants import POSITION_TOLERANCE
from net_queries import matches_net_filter
from qfn_fanout.layout import analyze_qfn_layout, analyze_pad
from qfn_fanout.geometry import calculate_fanout_stub


# Public API
__all__ = [
    'generate_qfn_fanout',
    'main',
    # Types re-exported for external use
    'QFNLayout',
    'PadInfo',
    'FanoutStub',
]


def check_endpoint_spacing(stubs: List[FanoutStub], min_spacing: float) -> List[Tuple[int, int, float]]:
    """Check for endpoints that are too close together."""
    collisions = []
    for i, s1 in enumerate(stubs):
        for j, s2 in enumerate(stubs[i+1:], i+1):
            if s1.pad.net_id == s2.pad.net_id:
                continue
            dist = math.sqrt((s1.stub_end[0] - s2.stub_end[0])**2 +
                           (s1.stub_end[1] - s2.stub_end[1])**2)
            if dist < min_spacing:
                collisions.append((i, j, dist))
    return collisions


def _board_edge_model(pcb_data, clearance, board_edge_clearance):
    """Edge.Cuts keep-out for fanout copper (issue #288, picodvi's 0.04mm
    board-edge graze -- qfn_fanout had no edge model at all). Returns
    (edge_clear, rings, outer, cutouts); rings is empty when the board has no
    usable outline AND no bounds (then no edge test is possible)."""
    from check_drc import board_edge_geometry
    edge_clear = board_edge_clearance if board_edge_clearance > 0 else clearance
    rings, outer, cutouts = board_edge_geometry(pcb_data.board_info)
    if not rings and pcb_data.board_info.board_bounds:
        bx0, by0, bx1, by1 = pcb_data.board_info.board_bounds
        outer = [(bx0, by0), (bx1, by0), (bx1, by1), (bx0, by1)]
        rings, cutouts = [outer], []
    return edge_clear, rings, outer, cutouts


def _underpad_via_escape(footprint, pcb_data, pad_infos, layout, layer,
                         track_width, clearance, via_size, via_drill, grid_step,
                         allow_via_in_pad=False, board_edge_clearance=0.0):
    """Via-drop escape (issue #164): instead of a surface 45-degree fan, run a
    short stub from each pad to a through-via just past the pad edge and let
    signal routing pick the net up on an inner/back layer. This escapes a
    crowded fine-pitch edge where the *surface* is full (a neighbour pair on one
    side, a foreign track on the other) -- the case the surface fan cannot solve.

    "Foreign" copper is anything NOT on a net we're escaping right now, even if
    it sits on the SAME component (issue #161 reopen): a routed neighbour pair
    and a crossing track are usually on the chip's own other nets, so they must
    block the via even though they share the footprint. Each candidate via is
    obstacle-checked against foreign vias, foreign PADS and foreign TRACKS (the
    track check was the gap that let a via land on a neighbour's trace and short
    it -- issue #161); only the via's own net is exempt.

    Adjacent vias are staggered along the escape axis so two neighbours one pitch
    apart still clear each other. With `allow_via_in_pad` the via may sit on its
    own pad, so its candidate offsets are a MIX: on-pad positions (centre, then
    inward toward the chip) *and* off-pad positions (outward past the pad) -- a
    pad that can't escape one way may still escape the other. Without it the via
    stays clear of the pad body and only moves outward.

    Placement is greedy per side, but if the default stagger drops a pad the side
    is re-tried under alternative configurations -- reversed order, and per-pad
    direction biases (e.g. one leg back/inward, its neighbour forward/outward) --
    keeping whichever escapes the most pads (issue #161 follow-up). The default
    (forward, nearest-offset-first) is tried first, so when it already escapes
    every pad nothing changes. A pad with no clear offset under any configuration
    is dropped. Returns (tracks, vias, dropped_net_names)."""
    from obstacle_map import (build_base_obstacle_map, build_layer_map,
                              check_line_clearance, point_to_segment_distance)
    from bga_fanout.reroute import _seg_hits_pad
    from bga_fanout.geometry import clamp_via_to_pad
    from list_nets import fab_floor_ladder, fab_floor_min, warn_fab_escalation
    from routing_config import GridRouteConfig

    # Fab floors for the via-in-pad clamp (#202): when a chosen via sits ON its
    # pad, size it to the pad edge so it can't bulge into a neighbouring net. Pass
    # the active fab-tier ladder so the clamp escalates standard->advanced (#237).
    _copper = len(getattr(pcb_data.board_info, 'copper_layers', None) or []) or 4
    floors = fab_floor_ladder(_copper)
    clamp_n = floor_n = escalated_n = 0

    # Only the nets we're escaping right now are exempt from the obstacle map --
    # the chip's OTHER nets (a routed neighbour pair, a crossing track) must
    # block, so we don't exclude all of the footprint's nets (issue #161).
    fanned_nets = {pi.pad.net_id for pi in pad_infos}
    cfg = GridRouteConfig(layers=list(pcb_data.board_info.copper_layers or [layer]),
                          track_width=track_width, clearance=clearance)
    layer_map = build_layer_map(cfg.layers)
    obstacles = build_base_obstacle_map(pcb_data, cfg, nets_to_route=list(fanned_nets),
                                        extra_clearance=track_width / 2)
    obs_layer_idx = layer_map.get(layer)

    # Foreign obstacles, keyed by net so the via's OWN net is exempt at check
    # time. A through-via spans every copper layer, so foreign tracks on ANY
    # layer matter -- don't filter tracks by layer.
    foreign_vias = [(v.x, v.y, v.size, v.net_id) for v in pcb_data.vias]
    foreign_pads = [p for plist in pcb_data.pads_by_net.values() for p in plist]
    foreign_tracks = [(s.start_x, s.start_y, s.end_x, s.end_y, s.width, s.net_id)
                      for s in pcb_data.segments]

    pitch = layout.pad_pitch or 0.5
    need_cc = via_size + clearance              # min centre-to-centre, different nets
    stagger = (math.sqrt(max(0.0, need_cc * need_cc - pitch * pitch)) + 0.05
               if need_cc > pitch else 0.0)
    step = max(stagger, grid_step, 0.05)        # offset increment along the escape axis

    _edge_clear, _edge_rings, _edge_outer, _edge_cutouts = _board_edge_model(
        pcb_data, clearance, board_edge_clearance)
    from check_drc import _point_to_rings_distance as _pt_rings_dist
    from check_drc import _point_on_board as _pt_on_board

    def via_clears(vx, vy, net_id, placed):
        if _edge_rings:
            if not _pt_on_board(vx, vy, _edge_outer, _edge_cutouts):
                return False
            if _pt_rings_dist(vx, vy, _edge_rings) < via_size / 2 + _edge_clear - 1e-6:
                return False
        for fx, fy, fs, fn in foreign_vias:
            if fn == net_id:
                continue
            if math.hypot(vx - fx, vy - fy) < via_size / 2 + fs / 2 + clearance - 1e-6:
                return False
        for pad in foreign_pads:
            if pad.net_id == net_id:            # own net (incl. via-in-pad) is fine
                continue
            # Rotation-exact rectangle clearance (the disc test over-rejected an
            # elongated neighbour pad a pitch away). The via is through-all-layers,
            # so an SMD pad on any single layer still conflicts.
            if _seg_hits_pad(vx, vy, vx, vy, pad, margin=via_size / 2 + clearance - 1e-6):
                return False
        for sx0, sy0, sx1, sy1, sw, sn in foreign_tracks:
            if sn == net_id:
                continue
            if point_to_segment_distance(vx, vy, sx0, sy0, sx1, sy1) \
                    < via_size / 2 + sw / 2 + clearance - 1e-6:
                return False
        for qx, qy, qn in placed:
            floor = (via_size + clearance) if qn != net_id else (via_size * 0.5)
            if math.hypot(vx - qx, vy - qy) < floor - 1e-6:
                return False
        return True

    def snap(v):
        return round(v / grid_step) * grid_step if grid_step > 0 else v

    def _onpad(mode):
        # On-pad (via-in-pad) offsets along the escape axis, ordered by `mode`:
        # 'in' sweeps inward (toward the chip) first; 'near' alternates nearest
        # first. 0 == via centred on the pad.
        seq = [0.0]
        if mode == 'in':
            seq += [-k * step for k in range(1, 9)] + [k * step for k in range(1, 9)]
        else:                                   # 'near'
            for k in range(1, 9):
                seq += [k * step, -k * step]
        return seq

    def candidate_offsets(pad_width, mode):
        # Off-pad outward offsets always clear the pad body. With via-in-pad we
        # ALSO offer on-pad offsets and mix the two: 'out' prefers off-pad
        # outward then falls back to on-pad; 'near'/'in' prefer on-pad then fall
        # back to off-pad outward.
        base = pad_width / 2 + via_size / 2 + clearance
        outward = [base + k * step for k in range(0, 9)]
        if not allow_via_in_pad:
            return outward
        if mode == 'out':
            return outward + _onpad('near')
        return _onpad(mode) + outward

    def place_pin(pi, mode, placed):
        ex, ey = pi.escape_direction
        px, py = pi.pad.global_x, pi.pad.global_y
        for d in candidate_offsets(pi.pad_width, mode):
            vx, vy = snap(px + ex * d), snap(py + ey * d)
            stub_ok = (obs_layer_idx is None or
                       check_line_clearance(obstacles, px, py, vx, vy, obs_layer_idx, cfg))
            if stub_ok and via_clears(vx, vy, pi.pad.net_id, placed):
                return (vx, vy)
        return None

    def trial(pis, order, mode_fn, committed):
        # Greedily place a side under one configuration; return idx->pos|None.
        placed = list(committed)
        results = {}
        for idx in order:
            pos = place_pin(pis[idx], mode_fn(idx), placed)
            results[idx] = pos
            if pos is not None:
                placed.append((pos[0], pos[1], pis[idx].pad.net_id))
        return results

    # Stagger configurations, tried in order; the most-escaped wins, ties keep
    # the earliest. The default (forward, nearest-offset-first) is first, so a
    # side every pad already escapes is unchanged. Alternatives only matter when
    # a pad failed: reversed order, and per-pad direction biases that stagger one
    # leg back/inward while its neighbour goes forward/outward. Direction biases
    # only do anything with via-in-pad (otherwise every mode is the outward
    # ladder), so skip them then.
    configs = [('fwd', lambda i: 'near'), ('rev', lambda i: 'near')]
    if allow_via_in_pad:
        configs += [
            ('fwd', lambda i: 'out' if i % 2 == 0 else 'in'),
            ('fwd', lambda i: 'in' if i % 2 == 0 else 'out'),
            ('fwd', lambda i: 'in'),
            ('fwd', lambda i: 'out'),
        ]
    # Test/debug hook: pin the search to the default config to measure how many
    # pads the alternative-stagger search rescues. Not a user-facing option.
    if os.environ.get("QFN_UNDERPAD_NO_ALT_STAGGER"):
        configs = configs[:1]

    tracks, vias, dropped = [], [], []
    placed_global = []
    by_side = defaultdict(list)
    for pi in pad_infos:
        by_side[pi.side].append(pi)

    n_alt = 0
    for side, pis in by_side.items():
        pis.sort(key=lambda pi: (pi.pad.global_x, pi.pad.global_y))
        order_fwd = list(range(len(pis)))
        best, best_n, best_ci = None, -1, 0
        for ci, (order_name, mode_fn) in enumerate(configs):
            order = order_fwd if order_name == 'fwd' else order_fwd[::-1]
            results = trial(pis, order, mode_fn, placed_global)
            n_placed = sum(1 for v in results.values() if v is not None)
            if n_placed > best_n:
                best, best_n, best_ci = results, n_placed, ci
            if best_n == len(pis):
                break
        if best_ci != 0:
            n_alt += 1
        # Commit the winning configuration.
        for idx in order_fwd:
            pi = pis[idx]
            pos = best.get(idx)
            if pos is None:
                dropped.append(pi.pad.net_name)
                continue
            vx, vy = pos
            placed_global.append((vx, vy, pi.pad.net_id))
            px, py = pi.pad.global_x, pi.pad.global_y
            # Zero-length stub (via centred on the pad) needs no track.
            # The connecting stub bridges the pad to the via, so it must live on
            # the pad's own copper layer (the footprint mount layer) -- the
            # through-via then carries the net down to the `--layer` target.
            # Putting it on `layer` (an inner/back escape layer) would float it
            # above the pad and leave the pad disconnected (issue #195).
            if math.hypot(vx - px, vy - py) > POSITION_TOLERANCE:
                tracks.append({'start': (px, py), 'end': (vx, vy),
                               'width': track_width, 'layer': footprint.layer,
                               'net_id': pi.pad.net_id})
                v_size, v_drill = via_size, via_drill   # off-pad via: not in a pad
            else:
                # via-in-pad: clamp to the pad edge so it never bulges past it (#202)
                v_size, v_drill, status, rung = clamp_via_to_pad(via_size, via_drill, pi.pad, floors)
                if status == 'clamped':
                    clamp_n += 1
                elif status == 'floor':
                    floor_n += 1
                if rung > 0:
                    escalated_n += 1
            vias.append({'x': vx, 'y': vy, 'size': v_size, 'drill': v_drill,
                         'layers': ['F.Cu', 'B.Cu'], 'net_id': pi.pad.net_id})

    print(f"  Underpad via-drop: {len(vias)} vias placed, {len(dropped)} dropped "
          f"(pitch {pitch:.2f}, via {via_size:.2f}, stagger {stagger:.3f} mm"
          f"{', via-in-pad' if allow_via_in_pad else ''}"
          f"{f', {n_alt} side(s) used an alternative stagger' if n_alt else ''})")
    if dropped:
        print(f"    dropped (no clear via offset): {dropped}")
    if clamp_n:
        print(f"    clamped {clamp_n} via-in-pad(s) to fit their pad edge (#202)")
    if escalated_n:
        warn_fab_escalation(f"{escalated_n} via-in-pad(s) (sub-0.45mm pads)")
    if floor_n:
        print(f"    WARNING: {floor_n} pad(s) smaller than the fab via floor "
              f"({fab_floor_min(_copper)['via_diameter']:.2f}mm dia); via held at the "
              f"floor and still bulges past the pad edge")
    return tracks, vias, dropped


def generate_qfn_fanout(footprint: Footprint,
                        pcb_data: PCBData,
                        net_filter: Optional[List[str]] = None,
                        layer: str = "F.Cu",
                        track_width: float = 0.1,
                        extension: float = 0.1,
                        clearance: float = 0.1,
                        grid_step: float = 0.0,
                        escape_method: str = "stub",
                        via_size: float = 0.45,
                        via_drill: float = 0.25,
                        allow_via_in_pad: bool = False,
                        board_edge_clearance: float = 0.0) -> Tuple[List[Dict], List[Dict], List[str]]:
    """
    Generate QFN fanout tracks for a footprint.

    Creates two-segment stubs:
    1. Straight segment perpendicular to chip edge
    2. 45 degree segment fanning outward from center

    Edge pads get short straight (just past pad) + long 45 degree for maximum fan.
    Center pads get full straight (no 45 degree) since already separated.

    Args:
        footprint: The QFN/QFP footprint
        pcb_data: Full PCB data
        net_filter: Optional list of net patterns to include
        layer: Routing layer (default F.Cu)
        track_width: Width of fanout tracks
        extension: Extension past pad edge before bend (mm)

    Returns:
        (tracks, vias, failed_nets) - tracks are the segments, vias is empty.
        failed_nets is the deduplicated list of net names whose stubs landed
        too close to another net's stub (endpoint spacing < track_width +
        extension); those tracks are still emitted but flagged as failing
        clearance so the GUI can surface them.
    """
    layout = analyze_qfn_layout(footprint)
    if layout is None:
        print(f"Warning: {footprint.reference} doesn't appear to be a QFN/QFP")
        return [], [], []

    # Fab-floor clamp (issue #223): an escape stub thinner than the board's
    # minimum manufacturable track width is un-routable at the stated fab class
    # (usb_sniffer's /T_USB_* bus emitted at 0.100mm against a 0.127mm 2-layer
    # floor -> a whole bus of TRACK-WIDTH violations). The width is a parameter,
    # not a search outcome, so clamp it up to the fab floor here -- mirroring the
    # clamp route.py / check_drc.py already apply.
    from list_nets import fab_floors
    ncu = len(pcb_data.board_info.copper_layers) if pcb_data.board_info.copper_layers else 2
    fab_min_track = fab_floors(ncu)['track_width']
    if track_width < fab_min_track - 1e-9:
        print(f"  Track width {track_width:.4f}mm is below the {ncu}-layer fab "
              f"floor {fab_min_track:.4f}mm - clamping escape stubs up (issue #223)")
        track_width = fab_min_track

    # Sanity-check pad geometry before escaping: overlapping same-footprint pads
    # mean the pad rotation/size is modelled wrong, so the stubs would be placed
    # across neighbouring pads (issue: rotated-package fanout). Warn loudly.
    from check_pads import find_pad_overlaps
    _ov = find_pad_overlaps(pcb_data, component=footprint.reference)
    if _ov:
        print(f"  WARNING: {footprint.reference} has {len(_ov)} overlapping "
              f"different-net pad pair(s) - pad geometry looks wrong, fanout "
              f"stubs may cross pads. Run: python3 check_pads.py <board> "
              f"--component {footprint.reference}")

    print(f"QFN/QFP Layout Analysis for {footprint.reference}:")
    print(f"  Center: ({layout.center_x:.2f}, {layout.center_y:.2f})")
    print(f"  Bounding box: X[{layout.min_x:.2f}, {layout.max_x:.2f}], Y[{layout.min_y:.2f}, {layout.max_y:.2f}]")
    print(f"  Size: {layout.width:.2f} x {layout.height:.2f} mm")
    print(f"  Detected pad pitch: {layout.pad_pitch:.2f} mm")
    print(f"  Edge tolerance: {layout.edge_tolerance:.2f} mm")
    print(f"  Stub length: pad_width / 2 + extension (clears pad before bend)")
    print(f"  Layer: {layer}")

    # Analyze all pads
    pad_infos: List[PadInfo] = []
    side_counts = defaultdict(int)

    for pad in footprint.pads:
        if not pad.net_name or pad.net_id == 0:
            continue

        # Skip unconnected nets (KiCad pins not connected in schematic)
        if pad.net_name.lower().startswith('unconnected-'):
            continue

        if net_filter and not matches_net_filter(pad.net_name, net_filter):
            continue

        pad_info = analyze_pad(pad, layout)
        if pad_info.side == 'center':
            continue  # Skip center/EP pads

        pad_infos.append(pad_info)
        side_counts[pad_info.side] += 1

    print(f"  Found {len(pad_infos)} pads to fanout:")
    for side, count in sorted(side_counts.items()):
        print(f"    {side}: {count} pads")

    # Show sample pad geometry
    if pad_infos:
        sample = pad_infos[0]
        print(f"  Sample pad geometry: {sample.pad_length:.2f} x {sample.pad_width:.2f} mm")

    if not pad_infos:
        return [], [], []

    # Via-drop / underpad escape (issue #164): drop a through-via just past each
    # pad and let signal routing pick the net up on an inner layer, for crowded
    # fine-pitch edges where the surface fan has no room.
    if escape_method == "underpad":
        print(f"  Escape method: underpad (via-drop), via {via_size:.2f}/{via_drill:.2f} mm"
              f"{', allow via-in-pad' if allow_via_in_pad else ''}")
        return _underpad_via_escape(footprint, pcb_data, pad_infos, layout, layer,
                                    track_width, clearance, via_size, via_drill, grid_step,
                                    allow_via_in_pad=allow_via_in_pad,
                                    board_edge_clearance=board_edge_clearance)

    # Build stubs
    stubs: List[FanoutStub] = []

    # Max diagonal length for corner pads = chip_width / 3
    max_diagonal_length = max(layout.width, layout.height) / 3

    # Per-side outermost pad offset (issue #200): normalizes the fan-angle ramp
    # so the corner pad on each side reaches a true 45 deg (the last pad is
    # inset from the bbox corner, so a bbox-half normalization would stop short).
    side_max_off: Dict[str, float] = {}
    for pi in pad_infos:
        ex, ey = pi.escape_direction
        tan_x, tan_y = -ey, ex
        off = abs((pi.pad.global_x - layout.center_global_x) * tan_x +
                  (pi.pad.global_y - layout.center_global_y) * tan_y)
        side_max_off[pi.side] = max(side_max_off.get(pi.side, 0.0), off)

    for pad_info in pad_infos:
        # Straight stub length = pad_width / 2 + extension (to clear the pad before bending)
        # pad_width is the dimension perpendicular to the chip edge (escape direction)
        straight_length = pad_info.pad_width / 2 + extension
        corner_pos, stub_end = calculate_fanout_stub(
            pad_info, layout, straight_length, max_diagonal_length, grid_step,
            angle_ref_off=side_max_off.get(pad_info.side, 0.0)
        )

        stub = FanoutStub(
            pad=pad_info.pad,
            pad_pos=(pad_info.pad.global_x, pad_info.pad.global_y),
            corner_pos=corner_pos,
            stub_end=stub_end,
            side=pad_info.side,
            layer=layer
        )
        stubs.append(stub)

    # Foreign-pad clearance (issue #123). The QFN fan emits each stub blind to
    # other components' pads, so a stub on a fine-pitch part routed out toward a
    # neighbouring passive grazes it within clearance (PAD-SEGMENT). Mirror the
    # bga_fanout escape clearing: shorten the 45 fan inward to clear the pad
    # (connectivity-neutral - the straight segment still escapes the chip), and
    # if even the straight escape itself grazes a pad, drop that stub and warn.
    from bga_fanout.reroute import _seg_hits_pad
    from obstacle_map import build_base_obstacle_map, build_layer_map, check_line_clearance
    from routing_config import GridRouteConfig
    margin = clearance + track_width / 2

    # Full obstacle map so a stub is checked against foreign TRACKS and VIAS too,
    # not just foreign pads (issue #149 part 2). The part's own nets are excluded
    # (nets_to_route) so same-net taps stay legal; extra_clearance = track half so
    # the cell test is a clearance test. Checked at stub-creation time (below), so
    # a stub/fan that would extend into a foreign obstacle is shortened or dropped.
    _obs_cfg = GridRouteConfig(layers=list(pcb_data.board_info.copper_layers or [layer]),
                               track_width=track_width, clearance=clearance)
    _obs_layer_map = build_layer_map(_obs_cfg.layers)
    _fanned_net_ids = [p.net_id for p in footprint.pads if p.net_id]
    _obstacles = build_base_obstacle_map(pcb_data, _obs_cfg, nets_to_route=_fanned_net_ids,
                                         extra_clearance=track_width / 2)
    _obs_layer_idx = _obs_layer_map.get(layer)
    # Window the foreign-pad scan by the actual STUB extent (every stub endpoint),
    # not the part's pad bbox: on large packages (LQFP) a straight escape runs
    # several mm past the pad bbox and grazes a nearby foreign component pad (a
    # connector/header) that a pad-bbox+3mm window misses -- and the grid obstacle
    # map is too coarse to catch a sub-grid graze (issue #214). Each pad is
    # included if its OWN footprint can reach the stub span within `margin`, so the
    # window is correct for large pads too.
    #
    # The part's OWN pads are included too (issue #356): the obstacle map excludes
    # every fanned net, and this scan used to exclude the whole footprint, so an
    # escape threading a dense pad field (AQFN inner grid, hex_gateway U1) shipped
    # 0.006mm from a NEIGHBOUR pad of the same part with no check at all. The
    # per-stub `pad.net_id == net_id` skip below already exempts the stub's own
    # pad(s); every other-net pad -- own part or not -- is real foreign copper.
    _sxs = [pt[0] for s in stubs for pt in (s.pad_pos, s.corner_pos, s.stub_end)]
    _sys = [pt[1] for s in stubs for pt in (s.pad_pos, s.corner_pos, s.stub_end)]
    if _sxs:
        _lo_x, _hi_x = min(_sxs), max(_sxs)
        _lo_y, _hi_y = min(_sys), max(_sys)

        def _pad_near(p):
            r = 0.5 * math.hypot(p.size_x or 0.0, p.size_y or 0.0) + margin
            return (_lo_x - r <= p.global_x <= _hi_x + r
                    and _lo_y - r <= p.global_y <= _hi_y + r)

        foreign_pads = [p for plist in pcb_data.pads_by_net.values() for p in plist
                        if _pad_near(p)]
    else:
        foreign_pads = []

    # EXISTING copper of the part's OWN nets, checked geometrically per stub:
    # the obstacle map excludes ALL fanned nets (so a stub may touch its own
    # net's copper), but that also hid a NEIGHBOURING chip's already-fanned
    # stubs on a DIFFERENT fanned net -- two facing QFNs fanned back-to-back
    # left foreign-net stub crossings in the gap between them (issue #257,
    # usb_sniffer /T_USB_CLK x /T_USB_NXT). Only copper present BEFORE this
    # fanout is in pcb_data, so this never blocks the part's own new stubs.
    from geometry_utils import segment_to_segment_distance, point_to_segment_distance
    _fanned_set = set(_fanned_net_ids)
    _fanned_existing_segs = [s for s in pcb_data.segments if s.net_id in _fanned_set]
    _fanned_existing_vias = [v for v in pcb_data.vias if v.net_id in _fanned_set]

    # Board-edge keep-out (issue #288): folded into _seg_grazes so an
    # edge-violating straight escape is dropped and an edge-violating 45 fan is
    # shortened by the existing recovery loop below, same as a foreign-pad graze.
    _edge_clear, _edge_rings, _edge_outer, _edge_cutouts = _board_edge_model(
        pcb_data, clearance, board_edge_clearance)
    from check_drc import _segment_to_rings_distance as _seg_rings_dist
    from check_drc import _point_on_board as _pt_on_board

    def _seg_hits_edge(p1, p2):
        if not _edge_rings:
            return False
        for (x, y) in (p1, p2):
            if not _pt_on_board(x, y, _edge_outer, _edge_cutouts):
                return True
        return _seg_rings_dist(p1[0], p1[1], p2[0], p2[1], _edge_rings) \
            < track_width / 2 + _edge_clear - 1e-6

    def _seg_grazes(p1, p2, net_id):
        # Board edge first (issue #288) - independent of net.
        if _seg_hits_edge(p1, p2):
            return True
        # Foreign tracks / vias via the shared obstacle map (issue #149 part 2).
        if _obs_layer_idx is not None and not check_line_clearance(
                _obstacles, p1[0], p1[1], p2[0], p2[1], _obs_layer_idx, _obs_cfg):
            return True
        # Foreign pads, geometric + rotation-exact (issue #123).
        for pad in foreign_pads:
            if pad.net_id == net_id:
                continue
            if pad.drill <= 0 and layer not in (pad.layers or []):
                continue
            if _seg_hits_pad(p1[0], p1[1], p2[0], p2[1], pad, margin=margin):
                return True
        # Existing copper of the part's other fanned nets (issue #257).
        for s in _fanned_existing_segs:
            if s.net_id == net_id or s.layer != layer:
                continue
            if segment_to_segment_distance(
                    p1[0], p1[1], p2[0], p2[1],
                    s.start_x, s.start_y, s.end_x, s.end_y) \
                    < s.width / 2 + track_width / 2 + clearance - 1e-6:
                return True
        for v in _fanned_existing_vias:
            if v.net_id == net_id:
                continue
            if point_to_segment_distance(v.x, v.y, p1[0], p1[1], p2[0], p2[1]) \
                    < v.size / 2 + track_width / 2 + clearance - 1e-6:
                return True
        return False

    qfn_dropped: List[str] = []
    n_short = 0
    kept_stubs: List[FanoutStub] = []
    for stub in stubs:
        nid = stub.net_id
        if _seg_grazes(stub.pad_pos, stub.corner_pos, nid):
            # The perpendicular escape itself hits a foreign pad - cannot fan out.
            if stub.pad.net_name and stub.pad.net_name not in qfn_dropped:
                qfn_dropped.append(stub.pad.net_name)
            continue
        if _seg_grazes(stub.corner_pos, stub.stub_end, nid):
            # Shorten the 45 fan toward the corner until it clears (worst case
            # collapse it entirely - the straight escape is already clear).
            cx, cy = stub.corner_pos
            ex, ey = stub.stub_end
            new_end = stub.corner_pos
            for i in range(1, 9):
                t = 1.0 - i / 9.0  # walk inward from current tip toward corner
                cand = (cx + (ex - cx) * t, cy + (ey - cy) * t)
                if not _seg_grazes(stub.corner_pos, cand, nid):
                    new_end = cand
                    break
            stub.stub_end = new_end
            n_short += 1
        kept_stubs.append(stub)
    if n_short or qfn_dropped:
        print(f"  Pad-clearance: shortened {n_short} fan(s); dropped {len(qfn_dropped)} "
              f"stub(s) grazing a foreign pad (issue #123)")
    stubs = kept_stubs

    # Generate tracks - two segments per stub
    tracks = []
    for stub in stubs:
        # Segment 1: Straight from pad to corner
        dx1 = abs(stub.corner_pos[0] - stub.pad_pos[0])
        dy1 = abs(stub.corner_pos[1] - stub.pad_pos[1])
        if dx1 > POSITION_TOLERANCE or dy1 > POSITION_TOLERANCE:
            tracks.append({
                'start': stub.pad_pos,
                'end': stub.corner_pos,
                'width': track_width,
                'layer': stub.layer,
                'net_id': stub.net_id
            })

        # Segment 2: 45 degree from corner to end
        dx2 = abs(stub.stub_end[0] - stub.corner_pos[0])
        dy2 = abs(stub.stub_end[1] - stub.corner_pos[1])
        if dx2 > POSITION_TOLERANCE or dy2 > POSITION_TOLERANCE:
            tracks.append({
                'start': stub.corner_pos,
                'end': stub.stub_end,
                'width': track_width,
                'layer': stub.layer,
                'net_id': stub.net_id
            })

    print(f"  Generated {len(tracks)} track segments ({len(stubs)} stubs x 2 segments)")

    # Fine-pitch escape warning (issue #97): the corner stubs reach a full
    # 45 deg fan, so the tightest adjacent pair sits about pitch/sqrt(2) apart
    # (inner stubs diverge wider, issue #200). At common defaults (clearance
    # 0.25) the router cannot launch from or pass between these stubs and every
    # net fails with 'no rippable blockers found'. Tell the user the workable
    # parameters up front instead. (pitch/sqrt2 is the conservative lower bound.)
    if layout.pad_pitch and layout.pad_pitch < 0.8:
        lateral = layout.pad_pitch / math.sqrt(2)
        # Escape at the stub's own width: route_track/2 + clearance +
        # stub_track/2 must fit in `lateral`.
        max_clear = lateral - track_width
        # One 0.05 grid step of margin for quantization, rounded down to 0.05,
        # capped at 0.15 (the combination verified to escape 0.5 mm LQFP fans).
        suggest_clear = min(max(0.05, int((max_clear - 0.05) / 0.05) * 0.05), 0.15)
        print(f"  NOTE: {layout.pad_pitch:.2f} mm pitch keeps adjacent fan stubs only "
              f"{lateral:.3f} mm apart (pitch/sqrt2).")
        print(f"  Routing these nets needs clearance below {max_clear:.2f} mm (with "
              f"{track_width:.2f} mm track) plus grid-quantization margin; the "
              f"default 0.25 clearance / 0.1 grid will fail to escape.")
        print(f"  Suggested: route.py --grid-step 0.05 --clearance {suggest_clear:.2f} "
              f"--track-width {track_width:.2f} for this component's nets.")

    # Validate endpoint spacing
    min_spacing = track_width + extension
    collisions = check_endpoint_spacing(stubs, min_spacing)

    failed_nets: List[str] = list(qfn_dropped)
    if collisions:
        print(f"  WARNING: {len(collisions)} endpoint pairs too close!")
        for i, j, dist in collisions[:5]:
            print(f"    {stubs[i].pad.net_name} <-> {stubs[j].pad.net_name}: {dist:.3f}mm")
        print(f"  Consider increasing extension")
        # Collect the deduplicated set of nets involved in any collision -
        # these are the "failed" nets the GUI surfaces.
        seen = set()
        for i, j, _dist in collisions:
            for name in (stubs[i].pad.net_name, stubs[j].pad.net_name):
                if name and name not in seen:
                    seen.add(name)
                    failed_nets.append(name)
    else:
        print(f"  Validated: No endpoint collisions")

    return tracks, [], failed_nets


def main():
    """Run QFN fanout generation."""
    import argparse

    parser = argparse.ArgumentParser(description='Generate QFN/QFP fanout routing')
    parser.add_argument('pcb', help='Input PCB file')
    parser.add_argument('--output', '-o', default='kicad_files/qfn_fanout_test.kicad_pcb',
                        help='Output PCB file')
    parser.add_argument('--component', '-c', default=None,
                        help='Component reference (auto-detected if not specified)')
    parser.add_argument('--layer', '-l', default=None,
                        help='Routing layer (default: the layer the component '
                             'is mounted on)')
    parser.add_argument('--width', '-w', type=float, default=0.1,
                        help='Track width in mm')
    parser.add_argument('--extension', type=float, default=0.1,
                        help='Extension past pad edge before bend (mm)')
    parser.add_argument('--clearance', type=float, default=0.1,
                        help='Min clearance to other-net pads (mm); stubs that '
                             'would graze a foreign pad are shortened or dropped')
    parser.add_argument('--nets', '-n', nargs='*',
                        help='Net patterns to include')
    parser.add_argument('--grid-step', type=float, default=0.1,
                        help='Routing grid step in mm (default: 0.1). Fanned stub ends are '
                             'snapped to this grid so the router gets on-grid terminals (issue '
                             '#149); MATCH the --grid-step you pass to route.py.')
    parser.add_argument('--escape-method', choices=['stub', 'underpad'], default='stub',
                        help="'stub' (default) = surface 45-degree fan; 'underpad' = drop a "
                             "through-via just past each pad and escape on an inner layer "
                             "(issue #164), for crowded fine-pitch edges where the surface is full.")
    parser.add_argument('--via-size', type=float, default=0.45,
                        help='Underpad escape via outer diameter (mm, default 0.45)')
    parser.add_argument('--via-drill', type=float, default=0.25,
                        help='Underpad escape via drill diameter (mm, default 0.25)')
    parser.add_argument('--board-edge-clearance', type=float, default=0.0,
                        help='Min clearance from stub/via copper to the Edge.Cuts '
                             'outline in mm (default 0 = use --clearance). Stubs '
                             'that would graze the board edge are shortened or '
                             'dropped; underpad escape vias near the edge are '
                             'rejected (issue #288).')
    parser.add_argument('--allow-via-in-pad', action='store_true',
                        help='Underpad escape: let the escape via overlap its OWN pad '
                             '(via-in-pad), so a via boxed in on the outward side can '
                             'stagger inward toward the chip instead of being dropped. '
                             'The via still must clear other-net pads, vias and tracks.')
    from fab_tiers import (add_fab_tier_args, fab_tier_from_args, set_default_fab_tier,
                           enforce_fab_floors, count_copper_layers_in_file)
    add_fab_tier_args(parser)
    # #381 D8: define --no-fix-drc-settings (and the shared DRC-fix flags) that
    # the writeback below already reads via getattr but that was never declared.
    # store_true default keeps no_fix_drc_settings=False => writeback ON =>
    # identical behavior for existing commands.
    from fix_kicad_drc_settings import add_drc_fix_args
    add_drc_fix_args(parser)
    args = parser.parse_args()
    set_default_fab_tier(*fab_tier_from_args(args))
    enforce_fab_floors(
        count_copper_layers_in_file(args.pcb),
        track_width=getattr(args, 'track_width', None),
        clearance=getattr(args, 'clearance', None),
        via_size=getattr(args, 'via_size', None),
        via_drill=getattr(args, 'via_drill', None),
        hole_to_hole_clearance=getattr(args, 'hole_to_hole_clearance', None))

    print(f"Parsing {args.pcb}...")
    pcb_data = parse_kicad_pcb(args.pcb)

    # Auto-detect QFN/QFP component if not specified
    if args.component is None:
        qfn_components = find_components_by_type(pcb_data, 'QFN')
        if not qfn_components:
            qfn_components = find_components_by_type(pcb_data, 'QFP')
        if qfn_components:
            args.component = qfn_components[0].reference
            print(f"Auto-detected QFN/QFP component: {args.component}")
            if len(qfn_components) > 1:
                print(f"  (Other QFN/QFPs found: {[fp.reference for fp in qfn_components[1:]]})")
        else:
            print("Error: No QFN/QFP components found in PCB")
            print(f"Available components: {list(pcb_data.footprints.keys())[:20]}...")
            return 1

    if args.component not in pcb_data.footprints:
        print(f"Error: Component {args.component} not found")
        print(f"Available: {list(pcb_data.footprints.keys())[:20]}...")
        return 1

    footprint = pcb_data.footprints[args.component]
    print(f"\nFound {args.component}: {footprint.footprint_name}")
    print(f"  Position: ({footprint.x:.2f}, {footprint.y:.2f})")
    print(f"  Rotation: {footprint.rotation}deg")
    print(f"  Pads: {len(footprint.pads)}")

    # Default the stub layer to the component's mounted layer (issue #96:
    # B.Cu-mounted parts silently got F.Cu stubs floating over their pads,
    # and route.py then reported their nets routed while electrically open).
    layer = args.layer
    if layer is None:
        layer = footprint.layer or 'F.Cu'
        print(f"  Layer: {layer} (from footprint)")
    elif footprint.layer and layer != footprint.layer:
        print(f"  WARNING: --layer {layer} differs from {args.component}'s "
              f"mounted layer {footprint.layer} - stubs will NOT touch the "
              f"SMD pads unless this is intentional")

    tracks, vias, _failed_nets = generate_qfn_fanout(
        footprint,
        pcb_data,
        net_filter=args.nets,
        layer=layer,
        track_width=args.width,
        extension=args.extension,
        clearance=args.clearance,
        grid_step=args.grid_step,
        board_edge_clearance=args.board_edge_clearance,
        escape_method=args.escape_method,
        via_size=args.via_size,
        via_drill=args.via_drill,
        allow_via_in_pad=args.allow_via_in_pad
    )

    if tracks:
        print(f"\nWriting {len(tracks)} tracks to {args.output}...")
        net_id_to_name = {nid: net.name for nid, net in pcb_data.nets.items()}
        add_tracks_and_vias_to_pcb(args.pcb, args.output, tracks, vias,
                                   net_id_to_name=net_id_to_name)
        print("Done!")
    else:
        print("\nNo fanout tracks generated")
        # Still produce the output file (board unchanged) so a multi-step
        # pipeline can continue - otherwise a fanout that finds nothing to do
        # (e.g. the component is already fanned on a retry) leaves the next step
        # with no input file.
        if getattr(args, 'output', None):
            from pcb_io_utils import passthrough_copy
            passthrough_copy(args.pcb, args.output)
            print(f"Wrote board through to {args.output} (unchanged)")

    # Structured summary + post-fanout DRC so downstream tooling (plan-pcb-routing
    # skill, stress harness) can detect when the escape left sub-clearance grazes
    # behind even though every pad escaped -- e.g. the 45-degree escape stubs of two
    # adjacent pads of a 0.4mm-pitch diff pair clipping at the wrist (issue #179).
    # The planner uses drc_grazes to retry the fanout with a thinner --width (and,
    # for the underpad method, a smaller via) toward the fab floor until it's clean.
    # Mirrors bga_fanout (#130/#122). Best-effort: a DRC hiccup must never fail the
    # fanout. drc_grazes is graded at --clearance.
    import json as _json
    escaped_net_ids = {t['net_id'] for t in tracks if t.get('net_id') is not None}
    unescaped = sorted(set(_failed_nets))
    escaped = len(escaped_net_ids)
    requested = escaped + len(unescaped)
    drc_grazes = {}
    out_path = getattr(args, 'output', None)
    if out_path:
        try:
            import io as _io, contextlib as _cl
            from check_drc import run_drc as _run_drc
            with _cl.redirect_stdout(_io.StringIO()):  # keep JSON_SUMMARY output clean
                _viols = _run_drc(out_path, clearance=args.clearance,
                                  quiet=True, max_print=0, check_sizes=False)
            _by = {}
            for _v in _viols:
                _by[_v['type']] = _by.get(_v['type'], 0) + 1
            drc_grazes = {
                'pad_via': _by.get('pad-via', 0),
                'via_segment': _by.get('via-segment', 0),
                'pad_segment': _by.get('pad-segment', 0),
                'segment_segment': _by.get('segment-segment', 0),
                'total': len(_viols),
            }
        except Exception as _e:
            drc_grazes = {'error': str(_e)}

    # Establish this fanout's clearance floor in the output .kicad_pro so the next
    # pipeline step and check_drc grade at the clearance the fanout used -- only
    # lowers, never tightens (issue #160).
    import clearance_ledger as _cl
    eff_clearance = _cl.effective(args.clearance)
    if out_path and os.path.isfile(out_path) \
            and not getattr(args, 'no_fix_drc_settings', False):
        try:
            from fix_kicad_drc_settings import fix_project_for_output
            fix_project_for_output(
                out_path, input_pcb=args.pcb,
                clearance=eff_clearance,
                track_width=args.width,
                via_diameter=getattr(args, 'via_size', None),
                via_drill=getattr(args, 'via_drill', None),
                clamp_nondefault_netclasses=not getattr(args, 'no_clamp_netclasses', False))
        except Exception as _e:
            print(f"  (skipped DRC-settings fix: {_e})")
    summary = {
        'component': args.component,
        'requested': requested,
        'escaped': escaped,
        'failed': len(unescaped),
        'unescaped_nets': unescaped,
        'clearance': args.clearance,
        'track_width': args.width,
        'escape_method': args.escape_method,
        'via_size': args.via_size,
        'via_drill': args.via_drill,
        'layer': layer,
        # grazes graded at --clearance; 'total' counts ALL DRC violations on the
        # output, the segment_segment/via_*/pad_* keys are the fanout-relevant classes.
        'drc_grazes': drc_grazes,
        # Smallest copper clearance any step actually routed at; downstream steps
        # and check_drc grade the board at this floor.
        'min_clearance_used': eff_clearance,
    }
    print(f"JSON_SUMMARY: {_json.dumps(summary)}")

    return 0


if __name__ == '__main__':
    exit(main())
