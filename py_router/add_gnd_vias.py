"""
GND via placement for single-ended routing.

Adds GND vias near signal vias for return current paths.
"""
from __future__ import annotations

import math
from typing import List, Dict

import fab_tiers
from kicad_parser import PCBData, Via, pad_drill_capsule
from geometry_utils import point_to_segment_distance
from routing_config import GridRouteConfig, GridCoord
import rust_alloc  # noqa: F401  # issue #419: set MIMALLOC_PURGE_DELAY before grid_router loads
from grid_router import GridObstacleMap


def add_gnd_vias_to_existing_board(
    pcb_data: PCBData,
    gnd_net_name: str,
    gnd_via_distance: float,
    config: GridRouteConfig,
    obstacles: GridObstacleMap,
    coord: GridCoord
) -> List[Via]:
    """
    Add GND vias near existing signal vias in a PCB that don't already have a nearby GND via.

    This function works with existing board vias, not routing results.

    Args:
        pcb_data: PCB data with existing vias and pads
        gnd_net_name: Net name for GND (e.g., "GND")
        gnd_via_distance: Maximum distance to place GND via from signal via (mm)
        config: Routing config with via size, drill, clearance, layers
        obstacles: Obstacle map built from the board
        coord: Grid coordinate converter

    Returns:
        List of new GND Via objects to add
    """
    # Find GND net ID. An exact name match first (so an explicit --gnd-via-net is
    # honoured verbatim), then the GND-family resolver.
    from net_queries import (resolve_gnd_net_id, resolve_return_net_id,
                             resolve_ground_domains, describe_ground_domains)

    # An empty gnd_net_name is AUTO: resolve per signal from its own ground
    # domain (the default, #489 §5). A NAMED net pins every return via to it.
    explicit_name = (gnd_net_name or "").strip() or None

    gnd_net_id = None
    if explicit_name:
        for net_id, net in pcb_data.nets.items():
            if net.name == explicit_name:
                gnd_net_id = net_id
                break
    if gnd_net_id is None:
        gnd_net_id, resolved_name = resolve_gnd_net_id(pcb_data, explicit_name)
        if gnd_net_id is not None and explicit_name:
            print(f"GND net '{explicit_name}' not matched exactly; using "
                  f"'{resolved_name}' for return vias")

    if gnd_net_id is None:
        target = explicit_name or "GND family"
        print(f"Warning: GND net '{target}' not found, skipping GND via placement")
        return []

    # Split-ground boards (#489 §5): every signal via used to be stitched to ONE
    # board-global ground, which can bridge the very domains the split exists to
    # separate. Each via now returns to its OWN net's domain; single-domain boards
    # resolve exactly as before.
    domains = {nid: refs for nid, refs in resolve_ground_domains(pcb_data).items() if refs}
    note = describe_ground_domains(pcb_data, explicit_name)
    if note:
        print(note)

    def _return_net_for(via) -> int:
        if explicit_name or len(domains) < 2:
            return gnd_net_id
        dom_id, _dom_name = resolve_return_net_id(pcb_data, via.net_id, explicit_name)
        return dom_id if dom_id is not None else gnd_net_id

    # Collect all signal vias from the board (non-ground, non-power vias). Every
    # ground DOMAIN's vias are excluded, not just the resolved one -- an AGND via
    # is not a signal via needing a return path.
    ground_ids = set(domains) or {gnd_net_id}
    ground_ids.add(gnd_net_id)
    signal_vias = [v for v in pcb_data.vias if v.net_id not in ground_ids and v.net_id != 0]

    if not signal_vias:
        print("No signal vias found in board")
        return []

    print(f"Checking {len(signal_vias)} signal vias for GND via placement")

    # Existing return copper, per ground net: a via only counts as "already has a
    # return" if the nearby ground copper is in the SAME domain (that is the whole
    # point of the split).
    existing_by_net: Dict[int, List] = {}
    for nid in ground_ids:
        pts = [(v.x, v.y) for v in pcb_data.vias if v.net_id == nid]
        # Through-hole pads of that net act as return vias too.
        for pad in pcb_data.pads_by_net.get(nid, []):
            if pad.drill and pad.drill > 0:
                pts.append((pad.global_x, pad.global_y))
        existing_by_net[nid] = pts

    # Minimum distance from signal via center to GND via center: larger of the
    # copper ring clearance (via_size + clearance) and the drill hole-to-hole
    # minimum (via_drill + hole_to_hole_clearance), so the paired GND via clears
    # the signal via on both measures (issue #125). ONE kernel, in fab_tiers,
    # shared with diff_pair_routing._min_via_center_distance and the placement
    # escape ledger -- three open-coded copies of one rule is how they drift.
    min_distance = fab_tiers.min_via_center_distance(
        config.via_size, config.clearance, config.via_drill,
        config.hole_to_hole_clearance)

    # Debug: Print clearance parameters
    print(f"GND via placement parameters:")
    print(f"  via_size={config.via_size}mm, via_drill={config.via_drill}mm, clearance={config.clearance}mm")
    print(f"  grid_step={config.grid_step}mm")
    print(f"  min_distance (center-to-center)={min_distance:.3f}mm")

    new_gnd_vias = []
    placed_gnd_positions = []

    # Via-to-via minimum clearance (center-to-center) for batch placement: larger
    # of copper ring (via_size + clearance) and drill hole-to-hole (via_drill +
    # hole_to_hole_clearance), so a thin-ring via can't land inside the drill
    # hole-to-hole minimum (issue #125). Same kernel as min_distance above.
    via_via_min_dist = fab_tiers.min_via_center_distance(
        config.via_size, config.clearance, config.via_drill,
        config.hole_to_hole_clearance)

    # Grid cells to check for via clearance
    # The obstacle map already expands tracks by track_width/2 + clearance.
    # We only need to check cells within via_drill/2 (the actual hole), not the full
    # via pad, since the obstacle map expansion already accounts for clearance.
    # Ceil (not floor) the hole radius so this circular clear-check doesn't miss the
    # outermost ring and let the GND-via hole land ~1 cell into a keep-out (#154 class).
    # The obstacle map expands each obstacle by (its_half + clearance +
    # TRACK_width/2) -- the body it models is a TRACK. A via's copper ring is
    # via_size/2 wide, so scanning only the DRILL radius under-enforced by
    # (via_size - track_width)/2 ~ 0.19mm and shipped 35-99um sub-clearance
    # grazes against tracks and SMD pads (Andy's bitaxe DRC2.rpt: every
    # violation was a stitching via). Scan the full ring shortfall.
    _ring_shortfall = max(config.via_drill / 2,
                          config.via_size / 2 - config.track_width / 2)
    via_check_radius_grid = max(1, coord.to_grid_dist_safe(_ring_shortfall))

    # Board outline, as a LAST-RESORT backstop. The obstacle map's static
    # off-board keep-out (#422) is the real mechanism and does this properly
    # (cutouts, milled contours, non-rectangular outlines); this only catches a
    # caller that built its map with board_edge_clearance unset, which is how a
    # GND return via once landed 1.40mm outside the board edge and would have
    # been milled away at depanelization. Bounding-box only, so it is
    # deliberately permissive -- it never rejects a via the keep-out accepts.
    _bounds = getattr(pcb_data.board_info, 'board_bounds', None)
    _edge_keepout = config.board_edge_clearance or 0.0

    def _outside_board(x_mm: float, y_mm: float) -> bool:
        if not _bounds:
            return False
        min_x, min_y, max_x, max_y = _bounds
        margin = _edge_keepout + config.via_size / 2
        return not (min_x + margin <= x_mm <= max_x - margin
                    and min_y + margin <= y_mm <= max_y - margin)

    # Sweep item 11 (#625 follow-up): the clearance checker scanned ALL board
    # vias and ALL drilled pads in Python per candidate position (candidates =
    # signal vias x 24 angles x radii -- ~100M+ scalar ops on big boards).
    # Precompute their arrays once; each candidate then runs two broadcasts
    # that NOMINATE possible violators, and only those are re-judged by the
    # scalar formulas in the original iteration order -- so the verdict AND
    # the first-failure reason string (with its formatted distance) are
    # byte-identical.
    import numpy as np
    _via_xs = np.array([v.x for v in pcb_data.vias], dtype=np.float64)
    _via_ys = np.array([v.y for v in pcb_data.vias], dtype=np.float64)
    _drill_pads = [pad for _nid, _pads in pcb_data.pads_by_net.items()
                   for pad in _pads if pad.drill and pad.drill > 0]
    _caps = [pad_drill_capsule(p) for p in _drill_pads]
    _cap_p1x = np.array([c[0][0] for c in _caps], dtype=np.float64)
    _cap_p1y = np.array([c[0][1] for c in _caps], dtype=np.float64)
    _cap_dx = np.array([c[1][0] - c[0][0] for c in _caps], dtype=np.float64)
    _cap_dy = np.array([c[1][1] - c[0][1] for c in _caps], dtype=np.float64)
    _cap_len_sq = _cap_dx * _cap_dx + _cap_dy * _cap_dy
    _cap_rad = np.array([c[2] for c in _caps], dtype=np.float64)

    def _cap_d2(px, py):
        with np.errstate(divide='ignore', invalid='ignore'):
            t = np.clip(((px - _cap_p1x) * _cap_dx + (py - _cap_p1y) * _cap_dy)
                        / _cap_len_sq, 0.0, 1.0)
        t = np.where(_cap_len_sq < 1e-10, 0.0, t)
        ddx = px - (_cap_p1x + t * _cap_dx)
        ddy = py - (_cap_p1y + t * _cap_dy)
        return ddx * ddx + ddy * ddy

    def is_via_position_clear(x_mm: float, y_mm: float, sig_via_x: float, sig_via_y: float) -> tuple:
        """Check if a via can be placed at the given position.

        Returns: (is_clear, reason) where reason explains why it's blocked if not clear.

        Note: We do NOT use obstacles.is_via_blocked() here because that blocks
        positions within via_size+clearance of ALL vias, including the signal via
        we're placing near. Since we already enforce min_distance (which equals
        via_size+clearance), we just need to check track clearance and manually
        check other vias.
        """
        if _outside_board(x_mm, y_mm):
            return (False, "outside_board_outline")

        gx, gy = coord.to_grid(x_mm, y_mm)

        # Check all layers for track clearance
        # The obstacle map expands tracks by track_width/2 + clearance, so we only
        # need to check cells within the via drill radius
        for layer_idx in range(len(config.layers)):
            for dx in range(-via_check_radius_grid, via_check_radius_grid + 1):
                for dy in range(-via_check_radius_grid, via_check_radius_grid + 1):
                    if dx * dx + dy * dy <= via_check_radius_grid * via_check_radius_grid:
                        if obstacles.is_blocked(gx + dx, gy + dy, layer_idx):
                            return (False, f"track_blocked_layer{layer_idx}_at_({dx},{dy})")

        # Check via-to-via clearance for OTHER vias (skip the signal via we're
        # targeting). Broadcast nominates (band around the strict < threshold
        # covers the **2-pow vs multiply ULP); candidates re-judged in via
        # order with the scalar formula so the first-failure reason matches.
        if len(_via_xs):
            _dxv = x_mm - _via_xs
            _dyv = y_mm - _via_ys
            _d2v = _dxv * _dxv + _dyv * _dyv
            _thr = (via_via_min_dist * (1 + 1e-9)) ** 2
            for i in np.nonzero(_d2v < _thr)[0]:
                via = pcb_data.vias[i]
                if abs(via.x - sig_via_x) < 0.01 and abs(via.y - sig_via_y) < 0.01:
                    continue
                dist = math.sqrt((x_mm - via.x)**2 + (y_mm - via.y)**2)
                if dist < via_via_min_dist:
                    return (False, f"too_close_to_via({dist:.2f}mm)")

        # Check drill hole-to-hole clearance from through-hole pad drills (a
        # physical drill-to-drill minimum -> hole_to_hole_clearance, NOT the copper
        # clearance, which under-enforces it; issue #125 PAD-DRILL-VIA-DRILL).
        # Same nomination pattern against the precomputed drill CAPSULES (a
        # slot is a milled slot, not a round hole; round drills degenerate to
        # the centre distance).
        if len(_cap_rad):
            min_pad_dist = config.hole_to_hole_clearance + config.via_drill / 2
            _d2c = _cap_d2(x_mm, y_mm)
            _lim = (min_pad_dist + _cap_rad) * (1 + 1e-9)
            for i in np.nonzero(_d2c < _lim * _lim)[0]:
                pad = _drill_pads[i]
                (p1x, p1y), (p2x, p2y), prad = pad_drill_capsule(pad)
                dist = point_to_segment_distance(x_mm, y_mm, p1x, p1y, p2x, p2y) - prad
                if dist < min_pad_dist:
                    return (False, f"too_close_to_th_pad({dist:.2f}mm)")

        # Check against GND vias we're placing in this batch. Physical spacing,
        # so EVERY batch via counts regardless of which ground domain it serves.
        for (px, py), _bnet in placed_gnd_positions:
            dist = math.sqrt((x_mm - px)**2 + (y_mm - py)**2)
            if dist < via_via_min_dist:
                return (False, f"too_close_to_batch_gnd_via({dist:.2f}mm)")

        return (True, "clear")

    # Try many angles for better coverage (every 15 degrees = 24 directions)
    angles_deg = list(range(0, 360, 15))

    skipped_has_gnd = 0
    skipped_no_space = 0
    placement_distances = []  # Track distances where GND vias were placed

    for sig_via in signal_vias:
        sx, sy = sig_via.x, sig_via.y
        via_gnd_net_id = _return_net_for(sig_via)

        # Check if there's already a same-domain GND via/pad within
        # gnd_via_distance. Ground copper of ANOTHER domain is not a return path
        # for this signal, so it must not satisfy the coverage test (#489 §5).
        has_nearby_gnd = False
        nearest_gnd_dist = float('inf')
        for gx, gy in (existing_by_net.get(via_gnd_net_id, [])
                       + [p for p, n in placed_gnd_positions if n == via_gnd_net_id]):
            dist = math.sqrt((sx - gx)**2 + (sy - gy)**2)
            nearest_gnd_dist = min(nearest_gnd_dist, dist)
            if dist <= gnd_via_distance:
                has_nearby_gnd = True
                break

        if has_nearby_gnd:
            skipped_has_gnd += 1
            continue

        # Try to place a GND via as close as possible to signal via
        best_pos = None
        distance_step = config.grid_step / 2  # Finer step for better placement

        distance = min_distance
        while distance <= gnd_via_distance:
            # Try all angles at this distance
            for angle_deg in angles_deg:
                angle_rad = math.radians(angle_deg)
                # Snap to the routing grid: the clearance check works on
                # grid CELLS, so an off-grid candidate can sit up to half a
                # cell (25um at 0.05) inside an obstacle the check calls
                # clear -- the residual 12-22um stitching-via overlaps.
                gnd_x = round((sx + distance * math.cos(angle_rad))
                              / config.grid_step) * config.grid_step
                gnd_y = round((sy + distance * math.sin(angle_rad))
                              / config.grid_step) * config.grid_step

                is_clear, reason = is_via_position_clear(gnd_x, gnd_y, sx, sy)
                if is_clear:
                    # Found a valid position at closest possible distance
                    best_pos = (gnd_x, gnd_y)
                    break

            if best_pos is not None:
                break  # Found a position, stop searching

            distance += distance_step

        if best_pos is None:
            skipped_no_space += 1
        else:
            actual_dist = math.sqrt((best_pos[0] - sx)**2 + (best_pos[1] - sy)**2)
            placement_distances.append(actual_dist)

            gnd_via = Via(
                x=best_pos[0],
                y=best_pos[1],
                size=config.via_size,
                drill=config.via_drill,
                # Through-hole span like every other routed via (see above).
                layers=["F.Cu", "B.Cu"],
                # This signal's OWN ground domain (#489 §5), not one board-global net.
                net_id=via_gnd_net_id,
                free=True
            )
            new_gnd_vias.append(gnd_via)
            placed_gnd_positions.append((best_pos, via_gnd_net_id))
            existing_by_net.setdefault(via_gnd_net_id, []).append(best_pos)

    # Summary
    if new_gnd_vias:
        print(f"\nAdded {len(new_gnd_vias)} GND vias near signal vias")
        if len(domains) > 1:
            per_net: Dict[int, int] = {}
            for v in new_gnd_vias:
                per_net[v.net_id] = per_net.get(v.net_id, 0) + 1
            breakdown = ", ".join(
                f"{(pcb_data.nets.get(nid).name if pcb_data.nets.get(nid) else nid)}: {n}"
                for nid, n in sorted(per_net.items(), key=lambda kv: -kv[1]))
            print(f"  Per ground domain -- {breakdown}")
        if placement_distances:
            # fsum for consistency with the rest of the codebase (#493); this
            # one only feeds the printed summary, so no board depends on it.
            avg_dist = math.fsum(placement_distances) / len(placement_distances)
            min_placed = min(placement_distances)
            max_placed = max(placement_distances)
            print(f"  Placement distances: min={min_placed:.3f}mm, avg={avg_dist:.3f}mm, max={max_placed:.3f}mm")
            print(f"  (min_distance theoretical={min_distance:.3f}mm)")
    if skipped_has_gnd > 0:
        print(f"  {skipped_has_gnd} signal vias already had nearby GND via/pad")
    if skipped_no_space > 0:
        print(f"  {skipped_no_space} signal vias had no clear space for GND via within {gnd_via_distance}mm")
    if not new_gnd_vias and skipped_has_gnd == len(signal_vias):
        print("All signal vias already have nearby GND vias/pads")

    return new_gnd_vias
