#!/usr/bin/env python3
"""
Blocking analysis for failed routes.

When a route fails, analyzes which previously-routed nets are blocking.
Uses frontier data from the router (cells that the A* search tried to
expand into but found blocked) for accurate analysis.

Usage:
    from blocking_analysis import analyze_frontier_blocking

    # After a route fails with frontier data:
    path, iterations, blocked_cells = router.route_with_frontier(...)

    if path is None:
        blockers = analyze_frontier_blocking(
            blocked_cells,      # From router
            pcb_data, config,
            routed_net_paths,   # Dict of net_id -> path
        )
        print_blocking_analysis(blockers)
"""

from typing import List, Tuple, Dict, Set, Optional
from dataclasses import dataclass
from collections import defaultdict

from kicad_parser import PCBData, Segment, Via
from routing_config import GridRouteConfig, GridCoord
from routing_utils import build_layer_map
from bresenham_utils import walk_line


def invalidate_obstacle_cache(cache: Dict, net_id: int) -> None:
    """Remove all cache entries for a net_id.

    Cache keys are (net_id, extra_clearance) tuples, so we need to remove
    all entries where the first element matches.
    """
    keys_to_remove = [k for k in cache if k[0] == net_id]
    for k in keys_to_remove:
        del cache[k]


@dataclass
class BlockingInfo:
    """Information about how much a net blocks a route."""
    net_id: int
    net_name: str
    blocked_count: int  # Number of frontier cells blocked by this net
    track_cells: int    # Cells blocked by tracks
    via_cells: int      # Cells blocked by vias
    unique_cells: int   # Cells where this net is the ONLY blocker
    near_target_cells: int  # Cells within proximity of target (more critical)
    near_source_cells: int  # Cells within proximity of source
    details: str        # Human-readable details


def compute_net_obstacle_cells(
    pcb_data: PCBData,
    net_id: int,
    path: Optional[List[Tuple[int, int, int]]],
    config: GridRouteConfig,
    extra_clearance: float = 0.0,
) -> Tuple[Set[Tuple[int, int, int]], Set[Tuple[int, int, int]]]:
    """
    Compute all obstacle cells for a net (tracks and vias).

    Returns (track_cells, via_cells) where each cell is (gx, gy, layer).
    """
    coord = GridCoord(config.grid_step)
    layer_map = build_layer_map(config.layers)
    num_layers = len(config.layers)

    # Match the expansion used in obstacle_map.py add_net_stubs_as_obstacles
    expansion_mm = config.track_width / 2 + config.clearance + extra_clearance
    expansion_grid = max(1, coord.to_grid_dist(expansion_mm))
    via_expansion_grid = max(1, coord.to_grid_dist(
        config.via_size / 2 + config.track_width / 2 + config.clearance + extra_clearance))

    track_cells = set()
    via_cells = set()

    # Add cells from routed path
    if path:
        for i in range(len(path) - 1):
            gx1, gy1, layer1 = path[i]
            gx2, gy2, layer2 = path[i + 1]

            if layer1 != layer2:
                # Via - blocks all layers
                for ex in range(-via_expansion_grid, via_expansion_grid + 1):
                    for ey in range(-via_expansion_grid, via_expansion_grid + 1):
                        if ex*ex + ey*ey <= via_expansion_grid * via_expansion_grid:
                            for layer_idx in range(num_layers):
                                via_cells.add((gx1 + ex, gy1 + ey, layer_idx))
            else:
                # Track segment
                for gx, gy in walk_line(gx1, gy1, gx2, gy2):
                    for ex in range(-expansion_grid, expansion_grid + 1):
                        for ey in range(-expansion_grid, expansion_grid + 1):
                            track_cells.add((gx + ex, gy + ey, layer1))

    # Add cells from original stubs
    for seg in pcb_data.segments:
        if seg.net_id != net_id:
            continue
        layer_idx = layer_map.get(seg.layer)
        if layer_idx is None:
            continue

        gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
        gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)

        for gx, gy in walk_line(gx1, gy1, gx2, gy2):
            for ex in range(-expansion_grid, expansion_grid + 1):
                for ey in range(-expansion_grid, expansion_grid + 1):
                    track_cells.add((gx + ex, gy + ey, layer_idx))

    # Add cells from existing vias
    for via in pcb_data.vias:
        if via.net_id != net_id:
            continue
        gx, gy = coord.to_grid(via.x, via.y)
        for ex in range(-via_expansion_grid, via_expansion_grid + 1):
            for ey in range(-via_expansion_grid, via_expansion_grid + 1):
                if ex*ex + ey*ey <= via_expansion_grid * via_expansion_grid:
                    for layer_idx in range(num_layers):
                        via_cells.add((gx + ex, gy + ey, layer_idx))

    return track_cells, via_cells


def analyze_frontier_blocking(
    blocked_cells: List[Tuple[int, int, int]],
    pcb_data: PCBData,
    config: GridRouteConfig,
    routed_net_paths: Dict[int, List[Tuple[int, int, int]]],
    exclude_net_ids: Optional[Set[int]] = None,
    extra_clearance: float = 0.0,
    target_xy: Optional[Tuple[float, float]] = None,
    source_xy: Optional[Tuple[float, float]] = None,
    obstacle_cache: Optional[Dict[int, Tuple[Set, Set]]] = None,
) -> List[BlockingInfo]:
    """
    Analyze which nets are blocking based on frontier data.

    Args:
        blocked_cells: List of (gx, gy, layer) cells that blocked the search
                      (from router's route_with_frontier)
        pcb_data: PCB data with segments, vias, etc.
        config: Routing configuration
        routed_net_paths: Dict mapping net_id -> routed path for previously routed nets
        exclude_net_ids: Net IDs to exclude from analysis (e.g., the current net)
        extra_clearance: Extra clearance for diff pair centerline routing
        target_xy: Optional (x, y) target coordinates in mm for proximity analysis
        source_xy: Optional (x, y) source coordinates in mm for proximity analysis
        obstacle_cache: Optional cache of net_id -> (track_cells, via_cells) to avoid
                       recomputing obstacle cells. Pass same dict across retry iterations.

    Returns:
        List of BlockingInfo sorted by blocking priority
    """
    if not blocked_cells:
        return []

    exclude_net_ids = exclude_net_ids or set()
    blocked_set = set(blocked_cells)

    # Compute source/target grid coords and proximity threshold
    coord = GridCoord(config.grid_step)
    target_gx, target_gy = None, None
    source_gx, source_gy = None, None
    # "Near" = within 3mm (30 grid cells at 0.1mm grid)
    near_radius_grid = int(3.0 / config.grid_step)
    if target_xy is not None:
        target_gx, target_gy = coord.to_grid(target_xy[0], target_xy[1])
    if source_xy is not None:
        source_gx, source_gy = coord.to_grid(source_xy[0], source_xy[1])

    # First pass: compute obstacle cells for each net and track which nets block each cell
    cell_to_blockers: Dict[Tuple[int, int, int], Set[int]] = defaultdict(set)
    net_blocking_data = {}  # net_id -> (track_cells, via_cells, blocking_track, blocking_via, blocking_total)

    # Use provided cache or create local one
    local_cache = obstacle_cache if obstacle_cache is not None else {}

    for net_id, path in routed_net_paths.items():
        if net_id in exclude_net_ids:
            continue

        # Get obstacle cells from cache or compute
        # Cache key includes extra_clearance since it affects expansion radius
        cache_key = (net_id, extra_clearance)
        if cache_key in local_cache:
            track_cells, via_cells = local_cache[cache_key]
        else:
            track_cells, via_cells = compute_net_obstacle_cells(
                pcb_data, net_id, path, config, extra_clearance
            )
            # Store in cache for future calls
            local_cache[cache_key] = (track_cells, via_cells)

        all_cells = track_cells | via_cells

        # Count how many blocked frontier cells this net is responsible for
        blocking_track = blocked_set & track_cells
        blocking_via = blocked_set & via_cells
        blocking_total = blocked_set & all_cells

        if len(blocking_total) > 0:
            net_blocking_data[net_id] = (track_cells, via_cells, blocking_track, blocking_via, blocking_total)
            # Track which nets block each cell
            for cell in blocking_total:
                cell_to_blockers[cell].add(net_id)

    # Second pass: count unique blocking and near-source/target blocking
    results = []
    for net_id, (track_cells, via_cells, blocking_track, blocking_via, blocking_total) in net_blocking_data.items():
        net = pcb_data.nets.get(net_id)
        net_name = net.name if net else f"Net {net_id}"

        # Count cells where this net is the ONLY blocker
        unique_count = sum(1 for cell in blocking_total if len(cell_to_blockers[cell]) == 1)

        # Count cells near target and source
        near_target_count = 0
        near_source_count = 0
        for gx, gy, _ in blocking_total:
            if target_gx is not None:
                dist_sq = (gx - target_gx) ** 2 + (gy - target_gy) ** 2
                if dist_sq <= near_radius_grid ** 2:
                    near_target_count += 1
            if source_gx is not None:
                dist_sq = (gx - source_gx) ** 2 + (gy - source_gy) ** 2
                if dist_sq <= near_radius_grid ** 2:
                    near_source_count += 1

        details = f"{len(blocking_track)} track, {len(blocking_via)} via cells on frontier"
        results.append(BlockingInfo(
            net_id=net_id,
            net_name=net_name,
            blocked_count=len(blocking_total),
            track_cells=len(blocking_track),
            via_cells=len(blocking_via),
            unique_cells=unique_count,
            near_target_cells=near_target_count,
            near_source_cells=near_source_count,
            details=details
        ))

    # Sort to prioritize nets that will actually open up routing:
    # 1. Nets with 100% unique blocking are top priority (guaranteed to help)
    # 2. For others, use weighted score that considers:
    #    - unique_cells (guaranteed to open up)
    #    - near_target_cells and near_source_cells (blocking critical areas)
    #    - shared blocking (partial credit)
    def sort_key(x):
        near_endpoint = x.near_target_cells + x.near_source_cells
        if x.blocked_count > 0 and x.unique_cells == x.blocked_count:
            # 100% unique - highest priority, use near_endpoint as tiebreaker
            return (2, x.unique_cells, near_endpoint)
        else:
            # Weighted score: unique counts full, near-endpoint unique counts extra, shared counts half
            shared = x.blocked_count - x.unique_cells
            # Near-endpoint unique cells are extra valuable
            near_endpoint_unique = min(near_endpoint, x.unique_cells)
            score = x.unique_cells + near_endpoint_unique + 0.5 * shared
            return (1, score, near_endpoint)

    results.sort(key=sort_key, reverse=True)

    return results


def analyze_static_blockers(
    blocked_cells: List[Tuple[int, int, int]],
    pcb_data: PCBData,
    config: GridRouteConfig,
    nets_to_route: Optional[Set[int]] = None,
) -> Dict[str, List[str]]:
    """
    Analyze what static obstacles are blocking the given cells.

    Returns a dict with categories of blockers:
    - 'pads': list of "NetName (pad ref)" strings for blocking pads
    - 'tracks': list of "NetName" strings for pre-existing tracks
    - 'zones': count of cells in BGA exclusion zones
    """
    coord = GridCoord(config.grid_step)
    layer_map = build_layer_map(config.layers)
    blocked_set = set(blocked_cells)
    nets_to_route = nets_to_route or set()

    result = {
        'pads': [],
        'tracks': [],
        'zone_cells': 0,
    }

    # Track which nets' pads are blocking
    pad_blockers = {}  # net_name -> set of pad refs

    # Check pads from other nets
    for net_id, pads in pcb_data.pads_by_net.items():
        if net_id in nets_to_route:
            continue
        net = pcb_data.nets.get(net_id)
        net_name = net.name if net else f"Net {net_id}"
        for pad in pads:
            # Get pad grid area
            pad_gx, pad_gy = coord.to_grid(pad.global_x, pad.global_y)

            # Calculate pad expansion (matches obstacle_map.py formula)
            # margin = track_width/2 + clearance (route needs half-width from center + clearance from pad edge)
            pad_half_w = pad.size_x / 2
            pad_half_h = pad.size_y / 2
            margin = config.track_width / 2 + config.clearance
            expand_x = max(1, coord.to_grid_dist(pad_half_w + margin))
            expand_y = max(1, coord.to_grid_dist(pad_half_h + margin))

            # Check if any blocked cells fall within this pad's area
            for cell in blocked_set:
                gx, gy, layer_idx = cell
                layer_name = config.layers[layer_idx] if layer_idx < len(config.layers) else None
                if layer_name and layer_name not in pad.layers:
                    continue
                if abs(gx - pad_gx) <= expand_x and abs(gy - pad_gy) <= expand_y:
                    if net_name not in pad_blockers:
                        pad_blockers[net_name] = set()
                    pad_blockers[net_name].add(pad.component_ref)
                    break  # Found blocking, move to next pad

    # Format pad blockers
    for net_name, refs in sorted(pad_blockers.items(), key=lambda x: -len(x[1])):
        if len(refs) <= 3:
            result['pads'].append(f"{net_name} ({', '.join(sorted(refs))})")
        else:
            result['pads'].append(f"{net_name} ({len(refs)} pads)")

    # Check pre-existing tracks from other nets
    track_blockers = set()
    for seg in pcb_data.segments:
        if seg.net_id in nets_to_route:
            continue
        layer_idx = layer_map.get(seg.layer)
        if layer_idx is None:
            continue

        # Get segment grid area
        gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
        gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)
        expansion_grid = max(1, coord.to_grid_dist(seg.width / 2 + config.clearance))

        # Check if any blocked cells fall near this segment
        for cell in blocked_set:
            gx, gy, cell_layer = cell
            if cell_layer != layer_idx:
                continue
            # Simple bounding box check
            min_gx = min(gx1, gx2) - expansion_grid
            max_gx = max(gx1, gx2) + expansion_grid
            min_gy = min(gy1, gy2) - expansion_grid
            max_gy = max(gy1, gy2) + expansion_grid
            if min_gx <= gx <= max_gx and min_gy <= gy <= max_gy:
                net = pcb_data.nets.get(seg.net_id)
                net_name = net.name if net else f"Net {seg.net_id}"
                track_blockers.add(net_name)
                break

    result['tracks'] = sorted(track_blockers)

    # Check BGA exclusion zones
    zone_cells = 0
    for zone in config.bga_exclusion_zones:
        min_x, min_y, max_x, max_y = zone[:4]
        gmin_x, gmin_y = coord.to_grid(min_x, min_y)
        gmax_x, gmax_y = coord.to_grid(max_x, max_y)
        for cell in blocked_set:
            gx, gy, _ = cell
            if gmin_x <= gx <= gmax_x and gmin_y <= gy <= gmax_y:
                zone_cells += 1
    result['zone_cells'] = zone_cells

    return result


def print_blocking_analysis(
    blockers: List[BlockingInfo],
    max_display: int = 10,
    prefix: str = "  ",
    blocked_cells: Optional[List[Tuple[int, int, int]]] = None,
    pcb_data: Optional[PCBData] = None,
    config: Optional[GridRouteConfig] = None,
    nets_to_route: Optional[Set[int]] = None,
):
    """Print blocking analysis results."""
    if not blockers:
        msg = f"{prefix}No previously-routed nets blocking"
        # If we have the data, analyze static blockers
        if blocked_cells and pcb_data and config:
            static = analyze_static_blockers(blocked_cells, pcb_data, config, nets_to_route)
            details = []
            if static['pads']:
                details.append(f"pads: {', '.join(static['pads'][:5])}")
                if len(static['pads']) > 5:
                    details[-1] += f" (+{len(static['pads'])-5} more)"
            if static['tracks']:
                details.append(f"pre-existing tracks: {', '.join(static['tracks'][:3])}")
                if len(static['tracks']) > 3:
                    details[-1] += f" (+{len(static['tracks'])-3} more)"
            if static['zone_cells'] > 0:
                details.append(f"BGA zone ({static['zone_cells']} cells)")
            if details:
                print(f"{msg}")
                print(f"{prefix}Static blockers: {'; '.join(details)}")
            else:
                print(f"{msg} (likely blocked by pads/stubs/zones)")
        else:
            print(f"{msg} (likely blocked by pads/stubs/zones)")
        return

    total_blocked = sum(b.blocked_count for b in blockers)
    total_unique = sum(b.unique_cells for b in blockers)
    total_near_target = sum(b.near_target_cells for b in blockers)
    total_near_source = sum(b.near_source_cells for b in blockers)

    summary = f"{prefix}Frontier blocked by {len(blockers)} nets ({total_blocked} cells, {total_unique} unique"
    if total_near_source > 0 or total_near_target > 0:
        summary += f"; near: {total_near_source} src, {total_near_target} tgt"
    summary += ")"
    print(summary)

    print(f"{prefix}Top blockers:")
    for i, info in enumerate(blockers[:max_display]):
        pct = 100.0 * info.blocked_count / total_blocked if total_blocked > 0 else 0
        unique_pct = 100.0 * info.unique_cells / info.blocked_count if info.blocked_count > 0 else 0

        # Build output string
        parts = [f"{info.blocked_count} ({pct:.1f}%)"]
        if info.unique_cells > 0:
            parts.append(f"{info.unique_cells} uniq ({unique_pct:.0f}%)")
        else:
            parts.append("no uniq")
        # Show near-source and near-target if either is non-zero
        if info.near_source_cells > 0 or info.near_target_cells > 0:
            parts.append(f"near: {info.near_source_cells} src, {info.near_target_cells} tgt")

        print(f"{prefix}  {i+1}. {info.net_name}: {', '.join(parts)}")

    if len(blockers) > max_display:
        remaining = sum(b.blocked_count for b in blockers[max_display:])
        print(f"{prefix}  ... and {len(blockers) - max_display} more nets ({remaining} cells)")


def filter_rippable_blockers(
    blockers: List[BlockerInfo],
    routed_results: Dict,
    diff_pair_by_net_id: Dict,
    get_canonical_net_id_func
) -> Tuple[List[BlockerInfo], Set[int]]:
    """
    Filter blockers to only those that can be ripped (in routed_results),
    deduplicating by diff pair (P and N count as one).

    Args:
        blockers: List of BlockerInfo from analyze_frontier_blocking
        routed_results: Dict of net_id -> result for routed nets
        diff_pair_by_net_id: Dict mapping net_id -> (pair_name, pair)
        get_canonical_net_id_func: Function to get canonical net ID

    Returns:
        Tuple of (rippable_blockers, seen_canonical_ids)
    """
    rippable_blockers = []
    seen_canonical_ids = set()
    for b in blockers:
        if b.net_id in routed_results:
            canonical = get_canonical_net_id_func(b.net_id, diff_pair_by_net_id)
            if canonical not in seen_canonical_ids:
                seen_canonical_ids.add(canonical)
                rippable_blockers.append(b)
    return rippable_blockers, seen_canonical_ids


if __name__ == "__main__":
    print("Blocking analysis module")
    print("Use analyze_frontier_blocking() with data from route_with_frontier()")
