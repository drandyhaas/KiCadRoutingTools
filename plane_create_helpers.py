"""
Helper functions for create_plane to improve code organization.

These functions handle specific phases of plane creation:
- Loading and validating PCB data
- Placing vias for a single pad
- Generating multi-net Voronoi zones
- Writing results and output
"""

import math
import sys
from typing import List, Dict, Tuple, Optional, Set
from dataclasses import dataclass

from kicad_parser import PCBData, Pad, Via, Segment
from kicad_writer import generate_zone_sexpr
from routing_config import GridRouteConfig, GridCoord
from route import batch_route
from obstacle_cache import ViaPlacementObstacleData

from plane_io import (
    ZoneInfo,
    extract_zones,
    check_existing_zones,
    resolve_net_id,
    write_plane_output
)
from plane_obstacle_builder import (
    identify_target_pads,
    build_via_obstacle_map,
    build_routing_obstacle_map,
    block_via_position
)
from plane_blocker_detection import (
    ViaPlacementResult,
    try_place_via_with_ripup
)
from plane_zone_geometry import (
    compute_zone_boundaries,
    find_polygon_groups,
    sample_route_for_voronoi
)

import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'rust_router'))
from grid_router import GridObstacleMap


@dataclass
class PlaneCreationContext:
    """Shared context for plane creation operations."""
    pcb_data: PCBData
    config: GridRouteConfig
    coord: GridCoord
    board_bounds: Tuple[float, float, float, float]
    zone_polygon: List[Tuple[float, float]]
    all_layers: List[str]
    plane_layers: List[str]
    verbose: bool

    # Via parameters
    via_size: float
    via_drill: float
    hole_to_hole_clearance: float
    max_search_radius: float
    max_via_reuse_radius: float

    # Zone parameters
    zone_clearance: float
    min_thickness: float
    board_edge_clearance: float

    # Rip-up parameters
    rip_blocker_nets: bool
    max_rip_nets: int
    protected_net_ids: Set[int]


@dataclass
class NetProcessingResult:
    """Results from processing a single net."""
    new_vias: List[Dict]
    new_segments: List[Dict]
    zone_sexpr: Optional[str]
    vias_placed: int
    vias_reused: int
    traces_added: int
    failed_pads: int
    ripped_net_ids: List[int]


def process_pad_via_placement(
    pad_info: Dict,
    ctx: PlaneCreationContext,
    net_id: int,
    obstacles: GridObstacleMap,
    routing_obstacles_cache: Dict[str, GridObstacleMap],
    available_vias: List[Tuple[float, float]],
    via_obstacle_cache: Dict[int, ViaPlacementObstacleData],
    find_existing_via_nearby_fn,
    find_via_position_fn,
    route_via_to_pad_fn
) -> Tuple[Optional[Dict], List[Dict], int, int, int, List[int]]:
    """
    Process via placement for a single pad.

    Returns:
        (new_via, new_segments, vias_placed, vias_reused, traces_added, ripped_net_ids)
    """
    pad = pad_info['pad']
    pad_layer = pad_info.get('pad_layer')

    new_via = None
    new_segments = []
    vias_placed = 0
    vias_reused = 0
    traces_added = 0
    ripped_net_ids = []

    print(f"  Pad {pad.component_ref}.{pad.pad_number}...", end=" ")

    def get_routing_obstacles(layer: str) -> GridObstacleMap:
        if layer not in routing_obstacles_cache:
            if ctx.verbose:
                print(f"  Building routing obstacle map for {layer}...")
            routing_obstacles_cache[layer] = build_routing_obstacle_map(
                ctx.pcb_data, ctx.config, net_id, layer, skip_pad_blocking=False
            )
        return routing_obstacles_cache[layer]

    # First check if there's an existing or already-placed via nearby
    existing_via = find_existing_via_nearby_fn(pad, available_vias, ctx.max_via_reuse_radius)

    if existing_via:
        via_pos = existing_via
        if pad_layer:
            routing_obs = get_routing_obstacles(pad_layer)
            route_result = route_via_to_pad_fn(via_pos, pad, pad_layer, net_id,
                                                routing_obs, ctx.config, verbose=ctx.verbose,
                                                return_blocked_cells=True)
            if route_result.success:
                if route_result.segments:
                    new_segments.extend(route_result.segments)
                    traces_added += len(route_result.segments)
                vias_reused += 1
                dist = ((via_pos[0] - pad.global_x)**2 + (via_pos[1] - pad.global_y)**2)**0.5
                if route_result.segments:
                    print(f"reusing via at ({via_pos[0]:.2f}, {via_pos[1]:.2f}), {len(route_result.segments)} segs, {dist:.2f}mm")
                else:
                    print(f"reusing via at pad center")
                return (None, new_segments, vias_placed, vias_reused, traces_added, ripped_net_ids)
            else:
                print(f"can't route to existing via, ", end="")
                existing_via = None  # Fall through to new via placement
        else:
            vias_reused += 1
            print(f"reusing via at ({via_pos[0]:.2f}, {via_pos[1]:.2f})")
            return (None, new_segments, vias_placed, vias_reused, traces_added, ripped_net_ids)

    # Need to place a new via
    routing_obs = get_routing_obstacles(pad_layer) if pad_layer else None
    failed_route_positions: Set[Tuple[int, int]] = set()
    via_pos = find_via_position_fn(
        pad, obstacles, ctx.coord, ctx.max_search_radius,
        routing_obstacles=routing_obs,
        config=ctx.config,
        pad_layer=pad_layer,
        net_id=net_id,
        verbose=ctx.verbose,
        failed_route_positions=failed_route_positions
    )

    placement_success = False
    trace_segments = None
    via_blocked = via_pos is None
    blocked_cells = []

    if via_pos:
        via_at_pad_center = (abs(via_pos[0] - pad.global_x) < 0.001 and
                            abs(via_pos[1] - pad.global_y) < 0.001)
        if via_at_pad_center:
            placement_success = True
        elif pad_layer:
            route_result = route_via_to_pad_fn(via_pos, pad, pad_layer, net_id,
                                                routing_obs, ctx.config, verbose=ctx.verbose,
                                                return_blocked_cells=True)
            if route_result.success:
                trace_segments = route_result.segments
                placement_success = True
            else:
                blocked_cells = route_result.blocked_cells
        else:
            placement_success = True

    # If fast path failed and rip_blocker_nets enabled, try iterative rip-up
    if not placement_success and ctx.rip_blocker_nets:
        print(f"blocked, trying rip-up...", end=" ")
        result = try_place_via_with_ripup(
            pad, pad_layer, net_id, ctx.pcb_data, ctx.config, ctx.coord,
            ctx.max_search_radius, ctx.max_rip_nets,
            obstacles, routing_obs,
            via_obstacle_cache, routing_obstacles_cache, ctx.all_layers,
            via_blocked=via_blocked,
            blocked_cells=blocked_cells,
            hole_to_hole_clearance=ctx.hole_to_hole_clearance,
            via_drill=ctx.via_drill,
            protected_net_ids=ctx.protected_net_ids,
            verbose=ctx.verbose,
            find_via_position_fn=find_via_position_fn,
            route_via_to_pad_fn=route_via_to_pad_fn
        )

        ripped_net_ids.extend(result.ripped_net_ids)

        if result.success:
            new_via = {
                'x': result.via_pos[0], 'y': result.via_pos[1],
                'size': ctx.via_size, 'drill': ctx.via_drill,
                'layers': ['F.Cu', 'B.Cu'], 'net_id': net_id
            }
            vias_placed += 1
            new_segments.extend(result.segments)
            traces_added += len(result.segments)
            block_via_position(obstacles, result.via_pos[0], result.via_pos[1], ctx.coord,
                               ctx.hole_to_hole_clearance, ctx.via_drill)
            print(f"\033[92mvia at ({result.via_pos[0]:.2f}, {result.via_pos[1]:.2f}), ripped {len(result.ripped_net_ids)} nets\033[0m")
        else:
            print(f"\033[91mFAILED after {len(result.ripped_net_ids)} rip-ups\033[0m")

    elif placement_success:
        via_at_pad_center = (abs(via_pos[0] - pad.global_x) < 0.001 and
                            abs(via_pos[1] - pad.global_y) < 0.001)
        new_via = {
            'x': via_pos[0], 'y': via_pos[1],
            'size': ctx.via_size, 'drill': ctx.via_drill,
            'layers': ['F.Cu', 'B.Cu'], 'net_id': net_id
        }
        vias_placed += 1
        if trace_segments:
            new_segments.extend(trace_segments)
            traces_added += len(trace_segments)
        block_via_position(obstacles, via_pos[0], via_pos[1], ctx.coord,
                           ctx.hole_to_hole_clearance, ctx.via_drill)

        if via_at_pad_center:
            print(f"via at pad center")
        else:
            print(f"via at ({via_pos[0]:.2f}, {via_pos[1]:.2f}), {len(trace_segments) if trace_segments else 0} segs")

    elif not ctx.rip_blocker_nets:
        # Fast path failed, no rip-up enabled - try fallback via reuse
        fallback_via = find_existing_via_nearby_fn(pad, available_vias, ctx.max_search_radius)
        if fallback_via:
            via_pos = fallback_via
            if pad_layer:
                routing_obs = get_routing_obstacles(pad_layer)
                route_result = route_via_to_pad_fn(via_pos, pad, pad_layer, net_id,
                                                    routing_obs, ctx.config, verbose=ctx.verbose)
                if not route_result.success:
                    print(f"\033[91mROUTING FAILED\033[0m")
                elif route_result.segments:
                    new_segments.extend(route_result.segments)
                    traces_added += len(route_result.segments)
                    vias_reused += 1
                    print(f"fallback via at ({via_pos[0]:.2f}, {via_pos[1]:.2f}), {len(route_result.segments)} segs")
                else:
                    vias_reused += 1
                    print(f"fallback via at ({via_pos[0]:.2f}, {via_pos[1]:.2f})")
            else:
                vias_reused += 1
                print(f"fallback via at ({via_pos[0]:.2f}, {via_pos[1]:.2f})")
        else:
            print(f"\033[91mFAILED - no valid position\033[0m")
    else:
        print(f"\033[91mFAILED - no valid position\033[0m")

    return (new_via, new_segments, vias_placed, vias_reused, traces_added, ripped_net_ids)


def generate_multinet_zones(
    layer_nets: Dict[str, List[str]],
    ctx: PlaneCreationContext,
    all_new_vias: List[Dict],
    route_plane_connection_fn,
    voronoi_seed_interval: float = 2.0,
    plane_proximity_radius: float = 3.0,
    plane_proximity_cost: float = 2.0,
    plane_max_iterations: int = 200000
) -> List[str]:
    """
    Generate Voronoi-based zones for multi-net layers.

    Returns list of zone S-expressions.
    """
    zone_sexprs = []

    for layer, nets_on_layer in layer_nets.items():
        if len(nets_on_layer) <= 1:
            continue

        print(f"\n{'='*60}")
        print(f"Computing zone boundaries for multi-net layer {layer}")
        print(f"Nets: {', '.join(nets_on_layer)}")
        print(f"{'='*60}")

        # Build vias_by_net for this layer
        vias_by_net: Dict[int, List[Tuple[float, float]]] = {}
        net_name_to_id = {}
        for net_name in nets_on_layer:
            net_id = next((nid for nid, n in ctx.pcb_data.nets.items() if n.name == net_name), None)
            if net_id is not None:
                net_name_to_id[net_name] = net_id
                vias_by_net[net_id] = []

        # Collect via positions
        for via in all_new_vias:
            if via['net_id'] in vias_by_net:
                vias_by_net[via['net_id']].append((via['x'], via['y']))

        for via in ctx.pcb_data.vias:
            if via.net_id in vias_by_net:
                via_pos = (via.x, via.y)
                if via_pos not in vias_by_net[via.net_id]:
                    vias_by_net[via.net_id].append(via_pos)

        # Check for nets with vias
        nets_with_vias = []
        for net_name in nets_on_layer:
            net_id = net_name_to_id.get(net_name)
            if net_id:
                via_count = len(vias_by_net.get(net_id, []))
                if via_count == 0:
                    print(f"  Warning: Net '{net_name}' has no vias on layer {layer}, skipping zone")
                else:
                    nets_with_vias.append(net_name)
                    print(f"  Net '{net_name}': {via_count} vias")

        if len(nets_with_vias) < 2:
            if nets_with_vias:
                net_name = nets_with_vias[0]
                net_id = net_name_to_id[net_name]
                print(f"  Only '{net_name}' has vias, using full board rectangle")
                zone_sexpr = generate_zone_sexpr(
                    net_id=net_id,
                    net_name=net_name,
                    layer=layer,
                    polygon_points=ctx.zone_polygon,
                    clearance=ctx.zone_clearance,
                    min_thickness=ctx.min_thickness,
                    direct_connect=True
                )
                zone_sexprs.append(zone_sexpr)
            continue

        # Compute zone boundaries using Voronoi
        try:
            result = compute_zone_boundaries(vias_by_net, ctx.board_bounds, return_raw_polygons=True,
                                              board_edge_clearance=ctx.board_edge_clearance, verbose=ctx.verbose)
            zone_polygons, raw_polygons, via_to_polygon_idx = result
        except ValueError as e:
            print(f"  Error computing zone boundaries: {e}")
            net_name = nets_with_vias[0]
            net_id = net_name_to_id[net_name]
            zone_sexpr = generate_zone_sexpr(
                net_id=net_id,
                net_name=net_name,
                layer=layer,
                polygon_points=ctx.zone_polygon,
                clearance=ctx.zone_clearance,
                min_thickness=ctx.min_thickness,
                direct_connect=True
            )
            zone_sexprs.append(zone_sexpr)
            continue

        # Route between disconnected regions
        augmented_vias_by_net = {net_id: list(vias) for net_id, vias in vias_by_net.items()}
        max_iterations = 5

        for iteration in range(max_iterations):
            routes_added_this_iteration = 0

            for net_id, polygons in raw_polygons.items():
                if len(polygons) <= 1:
                    continue

                net = ctx.pcb_data.nets.get(net_id)
                net_name = net.name if net else f"net_{net_id}"

                groups = find_polygon_groups(polygons, verbose=ctx.verbose)
                if len(groups) <= 1:
                    continue

                if iteration == 0:
                    print(f"  Net '{net_name}': Found {len(groups)} disconnected regions")
                else:
                    print(f"  Net '{net_name}': Still {len(groups)} disconnected regions (iteration {iteration + 1})")

                net_vias = augmented_vias_by_net.get(net_id, [])
                if len(net_vias) < 2:
                    continue

                # Build group -> vias mapping
                group_vias: Dict[int, List[Tuple[float, float]]] = {i: [] for i in range(len(groups))}
                for via_pos in net_vias:
                    poly_idx = via_to_polygon_idx.get(net_id, {}).get(via_pos)
                    if poly_idx is not None:
                        for group_idx, group in enumerate(groups):
                            if poly_idx in group:
                                group_vias[group_idx].append(via_pos)
                                break

                # Build other_nets_vias for proximity cost
                other_nets_vias: Dict[int, List[Tuple[float, float]]] = {}
                for other_net_id, other_vias in augmented_vias_by_net.items():
                    if other_net_id != net_id:
                        other_nets_vias[other_net_id] = other_vias

                # Route between groups
                for group_idx in range(1, len(groups)):
                    if not group_vias[0] or not group_vias[group_idx]:
                        continue

                    # Find closest pair between groups
                    best_dist = float('inf')
                    best_via_a, best_via_b = None, None
                    for via_a in group_vias[0]:
                        for via_b in group_vias[group_idx]:
                            dist = math.sqrt((via_a[0] - via_b[0])**2 + (via_a[1] - via_b[1])**2)
                            if dist < best_dist:
                                best_dist = dist
                                best_via_a, best_via_b = via_a, via_b

                    if best_via_a is None:
                        continue

                    print(f"    Routing from region 0 to region {group_idx} (dist: {best_dist:.2f}mm)")

                    route_path = route_plane_connection_fn(
                        via_a=best_via_a,
                        via_b=best_via_b,
                        plane_layer=layer,
                        net_id=net_id,
                        other_nets_vias=other_nets_vias,
                        config=ctx.config,
                        pcb_data=ctx.pcb_data,
                        proximity_radius=plane_proximity_radius,
                        proximity_cost=plane_proximity_cost,
                        max_iterations=plane_max_iterations,
                        verbose=ctx.verbose
                    )

                    if route_path:
                        print(f"      Route found with {len(route_path)} points")
                        routes_added_this_iteration += 1

                        samples = sample_route_for_voronoi(route_path, sample_interval=voronoi_seed_interval)
                        if samples:
                            print(f"      Added {len(samples)} seed points along route")
                            augmented_vias_by_net[net_id].extend(samples)

                        group_vias[0].extend(group_vias[group_idx])
                    else:
                        print(f"      Failed to route - regions may remain disconnected")

            if routes_added_this_iteration == 0:
                break

            # Re-compute zone boundaries
            try:
                result = compute_zone_boundaries(augmented_vias_by_net, ctx.board_bounds,
                                                  return_raw_polygons=True,
                                                  board_edge_clearance=ctx.board_edge_clearance,
                                                  verbose=ctx.verbose)
                zone_polygons, raw_polygons, via_to_polygon_idx = result
            except ValueError as e:
                print(f"  Error re-computing zone boundaries: {e}")
                break

            all_connected = all(len(polys) <= 1 for polys in zone_polygons.values())
            if all_connected:
                print(f"  All regions connected after {iteration + 1} iteration(s)")
                break
        else:
            print(f"  Warning: Max iterations ({max_iterations}) reached")

        # Generate zones
        for net_id, polygons in zone_polygons.items():
            net = ctx.pcb_data.nets.get(net_id)
            net_name = net.name if net else f"net_{net_id}"
            for poly_idx, polygon in enumerate(polygons):
                if len(polygons) > 1:
                    print(f"  Creating zone {poly_idx+1}/{len(polygons)} for '{net_name}'")
                else:
                    print(f"  Creating zone for '{net_name}' with {len(polygon)} vertices")
                zone_sexpr = generate_zone_sexpr(
                    net_id=net_id,
                    net_name=net_name,
                    layer=layer,
                    polygon_points=polygon,
                    clearance=ctx.zone_clearance,
                    min_thickness=ctx.min_thickness,
                    direct_connect=True
                )
                zone_sexprs.append(zone_sexpr)

    return zone_sexprs


def write_results_and_reroute(
    input_file: str,
    output_file: str,
    all_zone_sexprs: List[str],
    all_new_vias: List[Dict],
    all_new_segments: List[Dict],
    all_ripped_net_ids: List[int],
    ctx: PlaneCreationContext,
    dry_run: bool,
    reroute_ripped_nets: bool,
    track_width: float,
    clearance: float,
    grid_step: float
) -> bool:
    """
    Write output file and optionally re-route ripped nets.

    Returns True if successful.
    """
    if dry_run:
        print("\nDry run - no output file written")
        return True

    print(f"\nWriting output to {output_file}...")
    combined_zone_sexpr = '\n'.join(all_zone_sexprs) if all_zone_sexprs else None

    if not write_plane_output(input_file, output_file, combined_zone_sexpr, all_new_vias, all_new_segments,
                               exclude_net_ids=all_ripped_net_ids):
        print("Error writing output file")
        return False

    print(f"Output written to {output_file}")
    print("Note: Open in KiCad and press 'B' to refill zones")

    if all_ripped_net_ids:
        ripped_net_names = []
        for rid in all_ripped_net_ids:
            net = ctx.pcb_data.nets.get(rid)
            if net:
                ripped_net_names.append(net.name)

        if reroute_ripped_nets and ripped_net_names:
            print(f"\n{'='*60}")
            print(f"Re-routing {len(ripped_net_names)} ripped net(s)...")
            print(f"{'='*60}")

            old_recursion_limit = sys.getrecursionlimit()
            sys.setrecursionlimit(max(old_recursion_limit, 100000))
            try:
                routing_layers = [l for l in ctx.all_layers if l not in ctx.plane_layers]
                if not routing_layers:
                    routing_layers = ['F.Cu', 'B.Cu']
                all_copper_layers = list(set(ctx.all_layers + ctx.plane_layers))

                routed, failed, route_time = batch_route(
                    input_file=output_file,
                    output_file=output_file,
                    net_names=ripped_net_names,
                    layers=all_copper_layers,
                    track_width=track_width,
                    clearance=clearance,
                    via_size=ctx.via_size,
                    via_drill=ctx.via_drill,
                    grid_step=grid_step,
                    hole_to_hole_clearance=ctx.hole_to_hole_clearance,
                    verbose=ctx.verbose,
                    minimal_obstacle_cache=True
                )
                print(f"\nRe-routing complete: {routed} routed, {failed} failed in {route_time:.2f}s")
            finally:
                sys.setrecursionlimit(old_recursion_limit)
        else:
            print(f"WARNING: {len(all_ripped_net_ids)} net(s) were removed from output and need re-routing!")
            if ripped_net_names:
                print(f"  Ripped nets: {', '.join(ripped_net_names)}")

    return True
