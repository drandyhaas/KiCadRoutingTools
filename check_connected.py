"""
Connectivity Checker - Verify that tracks form fully connected routes from source to target pads.
"""

import sys
import argparse
import math
import fnmatch
from typing import List, Dict, Set, Tuple, Optional
from collections import defaultdict
from kicad_parser import parse_kicad_pcb, Segment, Via, Pad, PCBData


def matches_any_pattern(name: str, patterns: List[str]) -> bool:
    """Check if a net name matches any of the given patterns (fnmatch style)."""
    for pattern in patterns:
        if fnmatch.fnmatch(name, pattern):
            return True
    return False


class UnionFind:
    """Union-Find data structure for connectivity checking."""
    def __init__(self):
        self.parent = {}
        self.rank = {}

    def find(self, x):
        if x not in self.parent:
            self.parent[x] = x
            self.rank[x] = 0
        if self.parent[x] != x:
            self.parent[x] = self.find(self.parent[x])
        return self.parent[x]

    def union(self, x, y):
        px, py = self.find(x), self.find(y)
        if px == py:
            return
        if self.rank[px] < self.rank[py]:
            px, py = py, px
        self.parent[py] = px
        if self.rank[px] == self.rank[py]:
            self.rank[px] += 1


def point_key(x: float, y: float, layer: str, tolerance: float = 0.02) -> Tuple[int, int, str]:
    """Create a hashable key for a point, quantized to tolerance."""
    return (round(x / tolerance), round(y / tolerance), layer)


def points_match(x1: float, y1: float, x2: float, y2: float, tolerance: float = 0.02) -> bool:
    """Check if two points are within tolerance."""
    return abs(x1 - x2) < tolerance and abs(y1 - y2) < tolerance


def check_net_connectivity(net_id: int, segments: List[Segment], vias: List[Via],
                           pads: List[Pad], tolerance: float = 0.02,
                           verbose: bool = False) -> Dict:
    """Check connectivity for a single net.

    Returns dict with:
        - connected: bool - whether all pads are connected
        - num_components: int - number of disconnected components
        - pad_components: dict mapping pad location to component id
        - disconnected_pads: list of pad locations not connected to the main component
        - debug_info: dict with detailed component info (when verbose=True)
    """
    uf = UnionFind()
    all_copper_layers = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']

    # Collect all points with their actual coordinates and size info
    # Each point: (x, y, layer, point_id, size, type, extra_info)
    all_points = []
    point_id = 0
    point_info = {}  # Maps point_id to descriptive info

    # Add segment endpoints
    for seg_idx, seg in enumerate(segments):
        start_id = point_id
        all_points.append((seg.start_x, seg.start_y, seg.layer, start_id, seg.width))
        point_info[start_id] = ('segment_start', seg_idx, seg.layer, seg.start_x, seg.start_y)
        point_id += 1
        end_id = point_id
        all_points.append((seg.end_x, seg.end_y, seg.layer, end_id, seg.width))
        point_info[end_id] = ('segment_end', seg_idx, seg.layer, seg.end_x, seg.end_y)
        point_id += 1
        # Connect segment's own endpoints
        uf.union(start_id, end_id)

    # Add vias - they connect all layers at one location
    for via_idx, via in enumerate(vias):
        if via.layers:
            if 'F.Cu' in via.layers and 'B.Cu' in via.layers:
                via_layers = all_copper_layers
            else:
                via_layers = via.layers
        else:
            via_layers = all_copper_layers

        via_size = getattr(via, 'size', 0.6)  # Default via size if not available
        via_ids = []
        for layer in via_layers:
            all_points.append((via.x, via.y, layer, point_id, via_size))
            point_info[point_id] = ('via', via_idx, layer, via.x, via.y)
            via_ids.append(point_id)
            point_id += 1
        # Connect all via layers together
        for vid in via_ids[1:]:
            uf.union(via_ids[0], vid)

    # Add pads (use a reasonable default size for pads)
    pad_ids = []
    pad_locations = []
    copper_layers = set(all_copper_layers)
    for pad_idx, pad in enumerate(pads):
        for layer in pad.layers:
            if layer not in copper_layers:
                continue
            pad_size = 0.4  # Default pad connection tolerance
            all_points.append((pad.global_x, pad.global_y, layer, point_id, pad_size))
            point_info[point_id] = ('pad', pad_idx, layer, pad.global_x, pad.global_y, pad.component_ref)
            pad_ids.append(point_id)
            pad_locations.append((pad.global_x, pad.global_y, layer, pad.component_ref))
            point_id += 1

    # Connect all points that are within tolerance on the same layer
    # Use size/4 as the tolerance for each point pair (use the larger of the two)
    # but ensure we never go below the minimum tolerance parameter
    for i, (x1, y1, l1, id1, size1) in enumerate(all_points):
        for j in range(i + 1, len(all_points)):
            x2, y2, l2, id2, size2 = all_points[j]
            if l1 != l2:
                continue
            # Use max(size1, size2) / 4, but at least the minimum tolerance
            point_tolerance = max(max(size1, size2) / 4, tolerance)
            if points_match(x1, y1, x2, y2, point_tolerance):
                uf.union(id1, id2)

    # Check if all pads are in the same component
    if not pad_ids:
        return {
            'connected': True,
            'num_components': 0,
            'pad_components': {},
            'disconnected_pads': [],
            'message': 'No pads found for this net',
            'debug_info': None
        }

    pad_roots = [uf.find(pid) for pid in pad_ids]
    unique_roots = set(pad_roots)

    # Find which pads are disconnected from the main group
    root_counts = defaultdict(int)
    for root in pad_roots:
        root_counts[root] += 1
    main_root = max(root_counts.keys(), key=lambda r: root_counts[r])

    disconnected = []
    for pid, loc in zip(pad_ids, pad_locations):
        if uf.find(pid) != main_root:
            disconnected.append(loc)

    # Build debug info if verbose
    debug_info = None
    if verbose and len(unique_roots) > 1:
        # Group all points by component
        components = defaultdict(list)
        for pt in all_points:
            x, y, layer, pid, size = pt
            root = uf.find(pid)
            info = point_info.get(pid, ('unknown',))
            components[root].append((x, y, layer, info))

        # For each component, find boundary points (potential disconnection points)
        component_summaries = {}
        for root, points in components.items():
            by_layer = defaultdict(list)
            for x, y, layer, info in points:
                by_layer[layer].append((x, y, info))

            # Summarize component
            summary = {
                'layers': list(by_layer.keys()),
                'points_by_layer': {l: len(pts) for l, pts in by_layer.items()},
                'has_pads': any(info[0] == 'pad' for _, _, _, info in points),
                'has_vias': any(info[0] == 'via' for _, _, _, info in points),
                'sample_points': [(x, y, layer, info[0]) for x, y, layer, info in points[:10]]
            }
            component_summaries[root] = summary

        debug_info = {
            'components': component_summaries,
            'component_points': dict(components),  # Raw points by component for gap analysis
            'main_root': main_root,
            'all_points': all_points,
            'point_info': point_info,
            'segments': segments,
            'vias': vias
        }

    return {
        'connected': len(unique_roots) == 1,
        'num_components': len(unique_roots),
        'pad_components': {loc: uf.find(pid) for pid, loc in zip(pad_ids, pad_locations)},
        'disconnected_pads': disconnected,
        'message': None,
        'debug_info': debug_info
    }


def find_gap_between_components(debug_info: Dict, tolerance: float) -> Optional[Dict]:
    """Analyze debug info to find where the gap is between disconnected components.

    Returns information about the likely disconnection point.
    """
    if not debug_info:
        return None

    component_summaries = debug_info['components']
    component_points = debug_info.get('component_points', {})
    all_points = debug_info['all_points']
    segments = debug_info['segments']
    vias = debug_info['vias']

    if len(component_points) < 2:
        return None

    # Find the closest points between different components
    min_dist = float('inf')
    gap_info = None

    roots = list(component_points.keys())
    for i, root1 in enumerate(roots):
        for root2 in roots[i+1:]:
            for pt1 in component_points[root1]:
                x1, y1, l1, info1 = pt1[0], pt1[1], pt1[2], pt1[3]
                for pt2 in component_points[root2]:
                    x2, y2, l2, info2 = pt2[0], pt2[1], pt2[2], pt2[3]
                    # Only compare points on the same layer (that's where connections happen)
                    if l1 != l2:
                        continue
                    dist = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
                    if dist < min_dist:
                        min_dist = dist
                        info1_type = info1[0] if isinstance(info1, tuple) else info1
                        info2_type = info2[0] if isinstance(info2, tuple) else info2
                        gap_info = {
                            'distance': dist,
                            'point1': (x1, y1, l1, info1_type),
                            'point2': (x2, y2, l2, info2_type),
                            'component1_root': root1,
                            'component2_root': root2
                        }

    # Also find if there are points on different layers at the same position
    # (missing via situation)
    for root1 in roots:
        for root2 in roots:
            if root1 >= root2:
                continue
            for pt1 in component_points[root1]:
                x1, y1, l1, info1 = pt1[0], pt1[1], pt1[2], pt1[3]
                for pt2 in component_points[root2]:
                    x2, y2, l2, info2 = pt2[0], pt2[1], pt2[2], pt2[3]
                    if l1 == l2:
                        continue
                    if abs(x1 - x2) < tolerance and abs(y1 - y2) < tolerance:
                        # Points at same position but different layers and not connected
                        info1_type = info1[0] if isinstance(info1, tuple) else info1
                        info2_type = info2[0] if isinstance(info2, tuple) else info2
                        return {
                            'type': 'missing_via',
                            'position': (x1, y1),
                            'layers': [l1, l2],
                            'point1_type': info1_type,
                            'point2_type': info2_type,
                            'message': f"Gap at ({x1:.3f}, {y1:.3f}): {info1_type} on {l1} not connected to {info2_type} on {l2}"
                        }

    if gap_info:
        gap_info['type'] = 'gap_on_layer'
        gap_info['message'] = (
            f"Nearest gap: {gap_info['distance']:.3f}mm on {gap_info['point1'][2]} between "
            f"{gap_info['point1'][3]} at ({gap_info['point1'][0]:.3f}, {gap_info['point1'][1]:.3f}) and "
            f"{gap_info['point2'][3]} at ({gap_info['point2'][0]:.3f}, {gap_info['point2'][1]:.3f})"
        )

    return gap_info


def run_connectivity_check(pcb_file: str, net_patterns: Optional[List[str]] = None,
                           tolerance: float = 0.02, quiet: bool = False,
                           verbose: bool = False) -> List[Dict]:
    """Run connectivity checks on the PCB file.

    Args:
        pcb_file: Path to the KiCad PCB file
        net_patterns: Optional list of net name patterns (fnmatch style) to check.
        tolerance: Minimum connection tolerance in mm (default: 0.02mm)
        quiet: If True, only print a summary line unless there are issues
        verbose: If True, show detailed info about where breaks are

    Returns:
        List of connectivity issues found
    """
    if quiet and net_patterns:
        # Print a brief summary line in quiet mode
        print(f"Checking {', '.join(net_patterns)} for connectivity...", end=" ", flush=True)
    elif not quiet:
        print(f"Loading {pcb_file}...")

    pcb_data = parse_kicad_pcb(pcb_file)

    if not quiet:
        total_pads = sum(len(pads) for pads in pcb_data.pads_by_net.values())
        print(f"Found {len(pcb_data.segments)} segments, {len(pcb_data.vias)} vias, {total_pads} pads")

    # Group segments by net
    segments_by_net = defaultdict(list)
    for seg in pcb_data.segments:
        segments_by_net[seg.net_id].append(seg)

    # Group vias by net
    vias_by_net = defaultdict(list)
    for via in pcb_data.vias:
        vias_by_net[via.net_id].append(via)

    # Use existing pads_by_net from pcb_data
    pads_by_net = pcb_data.pads_by_net

    # Determine which nets to check
    nets_to_check = []
    for net_id, net_info in pcb_data.nets.items():
        if net_patterns:
            if matches_any_pattern(net_info.name, net_patterns):
                nets_to_check.append((net_id, net_info.name))
        else:
            # Only check nets that have both segments and pads
            if net_id in segments_by_net and net_id in pads_by_net:
                nets_to_check.append((net_id, net_info.name))

    if not quiet:
        if net_patterns:
            print(f"Checking {len(nets_to_check)} nets matching: {net_patterns}")
        else:
            print(f"Checking {len(nets_to_check)} routed nets")

    issues = []

    for net_id, net_name in nets_to_check:
        segments = segments_by_net.get(net_id, [])
        vias = vias_by_net.get(net_id, [])
        pads = pads_by_net.get(net_id, [])

        result = check_net_connectivity(net_id, segments, vias, pads, tolerance, verbose=verbose)

        if not result['connected']:
            issue = {
                'net_id': net_id,
                'net_name': net_name,
                'num_segments': len(segments),
                'num_vias': len(vias),
                'num_pads': len(pads),
                'num_components': result['num_components'],
                'disconnected_pads': result['disconnected_pads'],
                'message': result.get('message'),
                'debug_info': result.get('debug_info')
            }

            # Analyze the gap
            if verbose and result.get('debug_info'):
                gap = find_gap_between_components(result['debug_info'], tolerance)
                if gap:
                    issue['gap_info'] = gap

            issues.append(issue)

    # Report results
    if quiet:
        if issues:
            print(f"FAILED ({len(issues)} issues)")
        else:
            print("OK")
            return issues

    # Print detailed results (always for non-quiet, or when issues in quiet mode)
    if not quiet or issues:
        print("\n" + "=" * 60 if not quiet else "=" * 60)
        if issues:
            print(f"FOUND {len(issues)} CONNECTIVITY ISSUES:\n")

            for issue in issues:
                print(f"\n  {issue['net_name']} (net {issue['net_id']}):")
                print(f"    Segments: {issue['num_segments']}, Vias: {issue['num_vias']}, Pads: {issue['num_pads']}")
                print(f"    Disconnected components: {issue['num_components']}")
                if issue['disconnected_pads']:
                    print(f"    Disconnected pads:")
                    for loc in issue['disconnected_pads'][:5]:
                        print(f"      ({loc[0]:.2f}, {loc[1]:.2f}) on {loc[2]} [{loc[3]}]")
                    if len(issue['disconnected_pads']) > 5:
                        print(f"      ... and {len(issue['disconnected_pads']) - 5} more")
                if issue.get('gap_info'):
                    gap = issue['gap_info']
                    print(f"    Break location: {gap['message']}")
                    if verbose and gap.get('type') == 'gap_on_layer':
                        debug = issue.get('debug_info')
                        if debug:
                            # Show component details
                            for root, summary in debug['components'].items():
                                is_main = root == debug['main_root']
                                print(f"    Component {'(main)' if is_main else '(disconnected)'}: "
                                      f"layers={summary['layers']}, "
                                      f"has_pads={summary['has_pads']}, has_vias={summary['has_vias']}")
                                if verbose:
                                    print(f"      Points by layer: {summary['points_by_layer']}")
                if issue.get('message'):
                    print(f"    Note: {issue['message']}")
        else:
            print("ALL NETS FULLY CONNECTED!")

        print("=" * 60)
    return issues


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Check PCB for track connectivity (disconnected routes)')
    parser.add_argument('pcb', help='Input PCB file')
    parser.add_argument('--nets', '-n', nargs='+', default=None,
                        help='Net name patterns to check (fnmatch wildcards supported, e.g., "*lvds*")')
    parser.add_argument('--tolerance', '-t', type=float, default=0.02,
                        help='Minimum connection tolerance in mm (default: 0.02)')
    parser.add_argument('--quiet', '-q', action='store_true',
                        help='Only print a summary line unless there are issues')
    parser.add_argument('--verbose', '-v', action='store_true',
                        help='Show detailed break location info for disconnected nets')

    args = parser.parse_args()

    issues = run_connectivity_check(args.pcb, args.nets, args.tolerance, args.quiet, args.verbose)
    sys.exit(1 if issues else 0)
