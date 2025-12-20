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


def point_key(x: float, y: float, layer: str, tolerance: float = 0.01) -> Tuple[int, int, str]:
    """Create a hashable key for a point, quantized to tolerance."""
    return (round(x / tolerance), round(y / tolerance), layer)


def points_match(x1: float, y1: float, x2: float, y2: float, tolerance: float = 0.01) -> bool:
    """Check if two points are within tolerance."""
    return abs(x1 - x2) < tolerance and abs(y1 - y2) < tolerance


def check_net_connectivity(net_id: int, segments: List[Segment], vias: List[Via],
                           pads: List[Pad], tolerance: float = 0.2) -> Dict:
    """Check connectivity for a single net.

    Returns dict with:
        - connected: bool - whether all pads are connected
        - num_components: int - number of disconnected components
        - pad_components: dict mapping pad location to component id
        - disconnected_pads: list of pad locations not connected to the main component
    """
    uf = UnionFind()
    all_copper_layers = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']

    # Collect all points with their actual coordinates and size info
    # Each point: (x, y, layer, point_id, size) where size is track_width or via_size
    all_points = []
    point_id = 0

    # Add segment endpoints
    for seg in segments:
        start_id = point_id
        all_points.append((seg.start_x, seg.start_y, seg.layer, start_id, seg.width))
        point_id += 1
        end_id = point_id
        all_points.append((seg.end_x, seg.end_y, seg.layer, end_id, seg.width))
        point_id += 1
        # Connect segment's own endpoints
        uf.union(start_id, end_id)

    # Add vias - they connect all layers at one location
    for via in vias:
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
            via_ids.append(point_id)
            point_id += 1
        # Connect all via layers together
        for vid in via_ids[1:]:
            uf.union(via_ids[0], vid)

    # Add pads (use a reasonable default size for pads)
    pad_ids = []
    pad_locations = []
    copper_layers = set(all_copper_layers)
    for pad in pads:
        for layer in pad.layers:
            if layer not in copper_layers:
                continue
            pad_size = 0.4  # Default pad connection tolerance
            all_points.append((pad.global_x, pad.global_y, layer, point_id, pad_size))
            pad_ids.append(point_id)
            pad_locations.append((pad.global_x, pad.global_y, layer, pad.component_ref))
            point_id += 1

    # Connect all points that are within tolerance on the same layer
    # Use size/4 as the tolerance for each point pair (use the larger of the two)
    for i, (x1, y1, l1, id1, size1) in enumerate(all_points):
        for j in range(i + 1, len(all_points)):
            x2, y2, l2, id2, size2 = all_points[j]
            if l1 != l2:
                continue
            # Use max(size1, size2) / 4 as tolerance
            point_tolerance = max(size1, size2) / 4
            if points_match(x1, y1, x2, y2, point_tolerance):
                uf.union(id1, id2)

    # Check if all pads are in the same component
    if not pad_ids:
        return {
            'connected': True,
            'num_components': 0,
            'pad_components': {},
            'disconnected_pads': [],
            'message': 'No pads found for this net'
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

    return {
        'connected': len(unique_roots) == 1,
        'num_components': len(unique_roots),
        'pad_components': {loc: uf.find(pid) for pid, loc in zip(pad_ids, pad_locations)},
        'disconnected_pads': disconnected,
        'message': None
    }


def run_connectivity_check(pcb_file: str, net_patterns: Optional[List[str]] = None,
                           tolerance: float = 0.1, quiet: bool = False) -> List[Dict]:
    """Run connectivity checks on the PCB file.

    Args:
        pcb_file: Path to the KiCad PCB file
        net_patterns: Optional list of net name patterns (fnmatch style) to check.
        tolerance: Connection tolerance in mm (default: 0.01mm)
        quiet: If True, only print a summary line unless there are issues

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

        result = check_net_connectivity(net_id, segments, vias, pads, tolerance)

        if not result['connected']:
            issues.append({
                'net_id': net_id,
                'net_name': net_name,
                'num_segments': len(segments),
                'num_vias': len(vias),
                'num_pads': len(pads),
                'num_components': result['num_components'],
                'disconnected_pads': result['disconnected_pads'],
                'message': result.get('message')
            })

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
    parser.add_argument('--tolerance', '-t', type=float, default=0.01,
                        help='Connection tolerance in mm (default: 0.01)')
    parser.add_argument('--quiet', '-q', action='store_true',
                        help='Only print a summary line unless there are issues')

    args = parser.parse_args()

    issues = run_connectivity_check(args.pcb, args.nets, args.tolerance, args.quiet)
    sys.exit(1 if issues else 0)
