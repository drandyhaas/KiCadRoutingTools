#!/usr/bin/env python3
"""
Check for orphan trace stubs in a KiCad PCB file.

An orphan stub is a trace endpoint that:
1. Has only one connected segment (degree-1 node)
2. Is NOT near a via
3. Is NOT near a through-hole pad

These represent traces that end without a proper connection point.

Usage:
    python check_orphan_stubs.py input.kicad_pcb [--net NET_NAME] [--layer LAYER]

Examples:
    python check_orphan_stubs.py board.kicad_pcb
    python check_orphan_stubs.py board.kicad_pcb --net "+3.3V" --layer F.Cu
    python check_orphan_stubs.py original.kicad_pcb modified.kicad_pcb --compare
"""

from __future__ import annotations

#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['routing'], 'kind': 'instrument'}

import _path  # noqa: F401  (#522: makes ../py_router importable)

import argparse
import math
import sys
from collections import Counter, defaultdict
from typing import Set, Tuple, Dict, List, Optional

# Add current directory to path for imports
sys.path.insert(0, '.')


def load_pcb_data(filename: str):
    """Load PCB data using kicad_parser."""
    from kicad_parser import parse_kicad_pcb
    return parse_kicad_pcb(filename)


def _pad_probe(pad) -> Tuple[float, float, float, float, float, str]:
    """(cx, cy, w, h, rot_deg, shape) -- the pad's real copper footprint."""
    return (pad.global_x, pad.global_y, pad.size_x, pad.size_y,
            getattr(pad, 'rect_rotation', 0.0) or 0.0,
            (getattr(pad, 'shape', '') or '').lower())


def _point_in_pad(px: float, py: float, probe, margin: float) -> bool:
    """Is (px, py) within `margin` of this pad's copper?

    Run-7 A8: pads were modelled as circles of diameter max(w, h) about the
    centre, which is wrong in BOTH directions and produced measured false
    orphans on two boards. On a square pad the circle misses the corners, so a
    trace ending in the corner copper reads as a dead end; on a 1.5 x 0.9 pad
    the same circle over-credits 0.3mm past the long edges. A false orphan is
    not cosmetic -- one drove a net split that a watcher had to unpick.
    """
    if len(probe) == 3:
        # Legacy (x, y, size) probe: a circle of that diameter. Callers outside
        # this module (and its own older tests) still speak it.
        cx, cy, size = probe
        return math.hypot(px - cx, py - cy) <= size / 2.0 + margin
    cx, cy, w, h, rot, shape = probe
    dx, dy = px - cx, py - cy
    if rot:                                   # into the pad's own frame
        a = math.radians(-rot)
        ca, sa = math.cos(a), math.sin(a)
        dx, dy = dx * ca - dy * sa, dx * sa + dy * ca
    if shape == 'circle' or (shape != 'rect' and abs(w - h) < 1e-9
                             and shape in ('oval', 'roundrect')):
        return math.hypot(dx, dy) <= w / 2.0 + margin
    if shape == 'oval':
        # A capsule: the segment joining the two cap centres, inflated by the
        # short half-axis.
        if w >= h:
            half, r = (w - h) / 2.0, h / 2.0
            t = max(-half, min(half, dx))
            return math.hypot(dx - t, dy) <= r + margin
        half, r = (h - w) / 2.0, w / 2.0
        t = max(-half, min(half, dy))
        return math.hypot(dx, dy - t) <= r + margin
    # rect, roundrect, trapezoid, custom and anything unknown: the bounding
    # rectangle. For custom pads that is the conservative direction for an
    # orphan REPORTER -- over-crediting hides noise, under-crediting invents
    # a dead end that sends someone re-routing good copper.
    return abs(dx) <= w / 2.0 + margin and abs(dy) <= h / 2.0 + margin


def _endpoint_connected(pt: Tuple[float, float], segments: List[Dict],
                        vias: List[Tuple[float, float, float]] = None,
                        ph_pads: List = None,
                        layer_pads: List = None,
                        tol: float = 0.05,
                        end_half_width: float = 0.0) -> bool:
    """True if a degree-1 endpoint actually lands on same-net copper.

    The naive checker treated an endpoint as connected only if within a fixed
    0.15 mm of a via/pad CENTRE and only matched exact shared segment endpoints,
    so it mis-flagged copper that is electrically connected: a stub ending inside
    a via/pad's copper but >0.15 mm from its centre, a tap landing on another
    trace's body (T-junction), or two traces meeting near-coincidentally. This
    tests against the actual copper extents -- via radius, pad half-extent, trace
    half-width -- plus a small overlap margin, matching the connectivity model.

    vias / ph_pads span all layers; layer_pads are SMD pads on this layer. Via
    entries are (x, y, diameter) -- vias really are round. Pad entries are
    `_pad_probe` tuples carrying the pad's own width, height, rotation and
    shape, because a pad is not a circle (see `_point_in_pad`).
    `end_half_width` is half the width of the trace that owns this endpoint.
    The endpoint's own segment is skipped (an endpoint trivially touches itself).
    """
    px, py = pt
    # The endpoint is the CENTRE of the trace's end cap, so its copper reaches
    # half a track width further in every direction. Crediting that is the
    # second half of A8: one measured false orphan sat 0.01mm outside a pad
    # whose copper its end cap overlapped by 0.19mm.
    margin = tol + max(0.0, end_half_width)
    for cx, cy, csize in (vias or ()):        # vias really are circles
        if math.hypot(px - cx, py - cy) < csize / 2 + margin:
            return True
    for group in (ph_pads, layer_pads):
        for probe in (group or ()):
            if _point_in_pad(px, py, probe, margin):
                return True
    for s in segments:
        sx, sy = s['start']
        ex, ey = s['end']
        if (px == sx and py == sy) or (px == ex and py == ey):
            continue  # the endpoint's own segment
        dx, dy = ex - sx, ey - sy
        seg_len_sq = dx * dx + dy * dy
        if seg_len_sq < 1e-12:
            continue
        # Nearest point on the whole segment (endpoints included): catches
        # T-junction taps, near-coincident endpoints, and collinear overlap.
        t = max(0.0, min(1.0, ((px - sx) * dx + (py - sy) * dy) / seg_len_sq))
        if math.hypot(px - (sx + t * dx), py - (sy + t * dy)) < s.get('width', 0.0) / 2 + tol:
            return True
    return False


def find_orphan_stubs(filename: str, net_name: Optional[str] = None,
                      layer: Optional[str] = None) -> Dict[str, Dict[str, Set[Tuple[float, float]]]]:
    """
    Find all orphan stubs in a PCB file.

    Returns dict of net_name -> {layer: set of orphan positions}
    """
    pcb_data = load_pcb_data(filename)

    # Build lookup structures once
    # Vias by net_id, with copper size (vias connect on all layers)
    vias_by_net: Dict[int, List[Tuple[float, float, float]]] = defaultdict(list)
    for via in pcb_data.vias:
        vias_by_net[via.net_id].append((via.x, via.y, getattr(via, 'size', 0.0)))

    # Segments by (net_id, layer)
    segments_by_net_layer: Dict[Tuple[int, str], List[Dict]] = defaultdict(list)
    for seg in pcb_data.segments:
        segments_by_net_layer[(seg.net_id, seg.layer)].append({
            'start': (seg.start_x, seg.start_y),
            'end': (seg.end_x, seg.end_y),
            'width': getattr(seg, 'width', 0.0)
        })

    # Zone outlines by (net_id, layer) (#513 item 17): a plane-repair strap
    # whose free end lands INSIDE a same-net pour polygon is not an orphan --
    # the checker only examined segments/vias/pads, so wide Voronoi straps
    # (nesora_mixer) read as stubs. Outline containment slightly over-credits
    # (a fill void under the endpoint), which is the right bias for an
    # orphan REPORTER -- a missed orphan is noise, a phantom orphan drives
    # pointless retries.
    zones_by_net_layer: Dict[Tuple[int, str], List[List[Tuple[float, float]]]] = defaultdict(list)
    for z in (getattr(pcb_data, 'zones', []) or []):
        if z.net_id and z.layer.endswith('.Cu') and len(z.polygon or []) >= 3:
            zones_by_net_layer[(z.net_id, z.layer)].append(list(z.polygon))

    # Determine which nets to check
    if net_name:
        # Find net_id for the given name
        nets_to_check = [(nid, net) for nid, net in pcb_data.nets.items() if net.name == net_name]
    else:
        # Check all nets with segments
        nets_with_segments = {key[0] for key in segments_by_net_layer.keys()}
        nets_to_check = [(nid, net) for nid, net in pcb_data.nets.items()
                         if net.name and nid in nets_with_segments]

    # Determine which layers to check
    layers_to_check = [layer] if layer else ['F.Cu', 'B.Cu', 'In1.Cu', 'In2.Cu']

    results = {}

    for net_id, net in nets_to_check:
        vias = vias_by_net[net_id]

        # Through-hole pads (drill > 0 or *.Cu) connect on all layers; carry size.
        through_hole_pads = []
        for pad in net.pads:
            if pad.drill > 0 or '*.Cu' in pad.layers:
                through_hole_pads.append(_pad_probe(pad))

        net_results = {}
        for lyr in layers_to_check:
            segments = segments_by_net_layer.get((net_id, lyr), [])
            if not segments:
                continue

            # Find single endpoints (degree-1 nodes)
            endpoints = Counter()
            end_half = {}
            for seg in segments:
                _hw = seg.get('width', 0.0) / 2.0
                for _pt in (seg['start'], seg['end']):
                    endpoints[_pt] += 1
                    end_half[_pt] = max(end_half.get(_pt, 0.0), _hw)
            single_endpoints = [pt for pt, count in endpoints.items() if count == 1]

            if not single_endpoints:
                continue

            # SMD pads on this layer (carry size).
            layer_pads = []
            for pad in net.pads:
                if lyr in pad.layers or '*.Cu' in pad.layers:
                    layer_pads.append(_pad_probe(pad))

            # A degree-1 endpoint is an orphan only if it touches NO same-net
            # copper: not within a via/pad's copper extent, and not on another
            # same-net segment (T-junction / near-coincident / overlap). Matching
            # the real copper geometry instead of a fixed centre tolerance keeps
            # connected stubs from being reported as dead ends.
            zone_polys = zones_by_net_layer.get((net_id, lyr), [])

            def _in_same_net_zone(pt):
                if not zone_polys:
                    return False
                from connectivity import _point_in_polygon
                return any(_point_in_polygon(pt[0], pt[1], poly)
                           for poly in zone_polys)

            orphans = {pt for pt in single_endpoints
                       if not _endpoint_connected(
                           pt, segments, vias, through_hole_pads, layer_pads,
                           end_half_width=end_half.get(pt, 0.0))
                       and not _in_same_net_zone(pt)}

            if orphans:
                net_results[lyr] = orphans

        if net_results:
            results[net.name] = net_results

    return results


def compare_orphans(file1: str, file2: str, net_name: Optional[str] = None,
                   layer: Optional[str] = None) -> Dict:
    """Compare orphan stubs between two files."""
    orphans1 = find_orphan_stubs(file1, net_name, layer)
    orphans2 = find_orphan_stubs(file2, net_name, layer)

    # Collect all orphan positions
    all_orphans_1 = set()
    all_orphans_2 = set()

    for net, layers in orphans1.items():
        for lyr, pts in layers.items():
            for pt in pts:
                all_orphans_1.add((net, lyr, pt))

    for net, layers in orphans2.items():
        for lyr, pts in layers.items():
            for pt in pts:
                all_orphans_2.add((net, lyr, pt))

    new_orphans = all_orphans_2 - all_orphans_1
    removed_orphans = all_orphans_1 - all_orphans_2

    return {
        'file1_total': len(all_orphans_1),
        'file2_total': len(all_orphans_2),
        'new': new_orphans,
        'removed': removed_orphans
    }


def main():
    parser = argparse.ArgumentParser(
        description='Check for orphan trace stubs in KiCad PCB files')
    parser.add_argument('input', help='Input PCB file')
    parser.add_argument('compare_file', nargs='?',
                       help='Second file to compare against (optional)')
    parser.add_argument('--net', help='Only check this net')
    parser.add_argument('--layer', help='Only check this layer')
    parser.add_argument('--compare', action='store_true',
                       help='Compare two files (requires two input files)')

    args = parser.parse_args()

    if args.compare_file or args.compare:
        if not args.compare_file:
            print("Error: --compare requires two input files")
            sys.exit(1)

        result = compare_orphans(args.input, args.compare_file, args.net, args.layer)

        print(f"\nOrphan Stub Comparison")
        print(f"=" * 60)
        print(f"File 1 ({args.input}): {result['file1_total']} orphans")
        print(f"File 2 ({args.compare_file}): {result['file2_total']} orphans")
        print(f"\nNew orphans in file 2: {len(result['new'])}")
        if result['new']:
            for net, lyr, pt in sorted(result['new'])[:20]:
                print(f"  {net} {lyr}: ({pt[0]:.2f}, {pt[1]:.2f})")
            if len(result['new']) > 20:
                print(f"  ... and {len(result['new']) - 20} more")

        print(f"\nRemoved orphans (fixed): {len(result['removed'])}")

        if result['new']:
            sys.exit(1)
    else:
        print(f"Loading {args.input}...")
        orphans = find_orphan_stubs(args.input, args.net, args.layer)

        total = 0
        for net, layers in orphans.items():
            for lyr, pts in layers.items():
                total += len(pts)

        print(f"\nChecking for orphan trace stubs...")

        print("\n" + "=" * 60)
        if not orphans:
            print("NO ORPHAN STUBS FOUND!")
        else:
            print(f"FOUND {total} ORPHAN STUBS:\n")
            for net in sorted(orphans.keys()):
                layers = orphans[net]
                for lyr in sorted(layers.keys()):
                    pts = layers[lyr]
                    print(f"  {net} on {lyr}: {len(pts)} orphans")
                    for pt in sorted(pts)[:5]:
                        print(f"    ({pt[0]:.2f}, {pt[1]:.2f})")
                    if len(pts) > 5:
                        print(f"    ... and {len(pts) - 5} more")

        print("=" * 60)

        if orphans:
            sys.exit(1)


if __name__ == '__main__':
    import cli_banner; cli_banner.install()  # CMD/EXIT self-echo (run-3 B1)
    main()
