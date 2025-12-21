"""
Stub layer switching optimization for differential pair routing.

Provides functions to switch stub layers to avoid vias when the
source and target stubs are on different layers.
"""

import math
from typing import List, Optional, Tuple, Dict
from dataclasses import dataclass

from kicad_parser import PCBData, Segment, Via
from routing_config import GridRouteConfig
from routing_utils import get_stub_segments


@dataclass
class StubInfo:
    """Information about a stub endpoint."""
    net_id: int
    x: float
    y: float
    layer: str
    segments: List[Segment]
    pad_x: float
    pad_y: float
    has_pad_via: bool  # True if pad already has via to other layers


def get_stub_info(pcb_data: PCBData, net_id: int, stub_x: float, stub_y: float,
                  stub_layer: str, tolerance: float = 0.05) -> Optional[StubInfo]:
    """
    Gather information about a stub for layer switching analysis.

    Args:
        pcb_data: PCB data
        net_id: Net ID of the stub
        stub_x, stub_y: Position of the stub free end
        stub_layer: Current layer of the stub
        tolerance: Distance tolerance for matching

    Returns:
        StubInfo with all relevant data, or None if stub can't be analyzed
    """
    # Get segments forming the stub
    segments = get_stub_segments(pcb_data, net_id, stub_x, stub_y, stub_layer, tolerance)
    if not segments:
        return None

    # Find the pad this stub connects to
    net_pads = pcb_data.pads_by_net.get(net_id, [])
    if not net_pads:
        return None

    # Find which pad the stub connects to (end opposite from stub_x, stub_y)
    pad_x, pad_y = None, None
    for seg in segments:
        # Check which end is farther from stub position
        dist_start = abs(seg.start_x - stub_x) + abs(seg.start_y - stub_y)
        dist_end = abs(seg.end_x - stub_x) + abs(seg.end_y - stub_y)
        if dist_start > dist_end:
            check_x, check_y = seg.start_x, seg.start_y
        else:
            check_x, check_y = seg.end_x, seg.end_y

        for pad in net_pads:
            if abs(pad.global_x - check_x) < tolerance and abs(pad.global_y - check_y) < tolerance:
                pad_x, pad_y = pad.global_x, pad.global_y
                break
        if pad_x is not None:
            break

    if pad_x is None:
        # Use first pad as fallback
        pad_x, pad_y = net_pads[0].global_x, net_pads[0].global_y

    # Check if pad already has a via (connecting to other layers)
    has_pad_via = False
    for via in pcb_data.vias:
        if via.net_id == net_id:
            dist = math.sqrt((via.x - pad_x) ** 2 + (via.y - pad_y) ** 2)
            if dist < tolerance:
                has_pad_via = True
                break

    return StubInfo(
        net_id=net_id,
        x=stub_x,
        y=stub_y,
        layer=stub_layer,
        segments=segments,
        pad_x=pad_x,
        pad_y=pad_y,
        has_pad_via=has_pad_via
    )


def needs_pad_via_for_switch(stub: StubInfo) -> bool:
    """
    Check if switching this stub requires a new pad via.

    A pad via is needed only when:
    1. Stub is currently on F.Cu (top layer)
    2. No via already exists at the pad position

    Returns:
        True if pad via needed for layer switch
    """
    if stub.layer != 'F.Cu':
        return False  # Already has via from pad to stub layer
    return not stub.has_pad_via


def apply_stub_layer_switch(pcb_data: PCBData, stub: StubInfo, new_layer: str,
                             config: GridRouteConfig, debug: bool = True) -> Tuple[List[Via], List[Dict]]:
    """
    Switch a stub to a new layer by modifying segment layers.

    Args:
        pcb_data: PCB data (segments will be modified in place)
        stub: StubInfo for the stub to switch
        new_layer: Target layer name
        config: Routing configuration
        debug: If True, print debug info about modified segments

    Returns:
        Tuple of (new_vias, segment_modifications):
        - new_vias: List of new vias created (pad vias if switching from F.Cu)
        - segment_modifications: List of dicts with segment layer changes for writing to file
    """
    new_vias = []
    segment_mods = []

    if debug:
        print(f"      apply_stub_layer_switch: net={stub.net_id}, from {stub.layer} to {new_layer}")
        print(f"        Stub at ({stub.x:.2f}, {stub.y:.2f}), pad at ({stub.pad_x:.2f}, {stub.pad_y:.2f})")
        print(f"        Modifying {len(stub.segments)} segments:")

    # Collect modifications and modify segment layers
    for seg in stub.segments:
        old_layer = seg.layer
        if debug:
            print(f"          ({seg.start_x:.2f},{seg.start_y:.2f})->({seg.end_x:.2f},{seg.end_y:.2f}) {old_layer} -> {new_layer}")

        # Record modification for file writing
        segment_mods.append({
            'start': (seg.start_x, seg.start_y),
            'end': (seg.end_x, seg.end_y),
            'net_id': seg.net_id,
            'old_layer': old_layer,
            'new_layer': new_layer
        })

        # Modify in memory
        seg.layer = new_layer

    # Create pad via if needed (switching from F.Cu without existing via)
    if needs_pad_via_for_switch(stub):
        via = Via(
            x=stub.pad_x,
            y=stub.pad_y,
            size=config.via_size,
            drill=config.via_drill,
            layers=['F.Cu', 'B.Cu'],  # Through-hole via
            net_id=stub.net_id
        )
        new_vias.append(via)
        pcb_data.vias.append(via)

    return new_vias, segment_mods
