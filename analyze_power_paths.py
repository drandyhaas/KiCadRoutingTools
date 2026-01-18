#!/usr/bin/env python3
"""
AI-powered power path analysis for KiCad PCB files.

This module provides functions to:
1. Extract component information for AI analysis
2. Accept AI classifications of component roles
3. Trace current paths from sinks to sources
4. Output nets that carry significant current

Usage:
    from analyze_power_paths import (
        extract_components_for_analysis,
        classify_component,
        trace_power_paths,
        get_power_net_recommendations
    )
"""

from dataclasses import dataclass, field
from typing import Dict, List, Set, Optional, Tuple
from enum import Enum

from kicad_parser import parse_kicad_pcb, PCBData, Footprint, Pad


class ComponentRole(Enum):
    """Role of a component in power distribution."""
    POWER_SOURCE = "power_source"      # Outputs current (regulators, power inputs)
    CURRENT_SINK = "current_sink"      # Consumes current (ICs, LEDs, motors)
    PASS_THROUGH = "pass_through"      # Current flows through (inductors, fuses, switches)
    SHUNT = "shunt"                    # Branches off main path (decoupling caps, pull-ups)
    UNKNOWN = "unknown"                # Needs AI classification


@dataclass
class ComponentInfo:
    """Information about a component for analysis."""
    ref: str                           # Reference designator (U102, R101, etc.)
    value: str                         # Component value/part number
    footprint_name: str                # Footprint type
    pad_count: int                     # Number of pads
    net_connections: Dict[str, str]    # pad_number -> net_name
    pin_functions: Dict[str, str]      # pad_number -> pinfunction
    pin_types: Dict[str, str]          # pad_number -> pintype
    role: ComponentRole = ComponentRole.UNKNOWN
    current_rating_ma: Optional[float] = None  # Estimated current in mA
    notes: str = ""                    # Additional notes from AI analysis


@dataclass
class PowerPath:
    """A path through which current flows."""
    source_component: str              # Component ref that sources current
    sink_component: str                # Component ref that sinks current
    nets_in_path: List[str]            # Net names along the path
    components_in_path: List[str]      # Component refs along the path
    estimated_current_ma: float        # Estimated current in mA


def extract_components_for_analysis(pcb_data: PCBData) -> Dict[str, ComponentInfo]:
    """
    Extract component information from PCB for AI analysis.

    Returns a dict of ref -> ComponentInfo for all components.
    Components are pre-classified where obvious, marked UNKNOWN otherwise.
    """
    components = {}

    for ref, fp in pcb_data.footprints.items():
        # Build net connections and pin info
        net_connections = {}
        pin_functions = {}
        pin_types = {}

        for pad in fp.pads:
            net_name = pcb_data.nets[pad.net_id].name if pad.net_id in pcb_data.nets else "(unconnected)"
            net_connections[pad.pad_number] = net_name
            pin_functions[pad.pad_number] = pad.pinfunction or ""
            pin_types[pad.pad_number] = pad.pintype or ""

        info = ComponentInfo(
            ref=ref,
            value=fp.value,
            footprint_name=fp.footprint_name,
            pad_count=len(fp.pads),
            net_connections=net_connections,
            pin_functions=pin_functions,
            pin_types=pin_types
        )

        # Pre-classify obvious cases
        info.role = _auto_classify_component(ref, fp, pcb_data)

        components[ref] = info

    return components


def _auto_classify_component(ref: str, fp: Footprint, pcb_data: PCBData) -> ComponentRole:
    """
    Automatically classify obvious component types.
    Returns UNKNOWN for components that need AI analysis.
    """
    ref_upper = ref.upper()

    # Check for power_out pins (voltage regulators)
    has_power_out = any(p.pintype == 'power_out' for p in fp.pads)
    if has_power_out:
        return ComponentRole.POWER_SOURCE

    # Capacitors - check if decoupling (to GND) or series
    if ref_upper.startswith('C') and len(ref) > 1 and ref[1].isdigit():
        if len(fp.pads) == 2:
            net_names = [pcb_data.nets.get(p.net_id, type('', (), {'name': ''})()).name
                        for p in fp.pads if p.net_id]
            # If one side is GND, it's a decoupling cap (shunt)
            if any('GND' in n.upper() for n in net_names):
                return ComponentRole.SHUNT
        return ComponentRole.UNKNOWN  # Could be series cap, needs analysis

    # Resistors - usually shunts (pull-ups) unless in series power path
    if ref_upper.startswith('R') and len(ref) > 1 and ref[1].isdigit():
        return ComponentRole.SHUNT  # Default to shunt, AI can override

    # Inductors - pass-through in power path
    if ref_upper.startswith('L') and len(ref) > 1 and ref[1].isdigit():
        return ComponentRole.PASS_THROUGH

    # Ferrite beads - pass-through
    if ref_upper.startswith('FB') and len(ref) > 2 and ref[2].isdigit():
        return ComponentRole.PASS_THROUGH

    # Fuses - pass-through
    if ref_upper.startswith('F') and len(ref) > 1 and ref[1].isdigit():
        return ComponentRole.PASS_THROUGH

    # LEDs - current sinks
    if ref_upper.startswith('LED'):
        return ComponentRole.CURRENT_SINK
    if ref_upper.startswith('D') and len(ref) > 1 and ref[1].isdigit():
        # Could be LED or protection diode - needs analysis
        return ComponentRole.UNKNOWN

    # Switches - pass-through
    if ref_upper.startswith('SW') or (ref_upper.startswith('S') and len(ref) > 1 and ref[1].isdigit()):
        return ComponentRole.PASS_THROUGH

    # Connectors/Terminal blocks - potential power sources
    if ref_upper.startswith(('J', 'P', 'TB', 'CN')):
        return ComponentRole.UNKNOWN  # Could be power input or signal

    # ICs - usually current sinks
    if ref_upper.startswith('U') and len(ref) > 1 and ref[1].isdigit():
        return ComponentRole.CURRENT_SINK

    # Voltage regulators by ref
    if ref_upper.startswith('VR') and len(ref) > 2 and ref[2].isdigit():
        return ComponentRole.POWER_SOURCE

    # Transistors - could be switch or sink
    if ref_upper.startswith('Q') and len(ref) > 1 and ref[1].isdigit():
        return ComponentRole.UNKNOWN

    return ComponentRole.UNKNOWN


def get_components_needing_analysis(components: Dict[str, ComponentInfo]) -> List[ComponentInfo]:
    """
    Get list of components that need AI analysis to determine their role.
    """
    unknown = [c for c in components.values() if c.role == ComponentRole.UNKNOWN]

    # Sort by likely importance (ICs first, then connectors, then others)
    def sort_key(c):
        if c.ref.startswith('U'):
            return (0, c.ref)
        if c.ref.startswith(('J', 'P', 'TB')):
            return (1, c.ref)
        if c.ref.startswith('VR'):
            return (2, c.ref)
        return (3, c.ref)

    return sorted(unknown, key=sort_key)


def classify_component(components: Dict[str, ComponentInfo],
                       ref: str,
                       role: ComponentRole,
                       current_rating_ma: Optional[float] = None,
                       notes: str = "") -> None:
    """
    Set the classification for a component based on AI analysis.

    Args:
        components: The components dict to modify
        ref: Component reference designator
        role: The determined role
        current_rating_ma: Estimated current in milliamps
        notes: Any notes from the analysis
    """
    if ref in components:
        components[ref].role = role
        components[ref].current_rating_ma = current_rating_ma
        components[ref].notes = notes


def trace_power_paths(pcb_data: PCBData,
                      components: Dict[str, ComponentInfo]) -> List[PowerPath]:
    """
    Trace current paths from sinks back to sources through pass-through components.

    Returns list of PowerPath objects describing each current path.
    """
    paths = []

    # Build adjacency: net_id -> [(component_ref, other_net_id)]
    net_adjacency: Dict[int, List[Tuple[str, int]]] = {}

    for ref, comp in components.items():
        if comp.role != ComponentRole.PASS_THROUGH:
            continue

        # Get connected nets (excluding unconnected)
        connected_nets = []
        for pad_num, net_name in comp.net_connections.items():
            if 'unconnected' in net_name.lower():
                continue
            # Find net_id
            for net_id, net in pcb_data.nets.items():
                if net.name == net_name:
                    connected_nets.append(net_id)
                    break

        # If exactly 2 connected nets, create adjacency
        if len(connected_nets) == 2:
            net_a, net_b = connected_nets
            if net_a not in net_adjacency:
                net_adjacency[net_a] = []
            if net_b not in net_adjacency:
                net_adjacency[net_b] = []
            net_adjacency[net_a].append((ref, net_b))
            net_adjacency[net_b].append((ref, net_a))

    # Also add regulator connections (output -> input)
    for ref, comp in components.items():
        if comp.role != ComponentRole.POWER_SOURCE:
            continue

        output_nets = []
        input_nets = []

        for pad_num, pintype in comp.pin_types.items():
            net_name = comp.net_connections.get(pad_num, "")
            if 'unconnected' in net_name.lower():
                continue

            pinfunction = comp.pin_functions.get(pad_num, "").upper()

            # Find net_id
            net_id = None
            for nid, net in pcb_data.nets.items():
                if net.name == net_name:
                    net_id = nid
                    break

            if net_id is None:
                continue

            if pintype == 'power_out' or pinfunction in ('OUT', 'VOUT', 'OUTPUT'):
                output_nets.append(net_id)
            elif pinfunction in ('IN', 'VIN', 'INPUT'):
                input_nets.append(net_id)

        # Connect outputs to inputs
        for out_net in output_nets:
            for in_net in input_nets:
                if out_net != in_net:
                    if out_net not in net_adjacency:
                        net_adjacency[out_net] = []
                    if in_net not in net_adjacency:
                        net_adjacency[in_net] = []
                    net_adjacency[out_net].append((ref, in_net))
                    net_adjacency[in_net].append((ref, out_net))

    # Find sink nets (power_in pins on sink components)
    sink_nets: Dict[int, str] = {}  # net_id -> sink_component_ref
    for ref, comp in components.items():
        if comp.role != ComponentRole.CURRENT_SINK:
            continue
        for pad_num, pintype in comp.pin_types.items():
            if pintype == 'power_in':
                net_name = comp.net_connections.get(pad_num, "")
                for net_id, net in pcb_data.nets.items():
                    if net.name == net_name:
                        sink_nets[net_id] = ref
                        break

    # Find source nets (power_out pins on source components, or power input connectors)
    source_nets: Dict[int, str] = {}  # net_id -> source_component_ref
    for ref, comp in components.items():
        if comp.role != ComponentRole.POWER_SOURCE:
            continue
        for pad_num, pintype in comp.pin_types.items():
            net_name = comp.net_connections.get(pad_num, "")
            for net_id, net in pcb_data.nets.items():
                if net.name == net_name:
                    # For regulators, the output is the source
                    # For power inputs (connectors), all connected nets could be sources
                    pinfunction = comp.pin_functions.get(pad_num, "").upper()
                    if pintype == 'power_out' or pinfunction in ('OUT', 'VOUT', 'OUTPUT'):
                        source_nets[net_id] = ref
                    elif ref.startswith(('J', 'P', 'TB')):
                        # Power input connector - could be a source
                        source_nets[net_id] = ref
                    break

    # BFS from each sink to find paths to sources
    for sink_net_id, sink_ref in sink_nets.items():
        visited = {sink_net_id: (None, None)}  # net_id -> (prev_net_id, via_component)
        queue = [sink_net_id]

        while queue:
            net_id = queue.pop(0)

            # Check if we reached a source
            if net_id in source_nets and net_id != sink_net_id:
                # Reconstruct path
                path_nets = []
                path_components = []
                current = net_id
                while current is not None:
                    path_nets.append(pcb_data.nets[current].name)
                    prev_net, via_comp = visited[current]
                    if via_comp:
                        path_components.append(via_comp)
                    current = prev_net

                path_nets.reverse()
                path_components.reverse()

                # Estimate current based on sink
                sink_comp = components.get(sink_ref)
                current_ma = sink_comp.current_rating_ma if sink_comp and sink_comp.current_rating_ma else 100.0

                paths.append(PowerPath(
                    source_component=source_nets[net_id],
                    sink_component=sink_ref,
                    nets_in_path=path_nets,
                    components_in_path=path_components,
                    estimated_current_ma=current_ma
                ))

            # Expand through pass-through components
            if net_id in net_adjacency:
                for comp_ref, other_net in net_adjacency[net_id]:
                    if other_net not in visited:
                        visited[other_net] = (net_id, comp_ref)
                        queue.append(other_net)

    return paths


def get_power_net_recommendations(pcb_data: PCBData,
                                  components: Dict[str, ComponentInfo],
                                  paths: List[PowerPath],
                                  min_current_ma: float = 50.0) -> Dict[str, float]:
    """
    Get recommended track widths for power nets based on traced paths.

    Args:
        pcb_data: Parsed PCB data
        components: Classified components
        paths: Traced power paths
        min_current_ma: Minimum current to consider for power routing

    Returns:
        Dict of net_name -> recommended_width_mm
    """
    # Accumulate current on each net
    net_currents: Dict[str, float] = {}

    for path in paths:
        for net_name in path.nets_in_path:
            if net_name not in net_currents:
                net_currents[net_name] = 0.0
            net_currents[net_name] += path.estimated_current_ma

    # Also add direct power connections (power_in pins on sinks, power_out on sources)
    # These may not appear in traced paths if there's no pass-through component
    # Also detect mislabeled power pins by their function name
    power_pin_keywords = ('VCC', 'VDD', 'VSS', 'GND', 'VCCA', 'VSSA', 'VDDA',
                          'VDDPLL', 'VCCPLL', 'GNDPLL', 'VRH', 'VRL', 'AVDD', 'AVSS')

    def is_power_pin(pinfunction: str, pintype: str) -> bool:
        """Check if a pin is a power pin by function name or pintype."""
        if pintype in ('power_in', 'power_out'):
            return True
        if pinfunction:
            fn_upper = pinfunction.upper()
            # Check for exact matches or prefix matches
            return any(fn_upper == kw or fn_upper.startswith(kw) for kw in power_pin_keywords)
        return False

    for ref, comp in components.items():
        if comp.role == ComponentRole.CURRENT_SINK:
            current = comp.current_rating_ma or 100.0
            for pad_num, pintype in comp.pin_types.items():
                pinfunction = comp.pin_functions.get(pad_num, "")
                if is_power_pin(pinfunction, pintype):
                    net_name = comp.net_connections.get(pad_num, "")
                    if net_name and 'unconnected' not in net_name.lower():
                        if net_name not in net_currents:
                            net_currents[net_name] = 0.0
                        net_currents[net_name] += current

        elif comp.role == ComponentRole.POWER_SOURCE:
            current = comp.current_rating_ma or 100.0
            for pad_num, pintype in comp.pin_types.items():
                pinfunction = comp.pin_functions.get(pad_num, "")
                if pintype == 'power_out' or is_power_pin(pinfunction, pintype):
                    net_name = comp.net_connections.get(pad_num, "")
                    if net_name and 'unconnected' not in net_name.lower():
                        if net_name not in net_currents:
                            net_currents[net_name] = 0.0
                        net_currents[net_name] += current

    # Convert current to track width using IPC-2152 guidelines
    def current_to_width(current_ma: float) -> float:
        """Convert current in mA to recommended track width in mm."""
        if current_ma < 100:
            return 0.25
        elif current_ma < 500:
            return 0.35
        elif current_ma < 1000:
            return 0.50
        elif current_ma < 2000:
            return 0.60
        elif current_ma < 5000:
            return 1.00
        else:
            return 1.50  # Consider using planes

    # Generate recommendations
    recommendations = {}
    for net_name, current_ma in net_currents.items():
        if current_ma >= min_current_ma:
            recommendations[net_name] = current_to_width(current_ma)

    # Also add ground nets with wide traces
    for net_id, net in pcb_data.nets.items():
        if 'GND' in net.name.upper() and net.name not in recommendations:
            recommendations[net.name] = 0.50

    return recommendations


def format_analysis_report(pcb_data: PCBData,
                          components: Dict[str, ComponentInfo],
                          paths: List[PowerPath],
                          recommendations: Dict[str, float]) -> str:
    """
    Format a human-readable analysis report.
    """
    lines = []
    lines.append("=" * 70)
    lines.append("POWER PATH ANALYSIS REPORT")
    lines.append("=" * 70)

    # Component summary
    lines.append("\n## Component Classification Summary\n")
    role_counts = {}
    for comp in components.values():
        role_counts[comp.role] = role_counts.get(comp.role, 0) + 1

    for role, count in sorted(role_counts.items(), key=lambda x: x[0].value):
        lines.append(f"  {role.value}: {count} components")

    # Power sources
    lines.append("\n## Power Sources\n")
    for comp in components.values():
        if comp.role == ComponentRole.POWER_SOURCE:
            lines.append(f"  {comp.ref}: {comp.value}")
            if comp.notes:
                lines.append(f"    Notes: {comp.notes}")

    # Current sinks
    lines.append("\n## Current Sinks (ICs requiring power)\n")
    for comp in components.values():
        if comp.role == ComponentRole.CURRENT_SINK:
            current_str = f"{comp.current_rating_ma}mA" if comp.current_rating_ma else "unknown"
            lines.append(f"  {comp.ref}: {comp.value} ({current_str})")

    # Power paths
    lines.append("\n## Traced Power Paths\n")
    for i, path in enumerate(paths[:20], 1):  # Limit to first 20
        lines.append(f"  Path {i}: {path.source_component} -> {path.sink_component}")
        lines.append(f"    Current: {path.estimated_current_ma:.0f}mA")
        lines.append(f"    Nets: {' -> '.join(path.nets_in_path)}")
        if path.components_in_path:
            lines.append(f"    Via: {', '.join(path.components_in_path)}")

    if len(paths) > 20:
        lines.append(f"  ... and {len(paths) - 20} more paths")

    # Recommendations
    lines.append("\n## Power Net Recommendations\n")
    lines.append("  Net Name                          | Width (mm)")
    lines.append("  " + "-" * 50)
    for net_name, width in sorted(recommendations.items()):
        lines.append(f"  {net_name:35} | {width:.2f}")

    # Command line
    lines.append("\n## Routing Configuration\n")
    nets = list(recommendations.keys())
    widths = [recommendations[n] for n in nets]
    nets_str = ' '.join(f'"{n}"' for n in nets)
    widths_str = ' '.join(f'{w}' for w in widths)
    lines.append(f'  --power-nets {nets_str}')
    lines.append(f'  --power-nets-widths {widths_str}')

    lines.append("\n" + "=" * 70)

    return '\n'.join(lines)


# Main entry point for interactive use
def analyze_pcb(filepath: str) -> Tuple[Dict[str, ComponentInfo], PCBData]:
    """
    Load a PCB file and extract components for analysis.

    Returns (components, pcb_data) tuple.
    Use get_components_needing_analysis() to find which components need AI classification.
    """
    print(f"Loading {filepath}...")
    pcb_data = parse_kicad_pcb(filepath)
    print(f"Extracting components...")
    components = extract_components_for_analysis(pcb_data)

    # Summary
    auto_classified = sum(1 for c in components.values() if c.role != ComponentRole.UNKNOWN)
    needs_analysis = sum(1 for c in components.values() if c.role == ComponentRole.UNKNOWN)

    print(f"Found {len(components)} components:")
    print(f"  Auto-classified: {auto_classified}")
    print(f"  Needs AI analysis: {needs_analysis}")

    return components, pcb_data


if __name__ == "__main__":
    import sys
    if len(sys.argv) < 2:
        print("Usage: python analyze_power_paths.py <pcb_file>")
        sys.exit(1)

    components, pcb_data = analyze_pcb(sys.argv[1])

    # Show components needing analysis
    unknown = get_components_needing_analysis(components)
    if unknown:
        print(f"\nComponents needing AI analysis:")
        for comp in unknown[:10]:
            print(f"  {comp.ref}: {comp.value} ({comp.pad_count} pins)")
        if len(unknown) > 10:
            print(f"  ... and {len(unknown) - 10} more")
