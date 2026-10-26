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

from __future__ import annotations

#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['routing'], 'kind': 'instrument'}

import _path  # noqa: F401  (#522: makes ../py_router importable)

import warnings
from dataclasses import dataclass, field
from typing import Dict, List, Set, Optional, Tuple
from enum import Enum

from kicad_parser import parse_kicad_pcb, PCBData, Footprint, Pad, non_aperture_pads
# Re-exported, not re-implemented (#705). These used to be a local tuple and a
# closure inside `get_power_net_recommendations`; they are the repo's only
# pin-level power predicate and every other caller was locked out of them.
from net_queries import (  # noqa: F401  (POWER_PIN_KEYWORDS is a public re-export)
    POWER_PIN_KEYWORDS, is_power_pin, is_ground_net_name, is_power_net_name)


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
    supply_nets: Optional[List[str]] = None  # Nets the agent names as this part's rails
    classified: bool = False           # classify_component() was called on it


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
            pad_count=len(non_aperture_pads(fp)),   # pins, not apertures
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
        # Two-terminal by its pins: a 0201's split paste windows are not
        # terminals (#1143; orangecrab's 0201 caps read as 4-pad parts).
        from kicad_parser import non_aperture_pads
        if len(non_aperture_pads(fp)) == 2:
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
                       notes: str = "",
                       supply_nets: Optional[List[str]] = None) -> None:
    """
    Set the classification for a component based on AI analysis.

    Args:
        components: The components dict to modify
        ref: Component reference designator
        role: The determined role
        current_rating_ma: Estimated current in milliamps
        notes: Any notes from the analysis
        supply_nets: The nets this part draws on (a sink) or feeds (a source),
            for a part whose pads carry no pintype or pin function and whose
            nets are not named like rails. Optional: without it the pads are
            read by pintype, pin function, then net name.
    """
    if ref in components:
        if supply_nets:
            known = set(components[ref].net_connections.values())
            missing = [n for n in supply_nets if n not in known]
            if missing:
                raise ValueError(f"{ref} has no pad on net(s) {missing}")
        components[ref].role = role
        components[ref].current_rating_ma = current_rating_ma
        components[ref].notes = notes
        components[ref].supply_nets = list(supply_nets) if supply_nets else None
        components[ref].classified = True


_GROUND_PIN_PREFIXES = ('GND', 'VSS', 'AGND', 'DGND', 'PGND')
#: Supply pin FUNCTIONS beyond ``net_queries.POWER_PIN_KEYWORDS`` (which stays
#: as it is, since the placement side reads it too): the names a connector, a
#: bank-supply pin or a battery pin carries (``VBUS``, ``IOVDD_1``, ``VBAT``).
_SUPPLY_FUNCTION_TOKENS = ('VDD', 'VCC', 'VBUS', 'VBAT', 'VSYS')


def _is_supply_pad(pinfunction: str, pintype: str) -> bool:
    fn = (pinfunction or '').upper()
    return (is_power_pin(pinfunction, pintype)
            or any(t in fn for t in _SUPPLY_FUNCTION_TOKENS)
            or fn.startswith(('VIN', 'VOUT')))


def _is_connected(net_name: str) -> bool:
    return bool(net_name) and 'unconnected' not in net_name.lower()


def _supply_nets(comp: ComponentInfo) -> Tuple[List[str], List[str]]:
    """The (rail nets, ground nets) a classified part draws on or feeds.

    A pad is a ground pad when its net is in the GND family or its pin
    function says GND/VSS; it is a supply pad when the pin-level predicate
    says so (``is_power_pin``: a power_in/power_out pintype or a VCC/VDD-style
    pin function). A part whose pads carry neither (no pintype and no pin
    function in the footprint) falls back to its pads on nets NAMED like a rail,
    so a classification is never silently dropped for want of pin metadata
    (#1378). ``comp.supply_nets`` -- the agent saying so -- outranks both.
    """
    rails: List[str] = []
    grounds: List[str] = []
    for pad, net in comp.net_connections.items():
        if not _is_connected(net):
            continue
        fn = comp.pin_functions.get(pad, '')
        pt = comp.pin_types.get(pad, '')
        if is_ground_net_name(net) or fn.upper().startswith(_GROUND_PIN_PREFIXES):
            if net not in grounds:
                grounds.append(net)
        elif _is_supply_pad(fn, pt) and net not in rails:
            rails.append(net)
    if comp.supply_nets:
        rails = [n for n in comp.supply_nets if n not in grounds]
    elif not rails:
        for net in comp.net_connections.values():
            if (_is_connected(net) and net not in grounds and net not in rails
                    and is_power_net_name(net)):
                rails.append(net)
    return rails, grounds


def _regulator_terminals(comp: ComponentInfo) -> Tuple[List[str], List[str]]:
    """(output nets, input nets) of a POWER_SOURCE part.

    Outputs are power_out pins or OUT-named pins; inputs are IN-named pins or
    power_in pins on a non-ground net, read only when the part has outputs. A
    part with no typed output (a connector, a battery holder, a regulator whose
    footprint carries no pintypes) offers its supply rails as outputs and has
    no input, so it is a root source.
    """
    outs: List[str] = []
    ins: List[str] = []
    if not comp.supply_nets:
        for pad, pt in comp.pin_types.items():
            net = comp.net_connections.get(pad, '')
            if not _is_connected(net) or is_ground_net_name(net):
                continue
            fn = comp.pin_functions.get(pad, '').upper()
            if ('power_out' in (pt or '').split('+')
                    or fn in ('OUT', 'VOUT', 'OUTPUT')) and net not in outs:
                outs.append(net)
    for pad, net in comp.net_connections.items():
        if (not outs or not _is_connected(net) or is_ground_net_name(net)
                or net in outs or net in ins):
            continue
        fn = comp.pin_functions.get(pad, '').upper()
        pt = (comp.pin_types.get(pad, '') or '').split('+')
        if fn in ('IN', 'VIN', 'INPUT') or ('power_in' in pt and 'no_connect' not in pt):
            ins.append(net)
    if not outs:
        rails, _ = _supply_nets(comp)
        if not rails:
            # Nothing on the part looks like a rail: the classification is the
            # agent's word that it brings power on, so every non-ground net is
            # a terminal. A net no sink draws on and no pass-through joins is
            # never reached by the search, so a data pin here costs nothing.
            rails = [n for n in dict.fromkeys(comp.net_connections.values())
                     if _is_connected(n) and not is_ground_net_name(n)]
        return rails, []
    return outs, ins


def trace_power_paths(pcb_data: PCBData,
                      components: Dict[str, ComponentInfo]) -> List[PowerPath]:
    """
    Trace the current each load draws back to the source that feeds it.

    Paths run per (sink part, supply net the sink draws on), so a rail shared
    by several loads is traced for every one of them (#1380). The search runs
    UPSTREAM over a directed graph: a PASS_THROUGH part conducts between its
    non-ground nets in both directions (a 3-terminal FET or load switch
    included, #1378), and a regulator is entered only from its output to its
    input, never the other way (#1381), because a regulator's input rail does
    not draw from its output. Every branch stops at the first source that has
    nothing further upstream (a battery and a USB input feeding one rail are
    two paths, each rail on each branch carrying the load); a source on the
    sink's own net is a one-net path. When no branch reaches such a root, the
    path ends at the nearest regulator crossed, so its input rail still
    carries the load. The recommendation charges a sink once per net however
    many paths name it.

    ``nets_in_path`` runs source side first.
    """
    paths: List[PowerPath] = []

    # Pass-through conduction. A FET or load switch has three nets (the gate
    # being a control net); a clique over the non-ground nets is safe because
    # a dead-end net never lies on the route to a source.
    upstream: Dict[str, List[Tuple[str, str]]] = {}   # net -> [(via_ref, net)]

    def link(ref: str, a: str, b: str) -> None:
        upstream.setdefault(a, []).append((ref, b))

    for ref, comp in components.items():
        if comp.role != ComponentRole.PASS_THROUGH:
            continue
        nets = []
        for net in comp.net_connections.values():
            if _is_connected(net) and not is_ground_net_name(net) and net not in nets:
                nets.append(net)
        for a in nets:
            for b in nets:
                if a != b:
                    link(ref, a, b)

    # Source terminals; a regulator with an input continues the search.
    sources_at: Dict[str, List[str]] = {}
    continues: Set[Tuple[str, str]] = set()       # (source ref, output net)
    for ref, comp in components.items():
        if comp.role != ComponentRole.POWER_SOURCE:
            continue
        outs, ins = _regulator_terminals(comp)
        for out in outs:
            sources_at.setdefault(out, []).append(ref)
            if ins:
                continues.add((ref, out))
                for inn in ins:
                    if inn != out:
                        link(ref, out, inn)

    for ref, comp in components.items():
        if comp.role != ComponentRole.CURRENT_SINK:
            continue
        current_ma = comp.current_rating_ma or 100.0
        rails, _ = _supply_nets(comp)
        for start in rails:
            visited = {start: (None, None)}
            queue = [start]
            roots = []               # (net, source ref): one path each
            fallback = None          # first net entered through a regulator
            while queue:
                net = queue.pop(0)
                here = [r for r in sources_at.get(net, ()) if (r, net) not in continues]
                if here:
                    roots.extend((net, r) for r in sorted(here))
                    continue         # nothing upstream of a root is this load's
                for via, other in upstream.get(net, ()):
                    if other in visited:
                        continue
                    visited[other] = (net, via)
                    queue.append(other)
                    if (fallback is None and
                            components[via].role == ComponentRole.POWER_SOURCE):
                        fallback = (other, via)
            if not roots and fallback is not None:
                roots = [fallback]
            for end, source_ref in roots:
                nets_in, via_in = [], []
                cur = end
                while cur is not None:
                    nets_in.append(cur)
                    prev, via = visited[cur]
                    if via:
                        via_in.append(via)
                    cur = prev
                paths.append(PowerPath(
                    source_component=source_ref,
                    sink_component=ref,
                    nets_in_path=nets_in,
                    components_in_path=via_in,
                    estimated_current_ma=current_ma))
    return paths


def classified_parts_without_nets(components: Dict[str, ComponentInfo]
                                  ) -> List[str]:
    """Refs the agent classified (``classify_component``) that reach no net (#1378).

    A sink or source with no supply or ground net, or a pass-through joining
    fewer than two non-ground nets, contributes nothing to a width; the cure
    is to name its nets (``classify_component(..., supply_nets=[...])``).
    """
    lost = []
    for ref, comp in components.items():
        if not comp.classified:
            continue
        if comp.role == ComponentRole.CURRENT_SINK:
            rails, grounds = _supply_nets(comp)
            if not rails and not grounds:
                lost.append(ref)
        elif comp.role == ComponentRole.POWER_SOURCE:
            if not _regulator_terminals(comp)[0]:
                lost.append(ref)
        elif comp.role == ComponentRole.PASS_THROUGH:
            nets = {n for n in comp.net_connections.values()
                    if _is_connected(n) and not is_ground_net_name(n)}
            if len(nets) < 2:
                lost.append(ref)
    return lost


def get_power_net_recommendations(pcb_data: PCBData,
                                  components: Dict[str, ComponentInfo],
                                  paths: List[PowerPath],
                                  min_current_ma: float = 50.0) -> Dict[str, float]:
    """
    Get recommended track widths for power nets based on traced paths.

    A sink's ``current_rating_ma`` is its total supply current and is charged
    ONCE to every supply net it draws on, whether through a traced path or
    straight from its pins (#1377). A source's rating is a limit, not a load,
    and is never added. Each ground net carries the sinks that have a pad on
    it, once each.

    Args:
        pcb_data: Parsed PCB data
        components: Classified components
        paths: Traced power paths
        min_current_ma: Minimum current to consider for power routing

    Returns:
        Dict of net_name -> recommended_width_mm
    """
    lost = classified_parts_without_nets(components)
    if lost:
        warnings.warn(
            f"classified part(s) {', '.join(sorted(lost))} reach no power net, so "
            f"they contribute no width; name their nets with "
            f"classify_component(..., supply_nets=[...])", stacklevel=2)

    net_currents: Dict[str, float] = {}
    charged: Set[Tuple[str, str]] = set()          # (sink ref, net)

    def charge(ref: str, net: str, ma: float) -> None:
        if (ref, net) not in charged:
            charged.add((ref, net))
            net_currents[net] = net_currents.get(net, 0.0) + ma

    for path in paths:
        for net_name in path.nets_in_path:
            charge(path.sink_component, net_name, path.estimated_current_ma)

    for ref, comp in components.items():
        if comp.role != ComponentRole.CURRENT_SINK:
            continue
        current = comp.current_rating_ma or 100.0
        rails, grounds = _supply_nets(comp)
        for net_name in rails + grounds:
            charge(ref, net_name, current)

    # Convert current to track width from the IPC-2221 width table (see
    # docs/power-nets.md) with a 2x safety factor. That table is IPC-2221's, not
    # IPC-2152's as this claimed before #489 §6.
    def current_to_width(current_ma: float) -> float:
        """Convert current in mA to recommended track width in mm.

        Applies a 2x safety factor beyond the IPC-2221 table for:
        - Manufacturing tolerance margin
        - Thermal headroom
        - Trace resistance reduction
        """
        if current_ma < 50:
            return 0.30   # IPC: 0.15 -> 2x = 0.30
        elif current_ma < 100:
            return 0.50   # IPC: 0.25 -> 2x = 0.50
        elif current_ma < 500:
            return 0.70   # IPC: 0.35 -> 2x = 0.70
        elif current_ma < 1000:
            return 1.00   # IPC: 0.50 -> 2x = 1.00
        elif current_ma < 2000:
            return 1.20   # IPC: 0.60 -> 2x = 1.20
        elif current_ma < 5000:
            return 2.00   # IPC: 1.00 -> 2x = 2.00
        else:
            return 3.00   # IPC: 1.50 -> 2x = 3.00; consider using planes

    return {net_name: current_to_width(current_ma)
            for net_name, current_ma in net_currents.items()
            if current_ma >= min_current_ma}


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

    lost = classified_parts_without_nets(components)
    if lost:
        lines.append("\n## Classified parts that reach no power net\n")
        lines.append(f"  {', '.join(sorted(lost))} -- they contribute no width; "
                     f"name their nets with classify_component(..., supply_nets=[...])")

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
    import argparse

    # A REAL parser, not a hand-rolled `Usage:` print: `--help` used to be read
    # as the board FILENAME, so this answered a capability probe with
    # `FileNotFoundError: '--help'` (#937). See kicad_parser.py's `__main__`
    # for the same fix and why it matters.
    _ap = argparse.ArgumentParser(
        description="Classify components by power role and report the ones "
                    "that still need analysis.")
    _ap.add_argument('pcb_file', help='the .kicad_pcb to analyze')
    _a = _ap.parse_args()

    components, pcb_data = analyze_pcb(_a.pcb_file)

    # Show components needing analysis
    unknown = get_components_needing_analysis(components)
    if unknown:
        print(f"\nComponents needing AI analysis:")
        for comp in unknown[:10]:
            print(f"  {comp.ref}: {comp.value} ({comp.pad_count} pins)")
        if len(unknown) > 10:
            print(f"  ... and {len(unknown) - 10} more")
