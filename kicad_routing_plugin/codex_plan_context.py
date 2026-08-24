"""Build the read-only board context embedded in Codex routing-plan prompts."""

import json
import os
import sys


ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PY_ROUTER_DIR = os.path.join(ROOT_DIR, "py_router")
if PY_ROUTER_DIR not in sys.path:
    sys.path.insert(0, PY_ROUTER_DIR)

from kicad_parser import parse_kicad_pcb  # noqa: E402
from list_nets import (  # noqa: E402
    find_differential_pairs,
    find_high_connection_nets,
    find_power_nets,
    read_design_rules,
)


def _axis_steps(values):
    values = sorted(set(round(float(value), 4) for value in values))
    return sorted(set(round(b - a, 4) for a, b in zip(values, values[1:])
                      if b - a > 1e-6))[:8]


def _footprint_context(footprint):
    # Anonymous nodes are paste apertures, not electrical pads.
    pads = [pad for pad in footprint.pads if pad.pad_number or pad.net_name]
    result = {
        "reference": footprint.reference,
        "value": footprint.value,
        "footprint": footprint.footprint_name,
        "layer": footprint.layer,
        "pad_count": len(pads),
        "unique_pin_count": len({pad.pad_number for pad in pads
                                  if pad.pad_number}),
        "smd_count": sum(1 for pad in pads if not pad.drill),
        "through_hole_count": sum(1 for pad in pads if pad.drill),
        "local_x_steps_mm": _axis_steps(pad.local_x for pad in pads),
        "local_y_steps_mm": _axis_steps(pad.local_y for pad in pads),
    }
    if len(pads) >= 6:
        result["pads"] = [
            {
                "number": pad.pad_number,
                "net": pad.net_name,
                "xy_mm": [round(pad.local_x, 4), round(pad.local_y, 4)],
                "drill_mm": round(pad.drill, 4),
            }
            for pad in pads
        ]
    return result


def collect_plan_context(board_path):
    """Return compact JSON containing all local facts needed by the planner."""
    board_path = os.path.abspath(board_path)
    pcb = parse_kicad_pcb(board_path)
    grounds, powers, other_power = find_power_nets(pcb)
    bounds = pcb.board_info.board_bounds
    context = {
        "board": board_path,
        "statistics": {
            "nets": len(pcb.nets),
            "footprints": len(pcb.footprints),
            "existing_segments": len(pcb.segments),
            "existing_vias": len(pcb.vias),
            "zones": len(pcb.zones),
            "bounds_mm": list(bounds) if bounds else None,
        },
        "copper_layers": pcb.board_info.copper_layers,
        "stackup": [
            {"name": layer.name, "type": layer.layer_type,
             "thickness_mm": layer.thickness, "epsilon_r": layer.epsilon_r,
             "material": layer.material}
            for layer in pcb.board_info.stackup
        ],
        "differential_pairs": find_differential_pairs(pcb),
        "ground_nets": grounds,
        "power_nets": powers,
        "other_power_nets": other_power,
        "top_connected_nets": find_high_connection_nets(pcb, 25),
        "all_nets": sorted(
            ({"name": net.name, "pad_count": len(net.pads)}
             for net in pcb.nets.values()),
            key=lambda item: (-item["pad_count"], item["name"])),
        "design_rules": read_design_rules(board_path),
        "footprints": [_footprint_context(footprint)
                       for _, footprint in sorted(pcb.footprints.items())],
    }
    return json.dumps(context, ensure_ascii=False, separators=(",", ":"))
