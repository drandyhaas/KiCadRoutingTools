#!/usr/bin/env python3
"""Convert a stress-test redo_commands.sh manifest into a GUI plan JSON.

The Claude tab's Load button accepts the output, so a recorded stress chain
can be replayed through the GUI plan executor without any LLM run:

    python3 tests/stress/manifest_to_plan.py runs_set1/<board>/redo_commands.sh plan.json

Maps each routing command to a plan step (action + nets/pairs/assignments +
params). Since the GUI accepts ANY snake_case option name in params, every
recognized --flag value is carried over (--max-iterations, --max-ripup,
--grid-step, ...). Check/grade commands and unknown tools are skipped with a
note. Only the file-dependency chain to the FINAL board is emitted (retries
that were superseded are dropped), mirroring redo_stress_test's pruning.
"""
import json
import os
import re
import shlex
import sys

TOOL_ACTIONS = {
    'route.py': 'route',
    'route_diff.py': 'route_diff',
    'route_planes.py': 'route_planes',
    'route_disconnected_planes.py': 'repair_planes',
    'bga_fanout.py': 'fanout',
    'qfn_fanout.py': 'fanout',
}

# CLI flag -> plan params key (numbers parsed; lists collected).
FLAG_PARAMS = {
    '--track-width': 'track_width',
    '--clearance': 'clearance',
    '--via-size': 'via_size',
    '--via-drill': 'via_drill',
    '--grid-step': 'grid_step',
    '--max-iterations': 'max_iterations',
    '--max-ripup': 'max_ripup',
    '--ripup-abandon-metric': 'ripup_abandon_metric',
    '--ripup-blocker-select': 'ripup_blocker_select',
    # route.py spells the strategy flag --ordering; the GUI control (and the
    # plan executor's param name) is ordering_strategy. Without this mapping
    # the generic fallthrough carried it as 'ordering', which matches no
    # dialog control, so a replayed plan silently routed in default order.
    '--ordering': 'ordering_strategy',
    # route_planes' --zone-clearance is type=float (a value, NOT a toggle); it
    # must consume its argument here. It briefly lived in BOOL_FLAGS, which
    # dropped the value and set zone_clearance=True -> the plan executor's
    # generic loop stamped float(True)=1.0 onto the plane zone-clearance
    # SpinCtrlDouble (range max 2.0, so unclamped), replaying a board routed at
    # e.g. 0.12 mm pour clearance with a 1.0 mm clearance instead.
    '--zone-clearance': 'zone_clearance',
    '--hole-to-hole-clearance': 'hole_to_hole_clearance',
    '--board-edge-clearance': 'board_edge_clearance',
    '--via-cost': 'via_cost',
    '--heuristic-weight': 'heuristic_weight',
    '--turn-cost': 'turn_cost',
    '--diff-pair-gap': 'diff_pair_gap',
    '--impedance': 'impedance',
    '--gnd-via-distance': 'gnd_via_distance',
    '--exit-margin': 'exit_margin',
    '--extension': 'extension',
    '--max-track-width': 'max_track_width',
    '--min-track-width': 'min_track_width',
    '--analysis-grid-step': 'analysis_grid_step',
}
LIST_FLAGS = {
    '--layers': 'layers',
    '--power-nets': 'power_nets',
    '--power-nets-widths': 'power_nets_widths',
    '--layer-costs': 'layer_costs',
    # #381 D3: route_diff's polarity-swap allowlist (nargs='+' globs). Carried
    # explicitly (not via the generic unknown-flag fallthrough) so a scoped
    # allowlist survives as a list param that claude_plan's alias routes to the
    # diff tab's polarity_swap_nets_text field.
    '--polarity-swap-nets': 'polarity_swap_nets',
    '--nets': None,  # handled per action
    '--pairs': None,
    '--plane-layers': None,
}
BOOL_FLAGS = {
    '--rip-blocker-nets': 'rip_blocker_nets',
    '--add-gnd-vias': 'add_gnd_vias',
    '--no-gnd-vias': 'no_gnd_vias',
    # route.py spells it --no-bga-zones (plural, nargs='*'); bga_fanout uses the
    # singular. Both map to the GUI's no_bga_zone special (bare = exclude ALL).
    '--no-bga-zone': 'no_bga_zone',
    '--no-bga-zones': 'no_bga_zone',
}

# Flags whose values are file paths / bookkeeping -- consumed, never params.
# --output still feeds the chain-pruning file list. --net-clearances is a
# board-specific JSON path; the GUI derives the same map from the board's live
# net classes, so a replayed plan carries no param for it.
IGNORE_FLAGS = {'--output', '--summary-json', '--schematic-dir', '--report',
                '--net-clearances'}

# Per-tool flag renames: bga_fanout calls the trace width --width (routed to the
# Basic-tab track_width, which BGA fanout reads). qfn_fanout also uses --width
# but its GUI home is the QFN panel's own control (see TOOL_FLAG_PARAMS, #381 D7).
TOOL_FLAG_ALIASES = {
    'bga_fanout.py': {'--width': '--track-width'},
}

# Per-tool flag -> plan-param overrides (win over the global FLAG_PARAMS).
# #381 D4: route_diff.py's trace width is --track-width, but its GUI home is the
# diff tab's diff_pair_width control (NOT the Basic-tab track_width the global
# FLAG_PARAMS would target). Without this override a recorded route_diff
# --track-width 0.2 set the Basic-tab width and left the diff tab at its default,
# so a plan-replayed diff routed at the wrong width.
# #381 D7: qfn_fanout.py's --width/--clearance map to the QFN panel's own
# controls (default 0.1/0.1), not the Basic-tab track_width/clearance that
# BGA/route use, so a plan-replayed QFN fanout keeps its fine-pitch width.
TOOL_FLAG_PARAMS = {
    'route_diff.py': {'--track-width': 'diff_pair_width'},
    'qfn_fanout.py': {'--width': 'qfn_track_width', '--clearance': 'qfn_clearance'},
}


def _num(v):
    try:
        f = float(v)
        return int(f) if f == int(f) else f
    except ValueError:
        return v


def parse_command(argv):
    tool = None
    for a in argv:
        base = os.path.basename(a)
        if base in TOOL_ACTIONS:
            tool = base
            break
    if tool is None:
        return None
    # A `--help` invocation (the agent inspecting a tool during the run) is not a
    # routing step -- skip it so the plan doesn't carry no-op `help: true` steps.
    if '--help' in argv or '-h' in argv:
        return None
    action = TOOL_ACTIONS[tool]
    step = {'action': action, 'params': {}}
    step['_files'] = []  # positional .kicad_pcb args (input/output), for pruning
    lists = {}
    # Normalize `--flag=value` to `--flag value` (recorded manifests mix both
    # forms; core64_logic's `--nets=-BATT` otherwise fell into the unknown-flag
    # branch as a mangled param, left --nets empty, and cascaded into EMPTY
    # plane assignments even though --plane-layers parsed fine).
    argv = [t for a in argv
            for t in (a.split('=', 1) if a.startswith('--') and '=' in a
                      else (a,))]
    i = argv.index([a for a in argv if os.path.basename(a) == tool][0]) + 1
    positional = []
    aliases = TOOL_FLAG_ALIASES.get(tool, {})
    tool_params = TOOL_FLAG_PARAMS.get(tool, {})
    while i < len(argv):
        a = aliases.get(argv[i], argv[i])
        if a in IGNORE_FLAGS:
            i += 1
            while i < len(argv) and not argv[i].startswith('--'):
                if argv[i].endswith('.kicad_pcb'):
                    step['_files'].append(argv[i])
                i += 1
        elif a in BOOL_FLAGS:
            step['params'][BOOL_FLAGS[a]] = True
            i += 1
        elif a in tool_params or a in FLAG_PARAMS:
            step['params'][tool_params.get(a) or FLAG_PARAMS[a]] = _num(argv[i + 1])
            i += 2
        elif a in LIST_FLAGS:
            vals = []
            i += 1
            while i < len(argv) and not argv[i].startswith('--'):
                vals.append(_num(argv[i]))
                i += 1
            lists[a] = vals
        elif a == '--component':
            step['component'] = argv[i + 1]
            i += 2
        elif a.startswith('--'):
            # unknown flag: skip it and any non-flag values (still carried
            # generically when it maps to a control name)
            key = a.lstrip('-').replace('-', '_')
            vals = []
            i += 1
            while i < len(argv) and not argv[i].startswith('--'):
                vals.append(_num(argv[i]))
                i += 1
            if len(vals) == 1:
                step['params'][key] = vals[0]
            elif vals:
                step['params'][key] = vals
            else:
                step['params'][key] = True
        else:
            positional.append(a)
            if a.endswith('.kicad_pcb'):
                step['_files'].append(a)
            i += 1

    nets = lists.get('--nets', [])
    if action in ('route',):
        step['nets'] = [str(n) for n in nets] or ['*']
    elif action == 'route_diff':
        step['pairs'] = [str(n) for n in lists.get('--pairs', nets)]
    elif action in ('route_planes', 'repair_planes'):
        layers = [str(l) for l in lists.get('--plane-layers', [])]
        net_names = [str(n) for n in nets]
        if net_names and layers:
            step['assignments'] = [
                {'nets': [n], 'layer': layers[min(k, len(layers) - 1)]}
                for k, n in enumerate(net_names)]
        elif net_names and action == 'route_planes':
            # No --plane-layers (route_planes auto-detects zones): emit a
            # layer-less assignment; the GUI plane tab resolves the layer.
            step['assignments'] = [{'nets': net_names, 'layer': ''}]
        # repair_planes with no --plane-layers: emit NO assignments, so the
        # post-pass below inherits the preceding route_planes step's REAL
        # layers. route_disconnected_planes auto-detects zones on the CLI, but
        # the GUI repair needs explicit copper layers -- an empty-layer
        # assignment blocks the inherit fallback ("no valid copper layers in
        # ['']") and the whole repair step is silently dropped.
    elif action == 'fanout':
        step['kind'] = 'bga' if tool == 'bga_fanout.py' else 'qfn'
        step['nets'] = [str(n) for n in nets] or ['*']
    for k in ('--power-nets', '--power-nets-widths', '--layer-costs',
              '--layers', '--polarity-swap-nets'):
        if k in lists:
            step['params'][LIST_FLAGS[k]] = lists[k]
    return step


# place_fanout_clearance.py has no standalone CLI-tool action, but in the GUI it
# IS a plan step in its own right: the live Claude-plan format (claude_plan.py's
# KNOWN_ACTIONS + _insert_cap_optimization) represents "Optimize decoupling cap
# placement" (#130) as a SEPARATE `optimize_caps` step placed right after the last
# BGA fanout, run via fanout_tab.run_cap_optimization() -- NOT as a param on the
# fanout step. So a recorded place_fanout_clearance emits the same standalone step
# (in its manifest position, i.e. after the fanout it followed), for parity with a
# live-generated plan. Flag -> fanout-tab cap-placement control name:
CAP_FLAG_PARAMS = {
    '--capture-radius': 'cap_capture_radius',
    '--near-margin': 'cap_near_margin',
    '--step': 'cap_step',
    '--max-displacement': 'cap_max_displacement',
    '--max-displacement-cap': 'cap_max_displacement_cap',
    '--displacement-growth': 'cap_displacement_growth',
    '--max-passes': 'cap_max_passes',
    '--cap-prefix': 'cap_prefix',
}
CAP_BOOL_FLAGS = {'--no-rotate': ('cap_allow_rotation', False)}  # inverted sense


def cap_optimization_step(argv):
    """A place_fanout_clearance.py invocation -> a standalone `optimize_caps` plan
    step (matching claude_plan.py's live format), carrying the non-default cap_*
    knobs so a loaded plan optimizes caps the way the recorded run did."""
    params = {}
    i = 0
    while i < len(argv):
        a = argv[i]
        if a in CAP_FLAG_PARAMS and i + 1 < len(argv):
            params[CAP_FLAG_PARAMS[a]] = _num(argv[i + 1]); i += 2
        elif a in CAP_BOOL_FLAGS:
            key, val = CAP_BOOL_FLAGS[a]; params[key] = val; i += 1
        else:
            i += 1
    step = {'action': 'optimize_caps'}
    if params:
        step['params'] = params
    return step


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    manifest, out = sys.argv[1], sys.argv[2]

    # Prune with redo_stress_test's canonical file-dependency logic so the plan
    # is EXACTLY the simplified/deduplicated chain a replay runs -- the same
    # "N of M commands" set, with superseded retries and dead-end branches
    # dropped. Pruning on the FULL command set (before GUI-mapping) is what makes
    # this correct: a kept command the GUI can't represent as a step
    # (place_fanout_clearance.py, board_image.py) must still hold the file chain
    # together. The old inline pruner walked only GUI-recognized steps, so a
    # skipped intermediate (e.g. place_fanout_clearance) BROKE the chain and
    # silently dropped legitimate upstream steps (e.g. the whole bga_fanout).
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from redo_stress_test import parse_manifest, compute_prune_keep, is_check_cmd

    cmds = parse_manifest(manifest)
    keep, _info = compute_prune_keep(cmds)

    steps = []
    skipped = 0
    for i, (_cwd, argv) in enumerate(cmds):
        if i not in keep or is_check_cmd(argv):
            continue  # pruned out, or a check/grade command (no GUI step)
        if any(os.path.basename(a) == 'place_fanout_clearance.py' for a in argv):
            # standalone optimize_caps step, matching the live GUI plan (see above)
            steps.append(cap_optimization_step(argv))
            continue
        step = parse_command(argv)
        if step is None:
            # kept-but-not-GUI-representable (place_fanout_clearance, board_image,
            # --help, ...): pruning already accounted for it -- just emit no step.
            skipped += 1
            continue
        if step['action'] == 'repair_planes' and 'assignments' not in step:
            # The repair CLI auto-detects zones; the GUI repair needs
            # explicit assignments -- inherit the last plane step's.
            for prev in reversed(steps):
                if prev['action'] == 'route_planes' and prev.get('assignments'):
                    step['assignments'] = [dict(a) for a in prev['assignments']]
                    break
        steps.append(step)

    for s in steps:
        s.pop('_files', None)
    with open(out, 'w', encoding='utf-8') as f:
        json.dump({'steps': steps}, f, indent=2)
    print(f"{len(steps)} step(s) written to {out} "
          f"({skipped} kept-but-non-GUI command(s) skipped)")
    print("Load it in the Claude tab (Load... next to 'Parsed result') and "
          "press 'Run Selected Steps'.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
