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
    '--hole-to-hole-clearance': 'hole_to_hole_clearance',
    '--board-edge-clearance': 'board_edge_clearance',
    '--via-cost': 'via_cost',
    '--heuristic-weight': 'heuristic_weight',
    '--turn-cost': 'turn_cost',
    '--diff-pair-width': 'diff_pair_width',
    '--diff-pair-gap': 'diff_pair_gap',
    '--impedance': 'impedance',
    '--gnd-via-distance': 'gnd_via_distance',
    '--exit-margin': 'exit_margin',
    '--extension': 'extension',
    '--max-track-width': 'max_track_width',
    '--analysis-grid-step': 'analysis_grid_step',
}
LIST_FLAGS = {
    '--power-nets': 'power_nets',
    '--power-nets-widths': 'power_nets_widths',
    '--layer-costs': 'layer_costs',
    '--nets': None,  # handled per action
    '--pairs': None,
    '--plane-layers': None,
}
BOOL_FLAGS = {
    '--rip-blocker-nets': 'rip_blocker_nets',
    '--add-gnd-vias': 'add_gnd_vias',
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
    action = TOOL_ACTIONS[tool]
    step = {'action': action, 'params': {}}
    step['_files'] = []  # positional .kicad_pcb args (input/output), for pruning
    lists = {}
    i = argv.index([a for a in argv if os.path.basename(a) == tool][0]) + 1
    positional = []
    while i < len(argv):
        a = argv[i]
        if a in BOOL_FLAGS:
            step['params'][BOOL_FLAGS[a]] = True
            i += 1
        elif a in FLAG_PARAMS:
            step['params'][FLAG_PARAMS[a]] = _num(argv[i + 1])
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
        elif net_names:
            step['assignments'] = [{'nets': net_names, 'layer': ''}]
    elif action == 'fanout':
        step['kind'] = 'bga' if tool == 'bga_fanout.py' else 'qfn'
        step['nets'] = [str(n) for n in nets] or ['*']
    for k in ('--power-nets', '--power-nets-widths', '--layer-costs'):
        if k in lists:
            step['params'][LIST_FLAGS[k]] = lists[k]
    return step


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    manifest, out = sys.argv[1], sys.argv[2]
    steps = []
    skipped = 0
    for line in open(manifest, encoding='utf-8'):
        line = line.strip()
        if not line or line.startswith('#') or line.startswith('set '):
            continue
        try:
            argv = shlex.split(line)
        except ValueError:
            skipped += 1
            continue
        step = parse_command(argv)
        if step is None:
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

    # Prune to the file-dependency chain of the FINAL board (mirror
    # redo_stress_test): a recorded retry that was superseded (same input,
    # its output never consumed downstream) is dropped -- without this the
    # GUI would run e.g. the '*' route twice.
    kept = []
    if steps and any(s.get('_files') for s in steps):
        need = None
        for s in reversed(steps):
            files = s.get('_files') or []
            ins, outp = files[:-1], (files[-1] if len(files) > 1 else None)
            if not files:
                kept.append(s)
                continue
            if need is None or (outp and os.path.basename(outp) == need) \
                    or outp is None:
                kept.append(s)
                if ins:
                    need = os.path.basename(ins[0])
                elif outp is None and files:
                    need = os.path.basename(files[0])
        kept.reverse()
        if len(kept) != len(steps):
            print(f"pruned {len(steps) - len(kept)} superseded step(s) "
                  f"(retries not on the final board's file chain)")
        steps = kept
    for s in steps:
        s.pop('_files', None)
    with open(out, 'w', encoding='utf-8') as f:
        json.dump({'steps': steps}, f, indent=2)
    print(f"{len(steps)} step(s) written to {out} ({skipped} non-routing "
          f"line(s) skipped)")
    print("Load it in the Claude tab (Load... next to 'Parsed result') and "
          "press 'Run Selected Steps'.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
