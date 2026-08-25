#!/usr/bin/env python3
"""Convert a stress-test redo_commands.sh manifest into a GUI plan JSON.

The AI tab's Load button accepts the output, so a recorded stress chain
can be replayed through the GUI plan executor without any LLM run:

    python3 tests/stress/manifest_to_plan.py runs_set1/<board>/redo_commands.sh plan.json
    python3 tests/stress/manifest_to_plan.py runs_set1/<board>/redo_commands.sh -o plan.json

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
    # BOTH names: recorded manifests carry the historical spelling forever;
    # the module (CLI remains a standalone utility) is repair_planes.py now.
    'route_disconnected_planes.py': 'repair_planes',
    'repair_planes.py': 'repair_planes',
    'bga_fanout.py': 'fanout',
    'qfn_fanout.py': 'fanout',
}

# Board-mutating tools with NO plan representation (#431). They are RECORDED in
# the manifest (#132) so the file-dependency chain stays intact -- a
# refused-but-PRESENT command keeps compute_prune_keep linking
# board -> board_placed, whereas a missing one leaves the next step's input
# produced by nothing and the pruner drops legitimate upstream steps.
#
# Refused rather than mapped, because there is nothing to map TO: the plan
# format has no placement step. (The GUI's Placement sub-tab drives the
# placement SKILLS headless - it is not a plan action, so a plan step's
# max_displacement/crossing_penalty/halo_* would still resolve to nonexistent
# dialog attributes and silently run at hardcoded defaults while the plan JSON
# claims otherwise.) And refused rather than DROPPED, because the unknown-tool path
# only bumps a `skipped` counter -- a number, not a name -- so the converted
# plan looks complete when it is not.
REFUSED_TOOLS = {
    'place_optimize.py': (
        'moves footprints; the plan format has no placement step. Run it BEFORE '
        'the plan and start the plan from the placed board'),
    'place_route_loop.py': (
        'routes and moves footprints in a loop; the plan format has no placement '
        'step. Run it on the CLI'),
    'place_seed.py': (
        'seeds/repairs a placement from an intent; the plan format has no '
        'placement step. Run it BEFORE the plan and start the plan from the '
        'seeded board'),
    'place_reconstruct.py': (
        'reconstructs a damaged placement; the plan format has no placement '
        'step. Run it BEFORE the plan and start the plan from its output'),
    'place_portfolio.py': (
        'generates a SLATE of placements to choose between; the plan format has '
        'no placement step, and picking one is a decision, not a replayable '
        'step. Run it on the CLI and start the plan from the adopted board'),
    'render_placement.py': (
        'renders a PNG; it changes no board and has nothing to replay'),
    'beautify_labels.py': (
        'tidies reference-designator silkscreen; no plan step yet -- run it '
        'before/after the plan'),
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
    '--coplanar-gap': 'coplanar_gap',
    '--gnd-via-distance': 'gnd_via_distance',
    # #485: route_planes area via stitching -- value flags.
    '--stitch-pitch': 'stitch_pitch',
    '--stitch-fence-pitch': 'stitch_fence_pitch',
    '--stitch-inset': 'stitch_inset',
    '--stitch-max-freq': 'stitch_max_freq',
    '--exit-margin': 'exit_margin',
    '--extension': 'extension',
    # #581: same-net pad via clearance -- valid on planes, route, route_diff,
    # bga/qfn fanout and repair steps; the GUI control lives on the Basic tab.
    '--same-net-pad-clearance': 'same_net_pad_clearance',
    '--max-track-width': 'max_track_width',
    '--min-track-width': 'min_track_width',
    '--analysis-grid-step': 'analysis_grid_step',
    # #237's shared fab-capability flags (fab_tiers.add_fab_tier_args, on ten
    # CLIs) were in NONE of this module's tables. The unknown-flag fallthrough
    # still carried both into the plan, with opposite outcomes: `fab_tier`
    # happened to MATCH its dialog control's name, so it worked -- by luck,
    # unasserted -- while `fab_overrides` matched no control and no alias, so
    # ai_plan silently ignored it and a replayed plan routed with the bare
    # tier (and with escalation re-enabled, which supplying the file turns
    # off). Same class as `--ordering` above: the fallthrough name is not the
    # control name. Both are string-valued (`_num` falls through to the raw
    # string); both controls are already in reset_params_to_defaults.
    '--fab-tier': 'fab_tier',
    '--fab-overrides': 'fab_overrides_path',
}
LIST_FLAGS = {
    '--layers': 'layers',
    '--power-nets': 'power_nets',
    '--power-nets-widths': 'power_nets_widths',
    '--layer-costs': 'layer_costs',
    # #381 D3: route_diff's polarity-swap allowlist (nargs='+' globs). Carried
    # explicitly (not via the generic unknown-flag fallthrough) so a scoped
    # allowlist survives as a list param that ai_plan's alias routes to the
    # diff tab's polarity_swap_nets_text field.
    '--polarity-swap-nets': 'polarity_swap_nets',
    # #486: route.py's coplanar-waveguide net allowlist (nargs='+' globs).
    # LIST, not FLAG_PARAMS -- as a scalar flag only the FIRST pattern survived.
    '--coplanar-nets': 'coplanar_nets',
    # #284: the rip-existing allowlist (nargs='+' globs), routed to a TextCtrl
    # via ai_plan's alias table. #521's `--protect-nets` was REMOVED from every
    # tool in 53a5a16e and is deliberately absent here: it had no LIST_FLAGS
    # entry, so the conversion loop would have raised KeyError on any manifest
    # carrying it. Protection is now recorded in the .kicad_pro by the step that
    # routes a matched group or diff pair, not passed as a flag.
    '--rip-existing-nets': 'rip_existing_nets',
    # bga_fanout's future-pour declaration (NET:LAYER[,LAYER...] specs,
    # nargs='+'). Review parity finding 5: a recorded manifest carrying the
    # flag used to convert to a plan that silently dropped it. The GUI/plan
    # side accepts the same raw spec strings (fanout_gui parses them like
    # the CLI main does).
    '--plane-net-layers': 'plane_net_layers',
    '--nets': None,  # handled per action
    '--pairs': None,
    '--plane-layers': None,
}
BOOL_FLAGS = {
    '--rip-blocker-nets': 'rip_blocker_nets',
    '--smoothing': 'smoothing',      # #536 octolinear smoothing (default ON)
    '--no-smoothing': 'no_smoothing',  # the negative must survive a replay
    '--add-gnd-vias': 'add_gnd_vias',
    # #485: route_planes area via stitching toggles (planes-tab checkboxes
    # stitch_vias / stitch_edge_fence, applied by the plan executor's
    # generic loop).
    '--stitch-vias': 'stitch_vias',
    '--stitch-edge-fence': 'stitch_edge_fence',
    '--no-gnd-vias': 'no_gnd_vias',
    # route.py spells it --no-bga-zones (plural, nargs='*'); bga_fanout uses the
    # singular. Both map to the GUI's no_bga_zone special (bare = exclude ALL).
    '--no-bga-zone': 'no_bga_zone',
    '--no-bga-zones': 'no_bga_zone',
    # #489 section 9: now on every step that writes pad/via copper (route,
    # route_diff, route_planes, route_disconnected_planes, bga/qfn fanout).
    '--add-teardrops': 'add_teardrops',
    # #487: route_planes' default-on thermal-via arrays; the NEGATIVE flag
    # must survive conversion or a replay re-enables what the run disabled
    # (ai_plan's no_thermal_vias alias unchecks the planes checkbox).
    '--no-thermal-vias': 'no_thermal_vias',
    # The POSITIVE spellings must survive too: ai_plan already consumes
    # both params (thermal_relief checkbox; thermal_vias with the
    # default-ON absent-means-default rule), but a converted manifest that
    # DROPPED the flag would replay a --thermal-relief run without relief
    # spokes (found by the ef4c19a..4db8c18 parity audit).
    '--thermal-relief': 'thermal_relief',
    '--thermal-vias': 'thermal_vias',
    # #515 / PR #533: rip+re-route the selected nets from scratch. Same-named
    # basic-tab checkbox; applied by the plan executor's generic loop.
    '--force-reroute': 'force_reroute',
}

# Flags whose values are file paths / bookkeeping -- consumed, never params.
# --output still feeds the chain-pruning file list. --net-clearances is a
# board-specific JSON path; the GUI derives the same map from the board's live
# net classes, so a replayed plan carries no param for it.
#
# --deadline is a HARNESS budget, not a routing parameter: it changes nothing
# about the copper, only how long the process is allowed to spend making it, and
# the recorded number belongs to whatever external timeout the recording harness
# had. The GUI has no equivalent and needs none -- it has a live Cancel button
# driving the same `cancel_check`. Without this entry it falls through to the
# unknown-flag branch and lands in the plan as a `deadline` param that no dialog
# control matches, which the executor then drops silently; consuming it here
# says so on purpose instead.
IGNORE_FLAGS = {'--output', '--summary-json', '--schematic-dir', '--report',
                '--net-clearances', '--deadline'}

# Per-tool flag renames: bga_fanout calls the trace width --width (routed to the
# Basic-tab track_width, which BGA fanout reads). qfn_fanout also uses --width
# but its GUI home is the QFN panel's own control (see TOOL_FLAG_PARAMS, #381 D7).
TOOL_FLAG_ALIASES = {
    'bga_fanout.py': {'--width': '--track-width'},
    # Both fanout CLIs now accept BOTH spellings (they used to disagree, which
    # cost a recorded run and a replay a wasted step each). Normalize qfn's
    # --track-width to --width HERE, before TOOL_FLAG_PARAMS is consulted --
    # otherwise it falls through to the global FLAG_PARAMS['--track-width'] and
    # lands on the Basic-tab track_width instead of the QFN panel's own
    # control, which is precisely the #381 D7 bug the override below fixes.
    'qfn_fanout.py': {'--track-width': '--width'},
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
        if base in REFUSED_TOOLS:
            # Named and reported, not silently tallied into `skipped`.
            return {'_refused': f"{base}: {REFUSED_TOOLS[base]}"}
    if tool is None:
        return None
    # A `--help` invocation (the agent inspecting a tool during the run) is not a
    # routing step -- skip it so the plan doesn't carry no-op `help: true` steps.
    if '--help' in argv or '-h' in argv:
        return None
    # #459: --preview writes no board and --undo REMOVES copper. The plan format
    # has no way to say either, and the unknown-flag fallthrough turns both into
    # an ordinary route step with nets ['*'] -- so a replayed --undo would ROUTE
    # the whole board where the CLI unrouted a block, the exact inverse. There is
    # no faithful conversion, so refuse loudly rather than emit a wrong one; the
    # caller prints these so a dropped step is never silent.
    for _flag, _why in (('--preview', 'writes no board; nothing to replay'),
                        ('--list-groups', 'prints a listing and exits; '
                                          'converting it would ROUTE the board'),
                        ('--undo', 'removes copper; the plan format cannot '
                                   'express an unroute, and converting it '
                                   'would ROUTE those nets instead')):
        if _flag in argv:
            return {'_refused': f"{tool} {_flag}: {_why}"}
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
        elif a in ('--group', '--group-scope', '--group-by'):
            # #459 placement blocks: these must land as TOP-LEVEL step keys,
            # not in params. ai_plan's route action reads step["group"] /
            # ["group_by"] / ["group_scope"] directly (_group_net_names); the
            # generic unknown-flag path below would file them under params,
            # where nothing reads them -- and per that function's own comment
            # the step then silently widens to the WHOLE BOARD where the CLI
            # routed one block. Same shape as --component just below.
            i += 1
            if i < len(argv) and not argv[i].startswith('--'):
                step[a.lstrip('-').replace('-', '_')] = argv[i]
                i += 1
        elif a == '--component':
            # #537: --component now takes one or more references. One converts
            # to the `component` key the plan executor already understands;
            # several are kept under `components` so the step is not silently
            # narrowed to the first footprint (the GUI list control that
            # consumes them is still to come -- see the issue's Tier 2).
            # Stop at a positional board file: bga_fanout/qfn_fanout still take
            # a SINGLE --component, and `--component U9 out.kicad_pcb` must not
            # swallow the output path out of step['_files'] (which pruning uses).
            vals = []
            i += 1
            while (i < len(argv) and not argv[i].startswith('--')
                   and not argv[i].endswith('.kicad_pcb')):
                vals.append(argv[i])
                i += 1
            if len(vals) == 1:
                step['component'] = vals[0]
            elif vals:
                step['components'] = vals
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
        # route.py also takes net names POSITIONALLY, after the input and output
        # boards -- --nets is optional:
        #     route.py in.kicad_pcb out.kicad_pcb '/Mgmt/LED0' '/Mgmt/VSMPS' ...
        # Same defect as the route_diff branch below: those positionals were
        # collected and dropped, so the step converted to nets: ['*'] -- the
        # exact OPPOSITE of what was recorded, routing every net on the board
        # instead of the handful the CLI retried. eth_tap steps 12 and 16 are
        # both positional retries of specific failed nets.
        # Empty still legitimately means "all nets" (what the CLI does with no
        # net args), so the ['*'] fallback stays for genuinely net-less steps.
        net_globs = [p for p in positional if not p.endswith('.kicad_pcb')]
        step['nets'] = [str(n) for n in (nets or net_globs)] or ['*']
    elif action == 'route_diff':
        # route_diff.py takes its pair patterns POSITIONALLY, after the input and
        # output boards -- there is no --pairs flag on the real CLI:
        #     route_diff.py in.kicad_pcb out.kicad_pcb '*ETH0_A_*' '*ETH0_B_*' ...
        # Those were collected into `positional` and then dropped on the floor,
        # so every recorded diff step converted to `pairs: []`. The GUI reads an
        # empty pairs list as "route every auto-detected pair", so a replayed
        # plan routed pairs the CLI never touched (eth_tap: +22 segments on
        # VCP_P/N and C2_P/N, two pairs absent from the recorded chain, which
        # then cascaded through every later step). 204 of the corpus's 206
        # recorded diff steps were affected.
        # Empty still legitimately means "all pairs" -- that is what the CLI does
        # when given no patterns -- so the ambiguity resolves itself once the
        # patterns actually survive.
        pair_globs = [p for p in positional if not p.endswith('.kicad_pcb')]
        step['pairs'] = [str(n) for n in
                         (lists.get('--pairs') or nets or pair_globs)]
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
              '--layers', '--polarity-swap-nets', '--coplanar-nets',
              '--rip-existing-nets'):
        if k in lists:
            step['params'][LIST_FLAGS[k]] = lists[k]
    return step


# place_fanout_clearance.py has no standalone CLI-tool action, but in the GUI it
# IS a plan step in its own right: the live Claude-plan format (ai_plan.py's
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
    step (matching ai_plan.py's live format), carrying the non-default cap_*
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


def plan_steps_from_manifest(manifest, keep_files=False):
    """(steps, skipped_count) for a recorded manifest -- the whole conversion.

    THE single implementation: main() (and make_plan.py) dump these steps as the
    plan JSON, and tests/gui_parity/replay_plan_vs_run.py calls it with
    ``keep_files=True`` to pair each step with the CLI chain board it produced.
    It used to be a loop in main() with a hand-kept copy in the parity harness.

    ``keep_files``: keep each step's private ``_files`` (its CLI input/output
    boards). The plan JSON must NOT carry them, so main() leaves this False.
    """
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
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'py_router'))  # #522/py_placer layout
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'py_placer'))  # #522/py_placer layout
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'py_tools'))  # #522/py_placer layout
    from redo_stress_test import parse_manifest, compute_prune_keep, is_check_cmd

    cmds = parse_manifest(manifest)
    keep, _info = compute_prune_keep(cmds)

    steps = []
    skipped = 0
    refused = []
    for i, (_cwd, argv) in enumerate(cmds):
        if i not in keep or is_check_cmd(argv):
            continue  # pruned out, or a check/grade command (no GUI step)
        if any(os.path.basename(a) == 'place_fanout_clearance.py' for a in argv):
            # standalone optimize_caps step, matching the live GUI plan (see above)
            step = cap_optimization_step(argv)
            step['_files'] = [a for a in argv if a.endswith('.kicad_pcb')]
            steps.append(step)
            continue
        step = parse_command(argv)
        if step is None:
            # kept-but-not-GUI-representable (place_fanout_clearance, board_image,
            # --help, ...): pruning already accounted for it -- just emit no step.
            skipped += 1
            continue
        if '_refused' in step:
            # A command with no faithful plan representation (#459 --undo /
            # --preview). Emitting nothing is the safe half; the caller MUST
            # report these, because an unroute silently missing from a replay
            # leaves copper the recorded chain had removed.
            refused.append(step['_refused'])
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

    if not keep_files:
        for s in steps:
            s.pop('_files', None)
    if refused:
        # Loud, on stderr, every time -- never a silent drop.
        print(f"WARNING: {len(refused)} command(s) have NO faithful plan "
              f"representation and were NOT converted:", file=sys.stderr)
        for r in refused:
            print(f"  - {r}", file=sys.stderr)
        print("  The replayed plan will DIVERGE from the recorded chain at "
              "these steps. Run them on the CLI.", file=sys.stderr)
    return steps, skipped


def main():
    # Output is POSITIONAL, but `-o/--output` is accepted too: the -o form is the
    # reflex for a tool like this, and the old raw-argv parse silently took the
    # literal string '-o' AS the output path -- writing a plan to a file named
    # `-o` in the cwd while the real target kept its stale contents. That looks
    # exactly like "the converter is broken" until you check the file's mtime.
    argv, positional, out = sys.argv[1:], [], None
    i = 0
    while i < len(argv):
        a = argv[i]
        if a in ('-h', '--help'):
            print(__doc__)
            return 0
        if a in ('-o', '--output'):
            if i + 1 >= len(argv):
                print("error: -o/--output needs a path")
                return 2
            out, i = argv[i + 1], i + 2
            continue
        positional.append(a)
        i += 1
    manifest = positional[0] if positional else None
    if out is None and len(positional) > 1:
        out = positional[1]
    if not manifest or not out:
        print(__doc__)
        return 2
    if out.startswith('-'):
        # Belt and braces: never write to something that looks like a flag.
        print(f"error: refusing to write the plan to {out!r} -- that looks like a "
              f"flag, not a path. Use: manifest_to_plan.py <manifest> <plan.json>")
        return 2
    steps, skipped = plan_steps_from_manifest(manifest)
    with open(out, 'w', encoding='utf-8') as f:
        json.dump({'steps': steps}, f, indent=2)
    print(f"{len(steps)} step(s) written to {out} "
          f"({skipped} kept-but-non-GUI command(s) skipped)")
    print("Load it in the AI tab (Load... next to 'Parsed result') and "
          "press 'Run Selected Steps'.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
