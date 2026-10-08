#!/usr/bin/env python3
"""Headless CONVERTER-parity gate: manifest_to_plan must preserve every
routing-affecting CLI flag into the GUI plan step it emits.

A recorded stress `redo_commands.sh` is the source of truth for the CLI
routing chain. `tests/stress/manifest_to_plan.py` turns each kept command into
the plan step the AI tab loads. If a flag is dropped or renamed in that
translation, the GUI "replay" silently diverges from the CLI board it claims to
reproduce -- exactly how set11 rp2350_fpga_eensy came out with 242 DRC
violations vs the CLI's 0 (issue #361).

Needs NEITHER wx NOR pcbnew. It reuses the converter's OWN pruning to pair each
kept command 1:1 with its plan step (so there is no fragile positional
matching), then asserts each flag with an INDEPENDENT expectation table --
a converter that drops --no-bga-zones fails even though it "agrees with
itself". This is the converter half of GUI/CLI parity; the apply half
(ai_plan.apply_step_params control mapping) is covered by
test_gui_engine_parity.py under KiCad's python.

Example-driven checks only see a flag that a manifest uses AND a table here
names. check_flag_coverage closes that for each FLAG_COVERAGE tool (route.py,
route_diff.py, route_planes.py, bga_fanout.py, qfn_fanout.py): it enumerates
EVERY flag the
real parser accepts and requires each to reach the GUI, or to be listed
CLI-only or as a known gap with the reason. Its run_all half, with the
negative controls, is tests/test_route_flag_plan_coverage.py.

Run:  python3 tests/gui_parity/test_manifest_plan_parity.py [manifest ...]
      (no args -> every runs_set*/*/redo_commands.sh under $STRESS_DIR)
Exit code 1 on any mismatch.
"""
import functools
import glob
import os
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tests" / "stress"))
import manifest_to_plan as m2p  # noqa: E402
from redo_stress_test import (  # noqa: E402
    parse_manifest, compute_prune_keep, is_check_cmd)

STRESS = Path(os.environ.get("STRESS_DIR", str(Path.home() / "Documents/kicad_stress_test")))

# Flag -> plan-step params key it must land in. INDEPENDENT of the converter's
# own FLAG_PARAMS (else the check would be circular): these are what a human
# says must survive.
SCALAR_FLAGS = {
    '--clearance': 'clearance', '--track-width': 'track_width',
    '--width': 'track_width',  # qfn/bga_fanout spelling for trace width
    '--via-size': 'via_size', '--via-drill': 'via_drill',
    '--grid-step': 'grid_step', '--max-iterations': 'max_iterations',
    '--max-ripup': 'max_ripup', '--hole-to-hole-clearance': 'hole_to_hole_clearance',
    '--diff-pair-gap': 'diff_pair_gap', '--escape-method': 'escape_method',
    '--ripup-abandon-metric': 'ripup_abandon_metric',
    # #237's shared fab flags (fab_tiers.add_fab_tier_args). Absent from this
    # table AND the converter's FLAG_PARAMS until 2026-08, so the gate never
    # asserted them: --fab-tier survived only via the converter's unknown-flag
    # fallthrough coinciding with the control name (luck, now pinned), and
    # --fab-overrides fell through under the WRONG name (`fab_overrides` vs
    # the `fab_overrides_path` control) and was silently ignored at apply.
    # Nothing failed because a flag missing from both hand-maintained lists is
    # invisible here, and no corpus manifest used --fab-overrides -- which is
    # why the FIXTURE now carries both (and always runs). When adding a CLI
    # flag, add it to the converter's FLAG_PARAMS and to this table.
    '--fab-tier': 'fab_tier', '--fab-overrides': 'fab_overrides_path',
    # The bus family travels together in a recorded run (runs_set12's
    # duodyne_backplane20 passes all three), so the fixture carries it whole.
    '--ordering': 'ordering_strategy',
    '--bus-detection-radius': 'bus_detection_radius',
    # The guide-corridor / keepout layers and spacing land on their text
    # fields (the fallthrough spelled them after the flag, reaching nothing).
    '--guide-corridor-layer': 'guide_corridor_layer_ctrl',
    '--guide-corridor-spacing': 'guide_corridor_spacing_ctrl',
    '--keepout-layer': 'keepout_layer_ctrl',
    '--diff-chamfer-extra': 'chamfer_extra',
}
BOOL_FLAGS = {
    '--no-bga-zones': 'no_bga_zone', '--no-bga-zone': 'no_bga_zone',
    '--no-gnd-vias': 'no_gnd_vias', '--rip-blocker-nets': 'rip_blocker_nets',
    '--keep-input-copper': 'keep_input_copper',
    # #860 follow-up: qfn_fanout's under-pad via-in-pad opt-in, which #846 made
    # consequential and which reached the GUI through nothing -- unregistered in
    # manifest_to_plan, so a recorded manifest replayed the step without it.
    # manifest_set11 carries the flag, so this row is exercised rather than
    # merely declared.
    '--allow-via-in-pad': 'allow_via_in_pad',
    # route.py's bus mode. On no table here, so nothing asserted it, and the
    # converter's fallthrough carried it as `bus`, which reaches no control:
    # a replayed step routed with bus mode OFF. Its control is bus_enabled.
    '--bus': 'bus_enabled',
    # Found by check_flag_coverage below: controls with another name.
    '--can-swap-to-top-layer': 'can_swap_to_top',
    '--skip-routing': 'skip_routing_check',
    # #856's switch. The converter held it as a VALUE flag, which ate the
    # token after it; the fixture puts --clearance right behind it, so a
    # regression fails the --clearance row as well as this one.
    '--relax-drc-severities': 'relax_drc_severities',
    # Their enabling switches, carried by the fallthrough onto ai_plan's
    # keepout / guide_corridor aliases.
    '--keepout': 'keepout', '--guide-corridor': 'guide_corridor',
    # Found by the enumeration over route_diff.py and bga_fanout.py.
    '--diff-pair-intra-match': 'intra_match_check',
    '--ac-couple-match': 'ac_couple_check',
    '--check-for-previous': 'check_previous',
    '--no-inner-top-layer': 'no_inner_top',
    '--force-escape-direction': 'force_escape',
}
# route.py `--no-X` flags whose GUI home is a POSITIVE checkbox: the step must
# carry that checkbox's name with the value False. Asserting only that the
# name is present would pass a converter that TICKS it -- the exact inverse.
NEGATED_BOOL_FLAGS = {
    '--no-smoothing': 'smoothing',
    '--no-stub-layer-swap': 'enable_layer_switch',
    '--no-power-tap-neckdown': 'power_tap_neckdown_check',
    '--no-fix-drc-settings': 'fix_drc_check',
}
# Repeatable nargs='+' flags: each OCCURRENCE is one group, and every group
# must survive, in order. The fallthrough kept only the last one, under a name
# ai_plan ignored.
GROUP_FLAGS = {
    '--length-match-group': 'length_match_groups',
}
# nargs='+' glob-list flags: every pattern must survive into the plan param
# (as a list, or a single scalar for one pattern). #521 --protect-nets and the
# previously-unasserted --rip-existing-nets.
LIST_FLAGS = {
    '--rip-existing-nets': 'rip_existing_nets',
    '--polarity-swap-nets': 'polarity_swap_nets',
    '--coplanar-nets': 'coplanar_nets',
    # bga_fanout's future-pour declaration: collected by the converter and
    # then never copied into the step (26 kept corpus steps on 18 boards).
    '--plane-net-layers': 'plane_net_layers',
    '--layer-costs': 'layer_costs',
    # bga_fanout's coupled-pair patterns: the BGA panel's Coupled pairs field.
    '--diff-pairs': 'diff_pair_patterns_ctrl',
}
# Per-action overrides of SCALAR_FLAGS. #381 D4: route_diff.py's trace width is
# --track-width, but its GUI home is the diff tab's diff_pair_width control (not
# the Basic-tab track_width), so a diff step must carry it there.
ACTION_SCALAR_OVERRIDES = {
    'route_diff': {'--track-width': 'diff_pair_width'},
    # bga_fanout's --diff-pair-gap is the BGA panel's own coupled-pair gap,
    # never the diff tab's diff_pair_gap (#493).
    'fanout': {'--diff-pair-gap': 'bga_diff_pair_gap'},
}
# Fanout via/clearance/grid live on the fanout tab's shared params too, so
# fanout steps must carry them like route steps do.


def _num(v):
    try:
        f = float(v)
        return int(f) if f == int(f) else f
    except (TypeError, ValueError):
        return v


def _plan_pairs(manifest):
    """Replicate manifest_to_plan.main()'s kept-command loop to yield
    (argv, step) for every command that becomes a GUI step."""
    cmds = parse_manifest(manifest)
    keep, _info = compute_prune_keep(cmds)
    steps = []
    pairs = []
    for i, (_cwd, argv) in enumerate(cmds):
        if i not in keep or is_check_cmd(argv):
            continue
        if any(os.path.basename(a) == 'place_fanout_clearance.py' for a in argv):
            steps.append(m2p.cap_optimization_step(argv, cwd=_cwd))
            # A cap step is still appended (route_planes inheritance below
            # needs the sequence) but never enters `pairs`, because check_pair
            # validates against the ROUTE-step tables, where `--clearance` and
            # `--board-edge-clearance` mean different quantities. Its flags are
            # asserted by check_cap_flags(), against CAP_FLAG_PARAMS -- the
            # comment that used to sit here said "no routing flags to assert",
            # which stopped being true at #733 and is now actively misleading:
            # every cap flag IS asserted, just not from this loop.
            continue
        step = m2p.parse_command(argv)
        if step is None:
            continue
        if step['action'] == 'repair_planes' and 'assignments' not in step:
            for prev in reversed(steps):
                if prev['action'] == 'route_planes' and prev.get('assignments'):
                    step['assignments'] = [dict(a) for a in prev['assignments']]
                    break
        steps.append(step)
        pairs.append((argv, step))
    return pairs


def _plane_layers(argv):
    out = []
    if '--plane-layers' in argv:
        i = argv.index('--plane-layers') + 1
        while i < len(argv) and not argv[i].startswith('--'):
            out.append(argv[i]); i += 1
    return out


def _positional_pairs(argv):
    """The pair globs a route_diff command passes POSITIONALLY.

    Derived independently of manifest_to_plan (that is the point of this gate):
    take the tokens after the tool name up to the FIRST option, and drop the
    input/output boards. route_diff.py has no --pairs flag -- the patterns are
    positional -- and the converter used to collect them into a local it never
    read, so every recorded diff step became `pairs: []`. The GUI reads an empty
    pairs list as "route every auto-detected pair", so 204 of the corpus's 206
    diff steps replayed wider than the chain they came from.
    """
    tool_i = None
    for i, a in enumerate(argv):
        if os.path.basename(a) == 'route_diff.py':
            tool_i = i
            break
    if tool_i is None:
        return []
    out = []
    for a in argv[tool_i + 1:]:
        if a.startswith('-'):
            break
        if not a.endswith('.kicad_pcb'):
            out.append(a)
    return out


def _positional_nets(argv):
    """The net names a route command passes POSITIONALLY.

    Same shape as _positional_pairs, and the same defect: route.py accepts net
    names positionally after the input/output boards (--nets is optional), the
    converter read only lists['--nets'], and an empty list fell through to the
    `or ['*']` default. So a step that retried three specific failed nets
    converted to "route EVERY net on the board" -- the exact opposite of what
    was recorded. eth_tap steps 12 and 16 are both positional retries.
    """
    tool_i = None
    for i, a in enumerate(argv):
        if os.path.basename(a) == 'route.py':
            tool_i = i
            break
    if tool_i is None:
        return []
    out = []
    for a in argv[tool_i + 1:]:
        if a.startswith('-'):
            break
        if not a.endswith('.kicad_pcb'):
            out.append(a)
    return out


def _component_refs(argv):
    """The references after a route.py --component / -C (nargs='+'), up to the
    next option or a positional board file. Derived independently of
    manifest_to_plan, like the positional helpers above."""
    out = []
    for i, a in enumerate(argv):
        if a in ('--component', '-C'):
            for b in argv[i + 1:]:
                if b.startswith('-') or b.endswith('.kicad_pcb'):
                    break
                out.append(b)
    return out


def check_pair(argv, step):
    """Return list of (flag, reason) mismatches for one command/step pair."""
    params = step.get('params', {})
    scalar = dict(SCALAR_FLAGS)
    scalar.update(ACTION_SCALAR_OVERRIDES.get(step.get('action'), {}))
    # #381 D7: a QFN fanout step's --width/--clearance land on the QFN panel's
    # own controls (qfn_track_width/qfn_clearance), not the Basic-tab ones.
    if step.get('action') == 'fanout' and step.get('kind') == 'qfn':
        scalar['--width'] = 'qfn_track_width'
        scalar['--clearance'] = 'qfn_clearance'
    bad = []
    n = 0
    i = 0
    groups = {}          # GROUP_FLAGS flag -> [patterns of each occurrence]
    while i < len(argv):
        a = argv[i]
        if a in NEGATED_BOOL_FLAGS:
            n += 1
            key = NEGATED_BOOL_FLAGS[a]
            if params.get(key) is not False:
                bad.append((a, f"must UNTICK {key} (False), got "
                               f"{params.get(key)!r}"))
            i += 1
            continue
        if a in GROUP_FLAGS:
            vals = []
            i += 1
            while (i < len(argv) and not argv[i].startswith('--')
                   and not argv[i].endswith('.kicad_pcb')):
                vals.append(argv[i]); i += 1
            groups.setdefault(a, []).append(vals)
            continue
        if a in scalar and i + 1 < len(argv):
            want = _num(argv[i + 1])
            got = params.get(scalar[a])
            n += 1
            if got is None or _num(got) != want:
                bad.append((a, f"want {want!r} got {got!r}"))
            i += 2
            continue
        if a in BOOL_FLAGS:
            n += 1
            if not params.get(BOOL_FLAGS[a]):
                bad.append((a, f"bool flag not set ({BOOL_FLAGS[a]})"))
            i += 1
            continue
        if a in LIST_FLAGS:
            want = []
            i += 1
            while i < len(argv) and not argv[i].startswith('--'):
                want.append(argv[i]); i += 1
            got = params.get(LIST_FLAGS[a])
            got = [str(x) for x in got] if isinstance(got, list) else \
                  ([str(got)] if got is not None else [])
            n += 1
            # Compare as the converter's _num normalises: a recorded
            # `--layer-costs 1.0 3.0` is carried as [1, 3], the same numbers.
            if not ({str(_num(w)) for w in want}
                    <= {str(_num(g)) for g in got}):
                bad.append((a, f"want {want} got {got}"))
            continue
        i += 1
    for a, want in groups.items():
        got = params.get(GROUP_FLAGS[a])
        got = [[str(x) for x in g] for g in got] if isinstance(got, list) \
            and all(isinstance(g, list) for g in got) else got
        n += 1
        if got != want:
            bad.append((a, f"want every occurrence as its own group {want} "
                           f"under {GROUP_FLAGS[a]}, got {got!r}"))
    # Positional diff-pair globs must survive into step['pairs'].
    if step.get('action') == 'route_diff':
        want_pairs = _positional_pairs(argv)
        if want_pairs:
            got_pairs = [str(x) for x in (step.get('pairs') or [])]
            n += 1
            if not set(want_pairs).issubset(set(got_pairs)):
                missing = [g for g in want_pairs if g not in got_pairs]
                bad.append(('<positional pairs>',
                            f"want {want_pairs} got {got_pairs} "
                            f"(missing {missing})"))

    # Positional net names must survive into step['nets'] -- and must NOT be
    # silently widened to the ['*'] catch-all.
    if step.get('action') == 'route':
        want_nets = _positional_nets(argv)
        if want_nets:
            got_nets = [str(x) for x in (step.get('nets') or [])]
            n += 1
            if not set(want_nets).issubset(set(got_nets)):
                missing = [g for g in want_nets if g not in got_nets]
                bad.append(('<positional nets>',
                            f"want {want_nets} got {got_nets} "
                            f"(missing {missing})"))

    # route.py / route_diff.py --no-bga-zones is nargs='*': bare disables every
    # BGA zone, refs disable only those components'. The refs must survive as
    # the param (they used to be dropped, leaving "disable ALL").
    if step.get('action') in ('route', 'route_diff'):
        for i, a in enumerate(argv):
            if a not in ('--no-bga-zones', '--no-bga-zone'):
                continue
            refs = []
            for b in argv[i + 1:]:
                if b.startswith('-') or b.endswith('.kicad_pcb'):
                    break
                refs.append(b)
            want = refs or True
            got = params.get('no_bga_zone')
            n += 1
            if got != want:
                bad.append((a, f"want {want!r} (refs, or True for bare) "
                               f"got {got!r}"))

    # route.py --component: every reference must survive, and a step that
    # names components but no patterns must carry NO pattern. route.py drops
    # power/ground from a component's nets only in that case; a '*' fallback
    # would turn it into the intersection that keeps them (and before the
    # route selection read the refs at all, into "route every net").
    if step.get('action') == 'route':
        want_refs = _component_refs(argv)
        if want_refs:
            got_refs = step.get('components') or (
                [step['component']] if step.get('component') else [])
            n += 1
            if [str(r) for r in got_refs] != want_refs:
                bad.append(('--component', f"want refs {want_refs} got "
                                           f"{got_refs}"))
            named = (_positional_nets(argv)
                     or any(a == '--nets' or a.startswith('--nets=')
                            for a in argv))
            if not named and step.get('nets'):
                bad.append(('--component', f"a component-only step must carry "
                                           f"nets [], got {step.get('nets')}"))

    # --plane-layers must survive as the assignment layers
    pl = _plane_layers(argv)
    if pl:
        got = {l for asg in step.get('assignments', [])
               for l in ([asg['layer']] if asg.get('layer') else asg.get('layers', []))}
        n += 1
        if not set(pl).issubset(got):
            bad.append(('--plane-layers', f"want {pl} got {sorted(got)}"))
    return n, bad


FIXTURE = str(Path(__file__).resolve().parent / "fixtures" / "sample_redo_commands.sh")


# --- #381 D5: param -> control resolution gate --------------------------------
# ai_plan.py imports wx at module level, so we can't import it here (no-wx
# gate). Extract its resolution tables and the GUI control attribute names by
# AST instead, then assert every param that MUST reach a control actually does
# (via same-name control, alias->control, or a _apply_special handler). This is
# what blocks a new "no control, ignored" fallthrough (the D5 regression class).
import ast  # noqa: E402

# Params ai_plan resolves through action-specific blocks (not the generic
# alias/special path): composites / same-name-but-formatted controls. Kept
# explicit so the gate credits them without re-parsing every action block.
_ACTION_BLOCK_HANDLED = {
    'track_width', 'clearance', 'via_size', 'via_drill',
    'diff_pair_width', 'diff_pair_gap', 'power_nets', 'power_nets_widths',
    'layer_costs', 'add_gnd_vias', 'gnd_via_distance', 'gnd_via_net',
    'max_track_width', 'min_track_width',
}

# Params that MUST resolve to a GUI control (the D5 fallback list + D3 polarity
# + D7 QFN width/clearance).
_MUST_RESOLVE = {
    'rip_existing_nets',
    'impedance', 'ordering', 'direction', 'time_matching',
    'keepout', 'guide_corridor', 'length_match_groups', 'swappable_nets',
    'polarity_swap_nets', 'qfn_track_width', 'qfn_clearance',
    # #733: place_fanout_clearance's --board-edge-clearance -> the SHARED
    # Basic-tab edge control (not a cap_* one), which ai_plan's
    # _GEOMETRY_OVERRIDE_CHECKS ticks by this exact name. This set gates the
    # NAME (does it reach a control?); check_cap_flags below gates the
    # converter ROW that produces it. Measured: deleting the CAP_FLAG_PARAMS
    # row leaves this half green, which is why both exist.
    'board_edge_clearance',
    # route.py --bus: bus_enabled is what new conversions emit, `bus` is the
    # name plans converted before the converter row carry (the alias). Same
    # pair for the two other renamed controls and --length-match-group's
    # legacy singular.
    'bus', 'bus_enabled',
    'can_swap_to_top_layer', 'can_swap_to_top',
    'skip_routing', 'skip_routing_check',
    'length_match_group',
    'guide_corridor_layer', 'guide_corridor_layer_ctrl',
    'guide_corridor_spacing', 'guide_corridor_spacing_ctrl',
    'keepout_layer', 'keepout_layer_ctrl',
    # route_diff / bga_fanout fallthrough names older conversions carry.
    'diff_pair_intra_match', 'ac_couple_match', 'diff_chamfer_extra',
    'check_for_previous', 'no_inner_top_layer', 'force_escape_direction',
    'diff_pairs',
}


def _ai_plan_tables():
    """AST-extract _PARAM_CONTROL_ALIASES (dict) and _PARAM_SPECIAL (set) from
    ai_plan.py without importing it (it imports wx)."""
    src = (REPO / "kicad_routing_plugin" / "ai_plan.py").read_text()
    tree = ast.parse(src)
    aliases, special = {}, set()
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        for t in node.targets:
            if isinstance(t, ast.Name) and t.id == '_PARAM_CONTROL_ALIASES':
                aliases = ast.literal_eval(node.value)
            elif isinstance(t, ast.Name) and t.id == '_PARAM_SPECIAL':
                special = set(ast.literal_eval(node.value))
    return aliases, special


def _gui_control_attrs():
    """Collect every `self.X = ...` attribute name across the plugin GUI source
    files -- the universe of control attributes an alias may target."""
    attrs = set()
    gui_dir = REPO / "kicad_routing_plugin"
    # routing_dialog.py is this branch's swig_gui.py (renamed by the IPC port).
    for fn in ("routing_dialog.py", "differential_gui.py", "fanout_gui.py",
               "planes_gui.py"):
        tree = ast.parse((gui_dir / fn).read_text())
        for node in ast.walk(tree):
            if isinstance(node, ast.Assign):
                targets = node.targets
            elif isinstance(node, ast.AnnAssign):
                targets = [node.target]
            else:
                continue
            for t in targets:
                if (isinstance(t, ast.Attribute)
                        and isinstance(t.value, ast.Name)
                        and t.value.id == 'self'):
                    attrs.add(t.attr)
    return attrs


# --- #772: OWNER-SCOPED resolution -------------------------------------------
# check_param_resolution above asks "does a control with this name exist
# ANYWHERE across the four GUI files". Every cap_* control has always existed,
# so that half stayed green for the whole time the plan executor could not
# reach one of them: ai_plan._owners() searched [dialog] for an optimize_caps
# step, and the controls live on fanout_tab.bga_options.
#
# This arm closes that hole WITHOUT wx. It AST-extracts ai_plan's
# _ACTION_OWNERS table and a PER-CLASS map of control attributes, then walks
# the owner chain exactly as _owners() does and asserts the param resolves on
# one of them.
_OWNER_CLASSES = {
    'differential_tab': 'DifferentialTab',
    'fanout_tab': 'FanoutTab',
    'planes_tab': 'PlanesTab',
    'bga_options': 'BGAOptionsPanel',
    'qfn_options': 'QFNOptionsPanel',
    'create_options': 'CreatePlanesOptionsPanel',
    '<dialog>': 'RoutingDialog',
}

# action -> params that must resolve ON THAT ACTION'S OWNERS.
_MUST_RESOLVE_ON = {
    'optimize_caps': {
        'cap_capture_radius', 'cap_near_margin', 'cap_step',
        'cap_max_displacement', 'cap_max_displacement_cap',
        'cap_displacement_growth', 'cap_board_edge_clearance',
        'cap_max_passes', 'cap_prefix', 'cap_allow_rotation',
        # #742: the CLI's --default-via-size on its OWN control. It must NOT
        # resolve as via_size -- see the change detector in check_cap_flags.
        'cap_default_via_size',
        # #1067: the CLI's --intent, the path the cap pass loads its decap
        # limits from.
        'cap_intent_path',
        # the Basic-tab knobs a cap step legitimately drives: `clearance` is
        # the GUI's spelling of "--clearance was GIVEN" (#768), and grid_step
        # is the position snap the pass reads through get_shared_params.
        'clearance', 'grid_step',
    },
    'route_planes': {'stitch_pitch', 'gnd_via_net', 'zone_clearance'},
    'route_diff': {'diff_pair_width', 'diff_pair_gap'},
    # #772: these two need `fanout` to search the option PANELS. They are
    # what the per-action block reaches BY HAND today, so the generic loop
    # could not. If the fanout widening is ever dropped, drop this row with
    # it -- the per-action block still delivers them.
    #
    # qfn_track_width / qfn_clearance are deliberately NOT here: they are in
    # _GENERIC_SKIP['fanout'], so the generic loop never looks for them and
    # 'unreachable' would be the wrong word. Listing them made this row a
    # coupling gate for a commit marked SEPARABLE rather than a delivery
    # gate, which an adversarial review called out.
    'fanout': {'exit_margin', 'extension'},
}


def _class_control_attrs():
    """{class name: {control attribute names}} across the four GUI files.

    Per-CLASS, where _gui_control_attrs is a flat union -- that union is what
    made owner-scoped unreachability invisible.
    """
    out = {}
    gui_dir = REPO / "kicad_routing_plugin"
    # routing_dialog.py is this branch's swig_gui.py (renamed by the IPC port).
    for fn in ("routing_dialog.py", "differential_gui.py", "fanout_gui.py",
               "planes_gui.py"):
        tree = ast.parse((gui_dir / fn).read_text(encoding='utf-8'))
        for node in ast.walk(tree):
            if not isinstance(node, ast.ClassDef):
                continue
            attrs = out.setdefault(node.name, set())
            for n in ast.walk(node):
                if isinstance(n, ast.Assign):
                    targets = n.targets
                elif isinstance(n, ast.AnnAssign):
                    targets = [n.target]
                else:
                    continue
                for t in targets:
                    if (isinstance(t, ast.Attribute)
                            and isinstance(t.value, ast.Name)
                            and t.value.id == 'self'):
                        attrs.add(t.attr)
            attrs |= _setattr_loop_attrs(node)
    return out


def _setattr_loop_attrs(node):
    """Control names created by `for name, ... in <list of tuples>:
    setattr(self, name, ctrl)` (and `setattr(self, name + '_check', chk)`).

    THREE loops in swig_gui.py build controls this way -- the geometry floors
    with their override checkboxes, the integer params and the float params --
    and a plain `self.X = ...` walk cannot see any of them. That blind spot is
    pre-existing and was harmless only because the affected names are handled
    by per-action blocks; it is not harmless for an owner-scoped check, which
    would report FALSE failures for clearance / track_width / the via floors.
    """
    tables = {}
    for n in ast.walk(node):
        if (isinstance(n, ast.Assign) and len(n.targets) == 1
                and isinstance(n.targets[0], ast.Name)
                and isinstance(n.value, (ast.List, ast.Tuple))):
            names = [e.elts[0].value for e in n.value.elts
                     if isinstance(e, ast.Tuple) and e.elts
                     and isinstance(e.elts[0], ast.Constant)
                     and isinstance(e.elts[0].value, str)]
            if names:
                tables[n.targets[0].id] = names
    out = set()
    for n in ast.walk(node):
        if not isinstance(n, ast.For):
            continue
        names = tables.get(n.iter.id) if isinstance(n.iter, ast.Name) else None
        if (not names or not isinstance(n.target, ast.Tuple)
                or not n.target.elts
                or not isinstance(n.target.elts[0], ast.Name)):
            continue
        var = n.target.elts[0].id
        for c in ast.walk(n):
            if not (isinstance(c, ast.Call) and isinstance(c.func, ast.Name)
                    and c.func.id == 'setattr' and len(c.args) >= 2
                    and isinstance(c.args[0], ast.Name)
                    and c.args[0].id == 'self'):
                continue
            a = c.args[1]
            if isinstance(a, ast.Name) and a.id == var:
                out |= set(names)
            elif (isinstance(a, ast.BinOp) and isinstance(a.op, ast.Add)
                  and isinstance(a.left, ast.Name) and a.left.id == var
                  and isinstance(a.right, ast.Constant)
                  and isinstance(a.right.value, str)):
                out |= {x + a.right.value for x in names}
    return out


def _action_owners_table():
    """AST-extract ai_plan._ACTION_OWNERS without importing it (it needs wx)."""
    src = (REPO / "kicad_routing_plugin" / "ai_plan.py").read_text(
        encoding='utf-8')
    for node in ast.parse(src).body:
        if isinstance(node, ast.Assign):
            for t in node.targets:
                if isinstance(t, ast.Name) and t.id == '_ACTION_OWNERS':
                    return ast.literal_eval(node.value)
    return None


def check_owner_scoping():
    """Return [(what, why)] for params that cannot resolve on their action."""
    table = _action_owners_table()
    if table is None:
        return [('_ACTION_OWNERS',
                 'ai_plan no longer exposes the owner table as a module-level '
                 'literal -- this gate cannot see which controls a step can '
                 'reach, and #772 is exactly what that blindness costs')]
    bad = []
    ent = table.get('optimize_caps')
    if not ent or 'bga_options' not in (ent[1] or ()):
        bad.append(('optimize_caps',
                    'the _ACTION_OWNERS entry is %r -- it must search '
                    'fanout_tab.bga_options, which owns every cap_* control'
                    % (ent,)))
    by_class = _class_control_attrs()
    aliases, special = _ai_plan_tables()
    # The owner ATTRIBUTE must exist on its tab, not merely the class it
    # names. Without this the gate FALSE-PASSES the exact defect it was
    # written for: renaming `self.bga_options` makes _owners() fall back to
    # [fanout_tab, dialog] -- #772 verbatim -- while this function still
    # resolves every cap param through BGAOptionsPanel and reports OK.
    # Measured by mutation before this check existed: exit 0.
    _gui_src = {}
    # routing_dialog.py is this branch's swig_gui.py (renamed by the IPC port).
    for _fn in ('routing_dialog.py', 'differential_gui.py', 'fanout_gui.py',
                'planes_gui.py'):
        _gui_src[_fn] = (REPO / 'kicad_routing_plugin' / _fn).read_text(
            encoding='utf-8')
    _all_gui = '\n'.join(_gui_src.values())
    for _owner in sorted({o for _t, _s in table.values() for o in _s}):
        if ('self.%s = ' % _owner) not in _all_gui:
            bad.append(('<owner>.%s' % _owner,
                        'no GUI file assigns `self.%s`, so getattr on the '
                        'tab returns None and every param that should '
                        'resolve there is silently unreachable' % _owner))
    for action, params in sorted(_MUST_RESOLVE_ON.items()):
        tab_attr, subs = table.get(action, (None, ()))
        chain = list(subs) + ([tab_attr] if tab_attr else []) + ['<dialog>']
        reachable = set()
        for owner in chain:
            reachable |= by_class.get(_OWNER_CLASSES.get(owner, ''), set())
        for p in sorted(params):
            if p in special:
                continue
            tgt = aliases.get(p, p)
            if tgt not in reachable:
                bad.append(('%s.%s' % (action, p),
                            'resolves to %r, on none of %s -- the step would '
                            'log "no control, ignored" and run at the reset '
                            'default' % (tgt, chain)))
    return bad


# --- route.py FLAG ENUMERATION -----------------------------------------------
# Every arm above is driven by EXAMPLES: a flag is asserted only when a
# recorded manifest (or the fixture) uses it AND a hand-kept table here names
# it. A flag on neither is invisible, whatever the converter does with it.
# #237 said so of --fab-overrides, and `--bus` then shipped the same way: its
# Advanced-options checkbox is `bus_enabled`, the converter's fallthrough
# spelled the param `bus`, ai_plan logged "no control for bus, ignored", and a
# replayed step routed with bus mode OFF.
#
# This arm asks the question of EVERY flag route.py's argparse accepts --
# krt_capabilities.script_flags, which tests/test_798_registrar_flags.py
# proves EXACT against the real parser -- and requires each to be accounted
# for in exactly ONE way:
#
#   reached           the converter carries it into a plan param that the
#                     executor sets on a control the ROUTE action can reach
#                     (same name, or through _PARAM_CONTROL_ALIASES), that
#                     _apply_special handles, or that the route action block
#                     handles (_GENERIC_SKIP['route']); or into a top-level
#                     step key the route selection reads (step['nets'], ...);
#   ROUTE_CLI_ONLY    deliberately not replayed, with the reason;
#   ROUTE_KNOWN_GAPS  a real gap, recorded so it stays visible.
#
# A new route.py flag is in none of them and FAILS here until someone says
# which. A listed flag that starts reaching the GUI fails too, so no entry can
# outlive its reason.

ROUTE_CLI_ONLY = {
    # The converter REFUSES the whole command: no faithful plan step exists.
    '--undo': "removes copper; the converter refuses the command (#459)",
    '--preview': "writes no board; the converter refuses the command (#459)",
    '--list-groups': "prints a listing and exits; the converter refuses the "
                     "command",
    '--preview-png': "renders a PNG, and only with --preview, which the "
                     "converter refuses",
    '--capabilities': "prints the capability document and exits; routes "
                      "nothing",
    # Files and bookkeeping: the GUI routes the LIVE board and keeps its
    # results in memory, so there is no file for these to name.
    '--output': "the output FILE path (it feeds chain pruning); the GUI "
                "writes the live board",
    '--overwrite': "CLI file handling (write over the input file)",
    '--json-out': "writes the JSON_SUMMARY to a file for a harness; the GUI "
                  "consumes results in memory",
    '--strict-sizes': "a harness EXIT-CODE flag (#857); changes no copper",
    '--keep-thermal': "deprecated no-op (#856)",
    '--net-clearances': "a board-specific JSON path; the GUI derives the same "
                        "map from the live board's net classes",
    '--schematic-dir': "writes pad swaps back to .kicad_sch files, no copper; "
                       "the recorded path is the recording host's",
    '--write-fill': "fills zones in the OUTPUT FILE for delivery (#910); the "
                    "GUI edits a live board KiCad fills itself",
    '--enable-used-layers': "adds used layers back to the output FILE's "
                            "(layers) table; changes no copper",
    # Diagnostics: console or User-layer output only, never copper.
    '--verbose': "console verbosity only",
    '--stats': "prints A* search statistics only",
    '--debug-memory': "prints memory statistics only",
    '--debug-lines': "debug geometry on User.3/4/8/9 only, never copper",
}

# KNOWN GAPS, PENDING ANDY'S DECISION. Each flag changes what a step routes,
# a GUI plan replay does NOT honour it, and the fix is more than a converter
# row or an alias. Listed so this gate stays green while the list stays
# visible; taking an entry off is that code change.
#
# EMPTY now. It held --component (a route step never read its refs, so a
# replay routed every net), the keepout / guide-corridor layer and spacing
# flags (controls that were never reset), and --no-fix-drc-settings (its
# checkbox was a preference no step reset); each left when its fix landed.
ROUTE_KNOWN_GAPS = {}

# REACHED, but the control is not restored by swig_gui's
# reset_params_to_defaults, which the plan executor runs before EVERY step: a
# step that sets it leaks it into every later step (CLAUDE.md: add the control
# to reset_params_to_defaults "or the param leaks between steps").
#
# EMPTY on purpose. keepout_check and guide_corridor_check were here -- alias
# targets a step could tick and no reset ever unticked -- until their reset
# lines landed. An entry added here is a NEW leak: fix the reset instead.
ROUTE_RESET_KNOWN_GAPS = {}

# The probe: a route step with --nets already given, so a switch's trailing
# PROBE_VALUE lands as an (ignored) positional instead of changing the nets,
# while a valued flag -- including --nets itself -- takes it.
_ROUTE_PROBE = ['python3', 'py_router/route.py', 'in.kicad_pcb',
                'out.kicad_pcb', '--nets', 'PROBE_NET']
_PROBE_VALUE = 'PROBE_VALUE'

# The other tools. A reason route.py already gives is reused verbatim where
# the flag means the same thing there (the shared registrars and the file /
# diagnostic flags); the rest are the tool's own.
_SAME_AS_ROUTE = ('--debug-lines', '--debug-memory', '--enable-used-layers',
                  '--keep-thermal', '--net-clearances', '--output',
                  '--overwrite', '--schematic-dir', '--strict-sizes',
                  '--verbose')

DIFF_CLI_ONLY = {f: ROUTE_CLI_ONLY[f] for f in _SAME_AS_ROUTE}
# KNOWN GAPS, PENDING ANDY'S DECISION (route_diff.py).
DIFF_KNOWN_GAPS = {
    '--diff-pair-centerline-setback': (
        "its control exists (differential_tab.centerline_setback, 0 = auto, "
        "which the diff tab passes as diff_pair_centerline_setback) but "
        "reset_params_to_defaults never restores it, so an alias alone would "
        "leak one step's setback into every later diff step. Fix: alias + "
        "reset line. 0 recorded uses"),
}

PLANES_CLI_ONLY = dict(
    {f: ROUTE_CLI_ONLY[f] for f in ('--debug-lines', '--enable-used-layers',
                                    '--keep-thermal', '--output',
                                    '--overwrite', '--strict-sizes',
                                    '--verbose')},
    **{'--dry-run': "analysis only, writes no board (the planes tab always "
                    "runs create_plane with dry_run=True and applies the "
                    "result to the live board itself)",
       '--skip-existing-zones': "the planes tab always behaves as if it were "
                                "given (skip_existing_zones=True: keep an "
                                "existing same-net zone on the live board); "
                                "what it cannot replay is the flag's ABSENCE"})
# KNOWN GAPS, PENDING ANDY'S DECISION (route_planes.py). No control on the
# planes tab: planes_gui hands create_plane a fixed default for each, so a
# recorded non-default value is lost. They steer the routed connections of a
# multi-net (Voronoi split) plane layer and its zone fill. 0 recorded uses of
# any of them.
PLANES_KNOWN_GAPS = {
    '--min-thickness': "no control; planes_gui passes "
                       "defaults.PLANE_MIN_THICKNESS",
    '--plane-max-iterations': "no control; planes_gui passes "
                              "defaults.MAX_ITERATIONS",
    '--voronoi-seed-interval': "no control; planes_gui passes 2.0",
}

BGA_CLI_ONLY = {f: ROUTE_CLI_ONLY[f] for f in ('--enable-used-layers',
                                               '--keep-thermal', '--output',
                                               '--strict-sizes')}
# KNOWN GAPS, PENDING ANDY'S DECISION (bga_fanout.py). (--diff-pairs and
# --diff-pair-gap left when the BGA panel got its Coupled pairs field and its
# own coupled-pair gap: 51 kept corpus steps on 36 boards carry both.)
BGA_KNOWN_GAPS = {
    '--primary-escape': (
        "its control is the escape_direction RadioBox, which the plan "
        "executor's _set_control cannot set (no RadioBox branch, no "
        "SetValue) and reset_params_to_defaults does not restore. Needs a "
        "special handler + reset line. 0 recorded uses"),
}

QFN_CLI_ONLY = dict(
    {f: ROUTE_CLI_ONLY[f] for f in ('--enable-used-layers', '--keep-thermal',
                                    '--output', '--strict-sizes')},
    **{'--layer': "overrides the escape layer, default the part's MOUNTED "
                  "layer -- which is what the QFN panel always routes on "
                  "(the tab passes component_layer). Any other value floats "
                  "the stubs off the SMD pads (#96; the CLI warns). 0 of "
                  "1018 recorded qfn_fanout calls use it"})

# The tools whose EVERY flag is held to account: the plan action a recorded
# command converts to, and the probe -- a command that already names its
# scope, so a switch's trailing PROBE_VALUE lands as an ignored positional
# while a valued flag (the scope flag included) takes it. Each tool carries
# its own lists, because a flag can mean different things on two tools.
FLAG_COVERAGE = {
    'route.py': dict(action='route', probe=_ROUTE_PROBE,
                     cli_only=ROUTE_CLI_ONLY, known_gaps=ROUTE_KNOWN_GAPS,
                     reset_gaps=ROUTE_RESET_KNOWN_GAPS),
    'route_diff.py': dict(
        action='route_diff',
        probe=['python3', 'py_router/route_diff.py', 'in.kicad_pcb',
               'out.kicad_pcb', '--nets', 'PROBE_NET'],
        cli_only=DIFF_CLI_ONLY, known_gaps=DIFF_KNOWN_GAPS, reset_gaps={}),
    'route_planes.py': dict(
        action='route_planes',
        probe=['python3', 'py_router/route_planes.py', 'in.kicad_pcb',
               'out.kicad_pcb', '--nets', 'PROBE_NET', '--plane-layers',
               'In1.Cu'],
        cli_only=PLANES_CLI_ONLY, known_gaps=PLANES_KNOWN_GAPS,
        reset_gaps={}),
    'bga_fanout.py': dict(
        action='fanout',
        probe=['python3', 'py_router/bga_fanout.py', 'in.kicad_pcb',
               'out.kicad_pcb', '--component', 'U1', '--nets', 'PROBE_NET'],
        cli_only=BGA_CLI_ONLY, known_gaps=BGA_KNOWN_GAPS, reset_gaps={}),
    'qfn_fanout.py': dict(
        action='fanout',
        probe=['python3', 'py_router/qfn_fanout.py', 'in.kicad_pcb',
               'out.kicad_pcb', '--component', 'U1', '--nets', 'PROBE_NET'],
        cli_only=QFN_CLI_ONLY, known_gaps={}, reset_gaps={}),
}

# The widget classes the executor's _set_control can SET: its explicit
# CheckBox / SpinCtrl / SpinCtrlDouble / Choice branches, plus the wx classes
# that reach its `hasattr(ctrl, "SetValue")` branch. A Button, a StaticText, a
# custom panel or the `layer_checks` dict has no SetValue, so a param
# resolving to one is logged "no control ..., ignored" however its name
# matches.
_SETTABLE_WX = frozenset({'CheckBox', 'SpinCtrl', 'SpinCtrlDouble', 'Choice',
                          'TextCtrl', 'ComboBox', 'RadioButton', 'Slider',
                          'ToggleButton', 'Gauge'})

# routing_dialog.py is this branch's swig_gui.py (renamed by the IPC port).
_GUI_FILES = ("routing_dialog.py", "differential_gui.py", "fanout_gui.py",
              "planes_gui.py")


def _wx_class(node):
    """'CheckBox' for a `wx.CheckBox(...)` call node, else None."""
    if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
            and isinstance(node.func.value, ast.Name)
            and node.func.value.id == 'wx'):
        return node.func.attr
    return None


def _is_self_attr(node):
    return (isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name)
            and node.value.id == 'self')


def _local_wx_bindings(node):
    """{local name: wx class} for every `v = wx.Y(...)` under `node`."""
    out = {}
    for n in ast.walk(node):
        if isinstance(n, ast.Assign):
            kind = _wx_class(n.value)
            if kind:
                for t in n.targets:
                    if isinstance(t, ast.Name):
                        out[t.id] = kind
    return out


@functools.lru_cache(maxsize=None)
def _class_widgets():
    """{class name: {attribute: wx class}} for every SETTABLE control.

    Resolved per METHOD, because swig_gui builds controls three ways and a
    `self.X = ...` walk sees only the first:
      * `self.X = wx.Y(...)`, or `v = wx.Y(...)` then `self.X = v`;
      * a table loop, `for name, ... in <list of tuples>:` binding
        `ctrl = wx.Y(...)` and then `setattr(self, name, ctrl)` (and
        `setattr(self, name + '_check', chk)`). max_iterations and
        hole_to_hole_clearance exist ONLY that way: a prototype of this
        gate that missed the loops reported 57 of route.py's flags as
        unreached, most of them falsely.
    Stricter than _class_control_attrs on purpose: that one collects every
    `self.X`, so `pcb_data` or the `layer_checks` dict would pass as a
    control, and _set_control returns False for both.
    """
    out = {}
    for fn in _GUI_FILES:
        tree = ast.parse((REPO / "kicad_routing_plugin" / fn).read_text(
            encoding='utf-8'))
        for cls in ast.walk(tree):
            if not isinstance(cls, ast.ClassDef):
                continue
            widgets = out.setdefault(cls.name, {})
            for meth in cls.body:
                if not isinstance(meth, ast.FunctionDef):
                    continue
                # Two passes: ast.walk is breadth-first, not source order, so
                # `self.X = v` can be visited before the `v = wx.Y(...)` it
                # names.
                local, tables = _local_wx_bindings(meth), {}
                for n in ast.walk(meth):
                    if not isinstance(n, ast.Assign):
                        continue
                    kind = _wx_class(n.value)
                    if kind is None and isinstance(n.value, ast.Name):
                        kind = local.get(n.value.id)
                    for t in n.targets:
                        if kind and _is_self_attr(t):
                            widgets[t.attr] = kind
                    if (len(n.targets) == 1
                            and isinstance(n.targets[0], ast.Name)
                            and isinstance(n.value, (ast.List, ast.Tuple))):
                        names = [e.elts[0].value for e in n.value.elts
                                 if isinstance(e, ast.Tuple) and e.elts
                                 and isinstance(e.elts[0], ast.Constant)
                                 and isinstance(e.elts[0].value, str)]
                        if names:
                            tables[n.targets[0].id] = names
                for loop in ast.walk(meth):
                    if not (isinstance(loop, ast.For)
                            and isinstance(loop.iter, ast.Name)
                            and loop.iter.id in tables
                            and isinstance(loop.target, ast.Tuple)
                            and loop.target.elts
                            and isinstance(loop.target.elts[0], ast.Name)):
                        continue
                    var = loop.target.elts[0].id
                    # The loop's OWN bindings first: one method can bind
                    # `ctrl` to a SpinCtrl in one loop and a SpinCtrlDouble
                    # in the next.
                    local = dict(local, **_local_wx_bindings(loop))
                    for c in ast.walk(loop):
                        if not (isinstance(c, ast.Call)
                                and isinstance(c.func, ast.Name)
                                and c.func.id == 'setattr'
                                and len(c.args) == 3
                                and isinstance(c.args[0], ast.Name)
                                and c.args[0].id == 'self'):
                            continue
                        val = c.args[2]
                        kind = (local.get(val.id) if isinstance(val, ast.Name)
                                else _wx_class(val))
                        if not kind:
                            continue
                        key = c.args[1]
                        if isinstance(key, ast.Name) and key.id == var:
                            suffix = ''
                        elif (isinstance(key, ast.BinOp)
                              and isinstance(key.op, ast.Add)
                              and isinstance(key.left, ast.Name)
                              and key.left.id == var
                              and isinstance(key.right, ast.Constant)
                              and isinstance(key.right.value, str)):
                            suffix = key.right.value
                        else:
                            continue
                        for x in tables[loop.iter.id]:
                            widgets[x + suffix] = kind
            for attr in [a for a, k in widgets.items()
                         if k not in _SETTABLE_WX]:
                del widgets[attr]
    return out


def _function(tree, name):
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    return None


def _action_branch(func, action):
    """The body of `if action == "<action>":` directly in `func` (the elif
    chain lives in orelse, so only this action's own statements come back)."""
    for node in ast.walk(func):
        if (isinstance(node, ast.If) and isinstance(node.test, ast.Compare)
                and isinstance(node.test.left, ast.Name)
                and node.test.left.id == 'action'
                and len(node.test.comparators) == 1
                and isinstance(node.test.comparators[0], ast.Constant)
                and node.test.comparators[0].value == action):
            return node.body
    return None


def _strings_in(nodes):
    return {c.value for n in nodes for c in ast.walk(n)
            if isinstance(c, ast.Constant) and isinstance(c.value, str)}


def _keys_read(nodes, var):
    """Keys read as `<var>.get('k')` or `<var>['k']` anywhere in `nodes`."""
    keys = set()
    for n in nodes:
        for c in ast.walk(n):
            if (isinstance(c, ast.Call) and isinstance(c.func, ast.Attribute)
                    and c.func.attr == 'get'
                    and isinstance(c.func.value, ast.Name)
                    and c.func.value.id == var and c.args
                    and isinstance(c.args[0], ast.Constant)):
                keys.add(c.args[0].value)
            elif (isinstance(c, ast.Subscript)
                  and isinstance(c.value, ast.Name) and c.value.id == var
                  and isinstance(c.slice, ast.Constant)):
                keys.add(c.slice.value)
    return keys


@functools.lru_cache(maxsize=None)
def _apply_facts(action):
    """What ai_plan does with a step of `action`, read by AST (it imports wx).

    Returns (skip, block, special_handled, selection_keys):
      skip             _GENERIC_SKIP[action] -- params the generic loop leaves
                       to the action block;
      block            string constants in apply_step_params's block for the
                       action, so a skipped param is proved HANDLED there, not
                       dropped;
      special_handled  names _apply_special actually tests `name` against: a
                       _PARAM_SPECIAL entry with no branch returns False and is
                       logged "ignored" like any other;
      selection_keys   top-level step keys apply_step_selection's branch for
                       the action reads, directly or through a helper it hands
                       `step` to.
    """
    tree = ast.parse((REPO / "kicad_routing_plugin" / "ai_plan.py").read_text(
        encoding='utf-8'))
    apply = _function(tree, 'apply_step_params')
    skip = set()
    for n in ast.walk(apply):
        if (isinstance(n, ast.Assign) and len(n.targets) == 1
                and isinstance(n.targets[0], ast.Name)
                and n.targets[0].id == '_GENERIC_SKIP'):
            skip = set(ast.literal_eval(n.value).get(action, ()))
    block = _strings_in(_action_branch(apply, action) or [])
    special_handled = set()
    for c in ast.walk(_function(apply, '_apply_special')):
        if (isinstance(c, ast.Compare) and isinstance(c.left, ast.Name)
                and c.left.id == 'name'):
            special_handled |= _strings_in(c.comparators)
    branch = _action_branch(_function(tree, 'apply_step_selection'),
                            action) or []
    keys = _keys_read(branch, 'step')
    for n in branch:
        for c in ast.walk(n):
            if not (isinstance(c, ast.Call) and isinstance(c.func, ast.Name)):
                continue
            callee = _function(tree, c.func.id)
            for j, arg in enumerate(c.args):
                if (callee is not None and isinstance(arg, ast.Name)
                        and arg.id == 'step' and j < len(callee.args.args)):
                    keys |= _keys_read(callee.body, callee.args.args[j].arg)
    return skip, block, special_handled, keys


@functools.lru_cache(maxsize=None)
def _reset_touched():
    """Control names swig_gui's reset_params_to_defaults restores.

    It reaches the tab panels three ways besides `self.X`: a local alias
    (`_po = self.planes_tab.create_options; _po.stitch_pitch.SetValue(..)`),
    a holder search driven by a table of NAMES (`_fctl('exit_margin')` over
    the fanout tab and its option panels), and delegation to another dialog
    method (`self.reset_cap_params_to_defaults()`). So a control counts as
    restored when the reset -- or a dialog method it calls -- names it as an
    attribute or as an identifier string, plus the `for _name in (...):
    getattr(self, _name + sfx)` loops of the geometry floors.

    Names, not owner paths: this leans on control names being disjoint
    across the owners a step searches, which ai_plan's _ACTION_OWNERS comment
    measured on the live dialog (#772).
    """
    tree = ast.parse((REPO / "kicad_routing_plugin" / "routing_dialog.py").read_text(
        encoding='utf-8'))
    cls = next(n for n in tree.body
               if isinstance(n, ast.ClassDef) and n.name == 'RoutingDialog')
    methods = {m.name: m for m in cls.body if isinstance(m, ast.FunctionDef)}
    touched, seen, todo = set(), set(), ['reset_params_to_defaults']
    while todo:
        name = todo.pop()
        if name in seen or name not in methods:
            continue
        seen.add(name)
        fn = methods[name]
        for c in ast.walk(fn):
            if isinstance(c, ast.Attribute):
                touched.add(c.attr)
            elif (isinstance(c, ast.Constant) and isinstance(c.value, str)
                  and c.value.isidentifier()):
                touched.add(c.value)
            if (isinstance(c, ast.Call) and isinstance(c.func, ast.Attribute)
                    and _is_self_attr(c.func)):
                todo.append(c.func.attr)
    fn = methods['reset_params_to_defaults']
    for loop in ast.walk(fn):
        if not (isinstance(loop, ast.For) and isinstance(loop.target, ast.Name)
                and isinstance(loop.iter, (ast.Tuple, ast.List))):
            continue
        names = [e.value for e in loop.iter.elts
                 if isinstance(e, ast.Constant) and isinstance(e.value, str)]
        for c in ast.walk(loop):
            if not (isinstance(c, ast.Call) and isinstance(c.func, ast.Name)
                    and c.func.id == 'getattr' and len(c.args) >= 2):
                continue
            key = c.args[1]
            if isinstance(key, ast.Name) and key.id == loop.target.id:
                touched |= set(names)
            elif (isinstance(key, ast.BinOp) and isinstance(key.left, ast.Name)
                  and key.left.id == loop.target.id
                  and isinstance(key.right, ast.Constant)):
                touched |= {x + key.right.value for x in names}
    return touched


def tool_flags(tool):
    """Every long flag `tool`'s argparse accepts, from the capability scanner
    test_798 holds exact against the parsers."""
    sys.path.insert(0, str(REPO))
    import krt_capabilities as caps
    return caps.script_flags(caps._tool_path(caps.ROOT, tool))


def check_flag_coverage(tool):
    """Return (bad, rows): [(flag, why)] and {flag: (disposition, detail)}.

    The probe converts the tool's FLAG_COVERAGE probe plus `<flag>
    PROBE_VALUE` and diffs it against the probe alone, so whatever the flag
    contributes -- params, step keys, a refusal, or nothing -- is what gets
    resolved, on the owners the step's ACTION searches. A `--no-X` flag is
    probed a second time BARE, as a manifest records a switch, to check the
    sense it lands with."""
    spec = FLAG_COVERAGE[tool]
    action, probe = spec['action'], spec['probe']
    cli_only, known_gaps = spec['cli_only'], spec['known_gaps']
    reset_gaps = spec['reset_gaps']
    flags = sorted(tool_flags(tool))
    bad, rows = [], {}
    if len(flags) < 10:
        return ([('<%s>' % tool, 'the flag scan returned %d flags -- it read '
                  'nothing, so it would check nothing' % len(flags))], rows)
    aliases, special = _ai_plan_tables()
    owners = _action_owners_table() or {}
    tab_attr, subs = owners.get(action, (None, ()))
    chain = list(subs) + ([tab_attr] if tab_attr else []) + ['<dialog>']
    widgets = _class_widgets()
    reach = [(o, widgets.get(_OWNER_CLASSES.get(o, ''), {})) for o in chain]
    skip, block, special_handled, sel_keys = _apply_facts(action)
    reset = _reset_touched()
    # Flags the converter itself reads a VALUE for (its tables are its arity).
    renamed = m2p.TOOL_FLAG_ALIASES.get(tool, {})
    valued = (set(m2p.FLAG_PARAMS) | set(m2p.LIST_FLAGS)
              | set(m2p.GROUP_LIST_FLAGS)
              | set(m2p.TOOL_FLAG_PARAMS.get(tool, {}))
              | set(m2p.TOOL_OPTIONAL_LIST_FLAGS.get(tool, {}))
              | set(m2p.TOOL_LIST_FLAGS.get(tool, {})))
    base = m2p.parse_command(list(probe))

    def _control(p):
        tgt = aliases.get(p, p)
        for owner, attrs in reach:
            if tgt in attrs:
                return tgt, owner, attrs[tgt]
        return tgt, None, None

    def _delta(step):
        params = step.get('params') or {}
        new = {k: v for k, v in params.items()
               if base['params'].get(k, object()) != v}
        keys = sorted(k for k in set(step) | set(base)
                      if k not in ('params', '_files')
                      and step.get(k) != base.get(k))
        return new, keys

    for listed in (cli_only, known_gaps, reset_gaps):
        for flag, why in listed.items():
            if flag not in flags:
                bad.append((flag, "STALE list entry: %s no longer accepts "
                                  "this flag" % tool))
            if not (isinstance(why, str) and why.strip()):
                bad.append((flag, "list entry carries no reason"))
    for flag in sorted(set(cli_only) & set(known_gaps)):
        bad.append((flag, "on BOTH the CLI-only and the known-gaps list"))

    for flag in flags:
        step = m2p.parse_command(probe + [flag, _PROBE_VALUE])
        missing, got, controls = [], [], set()
        if step is None:
            missing.append('the converter returns no step at all')
        elif '_refused' in step:
            missing.append('the converter refuses the command (%s)'
                           % step['_refused'])
        else:
            new, keys = _delta(step)
            if not new and not keys:
                missing.append('the converter consumes it and emits nothing')
            for p, v in sorted(new.items()):
                if p in skip or p in special:
                    # Mirrors the generic loop's order: skip, then special.
                    where, names = (('the %s action block' % action, block)
                                    if p in skip else
                                    ('_apply_special', special_handled))
                    if p in names:
                        got.append(f'{p} ({where})')
                    else:
                        missing.append(f'{p} is left to {where}, which never '
                                       f'names it')
                    continue
                tgt, owner, kind = _control(p)
                if owner is None:
                    missing.append('param %r -> %r, which is no settable '
                                   'control on the %s owners %s'
                                   % (p, tgt, action, chain))
                    continue
                if isinstance(v, bool) and kind != 'CheckBox':
                    missing.append('switch param %r lands on the %s %r'
                                   % (p, kind, tgt))
                    continue
                if renamed.get(flag, flag) in valued and kind == 'CheckBox':
                    missing.append('the converter reads a VALUE for it, but '
                                   '%r lands on the CheckBox %r, which holds '
                                   'only on/off' % (p, tgt))
                    continue
                got.append('%s -> %s.%s' % (p, owner, tgt))
                controls.add(tgt)
            for k in keys:
                if k in sel_keys:
                    got.append(f'step[{k!r}]')
                else:
                    missing.append(f'step[{k!r}], which the {action} '
                                   f'selection never reads')
            if flag.startswith('--no-'):
                # SENSE, from the BARE switch a manifest actually records.
                bare = m2p.parse_command(probe[:4] + [flag] + probe[4:])
                for p, v in sorted(_delta(bare)[0].items()):
                    tgt, owner, kind = _control(p)
                    want = tgt.startswith('no_')
                    if (owner and kind == 'CheckBox' and isinstance(v, bool)
                            and v != want):
                        missing.append(
                            'it sets %r to %r: a --no-X switch landing on the '
                            'POSITIVE checkbox %r must untick it' % (p, v, tgt))
        reached = bool(got) and not missing
        detail = '; '.join(got if reached else missing + got)
        if flag in cli_only:
            rows[flag] = ('cli-only', cli_only[flag])
        elif flag in known_gaps:
            rows[flag] = ('known-gap', known_gaps[flag])
        else:
            rows[flag] = ('reached' if reached else 'NOT REACHED', detail)
        if reached and (flag in cli_only or flag in known_gaps):
            bad.append((flag, "STALE list entry: it now reaches the GUI (%s); "
                              "take it off %s's %s list" % (
                                  detail, tool, 'CLI-only' if flag in cli_only
                                  else 'known-gaps')))
        elif not reached and flag not in cli_only and flag not in known_gaps:
            bad.append((flag, "NOT REACHED -- %s. Map it onto its control (a "
                              "manifest_to_plan row, or an ai_plan alias), or "
                              "list it for %s as CLI-only with the reason, or "
                              "as a known gap" % (detail, tool)))
        unreset = sorted(c for c in controls if c not in reset) \
            if reached else []
        if unreset and flag not in reset_gaps:
            bad.append((flag, "LEAKS between plan steps: %s %s not restored "
                              "by reset_params_to_defaults" % (
                                  ', '.join(unreset),
                                  'is' if len(unreset) == 1 else 'are')))
        elif not unreset and flag in reset_gaps:
            bad.append((flag, "STALE reset-gap entry: its control is reset "
                              "now (or it no longer reaches one)"))
    return bad, rows


def check_param_resolution():
    """Return list of (param, reason) for MUST-resolve params that don't."""
    aliases, special = _ai_plan_tables()
    controls = _gui_control_attrs()
    bad = []
    for p in sorted(_MUST_RESOLVE):
        if p in special or p in _ACTION_BLOCK_HANDLED:
            continue
        if p in controls:
            continue
        tgt = aliases.get(p)
        if tgt is not None and tgt in controls:
            continue
        if tgt is not None:
            bad.append((p, f"alias -> {tgt!r}, but no such GUI control"))
        else:
            bad.append((p, "no control, no alias, not special -> would be ignored"))
    return bad


def check_cap_flags():
    """#733/#772: the optimize_caps step's flags survive conversion, and
    land on the CAP panel's names rather than the Basic tab's twins.

    The pairs loop above deliberately `continue`s on place_fanout_clearance.py
    ("optimize_caps: no routing flags to assert"), so nothing there looks at
    CAP_FLAG_PARAMS at all. That was harmless while every row named a
    bga_options control the plan executor ignores anyway; it stopped being
    harmless when --board-edge-clearance joined the table, because that one
    names a REAL dialog control and changes where cap copper may sit relative
    to Edge.Cuts. Measured: deleting its row left the whole gate green.

    Self-contained, like check_group_flags: no corpus needed.
    """
    bad = []
    argv = ['python3', 'py_placer/place_fanout_clearance.py', 'in.kicad_pcb',
            'out.kicad_pcb', '--board-edge-clearance', '0.85',
            '--capture-radius', '5', '--near-margin', '1.5',
            '--step', '0.35', '--max-displacement', '4',
            '--max-displacement-cap', '6', '--displacement-growth', '2',
            '--max-passes', '7', '--cap-prefix', 'C',
            '--grid-step', '0.05', '--clearance', '0.1',
            '--default-via-size', '0.42', '--no-rotate',
            '--intent', 'x.intent.json']
    step = m2p.cap_optimization_step(argv)
    if step.get('action') != 'optimize_caps':
        return [('(step)', f"action is {step.get('action')!r}")]
    params = step.get('params') or {}
    # #772: EVERY flag, not three of twelve. The three-row version could not
    # have caught a dropped --max-passes or --cap-prefix, and the executor
    # was dropping all ten cap knobs at the time it was written.
    for flag, key, want in (
            ('--board-edge-clearance', 'cap_board_edge_clearance', 0.85),
            ('--capture-radius', 'cap_capture_radius', 5),
            ('--near-margin', 'cap_near_margin', 1.5),
            ('--step', 'cap_step', 0.35),
            ('--max-displacement', 'cap_max_displacement', 4),
            ('--max-displacement-cap', 'cap_max_displacement_cap', 6),
            ('--displacement-growth', 'cap_displacement_growth', 2),
            ('--max-passes', 'cap_max_passes', 7),
            ('--cap-prefix', 'cap_prefix', 'C'),
            ('--grid-step', 'grid_step', 0.05),
            ('--clearance', 'clearance', 0.1),
            ('--default-via-size', 'cap_default_via_size', 0.42),
            ('--intent', 'cap_intent_path', 'x.intent.json'),
            ('--no-rotate', 'cap_allow_rotation', False)):
        if params.get(key) != want:
            bad.append((flag, f"-> {key}={params.get(key)!r}, expected {want!r}"))
    # #742 CHANGE DETECTOR, same shape as the --board-edge-clearance one below.
    # `via_size` is the Basic tab's via GEOMETRY: it sets the diameter of the
    # vias fanout places, ai_plan ticks via_size_check for it, and
    # PlanExecutor._write_drc_floors writes it into the project. A converter
    # that landed --default-via-size there would change copper and leave a via
    # floor behind, so asserting only the new name is not enough.
    if 'via_size' in params:
        bad.append(('--default-via-size',
                    "converted to the Basic tab's via GEOMETRY name "
                    "'via_size'; on a cap step the flag is the unreadable-via "
                    'FALLBACK (cap_default_via_size)'))
    # #772 CHANGE DETECTOR: the Basic-tab SIGNAL name must not appear on a cap
    # step. It is a different quantity, and ai_plan ticks edge_clearance_check
    # for it -- which the cap step then leaks into the next step's routing.
    # Asserting only the NEW name is not enough: a converter emitting BOTH
    # would look green here and still tick the wrong box.
    if 'board_edge_clearance' in params:
        bad.append(('--board-edge-clearance',
                    "converted to the Basic tab's SIGNAL name "
                    "'board_edge_clearance'; on a cap step the flag is the "
                    'PLACEMENT margin (cap_board_edge_clearance)'))
    # NEGATIVE CONTROL: an omitted flag must NOT be invented. An engine-resolved
    # default that leaked into the plan would pin the margin at plan time and
    # defeat the point of resolving it per board.
    bare = m2p.cap_optimization_step(
        ['python3', 'py_placer/place_fanout_clearance.py', 'in.kicad_pcb'])
    for _k in ('cap_board_edge_clearance', 'board_edge_clearance',
               'cap_default_via_size', 'via_size', 'cap_intent_path'):
        if _k in (bare.get('params') or {}):
            bad.append(('(omitted)',
                        f'an unset flag was materialised into the plan as {_k}'))
    return bad


def check_group_flags():
    """#459: --group must SURVIVE, and --undo/--preview must be REFUSED.

    Self-contained (no corpus needed) because the failure is silent and severe:
    all three flags used to fall through to the generic `params` bucket, where
    ai_plan drops any param with no matching dialog control -- and `nets` then
    defaults to ['*']. So a recorded `--undo BLOCK` converted to a plan step that
    ROUTES the whole board: the exact inverse of the recorded command, on 100x
    the scope, with nothing printed.
    """
    bad = []

    argv = ['python3', 'route.py', 'in.kicad_pcb', 'out.kicad_pcb',
            '--group', 'sheet:abcd1234', '--group-scope', 'internal',
            '--group-by', 'sheet', '--clearance', '0.09']
    step = m2p.parse_command(argv)
    if not step:
        bad.append(('--group', 'produced no step at all'))
    else:
        for flag, key, want in (('--group', 'group', 'sheet:abcd1234'),
                                ('--group-scope', 'group_scope', 'internal'),
                                ('--group-by', 'group_by', 'sheet')):
            got = step.get(key)
            if got != want:
                bad.append((flag, f"step[{key!r}] want {want!r} got {got!r} "
                                  f"(in params instead? "
                                  f"{step.get('params', {}).get(key)!r})"))
        if step.get('params', {}).get('group'):
            bad.append(('--group', "landed in params, where ai_plan drops it"))

    for flag in ('--undo', '--preview', '--list-groups'):
        step = m2p.parse_command(['python3', 'route.py', 'in.kicad_pcb',
                                  'out.kicad_pcb', flag, '--group', 'decap:U3'])
        if step is None or '_refused' not in (step or {}):
            bad.append((flag, f"NOT refused -- converted to {step!r}. A replayed "
                              f"{flag} step would route these nets instead."))
    return bad


def check_refused_tools():
    """#431: placement tools must be REFUSED loudly, and must not break the chain.

    They mutate the board, so the recorded command has to stay in the manifest
    to keep `compute_prune_keep` linking board -> board_placed. Dropping it
    silently (the unknown-tool path, which only bumps a `skipped` counter)
    leaves the next route step's input produced by nothing and the pruner then
    discards legitimate upstream steps.
    """
    bad = []
    # EVERY registered refusal is exercised, not a hand-picked four: the list
    # here used to name four of the seven entries, so a new placement CLI could
    # join REFUSED_TOOLS and never be checked (#892 added the eighth). The
    # membership assertion below keeps the deletion half of the gate that the
    # hardcoded tuple provided.
    for tool in sorted(m2p.REFUSED_TOOLS):
        step = m2p.parse_command(['python3', tool, 'a.kicad_pcb', 'b.kicad_pcb'])
        if not step or '_refused' not in step:
            bad.append((tool, f"NOT refused -- converted to {step!r}"))
    for tool in ('place_optimize.py', 'place_route_loop.py',
                 'place_seed.py', 'place_reconstruct.py', 'place_portfolio.py',
                 'place_pose.py', 'render_placement.py', 'beautify_labels.py',
                 'add_rule_area.py'):
        if tool not in m2p.REFUSED_TOOLS:
            bad.append((tool, "dropped from REFUSED_TOOLS -- a board tool with "
                              "no plan step that converts silently breaks the "
                              "replay chain"))

    # Chain integrity: a placement step between a fanout and a route must not
    # take either of them with it.
    import tempfile
    d = tempfile.mkdtemp()
    man = os.path.join(d, 'redo_commands.sh')
    with open(man, 'w', encoding='utf-8', newline='\n') as f:
        f.write("#!/bin/sh\n"
                "python3 bga_fanout.py b.kicad_pcb -o s1.kicad_pcb "
                "--component U1 --clearance 0.1\n"
                "python3 py_placer/place_optimize.py s1.kicad_pcb s2.kicad_pcb "
                "--max-displacement 3\n"
                "python3 route.py s2.kicad_pcb s3.kicad_pcb --nets '*' "
                "--clearance 0.1\n")
    try:
        steps, _skipped = m2p.plan_steps_from_manifest(man)
        actions = [s.get('action') for s in steps]
        if actions != ['fanout', 'route']:
            bad.append(('<chain>', f"expected ['fanout','route'], got {actions} "
                                   f"-- a refused placement step broke the chain"))
    except Exception as e:
        bad.append(('<chain>', f"{type(e).__name__}: {e}"))
    finally:
        import shutil
        shutil.rmtree(d, ignore_errors=True)
    return bad


def main():
    # Corpus manifests give broad coverage; the checked-in fixture makes the gate
    # self-contained (runs on a fresh checkout with no corpus) AND is always
    # included: it is the only manifest guaranteed to exercise every asserted
    # flag (--fab-overrides appears in no corpus manifest, so corpus-only runs
    # could never detect its loss). Explicit args win.
    manifests = sys.argv[1:] or (sorted(
        glob.glob(str(STRESS / "runs_set*/*/redo_commands.sh"))) + [FIXTURE])
    if not manifests:
        print("no manifests found (set $STRESS_DIR or pass paths)")
        return 1
    total, total_bad, bad_boards = 0, 0, []
    for man in manifests:
        board = Path(man).parent.name
        try:
            pairs = _plan_pairs(man)
        except Exception as e:
            bad_boards.append((board, [("<convert>", f"{type(e).__name__}: {e}")]))
            total_bad += 1
            continue
        board_bad = []
        for argv, step in pairs:
            n, bad = check_pair(argv, step)
            total += n
            for flag, why in bad:
                board_bad.append((step.get('action'), flag, why))
                total_bad += 1
        if board_bad:
            bad_boards.append((board, board_bad))
    print(f"\nConverter parity: {total} flag-checks across {len(manifests)} "
          f"manifest(s), {total_bad} mismatch(es).")
    for board, probs in bad_boards:
        print(f"\n  {board}:")
        for p in probs:
            if len(p) == 2:
                print(f"    {p[0]}: {p[1]}")
            else:
                print(f"    [{p[0]}] {p[1]}: {p[2]}")

    # #381 D5: param -> control resolution gate.
    res_bad = check_param_resolution()
    print(f"\nParam->control resolution: {len(_MUST_RESOLVE)} params checked, "
          f"{len(res_bad)} unresolved.")
    for p, why in res_bad:
        print(f"    {p}: {why}")

    # #733 optimize_caps flags (self-contained, no corpus needed).
    cap_bad = check_cap_flags()
    print(f"\nCap-optimization flags: {'OK' if not cap_bad else 'FAILED'} "
          f"(--board-edge-clearance and the cap_* knobs survive).")
    for f, why in cap_bad:
        print(f"    {f}: {why}")

    # #772: OWNER-scoped resolution. check_param_resolution above only asks
    # whether a control with the name exists SOMEWHERE; this asks whether
    # the action that carries the param can actually REACH it.
    own_bad = check_owner_scoping()
    print(f"\nOwner-scoped resolution: "
          f"{'OK' if not own_bad else 'FAILED'} "
          f"(each param resolves on the owners its ACTION searches).")
    for p, why in own_bad:
        print(f"    {p}: {why}")

    # EVERY flag of each FLAG_COVERAGE tool, not the ones a manifest happened
    # to use (self-contained, no corpus needed).
    flag_bad = []
    for tool in FLAG_COVERAGE:
        bad_t, rows = check_flag_coverage(tool)
        tally = {}
        for disp, _detail in rows.values():
            tally[disp] = tally.get(disp, 0) + 1
        print(f"\nFlag coverage, {tool}: {'OK' if not bad_t else 'FAILED'} "
              f"({len(rows)} flags: "
              + ', '.join(f"{n} {d}" for d, n in sorted(tally.items()))
              + ").")
        for f, why in bad_t:
            print(f"    {f}: {why}")
        flag_bad += bad_t

    # #459 placement-block flags (self-contained, no corpus needed).
    grp_bad = check_group_flags()
    print(f"\nPlacement-block flags: {'OK' if not grp_bad else 'FAILED'} "
          f"(--group survives, --undo/--preview refused).")
    for f, why in grp_bad:
        print(f"    {f}: {why}")

    ref_bad = check_refused_tools()
    print(f"Placement tools: {'OK' if not ref_bad else 'FAILED'} "
          f"(refused loudly, chain intact).")
    for f, why in ref_bad:
        print(f"    {f}: {why}")

    return 1 if (total_bad or res_bad or cap_bad or own_bad or flag_bad
                 or grp_bad or ref_bad) else 0


if __name__ == "__main__":
    sys.exit(main())
