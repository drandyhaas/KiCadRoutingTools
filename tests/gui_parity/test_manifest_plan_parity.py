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

Run:  python3 tests/gui_parity/test_manifest_plan_parity.py [manifest ...]
      (no args -> every runs_set*/*/redo_commands.sh under $STRESS_DIR)
Exit code 1 on any mismatch.
"""
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
}
BOOL_FLAGS = {
    '--no-bga-zones': 'no_bga_zone', '--no-bga-zone': 'no_bga_zone',
    '--no-gnd-vias': 'no_gnd_vias', '--rip-blocker-nets': 'rip_blocker_nets',
    '--keep-input-copper': 'keep_input_copper',
}
# nargs='+' glob-list flags: every pattern must survive into the plan param
# (as a list, or a single scalar for one pattern). #521 --protect-nets and the
# previously-unasserted --rip-existing-nets.
LIST_FLAGS = {
    '--rip-existing-nets': 'rip_existing_nets',
    '--polarity-swap-nets': 'polarity_swap_nets',
    '--coplanar-nets': 'coplanar_nets',
}
# Per-action overrides of SCALAR_FLAGS. #381 D4: route_diff.py's trace width is
# --track-width, but its GUI home is the diff tab's diff_pair_width control (not
# the Basic-tab track_width), so a diff step must carry it there.
ACTION_SCALAR_OVERRIDES = {
    'route_diff': {'--track-width': 'diff_pair_width'},
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
            steps.append(m2p.cap_optimization_step(argv))
            continue  # optimize_caps: no routing flags to assert
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
    while i < len(argv):
        a = argv[i]
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
            if not set(want).issubset(set(got)):
                bad.append((a, f"want {want} got {got}"))
            continue
        i += 1
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
            '--grid-step', '0.05', '--clearance', '0.1', '--no-rotate']
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
            ('--no-rotate', 'cap_allow_rotation', False)):
        if params.get(key) != want:
            bad.append((flag, f"-> {key}={params.get(key)!r}, expected {want!r}"))
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
    for _k in ('cap_board_edge_clearance', 'board_edge_clearance'):
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
    for tool in ('place_optimize.py', 'place_route_loop.py', 'render_placement.py',
                 'beautify_labels.py'):
        step = m2p.parse_command(['python3', tool, 'a.kicad_pcb', 'b.kicad_pcb'])
        if not step or '_refused' not in step:
            bad.append((tool, f"NOT refused -- converted to {step!r}"))

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

    return 1 if (total_bad or res_bad or cap_bad or own_bad or grp_bad
                 or ref_bad) else 0


if __name__ == "__main__":
    sys.exit(main())
