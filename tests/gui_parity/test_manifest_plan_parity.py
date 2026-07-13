#!/usr/bin/env python3
"""Headless CONVERTER-parity gate: manifest_to_plan must preserve every
routing-affecting CLI flag into the GUI plan step it emits.

A recorded stress `redo_commands.sh` is the source of truth for the CLI
routing chain. `tests/stress/manifest_to_plan.py` turns each kept command into
the plan step the Claude tab loads. If a flag is dropped or renamed in that
translation, the GUI "replay" silently diverges from the CLI board it claims to
reproduce -- exactly how set11 rp2350_fpga_eensy came out with 242 DRC
violations vs the CLI's 0 (issue #361).

Needs NEITHER wx NOR pcbnew. It reuses the converter's OWN pruning to pair each
kept command 1:1 with its plan step (so there is no fragile positional
matching), then asserts each flag with an INDEPENDENT expectation table --
a converter that drops --no-bga-zones fails even though it "agrees with
itself". This is the converter half of GUI/CLI parity; the apply half
(claude_plan.apply_step_params control mapping) is covered by
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
}
BOOL_FLAGS = {
    '--no-bga-zones': 'no_bga_zone', '--no-bga-zone': 'no_bga_zone',
    '--no-gnd-vias': 'no_gnd_vias', '--rip-blocker-nets': 'rip_blocker_nets',
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


def check_pair(argv, step):
    """Return list of (flag, reason) mismatches for one command/step pair."""
    params = step.get('params', {})
    scalar = dict(SCALAR_FLAGS)
    scalar.update(ACTION_SCALAR_OVERRIDES.get(step.get('action'), {}))
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
        i += 1
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
# claude_plan.py imports wx at module level, so we can't import it here (no-wx
# gate). Extract its resolution tables and the GUI control attribute names by
# AST instead, then assert every param that MUST reach a control actually does
# (via same-name control, alias->control, or a _apply_special handler). This is
# what blocks a new "no control, ignored" fallthrough (the D5 regression class).
import ast  # noqa: E402

# Params claude_plan resolves through action-specific blocks (not the generic
# alias/special path): composites / same-name-but-formatted controls. Kept
# explicit so the gate credits them without re-parsing every action block.
_ACTION_BLOCK_HANDLED = {
    'track_width', 'clearance', 'via_size', 'via_drill',
    'diff_pair_width', 'diff_pair_gap', 'power_nets', 'power_nets_widths',
    'layer_costs', 'add_gnd_vias', 'gnd_via_distance', 'gnd_via_net',
    'max_track_width', 'min_track_width',
}

# Params that MUST resolve to a GUI control (the D5 fallback list + D3 polarity).
_MUST_RESOLVE = {
    'rip_existing_nets', 'impedance', 'ordering', 'direction', 'time_matching',
    'keepout', 'guide_corridor', 'length_match_groups', 'swappable_nets',
    'polarity_swap_nets',
}


def _claude_plan_tables():
    """AST-extract _PARAM_CONTROL_ALIASES (dict) and _PARAM_SPECIAL (set) from
    claude_plan.py without importing it (it imports wx)."""
    src = (REPO / "kicad_routing_plugin" / "claude_plan.py").read_text()
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
    for fn in ("swig_gui.py", "differential_gui.py", "fanout_gui.py",
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


def check_param_resolution():
    """Return list of (param, reason) for MUST-resolve params that don't."""
    aliases, special = _claude_plan_tables()
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


def main():
    # Corpus manifests give broad coverage; the checked-in fixture makes the gate
    # self-contained (runs on a fresh checkout with no corpus). Explicit args win.
    manifests = sys.argv[1:] or sorted(
        glob.glob(str(STRESS / "runs_set*/*/redo_commands.sh"))) or [FIXTURE]
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

    return 1 if (total_bad or res_bad) else 0


if __name__ == "__main__":
    sys.exit(main())
