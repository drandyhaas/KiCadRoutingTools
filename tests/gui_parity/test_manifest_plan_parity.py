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
    bad = []
    n = 0
    i = 0
    while i < len(argv):
        a = argv[i]
        if a in SCALAR_FLAGS and i + 1 < len(argv):
            want = _num(argv[i + 1])
            got = params.get(SCALAR_FLAGS[a])
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
    return 1 if total_bad else 0


if __name__ == "__main__":
    sys.exit(main())
