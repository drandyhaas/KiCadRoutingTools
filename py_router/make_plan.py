#!/usr/bin/env python3
"""Make a GUI routing plan (plan.json) from a recorded command chain (#507).

A plan JSON is what the GUI's AI tab loads (Load... next to "Parsed result")
and what ``run_plan.py`` executes headless. Every stress run writes one; this
makes the same file on demand from any recorded manifest:

    python3 py_router/make_plan.py runs_set1/myboard                 # a run directory
    python3 py_router/make_plan.py runs_set1/myboard/redo_commands.sh -o plan.json
    python3 py_router/make_plan.py runs_set1/myboard --list          # just show the steps

**Recording your own chain.** The board-mutating tools self-record whenever
``REDO_MANIFEST`` points somewhere, so a hand-run chain becomes a plan too:

    export REDO_MANIFEST=$PWD/redo_commands.sh
    python3 py_router/bga_fanout.py board.kicad_pcb s1.kicad_pcb --component U1
    python3 py_router/route.py s1.kicad_pcb s2.kicad_pcb --nets '/SPI*'
    python3 py_router/make_plan.py .            # -> plan.json, ready to Load in the GUI

The conversion prunes superseded retries and dead-end branches (the same file
dependency chain a replay runs), drops check/grade commands, and carries every
recognized ``--flag`` into the step's params so the GUI routes the way the
recorded commands did.

Conversion lives in ``tests/stress/manifest_to_plan.py``; this is its front door
from outside the stress harness. (That directory is not shipped in the PCM
plugin package, so this script needs a git checkout.)
"""
from __future__ import annotations

#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['routing'], 'kind': 'instrument'}

import argparse
import json
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
STRESS = os.path.join(ROOT, 'tests', 'stress')
for _p in (ROOT, STRESS):
    if _p not in sys.path:
        sys.path.insert(0, _p)

MANIFEST_NAME = 'redo_commands.sh'


def resolve_manifest(path):
    """The manifest for a manifest path or a run/output directory."""
    path = os.path.abspath(path)
    if os.path.isdir(path):
        cand = os.path.join(path, MANIFEST_NAME)
        if not os.path.exists(cand):
            raise FileNotFoundError(
                f"no {MANIFEST_NAME} in {path} -- point at a recorded run "
                f"directory, or record one with REDO_MANIFEST=<file>")
        return cand
    if not os.path.exists(path):
        raise FileNotFoundError(path)
    return path


def default_output(manifest):
    """Beside the manifest, named after its directory -- the same
    ``<board>_plan.json`` a stress run leaves."""
    d = os.path.dirname(manifest)
    return os.path.join(d, f"{os.path.basename(d) or 'routing'}_plan.json")


def make_plan(source, out=None):
    """Write the plan JSON for a manifest/run dir. Returns (path, steps, skipped)."""
    try:
        from manifest_to_plan import plan_steps_from_manifest
    except ImportError as e:
        raise ImportError(
            f"the manifest converter (tests/stress/manifest_to_plan.py) is not "
            f"available: {e}. Run make_plan.py from a git checkout.") from e
    manifest = resolve_manifest(source)
    steps, skipped = plan_steps_from_manifest(manifest)
    out = os.path.abspath(out or default_output(manifest))
    os.makedirs(os.path.dirname(out) or '.', exist_ok=True)
    with open(out, 'w', encoding='utf-8') as f:
        json.dump({'steps': steps}, f, indent=2)
    return out, steps, skipped


def plan_commands(source):
    """[(cwd, argv)] for a manifest or run dir -- EVERY recorded command, in
    file order, with nothing pruned.

    `make_plan` above converts to GUI steps, and that conversion deliberately
    DROPS what the GUI cannot execute: every `check_*` command, `cd`, `cp`,
    superseded retries and dead-end branches, and any tool with no plan
    action. It also loses the tool NAME (`route_planes` and `repair_planes`
    both become `repair_planes`), the argv, and any index back into the file.

    A checker of the plan needs precisely what that drops -- whether the chain
    ends on route.py, whether a `cp` carried its `.kicad_pro`, whether a pipe
    got in -- so it gets the raw list, and `plan_steps_from_manifest` stays
    the one conversion.

    Note what even this cannot see: `parse_manifest` drops COMMENTS, and the
    routing skill requires verification commands to BE comments. A rule about
    those has to read the file text.
    """
    from redo_stress_test import parse_manifest
    return parse_manifest(resolve_manifest(source))


def describe(step, index):
    """One human-readable line for a step (no wx needed, unlike the GUI's
    step_label, so --list works under any python)."""
    action = step.get('action', '?')
    detail = ''
    if step.get('component'):
        detail = f" {step['component']} ({step.get('kind', 'bga')})"
    elif step.get('pairs'):
        detail = f" {len(step['pairs'])} pair(s)"
    elif step.get('assignments'):
        detail = " " + ", ".join(
            f"{a.get('layer') or '?'}:{len(a.get('nets') or [])} net(s)"
            for a in step['assignments'])
    elif step.get('nets'):
        nets = step['nets']
        shown = ", ".join(nets[:3]) + (", ..." if len(nets) > 3 else "")
        detail = f" {len(nets)} net(s): {shown}"
    params = step.get('params') or {}
    return (f"{index:2d}. {action}{detail}"
            + (f"   [{len(params)} param(s)]" if params else ""))


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('source',
                    help=f'a run/output directory holding {MANIFEST_NAME}, or the '
                         f'manifest file itself')
    ap.add_argument('-o', '--output', default=None,
                    help='plan JSON to write (default: <dir>/<dirname>_plan.json)')
    ap.add_argument('--list', action='store_true',
                    help='also print the steps')
    args = ap.parse_args()

    try:
        out, steps, skipped = make_plan(args.source, args.output)
    except (FileNotFoundError, ImportError) as e:
        print(f"make_plan: {e}", file=sys.stderr)
        return 1
    if not steps:
        print(f"make_plan: no GUI-representable routing commands found in "
              f"{args.source}", file=sys.stderr)
        return 1

    print(f"{len(steps)} step(s) written to {out}"
          + (f" ({skipped} kept-but-non-GUI command(s) skipped)" if skipped else ""))
    if args.list:
        for i, step in enumerate(steps, 1):
            print("  " + describe(step, i))
    rel = os.path.relpath(out)
    shown = out if rel.startswith('..') else rel
    print("Load it in the GUI (AI tab -> Load... -> Run Selected Steps), "
          "or run it headless:")
    print(f"  python3 py_router/run_plan.py <board>.kicad_pcb {shown} -o routed.kicad_pcb")
    return 0


if __name__ == '__main__':
    sys.exit(main())
