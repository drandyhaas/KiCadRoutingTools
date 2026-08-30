#!/usr/bin/env python3
"""Class-2 coverage lint: every CLI main() post-engine finalization pass must
have a GUI counterpart.

CLI fronts and the GUI tabs call the SAME shared engine functions (batch_route,
create_plane, generate_bga_fanout, ...), so a fix INSIDE those is inherited by
both for free (Class 1). The drift happens when a CLI `main()` runs an extra
pass AFTER its engine call -- clean_plane_copper, oracle_reconnect,
fix_project_for_output, ... -- that the GUI must separately replicate (Class 2).
That is exactly how the set11 GUI board shipped 35 plane shorts the CLI board
didn't have: repair_planes.main() ran clean_plane_copper on its
output file and the planes tab never did.

This lint has no gate-able runtime; it is a STATIC guard:
  A. Every known post-pass called in a CLI main() is registered here.
  B. Every registered post-pass in active CLI use has >=1 GUI counterpart symbol
     present in kicad_routing_plugin/ (the actual regression check -- catches a
     renamed/removed GUI counterpart).
  C. DISCOVERY: any symbol from a pure-finalization module (kicad_oracle,
     fix_kicad_drc_settings) called in a CLI main() but NOT registered fails,
     so a NEW CLI-only post-pass can't be added without wiring the GUI.

Pure Python, no wx / no pcbnew.  Run: python3 tests/gui_parity/test_cli_postpass_coverage.py
"""
import ast
import glob
import os
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
PLUGIN = REPO / "kicad_routing_plugin"

# CLI front mains to audit. route.py/route_diff/*_fanout put their finalization
# INSIDE the shared engine (Class 1) -- included so the discovery pass still
# guards them if that ever changes.
# #381 D8: bga_fanout.py / qfn_fanout.py are THIN WRAPPERS ('from pkg import
# main') with no main() def, so scanning them saw nothing -- the fanout
# post-passes (run_drc graze audit, fix_project_for_output) were a blind spot.
# Point at the package __init__ where main() actually lives.
# #522 layout: the CLI mains live under py_router/.
# #725: place_fanout_clearance.py was absent, so this gate never scanned the
# decoupling-cap repair CLI even though its main() runs two tracked post-passes
# (warn_if_missing_project_floor, fix_project_for_output). That matters
# specifically because the cap-repair engine now reads the sibling .kicad_pro
# and .kicad_dru for its own arithmetic: dropping the project on the way out
# does not just cost the next step its DRC floor, it silently changes what the
# NEXT cap-repair run prices at.
CLI_MAINS = ["py_router/route.py", "py_router/route_diff.py",
             "py_router/route_planes.py", "py_router/repair_planes.py",
             "py_router/bga_fanout/__init__.py",
             "py_router/qfn_fanout/__init__.py",
             "py_placer/place_fanout_clearance.py"]

# Known post-engine passes -> GUI counterpart symbol(s). A pass is "covered" if
# ANY listed symbol appears anywhere under kicad_routing_plugin/. Keep the RHS
# to symbols that genuinely evidence the GUI running the same finalization.
REGISTRY = {
    # plane dead-end / graze cleanup (this session's fix)
    'clean_plane_copper': ['_run_plane_copper_cleanup', 'compute_plane_copper_cleanup'],
    # KiCad-oracle recheck after plane repair AND route.py's plane-finalize
    # oracle leg (#562). IPC: the staged-save core is the adapter's
    # apply_oracle_reconnect (it snapshot-saves the live board, routes the
    # links kicad-cli reports missing, and applies them in one commit); the
    # signal tab consumes results_data['plane_finalize_oracle'] after apply.
    # The planes tab has no oracle hook since #562 -- after plane CREATION the
    # remaining gaps belong to the route step's finalize.
    'oracle_reconnect': ['apply_oracle_reconnect', 'oracle_reconnect',
                         'plane_finalize_oracle'],
    # Plane finalize (#562): repair_planes runs INSIDE batch_route (Class 1)
    # for both fronts -- under return_results it merges its board delta into
    # results_data and posts the oracle spec; the plugin-side evidence is
    # swig_gui's plane_finalize_oracle consumption.
    'repair_planes': ['plane_finalize_oracle'],
    # DRC-floor / project-file writeback after routing
    'fix_project_for_output': ['_write_drc_floors', 'update_live_drc_floors'],
    # #521: protected-nets record in the sibling .kicad_pro after routing.
    # Engines NOTE candidates (matched groups, routed pairs); each front's
    # writeback PERSISTS them: CLI mains here, GUI at plan end
    # (ai_plan._write_drc_floors) + per manual step (update_live_drc_floors).
    'persist_protected_nets': ['_write_drc_floors', 'update_live_drc_floors'],
    # #521 companion: per-net impedance declarations recorded the same way.
    'persist_impedance_specs': ['_write_drc_floors', 'update_live_drc_floors'],
    # GND return vias near signal vias
    'add_gnd_vias_to_existing_board': ['add_gnd_vias_to_existing_board'],
    # run-6 fix 1.7: castellated-landing retract (route.py + both plane
    # mains); GUI twin applies the shared compute core to the live board.
    'retract_castellated_landings': ['apply_castellated_landing_retract',
                                     'compute_castellated_landing_retract'],
}
# NOTE (deliberately NOT registered): move_copper_graphics_to_silkscreen runs
# inside the shared plane WRITER (plane_io), not a main() -- so this gate, which
# discovers CLI main() post-passes, does not see it. It is NOT "inherited" by the
# GUI though: the GUI applies copper to the live board and never calls the text
# writers, so parity there comes from a HAND-WRITTEN twin -- on IPC that is the
# adapter's move_copper_graphics_to_silkscreen (kipy), called from the route and
# planes apply paths. Any new writer-level pass needs the same treatment.
#
# OPEN GAP (2026-07-28): kicad_writer.strip_zero_length_edge_cuts sits in exactly
# that position -- wired into output_writer / plane_io / kicad_writer beside the
# silkscreen movers -- and has NO GUI twin yet. Consequence: a board routed
# through the CLI comes back with its null-length Edge.Cuts primitives dropped
# (KiCad grades each as invalid_outline and slows to a crawl: 656s -> 2.7s on
# pinci), while the same board routed through the GUI keeps them, because the
# GUI mutates the user's open document rather than authoring a file. The twin
# would be gui_utils.strip_zero_length_edge_cuts_board(board), called where
# move_copper_graphics_to_silkscreen_board already is (swig_gui _apply path and
# planes_gui). Not shipped blind: removing footprint-embedded Edge.Cuts items
# through the SWIG object API killed the interpreter in every headless attempt
# here, so it needs writing and verifying against a real running KiCad.
#
# fix_kicad_drc_settings is a MODULE; the main() call is
# fix_project_for_output (above).

# Modules whose public functions are ALL post-route finalization passes: a
# never-before-seen symbol from these, called in a CLI main, must be reviewed.
# #381 D8: added 'check_drc' -- the fanout mains run a post-engine DRC graze
# audit (check_drc.run_drc) that the lint's module list couldn't see.
POSTPASS_MODULES = ['kicad_oracle', 'fix_kicad_drc_settings', 'check_drc']

# Symbols intentionally exempt from discovery (helpers/args, not passes).
# read_project_edge_clearance (#338): a pure READER (project edge rule ->
# float) feeding the fix_project_for_output edge argument and the oracle
# config; the GUI reads the same rule live from pcbnew
# (m_CopperEdgeClearance in gui_utils/planes_gui), so parity holds at the
# VALUE level, not the call level.
DISCOVERY_EXEMPT = {'add_drc_fix_args', 'drc_fix_kwargs', 'find_kicad_cli',
                    'compute_targets', 'severity_plan',
                    'read_project_edge_clearance',
                    # the `if __name__ == "__main__"` dispatcher calling main()
                    # (that block is scanned as a main scope since #521).
                    'main',
                    # #441: pre-engine helper that WARNS when an input board has no
                    # sibling .kicad_pro (dropped DRC floor). Report-only, runs before
                    # routing, no board mutation; the GUI operates on the live board and
                    # never cp's, so no GUI counterpart is needed.
                    'warn_if_missing_project_floor',
                    # #441: pure RESOLVER (max(cli, project edge rule, fab floor) ->
                    # float) feeding the engine's board_edge_clearance; same VALUE-level
                    # parity story as read_project_edge_clearance above -- the GUI
                    # planes tab reads the live board rule via gui_utils.
                    'effective_board_edge_clearance'}

# #381 D8: post-engine passes that are CLI-only BY DESIGN -- report/audit ONLY,
# they do NOT mutate the output board, so the GUI producing an identical board
# without them is correct (they feed JSON_SUMMARY for the LLM planner's fanout
# retry loop). Tracked here so the lint SEES them (closing the check_drc blind
# spot) without demanding a GUI counterpart. A board-MUTATING pass must go in
# REGISTRY (with a real GUI counterpart), never here.
KNOWN_CLI_ONLY = {
    'run_drc': 'bga/qfn fanout post-engine DRC graze audit -> JSON_SUMMARY '
               'drc_grazes (report-only, no board mutation)',
}


def _main_calls(path):
    """Set of called function names inside the file's main() (AST). Resolves
    `from mod import name as alias` (incl. LOCAL imports inside main) so a call
    through the alias is recorded under the REAL name -- the fanout mains do
    `from check_drc import run_drc as _run_drc` (#381 D8)."""
    src = Path(path).read_text()
    try:
        tree = ast.parse(src)
    except SyntaxError:
        return set()
    def _is_dunder_main_if(node):
        # `if __name__ == "__main__":` -- route.py / route_diff.py run their
        # whole CLI there instead of a def main(), so post-passes added in
        # that block (#521 persist_protected_nets) must be scanned too.
        if not isinstance(node, ast.If):
            return False
        t = node.test
        return (isinstance(t, ast.Compare) and isinstance(t.left, ast.Name)
                and t.left.id == '__name__')

    calls = set()
    scopes = []
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == 'main':
            scopes.append(node)
    scopes.extend(n for n in tree.body if _is_dunder_main_if(n))
    for scope in scopes:
        aliases = {}  # local alias -> real imported name
        for sub in ast.walk(scope):
            if isinstance(sub, ast.ImportFrom):
                for a in sub.names:
                    if a.asname:
                        aliases[a.asname] = a.name
        for sub in ast.walk(scope):
            if isinstance(sub, ast.Call):
                f = sub.func
                if isinstance(f, ast.Name):
                    calls.add(aliases.get(f.id, f.id))
                elif isinstance(f, ast.Attribute):
                    calls.add(f.attr)
    return calls


def _module_public_funcs(mod_name):
    p = REPO / "py_router" / f"{mod_name}.py"     # #522 layout
    if not p.exists():
        p = REPO / "py_tools" / f"{mod_name}.py"
    if not p.exists():
        return set()
    tree = ast.parse(p.read_text())
    return {n.name for n in tree.body
            if isinstance(n, ast.FunctionDef) and not n.name.startswith('_')}


def _plugin_symbols():
    """All symbol tokens present anywhere under kicad_routing_plugin/."""
    toks = set()
    for f in glob.glob(str(PLUGIN / "*.py")):
        for m in re.finditer(r'[A-Za-z_][A-Za-z0-9_]*', Path(f).read_text()):
            toks.add(m.group(0))
    return toks


_TERMINAL = (ast.Return, ast.Raise, ast.Continue, ast.Break)


def _unreachable_calls():
    """Calls that can NEVER execute because a return/raise/continue/break
    precedes them in the same block.

    Check B only asks whether a GUI counterpart SYMBOL EXISTS anywhere under
    kicad_routing_plugin/ -- a textual presence test. That is satisfied by dead
    code, and it was: differential_gui's update_live_drc_floors call sat AFTER
    `return tracks_added, vias_added, tracks_removed`, so the diff-pair tab was
    the one routing tab that never relaxed the DRC floors. The gate stayed
    green while the GUI carried stock floors through an entire plan and every
    step after the diff-pair one resolved its geometry differently from the CLI
    (eth_tap: GUI 3581 segments vs CLI 3428 from step 11 on).

    Returns [(file, lineno, symbol)].
    """
    out = []
    for f in sorted(glob.glob(str(PLUGIN / "*.py"))):
        try:
            tree = ast.parse(Path(f).read_text())
        except SyntaxError:
            continue
        for node in ast.walk(tree):
            for field in ('body', 'orelse', 'finalbody'):
                stmts = getattr(node, field, None)
                if not isinstance(stmts, list):
                    continue
                dead = False
                for st in stmts:
                    if dead:
                        for sub in ast.walk(st):
                            if isinstance(sub, ast.Call):
                                fn = sub.func
                                name = (getattr(fn, 'id', None)
                                        or getattr(fn, 'attr', None))
                                if name:
                                    out.append((os.path.basename(f),
                                                getattr(sub, 'lineno', 0), name))
                    elif isinstance(st, _TERMINAL):
                        dead = True
    return out


def main():
    plugin_syms = _plugin_symbols()
    postpass_funcs = set()
    for m in POSTPASS_MODULES:
        postpass_funcs |= _module_public_funcs(m)

    used = {}  # symbol -> [cli files that call it in main()]
    for cli in CLI_MAINS:
        p = REPO / cli
        if not p.exists():
            continue
        for sym in _main_calls(p):
            if sym in REGISTRY or sym in postpass_funcs or sym in KNOWN_CLI_ONLY:
                used.setdefault(sym, []).append(cli)

    failures = []
    warnings = []
    acknowledged = []

    # C. Discovery: a finalization-module symbol used in a CLI main but not
    # registered = a new/unreviewed CLI-only post-pass.
    for sym, files in used.items():
        if sym in DISCOVERY_EXEMPT:
            continue
        if sym in KNOWN_CLI_ONLY:
            acknowledged.append(f"{sym} in {files}: {KNOWN_CLI_ONLY[sym]}")
            continue
        if sym not in REGISTRY:
            failures.append(
                f"UNREGISTERED post-pass '{sym}' called in {files} -- add it to "
                f"REGISTRY with its GUI counterpart, KNOWN_CLI_ONLY (report-only, "
                f"no board mutation), or DISCOVERY_EXEMPT (helper, not a pass).")

    # A/B. Every registered pass in active CLI use must have a GUI counterpart.
    for sym, guis in REGISTRY.items():
        cli_files = used.get(sym)
        if not cli_files:
            warnings.append(f"registry entry '{sym}' no longer called in any CLI "
                            f"main() -- stale? remove it or update CLI_MAINS.")
            continue
        if not any(g in plugin_syms for g in guis):
            failures.append(
                f"CLI post-pass '{sym}' (in {cli_files}) has NO GUI counterpart: "
                f"none of {guis} found under kicad_routing_plugin/. The GUI plan "
                f"run will diverge from the CLI chain (Class-2 drift).")

    # D. A registered GUI counterpart must be REACHABLE, not dead code.
    # Check B is satisfied by a symbol appearing anywhere -- including after a
    # return. Flag any unreachable call to a registered counterpart, and warn
    # on unreachable calls generally (they are almost always a mistake).
    counterparts = {g for guis in REGISTRY.values() for g in guis}
    dead = _unreachable_calls()
    for fname, lineno, sym in dead:
        if sym in counterparts:
            failures.append(
                f"GUI counterpart '{sym}' is UNREACHABLE at {fname}:{lineno} "
                f"(dead code after a return/raise). It satisfies the "
                f"presence check in B but never runs, so the GUI silently "
                f"skips a CLI post-pass.")
        else:
            warnings.append(f"unreachable call to '{sym}' at {fname}:{lineno}")

    print(f"CLI post-pass coverage: {len(REGISTRY)} registered, "
          f"{len(used)} in active CLI use, {len(acknowledged)} CLI-only "
          f"acknowledged, {len(dead)} unreachable call(s), "
          f"{len(failures)} failure(s), {len(warnings)} warning(s).")
    for a in acknowledged:
        print(f"  CLI-ONLY: {a}")
    for w in warnings:
        print(f"  WARN: {w}")
    for f in failures:
        print(f"  FAIL: {f}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
