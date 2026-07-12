#!/usr/bin/env python3
"""Class-2 coverage lint: every CLI main() post-engine finalization pass must
have a GUI counterpart.

CLI fronts and the GUI tabs call the SAME shared engine functions (batch_route,
create_plane, generate_bga_fanout, ...), so a fix INSIDE those is inherited by
both for free (Class 1). The drift happens when a CLI `main()` runs an extra
pass AFTER its engine call -- clean_plane_copper, oracle_reconnect,
fix_project_for_output, ... -- that the GUI must separately replicate (Class 2).
That is exactly how the set11 GUI board shipped 35 plane shorts the CLI board
didn't have: route_disconnected_planes.main() ran clean_plane_copper on its
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
CLI_MAINS = ["route.py", "route_diff.py", "route_planes.py",
             "route_disconnected_planes.py", "bga_fanout.py", "qfn_fanout.py"]

# Known post-engine passes -> GUI counterpart symbol(s). A pass is "covered" if
# ANY listed symbol appears anywhere under kicad_routing_plugin/. Keep the RHS
# to symbols that genuinely evidence the GUI running the same finalization.
REGISTRY = {
    # plane dead-end / graze cleanup (this session's fix)
    'clean_plane_copper': ['_run_plane_copper_cleanup', 'compute_plane_copper_cleanup'],
    # KiCad-oracle recheck after plane repair
    'oracle_reconnect': ['_run_kicad_oracle_after_apply', 'oracle_reconnect'],
    # DRC-floor / project-file writeback after routing
    'fix_project_for_output': ['_write_drc_floors', 'update_live_drc_floors'],
    # GND return vias near signal vias
    'add_gnd_vias_to_existing_board': ['add_gnd_vias_to_existing_board'],
}
# NOTE (deliberately NOT registered): move_copper_graphics_to_silkscreen runs
# inside the shared plane WRITER (plane_io), not a main() -- Class 1, inherited
# by both fronts. fix_kicad_drc_settings is a MODULE; the main() call is
# fix_project_for_output (above).

# Modules whose public functions are ALL post-route finalization passes: a
# never-before-seen symbol from these, called in a CLI main, must be reviewed.
POSTPASS_MODULES = ['kicad_oracle', 'fix_kicad_drc_settings']

# Symbols intentionally exempt from discovery (helpers/args, not passes).
DISCOVERY_EXEMPT = {'add_drc_fix_args', 'drc_fix_kwargs', 'find_kicad_cli',
                    'compute_targets', 'severity_plan'}


def _main_calls(path):
    """Set of called function names inside the file's main() (AST)."""
    src = Path(path).read_text()
    try:
        tree = ast.parse(src)
    except SyntaxError:
        return set()
    calls = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == 'main':
            for sub in ast.walk(node):
                if isinstance(sub, ast.Call):
                    f = sub.func
                    if isinstance(f, ast.Name):
                        calls.add(f.id)
                    elif isinstance(f, ast.Attribute):
                        calls.add(f.attr)
            break
    return calls


def _module_public_funcs(mod_name):
    p = REPO / f"{mod_name}.py"
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
            if sym in REGISTRY or sym in postpass_funcs:
                used.setdefault(sym, []).append(cli)

    failures = []
    warnings = []

    # C. Discovery: a finalization-module symbol used in a CLI main but not
    # registered = a new/unreviewed CLI-only post-pass.
    for sym, files in used.items():
        if sym in DISCOVERY_EXEMPT:
            continue
        if sym not in REGISTRY:
            failures.append(
                f"UNREGISTERED post-pass '{sym}' called in {files} -- add it to "
                f"REGISTRY with its GUI counterpart (or confirm it's engine-internal "
                f"and exempt it).")

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

    print(f"CLI post-pass coverage: {len(REGISTRY)} registered, "
          f"{len(used)} in active CLI use, {len(failures)} failure(s), "
          f"{len(warnings)} warning(s).")
    for w in warnings:
        print(f"  WARN: {w}")
    for f in failures:
        print(f"  FAIL: {f}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
