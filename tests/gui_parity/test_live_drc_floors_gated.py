#!/usr/bin/env python3
"""#693: no DRC-floor writer may run without the 'fix DRC settings' gate.

The bug: the GUI has independent DRC writers, and only some were gated, so
unchecking "Fix DRC settings after routing" suppressed one and left another
rewriting Board Setup anyway. The reporter watched Minimum annular width go
0.15 -> 0.05 with the box unchecked (#693). It was ungated at ALL FIVE call
sites -- every routing tab plus the plan executor -- so this is a drift guard,
not a spot fix: the CLI gates its twin (fix_project_for_output) on
--no-fix-drc-settings, and a new call site must not silently reintroduce it.

THIS BRANCH (ipc-migration) has a different writer set from the SWIG branch,
so the gate checks the writers that exist here rather than the ones that do
not:

  * `update_live_drc_floors` -- the SWIG live design-settings writer -- does
    NOT exist: kipy cannot write live design settings. Checked anyway, so a
    port of it cannot arrive ungated.
  * `gui_utils.apply_drc_settings_fix` is the ONE writer every routing tab
    (signal, differential, planes, fanout) goes through, and it carries the
    gate INTERNALLY. That makes the helper itself the choke point, so the
    check is that the gate is still inside it -- a call site needs no gate of
    its own, and losing the internal one would silently ungate all four tabs
    at once.
  * a DIRECT `fix_project_for_output` call in the plugin bypasses that choke
    point, so each one must carry its own gate. ai_plan.py has the only such
    call: it owns the real dialog, reads the checkbox into a local, and is
    the IPC plan path's only floor writer (the SWIG branch gates its live
    writer instead, which would leave the checkbox inert here).

Pure AST, no wx and no kipy -- runs anywhere in about a second.
"""
import ast
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
PLUGIN = os.path.join(ROOT, "kicad_routing_plugin")

# Writers that must never be called without a gate around the CALL.
GATED_AT_CALL = ("update_live_drc_floors", "fix_project_for_output")
# The shared helper that carries the gate INSIDE it.
CHOKE_POINT = "apply_drc_settings_fix"
# A gate is any enclosing `if` whose test mentions the toggle. ai_plan reads the
# live checkbox into a local first (it owns the real dialog), so accept that too.
GATE_TOKENS = ("fix_drc", "_fixdrc693")


def _call_name(node):
    """'foo' for foo(), 'foo' for mod.foo() -- the callee's bare name."""
    func = node.func
    return getattr(func, "id", None) or getattr(func, "attr", None)


def check_call_sites():
    """Every GATED_AT_CALL call site sits inside a gating `if`."""
    fails, checked = [], 0
    for name in sorted(os.listdir(PLUGIN)):
        if not name.endswith(".py"):
            continue
        path = os.path.join(PLUGIN, name)
        src = open(path, encoding="utf-8").read()
        if not any(f in src for f in GATED_AT_CALL):
            continue
        tree = ast.parse(src)
        for node in ast.walk(tree):
            if not (isinstance(node, ast.Call)
                    and _call_name(node) in GATED_AT_CALL):
                continue
            checked += 1
            line = node.lineno
            tests = [ast.get_source_segment(src, n.test) or ""
                     for n in ast.walk(tree)
                     if isinstance(n, ast.If)
                     and n.lineno <= line <= (n.end_lineno or n.lineno)]
            if not any(tok in t for t in tests for tok in GATE_TOKENS):
                fails.append(
                    f"{name}:{line} calls {_call_name(node)} WITHOUT a "
                    f"'fix DRC settings' gate -- an unchecked box would still "
                    f"rewrite the board's DRC floors (#693)")
    return fails, checked


def check_choke_point():
    """`apply_drc_settings_fix` still carries its own gate.

    Not a call-site check: every tab calls it UNgated by design, so the gate
    living inside it is the whole reason that is safe. If it goes, four tabs
    are ungated at once and no call-site check would notice.
    """
    path = os.path.join(PLUGIN, "gui_utils.py")
    src = open(path, encoding="utf-8").read()
    tree = ast.parse(src)
    for node in ast.walk(tree):
        if not (isinstance(node, ast.FunctionDef) and node.name == CHOKE_POINT):
            continue
        # The `if` TESTS inside the function, not its source text: the
        # docstring names `fix_drc_settings` to explain the gate, so a text
        # scan passes with the gate deleted. Measured -- the first cut of this
        # check did exactly that (mutation: guard removed, gate still PASSED).
        gate_tests = [ast.get_source_segment(src, n.test) or ""
                      for n in ast.walk(node) if isinstance(n, ast.If)]
        if not any(tok in t for t in gate_tests for tok in GATE_TOKENS):
            return [f"gui_utils.py:{node.lineno} {CHOKE_POINT} no longer gates "
                    f"on the 'Fix DRC settings after routing' toggle -- every "
                    f"routing tab writes floors through it, so all of them are "
                    f"ungated (#693)"]
        return []
    return [f"gui_utils.py: {CHOKE_POINT} not found -- has it been renamed? "
            f"Update this gate rather than deleting it."]


def main():
    fails, checked = check_call_sites()
    fails += check_choke_point()

    if not checked:
        print(f"FAIL: found no call sites for any of {list(GATED_AT_CALL)} -- "
              f"have they been renamed? Update this gate rather than deleting "
              f"it.")
        return 1
    if fails:
        for f in fails:
            print(f"FAIL: {f}")
        return 1
    print(f"PASS: {checked} direct floor-writer call site(s) gated, and "
          f"{CHOKE_POINT} still carries the gate every routing tab relies on "
          f"(#693)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
