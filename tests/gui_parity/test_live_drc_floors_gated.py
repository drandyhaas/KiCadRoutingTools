#!/usr/bin/env python3
"""#693: every update_live_drc_floors call must be gated on 'fix DRC settings'.

The GUI has TWO independent DRC writers, and only one of them was gated:

  1. apply_targets_to_board   -- netclasses + severities.  GATED.
  2. update_live_drc_floors   -- the live board's design-settings FLOORS
     (m_MinClearance, m_TrackMinWidth, m_ViasMinSize, m_MinThroughDrill,
     m_ViasMinAnnularWidth, m_HoleToHoleMin, + the Default netclass).  NOT.

So unchecking "Fix DRC settings after routing" suppressed (1) and left (2)
rewriting Board Setup anyway. The reporter watched Minimum annular width go
0.15 -> 0.05 with the box unchecked (#693). It was ungated at ALL FIVE call
sites -- every routing tab plus the plan executor -- so this is a drift guard,
not a spot fix: the CLI gates the twin (fix_project_for_output) on
--no-fix-drc-settings, and a sixth call site must not silently reintroduce the
bug.

Pure AST, no wx and no pcbnew -- runs anywhere in about a second.
"""
import ast
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
PLUGIN = os.path.join(ROOT, "kicad_routing_plugin")
FUNC = "update_live_drc_floors"
# A gate is any enclosing `if` whose test mentions the toggle. ai_plan reads the
# live checkbox into a local first (it owns the real dialog), so accept that too.
GATE_TOKENS = ("fix_drc", "_fixdrc693")


def main():
    fails, checked = [], 0
    for name in sorted(os.listdir(PLUGIN)):
        if not name.endswith(".py"):
            continue
        path = os.path.join(PLUGIN, name)
        src = open(path, encoding="utf-8").read()
        if FUNC not in src:
            continue
        tree = ast.parse(src)
        for node in ast.walk(tree):
            if not (isinstance(node, ast.Call)
                    and getattr(node.func, "id", None) == FUNC):
                continue
            checked += 1
            line = node.lineno
            tests = [ast.get_source_segment(src, n.test) or ""
                     for n in ast.walk(tree)
                     if isinstance(n, ast.If)
                     and n.lineno <= line <= (n.end_lineno or n.lineno)]
            if not any(tok in t for t in tests for tok in GATE_TOKENS):
                fails.append(
                    f"{name}:{line} calls {FUNC} WITHOUT a "
                    f"'fix DRC settings' gate -- an unchecked box would still "
                    f"rewrite the board's Board Setup floors (#693)")

    if not checked:
        print(f"FAIL: found no {FUNC} call sites at all -- has it been renamed? "
              f"Update this gate rather than deleting it.")
        return 1
    if fails:
        for f in fails:
            print(f"FAIL: {f}")
        return 1
    print(f"PASS: all {checked} {FUNC} call site(s) are gated on the "
          f"'Fix DRC settings after routing' toggle (#693)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
