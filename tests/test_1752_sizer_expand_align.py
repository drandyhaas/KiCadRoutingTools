#!/usr/bin/env python3
"""No sizer item in the plugin combines wx.EXPAND with an alignment flag (#1752).

A box sizer ignores wxALIGN_RIGHT / CENTRE_HORIZONTAL / BOTTOM / CENTRE_VERTICAL
when wxEXPAND is set, and a debug build of wx (what Linux distributions ship)
says so with an "An assertion failed!" dialog the moment the routing dialog
opens. The GUI gates cannot see it: they set WXSUPPRESS_SIZER_FLAGS_CHECK so a
headless dialog never blocks. So this reads the source instead: every call in
kicad_routing_plugin/ whose arguments name wx.EXPAND together with one of those
flags (wx.ALIGN_CENTER/CENTRE included, which is both centring flags).

  1. control: the scan flags a known-bad snippet and passes a good one;
  2. the plugin has no such call.

No wx needed.  python3 tests/test_1752_sizer_expand_align.py
"""
import ast
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PLUGIN = os.path.join(REPO, 'kicad_routing_plugin')
IGNORED_BY_EXPAND = ('ALIGN_RIGHT', 'ALIGN_CENTER', 'ALIGN_CENTRE', 'ALIGN_BOTTOM')


def _names(node):
    return {n.attr for n in ast.walk(node) if isinstance(n, ast.Attribute)} | \
        {n.id for n in ast.walk(node) if isinstance(n, ast.Name)}


def offending_calls(src):
    """Line numbers of calls with an argument naming EXPAND plus an ignored align."""
    hits = []
    for node in ast.walk(ast.parse(src)):
        if not isinstance(node, ast.Call):
            continue
        for arg in list(node.args) + [k.value for k in node.keywords]:
            if isinstance(arg, ast.Call):
                continue    # a nested call is visited on its own
            names = _names(arg)
            if 'EXPAND' in names and any(
                    n.startswith(IGNORED_BY_EXPAND) for n in names):
                hits.append(node.lineno)
                break
    return hits


def main():
    failures = []
    bad = "s.Add(w, 1, wx.EXPAND | wx.ALIGN_CENTER_VERTICAL)\n" \
          "s.Add(w, flag=wx.ALIGN_RIGHT | wx.EXPAND | wx.ALL)\n"
    good = "s.Add(w, 1, wx.EXPAND | wx.LEFT)\ns.Add(w, 0, wx.ALIGN_CENTER_VERTICAL)\n"
    if offending_calls(bad) != [1, 2] or offending_calls(good):
        failures.append(f"control: bad={offending_calls(bad)} good={offending_calls(good)}")

    scanned = 0
    for root, _dirs, files in os.walk(PLUGIN):
        for fn in sorted(files):
            if not fn.endswith('.py'):
                continue
            path = os.path.join(root, fn)
            with open(path, encoding='utf-8') as fh:
                src = fh.read()
            scanned += 1
            for line in offending_calls(src):
                failures.append(f"{os.path.relpath(path, REPO)}:{line}: "
                                "wx.EXPAND with an alignment it overrides")
    if scanned < 10:
        failures.append(f"scanned only {scanned} plugin files -- wrong path?")

    for f in failures:
        print(f"  FAIL {f}")
    print(f"{'FAIL' if failures else 'PASS'}: {scanned} plugin files scanned")
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
