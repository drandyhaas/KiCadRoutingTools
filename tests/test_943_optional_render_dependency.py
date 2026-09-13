#!/usr/bin/env python3
"""#943 / #944: the raster stack is OPTIONAL -- rendering only, never routing.

Main fixed three defects in `kicad_routing_plugin/deps_check.py`, the SWIG
entry point's first-launch pip dialog. **That file does not exist on this
branch**: the IPC port deleted it along with `action_plugin.py`, because
KiCad 10 provisions a per-plugin venv from `requirements.txt` on the first
action invocation. There is no import-name probe here, no `OPTIONAL_PACKAGES`
table and no one-click pip offer, so main's A (the `import Pillow` probe that
could never pass), B (the generated blocking list) and D (#944's PEP 668
install path) have no counterpart to test and are deliberately absent.

What DOES carry over, and is tested here:

  B'. The dialog's own hand-written gate. `routing_dialog.py` (this branch's
      `swig_gui.py`) re-implements `startup_checks.check_python_dependencies`
      BY HAND and must mirror that list and no more. Pillow is not on it --
      the GUI's only raster consumers, the movie recorder and the placement
      preview, both disable themselves -- so a venv that resolved everything
      except Pillow still opens a routing dialog. Nothing in the plugin may
      import PIL at module scope either, or the gate is bypassed by the
      import that reaches it.

  C.  `check_render_dependencies` raised StartupCheckError, a RuntimeError, so
      every `except ImportError` consumer that MEANS to disable rendering
      walked past it. It raises RenderDependencyError now, which is both.

  B2. Placement GRADING does not need the raster stack.

Run with:  python3 tests/test_943_optional_render_dependency.py
"""
import ast
import importlib
import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_router'))   # #522
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_tools'))    # #522
sys.path.insert(0, os.path.join(ROOT_DIR, 'kicad_routing_plugin'))

from startup_checks import (RenderDependencyError, StartupCheckError,  # noqa: E402
                            check_render_dependencies)

PLUGIN_DIR = os.path.join(ROOT_DIR, 'kicad_routing_plugin')

FAILS = []


def check(cond, msg):
    if not cond:
        FAILS.append(msg)
    return cond


def _module_scope_imports(path):
    """Top-level `import X` / `from X import ...` names in one file.

    AST, not a grep: an import nested inside a function or a try/except that
    the gate guards is exactly the shape this test must NOT flag, and a
    source-text scan cannot tell the two apart.
    """
    names = set()
    for node in ast.parse(open(path, encoding='utf-8').read()).body:
        if isinstance(node, ast.Import):
            names.update(a.name.split('.')[0] for a in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            names.add(node.module.split('.')[0])
    return names


# ---------------------------------------------------------------------------
# B'. the dialog's hand-written gate does not carry Pillow
# ---------------------------------------------------------------------------
def test_the_dialog_gate_does_not_block_on_pillow():
    dlg = os.path.join(PLUGIN_DIR, 'routing_dialog.py')
    src = open(dlg, encoding='utf-8').read()
    check("missing.append('Pillow')" not in src
          and 'missing.append("Pillow")' not in src,
          "routing_dialog.py's hand-written dependency gate blocks on Pillow, "
          "which routing does not need -- it would refuse a board this GUI "
          "can route (#943 B, #887)")

    # And the gate cannot be bypassed by an import that reaches it first.
    offenders = sorted(
        os.path.basename(p) for p in
        [os.path.join(PLUGIN_DIR, f) for f in sorted(os.listdir(PLUGIN_DIR))
         if f.endswith('.py')]
        if 'PIL' in _module_scope_imports(p))
    check(not offenders,
          f"these plugin modules import PIL at module scope, so a venv "
          f"without Pillow fails before the gate can decide: {offenders}")


# ---------------------------------------------------------------------------
# C. the render gate is catchable by the consumers that disable rendering
# ---------------------------------------------------------------------------
def test_render_gate_is_an_import_error():
    check(issubclass(RenderDependencyError, ImportError),
          "RenderDependencyError is not an ImportError, so the `except "
          "ImportError` consumers that disable rendering miss it (#943 C)")
    check(issubclass(RenderDependencyError, StartupCheckError),
          "RenderDependencyError is no longer a StartupCheckError, so callers "
          "that catch the startup contract miss it")

    saved = sys.modules.get('PIL', '<absent>')
    sys.modules['PIL'] = None          # makes `from PIL import ...` raise
    try:
        raised = None
        try:
            check_render_dependencies()
        except Exception as exc:                                # noqa: BLE001
            raised = exc
        check(isinstance(raised, RenderDependencyError),
              f"check_render_dependencies raised {type(raised).__name__} with "
              f"Pillow unavailable, not RenderDependencyError")
        check(raised is not None and 'Pillow' in str(raised),
              "the render gate's message does not name Pillow, so it is not "
              "the actionable message #887 asked for")
    finally:
        if saved == '<absent>':
            sys.modules.pop('PIL', None)
        else:
            sys.modules['PIL'] = saved


# ---------------------------------------------------------------------------
# B2. placement GRADING does not need the raster stack
# ---------------------------------------------------------------------------
def test_render_placement_imports_without_pillow():
    """`board_context` and the stress predictors import render_placement for
    PlacementModel / legality_findings and draw nothing. A module-scope raster
    gate made Pillow a requirement of grading a placement."""
    saved_pil = sys.modules.get('PIL', '<absent>')
    dropped = [m for m in list(sys.modules)
               if m == 'render_placement' or m.startswith('render_placement.')]
    saved_mods = {m: sys.modules.pop(m) for m in dropped}
    sys.modules['PIL'] = None
    try:
        mod = importlib.import_module('render_placement')
        for name in ('PlacementModel', 'legality_findings'):
            check(hasattr(mod, name),
                  f"render_placement imported without Pillow but has no "
                  f"{name} -- the lazy split dropped a non-raster export")
    except Exception as exc:                                    # noqa: BLE001
        check(False,
              f"render_placement cannot be imported without Pillow: "
              f"{type(exc).__name__}: {exc} (#943)")
    finally:
        sys.modules.pop('render_placement', None)
        sys.modules.update(saved_mods)
        if saved_pil == '<absent>':
            sys.modules.pop('PIL', None)
        else:
            sys.modules['PIL'] = saved_pil


def run():
    test_the_dialog_gate_does_not_block_on_pillow()
    test_render_gate_is_an_import_error()
    test_render_placement_imports_without_pillow()

    if FAILS:
        for f in FAILS:
            print(f"  FAIL  {f}")
        print(f"\n{len(FAILS)} check(s) FAILED")
        return False
    print("PASS  #943 optional raster dependency and catchable render gate "
          "(the deps_check half has no counterpart on ipc-migration)")
    return True


if __name__ == '__main__':
    sys.exit(0 if run() else 1)
