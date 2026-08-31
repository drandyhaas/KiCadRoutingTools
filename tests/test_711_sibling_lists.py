"""ONE sibling-extension list, and a lint that refuses a tenth (#711).

A board's siblings carry things the board file cannot: `.kicad_pro` the DRC
floor, `.kicad_dru` the custom rules, and since #711 `.design-brief.json` the
declared design intent. A chain step that copies the board without them does
not fail -- it silently reverts the next step to inferring what the sibling
declared.

This is a test because the drift already happened, measured on `upstream/main`
before this change: **nine** independent hand-written copies of the same tuple,
**four** of them narrower than the rest, and the one the placement CLIs
actually go through (`portfolio.copy_siblings`) was NOT the one anybody would
think to edit.

    py_router/copy_board.py            SIBLING_EXTS       <- the canonical list
    py_placer/board_store.py           SIBLING_EXT
    py_placer/placement/portfolio.py   inline in copy_siblings
    py_placer/place_reconstruct.py     _SIBLING_EXTS
    py_placer/plane_score.py           inline (pro, dru only)
    py_tools/check_join.py             inline (pro, dru only)
    kicad_routing_plugin/placement_run.py  inline (pro, dru only)
    kicad_routing_plugin/ai_gui.py     inline (pro, dru only)
    py_router/route.py                 a fallback literal

And the test that was supposed to cover this, `test_411_placement_siblings.py`,
carried a **tenth** copy of its own (`SIBLINGS = ('.kicad_pro', '.kicad_dru')`)
-- so it would have stayed green while `place_seed` stranded the brief. A guard
that restates the thing it guards cannot detect a change to it.

So: every site imports `copy_board.SIBLING_EXTS`, and the source scan below
refuses a new literal. The scan asserts it FOUND something before it asserts it
found nothing wrong -- a silently-empty regex reads exactly like a pass.
"""
import ast
import os
import re
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)
for _d in ('py_router', 'py_placer', 'py_tools'):
    sys.path.insert(0, os.path.join(REPO, _d))

from copy_board import SIBLING_EXTS  # noqa: E402

RUN_ALL_FAST_OK = True

#: The declared brief must be in the canonical list, or every chain step that
#: renames a board drops it.
BRIEF_EXT = '.design-brief.json'

#: Directories the scan walks. `.claude/worktrees` holds full clones of this
#: repo and would multiply every hit ~25x.
_ROOTS = ('py_router', 'py_placer', 'py_tools', 'kicad_routing_plugin')

#: A tuple/list literal of two or more KiCad sibling extensions, which is what
#: a hand-written copy looks like.
_LITERAL = re.compile(
    r"""[\(\[]\s*["']\.kicad_(?:pro|prl|dru)["']\s*,\s*["']\.kicad_""")

#: Sites allowed to keep a literal, each with the reason. Anything not here
#: must import the constant. Kept as an exact-count assertion so a new waiver
#: cannot be added silently.
_EXEMPT = {
    # The canonical definition itself.
    os.path.join('py_router', 'copy_board.py'),
    # A FALLBACK for a clone whose layout predates the #522 split, reached
    # only when the import above it fails. It cannot import the constant, by
    # construction -- that is the case it exists for.
    os.path.join('py_router', 'route.py'),
}


def _sources():
    for root in _ROOTS:
        base = os.path.join(REPO, root)
        for dirpath, dirnames, filenames in os.walk(base):
            dirnames[:] = [d for d in dirnames
                           if d not in ('__pycache__', 'target')]
            for fn in filenames:
                if fn.endswith('.py'):
                    yield os.path.join(dirpath, fn)


def test_the_brief_is_in_the_canonical_list():
    assert BRIEF_EXT in SIBLING_EXTS, sorted(SIBLING_EXTS)
    assert '.kicad_pro' in SIBLING_EXTS and '.kicad_dru' in SIBLING_EXTS
    print(f"  PASS: SIBLING_EXTS = {SIBLING_EXTS}")


def test_every_consumer_imports_the_one_list():
    """No module may hand-write a sibling tuple.

    The exempt set is asserted EXACTLY, in both directions: an entry that
    stops being needed is as much a finding as a new literal, because a stale
    waiver is how the next one gets added without argument.
    """
    scanned = 0
    literals = []
    for path in _sources():
        scanned += 1
        rel = os.path.relpath(path, REPO)
        with open(path, encoding='utf-8', errors='replace') as fh:
            text = fh.read()
        if _LITERAL.search(text):
            literals.append(rel)
    # The scan must have looked at something. A regex that silently matches
    # nothing -- a moved directory, a renamed extension -- reads exactly like
    # a clean tree, and that is the failure this whole file is about.
    assert scanned > 100, f"the scan only reached {scanned} files; it is not " \
                          f"looking where it thinks it is"
    found = set(literals)
    expect = {os.path.normpath(p) for p in _EXEMPT}
    assert found == expect, (
        f"hand-written sibling tuple(s): {sorted(found - expect)} must import "
        f"`copy_board.SIBLING_EXTS`; stale waiver(s): {sorted(expect - found)}")
    print(f"  PASS: {scanned} sources scanned, {len(found)} literal(s), all "
          f"exempt with a stated reason")


def test_the_importers_resolve_to_the_same_tuple():
    """Importing is not enough -- it has to be the SAME object's contents."""
    import board_store
    import place_reconstruct
    assert tuple(board_store.SIBLING_EXT) == tuple(SIBLING_EXTS)
    assert tuple(place_reconstruct._SIBLING_EXTS) == tuple(SIBLING_EXTS)
    # portfolio/plane_score/check_join import inside the function body, so
    # assert the call site names the constant rather than a literal.
    for rel, fn in ((os.path.join('py_placer', 'placement', 'portfolio.py'),
                     'copy_siblings'),):
        src = open(os.path.join(REPO, rel), encoding='utf-8').read()
        tree = ast.parse(src)
        body = [n for n in ast.walk(tree)
                if isinstance(n, ast.FunctionDef) and n.name == fn]
        assert body, f"{rel}: no function {fn}"
        names = {n.id for n in ast.walk(body[0]) if isinstance(n, ast.Name)}
        assert 'SIBLING_EXTS' in names, \
            f"{rel}:{fn} does not iterate SIBLING_EXTS"
    print("  PASS: importers resolve to the same tuple")


TESTS = [
    test_the_brief_is_in_the_canonical_list,
    test_every_consumer_imports_the_one_list,
    test_the_importers_resolve_to_the_same_tuple,
]


if __name__ == '__main__':
    for t in TESTS:
        print(f"--- {t.__name__}")
        t()
    print("ALL PASS")
