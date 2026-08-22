#!/usr/bin/env python3
"""No test may spawn a repo-root script that the #522 reorg moved (issue #718).

`ee860796` moved route.py, check_drc.py and the rest out of the repo root into
py_router/ py_tools/ py_placer/. A test that kept spawning
`os.path.join(ROOT, 'qfn_fanout.py')` does not fail usefully: python writes
`can't open file ...: [Errno 2]` to STDERR and exits 2, so stdout is EMPTY and
the test's own `assertIn(..., r.stdout)` fails with a bare message that reads
exactly like a product bug. `tests/run_utils.py`'s `tool()` docstring records
that EIGHTEEN tests had this and that several rotted into FALSE PASSES
underneath; `ed779096` swept 25 files.

It still recurred. At #718 four more carried it -- test_run5_exchange and
test_run6_backlog (both red, both misread as product failures) plus three
spawns in tests/stress/corpus_noop_sweep.py, which run_all does not even
discover, so those were broken silently. This is the static half of the gate:
every argv list that starts with `sys.executable` must name a script that
EXISTS.

The runtime half is `run_utils.tool()`, which raises and says where it looked.
Prefer it: a literal path that resolves today can go stale tomorrow, and only
`tool()` fails at the right place.

BE CLEAR ABOUT WHAT THIS CANNOT SEE. #718's two headline tests spawned
`os.path.join(ROOT, script)` with `script` a PARAMETER, so no static check
resolves them -- which is exactly why the fix moved them onto `tool()` rather
than repointing a string. `tool()`'s own docstring makes the same point about
test_431_board_gates. Remaining variable call sites are REPORTED here, not
failed: test_431_skill_commands feeds this shape from `discovered_tools()`,
which only admits names that already resolve, so failing it would be wrong.

    python3 tests/test_718_stale_tool_paths.py
"""
import ast
import glob
import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)

#: argv tokens that sit between the interpreter and the script.
_INTERPRETER_FLAGS = {'-X', 'utf8', '-u'}


def _is_sys_executable(node):
    return (isinstance(node, ast.Attribute) and node.attr == 'executable'
            and isinstance(node.value, ast.Name) and node.value.id == 'sys')


def _is_root(node):
    return isinstance(node, ast.Name) and node.id in ('ROOT', 'ROOT_DIR')


def _root_join(node):
    """(path_parts, all_literal) for `os.path.join(ROOT, ...)`, else None."""
    if not (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
            and node.func.attr == 'join'):
        return None
    if not node.args or not _is_root(node.args[0]):
        return None
    parts, literal = [], True
    for arg in node.args[1:]:
        if isinstance(arg, ast.Constant) and isinstance(arg.value, str):
            parts.append(arg.value)
        else:
            literal = False
    return parts, literal


def _script_element(argv_list):
    """The argv element naming the script: skip sys.executable and its flags."""
    for el in argv_list.elts[1:]:
        if isinstance(el, ast.Constant) and el.value in _INTERPRETER_FLAGS:
            continue
        return el
    return None


def scan():
    """(stale, non_literal): every `[sys.executable, ...]` argv in tests/.

    Scans LIST LITERALS rather than spawn call sites on purpose -- three of the
    #718 spawns were built as `argv = [...]` first and passed by name, so a
    scan anchored on the run/Popen call missed them.

    (Prose note: this file avoids the literal word s-u-b-p-r-o-c-e-s-s because
    run_all classifies a test as slow "integration" by substring-matching its
    SOURCE for it. This check spawns nothing and finishes in well under a
    second, so it belongs in the --fast loop where a stale path is caught
    immediately.)
    """
    stale, non_literal = [], []
    for path in sorted(glob.glob(os.path.join(TESTS_DIR, '**', '*.py'),
                                 recursive=True)):
        if os.path.basename(path) == os.path.basename(__file__):
            continue
        try:
            with open(path, encoding='utf-8', errors='replace') as f:
                tree = ast.parse(f.read())
        except SyntaxError:                                  # not ours to judge
            continue
        rel = os.path.relpath(path, ROOT).replace(os.sep, '/')
        for node in ast.walk(tree):
            if not (isinstance(node, ast.List) and node.elts
                    and _is_sys_executable(node.elts[0])):
                continue
            el = _script_element(node)
            if el is None:
                continue
            # resolved loudly at runtime: nothing to check statically
            if (isinstance(el, ast.Call) and isinstance(el.func, ast.Name)
                    and el.func.id in ('tool', '_tool')):
                continue
            found = _root_join(el)
            if found is None:
                continue                       # not a ROOT-join; not our class
            parts, literal = found
            if not literal:
                non_literal.append((rel, node.lineno))
            elif parts and not os.path.isfile(os.path.join(ROOT, *parts)):
                stale.append((rel, node.lineno, '/'.join(parts)))
    return stale, non_literal


def _elsewhere(name):
    """Where a moved CLI actually lives now -- the half of the message that
    turns 'this is broken' into 'this is the fix'."""
    for pkg in ('py_router', 'py_tools', 'py_placer'):
        if os.path.isfile(os.path.join(ROOT, pkg, name)):
            return '%s/%s' % (pkg, name)
    return None


def main():
    stale, non_literal = scan()

    # A scan that matches nothing passes for free. #718's own shapes are gone
    # from the tree now, so assert the scanner still SEES argv lists at all.
    seen = 0
    for path in glob.glob(os.path.join(TESTS_DIR, '**', '*.py'), recursive=True):
        try:
            with open(path, encoding='utf-8', errors='replace') as f:
                tree = ast.parse(f.read())
        except SyntaxError:
            continue
        seen += sum(1 for n in ast.walk(tree)
                    if isinstance(n, ast.List) and n.elts
                    and _is_sys_executable(n.elts[0]))
    if seen < 20:
        print('  FAIL  the scanner found only %d argv list(s) -- it has stopped '
              'matching, so a green result here means nothing' % seen)
        return 1
    print('  PASS  scanner sees %d [sys.executable, ...] argv list(s)' % seen)

    for rel, line in non_literal:
        print('  note  %s:%d builds the script name from a variable -- only '
              'run_utils.tool() can catch that one, at runtime' % (rel, line))

    if stale:
        print('\n  FAIL  %d test spawn(s) name a repo-root script that is not '
              'there:' % len(stale))
        for rel, line, named in stale:
            moved = _elsewhere(os.path.basename(named))
            print('    %s:%d  ->  %s%s' % (
                rel, line, named,
                '   (it moved to %s)' % moved if moved else
                '   (not found anywhere -- deleted?)'))
        print('\n  Use run_utils.tool(name), which resolves the #522 layout and '
              'RAISES\n  naming what it looked for, instead of letting python '
              'exit 2 into an\n  empty stdout three frames away.')
        return 1

    print('  PASS  no test spawns a missing repo-root script')
    return 0


if __name__ == '__main__':
    sys.exit(main())
