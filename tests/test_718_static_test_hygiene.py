#!/usr/bin/env python3
"""Static hygiene gates over the test suite itself (issue #718).

Two invariants that no test asserts about itself:

1. **No test may spawn a CLI at a path the #522 reorg vacated.** python writes
   `can't open file ...: [Errno 2]` to STDERR and exits 2, so a test asserting
   on STDOUT fails with a bare message that reads exactly like a product bug,
   and one asserting `returncode == 0` reads exit 2 as the tool's own.
   `ed779096` swept 25 such files; #718 found four more, because two of them
   were correct on their branches and went stale on MERGE. A sweep cannot
   prevent that -- only a standing gate can.

2. **A test depending on a board under the gitignored `wk/` tree is declared.**
   Absent that board it `skipTest`s and the file still exits 0: green while
   covering nothing. That is how #718 item 3 (a sub-test pinning tessellation
   `6166a98b` reverted) stayed invisible on every machine but one.

**Why this is its own file rather than part of test_457_fresh_clone_fixtures,
where it started.** `run_all.is_integration()` decides a file is slow by
substring-matching its SOURCE for the spawn-module name and the run_utils
imports (see `run_all._INTEGRATION_MARKERS`); `--fast` then skips it. test_457
shells out to `git ls-files`, so it carries that marker and everything in it is
skipped by `--fast` -- including checks that spawn nothing. These two checks
spawn nothing and finish in well under a second, so they belong in the fast
loop where a stale path is caught immediately instead of 900 s later. That is
also why the prose here says "child-process" rather than the marker word: the
classifier reads source text, not behaviour. (Credit: this point, and the
vacuity guard below, come from edgehero's PR #719.)

    python3 tests/test_718_static_test_hygiene.py
"""

import ast
import os
import re
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)


#: Tests whose coverage depends on a board under the GITIGNORED `wk/` work
#: tree. Registered, not merely tolerated.
#:
#: #718 item 5: absent those boards each of these `skipTest()`s and the FILE
#: STILL EXITS 0 -- green while covering nothing. That is how item 3 of the same
#: issue (a sub-test pinning custom-pad tessellation that 6166a98b deliberately
#: reverted) stayed invisible on every machine but the reporter's, and it makes
#: the suite's green/red state machine-dependent in a way nothing reported.
#: `test_no_test_reads_an_untracked_board_directly` cannot see this class: its
#: regexes scan `kicad_files/` only.
#:
#: These boards are stress/placement-run OUTPUTS, not fixtures -- there is no
#: recipe to add, so the contract here is DISCLOSURE, not reproducibility. The
#: guard holds the map in BOTH directions: a new `wk/` dependency must be
#: declared, AND a registration that no longer matches the source is reported.
#: One direction is not enough -- a stale registration passed the #696
#: containment guard 28/28 while the thing it named had moved.
_WK_DEPENDENT = {
    # #788 Arm B only: it re-grades 14 declared study boards to prove the
    # committed literals still match the instrument. Arm A -- the claim
    # those literals support -- runs on a clean clone and is what keeps
    # this file from being green-while-covering-nothing when wk/ is absent.
    'test_788_marginal_literals.py': ['wk/703/study'],
    'test_outline_prefilter.py': ['wk/run19/urchin/base.kicad_pcb'],
    'test_part_class.py': ['wk/b2/tigard__swap/d0/perturbed.control.kicad_pcb',
                           'wk/b2/tigard__swap/d0/perturbed.kicad_pcb'],
    'test_place_reconstruct.py':
        ['wk/b2/tigard__swap/d0/perturbed.control.kicad_pcb',
         'wk/b2/tigard__swap/d0/perturbed.kicad_pcb'],
    'test_placement_pad_legality.py':
        ['../wk/b2/tigard__swap/d0/perturbed.kicad_pcb'],
    'test_run20_run_watch.py': ['wk/run20'],
    'test_run4_custom_pad_circle.py': ['wk/run3/final2.kicad_pcb'],
    'test_run4_reconstruct.py': ['wk/b2/tigard__swap/d0/perturbed.kicad_pcb'],
    'test_run5_emit_guard.py': ['wk/b2/tigard__swap/d0/perturbed.kicad_pcb'],
    'test_run5_exchange.py': ['wk/b2/tigard__swap/d0/perturbed.kicad_pcb'],
    'test_run6_backlog.py': ['wk/run5/s1_pour.kicad_pcb'],
    'test_run6_body_overlap.py': ['wk/run2/original/tigard_v10.kicad_pcb',
                                  'wk/run5/final5.kicad_pcb'],
    'test_run6_check_assembly.py':
        ['wk/b2/tigard__swap/d0/perturbed.kicad_pcb',
         'wk/run2/original/tigard_v10.kicad_pcb',
         'wk/run5/final5.kicad_pcb'],
    'test_run7_vectors.py': ['wk/b2/tigard__swap/d0/perturbed.kicad_pcb'],
    'test_run8_airwire_refuted.py':
        ['wk/run7/glasgow_revC/perturbed.kicad_pcb'],
    'test_run8_oob_outline.py': ['wk/run7/glasgow_revC/perturbed.kicad_pcb'],
    'test_run8_rigid_consistency.py':
        ['wk/run7/glasgow_revC/perturbed.control.kicad_pcb',
         'wk/run7/glasgow_revC/perturbed.kicad_pcb',
         'wk/run7/glasgow_revC/rL_repair.kicad_pcb'],
    'test_run8_starved_face_gate.py': ['wk/run7/glasgow_revC'],
}

#: `os.path.join(<repo root>, 'a', 'b')` with every component after the base a
#: literal. Applied to `_code_only` output, which puts each call on one line;
#: the whitespace collapse keeps it working on raw text as a fallback.
_JOIN_RE = re.compile(
    r"""os\.path\.join\(\s*([A-Za-z_][A-Za-z0-9_.()'"\[\], ]*?)\s*,\s*"""
    r"""((?:'[^']*'|"[^"]*")\s*(?:,\s*(?:'[^']*'|"[^"]*")\s*)*)\)""")

#: One single- or double-quoted string literal.
_STR_RE = re.compile(r"'[^']*'" + r'|"[^"]*"')

_ROOT_BASES = ('ROOT', 'REPO', 'ROOT_DIR')

#: A child-process argv list literal -- `[sys.executable, ..., <path>, ...]`.
_ARGV_RE = re.compile(r"\[[^\[\]]*sys\.executable[^\[\]]*\]")

#: `os.path.join(<repo root>, <bare identifier>)`: the repo root joined with a
#: script name only known at runtime.
_JOIN_VAR_RE = re.compile(
    r"os\.path\.join\(\s*(?:ROOT|REPO|ROOT_DIR)\s*,\s*"
    r"[A-Za-z_][A-Za-z0-9_]*\s*\)")

#: Tests that legitimately join the repo root with a VARIABLE inside a
#: child-process argv. Declared with the reason, and held in both directions.
_ROOT_JOIN_ARGV_OK = {
    'test_431_skill_commands.py':
        'its `tool` values come from discovered_tools(), which yields paths '
        'ALREADY qualified by directory ("py_router/route.py") straight out '
        'of the skill text and filters them through os.path.isfile -- so the '
        'join is over a relative path, not a bare basename.',
}


def _code_only(src):
    """`src` with comments and docstrings gone, so PROSE cannot be scanned.

    These gates match path expressions, and a path expression written inside a
    docstring to EXPLAIN the defect is not an instance of it -- the first draft
    of test_no_test_spawns_a_script_that_moved flagged its own docstring. Round
    tripping through ast drops every comment and, once the docstring nodes are
    removed, every docstring; real string literals in code survive, which is
    the point. On a syntax error, fall back to the raw text: a gate that goes
    quiet on a file it cannot parse is worse than one that is slightly noisy.
    """
    try:
        tree = ast.parse(src)
    except SyntaxError:                                    # pragma: no cover
        return src
    for node in ast.walk(tree):
        if not isinstance(node, (ast.Module, ast.ClassDef, ast.FunctionDef,
                                 ast.AsyncFunctionDef)):
            continue
        body = node.body
        if (body and isinstance(body[0], ast.Expr)
                and isinstance(body[0].value, ast.Constant)
                and isinstance(body[0].value.value, str)):
            node.body = body[1:] or [ast.Pass()]
    ast.fix_missing_locations(tree)
    return ast.unparse(tree)


def _py_files(only_tests):
    """Every .py under tests/, RECURSIVELY, as (relpath-from-tests, code).

    Recursive and not limited to `test_*.py` on purpose. #718's worst instance
    was `tests/stress/corpus_noop_sweep.py`, which carried THREE stale root
    spawns and reported nothing for months, because `run_all.discover()` globs
    `tests/test_*.py` NON-recursively and never ran it at all. A gate scoped
    the same way as the runner is blind to exactly the files the runner cannot
    see -- which are the ones most likely to rot. (Scope taken from edgehero's
    PR #719; the first draft of this gate had the runner's blind spot.)

    only_tests limits the walk to `test_*.py`, for the wk/ census: a hand-run
    stress tool reading a work board is a different question from a TEST that
    silently skips, and the census is about coverage that lies.
    """
    me = os.path.basename(__file__)
    for dirpath, dirnames, filenames in os.walk(TESTS_DIR):
        dirnames[:] = [d for d in sorted(dirnames)
                       if d not in ('__pycache__', '.pytest_cache')]
        for fname in sorted(filenames):
            if not fname.endswith('.py') or fname == me:
                continue
            if only_tests and not fname.startswith('test_'):
                continue
            path = os.path.join(dirpath, fname)
            rel = os.path.relpath(path, TESTS_DIR).replace(os.sep, '/')
            try:
                with open(path, encoding='utf-8', errors='replace') as fh:
                    yield rel, _code_only(fh.read())
            except OSError:                                # pragma: no cover
                continue


def _test_files():
    """The wk/ census scope: `test_*.py`, recursively, keyed by BASENAME
    (every one of today's lives in tests/ itself)."""
    for rel, src in _py_files(only_tests=True):
        yield os.path.basename(rel), src


def _joins(src):
    """(base_expr, [literal, ...]) for every literal os.path.join in `src`."""
    flat = re.sub(r'\s+', ' ', src)
    for m in _JOIN_RE.finditer(flat):
        lits = [x[1:-1] for x in _STR_RE.findall(m.group(2))]
        yield m.group(1).strip(), lits


def _scan_wk_deps():
    """file -> sorted board paths it resolves out of the REPO's `wk/`.

    Only joins rooted at the repo count. A test that builds a `wk` inside its
    own tempdir (`os.path.join(td, 'wk')` -- test_provenance_audit,
    test_blind_stage_identity, test_run15_handback_contract) depends on nothing
    external and must NOT be swept in.
    """
    found = {}
    for fname, src in _test_files():
        for base, lits in _joins(src):
            if 'wk' not in lits:
                continue
            if base not in _ROOT_BASES and '__file__' not in base:
                continue
            found.setdefault(fname, set()).add('/'.join(lits))
    return {k: sorted(v) for k, v in found.items()}


def test_wk_dependent_tests_are_declared():
    """#718 item 5: a test that silently skips is not coverage -- name it."""
    found = _scan_wk_deps()
    unregistered = sorted(set(found) - set(_WK_DEPENDENT))
    departed = sorted(set(_WK_DEPENDENT) - set(found))
    changed = sorted(f for f in set(found) & set(_WK_DEPENDENT)
                     if found[f] != sorted(_WK_DEPENDENT[f]))
    assert not unregistered, (
        "test(s) reading a board out of the gitignored wk/ work tree without "
        "declaring it:\n  " + "\n  ".join(
            f"{f}: {', '.join(found[f])}" for f in unregistered)
        + "\nAbsent that board the test skipTest()s and the file still exits "
          "0 -- green while covering nothing. Add it to _WK_DEPENDENT so the "
          "dependency is on the record, or use a tracked board.")
    assert not departed, (
        "_WK_DEPENDENT names file(s) that no longer read a wk/ board:\n  "
        + "\n  ".join(departed) + "\nA stale registration is a guard that "
        "passes while covering nothing -- drop the entry.")
    assert not changed, (
        "_WK_DEPENDENT is out of date for:\n  " + "\n  ".join(
            f"{f}: declared {sorted(_WK_DEPENDENT[f])} != found {found[f]}"
            for f in changed))
    absent = sorted(
        f for f, boards in found.items()
        if any(not os.path.exists(
            os.path.normpath(os.path.join(ROOT, b))) for b in boards))
    print(f"  PASS: {len(found)} wk/-dependent test file(s) declared")
    if absent:
        print(f"  NOTE: {len(absent)} of them are INERT on this machine (the "
              f"wk/ board is absent) -- they report PASS while covering "
              f"nothing:")
        for f in absent:
            print(f"          {f}")


def test_no_test_spawns_a_script_that_moved():
    """#718 item 1: spawning a CLI at a path the #522 reorg vacated.

    CPython writes `can't open file ...: [Errno 2]` to STDERR and exits 2, so a
    test asserting on STDOUT fails as a bare AssertionError that reads like a
    product bug, and one asserting `returncode == 0` reads exit 2 as the tool's
    own. test_run6_backlog and test_run5_exchange were correct on their
    branches and went stale on MERGE -- which is why ed779096's sweep missed
    them, and why this is a standing gate rather than another sweep.

    TWO forms, because one of them is what actually shipped:

    * LITERAL -- `os.path.join(ROOT, 'route.py')`. Decided exactly: an offender
      is a name that is NOT at the root but IS where the shipped resolver
      looks. A literal that resolves nowhere (test_krt_capabilities' deliberate
      `no_such_file.py`) is a negative control, not a stale path.

    * VARIABLE -- `os.path.join(ROOT, script)` inside a child-process argv,
      which
      is the form BOTH #718 offenders used. `run_utils.tool`'s docstring says a
      static lint cannot do this job, and it is right that the *value* is only
      knowable at runtime -- but the SHAPE is not. Joining the repo root with a
      bare script name is the assumption #522 falsified, so the shape is the
      defect, and the few legitimate uses are declared below. Scoped to
      child-process argv on purpose: `os.path.join(ROOT, _p522)` for a sys.path
      insert is the same shape and is not a spawn (46 such joins across the
      suite, 1 of them an argv).
    """
    sys.path.insert(0, ROOT)
    from krt_capabilities import _tool_path
    offenders = []
    argv_var = set()
    for fname, src in _py_files(only_tests=False):
        flat = re.sub(r'\s+', ' ', src)
        for base, lits in _joins(src):
            if base not in _ROOT_BASES or len(lits) != 1:
                continue
            name = lits[0]
            if not name.endswith('.py'):
                continue
            if os.path.isfile(os.path.join(ROOT, name)):
                continue
            resolved = _tool_path(ROOT, name)
            if os.path.isfile(resolved):
                offenders.append(
                    f"{fname}: spawns ROOT/{name} (literal), which lives at "
                    f"{os.path.relpath(resolved, ROOT)}")
        for m in _ARGV_RE.finditer(flat):
            if _JOIN_VAR_RE.search(m.group(0)):
                argv_var.add(fname)
    undeclared = sorted(argv_var - set(_ROOT_JOIN_ARGV_OK))
    departed = sorted(set(_ROOT_JOIN_ARGV_OK) - argv_var)
    offenders += [
        f"{f}: spawns os.path.join(ROOT, <variable>) -- the shape #522 "
        f"falsified" for f in undeclared]
    assert not offenders, (
        "test(s) spawning a CLI at a path the #522 reorg vacated:\n  "
        + "\n  ".join(offenders)
        + "\nUse tests/run_utils.tool() / tool_env(), which resolve the tool "
          "wherever it is and RAISE by name when it is absent, instead of "
          "dying into an empty stdout three frames away. If the variable "
          "genuinely carries a directory-qualified path, declare it in "
          "_ROOT_JOIN_ARGV_OK with the reason.")
    assert not departed, (
        "_ROOT_JOIN_ARGV_OK declares file(s) that no longer join ROOT with a "
        "variable in an argv:\n  " + "\n  ".join(departed)
        + "\nDrop the entry -- an exemption for something that stopped "
          "happening is a hole nobody is watching.")
    print(f"  PASS: no test spawns a root-level script the reorg moved "
          f"({len(_ROOT_JOIN_ARGV_OK)} declared exemption)")


def test_the_scanners_still_match_something():
    """A scan that matches nothing passes for free.

    Both gates above are searches, and a search that has quietly stopped
    matching -- a renamed idiom, a broken pattern, an `ast` shape that moved --
    reports a clean tree indistinguishably from a tree that IS clean. #718's
    own shapes are gone from the repo now, so there is no live positive left to
    prove the scanners work; this asserts the corpus they run over is still
    populated. Thresholds are floors well under today's counts, not targets.

    (Taken from edgehero's PR #719, which had this and the first draft of this
    file did not -- after that draft shipped a version of the spawn gate that
    printed PASS against the very source it was written to reject.)
    """
    files = list(_py_files(only_tests=False))
    assert len(files) >= 200, (
        f'the scanner sees only {len(files)} test file(s) -- it has stopped '
        f'finding them, so a green result here means nothing')

    argvs = joins = 0
    for _fname, src in files:
        flat = re.sub(r'\s+', ' ', src)
        argvs += len(_ARGV_RE.findall(flat))
        joins += sum(1 for _ in _joins(src))
    assert argvs >= 20, (
        f'only {argvs} child-process argv list(s) matched -- _ARGV_RE has '
        f'stopped seeing them, so the spawn gate is vacuous')
    assert joins >= 100, (
        f'only {joins} literal os.path.join call(s) matched -- _JOIN_RE has '
        f'stopped seeing them, so both gates are vacuous')
    print(f'  PASS: scanners see {len(files)} file(s), {argvs} argv list(s), '
          f'{joins} literal join(s)')


#: Modules the standard library ships on POSIX and NOT on Windows. Imported at
#: module scope, any of these makes its file unimportable there -- and a file
#: that cannot be imported takes down everything that imports it, at import
#: time, before a single assertion runs.
_POSIX_ONLY = ('resource', 'fcntl', 'termios', 'pwd', 'grp', 'posix',
               'crypt', 'syslog', 'nis', 'spwd')


def test_no_module_scope_posix_only_import():
    """A POSIX-only module imported at top level is a Windows-wide outage.

    #882: `tests/stress/fill_timing_census.py` imported `resource` at module
    scope, so `tests/test_831_fill_preflight_census.py` -- a collected test
    that only calls the pure `analyse()` and never touches CPU accounting --
    died at IMPORT on every Windows machine. The suite carried a standing
    failure there, which is the expensive part: a permanent red teaches the
    reader to skim the failure list.

    The repo already had the right shape in both available forms --
    `py_router/memory_debug.py` guards with a `_HAS_RESOURCE` flag, and
    `tests/stress/redo_stress_test.py` imports inside the one function that
    needs it. This gate is what stops the third instance being written.

    Scoped to the WHOLE repo, not just `tests/`, deliberately: the same import
    in `py_router/` would break the router itself on Windows rather than one
    test, so the cheaper place to catch it is everywhere.
    """
    roots = [TESTS_DIR] + [os.path.join(ROOT, d) for d in
                           ('py_router', 'py_placer', 'py_tools',
                            'kicad_routing_plugin')]
    bad, scanned, unparseable = [], 0, []
    for root in roots:
        if not os.path.isdir(root):
            continue
        for dirpath, dirnames, filenames in os.walk(root):
            dirnames[:] = [d for d in sorted(dirnames)
                           if d not in ('__pycache__', '.pytest_cache')]
            for fname in sorted(filenames):
                if not fname.endswith('.py'):
                    continue
                path = os.path.join(dirpath, fname)
                try:
                    with open(path, encoding='utf-8', errors='replace') as fh:
                        tree = ast.parse(fh.read())
                except SyntaxError as exc:
                    # REPORTED, not skipped. A file this gate cannot parse is
                    # a file it cannot clear, and swallowing that is how the
                    # gate would go quiet on exactly the file someone just
                    # broke -- caught when a deliberate mutation of
                    # fill_timing_census.py produced a SyntaxError and this
                    # check printed PASS over one file FEWER. `_code_only`
                    # above already states the rule: a gate that goes quiet on
                    # a file it cannot parse is worse than one that is
                    # slightly noisy.
                    unparseable.append(
                        f'{os.path.relpath(path, ROOT)}: {exc.__class__.__name__}'
                        f' line {exc.lineno}')
                    continue
                except OSError:                            # pragma: no cover
                    continue
                scanned += 1
                # TOP-LEVEL only. An import inside a function or a try/except
                # is the correct shape and must not be flagged.
                for node in tree.body:
                    names = []
                    if isinstance(node, ast.Import):
                        names = [a.name.split('.')[0] for a in node.names]
                    elif isinstance(node, ast.ImportFrom) and node.module:
                        names = [node.module.split('.')[0]]
                    for name in names:
                        if name in _POSIX_ONLY:
                            bad.append(
                                f'{os.path.relpath(path, ROOT)}'
                                f':{node.lineno}: {name}')
    assert not bad, (
        'POSIX-only module(s) imported at module scope, so these files cannot '
        'be imported on Windows:\n  ' + '\n  '.join(bad)
        + '\n  Guard with try/except and a _HAS_X flag (see '
          'py_router/memory_debug.py), or import inside the function that '
          'needs it (see tests/stress/redo_stress_test.py).')
    assert not unparseable, (
        'file(s) this gate could not parse, so it cleared them without '
        'looking:\n  ' + '\n  '.join(unparseable))
    print(f'  PASS: no module-scope POSIX-only import, over {scanned} file(s)')
def _guard_end(tree):
    """Line of the last top-level `if __name__ == '__main__':` block's end."""
    end = None
    for node in tree.body:
        if (isinstance(node, ast.If)
                and isinstance(node.test, ast.Compare)
                and isinstance(node.test.left, ast.Name)
                and node.test.left.id == '__name__'):
            end = max(end or 0, node.end_lineno or node.lineno)
    return end


def _defines_tests(node):
    """Does this top-level node define test(s), and under what name?"""
    if isinstance(node, ast.ClassDef):
        if any(isinstance(b, ast.FunctionDef) and b.name.startswith('test')
               for b in node.body):
            return f'class {node.name}'
    elif (isinstance(node, ast.FunctionDef)
          and node.name.startswith('test')):
        return f'def {node.name}'
    return None


def test_no_test_is_defined_after_its_own_runner():
    """A test defined BELOW `if __name__ == '__main__':` never runs.

    The guard executes the runner and the runner exits, so anything defined
    after it does not exist yet -- `unittest.main()` discovers what is bound
    at the moment it is called, and a hand-rolled `for t in TESTS` cannot list
    a function defined later. Either way the file reports OK and `run_all`
    records a PASS, which is indistinguishable from having checked.

    Measured when this gate was written (#876): FIVE files, 27 dead tests.
    `test_run6_body_overlap.py` ran 9 of 20 and hid four stale corpus
    assertions; `test_431_render_placement.py` had 8 unregistered functions,
    one of which was red the moment it ran. This is the same failure as
    `test_every_test_in_this_file_is_registered` below, one file wider -- that
    one caught the pattern in THIS file, and nothing looked at the others.
    """
    bad = {}
    for rel, src in _py_files(only_tests=True):
        try:
            tree = ast.parse(src)
        except SyntaxError:                                # pragma: no cover
            continue
        end = _guard_end(tree)
        if end is None:
            continue
        after = [name for node in tree.body
                 if node.lineno > end
                 for name in [_defines_tests(node)] if name]
        if after:
            bad[rel] = after
    assert not bad, (
        'test(s) defined after the runner that would have run them, so they '
        'never run:\n  ' + '\n  '.join(
            f'{rel}: {", ".join(names)}' for rel, names in sorted(bad.items()))
        + '\n  Move `if __name__ == ...` to the END of the file.')
    print(f'  PASS: no test defined after its own runner, over '
          f'{sum(1 for _ in _py_files(only_tests=True))} test file(s)')


def test_every_test_is_registered_in_its_files_own_list():
    """A file with a module-level `TESTS = [...]` must list every test it
    defines.

    The registry idiom is hand-maintained, so a function added to the bottom
    of such a file runs only if someone also adds it to the list -- and one
    that never runs is indistinguishable from one that passes. Measured at
    #876: `test_431_render_placement.py` defined 28 tests and registered 20.

    Scoped to files that HAVE such a list: a unittest file has no registry and
    is covered by the gate above instead.
    """
    bad = {}
    for rel, src in _py_files(only_tests=True):
        try:
            tree = ast.parse(src)
        except SyntaxError:                                # pragma: no cover
            continue
        listed = None
        for node in tree.body:
            if (isinstance(node, ast.Assign)
                    and any(getattr(t, 'id', None) == 'TESTS'
                            for t in node.targets)
                    and isinstance(node.value, (ast.List, ast.Tuple))):
                listed = {e.id for e in node.value.elts
                          if isinstance(e, ast.Name)}
        if listed is None:
            continue
        # `TESTS.append(...)` is an accepted way to register one late.
        for node in ast.walk(tree):
            if (isinstance(node, ast.Call)
                    and isinstance(node.func, ast.Attribute)
                    and node.func.attr == 'append'
                    and isinstance(node.func.value, ast.Name)
                    and node.func.value.id == 'TESTS'):
                listed |= {a.id for a in node.args if isinstance(a, ast.Name)}
        defined = {node.name for node in tree.body
                   if isinstance(node, ast.FunctionDef)
                   and node.name.startswith('test')}
        missing = sorted(defined - listed)
        if missing:
            bad[rel] = missing
    assert not bad, (
        'test(s) defined but absent from their file\'s own TESTS list, so '
        'they never run:\n  ' + '\n  '.join(
            f'{rel}: {", ".join(names)}' for rel, names in sorted(bad.items())))
    print('  PASS: every registry-style test file lists all its tests')


TESTS = [
    test_wk_dependent_tests_are_declared,
    test_no_test_spawns_a_script_that_moved,
    test_the_scanners_still_match_something,
    test_no_module_scope_posix_only_import,
    test_no_test_is_defined_after_its_own_runner,
    test_every_test_is_registered_in_its_files_own_list,
]


def test_every_test_in_this_file_is_registered():
    """TESTS is hand-maintained, so a test added here runs only if someone
    remembers to list it -- and a test that never runs is indistinguishable
    from one that passes. (edgehero's PR #719 added this to test_457 after
    watching a new gate of his silently not run.)"""
    listed = ({t.__name__ for t in TESTS}
              | {'test_every_test_in_this_file_is_registered'})
    defined = {k for k, v in sorted(globals().items())
               if k.startswith('test_') and callable(v)}
    missing = sorted(defined - listed)
    assert not missing, (
        'test(s) defined in this file but absent from TESTS, so they never '
        'run:\n  ' + '\n  '.join(missing))
    print(f'  PASS: all {len(defined)} test(s) here are registered in TESTS')


TESTS.append(test_every_test_in_this_file_is_registered)


if __name__ == '__main__':
    for t in TESTS:
        print(f'--- {t.__name__}')
        t()
    print('ALL PASS')
