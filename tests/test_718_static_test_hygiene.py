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


#: Every committed `.json`/`.jsonl` baseline under `tests/`, mapped to the test
#: that READS it and fails when it is wrong -- or, in `_BASELINE_UNGATED`, to
#: the reason it has none (#879).
#:
#: THE EXTENSIONS ARE THE SCOPE, and saying "every committed baseline" would
#: overstate it. The first cut said `.json` and three committed `.jsonl`
#: measurements were invisible; the honest reading of that is not "now it is
#: complete" but "the scope is a list, and a list has an edge". A live one:
#: `tests/stress/RUNBOOK.md` hand-transcribes `831_fill_timing_census.json`'s
#: numbers into a markdown table, which is a recorded measurement under
#: `tests/` that this never sees. Widening to `.md` would mean parsing prose
#: for tables, so it stays out -- named here rather than left to be discovered
#: the way 713 was.
#:
#: "Reads it and fails when it is wrong" is deliberately weaker than
#: "re-derives every cell": four of these honestly do less, and say so.
#:
#: `test_553_recall_regen` re-derives 2 of 36 cells;
#: `test_554_relocation_regen` re-derives the summary only;
#: `test_803_calibration_claims` consumes its two files as the INPUT it holds
#: `docs/placement-calibration.md` to; and
#: `test_789_rule1_withdrawal` re-evaluates a rule over stored metrics. A map
#: that claimed full re-derivation for all of them would be the kind of
#: overstatement this file exists to catch.
#:
#: A baseline nothing re-derives is free to drift arbitrarily far from the tree
#: while still reading as authoritative, and whoever eventually re-records it
#: inherits every delta since the last recording as apparently their own. Not
#: hypothetical: `713_abstention_census.json` drifted for four days, and the
#: re-record was then published in a commit message AND a code comment as one
#: change's effect. It was somebody else's.
#:
#: Declared rather than discovered, and held in BOTH directions like
#: `_WK_DEPENDENT` above: a baseline missing from the map fails, and an entry
#: naming a gate that does not exist -- or a gate that never mentions the file
#: it claims to guard -- fails too. A registration nobody checks is how the
#: #696 containment guard passed 28/28 while the thing it named had moved.
_BASELINE_GATES = {
    'tests/553_diagnosis_recall_baseline.json':
        'tests/test_553_recall_regen.py',
    'tests/554_block_relocation_baseline.json':
        'tests/test_554_relocation_regen.py',
    'tests/554_relocation_reach_baseline.json':
        'tests/test_554_reach_regen.py',
    'tests/713_abstention_census.json':
        'tests/test_713_abstention_drift.py',
    'tests/792_seeding_rows.jsonl':
        'tests/test_792_seeding_claims.py',
    'tests/799_feasibility_rows.jsonl':
        'tests/test_799_feasibility_claims.py',
    'tests/799_feasibility_summary.json':
        'tests/test_799_feasibility_claims.py',
    'tests/831_fill_timing_census.json':
        'tests/test_831_fill_preflight_census.py',
    'tests/placement_ab_baseline.json': 'tests/test_placement_ab.py',
    'tests/placement_calibration_recovered.json':
        'tests/test_803_calibration_claims.py',
    'tests/placement_calibration_rows.json':
        'tests/test_803_calibration_claims.py',
    'tests/placement_rule1_withdrawal.json':
        'tests/test_789_rule1_withdrawal.py',
    'tests/data/714_identity_sha256.json':
        'tests/test_714_identity_write_unchanged.py',
}

#: Committed baselines with NO gate, each with its reason. Being on this list
#: is a DISCLOSURE, not an exemption -- see `_UNGATED_BASELINE_COUNT`.
_BASELINE_UNGATED = {
    'tests/797_seed_exclusive_baseline.json':
        'no test names it; its own `reproduce` key carries a two-command '
        'recipe, which is a recipe rather than a gate',
    'tests/836_flip_vs_reseat_baseline.json':
        'test_836_flip_census.py is a real gate on the two MECHANISMS the '
        'answer rests on, but it re-derives them from the engine and never '
        'opens this file, so the recorded verdict itself is unguarded',
    'tests/measure_847_calibration.json':
        'nothing references it; only measure_847_calibration.py is cited, in '
        'docs/utilities.md and check_channels.py',
    'tests/553_diagnosis_recall_rows.jsonl':
        'ZERO references repo-wide -- 52 KB and 36 recorded rows that nothing '
        'in the tree reads. test_553_recall_regen.py writes its OWN rows into '
        'a temp workdir and never opens this one. The most exposed file in '
        'the tree by this issue\'s own definition, and named here rather than '
        'left to be discovered the way 713 was',
    'tests/stress/corpus_noop_baseline.json':
        'read by tests/stress/corpus_noop_sweep.py, which is not a test_*.py '
        'and lives outside run_all.py the non-recursive glob',
    'tests/stress/modal_sweep/board_value.json':
        'a recorded measurement (`boards_scored: 148`) that is ALSO consumed '
        'as sweep config, written by modal_sweep/curate_boards.py after a big '
        'sweep. Nothing re-derives it',
    'tests/stress/modal_sweep/retry_shape_2127.json':
        'a per-board rescue-step census, cited as the frozen board list in '
        'modal_sweep/PREREG_590_sets21_27.md. Nothing re-derives it',
}

#: A PINNED literal, never `len(_BASELINE_UNGATED)`. Deriving it from the map
#: it guards would make this agree with whatever the map currently says, and
#: the point is that another ungated baseline be a decision somebody takes
#: rather than a line somebody adds.
_UNGATED_BASELINE_COUNT = 7

#: And the total the walk must CONSIDER. Without it the check goes vacuous in
#: silence: widen an exclusion to `tests/` and every baseline disappears, while
#: the map still holds its entries and the summary still prints PASS. Measured
#: at `8816eab5`, which had no such literal: widening one exclusion to `tests/`
#: printed a summary claiming ZERO baselines were considered and eleven were
#: gated, and exited 0. Three numbers in one line, one of them contradicting
#: the others, and nothing compared them. (Quoted as a shape rather than
#: verbatim, because the print string has since been reworded and the counts
#: have moved -- a "measured" line spliced from two versions is exactly what
#: #879 is about.)
_DECLARED_BASELINE_COUNT = 20

#: Committed JSON/JSONL under `tests/` that is an INPUT, not a recorded
#: measurement. Full-path regexes, each with its reason.
#:
#: Regexes rather than prefixes because a bare prefix widens silently:
#: `tests/stress/manifest_set` also exempted a hypothetical
#: `manifest_set_results_2026.json`, which would be a measurement. The pattern
#: says what the shape actually is.
_NOT_A_BASELINE = (
    (r'tests/stress/manifest_set\d+(monster)?\.json$',
     'stress-corpus manifests -- set1..set28 plus the three `monster` sets: '
     'they name the boards to fetch and record nothing'),
    (r'tests/stress/modal_sweep/arms\.[A-Za-z0-9_]+\.json$',
     'sweep ARM configurations, an input to a study. Deliberately not the '
     'whole modal_sweep/ directory: board_value.json and '
     'retry_shape_2127.json in it are recorded measurements, and a directory '
     'prefix quietly exempted both'),
    (r'tests/fixtures/.*\.jsonl?$',
     'fixture inputs a test feeds in, not measurements it took'),
)


def _names_in_live_code(src, needle):
    """Does `needle` appear in a string this file actually EVALUATES?

    `-> (answer, why_not)`, with `answer is None` when the file does not parse.

    `_code_only` strips comments and the docstring in `body[0]`, which is the
    right question for the path scanners above but not strong enough here: a
    file whose ONLY mention is `__doc__ = "...I guard tests/x.json..."`, or a
    second bare string statement after the real docstring, or an f-string used
    as a statement, survived it -- and one such file, opening nothing and
    asserting nothing, certified a baseline. So exclude every string used as a
    STATEMENT (a bare string is never functional code, wherever it sits) and
    every assignment to `__doc__`, and refuse a file that will not parse.

    What this proves is that the registration is not PROSE. It does not prove
    the gate is effective -- `note = 'x.json'` binds a name and nothing more,
    and no static rule separates that from `ROWS = os.path.join(D, 'x.json')`.
    That residual is why the map's own docstring claims "reads it and fails
    when it is wrong" rather than anything stronger, and why the mutation
    battery, not this check, is the evidence a gate can go red.
    """
    try:
        tree = ast.parse(src)
    except SyntaxError:
        return None, 'it does not parse'
    inert = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Expr) and isinstance(node.value,
                                                     (ast.Constant,
                                                      ast.JoinedStr)):
            inert.add(id(node.value))
        targets = []
        if isinstance(node, ast.Assign):
            targets = node.targets
        elif isinstance(node, (ast.AnnAssign, ast.AugAssign)):
            targets = [node.target]
        if any(getattr(t, 'id', getattr(t, 'attr', None)) == '__doc__'
               for t in targets) and node.value is not None:
            inert.add(id(node.value))
    for node in ast.walk(tree):
        if (isinstance(node, ast.Constant) and isinstance(node.value, str)
                and id(node) not in inert and needle in node.value):
            return True, ''
    return False, 'it names the file only in a docstring, a bare string or a '\
                  '`__doc__` assignment, if at all'


#: Directories the walk always skips, whatever git says about them.
_ALWAYS_SKIP = ('__pycache__', '.pytest_cache')


def _ignored_dirnames(dirpath):
    """Directory names a `.gitignore` IN `dirpath` excludes from `dirpath`.

    Only unambiguous entries -- a bare name, or `name/`. No globs, no `!`
    negation, no embedded path. Anything this cannot parse is left IN the walk,
    so an unhandled rule makes the check louder, never quieter.

    This exists because `os.walk` sees gitignored OUTPUT directories, which
    makes the result depend on which gates the machine has run.
    `tests/gui_parity/.gitignore` holds `work/`, and both
    `replay_plan_vs_run.py` and `test_gui_engine_parity.py` write `.json` into
    it -- so on a tree where CLAUDE.md's own parity instructions have been
    followed, this check went red about files git deliberately ignores. That is
    the `_WK_DEPENDENT` defect class inverted, and "delete the scratch file" is
    the wrong advice when the file is a gate's workdir.

    STRICTLY LOCAL, and that is the correction to the first cut. It also read
    the repo-root `.gitignore` and applied those 20 bare names -- `lib`,
    `parts`, `var`, `build`, `dist`, `wk`, ... -- at EVERY level under tests/.
    Measured: `git add -f tests/lib/probe_recorded_baseline.json` gave a
    TRACKED file that `git check-ignore` says is not ignored, and this check
    neither reported nor counted it, so `_DECLARED_BASELINE_COUNT` stayed
    green too. Git's ignore rules never apply to tracked files; a name-matcher
    has no way to know that, so it does not get to guess. A `.gitignore` next
    to the directory it names is the only claim specific enough to act on.
    """
    out = set()
    path = os.path.join(dirpath, '.gitignore')
    if not os.path.isfile(path):
        return out
    with open(path, encoding='utf-8', errors='replace') as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith('#') or line.startswith('!'):
                continue
            name = line.rstrip('/')
            if name and not set(name) & set('/*?[]'):
                out.add(name)
    return out


def test_every_committed_baseline_is_declared():
    """A committed baseline is either re-derived by a test, or says why not.

    Three failures, each meaning something different:

      unlisted -- a new baseline arrived and nobody decided how it stays
                  honest. (Or a scratch .json is sitting in tests/, which is
                  also worth being told about.)
      stale    -- the map names a gate that does not exist, or one that never
                  mentions the file it claims to guard. Both are worse than no
                  map: they certify nothing while looking like they do.
      count    -- the number of UNGATED baselines moved and nobody re-stated
                  it.
    """
    problems = []
    for path, gate in sorted(_BASELINE_GATES.items()):
        if not os.path.isfile(os.path.join(ROOT, path)):
            problems.append(f'{path}: registered, but the file is gone')
            continue
        gate_full = os.path.join(ROOT, gate)
        if not os.path.isfile(gate_full):
            problems.append(f'{path}: its gate {gate} does not exist')
            continue
        # A gate must be a TEST, under tests/, named so `run_all`'s glob can
        # find it. Without this, any file in the repo that happens to contain
        # the basename qualifies -- and a docs page did.
        if not (gate.startswith('tests/')
                and os.path.basename(gate).startswith('test_')
                and gate.endswith('.py')):
            problems.append(
                f'{path}: its gate {gate} is not a tests/**/test_*.py, so '
                f'nothing runs it as a gate')
            continue
        # THIS file can never be the gate. It names every baseline -- that is
        # what a registry is -- so the "does the gate mention it" check below
        # is satisfied trivially by pointing an entry here, and the entry then
        # certifies nothing. Found by mutating an entry to do exactly that and
        # watching the row survive.
        if os.path.abspath(gate_full) == os.path.abspath(__file__):
            problems.append(
                f'{path}: registered against this registry, which declares '
                f'every baseline and re-derives none. Name the test that '
                f're-computes it, or move it to _BASELINE_UNGATED')
            continue
        # The gate must NAME the file. A pairing nobody checks survives the
        # gate being rewritten to guard something else entirely.
        with open(gate_full, encoding='utf-8', errors='replace') as fh:
            src = fh.read()
        # The gate must name the file in CODE it evaluates. Measured, all
        # three of these registered a gate that certifies nothing and a raw
        # substring test accepted every one: `test_789_slate_harness.py` names
        # `placement_rule1_withdrawal.json` in its module docstring, and
        # `reseat.py` and `docs/placement-optimization.md` name
        # `836_flip_vs_reseat_baseline.json` in a docstring and in prose.
        # Comments are the other half of the same trap -- a comment quoting
        # code has satisfied a source-grep test in this repo before.
        live, why_not = _names_in_live_code(src, os.path.basename(path))
        if not live:
            problems.append(
                f'{path}: {gate} cannot support the registration -- '
                f'{why_not}. Prose is not a gate; name the file in code '
                f'that reads it')

    for path, reason in sorted(_BASELINE_UNGATED.items()):
        if not os.path.isfile(os.path.join(ROOT, path)):
            problems.append(f'{path}: listed as ungated, but the file is gone')
        if not (reason or '').strip():
            problems.append(f'{path}: listed as ungated with no reason given')

    for pattern, why in _NOT_A_BASELINE:
        if not (why or '').strip():
            problems.append(f'{pattern}: excluded with no reason given. An '
                            f'exclusion is a claim like any other')

    declared = set(_BASELINE_GATES) | set(_BASELINE_UNGATED)
    both = set(_BASELINE_GATES) & set(_BASELINE_UNGATED)
    if both:
        problems.append(f'listed as both gated and ungated: {sorted(both)}')

    # .jsonl too, and case-insensitively. The first cut matched `.json` only,
    # and three committed .jsonl measurements were invisible to it -- one of
    # them, `553_diagnosis_recall_rows.jsonl`, with zero references repo-wide.
    # A registry that cannot see the most exposed file in the tree is the #879
    # bug wearing a green tick.
    seen = set()
    for dirpath, dirnames, filenames in os.walk(TESTS_DIR):
        skip = set(_ALWAYS_SKIP) | _ignored_dirnames(dirpath)
        dirnames[:] = [d for d in sorted(dirnames) if d not in skip]
        for fname in sorted(filenames):
            if not fname.lower().endswith(('.json', '.jsonl')):
                continue
            rel = os.path.relpath(os.path.join(dirpath, fname),
                                  ROOT).replace(os.sep, '/')
            if any(re.fullmatch(pat, rel) for pat, _why in _NOT_A_BASELINE):
                continue
            seen.add(rel)
            if rel not in declared:
                problems.append(
                    f'{rel}: a committed baseline nothing declares. Register '
                    f'it against the test that re-derives it, or in '
                    f'_BASELINE_UNGATED with the reason it has none. (A '
                    f'scratch file? Delete it.)')

    # The other direction, and the reason the count below can be trusted: a
    # declared entry the walk never REACHES is a dead registration. `isfile`
    # does not catch it -- widen an exclusion to cover a listed baseline and
    # the entry stays green while guarding a file nothing looks at any more.
    for rel in sorted(declared - seen):
        problems.append(
            f'{rel}: declared, but the walk never reaches it. Either it is '
            f'outside tests/, or a _NOT_A_BASELINE pattern now swallows it -- '
            f'so its registration guards nothing')

    if len(_BASELINE_UNGATED) != _UNGATED_BASELINE_COUNT:
        problems.append(
            f'_UNGATED_BASELINE_COUNT says {_UNGATED_BASELINE_COUNT}, the map '
            f'holds {len(_BASELINE_UNGATED)}. Re-state the literal '
            f'deliberately -- an ungated baseline is a decision, not a line')
    if len(seen) != _DECLARED_BASELINE_COUNT:
        # Name the DIRECTION. The two cases have opposite remedies, and a
        # message that assumes the shrinking one sends a reader hunting for an
        # exclusion that does not exist.
        if len(seen) < _DECLARED_BASELINE_COUNT:
            why = ('a SHRINKING scope is how this check goes vacuous while '
                   'still printing PASS -- find the exclusion or the pruned '
                   'directory that swallowed one')
        else:
            why = ('the scope GREW -- a baseline arrived, so re-state the '
                   'literal once you have registered it')
        problems.append(
            f'_DECLARED_BASELINE_COUNT says {_DECLARED_BASELINE_COUNT}, the '
            f'walk considered {len(seen)}. {why}')

    assert not problems, ('committed-baseline registry:\n  '
                          + '\n  '.join(problems))
    print(f'  PASS: {len(seen)} committed baseline(s) declared -- '
          f'{len(_BASELINE_GATES)} read by a named gate, '
          f'{len(_BASELINE_UNGATED)} ungated with a stated reason')


TESTS = [
    test_wk_dependent_tests_are_declared,
    test_no_test_spawns_a_script_that_moved,
    test_the_scanners_still_match_something,
    test_no_module_scope_posix_only_import,
    test_no_test_is_defined_after_its_own_runner,
    test_every_test_is_registered_in_its_files_own_list,


    test_every_committed_baseline_is_declared,
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
