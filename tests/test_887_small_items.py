#!/usr/bin/env python3
"""#887, small defects of one shape: something DECLARED that nothing reads.

  * Pillow was a hard module-scope import declared in neither
    `requirements.txt` nor `startup_checks` -- and then, briefly, declared in
    the WRONG one: the routing gate, which does not need it.
  * The RUNBOOK's timing audit was a subagent's job, not a runnable command.

A substring check would pass TODAY-AND-BEFORE-THE-FIX, because the string was
already present somewhere (`PIL` in an import line). So each gate asserts the
SHAPE -- a fenced block, a resolved distribution -- not the presence of a word.
"""
import ast
import os
import re
import subprocess
import sys

RUN_ALL_FAST_OK = True

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)

REQS = os.path.join(ROOT, 'requirements.txt')
RUNBOOK = os.path.join(ROOT, 'tests', 'stress', 'RUNBOOK.md')

BAD = []


def want(cond, label, extra=''):
    if cond:
        print('  PASS: %s' % label)
    else:
        BAD.append(label)
        print('  FAIL: %s %s' % (label, extra))


# ---------------------------------------------------------------- the RUNBOOK

_FENCE = re.compile(r'```(?:bash|sh)\n(.*?)```', re.S)


def _fenced_tools(text):
    """Every `*.py` invoked inside a fenced bash block, in order."""
    out = []
    for block in _FENCE.findall(text):
        for m in re.finditer(r'(\S+\.py)', block):
            out.append(os.path.basename(m.group(1)))
    return out


def test_the_runbook_names_the_audit_tool_not_a_subagent():
    sec = open(RUNBOOK, encoding='utf-8').read()
    want('cmd_timing.py' in _fenced_tools(sec),
         'the RUNBOOK gives the audit as a runnable command, so the instruction '
         'and the tool cannot drift apart', )


# ---------------------------------------------------------- 3. requirements

def _declared_distributions():
    out = set()
    for line in open(REQS, encoding='utf-8'):
        line = line.split('#', 1)[0].strip()
        if line:
            out.add(re.split(r'[<>=!~\[]', line, 1)[0].strip())
    return out


#: distribution name -> the module it provides, where they differ.
DIST_TO_MODULE = {'Pillow': 'PIL'}

#: awx too since #1355: its bus step imported ortools at module scope in three
#: files, declared nowhere, and the step laid nothing without it.
SRC_DIRS = ('py_router', 'py_tools', 'py_placer', 'awx')

#: A module imported at module scope ONLY by the tool it is the runtime of,
#: which cannot start without it: awx's Modal launchers are run BY `modal run`.
LAUNCHER_ONLY = {'modal': ('awx/modal_curve.py', 'awx/modal_k.py', 'awx/modal_whole.py')}


def _repo_local_names():
    """Module names this repo provides itself, discovered from the filesystem.

    Includes COMPILED extensions. `grid_router` is `rust_router/grid_router.pyd`
    and is imported bare at module scope in six files; a scanner blind to
    binaries would report six false failures and get itself deleted.
    """
    names = set()
    for root, dirs, files in os.walk(ROOT):
        # RELATIVE to ROOT, not the absolute path: the dot-prefix skip is meant
        # to drop `.git`/`.claude` INSIDE the repo, and splitting the absolute
        # path applies it to the checkout's own location too. A repo checked out
        # under any dot-directory then yields ZERO local names and every
        # first-party import reads as an undeclared third-party one -- measured
        # as ~70 false failures in a worktree, which is exactly where
        # `EnterWorktree` puts one (`.claude/worktrees/<name>`).
        rel = os.path.relpath(root, ROOT).replace('\\', '/')
        if rel != '.' and any(part.startswith('.') or
                              part in ('__pycache__', 'target')
                              for part in rel.split('/')):
            continue
        for d in dirs:
            if os.path.isfile(os.path.join(root, d, '__init__.py')):
                names.add(d)
        for f in files:
            if f.endswith('.py'):
                names.add(f[:-3])
            elif f.endswith(('.pyd', '.so', '.dylib')):
                names.add(re.split(r'[.-]', f, 1)[0])
    return names


def _module_scope_imports(path):
    """Top-level import names in one file (level 0 only, no relative)."""
    try:
        tree = ast.parse(open(path, encoding='utf-8').read())
    except SyntaxError:
        return []
    out = []
    for node in tree.body:
        if isinstance(node, ast.Import):
            out += [a.name.split('.')[0] for a in node.names]
        elif isinstance(node, ast.ImportFrom):
            if node.level == 0 and node.module:
                out.append(node.module.split('.')[0])
        elif isinstance(node, ast.Try):
            # A module-scope try/except ImportError is a DELIBERATE optional
            # dependency, not an undeclared one. Skip it.
            continue
    return out


def test_every_module_scope_third_party_import_is_declared():
    """The gate that catches the NEXT one, not just Pillow.

    Nothing in tests/ read requirements.txt before this, which is how a hard
    module-scope import went undeclared long enough for a fresh clone to be
    unable to render anything.
    """
    stdlib = set(getattr(sys, 'stdlib_module_names', ()))
    local = _repo_local_names()
    declared = {DIST_TO_MODULE.get(d, d).lower() for d in _declared_distributions()}
    checked, undeclared = 0, {}
    for d in SRC_DIRS:
        base = os.path.join(ROOT, d)
        for root, _dirs, files in os.walk(base):
            # (a run's scratch, awx/tmp, is not the source)
            if '__pycache__' in root or 'tmp' in os.path.relpath(root, ROOT).replace('\\', '/').split('/'):
                continue
            for f in files:
                if not f.endswith('.py'):
                    continue
                p = os.path.join(root, f)
                for name in _module_scope_imports(p):
                    checked += 1
                    if (name in stdlib or name in local or name.startswith('_')
                            or name.lower() in declared
                            or os.path.relpath(p, ROOT).replace('\\', '/') in LAUNCHER_ONLY.get(name, ())):
                        continue
                    undeclared.setdefault(name, []).append(
                        os.path.relpath(p, ROOT))
    # Anti-vacuity: a scanner that walked nothing would report no problems.
    want(checked >= 30,
         'the scanner actually walked the source (%d module-scope imports)'
         % checked, checked)
    want(not undeclared,
         'every third-party module imported at module scope is declared in '
         'requirements.txt', {k: v[:2] for k, v in undeclared.items()})


def test_the_requirements_file_declares_pillow():
    dists = _declared_distributions()
    want('Pillow' in dists, 'Pillow is declared', sorted(dists))
    txt = open(REQS, encoding='utf-8').read()
    want('route_render' in txt and 'render_placement' in txt,
         'and the comment names the file that imports it at module scope AND '
         'the one that stopped doing so, so the next reader can check the '
         'claim against the source')
    want('OPTIONAL_PACKAGES' in txt,
         'and says where it is declared OPTIONAL (#943), because "declared in '
         'requirements.txt" now means two different things -- installed by the '
         'install path, and NOT a gate')


#: Blocks PIL for the child only, by refusing it at the import hook rather than
#: by uninstalling anything -- so this runs on a machine that HAS Pillow, which
#: is the machine anyone will run it on.
_NO_PIL = (
    "import builtins, sys\n"
    "_real = builtins.__import__\n"
    "def _blocked(name, *a, **k):\n"
    "    if name == 'PIL' or name.startswith('PIL.'):\n"
    "        raise ImportError(\"No module named 'PIL'\")\n"
    "    return _real(name, *a, **k)\n"
    "builtins.__import__ = _blocked\n"
)


def test_startup_checks_reports_pillow_from_the_RENDER_gate():
    """requirements.txt is documentation; startup_checks is what a user hits.

    Behavioural, not a substring: the first cut of this gate asserted only that
    `missing.append('Pillow')` appeared SOMEWHERE in startup_checks.py, which is
    true whichever function the line sits in -- so it could not see the defect
    that actually shipped, Pillow being probed by the ROUTING gate.
    """
    r = subprocess.run(
        [sys.executable, '-X', 'utf8', '-c', _NO_PIL + (
            "import startup_checks as S\n"
            "try:\n"
            "    S.check_python_dependencies(); print('ROUTING-OK')\n"
            "except S.StartupCheckError as e: print('ROUTING-REFUSED', e)\n"
            "try:\n"
            "    S.check_render_dependencies(); print('RENDER-OK')\n"
            "except S.StartupCheckError as e:\n"
            "    print('RENDER-REFUSED', 'Pillow' in str(e))\n")],
        cwd=os.path.join(ROOT, 'py_router'), capture_output=True, text=True)
    out = r.stdout
    want('RENDER-REFUSED True' in out,
         'the RENDER gate reports Pillow by name, so a fresh clone gets the '
         'actionable message rather than a runtime ImportError string', out)
    want('ROUTING-OK' in out,
         'and the ROUTING gate does NOT -- routing needs no Pillow, and '
         'probing it there made route.py/route_diff.py/route_planes.py/'
         'repair_planes.py refuse to start on the Modal corpus image, whose '
         '_PY_PINS install numpy/scipy/shapely and not Pillow', out)


def _module_scope_gate_call(path):
    """Position of a module-scope CALL to check_render_dependencies(), or -1.

    Read off the AST, never `src.find` (#943). The string also occurs in prose:
    render_placement.py's comment explains that route_render calls the gate, and
    a substring search accepted that comment as the call -- reporting the gate
    satisfied by a file that no longer invokes it, which is precisely the
    "defined and never called" shape this file exists to catch.
    """
    src = open(path, encoding='utf-8').read()
    tree = ast.parse(src)
    for node in tree.body:
        if not isinstance(node, ast.Expr):
            continue
        call = node.value
        if not isinstance(call, ast.Call):
            continue
        fn = call.func
        name = (fn.id if isinstance(fn, ast.Name)
                else fn.attr if isinstance(fn, ast.Attribute) else None)
        if name == 'check_render_dependencies':
            return node.lineno
    return -1


def test_the_render_gate_is_called_before_the_import_it_guards():
    """A gate defined and never called is the shape this whole file is about.

    route_render.py imports PIL at module scope with no fallback, so the call
    has to come FIRST or the bare ImportError wins the race and the message
    never prints.

    render_placement.py is the OTHER arm (#943): it must not reach the raster
    stack at module scope at all -- not PIL, and not route_render either, whose
    own module-scope gate would fire just the same. It is imported for
    PlacementModel / legality_findings by board_context.py and the stress
    predictors, which grade a placement and draw nothing, so a gate here made
    Pillow a requirement of grading.
    """
    rr = os.path.join(ROOT, 'py_router', 'route_render.py')
    src = open(rr, encoding='utf-8').read()
    call = _module_scope_gate_call(rr)
    pil = next((n.lineno for n in ast.parse(src).body
                if isinstance(n, ast.ImportFrom) and n.module == 'PIL'), -1)
    want(call != -1 and pil != -1 and call < pil,
         'route_render.py calls the render gate before importing PIL',
         'call=%s pil=%s' % (call, pil))

    rp = os.path.join(ROOT, 'py_tools', 'render_placement.py')
    ms = set(_module_scope_imports(rp))
    want(not ms & {'PIL', 'route_render'},
         'render_placement.py reaches the raster stack only inside the '
         'functions that draw, so grading a placement does not need Pillow',
         sorted(ms & {'PIL', 'route_render'}))
    want(_module_scope_gate_call(rp) == -1,
         'and it does not call the render gate at module scope either -- that '
         'would refuse the import for the same reason',
         _module_scope_gate_call(rp))


def test_the_gui_dep_check_mirrors_the_routing_gate_and_no_more():
    """The GUI re-implements `check_python_dependencies` BY HAND (CLAUDE.md's
    Class-2 drift). Mirroring it means the same list -- including not carrying
    an entry the CLI gate does not have, which would refuse a routing dialog on
    a board it can route."""
    src = open(os.path.join(ROOT, 'kicad_routing_plugin', 'swig_gui.py'),
               encoding='utf-8').read()
    want("missing.append('Pillow')" not in src,
         'the GUI routing dialog does not block on Pillow either (routing '
         'draws nothing)')

    # ...and the file that ACTUALLY decides, which this test did not read
    # until #943. `deps_check` derives its list from requirements.txt, so
    # declaring Pillow there gated the whole plugin on it -- past a check whose
    # stated subject is "the GUI does not block on Pillow either". Reading only
    # the hand-written list could not see a list that is generated.
    dc = os.path.join(ROOT, 'kicad_routing_plugin', 'deps_check.py')
    optional = set()
    for node in ast.parse(open(dc, encoding='utf-8').read()).body:
        if (isinstance(node, ast.Assign)
                and any(isinstance(t, ast.Name) and t.id == 'OPTIONAL_PACKAGES'
                        for t in node.targets)
                and isinstance(node.value, ast.Dict)):
            optional = {k.value for k in node.value.keys
                        if isinstance(k, ast.Constant)}
    want('Pillow' in optional,
         'and deps_check -- which builds its list FROM requirements.txt, so it '
         'is the one that actually gates the plugin -- declares Pillow '
         'optional (behaviour covered by test_943_optional_render_dependency)',
         sorted(optional))


TESTS_TO_RUN = [
    test_the_runbook_names_the_audit_tool_not_a_subagent,
    test_every_module_scope_third_party_import_is_declared,
    test_the_requirements_file_declares_pillow,
    test_startup_checks_reports_pillow_from_the_RENDER_gate,
    test_the_render_gate_is_called_before_the_import_it_guards,
    test_the_gui_dep_check_mirrors_the_routing_gate_and_no_more,
]


def main():
    for fn in TESTS_TO_RUN:
        print('--- %s' % fn.__name__)
        try:
            fn()
        except Exception as exc:                            # noqa: BLE001
            import traceback
            BAD.append('%s RAISED %s' % (fn.__name__, exc))
            traceback.print_exc()
    if BAD:
        print('\nFAILED: %d' % len(BAD))
        for b in BAD:
            print('  - %s' % b)
        return 1
    print('\nALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
