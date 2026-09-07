#!/usr/bin/env python3
"""#887, the three defects found while mapping the movie path.

Each is small; each was invisible in the way that matters. They are graded
together because they share one shape: something DECLARED that nothing reads.

  1. `place_route_loop --movie-tween` was parsed and never used.
  2. Two of the three skills never mentioned the movie at all, so a
     placement-only or routing-only run had no instruction to produce the
     artifact the combined skill calls mandatory.
  3. Pillow was a hard module-scope import declared in neither
     `requirements.txt` nor `startup_checks` -- and then, briefly, declared in
     the WRONG one: the routing gate, which does not need it.

Note the shape of these gates. A substring check would pass on all three
TODAY-AND-BEFORE-THE-FIX, because in every case the string was already present
somewhere: `movie_tween` in an `add_argument`, the word "movie" in a comment,
`PIL` in an import line. So each gate asserts the SHAPE -- an AST keyword, a
fenced block inside a named section, a resolved distribution -- not the presence
of a word.
"""
import ast
import os
import re
import subprocess
import sys

RUN_ALL_FAST_OK = True

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)

SKILLS = os.path.join(ROOT, '.claude', 'skills')
PLACE_SKILL = os.path.join(SKILLS, 'plan-pcb-placement', 'SKILL.md')
ROUTE_SKILL = os.path.join(SKILLS, 'plan-pcb-routing', 'SKILL.md')
BOTH_SKILL = os.path.join(SKILLS, 'plan-pcb-placement-and-routing', 'SKILL.md')
LOOP = os.path.join(ROOT, 'py_placer', 'place_route_loop.py')
REQS = os.path.join(ROOT, 'requirements.txt')
RUNBOOK = os.path.join(ROOT, 'tests', 'stress', 'RUNBOOK.md')

BAD = []


def want(cond, label, extra=''):
    if cond:
        print('  PASS: %s' % label)
    else:
        BAD.append(label)
        print('  FAIL: %s %s' % (label, extra))


# ------------------------------------------------------------ 1. movie-tween

def test_place_route_loop_passes_its_movie_tween_through():
    """AST, not substring. `'movie_tween' in src` was ALREADY TRUE before the
    fix -- from the add_argument line -- which is precisely the defect: argparse
    accepted the flag and nothing read it."""
    src = open(LOOP, encoding='utf-8').read()
    tree = ast.parse(src)
    calls = [n for n in ast.walk(tree)
             if isinstance(n, ast.Call)
             and isinstance(n.func, ast.Name) and n.func.id == 'make_movie']
    want(len(calls) == 1, 'there is exactly one make_movie call', len(calls))
    kw = {k.arg: k.value for k in calls[0].keywords if k.arg}
    want('tween' in kw, 'and it passes tween=', sorted(kw))
    v = kw.get('tween')
    want(isinstance(v, ast.Attribute) and v.attr == 'movie_tween',
         'sourced from args.movie_tween, not a literal that would ignore the '
         'flag just as thoroughly', ast.dump(v) if v is not None else None)


def test_the_movie_tween_flag_still_exists_to_be_passed():
    """The two halves must not drift: a passed-through flag nobody declares is
    an AttributeError, and a declared flag nobody passes is the original bug."""
    src = open(LOOP, encoding='utf-8').read()
    want('"--movie-tween"' in src or "'--movie-tween'" in src,
         'the flag is still declared')


# ---------------------------------------------------------------- 2. skills

_FENCE = re.compile(r'```(?:bash|sh)\n(.*?)```', re.S)


def _fenced_tools(text):
    """Every `*.py` invoked inside a fenced bash block, in order."""
    out = []
    for block in _FENCE.findall(text):
        for m in re.finditer(r'(\S+\.py)', block):
            out.append(os.path.basename(m.group(1)))
    return out


def _section(text, heading):
    """The text under `## heading`, up to the next `## `."""
    parts = re.split(r'(?m)^## ', text)
    for p in parts:
        if p.startswith(heading):
            return p
    return ''


def test_the_placement_skill_mandates_the_movie_with_a_runnable_command():
    text = open(PLACE_SKILL, encoding='utf-8').read()
    tools = _fenced_tools(text)
    want('make_film.py' in tools or 'make_movie.py' in tools,
         'the placement skill carries a FENCED command that builds the movie -- '
         'a fence, because prose quoting a command in backticks satisfies a '
         'naive grep and has done so in this repo before',
         [t for t in tools if 'm' in t][:8])


def test_the_routing_skill_mandates_the_movie_after_routing_completes():
    text = open(ROUTE_SKILL, encoding='utf-8').read()
    sec = _section(text, 'After Routing Completes')
    want(sec, 'the routing skill has an After Routing Completes section')
    want('make_movie.py' in _fenced_tools(sec),
         'and the movie command is fenced INSIDE it -- section-scoped, so a '
         'mention elsewhere in a 2000-line file cannot satisfy this',
         _fenced_tools(sec)[:8])


def test_the_movie_mandate_is_in_all_three_skills():
    """Asserted PER FILE. The existing skill gate concatenates all three via
    _all_skill_text(), which would pass with only the combined skill carrying
    it -- and the combined skill has carried it alone all along."""
    missing = []
    for path in (PLACE_SKILL, ROUTE_SKILL, BOTH_SKILL):
        tools = _fenced_tools(open(path, encoding='utf-8').read())
        if not ({'make_movie.py', 'make_film.py'} & set(tools)):
            missing.append(os.path.basename(os.path.dirname(path)))
    want(not missing,
         'every skill that drives a run says how to produce the movie', missing)


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

SRC_DIRS = ('py_router', 'py_tools', 'py_placer')


def _repo_local_names():
    """Module names this repo provides itself, discovered from the filesystem.

    Includes COMPILED extensions. `grid_router` is `rust_router/grid_router.pyd`
    and is imported bare at module scope in six files; a scanner blind to
    binaries would report six false failures and get itself deleted.
    """
    names = set()
    for root, dirs, files in os.walk(ROOT):
        if any(part.startswith('.') or part in ('__pycache__', 'target')
               for part in root.replace('\\', '/').split('/')):
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
            if '__pycache__' in root:
                continue
            for f in files:
                if not f.endswith('.py'):
                    continue
                p = os.path.join(root, f)
                for name in _module_scope_imports(p):
                    checked += 1
                    if (name in stdlib or name in local or name.startswith('_')
                            or name.lower() in declared):
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
         'and the comment names the two files that import it at module scope, '
         'so the next reader can check the claim')
    want('imageio' in txt,
         'while imageio stays UNdeclared, with the reason written down -- it is '
         'function-scope, optional, and its absence is audible')


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


def test_the_render_gate_is_called_before_the_import_it_guards():
    """A gate defined and never called is the shape this whole file is about.

    Both files import PIL at module scope with no fallback, so the call has to
    come FIRST or the bare ImportError wins the race and the message never
    prints.
    """
    for rel in (('py_router', 'route_render.py'),
                ('py_tools', 'render_placement.py')):
        src = open(os.path.join(ROOT, *rel), encoding='utf-8').read()
        call = src.find('check_render_dependencies()')
        pil = src.find('from PIL import')
        want(call != -1 and pil != -1 and call < pil,
             '%s calls the render gate before importing PIL' % rel[-1],
             'call=%d pil=%d' % (call, pil))


def test_the_gui_dep_check_mirrors_the_routing_gate_and_no_more():
    """The GUI re-implements `check_python_dependencies` BY HAND (CLAUDE.md's
    Class-2 drift). Mirroring it means the same list -- including not carrying
    an entry the CLI gate does not have, which would refuse a routing dialog on
    a board it can route."""
    src = open(os.path.join(ROOT, 'kicad_routing_plugin', 'swig_gui.py'),
               encoding='utf-8').read()
    want("missing.append('Pillow')" not in src,
         'the GUI routing dialog does not block on Pillow either (its only '
         'raster consumer, the movie recorder, is off by default)')


TESTS_TO_RUN = [
    test_place_route_loop_passes_its_movie_tween_through,
    test_the_movie_tween_flag_still_exists_to_be_passed,
    test_the_placement_skill_mandates_the_movie_with_a_runnable_command,
    test_the_routing_skill_mandates_the_movie_after_routing_completes,
    test_the_movie_mandate_is_in_all_three_skills,
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
