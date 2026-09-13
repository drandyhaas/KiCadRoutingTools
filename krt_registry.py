#!/usr/bin/env python3
"""Every runnable tool in this clone, what it is for, and which door it serves.

    python3 -X utf8 krt_registry.py --door routing
    python3 -X utf8 krt_registry.py --list
    python3 -X utf8 krt_registry.py --json
    python3 -X utf8 krt_registry.py --check

#937's anti-deletion guarantee: a restructure cannot quietly drop a tool,
because `tests/test_937_tool_registry.py` fails when a runnable tool has no
entry. The three catalogues that existed before this one each answered a
different question and none was complete -- `krt_capabilities.KNOWN_MODULES`
is a hand-written tuple of 29 (one of which is not a CLI), `FLAG_SCRIPTS`
covers 8, and `test_431.discovered_tools()` finds the 49 the skills happen to
invoke. Nothing asserted the reverse direction: that every CLI in the tree is
in the catalogue.

WHAT IS DERIVED, AND WHY EACH IS DERIVED RATHER THAN DECLARED
-------------------------------------------------------------
`purpose`  the module docstring's first line. All runnable tools have one
           (median body ~1.2 kB, no two first lines alike), so copying them
           into a table would create a second copy to drift. Reading is a
           stronger guarantee than any gate over a copy.
`doc`      a `## Name (`tool.py`)` heading in docs/utilities.md. An absent
           pointer is a recorded FACT and the backlog, never a gate failure --
           23 of 71 have one today, and failing on the other 48 would only
           get the gate deleted.
`flags`    left to `krt_capabilities.script_flags`, unchanged. One resolver.

WHAT IS DECLARED, AND WHY IT CANNOT BE DERIVED
-----------------------------------------------
`scope`    which door(s) the tool serves. NOT DERIVABLE, and that claim is
           not frozen here -- `tests/test_937_tool_registry.py` RE-MEASURES
           the two candidate rules every run and fails if either ever became
           complete and they agreed, because then this field should be
           retired rather than maintained. The rules measure different
           questions: the directory is a PROVENANCE fact (the #522 reorg) and
           the skills are a USAGE fact. One hard contradiction:
           py_placer/place_fanout_clearance.py lives in py_placer/ and is
           named only by plan-pcb-routing -- it is declared for BOTH doors,
           wider than either rule, because a routing chain runs it and it is
           the one placement step that lays copper.
`kind`     The best derivation available -- the union of `record_invocation`,
           a writer-symbol grep and an output argument -- has false positives
           and misses, and two of the misses are invisible to ANY scan of the
           file itself: py_router/place_fanout_clearance.py is a `runpy`
           forwarder, and py_router/fix_kicad_drc_settings.py rewrites the
           board IN PLACE and so has no output argument by construction.
           `record_invocation` alone is the precise half -- 17 call sites, 17
           of them actors -- so it is kept as a CROSS-CHECK (see
           `self_records`) rather than promoted to a derivation.

A MISSING DECLARATION IS A GATE FAILURE, NOT AN EXEMPTION. That is the whole
inversion: this repo's recorded failure mode is that the exempted names are
where the bug hides, so the registry has no allowlist. A tool that should not
be in a door's view says so -- `'scope': []` -- rather than being omitted.

Declarations are read BY AST, NEVER BY IMPORT, for the reason
`krt_capabilities.script_flags` gives for doing the same with flags: a
consumer asking "can this clone do X" must not be able to trigger a side
effect by asking.

THE PREDICATE IS BEHAVIOURAL, AND IT NAMES ITS OWN BUGS
--------------------------------------------------------
A tool is runnable when `python3 <file> --help` exits 0 and prints a line
starting with `usage:`. It costs ~11 s over the 240 tracked non-test files at
8 workers (~86 s of subprocess time) and yields 71.

It is a strict superset of the 49 tools `test_431.discovered_tools()` finds
by scanning what the skills invoke -- `discovered-only` is empty -- so nothing
that gate already covers is lost by adopting this one.

`grep __main__` is wrong in both directions and an allowlist would be needed
to patch it: four modules mention it only in a docstring or have a non-CLI
`__main__`, two package `__init__.py` files fail on sys.path when run
directly but are real CLIs through their shims -- and three tools FAILED the
predicate for reasons that were real bugs, fixed in the commit before this
one rather than exempted. That is the property being bought: the predicate
names its own false negatives instead of needing a list of them.
"""
import argparse
import ast
import concurrent.futures
import json
import os
import re
import subprocess
import sys

#: This module is itself a runnable tool, so it declares like every other one.
KRT_TOOL = {'scope': [], 'kind': 'utility'}

ROOT = os.path.dirname(os.path.abspath(__file__))

#: Every door a `scope` may name. A declaration naming anything else is a gate
#: failure -- a typo'd door is a tool that silently appears in no view, which
#: is the failure this registry exists to prevent.
DOORS = ('placement', 'routing', 'combined')

#: Every value `kind` may take.
#:
#: `actor` / `instrument` is the repo owner's split and the one that decides
#: how a tool is used: an actor changes the board and must be SCOPED to what
#: you already measured; an instrument says what is wrong and running it is
#: never optional.
#:
#: The other three exist because forcing every tool into that pair would make
#: the registry lie, and a catalogue that lies is worse than none:
#:   `conditional` -- writes a board only under an opt-in flag
#:                    (`check_drc --debug-lines`, `check_join --keep-staged`)
#:                    or only to a scratch copy (`plane_score`). Calling these
#:                    actors tells a reader to scope something that reports.
#:   `driver`      -- emits workflow instructions and touches no board. The
#:                    two tape heads. `instrument` would be true of the board
#:                    and false of the thing.
#:   `utility`     -- not board work at all: build, release, packaging, clone
#:                    hygiene. These carry `scope: []` and that is a
#:                    DECLARATION, not an omission.
KINDS = ('actor', 'instrument', 'conditional', 'driver', 'utility')

#: docs/utilities.md's heading form, which is formulaic and is the only
#: per-tool documentation index in the repo. Anchored at line start so a `#`
#: inside a fenced block cannot match, and `$`-anchored so the two
#: non-tool sections (`## Test Scripts`, `## Common Workflows`) cannot.
_DOC_HEADING = re.compile(
    r'^## (?P<title>.+) \(`(?P<tool>[A-Za-z0-9_]+\.py)`\)$', re.M)

_USAGE_PROBE_TIMEOUT = 40
_PROBE_WORKERS = 8


def tracked_python(root=ROOT):
    """Every tracked .py outside tests/, repo-relative, sorted.

    `git ls-files`, not a walk: an untracked scratch file in the tree is not
    part of this clone's catalogue, and a walk would put one there.
    """
    r = subprocess.run(['git', 'ls-files', '*.py'], cwd=root,
                       capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(f'git ls-files failed in {root}: {r.stderr}')
    return sorted(p.strip() for p in r.stdout.splitlines()
                  if p.strip() and not p.strip().startswith('tests/'))


def _probe(args):
    """(rel, ok, detail) -- does `python3 <rel> --help` answer like a tool?"""
    root, rel = args
    try:
        r = subprocess.run(
            [sys.executable, '-X', 'utf8', os.path.join(root, rel), '--help'],
            cwd=root, capture_output=True, text=True, encoding='utf-8',
            errors='replace', timeout=_USAGE_PROBE_TIMEOUT)
    except subprocess.TimeoutExpired:
        return rel, False, f'no answer in {_USAGE_PROBE_TIMEOUT}s'
    except OSError as exc:                                    # noqa: BLE001
        return rel, False, f'could not launch: {exc}'
    if r.returncode != 0:
        tail = (r.stderr or r.stdout or '').strip().splitlines()
        return rel, False, (f'exit {r.returncode}: '
                            + (tail[-1][:120] if tail else 'no output'))
    # A LINE starting with `usage:`, not the FIRST line: 24 tools print a
    # banner (`CMD: ...`, `ENV KNOBS: ...`) before argparse gets to speak, and
    # requiring line 1 would report every one of them as not a tool.
    if not any(ln.startswith('usage:')
               for ln in (r.stdout or '').splitlines()):
        return rel, False, 'exit 0 but no `usage:` line -- not an argparse CLI'
    return rel, True, ''


def runnable_tools(root=ROOT, files=None):
    """({rel: ''} for tools, {rel: why} for the rest) over tracked_python."""
    files = list(files if files is not None else tracked_python(root))
    ok, no = {}, {}
    with concurrent.futures.ThreadPoolExecutor(
            max_workers=_PROBE_WORKERS) as ex:
        for rel, good, why in ex.map(_probe, ((root, f) for f in files)):
            (ok if good else no)[rel] = why
    return ok, no


def _module_ast(root, rel):
    with open(os.path.join(root, rel), encoding='utf-8',
              errors='replace') as fh:
        return ast.parse(fh.read(), filename=rel)


def declaration(root, rel):
    """The module-level KRT_TOOL literal, or None. AST, never an import."""
    try:
        tree = _module_ast(root, rel)
    except SyntaxError:
        return None
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        names = [t.id for t in node.targets if isinstance(t, ast.Name)]
        if 'KRT_TOOL' not in names:
            continue
        try:
            got = ast.literal_eval(node.value)
        except (ValueError, SyntaxError):
            return None
        return got if isinstance(got, dict) else None
    return None


def purpose(root, rel):
    """The docstring's first non-empty line, or '' when there is none."""
    try:
        doc = ast.get_docstring(_module_ast(root, rel))
    except SyntaxError:
        return ''
    for line in (doc or '').splitlines():
        if line.strip():
            return line.strip()
    return ''


def doc_pointers(root=ROOT):
    """{basename: heading title} from docs/utilities.md's tool sections."""
    path = os.path.join(root, 'docs', 'utilities.md')
    if not os.path.isfile(path):
        return {}
    with open(path, encoding='utf-8', errors='replace') as fh:
        text = fh.read()
    return {m.group('tool'): m.group('title')
            for m in _DOC_HEADING.finditer(text)}


def self_records(root, rel):
    """Does the tool CALL record_invocation -- i.e. self-record into the redo
    manifest?

    The cross-check for `kind`, and the cleanest single signal there is: 17
    call sites, 17 of them actors, no false positive. It does NOT decide --
    it has 7 misses, including two structurally invisible ones (a `runpy`
    forwarder and an in-place rewriter) -- so a disagreement with the
    declaration is DISCLOSED BY NAME rather than resolved. A signal that
    silently overrode a declaration would be a derivation wearing a
    declaration's clothes.

    A CALL, found by AST, not the string. The substring version reported this
    very module as a self-recorder, because it is the one that explains the
    signal -- prose satisfying a source-grep is a recorded failure here, and
    a cross-check that fires on any file DISCUSSING the thing it checks is
    worse than no cross-check.
    """
    try:
        tree = _module_ast(root, rel)
    except (OSError, SyntaxError):
        return False
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        fn = node.func
        name = (fn.id if isinstance(fn, ast.Name)
                else fn.attr if isinstance(fn, ast.Attribute) else None)
        if name == 'record_invocation':
            return True
    return False


def registry(root=ROOT, files=None):
    """[row] for every runnable tool, sorted by path.

    A row always carries `path`, `purpose`, `doc`, `self_records`; `kind` and
    `scope` are None/None when the tool declares nothing, which is what the
    gate refuses on. Nothing here raises on a missing declaration: reporting
    is this function's job and refusing is the gate's.
    """
    ok, _no = runnable_tools(root, files)
    docs = doc_pointers(root)
    rows = []
    for rel in sorted(ok):
        decl = declaration(root, rel) or {}
        scope = decl.get('scope')
        rows.append({
            'path': rel,
            'purpose': purpose(root, rel),
            'kind': decl.get('kind'),
            'scope': list(scope) if isinstance(scope, (list, tuple)) else None,
            'declared': bool(decl),
            'doc': docs.get(os.path.basename(rel)),
            'self_records': self_records(root, rel),
        })
    return rows


def door_view(rows, door):
    """The rows one door should see. A tool with no declaration is EXCLUDED
    here and caught by the gate -- a view must not quietly widen to cover a
    registration hole, because then the hole never gets fixed."""
    return [r for r in rows if r['scope'] and door in r['scope']]


def _fmt(row, width):
    kind = row['kind'] or '?'
    return (f"  {row['path']:<{width}}  {kind:<11}  {row['purpose'][:70]}")


def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--door', choices=DOORS,
                    help='only the tools this door serves')
    ap.add_argument('--list', action='store_true',
                    help='one line per tool (the default when no --json)')
    ap.add_argument('--json', dest='as_json', action='store_true',
                    help='the whole registry as a JSON document')
    ap.add_argument('--check', action='store_true',
                    help='report undeclared tools and declaration/signal '
                         'disagreements; exit 4 if any tool is undeclared')
    ap.add_argument('--root', default=ROOT,
                    help='the clone to inventory (default: this one)')
    a = ap.parse_args(argv)

    rows = registry(a.root)
    if a.door:
        rows = door_view(rows, a.door)

    if a.as_json:
        json.dump({'schema': 1, 'door': a.door, 'tools': rows},
                  sys.stdout, indent=2, sort_keys=True)
        print()
        return 0

    if a.check:
        undeclared = [r for r in rows if not r['declared']]
        badkind = [r for r in rows if r['declared']
                   and r['kind'] not in KINDS]
        badscope = [r for r in rows if r['declared']
                    and (r['scope'] is None
                         or any(s not in DOORS for s in r['scope']))]
        # The disagreement is REPORTED, never resolved: see self_records().
        # `not in (actor, conditional)` rather than `== instrument`: adding
        # `driver` and `utility` to KINDS would otherwise have opened two new
        # places for a board-writing tool to hide from this check, which is
        # the shape of every whitelist defect in this repo's notes.
        disagree = [r for r in rows if r['self_records']
                    and r['kind'] not in ('actor', 'conditional')]
        for label, bad in (('undeclared', undeclared),
                           ('kind not in ' + repr(KINDS), badkind),
                           ('scope not a list of ' + repr(DOORS), badscope)):
            print(f'{label}: {len(bad)}')
            for r in bad:
                print(f'    {r["path"]}')
        print(f'self-records but declared neither actor nor conditional: {len(disagree)}'
              '  [reported, not resolved]')
        for r in disagree:
            print(f'    {r["path"]}')
        print(f'\n{len(rows)} runnable tool(s); '
              f'{sum(1 for r in rows if r["doc"])} with a docs/utilities.md '
              f'section')
        return 4 if (undeclared or badkind or badscope) else 0

    width = max((len(r['path']) for r in rows), default=4)
    for row in rows:
        print(_fmt(row, width))
    print(f'\n{len(rows)} tool(s)'
          + (f' at the {a.door} door' if a.door else ''))
    return 0


if __name__ == '__main__':
    sys.exit(main())
