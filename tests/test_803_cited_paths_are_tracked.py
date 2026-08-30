#!/usr/bin/env python3
"""A `.md` file a skill or a doc cites as AUTHORITY must exist in git (#803).

Eleven sites in the two placement drivers cited `wk/calibration/RESULT.md` as
the reason a scoring threshold was withdrawn. `wk/` is gitignored, the file was
authored in a throwaway worktree, and it was never committed -- so the citation
resolved for nobody. Three of the eleven were **printed to the operator at
runtime**: a run emitted a refusal naming a file its reader could not open.

Nothing caught it, and nothing could have: `test_718_static_test_hygiene.py`
knows about `wk/` but only as `os.path.join` argument tuples inside test code,
and only to demand DISCLOSURE rather than existence;
`test_457_fresh_clone_fixtures.py` is the repo's only other "a cited path must
be tracked" gate and its regexes scan `kicad_files/` boards inside `tests/`.

WHY `.md` AND NOT EVERY PATH. Run `--census` to see it rather than take it on
trust: the scan is here, so the number is reproducible instead of quoted. At
the commit that added this gate it prints

    variant                          sites  resolved  unresolved
    .md only                            56        55           1
    any known extension                670       509          90
      ... minus --flag-preceded        583       509          54

An earlier version of this docstring quoted "38 candidates, 37 legitimate" and
said the `--flag` filter did not help. Both came from a throwaway probe with a
narrower path regex than this gate's, neither was reproducible from this file,
and running the real thing says otherwise on both counts: 90 unresolved at the
wide scope, and the flag filter removes 36 of them. That is exactly the defect
#803 is about -- an authority nobody can check -- committed into the gate that
exists to prevent it, which is why the census ships instead of the sentence.

What survives is the part that does reproduce: the unresolved paths at any
wider scope are overwhelmingly artifacts a run WRITES (`wk/score.json`,
`wk/ledger.jsonl`, `wk/iter03.kicad_pcb`, `wk/place.mp4`), plus doc
placeholders (`kicad_files/my_board.kicad_pcb`, `path/to/file.kicad_pcb`) --
while `.md` is something you READ, so an unresolved one is a broken citation.

SCOPE, stated because it is narrower than "a skill or a doc": SEARCH_DIRS is
`.claude/skills` and `docs`. `README.md`, `CLAUDE.md` and `tests/` are NOT
scanned, so a stale `.md` citation there still passes. Widening is wanted; it
needs the two live offenders below fixed first, and they belong to their own
issue rather than to #803's:

  * `tests/stress/rank_stats.py` cites `wk/research-placement/...md` as the
    authority for its p-values -- the same defect, still shipping;
  * `py_tools/board_brief.py` cites `.claude/skills/plan-pcb-routing/
    references/evidence-map.md`, a directory that does not exist (the file
    lives under `plan-pcb-placement-and-routing/`).

A citation counts as resolved if it is tracked when read relative to the repo
root, relative to the CITING FILE's directory (`../tests/stress/RUNBOOK.md`
from `docs/`), or as a SUFFIX of some tracked path (`placement/README.md`,
which is how the docs refer to `py_placer/placement/README.md`). Suffix
matching is deliberately lenient: this gate exists to catch a pointer that
resolves to NOTHING, not to police how a human abbreviates one.

    python3 -X utf8 tests/test_803_cited_paths_are_tracked.py
"""
import io
import os
import posixpath
import re
import subprocess
import sys

RUN_ALL_TIMEOUT = 120
#: It shells `git ls-files`, which `run_all.is_integration` reads as an
#: integration marker. It is a sub-second static scan; keep it in --fast.
RUN_ALL_FAST_OK = True

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

#: Where an AUTHORITY citation lives: the skills (SKILL.md, references, and the
#: two drivers) and the docs they point at.
SEARCH_DIRS = ('.claude/skills', 'docs')
SEARCH_EXT = ('.md', '.py')
SKIP_PARTS = ('.claude/worktrees', '__pycache__', 'wk/')

#: A repo-relative-looking `.md` path: at least one `/`, no whitespace.
#:
#: The trailing boundary excludes only `[A-Za-z0-9_-]`, NOT `.` -- and that is
#: the whole reason it is written out rather than spelled `\b`-adjacent. A
#: citation that ENDS A SENTENCE (`See docs/foo.md.`) is followed by a period,
#: and a lookahead of `(?![A-Za-z0-9_.-])` silently skips it. Measured on the
#: #803 sites: that spelling found 9 of the 11, and the two it missed were
#: exactly the two written as sentences. `t_a_sentence_final_citation_matches`
#: pins it.
MD_CITATION = re.compile(
    r'(?<![A-Za-z0-9_/.-])'
    r'([A-Za-z0-9_.-]+(?:/[A-Za-z0-9_.-]+)+\.md)'
    r'(?![A-Za-z0-9_-])')

#: Cited `.md` paths that are deliberately NOT in the tree: path -> (the ONE
#: file allowed to name it, why).
#:
#: The exemption is SITE-scoped, not path-scoped, and that is not a detail.
#: The first draft keyed on the path alone, and the mutation test caught it
#: immediately: reverting a single driver citation back to
#: `wk/calibration/RESULT.md` -- the exact #803 defect -- passed, because the
#: doc's own historical mention had already excused the name everywhere.
#:
#: Held in BOTH directions: an undeclared unresolved citation fails, a
#: citation from the wrong file fails, and a declared entry nobody cites any
#: more is reported stale -- test_718's doctrine, for its reason (a
#: one-directional map decays into a list of things that used to matter).
UNTRACKED_OK = {
    'wk/calibration/RESULT.md': (
        'docs/placement-calibration.md',
        'the historical filename that page replaces. #803 is about this file '
        'never having been committed, so the page that repairs it has to be '
        'able to name it. No other file may cite it.'),
}

#: Below this, the scanner has stopped matching and is reporting a clean tree
#: indistinguishably from a clean tree. 55 resolve at the commit that added
#: this gate.
MIN_RESOLVED = 40

FAILURES = []


def check(cond, what, detail=''):
    if cond:
        print(f'  ok   {what}')
    else:
        print(f'  FAIL {what}{detail}')
        FAILURES.append(what)


def tracked_paths():
    """Every git-tracked path, or None when git cannot answer."""
    try:
        r = subprocess.run(['git', 'ls-files', '-z'],
                           capture_output=True, text=True, cwd=ROOT)
    except OSError:
        return None
    if r.returncode != 0:
        return None
    return set(p.replace('\\', '/') for p in r.stdout.split('\0') if p)


def suffixes_of(tracked):
    """Every trailing path fragment of every tracked file."""
    out = set()
    for t in tracked:
        parts = t.split('/')
        for i in range(len(parts)):
            out.add('/'.join(parts[i:]))
    return out


def walk():
    for d in SEARCH_DIRS:
        base = os.path.join(ROOT, d)
        for dirpath, dirnames, filenames in os.walk(base):
            dirnames[:] = [x for x in dirnames
                           if x not in ('__pycache__', 'worktrees', 'target')]
            for fn in filenames:
                if not fn.endswith(SEARCH_EXT):
                    continue
                p = os.path.join(dirpath, fn)
                rel = os.path.relpath(p, ROOT).replace('\\', '/')
                if any(s in rel for s in SKIP_PARTS):
                    continue
                yield rel, p


def citations_in(rel, path):
    """(cited_path, 'rel:line') for every `.md` citation in one file."""
    try:
        src = io.open(path, encoding='utf-8', errors='replace').read()
    except OSError:
        return []
    src = src.replace('\r\n', '\n')
    out = []
    for m in MD_CITATION.finditer(src):
        line = src.count('\n', 0, m.start()) + 1
        out.append((m.group(1), f'{rel}:{line}'))
    return out


def resolves(cited, citing_rel, tracked, suffixes):
    """Repo-root, citing-file-relative, or suffix-of-a-tracked-path."""
    flat = posixpath.normpath(cited)
    if flat in suffixes:
        return True
    near = posixpath.normpath(
        posixpath.join(posixpath.dirname(citing_rel), cited))
    return near in tracked


def scan():
    """(resolved_count, {cited: [sites]}) over SEARCH_DIRS."""
    tracked = tracked_paths()
    if tracked is None:
        return None, None
    suffixes = suffixes_of(tracked)
    resolved = 0
    unresolved = {}
    for rel, path in walk():
        for cited, site in citations_in(rel, path):
            if resolves(cited, rel, tracked, suffixes):
                resolved += 1
            else:
                unresolved.setdefault(cited, []).append(site)
    return resolved, unresolved


# --------------------------------------------------------------- the checks

def t_every_cited_md_resolves(resolved, unresolved):
    offenders = []
    for cited in sorted(unresolved):
        allowed = UNTRACKED_OK.get(cited, (None, ''))[0]
        for site in unresolved[cited]:
            if allowed and site.rsplit(':', 1)[0] == allowed:
                continue
            offenders.append(f'{site}  ->  {cited}')
    check(not offenders,
          'every cited .md resolves to a git-tracked file',
          ('' if not offenders else
           '\n       A skill or doc cites a .md that is not in the tree:\n         '
           + '\n         '.join(offenders)
           + '\n       On a fresh clone that file does not exist, so the citation'
             '\n       resolves for nobody -- and a driver that PRINTS it emits a'
             '\n       refusal naming a file its reader cannot open. Commit the'
             '\n       document, repoint the citation, or declare it in'
             '\n       UNTRACKED_OK with the reason.'))


def t_untracked_ok_has_no_stale_entry(unresolved):
    stale = sorted(set(UNTRACKED_OK) - set(unresolved))
    check(not stale,
          'no UNTRACKED_OK entry has stopped being cited',
          ('' if not stale else
           f'\n       STALE (nobody cites these any more; drop them): {stale}'))


def t_the_scanner_still_matches_something(resolved):
    check(resolved >= MIN_RESOLVED,
          f'the scanner still resolves >= {MIN_RESOLVED} citations',
          ('' if resolved >= MIN_RESOLVED else
           f'\n       only {resolved} matched. A regex that has quietly stopped'
           '\n       matching reports a clean tree indistinguishably from a'
           '\n       tree that IS clean.'))


def t_a_sentence_final_citation_matches():
    """The boundary bug that cost 2 of the 11 sites in the first draft.

    `See docs/foo.md.` -- a citation ending a sentence -- must match. So must
    one in a parenthetical, in backticks, and mid-list. A bare filename with no
    directory must NOT, and neither must a longer word that merely ends in it.
    """
    positive = (
        'See docs/placement-calibration.md.',
        '# withdrawn (docs/placement-calibration.md): the premise inverts',
        'quoted `py_placer/placement/README.md` inline',
        'a, docs/one.md, b',
        'end of docs/two.md',
    )
    negative = (
        'README.md on its own',
        'see the docs/three.mdx bundle',
        'docs/four.md_backup is not a citation',
    )
    bad = []
    for s in positive:
        if not MD_CITATION.search(s):
            bad.append(f'MISSED: {s!r}')
    for s in negative:
        if MD_CITATION.search(s):
            bad.append(f'FALSE POSITIVE: {s!r}')
    check(not bad, 'the citation regex matches the shapes it must',
          ('' if not bad else '\n       ' + '\n       '.join(bad)))


def t_the_gate_would_catch_the_803_shape(tracked, suffixes):
    """Negative control: an invented untracked citation must NOT resolve.

    Without this, a `resolves()` that answered True for everything would pass
    every other check in this file.
    """
    invented = 'wk/calibration/no-such-write-up.md'
    check(not resolves(invented, 'docs/x.md', tracked, suffixes),
          'an untracked cited path does not resolve (negative control)')
    check(resolves('docs/placement-predictors.md', 'docs/x.md',
                   tracked, suffixes),
          'a tracked cited path does resolve (positive control)')


TESTS = [
    't_every_cited_md_resolves',
    't_untracked_ok_has_no_stale_entry',
    't_the_scanner_still_matches_something',
    't_a_sentence_final_citation_matches',
    't_the_gate_would_catch_the_803_shape',
]


#: The wider regexes the census compares against. Same resolver, same walk,
#: same skips -- only the extension filter differs, so the comparison is
#: apples to apples.
_ANY_EXT = re.compile(
    r'(?<![A-Za-z0-9_/.-])'
    r'([A-Za-z0-9_.-]+(?:/[A-Za-z0-9_.-]+)+\.[A-Za-z0-9]{1,6})'
    r'(?![A-Za-z0-9_-])')
_FLAG_BEFORE = re.compile(r'--[A-Za-z0-9-]+[= ] *$|> *$|-o +$')


def census():
    """Print the measurement that justifies the `.md` scope. Reproducible."""
    tracked = tracked_paths()
    if tracked is None:
        print('SKIP: git ls-files unavailable')
        return 77
    suffixes = suffixes_of(tracked)
    rows = []
    for label, rx, drop_flagged in (
            ('.md only', MD_CITATION, False),
            ('any known extension', _ANY_EXT, False),
            ('  ... minus --flag-preceded', _ANY_EXT, True)):
        ok = 0
        bad = {}
        for rel, path in walk():
            src = io.open(path, encoding='utf-8',
                          errors='replace').read().replace('\r\n', '\n')
            for m in rx.finditer(src):
                if drop_flagged and _FLAG_BEFORE.search(
                        src[max(0, m.start() - 40):m.start()]):
                    continue
                cited = m.group(1)
                if resolves(cited, rel, tracked, suffixes):
                    ok += 1
                else:
                    bad.setdefault(cited, []).append(rel)
        rows.append((label, ok + sum(len(v) for v in bad.values()), ok,
                     len(bad)))
    print('#803 census -- why this gate is scoped to .md')
    print(f'  {"variant":30s} {"sites":>7s} {"resolved":>9s} '
          f'{"unresolved":>11s}')
    for label, sites, ok, un in rows:
        print(f'  {label:30s} {sites:7d} {ok:9d} {un:11d}')
    print('\n  The .md row is the gate. The wider rows are what it declines to'
          '\n  police: overwhelmingly paths a run WRITES, not authorities it'
          '\n  reads. Re-run this whenever the claim is questioned.')
    return 0


def main():
    if '--census' in sys.argv:
        return census()
    print('#803: every .md a skill or doc cites must be in the tree')
    tracked = tracked_paths()
    if tracked is None:
        print('SKIP: git ls-files unavailable; cannot tell tracked from not')
        return 77
    suffixes = suffixes_of(tracked)
    resolved, unresolved = scan()
    t_every_cited_md_resolves(resolved, unresolved)
    t_untracked_ok_has_no_stale_entry(unresolved)
    t_the_scanner_still_matches_something(resolved)
    t_a_sentence_final_citation_matches()
    t_the_gate_would_catch_the_803_shape(tracked, suffixes)
    print(f'  ({resolved} citations resolved, '
          f'{len(unresolved)} distinct unresolved, '
          f'{len(UNTRACKED_OK)} declared)')
    if FAILURES:
        print(f'FAIL: {len(FAILURES)} check(s)')
        return 1
    print('OK')
    return 0


if __name__ == '__main__':
    sys.exit(main())
