#!/usr/bin/env python3
"""Do the mutation batteries' anchors still match the code they quote? (#877)

A mutation row is applied with `src.replace(old, new, 1)`. When `old` no longer
occurs in the target the row mutates NOTHING, and whatever the runner prints
next is about a tree it never changed. Every battery here does catch that --
each counts the anchor and reports BROKEN rather than a kill -- but it catches
it *during* the run, after the witnesses have been paid for. #877's own words:

    a stale anchor reports BROKEN 50 minutes into a run rather than in one
    second before it -- mutate_847.py:67-73

This module is the one second. It answers a single question -- *does this
anchor match its target exactly once?* -- for two callers that must not drift
apart:

  * each `tests/mutate_*.py`, from `--verify-anchors`, over the rows it already
    holds in memory (`verify`); and
  * `tests/test_718_static_test_hygiene.py`, over every battery in the tree,
    without importing any of them (`resolve_static` -> `verify`).

**Static resolution is not an optimisation, it is the only safe way.**
`mutate_713_census.py`, `mutate_713_phase1.py` and `mutate_760.py` run their
runner at module scope, so IMPORTING one mutates the tree. #877 was filed after
a census did exactly that and rewrote 13 engine files under `py_router/` and
`py_placer/` -- and reported inflated numbers measured against its own damage.
Nothing here imports a battery; everything is `ast`.

Two distinctions this draws that a plain `count(old) != 1` does not:

  * **STALE (0 matches) is not AMBIGUOUS (>1).** A stale anchor guards nothing.
    A 2-match anchor still mutates -- whichever site comes first -- so it is a
    row whose subject is decided by file order rather than by the row. Both
    deserve fixing; they are not the same finding, and #877 reported three
    ambiguous rows in `mutate_703.py` as stale.

  * **Targets are read with UNIVERSAL NEWLINES.** `mutate_711.py:296-321`
    records why: matching a multi-line anchor against a raw byte decode
    "silently found NOTHING in three rows".

    python3 tests/mutation_anchors.py               # census over every battery
    python3 tests/mutation_anchors.py --verbose     # every anchor, one a line
    python3 tests/mutation_anchors.py --battery mutate_702.py
"""
import argparse
import ast
import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)

#: `_fold` could not decide. Distinct from None, which several row layouts use
#: as a MEANINGFUL value (`old is None` means "create this file", not "unknown").
UNKNOWN = object()

STALE = 'STALE'
AMBIGUOUS = 'AMBIGUOUS'
MISSING_TARGET = 'MISSING_TARGET'


class Anchor(object):
    """One `old` string, and the file it is supposed to occur in exactly once.

    `nth` is `mutate_760.py`'s optional occurrence selector, which replaces the
    Nth match instead of the first, so its requirement is "at least nth+1
    matches" rather than "exactly one".

    `old is None` marks `mutate_713_census.py`'s create-this-file row, which
    carries no anchor at all and must not be graded as one.
    """

    __slots__ = ('battery', 'row', 'target', 'old', 'nth', 'edit')

    def __init__(self, battery, row, target, old, nth=None, edit=0):
        self.battery = battery
        self.row = row
        self.target = target
        self.old = old
        self.nth = nth
        self.edit = edit

    @property
    def label(self):
        return self.row if self.edit == 0 else '%s[%d]' % (self.row, self.edit)

    def __repr__(self):                                    # pragma: no cover
        return '<Anchor %s %s -> %s>' % (
            self.battery, self.label, os.path.basename(self.target))


class Problem(object):
    __slots__ = ('anchor', 'kind', 'count', 'detail')

    def __init__(self, anchor, kind, count, detail=''):
        self.anchor = anchor
        self.kind = kind
        self.count = count
        self.detail = detail

    def __str__(self):
        return '%s: %s %s -- matched %d time(s) in %s%s' % (
            self.anchor.battery, self.kind, self.anchor.label, self.count,
            os.path.relpath(self.anchor.target, ROOT).replace(os.sep, '/'),
            ('  ' + self.detail) if self.detail else '')


# --------------------------------------------------------------------------
# constant folding
# --------------------------------------------------------------------------

def _dotted(node):
    """'os.path.join' for the Attribute/Name chain in a Call's func, else ''."""
    parts = []
    while isinstance(node, ast.Attribute):
        parts.append(node.attr)
        node = node.value
    if not isinstance(node, ast.Name):
        return ''
    parts.append(node.id)
    return '.'.join(reversed(parts))


def _fold(node, env):
    """The value of `node`, or UNKNOWN.

    Deliberately narrow: string literals, implicit and explicit concatenation,
    module-level names, and the `os.path` calls every battery builds its target
    constants with. An f-string is UNKNOWN rather than guessed -- an anchor
    whose text this cannot reproduce EXACTLY must be refused, not approximated,
    or the census grades a string the battery will never apply.
    """
    if isinstance(node, ast.Constant):
        return node.value
    if isinstance(node, ast.Name):
        return env.get(node.id, UNKNOWN)
    if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Add):
        left = _fold(node.left, env)
        right = _fold(node.right, env)
        if isinstance(left, str) and isinstance(right, str):
            return left + right
        return UNKNOWN
    if isinstance(node, ast.Call):
        fn = _dotted(node.func)
        if fn not in ('os.path.join', 'os.path.dirname', 'os.path.abspath'):
            return UNKNOWN
        args = [_fold(a, env) for a in node.args]
        if not args or not all(isinstance(a, str) for a in args):
            return UNKNOWN
        if fn == 'os.path.join':
            return os.path.join(*args)
        if fn == 'os.path.dirname':
            return os.path.dirname(args[0])
        return os.path.abspath(args[0])
    if isinstance(node, (ast.Tuple, ast.List)):
        return [_fold(e, env) for e in node.elts]
    if isinstance(node, ast.Dict):
        out = {}
        for k, v in zip(node.keys, node.values):
            key = _fold(k, env)
            if isinstance(key, str):
                out[key] = _fold(v, env)
        return out
    return UNKNOWN


def _module_env(tree, path):
    """Module-level names a battery's tables are built from.

    Walks top-level assignments in order, so a constant may be defined in terms
    of an earlier one -- which is how every battery spells its paths
    (`_ROOT = os.path.dirname(_TESTS)`).
    """
    env = {'__file__': os.path.abspath(path)}
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        value = _fold(node.value, env)
        for tgt in node.targets:
            if isinstance(tgt, ast.Name):
                env[tgt.id] = value
    return env


# --------------------------------------------------------------------------
# table discovery
# --------------------------------------------------------------------------

def _is_row_table(value):
    """A list of tuples whose first element is a string: a mutation table.

    Every battery's table is one, and nothing else at module scope in these
    files is. Empty lists are rejected -- a table that resolved to nothing is
    a parse failure wearing a clean result.
    """
    if not isinstance(value, list) or not value:
        return False
    for row in value:
        if not isinstance(row, list) or len(row) < 3:
            return False
        if not isinstance(row[0], str):
            return False
    return True


def _tables(tree, env):
    """[(variable name, rows)] for every module-level mutation table."""
    out = []
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        for tgt in node.targets:
            if not isinstance(tgt, ast.Name):
                continue
            value = env.get(tgt.id, UNKNOWN)
            if _is_row_table(value):
                out.append((tgt.id, value))
    return out


def _battery_dict_targets(tree, env):
    """`mutate_746.py`'s `BATTERIES = {'engine': (ENGINE, TEST, ENGINE_ROWS)}`.

    Maps a table's VARIABLE NAME to the source path in the same tuple. Read off
    the AST rather than the folded value because the association is by name --
    two tables of equal content would be indistinguishable once folded.
    """
    out = {}
    for node in tree.body:
        if not isinstance(node, ast.Assign) or not isinstance(node.value,
                                                              ast.Dict):
            continue
        for val in node.value.values:
            if not isinstance(val, (ast.Tuple, ast.List)):
                continue
            names = [e.id for e in val.elts if isinstance(e, ast.Name)]
            paths = [env.get(n) for n in names]
            table = None
            src = None
            for name, resolved in zip(names, paths):
                if _is_row_table(resolved):
                    table = name
                elif isinstance(resolved, str) and src is None:
                    src = resolved
            if table and src:
                out[table] = src
    return out


def _abs_target(value, env, must_exist=True):
    """An absolute path from a row's target slot, else None.

    Handles both spellings: an already-absolute constant (`mutate_760.py`) and
    a repo-relative string (`mutate_713_*.py`).

    `must_exist` is False for a CREATE row, whose whole point is a file that is
    not there yet (`mutate_713_census.py`'s "a NEW file with a clock,
    registered nowhere"). Requiring the file unconditionally made that battery
    report UNRESOLVED -- a resolver refusing the one row that is behaving
    exactly as designed.
    """
    if not isinstance(value, str) or not value:
        return None
    path = value if os.path.isabs(value) else os.path.join(ROOT, value)
    if must_exist and not os.path.isfile(path):
        return None
    if not must_exist and not value.endswith('.py'):
        return None
    return path


def _edits(old, new):
    """[(old, ...)] for a row, expanding the list-valued multi-edit form.

    14 batteries let `old` be a list of `(old, new)` pairs with `new` None
    (`mutate_554.py:78-83`). Each pair is its own anchor: one of them going
    stale breaks the row exactly as a scalar anchor would, and #877's census
    counted the ROW rather than the pair.
    """
    if isinstance(old, list):
        out = []
        for pair in old:
            if isinstance(pair, list) and pair and isinstance(pair[0], str):
                out.append(pair[0])
            else:
                out.append(UNKNOWN)
        return out
    return [old]


def resolve_static(path):
    """(anchors, unresolved_reason) for one `tests/mutate_*.py`, without import.

    `unresolved_reason` is a string when the file could not be read as a
    battery at all; the caller must treat that as a failure, never as "no
    anchors". A resolver that answers cleanly for a file it did not understand
    is the same bug this module exists to catch.
    """
    battery = os.path.basename(path)
    try:
        with open(path, encoding='utf-8') as fh:
            src = fh.read()
    except OSError as exc:                                 # pragma: no cover
        return [], 'cannot read: %s' % exc
    try:
        tree = ast.parse(src, battery)
    except SyntaxError as exc:                             # pragma: no cover
        return [], 'cannot parse: %s' % exc

    env = _module_env(tree, path)
    tables = _tables(tree, env)
    if not tables:
        return [], 'no mutation table found'

    targets_map = env.get('TARGETS')
    if not isinstance(targets_map, dict):
        targets_map = {}
    by_table = _battery_dict_targets(tree, env)

    #: The single-target fallback: a battery with one engine file names it
    #: ENGINE (`mutate_750.py:46`). Only consulted when the row itself carries
    #: no target, so it cannot mask a per-row path.
    lone = _abs_target(env.get('ENGINE'), env)

    anchors = []
    for table_name, rows in tables:
        table_target = by_table.get(table_name) or lone
        for row in rows:
            name = row[0]
            slot = row[1]

            # Layout A: row[1] keys the TARGETS dict.
            if isinstance(slot, str) and slot in targets_map:
                target = _abs_target(targets_map[slot], env)
                old, new = row[2], row[3] if len(row) > 3 else None
                nth = None
            else:
                # A CREATE row (`old is None`) names a file that does not exist
                # yet; every other row's target must be a real file, or a typo
                # would silently resolve and grade as stale.
                creates = len(row) > 2 and row[2] is None
                explicit = _abs_target(slot, env, must_exist=not creates)
                if explicit is not None:
                    # Layouts D and E: row[1] IS the path.
                    target = explicit
                    old = row[2]
                    new = row[3] if len(row) > 3 else None
                    nth = row[4] if len(row) > 4 and isinstance(
                        row[4], int) and not isinstance(row[4], bool) else None
                else:
                    # Layouts B and C: no per-row target; row[1] is the anchor.
                    target = table_target
                    old, new = slot, row[2]
                    nth = None

            if target is None:
                return [], ('row %r in %s names no resolvable target'
                            % (name, table_name))

            for i, one in enumerate(_edits(old, new)):
                if one is UNKNOWN:
                    return [], ('row %r in %s has an anchor this cannot '
                                'reproduce exactly' % (name, table_name))
                if one is not None and not isinstance(one, str):
                    return [], ('row %r in %s has a non-string anchor'
                                % (name, table_name))
                anchors.append(Anchor(battery, name, target, one, nth,
                                      0 if not isinstance(old, list) else i))
    return anchors, None


# --------------------------------------------------------------------------
# verification
# --------------------------------------------------------------------------

def _read_target(path, cache):
    """The target's text with UNIVERSAL NEWLINES, which is what the batteries
    match against (`mutate_711.py:296-305`). Reading it any other way makes
    every multi-line anchor stale on a CRLF checkout."""
    if path not in cache:
        with open(path, encoding='utf-8', errors='replace') as fh:
            cache[path] = fh.read()
    return cache[path]


def verify(anchors, repo_wide=False):
    """Every problem among `anchors`, worst first within each battery.

    Returns a list, never raises on a bad anchor: the caller decides whether a
    problem is fatal, and a run that stops at the first one hides the rest.
    """
    problems = []
    cache = {}
    for a in anchors:
        if a.old is None:
            # `mutate_713_census.py`'s create-this-file row. It carries no
            # anchor; grading it as one invents a finding.
            continue
        if not os.path.isfile(a.target):
            problems.append(Problem(a, MISSING_TARGET, 0,
                                    'target does not exist'))
            continue
        n = _read_target(a.target, cache).count(a.old)
        if a.nth is not None:
            if n < a.nth + 1:
                problems.append(Problem(
                    a, STALE, n,
                    'needs occurrence %d, so at least %d match(es)'
                    % (a.nth, a.nth + 1)))
            continue
        if n == 0:
            problems.append(Problem(a, STALE, 0,
                                    'the row mutates nothing'))
        elif n > 1:
            problems.append(Problem(
                a, AMBIGUOUS, n,
                'replace(..., 1) takes whichever comes first in the file'))
    if repo_wide:
        problems.extend(_prose_problems(anchors))
    return problems


def _prose_problems(anchors):
    """Anchors that also occur in another tracked file.

    `mutate_726.py:248-280`'s check, kept: a comment quoting code has satisfied
    a grep-shaped test in this repo before, and a battery that mutates a
    comment reports SURVIVED for a mutation that changed nothing executable.
    """
    import subprocess
    out = subprocess.run(['git', 'ls-files', '*.py', '*.md'], cwd=ROOT,
                         capture_output=True, text=True).stdout.split()
    texts = {}
    for rel in out:
        path = os.path.join(ROOT, rel)
        try:
            with open(path, encoding='utf-8', errors='replace') as fh:
                texts[os.path.abspath(path)] = fh.read()
        except OSError:                                    # pragma: no cover
            continue
    problems = []
    for a in anchors:
        if a.old is None:
            continue
        battery_path = os.path.abspath(os.path.join(TESTS_DIR, a.battery))
        others = [p for p, text in texts.items()
                  if p not in (os.path.abspath(a.target), battery_path)
                  and a.old in text]
        if others:
            problems.append(Problem(
                a, 'PROSE', len(others),
                'also in ' + ', '.join(
                    os.path.relpath(p, ROOT).replace(os.sep, '/')
                    for p in sorted(others)[:3])))
    return problems


def batteries():
    """Every `tests/mutate_*.py`, sorted. The corpus both callers run over."""
    names = sorted(n for n in os.listdir(TESTS_DIR)
                   if n.startswith('mutate_') and n.endswith('.py'))
    return [os.path.join(TESTS_DIR, n) for n in names]


def census():
    """[(battery, anchors, problems, unresolved_reason)] over the whole tree."""
    out = []
    for path in batteries():
        anchors, reason = resolve_static(path)
        problems = [] if reason else verify(anchors)
        out.append((os.path.basename(path), anchors, problems, reason))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--battery', help='census one file, by basename')
    ap.add_argument('--verbose', action='store_true',
                    help='print every anchor, not only the problems')
    ap.add_argument('--repo-wide', action='store_true',
                    help="also report anchors that occur in another tracked "
                         "file, which may be prose rather than code")
    args = ap.parse_args()

    rows = census()
    if args.battery:
        rows = [r for r in rows if r[0] == args.battery]
        if not rows:
            print('no battery named %r' % args.battery, file=sys.stderr)
            return 2

    total = stale = ambiguous = missing = unresolved = 0
    print('%-26s %6s %6s %6s %6s' % ('battery', 'rows', 'STALE', 'AMBIG',
                                     'UNRES'))
    for name, anchors, problems, reason in rows:
        if args.repo_wide and not reason:
            problems = verify(anchors, repo_wide=True)
        n_stale = sum(1 for p in problems if p.kind == STALE)
        n_amb = sum(1 for p in problems if p.kind == AMBIGUOUS)
        n_missing = sum(1 for p in problems if p.kind == MISSING_TARGET)
        total += len(anchors)
        stale += n_stale
        ambiguous += n_amb
        missing += n_missing
        if reason:
            unresolved += 1
        print('%-26s %6s %6s %6s %6s' % (
            name, len(anchors) if not reason else '-', n_stale, n_amb,
            'YES' if reason else ''))
        if reason:
            print('    UNRESOLVED: %s' % reason)
        for p in problems:
            print('    %s' % p)
        if args.verbose and not reason:
            for a in anchors:
                print('      %-52s %s' % (
                    a.label,
                    os.path.relpath(a.target, ROOT).replace(os.sep, '/')))

    print('\n%d battery(s), %d anchor(s): %d stale, %d ambiguous, %d missing '
          'target, %d unresolved' % (len(rows), total, stale, ambiguous,
                                     missing, unresolved))
    return 1 if (stale or missing or unresolved) else 0


if __name__ == '__main__':
    sys.exit(main())
