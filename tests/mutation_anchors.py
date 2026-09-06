#!/usr/bin/env python3
"""Do the mutation batteries' anchors still match the code they quote? (#877)

A mutation row is applied with `src.replace(old, new, 1)`. When `old` no longer
occurs in the target the row mutates NOTHING, and whatever the runner prints
next is about a tree it never changed. Every battery here does catch that --
each counts the anchor and reports BROKEN, and every one exits non-zero -- but
it catches it DURING the run, after the witnesses have been paid for. #877's
own citation:

    a stale anchor reports BROKEN 50 minutes into a run rather than in one
    second before it -- mutate_847.py:67-73

This module is the one second. It answers a single question -- *does this
anchor match its target exactly once?* -- and it is meant to be the ONLY
implementation of that question in the tree, for two callers:

  * each `tests/mutate_*.py`, from `--verify-anchors`, over the rows it already
    holds in memory (`verify`); and
  * `tests/test_718_static_test_hygiene.py`, over every battery, without
    importing any of them (`resolve_static` -> `verify`).

The second caller exists today; the batteries are converted in the same PR.
Until that lands, the five hand-rolled `--verify-anchors` implementations
(`mutate_714`, `726`, `837`, `847`, `850_848`) are still the shipped state, and
`mutate_726.py:248-280`'s is STRICTER than this module's default -- its prose
check is always on, where here it needs `--repo-wide`.

**Static resolution is not an optimisation, it is the only safe way.**
`mutate_713_census.py`, `mutate_713_phase1.py` and `mutate_760.py` run their
runner at module scope, so IMPORTING one mutates the tree. #877 was filed after
a census did exactly that, rewrote 13 engine files under `py_router/` and
`py_placer/`, and reported numbers inflated by its own damage. Nothing here
imports a battery; everything is `ast`.

Distinctions a plain `count(old) != 1` does not draw:

  * **STALE (0 matches) is not AMBIGUOUS (>1).** A stale anchor guards nothing.
    A 2-match anchor still mutates -- whichever site comes first -- so the row's
    subject is decided by file order rather than by the row. Both fail; they are
    not the same finding, and #877 reported `mutate_703.py`'s three ambiguous
    rows as stale.

  * **NEWLINE_SENSITIVE**: the anchor's verdict changes with how the target is
    read. This module reads universal-newline, and so does `mutate_711.py`
    (`:296-305`), whose comment records that matching a raw byte decode
    "silently found NOTHING in three rows". But 18 batteries read their target
    with `newline=''`, so on a CRLF checkout the census and the battery would
    disagree. Dormant on this tree -- `.gitattributes` pins `*.py text eol=lf`
    -- and free to keep.

  * A **create row** (`old is None`, `mutate_713_census.py`'s "a NEW file with
    a clock, registered nowhere") carries no anchor, but it is not unchecked:
    the battery reports BROKEN when the file it means to create already exists
    (`mutate_713_census.py:129-132`), and that is what is graded here.

    python3 tests/mutation_anchors.py               # census over every battery
    python3 tests/mutation_anchors.py --verbose     # every anchor, one a line
    python3 tests/mutation_anchors.py --battery mutate_702.py
"""
import argparse
import ast
import os
import re
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)

#: `_fold` could not decide. Distinct from None, which several row layouts use
#: as a MEANINGFUL value (`old is None` means "create this file", not "unknown").
UNKNOWN = object()

STALE = 'STALE'
AMBIGUOUS = 'AMBIGUOUS'
MISSING_TARGET = 'MISSING_TARGET'
NEWLINE_SENSITIVE = 'NEWLINE_SENSITIVE'
CREATE_EXISTS = 'CREATE_EXISTS'
PROSE = 'PROSE'

#: Names a battery gives a mutation table. A module-level binding matching this
#: MUST resolve to a table or the whole battery is reported UNRESOLVED -- see
#: `_tables`, and the defect that motivated it.
_TABLE_NAME_RE = re.compile(r'^(ROWS|[A-Z_]*_ROWS)$')


class Anchor(object):
    """One `old` string, and the file it must occur in exactly once.

    `nth` is the optional occurrence selector `mutate_760.py:65-79` implements
    (`row[4]`), which replaces the Nth match instead of the first -- so its
    requirement is "at least nth+1 matches", not "exactly one". No row uses it
    today; the form is supported because the battery accepts it.

    `old is None` marks a create-this-file row, which has no anchor.
    """

    __slots__ = ('battery', 'row', 'target', 'old', 'nth', 'edit', 'multi')

    def __init__(self, battery, row, target, old, nth=None, edit=0,
                 multi=False):
        self.battery = battery
        self.row = row
        self.target = target
        self.old = old
        self.nth = nth
        self.edit = edit
        self.multi = multi

    @property
    def label(self):
        """`name[i]` for every edit of a multi-edit row, INCLUDING edit 0.

        Suppressing `[0]` made a stale first pair print as a bare row name,
        indistinguishable from a stale scalar row -- so the reader could not
        tell which of the row's edits to fix.
        """
        return '%s[%d]' % (self.row, self.edit) if self.multi else self.row

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
        rel = os.path.relpath(self.anchor.target, ROOT).replace(os.sep, '/')
        if self.kind in (PROSE, CREATE_EXISTS):
            # `count` is not an occurrence count for these, and rendering it as
            # one printed "matched 458 time(s) in <the wrong file>".
            head = '%s: %s %s' % (self.anchor.battery, self.kind,
                                  self.anchor.label)
        else:
            head = '%s: %s %s -- matched %d time(s) in %s' % (
                self.anchor.battery, self.kind, self.anchor.label,
                self.count, rel)
        return head + (('  ' + self.detail) if self.detail else '')


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


def _assignments(tree):
    """(name, value_node) for each module-level binding, in source order.

    `ast.AnnAssign` is included: `ROWS: List[Row] = [...]` binds exactly as
    `ROWS = [...]` does, and skipping it made a whole table invisible while the
    battery still reported clean.
    """
    for node in tree.body:
        if isinstance(node, ast.Assign):
            for tgt in node.targets:
                if isinstance(tgt, ast.Name):
                    yield tgt.id, node.value
        elif isinstance(node, ast.AnnAssign) and node.value is not None:
            if isinstance(node.target, ast.Name):
                yield node.target.id, node.value


def _module_env(tree, path):
    """Module-level names a battery's tables are built from.

    In source order, so a constant may be defined in terms of an earlier one --
    which is how every battery spells its paths (`_ROOT =
    os.path.dirname(_TESTS)`). A rebound name keeps its LAST value, as Python
    would.
    """
    env = {'__file__': os.path.abspath(path)}
    for name, value_node in _assignments(tree):
        env[name] = _fold(value_node, env)
    return env


# --------------------------------------------------------------------------
# table discovery
# --------------------------------------------------------------------------

def _is_row_table(value):
    """A non-empty list of tuples whose first element is a string.

    Every battery's table is one and nothing else at module scope is. Empty
    lists are rejected: a table that resolved to nothing is a parse failure
    wearing a clean result.
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
    """([(name, rows)], reason) for the module-level mutation tables.

    Two rules, and the second is the one that matters:

    * a binding whose value IS a row table is a table; and
    * a binding whose NAME declares one (`ROWS`, `*_ROWS`) and whose value is
      not a row table makes the whole battery UNRESOLVED.

    Without the second rule a table this cannot fold -- rows built by a helper
    call, say -- simply vanished, and the battery still printed a clean line
    for the tables that did resolve. A resolver that answers cleanly for a file
    it did not understand is the bug this module exists to catch.

    Names are collected ONCE each. Iterating assignment NODES counted a table
    rebound at module scope twice over, silently doubling its anchors.
    """
    seen, out = set(), []
    for name, _node in _assignments(tree):
        if name in seen:
            continue
        seen.add(name)
        value = env.get(name, UNKNOWN)
        if _is_row_table(value):
            out.append((name, value))
        elif _TABLE_NAME_RE.match(name):
            return [], ('%s is named like a mutation table but did not '
                        'resolve to one' % name)
    if not out:
        return [], 'no mutation table found'
    return out, None


def _battery_dict_targets(tree, env):
    """`mutate_746.py`'s `BATTERIES = {'engine': (ENGINE, TEST, ENGINE_ROWS)}`.

    Maps a table's VARIABLE NAME to the source path in the same tuple. Read off
    the AST rather than the folded value because the association is by name --
    two tables of equal content would be indistinguishable once folded.

    The path goes through `_abs_target` like every other. Returning `env`'s raw
    value made a relative constant resolve against the PROCESS CWD, so the same
    battery answered differently depending on where it was run from.
    """
    out = {}
    for _name, node in _assignments(tree):
        if not isinstance(node, ast.Dict):
            continue
        for val in node.values:
            if not isinstance(val, (ast.Tuple, ast.List)):
                continue
            names = [e.id for e in val.elts if isinstance(e, ast.Name)]
            table = src = None
            for elt_name in names:
                resolved = env.get(elt_name)
                if _is_row_table(resolved):
                    table = elt_name
                elif src is None:
                    src = _abs_target(resolved)
            if table and src:
                out[table] = src
    return out


def _abs_target(value, must_exist=True):
    """An absolute path from a row's target slot, else None.

    Handles both spellings: an already-absolute constant (`mutate_760.py`) and
    a repo-relative string (`mutate_713_*.py`). Relative paths resolve against
    the REPO ROOT, never the process CWD.

    `must_exist` is False for a CREATE row, whose whole point is a file that is
    not there yet.
    """
    if not isinstance(value, str) or not value:
        return None
    path = value if os.path.isabs(value) else os.path.join(ROOT, value)
    if must_exist:
        return path if os.path.isfile(path) else None
    return path if value.endswith('.py') else None


def _edits(old):
    """The `old` of each edit in a row, expanding the list-valued form.

    14 batteries let `old` be a list of `(old, new)` pairs with `new` None
    (`mutate_554.py:78-83`); `mutate_553.py:301-310` applies them as
    `for o, nw in edits`, so element 0 of each pair is the anchor. Each pair is
    its own anchor -- one going stale breaks the row exactly as a scalar anchor
    would, and #877's census counted the ROW rather than the pair.
    """
    if isinstance(old, list):
        return [pair[0] if isinstance(pair, list) and pair else UNKNOWN
                for pair in old], True
    return [old], False


def _layout(rows, targets_map, table_target, table_name):
    """(layout, reason) for one table: 'key', 'path' or 'lone'.

    Decided ONCE for the whole table, from ALL its rows, because a table is
    homogeneous and a per-row guess is not safe. Trying `_abs_target(row[1])`
    per row meant a single row whose `old` happened to be a repo-relative path
    to a real file was read as a target -- grading that row's NEW text against
    the file its OLD text named, and reporting a false STALE. A false stale
    costs exactly what a missed one does.

    When both readings are viable the table is REFUSED, not guessed. No
    battery in the tree is ambiguous today; one that becomes so must be read by
    someone rather than resolved by a coin flip.
    """
    def creates(row):
        return len(row) > 2 and row[2] is None

    key = bool(targets_map) and all(
        isinstance(r[1], str) and r[1] in targets_map for r in rows)
    if key:
        return 'key', None

    path = all(_abs_target(r[1], must_exist=not creates(r)) is not None
               for r in rows)
    if path and table_target is not None:
        return None, ('%s can be read two ways -- every row[1] names a real '
                      'file AND the table has a target of its own' % table_name)
    if path:
        return 'path', None
    if table_target is not None:
        return 'lone', None
    return None, ('%s names no target: row[1] is not a TARGETS key, not a '
                  'resolvable path, and the table has no target of its own'
                  % table_name)


def resolve_static(path):
    """(anchors, unresolved_reason) for one `tests/mutate_*.py`, without import.

    `unresolved_reason` is a string when the file could not be read as a
    battery; the caller must treat that as a failure, never as "no anchors".
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
    tables, reason = _tables(tree, env)
    if reason:
        return [], reason

    targets_map = env.get('TARGETS')
    if not isinstance(targets_map, dict):
        targets_map = {}
    by_table = _battery_dict_targets(tree, env)

    #: The single-target fallback: a battery with one engine file names it
    #: ENGINE (`mutate_750.py:46`).
    lone = _abs_target(env.get('ENGINE'))

    anchors = []
    for table_name, rows in tables:
        table_target = by_table.get(table_name) or lone
        layout, reason = _layout(rows, targets_map, table_target, table_name)
        if reason:
            return [], reason

        for row in rows:
            name, slot = row[0], row[1]
            nth = None
            if layout == 'key':
                target = _abs_target(targets_map[slot])
                old = row[2]
            elif layout == 'path':
                creates = len(row) > 2 and row[2] is None
                target = _abs_target(slot, must_exist=not creates)
                old = row[2] if len(row) > 2 else UNKNOWN
                nth = (row[4] if len(row) > 4 and isinstance(row[4], int)
                       and not isinstance(row[4], bool) else None)
            else:
                target, old = table_target, slot

            if target is None:
                return [], ('row %r in %s names no resolvable target'
                            % (name, table_name))

            olds, multi = _edits(old)
            for i, one in enumerate(olds):
                if one is UNKNOWN:
                    return [], ('row %r in %s has an anchor this cannot '
                                'reproduce exactly' % (name, table_name))
                if one is not None and not isinstance(one, str):
                    return [], ('row %r in %s has a non-string anchor'
                                % (name, table_name))
                anchors.append(Anchor(battery, name, target, one, nth, i,
                                      multi))
    return anchors, None


# --------------------------------------------------------------------------
# verification
# --------------------------------------------------------------------------

def _read_target(path, cache):
    """(universal_newline_text, as_written_text) for a target.

    Both, because the batteries disagree about which they match against: this
    module and `mutate_711.py` read universal-newline, while 18 batteries read
    with `newline=''`. On an LF checkout the two are identical; on a CRLF one
    they are not, and an anchor whose count differs between them is a verdict
    that depends on the checkout rather than on the code.
    """
    if path not in cache:
        with open(path, 'rb') as fh:
            raw = fh.read().decode('utf-8', 'replace')
        cache[path] = (raw.replace('\r\n', '\n'), raw)
    return cache[path]


def verify(anchors, repo_wide=False):
    """Every problem among `anchors`.

    Returns a list and never raises: the caller decides what is fatal, and a
    run that stops at the first problem hides the rest.
    """
    problems = []
    cache = {}
    for a in anchors:
        if a.old is None:
            # A create row has no anchor, but it is not unchecked: the battery
            # refuses when the file it means to create is already there
            # (`mutate_713_census.py:129-132`).
            if os.path.exists(a.target):
                problems.append(Problem(
                    a, CREATE_EXISTS, 0,
                    'the row creates %s, which already exists -- the battery '
                    'reports BROKEN'
                    % os.path.relpath(a.target, ROOT).replace(os.sep, '/')))
            continue
        if not os.path.isfile(a.target):
            problems.append(Problem(a, MISSING_TARGET, 0,
                                    'target does not exist'))
            continue
        text, as_written = _read_target(a.target, cache)
        n = text.count(a.old)
        if n != as_written.count(a.old):
            problems.append(Problem(
                a, NEWLINE_SENSITIVE, n,
                'as-written count is %d -- the verdict depends on how the '
                'target is read, and the batteries disagree about that'
                % as_written.count(a.old)))
            continue
        if a.nth is not None:
            if n < a.nth + 1:
                problems.append(Problem(
                    a, STALE, n,
                    'needs occurrence %d, so at least %d match(es)'
                    % (a.nth, a.nth + 1)))
            continue
        if n == 0:
            problems.append(Problem(a, STALE, 0, 'the row mutates nothing'))
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
    listed = subprocess.run(['git', 'ls-files', '*.py', '*.md'], cwd=ROOT,
                            capture_output=True, text=True).stdout.split()
    texts = {}
    for rel in listed:
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
        others = sorted(p for p, text in texts.items()
                        if p not in (os.path.abspath(a.target), battery_path)
                        and a.old in text)
        if others:
            problems.append(Problem(
                a, PROSE, len(others),
                'also in %d other tracked file(s): %s' % (
                    len(others),
                    ', '.join(os.path.relpath(p, ROOT).replace(os.sep, '/')
                              for p in others[:3]))))
    return problems


def batteries():
    """Every `tests/mutate_*.py`, sorted. The corpus both callers run over."""
    names = sorted(n for n in os.listdir(TESTS_DIR)
                   if n.startswith('mutate_') and n.endswith('.py'))
    return [os.path.join(TESTS_DIR, n) for n in names]


def census(repo_wide=False):
    """[(battery, anchors, problems, unresolved_reason)] over the whole tree."""
    out = []
    for path in batteries():
        anchors, reason = resolve_static(path)
        problems = [] if reason else verify(anchors, repo_wide=repo_wide)
        out.append((os.path.basename(path), anchors, problems, reason))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--battery', help='census one file, by basename')
    ap.add_argument('--verbose', action='store_true',
                    help='print every anchor, not only the problems')
    ap.add_argument('--repo-wide', action='store_true',
                    help='also report anchors that occur in another tracked '
                         'file, which may be prose rather than code')
    args = ap.parse_args()

    rows = census(repo_wide=args.repo_wide)
    if args.battery:
        rows = [r for r in rows if r[0] == args.battery]
        if not rows:
            print('no battery named %r' % args.battery, file=sys.stderr)
            return 2

    total = unresolved = 0
    kinds = {}
    print('%-26s %6s %6s %6s %6s' % ('battery', 'rows', 'STALE', 'AMBIG',
                                     'UNRES'))
    for name, anchors, problems, reason in rows:
        total += len(anchors)
        if reason:
            unresolved += 1
        for p in problems:
            kinds[p.kind] = kinds.get(p.kind, 0) + 1
        print('%-26s %6s %6s %6s %6s' % (
            name, len(anchors) if not reason else '-',
            sum(1 for p in problems if p.kind == STALE),
            sum(1 for p in problems if p.kind == AMBIGUOUS),
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

    summary = ', '.join('%d %s' % (kinds[k], k.lower())
                        for k in sorted(kinds)) or 'no problems'
    print('\n%d battery(s), %d anchor(s): %s, %d unresolved'
          % (len(rows), total, summary, unresolved))
    # EVERY problem kind fails, not only STALE. The exit code and the #718 gate
    # must agree about what is wrong, or the one-second check reports green
    # while the suite goes red on the same rows.
    return 1 if (kinds or unresolved) else 0


if __name__ == '__main__':
    sys.exit(main())
