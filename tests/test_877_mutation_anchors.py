#!/usr/bin/env python3
"""Does `tests/mutation_anchors.py` actually bite? (#877)

The module exists to stop a mutation row asserting nothing. A checker that
answers "no problems" for a battery it did not understand has the same defect
it was written to find, one level up -- so every gate below is a battery-shaped
file built to be WRONG in a specific way, and the assertion is that the
resolver says so.

Every case here was found by an adversarial review of the first draft, which
reported a clean result for all five. They are kept as change detectors:

  1. a second table whose rows a helper builds -- `_fold` cannot read it, and
     the table used to VANISH while the battery still printed a clean line;
  2. a table bound with an ANNOTATED assignment (`ROWS: List[Row] = [...]`),
     which the first draft's `ast.Assign`-only walk did not see at all;
  3. a table bound twice at module scope, which was counted TWICE -- and an
     inflated anchor count is invisible against a floor;
  4. a create-this-file row aimed at a file that already exists, the one
     condition `mutate_713_census.py:129-132` reports BROKEN for and the one
     the first draft skipped outright;
  5. a table both layouts fit, which must be REFUSED rather than guessed; and
  6. a row whose `old` is itself a path to a real file, which was read as the
     row's TARGET -- grading the row's new text against the file its old text
     named, and reporting a FALSE STALE. A false stale costs what a missed one
     does.

Nothing here reads a battery in the repo and nothing imports one: three run
their runner at module scope, so importing them rewrites engine files. The
fixtures are written into a temporary directory and handed to
`resolve_static` as data, which is exactly how the #718 gate uses it.

    python3 tests/test_877_mutation_anchors.py
"""

import os
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, TESTS_DIR)

import mutation_anchors as ma                                     # noqa: E402

#: The stand-in engine every fixture battery mutates. Self-contained on
#: purpose: a fixture anchored to a real repo file would go stale exactly the
#: way the rows this module polices do, and then this gate would be testing
#: its own rot instead of the resolver.
ENGINE_TEXT = (
    'def alpha():\n'
    '    return 1\n'
    '\n'
    'def beta():\n'
    '    return 2\n'
    '\n'
    'def gamma():\n'
    '    return 2\n'
)

#: Occurs exactly once in ENGINE_TEXT.
UNIQUE = 'def alpha():'
#: Occurs exactly twice.
TWICE = '    return 2\n'
#: Occurs nowhere.
GONE = 'def delta():'


class _Fixture(object):
    """A temporary directory holding an engine file and battery-shaped files."""

    def __enter__(self):
        self.dir = tempfile.mkdtemp(prefix='mut877_')
        self.engine = os.path.join(self.dir, 'engine.py')
        with open(self.engine, 'w', encoding='utf-8', newline='') as fh:
            fh.write(ENGINE_TEXT)
        return self

    def __exit__(self, *exc):
        shutil.rmtree(self.dir, ignore_errors=True)
        return False

    def battery(self, name, body, with_engine=True):
        """Write a battery-shaped file and resolve it. Never imported.

        `with_engine` mirrors a real distinction: `mutate_750.py` names its one
        target `ENGINE`, and `mutate_713_census.py` / `mutate_760.py` name no
        table-level target at all and carry the path in each row. A fixture
        that always defined `ENGINE` could not express the second shape.
        """
        path = os.path.join(self.dir, name)
        head = ('import os\n'
                'ENGINE_TEST = %r\n'
                'OTHER = %r\n' % (self.engine, self.engine))
        if with_engine:
            head += 'ENGINE = %r\n' % self.engine
        with open(path, 'w', encoding='utf-8', newline='') as fh:
            fh.write(head + body)
        anchors, reason = ma.resolve_static(path)
        problems = [] if reason else ma.verify(anchors)
        return anchors, reason, problems


def test_a_helper_built_table_is_refused_not_dropped():
    """A table `_fold` cannot read must fail the battery, not disappear.

    `resolve_static` returns as soon as ONE table resolves, so a second table
    it could not fold was silently absent and the battery reported clean. The
    name is the signal: a binding called `*_ROWS` that is not a row table means
    the resolver has met a shape it does not know.
    """
    with _Fixture() as fx:
        _a, reason, _p = fx.battery('mutate_probe_a.py', (
            'def _row(a, b, c):\n    return (a, b, c)\n'
            'ENGINE_ROWS = [(%r, %r, %r)]\n'
            'GUI_ROWS = [_row(%r, %r, %r)]\n'
            'BATTERIES = {"e": (ENGINE, ENGINE_TEST, ENGINE_ROWS),\n'
            '             "g": (OTHER, ENGINE_TEST, GUI_ROWS)}\n'
            % ('r1', UNIQUE, 'x', 'r2', GONE, 'y')))
    assert reason and 'GUI_ROWS' in reason, (
        f'a table built by a helper call resolved to {reason!r} -- it must '
        f'name the table it could not read. A vanished table is a battery '
        f'reporting clean about rows nobody checked.')
    print('  PASS: a helper-built table is named, not dropped')


def test_an_annotated_table_assignment_is_seen():
    """`ROWS: List[Row] = [...]` binds exactly as `ROWS = [...]` does.

    An `ast.Assign`-only walk skips `ast.AnnAssign`, so the whole table was
    invisible -- and invisible reads as clean.
    """
    with _Fixture() as fx:
        anchors, reason, problems = fx.battery('mutate_probe_d.py', (
            'from typing import List, Tuple\n'
            'ENGINE_ROWS = [(%r, %r, %r)]\n'
            'GUI_ROWS: List[Tuple[str, str, str]] = [(%r, %r, %r)]\n'
            'BATTERIES = {"e": (ENGINE, ENGINE_TEST, ENGINE_ROWS),\n'
            '             "g": (OTHER, ENGINE_TEST, GUI_ROWS)}\n'
            % ('r1', UNIQUE, 'x', 'r2', GONE, 'y')))
    assert reason is None and len(anchors) == 2, (
        f'an annotated table assignment resolved to {len(anchors)} anchor(s), '
        f'reason={reason!r} -- both tables must be read')
    assert [p.kind for p in problems] == [ma.STALE], (
        f'the annotated table must still be GRADED, got '
        f'{[p.kind for p in problems]}')
    print('  PASS: an annotated table assignment is seen and graded')


def test_a_table_bound_twice_is_counted_once():
    """N assignments to one name are one table, at its LAST value.

    Counting assignment NODES emitted every row once per binding. An inflated
    anchor count is invisible against a floor, and it is the flattering
    direction: more anchors, all passing.
    """
    with _Fixture() as fx:
        anchors, reason, _p = fx.battery('mutate_probe_f.py', (
            'ROWS = [(%r, %r, %r)]\n'
            'ROWS = [(%r, %r, %r), (%r, %r, %r)]\n'
            % ('a', UNIQUE, 'x', 'b', UNIQUE, 'y', 'c', UNIQUE, 'z')))
    assert reason is None and [a.row for a in anchors] == ['b', 'c'], (
        f'a rebound table resolved to {[a.row for a in anchors]} -- it must be '
        f"the LAST binding's rows, once each")
    print('  PASS: a table bound twice is counted once, at its last value')


def test_a_create_row_whose_file_exists_is_reported():
    """The one condition a create row can fail, graded rather than skipped.

    A create row carries no anchor, so skipping it looked right -- but
    `mutate_713_census.py:129-132` reports BROKEN when the file it means to
    create is already there. Skipping meant the row the resolver singles out
    for special handling was the one row it checked nothing about.
    """
    with _Fixture() as fx:
        rel = os.path.relpath(fx.engine, ROOT).replace(os.sep, '/')
        _a, reason, problems = fx.battery('mutate_probe_h.py', (
            'GATE = ENGINE_TEST\n'
            'ROWS = [(%r, %r, None, %r, %r)]\n'
            % ('makes-an-existing-file', rel, 'body', 'why')),
            with_engine=False)
    assert reason is None and [p.kind for p in problems] == [ma.CREATE_EXISTS], (
        f'a create row aimed at an existing file gave reason={reason!r}, '
        f'problems={[p.kind for p in problems]} -- expected CREATE_EXISTS')
    print('  PASS: a create row whose file already exists is reported')


def test_an_ambiguous_table_is_refused_not_guessed():
    """Two viable readings must refuse, never resolve by a coin flip.

    When a table has a target of its own AND every row[1] names a real file,
    row[1] is either the anchor or the target and nothing in the file says
    which. No battery in the tree is ambiguous; one that becomes so should be
    read by a person.
    """
    with _Fixture() as fx:
        rel = os.path.relpath(fx.engine, ROOT).replace(os.sep, '/')
        _a, reason, _p = fx.battery('mutate_probe_i.py', (
            'ROWS = [(%r, %r, %r)]\n' % ('r', rel, 'x')))
    assert reason and 'two ways' in reason, (
        f'an ambiguous table resolved to reason={reason!r} -- it must refuse')
    print('  PASS: a table both layouts fit is refused, not guessed')


def test_an_anchor_that_looks_like_a_path_stays_an_anchor():
    """A row's `old` is not a target just because it parses as one.

    Probing `_abs_target(row[1])` per row meant a single row whose anchor was a
    repo-relative path to a real file was read as that row's TARGET -- the
    row's new text graded against the file its old text named, reported as a
    FALSE STALE. The layout is decided once per table, from all its rows.
    """
    with _Fixture() as fx:
        _a, reason, problems = fx.battery('mutate_probe_c.py', (
            'ROWS = [(%r, %r, %r)]\n'
            % ('drop-the-path', 'py_router/route_v2.py', 'replacement')))
    assert reason is None, f'reason={reason!r}'
    assert [p.kind for p in problems] == [ma.STALE], (
        f'expected the anchor graded against ENGINE, got '
        f'{[p.kind for p in problems]}')
    assert problems[0].anchor.old == 'py_router/route_v2.py', (
        f'row[1] was read as a TARGET, not as the anchor: '
        f'old={problems[0].anchor.old!r}')
    assert problems[0].anchor.target.endswith('engine.py'), (
        f'the target must come from the table, got {problems[0].anchor.target}')
    print('  PASS: an anchor that looks like a path stays an anchor')


def test_stale_ambiguous_and_ok_are_told_apart():
    """The three verdicts, on one table, against known content.

    #877 reported `mutate_703.py`'s three 2-match rows as stale. They are a
    different defect: a stale row rewrites nothing, an ambiguous one rewrites
    whichever site comes first.
    """
    with _Fixture() as fx:
        anchors, reason, problems = fx.battery('mutate_probe_v.py', (
            'ROWS = [(%r, %r, %r), (%r, %r, %r), (%r, %r, %r)]\n'
            % ('fine', UNIQUE, 'x', 'gone', GONE, 'y', 'twice', TWICE, 'z')))
    assert reason is None and len(anchors) == 3, f'reason={reason!r}'
    got = {p.anchor.row: p.kind for p in problems}
    assert got == {'gone': ma.STALE, 'twice': ma.AMBIGUOUS}, (
        f'expected one STALE and one AMBIGUOUS and the third clean, got {got}')
    print('  PASS: stale, ambiguous and matching are told apart')


def test_a_multi_edit_row_labels_every_edit():
    """Each `(old, new)` pair of a list-valued row is its own anchor.

    Edit 0 used to print as a bare row name, so a stale FIRST pair looked
    exactly like a stale scalar row and the reader could not tell which edit to
    fix.
    """
    with _Fixture() as fx:
        anchors, reason, problems = fx.battery('mutate_probe_m.py', (
            'ROWS = [(%r, [(%r, %r), (%r, %r)], None, %r)]\n'
            % ('pair', GONE, 'a', UNIQUE, 'b', 'KILLED')))
    assert reason is None and len(anchors) == 2, f'reason={reason!r}'
    assert [a.label for a in anchors] == ['pair[0]', 'pair[1]'], (
        f'every edit of a multi-edit row must be labelled, got '
        f'{[a.label for a in anchors]}')
    assert [(p.anchor.label, p.kind) for p in problems] == [
        ('pair[0]', ma.STALE)], (
        f'the stale FIRST pair must be named, got '
        f'{[(p.anchor.label, p.kind) for p in problems]}')
    print('  PASS: every edit of a multi-edit row is labelled and graded')


def test_the_corpus_is_populated():
    """A resolver that stopped reading the tree would pass every gate above.

    Those run on fixtures; this one asserts the real corpus is still there, so
    a green suite cannot mean "found nothing to check". A floor, not a target.
    """
    paths = ma.batteries()
    assert len(paths) >= 30, (
        f'only {len(paths)} mutation batteries found -- the scan has stopped '
        f'finding them')
    total = sum(len(ma.resolve_static(p)[0]) for p in paths)
    assert total >= 600, (
        f'only {total} anchor(s) resolved across {len(paths)} batteries -- the '
        f'resolver has stopped reading the tables')
    print(f'  PASS: {total} anchor(s) across {len(paths)} real batteries')


TESTS = [
    test_a_helper_built_table_is_refused_not_dropped,
    test_an_annotated_table_assignment_is_seen,
    test_a_table_bound_twice_is_counted_once,
    test_a_create_row_whose_file_exists_is_reported,
    test_an_ambiguous_table_is_refused_not_guessed,
    test_an_anchor_that_looks_like_a_path_stays_an_anchor,
    test_stale_ambiguous_and_ok_are_told_apart,
    test_a_multi_edit_row_labels_every_edit,
    test_the_corpus_is_populated,
]


if __name__ == '__main__':
    for t in TESTS:
        print(f'--- {t.__name__}')
        t()
    print('ALL PASS')
