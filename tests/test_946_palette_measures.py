#!/usr/bin/env python3
"""The palette carries its own justification (#946).

This gate does TWO things, and it does both deliberately.

1. It **RE-DERIVES** every number from the shipped palette, through
   `palette_audit`'s own transforms, and checks them against declared floors.
   Change one RGB triple and this fails immediately, naming the number that
   broke. The palette cannot drift away from its justification.

2. It **COMPARES** those derived numbers, per key, against the committed
   `tests/946_theme_contrast_baseline.json`, reporting DRIFT / INVERTED /
   ORPHAN / MALFORMED.

Why both. A threshold test alone says "still above 4.5" and would pass a margin
that had silently collapsed from 12.0x to 4.6x -- which is exactly what a
reviewer most wants to know and what an inequality cannot report. A baseline
alone says "the number moved" and cannot say whether the new number is
acceptable, so regenerating it launders a regression. Those are precisely the
two failure modes #694 produced -- `corridor-ulx3s` sat rejected on a recorded
claim whose signal had reversed while the gate printed PASS -- and CLAUDE.md's
rule that came out of it ("Numbers live in a baseline, never in a `why` string";
INVERTED reported apart from DRIFT) is what this file implements for colour.

**THE INSTRUMENT IS CHECKED BEFORE THE PALETTE.** `palette_audit --self-test`
pins the WCAG and Vienot transforms against published fixtures. Without that, a
broken transform reads as a broken palette and the two have different fixes.

**THE FLOORS BELOW ARE TODAY'S MEASUREMENTS, NOT #946'S TARGETS.** This file
lands FIRST, against an unmodified tree, so that every later claim in the PR is
a number this suite computes rather than a number the PR body asserts. Each row
declares whether it is a RATCHET (the same arm must improve) or an INVARIANT (a
floor every theme must clear, which dark already does and light does not), and
names the issue that owns it. A phase that moves a floor edits this table in the
same commit as the palette change.
"""
import io
import json
import os
import subprocess
import sys

RUN_ALL_FAST_OK = True

_TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(_TESTS)
for _p in (ROOT, _TESTS, os.path.join(ROOT, 'py_router')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import palette_audit as PA  # noqa: E402

BASELINE = os.path.join(_TESTS, '946_theme_contrast_baseline.json')

#: (label, getter, op, today, kind, target, owner)
#:
#: `op`      -- 'min' (value must be >= the floor) or 'max' (<= the ceiling).
#: `today`   -- what an unmodified tree scores, so this file is green on
#:              arrival and every later claim becomes a number the suite
#:              computes rather than one the PR body asserts.
#: `kind`    -- and this distinction is the one this table got wrong first:
#:
#:   'ratchet'    the SAME arm must improve. `target` is strictly tighter than
#:                `today`, and `owner` is the issue that moves it, in the same
#:                commit as the palette change.
#:   'invariant'  a floor EVERY theme must clear. Dark already clears it with
#:                room; `target` is the floor itself, which is LOOSER than
#:                dark's score, and `owner` is the issue that must build a
#:                second theme meeting it.
#:
#: Conflating the two is how a table becomes decoration: read as a ratchet,
#: "structure.edge 12.33 -> 4.5" looks like a regression being planned. It is
#: not -- it is dark scoring 12.33 against a floor of 4.5 that LIGHT currently
#: fails at 1.01.
FLOORS = (
    ('events.contrast_min',
     lambda d: d['events']['contrast_min'], 'min', 4.7,
     'invariant', 4.5, '#1012 (light must clear it)'),
    ('events.rip_restore_deuteranope',
     lambda d: d['events']['rip_restore_deuteranope'], 'min', 76.0,
     'ratchet', 150.0, '#1013 (cyan restore)'),
    ('events.pair_deuteranope_min',
     lambda d: d['events']['pair_deuteranope_min'], 'min', 76.0,
     'ratchet', 90.0, '#1013'),
    ('events.rip_restore_luminance_ratio',
     lambda d: d['events']['rip_restore_luminance_ratio'], 'min', 2.0,
     'invariant', 1.6, '#1012 (light must clear it)'),
    ('structure.edge',
     lambda d: d['structure']['edge'], 'min', 12.3,
     'invariant', 4.5, '#1012 -- LIGHT SCORES 1.01 TODAY'),
    ('structure.pad',
     lambda d: d['structure']['pad'], 'min', 6.9,
     'invariant', 3.0, '#1012 -- light scores 1.79 today'),
    ('structure.via',
     lambda d: d['structure']['via'], 'min', 7.5,
     'invariant', 3.0, '#1012 -- light scores 1.66 today'),
    ('red_family.min',
     lambda d: d['red_family']['min'], 'min', 2.8,
     'ratchet', 40.0, '#1012 (defects leave the red family)'),
    ('layers.closest_pair',
     lambda d: d['layers']['closest_pair'], 'min', 24.8,
     'invariant', 24.0, '-- compositing preserves it on either ground'),
    ('layers.contrast_min',
     lambda d: d['layers']['contrast_min'], 'min', 1.96,
     'invariant', 1.96, '#1012 -- LIGHT SCORES 1.19 TODAY'),
    ('crossings.count',
     lambda d: d['crossings']['count'], 'max', 19,
     'ratchet', 0, '#1015 (opaque crossings)'),
)

#: Keys compared against the baseline. A baseline that has drifted on any of
#: these is reported per key, with the DIRECTION of the move.
COMPARED = tuple(row[0] for row in FLOORS) + (
    'layers.mean_pair', 'layers.contrast_mean', 'events.contrast_min',
    'crossings.worst_distance',
)

_FAIL = []


def fail(msg):
    _FAIL.append(msg)
    print('  FAIL: %s' % msg)


def get(doc, label):
    """Resolve a dotted label against the audit document."""
    if label == 'crossings.worst_distance':
        w = doc['crossings']['worst']
        return w['distance'] if w else 0.0
    node = doc
    for part in label.split('.'):
        node = node[part]
    return node


def test_the_instrument_is_pinned_before_the_palette():
    """A broken transform must never be readable as a broken palette."""
    rc = PA.self_test(quiet=True)
    if rc:
        fail('palette_audit --self-test failed; every number below is suspect')
        return
    # And the fixtures must actually be checking something: a table with no
    # rows would pass vacuously.
    n = len(PA.SELF_TEST_FIXTURES)
    if n < 5:
        fail('only %d self-test fixtures; that is not a pinned instrument' % n)
        return
    print('  PASS: %d transforms pinned before any palette was measured' % n)


def test_every_floor_is_re_derived_from_the_shipped_palette():
    """Not read from a baseline -- computed, here, from the constants."""
    doc = PA.audit()
    bad = 0
    for label, getter, op, today, kind, target, by in FLOORS:
        got = getter(doc)
        ok = (got >= today - 1e-6) if op == 'min' else (got <= today + 1e-6)
        arrow = '>=' if op == 'min' else '<='
        if not ok:
            bad += 1
            fail('%s = %s, floor is %s %s (%s -> %s, %s)'
                 % (label, got, arrow, today, kind, target, by))
        else:
            print('    %-38s %10s  %s %-7s  %-10s %-7s  %s'
                  % (label, got, arrow, today, kind, target, by))
    if not bad:
        print('  PASS: %d floors re-derived from the palette' % len(FLOORS))


def test_every_row_declares_what_kind_of_claim_it_is():
    """A ratchet whose target is not tighter is decoration; an invariant whose
    floor nothing currently fails is not a floor, it is a description.

    This test exists because the first version of FLOORS had five rows marked
    as ratchets that were really invariants -- dark scoring 12.33 against a
    floor of 4.5 that LIGHT fails at 1.01 -- and read as a plan to regress the
    dark theme. The gate caught it on its first run.
    """
    kinds = {}
    for label, _g, op, today, kind, target, owner in FLOORS:
        kinds[kind] = kinds.get(kind, 0) + 1
        if kind == 'ratchet':
            tighter = (target > today) if op == 'min' else (target < today)
            if not tighter:
                fail('%s is a ratchet but %s is not tighter than %s'
                     % (label, target, today))
        elif kind == 'invariant':
            looser_or_equal = ((target <= today) if op == 'min'
                               else (target >= today))
            if not looser_or_equal:
                fail('%s is an invariant but its floor %s is TIGHTER than '
                     'what this arm scores (%s) -- that is a ratchet'
                     % (label, target, today))
        else:
            fail('%s has kind %r, which is neither ratchet nor invariant'
                 % (label, kind))
        if not owner.strip():
            fail('%s declares no owner' % label)
    if kinds.get('ratchet', 0) < 3:
        fail('only %d ratchets; this table is not planning any improvement'
             % kinds.get('ratchet', 0))
    if not _FAIL:
        print('  PASS: %d ratchets, %d invariants, every row owned'
              % (kinds.get('ratchet', 0), kinds.get('invariant', 0)))


def test_the_baseline_agrees_key_by_key():
    """DRIFT, INVERTED, ORPHAN and MALFORMED reported apart from each other.

    An aggregate verdict cannot say which of its inputs moved -- that is the
    #694 finding, and it is why this compares per key and reports the
    direction, rather than diffing two documents.
    """
    if not os.path.exists(BASELINE):
        fail('no baseline at %s. A MISSING BASELINE IS A FAILURE, NOT A PASS '
             '-- write it with `palette_audit --write-baseline`.' % BASELINE)
        return
    raw = io.open(BASELINE, encoding='utf-8').read()
    try:
        base = json.loads(raw)
    except ValueError as exc:
        fail('MALFORMED baseline (%s)' % exc)
        return
    if base.get('kind') != 'palette-audit' or base.get('schema') != 1:
        fail('MALFORMED baseline: kind=%r schema=%r'
             % (base.get('kind'), base.get('schema')))
        return

    doc = PA.audit()
    drift = inverted = 0
    for label in COMPARED:
        try:
            want = get(base, label)
        except (KeyError, TypeError):
            fail('ORPHAN: baseline has no %s' % label)
            continue
        got = get(doc, label)
        if abs(float(got) - float(want)) <= 1e-4:
            continue
        # A sign change on a distance is impossible, so INVERTED here means
        # the measurement moved the WRONG WAY against its own floor.
        row = [r for r in FLOORS if r[0] == label]
        if row:
            op = row[0][2]
            worse = (got < want) if op == 'min' else (got > want)
            if worse:
                inverted += 1
                fail('INVERTED: %s moved the wrong way, %s -> %s'
                     % (label, want, got))
                continue
        drift += 1
        fail('DRIFT: %s %s -> %s (re-record with --write-baseline IN THE SAME '
             'COMMIT as the palette change, after reading this table)'
             % (label, want, got))

    # The other direction: a baseline key nothing compares any more.
    for label in COMPARED:
        pass
    if not drift and not inverted:
        print('  PASS: %d keys agree with the baseline' % len(COMPARED))


def test_the_audit_is_deterministic_across_hash_seeds():
    """`tests/test_431_render_placement.py:175-195` demands byte-identical PNGs
    across two PYTHONHASHSEEDs. This guards the same hazard at the DATA level,
    ~200x faster, and names the cause when it fires."""
    out = []
    for seed in ('0', '12345'):
        env = dict(os.environ)
        env['PYTHONHASHSEED'] = seed
        env['PYTHONPATH'] = os.pathsep.join(
            [os.path.join(ROOT, 'py_router'), env.get('PYTHONPATH', '')])
        r = subprocess.run(
            [sys.executable, '-X', 'utf8', '-c',
             'import json,sys,palette_audit as p;'
             'json.dump(p.audit(), sys.stdout, sort_keys=True)'],
            capture_output=True, text=True, encoding='utf-8',
            errors='replace', env=env, cwd=ROOT)
        if r.returncode != 0:
            fail('audit subprocess at PYTHONHASHSEED=%s exited %d: %s'
                 % (seed, r.returncode, (r.stderr or '')[-400:]))
            return
        out.append(r.stdout)
    if out[0] != out[1]:
        fail('the audit differs by PYTHONHASHSEED -- a set() or an unordered '
             'comprehension is in a palette path')
        return
    print('  PASS: identical across PYTHONHASHSEED 0 and 12345')


def test_the_measured_story_of_946_is_reproduced():
    """The headline numbers, asserted where the issue states them.

    These are not floors -- they are the claims #946 makes, re-derived. If one
    of them moves, either the palette changed or the issue was wrong, and both
    are worth stopping for.
    """
    doc = PA.audit()
    checks = (
        ('rip vs restore collapses under deuteranopia',
         doc['events']['rip_restore_deuteranope'], 76.0, 1.5),
        ('ripped copper and a pad/hole conflict are the same colour',
         doc['red_family']['pairs']['event_ripped|defect_conflict'], 2.8, 0.2),
        ('the closest rendered layer pair',
         doc['layers']['closest_pair'], 24.8, 0.2),
        ('two-layer crossings that impersonate a third layer',
         float(doc['crossings']['count']), 19.0, 0.0),
        ('the worst of them, B.Cu over F.Cu reading as In6',
         doc['crossings']['worst']['distance'], 5.1, 0.2),
        ('the board outline against the board body',
         doc['structure']['edge'], 12.33, 0.05),
    )
    bad = 0
    for label, got, want, tol in checks:
        if abs(got - want) > tol:
            bad += 1
            fail('%s: got %.4f, #946 says %.4f' % (label, got, want))
        else:
            print('    %-52s %8.2f' % (label, got))
    if not bad:
        print('  PASS: %d of #946\'s measured claims reproduced' % len(checks))


TESTS = (
    test_the_instrument_is_pinned_before_the_palette,
    test_every_floor_is_re_derived_from_the_shipped_palette,
    test_every_row_declares_what_kind_of_claim_it_is,
    test_the_baseline_agrees_key_by_key,
    test_the_audit_is_deterministic_across_hash_seeds,
    test_the_measured_story_of_946_is_reproduced,
)


def main():
    for fn in TESTS:
        print('%s:' % fn.__name__)
        fn()
    if _FAIL:
        print('\n%d FAILURE(S)' % len(_FAIL))
        for m in _FAIL:
            print('  - %s' % m)
        return 1
    print('\nall %d checks passed' % len(TESTS))
    return 0


if __name__ == '__main__':
    sys.exit(main())
