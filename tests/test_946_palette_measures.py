#!/usr/bin/env python3
"""Every shipped palette carries its own justification (#946).

This gate does TWO things, and it does both deliberately.

1. It **RE-DERIVES** every number from the shipped palettes, through
   `palette_audit`'s own transforms, and checks them against declared floors.
   Change one RGB triple and this fails immediately, naming the number that
   broke. A palette cannot drift away from its justification.

2. It **COMPARES** those derived numbers, per key and per theme, against the
   committed `tests/946_theme_contrast_baseline.json`, reporting DRIFT and
   INVERTED apart from each other, plus ORPHAN and MALFORMED.

Why both. A threshold test alone says "still above 4.5" and would pass a margin
that had silently collapsed from 12.0x to 4.6x -- exactly what a reviewer most
wants to know and what an inequality cannot report. A baseline alone says "the
number moved" and cannot say whether the new number is acceptable, so
regenerating it launders a regression. Those are precisely the two failure modes
#694 produced -- `corridor-ulx3s` sat rejected on a recorded claim whose signal
had reversed while the gate printed PASS -- and CLAUDE.md's rule that came out
of it is what this file implements for colour.

**THE INSTRUMENT IS CHECKED BEFORE THE PALETTE.** `palette_audit --self-test`
pins the WCAG and Vienot transforms against published fixtures. Without that a
broken transform reads as a broken palette, and the two have different fixes.

**EVERY FLOOR IS ONE A SHIPPING ARM ALREADY CLEARS.** Twice in this work a
floor was invented instead of derived, and both times a gate caught it:

  * five rows were marked as RATCHETS that were really INVARIANTS -- dark
    scoring 12.33x against a floor of 4.5x that LIGHT failed at 1.01x. Read as
    a ratchet, "structure.edge 12.33 -> 4.5" looks like a plan to regress dark.
  * T4 (an event must not be mistaken for a composited layer) was set at 80,
    and EVERY light candidate failed -- as does the dark theme already
    shipping, at **38.0**. A floor no arm clears is not a floor.
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

#: (label, getter, op, floor, kind, target, owner)
#:
#: `floor` must be cleared by EVERY shipped theme. `kind` is:
#:
#:   'ratchet'    an arm still fails the eventual target; `owner` moves it, and
#:                edits this row in the same commit.
#:   'invariant'  every arm clears it today and must keep clearing it.
FLOORS = (
    ('events.contrast_min',
     lambda d: d['events']['contrast_min'], 'min', 4.7,
     'invariant', 4.5, 'both arms; dark 4.74, light 4.82'),
    ('events.rip_restore_deuteranope',
     lambda d: d['events']['rip_restore_deuteranope'], 'min', 150.0,
     'invariant', 150.0,
     '#1013 ACHIEVED: dark 76 -> 187 via cyan; light was already 154'),
    ('events.pair_deuteranope_min',
     lambda d: d['events']['pair_deuteranope_min'], 'min', 89.0,
     'invariant', 89.0,
     '#1013: dark 76 -> 89.5 (restored vs new is now the weakest pair, not '
     'restored vs ripped); light 120'),
    ('events.rip_restore_luminance_ratio',
     lambda d: d['events']['rip_restore_luminance_ratio'], 'min', 1.6,
     'invariant', 1.6,
     'dark separates on luminance, light on the blue axis; both need SOME'),
    ('structure.edge',
     lambda d: d['structure']['edge'], 'min', 4.5,
     'invariant', 4.5, '#1012 fixed light, which scored 1.01x -- no outline'),
    ('structure.pad',
     lambda d: d['structure']['pad'], 'min', 3.0,
     'invariant', 3.0, '#1012 fixed light, which scored 1.79x'),
    ('structure.via',
     lambda d: d['structure']['via'], 'min', 3.0,
     'invariant', 3.0, '#1012 fixed light, which scored 1.66x'),
    ('red_family.min',
     lambda d: d['red_family']['min'], 'min', 40.0,
     'invariant', 40.0,
     '#1012 took dark from 2.8 to 57.1 -- defects left the red family'),
    ('layers.closest_pair',
     lambda d: d['layers']['closest_pair'], 'min', 24.0,
     'invariant', 24.0, 'compositing preserves it on either ground'),
    ('layers.contrast_min',
     lambda d: d['layers']['contrast_min'], 'min', 1.96,
     'invariant', 1.96,
     '#1012 fixed light via k=0.74 at alpha 205; it scored 1.19x'),
    ('crossings.count',
     lambda d: d['crossings']['count'], 'max', 21,
     'ratchet', 0, '#1015 -- opaque crossings'),
)

#: Compared against the baseline but carrying no floor of their own.
EXTRA_COMPARED = ('layers.mean_pair', 'layers.contrast_mean',
                  'crossings.worst_distance', 'structure.board_vs_ground')
COMPARED = tuple(r[0] for r in FLOORS) + EXTRA_COMPARED

_FAIL = []


def fail(msg):
    _FAIL.append(msg)
    print('  FAIL: %s' % msg)


def get(doc, label):
    if label == 'crossings.worst_distance':
        w = doc['crossings']['worst']
        return w['distance'] if w else 0.0
    node = doc
    for part in label.split('.'):
        node = node[part]
    return node


def docs():
    return [PA.audit(p) for p in PA.shipped_palettes()]


def test_the_instrument_is_pinned_before_the_palette():
    if PA.self_test(quiet=True):
        fail('palette_audit --self-test failed; every number below is suspect')
        return
    n = len(PA.SELF_TEST_FIXTURES)
    if n < 5:
        fail('only %d self-test fixtures; that is not a pinned instrument' % n)
        return
    print('  PASS: %d transforms pinned before any palette was measured' % n)


def test_every_floor_is_cleared_by_every_shipped_theme():
    ds = docs()
    if len(ds) < 2:
        fail('only %d shipped theme(s) -- #1012 ships two, and a gate that '
             'iterates one arm cannot see a second arm regress' % len(ds))
        return
    names = ' '.join('%9s' % d['palette'] for d in ds)
    print('    %-36s %s   floor' % ('', names))
    bad = 0
    for label, getter, op, floor, kind, target, owner in FLOORS:
        vals = [getter(d) for d in ds]
        arrow = '>=' if op == 'min' else '<='
        ok = all((v >= floor - 1e-6) if op == 'min' else (v <= floor + 1e-6)
                 for v in vals)
        if not ok:
            bad += 1
            worst = min(vals) if op == 'min' else max(vals)
            fail('%s = %s on one arm, floor is %s %s (%s -> %s, %s)'
                 % (label, worst, arrow, floor, kind, target, owner))
        else:
            cells = ' '.join('%9s' % v for v in vals)
            print('    %-36s %s  %s %-6s %-10s %s'
                  % (label, cells, arrow, floor, kind, owner))
    if not bad:
        print('  PASS: %d floors x %d themes' % (len(FLOORS), len(ds)))


def test_every_row_declares_what_kind_of_claim_it_is():
    """A ratchet whose target is not tighter is decoration; an invariant whose
    floor no arm clears is not a floor. Both errors happened in this work."""
    ds = docs()
    kinds = {}
    for label, getter, op, floor, kind, target, owner in FLOORS:
        kinds[kind] = kinds.get(kind, 0) + 1
        if kind == 'ratchet':
            tighter = (target > floor) if op == 'min' else (target < floor)
            if not tighter:
                fail('%s is a ratchet but %s is not tighter than %s'
                     % (label, target, floor))
        elif kind == 'invariant':
            vals = [getter(d) for d in ds]
            clears = all((v >= floor - 1e-6) if op == 'min'
                         else (v <= floor + 1e-6) for v in vals)
            if not clears:
                fail('%s is an invariant but an arm does not clear %s -- that '
                     'is a ratchet, or an invented floor' % (label, floor))
        else:
            fail('%s has kind %r, neither ratchet nor invariant'
                 % (label, kind))
        if not owner.strip():
            fail('%s declares no owner' % label)
    # Ratchets are CONSUMED as phases land: #1013 turned two of them into
    # invariants by achieving them, and zero is the correct end state once
    # every planned improvement has shipped. What must never happen is a
    # ratchet that is not tighter than its floor, or an invariant no arm
    # clears -- both checked above. The count is only a reminder that a table
    # of pure invariants is a table that has stopped planning anything.
    if kinds.get('ratchet', 0) == 0:
        print('    note: no ratchets left -- every planned improvement has '
              'landed, or this table has stopped planning')
    if not _FAIL:
        print('  PASS: %d ratchets, %d invariants, every row owned'
              % (kinds.get('ratchet', 0), kinds.get('invariant', 0)))


def test_the_baseline_agrees_key_by_key_and_theme_by_theme():
    if not os.path.exists(BASELINE):
        fail('no baseline at %s. A MISSING BASELINE IS A FAILURE, NOT A PASS '
             '-- write it with `palette_audit --write-baseline`.' % BASELINE)
        return
    try:
        base = json.loads(io.open(BASELINE, encoding='utf-8').read())
    except ValueError as exc:
        fail('MALFORMED baseline (%s)' % exc)
        return
    themes = base.get('themes')
    if not isinstance(themes, list) or not themes:
        fail('MALFORMED baseline: no `themes` list')
        return
    by_name = {}
    for t in themes:
        if t.get('kind') != 'palette-audit' or t.get('schema') != 1:
            fail('MALFORMED baseline entry: kind=%r schema=%r'
                 % (t.get('kind'), t.get('schema')))
            return
        by_name[t['palette']] = t
    ds = docs()
    live = [d['palette'] for d in ds]
    for doc in ds:
        name = doc['palette']
        want_doc = by_name.get(name)
        if want_doc is None:
            fail('ORPHAN: the baseline has no arm named %r' % name)
            continue
        for label in COMPARED:
            try:
                want = get(want_doc, label)
            except (KeyError, TypeError):
                fail('ORPHAN: baseline arm %r has no %s' % (name, label))
                continue
            got = get(doc, label)
            if abs(float(got) - float(want)) <= 1e-4:
                continue
            row = [r for r in FLOORS if r[0] == label]
            if row:
                op = row[0][2]
                worse = (got < want) if op == 'min' else (got > want)
                if worse:
                    fail('INVERTED: %s on %s moved the wrong way, %s -> %s'
                         % (label, name, want, got))
                    continue
            fail('DRIFT: %s on %s %s -> %s (re-record with --write-baseline '
                 'IN THE SAME COMMIT as the palette change, after reading the '
                 'floors table)' % (label, name, want, got))
    for name in by_name:
        if name not in live:
            fail('ORPHAN: the baseline has arm %r that is no longer shipped'
                 % name)
    if not _FAIL:
        print('  PASS: %d keys x %d arms agree with the baseline'
              % (len(COMPARED), len(ds)))


def test_the_audit_is_deterministic_across_hash_seeds():
    out = []
    for seed in ('0', '12345'):
        env = dict(os.environ)
        env['PYTHONHASHSEED'] = seed
        env['PYTHONPATH'] = os.pathsep.join(
            [os.path.join(ROOT, 'py_router'), env.get('PYTHONPATH', '')])
        r = subprocess.run(
            [sys.executable, '-X', 'utf8', '-c',
             'import json,sys,palette_audit as p;'
             'json.dump([p.audit(x) for x in p.shipped_palettes()],'
             ' sys.stdout, sort_keys=True)'],
            capture_output=True, text=True, encoding='utf-8',
            errors='replace', env=env, cwd=ROOT)
        if r.returncode != 0:
            fail('audit subprocess at PYTHONHASHSEED=%s exited %d: %s'
                 % (seed, r.returncode, (r.stderr or '')[-400:]))
            return
        out.append(r.stdout)
    if not out[0]:
        fail('BROKEN: the subprocess produced nothing to compare')
        return
    if out[0] != out[1]:
        fail('the audit differs by PYTHONHASHSEED -- a set() or an unordered '
             'comprehension is in a palette path')
        return
    print('  PASS: identical across PYTHONHASHSEED 0 and 12345')


def test_the_measured_story_of_946_is_reproduced():
    """The claims #946 makes, re-derived on the DARK arm -- the arm it
    measured. If one moves, either the palette changed or the issue was wrong,
    and both are worth stopping for."""
    d = PA.audit('dark')
    checks = (
        # #946's headline was 76. #1013 fixed it, and the HISTORICAL number is
        # still pinned -- by `palette_audit --self-test`, whose fixture
        # measures the literal green against the literal rip. That is the
        # right home for it: it is a property of the TRANSFORM plus two
        # retired constants, not of the shipping palette.
        ('rip vs restore, after #1013 moved restore to cyan',
         d['events']['rip_restore_deuteranope'], 186.6, 1.0),
        ('the closest rendered layer pair',
         d['layers']['closest_pair'], 24.8, 0.2),
        ('two-layer crossings impersonating a third layer',
         float(d['crossings']['count']), 19.0, 0.0),
        ('the worst of them, B.Cu over F.Cu reading as In6',
         d['crossings']['worst']['distance'], 5.1, 0.2),
        ('the board outline against the dark board body',
         d['structure']['edge'], 12.33, 0.05),
    )
    bad = 0
    for label, got, want, tol in checks:
        if abs(got - want) > tol:
            bad += 1
            fail('%s: got %.4f, #946 says %.4f' % (label, got, want))
        else:
            print('    %-52s %8.2f' % (label, got))
    if PA.rgb_distance(PA.current_palette('dark')['event_restored'],
                       (80, 215, 230)) > 0.5:
        fail('the dark restore is not the cyan #1013 measured')
    lt = PA.audit('light')
    if d['red_family']['min'] < 40 or lt['red_family']['min'] < 40:
        fail('red still means four things: dark %.1f light %.1f'
             % (d['red_family']['min'], lt['red_family']['min']))
    else:
        print('    %-52s %5.1f / %5.1f'
              % ('the four reds, dark / light (was 2.8)',
                 d['red_family']['min'], lt['red_family']['min']))
    if not bad and not _FAIL:
        print("  PASS: #946's measured claims reproduced, and the one it "
              "opened on is fixed")


TESTS = (
    test_the_instrument_is_pinned_before_the_palette,
    test_every_floor_is_cleared_by_every_shipped_theme,
    test_every_row_declares_what_kind_of_claim_it_is,
    test_the_baseline_agrees_key_by_key_and_theme_by_theme,
    test_the_audit_is_deterministic_across_hash_seeds,
    test_the_measured_story_of_946_is_reproduced,
)


def main():
    for fn in TESTS:
        print('%s:' % fn.__name__)
        fn()
    if _FAIL:
        print('')
        print('%d FAILURE(S)' % len(_FAIL))
        for m in _FAIL:
            print('  - %s' % m)
        return 1
    print('')
    print('all %d checks passed' % len(TESTS))
    return 0


if __name__ == '__main__':
    sys.exit(main())
