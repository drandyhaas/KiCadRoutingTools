#!/usr/bin/env python3
"""Does the #847 suite BITE? Mutate the engine and see what survives.

Same contract as `tests/mutate_834_835.py`, which this copies: a row is KILLED
by a failure OR an error; an anchor that does not match EXACTLY ONCE is BROKEN,
never silently skipped; `str.replace(old, new, 1)`, never `sed`; originals
restored in a `finally`; and it REFUSES to start on a dirty engine, because
restoring would write the committed text back over uncommitted work.

Every row carries an EXPECTATION. A mutation that is deliberately inert is
recorded as an expected survivor rather than deleted -- an inert row kept is a
finding, an inert row deleted is a hole. A row whose verdict does not match its
expectation is reported as WRONG and the run exits non-zero.

NOT named `test_*.py`, so `tests/run_all.py` never collects it: it rewrites
engine files in place. ONE WRITER PER TREE -- run it in a worktree of its own,
or nothing else may touch these files while it runs.

WHY THIS BATTERY EXISTS, specifically. #847's whole subject is a constant that
decided a gate while being reachable from no entry point and asserted by almost
nothing, so "the suite is green" was never evidence about it. Two rows here are
the ones that matter and both were found the hard way:

  * `the-row-hard-codes-the-tie-value` reproduces a hole an independent
    verifier found in the first draft of `test_847_escape_band.py`. Its
    flagship "the reported band is the band that was searched" arm ran at a
    basis where `4 * pitch == 1.0` EXACTLY, so a row hard-coding 1.0 satisfied
    it. The arm was degenerate on the one basis it was given. A second arm at a
    non-degenerate basis closed it, and this row is the change detector.

  * `the-share-form-goes-away` is the one that says whether #847 was actually
    fixed. Everything else could pass with the gate still blind.

    python3 -X utf8 tests/mutate_847.py
    python3 -X utf8 tests/mutate_847.py --list
    python3 -X utf8 tests/mutate_847.py --selftest
    python3 -X utf8 tests/mutate_847.py --verify-anchors
    python3 -X utf8 tests/mutate_847.py --row the-share-form-goes-away

RECORDED at f5bbfce9, with `wk/run7/glasgow_revC` present -- 22 rows, 20 killed,
2 survived (both of them the intended controls), 0 undecided, 0 broken, 0
disagreeing with expectation. (Same result at 229b5a5e, before the review
round; the rows that changed there were the two whose only witness is the
wk/-gated file, and they still kill with it present.)

The FIRST run of this battery, at ca47c88f, was 17 killed / 5 survived with
THREE disagreeing, and all three were holes in the tests rather than wrong
expectations. They are named here because a battery that only ever reports a
clean sweep is not evidence that it can find anything:

    the-baseline-is-graded-at-a-different-band   dropping the band from the
        BASELINE leg alone passed every arm. Closed with a board against
        ITSELF at a deep band -- the two sides are identical there, so the
        only way to make a NEW row is to grade them differently.
    lost_last_lane-goes-away   the honesty test drives that predicate on
        hand-built dicts, so nothing exercised its MERGE into the list the
        exit code reads. Closed with a CLI arm that disables the share form at
        a band where a face crosses to zero.
    the-share-form-drops-its-demand-conjunct   could not be closed on tracked
        boards at all: MEASURED, no face on the tigard pair has both demand
        < 7 and a >= 20% drop. The arm lives in the wk/-gated file where
        D21 W (demand 1, 8 -> 3) does, so on a clean clone this row is
        reported UNDECIDED -- excluded from the verdict rather than graded as
        a survivor, which is what a skip used to be and which failed the
        battery on every clean clone.

RUN `--verify-anchors` FIRST. An anchor stales the moment a line it quotes is
reworded, and a stale anchor reports BROKEN 50 minutes into a run rather than
in one second before it. It also refuses an anchor that matches PROSE
elsewhere in the tree: a comment quoting code has satisfied a grep-shaped test
in this repo before, and a battery that mutates a comment reports SURVIVED for
a mutation that changed nothing executable.
"""
import argparse
import glob
import os
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

ESC = os.path.join(ROOT, 'py_placer', 'placement', 'escape.py')
ROU = os.path.join(ROOT, 'py_placer', 'placement', 'routability.py')
CHK = os.path.join(ROOT, 'py_tools', 'check_channels.py')
TARGETS = {'esc': ESC, 'rou': ROU, 'chk': CHK}

T847 = os.path.join(ROOT, 'tests', 'test_847_escape_band.py')
TSTV = os.path.join(ROOT, 'tests', 'test_run8_starved_face_gate.py')
THON = os.path.join(ROOT, 'tests', 'test_run8_channels_gate_honesty.py')
TCH6 = os.path.join(ROOT, 'tests', 'test_run6_check_channels.py')

#: (name, target, old, new, tests, expected)
ROWS = [
    # ---- the resolver -----------------------------------------------------
    ('the-band-floor-changes', 'esc',
     'ESCAPE_BAND_FLOOR_MM = 1.0',
     'ESCAPE_BAND_FLOOR_MM = 0.5',
     (T847,), 'KILLED'),

    ('the-band-multiplier-changes', 'esc',
     'ESCAPE_BAND_LANES = 4.0',
     'ESCAPE_BAND_LANES = 3.0',
     (T847, TSTV), 'KILLED'),

    ('the-floor-swallows-the-lane-term', 'esc',
     """    scaled = lanes * float(lane_mm)
    if scaled >= floor_mm:""",
     """    scaled = lanes * float(lane_mm)
    if False:""",
     (T847,), 'KILLED'),

    ('the-lane-term-swallows-the-floor', 'esc',
     """    scaled = lanes * float(lane_mm)
    if scaled >= floor_mm:""",
     """    scaled = lanes * float(lane_mm)
    if True:""",
     (T847,), 'KILLED'),

    # The TIE. `>=` vs `>` changes only which TERM is reported at the exact
    # tie, never the band itself -- so nothing that reads the band can see it,
    # and only an arm that asserts the SOURCE can. That is the distinction
    # #847's framing turns on: at the fixture's own basis `4 * pitch` is
    # exactly 1.0, so the floor ties and decides nothing, and calling that
    # "the floor binds" points the fix at the wrong term.
    ('the-tie-is-attributed-to-the-floor', 'esc',
     '    if scaled >= floor_mm:',
     '    if scaled > floor_mm:',
     (T847,), 'KILLED'),

    ('the-explicit-override-is-ignored', 'esc',
     """    if override is not None:
        return EscapeBand(float(override), 'caller', basis, float(lane_mm),
                          lanes, floor_mm)""",
     """    if False:
        return EscapeBand(float(override), 'caller', basis, float(lane_mm),
                          lanes, floor_mm)""",
     (T847, TSTV), 'KILLED'),

    # ---- the wiring, both ledgers ----------------------------------------
    ('routability-stops-using-the-resolver', 'rou',
     '    band = _band.mm',
     '    band = max(1.0, 4 * pitch_routed)',
     (T847,), 'KILLED'),

    ('escape-stops-using-the-resolver', 'esc',
     """    band = escape_band(lane, basis='raw_lane', override=reach_mm)
    reach = band.mm""",
     """    band = escape_band(lane, basis='raw_lane', override=reach_mm)
    reach = max(lane * 4.0, 1.0)""",
     (T847,), 'KILLED'),

    # The verifier's M7, kept as a permanent row. It SURVIVED the first draft
    # of the test file because the flagship arm ran only at a basis where the
    # band is exactly 1.0.
    ('the-row-hard-codes-the-tie-value', 'rou',
     "                    'escape_band_mm': round(_band.mm, 4),",
     "                    'escape_band_mm': 1.0,",
     (T847,), 'KILLED'),

    ('the-reported-basis-is-invented', 'rou',
     "                         override=escape_band_mm)",
     "                         override=escape_band_mm)._replace("
     "basis='raw_lane')",
     (T847,), 'KILLED'),

    # ---- the CLI surface --------------------------------------------------
    ('the-cli-flag-never-reaches-the-ledger', 'chk',
     """            grid_step=grid, escape_band_mm=args.escape_band,
            pcb_file=args.board)""",
     """            grid_step=grid, escape_band_mm=None,
            pcb_file=args.board)""",
     (T847,), 'KILLED'),

    # The BASELINE leg. A gate is a delta, so grading the two sides at
    # different depths is not a stricter test -- it is a different one. Nothing
    # in the printed output would show it.
    ('the-baseline-is-graded-at-a-different-band', 'chk',
     """                grid_step=grid, escape_band_mm=args.escape_band,
                pcb_file=args.baseline)""",
     """                grid_step=grid, escape_band_mm=None,
                pcb_file=args.baseline)""",
     (T847,),
     # The battery caught this as a HOLE and the hole was closed rather than
     # the expectation re-recorded: a board against ITSELF at a deep band is
     # the discriminator, since the only way to produce a NEW row there is to
     # grade the two sides differently.
     'KILLED'),

    # ---- the predicate ----------------------------------------------------
    ('the-share-form-goes-away', 'chk',
     """        share = []
        if args.min_supply_drop > 0:""",
     """        share = []
        if False:""",
     (TSTV, T847), 'KILLED'),

    ('the-share-form-drops-its-demand-conjunct', 'chk',
     "            if before <= 0 or now >= before or r['demand_nets'] < min_demand:",
     '            if before <= 0 or now >= before:',
     (TSTV,),
     # MEASURED: no face on the tracked tigard pair has both demand < 7 and a
     # >= 20% drop, so that pair cannot discriminate this at all. The arm
     # lives in the wk/-gated file, where D21 W (demand 1, 8 -> 3) does. On a
     # clean clone this row is reported UNDECIDED -- its only witness skipped,
     # so it judged nothing. An earlier draft called that "a survivor, which is
     # the honest answer"; it was neither, it FAILED the battery on every clean
     # clone. A skip is its own bucket now, excluded from the verdict.
     'KILLED'),

    ('the-share-threshold-changes', 'chk',
     'GATE_MIN_SUPPLY_DROP = 0.20',
     'GATE_MIN_SUPPLY_DROP = 0.60',
     (T847,), 'KILLED'),

    ('the-share-threshold-goes-to-zero', 'chk',
     'GATE_MIN_SUPPLY_DROP = 0.20',
     'GATE_MIN_SUPPLY_DROP = 0.0',
     (T847, THON), 'KILLED'),

    ('the-demand-floor-changes', 'chk',
     'GATE_MIN_DEMAND = 7',
     'GATE_MIN_DEMAND = 40',
     (T847, TSTV), 'KILLED'),

    ('lost_last_lane-goes-away', 'chk',
     """        for ref, face, dem, before in lost_last_lane(ledgers, base_ledgers):
            if (ref, face) in seen:
                continue""",
     """        for ref, face, dem, before in []:
            if (ref, face) in seen:
                continue""",
     (THON, T847),
     # The honesty test drives `lost_last_lane` directly on hand-built dicts,
     # so it never saw the MERGE into the list the exit code reads. Closed
     # with a CLI arm that turns the share form off at a band where a face
     # crosses to zero, leaving this predicate as the only channel that can
     # fire.
     'KILLED'),

    ('the-gate-stops-returning-4', 'chk',
     """    if args.gate and new_starved:
        return 4""",
     """    if args.gate and new_starved:
        return 0""",
     (T847, TSTV), 'KILLED'),

    ('the-empty-ledger-exit-3-goes-away', 'chk',
     '        return 3',
     '        return 0',
     (THON,), 'KILLED'),

    # ---- controls ---------------------------------------------------------
    # A battery in which everything dies is not measuring the tests, it is
    # measuring whether the file still imports. These two change nothing
    # observable and MUST survive.
    ('the-band-is-spelled-via-a-local', 'rou',
     '    band = _band.mm',
     '    _b2 = _band.mm; band = _b2',
     (T847,), 'SURVIVED'),

    ('the-share-loop-is-spelled-with-an-index', 'chk',
     '        for ref, face, dem, before, now in share:',
     '        for _i in range(len(share)):\n'
     '            ref, face, dem, before, now = share[_i]',
     (T847,), 'SURVIVED'),
]


def _git_clean(paths):
    out = subprocess.run(['git', 'status', '--porcelain', '--'] + list(paths),
                         cwd=ROOT, capture_output=True, text=True).stdout
    return not out.strip()


def _drop_pyc():
    """CPython validates a `.pyc` on the source's mtime with ONE-SECOND
    granularity and its size. Two size-preserving rows applied inside the same
    second can leave the second import reading the FIRST mutant -- reported as
    a survivor for a row that was never really applied."""
    for pat in ('py_placer/placement/__pycache__/*.pyc',
                'py_router/__pycache__/*.pyc',
                'py_tools/__pycache__/*.pyc',
                'tests/__pycache__/*.pyc'):
        for f in glob.glob(os.path.join(ROOT, pat)):
            try:
                os.remove(f)
            except OSError:
                pass


def _run(tests):
    """(killed, why) -- killed when ANY named test exits non-zero.

    Exit 77 is a SELF-SKIP, not a kill. A test that could not find its fixture
    has not judged the mutation, and counting it as a kill would let a battery
    report a clean sweep on a tree where the load-bearing arms never ran.
    """
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1',
               PYTHONPATH=os.pathsep.join(
                   [ROOT, os.path.join(ROOT, 'py_router'),
                    os.path.join(ROOT, 'py_placer'),
                    os.path.join(ROOT, 'py_tools')]))
    skipped, ran = [], []
    for t in tests:
        r = subprocess.run([sys.executable, '-B', '-X', 'utf8', t],
                           cwd=ROOT, capture_output=True, text=True,
                           env=env, timeout=3600)
        if r.returncode == 77:
            skipped.append(os.path.basename(t))
            continue
        ran.append(os.path.basename(t))
        if r.returncode != 0:
            return 'KILLED', '{} exit {}'.format(os.path.basename(t),
                                                 r.returncode)
    if not ran:
        # UNDECIDED, not SURVIVED. Every witness self-skipped, so this row
        # judged nothing -- and grading it against its expectation fails the
        # battery on any clean clone for a reason that is about the FIXTURES,
        # not the code. Its own bucket, excluded from the verdict.
        return 'UNDECIDED', 'no witness ran: ' + ', '.join(skipped) + ' skipped'
    return 'SURVIVED', ('{} skipped'.format(', '.join(skipped))
                        if skipped else '')


def _apply(path, old, new):
    with open(path, encoding='utf-8') as fh:
        src = fh.read()
    n = src.count(old)
    if n != 1:
        return None, n
    with open(path, 'w', encoding='utf-8', newline='') as fh:
        fh.write(src.replace(old, new, 1))
    return src, 1


def _verify_anchors():
    """Every anchor matches its target exactly once. Run this FIRST.

    Also reports an anchor that is a SINGLE LINE short enough to plausibly
    appear in prose, so a reviewer can decide whether it is specific enough.
    A comment quoting code has satisfied a grep-shaped test in this repo
    before, and a battery that mutates a comment reports SURVIVED for a
    mutation that changed nothing executable.
    """
    bad = 0
    for name, tgt, old, _new, _tests, _exp in ROWS:
        with open(TARGETS[tgt], encoding='utf-8') as fh:
            n = fh.read().count(old)
        flag = '' if n == 1 else '  <-- STALE'
        if n != 1:
            bad += 1
        note = ''
        if '\n' not in old and len(old.strip()) < 30:
            note = '  (short single-line anchor -- check it is not prose)'
        print('{:<46} {:<4} matches {}{}{}'.format(name, tgt, n, flag, note))
    print()
    if bad:
        print('{} anchor(s) STALE. Fix them before running the battery -- a '
              'stale anchor reports BROKEN after the run instead of before '
              'it.'.format(bad))
        return 1
    print('all {} anchors match exactly once'.format(len(ROWS)))
    return 0


def _selftest():
    """The defences, exercised rather than asserted."""
    probe = os.path.join(ROOT, 'tests', '_mutate_847_probe.py')
    try:
        with open(probe, 'w', encoding='utf-8') as fh:
            fh.write('VALUE = 1\n')
        sys.path.insert(0, os.path.join(ROOT, 'tests'))
        import _mutate_847_probe as m
        assert m.VALUE == 1
        time.sleep(0.01)
        with open(probe, 'w', encoding='utf-8') as fh:
            fh.write('VALUE = 2\n')          # same size, same second
        _drop_pyc()
        import importlib
        importlib.reload(m)
        assert m.VALUE == 2, (
            'a size-preserving rewrite inside one second was not picked up; '
            '_drop_pyc is not doing its job and every verdict here is suspect')
    finally:
        for f in (probe, probe + 'c'):
            if os.path.exists(f):
                os.remove(f)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--row')
    ap.add_argument('--list', action='store_true')
    ap.add_argument('--selftest', action='store_true')
    ap.add_argument('--verify-anchors', action='store_true')
    args = ap.parse_args()

    if args.list:
        for name, tgt, _o, _n, tests, exp in ROWS:
            print('{:<46} {:<4} {:<9} {}'.format(
                name, tgt, exp, ' '.join(os.path.basename(t) for t in tests)))
        return 0

    if args.verify_anchors:
        return _verify_anchors()

    _selftest()
    if args.selftest:
        print('selftest OK: a size-preserving same-second rewrite is picked up')
        return 0

    if not _git_clean(TARGETS.values()):
        print('REFUSED: the target files are dirty. Restoring a mutation '
              'writes the COMMITTED text back, so uncommitted work here would '
              'be destroyed. Commit or stash first.')
        return 2

    if _verify_anchors():
        return 2

    rows = [r for r in ROWS if args.row is None or r[0] == args.row]
    if not rows:
        print('no row named {!r}'.format(args.row))
        return 2

    # The battery is only evidence if the gate passes UNMUTATED first.
    _drop_pyc()
    got0, why0 = _run((T847, TSTV))
    if got0 == 'KILLED':
        print('BROKEN: the gate does not pass on the UNMUTATED tree ({}). '
              'Every verdict below would be meaningless.'.format(why0))
        return 2
    if why0:
        print('NOTE: on the unmutated tree, {}. Rows whose ONLY witness is a '
              'skipped file are reported UNDECIDED below, never as '
              'survivors.'.format(why0))

    verdicts, broken, wrong, undecided = [], [], [], []
    for name, tgt, old, new, tests, exp in rows:
        path = TARGETS[tgt]
        _drop_pyc()
        src, n = _apply(path, old, new)
        if src is None:
            broken.append('{} (anchor matched {}x)'.format(name, n))
            print('{:<46} BROKEN   anchor matched {}x'.format(name, n))
            continue
        try:
            _drop_pyc()
            got, why = _run(tests)
        finally:
            with open(path, 'w', encoding='utf-8', newline='') as fh:
                fh.write(src)
            _drop_pyc()
        verdicts.append((name, got))
        mark = ''
        if got == 'UNDECIDED':
            undecided.append(name)
        elif got != exp:
            wrong.append('{}: expected {}, got {}'.format(name, exp, got))
            mark = 'WRONG'
        print('{:<46} {:<10} {:<8} {}'.format(name, got, mark, why))

    n_k = sum(1 for _n, g in verdicts if g == 'KILLED')
    n_s = sum(1 for _n, g in verdicts if g == 'SURVIVED')
    print('\n{} row(s): {} killed, {} survived, {} undecided (no witness '
          'ran), {} broken, {} disagreeing with expectation'
          .format(len(rows), n_k, n_s, len(undecided), len(broken),
                  len(wrong)))
    for u in undecided:
        print('  UNDECIDED (needs wk/): ' + u)
    for w in wrong:
        print('  WRONG: ' + w)
    for b in broken:
        print('  BROKEN: ' + b)
    return 1 if (wrong or broken) else 0


if __name__ == '__main__':
    sys.exit(main())
