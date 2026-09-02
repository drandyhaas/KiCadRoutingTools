#!/usr/bin/env python3
"""Does the #834/#835 suite BITE? Mutate the engine and see what survives.

Same contract as `tests/mutate_761.py`, which this copies: a row is KILLED by a
failure OR an error; an anchor that does not match EXACTLY ONCE is BROKEN,
never silently skipped; `str.replace(old, new, 1)`, never `sed`; originals
restored in a `finally`; and it REFUSES to start on a dirty engine, because
restoring would write the committed text back over uncommitted work.

Every row carries an EXPECTATION. A mutation that is deliberately inert is
recorded as an expected survivor rather than deleted -- an inert row kept is a
finding, an inert row deleted is a hole. A row whose verdict does not match its
expectation is reported as WRONG.

NOT named `test_*.py`, so `tests/run_all.py` never collects it: it rewrites
engine files in place. ONE WRITER PER TREE -- run it in a worktree of its own,
or nothing else may touch these files while it runs. (Measured the hard way on
this very branch: an edit made to `legality.py` while this was running was
overwritten by the restore, and killing it mid-row left a mutant on disk.)

    python3 tests/mutate_834_835.py
    python3 tests/mutate_834_835.py --row the-side-filter-goes-away
    python3 tests/mutate_834_835.py --list
    python3 tests/mutate_834_835.py --selftest

RECORDED at 49fff127 -- 16 rows, 12 killed, 4 survived (4 of them expected),
0 broken, 0 disagreeing with expectation:

    pside-goes-back-to-back-only                  KILLED
    pad_sides-uses-the-BODY-answer                KILLED
    the-disjoint-early-return-goes-away           KILLED
    the-early-return-forgets-the-hole-channel     KILLED
    the-cap-branch-goes-back-to-the-whole-extent  KILLED
    the-per-side-extent-ignores-the-side          KILLED
    the-per-side-extent-drops-the-holes           SURVIVED  (expected)
    the-side-filter-goes-away                     KILLED
    the-side-filter-becomes-one-sided             SURVIVED  (expected: a no-op
                                                             control)
    the-container-filter-goes-away                KILLED
    span_eaten-sums-instead-of-unioning           KILLED
    span_eaten-drops-the-clamp                    SURVIVED  (expected, proved)
    the-sides-map-is-not-threaded-into-the-ledger SURVIVED  (expected)
    routability-keeps-its-one-sided-side-test     KILLED
    routability-goes-back-to-double-charging      KILLED
    routability-stops-exempting-containers        KILLED

The FIRST run of this table had four disagreements, three of which were holes
in the new tests rather than in the engine -- see the commit that fixed them.
That is the whole argument for the battery: the suite was green before it ran.
"""
import argparse
import glob
import os
import shutil
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

LEG = os.path.join(ROOT, 'py_placer', 'placement', 'legality.py')
ESC = os.path.join(ROOT, 'py_placer', 'placement', 'escape.py')
ROU = os.path.join(ROOT, 'py_placer', 'placement', 'routability.py')
TARGETS = {'leg': LEG, 'esc': ESC, 'rou': ROU}

T834 = os.path.join(ROOT, 'tests', 'test_834_cap_branch_side.py')
T835 = os.path.join(ROOT, 'tests', 'test_835_escape_side_aware.py')
T841 = os.path.join(ROOT, 'tests', 'test_841_obstruction_rect.py')
T700 = os.path.join(ROOT, 'tests', 'test_700_layer_term.py')
T761 = os.path.join(ROOT, 'tests', 'test_761_legality_npth_keepout.py')

#: (name, target, old, new, tests, expected)
ROWS = [
    # ---- #834: the pad channel ------------------------------------------
    ('pside-goes-back-to-back-only', 'leg',
     """            _star = any(str(l).startswith('*') for l in copper)
            _b = any(str(l).startswith('B') for l in copper)
            _f = any(not str(l).startswith(('B', '*')) for l in copper)
            pside = (None if (through or _star or (_b and _f))
                     else ('B' if _b else 'F'))""",
     """            pside = None if through else (
                'B' if any(str(l).startswith('B') for l in copper) else 'F')""",
     (T834,), 'KILLED'),

    ('pad_sides-uses-the-BODY-answer', 'leg',
     """        self.pad_sides = (BOTH_SIDES if _sides is BOTH_SIDES else
                          FRONT_ONLY if _sides == {'F'} else
                          BACK_ONLY if _sides == {'B'} else
                          BOTH_SIDES if _sides else NO_SIDES)""",
     '        self.pad_sides = sides_occupied(self.side, self.has_tht)',
     (T834,), 'KILLED'),

    ('the-disjoint-early-return-goes-away', 'leg',
     """        if not (pa.pad_sides & pb.pad_sides):
            if not pa.holes_local and not pb.holes_local:
                return ZERO_SHORTFALL""",
     """        if False:
            if not pa.holes_local and not pb.holes_local:
                return ZERO_SHORTFALL""",
     (T834,), 'KILLED'),

    ('the-early-return-forgets-the-hole-channel', 'leg',
     """        if not (pa.pad_sides & pb.pad_sides):
            if not pa.holes_local and not pb.holes_local:
                return ZERO_SHORTFALL""",
     """        if not (pa.pad_sides & pb.pad_sides):
            if True:
                return ZERO_SHORTFALL""",
     (T834,), 'KILLED'),

    ('the-cap-branch-goes-back-to-the-whole-extent', 'leg',
     """            g = min((rect_gap(pa.extent_side(xa, ya, ra, s),
                              pb.extent_side(xb, yb, rb, s))
                     for s in ('F', 'B')
                     if pa.extent_side(xa, ya, ra, s) is not None
                     and pb.extent_side(xb, yb, rb, s) is not None),
                    default=rect_gap(ea, eb))""",
     '            g = rect_gap(ea, eb)',
     (T834,), 'KILLED'),

    ('the-per-side-extent-ignores-the-side', 'leg',
     """                if not _sides_interact(_s, side):
                    continue""",
     """                if False:
                    continue""",
     (T834,), 'KILLED'),

    ('the-per-side-extent-drops-the-holes', 'leg',
     """            rad = math.radians(-key[0])
            hc, hs = math.cos(rad), math.sin(rad)
            for ox, oy, r in self.holes_extent:
                cx, cy = ox * hc - oy * hs, ox * hs + oy * hc
                xs0.append(cx - r); ys0.append(cy - r)
                xs1.append(cx + r); ys1.append(cy + r)
            ext = () if not xs0 else (min(xs0), min(ys0), max(xs1), max(ys1))
            self._ext_side_cache[key] = ext""",
     """            ext = () if not xs0 else (min(xs0), min(ys0), max(xs1), max(ys1))
            self._ext_side_cache[key] = ext""",
     (T834, T761),
     # A hole is only in a per-side box for a part that HAS one, and no
     # fixture puts a holed part on the cap branch's per-side path: over the
     # cap the disjoint case returns before it, and the mixed fixture carries
     # no NPTH. The corpus does have a mixed-side part above the cap --
     # glasgow_revC's U1, whose B box is strictly smaller than its whole
     # extent -- but its over-cap partner U30 is front-only, so the shared
     # face is F and the boxes coincide there. Recorded rather than deleted,
     # because it stops surviving the day either case changes, and because a
     # silent drop here would be a real defect on a board that has one.
     'SURVIVED'),

    # ---- #835: the escape ledger ----------------------------------------
    ('the-side-filter-goes-away', 'esc',
     """        if own is not None:
            oth = sides.get(other)
            if oth is not None and not (own & oth):
                continue""",
     """        if False:
            oth = sides.get(other)
            if oth is not None and not (own & oth):
                continue""",
     (T835,), 'KILLED'),

    ('the-side-filter-becomes-one-sided', 'esc',
     '            if oth is not None and not (own & oth):',
     '            if oth is not None and not (sides.get(ref) & oth):',
     (T835,),
     # `own` IS `sides.get(ref)`, so this is the same expression spelled
     # twice. Kept as the CONTROL: a row that mutates the line and changes
     # nothing proves the harness is not simply killing everything it touches.
     'SURVIVED'),

    ('the-container-filter-goes-away', 'esc',
     """        if containers is not None and other in containers:
            continue""",
     """        if False:
            continue""",
     (T835, T700), 'KILLED'),

    ('span_eaten-sums-instead-of-unioning', 'esc',
     """    intervals.sort()
    blocked = 0.0
    cur_a = cur_b = None""",
     """    intervals.sort()
    blocked = sum(b - a for a, b, _n in intervals)
    cur_a = cur_b = None""",
     (T835,), 'KILLED'),

    ('span_eaten-drops-the-clamp', 'esc',
     '    return min(blocked, hi - lo), order',
     '    return blocked, order',
     (T835,),
     # SURVIVES, and by algebra rather than by a gap in the tests: every
     # interval is clipped to [lo, hi] as it is built, so their UNION cannot
     # measure more than hi - lo and the clamp never binds. It binds the moment
     # the union is removed -- which is the row above -- so it is kept, and
     # this row is kept with it, because a future edit that reorders the
     # clipping makes the clamp live again. Expected first, then measured, not
     # the other way round: the first version of this row expected KILLED and
     # the battery corrected it.
     'SURVIVED'),

    ('the-sides-map-is-not-threaded-into-the-ledger', 'esc',
     """                       sides=sides, containers=containers,
                       obstruction_rects=orects)""",
     """                       sides=None, containers=containers,
                       obstruction_rects=orects)""",
     (T835,),
     # Inert by construction: `part_escape` builds its own map when given
     # None. Recorded so that if the fallback is ever removed -- making the
     # public entry point silently side-blind -- this row starts killing.
     'SURVIVED'),

    # ---- the reconciliation ---------------------------------------------
    ('routability-keeps-its-one-sided-side-test', 'rou',
     """    neighbors = [(g.ref, _geom[g.ref].rect) for g in _graded
                 if g.ref != ref and (own_sides & g.sides)
                 and g.ref not in _containers and g.ref in _geom]""",
     """    neighbors = [(g.ref, _geom[g.ref].rect) for g in _graded
                 if g.ref != ref and (footprint_side(fp) in g.sides)
                 and g.ref not in _containers and g.ref in _geom]""",
     (T835,), 'KILLED'),

    ('routability-goes-back-to-double-charging', 'rou',
     """        covered, order = span_eaten(lo, hi, band_across, horiz, neighbors)""",
     """        covered = 0.0
        order = []
        for _nref, rct in neighbors:
            across = (rct[1], rct[3]) if horiz else (rct[0], rct[2])
            if across[1] < band_across[0] or across[0] > band_across[1]:
                continue
            sp = ((min(rct[2], hi) - max(rct[0], lo)) if horiz
                  else (min(rct[3], hi) - max(rct[1], lo)))
            if sp <= 0:
                continue
            covered += sp
            order.append((_nref, sp))""",
     (T835,), 'KILLED'),

    # #841. Without this row nothing in the battery notices `face_lane_ledger`
    # going back to billing routing for assembly margin: the deficit GROWS,
    # so every "is there a deficit" arm is satisfied harder.
    ('routability-goes-back-to-the-courtyard', 'rou',
     """    neighbors = [(g.ref, _geom[g.ref].rect) for g in _graded""",
     """    neighbors = [(g.ref, g.rect) for g in _graded""",
     (T835,), 'KILLED'),

    # #841, the other direction. `escape` charged the bbox of pad CENTRES, so
    # this is the exact code that shipped before -- and the per-board table is
    # what has to catch it.
    ('escape-neighbour-goes-back-to-pad-centres', 'esc',
     """        obstacles.append((other, g.rect if g is not None else _part_rect(ofp)))""",
     """        obstacles.append((other, _part_rect(ofp)))""",
     (T835,), 'KILLED'),

    # #841. The pairing that keeps face ASSIGNMENT invariant while the box it
    # measures against grows. Measured, this alone makes all 244 of one corpus
    # board's pads interior, and the ledger then reports no demand at all --
    # every deficit number goes green DOWNWARD, which is why the demand arm
    # exists to kill it.
    ('escape-face-assignment-forgets-the-pad-edge', 'esc',
     """        box = None if own is None else _pad_box(own, pad)""",
     """        box = None""",
     (T835,), 'KILLED'),

    # #841. The subject rect. Separable from the row above on purpose: one
    # moves the face, the other moves which pads are on it.
    ('escape-subject-rect-goes-back-to-pad-centres', 'esc',
     """    rect = own.rect if own is not None else _part_rect(fp)""",
     """    rect = _part_rect(fp)""",
     (T835,), 'KILLED'),

    # #841, and the two rows the battery would have wanted six commits ago:
    # `copper` was built from the LOOKUP DICT, which is keyed per pad, so pads
    # stacked at one point collapsed and the union silently lost them. Every
    # arm in test_841 passed throughout, because the one that pins the
    # "every edge is attained by a pad" property is true by construction over
    # whatever survived the dict. A blind code review found it; these rows are
    # what would have.
    ('copper-goes-back-to-the-collapsed-dict', 'leg',
     """        if boxes:
            copper = (min(b[0] for b in boxes), min(b[1] for b in boxes),
                      max(b[2] for b in boxes), max(b[3] for b in boxes))""",
     """        if boxes:
            _v = list(rects.values()) if False else boxes
            _seen = {}
            for _b in boxes:
                _seen[(round((_b[0] + _b[2]) / 2.0, 4),
                       round((_b[1] + _b[3]) / 2.0, 4))] = _b
            _v = list(_seen.values())
            copper = (min(b[0] for b in _v), min(b[1] for b in _v),
                      max(b[2] for b in _v), max(b[3] for b in _v))""",
     (T841,), 'KILLED'),

    ('the-pad-lookup-key-drops-the-size', 'leg',
     """    hx, hy = pad_half_extents(pad)
    return geom.pads.get((round(pad.global_x, 4), round(pad.global_y, 4),
                          round(hx, 4), round(hy, 4)))""",
     """    for _k, _v in geom.pads.items():
        if (abs(_k[0] - round(pad.global_x, 4)) < 1e-9
                and abs(_k[1] - round(pad.global_y, 4)) < 1e-9):
            return _v
    return None""",
     (T841,), 'KILLED'),

    ('routability-stops-exempting-containers', 'rou',
     """                 and g.ref not in _containers and g.ref in _geom]""",
     """                 and g.ref not in () and g.ref in _geom]""",
     (T835,),
     # Expected SURVIVED on the reasoning that only `escape` is asserted on;
     # measured KILLED, because `test_face_lane_ledger_side_test_is_symmetric`
     # asserts the container is absent from THIS ledger's `eaten_by` too.
     'KILLED'),
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
    """(killed, why) -- killed when ANY named test exits non-zero."""
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1',
               PYTHONPATH=os.pathsep.join(
                   [ROOT, os.path.join(ROOT, 'py_router'),
                    os.path.join(ROOT, 'py_placer'),
                    os.path.join(ROOT, 'py_tools')]))
    for t in tests:
        r = subprocess.run([sys.executable, '-B', '-X', 'utf8', t],
                           cwd=ROOT, capture_output=True, text=True,
                           env=env, timeout=3600)
        if r.returncode != 0:
            return True, '{} exit {}'.format(os.path.basename(t),
                                             r.returncode)
    return False, ''


def _apply(path, old, new):
    with open(path, encoding='utf-8') as fh:
        src = fh.read()
    n = src.count(old)
    if n != 1:
        return None, n
    with open(path, 'w', encoding='utf-8', newline='') as fh:
        fh.write(src.replace(old, new, 1))
    return src, 1


def _selftest():
    """The defences, exercised rather than asserted.

    `_drop_pyc` is the one that matters: without it a size-preserving row can
    be reported as a survivor because the interpreter never re-read the file.
    """
    probe = os.path.join(ROOT, 'tests', '_mutate_834_probe.py')
    try:
        with open(probe, 'w', encoding='utf-8') as fh:
            fh.write('VALUE = 1\n')
        sys.path.insert(0, os.path.join(ROOT, 'tests'))
        import _mutate_834_probe as m
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
        shutil.rmtree(os.path.join(ROOT, 'tests', '__pycache__'),
                      ignore_errors=True)
    # ...and the anchor rule: a pattern matching twice is BROKEN, not applied.
    tmp = os.path.join(ROOT, 'tests', '_mutate_834_anchor.py')
    try:
        with open(tmp, 'w', encoding='utf-8') as fh:
            fh.write('x = 1\nx = 1\n')
        src, n = _apply(tmp, 'x = 1\n', 'x = 2\n')
        assert src is None and n == 2, (src, n)
    finally:
        if os.path.exists(tmp):
            os.remove(tmp)
    print('selftest OK: stale-pyc defence works, and a doubled anchor is '
          'refused rather than applied')


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--row')
    ap.add_argument('--list', action='store_true')
    ap.add_argument('--selftest', action='store_true')
    args = ap.parse_args()

    if args.list:
        for name, tgt, _o, _n, tests, exp in ROWS:
            print('{:<48} {:<4} {:<9} {}'.format(
                name, tgt, exp, ' '.join(os.path.basename(t) for t in tests)))
        return 0

    _selftest()
    if args.selftest:
        return 0

    if not _git_clean(TARGETS.values()):
        print('REFUSED: the target files are dirty. Restoring a mutation '
              'writes the COMMITTED text back, so uncommitted work here would '
              'be destroyed. Commit or stash first.')
        return 2

    rows = [r for r in ROWS if args.row is None or r[0] == args.row]
    if not rows:
        print('no row named {!r}'.format(args.row))
        return 2

    # The battery is only evidence if the gate passes UNMUTATED first.
    _drop_pyc()
    killed0, why0 = _run((T834, T835))
    if killed0:
        print('BROKEN: the gate does not pass on the UNMUTATED tree ({}). '
              'Every verdict below would be meaningless.'.format(why0))
        return 2

    verdicts, broken, wrong = [], [], []
    for name, tgt, old, new, tests, exp in rows:
        path = TARGETS[tgt]
        _drop_pyc()
        src, n = _apply(path, old, new)
        if src is None:
            broken.append('{} (anchor matched {}x)'.format(name, n))
            print('{:<48} BROKEN   anchor matched {}x'.format(name, n))
            continue
        try:
            _drop_pyc()
            killed, why = _run(tests)
        finally:
            with open(path, 'w', encoding='utf-8', newline='') as fh:
                fh.write(src)
            _drop_pyc()
        got = 'KILLED' if killed else 'SURVIVED'
        verdicts.append((name, got))
        if got != exp:
            wrong.append('{}: expected {}, got {}'.format(name, exp, got))
        print('{:<48} {:<9} {:<8} {}'.format(
            name, got, '' if got == exp else 'WRONG', why))

    n_k = sum(1 for _n, g in verdicts if g == 'KILLED')
    n_s = len(verdicts) - n_k
    n_exp_s = sum(1 for r in rows if r[5] == 'SURVIVED')
    print('\n{} rows: {} killed, {} survived ({} of them expected), '
          '{} broken, {} disagreeing with expectation'.format(
              len(verdicts), n_k, n_s, n_exp_s, len(broken), len(wrong)))
    for w in wrong + broken:
        print('  {}'.format(w))
    return 1 if (wrong or broken) else 0


if __name__ == '__main__':
    sys.exit(main())
