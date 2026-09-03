#!/usr/bin/env python3
"""Mutation battery for #850 (the shared face rule) and #848 (per-side rects).

    python tests/mutate_850_848.py --verify-anchors     # ALWAYS run this first
    python tests/mutate_850_848.py --selftest
    python tests/mutate_850_848.py --list
    python tests/mutate_850_848.py [--row NAME]

Not named `test_*`, because it REWRITES ENGINE FILES IN PLACE. Run it in a
worktree of its own -- one writer per tree, or it and a concurrent suite
invalidate each other silently.

A new file rather than rows appended to `tests/mutate_834_835.py`, which is
where #841's rows went: these need different witnesses (`test_850_*`,
`test_848_*`) and therefore a different unmutated gate, and a gate that
mixes them would report a #835 fixture's absence as a #850 survivor.

The contract, copied from `tests/mutate_847.py` deliberately rather than
paraphrased: refuse a dirty tree; verify every anchor matches EXACTLY ONCE
before running (a stale anchor is BROKEN, never a silent skip); require the
gate to pass UNMUTATED first; drop `__pycache__` before and after every row and
run children with `-B`; treat exit 77 from every witness as UNDECIDED rather
than a kill; restore in a `finally`.

RECORDED at bcb722d3 -- 19 rows, 18 killed, 1 survived, 0 undecided, 0 broken,
1 disagreeing with expectation. Re-run after the hole below was closed: 19
rows, 19 killed, 0 disagreeing.

THE ONE DISAGREEMENT WAS A HOLE IN MY OWN TESTS, not a wrong expectation, and
it is recorded here because a battery that only ever reports a clean sweep is
not evidence that it can find anything.

`the-face-geometry-moves-to-the-copper-box` builds `faces` from
`ctx.geom[ref].copper` instead of `pp.extent` -- so the face LENGTH, the
escape band and the obstruction span all move to a smaller box. It SURVIVED,
against two witnesses that each should have caught it and each could not:

  * `test_850`'s isolation arm toggles the face RULE and diffs the static row
    keys between the two arms. The mutation moves `length_mm` in BOTH arms
    equally, so the diff stays empty. That is #849's own warning about
    equivalence checks -- both sides move together -- landing in my arm.
  * `test_849`'s `GOLDEN_PRE_HOIST` pins `supply_*` and `eaten_by` against
    b5c567c7 on rp2350 U2 and tigard U3. Neither carries an NPTH hole, so
    `rect == copper` on both and the mutation is a value no-op on exactly the
    two refs that are pinned.

Closed by `test_850`'s `the_face_geometry_is_still_the_extent`, which asserts
`length_mm` against the extent-derived face on the FIVE refs corpus-wide where
the two boxes actually differ (rp2350 J2, watchy SW1-SW4), and asserts the
precondition `rect != copper` first so it cannot pass while measuring nothing.
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
LEG = os.path.join(ROOT, 'py_placer', 'placement', 'legality.py')
CHK = os.path.join(ROOT, 'py_tools', 'check_channels.py')
TARGETS = {'esc': ESC, 'rou': ROU, 'leg': LEG, 'chk': CHK}

T850 = os.path.join(ROOT, 'tests', 'test_850_demand_face_of.py')
T848 = os.path.join(ROOT, 'tests', 'test_848_side_obstruction.py')
T849 = os.path.join(ROOT, 'tests', 'test_849_lane_context.py')
T835 = os.path.join(ROOT, 'tests', 'test_835_escape_side_aware.py')
T841 = os.path.join(ROOT, 'tests', 'test_841_obstruction_rect.py')

#: (name, target, old, new, tests, expected)
ROWS = [
    # ---- #850: the shared face rule ---------------------------------------
    ('the-ledger-goes-back-to-pad-centres', 'rou',
     "    own = ctx.geom.get(ref)\n"
     "    asg = _assign_faces(fp, own, lane_mm=pitch_routed, fallback_rect=ext)",
     "    own = ctx.geom.get(ref)\n"
     "    asg = _assign_faces(fp, None, lane_mm=pitch_routed,"
     " fallback_rect=ext)",
     (T850,), 'KILLED'),

    ('face_of-gets-the-rect-not-the-copper', 'esc',
     "    return (min(b[0] for b in netted), min(b[1] for b in netted),\n"
     "            max(b[2] for b in netted), max(b[3] for b in netted))",
     "    return geom.rect if geom is not None else (\n"
     "        min(b[0] for b in netted), min(b[1] for b in netted),\n"
     "        max(b[2] for b in netted), max(b[3] for b in netted))",
     (T850,), 'KILLED'),

    ('the-assignment-box-goes-back-to-every-pad', 'esc',
     "    netted = [b for p, b in boxes\n"
     "              if b is not None and getattr(p, 'net_id', 0)]",
     "    netted = [b for p, b in boxes if b is not None]",
     (T850, T835), 'KILLED'),

    ('the-assignment-box-uses-pad-centres', 'esc',
     "    return (min(b[0] for b in netted), min(b[1] for b in netted),\n"
     "            max(b[2] for b in netted), max(b[3] for b in netted))\n",
     "    _c = [((b[0] + b[2]) / 2.0, (b[1] + b[3]) / 2.0) for b in netted]\n"
     "    return (min(c[0] for c in _c), min(c[1] for c in _c),\n"
     "            max(c[0] for c in _c), max(c[1] for c in _c))\n",
     (T850,), 'KILLED'),

    ('pad_box-is-dropped-so-it-degrades-to-centres', 'esc',
     "    boxes = [(p, None if geom is None else _pad_box(geom, p))"
     " for p in pads]",
     "    boxes = [(p, None) for p in pads]",
     (T850, T841), 'KILLED'),

    ('the-tolerance-becomes-the-lane', 'esc',
     "    pitch = pad_pitch(fp)\n"
     "    source = 'pad_lattice'\n"
     "    if pitch == float('inf'):\n"
     "        pitch, source = lane_mm, 'lane_fallback'",
     "    pitch, source = lane_mm, 'lane_fallback'",
     (T850,), 'KILLED'),

    ('face-pitch-reports-the-lane', 'rou',
     "                    'face_pitch_mm': round(asg.pitch_mm, 4),",
     "                    'face_pitch_mm': round(pitch_routed, 4),",
     (T850,), 'KILLED'),

    ('the-interior-bucket-swallows-a-face-pad', 'rou',
     "        if face is None:\n"
     "            interior_pads += 1\n"
     "            interior_nets.add(nid)",
     "        if face is None or face == 'north':\n"
     "            interior_pads += 1\n"
     "            interior_nets.add(nid)",
     (T850,), 'KILLED'),

    ('interior-pads-published-as-zero', 'rou',
     "                    'interior_pads': interior_pads,",
     "                    'interior_pads': 0,",
     (T850,), 'KILLED'),

    ('interior-nets-and-demand-nets-swapped', 'rou',
     "                    'interior_nets': len(interior_nets),\n"
     "                    'interior_demand_nets': len(interior_demand),",
     "                    'interior_nets': len(interior_demand),\n"
     "                    'interior_demand_nets': len(interior_nets),",
     (T850,), 'KILLED'),

    ('interior-demand-does-not-subtract-the-faces', 'rou',
     "    interior_demand -= set().union(*demand.values())"
     " if demand else set()",
     "    interior_demand -= set()",
     (T850,), 'KILLED'),

    ('interior-pads-counted-after-the-owner-filter', 'rou',
     "        if face is None:\n"
     "            interior_pads += 1\n"
     "            interior_nets.add(nid)\n"
     "        owners = net_owners.get(nid, set())",
     "        owners = net_owners.get(nid, set())\n"
     "        if face is None and 2 <= len(owners) <= DISPLACEMENT_MAX_FANOUT:\n"
     "            interior_pads += 1\n"
     "            interior_nets.add(nid)",
     (T850,), 'KILLED'),

    ('the-face-map-swaps-N-and-S', 'esc',
     "FACE_LETTER = {'north': 'N', 'east': 'E', 'south': 'S', 'west': 'W'}",
     "FACE_LETTER = {'north': 'S', 'east': 'E', 'south': 'N', 'west': 'W'}",
     (T850,), 'KILLED'),

    ('the-face-map-swaps-E-and-W', 'esc',
     "FACE_LETTER = {'north': 'N', 'east': 'E', 'south': 'S', 'west': 'W'}",
     "FACE_LETTER = {'north': 'N', 'east': 'W', 'south': 'S', 'west': 'E'}",
     (T850,), 'KILLED'),

    ('the-face-geometry-moves-to-the-copper-box', 'rou',
     "    faces = {'N': (ext[0], ext[1], ext[2], ext[1]),",
     "    ext = (ctx.geom[ref].copper if ref in ctx.geom else ext)\n"
     "    faces = {'N': (ext[0], ext[1], ext[2], ext[1]),",
     (T850, T849), 'KILLED'),

    # ---- #848: the per-side neighbour rect --------------------------------
    ('the-per-side-union-collapses-to-the-whole-part', 'leg',
     "    boxes = [geom.rect_sides[s] for s in sides if s in geom.rect_sides]\n"
     "    if not boxes:\n"
     "        return geom.rect",
     "    boxes = []\n"
     "    if not boxes:\n"
     "        return geom.rect",
     (T848,), 'KILLED'),

    ('the-union-is-over-the-neighbours-own-sides', 'rou',
     "        shared = own_sides & g.sides\n"
     "        if not shared:\n"
     "            continue\n"
     "        neighbors.append((g.ref, rect_on_sides(_geom[g.ref], shared)))",
     "        shared = own_sides & g.sides\n"
     "        if not shared:\n"
     "            continue\n"
     "        neighbors.append((g.ref, rect_on_sides(_geom[g.ref], g.sides)))",
     (T848,), 'KILLED'),

    ('escape-keeps-the-whole-part-rect', 'esc',
     "        obstacles.append((other, _rect_on_sides(g, shared)\n"
     "                          if g is not None else _part_rect(ofp)))",
     "        obstacles.append((other, g.rect\n"
     "                          if g is not None else _part_rect(ofp)))",
     (T848,), 'KILLED'),

    ('the-per-side-box-drops-the-holes', 'leg',
     "            rad = math.radians(-key[0])\n"
     "            hc, hs = math.cos(rad), math.sin(rad)\n"
     "            for ox, oy, r in self.holes_extent:\n"
     "                cx, cy = ox * hc - oy * hs, ox * hs + oy * hc\n"
     "                xs0.append(cx - r); ys0.append(cy - r)\n"
     "                xs1.append(cx + r); ys1.append(cy + r)\n"
     "            ext = () if not xs0 else",
     "            ext = () if not xs0 else",
     (T848,), 'KILLED'),
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
    probe = os.path.join(ROOT, 'tests', '_mutate_850_probe.py')
    try:
        with open(probe, 'w', encoding='utf-8') as fh:
            fh.write('VALUE = 1\n')
        sys.path.insert(0, os.path.join(ROOT, 'tests'))
        import _mutate_850_probe as m
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
    got0, why0 = _run((T850, T848))
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
