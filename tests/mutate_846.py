#!/usr/bin/env python3
"""Mutation battery for #846 / #854 / #652 -- do the new tests actually bite?

Each row rewrites ONE line of engine source into a plausible wrong version and
runs the tests that should notice. A row that SURVIVES is a test that does not
test what its name says; a row reported BROKEN is an anchor that no longer
matches, which is the most flattering possible bug (an anchor matching nothing
scores every mutation as killed) and is therefore checked before mutating.

Deliberately NOT named test_*: run_all.py must not collect it. It rewrites
engine files in place and restores them in a `finally`, so it refuses to start
on a dirty tree -- commit first.

    python3 tests/mutate_846.py            # every row
    python3 tests/mutate_846.py --row NAME

The measured table lives in the header of the test file each row defends,
written from the run and never edited to match a prediction.
"""
import argparse
import io
import os
import subprocess
import sys

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)

QFN = os.path.join(_ROOT, 'py_router', 'qfn_fanout', '__init__.py')
FAB = os.path.join(_ROOT, 'py_router', 'fab_notes.py')
COM = os.path.join(_ROOT, 'py_router', 'routing_common.py')
DIA = os.path.join(_ROOT, 'py_router', 'routing_diagnostics.py')
RTE = os.path.join(_ROOT, 'py_router', 'route.py')
TARGETS = {'qfn': QFN, 'fab': FAB, 'com': COM, 'dia': DIA, 'rte': RTE}

T846 = os.path.join(_TESTS, 'test_846_via_in_pad_classification.py')
TLAD = os.path.join(_TESTS, 'test_846_onpad_ladder_reach.py')
T652 = os.path.join(_TESTS, 'test_652_fanout_dropped_ball_hint.py')
TCLAMP = os.path.join(_TESTS, 'test_fanout_via_in_pad_clamp.py')

_CLASSIFY = '            if via_overlaps_pad(pi.pad, vx, vy, via_size):'

# (name, target, old, new, tests, expect)
ROWS = [
    # --- #846: the classification -----------------------------------------
    ('classification-reverted-to-the-centre-test', 'qfn',
     _CLASSIFY,
     '            if math.hypot(vx - px, vy - py) <= POSITION_TOLERANCE:',
     (T846, TCLAMP), 'KILLED'),

    ('classification-always-in-pad', 'qfn',
     _CLASSIFY, '            if True:',
     (T846,), 'KILLED'),

    ('classification-always-off-pad', 'qfn',
     _CLASSIFY, '            if False:',
     (T846, TCLAMP), 'KILLED'),

    # The scoping half: `via_in_pad_sites` breaks on the FIRST same-net pad it
    # overlaps, which need not be the pad this leg escapes -- and the clamp is
    # applied to `pi.pad`. Classifying by a neighbour while clamping to this
    # one is the shape the fix had to avoid, and why the commit loop passes
    # `pi.pad` rather than calling `via_in_pad_sites`.
    #
    # SURVIVED, and kept for the reason rather than deleted. On every board in
    # the set the two spellings agree: the nets being escaped have ONE pad on
    # the fanned footprint, and where they do not (tigard's GND, which owns the
    # 4.35mm exposed pad as well as leads) the via overlaps both, so the
    # BOOLEAN is the same and the clamp target is `pi.pad` either way. Killing
    # it needs a via overlapping a same-net NEIGHBOUR but not its own pad,
    # which no in-repo board produces. This row is a change detector for the
    # day one does -- deleting it is how that becomes folklore.
    ('classification-scoped-to-ANY-same-net-pad', 'qfn',
     _CLASSIFY,
     '            if any(via_overlaps_pad(_q, vx, vy, via_size)\n'
     '                   for _q in pcb_data.pads_by_net.get(pi.pad.net_id, ())):',
     (T846,), 'SURVIVED'),

    # A disclosure count, not a behaviour: the emitted copper is identical
    # either way, so no test that grades the BOARD can see it. Kept because a
    # number a reader is told to watch move is worth a row saying which kind of
    # thing it is.
    ('offcentre-count-never-incremented', 'qfn',
     '                if math.hypot(vx - px, vy - py) > POSITION_TOLERANCE:\n'
     '                    offcentre_n += 1',
     '                if False:\n'
     '                    offcentre_n += 1',
     (T846,), 'SURVIVED'),

    # --- #846: fab_notes owns ONE predicate --------------------------------
    ('fab-note-stops-calling-the-shared-predicate', 'fab',
     '            if via_overlaps_pad(pad, vx, vy, vsz, margin):',
     '            if abs(vx - _get(pad, \'global_x\', 0.0)) <= 1e-3 \\\n'
     '                    and abs(vy - _get(pad, \'global_y\', 0.0)) <= 1e-3:',
     (T846,), 'KILLED'),

    ('overlap-predicate-drops-the-barrel-radius', 'fab',
     '    vr = (via_size if via_size else 0.6) / 2.0\n'
     '    return _pad_holds(pad, via_x, via_y, max(vr - 1e-6, margin))',
     '    return _pad_holds(pad, via_x, via_y, margin)',
     (T846,), 'KILLED'),

    # --- #846: the ladder reach knob ---------------------------------------
    ('ladder-reach-filter-deleted', 'qfn',
     '    if reach is not None:\n'
     '        seq = [d for d in seq if abs(d) <= reach + 1e-9]',
     '    if False:\n'
     '        seq = [d for d in seq if abs(d) <= reach + 1e-9]',
     (TLAD,), 'KILLED'),

    ('unrecognised-knob-value-shortens-the-ladder', 'qfn',
     "    reach = {'pad': pad_width / 2.0,\n"
     "             'barrel': pad_width / 2.0 - via_size / 2.0,\n"
     "             }.get(env_knobs.QFN_ONPAD_REACH)",
     "    reach = {'full': None,\n"
     "             'barrel': pad_width / 2.0 - via_size / 2.0,\n"
     "             }.get(env_knobs.QFN_ONPAD_REACH, pad_width / 2.0)",
     (TLAD,), 'KILLED'),

    ('pad-and-barrel-arms-collapsed-into-one', 'qfn',
     "             'barrel': pad_width / 2.0 - via_size / 2.0,",
     "             'barrel': pad_width / 2.0,",
     (TLAD,), 'KILLED'),

    # The engine must have ONE source for these offsets. A second copy inside
    # candidate_offsets is how the first draft of the ladder test came to
    # mirror the engine instead of calling it, and every knob row survived.
    ('candidate_offsets-grows-its-own-copy-of-the-ladder', 'qfn',
     "        axis = axis_offset_ladder(pad_width, via_size, step,\n"
     "                                  'near' if mode == 'out' else mode)",
     "        axis = [0.0]\n"
     "        for _k in range(1, 9):\n"
     "            axis += [_k * step, -_k * step]",
     (TLAD,), 'KILLED'),

    # --- #652: the predicate ------------------------------------------------
    ('bare-test-ignores-attached-copper', 'com',
     '            if any(abs(x - pad.global_x) < reach and abs(y - pad.global_y) < reach\n'
     '                   for (x, y) in attached.get(pad.net_id, ())):\n'
     '                continue      # this pass or an earlier one attached copper',
     '            if False:\n'
     '                continue      # this pass or an earlier one attached copper',
     (T652,), 'KILLED'),

    ('bare-test-ignores-POURS', 'com',
     '            if any(point_in_polygon(pad.global_x, pad.global_y, poly)\n'
     '                   for poly in zones_by_net.get(pad.net_id, ())):\n'
     '                continue      # served by a pour, not bare',
     '            if False:\n'
     '                continue      # served by a pour, not bare',
     (T652,), 'KILLED'),

    ('bare-test-reports-the-OUTER-RING-too', 'com',
     '            if min(pad.global_x - x0, x1 - pad.global_x,\n'
     '                   pad.global_y - y0, y1 - pad.global_y) <= band:\n'
     '                continue      # outer ring: reachable through the band',
     '            if False:\n'
     '                continue      # outer ring: reachable through the band',
     (T652,), 'KILLED'),

    ('bare-test-reports-SINGLE-PAD-nets', 'com',
     '    want = {n for n in want\n'
     '            if n and len(pcb_data.pads_by_net.get(n, [])) >= min_net_pads}',
     '    want = {n for n in want if n}',
     (T652,), 'KILLED'),

    ('bare-test-reports-DRILLED-pads', 'com',
     '            if pad.net_id not in want or (pad.drill or 0) > 0:',
     '            if pad.net_id not in want:',
     (T652,), 'KILLED'),

    # --- #652: the hint and its channel -------------------------------------
    ('hint-never-fires', 'dia',
     '    if not bare:\n        return _ret(\'\')',
     '    if True:\n        return _ret(\'\')',
     (T652,), 'KILLED'),

    ('hint-fires-for-every-net', 'dia',
     '    if pcb_data is None or not net_id:\n        return _ret(\'\')',
     '    if pcb_data is None or not net_id:\n        return _ret(\'\')\n'
     '    bare = [(p, "U1") for p in pcb_data.pads_by_net.get(net_id, ())]\n'
     '    if bare:\n'
     '        return _ret("Hint: pad U1.x is a fanout-dropped ball (no escape '
     'stub) -- re-run the fanout (--escape-method underpad).",\n'
     '                    {"verdict": "fanout_dropped", "pad": "U1.x",\n'
     '                     "component": "U1", "pads": []})',
     (T652,), 'KILLED'),

    ('summary-key-never-set', 'rte',
     "        if fanout_dropped_report:\n"
     "            summary['fanout_dropped'] = fanout_dropped_report",
     "        if False:\n"
     "            summary['fanout_dropped'] = fanout_dropped_report",
     (T652,), 'KILLED'),

    ('results_data-mirror-dropped', 'rte',
     "            'fanout_dropped': fanout_dropped_report,",
     "            'fanout_dropped': [],",
     (T652,), 'KILLED'),

    # --- the battery's own control -----------------------------------------
    # A semantically inert edit. If this is ever KILLED the rig is reporting
    # noise, and every other row's verdict is suspect.
    ('control-semantically-inert-edit', 'com',
     '    out = []\n    for fp in pcb_data.footprints.values():',
     '    out = list()\n    for fp in pcb_data.footprints.values():',
     (T652,), 'SURVIVED'),
]


def _dirty(path):
    p = subprocess.run(['git', 'status', '--porcelain', '--', path],
                       capture_output=True, text=True, cwd=_ROOT)
    return bool(p.stdout.strip())


def run(only=None):
    rows = [r for r in ROWS if only is None or r[0] == only]
    if not rows:
        print('no row named %r' % only)
        return 1
    for path in TARGETS.values():
        if _dirty(path):
            print('REFUSING: %s has uncommitted changes. Commit or stash '
                  'first -- this battery restores by overwriting.'
                  % os.path.basename(path))
            return 2

    orig = {k: io.open(v, encoding='utf-8', newline='').read()
            for k, v in TARGETS.items()}
    results = []
    try:
        for name, tgt, old, new, tests, expect in rows:
            path = TARGETS[tgt]
            base = orig[tgt]
            edits = old if isinstance(old, list) else [(old, new)]
            if '\r\n' in base:
                edits = [(o.replace('\n', '\r\n'), n.replace('\n', '\r\n'))
                         for o, n in edits]
            counts = [base.count(o) for o, _n in edits]
            if counts != [1] * len(edits):
                results.append((name, 'BROKEN', expect,
                                'anchors matched %s times' % counts, []))
                continue
            mutated = base
            for o, nw in edits:
                mutated = mutated.replace(o, nw, 1)
            io.open(path, 'w', encoding='utf-8', newline='').write(mutated)
            killed, failed, crashed = False, [], False
            for t in tests:
                p = subprocess.run([sys.executable, '-X', 'utf8', t],
                                   capture_output=True, text=True,
                                   encoding='utf-8', errors='replace',
                                   timeout=1800, cwd=_ROOT)
                out = (p.stderr or '') + (p.stdout or '')
                # A mutant that makes the ENGINE crash proves nothing about
                # the test's discrimination -- a non-zero exit scores as a kill
                # either way. Three rows of mutate_620 spent a commit in
                # exactly that state (`NameError: ahx` after this PR deleted
                # the name their REPLACEMENT text used), reporting KILLED while
                # measuring nothing. The BROKEN-anchor guard cannot see it: it
                # validates the string being replaced, not the replacement.
                if 'NameError' in out or 'AttributeError' in out:
                    crashed = True
                if p.returncode:
                    killed = True
                failed += ['%s::%s' % (os.path.basename(t)[5:8],
                                       l.split('--')[0].replace('FAIL: ', '')
                                       .replace('ERROR: ', '').strip()[:70])
                           for l in out.splitlines()
                           if l.strip().startswith(('FAIL:', 'ERROR:'))]
            io.open(path, 'w', encoding='utf-8', newline='').write(base)
            results.append((name,
                            'CRASH' if crashed else
                            ('KILLED' if killed else 'SURVIVED'),
                            expect, '%d' % len(failed), failed))
    finally:
        for k, v in TARGETS.items():
            io.open(v, 'w', encoding='utf-8', newline='').write(orig[k])

    w = max(len(r[0]) for r in results)
    wrong = 0
    for name, verdict, expect, cnt, failed in results:
        mark = ''
        if verdict != expect:
            mark = '   <-- WRONG, expected %s' % expect
            wrong += 1
        print('%-*s  %-9s  %-3s%s' % (w, name, verdict, cnt, mark))
        for f in failed[:4]:
            print('%s      %s' % (' ' * w, f))
    killed = sum(1 for r in results if r[1] == 'KILLED')
    survived = sum(1 for r in results if r[1] == 'SURVIVED')
    broken = sum(1 for r in results if r[1] in ('BROKEN', 'CRASH'))
    print('\n%d rows: %d killed, %d survived (%d of them expected), %d broken'
          % (len(results), killed, survived,
             sum(1 for r in results if r[1] == r[2] == 'SURVIVED'), broken))
    if wrong or broken:
        print('%d row(s) did not match their expectation' % (wrong + broken))
    return 1 if (wrong or broken) else 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--row', help='run a single row by name')
    ap.add_argument('--list', action='store_true', help='print row names')
    a = ap.parse_args()
    if a.list:
        for r in ROWS:
            print('%-46s %-4s %s' % (r[0], r[1], r[5]))
        return 0
    return run(a.row)


if __name__ == '__main__':
    sys.exit(main())
