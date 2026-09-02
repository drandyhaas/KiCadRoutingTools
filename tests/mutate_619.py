#!/usr/bin/env python3
"""#619 mutation battery: does the erased-copper gate's test suite actually
bite, or does it merely run?

    python3 tests/mutate_619.py
    python3 tests/mutate_619.py --row over-strict-floor-x5
    python3 tests/mutate_619.py --list

NOT named `test_*`, so `run_all.py` never collects it: it REWRITES engine files
in place and restores them, and a suite running beside it would grade a mutated
tree. One writer per tree.

A row is KILLED when any named test exits non-zero -- a failed assertion and an
ERROR count the same, because a mutation that makes the graders crash is still
a mutation the graders noticed. A row whose anchor does not match EXACTLY ONCE
is BROKEN, not skipped: an anchor that silently matches nothing reports every
mutation as killed and is the most flattering possible bug.

Expected SURVIVORS are declared with the reason they are not a test hole.

BYTECODE. Every row deletes the target's `__pycache__` before running. CPython
invalidates a `.pyc` on (mtime, size), and several of these mutations are
size-preserving edits written within the same second as the original -- the
child process would then import the CACHED original and the row would report
SURVIVED for a mutation that was never actually loaded. That is a silent
false-negative in the flattering direction, so it is prevented rather than
hoped against.
"""
import argparse
import os
import shutil
import subprocess
import sys

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)

QFN = os.path.join(ROOT, 'py_router', 'qfn_fanout', '__init__.py')
TARGETS = {'q': QFN}

T_619 = os.path.join(TESTS, 'test_619_underpad_stub_vs_erased_copper.py')
T_UP = os.path.join(TESTS, 'test_qfn_underpad.py')

#: (name, target, old, new, tests, expectation)
ROWS = [
    # ---- the floor itself ------------------------------------------------
    # The one mutation the NEGATIVE CONTROL exists for. Everything else in the
    # file passes under it: an over-strict gate still drives every violation
    # count to zero, and only a control that asserts a NON-blocker stays
    # non-blocking can tell "correct" from "refuses everything".
    ('over-strict-floor-x5', 'q',
     "                        < v.size / 2 + track_width / 2 + _stub_clr - 1e-6:",
     "                        < 5.0 * (v.size / 2 + track_width / 2 + _stub_clr):",
     (T_619,), 'KILLED'),
    ('via-floor-uses-drill-not-size', 'q',
     "                if point_to_segment_distance(v.x, v.y, px, py, vx, vy) \\\n"
     "                        < v.size / 2 + track_width / 2 + _stub_clr - 1e-6:",
     "                if point_to_segment_distance(v.x, v.y, px, py, vx, vy) \\\n"
     "                        < v.drill / 2 + track_width / 2 + _stub_clr - 1e-6:",
     (T_619,), 'KILLED'),

    # ---- the guards ------------------------------------------------------
    ('drop-the-zero-stub-guard', 'q',
     "        if math.hypot(vx - px, vy - py) <= POSITION_TOLERANCE:",
     "        if False:",
     (T_619, T_UP), 'KILLED'),
    ('via-half-drops-the-own-net-skip', 'q',
     "                if v.net_id == net_id:\n"
     "                    continue                # own-net copper is no obstacle",
     "                if False:\n"
     "                    continue                # own-net copper is no obstacle",
     (T_619, T_UP), 'KILLED'),
    ('seg-half-drops-the-own-net-skip', 'q',
     "                if s.net_id == net_id or s.layer != footprint.layer:",
     "                if s.layer != footprint.layer:",
     (T_619,), 'KILLED'),
    ('pad-half-drops-the-own-net-skip', 'q',
     "                if p.net_id == net_id or id(p) in _tie:",
     "                if id(p) in _tie:",
     (T_619,), 'KILLED'),

    # ---- the layer decision ---------------------------------------------
    # The surface fan's spelling, which is correct THERE and wrong here: the
    # under-pad stub is emitted on footprint.layer (#195), not on `layer`.
    ('seg-half-filters-the-ESCAPE-layer', 'q',
     "                if s.net_id == net_id or s.layer != footprint.layer:",
     "                if s.net_id == net_id or s.layer != layer:",
     (T_619,), 'KILLED'),

    # ---- the halves are separable ----------------------------------------
    ('via-half-disabled', 'q',
     "        if _gate_via:", "        if False:", (T_619,), 'KILLED'),
    ('seg-half-disabled', 'q',
     "        if _gate_seg:", "        if False:", (T_619,), 'KILLED'),
    ('pad-half-disabled', 'q',
     "        if _gate_pad:", "        if False:", (T_619,), 'KILLED'),

    # ---- the knob's failure direction ------------------------------------
    # An unrecognised value must mean ALL, never OFF: a typo in a harness must
    # not silently ship the bug back.
    ('unknown-knob-value-means-OFF', 'q',
     "    _gate_all = not _gsel or 'all' in _gsel or not (_gsel & {'via', 'seg', 'pad'})",
     "    _gate_all = 'all' in _gsel",
     (T_619,), 'KILLED'),

    # ---- the pad half's exclusions ---------------------------------------
    ('pad-half-ignores-local-clearance', 'q',
     "                                 margin=max(_stub_clr,\n"
     "                                            getattr(p, 'local_clearance', 0.0)\n"
     "                                            or 0.0) + track_width / 2 - 1e-6):",
     "                                 margin=_stub_clr + track_width / 2 - 1e-6):",
     (T_619,), 'KILLED'),
    # NPTH and the wildcard are asserted against check_drc's own helpers rather
    # than through emitted geometry, because no tracked board places a netted
    # NPTH or an F&B.Cu pad where a stub can reach it. Removing the filter
    # therefore changes no output on this corpus -- a REAL test hole, named
    # rather than hidden, and the reason those two checks call the helpers
    # directly instead of pretending a board covers them.
    ('pad-half-includes-NPTH', 'q',
     "                    and not _pad_has_no_copper(p)",
     "                    and True",
     (T_619,), 'SURVIVED  (no tracked board has a netted NPTH within stub '
               'reach; check 8 asserts the helper, not emitted geometry)'),

    # ---- the dru clearance ----------------------------------------------
    # #770: the .kicad_dru layer-rule subset #498 models does not intersect any
    # board this repo ingests, so _stub_clr == clearance on every fixture. The
    # distinction is real (the stub lands on footprint.layer while the caller
    # resolved the rule for the escape layer) but unfalsifiable here.
    ('stub-clearance-ignores-the-dru-layer-map', 'q',
     "    _stub_clr = cfg.layer_clearance(footprint.layer, clearance)",
     "    _stub_clr = clearance",
     (T_619, T_UP), 'SURVIVED  (#770: no tracked board carries a .kicad_dru '
                    'layer rule, so the two values are equal on every fixture)'),
]


def run(tests):
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1')
    for t in tests:
        r = subprocess.run([sys.executable, '-X', 'utf8', t],
                           cwd=ROOT, capture_output=True, text=True, env=env)
        if r.returncode != 0:
            return True, os.path.basename(t)
    return False, ''


def _drop_pycache(path):
    d = os.path.join(os.path.dirname(path), '__pycache__')
    shutil.rmtree(d, ignore_errors=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--row')
    ap.add_argument('--list', action='store_true')
    a = ap.parse_args()
    if a.list:
        for n, t, _o, _w, tests, exp in ROWS:
            print(f'  {n:44s} {os.path.basename(TARGETS[t]):14s} {exp}')
        return 0

    dirty = subprocess.run(['git', 'diff', '--quiet', '--'] + list(TARGETS.values()),
                           cwd=ROOT).returncode
    if dirty:
        print('REFUSED: the files this battery rewrites have uncommitted '
              'changes.\nRestoring them would write the COMMITTED text back '
              'over your work. Commit first.')
        return 2

    rows = [r for r in ROWS if not a.row or r[0] == a.row]
    if not rows:
        print(f'no row named {a.row!r}')
        return 2
    originals = {k: open(v, encoding='utf-8').read() for k, v in TARGETS.items()}
    killed = survived = broken = disagree = 0
    try:
        for name, tgt, old, new, tests, exp in rows:
            src = originals[tgt]
            if src.count(old) != 1:
                print(f'  {name:44s} BROKEN (anchor matched {src.count(old)}x)')
                broken += 1
                continue
            with open(TARGETS[tgt], 'w', encoding='utf-8', newline='') as fh:
                fh.write(src.replace(old, new))
            _drop_pycache(TARGETS[tgt])
            try:
                died, by = run(tests)
            finally:
                with open(TARGETS[tgt], 'w', encoding='utf-8', newline='') as fh:
                    fh.write(src)
                _drop_pycache(TARGETS[tgt])
            got = 'KILLED' if died else 'SURVIVED'
            want = exp.split()[0]
            mark = '' if got == want else '   *** DISAGREES with ' + exp
            if got != want:
                disagree += 1
            killed += died
            survived += not died
            print(f'  {name:44s} {got:9s} {by}{mark}')
    finally:
        for k, v in TARGETS.items():
            with open(v, 'w', encoding='utf-8', newline='') as fh:
                fh.write(originals[k])
            _drop_pycache(v)
    print(f'\n{len(rows)} row(s): {killed} killed, {survived} survived, '
          f'{broken} broken, {disagree} disagreeing with expectation')
    return 1 if (broken or disagree) else 0


if __name__ == '__main__':
    sys.exit(main())
