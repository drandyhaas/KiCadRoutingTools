#!/usr/bin/env python3
"""#959 / #998: `place_pose --intent` grades a pose against the zone plan
BEFORE it is written.

Run 29, lap 10: the model moved `Ref*` with `place_pose set`, out of the
`fiducial-nw` zone its own plan declared, and learned it only from the next
grade. With `--intent`, the write is refused at exit 4 with nothing written,
and `JSON_SUMMARY.zone_check` names the block, the zone and the overrun.

The check is RELATIVE, like place_pose's legality verdict: on run 29's pile
`Ref*` starts 24 mm east of its zone, so a move that brings it closer but not
all the way home must pass. And it grades with the grade's own rule, so a
zone the author demoted to WARN is reported and never refused.
"""
import hashlib
import json
import os
import shutil
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _sub in ('', 'py_router', 'py_tools', 'py_placer'):
    _p = os.path.join(REPO, _sub) if _sub else REPO
    if _p not in sys.path:
        sys.path.insert(0, _p)

import run_utils                                            # noqa: E402
from kicad_parser import parse_kicad_pcb                    # noqa: E402

FIX = os.path.join(REPO, 'tests', 'fixtures', '959')
PILE = os.path.join(FIX, 'run29_pile.kicad_pcb')
LAP5 = os.path.join(FIX, 'zone_plan_r2_lap5.json')
TOOL = run_utils.tool('place_pose.py')
#: `fiducial-nw` in the lap-5 plan, which names `Ref[*]` -- `Ref*` alone.
ZONE = [114.3, 91.3, 116.9, 93.1]
#: Further from that zone than the pile's 24.8 mm, and ON the board (it spans
#: 114-145.75 x 91-105.5) with no new pad conflict -- so the zone is the only
#: thing a refusal there can be about.
FAR = ('143.5', '103.5')


def _stage(tmp):
    """The pile and its project, copied: place_pose writes beside its input
    when told to, and a fixture must never be the output."""
    board = os.path.join(tmp, 'pile.kicad_pcb')
    shutil.copyfile(PILE, board)
    shutil.copyfile(os.path.splitext(PILE)[0] + '.kicad_pro',
                    os.path.splitext(board)[0] + '.kicad_pro')
    return board


def _summary(r):
    lines = [x for x in r.stdout.splitlines() if x.startswith('JSON_SUMMARY:')]
    assert len(lines) == 1, r.stdout[-1500:]
    return json.loads(lines[0].split('JSON_SUMMARY: ', 1)[1])


def _sha(path):
    with open(path, 'rb') as fh:
        return hashlib.sha256(fh.read()).hexdigest()


def _pose(path, ref):
    fp_ = parse_kicad_pcb(path).footprints[ref]
    return round(fp_.x, 3), round(fp_.y, 3)


def test_run29_lap10_write_is_refused():
    run_utils.evidence(PILE)
    run_utils.evidence(LAP5)
    with tempfile.TemporaryDirectory() as tmp:
        board = _stage(tmp)
        out = os.path.join(tmp, 'out.kicad_pcb')
        before = _sha(board)
        r = run_utils.check([sys.executable, '-X', 'utf8', TOOL, board, out,
                             'set', 'Ref*', *FAR,
                             '--intent', LAP5],
                            refuse="the intent's zone_containment is an "
                                   "ERROR", code=4)
        s = _summary(r)
        assert s['output'] is None and not os.path.exists(out), s
        assert _sha(board) == before
        # Legality did NOT worsen, so the zone is the whole reason.
        assert s['no_worse'] is True, s['no_worse']
        zc = s['zone_check']
        assert [w['block'] for w in zc['worse']] == ['fiducial-nw'], zc
        w = zc['worse'][0]
        assert w['ref'] == 'Ref*' and w['zone'] == ZONE, w
        assert w['outside_mm_after'] > w['outside_mm_before'] > 20, w
        assert 'Ref*~2' not in zc['refs'], zc
    print("  PASS: moving Ref* further from fiducial-nw is refused at exit 4, "
          "nothing written; the summary names block, zone and overrun")


def test_force_writes_and_says_so():
    with tempfile.TemporaryDirectory() as tmp:
        board = _stage(tmp)
        out = os.path.join(tmp, 'out.kicad_pcb')
        r = run_utils.check([sys.executable, '-X', 'utf8', TOOL, board, out,
                             'set', 'Ref*', *FAR,
                             '--intent', LAP5, '--force'], accept=True)
        s = _summary(r)
        assert s['forced'] is True and s['zone_check']['worse'], s
        assert 'WRITTEN under --force' in r.stdout, r.stdout[-800:]
        assert _pose(out, 'Ref*') == tuple(map(float, FAR))
    print("  PASS: --force writes the pose and records the finding")


def test_a_move_toward_the_zone_passes():
    """Relative, not absolute: from 24 mm out, 12 mm out is not refused for
    not arriving -- and landing inside grades clean."""
    with tempfile.TemporaryDirectory() as tmp:
        board = _stage(tmp)
        for tag, x, y, inside in (('half', '128', '92.2', False),
                                  ('home', '115.6', '92.2', True)):
            out = os.path.join(tmp, tag + '.kicad_pcb')
            r = run_utils.check([sys.executable, '-X', 'utf8', TOOL, board,
                                 out, 'set', 'Ref*', x, y,
                                 '--intent', LAP5], accept=True)
            s = _summary(r)
            zc = s['zone_check']
            assert not zc['worse'], (tag, zc)
            row, = zc['rows']
            assert row['outside_mm_before'] > 20, (tag, row)
            assert row['outside_mm_after'] < row['outside_mm_before'], row
            assert (row['outside_mm_after'] == 0.0) == inside, (tag, row)
            assert _pose(out, 'Ref*') == (float(x), float(y)), tag
    print("  PASS: a move toward the zone is written; one into it grades clean")


def test_a_warn_zone_is_reported_not_refused():
    with tempfile.TemporaryDirectory() as tmp:
        board = _stage(tmp)
        with open(LAP5, encoding='utf-8') as fh:
            plan = json.load(fh)
        plan['severity'] = {'zone_containment': 'warn'}
        warn = os.path.join(tmp, 'warn.json')
        with open(warn, 'w', encoding='utf-8') as fh:
            json.dump(plan, fh)
        r = run_utils.check([sys.executable, '-X', 'utf8', TOOL, board,
                             os.path.join(tmp, 'o.kicad_pcb'), 'set', 'Ref*',
                             *FAR, '--intent', warn], accept=True)
        zc = _summary(r)['zone_check']
        assert not zc['worse'] and zc['rows'][0]['severity'] == 'warn', zc
    print("  PASS: a demoted zone_containment is reported, never refused")


def test_without_intent_nothing_changes_and_a_bad_intent_is_exit_2():
    with tempfile.TemporaryDirectory() as tmp:
        board = _stage(tmp)
        r = run_utils.check([sys.executable, '-X', 'utf8', TOOL, board,
                             os.path.join(tmp, 'o.kicad_pcb'), 'set', 'Ref*',
                             *FAR], accept=True)
        assert 'zone_check' not in _summary(r)
        bad = os.path.join(tmp, 'bad.json')
        with open(bad, 'w', encoding='utf-8') as fh:
            fh.write('{"schema": 1, "kind": "floorplan-intent", "nope": 1}')
        r = run_utils.check([sys.executable, '-X', 'utf8', TOOL, board,
                             os.path.join(tmp, 'o2.kicad_pcb'), 'set',
                             'Ref*', *FAR, '--intent', bad],
                            refuse='cannot read the intent', code=2)
        assert _summary(r)['output'] is None
    print("  PASS: no --intent, no zone_check; an unreadable intent is exit 2")


TESTS = [
    test_run29_lap10_write_is_refused,
    test_force_writes_and_says_so,
    test_a_move_toward_the_zone_passes,
    test_a_warn_zone_is_reported_not_refused,
    test_without_intent_nothing_changes_and_a_bad_intent_is_exit_2,
]


def main():
    failed = 0
    for t in TESTS:
        try:
            t()
        except AssertionError as exc:
            failed += 1
            print(f"  FAIL: {t.__name__}: {exc}")
    print(f"\n{len(TESTS) - failed} passed, {failed} failed")
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
