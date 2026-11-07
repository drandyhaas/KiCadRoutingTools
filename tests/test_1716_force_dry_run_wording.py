#!/usr/bin/env python3
"""#1716: `place_pose --force --dry-run` must not say it WROTE the pose.

A forced pose prints one WARNING line naming the finding it was written over.
On a dry run nothing is written (the summary's `output` is None and the file
does not exist), so the same line used to tell a caller who asked "what would
--force ship" that it had already shipped. The verb now follows the outcome:
WOULD BE WRITTEN on a dry run, WRITTEN on a run that wrote.

The case: a capacitor set onto a resistor's pads, a pose place_pose refuses
without --force (one new pad conflict).
"""
import json
import os
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, 'tests'))

import run_utils                                            # noqa: E402

BOARD = os.path.join(REPO, 'kicad_files', 'esp_prog.kicad_pcb')
COPY = os.path.join(REPO, 'py_router', 'copy_board.py')
TOOL = run_utils.tool('place_pose.py')
#: C1 onto R1's pads: pad_conflicts 0 -> 1.
POSE = ('set', 'C1', '136.4', '98.8')


def _summary(r):
    lines = [x for x in r.stdout.splitlines() if x.startswith('JSON_SUMMARY:')]
    assert len(lines) == 1, r.stdout[-1500:]
    return json.loads(lines[0].split('JSON_SUMMARY: ', 1)[1])


def _warning(r):
    lines = [x for x in r.stdout.splitlines() if x.startswith('WARNING:')
             and 'under --force' in x]
    assert len(lines) == 1, r.stdout[-1500:]
    return lines[0]


def test_dry_run_says_would_be_written():
    run_utils.evidence(BOARD)
    with tempfile.TemporaryDirectory() as tmp:
        board = os.path.join(tmp, 'in.kicad_pcb')
        run_utils.check([sys.executable, '-X', 'utf8', COPY, BOARD, board],
                        accept=True)
        out = os.path.join(tmp, 'out.kicad_pcb')
        # Without --force the pose is refused: the finding is real.
        run_utils.check([sys.executable, '-X', 'utf8', TOOL, board, out,
                         *POSE, '--dry-run'],
                        refuse='place_pose REFUSED', code=4)
        r = run_utils.check([sys.executable, '-X', 'utf8', TOOL, board, out,
                             *POSE, '--force', '--dry-run'], accept=True)
        s = _summary(r)
        assert s['dry_run'] is True and s['forced'] is True, s
        assert s['output'] is None and not os.path.exists(out), s
        w = _warning(r)
        assert w.startswith('WARNING: WOULD BE WRITTEN under --force'), w
        assert '--dry-run: nothing was written' in r.stdout, r.stdout[-800:]

        # The control: the same call that writes keeps the WRITTEN verb.
        r = run_utils.check([sys.executable, '-X', 'utf8', TOOL, board, out,
                             *POSE, '--force'], accept=True)
        s = _summary(r)
        assert s['output'] == out and os.path.isfile(out), s
        w = _warning(r)
        assert w.startswith('WARNING: WRITTEN under --force'), w
    print("  PASS: --force --dry-run says WOULD BE WRITTEN and writes nothing; "
          "--force alone says WRITTEN and writes")


TESTS = [test_dry_run_says_would_be_written]


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
