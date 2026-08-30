#!/usr/bin/env python3
"""The #703 rig, checked by REGENERATING a declared subset (not by shipping 87 rows).

WHY THIS REPLACED A COMMITTED ROWS FILE

The study's 87 rows were committed once, at 276 KB, so `docs/placement-predictors.md`
could not drift from them. drandyhaas reviewed that and asked for the file to go:
the results are regenerable, and he checked it rather than trusting the contract --
`--task esp_prog:authored` rebuilt a row in 4.4 s and every predictor,
`truth.blocking`, `truth.quality`, `argv_sha` and `poses_sha256` matched.

He is right, and the trade is worth stating plainly because it is a real cost.
With the rows gone, the doc's rho table has no automated change detector; it is
the recorded finding, and re-deriving it is an **8.8 hour** job (the sum of
`provenance.total_seconds` over the 87 rows; median 217 s, max 2012 s on
`tigard:perturb-wrong_side`). What is kept instead is a detector on the RIG:
four cheap variants, regenerated on demand, whose predictors and routed truth
are pinned here as literals.

That catches the failure that actually matters day to day -- a change to the
placement engine, the predictor extraction, or the route argv silently moving
the numbers the study measured -- without shipping the artifact.

WHAT IS COMPARED, AND WHAT DELIBERATELY IS NOT

  * `poses_sha256`, NOT the input board's raw bytes. The withdrawn rows file
    stamped `sha256(board file)`, and all four of its board hashes were CRLF
    hashes -- so the gate that checked them was green only on a Windows
    checkout and red on macOS for 3 of 4 boards. `poses_sha256` is computed
    from PARSED footprint poses and is identical either way. (Found in the same
    review; same family as the pre-existing `test_763_kicad_locate` failure,
    pointing the other way.)
  * `routed_board_sha` is NEVER compared. KiCad stamps fresh UUIDs into every
    written board, and CLAUDE.md says so in as many words: "outputs carry
    per-run random UUIDs ... never hash or whole-file-diff `.kicad_pcb` outputs
    to judge determinism". Compare the graded counts instead, which is what
    `truth` is.
  * `seconds` is recorded as measured and never asserted on -- it is a property
    of the machine, and this repo's own doctrine is that wall-clock breaks
    cross-machine determinism.

This test ROUTES, so it is an integration test: about 75 s for the four rows.

    python3 -X utf8 tests/test_703_predictor_regen.py
    python3 -X utf8 tests/test_703_predictor_regen.py --row esp_prog:authored
    python3 -X utf8 tests/stress/predictor_study.py --verify-row esp_prog:authored
"""
import argparse
import os
import shutil
import sys
import tempfile

RUN_ALL_TIMEOUT = 1200

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tests', 'stress'))

#: The declared subset, and the values a correct rig reproduces.
#:
#: Four variants spanning the three generators -- the authored board, a
#: perturbed one, a quench candidate, and a second board -- chosen for cost:
#: every one routes in well under a minute. They are NOT a sample of anything
#: and no statistic is computed from them; they are a change detector.
#:
#: MEASURED, from the run recorded in the pull request. Never predicted.
EXPECTED = {
    'esp_prog:authored': dict(
        poses_sha256='439485f758347d1929cde06f8ab34ce7c6e6e2174df5d85be98d48afbde51783',
        argv_sha='52aaeed47e14fea796be12c36db09c605a4b8ec588da12bded6a2573b1c7f0b0',
        seconds=8.2,
        truth={'headline': 0,
               'quality': {'vias': 30, 'copper_mm': 342.83, 'segments': 247}},
        predictors={
            'crossings': 53, 'hpwl': 253.98092000000003,
            'halo': 125.4377644075392, 'overlap_area': 1.1400451712000104,
            'pad_copper': 0, 'pad_clearance_pairs': 0,
            'edge': 16.612682999999876, 'total': 914.1253439875175,
            'oob_count': 0,
        }),
    'esp_prog:perturb-scatter-d1': dict(
        poses_sha256='f58b7226eee4c4826a6430d1c972deb2cab1daf05ec2a17b1b7d56c9f88c4bdc',
        argv_sha='52aaeed47e14fea796be12c36db09c605a4b8ec588da12bded6a2573b1c7f0b0',
        seconds=11.2,
        truth={'headline': 0,
               'quality': {'vias': 36, 'copper_mm': 340.87, 'segments': 282}},
        predictors={
            'crossings': 56, 'hpwl': 253.13733999999994,
            'halo': 132.15233459554824, 'overlap_area': 1.1400451712000104,
            'pad_copper': 0, 'pad_clearance_pairs': 0,
            'edge': 15.019439919999892, 'total': 948.3549654632784,
            'oob_count': 0,
        }),
    # The quench candidate, kept because it is the study's own headline in
    # miniature: this placement scores 23 crossings against the authored
    # board's 53 -- the best on the slate -- and routes to blocking 3 where the
    # authored board routes to 0.
    'esp_prog:portfolio-1': dict(
        poses_sha256='4c299873cd66c843dc6b27fcadf99c9782a7e7e1a3b51328a5b5cee9ca6593d8',
        argv_sha='52aaeed47e14fea796be12c36db09c605a4b8ec588da12bded6a2573b1c7f0b0',
        seconds=23.8,
        truth={'headline': 3,
               'quality': {'vias': 30, 'copper_mm': 303.26, 'segments': 270}},
        predictors={
            'crossings': 23, 'hpwl': 236.74883999999992,
            'halo': 113.8065780241933, 'overlap_area': 1.0,
            'pad_copper': 0, 'pad_clearance_pairs': 0,
            'edge': 11.574407139199987, 'total': 582.4219077763819,
            'oob_count': 0,
        }),
    # A SECOND board, so a change that only moves one board's numbers cannot
    # pass. Its argv_sha differs from esp_prog's, which is the frozen-argv
    # contract visible in the data.
    'splitflap_driver:authored': dict(
        poses_sha256='8b89746bc1a8528de6349f54dd18874ca11d8c7a1f73dd657edce65d460391d9',
        argv_sha='d32ea90c2e348bd6c1fb318983e73f953f179d1ea85aeca7ad4ed4b2bed5cbdc',
        seconds=32.6,
        truth={'headline': 0,
               'quality': {'vias': 168, 'copper_mm': 2915.15,
                           'segments': 1437}},
        predictors={
            'crossings': 300, 'hpwl': 2504.4400000000014,
            'halo': 297.4273114820511, 'overlap_area': 1.7621459846850488e-13,
            'pad_copper': 0, 'pad_clearance_pairs': 0,
            'edge': 135.1680159999995, 'total': 5872.4776824793535,
            'oob_count': 14,
        }),
}

FAILURES = []


def check(cond, what):
    if cond:
        print(f'  ok   {what}')
    else:
        print(f'  FAIL {what}')
        FAILURES.append(what)


def t_the_subset_is_declared_coherently():
    """Cheap, board-only assertions that need no routing at all."""
    import predictor_study as PS
    boards = {b['key'] for b in PS.CALIBRATION_CANDIDATES}
    for key, want in sorted(EXPECTED.items()):
        bk, _, variant = key.partition(':')
        check(bk in boards, f'{key}: {bk} is a declared board')
        check(variant in PS.VARIANTS, f'{key}: {variant} is a declared variant')
        check(len(want['poses_sha256']) == 64, f'{key}: poses_sha256 is a sha')
        check(set(want['predictors']) <= set(PS.PREDICTOR_KEYS),
              f'{key}: every pinned predictor is a real one')
    check(len({k.split(':')[0] for k in EXPECTED}) >= 2,
          'the subset spans at least TWO boards, so a one-board change cannot '
          'slip through')
    shas = {v['argv_sha'] for v in EXPECTED.values()}
    per_board = {}
    for k, v in EXPECTED.items():
        per_board.setdefault(k.split(':')[0], set()).add(v['argv_sha'])
    check(all(len(s) == 1 for s in per_board.values()),
          'every variant of a board pins ONE argv_sha -- the frozen-argv '
          'contract, visible in the data')
    check(len(shas) == len(per_board),
          'and different boards pin different argv_sha')


def t_regenerates(only=None):
    """The real check: rebuild each variant and diff against the literals."""
    import predictor_study as PS
    work = tempfile.mkdtemp(prefix='regen703_')
    try:
        for key, want in sorted(EXPECTED.items()):
            if only and key != only:
                continue
            bk, _, variant = key.partition(':')
            board = next(b for b in PS.CALIBRATION_CANDIDATES
                         if b['key'] == bk)
            row = PS.run_task(bk, board['file'], variant, work, 0, 900)
            bad = PS.compare_row(row, want)
            for line in bad:
                print(f'       {line}')
            check(not bad,
                  f'{key} regenerates identically '
                  f'({row["route"].get("seconds")}s vs {want["seconds"]}s '
                  f'recorded)')
    finally:
        shutil.rmtree(work, ignore_errors=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--row', default=None,
                    help='regenerate ONE declared row (BOARD:VARIANT)')
    ap.add_argument('--list', action='store_true')
    a = ap.parse_args()
    if a.list:
        for k in sorted(EXPECTED):
            print(k)
        return 0
    if a.row and a.row not in EXPECTED:
        print(f'no such declared row: {a.row}; try --list', file=sys.stderr)
        return 2
    print('t_the_subset_is_declared_coherently:')
    t_the_subset_is_declared_coherently()
    print('t_regenerates:')
    t_regenerates(a.row)
    if FAILURES:
        print(f'\nFAILED {len(FAILURES)}:')
        for f in FAILURES:
            print(f'  - {f}')
        return 1
    print('\ntest_703_predictor_regen: all checks passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
