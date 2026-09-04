#!/usr/bin/env python3
"""#714 IDENT: a write with no side key is BYTE-IDENTICAL to the shipped writer.

The #714 flip is opt-in: a placement dict carrying no `new_side` key must take
the path that shipped, byte for byte, on every board. About twenty producers
build those dicts (`quench`, `seeder`, `portfolio`, `relocate`, `perturb`,
`fanout_clearance`, `place_reconstruct`, `place_seed`, `converge`,
`net_rescue`), and none of them will be changed, so this gate is what stands
between them and a regression.

WHY A SHA AND NOT A FIELD COMPARISON. The failure this guards against is the
writer emitting *slightly* different text -- a `(layer ...)` line rewritten
unconditionally, a number reformatted, a node reordered. Every one of those
survives a parse-and-compare-fields check and none survives a hash.

WHY THE BASELINE IS TRUSTWORTHY. It is recorded in the same PR as the change it
guards, which makes it self-certifying and therefore worth nothing on its own.
What makes it load-bearing is `tests/mutate_714.py`'s row that perturbs the
`:.6f` coordinate format in `writer.py` and REQUIRES this gate to go red. A
baseline no mutation can redden is decoration.

The board set is `run_utils.corpus_boards()` -- the boards git TRACKS. Not a
glob: `kicad_files/` accumulates generated outputs, so the same glob returns 22
entries on a clean clone and 33 on a machine that has run the suite, and a
baseline pinned to the second one fails in CI as machine-dependent flake.

    python3 tests/test_714_identity_write_unchanged.py
    python3 tests/test_714_identity_write_unchanged.py --write-baseline
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, os.path.join(REPO, 'py_placer'))
sys.path.insert(0, TESTS)

RUN_ALL_TIMEOUT = 900
BASELINE = os.path.join(TESTS, 'data', '714_identity_sha256.json')

# The command that produced the baseline, recorded IN the baseline so a reader
# never has to guess how to regenerate it.
BASELINE_CMD = 'python3 tests/test_714_identity_write_unchanged.py --write-baseline'


def _all_at_current(pcb):
    """Every parsed footprint at the pose it already has.

    The `perturb._all_at_current` shape (perturb.py), which is the call that
    exercises the writer's whole per-block loop for every block on the board --
    the widest identity write there is. Poses are unchanged; only the `(at)`
    formatting is, which is exactly what the hash is pinning.
    """
    return [{'reference': ref, 'new_x': round(fp.x, 6),
             'new_y': round(fp.y, 6), 'new_rotation': fp.rotation}
            for ref, fp in sorted(pcb.footprints.items())]


def _sha(path):
    h = hashlib.sha256()
    with open(path, 'rb') as fh:
        for chunk in iter(lambda: fh.read(1 << 16), b''):
            h.update(chunk)
    return h.hexdigest()


def measure():
    from kicad_parser import parse_kicad_pcb
    from placement.writer import write_placed_output
    from run_utils import corpus_boards

    boards = corpus_boards()
    if not boards:
        # A board set this test cannot identify is not a set to grade against.
        print("SKIP: git could not name the tracked corpus")
        sys.exit(77)

    out = {}
    tmp = tempfile.mkdtemp(prefix='krt714ident')
    try:
        for src in boards:
            name = os.path.basename(src)
            pcb = parse_kicad_pcb(src)
            dst = os.path.join(tmp, name)
            write_placed_output(src, dst, _all_at_current(pcb))
            out[name] = {'sha256': _sha(dst), 'footprints': len(pcb.footprints)}
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--write-baseline', action='store_true')
    args = ap.parse_args(argv)

    got = measure()
    if args.write_baseline:
        os.makedirs(os.path.dirname(BASELINE), exist_ok=True)
        with open(BASELINE, 'w', encoding='utf-8') as fh:
            json.dump({'_command': BASELINE_CMD, 'boards': got}, fh,
                      indent=1, sort_keys=True)
        print(f"wrote {BASELINE}: {len(got)} boards")
        return 0

    if not os.path.exists(BASELINE):
        # FAIL, never pass. A missing baseline is the state in which this gate
        # can no longer tell a regression from a fresh checkout.
        print(f"FAIL: no baseline at {BASELINE}. Regenerate with:\n"
              f"    {BASELINE_CMD}")
        return 1

    with open(BASELINE, encoding='utf-8') as fh:
        want = json.load(fh)['boards']

    problems = []
    if set(want) != set(got):
        for k in sorted(set(want) - set(got)):
            problems.append(f"board in baseline but not measured: {k}")
        for k in sorted(set(got) - set(want)):
            problems.append(f"board measured but not in baseline: {k}")
    for k in sorted(set(want) & set(got)):
        if want[k]['sha256'] != got[k]['sha256']:
            problems.append(f"{k}: identity write CHANGED\n"
                            f"    baseline {want[k]['sha256']}\n"
                            f"    now      {got[k]['sha256']}")
        if want[k]['footprints'] != got[k]['footprints']:
            problems.append(f"{k}: footprint count {want[k]['footprints']} -> "
                            f"{got[k]['footprints']}")

    # Non-vacuity: a gate that graded nothing must not pass. 22 tracked boards
    # today; the floor is deliberately below that so adding a board is not a
    # failure, and far above zero so an empty run is.
    if len(got) < 15:
        problems.append(f"only {len(got)} boards measured -- this gate cannot "
                        f"pass on a corpus it did not see")
    total_fps = sum(v['footprints'] for v in got.values())
    if total_fps < 1000:
        problems.append(f"only {total_fps} footprints written -- too few for "
                        f"the identity claim to mean anything")

    if problems:
        print(f"FAIL: {len(problems)} problem(s)")
        for p in problems[:25]:
            print("  " + p)
        return 1
    print(f"PASS: identity write byte-identical on {len(got)} tracked boards, "
          f"{total_fps} footprints")
    return 0


if __name__ == '__main__':
    sys.exit(main())
