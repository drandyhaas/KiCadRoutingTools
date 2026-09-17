#!/usr/bin/env python3
"""#975: what one seat candidate's pad-copper edge check costs, before and after
the board constants were hoisted into `EdgeCopperContext`.

    python3 -B -X utf8 tests/measure_975_seat_copper.py            # every case
    python3 -B -X utf8 tests/measure_975_seat_copper.py --reps 5

NOT named `test_*`, so `run_all.py` never collects it: milliseconds depend on
the machine and its load, so this is a measurement, not a gate. The
load-independent half -- that a candidate reads no file -- is asserted in
`tests/test_975_edge_copper_context.py` (arm B) and is also counted here.

BEFORE is the shape #971's withdrawn first version used per candidate
(KRT-961 f0a3cc90, `connector_geometry.candidate_copper`): copy the footprint
with its pads moved to the pose, then `grade_pad_edge_clearance` on a board
holding only that footprint -- which re-reads the `.kicad_pcb`, the
`.kicad_dru` and the `.kicad_pro` every time. AFTER is
`EdgeCopperContext.pose_copper(fp, pose)` on a context built once; BUILD is
that one construction, timed separately because a search pays it once per
state, not per candidate.

Poses: 13 along-edge offsets (the seat ladder's rung count) x 3 rotations
(own, +90, +33) = 39 per part, the same 39 for BEFORE and AFTER. Timings are
the minimum over `--reps`. BEFORE moves its pads with `pads_at_pose` too, so
the two cannot disagree on a pose: whether `pads_at_pose` puts pads where the
writer does is `tests/test_975_edge_copper_context.py`'s arm F, not this.

The #975 target is <= 0.2 ms per candidate on RECTANGULAR outlines; sampled
outlines run the DRC perimeter sampler per pad and are reported, not targeted.
"""
import argparse
import copy
import os
import sys
import time
from unittest.mock import patch

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (ROOT, os.path.join(ROOT, 'py_router'), os.path.join(ROOT, 'py_placer')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import builtins  # noqa: E402
import list_nets  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402
from placement import legality as L  # noqa: E402

CASES = [('esp_prog', 'USB1'), ('tigard', 'J7'), ('tigard', 'J1'), ('ulx3s', 'J1'),
         ('watchy', 'J2'), ('interf_u_unrouted', 'P1')]
TARGET_MS = 0.2


def poses(fp):
    rot = fp.rotation or 0.0
    return [(fp.x + 0.3 * k, fp.y, (rot + turn) % 360)
            for turn in (0.0, 90.0, 33.0) for k in range(-6, 7)]


def before(pcb, ref, pose, required, path):
    """The withdrawn per-candidate shape: a one-footprint board, graded fresh."""
    fp = copy.copy(pcb.footprints[ref])
    fp.pads = L.pads_at_pose(pcb.footprints[ref], pose)
    fp.x, fp.y, fp.rotation = pose
    one = copy.copy(pcb)
    one.footprints = {ref: fp}
    return L.grade_pad_edge_clearance(one, required, path)


def best(fn, reps):
    out = None
    for _ in range(reps):
        t = time.perf_counter()
        fn()
        dt = time.perf_counter() - t
        out = dt if out is None else min(out, dt)
    return out


def reads(fn):
    counts = {'n': 0}
    real_open, real_rdr = builtins.open, list_nets.read_design_rules

    def spy_open(*a, **k):
        counts['n'] += 1
        return real_open(*a, **k)

    def spy_rdr(*a, **k):
        counts['n'] += 1
        return real_rdr(*a, **k)
    with patch.object(builtins, 'open', spy_open), \
            patch.object(list_nets, 'read_design_rules', spy_rdr):
        fn()
    return counts['n']


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--reps', type=int, default=3)
    ap.add_argument('--required', type=float, default=0.55)
    args = ap.parse_args()
    print(f'{"board/part":26s} {"pads":>4s} {"outline":>8s} {"build ms":>9s} '
          f'{"before ms/cand":>15s} {"after us/cand":>14s} {"reads/cand":>11s}  target')
    for board, ref in CASES:
        path = os.path.join(ROOT, 'kicad_files', board + '.kicad_pcb')
        if not os.path.exists(path):
            print(f'{board}/{ref}: board missing, skipped')
            continue
        pcb = parse_kicad_pcb(path)
        fp = pcb.footprints[ref]
        ps = poses(fp)
        build = best(lambda: L.EdgeCopperContext(pcb, args.required, path), args.reps)
        ctx = L.EdgeCopperContext(pcb, args.required, path)
        after = best(lambda: [ctx.pose_copper(fp, p) for p in ps], args.reps) / len(ps)
        old = best(lambda: [before(pcb, ref, p, args.required, path) for p in ps],
                   args.reps) / len(ps)
        reads_before = reads(lambda: before(pcb, ref, ps[0], args.required, path))
        reads_after = reads(lambda: ctx.pose_copper(fp, ps[0]))
        copper = sum(1 for p in fp.pads if not L._pad_has_no_copper(p))
        outline = 'rect' if ctx.rectangular else 'sampled'
        verdict = (('PASS' if after * 1e3 <= TARGET_MS else 'OVER') if ctx.rectangular
                   else 'n/a (sampled)')
        print(f'{board + "/" + ref:26s} {copper:4d} {outline:>8s} {build * 1e3:9.2f} '
              f'{old * 1e3:15.3f} {after * 1e6:14.1f} {reads_before:>4d} -> {reads_after:<3d}'
              f'  {verdict}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
