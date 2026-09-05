#!/usr/bin/env python3
"""#849: what the lane-ledger hoist costs and saves, measured here.

    python3 -X utf8 tests/measure_849_hoist.py            # every board below
    python3 -X utf8 tests/measure_849_hoist.py glasgow_revC

NOT named `test_*`, so `run_all.py` never collects it: it is a measurement,
not a gate, and its seconds are load-dependent by nature. It exists because
the numbers in the PR and in `board_lane_context`'s docstring are otherwise
only re-derivable from a scratch directory that is gitignored -- and a
"measured" number nobody else can re-measure is a claim, not a measurement.

TWO BASES, and they are not comparable -- which is the trap this file exists
to make impossible to fall into a second time:

  `--refs cli`    the refs `check_channels` auto-detects at the given lane
                  (2 / 7 / 9 on the three boards below). This is what the
                  TOOL costs.
  `--refs fine`   `escape.fine_pitch_parts` (10 / 10 / 24). This is the basis
                  issue #849 used for its 12.6-15.8s, and the one to compare
                  its "~97% of the sweep is avoidable" against.

The PR's first draft compared the issue's 24-ref sweep against a 9-ref CLI
wall clock and reported that the issue "does not reproduce". On its own basis
it reproduces: see the header of `tests/test_849_lane_context.py`.

The courtyard PARSE COUNT is the load-independent half. It is the count of
calls to `placement.parser.extract_courtyard_sides`, which opens and
regex-walks the whole board file, and it goes from one-per-ref to one.
"""
import argparse
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (ROOT, os.path.join(ROOT, 'py_router'), os.path.join(ROOT, 'py_placer'),
           os.path.join(ROOT, 'py_tools')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from kicad_parser import parse_kicad_pcb, detect_package_type   # noqa: E402
from placement import escape as E                               # noqa: E402
from placement import parser as pparser                         # noqa: E402
from placement import routability as R                          # noqa: E402

BOARDS = ['tigard', 'rp2350_fpga_eensy_prePlane', 'glasgow_revC']
LANE = {'clearance': 0.09, 'track_width': 0.127, 'grid_step': 0.05}


def cli_refs(pcb, track, clearance):
    """`check_channels`' own auto-detection, at the given lane.

    Copied rather than imported because it is inline in that tool's `main()`.
    It is the only copy in the tree and it is only ever used to REPORT a
    basis, never to grade anything -- see `--refs cli` above.
    """
    floor = 2 * (track + clearance)
    out = []
    for ref, fp in sorted((pcb.footprints or {}).items()):
        try:
            kind = detect_package_type(fp)
        except Exception:
            kind = None
        if kind in ('QFN', 'QFP', 'BGA'):
            out.append(ref)
            continue
        xs = sorted({round(p.global_x, 3) for p in (fp.pads or [])})
        gaps = [b - a for a, b in zip(xs, xs[1:]) if b - a > 1e-3]
        if gaps and min(gaps) < floor and len(fp.pads or []) >= 8:
            out.append(ref)
    return out


def sweep(pcb, path, refs, ctx=None):
    kw = dict(LANE, pcb_file=path)
    if ctx is not None:
        kw['context'] = ctx
    return [R.face_lane_ledger(pcb, r, **kw) for r in refs]


def hoisted(pcb, path, refs):
    """One FRESH context per run -- one per run is what the caller builds.

    Reusing a context across repetitions measures a warm cache and reports
    two orders of magnitude more than the change is worth.
    """
    return sweep(pcb, path, refs,
                 R.board_lane_context(pcb, LANE['clearance'], pcb_file=path))


def parses(fn, *a):
    """(result, times the board's courtyards were parsed)."""
    real = pparser.extract_courtyard_sides
    n = [0]

    def spy(p, _r=real, _n=n):
        _n[0] += 1
        return _r(p)

    pparser.extract_courtyard_sides = spy
    try:
        return fn(*a), n[0]
    finally:
        pparser.extract_courtyard_sides = real


def timed(fn, *a, reps=3):
    best = None
    for _ in range(reps):
        t = time.perf_counter()
        fn(*a)
        d = time.perf_counter() - t
        best = d if best is None else min(best, d)
    return best


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('boards', nargs='*', default=None)
    ap.add_argument('--refs', choices=('cli', 'fine', 'both'), default='both')
    ap.add_argument('--reps', type=int, default=3)
    a = ap.parse_args()

    names = a.boards or BOARDS
    bases = ('cli', 'fine') if a.refs == 'both' else (a.refs,)
    print(f'lane: clearance {LANE["clearance"]} track {LANE["track_width"]} '
          f'grid {LANE["grid_step"]}; min-of-{a.reps}, one process, the '
          f'context built INSIDE the timed region\n')
    print(f'{"board":28s} {"basis":5s} {"refs":>4s} {"plain":>8s} '
          f'{"hoisted":>8s} {"x":>6s}  parses')
    for name in names:
        path = os.path.join('kicad_files', name + '.kicad_pcb')
        full = os.path.join(ROOT, path)
        if not os.path.isfile(full):
            print(f'{name:28s} SKIP (not in this checkout)')
            continue
        pcb = parse_kicad_pcb(full)
        for basis in bases:
            refs = (cli_refs(pcb, LANE['track_width'], LANE['clearance'])
                    if basis == 'cli' else E.fine_pitch_parts(pcb))
            if not refs:
                print(f'{name:28s} {basis:5s} {0:4d}  (no refs at this basis)')
                continue
            _, n_plain = parses(sweep, pcb, full, refs)
            _, n_ctx = parses(hoisted, pcb, full, refs)
            t_plain = timed(sweep, pcb, full, refs, reps=a.reps)
            t_ctx = timed(hoisted, pcb, full, refs, reps=a.reps)
            print(f'{name:28s} {basis:5s} {len(refs):4d} {t_plain:7.3f}s '
                  f'{t_ctx:7.3f}s {t_plain / max(t_ctx, 1e-9):5.1f}x  '
                  f'{n_plain} -> {n_ctx}', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
