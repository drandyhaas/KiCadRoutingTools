#!/usr/bin/env python3
"""What #708 actually is, measured rather than argued.

The issue says the quench's 0.1mm grid step is "incommensurate with imperial
board grids" and proposes inferring the board's pitch from a ladder. Two of the
three tables below say the mechanism is something else, and the third says the
proposed cure would have been bolted onto a line that buys nothing:

  A. THE DAMAGE IS REAL. One quench pass takes splitflap_driver from 59 of 65
     parts on its own 0.3175mm lattice to a small fraction of that. This is the
     claim the issue makes and it holds.

  B. THE SNAP REMOVES NO CANDIDATES. `_candidate_positions` builds candidates as
     `seed + ix*step` and then snaps the ABSOLUTE result. Every shipped `step`
     (1.0 place_optimize, 0.5 converge/place_route_loop, 0.2
     place_fanout_clearance) is a multiple of `grid_step` 0.1, so the snap is a
     pure translation of the whole candidate set by the seed's residue -- same
     count, same shape, shifted off the board's lattice. It is not a search
     device. `_group_offsets` already snaps the OFFSET, and at those same steps
     that snap is a literal no-op, which is the existence proof that the delta
     form is the correct one.

  C. THE LADDER IS DEGENERATE. Occupancy is monotone non-increasing along
     divisibility chains, so ties are structural rather than accidental, and the
     argmax the issue proposes is undefined exactly where it matters. The table
     also refutes the issue's claim that kit-dev-coldfire-xilinx_5213 "shows the
     same signature at 1.27mm": 1.27 scores far below its own maximum, and that
     maximum is below any floor that admits a real board.

Boards are DE-DUPLICATED by their footprint-position multiset before the corpus
table is summarised, because several tracked files are the same placement --
counting one 8-part fixture six times is how a majority gets manufactured.

NOT named `test_*.py`: `tests/run_all.py` does not collect it, because table A
runs real quench passes (minutes) and it asserts nothing. It is a measurement,
and its numbers are quoted in the #708 PR and in placement/README.md.

    python3 -X utf8 tests/measure_708_lattice.py            # all three tables
    python3 -X utf8 tests/measure_708_lattice.py --no-quench # B and C only
"""
from __future__ import annotations

import argparse
import contextlib
import hashlib
import io
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _sub in ('py_router', 'py_placer', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _sub))

import run_utils  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402
from placement.utility import snap_to_grid  # noqa: E402

# The ladder issue #708 proposes. Kept verbatim so the table answers the
# issue's own question rather than a reshaped one.
LADDER = (0.05, 0.1, 0.2, 0.25, 0.3175, 0.5, 0.635, 1.0, 1.27, 2.54)
TOL_MM = 0.001          # 1 um, the issue's own tolerance

# The steps that actually ship, and the tool each comes from. `grid_step` is
# 0.1 (routing_defaults.GRID_STEP) at every one of them.
SHIPPED_STEPS = ((10.0, 1.0, 'place_optimize --step'),
                 (3.0, 1.0, 'place_optimize --max-displacement 3'),
                 (2.0, 0.5, 'converge poses'),
                 (1.0, 0.2, 'place_fanout_clearance'),
                 (3.0, 0.25, 'a step that is NOT a multiple of grid_step'))


def occupancy(vals, step, tol=TOL_MM):
    """Fraction of `vals` within `tol` of a multiple of `step`."""
    if not vals or step <= 0:
        return 0.0
    return sum(1 for v in vals
               if abs(v / step - round(v / step)) * step <= tol) / len(vals)


def coords(pcb):
    """Footprint ORIGINS, x and y pooled.

    Origins, not pads: a pad sits at origin + a package pitch (0.5mm, 0.65mm)
    that is on no board lattice, so a pad sample measures the footprint library
    rather than the layout.
    """
    fps = [f for f in pcb.footprints.values()]
    xs = [f.x for f in fps if math.isfinite(f.x)]
    ys = [f.y for f in fps if math.isfinite(f.y)]
    return xs + ys, len(fps)


def profile(vals):
    return {s: occupancy(vals, s) for s in LADDER}


def finest_within(prof, slack=0.02):
    """The FINEST rung within `slack` of the maximum -- not the argmax.

    The ladder contains divisibility chains (0.3175 | 0.635 | 1.27 | 2.54 and
    0.05 | 0.1 | 0.2), and occupancy is monotone non-increasing along one: if
    d divides s then every on-s point is on-d, so occ(d) >= occ(s). Ties are
    therefore structural, and an argmax has no defined answer on them.
    """
    best = max(prof.values())
    for s in LADDER:
        if prof[s] >= best - slack:
            return s, best
    return None, best


def pose_hash(pcb):
    """Identify a PLACEMENT, so the same layout in five files counts once."""
    fps = sorted((round(f.x, 6), round(f.y, 6))
                 for f in pcb.footprints.values())
    return hashlib.sha1(repr(fps).encode()).hexdigest()[:12]


# --------------------------------------------------------------- table B

def candidate_sets(seed, max_disp, step, grid_step):
    """(snapped, unsnapped) candidate lists, exactly as _candidate_positions
    builds them -- absolute snap, deduped on the same rounded key."""
    out = []
    for do_snap in (True, False):
        seen, got = set(), []
        n = int(max_disp / step)
        for ix in range(-n, n + 1):
            for iy in range(-n, n + 1):
                cx = seed[0] + ix * step
                cy = seed[1] + iy * step
                if math.hypot(cx - seed[0], cy - seed[1]) > max_disp + 1e-9:
                    continue
                if do_snap:
                    cx = snap_to_grid(cx, grid_step)
                    cy = snap_to_grid(cy, grid_step)
                key = (round(cx, 4), round(cy, 4))
                if key not in seen:
                    seen.add(key)
                    got.append((cx, cy))
        out.append(got)
    return out[0], out[1]


def table_b(grid_step=0.1):
    # An off-raster seed, so the residue the snap discards is non-zero.
    # 131.4450 is interf_u BUS1: exactly 207 * 0.635, a clean 25-mil pose.
    seed = (131.4450, 138.4300)
    print('B. What the ABSOLUTE snap removes, at every shipped step '
          '(grid_step=%.2f)' % grid_step)
    print('   seed = (%.4f, %.4f) = (%.1f, %.1f) x 0.635, i.e. ON a 25-mil '
          'lattice' % (seed[0], seed[1], seed[0] / 0.635, seed[1] / 0.635))
    print()
    print('   %-9s %-7s %9s %11s %7s  %s'
          % ('max_disp', 'step', 'snapped', 'unsnapped', 'lost', 'shape'))
    for max_disp, step, who in SHIPPED_STEPS:
        snapped, plain = candidate_sets(seed, max_disp, step, grid_step)
        off = (round(snapped[0][0] - plain[0][0], 9),
               round(snapped[0][1] - plain[0][1], 9))
        pure = (len(snapped) == len(plain) and
                all(abs((a - p) - off[0]) < 1e-9 and abs((b - q) - off[1]) < 1e-9
                    for (a, b), (p, q) in zip(snapped, plain)))
        shape = ('pure translation by %s' % (off,)) if pure else 'reshaped'
        print('   %-9s %-7s %9d %11d %7d  %s   (%s)'
              % (max_disp, step, len(snapped), len(plain),
                 len(plain) - len(snapped), shape, who))
    print()
    print('   _group_offsets snaps the OFFSET. max |snap(ix*step) - ix*step|:')
    for step in sorted({s for _md, s, _w in SHIPPED_STEPS}):
        m = max(abs(snap_to_grid(ix * step, grid_step) - ix * step)
                for ix in range(-40, 41))
        verdict = 'NO-OP' if m == 0.0 else 'perturbs by up to %.3f' % m
        print('     step=%-6s %.3e   %s' % (step, m, verdict))
    print()
    print('   Reading: at every step the tool actually ships, the absolute snap')
    print('   loses zero candidates and merely translates the set off the')
    print('   seed\'s own lattice. The delta snap loses nothing and preserves')
    print('   it. That is the whole fix.')
    print()


# --------------------------------------------------------------- table C

def table_c(boards):
    rows = []
    seen_poses = {}
    for path in boards:
        try:
            pcb = parse_kicad_pcb(path)
        except Exception as exc:                     # noqa: BLE001
            print('   %-34s PARSE FAILED: %s' % (os.path.basename(path), exc))
            continue
        vals, n = coords(pcb)
        if not vals:
            continue
        h = pose_hash(pcb)
        rows.append((os.path.basename(path).replace('.kicad_pcb', ''),
                     n, profile(vals), h))
        seen_poses.setdefault(h, []).append(
            os.path.basename(path).replace('.kicad_pcb', ''))

    print('C. Ladder occupancy per board (footprint origins, x and y pooled, '
          '%g um tolerance)' % (TOL_MM * 1000))
    print()
    hdr = '   %-32s %4s ' % ('board', 'n') + ' '.join('%6s' % s for s in LADDER)
    print(hdr + '   %8s %s' % ('pick', 'ties'))
    for name, n, prof, _h in rows:
        pick, best = finest_within(prof)
        ties = [s for s in LADDER if prof[s] >= best - 0.02]
        print('   %-32s %4d ' % (name, n)
              + ' '.join('%6.2f' % prof[s] for s in LADDER)
              + '   %8s %s' % (pick, ','.join(str(t) for t in ties)
                               if len(ties) > 1 else '-'))
    print()
    dupes = {h: names for h, names in seen_poses.items() if len(names) > 1}
    print('   %d files, %d DISTINCT placements. Files sharing a placement:'
          % (len(rows), len(seen_poses)))
    for _h, names in sorted(dupes.items(), key=lambda kv: -len(kv[1])):
        print('     %s' % ', '.join(sorted(names)))
    if not dupes:
        print('     (none)')
    print()
    print('   Reading: every board whose best rung is coarse ties with that')
    print('   rung\'s divisors, so the argmax #708 proposes is undefined there.')
    print('   Any occupancy floor must also survive de-duplication -- the')
    print('   duplicate groups above are one placement each, not one vote each.')
    print()
    return rows


# --------------------------------------------------------------- table A

def table_a(rows, names, max_displacement=3.0):
    """Quench each named board and report how many parts left its lattice."""
    from placement.quench import quench
    print('A. What one quench pass does to the board\'s own lattice')
    print()
    print('   %-32s %6s %8s  %-22s %-22s %s'
          % ('board', 'n', 'lattice', 'on-lattice BEFORE',
             'on-lattice AFTER', 'moved'))
    by_name = {r[0]: r for r in rows}
    for name in names:
        row = by_name.get(name)
        if row is None:
            print('   %-32s SKIP (not in corpus)' % name)
            continue
        _n_name, n, prof, _h = row
        lattice, _best = finest_within(prof)
        path = os.path.join(ROOT, 'kicad_files', name + '.kicad_pcb')
        pcb = parse_kicad_pcb(path)
        before_vals, _ = coords(pcb)
        occ_before = occupancy(before_vals, lattice)
        # The engine narrates unconditionally; swallow it so the table is a
        # table. The run itself is untouched.
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            placements = quench(pcb, path, max_displacement=max_displacement,
                                step=1.0, grid_step=0.1, clearance=0.2,
                                board_edge_clearance=0.55, max_passes=4,
                                verbose=False)
        moved = {p['reference']: (p['new_x'], p['new_y'])
                 for p in (placements or [])}
        after = []
        for ref, fp in pcb.footprints.items():
            x, y = moved.get(ref, (fp.x, fp.y))
            after.extend([x, y])
        occ_after = occupancy(after, lattice)
        print('   %-32s %6d %8s  %6.3f (%3d/%3d)        %6.3f (%3d/%3d)        %d'
              % (name, n, lattice,
                 occ_before, round(occ_before * len(before_vals)),
                 len(before_vals),
                 occ_after, round(occ_after * len(after)), len(after),
                 len(moved)))
    print()
    print('   Counted over x and y separately, so the denominator is 2n.')
    print()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--no-quench', action='store_true',
                    help='skip table A (the slow one)')
    ap.add_argument('--boards', nargs='*',
                    default=['splitflap_driver', 'flat_hierarchy', 'sonde_u'],
                    help='boards for table A (default: three with a real '
                         'non-0.1 lattice)')
    args = ap.parse_args()

    boards = run_utils.corpus_boards()
    if not boards:
        print('SKIP: git could not enumerate the corpus '
              '(tarball export, or no git on PATH)')
        return 0

    table_b()
    rows = table_c(boards)
    if not args.no_quench:
        table_a(rows, args.boards)
    return 0


if __name__ == '__main__':
    sys.exit(main())
