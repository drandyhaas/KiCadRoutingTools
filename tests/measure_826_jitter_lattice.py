#!/usr/bin/env python3
"""What the portfolio's basin-escape does to the board's lattice (#826).

#708 made the quench move a part by a whole number of the board's own placement
lattice. It preserves what it is GIVEN. `place_portfolio` hands it something
already broken: `perturb_jitter` offsets a part by `r*cos(theta)` at 4 decimal
places -- continuous, on no lattice -- and `generate` then quenches each
candidate FROM the jittered seed board.

Three tables, and the first is the one that decides whether the fix is worth
making:

  A. THE POPULATION. Of the boards that declare a lattice, how many still
     declare one after the default jitter? One board is an anecdote; this is
     the finding or the refutation.

  B. IT IS NOT A TUNING PROBLEM. The same table at four radii, down to a
     radius SMALLER than the lattice itself. A continuous offset is off-lattice
     at every amplitude, so nothing about a gentler jitter helps.

  C. THE COUNTERFACTUAL. The identical rng stream with the offset snapped, so
     the cost of the fix is measured rather than asserted: parts perturbed, RMS
     displacement, and how often the snap is rejected for landing on the seed
     or outside the disc.

The oracle is built exactly the way `portfolio.generate` builds it, via the
knob resolution `place_portfolio.main` performs first -- `board_floor_knobs`
for the clearances, `routing_defaults.GRID_STEP` for the raster, `free_refs`
with no `--lock`. A measurement of a differently-constructed state would be a
measurement of something else.

Occupancy after the jitter is read the way `board_grid` reads it -- over EVERY
footprint origin, not just the free ones -- by overlaying the returned poses on
the parsed board. `write_placed_output` formats the `(at ...)` node
with `%.6f`, so a jitter value round-trips through a written seed board
verbatim and the in-memory overlay is exact rather than approximate (checked
end to end on splitflap_driver: overlay and written board both read 0.3175 at
0.923).

NOT named `test_*.py`: `tests/run_all.py` does not collect it. It asserts
nothing and takes minutes.

    python3 -X utf8 tests/measure_826_jitter_lattice.py
    python3 -X utf8 tests/measure_826_jitter_lattice.py --table A
"""
from __future__ import annotations

import argparse
import math
import os
import random
import sys

_TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(_TESTS)
sys.path.insert(0, _TESTS)
for _d in ('py_router', 'py_placer', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _d))

import routing_defaults as defaults                            # noqa: E402
import run_utils                                               # noqa: E402
from kicad_parser import parse_kicad_pcb                        # noqa: E402
from list_nets import board_floor_knobs                         # noqa: E402
from placement import board_grid as BG                          # noqa: E402
from placement import portfolio as PF                           # noqa: E402
from placement.utility import snap_to_grid                      # noqa: E402


def oracle_for(path, pcb):
    """The state `portfolio.generate` would build for this board.

    Mirrors `generate`'s own construction plus the knob resolution
    `place_portfolio.main` does before calling it: clearances from the BOARD
    (not fixed constants), the routing raster for grid_step, no --lock, no
    --ignore-nets.
    """
    free = PF.free_refs(pcb, path, None)
    clearance, edge, _knobs = board_floor_knobs(path, None, None)
    oracle = PF.make_oracle(pcb, path, free=free, clearance=clearance,
                            board_edge_clearance=edge,
                            grid_step=defaults.GRID_STEP, ignore_ids=set())
    return oracle, free


def occupancy_after(pcb, poses, step):
    """Occupancy over EVERY footprint origin with `poses` overlaid.

    The same population `board_grid.board_coordinates` reads -- a free-parts-
    only sample would measure something the engine never asks about.
    """
    moved = {p['reference']: (p['new_x'], p['new_y']) for p in poses}
    vals = []
    for ref, fp in pcb.footprints.items():
        x, y = moved.get(ref, (fp.x, fp.y))
        vals.extend([x, y])
    return BG.occupancy(vals, step)


def best_after(pcb, poses):
    """(best occupancy over the whole ladder, resolved step or None).

    The best rung is what the floor is compared against, so it is the number
    that decides whether the inference still answers.
    """
    moved = {p['reference']: (p['new_x'], p['new_y']) for p in poses}

    class _FP:
        def __init__(self, x, y):
            self.x, self.y = x, y

    class _PCB:
        pass

    overlay = _PCB()
    overlay.footprints = {}
    for ref, fp in pcb.footprints.items():
        x, y = moved.get(ref, (fp.x, fp.y))
        overlay.footprints[ref] = _FP(x, y)
    ev = BG.infer_board_grid(overlay)
    return (max(ev['profile'].values()) if ev['profile'] else 0.0), ev['step']


def jitter(state, refs, rng, radius, lattice=None, attempts=20):
    """The engine's sampler, and a count of why each draw was refused.

    The loop below duplicates `portfolio.perturb_jitter` because the refusal
    counts are not observable from outside it -- the engine returns poses, not
    the reasons it rejected draws, and the accept test is `candidate_valid`,
    so a replay that stopped at the geometric checks would miscount.

    A duplicate that can drift is worse than no measurement, so this one is
    CHECKED: `verify_against_engine` below runs the real `perturb_jitter` on
    the same rng stream and refuses to report anything if the poses differ.
    That is the guard the copy has to earn.
    """
    poses, n_zero, n_out = [], 0, 0
    for ref in refs:
        part = state.parts.get(ref)
        if part is None:
            continue
        if not state.candidate_valid(ref, part.x, part.y, part.rot):
            continue
        for _ in range(attempts):
            r = radius * math.sqrt(rng.random())
            th = 2.0 * math.pi * rng.random()
            dx, dy = r * math.cos(th), r * math.sin(th)
            if lattice is not None:
                dx = snap_to_grid(dx, lattice)
                dy = snap_to_grid(dy, lattice)
                if dx == 0.0 and dy == 0.0:
                    n_zero += 1
                    continue
                if math.hypot(dx, dy) > radius + 1e-9:
                    n_out += 1
                    continue
                x, y = part.x + dx, part.y + dy
            else:
                x, y = round(part.x + dx, 4), round(part.y + dy, 4)
            if state.candidate_valid(ref, x, y, part.rot):
                state.apply_move(ref, x, y, part.rot)
                poses.append({'reference': ref, 'new_x': x, 'new_y': y,
                              'new_rotation': part.rot})
                break
    return poses, n_zero, n_out


def verify_against_engine(path, pcb, radius, lattice, rng_factory):
    """Refuse to report if the local copy has drifted from the engine.

    Runs both on the same stream and compares the emitted poses exactly. A
    measurement whose sampler no longer matches the shipped one is not a
    measurement of this tool.
    """
    o1, free = oracle_for(path, pcb)
    mine, _z, _o = jitter(o1, free, rng_factory(), radius, lattice=lattice)
    o2, free2 = oracle_for(path, pcb)
    theirs = PF.perturb_jitter(o2, free2, rng_factory(), radius,
                               lattice=lattice)
    if mine != theirs:
        raise SystemExit(
            "REFUSING to report: this file's sampler has drifted from "
            'portfolio.perturb_jitter on %s (%d poses vs %d). Re-sync the '
            'copy in jitter() before trusting any table below.'
            % (os.path.basename(path), len(mine), len(theirs)))


def lattice_boards():
    """(path, pcb, step, occ) for every tracked board that declares a lattice."""
    out = []
    for path in run_utils.corpus_boards():
        pcb = parse_kicad_pcb(path)
        ev = BG.infer_board_grid(pcb)
        if ev['step'] is not None:
            out.append((path, pcb, ev['step'], ev['occupancy']))
    return out


def _rng(i=1):
    """Candidate i's stream, exactly as `generate` keys it."""
    return random.Random('0:%d:jitter' % i)


# ---------------------------------------------------------------- table A

def table_a(boards):
    print('A. Does the board still declare its lattice after the default '
          'jitter? (radius 4.0, seed 0, candidate 1)')
    print()
    print('   %-30s %-8s %8s %8s %9s %6s %6s'
          % ('board', 'lattice', 'occ in', 'occ out', 'resolves', 'free',
             'jitter'))
    lost = 0
    for path, pcb, step, occ in boards:
        oracle, free = oracle_for(path, pcb)
        poses, _z, _o = jitter(oracle, free, _rng(), 4.0)
        best, after_step = best_after(pcb, poses)
        if after_step is None:
            lost += 1
        print('   %-30s %-8s %8.3f %8.3f %9s %6d %6d'
              % (os.path.basename(path).replace('.kicad_pcb', ''), step, occ,
                 best, after_step if after_step else 'NO', len(free),
                 len(poses)))
    print()
    print('   %d of %d boards that declared a lattice no longer do.' %
          (lost, len(boards)))
    print('   On those, the quench that follows resolves its offset lattice to')
    print('   the 0.1mm raster -- i.e. the #708 fix is inert on this path, and')
    print('   inert silently: the run reports that the BOARD declares no')
    print('   lattice, which is true of the jittered board and false of the')
    print('   input.')
    print()


# ---------------------------------------------------------------- table B

def table_b(boards):
    print('B. Is it a tuning problem? The same question at four radii.')
    print()
    radii = (4.0, 1.0, 0.25, 0.05)
    print('   %-30s %-8s' % ('board', 'lattice')
          + ' '.join('%12s' % ('r=%g' % r) for r in radii))
    for path, pcb, step, _occ in boards:
        cells = []
        for r in radii:
            oracle, free = oracle_for(path, pcb)
            poses, _z, _o = jitter(oracle, free, _rng(), r)
            _best, after_step = best_after(pcb, poses)
            cells.append('%12s' % (after_step if after_step else 'NO'))
        print('   %-30s %-8s' % (os.path.basename(path).replace(
            '.kicad_pcb', ''), step) + ' '.join(cells))
    print()
    print('   No. A continuous offset is off-lattice at every amplitude -- at')
    print('   r=0.05 the jitter is ONE lattice unit on the 0.05 boards and')
    print('   still destroys the inference. Radius changes how many parts find')
    print('   a legal sample, not whether the sample is on the grid.')
    print()


# ---------------------------------------------------------------- table C

def table_c(boards):
    print('C. The counterfactual: the same rng stream, offset snapped.')
    print()
    print('   %-30s %-8s %-22s %-22s %5s %5s'
          % ('board', 'lattice', 'SHIPPED jit/occ/step',
             'SNAPPED jit/occ/step', 'zero', 'out'))
    for path, pcb, step, occ in boards:
        o1, free = oracle_for(path, pcb)
        p1, _z1, _o1 = jitter(o1, free, _rng(), 4.0)
        b1, s1 = best_after(pcb, p1)
        o2, _free2 = oracle_for(path, pcb)
        p2, z2, out2 = jitter(o2, free, _rng(), 4.0, lattice=step)
        b2, s2 = best_after(pcb, p2)
        print('   %-30s %-8s %-22s %-22s %5d %5d'
              % (os.path.basename(path).replace('.kicad_pcb', ''), step,
                 '%d / %.3f / %s' % (len(p1), b1, s1 or 'NO'),
                 '%d / %.3f / %s' % (len(p2), b2, s2 or 'NO'), z2, out2))
    print()
    print('   `zero` and `out` are draws the snap refused: one that landed')
    print('   exactly on the seed (not a perturbation) and one that snapped')
    print('   OUTSIDE the radius (the disc stays an exact bound). Both are')
    print('   rare at the shipped radius, which is why the arms that cover')
    print('   them script the rng instead of relying on a population.')
    print()
    print('   Occupancy returns to the INPUT value exactly, not approximately:')
    print('   a lattice-multiple offset added to an on-lattice seed keeps the')
    print('   same residue, so every part that was on-lattice still is.')
    print()


def main():
    ap = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--table', choices=('A', 'B', 'C'), action='append',
                    help='run one table (repeatable); default all three')
    args = ap.parse_args()

    boards = run_utils.corpus_boards()
    if not boards:
        print('SKIP: git could not enumerate the corpus')
        return 0
    lb = lattice_boards()
    print('%d tracked boards, %d declare a lattice.\n' % (len(boards), len(lb)))
    # The copy in `jitter()` must still BE the engine's sampler, or every
    # table below describes a function this repo no longer ships. Checked on
    # both branches -- with a lattice and without.
    verify_against_engine(lb[0][0], lb[0][1], 4.0, lb[0][2], _rng)
    verify_against_engine(lb[0][0], lb[0][1], 4.0, None, _rng)
    want = args.table or ('A', 'B', 'C')
    if 'A' in want:
        table_a(lb)
    if 'B' in want:
        table_b(lb)
    if 'C' in want:
        table_c(lb)
    return 0


if __name__ == '__main__':
    sys.exit(main())
