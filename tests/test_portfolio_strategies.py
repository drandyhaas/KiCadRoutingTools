#!/usr/bin/env python3
"""Unit tests for the portfolio perturbation strategies, in-process.

The invariants each strategy must hold: jitter emits only candidate_valid
poses within its radius; the poses strategy never emits a rotation that
RAISES the part's forced-crossing floor (pair_order.ref_inversions is a lower
bound a router cannot beat, so raising it is provably worse); swaps exchange
only FREE, mutually-legal members; and free_refs honors both lock sources
(board `(locked yes)` and --lock globs)."""
import math
import os
import random
import shutil
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for p in (REPO,):
    if p not in sys.path:
        sys.path.insert(0, p)

        sys.path.insert(0, os.path.join(p, 'py_placer'))  # placement split
        sys.path.insert(0, os.path.join(p, 'py_router'))  # placement split
        sys.path.insert(0, os.path.join(p, 'py_tools'))  # placement split
BOARD = os.path.join(REPO, "kicad_files", "splitflap_driver.kicad_pcb")

passed = failed = 0


def check(name, ok, detail=""):
    global passed, failed
    passed += bool(ok)
    failed += not ok
    print(f"  {'OK  ' if ok else 'FAIL'} {name}{(' -- ' + detail) if detail else ''}")


if not os.path.isfile(BOARD):
    print("SKIP: fixture missing")
    sys.exit(0)

from kicad_parser import parse_kicad_pcb
from placement import portfolio
from placement.pair_order import ref_inversions

pcb = parse_kicad_pcb(BOARD)
free = portfolio.free_refs(pcb, BOARD, None)
state = portfolio.make_oracle(pcb, BOARD, free=free, clearance=0.25,
                              board_edge_clearance=0.55, grid_step=0.1,
                              ignore_ids=set())
origin = {r: (state.parts[r].x, state.parts[r].y, state.parts[r].rot)
          for r in free}


def restore():
    for r in sorted(origin):
        x, y, rot = origin[r]
        p = state.parts[r]
        if (p.x, p.y, p.rot) != (x, y, rot):
            state.apply_move(r, x, y, rot)


# ---- jitter ----------------------------------------------------------------
RADIUS = 3.0
poses = portfolio.perturb_jitter(state, free, random.Random("t"), RADIUS)
check("jitter produced poses", len(poses) > 0, f"{len(poses)}")
check("jitter refs are free refs",
      all(p['reference'] in set(free) for p in poses))
ok_radius = all(
    math.hypot(p['new_x'] - origin[p['reference']][0],
               p['new_y'] - origin[p['reference']][1]) <= RADIUS + 1e-6
    for p in poses)
check("jitter displacements within radius", ok_radius)
# each emitted pose is individually legal against the FINAL perturbed state
# minus itself (the sequential-validation contract)
all_valid = all(
    state.candidate_valid(p['reference'], p['new_x'], p['new_y'],
                          p['new_rotation'])
    for p in poses)
check("every jitter pose legal in the perturbed state", all_valid)
restore()
poses2 = portfolio.perturb_jitter(state, free, random.Random("t"), RADIUS)
restore()
check("jitter deterministic for a fixed rng seed", poses == poses2)
poses3 = portfolio.perturb_jitter(state, free, random.Random("u"), RADIUS)
restore()
check("different rng seed produces different jitter", poses != poses3)

# #826: the DEFAULT is continuous, and that is a contract rather than an
# accident. `placement/perturb.py`'s `scatter` damage kind calls this function
# with four positional arguments and no lattice, and its own comment calls it
# "the POSITIVE CONTROL ... the arm that MUST recover". A snapped offset would
# land the part exactly on the coset the quench generates from -- one nudge
# reaches it -- which makes the control easier to pass, the one direction a
# positive control must not move.
#
# Every other jitter arm above (radius, freeness, legality, determinism) stays
# green if the default silently became "snap to the inferred lattice", so
# without this check nothing in the repo detects that change.
from placement.board_grid import infer_board_grid            # noqa: E402
_lat = infer_board_grid(pcb)['step']
check("the board under test declares a lattice, so this check can fail",
      _lat is not None, f"{_lat}")
check("without an explicit lattice the sampler stays CONTINUOUS",
      any(abs((p['new_x'] - origin[p['reference']][0]) / _lat
              - round((p['new_x'] - origin[p['reference']][0]) / _lat)) * _lat
          > 1e-6 for p in poses),
      f"lattice {_lat}, {len(poses)} poses")

# ---- poses -----------------------------------------------------------------
n_variants = 0
while True:
    result = portfolio.perturb_poses(state, free, n_variants)
    if result is None:
        restore()
        break
    (p,) = result
    ref = p['reference']
    restore()
    before = ref_inversions(state, ref)
    after = ref_inversions(state, ref, p['new_x'], p['new_y'],
                           p['new_rotation'])
    if after > before:
        check(f"poses variant {n_variants} ({ref}) does not raise inversions",
              False, f"{before} -> {after}")
    rot_delta = (p['new_rotation'] - origin[ref][2]) % 360
    if rot_delta not in (90.0, 180.0, 270.0):
        check(f"poses variant {n_variants} is a rotation change",
              False, f"delta {rot_delta}")
    n_variants += 1
check("poses enumerated at least one variant", n_variants > 0,
      f"{n_variants}")
check("variant index past the end returns None",
      portfolio.perturb_poses(state, free, n_variants) is None)
restore()

# ---- swaps -----------------------------------------------------------------
# synthesize a block from same-side free parts; include a locked ref to prove
# the free-member filter, not just hope for it
same_side = [r for r in free if state.parts[r].side == 'F'][:6]
locked_ref = next(r for r in sorted(state.parts) if r not in free) \
    if any(r not in free for r in state.parts) else None
block = {'blk': list(same_side) + ([locked_ref] if locked_ref else [])}
sw = portfolio.perturb_swaps(state, block, random.Random("s"), 2)
check("swap produced an exchange", len(sw) >= 2, f"{len(sw)}")
check("swap never moved a locked ref",
      all(p['reference'] in set(free) for p in sw))
by_ref = {p['reference']: p for p in sw}
# positions are exchanges: the multiset of (x, y) before == after
before_xy = sorted((origin[r][0], origin[r][1]) for r in by_ref)
after_xy = sorted((p['new_x'], p['new_y']) for p in sw)
check("swap positions are an exchange of existing positions",
      before_xy == after_xy)
restore()
check("swap with no blocks is barren",
      portfolio.perturb_swaps(state, {}, random.Random("s"), 2) == [])

# ---- free_refs lock sources ------------------------------------------------
sub = portfolio.free_refs(pcb, BOARD, ['C*'])
check("--lock glob excludes matching refs",
      not any(r.startswith('C') for r in sub) and len(sub) < len(free))
with tempfile.TemporaryDirectory() as d:
    locked_board = os.path.join(d, 'locked.kicad_pcb')
    shutil.copyfile(BOARD, locked_board)
    victim = free[0]
    from placement.seeder import stamp_locked
    n = stamp_locked(locked_board, [victim])
    check("stamp_locked stamped one part", n == 1, f"{n}")
    pcb2 = parse_kicad_pcb(locked_board)
    free2 = portfolio.free_refs(pcb2, locked_board, None)
    check("board (locked yes) excludes the ref from free_refs",
          victim not in free2 and len(free2) == len(free) - 1)

print(f"\n{passed}/{passed + failed} checks passed")
sys.exit(1 if failed else 0)
