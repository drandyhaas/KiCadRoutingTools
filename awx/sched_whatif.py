#!/usr/bin/env python3
"""What the schedule would cost under a different TARGET order, with
no geometry: divers, the LIS floor, the columns (gate 'last'), and the
two-page count -- the nets that fit neither of two crossing-free
layers (Greene: the largest union of two increasing subsequences), i.e.
what a constant-layer corridor would still have to via.
usage: sched_whatif.py FANOUT_BOARD K [--exit-block NET,NET]
  --exit-block: move these nets out of the head-on exits into the
  side-exit block, in the block's own join order (what the plan would
  do if it sent them to the side face)."""
import argparse
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
import braid as te  # noqa: E402
from schedule import Schedule, lis_keep  # noqa: E402


def two_page(ranks):
    """Largest union of two increasing subsequences (RSK: the first
    two rows of the insertion tableau)."""
    rows = []
    for r in ranks:
        x = r
        for row in rows:
            # bump the smallest element greater than x
            k = next((i for i, v in enumerate(row) if v > x), None)
            if k is None:
                row.append(x)
                x = None
                break
            row[k], x = x, row[k]
        if x is not None:
            rows.append([x])
    return sum(len(r) for r in rows[:2])


def report(tag, launch, target, tooth_layer, dest_layer=None):
    sched = Schedule(launch, target, tooth_layer, dest_layer=dest_layer)
    ranks = [sched.trank[n] for n in launch]
    lis = len(lis_keep(ranks))
    cols = sched.columns({d: 1 for d in sched.divers}, {d: 0 for d in sched.divers},
                         gate='last')
    swaps = sum(len(c) for c in cols)
    tp = two_page(ranks)
    print(f'{tag}: {len(launch)} nets, LIS {lis} -> floor {2 * (len(launch) - lis)} '
          f'({len(sched.divers)} divers), {swaps} swaps in {len(cols)} columns; '
          f'two-page: {tp} fit, {len(launch) - tp} would still via')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('board')
    ap.add_argument('k', type=int)
    ap.add_argument('--exit-block', default='')
    ap.add_argument('--dest', default='DU1')
    a = ap.parse_args()
    nets = subprocess.run([sys.executable, os.path.join(HERE, 'coherent_nets.py'),
                           str(a.k)], capture_output=True, text=True).stdout.strip()
    names = [n for n in nets.split(',') if n]
    quiet = lambda *x: None  # noqa: E731
    ctx, groups = te.setup(a.board, names, a.dest, quiet)
    c = te.Corridor(0, groups[0], ctx, quiet)
    c.build_spine()
    c.classify()
    c.offsets(0.35)
    c.reserve_intervals()
    report('as planned', c.launch, c.target, ctx.tooth_layer, ctx.dest_layer)
    move = [n for n in a.exit_block.split(',') if n]
    if not move:
        return
    # the side-exit block's order: ports (head-on launched) by exit s,
    # then the joined by join order (farthest-out joiner exits last);
    # the moved nets are joiners, so they slot by their tooth s
    target = [n for n in c.target if n not in move]
    block = [n for n in target if n in c.exit_block]
    joined = [n for n in block if n in c.join_block] + move
    joined.sort(key=lambda n: -c.st[n][0])
    ports = [n for n in block if n not in c.join_block]
    new_block = ports + joined
    head = [n for n in target if n not in c.exit_block]
    # the block sits beyond the head-on exits on its side; keep the
    # head-on order, then the block
    sg = 1 if any(c.exit_side[n] > 0 for n in block) else -1
    target = head + new_block if sg > 0 else new_block[::-1] + head
    report(f'{",".join(move)} in the exit block', c.launch, target,
           ctx.tooth_layer, ctx.dest_layer)


if __name__ == '__main__':
    main()
