"""whole_ctx.py -- the bench every whole_* tool plans, and the braid's plan of it.

The whole-route tools read one bench from the environment, like the rest of the
chain: BENCH (the board, fanned out: teeth and berths laid), NETS (N1,N2,.. or
@FILE) and DEST (the destination part's reference). Under the chain's own plan
environment -- PLAN_PAGES=1 PLAN_JUDGE=count PLAN_JUDGE_LEN=lane BRAID_PAIRS=1
PLAN_PAIRS=1 BRAID_EXACT_PAGES=0 PLAN_PAGES_SIDERS=2 -- plan() returns the
corridors exactly as braid.run and plan_audit plan them."""
import contextlib
import io
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
os.chdir(HERE)

import plan_audit as pa  # noqa: E402


def bench():
    """(board, nets, dest) from the environment, or a usage stop"""
    miss = [k for k in ('BENCH', 'NETS', 'DEST') if not os.environ.get(k)]
    if miss:
        raise SystemExit(f'set {", ".join(miss)}: the bench board, its nets (N1,N2,.. or @FILE) and the '
                         f'destination part')
    return os.environ['BENCH'], pa.read_nets(os.environ['NETS']), os.environ['DEST']


def stub_dir(ctx, net, pt, layer):
    """the direction of net's stub's LAST segment at its free end pt ON THE LANE'S LAYER, pointing out of the stub (the
    way a lane leaves that end); None when the stub reaches pt on the other layer -- it ends in a via there, and the
    lane lands on the via and continues no line (SDQM1's F remnant at its berth is not its stub)"""
    tw = ctx.cfg.track_width
    nid = ctx.byname[net][0]
    if any(v_.net_id == nid and math.hypot(v_.x - pt[0], v_.y - pt[1]) < tw / 2 for v_ in ctx.base_vias):
        return None
    best = None
    for s_ in ctx.base_segments:
        if s_.net_id != nid or s_.layer != layer:
            continue
        for (ax, ay), (bx_, by_) in (((s_.start_x, s_.start_y), (s_.end_x, s_.end_y)),
                                     ((s_.end_x, s_.end_y), (s_.start_x, s_.start_y))):
            d_ = math.hypot(bx_ - pt[0], by_ - pt[1])
            L_ = math.hypot(bx_ - ax, by_ - ay)
            if L_ > 1e-6 and (best is None or d_ < best[0]):
                best = (d_, ((bx_ - ax) / L_, (by_ - ay) / L_))
    return best[1] if best is not None and best[0] < tw / 2 else None


def plan(quiet=True):
    """(ctx, corridors): the braid's plan of the bench"""
    board, nets, dest = bench()
    if quiet:
        with contextlib.redirect_stdout(io.StringIO()):
            ctx, cs, _logs = pa.plan(board, nets, dest)
    else:
        ctx, cs, _logs = pa.plan(board, nets, dest)
    return ctx, cs
