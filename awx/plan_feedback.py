"""plan_feedback.py -- the ROUTE's verdict handed back to the PLAN (#622,
2026-09-18; the consumer is plan_loop.py).

The plan-level loop puts the routing INSIDE the planning iteration: solve,
fan out, braid both arms, grade -- and the re-braid's verdict goes back
into the next pages-first re-solve. This module is the channel. It reads
ONE env variable, `PLAN_LOOP_FEEDBACK` -- a JSON file path, or the JSON
itself when the value starts with `{` (so a cloud arm can carry it in its
env) -- of the shape

    {"bans":   {net: {"dst": [[direction, layer, climb_lo, climb_hi], ...],
                      "src": [...]}},
     "prices": {net: {"dst": {"direction/layer": via_units, ...},
                      "src": {...}}},
     "hint":   {net: {"dst": <move signature>, "src": <move signature|null>}}}

and applies it in three places, each byte-identical when the variable is
unset:

  * BANS -- `fanout_from_plan.plan_state` drops every menu move of a
    banned CLASS (face + layer, optionally a climb window) at that end.
    Class-level on purpose: the 0918 re-braid arm banned the exact
    signature of a rejected move and the search re-proposed the SAME idea
    at the next gap along, eight rounds running. A destination menu is
    never emptied (the ban is skipped and said so); a source menu may be,
    because the tooth as it stands is always a candidate.
  * PRICES -- `pages_first._solve` adds `via_units * VIA_W` to the cost of
    every candidate of the priced class. These are the RESIDUALS the route
    measured: real vias minus the plan's own prediction, per net, split
    between its two ends -- "this net, in this class at this end, costs
    more (or less) than the model thinks". A per-net constant would steer
    nothing inside the solve (replan's `plan_ends.RESIDUAL` is one), so the
    price is per class.
  * HINT -- the incumbent plan's berth and tooth per net, matched to the
    menus by signature, as the CP-SAT's solution hint (in place of the
    greedy seed's). The solve then starts AT the best routed plan and can
    only improve on it in the model's own terms.

Nothing here reads a face, a ref or a board name.
"""
from __future__ import annotations

import json
import os
import sys
from typing import Dict, List, Optional

_ENV = 'PLAN_LOOP_FEEDBACK'


def _load() -> Dict:
    v = os.environ.get(_ENV, '')
    if not v:
        return {}
    if v.lstrip().startswith('{'):
        d = json.loads(v)
    else:
        with open(v, encoding='utf-8') as f:
            d = json.load(f)
    if not isinstance(d, dict):
        raise SystemExit(f'{_ENV}: expected a JSON object, got {type(d).__name__}')
    return d


FEEDBACK: Dict = _load()
BANS: Dict = FEEDBACK.get('bans') or {}
PRICES: Dict = FEEDBACK.get('prices') or {}
HINT: Dict = FEEDBACK.get('hint') or {}
# HOLD -- {net: {'dst': sig|null, 'dst_cls': [direction, layer], 'src': null}}:
# on the FREE first solve a held net keeps the incumbent's berth (the
# signature, else its class) and its tooth as it stands; a net absent
# here, or without the key for an end, is FREE at that end. RADIUS -- the
# ablation: no holds, at most this many ends off the hinted plan (the
# walk's trust region). The two are never combined by the loop.
HOLD: Dict = FEEDBACK.get('hold') or {}
RADIUS: int = int(FEEDBACK.get('radius') or 0)
# ACCEPT_LAID -- the fanout's realize-and-confirm keeps a realized source
# round whenever an asked tooth was laid IN CLASS, instead of asking its
# own count judge (the one measured anti-correlated at K51, and the one
# that reverted a K15 arm's forced moves 46 -> 47). Under the loop the
# ROUTE judges the board; the confirm's job is only to lay the hypothesis.
ACCEPT_LAID: bool = bool(FEEDBACK.get('accept_laid'))
# REACH -- {net: {'dst': [lo_net|null, hi_net|null], 'src': [...]}}: a free end
# may land only between the keys of these two nets (the held neighbours k
# ranks away in the incumbent's plan order, plan_loop --reach). Locality in
# ORDER space, which is what the F-block law says costs: measured on the
# K51 loop's first hold round, a freed berth sent from target rank 1 to 32
# crossed every lane between, and 23 vias landed on held nets whose pages
# and ranks had not changed at all -- pure realization (rips, last calls).
REACH: Dict = FEEDBACK.get('reach') or {}
# SEED -- the CP-SAT random seed for this plan (a jump: another feasible
# stopping point of the same instance). 0 = the solver's default.
SEED: int = int(FEEDBACK.get('seed') or 0)
ACTIVE = bool(BANS or PRICES or HINT or HOLD or RADIUS or ACCEPT_LAID or REACH or SEED)
SOURCE = os.environ.get(_ENV, '')
if SOURCE.lstrip().startswith('{'):
    SOURCE = '<inline>'


def cls(m) -> str:
    """The class of a move, as the feedback keys it: face/layer."""
    return f'{m.direction}/{m.layer}'


def sig_key(sig):
    """A move signature (source_realize.move_sig, nested tuples) in the
    form JSON round-trips it to, so a signature written by the loop and
    one computed here compare equal."""
    return json.loads(json.dumps(sig))


def banned(net: str, end: str, m) -> bool:
    for b in (BANS.get(net) or {}).get(end) or ():
        d, L = b[0], b[1]
        if m.direction != d or m.layer != L:
            continue
        lo = b[2] if len(b) > 2 else None
        hi = b[3] if len(b) > 3 else None
        c = int(getattr(m, 'climb', 0) or 0)
        if lo is not None and c < lo:
            continue
        if hi is not None and c > hi:
            continue
        return True
    return False


# what the filter did, for the one summary line the fanout stage prints
_NOTES: List[str] = []
_DROPPED = {'dst': 0, 'src': 0}


def filter_menu(net: str, end: str, moves: list) -> list:
    """The menu without its banned classes. A DESTINATION menu that the
    bans would empty is kept whole (a ball must leave somehow) and the
    skip is recorded; a SOURCE menu may empty -- the tooth as it stands
    stays a candidate regardless."""
    if not BANS or net not in BANS or not (BANS[net] or {}).get(end):
        return moves
    keep = [m for m in moves if not banned(net, end, m)]
    if end == 'dst' and moves and not keep:
        _NOTES.append(f'{net}.{end}: ban would empty the menu -- kept')
        return moves
    _DROPPED[end] += len(moves) - len(keep)
    return keep


def price(net: str, end: str, m) -> float:
    """The feedback price of a candidate, in via units (0 = none)."""
    p = (PRICES.get(net) or {}).get(end)
    if not p:
        return 0.0
    return float(p.get(cls(m), 0.0) or 0.0)


def hint_move(net: str, end: str, moves: list):
    """The menu move the hint names for this net and end, or None."""
    want = (HINT.get(net) or {}).get(end)
    if want is None:
        return None
    wk = sig_key(want)
    from source_realize import move_sig
    for m in moves:
        if sig_key(move_sig(m)) == wk:
            return m
    return None


_HOLD_NOTES: List[str] = []


def hold_dst(net: str, moves: list) -> list:
    """The held destination menu of `net`: the one move whose signature the
    hold names, else every move of the held class, else the menu as it is
    (recorded: a hold that cannot be honoured is a free net, said so)."""
    h = HOLD.get(net) or {}
    if 'dst' not in h and 'dst_cls' not in h:
        return moves
    from source_realize import move_sig
    if h.get('dst') is not None:
        wk = sig_key(h['dst'])
        hit = [m for m in moves if sig_key(move_sig(m)) == wk]
        if hit:
            return hit[:1]
    c = h.get('dst_cls')
    if c:
        keep = [m for m in moves if m.direction == c[0] and m.layer == c[1]]
        if keep:
            return keep
    _HOLD_NOTES.append(f'{net}: held berth not on the menu -- free')
    return moves


def held_src(net: str) -> bool:
    """Is the tooth of `net` held -- as it stands (null), or at a named move?"""
    return 'src' in (HOLD.get(net) or {})


def held_src_move(net: str, moves: list):
    """The menu move a source hold NAMES for `net` (a crossover holds a
    tooth at the other parent's move), or None when it holds the tooth
    as it stands or names a move not on this menu."""
    want = (HOLD.get(net) or {}).get('src')
    if want is None:
        return None
    wk = sig_key(want)
    from source_realize import move_sig
    for m in moves:
        if sig_key(move_sig(m)) == wk:
            return m
    return None


def reach_window(net: str, end: str):
    """(lo_net, hi_net) bounding a free end of `net`, or None."""
    w = (REACH.get(net) or {}).get(end)
    if not w or (w[0] is None and w[1] is None):
        return None
    return (w[0], w[1])


def summary() -> str:
    """One line for the log: what the feedback carries and what the menu
    filter did so far (call after plan_state has run)."""
    if not ACTIVE:
        return ''
    nb = sum(len(v or ()) for d in BANS.values() for v in (d or {}).values())
    npr = sum(len(v or {}) for d in PRICES.values() for v in (d or {}).values())
    nh = sum(1 for d in HINT.values() if d and d.get('dst') is not None)
    nhold = sum(1 for d in HOLD.values() if d and ('dst' in d or 'dst_cls' in d or 'src' in d))
    s = (f'plan feedback [{SOURCE}]: {nb} class ban(s) over {len(BANS)} net(s), '
         f'{npr} priced class(es) over {len(PRICES)} net(s), {nh} hinted berth(s), '
         f'{nhold} held net(s), radius {RADIUS}; '
         f'menu moves dropped: berths {_DROPPED["dst"]}, teeth {_DROPPED["src"]}')
    if _NOTES or _HOLD_NOTES:
        s += '; ' + '; '.join((_NOTES + _HOLD_NOTES)[:8])
    return s


def _self_test():
    """The channel checked on fake moves: a ban filters its class and its
    climb window only, a destination menu is never emptied, a price reads
    per class, and a hint written by the loop (JSON) matches the menu move
    it named. Run: python3 plan_feedback.py --self-test"""
    global BANS, PRICES, HINT
    from escape_moves import Move
    from source_realize import move_sig
    mk = lambda d, L, x, c=0, k='dogbone': Move('N', k, d, L, (x, 1.0), 1, legs=[((x, 0.0), (x, 1.0), L)],
                                              site=(x, 0.5), climb=c)
    menu = [mk('down', 'B.Cu', 1.0), mk('down', 'B.Cu', 2.0, c=3), mk('down', 'F.Cu', 3.0),
            mk('right', 'B.Cu', 4.0, c=1)]
    keep = (BANS, PRICES, HINT)
    try:
        BANS = {'N': {'dst': [['down', 'B.Cu', None, None]], 'src': [['down', 'B.Cu', 2, 4]]}}
        out = filter_menu('N', 'dst', menu)
        assert [m.exit_pt[0] for m in out] == [3.0, 4.0], out
        out = filter_menu('N', 'src', menu)                  # the window: climb 3 only
        assert [m.exit_pt[0] for m in out] == [1.0, 3.0, 4.0], out
        every = [['down', 'B.Cu', None, None], ['down', 'F.Cu', None, None],
                 ['right', 'B.Cu', None, None]]
        BANS = {'N': {'dst': every, 'src': every}}
        assert filter_menu('N', 'dst', menu) is menu, 'a destination menu must never be emptied'
        assert filter_menu('N', 'src', menu) == [], 'a source menu may be'
        assert filter_menu('M', 'dst', menu) is menu, 'an unbanned net keeps its menu object'
        PRICES = {'N': {'dst': {'down/B.Cu': 1.5}, 'src': {'right/B.Cu': -0.5}}}
        assert price('N', 'dst', menu[0]) == 1.5 and price('N', 'dst', menu[2]) == 0.0
        assert price('N', 'src', menu[3]) == -0.5 and price('M', 'src', menu[3]) == 0.0
        HINT = json.loads(json.dumps({'N': {'dst': move_sig(menu[1]), 'src': None}}))
        assert hint_move('N', 'dst', menu) is menu[1]
        assert hint_move('N', 'src', menu) is None
        assert sig_key(move_sig(menu[1])) == HINT['N']['dst']
        global HOLD
        keep_h = HOLD
        HOLD = json.loads(json.dumps({'N': {'dst': move_sig(menu[2]), 'dst_cls': ['down', 'F.Cu'], 'src': None},
                                      'P': {'dst': None, 'dst_cls': ['down', 'B.Cu']},
                                      'Q': {'dst': None, 'dst_cls': ['left', 'F.Cu'], 'src': None}}))
        assert hold_dst('N', menu) == [menu[2]], 'the signature hold is a one-move menu'
        assert [m.exit_pt[0] for m in hold_dst('P', menu)] == [1.0, 2.0], 'the class hold keeps its class'
        assert hold_dst('Q', menu) is menu and _HOLD_NOTES, 'an unhonourable hold frees the net, and says so'
        assert hold_dst('M', menu) is menu and held_src('N') and not held_src('P') and not held_src('M')
        HOLD = keep_h
    finally:
        BANS, PRICES, HINT = keep
    print('plan_feedback self-test: OK')


if __name__ == '__main__':
    if '--self-test' in sys.argv:
        _self_test()
    else:
        print(summary() or f'{_ENV} unset: no feedback')
