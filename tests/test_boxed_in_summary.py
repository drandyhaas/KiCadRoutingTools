#!/usr/bin/env python3
"""The static-vs-congestion verdict stops being a sentence.

The router has printed "boxed in by static obstacles (neighboring pads +
clearance), not by congestion" on this failure shape since #95. Run 20 spent
three grid refinements against exactly that sentence -- 0.05 -> 0.025 -> 0.0125,
about 40 minutes, with the same three nets failing at every resolution -- because
a sentence is not something a gate can read. Its sibling
`preexisting_blocker_hint` has been recorded via `record_net_event` and
serialized since #301; this one, the decision the whole retry ladder turns on,
was print-only.

THE INVARIANT THIS FILE EXISTS FOR: `summary['boxed_in']` is a SEPARATE key that
adds NOTHING to `summary['blockers']`. The routing skill's classifier row is
"`blockers` empty; the log says boxed in", so a verdict that leaked into
`blockers` would silently turn every box-in into a congestion finding -- breaking
the very clause being fixed. The row becomes "`blockers` empty AND `boxed_in`
names the net": one JSON test, no regex.

Note the invariant is NOT "`blockers` is empty on this fixture". It is not: the
frontier analysis legitimately attributes the static WALL, because since e2ffa29
pre-existing copper is rip-candidate and therefore attributable. The two keys
answer different questions -- WHICH copper is in the way, versus whether anything
RIPPABLE is in the way at all -- and the test asserts they keep their own schemas.

    python3 tests/test_boxed_in_summary.py
"""
import io
import json
import os
import re
import sys
from contextlib import redirect_stdout

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
_R = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_R, os.path.join(_R, 'py_router'), os.path.join(_R, 'py_tools')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from kicad_parser import BoardInfo                                # noqa: E402
from synth import make_net, make_pad, make_pcb, make_seg          # noqa: E402

X, WALL = 1, 2
CHECKS = []


def check(name, ok, detail=''):
    CHECKS.append((name, ok))
    print(f"  {'PASS' if ok else 'FAIL'}: {name}" + (f' -- {detail}' if not ok else ''))


def _board():
    """X's source pad sealed inside a box of STATIC copper.

    Static on purpose: the wall is pre-existing and out of scope, and with
    `max_rip_up_count=0` nothing is rippable at all. That is the exact
    signature -- a search that dies in almost no iterations with no rippable
    blockers -- and it is a geometry fact, not a congestion one.
    """
    bi = BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'},
                   copper_layers=['F.Cu', 'B.Cu'],
                   board_bounds=(0.0, 0.0, 10.0, 10.0))
    pads = {X: [make_pad(X, 2.0, 5.0, ref='U1', num='1', net_name='X',
                         size_x=0.3, size_y=0.3),
                make_pad(X, 8.0, 5.0, ref='U2', num='1', net_name='X',
                         size_x=0.3, size_y=0.3)],
            WALL: []}
    # A sealed box around the source pad, on BOTH layers so a via cannot
    # escape either.
    segs = []
    for lay in ('F.Cu', 'B.Cu'):
        segs += [make_seg(1.4, 4.4, 2.6, 4.4, net_id=WALL, width=0.4, layer=lay),
                 make_seg(1.4, 5.6, 2.6, 5.6, net_id=WALL, width=0.4, layer=lay),
                 make_seg(1.4, 4.4, 1.4, 5.6, net_id=WALL, width=0.4, layer=lay),
                 make_seg(2.6, 4.4, 2.6, 5.6, net_id=WALL, width=0.4, layer=lay)]
    return make_pcb(nets={X: make_net(X, 'X'), WALL: make_net(WALL, 'WALL')},
                    segments=segs, pads_by_net=pads, board_info=bi)


def main():
    from route import batch_route
    buf = io.StringIO()
    with redirect_stdout(buf):
        _ok, _fail, _t, results_data = batch_route(
                    'synthetic', '', ['X'], layers=['F.Cu', 'B.Cu'],
                    clearance=0.2, track_width=0.2, via_size=0.5,
                    via_drill=0.3, grid_step=0.1,
                    ordering_strategy='original', final_reconcile=False,
                    max_rip_up_count=0, return_results=True, pcb_data=_board())
    out = buf.getvalue()
    m = re.findall(r'JSON_SUMMARY: (\{.*\})', out)
    check('JSON_SUMMARY present', bool(m))
    summary = json.loads(m[-1]) if m else {}

    check('X failed to route (the fixture is a cage, not a routable board)',
          'X' in (summary.get('failed_single') or []),
          str(summary.get('failed_single')))

    boxed = summary.get('boxed_in')
    if not isinstance(boxed, list):
        check("summary carries a 'boxed_in' key", False,
              f'{boxed!r} -- the hint printed: '
              + ('yes' if 'boxed in by static obstacles' in out else 'NO, so '
                 'this fixture no longer reproduces the signature and the test '
                 'is measuring nothing'))
    else:
        check("summary carries a 'boxed_in' key", True)
        check('it names the failing net',
              [e.get('net') for e in boxed] == ['X'], str(boxed))
        e = boxed[0]
        check("the verdict is 'boxed_in_static'",
              e.get('verdict') == 'boxed_in_static', str(e))
        check('it carries the iteration count the decision was made on',
              isinstance(e.get('iterations'), int) and e['iterations'] < 20000,
              str(e.get('iterations')))
        g = e.get('geometry') or {}
        check('and the geometry that was IN FORCE, so a guard can ask whether '
              'it is at the board floor',
              abs((g.get('grid_step') or 0) - 0.1) < 1e-9
              and abs((g.get('clearance') or 0) - 0.2) < 1e-9
              and abs((g.get('track_width') or 0) - 0.2) < 1e-9, str(g))

    # THE INVARIANT, stated correctly. It is NOT "`blockers` is empty" -- on
    # this fixture the frontier analysis legitimately attributes the WALL, and
    # since e2ffa29 pre-existing copper is rip-candidate and so attributable.
    # The invariant is that `boxed_in` is a SEPARATE key that adds nothing to
    # `blockers`: the skill's classifier row reads both, and a verdict that
    # leaked into `blockers` would silently turn every box-in into a
    # congestion finding -- breaking the clause this key exists to make
    # readable.
    _bl = summary.get('blockers') or []
    check("no `blockers` entry carries a boxed-in verdict",
          all(set(b) <= {'net', 'stage', 'blocked_by'} for b in _bl),
          f'{[sorted(b) for b in _bl]} -- `blockers` entries must keep exactly '
          f'their #409/#301 schema')
    check('and no `boxed_in` entry carries blocker attribution',
          all('blocked_by' not in e for e in (boxed if isinstance(boxed, list)
                                              else [])),
          str(boxed) + ' -- the two answer different questions: WHICH copper '
          'is in the way, versus whether anything rippable is in the way at all')

    # GUI parity: the plugin reads results_data, never the printed summary.
    check('results_data carries the same boxed_in (GUI parity)',
          results_data.get('boxed_in') == summary.get('boxed_in'),
          f'{results_data.get("boxed_in")!r} vs {summary.get("boxed_in")!r}')

    # And the prose is still printed, because a human reading a log is still a
    # consumer -- the key is additive, not a replacement.
    check('the human-readable hint is still printed',
          'boxed in by static obstacles' in out, out[-400:])

    bad = [n for n, ok in CHECKS if not ok]
    print(f"\n{len(CHECKS) - len(bad)} passed, {len(bad)} failed")
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
