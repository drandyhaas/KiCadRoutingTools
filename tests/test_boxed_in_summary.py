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
the very clause being fixed. The row can then read "`blockers` empty AND
`boxed_in` names the net": one JSON test, no regex.

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


def case_static_cage():
    """First-pass failure: the single_ended_loop records the verdict."""
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


A, B = 3, 4   # WALL is 2 above; a locked wall keeps the same net id here


def _rip_victim_board():
    """Two in-scope nets share a one-track gap in a LOCKED wall.

    A and B start inside a box of KiCad-locked copper (never rippable, no
    override) whose only exit is a gap on F.Cu wide enough for one track;
    B.Cu is sealed. A routes first and takes the gap. B's only way out is
    through A, so B rips A and routes. A then comes back through the REROUTE
    queue, finds the gap full of B -- which its rip ancestry forbids it to
    rip -- and the locked wall everywhere else: a search that dies in a few
    thousand iterations with nothing rippable, i.e. the boxed-in signature,
    reached from `reroute_loop` rather than the first-pass loop. (Plain
    pre-existing copper would not do: since e2ffa29 it is a rip candidate,
    and with max_rip_up_count>0 the router rips the wall itself.)
    """
    bi = BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'},
                   copper_layers=['F.Cu', 'B.Cu'],
                   board_bounds=(0.0, 0.0, 10.0, 10.0))
    pads = {A: [make_pad(A, 1.5, 4.6, ref='U1', num='1', net_name='A',
                         size_x=0.3, size_y=0.3),
                make_pad(A, 8.0, 4.6, ref='U2', num='1', net_name='A',
                         size_x=0.3, size_y=0.3)],
            B: [make_pad(B, 1.5, 5.4, ref='U1', num='2', net_name='B',
                         size_x=0.3, size_y=0.3),
                make_pad(B, 8.0, 5.4, ref='U2', num='2', net_name='B',
                         size_x=0.3, size_y=0.3)],
            WALL: []}
    segs = []
    for lay in ('F.Cu', 'B.Cu'):
        segs += [make_seg(0.6, 3.6, 3.4, 3.6, net_id=WALL, width=0.4,
                          layer=lay, locked=True),
                 make_seg(0.6, 6.4, 3.4, 6.4, net_id=WALL, width=0.4,
                          layer=lay, locked=True),
                 make_seg(0.6, 3.6, 0.6, 6.4, net_id=WALL, width=0.4,
                          layer=lay, locked=True)]
    # Right wall: a one-track gap (copper edges 4.6..5.4) on F.Cu; sealed on
    # B.Cu so a via cannot escape either.
    segs += [make_seg(3.4, 3.6, 3.4, 4.4, net_id=WALL, width=0.4,
                      layer='F.Cu', locked=True),
             make_seg(3.4, 5.6, 3.4, 6.4, net_id=WALL, width=0.4,
                      layer='F.Cu', locked=True),
             make_seg(3.4, 3.6, 3.4, 6.4, net_id=WALL, width=0.4,
                      layer='B.Cu', locked=True)]
    return make_pcb(nets={A: make_net(A, 'A'), B: make_net(B, 'B'),
                          WALL: make_net(WALL, 'WALL')},
                    segments=segs, pads_by_net=pads, board_info=bi)


def case_rip_victim_reroute():
    """Reroute-path failure: `reroute_loop` records the verdict too.

    The recording call in reroute_loop.py sits on the rip-victim reroute path,
    which the first-pass cage above never reaches (its single net fails before
    anything is ripped). This fixture was run under `coverage` while writing
    the test: the reroute_loop call executed and the single_ended_loop one did
    not. Without a coverage run the test still pins the path causally: A
    ROUTED on its first pass (it is not in the failed set until after B ripped
    it), so the only place A can have acquired a boxed-in verdict is its
    reroute.

    RE-RECORDED 2026-08-30: which net ends up broken FLIPPED at `31004f43`
    (#622, victim-priority restore ON by default), and the flip is the feature
    working. A rip is a trade; when the victim's reroute fails and its restore
    is short-refused because the ripper now holds the channel, that trade has
    silently failed. The restore gives A its channel back and REQUEUES B -- "a
    re-run of the exchange, not a reversal" -- so B's reroute is the one that
    now fails. Bisected to that commit, and `KICAD_VICTIM_RESTORE=0` reproduces
    the old expectation exactly (routed=[B] failed=[A] boxed_in=[A]), which is
    what identifies the cause rather than merely dating it.

    The causal pin SURVIVES the flip, which is why re-recording is honest here
    rather than a rebaseline: B also had a SUCCESSFUL result before its verdict
    (`RETRY SUCCESS` right after it ripped A), so B's boxed-in verdict can only
    have come from its later `[REROUTE 4/4]` -- still the reroute_loop call
    this fixture exists to reach, still not the first-pass one. Both nets in
    fact record a reroute verdict here; A's is absent from the summary because
    A ends up ROUTED, which is the restore doing its job.
    """
    from route import batch_route
    buf = io.StringIO()
    with redirect_stdout(buf):
        _ok, _fail, _t, results_data = batch_route(
                    'synthetic', '', ['A', 'B'], layers=['F.Cu', 'B.Cu'],
                    clearance=0.2, track_width=0.2, via_size=0.5,
                    via_drill=0.3, grid_step=0.1,
                    ordering_strategy='original', final_reconcile=False,
                    max_rip_up_count=1, return_results=True,
                    pcb_data=_rip_victim_board())
    out = buf.getvalue()
    m = re.findall(r'JSON_SUMMARY: (\{.*\})', out)
    check('JSON_SUMMARY present (rip victim)', bool(m))
    summary = json.loads(m[-1]) if m else {}

    i_rip = out.find('Ripping up A')
    i_rer = out.find('Re-routing ripped net A')
    i_box = out.find('no rippable blockers found')
    check('B ripped A and A was re-queued (the fixture exercises the reroute '
          'path, not the first-pass loop)',
          0 <= i_rip < i_rer < i_box,
          f'rip@{i_rip} reroute@{i_rer} boxed@{i_box}')
    check('the victim A was restored and the ripper B requeued, so B is the '
          'net left broken (#622 victim-priority restore, default since '
          '31004f43)',
          summary.get('routed_single') == ['A']
          and summary.get('open_single') == ['B']
          and summary.get('failed_single') == []
          and summary.get('successful') == 1,
          f"routed={summary.get('routed_single')} "
          f"failed={summary.get('failed_single')} "
          f"open={summary.get('open_single')} "
          f"successful={summary.get('successful')}")
    boxed = summary.get('boxed_in')
    check("summary carries 'boxed_in' for the requeued ripper",
          isinstance(boxed, list) and [e.get('net') for e in boxed] == ['B'],
          f'{boxed!r}; hint printed: '
          + str('boxed in by static obstacles' in out))
    if isinstance(boxed, list) and boxed:
        e = boxed[0]
        check("the requeued net's verdict is 'boxed_in_static' with an iteration "
              "count below the static threshold",
              e.get('verdict') == 'boxed_in_static'
              and isinstance(e.get('iterations'), int)
              and e['iterations'] < 20000, str(e))
    # SCHEMA invariant -- the one the file exists for, and it still holds:
    # the two keys never bleed into each other's shape.
    _boxed_nets = [e.get('net') for e in (boxed or [])]
    check('no `blockers` entry carries a boxed-in verdict (rip victim)',
          not [b for b in (summary.get('blockers') or [])
               if 'verdict' in b], str(summary.get('blockers')))
    check('and no `boxed_in` entry carries blocker attribution (rip victim)',
          not [e for e in (boxed or []) if 'blocked_by' in e], str(boxed))

    # OPEN FINDING, asserted so it cannot go quiet. Under the pre-31004f43
    # default the boxed net (A) had never ripped anything, so it appeared in
    # `boxed_in` ONLY and the skill's classifier row -- "`blockers` empty; the
    # log says boxed in by static obstacles -> parameters, placement is not the
    # lever" (plan-pcb-routing SKILL.md:2486) -- discriminated cleanly.
    #
    # Since victim-priority restore, the net left broken is the REQUEUED RIPPER,
    # which carries a blockers entry from the first attempt it ripped on AND a
    # boxed_in verdict from its later reroute. Both are true histories, but the
    # row's `blockers`-empty precondition now fails for a net the router itself
    # declared statically boxed in -- so an operator following the table is sent
    # to a congestion/placement remedy instead of the parameters one.
    #
    # This is NOT asserted as desirable. It is pinned so that whichever way it
    # is resolved -- prune `blockers` for a net whose FINAL verdict is boxed_in,
    # or re-word the classifier row -- this arm fires and gets updated with it.
    _both = sorted(set(_boxed_nets)
                   & {b.get('net') for b in (summary.get('blockers') or [])})
    check('KNOWN (#622 flip): the requeued ripper is in BOTH `blockers` and '
          '`boxed_in`, which defeats the skill\'s "blockers empty AND '
          'boxed_in names the net" row -- see the comment above',
          _both == ['B'],
          f'nets in both keys: {_both!r} (expected [\'B\']); if this is now '
          f'empty the ambiguity was FIXED -- delete this arm and restore the '
          f'plain "blockers is empty for it" check')
    check('results_data carries the same boxed_in (GUI parity, rip victim)',
          results_data.get('boxed_in') == summary.get('boxed_in'),
          f'{results_data.get("boxed_in")!r} vs {summary.get("boxed_in")!r}')


def case_merge_keeps_boxed_in():
    """`boxed_in` merges like `blockers`: first-pass evidence, filtered to
    the nets the reconcile sub-run left failing."""
    from route_summary import merge_summaries
    first = {'scope': 'run', 'successful': 0, 'failed': 2,
             'failed_single': ['X', 'Y'],
             'blockers': [{'net': 'X', 'stage': 'initial', 'blocked_by': []},
                          {'net': 'Y', 'stage': 'initial', 'blocked_by': []}],
             'boxed_in': [{'net': 'X', 'verdict': 'boxed_in_static',
                           'iterations': 10},
                          {'net': 'Y', 'verdict': 'boxed_in_static',
                           'iterations': 12}]}
    second = {'scope': 'reconciliation-subset', 'successful': 1, 'failed': 1,
              'failed_single': ['Y']}
    merged = merge_summaries([first, second])
    check('merged boxed_in keeps only the net still failing (Y), clears X',
          [e.get('net') for e in (merged or {}).get('boxed_in') or []] == ['Y'],
          str((merged or {}).get('boxed_in')))
    check('and blockers filter the same way (unchanged behaviour)',
          [e.get('net') for e in (merged or {}).get('blockers') or []] == ['Y'],
          str((merged or {}).get('blockers')))
    check('a summary with no boxed_in at all gains no key',
          'boxed_in' not in (merge_summaries([second]) or {}),
          str(merge_summaries([second])))


def _open_board():
    """A routable two-pad net whose pads sit on OPPOSITE layers, so every
    run places exactly one via -- a countable effort the sink bug doubles."""
    bi = BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'},
                   copper_layers=['F.Cu', 'B.Cu'],
                   board_bounds=(0.0, 0.0, 10.0, 10.0))
    pads = {A: [make_pad(A, 2.0, 5.0, ref='U1', num='1', net_name='A',
                         size_x=0.5, size_y=0.5, layers=('F.Cu',)),
                make_pad(A, 8.0, 5.0, ref='U2', num='1', net_name='A',
                         size_x=0.5, size_y=0.5, layers=('B.Cu',))]}
    return make_pcb(nets={A: make_net(A, 'A')}, pads_by_net=pads,
                    board_info=bi)


def case_gui_front_second_run():
    """Two outermost calls in ONE process, the GUI's shape (no --json-out,
    default final_reconcile): the second run's JSON_SUMMARY_MIN must describe
    the second run only. Before the sink reset on every outermost entry the
    merge summed effort keys across both runs and stamped the second run's
    summaries 'reconciliation-subset'."""
    from route import batch_route
    outs = []
    for _ in range(2):
        buf = io.StringIO()
        with redirect_stdout(buf):
            batch_route('synthetic', '', ['A'], layers=['F.Cu', 'B.Cu'],
                        clearance=0.2, track_width=0.2, via_size=0.5,
                        via_drill=0.3, grid_step=0.1,
                        ordering_strategy='original', max_rip_up_count=0,
                        return_results=True, pcb_data=_open_board())
        outs.append(buf.getvalue())
    big = re.findall(r'JSON_SUMMARY: (\{.*\})', outs[1])
    mins = re.findall(r'JSON_SUMMARY_MIN: (\{.*\})', outs[1])
    check('second run prints its own JSON_SUMMARY', bool(big))
    check('second run prints exactly one JSON_SUMMARY_MIN', len(mins) == 1,
          str(len(mins)))
    if not (big and mins):
        return
    s = json.loads(big[-1]); m = json.loads(mins[0])
    check('the fixture routes with a via (or the effort check is vacuous)',
          s.get('successful') == 1 and (s.get('total_vias') or 0) >= 1,
          f"successful={s.get('successful')} total_vias={s.get('total_vias')}")
    check("second run's summary is scoped 'run', not 'reconciliation-subset'",
          s.get('scope') == 'run', str(s.get('scope')))
    check("MIN vias equal the second run's own total_vias (not both runs summed)",
          m.get('vias') == s.get('total_vias'),
          f"MIN {m.get('vias')} vs own {s.get('total_vias')}")
    check("MIN routed equals the second run's own count",
          m.get('routed') == s.get('successful'),
          f"MIN {m.get('routed')} vs own {s.get('successful')}")


def main():
    print('-- first-pass cage (single_ended_loop)')
    case_static_cage()
    print('-- rip victim rerouted into a cage (reroute_loop)')
    case_rip_victim_reroute()
    print('-- merge_summaries keeps boxed_in for nets still failing')
    case_merge_keeps_boxed_in()
    print('-- GUI front: second outermost run in one process')
    case_gui_front_second_run()

    bad = [n for n, ok in CHECKS if not ok]
    print(f"\n{len(CHECKS) - len(bad)} passed, {len(bad)} failed")
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
