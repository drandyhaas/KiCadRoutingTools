#!/usr/bin/env python3
"""A ball this run ripped is not called a fanout-dropped ball (#1311).

fanout_dropped_ball_hint (#652) reads the LIVE board: an interior ball of a
package's pad field with no copper of its net attached is "a fanout-dropped
ball (no escape stub)", and the hint says to re-run the fanout. Mid-run the
live board lacks whatever this run ripped, so a ball the fanout DID escape --
its stub ripped as a blocker, or with its own net's reroute -- read as never
escaped. On glasgow (gap7_s2a_c, the issue's route.py line) main printed it
for U30.E8, D7, J9, G9, D9, J3 and F9 inside the plane finalize, and every
one of those balls carries its escape on the input board.

The hint now compares against the copper the step was HANDED
(`pcb_data._input_copper`, rip_up_reroute.mark_input_copper): a ball whose
escape the input carried is reported as ripped by this run, with no fanout
advice, and is not recorded as a `fanout_dropped` event (the summary's
`fanout_dropped` key lists only balls the fanout never escaped).

  1. a ball never escaped: the #652 hint and verdict, unchanged;
  2. a ball escaped on the input and ripped since: verdict 'ripped_by_run',
     no "re-run the fanout";
  3. both on one net: the fanout verdict names the never-escaped ball only;
  4. every caller records `fanout_dropped` only for that verdict.

    python3 tests/test_1311_ripped_escape_not_fanout.py
"""
import ast
import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for p in (ROOT, TESTS_DIR, os.path.join(ROOT, 'py_router')):
    if p not in sys.path:
        sys.path.insert(0, p)

from synth import make_pad, make_seg, make_pcb, make_net         # noqa: E402
from kicad_parser import Footprint                              # noqa: E402
from rip_up_reroute import mark_input_copper                    # noqa: E402
from routing_diagnostics import fanout_dropped_ball_hint        # noqa: E402

FAILS = []
NET = 5
PITCH = 0.8


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name
          + (f'  {detail}' if detail else ''))


def bga(escaped):
    """A 6x6 ball field; NET on the interior balls C3 and C4, the rest net 9.
    `escaped` names the NET balls that carry an escape stub on the input."""
    pads, by_net = [], {}
    for r in range(6):
        for c in range(6):
            num = f"{'ABCDEF'[r]}{c + 1}"
            nid = NET if num in ('C3', 'C4') else 9
            p = make_pad(nid, 10 + c * PITCH, 10 + r * PITCH, ref='U30',
                         num=num, net_name='/N' if nid == NET else '/X',
                         size_x=0.4, size_y=0.4)
            pads.append(p)
            by_net.setdefault(nid, []).append(p)
    fp = Footprint(reference='U30', footprint_name='t:BGA36', x=12, y=12,
                   rotation=0, layer='F.Cu', pads=pads)
    stubs = [make_seg(p.global_x, p.global_y, p.global_x + 0.4,
                      p.global_y + 0.4, net_id=NET)
             for p in pads if p.pad_number in escaped]
    pcb = make_pcb(nets={NET: make_net(NET, '/N'), 9: make_net(9, '/X')},
                   footprints={'U30': fp}, segments=stubs, pads_by_net=by_net)
    mark_input_copper(pcb)
    return pcb, stubs


def run(pcb):
    return fanout_dropped_ball_hint(pcb, None, NET, '/N', return_verdict=True)


def main():
    print('1. never escaped: the fanout hint')
    pcb, _ = bga(escaped=())
    h, v = run(pcb)
    check('verdict fanout_dropped, fanout advice',
          (v or {}).get('verdict') == 'fanout_dropped'
          and 're-run the fanout' in h.lower(), h[:90])

    print('2. escaped on the input, ripped since: not the fanout')
    pcb, stubs = bga(escaped=('C3', 'C4'))
    h0, _v0 = run(pcb)
    check('with its stubs on the board there is no hint at all', h0 == '', h0)
    pcb.segments = []                        # this run ripped the escapes
    h, v = run(pcb)
    check("verdict ripped_by_run, no fanout advice",
          (v or {}).get('verdict') == 'ripped_by_run'
          and 'fanout' in h and 're-run the fanout' not in h.lower()
          and 'this run ripped it' in h, h[:120])

    print('3. one ball of each: the fanout verdict names only the dropped one')
    pcb, stubs = bga(escaped=('C3',))
    pcb.segments = []
    h, v = run(pcb)
    check('fanout_dropped on U30.C4, not C3',
          (v or {}).get('verdict') == 'fanout_dropped'
          and v.get('pad') == 'U30.C4' and v.get('pads') == ['U30.C4'],
          str(v))

    print('4. every caller records fanout_dropped only for that verdict')
    bad = []
    for mod in ('single_ended_loop', 'reroute_loop', 'phase3_routing',
                'diff_pair_loop'):
        src = open(os.path.join(ROOT, 'py_router', mod + '.py'),
                   encoding='utf-8').read()
        tree = ast.parse(src)
        parent = {c: p for p in ast.walk(tree) for c in ast.iter_child_nodes(p)}
        calls = [c for c in ast.walk(tree) if isinstance(c, ast.Call)
                 and len(c.args) >= 3 and isinstance(c.args[2], ast.Constant)
                 and c.args[2].value == 'fanout_dropped']
        if not calls:
            bad.append(f'{mod}: no record found')
        for call in calls:
            node = parent.get(call)
            while node is not None and not isinstance(node, ast.If):
                node = parent.get(node)
            if node is None or "'fanout_dropped'" not in ast.unparse(node.test):
                bad.append(f'{mod}:{call.lineno}')
    check('each record_net_event("fanout_dropped") sits under the verdict test',
          not bad, ', '.join(bad))

    print(f"\n{'ALL PASS' if not FAILS else f'{len(FAILS)} FAILED: ' + ', '.join(FAILS)}")
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
