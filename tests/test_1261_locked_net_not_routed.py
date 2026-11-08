#!/usr/bin/env python3
"""#1261: a net KiCad-connected by copper that is ALL locked is not routed.

The #578 fragment gate routes a zone-less net whose pads grade connected but
whose copper is several STRICT fragments. When every segment and via of that
net is KiCad-locked, the fragments are the designer's pinned copper: no pass
may move or remove it, so "routing to join them" can only lay new copper
beside it, and it put the net in front of every rip ladder and layer switch
of the run. The reporter's NET_A -- "copper all locked, KiCad reports it
fully connected" -- was in the routing set and logged "Could not find pads
for both stub groups in NET_A (found 1 pads)", the warning this board
reproduces.

The board: NET_A's two locked 1 mm tracks overlap by 0.03 mm (KiCad and the
grader: connected; the strict view, which wants a 0.05 mm lens: two
fragments), the second ending 0.5 mm inside P2 off its centre. NET_Q is an
ordinary unrouted net.

Checks:
  1. route.py: NET_A is reported as pinned copper and skipped, NET_Q still
     routes, NET_A gains no copper, every locked item is byte-identical.
  2. Control: the same board with NET_A UNLOCKED still goes through the gate
     and is routed to join its fragments (#578 unchanged).

    python3 tests/test_1261_locked_net_not_routed.py
"""
import json
import os
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from run_utils import check as run_check, evidence, tool        # noqa: E402
from synth import RoutingBoard, locked_items                    # noqa: E402

failures = []
GATE = 'NET_A: pads grade connected, but its copper is 2 separate track fragment(s)'


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def board(lock):
    b = RoutingBoard()
    b.pad('P1', 10, 15, 'NET_A', sx=1.6, sy=1.6)
    b.pad('P2', 30, 15, 'NET_A', sx=2.0, sy=2.0)
    b.seg(10, 15, 20, 15, 'NET_A', w=1.0, locked=lock)
    b.seg(20.97, 15, 29.5, 15, 'NET_A', w=1.0, locked=lock)
    b.pad('Q1', 10, 25, 'NET_Q', sx=1.0, sy=1.0)
    b.pad('Q2', 30, 25, 'NET_Q', sx=1.0, sy=1.0)
    return b.text()


def net_a_copper(path):
    """(locked, unlocked) NET_A segments of a board, parsed."""
    import contextlib
    import io
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(path)
    nid = next(i for i, n in pcb.nets.items() if n.name == 'NET_A')
    segs = [s for s in pcb.segments if s.net_id == nid]
    vias = [v for v in pcb.vias if v.net_id == nid]
    return ([s for s in segs if s.locked],
            [s for s in segs if not s.locked] + list(vias))


def added_beyond_welds(path):
    """New NET_A copper that is not a weld between two locked endpoints.
    The cleanup's stub-gap snap may join the fragments end to end -- an
    addition inside the pinned copper's own gap, moving nothing -- but a
    ROUTE leaves locked endpoints and so lands here."""
    locked, new = net_a_copper(path)
    ends = [(s.start_x, s.start_y) for s in locked] + [(s.end_x, s.end_y) for s in locked]

    def on_end(x, y):
        return any(abs(x - ex) < 0.01 and abs(y - ey) < 0.01 for ex, ey in ends)
    return [s for s in new
            if not (hasattr(s, 'start_x') and on_end(s.start_x, s.start_y)
                    and on_end(s.end_x, s.end_y))]


def route(text, tmp, tag):
    inp = os.path.join(tmp, f'{tag}_in.kicad_pcb')
    out = os.path.join(tmp, f'{tag}_out.kicad_pcb')
    js = os.path.join(tmp, f'{tag}.json')
    with open(inp, 'w', encoding='utf-8') as fh:
        fh.write(text)
    evidence(inp, 'synthetic input board')
    env = dict(os.environ, KICAD_PLANE_FINALIZE='0')
    r = run_check([sys.executable, '-X', 'utf8', tool('route.py'), inp, out,
                   '--nets', '*', '--json-out', js, '--keep-input-copper',
                   '--escalation', 'off', '--no-fix-drc-settings'],
                  accept=True, env=env)
    evidence(out, 'routed output')
    with open(evidence(js, '--json-out'), encoding='utf-8') as fh:
        doc = json.load(fh)
    return r.stdout + r.stderr, doc, inp, out


def gate_rows():
    """The gate itself, on the same geometry, per kind of immutable copper.
    Copper a `sense:` group holds (#1279) is never written `(locked yes)`,
    yet no pass may move or remove it either, so an all-held net is pinned
    copper too (#1781). The unlocked row is the control: it must enroll."""
    import contextlib
    import io
    from routing_common import filter_already_routed
    from routing_config import GridRouteConfig
    from synth import make_net, make_pad, make_pcb, make_seg
    print("3. the fragment gate, per kind of copper")
    pads = [make_pad(1, 10, 15, ref='P1', num='1', size_x=1.6, size_y=1.6),
            make_pad(1, 30, 15, ref='P2', num='1', size_x=2.0, size_y=2.0)]
    got = {}
    for tag, kw in (('unlocked', {}), ('locked', {'locked': True}),
                    ('sense-held', {'held_by': 'sense:P1.1'})):
        segs = [make_seg(10, 15, 20, 15, net_id=1, width=1.0, **kw),
                make_seg(20.97, 15, 29.5, 15, net_id=1, width=1.0, **kw)]
        pcb = make_pcb(segments=segs, vias=[], pads_by_net={1: pads},
                       nets={1: make_net(1, 'NET_A')})
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            to_route, _done = filter_already_routed(
                pcb, [('NET_A', 1)], GridRouteConfig(layers=['F.Cu', 'B.Cu']),
                fragment_gate=True)
        got[tag] = ([n for n, _i in to_route], buf.getvalue())
    if got['unlocked'][0] != ['NET_A']:
        print("  [BROKEN TEST] the unlocked row did not enroll: this geometry "
              f"does not reach the gate (got {got['unlocked'][0]})")
        failures.append('gate rows: unlocked control')
        return
    print("  [ok] control: the unlocked net enrolls")
    for tag in ('locked', 'sense-held'):
        to_route, log = got[tag]
        check(f"an all-{tag} net is pinned copper, not routed",
              to_route == [] and 'KiCad-locked or sense-held' in log,
              f"to_route={to_route}; "
              + '; '.join(l.strip() for l in log.splitlines() if 'NET_A' in l)[:200])


def main():
    gate_rows()
    tmp = tempfile.mkdtemp(prefix='t1261n_')
    try:
        print("2. control: NET_A unlocked")
        con, doc, inp, out = route(board(lock=False), tmp, 'control')
        if GATE not in con:
            print("  [BROKEN TEST] the unlocked control never reached the #578 "
                  "fragment gate: this board does not exercise it")
            return 1
        check("the unlocked net is routed to join its fragments",
              'NET_A' in (doc.get('routed_single') or []),
              f"routed_single={doc.get('routed_single')}")

        print("1. NET_A's copper all locked")
        con, doc, inp, out = route(board(lock=True), tmp, 'locked')
        check("the gate does not route it",
              GATE not in con and 'Routing NET_A' not in con,
              '; '.join(l.strip() for l in con.splitlines() if 'NET_A' in l)[:300])
        check("it is reported as pinned copper",
              'every segment and via is KiCad-locked' in con)
        check("NET_Q still routes", doc.get('routed_single') == ['NET_Q'],
              str(doc.get('routed_single')))
        extra = added_beyond_welds(out)
        check("NET_A gains no routed copper (at most a weld between its "
              "locked endpoints)", not extra,
              f"{len(extra)} new item(s), e.g. "
              f"{[(getattr(s, 'start_x', None), getattr(s, 'start_y', None), getattr(s, 'end_x', None), getattr(s, 'end_y', None)) for s in extra[:2]]}")
        before, after = locked_items(inp), locked_items(out)
        check("every locked item comes out byte-identical",
              len(before) == 2 and after == before,
              f"lost/changed: {sorted(set(before) - set(after))[:2]}")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
