#!/usr/bin/env python3
"""#1261: no cleanup pass removes, trims, re-bends or shifts KiCad-locked copper.

`(locked yes)` copper is user-pinned (#521). The cleanup passes honoured that
only under --keep-input-copper: without it, the dead-end sweep, the orphan-
island pass, the cycle prune, the dangle trim and the three graze passes
(prune, octolinear re-bend, micro-shift) treated a locked input segment like
any other and deleted or rewrote it. Measured on the whole-board case below
(`--nets "*"`): 6 of its 9 locked items gone from the output.
The KiCad oracle's stranded-fragment deleter, which runs inside the plane
finalize, did the same to a locked lone via (a block's "port" via).

Each case builds one small board twice -- copper UNLOCKED (the control: the
pass must act on it, or the fixture tests nothing and the case is reported as
a BROKEN TEST) and copper LOCKED (the pass must leave every locked item
exactly where it was: same ends, layer and width, still present).

The last case is that whole board through route.py (`--nets "*"`, no
--keep-input-copper): a spur, a pad-less island and a lone via, a loop, and
an ordinary net to route; every locked line must come out of the writer
byte-identical, which also covers the stale-input strip.

    python3 tests/test_1261_locked_copper_cleanup.py
"""
import contextlib
import io
import os
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from kicad_parser import BoardInfo                                  # noqa: E402
from synth import (RoutingBoard, locked_items, make_net, make_pad,  # noqa: E402
                   make_pcb, make_seg, make_via)
from run_utils import check as run_check, evidence, tool            # noqa: E402
import pcb_modification as pm                                       # noqa: E402

LAYERS = ['F.Cu', 'B.Cu']
failures = []
broken = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def _board(segs, vias, pads, nets):
    bi = BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'}, copper_layers=list(LAYERS),
                   board_bounds=(-20.0, -20.0, 60.0, 60.0))
    by_net = {}
    for p in pads:
        by_net.setdefault(p.net_id, []).append(p)
    return make_pcb(nets={i: make_net(i, n) for i, n in nets.items()},
                    segments=segs, vias=vias, pads_by_net=by_net,
                    board_info=bi)


def _sig(x):
    if hasattr(x, 'start_x'):
        return ('seg', x.net_id, x.layer, round(x.start_x, 4), round(x.start_y, 4),
                round(x.end_x, 4), round(x.end_y, 4), round(x.width, 4))
    return ('via', x.net_id, round(x.x, 4), round(x.y, 4), round(x.size, 4))


def _pads(*spec):
    return [make_pad(nid, x, y, ref=f'U{i}', num='1', size_x=1.0, size_y=1.0)
            for i, (nid, x, y) in enumerate(spec)]


# --- fixtures: (segments, vias, pads, nets, items under test) ---------------

def fx_dead_end(lock):
    trunk = make_seg(0, 0, 10, 0, net_id=1, width=0.3, locked=lock)
    spur = make_seg(5, 0, 5, 4, net_id=1, width=0.3, locked=lock)
    return [trunk, spur], [], _pads((1, 0, 0), (1, 10, 0)), {1: 'A'}, [spur]


def fx_orphan(lock):
    trunk = make_seg(0, 0, 10, 0, net_id=1, width=0.3, locked=lock)
    island = make_seg(2, 5, 6, 5, net_id=1, width=0.3, locked=lock)
    via = make_via(8, 7, net_id=1, locked=lock)
    return ([trunk, island], [via], _pads((1, 0, 0), (1, 10, 0)), {1: 'A'},
            [island, via])


def fx_cycle(lock):
    segs = [make_seg(0, 0, 10, 0, net_id=1, width=0.3, locked=lock),
            make_seg(0, 0, 0, 3, net_id=1, width=0.3, locked=lock),
            make_seg(0, 3, 10, 3, net_id=1, width=0.3, locked=lock),
            make_seg(10, 3, 10, 0, net_id=1, width=0.3, locked=lock)]
    return segs, [], _pads((1, 0, 0), (1, 10, 0)), {1: 'A'}, list(segs)


def fx_dangle(lock):
    # The trunk is load-bearing up to the tee at x=6; past it, it dangles.
    trunk = make_seg(0, 0, 10, 0, net_id=1, width=0.3, locked=lock)
    tee = make_seg(6, 0, 6, 5, net_id=1, width=0.3, locked=lock)
    return ([trunk, tee], [], _pads((1, 0, 0), (1, 6, 5)), {1: 'A'},
            [trunk, tee])


def fx_graze_redundant(lock):
    # A loop whose detour leg passes 0.20 mm from net B's via (clearance
    # 0.25): the leg is redundant, so the graze prune would drop it.
    segs = [make_seg(0, 0, 10, 0, net_id=1, width=0.3, locked=lock),
            make_seg(0, 0, 0, 2, net_id=1, width=0.3, locked=lock),
            make_seg(0, 2, 10, 2, net_id=1, width=0.3, locked=lock),
            make_seg(10, 2, 10, 0, net_id=1, width=0.3, locked=lock)]
    other = make_via(5, 2.6, net_id=2, size=0.5)
    return (segs, [other], _pads((1, 0, 0), (1, 10, 0), (2, 20, 20)),
            {1: 'A', 2: 'B'}, list(segs))


def fx_graze_jog(lock):
    # A load-bearing jog whose apex passes 0.20 mm from net B's via: it
    # cannot be pruned, so the octolinear pass re-bends it.
    segs = [make_seg(0, 0, 4, 0, net_id=1, width=0.3, locked=lock),
            make_seg(4, 0, 5, 1.0, net_id=1, width=0.3, locked=lock),
            make_seg(5, 1.0, 6, 0, net_id=1, width=0.3, locked=lock),
            make_seg(6, 0, 10, 0, net_id=1, width=0.3, locked=lock)]
    other = make_via(5, 1.6, net_id=2, size=0.5)
    return (segs, [other], _pads((1, 0, 0), (1, 10, 0), (2, 20, 20)),
            {1: 'A', 2: 'B'}, list(segs))


def fx_graze_vertex(lock):
    # A joint vertex 0.01 mm inside clearance of net B's via: the
    # micro-shift slides it (max_shift 0.025).
    segs = [make_seg(0, 0, 5, 0.69, net_id=1, width=0.3, locked=lock),
            make_seg(5, 0.69, 10, 0, net_id=1, width=0.3, locked=lock)]
    other = make_via(5, 1.33, net_id=2, size=0.5)
    return (segs, [other], _pads((1, 0, 0), (1, 10, 0), (2, 20, 20)),
            {1: 'A', 2: 'B'}, list(segs))


# --- passes ------------------------------------------------------------------

def run_pass(name, pcb):
    scope = {1}
    with contextlib.redirect_stdout(io.StringIO()):
        if name == 'sweep_dead_ends':
            pm.sweep_dead_ends([], pcb, scope)
        elif name == 'remove_orphan_islands':
            pm.remove_orphan_islands([], pcb, scope)
        elif name == 'prune_redundant_cycles':
            pm.prune_redundant_cycles([], pcb, scope, clearance=0.25)
        elif name == 'trim_dangles_past_body_anchor':
            pm.trim_dangles_past_body_anchor([], pcb, scope)
        elif name == 'prune_grazing_segments':
            pm.prune_grazing_segments([], pcb, scope, clearance=0.25)
        elif name == 'nudge_grazing_octolinear':
            pm.nudge_grazing_octolinear([], pcb, scope, clearance=0.25)
        elif name == 'nudge_grazing_microshift':
            pm.nudge_grazing_microshift([], pcb, scope, clearance=0.25)
        elif name == 'oracle_delete_stranded_fragment':
            import kicad_oracle
            # KiCad's link report: pad (0, 0) to the island, and to the via.
            for end in ((4, 5), (8, 7)):
                hit = kicad_oracle._delete_stranded_link_fragment(
                    pcb, 1, (0, 0), end)
                if hit:
                    gone = {id(x) for x in hit[0] + hit[1]}
                    pcb.segments[:] = [s for s in pcb.segments
                                       if id(s) not in gone]
                    pcb.vias[:] = [v for v in pcb.vias if id(v) not in gone]
        else:
            raise KeyError(name)


CASES = [
    ('sweep_dead_ends', fx_dead_end),
    ('remove_orphan_islands', fx_orphan),
    ('prune_redundant_cycles', fx_cycle),
    ('trim_dangles_past_body_anchor', fx_dangle),
    ('prune_grazing_segments', fx_graze_redundant),
    ('nudge_grazing_octolinear', fx_graze_jog),
    ('nudge_grazing_microshift', fx_graze_vertex),
    ('oracle_delete_stranded_fragment', fx_orphan),
]


def board_sigs(pcb):
    return {_sig(x) for x in list(pcb.segments) + list(pcb.vias)}


def oracle_sense_held():
    """Copper a `sense:` group holds (#1279) is immutable too, though it is
    never written `(locked yes)`: the oracle's deleter must ask
    is_immutable_copper, not `locked` alone. The unlocked control above
    already showed the deleter acts on this fixture."""
    print("oracle_delete_stranded_fragment, sense-held:")
    segs, vias, pads, nets, watched = fx_orphan(False)
    for x in watched:
        x.held_by = 'sense:U0.1'
    pcb = _board(segs, vias, pads, nets)
    before = {_sig(x) for x in watched}
    run_pass('oracle_delete_stranded_fragment', pcb)
    lost = sorted(before - board_sigs(pcb))
    check("oracle_delete_stranded_fragment leaves sense-held copper in place",
          not lost, f"changed or removed: {lost[:3]}")


def cleanup_board(lock):
    b = RoutingBoard(w=40, h=36)
    b.pad('D1', 5, 5, 'NET_D')
    b.pad('D2', 20, 5, 'NET_D')
    b.seg(5, 5, 20, 5, 'NET_D', locked=lock)
    b.seg(12, 5, 12, 9, 'NET_D', locked=lock)          # dead-end spur
    b.pad('O1', 5, 15, 'NET_O')
    b.pad('O2', 20, 15, 'NET_O')
    b.seg(5, 15, 20, 15, 'NET_O', locked=lock)
    b.seg(8, 18, 14, 18, 'NET_O', locked=lock)         # pad-less island
    b.via(17, 19, 'NET_O', locked=lock)                # lone via
    b.pad('L1', 5, 25, 'NET_L')
    b.pad('L2', 20, 25, 'NET_L')
    b.seg(5, 25, 20, 25, 'NET_L', locked=lock)          # a loop
    b.seg(5, 25, 5, 28, 'NET_L', locked=lock)
    b.seg(5, 28, 20, 28, 'NET_L', locked=lock)
    b.seg(20, 28, 20, 25, 'NET_L', locked=lock)
    b.pad('Q1', 25, 32, 'NET_Q')                        # a net to route
    b.pad('Q2', 38, 32, 'NET_Q')
    return b.text()


def whole_board():
    print("route.py, no --keep-input-copper:")
    tmp = tempfile.mkdtemp(prefix='t1261c_')
    try:
        res = {}
        for tag, lock in (('control', False), ('locked', True)):
            inp = os.path.join(tmp, f'{tag}_in.kicad_pcb')
            out = os.path.join(tmp, f'{tag}_out.kicad_pcb')
            with open(inp, 'w', encoding='utf-8') as fh:
                fh.write(cleanup_board(lock))
            evidence(inp, 'synthetic input board')
            env = dict(os.environ, KICAD_PLANE_FINALIZE='0')
            r = run_check([sys.executable, '-X', 'utf8', tool('route.py'), inp,
                           out, '--nets', '*', '--escalation', 'off',
                           '--no-fix-drc-settings'], accept=True, env=env)
            evidence(out, 'routed output')
            res[tag] = (r.stdout + r.stderr, inp, out)
        con = res['control'][0]
        if not ('Dead-end sweep: trimmed' in con and 'Orphan islands: removed' in con
                and 'Cycle prune: removed' in con):
            broken.append('whole board')
            print("  [BROKEN TEST] the unlocked control did not sweep, de-island "
                  "and prune: this board exercises nothing")
            return
        print("  [ok] control: the unlocked spur, island, via and loop are cleaned")
        _, inp, out = res['locked']
        before, after = locked_items(inp), locked_items(out)
        check("precondition: 9 locked items in", len(before) == 9, str(len(before)))
        check("every locked item comes out byte-identical", after == before,
              f"{len(before)} in, {len(after)} out; lost: "
              f"{sorted(set(before) - set(after))[:2]}")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def main():
    for name, fx in CASES:
        print(f"{name}:")
        # Control: unlocked copper -- the pass must change something, or this
        # fixture does not exercise it.
        segs, vias, pads, nets, watched = fx(False)
        pcb = _board(segs, vias, pads, nets)
        before = {_sig(x) for x in watched}
        run_pass(name, pcb)
        acted = not before <= board_sigs(pcb)
        if not acted:
            broken.append(name)
            print(f"  [BROKEN TEST] the unlocked control was not changed by "
                  f"{name}: this fixture tests nothing")
            continue
        # Locked: every watched item survives exactly.
        segs, vias, pads, nets, watched = fx(True)
        pcb = _board(segs, vias, pads, nets)
        before = {_sig(x) for x in watched}
        run_pass(name, pcb)
        after = board_sigs(pcb)
        lost = sorted(before - after)
        check(f"{name} leaves locked copper exactly in place", not lost,
              f"changed or removed: {lost[:3]}")
    oracle_sense_held()
    whole_board()

    if broken:
        print('BROKEN TEST: ' + ', '.join(broken))
        return 2
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
