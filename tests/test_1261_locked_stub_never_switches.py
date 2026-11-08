#!/usr/bin/env python3
"""#1261: no layer switch moves a KiCad-locked stub.

The SE stub-swap rescue (#424) re-layered a `(locked yes)` segment: the output
kept its uuid, ends, width and lock and changed only its layer. Every layer
switch -- the upfront solo/pair swaps, MPS swaps, the global plan, the rescue,
the diff-pair fallbacks, the boxed-stub switch -- reads its stub through
stub_layer_switching.get_stub_info, which now answers None for a stub whose
switch would move locked copper; apply_stub_layer_switch refuses one as a
backstop.

Checks:
  1. get_stub_info: None for a locked stub; include_locked=True returns it
     marked `locked`; validate_single_swap refuses it; apply_stub_layer_switch
     moves nothing.
  2. route.py, the STUB-SWAP RESCUE board (base: "STUB-SWAP RESCUE: NET_A stub
     F.Cu -> In1.Cu routed", the locked stub shipped on In1.Cu): no rescue
     switch of NET_A, and every locked item comes out byte-identical.
  3. route.py, the upfront solo switch board (base: "Solo source switch:
     NET_A (F.Cu->In1.Cu)", same signature): same two checks.

    python3 tests/test_1261_locked_stub_never_switches.py
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

from run_utils import check as run_check, evidence, tool        # noqa: E402
from synth import (RoutingBoard, locked_items, make_net, make_pad,  # noqa: E402
                   make_pcb, make_seg)
from kicad_parser import BoardInfo                              # noqa: E402
from routing_config import GridRouteConfig                      # noqa: E402
import stub_layer_switching as sls                              # noqa: E402

failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def unit():
    print("1. get_stub_info / validate / apply on a locked stub")
    bi = BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'}, copper_layers=['F.Cu', 'B.Cu'],
                   board_bounds=(0.0, 0.0, 40.0, 30.0))

    def pcb_with(lock):
        stub = make_seg(10, 15, 14, 15, net_id=1, width=0.3, locked=lock)
        pads = [make_pad(1, 10, 15, ref='P1', size_x=0.8, size_y=0.8),
                make_pad(1, 30, 15, ref='P2', size_x=0.8, size_y=0.8)]
        return make_pcb(nets={1: make_net(1, 'NET_A')}, segments=[stub],
                        pads_by_net={1: pads}, board_info=bi), stub

    cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'], clearance=0.25,
                          track_width=0.3, via_size=0.5, via_drill=0.3)
    pcb, stub = pcb_with(False)
    free = sls.get_stub_info(pcb, 1, 14, 15, 'F.Cu')
    check("precondition: the unlocked stub is a switch candidate",
          free is not None and not getattr(free, 'locked', False))
    pcb, stub = pcb_with(True)
    check("a locked stub is no switch candidate",
          sls.get_stub_info(pcb, 1, 14, 15, 'F.Cu') is None)
    try:
        held = sls.get_stub_info(pcb, 1, 14, 15, 'F.Cu', include_locked=True)
    except TypeError as e:          # a checkout without the #1261 parameter
        check("include_locked returns it, marked locked", False, str(e))
        return
    check("include_locked returns it, marked locked",
          held is not None and held.locked)
    ok, why = sls.validate_single_swap(held, 'B.Cu', {}, pcb, cfg)
    check("validate_single_swap refuses it", not ok and 'locked' in why, why)
    with contextlib.redirect_stdout(io.StringIO()):
        vias, mods = sls.apply_stub_layer_switch(pcb, held, 'B.Cu', cfg,
                                                 debug=False)
    check("apply_stub_layer_switch moves nothing",
          (vias, mods) == ([], []) and stub.layer == 'F.Cu' and not pcb.vias,
          f"{len(vias)} via(s), {len(mods)} mod(s), stub on {stub.layer}")


def rescue_board(lock=True):
    """NET_A: P1's locked F.Cu stub sits inside a locked B.Cu ring of NET_C
    that admits a via only at P1's exact (off-grid) centre -- where the
    rescue's own pad via goes, and where the router's grid cannot -- and the
    route is restricted to In1/In2, so the F.Cu stub must leave F.Cu. The
    centre is off the HALF grid too: the final reconciliation's finer-grid
    rung (#1763) re-routes a boxed-in net at grid 0.05, where a centre on
    x.x5 is reachable without any switch of this stub."""
    b = RoutingBoard(layers=('F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu'))
    px, py, ex = 10.025, 15.025, 14.025
    b.pad('P1', px, py, 'NET_A')
    b.pad('P2', 30, 15, 'NET_A')
    b.seg(px, py, ex, py, 'NET_A', locked=lock)
    b.seg(30, 15, 26, 15, 'NET_A')
    d = 0.25 + 0.25 + 0.15 + 0.01
    b.ring([(round(px - d, 4), round(py - d, 4)), (round(ex + d, 4), round(py - d, 4)),
            (round(ex + d, 4), round(py + d, 4)), (round(px - d, 4), round(py + d, 4))],
           'NET_C', w=0.3, layer='B.Cu', locked=True)
    b.pad('C1', 4, 15, 'NET_C', sx=1.6, sy=1.6, tht=True)
    b.pad('C2', 4, 25, 'NET_C', sx=1.6, sy=1.6, tht=True)
    b.seg(4, 15, round(px - d, 4), round(py - d, 4), 'NET_C', layer='B.Cu', locked=True)
    b.seg(4, 15, 4, 25, 'NET_C', layer='B.Cu', locked=True)
    return b.text(), ['--nets', '*', '--layers', 'In1.Cu', 'In2.Cu']


def upfront_board(lock=True):
    """NET_A: a locked F.Cu stub on an SMD pad and a through-hole target,
    routed on In1/In2 only: the upfront single-ended solo switch moved the
    stub to In1.Cu before any routing."""
    b = RoutingBoard(layers=('F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu'))
    b.pad('P1', 10, 15, 'NET_A')
    b.pad('P2', 30, 15, 'NET_A', sx=1.6, sy=1.6, tht=True)
    b.seg(10, 15, 14, 15, 'NET_A', locked=lock)
    return b.text(), ['--nets', '*', '--layers', 'In1.Cu', 'In2.Cu']


def _route(text, args):
    tmp = tempfile.mkdtemp(prefix='t1261s_')
    try:
        inp = os.path.join(tmp, 'in.kicad_pcb')
        out = os.path.join(tmp, 'out.kicad_pcb')
        with open(inp, 'w', encoding='utf-8') as fh:
            fh.write(text)
        evidence(inp, 'synthetic input board')
        env = dict(os.environ, KICAD_PLANE_FINALIZE='0')
        r = run_check([sys.executable, '-X', 'utf8', tool('route.py'), inp, out]
                      + args + ['--keep-input-copper', '--escalation', 'off',
                                '--no-fix-drc-settings'],
                      accept=True, env=env)
        evidence(out, 'routed output')
        return r.stdout + r.stderr, locked_items(inp), locked_items(out)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def route_case(label, board, banned):
    print(label)
    # Control: the same stub UNLOCKED must be switched, or this board does
    # not reach the path under test and the locked arm would pass vacuously.
    con, _, _ = _route(*board(lock=False))
    if not any(banned in l for l in con.splitlines()):
        print(f"  [BROKEN TEST] the unlocked control never printed {banned!r}: "
              f"this board does not exercise the switch")
        failures.append(f"{label}: broken fixture")
        return
    print(f"  [ok] control: the unlocked stub is switched ({banned!r})")
    con, before, after = _route(*board(lock=True))
    check("precondition: the input carries locked copper", len(before) >= 1)
    hit = [l.strip() for l in con.splitlines() if banned in l]
    check("NET_A's locked stub is not switched", not hit, '; '.join(hit))
    check("every locked item comes out byte-identical", after == before,
          f"lost/changed: {sorted(set(before) - set(after))[:2]}")


def main():
    unit()
    route_case("2. STUB-SWAP RESCUE board", rescue_board,
               banned='STUB-SWAP RESCUE: NET_A')
    route_case("3. upfront solo switch board", upfront_board,
               banned='Solo source switch: NET_A')
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
