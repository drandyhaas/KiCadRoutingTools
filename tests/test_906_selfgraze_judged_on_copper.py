#!/usr/bin/env python3
"""#1871: the self-graze exclusion from the protected nets is decided on the
copper that ships, and a routed pair the record leaves out is named.

The hybrid escape's self-graze fallback keeps the least-bad candidate when no
layer couples cleanly, and stamps the result `selfgraze` with the coupled
middle's P/N count AS BUILT -- before the #318 neck and the post-route
cleanup. `protection_candidates` (#906) kept every such pair out of the
protected nets, so a pair whose shipped copper has no P/N shortfall at all
was left rippable, and nothing said a routed pair had gone unrecorded.

`judge_selfgraze_on_copper` now measures each fallback pair on the board
(`pair_pn_shortfall`: P segments against N, and each member's vias against
the partner's copper): with no shortfall left the pair is admitted, with one
it stays out. Every routed pair the record leaves out is a row of
`JSON_SUMMARY.unprotected_pairs` and a console line.

Rows:
  1. end to end, orangecrab_ext_pll `/sheetIO/_USB_D*`: the fallback really
     ran (`[least self-graze; no clean layer]`), both members are recorded
     `diff-pair`, and `unprotected_pairs` is [];
  2. unit: a pair result carrying `selfgraze` whose P and N tracks stand
     0.1 mm apart under a 0.25 mm clearance stays out, and is listed with
     the reason `self-graze fallback` and its shortfall count;
  3. unit: the same pair 1.0 mm apart is admitted, and listed nowhere;
  4. unit: a via of one member 0.1 mm from the partner's track is a
     shortfall too;
  5. controls: `protection_candidates` with `selfgraze` and no judgement
     still returns {} (test_906_hybrid_pair_and_impedance_record's arm 1b),
     and a pair with no `selfgraze` is untouched by the judgement.
"""
import json
import os
import sys
import tempfile
from types import SimpleNamespace

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)
from run_utils import check, evidence, tool            # noqa: E402
from synth import make_seg, make_via, make_net, make_pcb  # noqa: E402
from kicad_parser import BoardInfo                     # noqa: E402

PY = [sys.executable, '-X', 'utf8']
BOARD = os.path.join(REPO, 'kicad_files', 'orangecrab_ext_pll.kicad_pcb')
FAILS = []


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def pair_board(gap, via_gap=None):
    """P at y=0 and N a `gap` away (edge to edge, 0.2 mm tracks); with
    `via_gap`, a 0.5 mm via of P that far from N's track."""
    segs = [make_seg(0, 0, 10, 0, net_id=1), make_seg(0, 0.2 + gap, 10, 0.2 + gap, net_id=2)]
    vias = []
    if via_gap is not None:
        vias.append(make_via(12.0, 0.2 + gap, net_id=1))
        segs.append(make_seg(10, 0.2 + gap, 12.0 - 0.25 - via_gap - 0.1, 0.2 + gap, net_id=2))
    pcb = make_pcb(nets={1: make_net(1, 'X_P'), 2: make_net(2, 'X_N')}, segments=segs,
                   vias=vias, pads_by_net={1: [], 2: []},
                   board_info=BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'},
                                        copper_layers=['F.Cu', 'B.Cu'],
                                        board_bounds=(-1.0, -1.0, 14.0, 3.0)),
                   zones=[])
    pair = SimpleNamespace(p_net_id=1, n_net_id=2, p_net_name='X_P', n_net_name='X_N')
    return pcb, [('X', pair)]


def units():
    from routing_config import GridRouteConfig
    from route_diff import (judge_selfgraze_on_copper, protection_candidates,
                            unprotected_pair_rows)
    cfg = GridRouteConfig(clearance=0.25, layers=['F.Cu', 'B.Cu'])

    def results():
        r = {'hybrid_escape': True, 'selfgraze': 3}
        return {1: r, 2: r}

    print("2. a fallback pair that still ships a shortfall")
    pcb, pairs = pair_board(0.1)
    res = results()
    short = judge_selfgraze_on_copper(res, pcb, cfg, pairs)
    prot = protection_candidates(res, pcb, pairs=pairs)
    rows = unprotected_pair_rows(pairs, prot, res, None, None, short, {'X'})
    ok(short.get('X', 0) >= 1 and prot == {}, f"stays out of the record: shortfall {short}")
    ok(rows == [{'pair': 'X', 'reason': 'self-graze fallback',
                 'pn_shortfall_segments': short.get('X')}], f"and is listed: {rows}")

    print("3. a fallback pair whose shipped copper is clear")
    pcb, pairs = pair_board(1.0)
    res = results()
    short = judge_selfgraze_on_copper(res, pcb, cfg, pairs)
    prot = protection_candidates(res, pcb, pairs=pairs)
    ok(short == {} and prot == {'X_P': 'diff-pair', 'X_N': 'diff-pair'},
       f"is admitted: {prot}")
    ok(unprotected_pair_rows(pairs, prot, res, None, None, short, {'X'}) == [],
       "and listed nowhere")

    print("4. a via against the partner's track")
    pcb, pairs = pair_board(1.0, via_gap=0.1)
    res = results()
    short = judge_selfgraze_on_copper(res, pcb, cfg, pairs)
    ok(short.get('X', 0) >= 1 and protection_candidates(res, pcb, pairs=pairs) == {},
       f"is a shortfall: {short}")

    print("5. controls")
    pcb, pairs = pair_board(1.0)
    res = results()
    ok(protection_candidates(res, pcb, pairs=pairs) == {},
       "`selfgraze` with no judgement still keeps the pair out")
    plain = {'hybrid_escape': True}
    res = {1: plain, 2: plain}
    ok(judge_selfgraze_on_copper(res, pcb, cfg, pairs) == {}
       and 'selfgraze_cleared' not in plain,
       "a pair with no `selfgraze` is not touched")


def end_to_end():
    print("1. orangecrab's USB pair through the self-graze fallback")
    from protected_nets import read_protected_nets, pro_path_for_board
    with tempfile.TemporaryDirectory() as td:
        board, out, js = (os.path.join(td, n) for n in
                          ('in.kicad_pcb', 'oc.kicad_pcb', 'oc.json'))
        check(PY + [tool('copy_board.py'), evidence(BOARD, 'the bundled board'), board],
              accept=True)
        r = check(PY + [tool('route_diff.py'), board, out, '--nets', '/sheetIO/_USB_D*',
                        '--json-out', js], accept=True, timeout=1800)
        log = (r.stdout or '') + (r.stderr or '')
        if '[least self-graze; no clean layer]' not in log:
            print("  [BROKEN TEST] the self-graze fallback did not run: nothing here "
                  "exercises #1871\n" + log[-600:])
            return False
        prot = read_protected_nets(pro_path_for_board(evidence(out, 'the routed board')))
        ok(prot.get('/sheetIO/_USB_D_P') == 'diff-pair'
           and prot.get('/sheetIO/_USB_D_N') == 'diff-pair',
           f"both members are recorded: {prot}")
        with open(evidence(js, '--json-out'), encoding='utf-8') as fh:
            doc = json.load(fh)
        ok(doc.get('unprotected_pairs') == [],
           f"and no routed pair is left out: {doc.get('unprotected_pairs')}")
        ok('Protected nets (not ripped by later steps): 2 recorded' in log,
           "the run says it recorded them")
    return True


def main():
    if not end_to_end():
        return 1
    units()
    if FAILS:
        print(f"\nFAILED ({len(FAILS)}):")
        for f in FAILS:
            print(f"  - {f}")
        return 1
    print("\nALL PASSED")
    return 0


if __name__ == '__main__':
    sys.exit(main())
