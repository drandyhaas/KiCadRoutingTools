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
(`pair_pn_shortfall`: each member's tracks and vias against the partner's
tracks, vias and pads, priced as check_drc grades the pair): with no shortfall left the pair is admitted, with one
it stays out. Every routed pair the record leaves out is a row of
`JSON_SUMMARY.unprotected_pairs` and a console line.

Rows:
  1. end to end, the haasoscope test chain's fanned-out board
     (`routed_output`), all 56 `*lvds*` pairs: the fallback really ran
     (`[least self-graze; no clean layer]`, on three of them), both members
     of every such pair that shipped are recorded `diff-pair` (one that took
     it and then failed has nothing to record), and
     `unprotected_pairs` is []. (orangecrab_ext_pll's USB pair was the witness
     until #1441 corrected the coupled middle's search, which then finds that
     pair a layer it couples on cleanly; the 13 `*lvds_rx1_*` pairs were,
     until the #1055 polarity crossover routed most of them coupled.)
  2. unit: a pair result carrying `selfgraze` whose P and N tracks stand
     0.1 mm apart under a 0.25 mm clearance stays out, and is listed with
     the reason `self-graze fallback` and its shortfall count;
  3. unit: the same pair 1.0 mm apart is admitted, and listed nowhere;
  4. unit: a via of one member 0.1 mm from the partner's track is a
     shortfall too;
  5. controls: `protection_candidates` with `selfgraze` and no judgement
     still returns {} (test_906_hybrid_pair_and_impedance_record's arm 1b),
     and a pair with no `selfgraze` is untouched by the judgement;
  6. unit: a P track 0.05 mm from an N PAD under 0.25 mm is a shortfall (and
     the same track 1.0 mm from it is not);
  7. unit: P and N tracks 0.3 mm apart under a 0.25 mm clearance and a
     0.4 mm #735 track rule are a shortfall: track against track is priced
     at the track rule, as check_drc grades it.
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
from synth import make_pad, make_seg, make_via, make_net, make_pcb  # noqa: E402
from kicad_parser import BoardInfo                     # noqa: E402

PY = [sys.executable, '-X', 'utf8']
BOARD = os.path.join(REPO, 'kicad_files', 'routed_output.kicad_pcb')
FAILS = []


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def pair_board(gap, via_gap=None, pad_gap=None):
    """P at y=0 and N a `gap` away (edge to edge, 0.2 mm tracks); with
    `via_gap`, a 0.5 mm via of P that far from N's track; with `pad_gap`, a
    0.5 mm square N pad that far beyond the end of P's track."""
    segs = [make_seg(0, 0, 10, 0, net_id=1), make_seg(0, 0.2 + gap, 10, 0.2 + gap, net_id=2)]
    pads = {1: [], 2: []}
    if pad_gap is not None:
        pads[2].append(make_pad(2, 10.0 + 0.1 + pad_gap + 0.25, 0.0, ref='J1',
                                num='2', net_name='X_N'))
    vias = []
    if via_gap is not None:
        vias.append(make_via(12.0, 0.2 + gap, net_id=1))
        segs.append(make_seg(10, 0.2 + gap, 12.0 - 0.25 - via_gap - 0.1, 0.2 + gap, net_id=2))
    pcb = make_pcb(nets={1: make_net(1, 'X_P'), 2: make_net(2, 'X_N')}, segments=segs,
                   vias=vias, pads_by_net=pads,
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
                 'pn_shortfall_items': short.get('X')}], f"and is listed: {rows}")

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

    print("6. a track against the partner's pad")
    from route_diff import pair_pn_shortfall
    pcb, pairs = pair_board(1.0, pad_gap=0.05)
    res = results()
    short = judge_selfgraze_on_copper(res, pcb, cfg, pairs)
    ok(pair_pn_shortfall(pcb, pairs[0][1], cfg) == 1 and short == {'X': 1}
       and protection_candidates(res, pcb, pairs=pairs) == {},
       f"P's track 0.05 mm from N's pad is a shortfall: {short}")
    pcb, pairs = pair_board(1.0, pad_gap=1.0)
    ok(pair_pn_shortfall(pcb, pairs[0][1], cfg) == 0,
       "...and 1.0 mm from it is not")

    print("7. track against track at the #735 track rule")
    pcb, pairs = pair_board(0.3)
    ok(pair_pn_shortfall(pcb, pairs[0][1], cfg) == 0,
       "0.3 mm apart under 0.25 mm and no track rule: clear")
    ruled = GridRouteConfig(clearance=0.25, layers=['F.Cu', 'B.Cu'])
    ruled.track_clearances = {1: 0.4, 2: 0.4}
    ok(ruled.pair_clearance(1, 2, 'F.Cu') == 0.25
       and ruled.pair_clearance(1, 2, 'F.Cu', kind='track') == 0.4,
       "the config prices the pair 0.25 on the layer and 0.4 track to track")
    ok(pair_pn_shortfall(pcb, pairs[0][1], ruled) == 2,
       f"under a 0.4 mm track rule both tracks are short: "
       f"{pair_pn_shortfall(pcb, pairs[0][1], ruled)}")


def end_to_end():
    print("1. haasoscope's lvds pairs through the self-graze fallback")
    import re
    from protected_nets import read_protected_nets, pro_path_for_board
    with tempfile.TemporaryDirectory() as td:
        board, out, js = (os.path.join(td, n) for n in
                          ('in.kicad_pcb', 'hs.kicad_pcb', 'hs.json'))
        check(PY + [tool('copy_board.py'), evidence(BOARD, 'the bundled board'), board],
              accept=True)
        r = check(PY + [tool('route_diff.py'), board, out,
                        '--nets', '*lvds*', '--swappable-nets', '*lvds*',
                        '--impedance', '100', '--proximity-heuristic-factor', '0.0',
                        '--clearance', '0.1', '--via-size', '0.3', '--via-drill', '0.2',
                        '--track-width', '0.1',
                        '--layers', 'F.Cu', 'In1.Cu', 'In2.Cu', 'In3.Cu', 'B.Cu',
                        '--json-out', js], accept=True, timeout=1800)
        log = (r.stdout or '') + (r.stderr or '')
        fell, pair = [], None
        for line in log.splitlines():
            m = re.match(r'\[\d+/\d+\] Routing diff pair (\S+)', line)
            if m:
                pair = m.group(1)
            elif '[least self-graze; no clean layer]' in line and pair:
                fell.append(pair)
        if not fell:
            print("  [BROKEN TEST] the self-graze fallback did not run: nothing here "
                  "exercises #1871\n" + log[-600:])
            return False
        prot = read_protected_nets(pro_path_for_board(evidence(out, 'the routed board')))
        with open(evidence(js, '--json-out'), encoding='utf-8') as fh:
            doc = json.load(fh)
        # A pair can take the fallback and still end failed (no result, no
        # copper to record); #1871 is about the ones that ship.
        shipped = [p for p in sorted(set(fell))
                   if p in set(doc.get('routed_diff_pairs') or [])]
        if not shipped:
            print("  [BROKEN TEST] no pair that took the self-graze fallback shipped: "
                  f"nothing here exercises #1871 (fell back: {sorted(set(fell))})")
            return False
        ok(all(prot.get(p + '_P') == 'diff-pair' and prot.get(p + '_N') == 'diff-pair'
               for p in shipped),
           f"both members of every shipped fallback pair are recorded: {shipped}")
        ok(doc.get('unprotected_pairs') == [],
           f"and no routed pair is left out: {doc.get('unprotected_pairs')}")
        # #1361: a pair shipped uncoupled is protected while its members still
        # run alongside, so it is recorded too without being credited routed
        alongside = [p for p in (doc.get('uncoupled_diff_pairs') or [])
                     if prot.get(p + '_P') == 'diff-pair' and prot.get(p + '_N') == 'diff-pair']
        n = 2 * (len(doc.get('routed_diff_pairs') or []) + len(alongside))
        ok(f'Protected nets (not ripped by later steps): {n} recorded' in log,
           f"the run says it recorded all {n}")
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
