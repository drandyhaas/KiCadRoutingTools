#!/usr/bin/env python3
"""Unit tests for prune_dead_end_segments (issue #84 dead-end sweep).

    python3 tests/test_dead_end_prune.py
"""
import os
import sys
from types import SimpleNamespace

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_parser import Segment, Via
from pcb_modification import prune_dead_end_segments


def _seg(x1, y1, x2, y2, layer='F.Cu', net=5):
    return Segment(start_x=x1, start_y=y1, end_x=x2, end_y=y2, width=0.2,
                   layer=layer, net_id=net)


def _via(x, y, net=5):
    return Via(x=x, y=y, size=0.5, drill=0.3, layers=['F.Cu', 'B.Cu'], net_id=net)


def _pad(x, y, sx=0.5, sy=0.5, layers=('F.Cu',)):
    return SimpleNamespace(global_x=x, global_y=y, size_x=sx, size_y=sy, layers=list(layers))


def run():
    fails = []

    def check(name, cond):
        if not cond:
            fails.append(name)

    # 1. pad -- A -- B(dead): the B spur is removed, the pad leg kept.
    p = _pad(0, 0)
    main = _seg(0, 0, 5, 0)          # pad(0,0) -> junction(5,0)
    spur = _seg(5, 0, 5, 3)          # junction(5,0) -> dead(5,3)
    kept, removed = prune_dead_end_segments([main, spur], pads=[p])
    check("1 spur removed", removed == [spur])
    check("1 main kept", kept == [main])

    # 2. spur chain unwinds fully (two dead segments in a row).
    p = _pad(0, 0)
    main = _seg(0, 0, 5, 0)
    s1 = _seg(5, 0, 8, 0)            # junction -> mid-dead
    s2 = _seg(8, 0, 8, 4)            # mid-dead -> dead tip
    kept, removed = prune_dead_end_segments([main, s1, s2], pads=[p])
    check("2 both spur segs removed", set(map(id, removed)) == {id(s1), id(s2)})
    check("2 main kept", kept == [main])

    # 3. pad-to-pad path is never touched (both ends anchored, mid is a junction).
    pa, pb = _pad(0, 0), _pad(10, 0)
    a = _seg(0, 0, 5, 0)
    b = _seg(5, 0, 10, 0)
    kept, removed = prune_dead_end_segments([a, b], pads=[pa, pb])
    check("3 nothing removed", removed == [])
    check("3 both kept", len(kept) == 2)

    # 4. a via anchors a layer transition: neither leg is a dead end.
    v = _via(5, 0)
    pa, pb = _pad(0, 0, layers=('F.Cu',)), _pad(5, 5, layers=('B.Cu',))
    top = _seg(0, 0, 5, 0, layer='F.Cu')      # pad -> via (F.Cu)
    bot = _seg(5, 0, 5, 5, layer='B.Cu')      # via -> pad (B.Cu)
    kept, removed = prune_dead_end_segments([top, bot], vias=[v], pads=[pa, pb])
    check("4 via-anchored legs kept", removed == [])

    # 5. a T-tap landing in the middle of a kept trace is anchored, not a dead end.
    pa, pb = _pad(0, 0), _pad(10, 0)
    trunk_a = _seg(0, 0, 10, 0)
    pc = _pad(5, 4)
    tap = _seg(5, 0, 5, 4)          # lands mid-trunk (5,0), other end at pad (5,4)
    kept, removed = prune_dead_end_segments([trunk_a, tap], pads=[pa, pb, pc])
    check("5 T-tap kept (lands on trunk + pad)", removed == [])

    # 6. a genuinely isolated fragment (both ends free) is removed.
    frag = _seg(20, 20, 21, 21)
    kept, removed = prune_dead_end_segments([frag])
    check("6 isolated fragment removed", removed == [frag] and kept == [])

    # 7. anchor_segments count for junctions but are never themselves removed.
    pa = _pad(0, 0)
    original = _seg(0, 0, 5, 0)      # original copper (anchor only)
    routed = _seg(5, 0, 8, 0)        # routed spur off the original -> dead at (8,0)
    kept, removed = prune_dead_end_segments([routed], anchor_segments=[original], pads=[pa])
    check("7 routed spur removed", removed == [routed])
    check("7 original never returned", kept == [])

    # 8. keep_terminal_escapes: a pad-rooted dead chain (the whole antenna) is
    #    KEPT by default (per-commit pass, net may still be routing) but fully
    #    removed by the final sweep (keep_terminal_escapes=False).
    p = _pad(0, 0)
    leg = _seg(0, 0, 5, 0)           # pad -> dead end (no junction, no other copper)
    spur = _seg(5, 0, 5, 3)          # -> dead tip
    kept_c, removed_c = prune_dead_end_segments([leg, spur], pads=[p])
    check("8 conservative keeps pad escape leg", kept_c == [leg] and removed_c == [spur])
    kept_a, removed_a = prune_dead_end_segments([leg, spur], pads=[p],
                                                keep_terminal_escapes=False)
    check("8 aggressive removes whole dead antenna",
          set(map(id, removed_a)) == {id(leg), id(spur)} and kept_a == [])

    # 9. aggressive mode must NOT remove a live pad-to-pad path.
    pa9, pb9 = _pad(0, 0), _pad(10, 0)
    a9 = _seg(0, 0, 5, 0)
    b9 = _seg(5, 0, 10, 0)
    _, removed9 = prune_dead_end_segments([a9, b9], pads=[pa9, pb9],
                                          keep_terminal_escapes=False)
    check("9 aggressive keeps live pad-to-pad path", removed9 == [])

    print("=" * 60)
    if fails:
        for f in fails:
            print(f"  FAIL  {f}")
        print(f"\n{len(fails)} failure(s)")
        return 1
    print("  PASS  spur removal, chain unwinding, pad/via/T anchoring, isolated")
    print("        fragments, anchor-only segments, terminal-escape modes (9 cases)")
    print("\n9/9 checks passed")
    return 0


def run_gated():
    """_safe_prune_net must remove genuine dead spurs but never disconnect a pad
    (it gates the geometric prune against check_net_connectivity)."""
    from kicad_parser import Pad
    from pcb_modification import _safe_prune_net

    def pad(ref, x, y, net=5):
        return Pad(component_ref=ref, pad_number='1', global_x=x, global_y=y,
                   local_x=0, local_y=0, size_x=0.5, size_y=0.5, shape='circle',
                   layers=['F.Cu'], net_id=net, net_name='N')

    fails = []
    # pad A --a-- J --b-- pad B (live path), plus J --spur-- dead tip.
    pa, pb = pad('A', 0, 0), pad('B', 10, 0)
    a = _seg(0, 0, 5, 0)
    b = _seg(5, 0, 10, 0)
    spur = _seg(5, 0, 5, 3)
    kept, removed = _safe_prune_net(5, [a, b, spur], vias=[], pads=[pa, pb],
                                    zones=[], aggressive=True)
    if removed != [spur]:
        fails.append(f"gated: expected only spur removed, got {len(removed)}")
    if set(map(id, kept)) != {id(a), id(b)}:
        fails.append("gated: live path not preserved")

    # A pad reachable ONLY through a single trace: prune must keep it (removing it
    # would disconnect the pad), even though its far end looks like a dead end.
    pc = pad('C', 0, 0)
    only = _seg(0, 0, 4, 0)   # pad C -> (4,0) with nothing else: degree-1 free end
    kept2, removed2 = _safe_prune_net(5, [only], vias=[], pads=[pc],
                                      zones=[], aggressive=True)
    # Removing it strands no pad (the (4,0) end connects nothing, pad C is alone),
    # so aggressive MAY remove it -- but connectivity must be no worse either way.
    from check_connected import check_net_connectivity
    before = len(check_net_connectivity(5, [only], [], [pc], [])['disconnected_pads'])
    after = len(check_net_connectivity(5, kept2, [], [pc], [])['disconnected_pads'])
    if after > before:
        fails.append("gated: removal disconnected a pad")

    print("=" * 60)
    if fails:
        for f in fails:
            print("  FAIL ", f)
        print(f"\n{len(fails)} failure(s)")
        return 1
    print("  PASS  gated prune removes dead spur, preserves live path,")
    print("        never increases disconnected pads")
    return 0


def run_sweep():
    """sweep_dead_ends must never remove the input file's own copper by
    default: an authored stub is the user's geometry (a launch point for a
    later routing pass), not this run's leftover, and a run that routed the
    net without using it must not delete it. This run's own dead copper is
    still pruned, and prune_original=True restores the input-strip behavior
    for callers sweeping a file whose copper they themselves wrote
    (clean_plane_copper)."""
    from kicad_parser import Pad
    from pcb_modification import sweep_dead_ends

    def pad(ref, x, y, net=5):
        return Pad(component_ref=ref, pad_number='1', global_x=x, global_y=y,
                   local_x=0, local_y=0, size_x=0.5, size_y=0.5, shape='circle',
                   layers=['F.Cu'], net_id=net, net_name='N')

    def board():
        # pad A(0,0) with an AUTHORED input stub A -> (0,2) -> (0.05,3) the
        # route below never uses; this run's copper is a pad-to-pad trunk
        # A -> B plus its own dead spur off the trunk at (7,0) -> (7,3).
        pa, pb = pad('A', 0, 0), pad('B', 10, 0)
        stub1 = _seg(0, 0, 0, 2)
        stub2 = _seg(0, 2, 0.05, 3)
        trunk = _seg(0, 0, 10, 0)
        spur = _seg(7, 0, 7, 3)
        pcb = SimpleNamespace(segments=[stub1, stub2, trunk, spur], vias=[],
                              pads_by_net={5: [pa, pb]}, zones=[])
        results = [{'new_segments': [trunk, spur], 'new_vias': []}]
        return pcb, results, stub1, stub2, trunk, spur

    fails = []

    def check(name, cond):
        if not cond:
            fails.append(name)

    # 1. Default sweep: the routed spur is pruned, the authored stub survives
    #    and nothing is returned for the writer to strip from the input.
    pcb, results, stub1, stub2, trunk, spur = board()
    n_segs, n_vias, to_remove = sweep_dead_ends(results, pcb, {5})
    check("sweep default strips no input copper", to_remove == [])
    check("sweep default prunes the routed spur",
          n_segs == 1 and results[0]['new_segments'] == [trunk])
    check("sweep default no vias touched", n_vias == 0)

    # 2. prune_original=True: the old behavior for own-output sweeps -- the
    #    dead input stub chain is returned to strip, the spur still pruned.
    pcb, results, stub1, stub2, trunk, spur = board()
    n_segs, n_vias, to_remove = sweep_dead_ends(results, pcb, {5},
                                                prune_original=True)
    check("sweep prune_original strips the dead input stub",
          set(map(id, to_remove)) == {id(stub1), id(stub2)})
    check("sweep prune_original still prunes the routed spur",
          results[0]['new_segments'] == [trunk] and n_segs == 3)

    # 3. A routed leg that LAUNCHES from the authored stub's free end is
    #    anchored by it (input copper still counts for junctions), so a
    #    stub-launched route is never cut loose from its stub.
    pa, pb = pad('A', 0, 0), pad('B', 10, 3)
    stub1 = _seg(0, 0, 0, 2)
    stub2 = _seg(0, 2, 0.05, 3)
    launch = _seg(0.05, 3, 10, 3)     # routed: stub free end -> pad B
    pcb = SimpleNamespace(segments=[stub1, stub2, launch], vias=[],
                          pads_by_net={5: [pa, pb]}, zones=[])
    results = [{'new_segments': [launch], 'new_vias': []}]
    n_segs, _, to_remove = sweep_dead_ends(results, pcb, {5})
    check("sweep keeps a stub-launched route",
          n_segs == 0 and to_remove == [] and results[0]['new_segments'] == [launch])

    print("=" * 60)
    if fails:
        for f in fails:
            print("  FAIL ", f)
        print(f"\n{len(fails)} failure(s)")
        return 1
    print("  PASS  final sweep keeps authored input stubs (default), still")
    print("        prunes this run's dead copper, prune_original=True strips")
    return 0


if __name__ == "__main__":
    rc = run() or run_gated() or run_sweep()
    sys.exit(rc)
