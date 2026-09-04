#!/usr/bin/env python3
"""Issue #672: sub-cell same-net slivers left by rip/restore/prune churn must
not ship as real seg-seg violations against foreign copper.

Two defects, each pinned with a negative control so the test cannot pass
vacuously:

1. SELF-PAIR in every soft-joint detector. A "soft joint" is two DANGLING free
   ends of one net whose caps overlap (gap in (SOFT_JOINT_MIN_GAP, cap)). None
   of the four detectors excluded the two ends of ONE segment, so a lone
   sliver shorter than its cap -- a 0.02mm close_soft_joints bridge whose
   neighbours were ripped or pruned -- paired with ITSELF. Consequences:
     * ``_restore_soft_joint_bridges`` read a clean removal of the neighbours
       as "created a soft joint" and put a DEAD neighbour back (the very
       segment a pass had just removed for grazing foreign copper);
     * ``close_soft_joints`` offered to bridge the sliver to itself;
     * ``check_drc`` / ``check_weird`` reported a phantom
       ``segment-endpoint-gap`` on a lone segment.
   Negative control: two distinct segments whose free ends are 0.02mm apart
   still pair, and removing a genuine bridge is still reverted (#319).

2. The #473 unfinished-net exemption kept EVERY piece of a net still being
   retried -- including a 0.02mm isolated sliver 0.054mm from a foreign
   track. ``sweep_dead_ends(sliver_eps=grid_step)`` now drops sub-CELL dead
   ends on protected nets when the authoritative connectivity check grades
   the net no worse; the epsilon is one routing cell because no A* span is
   shorter, so such a piece can only be debris, and a stub shorter than a
   cell offers no landing its root does not. Negative controls: a real
   landing stub (0.5mm, longer than a cell) on the same protected net stays;
   a sub-cell piece with no free end (BRIDGING two neighbours, or pad-to-
   track) stays; a sliver that is the net's only link between two pads stays
   (connectivity gate); ``sliver_eps=0`` restores the old keep-everything
   behaviour.

    python3 tests/test_672_rip_fragments.py
"""
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
for _d in ('', 'py_router', 'py_tools', 'rust_router'):
    sys.path.insert(0, os.path.join(_ROOT, _d))
sys.path.insert(0, _HERE)

from kicad_parser import BoardInfo  # noqa: E402
from synth import make_pad, make_seg, make_pcb, make_net  # noqa: E402
from pcb_modification import (_soft_joint_pairs, _restore_soft_joint_bridges,  # noqa: E402
                              sweep_dead_ends, _prune_sub_cell_slivers)

W = 0.0762          # the orangecrab track width the issue was observed at
GRID = 0.05         # its routing cell
NET = 7
LAYER = 'F.Cu'
FAILS = []


def check(cond, msg):
    print(('  ok   ' if cond else '  FAIL ') + msg)
    if not cond:
        FAILS.append(msg)


def seg(x1, y1, x2, y2, net=NET, w=W, layer=LAYER):
    return make_seg(x1, y1, x2, y2, net_id=net, width=w, layer=layer)


def _len(s):
    return math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)


# --------------------------------------------------------------------------
print("1. soft-joint detectors: one segment's two ends are not a joint")
A = seg(160.0, 102.87, 163.99, 102.87)
B = seg(163.99, 102.87, 164.01, 102.87)          # the 0.02mm bridge
C = seg(164.01, 102.87, 168.0, 102.87)
check(_len(B) < W, f"fixture: bridge {_len(B):.3f}mm is shorter than its cap {W}")
check(_soft_joint_pairs([B], [], []) == set(),
      "a lone sub-cap sliver forms NO soft joint with itself")
check(len(_soft_joint_pairs([A, C], [], [])) == 1,
      "NEGATIVE CONTROL: two distinct dangles 0.02mm apart still pair")
kept, removed = _restore_soft_joint_bridges([B], [A, C], [], [])
check(len(kept) == 1 and len(removed) == 2,
      "removing both neighbours of a sliver no longer resurrects one of them")
kept, removed = _restore_soft_joint_bridges([A, C], [B], [], [])
check(len(kept) == 3 and not removed,
      "NEGATIVE CONTROL (#319): removing a genuine bridge is still reverted")

# The same rule in check_drc / check_weird, through their real entry points.
import tempfile  # noqa: E402
from synth import board_text, write_board  # noqa: E402


def _board_with(segs):
    """A KiCad board text holding `segs` on net 1, written to a temp file
    (run_drc parses text). `board_text` takes the body as a TEXT blob."""
    segtxt = ''.join(
        f'\t(segment (start {s.start_x} {s.start_y}) (end {s.end_x} {s.end_y}) '
        f'(width {s.width}) (layer "{s.layer}") (net 1))\n' for s in segs)
    return write_board(board_text(segtxt, nets=(0, 1)),
                       os.path.join(tempfile.mkdtemp(prefix='t672_'), 'b.kicad_pcb'))


try:
    from check_drc import run_drc
    # Gap 0.05: above COINCIDENCE_TOL (0.02, which run_drc demotes to a
    # warning) and below the 0.1 cap, so a pair here is a returned VIOLATION.
    lone = _board_with([seg(10.0, 10.0, 10.05, 10.0, net=1, w=0.1)])
    two = _board_with([seg(9.0, 10.0, 9.975, 10.0, net=1, w=0.1),
                       seg(10.025, 10.0, 11.0, 10.0, net=1, w=0.1)])
    r_lone = run_drc(lone, clearance=0.1, quiet=True)
    r_two = run_drc(two, clearance=0.1, quiet=True)

    def _sj(r):
        return [x for x in (r or []) if isinstance(x, dict)
                and x.get('type') == 'segment-endpoint-gap']
    check(not _sj(r_lone), "check_drc: no phantom soft joint on a lone sliver")
    check(len(_sj(r_two)) == 1,
          "NEGATIVE CONTROL: check_drc still reports a real soft joint")
except Exception as e:  # the detector must be reachable, or this half is vacuous
    check(False, f"check_drc soft-joint probe did not run: {type(e).__name__}: {e}")

# --------------------------------------------------------------------------
print("2. sweep_dead_ends: sub-cell debris on a PROTECTED net")
FOREIGN = 9


def _fixture():
    """Net NET: pad P1 at x=150 -> trunk -> pad P2 at x=170, still OPEN
    (a third pad P3 at (160,110) is unrouted, so the net is protected). Debris
    on it: an isolated 0.02mm sliver at y=102.87 (0.13mm from a foreign
    track at y=103.00, i.e. a 0.054mm graze at W=0.0762 / clearance 0.0889)
    and a second isolated 0.03mm piece. (A spur off a trunk's INTERIOR that
    is shorter than the trunk's half-width is not debris: its free end lies
    inside the trunk's copper and _point_anchored counts it as landed, so
    no dead-end pass sees it.) Legit copper that must survive: a 0.5mm
    landing stub off the trunk, a 0.04mm piece BRIDGING two trunk halves, a
    0.03mm piece touching P2, and a 0.02mm piece that is the ONLY link
    between two halves of a second branch reaching P1 -- the connectivity
    gate must refuse that one."""
    p1 = make_pad(NET, 150.0, 100.0, ref='U1', num='1', size_x=0.3, size_y=0.3)
    p2 = make_pad(NET, 170.0, 100.0, ref='U2', num='1', size_x=0.3, size_y=0.3)
    p3 = make_pad(NET, 160.0, 110.0, ref='U3', num='1', size_x=0.3, size_y=0.3)
    p4 = make_pad(NET, 150.0, 96.0, ref='U4', num='1', size_x=0.3, size_y=0.3)
    trunk_a = seg(150.0, 100.0, 159.98, 100.0)
    bridge = seg(159.98, 100.0, 160.02, 100.0)              # 0.04, bridging
    trunk_b = seg(160.02, 100.0, 169.97, 100.0)
    pad_touch = seg(169.97, 100.0, 170.0, 100.0)             # 0.03, touches P2
    landing = seg(165.0, 100.0, 165.0, 100.5)                # 0.5mm spur: a landing site
    spur = seg(156.0, 101.5, 156.03, 101.5)                  # 0.03 isolated: debris
    sliver = seg(163.99, 102.87, 164.01, 102.87)             # the issue's fragment
    # second branch: P1 -> (150,98) ... 0.02 link ... -> P4 at (150,96)
    br1 = seg(150.0, 100.0, 150.0, 98.01)
    link = seg(150.0, 98.01, 150.0, 97.99)                   # 0.02, load-bearing
    br2 = seg(150.0, 97.99, 150.0, 96.0)
    foreign = seg(163.30, 103.0, 168.60, 103.0, net=FOREIGN)
    segs = [trunk_a, bridge, trunk_b, pad_touch, landing, spur, sliver,
            br1, link, br2, foreign]
    bi = BoardInfo(layers={}, copper_layers=['F.Cu', 'B.Cu'],
                   board_bounds=(140.0, 90.0, 180.0, 120.0))
    pcb = make_pcb(board_info=bi, segments=segs,
                   nets={NET: make_net(NET, 'RAM_ODT', pads=[p1, p2, p3, p4]),
                         FOREIGN: make_net(FOREIGN, 'RAM_D11')},
                   pads_by_net={NET: [p1, p2, p3, p4], FOREIGN: []},
                   zones=[])
    names = {'trunk_a': trunk_a, 'bridge': bridge, 'trunk_b': trunk_b,
             'pad_touch': pad_touch, 'landing': landing, 'spur': spur,
             'sliver': sliver, 'br1': br1, 'link': link, 'br2': br2,
             'foreign': foreign}
    return pcb, names


def _conn(pcb):
    from check_connected import check_net_connectivity
    r = check_net_connectivity(NET, [s for s in pcb.segments if s.net_id == NET],
                               [], pcb.pads_by_net[NET], [], pcb_data=pcb)
    return len(r.get('disconnected_pads') or []), r.get('num_components') or 1


pcb, n = _fixture()
results = [{'new_segments': [n['sliver'], n['spur'], n['landing']], 'new_vias': []}]
before = _conn(pcb)
check(before[0] == 1, f"fixture: net is OPEN before (P3 unrouted): {before}")
segs_removed, vias_removed, strip = sweep_dead_ends(
    results, pcb, scope_net_ids={NET}, protect_net_ids={NET}, sliver_eps=GRID)
left = {id(s) for s in pcb.segments}
check(id(n['sliver']) not in left, "isolated 0.02mm sliver on the protected net is dropped")
check(id(n['spur']) not in left, "second isolated 0.03mm piece is dropped")
check(id(n['landing']) in left, "NEGATIVE CONTROL: the 0.5mm landing stub stays (protection holds)")
check(id(n['bridge']) in left, "NEGATIVE CONTROL: a sub-cell piece BRIDGING two neighbours stays")
check(id(n['pad_touch']) in left, "NEGATIVE CONTROL: a sub-cell piece touching a pad stays")
check(id(n['link']) in left, "NEGATIVE CONTROL: a sub-cell piece that is the only link stays")
check(id(n['foreign']) in left, "foreign copper untouched")
check(_conn(pcb) == before, f"connectivity unchanged by the sweep: {before} -> {_conn(pcb)}")
check(all(id(s) in left for s in results[0]['new_segments']),
      "write-list mirrors the board (dropped pieces left the result too)")
check(segs_removed == 2, f"reported count is the two pieces ({segs_removed})")

pcb2, n2 = _fixture()
sweep_dead_ends([{'new_segments': [n2['sliver']], 'new_vias': []}], pcb2,
                scope_net_ids={NET}, protect_net_ids={NET}, sliver_eps=0.0)
check(id(n2['sliver']) in {id(s) for s in pcb2.segments},
      "NEGATIVE CONTROL: sliver_eps=0 keeps everything on a protected net (old behaviour)")

pcb3, n3 = _fixture()
sweep_dead_ends([{'new_segments': [n3['sliver']], 'new_vias': []}], pcb3,
                scope_net_ids={NET}, protect_net_ids=None, sliver_eps=GRID)
l3 = {id(s) for s in pcb3.segments}
check(id(n3['sliver']) not in l3 and id(n3['landing']) not in l3,
      "unprotected net: the full sweep still takes the sliver AND the dead landing stub")

# The helper alone, on the load-bearing link: refused by the gate.
pcb4, n4 = _fixture()
net_segs = [s for s in pcb4.segments if s.net_id == NET]
kept, removed = _prune_sub_cell_slivers(NET, net_segs, [], pcb4.pads_by_net[NET],
                                        [], GRID, pcb_data=pcb4)
check(n4['link'] in kept and n4['sliver'] in removed and n4['spur'] in removed,
      "_prune_sub_cell_slivers: keeps the only-link piece, drops the two debris pieces")

# ---------------------------------------------------------------------------
# The PIPELINE default: the trim is OPT-IN, and the knob is really read.
# Measured on orangecrab (paired route step, same input board): trimming a
# still-being-retried net's sub-cell debris cost RAM_UDQS+ (verdict 9 -> 10),
# while the self-pair fix alone reproduced the base copper exactly. So the
# pipeline must pass 0.0 unless KICAD_SLIVER_TRIM is set. Observe the VALUE
# the call site computes, not the source text: a source grep would pass on a
# gate wired to the wrong knob.
import importlib
_seen = {}
import pcb_modification as _pm
_real_sweep = _pm.sweep_dead_ends


def _spy(*a, **kw):
    _seen['eps'] = kw.get('sliver_eps')
    return _real_sweep(*a, **kw)


import cleanup_pipeline as _cp
import env_knobs as _ek
for _want, _val in (('off', None), ('on', '1')):
    if _val is None:
        os.environ.pop('KICAD_SLIVER_TRIM', None)
    else:
        os.environ['KICAD_SLIVER_TRIM'] = _val
    _ek.refresh()
    importlib.reload(_cp)
    _cp.sweep_dead_ends = _spy
    _seen.clear()
    pcb5, n5 = _fixture()

    from routing_config import GridRouteConfig as _GRC
    cfg5 = _GRC(track_width=0.15, clearance=0.15, via_size=0.4,
                via_drill=0.2, grid_step=GRID, layers=['F.Cu', 'B.Cu'])
    try:
        _cp.run_post_route_cleanup([], pcb5, {NET}, cfg5,
                                   protect_net_ids={NET})
    except Exception as _e:            # the pipeline wants more of a board
        _seen.setdefault('exc', repr(_e))
    if 'eps' not in _seen:             # a vacuous pass is not a pass
        check(False, f"pipeline reached sweep_dead_ends ({_want}) "
                     f"[aborted: {_seen.get('exc')}]")
    elif _want == 'off':
        check(_seen.get('eps') in (0.0, 0),
              f"pipeline default: sliver trim OFF (eps={_seen.get('eps')!r})")
    else:
        check(bool(_seen.get('eps')),
              f"KICAD_SLIVER_TRIM=1 is READ: the pipeline passes a real "
              f"epsilon (eps={_seen.get('eps')!r})")
os.environ.pop('KICAD_SLIVER_TRIM', None)
_ek.refresh()

print()
if FAILS:
    print(f"FAILED ({len(FAILS)}):")
    for f in FAILS:
        print("  - " + f)
    sys.exit(1)
print("test_672_rip_fragments: all checks passed")
