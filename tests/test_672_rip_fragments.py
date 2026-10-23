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

2. ``sweep_dead_ends`` on the same sliver: the #473 unfinished-net
   exemption keeps every piece of a PROTECTED net, and an unprotected net
   loses the sliver and its dead landing stub to the full sweep.

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
                              sweep_dead_ends)

W = 0.0762          # the orangecrab track width the issue was observed at
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
print("2. sweep_dead_ends: sub-cell debris, protected and unprotected")
FOREIGN = 9


def _fixture():
    """Net NET: pad P1 at x=150 -> trunk -> pad P2 at x=170, still OPEN
    (a third pad P3 at (160,110) is unrouted, so the net is protected). Debris
    on it: an isolated 0.02mm sliver at y=102.87 (0.13mm from a foreign
    track at y=103.00, i.e. a 0.054mm graze at W=0.0762 / clearance 0.0889)
    and a second isolated 0.03mm piece. Other copper on it: a 0.5mm
    landing stub off the trunk, a 0.04mm piece BRIDGING two trunk halves, a
    0.03mm piece touching P2, and a 0.02mm piece that is the ONLY link
    between two halves of a second branch reaching P1."""
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


pcb2, n2 = _fixture()
sweep_dead_ends([{'new_segments': [n2['sliver']], 'new_vias': []}], pcb2,
                scope_net_ids={NET}, protect_net_ids={NET})
check(id(n2['sliver']) in {id(s) for s in pcb2.segments},
      "a protected net keeps everything (the #473 exemption)")

pcb3, n3 = _fixture()
sweep_dead_ends([{'new_segments': [n3['sliver']], 'new_vias': []}], pcb3,
                scope_net_ids={NET}, protect_net_ids=None)
l3 = {id(s) for s in pcb3.segments}
check(id(n3['sliver']) not in l3 and id(n3['landing']) not in l3,
      "unprotected net: the full sweep still takes the sliver AND the dead landing stub")

print()
if FAILS:
    print(f"FAILED ({len(FAILS)}):")
    for f in FAILS:
        print("  - " + f)
    sys.exit(1)
print("test_672_rip_fragments: all checks passed")
