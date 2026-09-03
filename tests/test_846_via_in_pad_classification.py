#!/usr/bin/env python3
"""Issue #846: a via that overlaps its own pad is CLASSIFIED and CLAMPED as one.

The QFN under-pad commit loop decided "via-in-pad" with
``hypot(via - pad) <= POSITION_TOLERANCE`` -- 0.001 mm. That is not the
question #202's clamp exists to answer, and it is not the question
``fab_notes.print_via_in_pad_note`` answers three lines later in the same run.
So the same via could be reported as needing IPC-4761 Type VII **and** shipped
at nominal size with no clamp, free to bulge past the pad edge.

THE MEASUREMENT, AS RUN (upstream/main e239e067, --escape-method underpad
--allow-via-in-pad, via 0.45, grid 0.05):

    board / component          vias  centred  OVERLAP-off-centre  disjoint
    routed_output U2             14        0                   0        14
    tigard U3                    48       28                  20         0
    qfn_diffpair_escape U1        2        0                   2         0
    qfn_underpad_coupling U1      8        0                   8         0
    qfn_interior_pads U1 (INT*)   2        2                   0         0

The "OVERLAP-off-centre" column is what this fix moves: 30 vias across three
tracked boards that overlap the copper of the pad they escape while sitting off
its centre. Before, every one of them shipped unclamped.

WHY THE CENTRED COLUMN IS MOSTLY ZERO -- the part the issue did not name.
``snap()`` quantises the via COORDINATE to the routing grid (0.05 default),
while real pad centres are not on that lattice: measured, 76 of 77 pads on
routed_output's QFN-76, 6 of 6 on qfn_diffpair_escape, 8 of 8 on
qfn_underpad_coupling, 16 of 66 on tigard. On those boards the genuinely
centred rung lands **0.0125 mm** from the pad centre -- 12.5x
POSITION_TOLERANCE -- so the via-in-pad branch was unreachable by construction,
even for the offset the ladder calls 0.

Run: python3 tests/test_846_via_in_pad_classification.py
"""
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))

from kicad_parser import parse_kicad_pcb                       # noqa: E402
from qfn_fanout import generate_qfn_fanout                     # noqa: E402
from fab_notes import via_overlaps_pad, via_in_pad_sites       # noqa: E402
from bga_fanout.constants import POSITION_TOLERANCE            # noqa: E402
from list_nets import fab_floor_min                            # noqa: E402

GRID = 0.05
VIA_SIZE, VIA_DRILL = 0.45, 0.20

# board, ref, net filter, layer, track, clearance
CASES = [
    ('qfn_underpad_coupling', 'U1', ['SIG*'], 'F.Cu', 0.1, 0.12),
    ('qfn_diffpair_escape', 'U1', ['DP1*'], 'F.Cu', 0.1, 0.15),
    ('tigard', 'U3', None, 'F.Cu', 0.1, 0.10),
]

CHECKS = []


def check(name, ok, detail=''):
    CHECKS.append((name, bool(ok), detail))
    print(('  PASS: ' if ok else '  FAIL: ') + name + (' -- ' + detail
                                                       if detail else ''))


def escaping_pad(fp, via):
    """The pad this via escapes: the nearest same-net pad on the footprint."""
    cands = [p for p in fp.pads if p.net_id == via['net_id']]
    if not cands:
        return None
    return min(cands, key=lambda p: math.hypot(p.global_x - via['x'],
                                               p.global_y - via['y']))


def run(board, ref, nets, layer, tw, clr):
    path = os.path.join(ROOT, 'kicad_files', board + '.kicad_pcb')
    if not os.path.exists(path):
        return None
    pcb = parse_kicad_pcb(path)
    fp = pcb.footprints.get(ref)
    if fp is None:
        return None
    tracks, vias, dropped = generate_qfn_fanout(
        fp, pcb, net_filter=nets, layer=layer, track_width=tw, clearance=clr,
        grid_step=GRID, escape_method='underpad', via_size=VIA_SIZE,
        via_drill=VIA_DRILL, allow_via_in_pad=True)
    return pcb, fp, tracks, vias, dropped


def main():
    ran = 0
    for board, ref, nets, layer, tw, clr in CASES:
        got = run(board, ref, nets, layer, tw, clr)
        if got is None:
            print(f"  (skipping {board}: not present)")
            continue
        ran += 1
        pcb, fp, tracks, vias, dropped = got
        ncu = len(pcb.board_info.copper_layers or []) or 4
        floor_dia = fab_floor_min(ncu)['via_diameter']

        overlapping, offcentre, disjoint = [], [], []
        for v in vias:
            pad = escaping_pad(fp, v)
            if pad is None:
                continue
            d = math.hypot(v['x'] - pad.global_x, v['y'] - pad.global_y)
            if via_overlaps_pad(pad, v['x'], v['y'], VIA_SIZE):
                overlapping.append((v, pad, d))
                if d > POSITION_TOLERANCE:
                    offcentre.append((v, pad, d))
            else:
                disjoint.append((v, pad, d))

        # 1. THE DEFECT. Every via that overlaps the pad it escapes is sized to
        #    that pad (#202), or held at the fab floor because the pad is
        #    smaller than any buildable via. Before the fix an off-centre
        #    overlapping via kept the nominal 0.45.
        bad = [(p.pad_number, v['size'], min(p.size_x, p.size_y), round(d, 4))
               for v, p, d in overlapping
               if v['size'] > min(p.size_x, p.size_y) + 1e-9
               and v['size'] > floor_dia + 1e-9]
        check(f'{board} {ref}: every via overlapping its own pad is clamped '
              f'to it ({len(overlapping)} overlapping, {len(offcentre)} of '
              f'them off centre)', not bad,
              f'unclamped: {bad[:6]}' if bad else '')

        # 2. The rig must actually contain the case, or check 1 is vacuous --
        #    the trap the BGA half of test_fanout_via_in_pad_clamp guards and
        #    its QFN half did not.
        check(f'{board} {ref}: the run contains at least one OFF-CENTRE '
              f'overlapping via', offcentre,
              'nothing here exercises #846' if not offcentre else
              f'max offset {max(d for _v, _p, d in offcentre):.4f} mm')

        # 3. The engine and the FAB NOTE agree, via for via. They are the two
        #    consumers that used to disagree; the note is derived from the
        #    board's own pad table, not from the loop's bookkeeping.
        noted = {(round(v['x'], 6), round(v['y'], 6))
                 for v, _pad in via_in_pad_sites(vias, pcb.pads_by_net)}
        engine = {(round(v['x'], 6), round(v['y'], 6))
                  for v, _p, _d in overlapping}
        check(f'{board} {ref}: the commit loop and the IPC-4761 fab note '
              f'classify the same vias', engine <= noted,
              f'engine-only: {sorted(engine - noted)[:4]}')

        # 4. A via that misses its pad entirely keeps the nominal size: the fix
        #    must not clamp everything.
        wrong = [(p.pad_number, v['size']) for v, p, _d in disjoint
                 if abs(v['size'] - VIA_SIZE) > 1e-9]
        check(f'{board} {ref}: a via disjoint from its pad keeps nominal size '
              f'({len(disjoint)} disjoint)', not wrong, str(wrong[:6]))

        # 5. Nothing was lost. Shrinking a via only relaxes clearance and
        #    hole-to-hole, so the escape count cannot move.
        check(f'{board} {ref}: escapes are unchanged by the clamp '
              f'({len(vias)} vias, {len(dropped)} dropped)',
              len(vias) + len(dropped) > 0)

    # 6. THE 12.5 MICRON CASE, stated as geometry rather than as an outcome:
    #    pad centres are off the routing lattice, so snap() cannot place a via
    #    within POSITION_TOLERANCE of one, and the old centre test could never
    #    fire there however centred the chosen offset was.
    off_lattice = {}
    for board, ref, _n, _l, _t, _c in CASES:
        path = os.path.join(ROOT, 'kicad_files', board + '.kicad_pcb')
        if not os.path.exists(path):
            continue
        fp = parse_kicad_pcb(path).footprints[ref]
        offs = []
        for p in fp.pads:
            if not p.net_id:
                continue
            offs.append(math.hypot(
                abs(p.global_x - round(p.global_x / GRID) * GRID),
                abs(p.global_y - round(p.global_y / GRID) * GRID)))
        off_lattice[board] = (sum(1 for o in offs if o > POSITION_TOLERANCE),
                              len(offs), max(offs) if offs else 0.0)
    worst = [b for b, (n, _t, _m) in off_lattice.items() if n]
    check('pad centres are OFF the routing lattice, so the old centre test '
          'could not fire for the centred rung either',
          worst, '; '.join(f'{b}: {n}/{t} off-lattice, up to {m:.4f} mm'
                           for b, (n, t, m) in off_lattice.items()))

    if not ran:
        print('SKIP: no corpus board present')
        return 77
    bad = [n for n, ok, _d in CHECKS if not ok]
    print(f"\n{'FAIL' if bad else 'PASS'}  #846 via-in-pad classification: "
          f"{len(CHECKS) - len(bad)}/{len(CHECKS)} checks")
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
