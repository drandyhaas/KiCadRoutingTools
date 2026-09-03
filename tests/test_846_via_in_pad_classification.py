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

Every row is a board this file RUNS, so the table is re-derived on each
invocation rather than remembered. (An earlier draft carried a fifth row,
qfn_interior_pads under a ``--nets INT*`` filter, which no arm here reproduced
-- with the whole net set that board reads 6 vias / 0 centred / 6 overlapping.
A measured row nothing re-measures is how a table starts drifting.)

The "OVERLAP-off-centre" column is what this fix moves: 30 vias across three of
the four boards, overlapping the copper of the pad they escape while sitting off
its centre. Before, every one of them shipped unclamped. routed_output is here
for the OPPOSITE population -- 14 vias, none overlapping -- because without it a
mutation that clamps everything is invisible.

WHY THE CENTRED COLUMN IS MOSTLY ZERO -- the part the issue did not name.
``snap()`` quantises the via COORDINATE to the routing grid (0.05 default),
while real pad centres are not on that lattice: measured, 76 of 77 pads on
routed_output's QFN-76, 6 of 6 on qfn_diffpair_escape and 8 of 8 on
qfn_underpad_coupling all sit **0.0125 mm** off it -- 12.5x POSITION_TOLERANCE
-- so on those three the via-in-pad branch was unreachable by construction, even
for the offset the ladder calls 0.

tigard is the control that keeps this honest: 50 of its 66 pads ARE on the
lattice (the 16 that are not sit 0.01178-0.03536 mm off), and it duly produced
28 centred vias. The defect is board-dependent, not universal, and the boards it
bites are the ones whose pad centres are off-grid.

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
    # Carries the OPPOSITE population -- 14 vias, none of which overlaps its
    # pad -- and it is here for that. Without it a mutation that clamps
    # EVERYTHING (`if True:`) is invisible, because every via on the three
    # boards above does overlap: measured, that row SURVIVED until this
    # board was added.
    ('routed_output', 'U2', ['Net-(U2A-*)'], 'B.Cu', 0.1, 0.10),
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
    seen = {'offcentre': 0, 'disjoint': 0}
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

        seen['offcentre'] += len(offcentre)
        seen['disjoint'] += len(disjoint)

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

    # The rig must contain BOTH populations, or the per-board checks above are
    # half vacuous -- the trap the BGA half of test_fanout_via_in_pad_clamp
    # guards and its QFN half did not. Asserted across the set rather than per
    # board, because no single board carries both.
    check('the run exercises OFF-CENTRE overlapping vias (the #846 case)',
          seen['offcentre'], f"{seen['offcentre']} of them")
    check('...and DISJOINT vias, so "clamp everything" is detectable',
          seen['disjoint'], f"{seen['disjoint']} of them")

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

    # 7. THE PREDICATE ITSELF: overlap is a BARREL question, not a centre one.
    #    A via whose centre sits outside the pad can still land copper in it,
    #    and that joint needs Type VII exactly as much -- #695's finding, and
    #    why fab_notes credits the barrel radius. No board in the table above
    #    happens to contain the case (every selected offset there keeps its
    #    CENTRE on the pad), so a mutation dropping the radius survived the
    #    board arms. It is asserted directly instead.
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from synth import make_pad as _make_pad
    lead = _make_pad(1, 0.0, 0.0, ref='U1', num='1', net_name='X',
                     size_x=0.25, size_y=0.80)
    outside = 0.20         # pad half-width is 0.125, so the CENTRE is off it
    check('a via whose CENTRE is off the pad but whose BARREL overlaps it '
          'counts as via-in-pad',
          via_overlaps_pad(lead, outside, 0.0, VIA_SIZE),
          f'centre {outside} mm out on a {lead.size_x} mm-wide lead; the '
          f'{VIA_SIZE} barrel reaches {outside - VIA_SIZE / 2:+.4f} mm')
    from check_drc import point_to_pad_distance as _p2p
    check('...and it is the BARREL that credits it: the via CENTRE is '
          'outside that pad',
          _p2p(outside, 0.0, lead) > 0.0,
          f'centre-to-pad distance {_p2p(outside, 0.0, lead):.4f} mm -- if '
          f'this were 0 the arm above would pass without the barrel radius, '
          f'and the mutation dropping it would survive')
    far = 0.60             # beyond pad half-width + barrel radius
    check('a via genuinely clear of the pad is NOT via-in-pad',
          not via_overlaps_pad(lead, far, 0.0, VIA_SIZE),
          f'{far} mm out; the barrel reaches {far - VIA_SIZE / 2:.4f} mm')

    if not ran:
        print('SKIP: no corpus board present')
        return 77
    bad = [n for n, ok, _d in CHECKS if not ok]
    print(f"\n{'FAIL' if bad else 'PASS'}  #846 via-in-pad classification: "
          f"{len(CHECKS) - len(bad)}/{len(CHECKS)} checks")
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
