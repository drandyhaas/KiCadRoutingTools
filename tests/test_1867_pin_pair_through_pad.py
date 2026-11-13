#!/usr/bin/env python3
"""#1867: pin_pair_path_length walks a path that runs through a third pad.

A lane that enters an in-line part's pad at one point and leaves it at
another -- an ESD device, a test point, a connector's second contact -- has
no track between the two points: only the pad's copper joins them. The walk
attached the two end pads alone, so it stopped there and read None, the value
its docs keep for an unrouted or broken net. `board_score --length-groups`
in `pin_pair` mode then skipped the group as "pads not joined by track
copper" on a net check_connected calls connected.

Now, when no track path joins the two pads, every copper pad of the net is
attached, and a pad the path only passes is charged in its plane as a via
is: from where the walk meets its copper straight to its centre, and out
again. A length the track copper alone gives is returned as it was.

Rows (one net each, on one synthetic board):
  THRU     A -> a point inside C, another point inside C -> B: measured, at
           the two tracks plus the hop in and out of C's centre
  BARREL   the same through a drilled pad, changing layer in it
  and the controls, which read what they read before:
  JOINT    two tracks meeting outside any pad, a third pad on a stub
  OVER     one straight track that runs over a third pad
  ENDPAD   a track ending inside an END pad, off its centre (free)
  VIP      a via in each end pad, the track on the other layer
  VIAIN    a via inside the passed pad carrying the lane to the other layer
  CHORD    a track that bends inside a passed pad on its way to a via at the
           pad's centre: its own length, the pad's centre is no short cut
  OPEN     two stubs that meet nowhere: still None

    python3 tests/test_1867_pin_pair_through_pad.py
"""
import contextlib
import io
import json
import math
import os
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, os.path.join(REPO, 'py_tools'))
import run_utils  # noqa: E402
from synth import RoutingBoard  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402
from net_queries import net_pads_joined, pin_pair_path_length  # noqa: E402

failures = []
HOP = math.hypot(0.3, 0.2)          # a track end's distance to C's centre


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def board(path):
    """Each row's end pads are added FIRST, so they are its net's first two
    pads, the pair `mode: pin_pair` measures."""
    rb = RoutingBoard(w=40, h=100)

    def ends(net, y, tht=False):
        rb.pad(f'A_{net}', 10, y, net, tht=tht)
        rb.pad(f'B_{net}', 30, y, net, tht=tht)

    ends('THRU', 10)
    rb.pad('C_THRU', 20, 10, 'THRU', sx=1.0, sy=0.8)
    rb.seg(10, 10, 19.7, 10.2, 'THRU', w=0.2)
    rb.seg(20.3, 9.8, 30, 10, 'THRU', w=0.2)

    ends('BARREL', 20, tht=True)
    rb.pad('C_BARREL', 20, 20, 'BARREL', sx=1.6, sy=1.6, tht=True)
    rb.seg(10, 20, 19.8, 20, 'BARREL', w=0.2)
    rb.seg(20.2, 20, 30, 20, 'BARREL', w=0.2, layer='B.Cu')

    ends('JOINT', 30)
    rb.pad('C_JOINT', 20, 35, 'JOINT')
    rb.seg(10, 30, 20, 30, 'JOINT', w=0.2)
    rb.seg(20, 30, 30, 30, 'JOINT', w=0.2)
    rb.seg(20, 30, 20, 35, 'JOINT', w=0.2)

    ends('OVER', 40)
    rb.pad('C_OVER', 20, 40.2, 'OVER')
    rb.seg(10, 40, 30, 40, 'OVER', w=0.2)

    ends('ENDPAD', 50)
    rb.seg(10, 50, 29.7, 50.2, 'ENDPAD', w=0.2)

    ends('VIP', 60)
    rb.via(10, 60, 'VIP')
    rb.via(30, 60, 'VIP')
    rb.seg(10, 60, 30, 60, 'VIP', w=0.2, layer='B.Cu')

    rb.pad('A_VIAIN', 10, 70, 'VIAIN')
    rb.pad('B_VIAIN', 30, 70, 'VIAIN', tht=True)
    rb.pad('C_VIAIN', 20, 70, 'VIAIN', sx=1.6, sy=1.6)
    rb.seg(10, 70, 19.8, 70, 'VIAIN', w=0.2)
    rb.via(19.8, 70, 'VIAIN')
    rb.seg(19.8, 70, 30, 70, 'VIAIN', w=0.2, layer='B.Cu')

    rb.pad('A_CHORD', 10, 90.5, 'CHORD')
    rb.pad('B_CHORD', 30, 90, 'CHORD', tht=True)
    rb.pad('C_CHORD', 20, 90, 'CHORD', sx=1.6, sy=1.6)
    rb.seg(10, 90.5, 19.4, 90.5, 'CHORD', w=0.2)
    rb.seg(19.4, 90.5, 19.4, 90, 'CHORD', w=0.2)
    rb.seg(19.4, 90, 20, 90, 'CHORD', w=0.2)
    rb.via(20, 90, 'CHORD', size=0.6)
    rb.seg(20, 90, 30, 90, 'CHORD', w=0.2, layer='B.Cu')

    ends('OPEN', 80)
    rb.pad('C_OPEN', 20, 80, 'OPEN')
    rb.seg(10, 80, 15, 80, 'OPEN', w=0.2)
    rb.seg(25, 80, 30, 80, 'OPEN', w=0.2)
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(rb.text())
    return run_utils.evidence(path) or path


def main():
    td = tempfile.mkdtemp(prefix='krt1867_')
    try:
        path = board(os.path.join(td, 'b.kicad_pcb'))
        with contextlib.redirect_stdout(io.StringIO()):
            pcb = parse_kicad_pcb(path)
        ids = {n.name: i for i, n in pcb.nets.items()}

        def pads(net):
            return {p.component_ref.split('_')[0]: p for p in pcb.nets[ids[net]].pads}

        def walk(net, a='A', b='B'):
            p = pads(net)
            return pin_pair_path_length(pcb, ids[net], p[a], p[b])

        def near(got, want):
            return got is not None and abs(got - want) <= 1e-3

        print('1. a path through a third pad is walked, and the pad is charged')
        leg = math.hypot(9.7, 0.2)
        got = walk('THRU')
        check('THRU: A to B reads a length, not None', got is not None, str(got))
        check(f'THRU: the two tracks plus the hop in and out of the pad\'s centre '
              f'({2 * leg + 2 * HOP:.4f} mm)', near(got, 2 * leg + 2 * HOP), str(got))
        check('THRU: more than the two tracks alone (the hop is not free)',
              got is not None and got > 2 * leg + 0.5, str(got))
        check('THRU: either half ends ON that pad and is not charged for it',
              near(walk('THRU', 'A', 'C'), leg) and near(walk('THRU', 'C', 'B'), leg),
              f"{walk('THRU', 'A', 'C')} {walk('THRU', 'C', 'B')}")
        check('THRU: the net reads joined, as it did',
              net_pads_joined(pcb, ids['THRU']))
        got = walk('BARREL')
        check('BARREL: through a drilled pad, changing layer in it (9.8 + 0.2 + '
              '0.2 + 9.8 mm)', near(got, 20.0), str(got))

        print('2. the controls read what they read before')
        for net, want, why in (
                ('JOINT', 20.0, 'two tracks meeting outside any pad; the third '
                                'pad is on a stub'),
                ('OVER', 20.0, 'one track running over a third pad is charged '
                               'its own length'),
                ('ENDPAD', math.hypot(19.7, 0.2), 'an end pad\'s own copper is free'),
                ('VIP', 20.0, 'a via in each end pad'),
                ('VIAIN', 20.0, 'a via inside the passed pad carries the lane; '
                                'the pad adds nothing'),
                ('CHORD', 20.5, 'a track bending inside a passed pad, to a via at '
                                'its centre: 9.4 + 0.5 + 0.6 + 10 mm of track, not '
                                'the 0.78 mm chord to the pad\'s centre')):
            got = walk(net)
            check(f'{net}: {why} ({want:.4f} mm)', near(got, want), str(got))
        check('OPEN: two stubs that meet nowhere still read None',
              walk('OPEN') is None, str(walk('OPEN')))

        print('3. board_score measures the pin_pair group')
        import board_score
        groups = os.path.join(td, 'groups.json')
        with open(groups, 'w', encoding='utf-8') as fh:
            json.dump({'LANE': {'nets': ['THRU', 'JOINT'], 'mode': 'pin_pair',
                                'tolerance_mm': 1.0, 'max_mm': 25}}, fh)
        with contextlib.redirect_stdout(io.StringIO()):
            res = board_score.score_length(path, run_utils.evidence(groups) or groups)
        g = (res.get('groups') or {}).get('LANE') or {}
        check('the group ran and is not skipped as "pads not joined"',
              res.get('ran') is True and 'skipped' not in g, json.dumps(g)[:300])
        check('its lengths are the walked ones',
              near((g.get('lengths_mm') or {}).get('THRU'), 2 * leg + 2 * HOP)
              and near((g.get('lengths_mm') or {}).get('JOINT'), 20.0)
              and g.get('pass') is True, json.dumps(g)[:300])
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if failures:
        print(f'FAIL: {len(failures)}: ' + '; '.join(failures))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
