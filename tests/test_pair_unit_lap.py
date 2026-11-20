#!/usr/bin/env python3
"""Pair-unit lap: a pair leg left open against its partner after the
reconciliation laps is routed again together with its partner and the nets
crossing their shared pad row (pair_exchange.py, route._pair_unit_lap).

The field case: ftdi-jtag-programmer's USB-C row interleaves /USB_D+ and
/USB_D- (D+ D- D+ D- at 0.5 mm pitch, off the 0.05 grid); VBUS crossed the
row before the pair routed, and every lap swapped which leg shipped open
(99.62% on fork 4c44f0ea). The lap strips both legs and the crossers, routes
the pair, then the crossers, at nominal rules, and keeps the copy only when
every net closes.

Rows:
  1. unit: partner_name pairs _P/_N, DP/DM, +/-, P/N, and nothing else.
  2. unit: find_exchanges reports an open leg whose partner is open or a
     blocker, once per pair; an open leg blaming a stranger is not one.
  3. unit: shared_pad_box covers only the footprints both legs share.
  4. unit: crossers_in_box counts segments clipped by the box and vias in it.
  5. the lap: on a synthetic interleaved row with D- routed, D+ open and a
     crosser over the row, the lap closes D+ and keeps the board; with D+'s
     far pad walled by locked copper it keeps nothing and the file is
     byte-identical.

    python3 tests/test_pair_unit_lap.py
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

import pair_exchange as px                                      # noqa: E402
from synth import RoutingBoard                                  # noqa: E402

FAILS = []


def check(cond, msg):
    print(('PASS ' if cond else 'FAIL ') + msg)
    if not cond:
        FAILS.append(msg)


def row1_partner_name():
    names = ['/USB_D+', '/USB_D-', 'HDMI_D0_P', 'HDMI_D0_N', 'USB_DP',
             'USB_DM', 'CLKP', 'CLKN', 'VBUS', 'P', 'N']
    check(px.partner_name('/USB_D+', names) == '/USB_D-', 'D+ -> D-')
    check(px.partner_name('HDMI_D0_N', names) == 'HDMI_D0_P', '_N -> _P')
    check(px.partner_name('USB_DM', names) == 'USB_DP', 'DM -> DP')
    check(px.partner_name('CLKP', names) == 'CLKN', 'P -> N')
    check(px.partner_name('VBUS', names) is None, 'VBUS has no partner')
    check(px.partner_name('P', names) is None, 'a bare suffix is no pair')


def row2_find_exchanges():
    names = ['/USB_D+', '/USB_D-', 'A_P', 'A_N', 'VBUS']
    tlo = {'/USB_D+': {'blockers': ['/USB_D-', 'VBUS']}}
    check(px.find_exchanges(tlo, names) == [('/USB_D+', '/USB_D-')],
          'open leg blaming its partner is an exchange')
    tlo = {'A_P': {'blockers': []}, 'A_N': {'blockers': []}}
    check(px.find_exchanges(tlo, names) == [('A_N', 'A_P')],
          'both legs open: one exchange, reported once')
    tlo = {'/USB_D+': {'blockers': ['VBUS']}}
    check(px.find_exchanges(tlo, names) == [],
          'open leg blaming a stranger only is no exchange')
    check(px.find_exchanges({}, names) == [], 'nothing open, nothing found')


def row3_shared_box():
    dp = [('J1', 10.0, 5.0, 0.15, 0.5), ('J1', 11.0, 5.0, 0.15, 0.5),
          ('U1', 30.0, 20.0, 0.3, 0.3)]
    dm = [('J1', 10.5, 5.0, 0.15, 0.5), ('J1', 11.5, 5.0, 0.15, 0.5),
          ('U1', 31.0, 20.0, 0.3, 0.3), ('R9', 50.0, 50.0, 0.3, 0.3)]
    box = px.shared_pad_box([p for p in dp if p[0] == 'J1'],
                            [p for p in dm if p[0] == 'J1'], margin=0.5)
    check(box == (9.35, 4.0, 12.15, 6.0), f'J1 row box {box}')
    big = px.shared_pad_box(dp, dm, margin=0.0)
    check(big[2] == 31.3 and big[3] == 20.3, 'shared U1 joins the box, R9 not')
    check(px.shared_pad_box([('J1', 0, 0, 1, 1)], [('J2', 0, 0, 1, 1)]) is None,
          'no shared footprint: no box')


def row4_crossers():
    box = (0.0, 0.0, 10.0, 2.0)
    segs = [('VBUS', -5.0, 1.0, 15.0, 1.0), ('VBUS', 15.0, 1.0, 15.0, 9.0),
            ('CC1', 5.0, 1.5, 5.0, 8.0), ('FAR', 20.0, 20.0, 30.0, 20.0),
            ('DIAG', -1.0, 5.0, 5.0, -1.0), ('MISS', -2.0, 3.0, 3.0, 8.0),
            ('/D+', 1.0, 1.0, 2.0, 1.0)]
    vias = [('VBUS', 3.0, 1.0), ('GND', 50.0, 50.0)]
    got = px.crossers_in_box(segs, vias, box, ['/D+'])
    check(got == ['VBUS', 'CC1', 'DIAG'], f'crossers by hits {got}')


def _row_board(wall=False):
    """J1: an interleaved 4-pad row (D+ D- D+ D- at 0.5 mm, x.x5 off the
    0.1 grid) at y 5; each leg's far pad on U1/U2 at y 20; VBUS from J1's
    end pad to U3, laid across the row's +y exit; D- routed (its two row
    pads joined above the row, then down to U2); D+ unrouted."""
    b = RoutingBoard(w=40, h=30)
    nd, nm, nv = b.net('/USB_D+'), b.net('/USB_D-'), b.net('VBUS')
    pads = [(-1.25, 'VBUS', nv), (-0.75, '/USB_D+', nd), (-0.25, '/USB_D-', nm),
            (0.25, '/USB_D+', nd), (0.75, '/USB_D-', nm)]
    body = ''.join(
        '(pad "%d" smd rect (at %s 0) (size 0.3 1.0) (layers "F.Cu" "F.Mask") '
        '(net %d "%s") (uuid "j1-p%d"))' % (i + 1, dx, n, nm_, i + 1)
        for i, (dx, nm_, n) in enumerate(pads))
    b.items.append(
        '  (footprint "synth:ROW" (layer "F.Cu") (uuid "j1") (at 20.05 5)\n'
        '    (property "Reference" "J1" (at 0 -2 0) (layer "F.SilkS") '
        '(uuid "j1-ref")\n      (effects (font (size 0.5 0.5) '
        '(thickness 0.1))))\n    %s\n  )' % body)
    b.pad('U1', 14.0, 20.0, '/USB_D+')
    b.pad('U2', 26.0, 20.0, '/USB_D-')
    b.pad('U3', 34.0, 8.0, 'VBUS')
    # D-: its row pads (19.8, 20.8) joined above the row, then to U2.
    for a, c in (((19.8, 5.0), (19.8, 3.6)), ((19.8, 3.6), (20.8, 3.6)),
                 ((20.8, 3.6), (20.8, 5.0)), ((20.8, 5.0), (20.8, 6.4)),
                 ((20.8, 6.4), (26.0, 11.6)), ((26.0, 11.6), (26.0, 20.0))):
        b.seg(a[0], a[1], c[0], c[1], '/USB_D-', w=0.25)
    # VBUS: from its row pad (18.8) under the row, across to U3.
    for a, c in (((18.8, 5.0), (18.8, 6.4)), ((18.8, 6.4), (19.6, 7.2)),
                 ((19.6, 7.2), (34.0, 7.2)), ((34.0, 7.2), (34.0, 8.0))):
        b.seg(a[0], a[1], c[0], c[1], 'VBUS', w=0.25)
    if wall:
        b.ring([(12.6, 18.6), (15.4, 18.6), (15.4, 21.4), (12.6, 21.4)],
               'WALL', w=0.3, locked=True)
        b.ring([(12.6, 18.6), (15.4, 18.6), (15.4, 21.4), (12.6, 21.4)],
               'WALL', w=0.3, layer='B.Cu', locked=True)
    return b.text()


def _run_lap(text, tmp):
    import route
    path = os.path.join(tmp, 'row.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(text)
    before = open(path, 'rb').read()
    route.batch_route._targets_left_open = {
        '/USB_D+': {'boxed_in': False, 'blockers': ['/USB_D-', 'VBUS']}}
    route.batch_route._victims_left_open = None
    rk = dict(grid_step=0.05, track_width=0.25, clearance=0.2,
              layers=['F.Cu', 'B.Cu'], via_size=0.6, via_drill=0.3)
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        n = route._pair_unit_lap(path, rk, lambda c: list(c))
    log = buf.getvalue()
    after = open(path, 'rb').read()
    left = [f for f in os.listdir(tmp) if 'pairlap' in f]
    return n, log, before, after, path, left


def _dplus_joined(path):
    from kicad_parser import parse_kicad_pcb
    pcb = parse_kicad_pcb(path)
    nid = next(k for k, v in pcb.nets.items() if v.name == '/USB_D+')
    segs = [s for s in pcb.segments if s.net_id == nid]
    ends = {(round(s.start_x, 3), round(s.start_y, 3)) for s in segs} | \
           {(round(s.end_x, 3), round(s.end_y, 3)) for s in segs}
    pads = [(round(p.global_x, 3), round(p.global_y, 3))
            for p in pcb.pads_by_net.get(nid, [])]
    return len(segs), all(
        any(abs(e[0] - p[0]) < 0.2 and abs(e[1] - p[1]) < 0.55 for e in ends)
        for p in pads)


def row5_lap():
    tmp = tempfile.mkdtemp(prefix='pairlap-')
    try:
        n, log, before, after, path, left = _run_lap(_row_board(), tmp)
        check(n == 1, f'lap closed the open leg (returned {n})')
        check('routed as a unit' in log and 'kept' in log, 'lap kept its copy')
        check(before != after, 'board file replaced')
        k, joined = _dplus_joined(path)
        check(k > 0 and joined, f'D+ copper reaches all its pads ({k} segs)')
        check(not left, f'no .pairlap siblings left ({left})')
        shutil.rmtree(tmp, ignore_errors=True)
        tmp = tempfile.mkdtemp(prefix='pairlap-')
        n, log, before, after, path, left = _run_lap(_row_board(wall=True), tmp)
        check(n == 0, f'walled far pad: lap closes nothing (returned {n})')
        check('not kept' in log, 'walled far pad: copy rejected')
        check(before == after, 'walled far pad: file byte-identical')
        check(not left, f'no .pairlap siblings left ({left})')
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


if __name__ == '__main__':
    row1_partner_name()
    row2_find_exchanges()
    row3_shared_box()
    row4_crossers()
    row5_lap()
    print('FAILED: %d' % len(FAILS) if FAILS else 'ALL PASS')
    sys.exit(1 if FAILS else 0)
