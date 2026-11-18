#!/usr/bin/env python3
"""#1925: a layer a back-drill removes is not the barrel's copper.

A via declaring `(backdrill (size 0.5) (layers "B.Cu" "In2.Cu"))` has no
copper on In2.Cu and B.Cu. `via_on_layer` / `via_copper_layers` counted those
layers, so a route step ended a B.Cu track on the via, check_connected read
the net as joined and check_drc had nothing to say, while KiCad reports
`track_on_post_machined_layer`. The same held for a back-drilled plated pad.

The board is tests/backdrill_fixture.py's: F.Cu, In1.Cu, In2.Cu, B.Cu. Net /S
has an F.Cu pad P1 at (4, 5), a B.Cu pad P2 at (16, 5) and a 0.6 / 0.3 via at
(10, 5) back-drilled 0.5 from B.Cu to In2.Cu, with an F.Cu track P1 -> via.

  1. `via_on_layer` / `via_copper_layers` leave the removed layers out, for a
     Via and for a via dict, while `via_spans_layer` / `via_span_layers` keep
     them (the hole is still there for a clearance); `pad_joined_layers` does
     the same for a drilled pad; a via with no back-drill reads as before;
  2. check_drc reports `track-on-backdrilled-layer` on exactly the hand-made
     boards kicad-cli 10.0.8 reported `track_on_post_machined_layer` on, and
     on none of the ones it did not (CASES; the installed kicad-cli is asked
     again when there is one);
  3. check_connected reports /S open on every one of them, and on a pad of
     the net laid over the via on a removed layer; the same boards without
     the back-drill are connected and clean (the drill is what is graded);
  4. route.py joins P2 without a track of /S on the via's removed layers, and
     both checkers pass the board it writes; without the back-drill the same
     run ends its B.Cu track on the via (the fixture reaches that path);
  5. the same through a back-drilled plated pad.

Run with:  python3 tests/test_1925_backdrilled_layer_not_copper.py
"""
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, 'py_router'))

import run_utils  # noqa: E402
import backdrill_fixture as fx  # noqa: E402

PY = sys.executable
FAILS = []
TYPE = 'track-on-backdrilled-layer'

BD = fx.backdrill_token(0.5, 'B.Cu', 'In2.Cu')
PAD_BD = '\t\t\t(backdrill (size 0.9) (layers "B.Cu" "In2.Cu"))\n'
PADS = [('P1', '/S', 4, 5, 'F.Cu', 1.0), ('P2', '/S', 16, 5, 'B.Cu', 1.0)]
STUB = ('/S', 4, 5, 10, 5, 'F.Cu', 0.2)
VIA = ('/S', 10, 5, 0.6, 0.3, BD)
HOLE = ('J1', '/S', 10, 5, 1.2, 0.6, PAD_BD)

#: name: (second track of /S, through a pad instead of the via, what kicad-cli
#: 10.0.8 reported). The via's pad is 0.3 mm in radius, the drilled pad 0.6.
CASES = {
    'ends on the via, B.Cu':        (('/S', 10, 5, 16, 5, 'B.Cu', 0.2), False, True),
    'ends on the via, In2.Cu':      (('/S', 10, 5, 14, 5, 'In2.Cu', 0.2), False, True),
    'passes over the via':          (('/S', 8, 5, 16, 5, 'B.Cu', 0.2), False, True),
    'ends 0.28 from its centre':    (('/S', 10.28, 5, 16, 5, 'B.Cu', 0.2), False, True),
    'passes 0.20 from its centre':  (('/S', 8, 5.2, 12, 5.2, 'B.Cu', 0.2), False, True),
    'ends 0.35 from its centre':    (('/S', 10.35, 5, 16, 5, 'B.Cu', 0.2), False, False),
    'passes 0.38 from its centre':  (('/S', 8, 5.38, 12, 5.38, 'B.Cu', 0.2), False, False),
    'ends on the via, In1.Cu':      (('/S', 10, 5, 14, 5, 'In1.Cu', 0.2), False, False),
    'ends on the drilled pad':      (('/S', 10, 5, 16, 5, 'B.Cu', 0.2), True, True),
    'ends 0.50 inside the pad':     (('/S', 10.5, 5, 16, 5, 'B.Cu', 0.2), True, True),
    'ends 0.65 from the pad':       (('/S', 10.65, 5, 16, 5, 'B.Cu', 0.2), True, False),
}


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def write(td, name, track=None, pad=False, drilled=True, pads=PADS):
    via = VIA if drilled else VIA[:5] + ('',)
    hole = HOLE if drilled else HOLE[:6] + ('',)
    path = fx.write_board(os.path.join(td, name + '.kicad_pcb'), pads=pads,
                          tracks=[STUB] + ([track] if track else []),
                          vias=[] if pad else [via], holes=[hole] if pad else [])
    return run_utils.evidence(path, 'fixture board')


def drc(path):
    from check_drc import run_drc
    return [v for v in run_drc(path, clearance=0.2, quiet=True) if not v.get('accepted')]


def open_nets(path):
    from check_connected import run_connectivity_check
    return sorted({i['net_name'] for i in run_connectivity_check(path, quiet=True)})


def kicad_reports(cli, path):
    """How many `track_on_post_machined_layer` items kicad-cli reports."""
    out = path[:-len('.kicad_pcb')] + '.drc.json'
    subprocess.run([cli, 'pcb', 'drc', '--format', 'json', '--severity-all',
                    '--all-track-errors', '-o', out, path],
                   capture_output=True, timeout=300)
    with open(run_utils.evidence(out, 'kicad-cli DRC report'), encoding='utf-8') as fh:
        return sum(1 for v in json.load(fh).get('violations', ())
                   if v.get('type') == 'track_on_post_machined_layer')


def layers(td):
    print('1. the removed layers are not the barrel\'s copper')
    from geometry_utils import (via_on_layer, via_copper_layers, via_spans_layer,
                                via_span_layers)
    from kicad_parser import parse_kicad_pcb
    from net_queries import pad_joined_layers, expand_pad_layers
    cu = [name for name, _i, _d in fx.COPPER]
    pcb = parse_kicad_pcb(write(td, 'via'))
    via = pcb.vias[0]
    ok(bool(via.backdrills), 'the fixture via carries its back-drill')
    ok(via_copper_layers(via, cu) == ['F.Cu', 'In1.Cu']
       and [via_on_layer(via, l) for l in cu] == [True, True, False, False],
       f'a via has copper on the layers its back-drill leaves: {via_copper_layers(via, cu)}')
    ok(via_span_layers(via, cu) == cu and all(via_spans_layer(via, l) for l in cu),
       'its barrel still passes every layer, for a clearance or a hole')
    as_dict = {'x': via.x, 'y': via.y, 'size': via.size, 'drill': via.drill,
               'layers': list(via.layers), 'backdrills': dict(via.backdrills)}
    ok(via_copper_layers(as_dict, cu) == ['F.Cu', 'In1.Cu'],
       'a via dict with the key reads the same')
    plain = parse_kicad_pcb(write(td, 'via_plain', drilled=False)).vias[0]
    ok(not plain.backdrills and via_copper_layers(plain, cu) == cu,
       'a via with no back-drill has copper on every layer, as before')
    pad = [p for p in parse_kicad_pcb(write(td, 'pad', pad=True)).pads_by_net[1]
           if p.component_ref == 'J1'][0]
    ok(bool(pad.backdrills) and pad_joined_layers(pad, cu) == ['F.Cu', 'In1.Cu']
       and expand_pad_layers(pad.layers, cu) == cu,
       f'a back-drilled plated pad joins its net on {pad_joined_layers(pad, cu)}, '
       f'and its own layer set is unchanged')


def checkers(td):
    print('2. check_drc grades the joint as KiCad does; 3. check_connected reads it open')
    from kicad_oracle import find_kicad_cli
    cli = find_kicad_cli(warn=False)
    asked = 0
    for name, (track, pad, want) in CASES.items():
        path = write(td, 'case_' + name.replace(' ', '_').replace(',', '').replace('.', ''),
                     track=track, pad=pad)
        rows = [v for v in drc(path) if v['type'] == TYPE]
        ok(len(rows) == (1 if want else 0),
           f'{name}: check_drc reports {len(rows)}, kicad-cli 10.0.8 reported '
           f'{1 if want else 0}')
        if want:
            ok(rows and rows[0]['layer'] == track[5] and rows[0]['net1'] == '/S',
               f'{name}: the row names the net and the layer')
            ok(open_nets(path) == ['/S'], f'{name}: check_connected reports /S open')
        if cli:
            got = kicad_reports(cli, path)
            asked += 1
            ok(got == len(rows), f'{name}: the installed kicad-cli reports {got}')
    print(f'  ({"asked the installed kicad-cli on %d boards" % asked if cli else "no kicad-cli here: the recorded 10.0.8 answers stand"})')

    # the drill is what is graded: the same copper without it is one net
    track, _pad, _want = CASES['ends on the via, B.Cu']
    plain = write(td, 'plain', track=track, drilled=False)
    ok(not [v for v in drc(plain) if v['type'] == TYPE] and open_nets(plain) == [],
       'the same board without the back-drill is connected and has no such row')
    plain_pad = write(td, 'plain_pad', track=track, pad=True, drilled=False)
    ok(not [v for v in drc(plain_pad) if v['type'] == TYPE] and open_nets(plain_pad) == [],
       'and the same through an undrilled plated pad')

    # a pad of the net over the via on a removed layer joins nothing
    over = [PADS[0], ('P2', '/S', 10, 5, 'B.Cu', 1.0)]
    ok(open_nets(write(td, 'pad_over_via', pads=over)) == ['/S']
       and open_nets(write(td, 'pad_over_plain_via', pads=over, drilled=False)) == [],
       'a B.Cu pad of the net over the via is open with the back-drill and '
       'joined without it')


def _route(td, name, src):
    out = os.path.join(td, name + '_routed.kicad_pcb')
    r = run_utils.check([PY, run_utils.tool('route.py'), src, out, '--nets', '/S',
                         '--layers', 'F.Cu', 'B.Cu'], accept=True)
    from kicad_parser import parse_kicad_pcb
    return r, out, parse_kicad_pcb(run_utils.evidence(out, 'routed board'))


def _near(pcb, x, y, reach, removed=('In2.Cu', 'B.Cu')):
    from geometry_utils import point_to_segment_distance
    return [s for s in pcb.segments if s.layer in removed and point_to_segment_distance(
        x, y, s.start_x, s.start_y, s.end_x, s.end_y) < reach + s.width / 2.0]


def routed(td):
    print('4. route.py does not end a track on a removed layer of the via')
    _r, out, pcb = _route(td, 'via', write(td, 'route_via'))
    on_b = [s for s in pcb.segments if s.layer == 'B.Cu']
    ok(bool(on_b), f'the route reaches P2 on B.Cu ({len(on_b)} segment(s))')
    ok(not _near(pcb, 10, 5, 0.3),
       'no track of /S on In2.Cu or B.Cu touches the back-drilled via')
    ok(any(math.hypot(v.x - 10, v.y - 5) > 0.3 for v in pcb.vias),
       'the layer change is a via of its own, off the back-drilled one')
    ok(not drc(out) and open_nets(out) == [],
       'check_drc and check_connected pass the routed board')

    _r, _out, plain = _route(td, 'plain', write(td, 'route_plain', drilled=False))
    ok(any(s.layer == 'B.Cu' and min(math.hypot(s.start_x - 10, s.start_y - 5),
                                     math.hypot(s.end_x - 10, s.end_y - 5)) < 1e-6
           for s in plain.segments),
       'without the back-drill the same run ends its B.Cu track on the via')

    print('5. the same through a back-drilled plated pad')
    _r, out, pcb = _route(td, 'pad', write(td, 'route_pad', pad=True))
    ok(any(s.layer == 'B.Cu' for s in pcb.segments)
       and not _near(pcb, 10, 5, 0.6),
       'the route reaches P2 with no track of /S on the pad\'s removed layers')
    ok(not drc(out) and open_nets(out) == [],
       'check_drc and check_connected pass the routed board')
    _r, _out, plain = _route(td, 'plain_pad', write(td, 'route_plain_pad', pad=True,
                                                    drilled=False))
    ok(bool(_near(plain, 10, 5, 0.6, removed=('B.Cu',))),
       'without the back-drill the same run lands on the pad on B.Cu')


def main():
    td = tempfile.mkdtemp(prefix='krt1925n_')
    try:
        layers(td)
        checkers(td)
        routed(td)
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if FAILS:
        print(f'FAIL: {len(FAILS)}: ' + '; '.join(FAILS))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
