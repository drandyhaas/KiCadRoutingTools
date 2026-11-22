#!/usr/bin/env python3
"""No copper in an edge connector's finger strip but a finger's own entry (#2022).

A card edge's fingers are the mating surface, and the edge past them is
bevelled, so human card-edge boards keep the whole strip -- from the fingers'
inner ends to the board edge -- free of copper: each finger is entered from
its inner end, and a via stands above the finger row. The router used to run
tracks along the strip past the fingers' outer ends, between the fingers, and
drop vias in it. `net_keepouts.derive_finger_bands` now turns the strip into
net keep-outs, installed by `design_rules.install_design_rules` (both
front-ends) and graded by check_drc.

The board is written here from s-expressions (the suite image has no KiCad):
a 2-layer 30 x 20 mm board and a four-finger plug J1 on F.Cu, fingers ending
1.4 mm short of the bottom edge (as the stock PCIe footprints do), the fourth
a shorter presence finger in line with the others.

  1. derive: one via region over the row, a column per finger binding every
     net but its finger's, a tail past each finger's outer end binding only
     its own; the short finger is in the row, its tail from its own end;
  2. controls: strips lying ALONG an edge derive no band, and nor does the
     plug moved inside the board;
  3. route.py: SIG, whose shortest path runs under the fingers' outer ends to
     a pad beside the connector, and N2, which needs a via down to a pad on
     B.Cu under the strip, both route and connect, with no copper in the
     band (check_drc's census of it);
  4. check_drc grades a via in the strip, another net's track in a column and
     a finger's own track past its outer end as `keepout` rows
     (`finger_band` via / column / tail), each accepted `inherited-keepout`
     with the board itself as `--baseline`.

    python3 tests/test_2022_finger_band.py
"""
import contextlib
import io
import os
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

import run_utils                                                 # noqa: E402
from kicad_parser import parse_kicad_pcb                         # noqa: E402

failures = []
# Fingers 1.5 x 6 mm (the fourth 1.5 x 4.6, its inner end in line), centred
# on the plug, ending at y = 18.6: 1.4 mm short of the bottom edge (y = 20).
PLUG_AT = (10.0, 15.6)
FINGER_X = (-3.75, -1.25, 1.25, 3.75)
FINGER_NETS = ('SIG', 'N2', 'N3', 'N4')
INNER, OUTER = 12.6, 18.6
SHORT_OUTER = 17.2

HEAD = '''(kicad_pcb
\t(version 20260206)
\t(generator "pcbnew")
\t(generator_version "10.0")
\t(general
\t\t(thickness 1.6)
\t\t(legacy_teardrops no)
\t)
\t(paper "A4")
\t(layers
\t\t(0 "F.Cu" signal)
\t\t(2 "B.Cu" signal)
\t\t(13 "F.Paste" user)
\t\t(1 "F.Mask" user)
\t\t(3 "B.Mask" user)
\t\t(5 "F.SilkS" user "F.Silkscreen")
\t\t(25 "Edge.Cuts" user)
\t\t(35 "F.Fab" user)
\t)
\t(setup
\t\t(pad_to_mask_clearance 0)
\t)
'''


def ok(label, cond, detail=''):
    print(f"  [{'ok' if cond else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not cond:
        failures.append(label)


def _ref(ref):
    return ('\t\t(property "Reference" "%s"\n\t\t\t(at 0 0 0)\n'
            '\t\t\t(layer "F.SilkS")\n\t\t\t(effects\n\t\t\t\t(font\n'
            '\t\t\t\t\t(size 1.27 1.27)\n\t\t\t\t)\n\t\t\t)\n\t\t)\n' % ref)


def _plug(at=PLUG_AT, along=False):
    """J1: four strip fingers, `connect` pads, no model. `along` lays the
    strips flat along the bottom edge instead, side by side (the control)."""
    out = ['\t(footprint "Test:Plug"\n\t\t(layer "F.Cu")\n\t\t(at %g %g)\n' % at,
           _ref('J1'), '\t\t(attr exclude_from_pos_files board_only)\n']
    for i, (x, net) in enumerate(zip(FINGER_X, FINGER_NETS), 1):
        if along:
            pos, size = '%g 0' % (x * 2.0), '6 1.5'
        elif i == 4:   # the short presence finger: inner end in line
            pos, size = '%g %g' % (x, -0.7), '1.5 4.6'
        else:
            pos, size = '%g 0' % x, '1.5 6'
        out.append('\t\t(pad "%d" connect rect\n\t\t\t(at %s)\n\t\t\t(size %s)\n'
                   '\t\t\t(layers "F.Cu" "F.Mask")\n\t\t\t(net "%s")\n\t\t)\n'
                   % (i, pos, size, net))
    out.append('\t)\n')
    return ''.join(out)


def _pad(ref, x, y, net, layer='F.Cu'):
    mask = layer[0] + '.Mask'
    return ('\t(footprint ""\n\t\t(layer "%s")\n\t\t(at %g %g)\n' % (layer, x, y) + _ref(ref)
            + '\t\t(pad "1" smd rect\n\t\t\t(at 0 0)\n\t\t\t(size 1 1)\n'
            '\t\t\t(layers "%s" "%s")\n\t\t\t(net "%s")\n\t\t)\n\t)\n' % (layer, mask, net))


def _edge(x1, y1, x2, y2):
    return ('\t(gr_line\n\t\t(start %g %g)\n\t\t(end %g %g)\n'
            '\t\t(stroke\n\t\t\t(width 0.1)\n\t\t\t(type default)\n\t\t)\n'
            '\t\t(layer "Edge.Cuts")\n\t)\n' % (x1, y1, x2, y2))


def _segment(x1, y1, x2, y2, layer, net):
    return ('\t(segment\n\t\t(start %g %g)\n\t\t(end %g %g)\n\t\t(width 0.25)\n'
            '\t\t(layer "%s")\n\t\t(net "%s")\n\t)\n' % (x1, y1, x2, y2, layer, net))


def _via(x, y, net):
    return ('\t(via\n\t\t(at %g %g)\n\t\t(size 0.6)\n\t\t(drill 0.3)\n'
            '\t\t(layers "F.Cu" "B.Cu")\n\t\t(net "%s")\n\t)\n' % (x, y, net))


def write_board(path, plug_at=PLUG_AT, along=False, extra=''):
    parts = [HEAD, _plug(plug_at, along),
             # SIG's partner beside the connector, near the edge: its shortest
             # path from finger 1 runs under the fingers' outer ends
             _pad('U1', 18.5, 19.2, 'SIG'),
             # N2's partner on B.Cu under the strip: it needs a via
             _pad('U2', 8.75, 19.2, 'N2', 'B.Cu'), extra]
    corners = [(0, 0), (30, 0), (30, 20), (0, 20)]
    for i in range(4):
        parts.append(_edge(*corners[i], *corners[(i + 1) % 4]))
    parts.append(')\n')
    with open(path, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write(''.join(parts))
    return run_utils.evidence(path, 'finger-band board')


def parse(path):
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        return parse_kicad_pcb(path)


def bounds(poly):
    xs, ys = [p[0] for p in poly], [p[1] for p in poly]
    return min(xs), min(ys), max(xs), max(ys)


def derive(tmp):
    print('1. derive on the plug')
    from net_keepouts import derive_finger_bands
    kos = derive_finger_bands(parse(write_board(os.path.join(tmp, 'derive.kicad_pcb'))))
    via = [k for k in kos if k['finger_band'] == 'via']
    cols = [k for k in kos if k['finger_band'] == 'column']
    tails = [k for k in kos if k['finger_band'] == 'tail']
    ok('one via region over the row, vias forbidden on every layer, tracks allowed',
       len(via) == 1 and not via[0]['vias_allowed'] and via[0]['tracks_allowed']
       and {'F.Cu', 'B.Cu'} <= set(via[0]['layers']), str([(k['name'], k['layers']) for k in via]))
    if via:
        x0, y0, x1, y1 = bounds(via[0]['polygon'])
        ok('the via region spans the row from the inner ends past the edge',
           abs(y0 - INNER) < 1e-6 and y1 > 20.0 and x0 < 6.25 - 0.75 and x1 > 13.75 + 0.75,
           str((x0, y0, x1, y1)))
    ok('a column per finger, tracks forbidden on F.Cu only',
       len(cols) == 4 and all(not k['tracks_allowed'] and k['layers'] == {'F.Cu'}
                              for k in cols), str(len(cols)))
    ok("each column binds every net but its finger's",
       all(FINGER_NETS[int(k['pad']) - 1] not in k['nets']
           and all(n in k['nets'] for n in FINGER_NETS if n != FINGER_NETS[int(k['pad']) - 1])
           for k in cols))
    by_pad = {k['pad']: k for k in tails}
    ok("a tail per finger, binding only its own net",
       len(tails) == 4 and all(k['nets'] == {FINGER_NETS[int(p) - 1]} for p, k in by_pad.items()))
    if '4' in by_pad and '1' in by_pad:
        ok("the short finger is in the row: its tail starts at its own outer end",
           abs(bounds(by_pad['4']['polygon'])[1] - SHORT_OUTER) < 1e-6
           and abs(bounds(by_pad['1']['polygon'])[1] - OUTER) < 1e-6,
           f"{bounds(by_pad['4']['polygon'])}, {bounds(by_pad['1']['polygon'])}")
    cx = sorted(bounds(k['polygon'])[0] for k in cols)
    ok('the columns tile the row, edge to edge at the midpoints between fingers',
       all(abs(b - a - 2.5) < 1e-6 for a, b in zip(cx, cx[1:])), str(cx))


def controls(tmp):
    print('2. controls')
    from net_keepouts import derive_finger_bands
    along = derive_finger_bands(parse(write_board(
        os.path.join(tmp, 'along.kicad_pcb'), plug_at=(15.0, 19.0), along=True)))
    ok('strips lying along the edge: no band', along == [], str(len(along)))
    inside = derive_finger_bands(parse(write_board(
        os.path.join(tmp, 'inside.kicad_pcb'), plug_at=(15.0, 8.0))))
    ok('the plug inside the board: no band', inside == [], str(len(inside)))


def band_rows(path, baseline=None):
    from check_drc import _finger_band_pass
    pcb = parse(path)
    violations, accepted = [], []
    base = parse(baseline) if baseline else None
    with contextlib.redirect_stdout(io.StringIO()):
        _finger_band_pass(pcb, list(pcb.board_info.copper_layers), None, base,
                          violations, accepted, 1e-6, True)
    return violations, accepted


def route(tmp):
    print('3. route.py keeps the strip clear')
    src = write_board(os.path.join(tmp, 'in.kicad_pcb'))
    out = os.path.join(tmp, 'out.kicad_pcb')
    run_utils.run_step([run_utils.tool('route.py'), src, out, '--nets', 'SIG', 'N2',
                        '--layers', 'F.Cu', 'B.Cu', '--clearance', '0.2',
                        '--track-width', '0.25', '--via-size', '0.6', '--via-drill', '0.3'],
                       outputs=[out])
    r = subprocess.run([sys.executable, '-X', 'utf8', run_utils.tool('check_connected.py'),
                        out, '--nets', 'SIG', 'N2'], capture_output=True, text=True,
                       env=run_utils.tool_env(), timeout=600)
    ok('SIG and N2 connect', r.returncode == 0 and 'ALL NETS FULLY CONNECTED' in r.stdout,
       r.stdout[-300:])
    pcb = parse(out)
    nets = {n.name: nid for nid, n in pcb.nets.items()}
    ok('N2 dropped a via (its pad is on B.Cu)',
       any(v.net_id == nets['N2'] for v in pcb.vias))
    rows, _acc = band_rows(out)
    ok('no copper in the band', rows == [],
       str([(r['area'], r['item'], r['net1']) for r in rows][:6]))


def grade(tmp):
    print('4. check_drc grades copper in the strip')
    bad = (_via(8.0, 17.0, 'N3')                              # a via in the strip
           + _segment(5.0, 11.0, 7.5, 16.0, 'F.Cu', 'N3')       # N3 into SIG's column
           + _segment(6.25, 18.0, 6.25, 19.5, 'F.Cu', 'SIG'))   # SIG past its outer end
    path = write_board(os.path.join(tmp, 'bad.kicad_pcb'), extra=bad)
    rows, _acc = band_rows(path)
    kinds = sorted({r['finger_band'] for r in rows})
    ok('a via, a column track and a tail track: one kind each',
       kinds == ['column', 'tail', 'via'],
       str([(r['area'], r['item'], r['net1']) for r in rows]))
    ok('the via is one row, not one per layer',
       sum(1 for r in rows if r['item'] == 'via') == 1)
    ok('every row is a keepout row', all(r['type'] == 'keepout' for r in rows))
    rows2, acc2 = band_rows(path, baseline=path)
    ok('with the board as --baseline every row is inherited',
       rows2 == [] and acc2 and all(a.get('accepted') == 'inherited-keepout' for a in acc2),
       f"{len(rows2)} violation(s), {len(acc2)} accepted")


def main():
    with tempfile.TemporaryDirectory() as tmp:
        derive(tmp)
        controls(tmp)
        route(tmp)
        grade(tmp)
    if failures:
        print(f"FAIL: {len(failures)} check(s)")
        sys.exit(1)
    print('PASS')


if __name__ == '__main__':
    main()
