#!/usr/bin/env python3
"""Connectivity joins a pad through the copper KiCad joins it through.

Three cases where our copper grade passed a net KiCad 10.0.6 reports open:

  1. PER-LAYER PADSTACK -- a pad that is a 2.54 x 1.27 oval on F.Cu and a
     larger pad on B.Cu is modelled at the max over its layers (clearance
     keeps that). A track on F.Cu ending inside only the B.Cu copper is not
     joined; the same track on B.Cu is. `Pad.layer_copper` carries each
     smaller layer and `pad_on_layer` reads it.
  2. CUSTOM PAD RING -- KiCad's connectivity joins an UNFILLED stroked circle
     primitive only along its centreline (measured: a 1.5 mm-stroke spacer
     ring joins a track end at the stroke's centre radius and at none 0.1 mm
     either side). A track reaching r = 3.3 of a ring drawn at r = 2.95 is
     open; one reaching the centreline is joined. `Pad.join_polygons`.
  3. LINK END LAYER -- classify_unconnected_link matches a KiCad link end to
     copper on that end's layer: an art island on B.Cu whose corner
     coincides with a live F.Cu art twin is `graphic`, not `live`.

    python3 tests/test_pad_layer_joins.py
"""
import contextlib
import io
import math
import os
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

import run_utils                                                 # noqa: E402
from kicad_parser import parse_kicad_pcb, pad_on_layer           # noqa: E402
from check_connected import (check_net_connectivity,             # noqa: E402
                             classify_unconnected_link)

failures = []

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
\t\t(1 "F.Mask" user)
\t\t(3 "B.Mask" user)
\t\t(5 "F.SilkS" user "F.Silkscreen")
\t\t(25 "Edge.Cuts" user)
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
    return ('\t\t(property "Reference" "%s"\n\t\t\t(at 0 0 0)\n\t\t\t(layer "F.SilkS")\n'
            '\t\t\t(effects\n\t\t\t\t(font\n\t\t\t\t\t(size 1 1)\n\t\t\t\t)\n\t\t\t)\n\t\t)\n' % ref)


def _fp(ref, x, y, pads):
    return ('\t(footprint "Test:P"\n\t\t(layer "F.Cu")\n\t\t(at %g %g)\n' % (x, y)
            + _ref(ref) + ''.join(pads) + '\t)\n')


def _smd(num, x, y, net, size=1.0, layer='F.Cu'):
    return ('\t\t(pad "%s" smd rect\n\t\t\t(at %g %g)\n\t\t\t(size %g %g)\n'
            '\t\t\t(layers "%s")\n\t\t\t(net "%s")\n\t\t)\n'
            % (num, x, y, size, size, layer, net))


def _track(x1, y1, x2, y2, net, layer='F.Cu', w=0.25):
    return ('\t(segment\n\t\t(start %g %g)\n\t\t(end %g %g)\n\t\t(width %g)\n'
            '\t\t(layer "%s")\n\t\t(net "%s")\n\t)\n' % (x1, y1, x2, y2, w, layer, net))


def _rect_art(x0, y0, x1, y1, net, layer):
    return ('\t(gr_rect\n\t\t(start %g %g)\n\t\t(end %g %g)\n'
            '\t\t(stroke\n\t\t\t(width 0.2)\n\t\t\t(type solid)\n\t\t)\n'
            '\t\t(fill yes)\n\t\t(layer "%s")\n\t\t(net "%s")\n\t)\n'
            % (x0, y0, x1, y1, layer, net))


def write(path, parts):
    with open(path, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write(HEAD + ''.join(parts) + ')\n')
    return run_utils.evidence(path, 'pad-join board')


def parse(path):
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        return parse_kicad_pcb(path)


def connected(pcb, net):
    nid = next(k for k, n in pcb.nets.items() if n.name == net)
    segs = [s for s in pcb.segments if s.net_id == nid]
    with contextlib.redirect_stdout(io.StringIO()):
        return check_net_connectivity(nid, segs, [v for v in pcb.vias if v.net_id == nid],
                                      pcb.pads_by_net[nid], [], pcb_data=pcb)['connected']


PADSTACK = ('\t\t(pad "1" thru_hole oval\n\t\t\t(at 0 0)\n\t\t\t(size 2.54 1.27)\n'
            '\t\t\t(drill 0.889)\n\t\t\t(layers "*.Cu" "*.Mask")\n\t\t\t(net "A")\n'
            '\t\t\t(padstack\n\t\t\t\t(mode custom)\n\t\t\t\t(layer "B.Cu"\n'
            '\t\t\t\t\t(shape oval)\n\t\t\t\t\t(size 3.6 1.8)\n\t\t\t\t)\n\t\t\t)\n\t\t)\n')


def check_padstack(td):
    print('per-layer padstack:')
    # Track end at x = 11.6: 1.6 from P1's centre. F.Cu oval half-length
    # 1.27 + track half-width 0.125 = 1.395 short of it; B.Cu 1.8 reaches it.
    for layer, want in (('F.Cu', False), ('B.Cu', True)):
        pcb = parse(write(os.path.join(td, f'ps_{layer[0]}.kicad_pcb'), [
            _fp('P1', 10, 10, [PADSTACK]),
            _fp('P2', 20, 10, [_smd('1', 0, 0, 'A', layer=layer)]),
            _track(20, 10, 11.6, 10, 'A', layer=layer)]))
        pad = pcb.footprints['P1'].pads[0]
        if layer == 'F.Cu':
            f = pad_on_layer(pad, 'F.Cu')
            ok('the max model is the B.Cu copper (clearance keeps it)',
               abs(pad.size_x - 3.6) < 1e-6, f'{pad.size_x}x{pad.size_y}')
            ok('F.Cu reads its own 2.54 x 1.27 oval',
               (round(f.size_x, 3), round(f.size_y, 3), f.shape) == (2.54, 1.27, 'oval'),
               f'{f.size_x}x{f.size_y} {f.shape}')
            ok('B.Cu reads the max model', pad_on_layer(pad, 'B.Cu') is pad)
        got = connected(pcb, 'A')
        ok(f'a {layer} track ending in B.Cu-only copper is '
           f'{"joined" if want else "open"}', got == want, f'connected={got}')


RING = ('\t\t(pad "1" smd custom\n\t\t\t(at 2.95 0)\n\t\t\t(size 1.1 1.1)\n'
        '\t\t\t(layers "F.Cu" "F.Mask")\n\t\t\t(net "R")\n'
        '\t\t\t(options\n\t\t\t\t(clearance outline)\n\t\t\t\t(anchor circle)\n\t\t\t)\n'
        '\t\t\t(primitives\n\t\t\t\t(gr_circle\n\t\t\t\t\t(center -2.95 0)\n'
        '\t\t\t\t\t(end 0 0)\n\t\t\t\t\t(width 1.5)\n\t\t\t\t\t(fill no)\n'
        '\t\t\t\t)\n\t\t\t)\n\t\t)\n')


def check_ring(td):
    print('custom pad ring (joined only along its centreline):')
    cx, cy, a = 15.0, 15.0, math.radians(120)
    for r, want in ((3.3, False), (2.95, True)):
        x, y = cx + r * math.cos(a), cy + r * math.sin(a)
        far = (cx + 7 * math.cos(a), cy + 7 * math.sin(a))
        pcb = parse(write(os.path.join(td, f'ring_{r}.kicad_pcb'), [
            _fp('H1', cx, cy, [RING]),
            _fp('R2', far[0], far[1], [_smd('1', 0, 0, 'R')]),
            _track(far[0], far[1], x, y, 'R', w=0.1)]))
        pad = next(p for p in pcb.footprints['H1'].pads if p.shape == 'custom')
        if r == 3.3:
            ok('the ring pad carries a join outline', bool(pad.join_polygons))
        got = connected(pcb, 'R')
        ok(f'a track reaching r = {r} of a ring drawn at r = 2.95 is '
           f'{"joined" if want else "open"}', got == want, f'connected={got}')


def check_link_layer(td):
    print('link end matched on its own layer:')
    pcb = parse(write(os.path.join(td, 'art.kicad_pcb'), [
        _fp('U1', 5, 5, [_smd('1', 0, 0, 'N'), _smd('2', 0, 10, 'N')]),
        _track(5, 5, 5, 15, 'N'),
        # F.Cu art joined to the net's track; a B.Cu island whose corner
        # coincides with the F.Cu art's
        _track(5, 15, 10, 15, 'N'),
        _rect_art(10, 14, 12, 16, 'N', 'F.Cu'),
        _rect_art(10, 14, 12, 18, 'N', 'B.Cu')]))
    nid = next(k for k, n in pcb.nets.items() if n.name == 'N')
    with contextlib.redirect_stdout(io.StringIO()):
        cls = classify_unconnected_link(pcb, nid, (10.0, 14.0, 'B.Cu', 'track'),
                                        (5.0, 15.0, 'F.Cu', 'track'), 'track', 'track')
    ok('the B.Cu island end is graphic, the F.Cu end live',
       cls == ('graphic', 'live'), f'{cls}')


def main():
    with tempfile.TemporaryDirectory() as td:
        check_padstack(td)
        check_ring(td)
        check_link_layer(td)
    print('PASS' if not failures else f'FAIL ({len(failures)}): {failures}')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
