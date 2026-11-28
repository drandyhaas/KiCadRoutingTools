#!/usr/bin/env python3
"""#2024: a copper layer typed power is a plane. route.py keeps tracks off it
unless --layer-costs prices it, warns when a price lets them on, and
route_review reports the tracks on it.

The board, built here, is 4-layer, 30 x 20 mm: In1.Cu typed power with GND
poured over it, In2.Cu typed power with +3V3 poured over it, a GND flood on
F.Cu (typed signal), and the tracks
  SIG_A  F.Cu   x 4..26 at y 10;
  SIG_B  In2.Cu x 4..14 at y 12, across the +3V3 plane;
  +3V3   In2.Cu x 2..4 at y 15, the plane's own net.

Rows:
  1. the parser keeps each copper layer's type, and plane_layers is In1.Cu
     and In2.Cu; with In2.Cu typed signal, its board-wide pour does not make
     it a plane;
  2. route.py: with no --layer-costs both planes are forbidden and nothing is
     warned about; at 1 1 1 1 both planes are warned about; at 1 -1 2 1 only
     In2.Cu, with its price; at 1 -1 3 1 (PLANE_FALLBACK_COST, autoroute's
     later-pass price) nothing;
  3. route_review: SIG_B's 10 mm on In2.Cu is a plane track, the +3V3 track
     is not, and In2.Cu leaves the congestion grid's signal layers.
"""
import contextlib
import io
import os
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
for _d in ('py_tools', 'py_router', 'py_placer'):
    sys.path.insert(0, os.path.join(REPO, _d))
sys.path.insert(0, TESTS)
from run_utils import check  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
ROUTE = os.path.join(REPO, 'py_router', 'route.py')
FAILS = []


def _zone(net_id, net, layer):
    return (f'  (zone (net {net_id}) (net_name "{net}") (layer "{layer}") (hatch edge 0.5)\n'
            f'    (connect_pads (clearance 0.3)) (min_thickness 0.2)\n'
            f'    (fill (thermal_gap 0.3) (thermal_bridge_width 0.3))\n'
            f'    (polygon (pts (xy 0.5 0.5) (xy 29.5 0.5) (xy 29.5 19.5) (xy 0.5 19.5))))\n')


def _pad(ref, x, y, net_id, net):
    return (f'  (footprint "synth:P" (layer "F.Cu") (at {x} {y})\n'
            f'    (property "Reference" "{ref}" (at 0 -2 0) (layer "F.SilkS"))\n'
            f'    (pad "1" smd rect (at 0 0) (size 0.8 0.8) (layers "F.Cu" "F.Mask")'
            f' (net {net_id} "{net}")))\n')


def board(in2_type='power'):
    return ('(kicad_pcb (version 20240108) (generator pcbnew)\n'
            '  (general (thickness 1.6))\n'
            f'  (layers (0 "F.Cu" signal) (1 "In1.Cu" power) (2 "In2.Cu" {in2_type})'
            ' (31 "B.Cu" signal) (37 "F.SilkS" user) (39 "F.Mask" user) (44 "Edge.Cuts" user))\n'
            '  (net 0 "") (net 1 "GND") (net 2 "+3V3") (net 3 "SIG_A") (net 4 "SIG_B")\n'
            '  (gr_rect (start 0 0) (end 30 20) (stroke (width 0.1) (type default))'
            ' (layer "Edge.Cuts"))\n'
            + _pad('A1', 4, 10, 3, 'SIG_A') + _pad('A2', 26, 10, 3, 'SIG_A')
            + _pad('B1', 4, 12, 4, 'SIG_B') + _pad('B2', 14, 12, 4, 'SIG_B') +
            '  (segment (start 4 10) (end 26 10) (width 0.2) (layer "F.Cu") (net 3))\n'
            '  (segment (start 4 12) (end 14 12) (width 0.2) (layer "In2.Cu") (net 4))\n'
            '  (segment (start 2 15) (end 4 15) (width 0.2) (layer "In2.Cu") (net 2))\n'
            + _zone(1, 'GND', 'In1.Cu') + _zone(1, 'GND', 'F.Cu')
            + _zone(2, '+3V3', 'In2.Cu') + ')\n')


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def _write(path, text):
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(text)
    return path


def main():
    from kicad_parser import parse_kicad_pcb
    import route_review
    with tempfile.TemporaryDirectory() as td:
        b = _write(os.path.join(td, 'planes.kicad_pcb'), board())
        signal = _write(os.path.join(td, 'signal.kicad_pcb'), board(in2_type='signal'))
        with contextlib.redirect_stdout(io.StringIO()):
            info = parse_kicad_pcb(b).board_info
            signal_planes = parse_kicad_pcb(signal).board_info.plane_layers

        print("1. plane layers")
        ok(info.copper_layer_types == {'F.Cu': 'signal', 'In1.Cu': 'power',
                                       'In2.Cu': 'power', 'B.Cu': 'signal'},
           f"layer types: {info.copper_layer_types}")
        ok(info.plane_layers == ['In1.Cu', 'In2.Cu'], f"planes: {info.plane_layers}")
        ok(signal_planes == ['In1.Cu'], f"In2.Cu typed signal is no plane: {signal_planes}")

        print("2. route.py's default and warning")
        for costs, want_costs, want_warned in (
                ([], 'F.Cu=1.0x, In1.Cu=-1x, In2.Cu=-1x, B.Cu=1.0x', []),
                (['1', '1', '1', '1'], 'F.Cu=1.0x, In1.Cu=1.0x, In2.Cu=1.0x, B.Cu=1.0x',
                 ['In1.Cu', 'In2.Cu']),
                (['1', '-1', '2', '1'], 'F.Cu=1.0x, In1.Cu=-1.0x, In2.Cu=2.0x, B.Cu=1.0x',
                 ['In2.Cu']),
                (['1', '-1', '3', '1'], 'F.Cu=1.0x, In1.Cu=-1.0x, In2.Cu=3.0x, B.Cu=1.0x', [])):
            r = check(PY + [ROUTE, b, os.path.join(td, 'out.kicad_pcb'), '--nets', 'SIG_A',
                            '--layers', 'F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu', '--skip-routing']
                      + (['--layer-costs', *costs] if costs else []), accept=True)
            label = f"--layer-costs {' '.join(costs) or '(none)'}"
            warned = [ln.split()[1] for ln in r.stdout.splitlines()
                      if ln.strip().startswith('WARNING:') and 'typed power' in ln]
            ok(f"Layer costs: {want_costs}" in r.stdout, f"{label}: costs {want_costs}")
            ok(warned == want_warned, f"{label}: warned {warned}")
            if costs == ['1', '-1', '2', '1']:
                ok("In2.Cu is typed power (a plane) but routable at 2.0x" in r.stdout,
                   "the warning names the layer and its price")

        print("3. route_review's plane tracks")
        with contextlib.redirect_stdout(io.StringIO()):
            rev = route_review.review(b)
        ok(rev['plane_tracks'] == [{'layer': 'In2.Cu', 'net': 'SIG_B', 'length_mm': 10.0}],
           f"SIG_B's 10 mm on In2.Cu, the +3V3 track left out: {rev['plane_tracks']}")
        ok(rev['counts']['plane_track_mm'] == 10.0, f"counted: {rev['counts']}")
        ok('In2.Cu' not in rev['layers_with_signal'],
           f"In2.Cu is not a signal layer: {rev['layers_with_signal']}")

    if FAILS:
        print(f"\nFAILED ({len(FAILS)}):")
        for f in FAILS:
            print(f"  - {f}")
        return 1
    print("\nPASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
