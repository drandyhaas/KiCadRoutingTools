#!/usr/bin/env python3
"""Plane pad repair leaves alone a pad an earlier tap already joined to the plane.

repair_planes builds its queue of unconnected plane pads once per net, and its
PlaneComponentOracle once per pass. note_tap_committed credited only the tapped
pad and its new copper to the main component, not the copper the pad already
had. So when two SMD pads of a net were joined by locked copper and neither had
a via, the first pad's tap joined both to the plane, yet the second still read
pad_on_plane() False and was tapped again: a redundant strap, or a new via that
can block another net's escape.

The board (generated below): an SMD QFN-like column U1.1-U1.4 and an 0402 C1 on
B.Cu, with locked B.Cu tracks U1.3-C1.1 (GND) and U1.2-C1.2 (VCC), so each
pair is a two-pad island before any plane via exists; J1 gives both nets
through-hole pads. GND is poured on In1.Cu and VCC on In2.Cu, then the pad
repair runs for GND and then VCC.

  1. Neither repair stage adds a via, or a track end, inside C1.1 or C1.2: the
     tap of U1.3 / U1.2 already reaches them through the locked track.
  2. Each repair stage's JSON_SUMMARY reports no unconnected pad, so the
     skipped pads are on the plane, not abandoned.

    python3 tests/test_plane_repair_joined_pads.py
"""
import contextlib
import io
import json
import os
import sys
import tempfile
import uuid

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

import run_utils
from kicad_parser import parse_kicad_pcb
from routing_utils import point_in_pad_rect

FAB_FLOOR = ("track_width = 0.09\nclearance = 0.1\nvia_drill = 0.20\n"
             "via_diameter = 0.45\nhole_to_hole = 0.2\npad_hole_to_hole = 0.45\n"
             "board_edge = 0.2\n")
PROJECT = ('{"board":{"design_settings":{"rules":'
           '{"min_copper_edge_clearance":0.2,"min_hole_clearance":0.2}}}}\n')
POUR_ARGS = ['--via-size', '0.45', '--via-drill', '0.2',
             '--same-net-pad-clearance', '0.15']
JOINED = ('C1.1', 'C1.2')   # the pads the locked tracks join to U1.3 / U1.2
failures = []

BACK = '"B.Cu" "B.Mask" "B.Paste"'


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def _uid():
    return f'(uuid "{uuid.uuid4()}")'


def _smd(number, x, y, w, h, net):
    return (f'\t\t(pad "{number}" smd roundrect (at {x} {y}) (size {w} {h}) '
            f'(layers {BACK}) (roundrect_rratio 0.25) (net "{net}") {_uid()})\n')


def _tht(number, x, y, net):
    return (f'\t\t(pad "{number}" thru_hole circle (at {x} {y}) (size 1.7 1.7) '
            f'(drill 1.0) (layers "*.Cu" "*.Mask") (net "{net}") {_uid()})\n')


def _footprint(ref, x, y, pads):
    return (f'\t(footprint "" (layer "B.Cu") {_uid()} (at {x} {y})\n'
            f'\t\t(property "Reference" "{ref}" (at 0 0 0) (layer "B.SilkS") '
            f'{_uid()} (effects (font (size 0.5 0.5)) (justify mirror)))\n'
            + "".join(pads) + "\t)\n")


def _locked_track(net, x1, y1, x2, y2):
    return (f'\t(segment (start {x1} {y1}) (end {x2} {y2}) (width 0.2) '
            f'(locked yes) (layer "B.Cu") (net "{net}") {_uid()})\n')


def board_text():
    """A 10 x 10 mm 4-layer board, every part on B.Cu."""
    # U1: pads 0.85 x 0.25 mm at 0.5 mm pitch, exposed GND pad to the left.
    u1 = _footprint("U1", 4.0, 5.0, [
        _smd("1", 0, -0.5, 0.85, 0.25, "GND"),
        _smd("2", 0, 0.0, 0.85, 0.25, "VCC"),
        _smd("3", 0, 0.5, 0.85, 0.25, "GND"),
        _smd("4", 0, 1.0, 0.85, 0.25, "SIG"),
        _smd("5", -2.45, -0.75, 3.6, 3.6, "GND"),
    ])
    # C1 (0402): pad 2 level with U1.2, pad 1 0.64 mm below it.
    c1 = _footprint("C1", 5.032, 5.32, [
        _smd("2", 0, -0.32, 0.4, 0.46, "VCC"),
        _smd("1", 0, 0.32, 0.4, 0.46, "GND"),
    ])
    # J1: through-hole pads, which each net's pour reaches without a via.
    j1 = _footprint("J1", 8.5, 2.0, [
        _tht(str(n + 1), 0, 2.54 * n, net)
        for n, net in enumerate(("GND", "VCC", "GND", "VCC"))])
    tracks = (_locked_track("VCC", 4.262, 5.0, 5.032, 5.0)
              + _locked_track("GND", 4.262, 5.5, 5.032, 5.64))
    edge = (f'\t(gr_rect (start 0 0) (end 10 10) (stroke (width 0.1) '
            f'(type default)) (fill no) (layer "Edge.Cuts") {_uid()})\n')
    return ('(kicad_pcb (version 20260206) (generator "pcbnew") '
            '(generator_version "10.0")\n'
            '\t(general (thickness 1.6) (legacy_teardrops no))\n'
            '\t(paper "A4")\n'
            '\t(layers (0 "F.Cu" signal) (4 "In1.Cu" signal) '
            '(6 "In2.Cu" signal) (2 "B.Cu" signal)\n'
            '\t\t(13 "F.Paste" user) (15 "B.Paste" user) '
            '(5 "F.SilkS" user "F.Silkscreen")\n'
            '\t\t(7 "B.SilkS" user "B.Silkscreen") (1 "F.Mask" user) '
            '(3 "B.Mask" user)\n'
            '\t\t(25 "Edge.Cuts" user) (31 "F.CrtYd" user "F.Courtyard") '
            '(29 "B.CrtYd" user "B.Courtyard")\n'
            '\t\t(35 "F.Fab" user) (33 "B.Fab" user))\n'
            '\t(setup (pad_to_mask_clearance 0))\n'
            + edge + u1 + c1 + j1 + tracks + ')\n')


def load(path):
    run_utils.evidence(path, 'a board this test wrote')
    with contextlib.redirect_stdout(io.StringIO()):
        return parse_kicad_pcb(path)


def _key(x, y):
    return (round(x, 4), round(y, 4))


def added_copper(before, after):
    """Vias and segment ends on `after` that `before` does not have."""
    old_vias = {_key(v.x, v.y) for v in before.vias}
    old_ends = {(_key(x, y), s.layer) for s in before.segments
                for x, y in ((s.start_x, s.start_y), (s.end_x, s.end_y))}
    vias = [v for v in after.vias if _key(v.x, v.y) not in old_vias]
    ends = [(x, y, s.layer) for s in after.segments
            for x, y in ((s.start_x, s.start_y), (s.end_x, s.end_y))
            if (_key(x, y), s.layer) not in old_ends]
    return vias, ends


def joined_pads(pcb):
    pads = {f"{p.component_ref}.{p.pad_number}": p
            for plist in pcb.pads_by_net.values() for p in plist}
    missing = [name for name in JOINED if name not in pads]
    if missing:
        raise AssertionError(f"BROKEN TEST: the board has no pad {missing}")
    return [pads[name] for name in JOINED]


def summary_of(log):
    lines = [ln for ln in log.splitlines() if ln.startswith('JSON_SUMMARY: ')]
    if not lines:
        raise AssertionError('BROKEN TEST: repair_planes printed no JSON_SUMMARY')
    return json.loads(lines[-1].split(': ', 1)[1])


def graded(stage, before_path, after_path, log):
    before, after = load(before_path), load(after_path)
    vias, ends = added_copper(before, after)
    for pad in joined_pads(after):
        name = f"{pad.component_ref}.{pad.pad_number}"
        on_pad_vias = [(v.x, v.y) for v in vias
                       if point_in_pad_rect(v.x, v.y, pad)]
        on_pad_ends = [(x, y) for x, y, layer in ends
                       if layer in pad.layers and point_in_pad_rect(x, y, pad)]
        check(f"{stage}: no via added in {name}", not on_pad_vias,
              str(on_pad_vias))
        check(f"{stage}: no track end added in {name}", not on_pad_ends,
              f"{len(on_pad_ends)} end(s) {on_pad_ends}")
    summary = summary_of(log)
    check(f"{stage}: JSON_SUMMARY grades the board it wrote",
          'unconnected_pads' in summary, str(sorted(summary)))
    check(f"{stage}: no unconnected pad",
          summary.get('unconnected_pads') == [],
          str(summary.get('unconnected_pads')))


def main():
    with tempfile.TemporaryDirectory() as tmp:
        def path(name):
            return os.path.join(tmp, name)

        with open(path('input.kicad_pcb'), 'w', encoding='utf-8') as f:
            f.write(board_text())
        with open(path('input.kicad_pro'), 'w', encoding='utf-8') as f:
            f.write(PROJECT)
        with open(path('floors.txt'), 'w', encoding='utf-8') as f:
            f.write(FAB_FLOOR)
        repair_args = [
            '--fab-overrides', path('floors.txt'), '--escalation', 'fab',
            '--track-width', '0.127', '--max-track-width', '0.127',
            '--min-track-width', '0.09', '--max-search-radius', '2',
            '--clearance', '0.15', '--via-size', '0.45', '--via-drill', '0.2',
            '--hole-to-hole-clearance', '0.2', '--board-edge-clearance', '0.2',
            '--same-net-pad-clearance', '0.15',
            '--layers', 'F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu',
            '--layer-costs', '1', '-1', '-1', '1']
        print("1. pour GND on In1.Cu and VCC on In2.Cu")
        run_utils.run_step(
            [run_utils.tool('route_planes.py'), path('input.kicad_pcb'),
             path('plane1.kicad_pcb'), '--nets', 'GND',
             '--plane-layers', 'In1.Cu', *POUR_ARGS],
            outputs=[path('plane1.kicad_pcb')])
        run_utils.run_step(
            [run_utils.tool('route_planes.py'), path('plane1.kicad_pcb'),
             path('plane2.kicad_pcb'), '--nets', 'VCC',
             '--plane-layers', 'In2.Cu', *POUR_ARGS],
            outputs=[path('plane2.kicad_pcb')])
        stages = (('GND', 'In1.Cu', 'plane2', 'drops1'),
                  ('VCC', 'In2.Cu', 'drops1', 'drops2'))
        for n, (net, layer, src, out) in enumerate(stages, start=2):
            print(f"{n}. repair the {net} plane pads")
            _rc, log = run_utils.run_step(
                [run_utils.tool('repair_planes.py'), path(f'{src}.kicad_pcb'),
                 path(f'{out}.kicad_pcb'), '--nets', net,
                 '--plane-layers', layer, *repair_args],
                outputs=[path(f'{out}.kicad_pcb')])
            graded(f"{net} repair", path(f'{src}.kicad_pcb'),
                   path(f'{out}.kicad_pcb'), log)
    if failures:
        print(f"\n{len(failures)} check(s) FAILED")
        return 1
    print("\nall checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
