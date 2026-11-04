#!/usr/bin/env python3
"""A dead end is anchored only by copper on ITS OWN layer (#1524, #1525).

Two dead-end cleanups kept a dangling tail because they credited copper on
another layer as the anchor of its free end, and KiCad graded the tail
`track_dangling`:

  #1524  trim_dangles_past_body_anchor's exact pad test read the pad outline
         without its layers, so an In1.Cu free end over a B.Cu-only pad of its
         net counted as landing on that pad, and the tail past the via on the
         segment's body was never cut back.
  #1525  make_model_fill_anchor, asked about a layer with no fill model,
         handed the question to its fallback, which only checks that no foreign
         copper is near. A net poured on In1.Cu alone therefore read as
         fill-anchored at every quiet free end on B.Cu, and sweep_dead_ends kept
         those stubs.

Every case carries its control: the same fixture with the anchor really
there (a pad on the segment's layer, a pour on the stub's layer) must still
keep the copper, and the fixture with the anchor moved away must trim -- else
the fixture proves nothing.

    python3 tests/test_1524_1525_dead_end_anchors.py
"""
import os
import sys
import tempfile

RUN_ALL_TIMEOUT = 120
#: seconds; runs no chain and starts no child process.
RUN_ALL_FAST_OK = True

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'rust_router'))

from kicad_parser import BoardInfo, parse_kicad_pcb               # noqa: E402
from synth import make_net, make_pad, make_pcb, make_seg, make_via  # noqa: E402
import pcb_modification as pm                                     # noqa: E402

FAILURES = []


def check(name, ok, detail=''):
    print(f"  {'OK  ' if ok else 'FAIL'} {name}{('  -- ' + detail) if detail else ''}")
    if not ok:
        FAILURES.append(name)


# --- #1524: the trim's pad test ----------------------------------------------

LAYERS4 = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']


def trim_case(third_pad_x, third_pad_layers, drill=0.0, pad_type='smd'):
    """In1.Cu track (10,10)-(14,10); through vias at x=10 and x=12, each over a
    B.Cu pad of the net; a third pad of the net at (third_pad_x, 10). The free
    end is (14,10), and the tail past the via at x=12 is 2 mm. Returns the
    track's ends after the trim."""
    pads = [make_pad(1, 10, 10, ref='U1', layers=('B.Cu',)),
            make_pad(1, 12, 10, ref='U2', layers=('B.Cu',)),
            make_pad(1, third_pad_x, 10, ref='U3', layers=third_pad_layers,
                     drill=drill, pad_type=pad_type)]
    bi = BoardInfo(layers={0: 'F.Cu', 4: 'In1.Cu', 6: 'In2.Cu', 2: 'B.Cu'},
                   copper_layers=list(LAYERS4),
                   board_bounds=(0.0, 0.0, 30.0, 20.0))
    seg = make_seg(10, 10, 14, 10, layer='In1.Cu', width=0.1)
    pcb = make_pcb(nets={1: make_net(1, 'A')}, segments=[seg],
                   vias=[make_via(10, 10, size=0.3, drill=0.15,
                                  layers=('F.Cu', 'B.Cu')),
                         make_via(12, 10, size=0.3, drill=0.15,
                                  layers=('F.Cu', 'B.Cu'))],
                   pads_by_net={1: pads}, board_info=bi)
    pm.trim_dangles_past_body_anchor([], pcb, {1})
    (s,) = [x for x in pcb.segments if x.net_id == 1]
    return (round(s.start_x, 3), round(s.end_x, 3))


def test_1524():
    print("#1524 trim_dangles_past_body_anchor: a pad anchors only on its layers")
    ends = trim_case(16, ('B.Cu',))
    check("control: no pad under the free end -> tail trimmed to the via",
          ends == (10.0, 12.0), f"ends {ends}")
    ends = trim_case(14, ('B.Cu',))
    check("free end over a B.Cu-only pad of its net -> tail still trimmed",
          ends == (10.0, 12.0), f"ends {ends}")
    ends = trim_case(14, ('In1.Cu',))
    check("control: pad on the track's own layer anchors the free end",
          ends == (10.0, 14.0), f"ends {ends}")
    ends = trim_case(14, ('*.Cu', '*.Mask'), drill=0.4, pad_type='thru_hole')
    check("control: a through-hole pad (*.Cu) anchors it on an inner layer",
          ends == (10.0, 14.0), f"ends {ends}")


# --- #1525: the fill anchor ---------------------------------------------------

def zone_text(layer):
    return f'''
  (zone (net 1) (net_name "GND") (layer "{layer}") (hatch edge 0.5)
    (connect_pads (clearance 0.2)) (min_thickness 0.2) (filled_areas_thickness no)
    (fill yes (thermal_gap 0.5) (thermal_bridge_width 0.5))
    (polygon (pts (xy 2 2) (xy 38 2) (xy 38 38) (xy 2 38))))'''


def plane_board(td, zone_layers):
    """GND pad on F.Cu at (10,20), F.Cu track to a through via at (15,20), and
    a B.Cu stub (15,20)-(20,20) off that via whose far end connects nothing.
    The net is poured on `zone_layers`."""
    zones = ''.join(zone_text(L) for L in zone_layers)
    text = f'''(kicad_pcb (version 20240108) (generator "test") (generator_version "8.0")
  (general (thickness 1.6) (legacy_teardrops no))
  (paper "A4")
  (layers (0 "F.Cu" signal) (1 "In1.Cu" signal) (2 "In2.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))
  (setup (pad_to_mask_clearance 0))
  (net 0 "")
  (net 1 "GND")
  (footprint "P:Pad" (layer "F.Cu") (at 10 20)
    (property "Reference" "U1" (at 0 -1.5 0) (layer "F.SilkS") (hide yes) (effects (font (size 1 1) (thickness 0.15))))
    (property "Value" "P" (at 0 1.5 0) (layer "F.Fab") (hide yes) (effects (font (size 1 1) (thickness 0.15))))
    (attr smd)
    (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 1 "GND")))
  (gr_rect (start 0 0) (end 40 40) (stroke (width 0.1) (type default)) (fill none) (layer "Edge.Cuts"))
  (segment (start 10 20) (end 15 20) (width 0.2) (layer "F.Cu") (net 1))
  (via (at 15 20) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") (net 1))
  (segment (start 15 20) (end 20 20) (width 0.2) (layer "B.Cu") (net 1)){zones}
)
'''
    tag = '_'.join(L.replace('.', '') for L in zone_layers) or 'none'
    path = os.path.join(td, f'p_{tag}.kicad_pcb')
    with open(path, 'w') as f:
        f.write(text)
    with open(path[:-len('.kicad_pcb')] + '.kicad_pro', 'w') as f:
        f.write('{}')
    return parse_kicad_pcb(path)


def b_stub_kept(pcb):
    pm.sweep_dead_ends([], pcb, {1})
    return any(s.layer == 'B.Cu' for s in pcb.segments)


def test_1525():
    print("#1525 make_model_fill_anchor: fill anchors only on the net's zone layers")
    from check_connected import make_real_fill_validator
    td = tempfile.mkdtemp(prefix='t1525_')

    pcb = plane_board(td, ['In1.Cu'])
    fa = pm.make_model_fill_anchor(
        pcb, 1, fallback=make_real_fill_validator(pcb, 1, margin=0.02))
    check("pour on In1.Cu only: the B.Cu free end is not fill-anchored",
          fa is not None and not fa(20, 20, 'B.Cu'))
    check("control: the In1.Cu pour still anchors a point inside it",
          fa is not None and bool(fa(20, 20, 'In1.Cu')))
    check("pour on In1.Cu only: sweep_dead_ends removes the B.Cu stub",
          not b_stub_kept(plane_board(td, ['In1.Cu'])))
    check("control: no pour at all -> the stub is swept",
          not b_stub_kept(plane_board(td, [])))
    check("control: a pour on B.Cu anchors the stub's end -> kept",
          b_stub_kept(plane_board(td, ['In1.Cu', 'B.Cu'])))

    # A zone whose fill model cannot be built leaves the fallback to answer,
    # and it may only answer inside that zone's outline on that zone's layer.
    import plane_fill_model
    real = plane_fill_model.get_fill_models
    plane_fill_model.get_fill_models = lambda pcb_data, net_id: {}
    try:
        pcb = plane_board(td, ['In1.Cu'])
        fa = pm.make_model_fill_anchor(pcb, 1, fallback=lambda x, y, layer: True)
        check("no model: the fallback answers inside the In1.Cu outline",
              fa is not None and fa(20, 20, 'In1.Cu') is True)
        check("no model: the fallback is not asked on B.Cu (no zone there)",
              fa is not None and fa(20, 20, 'B.Cu') is False)
        check("no model: nor outside the outline on In1.Cu",
              fa is not None and fa(39, 39, 'In1.Cu') is False)
    finally:
        plane_fill_model.get_fill_models = real


def main():
    test_1524()
    test_1525()
    if FAILURES:
        print(f"\nFAIL: {len(FAILURES)} check(s): {FAILURES}")
        return 1
    print("\nPASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
