"""
Tests for RefLabel parsing (#481, text-parser path).

The Reference sub-node is scoped with `find_matching_paren` before any regex
runs, so a sibling property's `(at ...)` / `(effects ...)` can never bleed in
(the discipline the courtyard reader learned in #456 item 3). The real-board
expectations below were read out of the board files by hand.

The pcbnew-path twin (`build_pcb_data_from_board`) is pinned against this
parser by tests/gui_parity/test_ref_label_pcbnew_parity.py.
"""

import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_router'))  # #522
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_placer'))  # placement split
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_tools'))  # #522

from kicad_parser import RefLabel, parse_kicad_pcb

KICAD_FILES = os.path.join(os.path.dirname(__file__), '..', 'kicad_files')


def _board(footprints):
    """Write a minimal board carrying `footprints` text; returns the path."""
    body = ('(kicad_pcb\n\t(version 20241229)\n\t(net 0 "")\n'
            '\t(gr_rect\n\t\t(start 0 0)\n\t\t(end 100 100)\n'
            '\t\t(layer "Edge.Cuts")\n\t\t(uuid "e1")\n\t)\n'
            + footprints + ')\n')
    fd, path = tempfile.mkstemp(suffix='.kicad_pcb')
    with os.fdopen(fd, 'w') as f:
        f.write(body)
    return path


# --- real boards: modern (property "Reference" ...) form ---------------------

def test_splitflap_known_values():
    pcb = parse_kicad_pcb(os.path.join(KICAD_FILES,
                                       'splitflap_driver.kicad_pcb'))
    # C1's Reference node, verbatim from the file:
    #   (at 0 1.27 90) (layer "F.SilkS") (effects (font (size 1 1)
    #   (thickness 0.15)))
    label = pcb.footprints['C1'].ref_label
    assert label is not None
    assert label.at_x == 0.0 and label.at_y == 1.27
    assert label.rotation == 90.0
    assert label.size_h == 1.0 and label.size_w == 1.0
    assert label.thickness == 0.15
    assert label.layer == 'F.SilkS'
    assert label.justify == ''
    assert not label.hidden
    assert label.is_property_node
    # Every real footprint on this board has a parseable label.
    missing = [r for r, fp in pcb.footprints.items()
               if fp.ref_label is None and not r.startswith('#')]
    assert missing == [], missing
    print("PASS test_splitflap_known_values")


def test_splitflap_hidden_count():
    """The board carries exactly 19 hidden Reference labels ((hide yes))."""
    pcb = parse_kicad_pcb(os.path.join(KICAD_FILES,
                                       'splitflap_driver.kicad_pcb'))
    hidden = sorted(r for r, fp in pcb.footprints.items()
                    if fp.ref_label is not None and fp.ref_label.hidden)
    assert len(hidden) == 19, hidden
    assert 'H1' in hidden and 'H7' in hidden
    print("PASS test_splitflap_hidden_count")


def test_glasgow_bside_mirror():
    """glasgow_revC: 94 B.SilkS labels, every one carrying (justify mirror)."""
    pcb = parse_kicad_pcb(os.path.join(KICAD_FILES, 'glasgow_revC.kicad_pcb'))
    b_side = [fp.ref_label for fp in pcb.footprints.values()
              if fp.ref_label is not None
              and fp.ref_label.layer == 'B.SilkS']
    assert len(b_side) == 94, len(b_side)
    assert all(lbl.justify == 'mirror' for lbl in b_side)
    print("PASS test_glasgow_bside_mirror")


# --- synthetic fixtures ------------------------------------------------------

def _fp(reference_node, ref='R1'):
    return f'''\t(footprint "test:FP"
\t\t(layer "F.Cu")
\t\t(uuid "fp-{ref}")
\t\t(at 50 50 90)
{reference_node}\t\t(property "Value" "10K"
\t\t\t(at 3.3 4.4 180)
\t\t\t(layer "F.Fab")
\t\t\t(effects
\t\t\t\t(font
\t\t\t\t\t(size 0.5 0.6)
\t\t\t\t\t(thickness 0.08)
\t\t\t\t)
\t\t\t)
\t\t)
\t\t(pad "1" smd rect
\t\t\t(at 0 0)
\t\t\t(size 0.6 0.8)
\t\t\t(layers "F.Cu")
\t\t\t(uuid "p1-{ref}")
\t\t)
\t)
'''


def test_property_node_scoping():
    """The VALUE property's (at ...)/(effects ...) must not bleed into the
    Reference label -- the sub-node slice is the whole point."""
    node = '''\t\t(property "Reference" "R1"
\t\t\t(at 1.1 -2.2 270)
\t\t\t(layer "B.SilkS")
\t\t\t(uuid "ref-r1")
\t\t\t(effects
\t\t\t\t(font
\t\t\t\t\t(size 0.9 0.8)
\t\t\t\t\t(thickness 0.12)
\t\t\t\t)
\t\t\t\t(justify left bottom mirror)
\t\t\t)
\t\t)
'''
    path = _board(_fp(node))
    try:
        label = parse_kicad_pcb(path).footprints['R1'].ref_label
    finally:
        os.unlink(path)
    assert label.at_x == 1.1 and label.at_y == -2.2
    assert label.rotation == 270.0
    assert label.layer == 'B.SilkS'
    assert label.size_h == 0.9 and label.size_w == 0.8
    assert label.thickness == 0.12
    assert label.justify == 'left bottom mirror'
    assert not label.hidden
    print("PASS test_property_node_scoping")


def test_missing_effects_defaults():
    """No effects block: KiCad's stock label font (1.0 / 0.15) applies."""
    node = '''\t\t(property "Reference" "R1"
\t\t\t(at 0 -1.5)
\t\t\t(layer "F.SilkS")
\t\t\t(uuid "ref-r1")
\t\t)
'''
    path = _board(_fp(node))
    try:
        label = parse_kicad_pcb(path).footprints['R1'].ref_label
    finally:
        os.unlink(path)
    assert label.rotation == 0.0
    assert label.size_h == 1.0 and label.size_w == 1.0
    assert label.thickness == 0.15
    print("PASS test_missing_effects_defaults")


def test_hide_yes_and_negative_rotation():
    node = '''\t\t(property "Reference" "R1"
\t\t\t(at 0 1.5 -90)
\t\t\t(hide yes)
\t\t\t(layer "F.SilkS")
\t\t\t(uuid "ref-r1")
\t\t)
'''
    path = _board(_fp(node))
    try:
        label = parse_kicad_pcb(path).footprints['R1'].ref_label
    finally:
        os.unlink(path)
    assert label.hidden
    # normalized to [0, 360) -- the parity contract with GetTextAngle() % 360
    assert label.rotation == 270.0
    print("PASS test_hide_yes_and_negative_rotation")


def test_bare_hide_property_node():
    """KiCad 8 wrote visibility as a bare `hide` token in property nodes."""
    node = '''\t\t(property "Reference" "R1"
\t\t\t(at 0 1.5)
\t\t\t(layer "F.SilkS")
\t\t\thide
\t\t\t(uuid "ref-r1")
\t\t)
'''
    path = _board(_fp(node))
    try:
        label = parse_kicad_pcb(path).footprints['R1'].ref_label
    finally:
        os.unlink(path)
    assert label.hidden
    print("PASS test_bare_hide_property_node")


def test_legacy_fp_text_form():
    """KiCad 6/7 (fp_text reference ...) with the bare `hide` token."""
    node = '''\t\t(fp_text reference "R1"
\t\t\t(at 0.5 -1.25 45)
\t\t\t(layer "F.SilkS")
\t\t\thide
\t\t\t(effects
\t\t\t\t(font
\t\t\t\t\t(size 1.2 1.1)
\t\t\t\t\t(thickness 0.2)
\t\t\t\t)
\t\t\t)
\t\t)
'''
    path = _board(_fp(node))
    try:
        label = parse_kicad_pcb(path).footprints['R1'].ref_label
    finally:
        os.unlink(path)
    assert label is not None
    assert not label.is_property_node
    assert label.at_x == 0.5 and label.at_y == -1.25
    assert label.rotation == 45.0
    assert label.size_h == 1.2 and label.size_w == 1.1
    assert label.thickness == 0.2
    assert label.hidden
    print("PASS test_legacy_fp_text_form")


def test_ref_label_appended_not_inserted():
    """Positional Footprint(...) constructions exist across the tree, so a new
    field may only be APPENDED -- never inserted before an existing one.

    This asserted `fields(Footprint)[-1].name == 'ref_label'`, which pinned the
    right property by the wrong means: it also forbade appending a *later*
    field, so #829's `owns_edge_cuts` tripped it while shifting nothing. Pin
    the PREFIX instead -- that is the invariant positional callers actually
    depend on, and it still fails loudly on a mid-list insert.
    """
    import dataclasses
    from kicad_parser import Footprint
    names = [f.name for f in dataclasses.fields(Footprint)]
    expected_prefix = [
        'reference', 'footprint_name', 'x', 'y', 'rotation', 'layer', 'pads',
        'value', 'dnp', 'locked', 'clearance', 'uuid', 'sheet_path',
        'net_tie_groups', 'ref_label',
    ]
    assert names[:len(expected_prefix)] == expected_prefix, (
        f"a Footprint field was inserted or reordered, which silently rebinds "
        f"every positional Footprint(...) in the tree:\n  {names}")
    assert isinstance(RefLabel(at_x=0, at_y=0, rotation=0), RefLabel)
    print("PASS test_ref_label_appended_not_inserted")


if __name__ == '__main__':
    test_splitflap_known_values()
    test_splitflap_hidden_count()
    test_glasgow_bside_mirror()
    test_property_node_scoping()
    test_missing_effects_defaults()
    test_hide_yes_and_negative_rotation()
    test_bare_hide_property_node()
    test_legacy_fp_text_form()
    test_ref_label_appended_not_inserted()
    print("All RefLabel parser tests passed")
