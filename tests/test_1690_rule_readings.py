#!/usr/bin/env python3
"""#1690: how a `.kicad_dru` condition reads, held to kicad-cli.

`READINGS` is {arm: (classes, assignments, {condition: the items it holds on})},
measured on kicad-cli 10.0.6. KiCad's `assertion` constraint evaluates a condition
on one item with no B, so no swap retry, and reports each item it fails on under
the rule's name: one DRC pass answers 14 rows on all 13 items, where an edge
clearance table needs a run per row. (Past about 199 failures of one type KiCad
stops looking, `--all-track-errors` or not, which is why 14: 14 x 13 is under it.)

What the table pins:
  1. isPlated() holds on a via of any type and a through-hole pad (castellated
     too), and on no SMD, connector or NPTH pad, track, zone or graphic;
  2. a net's class reads as KiCad's effective class: its classes by priority,
     then Default when they leave a field unset between them, the name those
     joined. So `A.NetClass == 'Default'` and `A.hasNetclass('Default')` are NOT
     true on a net in a class with every field set, and ARE on one with a gap;
  3. Pad_Type and Via_Type read KiCad's names, 'Through-hole', 'NPTH, mechanical',
     'Blind', compared as a net name is;
  4. a literal with no closing quote runs to the end of the condition;
  5. a term on a B there is not is false, `A.Net != B.Net` too.

`LAYER_ROWS` and `PART_ROWS` do the same on a second fixture, for
existsOnLayer() (wildcards by wxString::Matches, case-sensitive, a pad's mask and
paste layers, every copper layer for `*.Cu`) and memberOfFootprint() (the reference
by the same match; the library id too when the name holds a ':'; the reference as the
file spells it on a second block claiming it). `COURTYARD_ARGS` holds
`PadEdgeRules`' reading of a courtyard term as memberOfFootprint() to KiCad, and
`EDGE_LIBRARY` holds `for_pad` to an edge rule naming a library.

`COMPILES` / `REFUSED` are conditions KiCad does and does not compile: a refused
one makes it enforce no rule of the file, with exit 0 and nothing printed, which
`kicad_condition_refusal` models and the run says. `FILES_REFUSED` are the same
for what surrounds a condition. `ZONE` holds `PadEdgeRules.for_zone` to the filler,
which retries a failed condition with the Edge.Cuts shape as A, as the DRC does.

Cost with kicad-cli (the fixture is a dozen items): `READINGS` about 10 runs,
`COMPILES` one, each row of `REFUSED` and `FILES_REFUSED` one on three workers,
`ZONE` one each: under 4 minutes on a laptop busy with other runs, 100 s of it `REFUSED`.
Without KiCad the model half runs in seconds.

    python3 tests/test_1690_rule_readings.py
"""
import concurrent.futures
import json
import os
import re
import subprocess
import sys
import tempfile
import unittest

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)

import run_utils  # noqa: E402
from kicad_exact_fill import find_kicad_python  # noqa: E402
from kicad_oracle import find_kicad_cli  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402
import design_rules as dr_mod  # noqa: E402
from design_rules import DesignRules, PadEdgeRules, kicad_condition_refusal  # noqa: E402

RUN_ALL_TIMEOUT = 900
CLI = find_kicad_cli(warn=False)
WORKERS = 3     # kicad-cli at once, for the rows that each need their own file

_FP = ('(footprint "x:{name}" (layer "F.Cu") (uuid "f0000000-0000-0000-0000-00000000000{i}") '
       '(at {x} 10)\n  (property "Reference" "{name}1" (at 0 -2 0) (layer "F.SilkS") '
       '(uuid "f1000000-0000-0000-0000-00000000000{i}") '
       '(effects (font (size 1 1) (thickness 0.15))))\n  {pad})\n')
_ITEMS = {
    'smd': '(pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu" "F.Mask") (net 1 "A") ',
    'pth': '(pad "1" thru_hole circle (at 0 0) (size 1.6 1.6) (drill 0.8) (layers "*.Cu" "*.Mask") '
           '(net 2 "HSNET") ',
    'npth': '(pad "1" np_thru_hole circle (at 0 0) (size 1.6 1.6) (drill 1.6) (layers "*.Cu" "*.Mask") ',
    'connect': '(pad "1" connect rect (at 0 0) (size 1 1) (layers "F.Cu" "F.Mask") (net 1 "A") ',
    'castellated': '(pad "1" thru_hole circle (at 0 0) (size 1.6 1.6) (drill 0.8) '
                   '(property pad_prop_castellated) (layers "*.Cu" "*.Mask") (net 1 "A") ',
}
REFS = {'smd': 'S', 'pth': 'P', 'npth': 'H', 'connect': 'C', 'castellated': 'E'}
_PARTS = ''.join(
    _FP.format(name=REFS[name], i=i + 1, x=10 + 10 * i,
               pad=f'{body}(uuid "a0000000-0000-0000-0000-00000000000{i + 1}"))')
    for i, (name, body) in enumerate(_ITEMS.items()))
FIELDS = ('clearance', 'track_width', 'via_diameter', 'via_drill', 'diff_pair_gap',
          'diff_pair_width', 'diff_pair_via_gap', 'microvia_diameter', 'microvia_drill',
          'schematic_color', 'pcb_color', 'wire_width', 'bus_width', 'line_style')
#: One net and one track per field a class may drop: `test_each_field`
_FIELD_NETS = ''.join(f' (net {5 + i} "F{i}")\n' for i in range(len(FIELDS)))
_FIELD_TRACKS = ''.join(
    f'(segment (start 2 {3 + i}) (end 5 {3 + i}) (width 0.2) (layer "F.Cu") (net {5 + i}) '
    f'(uuid "ba000000-0000-0000-0000-0000000000{i:02d}"))\n' for i in range(len(FIELDS)))
BOARD = f'''(kicad_pcb (version 20241229) (generator "pcbnew") (generator_version "9.0")
 (general (thickness 1.6))
 (paper "A4")
 (layers
  (0 "F.Cu" signal) (1 "In1.Cu" signal) (2 "In2.Cu" signal) (31 "B.Cu" signal)
  (37 "F.SilkS" user "F.Silkscreen") (39 "F.Mask" user) (38 "B.Mask" user)
  (44 "Edge.Cuts" user) (47 "F.CrtYd" user "F.Courtyard"))
 (setup (pad_to_mask_clearance 0))
 (net 0 "") (net 1 "A") (net 2 "HSNET") (net 3 "MIXED") (net 4 "DEF")
 (gr_rect (start 0 0) (end 60 40) (stroke (width 0.1) (type default)) (fill no) (layer "Edge.Cuts")
  (uuid "e0000000-0000-0000-0000-000000000001"))
 {_PARTS}
 (segment (start 10 25) (end 15 25) (width 0.2) (layer "F.Cu") (net 2) (uuid "b0000000-0000-0000-0000-000000000001"))
 (segment (start 10 28) (end 15 28) (width 0.2) (layer "F.Cu") (net 3) (uuid "b0000000-0000-0000-0000-000000000002"))
 (segment (start 10 31) (end 15 31) (width 0.2) (layer "F.Cu") (net 4) (uuid "b0000000-0000-0000-0000-000000000003"))
 (via (at 20 25) (size 0.8) (drill 0.4) (layers "F.Cu" "B.Cu") (net 2) (uuid "c0000000-0000-0000-0000-000000000001"))
 (via micro (at 25 25) (size 0.5) (drill 0.2) (layers "F.Cu" "In1.Cu") (net 2) (uuid "c0000000-0000-0000-0000-000000000002"))
 (via blind (at 30 25) (size 0.6) (drill 0.3) (layers "F.Cu" "In2.Cu") (net 2) (uuid "c0000000-0000-0000-0000-000000000003"))
 (zone (net 2) (net_name "HSNET") (layer "B.Cu") (uuid "d0000000-0000-0000-0000-000000000001") (hatch edge 0.5)
  (connect_pads (clearance 0.2)) (min_thickness 0.2) (fill (thermal_gap 0.5) (thermal_bridge_width 0.5))
  (polygon (pts (xy 40 20) (xy 55 20) (xy 55 35) (xy 40 35))))
)
'''
FIELD_BOARD = (BOARD.replace('(net 4 "DEF")\n', '(net 4 "DEF")\n' + _FIELD_NETS, 1).rstrip()[:-1]
               + _FIELD_TRACKS + ')\n')
NAMES = {f'a0000000-0000-0000-0000-00000000000{i + 1}': name for i, name in enumerate(_ITEMS)}
NAMES.update({'b0000000-0000-0000-0000-000000000001': 'track_hs',
              'b0000000-0000-0000-0000-000000000002': 'track_mixed',
              'b0000000-0000-0000-0000-000000000003': 'track_def',
              'c0000000-0000-0000-0000-000000000001': 'via',
              'c0000000-0000-0000-0000-000000000002': 'microvia',
              'c0000000-0000-0000-0000-000000000003': 'blindvia',
              'd0000000-0000-0000-0000-000000000001': 'zone',
              'e0000000-0000-0000-0000-000000000001': 'edge'})
ALL = frozenset(NAMES.values())
VIAS = frozenset({'via', 'microvia', 'blindvia'})
PADS = frozenset(_ITEMS)
HS = VIAS | {'track_hs', 'pth', 'zone'}          # the items on HSNET
OTHER = frozenset({'smd', 'connect', 'castellated', 'npth', 'track_def'})   # in no class
NONE = frozenset()
_SCOPE = ("A.Type == 'Track' || A.Type == 'Via' || A.Type == 'Pad' || A.Type == 'Zone' "
          "|| A.Layer == 'Edge.Cuts'")
_FULL = dict(clearance=0.2, track_width=0.25, via_diameter=0.8, via_drill=0.4, diff_pair_gap=0.25,
             diff_pair_width=0.2, diff_pair_via_gap=0.25, microvia_diameter=0.3, microvia_drill=0.1,
             schematic_color='rgba(0,0,0,0)', pcb_color='rgba(0,0,0,0)', wire_width=6, bus_width=12,
             line_style=0)


def cls(name, priority, drop=()):
    return {**{k: v for k, v in _FULL.items() if k not in drop}, 'name': name, 'priority': priority}


DEFAULT = cls('Default', 2147483647)

READINGS = {
    'one full class': ([DEFAULT, cls('HS', 0)], {'HSNET': 'HS'}, {
        "A.isPlated()": VIAS | {'pth', 'castellated'},
        "!A.isPlated()": ALL - VIAS - {'pth', 'castellated'},
        "A.Type == 'Via' && A.isPlated()": VIAS,
        "A.Type != 'Via' && A.isPlated()": {'pth', 'castellated'},
        "A.Type == 'Pad' && !A.isPlated()": {'smd', 'connect', 'npth'},
        # the reported disagreement is not there on a class with every field set
        "A.NetClass == 'Default'": OTHER | {'track_mixed', 'edge'},
        "A.NetClass != 'Default'": HS,
        "A.hasNetclass('Default')": OTHER | {'track_mixed'},
        "A.hasExactNetclass('Default')": OTHER | {'track_mixed'},
        "A.NetClass == 'HS'": HS,
        "A.NetClass == 'hs'": HS,
        "A.NetClass == 'H*'": HS,
        "A.NetClass == 'HS,Default'": NONE,
        "A.hasNetclass('HS')": HS,
        "A.hasNetclass('hs')": NONE,
        "A.hasNetclass('H*')": HS,
        "A.hasExactNetclass('HS')": HS,
        "A.hasExactNetclass('hs')": NONE,
        "A.hasExactNetclass('H*')": NONE,
        "'hs' == A.NetClass": HS,
        "'H*' == A.NetClass": NONE,
        # KiCad's names for the types, compared as a net name is
        "A.Pad_Type == 'SMD'": {'smd'},
        "A.Pad_Type == 'smd'": {'smd'},
        "A.Pad_Type == 'Through-hole'": {'pth', 'castellated'},
        "A.Pad_Type == 'through-hole'": {'pth', 'castellated'},
        "A.Pad_Type == 'T*'": {'pth', 'castellated'},
        "A.Pad_Type == 'thru_hole'": NONE,
        "A.Pad_Type == 'NPTH, mechanical'": {'npth'},
        "A.Pad_Type == 'NPTH'": NONE,
        "A.Pad_Type == 'Edge connector'": {'connect'},
        "A.Pad_Type == 'connect'": NONE,
        "A.Pad_Type != 'SMD'": PADS - {'smd'},
        "'Through-hole' == A.Pad_Type": {'pth', 'castellated'},
        "'S*' == A.Pad_Type": NONE,
        "A.Via_Type == 'Through'": {'via'},
        "A.Via_Type == 'through'": {'via'},
        "A.Via_Type == 'Micro'": {'microvia'},
        "A.Via_Type == 'Blind'": {'blindvia'},
        "A.Via_Type == 'Blind/buried'": NONE,
        "A.Via_Type == 'blind_buried'": NONE,
        "A.Via_Type == 'T*'": {'via'},
        "A.isBlindBuriedVia()": {'blindvia'},
        "A.isMicroVia()": {'microvia'},
        "A.Type == 'V*'": VIAS,
        "A.Type == 'via'": VIAS,
        "A.Type == 'hole'": NONE,
        # a literal with no closing quote runs to the end of the condition
        "A.Type == 'Via": VIAS,
        "A.Pad_Type == 'Through-hole": {'pth', 'castellated'},
        "A.Net != B.Net && A.Type == 'Via": NONE,
        # a term on a B there is not is false both ways
        "A.Net != B.Net": NONE,
        "A.Net == B.Net": NONE,
        "A.NetName != B.NetName": NONE,
        "A.NetClass != B.NetClass": NONE,
        "A.Type != B.Type": NONE,
        "!(A.Net == B.Net)": ALL,
    }),
    'a class with a gap': ([DEFAULT, cls('HS', 0, drop=('bus_width',))], {'HSNET': 'HS'}, {
        "A.NetClass == 'Default'": HS | OTHER | {'track_mixed', 'edge'},
        "A.NetClass != 'Default'": NONE,
        "A.hasNetclass('Default')": HS | OTHER | {'track_mixed'},
        "A.hasExactNetclass('Default')": OTHER | {'track_mixed'},
        "A.hasExactNetclass('HS')": NONE,
        "A.hasExactNetclass('HS,Default')": HS,
        "A.hasExactNetclass('Default,HS')": NONE,
        "A.NetClass == 'HS'": HS,
        "A.NetClass == 'HS,Default'": HS,
        "'HS' == A.NetClass": NONE,
        "'HS,Default' == A.NetClass": HS,
    }),
    'a net in two full classes': ([DEFAULT, cls('HS', 0), cls('LS', 1)],
                                  {'HSNET': 'HS', 'MIXED': ['LS', 'HS']}, {
        "A.NetClass == 'HS'": HS | {'track_mixed'},
        "A.NetClass == 'LS'": {'track_mixed'},
        "A.NetClass == 'HS,LS'": {'track_mixed'},
        "A.NetClass == 'hs,ls'": {'track_mixed'},
        "A.NetClass == 'LS,HS'": NONE,
        "A.NetClass == '*,LS'": {'track_mixed'},
        "A.NetClass == 'Default'": OTHER | {'edge'},
        "A.NetClass != 'LS'": ALL - {'track_mixed'},
        "A.NetClass != 'HS,LS'": ALL - {'track_mixed'},
        "A.hasNetclass('L?')": {'track_mixed'},
        "A.hasNetclass('HS,LS')": NONE,
        "A.hasNetclass('Default')": OTHER,
        "A.hasExactNetclass('HS,LS')": {'track_mixed'},
        "A.hasExactNetclass('LS,HS')": NONE,
        "A.hasExactNetclass('hs,ls')": NONE,
        "A.hasExactNetclass('HS')": HS,
        "'HS' == A.NetClass": HS,
        "'HS,LS' == A.NetClass": {'track_mixed'},
        "'HS' != A.NetClass": ALL - HS,
    }),
    'the priority sets the name': ([DEFAULT, cls('HS', 1), cls('LS', 0)], {'MIXED': ['HS', 'LS']}, {
        "A.hasExactNetclass('LS,HS')": {'track_mixed'},
        "A.hasExactNetclass('HS,LS')": NONE,
    }),
    'one priority: by name': ([DEFAULT, cls('LS', 0), cls('HS', 0)], {'MIXED': ['LS', 'HS']}, {
        "A.hasExactNetclass('HS,LS')": {'track_mixed'},
    }),
    'two classes with one gap': ([DEFAULT, cls('HS', 0, drop=('clearance',)),
                                  cls('LS', 1, drop=('clearance',))], {'MIXED': ['HS', 'LS']}, {
        "A.hasExactNetclass('HS,LS')": NONE,
        "A.hasExactNetclass('HS,LS,Default')": {'track_mixed'},
        "A.hasNetclass('Default')": ALL - {'edge'},
    }),
    'the other class fills the gap': ([DEFAULT, cls('HS', 0, drop=('clearance',)), cls('LS', 1)],
                                      {'MIXED': ['HS', 'LS']}, {
        "A.hasExactNetclass('HS,LS')": {'track_mixed'},
        "A.hasNetclass('Default')": ALL - {'edge', 'track_mixed'},
    }),
    'Default named beside a class': ([DEFAULT, cls('HS', 0)], {'MIXED': ['HS', 'Default']}, {
        "A.hasExactNetclass('HS,Default')": {'track_mixed'},
        "A.hasExactNetclass('HS')": NONE,
    }),
}

#: Conditions KiCad compiles: all in one file, so one run
COMPILES = (
    "", "1 == 1", "1 < 2", "1 + 1", "1 * 2", "2 / 1", "1 && 1", "1 || 2", "1mm && 1", "1.0 == 1", "1.5 + 1",
    "'x' == 'x'", "'a' < 'b'", "'x' + 'y'", "'x' && 'y'", "!'x'", "A", "B", "AB", "L", "-1", "2mm", "1.0mm",
    "1 mm", "1mil", "1in", "1deg", "1 deg", "1ps", "1fs", "1mm == 1mil", "L == 'F.Cu'", "L == 'x'",
    "A.Layer == L", "A == 'x'",
    # a function by any case, and whatever extra it is given
    "A.ISPLATED()", "A.isplated()", "A.MEMBEROFFOOTPRINT('x')", "A.isPlated('x')", "A.isMicroVia('x')",
    "A.isBlindBuriedVia(A.NetName)", "A.hasNetclass('a','b')", "A.memberOfFootprint('a','b')",
    "A.existsOnLayer('F.Cu','B.Cu')", "A.hasNetclass(1mm)", "A.isCoupledDiffPair('x')",
    "A.existsOnLayer('Nope')", "A.existsOnLayer('In*.Cu')",
    "AB.isCoupledDiffPair()", "AB.isPlated()", "AB.isMicroVia()", "AB.memberOfFootprint('x')",
    "AB.hasNetclass('x')", "B.isPlated()",
    # a number with a unit against anything, or an expression of numbers
    "A.NetName == 1mm", "A.NetName > 2mm", "A.Pad_Number == 1mm", "A.Pad_Number < 2mm",
    "A.Width == 1mm", "A.Width > 1mm", "A.Width > 1 mm", "1mm < A.Width", "A.Width + 1mm", "A.Width * 2mm",
    "A.Orientation == 90deg", "A.isPlated() == 1mm", "A.isPlated() && 1mm",
    "A.NetName == 1 + 1", "A.NetName == 1mm + 1", "A.NetName == -1", "-1 == A.NetName", "A.Net == -1",
    # strings against anything
    "A.NetName < 'x'", "A.Pad_Number > 'x'", "A.Width == 'x'", "A.Width < 'x'", "A.Orientation == 'x'",
    "A.Locked == 'x'", "A.Net == 'x'", "A.Type == 'Nope'", "A.Layer == 'Nope'",
    "A.isPlated() == 'x'", "(A.Type == 'Via') == 'x'", "(1 == 2) == 'x'", "A.getField('x') == 'y'",
    "A.Type == ('Via')", "A.NetName + 'x'", "A.NetName + A.NetName", "A.NetName || 'x'",
    # properties against properties
    "A.Net != B.Net", "A.Net == B.NetName", "A.NetName == B.Net", "A.NetName == A.Type",
    "A.Width == A.NetName", "A.Width == A.Hole_Size_X", "A.Hole_Size_X != A.Hole_Size_Y",
    "- A.Width", "-A.NetName", "!A.NetName", "!!A.isPlated()", "A.Locked",
    "A.Fabrication_Property == 'Castellated pad'", "A.Reference == 'x'",
    # spellings
    "A.NetClass", "A.Net_Class", "A.netname", "A.ComponentClass", "A.Component_Class",
    "A.Type=='Via'", "  A.Type == 'Via'  ",
    # a literal with no closing quote
    "A.Type == 'Via", "'Via",
) + tuple(f"A.{f}" for f in (
    "existsOnLayer('F.Cu')", "isPlated()", "insideCourtyard('x')", "insideFrontCourtyard('x')",
    "insideBackCourtyard('x')", "intersectsCourtyard('x')", "intersectsFrontCourtyard('x')",
    "intersectsBackCourtyard('x')", "insideArea('x')", "intersectsArea('x')", "enclosedByArea('x')",
    "isMicroVia()", "isBlindBuriedVia()", "isBlindVia()", "isBuriedVia()", "memberOf('x')",
    "memberOfGroup('x')", "memberOfFootprint('x')", "memberOfSheet('x')",
    "memberOfSheetOrChildren('x')", "fromTo('a','b')", "isCoupledDiffPair()", "inDiffPair('x')",
    "getField('x')", "hasNetclass('x')", "hasExactNetclass('x')", "hasComponentClass('x')"))

#: Conditions it does not, each its own file, and so its own run
REFUSED = (
    # functions it does not know
    "A.onLayer('F.Cu')", "A.foo()", "A.NetName()",
    "A.Type == 'Via' || A.onLayer('F.Cu')",
    # a call short of what it needs
    "A.memberOfFootprint()", "A.hasNetclass()", "A.existsOnLayer()", "A.fromTo('a')",
    "A.memberOfFootprint('x'",
    # a plain number as an argument, a property for a string
    "A.isPlated(1)", "A.memberOfFootprint(1)", "A.memberOfFootprint(A.NetName)",
    # items and bare names it does not know
    "C.isPlated()", "foo.bar()", "isPlated()", "a.isPlated()",
    "L.x", "foo",
    # properties it does not know
    "A.Foo == 'x'", "A.Foo", "A.isPlated", "AB.NetName == 'x'",
    "A.Net_Name", "A.PadNumber",
    "A.NetName.x", "A.",
    # a plain number against anything but a number
    "A.NetName == 1", "1 == A.NetName", "A.NetName < 2", "A.Type == 1",
    "A.Pad_Number < 2", "A.Pad_Number + 1",
    "A.Width == 1", "A.Width > 1",
    "A.Net == 1", "'x' == 1",
    "'x' + 1", "(A.Type == 'Via') == 1", "A.isPlated() == 1",
    "A.NetName == (1)", "A.NetName && 1", "A.Type == 'Via' && 1", "!1",
    "L == 1", "A == 1",
    # a number alone, or misspelled
    "1", "1.0", "(1)", "1.", ".5", "2xx", "1um", "1cm",
    "1MM", "1 1",
    # syntax
    "A.NetName ==", "(A.Type == 'Via'", "A.Type = 'Via'", "A.Type == 'Via' &",
    "!", "&&", "A.Type == Via", "A.Type == 'a' 'b'",
    "A.Type == 'Via' == 'x'", "1 < 2 < 3", "A.Type % 2",
    "A.Type == 'Via'  # note", ".NetName", "!(A.Type == 'Via)",
)


#: More of the same, measured when this table was written and not re-run each time, to
#: keep the kicad-cli half short. `test_compile_table_is_the_model` still reads them.
REFUSED_RECORDED = (
    "A.isBetween('a','b')", 'AB.foo()', "A.Type('x')", "A.Type != 'Via' && 0 && A.onLayer('F.Cu')",
    'A.getField()', 'A.inDiffPair()', 'A.insideArea()', 'A.isMicroVia(1)', 'foo.NetName',
    "b.NetName == 'x'", 'ab.isCoupledDiffPair()', 'true', 'a', '!A.Foo', 'B.Foo', 'AB.Foo',
    'A.PadType', 'A.ViaType', 'A.Track_Width == 1mm', "A.Priority == 'x'", 'A.NetName != 1',
    'A.NetClass == 1', 'A.Layer == 1', 'A.Pad_Type == 1', 'A.Via_Type == 1', 'A.Reference == 1',
    '1 < A.Width', 'A.Width + 1', 'A.Width * 2', 'A.Hole_Size_X > 1', 'A.Net > 1', 'A.Locked == 1',
    "1 == 'x'", "'x' < 2", "'x' * 2", '!A.isPlated() == 1', "1 && 'x'", '(A.isPlated()) || 1',
    '!(1)', 'L < 2', "A.getField('x') == 1", '0', '(1.5)', '1.5.5', 'A.NetName == .5', '1nm',
    '1Mil', '1thou', '1mm2', "== 'x'", "A.Type == 'Via')", "A.Type == 'Via' ||", '(())',
    "A.Type == 'Via' A.Type == 'Via'", 'A..NetName',
)

_OK_RULE = '(rule "c" (constraint clearance (min 0.1mm)))'
#: What surrounds a condition: (the file body, what the reader calls it), loaded or not
FILES_LOAD = (
    '(rule "c" (layer "?.Silkscreen") (constraint clearance (min 0.1mm)))',
    '(rule "c" (layer "*.Cu") (constraint clearance (min 0.1mm)))',
    '(rule "c" (layer "In9.Cu") (constraint clearance (min 0.1mm)))',
    '(rule "c" (layer outer) (severity warning) (constraint clearance (min 0.1mm)))',
    '(rule "c" (severity error) (constraint clearance (min 0.1mm)))',
    '(rule "c" (severity ignore) (constraint clearance (min 0.1mm)))',
    '(rule "c" (severity exclusion) (constraint clearance (min 0.1mm)))',
    '(rule "c" (constraint via_count (max 2)))',
    '(rule "c" (constraint courtyard_clearance (min -1mm)))',
    '(rule "c" (constraint disallow track via micro_via blind_via buried_via through_via pad '
    'zone text graphic hole footprint))',
    '(rule "c")',
)
FILES_REFUSED = (
    ('(rule "c" (layer "Foo.Cu") (constraint clearance (min 0.1mm)))', "'Foo.Cu'"),
    ('(rule "c" (constraint foo (min 0.1mm)))', "constraint 'foo'"),
    ('(rule "c" (constraint clearance (min xyz)))', "'xyz'"),
    ('(rule "c" (constraint clearance (min 1xx)))', "'1xx'"),
    ('(rule "c" (constraint clearance (min 1)))', "'1'"),
    ('(rule "c" (constraint track_width (min 1)))', "'1'"),
    ('(rule "c" (severity foo) (constraint clearance (min 0.1mm)))', "severity 'foo'"),
    ('(rule "c" (foo bar) (constraint clearance (min 0.1mm)))', "clause 'foo'"),
    ('(rule "c" (constraint disallow foo))', "item 'foo'"),
    ('(foo bar)', "form 'foo'"),
    ('(rule (constraint clearance (min 0.1mm)))', 'no name'),
)

#: A zone on `GND`, an edge rule of 0.4 over a board's 0.05: the condition, whether
#: the filler holds to 0.4, and what `for_zone` reads (a range where it reads B)
ZONE = (
    ("A.NetName == 'GND'", True, (0.4, 0.4)),
    ("A.NetName == 'NOPE'", False, (0.05, 0.05)),
    ("A.Type == 'Track'", False, (0.05, 0.05)),
    ("!(A.NetClass == 'Default')", False, (0.05, 0.05)),
    # the swap: A is the Edge.Cuts shape
    ("A.Layer == 'Edge.Cuts'", True, (0.4, 0.4)),
    ("A.Type == 'Graphic'", True, (0.4, 0.4)),
    ("!(A.Type == 'Zone')", True, (0.4, 0.4)),
    ("!(A.NetName == 'GND')", True, (0.4, 0.4)),
    ("A.NetName == ''", True, (0.4, 0.4)),
    ("A.Layer != 'F.Cu'", True, (0.4, 0.4)),
    # this reader does not answer for B
    ("B.Type == 'Graphic'", True, (0.05, 0.4)),
)


def _part(lib, ref, uid, x):
    return (f'(footprint "{lib}" (layer "F.Cu") (uuid "f0{uid}0000-0000-0000-0000-000000000000") (at {x} 10)\n'
            f'  (property "Reference" "{ref}" (at 0 -2 0) (layer "F.SilkS") '
            f'(uuid "f1{uid}0000-0000-0000-0000-000000000000") '
            f'(effects (font (size 1 1) (thickness 0.15))))\n'
            f'  (fp_rect (start -1 -1) (end 1 1) (stroke (width 0.05) (type default)) (fill no) '
            f'(layer "F.CrtYd") (uuid "f3{uid}0000-0000-0000-0000-000000000000"))\n'
            f'  (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu" "F.Mask") (net 1 "A") '
            f'(uuid "a0{uid}0000-0000-0000-0000-000000000000")))\n')


_PARTS2 = (('Device:R_0402', 'R7', '01'), ('x:Z', 'a:b', '02'), ('Bare0603', 'B3', '03'),
           ('Device:R_0402', 'R8', '04'), ('lib.v2:Name With Space', 'U1', '05'),
           ('x:D', 'D1', '09'), ('x:D', 'D1', '10'))
#: a part's reference may read as a library name; two blocks may claim one reference
BOARD2 = ('''(kicad_pcb (version 20241229) (generator "pcbnew") (generator_version "9.0")
 (general (thickness 1.6)) (paper "A4")
 (layers (0 "F.Cu" signal) (1 "In1.Cu" signal) (2 "In2.Cu" signal) (31 "B.Cu" signal)
  (36 "B.SilkS" user "B.Silkscreen") (37 "F.SilkS" user "F.Silkscreen") (38 "B.Mask" user)
  (39 "F.Mask" user) (34 "B.Paste" user) (35 "F.Paste" user) (44 "Edge.Cuts" user)
  (46 "B.CrtYd" user "B.Courtyard") (47 "F.CrtYd" user "F.Courtyard"))
 (setup (pad_to_mask_clearance 0))
 (net 0 "") (net 1 "A") (net 2 "B")
 (gr_rect (start 0 0) (end 60 40) (stroke (width 0.1) (type default)) (fill no) (layer "Edge.Cuts")
  (uuid "e0000000-0000-0000-0000-000000000001"))
 ''' + ''.join(_part(lib, ref, uid, 5 + 7 * i) for i, (lib, ref, uid) in enumerate(_PARTS2)) + '''
 (footprint "x:P" (layer "F.Cu") (uuid "f0060000-0000-0000-0000-000000000000") (at 10 20)
  (property "Reference" "P1" (at 0 -2 0) (layer "F.SilkS") (uuid "f1060000-0000-0000-0000-000000000000")
   (effects (font (size 1 1) (thickness 0.15))))
  (pad "1" thru_hole circle (at 0 0) (size 1.6 1.6) (drill 0.8) (layers "*.Cu" "*.Mask") (net 2 "B")
   (uuid "a0060000-0000-0000-0000-000000000000")))
 (footprint "x:H" (layer "F.Cu") (uuid "f0070000-0000-0000-0000-000000000000") (at 20 20)
  (property "Reference" "H1" (at 0 -2 0) (layer "F.SilkS") (uuid "f1070000-0000-0000-0000-000000000000")
   (effects (font (size 1 1) (thickness 0.15))))
  (pad "1" np_thru_hole circle (at 0 0) (size 1.6 1.6) (drill 1.6) (layers "*.Cu" "*.Mask")
   (uuid "a0070000-0000-0000-0000-000000000000")))
 (footprint "x:BK" (layer "B.Cu") (uuid "f0080000-0000-0000-0000-000000000000") (at 30 20)
  (property "Reference" "K1" (at 0 -2 0) (layer "B.SilkS") (uuid "f1080000-0000-0000-0000-000000000000")
   (effects (font (size 1 1) (thickness 0.15))))
  (pad "1" smd rect (at 0 0) (size 1 1) (layers "B.Cu" "B.Mask" "B.Paste") (net 2 "B")
   (uuid "a0080000-0000-0000-0000-000000000000")))
 (segment (start 5 30) (end 8 30) (width 0.2) (layer "F.Cu") (net 2) (uuid "b0000000-0000-0000-0000-000000000001"))
 (segment (start 5 32) (end 8 32) (width 0.2) (layer "In1.Cu") (net 2) (uuid "b0000000-0000-0000-0000-000000000002"))
 (segment (start 5 34) (end 8 34) (width 0.2) (layer "B.Cu") (net 2) (uuid "b0000000-0000-0000-0000-000000000003"))
 (via (at 20 30) (size 0.8) (drill 0.4) (layers "F.Cu" "B.Cu") (net 2) (uuid "c0000000-0000-0000-0000-000000000001"))
 (via micro (at 25 30) (size 0.5) (drill 0.2) (layers "F.Cu" "In1.Cu") (net 2) (uuid "c0000000-0000-0000-0000-000000000002"))
 (via blind (at 30 30) (size 0.6) (drill 0.3) (layers "F.Cu" "In2.Cu") (net 2) (uuid "c0000000-0000-0000-0000-000000000003"))
 (zone (net 2) (net_name "B") (layer "B.Cu") (uuid "d0000000-0000-0000-0000-000000000001") (hatch edge 0.5)
  (connect_pads (clearance 0.2)) (min_thickness 0.2) (fill (thermal_gap 0.5) (thermal_bridge_width 0.5))
  (polygon (pts (xy 40 25) (xy 55 25) (xy 55 35) (xy 40 35))))
)
''')
NAMES2 = {'a0010000-0000-0000-0000-000000000000': 'R7', 'a0020000-0000-0000-0000-000000000000': 'a:b',
          'a0030000-0000-0000-0000-000000000000': 'B3', 'a0040000-0000-0000-0000-000000000000': 'R8',
          'a0050000-0000-0000-0000-000000000000': 'U1', 'a0090000-0000-0000-0000-000000000000': 'D1',
          'a0100000-0000-0000-0000-000000000000': 'D1~2', 'a0060000-0000-0000-0000-000000000000': 'P1',
          'a0070000-0000-0000-0000-000000000000': 'H1', 'a0080000-0000-0000-0000-000000000000': 'K1',
          'b0000000-0000-0000-0000-000000000001': 't_F', 'b0000000-0000-0000-0000-000000000002': 't_In1',
          'b0000000-0000-0000-0000-000000000003': 't_B', 'c0000000-0000-0000-0000-000000000001': 'via',
          'c0000000-0000-0000-0000-000000000002': 'micro', 'c0000000-0000-0000-0000-000000000003': 'blind',
          'd0000000-0000-0000-0000-000000000001': 'zone', 'e0000000-0000-0000-0000-000000000001': 'edge'}
ALL2 = frozenset(NAMES2.values())
F_PADS = frozenset({'R7', 'a:b', 'B3', 'R8', 'U1', 'D1', 'D1~2'})
HOLES = frozenset({'P1', 'H1'})
ALL_PADS = F_PADS | HOLES | {'K1'}
_F = F_PADS | HOLES | {'t_F', 'via', 'micro', 'blind'}
_INNER1 = HOLES | {'t_In1', 'via', 'micro', 'blind'}


def _on(arg):
    return f"A.existsOnLayer('{arg}')"


LAYER_ROWS = {
    _on('F.Cu'): _F,
    _on('B.Cu'): HOLES | {'K1', 't_B', 'via', 'zone'},
    _on('In1.Cu'): _INNER1,
    _on('In2.Cu'): HOLES | {'via', 'blind'},
    # wildcards, by wxString::Matches: case-sensitive, and '[' is no class
    _on('In*.Cu'): _INNER1,
    _on('In?.Cu'): _INNER1,
    _on('In1*'): _INNER1,
    _on('*.Cu'): ALL2 - {'edge'},
    _on('?.Cu'): ALL2 - {'edge', 't_In1'},
    _on('F.*'): _F,
    _on('*'): ALL2,
    _on('[FB].Cu'): NONE,
    _on('f.cu'): NONE,
    _on('F.cu'): NONE,
    _on('B.cu'): NONE,
    _on('in1.cu'): NONE,
    _on('in*.cu'): NONE,
    _on('*.cu'): NONE,
    # not copper
    _on('Edge.Cuts'): {'edge'},
    _on('Edge*'): {'edge'},
    _on('F.Mask'): F_PADS | HOLES,
    _on('*.Mask'): ALL_PADS,
    _on('B.Paste'): {'K1'},
    _on('*.Paste'): {'K1'},
    _on('Nope'): NONE,
    _on('Dwgs.User'): NONE,
    # a pad's *.Cu is every layer KiCad has, a through via every layer of the board
    _on('In9.Cu'): HOLES,
    _on('In3*'): HOLES,
    # a second argument
    "A.existsOnLayer('F.Cu','B.Cu')": NONE,
    "A.existsOnLayer('Nope','F.Cu')": NONE,
    # in a condition, as a rule file writes them
    "!A.existsOnLayer('In*.Cu')": ALL2 - _INNER1,
    "A.existsOnLayer('In*.Cu') && A.Type == 'Track'": {'t_In1'},
    "(A.existsOnLayer('F.Cu') || A.existsOnLayer('B.cu')) && A.Type == 'Track'": {'t_F'},
}


def _of(arg):
    return f"A.memberOfFootprint('{arg}')"


PART_ROWS = {
    # the reference
    _of('R7'): {'R7'},
    _of('r7'): NONE,
    _of('R*'): {'R7', 'R8'},
    _of('R?'): {'R7', 'R8'},
    _of('?7'): {'R7'},
    _of('[R]7'): NONE,
    _of('*'): ALL_PADS,
    _of('D1'): {'D1', 'D1~2'},      # both blocks claim it
    # the library id, only when the name holds a ':'
    _of('Device:R_0402'): {'R7', 'R8'},
    _of('device:r_0402'): NONE,
    _of('Device:*'): {'R7', 'R8'},
    _of('*:R_0402'): {'R7', 'R8'},
    _of('Device:R_????'): {'R7', 'R8'},
    _of('Dev*:R*'): {'R7', 'R8'},
    _of('R_0402'): NONE,
    _of('*R_0402'): NONE,
    _of('*0402*'): NONE,
    _of(':R_0402'): NONE,
    _of('Device:'): NONE,
    _of('*:*'): ALL_PADS - {'B3'},
    _of('x:D'): {'D1', 'D1~2'},
    _of('x:*'): {'a:b', 'D1', 'D1~2', 'P1', 'H1', 'K1'},
    _of('lib.v2:Name With Space'): {'U1'},
    _of('*:Name*'): {'U1'},
    # a reference that reads as a library name matches either way
    _of('a:b'): {'a:b'},
    _of('a:*'): {'a:b'},
    _of('*:b'): {'a:b'},
    _of('x:Z'): {'a:b'},
    _of('*:Z'): {'a:b'},
    _of('A:B'): NONE,
    # a part with no library
    _of('Bare0603'): NONE,
    _of(':Bare0603'): NONE,
    _of('*:Bare0603'): NONE,
    _of('*0603*'): NONE,
    _of('B3'): {'B3'},
    # a second argument, and what is no part's
    "A.memberOfFootprint('R7','x')": NONE,
    "A.memberOfFootprint('x','R7')": NONE,
    "A.Type == 'Track' && A.memberOfFootprint('*')": NONE,
}

#: `PadEdgeRules` reads `A.insideCourtyard('X')` as `A.memberOfFootprint('X')`. For a
#: part with a courtyard, these X read alike in KiCad.
COURTYARD_ARGS = ('R7', 'R*', 'Device:R_0402', 'Device:*', '*:R_0402', 'a:b', 'x:Z', 'r7', '*0402*',
                  'Bare0603', 'D1', 'x:D')

#: On J3 (Connector_Coaxial:SMA_Amphenol_132289_EdgeMount), 0.05 mm inside the edge
#: under a 0.4 mm floor: the condition freeing it, and whether it does
EDGE_LIBRARY = (
    ("A.memberOfFootprint('Connector_Coaxial:*')", True),
    ("A.memberOfFootprint('*:SMA_Amphenol*')", True),
    ("A.memberOfFootprint('Connector_Coaxial:SMA_Amphenol_132289_EdgeMount')", True),
    ("A.memberOfFootprint('connector_coaxial:*')", False),
    ("A.memberOfFootprint('SMA_Amphenol*')", False),
    ("A.Type == 'Pad' && A.memberOfFootprint('Connector_Coaxial:*')", True),
    ("A.insideCourtyard('Connector_Coaxial:*')", True),
)


def project(classes=(DEFAULT,), assign=None, rules=None):
    return {'board': {'design_settings': {'rules': rules or {}}},
            'meta': {'filename': 'p.kicad_pro', 'version': 3},
            'net_settings': {'classes': list(classes), 'meta': {'version': 4},
                             'netclass_assignments': assign, 'netclass_patterns': []}}


def stage(work, name, dru=None, pro=None, board=BOARD):
    path = os.path.join(work, name + '.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(board)
    with open(os.path.join(work, name + '.kicad_pro'), 'w', encoding='utf-8') as fh:
        json.dump(pro or project(), fh)
    if dru is not None:
        with open(os.path.join(work, name + '.kicad_dru'), 'w', encoding='utf-8') as fh:
            fh.write(dru)
    return path


def kicad_drc(path, *extra):
    out = path + '.drc.json'
    subprocess.run([CLI, 'pcb', 'drc', '--format', 'json', '--severity-all', *extra, '-o', out, path],
                   capture_output=True, text=True, timeout=300)
    with open(run_utils.evidence(out, 'the kicad-cli report'), encoding='utf-8') as fh:
        return json.load(fh)['violations']


def kicad_holds(work, name, pro, conds, board=BOARD, names=NAMES, size=14):
    """{condition: the items it holds on}, in batches of `size` to a DRC pass."""
    holds = {}
    every = frozenset(names.values())
    for start in range(0, len(conds), size):
        batch = conds[start:start + size]
        path = stage(work, f'{name}_{start}', '(version 1)\n' + ''.join(
            f'(rule "row{i}" (condition "{_SCOPE}") (constraint assertion "{c}"))\n'
            for i, c in enumerate(batch)), pro, board)
        failed = {c: set() for c in batch}
        rows = [v for v in kicad_drc(path) if v['type'] == 'assertion_failure']
        assert len(rows) < 190, f'{len(rows)} failures: KiCad stops looking near 199'
        for v in rows:
            c = batch[int(re.search(r"row(\d+)", v['description']).group(1))]
            failed[c] |= {names.get(i['uuid'], i['uuid']) for i in v['items']}
        for c in batch:
            holds[c] = every - failed[c]
    return holds


def kicad_loads(path):
    """Does KiCad enforce the sentinel the file ends with?"""
    return any("'zz_sentinel'" in v['description'] for v in kicad_drc(path)
               if v['type'] == 'assertion_failure')


_SENTINEL = '(rule "zz_sentinel" (constraint assertion "1 == 2"))\n'


def compile_file(*conds):
    return '(version 1)\n' + ''.join(
        f'(rule "c{i}" (condition "{c}") (constraint clearance (min 0.1mm)))\n'
        for i, c in enumerate(conds)) + _SENTINEL


def model_items(path):
    """{item name: (RuleItem, layer)}, as the engines build them."""
    pcb = parse_kicad_pcb(path)
    dr = DesignRules.from_project(pcb, path)
    nets = {n.name: nid for nid, n in pcb.nets.items() if n.name}
    items = {'track_hs': (dr.item_for_net(nets['HSNET'], 'track', 'F.Cu'), 'F.Cu'),
             'track_mixed': (dr.item_for_net(nets['MIXED'], 'track', 'F.Cu'), 'F.Cu'),
             'track_def': (dr.item_for_net(nets['DEF'], 'track', 'F.Cu'), 'F.Cu'),
             'zone': (dr.item_for_net(nets['HSNET'], 'zone', 'B.Cu'), 'B.Cu'),
             'edge': (dr_mod._EDGE_ITEM, None)}
    names = {'through': 'via', 'micro': 'microvia', 'blind': 'blindvia'}
    for v in pcb.vias:
        items[names[v.via_type]] = (dr.item_for_net(v.net_id, 'via', None, via_type=v.via_type), None)
    for name in _ITEMS:
        pad = pcb.footprints[REFS[name] + '1'].pads[0]
        items[name] = (dr.item_for_pad(pad), 'F.Cu')
    return items, dr


def model_items2(path):
    from geometry_utils import via_copper_layers
    pcb = parse_kicad_pcb(path)
    dr = DesignRules.from_project(pcb, path)
    b = next(nid for nid, n in pcb.nets.items() if n.name == 'B')
    items = {f't_{ln.split(".")[0]}': (dr.item_for_net(b, 'track', ln), ln)
             for ln in ('F.Cu', 'In1.Cu', 'B.Cu')}
    items['zone'] = (dr.item_for_net(b, 'zone', 'B.Cu'), 'B.Cu')
    items['edge'] = (dr_mod._EDGE_ITEM, None)
    for v in pcb.vias:
        if v.via_type == 'through':     # as the engines ask, with no span given
            items['via'] = (dr.item_for_net(v.net_id, 'via', None, via_type='through'), None)
        else:
            items[v.via_type] = (dr.item_for_net(
                v.net_id, 'via', None, via_type=v.via_type,
                layers=frozenset(via_copper_layers(v, pcb.board_info.copper_layers))), None)
    for key, fp in pcb.footprints.items():
        items[key] = (dr.item_for_pad(fp.pads[0]), None)
    return items, dr


def model_holds(path, conds, build=None):
    items, dr = (build or model_items)(path)
    ctx = ('clearance', dr._net_name_set())
    out = {}
    for c in conds:
        ast = dr_mod.parse_condition_maybe(c)
        got = {name: dr_mod._eval3(ast, it, None, layer, ctx) for name, (it, layer) in items.items()}
        assert None not in got.values(), f'{c!r} reads unknown: {got}'
        out[c] = frozenset(k for k, v in got.items() if v)
    return out


#: Runs in KiCad's python: the table's readings off the board pcbnew loads
PCBNEW_PROBE = r'''
import json, sys
sys.path.insert(0, sys.argv[1])
import pcbnew
import design_rules as d
from geometry_utils import via_copper_layers
from kicad_parser import build_pcb_data_from_board
board = pcbnew.LoadBoard(sys.argv[2])
pcb = build_pcb_data_from_board(board)
dr = d.DesignRules.from_pcbnew(board, pcb)
ctx = ('clearance', dr._net_name_set())
nets = {n.name: i for i, n in pcb.nets.items() if n.name}
items = {'track_hs': dr.item_for_net(nets['HSNET'], 'track', 'F.Cu'),
         'smd': dr.item_for_pad(pcb.footprints['S1'].pads[0]),
         'pth': dr.item_for_pad(pcb.footprints['P1'].pads[0]),
         'connect': dr.item_for_pad(pcb.footprints['C1'].pads[0]),
         'npth': dr.item_for_pad(pcb.footprints['H1'].pads[0])}
for v in pcb.vias:
    items[v.via_type] = dr.item_for_net(v.net_id, 'via', None, via_type=v.via_type,
                                        layers=frozenset(via_copper_layers(v, pcb.board_info.copper_layers)))
out = {}
for c in json.loads(sys.argv[3]):
    ast = d.parse_condition_maybe(c)
    out[c] = sorted(k for k, it in items.items() if d._eval3(ast, it, None, None, ctx))
print('RESULT ' + json.dumps(out))
'''


class RuleReadings(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.work = self._tmp.name

    def tearDown(self):
        self._tmp.cleanup()

    def test_readings_are_the_model(self):
        for arm, (classes, assign, rows) in READINGS.items():
            path = stage(self.work, 'm', None, project(classes, assign))
            got = model_holds(path, list(rows))
            for c, want in rows.items():
                with self.subTest(arm=arm, cond=c):
                    self.assertEqual(got[c], frozenset(want))

    @unittest.skipUnless(CLI, 'no kicad-cli')
    def test_readings_are_kicad(self):
        for a, (arm, (classes, assign, rows)) in enumerate(READINGS.items()):
            got = kicad_holds(self.work, f'k{a}', project(classes, assign), list(rows))
            for c, want in rows.items():
                with self.subTest(arm=arm, cond=c):
                    self.assertEqual(got[c], frozenset(want), 'the table is stale')

    def test_layers_and_parts_are_the_model(self):
        path = stage(self.work, 'lp', None, project(), BOARD2)
        rows = {**LAYER_ROWS, **PART_ROWS}
        got = model_holds(path, list(rows), model_items2)
        for c, want in rows.items():
            with self.subTest(cond=c):
                self.assertEqual(got[c], frozenset(want))

    @unittest.skipUnless(CLI, 'no kicad-cli')
    def test_layers_and_parts_are_kicad(self):
        rows = {**LAYER_ROWS, **PART_ROWS}
        got = kicad_holds(self.work, 'lpk', project(), list(rows), BOARD2, NAMES2, 9)
        for c, want in rows.items():
            with self.subTest(cond=c):
                self.assertEqual(got[c], frozenset(want), 'the table is stale')

    @unittest.skipUnless(CLI, 'no kicad-cli')
    def test_courtyard_reads_as_the_member_in_kicad(self):
        """The rewrite `PadEdgeRules` makes is true of a part with a courtyard."""
        conds = [f"A.{fn}('{x}')" for x in COURTYARD_ARGS
                 for fn in ('insideCourtyard', 'intersectsCourtyard', 'memberOfFootprint')]
        got = kicad_holds(self.work, 'cy', project(), conds, BOARD2, NAMES2, 9)
        for x in COURTYARD_ARGS:
            member = got[f"A.memberOfFootprint('{x}')"]
            for fn in ('insideCourtyard', 'intersectsCourtyard'):
                with self.subTest(fn=fn, arg=x):
                    self.assertEqual(got[f"A.{fn}('{x}')"], member)

    def _edge_board(self, name, cond):
        from test_1615_edge_rule_pads import EDGE_X
        from test_1615_edge_rule_pads import stage as board_stage
        return board_stage(self.work, name, EDGE_X,
                           f'(version 1)\n(rule "e" (condition "{cond}") '
                           f'(constraint edge_clearance (min 0mm)))\n')

    def test_edge_library_is_the_model(self):
        for i, (cond, frees) in enumerate(EDGE_LIBRARY):
            with self.subTest(cond=cond):
                path = self._edge_board(f'em{i}', cond)
                pcb = parse_kicad_pcb(path)
                rules = PadEdgeRules(pcb, path)
                self.assertEqual(rules.unmodelled, [])
                want = 0.0 if frees else 0.4
                for pad in pcb.footprints['J3'].pads:
                    self.assertEqual(rules.for_pad(pad, 0.4)[:2], (want, want))
                # another part's pad stays at the floor
                self.assertEqual(rules.for_pad(pcb.footprints['C1'].pads[0], 0.4)[:2], (0.4, 0.4))

    @unittest.skipUnless(CLI, 'no kicad-cli')
    def test_edge_library_in_kicad(self):
        for i, (cond, frees) in enumerate(EDGE_LIBRARY):
            with self.subTest(cond=cond):
                flagged = sum(v['type'] == 'copper_edge_clearance'
                              and any('J3' in it['description'] for it in v['items'])
                              for v in kicad_drc(self._edge_board(f'ek{i}', cond)))
                self.assertEqual(flagged == 0, frees, 'the table is stale')

    def _field_project(self):
        classes = [DEFAULT] + [cls(f'C{i}', i, drop=(f,)) for i, f in enumerate(FIELDS)]
        return project(classes, {f'F{i}': f'C{i}' for i in range(len(FIELDS))})

    def test_each_field_is_the_model(self):
        path = stage(self.work, 'fields', None, self._field_project(), FIELD_BOARD)
        pcb = parse_kicad_pcb(path)
        dr = DesignRules.from_project(pcb, path)
        nets = {n.name: nid for nid, n in pcb.nets.items()}
        got = {f for i, f in enumerate(FIELDS)
               if 'Default' in dr.class_constituents(nets[f'F{i}'])}
        self.assertEqual(got, set(dr_mod._CLASS_COMPLETE_FIELDS))
        self.assertEqual(set(FIELDS) - got,
                         {'diff_pair_via_gap', 'schematic_color', 'pcb_color', 'line_style'})

    @unittest.skipUnless(CLI, 'no kicad-cli')
    def test_each_field_in_kicad(self):
        path = stage(self.work, 'fieldsk', '(version 1)\n'
                     '(rule "row" (condition "A.Type == \'Track\'") '
                     '(constraint assertion "A.hasNetclass(\'Default\')"))\n', self._field_project(),
                     FIELD_BOARD)
        without = set()
        for v in kicad_drc(path):
            if v['type'] == 'assertion_failure':
                for item in v['items']:
                    m = re.match(r'ba000000-0000-0000-0000-0000000000(\d\d)$', item['uuid'])
                    if m:
                        without.add(FIELDS[int(m.group(1))])
        self.assertEqual(set(FIELDS) - without, set(dr_mod._CLASS_COMPLETE_FIELDS), 'the table is stale')

    def test_compile_table_is_the_model(self):
        for c in COMPILES:
            with self.subTest(cond=c):
                self.assertIsNone(kicad_condition_refusal(c))
        for c in REFUSED + REFUSED_RECORDED:
            with self.subTest(cond=c):
                self.assertIn('enforces no rule of this file', kicad_condition_refusal(c) or '')

    @unittest.skipUnless(CLI, 'no kicad-cli')
    def test_compile_table_is_kicad(self):
        path = stage(self.work, 'ok', compile_file(*COMPILES))
        self.assertTrue(kicad_loads(path), 'a row of COMPILES no longer compiles')

        def refused(job):
            i, c = job
            return c, kicad_loads(stage(self.work, f'r{i}', compile_file(c)))
        with concurrent.futures.ThreadPoolExecutor(WORKERS) as pool:
            loaded = [c for c, ok in pool.map(refused, enumerate(REFUSED)) if ok]
        self.assertEqual(loaded, [], 'the table is stale')

    @unittest.skipUnless(CLI, 'no kicad-cli')
    def test_every_property_named_compiles(self):
        """One pass over the table, so a name it should not hold fails here."""
        body = compile_file(*(f'A.{p}' for p in sorted(dr_mod._KICAD_PROPERTIES)))
        self.assertTrue(kicad_loads(stage(self.work, 'props', body)))

    def test_file_table_is_the_model(self):
        for body in FILES_LOAD:
            with self.subTest(body=body):
                text = '(version 1)\n' + body + '\n'
                self.assertEqual(dr_mod.dru_refusals(text, dr_mod.parse_dru(text)[0]), [])
        for body, word in FILES_REFUSED:
            with self.subTest(body=body):
                text = '(version 1)\n' + body + '\n'
                found = dr_mod.dru_refusals(text, dr_mod.parse_dru(text)[0])
                self.assertEqual(len(found), 1, found)
                self.assertIn(word, found[0]['reason'])
                self.assertIn('KiCad refuses', found[0]['reason'])
        text = _OK_RULE + '\n'
        found = dr_mod.dru_refusals(text, dr_mod.parse_dru(text)[0])
        self.assertEqual([r['rule'] for r in found], [''])
        self.assertIn('(version', found[0]['reason'])

    @unittest.skipUnless(CLI, 'no kicad-cli')
    def test_file_table_is_kicad(self):
        path = stage(self.work, 'fl', '(version 1)\n' + '\n'.join(
            b.replace('"c"', f'"c{i}"') for i, b in enumerate(FILES_LOAD)) + '\n' + _SENTINEL)
        self.assertTrue(kicad_loads(path), 'a row of FILES_LOAD is no longer loaded')
        bodies = [b for b, _w in FILES_REFUSED]

        def loads(job):
            i, body = job
            return body, kicad_loads(stage(self.work, f'fr{i}', f'(version 1)\n{body}\n{_SENTINEL}'))
        with concurrent.futures.ThreadPoolExecutor(WORKERS) as pool:
            loaded = [b for b, ok in pool.map(loads, enumerate(bodies)) if ok]
        self.assertEqual(loaded, [], 'the table is stale')
        self.assertFalse(kicad_loads(stage(self.work, 'nov', _SENTINEL)), 'no (version) line')

    @unittest.skipUnless(CLI, 'no kicad-cli')
    def test_the_rule_before_it_goes_too(self):
        """A size rule ahead of the refused one binds nothing either. (An
        `assertion` ahead of it does still fire: `compile_file` ends with its
        sentinel for that reason.)"""
        width = "(rule \"w\" (condition \"A.Type == 'Track'\") (constraint track_width (min 5mm)))\n"
        refused = "(rule \"c\" (condition \"A.onLayer('F.Cu')\") (constraint clearance (min 0.1mm)))\n"

        def flagged(name, body):
            return sum(v['type'] == 'track_width'
                       for v in kicad_drc(stage(self.work, name, '(version 1)\n' + body)))
        self.assertGreater(flagged('wo', width), 0)
        self.assertEqual(flagged('wb', width + refused), 0)
        self.assertEqual(flagged('wa', refused + width), 0)

    def test_the_finding_names_the_file(self):
        dru = ("(version 1)\n"
               "(rule \"good\" (condition \"A.Type == 'Track'\") (constraint track_width (min 0.5mm)))\n"
               "(rule \"typo\" (condition \"A.onLayer('F.Cu')\") (constraint track_width (min 0.7mm)))\n"
               "(rule \"num\" (condition \"A.NetName == 1\") (constraint track_width (min 0.9mm)))\n")
        path = stage(self.work, 'f', dru)
        dr = DesignRules.from_project(parse_kicad_pcb(path), path)
        self.assertEqual([r['rule'] for r in dr.kicad_refused], ['typo', 'num'])
        lines = dr.report_lines()
        self.assertTrue(lines[0].startswith('.kicad_dru: KiCad ENFORCES NO RULE OF THIS FILE'), lines)
        self.assertIn("rule 'typo'", lines[0])
        self.assertIn("A.onLayer() is not a function it knows", lines[0])
        self.assertIn("still reads the file's other rules", lines[0])
        self.assertIn("rule 'num'", lines[1])
        # the rule it cannot read is not said to be enforced
        rule = [ln for ln in lines if ln.startswith(".kicad_dru rule 'num'")]
        self.assertEqual(len(rule), 1, lines)
        self.assertNotIn('KiCad will still enforce', rule[0])
        # disclosure: what is read stays read, the refused onLayer rule included
        nid = next(i for i, n in dr.net_names.items() if n == 'DEF')
        self.assertEqual(dr.resolve('track_width', dr.item_for_net(nid, 'track', 'F.Cu'),
                                    None, 'F.Cu').min, 0.7)
        self.assertEqual(dr.resolve('track_width', dr.item_for_net(nid, 'track', 'B.Cu'),
                                    None, 'B.Cu').min, 0.5)
        clean = stage(self.work, 'c', "(version 1)\n(rule \"good\" (condition \"A.Type == 'Track'\") "
                                      "(constraint track_width (min 0.5mm)))\n")
        dr = DesignRules.from_project(parse_kicad_pcb(clean), clean)
        self.assertEqual(dr.kicad_refused, [])
        self.assertFalse(any('ENFORCES NO RULE' in ln for ln in dr.report_lines()))

    def _route_board(self, name, dru):
        from test_1615_edge_rule_pads import EDGE_X
        from test_1615_edge_rule_pads import stage as board_stage
        return board_stage(self.work, name, EDGE_X, dru)

    def test_check_drc_says_it(self):
        path = stage(self.work, 'cd', "(version 1)\n(rule \"typo\" (condition \"A.onLayer('F.Cu')\") "
                                      "(constraint clearance (min 0.1mm)))\n")
        out = path + '.json'
        r = subprocess.run([sys.executable, '-X', 'utf8', os.path.join(REPO, 'py_router', 'check_drc.py'),
                            path, '--json', out], cwd=REPO, capture_output=True, text=True,
                           encoding='utf-8', timeout=300)
        self.assertNotIn('Traceback', r.stdout + r.stderr)
        self.assertIn('KiCad ENFORCES NO RULE OF THIS FILE', r.stdout)
        with open(run_utils.evidence(out), encoding='utf-8') as fh:
            self.assertEqual([x['rule'] for x in json.load(fh)['kicad_refuses_file']], ['typo'])

    def test_route_says_both(self):
        """`memberOfFootprint` in the JSON's unsupported_rules, and the file's finding."""
        path = self._route_board('rt', "(version 1)\n"
                                 "(rule \"pads\" (condition \"A.memberOfFootprint('J3')\") "
                                 "(constraint clearance (min 0.1mm)))\n"
                                 "(rule \"typo\" (condition \"A.onLayer('F.Cu')\") "
                                 "(constraint track_width (min 0.2mm)))\n")
        out = os.path.join(self.work, 'rt_out.kicad_pcb')
        r = subprocess.run([sys.executable, '-X', 'utf8', os.path.join(REPO, 'py_router', 'route.py'),
                            path, out], cwd=REPO, capture_output=True, text=True, encoding='utf-8',
                           timeout=600)
        self.assertNotIn('Traceback', r.stdout + r.stderr)
        self.assertIn('KiCad ENFORCES NO RULE OF THIS FILE', r.stdout)
        self.assertIn("rule 'pads': NOT honoured (clearance scoped by A.memberOfFootprint()", r.stdout)
        rows = [ln for ln in r.stdout.splitlines() if ln.startswith('JSON_SUMMARY:')]
        self.assertTrue(rows, r.stdout[-400:])
        block = json.loads(rows[-1].split(':', 1)[1])['design_rules']
        self.assertIn('pads', block['unsupported_rules'])
        self.assertEqual([x['rule'] for x in block['kicad_refuses_file']], ['typo'])

    def test_member_of_footprint_is_named(self):
        path = stage(self.work, 'mf', "(version 1)\n"
                     "(rule \"one\" (condition \"A.Type == 'Via' && B.memberOfFootprint('S*')\") "
                     "(constraint clearance (min 0.1mm)))\n"
                     "(rule \"size\" (condition \"A.memberOfFootprint('S1')\") "
                     "(constraint hole_size (min 0.2mm)))\n"
                     "(rule \"typed\" (condition \"A.Type == 'Via' && B.Type == 'Pad'\") "
                     "(constraint clearance (min 0.1mm)))\n")
        dr = DesignRules.from_project(parse_kicad_pcb(path), path)
        self.assertEqual([n for n, _w in dr.unsupported()], ['one'])
        self.assertIn('B.memberOfFootprint()', dr.unsupported()[0][1])

    def test_zone_is_the_model(self):
        import test_1643_zone_edge_gap as t
        for i, (cond, _binds, want) in enumerate(ZONE):
            with self.subTest(cond=cond):
                d = os.path.join(self.work, f'zm{i}')
                os.makedirs(d)
                pcb = t.stage(d, 0.2, 0.2, 0.05, '(rule e (constraint edge_clearance (min 0.4mm)) '
                                                 f'(condition "{cond}"))', 0.1, 0.97)
                data = parse_kicad_pcb(pcb)
                net = next(n for n, x in data.nets.items() if x.name == 'GND')
                self.assertEqual(PadEdgeRules(data, pcb).for_zone(net, 'F.Cu', 0.05)[:2], want)

    @unittest.skipUnless(CLI, 'no kicad-cli')
    def test_zone_in_kicad(self):
        """A 0.97 mm isthmus fills whole at 0.05 mm and splits at 0.4 mm."""
        import test_1643_zone_edge_gap as t
        for i, (cond, binds, _want) in enumerate(ZONE):
            with self.subTest(cond=cond):
                d = os.path.join(self.work, f'zk{i}')
                os.makedirs(d)
                pcb = t.stage(d, 0.2, 0.2, 0.05, '(rule e (constraint edge_clearance (min 0.4mm)) '
                                                 f'(condition "{cond}"))', 0.1, 0.97)
                self.assertEqual(not t.kicad_whole(CLI, pcb), binds, 'the table is stale')

    def test_both_parse_paths(self):
        # ipc-migration: DesignRules.from_pcbnew is the SWIG GUI's loader and
        # the pcbnew builder it reads was deleted; the IPC front has no live
        # loader, so its rules come from DesignRules.from_project, which
        # test_readings_are_the_model and test_readings_are_kicad grade.
        self.skipTest('ipc-migration: DesignRules.from_pcbnew has no caller on '
                      'the IPC front, which reads the rules from the project '
                      '(graded by the other tests here)')
        python = find_kicad_python()
        if not python:
            self.skipTest('no KiCad python')
        path = stage(self.work, 'gui', None, project(
            [DEFAULT, cls('HS', 0, drop=('bus_width',))], {'HSNET': 'HS'}))
        conds = ["A.isPlated()", "A.hasNetclass('Default')", "A.hasExactNetclass('HS,Default')",
                 "A.Pad_Type == 'Through-hole'", "A.Pad_Type == 'NPTH, mechanical'",
                 "A.Via_Type == 'Blind'", "A.existsOnLayer('In*.Cu')", "A.existsOnLayer('F.Mask')",
                 "A.memberOfFootprint('x:*')", "A.memberOfFootprint('S*')"]
        r = subprocess.run([python, '-c', PCBNEW_PROBE, os.path.join(REPO, 'py_router'), path,
                            json.dumps(conds)], capture_output=True, text=True, encoding='utf-8',
                           timeout=300)
        got = [ln for ln in r.stdout.splitlines() if ln.startswith('RESULT ')]
        self.assertEqual(len(got), 1, r.stdout + r.stderr)
        got = json.loads(got[0][len('RESULT '):])
        self.assertEqual(got["A.isPlated()"], ['blind', 'micro', 'pth', 'through'])
        self.assertIn('track_hs', got["A.hasNetclass('Default')"])
        self.assertEqual(got["A.hasExactNetclass('HS,Default')"],
                         ['blind', 'micro', 'pth', 'through', 'track_hs'])
        self.assertEqual(got["A.Pad_Type == 'Through-hole'"], ['pth'])
        self.assertEqual(got["A.Pad_Type == 'NPTH, mechanical'"], ['npth'])
        self.assertEqual(got["A.Via_Type == 'Blind'"], ['blind'])
        self.assertEqual(got["A.existsOnLayer('In*.Cu')"], ['blind', 'micro', 'npth', 'pth', 'through'])
        self.assertEqual(got["A.existsOnLayer('F.Mask')"], ['connect', 'npth', 'pth', 'smd'])
        self.assertEqual(got["A.memberOfFootprint('x:*')"], ['connect', 'npth', 'pth', 'smd'])
        self.assertEqual(got["A.memberOfFootprint('S*')"], ['smd'])


if __name__ == '__main__':
    unittest.main()
