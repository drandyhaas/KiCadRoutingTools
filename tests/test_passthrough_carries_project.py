#!/usr/bin/env python3
"""A board written through unchanged carries its project (#441, #513).

pcb_io_utils.passthrough_copy writes the output as a copy of the input when a
routing or fanout stage produced no new copper. It copied the .kicad_pcb
ALONE, so the output had no project until main()'s writeback at the very end,
and everything that read the output's rules mid-run read nothing. Found on
neo6502 (set1, the route.py GND step: "nothing to route", the board passed
through to the plane finalize): every copper-to-hole lookup of the KiCad
oracle and of the final reconciliation returned 0.0 against a board
declaring 0.25. Every other routing writer seeds the output's siblings
BEFORE the board exists (seed_project_for_output); the passthrough now does
the same.

Asserted:
  * the passthrough output has the input's .kicad_pro and .kicad_dru;
  * the declared floor resolves on it (0.25 from fab_floor_origin over a
    relaxed 0.1 rule -- a mid-chain project, neo6502's shape);
  * an output sibling that already exists is never overwritten;
  * in place (output == input) nothing is copied.

    python3 tests/test_passthrough_carries_project.py
"""
import contextlib
import io
import json
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from pcb_io_utils import passthrough_copy  # noqa: E402

fails = []


def check(name, cond, detail=''):
    print(("  ok  " if cond else " FAIL ") + name + (f"  {detail}" if detail else ''))
    if not cond:
        fails.append(name)


BOARD = '''(kicad_pcb (version 20240108) (generator test)
 (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (44 "Edge.Cuts" user))
 (net 0 "")
 (gr_rect (start 0 0) (end 20 20) (layer "Edge.Cuts") (width 0.1))
 (footprint "t:mh" (layer "F.Cu") (at 10 10)
  (property "Reference" "MH1" (at 0 0) (layer "F.SilkS"))
  (pad "" np_thru_hole circle (at 0 0) (size 1 1) (drill 1) (layers "*.Cu" "*.Mask")))
)
'''
PROJECT = {'board': {'design_settings': {'rules': {'min_hole_clearance': 0.1}}},
           'kicad_routing_tools': {'fab_floor_origin': {'min_hole_clearance': 0.25}}}
DRU = '(version 1)\n(rule inner (layer inner) (constraint clearance (min 0.15mm)))\n'


def _stage(td):
    src = os.path.join(td, 'in.kicad_pcb')
    with open(src, 'w', encoding='utf-8') as f:
        f.write(BOARD)
    with open(os.path.join(td, 'in.kicad_pro'), 'w', encoding='utf-8') as f:
        json.dump(PROJECT, f)
    with open(os.path.join(td, 'in.kicad_dru'), 'w', encoding='utf-8') as f:
        f.write(DRU)
    return src


def _read(p):
    with open(p, encoding='utf-8') as f:
        return f.read()


with tempfile.TemporaryDirectory() as td:
    src = _stage(td)
    dst = os.path.join(td, 'out.kicad_pcb')
    check("a passthrough copy is made", passthrough_copy(src, dst) is True)
    pro = os.path.join(td, 'out.kicad_pro')
    check("the output carries the input's .kicad_pro",
          os.path.isfile(pro) and json.loads(_read(pro)) == PROJECT)
    dru = os.path.join(td, 'out.kicad_dru')
    check("...and its .kicad_dru", os.path.isfile(dru) and _read(dru) == DRU)

    from kicad_parser import parse_kicad_pcb
    from obstacle_map import resolve_hole_clearance
    from routing_config import GridRouteConfig
    with contextlib.redirect_stdout(io.StringIO()):
        got = resolve_hole_clearance(parse_kicad_pcb(dst), GridRouteConfig())
    check("the declared copper-to-hole floor resolves on the output", got == 0.25,
          f"got {got}")

with tempfile.TemporaryDirectory() as td:
    src = _stage(td)
    dst = os.path.join(td, 'out.kicad_pcb')
    mine = {'board': {'design_settings': {'rules': {'min_hole_clearance': 0.3}}}}
    with open(os.path.join(td, 'out.kicad_pro'), 'w', encoding='utf-8') as f:
        json.dump(mine, f)
    passthrough_copy(src, dst)
    check("an existing output project is never overwritten",
          json.loads(_read(os.path.join(td, 'out.kicad_pro'))) == mine)

with tempfile.TemporaryDirectory() as td:
    src = _stage(td)
    check("in place: nothing copied", passthrough_copy(src, src) is False
          and _read(src) == BOARD)

print("-" * 60)
if fails:
    print(f"{len(fails)} FAILED: " + "; ".join(fails))
    sys.exit(1)
print("ALL PASS")
