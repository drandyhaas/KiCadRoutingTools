#!/usr/bin/env python3
"""parse_kicad_pcb must pick the net encoding from the file CONTENT, not the
(version ...) stamp.

A file stamped >= KICAD_10_MIN_VERSION can still carry the numeric top-level
net table `(net N "name")` with `(net N)` refs (KiCad 10.0.x pcbnew writes
version 20250513 with a numeric table; other emitters/converters do the same).
extract_nets used to switch on the stamp alone, found no `(net "name")` refs,
and returned ZERO nets -- every pcb.nets consumer went silently inert.

    python3 tests/test_parser_net_encoding_by_content.py
"""
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_router'))  # #522
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_tools'))  # #522

from kicad_parser import parse_kicad_pcb

HEADER = '''(kicad_pcb (version %s) (generator "pcbnew")
  (general (thickness 1.6))
  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))
'''

NUMERIC_BODY = '''  (net 0 "")
  (net 1 "GND")
  (net 2 "SIG")
  (footprint "R" (layer "F.Cu") (at 10 10)
    (property "Reference" "R1" (at 0 0) (layer "F.SilkS"))
    (pad "1" smd rect (at -1 0) (size 1 1) (layers "F.Cu") (net 1 "GND"))
    (pad "2" smd rect (at 1 0) (size 1 1) (layers "F.Cu") (net 2 "SIG")))
  (segment (start 11 10) (end 15 10) (width 0.2) (layer "F.Cu") (net 2) (uuid "s1"))
  (via (at 15 10) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") (net 2) (uuid "v1"))
)
'''

NAME_BODY = '''  (footprint "R" (layer "F.Cu") (at 10 10)
    (property "Reference" "R1" (at 0 0) (layer "F.SilkS"))
    (pad "1" smd rect (at -1 0) (size 1 1) (layers "F.Cu") (net "GND"))
    (pad "2" smd rect (at 1 0) (size 1 1) (layers "F.Cu") (net "SIG")))
  (segment (start 11 10) (end 15 10) (width 0.2) (layer "F.Cu") (net "SIG") (uuid "s1"))
  (via (at 15 10) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") (net "SIG") (uuid "v1"))
)
'''


def _parse(text):
    d = tempfile.mkdtemp()
    path = os.path.join(d, 'b.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(text)
    return parse_kicad_pcb(path)


def _assert_board(label, pcb, check):
    names = {n.name: nid for nid, n in pcb.nets.items()}
    check(f"{label}: GND and SIG nets present (got {sorted(names)})",
          'GND' in names and 'SIG' in names)
    gnd, sig = names.get('GND'), names.get('SIG')
    pads = {p.pad_number: p for p in pcb.footprints['R1'].pads}
    check(f"{label}: pad 1 on GND", pads['1'].net_id == gnd and gnd)
    check(f"{label}: pad 2 on SIG", pads['2'].net_id == sig and sig)
    check(f"{label}: SIG net lists its pad",
          sig in pcb.nets and [p.pad_number for p in pcb.nets[sig].pads] == ['2'])
    check(f"{label}: segment on SIG", [s.net_id for s in pcb.segments] == [sig])
    check(f"{label}: via on SIG", [v.net_id for v in pcb.vias] == [sig])


def run():
    fails = []

    def check(name, cond):
        if not cond:
            fails.append(name)

    cases = [
        # The bug: KiCad-10 stamps over a numeric net table.
        ("v20260206 stamp + numeric table", HEADER % 20260206 + NUMERIC_BODY),
        ("v20250513 stamp + numeric table", HEADER % 20250513 + NUMERIC_BODY),
        # Inverse: pre-10 stamp over name-only nets.
        ("v20241229 stamp + name nets", HEADER % 20241229 + NAME_BODY),
        # Regression guards: the canonical pairings.
        ("KiCad 9 numeric", HEADER % 20241229 + NUMERIC_BODY),
        ("KiCad 10 name nets", HEADER % 20260206 + NAME_BODY),
    ]
    for label, text in cases:
        _assert_board(label, _parse(text), check)

    # Numeric ids are preserved from the table (not synthesized).
    pcb = _parse(HEADER % 20260206 + NUMERIC_BODY)
    check("numeric ids kept from the table",
          {nid: n.name for nid, n in pcb.nets.items() if nid} == {1: 'GND', 2: 'SIG'})

    print("=" * 60)
    if fails:
        for f in fails:
            print(f"  FAIL  {f}")
        print(f"FAILED: {len(fails)} check(s)")
        return 1
    print("PASS: net encoding picked from content for all stamp/content pairings")
    return 0


if __name__ == '__main__':
    sys.exit(run())
