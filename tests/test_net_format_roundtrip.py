#!/usr/bin/env python3
"""
Net-token format round-trip test: writers must keep the output's net format
consistent with the INPUT board, never injecting KiCad-10 name-only nets
`(net "name")` into a KiCad-9 numeric board (which KiCad 9 reads as net-less),
nor numeric ids into a name-net board.

Background: the fanout tools pass a net_id_to_name map to add_tracks_and_vias_to_pcb
unconditionally, which used to force name-only output regardless of the board
version -> a KiCad-9 board came out as a hybrid (numeric net table + name-only
routing refs). The fix (`board_uses_name_nets`) detects the format from content
and ignores a passed name map for numeric boards.

Run:
    python3 tests/test_net_format_roundtrip.py
"""

import os
import re
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)

from kicad_parser import board_uses_name_nets
from kicad_writer import add_tracks_and_vias_to_pcb, add_tracks_to_pcb

V9 = '(kicad_pcb\n\t(version 20241229)\n\t(net 0 "")\n\t(net 1 "SIG1")\n)\n'
V10 = ('(kicad_pcb\n\t(version 20260206)\n'
       '\t(segment\n\t\t(start 0 0)\n\t\t(end 1 0)\n\t\t(width 0.1)\n'
       '\t\t(layer "F.Cu")\n\t\t(net "EXISTING")\n\t)\n)\n')
# Pre-2025 header but already carrying name-only refs (a previously round-tripped
# board) - must be treated as a name-net board so we stay consistent.
HYBRID = ('(kicad_pcb\n\t(version 20241229)\n\t(net 0 "")\n\t(net 1 "SIG1")\n'
          '\t(segment\n\t\t(start 0 0)\n\t\t(end 1 0)\n\t\t(width 0.1)\n'
          '\t\t(layer "F.Cu")\n\t\t(net "SIG1")\n\t)\n)\n')

TRACK = [{'start': (10.0, 10.0), 'end': (11.0, 10.0), 'width': 0.1,
          'layer': 'F.Cu', 'net_id': 1}]
NAME_MAP = {1: 'SIG1'}


def _write(content):
    fd, p = tempfile.mkstemp(suffix=".kicad_pcb")
    os.close(fd)
    with open(p, 'w') as f:
        f.write(content)
    return p


def _added_net_token(out_path, board_in):
    """The net token on the NEW track (10,10)->(11,10), from the output file."""
    txt = open(out_path).read()
    m = re.search(r'\(segment\s+\(start\s+10\b.*?\(net\s+("?[^)"]*"?)\)', txt, re.DOTALL)
    return m.group(1) if m else None


def main():
    results = []

    def check(name, ok, detail=""):
        results.append((name, ok))
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}{('  ' + detail) if detail else ''}")

    # 1. Format detection from content.
    check("board_uses_name_nets: KiCad 9 numeric -> False", not board_uses_name_nets(V9))
    check("board_uses_name_nets: KiCad 10 name -> True", board_uses_name_nets(V10))
    check("board_uses_name_nets: hybrid (v9 hdr + name refs) -> True", board_uses_name_nets(HYBRID))

    # 2. Round-trip: caller passes a name map in ALL cases; the writer must honor
    #    the board's own format (KiCad 9 -> numeric (net 1), name boards -> (net "SIG1")).
    tmp = []
    try:
        for label, content, want in (("KiCad 9", V9, '1'),
                                     ("KiCad 10", V10, '"SIG1"'),
                                     ("hybrid", HYBRID, '"SIG1"')):
            for fn in (add_tracks_and_vias_to_pcb, add_tracks_to_pcb):
                inp = _write(content)
                outp = _write("")
                tmp += [inp, outp]
                if fn is add_tracks_and_vias_to_pcb:
                    fn(inp, outp, TRACK, [], net_id_to_name=dict(NAME_MAP))
                else:
                    fn(inp, outp, TRACK, net_id_to_name=dict(NAME_MAP))
                tok = _added_net_token(outp, content)
                check(f"{label}: {fn.__name__} writes (net {want})", tok == want,
                      f"got (net {tok})")
    finally:
        for p in tmp:
            if os.path.exists(p):
                os.remove(p)

    passed = sum(1 for _, ok in results if ok)
    total = len(results)
    print("\n" + "=" * 60)
    print(f"  {passed}/{total} checks passed")
    print("=" * 60)
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
