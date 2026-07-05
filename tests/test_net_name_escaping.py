#!/usr/bin/env python3
"""Net-name escaping in the KiCad-10 writer (issue #312).

route.py writes KiCad-10 name-based nets as `(net "name")` using each net's
UNESCAPED display name. The parser (kicad_parser.extract_nets) keys name-based
net dedup on the RAW file text, so a net whose name contains a backslash or quote
-- e.g. `/GPIO24\\SPI1_RX(MISO)` -- must be re-escaped on write. Without it, the
writer emits a single backslash where the original board had it double-escaped;
the two raw tokens parse to two different synthetic net_ids for ONE net, the
routed segment lands on a duplicate net, and check_drc flags a phantom
pad-segment/pad-via short (neo6502: 90 phantom violations, gone after the fix).

    python3 tests/test_net_name_escaping.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_writer import (_escape_net_name, generate_segment_sexpr,
                          generate_via_sexpr)
from kicad_parser import _unescape_kicad_string, extract_nets, detect_kicad_version


def run():
    fails = []

    def check(name, cond):
        if not cond:
            fails.append(name)
        print(("  PASS " if cond else "  FAIL ") + name)

    # 1. escape is the exact inverse of the parser's unescape.
    for n in ['/GPIO24\\SPI1_RX(MISO)', '/PLAIN', 'has"quote', 'a\\b"c', '/GND']:
        check(f"round-trip {n!r}", _unescape_kicad_string(_escape_net_name(n)) == n)

    # 2. a plain name is byte-identical (no-op) -- the 99% case must not change.
    check("plain name unchanged", _escape_net_name('/VCC') == '/VCC')

    # 3. the writer emits the ESCAPED (double-backslash) token for a backslash net.
    bs = '/GPIO24\\SPI1_RX(MISO)'
    seg = generate_segment_sexpr((0, 0), (1, 1), 0.1, 'F.Cu', 5, net_name=bs)
    via = generate_via_sexpr(0, 0, 0.5, 0.3, ['F.Cu', 'B.Cu'], 5, net_name=bs)
    check("segment emits escaped backslash", '/GPIO24\\\\SPI1_RX(MISO)' in seg)
    check("segment does NOT emit bare backslash", '/GPIO24\\SPI1_RX(MISO)")' not in seg)
    check("via emits escaped backslash", '/GPIO24\\\\SPI1_RX(MISO)' in via)

    # 4. end-to-end: an original (escaped) declaration and a writer-emitted
    #    reference for the SAME net parse to ONE synthetic net_id (no duplicate).
    content = (
        '(kicad_pcb (version 20260206) (generator "test")\n'   # >= KICAD_10_MIN_VERSION
        '(net "/GPIO24\\\\SPI1_RX(MISO)")\n'   # original, escaped
        + generate_segment_sexpr((0, 0), (1, 1), 0.1, 'F.Cu', 5, net_name=bs) + '\n'
        ')\n'
    )
    nets, name_to_id = extract_nets(content, detect_kicad_version(content))
    ids = {nid for nid, net in nets.items() if net.name == bs}
    check("write+parse yields ONE net_id (no duplicate)", len(ids) == 1)

    print()
    if fails:
        print(f"FAIL: {len(fails)} check(s): {fails}")
        return 1
    print("ALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(run())
