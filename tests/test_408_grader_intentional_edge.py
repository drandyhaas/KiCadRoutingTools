#!/usr/bin/env python3
"""#408 grader side: the shared grading core accepts-by-design every edge-
clearance violation on a net the router routed into the board-edge band (a
card-edge connector / edge-mounted part), which the router emits as
intentional_edge_band_nets (a list of net names) in its JSON_SUMMARY.

Covers the pure logic (no kicad-cli): the log parser and the NET-SCOPED drop
(all edge items on an intentional net are dropped, non-edge classes and other
nets untouched). Run: python3 tests/test_408_grader_intentional_edge.py
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tests", "stress"))

from kicad_drc_compare import (parse_intentional_edge_nets, _drop_intentional_edge,
                               EDGE_KICAD_TYPES, EDGE_CD_TYPES)

fails = []


def ck(cond, msg):
    if not cond:
        fails.append(msg)


# -- log parsing: union of net names across steps, dedup, tolerate junk --------
# Real logs print each JSON_SUMMARY on ONE line (json.dumps), one per chain step.
LOG = "\n".join([
    "some noise line",
    'JSON_SUMMARY: {"routed_single": 3, "intentional_edge_band_nets": ["/RST0"]}',
    "not json: JSON_SUMMARY: {oops",
    'JSON_SUMMARY: {"total_routes": 1, "intentional_edge_band_nets": ["/RST0", "+5V"]}',
    'JSON_SUMMARY: {"min_clearance_used": 0.1}',
    # tolerate the older rich {net, ...} dict schema too
    'JSON_SUMMARY: {"intentional_edge_band_nets": [{"net": "/GND", "x": 1.0}]}',
])
nets = parse_intentional_edge_nets(LOG)
ck(nets == ["+5V", "/GND", "/RST0"], f"parse: expected sorted union, got {nets}")
ck(parse_intentional_edge_nets("no summaries here") == [], "parse: empty log -> []")

inets = frozenset(["/RST0", "+5V"])


def _kv(net, pos, typ="copper_edge_clearance"):
    return {"type": typ, "nets": frozenset([net]), "pos": pos}


# -- net-scoped drop: ALL edge items on an intentional net are removed ---------
near = _kv("/RST0", (0.45, 10.0))
far = _kv("/RST0", (0.45, 99.0))       # same net, far away -> STILL dropped (net-scoped)
other = _kv("/OTHER", (0.45, 10.0))    # not an intentional net -> kept
kept, dropped = _drop_intentional_edge([near, far, other], EDGE_KICAD_TYPES, inets)
ck(dropped == 2, f"drop: both /RST0 edge items dropped regardless of position, got {dropped}")
ck(kept == [other], "drop: only the non-intentional net's edge item survives")

# -- non-edge class on an intentional net is NEVER dropped --------------------
clr = _kv("/RST0", (0.45, 10.0), typ="clearance")
kept2, dropped2 = _drop_intentional_edge([clr], EDGE_KICAD_TYPES, inets)
ck(dropped2 == 0 and kept2 == [clr], "drop: non-edge class must survive")

# -- check_drc side uses its own edge types, same net scoping -----------------
seg = _kv("+5V", (25.0, 5.0), typ="segment-board-edge")
via = _kv("/OTHER", (3.0, 3.0), typ="via-board-edge")
kept3, dropped3 = _drop_intentional_edge([seg, via], EDGE_CD_TYPES, inets)
ck(dropped3 == 1 and kept3 == [via], "drop: check_drc edge on intentional net dropped, other kept")

# -- empty intentional set is a no-op ----------------------------------------
kept4, dropped4 = _drop_intentional_edge([near, far], EDGE_KICAD_TYPES, frozenset())
ck(dropped4 == 0 and kept4 == [near, far], "drop: empty intentional -> no-op")


if fails:
    print("FAIL:")
    for f in fails:
        print("  -", f)
    sys.exit(1)
print("PASS: #408 grader net-scoped intentional-edge subtraction "
      "(parse net names / drop-by-net / non-edge kept / no-op)")
