#!/usr/bin/env python3
"""The fill model's board-edge gap reads the live board first, as its hole
clearance does.

`plane_fill_model.board_edge_rule` (the edge guard every default ZoneFillModel
keeps) read `min_copper_edge_clearance` only from the board file's sibling
.kicad_pro. Mid-plan the GUI's writeback pins the live rule up to the fab floor
in pcbnew's memory while the file beside the board is still the original, so
the GUI's model poured to an edge KiCad's live refill keeps clear of -- the
CLI reads the project its previous step wrote. Found by the 7978996d..HEAD
parity audit. It now asks `PCBData.live_rules_provider` first (#1187's
channel, which `_live_design_rules` now answers the edge rule through), then
the file.

Checks:
  1. A live answer wins over the file (0.5 live, 0.3 in the project).
  2. No live answer for the key: the file (0.3); no provider: the file.
  3. A declared live 0 is 0, not the file's value.
  4. connection_width_floor, which reads the same provider, is unmoved: a
     live answer carrying ONLY the edge rule still sends it to the file.
  5. _live_design_rules asks for the edge rule.

    python3 tests/test_fill_model_edge_rule_live.py
"""
import json
import os
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from synth import make_pcb                                  # noqa: E402
import plane_fill_model as pfm                              # noqa: E402
import fix_kicad_drc_settings as F                          # noqa: E402

failures = []


def check(label, got, want):
    ok = got == want
    print(f"  [{'ok' if ok else 'FAIL'}] {label}: got {got!r}, want {want!r}")
    if not ok:
        failures.append(label)


d = tempfile.mkdtemp(prefix='edgerule_')
try:
    board = os.path.join(d, 'b.kicad_pcb')
    open(board, 'w').write('(kicad_pcb (version 20241229))\n')
    with open(os.path.join(d, 'b.kicad_pro'), 'w') as f:
        json.dump({'board': {'design_settings': {'rules': {
            'min_copper_edge_clearance': 0.3, 'min_track_width': 0.2}}}}, f)

    def pcb(live=None, provider=True):
        p = make_pcb()
        p.source_path = board
        if provider:
            p.live_rules_provider = lambda: live
        return p

    print("1. the live board's rule wins")
    check("live 0.5 over the file's 0.3",
          pfm.board_edge_rule(pcb({'min_copper_edge_clearance': 0.5})), 0.5)
    print("2. the file without a live answer")
    check("live answers other keys", pfm.board_edge_rule(pcb({'min_track_width': 0.1})), 0.3)
    check("live answers nothing", pfm.board_edge_rule(pcb(None)), 0.3)
    check("no provider (the CLI)", pfm.board_edge_rule(pcb(provider=False)), 0.3)
    print("3. a declared live 0")
    check("is 0", pfm.board_edge_rule(pcb({'min_copper_edge_clearance': 0.0})), 0.0)
    print("4. connection_width_floor is unmoved")
    check("edge-only live answer reads the file's track floor",
          F.connection_width_floor(pcb({'min_copper_edge_clearance': 0.5})), 0.2)
    check("a live track rule still wins",
          F.connection_width_floor(pcb({'min_track_width': 0.15,
                                        'min_copper_edge_clearance': 0.5})), 0.15)
    print("5. the provider asks for the edge rule")
    src = open(os.path.join(ROOT, 'py_router', 'kicad_parser.py'), encoding='utf-8').read()
    body = src[src.index('def _live_design_rules'):]
    body = body[:body.index('\ndef ')]
    check("_live_design_rules names min_copper_edge_clearance",
          "'min_copper_edge_clearance'" in body, True)
finally:
    shutil.rmtree(d, ignore_errors=True)

if failures:
    print(f"\nFAILED: {len(failures)}: {failures}")
    sys.exit(1)
print("\nALL PASS")
