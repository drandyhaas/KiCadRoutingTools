#!/usr/bin/env python3
"""#1180: net 0 is never graded for connectivity, on any path.

check_connected's no-component path selected a net by "has copper and has
pads". Net 0 used to fail that test because it had no copper, until #908
parsed footprint copper (solder-jumper bridges, a SOT89 tab) as net-0
graphic segments. On a KiCad 9 file, whose parse keeps nets[0] (#497), every
no-net pad then read as a disconnected component: One-Air-Max reported
"(net 0): 7 disconnected components" where KiCad reported 0 unconnected, and
board_score counted 6 `broken` it could not name.

esp_prog is a KiCad 10 file (no nets[0]), with U2's tab as 8 net-0 segments
and 7 no-net pads; restoring nets[0], as a KiCad 9 parse and
build_pcb_data_from_board both do, reproduces the KiCad 9 shape.

Checks:
  1. The precondition holds: the tab parses as net-0 segments and there are
     >= 2 no-net pads (otherwise this test proves nothing).
  2. With nets[0] restored, the default path, a `--nets '*'` pattern and a
     `--component U2` filter grade nothing for net 0.
  3. The grade is otherwise unchanged: restoring nets[0] moves no other row.

    python3 tests/test_1180_net0_not_graded.py
"""
import contextlib
import io
import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from kicad_parser import parse_kicad_pcb, Net               # noqa: E402
from check_connected import run_connectivity_check           # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def grade(**kw):
    with contextlib.redirect_stdout(io.StringIO()):
        return run_connectivity_check(BOARD, quiet=True, **kw)


def parse(with_net0):
    with contextlib.redirect_stdout(io.StringIO()):
        p = parse_kicad_pcb(BOARD)
    if with_net0:
        p.nets[0] = Net(net_id=0, name='')
        p.nets[0].pads = list(p.pads_by_net.get(0, []))
    return p


def rows(res):
    return sorted((r['net_id'], r.get('num_components'),
                   r.get('num_pads') if 'num_pads' in r else None)
                  for r in res)


print("1. precondition")
base = parse(False)
net0_segs = [s for s in base.segments if s.net_id == 0]
net0_pads = base.pads_by_net.get(0, [])
check("U2's tab parses as net-0 segments", len(net0_segs) >= 2,
      f"{len(net0_segs)} segments")
check(">= 2 no-net pads", len(net0_pads) >= 2, f"{len(net0_pads)} pads")
check("KiCad 10 parse has no nets[0]", 0 not in base.nets)

print("2. net 0 is not graded with nets[0] restored")
for label, kw in (("default path", {}),
                  ("--nets '*'", {'net_patterns': ['*']}),
                  ("--component U2", {'component': 'U2'})):
    res = grade(pcb_data=parse(True), **kw)
    bad = [r for r in res if r['net_id'] == 0]
    check(label, not bad, f"{len(bad)} net-0 rows")

print("3. nothing else moves")
for label, kw in (("default path", {}), ("--nets '*'", {'net_patterns': ['*']})):
    a = rows(grade(pcb_data=parse(False), **kw))
    b = rows(grade(pcb_data=parse(True), **kw))
    check(label, a == b, f"{len(a)} vs {len(b)} rows")

if failures:
    print(f"\nFAILED: {failures}")
    sys.exit(1)
print("\nALL PASSED")
