#!/usr/bin/env python3
"""#1347: check_reachability's verdict grades the placement, not the routing.

The field walled every foreign track and via, so on a ROUTED board the tool
reported the copper as geometry: gp14 StickHub U1.37 read "CAGED for any
track width" (exit 1) where the same placement with its copper stripped read
PASSABLE by +1030.5 um. CAGED is the verdict the skills send back to
placement, so a congestion failure discarded the routed board and re-placed.

The verdict (and the exit code) now grade the placement: rippable routed
copper is set aside, while graphic copper and immutable (locked / held)
copper stay, being as fixed as a pad. The board AS ROUTED is measured too,
and when its copper alone cages the pad the report names it.

The fixture is cap_chain with a ring of foreign-net track drawn around C1.1
(net DPA_P; its partner J1.1 is 7.8 mm away):
  1. without_routed_copper sets the ring aside and keeps graphic and locked
     copper;
  2. routed ring -> exit 0, verdict PASSABLE, `graded` placement, and
     routed_copper says CAGED and that it cages the pad;
  3. the same ring LOCKED -> exit 1 CAGED: locked copper is geometry;
  4. the copper-free board -> routed_copper is None.

    python3 tests/test_1347_reachability_routed_copper.py
"""
import json
import os
import shutil
import sys
import tempfile
from types import SimpleNamespace as NS

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for _d in ('py_router', 'py_placer', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _d))
sys.path.insert(0, TESTS_DIR)

from kicad_parser import parse_kicad_pcb                 # noqa: E402
from kicad_writer import generate_segment_sexpr          # noqa: E402
from placement.reachability import without_routed_copper  # noqa: E402
from run_utils import check, evidence                    # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'cap_chain.kicad_pcb')
TOOL = os.path.join(ROOT, 'py_tools', 'check_reachability.py')
CX, CY, HALF = 17.52, 13.0, 0.5       # C1.1 and the ring's half-size
failures = []


def ok(label, cond, detail=''):
    print(f"  [{'ok' if cond else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not cond:
        failures.append(label)


def ringed(tmp, name, locked=False):
    """cap_chain with a DPB_P track square around C1.1 on F.Cu."""
    path = os.path.join(tmp, name)
    shutil.copy(BOARD, path)
    nid = next(n.net_id for n in parse_kicad_pcb(path).nets.values()
               if n.name == 'DPB_P')
    x0, y0, x1, y1 = CX - HALF, CY - HALF, CX + HALF, CY + HALF
    corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)]
    sx = [generate_segment_sexpr(a, b, 0.2, 'F.Cu', nid, 'DPB_P')
          for a, b in zip(corners, corners[1:])]
    if locked:
        sx = [s.replace('(layer ', '(locked yes)\n\t\t(layer ') for s in sx]
    with open(path, encoding='utf-8') as f:
        c = f.read()
    lp = c.rfind(')')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(c[:lp] + '\n'.join(sx) + '\n' + c[lp:])
    return path


def run(board, tmp, tag, **kw):
    jp = os.path.join(tmp, f'{tag}.json')
    r = check([sys.executable, '-X', 'utf8', TOOL, board, '--pad', 'C1.1',
               '--json-out', jp], **kw)
    return json.load(open(evidence(jp, 'the JSON report'), encoding='utf-8')), \
        r.stdout + r.stderr


def main():
    tmp = tempfile.mkdtemp(prefix='t1347_')
    try:
        print("1: without_routed_copper")
        b = ringed(tmp, 'ring.kicad_pcb')
        pcb = parse_kicad_pcb(b)
        view, ns, nv = without_routed_copper(pcb)
        ok("the four ring tracks are set aside", (ns, nv) == (4, 0)
           and view.segments == [], f"{ns} segs, {nv} vias")
        ok("the board itself is untouched", len(pcb.segments) == 4)
        fake = NS(segments=[NS(graphic=True), NS(locked=True),
                            NS(held_by='sense:R1.1'), NS()],
                  vias=[NS(locked=True), NS()])
        v2, ns2, nv2 = without_routed_copper(fake)
        ok("graphic, locked and held copper stay",
           (len(v2.segments), len(v2.vias), ns2, nv2) == (3, 1, 1, 1),
           f"{len(v2.segments)} segs, {len(v2.vias)} vias kept")

        print("2: a ring of ROUTED copper")
        doc, log = run(b, tmp, 'routed', accept=True)
        ok("the verdict grades the placement: PASSABLE, exit 0",
           doc['verdict'] == 'PASSABLE' and doc.get('graded') == 'placement',
           doc['verdict'])
        rc = doc.get('routed_copper') or {}
        ok("the routed reading is CAGED and named as the cause",
           rc.get('verdict') == 'CAGED' and rc.get('cages_the_pad') is True
           and rc.get('segments_set_aside') == 4, str(rc)[:200])
        ok("the text names routed copper", 'ROUTED COPPER cages this pad' in log,
           log[-300:])

        print("3: the same ring LOCKED")
        bl = ringed(tmp, 'ring_locked.kicad_pcb', locked=True)
        ok("precondition: the parser reads the ring as locked",
           all(s.locked for s in parse_kicad_pcb(bl).segments))
        doc, log = run(bl, tmp, 'locked', code=1, refuse='CAGED')
        ok("locked copper is geometry: CAGED", doc['verdict'] == 'CAGED'
           and doc.get('routed_copper') is None, doc['verdict'])

        print("4: a copper-free board")
        doc, _ = run(BOARD, tmp, 'bare', accept=True)
        ok("no routed copper, no second reading",
           doc['verdict'] == 'PASSABLE' and doc.get('routed_copper') is None
           and doc.get('graded') == 'placement')
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    if failures:
        print(f"\nFAIL ({len(failures)}): " + '; '.join(failures))
        return 1
    print("\nAll checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
