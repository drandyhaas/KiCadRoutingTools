#!/usr/bin/env python3
"""#980 / #1138: the restore test prices each pair as check_drc does, and its
prefilter box reaches every threshold.

`rip_up_reroute._saved_route_colliders` decides whether a ripped net's saved
copper may be put back. It priced every foreign item at one flat clearance
(#980), and prefiltered the board to a fixed 1 mm box around the saved copper,
so a collision past the box was never tested (#1138).

Checks:
  1. No config, or a config that declares nothing: the flat verdict.
  2. A class on a THIRD net only: still the flat verdict (the pair's own
     value, not the stamp floor `obstacle_clearance`).
  3. A class on the foreign net: a graze between the flat and the class value
     is refused -- seg/seg, seg/via, via/via and via/seg.
  4. A class on ONE of two restored nets: each restored item is priced at its
     own net.
  5. A .kicad_dru layer rule replaces the value (a relaxing rule admits what
     the flat value refused).
  6. Copper the step was HANDED (mark_input_copper) keeps the flat value: an
     inherited graze does not refuse a restore, a graze under the flat value
     still does.
  7. #1138: a 2 mm power track, a 1.2 mm class and a large via centred
     outside the old 1 mm box are all found.
  8. Every caller of the restore test in py_router passes `config=` (AST).

    python3 tests/test_980_restore_pairwise.py
"""
import ast
import glob
import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from routing_config import GridRouteConfig                  # noqa: E402
from synth import make_seg, make_via, make_pcb              # noqa: E402
import rip_up_reroute as rr                                 # noqa: E402

OWN, OWN2, FOREIGN, THIRD = 1, 2, 9, 7
failures = []


def check(label, got, want):
    ok = got == want
    print(f"  [{'ok' if ok else 'FAIL'}] {label}: got {got!r}, want {want!r}")
    if not ok:
        failures.append(label)


def cfg(classes=None, routed=(OWN,), layer_clearances=None):
    c = GridRouteConfig(clearance=0.2, track_width=0.2, via_size=0.6,
                        via_drill=0.3, layers=['F.Cu', 'B.Cu'], grid_step=0.05)
    if classes:
        c.set_net_clearances(dict(classes), routed_net_ids=list(routed))
    if layer_clearances:
        c.layer_clearances = dict(layer_clearances)
    return c


def collides(saved_segs, saved_vias, board_segs=(), board_vias=(),
             config=None, own=(OWN,), mark=False):
    pcb = make_pcb(segments=list(board_segs), vias=list(board_vias))
    if mark:
        rr.mark_input_copper(pcb)
    return rr._saved_route_collides(
        {'new_segments': list(saved_segs), 'new_vias': list(saved_vias)},
        pcb, list(own), 0.2, config=config)


# Two parallel 0.2 mm tracks with a 0.3 mm edge gap: clear at 0.2, a graze
# under a 0.35 class.
def own_seg(net=OWN, w=0.2):
    return make_seg(0, 0, 10, 0, net_id=net, width=w)


def foreign_seg(y=0.5, net=FOREIGN, w=0.2, layer='F.Cu'):
    return make_seg(0, y, 10, y, net_id=net, width=w, layer=layer)


print("1. flat verdicts")
check("no config, 0.3 gap", collides([own_seg()], [], [foreign_seg()]), False)
check("no config, 0.1 gap", collides([own_seg()], [], [foreign_seg(0.3)]), True)
check("inert config, 0.3 gap",
      collides([own_seg()], [], [foreign_seg()], config=cfg()), False)

print("2. a class on a third net only")
check("THIRD 0.35, 0.3 gap", collides(
    [own_seg()], [], [foreign_seg()],
    config=cfg({THIRD: 0.35}, routed=(OWN, THIRD))), False)

print("3. a class on the foreign net")
wide = cfg({FOREIGN: 0.35})
check("seg/seg 0.3 gap at 0.35", collides([own_seg()], [], [foreign_seg()],
                                          config=wide), True)
check("seg/seg on another layer", collides(
    [own_seg()], [], [foreign_seg(layer='B.Cu')], config=wide), False)
# via r 0.3 centred 0.7 from a 0.2 track: edge gap 0.3
fv = make_via(5, 0.7, net_id=FOREIGN, size=0.6)
check("seg/via flat", collides([own_seg()], [], board_vias=[fv],
                               config=cfg()), False)
check("seg/via at 0.35", collides([own_seg()], [], board_vias=[fv],
                                  config=wide), True)
ov = make_via(0, 0, net_id=OWN, size=0.6)
fv2 = make_via(0.9, 0, net_id=FOREIGN, size=0.6)   # edge gap 0.3
check("via/via flat", collides([], [ov], board_vias=[fv2], config=cfg()), False)
check("via/via at 0.35", collides([], [ov], board_vias=[fv2], config=wide), True)
fs = make_seg(-5, 0.7, 5, 0.7, net_id=FOREIGN)      # edge gap 0.3 to the via
check("via/seg flat", collides([], [ov], [fs], config=cfg()), False)
check("via/seg at 0.35", collides([], [ov], [fs], config=wide), True)

print("4. each restored item at its own net")
pn = cfg({OWN2: 0.35}, routed=(OWN, OWN2))
check("restored OWN (no class) beside FOREIGN", collides(
    [own_seg(OWN)], [], [foreign_seg()], config=pn, own=(OWN, OWN2)), False)
check("restored OWN2 (0.35) beside FOREIGN", collides(
    [own_seg(OWN2)], [], [foreign_seg()], config=pn, own=(OWN, OWN2)), True)

print("5. a .kicad_dru layer rule replaces the value")
check("0.17 gap, flat", collides([own_seg()], [], [foreign_seg(0.37)],
                                 config=cfg()), True)
check("0.17 gap, F.Cu rule 0.15", collides(
    [own_seg()], [], [foreign_seg(0.37)],
    config=cfg(layer_clearances={'F.Cu': 0.15})), False)
check("0.3 gap, F.Cu rule 0.35 over no class", collides(
    [own_seg()], [], [foreign_seg()],
    config=cfg(layer_clearances={'F.Cu': 0.35})), True)

print("6. input copper keeps the flat value")
check("inherited 0.3 graze under a 0.35 class", collides(
    [own_seg()], [], [foreign_seg()], config=wide, mark=True), False)
check("inherited 0.1 graze", collides(
    [own_seg()], [], [foreign_seg(0.3)], config=wide, mark=True), True)
check("relaxing rule still applies to input copper", collides(
    [own_seg()], [], [foreign_seg(0.37)], mark=True,
    config=cfg(layer_clearances={'F.Cu': 0.15})), False)
pcb = make_pcb(segments=[foreign_seg()])
rr.mark_input_copper(pcb)
pcb.segments = [foreign_seg()]   # a NEW object at the same place
check("copper laid after the mark is priced at the pair", rr._saved_route_collides(
    {'new_segments': [own_seg()], 'new_vias': []}, pcb, [OWN], 0.2,
    config=wide), True)
rr.forget_input_copper(pcb)
check("forget_input_copper drops the mark",
      getattr(pcb, '_input_copper', None), None)

print("7. #1138: the prefilter box reaches every threshold")
check("2 mm track, 0.2 track 1.25 away (edge gap 0.15)", collides(
    [own_seg(w=2.0)], [], [foreign_seg(1.25)]), True)
check("1.2 mm class, 0.2 tracks 1.2 apart (edge gap 1.0)", collides(
    [own_seg()], [], [foreign_seg(1.2)], config=cfg({FOREIGN: 1.2})), True)
check("2 mm via centred 1.5 from a 0.2 track (edge gap 0.4)", collides(
    [own_seg()], [], board_vias=[make_via(5, 1.5, net_id=FOREIGN, size=2.2)],
    config=cfg({FOREIGN: 0.5})), True)
check("still clear past the threshold", collides(
    [own_seg(w=2.0)], [], [foreign_seg(1.35)]), False)

print("8. every restore caller passes config=")
NAMES = {'_saved_route_collides', '_saved_route_colliders', '_src517',
         '_pe_collides', '_pe_colliders', 'partition_force_restores'}
missing = []
for path in sorted(glob.glob(os.path.join(ROOT, 'py_router', '*.py'))):
    tree = ast.parse(open(path, encoding='utf-8').read())
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        f = node.func
        name = f.id if isinstance(f, ast.Name) else (
            f.attr if isinstance(f, ast.Attribute) else None)
        if name in NAMES and not any(k.arg == 'config' for k in node.keywords):
            if os.path.basename(path) == 'rip_up_reroute.py' and \
                    name == '_saved_route_colliders':
                continue   # the boolean wrapper forwards its own config
            missing.append(f"{os.path.relpath(path, ROOT)}:{node.lineno} {name}")
check("callers without config=", missing, [])

print()
if failures:
    print(f"FAIL: {len(failures)} check(s): {failures}")
    sys.exit(1)
print("PASS: test_980_restore_pairwise")
