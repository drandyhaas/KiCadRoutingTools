#!/usr/bin/env python3
"""#1842: `route.py --via-kinds` lets one search change layers through several
kinds of via, and `--jog-layers` keeps a layer to short via-to-via jogs.

A route step laid one kind of via: the through via, or the one `--via-span`
names, on that span's layers only. On the fixture
(tests/fixtures/1842/mixed_kinds.py, synthetic: four nets from F.Cu pads
walled in on F.Cu and covered on B.Cu, to B.Cu pads walled in on B.Cu and
covered on F.Cu) a net needs a micro via F.Cu-In1.Cu, a buried via
In1.Cu-In6.Cu and a micro via In6.Cu-B.Cu, so no step of one kind completes
any of them.

Checks:
  1. the search core, on a hand-built map (no board): with no kinds, an
     empty list and a single through kind the search returns the same path
     in the same number of iterations; a kind is used only between layers it
     joins and only where ITS map is clear; with a minimum run two vias of
     a path stand apart, reached by one jog-then-via edge; a jog-only layer
     carries one straight jog no longer than its cap between two vias;
  2. `via_kinds.resolve_via_kinds` reads the pairs, their types and sizes,
     and refuses an odd list, one span twice and a kind larger than the
     step's via; route.py refuses the same by name, and --via-kinds with
     --via-span;
  3. legality per transition, on the fixture's base map: over the B.Cu pad a
     through via is blocked and both shorter kinds are legal; on the In3.Cu
     track the micro via is legal and the buried via is not; on the F.Cu
     frame the F.Cu micro via is blocked and the B.Cu one is legal. A map
     built without kinds carries no kind map (the off path);
  4. the fixture routes 0 of 4 with through vias and with each of the three
     single spans, and 4 of 4 with `--via-kinds` naming all three: 4 micro
     vias F.Cu-In1.Cu, 4 buried In1.Cu-In6.Cu, 4 micro In6.Cu-B.Cu, each at
     its own size; check_drc and check_connected clean, and kicad-cli (when
     installed) finds no violation and nothing unconnected;
  5. `--jog-layers In1.Cu In6.Cu`: every net still routes, every run of
     track on either layer is one jog between two vias no longer than
     --jog-max-length, and without the flag the same nets lay runs several
     times that on In1.Cu (so the flag is what moved them); same graders;
  6. a recorded command carrying the three flags converts to a plan step
     with the three params (`manifest_to_plan`).
"""
import collections
import contextlib
import io
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, os.path.join(REPO, 'rust_router'))
sys.path.insert(0, TESTS)
sys.path.insert(0, os.path.join(TESTS, 'fixtures', '1842'))
sys.path.insert(0, os.path.join(TESTS, 'stress'))
from run_utils import check, evidence  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402
import mixed_kinds  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
ROUTE = os.path.join(REPO, 'py_router', 'route.py')
STACK = ['F.Cu', 'In1.Cu', 'In2.Cu', 'In3.Cu', 'In4.Cu', 'In5.Cu', 'In6.Cu', 'B.Cu']
KINDS = ['F.Cu', 'In1.Cu', 'In1.Cu', 'In6.Cu', 'In6.Cu', 'B.Cu']
KICAD_CLI = shutil.which('kicad-cli') or next(
    (p for p in ('/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli',
                 '/usr/bin/kicad-cli') if os.path.isfile(p)), None)
FAILS = []


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def load(path):
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        return parse_kicad_pcb(path)


def search_core():
    print("1. the search core")
    import numpy as np
    from grid_router import GridObstacleMap, GridRouter

    def router():
        return GridRouter(via_cost=5000, h_weight=1.0, turn_cost=0)

    def wall(obs, layer, x0, x1):
        obs.add_blocked_cells_batch(np.array(
            [[x, y, layer] for x in range(x0, x1 + 1) for y in range(-30, 31)],
            dtype=np.int32))

    # two layers, a wall on layer 0: the path changes layer twice
    obs = GridObstacleMap(2)
    wall(obs, 0, 10, 11)
    src, dst = [(0, 0, 0)], [(20, 0, 0)]
    plain = router().route_multi(obs, src, dst, 200000)
    empty = router()
    empty.set_via_kinds([])
    through = router()
    through.set_via_kinds([(0b11, 5000, -1)])
    got = [r.route_multi(obs, src, dst, 200000) for r in (empty, through)]
    ok(plain[0] is not None and sum(a[2] != b[2] for a, b in zip(plain[0], plain[0][1:])) == 2,
       f"the wall is crossed on the other layer: {len(plain[0] or [])} points, 2 vias")
    ok(all(g[0] == plain[0] and g[1] == plain[1] for g in got),
       f"no kinds, an empty list and one through kind search alike: "
       f"{plain[1]} iterations, {[g[1] for g in got]}")

    # three layers; kind 0 joins layers 0-1, kind 1 joins layers 1-2
    obs = GridObstacleMap(3)
    obs.ensure_via_kinds(2)
    r = router()
    r.set_via_kinds([(0b011, 5000, 0), (0b110, 5000, 1)])
    path, _it, _st = r.route_multi(obs, [(0, 0, 0)], [(0, 0, 2)], 200000)
    ok(path == [(0, 0, 0), (0, 0, 1), (0, 0, 2)]
       and (r.via_kind_at(obs, 0, 0, 0, 1), r.via_kind_at(obs, 0, 0, 1, 2),
            r.via_kind_at(obs, 0, 0, 0, 2)) == (0, 1, -1),
       f"layer 0 to layer 2 is two vias, one of each kind; no kind joins 0 and 2: {path}")
    obs.add_blocked_vias_kind_batch(1, np.array([[x, y] for x in range(-2, 3)
                                                 for y in range(-2, 3)], dtype=np.int32))
    path, _it, _st = r.route_multi(obs, [(0, 0, 0)], [(0, 0, 2)], 200000)
    changes = [(a[0], a[1], a[2], b[2]) for a, b in zip(path, path[1:]) if a[2] != b[2]]
    ok(len(changes) == 2 and all(max(abs(x), abs(y)) > 2 for x, y, la, lb in changes
                                 if {la, lb} == {1, 2})
       and r.via_kind_at(obs, 0, 0, 0, 1) == 0 and r.via_kind_at(obs, 0, 0, 1, 2) == -1,
       f"with kind 1's map blocked round the origin its via moves out, and kind 0's "
       f"site there stays legal: {changes}")
    spaced = router()
    spaced.set_via_kinds([(0b011, 5000, 0), (0b110, 5000, 1)], 3)
    free = GridObstacleMap(3)
    free.ensure_via_kinds(2)
    path, _it, _st = spaced.route_multi(free, [(0, 0, 0)], [(0, 0, 2)], 200000)
    sites = [(a[0], a[1]) for a, b in zip(path, path[1:]) if a[2] != b[2]]
    ok(len(sites) == 2 and max(abs(sites[0][0] - sites[1][0]),
                               abs(sites[0][1] - sites[1][1])) >= 3,
       f"with a minimum run of 3 cells the two vias stand apart, not stacked, in "
       f"{_it} iterations: {sites}")
    bare = GridObstacleMap(3)
    ok(r.route_multi(bare, [(0, 0, 0)], [(0, 0, 2)], 20000)[0] is None
       and bare.is_via_blocked_kind(0, 0, 0),
       "on a map that carries no kind map a kind is legal nowhere")
    obs.set_kind_mirror(True)
    obs.add_blocked_vias_batch(np.array([[40, 40]], dtype=np.int32))
    mirrored = (obs.is_via_blocked_kind(40, 40, 0), obs.is_via_blocked_kind(40, 40, 1))
    obs.remove_blocked_vias_batch(np.array([[40, 40]], dtype=np.int32))
    obs.set_kind_mirror(False)
    obs.add_blocked_vias_batch(np.array([[41, 41]], dtype=np.int32))
    ok(mirrored == (True, True) and not obs.is_via_blocked_kind(40, 40, 0)
       and obs.is_via_blocked(41, 41) and not obs.is_via_blocked_kind(41, 41, 0),
       "the kind mirror repeats a through-via stamp and its removal in every kind "
       "map, and only while it is on")

    # a jog-only layer: layer 1 carries no track but a jog of at most 3 cells
    # between two vias. The source is boxed in on layer 0 by a ring of the
    # given thickness.
    for thick, want in ((2, True), (6, False)):
        obs = GridObstacleMap(2)
        obs.add_blocked_cells_batch(np.array(
            [[x, y, 0] for x in range(-12, 13) for y in range(-12, 13)
             if 5 <= max(abs(x), abs(y)) < 5 + thick], dtype=np.int32))
        free = router()
        free.set_via_kinds([(0b11, 5000, -1)], 1)
        free = free.route_multi(obs, src, dst, 200000)[0]
        capped = router()
        capped.set_via_kinds([(0b11, 5000, -1)], 1)
        capped.set_layer_jog_caps([0, 3])
        path = capped.route_multi(obs, src, dst, 200000)[0]
        runs = [max(abs(a[0] - b[0]), abs(a[1] - b[1]))
                for a, b in zip(path or [], (path or [])[1:]) if a[2] == b[2] == 1]
        ok(free is not None and (path is not None) == want
           and (not want or (len(runs) == 1 and runs[0] <= 3)),
           f"a ring {thick} cells thick: with layer 1 free the path crosses under it; "
           f"jog-only at 3 cells it "
           f"{'crosses in one jog of ' + str(runs) if path else 'finds no path'}")


def resolving(board):
    print("2. the kinds a step declares")
    from via_kinds import resolve_via_kinds
    pcb = load(board)
    kinds = resolve_via_kinds(pcb, KINDS + ['F.Cu', 'B.Cu'], 0.46, 0.2, board)
    got = [(k.via_type, k.pair, k.size, k.drill, k.map) for k in kinds]
    ok(got == [('micro', ['F.Cu', 'In1.Cu'], 0.25, 0.1, 0),
               ('buried', ['In1.Cu', 'In6.Cu'], 0.46, 0.2, 1),
               ('micro', ['In6.Cu', 'B.Cu'], 0.25, 0.1, 2),
               ('through', ['F.Cu', 'B.Cu'], 0.46, 0.2, -1)],
       f"types, sizes and maps: {got}")

    def refused(names, size=0.46, drill=0.2):
        try:
            resolve_via_kinds(pcb, names, size, drill, board)
        except ValueError as e:
            return str(e)
        return ''
    ok('layer pairs' in refused(['F.Cu', 'In1.Cu', 'B.Cu'])
       and 'given twice' in refused(['F.Cu', 'In1.Cu', 'In1.Cu', 'F.Cu'])
       and 'not a copper layer' in refused(['F.Cu', 'In9.Cu'])
       and 'larger than the step' in refused(['F.Cu', 'In1.Cu'], 0.2, 0.08)
       and resolve_via_kinds(pcb, None, 0.46, 0.2) is None,
       "an odd list, one span twice, a layer the board lacks and a kind larger than "
       "the step's via are refused; no list is no kinds")
    x = os.path.join(os.path.dirname(board), 'x.kicad_pcb')
    for extra, reason in ((['--via-kinds', 'F.Cu', 'In9.Cu'], '--via-kinds: In9.Cu is not a copper layer'),
                          (['--via-kinds', 'F.Cu'], '--via-kinds: layer names come in TOP BOTTOM pairs'),
                          (['--via-kinds', 'F.Cu', 'In1.Cu', '--via-span', 'F.Cu', 'In1.Cu'],
                           '--via-kinds: not with --via-span'),
                          (['--via-kinds', 'F.Cu', 'In1.Cu', '--jog-layers', 'In9.Cu'],
                           '--jog-layers: In9.Cu is not a copper layer'),
                          (['--jog-layers', 'In1.Cu'], '--jog-layers: needs --via-kinds')):
        check(PY + [ROUTE, board, x, '--nets', '/K*'] + extra, refuse=reason, code=2,
              allow=('error:',))
    ok(not os.path.exists(x), "route.py refuses each by name (exit 2) and writes nothing")


def legality(board):
    print("3. legality per transition")
    from routing_config import GridRouteConfig, GridCoord
    from obstacle_map import build_base_obstacle_map
    from via_kinds import resolve_via_kinds
    pcb = load(board)
    common = dict(track_width=0.1, clearance=0.1, via_size=0.46, via_drill=0.2,
                  grid_step=0.1, layers=list(STACK), hole_to_hole_clearance=0.2)
    kinds = resolve_via_kinds(pcb, KINDS, 0.46, 0.2, board)
    with contextlib.redirect_stdout(io.StringIO()):
        obs = build_base_obstacle_map(pcb, GridRouteConfig(via_kinds=kinds, **common), [])
        off = build_base_obstacle_map(pcb, GridRouteConfig(**common), [])
    coord = GridCoord(0.1)

    def at(x, y):
        gx, gy = coord.to_grid(x, y)
        return (not obs.is_via_blocked(gx, gy),) + tuple(
            not obs.is_via_blocked_kind(gx, gy, k) for k in range(3))
    ok(obs.via_kind_count() == 3 and obs.kind_mirror, "the base map carries one map "
       "per kind and its mirror is on")
    ok(at(8.0, 8.0) == (False, True, True, False),
       f"over the B.Cu pad: no through via, no B.Cu micro via, the F.Cu micro via "
       f"and the buried via fit: {at(8.0, 8.0)}")
    ok(at(6.0, 10.0) == (False, True, False, False),
       f"on the In3.Cu track: the micro via fits, the buried via does not: {at(6.0, 10.0)}")
    ok(at(4.5, 10.0)[1] is False and at(4.5, 10.0)[2] is True,
       f"on the F.Cu frame: no F.Cu micro via, the buried via fits: {at(4.5, 10.0)}")
    ok(at(22.0, 8.0) == (False, False, True, True),
       f"under the F.Cu pad on the other side: the B.Cu micro via and the buried via "
       f"fit: {at(22.0, 8.0)}")
    gx, gy = coord.to_grid(8.0, 8.0)
    ok(off.via_kind_count() == 0 and not off.kind_mirror
       and off.is_via_blocked(gx, gy) == obs.is_via_blocked(gx, gy),
       "a map built without kinds carries no kind map and no mirror")


def grade(out, nets='/K*'):
    """check_drc, check_connected and (when installed) kicad-cli on ``out``;
    a short text of what kicad-cli found, '' when clean or not installed."""
    check(PY + [os.path.join(REPO, 'py_router', 'check_drc.py'), out, '--quiet'],
          accept=True)
    check(PY + [os.path.join(REPO, 'py_router', 'check_connected.py'), out,
                '--nets', nets], accept=True)
    if not KICAD_CLI:
        return ''
    rep = out + '.kc.json'
    r = subprocess.run([KICAD_CLI, 'pcb', 'drc', '--format', 'json', '--severity-all',
                        '-o', rep, out], capture_output=True, text=True)
    if r.returncode not in (0, 5) or not os.path.isfile(rep):
        return f"kicad-cli did not run: exit {r.returncode} {r.stderr[-300:]}"
    with open(rep, encoding='utf-8') as fh:
        doc = json.load(fh)
    # the fixture's footprints are in no library; nothing else is waived
    found = [v['type'] for v in doc.get('violations', [])
             if v['type'] != 'lib_footprint_issues']
    found += ['unconnected'] * len(doc.get('unconnected_items', []))
    return ', '.join(sorted(found))


def routed_nets(summary):
    with open(evidence(summary, 'the route summary'), encoding='utf-8') as fh:
        doc = json.load(fh)
    return doc, len(doc.get('routed_single') or [])


def runs_on(pcb, layer):
    """Lengths (mm) of the connected runs of track on ``layer``, per net."""
    segs = [s for s in pcb.segments if s.layer == layer]
    parent = list(range(len(segs)))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i
    ends = collections.defaultdict(list)
    for i, s in enumerate(segs):
        for x, y in ((s.start_x, s.start_y), (s.end_x, s.end_y)):
            ends[(s.net_id, round(x, 4), round(y, 4))].append(i)
    for group in ends.values():
        for i in group[1:]:
            parent[find(i)] = find(group[0])
    total = collections.Counter()
    for i, s in enumerate(segs):
        total[find(i)] += math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)
    return sorted(total.values(), reverse=True)


def mixed(td, board):
    print("4. no one kind completes a net; the three together complete all four")
    base = ['--nets', '/K*', '--layers'] + STACK
    for label, extra in (('through vias', []),
                         ('--via-span F.Cu In1.Cu', ['--via-span', 'F.Cu', 'In1.Cu']),
                         ('--via-span In1.Cu In6.Cu', ['--via-span', 'In1.Cu', 'In6.Cu']),
                         ('--via-span In6.Cu B.Cu', ['--via-span', 'In6.Cu', 'B.Cu'])):
        tag = label.replace(' ', '_').replace('.', '').replace('-', '')
        summary = os.path.join(td, f'{tag}.json')
        check(PY + [ROUTE, board, os.path.join(td, f'{tag}.kicad_pcb')] + base + extra
              + ['--json-out', summary], accept=True, timeout=1800)
        _doc, n = routed_nets(summary)
        ok(n == 0, f"{label}: {n} of 4 nets routed")
    out = os.path.join(td, 'kinds.kicad_pcb')
    summary = os.path.join(td, 'kinds.json')
    r = check(PY + [ROUTE, board, out] + base + ['--via-kinds'] + KINDS
              + ['--json-out', summary], accept=True, timeout=1800)
    doc, n = routed_nets(summary)
    ok(n == 4 and not doc.get('failed_single') and not doc.get('open_single'),
       f"--via-kinds {' '.join(KINDS)}: {n} of 4 nets routed")
    ok('Via kinds (#1842): the search changes layers through micro F.Cu-In1.Cu 0.25/0.1, '
       'buried In1.Cu-In6.Cu 0.46/0.2, micro In6.Cu-B.Cu 0.25/0.1' in r.stdout,
       "the run names the kinds and their sizes")
    pcb = load(evidence(out, 'the routed board'))
    census = collections.Counter((v.via_type, tuple(v.layers), v.size, v.drill)
                                 for v in pcb.vias)
    ok(census == {('micro', ('F.Cu', 'In1.Cu'), 0.25, 0.1): 4,
                  ('buried', ('In1.Cu', 'In6.Cu'), 0.46, 0.2): 4,
                  ('micro', ('In6.Cu', 'B.Cu'), 0.25, 0.1): 4},
       f"four vias of each kind, each at its own size: {dict(census)}")
    told = {tuple(k['layers']): k['on_board'] for k in (doc.get('via_kinds') or {}).get('kinds', [])}
    ok(told == {('F.Cu', 'In1.Cu'): 4, ('In1.Cu', 'In6.Cu'): 4, ('In6.Cu', 'B.Cu'): 4},
       f"and the summary counts them: {told}")
    kc = grade(out)
    ok(kc == '', "check_drc and check_connected are clean"
       + (f", and kicad-cli finds nothing{': ' + kc if kc else ''}" if KICAD_CLI
          else " (kicad-cli not installed: not run)"))
    return max(runs_on(pcb, 'In1.Cu') + [0.0])


def jogs(td, board, longest_free):
    print("5. --jog-layers In1.Cu In6.Cu")
    out = os.path.join(td, 'jog.kicad_pcb')
    summary = os.path.join(td, 'jog.json')
    check(PY + [ROUTE, board, out, '--nets', '/K*', '--layers'] + STACK
          + ['--via-kinds'] + KINDS + ['--jog-layers', 'In1.Cu', 'In6.Cu',
                                       '--json-out', summary], accept=True, timeout=1800)
    doc, n = routed_nets(summary)
    ok(n == 4 and not doc.get('failed_single') and not doc.get('open_single'),
       f"{n} of 4 nets routed")
    pcb = load(evidence(out, 'the routed board'))
    runs = {layer: runs_on(pcb, layer) for layer in ('In1.Cu', 'In6.Cu')}
    longest = max(runs['In1.Cu'] + runs['In6.Cu'] + [0.0])
    ok(runs['In1.Cu'] and runs['In6.Cu'] and longest <= 2.0 + 0.02,
       f"{len(runs['In1.Cu'])} + {len(runs['In6.Cu'])} runs of track on the two layers, "
       f"the longest {longest:.3f} mm (cap 2.0)")
    ok(longest_free > 3 * 2.0,
       f"without the flag the longest run on In1.Cu was {longest_free:.2f} mm")
    ok((doc.get('via_kinds') or {}).get('jog_caps') == {'In1.Cu': 2.0, 'In6.Cu': 2.0},
       f"the summary carries the caps: {(doc.get('via_kinds') or {}).get('jog_caps')}")
    kc = grade(out)
    ok(kc == '', "check_drc and check_connected are clean"
       + (f", and kicad-cli finds nothing{': ' + kc if kc else ''}" if KICAD_CLI
          else " (kicad-cli not installed: not run)"))


def plan():
    print("6. a recorded command converts to a plan step")
    import manifest_to_plan as m2p
    cmd = ("python3 py_router/route.py in.kicad_pcb out.kicad_pcb --nets '/K*' "
           "--via-kinds F.Cu In1.Cu In1.Cu In6.Cu --jog-layers In1.Cu --jog-max-length 1.5")
    import shlex
    step = m2p.parse_command(shlex.split(cmd)) or {}
    params = step.get('params', {})
    ok(params.get('via_kinds') == ['F.Cu', 'In1.Cu', 'In1.Cu', 'In6.Cu']
       and params.get('jog_layers') == ['In1.Cu']
       and float(params.get('jog_max_length', 0)) == 1.5,
       f"via_kinds, jog_layers and jog_max_length survive: "
       f"{ {k: params.get(k) for k in ('via_kinds', 'jog_layers', 'jog_max_length')} }")


def main():
    search_core()
    with tempfile.TemporaryDirectory() as td:
        board = mixed_kinds.write(os.path.join(td, 'in.kicad_pcb'))
        evidence(board, 'the fixture')
        resolving(board)
        legality(board)
        longest_free = mixed(td, board)
        jogs(td, board, longest_free)
    plan()
    if FAILS:
        print(f"\nFAILED: {len(FAILS)}")
        return 1
    print("\nALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
