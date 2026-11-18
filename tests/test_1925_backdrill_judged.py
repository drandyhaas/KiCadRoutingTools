#!/usr/bin/env python3
"""#1925: every back-drill a route step ships has been judged, at KiCad's price.

Found in review of the step that defines back-drills:

  1. a net routed AGAIN lays its vias beside the ones it had; the final
     check took those for the input's own and skipped them, so the marks the
     route gave them shipped unjudged and the record said 0 of 0. A via is
     the input's own by its uuid on the written board, or by standing on an
     input via's spot with the input's own drills; any other is judged;
  2. the wall is priced at the wider of the two nets' classes, the
     back-drilled net's own included, as KiCad grades the disc (kicad-cli is
     asked on the same board when one is installed);
  3. the via geometry a site is tested at never reaches less far than the
     wall: a clearance above the copper-to-hole floor subtracts nothing;
  4. when the attempt at the drill's legal sites and the plain attempt both
     fail, the main edge keeps the one that connected more, the first on a
     tie, as the tap edges do;
  5. `board_has_backdrills` does not keep its answer across a rip plus an
     add, or a mark made in place.

Run with:  python3 tests/test_1925_backdrill_judged.py
"""
import contextlib
import io
import json
import os
import re
import subprocess
import sys
import tempfile
from dataclasses import replace

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, os.path.join(REPO, 'rust_router'))
sys.path.insert(0, os.path.join(REPO, 'py_tools'))

import run_utils  # noqa: E402
from backdrill_fixture import backdrill_token, write_board  # noqa: E402

FAILS = []
ROUTE = os.path.join(REPO, 'py_router', 'route.py')
RUN = ['--clearance', '0.1', '--track-width', '0.2', '--via-size', '0.6',
       '--via-drill', '0.3', '--grid-step', '0.05']
ASK = ['--backdrill-nets', '/S', '--backdrill-oversize', '0.3',
       '--backdrill-margin', '0.2', '--backdrill-min-stub', '0.25']
#: /S: two F.Cu pads with a wall of another net between them on F.Cu, so the
#: route changes to In1.Cu and back: two vias, In2.Cu and B.Cu their stubs
S_PADS = [('P1', '/S', 3, 5, 'F.Cu', 1.0), ('P2', '/S', 17, 5, 'F.Cu', 1.0)]
WALL = [('/W', 10, 0, 10, 10, 'F.Cu', 0.3)]
DRILL = {'backdrill': (0.6, 'B.Cu', 'In2.Cu')}


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def parse(path):
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        return parse_kicad_pcb(run_utils.evidence(path, 'a board to read'))


def route(src, out, *args):
    r = run_utils.check([sys.executable, '-X', 'utf8', ROUTE, src, out, '--nets', '/S',
                         '--layers', 'F.Cu', 'In1.Cu', *RUN, *args], accept=True)
    log = (r.stdout or '') + (r.stderr or '')
    run_utils.evidence(out, 'the routed board')
    m = re.findall(r'^BACKDRILL_JSON: (.*)$', log, re.M)
    return log, (json.loads(m[-1]) if m else None)


def net_id(pcb, name):
    return next(n for n, net in pcb.nets.items() if net.name == name)


def test_rerouted_vias_are_judged():
    print("1: a via laid again beside an input via is judged")
    import backdrill
    import math
    with tempfile.TemporaryDirectory() as td:
        src = write_board(os.path.join(td, 'a.kicad_pcb'), pads=S_PADS, tracks=WALL)
        first = os.path.join(td, 'first.kicad_pcb')
        route(src, first)
        before = parse(first)
        ok(len(before.vias) == 2 and not any(v.backdrills for v in before.vias),
           "routed without the option: two vias, no back-drill")
        again = os.path.join(td, 'again.kicad_pcb')
        _log, rec = route(first, again, '--force-reroute', *ASK)
        after = parse(again)
        near = [min(math.hypot(v.x - o.x, v.y - o.y) for o in before.vias)
                for v in after.vias]
        ok(len(after.vias) == 2 and all(0 < d < 0.3 for d in near)
           and not {v.uuid for v in after.vias} & {v.uuid for v in before.vias},
           f"routed again with the option: two new vias, {max(near):.2f} mm at most "
           f"from the old ones (inside half a via, where the old test read them "
           f"as the input's)")
        ok(rec is not None and (rec['vias'], rec['drilled']) == (2, 2),
           f"the final check judged both: {rec and (rec['vias'], rec['drilled'])} "
           f"(it judged 0 of 0)")
        ok(all({t: tuple(bd) for t, bd in v.backdrills.items()} == DRILL
               for v in after.vias), "and they ship with the drill it decided")

        # the rule itself, on the written board and on a GUI run's results
        snap = backdrill.input_snapshot(before.vias)
        by_net = {}
        for ent in snap:
            by_net.setdefault(ent[0], []).append(ent[1:])
        uuids = {ent[5] for ent in snap}
        old = before.vias[0]
        ok(len(uuids) == 2 and backdrill._inputs_own(old, by_net, uuids, False),
           "a via carrying an input via's uuid is the input's own")
        moved = replace(old, x=old.x + 0.1, uuid='new', backdrills=dict(DRILL))
        ok(not backdrill._inputs_own(moved, by_net, uuids, False),
           "a new via 0.1 mm from it, carrying a mark, is not")
        ok(not backdrill._inputs_own(replace(old, uuid='new'), by_net, uuids, False),
           "nor a new via on its very spot with no mark: it is judged afresh")
        ok(backdrill._inputs_own(replace(old, uuid=''), by_net, uuids, False),
           "with no uuid to read, the same spot and the same drills is the input's")
        ok(not backdrill._inputs_own(replace(old, uuid=''), by_net, uuids, True),
           "a GUI run's result via on an input spot is the run's, and is judged")
        drilled = backdrill.input_snapshot([replace(old, backdrills=dict(DRILL))])
        by_net = {drilled[0][0]: [drilled[0][1:]]}
        ok(backdrill._inputs_own(replace(old, uuid='new', backdrills=dict(DRILL)),
                                 by_net, set(), True)
           and not backdrill._inputs_own(replace(old, uuid='new', backdrills={}),
                                         by_net, set(), True),
           "a via laid again with the input's own drills keeps them; with other "
           "drills it is judged")


def _class_board(path, gap, wide):
    """/S's 0.6 / 0.3 via at (10, 5), back-drilled 0.9 mm (wider than its pad)
    on In2.Cu and B.Cu; a 0.2 mm track of /W on B.Cu `gap` mm from the wall.
    /S is in a class of clearance 0.3 when `wide`; /W is Default, 0.1."""
    y = round(5 + 0.45 + gap + 0.1, 4)
    write_board(path, tracks=[('/S', 10, 5, 10, 2, 'F.Cu', 0.2),
                              ('/S', 10, 5, 13, 5, 'In1.Cu', 0.2),
                              ('/W', 5, y, 15, y, 'B.Cu', 0.2)],
                vias=[('/S', 10, 5, 0.6, 0.3, backdrill_token(0.9, 'B.Cu', 'In2.Cu'))])
    with open(os.path.join(REPO, 'kicad_files', 'flat_hierarchy.kicad_pro'),
              encoding='utf-8') as fh:
        pro = json.load(fh)
    pro['board']['design_settings']['rules'].update(
        min_clearance=0.0, min_hole_clearance=0.2, min_hole_to_hole=0.25,
        min_via_annular_width=0.02, min_via_diameter=0.2, min_through_hole_diameter=0.1,
        min_track_width=0.1, min_copper_edge_clearance=0.1)
    classes = pro['net_settings']['classes']
    del classes[1:]
    classes[0].update(clearance=0.1)
    pro['net_settings']['netclass_patterns'] = []
    if wide:
        classes.append(dict(classes[0], name='Wide', clearance=0.3))
        pro['net_settings']['netclass_patterns'] = [{'netclass': 'Wide', 'pattern': '/S'}]
    pro['net_settings']['netclass_assignments'] = None
    with open(os.path.splitext(path)[0] + '.kicad_pro', 'w', encoding='utf-8') as fh:
        json.dump(pro, fh)
    return run_utils.evidence(path, 'the class board')


def test_wall_priced_by_both_classes():
    print("2: the wall is priced at the wider of the two nets' classes")
    import backdrill
    from check_drc import run_drc
    from list_nets import net_clearance_map
    from routing_config import GridRouteConfig
    cli = None
    try:
        from kicad_oracle import find_kicad_cli
        cli = find_kicad_cli(warn=False)
    except Exception:                                          # noqa: BLE001
        pass
    #: (gap, /S in the 0.3 class, the wall is too close: kicad-cli 10.0.8's answer)
    for gap, wide, want in ((0.25, True, True), (0.32, True, False), (0.25, False, False)):
        with tempfile.TemporaryDirectory() as td:
            board = _class_board(os.path.join(td, 'c.kicad_pcb'), gap, wide)
            pcb = parse(board)
            cfg = GridRouteConfig(clearance=0.1, via_size=0.6, via_drill=0.3,
                                  hole_to_hole_clearance=0.25,
                                  layers=list(pcb.board_info.copper_layers))
            cfg.net_clearances = {net_id(pcb, '/S'): 0.3} if wide else {}
            cfg.backdrill = backdrill.make_spec(pcb, ['/S'], 0.6, 0.2, 0.25)
            backdrill.install(cfg, pcb)
            pcb.vias[0].backdrills = {}
            hits = backdrill.wall_conflicts(backdrill._Index(pcb, cfg.layers), pcb.vias[0],
                                            ['In2.Cu', 'B.Cu'], 0.9, cfg)
            what = f"/S in a {'0.3' if wide else '0.1'} class, /W 0.1, {gap} mm from the wall"
            ok(bool(hits) == want and (not want or (hits[0][1], hits[0][4]) == ('track', 0.3)),
               f"{what}: the judge {'refuses' if hits else 'keeps'} the drill"
               + (f" (needs {hits[0][4]})" if hits else ''))
            with contextlib.redirect_stdout(io.StringIO()):
                rows = [r for r in run_drc(
                            board, clearance=0.1, quiet=True, print_summary=False,
                            net_clearances=net_clearance_map(
                                board, [n.name for n in pcb.nets.values()]) or None)
                        if r.get('type') == 'via-segment']
            ok(bool(rows) == want, f"{what}: check_drc reports {len(rows)}")
            if not cli:
                continue
            rep = os.path.join(td, 'k.json')
            subprocess.run([cli, 'pcb', 'drc', '--format', 'json', '--severity-all',
                            '--all-track-errors', '-o', rep, board],
                           capture_output=True, text=True, timeout=300)
            with open(run_utils.evidence(rep, "kicad-cli's report"), encoding='utf-8') as fh:
                doc = json.load(fh)
            kicad = [v for v in doc.get('violations', []) if v.get('type') == 'clearance']
            ok(bool(kicad) == want, f"{what}: kicad-cli {doc.get('kicad_version')} "
               f"reports {len(kicad)} clearance violation(s)")
    if not cli:
        print("  [skip] no kicad-cli here: KiCad itself is not asked (the answers "
              "above were measured on 10.0.8)")


def test_legality_reach():
    print("3: the legality via reaches at least as far as the wall")
    import backdrill
    from routing_config import GridRouteConfig
    with tempfile.TemporaryDirectory() as td:
        pcb = parse(write_board(os.path.join(td, 'l.kicad_pcb'), pads=S_PADS))

    def at(clearance):
        cfg = GridRouteConfig(clearance=clearance, via_size=0.6, via_drill=0.3,
                              hole_to_hole_clearance=0.25,
                              layers=list(pcb.board_info.copper_layers))
        cfg.backdrill = backdrill.make_spec(pcb, ['/S'], 0.4, 0.2, 0.25)
        backdrill.install(cfg, pcb)
        return backdrill.wall_floor(cfg), backdrill.legality_via(cfg, 0.6, 0.3)

    floor, got = at(0.25)
    ok(floor == 0.2 and got == (0.7, 0.7),
       f"clearance 0.25 over a {floor} floor, a 0.7 mm wall: tested at {got} "
       f"(it was 0.6, inside the wall)")
    floor, got = at(0.1)
    ok(got == (0.9, 0.7),
       f"clearance 0.1 under the floor: the wall and twice the difference, {got}")


def test_main_keeps_the_attempt_that_connected_more():
    print("4: both attempts failing, the main edge keeps the one that connected more")
    import backdrill
    import single_ended_routing as ser
    from routing_config import GridRouteConfig
    ok(ser._connects_more({'failed': True}, None)
       and ser._connects_more({'tap_edges_failed': 1}, {'failed': True})
       and ser._connects_more({'tap_edges_failed': 0}, {'tap_edges_failed': 2})
       and not ser._connects_more({'failed': True, 'iterations': 9}, {'failed': True}),
       "a result over none, a routed one over a failed one, fewer failed taps; "
       "two failures tie")
    with tempfile.TemporaryDirectory() as td:
        pcb = parse(write_board(os.path.join(td, 'm.kicad_pcb'), pads=S_PADS))
    cfg = GridRouteConfig(clearance=0.1, via_size=0.6, via_drill=0.3,
                          layers=list(pcb.board_info.copper_layers))
    cfg.backdrill = backdrill.make_spec(pcb, ['/S'], 0.3, 0.2, 0.25)
    nid = net_id(pcb, '/S')
    calls = []

    def run(answers):
        calls.clear()

        def impl(_pcb, _net, config, _obstacles, *a, **k):
            calls.append('rung' if config.backdrill.search else 'plain')
            return answers[len(calls) - 1]
        keep = ser._route_multipoint_main_impl, ser._backdrill_search_config
        ser._route_multipoint_main_impl = impl
        ser._backdrill_search_config = lambda c, _p, _n: replace(c, via_rung=1)
        try:
            with contextlib.redirect_stdout(io.StringIO()):
                return ser.route_multipoint_main(pcb, nid, cfg, None, [])
        finally:
            ser._route_multipoint_main_impl, ser._backdrill_search_config = keep

    first, second = {'failed': True, 'who': 'rung'}, {'failed': True, 'who': 'plain'}
    got = run([first, second])
    ok(calls == ['rung', 'plain'] and got is first,
       f"both fail alike: the attempt at the drill's legal sites is kept "
       f"(got {got and got.get('who')})")
    got = run([None, second])
    ok(got is second, "the first gave nothing: the plain attempt's failure is kept")
    routed = {'who': 'plain', 'new_segments': [1]}
    ok(run([first, routed]) is routed and run([routed, first]) is routed
       and calls == ['rung'],
       "a routed attempt is kept, and a first success is not tried again")


def test_has_backdrills_is_not_stale():
    print("5: board_has_backdrills follows a rip plus an add, and a mark in place")
    import backdrill
    with tempfile.TemporaryDirectory() as td:
        pcb = parse(write_board(os.path.join(td, 'h.kicad_pcb'),
                                vias=[('/S', 5, 5, 0.6, 0.3, ''), ('/S', 8, 5, 0.6, 0.3, '')]))
    ok(backdrill.board_has_backdrills(pcb) is False, "two plain vias: no")
    ripped = pcb.vias.pop(0)
    pcb.vias.append(replace(ripped, x=12.0, backdrills=dict(DRILL)))
    ok(len(pcb.vias) == 2 and backdrill.board_has_backdrills(pcb) is True,
       "one ripped and a back-drilled one added, the count unchanged: yes")
    pcb.vias[-1] = replace(pcb.vias[-1], backdrills={})
    ok(backdrill.board_has_backdrills(pcb) is False, "and no again once it is gone")
    cfg_pcb_vias = list(pcb.vias)
    from routing_config import GridRouteConfig
    cfg = GridRouteConfig(clearance=0.1, via_size=0.6, via_drill=0.3,
                          layers=list(pcb.board_info.copper_layers))
    cfg.backdrill = backdrill.make_spec(pcb, ['/S'], 0.3, 0.2, 0.25)
    backdrill.install(cfg, pcb)
    from kicad_parser import Segment
    nid = net_id(pcb, '/S')
    v = cfg_pcb_vias[0]
    joins = [Segment(start_x=v.x, start_y=v.y, end_x=v.x + 2, end_y=v.y, width=0.2,
                     layer=layer, net_id=nid) for layer in ('F.Cu', 'In1.Cu')]
    marked = backdrill.mark_new_vias(pcb, nid, cfg, [v], joins)
    ok(marked == 1 and bool(v.backdrills) and backdrill.board_has_backdrills(pcb) is True,
       "a via of the board marked in place, the list untouched: yes")


def main():
    for fn in (test_rerouted_vias_are_judged, test_wall_priced_by_both_classes,
               test_legality_reach, test_main_keeps_the_attempt_that_connected_more,
               test_has_backdrills_is_not_stale):
        fn()
    print()
    if FAILS:
        print(f"FAILED: {len(FAILS)} check(s)")
        for f in FAILS:
            print(f"  - {f}")
        return 1
    print("PASSED")
    return 0


if __name__ == '__main__':
    sys.exit(main())
