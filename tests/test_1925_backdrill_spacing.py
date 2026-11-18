#!/usr/bin/env python3
"""#1925 piece 3: route to a back-drill's spacing; define back-drills when asked.

A back-drill is a wider hole with no copper on the layers it removes, so
other nets' copper has to keep off its wall there. Nothing held it off, and
nothing could ask a route step to back-drill the vias it adds.

  1. `plan_spans`: which layers a via's back-drill removes, from the layers
     its net joins it on, the stackup, the margin and the shortest stub;
  2. the option is all or nothing: nets without the fabricator's three
     numbers are refused by name, on the CLI and in the engine;
  3. a board that DECLARES a back-drill: a track of another net is routed
     off the wall on a layer the drill removes, and exactly as before on a
     layer it leaves and on the same board without the token;
  4. a new via is held off the wall, and its drill off the wider hole;
  5. `--backdrill-nets`: the vias a route adds are back-drilled from the far
     face up to the last stub layer, and the same route without the option
     writes no token;
  6. the search takes a via site only where the drill is legal: with copper
     of another net under the direct sites it places the vias elsewhere, and
     the sites the plain route took are refused by the judge;
  6c. the wall is priced by the pair's clearance as the board's rules give
     it: the other net's class, a `.kicad_dru` layer rule replacing it, and
     the board's copper-to-hole floor under all of them;
  7. completion first: with no legal site anywhere the net is still routed,
     its vias are left undrilled and the summary says why;
  8. `check_drc` grades a back-drilled via as KiCad 10.0.8 does: on a layer
     the drill removes, a disc of the back-drill's diameter at the copper
     clearance (and kicad-cli itself is asked, when one is installed);
  9. `set_via_backdrills_in_content` rewrites only the vias named;
 10. `manifest_to_plan` carries the four flags into a GUI plan step.

Run with:  python3 tests/test_1925_backdrill_spacing.py
"""
import contextlib
import io
import json
import os
import re
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, os.path.join(REPO, 'rust_router'))
sys.path.insert(0, os.path.join(REPO, 'py_tools'))

import run_utils  # noqa: E402
from backdrill_fixture import backdrill_token, board_text, write_board  # noqa: E402

FAILS = []
ROUTE = os.path.join(REPO, 'py_router', 'route.py')
#: the run every routed row shares: via 0.6 / 0.3, clearance 0.1, track 0.2
RUN = ['--clearance', '0.1', '--track-width', '0.2', '--via-size', '0.6',
       '--via-drill', '0.3', '--grid-step', '0.05']
ASK = ['--backdrill-nets', '/S', '--backdrill-oversize', '0.3',
       '--backdrill-margin', '0.2', '--backdrill-min-stub', '0.25']
#: /S: two F.Cu pads with a wall of another net between them on F.Cu, so the
#: route has to change to In1.Cu and back: two vias, each with In2.Cu and B.Cu
#: as its stub
S_PADS = [('P1', '/S', 3, 5, 'F.Cu', 1.0), ('P2', '/S', 17, 5, 'F.Cu', 1.0)]
#: the same with a third pad: a multipoint net, routed as a main edge and a tap
S3_PADS = S_PADS + [('P3', '/S', 17, 8, 'F.Cu', 1.0)]
WALL = [('/W', 10, 0, 10, 10, 'F.Cu', 0.3)]


def lanes(x1, x2):
    """Tracks of /X on B.Cu, 0.9 mm apart edge to edge: a 0.6 mm via fits
    between two of them at clearance 0.1, a 0.6 mm back-drill (0.2 mm off
    each) does not."""
    return [('/X', x1, round(0.55 + 1.1 * k, 2), x2, round(0.55 + 1.1 * k, 2), 'B.Cu', 0.2)
            for k in range(9)]


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def parse(path):
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        return parse_kicad_pcb(run_utils.evidence(path, 'a board to read'))


def route(src, out, *args):
    """Route with route.py; (log, the BACKDRILL_JSON record or None)."""
    r = run_utils.check([sys.executable, '-X', 'utf8', ROUTE, src, out] + list(args),
                        accept=True)
    log = (r.stdout or '') + (r.stderr or '')
    run_utils.evidence(out, 'the routed board')
    m = re.findall(r'^BACKDRILL_JSON: (.*)$', log, re.M)
    return log, (json.loads(m[-1]) if m else None)


def net_id(pcb, name):
    return next(n for n, net in pcb.nets.items() if net.name == name)


def drills(pcb, name):
    nid = net_id(pcb, name)
    return sorted(tuple(sorted((t, tuple(bd)) for t, bd in v.backdrills.items()))
                  for v in pcb.vias if v.net_id == nid)


def step_config(pcb, spec=None, clearance=0.1):
    from routing_config import GridRouteConfig
    import backdrill
    cfg = GridRouteConfig(clearance=clearance, via_size=0.6, via_drill=0.3,
                          hole_to_hole_clearance=0.25,
                          layers=list(pcb.board_info.copper_layers))
    cfg.backdrill = spec
    backdrill.install(cfg, pcb)
    return cfg


def test_plan_spans():
    print("1: the layers a back-drill removes")
    from backdrill import BackdrillSpec, plan_spans
    from kicad_parser import Via
    from via_stubs import copper_depths
    with tempfile.TemporaryDirectory() as td:
        pcb = parse(write_board(os.path.join(td, 'b.kicad_pcb')))
        bare = parse(write_board(os.path.join(td, 'n.kicad_pcb'), stackup=False))
    copper = list(pcb.board_info.copper_layers)
    depths = copper_depths(pcb)
    ok(copper == ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu'] and len(depths) == 4,
       f"the fixture is four layers with a stackup: {copper}")
    via = Via(x=1, y=1, size=0.6, drill=0.3, layers=['F.Cu', 'B.Cu'], net_id=1)

    def plan(joined, margin, min_stub, d=depths):
        tokens, why = plan_spans(via, joined, copper, d,
                                 BackdrillSpec(frozenset({1}), 0.2, margin, min_stub))
        return {t: tuple(bd) for t, bd in tokens.items()}, why
    ok(plan(['F.Cu', 'In1.Cu'], 0.2, 0.25) == ({'backdrill': (0.5, 'B.Cu', 'In2.Cu')}, {}),
       f"joined on F.Cu and In1.Cu: drilled from B.Cu up to In2.Cu at drill + oversize: "
       f"{plan(['F.Cu', 'In1.Cu'], 0.2, 0.25)}")
    ok(plan(['F.Cu', 'In1.Cu'], 1.1, 0.25) == ({'backdrill': (0.5, 'B.Cu', 'B.Cu')}, {}),
       "a margin wider than the 1.0 mm core leaves In2.Cu and removes B.Cu only")
    ok(plan(['F.Cu', 'In1.Cu'], 1.3, 0.25) == ({}, {'bottom': 'margin'}),
       "a margin no stub layer clears drills nothing, and says so")
    ok(plan(['F.Cu', 'In1.Cu'], 0.2, 2.0) == ({}, {'bottom': 'short'}),
       "a 1.27 mm stub under a 2.0 mm minimum is left")
    both = plan(['In1.Cu', 'In2.Cu'], 0.2, 0.2)
    ok(both == ({'backdrill': (0.5, 'F.Cu', 'F.Cu'),
                 'tertiary_drill': (0.5, 'B.Cu', 'B.Cu')}, {}),
       f"joined on the two inner layers: one drill from each face: {both}")
    ok(plan(['F.Cu', 'B.Cu'], 0.0, 0.0) == ({}, {}),
       "a via joined on both outer layers has no stub")
    ok(plan([], 0.0, 0.0)[0] == {} and plan([], 0.0, 0.0)[1]['bottom'] == 'unjoined',
       "a via its net joins on no layer is not drilled")
    ok(plan(['F.Cu', 'In1.Cu'], 0.2, 0.25, {}) == ({}, {'bottom': 'no stackup'}),
       "no stackup: a margin or a shortest stub in mm cannot be measured, nothing is drilled")
    ok(plan(['F.Cu', 'In1.Cu'], 0.0, 0.0, {}) == ({'backdrill': (0.5, 'B.Cu', 'In2.Cu')}, {}),
       "no stackup and both numbers 0: every stub layer is removed")
    ok(not copper_depths(bare), "(the board without a stackup really has no depths)")


def test_option_is_whole():
    print("2: nets without the fabricator's numbers are refused by name")
    from backdrill import make_spec
    with tempfile.TemporaryDirectory() as td:
        src = write_board(os.path.join(td, 'a.kicad_pcb'), pads=S_PADS, tracks=WALL)
        pcb = parse(src)
        try:
            make_spec(pcb, ['/S'], 0.2, None, None)
            said = ''
        except ValueError as e:
            said = str(e)
        ok('--backdrill-margin' in said and '--backdrill-min-stub' in said
           and '--backdrill-oversize' not in said.split('needs')[1],
           f"the engine names the two missing numbers: {said!r}")
        ok(make_spec(pcb, None, None, None, None) is None
           and make_spec(pcb, [], 0.2, 0.2, 0.2) is None,
           "no nets asked for is no option at all")
        spec = make_spec(pcb, ['/*', '!/W'], 0.2, 0.0, 0.0)
        names = sorted(pcb.nets[n].name for n in spec.net_ids)
        ok(names == ['/S'], f"patterns select nets and !pattern excludes: {names}")
        run_utils.check([sys.executable, ROUTE, src, os.path.join(td, 'o.kicad_pcb'),
                         '--nets', '/S', '--backdrill-nets', '/S',
                         '--backdrill-oversize', '0.2'],
                        refuse='--backdrill-nets needs --backdrill-margin, '
                               '--backdrill-min-stub', code=2)
        ok(not os.path.exists(os.path.join(td, 'o.kicad_pcb')),
           "route.py refuses before it routes, naming the missing flags")


def _track_gap(pcb, name, layer, x, y):
    """Smallest centre distance from (x, y) to `name`'s tracks on `layer`."""
    from geometry_utils import point_to_segment_distance as dist
    nid = net_id(pcb, name)
    return min(dist(x, y, s.start_x, s.start_y, s.end_x, s.end_y)
               for s in pcb.segments if s.net_id == nid and s.layer == layer)


def test_declared_backdrill_is_an_obstacle():
    print("3: a declared back-drill holds another net's track off its wall")
    # /S has a via at (10, 5), back-drilled 0.6 mm from B.Cu to In2.Cu. /T's
    # pads face each other 0.55 mm below it: a straight track clears the
    # 0.6 mm pad at clearance 0.1 (0.3 + 0.1 + 0.1 = 0.5) and not the wall
    # (0.3 + 0.2 + 0.1 = 0.6).
    own = [('/S', 10, 5, 10, 2, 'F.Cu', 0.2), ('/S', 10, 5, 13, 5, 'In1.Cu', 0.2)]
    token = backdrill_token(0.6, 'B.Cu', 'In2.Cu')
    with tempfile.TemporaryDirectory() as td:
        got = {}
        for tag, layer, bd in (('drilled', 'B.Cu', token), ('plain', 'B.Cu', ''),
                               ('kept', 'F.Cu', token)):
            src = write_board(os.path.join(td, tag + '.kicad_pcb'),
                              pads=[('P3', '/T', 5, 5.55, layer, 0.6),
                                    ('P4', '/T', 15, 5.55, layer, 0.6)],
                              tracks=own, vias=[('/S', 10, 5, 0.6, 0.3, bd)])
            out = os.path.join(td, tag + '_out.kicad_pcb')
            route(src, out, '--nets', '/T', '--layers', layer, '--clearance', '0.1',
                  '--track-width', '0.2', '--grid-step', '0.05')
            pcb = parse(out)
            got[tag] = _track_gap(pcb, '/T', layer, 10, 5)
            if tag == 'drilled':
                ok(drills(pcb, '/S') == [(('backdrill', (0.6, 'B.Cu', 'In2.Cu')),)],
                   "the routed board still declares the back-drill")
                from check_drc import run_drc
                with contextlib.redirect_stdout(io.StringIO()):
                    v = run_drc(out, clearance=0.1, quiet=True, print_summary=False)
                ok(not [r for r in v if r.get('type') in ('via-segment', 'via-via')],
                   "and check_drc finds no copper too close to the via")
        ok(got['drilled'] >= 0.6 - 1e-6,
           f"on B.Cu, which the drill removes, the track stays {got['drilled']:.3f} mm "
           f"from the via's centre: wall 0.3 + 0.2 + half the track")
        ok(abs(got['plain'] - 0.55) < 1e-6,
           f"without the token the same route runs straight past at {got['plain']:.3f} mm "
           f"(the control: the detour is the back-drill's doing)")
        ok(abs(got['kept'] - 0.55) < 1e-6,
           f"and on F.Cu, which the drill leaves, it runs straight too: {got['kept']:.3f} mm")


def test_new_via_off_the_wall():
    print("4: a new via is held off the wall, and its drill off the wider hole")
    import backdrill
    from kicad_parser import Backdrill, Via
    with tempfile.TemporaryDirectory() as td:
        pcb = parse(write_board(os.path.join(td, 'b.kicad_pcb'), vias=[
            ('/S', 5, 5, 0.6, 0.3, backdrill_token(0.9, 'B.Cu', 'In2.Cu')),
            ('/W', 9, 5, 0.6, 0.3, '')]))
    drilled, plain = sorted(pcb.vias, key=lambda v: v.x)
    cfg = step_config(pcb)
    ok(backdrill.wall_floor(cfg) == 0.2,
       "a board declaring no copper-to-hole floor holds a wall at the NPTH-to-track 0.2 mm")
    extra = cfg.via_hole_extra(0.1, drilled)
    # wall 0.45 + 0.2 + new via 0.3 = 0.95, against ring to ring 0.3 + 0.3 + 0.1
    ok(abs(extra - 0.25) < 1e-9, f"a new 0.6 via stands 0.95 mm from a 0.9 mm back-drill, "
       f"0.25 past the ring-to-ring 0.7: {extra:.3f}")
    ok(cfg.via_hole_extra(0.1, plain) == 0.0, "nothing more off a via with no back-drill")
    ok(backdrill.hole_diameter(drilled) == 0.9 and backdrill.hole_diameter(plain) == 0.3,
       "the hole-to-hole keep-out reads the back-drill where it is the wider hole")
    ok(backdrill.track_reach(cfg, plain, 'B.Cu', 0.1, 0.2) is None
       and backdrill.track_reach(cfg, drilled, 'F.Cu', 0.1, 0.2) is None
       and abs(backdrill.track_reach(cfg, drilled, 'In2.Cu', 0.1, 0.2) - 0.75) < 1e-9,
       "a track's reach is the wall's on a removed layer (0.45 + 0.2 + 0.1) and none elsewhere")
    small = Via(x=0, y=0, size=0.6, drill=0.3, layers=['F.Cu', 'B.Cu'], net_id=1)
    small.backdrills = {'backdrill': Backdrill(0.4, 'B.Cu', 'B.Cu')}
    ok(cfg.via_hole_extra(0.1, small) == 0.0,
       "a back-drill narrower than the pad asks no more than the pad already does")


def test_define_when_asked():
    print("5: --backdrill-nets back-drills the vias the route adds")
    with tempfile.TemporaryDirectory() as td:
        src = write_board(os.path.join(td, 'a.kicad_pcb'), pads=S_PADS, tracks=WALL)
        out = os.path.join(td, 'on.kicad_pcb')
        js = os.path.join(td, 'on.json')
        log, rec = route(src, out, '--nets', '/S', '--layers', 'F.Cu', 'In1.Cu',
                         *RUN, *ASK, '--json-out', js)
        pcb = parse(out)
        want = (('backdrill', (0.6, 'B.Cu', 'In2.Cu')),)
        ok(drills(pcb, '/S') == [want, want],
           f"both vias are drilled 0.6 mm from B.Cu up to In2.Cu: {drills(pcb, '/S')}")
        ok(rec is not None and (rec['vias'], rec['drilled'], rec['conflict']) == (2, 2, 0),
           f"the step's record counts them: {rec}")
        ok('Back-drills, not covered:' in log and 'diff-pair step' in log
           and len(rec.get('not_covered') or []) == 3,
           "the log and the record say what the step does not cover")
        with open(run_utils.evidence(js, 'the --json-out file'), encoding='utf-8') as fh:
            doc = json.load(fh)
        ok((doc.get('backdrill') or {}).get('drilled') == 2,
           "and it is in the --json-out document")
        off = os.path.join(td, 'off.kicad_pcb')
        log_off, rec_off = route(src, off, '--nets', '/S', '--layers', 'F.Cu', 'In1.Cu', *RUN)
        with open(off, encoding='utf-8') as fh:
            text = fh.read()
        ok(len(parse(off).vias) == 2 and 'backdrill' not in text and rec_off is None
           and 'BACK-DRILL' not in log_off and 'Back-drills' not in log_off,
           "the same route without the option adds two vias and no back-drill")
        # a file format KiCad 9 reads cannot carry the token
        with open(src, encoding='utf-8') as fh:
            old_text = fh.read().replace('(version 20260206)', '(version 20241229)')
        old = os.path.join(td, 'old.kicad_pcb')
        with open(old, 'w', encoding='utf-8', newline='') as fh:
            fh.write(old_text)
        old_out = os.path.join(td, 'old_on.kicad_pcb')
        log_old, rec_old = route(old, old_out, '--nets', '/S', '--layers', 'F.Cu', 'In1.Cu',
                                 *RUN, *ASK)
        with open(old_out, encoding='utf-8') as fh:
            old_routed = fh.read()
        ok('(version 20241229)' in old_text and 'back-drills NOT defined' in log_old
           and 'predates KiCad 10' in log_old and rec_old is None
           and 'backdrill' not in old_routed and len(parse(old_out).vias) == 2,
           "asked for on a board in KiCad 9's file format: routed, no back-drill "
           "written, and the log says why")


def test_search_takes_legal_sites():
    print("6: the search places a via where its back-drill is legal")
    import backdrill
    import fab_notes
    for kind, pads in (('a net of two pads', S_PADS), ('a net of three pads', S3_PADS)):
        with tempfile.TemporaryDirectory() as td:
            src = write_board(os.path.join(td, 'b.kicad_pcb'), pads=pads,
                              tracks=WALL + lanes(1, 19))
            args = ['--nets', '/S', '--layers', 'F.Cu', 'In1.Cu', *RUN]
            on, off = os.path.join(td, 'on.kicad_pcb'), os.path.join(td, 'off.kicad_pcb')
            _log, rec = route(src, on, *args, *ASK)
            route(src, off, *args)
            a, b = parse(on), parse(off)
            ok(rec is not None and (rec['drilled'], rec['conflict']) == (2, 0),
               f"{kind}, asked for: both vias are back-drilled: {rec and rec['drilled']}")
            on_x = sorted(round(v.x, 2) for v in a.vias)
            off_x = sorted(round(v.x, 2) for v in b.vias)
            ok(all(x < 1 or x > 19 for x in on_x),
               f"they stand past the ends of the B.Cu tracks, where the wall is clear: "
               f"x = {on_x}")
            ok(all(1 < x < 19 for x in off_x),
               f"the plain route puts its vias between those tracks: x = {off_x}")
            # the judge refuses the plain route's sites: the control for the rows above
            spec = backdrill.make_spec(b, ['/S'], 0.3, 0.2, 0.25)
            cfg = step_config(b, spec)
            _decided, judged = backdrill.finalize(b, cfg, fab_notes.via_snapshot([], None))
            ok((judged['vias'], judged['drilled'], judged['conflict']) == (2, 0, 2),
               f"and a back-drill at either of those sites is refused on B.Cu: "
               f"{judged['conflict']} of {judged['vias']}")


def test_neighbours_judged_in_turn():
    print("6b: two marked vias whose walls cannot both stand: one is kept")
    import backdrill
    import fab_notes
    # 0.8 mm apart: legal as plain 0.6 / 0.3 vias (0.2 mm pad to pad at
    # clearance 0.1), but two 0.6 mm back-drills would leave 0.2 mm between
    # the walls, under the 0.25 mm hole-to-hole minimum.
    mark = backdrill_token(0.6, 'B.Cu', 'In2.Cu')
    tracks = []
    for net, x in (('/S', 9.6), ('/R', 10.4)):
        tracks += [(net, x, 5, x, 2, 'F.Cu', 0.2), (net, x, 5, x, 8, 'In1.Cu', 0.2)]
    with tempfile.TemporaryDirectory() as td:
        pcb = parse(write_board(os.path.join(td, 'n.kicad_pcb'), tracks=tracks,
                                vias=[('/S', 9.6, 5, 0.6, 0.3, mark),
                                      ('/R', 10.4, 5, 0.6, 0.3, mark)]))
    ok(all(v.backdrills for v in pcb.vias), "(both vias arrive marked, as a route leaves them)")
    spec = backdrill.make_spec(pcb, ['/S', '/R'], 0.3, 0.2, 0.25)
    cfg = step_config(pcb, spec)
    decided, rec = backdrill.finalize(pcb, cfg, fab_notes.via_snapshot([], None))
    kept = sorted(round(v.x, 2) for v, tokens in decided.values() if tokens)
    ok((rec['vias'], rec['drilled'], rec['conflict']) == (2, 1, 1) and kept == [9.6],
       f"one drill is kept and the other refused, not both refused: drilled at x = {kept}, "
       f"{rec['conflict']} left")
    ok(rec['conflicts'][0]['with'] == 'drill' and rec['conflicts'][0]['needed_mm'] == 0.25,
       f"the refusal names the other drill at the hole-to-hole minimum: {rec['conflicts'][0]}")
    ok([bool(v.backdrills) for v in sorted(pcb.vias, key=lambda v: v.x)] == [True, False],
       "and the vias are left as decided")


def test_wall_priced_by_the_boards_rules():
    print("6c: the wall is priced by the pair's clearance as the board's rules give it")
    import backdrill
    from routing_config import GridRouteConfig
    # /S's via at (10, 5), to be drilled 0.6 mm on In2.Cu and B.Cu; a 0.2 mm
    # track of /W on B.Cu passes `gap` mm from the wall.
    own = [('/S', 10, 5, 10, 2, 'F.Cu', 0.2), ('/S', 10, 5, 13, 5, 'In1.Cu', 0.2)]

    def needed(gap, floor=0.0, classes=None, layers=None):
        y = round(5 + 0.3 + gap + 0.1, 4)
        with tempfile.TemporaryDirectory() as td:
            pcb = parse(write_board(os.path.join(td, 'w.kicad_pcb'),
                                    tracks=own + [('/W', 5, y, 15, y, 'B.Cu', 0.2)],
                                    vias=[('/S', 10, 5, 0.6, 0.3, '')]))
        cfg = GridRouteConfig(clearance=0.1, via_size=0.6, via_drill=0.3,
                              hole_to_hole_clearance=0.25, hole_clearance=floor,
                              layers=list(pcb.board_info.copper_layers))
        w = net_id(pcb, '/W')
        cfg.net_clearances = {w: c for c in ([classes] if classes else [])}
        cfg.layer_clearances = dict(layers or {})
        cfg.backdrill = backdrill.make_spec(pcb, ['/S'], 0.3, 0.2, 0.25)
        backdrill.install(cfg, pcb)     # reads the board's floor, as a step does
        hits = backdrill.wall_conflicts(backdrill._Index(pcb, cfg.layers), pcb.vias[0],
                                        ['In2.Cu', 'B.Cu'], 0.6, cfg)
        return [(h[0], h[1], h[4]) for h in hits]

    ok(needed(0.22) == [] and needed(0.18) == [('B.Cu', 'track', 0.2)],
       "no class, no rule: the 0.2 mm copper-to-hole floor, over the 0.1 clearance")
    ok(needed(0.22, classes=0.3) == [('B.Cu', 'track', 0.3)],
       "the other net's class at 0.3 is wider than the floor, and is what is asked")
    ok(needed(0.32, classes=0.3, layers={'B.Cu': 0.35}) == [('B.Cu', 'track', 0.35)],
       "a .kicad_dru layer rule of 0.35 on B.Cu replaces the class there")
    ok(needed(0.22, classes=0.3, layers={'B.Cu': 0.05}) == []
       and needed(0.18, classes=0.3, layers={'B.Cu': 0.05}) == [('B.Cu', 'track', 0.2)],
       "a relaxing layer rule (0.05) replaces the class too, and the wall keeps the "
       "hole floor: a wall is a hole")
    ok(needed(0.22, floor=0.3) == [('B.Cu', 'track', 0.3)],
       "a board declaring a 0.3 mm copper-to-hole floor holds the wall at it")
    ok(needed(0.32, classes=0.3, layers={'In2.Cu': 0.5}) == [],
       "(a rule on another layer does not reach the track's)")
    # the plane tap's window memo keys a via with its back-drill
    from plane_pad_tap import _tap_window_key
    keys = []
    with tempfile.TemporaryDirectory() as td:
        for tag, bd in (('a', ''), ('b', backdrill_token(0.6, 'B.Cu', 'In2.Cu')),
                        ('c', backdrill_token(0.6, 'B.Cu', 'In2.Cu'))):
            pcb = parse(write_board(os.path.join(td, tag + '.kicad_pcb'), tracks=own,
                                    vias=[('/S', 10, 5, 0.6, 0.3, bd)]))
            keys.append(_tap_window_key(pcb, pcb)[0][1])
    ok(keys[0] != keys[1] and keys[1] == keys[2],
       "a tap window's memo key tells a back-drilled via from a plain one")


def test_completion_first():
    print("7: with no legal site the net is still routed, and left undrilled")
    for kind, pads in (('a net of two pads', S_PADS), ('a net of three pads', S3_PADS)):
        with tempfile.TemporaryDirectory() as td:
            src = write_board(os.path.join(td, 'b.kicad_pcb'), pads=pads,
                              tracks=WALL + lanes(0.2, 19.8))
            out = os.path.join(td, 'on.kicad_pcb')
            log, rec = route(src, out, '--nets', '/S', '--layers', 'F.Cu', 'In1.Cu',
                             *RUN, *ASK)
            pcb = parse(out)
            m = re.findall(r'^JSON_SUMMARY_MIN: (.*)$', log, re.M)
            tally = json.loads(m[-1]) if m else {}
            ok(tally.get('failed') == 0 and tally.get('routed') == 1
               and tally.get('multipoint_deficit') == 0 and len(pcb.vias) == 2,
               f"{kind}: routed, with its two vias ({tally.get('routed')} routed, "
               f"{tally.get('failed')} failed)")
            ok("routed without the back-drill's via legality" in log,
               "the log says it was routed as it would be without the option")
            ok(rec is not None and (rec['drilled'], rec['conflict']) == (0, 2)
               and rec['conflicts'][0]['layer'] == 'B.Cu'
               and rec['conflicts'][0]['with'] == 'track',
               f"neither via is drilled, and the record names the B.Cu track in the "
               f"way: {rec and rec['conflicts'][:1]}")
            ok(drills(pcb, '/S') == [(), ()], "no back-drill is written")


#: (x, back-drill diameter or 0, what stands near it, its layer, its gap from
#: the wall or -- with no back-drill, or one narrower than the pad -- from
#: the pad's edge, does KiCad 10.0.8 report a clearance violation). Vias are
#: 0.6 / 0.3 at y = 5, drilled from B.Cu to In2.Cu; clearance 0.1, hole
#: clearance 0.25.
CASES = [
    (2.0, 0.8, 'track', 'B.Cu', 0.22, False),    # past the clearance, inside 0.25
    (4.5, 0.8, 'track', 'B.Cu', 0.07, True),     # inside the clearance of the wall
    (7.0, 0.8, 'track', 'In1.Cu', 0.07, False),  # same place on a layer the drill leaves
    (9.5, 0.8, 'track', 'In2.Cu', 0.07, True),   # the span's END layer is removed too
    (12.0, 0.4, 'track', 'B.Cu', 0.05, False),   # a narrower drill REPLACES the pad
    (14.5, 1.0, 'via', None, 0.07, True),        # another net's via pad at the wall
    (17.0, 0.0, 'track', 'B.Cu', 0.07, True),    # no back-drill: the pad, as before
]


def _graded_board(path):
    tracks, vias = [], []
    for x, bd, what, layer, gap, _kicad in CASES:
        vias.append(('/A', x, 5, 0.6, 0.3,
                     backdrill_token(bd, 'B.Cu', 'In2.Cu') if bd else ''))
        r = bd / 2.0 if bd > 0.6 else 0.3
        if what == 'track':
            d = round(r + gap + 0.1, 4)
            tracks.append(('/I', x - 1, 5 + d, x + 1, 5 + d, layer, 0.2))
        else:
            vias.append(('/I', x, round(5 + r + gap + 0.3, 4), 0.6, 0.2, ''))
    write_board(path, tracks=tracks, vias=vias)
    with open(os.path.join(REPO, 'kicad_files', 'flat_hierarchy.kicad_pro'),
              encoding='utf-8') as fh:
        pro = json.load(fh)
    pro['board']['design_settings']['rules'].update(
        min_clearance=0.0, min_hole_clearance=0.25, min_hole_to_hole=0.25,
        min_via_annular_width=0.02, min_via_diameter=0.2, min_through_hole_diameter=0.1,
        min_track_width=0.1, min_copper_edge_clearance=0.1)
    pro['net_settings']['classes'][0].update(clearance=0.1)
    pro['net_settings']['netclass_patterns'] = []
    pro['net_settings']['netclass_assignments'] = None
    with open(os.path.splitext(path)[0] + '.kicad_pro', 'w', encoding='utf-8') as fh:
        json.dump(pro, fh)
    return run_utils.evidence(path, 'the graded board')


def _case_of(x):
    return min(CASES, key=lambda c: abs(c[0] - x))[0]


def test_check_drc_grades_the_disc():
    print("8: check_drc grades a back-drilled via as KiCad 10.0.8 does")
    from check_drc import run_drc
    want = {c[0] for c in CASES if c[5]}
    with tempfile.TemporaryDirectory() as td:
        board = _graded_board(os.path.join(td, 'g.kicad_pcb'))
        with contextlib.redirect_stdout(io.StringIO()):
            rows = run_drc(board, clearance=0.1, quiet=True, print_summary=False)
        got = {_case_of((r.get('via_loc') or r.get('loc1'))[0]) for r in rows
               if r.get('type') in ('via-segment', 'via-via')}
        ok(got == want, f"copper-clearance findings at x = {sorted(got)}; "
           f"KiCad's are at {sorted(want)}")
        ok(4.5 in got and 2.0 not in got,
           "0.07 mm from the wall is a violation and 0.22 mm is not: the wall is held "
           "to the copper clearance, as KiCad holds it")
        ok(12.0 not in got,
           "a back-drill narrower than the pad is the via's copper there, as in KiCad")
        cli = None
        try:
            from kicad_oracle import find_kicad_cli
            cli = find_kicad_cli(warn=False)
        except Exception:                                      # noqa: BLE001
            pass
        if not cli:
            print("  [skip] no kicad-cli here: KiCad itself is not asked "
                  "(the expectations above were measured on 10.0.8)")
            return
        rep = os.path.join(td, 'k.json')
        subprocess.run([cli, 'pcb', 'drc', '--format', 'json', '--severity-all',
                        '--all-track-errors', '-o', rep, board],
                       capture_output=True, text=True, timeout=300)
        with open(run_utils.evidence(rep, "kicad-cli's report"), encoding='utf-8') as fh:
            doc = json.load(fh)
        kicad = {_case_of(min(i['pos']['x'] for i in v['items']))
                 for v in doc.get('violations', []) if v.get('type') == 'clearance'}
        ok(kicad == got, f"kicad-cli {doc.get('kicad_version')} reports clearance "
           f"violations at the same vias: {sorted(kicad)}")


def test_rewrite_only_the_named_vias():
    print("9: set_via_backdrills_in_content rewrites only the vias named")
    from kicad_parser import Backdrill
    from kicad_writer import set_via_backdrills_in_content
    text = board_text(vias=[('/A', 2, 5, 0.6, 0.3, backdrill_token(0.5, 'B.Cu', 'In2.Cu')),
                            ('/A', 6, 5, 0.6, 0.3, ''),
                            ('/A', 10, 5, 0.6, 0.3, backdrill_token(0.5, 'B.Cu', 'B.Cu'))])
    uuids = re.findall(r'\(via\n.*?\(uuid "([^"]+)"\)', text, re.S)
    ok(len(uuids) == 3, "(the fixture has three vias)")
    same, n0 = set_via_backdrills_in_content(
        text, {uuids[0]: {'backdrill': Backdrill(0.5, 'B.Cu', 'In2.Cu')}})
    ok(n0 == 0 and same == text, "a via that already reads as asked is left byte for byte")
    new, n = set_via_backdrills_in_content(
        text, {uuids[0]: {}, uuids[1]: {'backdrill': Backdrill(0.55, 'B.Cu', 'In2.Cu')}})
    with tempfile.TemporaryDirectory() as td:
        path = os.path.join(td, 'w.kicad_pcb')
        with open(path, 'w', encoding='utf-8', newline='') as fh:
            fh.write(new)
        got = [{t: tuple(bd) for t, bd in v.backdrills.items()}
               for v in sorted(parse(path).vias, key=lambda v: v.x)]
    ok(n == 2 and got == [{}, {'backdrill': (0.55, 'B.Cu', 'In2.Cu')},
                          {'backdrill': (0.5, 'B.Cu', 'B.Cu')}],
       f"one cleared, one set, the third untouched: {got}")


def test_manifest_to_plan():
    print("10: manifest_to_plan carries the flags into a plan step")
    sys.path.insert(0, os.path.join(HERE, 'stress'))
    import manifest_to_plan as m2p
    step = m2p.parse_command(
        ['python3', 'py_router/route.py', 'in.kicad_pcb', 'out.kicad_pcb', '--nets', '*',
         '--backdrill-nets', '/DDR*', '!/DDR_CK*', '--backdrill-oversize', '0.2',
         '--backdrill-margin', '0.15', '--backdrill-min-stub', '0.3', '--clearance', '0.1'])
    flat = json.dumps(step)
    params = step.get('params', step) if isinstance(step, dict) else {}
    ok(params.get('backdrill_nets') == ['/DDR*', '!/DDR_CK*'],
       f"the patterns stay a list: {params.get('backdrill_nets')}")
    ok([params.get(k) for k in ('backdrill_oversize', 'backdrill_margin',
                                'backdrill_min_stub')] == [0.2, 0.15, 0.3],
       "and the three numbers ride as plain values")
    ok(params.get('clearance') == 0.1 and step.get('nets') == ['*'],
       f"(the flags around them are still read: {flat})")


def main():
    for fn in (test_plan_spans, test_option_is_whole,
               test_declared_backdrill_is_an_obstacle, test_new_via_off_the_wall,
               test_define_when_asked, test_search_takes_legal_sites,
               test_neighbours_judged_in_turn, test_wall_priced_by_the_boards_rules,
               test_completion_first, test_check_drc_grades_the_disc,
               test_rewrite_only_the_named_vias, test_manifest_to_plan):
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
