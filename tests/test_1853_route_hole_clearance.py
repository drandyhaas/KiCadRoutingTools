#!/usr/bin/env python3
"""#1853: route.py routes to a `.kicad_dru` `hole_clearance` rule.

KiCad holds the copper of one net off the HOLE of another net's via or pad
at the last `hole_clearance` rule binding the pair. check_drc graded that
(`hole_rule_grade`); the route step did not read it: it kept copper off a
plated hole only through the clearance to the ring or pad around it, and off
an unplated hole at Board Setup's minimum.

The fixtures (tests/fixtures/1853) are synthetic: Default class clearance
0.0889 mm, track 0.1 mm, and one rule, `hole_clearance (min 0.254mm)` (0.35
for `npth`), with no condition. Routed at a 0.01 mm grid.

  * track_tight / track_fits: a wall with one slot, a 0.6/0.3 mm via of
    another net standing in it. The ring's keep-out is 0.3 + 0.0889 + 0.05
    = 0.4389 mm from the via's centre, the rule's 0.15 + 0.254 + 0.05 =
    0.454. track_tight leaves the row at 0.45 only; track_fits also the
    rows from 0.46.
  * pth_tight / pth_fits: the same with a plated pad's hole.
  * npth: an unplated hole, rule 0.35 mm.
  * newvia_tight / newvia_fits: SIG has one pad on F.Cu and one on B.Cu
    inside a closed, locked ring of another net's track, so its via stands
    between the ring's long sides: 0.445 mm from each (the ring's 0.095 mm
    from the via's copper is legal, the hole's 0.245 mm is not) and 0.46.

Measured with kicad-cli 10.0.8: the boards the unfixed step routed from
track_tight and newvia_tight carry 1 and 2 hole_clearance errors; every
board routed here carries none.

Checks:
  1. the rule walk: the value per hole and layer, a rule's layer clause, a
     rule of severity ignore, conditions on net names (nets are asked for in
     groups that read alike), a condition that is not read;
  2. how far a new via's keep-out grows, and that it is 0 without a rule;
  3. each *_tight board: SIG is not routed with the rule and is routed with
     the rule file emptied, and that second board fails check_drc once the
     rule is put back (the fixture tells the two apart);
  4. each *_fits board and npth: SIG routes, the copper stands the rule's
     distance off the hole, check_drc is clean, the run names the rule;
  5. the gate a via generator off the map asks (`hole_rule_refusal`);
  6. the map's reference counts balance (KICAD_OBSTACLE_AUDIT).
"""
import json
import math
import os
import shutil
import sys
import tempfile
from types import SimpleNamespace

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)
from run_utils import check, evidence  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
FIX = os.path.join(TESTS, 'fixtures', '1853')
FAILS = []
NO_RULES = '(version 1)\n'


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def stage(td, name, dru=None):
    os.makedirs(td, exist_ok=True)
    for ext in ('.kicad_pcb', '.kicad_pro', '.kicad_dru'):
        shutil.copy(evidence(os.path.join(FIX, name + ext), 'the fixture'),
                    os.path.join(td, name + ext))
    if dru is not None:
        with open(os.path.join(td, name + '.kicad_dru'), 'w', encoding='utf-8') as fh:
            fh.write(dru)
    return os.path.join(td, name + '.kicad_pcb')


def route(td, board, tag, one_layer, env=None):
    out = os.path.join(td, tag + '.kicad_pcb')
    summary = os.path.join(td, tag + '.json')
    args = ['--nets', 'SIG', '--grid-step', '0.01', '--escalation', 'off',
            '--json-out', summary]
    args += (['--layers', 'F.Cu'] if one_layer else
             ['--via-size', '0.6', '--via-drill', '0.3', '--clearance', '0.0889',
              '--track-width', '0.1'])
    r = check(PY + [os.path.join(REPO, 'py_router', 'route.py'), board, out] + args,
              accept=True, timeout=900, env=env)
    with open(evidence(summary, 'the summary'), encoding='utf-8') as fh:
        doc = json.load(fh)
    return (out, sorted(doc.get('routed_single') or []),
            sorted(doc.get('failed_single') or []), (r.stdout or '') + (r.stderr or ''))


def drc(board, flagged=False):
    """check_drc's report; `flagged` asserts it fails on the rule, by name."""
    argv = PY + [os.path.join(REPO, 'py_router', 'check_drc.py'),
                 evidence(board, 'the routed board'), '--clearance-margin-mm', '0.0005']
    r = (check(argv, refuse="rule 'holes'", code=1) if flagged
         else check(argv, accept=True))
    return r.stdout or ''


def load(board):
    from kicad_parser import parse_kicad_pcb
    return parse_kicad_pcb(board)


def rules_of(board, pcb, dru_text=None):
    """(config with the board's rules installed, the DesignRules)."""
    from routing_config import GridRouteConfig
    from design_rules import install_design_rules
    cfg = GridRouteConfig(grid_step=0.01, clearance=0.0889, track_width=0.1,
                          via_size=0.6, via_drill=0.3,
                          layers=list(pcb.board_info.copper_layers))
    import contextlib
    import io
    with contextlib.redirect_stdout(io.StringIO()):
        install_design_rules(cfg, board, pcb)
    return cfg


def sig_reach(pcb, x, y):
    """The least distance from (x, y) to the centreline of SIG's tracks."""
    from geometry_utils import point_to_segment_distance
    sig = next(n for n in pcb.nets.values() if n.name == 'SIG').net_id
    return min(point_to_segment_distance(x, y, s.start_x, s.start_y, s.end_x, s.end_y)
               for s in pcb.segments if s.net_id == sig)


def rule_walk(td):
    print("1. the rule walk")
    from hole_rule_routing import RoutedHoleRules
    board = stage(os.path.join(td, 'walk'), 'track_fits')
    pcb = load(board)
    cfg = rules_of(board, pcb)
    hr = cfg.hole_rules
    via = pcb.vias[0]
    ok(hr is not None and all(abs((hr.via_value(via, l) or 0) - 0.254) < 1e-9
                              for l in pcb.board_info.copper_layers),
       "an unconditional rule gives its value at a via's hole on every layer")
    for tag, dru, want in (
            ('layer', '(version 1)\n(rule "holes"\n  (layer "B.Cu")\n'
                      '  (constraint hole_clearance (min 0.254mm)))\n',
             {'F.Cu': None, 'B.Cu': 0.254}),
            ('ignore', '(version 1)\n(rule "holes"\n  (severity ignore)\n'
                       '  (constraint hole_clearance (min 0.254mm)))\n',
             {'F.Cu': None, 'B.Cu': None})):
        b = stage(os.path.join(td, 'walk_' + tag), 'track_fits', dru=dru)
        p = load(b)
        h = rules_of(b, p).hole_rules
        got = {l: (h.via_value(p.vias[0], l) if h is not None else None)
               for l in ('F.Cu', 'B.Cu')}
        ok(got == want, f"{tag}: {got}")
    # nets are asked for in groups that read alike; a condition on a net's
    # name, or comparing the two items' names, must still decide every pair
    for tag, cond, want in (('names differ', 'A.NetName != B.NetName', 0.254),
                            ('the other net by name', "A.NetName == 'SIG'", 0.254),
                            ('a name pattern', "A.NetName == 'SI*'", 0.254),
                            ('a net that is not there', "A.NetName == 'NOPE'", None),
                            ("the hole's own net alone", "A.NetName == 'OTHER' && B.NetName == 'OTHER'",
                             None)):
        b = stage(os.path.join(td, 'walk_' + tag.replace(' ', '_').replace("'", '')),
                  'track_fits', dru=('(version 1)\n(rule "holes"\n'
                                     '  (constraint hole_clearance (min 0.254mm))\n'
                                     f'  (condition "{cond}"))\n'))
        p = load(b)
        h = rules_of(b, p).hole_rules
        got = h.via_value(p.vias[0], 'F.Cu') if h is not None else None
        ok(got == want, f"{tag} ({cond}): {got}")
    b = stage(os.path.join(td, 'walk_unread'), 'track_fits', dru=(
        '(version 1)\n(rule "holes"\n  (constraint hole_clearance (min 0.254mm))\n'
        "  (condition \"A.someFutureFunction('x')\"))\n"))
    p = load(b)
    c = rules_of(b, p)
    h = c.hole_rules
    ok(h is None or h.via_value(p.vias[0], 'F.Cu') is None,
       "a rule whose condition is not read binds nothing here")
    ok(not any('routed tracks and vias' in (r.honoured or '') for r in c.rules.rules),
       "and is not named as routed to")
    ok(rules_of(stage(os.path.join(td, 'walk_none'), 'track_fits', dru=NO_RULES),
                pcb).hole_rules is None, "no hole_clearance rule: nothing installed")


def via_growth(td):
    print("2. a new via's keep-out")
    board = stage(os.path.join(td, 'grow'), 'newvia_fits')
    pcb = load(board)
    cfg = rules_of(board, pcb)
    want = 0.15 + 0.254 - 0.3 - 0.0889
    ok(abs(cfg.via_hole_extra(0.0889) - want) < 1e-9,
       f"grows by drill/2 + rule - size/2 - clearance = {want:.4f} mm")
    ok(cfg.via_hole_extra(0.2) == 0.0, "by nothing where the ring and clearance cover the rule")
    b2 = stage(os.path.join(td, 'grow_none'), 'newvia_fits', dru=NO_RULES)
    ok(rules_of(b2, load(b2)).via_hole_extra(0.0889) == 0.0, "and by nothing without a rule")


def tight(td, name, one_layer):
    print(f"3. {name}")
    sub = os.path.join(td, name)
    _out, routed, failed, log = route(sub, stage(sub, name), 'with', one_layer)
    ok(routed == [] and failed == ['SIG'], f"with the rule SIG is not routed: {routed} / {failed}")
    if not one_layer:
        ok(('declined' in log or 'refused:' in log) and 'hole_clearance 0.254mm' in log,
           "and the rescue's escape via is refused by the rule, by name")
    sub2 = os.path.join(td, name + '_norule')
    out, routed, failed, _log = route(sub2, stage(sub2, name, dru=NO_RULES), 'without',
                                      one_layer)
    ok(routed == ['SIG'], f"with the rule file emptied it is routed: {routed} / {failed}")
    shutil.copy(os.path.join(FIX, name + '.kicad_dru'), os.path.join(sub2, 'without.kicad_dru'))
    ok('-HOLE violations' in drc(out, flagged=True),
       "and check_drc fails that board on the rule once it is back")


def fits(td, name, one_layer, hole_xy, hole_r, rule):
    print(f"4. {name}")
    sub = os.path.join(td, name)
    out, routed, failed, log = route(sub, stage(sub, name), 'out', one_layer)
    ok(routed == ['SIG'] and failed == [], f"SIG routes: {routed} / {failed}")
    pcb = load(out)
    if hole_xy is not None:
        d = sig_reach(pcb, *hole_xy)
        need = hole_r + rule + 0.05
        ok(d >= need - 1e-6, f"its track stays {d:.3f} mm from the hole's centre (needs {need:.3f})")
    else:
        sig = next(n for n in pcb.nets.values() if n.name == 'SIG').net_id
        vias = [v for v in pcb.vias if v.net_id == sig]
        ok(len(vias) >= 1, f"through {len(vias)} via(s) of its own")
        from geometry_utils import point_to_segment_distance
        gap = min(point_to_segment_distance(v.x, v.y, s.start_x, s.start_y, s.end_x, s.end_y)
                  - v.drill / 2.0 - s.width / 2.0
                  for v in vias for s in pcb.segments if s.net_id != sig)
        ok(gap >= rule - 1e-6, f"whose hole stands {gap:.3f} mm from the other net's track")
    ok('NO DRC VIOLATIONS FOUND' in drc(out), "check_drc is clean at KiCad's tolerance")
    ok(f"hole_clearance min {rule:g} mm: routed tracks and vias are kept" in log,
       "the run names the rule as routed to")
    return sub


def gate(td):
    print("5. the gate for a via placed off the map")
    from check_drc import hole_rule_refusal
    from kicad_parser import Via
    for name, refused in (('newvia_tight', True), ('newvia_fits', False)):
        board = stage(os.path.join(td, 'gate_' + name), name)
        pcb = load(board)
        cfg = rules_of(board, pcb)
        sig = next(n for n in pcb.nets.values() if n.name == 'SIG').net_id
        v = Via(x=9.0, y=6.0, size=0.6, drill=0.3, layers=['F.Cu', 'B.Cu'], net_id=sig)
        why = hole_rule_refusal(pcb, cfg.rules, vias=[v])
        ok(bool(why) == refused, f"{name}: {'refused' if why else 'allowed'}"
                                  + (f" ({why})" if why else ''))
        ok(hole_rule_refusal(pcb, None, vias=[v]) == '', f"{name}: free without rules")


def audit(td, sub, name, one_layer):
    env = dict(os.environ, KICAD_OBSTACLE_AUDIT='1')
    _o, _r, _f, log = route(sub, os.path.join(sub, name + '.kicad_pcb'), 'audit',
                            one_layer, env=env)
    ok('[OBSTACLE AUDIT]' in log and 'BALANCED' in log,
       f"{name}: the working map returns exactly to the base map")


def main():
    with tempfile.TemporaryDirectory() as td:
        rule_walk(td)
        via_growth(td)
        tight(td, 'track_tight', True)
        tight(td, 'pth_tight', True)
        tight(td, 'newvia_tight', False)
        s1 = fits(td, 'track_fits', True, (15.0, 6.0), 0.15, 0.254)
        fits(td, 'pth_fits', True, (15.0, 6.0), 0.15, 0.254)
        fits(td, 'npth', True, (15.0, 6.0), 0.15, 0.35)
        s2 = fits(td, 'newvia_fits', False, None, 0.15, 0.254)
        gate(td)
        print("6. the map's reference counts balance")
        audit(td, s1, 'track_fits', True)
        audit(td, s2, 'newvia_fits', False)
    if FAILS:
        print(f"\nFAILED ({len(FAILS)}):")
        for f in FAILS:
            print(f"  - {f}")
        return 1
    print("\nALL PASSED")
    return 0


if __name__ == '__main__':
    sys.exit(main())
