#!/usr/bin/env python3
"""#1853 review: the gaps a hole_clearance rule left after the route step
first read it (test_1853_route_hole_clearance holds the first cut).

The fixtures are tests/fixtures/1853's (Default class clearance 0.0889 mm,
track 0.1 mm, via 0.6/0.3, rule `hole_clearance (min 0.254mm)`, routed at a
0.01 mm grid), edited here where a case needs a second pad or another rule.

Checks:
  1. a pad's hole is held when the pad's OWN net is routed in the same step:
     `pth_tight` / `pth_fits` with a second pad of the hole's net, routed
     `--nets SIG OTHER`. The base map leaves a routed net's pads to that
     net's own keep-outs, which held the pad's copper and not its hole, so
     SIG ran 0.45 mm from the hole's centre (0.250 mm from the hole, 0.254
     needed) and check_drc failed the board on the rule. Now SIG is refused
     on the tight board and routed clear on the other, check_drc clean; and
     the per-net cache holds a new via's copper off such a hole too.
  2. the rule walk reads every rule as check_drc's grader does, and the
     grader (`hole_rule_refusal`) agrees with each value the walk gives:
     (a) a rule naming one inner layer binds that layer only -- In2.Cu was
         read as In1.Cu, so rules `a` 0.3 then `b` (layer In1.Cu) 0.1 gave
         In2.Cu 0.1 where the grader asks 0.3;
     (b) `B.inDiffPair('D')` binds a pair net however the nets group -- the
         groups were keyed by class and quoted names, so a pair net added
         after two others was never asked and the rule bound nothing;
     (c) a rule on a via and a pad (`A.Type == 'Via' && B.Type == 'Pad'`)
         holds a new via's copper off a pad's hole and its hole off the
         pad's copper, and no track -- the walk asked about tracks only;
     (d) a rule whose condition is not read binds nothing and is named NOT
         honoured, and the rules before it still bind -- it used to end the
         walk, so a rule above it bound nothing while named honoured.
"""
import json
import math
import os
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)
from run_utils import check, evidence  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
FIX = os.path.join(TESTS, 'fixtures', '1853')
FAILS = []

# a second pad of the hole's net, away from SIG's row
SECOND_PAD = '''		(pad "2" smd rect
			(at 2 -5)
			(size 0.3 0.3)
			(layers "F.Cu" "F.Mask")
			(net "OTHER")
			(uuid "72cb077a-5ebd-58ae-bf6c-43ff023feac7")
		)
'''


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def stage(td, name, dru=None, edit=None):
    """Copy a fixture into ``td``; ``edit`` rewrites its board text."""
    os.makedirs(td, exist_ok=True)
    for ext in ('.kicad_pcb', '.kicad_pro', '.kicad_dru'):
        shutil.copy(evidence(os.path.join(FIX, name + ext), 'the fixture'),
                    os.path.join(td, name + ext))
    board = os.path.join(td, name + '.kicad_pcb')
    if edit is not None:
        with open(board, encoding='utf-8') as fh:
            text = fh.read()
        new = edit(text)
        if new == text:
            raise RuntimeError(f"the edit of {name} changed nothing")
        with open(board, 'w', encoding='utf-8') as fh:
            fh.write(new)
    if dru is not None:
        with open(os.path.join(td, name + '.kicad_dru'), 'w', encoding='utf-8') as fh:
            fh.write(dru)
    return board


def add_second_pad(text):
    """Give the hole's footprint a second pad of its net, after the first."""
    i = text.index('(net "OTHER")')
    j = text.index('\t\t)\n', i) + len('\t\t)\n')
    return text[:j] + SECOND_PAD + text[j:]


def route(td, board, tag, nets, extra=()):
    out = os.path.join(td, tag + '.kicad_pcb')
    summary = os.path.join(td, tag + '.json')
    argv = (PY + [os.path.join(REPO, 'py_router', 'route.py'), board, out, '--nets']
            + list(nets) + ['--grid-step', '0.01', '--escalation', 'off',
                            '--json-out', summary] + list(extra))
    r = check(argv, accept=True, timeout=900)
    with open(evidence(summary, 'the summary'), encoding='utf-8') as fh:
        doc = json.load(fh)
    return (out, sorted(doc.get('routed_single') or []),
            sorted(doc.get('failed_single') or []), (r.stdout or '') + (r.stderr or ''))


def drc_clean(board):
    """check_drc passes the board; a failure is reported, a crash raised."""
    argv = PY + [os.path.join(REPO, 'py_router', 'check_drc.py'),
                 evidence(board, 'the routed board'), '--clearance-margin-mm', '0.0005']
    try:
        r = check(argv, accept=True)
    except AssertionError as e:
        if 'BROKEN TEST' in str(e) or 'DRC VIOLATIONS' not in str(e):
            raise
        print(str(e)[-700:])
        return False
    return 'NO DRC VIOLATIONS FOUND' in (r.stdout or '')


def load(board):
    from kicad_parser import parse_kicad_pcb
    return parse_kicad_pcb(board)


def rules_of(board, pcb, report=False):
    from routing_config import GridRouteConfig
    from design_rules import install_design_rules
    import contextlib
    import io
    cfg = GridRouteConfig(grid_step=0.01, clearance=0.0889, track_width=0.1,
                          via_size=0.6, via_drill=0.3,
                          layers=list(pcb.board_info.copper_layers))
    with contextlib.redirect_stdout(io.StringIO()):
        install_design_rules(cfg, board, pcb, net_keepouts=report)
    return cfg


def net_id(pcb, name):
    return next(n for n in pcb.nets.values() if n.name == name).net_id


def sig_reach(pcb, x, y):
    from geometry_utils import point_to_segment_distance
    sig = net_id(pcb, 'SIG')
    return min(point_to_segment_distance(x, y, s.start_x, s.start_y, s.end_x, s.end_y)
               for s in pcb.segments if s.net_id == sig)


def routed_pad_hole(td):
    print("1. a pad's hole, its own net routed in the same step")
    sub = os.path.join(td, 'tight')
    out, routed, failed, _log = route(sub, stage(sub, 'pth_tight', edit=add_second_pad),
                                      'out', ('SIG', 'OTHER'), ('--layers', 'F.Cu'))
    ok('SIG' in failed and 'SIG' not in routed,
       f"pth_tight: SIG is refused the row 0.45 mm from the hole: {routed} / {failed}")
    ok(drc_clean(out), "and check_drc is clean")

    sub = os.path.join(td, 'fits')
    out, routed, failed, _log = route(sub, stage(sub, 'pth_fits', edit=add_second_pad),
                                      'out', ('SIG', 'OTHER'), ('--layers', 'F.Cu'))
    ok(routed == ['OTHER', 'SIG'] and failed == [], f"pth_fits: both route: {routed} / {failed}")
    d = sig_reach(load(out), 15.0, 6.0)
    ok(d >= 0.15 + 0.254 + 0.05 - 1e-6,
       f"SIG stays {d:.3f} mm from the hole's centre (needs {0.15 + 0.254 + 0.05:.3f})")
    ok(drc_clean(out), "and check_drc is clean")

    # the per-net cache: the hole's net is routed, so its pads come from there
    print("   the per-net cache of the hole's net")
    from obstacle_cache import precompute_net_obstacles
    small_ring = (lambda t: add_second_pad(t).replace(
        '(size 0.6 0.6)\n\t\t\t(drill 0.3)', '(size 0.34 0.34)\n\t\t\t(drill 0.3)'))
    got = {}
    for tag, dru in (('rule', None), ('none', '(version 1)\n')):
        b = stage(os.path.join(td, 'cache_' + tag), 'pth_fits', dru=dru, edit=small_ring)
        pcb = load(b)
        cfg = rules_of(b, pcb)
        data = precompute_net_obstacles(pcb, net_id(pcb, 'OTHER'), cfg)
        got[tag] = (cfg, {tuple(c) for c in data.blocked_cells.tolist()},
                    {tuple(c) for c in data.blocked_vias.tolist()})
    cfg, cells, vias = got['rule']
    _c0, cells0, vias0 = got['none']
    # a track centre 0.45 mm from the hole: past the 0.17 mm pad's copper
    # keep-out (0.17 + 0.0889 + 0.05), inside the rule's (0.15 + 0.254 + 0.05)
    ok((1500, 645, 0) in cells and (1500, 645, 0) not in cells0,
       "a track cell 0.45 mm from the hole is held with the rule, not without")
    # a new via's centre 0.68 mm from the hole: its copper stands 0.23 mm
    # from the hole (hole_r + rule + via_size/2 = 0.704 needed), past the
    # thin pad's via keep-out and the hole-to-hole stamp
    h2h_reach = 0.15 + 0.15 + cfg.hole_to_hole_clearance
    pad_reach = 0.17 + 0.3 + 0.0889 + 0.005 + cfg.via_hole_extra(0.0889)
    r = 0.68
    ok(max(h2h_reach, pad_reach) < r < 0.704,
       f"the probe at {r} mm lies past the other keep-outs ({max(h2h_reach, pad_reach):.3f})")
    probe = (1500, 600 + int(round(r / 0.01)))
    ok(probe in vias and probe not in vias0,
       "a new via 0.68 mm from the hole is held with the rule, not without")


# two pads of a pair, added after the fixture's own nets
PAIR = '''	(footprint "fixture:DP"
		(layer "F.Cu")
		(uuid "11111111-2222-3333-4444-555555555555")
		(at 0 0)
		(property "Reference" "DP1" (at 0 0 0) (layer "F.SilkS") (hide yes)
			(uuid "11111111-2222-3333-4444-555555555556")
			(effects (font (size 1 1) (thickness 0.15))))
		(attr smd)
		(pad "1" smd rect (at 2 1) (size 0.3 0.3) (layers "F.Cu" "F.Mask") (net "D_P")
			(uuid "11111111-2222-3333-4444-555555555557"))
		(pad "2" smd rect (at 3 1) (size 0.3 0.3) (layers "F.Cu" "F.Mask") (net "D_N")
			(uuid "11111111-2222-3333-4444-555555555558"))
	)
'''


def add_pair(text):
    return text.replace('\t(embedded_fonts no)', PAIR + '\t(embedded_fonts no)')


def dru(*rules):
    return '(version 1)\n' + ''.join(rules)


def grader(pcb, cfg, gap, layer='F.Cu', net='SIG', hole=(15.0, 6.0), hole_r=0.15,
           kind='track'):
    """check_drc's refusal of one candidate of ``net`` whose copper edge
    stands ``gap`` mm from the hole's edge, beside the hole on ``layer``."""
    from check_drc import hole_rule_refusal
    from kicad_parser import Segment, Via
    nid = net_id(pcb, net)
    if kind == 'via':
        v = Via(x=hole[0] + hole_r + gap + 0.3, y=hole[1], size=0.6, drill=0.3,
                layers=['F.Cu', 'B.Cu'], net_id=nid)
        return hole_rule_refusal(pcb, cfg.rules, vias=[v])
    x = hole[0] + hole_r + gap + 0.05
    s = Segment(start_x=x, start_y=hole[1] - 1.0, end_x=x, end_y=hole[1] + 1.0,
                width=0.1, layer=layer, net_id=nid)
    return hole_rule_refusal(pcb, cfg.rules, segments=[s])


def walk(td):
    print("2. the rule walk, against the grader")
    from obstacle_map import build_base_obstacle_map
    # (a)
    b = stage(os.path.join(td, 'wa'), 'track_fits', dru=dru(
        '(rule "a" (constraint hole_clearance (min 0.3mm)))\n',
        '(rule "b" (layer "In1.Cu") (constraint hole_clearance (min 0.1mm)))\n'))
    pcb = load(b)
    cfg = rules_of(b, pcb)
    got = {l: cfg.hole_rules.via_value(pcb.vias[0], l) for l in pcb.board_info.copper_layers}
    ok(got == {'F.Cu': 0.3, 'In1.Cu': 0.1, 'In2.Cu': 0.3, 'B.Cu': 0.3},
       f"(a) rule b binds In1.Cu alone: {got}")
    ok(bool(grader(pcb, cfg, 0.25, 'In2.Cu', 'WALL')) and not grader(pcb, cfg, 0.25, 'In1.Cu', 'WALL'),
       "(a) and the grader asks 0.3 on In2.Cu, 0.1 on In1.Cu")
    # (b)
    b = stage(os.path.join(td, 'wb'), 'track_fits', edit=add_pair, dru=dru(
        '(rule "dp" (constraint hole_clearance (min 0.3mm))\n'
        "  (condition \"B.inDiffPair('D')\"))\n"))
    pcb = load(b)
    cfg = rules_of(b, pcb)
    got = cfg.hole_rules.via_value(pcb.vias[0], 'F.Cu')
    ok(got == 0.3, f"(b) inDiffPair binds the pair's copper at the via's hole: {got}")
    ok(bool(grader(pcb, cfg, 0.25, net='D_P')) and not grader(pcb, cfg, 0.25, net='SIG'),
       "(b) and the grader holds D_P at it, SIG not")
    # (c)
    vp = dru('(rule "vp" (constraint hole_clearance (min 0.4mm))\n'
             "  (condition \"A.Type == 'Via' && B.Type == 'Pad'\"))\n")
    b = stage(os.path.join(td, 'wc'), 'pth_fits', dru=vp)
    pcb = load(b)
    cfg = rules_of(b, pcb, report=True)
    hr = cfg.hole_rules
    pad = next(p for fp in pcb.footprints.values() for p in fp.pads if (p.drill or 0) > 0)
    ok(hr.pad_value(pad, 'F.Cu') is None, "(c) no track is held off the pad's hole")
    ok(hr.pad_value(pad, 'F.Cu', 'via') == 0.4, "(c) a new via's copper is, at 0.4")
    want = 0.15 + 0.4 - 0.3 - 0.0889
    ok(abs(cfg.via_hole_extra(0.0889, pad=pad) - want) < 1e-9,
       f"(c) and a new via's hole off the pad's copper: grows by {want:.4f} mm")
    ok(cfg.via_hole_extra(0.0889) == 0.0, "(c) but not off a track's")
    ok(bool(grader(pcb, cfg, 0.30, kind='via')) and not grader(pcb, cfg, 0.30),
       "(c) the grader refuses a via 0.30 mm from the hole and allows a track there")
    sig = net_id(pcb, 'SIG')
    maps = {}
    for tag, text in (('rule', vp), ('none', '(version 1)\n')):
        bb = stage(os.path.join(td, 'wc_' + tag), 'pth_fits', dru=text)
        pp = load(bb)
        cc = rules_of(bb, pp)
        maps[tag] = build_base_obstacle_map(pp, cc, [sig])
    # a new via's centre 0.15 + 0.35 + 0.3 = 0.80 mm right of the hole
    gx, gy = 1500 + 80, 600
    ok(maps['rule'].is_via_blocked(gx, gy) and not maps['none'].is_via_blocked(gx, gy),
       "(c) the base map holds a new via 0.35 mm from the hole with the rule, not without")
    ok(any(r.name == 'vp' and r.honoured for r in cfg.rules.rules), "(c) the rule is honoured")
    # (d)
    b = stage(os.path.join(td, 'wd'), 'track_fits', dru=dru(
        '(rule "a" (constraint hole_clearance (min 0.254mm)))\n',
        '(rule "odd" (constraint hole_clearance (min 0.3mm))\n'
        "  (condition \"A.getField('x') == 'y'\"))\n"))
    pcb = load(b)
    cfg = rules_of(b, pcb, report=True)
    got = cfg.hole_rules.via_value(pcb.vias[0], 'F.Cu')
    ok(got == 0.254, f"(d) rule a still binds below a rule not read: {got}")
    honoured = {r.name: bool(r.honoured) for r in cfg.rules.rules}
    named = dict(cfg.rules.unsupported())
    ok(honoured == {'a': True, 'odd': False} and 'odd' in named and 'a' not in named,
       f"(d) a is honoured, odd named NOT honoured: {honoured} / {named}")


def main():
    with tempfile.TemporaryDirectory() as td:
        routed_pad_hole(td)
        walk(td)
    if FAILS:
        print(f"\nFAILED ({len(FAILS)}):")
        for f in FAILS:
            print(f"  - {f}")
        return 1
    print("\nALL PASSED")
    return 0


if __name__ == '__main__':
    sys.exit(main())
