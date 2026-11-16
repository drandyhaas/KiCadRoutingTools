#!/usr/bin/env python3
"""#1853, second part: the via generators that do not place on the route
step's obstacle map hold a `.kicad_dru` `hole_clearance` rule too.

The fan-out engines, the GND vias and the plane taps test a via site by their
own means. Each now also asks `check_drc.HoleRuleGate` (many sites on one
board, indexed once) or `check_drc.hole_rule_refusal` (one candidate), which
run `hole_rule_grade` -- the checker's own measurement -- over the board near
the candidate, so they refuse what check_drc would report.

Checks:
  A. the gate: on tests/fixtures/1853/newvia_tight a via between the two
     tracks is refused, on newvia_fits it is allowed, a second ask is the
     same answer, and a board with no hole_clearance rule has no gate;
  B. the BGA under-pad engine (the 6x6 0.5 mm part of
     tests/test_fanout_through_via_layers.py: a 0.25/0.15 via under a ball
     leaves 0.275 mm from its hole to the next ball): with a 0.2 mm rule it
     drills what it drills with no rule; with a 0.3 mm rule it drills none;
  C. the plane tap (through the route step's rescue, which calls
     `tap_pad_with_escalation`): on newvia_tight the tap is refused by the
     rule, by name;
  D. every generator asks: the BGA dogbone and under-pad engines, the QFN
     engine, the GND vias and the plane tap each call the gate.
  E. the gate grades the generator's own copper: a via site beside a via of
     another net in a list the gate watches is refused while the via is in
     it and allowed once it is taken back; a trial's own via passed for one
     question refuses the same way; a track 0.25 mm from a via's hole is
     refused at a 0.254 rule and one 0.26 mm from it allowed.
  F. the call's copper is graded once the engines are done, completion
     first: a fan-out track 0.250 mm from another net's via hole (0.254
     needed) is kept and named against the rule (`hole_disclose`, the QFN
     fan-out's pass), one 0.27 mm from it is not named; and the BGA channel
     engine on the 6x6 part with a 0.25 mm rule, whose escapes stood inside
     it, re-escapes those balls at the rule, leaves no ball it had escaped
     bare (the first escape is the last rung), and names in LAST_HOLE_RULES
     exactly the rows its copper still has.
  The gate and the passes read the board's rules through the routing
  chokepoint (`check_drc.board_design_rules`), the table a step's
  `config.rules` holds.
"""
import ast
import contextlib
import io
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


def gate(td):
    print("A. the gate")
    from check_drc import HoleRuleGate
    from kicad_parser import parse_kicad_pcb
    for name, refused in (('newvia_tight', True), ('newvia_fits', False)):
        pcb = parse_kicad_pcb(stage(os.path.join(td, 'a_' + name), name))
        g = HoleRuleGate(pcb)
        sig = next(n for n in pcb.nets.values() if n.name == 'SIG').net_id
        why = g.via_why(9.0, 6.0, 0.6, 0.3, sig)
        ok(bool(g) and bool(why) == refused,
           f"{name}: {'refused' if why else 'allowed'}" + (f" ({why})" if why else ''))
        ok(g.via_why(9.0, 6.0, 0.6, 0.3, sig) == why, f"{name}: asked again, the same answer")
    pcb = parse_kicad_pcb(stage(os.path.join(td, 'a_none'), 'newvia_tight', dru='(version 1)\n'))
    g = HoleRuleGate(pcb)
    ok(not g and g.via_why(9.0, 6.0, 0.6, 0.3, 1) == '', "no hole_clearance rule: no gate")


def bga(td):
    print("B. the BGA under-pad engine")
    import test_fanout_through_via_layers as bgafix
    from bga_fanout import generate_bga_fanout
    counts = {}
    for tag, rule in (('none', None), ('0.2', 0.2), ('0.3', 0.3)):
        sub = os.path.join(td, 'b_' + tag)
        os.makedirs(sub)
        pcb = bgafix.fixture('F')
        pcb.source_path = os.path.join(sub, 'b.kicad_pcb')
        with open(pcb.source_path, 'w', encoding='utf-8') as fh:
            fh.write('(kicad_pcb)\n')
        if rule is not None:
            with open(os.path.join(sub, 'b.kicad_dru'), 'w', encoding='utf-8') as fh:
                fh.write(f'(version 1)\n(rule "holes"\n  (constraint hole_clearance '
                         f'(min {rule}mm)))\n')
        with contextlib.redirect_stdout(io.StringIO()):
            _t, vias, _rm, _failed = generate_bga_fanout(
                pcb.footprints['U1'], pcb, net_filter=[n.name for n in pcb.nets.values()],
                layers=list(bgafix.CU), track_width=0.1, clearance=0.1, via_size=0.25,
                via_drill=0.15, exit_margin=0.5, escape_method='underpad', plane_drop='off')
        counts[tag] = len(vias)
    ok(counts['none'] >= 10, f"with no rule it drills vias: {counts['none']}")
    ok(counts['0.2'] == counts['none'],
       f"a 0.2 mm rule, which every site keeps, changes nothing: {counts['0.2']}")
    ok(counts['0.3'] == 0, f"a 0.3 mm rule, which no site under a ball keeps, drills none: "
                           f"{counts['0.3']}")


def tap(td):
    print("C. the plane tap")
    sub = os.path.join(td, 'c')
    board = stage(sub, 'newvia_tight')
    out = os.path.join(sub, 'out.kicad_pcb')
    r = check(PY + [os.path.join(REPO, 'py_router', 'route.py'), board, out, '--nets', 'SIG',
                    '--grid-step', '0.01', '--escalation', 'off', '--via-size', '0.6',
                    '--via-drill', '0.3', '--clearance', '0.0889', '--track-width', '0.1'],
              accept=True, timeout=900)
    log = (r.stdout or '') + (r.stderr or '')
    ok('tap of P1.1 refused:' in log and "hole_clearance 0.254mm (rule 'holes')" in log,
       "the tap of P1.1 is refused by the rule, by name")


def asked():
    print("D. every generator asks")
    for rel, names in (('bga_fanout/__init__.py', {'via_why'}),
                       ('bga_fanout/underpad.py', {'via_why'}),
                       ('qfn_fanout/__init__.py', {'via_why'}),
                       ('add_gnd_vias.py', {'via_why'}),
                       ('plane_pad_tap.py', {'hole_rule_refusal'})):
        path = evidence(os.path.join(REPO, 'py_router', rel), 'the module')
        with open(path, encoding='utf-8') as fh:
            tree = ast.parse(fh.read())
        called = {getattr(n.func, 'attr', getattr(n.func, 'id', None))
                  for n in ast.walk(tree) if isinstance(n, ast.Call)}
        ok(names <= called, f"{rel} calls {', '.join(sorted(names))}")


def own_copper(td):
    print("E. the gate grades the generator's own copper")
    from check_drc import HoleRuleGate
    from kicad_parser import parse_kicad_pcb
    pcb = parse_kicad_pcb(stage(os.path.join(td, 'e_fits'), 'newvia_fits'))
    sig = next(n for n in pcb.nets.values() if n.name == 'SIG').net_id
    other = next(n for n in pcb.nets.values() if n.name == 'OTHER').net_id
    g = HoleRuleGate(pcb)
    laid = []
    g.watch(vias=laid)
    ok(g.via_why(9.0, 6.0, 0.6, 0.3, sig) == '', "the site is allowed on the board as it is")
    # its copper 0.25 mm from the laid via's hole, 0.10 from the via's copper
    laid.append({'x': 9.70, 'y': 6.0, 'size': 0.6, 'drill': 0.3, 'net_id': other})
    why = g.via_why(9.0, 6.0, 0.6, 0.3, sig)
    ok("hole_clearance 0.254mm" in why, f"refused beside a via the generator laid: {why}")
    laid.clear()
    ok(g.via_why(9.0, 6.0, 0.6, 0.3, sig) == '', "allowed again once it is taken back")
    why = g.via_why(9.0, 6.0, 0.6, 0.3, sig, extra_vias=[
        {'x': 9.70, 'y': 6.0, 'size': 0.6, 'drill': 0.3, 'net_id': other}])
    ok("hole_clearance 0.254mm" in why, f"and beside a trial's own via: {why}")
    pcb = parse_kicad_pcb(stage(os.path.join(td, 'e_track'), 'track_fits'))
    sig = next(n for n in pcb.nets.values() if n.name == 'SIG').net_id
    g = HoleRuleGate(pcb)
    why = g.segment_why(15.45, 5.5, 15.45, 6.5, 0.1, 'F.Cu', sig)
    ok("hole_clearance 0.254mm" in why, f"a track 0.25 mm from a via's hole is refused: {why}")
    ok(g.segment_why(15.46, 5.5, 15.46, 6.5, 0.1, 'F.Cu', sig) == '',
       "one 0.26 mm from it is allowed")


def passes(td):
    print("F. the call's copper, graded once the engines are done")
    from fanout_hole_rule import hole_disclose
    from check_drc import board_design_rules, hole_rule_step_grade
    from kicad_parser import parse_kicad_pcb
    pcb = parse_kicad_pcb(stage(os.path.join(td, 'f_track'), 'track_fits'))
    sig = next(n for n in pcb.nets.values() if n.name == 'SIG').net_id
    for y, named in ((6.45, True), (6.47, False)):
        tracks = [{'start': (9.0, 6.45), 'end': (12.0, 6.45), 'width': 0.1, 'layer': 'F.Cu',
                   'net_id': sig},
                  {'start': (12.0, 6.45), 'end': (12.0, y), 'width': 0.1, 'layer': 'F.Cu',
                   'net_id': sig},
                  {'start': (12.0, y), 'end': (18.0, y), 'width': 0.1, 'layer': 'F.Cu',
                   'net_id': sig}]
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            grade = hole_disclose(pcb, tracks, [])
        if named:
            ok(len(grade) == 1 and grade[0]['rule'] == 'holes' and grade[0]['rows'] == 1
               and "'holes' (hole_clearance min 0.254 mm): 1 row(s)" in buf.getvalue(),
               f"a track 0.250 mm from the hole is kept and named: {grade}")
        else:
            ok(grade == [] and buf.getvalue() == '', "0.270 mm from it: nothing named")

    import bga_fanout
    import test_fanout_through_via_layers as bgafix
    from bga_fanout import generate_bga_fanout
    got = {}
    for tag, rule in (('none', None), ('rule', 0.25)):
        sub = os.path.join(td, 'f_bga_' + tag)
        os.makedirs(sub)
        pcb = bgafix.fixture('F')
        pcb.source_path = os.path.join(sub, 'b.kicad_pcb')
        with open(pcb.source_path, 'w', encoding='utf-8') as fh:
            fh.write('(kicad_pcb)\n')
        if rule:
            with open(os.path.join(sub, 'b.kicad_dru'), 'w', encoding='utf-8') as fh:
                fh.write(f'(version 1)\n(rule "holes"\n  (constraint hole_clearance '
                         f'(min {rule}mm)))\n')
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            t, v, _rm, failed = generate_bga_fanout(
                pcb.footprints['U1'], pcb, net_filter=[n.name for n in pcb.nets.values()],
                layers=list(bgafix.CU), track_width=0.1, clearance=0.1, via_size=0.25,
                via_drill=0.15, exit_margin=0.5, escape_method='channel', plane_drop='off')
        got[tag] = (pcb, t, v, set(failed), buf.getvalue(), list(bga_fanout.LAST_HOLE_RULES))
    import re
    pcb, t, v, failed, log, disclosed = got['rule']
    m = re.search(r"hole_clearance \(#1853\): (\d+) row\(s\) of this escape", log)
    ok(m is not None and int(m.group(1)) > 0,
       f"the channel escape stood inside the rule and was re-escaped: "
       f"{m.group(0) if m else 'no #1853 line'}")
    ok(failed <= got['none'][3], f"no ball the rule-free escape served is left bare: "
                                 f"{sorted(failed - got['none'][3])}")
    still = hole_rule_step_grade(pcb, board_design_rules(pcb), t, v)
    ok(disclosed == still, f"LAST_HOLE_RULES names exactly the rows left: {disclosed}")
    ok(got['none'][5] == [], "and nothing with no rule")


def main():
    with tempfile.TemporaryDirectory() as td:
        gate(td)
        bga(td)
        tap(td)
        own_copper(td)
        passes(td)
    asked()
    if FAILS:
        print(f"\nFAILED ({len(FAILS)}):")
        for f in FAILS:
            print(f"  - {f}")
        return 1
    print("\nALL PASSED")
    return 0


if __name__ == '__main__':
    sys.exit(main())
