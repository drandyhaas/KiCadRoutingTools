#!/usr/bin/env python3
"""A `.kicad_dru` hole_clearance rule scoped by a rule area binds by where
each item stands -- in check_drc, in the generators' hole gate and in the
route step.

The rule `(constraint hole_clearance (min 0.4mm))` on
`A.insideArea('TIGHT')` was read as unsupported: check_drc graded its pairs
at the board minimum and named it NOT GRADED, the gate the fan-out and the
other generators ask (`HoleRuleGate`, `hole_rule_step_rows`) passed over it,
and the route step held nothing at it -- while kicad-cli reports every pair
it binds. Since #1843 the rule areas are measured for clearance and width
rules (`DesignRules.area_index`); hole_clearance now reads the same index.

The fixture is tests/fixtures/1843's board (rule area `TIGHT` on F.Cu,
x 2-9, y 2-6; class clearance 0.2, min_hole_clearance 0.15), its tracks
replaced by pairs of a via (0.46/0.2) and another net's copper: a 0.1 mm
track 0.5 mm from the via (copper gap 0.22, hole-to-copper gap 0.35), or a
second via 0.7 mm away (`VV`). A rule of min 0.4 flags a pair it binds; the
board minimum flags none.

  IN   via and track inside the area
  VIN  via inside, track outside        TIN  via outside, track inside
  OUT  both outside                     BCU  via inside the outline, track
                                             on B.Cu (the area is on F.Cu)
  STR  a track crossing the area's edge, passing a via outside it
  VV   a via inside against a via outside

`KICAD` is the pairs kicad-cli 10.0.8 flags hole_clearance on
(`--all-track-errors`), per rule file -- the spec:
  * insideArea() is intersectsArea(), read on the layer the pair is asked
    on (BCU is not bound, the area being on F.Cu only); A and B read alike
    (a condition binds the pair either way round); enclosedByArea() wants
    the whole item inside, on any layer (STR is not bound, BCU is);
  * a through via pair is asked on every layer it shares: with a later
    rule that relaxes inside the area, VV is still flagged on B.Cu;
  * the last matching rule wins, both orders;
  * an area drawn on B.Cu binds B.Cu copper only;
  * a rule naming an area the board does not draw binds nothing
    (`missing_area`) -- check_drc names it NOT GRADED rather than reading
    its negation (`not_missing`) as matching every item.

Checks:
  1. check_drc flags exactly kicad-cli's pairs for each rule file, and names
     no rule NOT GRADED but the missing area's;
  2. the gate refuses a via or track inside the rule where the rule binds,
     and the same copper where it does not bind is laid; the step's rows
     (`hole_rule_step_rows`, the fan-out's) carry the rule;
  3. the route step's walk (`RoutedHoleRules`) holds a hole inside the area
     at the rule, a hole outside it within reach of the area tighten-safe,
     and a hole far from every area not at all (negative control: an area
     drawn away from all the copper binds nothing);
  4. a routed board: splitflap_driver with a rule area over five connectors
     and the rule `A.insideArea('ROOM') || B.insideArea('ROOM')` 0.5 mm,
     routed by the parity gate's first route step: every net routes and
     check_drc reports no hole row (69 on main, which kicad-cli confirms);
  5. kicad-cli's own report of the boards of part 1 (exit 77 without it,
     after parts 1-4).

    python3 tests/test_hole_rule_area.py
"""
import contextlib
import io
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)
from run_utils import check, evidence  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
BASE = os.path.join(TESTS, 'fixtures', '1843', 'area_rule')
SPLITFLAP = os.path.join(REPO, 'kicad_files', 'splitflap_driver.kicad_pcb')
FAILS = []


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def _seg(x1, y1, x2, y2, net, layer='F.Cu', w=0.1):
    return (f'\t(segment (start {x1} {y1}) (end {x2} {y2}) (width {w}) '
            f'(layer "{layer}") (net "{net}"))\n')


def _via(x, y, net):
    return (f'\t(via (at {x} {y}) (size 0.46) (drill 0.2) (layers "F.Cu" "B.Cu") '
            f'(net "{net}"))\n')


PAIRS = (_via(4, 3, 'V_IN') + _seg(3, 3.5, 5, 3.5, 'T_IN')
         + _via(8.7, 4, 'V_VIN') + _seg(9.2, 3.2, 9.2, 4.8, 'T_VIN')
         + _via(9.3, 5.6, 'V_TIN') + _seg(8.8, 5.3, 8.8, 5.9, 'T_TIN')
         + _via(14, 3, 'V_OUT') + _seg(13, 3.5, 15, 3.5, 'T_OUT')
         + _via(4, 5, 'V_BCU') + _seg(3, 5.5, 5, 5.5, 'T_BCU', 'B.Cu')
         + _via(6.5, 7.5, 'V_STR') + _seg(6, 5, 6, 8, 'T_STR')
         + _via(8.6, 2.6, 'VA_VV') + _via(9.3, 2.6, 'VB_VV'))
FAR = ('FAR', '(xy 16 6) (xy 19 6) (xy 19 9) (xy 16 9)')


def rule(name, cond, mn):
    c = f'  (condition "{cond}")\n' if cond else ''
    return f'(rule "{name}"\n  (constraint hole_clearance (min {mn}mm))\n{c})\n'


IN = "A.insideArea('TIGHT')"
ALL_IN = {'IN', 'STR', 'TIN', 'VIN', 'VV'}
#: name -> (rule file, staging, the pairs kicad-cli 10.0.8 flags)
KICAD = {
    'A_inside': (rule('room', IN, 0.4), {}, ALL_IN),
    'B_inside': (rule('room', "B.insideArea('TIGHT')", 0.4), {}, ALL_IN),
    'A_intersects': (rule('room', "A.intersectsArea('TIGHT')", 0.4), {}, ALL_IN),
    'A_or_B': (rule('room', f"{IN} || B.insideArea('TIGHT')", 0.4), {}, ALL_IN),
    'A_and_B': (rule('room', f"{IN} && B.insideArea('TIGHT')", 0.4), {}, {'IN'}),
    'enclosed': (rule('room', "A.enclosedByArea('TIGHT')", 0.4), {},
                 {'BCU', 'IN', 'TIN', 'VIN', 'VV'}),
    'area_then_wide': (rule('room', IN, 0.4) + rule('wide', None, 0.1), {}, set()),
    'wide_then_area': (rule('wide', None, 0.1) + rule('room', IN, 0.4), {}, ALL_IN),
    'relax_then_wide': (rule('room', IN, 0.1) + rule('wide', None, 0.4), {},
                        ALL_IN | {'BCU', 'OUT'}),
    'wide_then_relax': (rule('wide', None, 0.4) + rule('room', IN, 0.1), {},
                        {'BCU', 'OUT', 'VV'}),
    'area_on_bcu': (rule('room', IN, 0.4), {'zone_layer': 'B.Cu'}, {'BCU', 'VV'}),
    'far_area': (rule('room', "A.insideArea('FAR')", 0.4), {'extra_zone': FAR}, set()),
    'via_and_in': (rule('room', f"A.Type == 'Via' && {IN}", 0.4), {},
                   {'IN', 'VIN', 'VV'}),
    'track_and_in': (rule('room', f"A.Type == 'Track' && {IN}", 0.4), {},
                     {'IN', 'STR', 'TIN'}),
    'not_inside': (rule('room', f"!{IN}", 0.4), {},
                   {'BCU', 'OUT', 'STR', 'TIN', 'VIN', 'VV'}),
    'missing_area': (rule('room', "A.insideArea('NOPE')", 0.4), {}, set()),
    'not_missing': (rule('room', "!A.insideArea('NOPE')", 0.4), {},
                    ALL_IN | {'BCU', 'OUT'}),
}
#: the rule files check_drc names NOT GRADED (their area is not drawn): it
#: grades their pairs at the board minimum, so flags none
NAMED = {'missing_area', 'not_missing'}


def board_text(zone_layer='F.Cu', extra_zone=None):
    with open(evidence(BASE + '.kicad_pcb'), encoding='utf-8') as fh:
        t = fh.read()
    t = re.sub(r'\t\(segment\n.*?\n\t\)\n', '', t, flags=re.S)
    if '(segment' in t or '(name "TIGHT")' not in t:
        raise AssertionError("BROKEN TEST: the 1843 fixture changed shape")
    if zone_layer != 'F.Cu':
        t = t.replace('\t(zone\n\t\t(layer "F.Cu")', f'\t(zone\n\t\t(layer "{zone_layer}")')
    if extra_zone:
        k0, k1 = t.index('\t(zone'), t.index('\t(embedded_fonts')
        z = (t[k0:k1].replace('(name "TIGHT")', f'(name "{extra_zone[0]}")')
             .replace('(xy 2 2) (xy 9 2) (xy 9 6) (xy 2 6)', extra_zone[1])
             .replace('40b3d710-7d00-43ca-8803-b4b5e6f64abb',
                      '40b3d710-7d00-43ca-8803-b4b5e6f64abc'))
        t = t[:k1] + z + t[k1:]
    k = t.index('\t(zone')
    return t[:k] + PAIRS + t[k:]


def stage(td, name, rules, **kw):
    b = os.path.join(td, name + '.kicad_pcb')
    with open(b, 'w', encoding='utf-8') as fh:
        fh.write(board_text(**kw))
    shutil.copyfile(evidence(BASE + '.kicad_pro'), os.path.join(td, name + '.kicad_pro'))
    with open(os.path.join(td, name + '.kicad_dru'), 'w', encoding='utf-8') as fh:
        fh.write('(version 1)\n' + rules)
    return evidence(b, 'the staged board')


def _pair(n1, n2):
    """The pair's name: the two nets' common suffix (V_IN / T_IN -> IN)."""
    return '+'.join(sorted({n.split('_', 1)[-1] for n in (n1, n2)}))


def grade(path):
    out = path[:-len('.kicad_pcb')] + '.drc.json'
    # exit 1 is a board with violations, which some rows want
    r = subprocess.run(PY + [os.path.join(REPO, 'py_router', 'check_drc.py'), path,
                             '--json', out],
                       capture_output=True, text=True, encoding='utf-8', errors='replace')
    text = (r.stdout or '') + (r.stderr or '')
    if 'Traceback' in text or r.returncode not in (0, 1):
        raise AssertionError(f"BROKEN TEST: check_drc died\n{text[-1500:]}")
    with open(evidence(out, 'the check_drc report'), encoding='utf-8') as fh:
        doc = json.load(fh)
    rows = {_pair(v['net1'], v['net2']) for v in doc['items']
            if v['type'].endswith('-hole')}
    return rows, doc, text


def grader(boards):
    print("1. check_drc flags kicad-cli's pairs")
    for name, (_rules, _kw, want) in KICAD.items():
        if name in NAMED:
            want = set()
        got, doc, text = grade(boards[name])
        ok(got == want, f"{name}: {sorted(got)}" + ('' if got == want else
                                                   f" (kicad-cli: {sorted(want)})"))
        named = 'NOT GRADED (hole_clearance)' in text
        ok(named == (name in NAMED),
           f"   rule named NOT GRADED: {named}" + (" (its area is not drawn)"
                                                  if name in NAMED else ''))
        if name == 'A_inside':
            ok("Area-scoped hole-clearance rules" in text and "'room':0.4" in text,
               "   the run says the rule is graded by position")
            rows = [v for v in doc['items'] if v['type'].endswith('-hole')]
            ok(rows and all(v.get('rule') == 'room' and abs(v['required_mm'] - 0.4) < 1e-9
                            for v in rows),
               f"   every row is graded at rule 'room' 0.4 ({len(rows)} rows)")


def _load(path):
    from kicad_parser import parse_kicad_pcb
    from routing_config import GridRouteConfig
    from kicad_dru import install_layer_clearances
    pcb = parse_kicad_pcb(path)
    cfg = GridRouteConfig(layers=list(pcb.board_info.copper_layers),
                          via_size=0.46, via_drill=0.2, track_width=0.1)
    with contextlib.redirect_stdout(io.StringIO()):
        install_layer_clearances(cfg, None, None, pcb)
    nid = {n.name: i for i, n in pcb.nets.items() if n.name}
    return pcb, cfg, nid


def gate(boards):
    print("2. the generators' gate and the step's rows")
    from check_drc import HoleRuleGate, hole_rule_step_rows
    pcb, cfg, nid = _load(boards['A_inside'])
    g = HoleRuleGate(pcb, cfg.rules)
    ok(bool(g), "the gate is live on the board")
    # a via of another net 0.7 mm from a via's centre: hole gap 0.37
    why = g.via_why(4, 3.7, 0.46, 0.2, nid['T_OUT'])
    ok("rule 'room'" in why, f"a via beside the hole inside the area is refused: {why!r}")
    why = g.via_why(14, 3.7, 0.46, 0.2, nid['T_IN'])
    ok(why == '', f"the same via beside the hole outside it is laid: {why!r}")
    # a track 0.5 mm from the via: copper gap 0.22, hole gap 0.35
    why = g.segment_why(3, 2.5, 5, 2.5, 0.1, 'F.Cu', nid['T_OUT'])
    ok("rule 'room'" in why, f"a track beside the hole inside the area is refused: {why!r}")
    why = g.segment_why(13, 2.5, 15, 2.5, 0.1, 'F.Cu', nid['T_IN'])
    ok(why == '', f"the same track outside it is laid: {why!r}")
    from kicad_parser import Segment
    mine = [Segment(start_x=3, start_y=2.5, end_x=5, end_y=2.5, width=0.1, layer='F.Cu',
                    net_id=nid['T_OUT']),
            Segment(start_x=13, start_y=2.5, end_x=15, end_y=2.5, width=0.1, layer='F.Cu',
                    net_id=nid['T_IN'])]
    rows = hole_rule_step_rows(pcb, cfg.rules, mine, [])
    got = sorted((r.get('rule'), c) for r, c, _h in rows)
    ok(got == [('room', ('t', 0))], f"the step's own rows: {got} (its track inside only)")
    pcb, cfg, nid = _load(boards['far_area'])
    g = HoleRuleGate(pcb, cfg.rules)
    why = g.via_why(4, 3.7, 0.46, 0.2, nid['T_OUT'])
    ok(why == '', f"negative control, a rule on an area away from the copper: {why!r}")


def walk(boards):
    print("3. the route step's walk")
    pcb, cfg, nid = _load(boards['A_inside'])
    hr = cfg.hole_rules
    ok(hr is not None and hr.positional, "the route step reads the rule by position")
    if hr is None:
        return
    vias = {pcb.nets[v.net_id].name: v for v in pcb.vias}
    for name, want in (('V_IN', 0.4), ('VA_VV', 0.4), ('V_TIN', 0.4), ('V_OUT', None),
                       ('V_STR', None)):
        got = hr.via_value(vias[name], 'F.Cu')
        why = {'V_IN': 'inside', 'VA_VV': 'inside', 'V_TIN': 'outside, within reach (tighten-safe)',
               'V_OUT': 'far from the area', 'V_STR': 'outside, beyond reach'}[name]
        ok(got == want, f"{name} ({why}) on F.Cu: {got}")
    got = hr.via_value(vias['V_IN'], 'B.Cu')
    ok(got is None, f"V_IN on B.Cu, where the area is not drawn: {got}")
    pcb, cfg, nid = _load(boards['A_and_B'])
    got = cfg.hole_rules.via_value({pcb.nets[v.net_id].name: v for v in pcb.vias}['V_TIN'],
                                   'F.Cu')
    ok(got is None, f"A && B: a hole outside the area is never bound: {got}")
    pcb, cfg, nid = _load(boards['far_area'])
    vals = {cfg.hole_rules.via_value(v, 'F.Cu') for v in pcb.vias}
    ok(vals == {None}, f"negative control, an area away from every hole: {vals}")
    pcb, cfg, nid = _load(boards['wide_then_relax'])
    vias = {pcb.nets[v.net_id].name: v for v in pcb.vias}
    got = (cfg.hole_rules.via_value(vias['V_IN'], 'F.Cu'),
           cfg.hole_rules.via_value(vias['V_OUT'], 'F.Cu'))
    ok(got == (0.1, 0.4), f"a later rule relaxing inside the area: in {got[0]}, out {got[1]}")


def routed(td):
    print("4. a routed board")
    with open(evidence(SPLITFLAP), encoding='utf-8') as fh:
        t = fh.read()
    zone = ('\t(zone (net 0) (net_name "") (layers "F.Cu" "B.Cu") '
            '(uuid "11111111-2222-3333-4444-555555555555") (name "ROOM") (hatch edge 0.5)\n'
            '\t\t(connect_pads (clearance 0)) (min_thickness 0.25)\n'
            '\t\t(keepout (tracks allowed) (vias allowed) (pads allowed) '
            '(copperpour allowed) (footprints allowed))\n'
            '\t\t(fill (thermal_gap 0.5) (thermal_bridge_width 0.5))\n'
            '\t\t(polygon (pts (xy 45 25.4) (xy 95 25.4) (xy 95 55.88) (xy 45 55.88)))\n\t)\n')
    k = t.rindex(')')
    src = os.path.join(td, 'room.kicad_pcb')
    with open(src, 'w', encoding='utf-8') as fh:
        fh.write(t[:k] + zone + t[k:])
    with open(os.path.join(td, 'room.kicad_dru'), 'w', encoding='utf-8') as fh:
        fh.write('(version 1)\n' + rule('room hole',
                                        "A.insideArea('ROOM') || B.insideArea('ROOM')", 0.5))
    out = os.path.join(td, 'room_routed.kicad_pcb')
    r = check(PY + [os.path.join(REPO, 'py_router', 'route.py'), src, out,
                    '--nets', '*', '!GND', '--clearance', '0.15', '--track-width', '0.127',
                    '--via-size', '0.45', '--via-drill', '0.2', '--power-nets', '+3V3',
                    '+12V', '--power-nets-widths', '0.4', '0.4', '--max-ripup', '10',
                    '--max-iterations', '1000000'], accept=True, cwd=td)
    m = re.search(r'^JSON_SUMMARY_MIN: (\{.*\})$', r.stdout, re.M)
    doc = json.loads(m.group(1)) if m else {}
    ok(doc.get('routed') == 82 and doc.get('failed') == 0 and not doc.get('open_single'),
       f"every net routes: routed {doc.get('routed')}, failed {doc.get('failed')}")
    ok("'room hole': hole_clearance min 0.5 mm: routed" in r.stdout,
       "the route step names the rule honoured")
    rows, _doc, text = grade(evidence(out, 'the routed board'))
    ok(not rows, f"check_drc reports no copper-to-hole row ({len(rows)} pairs)")
    # non-vacuity: the rule reaches the board -- an area hole, graded at it
    ok("Area-scoped hole-clearance rules" in text and "'room hole':0.5" in text,
       "   and grades the rule by position on it")


def kicad(boards, cli):
    print("5. kicad-cli's own report of the boards (--all-track-errors)")
    for name, (_rules, _kw, want) in KICAD.items():
        rep = boards[name][:-len('.kicad_pcb')] + '.kicad.json'
        subprocess.run([cli, 'pcb', 'drc', '--format', 'json', '--severity-all',
                        '--all-track-errors', '-o', rep, boards[name]],
                       capture_output=True, text=True, timeout=600)
        with open(evidence(rep, 'kicad-cli DRC report'), encoding='utf-8') as fh:
            rep = json.load(fh)
        got = set()
        for v in rep['violations']:
            if v['type'] != 'hole_clearance':
                continue
            nets = [m.group(1) for m in (re.search(r'\[([^\]]*)\]', i.get('description', ''))
                                         for i in v['items']) if m]
            got.add(_pair(*nets) if len(nets) == 2 else '?')
        ok(got == want, f"kicad-cli {rep.get('kicad_version')}, {name}: {sorted(got)}"
           + ('' if got == want else f" (the table: {sorted(want)})"))


def main():
    td = tempfile.mkdtemp(prefix='krt_hole_area_')
    skipped = False
    try:
        boards = {name: stage(td, name, spec[0], **spec[1]) for name, spec in KICAD.items()}
        grader(boards)
        gate(boards)
        walk(boards)
        routed(td)
        with contextlib.redirect_stdout(io.StringIO()):
            from kicad_oracle import find_kicad_cli
            cli = find_kicad_cli(warn=False)
        if cli:
            kicad(boards, cli)
        else:
            print("5. SKIPPED: no kicad-cli (set KICAD_CLI)")
            skipped = True
    except AssertionError as exc:
        ok(False, f"a run_utils assertion held: {str(exc)[:1500]}")
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if FAILS:
        print(f"\nFAILED: {len(FAILS)}")
        return 1
    if skipped:
        print("\nPARTS 1-4 PASS")
        print("SKIP: part 5 needs kicad-cli (none found; set KICAD_CLI) -- "
              "KiCad's own grade of the boards was not checked")
        return 77
    print("\nALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
