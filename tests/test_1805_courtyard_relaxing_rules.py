#!/usr/bin/env python3
"""#1805: a `.kicad_dru` via rule scoped by a courtyard term is listed, its nets
named, and check_drc grades a via under it by where the via stands.

A project may allow a smaller via ring inside one part's courtyard only:
`(condition "A.NetName == '/WR' && A.insideCourtyard('U1')")`. The rule was
read as unsupported, so `list_nets.py --design-rules` left it out of its
RELAX list without a word, and check_drc graded every via at the Board Setup
ring -- flagging the via on U1's pad that KiCad passes.

Checks (no routing):
  1. `relaxing_rules` lists all three rules of the issue's file; the two
     courtyard rules name U1 and are not marked unresolved, and a rule with
     an unread term is listed as unresolved;
  2. `relaxed_via_nets`: the net-and-courtyard rule binds exactly /WR, the
     class-and-courtyard rule only nets with a pad on U1, the net rule /RD;
     the same nets when each rule also has a `(layer outer)` clause, which
     `list_nets.py` prints on the rule line, and no net under `(layer
     "B.Cu")`, which binds no via for an `annular_width` rule (#1805's
     via-clause reading, measured on kicad-cli 10.0.6 and 10.0.7), and the
     floor `via_floors` gives /RD follows the clause the same way;
  3. `list_nets.py --design-rules` prints the three rules, marks the two
     bound by position, and names the nets; the net-name rule's line is as
     before (#1649); a rule whose courtyard term is NEGATED, or ORed with a
     net term, is not marked "inside the courtyard", and its net is listed
     as one it may bind;
  4. check_drc on two /WR vias with a 0.07 ring flags the one at (100, 80)
     and not the one on pad U1.2 -- what kicad-cli 10.0.6 reports on the
     same board -- and prints no NOT GRADED line for the courtyard rule;
  5. `CourtyardIndex` holds a via to KiCad's reading of a courtyard term,
     `KICAD` below, measured on kicad-cli 10.0.6 on `PROBE_BOARD`: the
     disc must OVERLAP the courtyard (a tangent one does not), and
     front/back are the part's own side and the other one.
"""
import contextlib
import io
import json
import os
import re
import shutil
import sys
import tempfile
from types import SimpleNamespace

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)
from run_utils import check, evidence  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402
from design_rules import (DesignRules, relaxing_rules, relaxed_via_nets,  # noqa: E402
                          CourtyardIndex)

PY = [sys.executable, '-X', 'utf8']
BOARD = os.path.join(REPO, 'kicad_files', 'glasgow_revC.kicad_pcb')
DRU = '''(version 1)
(rule "ring_by_net"
  (condition "A.NetName == '/RD'")
  (constraint annular_width (min 0.07mm)))
(rule "ring_by_class_in_courtyard"
  (condition "A.NetClass == 'Default' && A.insideCourtyard('U1')")
  (constraint annular_width (min 0.07mm)))
(rule "ring_by_net_in_courtyard"
  (condition "A.NetName == '/WR' && A.insideCourtyard('U1')")
  (constraint annular_width (min 0.07mm)))
'''
UNREAD = '''(rule "ring_unread"
  (condition "A.NetName == '/OE' && A.getField('x') == 'y'")
  (constraint annular_width (min 0.07mm)))
'''
NEGATED = '''(version 1)
(rule "ring_outside_u1"
  (condition "A.NetName == '/WR' && !A.insideCourtyard('U1')")
  (constraint annular_width (min 0.07mm)))
(rule "ring_not_rd_in_u1"
  (condition "!(A.NetName == '/RD') && A.insideCourtyard('U1')")
  (constraint annular_width (min 0.07mm)))
(rule "ring_u1_or_rd"
  (condition "A.insideCourtyard('U1') || A.NetName == '/RD'")
  (constraint annular_width (min 0.07mm)))
'''
VIAS = ((65.85, 92.75), (100, 80))      # on pad U1.2, and well outside U1

FAILS = []


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def stage(td, name, dru, vias=()):
    b = os.path.join(td, name + '.kicad_pcb')
    check(PY + [os.path.join(REPO, 'py_router', 'copy_board.py'), BOARD, b], accept=True)
    with open(os.path.join(td, name + '.kicad_dru'), 'w', encoding='utf-8') as fh:
        fh.write(dru)
    if vias:
        with open(b, encoding='utf-8') as fh:
            t = fh.read()
        k = t.rindex(')')
        t = t[:k] + ''.join(f'\t(via (at {x} {y}) (size 0.3) (drill 0.16) (layers "F.Cu" "B.Cu") '
                            f'(net "/WR"))\n' for x, y in vias) + t[k:]
        with open(b, 'w', encoding='utf-8') as fh:
            fh.write(t)
    return evidence(b, 'the staged board')


def load(path):
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        pcb = parse_kicad_pcb(path)
        return pcb, DesignRules.from_project(pcb, path)


def engine(td):
    print("1-2. relaxing_rules and relaxed_via_nets")
    path = stage(td, 'e', DRU + UNREAD)
    pcb, dr = load(path)
    rows = relaxing_rules(dr.rules, dr.board_min)
    by = {r['rule']: r for r in rows}
    ok(sorted(by) == ['ring_by_class_in_courtyard', 'ring_by_net', 'ring_by_net_in_courtyard',
                      'ring_unread'], f"every relaxing rule listed: {sorted(by)}")
    for n in ('ring_by_class_in_courtyard', 'ring_by_net_in_courtyard'):
        ok(by.get(n, {}).get('courtyard') == ['U1'] and by[n]['unresolved'] is None,
           f"{n} bound by position inside U1: {by.get(n)}")
    ok(by.get('ring_by_net', {}).get('courtyard') == [] and by['ring_by_net']['unresolved'] is None,
       "the net-name rule is read exactly")
    ok('getField' in (by.get('ring_unread', {}).get('unresolved') or ''),
       f"the unread rule says why: {by.get('ring_unread', {}).get('unresolved')}")
    nets = relaxed_via_nets(dr, pcb, rows)
    ok(nets['ring_by_net'] == {'nets': ['/RD'], 'how': 'condition'}, f"/RD: {nets['ring_by_net']}")
    ok(nets['ring_by_net_in_courtyard'] == {'nets': ['/WR'], 'how': 'courtyard'},
       f"/WR: {nets['ring_by_net_in_courtyard']}")
    on_u1 = {p.net_name for p in pcb.footprints['U1'].pads if p.net_id}
    cls = nets['ring_by_class_in_courtyard']
    ok(cls['how'] == 'courtyard' and cls['nets'] and set(cls['nets']) <= on_u1
       and '/WR' in cls['nets'], f"class rule: {len(cls['nets'])} net(s), all on U1")
    ok(nets['ring_unread'] == {'nets': ['/OE'], 'how': 'may'}, f"unread: {nets['ring_unread']}")
    ok(dr.via_floors(next(i for i, n in pcb.nets.items() if n.name == '/RD')) == {'annular': 0.07},
       "control: /RD keeps its rule floor")
    # the same rules with a (layer outer) clause: a net has no layer, so the
    # clause is the row's `layer` and the nets are still the condition's
    lpath = stage(td, 'el', DRU.replace('  (condition', '  (layer outer)\n  (condition'))
    lpcb, ldr = load(lpath)
    lrows = relaxing_rules(ldr.rules, ldr.board_min)
    ok(len(lrows) == 3 and all(r['layer'] == 'outer' for r in lrows),
       f"three rules with a layer clause listed: {[(r['rule'], r['layer']) for r in lrows]}")
    lnets = relaxed_via_nets(ldr, lpcb, lrows)
    ok(lnets.get('ring_by_net') == {'nets': ['/RD'], 'how': 'condition'},
       f"(layer outer) /RD: {lnets.get('ring_by_net')}")
    ok(lnets.get('ring_by_net_in_courtyard') == {'nets': ['/WR'], 'how': 'courtyard'},
       f"(layer outer) /WR: {lnets.get('ring_by_net_in_courtyard')}")
    out = check(PY + [os.path.join(REPO, 'py_router', 'list_nets.py'), lpath, '--design-rules'],
                accept=True).stdout
    ok("  - 'ring_by_net': annular_width min 0.07 (board min_via_annular_width 0.1) on outer "
       "if A.NetName == '/RD'\n" in out, "the rule line names the layer clause")
    ok("  - 'ring_by_net' (annular_width): 1 net(s): /RD\n" in out,
       "list_nets names the net of a rule with a layer clause")
    ok('): 0 net(s)' not in out, "no rule of the three is printed with no net")
    rd = next(i for i, n in lpcb.nets.items() if n.name == '/RD')
    ok(ldr.via_floors(rd) == {'annular': 0.07},
       f"(layer outer): /RD's via floor is the rule's, as KiCad binds it: {ldr.via_floors(rd)}")
    # (layer "B.Cu") on a ring rule: KiCad tests the clause against F.Cu for a
    # via, so the rule binds no via -- no net is named, no floor is set
    bpath = stage(td, 'eb', DRU.replace('  (condition', '  (layer "B.Cu")\n  (condition'))
    bpcb, bdr = load(bpath)
    brows = relaxing_rules(bdr.rules, bdr.board_min)
    bnets = relaxed_via_nets(bdr, bpcb, brows)
    ok(len(brows) == 3 and all(bnets.get(r['rule'], {}).get('nets') == [] for r in brows),
       f"(layer \"B.Cu\"): no rule names a net: {bnets}")
    brd = next(i for i, n in bpcb.nets.items() if n.name == '/RD')
    ok(bdr.via_floors(brd) == {}, f"(layer \"B.Cu\"): /RD's via has no rule floor: "
                                   f"{bdr.via_floors(brd)}")


def listing(td):
    print("3. list_nets.py --design-rules")
    path = stage(td, 'l', DRU)
    r = check(PY + [os.path.join(REPO, 'py_router', 'list_nets.py'), path, '--design-rules'],
              accept=True)
    out = r.stdout
    listed = re.findall(r"^  - '([^']+)': annular_width min", out, re.M)
    ok(listed == ['ring_by_net', 'ring_by_class_in_courtyard', 'ring_by_net_in_courtyard'],
       f"three rules listed: {listed}")
    ok(out.count('bound by POSITION: inside the courtyard of U1 only') == 2,
       "the two courtyard rules marked bound by position")
    ok("  - 'ring_by_net': annular_width min 0.07 (board min_via_annular_width 0.1) "
       "if A.NetName == '/RD'\n" in out, "control: the net-name rule's line is unchanged")
    ok(re.search(r"^Nets with a relaxed via rule \(\d+\)", out, re.M) is not None,
       "the nets line is printed")
    ok("'ring_by_net_in_courtyard' (annular_width): 1 net(s); its via inside the courtyard "
       "of U1 only" in out and out.rstrip().find('/WR') > 0, "the courtyard rule's net is named")
    bare = stage(td, 'n', '')
    os.remove(os.path.join(td, 'n.kicad_dru'))
    r = check(PY + [os.path.join(REPO, 'py_router', 'list_nets.py'), bare, '--design-rules'],
              accept=True)
    ok('RELAX' not in r.stdout and 'relaxed via rule' not in r.stdout,
       "control: no rules file, no RELAX block")
    # a NEGATED courtyard term binds a via OUTSIDE the part: the rule is never
    # listed as bound inside it, and its net is still named
    neg = stage(td, 'g', NEGATED)
    pcb, dr = load(neg)
    rows = relaxing_rules(dr.rules, dr.board_min)
    by = {x['rule']: x for x in rows}
    row = by.get('ring_outside_u1', {})
    ok(row.get('courtyard') == [] and 'negated courtyard term' in (row.get('unresolved') or ''),
       f"the negated rule names no courtyard and says why: {row.get('courtyard')}, "
       f"{row.get('unresolved')}")
    ok(by.get('ring_not_rd_in_u1', {}).get('courtyard') == ['U1']
       and by['ring_not_rd_in_u1']['unresolved'] is None,
       "control: a negation on another term leaves the rule bound inside U1")
    orow = by.get('ring_u1_or_rd', {})
    ok(orow.get('courtyard') == [] and '(||)' in (orow.get('unresolved') or ''),
       f"a courtyard term ORed with a net term names no courtyard and says why: "
       f"{orow.get('courtyard')}, {orow.get('unresolved')}")
    nets = relaxed_via_nets(dr, pcb, rows)
    ok(nets.get('ring_outside_u1') == {'nets': ['/WR'], 'how': 'may'},
       f"the negated rule's net: {nets.get('ring_outside_u1')}")
    onets = nets.get('ring_u1_or_rd') or {}
    ok(onets.get('how') == 'may' and '/RD' in onets.get('nets', ()),
       f"the ORed rule may bind /RD's via anywhere: {onets.get('how')}, "
       f"{len(onets.get('nets', ()))} net(s)")
    out = check(PY + [os.path.join(REPO, 'py_router', 'list_nets.py'), neg, '--design-rules'],
                accept=True).stdout
    line = next((x for x in out.splitlines() if x.startswith("  - 'ring_outside_u1': ")), '')
    ok('inside the courtyard of' not in line and 'a negated courtyard term' in line
       and line.endswith('it MAY relax'), f"its rule line: ...{line[-95:]}")
    ok("  - 'ring_outside_u1' (annular_width): 1 net(s); the condition is not read, so these "
       "are the nets it MAY bind: /WR\n" in out, "its nets line names /WR as a net it may bind")
    ok(out.count('bound by POSITION: inside the courtyard of U1 only') == 1,
       "control: the other rule is the one line marked inside U1")
    oline = next((x for x in out.splitlines() if x.startswith("  - 'ring_u1_or_rd': ")), '')
    ok('inside the courtyard of' not in oline and oline.endswith('it MAY relax'),
       f"the ORed rule's line: ...{oline[-95:]}")
    ok(re.search(r"^  - 'ring_u1_or_rd' \(annular_width\): \d+ net\(s\); the condition is "
                 r"not read, so these are the nets it MAY bind: ", out, re.M) is not None,
       "the ORed rule's nets line lists the nets it may bind")


def grade(td):
    print("4. check_drc grades the via by where it stands")
    path = stage(td, 'v', DRU, VIAS)
    r = check(PY + [os.path.join(REPO, 'py_router', 'check_drc.py'), path],
              refuse='VIA-ANNULAR violations (1)')
    block = r.stdout.split('VIA-ANNULAR violations (1)', 1)[1].split('\n\n', 1)[0]
    ok('(100.00,80.00)' in block, "the via outside U1's courtyard is flagged")
    ok('(65.85,92.75)' not in block, "the via on pad U1.2, which KiCad passes, is not")
    ok('NOT GRADED (size): rule \'ring_by_net_in_courtyard\'' not in r.stdout,
       "no NOT GRADED line for a rule graded by position")


#: A part U1 on F.Cu at (100,100) and U2 on B.Cu at (120,100), each with a
#: 4x4 mm courtyard on its own side and a second 4x4 one on the other side
#: 5 mm lower; U3 (F.Cu) and U4 (B.Cu) turned 30 degrees, each with a 4x1 mm
#: courtyard off its origin. One via (0.3 / 0.16) per entry below.
PROBE_VIAS = {
    'u1_centre': (100.0, 100.0), 'u1_edge_in': (101.9, 100.0), 'u1_overlap': (102.1, 100.0),
    'u1_touch': (102.15, 100.0), 'u1_clear': (102.2, 100.0), 'u1_far': (110.0, 100.0),
    'u2_centre': (120.0, 100.0), 'u2_corner': (121.9, 101.9),
    'u1_other': (100.0, 105.0), 'u2_other': (120.0, 105.0),
    # 2.5 mm out along the long side of a part turned 30 degrees, either way
    'u3_ccw': (102.165, 91.75), 'u3_cw': (102.165, 94.25),
    'u4_ccw': (122.165, 91.75), 'u4_cw': (122.165, 94.25),
}


def _fp(ref, x, y, side, rot=0, rect=(-2, -2, 2, 2), other_side=True):
    other = 'B' if side == 'F' else 'F'
    n = ref[-1]
    x0, y0, x1, y1 = rect
    return (f'(footprint "lib:{ref}_fp" (layer "{side}.Cu") (uuid "f0000000-0000-0000-0000-0000000000{n}0")\n'
            f'  (at {x} {y} {rot})\n'
            f'  (property "Reference" "{ref}" (at 0 -3 0) (layer "{side}.SilkS") (uuid "f1000000-0000-0000-0000-0000000000{n}0") (effects (font (size 1 1) (thickness 0.15))))\n'
            f'  (fp_rect (start {x0} {y0}) (end {x1} {y1}) (stroke (width 0.05) (type solid)) (fill no) (layer "{side}.CrtYd") (uuid "f2000000-0000-0000-0000-0000000000{n}0"))\n'
            + (f'  (fp_rect (start -2 3) (end 2 7) (stroke (width 0.05) (type solid)) (fill no) (layer "{other}.CrtYd") (uuid "f4000000-0000-0000-0000-0000000000{n}0"))\n'
               if other_side else '') +
            f'  (pad "1" smd rect (at 0 -1.5) (size 0.5 0.5) (layers "{side}.Cu") (net 1 "N") (uuid "f3000000-0000-0000-0000-0000000000{n}0")))\n')


PROBE_BOARD = ('(kicad_pcb (version 20241229) (generator "pcbnew") (generator_version "9.0")\n'
               ' (general (thickness 1.6) (legacy_teardrops no))\n (paper "A4")\n'
               ' (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user) (31 "F.CrtYd" user "F.Courtyard") (29 "B.CrtYd" user "B.Courtyard") (5 "F.SilkS" user "F.Silkscreen") (7 "B.SilkS" user "B.Silkscreen"))\n'
               ' (setup (pad_to_mask_clearance 0))\n (net 0 "") (net 1 "N")\n'
               ' (gr_rect (start 90 90) (end 130 110) (stroke (width 0.05) (type default)) (fill no) (layer "Edge.Cuts") (uuid "e0000000-0000-0000-0000-000000000001"))\n'
               + _fp('U1', 100, 100, 'F') + _fp('U2', 120, 100, 'B')
               + _fp('U3', 100, 93, 'F', 30, (-1, -0.5, 3, 0.5), False)
               + _fp('U4', 120, 93, 'B', 30, (-1, -0.5, 3, 0.5), False)
               + ''.join(f' (via (at {x} {y}) (size 0.3) (drill 0.16) (layers "F.Cu" "B.Cu") (net 1) '
                         f'(uuid "c0000000-0000-0000-0000-0000000000{i:02d}"))\n'
                         for i, (x, y) in enumerate(PROBE_VIAS.values()))
               + ')\n')
#: {condition: the vias a 0.07 ring rule binds}, kicad-cli 10.0.6 (Board Setup 0.1)
KICAD = {
    "A.insideCourtyard('U1')": {'u1_centre', 'u1_edge_in', 'u1_overlap', 'u1_other'},
    "A.intersectsCourtyard('U1')": {'u1_centre', 'u1_edge_in', 'u1_overlap', 'u1_other'},
    "A.insideFrontCourtyard('U1')": {'u1_centre', 'u1_edge_in', 'u1_overlap'},
    "A.insideBackCourtyard('U1')": {'u1_other'},
    "A.insideCourtyard('U2')": {'u2_centre', 'u2_corner', 'u2_other'},
    "A.insideFrontCourtyard('U2')": {'u2_centre', 'u2_corner'},
    "A.insideBackCourtyard('U2')": {'u2_other'},
    "A.insideCourtyard('U*')": {'u1_centre', 'u1_edge_in', 'u1_overlap', 'u1_other',
                                'u2_centre', 'u2_corner', 'u2_other', 'u3_ccw', 'u4_ccw'},
    # the 30-degree parts: which side of the long axis the turn puts each
    "A.insideCourtyard('U3')": {'u3_ccw'},
    "A.insideCourtyard('U4')": {'u4_ccw'},
    "A.insideFrontCourtyard('U4')": {'u4_ccw'},
    "A.insideCourtyard('lib:U1_fp')": {'u1_centre', 'u1_edge_in', 'u1_overlap', 'u1_other'},
    "A.NetName == 'N' && A.insideCourtyard('U1')": {'u1_centre', 'u1_edge_in', 'u1_overlap',
                                                    'u1_other'},
}


def courtyards(td):
    print("5. CourtyardIndex against kicad-cli's reading")
    path = os.path.join(td, 'p.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(PROBE_BOARD)
    with open(os.path.join(td, 'p.kicad_pro'), 'w', encoding='utf-8') as fh:
        json.dump({'board': {'design_settings': {'rules': {'min_via_annular_width': 0.1}}},
                   'net_settings': {'meta': {'version': 4}, 'classes': [{'name': 'Default'}],
                                    'netclass_patterns': []}}, fh)
    for cond, want in KICAD.items():
        with open(os.path.join(td, 'p.kicad_dru'), 'w', encoding='utf-8') as fh:
            fh.write(f'(version 1)\n(rule "cy" (condition "{cond}") '
                     f'(constraint annular_width (min 0.07mm)))\n')
        pcb, dr = load(evidence(path, 'the probe board'))
        idx = dr.courtyard_index(pcb, path)
        if idx is None:
            ok(False, f"{cond}: no courtyard index")
            continue
        got = {n for n, (x, y) in PROBE_VIAS.items()
               if dr.resolve('annular_width', dr.via_item(
                   SimpleNamespace(net_id=1, x=x, y=y, size=0.3), idx), None, None).min_rule}
        ok(got == want, f"{cond}: {sorted(got)}" + ('' if got == want else f" want {sorted(want)}"))


def main():
    with tempfile.TemporaryDirectory() as td:
        engine(td)
        listing(td)
        grade(td)
        courtyards(td)
    if FAILS:
        print(f"\nFAILED: {len(FAILS)}")
        return 1
    print("\nALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
