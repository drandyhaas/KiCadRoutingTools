#!/usr/bin/env python3
"""#1843: check_drc grades a `.kicad_dru` clearance / track_width rule scoped
by a rule area, by where each item stands.

A rule conditioned on `A.insideArea('NAME')` was read as unsupported: its
copper was graded at the class value, so legal copper inside the area was
counted, and every violation it might have bound -- the ones outside the
area too -- was marked UNVERIFIED.

The fixture (tests/fixtures/1843) is synthetic: Default class clearance
0.2 mm, a rule area `TIGHT` on F.Cu (x 2-9, y 2-6), the rule
`clearance (min 0.09mm)` on `A.insideArea('TIGHT')`, and two pairs of F.Cu
tracks 0.09 mm apart, `IN_A`/`IN_B` inside the area and `OUT_A`/`OUT_B`
outside it.

`EXTRA` adds the copper the predicates were measured on, and every row of
`ROWS` is one condition with the clearance pairs and narrow tracks
kicad-cli 10.0.8 reports for it (`--all-track-errors`):
  * `A1`/`A2`: one track inside the area, one just outside it -- bound when
    either item is in the area, not when the rule asks it of both;
  * `B1`/`B2`: both straddle the area's edge -- in it for insideArea() and
    intersectsArea(), not for enclosedByArea();
  * `C1`/`C2`: inside the area's outline on B.Cu, which the area is not on;
  * `V1`, `V2`: through vias inside the outline, 0.09 mm from a B.Cu and
    from an F.Cu track -- insideArea() holds an item on the area's layers
    only, while enclosedByArea() holds a via the area encloses on every
    layer the via has;
  * `W1`, `W2`: 0.08 mm tracks outside and inside, under a `track_width`
    rule of the same condition (Board Setup min 0.09).

Checks (no routing):
  1. the fixture: one violation, the OUT pair, not marked unverified, and
     the rule is not listed NOT GRADED;
  2. each row's check_drc pairs and narrow tracks are KiCad's;
  3. `DesignRules.area_index` reads the rule by position, and a rule naming
     an area the board does not draw stays unread and is named NOT GRADED;
  4. an area drawn on `*.Cu`, `F&B.Cu` or `*In.Cu` holds items on every
     copper layer the token stands for, as kicad-cli 10.0.6 reads it.
"""
import contextlib
import io
import json
import os
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)
from run_utils import evidence  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402
from design_rules import DesignRules  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
BOARD = os.path.join(TESTS, 'fixtures', '1843', 'area_rule')
CLEARANCE_TYPES = ('segment-segment', 'via-segment')


def _seg(x1, y1, x2, y2, net, layer='F.Cu', w=0.1):
    return (f'\t(segment\n\t\t(start {x1} {y1})\n\t\t(end {x2} {y2})\n\t\t(width {w})\n'
            f'\t\t(layer "{layer}")\n\t\t(net "{net}")\n\t)\n')


def _via(x, y, net):
    return (f'\t(via\n\t\t(at {x} {y})\n\t\t(size 0.46)\n\t\t(drill 0.2)\n'
            f'\t\t(layers "F.Cu" "B.Cu")\n\t\t(net "{net}")\n\t)\n')


EXTRA = (_seg(3, 5.9, 8, 5.9, 'A1') + _seg(3, 6.09, 8, 6.09, 'A2')
         + _seg(8, 4, 11, 4, 'B1') + _seg(8, 4.19, 11, 4.19, 'B2')
         + _seg(3, 5, 8, 5, 'C1', 'B.Cu') + _seg(3, 5.19, 8, 5.19, 'C2', 'B.Cu')
         + _seg(12, 5, 17, 5, 'W1', w=0.08) + _seg(3, 4.5, 8, 4.5, 'W2', w=0.08)
         + _via(2.63, 5, 'V1') + _via(2.63, 3, 'V2'))
ALWAYS = {('C1', 'C2'), ('C1', 'V1'), ('C2', 'V1'), ('OUT_A', 'OUT_B')}

#: clearance condition -> (track_width condition, KiCad's clearance pairs)
ROWS = {
    "A.insideArea('TIGHT')": ("A.insideArea('TIGHT')", ALWAYS),
    "A.intersectsArea('TIGHT')": ("A.intersectsArea('TIGHT')", ALWAYS),
    "A.insideArea('TIGHT') && B.insideArea('TIGHT')": (
        "A.insideArea('TIGHT')", ALWAYS | {('A1', 'A2')}),
    "A.enclosedByArea('TIGHT')": (
        "A.enclosedByArea('TIGHT')",
        {('B1', 'B2'), ('C1', 'C2'), ('OUT_A', 'OUT_B')}),
}

#: the same rows with the area drawn on every copper layer, spelled with the
#: composite tokens KiCad reads (`*.Cu`; `F&B.Cu`, which here is F.Cu and
#: B.Cu): the B.Cu pairs and vias are now in the area too (kicad-cli 10.0.6)
ALL_LAYERS = {
    "A.insideArea('TIGHT')": {('OUT_A', 'OUT_B')},
    "A.intersectsArea('TIGHT')": {('OUT_A', 'OUT_B')},
    "A.insideArea('TIGHT') && B.insideArea('TIGHT')": {('A1', 'A2'), ('OUT_A', 'OUT_B')},
    "A.enclosedByArea('TIGHT')": {('B1', 'B2'), ('OUT_A', 'OUT_B')},
}
#: an area on `*In.Cu` holds two inner-layer tracks 0.09 mm apart, and not
#: the F.Cu pair (kicad-cli 10.0.6)
INNER = _seg(3, 4, 8, 4, 'I1', 'In1.Cu') + _seg(3, 4.19, 8, 4.19, 'I2', 'In1.Cu')

FAILS = []


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def dru(clearance_cond, width_cond=None):
    text = ('(version 1)\n(rule "tight room"\n  (constraint clearance (min 0.09mm))\n'
            f'  (condition "{clearance_cond}"))\n')
    if width_cond:
        text += ('(rule "tight width"\n  (constraint track_width (min 0.08mm))\n'
                 f'  (condition "{width_cond}"))\n')
    return text


def stage(td, name, rules=None, extra='', layers=None):
    with open(evidence(BOARD + '.kicad_pcb'), encoding='utf-8') as fh:
        t = fh.read()
    if layers:
        zone = '\t(zone\n\t\t(layer "F.Cu")'
        if zone not in t:
            raise AssertionError("BROKEN TEST: the fixture's area is not drawn on F.Cu")
        t = t.replace(zone, f'\t(zone\n\t\t(layers "{layers}")')
    if extra:
        k = t.index('\t(zone')
        t = t[:k] + extra + t[k:]
    b = os.path.join(td, name + '.kicad_pcb')
    with open(b, 'w', encoding='utf-8') as fh:
        fh.write(t)
    for ext in ('.kicad_pro', '.kicad_dru'):
        with open(evidence(BOARD + ext), encoding='utf-8') as fh:
            text = fh.read()
        if ext == '.kicad_dru' and rules is not None:
            text = rules
        with open(os.path.join(td, name + ext), 'w', encoding='utf-8') as fh:
            fh.write(text)
    return evidence(b, 'the staged board')


def grade(path):
    out = path[:-len('.kicad_pcb')] + '.drc.json'
    r = subprocess.run(PY + [os.path.join(REPO, 'py_router', 'check_drc.py'), path,
                             '--json', out],
                       capture_output=True, text=True, encoding='utf-8', errors='replace')
    text = (r.stdout or '') + (r.stderr or '')
    if 'Traceback' in text or r.returncode not in (0, 1):
        raise AssertionError(f"BROKEN TEST: check_drc died\n{text[-1500:]}")
    with open(evidence(out, 'the check_drc report'), encoding='utf-8') as fh:
        return json.load(fh), text


def pairs(doc):
    return {tuple(sorted((v['net1'], v['net2']))) for v in doc['items']
            if v['type'] in CLEARANCE_TYPES and not v.get('accepted')}


def fixture(td):
    print("1. the issue's board")
    doc, text = grade(stage(td, 'f'))
    ok(pairs(doc) == {('OUT_A', 'OUT_B')} and doc['violations'] == 1,
       f"one violation, the pair outside the area: {sorted(pairs(doc))}")
    ok('UNVERIFIED' not in text and 'Unverified' not in text
       and not doc.get('unverified'), "nothing is marked unverified")
    ok('NOT GRADED' not in text and not doc.get('not_graded'),
       "and the rule is not listed NOT GRADED")
    ok("Area-scoped clearance rules" in text and "'tight room':0.09" in text,
       "the run says the rule is graded by position")


def rows(td):
    print("2. check_drc reports KiCad's pairs for every area predicate")
    for i, (cond, (wcond, want)) in enumerate(ROWS.items()):
        doc, _ = grade(stage(td, f'r{i}', dru(cond, wcond), EXTRA))
        got = pairs(doc)
        ok(got == want, f"{cond}: {sorted(got)}"
           + ('' if got == want else f" want {sorted(want)}"))
        narrow = sorted(v['net1'] for v in doc['items'] if v['type'] == 'track-width')
        ok(narrow == ['W1'], f"   narrow tracks: {narrow} (W2 is inside the area)")


def layer_tokens(td):
    print("4. an area drawn with a composite layer token")
    for tok in ('*.Cu', 'F&B.Cu'):
        for i, (cond, want) in enumerate(ALL_LAYERS.items()):
            doc, _ = grade(stage(td, f't{tok[0]}{i}', dru(cond, ROWS[cond][0]), EXTRA,
                                 layers=tok))
            got = pairs(doc)
            ok(got == want, f"({tok}) {cond}: {sorted(got)}"
               + ('' if got == want else f" want {sorted(want)}"))
            narrow = sorted(v['net1'] for v in doc['items'] if v['type'] == 'track-width')
            ok(narrow == ['W1'], f"   narrow tracks: {narrow}")
    doc, _ = grade(stage(td, 'ti', extra=INNER, layers='*In.Cu'))
    got = pairs(doc)
    want = {('IN_A', 'IN_B'), ('OUT_A', 'OUT_B')}
    ok(got == want, f"(*In.Cu) the inner pair is in the area, the F.Cu pair is not: {sorted(got)}")


def engine(td):
    print("3. the resolver, and an area the board does not have")
    path = stage(td, 'e', dru("A.insideArea('TIGHT')"), EXTRA)
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        pcb = parse_kicad_pcb(path)
        dr = DesignRules.from_project(pcb, path, copper_layers=pcb.board_info.copper_layers)
    via = {pcb.nets[v.net_id].name: v for v in pcb.vias}['V2']
    before = dr.resolve('clearance', dr.via_item(via), None, 'F.Cu').min
    idx = dr.area_index(pcb)
    item = dr.via_item(via)
    ok(idx is not None and idx.names == frozenset({'TIGHT'}), "the board's area is indexed")
    ok(item.area_layers == frozenset({('TIGHT', 'F.Cu')}),
       f"a through via inside it is in it on F.Cu only: {sorted(item.area_layers or ())}")
    ok((before, dr.resolve('clearance', item, None, 'F.Cu').min,
        dr.resolve('clearance', item, None, 'B.Cu').min) == (0.2, 0.09, 0.2),
       "unmeasured 0.2; measured 0.09 on F.Cu and 0.2 on B.Cu")
    doc, text = grade(stage(td, 'm', dru("A.insideArea('NOWHERE')")))
    ok(pairs(doc) == {('IN_A', 'IN_B'), ('OUT_A', 'OUT_B')},
       "a rule on an area the board lacks binds nothing")
    ok("NOT GRADED (clearance): rule 'tight room'" in text,
       "and is still named NOT GRADED")


def main():
    with tempfile.TemporaryDirectory() as td:
        fixture(td)
        rows(td)
        engine(td)
        layer_tokens(td)
    if FAILS:
        print(f"\nFAILED: {len(FAILS)}")
        return 1
    print("\nALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
