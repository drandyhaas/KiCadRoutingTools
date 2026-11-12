#!/usr/bin/env python3
"""#1805: check_drc reads the `(layer ...)` clause of a via-size rule as
kicad-cli does for a via.

A `.kicad_dru` rule may scope a via's ring, diameter or hole by layer:
`(rule "r" (layer outer) (condition "A.insideCourtyard('U1')") (constraint
annular_width (min 0.07mm)))`. check_drc resolved a via with no layer, and a
rule with a clause then bound nothing: a relaxing rule's via was flagged where
KiCad passes it, a tightening rule's via passed where KiCad flags it, and
nothing said the rule was not applied.

kicad-cli 10.0.7 grades a via's ring and diameter on F.Cu whatever layers the
via spans, and its hole on no layer. So (`KICAD_BINDS`, measured on `board()`
with one rule per file, 84 files, every via type alike):

  annular_width, via_diameter   bind under no clause, "F.Cu" and outer; not
                                under "B.Cu", "In1.Cu", "In2.Cu" or inner
  hole_size                     binds under every clause

1. check_drc.py on four boards -- rules that RELAX the Board Setup minimum and
   rules that TIGHTEN it, each bound by net and by net inside a courtyard, one
   rule per kind and clause on a net of its own, every net with a through, two
   blind, a buried and a micro via inside the courtyard and outside it --
   flags exactly the vias `KICAD_BINDS` says KiCad flags.
2. the #1805 board (glasgow_revC, `(layer outer)` on the courtyard rule):
   only the via outside U1's courtyard is flagged, and no rule is NOT GRADED.
3. control: the routing steps resolve a via rule by net, with no layer, and a
   rule with a clause still sets no floor there.
4. kicad-cli's own report of the four boards (needs kicad-cli; exit 77
   without it, after parts 1 to 3).
"""
import contextlib
import io
import json
import os
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
CHECK_DRC = os.path.join(REPO, 'py_router', 'check_drc.py')
GLASGOW = os.path.join(REPO, 'kicad_files', 'glasgow_revC.kicad_pcb')

#: kicad-cli 10.0.7: the layer clauses ('' = none) under which a rule of each
#: kind binds a via its condition names -- through, blind, buried, micro alike
KICAD_BINDS = {
    'annular_width': {'', '"F.Cu"', 'outer'},
    'via_diameter': {'', '"F.Cu"', 'outer'},
    'hole_size': {'', '"F.Cu"', '"B.Cu"', '"In1.Cu"', '"In2.Cu"', 'outer', 'inner'},
}
CLAUSES = ['', '"F.Cu"', '"B.Cu"', '"In1.Cu"', '"In2.Cu"', 'outer', 'inner']
#: kind: (check_drc's type, kicad-cli's types, the .kicad_pro key, the Board
#: Setup minimum the rule relaxes / tightens, the relaxing / tightening rule).
#: Every via is 0.4 / 0.2: ring 0.1, diameter 0.4, hole 0.2.
KINDS = {
    'annular_width': ('via-annular', ('annular_width',), 'min_via_annular_width',
                      (0.15, 0.05), (0.05, 0.15)),
    'via_diameter': ('via-size', ('via_diameter',), 'min_via_diameter',
                     (0.5, 0.3), (0.3, 0.5)),
    'hole_size': ('via-drill-size', ('drill_out_of_range', 'microvia_drill_out_of_range'),
                  'min_through_hole_diameter', (0.3, 0.1), (0.1, 0.3)),
}
VIA_TYPES = [('thru', '', 'F.Cu', 'B.Cu'), ('blind_front', 'blind ', 'F.Cu', 'In1.Cu'),
             ('blind_back', 'blind ', 'In2.Cu', 'B.Cu'), ('buried', 'buried ', 'In1.Cu', 'In2.Cu'),
             ('micro', 'micro ', 'F.Cu', 'In1.Cu')]
NETS = [(k, c) for k in KINDS for c in CLAUSES]         # one net per rule
CONDS = {'net': "A.NetName == '{net}'",
         'courtyard': "A.NetName == '{net}' && A.insideCourtyard('U1')"}

FAILS = []


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def net_name(kind, clause):
    return f"{kind}_{clause.strip(chr(34)).replace('.', '') or 'any'}"


def vias():
    """{(kind, clause, via type, inside U1's courtyard): (x, y)}"""
    out = {}
    for i, (kind, clause) in enumerate(NETS):
        for j, vt in enumerate(VIA_TYPES):
            out[(kind, clause, vt[0], True)] = (91.6 + 0.8 * i, 98.6 + 0.8 * j)
            out[(kind, clause, vt[0], False)] = (91.6 + 0.8 * i, 106.0 + 0.8 * j)
    return out


def board():
    """Four copper layers; U1 on F.Cu with a 20 x 6 mm courtyard; per net a
    column of five vias inside the courtyard and five below it."""
    nets = ''.join(f' (net {n + 2} "{net_name(*kc)}")' for n, kc in enumerate(NETS))
    types = {vt[0]: vt for vt in VIA_TYPES}
    body = ''
    for n, ((kind, clause, vt, _inside), (x, y)) in enumerate(vias().items()):
        _name, token, top, bottom = types[vt]
        body += (f' (via {token}(at {x:.2f} {y:.2f}) (size 0.4) (drill 0.2) (layers "{top}" "{bottom}") '
                 f'(net {NETS.index((kind, clause)) + 2}) (uuid "c0000000-0000-0000-0000-{n:012d}"))\n')
    return ('(kicad_pcb (version 20241229) (generator "pcbnew") (generator_version "9.0")\n'
            ' (general (thickness 1.6) (legacy_teardrops no))\n (paper "A4")\n'
            ' (layers (0 "F.Cu" signal) (4 "In1.Cu" signal) (6 "In2.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user) (31 "F.CrtYd" user "F.Courtyard") (29 "B.CrtYd" user "B.Courtyard") (5 "F.SilkS" user "F.Silkscreen") (7 "B.SilkS" user "B.Silkscreen"))\n'
            f' (setup (pad_to_mask_clearance 0))\n (net 0 "") (net 1 "PAD"){nets}\n'
            ' (gr_rect (start 85 90) (end 115 115) (stroke (width 0.05) (type default)) (fill no) (layer "Edge.Cuts") (uuid "e0000000-0000-0000-0000-000000000001"))\n'
            '(footprint "lib:U1_fp" (layer "F.Cu") (uuid "f0000000-0000-0000-0000-000000000010")\n'
            '  (at 100 100 0)\n'
            '  (property "Reference" "U1" (at 0 -4 0) (layer "F.SilkS") (uuid "f1000000-0000-0000-0000-000000000010") (effects (font (size 1 1) (thickness 0.15))))\n'
            '  (fp_rect (start -10 -3) (end 10 3) (stroke (width 0.05) (type solid)) (fill no) (layer "F.CrtYd") (uuid "f2000000-0000-0000-0000-000000000010"))\n'
            '  (pad "1" smd rect (at 0 -2.5) (size 0.5 0.5) (layers "F.Cu") (net 1 "PAD") (uuid "f3000000-0000-0000-0000-000000000010")))\n'
            + body + ')\n')


def stage(td, direction, cond):
    """One board: every kind and clause as a rule of its own, on its own net."""
    tighten = direction == 'tighten'
    path = os.path.join(td, f'{direction}_{cond}.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(board())
    rules = {spec[2]: spec[3][tighten] for spec in KINDS.values()}
    rules.update(min_clearance=0.1, min_microvia_diameter=rules['min_via_diameter'],
                 min_microvia_drill=rules['min_through_hole_diameter'])
    with open(path[:-len('.kicad_pcb')] + '.kicad_pro', 'w', encoding='utf-8') as fh:
        json.dump({'board': {'design_settings': {'rules': rules}},
                   'net_settings': {'meta': {'version': 4},
                                    'classes': [{'name': 'Default', 'clearance': 0.1}],
                                    'netclass_patterns': []}}, fh)
    with open(path[:-len('.kicad_pcb')] + '.kicad_dru', 'w', encoding='utf-8') as fh:
        fh.write('(version 1)\n' + ''.join(
            f'(rule "{net_name(kind, clause)}"\n'
            + (f'  (layer {clause})\n' if clause else '')
            + f'  (condition "{CONDS[cond].format(net=net_name(kind, clause))}")\n'
            f'  (constraint {kind} (min {KINDS[kind][4][tighten]}mm)))\n'
            for kind, clause in NETS))
    return evidence(path, 'the staged board')


def kicad_flags(direction, cond):
    """The vias KiCad flags on a board, from `KICAD_BINDS`: a relaxing rule's
    via where the rule does NOT bind it, a tightening rule's where it does."""
    out = set()
    for (kind, clause, vt, inside), _xy in vias().items():
        binds = clause in KICAD_BINDS[kind] and (cond == 'net' or inside)
        if binds == (direction == 'tighten'):
            out.add((kind, clause, vt, inside))
    return out


def at(xy):
    for key, (x, y) in vias().items():
        if abs(x - xy[0]) < 0.005 and abs(y - xy[1]) < 0.005:
            return key
    return None


def differ(got, want):
    d = sorted(got ^ want)
    return '' if not d else (f": {len(d)} via(s) differ, first "
                             f"{[('flagged' if k in got else 'passed') + ' ' + '/'.join(map(str, k)) for k in d[:3]]}")


def tool(td, boards):
    print("1. check_drc flags the vias KiCad flags")
    for (direction, cond), path in boards.items():
        js = path + '.tool.json'
        check(PY + [CHECK_DRC, path, '--json', js], refuse='DRC VIOLATIONS', code=1)
        with open(evidence(js, "check_drc's JSON"), encoding='utf-8') as fh:
            items = json.load(fh)['items']
        want = kicad_flags(direction, cond)
        for kind, spec in KINDS.items():
            got = {at(i['via_loc']) for i in items if i['type'] == spec[0]}
            ok(None not in got, f"{direction}, by {cond}, {kind}: every flag is a fixture via")
            got = {k for k in got if k and k[0] == kind}
            w = {k for k in want if k[0] == kind}
            ok(got == w, f"{direction}, by {cond}, {kind}: {len(got)} via(s) flagged, "
                         f"KiCad {len(w)}{differ(got, w)}")


def issue_board(td):
    print("2. the #1805 board with (layer outer) on its courtyard rule")
    b = os.path.join(td, 'glasgow.kicad_pcb')
    check(PY + [os.path.join(REPO, 'py_router', 'copy_board.py'), GLASGOW, b], accept=True)
    with open(os.path.join(td, 'glasgow.kicad_dru'), 'w', encoding='utf-8') as fh:
        fh.write('(version 1)\n(rule "ring_in_u1"\n  (layer outer)\n'
                 '  (condition "A.NetName == \'/WR\' && A.insideCourtyard(\'U1\')")\n'
                 '  (constraint annular_width (min 0.07mm)))\n')
    with open(b, encoding='utf-8') as fh:
        t = fh.read()
    k = t.rindex(')')
    t = t[:k] + ''.join(f'\t(via (at {x} {y}) (size 0.3) (drill 0.16) (layers "F.Cu" "B.Cu") '
                        f'(net "/WR"))\n' for x, y in ((65.85, 92.75), (100, 80))) + t[k:]
    with open(b, 'w', encoding='utf-8') as fh:
        fh.write(t)
    r = check(PY + [CHECK_DRC, evidence(b, 'the staged board')],
              refuse='VIA-ANNULAR violations (1)')
    block = r.stdout.split('VIA-ANNULAR violations (1)', 1)[1].split('\n\n', 1)[0]
    ok('(100.00,80.00)' in block, "the via outside U1's courtyard is flagged")
    ok('(65.85,92.75)' not in block, "the via on pad U1.2, which KiCad passes, is not")
    ok('NOT GRADED' not in r.stdout, "no rule is NOT GRADED: the rule is applied")
    return b


def routing_control(b):
    print("3. control: the routing steps' reading is unchanged")
    from kicad_parser import parse_kicad_pcb
    from design_rules import DesignRules
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        pcb = parse_kicad_pcb(b)
        dr = DesignRules.from_project(pcb, b)
    wr = next(i for i, n in pcb.nets.items() if n.name == '/WR')
    ok(dr.via_floors(wr) == {}, f"via_floors of the rule's net sets no floor: {dr.via_floors(wr)}")
    plain = dr.resolve('annular_width', dr.item_for_net(wr, 'via'), None, None)
    ok(plain.min_rule is None, f"a net's via, no layer: no rule binds ({plain.source})")


def kicad(boards, cli):
    print("4. kicad-cli's own report of the four boards")
    for (direction, cond), path in boards.items():
        rep = path + '.kicad.json'
        subprocess.run([cli, 'pcb', 'drc', '--format', 'json', '--severity-all', '-o', rep, path],
                       capture_output=True, text=True, timeout=600)
        with open(evidence(rep, 'kicad-cli DRC report'), encoding='utf-8') as fh:
            rep = json.load(fh)
        want = kicad_flags(direction, cond)
        for kind, spec in KINDS.items():
            got = {at((i['pos']['x'], i['pos']['y'])) for v in rep['violations']
                   if v['type'] in spec[1] for i in v['items']}
            got = {k for k in got if k and k[0] == kind}
            w = {k for k in want if k[0] == kind}
            ok(got == w, f"kicad-cli {rep.get('kicad_version')}, {direction}, by {cond}, {kind}: "
                         f"{len(got)} via(s) flagged, the table {len(w)}{differ(got, w)}")


def main():
    td = tempfile.mkdtemp(prefix='krt1805l_')
    skipped = False
    try:
        boards = {(d, c): stage(td, d, c) for d in ('relax', 'tighten') for c in CONDS}
        tool(td, boards)
        routing_control(issue_board(td))
        with contextlib.redirect_stdout(io.StringIO()):
            from kicad_oracle import find_kicad_cli
            cli = find_kicad_cli(warn=False)
        if cli:
            kicad(boards, cli)
        else:
            print("4. SKIPPED: no kicad-cli (set KICAD_CLI)")
            skipped = True
    except AssertionError as exc:
        ok(False, f"a run_utils assertion held: {str(exc)[:1500]}")
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if FAILS:
        print(f"\nFAILED: {len(FAILS)}")
        return 1
    if skipped:
        print("\nPARTS 1-3 PASS")
        print("SKIP: part 4 needs kicad-cli (none found; set KICAD_CLI) -- "
              "KiCad's own grade of the four boards was not checked")
        return 77
    print("\nALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
