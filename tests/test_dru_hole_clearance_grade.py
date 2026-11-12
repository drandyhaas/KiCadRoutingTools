#!/usr/bin/env python3
"""check_drc grades a `.kicad_dru` `hole_clearance` rule as kicad-cli does.

check_drc's copper-to-hole passes grade the board minimum against NPTH holes
(and a few plated ones). A `hole_clearance` RULE binds far more: every via
drill, plated and NPTH hole against the copper of every track, via and pad of
another net, on each copper layer both occupy, the last matching rule in the
file deciding. Such a rule was graded by nothing, so a board carrying the
JLCPCB or PCBWay rules template (each has several) read clean wherever its
copper broke them.

`KICAD` is what kicad-cli 10.0.6 reports on `board()`: per rules file, the
configurations (cells) it flags. Measured on these probes:

  - a hole binds the copper of another net only: same-net pairs, and two
    net-less items, are never flagged;
  - a condition is read either way round (`A.Type == 'Via'` binds a via as
    B too), and a (layer ...) clause scopes the COPPER item's layer;
  - the last matching rule wins, below the board minimum too (a relaxing
    rule clears a pair the minimum would flag);
  - a pad clearance override binds before any rule;
  - where the pair's copper clearance also fails, KiCad reports the
    clearance row alone, except a pad against a via's hole.

1. check_drc flags exactly the cells `KICAD` lists, each row naming its rule;
   a rule of severity warning is demoted; a rule whose condition it cannot
   read is named `NOT GRADED (hole_clearance)`.
2. kicad-cli's own report of the same boards (needs kicad-cli; exit 77
   without it, after part 1).

    python3 tests/test_dru_hole_clearance_grade.py
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

W = 0.2
#: cell: items. A via is 0.6/0.3, a PTH 1.0/0.6, an NPTH 1.0, an SMD pad 0.5
#: square; tracks 0.2 wide. Under a 0.5 rule every cell but 3, 5, 7, 10 and
#: 17 is short (hole gap 0.30-0.45), at a copper gap >= 0.15 over a 0.1 class.
CONFIGS = {
    1: [('via', 'H1', 0, 0), ('trk', 'C1', 0.6, 'F.Cu')],          # hole gap .35
    2: [('via', 'H2', 0, 0), ('trk', 'C2', 0.6, 'B.Cu')],
    3: [('via', 'H3', 0, 0), ('trk', 'H3', 0.6, 'F.Cu')],          # same net
    4: [('via', 'H4', 0, 0), ('via', 'C4', 0.8, 0)],
    5: [('via', 'H5', 0, 0), ('via', 'H5', 0.8, 0)],               # same net
    6: [('pth', 'H6', 0, 0), ('trk', 'C6', 0.8, 'F.Cu')],          # hole gap .40
    7: [('pth', 'H7', 0, 0), ('trk', 'H7', 0.8, 'F.Cu')],          # same net
    8: [('npth', '', 0, 0), ('trk', 'C8', 0.9, 'F.Cu')],           # hole gap .30
    9: [('via', 'H9', 0, 0), ('smd', 'C9', 0.75, 'F.Cu')],
    10: [('via', 'H10', 0, 0), ('smd', 'H10', 0.75, 'F.Cu')],      # same net
    11: [('pth', 'H11', 0, 0), ('via', 'C11', 0.95, 0)],
    12: [('pth', 'H12', 0, 0), ('pth', 'C12', 1.15, 0)],
    13: [('via', 'H13', 0, 0), ('smd', 'C13', 0.75, 'B.Cu')],
    14: [('npth', '', 0, 0), ('via', 'C14', 1.1, 0)],
    15: [('npth', '', 0, 0), ('smd', 'C15', 1.05, 'F.Cu')],
    16: [('pth', 'H16', 0, 0), ('smd', 'C16', 0.9, 'F.Cu')],
    17: [('via', 'H17', 0, 0), ('trk', 'C17', 0.8, 'F.Cu')],       # hole gap .55
    18: [('npth', '', 0, 0), ('trk', '', 0.9, 'F.Cu')],            # both net-less
    19: [('via', 'H19', 0, 0), ('trk', 'C19', 0.45, 'F.Cu')],      # copper .05 too
    20: [('via', 'H20', 0, 0), ('smd', 'C20', 0.6, 'F.Cu')],       # copper .05 too
    21: [('pth', 'H21', 0, 0, 0.2), ('trk', 'C21', 0.8, 'F.Cu')],  # override .2
}
ALL = {1, 2, 4, 6, 8, 9, 11, 12, 13, 14, 15, 16, 20}
HC = '(constraint hole_clearance (min 0.5mm))'
#: name: (rules file, board min_hole_clearance, cells kicad-cli 10.0.6 flags,
#: severity of its rows)
KICAD = {
    'uncond': (f'(rule "hc" {HC})', 0.0, ALL, 'error'),
    'via': (f'(rule "hc" {HC} (condition "A.Type == \'Via\'"))', 0.0,
            {1, 2, 4, 9, 11, 13, 14, 20}, 'error'),
    'pad_track': (f'(rule "hc" {HC} (condition "A.Type == \'Pad\' && B.Type == \'Track\'"))',
                  0.0, {6, 8}, 'error'),
    'npth': (f'(rule "hc" {HC} (condition "A.Pad_Type == \'NPTH, mechanical\'"))', 0.0,
             {8, 14, 15}, 'error'),
    'layer_b': (f'(rule "hc" (layer "B.Cu") {HC})', 0.0, {2, 4, 11, 12, 13, 14}, 'error'),
    'same_net': (f'(rule "hc" {HC} (condition "A.Net == B.Net"))', 0.0, set(), 'error'),
    'last_wins': (f'(rule "hc" {HC})\n(rule "lo" (constraint hole_clearance (min 0.05mm))'
                  f' (condition "A.NetName == \'H1\' || B.NetName == \'H1\'"))', 0.0,
                  ALL - {1}, 'error'),
    'warning': (f'(rule "hc" (severity warning) {HC})', 0.0, ALL, 'warning'),
    'relax_min': ('(rule "lo" (constraint hole_clearance (min 0.05mm)))', 0.5, set(), 'error'),
}
#: cells left off a board: a pad override is floored at the board minimum,
#: which the board-minimum passes grade, not a rule
OMIT = {'relax_min': {21}}
UNREAD = f'(rule "hc_unread" {HC} (condition "A.getField(\'x\') == \'y\'"))'


def cell(k):
    return 100 + ((k - 1) % 6) * 6, 100 + ((k - 1) // 6) * 6


def cell_of(x, y):
    return int(round((y - 100) / 6)) * 6 + int(round((x - 100) / 6)) + 1


def board(omit=frozenset()):
    names = sorted({it[1] for items in CONFIGS.values() for it in items if it[1]})
    nid = {n: i + 1 for i, n in enumerate(names)}
    out = ['(kicad_pcb (version 20241229) (generator "pcbnew") (generator_version "9.0")',
           ' (general (thickness 1.6) (legacy_teardrops no))', ' (paper "A4")',
           ' (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user)'
           ' (1 "F.Mask" user) (3 "B.Mask" user) (31 "F.CrtYd" user "F.Courtyard"))',
           ' (setup (pad_to_mask_clearance 0))', ' (net 0 "")']
    out += [f' (net {i} "{n}")' for n, i in nid.items()]
    out.append(' (gr_rect (start 90 90) (end 140 125) (stroke (width 0.05) (type default))'
               ' (fill no) (layer "Edge.Cuts") (uuid "e0000000-0000-0000-0000-000000000001"))')
    for k, items in CONFIGS.items():
        if k in omit:
            continue
        cx, cy = cell(k)
        for r, it in enumerate(items, 1):
            uid = f'00000000-0000-0000-{k:04d}-{r:012d}'
            kind, net, a, b = it[:4]
            n = nid.get(net, 0)
            if kind == 'via':
                out.append(f' (via (at {cx + a:.4f} {cy + b:.4f}) (size 0.6) (drill 0.3)'
                           f' (layers "F.Cu" "B.Cu") (net {n}) (uuid "{uid}"))')
            elif kind == 'trk':
                out.append(f' (segment (start {cx - 2:.4f} {cy + a:.4f}) (end {cx + 2:.4f} '
                           f'{cy + a:.4f}) (width {W}) (layer "{b}") (net {n}) (uuid "{uid}"))')
            else:
                ovr = f' (clearance {it[4]})' if len(it) > 4 else ''
                netx = f' (net {n} "{net}")' if n else ''
                if kind == 'pth':
                    pad = (f'(pad "1" thru_hole circle (at 0 0) (size 1 1) (drill 0.6) '
                           f'(layers "*.Cu" "*.Mask"){netx}{ovr} (uuid "{uid}"))')
                    x, y = cx + a, cy + b
                elif kind == 'npth':
                    pad = (f'(pad "" np_thru_hole circle (at 0 0) (size 1 1) (drill 1) '
                           f'(layers "*.Cu" "*.Mask") (uuid "{uid}"))')
                    x, y = cx + a, cy + b
                else:
                    pad = (f'(pad "1" smd rect (at 0 0) (size 0.5 0.5) (layers "{b}" '
                           f'"{b[0]}.Mask"){netx} (uuid "{uid}"))')
                    x, y = cx + a, cy
                out.append(f'(footprint "probe:P{k}_{r}" (layer "F.Cu") '
                           f'(uuid "{uid[:-3]}1{uid[-2:]}") (at {x:.4f} {y:.4f} 0)'
                           f' (property "Reference" "P{k}_{r}" (at 0 -2 0) (layer "F.CrtYd")'
                           f' (uuid "{uid[:-3]}2{uid[-2:]}") (effects (font (size 0.5 0.5)'
                           f' (thickness 0.1)))) {pad})')
    return '\n'.join(out) + '\n)\n'


def stage(td, name, rules, min_hole):
    path = os.path.join(td, f'{name}.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(board(OMIT.get(name, frozenset())))
    zero = ('min_clearance', 'min_hole_to_hole', 'min_copper_edge_clearance',
            'min_through_hole_diameter', 'min_via_diameter', 'min_via_annular_width',
            'min_track_width')
    with open(path[:-len('.kicad_pcb')] + '.kicad_pro', 'w', encoding='utf-8') as fh:
        json.dump({'board': {'design_settings': {'rules': dict(
                      {k: 0.0 for k in zero}, min_hole_clearance=min_hole)}},
                   'net_settings': {'meta': {'version': 4},
                                    'classes': [{'name': 'Default', 'clearance': 0.1,
                                                 'track_width': 0.2}],
                                    'netclass_patterns': []}}, fh)
    with open(path[:-len('.kicad_pcb')] + '.kicad_dru', 'w', encoding='utf-8') as fh:
        fh.write('(version 1)\n' + rules + '\n')
    return evidence(path, 'the staged board')


FAILS = []


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def tool(boards):
    print("1. check_drc flags the cells kicad-cli flags")
    for name, path in boards.items():
        _rules, _mh, want, sev = KICAD[name]
        js = path + '.tool.json'
        # cells 19 and 20 fail their copper clearance on every board
        r = check(PY + [CHECK_DRC, path, '--json', js], refuse='VIA-SEGMENT violations (1)',
                  code=1)
        with open(evidence(js, "check_drc's JSON"), encoding='utf-8') as fh:
            doc = json.load(fh)
        hole = [v for v in doc['items'] if v['type'] in ('track-hole', 'via-hole', 'pad-hole')]
        rows = [v for v in hole if not v.get('accepted')]
        warned = [v for v in hole if v.get('accepted') == 'severity-warning']
        got = {cell_of(*v['hole_loc']) for v in (rows if sev == 'error' else warned)}
        ok(got == want, f"{name}: cells {sorted(got)}, kicad-cli {sorted(want)}")
        if sev == 'warning':
            ok(not rows, f"{name}: a warning rule's rows are demoted, none counted")
        named = {v.get('rule') for v in rows + warned}
        ok(named <= {'hc', 'lo'} and (not got or named), f"{name}: each row names its rule {named}")
        ok('NOT GRADED (hole_clearance)' not in r.stdout,
           f"{name}: the rule is graded, not named NOT GRADED")


def unread(td):
    print("1b. a hole_clearance rule check_drc cannot read is named")
    path = stage(td, 'unread', UNREAD, 0.0)
    js = path + '.tool.json'
    r = check(PY + [CHECK_DRC, path, '--json', js], refuse='VIA-SEGMENT violations (1)', code=1)
    ok("NOT GRADED (hole_clearance): rule 'hc_unread'" in r.stdout,
       "check_drc prints NOT GRADED (hole_clearance) for it")
    with open(evidence(js, "check_drc's JSON"), encoding='utf-8') as fh:
        rows = json.load(fh).get('not_graded') or []
    ok(any(x.get('rule') == 'hc_unread' and x.get('kind') == 'hole_clearance' for x in rows),
       "and lists it in --json not_graded")


def kicad(boards, cli):
    print("2. kicad-cli's own report of the boards")
    for name, path in boards.items():
        _rules, _mh, want, sev = KICAD[name]
        rep = path + '.kicad.json'
        subprocess.run([cli, 'pcb', 'drc', '--format', 'json', '--severity-all', '-o', rep, path],
                       capture_output=True, text=True, timeout=600)
        with open(evidence(rep, 'kicad-cli DRC report'), encoding='utf-8') as fh:
            rep = json.load(fh)
        rows = [v for v in rep['violations'] if v['type'] == 'hole_clearance']
        got = {int(v['items'][0]['uuid'].split('-')[3]) for v in rows}
        ok(got == want and all(v['severity'] == sev for v in rows),
           f"kicad-cli {rep.get('kicad_version')}, {name}: cells {sorted(got)}, "
           f"the table {sorted(want)} ({sev})")


def main():
    td = tempfile.mkdtemp(prefix='krt_hole_rule_')
    skipped = False
    try:
        boards = {name: stage(td, name, spec[0], spec[1]) for name, spec in KICAD.items()}
        tool(boards)
        unread(td)
        with contextlib.redirect_stdout(io.StringIO()):
            from kicad_oracle import find_kicad_cli
            cli = find_kicad_cli(warn=False)
        if cli:
            kicad(boards, cli)
        else:
            print("2. SKIPPED: no kicad-cli (set KICAD_CLI)")
            skipped = True
    except AssertionError as exc:
        ok(False, f"a run_utils assertion held: {str(exc)[:1500]}")
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if FAILS:
        print(f"\nFAILED: {len(FAILS)}")
        return 1
    if skipped:
        print("\nPART 1 PASS")
        print("SKIP: part 2 needs kicad-cli (none found; set KICAD_CLI) -- "
              "KiCad's own grade of the boards was not checked")
        return 77
    print("\nALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
