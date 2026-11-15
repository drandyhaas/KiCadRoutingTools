#!/usr/bin/env python3
"""#1850: check_drc grades the board's min_hole_clearance against every hole.

KiCad's hole_clearance holds the copper of every other-net track, via and pad
off every hole -- via drills, plated pad holes and NPTH holes -- at the
board's `min_hole_clearance` wherever no `.kicad_dru` rule binds the pair.
check_drc graded that minimum against NPTH holes (and via drills, #1914)
only, so copper standing inside it from a plated pad's hole, and any pad
near an NPTH hole, read clean. It is now one pass, `hole_rule_grade`, for
the minimum and the rules alike.

`KICAD` lists, per board, the cells kicad-cli 10.0.6 flags `hole_clearance`
on with `--all-track-errors` (without it a track or a via stops at its first
error, so its hole row beside a clearance row goes unreported). Measured on
these probes, Board Setup min_hole_clearance 0.35 under a 0.1 class:

  - every hole type binds every copper type of another net (cells 1-11, 17);
  - a pad clearance override binds before any rule, FLOORED at the board
    minimum: an override of 0.15 grades the pair at 0.35 (12, 21, 22; and
    23, where a rule would relax the pair), one of 0.5 at 0.5 (13);
  - a rule replaces the minimum where it binds, below it (19, 24, 25 on the
    rules board) and above it (20);
  - where the pair's copper clearance also fails, the hole row stands beside
    the clearance row (26-29, 31, 33, 35), except a pad's copper against a
    plated pad's hole (30, 32, 34);
  - same-net copper (16), and copper beyond the minimum (18), is not graded.

check_drc is run at KiCad's tolerance (`--clearance-margin-mm 0.0005`).

1. check_drc flags exactly the cells `KICAD` lists, each row graded at the
   value that binds it (`required_mm`, `required_source`, `rule`).
2. kicad-cli's own report of the same boards (needs kicad-cli; exit 77
   without it, after part 1).

    python3 tests/test_1850_hole_clearance_board_minimum.py
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
MIN_HOLE = 0.35
#: cell: items. A via is 0.6/0.3, a PTH 0.8/0.6, an NPTH 1.0, an SMD pad 0.5
#: square, tracks 0.2 wide; the optional last number is a pad override. The
#: second item stands 0.28 from the first's hole (0.40 in cells 18 and 20)
#: with its copper 0.13-0.18 clear (cells 1-25), or 0.05 clear (26-35).
#: ('via', net, dx, dy) ('pth', net, dx, dy[, ovr]) ('npth', dx, dy[, ovr])
#: ('smd', net, dx, layer[, ovr]) ('trk', net, offset, layer)
CELLS = {
    1: [('via', 'H1', 0, 0), ('trk', 'C1', 0.53, 'F.Cu')],
    2: [('via', 'H2', 0, 0), ('via', 'C2', 0.73, 0)],
    3: [('via', 'H3', 0, 0), ('smd', 'C3', 0.68, 'F.Cu')],
    4: [('pth', 'H4', 0, 0), ('trk', 'C4', 0.68, 'F.Cu')],
    5: [('pth', 'H5', 0, 0), ('via', 'C5', 0.88, 0)],
    6: [('pth', 'H6', 0, 0), ('smd', 'C6', 0.83, 'F.Cu')],
    7: [('pth', 'H7', 0, 0), ('pth', 'C7', 0.98, 0)],
    8: [('npth', 0, 0), ('trk', 'C8', 0.88, 'F.Cu')],
    9: [('npth', 0, 0), ('via', 'C9', 1.08, 0)],
    10: [('npth', 0, 0), ('smd', 'C10', 1.03, 'F.Cu')],
    11: [('npth', 0, 0), ('pth', 'C11', 1.18, 0)],
    12: [('pth', 'H12', 0, 0, 0.15), ('trk', 'C12', 0.68, 'F.Cu')],
    13: [('pth', 'H13', 0, 0, 0.5), ('trk', 'C13', 0.68, 'F.Cu')],
    14: [('npth', 0, 0, 0.15), ('trk', 'C14', 0.88, 'F.Cu')],
    15: [('via', 'H15', 0, 0), ('smd', 'C15', 0.68, 'F.Cu', 0.12)],
    16: [('via', 'H16', 0, 0), ('trk', 'H16', 0.53, 'F.Cu')],
    17: [('pth', 'H17', 0, 0), ('trk', 'C17', 0.68, 'B.Cu')],
    18: [('via', 'H18', 0, 0), ('trk', 'C18', 0.65, 'F.Cu')],
    19: [('via', 'R19', 0, 0), ('trk', 'C19', 0.53, 'F.Cu')],
    20: [('via', 'T20', 0, 0), ('trk', 'C20', 0.65, 'F.Cu')],
    21: [('pth', 'H21', 0, 0, 0.15), ('via', 'C21', 0.88, 0)],
    22: [('pth', 'H22', 0, 0, 0.15), ('smd', 'C22', 0.83, 'F.Cu')],
    23: [('pth', 'R23', 0, 0, 0.15), ('trk', 'C23', 0.68, 'F.Cu')],
    24: [('pth', 'R24', 0, 0), ('trk', 'C24', 0.68, 'F.Cu')],
    25: [('npth', 0, 0), ('trk', 'R25', 0.88, 'F.Cu')],
    26: [('via', 'H26', 0, 0), ('trk', 'C26', 0.45, 'F.Cu')],
    27: [('via', 'H27', 0, 0), ('via', 'C27', 0.65, 0)],
    28: [('via', 'H28', 0, 0), ('smd', 'C28', 0.60, 'F.Cu')],
    29: [('pth', 'H29', 0, 0), ('trk', 'C29', 0.55, 'F.Cu')],
    30: [('pth', 'H30', 0, 0), ('smd', 'C30', 0.70, 'F.Cu')],
    31: [('pth', 'H31', 0, 0), ('via', 'C31', 0.75, 0)],
    32: [('pth', 'H32', 0, 0), ('pth', 'C32', 0.85, 0)],
    33: [('smd', 'H33', 0, 'F.Cu'), ('via', 'C33', 0.60, 0)],
    34: [('pth', 'H34', 0, 0, 0.5), ('smd', 'C34', 0.83, 'F.Cu')],
    35: [('pth', 'H35', 0, 0, 0.5), ('via', 'C35', 0.88, 0)],
}
#: the rules board: relax the R* cells to 0.1, tighten T20 to 0.5
RULES = ('(rule "relax" (constraint hole_clearance (min 0.1mm)) (condition "'
         + ' || '.join(f"A.NetName == '{n}' || B.NetName == '{n}'"
                       for n in ('R19', 'R23', 'R24', 'R25')) + '"))\n'
         '(rule "tight" (constraint hole_clearance (min 0.5mm))'
         ' (condition "A.NetName == \'T20\' || B.NetName == \'T20\'"))')
FLAGGED = set(range(1, 16)) | {17, 19} | set(range(21, 30)) | {31, 33, 35}
#: name: (rules file or None, cells kicad-cli 10.0.6 flags hole_clearance on)
KICAD = {
    'minimum': (None, FLAGGED),
    'rules': (RULES, (FLAGGED - {19, 24, 25}) | {20}),
}
#: cell: (required_mm, required_source) on the minimum board
REQUIRED = {
    1: (0.35, 'declared hole clearance'), 4: (0.35, 'declared hole clearance'),
    10: (0.35, 'declared hole clearance'), 12: (0.35, 'declared hole clearance'),
    13: (0.5, 'pad override'), 22: (0.35, 'declared hole clearance'),
    35: (0.5, 'pad override'),
}
KICAD_TOL = '0.0005'


def cell(k):
    return 100 + ((k - 1) % 6) * 6, 100 + ((k - 1) // 6) * 6


def cell_of(x, y):
    return int(round((y - 100) / 6)) * 6 + int(round((x - 100) / 6)) + 1


def board():
    names = sorted({it[1] for items in CELLS.values() for it in items
                    if it[0] != 'npth'})
    nid = {n: i + 1 for i, n in enumerate(names)}
    out = ['(kicad_pcb (version 20241229) (generator "pcbnew") (generator_version "9.0")',
           ' (general (thickness 1.6) (legacy_teardrops no))', ' (paper "A4")',
           ' (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user)'
           ' (1 "F.Mask" user) (3 "B.Mask" user) (31 "F.CrtYd" user "F.Courtyard"))',
           ' (setup (pad_to_mask_clearance 0))', ' (net 0 "")']
    out += [f' (net {i} "{n}")' for n, i in nid.items()]
    out.append(' (gr_rect (start 90 90) (end 140 140) (stroke (width 0.05) (type default))'
               ' (fill no) (layer "Edge.Cuts") (uuid "e0000000-0000-0000-0000-000000000001"))')
    for k, items in CELLS.items():
        cx, cy = cell(k)
        for r, it in enumerate(items, 1):
            uid = f'00000000-0000-0000-{k:04d}-{r:012d}'
            kind = it[0]
            if kind == 'via':
                _, net, a, b = it
                out.append(f' (via (at {cx + a:.4f} {cy + b:.4f}) (size 0.6) (drill 0.3)'
                           f' (layers "F.Cu" "B.Cu") (net {nid[net]}) (uuid "{uid}"))')
                continue
            if kind == 'trk':
                _, net, a, layer = it
                out.append(f' (segment (start {cx - 2:.4f} {cy + a:.4f}) (end {cx + 2:.4f} '
                           f'{cy + a:.4f}) (width {W}) (layer "{layer}") (net {nid[net]})'
                           f' (uuid "{uid}"))')
                continue
            if kind == 'npth':
                a, b = it[1:3]
                ovr = it[3] if len(it) > 3 else None
                pad = ('(pad "" np_thru_hole circle (at 0 0) (size 1 1) (drill 1) '
                       '(layers "*.Cu" "*.Mask")')
                x, y = cx + a, cy + b
            elif kind == 'pth':
                net, a, b = it[1:4]
                ovr = it[4] if len(it) > 4 else None
                pad = (f'(pad "1" thru_hole circle (at 0 0) (size 0.8 0.8) (drill 0.6) '
                       f'(layers "*.Cu" "*.Mask") (net {nid[net]} "{net}")')
                x, y = cx + a, cy + b
            else:
                net, a, layer = it[1:4]
                ovr = it[4] if len(it) > 4 else None
                pad = (f'(pad "1" smd rect (at 0 0) (size 0.5 0.5) (layers "{layer}" '
                       f'"{layer[0]}.Mask") (net {nid[net]} "{net}")')
                x, y = cx + a, cy
            pad += (f' (clearance {ovr})' if ovr else '') + f' (uuid "{uid}"))'
            out.append(f'(footprint "probe:P{k}_{r}" (layer "F.Cu") '
                       f'(uuid "{uid[:-3]}1{uid[-2:]}") (at {x:.4f} {y:.4f} 0)'
                       f' (property "Reference" "P{k}_{r}" (at 0 -2 0) (layer "F.CrtYd")'
                       f' (uuid "{uid[:-3]}2{uid[-2:]}") (effects (font (size 0.5 0.5)'
                       f' (thickness 0.1)))) {pad})')
    return '\n'.join(out) + '\n)\n'


def stage(td, name, rules):
    path = os.path.join(td, f'{name}.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(board())
    zero = ('min_clearance', 'min_hole_to_hole', 'min_copper_edge_clearance',
            'min_through_hole_diameter', 'min_via_diameter', 'min_via_annular_width',
            'min_track_width')
    with open(path[:-len('.kicad_pcb')] + '.kicad_pro', 'w', encoding='utf-8') as fh:
        json.dump({'board': {'design_settings': {'rules': dict(
                      {k: 0.0 for k in zero}, min_hole_clearance=MIN_HOLE)}},
                   'net_settings': {'meta': {'version': 4},
                                    'classes': [{'name': 'Default', 'clearance': 0.1,
                                                 'track_width': 0.2}],
                                    'netclass_patterns': []}}, fh)
    if rules:
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
        _rules, want = KICAD[name]
        js = path + '.tool.json'
        # cell 1's track always stands inside the via's hole minimum
        check(PY + [CHECK_DRC, path, '--json', js, '--clearance-margin-mm', KICAD_TOL],
              refuse='TRACK-HOLE violations', code=1)
        with open(evidence(js, "check_drc's JSON"), encoding='utf-8') as fh:
            doc = json.load(fh)
        rows = [v for v in doc['items'] if v['type'] in ('track-hole', 'via-hole', 'pad-hole')
                and not v.get('accepted')]
        by = {}
        for v in rows:
            by.setdefault(cell_of(*v['hole_loc']), []).append(v)
        got = set(by)
        ok(got == want, f"{name}: hole rows on cells {sorted(got)}; kicad-cli "
                        f"{sorted(want)} (missing {sorted(want - got)}, extra "
                        f"{sorted(got - want)})")
        ok(all(len(v) == 1 for v in by.values()),
           f"{name}: one row per pair of items "
           f"{sorted(k for k, v in by.items() if len(v) != 1)}")
        if name == 'minimum':
            for k, (mm, src) in REQUIRED.items():
                v = (by.get(k) or [{}])[0]
                ok(abs((v.get('required_mm') or 0) - mm) < 1e-9
                   and v.get('required_source') == src,
                   f"cell {k}: graded at {v.get('required_mm')} ({v.get('required_source')}),"
                   f" KiCad's {mm} ({src})")
        else:
            v = (by.get(20) or [{}])[0]
            ok(v.get('rule') == 'tight' and abs((v.get('required_mm') or 0) - 0.5) < 1e-9,
               f"cell 20: graded at rule {v.get('rule')!r} {v.get('required_mm')}")
            v = (by.get(23) or [{}])[0]
            ok(v.get('required_source') == 'declared hole clearance' and not v.get('rule'),
               f"cell 23: the override (floored at the minimum) beats the rule: "
               f"{v.get('required_source')}, rule {v.get('rule')!r}")


def kicad(boards, cli):
    print("2. kicad-cli's own report of the boards (--all-track-errors)")
    for name, path in boards.items():
        _rules, want = KICAD[name]
        rep = path + '.kicad.json'
        subprocess.run([cli, 'pcb', 'drc', '--format', 'json', '--severity-all',
                        '--all-track-errors', '-o', rep, path],
                       capture_output=True, text=True, timeout=600)
        with open(evidence(rep, 'kicad-cli DRC report'), encoding='utf-8') as fh:
            rep = json.load(fh)
        rows = [v for v in rep['violations'] if v['type'] == 'hole_clearance']
        got = {int(v['items'][0]['uuid'].split('-')[3]) for v in rows}
        ok(got == want, f"kicad-cli {rep.get('kicad_version')}, {name}: cells "
                        f"{sorted(got)}, the table {sorted(want)}")


def main():
    td = tempfile.mkdtemp(prefix='krt_1850_')
    skipped = False
    try:
        boards = {name: stage(td, name, spec[0]) for name, spec in KICAD.items()}
        tool(boards)
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
