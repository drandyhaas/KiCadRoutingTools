#!/usr/bin/env python3
"""A via a step sizes itself meets Board Setup under --escalation board, and
is named when it does not (#1775).

qfn_fanout laid its built-in 0.45/0.25 via (ring 0.10) on a board whose Board
Setup declares min_via_annular_width 0.13, silently, also under `--escalation
board --no-fix-drc-settings`: #1676 raised the self-sized via only to
`.kicad_dru` rules and #1579 only the descent rungs. Now, in the shared
resolver (`design_rules.run_via_for_rules` / `via_for_board_setup`, which every
self-sizing step and the GUI fanout tab go through):

  1. under `board` the self-sized via is raised to min_through_hole_diameter,
     min_via_diameter and the ring round its drill (0.51/0.25 here), with or
     without the writeback, and check_drc finds no annular violation;
  2. under the default `fab` it keeps 0.45/0.25; with the writeback off the
     step prints a WARNING naming the Board Setup minimum and JSON_SUMMARY
     `board_setup_vias` carries the row (8 vias below 0.13);
  3. with the writeback on the row is absent (the writeback lowers the
     project's minimum and says FAB FLOOR RELAXED);
  4. units: the GUI's policy setter (set_fab_tier_from_config) reaches the
     same raise and records the writeback; `board_setup=False` and an
     explicit policy other than board leave the via alone; annular_grade
     grades a fanout's via dicts against the Board Setup ring only when asked.

The fixture is the issue's: qfn_csi_underpad_diff plus J9, a 2-row 0.5 mm
socket with two un-netted mounting tabs (so row B reads interior and is
via-dropped), and a partner J10, with a project declaring ring 0.13, via 0.25,
hole 0.15 and a Default class via 0.5/0.2.

    python3 tests/test_1775_self_sized_via_board_setup.py
"""
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

import run_utils                                                 # noqa: E402

SRC = os.path.join(ROOT, 'kicad_files', 'qfn_csi_underpad_diff.kicad_pcb')
failures = []


def ok(label, cond, detail=''):
    print(f"  [{'ok' if cond else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not cond:
        failures.append(label)


def make_board(base, class_via=(0.5, 0.2)):
    """The issue's board at BASE.kicad_pcb + .kicad_pro (CLASS_VIA: the
    Default class via, which the GUI fanout tab starts from)."""
    src = open(SRC, encoding='utf-8').read()

    def pad(num, x, y, w, h, net):
        n = f' (net "{net}")' if net else ''
        return (f'\t\t(pad "{num}" smd rect (at {x:g} {y:g}) (size {w:g} {h:g}) '
                f'(layers "F.Cu" "F.Mask" "F.Paste"){n})\n')

    def fp(ref, at, pads):
        return (f'\t(footprint "T:{ref}" (layer "F.Cu") (at {at})\n'
                f'\t\t(property "Reference" "{ref}" (at 0 -1.5 0) (layer "F.SilkS") (hide yes) '
                f'(effects (font (size 1 1) (thickness 0.15))))\n\t\t(attr smd)\n'
                + ''.join(pads) + '\t)\n')
    rows = [pad(2 * i + 1, -3.5 + i, 0, 0.3, 1.0, f'SIG{2 * i + 1}') for i in range(8)]
    rows += [pad(2 * i + 2, -3.0 + i, -2.0, 0.3, 1.0, f'SIG{2 * i + 2}') for i in range(8)]
    tabs = [pad('MP', x, -4.5, 1.2, 1.6, None) for x in (-6, 6)]
    j10 = fp('J10', '22 37.5', [pad(k + 1, -5.25 + 0.7 * k, 0, 0.35, 1.0, f'SIG{k + 1}')
                                for k in range(16)])
    rules = dict(min_clearance=0.15, min_track_width=0.1, min_via_diameter=0.25,
                 min_through_hole_diameter=0.15, min_via_annular_width=0.13,
                 min_hole_clearance=0.2)
    cls = dict(name="Default", clearance=0.15, track_width=0.2, via_diameter=class_via[0],
               via_drill=class_via[1], diff_pair_gap=0.25, diff_pair_width=0.2,
               microvia_diameter=0.3, microvia_drill=0.1, wire_width=6, bus_width=12,
               line_style=0, pcb_color="rgba(0,0,0,0)", schematic_color="rgba(0,0,0,0)",
               priority=2147483647)
    k = src.rindex('\n\t(embedded_fonts no)') + 1
    open(base + '.kicad_pcb', 'w', encoding='utf-8').write(
        src[:k] + fp('J9', '22 30', rows + tabs) + j10 + src[k:])
    json.dump({"board": {"design_settings": {"rules": rules, "rule_severities": {}}},
               "meta": {"filename": os.path.basename(base) + ".kicad_pro", "version": 1},
               "net_settings": {"classes": [cls], "meta": {"version": 3}}},
              open(base + '.kicad_pro', 'w'), indent=2)
    return run_utils.evidence(base + '.kicad_pcb', 'the socket board')


def via_sizes(path):
    import contextlib
    import io
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(path)
    return sorted({(round(v.size, 3), round(v.drill, 3)) for v in pcb.vias})


def run_fanout(board, out, escalation, writeback):
    argv = [sys.executable, '-B', '-X', 'utf8', 'py_router/qfn_fanout.py', board,
            '--output', out, '--component', 'J9', '--nets', 'SIG*', '--width', '0.1',
            '--escalation', escalation] + ([] if writeback else ['--no-fix-drc-settings'])
    r = run_utils.check(argv, accept=True)
    run_utils.evidence(out, 'fanout output')
    log = r.stdout + r.stderr
    m = re.search(r'^JSON_SUMMARY: (.*)$', log, re.M)
    if not m:
        raise AssertionError(f'BROKEN TEST: no JSON_SUMMARY\n{log[-1500:]}')
    return log, json.loads(m.group(1))


def drc_annular(out):
    r = subprocess.run([sys.executable, '-B', '-X', 'utf8', 'py_router/check_drc.py', out],
                       cwd=ROOT, capture_output=True, text=True, encoding='utf-8',
                       errors='replace')
    o = r.stdout + r.stderr
    if 'Traceback' in o:
        raise AssertionError(f'BROKEN TEST: check_drc crashed\n{o[-1500:]}')
    return len(re.findall(r'annular ring too small', o))


def end_to_end(tmp):
    board = make_board(os.path.join(tmp, 'b'))
    print('1. --escalation board raises the self-sized via')
    for wb in (False, True):
        out = os.path.join(tmp, f'board_{int(wb)}.kicad_pcb')
        log, s = run_fanout(board, out, 'board', wb)
        tag = 'writeback on' if wb else 'writeback off'
        ok(f'{tag}: raised to 0.51/0.25', 'raised to 0.51/0.25' in log
           and 'Board Setup via minimums' in log)
        ok(f'{tag}: the vias laid hold the ring', via_sizes(out) == [(0.51, 0.25)],
           repr(via_sizes(out)))
        ok(f'{tag}: no board_setup_vias row', s.get('board_setup_vias') == [],
           repr(s.get('board_setup_vias')))
    ok('check_drc: no annular violation', drc_annular(
        os.path.join(tmp, 'board_0.kicad_pcb')) == 0)
    print('2. default escalation, writeback off: kept and named')
    out = os.path.join(tmp, 'fab_0.kicad_pcb')
    log, s = run_fanout(board, out, 'fab', False)
    ok('the via keeps its built-in size', via_sizes(out) == [(0.45, 0.25)],
       repr(via_sizes(out)))
    ok('a WARNING names the Board Setup minimum',
       'WARNING: Board Setup min_via_annular_width' in log and '8 BELOW it' in log)
    rows = s.get('board_setup_vias') or []
    ok('JSON_SUMMARY board_setup_vias names the 8 vias',
       len(rows) == 1 and rows[0]['rule'] == 'Board Setup min_via_annular_width'
       and rows[0]['below'] == 8 and abs(rows[0]['min'] - 0.13) < 1e-9, repr(rows))
    ok('check_drc agrees (8 annular rows)', drc_annular(out) == 8)
    print('3. default escalation, writeback on: the writeback says it')
    out = os.path.join(tmp, 'fab_1.kicad_pcb')
    log, s = run_fanout(board, out, 'fab', True)
    ok('no board_setup_vias row', s.get('board_setup_vias') == [],
       repr(s.get('board_setup_vias')))
    ok('FAB FLOOR RELAXED printed', 'FAB FLOOR RELAXED' in log)


def units():
    print('4. units')
    import fab_tiers
    from design_rules import run_via_for_rules, via_for_board_setup, DesignRules
    floors = {'annular': 0.13, 'via_diameter': 0.25, 'via_drill': 0.15}
    prev = fab_tiers.get_escalation_policy()
    try:
        fab_tiers.set_fab_tier_from_config({
            'escalation': 'board', 'board_floors': floors, 'fix_drc_settings': False,
            'via_size': 0.45, 'via_drill': 0.25, 'track_width': 0.1, 'clearance': 0.15})
        ok('GUI policy setter: writeback recorded off', not fab_tiers.project_writeback())
        ok('GUI path: run_via_for_rules raises 0.45/0.25',
           run_via_for_rules(None, None, 0.45, 0.25) == (0.51, 0.25),
           repr(run_via_for_rules(None, None, 0.45, 0.25)))
        ok('board_setup=False leaves it', run_via_for_rules(
            None, None, 0.45, 0.25, board_setup=False) == (0.45, 0.25))
        ok('a via that holds them is unchanged', via_for_board_setup(0.5, 0.2) == (0.5, 0.2))
        fab_tiers.set_escalation_policy('fab', floors, writeback=False)
        ok('fab: unchanged', via_for_board_setup(0.45, 0.25) == (0.45, 0.25))
        dr = DesignRules()
        vias = [{'x': 0, 'y': 0, 'size': 0.45, 'drill': 0.25, 'net_id': 1}]
        ok('annular_grade without board floors: no row', dr.annular_grade(vias) == [])
        rows = dr.annular_grade(vias, floors)
        ok('annular_grade with board floors: a Board Setup row',
           len(rows) == 1 and rows[0].get('board_setup') and rows[0]['below'] == 1,
           repr(rows))
    finally:
        fab_tiers.set_escalation_policy(*prev)


def main():
    tmp = tempfile.mkdtemp(prefix='t1775_')
    try:
        units()
        end_to_end(tmp)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    if failures:
        print(f'FAILED: {len(failures)}: {failures}')
        return 1
    print('PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
