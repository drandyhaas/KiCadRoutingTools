#!/usr/bin/env python3
"""#1376: KiCad's `track_dangling` is reported by the cross-check.

kicad_drc_compare read the copper, web, thermal and courtyard classes and
dropped `track_dangling`, so a delivered board that check_weird and
check_orphan_stubs call clean (a short tail past a T or via anchor, a stub
lying on a track that also touches its far end) shipped KiCad's warnings with
nothing in the repo naming them. It is now a labelled channel: graded at the
project's severity (a warning by default), baseline-subtracted, reported with
the net and position, and never part of the copper match, the CONSISTENT
verdict or the exit code.

  1. parse_kicad_report keeps a warning-severity track_dangling with its net
     and position, and still drops a warning-severity copper class.
  2. the grading core counts it apart from the copper count, subtracts a
     baseline twin, and lists the summary key.
  3. (kicad-cli) a real board with a joint stub and a dead tail: KiCad's two
     reports come through the channel.

    python3 tests/test_1376_track_dangling_channel.py
"""
import contextlib
import io
import os
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for _d in ('py_router', 'py_placer', 'py_tools', os.path.join('tests', 'stress'),
           'tests'):
    sys.path.insert(0, os.path.join(ROOT, _d))

import kicad_drc_compare as kdc                      # noqa: E402
from copy_board import copy_board                    # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'cap_chain.kicad_pcb')
SKIP_EXIT = 77
failures = []


def ok(label, cond, detail=''):
    print(f"  [{'ok' if cond else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not cond:
        failures.append(label)


def dangling(net, x, y, severity='warning'):
    return {'type': 'track_dangling', 'severity': severity,
            'description': 'Track has unconnected end',
            'items': [{'description': f'Track [{net}] on F.Cu, length 0.3536 mm',
                       'pos': {'x': x, 'y': y}}]}


def main():
    print("1: parse_kicad_report")
    rows = kdc.parse_kicad_report({'violations': [
        dangling('+3V3', 74.2, 112.6),
        {'type': 'clearance', 'severity': 'warning', 'items': []}]})
    dg = [r for r in rows if r['type'] == 'track_dangling']
    ok('a warning-severity track_dangling is graded',
       len(dg) == 1 and dg[0]['nets'] == frozenset({'+3V3'}), str(dg))
    ok('it sits at the reported position', dg and dg[0]['pos'] == (74.2, 112.6))
    ok('a warning-severity copper class stays out',
       all(r['type'] != 'clearance' for r in rows))

    print("2: the grading core")
    final_items = kdc.parse_kicad_report({'violations': [
        dangling('+3V3', 74.2, 112.6), dangling('GND', 10.0, 20.0)]})
    base_items = kdc.parse_kicad_report({'violations': [
        dangling('GND', 10.0, 20.0)]})
    saved = kdc.kicad_items_for
    kdc.kicad_items_for = lambda b, c=None: (
        (base_items if b.endswith('base.kicad_pcb') else final_items), None)
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            d = kdc.compare_board_data(BOARD)
        ok('both are reported', d['kicad_track_dangling'] == 2
           and sorted(sorted(t['nets'])[0] for t in d['track_dangling_items'])
           == ['+3V3', 'GND'], str(d.get('kicad_track_dangling')))
        ok('never in the copper count or the verdict',
           d['kicad'] == 0 and d['kicad_only'] == 0
           and d['verdict'] == 'CONSISTENT', f"{d['kicad']} {d['verdict']}")
        ok('a summary key', 'kicad_track_dangling' in kdc._SUMMARY_KEYS)
        tmp = tempfile.mkdtemp(prefix='t1376_')
        try:
            base = os.path.join(tmp, 'base.kicad_pcb')
            with contextlib.redirect_stdout(io.StringIO()):
                copy_board(BOARD, base)
                d = kdc.compare_board_data(BOARD, baseline=base)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)
        ok('a baseline twin is subtracted, one for one',
           d['kicad_track_dangling'] == 1
           and d['track_dangling_preexisting'] == 1
           and sorted(d['track_dangling_items'][0]['nets']) == ['+3V3'],
           str((d['kicad_track_dangling'], d['track_dangling_preexisting'])))
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            kdc.compare_board(BOARD)
        out = buf.getvalue()
        ok('the report line and the item lines name them',
           'dangling=2' in out and 'TRACK-DANGLING' in out, out[-300:])
    finally:
        kdc.kicad_items_for = saved

    print("3: a real KiCad report")
    ran = True
    if not (os.path.isfile(kdc.KICAD_CLI) or shutil.which(kdc.KICAD_CLI)):
        print('  NOT RUN: no kicad-cli here -- this is not a pass')
        ran = False
    else:
        from synth import RoutingBoard
        rb = RoutingBoard(w=160, h=120)
        rb.pad('U4', 115.758, 99.952, '/OSCO', sx=1.55, sy=0.3)
        rb.pad('Y1', 113.2, 100.8, '/OSCO', sx=0.4, sy=0.4)
        rb.seg(115.758, 99.952, 114.683, 99.952, '/OSCO', w=0.15)
        rb.seg(114.683, 99.952, 113.4, 99.0, '/OSCO', w=0.15)
        rb.seg(113.742133, 99.253867, 113.7, 99.3, '/OSCO', w=0.25)
        rb.seg(113.7, 99.3, 113.6, 99.3, '/OSCO', w=0.25)
        rb.seg(113.6, 99.3, 113.2, 99.7, '/OSCO', w=0.25)
        rb.seg(113.2, 99.7, 113.2, 100.8, '/OSCO', w=0.25)
        tmp = tempfile.mkdtemp(prefix='t1376k_')
        try:
            path = os.path.join(tmp, 'osco.kicad_pcb')
            with open(path, 'w', encoding='utf-8', newline='\n') as fh:
                fh.write(rb.text())
            items, err = saved(path)
            dg = [r for r in (items or []) if r['type'] == 'track_dangling']
            ok('KiCad flags the joint stub and the host\'s dead stretch, and '
               'both come through', not err and len(dg) == 2
               and all(r['nets'] == frozenset({'/OSCO'}) for r in dg),
               f'{err} {dg}')
        finally:
            shutil.rmtree(tmp, ignore_errors=True)

    if failures:
        print(f"\nFAIL ({len(failures)}): " + '; '.join(failures))
        return 1
    print("\nAll checks passed" if ran else "\nUnit checks passed; kicad-cli leg NOT RUN")
    if not ran:
        print("SKIP: the real-report leg needs kicad-cli")
    return 0 if ran else SKIP_EXIT


if __name__ == '__main__':
    sys.exit(main())
