#!/usr/bin/env python3
"""#1731: a board whose Board Setup counts no via height is measured as KiCad
measures it.

KiCad's Board Setup "use stackup height for length calculations"
(`rules.use_height_for_length_calcs` in the .kicad_pro, default on) decides
whether a via contributes its stackup span to a net's length. With it off,
kicad-cli 10.0.6's length and skew DRC charge no via anything: probed on the
#1432 length_cases fixture (a via-in-pad, a via between two inner-layer
tracks, off-centre ends, a through-hole pad's side) and on two pad-to-pad
chains through vias, every reported length was the tracks plus pad-to-die,
exactly. With it on, or absent, the existing per-release model already
matched KiCad. The measure ignored the setting, so on a board that turns it
off every length, skew, length-group target and intra-pair match charged
each via a span KiCad does not grade (about 0.119 mm per via on an 8-layer
DDR route).

`net_queries.via_height_in_length` reads the setting (the live board's on
the GUI path, else the sibling .kicad_pro, else KiCad's default on), and
every via charge reads it: `kicad_net_length_nm` (net_copper_length,
net_match_length, diff_pair_skew, net_signal_length, check_drc, board_score),
the pad-to-pad walk's barrel steps (pin_pair_path_length), and the router's
own `calculate_route_length` / `calculate_via_barrel_length` and the stub
via barrel (group and intra-pair matching).

Checks:
  1. the fixture nets and the chains read KiCad 10.0.6's lengths with the
     setting off (pinned below), on (unchanged), and absent (= on);
  2. pin_pair_path_length, calculate_route_length and the stub via barrel
     follow the setting;
  3. a PCBData's live value (the GUI path) outranks the file;
  4. when KiCad's python is installed: build_pcb_data_from_board reads the
     live Board Setup value, and compare_pcb_data reports no difference.

    python3 tests/test_1731_via_height_setting.py
"""
import contextlib
import io
import json
import os
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

import net_queries as nq                              # noqa: E402
from kicad_parser import parse_kicad_pcb              # noqa: E402
from connectivity import calculate_stub_via_barrel_length  # noqa: E402
from run_utils import evidence                        # noqa: E402

failures = []
FIXTURE = os.path.join(TESTS_DIR, 'fixtures', '1432', 'length_cases.kicad_pcb')

#: kicad-cli 10.0.6 `length_out_of_range` actuals, mm (no pad-to-die on these
#: nets). Arc nets are left out: they are measured on chords (#1432).
KICAD = {
    'off': {'/via_in_pad': 5.0, '/two_inner': 10.0, '/off_centre_in': 10.0,
            '/off_centre_out': 9.8, '/no_centre_end': 5.0, '/th_pad_side': 5.0,
            '/merge_at_via': 7.8243, '/pad_bend': 3.0597, '/circle_pad': 3.0266,
            '/start_in_pad': 10.0, '/chain2': 15.0, '/chain1': 10.0},
    'on': {'/via_in_pad': 5.5525, '/two_inner': 10.535, '/off_centre_in': 10.535,
           '/off_centre_out': 9.8, '/no_centre_end': 5.0, '/th_pad_side': 6.0875,
           '/merge_at_via': 8.3768, '/pad_bend': 3.0597, '/circle_pad': 3.0266,
           '/start_in_pad': 10.0, '/chain2': 16.64, '/chain1': 11.64},
}


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def chain_board():
    """Two pad-to-pad chains on the fixture's 4-layer stackup: F.Cu pad ->
    F.Cu -> via -> In2.Cu -> via -> B.Cu -> B.Cu pad, and F.Cu pad -> F.Cu ->
    via -> B.Cu -> B.Cu pad."""
    with open(FIXTURE, encoding='utf-8') as fh:
        src = fh.read()
    b = src[:src.index(' (net 0 "")')] + ' (net 0 "")\n (net 1 "/chain2")\n (net 2 "/chain1")\n'

    def pad(ref, layer, x, y, net, nid):
        return (f' (footprint "L:{ref}" (layer "{layer}") (at {x} {y})\n'
                f'  (property "Reference" "{ref}" (at 0 -2 0) (layer "F.SilkS") '
                f'(effects (font (size 1 1) (thickness 0.15))))\n'
                f'  (pad "1" smd rect (at 0 0) (size 0.5 0.5) (layers "{layer}") '
                f'(net {nid} "{net}")))\n')

    def seg(x1, y1, x2, y2, layer, nid):
        return (f' (segment (start {x1} {y1}) (end {x2} {y2}) (width 0.1) '
                f'(layer "{layer}") (net {nid}))\n')

    def via(x, y, nid):
        return (f' (via (at {x} {y}) (size 0.3) (drill 0.15) (layers "F.Cu" "B.Cu") '
                f'(net {nid}))\n')
    b += (pad('A1', 'F.Cu', 20, 30, '/chain2', 1) + seg(20, 30, 25, 30, 'F.Cu', 1)
          + via(25, 30, 1) + seg(25, 30, 30, 30, 'In2.Cu', 1) + via(30, 30, 1)
          + seg(30, 30, 35, 30, 'B.Cu', 1) + pad('A2', 'B.Cu', 35, 30, '/chain2', 1))
    b += (pad('B1', 'F.Cu', 20, 40, '/chain1', 2) + seg(20, 40, 25, 40, 'F.Cu', 2)
          + via(25, 40, 2) + seg(25, 40, 30, 40, 'B.Cu', 2)
          + pad('B2', 'B.Cu', 30, 40, '/chain1', 2))
    return b + ')\n'


def stage(tmp, name, text, flag):
    stem = os.path.join(tmp, name)
    with open(stem + '.kicad_pcb', 'w', encoding='utf-8') as fh:
        fh.write(text)
    if flag is not None:
        with open(stem + '.kicad_pro', 'w', encoding='utf-8') as fh:
            json.dump({'meta': {'filename': name + '.kicad_pro', 'version': 3},
                       'board': {'design_settings': {'rules': {
                           'use_height_for_length_calcs': flag}}}}, fh)
    evidence(stem + '.kicad_pcb', name)
    with contextlib.redirect_stdout(io.StringIO()), \
            contextlib.redirect_stderr(io.StringIO()):
        return stem + '.kicad_pcb', parse_kicad_pcb(stem + '.kicad_pcb')


def lengths(pcb):
    return {n.name: nq.net_copper_length(pcb, i) for i, n in pcb.nets.items()
            if n.name in KICAD['on']}


def main():
    nq.use_length_measure_release('10.0.6')
    with open(FIXTURE, encoding='utf-8') as fh:
        cases = fh.read()
    chains = chain_board()
    with tempfile.TemporaryDirectory() as tmp:
        boards = {}
        for flag, tag in ((False, 'off'), (True, 'on'), (None, 'absent')):
            for name, text in (('cases', cases), ('chain', chains)):
                boards[(name, tag)] = stage(tmp, f'{name}_{tag}', text, flag)
        for tag, want in (('off', 'off'), ('on', 'on'), ('absent', 'on')):
            got = {}
            for name in ('cases', 'chain'):
                got.update(lengths(boards[(name, tag)][1]))
            bad = {k: (round(v, 4), KICAD[want][k]) for k, v in got.items()
                   if abs(v - KICAD[want][k]) > 5e-5}
            check(f'setting {tag}: every net reads kicad-cli 10.0.6\'s length',
                  len(got) == len(KICAD[want]) and not bad,
                  f'{len(got)} nets; off: {bad}' if bad else f'{len(got)} nets')

        for tag, on in (('off', False), ('on', True)):
            path, pcb = boards[('chain', tag)]
            check(f'setting {tag}: via_height_in_length reads it from the project',
                  nq.via_height_in_length(pcb) is on)
            nid = next(i for i, n in pcb.nets.items() if n.name == '/chain2')
            pads = pcb.pads_by_net[nid]
            pp = nq.pin_pair_path_length(pcb, nid, pads[0], pads[1])
            want = KICAD[tag]['/chain2']
            check(f'setting {tag}: pin_pair_path_length follows it ({want} mm)',
                  pp is not None and abs(pp - want) < 5e-5, f'{pp}')
            segs = [s for s in pcb.segments if s.net_id == nid]
            vias = [v for v in pcb.vias if v.net_id == nid]
            rl = nq.calculate_route_length(segs, vias, pcb)
            check(f'setting {tag}: calculate_route_length charges '
                  f'{"the barrels" if on else "no barrel"}',
                  (rl > 15.0 + 1e-6) if on else abs(rl - 15.0) < 1e-9, f'{rl:.4f}')
            sv = calculate_stub_via_barrel_length(vias, 'F.Cu', pcb)
            check(f'setting {tag}: the stub via barrel follows it',
                  (sv > 0) if on else sv == 0.0, f'{sv:.4f}')

        # the GUI path: the live board's value outranks the file
        _p, pcb = boards[('chain', 'absent')]
        pcb.use_height_for_length_calcs = False
        got = lengths(pcb)
        check('a live value outranks the file (absent = on, live off)',
              abs(got['/chain2'] - 15.0) < 5e-5 and abs(got['/chain1'] - 10.0) < 5e-5,
              str({k: round(v, 4) for k, v in got.items()}))

        # pcbnew half: build_pcb_data_from_board reads Board Setup.
        # ipc-migration: that builder is the kipy one here, which leaves
        # use_height_for_length_calcs None (kipy exposes no such setting) and
        # sets source_path from the live board's file, so the reader takes the
        # project's value -- what check 1 grades on the files. The pcbnew
        # LoadBoard arm below drives the deleted SWIG builder.
        kpy = None
        if not kpy:
            print('  [skip] pcbnew half: ipc-migration -- the kipy builder reads '
                  'this setting from the project file (check 1)')
        else:
            probe = (
                'import json, sys\n'
                'sys.path.insert(0, sys.argv[1])\n'
                'import pcbnew\n'
                'from kicad_parser import parse_kicad_pcb, build_pcb_data_from_board, compare_pcb_data\n'
                'import net_queries as nq\n'
                'for path in sys.argv[2:]:\n'
                '    pb = build_pcb_data_from_board(pcbnew.LoadBoard(path))\n'
                '    pf = parse_kicad_pcb(path)\n'
                '    print("RESULT " + json.dumps({"board": path, "live": pb.use_height_for_length_calcs,\n'
                '        "board_says": nq.via_height_in_length(pb), "file_says": nq.via_height_in_length(pf),\n'
                '        "diff": [d for d in compare_pcb_data(pb, pf) if "stackup height" in d]}))\n')
            paths = [boards[('chain', t)][0] for t in ('off', 'on', 'absent')]
            r = subprocess.run([kpy, '-c', probe, os.path.join(ROOT, 'py_router')] + paths,
                               capture_output=True, text=True, timeout=600)
            res = [json.loads(ln[len('RESULT '):]) for ln in r.stdout.splitlines()
                   if ln.startswith('RESULT ')]
            check('pcbnew half ran on all three boards', len(res) == 3,
                  (r.stdout + r.stderr)[-400:])
            for row, want in zip(res, (False, True, True)):
                tag = os.path.basename(row['board'])
                check(f'pcbnew: {tag} reads Board Setup {want} on both fronts, no diff',
                      row['live'] is want and row['board_says'] is want
                      and row['file_says'] is want and not row['diff'], str(row))
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
