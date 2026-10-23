#!/usr/bin/env python3
"""route.py removes the same-net stacked copper it ships (#162, gp14 H03).

KiCad never flags a same-net stack, and route.py removed none on a zoned
net: the strict collapse skips zoned nets, the write-list via dedup is
vias-only, and the stacked-copper census reads the board BEFORE the plane
finalize, whose pad repair lays the copies (two pads tapped into one reused
via each lay the shared run into it). One-Air-Max's route step (gp14 07,
s3_diff -> s4) shipped 54 surplus copies on GND/PGND B.Cu, most a 0.127 mm
tap copy under the 0.3 mm power track, and every later step re-shipped them.

_late_stacked_dedup162 now removes them at the end of the run, from the
board it ships (the written file on the CLI, the write model on the GUI):
the widest copy of a same-net, same-layer, same-span group stays.

  1. remove_segments_from_content(match_width=True) takes the copy of the
     named width, not the first block of the span;
  2. CLI: a 0.127 copy under a 0.3 track and a byte-identical twin both go;
     the 0.3 track and one twin stay, and the count says so;
  3. --keep-input-copper: an input copy stays; a locked group is left alone;
  4. GUI: the copy leaves its result's new_segments and pcb_data.

    python3 tests/test_162_late_stacked_dedup.py
"""
import os
import re
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for p in (ROOT, TESTS_DIR, os.path.join(ROOT, 'py_router')):
    if p not in sys.path:
        sys.path.insert(0, p)

from synth import RoutingBoard                                  # noqa: E402
from kicad_parser import parse_kicad_pcb                        # noqa: E402
from improvement_gate import copper_signature                   # noqa: E402
from run_utils import evidence                                  # noqa: E402

FAILS = []


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name
          + (f'  {detail}' if detail else ''))


def board(tmp, tag, extra):
    """GND pads joined by a 0.3 track; `extra(rb)` adds the stack."""
    rb = RoutingBoard()
    rb.pad('G1', 5, 15, 'GND')
    rb.pad('G2', 25, 15, 'GND')
    rb.seg(5, 15, 15, 15, 'GND', w=0.3)
    rb.seg(15, 15, 25, 15, 'GND', w=0.3)
    base = os.path.join(tmp, f'{tag}_base.kicad_pcb')
    with open(base, 'w', encoding='utf-8') as fh:
        fh.write(rb.text())
    extra(rb)
    path = os.path.join(tmp, f'{tag}.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(rb.text())
    return base, path


def widths(path):
    txt = open(evidence(path, 'the deduped board'), encoding='utf-8').read()
    return sorted(float(w) for w in re.findall(
        r'\(segment \(start 5 15\) \(end 15 15\) \(width ([\d.]+)\)', txt))


def match_width(tmp):
    print('1. the width-aware strip takes the named copy')
    from kicad_writer import remove_segments_from_content
    _b, path = board(tmp, 'mw', lambda rb: rb.seg(5, 15, 15, 15, 'GND', w=0.127))
    pcb = parse_kicad_pcb(path)
    thin = [s for s in pcb.segments if abs(s.width - 0.127) < 1e-6]
    txt = open(path, encoding='utf-8').read()
    out, n = remove_segments_from_content(txt, thin, match_width=True)
    left = re.findall(r'\(segment \(start 5 15\) \(end 15 15\) \(width ([\d.]+)\)', out)
    check('the 0.127 block goes, the 0.3 block stays', n == 1 and left == ['0.3'],
          f'removed {n}, left {left}')
    out, n = remove_segments_from_content(txt, thin)
    left = re.findall(r'\(segment \(start 5 15\) \(end 15 15\) \(width ([\d.]+)\)', out)
    check('control: without it the first block of the span goes (the 0.3)',
          n == 1 and left == ['0.127'], f'left {left}')


def cli(tmp):
    print('2. CLI: the copies leave the written file')
    from route import _late_stacked_dedup162

    def stack(rb):
        rb.seg(5, 15, 15, 15, 'GND', w=0.127)     # a tap copy under the track
        rb.seg(15, 15, 25, 15, 'GND', w=0.3)      # a byte-identical twin
    base, path = board(tmp, 'cli', stack)
    b = parse_kicad_pcb(base)
    sig = copper_signature(b.segments, b.vias,
                           {i: n.name for i, n in b.nets.items()}.get)
    n = _late_stacked_dedup162(None, path, False, None, None, sig, {'GND'},
                               False)
    pcb = parse_kicad_pcb(path)
    check('two copies removed, the 0.3 track and one twin stay',
          n == 2 and len(pcb.segments) == 2 and widths(path) == [0.3],
          f'removed {n}, {len(pcb.segments)} segments left')

    print('3. --keep-input-copper, and a locked group')
    _b, path = board(tmp, 'keep', lambda rb: rb.seg(5, 15, 15, 15, 'GND', w=0.127))
    pcb = parse_kicad_pcb(path)
    sig_all = copper_signature(pcb.segments, pcb.vias,
                               {i: n.name for i, n in pcb.nets.items()}.get)
    n = _late_stacked_dedup162(None, path, False, None, None, sig_all, {'GND'},
                               True)
    check('an input copy is read-only under --keep-input-copper',
          n == 0 and widths(path) == [0.127, 0.3], f'removed {n}')
    _b, path = board(tmp, 'lock', lambda rb: rb.seg(5, 15, 15, 15, 'GND',
                                                     w=0.127, locked=True))
    n = _late_stacked_dedup162(None, path, False, None, None, sig, {'GND'},
                               False)
    check('a group with locked copper is left alone',
          n == 0 and widths(path) == [0.127, 0.3], f'removed {n}')


def gui(tmp):
    print('4. GUI: the copy leaves the write model')
    from route import _late_stacked_dedup162
    base, path = board(tmp, 'gui', lambda rb: None)
    pcb = parse_kicad_pcb(path)
    gnd = next(i for i, n in pcb.nets.items() if n.name == 'GND')
    trunk = [s for s in pcb.segments if s.start_x == 5][0]
    import copy as _copy
    tap = _copy.copy(trunk)
    tap.width = 0.127
    pcb.segments.append(tap)
    sig = copper_signature([s for s in pcb.segments if s is not tap], [],
                           {i: n.name for i, n in pcb.nets.items()}.get)
    rd = {'results': [{'net_id': gnd, 'new_segments': [tap], 'new_vias': []}]}

    def write_model(r):
        segs = [s for s in pcb.segments if s is not tap]
        segs += [s for x in r['results'] for s in x['new_segments']]
        return {gnd: segs}, {}
    n = _late_stacked_dedup162(pcb, None, True, rd, write_model, sig, {'GND'},
                               False)
    check("the tap copy left its result and pcb_data; no input strip",
          n == 1 and rd['results'][0]['new_segments'] == []
          and tap not in pcb.segments and not rd.get('segments_to_remove'),
          f'removed {n}')


def main():
    with tempfile.TemporaryDirectory() as tmp:
        match_width(tmp)
        cli(tmp)
        gui(tmp)
    print(f"\n{'ALL PASS' if not FAILS else f'{len(FAILS)} FAILED: ' + ', '.join(FAILS)}")
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
