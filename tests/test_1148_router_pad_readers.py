#!/usr/bin/env python3
"""#1148: the router's pad readers read pins, not paste or mask windows.

`detect_package_type`, `detect_bga_pitch` and the QFN auto-pick's pad count
read every pad, so an aperture-only pad (no copper layer, no drill, not NPTH:
a thermal pad's split paste windows) counted as a pin. #1143 measured it:
rp2350's 0201s C28/R9 read QFN through their split paste windows, orangecrab
U6 read QFN and is OTHER without its apertures, the BGA pitch moved on 7
tracked boards (glasgow U36/U8 0.1 -> 0.325, rp2350's 0402 caps 0.025 ->
0.64), and the auto-pick ranked by stencil (watchy U4: 73 for 57 pins).

Checks:
  1. Synthetic: a 2-pin chip with four paste windows is OTHER, not QFN; a
     4x4 ball grid's pitch is the grid's with windows between the balls; the
     auto-pick counts pins. An F.Cu+F.Paste pad, an NPTH hole and a drilled
     pad still count.
  2. The tracked witnesses: rp2350 C28/R9 and orangecrab U6 are not QFN;
     glasgow U36/U8 read their 0.325 mm pitch.
  3. Every tracked board: each reader gives the same answer with the
     aperture-only pads removed as with them.

    python3 tests/test_1148_router_pad_readers.py
"""
import contextlib
import copy
import io
import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from kicad_parser import (Footprint, parse_kicad_pcb, detect_package_type,  # noqa: E402
                          detect_bga_pitch, pad_is_aperture_only)
from qfn_fanout import autopick_rank                                     # noqa: E402
from synth import make_pad                                               # noqa: E402

failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def fp(name, pads):
    return Footprint(reference='U1', footprint_name=name, x=0, y=0, rotation=0,
                     layer='F.Cu', pads=pads)


def paste(x, y, num=''):
    return make_pad(0, x, y, num=num, size_x=0.2, size_y=0.2, layers=('F.Paste',))


print("1. synthetic parts")
chip = fp('lib:R_0201', [make_pad(1, -0.3, 0, num='1', size_x=0.3, size_y=0.3),
                         make_pad(2, 0.3, 0, num='2', size_x=0.3, size_y=0.3)]
          + [paste(dx, dy) for dx in (-0.35, -0.25) for dy in (-0.1, 0.1)]
          + [paste(dx, dy) for dx in (0.25, 0.35) for dy in (-0.1, 0.1)])
check("a 2-pin chip with eight paste windows is not a QFN",
      detect_package_type(chip) == 'OTHER', detect_package_type(chip))
check("the auto-pick counts its 2 pins", autopick_rank(chip)[1] == 2,
      str(autopick_rank(chip)))
balls = [make_pad(1, 0.8 * i, 0.8 * j, num=f'{i}{j}', shape='circle',
                  size_x=0.4, size_y=0.4) for i in range(4) for j in range(4)]
windows = [paste(0.8 * i + 0.4, 0.8 * j + 0.4) for i in range(3) for j in range(3)]
grid = fp('lib:Custom', balls + windows)
check("a 0.8 mm grid with windows between the balls reads 0.8",
      abs(detect_bga_pitch(grid) - 0.8) < 1e-9, f"{detect_bga_pitch(grid)}")
kept = fp('lib:X', [make_pad(1, 0, 0, layers=('F.Cu', 'F.Paste')),
                    make_pad(0, 2, 0, pad_type='np_thru_hole', drill=1.0,
                             layers=('*.Cu', '*.Mask')),
                    make_pad(2, 4, 0, pad_type='thru_hole', drill=0.8,
                             layers=('*.Cu',))])
check("copper+paste, NPTH and drilled pads still count",
      autopick_rank(kept)[1] == 3 and not any(pad_is_aperture_only(p) for p in kept.pads))

print("2. tracked witnesses")


def load(name):
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        return parse_kicad_pcb(os.path.join(ROOT, 'kicad_files', name))


rp = load('rp2350_fpga_eensy_prePlane.kicad_pcb')
check("rp2350 C28 and R9 are not QFN",
      [detect_package_type(rp.footprints[r]) for r in ('C28', 'R9')] == ['OTHER', 'OTHER'])
oc = load('orangecrab_ext_pll.kicad_pcb')
check("orangecrab U6 is not QFN", detect_package_type(oc.footprints['U6']) == 'OTHER',
      detect_package_type(oc.footprints['U6']))
gl = load('glasgow_revC.kicad_pcb')
check("glasgow U36/U8 read their 0.325 mm pitch",
      all(abs(detect_bga_pitch(gl.footprints[r]) - 0.325) < 1e-6 for r in ('U36', 'U8')),
      str([round(detect_bga_pitch(gl.footprints[r]), 4) for r in ('U36', 'U8')]))

print("3. every tracked board: the readers ignore apertures")
from run_utils import corpus_boards                                     # noqa: E402
moved = []
n = 0
for b in corpus_boards():
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        try:
            pcb = parse_kicad_pcb(b)
        except Exception:
            continue
    for ref, f in pcb.footprints.items():
        if not any(pad_is_aperture_only(p) for p in f.pads or ()):
            continue
        n += 1
        g = copy.copy(f)
        g.pads = [p for p in f.pads if not pad_is_aperture_only(p)]
        for name, fn in (('package', detect_package_type), ('pitch', detect_bga_pitch),
                         ('pins', lambda q: autopick_rank(q)[1])):
            if fn(f) != fn(g):
                moved.append(f"{os.path.basename(b)} {ref} {name}: {fn(f)} vs {fn(g)}")
check(f"{n} parts with apertures, every reader unmoved", n > 50 and not moved,
      '; '.join(moved[:5]))

if failures:
    print(f"\nFAILED: {failures}")
    sys.exit(1)
print("\nALL PASSED")
