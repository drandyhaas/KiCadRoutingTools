#!/usr/bin/env python3
"""A QFN fanout leaves a neighbouring chip its half of a shared gap.

  python3 tests/test_qfn_neighbour_escape_room.py [-v]

A QFN stub's 45-degree fan grows until it grazes copper that already exists,
so between two facing chips whichever was fanned FIRST took the whole gap and
the second found no room in front of its own pins. Measured on rein_r1 (two
RP2040s, pin rows 1.7 mm apart): fanned U1-first, U3's USB pair was boxed in
and never coupled; fanned U3-first, it coupled. Now every chip's fan stops at
the midline of a gap it shares with another chip's pins
(`qfn_fanout._neighbour_escape_reserves`), so the result does not depend on
the order the chain fans them in.

Fixture: two 48-pin QFNs (0.4 mm pitch, rows 2.8 mm off centre) whose facing
pad tips are 0.8 mm apart. Three checks:

  1. CONTROL -- the fixture discriminates: fanned with NO neighbour, U1's fan
     reaches past the midline. Without this the checks below could pass on
     fans too short to ever reach it.
  2. In BOTH orders each chip stays on its own side of the midline, and the
     chip fanned FIRST reports fans stopped there (it yields, rather than
     taking the gap).
  3. The copper is identical in both orders.
"""
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
from run_utils import tool  # noqa: E402

N, PITCH, ROW = 12, 0.4, 2.8        # pads per side, pitch, pad-row offset
PAD_L, TIP_GAP = 0.8, 0.8           # pad length along escape, facing tip gap
U1_Y = 100.0
U2_Y = U1_Y + 2 * ROW + PAD_L + TIP_GAP
MID = (U1_Y + ROW + PAD_L / 2 + U2_Y - ROW - PAD_L / 2) / 2
TRACK = 0.1


def qfn(ref, x, y, net0, uid):
    pads, n = [], 0
    for side in ('L', 'B', 'R', 'T'):
        for k in range(N):
            o = -PITCH * (N - 1) / 2 + PITCH * k
            px, py, sx, sy = {'L': (-ROW, o, PAD_L, 0.2), 'R': (ROW, -o, PAD_L, 0.2),
                              'B': (o, ROW, 0.2, PAD_L), 'T': (-o, -ROW, 0.2, PAD_L)}[side]
            n += 1
            pads.append(f'    (pad "{n}" smd rect (at {px} {py}) (size {sx} {sy}) '
                        f'(layers "F.Cu") (net {net0 + n} "/{ref}_P{n}"))')
    return (f'  (footprint "Package_DFN_QFN:QFN-48_6x6mm_P0.4mm" (at {x} {y}) '
            f'(layer "F.Cu")\n    (attr smd)\n'
            f'    (fp_text reference "{ref}" (at 0 -4) (layer "F.SilkS") '
            f'(uuid "aaaaaaaa-0000-0000-0000-00000000000{uid}"))\n'
            + '\n'.join(pads) + '\n  )\n')


def board(with_u2):
    refs = (('U1', 0), ('U2', 4 * N)) if with_u2 else (('U1', 0),)
    nets = '\n'.join(f'  (net {i} "/{r}_P{i - b}")' for r, b in refs
                     for i in range(b + 1, b + 4 * N + 1))
    edges = ''.join(f'  (gr_line (start {a} {b}) (end {c} {d}) (layer "Edge.Cuts") '
                    f'(width 0.1))\n' for a, b, c, d in ((85, 85, 115, 85),
                                                         (115, 85, 115, 120),
                                                         (115, 120, 85, 120),
                                                         (85, 120, 85, 85)))
    return ('(kicad_pcb (version 20240108) (generator test)\n'
            '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
            '  (net 0 "")\n' + nets + '\n' + edges
            + qfn('U1', 100, U1_Y, 0, 1)
            + (qfn('U2', 100, U2_Y, 4 * N, 2) if with_u2 else '') + ')\n')


def fan(src, ref, out):
    r = subprocess.run([sys.executable, tool('qfn_fanout.py'), src,
                        '--component', ref, '--output', out,
                        '--width', str(TRACK), '--clearance', '0.1', '--nets', '*'],
                       capture_output=True, text=True, cwd=ROOT)
    text = r.stdout + r.stderr
    assert r.returncode == 0 and os.path.isfile(out), text[-1500:]
    return text


def deepest(path):
    from kicad_parser import parse_kicad_pcb
    p = parse_kicad_pcb(path)
    u1 = [max(s.start_y, s.end_y) for s in p.segments
          if p.nets[s.net_id].name.startswith('/U1_')]
    u2 = [min(s.start_y, s.end_y) for s in p.segments
          if p.nets[s.net_id].name.startswith('/U2_')]
    geo = sorted((p.nets[s.net_id].name, round(s.start_x, 4), round(s.start_y, 4),
                  round(s.end_x, 4), round(s.end_y, 4)) for s in p.segments)
    return (max(u1) if u1 else None), (min(u2) if u2 else None), geo


def main():
    verbose = '-v' in sys.argv
    with tempfile.TemporaryDirectory() as wd:
        alone = os.path.join(wd, 'alone.kicad_pcb')
        open(alone, 'w').write(board(False))
        fan(alone, 'U1', os.path.join(wd, 'alone_U1.kicad_pcb'))
        reach, _, _ = deepest(os.path.join(wd, 'alone_U1.kicad_pcb'))
        assert reach > MID + TRACK, (
            f"CONTROL: the fixture must let an unrestricted fan cross the "
            f"midline ({reach:.3f} vs {MID:.3f}), or nothing below is tested")
        print(f"PASS control: with no neighbour U1's fan reaches {reach:.2f} mm, "
              f"{reach - MID:.2f} mm past the midline")

        both = os.path.join(wd, 'both.kicad_pcb')
        open(both, 'w').write(board(True))
        results = {}
        for first, second in (('U1', 'U2'), ('U2', 'U1')):
            a = os.path.join(wd, f'{first}.kicad_pcb')
            b = os.path.join(wd, f'{first}{second}.kicad_pcb')
            t1 = fan(both, first, a)
            t2 = fan(a, second, b)
            if verbose:
                print(t1, t2)
            assert 'Neighbour escape room' in t1, (
                f"{first} fanned first must yield at the midline, not take "
                f"the gap:\n{t1[-800:]}")
            u1, u2, geo = deepest(b)
            assert u1 is not None and u1 <= MID - TRACK / 2 + 1e-6, (first, u1, MID)
            assert u2 is not None and u2 >= MID + TRACK / 2 - 1e-6, (first, u2, MID)
            results[first] = geo
            print(f"PASS {first} then {second}: U1 reaches {u1:.2f}, U2 {u2:.2f}, "
                  f"midline {MID:.2f}; the first chip yielded")
        assert results['U1'] == results['U2'], "copper differs between orders"
        print("PASS order independence: identical copper in both orders")
    print("ALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
