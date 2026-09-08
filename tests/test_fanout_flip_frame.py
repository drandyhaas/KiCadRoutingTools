#!/usr/bin/env python3
"""A BGA on the BACK fans out as the mirror of the same BGA on the front
(`bga_fanout/flip_frame.py`).

Measured before the wrapper on a real board and its mirror (the #622
pose gate): 18 of 51 escapes differed -- the gaps along a face assigned
in the other order, one net on the other layer with a via the front did
not need -- because the engine was written for a part on F. The wrapper
turns the board over in memory, runs the engine on the part now on F,
and mirrors the copper back.

Covered here, wx-free and pcbnew-free:
  1. `to_front_frame` on real boards: every pad lands at (x, 2*CY - y)
     on the swapped layer, its locals re-derived under the parser's own
     `local_to_global` convention; segments and vias mirrored and
     swapped; the map is an involution (turning over twice is the
     identity) and the board bounds are their own mirror.
  2. the symmetry itself on a synthetic BGA: a front-side fixture and an
     INDEPENDENTLY built back-side twin (not made with the function under
     test), fanned out with the chain's engine call; the front's copper
     mirrored must equal the back's exactly.
  3. a CHANGE DETECTOR: with the wrapper disabled the back-side fanout
     must differ from the mirror, so this file cannot go vacuous if the
     engine is ever made face-symmetric some other way (then the
     detector fails and says so, which is the moment to retire it).
"""
import io
import contextlib
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)

from kicad_parser import parse_kicad_pcb, local_to_global, BoardInfo, Footprint  # noqa: E402
from bga_fanout import generate_bga_fanout  # noqa: E402
from bga_fanout import flip_frame as ff  # noqa: E402
from synth import make_pad, make_net, make_pcb  # noqa: E402

KICAD = os.path.join(HERE, '..', 'kicad_files')
BOARDS = ['orangecrab_ext_pll.kicad_pcb', 'haasoscope_pro_max_test.kicad_pcb']


def _ok(results, name, cond, detail=''):
    results.append((name, bool(cond)))
    print(f'  {"PASS" if cond else "FAIL"}  {name}' + (f'  ({detail})' if detail else ''))


# --- 1. the transform on real boards ------------------------------------------
def test_transform_on_real_boards(results):
    for name in BOARDS:
        path = os.path.join(KICAD, name)
        if not os.path.exists(path):
            print(f'  skip {name}: not present')
            continue
        with contextlib.redirect_stdout(io.StringIO()):
            pcb = parse_kicad_pcb(path)
        CY = ff.mirror_axis(pcb)
        rp, back = ff.to_front_frame(pcb, next(iter(pcb.footprints)))
        bad_pos = bad_lay = bad_loc = 0
        n = 0
        for ref, f0 in pcb.footprints.items():
            f1 = rp.footprints[ref]
            if ff.other_layer(f0.layer) != f1.layer:
                bad_lay += 1
            for p0, p1 in zip(f0.pads, f1.pads):
                n += 1
                if abs(p1.global_x - p0.global_x) > 1e-6 or abs(p1.global_y - (2 * CY - p0.global_y)) > 1e-6:
                    bad_pos += 1
                if sorted(p1.layers) != sorted(ff.other_layer(L) for L in p0.layers):
                    bad_lay += 1
                gx, gy = local_to_global(f1.x, f1.y, f1.rotation, p1.local_x, p1.local_y)
                if abs(gx - p1.global_x) > 1e-6 or abs(gy - p1.global_y) > 1e-6:
                    bad_loc += 1
        _ok(results, f'{name}: every pad mirrored', bad_pos == 0, f'{n} pads, {bad_pos} off')
        _ok(results, f'{name}: every layer swapped', bad_lay == 0, f'{bad_lay} wrong')
        _ok(results, f'{name}: pad locals consistent with the parser convention',
            bad_loc == 0, f'{bad_loc} inconsistent')
        bad_seg = sum(1 for s0, s1 in zip(pcb.segments, rp.segments)
                      if abs(s1.start_y - (2 * CY - s0.start_y)) > 1e-6
                      or abs(s1.end_y - (2 * CY - s0.end_y)) > 1e-6
                      or s1.layer != ff.other_layer(s0.layer))
        bad_via = sum(1 for v0, v1 in zip(pcb.vias, rp.vias)
                      if abs(v1.y - (2 * CY - v0.y)) > 1e-6 or abs(v1.x - v0.x) > 1e-6)
        _ok(results, f'{name}: segments and vias mirrored',
            bad_seg == 0 and bad_via == 0, f'{len(pcb.segments)} segs {len(pcb.vias)} vias')
        # involution
        rp2, _ = ff.to_front_frame(rp, next(iter(rp.footprints)))
        back_pos = sum(1 for f0, f2 in zip(pcb.footprints.values(), rp2.footprints.values())
                       for p0, p2 in zip(f0.pads, f2.pads)
                       if abs(p2.global_x - p0.global_x) > 1e-6 or abs(p2.global_y - p0.global_y) > 1e-6
                       or sorted(p2.layers) != sorted(p0.layers))
        _ok(results, f'{name}: turning over twice is the identity', back_pos == 0)
        if pcb.board_info and pcb.board_info.board_bounds:
            b0, b1 = pcb.board_info.board_bounds, rp.board_info.board_bounds
            _ok(results, f'{name}: the bounds are mirrored about the axis, on the lattice',
                abs(b1[1] - (2 * CY - b0[3])) < 1e-9 and abs(b1[3] - (2 * CY - b0[1])) < 1e-9
                and abs(2 * CY / 0.1 - round(2 * CY / 0.1)) < 1e-9, f'axis {CY}')


# --- 2. the symmetry on a synthetic BGA ----------------------------------------
N, PITCH, BALL = 6, 0.8, 0.4


def _bi():
    return BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'}, copper_layers=['F.Cu', 'B.Cu'],
                     board_bounds=(0.0, 0.0, 20.0, 20.0))


def _fixture(side):
    """A 6x6 BGA at (10, 10), its 20 outer-ring balls on 20 nets, plus a
    lone SMD pad (a foreign part off-centre, so the escape order has
    something to be asymmetric about). `side` 'F' or 'B'; the back-side
    twin is built here from the front's numbers, NOT with to_front_frame."""
    cx, cy = 10.0, 10.0
    CY = 10.0                                  # the bounds' centre line
    pads, nets, byid = [], {}, {}
    k = 0
    for r in range(N):
        for c in range(N):
            lx = (c - (N - 1) / 2) * PITCH
            ly = (r - (N - 1) / 2) * PITCH
            outer = r in (0, N - 1) or c in (0, N - 1)
            nid = 0
            if outer:
                k += 1
                nid = k
            gx, gy = cx + lx, cy + ly
            layers = ('F.Cu',)
            if side == 'B':
                gy = 2 * CY - gy
                layers = ('B.Cu',)
            pad = make_pad(nid, gx, gy, ref='U1', num=f'{chr(65 + r)}{c + 1}',
                           net_name=f'N{nid}' if nid else '', size_x=BALL, size_y=BALL,
                           shape='circle', layers=layers,
                           local_x=gx - cx, local_y=gy - cy)   # rotation 0: local = global - centre
            pads.append(pad)
            if nid:
                nets[nid] = make_net(nid, f'N{nid}', pads=[pad])
                byid.setdefault(nid, []).append(pad)
    fp = Footprint(reference='U1', footprint_name='test:BGA36', x=cx, y=cy, rotation=0.0,
                   layer='F.Cu' if side == 'F' else 'B.Cu', pads=pads)
    # the far end of every net: a row of pads 6 mm east (a net with one
    # pad has nowhere to go and the engine rightly does nothing)
    far = []
    for i, nid in enumerate(sorted(n for n in nets)):
        fy_ = 4.0 + 0.6 * i
        gy = fy_ if side == 'F' else 2 * CY - fy_
        q = make_pad(nid, 17.0, gy, ref='J1', num=str(i + 1), net_name=f'N{nid}',
                     size_x=0.4, size_y=0.4, layers=('F.Cu',) if side == 'F' else ('B.Cu',),
                     local_x=0.0, local_y=gy - (10.0 if side == 'F' else 10.0))
        far.append(q)
        nets[nid].pads.append(q)
        byid[nid].append(q)
    j1 = Footprint(reference='J1', footprint_name='test:J', x=17.0, y=10.0, rotation=0.0,
                   layer='F.Cu' if side == 'F' else 'B.Cu', pads=far)
    # the foreign pad: 3 mm east of the array, above its centre line on F
    fy = 8.2 if side == 'F' else 2 * CY - 8.2
    fpad = make_pad(99, 15.0, fy, ref='R1', num='1', net_name='FOREIGN', size_x=0.6, size_y=0.8,
                    layers=('F.Cu',) if side == 'F' else ('B.Cu',), local_x=0.0, local_y=0.0)
    r1 = Footprint(reference='R1', footprint_name='test:R', x=15.0, y=fy, rotation=0.0,
                   layer='F.Cu' if side == 'F' else 'B.Cu', pads=[fpad])
    nets[99] = make_net(99, 'FOREIGN', pads=[fpad])
    byid[99] = [fpad]
    pcb = make_pcb(nets=nets, footprints={'U1': fp, 'R1': r1, 'J1': j1}, pads_by_net=byid,
                   board_info=_bi(), zones=[])
    pcb._fanout_all_foreign_immovable = True
    return pcb


def _fan(pcb):
    names = [n.name for n in pcb.nets.values() if n.name.startswith('N')]
    with contextlib.redirect_stdout(io.StringIO()):
        tracks, vias, _rm, failed = generate_bga_fanout(
            pcb.footprints['U1'], pcb, net_filter=names, layers=['F.Cu', 'B.Cu'],
            track_width=0.1, clearance=0.1, via_size=0.25, via_drill=0.15,
            exit_margin=0.5, escape_method='auto', plane_drop='off')
    return tracks, vias, failed


def _key(tracks, vias, mirror=False, CY=10.0):
    def m(p):
        return (round(p[0], 4), round(2 * CY - p[1], 4)) if mirror else (round(p[0], 4), round(p[1], 4))
    segs = sorted((m(t['start']), m(t['end']), ff.other_layer(t['layer']) if mirror else t['layer'], t['net_id'])
                  for t in tracks)
    vs = sorted((m((v['x'], v['y'])), v['net_id']) for v in vias)
    return segs, vs


def test_symmetry_on_synthetic_bga(results):
    front, back = _fixture('F'), _fixture('B')
    tf, vf, failf = _fan(front)
    tb, vb, failb = _fan(back)
    _ok(results, 'the front fixture fans out', tf and not failf, f'{len(tf)} tracks {len(vf)} vias')
    _ok(results, 'the back fixture fans out', tb and not failb, f'{len(tb)} tracks {len(vb)} vias')
    kf, kb = _key(tf, vf, mirror=True), _key(tb, vb)
    _ok(results, 'the back-side fanout is the mirror of the front-side one',
        kf == kb, f'{sum(1 for a, b in zip(kf[0], kb[0]) if a != b)} segment(s) differ')
    # 3. the change detector: the wrapper off, the engine alone must differ
    saved = ff.is_back_side
    try:
        ff.is_back_side = lambda fp: False
        tb2, vb2, _f = _fan(_fixture('B'))
    finally:
        ff.is_back_side = saved
    kb2 = _key(tb2, vb2)
    _ok(results, 'CHANGE DETECTOR: without the wrapper the back differs from the mirror',
        kb2 != kf, '' if kb2 != kf else
        'the engine is face-symmetric on this fixture -- retire the detector')


def main():
    results = []
    print('to_front_frame on real boards')
    test_transform_on_real_boards(results)
    print('the symmetry on a synthetic BGA')
    test_symmetry_on_synthetic_bga(results)
    n_ok = sum(1 for _, ok in results if ok)
    print('=' * 60)
    print(f'{n_ok}/{len(results)} flip-frame tests passed')
    return 0 if n_ok == len(results) else 1


if __name__ == '__main__':
    sys.exit(main())
