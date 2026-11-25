#!/usr/bin/env python3
"""Before a group's arrays are fanned out: a ball joined to its pad beneath the array, and a tie that leaves a net no
way taken out (awx/field_ties.beneath_nets, lay_beneath; awx/group_ends.ties_boxed).

  python3 tests/test_622_beneath_joins.py

A net of one ball whose only other pad stands under its array on the far face -- a test point, a calibration
resistor -- is a millimetre of copper to a hand: a dog-bone and a short track on the back. Fanned out as any signal's,
the ball's escape led out to the array's edge on a layer of its own, a neighbour took the via site beside the pad,
and no later step found a way back in between the field's vias. And a supply tie laid on the fanned-out field stood
its via a third of a millimetre in front of another signal's edge stub: the walls' test passed the field, the route
step's own plan found the net no way on it.

1. beneath_nets: the net with a ball and one far-face pad under the array -- not one with a pad outside the array's
   box, not one whose other pad is on the array's own face, not a net named to be skipped;
2. lay_beneath joins it whole: a stub on the array's face to a via on one of the four sites round the ball, a track
   on the far face from the via to the pad; the ball and the pad then carry copper;
3. two such nets that want one site are both joined, their vias apart;
4. ties_boxed: a tie that closes the one gap a net had is named and taken out, a tie elsewhere is left.
"""
import math
import os
import sys
import tempfile
import uuid

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in ('awx', 'py_router', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _d))

try:
    import dataclasses
    import rules  # noqa: E402
    rules.install_defaults()
    import braid  # noqa: E402,F401  (read at the rules' own sizes, before this file's are stood in)
    import field_ties  # noqa: E402
    import group_ends  # noqa: E402
except ImportError as exc:
    print(f'SKIP: {exc}')
    sys.exit(77)

N, BAD = [0], []
U = lambda: str(uuid.uuid4())  # noqa: E731


def check(what, got, want):
    N[0] += 1
    ok = got == want
    if not ok:
        BAD.append(what)
    print(f'{"ok  " if ok else "FAIL"} {what}' + ('' if ok else f': got {got!r}, want {want!r}'))


def write_board(path, parts, w=20.0, h=20.0, tracks=()):
    """a board: `parts` [(ref, layer, [(pad, x, y, net, kind)])] -- kind 'smd' (0.4 mm round, on the part's layer) or
    'thru' (0.8 mm, drill 0.4) --, `tracks` [(x1, y1, x2, y2, layer, net)], the outline (0, 0)-(w, h)"""
    nets = sorted({p[3] for _r, _l, ps in parts for p in ps} | {t[5] for t in tracks})
    nid = {n_: i + 1 for i, n_ in enumerate(nets)}
    L = ['(kicad_pcb', '\t(version 20241229)', '\t(generator "pcbnew")', '\t(generator_version "9.0")',
         '\t(general (thickness 1.6) (legacy_teardrops no))', '\t(paper "A4")',
         '\t(layers (0 "F.Cu" signal) (2 "B.Cu" signal) (5 "F.SilkS" user) (1 "F.Mask" user) (3 "B.Mask" user)'
         ' (25 "Edge.Cuts" user) (35 "F.Fab" user))',
         '\t(setup (pad_to_mask_clearance 0))', '\t(net 0 "")']
    L += [f'\t(net {i} "{n_}")' for n_, i in nid.items()]
    for ref, layer, ps in parts:
        x0, y0 = ps[0][1], ps[0][2]
        L += [f'\t(footprint "synth:P" (layer "{layer}") (uuid "{U()}") (at {x0} {y0})',
              f'\t\t(property "Reference" "{ref}" (at 0 -1.2 0) (layer "F.SilkS") (uuid "{U()}") (effects (font (size 0.5 0.5) (thickness 0.08))))']
        for num, x, y, net, kind in ps:
            at = f'(at {round(x - x0, 4)} {round(y - y0, 4)})'
            if kind == 'thru':
                L.append(f'\t\t(pad "{num}" thru_hole circle {at} (size 0.8 0.8) (drill 0.4) (layers "*.Cu" "*.Mask") '
                         f'(net {nid[net]} "{net}") (uuid "{U()}"))')
            else:
                L.append(f'\t\t(pad "{num}" smd circle {at} (size 0.4 0.4) (layers "{layer}" '
                         f'"{layer[0]}.Mask") (net {nid[net]} "{net}") (uuid "{U()}"))')
        L.append('\t)')
    for x1, y1, x2, y2, layer, net in tracks:
        L.append(f'\t(segment (start {x1} {y1}) (end {x2} {y2}) (width 0.15) (layer "{layer}") (net {nid[net]}) '
                 f'(uuid "{U()}"))')
    for (x1, y1), (x2, y2) in (((0, 0), (w, 0)), ((w, 0), (w, h)), ((w, h), (0, h)), ((0, h), (0, 0))):
        L.append(f'\t(gr_line (start {x1} {y1}) (end {x2} {y2}) (stroke (width 0.1) (type default)) (layer "Edge.Cuts") (uuid "{U()}"))')
    L.append(')')
    with open(path, 'w') as f:
        f.write('\n'.join(L) + '\n')
    return nid


# an array of 4 x 4 balls at 0.8 mm, its corner ball at (8, 8): ball 22 at (8.8, 8.8), ball 23 at (9.6, 8.8)
P = 0.8
balls = [(f'{r + 1}{c + 1}', round(8 + c * P, 4), round(8 + r * P, 4)) for r in range(4) for c in range(4)]
net_of = {'22': 'TPA', '23': 'TPB', '32': 'OUT', '33': 'SAME'}
array = ('U1', 'F.Cu', [(n_, x, y, net_of.get(n_, f'S{n_}'), 'smd') for n_, x, y in balls])
others = [('TP1', 'B.Cu', [('1', 8.4, 9.2, 'TPA', 'smd')]),          # under the array, on the far face
          ('TP2', 'B.Cu', [('1', 9.2, 9.2, 'TPB', 'smd')]),          # ...beside it: the two want the site between
          ('R1', 'B.Cu', [('1', 15.0, 15.0, 'OUT', 'smd')]),         # outside the array's box
          ('R2', 'F.Cu', [('1', 9.2, 10.0, 'SAME', 'smd')])]         # under it, on the array's own face
far = [(f'X{n_}', 'B.Cu', [('1', 16.0 + 0.1 * i, 3.0, f'S{n_}', 'smd')]) for i, (n_, _x, _y) in enumerate(balls)
       if n_ not in net_of]

_active = rules.active
_base = rules.active()
_fine = dataclasses.replace(_base, clearance=0.09, track=0.0762, fan_track=0.0762, via_size=0.25, via_drill=0.15,
                            grid=0.05, hole_to_hole=0.25, edge_clearance=0.2)
rules.active = lambda: _fine
try:
    with tempfile.TemporaryDirectory() as td:
        b = os.path.join(td, 'b.kicad_pcb')
        write_board(b, [array] + others + far)
        pcb = field_ties._load(b)
        # ---- 1. the nets whose whole route lies beneath the array
        got = field_ties.beneath_nets(pcb, ['U1'])
        check('1. a ball and one far-face pad under the array: named; a pad outside its box, or on its own face: not',
              sorted(got), ['TPA', 'TPB'])
        check('1. ...each with its ball and its pad', sorted((p.component_ref, p.pad_number) for p in got['TPA']),
              [('TP1', '1'), ('U1', '22')])
        check('1. a net named to be skipped is not', sorted(field_ties.beneath_nets(pcb, ['U1'], skip={'TPA'})), ['TPB'])
        # ---- 2, 3. joined whole, on the lattice's sites, the two apart
        said = []
        J = field_ties.lay_beneath(b, ['U1'], log=said.append)
        check('2. both nets joined, none left', (J['nets'], J['left'], J['vias']), (['TPA', 'TPB'], [], 2))
        after = field_ties._load(b)
        ids = {n.name: i for i, n in after.nets.items()}
        ball = {p.pad_number: p for p in after.footprints['U1'].pads}
        for name, num, ref in (('TPA', '22', 'TP1'), ('TPB', '23', 'TP2')):
            vias = [v for v in after.vias if v.net_id == ids[name]]
            segs = [s for s in after.segments if s.net_id == ids[name]]
            pad = after.footprints[ref].pads[0]
            site = (round(abs(vias[0].x - ball[num].global_x), 4), round(abs(vias[0].y - ball[num].global_y), 4))
            check(f'2. {name}: one via, on a site half a pitch from its ball each way', (len(vias), site), (1, (0.4, 0.4)))
            check(f'2. {name}: a stub on the array\'s face and a track on the far face',
                  sorted({s.layer for s in segs}), ['B.Cu', 'F.Cu'])
            check(f'2. {name}: its ball and its pad carry copper',
                  (field_ties.joined(after, ball[num]), field_ties.joined(after, pad)), (True, True))
        va, vb = ([v for v in after.vias if v.net_id == ids[n_]][0] for n_ in ('TPA', 'TPB'))
        check('3. the two nets\' vias stand a via and the clearance apart',
              math.hypot(va.x - vb.x, va.y - vb.y) >= 0.25 + 0.09 - 1e-9, True)
        check('3. ...and the run says what it joined', any('joined beneath their array: 2 of 2' in s_ for s_ in said), True)
        check('2. a board with no such net: nothing to do',
              field_ties.lay_beneath(b, ['U1'], skip={'TPA', 'TPB'}), None)

    # ---- 4. a tie that leaves a net no way
    ring = [(round(10 + 0.9 * i, 2), round(10 + 0.9 * j, 2)) for i in range(-2, 3) for j in range(-2, 3)
            if max(abs(i), abs(j)) == 2 and (i, j) != (0, -2)]         # a ring of pads round (10, 10), one gap in it:
    #                                                                    net A's pad inside, 0.8 mm from the gap
    parts = [(f'W{k}', 'F.Cu', [('1', x, y, 'WALL', 'thru')]) for k, (x, y) in enumerate(ring)] \
        + [('A1', 'F.Cu', [('1', 10.0, 9.0, 'A', 'thru')]), ('A2', 'F.Cu', [('1', 10.0, 3.0, 'A', 'thru')]),
           ('T1', 'F.Cu', [('1', 4.0, 16.0, 'TIE', 'thru')]), ('T2', 'F.Cu', [('1', 16.0, 16.0, 'TIE', 'thru')])]
    gap = [(9.1, 8.2, 10.9, 8.2, L_, 'WALL') for L_ in ('F.Cu', 'B.Cu')]      # the gap closed on both layers
    off = [(4.0, 16.0, 16.0, 16.0, 'F.Cu', 'TIE')]                             # a tie nowhere near
    _wide = dataclasses.replace(_base, clearance=0.15, track=0.15, via_size=0.5, via_drill=0.3)
    rules.active = lambda: _wide
    with tempfile.TemporaryDirectory() as td:
        tied = os.path.join(td, 'shared_field.kicad_pcb')
        nid = write_board(tied + '.untied', parts)
        write_board(tied, parts, tracks=gap + off)
        ties = {'tracks': [{'start': (t[0], t[1]), 'end': (t[2], t[3]), 'width': 0.15, 'layer': t[4], 'net_id': nid[t[5]]}
                           for t in gap + off], 'vs': []}
        said = []
        gs, gv = group_ends.ties_boxed(td, tied, ties, dict(os.environ), said.append)
        check('4. the ties across the gap go, the tie elsewhere stays', (sorted(gs), sorted(gv)), ([0, 1], []))
        check('4. ...and the run names the net left no way', any('no way in the route step\'s plan for A' in s_
                                                                 for s_ in said), True)
        write_board(tied, parts, tracks=off)
        check('4. ties that leave every net its way: none taken out',
              group_ends.ties_boxed(td, tied, {'tracks': ties['tracks'][2:], 'vs': []}, dict(os.environ), said.append),
              (set(), set()))
finally:
    rules.active = _active

print(f'{N[0] - len(BAD)}/{N[0]} checks passed')
sys.exit(1 if BAD else 0)
