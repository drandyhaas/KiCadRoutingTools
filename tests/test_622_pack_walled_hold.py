"""The bus step's pack keeps no packed board that walls a pad in (awx/whole_route.py: pack_walls, pack_lay).

  python3 tests/test_622_pack_walled_hold.py

The pack (route_bus --pack) lays a round's lanes again, pulled tight, and its board stands in the lay's place only where
it grades no worse. Its grade read open nets and violations alone: measured, the pack pulled a lane across the last way
out of a supply ball of the destination array, the step's walled grade then named it, and the bus result was not
clean. `pack_walls(lay, packed, nets)` is the step's own test (route_bus.walled_grade) of the packed board against the
lay -- a pad reached on the lay and from nowhere on the packed board -- and the lanes the pack MOVED near such a pad
(route_bus.WALL_NEAR), which pack_lay holds as laid (pack_lanes --hold) and packs the rest again.

On synthetic boards, no routing: a 5x5 array (an array: 16 pads or more) whose middle ball is on a net with a pad
elsewhere, and round it a square of a bus net's track on both copper layers -- whole, or with a gap on one layer.

1. the lay leaves a gap, the packed board closes the ring: the middle ball is named, and the lane that closed it;
2. a lane the pack moved far off the ball is not named to hold, nor one by the ball the pack did not move;
3. controls: the packed board the lay itself walls nothing; a ball the lay already walled in is not the pack's.
"""
import contextlib
import io
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

PITCH = 1.0             # the array's pitch, mm
CX, CY = 20.0, 15.0     # the array's middle ball
RING = 0.5              # the ring's half side round the middle ball: between it and its neighbours
NETS = ['', 'SIG', 'BUS1', 'BUS2', 'BUS3']


def board(gap=None, far=(0.0, 0.0), near=True):
    """a two-layer board: U1, a 5x5 array of 0.5 mm pads (the middle one on SIG, the rest on no net), R1's pad on SIG
    10 mm off; BUS1's closed square of track round the middle ball on both layers -- on `gap`'s layer its east side left
    out; BUS2's track 6 mm below the array, moved by `far`; BUS3's track 1 mm above the ring when `near`"""
    pads = []
    for i in range(5):
        for j in range(5):
            k = 5 * i + j + 1
            netf = ' (net 1 "SIG")' if k == 13 else ''
            pads.append(f'    (pad "{k}" smd rect (at {(j - 2) * PITCH} {(i - 2) * PITCH}) (size 0.5 0.5) '
                        f'(layers "F.Cu" "F.Mask"){netf} (uuid "u1-p{k}"))')
    items = [
        f'  (footprint "synth:ARRAY5" (layer "F.Cu") (uuid "u1") (at {CX} {CY})\n'
        f'    (property "Reference" "U1" (at 0 -4 0) (layer "F.SilkS") (uuid "u1-ref")\n'
        f'      (effects (font (size 0.5 0.5) (thickness 0.1))))\n' + '\n'.join(pads) + '\n  )',
        f'  (footprint "synth:P" (layer "F.Cu") (uuid "r1") (at {CX + 10} {CY})\n'
        f'    (property "Reference" "R1" (at 0 -2 0) (layer "F.SilkS") (uuid "r1-ref")\n'
        f'      (effects (font (size 0.5 0.5) (thickness 0.1))))\n'
        f'    (pad "1" smd rect (at 0 0) (size 0.8 0.8) (layers "F.Cu" "F.Mask") (net 1 "SIG") (uuid "r1-p1"))\n  )']
    corners = [(CX - RING, CY - RING), (CX + RING, CY - RING), (CX + RING, CY + RING), (CX - RING, CY + RING)]
    n = 0
    for layer in ('F.Cu', 'B.Cu'):
        for side, ((x1, y1), (x2, y2)) in enumerate(zip(corners, corners[1:] + corners[:1])):
            if layer == gap and side == 1:
                continue                        # (the east side)
            n += 1
            items.append(f'  (segment (start {x1} {y1}) (end {x2} {y2}) (width 0.1) (layer "{layer}") (net 2) '
                         f'(uuid "ring-{n}"))')
    fx, fy = far
    items.append(f'  (segment (start {CX - 3 + fx} {CY + 6 + fy}) (end {CX + 3 + fx} {CY + 6 + fy}) (width 0.1) '
                 f'(layer "F.Cu") (net 3) (uuid "far-1"))')
    if near:
        items.append(f'  (segment (start {CX - 0.5} {CY - 1.5}) (end {CX + 0.5} {CY - 1.5}) (width 0.1) '
                     f'(layer "B.Cu") (net 4) (uuid "near-1"))')
    head = ('(kicad_pcb (version 20241229) (generator "synth") (generator_version "9.0")\n'
            '  (general (thickness 1.6) (legacy_teardrops no))\n  (paper "A4")\n'
            '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (37 "F.SilkS" user) (39 "F.Mask" user) '
            '(38 "B.Mask" user) (44 "Edge.Cuts" user))\n  (setup (pad_to_mask_clearance 0))')
    body = [f'  (net {i} "{nm}")' for i, nm in enumerate(NETS)] + items
    body.append('  (gr_rect (start 0 0) (end 40 30) (layer "Edge.Cuts") (stroke (width 0.1) (type solid)) '
                '(fill none) (uuid "edge"))')
    return head + '\n' + '\n'.join(body) + '\n)\n'


def walls(lay, packed, nets=('BUS1', 'BUS2', 'BUS3')):
    """pack_walls of `packed` against `lay` (texts), the bus `nets`: (walled, near)"""
    import whole_route
    with tempfile.TemporaryDirectory() as tmp:
        paths = []
        for name, text in (('lay', lay), ('packed', packed)):
            p = os.path.join(tmp, name + '.kicad_pcb')
            with open(p, 'w', encoding='utf-8') as f:
                f.write(text)
            paths.append(p)
        with contextlib.redirect_stdout(io.StringIO()):
            return whole_route.pack_walls(paths[0], paths[1], list(nets))


def test_pack_that_closes_the_ring_is_named_with_its_lane():
    walled, near = walls(board(gap='F.Cu'), board())
    assert walled == ['U1.13'], walled
    assert near == ['BUS1'], near


def test_far_moved_lane_and_unmoved_near_lane_not_held():
    # (BUS2 moved too, but 6 mm off the ball; BUS3 stands 1 mm off it, as laid)
    walled, near = walls(board(gap='F.Cu'), board(far=(0.5, 0.2)))
    assert walled == ['U1.13'], walled
    assert near == ['BUS1'], near


def test_controls():
    # (the packed board the lay itself; a ball the lay already walled in is not the pack's, moved lanes or not)
    assert walls(board(gap='F.Cu'), board(gap='F.Cu')) == ([], []), 'unchanged board'
    assert walls(board(), board(far=(0.5, 0.2))) == ([], []), 'walled on the lay already'


if __name__ == '__main__':
    test_controls()
    print('controls ok')
    test_far_moved_lane_and_unmoved_near_lane_not_held()
    print('far / unmoved ok')
    test_pack_that_closes_the_ring_is_named_with_its_lane()
    print('ok')
