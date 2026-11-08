"""#1430: the bus step names the balls of its arrays it walls in.

A plain bus between two array parts can leave an array's OTHER ball (a net that
is not a bus net) reachable on the board as it came and reachable from nowhere
once the locked lanes are laid: its net then ships open, and no check of the
step saw it (exit 0, connected=1 drc=0). `route_bus.walled_grade` names such
balls -- in its log line, and in the result the step puts on its BUS line
(`walled=`) and in its summary (`walled`).

On synthetic boards, no routing: a 3x3 array whose middle ball is on a net with
a pad elsewhere, and the same board with a closed ring of a bus net's track
round that ball on every copper layer -- the copper a bus's lanes leave there.

1. the ring walls the ball in: it is named, with its net, in the log line;
2. the same ring with a gap on either layer walls nothing -- the ball leaves
   through the gap (by a via in its own pad where the gap is on the other layer)
   -- nor does a ring on one layer only;
3. the control: a board graded against itself walls nothing.
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


def board(rings=(), gap=None):
    """a two-layer board: U1, a 3x3 array of 0.5 mm pads (the middle one on SIG, the rest on no net), R1's pad on SIG
    10 mm off, and on each layer of `rings` a closed square of BUS1's track round the middle ball -- on `gap`'s layer
    its east side left out"""
    nets = ['', 'SIG', 'BUS1']
    pads = []
    for i in range(3):
        for j in range(3):
            k = 3 * i + j + 1
            net = 1 if k == 5 else 0
            netf = f' (net {net} "{nets[net]}")' if net else ''
            pads.append(f'    (pad "{k}" smd rect (at {(j - 1) * PITCH} {(i - 1) * PITCH}) (size 0.5 0.5) '
                        f'(layers "F.Cu" "F.Mask"){netf} (uuid "u1-p{k}"))')
    items = [
        f'  (footprint "synth:ARRAY3" (layer "F.Cu") (uuid "u1") (at {CX} {CY})\n'
        f'    (property "Reference" "U1" (at 0 -3 0) (layer "F.SilkS") (uuid "u1-ref")\n'
        f'      (effects (font (size 0.5 0.5) (thickness 0.1))))\n' + '\n'.join(pads) + '\n  )',
        f'  (footprint "synth:P" (layer "F.Cu") (uuid "r1") (at {CX + 10} {CY})\n'
        f'    (property "Reference" "R1" (at 0 -2 0) (layer "F.SilkS") (uuid "r1-ref")\n'
        f'      (effects (font (size 0.5 0.5) (thickness 0.1))))\n'
        f'    (pad "1" smd rect (at 0 0) (size 0.8 0.8) (layers "F.Cu" "F.Mask") (net 1 "SIG") (uuid "r1-p1"))\n  )']
    corners = [(CX - RING, CY - RING), (CX + RING, CY - RING), (CX + RING, CY + RING), (CX - RING, CY + RING)]
    n = 0
    for layer in rings:
        for side, ((x1, y1), (x2, y2)) in enumerate(zip(corners, corners[1:] + corners[:1])):
            if layer == gap and side == 1:
                continue                        # (the east side)
            n += 1
            items.append(f'  (segment (start {x1} {y1}) (end {x2} {y2}) (width 0.1) (layer "{layer}") (net 2) '
                         f'(uuid "ring-{n}"))')
    head = ('(kicad_pcb (version 20241229) (generator "synth") (generator_version "9.0")\n'
            '  (general (thickness 1.6) (legacy_teardrops no))\n  (paper "A4")\n'
            '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (37 "F.SilkS" user) (39 "F.Mask" user) '
            '(38 "B.Mask" user) (44 "Edge.Cuts" user))\n  (setup (pad_to_mask_clearance 0))')
    body = [f'  (net {i} "{nm}")' for i, nm in enumerate(nets)] + items
    body.append('  (gr_rect (start 0 0) (end 40 30) (layer "Edge.Cuts") (stroke (width 0.1) (type solid)) '
                '(fill none) (uuid "edge"))')
    return head + '\n' + '\n'.join(body) + '\n)\n'


def grade(before, after):
    """walled_grade of `after` against `before` (texts), U1 the array, BUS1 the bus: (result, its log lines)"""
    import route_bus
    with tempfile.TemporaryDirectory() as tmp:
        paths = []
        for name, text in (('before', before), ('after', after)):
            p = os.path.join(tmp, name + '.kicad_pcb')
            with open(p, 'w', encoding='utf-8') as f:
                f.write(text)
            paths.append(p)
        lines = []
        with contextlib.redirect_stdout(io.StringIO()):
            got = route_bus.walled_grade(paths[0], paths[1], ['U1'], ['BUS1'], log=lines.append)
    return got, lines


def test_ring_walls_the_ball_in():
    got, lines = grade(board(), board(rings=('F.Cu', 'B.Cu')))
    assert got['balls'] == 1, got                       # (the middle ball alone: the rest are on no net)
    assert got['walled'] == ['U1.5'], got
    assert any('WALLED IN by the step' in ln and 'U1.5 (SIG)' in ln for ln in lines), lines


def test_ring_with_a_gap_walls_nothing():
    # (a gap on either layer; and the ring on one layer only, the ball down a via in its own pad to the other)
    for after in (board(rings=('F.Cu', 'B.Cu'), gap='F.Cu'), board(rings=('F.Cu', 'B.Cu'), gap='B.Cu'),
                  board(rings=('F.Cu',))):
        got, lines = grade(board(), after)
        assert got['balls'] == 1 and got['walled'] == [], got
        assert not any('WALLED IN' in ln for ln in lines), lines


def test_unchanged_board_walls_nothing():
    """a board graded against itself walls nothing: a ball is named only when the step made it unreachable"""
    walled = board(rings=('F.Cu', 'B.Cu'))
    got, lines = grade(walled, walled)
    assert got['balls'] == 1 and got['walled'] == [], got


if __name__ == '__main__':
    test_unchanged_board_walls_nothing()
    print('control ok')
    test_ring_with_a_gap_walls_nothing()
    print('gap ok')
    test_ring_walls_the_ball_in()
    print('ok')
