#!/usr/bin/env python3
"""A walled ball's way out is kept BETWEEN the lanes (awx/route_bus.py: ball_channel; awx/route_lanes.py: lift_walls).

  python3 tests/test_622_ball_channel.py

A bus step leaves the balls of its arrays on no bus net a way out; where the lanes laid wall one in, the lay lifts the
walling lanes and routes them again round the ball's via site. A ball whose wall stands PAST that site -- its stub out
of the array between two lanes that run on side by side, a track's room short of a track -- kept nothing by it: the
lanes routed again took the copper they had. ball_channel gives the ball's whole way, as the cage test asks it: from
the end of its own copper to the rest of its net in its view, else to the view's edge, over the cells a track fits
with the lanes' copper gone and never across a piece of it -- and the lanes a track along it is short of room to,
which the lay lifts and routes again round it.

On synthetic two-layer boards, no routing: an array's edge ball with a stub out of the array, fenced in the array by
standing copper, two lanes leaving either side of the stub's end; the back layer full of tracks (no via fits).

1. the lanes close past the stub's end, short of a track's room: the ball is caged, and free with the lanes gone;
   its way runs from the stub's end on between the two lanes to the edge of its view, on the ball's layer, with no
   change of layer, a few straight pieces; both lanes are named, the nearer the ball first; the stretch with no room
   as laid is the stretch they run closed;
2. one lane round the stub's end (down one side, across, up the other): every way out crosses it -- none is given;
3. the lanes far enough apart all the way: a way that asks no lane to move (no lane named, nothing short);
4. the rest of the ball's net in the channel, past where the lanes close: the way ends on it, not at the edge;
5. a lane that STANDS (its copper not the remedy's to lay again -- one with no room to part): the way keeps a track's
   whole room off it and names the other lane alone; both standing, there is none.
"""
import contextlib
import io
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__))) if os.path.basename(
    os.path.dirname(os.path.abspath(__file__))) == 'tests' else os.getcwd()
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
os.environ.pop('AWX_RULES', None)
try:
    import scipy  # noqa: F401
except ImportError:
    print('SKIP: scipy is not installed (the cage test labels its raster with it)')
    sys.exit(77)

import route_bus as rb  # noqa: E402
from kicad_parser import parse_kicad_pcb, is_immutable_copper  # noqa: E402

BAD, N = [], [0]


def check(what, ok, got=None):
    N[0] += 1
    if not ok:
        BAD.append(what)
    print(('ok   ' if ok else 'FAIL ') + what + ('' if ok or got is None else f' -- got {got}'))


PITCH = 1.0                 # the array's pitch, mm
CX, CY = 20.0, 16.0         # the ball: the middle one of the array's bottom row
RUNG = (0.1, 0.25, 0.1)     # the track, via and clearance a cage is tested at
TOP, END, MOUTH, FAR = CY - 0.5, CY + 1.0, CY + 0.5, CY + 5.0       # the fence's top, the stub's end, the lanes' start
LEFT, RIGHT = CX - 0.4, CX + 0.4                                    # the two lanes' lines at the stub's end


def board(right_to, hairpin=False, more=()):
    """a two-layer board: U1, a 3x3 array of 0.5 mm pads, its bottom-middle ball on SIG with a stub straight out to
    END and R1's pad on SIG 15 mm off; FENCE's copper round the ball inside the array, down to MOUTH either side;
    lane LA straight down the left of the stub from MOUTH, lane LB down the right and over to x = `right_to` past the
    stub's end (`hairpin`: LA alone instead, down the left, across under the stub's end and up the right); the back
    layer full of BACK's tracks, a quarter millimetre apart. `more`: [(x1, y1, x2, y2, net)] on the front"""
    nets = ['', 'SIG', 'FENCE', 'LA', 'LB', 'BACK']
    pads = []
    for i in range(3):
        for j in range(3):
            k = 3 * i + j + 1
            netf = ' (net 1 "SIG")' if k == 8 else ''
            pads.append(f'    (pad "{k}" smd rect (at {(j - 1) * PITCH} {(i - 2) * PITCH}) (size 0.5 0.5) '
                        f'(layers "F.Cu" "F.Mask"){netf} (uuid "u1-p{k}"))')
    items = [
        f'  (footprint "synth:ARRAY3" (layer "F.Cu") (uuid "u1") (at {CX} {CY})\n'
        f'    (property "Reference" "U1" (at 0 -4 0) (layer "F.SilkS") (uuid "u1-ref")\n'
        f'      (effects (font (size 0.5 0.5) (thickness 0.1))))\n' + '\n'.join(pads) + '\n  )',
        f'  (footprint "synth:P" (layer "F.Cu") (uuid "r1") (at {CX + 15} {CY})\n'
        f'    (property "Reference" "R1" (at 0 -2 0) (layer "F.SilkS") (uuid "r1-ref")\n'
        f'      (effects (font (size 0.5 0.5) (thickness 0.1))))\n'
        f'    (pad "1" smd rect (at 0 0) (size 0.8 0.8) (layers "F.Cu" "F.Mask") (net 1 "SIG") (uuid "r1-p1"))\n  )']
    front = [(CX, CY, CX, END, 1),                                              # the ball's stub
             (LEFT, MOUTH, LEFT, TOP, 2), (LEFT, TOP, RIGHT, TOP, 2), (RIGHT, TOP, RIGHT, MOUTH, 2)]
    if hairpin:
        front += [(LEFT, MOUTH, LEFT, END + 0.5, 3), (LEFT, END + 0.5, RIGHT, END + 0.5, 3),
                  (RIGHT, END + 0.5, RIGHT, MOUTH, 3)]
    else:
        d = RIGHT - right_to
        front += [(LEFT, MOUTH, LEFT, FAR, 3),
                  (RIGHT, MOUTH, RIGHT, END + 0.5, 4), (RIGHT, END + 0.5, right_to, END + 0.5 + d, 4),
                  (right_to, END + 0.5 + d, right_to, FAR, 4)]
    front += list(more)
    n = 0
    for x1, y1, x2, y2, net in front:
        n += 1
        items.append(f'  (segment (start {x1:.4f} {y1:.4f}) (end {x2:.4f} {y2:.4f}) (width 0.1) (layer "F.Cu") '
                     f'(net {net}) (uuid "f-{n}"))')
    for k in range(int(12 / 0.25) + 1):
        y = CY - 6 + 0.25 * k
        items.append(f'  (segment (start {CX - 6} {y:.4f}) (end {CX + 6} {y:.4f}) (width 0.1) (layer "B.Cu") (net 5) '
                     f'(uuid "b-{k}"))')
    head = ('(kicad_pcb (version 20241229) (generator "synth") (generator_version "9.0")\n'
            '  (general (thickness 1.6) (legacy_teardrops no))\n  (paper "A4")\n'
            '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (37 "F.SilkS" user) (39 "F.Mask" user) '
            '(38 "B.Mask" user) (44 "Edge.Cuts" user))\n  (setup (pad_to_mask_clearance 0))')
    body = [f'  (net {i} "{nm}")' for i, nm in enumerate(nets)] + items
    body.append('  (gr_rect (start 0 0) (end 50 40) (layer "Edge.Cuts") (stroke (width 0.1) (type solid)) '
                '(fill none) (uuid "edge"))')
    return head + '\n' + '\n'.join(body) + '\n)\n'


def load(text):
    with tempfile.TemporaryDirectory() as tmp:
        p = os.path.join(tmp, 'b.kicad_pcb')
        with open(p, 'w', encoding='utf-8') as f:
            f.write(text)
        with contextlib.redirect_stdout(io.StringIO()):
            return parse_kicad_pcb(p)


def asked(pcb):
    """(the ball, the walls' nets, the lanes' copper: {id: its lane})"""
    ball = next(p for p in pcb.footprints['U1'].pads if p.pad_number == '8')
    fixed = {i for i, nn in pcb.nets.items() if nn.name in ('FENCE', 'LA', 'LB', 'BACK')}
    soft = {id(s): pcb.nets[s.net_id].name for s in pcb.segments if pcb.nets[s.net_id].name in ('LA', 'LB')}
    return ball, fixed, soft


def caged(pcb, ball, fixed, without=()):
    crop = rb._cropped(pcb, pcb.footprints['U1'])
    crop.segments = [s for s in crop.segments if id(s) not in without]
    return rb._caged(rb._walls(crop, fixed, ball.net_id, is_immutable_copper), ball, *RUNG)


# 1. the lanes close past the stub's end: the way between them
CLOSED = LEFT + 0.36            # (0.36 between the two lanes' lines: 0.26 of room, a track and its clearances need 0.3)
pcb = load(board(CLOSED))
ball, fixed, soft = asked(pcb)
check('1. the lanes closed past the stub\'s end cage the ball', caged(pcb, ball, fixed))
check('1. ...and with the lanes\' copper gone it is free', not caged(pcb, ball, fixed, without=soft))
ch = rb.ball_channel(pcb, ball, fixed, RUNG, soft)
check('1. a way out between the lanes is given', ch is not None)
if ch is not None:
    pts = [p for a, b, _L in ch['lines'] for p in (a, b)]
    check('1. ...on the ball\'s layer, with no change of layer',
          {L for _a, _b, L in ch['lines']} == {'F.Cu'} and ch['vias'] == [], (ch['lines'], ch['vias']))
    check('1. ...from the end of the ball\'s stub on', abs(pts[0][0] - CX) < 0.06 and abs(pts[0][1] - END) < 0.1, pts[0])
    check('1. ...to the edge of its view', ch['end'] == 'edge' and abs(pts[-1][1] - (CY + rb.CAGE_VIEW)) < 0.06, pts[-1])
    check('1. ...between the two lanes all the way',
          all(LEFT < x < RIGHT and (y < END + 0.5 + (RIGHT - CLOSED) or x < CLOSED) for x, y in pts), pts)
    check('1. ...a few straight pieces, not a cell a piece', len(ch['lines']) <= 6, len(ch['lines']))
    check('1. both lanes named', sorted(ch['lanes']) == ['LA', 'LB'], ch['lanes'])
    run = (CY + rb.CAGE_VIEW) - (END + 0.5 + (RIGHT - CLOSED))
    check('1. the stretch with no room as laid is the stretch the lanes run closed',
          run - 0.3 < ch['short'] < run + 0.5, (ch['short'], run))

# 2. one lane round the stub's end: no way but across it
pcb = load(board(None, hairpin=True))
ball, fixed, soft = asked(pcb)
check('2. a lane round the stub\'s end cages the ball; free with it gone',
      caged(pcb, ball, fixed) and not caged(pcb, ball, fixed, without=soft))
check('2. ...and no way is given across it', rb.ball_channel(pcb, ball, fixed, RUNG, soft) is None)

# 3. the lanes apart all the way: a way that asks none to move
pcb = load(board(LEFT + 0.6))
ball, fixed, soft = asked(pcb)
ch = rb.ball_channel(pcb, ball, fixed, RUNG, soft)
check('3. lanes a track\'s room apart cage nothing', not caged(pcb, ball, fixed))
check('3. ...the way between them asks no lane to move', ch is not None and ch['lanes'] == [] and ch['short'] == 0,
      ch and (ch['lanes'], ch['short']))

# 4. the rest of the net in the channel past where the lanes close: the way ends on it
MID = LEFT + 0.18
pcb = load(board(CLOSED, more=[(MID, END + 2.0, MID, END + 2.05, 1)]))
ball, fixed, soft = asked(pcb)
ch = rb.ball_channel(pcb, ball, fixed, RUNG, soft)
check('4. the rest of its net past the closed stretch: the way ends on it',
      ch is not None and ch['end'] == 'net' and abs(ch['lines'][-1][1][1] - (END + 2.0)) < 0.2,
      ch and (ch['end'], ch['lines'][-1]))

# 5. a lane that stands: the way clear of it, the other lane named alone
pcb = load(board(CLOSED))
ball, fixed, soft = asked(pcb)
one = {i: m for i, m in soft.items() if m == 'LA'}
ch = rb.ball_channel(pcb, ball, fixed, RUNG, one)
check('5. one lane standing: a way is given, the other lane named alone', ch is not None and ch['lanes'] == ['LA'],
      ch and ch['lanes'])
if ch is not None:
    room = RUNG[0] / 2 + RUNG[2] + 0.05         # (a track's half, its clearance, the standing lane's half)
    low = [x for a, b, _L in ch['lines'] for x, y in (a, b) if y > END + 0.5 + (RIGHT - CLOSED) + 0.3]
    check('5. ...its whole room off the standing lane where they run side by side',
          bool(low) and all(LEFT < x <= CLOSED - room + 0.03 for x in low), low)
check('5. both standing: no way', rb.ball_channel(pcb, ball, fixed, RUNG, {}) is None)

print(f'{N[0] - len(BAD)}/{N[0]} checks passed')
sys.exit(1 if BAD else 0)
