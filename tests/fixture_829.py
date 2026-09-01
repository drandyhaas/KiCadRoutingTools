#!/usr/bin/env python3
"""The #829 fixture board, shared by the wx-free test and the pcbnew parity gate.

NOT named ``test_*.py``, so ``tests/run_all.py`` never collects it: it defines
no checks, it only builds a board.

It has to be synthetic. **No board in the corpus carries footprint-embedded
Edge.Cuts at all** -- 0 of the 27 tracked ``.kicad_pcb`` files -- and the one the
parser's own docstring cites (``rp2350_fpga_eensy``) was normalised to
board-level Edge.Cuts by ``tests/stress/prep_set11.sh`` before it entered the
repo. So the hazard #829 describes cannot be reproduced from a tracked board,
and a fixture that can fail has to be written.

Four footprints, covering the distinction the whole fix turns on:

``U_STRUCT``  pads + an Edge.Cuts line OUTSIDE the board rectangle.
             It draws the board's own boundary. Moving it resizes the board.
             -> owns_edge_cuts, owns_board_outline
``U_CARRY``  pads + a closed Edge.Cuts window INSIDE, under its own body.
             A relief the designer parented to the part so it travels with it,
             the shape crkbd uses 184 times. -> owns_edge_cuts only
``U_PLAIN``  pads, no Edge.Cuts. The ordinary case; must be untouched.
``U_NOPAD``  an Edge.Cuts line OUTSIDE the board, no pads -- the zero-pad
             "outline footprint" shape the issue's first draft cited. It is
             structural, and it is what proves the PAD gate rather than the
             outline gate is what already made it unreachable.
"""
import os
import tempfile

_HDR = ('(kicad_pcb (version 20240108) (generator pcbnew)\n'
        '  (general (thickness 1.6))\n'
        '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) '
        '(44 "Edge.Cuts" user))\n')

_STROKE = '(stroke (width 0.1) (type default))'

# A board-level rectangle. NOTE it does NOT exercise the ring arm of the
# containment ladder, and the comment here used to claim it did:
# `extract_board_contours` short-circuits a 4-segment axis-aligned rectangle to
# NO rings (see its "Simple rectangle, use bounding box" branch), so this board
# takes the BOUNDS arm -- as does splitflap_driver, for a different reason.
# `REVIEW_BOARDS` below carries the boards that reach the ring arm, and
# `test_both_arms_of_the_containment_ladder_are_exercised` asserts both are hit.
BOARD_RECT = (0.0, 0.0, 40.0, 30.0)


def _edge_rect(x1, y1, x2, y2, tag):
    pts = [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]
    return '\n'.join(
        f'  (gr_line (start {pts[i][0]} {pts[i][1]}) '
        f'(end {pts[(i + 1) % 4][0]} {pts[(i + 1) % 4][1]}) '
        f'{_STROKE} (layer "Edge.Cuts") (uuid "{tag}-{i}"))'
        for i in range(4))


def _pads(net_id=0):
    return ('\n    (pad "1" smd rect (at -0.5 0) (size 0.6 0.6) '
            '(layers "F.Cu") (uuid "p1"))'
            '\n    (pad "2" smd rect (at 0.5 0) (size 0.6 0.6) '
            '(layers "F.Cu") (uuid "p2"))')


def _window(half=0.8):
    """A closed Edge.Cuts square in LOCAL coords, centred on the origin."""
    c = [(-half, -half), (half, -half), (half, half), (-half, half)]
    return '\n'.join(
        f'    (fp_line (start {c[i][0]} {c[i][1]}) '
        f'(end {c[(i + 1) % 4][0]} {c[(i + 1) % 4][1]}) '
        f'{_STROKE} (layer "Edge.Cuts"))' for i in range(4))


def _fp(ref, x, y, body, rot=None):
    at = f'(at {x} {y})' if rot is None else f'(at {x} {y} {rot})'
    return (f'  (footprint "t:{ref}" (layer "F.Cu") {at}\n'
            f'    (uuid "uuid-{ref}")\n'
            f'    (property "Reference" "{ref}" (at 0 0 0) '
            f'(layer "F.SilkS") (uuid "r-{ref}"))\n'
            f'{body}\n  )')


def board_text(struct_rot=None, carry_rot=None):
    """The fixture as text. Rotations are exposed because a footprint's pose
    transforms its Edge.Cuts through `local_to_global`, so a PURE ROTATION
    moves the outline with the part sitting still."""
    x0, y0, x1, y1 = BOARD_RECT
    parts = [
        _edge_rect(x0, y0, x1, y1, 'edge'),
        # 6 mm past the right edge: unambiguously outside the board.
        _fp('U_STRUCT', 20.0, 10.0,
            _pads() + f'\n    (fp_line (start {x1 - 20.0 + 6.0} -2) '
                      f'(end {x1 - 20.0 + 6.0} 2) {_STROKE} '
                      f'(layer "Edge.Cuts"))',
            rot=struct_rot),
        _fp('U_CARRY', 10.0, 20.0, _pads() + '\n' + _window(), rot=carry_rot),
        _fp('U_PLAIN', 30.0, 20.0, _pads()),
        _fp('U_NOPAD', 6.0, -6.0,
            f'    (fp_line (start -1 -1) (end 1 -1) {_STROKE} '
            f'(layer "Edge.Cuts"))'),
    ]
    return _HDR + '\n'.join(parts) + '\n)\n'


EXPECTED = {
    # ref:        (owns_edge_cuts, owns_board_outline)
    'U_STRUCT':   (True,  True),
    'U_CARRY':    (True,  False),
    'U_PLAIN':    (False, False),
    'U_NOPAD':    (True,  True),
}


def write(dirpath=None, **kw):
    """Materialise the fixture and return its path. A real file, because a
    check whose input is not a real file tests nothing (run_utils.evidence)."""
    d = dirpath or tempfile.mkdtemp(prefix='fix829_')
    p = os.path.join(d, 'edge_cuts_owner.kicad_pcb')
    with open(p, 'w', encoding='utf-8') as f:
        f.write(board_text(**kw))
    return p



# --- boards a blind review found the first classifier got wrong -------------
# Kept here rather than inline in the test so the parity gate can use them too.

def _gr(x1, y1, x2, y2, tag):
    return (f'  (gr_line (start {x1} {y1}) (end {x2} {y2}) {_STROKE} '
            f'(layer "Edge.Cuts") (uuid "{tag}"))')


def _fp_pads(ref, x, y, body):
    return (f'  (footprint "t:{ref}" (layer "F.Cu") (at {x} {y})\n'
            f'    (uuid "u-{ref}")\n'
            f'    (property "Reference" "{ref}" (at 0 0 0) '
            f'(layer "F.SilkS") (uuid "r-{ref}"))\n'
            f'{_pads()}\n{body}\n  )')


def panel_board():
    """A PANELISED board: the real outline nests inside the panel frame, so
    `_classify_contours` calls it a CUTOUT and the only OUTER ring is the
    frame. `J1` draws the real board's fourth edge -- an open path -- and
    `LED1` carries a window. Containment alone called J1 carried, because it
    does sit inside the frame."""
    parts = [_gr(0, 0, 100, 0, 'p0'), _gr(100, 0, 100, 50, 'p1'),
             _gr(100, 50, 0, 50, 'p2'), _gr(0, 50, 0, 0, 'p3'),
             _gr(20, 10, 60, 10, 'r0'), _gr(60, 10, 60, 40, 'r1'),
             _gr(60, 40, 20, 40, 'r2'),
             _fp_pads('J1', 20, 25,
                      f'    (fp_line (start 0 -15) (end 0 15) {_STROKE} '
                      f'(layer "Edge.Cuts"))'),
             _fp_pads('LED1', 40, 25, _window(1.0))]
    return _HDR + '\n'.join(parts) + '\n)\n', {'J1': True, 'LED1': False}


def rect_board(segments=4):
    """The same physical geometry spelled with 4 or 8 board-level segments.
    `extract_board_contours` short-circuits a 4-segment axis-aligned rectangle
    to NO rings, so the two spellings used to classify DIFFERENTLY."""
    if segments == 4:
        edge = [_gr(0, 0, 40, 0, 'a'), _gr(40, 0, 40, 30, 'b'),
                _gr(40, 30, 0, 30, 'c'), _gr(0, 30, 0, 0, 'd')]
    else:
        edge = [_gr(0, 0, 20, 0, 'a'), _gr(20, 0, 40, 0, 'a2'),
                _gr(40, 0, 40, 15, 'b'), _gr(40, 15, 40, 30, 'b2'),
                _gr(40, 30, 20, 30, 'c'), _gr(20, 30, 0, 30, 'c2'),
                _gr(0, 30, 0, 15, 'd'), _gr(0, 15, 0, 0, 'd2')]
    jf = _fp_pads('JF', 40, 15,
                  f'    (fp_line (start 0 -5) (end 0 5) {_STROKE} '
                  f'(layer "Edge.Cuts"))')
    return _HDR + '\n'.join(edge) + '\n' + jf + '\n)\n', {'JF': True}


def round_board():
    """A round relief entirely ON a round board. The bounds scan represents a
    circle by its bounding-box CORNERS, and a corner of this window falls
    outside the board while every point of the circle is inside -- so it was
    classified structural and frozen: the #628 over-lock, produced by the fix
    meant to avoid it."""
    board = (f'  (gr_circle (center 20 20) (end 40 20) {_STROKE} '
             f'(layer "Edge.Cuts") (uuid "c0"))')
    ur = _fp_pads('U_R', 32.45, 32.45,
                  f'    (fp_circle (center 0 0) (end 2 0) {_STROKE} '
                  f'(layer "Edge.Cuts"))')
    return _HDR + board + '\n' + ur + '\n)\n', {'U_R': False}


REVIEW_BOARDS = {
    'panelised (open path inside the panel frame)': panel_board,
    'rectangle spelled with 4 segments': lambda: rect_board(4),
    'rectangle spelled with 8 segments': lambda: rect_board(8),
    'round window on a round board': round_board,
}


if __name__ == '__main__':
    print(write())
