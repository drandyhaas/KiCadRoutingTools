#!/usr/bin/env python3
"""#1700 parity gate: both parse paths read an Edge.Cuts arc into `board_bounds`
alike.

The text parser reads an arc off its chords (`kicad_parser.BOUNDS_CHORD_EPS`);
the live builder used to take KiCad's exact bounding box, so where the chords
miss an apex the two fronts laid copper out from bounds up to 5 um apart. On
#1603's fixture with its east edge redrawn as arcs of several sags, and an
outline a footprint draws at several turns and on the back:

  1. the two read the same `board_bounds`, to 10 nm;
  2. and the same ring, to 10 nm, so neither scan is reading another outline;
  3. the fixture has teeth: on some sags KiCad's own exact box stands over 1 um
     past the chords, so a path taking it fails row 1;
  4. a board the text path trusts, the live path's data trusts too;
  5. the live reading has teeth of its own: with the bounds' chord sag changed
     under the live helper alone, row 1 fails on some board.

IPC front: the live builder is the kipy one (`build_pcb_data_from_board` over
a kipy Board), and it reads the outline through `_compute_edge_cuts_bbox_kipy`
(bounds) and `kipy_board_contours` (rings) -- the two calls this gate makes,
with the same arguments the builder passes: the board's Edge.Cuts shapes and
(key, footprint) pairs. Those are REAL kipy objects (`kipy.board_types`
BoardSegment / BoardArc / FootprintInstance), each round-tripped through its
protobuf as the API hands it over. Their geometry is what KiCad holds in memory
and the IPC server serialises -- absolute coordinates, a footprint's children
included, an arc as start / mid / end in nanometres -- and is read here from
pcbnew's own load of the same file, which stands in for the server: the plugin
under test never touches pcbnew. The live PCBData of rows 2 and 4 is the text
parse with its outline fields replaced by the kipy readings, which are the
only outline fields the kipy builder fills.

Needs KiCad's python (pcbnew for the stand-in server, kipy for the shapes);
re-execs into it automatically. Exits 2 when no such python is found, rather
than 0, because a gate that exits 0 without running reads as a pass.

    python3 -X utf8 tests/gui_parity/test_1700_arc_bounds_parity.py
"""
import contextlib
import copy
import io
import os
import re
import shutil
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
for _p in ('py_router', 'py_placer', 'py_tools'):
    sys.path.insert(0, os.path.join(REPO, _p))
sys.path.insert(0, REPO)

FIXTURE = os.path.join(REPO, 'tests', 'fixtures', '1603', 'edge_sma.kicad_pcb')
TOL = 1e-5      # mm: pcbnew's GetArcMid rounds to whole nanometres
#: the apex of each arc stands this far past its 20 mm chord
SAGS = (0.7, 1.0, 1.3, 1.6, 2.0, 2.3, 2.6, 3.1, 3.7, 4.3)
failures = []


def reexec_into_kicad():
    from kicad_locate import kicad_python_candidates
    for py in kicad_python_candidates():
        if py == sys.executable or not os.path.exists(py):
            continue
        if subprocess.run([py, '-c', 'import pcbnew, kipy'],
                          capture_output=True).returncode == 0:
            sys.exit(subprocess.run([py, '-X', 'utf8', os.path.abspath(__file__)]).returncode)
    print('NOT RUN: no python with pcbnew + kipy found')
    sys.exit(2)


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def line(a, b, tok='gr_line'):
    return (f'({tok} (start {a[0]} {a[1]}) (end {b[0]} {b[1]}) '
            f'(stroke (width 0.1) (type default)) (layer "Edge.Cuts"))\n')


def arc(a, m, b, tok='gr_arc'):
    return (f'({tok} (start {a[0]} {a[1]}) (mid {m[0]} {m[1]}) (end {b[0]} {b[1]}) '
            f'(stroke (width 0.1) (type default)) (layer "Edge.Cuts"))\n')


def drawn(tmp, name, items):
    """The fixture with `items` for its outline."""
    with open(FIXTURE, encoding='utf-8', newline='') as f:
        t = f.read()
    t, n = re.subn(r'\t\(gr_line\n(?:\t\t[^\n]*\n|\t\t\t[^\n]*\n)*?\t\t\(layer "Edge.Cuts"\)\n'
                   r'[^\n]*\n\t\)\n', '', t)
    if n != 4:
        raise AssertionError(f'BROKEN TEST: the fixture draws {n} outline lines, not 4')
    i = t.rindex('\t(embedded_fonts')
    path = os.path.join(tmp, name + '.kicad_pcb')
    with open(path, 'w', encoding='utf-8', newline='') as f:
        f.write(t[:i] + ''.join(items) + t[i:])
    return path


def in_footprint(rot, back=False):
    """A pad-less footprint drawing the whole outline: arcs on its short sides,
    the side a straight chord cannot read, at the pose `rot`, optionally on the
    back. The file's coordinates are the footprint's own, as KiCad writes them."""
    layer = 'B.Cu' if back else 'F.Cu'
    return ('\t(footprint "outline:drawn" (layer "%s")\n'
            '\t\t(uuid "00000000-0000-4000-8000-000000001700")\n'
            '\t\t(at 18 15 %s)\n'
            '\t\t(property "Reference" "BRD" (at 0 0 0) (layer "F.SilkS") '
            '(effects (font (size 1 1) (thickness 0.15))))\n' % (layer, rot)
            + line((-11, -10), (11, -10), 'fp_line') + line((-11, 10), (11, 10), 'fp_line')
            + arc((11, -10), (13, 0), (11, 10), 'fp_arc')
            + arc((-11, 10), (-13, 0), (-11, -10), 'fp_arc')
            + '\t)\n')


def apart(a, b):
    return max(abs(x - y) for x, y in zip(a, b))


def kicad_box(board, pcbnew):
    """KiCad's own exact reading of the outline, its stroke taken off: what the
    live path used to take."""
    box = None
    drawings = list(board.GetDrawings()) + [g for fp in board.GetFootprints()
                                            for g in fp.GraphicalItems()]
    for d in drawings:
        if d.GetLayer() != pcbnew.Edge_Cuts:
            continue
        bb = d.GetBoundingBox()
        hs = d.GetWidth() / 2.0
        here = [(bb.GetX() + hs) / 1e6, (bb.GetY() + hs) / 1e6,
                (bb.GetX() + bb.GetWidth() - hs) / 1e6, (bb.GetY() + bb.GetHeight() - hs) / 1e6]
        box = here if box is None else [min(box[0], here[0]), min(box[1], here[1]),
                                        max(box[2], here[2]), max(box[3], here[3])]
    return box


def kipy_shape(d, pcbnew):
    """A real kipy BoardSegment / BoardArc for one pcbnew Edge.Cuts shape, as
    the IPC server serialises it: absolute nanometres, an arc by its mid."""
    from kipy.board_types import BoardArc, BoardLayer, BoardSegment
    from kipy.geometry import Vector2

    def v(p):
        return Vector2.from_xy(int(p.x), int(p.y))
    kind = d.GetShape()
    if kind == pcbnew.SHAPE_T_SEGMENT:
        s = BoardSegment()
        s.start, s.end = v(d.GetStart()), v(d.GetEnd())
        cls = BoardSegment
    elif kind == pcbnew.SHAPE_T_ARC:
        s = BoardArc()
        s.start, s.mid, s.end = v(d.GetStart()), v(d.GetArcMid()), v(d.GetEnd())
        cls = BoardArc
    else:
        raise AssertionError(f'BROKEN TEST: an Edge.Cuts shape of kind {kind} '
                             'this gate does not serialise')
    s.layer = BoardLayer.BL_Edge_Cuts
    return cls(proto=s.proto)      # as it comes off the wire


def kipy_outline(board, pcbnew):
    """(drawings, (key, footprint) pairs): the builder's two outline inputs."""
    from kipy.board_types import BoardLayer, FootprintInstance
    from kipy.geometry import Angle, Vector2
    drawings = [kipy_shape(d, pcbnew) for d in board.GetDrawings()
                if d.GetLayer() == pcbnew.Edge_Cuts]
    pairs = []
    for fp in board.GetFootprints():
        shapes = [kipy_shape(g, pcbnew) for g in fp.GraphicalItems()
                  if g.GetLayer() == pcbnew.Edge_Cuts]
        if not shapes:
            continue
        fi = FootprintInstance()
        # Pose first: kipy's setters carry the children along, and these
        # children are already at their absolute place.
        p = fp.GetPosition()
        fi.position = Vector2.from_xy(int(p.x), int(p.y))
        fi.orientation = Angle.from_degrees(fp.GetOrientationDegrees())
        fi.layer = BoardLayer.BL_B_Cu if fp.IsFlipped() else BoardLayer.BL_F_Cu
        for s in shapes:
            fi.definition.add_item(s)
        pairs.append((fp.GetReference(), FootprintInstance(proto=fi.proto)))
    return drawings, pairs


def live_read(drawings, pairs):
    """The kipy builder's outline reading, by its own calls."""
    import kicad_parser
    from kipy.board_types import BoardLayer
    edge = BoardLayer.BL_Edge_Cuts
    bounds = kicad_parser._compute_edge_cuts_bbox_kipy(drawings, edge, pairs)
    outers, cutouts = kicad_parser.kipy_board_contours(drawings, edge, pairs)
    return bounds, outers, cutouts


def main():
    try:
        import pcbnew
        import kipy  # noqa: F401
    except ImportError:
        reexec_into_kicad()
    import kicad_parser
    from kicad_parser import parse_kicad_pcb
    from placement.floorplan import outline_state
    if not os.path.isfile(FIXTURE):
        raise AssertionError(f'BROKEN TEST: no fixture at {FIXTURE}')
    tmp = tempfile.mkdtemp(prefix='t1700_')
    try:
        boards = {}
        for sag in SAGS:
            boards[f'gr_arc, sag {sag}'] = drawn(tmp, f'sag{sag}'.replace('.', '_'), [
                line((5, 5), (5, 25)), line((5, 25), (31 - sag, 25)), line((31 - sag, 5), (5, 5)),
                arc((31 - sag, 5), (31, 15), (31 - sag, 25))])
        for rot, back in ((0, False), (90, False), (180, False), (34, False), (90, True)):
            boards[f'fp_arc, turned {rot}{" on the back" if back else ""}'] = drawn(
                tmp, f'fp{rot}{back}', [in_footprint(rot, back)])
        missed = 0
        bites = 0
        for name, path in boards.items():
            with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                text = parse_kicad_pcb(path)
                board = pcbnew.LoadBoard(path)
            drawings, pairs = kipy_outline(board, pcbnew)
            if not drawings and not pairs:
                check(f'{name}: the stand-in server serialised an outline', False)
                continue
            lb, outers, cutouts = live_read(drawings, pairs)
            tb = text.board_info.board_bounds
            if tb is None or lb is None:
                check(f'{name}: both read bounds', False, f'{tb} {lb}')
                continue
            d = apart(tb, lb)
            check(f'{name}: the same bounds', d <= TOL, f'{d * 1e3:.4f} um: {tb} {lb}')
            live = copy.copy(text)
            live.board_info = copy.copy(text.board_info)
            live.board_info.board_bounds = lb
            live.board_info.board_outlines = outers
            live.board_info.board_outline = outers[0] if outers else []
            live.board_info.board_cutouts = cutouts
            ta, la = text.board_info.board_outlines, live.board_info.board_outlines
            ring = len(ta) == len(la) and all(
                len(x) == len(y) and all(min(abs(p[0] - q[0]) + abs(p[1] - q[1]) for q in y) <= 2 * TOL
                                         for p in x)
                for x, y in zip(ta, la))
            check('...and the same ring', ring and bool(ta), f'{len(ta)} {len(la)}')
            exact = kicad_box(board, pcbnew)
            missed += apart(exact, tb) > 1e-3
            ts, ls = outline_state(text, path), outline_state(live, path)
            check('...trusted on both', ts['trustworthy'] and ls['trustworthy'],
                  str((ts['problems'], ls['problems'])))
            # Row 5: the live helper reads the module's BOUNDS_CHORD_EPS at call
            # time, the text parser's default was bound at import, so this moves
            # the live side alone.
            saved = kicad_parser.BOUNDS_CHORD_EPS
            kicad_parser.BOUNDS_CHORD_EPS = kicad_parser.EDGE_CHORD_EPS
            try:
                moved = live_read(drawings, pairs)[0]
            finally:
                kicad_parser.BOUNDS_CHORD_EPS = saved
            bites += moved is not None and apart(tb, moved) > TOL
        check("the boards have teeth: KiCad's exact box stands over 1 um past the chords on some",
              missed > 0, f'{missed} of {len(boards)}')
        check("the live reading has teeth: another chord sag under the kipy helper "
              "moves its bounds past the text parser's on some",
              bites > 0, f'{bites} of {len(boards)}')
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
