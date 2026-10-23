"""Which way a connector OPENS: its mating face, read off its drawn body (#1155).

Before this module the toolchain could say which edge a connector's courtyard
was NEAREST, and nothing else. A connector seated backwards, its shell lying
over the board and its mating face pointing inward, is still nearest the edge
it was declared on whenever the shell's back grazes that edge -- so
`rule_edge_connector`, board_context's "Mating face", check_assembly and
board_score all passed sonde_xilinx's reversed J2 (#1155).

The fact that decides it is where the BODY sits relative to the PAD ROW. A
horizontal or edge-mount connector solders at the back and extends forward to
the opening: a DSUB edge-mount's shell runs 14 mm past its pads, a USB-C
receptacle's 7 mm, a right-angle header's plastic 9 mm. So the mating face
is the direction of the body's centre from the pad bbox's centre, in the
footprint's own frame, mapped through the pose to a board edge.

It abstains -- `face is None`, with a reason -- whenever that reading is not
a fact:

* no pads (nothing for a body to be offset FROM);
* a board-copper plug (`floorplan.is_board_copper_plug`): the BOARD is the
  mating body, and its courtyard points inward by construction (StickHub J1);
* a `Vertical` footprint name: a vertical mount mates perpendicular to the
  board, so it has no in-plane face, however its plastic is drawn (a JST-XH
  vertical courtyard sits 0.5 mm off its pad row);
* no source offset by at least `FACE_MIN_OFFSET_MM` and `FACE_DOMINANCE`
  times its cross-axis offset (a symmetric part: a box header, a terminal);
* sources that disagree (courtyard says north, fab says south: glasgow SW1);
* a pose more than 30 degrees off every quarter turn (within 15 degrees
  of a diagonal).

Two sources vote: the courtyard (when that rung answered) and the drawn body
(`drawn_local`, fab else silk -- AFTER #1201's guard, so a pin-1 dot casts no
vote: unguarded, CM5_MINIMA_3's USB-C J101/USB701 read east off their dots;
guarded, their silk shells read west, which is right). Every decisive vote
must agree.

A face is STRONG when its offset reaches `FACE_STRONG_OFFSET_MM`, and only a
strong face may fail a declared edge at ERROR (`floorplan.edge_facing`); a
weaker one is reported at WARN. The phase verifier swept KiCad's stock
library: confidently WRONG on three receptacles whose PAD layout, not their
housing, sets the offset -- CUI SJ1-3533NG (x2, -x at 0.90 mm; its twin
SJ1-3535NG, same housing, reads -y) and Neutrik NMJ6HCD3 (-x at 1.36 mm; the
opening is +x) -- while 60+ correct micro-USB / USB-C faces sit in the same
0.6-1.6 mm range. Above 2 mm no wrong face is known (227 checked). A weak
face that is itself wrong is NOT caught: a jack seated with its opening into
the board by a search that points even a weak face at the edge reads as a
pass to the facing conjunct.

A THIRD source, since #1245: the "PCB Edge" marker many footprints draw, a
text beside a line where the board edge goes (`marker_face_local`, carried
on `body.BodyGeometry.edge_marker`). It reads the LINE, never the text's
position (sonde_u's DSUBs put the text 21.55 mm along the line), and it
never names a face alone: a marker that agrees makes the body's face STRONG
(basis `+marker`), one that contradicts makes it abstain -- never fail --
and no marker leaves today's reading. Calibrated on KiCad 10's 154 marked
stock footprints: it agrees with all
73 strong body faces at every quarter turn, agrees with 75 weak ones and
vetoes one (the Pico W module's RF-edge marker). The three wrong jacks above
draw no marker, so for them the weak tier is still the only guard.

Known limit: a card-edge PLUG in a file with no `attrs` (interf_u's ISA
BUS1: 62 netted SMD fingers, no model) is not recognised as one, and reads
inward (0.64 mm, weak). Its class is none, so it is graded only when an
author declares it `edge_mount` / `through_edge`.

The board frame uses the sign `connector_geometry.rect` and
`legality.rotate_local_bounds` use, with NO extra mirror for a B-side part:
the board file stores a B-side footprint's local coordinates already mirrored,
measured on sonde_u J2, orangecrab J4 and rp2350 J3.

Measured over the tracked corpus and the fa10 human boards: every decisive
receptacle faces the edge it is seated on.
"""
import math
from typing import NamedTuple, Optional, Tuple

#: A body must sit at least this far (mm) off its pad row's centre to name a
#: face. The census's gap among receptacles is 0.085 (the most offset that
#: names none) to 0.59 (orangecrab J3's courtyard); connectors and actuators
#: reach 0.50 without a face. 0.57 rather than 0.55: two stock footprints
#: (microSD_HC_Molex_47219-2001, CM5's SW501) sit at exactly 0.55, where float
#: rounding would decide them.
FACE_MIN_OFFSET_MM = 0.57
#: ...and a face this far off is STRONG: the only kind that fails a declared
#: edge at ERROR. See the module docstring for the counterexamples below it.
FACE_STRONG_OFFSET_MM = 2.0
#: Both thresholds compare with this slack: the stock SJ1-3535NG's courtyard
#: offset is 2.0 mm on paper and 1.9999999999999996 in float.
_TOL_MM = 1e-6
#: ...and by this factor more on the face axis than across it. The lowest
#: ratio among faced receptacles is 2.02 (coldfire J201's courtyard).
FACE_DOMINANCE = 1.5
#: A pose more than this far off the nearest cardinal angle names no edge.
CARDINAL_TOL_DEG = 30.0

_UNIT = {'+x': (1.0, 0.0), '-x': (-1.0, 0.0),
         '+y': (0.0, 1.0), '-y': (0.0, -1.0)}

#: #1245: the line a "PCB Edge" marker labels must be at least this many
#: times nearer its text than the nearest line that would name another face
#: (`marker_face_local`)...
MARKER_NEARER_RATIO = 2.0
#: ...and at least this much nearer (mm). Measured on KiCad 10's 154 marked
#: stock footprints: no rival there is
#: nearer than 2.57 times (Amphenol 12401948E412A) or 2.15 mm (Technik
#: TWP-4002D-H3) farther than the line read, so neither floor binds on the
#: library; they are there for a footprint whose text sits between two lines.
MARKER_NEARER_MM = 1.0
#: A marker text whose baseline is more than this far (degrees) off both axes
#: of the footprint names no line.
_MARKER_AXIS_TOL_DEG = 1.0
#: A drawn line this far (mm) off an axis is not straight along it.
_MARKER_STRAIGHT_MM = 1e-3


class MarkerFace(NamedTuple):
    """What a footprint's "PCB Edge" marker says, in its UNROTATED frame
    (#1245): the face it implies, or None with the reason it abstains."""
    face: Optional[str]
    reason: str
    # (axis of the line 'x' | 'y', its position, its layer, how near the
    # text it is, mm), or None when no line was chosen.
    line: Optional[Tuple] = None


class LocalFace(NamedTuple):
    """A footprint's mating face in its own UNROTATED frame."""
    face: Optional[str]          # '+x' | '-x' | '+y' | '-y', or None
    offset_mm: float             # the decisive offset (largest vote), 0 if none
    basis: str                   # 'courtyard+fab', 'silk', ...; '' if abstained
    reason: str                  # why it abstained; '' when it did not
    votes: Tuple = ()            # ((source, face_or_None, ox, oy), ...)
    # offset_mm >= FACE_STRONG_OFFSET_MM, or a face the footprint's "PCB
    # Edge" marker confirms (#1245): may fail a declared edge at ERROR.
    strong: bool = False
    # #1245: the marker's reading (`MarkerFace`), None when it draws none.
    marker: Optional[MarkerFace] = None


def _centre(b):
    return (b[0] + b[2]) / 2.0, (b[1] + b[3]) / 2.0


def _vote(body, pads):
    """`(face_or_None, ox, oy)` for one body bbox against the pad bbox."""
    bx, by = _centre(body)
    px, py = _centre(pads)
    ox, oy = bx - px, by - py
    major, minor = max(abs(ox), abs(oy)), min(abs(ox), abs(oy))
    if major < FACE_MIN_OFFSET_MM - _TOL_MM or major < FACE_DOMINANCE * minor:
        return None, ox, oy
    if abs(ox) >= abs(oy):
        return ('+x' if ox > 0 else '-x'), ox, oy
    return ('+y' if oy > 0 else '-y'), ox, oy


def is_vertical_name(fp) -> bool:
    """True when the footprint's ITEM name says it is a vertical mount.

    The item name, not the `lib:item` string: the text parser keeps the
    library prefix and the pcbnew path does not (`part_class` has the same
    rule for the same reason)."""
    name = str(getattr(fp, 'footprint_name', '') or '')
    item = name.rsplit(':', 1)[-1].lower()
    return 'vertical' in item and 'horizontal' not in item


def mating_face_local(fp, geom) -> LocalFace:
    """The mating face of one footprint, from its `body.BodyGeometry`."""
    if geom is None or getattr(geom, 'pads_local', None) is None:
        return LocalFace(None, 0.0, '', 'no pads to be offset from')
    if is_vertical_name(fp):
        return LocalFace(None, 0.0, '',
                         'a vertical mount (footprint name): it mates '
                         'perpendicular to the board')
    from .floorplan import is_board_copper_plug
    if is_board_copper_plug(fp):
        return LocalFace(None, 0.0, '',
                         'a board-copper plug: the board is the mating body')
    from .body import SOURCE_COURTYARD, SOURCE_FAB, SOURCE_SILK
    pads = geom.pads_local
    srcs = []
    if geom.source == SOURCE_COURTYARD and geom.body_local is not None:
        srcs.append(('courtyard', geom.body_local))
    if geom.drawn_source in (SOURCE_FAB, SOURCE_SILK) \
            and geom.drawn_local is not None:
        srcs.append((geom.drawn_source, geom.drawn_local))
    votes = tuple((name,) + _vote(b, pads) for name, b in srcs)
    decisive = [v for v in votes if v[1] is not None]
    if not decisive:
        # #1245: the marker names no face on its own; read for disclosure.
        return LocalFace(None, 0.0, '',
                         'symmetric about its pads' if votes
                         else 'no drawn body', votes,
                         marker=marker_face_local(fp, geom))
    faces = sorted({v[1] for v in decisive})
    if len(faces) > 1:
        return LocalFace(None, 0.0, '', 'sources disagree: ' + ', '.join(
            '%s %s' % (v[0], v[1]) for v in decisive), votes,
            marker=marker_face_local(fp, geom))
    offset = max(max(abs(v[2]), abs(v[3])) for v in decisive)
    lf = LocalFace(faces[0], round(offset, 4),
                   '+'.join(v[0] for v in decisive), '', votes,
                   strong=offset >= FACE_STRONG_OFFSET_MM - _TOL_MM)
    mk = marker_face_local(fp, geom)
    if mk is None or mk.face is None:
        return lf._replace(marker=mk)
    if mk.face == lf.face:
        # The library's own word for where the board edge goes, read off a
        # different drawing than the body: a weak face it confirms is strong.
        return lf._replace(strong=True, basis=lf.basis + '+marker', marker=mk)
    return LocalFace(None, 0.0, '',
                     'its PCB-edge marker puts the board edge on its %s side, '
                     'against the %s face its body reads' % (mk.face, lf.face),
                     votes, marker=mk)


def marker_face_local(fp, geom) -> Optional[MarkerFace]:
    """The face a footprint's "PCB Edge" marker implies (#1245), in its own
    UNROTATED frame, or None when it draws no marker.

    The marker is a text item ("PCB Edge", "Board edge", ...:
    `parser.EDGE_MARKER_TEXT`) beside a straight line the library drew where
    the board edge goes. The TEXT's position is not the reading: it sits at
    an end of the line, often past the pad row (sonde_u's DSUB J1/J2 put it
    21.55 mm along, which reads -x for a +y part), and stock DIN41612
    footprints put it 2 mm off the line with an arrow pointing at it. The
    LINE is: the board edge, with the pads on the board side of it, so the
    mating face is the side of the line away from the pad bbox's centre.

    Which line: one PARALLEL to the text's baseline (the text's angle less
    the footprint's own rotation, since a board file stores it on screen) --
    that is what keeps an arrow's shaft, perpendicular to the edge, out --
    on any layer but a courtyard or copper, nearest the text (its distance
    off the line's run plus how far past either end the text sits). It
    abstains when:

    * the text's baseline is off both axes;
    * no parallel line is drawn;
    * the line runs within `FACE_MIN_OFFSET_MM` of the pad row's centre --
      too near to say which side the pads are on;
    * a line that would name another face is not clearly farther: at least
      `MARKER_NEARER_RATIO` times and `MARKER_NEARER_MM` farther from the
      text;
    * two marker texts on one footprint disagree.

    The marker never names a face on its own: `mating_face_local` uses it to
    confirm the body's face (strong) or to veto it (an abstention), so a
    marker read wrongly can at worst cost a face, never invent one. Marker
    or not, the courtyard and body votes are unchanged."""
    marker = getattr(geom, 'edge_marker', None)
    if marker is None or not getattr(marker, 'texts', None):
        return None
    pads = getattr(geom, 'pads_local', None)
    if pads is None:
        return MarkerFace(None, 'no pads to put on the board side of it')
    pcx, pcy = _centre(pads)
    rot = float(getattr(fp, 'rotation', 0.0) or 0.0)
    eps = _MARKER_STRAIGHT_MM
    faces = []
    best_line = None
    for tx, ty, angle, _layer, _text in marker.texts:
        rel = (angle - rot) % 180.0
        if min(rel, 180.0 - rel) <= _MARKER_AXIS_TOL_DEG:
            axis = 'y'            # a horizontal baseline: a line y = const
        elif abs(rel - 90.0) <= _MARKER_AXIS_TOL_DEG:
            axis = 'x'
        else:
            return MarkerFace(None, 'its PCB-edge marker text runs off both '
                                    'axes, so it labels no line')
        cands = []
        for x1, y1, x2, y2, layer in marker.lines:
            if layer.endswith(('CrtYd', '.Cu')):
                continue
            if axis == 'y' and abs(y1 - y2) <= eps and abs(x1 - x2) > eps:
                lo, hi = sorted((x1, x2))
                pos, perp, along = y1, abs(ty - y1), tx
                off = pos - pcy
                face = '+y' if off > 0 else '-y'
            elif axis == 'x' and abs(x1 - x2) <= eps and abs(y1 - y2) > eps:
                lo, hi = sorted((y1, y2))
                pos, perp, along = x1, abs(tx - x1), ty
                off = pos - pcx
                face = '+x' if off > 0 else '-x'
            else:
                continue
            near = perp + max(0.0, lo - along, along - hi)
            cands.append((near, face, off, pos, layer))
        if not cands:
            return MarkerFace(None, 'no straight line beside its PCB-edge '
                                    'marker runs along the text')
        cands.sort(key=lambda c: c[0])
        near, face, off, pos, layer = cands[0]
        line = (axis, round(pos, 4), layer, round(near, 4))
        if abs(off) < FACE_MIN_OFFSET_MM - _TOL_MM:
            return MarkerFace(None, 'the line its PCB-edge marker labels runs '
                                    'through its pad row', line)
        rival = next((c for c in cands[1:] if c[1] != face), None)
        if rival is not None and (
                rival[0] < MARKER_NEARER_RATIO * near - _TOL_MM
                or rival[0] - near < MARKER_NEARER_MM - _TOL_MM):
            return MarkerFace(None, 'two lines beside its PCB-edge marker '
                                    'name different faces', line)
        faces.append(face)
        best_line = best_line or line
    if len(set(faces)) > 1:
        return MarkerFace(None, 'its PCB-edge markers disagree: '
                          + ', '.join(sorted(set(faces))))
    return MarkerFace(faces[0], '', best_line)


def facing_edge(face: Optional[str], rotation: float) -> Optional[str]:
    """The board edge (`west|north|east|south`) a local face points at.

    Board y grows southward. The transform is `connector_geometry.rect`'s:
    local (u, v) -> (cos r * u + sin r * v, -sin r * u + cos r * v)."""
    if face not in _UNIT:
        return None
    ux, uy = _UNIT[face]
    r = math.radians(rotation or 0.0)
    c, s = math.cos(r), math.sin(r)
    vx, vy = c * ux + s * uy, -s * ux + c * uy
    if max(abs(vx), abs(vy)) < math.cos(math.radians(CARDINAL_TOL_DEG)):
        return None
    if abs(vx) >= abs(vy):
        return 'east' if vx > 0 else 'west'
    return 'south' if vy > 0 else 'north'


