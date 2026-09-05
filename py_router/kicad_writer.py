"""
KiCad PCB Writer - Writes routing results to .kicad_pcb files.
"""
from __future__ import annotations

import re
import uuid
from typing import List, Dict, Tuple, Optional

from kicad_parser import Pad, is_kicad_10, _unescape_kicad_string
from routing_utils import pos_key, POSITION_DECIMALS


def _escape_net_name(name: Optional[str]) -> Optional[str]:
    """Escape a net name for a quoted KiCad S-expr token — the exact inverse of
    kicad_parser._unescape_kicad_string (backslash then quote).

    The parser stores each net's DISPLAY name unescaped, but keys name-based net
    dedup on the RAW file text (kicad_parser.extract_nets). So a net whose name
    contains a literal backslash or quote -- e.g. `/GPIO24\\SPI1_RX(MISO)` -- must
    be re-escaped on write, or route.py emits `(net "/GPIO24\\SPI1...")` with a
    SINGLE backslash where the original board had it DOUBLE-escaped. The two raw
    tokens then parse to two different synthetic net_ids for one net, so the
    routed segment lands on a duplicate net and check_drc flags a phantom
    pad-segment/via short against the pad (still on the original id). Compounds
    every route.py step (issue #312, neo6502). Escape backslash FIRST, then quote.
    """
    if name is None:
        return None
    return name.replace('\\', '\\\\').replace('"', '\\"')


def move_copper_text_to_silkscreen(content: str) -> str:
    """
    Move gr_text elements from copper layers to silkscreen layers.

    Text on F.Cu is moved to F.SilkS, text on B.Cu is moved to B.SilkS.
    This prevents text from interfering with routing while preserving it visually.

    Args:
        content: Raw PCB file content

    Returns:
        Modified content with text layers changed
    """
    count = 0

    # Find all gr_text blocks by finding "(gr_text" and matching to closing paren
    i = 0
    result_parts = []
    last_end = 0

    while True:
        # Find next gr_text
        start = content.find('(gr_text', i)
        if start == -1:
            break

        # Find the matching closing paren by counting nesting
        depth = 0
        end = start
        for j in range(start, len(content)):
            if content[j] == '(':
                depth += 1
            elif content[j] == ')':
                depth -= 1
                if depth == 0:
                    end = j + 1
                    break

        # Extract the gr_text block
        block = content[start:end]

        # Check if it's on F.Cu or B.Cu
        layer_match = re.search(r'\(layer\s+"(F\.Cu|B\.Cu)"\)', block)
        if layer_match:
            layer = layer_match.group(1)
            new_layer = 'F.SilkS' if layer == 'F.Cu' else 'B.SilkS'
            new_block = block.replace(f'(layer "{layer}")', f'(layer "{new_layer}")')

            # Add content before this block, then the modified block
            result_parts.append(content[last_end:start])
            result_parts.append(new_block)
            last_end = end
            count += 1

        i = end

    # Add remaining content
    result_parts.append(content[last_end:])

    if count > 0:
        print(f"  Moved {count} text element(s) from copper layers to silkscreen")

    return ''.join(result_parts)


# Graphic primitives that make up copper logos/artwork (no net). Functional copper
# is zones / segments / vias / pads — never standalone graphics — so moving these
# off copper is safe and only affects decoration.
_COPPER_GRAPHIC_TAGS = (
    'fp_poly', 'gr_poly', 'fp_line', 'gr_line', 'fp_circle', 'gr_circle',
    'fp_arc', 'gr_arc', 'fp_rect', 'gr_rect', 'fp_curve', 'gr_curve',
)


def strip_zero_length_edge_cuts(content: str) -> str:
    """Drop null-length Edge.Cuts primitives from board content.

    A segment whose start equals its end draws nothing and mills nothing, but
    KiCad treats it as a MALFORMED OUTLINE: it reports one `invalid_outline`
    violation per occurrence ("segment has null or very small length: 0 nm") and
    then grinds. Measured on the pinci corpus board, which carries 335 of them
    (12 board-level, 323 inside footprints):

        kicad-cli pcb drc, as-is                    656 s
        kicad-cli pcb drc, --refill-zones           543 s
        kicad-cli pcb drc, these primitives removed 2.7 s      (~280x)

    with the real findings unchanged -- the 199 phantom invalid_outline vanish
    and every other violation is byte-identical (shorting_items 8->8,
    solder_mask_bridge 16->16, unconnected 101->101).

    Applied on WRITE, beside move_copper_*_to_silkscreen, because our writers
    copy the input file's text: without this, garbage in the user's source
    passes straight through into the routed board we hand back, and THEIR KiCad
    DRC pays the same cost. The read side already tolerates these (see the
    degenerate-line guard in kicad_parser._extract_board_contours), so this is
    purely about not propagating them.

    Deliberately ONLY the zero-length class. Repairing genuinely OPEN outlines
    (bridging gaps, culling fragments) is a real geometry change that made DRC
    worse on both corpus boards that needed it, and stays opt-in in
    tests/stress/fix_outline_gaps.py.
    """
    removed = 0
    out = []
    i = 0
    for m in re.finditer(r'\((gr_line|fp_line)\b', content):
        if m.start() < i:
            continue
        s0 = m.start()
        depth = 0
        end = None
        for j in range(s0, min(s0 + 4000, len(content))):
            if content[j] == '(':
                depth += 1
            elif content[j] == ')':
                depth -= 1
                if depth == 0:
                    end = j + 1
                    break
        if end is None:
            continue
        blk = content[s0:end]
        if '"Edge.Cuts"' in blk:
            a = re.search(r'\(start ([-\d.]+) ([-\d.]+)\)', blk)
            b = re.search(r'\(end ([-\d.]+) ([-\d.]+)\)', blk)
            if a and b and float(a.group(1)) == float(b.group(1)) \
                    and float(a.group(2)) == float(b.group(2)):
                out.append(content[i:s0])
                i = end
                removed += 1
    if not removed:
        return content
    out.append(content[i:])
    print(f"Dropped {removed} zero-length Edge.Cuts primitive(s) "
          f"(KiCad grades these as invalid_outline and slows to a crawl)")
    return ''.join(out)


def move_copper_graphics_to_silkscreen(content: str) -> str:
    """Move graphic primitives (logos / artwork drawn as polys, lines, arcs, ...)
    from copper layers to silkscreen, mirroring move_copper_text_to_silkscreen:
    F.Cu -> F.SilkS, B.Cu -> B.SilkS.

    Only NET-LESS copper graphics (e.g. a copper OSHW logo, net 0 / no net) are
    moved: unmodelled by the router, plane pours and routed copper run straight
    over them and short (orangecrab: one F.Cu logo shorted ~30 plane traces/vias);
    relocating to silkscreen preserves them visually while taking them out of
    copper. A NET-TIED copper graphic is real functional copper (#337 models it
    as an immutable obstacle and DRC treats it as copper) and is LEFT IN PLACE --
    moving it would delete a real connection.
    """
    count = 0
    for tag in _COPPER_GRAPHIC_TAGS:
        token = '(' + tag
        tlen = len(token)
        result_parts = []
        last_end = 0
        i = 0
        while True:
            start = content.find(token, i)
            if start == -1:
                break
            # Require a token boundary so '(gr_line' doesn't match '(gr_linex'.
            nxt = content[start + tlen: start + tlen + 1]
            if nxt and (nxt.isalnum() or nxt == '_'):
                i = start + tlen
                continue
            depth = 0
            end = start
            for j in range(start, len(content)):
                if content[j] == '(':
                    depth += 1
                elif content[j] == ')':
                    depth -= 1
                    if depth == 0:
                        end = j + 1
                        break
            block = content[start:end]
            layer_match = re.search(r'\(layer\s+"(F\.Cu|B\.Cu)"\)', block)
            # #337: a NET-TIED copper graphic is FUNCTIONAL copper (the router
            # models it as an immutable obstacle and check_drc treats it as
            # copper) -- relocating it to silk would DELETE a real connection and
            # re-expose the shorts #337 catches. Only net-less decoration (a
            # copper logo, net 0 / no net) is safe to move off copper.
            net_match = re.search(r'\(net\s+(\d+)(?:\s+"([^"]*)")?\)', block)
            net_tied = net_match and (int(net_match.group(1)) != 0
                                      or (net_match.group(2) or '') != '')
            # #369 A6: KiCad 10 boards reference nets by NAME only -- the
            # numeric-first regex missed (net "GND") entirely, classing real
            # net-tied copper as decoration and deleting the connection the
            # router still models (#337 immutable-copper class).
            if not net_tied:
                name_match = re.search(r'\(net\s+"((?:[^"\\]|\\.)*)"\)', block)
                net_tied = bool(name_match and name_match.group(1) != '')
            if layer_match and not net_tied:
                layer = layer_match.group(1)
                new_layer = 'F.SilkS' if layer == 'F.Cu' else 'B.SilkS'
                block = block.replace(f'(layer "{layer}")', f'(layer "{new_layer}")')
                count += 1
            result_parts.append(content[last_end:start])
            result_parts.append(block)
            last_end = end
            i = end
        result_parts.append(content[last_end:])
        content = ''.join(result_parts)

    if count > 0:
        print(f"  Moved {count} copper graphic(s)/logo(s) to silkscreen")

    return content


def generate_segment_sexpr(start: Tuple[float, float], end: Tuple[float, float],
                           width: float, layer: str, net_id: int,
                           net_name: str = None) -> str:
    """Generate KiCad S-expression for a track segment.

    Args:
        net_name: If provided, output KiCad 10 format (net "name") instead of (net id).
    """
    net_str = f'(net "{_escape_net_name(net_name)}")' if net_name is not None else f'(net {net_id})'
    return f'''	(segment
		(start {start[0]:.6f} {start[1]:.6f})
		(end {end[0]:.6f} {end[1]:.6f})
		(width {width})
		(layer "{layer}")
		{net_str}
		(uuid "{uuid.uuid4()}")
	)'''


def generate_gr_line_sexpr(start: Tuple[float, float], end: Tuple[float, float],
                           width: float, layer: str) -> str:
    """Generate KiCad S-expression for a graphic line (for non-copper layers)."""
    return f'''	(gr_line
		(start {start[0]:.6f} {start[1]:.6f})
		(end {end[0]:.6f} {end[1]:.6f})
		(stroke
			(width {width})
			(type solid)
		)
		(layer "{layer}")
		(uuid "{uuid.uuid4()}")
	)'''


DEFAULT_VIA_TENTING = {'tenting': '(front yes) (back yes)'}

VIA_PROTECTION_TOKEN_ORDER = ('tenting', 'covering', 'plugging', 'capping', 'filling')




def via_protection_sexpr(tenting_attrs: dict = None,
                         net_name: str = None,
                         inherit_when_unspecified: bool = False) -> str:
    """The `(tenting ...)` / `(covering ...)` / ... fragment to emit for a via.

    `tenting_attrs` is a Via's parsed spec ({token: raw inner text}); passing it
    keeps what the board actually specified. Without it the previous behavior
    stands -- front+back tenting on KiCad 10 output -- since there is no
    board-level policy to consult here (#489 §8).

    `inherit_when_unspecified` is what an EXISTING via needs, and it exists
    because `None` and `{}` cannot answer the question on their own (#741):

      a real spec, either way            -> emit it
      empty + inherit_when_unspecified   -> emit NOTHING. The board said
                                            nothing about this via, so the via
                                            keeps inheriting
                                            `(setup (tenting ...))` -- which is
                                            what it had before it was lifted.
      empty, KiCad 10 (net_name given)   -> the #489 default, front+back
      empty, numeric net                 -> nothing, as before

    Pass `inherit_when_unspecified=True` whenever you are RE-PLACING a via --
    rip-up, sub-grid nudge, tap relocation. `{}` is exactly what
    `Via.tenting_attrs` holds for a via that carries no spec, so handing it back
    verbatim without this flag re-stamps the via with front+back tenting it
    never had.

    A KEYWORD rather than a sentinel value, deliberately. A sentinel object has
    to survive being copied, and the copy idiom this repo actually uses for a
    spec is `dict(via.tenting_attrs or {})` (fanout_clearance.py, plane_io.py)
    -- which turns any dict-shaped sentinel back into a plain `{}` and silently
    restores the bug. Carrying the DECISION separately from the VALUE cannot be
    undone by copying the value.
    """
    # No spec -> emit NOTHING, in either dialect. The via then carries no
    # protection token, which in KiCad means `*_MODE_FROM_BOARD`: it follows the
    # board's own `(setup ...)` policy.
    #
    # This used to fall through to DEFAULT_VIA_TENTING on KiCad-10 output, and
    # that was wrong in KiCad's own terms. Probed against pcbnew 10.0.0: a via
    # left at FROM_BOARD serialises with NO token, and the token appears ONLY
    # when the via explicitly overrides -- so stamping one converted an
    # inheriting via into an OVERRIDE, which is not a thing this tool was ever
    # asked to decide.
    #
    # Measured cost of the old behaviour, over 886 corpus boards: three
    # (nanovoltmeter_marge, hexberry_fpga, pedal_404) declare
    # `(tenting (front no) (back no))` board-wide -- do not tent -- and every
    # via the tool added to them was stamped `(front yes) (back yes)`, silently
    # contradicting the board. Tenting a via meant to stay exposed is a fab
    # error, not a cosmetic one. On a board whose policy is KiCad's factory
    # default the two agree, which is why this hid for so long.
    #
    # `inherit_when_unspecified` is therefore now the behaviour in ALL cases.
    # The parameter is kept because callers pass it and it still records, at the
    # call site, that a via ALREADY EXISTED -- see generate_via_sexpr.
    spec = tenting_attrs if tenting_attrs else {}
    if not spec:
        return ""
    def _one(token: str, inner: str) -> str:
        # The parsed inner text keeps the source file's line breaks and tabs;
        # collapse them so the re-emitted token reads on one line. Same s-expr.
        inner = " ".join((inner or '').split())
        return f"({token} {inner})" if inner else f"({token})"

    parts = [_one(t, spec[t]) for t in VIA_PROTECTION_TOKEN_ORDER if t in spec]
    # Emit an unrecognised token rather than dropping it -- losing the spec is
    # the bug being fixed.
    parts += [_one(t, v) for t, v in spec.items()
              if t not in VIA_PROTECTION_TOKEN_ORDER]
    return "".join(f"\n\t\t{p}" for p in parts)


def via_net_name(net_id: int, net_id_to_name: dict) -> Optional[str]:
    """This board's name for a net id, or None to mean "use the numeric dialect".

    THE one resolver for every via emit site (#749 D). `net_id_to_name` has no
    key `0` on any board -- `extract_nets` records `name_to_id[""] = 0` but never
    builds a Net for id 0 -- so a legitimate no-net via missed the lookup at
    every site and fell back to numeric. Measured: orangecrab_ext_pll resolves
    169 nets and has no key 0.

    That mattered more than a cosmetic dialect slip. `generate_via_sexpr` picks
    the net dialect from `net_name` and the protection tokens from
    `tenting_attrs` INDEPENDENTLY, so a numeric `(net N)` emitted alongside a
    spec used to be a shape `extract_vias` could not read back at all -- the
    barrel vanished from the model rather than merely losing its spec (#748,
    fixed in the parser; this keeps the writer from producing the shape in the
    first place). It also keeps a name-net board from acquiring numeric refs,
    the mixed-dialect state `tests/stress/fix_mixed_net_refs.py` exists to undo.

    An id the map genuinely does not know still answers None: that is a caller
    passing an id from another board, and guessing a name for it would be worse
    than the numeric ref.
    """
    if not net_id_to_name:
        return None
    name = net_id_to_name.get(net_id)
    return '' if name is None and net_id == 0 else name


def prevailing_via_protection(vias) -> Optional[dict]:
    """The protection spec most of a board's existing vias carry, or None.

    Used as the default for vias this tool ADDS, so a new via follows the
    board's own convention instead of a hardcoded front+back tenting (#489 §8).
    hackrf_one is the motivating case: all 498 of its vias explicitly declare
    covering/plugging/capping/filling = no, and the tool would have stamped
    tenting yes on every via it added to that board.

    Deterministic: ties break on the canonical spec text, not dict order.
    """
    from collections import Counter

    counts = Counter()
    canonical = {}
    for via in vias or ():
        spec = getattr(via, 'tenting_attrs', None)
        if not spec:
            continue
        key = tuple(sorted((t, " ".join((v or '').split())) for t, v in spec.items()))
        counts[key] += 1
        canonical[key] = dict(spec)
    if not counts:
        return None
    best = min(counts.items(), key=lambda kv: (-kv[1], kv[0]))[0]
    return canonical[best]


def prevailing_via_protection_in_text(content: str) -> Optional[dict]:
    """`prevailing_via_protection` for a writer that holds the board TEXT rather
    than parsed Via objects (the plane / repair writers work on file text)."""
    from kicad_parser import _extract_via_protection_attrs

    class _V:
        __slots__ = ('tenting_attrs',)

        def __init__(self, spec):
            self.tenting_attrs = spec

    specs = _extract_via_protection_attrs(content)
    if not specs:
        return None
    return prevailing_via_protection([_V(s) for s in specs.values()])


def generate_via_sexpr(x: float, y: float, size: float, drill: float,
                       layers: List[str], net_id: int, free: bool = False,
                       net_name: str = None, tenting_attrs: dict = None,
                       inherit_when_unspecified: bool = False) -> str:
    """Generate KiCad S-expression for a via.

    Args:
        free: If True, adds (free yes) to prevent KiCad from auto-assigning net based on overlapping tracks.
        net_name: If provided, output KiCad 10 format (net "name") instead of (net id).
        tenting_attrs: The via's own protection spec as parsed from the board
            (Via.tenting_attrs). Pass it for any via that already existed so a
            ripped-and-re-placed via keeps its real tenting/plugging/filling
            instead of being re-stamped with front+back tenting (#489 §8).
        inherit_when_unspecified: Pass True alongside it for a RE-PLACED via.
            An empty spec then emits nothing, so a via that was inheriting the
            board's `(setup (tenting ...))` keeps inheriting it rather than
            gaining an explicit token it never had (#741). See
            via_protection_sexpr.
    """
    layers_str = '" "'.join(layers)
    free_str = "\n\t\t(free yes)" if free else ""
    net_str = f'(net "{_escape_net_name(net_name)}")' if net_name is not None else f'(net {net_id})'
    # KiCad 10 adds structured tenting/covering/plugging fields after layers
    tenting_str = via_protection_sexpr(tenting_attrs, net_name,
                                      inherit_when_unspecified)
    return f'''	(via
		(at {x:.6f} {y:.6f})
		(size {size})
		(drill {drill})
		(layers "{layers_str}"){tenting_str}{free_str}
		{net_str}
		(uuid "{uuid.uuid4()}")
	)'''


def generate_gr_text_sexpr(text: str, x: float, y: float, layer: str,
                           size: float = 0.5, angle: float = 0) -> str:
    """Generate KiCad S-expression for a graphic text label."""
    return f'''	(gr_text "{text}"
		(at {x:.6f} {y:.6f} {angle})
		(layer "{layer}")
		(uuid "{uuid.uuid4()}")
		(effects
			(font
				(size {size} {size})
				(thickness 0.1)
			)
		)
	)'''


def _polygon_area(poly) -> float:
    """Unsigned shoelace area of a polygon given as [(x, y), ...]."""
    a = 0.0
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i][0], poly[i][1]
        x2, y2 = poly[(i + 1) % n][0], poly[(i + 1) % n][1]
        a += x1 * y2 - x2 * y1
    return abs(a) / 2.0


def _point_in_polygon(x: float, y: float, poly) -> bool:
    """Even-odd ray cast. Boundary cases are don't-care here: this only feeds
    the overlap test, and a zone grazing another's edge needs no tie-break."""
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i][0], poly[i][1]
        x2, y2 = poly[(i + 1) % n][0], poly[(i + 1) % n][1]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            inside = not inside
    return inside


def _segments_properly_cross(p1, p2, p3, p4) -> bool:
    """True only when p1p2 and p3p4 cross TRANSVERSALLY. Collinear or merely
    touching segments are excluded on purpose -- see _polygons_overlap."""
    def orient(a, b, c):
        v = ((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
        # int(), not bare bool arithmetic: zone points reach here as numpy
        # scalars on some boards, and numpy 2.x REMOVED np.bool_ - np.bool_
        # (TypeError). That killed route_planes outright -- the step wrote no
        # board, downstream steps hit FileNotFoundError, and the board was
        # dropped from grading as chain=BROKEN rather than reported. Identical
        # for Python floats (bool is an int subclass).
        return int(v > 1e-12) - int(v < -1e-12)
    d1, d2 = orient(p3, p4, p1), orient(p3, p4, p2)
    d3, d4 = orient(p1, p2, p3), orient(p1, p2, p4)
    return d1 * d2 < 0 and d3 * d4 < 0


def _overlap_area(pa, pb, samples: int = 64) -> float:
    """Approximate shared area (mm^2) of two outlines, by sampling a fixed grid
    over their bounding-box intersection.

    Deliberately NOT exact polygon clipping: all we need is "is the contested
    region big enough to hold copper", and an exact clipper (shapely) is only
    LAZILY importable here -- KiCad's bundled Python may not have it, and a
    silent fallback would let the GUI and CLI assign DIFFERENT priorities,
    which is the CLI/GUI drift this codebase keeps getting bitten by. A fixed
    grid is pure Python, dependency-free and deterministic, so both front ends
    always agree.
    """
    ax1 = min(p[0] for p in pa); ax2 = max(p[0] for p in pa)
    ay1 = min(p[1] for p in pa); ay2 = max(p[1] for p in pa)
    bx1 = min(p[0] for p in pb); bx2 = max(p[0] for p in pb)
    by1 = min(p[1] for p in pb); by2 = max(p[1] for p in pb)
    x1, x2 = max(ax1, bx1), min(ax2, bx2)
    y1, y2 = max(ay1, by1), min(ay2, by2)
    if x2 <= x1 or y2 <= y1:
        return 0.0
    box = (x2 - x1) * (y2 - y1)
    hits = 0
    for i in range(samples):
        x = x1 + (i + 0.5) * (x2 - x1) / samples
        for j in range(samples):
            y = y1 + (j + 0.5) * (y2 - y1) / samples
            if _point_in_polygon(x, y, pa) and _point_in_polygon(x, y, pb):
                hits += 1
    return box * hits / float(samples * samples)


def _polygons_overlap(pa, pb, eps: float = 0.02,
                      min_area: float = 0.01) -> bool:
    """True when two zone outlines contend for a MEANINGFUL patch of copper.

    Two things must not count as overlap, and each bit us in turn:

    * ADJACENCY. route_planes' Voronoi cells tile the board, so neighbours
      share a boundary and their vertices lie exactly ON each other's edges --
      where the even-odd rule is undefined, so a plain point-in-polygon test
      calls adjacent pairs "overlapping".
    * NUMERICAL SLIVERS. Those shared boundaries are floating point, so
      neighbouring edges routinely cross by nanometres. Topology alone
      (transversal crossing / vertex containment) flags those too: on
      scalenode_cm4 it reported 31 contending pairs of which exactly ONE had
      real area -- 29 were under 1e-4 mm^2. Re-prioritising those would have
      changed the fill of zone pairs that were never ambiguous.

    So: cheap topology first (bbox reject, transversal crossing, or a vertex
    nudged INWARD toward its own centroid -- which catches nesting, where no
    edges cross), then an AREA test. `min_area` = 0.01 mm^2 is a 0.1x0.1mm
    patch, min_thickness scale: a thinner contested region is opened away by
    the fill's own minimum-thickness rule, so who "wins" it cannot matter.
    Measured corpus areas separate cleanly either side of it (noise <= 1e-4,
    real >= 0.01), so the threshold is not near anything.
    """
    if not pa or not pb:
        return False
    ax1 = min(p[0] for p in pa); ax2 = max(p[0] for p in pa)
    ay1 = min(p[1] for p in pa); ay2 = max(p[1] for p in pa)
    bx1 = min(p[0] for p in pb); bx2 = max(p[0] for p in pb)
    by1 = min(p[1] for p in pb); by2 = max(p[1] for p in pb)
    if ax2 < bx1 or bx2 < ax1 or ay2 < by1 or by2 < ay1:
        return False

    na, nb = len(pa), len(pb)
    topo = False
    for i in range(na):
        if topo:
            break
        a1, a2 = pa[i], pa[(i + 1) % na]
        for j in range(nb):
            if _segments_properly_cross(a1, a2, pb[j], pb[(j + 1) % nb]):
                topo = True
                break

    if not topo:
        def nudged_inside(poly, other):
            cx = sum(p[0] for p in poly) / len(poly)
            cy = sum(p[1] for p in poly) / len(poly)
            for p in poly:
                x, y = p[0], p[1]
                dx, dy = cx - x, cy - y
                n = (dx * dx + dy * dy) ** 0.5
                if n < 1e-12:
                    continue
                if _point_in_polygon(x + dx / n * eps, y + dy / n * eps, other):
                    return True
            return False
        topo = nudged_inside(pa, pb) or nudged_inside(pb, pa)

    return topo and _overlap_area(pa, pb) >= min_area


def zone_overlap_priorities(zones) -> List[int]:
    """Priorities that make an overlapping set of zones fill DETERMINISTICALLY.

    `zones` is a list of (layer, net_id, polygon_points); the return value is a
    parallel list of priorities.

    KiCad fills higher-priority zones first and only lower-priority zones pull
    back, so overlapping zones left at the SAME priority have no defined winner
    and KiCad falls back to KIID (UUID) order. We mint a fresh uuid4 for every
    zone we write, so that made the fill -- and therefore island topology and
    connectivity -- vary between runs over identical copper (usp_obc_v7: a
    nested Net-(U5-GND) pour inside the GND pour, both priority 0 on In1.Cu,
    graded "1 net unconnected" on 4 of 6 UUID rolls). Emitting distinct
    priorities removes the ambiguity at the source.

    SMALLER area wins (higher priority). A pour nested inside a bigger one is
    there precisely because it must own that patch -- give the big pour the tie
    and the small one gets carved up, which is the failure we started from.
    Ties break on (net_id, index) so the result never depends on dict order.
    Zones that overlap nothing keep priority 0, so boards that never had the
    ambiguity are byte-for-byte unaffected.
    """
    n = len(zones)
    prio = [0] * n
    # Group by layer: zones on different layers never contend.
    by_layer: Dict[str, List[int]] = {}
    for i, (layer, _nid, _poly) in enumerate(zones):
        by_layer.setdefault(layer, []).append(i)

    for _layer, idxs in by_layer.items():
        # Connected components of the "overlaps" relation: only zones that
        # actually contend need separated priorities.
        parent = {i: i for i in idxs}

        def find(a):
            while parent[a] != a:
                parent[a] = parent[parent[a]]
                a = parent[a]
            return a

        for pos, i in enumerate(idxs):
            for j in idxs[pos + 1:]:
                if zones[i][1] == zones[j][1]:
                    continue  # same net: overlap is harmless, they merge
                if _polygons_overlap(zones[i][2], zones[j][2]):
                    ri, rj = find(i), find(j)
                    if ri != rj:
                        parent[ri] = rj

        groups: Dict[int, List[int]] = {}
        for i in idxs:
            groups.setdefault(find(i), []).append(i)

        for members in groups.values():
            if len(members) < 2:
                continue  # no contention -> leave at 0
            ordered = sorted(members,
                             key=lambda i: (-_polygon_area(zones[i][2]),
                                            zones[i][1], i))
            for rank, i in enumerate(ordered):
                prio[i] = rank
    return prio


def generate_zone_sexpr(
    net_id: int,
    net_name: str,
    layer: str,
    polygon_points: List[Tuple[float, float]],
    clearance: float = 0.2,
    min_thickness: float = 0.1,
    thermal_gap: float = 0.2,
    thermal_bridge_width: float = 0.2,
    direct_connect: bool = True,
    use_net_name: bool = False,
    priority: int = 0
) -> str:
    """Generate KiCad S-expression for a filled copper zone.

    Args:
        net_id: Net ID (e.g., 690 for GND)
        net_name: Net name string (e.g., "GND")
        layer: Copper layer (e.g., "B.Cu", "In1.Cu")
        polygon_points: List of (x, y) coordinates defining zone boundary
        clearance: Clearance from other copper in mm
        min_thickness: Minimum copper thickness in mm
        thermal_gap: Gap for thermal relief spokes
        thermal_bridge_width: Width of thermal bridges
        direct_connect: If True, use solid/direct pad connections; if False, use thermal relief
        use_net_name: If True, output KiCad 10 format (net "name") instead of (net id)
        priority: Fill priority. KiCad fills HIGHER priority zones first and only
            LOWER-priority zones pull back, so two OVERLAPPING zones left at the
            same priority have no defined winner -- KiCad tie-breaks on the zones'
            KIIDs, i.e. on their UUIDs. Since we mint a fresh uuid4 per run, that
            made the FILL itself vary run to run over bit-identical copper:
            usp_obc_v7 (a Net-(U5-GND) pour nested wholly inside the GND pour,
            both priority 0 on In1.Cu) graded "1 net unconnected" on 4 of 6 UUID
            rolls. Overlapping zones must therefore be emitted at DISTINCT
            priorities -- see zone_overlap_priorities().

    Returns:
        S-expression string for the zone
    """
    # Format polygon points
    pts_lines = []
    for x, y in polygon_points:
        pts_lines.append(f"(xy {x:.6f} {y:.6f})")
    pts_str = " ".join(pts_lines)

    # Connect pads mode: "yes" for direct solid connection, omit for thermal relief
    if direct_connect:
        connect_pads_str = f'''(connect_pads yes
		(clearance {clearance})
	)'''
    else:
        connect_pads_str = f'''(connect_pads
		(clearance {clearance})
	)'''

    # KiCad 10: (net "name"), no (net_name ...) line; KiCad 9: (net id) + (net_name "name")
    if use_net_name:
        net_lines = f'(net "{_escape_net_name(net_name)}")'
    else:
        net_lines = f'(net {net_id})\n\t\t(net_name "{_escape_net_name(net_name)}")'

    # KiCad 10 removes (filled_areas_thickness no) and adds (island_removal_mode 0) in fill
    if use_net_name:
        fill_block = f'''(fill yes
			(thermal_gap {thermal_gap})
			(thermal_bridge_width {thermal_bridge_width})
			(island_removal_mode 0)
		)'''
        extra_zone_props = ''
    else:
        fill_block = f'''(fill yes
			(thermal_gap {thermal_gap})
			(thermal_bridge_width {thermal_bridge_width})
		)'''
        extra_zone_props = '\n\t\t(filled_areas_thickness no)'

    priority_str = f'\n\t\t(priority {int(priority)})' if priority else ''

    return f'''	(zone
		{net_lines}
		(layer "{layer}")
		(uuid "{uuid.uuid4()}")
		(hatch edge 0.5){priority_str}
		{connect_pads_str}
		(min_thickness {min_thickness}){extra_zone_props}
		{fill_block}
		(polygon
			(pts
				{pts_str}
			)
		)
	)'''


def npth_slot_keepout_polygons(pcb_data, dilate: float,
                               arc_segments: int = 32) -> List[Tuple[str, List[Tuple[float, float]]]]:
    """[(ref, polygon_points)] outlining every NPTH SLOT (oval-drill) hole's
    capsule dilated by ``dilate`` mm (#448).

    KiCad's DRC edge provider treats milled NPTH slots as board edge
    (copper_edge_clearance), but its zone FILLER pulls fill back from them at
    only the hole clearance -- so a plane zone poured over a slotted footprint
    (keyboard switches, USB shields) self-flags against the board's edge rule.
    These polygons back copper_pour keepout rule areas that make the filler
    honor the edge clearance. Round NPTH drills are excluded: KiCad grades
    those under hole_clearance, which the filler already honors.

    ``dilate`` should be the board's effective copper-to-edge clearance. A
    small sagitta allowance for the inscribed arc polygons is added here so
    the chord error cannot re-expose sub-clearance fill.
    """
    import math as _math
    from kicad_parser import pad_drill_capsule
    dilate = dilate + 0.01  # inscribed-polygon sagitta allowance
    out = []
    for fp in pcb_data.footprints.values():
        for pad in fp.pads:
            if getattr(pad, 'pad_type', '') != 'np_thru_hole' or pad.drill <= 0:
                continue
            (x1, y1), (x2, y2), r = pad_drill_capsule(pad)
            if _math.hypot(x2 - x1, y2 - y1) <= 1e-9:
                continue  # round drill: not part of the milled edge
            rr = r + dilate
            a0 = _math.atan2(y2 - y1, x2 - x1)
            pts = []
            for i in range(arc_segments + 1):  # cap around (x2, y2)
                ang = a0 - _math.pi / 2 + _math.pi * i / arc_segments
                pts.append((x2 + rr * _math.cos(ang), y2 + rr * _math.sin(ang)))
            for i in range(arc_segments + 1):  # cap around (x1, y1)
                ang = a0 + _math.pi / 2 + _math.pi * i / arc_segments
                pts.append((x1 + rr * _math.cos(ang), y1 + rr * _math.sin(ang)))
            ref = f"{pad.component_ref}.{pad.pad_number}".rstrip('.')
            # Index into the name: split keyboards repeat footprint refs
            # (two SW25 halves), and rule-area names must be unique.
            out.append((f"{ref}-{len(out)}", pts))
    return out


def generate_keepout_zone_sexpr(layers: List[str],
                                polygon_points: List[Tuple[float, float]],
                                name: str,
                                use_net_name: bool = False) -> str:
    """Rule-area zone blocking only copper POUR (tracks/vias/pads stay
    allowed -- the router enforces its own clearances). Used for the NPTH
    slot edge keepouts (#448). use_net_name=True emits the KiCad 10 net
    header (same switch as generate_zone_sexpr)."""
    pts_str = " ".join(f"(xy {x:.6f} {y:.6f})" for x, y in polygon_points)
    layers_str = " ".join(f'"{l}"' for l in layers)
    net_lines = '(net "")' if use_net_name else '(net 0)\n\t\t(net_name "")'
    return f'''	(zone
		{net_lines}
		(layers {layers_str})
		(uuid "{uuid.uuid4()}")
		(name "{name}")
		(hatch edge 0.5)
		(connect_pads
			(clearance 0)
		)
		(min_thickness 0.25)
		(keepout
			(tracks allowed)
			(vias allowed)
			(pads allowed)
			(copperpour not_allowed)
			(footprints allowed)
		)
		(fill
			(thermal_gap 0.5)
			(thermal_bridge_width 0.5)
		)
		(polygon
			(pts
				{pts_str}
			)
		)
	)'''


def add_tracks_to_pcb(input_path: str, output_path: str, tracks: List[Dict],
                      net_id_to_name: Dict[int, str] = None) -> bool:
    """
    Add track segments to a PCB file.

    Args:
        input_path: Path to original .kicad_pcb file
        output_path: Path for output file
        tracks: List of track dicts with keys: start, end, width, layer, net_id
        net_id_to_name: For KiCad 10, mapping of synthetic net IDs to net names

    Returns:
        True if successful
    """
    with open(input_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Match the input board's net-token format (see add_tracks_and_vias_to_pcb):
    # name-only refs only for name-net boards, numeric (net <id>) for KiCad 9.
    from kicad_parser import board_uses_name_nets, detect_kicad_version, extract_nets
    if board_uses_name_nets(content):
        if net_id_to_name is None:
            nets, _ = extract_nets(content, detect_kicad_version(content))
            net_id_to_name = {nid: n.name for nid, n in nets.items()}
    else:
        net_id_to_name = None

    # Move text from copper layers to silkscreen (prevents routing interference)
    content = move_copper_text_to_silkscreen(content)
    # Move copper logos/artwork to silkscreen too (net-less copper graphics would
    # otherwise short against routed/poured copper).
    content = move_copper_graphics_to_silkscreen(content)
    content = strip_zero_length_edge_cuts(content)

    # Generate segment S-expressions
    segments = []
    for track in tracks:
        track_net_name = net_id_to_name.get(track['net_id']) if net_id_to_name else None
        seg = generate_segment_sexpr(
            track['start'],
            track['end'],
            track['width'],
            track['layer'],
            track['net_id'],
            net_name=track_net_name
        )
        segments.append(seg)

    routing_text = '\n'.join(segments)

    if not routing_text.strip():
        print("Warning: No routing elements to add")
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write(content)
        return True

    # Find the last closing parenthesis
    last_paren = content.rfind(')')

    if last_paren == -1:
        print("Error: Could not find closing parenthesis in PCB file")
        return False

    # Insert routing before the final closing paren
    new_content = content[:last_paren] + '\n' + routing_text + '\n' + content[last_paren:]

    # Write output file
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(new_content)

    return True


def add_tracks_and_vias_to_pcb(input_path: str, output_path: str,
                               tracks: List[Dict], vias: List[Dict] = None,
                               remove_vias: List[Dict] = None,
                               net_id_to_name: Dict[int, str] = None,
                               add_teardrops: bool = False) -> bool:
    """
    Add track segments and vias to a PCB file, optionally removing existing vias.

    Args:
        input_path: Path to original .kicad_pcb file
        output_path: Path for output file
        tracks: List of track dicts with keys: start, end, width, layer, net_id
        vias: List of via dicts with keys: x, y, size, drill, layers, net_id;
            optionally free, and the two protection keys (#749 A):
            tenting_attrs (the via's own parsed spec) and
            inherit_when_unspecified (True for a via that ALREADY EXISTED, so
            one carrying no spec keeps inheriting the board's `(setup ...)`
            rather than gaining a token). A via with neither key is new copper
            and gets the board's prevailing convention. See
            docs/api-kicad-writer.md.
        remove_vias: List of via dicts with keys: x, y (position to match for removal)
        add_teardrops: Add teardrop settings to every pad and via in the output.
            Here rather than in each fanout main() so bga_fanout and qfn_fanout
            share one implementation with the other writers (#489 §9).

    Returns:
        True if successful
    """
    # Seed the output's sibling .kicad_pro before the board exists (#513 item
    # 12): a kill between this write and the post-write floor writeback must
    # not strand the DRC floor (stock-netclass fallback, the #441/#338 class).
    from fix_kicad_drc_settings import seed_project_for_output
    seed_project_for_output(output_path, input_path)
    with open(input_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Keep the output's net-token format consistent with the input board. Only
    # emit name-only (net "name") refs when the board actually uses them; for a
    # KiCad 9 numeric board write (net <id>) and IGNORE any name map the caller
    # passed (the fanout tools pass one unconditionally - issue #80 fix went the
    # other way and injected KiCad-10 name nets into KiCad-9 boards, producing a
    # hybrid file KiCad 9 reads as net-less).
    from kicad_parser import board_uses_name_nets, detect_kicad_version, extract_nets
    if board_uses_name_nets(content):
        if net_id_to_name is None:
            nets, _ = extract_nets(content, detect_kicad_version(content))
            net_id_to_name = {nid: n.name for nid, n in nets.items()}
    else:
        net_id_to_name = None

    # Move text from copper layers to silkscreen (prevents routing interference)
    content = move_copper_text_to_silkscreen(content)
    # Move copper logos/artwork to silkscreen too (net-less copper graphics would
    # otherwise short against routed/poured copper).
    content = move_copper_graphics_to_silkscreen(content)
    content = strip_zero_length_edge_cuts(content)

    # Remove existing vias if specified
    if remove_vias:
        import re

        # #369 A9: match ANY numeric spelling of the coordinate. Our own writer
        # emits %.6f WITH trailing zeros ("104.140000") while the old
        # zero-stripped pattern only matched KiCad-normalized short forms
        # ("104.14"), so a re-run's stale-via removal silently no-op'd on this
        # tool's own previous output (bga_fanout --check-for-previous) and the
        # stale via shipped under the pad.
        def _num_pat(v):
            s = f"{v:.6f}".rstrip('0').rstrip('.')
            if '.' in s:
                return re.escape(s) + '0*'
            return re.escape(s) + r'(?:\.0+)?'

        removed_count = 0
        for via_to_remove in remove_vias:
            x, y = via_to_remove['x'], via_to_remove['y']
            want_net = via_to_remove.get('net_id')
            pattern = re.compile(
                rf'\t\(via\s*\n\s*\(at\s+{_num_pat(x)}\s+{_num_pat(y)}\)[\s\S]*?\n\t\)')
            for m in pattern.finditer(content):
                block = m.group(0)
                # Net gate (was net-blind): never delete a DIFFERENT net's via
                # that happens to sit at the requested position. Soft -- if the
                # block's net can't be determined, keep the old match-by-
                # position behavior.
                if want_net is not None:
                    nm = re.search(r'\(net\s+(\d+)\)', block)
                    if nm is not None and int(nm.group(1)) != want_net:
                        continue
                    if nm is None and net_id_to_name:
                        nn = re.search(r'\(net\s+"((?:[^"\\]|\\.)*)"\)', block)
                        want_name = net_id_to_name.get(want_net)
                        if nn is not None and want_name is not None \
                                and _unescape_kicad_string(nn.group(1)) != want_name:
                            continue
                # Remove exactly this one block: the old re.sub removed EVERY
                # match at the position while counting a single removal.
                content = content[:m.start()] + content[m.end():]
                removed_count += 1
                break
        if removed_count > 0:
            print(f"  Removed {removed_count} vias from file")

    elements = []

    # Generate segment S-expressions
    for track in tracks:
        track_net_name = net_id_to_name.get(track['net_id']) if net_id_to_name else None
        seg = generate_segment_sexpr(
            track['start'],
            track['end'],
            track['width'],
            track['layer'],
            track['net_id'],
            net_name=track_net_name
        )
        elements.append(seg)

    # Generate via S-expressions
    if vias:
        for via in vias:
            # #749 A: this writer had NO tenting_attrs parameter at all, and it
            # also takes remove_vias -- so a via removed and re-added round-trips
            # through here and comes back re-stamped. Which behaviour is right
            # depends on the caller, so the via dict carries the answer:
            #
            #   fanout / route_planes  -> new copper, no keys: prevailing spec
            #   route.py's #666 re-emit -> PRE-EXISTING copper the written file
            #                              lost, so its own spec, and inherit
            #                              when it had none
            #
            # `inherit_when_unspecified` is the second half and not redundant:
            # {} is what Via.tenting_attrs holds for a via that inherits the
            # board's `(setup ...)`, so without it such a via would silently
            # GAIN the prevailing spec on the way back in.
            # A via this call ADDS carries no spec and so emits no token: it
            # INHERITS the board's `(setup ...)` policy, which is what pcbnew
            # does for a via the GUI adds and what KiCad does for one the user
            # places. Copying the board's PREVAILING per-via spec onto it (the
            # #489 s8 rule) is retired -- measured over 886 corpus boards, a
            # prevailing spec never once disagreed with the board's own setup,
            # so it only ever wrote a redundant token that turned an inheriting
            # via into an override. Worse, the tool then read its OWN stamps
            # back as "the board's convention" on the next run.
            existing = bool(via.get('inherit_when_unspecified'))
            attrs = via.get('tenting_attrs')
            v = generate_via_sexpr(
                via['x'],
                via['y'],
                via['size'],
                via['drill'],
                via['layers'],
                via['net_id'],
                via.get('free', False),
                # #749 D: the ONE resolver, so a no-net via does not fall to the
                # numeric dialect on a name-net board (net 0 is missing from
                # every map).
                net_name=via_net_name(via['net_id'], net_id_to_name),
                tenting_attrs=attrs,
                inherit_when_unspecified=existing,
            )
            elements.append(v)

    routing_text = '\n'.join(elements)

    # Teardrops on pads, if asked (#489 §9). Pads are never ADDED here, so this
    # runs on the input text; the VIA pass waits until the new copper is in.
    if add_teardrops:
        print("Adding teardrop settings to pads and vias...")
        content, _td = add_teardrops_to_pads(content)
        print(f"  Added teardrops to {_td} pads" if _td
              else "  All pads already have teardrop settings")

    def _finish(text: str) -> str:
        # Via teardrops LAST so THIS run's vias get them too -- for a fanout that
        # is the whole point (a 0.1mm trace meeting a 0.25mm via pad).
        if not add_teardrops:
            return text
        text, _vtd = add_teardrops_to_vias(text)
        print(f"  Added teardrops to {_vtd} vias" if _vtd
              else "  No vias needed teardrops (none present, or all already set)")
        return text

    if not routing_text.strip():
        print("Warning: No routing elements to add")
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write(_finish(content))
        return True

    # Find the last closing parenthesis
    last_paren = content.rfind(')')

    if last_paren == -1:
        print("Error: Could not find closing parenthesis in PCB file")
        return False

    # Insert routing before the final closing paren
    new_content = content[:last_paren] + '\n' + routing_text + '\n' + content[last_paren:]

    # Write output file
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(_finish(new_content))

    return True


def modify_segment_layers(content: str, segment_mods: List[Dict],
                          unapplied_out: List[Dict] = None) -> Tuple[str, int]:
    """
    Modify the layer of existing segments in the KiCad PCB content.

    Args:
        content: KiCad PCB file content
        segment_mods: List of dicts with keys:
            - start: (x, y) tuple
            - end: (x, y) tuple
            - net_id: int
            - old_layer: str (optional, for verification)
            - new_layer: str

    Returns:
        (modified_content, count_of_modified_segments)
    """
    if not segment_mods:
        return content, 0

    # Build a lookup for segment modifications
    def coord_key(x, y):
        return (round(x, POSITION_DECIMALS), round(y, POSITION_DECIMALS))

    # Per-net lookups: KiCad 9 segments carry a numeric net id, KiCad 10 segments
    # carry the net NAME, so index mods under both. Values are LISTS in append
    # order so chained swaps resolve to the final layer (last mod wins per net).
    mod_lookup_by_id = {}
    mod_lookup_by_name = {}
    # Coordinate-only fallback (net changed between mod recording and writing,
    # e.g. target swaps reassigned the stub's net).
    mod_lookup_by_coords = {}
    for mod in segment_mods:
        # #340 reuse-connector bookkeeping (an added NEW segment, emitted via
        # the all_swap_segments channel) rides in segment_mods for revert only;
        # it is not a file-text layer change, so skip it here.
        if 'start' not in mod:
            continue
        start_key = coord_key(mod['start'][0], mod['start'][1])
        end_key = coord_key(mod['end'][0], mod['end'][1])
        # Also store reverse order since segment endpoints can be swapped
        for ckey in ((start_key, end_key), (end_key, start_key)):
            mod_lookup_by_id.setdefault(ckey + (mod['net_id'],), []).append(mod)
            if mod.get('net_name'):
                mod_lookup_by_name.setdefault(ckey + (mod['net_name'],), []).append(mod)
            mod_lookup_by_coords.setdefault(ckey, []).append(mod)

    count = 0

    # Pattern to match segment blocks - handle both KiCad 9 (net <id>) and
    # KiCad 10 (net "name"). (locked yes) is optional at the same two spots the
    # parser's segment patterns allow it (#150 / #369 A7): without it a LOCKED
    # segment silently never matched, the mod no-op'd, and the in-memory model
    # said the copper moved layers while the file kept it on the old one.
    segment_pattern = re.compile(
        r'(\(segment\s*\n?\s*'
        r'\(start\s+([\d.-]+)\s+([\d.-]+)\)\s*\n?\s*'
        r'\(end\s+([\d.-]+)\s+([\d.-]+)\)\s*\n?\s*'
        r'\(width\s+[\d.]+\)\s*\n?\s*'
        r'(?:\(locked\s+yes\)\s*\n?\s*)?'
        r'\(layer\s+")([^"]+)("\)\s*\n?\s*'
        r'(?:\(locked\s+yes\)\s*\n?\s*)?'
        r'\(net\s+(?:(\d+)|"((?:[^"\\]|\\.)*)")\))',
        re.MULTILINE
    )

    def replace_layer(match):
        nonlocal count
        full_match = match.group(0)
        start_x = float(match.group(2))
        start_y = float(match.group(3))
        end_x = float(match.group(4))
        end_y = float(match.group(5))
        layer = match.group(6)
        net_id = int(match.group(8)) if match.group(8) else None
        # KiCad 10: (net "name") -- unescape the raw file text before the
        # name lookup, or backslash-named nets fall to the coordinate-only
        # fallback (ambiguous on stacked identical-geometry stubs, #264).
        net_name = _unescape_kicad_string(match.group(9)) if match.group(9) else None

        start_key = coord_key(start_x, start_y)
        end_key = coord_key(end_x, end_y)
        coord_only_key = (start_key, end_key)

        # First try an exact per-net match (numeric id for KiCad 9 files, net
        # name for KiCad 10 files). Take the LAST mod so chained swaps land on
        # the final layer. Matching by net matters: two nets can carry
        # identical-geometry stub segments (stacked fanout escapes), and the
        # coordinate-only fallback then applies one net's mod to the other's
        # segment, leaving copper on the wrong layer mid-run (issue #264).
        mods = None
        if net_id is not None:
            mods = mod_lookup_by_id.get(coord_only_key + (net_id,))
        if mods is None and net_name is not None:
            mods = mod_lookup_by_name.get(coord_only_key + (net_name,))
        # Prefer a mod whose old_layer IS this block's current layer. A mod
        # says "the copper on old_layer moves to new_layer", so old_layer is
        # part of its identity -- taking mods[-1] blindly mis-assigns when ONE
        # net owns twin segments of identical geometry on different layers
        # (the #264 ambiguity, which is not limited to twins of DIFFERENT
        # nets). cynthion shipped the failure this guards: VCCRAM's stub
        # (138.425,92.1)-(138.5,92.1) stayed In1.Cu in the file while pcb_data
        # moved it to In3.Cu, leaving a 0.075mm In1 sliver between two In3
        # segments with NO via -- a broken track plus stray copper.
        mod = None
        chain = []
        if mods and any(m.get('old_layer') for m in mods):
            # FOLLOW THE CHAIN from THIS block's current layer: repeatedly take
            # the mod whose old_layer is where the copper currently sits. That
            # satisfies both cases at once -- chained swaps on one segment
            # (In1->In2->B lands on B) and twin disambiguation (a twin already
            # on In3 has no In3-rooted mod, so it is left alone).
            cur = layer
            seen = {layer}
            while True:
                cands = [m for m in mods if m.get('old_layer') == cur]
                if not cands:
                    break
                step = cands[-1]
                chain.append(step)
                nxt = step['new_layer']
                if nxt in seen:      # cycle guard (a true A<->B swap)
                    cur = nxt
                    break
                seen.add(nxt)
                cur = nxt
            if chain:
                mod = dict(chain[-1])
                mod['new_layer'] = cur
        elif mods:
            # No old_layer recorded anywhere -> legacy "last mod wins".
            mod = mods[-1]
        # Fallback: coordinate match with the net left unmatched (the net was
        # reassigned between recording and writing, e.g. by target swaps). Only
        # accept mods whose old_layer equals THIS segment's current layer, so a
        # twin segment of another net (different layer) never steals the mod.
        if mod is None and coord_only_key in mod_lookup_by_coords:
            cands = [m for m in mod_lookup_by_coords[coord_only_key]
                     if m.get('old_layer') == layer]
            mod = cands[-1] if cands else None

        if mod:
            new_layer = mod['new_layer']
            # Credit every mod the chain consumed (mod itself may be a
            # synthesized end-of-chain dict). Satisfied either way: a layer
            # that already reads correctly needs no edit.
            for _m in (chain or [mod]):
                satisfied.add(id(_m))
            if layer != new_layer:
                count += 1
                # Replace the layer in the match
                return full_match.replace(f'(layer "{layer}")', f'(layer "{new_layer}")')
        return full_match

    satisfied = set()
    result = segment_pattern.sub(replace_layer, content)
    # A mod that matched NO block in the file is the dangerous case: the
    # in-memory board already moved the copper (stub_layer_switching mutates
    # seg.layer at decision time), so a silent miss ships a file that
    # disagrees with pcb_data -- exactly the break cynthion shipped. Surface
    # it; the FILE_LEDGER only catches it when that audit is enabled.
    if unapplied_out is not None:
        unapplied_out.extend(m for m in segment_mods
                             if 'start' in m and id(m) not in satisfied)
    return result, count


def swap_segment_nets_at_positions(content: str, positions: set,
                                   old_net_id: int, new_net_id: int,
                                   layer: str = None,
                                   old_net_name: str = None,
                                   new_net_name: str = None) -> Tuple[str, int]:
    """
    Swap net IDs of segments that have endpoints at the given positions.

    Args:
        content: KiCad file content
        positions: Set of position keys to match
        old_net_id: Net ID to replace
        new_net_id: Net ID to replace with
        layer: Optional layer name to filter segments. When specified, only segments
               on this layer are swapped. This prevents incorrectly swapping
               stubs that share XY coordinates but are on different layers.
        old_net_name: For KiCad 10, the net name to match/replace
        new_net_name: For KiCad 10, the replacement net name

    Returns (modified_content, count_of_swapped_segments).
    """
    use_names = old_net_name is not None and new_net_name is not None

    if use_names:
        # KiCad 10: match (net "name")
        segment_pattern = r'\(segment\s+\(start\s+([\d.-]+)\s+([\d.-]+)\)\s+\(end\s+([\d.-]+)\s+([\d.-]+)\)\s+\(width[^)]*\)\s+\(layer\s+"?([^")]+)"?\).*?\(net\s+"((?:[^"\\]|\\.)*)"\)'
    else:
        # KiCad 9: match (net <id>)
        segment_pattern = r'\(segment\s+\(start\s+([\d.-]+)\s+([\d.-]+)\)\s+\(end\s+([\d.-]+)\s+([\d.-]+)\)\s+\(width[^)]*\)\s+\(layer\s+"?([^")]+)"?\).*?\(net\s+(\d+)\)'

    count = 0

    def replace_net(match):
        nonlocal count
        start_x, start_y = float(match.group(1)), float(match.group(2))
        end_x, end_y = float(match.group(3)), float(match.group(4))
        seg_layer = match.group(5)

        start_key = pos_key(start_x, start_y)
        end_key = pos_key(end_x, end_y)

        # Check if this segment has endpoints in our position set and correct net
        if use_names:
            # group(6) is the RAW escaped file text; the caller's names are the
            # parser's unescaped display names (#312/#264 escaping family --
            # without this, backslash-named nets silently evade the swap).
            raw_net = match.group(6)
            matches_net = _unescape_kicad_string(raw_net) == old_net_name
        else:
            seg_net_id = int(match.group(6))
            matches_net = seg_net_id == old_net_id

        if matches_net and (start_key in positions or end_key in positions):
            if layer is None or seg_layer == layer:
                count += 1
                if use_names:
                    # Replace the raw token as it appears in the file; write the
                    # new name RE-ESCAPED or a backslash name ships under-escaped.
                    return match.group(0).replace(
                        f'(net "{raw_net}")',
                        f'(net "{_escape_net_name(new_net_name)}")')
                else:
                    return match.group(0).replace(f'(net {old_net_id})', f'(net {new_net_id})')
        return match.group(0)

    result = re.sub(segment_pattern, replace_net, content, flags=re.DOTALL)
    return result, count


def assert_balanced_sexpr(text: str, label: str = 'board'):
    """#523 backstop: refuse to write a structurally broken board.

    Quote-aware paren balance of the full text must be exactly zero; a text
    transform that ate or duplicated a paren otherwise ships a file KiCad
    cannot load while check_drc/check_connected (text parsers) grade it
    normally -- the worst kind of corruption, invisible to our own checkers.
    Raises ValueError; callers must NOT catch-and-continue.
    """
    depth = 0
    in_str = False
    esc = False
    for ch in text:
        if in_str:
            if esc:
                esc = False
            elif ch == '\\':
                esc = True
            elif ch == '"':
                in_str = False
            continue
        if ch == '"':
            in_str = True
        elif ch == '(':
            depth += 1
        elif ch == ')':
            depth -= 1
    if depth != 0 or in_str:
        raise ValueError(
            f"{label}: refusing to write structurally broken s-expression "
            f"(paren depth {depth}, in_string={in_str}) -- a text transform "
            f"corrupted the board (#523)")


def remove_segments_from_content(content: str, segments: List,
                                 net_id_to_name: Dict = None,
                                 unmatched_out: List = None) -> Tuple[str, int]:
    """Delete whole ``(segment ...)`` blocks matching the given segments.

    Used by the dead-end sweep (issue #84) to strip original input-file copper
    that prune_dead_end_segments flagged as a dead end -- copper the writer would
    otherwise carry into the output verbatim. A segment is matched by its
    unordered endpoint pair plus layer and net, which uniquely identifies it
    (a duplicate with the same endpoints/layer/net is interchangeable). The whole
    block (including its trailing uuid) is removed, not just the net token.

    Returns ``(modified_content, count_removed)``.
    """
    if not segments:
        return content, 0

    # Net tokens are resolved PER BLOCK, in whichever dialect that block
    # uses -- never from a file-level gate. The #369 A1 gate (match by the
    # format the file "uses") still assumed a HOMOGENEOUS file; a MIXED-
    # dialect board -- which this repo's own chain produces (#748/#749:
    # segments `(net 6)` beside name-ref pads) -- reads as name-style, so
    # every numeric segment block silently evaded the strip. Measured on
    # the #622 bench: a force-rerouted net's stale input diagonal shipped
    # verbatim and CROSSED the net routed into its vacated corridor (a
    # hard short at (128.6, 64.2), the route.py same-call-shorts class).
    # Both a numeric and a name ref now canonicalize to the same key.
    def _canon(nid):
        return net_id_to_name.get(nid, nid) if net_id_to_name else nid

    def seg_key(p1, p2, layer, net_token):
        return (frozenset((p1, p2)), layer, net_token)

    # COUNTED multiset, not a set (#318 follow-up): a board can legitimately
    # carry N identical-span segments with only K < N of them strip-listed
    # (sechzig /PWR1V35: two byte-identical slivers, cleanup removed one). A
    # set-based match deleted EVERY block with the key, including the keeper.
    from collections import Counter
    targets = Counter()
    by_key = {}
    for s in segments:
        k = seg_key(pos_key(s.start_x, s.start_y),
                    pos_key(s.end_x, s.end_y), s.layer, _canon(s.net_id))
        targets[k] += 1
        by_key.setdefault(k, []).append(s)

    start_re = re.compile(r'\(start\s+([\d.-]+)\s+([\d.-]+)\)')
    end_re = re.compile(r'\(end\s+([\d.-]+)\s+([\d.-]+)\)')
    layer_re = re.compile(r'\(layer\s+"?([^")]+)"?\)')
    net_name_re = re.compile(r'\(net\s+"((?:[^"\\]|\\.)*)"\)')
    net_id_re = re.compile(r'\(net\s+(\d+)\)')

    # Scan top-level (segment ...) blocks with a quote-aware paren matcher. A
    # plain nested-paren regex breaks on KiCad v10 auto-net names that contain
    # parentheses, e.g. (net "Net-(R2-Pad1)") -- the inner parens are inside a
    # string and must not count toward block nesting.
    out = []
    pos = 0
    n = len(content)
    count = 0
    while True:
        j = content.find('(segment', pos)
        if j < 0:
            out.append(content[pos:])
            break
        out.append(content[pos:j])
        depth = 0
        in_str = False
        k = j
        while k < n:
            ch = content[k]
            if in_str:
                if ch == '"':
                    in_str = False
            elif ch == '"':
                in_str = True
            elif ch == '(':
                depth += 1
            elif ch == ')':
                depth -= 1
                if depth == 0:
                    k += 1
                    break
            k += 1
        block = content[j:k]
        ms, me, ml = start_re.search(block), end_re.search(block), layer_re.search(block)
        keep = True
        if ms and me and ml:
            mn = net_name_re.search(block)
            if mn:
                # The file stores the ESCAPED name (backslash doubled, quote
                # backslashed); targets use the parser's unescaped name. Undo
                # the escapes or every backslash-named net silently evades the
                # strip and its stale copper ships (neo6502 /GPIO*\* nets,
                # found by the FILE_LEDGER audit -- #312/#264 family).
                net_token = _unescape_kicad_string(mn.group(1))
            else:
                mi = net_id_re.search(block)
                net_token = _canon(int(mi.group(1))) if mi else None
            key = seg_key(pos_key(float(ms.group(1)), float(ms.group(2))),
                          pos_key(float(me.group(1)), float(me.group(2))),
                          ml.group(1), net_token)
            if targets.get(key, 0) > 0:
                targets[key] -= 1
                keep = False
                count += 1
        if keep:
            out.append(block)
        pos = k
    if unmatched_out is not None:
        # Strip targets whose block was NOT found (e.g. its layer/net token in
        # the text only matches after the writer's later transforms). The
        # residual post-transform pass retries EXACTLY these -- passing the
        # full list again would let round 2 consume a same-key KEEPER block
        # that round 1 legitimately left (the counted-multiset guarantee).
        for k, n in targets.items():
            if n > 0:
                unmatched_out.extend(by_key[k][:n])
    return ''.join(out), count


def remove_vias_from_content(content: str, vias: List,
                             net_id_to_name: Dict = None,
                             unmatched_out: List = None) -> Tuple[str, int]:
    """Delete whole ``(via ...)`` blocks matching the given vias.

    Via twin of remove_segments_from_content: strips original input-file vias
    of a ripped/re-routed net that are no longer on the final board (#103 rip-
    existing; without this the old via AND its reroute's replacement both ship,
    stacking same-net drill pairs). Matched by position + net token; duplicates
    are interchangeable. Returns ``(modified_content, count_removed)``.
    """
    if not vias:
        return content, 0

    # Net tokens resolved PER BLOCK in either dialect -- see the segment
    # twin above for the mixed-dialect hole this closes (#748/#749 boards:
    # numeric refs on copper beside name refs elsewhere read as name-style,
    # so every numeric block evaded the strip).
    def _canon(nid):
        return net_id_to_name.get(nid, nid) if net_id_to_name else nid

    # Counted multiset like the segment strip (#318 follow-up): only remove as
    # many blocks per key as were actually strip-listed.
    from collections import Counter
    targets = Counter()
    _v_by_key = {}
    for v in vias:
        _vk = (pos_key(v.x, v.y), _canon(v.net_id))
        targets[_vk] += 1
        _v_by_key.setdefault(_vk, []).append(v)

    at_re = re.compile(r'\(at\s+([\d.-]+)\s+([\d.-]+)\)')
    net_name_re = re.compile(r'\(net\s+"((?:[^"\\]|\\.)*)"\)')
    net_id_re = re.compile(r'\(net\s+(\d+)\)')

    out = []
    pos = 0
    n = len(content)
    count = 0
    while True:
        j = content.find('(via', pos)
        # skip non-via tokens sharing the prefix: (vias allowed) in rule areas
        while j >= 0 and j + 4 < n and content[j + 4] not in ' \t\r\n(':
            j = content.find('(via', j + 4)
        if j < 0:
            out.append(content[pos:])
            break
        out.append(content[pos:j])
        depth = 0
        in_str = False
        k = j
        while k < n:
            ch = content[k]
            if in_str:
                if ch == '"':
                    in_str = False
            elif ch == '"':
                in_str = True
            elif ch == '(':
                depth += 1
            elif ch == ')':
                depth -= 1
                if depth == 0:
                    k += 1
                    break
            k += 1
        block = content[j:k]
        ma = at_re.search(block)
        keep = True
        if ma:
            mn = net_name_re.search(block)
            if mn:
                # The file stores the ESCAPED name (backslash doubled, quote
                # backslashed); targets use the parser's unescaped name. Undo
                # the escapes or every backslash-named net silently evades the
                # strip and its stale copper ships (neo6502 /GPIO*\* nets,
                # found by the FILE_LEDGER audit -- #312/#264 family).
                net_token = _unescape_kicad_string(mn.group(1))
            else:
                mi = net_id_re.search(block)
                net_token = _canon(int(mi.group(1))) if mi else None
            _vk = (pos_key(float(ma.group(1)), float(ma.group(2))), net_token)
            if targets.get(_vk, 0) > 0:
                targets[_vk] -= 1
                keep = False
                count += 1
        if keep:
            out.append(block)
        pos = k
    if unmatched_out is not None:
        for _vk, n in targets.items():
            if n > 0:
                unmatched_out.extend(_v_by_key[_vk][:n])
    return ''.join(out), count


def swap_via_nets_at_positions(content: str, positions: set,
                               old_net_id: int, new_net_id: int,
                               tolerance: float = 0.02,
                               old_net_name: str = None,
                               new_net_name: str = None) -> Tuple[str, int]:
    """
    Swap net IDs of vias that are at the given positions.

    Uses tolerance-based matching to handle slight coordinate differences
    between vias and segment endpoints (e.g., original vias may be slightly
    off-grid from routed segments).

    Args:
        old_net_name: For KiCad 10, the net name to match/replace
        new_net_name: For KiCad 10, the replacement net name

    Returns (modified_content, count_of_swapped_vias).
    """
    use_names = old_net_name is not None and new_net_name is not None

    if use_names:
        via_pattern = r'\(via\s+\(at\s+([\d.-]+)\s+([\d.-]+)\).*?\(net\s+"((?:[^"\\]|\\.)*)"\)'
    else:
        via_pattern = r'\(via\s+\(at\s+([\d.-]+)\s+([\d.-]+)\).*?\(net\s+(\d+)\)'

    count = 0

    def is_near_any_position(x, y, positions, tol):
        """Check if (x, y) is within tolerance of any position in the set."""
        for px, py in positions:
            if abs(x - px) < tol and abs(y - py) < tol:
                return True
        return False

    def replace_net(match):
        nonlocal count
        via_x, via_y = float(match.group(1)), float(match.group(2))

        if use_names:
            # Raw escaped file text vs unescaped display name (see the
            # segment twin above).
            raw_net = match.group(3)
            matches_net = _unescape_kicad_string(raw_net) == old_net_name
        else:
            via_net_id = int(match.group(3))
            matches_net = via_net_id == old_net_id

        if matches_net and is_near_any_position(via_x, via_y, positions, tolerance):
            count += 1
            if use_names:
                return match.group(0).replace(
                    f'(net "{raw_net}")',
                    f'(net "{_escape_net_name(new_net_name)}")')
            else:
                return match.group(0).replace(f'(net {old_net_id})', f'(net {new_net_id})')
        return match.group(0)

    result = re.sub(via_pattern, replace_net, content, flags=re.DOTALL)
    return result, count


def add_teardrops_to_pads(content: str,
                          best_length_ratio: float = 0.5,
                          max_length: float = 1.0,
                          best_width_ratio: float = 1.0,
                          max_width: float = 2.0,
                          curved_edges: bool = False,
                          filter_ratio: float = 0.9,
                          allow_two_segments: bool = True,
                          prefer_zone_connections: bool = True) -> tuple[str, int]:
    """
    Add teardrop settings to all pads that don't already have them.

    Teardrops are added before the (uuid ...) line in each pad definition.

    Args:
        content: KiCad PCB file content
        best_length_ratio: Target length as ratio of track width (default 0.5)
        max_length: Maximum teardrop length in mm (default 1.0)
        best_width_ratio: Target width as ratio of pad size (default 1.0)
        max_width: Maximum teardrop width in mm (default 2.0)
        curved_edges: Use curved teardrop edges (default False)
        filter_ratio: Min ratio of teardrop to pad size to apply (default 0.9)
        allow_two_segments: Allow teardrops with two segments (default True)
        prefer_zone_connections: Prefer zone connections over teardrops (default True)

    Returns:
        (modified_content, count_of_pads_with_teardrops_added)
    """
    curved_str = "yes" if curved_edges else "no"
    two_seg_str = "yes" if allow_two_segments else "no"
    zone_str = "yes" if prefer_zone_connections else "no"

    teardrop_block = f'''(teardrops
				(best_length_ratio {best_length_ratio})
				(max_length {max_length})
				(best_width_ratio {best_width_ratio})
				(max_width {max_width})
				(curved_edges {curved_str})
				(filter_ratio {filter_ratio})
				(enabled yes)
				(allow_two_segments {two_seg_str})
				(prefer_zone_connections {zone_str})
			)
			'''

    count = 0
    result_parts = []
    last_end = 0
    i = 0

    while True:
        # Find next pad definition
        pad_start = content.find('(pad ', i)
        if pad_start == -1:
            break

        # Find the end of this pad block by counting parens
        depth = 0
        pad_end = pad_start
        for j in range(pad_start, len(content)):
            if content[j] == '(':
                depth += 1
            elif content[j] == ')':
                depth -= 1
                if depth == 0:
                    pad_end = j + 1
                    break

        pad_block = content[pad_start:pad_end]

        # Check if this pad already has teardrops
        if '(teardrops' not in pad_block:
            # Find the (uuid line to insert before it
            uuid_pos = pad_block.find('(uuid ')
            if uuid_pos != -1:
                # Insert teardrop block before uuid
                new_pad_block = pad_block[:uuid_pos] + teardrop_block + pad_block[uuid_pos:]
                result_parts.append(content[last_end:pad_start])
                result_parts.append(new_pad_block)
                last_end = pad_end
                count += 1

        i = pad_end

    # Add remaining content
    result_parts.append(content[last_end:])

    return ''.join(result_parts), count


def add_teardrops_to_vias(content: str,
                          best_length_ratio: float = 0.5,
                          max_length: float = 1.0,
                          best_width_ratio: float = 1.0,
                          max_width: float = 2.0,
                          curved_edges: bool = False,
                          filter_ratio: float = 0.9,
                          allow_two_segments: bool = True,
                          prefer_zone_connections: bool = True) -> tuple[str, int]:
    """Add teardrop settings to every via that does not already have them.

    `--add-teardrops` reached PADS only: `add_teardrops_to_pads` above, with zero
    teardrop references anywhere near via emission, so vias got teardrops on NO
    path (#489 §9). Track-to-via teardrops matter most exactly where this tool is
    weakest -- fine-pitch BGA escape, where a 0.1mm trace meets a 0.25mm via pad --
    and they are the standard mitigation for drill breakout under layer-to-layer
    registration error.

    KiCad accepts the same 9-field `(teardrops ...)` block on a via as on a pad
    (verified by round-tripping SetTeardropsEnabled through pcbnew). The block is
    inserted AFTER `(uuid ...)`, as the via's last child: KiCad's parser does not
    care about child order, while this repo's own via regexes require
    layers -> (free)? -> net -> uuid to be contiguous, so inserting earlier would
    make the boards we write unparseable BY US.

    Returns (modified_content, count_of_vias_given_teardrops).
    """
    curved_str = "yes" if curved_edges else "no"
    two_seg_str = "yes" if allow_two_segments else "no"
    zone_str = "yes" if prefer_zone_connections else "no"

    teardrop_block = f'''
		(teardrops
			(best_length_ratio {best_length_ratio})
			(max_length {max_length})
			(best_width_ratio {best_width_ratio})
			(max_width {max_width})
			(curved_edges {curved_str})
			(filter_ratio {filter_ratio})
			(enabled yes)
			(allow_two_segments {two_seg_str})
			(prefer_zone_connections {zone_str})
		)'''

    count = 0
    result_parts = []
    last_end = 0
    i = 0

    while True:
        via_start = content.find('(via', i)
        if via_start == -1:
            break
        # '(via' must be the whole token, not a prefix of something else.
        nxt = content[via_start + 4:via_start + 5]
        if nxt and nxt not in ' \t\r\n(':
            i = via_start + 4
            continue

        depth = 0
        via_end = via_start
        for j in range(via_start, len(content)):
            if content[j] == '(':
                depth += 1
            elif content[j] == ')':
                depth -= 1
                if depth == 0:
                    via_end = j + 1
                    break

        via_block = content[via_start:via_end]

        if '(teardrops' not in via_block:
            uuid_match = re.search(r'\(uuid\s+"[^"]*"\)', via_block)
            if uuid_match:
                cut = uuid_match.end()
                new_via_block = via_block[:cut] + teardrop_block + via_block[cut:]
                result_parts.append(content[last_end:via_start])
                result_parts.append(new_via_block)
                last_end = via_end
                count += 1

        i = via_end

    result_parts.append(content[last_end:])
    return ''.join(result_parts), count


def swap_pad_nets_in_content(content: str, pad1: Pad, pad2: Pad) -> str:
    """
    Swap the net assignments of two pads in the KiCad PCB file content.

    Finds each pad by its component reference and pad number, then swaps their (net ...) declarations.
    """
    def find_pad_net_spans(content: str, component_ref: str, pad_number: str) -> List[Tuple[int, int, str]]:
        """(start, end, match_text) of EVERY matching pad's (net ...) token.

        #369 A8: returns ALL same-number pads, not just the first -- connector
        shields / split EP paddles legally repeat one pad number, and swapping
        only the first left the twins on the old net (half-applied swap =
        short/open at the target).

        #726: the footprint is located by the PARSER's key rather than by its
        Reference string, so the KiCad 6/7 `(fp_text reference ...)` form is
        handled for free and a duplicated reference resolves to the one block
        the pad actually belongs to. Bailing here AFTER the segment/via
        relabels have applied ships mixed-net copper, so a silent miss is the
        expensive failure."""
        from kicad_parser import iter_footprint_blocks
        spans = []

        # Blocks are resolved by the PARSER's key (#726). `component_ref` comes
        # off a `kicad_parser.Pad`, so on a board with two blocks named `TP4`
        # it is `TP4~2` -- a name the old Reference-string match could never
        # find, which made this a SILENT no-op after the segment/via relabels
        # had already applied. Matching the string was equally wrong the other
        # way: it took the first block carrying the name, which is a different
        # physical part than the one the pad belongs to.
        for fp_start, fp_end, fp_text, _raw_ref, key in iter_footprint_blocks(
                content):
            if key != component_ref:
                continue

            # Find every pad with this number in this footprint
            pad_pattern = rf'\(pad\s+"{re.escape(pad_number)}"\s+\w+\s+\w+'
            for pad_match in re.finditer(pad_pattern, fp_text):
                pad_start_rel = pad_match.start()
                # Find end of this pad block
                depth = 0
                pad_end_rel = pad_start_rel
                for i, char in enumerate(fp_text[pad_start_rel:], pad_start_rel):
                    if char == '(':
                        depth += 1
                    elif char == ')':
                        depth -= 1
                        if depth == 0:
                            pad_end_rel = i + 1
                            break

                pad_text = fp_text[pad_start_rel:pad_end_rel]

                # Find the (net id "name") or (net "name") in this pad
                net_match = re.search(r'\(net\s+\d+\s+"(?:[^"\\]|\\.)*"\)', pad_text)
                if not net_match:
                    # KiCad 10: (net "name") with no numeric ID
                    net_match = re.search(r'\(net\s+"(?:[^"\\]|\\.)*"\)', pad_text)
                if net_match:
                    # Convert to absolute positions in content
                    abs_start = fp_start + pad_start_rel + net_match.start()
                    abs_end = fp_start + pad_start_rel + net_match.end()
                    spans.append((abs_start, abs_end, net_match.group()))
            if spans:
                return spans  # the right footprint was found; done

        return spans

    # Find both pads' net declarations (all same-number twins included)
    spans1 = find_pad_net_spans(content, pad1.component_ref, pad1.pad_number)
    spans2 = find_pad_net_spans(content, pad2.component_ref, pad2.pad_number)

    if not spans1 or not spans2:
        print(f"  WARNING: Could not find pad(s) to swap nets")
        if not spans1:
            print(f"    Missing: {pad1.component_ref} pad {pad1.pad_number}")
        if not spans2:
            print(f"    Missing: {pad2.component_ref} pad {pad2.pad_number}")
        return content

    net1_text = spans1[0][2]
    net2_text = spans2[0][2]

    # Swap every span, highest file position first so indices stay valid
    repls = ([(s, e, net2_text) for s, e, _ in spans1]
             + [(s, e, net1_text) for s, e, _ in spans2])
    for s, e, txt in sorted(repls, key=lambda r: r[0], reverse=True):
        content = content[:s] + txt + content[e:]

    return content
