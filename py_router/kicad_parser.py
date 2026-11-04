"""
KiCad PCB Parser - Extracts pads, nets, tracks, vias, and board info from .kicad_pcb files.
"""
from __future__ import annotations

#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': [], 'kind': 'instrument'}

import functools
from array import array
import os
import re
import math
import json
import sys
import routing_defaults as defaults  # fab-floor outline width for 0-stroke copper polys (#337/M2)
from swig_compat import patch_swig_iterators as _patch_swig_iterators
# #962: the paste-stencil model. A leaf module (it imports check_drc/this module
# only inside its functions), so importing it here cannot cycle.
from paste_apertures import PasteAperture, build_paste_apertures, pad_has_copper
from dataclasses import dataclass, field
from typing import Dict, List, Sequence, Tuple, Optional
from pathlib import Path

# #795: KiCad's own `__iter__` for GetTracks()/GetDrawings() calls the py2
# `SwigPyIterator.next()`, which a current SWIG no longer supplies. Restore the
# alias before anything here walks a live board. No-op off a KiCad python.
_patch_swig_iterators()


# Position rounding precision for coordinate COMPARISONS (dedup keys, position
# lookups) -- never for geometry actually written to a board. Rounding applied
# copper to 1 um was #493 item 5; use mm_to_iu() below to emit geometry.
POSITION_DECIMALS = 3

# KiCad's internal unit for PCBs is the integer nanometre.
IU_PER_MM = 1000000


def mm_to_iu(mm):
    """mm -> KiCad internal units (integer nanometres), correctly rounded.

    Use this instead of `pcbnew.FromMM` wherever geometry is written to a board.
    Two separate bugs made GUI-applied copper differ from the identical route
    written by the CLI text writer (#493 item 5):

    1. The GUI rounded positions to POSITION_DECIMALS (1 um) before converting,
       so any point not already on a 1 um boundary -- diagonal joins, nudged
       vias -- moved by up to 0.5 um. It rounded via SIZE and DRILL the same
       way, the same class of bug as the #362 track-width one (round(w,3) turns
       a 0.0762 fab-floor width into 0.076).
    2. `pcbnew.FromMM` TRUNCATES rather than rounds: 66.1 * 1e6 is
       66099999.99999999 in binary, so FromMM(66.1) == 66099999 -- one nm short.
       The CLI writes "66.1" as text and KiCad parses it back to 66100000, so
       the two fronts stored genuinely different integers for the same point.

    Rounding the scaled value fixes both, and nm is the board's real resolution,
    so nothing is lost.
    """
    return int(round(float(mm) * IU_PER_MM))


def canonicalize_pcb_data_order(pcb_data):
    """Sort pcb_data.segments and .vias into a canonical GEOMETRIC order,
    in place. Returns True if anything moved.

    The GUI adds copper to a live pcbnew board and the CLI writer emits it from
    its own lists, so the two fronts hold IDENTICAL copper in DIFFERENT order --
    measured repeatedly on eth_tap: same 1159-segment set, three different
    orders across two GUI chains and the CLI. Nothing downstream should depend
    on that, but list position keeps leaking into decisions: representative
    endpoint selection, connected-group ordering, stub free ends, MST terminal
    labelling, unit distances. Each was fixed individually and another surfaced.

    Canonicalising the order ONCE, at the engine entry, closes the whole class
    instead of one consumer at a time. The key is pure geometry (layer, then the
    unordered endpoint pair, then width / size), so it is identical on both
    fronts and stable across runs.

    This does NOT reorder anything the router later appends -- it only fixes the
    starting state, which is what the two fronts disagree about.
    """
    def _segkey(s):
        a = (s.start_x, s.start_y)
        b = (s.end_x, s.end_y)
        return (s.layer, s.net_id, min(a, b), max(a, b), s.width)

    def _viakey(v):
        return (v.x, v.y, v.net_id, getattr(v, 'size', 0.0) or 0.0,
                getattr(v, 'drill', 0.0) or 0.0)

    moved = False
    try:
        segs = getattr(pcb_data, 'segments', None)
        if segs:
            new = sorted(segs, key=_segkey)
            if any(x is not y for x, y in zip(new, segs)):
                moved = True
            pcb_data.segments[:] = new
        vias = getattr(pcb_data, 'vias', None)
        if vias:
            new = sorted(vias, key=_viakey)
            if any(x is not y for x, y in zip(new, vias)):
                moved = True
            pcb_data.vias[:] = new
    except Exception as e:
        print(f"(pcb_data order canonicalisation skipped: {e})")
    return moved


def snap_pcb_data_to_iu_grid(pcb_data):
    """Snap every routed-copper coordinate in ``pcb_data`` onto KiCad's integer
    nanometre grid, IN PLACE. Returns the number of values changed.

    The router works in float mm, so ~20% of the copper it commits lands OFF the
    nm grid by up to ~0.49 nm -- values a board cannot actually store. The writer
    quantises on the way out, so the FILE is always 100% grid-exact while the
    in-memory ``pcb_data`` is not. Same board, different bits.

    That mismatch forked GUI from CLI. ``batch_route``'s final-reconcile pass
    retries incomplete nets against "the finished board", and each front had a
    different idea of what that board is: the CLI re-parses the WRITTEN file
    (grid-exact), the GUI re-invokes on the in-memory ``pcb_data`` (not). So
    ``return_results`` -- a flag that should only decide whether results are
    RETURNED -- changed what got ROUTED. Measured on eth_tap step 11, same board
    and same kwargs: reconcile ON gave CLI 3423 vs GUI 3428 segments; reconcile
    OFF gave 3423 vs 3423, bit-identical.

    Snapping is geometrically a no-op: the largest deviation measured is 0.4878
    nm, below the 0.5 nm rounding threshold, so every value lands on the SAME
    integer nm the writer would have emitted. Fields cover exactly what the
    writer canonicalises (verified grid-exact in written output): segment
    start/end and width, via position, size and drill.
    """
    changed = 0

    def _snap(obj, attr):
        nonlocal changed
        v = getattr(obj, attr, None)
        if isinstance(v, float):
            q = mm_to_iu(v) / IU_PER_MM
            if q != v:
                setattr(obj, attr, q)
                changed += 1

    for s in getattr(pcb_data, 'segments', None) or ():
        for a in ('start_x', 'start_y', 'end_x', 'end_y', 'width'):
            _snap(s, a)
    for v in getattr(pcb_data, 'vias', None) or ():
        for a in ('x', 'y', 'size', 'drill'):
            _snap(v, a)
    return changed


# KiCad 10 removed numeric net IDs from the file format.
# Files with version >= this threshold use name-only nets: (net "name") instead of (net 29 "name").
# KiCad 9 uses version 20241229; KiCad 10 uses version 20260206.
KICAD_10_MIN_VERSION = 20250000


def detect_kicad_version(content: str) -> int:
    """Extract version number from (version YYYYMMDD) header."""
    m = re.search(r'\(version\s+(\d+)\)', content)
    return int(m.group(1)) if m else 0


# ---------------------------------------------------------------------------
# KiCad 6-era file conventions pcbnew converts on load. Each cutoff is KiCad's
# own (pcb_io_kicad_sexpr_parser.cpp / pcb_io_kicad_sexpr.h at 10.0.0); the
# text parser applies the same conversion so a KiCad 6 board parses the way
# the GUI -- which builds from pcbnew -- already sees it. Found by parsing all
# 89 KiCad 6 corpus sources raw and after a pcbnew round trip.
# ---------------------------------------------------------------------------

#: Files at or before this version wrote arcs as (start CENTER) (end ARC-START)
#: (angle SWEEP) -- LEGACY_ARC_FORMATTING.
LEGACY_ARC_FORMATTING = 20210925
#: Files at or before this version declared a net tie by footprint keywords
#: (tags "net tie ...") rather than (net_tie_pad_groups ...) -- LEGACY_NET_TIES.
LEGACY_NET_TIES = 20220815
#: Files before this version wrote overbars as ~TEXT~ rather than ~{TEXT}.
NEW_OVERBAR_NOTATION = 20210606
#: Up to this version a footprint property is a bare name/value pair, and
#: pcbnew's reader takes `(property "Sheetname" ...)` / `(property "Sheet
#: name" ...)` (and the file pair) as the footprint's SHEET; from the next
#: version on a property is a field and those names are ordinary user fields
#: (probed on pcbnew 10.0.0: 20230619 reads them, 20230620 does not).
SHEET_PROPERTIES_MAX_VERSION = 20230619

#: The first board format with (footprint ...) blocks ("module -> footprint",
#: KiCad 5.99). An older file -- KiCad 5, KiCad 4 -- writes (module ...) and
#: bare net names, and this parser would read it as a board with no footprints
#: and no nets: it is REFUSED instead (UnsupportedBoardFormat).
FIRST_SUPPORTED_BOARD_VERSION = 20201115


class UnsupportedBoardFormat(ValueError):
    """A board file this parser cannot read. The message says what the file is
    and what to do, so a CLI prints it as a clean ERROR line rather than as a
    traceback (``user_facing``, honoured by cli_banner's excepthook)."""
    user_facing = True


_TSTAMP_RE = re.compile(r'\(tstamp\s+"?([0-9A-Fa-f-]+)"?\)')


def kiid_from_tstamp(raw: str) -> str:
    """The uuid pcbnew gives a KiCad 6 ``(tstamp ...)`` (kiid.cpp).

    A value of at most 8 hex digits is a LEGACY timestamp: only the last four
    octets are filled, from the END of the string. Anything else is already a
    uuid and is kept as written.
    """
    s = raw.strip('"')
    if s and len(s) <= 8 and all(c in '0123456789abcdefABCDEF' for c in s):
        octets = []
        for i in range(4):
            start = len(s) - 8 + i * 2
            end = start + 2
            start = max(0, start)
            o = s[start:start + max(0, end - start)]
            octets.append(int(o, 16) if o else 0)
        return '00000000-0000-0000-0000-0000' + ''.join('%02x' % o for o in octets)
    return s


def uuid_or_tstamp(text: str) -> str:
    """The first ``(uuid "...")`` in TEXT, else its KiCad 6 ``(tstamp ...)``
    as pcbnew converts it, else ''. Callers bound TEXT to the item's own
    header, exactly as they did for the uuid alone."""
    m = re.search(r'\(uuid\s+"([^"]+)"', text)
    if m:
        return m.group(1)
    m = _TSTAMP_RE.search(text)
    return kiid_from_tstamp(m.group(1)) if m else ""


def convert_to_new_overbar_notation(old: str) -> str:
    """KiCad's ConvertToNewOverbarNotation (string_utils.cpp), ported as is:
    ``~X~`` -> ``~{X}``, ``~~`` -> ``~``, a space / ``}`` / ``)`` ends an
    open overbar, and a string already holding ``~{`` is returned untouched."""
    if old == '~':
        return old
    out = []
    in_overbar = False
    i, n = 0, len(old)
    while i < n:
        ch = old[i]
        if ch == '~':
            la = i + 1
            if la < n and old[la] == '~':
                if la + 1 < n and old[la + 1] == '{':
                    out.append('~~{}')
                    i += 1
                    continue
                out.append('~')
                i += 2
                continue
            if la < n and old[la] == '{':
                return old
            out.append('}' if in_overbar else '~{')
            in_overbar = not in_overbar
            i += 1
            continue
        if ch in ' })' and in_overbar:
            out.append('}')
            in_overbar = False
        out.append(ch)
        i += 1
    if in_overbar:
        out.append('}')
    return ''.join(out)


_LEGACY_ARC_RE = re.compile(
    r'\((gr_arc|fp_arc)(\s+(?:locked\s+)?)'
    r'\(start\s+([-\d.eE+]+)\s+([-\d.eE+]+)\)\s*'
    r'\(end\s+([-\d.eE+]+)\s+([-\d.eE+]+)\)\s*'
    r'\(angle\s+([-\d.eE+]+)\)')


def _fmt_mm(v: float) -> str:
    s = '%.6f' % v
    s = s.rstrip('0').rstrip('.')
    return '0' if s in ('-0', '') else s


def upgrade_legacy_arcs(content: str, kicad_version: Optional[int] = None) -> str:
    """CONTENT with every legacy center/angle arc rewritten as start/mid/end.

    For files at or before LEGACY_ARC_FORMATTING, KiCad reads ``(start)`` as the
    arc CENTER, ``(end)`` as the arc's START point and ``(angle)`` as the sweep
    (SetArcAngleAndEnd: end = start rotated by the angle, the two swapped when
    the angle is negative). Every reader here matches only the three-point
    form, so an old arc was simply dropped: a rounded outline lost its corners
    (or closed nothing at all). Analysis copies ONLY -- never write this back
    into a file of that version, which KiCad would then fail to load.
    """
    if kicad_version is None:
        kicad_version = detect_kicad_version(content)
    if not kicad_version or kicad_version > LEGACY_ARC_FORMATTING or '(angle' not in content:
        return content

    def _sub(m):
        cx, cy, sx, sy = (float(m.group(k)) for k in (3, 4, 5, 6))
        ang = float(m.group(7))

        def rot(a):
            r = math.radians(a)
            dx, dy = sx - cx, sy - cy
            return (cx + dx * math.cos(r) - dy * math.sin(r),
                    cy + dx * math.sin(r) + dy * math.cos(r))
        ex, ey = rot(ang)
        mx, my = rot(ang / 2.0)
        start, end = (sx, sy), (ex, ey)
        if ang < 0:
            start, end = end, start
        return '(%s%s(start %s %s) (mid %s %s) (end %s %s)' % (
            m.group(1), m.group(2),
            _fmt_mm(start[0]), _fmt_mm(start[1]), _fmt_mm(mx), _fmt_mm(my),
            _fmt_mm(end[0]), _fmt_mm(end[1]))
    return _LEGACY_ARC_RE.sub(_sub, content)


def is_kicad_10(content: str) -> bool:
    """Check if file content is KiCad 10+ format (name-only nets)."""
    return detect_kicad_version(content) >= KICAD_10_MIN_VERSION


def has_numeric_net_table(content: str) -> bool:
    """True if the file declares nets as a numeric table, ``(net <id> "name")``.

    KiCad 9 and earlier always write one (``(net 0 "")`` at least); pcbnew 10
    never does. Third-party generators write one under a KiCad-10 version
    stamp, which is why the stamp alone cannot decide the net dialect (#1028).
    """
    return bool(re.search(r'\(net\s+\d+\s+"', content))


def board_uses_name_nets(content: str) -> bool:
    """True if the board references nets by NAME (KiCad 10 style) rather than by
    numeric id (KiCad 9). Decided from the ACTUAL content, in this order:

    1. name-only refs ``(net "name")`` present -> names. Includes a pre-2025
       board a previous pass round-tripped (#163): its refs must be matched,
       and extended, by name.
    2. a numeric net table and no name refs -> ids, WHATEVER the version stamp
       says: third-party generators write a KiCad-10 stamp over a numeric table
       (#1028), and emitting name refs into it made a mixed file whose
       by-name strip/relabel matched none of its numeric-ref copper.
    3. neither (a board with no nets yet) -> the stamp decides.

    For every file pcbnew itself writes this agrees with the stamp: KiCad 9
    has the table and no name refs, KiCad 10 has name refs and no table.

    Writers use this to keep the output's net-token format consistent with the
    input - never emitting KiCad-10 name nets into a KiCad-9 numeric board (which
    KiCad 9 reads as net-less), nor numeric ids into a name-net board."""
    if re.search(r'\(net\s+"', content):
        return True
    if has_numeric_net_table(content):
        return False
    return is_kicad_10(content)


def pcb_uses_name_nets(pcb) -> bool:
    """board_uses_name_nets() for a parsed PCBData: the answer parse_kicad_pcb
    recorded from the file, else (the pcbnew path, which has no file text) the
    version stamp -- the GUI writes through pcbnew, never these net tokens."""
    recorded = getattr(pcb, 'uses_name_nets', None)
    if recorded is not None:
        return recorded
    return getattr(pcb, 'kicad_version', 0) >= KICAD_10_MIN_VERSION


@dataclass
class Pad:
    """Represents a component pad with global board coordinates."""
    component_ref: str
    pad_number: str
    global_x: float
    global_y: float
    local_x: float
    local_y: float
    size_x: float
    size_y: float
    shape: str  # circle, rect, roundrect, etc.
    layers: List[str]
    net_id: int
    net_name: str
    rotation: float = 0.0  # Pad's ABSOLUTE board angle in degrees. The file's
    # (at ... angle) / pcbnew GetOrientation already include the footprint's
    # rotation -- do NOT add it again (#369 A4; slot-drill capsules orient by this)
    pinfunction: str = ""
    drill: float = 0.0  # Drill size for through-hole pads (0 for SMD). For an
                         # oval/slot drill this is max(w, h) (back-compat: the
                         # slot's long axis); use pad_drill_capsule()/
                         # pad_drill_circles() for the real slot geometry.
    drill_w: float = 0.0  # Oval/slot drill x-size in the PAD frame (0 = round)
    # Hole position (#324/#325): pads with a copper offset ((drill (offset ..)))
    # have global_x/global_y at the COPPER CENTER (what every clearance/DRC/
    # obstacle consumer wants) while the drill stays at the anchor. None =
    # hole coincides with global_x/global_y (the overwhelmingly common case).
    hole_x: object = None
    hole_y: object = None
    drill_h: float = 0.0  # Oval/slot drill y-size in the PAD frame (0 = round)
    pintype: str = ""
    pad_type: str = ""  # KiCad pad kind: 'smd', 'thru_hole', 'np_thru_hole',
    # 'connect'. NPTH pads have NO copper (their size is just a mask opening /
    # keep-out), so copper-vs-copper DRC must skip them; only the hole matters.
    roundrect_rratio: float = 0.0  # Corner radius ratio for roundrect pads
    rect_rotation: float = 0.0  # Residual rect tilt in global frame for non-
    # orthogonal pads (deg, in (-90,90]). 0 for axis-aligned pads (the common
    # case) where the rotation is already baked into size_x/size_y. The
    # obstacle/DRC geometry rotates the pad rectangle by this angle.
    local_clearance: float = 0.0  # Per-pad clearance override (mm) from the
    # pad's own (clearance ...) token. 0 means "no override, use the global
    # clearance". Larger-than-global values (e.g. fiducial keep-clear rings)
    # must widen the obstacle halo or copper routes within the pad's clearance.
    polygons: Optional[List[List[Tuple[float, float]]]] = None  # Real copper
    # outline(s) in GLOBAL board mm for custom comb/finger pads (issue #188).
    # When set, the obstacle map and DRC rasterize these polygons instead of the
    # size_x x size_y bounding box, so the finger channels (and the empty side of
    # an off-anchor pad) stay routable. A chamfered-rect pad (#1450) carries
    # its outline here too, so a cut corner is not copper. None for ordinary
    # rect/circle/roundrect pads, which the bounding box models exactly.
    castellated: bool = False  # KiCad (property pad_prop_castellated): a
    # deliberate half-hole pad ON the board edge (run-6 fix 1.7). Consumers:
    # the castellated-landing retract post-pass pulls track endpoints that
    # land in the edge-clearance zone of such a pad back to its inner reach.
    # Set by BOTH parse paths (text + pcbnew).
    geometry_approximations: Tuple[str, ...] = ()  # Shape variants flattened by
    # this parser. Consumers claiming exact primitive geometry must disclose
    # these instead of certifying the simplified shape/size as native copper.
    # #962: the pad's OWN solder-paste overrides, raw, in mm and ratio. None
    # means the pad sets none, so the footprint's and then the board's apply
    # (paste_apertures.resolve_paste_margin, which follows KiCad's
    # PAD::GetSolderPasteMargin precedence). APPENDED: positional Pad(...)
    # constructions exist.
    paste_margin: Optional[float] = None
    paste_margin_ratio: Optional[float] = None
    # #962: a CUSTOM pad's anchor `(size w h)` in the PAD frame. `size_x` /
    # `size_y` of a custom pad are the primitive extent, but KiCad sizes the
    # paste ratio term from the anchor. None for every other shape.
    anchor_size: Optional[Tuple[float, float]] = None
    # KiCad's unconnected-layer mode (`(remove_unused_layers ..)` +
    # `(keep_end_layers ..)`): 'keep_all', 'remove_all' or
    # 'remove_except_start_end'. On a layer the mode removes, KiCad flashes the
    # pad only when copper reaches its HOLE, so a track ending in the annulus
    # but short of the drill is not connected there.
    # connectivity.pad_unflashed_layers reads it. Set by BOTH parse paths.
    unconnected_layer_mode: str = 'keep_all'
    # #1248: KiCad's PAD-TO-DIE length (`(die_length mm)`, PAD::
    # GetPadToDieLength) and delay (`(die_delay ps)`, GetPadToDieDelay): the
    # bond wire / package trace inside the part, which KiCad adds to the net's
    # length in its tuners and its length/skew DRC. 0.0 = none declared. Set by
    # BOTH parse paths. Consumers: net_queries.pad_to_die_length/_delay.
    die_length: float = 0.0
    die_delay: float = 0.0
    # #1279: the name of the protected Kelvin sense branch (a KiCad group
    # named `sense:<REF>.<PAD>`) this pad is a SENSE pad of, '' otherwise.
    # Set by sense_branch.resolve_sense_branches on both parse paths; the
    # router may never attach new copper to it.
    sense: str = ""
    # A CUSTOM pad's copper box `(x0, y0, x1, y1)` about (global_x, global_y),
    # board frame, when the copper's centre is off the pad's
    # (`copper_box_if_offset`): `size_x`/`size_y` box the copper
    # symmetrically about the pad, so copper drawn to one side of its anchor
    # reads twice its size (bitaxe's CSD17311Q5 drain pad, 10.0 x 9.1 mm for
    # 5.0 x 4.6 mm of copper). None for every other pad. Set by BOTH parse
    # paths; placement reads it (`legality.pad_copper_box`), routing reads
    # `polygons`.
    copper_box: Optional[Tuple[float, float, float, float]] = None
    # The pad's zone-connection OVERRIDE, spelled like Zone.pad_connection
    # ('none', 'thermal', 'solid', 'thru_hole_only'): the pad's own
    # `(zone_connect N)`, else its footprint's, resolved at parse time the way
    # local_clearance is. None = no override, so each zone's own connect_pads
    # decides. Never read it directly to ask "does the pour reach this pad":
    # pad_zone_connection(pad, zone) applies the zone and THT rules. Set by
    # BOTH parse paths.
    zone_connection: Optional[str] = None


_VIA_BIRTH_WATCH = None


def _via_birth_watch():
    """Parsed KICAD_VIA_BIRTH_WATCH spec (mm points), [] when unset."""
    global _VIA_BIRTH_WATCH
    if _VIA_BIRTH_WATCH is None:
        spec = os.environ.get("KICAD_VIA_BIRTH_WATCH", "").strip()
        out = []
        for part in spec.split(";"):
            part = part.strip()
            if part:
                try:
                    xs, ys = part.split(",")
                    out.append((float(xs), float(ys)))
                except ValueError:
                    pass
        _VIA_BIRTH_WATCH = out
    return _VIA_BIRTH_WATCH


@dataclass
class Via:
    """Represents a via."""
    x: float
    y: float
    size: float
    drill: float
    layers: List[str]
    net_id: int
    uuid: str = ""
    free: bool = False  # If True, KiCad won't auto-assign net based on overlapping tracks
    # (locked yes): the user pinned this via. Rip machinery must NEVER rip a
    # net's copper when any of it is locked (#521 follow-up); locked copper was
    # already an obstacle (#150). Set by BOTH parse paths (text + pcbnew).
    locked: bool = False
    # Protection spec: {token: raw inner s-expr}, e.g.
    # {'tenting': '(front yes) (back yes)'} for tenting/covering/plugging/
    # capping/filling. Held as raw text so it round-trips byte-faithfully
    # whatever sub-syntax the KiCad version uses. Empty = the board said nothing.
    #
    # This existed nowhere before #489 §8: the parser skipped these tokens and
    # the writer stamped `(tenting (front yes) (back yes))` on every via it
    # emitted, so any via that got RIPPED AND RE-PLACED (rip-up/reroute, the
    # sub-grid via nudge, tap relocation) silently lost its real spec -- which
    # matters most for via-in-pad, where IPC-4761 Type VII filled+capped+plated
    # is what keeps solder out of the barrel.
    tenting_attrs: Dict[str, str] = field(default_factory=dict)
    # #1279: the protected sense branch (KiCad group `sense:<REF>.<PAD>`) this
    # via is a member of, '' otherwise (see Pad.sense).
    sense: str = ""
    # #1279: the `sense:` group that holds this via immutable, '' otherwise
    # (see Segment.held_by).
    held_by: str = ""
    # KiCad's via type: 'through', 'blind', 'buried' or 'micro', from the
    # block's type word (text) or GetViaType() (pcbnew). It, not `layers`,
    # says whether the via has copper on only part of the stack: KiCad loads
    # a via block with NO type word as a THROUGH via whatever its `(layers ...)`
    # says, and both parse paths then report its pair as F.Cu/B.Cu (the text
    # parser snaps it, pcbnew already has). A Via the tool builds in-run is
    # through unless given a type, whatever pair it carries -- the back-side
    # BGA fanout's through vias used to carry `["B.Cu", "In6.Cu"]`. Read a
    # via's copper through geometry_utils.via_on_layer / via_copper_layers,
    # never off this pair.
    via_type: str = "through"


    def __post_init__(self):
        # VIA BIRTH WATCH (debug only): KICAD_VIA_BIRTH_WATCH="x,y;x,y" prints a
        # stack trace for every Via CONSTRUCTED near a watched point, naming the
        # producer. The obstacle probes can only say a cell was free; this says
        # who put copper there. Off unless the env var is set.
        _w = _via_birth_watch()
        if _w:
            for wx, wy in _w:
                if abs(self.x - wx) <= 0.05 and abs(self.y - wy) <= 0.05:
                    import traceback
                    print(f"[VIABIRTH] via net={self.net_id} "
                          f"({self.x:.3f},{self.y:.3f}) size={self.size} "
                          f"drill={self.drill}")
                    for ln in traceback.format_stack()[-7:-1]:
                        print("    " + ln.rstrip().replace("\n", " | "))
                    break

@dataclass
class Segment:
    """Represents a track segment."""
    start_x: float
    start_y: float
    end_x: float
    end_y: float
    width: float
    layer: str
    net_id: int
    uuid: str = ""
    # Original string representations for exact file matching
    start_x_str: str = ""
    start_y_str: str = ""
    end_x_str: str = ""
    end_y_str: str = ""
    # True for copper derived from a gr_line/gr_arc GRAPHIC on a copper layer
    # (#337): real copper for DRC/obstacles, but NOT a track -- cleanup passes
    # must never prune it and writers cannot strip it (there is no (segment)
    # block to match).
    graphic: bool = False
    # (locked yes): the user pinned this track. Rip machinery must NEVER rip a
    # net's copper when any of it is locked (#521 follow-up); locked copper was
    # already an obstacle (#150). Set by BOTH parse paths (text + pcbnew).
    locked: bool = False
    # #908: for copper drawn INSIDE a footprint, the DISAMBIGUATED footprint
    # key that owns it ('U2', 'TP4~2'); '' for board-level graphics and for
    # every routed track. It is what lets a DRC report name the object the way
    # KiCad does ("Polygon of U2 on F.Cu") instead of the anonymous `net_0`,
    # and what scopes the own-pad obstacle lift to the owning part.
    owner_ref: str = ""
    # #962: the stroke width AS DRAWN, for graphic copper; None for tracks.
    # `width` is what the obstacle model uses, and a filled shape drawn with
    # stroke 0 gets the fab track width there (0.3). So `width` over-states a
    # filled shape's outline by 0.15 mm a side, which is harmless as an obstacle
    # and wrong as a measurement: watchy AE1 is really 0.18 mm inside its edge
    # and would read as 0.03. Off-outline measurements read THIS.
    drawn_width: Optional[float] = None
    # #962: the primitive this graphic segment came from ('line', 'arc',
    # 'poly', 'rect', 'circle'; '' for tracks), and for a circle its TRUE
    # geometry, global (cx, cy, r). The outline emitter draws a circle as a
    # 16-gon whose chord midpoints sit 1.9% of r inside the real curve, so a
    # measurement taken on the chords under-reads a circle's reach.
    graphic_kind: str = ""
    graphic_circle: Optional[Tuple[float, float, float]] = None
    # #962: a closed graphic (poly/rect/circle) whose INTERIOR is copper, by
    # KiCad's loader rules (`_shape_filled` / pcbnew IsAnyFill). The segments
    # model only the outline, so a board cutout lying wholly inside a filled
    # tab is invisible to them; the off-outline census reads this to look
    # inside.
    graphic_filled: bool = False
    # #1181: for a FILLED closed graphic, its outline as one tuple of (x, y)
    # vertices, shared by every segment of the shape; None otherwise. The
    # segments are only the perimeter, so a via lying wholly inside the shape
    # touched none of them: the router put a foreign via in a filled shape and
    # check_drc graded the short clean. The obstacle map stamps this interior
    # and check_drc grades containment in it.
    graphic_ring: Optional[Tuple[Tuple[float, float], ...]] = None
    # #1279: the protected sense branch (KiCad group `sense:<REF>.<PAD>`) this
    # track is a member of, '' otherwise (see Pad.sense).
    sense: str = ""
    # #1279: the `sense:` group that holds this track immutable, '' otherwise.
    # Set on a group's member copper whenever it resolves on the join pad's
    # net, protected or refused, so no pass moves, rewrites or removes the
    # declared sense copper. It is not `locked`: nothing writes it to the file,
    # and it does not make the net never-rippable (only the net's pre-existing
    # copper is spared from whole-net strips, protected_nets.protection_map).
    held_by: str = ""


def is_immutable_copper(item) -> bool:
    """True for a track or via no pass may move, rewrite or remove: KiCad-
    locked copper (#1261) or copper a `sense:` group holds (#1279)."""
    return bool(getattr(item, 'locked', False) or getattr(item, 'held_by', ''))


@dataclass
class Zone:
    """Represents a filled zone (power plane)."""
    net_id: int
    net_name: str
    layer: str
    polygon: List[Tuple[float, float]]  # List of (x, y) vertices defining the zone outline
    uuid: str = ""
    priority: int = 0  # (priority N); higher-priority zones win where outlines overlap
    island_removal_mode: int = 0  # 0 = always remove isolated islands (KiCad default,
    # token absent from the file), 1 = never remove, 2 = remove below island_area_min
    island_area_min: float = 0.0  # mm^2 floor for mode 2 (KiCad file units are mm^2)
    clearance: float = None  # zone fill clearance (connect_pads (clearance X)); None = unknown
    min_thickness: float = None  # minimum fill width; None = unknown
    # #1329: how the pour joins its net's pads, and its spoke geometry -- read
    # so that a pass REPLACING the zone (route_planes) can say what it changed.
    # 'thermal' (KiCad's default, `(connect_pads (clearance X))`), 'solid'
    # (`connect_pads yes`), 'none' (`no`) or 'thru_hole_only'; None = unknown.
    pad_connection: str = None
    thermal_gap: float = None  # (fill (thermal_gap X)); None = unknown
    thermal_bridge_width: float = None  # (fill (thermal_bridge_width X)); spoke width
    in_footprint: bool = False  # #478: zone is owned by a footprint (nested in its
    # (footprint ...) block). Points are still BOARD coordinates. Consumers treat it
    # as normal copper/keep-out; plane creation must not replace or abort on it.
    teardrop: bool = False  # #1164: KiCad's (attr (teardrop ...)) -- a GENERATED
    # teardrop area where a track meets a pad or via. Routing output, like a
    # track: KiCad regenerates it on refill, and it sits on its pad's old pose.
    # #1279: the zone's CUT-OUTS. KiCad's file holds one outline per zone and
    # every later (polygon ...) block in it is a hole in that outline (pcbnew
    # loads it so: OutlineCount 1, HoleCount(0) = the rest; writing a second
    # outline drops it). The fill never enters a hole. Both parse paths fill
    # it; `polygon` stays the outline alone.
    holes: List[List[Tuple[float, float]]] = field(default_factory=list)


def pour_zones(zones):
    """Every zone but KiCad's generated TEARDROP areas: the zones that say
    "this net POURS here".

    A teardrop (`Zone.teardrop`) is real copper, and the obstacle map, DRC and
    connectivity keep reading it through `pcb.zones`. But it is routing output
    on whatever net the track carries, not a pour, so a consumer deciding "is
    this net a plane / poured net" must ask this instead: counted as a pour, a
    teardropped signal net read as a plane net (never rip-eligible, skipped
    by --force-reroute, handed to the plane finalize), and a GND teardrop on
    F.Cu read as "GND already pours on F.Cu", so route_planes kept the pour it
    was asked to create."""
    return [z for z in (zones or ()) if not getattr(z, 'teardrop', False)]


@dataclass
class GuidePath:
    """A user-drawn graphic polyline (e.g. on User.1) used as a routing corridor."""
    layer: str  # e.g. "User.1"
    points: List[Tuple[float, float]]  # ordered (x, y) in mm, >= 2 points
    is_closed: bool = False  # True for closed polygons (gr_poly)


@dataclass
class RefLabel:
    """The footprint's Reference-designator text (silkscreen label, #481).

    Parsed from the KiCad 8+ ``(property "Reference" ...)`` sub-node (KiCad
    6/7 ``(fp_text reference ...)`` as fallback) by BOTH parse paths -- the
    text parser and ``build_pcb_data_from_board`` -- in the same normalized
    form, so label geometry decisions are front-end independent (the parity
    contract, pinned by tests/gui_parity/test_ref_label_pcbnew_parity.py).

    Angle semantics (settled by a pcbnew probe, see
    ``placement/labels.py::label_world_angle``): ``rotation`` is the label's
    ABSOLUTE board angle, not a footprint-relative one. ``at_x``/``at_y`` are
    the offset from the footprint origin in its UNROTATED local frame
    (pre-mirrored for B-side parts, like pad locals) -- world position is
    ``local_to_global(fp.x, fp.y, fp.rotation, at_x, at_y)``.
    """
    at_x: float          # local-frame offset from the footprint origin (mm)
    at_y: float
    rotation: float      # ABSOLUTE board angle, degrees, normalized to [0, 360)
    size_h: float = 1.0  # font height/width in mm; the FILE stores
    size_w: float = 1.0  # (size HEIGHT WIDTH) -- probed, KiCad 10 writeback
    thickness: float = 0.15  # stroke width in mm
    layer: str = "F.SilkS"
    justify: str = ""    # raw (justify ...) tokens, space-joined ("left bottom
    #                      mirror"); "" = KiCad's centered default (no token
    #                      on any corpus Reference node)
    hidden: bool = False  # (hide yes), or the bare `hide` token KiCad <=8 wrote
    is_property_node: bool = True  # False only for the KiCad 6/7 fp_text form


@dataclass
class Footprint:
    """Represents a component footprint."""
    reference: str
    footprint_name: str
    x: float
    y: float
    rotation: float
    layer: str
    pads: List[Pad] = field(default_factory=list)
    value: str = ""  # Component value (e.g., "MCF5213", "100nF", "10K")
    dnp: bool = False  # Do-not-populate / no-pop. A no-pop series part is an open
                       # circuit, so its pads do NOT bridge two nets into one signal.
    locked: bool = False  # Footprint (locked yes) flag: the user pinned this part,
                          # so placement passes must not move it and routing/fanout
                          # cannot assume a later step will move it out of the way.
    clearance: float = 0.0  # Footprint-level (clearance ...) override (mm),
    # inherited by every pad of this footprint that has no override of its own
    # (KiCad resolution: pad override, else footprint override, else netclass).
    # The parser RESOLVES the inheritance into each pad's local_clearance at
    # parse time, so clearance consumers only ever read pad.local_clearance;
    # this field records the raw footprint value for fidelity (issue #326).
    uuid: str = ""  # The footprint's own (uuid ...). KiCad's (group ...) blocks
    # list their members BY UUID, so placement grouping (#459) needs it to
    # resolve membership; it is also already the fallback dict key for a
    # reference-less footprint ("#" + uuid), so both parse paths must agree.
    sheet_path: str = ""  # (path "/sheet-uuid/.../symbol-uuid"). The LAST
    # component is the symbol; the prefix is the schematic sheet, which is the
    # functional block a floorplan thinks in and the most widely available
    # grouping signal (12 of 22 corpus boards with paths have >1 sheet, where
    # NONE has a (group ...) block). A bare single-uuid path is top level, i.e.
    # an EMPTY sheet prefix -- those must not collapse into one bogus block.
    net_tie_groups: List[List[str]] = field(default_factory=list)
    # KiCad (net_tie_pad_groups "1,2" ...): pad-number groups this footprint
    # DELIBERATELY shorts (Kelvin shunts, net-tie parts). KiCad exempts copper
    # of the grouped pads' nets from mutual clearance at the footprint, so a
    # sense pad enclosed by its partner's ring is still routable (cynthion
    # R42: the AUX_SENSE- tab sits inside AUX_VBUS_IN's pad -- treating the
    # partner as a hard obstacle seals the net forever). Obstacle builders and
    # check_drc consume this via PCBData.net_tie_exempt_pad_ids().
    ref_label: Optional['RefLabel'] = None  # Reference-designator text geometry
    # (#481); None for a reference-less footprint or an unparseable node.
    # APPENDED (never mid-insert): positional Footprint(...) constructions
    # exist across the tree and a mid-list field would silently shift them.
    owns_edge_cuts: bool = False    # draws Edge.Cuts geometry of its own (#829)
    owns_board_outline: bool = False
    # #829. `owns_edge_cuts` is the FACT (this footprint's pose transforms a
    # piece of Edge.Cuts); `owns_board_outline` is the DECISION that follows
    # from it -- True when that geometry IS part of the board boundary: an
    # open path, or a closed ring that encloses some of the part's own pads
    # (or has none to judge by) and lies outside the board-level outline
    # (`classify_outline_owners`).
    # A milled relief or a window parented to a part so it travels with the
    # part (crkbd's 184 per-LED windows; #628's owned rings) is
    # owns_edge_cuts=True, owns_board_outline=False, and stays movable.
    # Movers must gate on `owns_board_outline`; see kicad_parser.
    # footprint_outline_owners. Also APPENDED, for the reason above.
    # #962: footprint-level solder-paste overrides (raw mm / ratio; None = not
    # set). Pads without their own inherit these; see Pad.paste_margin.
    paste_margin: Optional[float] = None
    paste_margin_ratio: Optional[float] = None
    # #962: the pose (x, y, rotation, layer) the footprint had when this
    # PCBData was PARSED. Graphic copper (Segment.graphic, owner_ref) is placed
    # at parse time. A caller that moves `x`/`y`/`rotation` in memory would
    # otherwise grade that copper where the part USED to be.
    # check_drc.footprint_graphic_outline_census re-poses it from this
    # (a side flip is reported unmeasured). None = unknown, taken as unmoved.
    parsed_pose: Optional[Tuple[float, float, float, str]] = None
    # #1098: the footprint's (attr ...) flags, as the sorted subset of
    # FOOTPRINT_ATTR_TOKENS it carries, and whether it declares a 3D model.
    # Together they say what the ASSEMBLY house places: a part flagged
    # `board_only` or `exclude_from_pos_files` with no model is board copper
    # (a PCB-trace USB plug, a card edge, a jumper), not a component. Both
    # parse paths fill them; APPENDED like the fields above.
    attrs: Tuple[str, ...] = ()
    has_model: bool = False
    # #1211: the schematic sheet this footprint's symbol sits on, as the FILE
    # spells it -- the human name beside `sheet_path`'s uuids. KiCad 8+ writes
    # header tokens `(sheetname "/CH1/")` / `(sheetfile "strip.kicad_sch")`;
    # KiCad 6/7 wrote `(property "Sheetname" "CH1")` / `(property "Sheetfile"
    # ...)`, and the VALUE differs too (a full path "/CH1/" on some boards,
    # the bare name "IO_Buffer_B" on others). Not normalised: pcbnew's
    # GetSheetname()/GetSheetfile() return the same spelling, so both parse
    # paths agree string for string. '' = the file names no sheet. Appended.
    sheet_name: str = ""
    sheet_file: str = ""
    # The footprint-level `(zone_connect N)` override, raw (see
    # Pad.zone_connection, which already carries it for every pad without one
    # of its own). None = not set. Appended.
    zone_connection: Optional[str] = None


#: The (attr ...) flags `Footprint.attrs` keeps -- the ones the text form and
#: pcbnew's GetAttributes() bitmask can both spell, so the two parse paths
#: agree token for token (#1098).
FOOTPRINT_ATTR_TOKENS = ('allow_missing_courtyard', 'allow_soldermask_bridges',
                         'board_only', 'dnp', 'exclude_from_bom',
                         'exclude_from_pos_files', 'smd', 'through_hole')


@dataclass
class Net:
    """Represents a net (electrical connection)."""
    net_id: int
    name: str
    pads: List[Pad] = field(default_factory=list)


@dataclass
class StackupLayer:
    """A layer in the board stackup."""
    name: str
    layer_type: str  # 'copper', 'core', 'prepreg', etc.
    thickness: float  # in mm
    epsilon_r: float = 0.0  # Dielectric constant (for dielectric layers)
    loss_tangent: float = 0.0  # Loss tangent (for dielectric layers)
    material: str = ""  # Material name (e.g., "FR4", "S1000-2M")


@dataclass
class BoardInfo:
    """Board-level information."""
    layers: Dict[int, str]  # layer_id -> layer_name
    copper_layers: List[str]
    board_bounds: Optional[Tuple[float, float, float, float]] = None  # min_x, min_y, max_x, max_y
    stackup: List[StackupLayer] = field(default_factory=list)  # ordered top to bottom
    board_outline: List[Tuple[float, float]] = field(default_factory=list)  # Polygon vertices for non-rectangular boards
    board_cutouts: List[List[Tuple[float, float]]] = field(default_factory=list)  # Interior cutout polygons
    # ALL outer boundary rings (issue #304). A board can carry several disjoint
    # outlines in one file (split keyboards drawn as left+right halves,
    # panelized boards); board_outline keeps the LARGEST for back-compat, this
    # holds every outer ring. Empty on single-outline/rectangular boards means
    # "use board_outline".
    board_outlines: List[List[Tuple[float, float]]] = field(default_factory=list)
    # Edge.Cuts contours that are MILLED BOUNDARIES for clearance but are not
    # holes and not outer rings (#505). drop_pad_containing_cutouts moves a
    # pad-containing inner contour here: it must not act as a cutout (that would
    # mark every pad it encloses off-board, #291), but KiCad still routes the
    # board along it, so copper has to keep its edge clearance from it. Consumers
    # measure clearance / stamp a keep-out BAND around these, and never treat
    # their interior as off-board. crkbd: the real 64-point board outline nested
    # inside the panel frame -- dropping it outright let the router lay copper
    # straight across the milled gap between the two keyboard halves (85 items).
    board_edge_contours: List[List[Tuple[float, float]]] = field(default_factory=list)
    keepouts: List[dict] = field(default_factory=list)  # Keep-out rule areas: {polygon, layers:set, tracks_allowed, vias_allowed, copper_pour_allowed, pads_allowed, footprints_allowed, owner, name}
    # Smallest copper clearance any routing step actually used on this board this
    # run -- e.g. a fine-pitch tap that escalated below the nominal --clearance.
    # None until a step records one. Routers fold this into the .kicad_pro DRC
    # floor and JSON so check_drc grades at the true routed clearance, not the
    # nominal one (which would flag legitimately tight copper).
    min_clearance_used: Optional[float] = None
    # #962: board `(setup (pad_to_paste_clearance X) (pad_to_paste_clearance_ratio R))`,
    # the last rung of the paste-margin precedence (0 = not set, KiCad's default).
    pad_to_paste_clearance: float = 0.0
    pad_to_paste_clearance_ratio: float = 0.0
    # #962: the board's via-protection POLICY from `(setup ...)`, in the same
    # {token: inner} form as Via.tenting_attrs, CANONICALISED so both parse
    # paths agree:
    # - all five tokens are present;
    # - absent ones carry KiCad's factory default;
    # - the legacy KiCad 9 `(tenting front back)` is spelled
    #   `(front yes) (back yes)`.
    # A via with no spec of its own inherits this. check_drc's via-in-paste
    # grade reads it per token.
    via_protection_setup: Dict[str, str] = field(default_factory=dict)


@dataclass
class PCBData:
    """Complete parsed PCB data."""
    board_info: BoardInfo
    nets: Dict[int, Net]
    footprints: Dict[str, Footprint]
    vias: List[Via]
    segments: List[Segment]
    pads_by_net: Dict[int, List[Pad]]
    zones: List[Zone] = field(default_factory=list)
    kicad_version: int = 0  # File format version (e.g., 20241229 for KiCad 9)
    # The file's net-token dialect, board_uses_name_nets() over its text. Read
    # it through pcb_uses_name_nets(). None on the pcbnew path, which has no
    # file text and writes through pcbnew rather than net tokens.
    uses_name_nets: Optional[bool] = None
    net_id_to_name: Dict[int, str] = field(default_factory=dict)  # Synthetic ID -> net name (for KiCad 10 output)
    # Net class data read from the sibling .kicad_pro file. kipy's
    # BoardDesignRules only exposes board-wide minimums and predefined
    # sizes — per-class settings (track width, clearance, via diameter,
    # diff-pair width/gap) live in the project JSON.
    netclass_params: Dict[str, Dict[str, float]] = field(default_factory=dict)  # class_name -> {param: mm}
    net_to_class: Dict[str, str] = field(default_factory=dict)  # net_name -> class_name
    guide_paths: List[GuidePath] = field(default_factory=list)  # User-drawn guide corridors (issue #7)
    keepout_zones: List[GuidePath] = field(default_factory=list)  # User-drawn keepout polygons (issue #27)
    # #498: absolute path of the board this data came from ("" = in-memory /
    # unknown). Lets engines whose signatures carry no input_file (planes,
    # fanout) auto-discover sibling project files -- today the .kicad_dru
    # per-layer clearance rules. parse_kicad_pcb sets it from its argument;
    # build_pcb_data_from_board from board.GetFileName().
    source_path: str = ""
    # #424: callable returning {(net_name, layer): [island_polygon, ...]}
    # (the refill_islands result shape) for exact-fill consumers
    # (plane_fragility, anything wanting KiCad fill truth). None = refill
    # source_path. build_pcb_data_from_board sets it to an IN-PROCESS
    # ZONE_FILLER refill of the LIVE board (kicad_exact_fill.
    # live_fill_islands) -- the on-disk file can be stale mid-plan (earlier
    # steps' copper applied to the live board only), and the fill truth must
    # match the copper the engine is routing against. Not a file path on
    # purpose: the GUI never needs to save to be priced correctly.
    exact_fill_provider: object = None
    # #1187: callable returning the LIVE board's design-settings rules,
    # {rule key: mm} in the .kicad_pro's own keys (min_track_width,
    # min_connection), which the floor readers (fix_kicad_drc_settings.
    # connection_width_floor) take in place of the sibling file. A callable,
    # read when the floor is asked for, like the CLI's file read: mid-plan the
    # file beside the live board is the ORIGINAL project, the floors the plan's
    # steps lowered live in pcbnew's memory (update_live_drc_floors), and a
    # PCBData the GUI built before a step's writeback is still in use after it
    # -- a value captured at build time read the pre-writeback floor there.
    # None = read the file (parse_kicad_pcb). It also answers
    # min_hole_clearance, the copper-to-hole floor obstacle_map prices NPTH
    # holes at (declared_hole_clearance). It answers only the rules the live
    # board DECLARES (fix_kicad_drc_settings.live_declared_rules): pcbnew's
    # factory stand-ins on a project-less board are left out, as the file
    # leaves them out.
    live_rules_provider: object = None
    # Callable returning the LIVE board's kicad_routing_tools records (the
    # fab-floor origin, protected nets, impedance specs, pour promises, ...):
    # the newer of its project's copy and the copy on the board itself
    # (tool_records.live_namespace), read when called. The GUI keeps them on
    # the board because KiCad's own project save drops what was written into
    # the .kicad_pro after load. None = read the files (parse_kicad_pcb).
    live_records_provider: object = None
    # #459: KiCad (group "name" (uuid ...) (members <uuid> ...)) blocks, as
    # {group name: [footprint reference, ...]}. The designer's own statement that
    # these parts belong together, so placement grouping ranks it above any
    # inferred signal. Members are stored by uuid in the file and resolved
    # against Footprint.uuid here; non-footprint members (zones, graphics) are
    # dropped. Empty on every board in the corpus -- no in-repo board uses
    # groups -- so consumers must treat absence as normal, not as an error.
    groups: Dict[str, List[str]] = field(default_factory=dict)
    #: #726: {reference as the FILE spells it: how many blocks claim it}, for
    #: the references claimed by more than one. The value is the total
    #: occurrence count (`TP4: 2`), not the number of extras.
    #:
    #: `footprints` keys are the DISAMBIGUATED names -- the first block keeps
    #: the bare reference, later ones get `~2`, `~3` -- so every block is
    #: present and `len(footprints)` is the block count. This dict is what
    #: says the disambiguation happened, and it is how a consumer reports the
    #: board's own spelling back to a human who has to go and fix the
    #: schematic. Filled identically by BOTH parse paths.
    #:
    #: Advisory. A duplicate reference is legal in KiCad and can be deliberate
    #: (a paired testpoint, a net tie), so nothing here refuses. It measured 5
    #: of the 22 tracked boards: watchy TP4/TP5 (real test points), esp_prog
    #: Ref*, glasgow_revC REF** x7, orangecrab_ext_pll G*** x3, ulx3s EMARD.
    duplicate_references: Dict[str, int] = field(default_factory=dict)
    #: #962: every solder-paste opening on the board: pad openings inflated by
    #: their resolved margin, paste-only pads, and F.Paste/B.Paste graphics.
    #: Built by paste_apertures.build_paste_apertures on BOTH parse paths.
    #: Which ones concern a given net's vias: paste_apertures.apertures_for_net.
    paste_apertures: List[PasteAperture] = field(default_factory=list)
    #: #962: footprint copper the parser does NOT model, named so a grade that
    #: cannot see it says so instead of passing it:
    #: `{'owner_ref', 'kind', 'reason'}`, kind 'logo' (a pad-less footprint's
    #: copper, which the writer relocates to silk, #146), 'curve' (bezier) or
    #: 'text' (copper text on a part with pads).
    #: Filled by both parse paths.
    graphic_copper_unmeasured: List[dict] = field(default_factory=list)
    #: #1279: protected Kelvin sense branches, one per KiCad group named
    #: `sense:<REF>.<PAD>` (sense_branch.SenseBranch). Separate from `groups`,
    #: which holds footprint members only (#459). Resolved against the parsed
    #: tracks and vias by uuid on BOTH parse paths; a branch that cannot be
    #: protected carries its `reason`.
    sense_branches: list = field(default_factory=list)

    def net_tie_exempt_pad_ids(self, net_id: int):
        """id()s of pads whose keep-out copper of `net_id` may IGNORE.

        For every footprint net-tie group (net_tie_pad_groups) that contains a
        pad on `net_id`, the OTHER pads of that group are deliberate shorts at
        the footprint -- KiCad exempts their mutual clearance, and the partner
        pad often physically encloses the tied pad (Kelvin shunts: cynthion
        R42's AUX_SENSE- tab sits inside AUX_VBUS_IN's pad). Obstacle builders
        skip stamping these pads when routing `net_id`; check_drc skips the
        corresponding pairs. Cached per net; identity keying is safe because
        the pad objects live in this PCBData for its whole lifetime.
        """
        cache = getattr(self, '_net_tie_exempt_cache', None)
        if cache is None:
            cache = {}
            self._net_tie_exempt_cache = cache
        got = cache.get(net_id)
        if got is not None:
            return got
        exempt = set()
        for fp in self.footprints.values():
            if not fp.net_tie_groups:
                continue
            by_num = {}
            for p in fp.pads:
                by_num.setdefault(p.pad_number, []).append(p)
            for group in fp.net_tie_groups:
                members = [p for num in group for p in by_num.get(num, [])]
                if any(p.net_id == net_id for p in members):
                    exempt.update(id(p) for p in members if p.net_id != net_id)
        cache[net_id] = exempt
        return exempt

    def get_via_barrel_length(self, layer1: str, layer2: str) -> float:
        """Calculate the via barrel length between two copper layers.

        Args:
            layer1: First copper layer name (e.g., 'F.Cu', 'In2.Cu')
            layer2: Second copper layer name

        Returns:
            Distance in mm through the board between the two layers
        """
        stackup = self.board_info.stackup
        if not stackup:
            return 0.0

        # Find indices of the two layers in stackup
        idx1 = idx2 = -1
        for i, layer in enumerate(stackup):
            if layer.name == layer1:
                idx1 = i
            elif layer.name == layer2:
                idx2 = i

        if idx1 < 0 or idx2 < 0:
            return 0.0

        # Sum thicknesses between the two layers (exclusive of the layers themselves)
        start_idx = min(idx1, idx2)
        end_idx = max(idx1, idx2)

        total = 0.0
        for i in range(start_idx, end_idx + 1):
            total += stackup[i].thickness

        return total


def flip_layer_token(name: str, inner_count: Optional[int] = None) -> str:
    """The other face's spelling of a layer token (#714).

    A PREFIX RULE, deliberately not a table. Three name->id tables in this repo
    already happen to enumerate the F/B pairs (`fix_kicad_drc_settings.
    _TECH_LAYER_IDS`, and twice in this file's pcbnew paths); a fourth
    enumeration is a fourth thing to keep in sync, and the enumeration is not
    where the knowledge is. The knowledge is that KiCad spells a sided layer
    `F.<x>` / `B.<x>` and an unsided one anything else.

    Measured over the 22 tracked boards, the layer tokens that appear inside a
    `(footprint ...)` block are exactly:

        F.Cu F.Mask F.Paste F.SilkS F.CrtYd F.Fab F.Adhes
        B.Cu B.Mask B.Paste B.SilkS B.CrtYd B.Fab
        *.Cu *.Mask Dwgs.User Cmts.User Eco1.User Eco2.User User.1

    This flips the THIRTEEN sided ones and returns the other SEVEN unchanged --
    including both wildcards, which pcbnew also leaves alone (`*.Cu` is KiCad's
    ALL-copper set and is already its own mirror; rewriting it to `B*.Cu` or
    `B.Cu` would silently narrow 66 pads on rp2350's U8 alone).

    Inner copper mirrors through the stack when `inner_count` (the board's
    number of inner copper layers) is given: `In<k>.Cu` becomes
    `In<n+1-k>.Cu`, so a flipped part's inner-layer items stay next to the
    face they were next to. That is what pcbnew does since 10.0.6, measured by
    the parity gate's inner-layer witness (an `fp_text` on `In1.Cu` and a
    footprint zone on `In1.Cu`/`In2.Cu` of a six-layer board land on `In4.Cu`
    and `In3.Cu`/`In4.Cu`). pcbnew 10.0.0 left them where they were, which is
    why the count is a parameter rather than a guess: without it an inner
    layer passes through unchanged. `placement.writer` always passes it, and
    still refuses a PAD naming an explicit `In<n>.Cu` (padstack semantics no
    tracked board exercises).
    """
    if name.startswith('F.'):
        return 'B.' + name[2:]
    if name.startswith('B.'):
        return 'F.' + name[2:]
    m = re.match(r'In(\d+)\.Cu$', name)
    if m and inner_count:
        k = int(m.group(1))
        if 1 <= k <= inner_count:
            return f'In{inner_count + 1 - k}.Cu'
    return name


def local_to_global(fp_x: float, fp_y: float, fp_rotation_deg: float,
                    pad_local_x: float, pad_local_y: float) -> Tuple[float, float]:
    """
    Transform pad LOCAL coordinates to GLOBAL board coordinates.

    CRITICAL: Negate the rotation angle! KiCad's rotation convention requires
    negating the angle when applying the standard rotation matrix formula.

    The result is SNAPPED to KiCad's integer-nanometre grid, because that is the
    only coordinate space a board can actually store. Everything else in PCBData
    is read verbatim from the file (segments, vias, footprint origins) and so
    matches pcbnew bit-for-bit; pad globals are the one value we must COMPUTE,
    and pcbnew computes them in integer nm while we compute them in float mm.
    Without the snap the two answers land up to ~0.3 nm apart -- geometrically
    nil, but different BIT PATTERNS, which is enough to flip an A* tie-break.
    That is exactly how eth_tap step 11 diverged: 64 of 1699 pads came out a few
    femtometres off (44.300000000000004 vs 44.3), and the GUI (pcbnew-backed
    PCBData) then routed 16 segments differently from the CLI (text-parsed
    PCBData) -- 3569 vs 3472 segments by the end of the chain. Snapping makes
    all 1699 pads bit-identical to pcbnew and the two fronts route the same.

    Same family as #493: canonicalise to integer nm rather than trusting float
    mm arithmetic to agree across two implementations.
    """
    rad = math.radians(-fp_rotation_deg)  # CRITICAL: negate the angle
    cos_r = math.cos(rad)
    sin_r = math.sin(rad)

    global_x = fp_x + (pad_local_x * cos_r - pad_local_y * sin_r)
    global_y = fp_y + (pad_local_x * sin_r + pad_local_y * cos_r)

    return (mm_to_iu(global_x) / IU_PER_MM, mm_to_iu(global_y) / IU_PER_MM)


# Tolerance (deg) within which a pad rotation is treated as axis-aligned and
# baked into size_x/size_y rather than carried as a residual rect_rotation.
_PAD_ORTHO_TOL = 1.0


# 3-point arc entry, two spellings with identical fields: (arc ...) inside a
# KiCad 7+ polygon point list, and a standalone (gr_arc ...) pad primitive.
#   (pts (arc (start x y) (mid x y) (end x y)) (xy x y) ...)
_PTS_ARC_RE = (r'\((?:gr_)?arc\s+\(start\s+(-?[\d.]+)\s+(-?[\d.]+)\)\s+'
               r'\(mid\s+(-?[\d.]+)\s+(-?[\d.]+)\)\s+'
               r'\(end\s+(-?[\d.]+)\s+(-?[\d.]+)\)')


def _rect_corner_radius(block: str, w: float, h: float) -> float:
    """A rect primitive's KiCad 9 corner `(radius r)`, clamped (as KiCad
    clamps it) just under half the shorter side; 0 when absent."""
    m = re.search(r'\(radius\s+(%s)\)' % r'-?[\d.]+(?:[eE][-+]?\d+)?', block)
    r = float(m.group(1)) if m else 0.0
    return max(0.0, min(r, min(abs(w), abs(h)) / 2.0 - 1e-6))


def _custom_pad_primitive_points(pad_text: str):
    """Per-primitive (points, half_stroke) tuples from a custom pad's
    (primitives ...) block, in the pad's local frame. Arcs (gr_poly pts arcs
    AND standalone gr_arc) are linearized; gr_circle yields its axis extremes.
    Used by the board-frame extent below."""
    pm = re.search(r'\(primitives\b', pad_text)
    if not pm:
        return []
    prim = pad_text[pm.start():find_matching_paren(pad_text, pm.start()) - 1]
    out = []
    for m2 in re.finditer(r'\(gr_(poly|circle|rect|line|arc)\b', prim):
        block = prim[m2.start():find_matching_paren(prim, m2.start()) - 1]
        wm = re.search(r'\(width\s+(-?[\d.]+)\)', block)
        hw = (float(wm.group(1)) / 2.0) if wm else 0.0
        pts = []
        for m in re.finditer(r'\(xy\s+(-?[\d.]+)\s+(-?[\d.]+)\)', block):
            pts.append((float(m.group(1)), float(m.group(2))))
        for m in re.finditer(_PTS_ARC_RE, block):
            s = (float(m.group(1)), float(m.group(2)))
            mid = (float(m.group(3)), float(m.group(4)))
            e = (float(m.group(5)), float(m.group(6)))
            for p1, p2 in _arc_to_segments(s, mid, e):
                pts.append(p1); pts.append(p2)
        cm = re.search(r'\(center\s+(-?[\d.]+)\s+(-?[\d.]+)\)\s+\(end\s+(-?[\d.]+)\s+(-?[\d.]+)\)', block)
        if cm and m2.group(1) == 'circle':
            cx, cy, ex, ey = map(float, cm.groups())
            r = math.hypot(ex - cx, ey - cy)
            # A circle is rotation-invariant about its centre: represent it as
            # the centre point with the OUTER radius (centreline radius + half
            # the stroke -- KiCad strokes the outline of both filled and
            # unfilled circles) folded into the half-stroke term. The extent
            # consumer then computes |rotated centre| + r + w/2 exactly at ANY
            # pad angle. Sampling the four axis extremes under-covered rotated
            # pads (#418, ottercast MK1: a 315deg unfilled ring lost ~0.4mm of
            # copper per side vs pcbnew).
            out.append(([(cx, cy)], r + hw))
            continue
        elif m2.group(1) in ('rect', 'line'):
            sm = re.search(r'\(start\s+(-?[\d.]+)\s+(-?[\d.]+)\)\s+\(end\s+(-?[\d.]+)\s+(-?[\d.]+)\)', block)
            if sm:
                x1, y1, x2, y2 = map(float, sm.groups())
                if m2.group(1) == 'line':
                    pts += [(x1, y1), (x2, y2)]
                else:
                    # A rounded corner (KiCad 9 `(radius r)`): the inset
                    # corners plus r, which is the exact extent.
                    x1, x2 = min(x1, x2), max(x1, x2)
                    y1, y2 = min(y1, y2), max(y1, y2)
                    rr = _rect_corner_radius(block, x2 - x1, y2 - y1)
                    x1, y1, x2, y2 = x1 + rr, y1 + rr, x2 - rr, y2 - rr
                    hw += rr
                    pts += [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]
        if pts:
            out.append((pts, hw))
    return out


def _offset_polygon_outward(pts, hw):
    """Offset a closed polygon outward by ``hw`` (mitered per-vertex normals,
    miter capped at 4x for very sharp corners). Winding is derived from the
    shoelace sum, so either vertex order works. Approximate at reflex
    (concave) vertices -- the notch is offset too, mildly over-covering it --
    which is conservative for clearance to external copper."""
    n = len(pts)
    if n < 3 or hw <= 0:
        return pts
    area2 = sum(pts[i][0] * pts[(i + 1) % n][1] - pts[(i + 1) % n][0] * pts[i][1]
                for i in range(n))
    sign = 1.0 if area2 > 0 else -1.0
    normals = []
    for i in range(n):
        dx = pts[(i + 1) % n][0] - pts[i][0]
        dy = pts[(i + 1) % n][1] - pts[i][1]
        length = math.hypot(dx, dy)
        normals.append((sign * dy / length, -sign * dx / length)
                       if length > 1e-12 else None)
    out = []
    for i in range(n):
        adj = [v for v in (normals[i - 1], normals[i]) if v is not None]
        if not adj:
            out.append(pts[i])
            continue
        ax = sum(v[0] for v in adj)
        ay = sum(v[1] for v in adj)
        length = math.hypot(ax, ay)
        if length < 1e-12:
            out.append(pts[i])
            continue
        bx, by = ax / length, ay / length
        cos_half = max(0.25, bx * adj[0][0] + by * adj[0][1])
        out.append((pts[i][0] + bx * hw / cos_half,
                    pts[i][1] + by * hw / cos_half))
    return out


def _round_join_steps(sweep: float, r: float, eps: float = 0.001) -> int:
    """Chords for an INSCRIBED arc of radius `r` over `sweep` radians with a
    sagitta of at most `eps` mm (the pad-primitive budget, run-4 B6)."""
    if r <= eps:
        return 1
    step = 2.0 * math.acos(max(-1.0, min(1.0, 1.0 - eps / r)))
    return max(1, min(64, int(math.ceil(abs(sweep) / step)))) if step > 0 else 64


def _offset_polygon_round(pts, hw):
    """A closed polygon grown by ``hw`` with ROUND joins: its Minkowski sum
    with a disc, which is the copper KiCad builds for a filled polygon stroked
    at ``2*hw`` (EDA_SHAPE::TransformShapeToPolygon inflates with
    ROUND_ALL_CORNERS). A convex vertex gets an inscribed arc between its two
    edge offsets; a reflex vertex the offsets' intersection, which is exact
    there. `_offset_polygon_outward`'s miter at a convex corner reached up to
    4*hw past the real copper (laser_backplane_dvi J8.2: 18 um)."""
    n = len(pts)
    if n < 3 or hw <= 0:
        return pts
    area2 = sum(pts[i][0] * pts[(i + 1) % n][1] - pts[(i + 1) % n][0] * pts[i][1]
                for i in range(n))
    sign = 1.0 if area2 > 0 else -1.0
    normals = []
    for i in range(n):
        dx = pts[(i + 1) % n][0] - pts[i][0]
        dy = pts[(i + 1) % n][1] - pts[i][1]
        length = math.hypot(dx, dy)
        normals.append((sign * dy / length, -sign * dx / length)
                       if length > 1e-12 else None)
    out = []
    for i in range(n):
        n0, n1 = normals[i - 1], normals[i]
        if n0 is None or n1 is None:
            v = n0 or n1
            out.append((pts[i][0] + v[0] * hw, pts[i][1] + v[1] * hw) if v else pts[i])
            continue
        turn = math.atan2(n0[0] * n1[1] - n0[1] * n1[0], n0[0] * n1[0] + n0[1] * n1[1])
        if turn * sign > 1e-9:          # convex: the disc rounds this corner
            a0 = math.atan2(n0[1], n0[0])
            m = _round_join_steps(turn, hw)
            for k in range(m + 1):
                a = a0 + turn * k / m
                out.append((pts[i][0] + hw * math.cos(a), pts[i][1] + hw * math.sin(a)))
        else:                           # reflex or straight: the offsets meet
            bx, by = n0[0] + n1[0], n0[1] + n1[1]
            length = math.hypot(bx, by)
            if length < 1e-12:
                out.append((pts[i][0] + n1[0] * hw, pts[i][1] + n1[1] * hw))
                continue
            bx, by = bx / length, by / length
            cos_half = max(0.25, bx * n0[0] + by * n0[1])
            out.append((pts[i][0] + bx * hw / cos_half, pts[i][1] + by * hw / cos_half))
    return out


def _stroke_band(pts, hw):
    """A stroked polyline's copper as ONE ring with ROUND caps, as KiCad
    strokes a line or an arc: both sides offset by ``hw`` (per-vertex
    averaged normals) and an inscribed half-disc round each end. The square
    cap this replaced reached ``hw*(sqrt(2)-1)`` past the round one at a
    corner (cm4_baseboard H101's 0.2 mm arc strokes: 17 um past pcbnew's
    copper)."""
    if len(pts) < 2 or hw <= 0:
        return None
    normals = []
    for i in range(len(pts)):
        acc_x = acc_y = 0.0
        for j in (i - 1, i):
            if 0 <= j < len(pts) - 1:
                dx = pts[j + 1][0] - pts[j][0]
                dy = pts[j + 1][1] - pts[j][1]
                L = math.hypot(dx, dy)
                if L > 1e-12:
                    acc_x += -dy / L
                    acc_y += dx / L
        L = math.hypot(acc_x, acc_y)
        normals.append((acc_x / L, acc_y / L) if L > 1e-12 else (0.0, 1.0))
    left = [(p[0] + n[0] * hw, p[1] + n[1] * hw) for p, n in zip(pts, normals)]
    right = [(p[0] - n[0] * hw, p[1] - n[1] * hw) for p, n in zip(pts, normals)]
    m = max(2, _round_join_steps(math.pi, hw))

    def cap(c, nx, ny):
        # From +n through the outward tangent to -n (n is the tangent turned
        # +90 deg, so decreasing angle passes the tangent first).
        a0 = math.atan2(ny, nx)
        return [(c[0] + hw * math.cos(a0 - math.pi * k / m),
                 c[1] + hw * math.sin(a0 - math.pi * k / m)) for k in range(1, m)]
    return (left + cap(pts[-1], *normals[-1]) + right[::-1]
            + cap(pts[0], -normals[0][0], -normals[0][1]))


def _custom_anchor_kind(pad_text: str) -> str:
    """'rect' or 'circle': a custom pad's anchor shape. KiCad's default anchor
    is a circle; the file writes `(options ... (anchor rect))` otherwise."""
    am = re.search(r'\(anchor\s+(\w+)\)', pad_text)
    return 'rect' if am and am.group(1) == 'rect' else 'circle'


def _custom_copper_is_anchor(pad_text: str, anchor_x: float, anchor_y: float,
                             eps: float = 1e-4) -> bool:
    """Is a custom pad's copper exactly its ANCHOR -- no primitive, or every
    primitive inside the anchor shape?

    Such a pad is its anchor rect (or circle), so it takes an ordinary pad's
    size and residual tilt (`_resolve_pad_rect`) rather than the board-frame
    box a custom pad with real primitives gets. awkb_36_split U1's 0.25 x 1
    anchors at 187.5 deg and len42_filter2 SW1.3's 4.572 x 0.406 anchor at 333
    deg carry `(primitives)` and nothing else. `build_pcb_data_from_board`
    asks the same question of pcbnew's pad-frame merged copper. A primitive
    this parser does not model (`gr_curve`) is never assumed inside."""
    pm = re.search(r'\(primitives\b', pad_text)
    if not pm:
        return True
    prim = pad_text[pm.start():find_matching_paren(pad_text, pm.start())]
    if re.search(r'\(gr_(?!(?:poly|circle|rect|line|arc)\b)\w', prim):
        return False
    hx, hy = anchor_x / 2.0, anchor_y / 2.0
    rect = _custom_anchor_kind(pad_text) == 'rect'
    r = min(hx, hy)
    for pts, hw in _custom_pad_primitive_points(pad_text):
        for vx, vy in pts:
            if rect:
                if abs(vx) + hw > hx + eps or abs(vy) + hw > hy + eps:
                    return False
            elif math.hypot(vx, vy) + hw > r + eps:
                return False
    return True


def _custom_pad_board_extent(pad_text: str, abs_rotation_deg: float,
                             anchor_x: float, anchor_y: float) -> Tuple[float, float]:
    """Board-frame half-extents (|x|, |y|) of a custom pad's copper about its
    anchor: primitive points (per-primitive stroke half-width) plus the ANCHOR
    SHAPE -- a rect's corners, a circle's radius -- all rotated by the pad's
    ABSOLUTE angle (KiCad negate convention). Matches build_pcb_data_from_board's
    symmetric box about the copper centre, so a 27deg/45deg custom pad models
    the same axis-aligned copper on both paths (sofle_pico D26/LED*,
    eurorack_pmod U1). A circle anchor used to count its bounding RECT's
    corners, 23 um past the copper on awkb_36_split SW16 at 345 deg. Returns
    (0, 0) with no primitives.
    """
    ext_x = ext_y = 0.0
    for bx, by, hw in _custom_pad_board_points(pad_text, abs_rotation_deg,
                                               anchor_x, anchor_y):
        ext_x = max(ext_x, abs(bx) + hw)
        ext_y = max(ext_y, abs(by) + hw)
    return ext_x, ext_y


def _custom_pad_board_points(pad_text: str, abs_rotation_deg: float,
                             anchor_x: float, anchor_y: float):
    """[(bx, by, hw)]: a custom pad's copper as board-frame points about its
    anchor, each with the half-width its copper reaches round it -- the
    primitives' points and strokes plus the anchor shape (a rect's corners, a
    circle's centre and radius), turned by the pad's ABSOLUTE angle (KiCad
    negate convention). [] with no primitives."""
    prims = _custom_pad_primitive_points(pad_text)
    if not prims:
        return []
    rad = math.radians(-abs_rotation_deg)
    cos_r, sin_r = math.cos(rad), math.sin(rad)
    hx, hy = anchor_x / 2.0, anchor_y / 2.0
    if _custom_anchor_kind(pad_text) == 'rect':
        prims = prims + [([(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)], 0.0)]
    else:
        # A centre point plus its radius: rotation-invariant, as the circle
        # primitives above. Same radius as the anchor polygon.
        prims = prims + [([(0.0, 0.0)], min(hx, hy))]
    return [(vx * cos_r - vy * sin_r, vx * sin_r + vy * cos_r, hw)
            for pts, hw in prims for vx, vy in pts]


#: mm the centre of a custom pad's copper must lie off its own centre before
#: the pad carries `Pad.copper_box`: far above the two parse paths' few-um
#: disagreement, so both decide alike, and a pad drawn about its anchor keeps
#: the box it always had.
COPPER_BOX_OFFSET_MM = 0.01


def copper_box_if_offset(x0: float, y0: float, x1: float, y1: float):
    """`(x0, y0, x1, y1)` -- a custom pad's copper box about its centre --
    when its centre is more than COPPER_BOX_OFFSET_MM off the pad's, else
    None (the pad's symmetric `size_x`/`size_y` box is then its copper's)."""
    if max(abs(x0 + x1), abs(y0 + y1)) / 2.0 > COPPER_BOX_OFFSET_MM:
        return (x0, y0, x1, y1)
    return None


def _custom_pad_board_box(pad_text: str, abs_rotation_deg: float,
                          anchor_x: float, anchor_y: float):
    """The board-frame box `(x0, y0, x1, y1)` of a custom pad's copper about
    its anchor, side by side -- `_custom_pad_board_extent` takes the farther
    side on each axis and centres a box twice that on the anchor -- or None
    with no primitives."""
    pts = _custom_pad_board_points(pad_text, abs_rotation_deg, anchor_x, anchor_y)
    if not pts:
        return None
    return (min(bx - hw for bx, _by, hw in pts), min(by - hw for _bx, by, hw in pts),
            max(bx + hw for bx, _by, hw in pts), max(by + hw for _bx, by, hw in pts))


def _adaptive_circle_n(r: float, eps: float = 0.001, lo: int = 16,
                       hi: int = 64) -> int:
    """Vertex count for an INSCRIBED polygon of radius ``r`` with radial
    (sagitta) error <= ``eps`` mm, clamped to [lo, hi].

    Run-4 B6: clearance grading happens against these polygons, so their
    error budget must sit BELOW the smallest shortfall worth flagging. Run
    3's tigard shipped a 0.0884mm-vs-0.09 graze (1.6um short) against a
    r=0.5 solder-jumper circle primitive whose fixed 32-gon carried 2.4um
    of sagitta -- the inscribed polygon under-reached the true copper by
    more than the defect, so check_drc graded it clean while kicad-cli
    (exact arcs) flagged it. eps=1um keeps that class visible at
    --clearance-margin 0; the cap bounds the 266-custom-pad boards.
    """
    if r <= eps:
        return lo
    try:
        n = int(math.ceil(math.pi / math.acos(1.0 - eps / r)))
    except ValueError:  # numeric edge: eps/r ~ 0
        return hi
    return max(lo, min(hi, n))


#: KiCad's chamfer corner names (`(chamfer top_left ...)`) in the pad's own
#: frame (y down), walked clockwise on screen.
CHAMFER_CORNERS = (('top_left', -1, -1), ('top_right', 1, -1),
                   ('bottom_right', 1, 1), ('bottom_left', -1, 1))


def chamfered_rect_polygon(cx: float, cy: float, w: float, h: float,
                           abs_rotation_deg: float, corners, chamfer_ratio: float,
                           corner_radius: float) -> List[Tuple[float, float]]:
    """The copper outline of KiCad's chamfered rect pad, in board mm (#1450).

    KiCad writes a chamfered-rect pad (Eagle's octagon pads come through its
    importer this way) as `roundrect` plus `(chamfer <corners>)` and
    `(chamfer_ratio r)`. Its outline (TransformRoundChamferedRectToPolygon) is
    the `w` x `h` rect centred on the copper (`cx`, `cy`) with each named
    corner cut by a 45-degree flat `chamfer_ratio * min(w, h)` along each side,
    and every OTHER corner rounded by `corner_radius` (the pad's roundrect
    radius, `roundrect_rratio * min(w, h)`) -- a chamfered corner is never
    rounded. As in pcbnew, the radius is at most half the short side and the
    chamfer is not clamped (a ratio past 0.5 overlaps, there too).

    `corners` are names from CHAMFER_CORNERS in the pad's own frame, which
    KiCad already mirrors when it flips a footprint, so the file's names and
    the pad's ABSOLUTE angle (`abs_rotation_deg`, applied with KiCad's negate
    convention as for custom pads) place the outline. Arcs are inscribed with
    1 um of sagitta, below pcbnew's own 5 um max error."""
    hx, hy = w / 2.0, h / 2.0
    short = min(w, h)
    cut = max(0.0, chamfer_ratio) * short
    r = max(0.0, min(corner_radius, short / 2.0))
    named = set(corners)
    # Chords of a quarter arc with 1 um of sagitta each.
    n_arc = (min(64, max(2, math.ceil(
        (math.pi / 2.0) / (2.0 * math.acos(max(-1.0, 1.0 - 0.001 / r))))))
        if r > 0 else 0)
    local: List[Tuple[float, float]] = []
    k = len(CHAMFER_CORNERS)
    for i, (name, sx, sy) in enumerate(CHAMFER_CORNERS):
        ccx, ccy = sx * hx, sy * hy
        _pn, psx, psy = CHAMFER_CORNERS[(i - 1) % k]
        _nn, nsx, nsy = CHAMFER_CORNERS[(i + 1) % k]
        # Unit vectors from this corner along its incoming / outgoing side.
        ux_in, uy_in = (psx - sx) / 2.0, (psy - sy) / 2.0
        ux_out, uy_out = (nsx - sx) / 2.0, (nsy - sy) / 2.0
        if name in named:
            if cut > 0:
                local.append((ccx + ux_in * cut, ccy + uy_in * cut))
                local.append((ccx + ux_out * cut, ccy + uy_out * cut))
            else:
                local.append((ccx, ccy))
        elif r > 0:
            # The quarter arc about the inset corner, from the incoming side
            # to the outgoing one (+90 degrees on screen, every corner).
            acx, acy = ccx - sx * r, ccy - sy * r
            a0 = math.atan2(ccy + uy_in * r - acy, ccx + ux_in * r - acx)
            for j in range(n_arc + 1):
                a = a0 + (math.pi / 2.0) * j / n_arc
                local.append((acx + r * math.cos(a), acy + r * math.sin(a)))
        else:
            local.append((ccx, ccy))
    # Two cuts meeting mid-side (ratio 0.5) leave a repeated vertex.
    pts: List[Tuple[float, float]] = []
    for p in local:
        if not pts or math.hypot(p[0] - pts[-1][0], p[1] - pts[-1][1]) > 1e-9:
            pts.append(p)
    if len(pts) > 1 and math.hypot(pts[0][0] - pts[-1][0], pts[0][1] - pts[-1][1]) <= 1e-9:
        pts.pop()
    rad = math.radians(-abs_rotation_deg)
    c, s = math.cos(rad), math.sin(rad)
    return [(cx + x * c - y * s, cy + x * s + y * c) for x, y in pts]


_CHAMFER_RE = re.compile(r'\(chamfer\s+([^()]*)\)')


#: KiCad's default `chamfer_ratio`, and the `roundrect_rratio` its loader
#: gives a chamfered pad whose file names none (PADSTACK's defaults).
CHAMFER_RATIO_DEFAULT = 0.2
ROUNDRECT_RRATIO_DEFAULT = 0.25


def chamfered_pad_from_text(pad_text: str, pad_shape: str, w: float, h: float,
                            cx: float, cy: float, abs_rotation_deg: float):
    """`([outline], roundrect_rratio)` for a chamfered-rect pad block, else
    None (#1450).

    `w`/`h` are the pad-frame `(size ...)`, (`cx`, `cy`) the copper centre.
    KiCad loads a `rect` or `roundrect` pad naming a corner as its chamfered
    rect, whose other corners take the pad's `roundrect_rratio` (0.25 when the
    file has none, at most 0.5) -- the ratio returned, as pcbnew reports it. A
    per-layer padstack is left to its box (its layers may differ), and a pad
    naming no corner is not chamfered."""
    if pad_shape not in ('rect', 'roundrect') or re.search(r'\(padstack\s', pad_text):
        return None
    m = _CHAMFER_RE.search(pad_text)
    names = {n for n, _x, _y in CHAMFER_CORNERS}
    corners = [t for t in (m.group(1).split() if m else ()) if t in names]
    if not corners or w <= 0 or h <= 0:
        return None
    cm = re.search(r'\(chamfer_ratio\s+(%s)\)' % AT_NUM, pad_text)
    ratio = float(cm.group(1)) if cm else CHAMFER_RATIO_DEFAULT
    rm = re.search(r'\(roundrect_rratio\s+(%s)\)' % AT_NUM, pad_text)
    rratio = max(0.0, min(0.5, float(rm.group(1)) if rm else ROUNDRECT_RRATIO_DEFAULT))
    return ([chamfered_rect_polygon(cx, cy, w, h, abs_rotation_deg, corners,
                                    ratio, rratio * min(w, h))], rratio)


def _custom_pad_global_polygons(pad_text: str, global_x: float, global_y: float,
                                pad_abs_rotation_deg: float):
    """Real copper outline(s) of a custom pad in GLOBAL board mm (issue #188).

    A custom pad's exposed source/drain copper is a comb/finger ``gr_poly``
    primitive whose notches our bounding-box model fills in, boxing in the pads
    that sit in the channels. This returns the actual polygon(s) so the obstacle
    map and DRC can rasterize the true copper instead.

    Each primitive vertex is in the pad's local frame (relative to the pad anchor,
    which is at the pad's global position). The pad's primitives are oriented by
    the pad's ABSOLUTE board angle (the file's ``(at x y angle)`` already folds in
    the footprint rotation), applied with KiCad's negate convention -- same as
    local_to_global. Verified against pcbnew GetEffectivePolygon for bitaxe Q1/Q2.

    Handles gr_poly, gr_circle, gr_rect and gr_line primitives -- each becomes one
    polygon (their union is the real copper; the obstacle map and DRC already treat
    polygons as a list and OR/min over them). A round mounting/logo pad drawn as a
    gr_circle thus models its true disc instead of the (size) bounding box, killing
    the phantom over-block on the empty side of the anchor (issue #232).

    Returns a list of vertex lists, or None when the pad has no primitives, only
    sub-3-vertex shapes, or any gr_arc / gr_curve primitive -- those curved shapes
    are still left to the conservative bounding-box model rather than approximated."""
    pm = re.search(r'\(primitives\b', pad_text)
    if not pm:
        return None
    prim = pad_text[pm.start():find_matching_paren(pad_text, pm.start()) - 1]
    # Beziers aren't approximated yet -- fall back to the bbox for the whole pad.
    # (gr_arc strokes ARE handled below: linearized into a width band. sofle_pico's
    # rotated LED lens pads are drawn from gr_arc+gr_line strokes; bailing to the
    # bbox modelled ~10x more copper than exists and made phantom PAD-SEGMENT.)
    if re.search(r'\(gr_curve\b', prim):
        return None
    rad = math.radians(-pad_abs_rotation_deg)
    cos_r, sin_r = math.cos(rad), math.sin(rad)

    def to_global(local_pts):
        return [(global_x + (vx * cos_r - vy * sin_r),
                 global_y + (vx * sin_r + vy * cos_r)) for vx, vy in local_pts]

    def _field(block, name, n):
        m = re.search(r'\(' + name + r'\s+' + r'\s+'.join([r'(-?[\d.]+)'] * n) + r'\)', block)
        return tuple(map(float, m.groups())) if m else None

    def _width(block):
        m = re.search(r'\(width\s+(-?[\d.]+)\)', block)
        return float(m.group(1)) if m else 0.0

    polys = []
    # Iterate primitives in order; one polygon per primitive, transformed to global.
    for pm2 in re.finditer(r'\(gr_(poly|circle|rect|line|arc)\b', prim):
        kind = pm2.group(1)
        block = prim[pm2.start():find_matching_paren(prim, pm2.start()) - 1]
        local = None
        if kind == 'poly':
            # Walk the pts list in order; KiCad 7+ outlines mix (xy ...) with
            # (arc (start)(mid)(end)) entries -- and can be arcs ONLY
            # (thunderscope U11's rounded B0QFN pads). Linearize each arc in
            # place so the polygon follows the real curved outline.
            pts = []
            for m in re.finditer(r'\(xy\s+(-?[\d.]+)\s+(-?[\d.]+)\)|' + _PTS_ARC_RE, block):
                if m.group(1) is not None:
                    pts.append((float(m.group(1)), float(m.group(2))))
                else:
                    s = (float(m.group(3)), float(m.group(4)))
                    a_mid = (float(m.group(5)), float(m.group(6)))
                    e = (float(m.group(7)), float(m.group(8)))
                    segs = _arc_to_segments(s, a_mid, e)
                    pts.append(segs[0][0])
                    for _p1, p2 in segs:
                        pts.append(p2)
            local = pts if len(pts) >= 3 else None
            if local:
                # KiCad strokes the gr_poly OUTLINE with its (width ...) --
                # for filled and unfilled polys alike -- so copper reaches
                # width/2 beyond the outline (#418). Grow the outline by that
                # half-stroke with ROUND corners, as KiCad does. An unfilled
                # poly's interior is not really copper, but the filled-outline
                # model is kept (conservative, and exact for clearance to
                # external copper).
                hw = _width(block) / 2.0
                if hw > 0:
                    local = _offset_polygon_round(local, hw)
        elif kind == 'circle':
            c = _field(block, 'center', 2)
            e = _field(block, 'end', 2)
            if c and e:
                r_mid = math.hypot(e[0] - c[0], e[1] - c[1])
                hw = _width(block) / 2.0
                R = r_mid + hw
                # An UNFILLED gr_circle is a stroked RING: copper only in the
                # annulus [r_mid-hw, r_mid+hw]. The old solid-disc model
                # ("conservative, clearance to external copper is all that
                # matters") entombed anything living INSIDE the ring --
                # ottercast MK1: a bottom-port MEMS mic's signal pad sits in
                # the opening (the human connects it with a via there) and the
                # phantom disc made it unroutable even on an empty board.
                # Model the annulus as a single SEAMED polygon (outer loop +
                # radial seam + inner loop): even-odd crossing tests -- which
                # every consumer of pad.polygons uses -- see the opening as
                # outside, and the zero-width seam lies inside real copper.
                filled = not re.search(r'\(fill\s+(?:no|none)\b', block)
                r_in = 0.0 if filled else max(0.0, r_mid - hw)
                if R > 0:
                    # An inscribed 32-gon, made finer only where it would sit
                    # more than pcbnew's own 5 um max error inside the circle
                    # (R > 1.038 mm): there the two parse paths disagreed by
                    # up to 13 um (kirdy_laser_drv H1..H8's 2.75 mm rings,
                    # reterminal_em J6/J7). Every smaller circle keeps its
                    # 32-gon, so the geometry of most boards does not move
                    # (run-4 B6's 1 um adaptive version moved all of them).
                    N = _adaptive_circle_n(R, eps=0.005, lo=32, hi=128)
                    outer = [(c[0] + R * math.cos(2 * math.pi * k / N),
                              c[1] + R * math.sin(2 * math.pi * k / N))
                             for k in range(N)]
                    if r_in > 0.05:
                        inner = [(c[0] + r_in * math.cos(-2 * math.pi * k / N),
                                  c[1] + r_in * math.sin(-2 * math.pi * k / N))
                                 for k in range(N)]
                        local = outer + [outer[0], inner[0]] + inner[1:] + [inner[0]]
                    else:
                        local = outer
        elif kind == 'rect':
            s = _field(block, 'start', 2)
            e = _field(block, 'end', 2)
            if s and e:
                hw = _width(block) / 2.0
                x0, x1 = min(s[0], e[0]), max(s[0], e[0])
                y0, y1 = min(s[1], e[1]), max(s[1], e[1])
                # The stroke rounds the corners, as on a gr_poly, and so does
                # a KiCad 9 corner `(radius r)` (rp2350_fpga_eensy U5: r 0.05,
                # drawn sharp it reached 21 um past pcbnew's copper): the rect
                # inset by r, grown by r plus the half-stroke.
                rr = _rect_corner_radius(block, x1 - x0, y1 - y0)
                local = _offset_polygon_round(
                    [(x0 + rr, y0 + rr), (x1 - rr, y0 + rr), (x1 - rr, y1 - rr),
                     (x0 + rr, y1 - rr)], hw + rr)
        elif kind == 'arc':
            # 3-point arc stroke: linearize the centerline, offset into a band.
            # chord_eps: pad-primitive arcs get the same 1um chord tolerance
            # as primitive circles (run-4 B6) -- the default 16 segments has
            # millimetre-scale radius-dependent error on large arcs.
            am = re.search(_PTS_ARC_RE, block)
            if am:
                s = (float(am.group(1)), float(am.group(2)))
                a_mid = (float(am.group(3)), float(am.group(4)))
                e = (float(am.group(5)), float(am.group(6)))
                cpts = [s]
                for _p1, p2 in _arc_to_segments(s, a_mid, e, chord_eps=0.001):
                    cpts.append(p2)
                local = _stroke_band(cpts, max(_width(block) / 2.0, 1e-3))
        elif kind == 'line':
            s = _field(block, 'start', 2)
            e = _field(block, 'end', 2)
            if s and e:
                hw = _width(block) / 2.0 or 1e-6
                if math.hypot(e[0] - s[0], e[1] - s[1]) > 0:
                    # The capsule itself, round caps and all.
                    local = _stroke_band([s, e], hw)
        if local:
            polys.append(to_global(local))

    # A custom pad's copper is the ANCHOR SHAPE union the primitives (#337).
    # Modeling primitives alone under-covers: urchin's router drilled a via
    # inside D31.2's 0.95x2.29 anchor rect because only the small finger
    # primitive was in the polygon set, and check_drc graded the real contact
    # clean for the same reason. KiCad's default anchor is a circle; the file
    # writes (options ... (anchor rect)) when it is a rectangle.
    sm = re.search(r'\(size\s+(-?[\d.]+)\s+(-?[\d.]+)\)', pad_text)
    if sm:
        ax, ay = float(sm.group(1)), float(sm.group(2))
        if ax > 0 and ay > 0:
            am = re.search(r'\(anchor\s+(\w+)\)', pad_text)
            akind = am.group(1) if am else 'circle'
            if akind == 'rect':
                anchor_local = [(-ax / 2, -ay / 2), (ax / 2, -ay / 2),
                                (ax / 2, ay / 2), (-ax / 2, ay / 2)]
            else:
                r = min(ax, ay) / 2.0
                # Adaptive tessellation (#450): a fixed inscribed 16-gon has a
                # 38um radial error on a 4mm anchor (scalenode SP6: a via KiCad
                # measures 70um from the real circle read as 102um here, so a
                # sub-clearance graze graded clean). Stay INSCRIBED (the
                # polygon never exceeds the real copper -- no phantom grazes,
                # no router over-block) but pick the vertex count for a <=5um
                # sagitta.
                _eps = 0.005
                n = 16
                if r > _eps:
                    n = max(16, int(math.ceil(math.pi / math.acos(1.0 - _eps / r))))
                anchor_local = [(r * math.cos(2.0 * i * math.pi / n),
                                 r * math.sin(2.0 * i * math.pi / n)) for i in range(n)]
            polys.append(to_global(anchor_local))
    return polys or None


def pad_drill_capsule(pad) -> Tuple[Tuple[float, float], Tuple[float, float], float]:
    """A pad's drill hole as a capsule: (end1, end2, radius) in board mm.

    KiCad slot drills -- ``(drill oval w h)`` -- are milled slots, not round
    holes. Modelling one as a circle of its LONG dimension (the back-compat
    ``pad.drill`` scalar) turns a 30.2x1mm milled slot into a phantom 15mm-
    radius hole that "conflicts" with every track within reach (thunderscope:
    817 phantom copper-to-hole violations; KiCad's own DRC reports none).
    The capsule axis follows the pad's absolute rotation with the same negate
    convention as local_to_global. Round drills (and drills parsed before the
    slot fields existed) return a zero-length capsule at the pad centre.
    """
    w = getattr(pad, 'drill_w', 0.0) or 0.0
    h = getattr(pad, 'drill_h', 0.0) or 0.0
    # Offset pads (#325): the DRILL stays at the anchor (hole_x/hole_y) while
    # global_x/global_y is the copper centre.
    hx = getattr(pad, 'hole_x', None)
    hy = getattr(pad, 'hole_y', None)
    px = hx if hx is not None else pad.global_x
    py = hy if hy is not None else pad.global_y
    if w <= 0 or h <= 0 or abs(w - h) < 1e-9:
        centre = (px, py)
        return centre, centre, (pad.drill or max(w, h)) / 2.0
    radius = min(w, h) / 2.0
    half_span = (max(w, h) - min(w, h)) / 2.0
    # Local long-axis direction: +x when w >= h, else +y; rotate to board frame.
    rad = math.radians(-(getattr(pad, 'rotation', 0.0) or 0.0))
    cos_r, sin_r = math.cos(rad), math.sin(rad)
    if w >= h:
        ux, uy = cos_r, sin_r          # local (1, 0)
    else:
        ux, uy = -sin_r, cos_r         # local (0, 1)
    p1 = (px - ux * half_span, py - uy * half_span)
    p2 = (px + ux * half_span, py + uy * half_span)
    return p1, p2, radius


_REMOVE_UNUSED_RE = re.compile(r'\(remove_unused_layers(?:\s+(yes|no))?\s*\)')
_KEEP_END_RE = re.compile(r'\(keep_end_layers(?:\s+(yes|no))?\s*\)')

# One name in a `(layers ...)` list, either spelling. KiCad 7+ quotes every
# name, `(layers "*.Cu" "*.Mask")`; KiCad 6 writes a pad's list bare,
# `(layers *.Cu *.Mask)`. A quoted-only reader returns [] for every pad of a
# KiCad 6 file, so the board parses with no pad on any copper layer and every
# net fails at its first step -- seen on a KiCad 6 corpus source routed
# without the pcbnew round-trip that corpus prep always gives it.
_LAYER_LIST_TOKEN_RE = re.compile(r'"((?:[^"\\]|\\.)*)"|([^\s"()]+)')


def layer_list_tokens(body: str) -> List[str]:
    """The layer names in the BODY of a `(layers ...)` list -- the text after
    the keyword -- quoted or bare, in file order."""
    return [bare or quoted
            for quoted, bare in _LAYER_LIST_TOKEN_RE.findall(body)]


#: Every copper layer a KiCad layer set can hold, in the order the pcbnew
#: builder (`build_pcb_data_from_board.get_pad_layers`) lists them.
ALL_COPPER_LAYER_NAMES = ('F.Cu', 'B.Cu') + tuple('In%d.Cu' % i for i in range(1, 31))
#: KiCad's layer-SET tokens whose two-sided form the pcbnew builder spells
#: as a wildcard of its own, and the members it stands for.
_PAD_LAYER_PAIRS = {'*.Mask': ('F.Mask', 'B.Mask'), '*.Paste': ('F.Paste', 'B.Paste'),
                    '*.SilkS': ('F.SilkS', 'B.SilkS')}
_PAD_LAYER_PAIR_OF = {m: w for w, ms in _PAD_LAYER_PAIRS.items() for m in ms}
#: The layer-set tokens KiCad's reader accepts that the pcbnew builder lists
#: member by member (pcbnew's `m_layerMasks`, probed on 10.0.0).
_PAD_LAYER_SETS = {'F&B.Cu': ('F.Cu', 'B.Cu'),
                   '*In.Cu': ALL_COPPER_LAYER_NAMES[2:],
                   '*.Adhes': ('F.Adhes', 'B.Adhes'),
                   '*.Fab': ('F.Fab', 'B.Fab'),
                   '*.CrtYd': ('F.CrtYd', 'B.CrtYd')}


def canonical_pad_layers(tokens: Sequence[str], board_copper: Sequence[str] = (),
                         plated_through: bool = False) -> List[str]:
    """A pad's `(layers ...)` tokens as the layer SET pcbnew loads, spelled the
    way the pcbnew builder lists it, so `Pad.layers` is the same set on both
    parse paths.

    - The set tokens expand: `F&B.Cu` -> F.Cu + B.Cu, `*In.Cu` -> In1..In30,
      `*.Adhes` / `*.Fab` / `*.CrtYd` -> their F/B members. `*.Mask`,
      `*.Paste` and `*.SilkS` stay wildcards, and two explicit members of one
      of them collapse to it; every copper layer KiCad has collapses to `*.Cu`,
      and a name `*.Cu` already covers is dropped (glasgow J10 lists `*.Cu`
      and In3..In30 both).
    - A PLATED through-hole pad also takes every copper layer of the board
      (`board_copper`): pcbnew's loader puts a PTH pad on all of them whatever
      the file lists (probed on 10.0.0: `(layers "B.Cu" "B.Mask")` on a
      4-layer board loads as F.Cu, In1.Cu, In2.Cu, B.Cu, B.Mask). An NPTH pad
      keeps what it declares.

    First-appearance order is kept, so a pad that names none of these tokens
    keeps its list exactly as the file spells it."""
    names: List[str] = []
    for tok in tokens:
        if tok == '*.Cu':
            names.extend(ALL_COPPER_LAYER_NAMES)
        else:
            names.extend(_PAD_LAYER_SETS.get(tok) or _PAD_LAYER_PAIRS.get(tok) or (tok,))
    if plated_through:
        names.extend(board_copper)
    have = set(names)
    all_copper = have.issuperset(ALL_COPPER_LAYER_NAMES)
    out: List[str] = []
    for n in names:
        if all_copper and n in ALL_COPPER_LAYER_NAMES:
            n = '*.Cu'
        elif n in _PAD_LAYER_PAIR_OF and have.issuperset(
                _PAD_LAYER_PAIRS[_PAD_LAYER_PAIR_OF[n]]):
            n = _PAD_LAYER_PAIR_OF[n]
        if n not in out:
            out.append(n)
    return out


def map_layer_list_tokens(body: str, fn) -> str:
    """BODY with every layer name replaced by ``fn(name)``, each keeping the
    spelling it had: a quoted name stays quoted, a bare one stays bare."""
    def _sub(m):
        if m.group(2) is not None:
            return fn(m.group(2))
        return '"' + fn(m.group(1)) + '"'
    return _LAYER_LIST_TOKEN_RE.sub(_sub, body)


def unconnected_layer_mode_from_text(pad_text: str) -> str:
    """A pad block's unconnected-layer mode (see Pad.unconnected_layer_mode).

    KiCad 8+ writes `(remove_unused_layers yes|no)` and `(keep_end_layers
    yes|no)`; KiCad 7 wrote bare flags, which mean yes. Probed on pcbnew
    10.0.0: KEEP_ALL saves `no`, REMOVE_EXCEPT_START_AND_END saves `yes` +
    `(keep_end_layers yes)`, and REMOVE_ALL and START_END_ONLY both save `yes`
    + `(keep_end_layers no)`, so the file reads either of those as remove_all.
    """
    m = _REMOVE_UNUSED_RE.search(pad_text)
    if not m or m.group(1) == 'no':
        return 'keep_all'
    k = _KEEP_END_RE.search(pad_text)
    if k and k.group(1) != 'no':
        return 'remove_except_start_end'
    return 'remove_all'


def unconnected_layer_mode_from_pcbnew(pad) -> str:
    """The pcbnew twin of unconnected_layer_mode_from_text. START_END_ONLY
    maps to remove_all, which is what the same board reads as once saved."""
    try:
        import pcbnew
        mode = pad.Padstack().UnconnectedLayerMode()
        if mode == pcbnew.UNCONNECTED_LAYER_MODE_KEEP_ALL:
            return 'keep_all'
        if mode == pcbnew.UNCONNECTED_LAYER_MODE_REMOVE_EXCEPT_START_AND_END:
            return 'remove_except_start_end'
        return 'remove_all'
    except Exception:
        pass
    try:  # KiCad < 9: no padstack object
        if not pad.GetRemoveUnconnected():
            return 'keep_all'
        return ('remove_except_start_end' if pad.GetKeepTopBottom()
                else 'remove_all')
    except Exception:
        return 'keep_all'


_DIE_LENGTH_RE = re.compile(r'\(die_length\s+(-?[\d.]+(?:[eE][-+]?\d+)?)\s*\)')
_DIE_DELAY_RE = re.compile(r'\(die_delay\s+(-?[\d.]+(?:[eE][-+]?\d+)?)\s*\)')


def pad_to_die_from_text(pad_text: str) -> Tuple[float, float]:
    """A pad block's (die_length mm, die_delay ps); (0.0, 0.0) when unset (#1248).

    KiCad writes each only when non-zero (pcb_io_kicad_sexpr). `die_delay` is
    in ps in every file version: read up to 20250926 as board units and after
    it as TIME units, both scale the file value by 1e6 IU, and pcbIUScale's
    IU_PER_PS is 1e6 (probed on pcbnew 10.0.0: SetPadToDieDelay(12345) saves
    `(die_delay 0.012345)`).
    """
    m = _DIE_LENGTH_RE.search(pad_text)
    d = _DIE_DELAY_RE.search(pad_text)
    return (float(m.group(1)) if m else 0.0, float(d.group(1)) if d else 0.0)


def pad_to_die_from_pcbnew(pad) -> Tuple[float, float]:
    """The pcbnew twin of pad_to_die_from_text: (mm, ps). KiCad 9 has no
    delay getter, so its delay reads 0.0, as that KiCad's file would."""
    try:
        length = pad.GetPadToDieLength() / 1e6
    except Exception:
        length = 0.0
    try:
        import pcbnew
        per_ps = float(getattr(getattr(pcbnew, 'pcbIUScale', None),
                               'IU_PER_PS', 1e6))
        delay = pad.GetPadToDieDelay() / per_ps
    except Exception:
        delay = 0.0
    return length, delay


def _pcbnew_custom_copper_is_anchor(pad, to_mm, eps: float = 1e-4) -> bool:
    """`_custom_copper_is_anchor` for a live pcbnew custom pad: every vertex of
    its PAD-FRAME merged copper (`MergePrimitivesAsPolygon`: anchor plus
    primitives, about the anchor, orientation 0) lies inside the anchor shape.
    The farthest point of a polygon from a convex shape is a vertex, so this
    is the text path's test asked of pcbnew's own copper. False when pcbnew
    cannot say."""
    try:
        import pcbnew as _pn
        layer = pad.GetLayer()
        sps = _pn.SHAPE_POLY_SET()
        pad.MergePrimitivesAsPolygon(layer, sps)
        try:
            a = pad.GetSize(layer)
            circle = pad.GetAnchorPadShape(layer) == getattr(_pn, 'PAD_SHAPE_CIRCLE', 0)
        except TypeError:       # pre-padstack API: no layer argument
            a = pad.GetSize()
            circle = pad.GetAnchorPadShape() == getattr(_pn, 'PAD_SHAPE_CIRCLE', 0)
        hx, hy = to_mm(a.x) / 2.0, to_mm(a.y) / 2.0
        r = min(hx, hy)
        n = 0
        for oi in range(sps.OutlineCount()):
            ol = sps.Outline(oi)
            for i in range(ol.PointCount()):
                x, y = to_mm(ol.CPoint(i).x), to_mm(ol.CPoint(i).y)
                n += 1
                if circle:
                    if math.hypot(x, y) > r + eps:
                        return False
                elif abs(x) > hx + eps or abs(y) > hy + eps:
                    return False
        return n > 0
    except Exception:
        return False


def _pcbnew_chamfered_pad_polygons(pad, to_mm):
    """`[outline]` in board mm for a live pcbnew chamfered-rect pad, else None
    (#1450) -- `chamfered_pad_from_text` for the GUI path.

    The outline is pcbnew's effective polygon, which tests/
    test_1450_chamfered_pad_pcbnew.py holds the text path's
    chamfered_rect_polygon to. Should pcbnew give none, the pad keeps its box
    (as a custom pad does). A per-layer padstack is left to its box, as on the
    text path."""
    try:
        import pcbnew as _pn
        if pad.GetShape() != getattr(_pn, 'PAD_SHAPE_CHAMFERED_RECT', object()):
            return None
        if not pad.GetChamferPositions():
            return None
        if hasattr(pad, 'Padstack') and pad.Padstack().Mode() != _pn.PADSTACK.MODE_NORMAL:
            return None
    except Exception:
        return None
    try:
        poly_set = pad.GetEffectivePolygon(pad.GetLayer())
        polys = []
        for oi in range(poly_set.OutlineCount()):
            ol = poly_set.Outline(oi)
            pts = [(to_mm(ol.CPoint(i).x), to_mm(ol.CPoint(i).y))
                   for i in range(ol.PointCount())]
            if len(pts) >= 3:
                polys.append(pts)
        return polys or None
    except Exception:
        return None


def pad_is_plated_through(pad) -> bool:
    """True if the pad's hole has a PLATED copper barrel tying every copper
    layer together (a real through-hole pin/via-in-pad). `drill > 0` alone is
    NOT enough: an NPTH pad (`pad_type == 'np_thru_hole'`) has a hole but NO
    copper, so treating it as an all-layer connection point invents copper
    that isn't there (issue #328 -- a net-tied mounting hole would seed a
    plane / offer launch layers on nothing). Use this instead of bare
    `pad.drill > 0` wherever the question is "does this pad connect layers"."""
    return ((getattr(pad, 'drill', 0.0) or 0.0) > 0
            and getattr(pad, 'pad_type', '') != 'np_thru_hole')


def pad_is_aperture_only(pad) -> bool:
    """True for a pad that is ONLY an aperture: not NPTH, no drill, and no
    copper layer -- a paste or mask window, e.g. a thermal pad's split paste
    windows or a spacer's paste ring (#1143).

    Such a pad is not copper, not a hole and not a pin, so a measure that asks
    "where are this part's pads" or "does this part have pads" must not read it.
    NPTH and drilled pads are deliberately NOT aperture-only: a mounting hole is
    physical extent, and dropping it moves splitflap's H6/H7 rects and the
    #837 assembly census. Use `pad_has_copper` instead where the question is
    copper (it is False for NPTH too).

    A pad object with no `layers` attribute at all is NOT an aperture: nothing
    says it is one. Both parsers always fill `layers`, but placement code builds
    position-only stand-ins (`arrays.pose_free_chip_refs`' SimpleNamespace
    pads, test stubs), and reading those as apertures dropped every pad of a
    1xN header from the row test, so headers became "chips" (Phase-1
    verifier: 13 of 22 corpus boards)."""
    if getattr(pad, 'pad_type', '') == 'np_thru_hole':
        return False
    if (getattr(pad, 'drill', 0.0) or 0.0) > 0:
        return False
    if getattr(pad, 'layers', None) is None:
        return False
    return not pad_has_copper(pad)


def non_aperture_pads(fp) -> list:
    """`fp.pads` without its aperture-only pads (`pad_is_aperture_only`): the
    pads a placement measure of extent, centre, pitch or "has pads" reads."""
    return [p for p in (getattr(fp, 'pads', None) or ())
            if not pad_is_aperture_only(p)]


def pad_drill_circles(pad, step: float = 0.0) -> List[Tuple[float, float, float]]:
    """A pad's drill hole as (x, y, diameter) circles for circle-based hole
    keep-outs (obstacle maps, via hole-to-hole tests).

    Round drills yield the single (x, y, drill) circle -- identical to the old
    behaviour. Slot drills yield circles of the slot's SHORT dimension sampled
    along the capsule axis (default spacing radius/2, max union sag ~3%% of the
    radius), so the keep-out follows the real slot instead of a long-axis disc.
    """
    p1, p2, r = pad_drill_capsule(pad)
    if r <= 0:
        return []
    dx, dy = p2[0] - p1[0], p2[1] - p1[1]
    span = math.hypot(dx, dy)
    if span < 1e-9:
        return [(p1[0], p1[1], 2.0 * r)]
    if step <= 0:
        step = r / 2.0
    n = max(1, int(math.ceil(span / step)))
    return [(p1[0] + dx * (i / n), p1[1] + dy * (i / n), 2.0 * r)
            for i in range(n + 1)]


def _padstack_copper_size(pad_text: str, base: Tuple[float, float],
                          pad_layers: Sequence[str],
                          board_copper: Sequence[str]) -> Tuple[float, float]:
    """A pad's size (pad frame) as the per-axis MAX over the copper layers it
    is on, each read from its KiCad 9 `(padstack ...)` entry.

    The base `(size ...)` is the F.Cu geometry. `front_inner_back` adds an
    `"Inner"` and a `"B.Cu"` entry; `custom` one entry per named layer; a
    layer with no entry keeps the base. Only the pad's own copper layers
    within the board's copper count -- a padstack entry for a layer the pad is
    not on describes no copper. `build_pcb_data_from_board` takes the same
    max over `PAD.GetSize(layer)` for the same layers."""
    m = re.search(r'\(padstack\b', pad_text)
    if not m:
        return base
    blk = pad_text[m.start():find_matching_paren(pad_text, m.start())]
    mm = re.search(r'\(mode\s+(\w+)\)', blk)
    mode = mm.group(1) if mm else 'normal'
    per = {}
    for lm in re.finditer(r'\(layer\s+"([^"]+)"', blk):
        lb = blk[lm.start():find_matching_paren(blk, lm.start())]
        sm = re.search(r'\(size\s+(%s)\s+(%s)\)' % (AT_NUM, AT_NUM), lb)
        if sm:
            per[lm.group(1)] = (float(sm.group(1)), float(sm.group(2)))
    on = [ln for ln in board_copper if '*.Cu' in pad_layers or ln in pad_layers]
    if not on:
        return base

    def size_on(ln):
        if ln == 'F.Cu' or mode not in ('front_inner_back', 'custom'):
            return base
        if mode == 'front_inner_back':
            return per.get('B.Cu' if ln == 'B.Cu' else 'Inner', base)
        return per.get(ln, base)
    got = [size_on(ln) for ln in on]
    return max(s[0] for s in got), max(s[1] for s in got)


def _resolve_pad_rect(size_x: float, size_y: float,
                      pad_rotation_deg: float) -> Tuple[float, float, float]:
    """Resolve a pad rectangle (given in the pad's own frame) into board space.

    pad_rotation_deg is the pad's ABSOLUTE orientation: the KiCad pad ``(at ...)``
    angle already includes the footprint rotation (verified physically -- using
    pad+footprint here makes adjacent pad copper overlap on rotated parts), and
    pcbnew's GetOrientationDegrees() is likewise absolute. The footprint rotation
    still applies to the pad POSITION (see local_to_global) but NOT a second time
    to its orientation.

    Returns (size_x, size_y, rect_rotation): board-axis-aligned dimensions for
    orthogonal pads (0/90/180/270°, the overwhelmingly common case) with
    rect_rotation 0; for genuine diagonal pads the dimensions are left as given
    and rect_rotation carries the residual tilt in the GLOBAL frame, folded to
    (-90, 90]. The global frame negates the KiCad angle (see local_to_global), so
    the geometric tilt is -pad_rotation.
    """
    t = pad_rotation_deg % 180.0  # rect is symmetric under 180°
    if abs(t - 90.0) <= _PAD_ORTHO_TOL:
        return size_y, size_x, 0.0           # ~90°: swap to board axes
    if t <= _PAD_ORTHO_TOL or t >= 180.0 - _PAD_ORTHO_TOL:
        return size_x, size_y, 0.0           # ~0/180°: already board-aligned
    g = (-pad_rotation_deg) % 180.0          # global-frame tilt, folded to [0,180)
    rect_rotation = g if g <= 90.0 else g - 180.0
    return size_x, size_y, rect_rotation


# A quoted KiCad S-expr string body, honoring backslash escapes. `[^"]*` is
# the wrong spelling everywhere a NET NAME is captured: it ends at the first
# escaped quote, so the enclosing token stops matching and the object is read
# with the wrong net -- or, for segments and arcs, is not read AT ALL.
_ESC_STR = r'((?:[^"\\]|\\.)*)'


def _unescape_kicad_string(s: str) -> str:
    """Undo KiCad s-expression string escapes (backslash and quote).

    The file stores net names like ROT_RDY\\ROT_GPIO0 with the backslash
    escaped; pcbnew returns the unescaped name, so keeping the raw text made
    every such net name diff between the two parsers. Applied to DISPLAY
    names only -- name_to_id lookups key on the raw file text, which every
    in-file reference shares.
    """
    if '\\' not in s:
        return s
    return s.replace('\\\\', '\\').replace('\\"', '"')


#: #1211. A footprint's schematic sheet, both spellings: KiCad 8+ header
#: tokens `(sheetname "...")` / `(sheetfile "...")`, KiCad 6/7 properties
#: `(property "Sheetname" "...")` / `(property "Sheetfile" "...")`.
_SHEET_FIELD_RE = {
    which: re.compile(r'\(%s\s+"((?:[^"\\]|\\.)*)"' % which.lower())
    for which in ('Sheetname', 'Sheetfile')}
#: The PROPERTY spellings pcbnew's reader honours: "Sheetname" and KiCad 6's
#: "Sheet name" (ecp5_mini, rfswitch01), and the same pair for the file.
_SHEET_PROPERTY_RE = {
    which: re.compile(r'\(property\s+"(?:%s|%s %s)"\s+"((?:[^"\\]|\\.)*)"'
                      % (which, which[:5], which[5:].lower()))
    for which in ('Sheetname', 'Sheetfile')}


def _footprint_sheet_fields(fp_text: str, version: int = 0) -> Tuple[str, str]:
    """`(sheet_name, sheet_file)` of one footprint block, as the file spells
    them ('' when absent). Read from the block header -- before the first
    `(pad ` -- so a pad or a graphic can never lend a block a sheet.

    The `(sheetname ...)` token counts at any version; the PROPERTY spellings
    only up to SHEET_PROPERTIES_MAX_VERSION, where pcbnew stops reading them.
    `version` 0 (unknown) reads them too."""
    first_pad = fp_text.find('(pad ')
    head = fp_text[:first_pad] if first_pad > 0 else fp_text
    legacy = not version or version <= SHEET_PROPERTIES_MAX_VERSION
    out = []
    for which in ('Sheetname', 'Sheetfile'):
        m = _SHEET_FIELD_RE[which].search(head)
        if m is None and legacy:
            m = _SHEET_PROPERTY_RE[which].search(head)
        out.append(_unescape_kicad_string(m.group(1)) if m else '')
    return out[0], out[1]


_PAREN_OR_QUOTE = re.compile(r'["()]')


#: A KiCad number as the file may spell it. KiCad writes ANGLES with a
#: significant-digit format (probed on pcbnew 10.0.0: `1e-14`,
#: `1.421085472e-14`), and this repo's writer uses `:.6g`, so both emit
#: exponent form for |angle| < 1e-4.
AT_NUM = r'[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?'
AT_NODE_RE = re.compile(r'\(\s*at\s+(' + AT_NUM + r')\s+(' + AT_NUM + r')'
                        r'(?:\s+(' + AT_NUM + r'))?\s*\)')


def _footprint_at_start(fp_text: str) -> int:
    """Index of the footprint's OWN `(at` -- its direct child -- or -1.

    String-aware (a `(descr "(at 9 9)")` is not markup) and depth-scoped (a
    property's or pad's `(at ...)` is not the footprint's, whatever order the
    file lists them in).
    """
    depth = 0
    i = 0
    search = _PAREN_OR_QUOTE.search
    find_quote = fp_text.find
    while True:
        m = search(fp_text, i)
        if m is None:
            return -1
        j = m.start()
        c = fp_text[j]
        if c == '(':
            depth += 1
            if depth == 2:
                # `( at ...)` is the same node to KiCad's reader.
                k = j + 1
                while fp_text[k:k + 1] in (' ', '\t', '\n', '\r'):
                    k += 1
                if (fp_text.startswith('at', k)
                        and fp_text[k + 2:k + 3] in (' ', '\t', '\n', '\r')):
                    return j
            i = j + 1
        elif c == ')':
            depth -= 1
            if depth == 0:
                return -1
            i = j + 1
        else:
            i = j + 1
            while True:
                k = find_quote('"', i)
                if k == -1:
                    return -1
                b = k - 1
                while b >= i and fp_text[b] == '\\':
                    b -= 1
                i = k + 1
                if (k - 1 - b) % 2 == 0:
                    break


def footprint_at_match(fp_text: str):
    """The footprint's OWN `(at ...)` as a re.Match, or None.

    Groups 1..3 are x, y and the optional angle; `.start()`/`.end()` are the
    node's span inside `fp_text`, which is what a writer needs to rewrite it.
    None when the block has no `(at ...)` of its own or it does not parse.

    Two ways the textually-first `(at` was wrong, both fixed here: a child's
    `(at ...)` placed before the footprint's own was read as the footprint's
    pose, and an angle in exponent form (`1e-05`, which the writer's `:.6g`
    emits and pcbnew 10 writes as `1e-14`) did not match the old digits, dots
    and minus class at all, so the FIRST CHILD's `(at ...)` was read instead
    -- or the footprint was dropped with its pads.
    """
    j = _footprint_at_start(fp_text)
    return None if j < 0 else AT_NODE_RE.match(fp_text, j)


def find_matching_paren(content: str, open_idx: int) -> int:
    """Return the index just past the ``)`` matching the ``(`` at ``open_idx``.

    Counts parentheses but SKIPS those inside quoted strings, honoring KiCad's
    backslash escapes (``\\"`` and ``\\\\``). A naive depth counter breaks when a
    property value contains a lone paren, e.g. an MPN like ``"TCR2EF115,LM(CT"``:
    the unmatched ``(`` makes the scan run far past the block end and swallow
    following footprints' pads (issue #113). Returns ``len(content)`` if no match
    is found.
    """
    # Hot path for every block extraction: instead of a per-character state
    # machine, jump between the only characters that matter (quote/parens)
    # with C-level scans. Semantics are identical to the classic loop: a
    # quote toggles string mode, and inside a string a quote only closes it
    # when preceded by an even run of backslashes (\" and \\ escapes).
    depth = 0
    i = open_idx
    n = len(content)
    search = _PAREN_OR_QUOTE.search
    find_quote = content.find
    while True:
        m = search(content, i)
        if m is None:
            return n
        j = m.start()
        c = content[j]
        if c == '(':
            depth += 1
            i = j + 1
        elif c == ')':
            depth -= 1
            if depth == 0:
                return j + 1
            i = j + 1
        else:  # opening quote: skip the whole string
            i = j + 1
            while True:
                k = find_quote('"', i)
                if k == -1:
                    return n  # unterminated string: no match, like the old loop
                nb = 0
                while k - 1 - nb >= 0 and content[k - 1 - nb] == '\\':
                    nb += 1
                if nb % 2 == 0:
                    i = k + 1
                    break
                i = k + 1


def extract_layers(content: str) -> BoardInfo:
    """Extract layer information from PCB file."""
    layers, copper_layers = _layer_table(content)

    # Extract board bounds from Edge.Cuts
    bounds = extract_board_bounds(content)

    # Extract board outline polygon(s) and cutouts for non-rectangular boards
    outers, cutouts = extract_board_contours(content)

    # Extract stackup information
    stackup = extract_stackup(content)

    return BoardInfo(layers=layers, copper_layers=copper_layers, board_bounds=bounds,
                     stackup=stackup, board_outline=(outers[0] if outers else []),
                     board_outlines=outers, board_cutouts=cutouts)


def _layer_table(content: str) -> Tuple[Dict[int, str], List[str]]:
    """The board's (layers ...) table: {id: name}, and its copper layer names
    in table order."""
    layers = {}
    copper_layers = []

    # Find the layers section. Capture lazily up to the block's closing paren on
    # its own line, rather than requiring a run of paren-free (...) entries: layer
    # user-names can contain parentheses (e.g. "In1(GND).Cu", "plate (unused)"),
    # which truncated the old regex and made such boards parse as 0 copper layers.
    layers_match = re.search(r'\(layers\b(.*?)\n[ \t]*\)', content, re.DOTALL)
    if layers_match:
        layers_text = layers_match.group(1)
        # Parse individual layer entries: (0 "F.Cu" signal)
        layer_pattern = r'\((\d+)\s+"([^"]+)"\s+(\w+)'
        for m in re.finditer(layer_pattern, layers_text):
            layer_id = int(m.group(1))
            layer_name = m.group(2)
            layer_type = m.group(3)
            layers[layer_id] = layer_name
            # Any copper layer counts, whatever its declared use. KiCad types
            # plane layers 'power' (and sometimes 'mixed'/'jumper'); restricting
            # to 'signal' dropped real layers, making a 4-layer board look
            # 2-layer (issue #76).
            if '.Cu' in layer_name and layer_type in ('signal', 'power', 'mixed', 'jumper'):
                copper_layers.append(layer_name)
    return layers, copper_layers


def _stackup_block_span(content: str) -> Optional[Tuple[int, int]]:
    """``(start, end)`` of the ``(stackup ...)`` block, paren-balanced, or None
    when the content has none. ``end`` is ``len(content)`` when the block does
    not close inside ``content`` (a truncated head of the file)."""
    m = re.search(r'\(stackup(?=[\s)])', content)
    if not m:
        return None
    return m.start(), find_matching_paren(content, m.start())


def _stackup_number(text: str, token: str) -> float:
    """The number of ``(token N)`` in ``text``, else 0.0. A dielectric whose
    thickness the user locked is written ``(thickness 0.2104 locked)`` (#1222);
    reading only the bare form parsed every locked dielectric as 0 mm."""
    m = re.search(r'\(' + token + r'\s+([-+\d.eE]+)(?:\s+locked)?\s*\)', text)
    if not m:
        return 0.0
    try:
        return float(m.group(1))
    except ValueError:
        return 0.0


def _combine_stackup_sheets(sheets: List[Tuple[float, float, float, str]]
                            ) -> Tuple[float, float, float, str]:
    """One ``(thickness, epsilon_r, loss_tangent, material)`` for a stackup
    layer built from several sheets (#1222).

    KiCad writes a core/prepreg made of several sheets (JLCPCB's 3 x 2116) as
    ONE layer whose sheets are joined by ``addsublayer``; a consumer that sees
    one `StackupLayer` per dielectric needs the whole layer. Thickness is the
    sum. epsilon_r is the SERIES value ``sum(t) / sum(t / er)``: the field of a
    trace crosses the sheets one after another, which is what the impedance
    model prices. The loss tangent is weighted the same way, by ``t / er``.

    Only DECLARED values are averaged. KiCad starts a sheet added with
    ``addsublayer`` at epsilon_r 1.0 and loss tangent 0 (``DIELECTRIC_PRMS``'s
    defaults; the layer's first sheet starts at FR4's 4.5 / 0.02) and writes
    them as if they were values, so ``(epsilon_r 1)`` on a sheet named "2116"
    is a field nobody filled in, not vacuum (stm32h7_hdmi: averaging it in
    gives 2.3 for a 2116/7628/2116 prepreg). A sheet with epsilon_r <= 1, or
    none, sits out of the epsilon_r average -- the same as taking the declared
    sheets' value for it -- and a sheet with no positive loss tangent sits out
    of that one. The material is the sheets' common name, or every sheet's
    name joined by ``" + "`` when they differ. A single sheet comes back
    exactly as read.
    """
    if len(sheets) == 1:
        return sheets[0]
    thickness = sum(s[0] for s in sheets)
    rated = [s for s in sheets if s[0] > 0 and s[1] > 1.0]
    if rated:
        epsilon_r = sum(s[0] for s in rated) / sum(s[0] / s[1] for s in rated)
    else:
        epsilon_r = next((s[1] for s in sheets if s[1] > 0), 0.0)
    lossy = [s for s in sheets if s[0] > 0 and s[2] > 0]
    if lossy:
        weights = [s[0] / (s[1] if s[1] > 1.0 else max(epsilon_r, 1.0))
                   for s in lossy]
        loss_tangent = sum(w * s[2] for w, s in zip(weights, lossy)) / sum(weights)
    else:
        loss_tangent = next((s[2] for s in sheets if s[2] > 0), 0.0)
    names = [s[3] for s in sheets if s[3]]
    material = names[0] if len(set(names)) == 1 else " + ".join(names)
    return thickness, epsilon_r, loss_tangent, material


def extract_stackup(content: str) -> List[StackupLayer]:
    """Extract board stackup information for impedance calculation and via barrel length.

    Extracts copper and dielectric layers with their electrical properties:
    - thickness: layer thickness in mm
    - epsilon_r: dielectric constant (for dielectric layers)
    - loss_tangent: loss tangent (for dielectric layers)
    - material: material name

    A layer made of several sheets (``addsublayer``) is combined into one
    entry by `_combine_stackup_sheets` (#1222).
    """
    stackup = []

    span = _stackup_block_span(content)
    if span is None:
        return stackup
    stackup_text = content[span[0]:span[1]]

    # Each top-level (layer "name" ...) child, paren-balanced: the block holds
    # every sheet of the layer, and its end does not depend on what follows.
    layer_re = re.compile(r'\(layer\s+"([^"]+)"')
    pos = 0
    while True:
        m = layer_re.search(stackup_text, pos)
        if not m:
            break
        end = find_matching_paren(stackup_text, m.start())
        layer_name, layer_content = m.group(1), stackup_text[m.end():end]
        pos = end

        type_match = re.search(r'\(type\s+"([^"]+)"\)', layer_content)
        layer_type = type_match.group(1) if type_match else 'unknown'

        # Only include copper and dielectric layers (skip mask, silk, paste)
        if layer_type not in ('copper', 'core', 'prepreg'):
            continue

        sheets = []
        for sheet in re.split(r'\baddsublayer\b', layer_content):
            material_match = re.search(r'\(material\s+"([^"]+)"\)', sheet)
            sheets.append((_stackup_number(sheet, 'thickness'),
                           _stackup_number(sheet, 'epsilon_r'),
                           _stackup_number(sheet, 'loss_tangent'),
                           material_match.group(1) if material_match else ""))
        thickness, epsilon_r, loss_tangent, material = _combine_stackup_sheets(sheets)

        stackup.append(StackupLayer(
            name=layer_name,
            layer_type=layer_type,
            thickness=thickness,
            epsilon_r=epsilon_r,
            loss_tangent=loss_tangent,
            material=material
        ))

    return stackup


def _arc_to_segments(start: Tuple[float, float], mid: Tuple[float, float],
                     end: Tuple[float, float], num_segments: int = 16,
                     chord_eps: float = 0.005
                     ) -> List[Tuple[Tuple[float, float], Tuple[float, float]]]:
    """Convert a 3-point arc to a polyline of straight segments.

    Given start, mid (on arc), and end points, computes the circular arc
    and returns it as a list of (start, end) line segment tuples.
    """
    ax, ay = start
    bx, by = mid
    cx, cy = end

    # Find circle center from 3 points using perpendicular bisector intersection
    d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(d) < 1e-10:
        # Degenerate (collinear points) - just return a straight line
        return [(start, end)]

    ux = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay) + (cx * cx + cy * cy) * (ay - by)) / d
    uy = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx) + (cx * cx + cy * cy) * (bx - ax)) / d
    radius = math.hypot(ax - ux, ay - uy)

    # Compute angles
    angle_start = math.atan2(ay - uy, ax - ux)
    angle_mid = math.atan2(by - uy, bx - ux)
    angle_end = math.atan2(cy - uy, cx - ux)

    # Determine arc direction: going from start to end, mid must be on the arc
    # Normalize angles relative to start
    def normalize(a, ref):
        a = a - ref
        while a < 0:
            a += 2 * math.pi
        while a >= 2 * math.pi:
            a -= 2 * math.pi
        return a

    mid_ccw = normalize(angle_mid, angle_start)
    end_ccw = normalize(angle_end, angle_start)

    # If going CCW from start, mid should come before end
    if mid_ccw <= end_ccw:
        # CCW direction, sweep = end_ccw
        sweep = end_ccw
    else:
        # CW direction, sweep is negative
        sweep = end_ccw - 2 * math.pi

    # Adaptive resolution: cap the chord sag at chord_eps (default ~5um) so a
    # large-radius, wide-sweep arc (nebula_watch's ~350deg, r~20mm watch
    # outline) doesn't under-reach its true extremes -- 16 fixed segments left
    # the board outline 57um short of the real bounding box. Small arcs keep
    # the old cost; the count is capped to stay bounded on huge radii. Pad
    # PRIMITIVE arcs pass 0.001 (run-4 B6: clearance-grade geometry).
    if radius > 0:
        max_step = 2.0 * math.acos(max(-1.0, min(1.0, 1.0 - chord_eps / radius)))
        if max_step > 0:
            num_segments = max(num_segments,
                               min(512, int(math.ceil(abs(sweep) / max_step))))

    # Generate points along the arc, using exact start/end to preserve chaining
    points = [start]
    for i in range(1, num_segments):
        t = i / num_segments
        angle = angle_start + t * sweep
        points.append((ux + radius * math.cos(angle), uy + radius * math.sin(angle)))
    points.append(end)

    # Build segments
    segments = []
    for i in range(len(points) - 1):
        segments.append((points[i], points[i + 1]))
    return segments


_PTS_XY_OR_ARC_RE = re.compile(r'\(xy\s+(-?[\d.]+)\s+(-?[\d.]+)\)|' + _PTS_ARC_RE)


def _ring_without_repeats(pts):
    """Consecutive duplicate points dropped, and a last point equal to the
    first: an arc's end and the next element's start are written twice."""
    out = []
    for p in pts:
        if not out or p != out[-1]:
            out.append(p)
    if len(out) > 1 and out[0] == out[-1]:
        out.pop()
    return out


def poly_points_with_arcs(blk: str, transform) -> List[Tuple[float, float]]:
    """The vertices of a graphic's `(pts ...)` list in file order, each
    `(arc (start) (mid) (end))` entry linearized (`_arc_to_segments`) AFTER its
    three control points are transformed -- the order the pcbnew builder uses
    on the live chain's arcs (`_pcbnew_chain_points`), so both parse paths
    emit the same points. A KiCad 7+ polygon may be arcs only (apple1_mainboard's
    F.Cu gr_poly, zynq7020_som U9's F.Paste openings), which a reader of
    `(xy ...)` alone saw as no shape at all. A list with no arc is returned
    exactly as written."""
    if '(arc' not in blk:
        return [transform(float(x), float(y)) for x, y in
                re.findall(r'\(xy\s+([-\d.]+)\s+([-\d.]+)\)', blk)]
    pts = []
    for m in _PTS_XY_OR_ARC_RE.finditer(blk):
        if m.group(1) is not None:
            pts.append(transform(float(m.group(1)), float(m.group(2))))
            continue
        s = transform(float(m.group(3)), float(m.group(4)))
        mid = transform(float(m.group(5)), float(m.group(6)))
        e = transform(float(m.group(7)), float(m.group(8)))
        segs = _arc_to_segments(s, mid, e)
        pts.append(segs[0][0])
        pts.extend(p1 for _p0, p1 in segs)
    return _ring_without_repeats(pts)


def _pcbnew_chain_points(chain, to_mm) -> List[Tuple[float, float]]:
    """A live SHAPE_LINE_CHAIN's vertices, `poly_points_with_arcs`' twin: a
    chain with no arc is its points as pcbnew stores them; an arc in it is
    linearized by `_arc_to_segments` from its own start / mid / end, not taken
    from pcbnew's approximation, so the two parse paths emit the same
    points."""
    n = chain.PointCount()
    pts = [(to_mm(chain.CPoint(k).x), to_mm(chain.CPoint(k).y)) for k in range(n)]
    try:
        n_arcs = chain.ArcCount()
    except Exception:
        n_arcs = 0
    if not n_arcs:
        return pts
    arcs = []
    for i in range(n_arcs):
        a = chain.Arc(i)
        arcs.append(((to_mm(a.GetP0().x), to_mm(a.GetP0().y)),
                     (to_mm(a.GetArcMid().x), to_mm(a.GetArcMid().y)),
                     (to_mm(a.GetP1().x), to_mm(a.GetP1().y))))
    out, k, used = [], 0, set()
    while k < n:
        if chain.IsArcStart(k):
            hit = next((i for i, (s, _m, _e) in enumerate(arcs)
                        if i not in used and s == pts[k]), None)
            if hit is None:
                hit = next((i for i, (_s, _m, e) in enumerate(arcs)
                            if i not in used and e == pts[k]), None)
            if hit is not None:
                used.add(hit)
                s, mid, e = arcs[hit]
                if e == pts[k] and s != pts[k]:
                    s, e = e, s
                segs = _arc_to_segments(s, mid, e)
                out.append(segs[0][0])
                out.extend(p1 for _p0, p1 in segs)
                j = k + 1
                while j < n and not chain.IsArcEnd(j):
                    j += 1
                if j >= n:
                    break
                # A point two arcs share is the next arc's start too.
                k = j if chain.IsArcStart(j) else j + 1
                continue
        out.append(pts[k])
        k += 1
    return _ring_without_repeats(out)


# Regex gap that matches anything EXCEPT the start of another graphic element,
# so a lazy match can't run past the current element to a later (layer "...")
# token. A plain `.*?` gap let a silk/fab gr_* element match across element
# boundaries to a later Edge.Cuts token, consuming the real edge elements in
# between (issue #77). Shared by the Edge.Cuts and guide-corridor readers.
# ... and must not cross a (layer ...)/(layers ...) tag either: the first
# layer tag after an element's fields is that element's own layer, so allowing
# the gap to run past one lets a layer-less element (e.g. a pad-primitive
# gr_line) borrow a LATER element's layer tag and materialize as a phantom
# board graphic.
_GR_ELEMENT_GAP = r'(?:(?!\(gr_|\(layers?\b)[\s\S])*?'

# Shared by BOTH Edge.Cuts scans -- the bbox scan (extract_board_bounds) and the
# contour chainer (_collect_edge_cuts_segments). They must read the same
# primitive set: a shape only the chainer knows about yields an outline ring
# outside board_bounds, and a round board (one gr_circle) reported no bounds at
# all, so add_board_edge_obstacles stamped no keep-out whatsoever (#550).
def _curve_pattern(token: str) -> str:
    """Cubic-bezier Edge.Cuts pattern for ``gr_curve`` (board) / ``fp_curve``
    (footprint-embedded). One template so the board- and footprint-level scans
    cannot describe the same primitive differently."""
    return (r'\(' + token + r'\s+\(pts\s+'
            r'\(xy\s+([\d.-]+)\s+([\d.-]+)\)\s*\(xy\s+([\d.-]+)\s+([\d.-]+)\)\s*'
            r'\(xy\s+([\d.-]+)\s+([\d.-]+)\)\s*\(xy\s+([\d.-]+)\s+([\d.-]+)\)\s*\)'
            + _GR_ELEMENT_GAP + r'\(layer\s+"Edge\.Cuts"\)')


_GR_CURVE_PATTERN = _curve_pattern('gr_curve')
_FP_CURVE_PATTERN = _curve_pattern('fp_curve')
_GR_CIRCLE_PATTERN = (r'\(gr_circle\s+\(center\s+([\d.-]+)\s+([\d.-]+)\)\s+'
                      r'\(end\s+([\d.-]+)\s+([\d.-]+)\)' + _GR_ELEMENT_GAP
                      + r'\(layer\s+"Edge\.Cuts"\)')


#: Work one `parse_kicad_pcb` shares between its extractors, set only while a
#: parse runs (so a long-lived caller -- the GUI, the writer -- never
#: accumulates it) and emptied as the parse returns:
#:   * `live`: {id: text} of the parse's own texts (the board text and its
#:     `_mask_pad_primitives` copy), the only texts the entries below are kept
#:     for -- an id is reused only after its object is freed, and these stay
#:     alive for the whole parse;
#:   * `masked_from`: {id(masked copy): board text};
#:   * `ends`: {(id(text), open index): `find_matching_paren`'s answer};
#:   * `shapes`: {(id(text), block start, tags): each fp shape's (tag, start,
#:     end) inside that footprint block}.
#: On a 54 MB board (ddr4_rdimm_tester) the footprint blocks were scanned
#: three times and every footprint's shapes twice: 4.8 s of an 8.5 s parse.
_PARSE_MEMO: Optional[dict] = None


def _memo_for(content: str) -> Optional[dict]:
    memo = _PARSE_MEMO
    if memo is not None and memo['live'].get(id(content)) is content:
        return memo
    return None


def _block_end(content: str, start: int) -> int:
    """`find_matching_paren(content, start)`, answered once per parse for the
    parse's own texts: every footprint-block scan opens the same blocks."""
    memo = _memo_for(content)
    if memo is None:
        return find_matching_paren(content, start)
    key = (id(content), start)
    end = memo['ends'].get(key)
    if end is None:
        end = memo['ends'][key] = find_matching_paren(content, start)
    return end


@functools.lru_cache(maxsize=2)
def _mask_pad_primitives(content: str) -> str:
    """Blank out pad ``(primitives ...)`` blocks before board-level graphic scans.

    Memoised (#962): a pure function of the text, and one parse now calls it
    from both `extract_segments` and `extract_paste_graphics`.

    Custom pads draw their copper with gr_line/gr_arc/gr_poly PRIMITIVES. The
    board-level gr_* scanners (Edge.Cuts bounds/outline, guide corridors,
    keepouts) regex over the whole file with a gap that only refuses to cross
    another ``(gr_`` token -- so a primitive's coordinates could be stitched to
    a LATER element's layer tag (comexpress7: a thermal-spoke gr_line inside an
    F.Paste aperture pad + a footprint fp_line's ``(layer "User.1")`` = a
    phantom guide path with pad-local coordinates). Masking the primitives
    blocks keeps those scans to real board graphics. Returns content unchanged
    when there are no primitives.
    """
    out = []
    pos = 0
    for m in re.finditer(r'\(primitives\b', content):
        if m.start() < pos:
            continue
        end = find_matching_paren(content, m.start())
        if end <= m.start():
            continue
        out.append(content[pos:m.start()])
        # Leave a (gr_ barrier token in place of the block: the gr_* scanners'
        # gap regex refuses to cross (gr_ tokens, and blanking the primitives
        # entirely would REMOVE barriers that stopped an earlier element's
        # scan from bridging across this pad.
        barrier = '(gr_masked)'
        pad_len = end - m.start()
        out.append(barrier + ' ' * (pad_len - len(barrier)) if pad_len >= len(barrier)
                   else ' ' * pad_len)
        pos = end
    if not out:
        return content
    out.append(content[pos:])
    masked = ''.join(out)
    memo = _memo_for(content)
    if memo is not None:
        memo['live'][id(masked)] = masked
        memo['masked_from'][id(masked)] = content
    return masked


@functools.lru_cache(maxsize=2)
def _footprint_blocks_by_key_cached(content: str):
    return _footprint_blocks_by_key_uncached(content)


def _footprint_blocks_by_key(content: str):
    """Memoised wrapper. See `_footprint_blocks_by_key_uncached`.

    ONE parse now asks this question four times -- `extract_board_bounds`,
    `footprint_outline_owners` (#829), `_collect_footprint_edge_segments_by_ref`
    (#550) and the #908 footprint-copper pass -- and each call is a
    `find_matching_paren` walk over every footprint block on the board, i.e.
    over the whole file. Uncached, the #908 pass alone cost +41% on ulx3s,
    +47% on glasgow_revC (which carries NO footprint copper at all: the walk,
    not the work) and +74% on watchy. Memoised, the #908 pass is free AND the
    three older callers stop paying for their own repeats.

    The returned list is treated as READ-ONLY by every caller; do not mutate
    it. `maxsize=2` for the same reason `_footprint_edge_points_by_ref` uses
    it: the only repeat caller is one parse, and caching board texts is not
    worth the memory.
    """
    try:
        hash(content)
    except TypeError:              # not a str: cannot be memoised
        return _footprint_blocks_by_key_uncached(content)
    # A masked copy has the original's blocks: masking replaces each pad's
    # `(primitives ...)` with a balanced token of the same length, inside the
    # pad, so no footprint's start, end or reference moves.
    memo = _memo_for(content)
    if memo is not None:
        content = memo['masked_from'].get(id(content), content)
    # Deliberately NOT a try/except around the cached call: that would also
    # catch a TypeError raised INSIDE the scan, re-run the whole whole-file
    # walk, and hide the real bug behind a slow success.
    return _footprint_blocks_by_key_cached(content)


def _footprint_blocks_by_key_uncached(content: str):
    """Ordered `(start, end, key)` for every `(footprint ...)` block, keyed the
    way `extract_footprints_and_pads` keys the footprints dict (#829).

    Routed through #726's `footprint_raw_reference` and
    `disambiguate_references` rather than resolving a reference locally. Two
    blocks may claim one name, and #726 keys the second `TP4~2`; an owner map
    keyed by the RAW name would stamp `owns_board_outline` onto whichever block
    won the dict, which on a duplicated reference need not be the block that
    draws the outline. Sharing the disambiguator is what keeps the two in step.
    """
    spans, raws = [], []
    for m in re.finditer(r'\(footprint\s+"', content):
        end = _block_end(content, m.start())
        spans.append((m.start(), end))
        raws.append(footprint_raw_reference(content[m.start():end]))
    return [(a, b, k) for (a, b), k in
            zip(spans, disambiguate_references(raws))]


#: The footprint-embedded drawing primitives. `fp_curve` is deliberately absent
#: from the COPPER scan (see `iter_footprint_shapes`): the Edge.Cuts scans
#: linearize it for bounds, but no corpus board draws copper with one, and a
#: bezier needs its own linearizer rather than a shared block reader.
_FP_SHAPE_TAGS = ('fp_line', 'fp_arc', 'fp_poly', 'fp_rect', 'fp_circle')
_FP_SHAPE_RE = re.compile(r'\(fp_(?:line|arc|poly|rect|circle)\b')


def iter_footprint_shapes(fp_text: str, tags=_FP_SHAPE_TAGS):
    """Yield `(tag, block)` for each `(fp_* ...)` shape in ONE footprint block,
    every block read as a BALANCED sub-expression (#908).

    Balanced, not regex-sliced, for the same reason the board-level `gr_*` scan
    in `extract_segments` is block-based: an `fp_poly`'s vertices live in a
    nested `(pts ...)` and its `(layer ...)` may sit on either side of them, so
    a flat field-ordered pattern either misses the layer or runs out of one
    shape into the next.

    Custom-pad `(primitives ...)` must never reach this. KiCad spells those
    shapes `gr_*`, so the tag list alone happens to exclude them -- but that
    is a spelling coincidence, not a guarantee, and the caller therefore feeds
    this the `_mask_pad_primitives` text. A primitive that leaks in is not
    merely phantom copper: its coordinates are PAD-local, so the footprint
    pose places it millimetres from where it belongs (measured: 3.5mm off on
    a synthetic 45-degree part) -- a false obstacle AND missing copper at
    once. Do not widen this to "any shape inside a footprint block".
    """
    for tag, i, j in _footprint_shape_spans(fp_text, tags):
        yield tag, fp_text[i:j]


def _footprint_shape_spans(fp_text: str, tags=_FP_SHAPE_TAGS):
    """`iter_footprint_shapes`' shapes as `(tag, start, end)` in `fp_text`."""
    for tag in tags:
        needle = '(' + tag
        pos = 0
        while True:
            i = fp_text.find(needle, pos)
            if i < 0:
                break
            nxt = fp_text[i + len(needle): i + len(needle) + 1]
            if nxt and (nxt.isalnum() or nxt == '_'):
                pos = i + len(needle)      # token boundary: '(fp_linex'
                continue
            j = find_matching_paren(fp_text, i)
            yield tag, i, j
            pos = j


def _footprint_shapes(content: str, start: int, fp_text: str,
                      tags=_FP_SHAPE_TAGS):
    """`iter_footprint_shapes(fp_text, tags)` for the footprint block that
    starts at `start` in `content` (`fp_text` is its text). During a parse,
    for the parse's own texts, the shapes are found once and kept as offsets
    (`_PARSE_MEMO`): the segment and paste-graphic extractors walk the same
    blocks."""
    memo = _memo_for(content)
    if memo is None:
        yield from iter_footprint_shapes(fp_text, tags)
        return
    key = (id(content), start, tags)
    spans = memo['shapes'].get(key)
    if spans is None:
        flat = array('l')
        for tag, i, j in _footprint_shape_spans(fp_text, tags):
            flat.extend((tags.index(tag), i, j))
        spans = memo['shapes'][key] = flat
    for k in range(0, len(spans), 3):
        yield tags[spans[k]], fp_text[spans[k + 1]:spans[k + 2]]


def footprint_pose(fp_text: str):
    """`(x, y, rotation_deg)` of one footprint block, or None.

    `(0, 0, 0)` -- KiCad's default -- when the block has no `(at ...)` of its
    own; None only when it has one that does not parse.

    The footprint's own `(at ...)` is the one at the block's TOP LEVEL. Taking
    the textually first one instead is wrong twice over, and #908 raised the
    stakes from a bounds point to a copper obstacle at the wrong place:

      * a child's `(at ...)` -- a property's, a pad's -- can precede it, and
        the shape then lands relative to the child instead of the part;
      * an `(at ...)` inside a QUOTED STRING (a `(descr "trap (at 99 99 45)")`)
        is not markup at all. `find_matching_paren` was taught to skip quoted
        strings for exactly this class of bug (#113, the footprint named
        `"TCR2EF115,LM(CT"`); a regex here would re-introduce it on the same
        file.

    Neither shape is what KiCad writes today -- this agrees with pcbnew's
    `GetPosition()`/`GetOrientationDegrees()` on all 1349 corpus footprints --
    so both are latent. They are also free to rule out.
    """
    j = _footprint_at_start(fp_text)
    if j < 0:
        # KiCad's own reader leaves a footprint with no `(at ...)` at its
        # default pose, the origin; taking a child's instead was the bug.
        return (0.0, 0.0, 0.0)
    m = AT_NODE_RE.match(fp_text, j)
    if m is None:
        return None
    return (float(m.group(1)), float(m.group(2)),
            float(m.group(3)) if m.group(3) else 0.0)


_FP_PAD_RE = re.compile(r'\(pad\s+("(?:[^"\\]|\\.)*"|\S+)\s+(\w+)')

#: A pad's number, type and shape, in both spellings KiCad reads (#1290): the
#: quoted `(pad "1" smd rect` and the older unquoted `(pad 1 smd rect`, which
#: pcbnew still loads. Group 1 is a quoted number, group 2 a bare one; a bare
#: token reads as the same token quoted.
PAD_HEAD_RE = re.compile(r'\(pad\s+(?:"([^"]*)"|([^\s()"]+))\s+(\w+)\s+(\w+)')
#: A string node's value, quoted or bare (`(pinfunction "GND")`, `(pinfunction
#: GND)`): group 1 quoted, group 2 bare.
_STR_OR_BARE = r'(?:"([^"]*)"|([^\s()"]+))'
#: A net reference whose NAME is bare -- `(net GND)` or `(net 3 GND)` -- as a
#: footprint written in the older unquoted spelling has it (#1290). Group 1 is
#: the name; a lone number, `(net 5)`, is an id and never matches.
_BARE_NET_REF_RE = re.compile(r'\(net\s+(?:\d+\s+)?(?!\d+\))([^\s()"]+)\)')


def pad_net_from_text(pad_text: str, name_to_id) -> Tuple[int, str]:
    """(net_id, net_name) of one pad block's `(net ...)` node, in each spelling
    pcbnew reads: `(net 3 "GND")`, `(net "GND")` (KiCad 10), and the unquoted
    `(net 3 GND)` / `(net GND)` of footprints written in the older spelling
    (#1290). A bare name resolves by NAME through the board's net table, as
    pcbnew resolves it on a board with no numeric table; the id is the
    fallback when the table does not know the name. (0, "") when the pad has
    no net."""
    m = re.search(r'\(net\s+(\d+)\s+"%s"\)' % _ESC_STR, pad_text)
    if m:
        return int(m.group(1)), _unescape_kicad_string(m.group(2))
    m = re.search(r'\(net\s+"%s"\)' % _ESC_STR, pad_text)
    if m:
        if name_to_id:
            return (name_to_id.get(m.group(1), 0),
                    _unescape_kicad_string(m.group(1)))
        return 0, ""
    m = _BARE_NET_REF_RE.search(pad_text)
    if m:
        name = m.group(1)
        if name in (name_to_id or {}):
            return name_to_id[name], name
        num = re.match(r'\(net\s+(\d+)\s', m.group(0))
        return (int(num.group(1)), name) if num else (0, "")
    return 0, ""


def footprint_pad_count(fp_text: str) -> int:
    """Number of pads with COPPER in one footprint block's TEXT.

    NPTH pads are excluded: CLAUDE.md's own rule is that they have no copper
    even when `layers` lists `*.Cu` (the size is the mask opening), so a
    footprint whose only "pad" is a net-tied mounting hole owns no land
    pattern and its copper artwork is decoration like any other logo's.
    Without this a decorative footprint carrying one mounting hole would be
    modelled AND kept on copper by the writer.
    """
    return sum(1 for _num, _type in _FP_PAD_RE.findall(fp_text)
               if _type != 'np_thru_hole')


def footprint_copper_is_functional(pad_count: int) -> bool:
    """Is copper drawn inside a footprint the PART's copper, or decoration?

    #908. A footprint shape cannot carry a `(net ...)` in KiCad, so the #337
    net guard -- "net-tied copper is functional, net-less copper is a logo" --
    is a NO-OP for `fp_*` and cannot separate the two. The owning footprint's
    PAD COUNT can, and does so exactly on the corpus:

    | board              | ref                        | pads | copper polys | what it is          |
    |--------------------|----------------------------|------|--------------|---------------------|
    | esp_prog           | U2                         | 3    | 1            | SOT89 drawn tab     |
    | watchy             | AE1                        | 2    | 12           | PCB meander antenna |
    | tigard             | JP1                        | 2    | 1            | bridged jumper      |
    | ulx3s              | RP1-3, D9, D51, D52        | 2    | 1 each       | NC jumpers          |
    | orangecrab_ext_pll | `G***`, `G***~2`           | 0    | 1 + 3        | OSHW / LOGO art     |

    kicad-cli 10.0.0 agrees independently: the pad-BEARING footprints produce
    17 pad-vs-polygon `shorting_items` between them, the pad-LESS logo
    footprints produce zero.

    A part's land-pattern copper is therefore MODELLED (obstacle + DRC) and
    never relocated to silkscreen; a logo footprint keeps #146's treatment
    unchanged -- moved to silk by the writer, and for that reason not modelled
    either, since modelling copper the writer is about to move off copper
    would cost routing area for nothing.

    Takes a scalar, not a footprint, precisely so the four fronts that ask it
    (text parser, pcbnew parser, CLI writer, GUI writer) cannot drift: each
    counts pads in its own idiom and there is nothing else to disagree about.
    """
    return pad_count > 0


def _footprint_edge_points(content: str) -> List[Tuple[float, float]]:
    """GLOBAL points of footprint-embedded Edge.Cuts shapes (fp_line/fp_rect/
    fp_arc/fp_circle/fp_poly), transformed by each footprint's (at x y rot).

    Some boards keep their entire outline inside a footprint (Adiuvo's
    rp2350_fpga_eensy: fp_lines on Edge.Cuts in an outline footprint); a
    board-level gr_* scan alone reports no bounds, which silently disables
    the routing edge keep-out. KiCad's own edges bounding box includes
    footprint shapes, so these points belong in board_bounds."""
    return [p for pts in _footprint_edge_points_by_ref(content).values()
            for p in pts]


@functools.lru_cache(maxsize=2)
def _footprint_edge_points_by_ref_cached(content: str):
    return _footprint_edge_points_by_ref_uncached(content)


def _footprint_edge_points_by_ref(
        content: str) -> Dict[str, List[Tuple[float, float]]]:
    """Memoised wrapper. See `_footprint_edge_points_by_ref_uncached`.

    A single `parse_kicad_pcb` asks this question TWICE -- once through
    `extract_board_bounds` (which has always made this pass) and once through
    `footprint_outline_owners` (#829) -- and the pass is a
    `find_matching_paren` walk plus a slice of every footprint block on the
    board. Without the memo #829 added a second full scan to every parse:
    measured +15% on ulx3s, +18% on glasgow_revC, +29% on watchy. With it the
    second call is free and a parse costs what it did before.

    The returned dict is treated as read-only by both callers; do not mutate it.
    `maxsize=2` because the only repeat caller is one parse, and a big cache of
    board texts is not worth the memory.
    """
    try:
        return _footprint_edge_points_by_ref_cached(content)
    except TypeError:                                            # unhashable
        return _footprint_edge_points_by_ref_uncached(content)


def _footprint_edge_points_by_ref_uncached(
        content: str) -> Dict[str, List[Tuple[float, float]]]:
    """The SAME scan as `_footprint_edge_points`, keyed by owning ref (#829).

    The scan always had the reference in hand and threw it away, so nothing
    downstream could ask "which footprint owns this piece of the outline?" --
    and the placement movable set, gated on pads alone, would happily move it.
    One scan and one predicate feed both callers deliberately: the tree already
    carries FOUR disagreeing answers to "which fp shapes are Edge.Cuts
    geometry" (this scan, `_collect_footprint_edge_segments`, and the two
    pcbnew mirrors, whose class filters differ), and a fifth written by hand
    would drift from all of them.

    A footprint appears here only if it contributed at least one point.
    """
    owners: Dict[str, List[Tuple[float, float]]] = {}
    if '"Edge.Cuts"' not in content:
        return owners
    for _start, _end, _key in _footprint_blocks_by_key(content):
        fp_text = content[_start:_end]
        if '"Edge.Cuts"' not in fp_text:
            continue
        pts: List[Tuple[float, float]] = []
        _pose = footprint_pose(fp_text)
        if _pose is None:
            continue
        fx, fy, frot = _pose
        local: List[Tuple[float, float]] = []
        for sm in re.finditer(
                r'\(fp_(line|rect)\s+\(start\s+([\d.-]+)\s+([\d.-]+)\)\s+'
                r'\(end\s+([\d.-]+)\s+([\d.-]+)\)' + _GR_ELEMENT_GAP +
                r'\(layer\s+"Edge\.Cuts"\)', fp_text, re.DOTALL):
            x1, y1, x2, y2 = (float(sm.group(i)) for i in range(2, 6))
            if sm.group(1) == 'rect':
                # all four corners: under rotation the rect tilts, and the
                # two derived corners bound it where start/end alone don't
                local += [(x1, y1), (x2, y2), (x1, y2), (x2, y1)]
            else:
                local += [(x1, y1), (x2, y2)]
        for sm in re.finditer(
                r'\(fp_arc\s+\(start\s+([\d.-]+)\s+([\d.-]+)\)\s+'
                r'\(mid\s+([\d.-]+)\s+([\d.-]+)\)\s+'
                r'\(end\s+([\d.-]+)\s+([\d.-]+)\)' + _GR_ELEMENT_GAP +
                r'\(layer\s+"Edge\.Cuts"\)', fp_text, re.DOTALL):
            sx, sy, mx, my, ex, ey = (float(sm.group(i)) for i in range(1, 7))
            for seg in _arc_to_segments((sx, sy), (mx, my), (ex, ey)):
                local += list(seg)
        # fp_curve: a footprint-embedded bezier edge. Linearize in LOCAL coords
        # (the control points do not bound the curve) and let the shared
        # transform below place it -- mirrors the board-level gr_curve scan.
        for sm in re.finditer(_FP_CURVE_PATTERN, fp_text, re.DOTALL):
            g = [float(v) for v in sm.groups()]
            for seg in _bezier_to_segments((g[0], g[1]), (g[2], g[3]),
                                           (g[4], g[5]), (g[6], g[7])):
                local += list(seg)
        for lx, ly in local:
            pts.append(local_to_global(fx, fy, frot, lx, ly))
        for sm in re.finditer(
                r'\(fp_circle\s+\(center\s+([\d.-]+)\s+([\d.-]+)\)\s+'
                r'\(end\s+([\d.-]+)\s+([\d.-]+)\)' + _GR_ELEMENT_GAP +
                r'\(layer\s+"Edge\.Cuts"\)', fp_text, re.DOTALL):
            cx, cy, ex, ey = (float(sm.group(i)) for i in range(1, 5))
            r = math.hypot(ex - cx, ey - cy)
            gcx, gcy = local_to_global(fx, fy, frot, cx, cy)
            # circle bounds are rotation-invariant about the moved center
            pts += [(gcx - r, gcy - r), (gcx + r, gcy + r)]
        for pm in re.finditer(r'\(fp_poly\s*\(pts', fp_text):
            p_end = find_matching_paren(fp_text, pm.start())
            poly_text = fp_text[pm.start():p_end]
            if not re.search(r'\(layer\s+"Edge\.Cuts"\)', poly_text):
                continue
            for xm in re.finditer(r'\(xy\s+([\d.-]+)\s+([\d.-]+)\)', poly_text):
                pts.append(local_to_global(fx, fy, frot,
                                           float(xm.group(1)), float(xm.group(2))))
        if pts:
            owners.setdefault(_key, []).extend(pts)
    return owners


# #829: tolerance for "does this point extend the board?". A point on the
# board-level boundary is CARRIED, not structural -- a shape flush with the
# edge did not move the edge.
_OUTLINE_EPS = 1e-6


def _mask_footprint_blocks(content: str, only_refs=None) -> str:
    """`content` with `(footprint ...)` blocks blanked to spaces.

    Length-preserving, so any offset computed against the masked text still
    lines up with the original. With `only_refs`, mask just those references;
    otherwise mask every footprint, which asks what the board's Edge.Cuts would
    say if no footprint contributed any.
    """
    out = list(content)
    for start, end, key in _footprint_blocks_by_key(content):
        if only_refs is not None and key not in only_refs:
            continue
        for i in range(start, end):
            if out[i] != '\n':
                out[i] = ' '
    return ''.join(out)


def footprint_outline_owners(content: str) -> Dict[str, bool]:
    """`{ref: owns_board_outline}` for every footprint that draws Edge.Cuts.

    #829. A footprint's pose transforms its Edge.Cuts shapes, so moving one
    moves board geometry. But that is only a DEFECT for some of them, and the
    split is what this function computes:

    * **Structural** (`True`) -- the shape lies outside every ring the
      BOARD-LEVEL Edge.Cuts forms, so it is part of the board's own boundary.
      Moving it resizes the board, which is never a placement decision. A board
      whose outline lives entirely in footprints (Adiuvo's rp2350_fpga_eensy)
      has no board-level rings at all, so every owner is structural.
    * **Carried** (`False`) -- a closed shape beside the part's own copper
      (#1409: it encloses none of the part's pads, wherever the part sits), or
      one inside the board outline: a
      milled relief or a window the designer PARENTED to the part precisely so
      it would travel with it. crkbd draws 184 per-LED cutout windows this way
      (see `_collect_edge_cuts_segments`), and #628 exists to keep exactly this
      class of part PLACEABLE -- it exempts a part's own milled ring from the
      edge-margin test, because without that exemption run 20's SW2, sitting
      inside its own strap slot, had 0 legal poses of 14884 (`legality.py`).
      Locking every owner would freeze the very parts that work protects.

    Deciding by containment rather than by "does it own Edge.Cuts at all" is
    the whole point: parentage is the designer's statement about which of the
    two a shape is, and the first draft of this fix read it backwards.

    Costs nothing on a board with no footprint-embedded Edge.Cuts (which is
    every board in this repo's corpus): the owner scan short-circuits and the
    second contour pass never runs.
    """
    # The cheap gate first: the bounds scan is already memoised from this same
    # parse, and on every board in this repo's corpus it answers "nobody", so
    # nothing below ever runs.
    if not _footprint_edge_points_by_ref(content):
        return {}
    segs_by_ref = _collect_footprint_edge_segments_by_ref(content)
    board_only = _mask_footprint_blocks(content)
    outers, _cutouts = extract_board_contours(board_only)
    return classify_outline_owners(
        segs_by_ref, extract_board_bounds(board_only), outers,
        own_pads=_footprint_own_pads_by_ref(content, set(segs_by_ref)))


def outline_fingerprint(board_info) -> Tuple:
    """A canonical, comparable summary of a board's Edge.Cuts geometry (#829).

    `board_bounds` alone is NOT a sufficient detector and this is the reason
    this helper exists: an interior cutout moving never touches the bounding
    box, and `_footprint_edge_points` treats an `fp_circle` as rotation-
    invariant about its moved centre, so rotating a footprint that owns a
    circular window changes nothing in the bbox. Rings and cutouts catch what
    bounds cannot; bounds catch an open stub that never chains into a ring,
    which rings cannot. Both are needed -- measured: the #829 repro changed
    `board_bounds` while `board_outlines` stayed identical.

    Rounded to 1 nm, and rings are compared as SETS of rounded vertices so a
    chainer that happens to start or wind a ring differently is not reported as
    a moved outline.
    """
    def ring(r):
        return tuple(sorted((round(x, 6), round(y, 6)) for x, y in r))

    b = board_info.board_bounds
    return (
        tuple(round(v, 6) for v in b) if b else None,
        tuple(sorted(ring(r) for r in (board_info.board_outlines or []))),
        tuple(sorted(ring(r) for r in (board_info.board_cutouts or []))),
        len(getattr(board_info, 'board_edge_contours', None) or []),
    )


def structural_outline_fingerprint(content: str, carried=None) -> Tuple:
    """`outline_fingerprint` of the board's STRUCTURAL outline only (#829).

    `carried` (#1409), a set of refs, replaces the classification read off
    `content` itself. A write compares the board BEFORE and AFTER one move, and
    a ring the part's own pads cannot settle (one around a pad of its own, or
    a pad-less footprint's) is judged by containment at the footprint's
    CURRENT pose (`classify_outline_owners` rule 2b): carried while the part
    is on the board and structural once it is off, so classifying each text
    separately read a legal move of the part off the board as the outline
    changing. The caller decides once, on the input, and passes the
    same set for both texts.

    The board-level Edge.Cuts plus the footprints that draw the boundary --
    with every CARRIED owner masked out. That distinction is the whole reason
    this exists rather than a plain whole-board fingerprint: a relief parented
    to a part is *supposed* to travel with it, so a run that legitimately moves
    one changes `board_cutouts`, and comparing the full outline would refuse
    exactly the move #829's fix is careful to permit. Measured while writing
    this: the naive version fired on the carried case on its first run.

    Invariant under any permitted placement move, by construction. A change
    means something moved a footprint that draws the board.
    """
    if carried is None:
        owners = footprint_outline_owners(content)
        carried = {ref for ref, structural in owners.items() if not structural}
    if carried:
        content = _mask_footprint_blocks(content, only_refs=carried)
    outers, cutouts = extract_board_contours(content)
    return outline_fingerprint(BoardInfo(
        layers={}, copper_layers=[],
        board_bounds=extract_board_bounds(content),
        board_outlines=outers, board_cutouts=cutouts))


def _segments_close_on_themselves(segments, tol: float = 0.01) -> bool:
    """Do these segments form only CLOSED loops?

    Euler's condition: a set of edges decomposes into closed circuits exactly
    when every vertex has even degree. Endpoints are snapped to `tol` (10 um,
    the chainer's own tolerance) so a KiCad rounding difference between two
    shapes that meet does not read as two odd vertices.

    Cheap, exact for the question asked, and it does not need the chainer --
    which cannot answer it: `_chain_segments_into_contours` returns the same
    vertex list for a closed square and for three sides of one.
    """
    if not segments:
        return False
    deg = {}
    for a, b in segments:
        for p in (a, b):
            k = (round(p[0] / tol), round(p[1] / tol))
            deg[k] = deg.get(k, 0) + 1
    return all(d % 2 == 0 for d in deg.values())


# #1409: a pad centre within this distance (mm) of a ring counts as ON it --
# the chainer's own tolerance. On the ring is not "beside the hole": a
# castellated pad sits on the edge its part draws.
_OWN_PAD_EPS = 0.01


def _footprint_own_pad_centres(block: str):
    """The LOCAL-frame centres of every pad in one footprint block (#1409):
    each pad's `(at x y ...)`, which the file stores in the footprint's own
    coordinates. Read wherever the part sits, so the answer cannot change with
    its pose. A pad whose `(at ...)` does not parse is skipped."""
    out = []
    for pm in PAD_HEAD_RE.finditer(block):
        pad_text = block[pm.start():find_matching_paren(block, pm.start())]
        at = AT_NODE_RE.search(pad_text)
        if at is not None:
            out.append((float(at.group(1)), float(at.group(2))))
    return out


def _footprint_own_pads_by_ref(content: str, refs):
    """`{ref: (fx, fy, rot, [local pad centres])}` for the footprints in
    `refs`: the pose the classifier needs to carry a ring into the footprint's
    own frame, and the footprint's pads in that frame."""
    out = {}
    for start, end, key in _footprint_blocks_by_key(content):
        if key not in refs:
            continue
        block = content[start:end]
        pose = footprint_pose(block)
        if pose is None:
            continue
        out[key] = pose + (_footprint_own_pad_centres(block),)
    return out


def _dist_to_segment(px, py, a, b) -> float:
    (x1, y1), (x2, y2) = a, b
    dx, dy = x2 - x1, y2 - y1
    ll = dx * dx + dy * dy
    t = 0.0 if ll == 0 else max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / ll))
    return math.hypot(px - x1 - t * dx, py - y1 - t * dy)


def _ring_spares_own_pads(segs, own) -> bool:
    """Is this closed ring a hole BESIDE its footprint's copper (#1409)?

    `own` is `(fx, fy, rot, [local pad centres])`. True when the footprint has
    at least one pad and no pad centre lies inside the ring or on it, judged
    in the footprint's LOCAL frame -- the ring is carried there by
    `_global_to_local`, the pose's exact inverse -- so the answer is the same
    at every pose.

    Inside is even-odd over the ring's segments, which needs no chaining and
    reads several loops (a part's row of slots) at once. A segment drawn twice
    is counted once: a doubled loop would otherwise cancel itself out and
    enclose nothing.
    """
    if own is None:
        return False
    fx, fy, rot, pads = own
    if not pads:
        return False
    uniq = {}
    for a, b in segs:
        la = _global_to_local(fx, fy, rot, *a)
        lb = _global_to_local(fx, fy, rot, *b)
        ka = (round(la[0], 3), round(la[1], 3))
        kb = (round(lb[0], 3), round(lb[1], 3))
        uniq.setdefault((min(ka, kb), max(ka, kb)), (la, lb))
    local = list(uniq.values())
    for px, py in pads:
        inside = False
        for (x1, y1), (x2, y2) in local:
            if (y1 > py) != (y2 > py):
                if x1 + (py - y1) * (x2 - x1) / (y2 - y1) > px:
                    inside = not inside
        if inside:
            return False
        if any(_dist_to_segment(px, py, a, b) <= _OWN_PAD_EPS
               for a, b in local):
            return False
    return True


def classify_outline_owners(owners, board_only_bounds, board_only_outers,
                            own_pads=None):
    """`{ref: owns_board_outline}` from per-ref SEGMENTS and the board's own
    outline. Shared by BOTH parse paths (#829) so they cannot decide
    differently -- each supplies its own inputs, the rule lives here once.

    `owners` is `{ref: [((x1,y1),(x2,y2)), ...]}` in global mm. `own_pads`
    (#1409) is `{ref: (fx, fy, rot, [local pad centres])}`: each owner's pose
    and its own pads in its LOCAL frame (`_footprint_own_pad_centres`); a ref
    missing from it is judged by rule 2b alone.

    A footprint is CARRIED (its geometry travels with it, and it stays movable)
    only when:

      1. its Edge.Cuts segments CLOSE ON THEMSELVES -- it is a window, a slot,
         a relief: a shape cut out of the board; and
      2a. the footprint has pads and that shape encloses none of them, judged
          in the footprint's own frame (#1409) -- a hole beside the part's
          copper, which the part carries wherever it is; or, failing that,
      2b. that shape lies inside the outline the board draws without it.

    Anything else is STRUCTURAL. Closedness carries most of the weight, and it
    is what the first version of this function lacked. Deciding on containment
    alone was wrong in three measured ways:

      * a connector drawing the real board's fourth EDGE (an open path joining
        the board-level outline) sits inside the panel frame on a panelised
        board, so containment called it carried -- and it draws the boundary;
      * `extract_board_contours` short-circuits a 4-segment axis-aligned
        rectangle to NO rings, so the same physical geometry classified
        differently depending on whether the board's outline was spelled with 4
        segments or 8;
      * an open stub flush with the edge read as "on the boundary, therefore
        carried", when a shape flush with the edge IS the edge.

    An open path cannot be a cut-out, so all three become structural on rule 1
    without consulting containment at all.

    Rule 2a is why the answer does not flip with a move (#1409). Rule 2b is
    judged at the part's CURRENT pose: a part carrying a milled window read
    carried on the board and structural once off it, so a pile made by any
    means froze it there (`place_pose set` refused to bring it back: "draws
    the board outline"). 2a reads only the part's own geometry, which is
    rigid in its frame. It asks which side of the ring the part's copper is
    on, because that is what tells a hole from a boundary: a hole cut beside
    or under a part leaves its pads on the board, outside the ring; a ring
    that IS the board's edge has the part's pads on the board, inside it (a
    module drawing its own outline encloses most of its pads, and its
    castellations sit ON the ring, which counts as enclosed). Measured over
    every closed footprint ring in the stress corpus: each window, slot and
    relief encloses no pad of its part, each footprint-drawn board outline
    encloses at least half of its own. A bounding box of the part's pads and
    courtyard does NOT separate them -- an LED window under a reverse-mount
    part stands up to 0.74 mm proud of it, further than a footprint-drawn
    board outline stands past its own pads (0.41 mm). A ring around a
    drilled pin (a plated slot) encloses that pad, and a pad-less footprint
    has nothing to judge by, so both are left to 2b, as before.
    """
    if not owners:
        return {}
    own_pads = own_pads or {}
    out = {}
    pending = {}
    for ref, segs in owners.items():
        if not _segments_close_on_themselves(segs):
            out[ref] = True                                    # rule 1
        elif _ring_spares_own_pads(segs, own_pads.get(ref)):
            out[ref] = False                                   # rule 2a
        else:
            pending[ref] = segs                                # rule 2b
    if not pending:
        return out
    if board_only_bounds is None:
        # The board-level Edge.Cuts draws nothing at all, so the footprints ARE
        # the outline. Every owner rule 2a did not settle is structural.
        out.update({ref: True for ref in pending})
        return out

    rings = [r for r in (board_only_outers or []) if len(r) >= 3]

    if rings:
        def inside(px, py):
            return any(_pt_in_ring(px, py, r) for r in rings)
    else:
        # Bounds but no CLOSED ring -- and that is NOT "no outline":
        # splitflap_driver has a perfectly good board-level outline whose
        # segments never chain closed, so extract_board_contours returns 0
        # outers while extract_board_bounds returns the right rectangle. The
        # bbox is weaker than a ring test (it cannot see a concave notch), but
        # rule 1 has already rejected everything that is not a closed shape, so
        # what reaches here is a window being asked "are you on this board".
        x0, y0, x1, y1 = board_only_bounds

        def inside(px, py):
            return (x0 - _OUTLINE_EPS <= px <= x1 + _OUTLINE_EPS
                    and y0 - _OUTLINE_EPS <= py <= y1 + _OUTLINE_EPS)

    for ref, segs in pending.items():
        out[ref] = not all(inside(*p) for seg in segs for p in seg)
    return out


def extract_board_bounds(content: str) -> Optional[Tuple[float, float, float, float]]:
    """Extract board outline bounds from Edge.Cuts layer."""
    content = _mask_pad_primitives(content)  # pad primitives are not board graphics
    min_x = min_y = float('inf')
    max_x = max_y = float('-inf')
    found = False

    # Look for gr_rect on Edge.Cuts (multi-line format)
    rect_pattern = r'\(gr_rect\s+\(start\s+([\d.-]+)\s+([\d.-]+)\)\s+\(end\s+([\d.-]+)\s+([\d.-]+)\)' + _GR_ELEMENT_GAP + r'\(layer\s+"Edge\.Cuts"\)'
    for m in re.finditer(rect_pattern, content, re.DOTALL):
        x1, y1, x2, y2 = float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4))
        min_x = min(min_x, x1, x2)
        min_y = min(min_y, y1, y2)
        max_x = max(max_x, x1, x2)
        max_y = max(max_y, y1, y2)
        found = True

    # Look for gr_line on Edge.Cuts (multi-line format)
    line_pattern = r'\(gr_line\s+\(start\s+([\d.-]+)\s+([\d.-]+)\)\s+\(end\s+([\d.-]+)\s+([\d.-]+)\)' + _GR_ELEMENT_GAP + r'\(layer\s+"Edge\.Cuts"\)'
    for m in re.finditer(line_pattern, content, re.DOTALL):
        x1, y1, x2, y2 = float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4))
        min_x = min(min_x, x1, x2)
        min_y = min(min_y, y1, y2)
        max_x = max(max_x, x1, x2)
        max_y = max(max_y, y1, y2)
        found = True

    # Look for gr_arc on Edge.Cuts (multi-line format)
    arc_pattern = r'\(gr_arc\s+\(start\s+([\d.-]+)\s+([\d.-]+)\)\s+\(mid\s+([\d.-]+)\s+([\d.-]+)\)\s+\(end\s+([\d.-]+)\s+([\d.-]+)\)' + _GR_ELEMENT_GAP + r'\(layer\s+"Edge\.Cuts"\)'
    for m in re.finditer(arc_pattern, content, re.DOTALL):
        sx, sy = float(m.group(1)), float(m.group(2))
        mx, my = float(m.group(3)), float(m.group(4))
        ex, ey = float(m.group(5)), float(m.group(6))
        # Use linearized arc points for accurate bounds
        for seg in _arc_to_segments((sx, sy), (mx, my), (ex, ey)):
            for px, py in seg:
                min_x = min(min_x, px)
                max_x = max(max_x, px)
                min_y = min(min_y, py)
                max_y = max(max_y, py)
        found = True

    # Look for gr_poly on Edge.Cuts (free-form board outlines), with the
    # points the outline scan reads (arc vertices included)
    for poly in _edge_cuts_gr_poly_points(content):
        for px, py in poly:
            min_x = min(min_x, px)
            max_x = max(max_x, px)
            min_y = min(min_y, py)
            max_y = max(max_y, py)
        if poly:
            found = True

    # gr_curve on Edge.Cuts (#550): a bezier's own control points do NOT bound
    # it, so linearize exactly as the contour scan does -- a curved edge
    # bulging outward otherwise leaves the bbox short by the whole bulge.
    for m in re.finditer(_GR_CURVE_PATTERN, content, re.DOTALL):
        g = [float(v) for v in m.groups()]
        for seg in _bezier_to_segments((g[0], g[1]), (g[2], g[3]),
                                       (g[4], g[5]), (g[6], g[7])):
            for px, py in seg:
                min_x = min(min_x, px)
                max_x = max(max_x, px)
                min_y = min(min_y, py)
                max_y = max(max_y, py)
        found = True

    # gr_circle on Edge.Cuts (#550): how KiCad writes a ROUND BOARD, and also
    # round cutouts. Missing it made board_bounds None on a circular board, so
    # add_board_edge_obstacles returned before stamping any keep-out at all.
    for m in re.finditer(_GR_CIRCLE_PATTERN, content, re.DOTALL):
        cx, cy, ex, ey = (float(v) for v in m.groups())
        r = math.hypot(ex - cx, ey - cy)
        if r <= 1e-6:  # degenerate: the contour scan drops these too
            continue
        min_x = min(min_x, cx - r)
        max_x = max(max_x, cx + r)
        min_y = min(min_y, cy - r)
        max_y = max(max_y, cy + r)
        found = True

    # Footprint-embedded Edge.Cuts (fp_* shapes, transformed to global):
    # boards whose outline lives in a footprint otherwise report no bounds
    # and route with no edge keep-out. Mirrored in build_pcb_data_from_board.
    for px, py in _footprint_edge_points(content):
        min_x = min(min_x, px)
        max_x = max(max_x, px)
        min_y = min(min_y, py)
        max_y = max(max_y, py)
        found = True

    if found:
        return (min_x, min_y, max_x, max_y)
    return None


def _bezier_to_segments(p0, p1, p2, p3, num_segments: int = 16):
    """Cubic bezier -> consecutive line segments (de Casteljau sampling)."""
    pts = []
    for i in range(num_segments + 1):
        t = i / num_segments
        mt = 1.0 - t
        x = (mt ** 3) * p0[0] + 3 * (mt ** 2) * t * p1[0] + 3 * mt * (t ** 2) * p2[0] + (t ** 3) * p3[0]
        y = (mt ** 3) * p0[1] + 3 * (mt ** 2) * t * p1[1] + 3 * mt * (t ** 2) * p2[1] + (t ** 3) * p3[1]
        pts.append((x, y))
    return list(zip(pts, pts[1:]))


#: Chord sag allowed on an Edge.Cuts arc or circle (#1404). Copper is cleared
#: against the LINEARISED outline, and a chord lies inside its arc: on a convex
#: edge (a round cutout, a rounded corner seen from the copper side) the
#: polygon is that sag FARTHER from the copper than the true edge, so a track
#: the router thought exactly at the edge floor sat up to the sag inside it
#: (KiCad: 0.1980 against 0.2000 with the 5 um default). 0.5 um keeps the
#: model inside the sub-micron slack the grid-tie epsilon and KiCad's own
#: reporting resolution leave.
EDGE_CHORD_EPS = 0.0005


def _circle_to_segments(cx, cy, r, num_segments: int = 64,
                        chord_eps: float = 0.0):
    """Circle -> closed ring of line segments. With `chord_eps` the ring has
    as many segments as keep the chord sag at or under it (never fewer than
    `num_segments`, at most 2048)."""
    if chord_eps and r > 0:
        max_step = 2.0 * math.acos(max(-1.0, min(1.0, 1.0 - chord_eps / r)))
        if max_step > 0:
            num_segments = max(num_segments,
                               min(2048, int(math.ceil(2 * math.pi / max_step))))
    pts = [(cx + r * math.cos(2 * math.pi * k / num_segments),
            cy + r * math.sin(2 * math.pi * k / num_segments))
           for k in range(num_segments + 1)]
    return list(zip(pts, pts[1:]))


def _collect_edge_cuts_segments(content: str) -> List[Tuple[Tuple[float, float], Tuple[float, float]]]:
    """Parse all gr_line, gr_arc, and gr_rect segments on Edge.Cuts from file content."""
    content = _mask_pad_primitives(content)  # pad primitives are not board graphics
    segments = []

    # gr_line
    line_pattern = r'\(gr_line\s+\(start\s+([\d.-]+)\s+([\d.-]+)\)\s+\(end\s+([\d.-]+)\s+([\d.-]+)\)' + _GR_ELEMENT_GAP + r'\(layer\s+"Edge\.Cuts"\)'
    for m in re.finditer(line_pattern, content, re.DOTALL):
        x1, y1, x2, y2 = float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4))
        # Skip degenerate zero-length lines (matches the pcbnew extraction):
        # zynq_ad9364 has dot-lines on Edge.Cuts that only duplicate vertices.
        if abs(x2 - x1) < 0.001 and abs(y2 - y1) < 0.001:
            continue
        segments.append(((x1, y1), (x2, y2)))

    # gr_arc - approximate as polyline
    arc_pattern = r'\(gr_arc\s+\(start\s+([\d.-]+)\s+([\d.-]+)\)\s+\(mid\s+([\d.-]+)\s+([\d.-]+)\)\s+\(end\s+([\d.-]+)\s+([\d.-]+)\)' + _GR_ELEMENT_GAP + r'\(layer\s+"Edge\.Cuts"\)'
    for m in re.finditer(arc_pattern, content, re.DOTALL):
        sx, sy = float(m.group(1)), float(m.group(2))
        mx, my = float(m.group(3)), float(m.group(4))
        ex, ey = float(m.group(5)), float(m.group(6))
        segments.extend(_arc_to_segments((sx, sy), (mx, my), (ex, ey),
                                         chord_eps=EDGE_CHORD_EPS))

    # gr_rect - expand to 4 line segments
    rect_pattern = r'\(gr_rect\s+\(start\s+([\d.-]+)\s+([\d.-]+)\)\s+\(end\s+([\d.-]+)\s+([\d.-]+)\)' + _GR_ELEMENT_GAP + r'\(layer\s+"Edge\.Cuts"\)'
    for m in re.finditer(rect_pattern, content, re.DOTALL):
        x1, y1, x2, y2 = float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4))
        segments.append(((x1, y1), (x2, y1)))
        segments.append(((x2, y1), (x2, y2)))
        segments.append(((x2, y2), (x1, y2)))
        segments.append(((x1, y2), (x1, y1)))

    # gr_poly - closed polygon outline, expand to line segments. The points
    # are pcbnew's (`kicad_poly_shape_points`: `(arc ..)` vertices too, which
    # the old xy-only pattern did not match at all), and a ZERO-LENGTH edge is
    # skipped as the gr_line reader above and the pcbnew builder skip it: a
    # polygon written with its first point repeated at the end
    # (Castor_and_Pollux's faceplate) otherwise chained into a 5-vertex ring
    # here while the GUI saw the plain 4-edge rectangle KiCad's own board
    # outline is.
    for poly in _edge_cuts_gr_poly_points(content):
        segments.extend(_closed_ring_segments(poly))

    # gr_curve - cubic bezier corner/edge (#304: crkbd's outline corners are
    # 100 gr_curves; without them the outline never chains closed, the halves'
    # small slots masquerade as the whole outline, and every track "leaves the
    # board"). Control points are (pts (xy)x4) in GLOBAL coords.
    for m in re.finditer(_GR_CURVE_PATTERN, content, re.DOTALL):
        g = [float(v) for v in m.groups()]
        segments.extend(_bezier_to_segments((g[0], g[1]), (g[2], g[3]),
                                            (g[4], g[5]), (g[6], g[7])))

    # gr_circle - a standalone ring (mounting hole / round cutout)
    for m in re.finditer(_GR_CIRCLE_PATTERN, content, re.DOTALL):
        cx, cy, ex, ey = (float(v) for v in m.groups())
        r = math.hypot(ex - cx, ey - cy)
        if r > 1e-6:
            segments.extend(_circle_to_segments(cx, cy, r,
                                                chord_eps=EDGE_CHORD_EPS))

    # Footprint-embedded Edge.Cuts (#304): reversible split keyboards draw
    # per-LED cutout windows as fp_lines on Edge.Cuts inside the LED
    # footprints (crkbd: 184 of them). File-local coords are already
    # side-resolved; transform = footprint position + rotation with the KiCad
    # negate convention (verified against pcbnew GraphicalItems on crkbd
    # LED16, flipped + rot 180).
    segments.extend(_collect_footprint_edge_segments(content))

    return segments


def _collect_footprint_edge_segments(content: str):
    """Edge.Cuts fp_line/fp_arc/fp_circle/fp_rect segments inside footprints,
    transformed to global coordinates (issue #304)."""
    return [s for segs in
            _collect_footprint_edge_segments_by_ref(content).values()
            for s in segs]


def _collect_footprint_edge_segments_by_ref(content: str):
    """The same scan, keyed by owning ref (#829).

    The classifier needs SEGMENTS rather than the bounds scan's points: a
    circle's points there are its bounding-box CORNERS (deliberately, because
    bounds are rotation-invariant that way), and a corner of a round window can
    sit outside a round board while every point of the actual circle is inside
    -- which classified a 4 mm relief on a 40 mm round board as structural and
    froze it, the exact #628 over-lock this fix exists to avoid.
    """
    out_by_ref = {}
    for _bstart, _bend, _key in _footprint_blocks_by_key(content):
        block = content[_bstart:_bend]
        if '"Edge.Cuts"' not in block:
            continue
        out = out_by_ref.setdefault(_key, [])
        _pose = footprint_pose(block)
        if _pose is None:
            continue
        fx, fy, rot = _pose
        rad = math.radians(-rot)
        cos_r, sin_r = math.cos(rad), math.sin(rad)

        def to_g(x, y):
            return (fx + x * cos_r - y * sin_r, fy + x * sin_r + y * cos_r)

        gap = _GR_ELEMENT_GAP
        for m in re.finditer(r'\(fp_line\s+\(start\s+([\d.-]+)\s+([\d.-]+)\)\s+\(end\s+([\d.-]+)\s+([\d.-]+)\)'
                             + gap + r'\(layer\s+"Edge\.Cuts"\)', block, re.DOTALL):
            a = to_g(float(m.group(1)), float(m.group(2)))
            b = to_g(float(m.group(3)), float(m.group(4)))
            if abs(b[0] - a[0]) >= 0.001 or abs(b[1] - a[1]) >= 0.001:
                out.append((a, b))
        for m in re.finditer(r'\(fp_rect\s+\(start\s+([\d.-]+)\s+([\d.-]+)\)\s+\(end\s+([\d.-]+)\s+([\d.-]+)\)'
                             + gap + r'\(layer\s+"Edge\.Cuts"\)', block, re.DOTALL):
            x1, y1, x2, y2 = (float(v) for v in m.groups())
            c = [to_g(x1, y1), to_g(x2, y1), to_g(x2, y2), to_g(x1, y2)]
            out.extend([(c[0], c[1]), (c[1], c[2]), (c[2], c[3]), (c[3], c[0])])
        for m in re.finditer(r'\(fp_arc\s+\(start\s+([\d.-]+)\s+([\d.-]+)\)\s+\(mid\s+([\d.-]+)\s+([\d.-]+)\)\s+\(end\s+([\d.-]+)\s+([\d.-]+)\)'
                             + gap + r'\(layer\s+"Edge\.Cuts"\)', block, re.DOTALL):
            g = [float(v) for v in m.groups()]
            for p1, p2 in _arc_to_segments((g[0], g[1]), (g[2], g[3]), (g[4], g[5]),
                                           chord_eps=EDGE_CHORD_EPS):
                out.append((to_g(*p1), to_g(*p2)))
        for m in re.finditer(r'\(fp_circle\s+\(center\s+([\d.-]+)\s+([\d.-]+)\)\s+\(end\s+([\d.-]+)\s+([\d.-]+)\)'
                             + gap + r'\(layer\s+"Edge\.Cuts"\)', block, re.DOTALL):
            cx, cy, ex, ey = (float(v) for v in m.groups())
            r = math.hypot(ex - cx, ey - cy)
            if r > 1e-6:
                for p1, p2 in _circle_to_segments(cx, cy, r,
                                                  chord_eps=EDGE_CHORD_EPS):
                    out.append((to_g(*p1), to_g(*p2)))
        # fp_curve: without it a footprint-embedded curved edge never reaches
        # the chainer, so the outline cannot close and the true extent is
        # invisible to BOTH scans (the board-level twin of #550).
        for m in re.finditer(_FP_CURVE_PATTERN, block, re.DOTALL):
            g = [float(v) for v in m.groups()]
            for p1, p2 in _bezier_to_segments((g[0], g[1]), (g[2], g[3]),
                                              (g[4], g[5]), (g[6], g[7])):
                out.append((to_g(*p1), to_g(*p2)))
        # fp_poly: a closed polygon outline drawn inside a footprint. The bbox
        # scan already read it, so bounds looked right while board_outlines came
        # back EMPTY -- the keep-out then degrades to the bounding box and any
        # concave region outside the polygon goes unblocked.
        for pm in re.finditer(r'\(fp_poly\s*\(pts', block):
            p_end = find_matching_paren(block, pm.start())
            poly_text = block[pm.start():p_end]
            if not re.search(r'\(layer\s+"Edge\.Cuts"\)', poly_text):
                continue
            # pcbnew's points (arcs included), then the footprint transform,
            # and no zero-length edge -- the gr_poly rule above.
            pts = [to_g(x, y) for x, y in kicad_poly_shape_points(
                poly_text[:find_matching_paren(poly_text, poly_text.find('(pts'))])]
            out.extend(_closed_ring_segments(pts))
    return {r: segs for r, segs in out_by_ref.items() if segs}


def _closed_ring_segments(pts):
    """The edges of a closed ring of points, skipping a zero-length one
    (under 1 um on both axes, the gr_line and pcbnew-builder rule)."""
    segs = []
    for i in range(len(pts)):
        a, b = pts[i], pts[(i + 1) % len(pts)]
        if abs(b[0] - a[0]) >= 0.001 or abs(b[1] - a[1]) >= 0.001:
            segs.append((a, b))
    return segs


def _edge_cuts_gr_poly_points(content: str) -> List[List[Tuple[float, float]]]:
    """The points of every board-level gr_poly on Edge.Cuts, as pcbnew holds
    them (`kicad_poly_shape_points`). `content` is already pad-masked."""
    polys = []
    for m in re.finditer(r'\(gr_poly(?=[\s(])', content):
        end = find_matching_paren(content, m.start())
        blk = content[m.start():end]
        if not re.search(r'\(layer\s+"Edge\.Cuts"\)', blk):
            continue
        p = blk.find('(pts')
        if p < 0:
            continue
        pts = kicad_poly_shape_points(blk[p:find_matching_paren(blk, p)])
        if pts:
            polys.append(pts)
    return polys


def _parse_gr_polys_on_layer(content: str, layer: str) -> List[List[Tuple[float, float]]]:
    """Return the vertex list of every gr_poly drawn on the given layer."""
    content = _mask_pad_primitives(content)  # pad primitives are not board graphics
    layer_re = re.escape(layer)
    pattern = (
        r'\(gr_poly\s+\(pts\s+((?:\(xy\s+[\d.-]+\s+[\d.-]+\)\s*)+)\)'
        + _GR_ELEMENT_GAP + r'\(layer\s+"' + layer_re + r'"\)'
    )
    polys = []
    for m in re.finditer(pattern, content, re.DOTALL):
        polys.append([(float(px), float(py))
                      for px, py in re.findall(r'\(xy\s+([\d.-]+)\s+([\d.-]+)\)', m.group(1))])
    return polys


def parse_guide_paths(content: str, layer: str) -> List["GuidePath"]:
    """Parse user-drawn graphic polylines on a given layer from file content.

    Reads gr_line and gr_poly graphics on the named layer (e.g. "User.1") and
    returns them as GuidePath objects. Consecutive gr_line segments whose
    endpoints coincide are stitched into a single multi-point path.

    Args:
        content: Raw .kicad_pcb file text.
        layer: Layer name to read from (e.g. "User.1").

    Returns:
        List of GuidePath (mm coordinates). Empty if none found.
    """
    content = _mask_pad_primitives(content)  # pad primitives are not board graphics
    layer_re = re.escape(layer)
    paths: List[GuidePath] = []

    # gr_poly: a closed polygon with a (pts (xy ..) (xy ..) ...) block.
    for points in _parse_gr_polys_on_layer(content, layer):
        if len(points) >= 2:
            paths.append(GuidePath(layer=layer, points=points, is_closed=True))

    # gr_line: collect 2-point segments, then stitch into chains.
    line_pattern = (
        r'\(gr_line\s+\(start\s+([\d.-]+)\s+([\d.-]+)\)\s+\(end\s+([\d.-]+)\s+([\d.-]+)\)'
        + _GR_ELEMENT_GAP + r'\(layer\s+"' + layer_re + r'"\)'
    )
    segments = []
    for m in re.finditer(line_pattern, content, re.DOTALL):
        segments.append(((float(m.group(1)), float(m.group(2))),
                         (float(m.group(3)), float(m.group(4)))))

    paths.extend(_chain_guide_segments(segments, layer))
    return paths


def _chain_guide_segments(segments, layer: str, tol: float = 0.01) -> List["GuidePath"]:
    """Stitch line segments into open polylines by shared endpoints."""
    if not segments:
        return []

    def approx_equal(p1, p2):
        return abs(p1[0] - p2[0]) < tol and abs(p1[1] - p2[1]) < tol

    remaining = list(segments)
    paths: List[GuidePath] = []
    while remaining:
        a, b = remaining.pop(0)
        chain = [a, b]
        extended = True
        while extended:
            extended = False
            for i, (s, e) in enumerate(remaining):
                if approx_equal(chain[-1], s):
                    chain.append(e)
                elif approx_equal(chain[-1], e):
                    chain.append(s)
                elif approx_equal(chain[0], e):
                    chain.insert(0, s)
                elif approx_equal(chain[0], s):
                    chain.insert(0, e)
                else:
                    continue
                remaining.pop(i)
                extended = True
                break
        paths.append(GuidePath(layer=layer, points=chain, is_closed=False))
    return paths


def parse_keepout_zones(content: str, layer: str) -> List["GuidePath"]:
    """Parse user-drawn closed keepout polygons on a given layer from file content.

    Reads gr_poly and gr_rect graphics on the named layer (e.g. "User.2") and
    returns them as closed GuidePath objects (issue #27). Only closed regions are
    returned -- open gr_line polylines are ignored (they don't bound an area).

    Args:
        content: Raw .kicad_pcb file text.
        layer: Layer name to read from (e.g. "User.2").

    Returns:
        List of closed GuidePath (mm coordinates). Empty if none found.
    """
    content = _mask_pad_primitives(content)  # pad primitives are not board graphics
    layer_re = re.escape(layer)
    zones: List[GuidePath] = []

    # gr_poly: a closed polygon with a (pts (xy ..) (xy ..) ...) block.
    for points in _parse_gr_polys_on_layer(content, layer):
        if len(points) >= 3:
            zones.append(GuidePath(layer=layer, points=points, is_closed=True))

    # gr_rect: a rectangle given by opposite corners -> 4-vertex closed box.
    rect_pattern = (
        r'\(gr_rect\s+\(start\s+([\d.-]+)\s+([\d.-]+)\)\s+\(end\s+([\d.-]+)\s+([\d.-]+)\)'
        + _GR_ELEMENT_GAP + r'\(layer\s+"' + layer_re + r'"\)'
    )
    for m in re.finditer(rect_pattern, content, re.DOTALL):
        x1, y1, x2, y2 = (float(m.group(1)), float(m.group(2)),
                          float(m.group(3)), float(m.group(4)))
        zones.append(GuidePath(
            layer=layer,
            points=[(x1, y1), (x2, y1), (x2, y2), (x1, y2)],
            is_closed=True))

    return zones


def _chain_segments_into_contours(segments: List[Tuple[Tuple[float, float], Tuple[float, float]]],
                                   tol: float = 0.01
                                   ) -> List[List[Tuple[float, float]]]:
    """Chain line segments into closed polygon contours.

    Groups segments by connectivity (shared endpoints), chains each group
    into a closed polygon. Returns list of polygons (each a list of vertices).
    """
    if not segments:
        return []

    def approx_equal(p1, p2):
        return abs(p1[0] - p2[0]) < tol and abs(p1[1] - p2[1]) < tol

    # Build connected components using union-find
    parent = list(range(len(segments)))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(a, b):
        a, b = find(a), find(b)
        if a != b:
            parent[a] = b

    # Build spatial index: bucket endpoints for fast lookup
    bucket_size = tol * 2
    from collections import defaultdict
    endpoint_buckets = defaultdict(list)
    for i, seg in enumerate(segments):
        for pt in [seg[0], seg[1]]:
            bx = int(pt[0] / bucket_size)
            by = int(pt[1] / bucket_size)
            endpoint_buckets[(bx, by)].append(i)

    # Union segments that share endpoints
    for i, seg in enumerate(segments):
        for pt in [seg[0], seg[1]]:
            bx = int(pt[0] / bucket_size)
            by = int(pt[1] / bucket_size)
            # Check neighboring buckets
            for dbx in [-1, 0, 1]:
                for dby in [-1, 0, 1]:
                    for j in endpoint_buckets.get((bx + dbx, by + dby), []):
                        if j <= i:
                            continue
                        seg_j = segments[j]
                        if (approx_equal(pt, seg_j[0]) or approx_equal(pt, seg_j[1])):
                            union(i, j)

    # Group segments by component
    groups = defaultdict(list)
    for i in range(len(segments)):
        groups[find(i)].append(i)

    # Chain each component into a polygon
    contours = []
    for group_indices in groups.values():
        if len(group_indices) < 3:
            continue

        group_segs = [segments[idx] for idx in group_indices]

        # Build spatial index for this group's segments
        seg_buckets = defaultdict(list)
        for i, seg in enumerate(group_segs):
            for pt in [seg[0], seg[1]]:
                bx = int(pt[0] / bucket_size)
                by = int(pt[1] / bucket_size)
                seg_buckets[(bx, by)].append(i)

        # Loose ends = endpoints with no partner within tol. A group with
        # exactly two is an OPEN chain: KiCad's outline assembler
        # (ConvertOutlineToPolygon) closes it straight back to its start
        # rather than discarding it -- bus_pirate5's two panel slots close
        # across a 60mm synthesized edge. Walk such a chain from a loose
        # end (starting mid-chain consumes only one side and drops the
        # group); the self-intersection guard below then mirrors pcbnew's
        # normalization dropping sliver rings whose closure crosses their
        # own chain (the same board's panel-frame strips).
        endpoints = []
        for seg in group_segs:
            endpoints.append(seg[0])
            endpoints.append(seg[1])
        loose = [p for p in endpoints
                 if sum(1 for q in endpoints if approx_equal(p, q)) == 1]
        start_idx, start_flip = 0, False
        if len(loose) == 2:
            for i, seg in enumerate(group_segs):
                if approx_equal(seg[0], loose[0]):
                    start_idx, start_flip = i, False
                    break
                if approx_equal(seg[1], loose[0]):
                    start_idx, start_flip = i, True
                    break
        s0 = group_segs[start_idx]
        polygon = [s0[1], s0[0]] if start_flip else [s0[0], s0[1]]
        used = {start_idx}

        max_iterations = len(group_segs) * 2
        for _ in range(max_iterations):
            if len(used) >= len(group_segs):
                break
            current_end = polygon[-1]
            bx = int(current_end[0] / bucket_size)
            by = int(current_end[1] / bucket_size)

            # Take the NEAREST matching endpoint, not the first one the bucket
            # order reaches (#1074). A finely tessellated arc (the 16-chord
            # floor of _arc_to_segments on r=0.1mm: 0.0098mm chords) puts
            # several unused endpoints inside the tol box; a shared endpoint
            # of a real tessellation is exactly coincident, so nearest-wins
            # always takes the true continuation, where first-wins latched
            # onto a neighbour, dead-ended, and dropped the whole ring --
            # a footprint-carried cut-out window then never became a
            # board_cutout and check_drc could not see copper crossing it.
            best_i, best_far, best_d = -1, None, None
            for dbx in (-1, 0, 1):
                for dby in (-1, 0, 1):
                    for i in seg_buckets.get((bx + dbx, by + dby), []):
                        if i in used:
                            continue
                        seg = group_segs[i]
                        for far, near in ((seg[1], seg[0]), (seg[0], seg[1])):
                            if not approx_equal(near, current_end):
                                continue
                            d = math.hypot(near[0] - current_end[0],
                                           near[1] - current_end[1])
                            if best_d is None or d < best_d:
                                best_i, best_far, best_d = i, far, d

            if best_i < 0:
                break
            polygon.append(best_far)
            used.add(best_i)

        # Remove duplicate closing point
        gap_filled = False
        if len(polygon) > 1 and approx_equal(polygon[0], polygon[-1]):
            polygon = polygon[:-1]
        elif len(loose) == 2 and len(used) == len(group_segs):
            gap_filled = True  # open chain: ring closes polygon[-1]->polygon[0]

        if len(used) != len(group_segs) or len(polygon) < 3:
            continue
        if gap_filled:
            # pcbnew parity: a gap-filled ring whose synthesized closing edge
            # properly CROSSES its own chain is a degenerate sliver (an open
            # panel-frame strip, not a real cutout) -- polygon normalization
            # drops it on the pcbnew side, so drop it here too. "Properly"
            # means penetrating deeper than the chaining tolerance: a real
            # slot's chain can graze its closure line by a sub-micron
            # excursion at the loose ends (bus_pirate5: 0.8um), while a
            # sliver strip oscillates across it by its full ~0.1mm width.
            c1, c2 = polygon[-1], polygon[0]
            clen = ((c2[0] - c1[0]) ** 2 + (c2[1] - c1[1]) ** 2) ** 0.5
            crosses = False
            if clen > tol:
                ux, uy = (c2[0] - c1[0]) / clen, (c2[1] - c1[1]) / clen
                for k in range(1, len(polygon) - 2):
                    p1, p2 = polygon[k], polygon[k + 1]
                    # signed perpendicular distance to the closure line
                    s1 = (p1[0] - c1[0]) * uy - (p1[1] - c1[1]) * ux
                    s2 = (p2[0] - c1[0]) * uy - (p2[1] - c1[1]) * ux
                    if (s1 > 0) == (s2 > 0) or min(abs(s1), abs(s2)) <= tol:
                        continue  # same side, or grazing shallower than tol
                    # crossing point must lie within the closure segment
                    t = s1 / (s1 - s2)
                    cx = p1[0] + (p2[0] - p1[0]) * t
                    cy = p1[1] + (p2[1] - p1[1]) * t
                    along = (cx - c1[0]) * ux + (cy - c1[1]) * uy
                    if tol < along < clen - tol:
                        crosses = True
                        break
            if crosses:
                continue
            area = 0.0
            perim = 0.0
            for k in range(len(polygon)):
                x1, y1 = polygon[k]
                x2, y2 = polygon[(k + 1) % len(polygon)]
                area += x1 * y2 - x2 * y1
                perim += ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
            area = abs(area) / 2.0
            # Mean-thickness sliver rule: a gap-filled ring thinner than any
            # manufacturable cutout (fab min slot ~0.8mm) is a V-cut / tab
            # guide LINE, not a hole -- pcbnew's normalization drops these
            # (bus_pirate5's three panel-frame strips, 0.013-0.24mm thick,
            # vs its real slots at ~3.8mm; the rule only applies to rings we
            # closed OURSELVES, never to deliberately drawn thin rings).
            if perim < tol or (2.0 * area) / perim < 0.5:
                continue
        contours.append(polygon)

    return contours


def _pt_in_ring(x: float, y: float, ring) -> bool:
    """Ray-cast point-in-polygon for a vertex ring."""
    inside = False
    n = len(ring)
    j = n - 1
    for i in range(n):
        xi, yi = ring[i]
        xj, yj = ring[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


def _pad_carries_copper_layer(pad) -> bool:
    """An NPTH pad never carries copper, whatever layers it lists; any other
    pad does when it names a copper layer (`check_drc._pad_has_no_copper`'s
    rule, negated; this module cannot import check_drc)."""
    if getattr(pad, 'pad_type', '') == 'np_thru_hole':
        return False
    return any(ln == '*.Cu' or ln.endswith('.Cu')
               for ln in (getattr(pad, 'layers', None) or ()))


def drop_pad_containing_cutouts(board_info, pads_by_net):
    """Drop bogus board cutouts -- a genuine cutout is a HOLE in the copper and
    contains no pads.

    _classify_contours marks ANY Edge.Cuts ring nested inside the outline as a
    cutout (by containment). That over-classifies: a large inner Edge.Cuts
    contour -- a frame line, an internal keep-out drawn on Edge.Cuts, a milled
    pocket that isn't a through hole -- then swallows the board interior, so
    make_off_board_test/_point_on_board reports every enclosed pad as off-board
    and drop_off_board_pads (#291) removes it. bus_pirate5: a 60.8x80.2mm inner
    contour dropped all 870 pads, every net lost its endpoints, and the router
    ground for ~3h on 0-endpoint nets before the run's time cap killed it (no
    output -> chain break). A real cutout encloses no pad centres, so a "cutout"
    that contains >=2 of them is a mis-classified inner contour.

    It is RECLASSIFIED, not discarded (#505). The contour is still Edge.Cuts --
    KiCad mills the board along it and grades copper against it -- so dropping
    it outright blinded BOTH the router's edge keep-out and check_drc's edge
    measurement, and copper landed ON the milled line (kicad-cli "actual 0.0000
    mm"): crkbd's router laid 85 items of copper across the milled gap between
    its two halves, none of which check_drc could see. Moving it to
    board_edge_contours keeps the #291 rescue (no pad is marked off-board) while
    restoring the edge clearance the geometry actually demands.

    Only pads that CARRY COPPER are counted (#1607). An NPTH hole is itself a
    hole, and nothing #291 protects (a routing endpoint) is lost when one
    reads off-board. crkbd's 64-point ring is that milled gap -- the slot
    between its two halves and the panel rails, enclosing no copper pad --
    and it was dropped above, then reclassified, only because two NPTH
    mouse-bite holes of its breakaway tabs (BT2, BT4) sit on its edge. So the
    board's interior depended on where other parts sat: piled, the tabs left,
    the gap became a hole again, and the locked USB-C J1 at an unchanged pose
    read a 4.63 mm overhang into it there and 0.0 on the placed board.
    """
    cutouts = getattr(board_info, 'board_cutouts', None)
    if not cutouts or not pads_by_net:
        return
    pad_xy = [(p.global_x, p.global_y)
              for pads in pads_by_net.values() for p in pads
              if _pad_carries_copper_layer(p)]
    if not pad_xy:
        return
    kept, dropped = [], []
    for c in cutouts:
        if len(c) < 3:
            kept.append(c)
            continue
        n_inside = 0
        for (px, py) in pad_xy:
            if _pt_in_ring(px, py, c):
                n_inside += 1
                if n_inside >= 2:
                    break
        (dropped if n_inside >= 2 else kept).append(c)
    if dropped:
        board_info.board_cutouts = kept
        # Keep them as milled EDGES (#505): not holes, but copper must still
        # hold its board-edge clearance from them.
        board_info.board_edge_contours = list(
            getattr(board_info, 'board_edge_contours', None) or []) + dropped
        print(f"WARNING: reclassified {len(dropped)} Edge.Cuts contour(s) mis-read "
              f"as board cutout(s) -- each encloses pads, so it is an inner outline "
              f"/ keep-out, not a hole (as a cutout it would have marked the "
              f"enclosed pads off-board). Kept as a milled edge for clearance.")


def _classify_contours(contours):
    """Split closed Edge.Cuts contours into (outers, cutouts) by CONTAINMENT.

    The old rule -- largest contour is THE outline, everything else a cutout --
    breaks boards with several disjoint outlines in one file (split keyboards:
    crkbd's right half became a "cutout", so every track in it graded as a
    board-edge violation, issue #304). A contour is a CUTOUT only if it lies
    inside another contour; disjoint top-level contours are all outer
    boundaries. Containment is decided by majority vote over sampled vertices
    (robust to shared/touching vertices). Nested islands (ring inside a cutout)
    are rare and classified as cutouts -- conservative.
    Returns (outers sorted largest-first, cutouts).
    """
    def bbox_area(c):
        xs = [p[0] for p in c]
        ys = [p[1] for p in c]
        return (max(xs) - min(xs)) * (max(ys) - min(ys))

    outers, cutouts = [], []
    for i, c in enumerate(contours):
        samples = c[:: max(1, len(c) // 5)][:5]
        contained = False
        for j, other in enumerate(contours):
            if j == i or len(other) < 3:
                continue
            votes = sum(1 for (px, py) in samples if _pt_in_ring(px, py, other))
            if votes * 2 > len(samples):
                contained = True
                break
        (cutouts if contained else outers).append(c)
    outers.sort(key=bbox_area, reverse=True)
    return outers, cutouts


def extract_board_contours(content: str) -> Tuple[List[List[Tuple[float, float]]], List[List[Tuple[float, float]]]]:
    """Extract board outline and cutout polygons from Edge.Cuts layer.

    Returns:
        (outers, cutouts): outers is the list of OUTER boundary rings (largest
        first -- several on split-keyboard/panelized boards, issue #304), and
        cutouts the truly-contained interior rings. outers is empty if no
        outline is found or if it's a simple axis-aligned rectangle.
    """
    segments = _collect_edge_cuts_segments(content)

    if len(segments) < 3:
        return [], []

    # Check if this is a simple 4-segment rectangle (no need for polygon handling)
    if len(segments) == 4:
        vertices = set()
        for seg in segments:
            vertices.add((round(seg[0][0], 3), round(seg[0][1], 3)))
            vertices.add((round(seg[1][0], 3), round(seg[1][1], 3)))
        if len(vertices) == 4:
            all_axis_aligned = all(
                abs(s[0][0] - s[1][0]) < 0.001 or abs(s[0][1] - s[1][1]) < 0.001
                for s in segments
            )
            if all_axis_aligned:
                return [], []  # Simple rectangle, use bounding box

    contours = _chain_segments_into_contours(segments)
    if not contours:
        return [], []

    outers, cutouts = _classify_contours(contours)
    if not outers:
        return [], []
    return outers, cutouts


def extract_nets(content: str, kicad_version: int = 0) -> Tuple[Dict[int, Net], Dict[str, int]]:
    """Extract all net definitions.

    Returns:
        Tuple of (nets dict keyed by net_id, name_to_id mapping).
        With a numeric net table, net_id comes from the file. Without one
        (KiCad 10 name nets), synthetic IDs are assigned.

    The encoding is decided from the CONTENT, not ``kicad_version`` (kept for
    caller compatibility). pcbnew keeps the two paired (10.0.3 saves version
    20260206 with name nets and no table), but third-party generators and
    converters write a KiCad-10 stamp over a numeric ``(net N "name")`` table,
    and a pre-10 stamp can sit over name-only refs. Keying on the stamp
    returned ZERO nets for both.
    """
    nets = {}
    name_to_id: Dict[str, int] = {}

    if not has_numeric_net_table(content):
        # No numeric table: KiCad 10 name nets.
        # Discover all net names from their usage in pads, segments, vias, and zones.
        # Match (net "name") anywhere in the file — deduplicate to build the net list.
        # `(?:[^"\\]|\\.)*` -- not `[^"]*`, which ENDS at the first escaped
        # quote, so the whole `(net ...)` failed to match and the net never
        # entered the table. A net name may legally carry a backslash
        # (neo6502's `/GPIO22\\I2C1_SDA`) or a quote; both spellings must
        # round-trip through the same raw key every lookup site uses.
        net_pattern = r'\(net\s+"%s"\)' % _ESC_STR
        # (net "") is the canonical NO-NET (net 0), not a real net: pcbnew maps
        # it to net code 0, so synthesizing an id for it split no-net copper
        # onto a phantom net (comexpress7's two dangling F.Cu segments).
        name_to_id[""] = 0
        synthetic_id = 1
        for m in re.finditer(net_pattern, content):
            net_name = m.group(1)
            if net_name in name_to_id:
                continue  # Already seen
            nets[synthetic_id] = Net(net_id=synthetic_id,
                                     name=_unescape_kicad_string(net_name))
            name_to_id[net_name] = synthetic_id
            synthetic_id += 1
        # #1290: a footprint written in the older unquoted spelling names its
        # pad's net bare -- `(net GND)` or `(net 3 GND)` -- and pcbnew reads
        # it as that name. A second pass, so a file with no bare names keeps
        # exactly the ids above.
        for m in re.finditer(_BARE_NET_REF_RE, content):
            net_name = m.group(1)
            if net_name in name_to_id:
                continue
            nets[synthetic_id] = Net(net_id=synthetic_id, name=net_name)
            name_to_id[net_name] = synthetic_id
            synthetic_id += 1
    else:
        # Numeric table (KiCad 9, whatever the stamp says): (net <id> "name")
        net_pattern = r'\(net\s+(\d+)\s+"%s"\)' % _ESC_STR
        for m in re.finditer(net_pattern, content):
            net_id = int(m.group(1))
            net_name = m.group(2)
            # Display name unescaped (pcbnew parity); raw name keeps resolving
            # in-file references.
            nets[net_id] = Net(net_id=net_id, name=_unescape_kicad_string(net_name))
            name_to_id[net_name] = net_id
        # #1290: `(net N NAME)` with the name bare, as the older spelling
        # writes it. Never overrides an id the quoted pass above declared.
        for m in re.finditer(r'\(net\s+(\d+)\s+([^\s()"]+)\)', content):
            net_id = int(m.group(1))
            if net_id in nets or m.group(2) in name_to_id:
                continue
            nets[net_id] = Net(net_id=net_id, name=m.group(2))
            name_to_id[m.group(2)] = net_id

    return nets, name_to_id


def _parse_ref_label(fp_text: str, ref_start: int,
                     is_property: bool) -> Optional[RefLabel]:
    """Parse the Reference text sub-node at ``ref_start`` into a RefLabel (#481).

    Scoped with find_matching_paren to the Reference node's OWN slice before
    any regex runs, so a sibling property's ``(at ...)`` / ``(effects ...)``
    can never bleed in (the same discipline the courtyard reader learned in
    #456 item 3). Missing effects fall back to KiCad's stock label font
    (size 1.0, thickness 0.15).
    """
    ref_text = fp_text[ref_start:find_matching_paren(fp_text, ref_start)]
    at_match = AT_NODE_RE.search(ref_text)
    if not at_match:
        return None
    layer_match = re.search(r'\(layer\s+"([^"]+)"\)', ref_text)

    # Visibility: KiCad 9+ writes (hide yes); KiCad <=8 (property nodes
    # included) wrote a bare `hide` token. Strip quoted strings first so a
    # pathological reference VALUE containing the word cannot match.
    hidden = bool(re.search(r'\(hide\s+yes\)', ref_text))
    if not hidden:
        no_strings = re.sub(r'"(?:[^"\\]|\\.)*"', '""', ref_text)
        hidden = bool(re.search(r'(?<![\w(])hide\b', no_strings))

    size_h = size_w = 1.0
    thickness = 0.15
    justify = ""
    eff_match = re.search(r'\(effects\b', ref_text)
    if eff_match:
        eff_text = ref_text[eff_match.start():
                            find_matching_paren(ref_text, eff_match.start())]
        size_match = re.search(r'\(size\s+([\d.-]+)\s+([\d.-]+)\)', eff_text)
        if size_match:
            # File order is (size HEIGHT WIDTH) -- probed via a pcbnew
            # SetTextSize(w=2, h=1) writeback, which lands as (size 1 2).
            size_h = float(size_match.group(1))
            size_w = float(size_match.group(2))
        thick_match = re.search(r'\(thickness\s+([\d.-]+)\)', eff_text)
        if thick_match:
            thickness = float(thick_match.group(1))
        justify_match = re.search(r'\(justify\s+([^)]*)\)', eff_text)
        if justify_match:
            justify = ' '.join(justify_match.group(1).split())

    return RefLabel(
        at_x=float(at_match.group(1)),
        at_y=float(at_match.group(2)),
        rotation=(float(at_match.group(3)) if at_match.group(3) else 0.0) % 360,
        size_h=size_h,
        size_w=size_w,
        thickness=thickness,
        layer=layer_match.group(1) if layer_match else "F.SilkS",
        justify=justify,
        hidden=hidden,
        is_property_node=is_property,
    )


#: KiCad 8+ property form, then the KiCad 6/7 fallback. Both require at least
#: one character, so an EMPTY `(property "Reference" "")` falls through to the
#: uuid branch -- deliberate, and load-bearing (see #78 and the reference-less
#: note in `footprint_raw_reference`).
_FP_REF_RE = re.compile(r'\(property\s+"Reference"\s+"([^"]+)"')
_FP_REF_LEGACY_RE = re.compile(r'\(fp_text\s+reference\s+"([^"]+)"')
_FP_START_RE = re.compile(r'\(footprint\s+"')

#: Separator between a duplicated reference and its file-order ordinal (#726).
#: `~` is NOT an fnmatch metacharacter (those are `*?[]`), so `--lock 'TP4*'`
#: still covers both twins and no existing literal reference is newly shadowed:
#: measured over the tracked corpus, the parser produced 547 distinct keys
#: before this change, and ZERO of them contain `~`. `#` is NOT free, and the
#: same measurement says why -- 3 of those 547 begin with it, because it is
#: already the reference-less key prefix and `placement/labels.py` keys its
#: skip off `ref.startswith('#')`.
#:
#: A SUFFIX rather than a prefix because `placement/part_class._PREFIX_RE` is
#: `^([A-Za-z]+)`, which a prefix destroys. Measured honestly, that costs
#: nothing on THIS corpus: a prefix scheme reclassifies 0 of the 18 duplicate
#: blocks, because `classify_part` matches the footprint NAME
#: (`Fiducial1x3_transp`, `TestPoint_Pad_D1.0mm`) before it ever reads the
#: prefix. The suffix is still the right choice -- for a part whose footprint
#: name carries no class keyword, the prefix is the only channel there is --
#: but this corpus does not demonstrate the harm, and an earlier version of
#: this comment claimed it did.
DUP_REF_SEP = '~'


def _footprint_header_end(fp_text: str) -> int:
    """Index of the footprint block's first CHILD element.

    Bounds a header-only search so a PAD's or a graphic's own ``(uuid ...)``
    cannot be read as the footprint's. The reference-less branch used to run
    this search unbounded; the two agree on every corpus board, and the
    bounded form is the one that is actually correct.
    """
    end = len(fp_text)
    for tok in ('(pad', '(fp_', '(zone', '(model', '(property'):
        i = fp_text.find(tok)
        if i != -1:
            end = min(end, i)
    return end


_FP_HEAD_RE = re.compile(
    r'\(\s*(?:footprint|module)\s+(?:"(?:[^"\\]|\\.)*"|[^\s()"]+)((?:\s+[A-Za-z_]+)*)')


def footprint_head_flags(fp_text: str) -> set:
    """The bare keywords right after a footprint's name. KiCad 6 writes its
    flags there -- ``(footprint "X" locked placed (layer ...`` -- where KiCad
    7+ writes ``(locked yes)`` children; pcbnew reads both."""
    m = _FP_HEAD_RE.match(fp_text)
    return set(m.group(1).split()) if m else set()


_BARE_SHAPE_LOCK_RE = re.compile(r'\((gr_[a-z]+|fp_[a-z]+)\s+locked(?=\s*\()')


def strip_bare_shape_locks(content: str) -> str:
    """CONTENT with KiCad 6's bare ``locked`` dropped from graphic shapes:
    ``(gr_line locked (start ...`` -> ``(gr_line (start ...``. Every shape
    reader expects the geometry first, so a locked Edge.Cuts line was dropped
    and a board drawn with locked outline segments parsed with no outline.
    Graphic lock state is not modelled. Analysis copies only."""
    if ' locked' not in content:
        return content
    return _BARE_SHAPE_LOCK_RE.sub(r'(\1', content)


def footprint_uuid(fp_text: str) -> str:
    """The footprint's OWN uuid -- or its KiCad 6 (tstamp ...) as pcbnew
    converts it -- or '' when it has none."""
    return uuid_or_tstamp(fp_text[:_footprint_header_end(fp_text)])


def footprint_raw_reference(fp_text: str) -> str:
    """The reference this block claims, BEFORE duplicate disambiguation.

    KiCad 8+ uses ``(property "Reference" "R1")``; KiCad 6/7 use
    ``(fp_text reference "R1" ...)``. Without the fallback every 6/7 footprint
    got reference "?" and collapsed onto one dict key, so a whole board parsed
    as a single footprint (issue #78).

    A reference-LESS footprint (a locked NPTH drill dot; thunderscope has 86)
    is keyed by its uuid instead, because a shared '' / '?' key would collapse
    them all onto one entry and lose the rest's pads.
    """
    m = _FP_REF_RE.search(fp_text) or _FP_REF_LEGACY_RE.search(fp_text)
    if m:
        return m.group(1)
    uid = footprint_uuid(fp_text)
    return "#" + uid if uid else "?"


def disambiguate_references(raw_refs: Sequence[str]) -> List[str]:
    """Ordered raw references -> ordered UNIQUE keys, by file-order ordinal.

    The first bearer of a name keeps it; the n-th appends ``~n`` (#726). Pure
    and order-only: both parse paths call this on their own ordered list and
    therefore agree, which is the whole contract between them.

    Why the FIRST occurrence keeps the bare name, when today the LAST block is
    the one the dict retains: `placement/floorplan.py` writes `'refs'` into an
    ``--emit-intent`` file the user keeps and edits, and resolves it back with
    `fnmatch.filter`. Suffixing every occurrence would make a saved intent
    naming `TP4` resolve to nothing on a file the user never touched.

    Why uuid is NOT the key: uuids are supposed to be unique, and in real
    KiCad exports they are -- but hand-made and copy-pasted boards repeat them.
    `kicad_files/cap_chain.kicad_pcb` shares one footprint uuid across C1/C2
    and another across J1/J2, so keying on it would reintroduce exactly this
    bug on a board already in the repo (`extract_groups` reached the same
    verdict for group members). A KiCad 6/7 block carries `tstamp` rather than
    `uuid` at all.

    The ordinal is issued against BOTH the file's own names and the names
    already issued, so a board that genuinely contains `TP4`, `TP4` and
    `TP4~2` cannot be handed two identical keys whatever the file order.
    """
    taken = set(raw_refs)
    issued = set()
    seen: Dict[str, int] = {}
    out: List[str] = []
    for ref in raw_refs:
        n = seen.get(ref, 0) + 1
        seen[ref] = n
        if n == 1:
            out.append(ref)
            issued.add(ref)
            continue
        cand = '%s%s%d' % (ref, DUP_REF_SEP, n)
        while cand in taken or cand in issued:
            n += 1
            cand = '%s%s%d' % (ref, DUP_REF_SEP, n)
        out.append(cand)
        issued.add(cand)
    return out


def iter_footprint_blocks(content: str):
    """(start, end, fp_text, raw_reference, key) per block, in FILE ORDER.

    The single place that decides what a footprint block is CALLED. The text
    parser, `placement/writer.py` and `placement/parser.py` all scan the same
    file for the same blocks; before #726 each did it with its own regex and
    its own ref-keyed dict, so the writer could rewrite a block the parser had
    dropped. They resolve names here now, together or not at all.

    `end` comes from the shipped string-aware `find_matching_paren`, so a lone
    paren inside a property value (an MPN like "TCR2EF115,LM(CT") cannot run
    the scan into the next footprint (#113).

    A match that starts INSIDE the block already accepted is skipped. The
    block-END scan is string-aware and the block-START scan is a plain regex,
    so a property value containing the literal `(footprint "` matches -- and
    where that used to produce a phantom entry that last-wins quietly
    swallowed, it would now be a first-class key that SHIFTS every later
    ordinal and splits the two parse paths. Contrived, and #113 exists because
    parens in property values are not.
    """
    spans = []
    raw = []
    reach = -1
    for m in _FP_START_RE.finditer(content):
        start = m.start()
        if start < reach:
            continue
        end = _block_end(content, start)
        reach = end
        text = content[start:end]
        spans.append((start, end, text))
        raw.append(footprint_raw_reference(text))
    for (start, end, text), r, key in zip(spans, raw,
                                          disambiguate_references(raw)):
        yield start, end, text, r, key


def duplicate_reference_counts(raw_refs: Sequence[str]) -> Dict[str, int]:
    """{raw reference: how many blocks claim it}, for the ones claimed twice.

    The VALUE is the total occurrence count, not the number of extras --
    `check_assembly` prints it as "TP4 x2" and derives the block total from it.
    """
    counts: Dict[str, int] = {}
    for r in raw_refs:
        counts[r] = counts.get(r, 0) + 1
    return {r: c for r, c in counts.items() if c > 1}


def extract_footprints_and_pads(content: str, nets: Dict[int, Net],
                                name_to_id: Dict[str, int] = None,
                                duplicates: Dict[str, int] = None,
                                board_copper: Sequence[str] = None) -> Tuple[Dict[str, Footprint], Dict[int, List[Pad]]]:
    """Extract footprints and their pads with global coordinates.

    `duplicates`, when given, is FILLED with `duplicate_reference_counts` --
    an out-param rather than a third return value, because the 2-tuple is a
    documented public signature (`docs/api-kicad-parser.md`).

    `board_copper` is the board's copper layer list, which a plated pad's
    layer set takes in full (`canonical_pad_layers`); read from the file's
    layer table when not given.
    """
    footprints = {}
    pads_by_net: Dict[int, List[Pad]] = {}
    if board_copper is None:
        board_copper = _layer_table(content)[1]

    # Find all footprints - need to handle nested parentheses properly.
    # Named through iter_footprint_blocks so this path and every writer agree
    # on what each block is called, including when two blocks claim one
    # reference (#726).
    _blocks = list(iter_footprint_blocks(content))
    if duplicates is not None:
        duplicates.update(duplicate_reference_counts([b[3] for b in _blocks]))
    # #962: an explicit zero paste override means different things by version
    _file_version = detect_kicad_version(content)

    for start, end, fp_text, _raw_reference, _block_key in _blocks:

        # Extract footprint name. May be EMPTY: KiCad writes (footprint "")
        # for reference-less drill/graphic footprints (thunderscope's 86 locked
        # NPTH dots) -- requiring a non-empty name silently dropped them and
        # their hole obstacles.
        fp_name_match = re.search(r'\(footprint\s+"([^"]*)"', fp_text)
        if not fp_name_match:
            continue
        fp_name = fp_name_match.group(1)

        # Extract position and rotation: the footprint's OWN `(at ...)`, not
        # the first one in the block, in any number spelling KiCad writes.
        _pose = footprint_pose(fp_text)
        if _pose is None:
            # KiCad refuses to load such a file; the part is dropped with its
            # pads, and that must not happen in silence.
            print("WARNING: footprint %s has an (at ...) that does not parse; "
                  "the footprint and its pads are skipped" % _block_key,
                  file=sys.stderr)
            continue
        fp_x, fp_y, fp_rotation = _pose
        if _footprint_at_start(fp_text) < 0:
            # Said, not silent: a footprint with no pose of its own is not
            # something KiCad writes, and the origin is only its default.
            print("WARNING: footprint %s has no (at x y) of its own; placed at "
                  "the origin, as KiCad places it" % _block_key, file=sys.stderr)

        # Extract layer
        layer_match = re.search(r'\(layer\s+"([^"]+)"\)', fp_text)
        fp_layer = layer_match.group(1) if layer_match else "F.Cu"

        # The name this block answers to. `footprint_raw_reference` handles the
        # KiCad 8+ property form, the 6/7 `fp_text` fallback (#78) and the
        # reference-LESS `#uuid` key (thunderscope's 86 NPTH dots);
        # `disambiguate_references` then makes it unique when two blocks claim
        # one reference (#726). `build_pcb_data_from_board` runs the same two
        # functions over its own ordered list, so the GUI and file models stay
        # comparable.
        #
        # `ref_match` is still resolved locally, but only for its OFFSET: the
        # #481 label parser needs the position of the Reference sub-node, and
        # `None` there means "reference-less, so no label".
        reference = _block_key
        ref_match = _FP_REF_RE.search(fp_text)
        ref_is_property = ref_match is not None
        if not ref_match:
            ref_match = _FP_REF_LEGACY_RE.search(fp_text)

        # Extract value (component part number or value)
        value_match = re.search(r'\(property\s+"Value"\s+"([^"]+)"', fp_text)
        value = value_match.group(1) if value_match else ""

        # Extract the do-not-populate flag from the footprint (attr ...) token.
        # KiCad 7+ writes a standalone `dnp` keyword among the attr flags
        # (e.g. "(attr smd dnp exclude_from_pos_files)"). A no-pop part is an
        # open circuit, so callers must not treat its pads as bridging two nets.
        attr_match = re.search(r'\(attr\b([^)]*)\)', fp_text)
        is_dnp = bool(attr_match and re.search(r'\bdnp\b', attr_match.group(1)))
        fp_attrs = tuple(sorted(
            t for t in (attr_match.group(1).split() if attr_match else ())
            if t in FOOTPRINT_ATTR_TOKENS))
        fp_has_model = bool(re.search(r'\(model\s', fp_text))

        # Footprint-level (locked yes): appears in the block header, before the
        # first nested element. Limit the search there so a locked PAD or
        # graphic inside the footprint doesn't read as a locked footprint.
        _hdr_end = len(fp_text)
        for _tok in ('(property', '(pad', '(fp_'):
            _i = fp_text.find(_tok)
            if _i != -1:
                _hdr_end = min(_hdr_end, _i)
        # (locked yes) from KiCad 7; (locked) in files 20210108-20210423.
        is_locked = (bool(re.search(r'\(locked(?:\s+yes)?\)', fp_text[:_hdr_end]))
                     or 'locked' in footprint_head_flags(fp_text))  # KiCad 6: bare

        # Footprint-level (clearance ...) override (issue #326). KiCad writes it
        # in the footprint header, after the properties but before any graphic/
        # pad/zone child, so bound the search there (a PAD's own (clearance ...)
        # or a custom pad's (clearance outline) must not match). Negative
        # overrides (KiCad allows them to SHRINK the netclass clearance) are
        # clamped to 0: this codebase models overrides only as keep-out floors,
        # so a shrinking override safely degrades to "no override".
        _clr_end = len(fp_text)
        for _tok in ('(pad', '(fp_', '(zone', '(model'):
            _i = fp_text.find(_tok)
            if _i != -1:
                _clr_end = min(_clr_end, _i)
        fp_clr_match = re.search(r'\(clearance\s+(-?[\d.]+)\)', fp_text[:_clr_end])
        fp_clearance = max(0.0, float(fp_clr_match.group(1))) if fp_clr_match else 0.0
        # #962: footprint-level paste overrides. Same header bound as the
        # clearance above, so a PAD's own token cannot be mistaken for one.
        fp_paste_margin, fp_paste_ratio = _paste_overrides(fp_text[:_clr_end],
                                                           _file_version)
        # Footprint-level zone connection override. KiCad writes it right
        # after the header (clearance ...); its own bound, because the one
        # above stops at '(zone', which is a prefix of '(zone_connect'.
        _zc_child = re.search(r'\((?:pad\s|fp_|zone\s|model\s)', fp_text)
        fp_zone_connection = zone_connect_from_text(
            fp_text[:_zc_child.start()] if _zc_child else fp_text)

        # Net-tie pad groups: (net_tie_pad_groups "1, 2" "3, 4") -- each quoted
        # string is one comma-separated group of pad numbers this footprint
        # deliberately shorts (Kelvin shunt / net-tie). Whitespace around the
        # numbers varies by KiCad version; strip it.
        net_tie_groups: List[List[str]] = []
        ntpg_match = re.search(r'\(net_tie_pad_groups\b([^)]*)\)', fp_text)
        group_strings = (re.findall(r'"([^"]*)"', ntpg_match.group(1))
                         if ntpg_match else [])
        if (not ntpg_match and _file_version
                and _file_version <= LEGACY_NET_TIES):
            # A legacy net tie is declared by keyword: pcbnew turns a footprint
            # whose (tags ...) START with "net tie" into one group of every pad
            # (NetTie parts, and the library's BRIDGED solder jumpers).
            _tag_end = min([i for i in (fp_text.find(t) for t in
                                        ('(pad', '(fp_', '(zone', '(model'))
                            if i != -1] or [len(fp_text)])
            _tags = re.search(r'\(tags\s+"((?:[^"\\]|\\.)*)"\)', fp_text[:_tag_end])
            if _tags and _unescape_kicad_string(_tags.group(1)).startswith('net tie'):
                group_strings = [', '.join(
                    q or b for q, b in re.findall(
                        r'\(pad\s+(?:"((?:[^"\\]|\\.)*)"|([^\s()"]+))', fp_text))]
        for grp in group_strings:
            pads_in_group = [p.strip() for p in grp.split(',') if p.strip()]
            if len(pads_in_group) >= 2:
                net_tie_groups.append(pads_in_group)

        # Own uuid + schematic sheet path, for placement grouping (#459). Both
        # sit in the footprint header; bound the uuid search to before the first
        # child so a PAD's or graphic's own (uuid ...) cannot win.
        _uid_end = len(fp_text)
        for _tok in ('(pad', '(fp_', '(zone', '(model', '(property'):
            _i = fp_text.find(_tok)
            if _i != -1:
                _uid_end = min(_uid_end, _i)
        fp_uuid = uuid_or_tstamp(fp_text[:_uid_end])  # KiCad 6: (tstamp ...)
        _path_m = re.search(r'\(path\s+"([^"]*)"', fp_text)
        fp_path = _path_m.group(1) if _path_m else ""
        # #1211: the sheet's NAME and FILE, either spelling (see
        # Footprint.sheet_name), read from the header only -- before the
        # first pad -- so a pad or graphic cannot lend a block a sheet.
        fp_sheet_name, fp_sheet_file = _footprint_sheet_fields(fp_text, _file_version)

        # Reference-label geometry (#481): parsed from the Reference sub-node's
        # own slice. None for the uuid-keyed reference-less footprints above.
        ref_label = (_parse_ref_label(fp_text, ref_match.start(),
                                      ref_is_property)
                     if ref_match else None)

        footprint = Footprint(
            reference=reference,
            footprint_name=fp_name,
            x=fp_x,
            y=fp_y,
            rotation=fp_rotation,
            layer=fp_layer,
            value=value,
            dnp=is_dnp,
            locked=is_locked,
            clearance=fp_clearance,
            uuid=fp_uuid,
            sheet_path=fp_path,
            net_tie_groups=net_tie_groups,
            ref_label=ref_label,
            paste_margin=fp_paste_margin,
            paste_margin_ratio=fp_paste_ratio,
            zone_connection=fp_zone_connection,
            parsed_pose=(fp_x, fp_y, fp_rotation, fp_layer),
            attrs=fp_attrs,
            has_model=fp_has_model,
            sheet_name=fp_sheet_name,
            sheet_file=fp_sheet_file,
        )

        # Extract pads
        # Pattern for pad: (pad "num" type shape ... (at x y [rot]) ... (size sx sy) ... (net id "name") ...)
        pad_pattern = r'\(pad\s+"([^"]*)"\s+(\w+)\s+(\w+)(.*?)\)\s*(?=\(pad|\(model|\(zone|\Z|$)'

        # Simpler approach: find pad starts and extract info
        # Note: pad number can be empty string (pad "") so use [^"]* not [^"]+
        for pad_match in PAD_HEAD_RE.finditer(fp_text):
            pad_start = pad_match.start()
            # Find end of this pad block (string-aware paren matching).
            pad_end = find_matching_paren(fp_text, pad_start)

            pad_text = fp_text[pad_start:pad_end]

            pad_num = (pad_match.group(1) if pad_match.group(1) is not None
                       else pad_match.group(2))
            pad_type = pad_match.group(3)  # smd, thru_hole, etc.
            pad_shape = pad_match.group(4)  # circle, rect, roundrect, etc.

            # Extract pad local position and rotation
            pad_at_match = AT_NODE_RE.search(pad_text)
            if not pad_at_match:
                continue

            local_x = float(pad_at_match.group(1))
            local_y = float(pad_at_match.group(2))
            pad_rotation = float(pad_at_match.group(3)) if pad_at_match.group(3) else 0.0

            # #369 A4: the file's (at ... angle) is the pad's ABSOLUTE board
            # angle (3d197c0, verified physically) -- adding fp_rotation
            # double-counted it, so Pad.rotation was wrong on every rotated
            # footprint and pad_drill_capsule oriented slot drills 90° off
            # (size resolution below was already fixed to the absolute angle).
            total_rotation = pad_rotation % 360

            # Extract layers, quoted (KiCad 7+) or bare (KiCad 6), as the SET
            # pcbnew loads (canonical_pad_layers): `F&B.Cu` is F.Cu + B.Cu,
            # and a plated pad is on every copper layer of the board.
            layers_section = re.search(r'\(layers\s+([^)]+)\)', pad_text)
            pad_layers = []
            if layers_section:
                pad_layers = canonical_pad_layers(
                    layer_list_tokens(layers_section.group(1)), board_copper,
                    plated_through=(pad_type == 'thru_hole'))
            # pcbnew's reader blanks the number of a pad that cannot carry
            # one -- an NPTH hole, or an SMD/connector pad with no copper (an
            # aperture) -- whatever the file wrote (PAD::CanHaveNumber;
            # ovc5_camera_carrier's KiCad 6 mounting holes are pad "1").
            if pad_type == 'np_thru_hole' or (
                    pad_type in ('smd', 'connect')
                    and not any(ln.endswith('.Cu') for ln in pad_layers)):
                pad_num = ''

            # Extract size. A KiCad 9 PADSTACK gives per-copper-layer geometry
            # inside the pad: e.g. an oval 2.54x1.27 on top but a larger custom
            # 2.6416x1.778 on B.Cu (apple1_mainboard DIP pads). The base
            # (size ...) alone UNDER-models the pad on the bigger layer -- the
            # router then drops vias against that layer's copper (apple1: a via
            # 0.027mm from a B.Cu pad KiCad flags but check_drc missed, because
            # both read only the 2.54x1.27 base). Take the MAX extent over the
            # layers the pad has COPPER on (`_padstack_copper_size`): DRC-safe,
            # slightly conservative where a layer's copper is smaller. A layer
            # the pad is not on has no copper, so its entry does not count --
            # mokya_lora_phone's SW1 pads are 0.65 mm on F.Cu, their only layer,
            # and were read as their 1.905 x 1.27 inner/back entries.
            sizes = re.findall(r'\(size\s+([\d.-]+)\s+([\d.-]+)\)', pad_text)
            size_x, size_y = _padstack_copper_size(
                pad_text, (float(sizes[0][0]), float(sizes[0][1])) if sizes
                else (0.5, 0.5), pad_layers, board_copper)
            # #962: a custom pad's ANCHOR size (the base `(size ...)`), in the
            # pad frame. KiCad sizes the paste-ratio term from it.
            _anchor_size = ((float(sizes[0][0]), float(sizes[0][1]))
                            if pad_shape == 'custom' and sizes else None)

            # Custom pads: enclose the real primitive copper, not just the anchor
            # (size ...). Use a centred rect around the connection point that
            # covers the full local extent -- conservative (may over-block the
            # empty side of an asymmetric pad) but DRC-safe; modelling only the
            # anchor lets tracks graze copper that reaches past it.
            # A custom pad whose copper IS its anchor (`_custom_copper_is_anchor`)
            # is sized as the anchor rect, with its residual tilt, below.
            custom_resolved = False
            copper_box = None
            if pad_shape == 'custom' and not _custom_copper_is_anchor(
                    pad_text, size_x, size_y):
                # pad_rotation: the file's (at ...) angle is the pad's ABSOLUTE
                # board angle (same convention as _custom_pad_global_polygons /
                # _resolve_pad_rect).
                ext_bx, ext_by = _custom_pad_board_extent(
                    pad_text, pad_rotation, size_x, size_y)
                _box = _custom_pad_board_box(pad_text, pad_rotation, size_x, size_y)
                if _box is not None:
                    copper_box = copper_box_if_offset(*_box)
                if ext_bx > 0 or ext_by > 0:
                    # Board-frame symmetric box about the anchor, matching the
                    # pcbnew builder's convention exactly (rect_rotation 0) --
                    # a rotated custom pad otherwise modelled a different
                    # rectangle on the two paths.
                    size_x, size_y = 2.0 * ext_bx, 2.0 * ext_by
                    rect_rotation = 0.0
                    custom_resolved = True

            # Resolve the pad rectangle into board space. size_x/size_y are in
            # the pad's own frame; its absolute board orientation is the file's
            # pad_rotation (the (at ...) angle already includes the footprint's
            # rotation -- 3d197c0). For the common orthogonal cases bake the
            # rotation into the dimensions so all downstream axis-aligned
            # geometry stays exact; genuine diagonal pads keep a residual
            # rect_rotation the obstacle/DRC geometry applies.
            _frame_size = (size_x, size_y)  # pad frame, for a chamfer (#1450)
            if not custom_resolved:
                size_x, size_y, rect_rotation = _resolve_pad_rect(size_x, size_y, pad_rotation)

            # Extract net: KiCad 9 `(net N "name")`, KiCad 10 `(net "name")`,
            # or either one unquoted (#1290).
            net_id, net_name = pad_net_from_text(pad_text, name_to_id)

            # Extract pinfunction
            pinfunc_match = re.search(r'\(pinfunction\s+%s\)' % _STR_OR_BARE, pad_text)
            pinfunction = ((pinfunc_match.group(1) if pinfunc_match.group(1) is not None
                            else pinfunc_match.group(2)) if pinfunc_match else "")

            # Extract pintype
            pintype_match = re.search(r'\(pintype\s+%s\)' % _STR_OR_BARE, pad_text)
            pintype = ((pintype_match.group(1) if pintype_match.group(1) is not None
                        else pintype_match.group(2)) if pintype_match else "")

            # Extract drill size for through-hole pads. Oval/slot drills are
            # (drill oval w h); take max(w, h) so the pad keeps drill>0 (TH)
            # semantics instead of being misread as SMD (issue #106).
            # Numbers in AT_NUM form: KiCad writes a value under 1e-4 with an
            # exponent, and arty_mpw_tester's Flexypin pads carry
            # `(drill 1e-06)`, which `[\d.]+` read as a 1 mm hole.
            oval_match = re.search(r'\(drill\s+oval\s+(%s)(?:\s+(%s))?' % (AT_NUM, AT_NUM),
                                   pad_text)
            drill_w = drill_h = 0.0
            if oval_match:
                # (drill oval w) with a single value is legal and means w x w
                # (dilemma J102) -- requiring two numbers parsed it as drill 0.
                drill_w = float(oval_match.group(1))
                drill_h = float(oval_match.group(2)) if oval_match.group(2) else drill_w
                drill_size = max(drill_w, drill_h)
            else:
                drill_match = re.search(r'\(drill\s+(%s)' % AT_NUM, pad_text)
                drill_size = float(drill_match.group(1)) if drill_match else 0.0

            # Pad copper offset (#324): (drill (offset x y)) displaces the
            # pad COPPER relative to the pad anchor/hole (castellated module
            # paddles: RPi_Pico's 3.5x1.7 SMD pads carry (offset 0.9 0)).
            # Ignoring it modelled the copper up to its full offset away from
            # reality -- the router laid tracks THROUGH real pad copper and
            # check_drc graded the shorts clean (piantor).
            offset_match = re.search(r'\(offset\s+([-\d.]+)\s+([-\d.]+)\)', pad_text)
            pad_offset_x = float(offset_match.group(1)) if offset_match else 0.0
            pad_offset_y = float(offset_match.group(2)) if offset_match else 0.0

            # Extract roundrect_rratio for roundrect pads
            rratio_match = re.search(r'\(roundrect_rratio\s+([\d.]+)\)', pad_text)
            roundrect_rratio = float(rratio_match.group(1)) if rratio_match else 0.0

            # Extract per-pad local clearance override, e.g. fiducial keep-clear
            # rings carry (clearance 0.375). The leading "(" avoids matching the
            # footprint's (pad_to_mask_clearance ...) token. 0 = no override
            # (KiCad's explicit "(clearance 0)" also means inherit). A pad with
            # no override of its own inherits the FOOTPRINT-level override
            # (issue #326) -- resolved here so every consumer of
            # pad.local_clearance honors both. Negative (shrinking) overrides
            # clamp to 0 = no override (see the footprint-level comment).
            clr_match = re.search(r'\(clearance\s+(-?[\d.]+)\)', pad_text)
            local_clearance = max(0.0, float(clr_match.group(1))) if clr_match else 0.0
            if local_clearance == 0.0:
                local_clearance = fp_clearance
            # #962: the pad's own paste overrides (raw; inheritance is resolved
            # by paste_apertures, not here, so both parse paths share it).
            _pad_paste = _paste_overrides(pad_text, _file_version)
            # #1248: pad-to-die length/delay, parity with the pcbnew path.
            _pad_die = pad_to_die_from_text(pad_text)

            # Calculate global coordinates
            global_x, global_y = local_to_global(fp_x, fp_y, fp_rotation, local_x, local_y)

            # Fold the copper offset into the pad position (#324/#325): the
            # shifted rect IS the pad copper, so every clearance/DRC/obstacle
            # consumer gets it for free. The offset rotates with the pad's
            # ABSOLUTE angle (the file's (at) angle; same negate convention as
            # local_to_global). For drilled pads the HOLE stays at the anchor:
            # recorded in hole_x/hole_y for the drill-geometry consumers
            # (pad_drill_capsule / through-hole cells). caravel U8 proved the
            # drilled case is not rare: its THT pins carry a 0.3mm copper
            # offset, and modelling the copper at the hole shipped 89 real
            # cross-net shorts that check_drc graded clean.
            pad_hole_x = pad_hole_y = None
            if pad_offset_x or pad_offset_y:
                if drill_size > 0:
                    pad_hole_x, pad_hole_y = global_x, global_y
                _orad = math.radians(-pad_rotation)
                global_x += pad_offset_x * math.cos(_orad) - pad_offset_y * math.sin(_orad)
                global_y += pad_offset_x * math.sin(_orad) + pad_offset_y * math.cos(_orad)

            # Custom comb/finger pads: carry the real copper polygon(s) so the
            # obstacle map / DRC model the notches instead of the bounding box
            # (issue #188). pad_rotation here is the file's pad (at) angle, which
            # is the pad's absolute board orientation (folds in fp rotation).
            pad_polygons = None
            if pad_shape == 'custom':
                pad_polygons = _custom_pad_global_polygons(
                    pad_text, global_x, global_y, pad_rotation)
            elif '(chamfer' in pad_text:
                # A chamfered rect (#1450): its cut corners are not copper,
                # and every consumer that reads `polygons` measures them so.
                # Shape and ratio as pcbnew loads it (a chamfered `rect` is
                # its roundrect).
                _chamfered = chamfered_pad_from_text(
                    pad_text, pad_shape, _frame_size[0], _frame_size[1],
                    global_x, global_y, pad_rotation)
                if _chamfered is not None:
                    pad_polygons, roundrect_rratio = _chamfered
                    pad_shape = 'roundrect'

            pad = Pad(
                component_ref=reference,
                pad_number=pad_num,
                global_x=global_x,
                global_y=global_y,
                local_x=local_x,
                local_y=local_y,
                size_x=size_x,
                size_y=size_y,
                shape=pad_shape,
                layers=pad_layers,
                net_id=net_id,
                net_name=net_name,
                rotation=total_rotation,
                pinfunction=pinfunction,
                pintype=pintype,
                pad_type=pad_type,
                drill=drill_size,
                drill_w=drill_w,
                drill_h=drill_h,
                roundrect_rratio=roundrect_rratio,
                rect_rotation=rect_rotation,
                local_clearance=local_clearance,
                polygons=pad_polygons,
                hole_x=pad_hole_x,
                hole_y=pad_hole_y,
                castellated='pad_prop_castellated' in pad_text,
                geometry_approximations=tuple(reason for token, reason in (
                    ('chamfer', 'chamfered pad'),
                    ('padstack', 'per-layer padstack'))
                    if re.search(r'\(' + token + r'\s', pad_text)),
                paste_margin=_pad_paste[0],
                paste_margin_ratio=_pad_paste[1],
                anchor_size=_anchor_size,
                unconnected_layer_mode=unconnected_layer_mode_from_text(pad_text),
                die_length=_pad_die[0],
                die_delay=_pad_die[1],
                copper_box=copper_box,
                zone_connection=(zone_connect_from_text(pad_text)
                                 or fp_zone_connection),
            )

            footprint.pads.append(pad)

            # Add to pads_by_net
            if net_id not in pads_by_net:
                pads_by_net[net_id] = []
            pads_by_net[net_id].append(pad)

            # Also add to Net object
            if net_id in nets:
                nets[net_id].pads.append(pad)

        footprints[reference] = footprint

    return footprints, pads_by_net



def extract_groups(content: str, footprints: Dict[str, 'Footprint']) -> Dict[str, List[str]]:
    """KiCad ``(group "name" (uuid ...) (members <uuid> ...))`` -> {name: [ref]}.

    Groups are the designer's own statement that a set of parts belongs
    together, which is why placement grouping ranks them above any inferred
    signal (#459). Members are listed BY UUID; anything that does not resolve to
    a footprint (a zone, a graphic, a nested group) is dropped rather than
    guessed at.

    Note the group's own ``(uuid ...)`` is its identity, NOT a member -- read the
    members list explicitly rather than scooping every uuid in the block. The
    block is bounded with find_matching_paren, not a flat regex, because
    ``(members ...)`` is a nested s-expression (#113).

    No board in the in-repo corpus has a single ``(group ...)`` block, so an
    empty result is the normal case, not a parse failure.

    A uuid is supposed to be unique, and in real KiCad exports it is -- but
    hand-made and copy-pasted boards do repeat them (``cap_chain`` shares one
    uuid across two footprints, and another across two more). A uuid->ref dict
    would silently drop all but the last, so a member could resolve to the WRONG
    part. Map to every ref that claims the uuid instead: they are genuinely
    indistinguishable, and pulling both into the group is the conservative
    reading.
    """
    by_uuid: Dict[str, List[str]] = {}
    for ref, fp in footprints.items():
        if fp.uuid:
            by_uuid.setdefault(fp.uuid, []).append(ref)
    out: Dict[str, List[str]] = {}
    for m in re.finditer(r'\(group\s+"([^"]*)"', content):
        block = content[m.start():find_matching_paren(content, m.start())]
        mem = re.search(r'\(members\b', block)
        if not mem:
            continue
        members = block[mem.start():find_matching_paren(block, mem.start())]
        refs = [r for u in re.findall(r'"?([0-9a-fA-F-]{36})"?', members)
                for r in by_uuid.get(u, ())]
        if refs:
            # Sorted + de-duplicated: this order reaches the placement engine,
            # and anything that does must be deterministic (#457).
            out[m.group(1)] = sorted(set(refs))
    return out


# Every child KiCad's teardrop reader knows (parseTEARDROP_PARAMETERS).
_TEARDROP_KEYS = frozenset((
    'best_length_ratio', 'max_length', 'best_width_ratio', 'max_width',
    'curved_edges', 'filter_ratio', 'enabled', 'allow_two_segments',
    'prefer_zone_connections'))
_TEARDROP_OPEN_RE = re.compile(r'\(teardrops?(?=[\s(])')
_TOKEN_POS_RE = re.compile(r'"(?:[^"\\]|\\.)*"|[()]|[^\s()"]+')


def repair_bare_teardrop_tokens(content: str) -> Tuple[str, int]:
    """Give a teardrop child written without its ``(`` the paren KiCad reads
    into it (#1149). Returns ``(content, number of tokens repaired)``.

    KiCad's teardrop reader takes each child's opening paren as OPTIONAL --
    ``if( token == T_LEFT ) token = NextTok();`` before the keyword switch -- so
    ``(curved_edges no)filter_ratio 0.9)`` loads as ``(filter_ratio 0.9)``, and
    the ``)`` that looks surplus is that child's own close. Every paren-counting
    reader here took it as the pad's close instead: KiCad 10.0.0's
    RoyalBlue54L-Feather demo carries 349 such blocks, and the text parser ended
    U2 (QFN-32), U4 and U6 after their first pad (pcbnew: 59/10/21).

    The repair walks each ``(teardrop``/``(teardrops`` block token by token with
    KiCad's rule: a known key as a BARE word directly inside the block opens a
    child, which its value's ``)`` then closes. Nothing else is touched, and a
    file without the defect is returned as the same string.
    """
    if '(teardrop' not in content:
        return content, 0
    inserts: List[int] = []
    pos = 0
    for m in _TEARDROP_OPEN_RE.finditer(content):
        if m.start() < pos:
            continue
        depth = 1
        pos = m.end()
        for t in _TOKEN_POS_RE.finditer(content, m.end()):
            tok = t.group(0)
            pos = t.end()
            if tok == '(':
                depth += 1
            elif tok == ')':
                depth -= 1
                if depth == 0:
                    break
            elif depth == 1 and tok in _TEARDROP_KEYS:
                inserts.append(t.start())
                depth += 1
    if not inserts:
        return content, 0
    parts = []
    last = 0
    for i in inserts:
        parts.append(content[last:i])
        parts.append('(')
        last = i
    parts.append(content[last:])
    return ''.join(parts), len(inserts)


def reflow_one_line_board(content: str) -> Tuple[str, bool]:
    """A board written (almost) on one line, laid out the way KiCad's own
    formatter writes it: one list per line, tab-indented by depth, a list of
    atoms inline (``(at 1 2)``), a ``(pts`` list's vertices on one line, and
    every closing paren of a multi-line list on its own line.

    KiCad reads both layouts, and SolderCAD's file validator writes the
    one-line one (``...)(zone(net 1)...(layer "F.Cu" )...``). This parser's
    readers were written against the multi-line layout: on one line the zone
    iterator and the ``(layers`` table need a newline that is not there, and a
    ``(layer "F.Cu" )`` with KiCad's unprettified trailing space defeats the
    tight ``"\\)`` field patterns. Measured on the same routed board in both
    layouts: zones 1 -> 0, copper layers 2 -> 0, board bounds and outline
    gone, and a footprint's copper fp_poly (8 segments) dropped -- so
    check_connected called a poured GND net broken and the router would have
    seen no pours and no tab copper. Reflowing at the read boundary gives
    every reader the layout it was written for. Only whitespace between
    tokens changes; a file that already has its lines is returned unchanged.
    """
    if content.count('\n') * 20 >= content.count('('):
        return content, False
    root: list = []
    stack = [root]
    for tok in _TOKEN_POS_RE.findall(content):
        if tok == '(':
            node: list = []
            stack[-1].append(node)
            stack.append(node)
        elif tok == ')':
            if len(stack) == 1:
                return content, False       # unbalanced: leave it to the readers
            stack.pop()
        else:
            stack[-1].append(tok)
    if len(stack) != 1:
        return content, False

    lines: List[str] = []

    def inline(node) -> str:
        return '(' + ' '.join(inline(c) if isinstance(c, list) else c
                              for c in node) + ')'

    def emit(node, depth):
        pad = '\t' * depth
        if not any(isinstance(c, list) for c in node):
            lines.append(pad + inline(node))
            return
        head = []
        for c in node:
            if isinstance(c, list):
                break
            head.append(c)
        children = node[len(head):]
        lines.append(pad + '(' + ' '.join(head))
        if head and head[0] == 'pts':
            lines.append(pad + '\t' + ' '.join(
                inline(c) if isinstance(c, list) else c for c in children))
        else:
            for c in children:
                if isinstance(c, list):
                    emit(c, depth + 1)
                else:
                    lines.append(pad + '\t' + c)
        lines.append(pad + ')')

    for top in root:
        if isinstance(top, list):
            emit(top, 0)
        else:
            lines.append(top)
    return '\n'.join(lines) + '\n', True


def read_board_text(path: str, quiet: bool = False) -> str:
    """A .kicad_pcb's text as KiCad reads it: `repair_bare_teardrop_tokens`
    applied, with a one-line note when it changed anything (#1149), and a
    one-line board reflowed into KiCad's multi-line layout
    (`reflow_one_line_board`). For every reader that walks footprint or pad
    blocks by their parens."""
    with open(path, 'r', encoding='utf-8') as f:
        content = f.read()
    content, n = repair_bare_teardrop_tokens(content)
    if n and not quiet:
        print(f"NOTE: {n} teardrop token(s) in {os.path.basename(path)} lack their "
              f"opening paren; read the way KiCad reads them (#1149).",
              file=sys.stderr)
    content, reflowed = reflow_one_line_board(content)
    if reflowed and not quiet:
        print(f"NOTE: {os.path.basename(path)} is written on one line; read in "
              f"KiCad's multi-line layout.", file=sys.stderr)
    return content


def _via_blocks(content: str) -> List[Tuple[int, str]]:
    """``(start offset, paren-balanced text)`` for every ``(via ...)`` in a file.

    ONE scan, shared by the geometry parse and the protection-attr pass, and the
    reason both are safe (#748). The geometry regexes used to run over the WHOLE
    file, which let the KiCad-10 pattern's ``.*?`` gap run out of one via and
    into a LATER via's ``(net "...")``. On a MIXED-dialect board that is not a
    near-miss: measured, a numeric via followed by a named one parsed as two
    vias BOTH at the numeric one's coordinates -- one barrel invented, one real
    barrel swallowed. Mixed-dialect boards are not hypothetical either; this
    repo's own fanout step writes numeric refs into KiCad-10 boards, which is
    what ``tests/stress/fix_mixed_net_refs.py`` exists to undo. A pattern
    confined to a block cannot reach another via at all.

    The balanced scan stops at the via's own closing paren, so it costs the via
    text rather than the file. Deliberately NOT ``content[start:]`` per via:
    that copies the whole tail for EVERY via (O(vias x filesize)) -- the same
    quadratic shape this file already carries a guard for at #225, and what
    ``_extract_via_protection_attrs`` was doing before it shared this scan.

    String-aware, because KiCad net names may contain parentheses (``/BUS(0)``)
    and a bare depth count would end the block inside one. An UNBALANCED via
    (truncated file) is skipped: there is no field ordering to trust in a block
    whose end cannot be found.
    """
    out: List[Tuple[int, str]] = []
    n = len(content)
    for m in re.finditer(r'\(via(?=[\s(])', content):
        start = m.start()
        depth = 0
        in_str = False
        i = start
        while i < n:
            c = content[i]
            if in_str:
                if c == '\\':
                    i += 2
                    continue
                if c == '"':
                    in_str = False
            elif c == '"':
                in_str = True
            elif c == '(':
                depth += 1
            elif c == ')':
                depth -= 1
                if depth == 0:
                    out.append((start, content[start:i + 1]))
                    break
            i += 1
    return out


# Geometry prefix, identical in both net dialects: the type token (blind/micro)
# precedes (at ...), and at -> size -> drill -> layers is what KiCad and this
# repo's own writer emit.
_VIA_PREFIX_RE = re.compile(
    r'\(via\s+(?:blind\s+|micro\s+)?'
    r'\(at\s+([\d.-]+)\s+([\d.-]+)\)\s+'
    r'\(size\s+([\d.-]+)\)\s+'
    r'\(drill\s+([\d.-]+)\)\s+'
    r'\(layers\s+"([^"]+)"\s+"([^"]+)"\)')
# Either dialect, whichever the block carries. Escape-aware on the name side
# (pads and zones already are): with a plain [^"]* a net name containing an
# escaped quote ends the capture early, the pattern fails, and the via is lost
# -- the same disappearance #748 is about, by a different route.
_VIA_NET_RE = re.compile(r'\(net\s+(?:"((?:[^"\\]|\\.)*)"|(\d+))\)')
_VIA_UUID_RE = re.compile(r'\(uuid\s+"([^"]+)"\)')
_VIA_FREE_RE = re.compile(r'\(free\s+yes\)')
# KiCad's via type is a bare word of the via block (`(via micro (at ...`,
# `(via buried ...`); a block with none is a THROUGH via. KiCad 9 writes
# `blind` for its BLIND_BURIED type; KiCad 10 also writes `buried`.
_VIA_TYPE_WORDS = ('blind', 'buried', 'micro')


def _via_type_from_block(block: str) -> str:
    """'blind' / 'buried' / 'micro' from a via block's bare words, else
    'through'. KiCad reads the type from that word alone, never from
    `(layers ...)`."""
    if not any(w in block for w in _VIA_TYPE_WORDS):
        return 'through'
    return next((w for w in _sexpr_children(block)[1] if w in _VIA_TYPE_WORDS),
                'through')


def extract_vias(content: str, name_to_id: Dict[str, int] = None) -> List[Via]:
    """Extract all vias from PCB file.

    Only the GEOMETRY prefix is positional. Everything after ``(layers ...)`` --
    net, uuid, ``(free yes)``, ``(locked yes)``, the protection family -- is read
    from the block by its own token, in either order and with anything between
    (#748).

    That asymmetry used to be the bug. The KiCad-10 pattern was given a flexible
    gap before ``(net ...)`` for exactly the protection tokens KiCad 10 writes
    there; its KiCad-9 twin kept strict field ordering, so a NUMERIC-net via
    carrying any protection token matched neither pattern and vanished from the
    model. Not an invisible spec -- an invisible barrel, which the router then
    plans tracks through (the hazard PR #534 was written against). Our own
    writer produces that exact shape whenever a net name fails to resolve, and
    ``net_id_to_name`` has no key 0 on a name-net board.

    A token sitting between ``(net ...)`` and ``(uuid ...)`` used to cost the
    SPEC in both dialects for the same positional reason -- the uuid capture
    failed and the uuid-keyed protection join then missed. Reading each field
    independently retires that too, and lets a uuid-less via carry a spec at all.
    """
    vias = []
    unparsed = []
    # One linear scan: most boards mention no protection token and skip the
    # per-block spec work entirely.
    want_spec = any(('(' + t) in content for t in VIA_PROTECTION_TOKENS)

    for _start, block in _via_blocks(content):
        m = _VIA_PREFIX_RE.match(block)
        if m:
            geom = (m.group(1), m.group(2), m.group(3), m.group(4),
                    m.group(5), m.group(6))
        else:
            # The geometry prefix out of its usual order -- a (locked yes)
            # stamped inside it, or the bare `(via locked (at ...` form, both
            # of which KiCad loads (#1158). Read it by its own depth-1 tokens
            # (never a (size) inside a per-layer padstack), and REPORT a block
            # that still cannot be modelled: it used to vanish silently.
            geom, why = _via_geometry_from_tokens(block)
            if geom is None:
                unparsed.append((_start, why))
                continue
        nm = _VIA_NET_RE.search(block)
        net_name, numeric = (nm.group(1), nm.group(2)) if nm else (None, None)
        if nm is None:
            # No (net ...) at all: KiCad loads the via on net 0, and so does
            # the track path. Skipping it made a real barrel invisible -- 152
            # of RoyalBlue54L-Feather's 183 vias, whose (net) a malformed
            # teardrop block had cut off (#1149), went that way, silently.
            net_id = 0
        elif numeric is not None:
            net_id = int(numeric)
        else:
            # KiCad 10 dialect. Without the name map the id is unknowable, so
            # skip the via rather than model it on a guessed one -- unchanged
            # behavior, and parse_kicad_pcb always passes a map.
            if not name_to_id:
                continue
            # Key on the RAW file text, exactly like every other name_to_id
            # lookup (pads 3373, segments 4376, the zone/graphic sites).
            # name_to_id is BUILT from the raw text -- _unescape_kicad_string's
            # own docstring says so -- so unescaping first missed every net
            # whose name carries a backslash. neo6502's `/GPIO22\\I2C1_SDA`
            # resolved for its 11 segments and for none of its 4 vias, which
            # then modelled as net 0 and graded as 8 phantom DRC violations
            # against the net's own copper.
            net_id = name_to_id.get(net_name, 0)
        u = _VIA_UUID_RE.search(block)
        via_type = _via_type_from_block(block)
        layers = [geom[4], geom[5]]
        if via_type == 'through':
            # KiCad loads a via with no type word as THROUGH and snaps its pair
            # to F.Cu/B.Cu (probed on pcbnew 10.0.3 and 9.0.6), which is what
            # build_pcb_data_from_board then reads back. Keeping the file's
            # pair -- this repo's writer emitted `(layers "B.Cu" "In6.Cu")` for
            # back-side BGA fanout vias -- made every consumer that reads the
            # pair model a through via as a blind one, and the two parse paths
            # disagree (found reviewing #1268). A through via written
            # `(layers "B.Cu" "F.Cu")` reads F.Cu first too: pcbnew's
            # TopLayer()/BottomLayer() order is what the GUI path holds.
            layers = ['F.Cu', 'B.Cu']
        else:
            # A typed pair top layer first, as pcbnew reads it back: the two
            # paths then hold one tuple, which placement Apply's via key
            # compares (a pair written bottom first was re-created there,
            # with a new uuid, on every apply).
            from geometry_utils import _copper_order
            layers.sort(key=_copper_order)
        via = Via(
            x=float(geom[0]),
            y=float(geom[1]),
            size=float(geom[2]),
            drill=float(geom[3]),
            layers=layers,
            net_id=net_id,
            uuid=(u.group(1) if u else uuid_or_tstamp(block)),  # KiCad 6: (tstamp ...)
            # Read from the block, not from a capture group and not from a
            # second uuid-keyed pass over the file. The old v10 path needed
            # that pass (#225's O(vias x filesize) backtracking, guarded by a
            # linear pre-scan) and it could only reach vias that HAD a uuid.
            free=bool(_VIA_FREE_RE.search(block)),
            locked=_track_locked(block),
            via_type=via_type,
        )
        # Protection spec per via (#489 §8): tenting/covering/plugging/capping/
        # filling were parsed by NOBODY, so a re-placed via lost whatever the
        # board specified and got the writer's hardcoded front+back tenting.
        if want_spec:
            spec = _via_spec_from_block(block)
            if spec:
                via.tenting_attrs = spec
        vias.append(via)

    _report_unparsed_copper(content, 'via', unparsed)
    return vias


def _via_geometry_from_tokens(block: str):
    """``((x, y, size, drill, layer_a, layer_b) as strings, None)`` read from a
    via block's depth-1 tokens in any order, or ``(None, reason)`` (#1158)."""
    ch, _bare = _sexpr_children(block)
    at = next((a[:2] for a in ch.get('at', ()) if len(a) >= 2
               and all(_NUM_TOKEN_RE.match(x) for x in a[:2])), None)
    size = next((a[0] for a in ch.get('size', ()) if a and _NUM_TOKEN_RE.match(a[0])), None)
    drill = next((a[0] for a in ch.get('drill', ()) if a and _NUM_TOKEN_RE.match(a[0])), None)
    layers = next((a[:2] for a in ch.get('layers', ()) if len(a) >= 2
                   and all(x.startswith('"') for x in a[:2])), None)
    missing = [t for t, v in (('at', at), ('size', size), ('drill', drill),
                              ('layers', layers)) if v is None]
    if missing:
        return None, f"via without {'/'.join(missing)}"
    return (at[0], at[1], size, drill, layers[0][1:-1], layers[1][1:-1]), None


VIA_PROTECTION_TOKENS = ('tenting', 'covering', 'plugging', 'capping', 'filling')


# Every enum value _pcbnew_via_protection_attrs compares against. ALL of them,
# not just the two whose absence raises: a wrapper exporting the tenting pair
# and not the capping one yields a PARTIAL spec, which is a quiet loss rather
# than a loud one.
_PROTECTION_ENUM_NAMES = (
    'TENTING_MODE_TENTED', 'TENTING_MODE_NOT_TENTED',
    'COVERING_MODE_COVERED', 'COVERING_MODE_NOT_COVERED',
    'PLUGGING_MODE_PLUGGED', 'PLUGGING_MODE_NOT_PLUGGED',
    'CAPPING_MODE_CAPPED', 'CAPPING_MODE_NOT_CAPPED',
    'FILLING_MODE_FILLED', 'FILLING_MODE_NOT_FILLED')


def pcbnew_protection_accessors_usable(via=None) -> bool:
    """Whether THIS pcbnew can be asked for a via's protection spec (#751).

    KiCad 10.0.0's shipping SWIG wrapper cannot. The SETTERS are exported but
    the enum VALUES are not, and the getters hand back an opaque SwigPyObject
    ('TENTING_MODE *') that compares equal to nothing -- so building the
    comparison table raised inside `_pcbnew_via_protection_attrs`' own `try`
    and it returned {} for EVERY via, leaving the GUI half of the #489 s8 round
    trip carrying no spec at all. Measured against
    `C:/Program Files/KiCad/10.0/bin/python.exe` (pcbnew 10.0.0, wx 4.2.2).

    BUILD-dependent, not version-dependent: 10.0.0-103-gacbf1898e0 on macOS
    exports all ten constants and returns plain ints. So this is a question to
    ask the wrapper in front of us, never one to answer from GetBuildVersion().

    Both failure modes are checked, because either alone empties the answer:
    the constants must exist AND a getter must return something comparable to
    them. Pass a real PCB_VIA for the second half -- without one only the
    constants are checked, which is the weaker question.
    """
    try:
        import pcbnew
    except ImportError:
        return False
    if any(getattr(pcbnew, n, None) is None for n in _PROTECTION_ENUM_NAMES):
        return False
    if via is None:
        return True
    try:
        # A SWIG enum arrives as a plain int; the broken wrapper's SwigPyObject
        # does not, and `mode == const` on it is False for every mode.
        return isinstance(via.GetFrontTentingMode(), int)
    except Exception:
        return False


def via_protection_attrs_from_path(path) -> Dict[str, Dict[str, str]]:
    """{via uuid: spec} read from a board FILE. Shared by both live fronts.

    Split out of `via_protection_attrs_from_board_file` so the IPC builder can
    use the same scan: kipy has no `GetFileName()`, it resolves the open
    document's path through the adapter, and re-deriving the map there would
    be a second implementation of the one thing #751 exists to keep single.
    """
    try:
        if not path or not os.path.isfile(path):
            return {}
        with open(path, 'r', encoding='utf-8') as f:
            return _extract_via_protection_attrs(f.read())
    except Exception:
        return {}


def via_protection_attrs_from_board_file(board) -> Dict[str, Dict[str, str]]:
    """{via uuid: spec} read from a pcbnew BOARD's own file, for a wrapper that
    cannot be asked (#751).

    The text parser is the CLI's, which is the point: the two parse paths agree
    field-for-field because it is the same code, rather than because two
    implementations were kept in step by hand.

    The file can LAG the live board mid-plan (earlier steps apply copper to the
    board only). For this key that degrades gently in the safe direction: a via
    added in-session has no entry, so it reads as unspecified and keeps
    INHERITING, which is exactly right for new copper. Only a spec the user
    changed in the GUI and has not saved reads stale -- and that is strictly
    better than the {} every via got before.
    """
    try:
        return via_protection_attrs_from_path(board.GetFileName())
    except Exception:
        return {}


def pcbnew_via_protection_attrs(via, text_specs: Dict[str, Dict[str, str]] = None) -> Dict[str, str]:
    """A PCB_VIA's protection spec: from the live object where that works, from
    the board's file where it does not (#751).

    THE entry point for GUI-side callers. `_pcbnew_via_protection_attrs` below
    is the live-object half and answers {} on a wrapper that cannot be asked --
    which, on the shipping KiCad 10, is every via.

    The choice is made on whether the ACCESSORS work, not on whether they
    returned something: a via that genuinely inherits the board's `(setup ...)`
    answers {} legitimately, and on a working wrapper the live board is the
    authority for that answer.
    """
    if pcbnew_protection_accessors_usable(via):
        return _pcbnew_via_protection_attrs(via)
    if not text_specs:
        return {}
    try:
        uid = str(via.m_Uuid.AsString())
    except Exception:
        return {}
    return dict(text_specs.get(uid, {}))


def _pcbnew_via_protection_attrs(via) -> Dict[str, str]:
    """A pcbnew PCB_VIA's protection spec in the same {token: raw inner text}
    shape the text parser produces (#489 §8 parity).

    Only modes the via OVERRIDES are recorded: *_MODE_FROM_BOARD means "inherit
    the board setting", which is the absence of a token in the file. Returns {}
    on a KiCad without these accessors, so the builder degrades to today's
    behavior rather than raising.
    """
    import pcbnew

    def _side(getter, tented_const, not_tented_const):
        try:
            mode = getter()
        except Exception:
            return None
        if mode == tented_const:
            return 'yes'
        if mode == not_tented_const:
            return 'no'
        return None  # FROM_BOARD -> inherit, emit nothing

    out: Dict[str, str] = {}
    try:
        pairs = (
            ('tenting', via.GetFrontTentingMode, via.GetBackTentingMode,
             pcbnew.TENTING_MODE_TENTED, pcbnew.TENTING_MODE_NOT_TENTED),
            ('covering', via.GetFrontCoveringMode, via.GetBackCoveringMode,
             pcbnew.COVERING_MODE_COVERED, pcbnew.COVERING_MODE_NOT_COVERED),
        )
    except AttributeError:
        return {}

    for token, front_get, back_get, yes_const, no_const in pairs:
        front = _side(front_get, yes_const, no_const)
        back = _side(back_get, yes_const, no_const)
        parts = []
        if front is not None:
            parts.append(f"(front {front})")
        if back is not None:
            parts.append(f"(back {back})")
        if parts:
            out[token] = " ".join(parts)

    # plugging is per-side too; capping and filling are whole-via.
    try:
        front = _side(via.GetFrontPluggingMode, pcbnew.PLUGGING_MODE_PLUGGED,
                      pcbnew.PLUGGING_MODE_NOT_PLUGGED)
        back = _side(via.GetBackPluggingMode, pcbnew.PLUGGING_MODE_PLUGGED,
                     pcbnew.PLUGGING_MODE_NOT_PLUGGED)
        parts = []
        if front is not None:
            parts.append(f"(front {front})")
        if back is not None:
            parts.append(f"(back {back})")
        if parts:
            out['plugging'] = " ".join(parts)
    except AttributeError:
        pass

    for token, getter, yes_name, no_name in (
            ('capping', getattr(via, 'GetCappingMode', None),
             'CAPPING_MODE_CAPPED', 'CAPPING_MODE_NOT_CAPPED'),
            ('filling', getattr(via, 'GetFillingMode', None),
             'FILLING_MODE_FILLED', 'FILLING_MODE_NOT_FILLED')):
        yes_const = getattr(pcbnew, yes_name, None)
        no_const = getattr(pcbnew, no_name, None)
        if getter is None or yes_const is None or no_const is None:
            continue
        value = _side(getter, yes_const, no_const)
        if value is not None:
            out[token] = value

    return out


def _balanced_token_text(text: str, token: str) -> Optional[str]:
    """Inner text of `(token ...)` in `text`, or None. Paren-balanced, so a
    nested `(front yes) (back yes)` comes back whole."""
    m = re.search(r'\(' + re.escape(token) + r'(?=[\s)])', text)
    if not m:
        return None
    start = m.start()
    depth = 0
    for i, c in enumerate(text[start:]):
        if c == '(':
            depth += 1
        elif c == ')':
            depth -= 1
            if depth == 0:
                inner = text[start + len(token) + 1:start + i]
                # Collapse the file's line breaks/tabs: the pcbnew builder
                # produces "(front no) (back no)" for the same via, and the two
                # parse paths must be comparable field-for-field (CLAUDE.md
                # parser-parity rule), not just semantically equal.
                return " ".join(inner.split())
    return None


def _via_spec_from_block(block: str) -> Dict[str, str]:
    """The tenting-family spec carried by ONE via block, as {token: inner text}.

    Read from the block by token rather than by extending the via regexes: the
    tokens are optional, differ by KiCad version, and one of them appearing
    where a positional pattern did not expect it is exactly how #748 lost a via.
    """
    spec: Dict[str, str] = {}
    for token in VIA_PROTECTION_TOKENS:
        inner = _balanced_token_text(block, token)
        if inner is not None:
            spec[token] = inner
    return spec


#: #962: KiCad's factory via-protection policy, i.e. what `BOARD_DESIGN_SETTINGS`
#: holds for a token the board's `(setup ...)` does not write (probed on pcbnew
#: 10.0.0: sonde_u's legacy setup names only tenting, and the other four read
#: back as covering/plugging off, capping/filling no).
VIA_PROTECTION_SETUP_DEFAULTS = {
    'tenting': '(front yes) (back yes)',
    'covering': '(front no) (back no)',
    'plugging': '(front no) (back no)',
    'capping': 'no',
    'filling': 'no',
}


def canonical_via_protection_setup(raw: Dict[str, str]) -> Dict[str, str]:
    """The board's via-protection policy in ONE spelling (#962).

    - All five tokens are present; absent ones take KiCad's factory default.
    - The legacy KiCad 9 per-side form `(tenting front back)` becomes
      `(front yes) (back yes)`. `front` alone becomes `(front yes) (back no)`,
      and `none` or empty becomes both no.

    One spelling means the text parser and the pcbnew path, which reads
    booleans off the design settings, compare field for field.
    """
    out = dict(VIA_PROTECTION_SETUP_DEFAULTS)
    for tok, inner in (raw or {}).items():
        if tok not in out:
            continue
        inner = ' '.join((inner or '').split())
        if tok in ('tenting', 'covering', 'plugging'):
            if '(' not in inner:
                words = inner.split()
                front = 'front' in words
                back = 'back' in words
            else:
                # Nested form. A side it does not name reads as `no`, which is
                # what pcbnew makes of `(tenting (front no))` (probed, 10.0.0).
                fm = re.search(r'\(front\s+(\w+)\)', inner)
                bm = re.search(r'\(back\s+(\w+)\)', inner)
                front = bool(fm) and fm.group(1) in ('yes', 'true')
                back = bool(bm) and bm.group(1) in ('yes', 'true')
            inner = '(front %s) (back %s)' % ('yes' if front else 'no',
                                             'yes' if back else 'no')
        out[tok] = inner
    return out


def via_protection_setup_from_design_settings(ds) -> Dict[str, str]:
    """`canonical_via_protection_setup` read off a live pcbnew
    BOARD_DESIGN_SETTINGS (#962, GUI parse path). Every field is a plain bool
    on 10.0.0 (probed), so no SWIG enum is involved."""
    def yn(field, factory):
        v = getattr(ds, field, None)
        if v is None:           # an older pcbnew without the field: factory value
            return factory
        return 'yes' if bool(v) else 'no'
    return {
        'tenting': '(front %s) (back %s)' % (yn('m_TentViasFront', 'yes'),
                                             yn('m_TentViasBack', 'yes')),
        'covering': '(front %s) (back %s)' % (yn('m_CoverViasFront', 'no'),
                                              yn('m_CoverViasBack', 'no')),
        'plugging': '(front %s) (back %s)' % (yn('m_PlugViasFront', 'no'),
                                              yn('m_PlugViasBack', 'no')),
        'capping': yn('m_CapVias', 'no'),
        'filling': yn('m_FillVias', 'no'),
    }


def extract_board_setup_paste_and_protection(content: str):
    """`(pad_to_paste_clearance, pad_to_paste_clearance_ratio, via_protection)`
    from the board's own `(setup ...)` (#962).

    The first two are the last rung of KiCad's paste-margin precedence (0 when
    unset). The third is the via-protection policy a via with no spec of its
    own inherits, canonicalised (see `canonical_via_protection_setup`).
    """
    setup = _balanced_token_text(content, 'setup') or ''

    def num(tok):
        v = _balanced_token_text(setup, tok)
        try:
            return float(v) if v is not None else 0.0
        except ValueError:
            return 0.0

    raw = {}
    for token in VIA_PROTECTION_TOKENS:
        inner = _balanced_token_text(setup, token)
        if inner is not None:
            raw[token] = inner
    # KiCad 6-8 expressed via tenting as the plot option `(viasonmask
    # yes|true)` = UNtented / `no|false` = tented. pcbnew 10 still honours it,
    # and the LATER of the two tokens in the setup wins: a `(tenting ...)`
    # written before `(pcbplotparams (viasonmask yes))` loads UNtented, and
    # `(tenting none)` before `(viasonmask no)` loads tented (probed on v8,
    # v9 and v10 files; #962 phase-1 verification, rounds 1 and 2).
    vm = re.search(r'\(viasonmask\s+(yes|true|no|false)\)', setup)
    tm = re.search(r'\(tenting(?=[\s)])', setup)
    if vm and (tm is None or vm.start() > tm.start()):
        raw['tenting'] = ('(front no) (back no)' if vm.group(1) in ('yes', 'true')
                          else '(front yes) (back yes)')
    return (num('pad_to_paste_clearance'), num('pad_to_paste_clearance_ratio'),
            canonical_via_protection_setup(raw))


#: #962: the last file version whose loader reads an explicit
#: `(solder_paste_margin 0)` / `(... _ratio 0)` as UNSET. From 20240202 on,
#: KiCad reads it as an explicit 0 that overrides the footprint and the board.
#: Probed on pcbnew 10.0.0 across ten file versions, at pad and footprint level
#: (#962 phase-1 verification, round 2).
PASTE_ZERO_IS_UNSET_MAX_VERSION = 20240201


def _paste_overrides(text: str, version: int = 0):
    """`(solder_paste_margin, ratio)` written directly in `text` (a pad block, or
    a footprint HEADER the caller has already bounded); None where unset.

    KiCad has spelled the ratio `solder_paste_margin_ratio` (pads, and older
    footprints) and `solder_paste_ratio` (footprints), so both are read.

    An explicit 0 means what the loader of THAT file version makes of it:
    unset up to PASTE_ZERO_IS_UNSET_MAX_VERSION, an override after it.
    `version` 0 (unknown) is read the modern way.
    """
    m = re.search(r'\(solder_paste_margin\s+(-?[\d.]+)\)', text)
    r = (re.search(r'\(solder_paste_margin_ratio\s+(-?[\d.]+)\)', text)
         or re.search(r'\(solder_paste_ratio\s+(-?[\d.]+)\)', text))
    mv = float(m.group(1)) if m else None
    rv = float(r.group(1)) if r else None
    if version and version <= PASTE_ZERO_IS_UNSET_MAX_VERSION:
        mv = mv if mv else None
        rv = rv if rv else None
    return mv, rv


def _shape_layer_names(blk: str) -> List[str]:
    """Every layer a shape block names: the singular `(layer "X")` or the plural
    `(layers "X" "Y")`."""
    lm = re.search(r'\(layer\s+"([^"]+)"\)', blk)
    if lm:
        return [lm.group(1)]
    lsm = re.search(r'\(layers\s+((?:"[^"]*"\s*)+)\)', blk)
    return re.findall(r'"([^"]*)"', lsm.group(1)) if lsm else []


def _shape_filled(blk: str, kind: str, width: float) -> bool:
    """Is the shape filled, by KiCad's loader rules?

    - With a `(fill X)` token: filled unless X is `no` / `none`. Hatched counts
      as filled, as pcbnew's IsAnyFill() does.
    - With NO token: a poly is filled, and a rect or circle is filled only when
      its stroke width is 0. Lines and arcs never are.

    The no-token rule was probed on pcbnew 10.0.0 for file versions 20211014,
    20221018, 20240108, 20241229 and 20260206 (#962 phase-1 verification, B2).
    The corpus carries no token-less paste shape, so only that probe pins it.
    """
    fm = re.search(r'\(fill\s+(\w+)\)', blk)
    if fm:
        return fm.group(1) not in ('no', 'none')
    if kind == 'poly':
        return True
    if kind in ('rect', 'circle'):
        return width <= 0
    return False


#: The stroke pcbnew's reader gives an UNFILLED shape whose file stroke width
#: is 0 (KiCad's DEFAULT_LINE_WIDTH). Probed on pcbnew 10.0.0 at file versions
#: 20211014, 20221018 and 20241229, for lines, arcs, rects, circles and polys:
#: unfilled -> 0.1, filled -> 0 kept. piunora's KiCad 6 MP1 paste outlines
#: are `(width 0) (fill none)` polys, which loaded as 0.1 mm stroked openings.
UNSTROKED_SHAPE_WIDTH = 0.1


def _loaded_stroke_width(blk: str, kind: str, width: float) -> float:
    """A graphic's stroke width as pcbnew loads it (UNSTROKED_SHAPE_WIDTH)."""
    if width <= 0 and not _shape_filled(blk, kind, width):
        return UNSTROKED_SHAPE_WIDTH
    return width


def _paste_shape_record(tag: str, blk: str, owner: str, transform):
    """One paste graphic as the `paste_apertures.graphic_aperture` dict, or
    None. `transform` maps a block-local point to global mm (the identity for
    board-level `gr_*`)."""
    layers = [ln for ln in _shape_layer_names(blk) if ln in ('F.Paste', 'B.Paste')]
    if not layers:
        return []
    wm = re.search(r'\(width\s+([-\d.]+)\)', blk)
    width = float(wm.group(1)) if wm else 0.0
    uuid = uuid_or_tstamp(blk)

    def xy(name):
        m = re.search(r'\(' + name + r'\s+([-\d.]+)\s+([-\d.]+)\)', blk)
        return (float(m.group(1)), float(m.group(2))) if m else None

    kind = tag.split('_', 1)[1]
    filled = _shape_filled(blk, kind, width)
    rec = {'owner_ref': owner, 'kind': kind,
           'width': _loaded_stroke_width(blk, kind, width),
           'filled': filled, 'uuid': uuid}
    if kind == 'poly':
        rec['points'] = poly_points_with_arcs(blk, transform)
    elif kind == 'rect':
        a, b = xy('start'), xy('end')
        if not (a and b):
            return []
        rec['points'] = [transform(a[0], a[1]), transform(b[0], a[1]),
                         transform(b[0], b[1]), transform(a[0], b[1])]
    elif kind == 'circle':
        c, e = xy('center'), xy('end')
        if not (c and e):
            return []
        rec['center'] = transform(*c)
        rec['radius'] = math.hypot(e[0] - c[0], e[1] - c[1])
    elif kind == 'line':
        a, b = xy('start'), xy('end')
        if not (a and b):
            return []
        rec['points'] = [transform(*a), transform(*b)]
    elif kind == 'arc':
        a, mid, b = xy('start'), xy('mid'), xy('end')
        if not (a and mid and b):
            return []
        pairs = _arc_to_segments(a, mid, b)
        pts = [pairs[0][0]] + [p1 for _p0, p1 in pairs] if pairs else []
        rec['points'] = [transform(*p) for p in pts]
    else:
        return []
    return [dict(rec, layer=ln) for ln in layers]


_UNMODELLED_LOGO_REASON = ('pad-less footprint: its copper is decoration the '
                           'writer relocates to silk (#146), so it is not modelled')
_UNMODELLED_CURVE_REASON = 'bezier copper (fp_curve) is not modelled'
_UNMODELLED_TEXT_REASON = 'visible text on a copper layer is not modelled'

# A singular copper `(layer ...)` token, `knockout` or not. Text always uses
# the singular form, so a footprint block with only ONE such token (its own
# header) carries no copper text. Shapes may use the PLURAL `(layers ...)`, so
# this is never the test for copper SHAPES.
_SINGULAR_CU_LAYER_RE = re.compile(r'\(layer\s+"?[^"\s)]*\.Cu\b')
_FP_TEXT_OPEN_RE = re.compile(r'\((?:fp_text|fp_text_box|property)\s')
_TEXT_LAYER_RE = re.compile(r'\(layer\s+"?([^"\s)]+)"?')
# `(hide yes)`, `(hide)`, or KiCad 6/7's bare `hide` token -- never `(hide no)`.
# Read with the quoted strings blanked, so a text SAYING "hide" is not hidden.
_TEXT_HIDDEN_RE = re.compile(r'\(hide\s+yes\)|\(hide\)|(?<![\w(])hide(?=[\s)])')
_QUOTED_RE = re.compile(r'"(?:[^"\\]|\\.)*"')


def _has_visible_copper_text(fp_text: str) -> bool:
    """Does this footprint block carry VISIBLE text on a copper layer?"""
    for m in _FP_TEXT_OPEN_RE.finditer(fp_text):
        j = find_matching_paren(fp_text, m.start())
        blk = fp_text[m.start():j]
        lm = _TEXT_LAYER_RE.search(blk)
        if (lm and lm.group(1).endswith('.Cu')
                and not _TEXT_HIDDEN_RE.search(_QUOTED_RE.sub('""', blk))):
            return True
    return False


def extract_unmodelled_footprint_copper(content: str) -> List[dict]:
    """Footprint copper the parser skips, by owner (#962). See
    `PCBData.graphic_copper_unmeasured`.

    Three kinds: a pad-less footprint's copper (`logo`), bezier copper
    (`curve`) and visible copper text (`text`). An ordinary part -- pads, no
    `fp_curve`, no second singular copper layer token (text is always
    singular) -- is skipped before any shape is walked, which keeps this pass
    cheap. A pad-less footprint is always walked: its shapes may name their
    layers in the plural form.
    """
    if '(fp_' not in content and '(property' not in content:
        return []
    masked = _mask_pad_primitives(content)
    out: List[dict] = []
    for _fstart, _fend, _fkey in _footprint_blocks_by_key(content):
        fp_text = masked[_fstart:_fend]
        functional = footprint_copper_is_functional(footprint_pad_count(fp_text))
        may_have_text = len(_SINGULAR_CU_LAYER_RE.findall(fp_text)) >= 2
        if functional and 'fp_curve' not in fp_text and not may_have_text:
            continue
        copper_tags = []
        if not functional or 'fp_curve' in fp_text:
            for tag, blk in iter_footprint_shapes(fp_text, _FP_SHAPE_TAGS + ('fp_curve',)):
                if any(ln.endswith('.Cu') or ln == 'F&B.Cu'
                       for ln in _shape_layer_names(blk)):
                    copper_tags.append(tag)
        if not functional and copper_tags:
            out.append({'owner_ref': _fkey, 'kind': 'logo',
                        'reason': _UNMODELLED_LOGO_REASON})
            continue
        if 'fp_curve' in copper_tags:
            out.append({'owner_ref': _fkey, 'kind': 'curve',
                        'reason': _UNMODELLED_CURVE_REASON})
        if may_have_text and _has_visible_copper_text(fp_text):
            out.append({'owner_ref': _fkey, 'kind': 'text',
                        'reason': _UNMODELLED_TEXT_REASON})
    return out


def extract_paste_graphics(content: str) -> List[dict]:
    """Every F.Paste/B.Paste GRAPHIC on the board, footprint-owned and
    board-level, as `paste_apertures.graphic_aperture` dicts (#962).

    Uses the same block walkers and pose transform as the #908 copper pass
    (`iter_footprint_shapes`, `footprint_pose`, `local_to_global`), over the
    pad-primitive-MASKED text, so a custom pad's `gr_*` primitive cannot leak
    in as a board-level shape.
    """
    if 'Paste' not in content:
        return []
    masked = _mask_pad_primitives(content)
    out: List[dict] = []
    fp_spans = []
    for _fstart, _fend, _fkey in _footprint_blocks_by_key(content):
        fp_spans.append((_fstart, _fend))
        fp_text = masked[_fstart:_fend]
        if 'Paste' not in fp_text or not _FP_SHAPE_RE.search(fp_text):
            continue
        pose = footprint_pose(fp_text)
        if pose is None:
            continue
        fx, fy, frot = pose

        def tf(x, y, _ox=fx, _oy=fy, _or=frot):
            return local_to_global(_ox, _oy, _or, x, y)
        for tag, blk in _footprint_shapes(masked, _fstart, fp_text):
            if 'Paste' in blk:      # most shapes are silk/fab: skip the regexes
                out.extend(_paste_shape_record(tag, blk, _fkey, tf))
    # Board-level gr_* on a paste layer (rare; owner '').
    for tag in ('gr_line', 'gr_arc', 'gr_poly', 'gr_rect', 'gr_circle'):
        needle = '(' + tag
        pos = 0
        while True:
            i = masked.find(needle, pos)
            if i < 0:
                break
            nxt = masked[i + len(needle): i + len(needle) + 1]
            if nxt and (nxt.isalnum() or nxt == '_'):
                pos = i + len(needle)
                continue
            j = find_matching_paren(masked, i)
            pos = j
            if any(s <= i < e for s, e in fp_spans):
                continue
            blk = masked[i:j]
            if 'Paste' in blk:
                out.extend(_paste_shape_record(
                    'fp_' + tag[3:], blk, '', lambda x, y: (x, y)))
    return out


def _extract_via_protection_attrs(content: str) -> Dict[str, Dict[str, str]]:
    """{via uuid: {token: raw inner text}} for the tenting-family tokens.

    Skipped entirely (single linear scan) when the file mentions none of them,
    so the common board pays nothing.

    ``extract_vias`` no longer needs this -- it reads each via's spec from the
    block it already has, so a UUID-LESS via can carry one now. This stays for
    the callers that hold board TEXT and no parsed vias (the GUI's #751
    fallback), and those genuinely need the uuid key.
    """
    if not any(('(' + t) in content for t in VIA_PROTECTION_TOKENS):
        return {}

    out: Dict[str, Dict[str, str]] = {}
    for _start, block in _via_blocks(content):
        via_uuid = uuid_or_tstamp(block)  # KiCad 6 vias carry (tstamp ...)
        if not via_uuid:
            continue
        spec = _via_spec_from_block(block)
        if spec:
            out[via_uuid] = spec
    return out


def warn_net_tagged_graphics(segments, name_to_id=None) -> int:
    """Say when a board carries NET-TAGGED copper graphics.

    KiCad allows a ``gr_line``/``gr_rect``/... on a copper layer to carry a
    ``(net ...)`` field (card-edge/pogo connector fingers drawn as graphics,
    a filled copper area). KiCad 10's connectivity conducts through them --
    measured with pcbnew and kicad-cli 10.0.0: a stroke joins the same-net
    copper it touches, a filled shape its whole interior -- and so does
    `check_connected.check_net_connectivity` (`conducting_stroke`). #513
    item 6 had concluded the opposite and graded such joins open: an
    inductor pad inside its net's own filled B.Cu gr_poly read
    open to the grader and to the route step while KiCad called it
    connected. They parse as ``Segment(graphic=True)``, immovable art (#337)
    the router routes around and never targets. Returns the number of
    net-tagged graphic segments found. Called by BOTH parse paths (file text
    and pcbnew build)."""
    tagged = [s for s in segments
              if getattr(s, 'graphic', False) and getattr(s, 'net_id', 0)]
    if not tagged:
        return 0
    id_to_name = {v: k for k, v in (name_to_id or {}).items()}
    nets = sorted({id_to_name.get(s.net_id, f'net {s.net_id}') for s in tagged})
    preview = ', '.join(nets[:6]) + (', ...' if len(nets) > 6 else '')
    print(f"WARNING: {len(tagged)} NET-TAGGED copper graphic segment(s) on "
          f"{len(nets)} net(s) ({preview}). KiCad conducts through them (a "
          f"stroke joins the copper it touches, a filled shape its whole "
          f"interior) and so does our connectivity grading, but they are "
          f"immovable art the router routes around and never targets (#337).")
    # ...and say what it will look like AFTERWARDS, which is the part people
    # actually hit (#659): art that touches none of its net's other copper is
    # an island KiCad's DRC demands a link to, and #337 makes it immutable, so
    # no routing pass welds to it or deletes it. Measured over 31 recorded
    # boards it was 36 of 51 KiCad-only open links on zone-less signal nets.
    # Nobody should spend a retry on it.
    print(f"         A net whose art touches none of its other copper stays "
          f"UNCONNECTED in KiCad's DRC on every run: no routing pass welds "
          f"to art, so joining it is a board-authoring fix, not a routing "
          f"failure (#659).")
    return len(tagged)


# A track block's FAST PATH: the field order KiCad (9 and 10) and this repo's
# writer emit. (locked yes) is optional between width/layer or layer/net
# (issue #150); uuid is optional (PR #534: KiCad accepts uuid-less copper).
# A block these do not match is NOT dropped any more (#1158): it is read by its
# own tokens in `_track_from_tokens`, and reported if even that fails.
_SEG_CANON_NUM_RE = re.compile(
    r'\(segment\s+\(start\s+([\d.-]+)\s+([\d.-]+)\)\s+\(end\s+([\d.-]+)\s+([\d.-]+)\)\s+\(width\s+([\d.-]+)\)\s+(?:\(locked\s+yes\)\s+)?\(layer\s+"([^"]+)"\)\s+(?:\(locked\s+yes\)\s+)?\(net\s+(\d+)\)(?:\s+\(uuid\s+"([^"]+)"\))?')
_SEG_CANON_NAMED_RE = re.compile(
    r'\(segment\s+\(start\s+([\d.-]+)\s+([\d.-]+)\)\s+\(end\s+([\d.-]+)\s+([\d.-]+)\)\s+\(width\s+([\d.-]+)\)\s+(?:\(locked\s+yes\)\s+)?\(layer\s+"([^"]+)"\)\s+(?:\(locked\s+yes\)\s+)?\(net\s+"' + _ESC_STR + r'"\)(?:\s+\(uuid\s+"([^"]+)"\))?')
_ARC_CANON_FIELDS = (r'\(arc\s+\(start\s+([\d.-]+)\s+([\d.-]+)\)\s+'
                     r'\(mid\s+([\d.-]+)\s+([\d.-]+)\)\s+'
                     r'\(end\s+([\d.-]+)\s+([\d.-]+)\)\s+'
                     r'\(width\s+([\d.-]+)\)\s+(?:\(locked\s+yes\)\s+)?'
                     r'\(layer\s+"([^"]+)"\)\s+(?:\(locked\s+yes\)\s+)?\(net\s+')
_ARC_CANON_NUM_RE = re.compile(_ARC_CANON_FIELDS + r'(\d+)\)(?:\s+\(uuid\s+"([^"]+)"\))?')
_ARC_CANON_NAMED_RE = re.compile(_ARC_CANON_FIELDS + r'"' + _ESC_STR + r'"\)(?:\s+\(uuid\s+"([^"]+)"\))?')

# Where a track block opens. `(arc` also opens the arc vertex of a polygon's
# (pts ...); `_track_from_tokens` tells the two apart (a vertex carries no
# width, layer or net).
_TRACK_OPEN_RE = re.compile(r'\((segment|arc)(?=[\s()])')
# The rest of a block whose children nest one level deep (every track block
# KiCad writes), string-aware. Unrolled -- normal* (special normal*)* with
# disjoint first characters -- so a block that does not close fails in linear
# time instead of backtracking. Anything deeper falls back to
# find_matching_paren.
_STR_RE_SRC = r'"(?:[^"\\]|\\.)*"'
_SHALLOW_TAIL_RE = re.compile(
    r'[^()"]*(?:(?:' + _STR_RE_SRC + r'|\([^()"]*(?:' + _STR_RE_SRC
    + r'[^()"]*)*\))[^()"]*)*\)')
_SEXPR_TOKEN_RE = re.compile(_STR_RE_SRC + r'|[()]|[^\s()"]+')
_LOCKED_CHILD_RE = re.compile(r'\(locked(?:\s+(yes|no|true|false))?\s*\)')
_NUM_TOKEN_RE = re.compile(r'^-?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?$')


def _sexpr_children(block: str):
    """``({token: [[arg, ...], ...]}, [bare word, ...])`` for one block's
    depth-1 children, string-aware. Only the per-token fallback path pays
    for this; quoted args keep their quotes."""
    children: Dict[str, List[List[str]]] = {}
    bare: List[str] = []
    depth = 0
    cur: Optional[List[str]] = None
    for t in _SEXPR_TOKEN_RE.findall(block):
        if t == '(':
            depth += 1
            if depth == 2:
                cur = []
        elif t == ')':
            if depth == 2 and cur:
                children.setdefault(cur[0], []).append(cur[1:])
                cur = None
            depth -= 1
        elif depth == 2 and cur is not None:
            cur.append(t)
        elif depth == 1:
            bare.append(t)
    # bare[0] is the block's own keyword (segment / arc / via).
    return children, bare[1:]


def _track_locked(block: str) -> bool:
    """KiCad's lock on a track or via block, wherever the token sits (#1158).

    KiCad's reader accepts ``(locked yes)``, ``(locked)`` (parseMaybeAbsentBool)
    and the older bare ``locked`` word anywhere in the block. The fast-path
    patterns used to read the flag from their own match, which ends at
    ``(net ...)``, so a lock written after the net was lost.
    """
    if 'locked' not in block:
        return False
    m = _LOCKED_CHILD_RE.search(block)
    if m:
        return m.group(1) in (None, 'yes', 'true')
    return 'locked' in _sexpr_children(block)[1]


def _child_floats(children, tok, n):
    """The first `tok` child's first `n` numeric args, or None."""
    for args in children.get(tok, ()):
        if len(args) >= n and all(_NUM_TOKEN_RE.match(a) for a in args[:n]):
            return [float(a) for a in args[:n]]
    return None


def _child_net(children, name_to_id):
    """``(net_id, named)`` from a block's (net ...) child; net 0 when the block
    carries none, which is what KiCad assigns. ``None`` for a named net with no
    name map to resolve it (the caller skips the block, as it always has)."""
    for args in children.get('net', ()):
        if not args:
            continue
        a = args[0]
        if a.startswith('"'):
            if not name_to_id:
                return None
            return name_to_id.get(a[1:-1], 0), True
        if _NUM_TOKEN_RE.match(a):
            return int(float(a)), False
    return 0, bool(name_to_id)


def _child_uuid(children) -> str:
    for args in children.get('uuid', ()):
        if args and args[0].startswith('"'):
            return args[0][1:-1]
    for args in children.get('tstamp', ()):  # KiCad 6's spelling
        if args:
            return kiid_from_tstamp(args[0])
    return ""


def _block_uuid(block: str) -> str:
    """A block's (uuid ...) wherever it sits; the fast path captures it only
    directly after (net ...)."""
    if '(uuid' not in block and '(tstamp' not in block:
        return ""
    return _child_uuid(_sexpr_children(block)[0])


def _track_blocks(content: str):
    """``(kind, start offset, block text)`` for every (segment ...) and (arc ...)
    in file order, plus the offsets of blocks that never close."""
    out = []
    unbalanced = []
    for m in _TRACK_OPEN_RE.finditer(content):
        t = _SHALLOW_TAIL_RE.match(content, m.end())
        if t is not None:
            out.append((m.group(1), m.start(), content[m.start():t.end()]))
            continue
        j = find_matching_paren(content, m.start())
        if j >= len(content) and not content.endswith(')'):
            unbalanced.append(m.start())
            continue
        out.append((m.group(1), m.start(), content[m.start():j]))
    return out, unbalanced


def _report_unparsed_copper(content: str, kind: str, rows) -> None:
    """One WARNING naming copper blocks the parser could not model (#1158).

    A block KiCad loads and this parser drops is copper the router plans
    through and every grader misses, so it is never silent. ``rows`` are
    ``(offset, reason)``.
    """
    if not rows:
        return
    shown = ', '.join(f"line {content.count(chr(10), 0, off) + 1} ({why})"
                      for off, why in rows[:5])
    more = f", and {len(rows) - 5} more" if len(rows) > 5 else ""
    print(f"WARNING: {len(rows)} {kind} block(s) could not be read and are NOT "
          f"modelled -- not an obstacle, not graded: {shown}{more}. KiCad may "
          f"still load them as copper; fix the block in the file (#1158).",
          file=sys.stderr)


def _track_from_tokens(kind, block, name_to_id):
    """Read one track block by its own tokens, in any order (#1158).

    Returns ``(fields, None)``, ``(None, reason)`` for a block that is a track
    but cannot be modelled, or ``(None, None)`` for a block that is not track
    copper at all (a polygon's arc vertex; a named net with no name map).
    """
    ch, _bare = _sexpr_children(block)
    if kind == 'arc' and not any(k in ch for k in ('width', 'layer', 'net')):
        return None, None          # a (pts ...) arc vertex, not a track
    net = _child_net(ch, name_to_id)
    if net is None:
        return None, None
    start = _child_floats(ch, 'start', 2)
    end = _child_floats(ch, 'end', 2)
    mid = _child_floats(ch, 'mid', 2) if kind == 'arc' else None
    width = _child_floats(ch, 'width', 1)
    layer = next((a[0][1:-1] for a in ch.get('layer', ()) if a and a[0].startswith('"')), None)
    if layer is None:
        # KiCad 9's solder-mask-exposed track: (layers "F.Cu" "F.Mask"). Its
        # copper is the .Cu layer (endgame_trackball's antenna feed, six
        # segments the fixed-order patterns never saw).
        layer = next((x[1:-1] for a in ch.get('layers', ()) for x in a
                      if x.startswith('"') and x[1:-1].endswith('.Cu')), None)
    missing = [t for t, v in (('start', start), ('end', end), ('width', width),
                              ('layer', layer)) if v is None]
    if kind == 'arc' and mid is None:
        missing.insert(1, 'mid')
    if missing:
        return None, f"{kind} without {'/'.join(missing)}"
    return dict(start=start, mid=mid, end=end, width=width[0], layer=layer,
                net_id=net[0], named=net[1], uuid=_child_uuid(ch),
                locked=_track_locked(block)), None


def extract_segments(content: str, name_to_id: Dict[str, int] = None) -> List[Segment]:
    """Extract all track segments (and linearised track arcs) from a PCB file.

    BLOCK-BASED (#1158). Each ``(segment ...)`` / ``(arc ...)`` block is found
    first, then matched against the canonical field order (the fast path, which
    is every block KiCad writes); a block in any other order is read by its own
    tokens, and one that still cannot be modelled is REPORTED. The patterns used
    to run over the whole file and accept ``(locked yes)`` at only two
    positions, so a hand-stamped lock before ``(width)`` -- or the bare
    ``(segment locked ...`` form -- made the track vanish from the model,
    silently, while KiCad loaded it as copper (StickHub: 747 of 749 segments).

    Output order is unchanged for every block the old patterns read: numeric
    -net segments, then named-net segments, then numeric-net arcs, then
    named-net arcs, each in file order (#79: mixed-dialect files are legal).
    """
    seg_num: List[Segment] = []
    seg_named: List[Segment] = []
    arc_num: List[Segment] = []
    arc_named: List[Segment] = []
    unparsed = []

    def _seg(sx, sy, ex, ey, width, layer, net_id, uuid, locked, strs=None):
        if strs is None:
            strs = (repr(sx), repr(sy), repr(ex), repr(ey))
        return Segment(
            start_x=sx, start_y=sy, end_x=ex, end_y=ey, width=width,
            layer=layer, net_id=net_id, uuid=uuid,
            # Store original strings for exact file matching
            start_x_str=strs[0], start_y_str=strs[1],
            end_x_str=strs[2], end_y_str=strs[3], locked=locked)

    def _arc(out, sx, sy, mx, my, ex, ey, width, layer, net_id, uuid, locked):
        # Arc tracks: KiCad routes rounded corners as (arc (start)(mid)(end)...).
        # Linearize each into straight Segments so the copper graph is complete
        # -- dropped, they fragmented arc-routed (human) boards into false gaps.
        # (The router never emits arcs; this only matters for ingested boards.)
        for (p0, p1) in _arc_to_segments((sx, sy), (mx, my), (ex, ey)):
            out.append(_seg(p0[0], p0[1], p1[0], p1[1], width, layer, net_id,
                            uuid, locked))

    blocks, unbalanced = _track_blocks(content)
    for off in unbalanced:
        unparsed.append((off, 'block never closes'))
    for kind, off, block in blocks:
        if kind == 'segment':
            m = _SEG_CANON_NUM_RE.match(block)
            named = False
            if m is None and name_to_id:
                m = _SEG_CANON_NAMED_RE.match(block)
                named = True
            if m is not None:
                net_id = (name_to_id.get(m.group(7), 0) if named
                          else int(m.group(7)))
                (seg_named if named else seg_num).append(_seg(
                    float(m.group(1)), float(m.group(2)), float(m.group(3)),
                    float(m.group(4)), float(m.group(5)), m.group(6), net_id,
                    m.group(8) or _block_uuid(block),
                    _track_locked(block),
                    strs=(m.group(1), m.group(2), m.group(3), m.group(4))))
                continue
        else:
            m = _ARC_CANON_NUM_RE.match(block)
            named = False
            if m is None and name_to_id:
                m = _ARC_CANON_NAMED_RE.match(block)
                named = True
            if m is not None:
                net_id = (name_to_id.get(m.group(9), 0) if named
                          else int(m.group(9)))
                _arc(arc_named if named else arc_num,
                     float(m.group(1)), float(m.group(2)), float(m.group(3)),
                     float(m.group(4)), float(m.group(5)), float(m.group(6)),
                     float(m.group(7)), m.group(8), net_id,
                     m.group(10) or _block_uuid(block),
                     _track_locked(block))
                continue
        f, why = _track_from_tokens(kind, block, name_to_id)
        if f is None:
            if why:
                unparsed.append((off, why))
            continue
        if kind == 'segment':
            (seg_named if f['named'] else seg_num).append(_seg(
                f['start'][0], f['start'][1], f['end'][0], f['end'][1],
                f['width'], f['layer'], f['net_id'], f['uuid'], f['locked']))
        else:
            _arc(arc_named if f['named'] else arc_num,
                 f['start'][0], f['start'][1], f['mid'][0], f['mid'][1],
                 f['end'][0], f['end'][1], f['width'], f['layer'],
                 f['net_id'], f['uuid'], f['locked'])
    _report_unparsed_copper(content, 'track', unparsed)
    segments = seg_num + seg_named + arc_num + arc_named

    # Net-tied copper GRAPHICS (#337): KiCad renders gr_line / gr_arc drawn on
    # a copper layer as real copper (optionally carrying a (net ...)). They are
    # invisible as (segment) records, so without this pass both check_drc and
    # the router's obstacle model were blind to them -- openstint's router
    # placed a via 62um from a hand-drawn GND gr_line it could not see, and
    # kicad-cli flagged dozens of such contacts we graded clean. Parse them
    # into Segments tagged graphic=True (obstacle/DRC-real, never prunable).
    def _resolve_net(tok):
        if tok is None:
            return 0
        tok = tok.strip()
        if tok.startswith('"'):
            return name_to_id.get(tok[1:-1], 0) if name_to_id else 0
        try:
            return int(tok)
        except ValueError:
            return 0

    # #337: model copper GRAPHICS (gr_line/gr_arc/gr_poly/gr_rect/gr_circle on a
    # copper layer) as graphic=True copper. BLOCK-BASED extraction -- find each
    # (gr_* ...) block, then pull each field from within it regardless of ORDER
    # or extra tokens. A fixed field-order regex silently dropped a graphic with
    # a (locked yes) token, KiCad-6/7 (tstamp ..) instead of (uuid ..), net-
    # before-layer ordering, or extra stroke fields -- a silent miss in a
    # "never miss real copper" pass. Lines/arcs need a real stroke (width>0);
    # filled poly/rect/circle default the outline to the fab track width.
    def _emit_outline(pts, w, layer, nid, uuid, closed=True, kind='poly',
                      circle=None, filled=False):
        if len(pts) < 2:
            return
        ew = w if w > 0 else defaults.TRACK_WIDTH
        seq = pts + [pts[0]] if closed else pts
        ring = tuple((float(x), float(y)) for x, y in pts) if (filled and closed) else None
        for a, b in zip(seq, seq[1:]):
            if a == b:
                # A poly whose vertex list already REPEATS its first point --
                # KiCad writes many that way (every one of watchy's twelve
                # antenna polys) -- would close onto itself and yield a
                # zero-length "segment". That models no copper and each one
                # becomes a duplicate DRC row a human has to dismiss: watchy
                # graded 12 board-edge violations where the truth is 9.
                continue
            segments.append(Segment(
                start_x=a[0], start_y=a[1], end_x=b[0], end_y=b[1],
                width=ew, layer=layer, net_id=nid, uuid=uuid, graphic=True,
                drawn_width=max(0.0, w), graphic_kind=kind,
                graphic_circle=circle, graphic_filled=bool(filled and closed),
                graphic_ring=ring))

    def _blk_fields(blk):
        # BOTH layer tokens (#659 follow-up). KiCad writes the singular
        # (layer "F.Cu") for a one-layer graphic and the PLURAL
        # (layers "F.Cu" "F.Mask") for one that also carries mask/paste --
        # and a singular-only regex dropped the plural form ENTIRELY, so its
        # copper was neither connectivity nor an OBSTACLE. Measured on
        # endgame_trackball: 8 filled 3.2mm copper gr_circles invisible, and
        # the router laid 13 foreign-net crossings straight through them (the
        # unrouted input had 0) while every checker read clean, because
        # check_drc reads this same parser. Returns the copper layers as a
        # LIST -- a two-sided graphic is copper on both.
        lm = re.search(r'\(layer\s+"([^"]+)"\)', blk)
        if lm:
            names = [lm.group(1)]
        else:
            lsm = re.search(r'\(layers\s+((?:"[^"]*"\s*)+)\)', blk)
            names = re.findall(r'"([^"]*)"', lsm.group(1)) if lsm else []
        layers = []
        for _nm in names:
            # F&B.Cu is KiCad's two-sided shorthand. A bare *.Cu wildcard
            # cannot be expanded here (extract_segments has no stackup), so
            # it falls through as non-copper rather than being guessed at.
            for _one in (('F.Cu', 'B.Cu') if _nm == 'F&B.Cu' else (_nm,)):
                if _one.endswith('.Cu') and _one not in layers:
                    layers.append(_one)
        wm = re.search(r'\(width\s+([-\d.]+)\)', blk)
        nm = re.search(r'\(net\s+("[^"]*"|\d+)\)', blk)
        return (layers, float(wm.group(1)) if wm else 0.0,
                _resolve_net(nm.group(1)) if nm else 0,
                uuid_or_tstamp(blk))

    def _xy(blk, name):
        m = re.search(r'\(' + name + r'\s+([-\d.]+)\s+([-\d.]+)\)', blk)
        return (float(m.group(1)), float(m.group(2))) if m else None

    # A custom pad draws its copper with gr_* PRIMITIVES, in PAD-LOCAL
    # coordinates. Real KiCad writes those without a `(layer ...)`, so
    # `_blk_fields` drops them -- but that immunity is INCIDENTAL, and a
    # primitive that does carry a copper layer lands as phantom copper
    # hundreds of millimetres from the pad. Mask them, which is the structural
    # answer the other board-level gr_* scans already use. Length-preserving,
    # so the footprint spans below still line up.
    _gcontent = _mask_pad_primitives(content)
    for tag in ('gr_line', 'gr_arc', 'gr_poly', 'gr_rect', 'gr_circle'):
        needle = '(' + tag
        pos = 0
        while True:
            i = _gcontent.find(needle, pos)
            if i < 0:
                break
            nxt = _gcontent[i + len(needle): i + len(needle) + 1]
            if nxt and (nxt.isalnum() or nxt == '_'):
                pos = i + len(needle)
                continue
            j = find_matching_paren(_gcontent, i)
            blk = _gcontent[i:j]
            pos = j
            layers, w, nid, uuid = _blk_fields(blk)
            if not layers:
                continue
            # pcbnew strokes an UNFILLED zero-width shape at 0.1 mm on load.
            w = _loaded_stroke_width(blk, tag[3:], w)
            if tag in ('gr_line', 'gr_arc') and w <= 0:
                continue        # unstroked line/arc: no copper to model
            for layer in layers:
                if tag == 'gr_line':
                    a, b = _xy(blk, 'start'), _xy(blk, 'end')
                    if a and b:
                        segments.append(Segment(
                            start_x=a[0], start_y=a[1], end_x=b[0], end_y=b[1],
                            width=w, layer=layer, net_id=nid, uuid=uuid, graphic=True,
                            drawn_width=w, graphic_kind='line'))
                elif tag == 'gr_arc':
                    a, mid, b = _xy(blk, 'start'), _xy(blk, 'mid'), _xy(blk, 'end')
                    if a and mid and b:
                        for p0, p1 in _arc_to_segments(a, mid, b):
                            segments.append(Segment(
                                start_x=p0[0], start_y=p0[1], end_x=p1[0], end_y=p1[1],
                                width=w, layer=layer, net_id=nid, uuid=uuid, graphic=True,
                                drawn_width=w, graphic_kind='arc'))
                elif tag == 'gr_poly':
                    pts = poly_points_with_arcs(blk, lambda x, y: (x, y))
                    _emit_outline(pts, w, layer, nid, uuid,
                                  filled=_shape_filled(blk, 'poly', w))
                elif tag == 'gr_rect':
                    a, b = _xy(blk, 'start'), _xy(blk, 'end')
                    if a and b:
                        _emit_outline([(a[0], a[1]), (b[0], a[1]),
                                       (b[0], b[1]), (a[0], b[1])], w, layer, nid, uuid,
                                      kind='rect', filled=_shape_filled(blk, 'rect', w))
                elif tag == 'gr_circle':
                    c, e = _xy(blk, 'center'), _xy(blk, 'end')
                    if c and e:
                        r = math.hypot(e[0] - c[0], e[1] - c[1])
                        _emit_outline([(c[0] + r * math.cos(k * math.pi / 8),
                                        c[1] + r * math.sin(k * math.pi / 8))
                                       for k in range(16)], w, layer, nid, uuid,
                                      kind='circle', circle=(c[0], c[1], r),
                                      filled=_shape_filled(blk, 'circle', w))

    # #908: the same model for copper drawn INSIDE a footprint -- the drawn tab
    # of a SOT89/DPAK, a PCB antenna, a solder-jumper bridge. The scan above
    # walks `gr_*` only, so this copper was invisible to the obstacle builders
    # and to check_drc, which reads this same parser: kicad-cli 10.0.0 reports
    # a real short on esp_prog (U2 pad 2 against its own tab polygon) that
    # check_drc graded clean, and a foreign track laid across the tab would
    # have routed and graded clean here and shorted on the fab.
    #
    # Same emission as the board-level pass -- net-0 `graphic=True` outline
    # Segments, sharing `_blk_fields`/`_xy`/`_emit_outline` so the two cannot
    # describe the same primitive differently -- plus the ONE thing footprint
    # shapes need: the owning footprint's pose. Points go through
    # `local_to_global`, never a hand-rolled transform, because its
    # integer-nanometre snap is what keeps the text and pcbnew parse paths
    # bit-identical.
    #
    # B-side needs no mirror term: footprint child coordinates are stored
    # already side-resolved and the shape carries its own `(layer "B.Cu")`
    # (verified against pcbnew GraphicalItems on a flipped rot-180 part, see
    # the #304 note in `_collect_footprint_edge_segments_by_ref`).
    if '(fp_' in content:
        # Spans come from the UNMASKED text so the memo is shared with the
        # other three callers; the slice comes from the masked text, which is
        # the same length, so a layered pad primitive cannot leak in here
        # either.
        for _fstart, _fend, _fkey in _footprint_blocks_by_key(content):
            fp_text = _gcontent[_fstart:_fend]
            if not _FP_SHAPE_RE.search(fp_text):
                continue
            # A pad-LESS footprint is a logo the writer relocates to silk
            # (#146); modelling copper that is about to leave copper would
            # cost routing area for nothing. See footprint_copper_is_functional.
            if not footprint_copper_is_functional(footprint_pad_count(fp_text)):
                continue
            _pose = footprint_pose(fp_text)
            if _pose is None:
                # Copper dropped in silence is the failure mode this whole
                # pass exists to end, so say it rather than `continue`.
                print(f"  WARNING: footprint {_fkey} draws copper but has no "
                      f"readable (at x y) pose; its copper is NOT modelled "
                      f"(#908)")
                continue
            _fx, _fy, _frot = _pose

            def _g(_x, _y, _ox=_fx, _oy=_fy, _or=_frot):
                return local_to_global(_ox, _oy, _or, _x, _y)

            _mark_from = len(segments)
            for tag, blk in _footprint_shapes(_gcontent, _fstart, fp_text):
                layers, w, nid, uuid = _blk_fields(blk)
                if not layers:
                    continue
                w = _loaded_stroke_width(blk, tag[3:], w)   # see the board pass
                if tag in ('fp_line', 'fp_arc') and w <= 0:
                    continue    # unstroked line/arc: no copper to model
                for layer in layers:
                    if tag == 'fp_line':
                        a, b = _xy(blk, 'start'), _xy(blk, 'end')
                        if a and b:
                            ga, gb = _g(*a), _g(*b)
                            segments.append(Segment(
                                start_x=ga[0], start_y=ga[1],
                                end_x=gb[0], end_y=gb[1],
                                width=w, layer=layer, net_id=nid,
                                uuid=uuid, graphic=True,
                                drawn_width=w, graphic_kind='line'))
                    elif tag == 'fp_arc':
                        a, mid, b = (_xy(blk, 'start'), _xy(blk, 'mid'),
                                     _xy(blk, 'end'))
                        if a and mid and b:
                            for p0, p1 in _arc_to_segments(a, mid, b):
                                g0, g1 = _g(*p0), _g(*p1)
                                segments.append(Segment(
                                    start_x=g0[0], start_y=g0[1],
                                    end_x=g1[0], end_y=g1[1],
                                    width=w, layer=layer, net_id=nid,
                                    uuid=uuid, graphic=True,
                                    drawn_width=w, graphic_kind='arc'))
                    elif tag == 'fp_poly':
                        pts = poly_points_with_arcs(blk, _g)
                        _emit_outline(pts, w, layer, nid, uuid,
                                      filled=_shape_filled(blk, 'poly', w))
                    elif tag == 'fp_rect':
                        a, b = _xy(blk, 'start'), _xy(blk, 'end')
                        if a and b:
                            # all four corners transformed: under rotation the
                            # rect tilts, so start/end alone do not bound it
                            # pcbnew turns a rect in a footprint at a
                            # non-cardinal angle into a POLY on load, so name
                            # it the way the live path will read it.
                            _emit_outline([_g(a[0], a[1]), _g(b[0], a[1]),
                                           _g(b[0], b[1]), _g(a[0], b[1])],
                                          w, layer, nid, uuid,
                                          kind='rect' if _frot % 90 == 0 else 'poly',
                                          filled=_shape_filled(blk, 'rect', w))
                    elif tag == 'fp_circle':
                        c, e = _xy(blk, 'center'), _xy(blk, 'end')
                        if c and e:
                            r = math.hypot(e[0] - c[0], e[1] - c[1])
                            _gc = _g(*c)
                            _emit_outline(
                                [_g(c[0] + r * math.cos(k * math.pi / 8),
                                    c[1] + r * math.sin(k * math.pi / 8))
                                 for k in range(16)], w, layer, nid, uuid,
                                kind='circle', circle=(_gc[0], _gc[1], r),
                                filled=_shape_filled(blk, 'circle', w))
            # Tag afterwards rather than threading an owner through
            # `_emit_outline`, whose signature the board-level pass shares.
            for _s in segments[_mark_from:]:
                _s.owner_ref = _fkey

    warn_net_tagged_graphics(segments, name_to_id)
    return segments


def _iter_zone_blocks(content: str):
    """Yield `(body, in_footprint)` for each ``(zone ...)`` block
    (`_iter_zone_spans` without the offset)."""
    for _at, body, in_footprint in _iter_zone_spans(content):
        yield body, in_footprint


def _iter_zone_spans(content: str):
    """Yield `(offset, body, in_footprint)` for each ``(zone ...)`` block: the
    offset of its opening paren in `content`, which names the footprint block
    a footprint-owned zone sits in (`extract_keepouts`).

    Zones are at the top level, indented with a single tab. Uses ``\\r?\\n`` to
    handle both Unix and Windows line endings. Each yielded string is the
    content between the opening ``(zone`` line and its matching closing paren.
    Shared by :func:`extract_zones` and :func:`extract_keepouts`.
    """
    # #369 A5: match the zone header token itself, not "(zone" + NEWLINE --
    # KiCad v5/v6 write zones single-line ("(zone (net 0) (layer F.Cu) ...")
    # and the newline-anchored pattern parsed ZERO zones/keepouts from those
    # files (every pour and rule area silently dropped).
    # #478: accept ANY indentation, not exactly one tab -- KiCad 7+ footprints
    # can own zones (keep-outs AND copper pours), written at two tabs, and the
    # single-tab anchor dropped every one of them (router routed through
    # footprint keep-outs; fill model over-credited across them). Yields
    # (body, in_footprint): >= 2 tabs (or a deep space indent in legacy
    # space-indented files) marks a footprint-nested zone. Footprint-zone
    # polygon points are stored in BOARD coordinates, so no transform applies.
    zone_start_pattern = r'\r?\n([\t ]*)\(zone(?=[\s(])'
    for start_match in re.finditer(zone_start_pattern, content):
        indent = start_match.group(1)
        in_footprint = indent.count('\t') >= 2 or \
            ('\t' not in indent and len(indent) >= 4)
        # Find the matching closing paren (string-aware, so a lone paren inside
        # a quoted token cannot run the scan past the zone end — see issue #113).
        # start_match begins at the newline before "(zone"; locate that "(".
        open_idx = content.index('(', start_match.start())
        zone_end = find_matching_paren(content, open_idx) - 1
        body_start = open_idx + len('(zone')
        if zone_end <= body_start:
            continue
        yield open_idx, content[body_start:zone_end], in_footprint


# --- Zone outlines as pcbnew loads them ---------------------------------------
# A zone or rule-area outline is a list of `(xy X Y)` points and
# `(arc (start ..) (mid ..) (end ..))` vertices. pcbnew keeps the arc and hands
# every consumer (Outline().CPoint) its polyline, built by
# SHAPE_LINE_CHAIN::Append(SHAPE_ARC) -> SHAPE_ARC::ConvertToPolyline. The
# helpers below port that code from KiCad 10.0.0 (shape_arc.cpp,
# shape_line_chain.cpp, geometry_utils.cpp, trigo.cpp, eda_angle.h; unchanged
# in 10.0.6 except for a CalcArcCenter early-out that only fires on arcs whose
# three points lie within 5 nm, which are a single chord either way), so the
# text parser's vertices are pcbnew's, on the nm grid.

#: The max error SHAPE_LINE_CHAIN::Append(SHAPE_ARC) polygonises an outline arc
#: at: getArcPolygonizationMaxError() = ARC_HIGH_DEF / 5 = 1000 nm in KiCad 10
#: (KiCad 9 used ARC_HIGH_DEF itself, 5000 nm, so a KiCad 9 pcbnew holds
#: fewer vertices on the same arc).
ZONE_ARC_MAX_ERROR_NM = 1000

_KI_DEG_TO_RAD = math.pi / 180.0
_KI_SQRT1_2 = math.sqrt(0.5)          # M_SQRT1_2
_KI_SQRT2 = math.sqrt(2.0)            # M_SQRT2
_KI_DBL_EPS = sys.float_info.epsilon
_KI_INT_MAX = 2 ** 31 - 1


def _kiround(v: float, bits: int = 32) -> int:
    """KiCad's KiROUND: std::llround (halves away from zero; Python's round
    is half-to-even), clamped to the signed `bits`-bit range of its return
    type (int unless a caller asks for an ecoord)."""
    a = abs(v)
    r = math.floor(a)
    if a - r >= 0.5:
        r += 1
    r = int(r) if v >= 0 else -int(r)
    return min(max(r, -(1 << (bits - 1))), (1 << (bits - 1)) - 1)


def _board_iu(token: str) -> int:
    """A file coordinate as pcbnew's parser stores it: KiROUND(mm * 1e6) nm,
    clamped to INT_LIMIT (parseBoardUnits)."""
    lim = float(_KI_INT_MAX - 10)
    return _kiround(min(max(float(token) * 1e6, -lim), lim))


def _ki_div(a: float, b: float) -> float:
    """IEEE division as C++ does it: x/0 is a signed infinity and 0/0 NaN
    (Python raises instead)."""
    try:
        return a / b
    except ZeroDivisionError:
        if a != a or a == 0.0:
            return math.nan
        return math.copysign(math.inf, a) * math.copysign(1.0, b)


def _ki_floor(v: float) -> float:
    return v if not math.isfinite(v) else float(math.floor(v))


def _ki_norm_d(x: float, y: float) -> float:
    """VECTOR2D::EuclideanNorm."""
    if abs(x) == abs(y):
        return abs(x) * _KI_SQRT2
    if x == 0:
        return abs(y)
    if y == 0:
        return abs(x)
    return math.hypot(x, y)


def _ki_arc_center(start, mid, end) -> Tuple[int, int]:
    """CalcArcCenter(VECTOR2I start, mid, end): the double-precision center,
    snapped to a round 100 / 10 nm value when that lies inside its own
    propagated rounding uncertainty, then KiROUNDed."""
    sx, sy = float(start[0]), float(start[1])
    mx, my = float(mid[0]), float(mid[1])
    ex, ey = float(end[0]), float(end[1])
    yd21, xd21 = my - sy, mx - sx
    yd32, xd32 = ey - my, ex - mx
    if (xd21 == 0.0 and yd32 == 0.0) or (yd21 == 0.0 and xd32 == 0.0):
        cx, cy = (sx + ex) / 2.0, (sy + ey) / 2.0
    else:
        if xd21 == 0.0:
            xd21 = _KI_DBL_EPS
        if xd32 == 0.0:
            xd32 = -_KI_DBL_EPS
        a_slope = yd21 / xd21
        b_slope = yd32 / xd32
        da_slope = a_slope * _ki_norm_d(_ki_div(0.5, yd21), _ki_div(0.5, xd21))
        db_slope = b_slope * _ki_norm_d(_ki_div(0.5, yd32), _ki_div(0.5, xd32))
        cx = cy = None
        if a_slope == b_slope:
            if sx == ex and sy == ey:
                cx, cy = (sx + mx) / 2.0, (sy + my) / 2.0
            else:
                a_slope += _KI_DBL_EPS
                b_slope -= _KI_DBL_EPS
        if cx is None:
            if a_slope == 0.0:
                a_slope = 1e-10
            if b_slope == 0.0:
                b_slope = 1e-10
            ab_y = a_slope * b_slope * (sy - ey)
            t = _ki_div(_KI_SQRT1_2, sy - ey)
            dab_y = ab_y * math.sqrt((da_slope / a_slope * da_slope / a_slope)
                                     + (db_slope / b_slope * db_slope / b_slope)
                                     + (t * t))
            b_sm = b_slope * (sx + mx)
            t = _ki_div(_KI_SQRT1_2, sx + mx)
            db_sm = b_sm * math.sqrt((db_slope / b_slope * db_slope / b_slope) + (t * t))
            a_me = a_slope * (mx + ex)
            t = _ki_div(_KI_SQRT1_2, mx + ex)
            da_me = a_me * math.sqrt((da_slope / a_slope * da_slope / a_slope) + (t * t))
            twice = 2 * (b_slope - a_slope)
            dtwice = 2 * math.sqrt(db_slope * db_slope + da_slope * da_slope)
            num_x = ab_y + b_sm - a_me
            dnum_x = math.sqrt(dab_y * dab_y + db_sm * db_sm + da_me * da_me)
            cx = _ki_div(ab_y + b_sm - a_me, twice)
            q1 = _ki_div(dnum_x, num_x)
            q2 = _ki_div(dtwice, twice)
            dcx = cx * math.sqrt((q1 * q1) + (q2 * q2))
            num_y = (sx + mx) / 2.0 - cx
            dnum_y = math.sqrt(1.0 / 8.0 + dcx * dcx)
            first = num_y / a_slope
            q1 = _ki_div(dnum_y, num_y)
            q2 = da_slope / a_slope
            dfirst = first * math.sqrt((q1 * q1) + (q2 * q2))
            cy = first + (sy + my) / 2.0
            dcy = math.sqrt(dfirst * dfirst + 1.0 / 8.0)
            r100x = _ki_floor((cx + 50.0) / 100.0) * 100.0
            r100y = _ki_floor((cy + 50.0) / 100.0) * 100.0
            r10x = _ki_floor((cx + 5.0) / 10.0) * 10.0
            r10y = _ki_floor((cy + 5.0) / 10.0) * 10.0
            if abs(r100x - cx) < dcx and abs(r100y - cy) < dcy:
                cx, cy = r100x, r100y
            elif abs(r10x - cx) < dcx and abs(r10y - cy) < dcy:
                cx, cy = r10x, r10y
    lo, hi = float(-_KI_INT_MAX - 1 + 100), float(_KI_INT_MAX - 100)

    def snap(v):
        if v != v:
            return 0
        return _kiround(min(max(v, lo), hi))
    return snap(cx), snap(cy)


def _ki_angle_deg(x: float, y: float) -> float:
    """EDA_ANGLE(VECTOR2D): exact at the eight 45-degree directions."""
    if x == 0.0 and y == 0.0:
        return 0.0
    if y == 0.0:
        return 0.0 if x >= 0 else -180.0
    if x == 0.0:
        return 90.0 if y >= 0.0 else -90.0
    if x == y:
        return 45.0 if x >= 0.0 else -180.0 + 45.0
    if x == -y:
        return -45.0 if x >= 0.0 else 180.0 - 45.0
    return math.atan2(y, x) / _KI_DEG_TO_RAD


def _ki_normalize_deg(v: float) -> float:
    while v < -0.0:
        v += 360.0
    while v >= 360.0:
        v -= 360.0
    return v


def _ki_cos(deg: float) -> float:
    t = _ki_normalize_deg(deg)
    if t == 0.0:
        return 1.0
    if t == 180.0:
        return -1.0
    if t == 90.0 or t == 270.0:
        return 0.0
    if t == 45.0 or t == 315.0:
        return _KI_SQRT1_2
    if t == 135.0 or t == 225.0:
        return -_KI_SQRT1_2
    return math.cos(deg * _KI_DEG_TO_RAD)


def _ki_sin(deg: float) -> float:
    t = _ki_normalize_deg(deg)
    if t == 0.0 or t == 180.0:
        return 0.0
    if t == 45.0 or t == 135.0:
        return _KI_SQRT1_2
    if t == 225.0 or t == 315.0:
        return -_KI_SQRT1_2
    if t == 90.0:
        return 1.0
    if t == 270.0:
        return -1.0
    return math.sin(deg * _KI_DEG_TO_RAD)


def _ki_arc_to_segment_count(radius: int, max_error: int, angle_deg: float) -> int:
    """GetArcToSegmentCount."""
    radius = max(1, radius)
    max_error = max(1, max_error)
    rel_error = max_error / radius
    try:
        inc = 180 / math.pi * math.acos(1.0 - rel_error) * 2
    except ValueError:
        inc = math.nan
    inc = inc if inc < 360.0 / 8 else 360.0 / 8      # std::min(45, inc)
    return max(_kiround(abs(angle_deg) / inc), 2)


def _ki_circle_to_end_segment_delta_radius(radius: int, seg_count: int) -> int:
    """CircleToEndSegmentDeltaRadius."""
    if seg_count <= 2:
        seg_count = 3
    alpha = math.pi / seg_count
    return _kiround(abs(radius * (1 - 1 / math.cos(alpha))))


def _ki_seg_distance(a, b, p) -> int:
    """SEG(a, b).Distance(p): the integer floor of the true distance."""
    abx, aby = b[0] - a[0], b[1] - a[1]
    apx, apy = p[0] - a[0], p[1] - a[1]
    e = apx * abx + apy * aby
    if e <= 0:
        d2 = apx * apx + apy * apy
    else:
        f = abx * abx + aby * aby
        if e >= f:
            d2 = (p[0] - b[0]) ** 2 + (p[1] - b[1]) ** 2
        else:
            g = float(apx * apx + apy * apy) - (float(e) * e) / f
            d2 = 0 if (g < 0 or g > float(2 ** 63 - 1)) else _kiround(g, 64)
    return math.isqrt(d2)


def kicad_arc_polyline(start: Tuple[int, int], mid: Tuple[int, int],
                       end: Tuple[int, int],
                       max_error: int = ZONE_ARC_MAX_ERROR_NM) -> List[Tuple[int, int]]:
    """The polyline pcbnew holds for a three-point arc, in integer nm:
    SHAPE_ARC(start, mid, end, 0).ConvertToPolyline(max_error), KiCad 10.0.0.

    The start and end points are exact; between them each vertex sits at the
    middle of its angular step, on a radius pushed out by half the effective
    error so the chords straddle the arc."""
    start, mid, end = tuple(start), tuple(mid), tuple(end)
    c = _ki_arc_center(start, mid, end)
    dx, dy = float(start[0] - c[0]), float(start[1] - c[1])
    r = math.sqrt(dx * dx + dy * dy)
    sa = _ki_normalize_deg(_ki_angle_deg(dx, dy))
    if start == end:
        ca = 360.0
    else:
        ca = (_ki_angle_deg(float(end[0] - c[0]), float(end[1] - c[1]))
              - _ki_angle_deg(dx, dy))
        v1 = (end[0] - mid[0], end[1] - mid[1])
        v2 = (start[0] - mid[0], start[1] - mid[1])
        if v1[0] * v2[1] - v1[1] * v2[0] > 0:        # IsCCW
            if ca < 0.0:
                ca += 360.0
        elif ca > 0.0:
            ca -= 360.0
    half = max(1.0, max_error / 2.0)
    if r < half or _ki_seg_distance(start, end, mid) < half:
        n = 0
        eff = r
    else:
        # The radius is passed as an int. Past 2^31 nm (a 2.1 m radius, an
        # arc indistinguishable from its chord) that conversion is undefined
        # in C++; arm64 saturates it, which is what this does.
        ir = min(int(r), _KI_INT_MAX)
        n = _ki_arc_to_segment_count(ir, max_error, ca)
        seg360 = int(n * 360.0 / abs(ca))
        eff = float(_ki_circle_to_end_segment_delta_radius(ir, seg360))
    r += eff / 2
    n *= 2
    pts = [start]
    for i in range(1, n, 2):
        a = sa + (ca * i) / n
        p = (_kiround(c[0] + r * _ki_cos(a)), _kiround(c[1] + r * _ki_sin(a)))
        if p != pts[-1]:
            pts.append(p)
    if end != pts[-1]:
        pts.append(end)
    return pts


_OUTLINE_VERTEX_RE = re.compile(r'\((xy|arc)(?=[\s(])')
_OUTLINE_XY_RE = re.compile(r'\(xy\s+([-+\d.eE]+)\s+([-+\d.eE]+)\s*\)')
_OUTLINE_ARC_PT_RE = re.compile(r'\((start|mid|end)\s+([-+\d.eE]+)\s+([-+\d.eE]+)\s*\)')


class _KicadOutlineChain:
    """The part of SHAPE_LINE_CHAIN (KiCad 10.0.0) a zone outline goes
    through on load: Append(VECTOR2I), Append(SHAPE_ARC) and SetClosed(true).
    Each point carries KiCad's pair of arc indices (first, second; -1 = none),
    because closing an outline can ROTATE it: mergeFirstLastPointIfNeeded
    drops a last point repeating the first and fixIndicesRotation then turns
    the chain until it starts on a plain point or at an arc's start.

    `closed=True` is a chain that is closed BEFORE its points arrive -- a
    gr_poly / fp_poly's (EDA_SHAPE::SetPolyPoints -> SHAPE_POLY_SET::NewOutline
    closes it): there the merge runs after every arc, never after an xy, so a
    polygon whose last xy repeats its first point keeps it."""

    def __init__(self, closed=False):
        self.closed = closed
        self.keys = []        # (x_nm, y_nm)
        self.mm = []          # (x_mm, y_mm) as the file spells an xy point
        self.shapes = []      # [first, second] arc index per point
        self.arcs = []        # (start_nm, end_nm) of each arc kept as an arc

    def _shared(self, i):
        return self.shapes[i][0] != -1 and self.shapes[i][1] != -1

    def _arc_index(self, i):
        return self.shapes[i][1] if self._shared(i) else self.shapes[i][0]

    def append_xy(self, key, mm):
        if not self.keys or self.keys[-1] != key:
            self.keys.append(key)
            self.mm.append(mm)
            self.shapes.append([-1, -1])

    def append_arc(self, start, mid, end):
        chain = kicad_arc_polyline(start, mid, end)
        idx = -1
        if len(chain) > 2:
            idx = len(self.arcs)
            self.arcs.append((tuple(start), tuple(end)))
        if not self.keys or chain[0] != self.keys[-1]:
            self.keys.append(chain[0])
            self.mm.append((chain[0][0] / 1e6, chain[0][1] / 1e6))
            self.shapes.append([idx, -1])
        elif idx != -1 and len(chain) > 1:
            # the arc's first segment hangs off the point already held
            if self.shapes[-1] == [-1, -1]:
                self.shapes[-1][0] = idx
            else:
                self.shapes[-1][1] = idx
        for p in chain[1:]:
            self.keys.append(p)
            self.mm.append((p[0] / 1e6, p[1] / 1e6))
            self.shapes.append([idx, -1])
        if self.closed:
            self._merge_closed()
        # mergeFirstLastPointIfNeeded, open branch
        elif len(self.keys) > 1 and self._shared(0):
            self.keys.append(self.keys[0])
            self.mm.append(self.mm[0])
            self.shapes.append([self.shapes[0][0], -1])
            self.shapes[0] = [self.shapes[0][1], -1]

    def _is_arc_segment(self, i):
        nxt = i + 1
        if nxt > len(self.shapes) - 1:
            if nxt == len(self.shapes) and self._shared(0):    # closed
                nxt = 0
            else:
                return False
        return self.shapes[i] != [-1, -1] and self._arc_index(i) == self.shapes[nxt][0]

    def _is_arc_start(self, i):
        if not self._is_arc_segment(i):
            return False
        if self._shared(i):
            return True
        return self.arcs[self._arc_index(i)][0] == self.keys[i]

    def close(self):
        """SetClosed(true)."""
        self.closed = True
        self._merge_closed()

    def _merge_closed(self):
        """mergeFirstLastPointIfNeeded, closed branch."""
        if len(self.keys) > 1 and self.keys[0] == self.keys[-1]:
            last = self._arc_index(len(self.keys) - 1)
            if last != -1:
                self.shapes[0] = [last, self.shapes[0][0]]
            self.keys.pop()
            self.mm.pop()
            self.shapes.pop()
            # fixIndicesRotation
            if len(self.shapes) <= 1:
                return
            rotations = 0
            while self._arc_index(0) != -1 and not self._is_arc_start(0):
                for lst in (self.keys, self.mm, self.shapes):
                    lst.insert(0, lst.pop())
                if rotations > len(self.shapes):
                    return
                rotations += 1


def _read_outline_chain(pts_text: str, closed: bool) -> '_KicadOutlineChain':
    """Every `(xy ..)` / `(arc ..)` vertex of a `(pts ...)` through a
    `_KicadOutlineChain`, as KiCad's parseOutlinePoints feeds them."""
    chain = _KicadOutlineChain(closed=closed)
    pos = 0
    while True:
        m = _OUTLINE_VERTEX_RE.search(pts_text, pos)
        if not m:
            break
        if m.group(1) == 'xy':
            xm = _OUTLINE_XY_RE.match(pts_text, m.start())
            pos = m.end()
            if not xm:
                continue
            chain.append_xy((_board_iu(xm.group(1)), _board_iu(xm.group(2))),
                            (float(xm.group(1)), float(xm.group(2))))
            pos = xm.end()
            continue
        end = find_matching_paren(pts_text, m.start())
        body = pts_text[m.start():end]
        pos = end
        got = {k: (_board_iu(x), _board_iu(y))
               for k, x, y in _OUTLINE_ARC_PT_RE.findall(body)}
        if len(got) != 3:
            continue          # pcbnew refuses the file; nothing to mirror
        chain.append_arc(got['start'], got['mid'], got['end'])
    return chain


def zone_outline_points(pts_text: str) -> List[Tuple[float, float]]:
    """The vertices of one zone / rule-area ``(pts ...)`` as pcbnew loads it
    (mm): each ``(xy X Y)`` as written, each ``(arc (start) (mid) (end))``
    replaced by `kicad_arc_polyline` (vertices on the nm grid), a point
    equal on the nm grid to the one before it dropped, the way
    SHAPE_LINE_CHAIN::Append skips it, and a last point repeating the first
    dropped once: parseZONE closes the outline (SetClosed(true) ->
    mergeFirstLastPointIfNeeded) in every file format. KiCad never writes
    one, but a KiCad 5-converted board does on every zone, and an outline
    can end on its own start through an arc -- after which pcbnew turns the
    chain to start on a plain point or an arc's start (`_KicadOutlineChain`)."""
    chain = _read_outline_chain(pts_text, closed=False)
    chain.close()
    return chain.mm


def kicad_poly_shape_points(pts_text: str) -> List[Tuple[float, float]]:
    """The vertices of a `gr_poly` / `fp_poly` ``(pts ...)`` as pcbnew holds
    them in GetPolyShape().Outline(0) (mm, before a footprint's transform):
    arcs as `kicad_arc_polyline`, consecutive duplicates dropped -- but the
    chain is closed before its points arrive, so a last `xy` repeating the
    first is KEPT (pcbnew keeps it; Castor_and_Pollux's faceplate outline)
    and only an arc that ends on the first point merges it."""
    return _read_outline_chain(pts_text, closed=True).mm


def extract_zones(content: str, name_to_id: Dict[str, int] = None) -> List[Zone]:
    """Extract all filled zones from PCB file.

    Parses zone definitions including their net assignment, layer, and polygon outline.
    These are used for power planes and other filled copper areas.
    """
    zones = []
    copper_layers = None  # the board's copper stack, read once if a zone says *.Cu

    for zone_content, in_footprint in _iter_zone_blocks(content):
        # Rule areas are routing restrictions, not copper -- skip regardless
        # of net clause. v5/v6 keepouts DO carry (net 0), so the old skip
        # (inside the no-numeric-net branch only) would have modeled them as
        # phantom net-0 pours once single-line zones parse (#369 A5).
        if '(keepout' in zone_content:
            continue
        # Extract net id - try KiCad 9 format first, then KiCad 10
        net_match = re.search(r'\(net\s+(\d+)\)', zone_content)
        net_match_v10 = None
        if net_match:
            net_id = int(net_match.group(1))
        else:
            # KiCad 10: (net "name") - first net reference in zone. Look up
            # the RAW token: name_to_id is keyed by the raw FILE TEXT, not by
            # the unescaped Net.name -- extract_nets says so and every other
            # site (pads, segments, vias) reads it that way. This comment used
            # to claim the opposite and unescape first, which is the same
            # defect #369 A12 meant to fix: measured, a zone on
            # `/GPIO22\\I2C1_SDA` resolved to net 0 while the identical
            # segment token resolved to 4.
            if name_to_id:
                net_match_v10 = re.search(r'\(net\s+"%s"\)' % _ESC_STR, zone_content)
            if net_match_v10:
                net_id = name_to_id.get(net_match_v10.group(1), 0)
                # The DISPLAY name is unescaped, which is all #369 A12 was
                # ever about (a raw net_name evaded --nets filters and zone
                # dedup keys). The LOOKUP above is the half that must stay raw.
                net_name_v10 = _unescape_kicad_string(net_match_v10.group(1))
            else:
                # No net clause at all: a NO-NET copper pour (net 0). It still
                # pours real copper (nitrokey decorative fills), and pcbnew
                # keeps it -- dropping it under-modelled the board. (vfo_ctrl's
                # net-less zones are F.Mask / B.Mask openings, not copper; the
                # non-copper skip below drops those on both fronts.)
                net_id = 0

        # Extract net name. Unescaped like every other net_name in the model
        # (#369 A12: zones kept the RAW file text, so backslash/quote-named
        # plane nets evaded --nets filters and zone dedup keys).
        net_name_match = re.search(r'\(net_name\s+"%s"\)' % _ESC_STR, zone_content)
        if net_name_match:
            net_name = _unescape_kicad_string(net_name_match.group(1))
        elif net_match_v10 is not None:
            # KiCad 10: net name was already captured (and unescaped) above
            net_name = net_name_v10
        else:
            net_name = ""

        # Extract layer(s). A zone can span several copper layers via
        # (layers "F.Cu" "In2.Cu" ...) -- emit one Zone per copper layer so the
        # obstacle/connectivity model sees its copper everywhere it exists
        # (single-layer (layer "...") zones are the common case). Search the
        # zone HEADER only: each (filled_polygon ...) inside the zone carries
        # its own (layer "...") tag, which would mask the (layers ...) clause
        # and pin a 5-layer zone to just its first filled layer.
        _hdr_end = len(zone_content)
        for _tok in ('(polygon', '(filled_polygon', '(keepout'):
            _i = zone_content.find(_tok)
            if _i != -1:
                _hdr_end = min(_hdr_end, _i)
        zone_header = zone_content[:_hdr_end]
        # Layer tokens are UNQUOTED in v5/v6 files ((layer F.Cu)) -- accept
        # both spellings (#369 A5).
        layer_match = re.search(r'\(layer\s+"?([^")\s]+)"?\)', zone_header)
        if layer_match:
            # A zone on one NON-copper layer (a mask opening, a silkscreen
            # fill) is not copper -- pcbnew strips its net on load -- and
            # pcb.zones is copper: skip it, as a (layers ...) zone naming no
            # copper layer always was, and as build_pcb_data_from_board does.
            if not layer_match.group(1).endswith('.Cu'):
                continue
            zone_layers = [layer_match.group(1)]
        else:
            layers_match = re.search(r'\(layers\s+([^)]+)\)', zone_header)
            if not layers_match:
                continue
            # KiCad 6 spells a two-face pour (layers F&B.Cu) and an all-copper
            # one (layers *.Cu); pcbnew loads them as F.Cu + B.Cu and as every
            # copper layer. Kept literally, each became ONE zone on a layer
            # nothing else recognizes, so the pour vanished from the model.
            zone_layers = []
            for l in layer_list_tokens(layers_match.group(1)):
                if l == 'F&B.Cu':
                    names = ['F.Cu', 'B.Cu']
                elif l == '*.Cu':
                    if copper_layers is None:
                        copper_layers = _layer_table(content)[1]
                    names = copper_layers
                else:
                    names = [l] if l.endswith('.Cu') else []
                for nm in names:
                    if nm not in zone_layers:
                        zone_layers.append(nm)
            if not zone_layers:
                continue
        layer = zone_layers[0]

        uuid = uuid_or_tstamp(zone_header)  # KiCad 6 zones carry (tstamp ...)

        # Fill semantics (#350). All three live before the polygon blocks, so
        # the header slice covers them: (priority N) is a direct zone child;
        # island_removal_mode / island_area_min sit inside the (fill ...) block.
        # Absent tokens mean KiCad defaults: priority 0, mode 0 (always remove
        # isolated islands).
        prio_match = re.search(r'\(priority\s+(\d+)\)', zone_header)
        priority = int(prio_match.group(1)) if prio_match else 0
        irm_match = re.search(r'\(island_removal_mode\s+(\d+)\)', zone_header)
        island_removal_mode = int(irm_match.group(1)) if irm_match else 0
        iam_match = re.search(r'\(island_area_min\s+([\d.]+)\)', zone_header)
        island_area_min = float(iam_match.group(1)) if iam_match else 0.0
        # Fill-model inputs (#validator-parity): the pour's own clearance
        # lives under (connect_pads ... (clearance X)); min_thickness is a
        # direct child. Absent (older files) -> None; the fill model then
        # falls back to routing_defaults.
        zcl_match = re.search(r'\(clearance\s+([\d.]+)\)', zone_header)
        zone_clearance_val = float(zcl_match.group(1)) if zcl_match else None
        mth_match = re.search(r'\(min_thickness\s+([\d.]+)\)', zone_header)
        zone_min_th = float(mth_match.group(1)) if mth_match else None
        # #1329: the pad connection is the bare word after connect_pads (none
        # = KiCad's default, thermal relief); the spoke settings sit inside
        # (fill ...). The build_pcb_data_from_board twin reads the same three
        # off pcbnew.
        zone_pad_conn, zone_th_gap, zone_th_bridge = zone_connection_settings(
            zone_header)
        # #1164: a teardrop area is (attr (teardrop (type padvia|track_end)))
        # in the header. A pad's own `(teardrops ...)` settings never reach a
        # zone block.
        is_teardrop = re.search(r'\(attr\s*\(teardrop[\s)]', zone_header) is not None

        # Extract polygon points - find (pts ...) and extract xy coordinates
        pts_start = zone_content.find('(pts')
        if pts_start < 0:
            continue

        # Find matching closing paren for (pts
        paren_count = 0
        pts_end = pts_start
        for i in range(pts_start, len(zone_content)):
            if zone_content[i] == '(':
                paren_count += 1
            elif zone_content[i] == ')':
                paren_count -= 1
                if paren_count == 0:
                    pts_end = i
                    break

        pts_content = zone_content[pts_start:pts_end + 1]
        # The outline as pcbnew loads it (`zone_outline_points`): an
        # `(arc ...)` vertex becomes KiCad's own polyline -- reading only
        # `(xy ...)` dropped an all-arc outline's zone outright (key_ripper's
        # F&B.Cu GND pour) and cut the arcs out of a mixed one -- and a
        # CONSECUTIVE coincident vertex is dropped, as
        # SHAPE_LINE_CHAIN::Append (integer-nm compare) skips it on load
        # (route_planes rewrites can emit these; parser parity used to flag a
        # phantom vertex-count mismatch), and so is a last vertex repeating
        # the first (pcbnew closes the outline as it parses it; a KiCad
        # 5-converted board writes one on every zone). Collinear runs are KEPT.
        polygon = zone_outline_points(pts_content)

        if not polygon:
            continue

        # #1279: every later (polygon ...) block of the zone is a hole in its
        # outline; the (filled_polygon ...) blocks after them are the fill.
        holes = []
        _fill_at = zone_content.find('(filled_polygon')
        for _hm in re.finditer(r'\(polygon\b', zone_content):
            if _hm.start() <= pts_start or (_fill_at != -1 and _hm.start() > _fill_at):
                continue
            _he = find_matching_paren(zone_content, _hm.start())
            _hole = zone_outline_points(zone_content[_hm.start():_he])
            if len(_hole) >= 3:
                holes.append(_hole)

        for zl in zone_layers:
            zones.append(Zone(
                net_id=net_id,
                net_name=net_name,
                layer=zl,
                polygon=polygon,
                uuid=uuid,
                priority=priority,
                island_removal_mode=island_removal_mode,
                island_area_min=island_area_min,
                clearance=zone_clearance_val,
                min_thickness=zone_min_th,
                pad_connection=zone_pad_conn,
                thermal_gap=zone_th_gap,
                thermal_bridge_width=zone_th_bridge,
                in_footprint=in_footprint,
                teardrop=is_teardrop,
                holes=holes,
            ))

    return zones


def zone_connection_settings(zone_header: str):
    """(pad_connection, thermal_gap, thermal_bridge_width) of one zone block's
    header text (#1329) -- see `Zone.pad_connection`. A zone with no
    `(connect_pads ...)` clause connects by thermal relief, KiCad's default;
    an absent spoke token reads None (unknown), never a guessed default."""
    m = re.search(r'\(connect_pads(?:\s+([a-z_]+))?', zone_header)
    word = (m.group(1) if m else None) or ''
    pad_connection = {'': 'thermal', 'yes': 'solid', 'no': 'none',
                      'thru_hole_only': 'thru_hole_only'}.get(word, word)
    g = re.search(r'\(thermal_gap\s+([\d.]+)\)', zone_header)
    b = re.search(r'\(thermal_bridge_width\s+([\d.]+)\)', zone_header)
    return (pad_connection, float(g.group(1)) if g else None,
            float(b.group(1)) if b else None)


#: KiCad's ZONE_CONNECTION enum as a pad or footprint `(zone_connect N)` writes
#: it, spelled like Zone.pad_connection. INHERITED (-1) is never written.
ZONE_CONNECT_NAMES = {0: 'none', 1: 'thermal', 2: 'solid', 3: 'thru_hole_only'}

_ZONE_CONNECT_RE = re.compile(r'\(zone_connect\s+(-?\d+)\)')


def zone_connect_from_text(text: str) -> Optional[str]:
    """The `(zone_connect N)` override in a pad block or a footprint header,
    spelled like Zone.pad_connection; None = not set (inherit)."""
    m = _ZONE_CONNECT_RE.search(text)
    return ZONE_CONNECT_NAMES.get(int(m.group(1))) if m else None


def zone_connect_from_pcbnew(item) -> Optional[str]:
    """The pcbnew twin of zone_connect_from_text, for a PAD or a FOOTPRINT:
    its OWN override (GetLocalZoneConnection), never an inherited one."""
    try:
        return ZONE_CONNECT_NAMES.get(int(item.GetLocalZoneConnection()))
    except Exception:
        return None


def pad_zone_connection(pad, zone) -> str:
    """How `zone` connects to `pad`: 'none', 'thermal' or 'solid' -- KiCad's
    resolution (DRC_ENGINE::EvalZoneConnection): the pad's own override, else
    its footprint's (both already folded into Pad.zone_connection), else the
    zone's connect_pads (thermal when the zone does not say). Thermal-for-PTH
    ('thru_hole_only') gives a plated through-hole pad a thermal relief and
    every other pad a solid joint. A `zone_connection` custom rule in the
    .kicad_dru (which KiCad ranks between the pad's and the footprint's
    override) is not read."""
    mode = (getattr(pad, 'zone_connection', None)
            or getattr(zone, 'pad_connection', None) or 'thermal')
    if mode == 'thru_hole_only':
        return 'thermal' if getattr(pad, 'pad_type', '') == 'thru_hole' else 'solid'
    return mode


def pad_joins_zone(pad, zone) -> bool:
    """False when KiCad keeps `zone`'s fill off this pad of its own net
    (connection 'none'): the pour then pulls back by its clearance, and the
    pad reaches the net only through a track, a via or another pad."""
    return pad_zone_connection(pad, zone) != 'none'


def extract_keepouts(content: str) -> List[dict]:
    """Extract keep-out rule areas (zones with a (keepout ...) clause and no net fill).

    These define regions where tracks, vias, zone fill, pads and/or
    footprints are not allowed — e.g. an antenna-flange RF clearance. Returns
    a list of dicts:
        {polygon: [(x,y),...], layers: set(layer_names),
         tracks_allowed: bool, vias_allowed: bool, copper_pour_allowed: bool,
         pads_allowed: bool, footprints_allowed: bool,
         in_footprint: bool, owner: str, name: str}
    `owner` is the key (as `parse_kicad_pcb` names it, `iter_footprint_blocks`)
    of the footprint a footprint-owned area sits in, '' for a board-level one.
    A pad or a footprint inside an area forbidding it is KiCad's
    `items_not_allowed` (#1451, `legality.rule_area_item_findings`).
    `name` is the area's `(name ...)` ('' when it has none): a .kicad_dru
    condition names the area it restricts (`A.intersectsArea('corridor')`),
    and an area that allows everything can still be one (net_keepouts).
    """
    keepouts = []
    spans = None
    for zone_at, zc, in_footprint in _iter_zone_spans(content):
        # The keepout clause holds nested sub-clauses, e.g.
        #   (keepout (tracks not_allowed) (vias not_allowed) (pads allowed) ...)
        # so capture its full balanced-paren body rather than just the first ).
        ko_start = zc.find('(keepout')
        if ko_start < 0:
            continue
        pc = 0
        ko_end = ko_start
        for i in range(ko_start, len(zc)):
            if zc[i] == '(':
                pc += 1
            elif zc[i] == ')':
                pc -= 1
                if pc == 0:
                    ko_end = i
                    break
        ko_body = zc[ko_start:ko_end + 1]
        tracks_allowed = 'tracks not_allowed' not in ko_body
        vias_allowed = 'vias not_allowed' not in ko_body
        # #477: KiCad's filler subtracts (copperpour not_allowed) areas from
        # zone fill, splitting planes into islands -- the fill model must see
        # them or it grades keep-out-split planes as one piece.
        copper_pour_allowed = 'copperpour not_allowed' not in ko_body
        pads_allowed = 'pads not_allowed' not in ko_body
        footprints_allowed = 'footprints not_allowed' not in ko_body
        owner = ''
        if in_footprint:
            if spans is None:
                spans = [(s0, e0, key) for s0, e0, _t, _r, key in iter_footprint_blocks(content)]
            owner = next((key for s0, e0, key in spans if s0 <= zone_at < e0), '')

        # Layers: (layers "F.Cu" "In1.Cu" ...) or single (layer "F.Cu").
        # v5/v6 write tokens UNQUOTED ((layers F&B.Cu)) -- accept both (#369 A5).
        lm = re.search(r'\(layers\s+([^)]+)\)', zc) or re.search(r'\(layer\s+("?[^")\s]+"?)\)', zc)
        layers = ({t.strip('"') for t in re.findall(r'"[^"]+"|\S+', lm.group(1))}
                  if lm else set())

        # Outline polygons. A rule area can have several (polygon (pts ...))
        # blocks: the first is the outer outline, any further ones are HOLES
        # (even-odd fill). An edge-keepout ring is the board rectangle with an
        # inset hole; parsing only the first polygon collapses the ring to its
        # bounding box and bans the whole board (issue #95). Capture them all.
        polys = []
        search = 0
        while True:
            p_start = zc.find('(polygon', search)
            if p_start < 0:
                break
            pc = 0
            p_end = p_start
            for i in range(p_start, len(zc)):
                if zc[i] == '(':
                    pc += 1
                elif zc[i] == ')':
                    pc -= 1
                    if pc == 0:
                        p_end = i
                        break
            # As pcbnew loads it, `(arc ...)` vertices included: a rule area
            # drawn with rounded corners kept only its straight runs, and an
            # all-arc one (a round keep-out) was dropped.
            pts = zone_outline_points(zc[p_start:p_end + 1])
            if len(pts) >= 3:
                polys.append(pts)
            search = p_end + 1
        if not polys:
            continue
        # The zone's own (name "..."): the first one in the block -- no
        # sub-clause of a zone carries a (name ...) of its own.
        nm = re.search(r'\(name\s+"((?:[^"\\]|\\.)*)"\s*\)', zc)
        keepouts.append({'polygon': polys[0], 'holes': polys[1:], 'layers': layers,
                         'tracks_allowed': tracks_allowed, 'vias_allowed': vias_allowed,
                         'copper_pour_allowed': copper_pour_allowed,
                         'pads_allowed': pads_allowed,
                         'footprints_allowed': footprints_allowed,
                         'in_footprint': in_footprint, 'owner': owner,
                         'name': nm.group(1).replace('\\"', '"') if nm else ''})
    return keepouts


def release_text_caches() -> None:
    """Drop the text-keyed memos (`_mask_pad_primitives`,
    `_footprint_blocks_by_key`, `_footprint_edge_points_by_ref`).

    They exist so ONE parse asks each whole-file question once, and they are
    keyed on the board's whole text: kept past the parse they hold that text
    and a masked copy of it, 216 MB for a 54 MB board (ddr4_rdimm_tester,
    whose one non-ASCII character makes the text two bytes a character), and
    with `maxsize=2` twice that. `parse_kicad_pcb` calls this as it returns;
    a writer that scans a board's text calls it when done."""
    _mask_pad_primitives.cache_clear()
    _footprint_blocks_by_key_cached.cache_clear()
    _footprint_edge_points_by_ref_cached.cache_clear()


def parse_kicad_pcb(filepath: str, guide_layer: str = "User.1",
                    keepout_layer: str = "User.2") -> PCBData:
    """
    Parse a KiCad PCB file and extract all routing-relevant information.

    Args:
        filepath: Path to .kicad_pcb file
        guide_layer: User layer to read guide corridor polylines from (issue #7)
        keepout_layer: User layer to read keepout polygons from (issue #27)

    Returns:
        PCBData object containing all parsed data
    """
    global _PARSE_MEMO
    outer = _PARSE_MEMO is None
    if outer:
        _PARSE_MEMO = {'live': {}, 'masked_from': {}, 'ends': {}, 'shapes': {}}
    try:
        return _parse_kicad_pcb(filepath, guide_layer, keepout_layer)
    finally:
        if outer:
            _PARSE_MEMO = None
        release_text_caches()


def _parse_kicad_pcb(filepath: str, guide_layer: str,
                     keepout_layer: str) -> PCBData:
    content = read_board_text(filepath)

    kicad_version = detect_kicad_version(content)
    if 0 < kicad_version < FIRST_SUPPORTED_BOARD_VERSION:
        raise UnsupportedBoardFormat(
            f"{filepath} is a KiCad 5 (or older) board, file format "
            f"{kicad_version}. This tool reads KiCad 6 and later; it would see "
            f"no footprints or nets in this file. Open the board in KiCad 6 or "
            f"newer and save it, which converts it, or use the KiCad plugin, "
            f"which reads the board through KiCad itself.")
    # A file from before 6.0's arc format: rewrite its center/angle arcs as
    # start/mid/end on this ANALYSIS copy, the form every reader below matches.
    content = strip_bare_shape_locks(upgrade_legacy_arcs(content, kicad_version))
    if _PARSE_MEMO is not None:
        _PARSE_MEMO['live'][id(content)] = content

    # Extract components in order
    board_info = extract_layers(content)
    nets, name_to_id = extract_nets(content, kicad_version)
    _dups: Dict[str, int] = {}
    footprints, pads_by_net = extract_footprints_and_pads(
        content, nets, name_to_id, duplicates=_dups,
        board_copper=board_info.copper_layers)
    if _dups:
        # ON STDERR, not stdout: 9 shipped call sites across 4 files emit bare
        # JSON on stdout (`route.py`, `converge.py`, `check_reachability.py`,
        # `krt_capabilities.py`), where a WARNING line in front is a
        # JSONDecodeError at char 0. Said at all because a duplicated
        # reference is a schematic
        # fact the operator has to decide about -- the parser keeps every
        # block, but two parts answering to one name is still something only
        # the human can resolve.
        _n = sum(_dups.values())
        print("WARNING: %d footprint block(s) share %d reference(s) -- %s. "
              "Every block is kept; the later ones are keyed %s2, %s3 ... in "
              "file order, so this board parses as %d parts. Legal in KiCad "
              "(a paired testpoint, a net tie), but rename them if they are "
              "meant to be distinct parts."
              % (_n, len(_dups),
                 ', '.join('%s x%d' % rc for rc in sorted(_dups.items())),
                 DUP_REF_SEP, DUP_REF_SEP, len(footprints)),
              file=sys.stderr)
    # #829: which footprints draw Edge.Cuts, and which of those are drawing the
    # BOARD's boundary rather than a relief that travels with the part. Stamped
    # here because it needs the footprints and the board-level contours
    # together, and neither extractor sees both. Keyed through #726's
    # `disambiguate_references`, so on a board where two blocks share a
    # reference the flag lands on the block that actually draws the outline.
    for _ref, _structural in footprint_outline_owners(content).items():
        _fp = footprints.get(_ref)
        if _fp is not None:
            _fp.owns_edge_cuts = True
            _fp.owns_board_outline = _structural
    vias = extract_vias(content, name_to_id)
    segments = extract_segments(content, name_to_id)
    zones = extract_zones(content, name_to_id)
    propagate_via_cluster_nets(content, vias, segments, footprints, zones,
                               board_info.copper_layers)
    if kicad_version and kicad_version < NEW_OVERBAR_NOTATION:
        _apply_new_overbar_notation(nets, footprints, zones)
    board_info.keepouts = extract_keepouts(content)
    guide_paths = parse_guide_paths(content, guide_layer)
    keepout_zones = parse_keepout_zones(content, keepout_layer)

    # Build net_id_to_name mapping for writer output
    net_id_to_name = {net_id: net.name for net_id, net in nets.items()}

    # Drop Edge.Cuts contours mis-classified as cutouts (they enclose pads).
    drop_pad_containing_cutouts(board_info, pads_by_net)

    groups = extract_groups(content, footprints)

    # #962: the paste stencil. The board setup provides the last rung of the
    # margin precedence and the via-protection policy; the apertures come from
    # the shared builder, fed this path's graphics.
    (board_info.pad_to_paste_clearance, board_info.pad_to_paste_clearance_ratio,
     board_info.via_protection_setup) = extract_board_setup_paste_and_protection(content)
    paste_apertures = build_paste_apertures(
        footprints, board_info, extract_paste_graphics(content))
    graphic_copper_unmeasured = extract_unmodelled_footprint_copper(content)

    pcb = PCBData(
        board_info=board_info,
        nets=nets,
        footprints=footprints,
        vias=vias,
        segments=segments,
        pads_by_net=pads_by_net,
        zones=zones,
        kicad_version=kicad_version,
        uses_name_nets=board_uses_name_nets(content),
        net_id_to_name=net_id_to_name,
        guide_paths=guide_paths,
        keepout_zones=keepout_zones,
        groups=groups,
        duplicate_references=_dups,
        source_path=os.path.abspath(filepath) if filepath else "",
        paste_apertures=paste_apertures,
        graphic_copper_unmeasured=graphic_copper_unmeasured,
    )
    # #1279: KiCad groups named `sense:<REF>.<PAD>` declare protected Kelvin
    # sense branches; resolved against the copper parsed above.
    from sense_branch import sense_groups_from_text, resolve_sense_branches
    pcb.sense_branches = resolve_sense_branches(pcb, sense_groups_from_text(content))
    return pcb


def propagate_via_cluster_nets(content: str, vias, segments, footprints, zones,
                               copper_layers) -> int:
    """Give a via the net of the copper it connects to, as pcbnew's
    connectivity pass does on load: a track or via takes its cluster's ORIGIN
    net -- the one net of the pads and zone fills it touches -- whatever net
    the file wrote (CN_CONNECTIVITY_ALGO::propagateConnections; a via joins a
    zone's FILL of any net, since a via can change net).

    antmicro lpddr4_test_board writes nine stitching vias, seven on `(net 0)`
    and two on `(net 9)` TMS_JTAG, inside GND fills on five inner layers with
    nothing else on them: pcbnew's bare reader loads them as written,
    `pcbnew.LoadBoard` and the KiCad GUI as GND, and kicad-cli grades them as
    GND.

    Bounded so an ordinary board pays almost nothing: only a via with no
    same-net track end and no same-net pad on it is a candidate (a routed via
    is never one), a via inside the outline of a pour of its OWN net on a
    layer it spans is taken to be that pour's stitching via and kept (a via
    in a void of its own pour is the one case this skips), and a zone's fills
    are read from the file only when a remaining candidate lies inside the
    OUTLINE of a zone of another net on a layer it spans. The origin nets are the fills containing the via's centre on its
    layers, plus -- as conflicts, since they stand for the pads behind them --
    the nets of other tracks ending in it and of pads whose box covers it. A
    via with exactly one origin net takes it; otherwise it keeps its own. A
    `(free yes)` via keeps its net, as in KiCad. Tracks are not re-netted.
    Returns how many vias changed."""
    if not vias:
        return 0
    order = {ln: i for i, ln in enumerate(copper_layers)}

    def span(v):
        ix = [order[ln] for ln in (v.layers or []) if ln in order]
        return set(copper_layers[min(ix):max(ix) + 1]) if ix else set()

    cell = 1.0
    ends = {}
    for s in segments:
        if getattr(s, 'graphic', False):
            continue
        for ex, ey in ((s.start_x, s.start_y), (s.end_x, s.end_y)):
            ends.setdefault((int(ex // cell), int(ey // cell)), []).append((ex, ey, s))
    # Each netted pad in every cell its bounding square covers, so a via asks
    # one cell: the pads whose square could hold its centre.
    pads = {}
    for fp in footprints.values():
        for p in fp.pads:
            if p.net_id:
                h = max(p.size_x, p.size_y) / 2.0
                for i in range(int((p.global_x - h) // cell), int((p.global_x + h) // cell) + 1):
                    for j in range(int((p.global_y - h) // cell), int((p.global_y + h) // cell) + 1):
                        pads.setdefault((i, j), []).append(p)

    def pads_on(x, y):
        return pads.get((int(x // cell), int(y // cell)), ())

    def near(table, x, y, reach):
        k = int(reach // cell) + 1
        cx, cy = int(x // cell), int(y // cell)
        for i in range(cx - k, cx + k + 1):
            for j in range(cy - k, cy + k + 1):
                yield from table.get((i, j), ())

    copper_zones = [z for z in zones if z.net_id and not getattr(z, 'teardrop', False)
                    and len(z.polygon) >= 3]
    zone_box = [(z, (min(p[0] for p in z.polygon), min(p[1] for p in z.polygon),
                     max(p[0] for p in z.polygon), max(p[1] for p in z.polygon)))
                for z in copper_zones]
    todo = []
    for v in vias:
        if getattr(v, 'free', False):
            continue        # `(free yes)`: KiCad never re-nets a free via
        layers = span(v)
        r = v.size / 2.0
        own = any(s.net_id == v.net_id and s.layer in layers
                  and (ex - v.x) ** 2 + (ey - v.y) ** 2 <= r * r
                  for ex, ey, s in near(ends, v.x, v.y, r)) if v.net_id else False
        own = own or (v.net_id and any(
            p.net_id == v.net_id and abs(p.global_x - v.x) <= max(p.size_x, p.size_y) / 2
            and abs(p.global_y - v.y) <= max(p.size_x, p.size_y) / 2
            for p in pads_on(v.x, v.y)))
        if own:
            continue
        over = [z for z, (x0, y0, x1, y1) in zone_box
                if z.layer in layers and x0 <= v.x <= x1 and y0 <= v.y <= y1
                and _pt_in_ring(v.x, v.y, z.polygon)]
        # Inside a pour of its OWN net it is that pour's stitching via: its
        # own net is an origin, so it cannot change (this skips the GND
        # vias every human-routed board carries, which used to cost
        # thunderscope 22 s of fill reading for no change at all).
        if any(z.net_id == v.net_id for z in over):
            continue
        if over:
            todo.append((v, layers, over))
    if not todo:
        return 0
    want = {z.uuid for _v, _l, over in todo for z in over if z.uuid}
    fills = {}          # (zone uuid, layer) -> [rings]
    for body, _in_fp in _iter_zone_blocks(content):
        if '(filled_polygon' not in body:
            continue
        # The zone's own id, from its header, as extract_zones reads it.
        _hdr = min([i for i in (body.find(t) for t in ('(polygon', '(filled_polygon',
                                                       '(keepout')) if i != -1]
                   or [len(body)])
        um = uuid_or_tstamp(body[:_hdr])
        if um not in want:
            continue
        for fm in re.finditer(r'\(filled_polygon\b', body):
            fb = body[fm.start():find_matching_paren(body, fm.start())]
            lm = re.search(r'\(layer\s+"([^"]+)"\)', fb)
            ring = [(float(x), float(y)) for x, y in
                    re.findall(r'\(xy\s+([-\d.]+)\s+([-\d.]+)\)', fb)]
            if lm and len(ring) >= 3:
                fills.setdefault((um, lm.group(1)), []).append(ring)
    changed = 0
    for v, layers, over in todo:
        origin = {z.net_id for z in over
                  if any(_pt_in_ring(v.x, v.y, ring)
                         for ring in fills.get((z.uuid, z.layer), ()))}
        if not origin:
            continue
        r = v.size / 2.0
        origin |= {s.net_id for ex, ey, s in near(ends, v.x, v.y, r)
                   if s.net_id and s.layer in layers
                   and (ex - v.x) ** 2 + (ey - v.y) ** 2 <= r * r}
        origin |= {p.net_id for p in pads_on(v.x, v.y)
                   if abs(p.global_x - v.x) <= max(p.size_x, p.size_y) / 2
                   and abs(p.global_y - v.y) <= max(p.size_x, p.size_y) / 2
                   and ('*.Cu' in p.layers or layers & set(p.layers))}
        if len(origin) == 1 and v.net_id not in origin:
            v.net_id = origin.pop()
            changed += 1
    return changed


def _apply_new_overbar_notation(nets, footprints, zones) -> None:
    """Rename ~X~ overbars to ~{X} on every DISPLAY net name, as pcbnew does on
    loading a file from before NEW_OVERBAR_NOTATION -- so `--nets "/~{RST}"`,
    which is how KiCad shows the net, matches on the CLI too. `name_to_id`
    keeps the raw file text: in-file references still resolve against it."""
    for net in nets.values():
        net.name = convert_to_new_overbar_notation(net.name)
    for fp in footprints.values():
        for pad in fp.pads:
            pad.net_name = convert_to_new_overbar_notation(pad.net_name or '')
    for zone in zones:
        zone.net_name = convert_to_new_overbar_notation(zone.net_name or '')


def _nm_quantize_bounds(bounds):
    """Snap board bounds to KiCad's own nanometre grid.

    GUI/CLI PARITY (do not drop). pcbnew stores geometry as integer
    nanometres, but this path converts to mm and then does float arithmetic
    on it -- the Edge.Cuts stroke deflation below is `x1 - half_stroke`, so a
    board edge at 223.57 with a 0.1 stroke yields 223.51999999999998 where
    the TEXT parser, reading the centerline decimal straight out of the file,
    yields 223.52.

    That residue is not cosmetic. board_bounds lays out the routing grid and
    the plane-fragility cost field, so an epsilon-narrower board shifts the
    last column: measured as a 795-cell difference in the fragility field
    (217841 GUI vs 217046 CLI) on splitflap_driver, which re-priced the A*
    and left the two fronts with different GND copper -- on boards whose
    copper, zones, pads and project rules were otherwise identical.

    1e-6 mm IS KiCad's resolution, so rounding there discards only error.
    Applied to the pcbnew path; the text parser's decimals are already exact
    at this precision, which is what makes the two agree.
    """
    if not bounds:
        return bounds
    return tuple(round(v, 6) for v in bounds)


def stage_live_project_rules(board_path: str, board):
    """Author `board_path`'s sibling .kicad_pro from the LIVE pcbnew board's
    in-memory design settings. Crash-safe replacement for the sibling that
    pcbnew.SaveBoard's implicit settings save used to write (#627).

    dab5c82 (the PR #613 SIGABRT fix) passed aSkipSettings=True at every
    SaveBoard, so GUI temp snapshots stopped carrying a sibling .kicad_pro
    -- and the exact-fill refill of such a snapshot fell back to pcbnew's
    STOCK rules (the "bare board refills at stock rules" trap refill_islands
    documents). The CLI's counterpart refill resolves the chain-stamped
    project (update_live_drc_floors mirrors those clamps into the live
    board's memory, but nothing put them back on disk for the subprocess),
    so the GUI's plane-fragility field priced the A* off a different fill:
    4 GND segments + 1 via on the parity board, one grid cell apart.

    Seeds from the board's real project file when one exists (non-Default
    classes, assignments, and every key we do not own survive verbatim),
    then overwrites the design-settings rule floors and the Default net
    class with the live in-memory values -- the only values the GUI mutates
    mid-session. Serializing plain values cannot re-trigger the KiCad 10
    settings-save merge crash. Best-effort: returns the written path, or
    None (callers refill bare, exactly today's behavior).
    """
    import json
    try:
        pro = os.path.splitext(board_path)[0] + '.kicad_pro'
        proj = None
        try:
            src = board.GetFileName() or ''
        except Exception:
            src = ''
        if src:
            src_pro = os.path.splitext(src)[0] + '.kicad_pro'
            if os.path.isfile(src_pro):
                try:
                    with open(src_pro, 'r', encoding='utf-8') as fh:
                        proj = json.load(fh)
                except Exception:
                    proj = None
        if proj is None:
            proj = {"board": {"design_settings": {"rules": {}}},
                    "meta": {"filename": os.path.basename(pro),
                             "version": 1},
                    "net_settings": {"classes": [], "meta": {"version": 0}}}
        rules = proj.setdefault('board', {}).setdefault(
            'design_settings', {}).setdefault('rules', {})
        bds = board.GetDesignSettings()
        # The rules the live board DECLARES, not every value pcbnew holds: a
        # project-less board sits at pcbnew's factory values (hole clearance
        # 0.25, edge 0.5, ...), which its file does not declare, and the
        # snapshot is parsed in-process by the oracle legs -- their hole floor
        # (obstacle_map.resolve_hole_clearance) and edge band
        # (effective_board_edge_clearance) read these rules, and would have
        # priced floors the CLI reads nowhere. pcbnew loads a missing rule at
        # that same factory value, so the snapshot's own refill is unchanged.
        from fix_kicad_drc_settings import live_declared_rules, LIVE_RULE_ATTR
        _declared = live_declared_rules(board) or {}
        for key in LIVE_RULE_ATTR:
            if key == 'min_connection':
                continue          # never staged before; its own reader (#1187)
            if key in _declared:
                rules[key] = _declared[key]
            else:
                rules.pop(key, None)
        # Default net class: KiCad 10 accessor first (board.GetNetClasses()
        # returns an EMPTY map there), then the legacy map.
        nc = None
        try:
            nc = bds.m_NetSettings.GetDefaultNetclass()
        except Exception:
            nc = None
        if nc is None:
            try:
                for _name, _c in board.GetNetClasses().items():
                    if _name == 'Default':
                        nc = _c
                        break
            except Exception:
                nc = None
        if nc is not None:
            classes = proj.setdefault('net_settings',
                                      {}).setdefault('classes', [])
            entry = None
            for c in classes:
                if isinstance(c, dict) and c.get('name') == 'Default':
                    entry = c
                    break
            if entry is None:
                entry = {'name': 'Default'}
                classes.append(entry)
            for key, getter in (('clearance', 'GetClearance'),
                                ('track_width', 'GetTrackWidth'),
                                ('via_diameter', 'GetViaDiameter'),
                                ('via_drill', 'GetViaDrill')):
                try:
                    entry[key] = getattr(nc, getter)() / 1e6
                except Exception:
                    pass
        tmp = pro + '.tmp'
        with open(tmp, 'w', encoding='utf-8') as fh:
            json.dump(proj, fh, indent=2)
        os.replace(tmp, pro)
        return pro
    except Exception:
        return None


def read_pin_metadata_from_kicad_pcb(board_filename: str) -> Dict[Tuple[str, str], Tuple[str, str]]:
    """Extract (component_ref, pad_number) → (pinfunction, pintype) from .kicad_pcb.

    kipy.Pad doesn't expose pin_function or pin_type, but the .kicad_pcb
    file stores them as `(pinfunction "...")` / `(pintype "...")` on each
    pad. This reader walks footprint blocks with a regex pass so the
    power-net / high-speed-net / plan-routing skills get real data
    without parsing the whole board file.

    Returns {} on any failure — callers should treat absence as "all pads
    have empty pinfunction/pintype".
    """
    if not board_filename or not os.path.isfile(board_filename):
        return {}
    try:
        with open(board_filename, "r", encoding="utf-8") as f:
            content = f.read()
    except Exception:
        return {}

    result: Dict[Tuple[str, str], Tuple[str, str]] = {}
    # Walk each (footprint ...) block by paren-matching from the opening
    # token, so we don't get confused by nested s-expressions.
    fp_marker = "(footprint "
    pos = 0
    while True:
        start = content.find(fp_marker, pos)
        if start < 0:
            break
        # Find the matching close-paren for this footprint block.
        depth = 0
        end = start
        for i in range(start, len(content)):
            c = content[i]
            if c == "(":
                depth += 1
            elif c == ")":
                depth -= 1
                if depth == 0:
                    end = i + 1
                    break
        block = content[start:end]
        pos = end

        # Reference: (property "Reference" "U1" ...) — newer KiCad, or
        # (fp_text reference "U1" ...) — older.
        ref_match = (re.search(r'\(property\s+"Reference"\s+"([^"]*)"', block)
                     or re.search(r'\(fp_text\s+reference\s+"([^"]*)"', block))
        if not ref_match:
            continue
        ref = ref_match.group(1)

        # Iterate pad blocks inside this footprint. (pad "N" SHAPE ...).
        # Use the same paren walker scoped to the footprint block.
        ppos = 0
        pad_marker = "(pad "
        while True:
            pstart = block.find(pad_marker, ppos)
            if pstart < 0:
                break
            pdepth = 0
            pend = pstart
            for i in range(pstart, len(block)):
                c = block[i]
                if c == "(":
                    pdepth += 1
                elif c == ")":
                    pdepth -= 1
                    if pdepth == 0:
                        pend = i + 1
                        break
            pad_block = block[pstart:pend]
            ppos = pend

            num_m = re.match(r'\(pad\s+"([^"]*)"', pad_block)
            if not num_m:
                continue
            pad_number = num_m.group(1)
            pinfunc_m = re.search(r'\(pinfunction\s+"([^"]*)"\)', pad_block)
            pintype_m = re.search(r'\(pintype\s+"([^"]*)"\)', pad_block)
            pinfunc = pinfunc_m.group(1) if pinfunc_m else ""
            pintype = pintype_m.group(1) if pintype_m else ""
            if pinfunc or pintype:
                result[(ref, pad_number)] = (pinfunc, pintype)

    return result


def read_castellated_pads_from_kicad_pcb(board_filename: str) -> set:
    """Return {(component_ref, pad_number)} for pads marked castellated.

    kipy's Pad proto carries no pad property field at all (id, locked, number,
    net, type, pad_stack, position, copper_clearance_override, pad_to_die_*,
    symbol_pin) -- so `(property pad_prop_castellated)` is invisible over IPC,
    the same wall as via tenting. Read it from the .kicad_pcb instead, exactly
    as read_pin_metadata_from_kicad_pcb does for pinfunction/pintype and as the
    keepout rule-areas are read: a footprint property is not something the user
    edits mid-route, so file-vs-live staleness is not a practical risk here.

    Without this pad.castellated is False for every pad under IPC, and the
    castellated-landing retract (run-6 fix 1.7) silently never fires --
    edge-zone track ends the CLI pulls back would ship as-is.

    Returns an empty set on any failure.
    """
    if not board_filename or not os.path.isfile(board_filename):
        return set()
    try:
        with open(board_filename, "r", encoding="utf-8") as f:
            content = f.read()
    except Exception:
        return set()

    out = set()
    for fp_start, fp_end in _sexpr_blocks(content, "(footprint "):
        block = content[fp_start:fp_end]
        ref_match = (re.search(r'\(property\s+"Reference"\s+"([^"]*)"', block)
                     or re.search(r'\(fp_text\s+reference\s+"([^"]*)"', block))
        if not ref_match:
            continue
        ref = ref_match.group(1)
        for p_start, p_end in _sexpr_blocks(block, "(pad "):
            pad_block = block[p_start:p_end]
            if 'pad_prop_castellated' not in pad_block:
                continue
            num_m = re.match(r'\(pad\s+"([^"]*)"', pad_block)
            if num_m:
                out.add((ref, num_m.group(1)))
    return out


def _sexpr_blocks(text: str, marker: str):
    """Yield (start, end) spans of each top-level `marker` block in *text*.

    Paren-matched from the opening token, so nested s-expressions do not
    confuse it -- the same walk read_pin_metadata_from_kicad_pcb does inline.
    """
    pos = 0
    while True:
        start = text.find(marker, pos)
        if start < 0:
            return
        depth = 0
        end = start
        for i in range(start, len(text)):
            c = text[i]
            if c == "(":
                depth += 1
            elif c == ")":
                depth -= 1
                if depth == 0:
                    end = i + 1
                    break
        if end <= start:
            return
        yield start, end
        pos = end


def read_netclasses_from_kicad_pro(board_filename: str) -> Tuple[Dict[str, Dict[str, float]], List[Tuple[str, str]], Dict[str, str]]:
    """Parse net-class definitions out of the .kicad_pro project JSON.

    kipy.BoardDesignRules exposes board-wide minimums and predefined sizes
    but NOT per-net-class settings (track width, clearance, via diameter,
    etc.) — those live in the sibling .kicad_pro file under
    `net_settings`. This helper reads them so the routing dialog's
    "use net class definitions" flow can pull real values.

    Args:
        board_filename: Absolute path to a .kicad_pcb file (or its
            project's .kicad_pro). If neither is reachable, returns empty
            mappings so callers can degrade gracefully.

    Returns:
        (classes_by_name, patterns, explicit_assignments) where
          - classes_by_name: dict of class_name -> {track_width, clearance,
                                                     via_size, via_drill,
                                                     diff_pair_width,
                                                     diff_pair_gap, ...}
            (values in mm)
          - patterns: list of (glob_pattern, class_name) for
            netclass_patterns assignments (order matters; first match wins)
          - explicit_assignments: dict net_name -> class_name for any
            explicit netclass_assignments entries
    """
    if not board_filename:
        return {}, [], {}
    # .kicad_pcb → .kicad_pro alongside it.
    pcb_path = Path(board_filename)
    if pcb_path.suffix == ".kicad_pro":
        pro_path = pcb_path
    else:
        pro_path = pcb_path.with_suffix(".kicad_pro")
    if not pro_path.is_file():
        return {}, [], {}

    try:
        with open(pro_path, "r", encoding="utf-8") as f:
            data = json.load(f)
    except Exception as e:
        print(f"read_netclasses_from_kicad_pro: failed to read {pro_path}: {e}")
        return {}, [], {}

    net_settings = (data.get("net_settings") or {})
    raw_classes = net_settings.get("classes") or []
    classes_by_name: Dict[str, Dict[str, float]] = {}
    for c in raw_classes:
        name = c.get("name") or ""
        if not name:
            continue
        # All numeric fields the routing code consumes. Keep keys aligned
        # with what _get_netclass_parameters previously returned.
        params: Dict[str, float] = {}
        for src_key, dst_key in (
            ("track_width", "track_width"),
            ("clearance", "clearance"),
            ("via_diameter", "via_size"),
            ("via_drill", "via_drill"),
            ("diff_pair_width", "diff_pair_width"),
            ("diff_pair_gap", "diff_pair_gap"),
            ("microvia_diameter", "microvia_size"),
            ("microvia_drill", "microvia_drill"),
        ):
            v = c.get(src_key)
            if isinstance(v, (int, float)):
                params[dst_key] = float(v)
        classes_by_name[name] = params

    patterns: List[Tuple[str, str]] = []
    for entry in (net_settings.get("netclass_patterns") or []):
        pat = entry.get("pattern") or ""
        cls = entry.get("netclass") or ""
        if pat and cls:
            patterns.append((pat, cls))

    explicit: Dict[str, str] = {}
    raw_assigns = net_settings.get("netclass_assignments")
    if isinstance(raw_assigns, dict):
        for net_name, cls in raw_assigns.items():
            if isinstance(net_name, str) and isinstance(cls, str):
                explicit[net_name] = cls

    return classes_by_name, patterns, explicit


def _resolve_net_to_class(net_name: str, patterns: List[Tuple[str, str]],
                         explicit: Dict[str, str]) -> str:
    """Match a net to its class using KiCad's lookup order.

    1. explicit assignment in `netclass_assignments`
    2. first matching glob in `netclass_patterns`
    3. fallback 'Default'
    """
    if not net_name:
        return "Default"
    if net_name in explicit:
        return explicit[net_name]
    from fnmatch import fnmatchcase
    for pat, cls in patterns:
        try:
            if fnmatchcase(net_name, pat):
                return cls
        except Exception:
            continue
    return "Default"


def build_pcb_data_from_board(board, guide_layer: str = "User.1",
                              keepout_layer: str = "User.2") -> PCBData:
    """Build PCBData directly from a kipy (KiCad IPC) board object.

    Reads from the running KiCad's in-memory board over the IPC socket
    rather than parsing the .kicad_pcb file from disk. Faster than
    parse_kicad_pcb() and reflects any unsaved changes.

    User-layer guide corridors (#7) and keepout polygons (#27) are read live
    from the running board via the kipy graphics API, so unsaved drawings are
    honoured.

    Args:
        board: A kipy.board.Board object (from KiCad().get_board())
        guide_layer: User layer to read guide corridor polylines from (issue #7)
        keepout_layer: User layer to read keepout polygons from (issue #27)

    Returns:
        PCBData object containing all routing-relevant data
    """
    from kicad_ipc_adapter import layer_maps, layer_name_for, ipc_lock
    # All board.* reads share one ipc_lock acquisition so the snapshot is
    # consistent and we don't interleave with a concurrent apply commit.
    with ipc_lock():
        return _build_pcb_data_from_board_impl(
            board, guide_layer, keepout_layer)


def _kipy_pad_polygon_points(pwh):
    """Outer outline of a kipy PolygonWithHoles as global-board-mm vertices.

    Returns a single-polygon list ([[(x, y), ...]]) to match Pad.polygons, or
    None when there is no usable (>=3 point) outline. Holes are ignored: a
    finger/comb pad's channels are concavities in the OUTLINE, not holes, and
    treating any genuine hole as copper only over-blocks (conservative, never
    under-blocks) -- same safety direction as the bbox fallback.
    """
    if pwh is None:
        return None
    outline = getattr(pwh, "outline", None)
    if outline is None:
        return None
    # _polyline_points_mm, not a bare p.x_mm: kipy 0.7.1's Vector2 exposes only
    # integer-nm .x/.y, so the direct read raised AttributeError for EVERY node
    # and the `except: continue` swallowed it -- the outline came back empty and
    # the custom pad silently fell back to its bounding box (#188 copper
    # fidelity lost with no error). That helper probes both spellings.
    pts = _polyline_points_mm(outline)
    if len(pts) < 3:
        return None
    return [pts]


def _attach_custom_pad_polygons(board, custom_pads):
    """Populate Pad.polygons with each custom pad's REAL copper outline (issue
    #188), read from the live board via kipy get_pad_shapes_as_polygons -- the
    IPC analogue of the text parser's _custom_pad_global_polygons and pcbnew's
    GetEffectivePolygon. The kipy renderer returns the true comb/finger copper
    in global board coords for any primitive shape, so the obstacle map / DRC
    rasterize the actual copper instead of the bounding box that boxes in pads
    sitting in the channels.

    Best-effort: on any failure (old kipy without the method, IPC error) the
    bbox model set by _build_pad_from_kipy stays in place -- no regression.
    custom_pads is a list of (pad_obj, kipy_pad). Front-side pads resolve on
    F.Cu; pads that return nothing there are retried on B.Cu.
    """
    try:
        from kipy.board_types import BoardLayer
    except Exception:
        return
    get_polys = getattr(board, "get_pad_shapes_as_polygons", None)
    if get_polys is None:
        return
    pad_objs = [po for po, _ in custom_pads]
    kpads = [kp for _, kp in custom_pads]
    for layer in (BoardLayer.BL_F_Cu, BoardLayer.BL_B_Cu):
        idx = [i for i, po in enumerate(pad_objs) if not po.polygons]
        if not idx:
            break
        try:
            results = get_polys([kpads[i] for i in idx], int(layer))
        except Exception:
            continue
        if results is None:
            continue
        if not isinstance(results, list):
            results = [results]
        for j, i in enumerate(idx):
            pwh = results[j] if j < len(results) else None
            pts = _kipy_pad_polygon_points(pwh)
            if pts:
                pad_objs[i].polygons = pts
                _size_custom_pad_from_copper(pad_objs[i], kpads[i])


def _size_custom_pad_from_copper(pad_obj, kpad):
    """#967: a custom pad whose copper is NOT just its anchor is sized by the
    board-frame box of that copper about the copper centre (global_x/y), with
    no residual tilt -- main's rule on both parse paths. A pad whose copper is
    its anchor (`_kipy_custom_copper_is_anchor`) keeps the anchor rect and its
    tilt, which `_build_pad_from_kipy` already gave it. The box comes from the
    rendered polygon, which sits up to the board's max error inside a curve
    (main's pcbnew builder reads the exact effective-shape box first and this
    polygon only as its fallback; kipy has no effective-shape box)."""
    try:
        primary = kpad.padstack.copper_layers[0]
    except Exception:
        return
    anchor = pad_obj.anchor_size or (pad_obj.size_x, pad_obj.size_y)
    if _kipy_custom_copper_is_anchor(primary, anchor):
        return
    pts = [p for ring in (pad_obj.polygons or []) for p in ring]
    if not pts:
        return
    pad_obj.size_x = 2 * max(abs(x - pad_obj.global_x) for x, _ in pts)
    pad_obj.size_y = 2 * max(abs(y - pad_obj.global_y) for _, y in pts)
    pad_obj.rect_rotation = 0.0
    pad_obj.copper_box = copper_box_if_offset(
        min(x for x, _ in pts) - pad_obj.global_x,
        min(y for _, y in pts) - pad_obj.global_y,
        max(x for x, _ in pts) - pad_obj.global_x,
        max(y for _, y in pts) - pad_obj.global_y)


def _build_pcb_data_from_board_impl(board, guide_layer: str = "User.1",
                                    keepout_layer: str = "User.2") -> PCBData:
    """Inner impl — caller is responsible for holding ipc_lock.

    guide_layer/keepout_layer MUST be threaded in from the public wrapper.
    They used to be read as free variables here, which raised NameError inside
    the best-effort try/except below -- so guide corridors (#7) and keepout
    polygons (#27) were silently never read from the live board, on every run,
    while the wrapper's docstring promised they were.
    """
    from kicad_ipc_adapter import layer_maps, layer_name_for

    name_to_bl, bl_to_name = layer_maps()

    def get_layer_name(bl):
        return bl_to_name.get(bl) or layer_name_for(bl)

    # --- Build copper-layer info ---
    # `get_enabled_layers()` returns the actual stackup (e.g. just F.Cu+B.Cu
    # on a 2-layer board), which is what the router needs. Fall back to
    # F.Cu/B.Cu only if we can't read the stackup at all.
    layers_dict: Dict[int, str] = {}
    copper_layers: List[str] = []
    enabled = None
    try:
        enabled = set(board.get_enabled_layers() or [])
    except Exception as e:
        print(f"Warning: board.get_enabled_layers() failed ({e}); "
              "defaulting to F.Cu/B.Cu")

    for name, bl in name_to_bl.items():
        if not name.endswith(".Cu"):
            continue
        # Only include layers actually enabled in this board's stackup.
        if enabled is not None:
            if bl not in enabled:
                continue
        else:
            if name not in ("F.Cu", "B.Cu"):
                continue
        layers_dict[int(bl) if hasattr(bl, "__int__") else hash(bl)] = name
        if name not in copper_layers:
            copper_layers.append(name)

    # --- Nets ---
    # kipy.Net.code is deprecated in KiCad 10 and returns unreliable values
    # (often the same code for different nets, or 0 for everything). Key
    # everything by `.name` instead — that's the canonical identifier
    # kicad-python guarantees. We synthesise a unique integer per net so
    # the rest of the codebase, which speaks in net_id ints, keeps working.
    nets: Dict[int, Net] = {}
    net_name_to_id: Dict[str, int] = {"": 0}  # reserve 0 for "no net"
    try:
        net_list = list(board.get_nets())
    except Exception as e:
        print(f"Warning: board.get_nets() failed: {e}")
        net_list = []
    for n in net_list:
        name = getattr(n, "name", "") or ""
        if not name:
            continue
        if name in net_name_to_id:
            continue  # duplicate name from kipy — keep the first
        net_id = 1 + len(net_name_to_id)  # 1, 2, 3, ... (0 reserved)
        net_name_to_id[name] = net_id
        nets[net_id] = Net(net_id=net_id, name=name)
    print(f"build_pcb_data_from_board: loaded {len(nets)} nets, "
          f"{len(copper_layers)} copper layers: {copper_layers}")

    def resolve_net(net_obj) -> Tuple[int, str]:
        """Resolve a kipy Net object to (net_id, net_name) by name lookup.

        Returns (0, '') for the no-net / unconnected case (pads on board
        with no electrical connection assigned).
        """
        if net_obj is None:
            return (0, "")
        name = getattr(net_obj, "name", "") or ""
        return (net_name_to_id.get(name, 0), name)

    # --- Drawings (Edge.Cuts outline, copper graphics) ---
    #
    # `get_shapes()` returns every board-level PCB_SHAPE, but no footprint
    # child (it asks for top-level KOT_PCB_SHAPE items). Both halves the text
    # parser models therefore read the footprints' own shapes as well
    # (`_kipy_fp_shapes`): a footprint's Edge.Cuts (#304) and the copper its
    # land pattern draws (#908) -- the latter through `kipy_graphic_copper`,
    # whose pad-count decision is `footprint_copper_is_functional`, the
    # predicate the text parser, main's pcbnew parser and both writers share.
    edge_cuts_bl = name_to_bl.get("Edge.Cuts")
    drawings = []
    try:
        drawings = list(board.get_shapes())
    except AttributeError:
        try:
            drawings = list(board.get_drawings())  # type: ignore[attr-defined]
        except Exception:
            drawings = []

    # The footprint list is read FIRST: a footprint's own Edge.Cuts count
    # toward the bounds and the outline (#304, a board whose whole outline
    # lives in a footprint), as on both of main's parse paths.
    try:
        fp_iter = board.get_footprints()
    except Exception:
        fp_iter = []
    # #726: two footprint BLOCKS can claim one reference, and the ordinal that
    # separates them is a property of the ordered list rather than of any one
    # footprint -- so names are resolved for the WHOLE board first and zipped
    # back on, exactly as the text parser does through `iter_footprint_blocks`.
    # Keying each footprint by its bare reference as this builder used to do is
    # the defect #726 fixed: the second `TP4` overwrote the first in the dict,
    # and the IPC model silently carried one fewer part (thunderscope loses six)
    # with its pads gone from `pads_by_net` -- copper the router would route
    # straight through.
    #
    # `board.get_footprints()` returns KiCad's own footprint list; the two parse
    # paths compute the same ordinal only while it iterates in FILE ORDER, which
    # is an empirical fact about a release rather than a documented contract.
    # `tests/gui_parity/test_726_parse_path_parity.py` is where that is pinned.
    _live_fps = list(fp_iter)
    _raw_refs = kipy_raw_references(_live_fps)
    _dups_live = duplicate_reference_counts(_raw_refs)
    _live_keys = disambiguate_references(_raw_refs)
    _fp_pairs = list(zip(_live_keys, _live_fps))

    # Edge.Cuts: main's pcbnew contour scan, shape for shape
    # (`kipy_board_contours`): lines, rectangles, arcs, circles, beziers and
    # polygons (pcbnew's points, arcs included, zero-length edges skipped --
    # 6febcfad), footprint-drawn Edge.Cuts too, classified into OUTER rings and
    # true cut-outs (#304). The reader this replaces matched only polygons (a
    # kipy shape has no `shape_type`), so a line-drawn outline came back as no
    # outline at all, and it called every ring but the largest a cut-out.
    board_bounds = _compute_edge_cuts_bbox_kipy(drawings, edge_cuts_bl, _fp_pairs)
    board_outlines, board_cutouts = kipy_board_contours(
        drawings, edge_cuts_bl, _fp_pairs)
    board_outline = board_outlines[0] if board_outlines else []

    # Stackup (best-effort via kipy; falls back to parsing the file)
    stackup = _extract_stackup_kipy(board)

    board_info = BoardInfo(
        layers=layers_dict,
        copper_layers=copper_layers,
        board_bounds=board_bounds,
        stackup=stackup,
        board_outline=board_outline,
        board_outlines=board_outlines,
        board_cutouts=board_cutouts
    )

    # --- Footprints and pads ---
    footprints: Dict[str, Footprint] = {}
    pads_by_net: Dict[int, List[Pad]] = {}

    # (pad_obj, kipy_pad) for custom-shaped pads, resolved to their real copper
    # outline in a batch after the loop (issue #188).
    custom_pads: List[Tuple[Pad, object]] = []

    # (#726: the footprints were listed and keyed above, before the outline.)
    for fp, reference in zip(_live_fps, _live_keys):
        fp_name = _fp_library_name(fp)
        fp_x, fp_y = _vec_to_xy_mm(_fp_position(fp))
        fp_rotation = _fp_orientation_deg(fp)
        fp_layer = get_layer_name(getattr(fp, "layer", None))
        fp_value = _fp_value(fp)

        # #1098: (attr ...) flags and the 3D-model declaration, parity with
        # the text parser (FOOTPRINT_ATTR_TOKENS).
        fp_attrs, fp_has_model = kipy_footprint_attrs(fp)
        # DNP (do-not-populate) flag — kept at parity with the text parser. A
        # no-pop series part is an open circuit, so its pads must not be treated
        # as bridging two nets into one logical net. It is one of the kipy
        # attributes: this read used to be pcbnew's IsDNP(), which a kipy
        # footprint does not have, so every part read as populated.
        fp_dnp = 'dnp' in fp_attrs

        # Locked flag — parity with the text parser's footprint (locked yes).
        # Was pcbnew's IsLocked(), likewise absent on kipy, so every part read
        # as unlocked; a kipy FootprintInstance has the `locked` property.
        fp_locked = kipy_locked(fp)

        # Footprint-level clearance override (#326) — IPC parity with the text
        # parser and the SWIG builder's fp.GetLocalClearance(). kipy exposes it
        # on FootprintInstance.proto.overrides.copper_clearance (a Distance in
        # nm); unset = no override. Negative (shrinking) overrides clamp to 0
        # like the text parser (overrides act only as keep-out floors). The
        # parser RESOLVES the inheritance into each pad's local_clearance below,
        # so clearance consumers only ever read pad.local_clearance.
        fp_clearance = 0.0
        try:
            _fpp = getattr(fp, "proto", None)
            if (_fpp is not None and _fpp.HasField("overrides")
                    and _fpp.overrides.HasField("copper_clearance")):
                fp_clearance = max(
                    0.0, _nm_to_mm(_fpp.overrides.copper_clearance.value_nm))
        except Exception:
            fp_clearance = 0.0
        # #962: the footprint's own solder-paste overrides, from the same
        # overrides message (None = not set; pads without their own inherit).
        _fp_paste = kipy_footprint_paste_overrides(fp)
        # Footprint-level zone connection override, parity with the text read.
        fp_zone_connection = kipy_footprint_zone_connection(fp)

        # Net-tie pad groups -- parity with the text parser (Kelvin shunts /
        # net-tie parts; KiCad exempts the grouped pads' mutual clearance),
        # off the kipy definition's `net_ties` (`kipy_net_tie_groups`). This
        # read was pcbnew's IsNetTie()/GetNetTiePads(), which a kipy
        # footprint does not have, so every IPC footprint came back with no
        # group and the clearance waiver was blind to net ties.
        fp_net_tie = kipy_net_tie_groups(fp)

        # Own uuid + sheet path (#459), the kipy mirrors of the text parser's
        # (uuid ...) / (path ...). These read `fp.m_Uuid` / `fp.GetPath()` --
        # the SWIG builder's accessors -- until the IPC port reached them; a
        # kipy FootprintInstance has neither, so both try/excepts fired on
        # EVERY footprint and the IPC model carried no uuid and no sheet path
        # at all. The kipy spellings are `fp.id` (a KIID whose `.value` is the
        # uuid string) and `fp.sheet_path` (a SheetPath whose `.path` is the
        # list of KIIDs the text parser writes as "/a/b"). Still defensive:
        # sheet_path is kipy 0.4.0+, and a missing accessor must not take the
        # whole parse down.
        #
        # The uuid is load-bearing beyond #459 here: a reference-LESS footprint
        # is keyed by it (#726, below), so an empty one would collapse every
        # NPTH drill dot onto a single entry.
        try:
            fp_uuid = str(fp.id.value)
        except Exception:
            fp_uuid = ""
        try:
            _sp = fp.sheet_path.path
            fp_path = ('/' + '/'.join(str(k.value) for k in _sp)) if _sp else ""
        except Exception:
            fp_path = ""

        # #1211: the sheet's name and file, the text parser's
        # `_footprint_sheet_fields` (`kipy_footprint_sheet_fields`).
        fp_sheet_name, fp_sheet_file = kipy_footprint_sheet_fields(fp)

        # Reference-label geometry (#481) -- parity with the text parser's
        # RefLabel, off the kipy Reference field (`kipy_ref_label`). This
        # read used pcbnew's Reference()/GetFPRelativePosition() and an
        # undefined `to_mm`, so it raised on every footprint and the IPC
        # model carried no label at all.
        fp_ref_label = kipy_ref_label(fp, fp_x, fp_y, fp_rotation)

        footprint = Footprint(
            reference=reference,
            footprint_name=fp_name,
            x=fp_x,
            y=fp_y,
            rotation=fp_rotation,
            layer=fp_layer,
            value=fp_value,
            dnp=fp_dnp,
            locked=fp_locked,
            clearance=fp_clearance,
            uuid=fp_uuid,
            sheet_path=fp_path,
            net_tie_groups=fp_net_tie,
            ref_label=fp_ref_label,
            # #962: footprint-level paste overrides, and the pose this model
            # was read at (parity with both other parse paths).
            paste_margin=_fp_paste[0],
            paste_margin_ratio=_fp_paste[1],
            zone_connection=fp_zone_connection,
            parsed_pose=(fp_x, fp_y, fp_rotation, fp_layer),
            attrs=fp_attrs,
            has_model=fp_has_model,
            sheet_name=fp_sheet_name,
            sheet_file=fp_sheet_file,
        )

        for pad in _fp_pads(fp):
            pad_obj = _build_pad_from_kipy(pad, reference, fp_x, fp_y, fp_rotation,
                                            get_layer_name, resolve_net,
                                            copper_layers,
                                            fp_clearance=fp_clearance,
                                            fp_zone_connection=fp_zone_connection)
            if pad_obj is None:
                continue
            footprint.pads.append(pad_obj)
            pads_by_net.setdefault(pad_obj.net_id, []).append(pad_obj)
            if pad_obj.net_id in nets:
                nets[pad_obj.net_id].pads.append(pad_obj)
            if pad_obj.shape == 'custom':
                custom_pads.append((pad_obj, pad))

        footprints[reference] = footprint

    # Replace each custom pad's bbox model with its real copper outline read
    # from the live board (issue #188); best-effort, no-op on older kipy.
    if custom_pads:
        _attach_custom_pad_polygons(board, custom_pads)

    # #829, mirroring both of main's parse paths: which footprints draw
    # Edge.Cuts, and which of those draw the BOARD's boundary rather than a
    # relief that travels with the part (`kipy_footprint_outline_owners`,
    # through the shared `classify_outline_owners`). Anything that moves a
    # footprint gates on `owns_board_outline`; with no stamp here every IPC
    # footprint read False and the guard was disarmed. Its own try, so a
    # failure says so rather than leaving the guard silently off.
    try:
        for _okey, _structural in kipy_footprint_outline_owners(
                drawings, edge_cuts_bl, _fp_pairs).items():
            _ofp = footprints.get(_okey)
            if _ofp is not None:
                _ofp.owns_edge_cuts = True
                _ofp.owns_board_outline = _structural
    except Exception as _exc:                                    # noqa: BLE001
        print(f"  WARNING: could not determine Edge.Cuts ownership from the "
              f"live board ({type(_exc).__name__}: {_exc}); a footprint that "
              f"draws the board outline will NOT be protected from being "
              f"moved (#829)")

    # --- Segments (tracks) ---
    segments: List[Segment] = []
    try:
        track_iter = board.get_tracks()
    except Exception:
        track_iter = []
    for track in track_iter:
        try:
            start_x, start_y = _vec_to_xy_mm(track.start)
            end_x, end_y = _vec_to_xy_mm(track.end)
            net_id, _ = resolve_net(getattr(track, "net", None))
            width = _nm_to_mm(getattr(track, "width", 0))
            layer = get_layer_name(getattr(track, "layer", None))
            # #521: user-PINNED copper. Every arc segment inherits the arc's
            # own flag -- a locked arc is locked as a whole, and the pieces
            # are this parser's linearization, not the board's objects.
            seg_locked = kipy_locked(track)
            # The file's (uuid ...) on every piece, as on both of main's parse
            # paths: a `sense:` group's members resolve against it (#1279).
            seg_uuid = _kipy_uuid(track)
            # Arc tracks: kipy models them as a distinct ArcTrack carrying a
            # `.mid` point (a straight Track has none). Linearize the bulge with
            # the SAME helper the text parser uses (three defining points), so
            # the GUI/IPC obstacle + connectivity model sees the real arc-routed
            # copper instead of just its chord -- skipping it left the model
            # blind to arc-routed tracks (thunderscope: ~19.5k file-only
            # segments in validate-board). Parity with the pcbnew builder.
            mid = getattr(track, "mid", None)
            if mid is not None:
                mid_x, mid_y = _vec_to_xy_mm(mid)
                for p0, p1 in _arc_to_segments((start_x, start_y),
                                               (mid_x, mid_y), (end_x, end_y)):
                    segments.append(Segment(
                        start_x=p0[0], start_y=p0[1], end_x=p1[0], end_y=p1[1],
                        width=width, layer=layer, net_id=net_id,
                        uuid=seg_uuid, locked=seg_locked))
            else:
                segments.append(Segment(
                    start_x=start_x, start_y=start_y,
                    end_x=end_x, end_y=end_y,
                    width=width, layer=layer, net_id=net_id,
                    uuid=seg_uuid, locked=seg_locked))
        except Exception:
            continue

    # --- Vias ---
    vias: List[Via] = []
    # #489 s8 / #741: the protection spec. Read off each LIVE via when kipy
    # can see every token (kicad-python 0.7+); only an older kipy needs the
    # open document's own file, read ONCE here and handed out by uuid (see
    # kipy_via_protection_attrs for why a PARTIAL live read would be worse
    # than none). `_ipc_board_path()` is the resolver PCBData.source_path
    # uses, so an unsaved board yields {} and every via reads as inheriting.
    _via_specs = ({} if _kipy_protection_enums() is not None
                  else via_protection_attrs_from_path(_ipc_board_path()))
    try:
        via_iter = board.get_vias()
    except Exception:
        via_iter = []
    for v in via_iter:
        try:
            x, y = _vec_to_xy_mm(v.position)
            # #1268: the via's TYPE, and the layer pair read from it -- a via
            # spans part of the stack only when KiCad types it blind, buried or
            # micro; a through via is F.Cu/B.Cu whatever its drill says
            # (db60fe77), and a partial pair reads top layer first (6552e70f).
            # The span lives on padstack.drill (`kipy_via_layers`).
            v_type = kipy_via_type(v)
            net_id, _ = resolve_net(getattr(v, "net", None))
            vias.append(Via(
                x=x, y=y,
                size=_nm_to_mm(getattr(v, "diameter", 0)),
                drill=_nm_to_mm(getattr(v, "drill_diameter", 0)),
                layers=kipy_via_layers(v, v_type),
                net_id=net_id,
                via_type=v_type,
                # The file's (uuid ...): a `sense:` group member (#1279).
                uuid=_kipy_uuid(v),
                # Parity with BOTH of main's parse paths, which fill these and
                # which this builder did not: without tenting_attrs a via the
                # plugin RE-PLACES (the #313 cap nudge, a rip-up, a tap
                # relocation) loses its own protection, and without locked a
                # user-pinned via's net was rip-eligible in defiance of #521.
                tenting_attrs=kipy_via_protection_attrs(v, _via_specs),
                locked=kipy_locked(v),
            ))
        except Exception:
            continue

    # --- Copper GRAPHICS (#337) and footprint copper (#908) ---
    # After the tracks, as main's builder orders them (`kipy_graphic_copper`).
    # A fresh build per dialog step is how a footprint moved in the session
    # carries its copper art (main 7b0f15d0 re-reads it in the SWIG re-sync).
    segments.extend(kipy_graphic_copper(drawings, _fp_pairs, resolve_net))

    # Parity with parse_kicad_pcb: the same net-tagged graphics notice.
    warn_net_tagged_graphics(segments,
                             {n.name: nid for nid, n in nets.items()})

    # --- Zones ---
    # Copper only, footprint zones included (`_extract_zones_kipy`): teardrop
    # areas flagged (#1164), holes (#1279), arc outlines (fe367f3f), the
    # closing repeat merged (37fb1b93), mask/paste zones and rule areas out
    # (da63ad5c).
    zones = _extract_zones_kipy(board, get_layer_name, resolve_net, _fp_pairs)

    # IPC plugins only run on KiCad 10+, so unconditionally flag this as
    # K10-format data. route_planes / output_writer / route_disconnected_planes
    # gate their "use net names instead of integer net codes" branches on
    # this; without it they fall back to writing integer codes (our
    # synthetic ones, which never match what KiCad expects).
    net_id_to_name = {nid: n.name for nid, n in nets.items()}

    # Net-class settings live in the sibling .kicad_pro file (kipy doesn't
    # expose per-class values). Best-effort; an empty result is fine.
    netclass_params: Dict[str, Dict[str, float]] = {}
    net_to_class: Dict[str, str] = {}
    # #962 file reads (below); KiCad's defaults when there is no file.
    board_info.via_protection_setup = dict(VIA_PROTECTION_SETUP_DEFAULTS)
    _paste_graphics962: List[dict] = []
    _unmeasured962: List[dict] = []
    try:
        from kicad_ipc_adapter import get_board_full_path
        board_path = get_board_full_path()
    except Exception:
        board_path = None
    if board_path:
        try:
            classes_by_name, patterns, explicit = read_netclasses_from_kicad_pro(board_path)
            netclass_params = classes_by_name
            for n in nets.values():
                if n.name:
                    net_to_class[n.name] = _resolve_net_to_class(
                        n.name, patterns, explicit)
        except Exception as e:
            print(f"build_pcb_data_from_board: netclass read failed: {e}")

        # kipy.Pad doesn't expose pin_function / pin_type, so pull them
        # from the .kicad_pcb file. Power-net detection and the
        # high-speed-net / plan-routing skills rely on these.
        try:
            pin_meta = read_pin_metadata_from_kicad_pcb(board_path)
        except Exception as e:
            print(f"build_pcb_data_from_board: pin metadata read failed: {e}")
            pin_meta = {}
        if pin_meta:
            for ref, fp in footprints.items():
                for pad_obj in fp.pads:
                    meta = pin_meta.get((ref, pad_obj.pad_number))
                    if meta:
                        pad_obj.pinfunction, pad_obj.pintype = meta

        # Same story for (property pad_prop_castellated): kipy's Pad proto has
        # no pad-property field, so read it from the file or the castellated-
        # landing retract can never fire on the live board (run-6 fix 1.7).
        try:
            castellated = read_castellated_pads_from_kicad_pcb(board_path)
        except Exception as e:
            print(f"build_pcb_data_from_board: castellated read failed: {e}")
            castellated = set()
        if castellated:
            for ref, fp in footprints.items():
                for pad_obj in fp.pads:
                    if (ref, pad_obj.pad_number) in castellated:
                        pad_obj.castellated = True

        # #962: the board's paste margin and via-protection policy, the paste
        # GRAPHICS, and the footprint copper the parser does not model. kipy's
        # BoardDesignRules carries neither setup value and its shape listing
        # omits footprint children, so all three come from the file, like the
        # pin metadata above -- the graphics posed at each part's LIVE pose
        # (kipy_paste_graphics), since a move since the save must not strand
        # a paste tab where the part used to be.
        try:
            with open(board_path, encoding='utf-8') as _fh962:
                _content962 = _fh962.read()
            (board_info.pad_to_paste_clearance,
             board_info.pad_to_paste_clearance_ratio,
             board_info.via_protection_setup) = \
                extract_board_setup_paste_and_protection(_content962)
            _paste_graphics962, _unposed962 = kipy_paste_graphics(
                _content962, footprints)
            if _unposed962:
                print(f"build_pcb_data_from_board: paste graphics of "
                      f"{len(_unposed962)} part(s) not modelled -- flipped or "
                      f"replaced since the board was saved: "
                      f"{', '.join(_unposed962[:6])}"
                      f"{' ...' if len(_unposed962) > 6 else ''} (#962)")
            _unmeasured962 = extract_unmodelled_footprint_copper(_content962)
        except Exception as e:
            print(f"build_pcb_data_from_board: paste/via-protection setup read "
                  f"failed ({e}); using KiCad's defaults (#962)")
            board_info.via_protection_setup = dict(VIA_PROTECTION_SETUP_DEFAULTS)

    # --- User-layer guide corridors (#7) and keepout polygons (#27) ---
    # Read live from the running board over IPC (reflects unsaved edits), via
    # the kipy graphics API. Best-effort; empty on any failure.
    guide_paths = []
    keepout_zones = []
    try:
        from kicad_ipc_adapter import read_guide_paths, read_keepout_zones
        guide_paths = read_guide_paths(board, guide_layer)
        keepout_zones = read_keepout_zones(board, keepout_layer)
    except Exception as e:
        print(f"build_pcb_data_from_board: guide/keepout read failed: {e}")

    # --- Extract KiCad rule-area keep-outs (issue #95) ---
    # board_info.keepouts (the (keepout ...) rule areas the router must avoid)
    # isn't exposed cleanly via the IPC zone API, so read them from the board
    # file content exactly as the text parser does (parse_kicad_pcb), keeping the
    # IPC path at parity with the CLI. The kipy board only knows its basename, so
    # resolve the on-disk path via the adapter. Best-effort: the saved file is the
    # last-saved state, which is fine for rule areas (design-time features, not
    # edited mid-route). Without this the router lays copper through rule areas.
    try:
        from kicad_ipc_adapter import get_board_full_path
        board_filename = get_board_full_path()
        if board_filename:
            with open(board_filename, 'r', encoding='utf-8') as f:
                board_info.keepouts = extract_keepouts(f.read())
    except Exception:
        pass

    # --- KiCad (group ...) blocks (#459) ---
    # Read from the board FILE rather than board.Groups(), for the same reason
    # rule areas are above: it keeps this path bit-identical to parse_kicad_pcb's
    # member resolution instead of maintaining a second uuid-walk against a SWIG
    # API whose accessors move between KiCad versions. Best-effort -- an unsaved
    # or missing file simply yields no groups, which is also the corpus norm.
    # The path through `_ipc_board_path()`: this read called pcbnew's
    # board.GetFileName(), which a kipy board does not have, so it raised and
    # every IPC board came back with no groups.
    groups = {}
    try:
        _bf = _ipc_board_path()
        if _bf:
            with open(_bf, 'r', encoding='utf-8') as f:
                groups = extract_groups(f.read(), footprints)
    except Exception:
        groups = {}

    # Drop Edge.Cuts contours mis-classified as cutouts (they enclose pads).
    drop_pad_containing_cutouts(board_info, pads_by_net)

    # #424: exact-fill consumers must price the LIVE board's fill, not the
    # possibly-stale file at source_path (mid-plan, steps apply copper to the
    # live board first).
    #
    # Via a TEMP SAVE, not an in-process refill of the board object. An
    # in-process ZONE_FILLER sees the live COPPER but NOT the live
    # CLEARANCES: pcbnew resolves zone clearance through project state fixed
    # when the board was loaded, so a netclass the run itself lowered -- and
    # the DRC write-back lowers Default to the routed floor on every step --
    # is invisible to it. Measured directly: SetClearance(0.2 -> 0.15) leaves
    # the fill bit-identical (6702 verts / 5288.29mm^2), and NET_SETTINGS.
    # ClearAllCaches() does not change that; only a save+reload picks it up
    # (6748 / 5332.01, exactly what the CLI's staged refill produces).
    #
    # That gap is not cosmetic. The fill feeds the plane-fragility cost
    # field, so the GUI priced the A* off a pour filled at the PRE-routing
    # clearance while the CLI used the routed one -- 795 divergent field
    # cells on splitflap_driver, and different GND copper on boards whose
    # copper, zones, pads and project rules were otherwise identical.
    # Saving first costs a board write per fill (refill_islands memoizes on
    # the board+project bytes) and removes BOTH staleness modes: the temp
    # file carries the live copper AND re-resolves the live settings.
    # Memo (review DRC-4): a fresh mkstemp path every call defeated
    # refill_islands' own (path, mtime, size) memo, so EVERY consumer of the
    # provider (fragility registration, each oracle round) paid SaveBoard +
    # a full pcbnew-subprocess refill. SaveBoard's serialization of an
    # unchanged board is byte-stable, so memoize here on the saved BYTES:
    # unchanged board -> hash hit -> cached islands, changed board -> miss.
    _live_fill_memo = {}

    def _warn_live_fill_fallback(why):
        """LOUD fallback (#713 item 4). There is no in-process fallback filler
        under IPC (no pcbnew in the plugin process), so a provider that yields
        nothing sends every exact-fill consumer to the FILE at source_path --
        which mid-plan is missing the live board's copper. The operator must
        know which fill truth they got.

        ONE function, TWO callers, because the two ways of arriving here used
        to be one loud and one silent: the warning fired only from `except`,
        so a refill that RETURNED nothing -- a timeout above all -- fell
        through to `return None` without a word, on the routing worker thread
        where nobody is watching.

        "did not yield a fill" rather than "failed", because one of the ways
        to arrive is a refill that SUCCEEDED and poured nothing.
        """
        print(f"WARNING: live-board staged refill did not yield a fill "
              f"({why}); exact-fill consumers will fall back to the last "
              f"SAVED file, which mid-plan is missing copper this session "
              f"already applied.")

    def _live_fill(_board=board, _memo=_live_fill_memo):
        import copy as _copy
        import hashlib
        import tempfile
        from kicad_exact_fill import refill_islands_ex
        tmp = None
        # #831: the provider's own account of WHICH fill it returned, read by
        # plane_fragility (`provider.last_status`) so a fallback stops being
        # reported as "the LIVE board" and a MACHINE-DEPENDENT one (the refill
        # timed out) reaches the JSON summary. Published in the `finally`, so
        # every return path leaves an account. A function attribute on this
        # closure, not a module global: it lives exactly as long as this
        # PCBData.
        #
        # IPC note on `source`: there is no in-process filler on this front,
        # so the fallback is not 'live_board_in_process' as on SWIG -- it is
        # the last SAVED FILE, which is a different and WORSE fidelity (it can
        # be missing copper this session already applied). Naming it honestly
        # matters more than matching the other front's vocabulary.
        _acct = {'source': 'saved_file', 'refill_status': None,
                 'why': 'no account recorded', 'machine_dependent': False}
        try:
            from kicad_ipc_adapter import (save_board_snapshot,
                                           get_board_full_path)
            fd, tmp = tempfile.mkstemp(suffix='.kicad_pcb')
            os.close(fd)
            # IPC twin of the SWIG path's SaveBoard(aSkipSettings=True):
            # save_as(include_project=False) writes copper only and never
            # touches the user's open document. The sibling .kicad_pro is
            # what makes the refill run at the session's rules rather than
            # stock ones (#627), and the .kicad_dru carries the per-layer
            # clearance rules (#498) -- kipy cannot author either from live
            # design settings, so stage the REAL project's files beside the
            # snapshot. A stale sibling from an earlier snapshot of a
            # different project must be removed, not inherited.
            save_board_snapshot(tmp)
            import shutil
            # #711: the ONE canonical sibling list, not a hand-written tuple.
            # This site used to spell the project and rules extensions itself,
            # which is exactly how a newly-added sibling type (the design
            # brief) comes to be carried everywhere except here. The gate that
            # enforces this is a TEXT scan, so do not quote such a pair even
            # in a comment -- it cannot tell prose from code.
            from copy_board import SIBLING_EXTS
            _src = get_board_full_path()
            _pro = None
            for _ext in SIBLING_EXTS:
                _sib = (os.path.splitext(_src)[0] + _ext) if _src else None
                _dst = os.path.splitext(tmp)[0] + _ext
                try:
                    if _sib and os.path.isfile(_sib):
                        shutil.copy(_sib, _dst)
                        if _ext == '.kicad_pro':
                            _pro = _dst
                    elif os.path.isfile(_dst):
                        os.remove(_dst)
                except OSError:
                    pass
            _h = hashlib.sha256()
            with open(tmp, 'rb') as _fh:
                _h.update(_fh.read())
            if _pro:
                # The fill depends on the staged rules too: a step boundary
                # can change the floors on an unchanged board, and the memo
                # must miss then.
                with open(_pro, 'rb') as _fh:
                    _h.update(_fh.read())
            _key = _h.digest()
            if _key in _memo:
                # Deep copy on hit: a caller mutating its polygons must not
                # poison later consumers (same contract as refill_islands'
                # own memo).
                _acct = {'source': 'live_board', 'refill_status': 'ok',
                         'why': 'memo hit', 'machine_dependent': False}
                return _copy.deepcopy(_memo[_key])
            islands, _st = refill_islands_ex(tmp)
            if islands:
                _memo.clear()          # one board state at a time
                _memo[_key] = _copy.deepcopy(islands)
                _acct = {'source': 'live_board', 'refill_status': _st.reason,
                         'why': _st.why(), 'machine_dependent': False}
                return islands
            # #713 item 4: the quiet arrival. A refill that returned NOTHING
            # is just as much a fallback as one that raised, and it is the
            # one reached by a timeout.
            _warn_live_fill_fallback(_st.why())
            _acct = {'source': 'saved_file', 'refill_status': _st.reason,
                     'why': _st.why(), 'machine_dependent': _st.is_timeout}
        except Exception as _e:
            _warn_live_fill_fallback(f"{type(_e).__name__}: {_e}")
            _acct = {'source': 'saved_file', 'refill_status': 'error',
                     'why': f"{type(_e).__name__}: {_e}",
                     'machine_dependent': False}
        finally:
            _live_fill.last_status = _acct
            if tmp:
                # Same canonical list the staging above copies from (#711),
                # so a newly-added sibling type is cleaned up, not leaked.
                from copy_board import SIBLING_EXTS as _SX
                for _p in ((tmp,) + tuple(os.path.splitext(tmp)[0] + _e
                                          for _e in _SX)):
                    try:
                        os.unlink(_p)
                    except OSError:
                        pass
        return None

    pcb = PCBData(
        board_info=board_info,
        nets=nets,
        footprints=footprints,
        vias=vias,
        segments=segments,
        pads_by_net=pads_by_net,
        zones=zones,
        kicad_version=KICAD_10_MIN_VERSION,
        net_id_to_name=net_id_to_name,
        netclass_params=netclass_params,
        net_to_class=net_to_class,
        guide_paths=guide_paths,
        keepout_zones=keepout_zones,
        groups=groups,
        # #726: how many blocks claim each duplicated reference, spelled the
        # way the FILE spells it, so a consumer can report the board's own
        # name back to a human ("TP4 x2").
        duplicate_references=_dups_live,
        # #498 parity with parse_kicad_pcb: sibling-file discovery
        # (.kicad_dru). kipy has no GetFileName(); the adapter resolves the
        # open document path.
        source_path=_ipc_board_path(),
        # #424: exact-fill consumers read the LIVE board, never a stale file
        exact_fill_provider=_live_fill,
        # #962: the paste stencil from the shared builder -- pad openings from
        # the live pads, graphics posed at the live footprints.
        paste_apertures=build_paste_apertures(footprints, board_info,
                                              _paste_graphics962),
        graphic_copper_unmeasured=_unmeasured962,
        # 250f5183's `live_records_provider` (the kicad_routing_tools records
        # the SWIG board holds as a top-level property) stays None: kipy can
        # neither read nor write a board property, so on this front the
        # records live in the project only. The live-rules provider (#1187,
        # #1038 declared floors) likewise has no kipy source; readers fall
        # back to the file, as for a board pcbnew has no settings for.
    )
    # #1279 parity with parse_kicad_pcb and main's pcbnew builder: the LIVE
    # board's `sense:<REF>.<PAD>` groups (a group drawn in this session counts
    # before it is saved), resolved by the same function against the same
    # uuids. Every fresh build re-resolves them, which is how the IPC dialog's
    # re-sync (a fresh PCBData per step) re-reads a group edited mid-session.
    try:
        from sense_branch import resolve_sense_branches
        pcb.sense_branches = resolve_sense_branches(pcb, kipy_sense_groups(board))
    except Exception as _e1279:                                 # noqa: BLE001
        print(f"  WARNING: could not resolve the board's sense: groups "
              f"({type(_e1279).__name__}: {_e1279}); sense branches (#1279) "
              f"are not protected")
        pcb.sense_branches = []
    return pcb


# --- kipy helpers used by build_pcb_data_from_board ----------------------

def _ipc_board_path() -> str:
    """Absolute path of the open document, or "" when it cannot be resolved.

    IPC twin of the SWIG builder's ``board.GetFileName()``: kipy's Board has
    no such accessor, so the adapter resolves it from the open document.
    Feeds PCBData.source_path, which is how sibling-file discovery finds the
    board's .kicad_dru (#498).
    """
    try:
        from kicad_ipc_adapter import get_board_full_path
        p = get_board_full_path()
        return os.path.abspath(p) if p else ""
    except Exception:
        return ""


def _nm_to_mm(value) -> float:
    """Convert a nanometre integer (kipy's internal unit) to millimetres."""
    if value is None:
        return 0.0
    try:
        return float(value) / 1_000_000.0
    except (TypeError, ValueError):
        return 0.0


def _vec_to_xy_mm(vec) -> Tuple[float, float]:
    """Read a kipy Vector2 as (x_mm, y_mm), tolerating nm-only fallbacks."""
    if vec is None:
        return (0.0, 0.0)
    x_mm = getattr(vec, "x_mm", None)
    y_mm = getattr(vec, "y_mm", None)
    if x_mm is not None and y_mm is not None:
        return (float(x_mm), float(y_mm))
    return (_nm_to_mm(getattr(vec, "x", 0)), _nm_to_mm(getattr(vec, "y", 0)))


# NOTE: build_pcb_data_from_board threads a `resolve_net(net) -> (id, name)`
# closure into every helper that needs to translate kipy Net objects into
# our synthetic-id space. kipy.Net.code is deprecated and unreliable in
# KiCad 10 — relying on it silently sends every read to net_id=0.
# Callers MUST always go through `resolve_net`; the helper above exists
# only as a fail-loud guard against future regressions.

def _net_code(_net) -> int:  # pragma: no cover - deliberate footgun guard
    raise RuntimeError(
        "_net_code() was called — this used to rely on kipy.Net.code "
        "which is deprecated in KiCad 10. Use the resolve_net closure "
        "from build_pcb_data_from_board (resolves by net name) instead."
    )


def _fp_reference(fp) -> str:
    """Extract the footprint's reference designator (e.g. 'U1', 'C2').

    kipy chains the reference through three wrappers:
        fp.reference_field            -> Field
        Field.text                    -> BoardText
        BoardText.value               -> str
    Older releases sometimes flatten this. Probe defensively, returning
    the first plain string we find by descending into .value / .text.
    """
    candidates = (
        getattr(fp, "reference", None),
        getattr(fp, "reference_field", None),
    )
    for v in candidates:
        for _ in range(4):  # cap recursion depth
            if v is None:
                break
            if isinstance(v, str):
                return v
            # Prefer .value (BoardText / Field text); fall back to .text
            # (Field wrapping a BoardText).
            nxt = getattr(v, "value", None)
            if nxt is None:
                nxt = getattr(v, "text", None)
            if nxt is v:  # avoid infinite loop if a wrapper is self-referential
                break
            v = nxt
    return ""


def kipy_locked(item) -> bool:
    """Is this kipy track / arc / via user-PINNED (`(locked yes)`)?

    #521's contract is that KiCad-locked copper makes its net never-rippable,
    with NO override, and both of main's parse paths fill `locked`. The IPC
    builder filled it for FOOTPRINTS only, so on this front every locked track
    and via read as unlocked and its net was rip-eligible like any other --
    a promise the tool makes in CLAUDE.md and did not keep here.

    kipy exposes `.locked` as a property on `Via` but NOT on `Track` /
    `ArcTrack`, even though every one of those protos carries the field. So
    prefer the property where it exists and fall back to the proto, rather
    than reading the proto for everything: a future kipy that adds the
    property should be read through it, and a future kipy that drops the
    field should make this return False rather than raise mid-parse.
    """
    v = getattr(item, "locked", None)
    if isinstance(v, bool):
        return v
    try:
        from kipy.proto.common.types import LockedState
        return item.proto.locked == LockedState.LS_LOCKED
    except Exception:
        return False


#: #1098 over IPC: (attr ...) token -> kipy FootprintAttributes field. kipy
#: 0.7.1 wraps the first four as properties; the last two exist only on the
#: proto, which carries all of them, so each is read property first, proto
#: second (as kipy_locked reads `locked`).
_KIPY_FP_ATTR_FIELDS = (
    ('board_only', 'not_in_schematic'),
    ('exclude_from_bom', 'exclude_from_bill_of_materials'),
    ('exclude_from_pos_files', 'exclude_from_position_files'),
    ('dnp', 'do_not_populate'),
    ('allow_missing_courtyard', 'exempt_from_courtyard_requirement'),
    ('allow_soldermask_bridges', 'allow_soldermask_bridges'),
)


def kipy_footprint_attrs(fp) -> Tuple[Tuple[str, ...], bool]:
    """#1098 over IPC: a kipy footprint's ``(attr ...)`` flags, spelled as
    FOOTPRINT_ATTR_TOKENS, and whether it declares a 3D model -- the two
    Footprint fields both of main's parse paths fill.

    kipy footprints have none of pcbnew's GetAttributes() / IsDNP() /
    Models(): the flags are FootprintAttributes fields, smd / through_hole is
    its mounting_style, and the models are footprint items. A flag kipy
    cannot read is left unset rather than raising mid-parse."""
    toks = set()
    a = getattr(fp, 'attributes', None)
    if a is not None:
        proto = getattr(a, 'proto', None)
        for tok, field in _KIPY_FP_ATTR_FIELDS:
            v = getattr(a, field, None)
            if not isinstance(v, bool) and proto is not None:
                v = getattr(proto, field, None)
            if v is True:
                toks.add(tok)
        try:
            from kipy.proto.board.board_types_pb2 import FMS_SMD, FMS_THROUGH_HOLE
        except Exception:
            FMS_THROUGH_HOLE, FMS_SMD = 1, 2   # the proto's own enum values
        style = getattr(a, 'mounting_style', None)
        if style == FMS_SMD:
            toks.add('smd')
        elif style == FMS_THROUGH_HOLE:
            toks.add('through_hole')
    try:
        has_model = len(fp.definition.models) > 0
    except Exception:
        has_model = False
    return tuple(sorted(toks)), has_model


#: The per-side protection tokens on a kipy padstack's front/back outer layers:
#: token -> (field, yes enum, no enum). `tenting` IS the solder mask:
#: SMM_MASKED on the front writes `(tenting (front yes) ...)`. Probed against
#: KiCad 10.0.0 over IPC, as are the rest of these mappings.
_KIPY_SIDE_PROTECTION = (
    ('tenting', 'solder_mask_mode', 'SMM_MASKED', 'SMM_UNMASKED'),
    ('covering', 'covering_mode', 'VCM_COVERED', 'VCM_UNCOVERED'),
    ('plugging', 'plugging_mode', 'VPM_PLUGGED', 'VPM_UNPLUGGED'),
)
#: The whole-via tokens, on the padstack's drill: token -> (field, yes, no).
_KIPY_DRILL_PROTECTION = (
    ('capping', 'capped', 'VDCM_CAPPED', 'VDCM_UNCAPPED'),
    ('filling', 'filled', 'VDFM_FILLED', 'VDFM_UNFILLED'),
)
_SIDE_VALUE_RE = re.compile(r'\((front|back)\s+(yes|no|none)\s*\)')


def _kipy_protection_enums():
    """kipy's board_types_pb2 when it carries every protection enum this
    reader and writer use (kicad-python 0.7+), else None. Older kipy exposes
    only the solder mask, and a PARTIAL spec is worse than none: handed back on
    a re-placed via it would drop four tokens and turn an inheriting via into
    an override."""
    try:
        from kipy.proto.board import board_types_pb2 as bt
        for _t, _f, yes, no in _KIPY_SIDE_PROTECTION + _KIPY_DRILL_PROTECTION:
            getattr(bt, yes)
            getattr(bt, no)
        return bt
    except Exception:
        return None


def kipy_via_protection_live(via) -> Optional[Dict[str, str]]:
    """A kipy Via's protection spec `{token: raw inner s-expr}`, read off the
    live object, in exactly the form the text parser gives for the file KiCad
    writes -- or None when this kipy cannot see every token.

    KiCad reports a mode the via does not override as UNKNOWN (FROM_DESIGN_RULES
    comes back as UNKNOWN too), which is the absence of a token in the file. A
    per-side token with one side inheriting is written `(front none)` (probed),
    and so read here.
    """
    bt = _kipy_protection_enums()
    if bt is None:
        return None
    try:
        ps = via.padstack.proto
    except Exception:
        return None
    out: Dict[str, str] = {}
    for token, field, yes, no in _KIPY_SIDE_PROTECTION:
        sides = []
        for name, layer in (('front', ps.front_outer_layers),
                            ('back', ps.back_outer_layers)):
            val = getattr(layer, field)
            sides.append((name, 'yes' if val == getattr(bt, yes)
                          else 'no' if val == getattr(bt, no) else None))
        if any(v is not None for _n, v in sides):
            out[token] = ' '.join(f'({n} {v or "none"})' for n, v in sides)
    for token, field, yes, no in _KIPY_DRILL_PROTECTION:
        val = getattr(ps.drill, field)
        if val == getattr(bt, yes):
            out[token] = 'yes'
        elif val == getattr(bt, no):
            out[token] = 'no'
    return out


def kipy_set_via_protection(via, spec) -> List[str]:
    """Write a protection spec onto a kipy Via's padstack; returns the tokens
    it could NOT write ([] = the whole spec is on the via).

    The inverse of `kipy_via_protection_live`. A token the spec leaves out,
    and a side it gives as `none`, stay UNKNOWN, which KiCad reads as
    inheriting the board's `(setup ...)` -- an empty spec therefore writes
    nothing at all, as `kicad_writer.via_protection_sexpr` emits nothing.
    """
    if not spec:
        return []
    bt = _kipy_protection_enums()
    if bt is None:
        return sorted(spec)
    try:
        ps = via.padstack.proto
    except Exception:
        return sorted(spec)
    side = {t: (f, y, n) for t, f, y, n in _KIPY_SIDE_PROTECTION}
    drill = {t: (f, y, n) for t, f, y, n in _KIPY_DRILL_PROTECTION}
    unwritten = []
    for token, inner in spec.items():
        try:
            if token in side:
                field, yes, no = side[token]
                pairs = dict(_SIDE_VALUE_RE.findall(inner or ''))
                if not pairs:
                    unwritten.append(token)
                    continue
                for name, layer in (('front', ps.front_outer_layers),
                                    ('back', ps.back_outer_layers)):
                    v = pairs.get(name)
                    if v in ('yes', 'no'):
                        setattr(layer, field, getattr(bt, yes if v == 'yes' else no))
            elif token in drill:
                field, yes, no = drill[token]
                v = ' '.join(str(inner).split())
                if v in ('yes', 'no'):
                    setattr(ps.drill, field, getattr(bt, yes if v == 'yes' else no))
                else:
                    unwritten.append(token)
            else:
                unwritten.append(token)
        except Exception:
            unwritten.append(token)
    return sorted(set(unwritten))


def kipy_via_protection_attrs(via, text_specs: Dict[str, Dict[str, str]]
                              ) -> Dict[str, str]:
    """A kipy Via's protection spec `{token: raw inner s-expr}`.

    The IPC twin of `pcbnew_via_protection_attrs`. Read off the LIVE via
    (`kipy_via_protection_live`) whenever kipy can see every token, which is
    what the running board says rather than what was last saved. On an older
    kipy it falls back to the board's own FILE: `text_specs` comes from
    `via_protection_attrs_from_path`, keyed by via uuid, and carries the caveat
    #751 documents -- the file can LAG the live board mid-plan, which degrades
    in the SAFE direction (a via added in-session reads as inheriting).
    """
    live = kipy_via_protection_live(via)
    if live is not None:
        return live
    if not text_specs:
        return {}
    try:
        uid = str(via.id.value)
    except Exception:
        return {}
    return dict(text_specs.get(uid, {}))


def kipy_raw_references(live_fps) -> List[str]:
    """Ordered RAW references of a kipy footprint list, before #726 ordinals.

    The kipy twin of `footprint_raw_reference`, which reads the same two facts
    off one footprint BLOCK of text. Extracted rather than left inline in
    `build_pcb_data_from_board` (where the SWIG builder keeps its copy) for one
    reason: the IPC read path cannot be driven without a running KiCad, so an
    inline pre-pass is code no gate can reach. As a function, the keying that
    #726 turns on is gradable from a stub list --
    `tests/test_726_kipy_reference_keys.py` feeds it a board's own footprint
    order and holds the result to what `iter_footprint_blocks` names the same
    board's blocks.

    A reference-LESS footprint (a locked NPTH drill dot; thunderscope has 86)
    is keyed by its uuid, exactly as the text parser keys it, so a board full
    of them does not collapse onto one '' entry. `"?"` is the last resort and
    is deliberately NOT unique: it means "this footprint could not say what it
    is called", and `disambiguate_references` then separates the bearers by
    ordinal like any other repeated name.
    """
    out: List[str] = []
    for fp in live_fps:
        r = _fp_reference(fp)
        if not r:
            try:
                uid = str(fp.id.value)
            except Exception:
                uid = ""
            r = ("#" + uid) if uid else "?"
        out.append(r)
    return out


def kipy_unconnected_layer_mode(padstack) -> str:
    """Pad.unconnected_layer_mode for a kipy padstack (#1063).

    The IPC twin of `unconnected_layer_mode_from_pcbnew`, read off
    `PadStack.unconnected_layer_removal`. START_END_ONLY maps to remove_all,
    which is what the same board reads as once saved. Anything unreadable is
    keep_all, the text parser's reading of a pad with no token.
    """
    try:
        from kipy.proto.board import board_types_pb2 as _bt
        name = _bt.UnconnectedLayerRemoval.Name(
            padstack.proto.unconnected_layer_removal)
    except Exception:
        return 'keep_all'
    if name == 'ULR_REMOVE_EXCEPT_START_AND_END':
        return 'remove_except_start_end'
    if name in ('ULR_REMOVE', 'ULR_START_END_ONLY'):
        return 'remove_all'
    return 'keep_all'


def _kipy_paste_override_pair(ovr) -> Tuple[Optional[float], Optional[float]]:
    """`(margin mm, ratio)` of a SolderPasteOverrides PROTO, None where unset.

    HasField, not the value: an unset margin reads 0 on the wire, and 0 is a
    real override ("the opening is the pad") that must not be confused with
    "inherit"."""
    margin = ratio = None
    try:
        if ovr.HasField('solder_paste_margin'):
            margin = _nm_to_mm(ovr.solder_paste_margin.value_nm)
    except Exception:
        margin = None
    try:
        if ovr.HasField('solder_paste_margin_ratio'):
            ratio = float(ovr.solder_paste_margin_ratio.value)
    except Exception:
        ratio = None
    return margin, ratio


def kipy_pad_paste_overrides(padstack) -> Tuple[Optional[float], Optional[float]]:
    """A kipy pad's OWN solder-paste overrides, raw (Pad.paste_margin /
    paste_margin_ratio, #962); None where the pad sets none.

    A KiCad 10 padstack carries them per outer layer. The file has one pair
    per pad, so the front layer's is read, and the back's only when the front
    sets neither."""
    try:
        ps = padstack.proto
    except Exception:
        return None, None
    for side in ('front_outer_layers', 'back_outer_layers'):
        try:
            if not ps.HasField(side):
                continue
            outer = getattr(ps, side)
            if not outer.HasField('solder_paste_settings'):
                continue
            pair = _kipy_paste_override_pair(outer.solder_paste_settings)
        except Exception:
            continue
        if pair != (None, None):
            return pair
    return None, None


def kipy_footprint_paste_overrides(fp) -> Tuple[Optional[float], Optional[float]]:
    """Footprint.paste_margin / paste_margin_ratio for a kipy footprint (#962),
    from `overrides.solder_paste` -- the same message #326's copper clearance
    is read from. None where unset."""
    try:
        p = fp.proto
        if p.HasField('overrides') and p.overrides.HasField('solder_paste'):
            return _kipy_paste_override_pair(p.overrides.solder_paste)
    except Exception:
        pass
    return None, None


#: kipy's ZoneConnectionStyle, spelled like Zone.pad_connection (the names of
#: ZONE_CONNECT_NAMES). ZCS_INHERITED and ZCS_UNKNOWN are "not set".
_KIPY_ZONE_CONNECT_NAMES = {'ZCS_NONE': 'none', 'ZCS_THERMAL': 'thermal',
                            'ZCS_FULL': 'solid', 'ZCS_PTH_THERMAL': 'thru_hole_only'}


def kipy_zone_connect_name(style) -> Optional[str]:
    """A kipy ZoneConnectionStyle value, spelled like Zone.pad_connection;
    None when it is inherited, unknown or unreadable."""
    try:
        from kipy.proto.board.board_types_pb2 import ZoneConnectionStyle
        return _KIPY_ZONE_CONNECT_NAMES.get(ZoneConnectionStyle.Name(int(style)))
    except Exception:
        return None


def kipy_footprint_zone_connection(fp) -> Optional[str]:
    """Footprint.zone_connection for a kipy footprint: the IPC twin of
    `zone_connect_from_pcbnew(fp)`, off `overrides.zone_connection` (the
    message #326's clearance and #962's paste are read from). None = not set."""
    try:
        p = fp.proto
        if not p.HasField('overrides'):
            return None
        return kipy_zone_connect_name(p.overrides.zone_connection)
    except Exception:
        return None


def kipy_pad_zone_connection(padstack) -> Optional[str]:
    """A kipy pad's OWN zone-connection override (the IPC twin of
    `zone_connect_from_pcbnew(pad)`), never an inherited one; None = not set.

    KiCad 9+ keeps the setting per copper layer of the padstack and the API
    also carries a padstack-level copy, so the padstack's is read first and
    the primary copper layer's when that says nothing."""
    try:
        ps = padstack.proto
    except Exception:
        return None
    try:
        if ps.HasField('zone_settings'):
            name = kipy_zone_connect_name(ps.zone_settings.zone_connection)
            if name:
                return name
    except Exception:
        pass
    try:
        layer = ps.copper_layers[0]
        if layer.HasField('zone_settings'):
            return kipy_zone_connect_name(layer.zone_settings.zone_connection)
    except Exception:
        pass
    return None


_FP_OWN_LAYER_RE = re.compile(r'\(layer\s+"?([^"\s)]+)')
_FP_OWN_UUID_RE = re.compile(r'\((?:uuid|tstamp)\s+"?([^"\s)]+)')


def kipy_paste_graphics(content: str, footprints) -> Tuple[List[dict], List[str]]:
    """`extract_paste_graphics` for the IPC builder (#962): every paste GRAPHIC,
    each footprint's posed at its LIVE pose. Returns `(graphics, skipped)`.

    kipy's shape listing omits footprint children, so the geometry comes from
    the board FILE -- but in each footprint's LOCAL frame, which a move does not
    change, then through `local_to_global` at the pose the live board reports.
    Posing at the file's `(at ...)` instead would put a moved part's paste tab
    where the part was at the last save.

    `skipped` names the footprints whose graphics could not be posed, rather
    than guessing them: one FLIPPED since the save (the file's local frame is
    the other side's), one whose block uuid is not the live part's, and one
    the live board no longer has. Board-level paste graphics do not move and
    are taken as the file has them.
    """
    if 'Paste' not in content:
        return [], []
    out = [g for g in extract_paste_graphics(content) if not g.get('owner_ref')]
    skipped: List[str] = []
    masked = _mask_pad_primitives(content)
    for fstart, fend, fkey in _footprint_blocks_by_key(content):
        fp_text = masked[fstart:fend]
        if 'Paste' not in fp_text or not _FP_SHAPE_RE.search(fp_text):
            continue
        shapes = [(tag, blk) for tag, blk in iter_footprint_shapes(fp_text)
                  if 'Paste' in blk]
        if not shapes:
            continue
        live = (footprints or {}).get(fkey)
        m_layer = _FP_OWN_LAYER_RE.search(fp_text)
        m_uuid = _FP_OWN_UUID_RE.search(fp_text)
        if (live is None
                or (m_layer and live.layer and m_layer.group(1) != live.layer)
                or (m_uuid and live.uuid and m_uuid.group(1) != live.uuid)):
            skipped.append(fkey)
            continue

        def tf(x, y, _ox=live.x, _oy=live.y, _or=live.rotation or 0.0):
            return local_to_global(_ox, _oy, _or, x, y)
        for tag, blk in shapes:
            out.extend(_paste_shape_record(tag, blk, fkey, tf))
    return out, skipped


def _fp_library_name(fp) -> str:
    """Best-effort 'library:item' name for a kipy footprint."""
    defn = getattr(fp, "definition", None)
    for src in (defn, fp):
        if src is None:
            continue
        for attr in ("library_link", "name", "id", "footprint_id"):
            v = getattr(src, attr, None)
            if isinstance(v, str) and v:
                return v
            if v is not None:
                try:
                    return str(v)
                except Exception:
                    continue
    return ""


def _fp_position(fp):
    return getattr(fp, "position", None)


def _fp_orientation_deg(fp) -> float:
    o = getattr(fp, "orientation", None)
    if o is None:
        return 0.0
    for attr in ("degrees", "as_degrees", "deg"):
        v = getattr(o, attr, None)
        if callable(v):
            try:
                return float(v())
            except Exception:
                continue
        if isinstance(v, (int, float)):
            return float(v)
    try:
        return float(o)
    except (TypeError, ValueError):
        return 0.0


def _fp_value(fp) -> str:
    """Extract the footprint's value field (e.g. '10k', '0.1uF').

    Same Field → BoardText → value chain as _fp_reference.
    """
    candidates = (
        getattr(fp, "value", None),
        getattr(fp, "value_field", None),
    )
    for v in candidates:
        for _ in range(4):
            if v is None:
                break
            if isinstance(v, str):
                return v
            nxt = getattr(v, "value", None)
            if nxt is None:
                nxt = getattr(v, "text", None)
            if nxt is v:
                break
            v = nxt
    return ""


def _fp_pads(fp):
    """Iterate pads on a kipy FootprintInstance regardless of API shape."""
    # Newer kipy: footprint exposes .pads() / .pads attribute directly
    pads = getattr(fp, "pads", None)
    if callable(pads):
        try:
            return list(pads())
        except Exception:
            pass
    elif pads is not None:
        try:
            return list(pads)
        except Exception:
            pass
    # Older / definition-based: pads live under .definition.pads
    defn = getattr(fp, "definition", None)
    if defn is not None:
        defn_pads = getattr(defn, "pads", None)
        if callable(defn_pads):
            try:
                return list(defn_pads())
            except Exception:
                pass
        elif defn_pads is not None:
            try:
                return list(defn_pads)
            except Exception:
                pass
    return []


def _kipy_geometry_approximations(padstack, primary_layer) -> Tuple[str, ...]:
    """Shape variants this builder flattens, for the pad to disclose (#974).

    The IPC twin of the SWIG builder's `GetChamferPositions()` /
    `Padstack().Mode()` probe and of the text parser's `(chamfer`/`(padstack`
    tokens. It emits the SAME reason strings as both, because a consumer must
    not be able to tell which front read the board.

    Both approximations are real on this path, not theoretical:
    `_pad_shape_label` maps a CHAMFEREDRECT to `roundrect`, and the caller
    reads size/shape from `copper_layers[0]` alone -- so a padstack whose
    layers differ is modelled by its first copper layer.
    """
    out: List[str] = []

    # A chamfer KiCad actually CUTS: any corner flag set. Deliberately not the
    # PSS_CHAMFEREDRECT shape enum -- the other two fronts key on the corners
    # (`GetChamferPositions()`, and the `(chamfer` node, which KiCad writes only
    # when corners are set), so a chamfered-rect with no corner flagged cuts
    # nothing and must stay silent here too, or this front would disclose an
    # approximation the other two do not.
    try:
        corners = getattr(primary_layer, "chamfered_corners", None)
        if corners is not None and any(
                bool(getattr(corners, c, False)) for c in
                ("top_left", "top_right", "bottom_left", "bottom_right")):
            out.append('chamfered pad')
    except Exception:
        out.append('chamfer geometry unavailable')

    # PST_NORMAL is one padstack for every layer; anything else (front/inner/
    # back, or fully custom) is per-layer geometry this builder does not model.
    try:
        from kipy.proto.board.board_types_pb2 import PadStackType
        pst_type = getattr(padstack, "type", None)
        if pst_type is not None and pst_type != PadStackType.PST_NORMAL:
            out.append('per-layer padstack')
    except Exception:
        out.append('padstack geometry unavailable')

    return tuple(out)


def _kipy_chamfered_pad_polygons(padstack, primary_layer, w, h, cx, cy,
                                 abs_rotation_deg):
    """`[outline]` in board mm for a kipy chamfered-rect pad, else None
    (#1450) -- `chamfered_pad_from_text` / `_pcbnew_chamfered_pad_polygons`
    for the IPC path, through the text parser's own `chamfered_rect_polygon`.

    `w`/`h` are the pad-frame size, (`cx`, `cy`) the copper centre. The
    corners are the PadStackLayer's `chamfered_corners` flags, in the pad's
    own frame as the file names them; the other corners round at its
    `corner_rounding_ratio`. A per-layer padstack (not PST_NORMAL) is left to
    its box, as on both other paths, and a pad naming no corner is not
    chamfered."""
    if primary_layer is None or w <= 0 or h <= 0:
        return None
    try:
        from kipy.proto.board import board_types_pb2 as _bt
        if int(getattr(primary_layer, "shape", 0)) != _bt.PSS_CHAMFEREDRECT:
            return None
        ptype = getattr(getattr(padstack, "proto", None), "type", None)
        if ptype is not None and int(ptype) not in (_bt.PST_NORMAL, _bt.PST_UNKNOWN):
            return None
        flags = primary_layer.chamfered_corners
        corners = [n for n, _x, _y in CHAMFER_CORNERS if getattr(flags, n, False)]
        if not corners:
            return None
        ratio = float(getattr(primary_layer, "chamfer_ratio", 0.0) or 0.0)
        rratio = max(0.0, min(0.5, float(
            getattr(primary_layer, "corner_rounding_ratio", 0.0) or 0.0)))
    except Exception:
        return None
    return [chamfered_rect_polygon(cx, cy, w, h, abs_rotation_deg, corners,
                                   ratio, rratio * min(w, h))]


def _build_pad_from_kipy(pad, reference: str, fp_x: float, fp_y: float,
                         fp_rotation: float, get_layer_name,
                         resolve_net,
                         board_copper_layer_names: List[str],
                         fp_clearance: float = 0.0,
                         fp_zone_connection: Optional[str] = None) -> Optional[Pad]:
    """Translate a kipy pad into the project's Pad dataclass.

    board_copper_layer_names is the list of enabled copper layer names on
    the board (e.g. ['F.Cu', 'In1.Cu', 'B.Cu']); through-hole pads
    advertise themselves on every one of these layers.
    """
    try:
        pad_num = str(getattr(pad, "number", "") or "")

        # Position
        pad_pos = getattr(pad, "position", None)
        global_x, global_y = _vec_to_xy_mm(pad_pos)
        local_pos = getattr(pad, "position_relative", None)
        if local_pos is not None:
            local_x, local_y = _vec_to_xy_mm(local_pos)
        else:
            local_x, local_y = _global_to_local(fp_x, fp_y, fp_rotation,
                                                global_x, global_y)

        # kipy's Padstack stores size + shape per-copper-layer on a
        # PadStackLayer (not directly on the padstack). For most pads the
        # F.Cu / B.Cu / inner-Cu copies are identical; we read from the
        # first copper layer.
        padstack = getattr(pad, "padstack", None) or pad
        copper_layers = []
        try:
            copper_layers = list(getattr(padstack, "copper_layers", None) or [])
        except Exception:
            copper_layers = []
        primary_layer = copper_layers[0] if copper_layers else None

        if primary_layer is not None:
            size_vec = getattr(primary_layer, "size", None)
            if size_vec is not None:
                size_x, size_y = _vec_to_xy_mm(size_vec)
            else:
                size_x = size_y = 0.0
            shape_attr = getattr(primary_layer, "shape", None)
            roundrect_rratio = float(getattr(primary_layer, "corner_rounding_ratio", 0.0) or 0.0)
        else:
            # Defensive fallback: older kipy releases or unusual pads where
            # the padstack exposed size directly.
            size = getattr(padstack, "size", None)
            if size is not None:
                size_x, size_y = _vec_to_xy_mm(size)
            else:
                size_x = _nm_to_mm(getattr(padstack, "size_x", 0))
                size_y = _nm_to_mm(getattr(padstack, "size_y", 0))
            shape_attr = getattr(padstack, "shape", None)
            roundrect_rratio = float(getattr(padstack, "corner_radius_ratio", 0.0) or 0.0)

        if shape_attr is None:
            shape = "rect"
        else:
            # kipy returns the PadStackShape enum as a bare int, which has no
            # `.name`: read through `str(int)` it labelled EVERY pad 'rect' --
            # circles, ovals and roundrects boxed, and no custom pad ever
            # reached the #188 polygon read below. The enum's own name first.
            name = getattr(shape_attr, "name", None)
            if not name:
                try:
                    from kipy.proto.board.board_types_pb2 import PadStackShape
                    name = _kipy_enum_name(PadStackShape, shape_attr)
                except Exception:
                    name = ""
            if name:
                shape = _pad_shape_label(str(name).lower())
            else:
                shape = _pad_shape_label(str(shape_attr).lower())

        # The pad's kind and its true layer SET (`kipy_pad_layer_names`, the
        # spelling both of main's parse paths give, 4f3e4e6e); the old
        # drill-based guess only when kipy gives no layer set.
        pad_type = kipy_pad_type(pad)
        pad_layers = (kipy_pad_layer_names(padstack, board_copper_layer_names,
                                           pad_type)
                      or _pad_layer_names(pad, padstack, get_layer_name,
                                          copper_layers, board_copper_layer_names))
        # #967 (62aace71): the size is the max over the copper layers the pad
        # is ON -- a back-only pad of a front/inner/back stack reads its B.Cu
        # entry, not the F.Cu base, and an inner entry counts only where the
        # pad has inner copper.
        _base_size = (size_x, size_y)
        if primary_layer is not None:
            size_x, size_y = _kipy_padstack_on_size(
                padstack, copper_layers, _base_size, pad_layers,
                board_copper_layer_names)

        net = getattr(pad, "net", None)
        net_id, net_name = resolve_net(net)

        # kipy's pad rotation lives on padstack.angle (an Angle wrapper);
        # older releases sometimes exposed it as pad.orientation.
        pad_rotation = 0.0
        rot_obj = None
        for src in (padstack, pad):
            if src is None:
                continue
            for attr in ("angle", "orientation"):
                candidate = getattr(src, attr, None)
                if candidate is not None:
                    rot_obj = candidate
                    break
            if rot_obj is not None:
                break
        if rot_obj is not None:
            for attr in ("degrees", "as_degrees", "deg"):
                v = getattr(rot_obj, attr, None)
                if callable(v):
                    try:
                        pad_rotation = float(v())
                        break
                    except Exception:
                        continue
                elif isinstance(v, (int, float)):
                    pad_rotation = float(v)
                    break
            else:
                try:
                    pad_rotation = float(rot_obj)
                except (TypeError, ValueError):
                    pad_rotation = 0.0

        # Custom pads: the per-layer `size` is only the anchor; enclose the real
        # primitive copper so a track can't graze copper that reaches past the
        # anchor (mirrors the text parser's _custom_pad_local_extent and the SWIG
        # board builder's bbox). PadStackLayer.custom_shapes are BoardShapes in
        # the pad-local frame (centred on the anchor); each exposes a Box2
        # bounding_box (pos = min corner, size = w/h) that correctly covers rects,
        # circles and arcs. Half-extents = furthest corner from the anchor. Done
        # before _resolve_pad_rect so the rotation bakes in like any other pad.
        # #962: a custom pad's ANCHOR size in the pad frame, kept before the
        # primitive-extent override below (parity with the text parser's base
        # `(size ...)`): KiCad sizes the paste ratio term from the anchor.
        anchor_size = _base_size if shape == "custom" else None
        if shape == "custom" and primary_layer is not None:
            ext_x = ext_y = 0.0
            try:
                for cshape in (getattr(primary_layer, "custom_shapes", None) or []):
                    bb = getattr(cshape, "bounding_box", None)
                    bb = bb() if callable(bb) else bb
                    if bb is None:
                        continue
                    px, py = _vec_to_xy_mm(bb.pos)
                    bw, bh = _vec_to_xy_mm(bb.size)
                    ext_x = max(ext_x, abs(px), abs(px + bw))
                    ext_y = max(ext_y, abs(py), abs(py + bh))
            except Exception as e:
                print(f"_build_pad_from_kipy: custom-pad extent failed: {e}")
            if ext_x > 0:
                size_x = max(size_x, 2.0 * ext_x)
            if ext_y > 0:
                size_y = max(size_y, 2.0 * ext_y)

        total_rotation = (pad_rotation + fp_rotation) % 360
        # A chamfered rect (#1450): its cut corners are not copper. The
        # outline the text parser builds (chamfered_rect_polygon), from the
        # pad-frame size, before it is resolved into board space below.
        _cham_polys = _kipy_chamfered_pad_polygons(
            padstack, primary_layer, size_x, size_y, global_x, global_y,
            total_rotation)
        # Padstack size is in pad-local coordinates; resolve it into board space
        # (swaps near 90° and carries a residual rect_rotation for diagonal pads),
        # mirroring the text-parser path so GUI/IPC geometry matches the CLI.
        size_x, size_y, rect_rotation = _resolve_pad_rect(size_x, size_y, pad_rotation)

        pinfunction = getattr(pad, "pin_function", "") or ""
        pintype = getattr(pad, "pin_type", "") or ""

        # kipy Padstack: drill = DrillProperties { diameter: Vector2 }.
        # Oval/slot drills have distinct x/y; take max(x, y) to match the text
        # parser (issue #106) - using only .x under-reports a rotated slot's
        # hole and risks hole-to-hole DRC.
        drill = 0.0
        drill_obj = getattr(padstack, "drill", None) or getattr(padstack, "drill_diameter", None)
        if drill_obj is not None:
            diameter = getattr(drill_obj, "diameter", None) or drill_obj
            if hasattr(diameter, "x_mm"):
                drill = max(float(diameter.x_mm), float(getattr(diameter, "y_mm", diameter.x_mm)))
            elif hasattr(diameter, "x"):
                drill = _nm_to_mm(max(diameter.x, getattr(diameter, "y", diameter.x)))
            else:
                try:
                    drill = _nm_to_mm(diameter)
                except Exception:
                    drill = 0.0

        # Per-pad local clearance override (fiducial keep-clear rings etc.):
        # kipy exposes it as Pad.copper_clearance_override (a Distance) on the
        # proto; unset/0 means no override. Mirrors the text parser's local
        # clearance read (issue #326). Negative (shrinking) overrides clamp to
        # 0. A pad with no own override INHERITS the footprint-level override
        # (fp_clearance), exactly like the text parser and SWIG builder, so
        # clearance consumers only ever read the resolved pad.local_clearance.
        local_clearance = 0.0
        try:
            proto = getattr(pad, "proto", None)
            if proto is not None and proto.HasField("copper_clearance_override"):
                local_clearance = max(
                    0.0, _nm_to_mm(proto.copper_clearance_override.value_nm))
        except Exception:
            local_clearance = 0.0
        if local_clearance == 0.0:
            local_clearance = fp_clearance
        _pad_paste = kipy_pad_paste_overrides(padstack)
        # #1248: pad-to-die length/delay, parity with the text parser.
        _pad_die = kipy_pad_to_die(pad)
        # The pad's zone-connection override, else its footprint's, resolved
        # the way local_clearance is (parity with both other parse paths).
        _pad_zc = kipy_pad_zone_connection(padstack) or fp_zone_connection

        return Pad(
            component_ref=reference,
            pad_number=pad_num,
            global_x=global_x, global_y=global_y,
            local_x=local_x, local_y=local_y,
            size_x=size_x, size_y=size_y,
            shape=shape,
            layers=pad_layers,
            net_id=net_id,
            net_name=net_name,
            rotation=total_rotation,
            pinfunction=pinfunction,
            pintype=pintype,
            drill=drill,
            roundrect_rratio=roundrect_rratio,
            rect_rotation=rect_rotation,
            local_clearance=local_clearance,
            geometry_approximations=_kipy_geometry_approximations(
                padstack, primary_layer),
            # #962: the pad's own paste overrides, raw -- inheritance is
            # resolved by paste_apertures, as on the other two paths.
            paste_margin=_pad_paste[0],
            paste_margin_ratio=_pad_paste[1],
            anchor_size=anchor_size,
            # #1063: which layers KiCad flashes this pad on.
            unconnected_layer_mode=kipy_unconnected_layer_mode(padstack),
            pad_type=pad_type,
            die_length=_pad_die[0],
            die_delay=_pad_die[1],
            zone_connection=_pad_zc,
            polygons=_cham_polys,
        )
    except Exception as e:
        print(f"Warning: failed to read pad {getattr(pad, 'number', '?')}: {e}")
        return None


def _pad_shape_label(raw: str) -> str:
    """Normalise a kipy pad-shape name into the strings the rest of the code uses."""
    raw = raw.lower()
    if "circle" in raw:
        return "circle"
    if "oval" in raw:
        return "oval"
    if "roundrect" in raw or "round_rect" in raw or "chamfered" in raw:
        return "roundrect"
    if "trapezoid" in raw:
        return "trapezoid"
    if "custom" in raw:
        return "custom"
    return "rect"


def _pad_layer_names(pad, padstack, get_layer_name, copper_layers,
                     all_copper_layer_names) -> List[str]:
    """Recover the layer-name list for a kipy pad.

    kipy doesn't expose the wildcard *.Cu / *.Mask / *.Paste forms that the
    .kicad_pcb file format uses. We seed from `padstack.copper_layers[*]`
    (the explicit per-layer entries), then expand for through-hole pads
    where the copper must span every copper layer in the stackup — kipy
    sometimes only stores the F.Cu PadStackLayer for a simple THT pad
    even though the pad punctures every copper layer.
    """
    layers: List[str] = []
    for cl in (copper_layers or []):
        bl = getattr(cl, "layer", None)
        if bl is None:
            continue
        name = get_layer_name(bl)
        if name and name not in layers:
            layers.append(name)

    # Through-hole detection by drill > 0.
    drill_obj = getattr(padstack, "drill", None) or getattr(padstack, "drill_diameter", None)
    drill_mm = 0.0
    if drill_obj is not None:
        diameter = getattr(drill_obj, "diameter", None) or drill_obj
        try:
            if hasattr(diameter, "x_mm"):
                drill_mm = float(diameter.x_mm)
            elif hasattr(diameter, "x"):
                drill_mm = _nm_to_mm(diameter.x)
        except Exception:
            drill_mm = 0.0

    if drill_mm > 0:
        # THT: pad copper exists on every copper layer in the stackup,
        # plus masks on both sides. (No paste — through-hole pads aren't
        # soldered with paste/reflow.)
        for cu_name in (all_copper_layer_names or []):
            if cu_name not in layers:
                layers.append(cu_name)
        for extra in ("F.Mask", "B.Mask"):
            if extra not in layers:
                layers.append(extra)
    else:
        # SMD: mask + paste on the same side as the (single) copper layer.
        if "F.Cu" in layers:
            for extra in ("F.Mask", "F.Paste"):
                if extra not in layers:
                    layers.append(extra)
        elif "B.Cu" in layers:
            for extra in ("B.Mask", "B.Paste"):
                if extra not in layers:
                    layers.append(extra)

    if not layers:
        # Defensive last resort if we couldn't read copper_layers at all.
        layers = ["F.Cu", "F.Mask", "F.Paste"]
    return layers


def _kipy_on_layer(item, bl) -> bool:
    """Is a kipy item on `bl`? kipy hands a proto enum field back as a bare
    int, and `layer_maps()` holds the enum value, so compare by int too."""
    if bl is None:
        return False
    layer = getattr(item, "layer", None)
    if layer is None:
        return False
    if layer == bl:
        return True
    try:
        return int(layer) == int(getattr(bl, "value", bl))
    except (TypeError, ValueError):
        return False


def _kipy_shape_kind(shape) -> str:
    """'segment' / 'arc' / 'circle' / 'rectangle' / 'polygon' / 'bezier' for a
    kipy graphic shape, '' when it is none of them.

    Read off the proto's `geometry` oneof -- a kipy BoardShape has no
    `shape_type` attribute at all, which is why the reader this replaces
    matched no segment, rectangle or arc on Edge.Cuts and an outline drawn
    with lines reached the IPC model as NO outline. Duck-typed fallback for an
    object without the proto (the tests' stand-ins)."""
    try:
        kind = shape.proto.shape.WhichOneof("geometry")
        if kind:
            return kind
    except Exception:
        pass
    for kind, attrs in (("polygon", ("polygons",)),
                        ("bezier", ("control1", "control2")),
                        ("circle", ("radius_point",)),
                        ("rectangle", ("top_left", "bottom_right")),
                        ("arc", ("start", "mid", "end")),
                        ("segment", ("start", "end"))):
        try:
            if all(getattr(shape, a, None) is not None for a in attrs):
                return kind
        except Exception:
            continue
    return ""


def _kipy_edge_shape_segments(shape, chord_eps: Optional[float] = None
                              ) -> List[Tuple[Tuple[float, float], Tuple[float, float]]]:
    """The Edge.Cuts segments of one kipy graphic shape, shape by shape as
    main's pcbnew contour scan reads a PCB_SHAPE (`_shape_segments` in
    `_extract_board_contours_from_pcbnew`): a line under 1 um on both axes is
    skipped (zynq_ad9364's point-lines), a rectangle is its four edges, an arc
    and a bezier are linearized by the text parser's helpers, a circle is a
    ring, and a polygon is every outline ring, its points as pcbnew holds them
    (`kipy_outline_points`, arcs included) with its zero-length edges skipped
    (`_closed_ring_segments`, 6febcfad).

    `chord_eps` is the contour scan's EDGE_CHORD_EPS (#1404), which the arc
    and circle are linearised at; None keeps the helpers' defaults, which the
    bounds scan reads, as the text parser's `extract_board_bounds` does."""
    kind = _kipy_shape_kind(shape)
    _arc_kw = {} if chord_eps is None else {"chord_eps": chord_eps}
    segs = []
    if kind == "segment":
        a, b = _vec_to_xy_mm(shape.start), _vec_to_xy_mm(shape.end)
        if abs(b[0] - a[0]) >= 0.001 or abs(b[1] - a[1]) >= 0.001:
            segs.append((a, b))
    elif kind == "rectangle":
        (x1, y1), (x2, y2) = (_vec_to_xy_mm(shape.top_left),
                              _vec_to_xy_mm(shape.bottom_right))
        segs.extend((((x1, y1), (x2, y1)), ((x2, y1), (x2, y2)),
                     ((x2, y2), (x1, y2)), ((x1, y2), (x1, y1))))
    elif kind == "arc":
        segs.extend(_arc_to_segments(_vec_to_xy_mm(shape.start),
                                     _vec_to_xy_mm(shape.mid),
                                     _vec_to_xy_mm(shape.end), **_arc_kw))
    elif kind == "circle":
        c, e = _vec_to_xy_mm(shape.center), _vec_to_xy_mm(shape.radius_point)
        r = math.hypot(e[0] - c[0], e[1] - c[1])
        if r > 1e-6:
            segs.extend(_circle_to_segments(c[0], c[1], r, **_arc_kw))
    elif kind == "bezier":
        segs.extend(_bezier_to_segments(
            _vec_to_xy_mm(shape.start), _vec_to_xy_mm(shape.control1),
            _vec_to_xy_mm(shape.control2), _vec_to_xy_mm(shape.end)))
    elif kind == "polygon":
        for pwh in (getattr(shape, "polygons", None) or []):
            segs.extend(_closed_ring_segments(
                kipy_outline_points(getattr(pwh, "outline", None), closed=True)))
    return segs


def _kipy_edge_bound_points(shape) -> List[Tuple[float, float]]:
    """The points one Edge.Cuts shape contributes to the board bounds, by the
    text parser's `extract_board_bounds` rules: a line's ends (zero-length
    included), a rectangle's corners, an arc's and a bezier's linearized
    points, a polygon's pcbnew points, a circle's exact extremes."""
    kind = _kipy_shape_kind(shape)
    if kind == "segment":
        return [_vec_to_xy_mm(shape.start), _vec_to_xy_mm(shape.end)]
    if kind == "rectangle":
        return [_vec_to_xy_mm(shape.top_left), _vec_to_xy_mm(shape.bottom_right)]
    if kind in ("arc", "bezier"):
        return [p for seg in _kipy_edge_shape_segments(shape) for p in seg]
    if kind == "circle":
        c, e = _vec_to_xy_mm(shape.center), _vec_to_xy_mm(shape.radius_point)
        r = math.hypot(e[0] - c[0], e[1] - c[1])
        if r <= 1e-6:
            return []
        return [(c[0] - r, c[1] - r), (c[0] + r, c[1] + r)]
    if kind == "polygon":
        return [p for pwh in (getattr(shape, "polygons", None) or [])
                for p in kipy_outline_points(getattr(pwh, "outline", None),
                                             closed=True)]
    return []


def _kipy_fp_shapes(fp) -> list:
    """A kipy footprint's own graphic shapes, in BOARD coordinates (KiCad keeps
    a footprint's children at absolute positions, and the API serializes them
    so). `board.get_shapes()` asks for top-level shapes only and never returns
    these -- which is how a footprint's Edge.Cuts and copper art went missing
    from the IPC model."""
    try:
        return list(fp.definition.shapes)
    except Exception:
        return []


def _compute_edge_cuts_bbox_kipy(drawings, edge_cuts_bl,
                                 footprints=()) -> Optional[Tuple[float, float, float, float]]:
    """Bounding box of the Edge.Cuts geometry: board drawings and, as on both
    of main's parse paths, the Edge.Cuts a footprint draws (a board whose
    whole outline lives in a footprint). `footprints` is (key, kipy footprint)
    pairs. The old reader probed attribute names a kipy shape does not have
    (a rectangle's corners, a circle's radius) and missed both."""
    if edge_cuts_bl is None:
        return None
    shapes = [d for d in (drawings or ()) if _kipy_on_layer(d, edge_cuts_bl)]
    for _key, fp in (footprints or ()):
        shapes.extend(g for g in _kipy_fp_shapes(fp)
                      if _kipy_on_layer(g, edge_cuts_bl))
    bmin_x = bmin_y = float("inf")
    bmax_x = bmax_y = float("-inf")
    found = False
    for d in shapes:
        try:
            pts = _kipy_edge_bound_points(d)
        except Exception:
            continue
        for x, y in pts:
            bmin_x = min(bmin_x, x); bmax_x = max(bmax_x, x)
            bmin_y = min(bmin_y, y); bmax_y = max(bmax_y, y)
            found = True
    if not found or bmax_x <= bmin_x or bmax_y <= bmin_y:
        return None
    return (bmin_x, bmin_y, bmax_x, bmax_y)


def _polyline_points_mm(polyline) -> List[Tuple[float, float]]:
    """Extract (x_mm, y_mm) from every node of a kipy PolyLine.

    PolyLineNode wraps a Vector2 — either exposed as .point or directly as
    .x/.y (nm) or .x_mm/.y_mm. Probe defensively across kipy versions. An ARC
    node is replaced by its pcbnew polyline (`kicad_arc_polyline`): read as a
    point it is the empty Vector2, i.e. a phantom (0, 0) vertex.
    """
    out: List[Tuple[float, float]] = []
    if polyline is None:
        return out
    for node in _kipy_nodes(polyline):
        arc = _kipy_node_arc(node)
        if arc is not None:
            for x, y in kicad_arc_polyline(*arc):
                if not out or out[-1] != (x / 1e6, y / 1e6):
                    out.append((x / 1e6, y / 1e6))
            continue
        candidate = getattr(node, "point", node)
        if hasattr(candidate, "x_mm") and hasattr(candidate, "y_mm"):
            try:
                out.append((float(candidate.x_mm), float(candidate.y_mm)))
                continue
            except Exception:
                pass
        if hasattr(candidate, "x") and hasattr(candidate, "y"):
            try:
                out.append((_nm_to_mm(candidate.x), _nm_to_mm(candidate.y)))
            except Exception:
                continue
    return out


def _kipy_nodes(polyline) -> list:
    """The nodes of a kipy PolyLine (or of anything iterable of nodes)."""
    if polyline is None:
        return []
    nodes = getattr(polyline, "nodes", None)
    if nodes is None:
        try:
            nodes = list(polyline)
        except TypeError:
            nodes = []
    return list(nodes)


def _kipy_vec_nm(v) -> Tuple[int, int]:
    """A kipy Vector2 as integer nanometres (kipy 0.7 exposes .x/.y in nm;
    a stand-in may expose .x_mm/.y_mm only)."""
    try:
        return (int(v.x), int(v.y))
    except Exception:
        return (int(round(float(v.x_mm) * 1e6)), int(round(float(v.y_mm) * 1e6)))


def _kipy_node_arc(node):
    """(start, mid, end) in integer nm for an ARC node of a kipy PolyLine,
    None for a point node."""
    try:
        if not getattr(node, "has_arc", False):
            return None
        a = node.arc
        return (_kipy_vec_nm(a.start), _kipy_vec_nm(a.mid), _kipy_vec_nm(a.end))
    except Exception:
        return None


def kipy_outline_points(polyline, closed: bool) -> List[Tuple[float, float]]:
    """A kipy PolyLine's vertices as pcbnew holds them (mm), through the text
    parser's own `_KicadOutlineChain`:

    - closed=False: a ZONE or rule-area outline (`zone_outline_points`) -- the
      chain is closed after its points arrive, so a last point repeating the
      first is merged (37fb1b93);
    - closed=True: a gr_poly / fp_poly (`kicad_poly_shape_points`) -- closed
      before, so a repeated closing point is kept (6febcfad).

    An arc node is `kicad_arc_polyline` from its own start/mid/end, which is
    what the text parser computes from the file's `(arc ...)` entry (fe367f3f):
    the reader this replaces took an arc node's empty `point`, putting a
    (0, 0) vertex into the outline. NOT probed against a live KiCad: that the
    API packs an arc segment of the loaded chain as one arc node carrying the
    SHAPE_ARC's start/mid/end is read from its PolyLine contract, and the
    tests feed the nodes as the file spells them."""
    chain = _KicadOutlineChain(closed=closed)
    for node in _kipy_nodes(polyline):
        arc = _kipy_node_arc(node)
        if arc is not None:
            chain.append_arc(*arc)
            continue
        if getattr(node, "has_point", True) is False:
            continue
        try:
            p = _kipy_vec_nm(getattr(node, "point", node))
        except Exception:
            continue
        chain.append_xy(p, (p[0] / 1e6, p[1] / 1e6))
    if not closed:
        chain.close()
    return chain.mm


def kipy_board_contours(drawings, edge_cuts_bl, footprints=(),
                        include_footprints=True, owners_out=None,
                        own_pads_out=None):
    """(outers, cutouts) from Edge.Cuts -- the kipy twin of main's
    `_extract_board_contours_from_pcbnew`, rule for rule: every shape's
    segments (`_kipy_edge_shape_segments`), the Edge.Cuts a footprint draws
    (#304: crkbd's per-LED windows), the 4-segment axis-aligned rectangle
    short-circuit, the chainer, and `_classify_contours` (multi-outline
    boards, true cut-outs). `footprints` is (key, kipy footprint) pairs;
    `owners_out`, when a dict, gets each footprint's own segments by key
    (#829), and `own_pads_out` each such footprint's own pads
    (`_kipy_footprint_own_pads`, #1409); `include_footprints=False` asks what
    the outline would be with no footprint contributing. Arcs and circles are
    linearised at EDGE_CHORD_EPS (#1404), as the pcbnew and text scans are."""
    if edge_cuts_bl is None:
        return [], []
    segments = []
    for d in (drawings or ()):
        if not _kipy_on_layer(d, edge_cuts_bl):
            continue
        try:
            segments.extend(_kipy_edge_shape_segments(d, chord_eps=EDGE_CHORD_EPS))
        except Exception:
            continue
    for key, fp in (footprints if include_footprints else ()):
        fp_segs = []
        for g in _kipy_fp_shapes(fp):
            if not _kipy_on_layer(g, edge_cuts_bl):
                continue
            try:
                segs = _kipy_edge_shape_segments(g, chord_eps=EDGE_CHORD_EPS)
            except Exception:
                continue
            fp_segs.extend(segs)
        segments.extend(fp_segs)
        if owners_out is not None and fp_segs:
            # #1409: the owner's own pads, keyed with its segments so the two
            # can never be matched up wrong. Two owners sharing one key pool
            # their segments under it, so no one footprint's pads describe
            # the pool: leave it to containment.
            if own_pads_out is not None:
                own_pads_out[key] = (None if key in own_pads_out
                                     else _kipy_footprint_own_pads(fp))
            owners_out.setdefault(key, []).extend(fp_segs)

    if len(segments) < 3:
        return [], []
    if len(segments) == 4:
        vertices = set()
        for seg in segments:
            vertices.add((round(seg[0][0], 3), round(seg[0][1], 3)))
            vertices.add((round(seg[1][0], 3), round(seg[1][1], 3)))
        if len(vertices) == 4 and all(
                abs(s[0][0] - s[1][0]) < 0.001 or abs(s[0][1] - s[1][1]) < 0.001
                for s in segments):
            return [], []
    contours = _chain_segments_into_contours(segments)
    if not contours:
        return [], []
    return _classify_contours(contours)


def _extract_board_contours_kipy(drawings, edge_cuts_bl):
    """(outline, other rings) -- the shape this reader had before it took
    main's classification (`kipy_board_contours`); kept for its callers that
    want one ring. The builder reads `kipy_board_contours` directly."""
    outers, cutouts = kipy_board_contours(drawings, edge_cuts_bl)
    return (outers[0] if outers else []), list(outers[1:]) + list(cutouts)


def _kipy_footprint_own_pads(fp):
    """The kipy mirror of `_footprint_own_pads_by_ref` for one footprint
    (#1409): `(fx, fy, rot, [local pad centres])`. kipy hands back pad
    positions in BOARD coordinates, so each is carried into the footprint's
    frame by `_global_to_local` -- the pose's exact inverse, the same one the
    classifier applies to the ring -- as `_pcbnew_footprint_own_pads` does."""
    fx, fy = _vec_to_xy_mm(_fp_position(fp))
    rot = _fp_orientation_deg(fp)
    pads = []
    for pad in _fp_pads(fp):
        try:
            px, py = _vec_to_xy_mm(pad.position)
            pads.append(_global_to_local(fx, fy, rot, px, py))
        except Exception:                                        # noqa: BLE001
            continue
    return (fx, fy, rot, pads)


def kipy_footprint_outline_owners(drawings, edge_cuts_bl, footprints) -> Dict[str, bool]:
    """`{key: owns_board_outline}` for every footprint that draws Edge.Cuts --
    the kipy twin of `footprint_outline_owners_from_pcbnew` (#829), through the
    same decision function (`classify_outline_owners`) on the same three
    inputs: each footprint's own segments, the bounds of the board-level
    Edge.Cuts alone, and the outline the board draws with no footprint."""
    owners: Dict[str, list] = {}
    own_pads: Dict[str, object] = {}
    kipy_board_contours(drawings, edge_cuts_bl, footprints, owners_out=owners,
                        own_pads_out=own_pads)
    if not owners:
        return {}
    board_only_bounds = _compute_edge_cuts_bbox_kipy(drawings, edge_cuts_bl)
    outers, _c = kipy_board_contours(drawings, edge_cuts_bl,
                                     include_footprints=False)
    return classify_outline_owners(owners, board_only_bounds, outers,
                                   own_pads=own_pads)


def _extract_stackup_kipy(board) -> List[StackupLayer]:
    """Stackup extraction via kipy with file-parse fallback.

    BoardStackupLayer in kipy 0.7 exposes:
        layer (BoardLayer enum), user_name (str), thickness (nm int),
        type (BoardStackupLayer.Type enum), material_name (str),
        dielectric (BoardStackupDielectricLayer with epsilon_r,
                    loss_tangent, material_name, thickness)
    We map type → 'copper' / 'core' / 'prepreg' / 'dielectric' and pull
    epsilon_r / loss_tangent off the .dielectric sub-message when present.
    """
    stackup: List[StackupLayer] = []

    try:
        s = board.get_stackup()
        layers = getattr(s, "layers", None) or []
        for item in layers:
            try:
                name = getattr(item, "user_name", "") or ""
                # type is a proto enum; map by its .name (e.g. "BSLT_COPPER").
                type_enum = getattr(item, "type", None)
                if type_enum is None:
                    continue
                type_name = (getattr(type_enum, "name", None) or str(type_enum)).lower()
                if "copper" in type_name:
                    ltype = "copper"
                elif "core" in type_name:
                    ltype = "core"
                elif "prepreg" in type_name:
                    ltype = "prepreg"
                elif "dielec" in type_name:
                    ltype = "dielectric"
                else:
                    # Skip silkscreen / soldermask / paste / etc.
                    continue

                thickness_mm = _nm_to_mm(getattr(item, "thickness", 0) or 0)
                material = str(getattr(item, "material_name", "") or "")

                epsilon_r = 0.0
                loss_tangent = 0.0
                dielectric = getattr(item, "dielectric", None)
                if dielectric is not None:
                    try:
                        epsilon_r = float(getattr(dielectric, "epsilon_r", 0.0) or 0.0)
                        loss_tangent = float(getattr(dielectric, "loss_tangent", 0.0) or 0.0)
                        # Dielectric sub-message may carry its own thickness /
                        # material_name; prefer them if the top-level fields
                        # are empty.
                        if thickness_mm == 0:
                            thickness_mm = _nm_to_mm(getattr(dielectric, "thickness", 0) or 0)
                        if not material:
                            material = str(getattr(dielectric, "material_name", "") or "")
                    except Exception:
                        pass

                stackup.append(StackupLayer(
                    name=name, layer_type=ltype, thickness=thickness_mm,
                    epsilon_r=epsilon_r, loss_tangent=loss_tangent, material=material,
                ))
            except Exception:
                continue
    except Exception:
        pass

    if stackup:
        return stackup

    # Fallback: parse from the .kicad_pcb file using the absolute path.
    try:
        from kicad_ipc_adapter import get_board_full_path
        board_file = get_board_full_path()
    except Exception:
        board_file = None
    if board_file:
        try:
            with open(board_file, "r", encoding="utf-8") as f:
                content = f.read(8192)  # stackup lives near the top of the file
                # ...but it can start past the head (a long title block) or
                # run out of it (a deep stackup of multi-sheet dielectrics):
                # read on rather than parse none or half of it (#1222).
                span = _stackup_block_span(content)
                if (span[1] >= len(content)) if span else ('(setup' not in content):
                    content += f.read()
            stackup = extract_stackup(content)
        except Exception:
            pass

    return stackup


def _kipy_enum_name(enum_cls, value) -> str:
    """A proto enum value's NAME ('PSS_CIRCLE'), '' when it has none. kipy
    returns enum fields as bare ints, so `getattr(value, 'name')` -- which
    this builder used to ask -- answers nothing for every one of them."""
    try:
        return str(enum_cls.Name(int(value)))
    except Exception:
        return ""


def _kipy_layer_token(bl) -> str:
    """A BoardLayer's canonical name ('F.Fab', 'In3.Cu', 'User.12'), '' when
    unknown. Not `layer_maps()`: that table names copper, mask, paste, silk,
    Edge.Cuts and User.1-9 only and maps anything else to 'F.Cu', which would
    put an F.Fab pad layer or a courtyard zone on copper."""
    try:
        from kipy.util.board_layer import canonical_name
        name = canonical_name(bl)
        return "" if name == "Unknown" else str(name)
    except Exception:
        return ""


def _kipy_uuid(item) -> str:
    """A kipy item's KIID as the file spells it (its `(uuid ...)`), '' when it
    cannot be read. Tracks, vias and zones carry it on both of main's parse
    paths: a `sense:` group's members resolve against it (#1279), and a zone's
    ties its per-layer entries back together."""
    for get in (lambda: item.id.value, lambda: item.proto.id.value):
        try:
            v = get()
        except Exception:
            continue
        if v:
            return str(v)
    return ""


_KIPY_VIA_TYPE_NAMES = {'VT_THROUGH': 'through', 'VT_MICRO': 'micro',
                        'VT_BURIED': 'buried', 'VT_BLIND': 'blind',
                        'VT_BLIND_BURIED': 'blind'}


def kipy_via_type(via) -> str:
    """A kipy Via's type in Via.via_type's spelling -- 'through', 'blind',
    'buried' or 'micro' -- the twin of main's `pcbnew_via_type` and of the text
    parser's `_via_type_from_block` (#1268). KiCad 9 has one BLIND_BURIED type
    (read as 'blind', as its file writes it); KiCad 10 splits it. A via whose
    type kipy cannot name falls back to its drill span, as main's does to the
    layer pair: F.Cu/B.Cu (or a span kipy left unknown) is 'through', anything
    else 'blind'. Never raises."""
    try:
        from kipy.proto.board.board_types_pb2 import ViaType
        name = _KIPY_VIA_TYPE_NAMES.get(_kipy_enum_name(ViaType, via.type))
        if name:
            return name
    except Exception:
        pass
    try:
        from kipy.proto.board.board_types_pb2 import BoardLayer
        d = via.padstack.drill
        span = {int(d.start_layer), int(d.end_layer)}
        if span & {int(BoardLayer.BL_UNKNOWN), int(BoardLayer.BL_UNDEFINED)}:
            return 'through'
        if span == {int(BoardLayer.BL_F_Cu), int(BoardLayer.BL_B_Cu)}:
            return 'through'
        return 'blind'
    except Exception:
        return 'through'


def _copper_rank(name: str) -> int:
    """Stack position of a copper layer name: F.Cu, In1..In30, B.Cu."""
    if name == 'F.Cu':
        return 0
    if name == 'B.Cu':
        return 99
    m = re.match(r'In(\d+)\.Cu$', name or '')
    return int(m.group(1)) if m else 50


def kipy_via_layers(via, via_type: str) -> List[str]:
    """A kipy via's layer pair, top layer first, as both of main's parse paths
    read it (6552e70f: pcbnew's TopLayer()/BottomLayer()). A THROUGH via spans
    F.Cu/B.Cu whatever its drill says, as KiCad loads an untyped via
    (db60fe77); a span kipy cannot name reads F.Cu/B.Cu too."""
    if via_type == 'through':
        return ['F.Cu', 'B.Cu']
    try:
        d = via.padstack.drill
        names = [_kipy_layer_token(d.start_layer), _kipy_layer_token(d.end_layer)]
    except Exception:
        return ['F.Cu', 'B.Cu']
    if not all(n.endswith('.Cu') for n in names):
        return ['F.Cu', 'B.Cu']
    return sorted(names, key=_copper_rank)


def kipy_sense_groups(board) -> List[Tuple[str, str, List[str]]]:
    """`sense_branch.sense_groups_from_text` for a live kipy board (#1279):
    [(group name, group uuid, [member uuid, ...])] for every `sense:` group.

    The twin of main's `sense_groups_from_board`, which reads pcbnew's
    board.Groups(); here it is `board.get_groups()` (kipy 0.7 / KiCad 10).
    The members are the group's own item ids as KiCad holds them -- KiCad
    drops a member that matches no item when it loads the board, so a live
    group has no dangling member, which is what the text reader reports apart.
    A board whose groups cannot be read says so, as main's does, rather than
    protecting nothing in silence."""
    from sense_branch import GROUP_PREFIX
    out = []
    try:
        groups = list(board.get_groups())
    except Exception as e:  # noqa: BLE001
        print(f"  WARNING: could not read the board's groups over IPC "
              f"({type(e).__name__}: {e}); sense branches (#1279) are not "
              f"protected")
        return out
    for g in groups:
        name = str(getattr(g, "name", "") or "")
        if not name.startswith(GROUP_PREFIX):
            continue
        try:
            members = [str(k.value) for k in g.proto.items]
        except Exception:
            members = [_kipy_uuid(it) for it in (getattr(g, "items", None) or [])]
        out.append((name, _kipy_uuid(g), members))
    return out


def kipy_resolve_sense_branches(pcb_data, board) -> None:
    """Re-resolve `pcb_data.sense_branches` from a live kipy board's `sense:`
    groups (#1279) -- the twin of main's `resolve_live_sense_branches`, for a
    dialog that keeps ONE PCBData across steps.

    Resolve against the dialog's OWN PCBData, never adopt a fresh build's
    branches: the resolver tags each sense pad (`pad.sense`, read by the plane
    taps, the rescue and the oracle off `pads_by_net`), and a fresh build's
    branches tag the fresh build's Pad objects, leaving the dialog's pads with
    the tags of the last resolution. So the old tags are cleared first, as on
    main, and the protection memo is dropped because holds decide it. Call it
    once the dialog's copper, pads and zones are current."""
    from sense_branch import resolve_sense_branches
    for pads in (getattr(pcb_data, 'pads_by_net', None) or {}).values():
        for pad in pads:
            if getattr(pad, 'sense', ''):
                pad.sense = ''
    pcb_data.sense_branches = resolve_sense_branches(pcb_data,
                                                     kipy_sense_groups(board))
    vars(pcb_data).pop('_protection_map_memo', None)


_KIPY_PAD_TYPES = {'PT_PTH': 'thru_hole', 'PT_SMD': 'smd',
                   'PT_EDGE_CONNECTOR': 'connect', 'PT_NPTH': 'np_thru_hole'}


def kipy_pad_type(pad) -> str:
    """A kipy pad's kind in Pad.pad_type's spelling ('smd', 'thru_hole',
    'np_thru_hole', 'connect' -- the file's own words), '' when unknown.
    `pad_is_plated_through` and the NPTH-has-no-copper rule read it."""
    try:
        from kipy.proto.board.board_types_pb2 import PadType
        return _KIPY_PAD_TYPES.get(_kipy_enum_name(PadType, pad.pad_type), '')
    except Exception:
        return ''


def kipy_pad_layer_names(padstack, board_copper: Sequence[str],
                         pad_type: str) -> List[str]:
    """A kipy pad's layer SET as `canonical_pad_layers` spells it -- the set
    pcbnew loads, the spelling both of main's parse paths agree on (`*.Cu`,
    `*.Mask`, an F.Fab or User layer by its name; 4f3e4e6e and main's pcbnew
    builder). Read off `padstack.layers`, the pad's own layer set; [] when
    kipy gives none, and the caller falls back to `_pad_layer_names`. Copper
    is listed first, in stack order: a consumer reading `layers[0]` takes it
    for the pad's copper layer."""
    try:
        names = [_kipy_layer_token(bl) for bl in (padstack.layers or ())]
    except Exception:
        return []
    names = [n for n in names if n]
    if not names:
        return []
    cu = sorted((n for n in names if n.endswith('.Cu')), key=_copper_rank)
    names = cu + [n for n in names if not n.endswith('.Cu')]
    return canonical_pad_layers(names, board_copper,
                                plated_through=(pad_type == 'thru_hole'))


def _kipy_padstack_on_size(padstack, copper_layers, base: Tuple[float, float],
                           pad_layers: Sequence[str],
                           board_copper: Sequence[str]) -> Tuple[float, float]:
    """`_padstack_copper_size` for a kipy padstack (#967, 62aace71): the
    per-axis MAX of the pad's size over the copper layers it is ON, within the
    board's copper. `base` is the first copper entry, the F.Cu geometry, as the
    file's base `(size ...)` is; a front/inner/back stack adds its inner and
    B.Cu entries, a custom one an entry per layer, and a layer with no entry
    keeps the base. A padstack entry for a layer the pad is not on describes
    no copper."""
    try:
        from kipy.proto.board.board_types_pb2 import PadStackType
        mode = int(getattr(padstack, "type", PadStackType.PST_NORMAL))
        fib = int(PadStackType.PST_FRONT_INNER_BACK)
        custom = int(PadStackType.PST_CUSTOM)
    except Exception:
        return base
    if mode not in (fib, custom):
        return base
    per = {}
    for cl in (copper_layers or []):
        n = _kipy_layer_token(getattr(cl, "layer", None))
        size = getattr(cl, "size", None)
        if n and size is not None:
            per.setdefault(n, _vec_to_xy_mm(size))
    on = [ln for ln in board_copper if '*.Cu' in pad_layers or ln in pad_layers]
    if not on:
        return base
    inner = next((sz for n, sz in per.items() if n.startswith('In')), base)

    def size_on(ln):
        if ln == 'F.Cu':
            return per.get('F.Cu', base)
        if mode == fib:
            return per.get('B.Cu', base) if ln == 'B.Cu' else inner
        return per.get(ln, base)
    got = [size_on(ln) for ln in on]
    return max(s[0] for s in got), max(s[1] for s in got)


def _kipy_custom_copper_is_anchor(primary_layer, anchor: Tuple[float, float],
                                  eps: float = 1e-4) -> bool:
    """`_custom_copper_is_anchor` for a kipy custom pad: is its copper exactly
    the anchor -- no primitive, or every primitive (with half its stroke)
    inside the anchor rect or circle? The primitives are the padstack layer's
    `custom_shapes`, in the pad frame about the anchor. A primitive this
    cannot measure (a bezier) is never assumed inside, the text rule. Such a
    pad takes an ordinary pad's size and residual tilt; any other custom pad
    is sized by the board-frame box of its copper (#967)."""
    try:
        shapes = list(getattr(primary_layer, "custom_shapes", None) or [])
    except Exception:
        return False
    if not shapes:
        return True
    try:
        from kipy.proto.board.board_types_pb2 import PadStackShape
        circle = (int(getattr(primary_layer, "custom_anchor_shape", 0))
                  == int(PadStackShape.PSS_CIRCLE))
    except Exception:
        circle = False
    hx, hy = anchor[0] / 2.0, anchor[1] / 2.0
    r = min(hx, hy)

    def inside(x, y, grow):
        if circle:
            return math.hypot(x, y) + grow <= r + eps
        return abs(x) + grow <= hx + eps and abs(y) + grow <= hy + eps

    for sh in shapes:
        kind = _kipy_shape_kind(sh)
        try:
            hw = _nm_to_mm(sh.attributes.stroke.width) / 2.0
        except Exception:
            hw = 0.0
        if kind == "segment":
            pts = [_vec_to_xy_mm(sh.start), _vec_to_xy_mm(sh.end)]
        elif kind == "arc":
            pts = [p for seg in _arc_to_segments(_vec_to_xy_mm(sh.start),
                                                 _vec_to_xy_mm(sh.mid),
                                                 _vec_to_xy_mm(sh.end))
                   for p in seg]
        elif kind == "rectangle":
            (x1, y1), (x2, y2) = (_vec_to_xy_mm(sh.top_left),
                                  _vec_to_xy_mm(sh.bottom_right))
            pts = [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]
        elif kind == "circle":
            c, e = _vec_to_xy_mm(sh.center), _vec_to_xy_mm(sh.radius_point)
            rad = math.hypot(e[0] - c[0], e[1] - c[1])
            if circle:
                if math.hypot(*c) + rad + hw > r + eps:
                    return False
                continue
            pts = [(c[0] - rad, c[1] - rad), (c[0] + rad, c[1] + rad)]
        elif kind == "polygon":
            pts = [p for pwh in (getattr(sh, "polygons", None) or [])
                   for p in _kipy_graphic_poly_points(getattr(pwh, "outline", None))]
        else:
            return False
        if not pts:
            return False
        if not all(inside(x, y, hw) for x, y in pts):
            return False
    return True


def kipy_pad_to_die(pad) -> Tuple[float, float]:
    """`pad_to_die_from_text` for a kipy pad (#1248): (die_length mm,
    die_delay ps), (0.0, 0.0) when unset. The delay is a kipy `Time` in
    attoseconds, which is KiCad's time IU (IU_PER_PS = 1e6, the text
    parser's docstring), so ps = value_as / 1e6. A KiCad older than 10 has no
    delay field and reads 0.0, as its file would."""
    length = delay = 0.0
    try:
        length = _nm_to_mm(pad.pad_to_die_length)
    except Exception:
        length = 0.0
    try:
        p = pad.proto
        if p.HasField("pad_to_die_delay"):
            delay = float(p.pad_to_die_delay.value_as) / 1e6
    except Exception:
        delay = 0.0
    return length, delay


def kipy_footprint_sheet_fields(fp) -> Tuple[str, str]:
    """`(sheet_name, sheet_file)` of a kipy footprint (#1211), the text
    parser's `_footprint_sheet_fields`: FootprintInstance's
    `symbol_sheet_name` / `symbol_sheet_filename` (KiCad 9.0.7+), which KiCad
    fills from the same FOOTPRINT fields main's pcbnew builder reads
    (GetSheetname / GetSheetfile). '' when this KiCad does not send them."""
    try:
        p = fp.proto
        return (str(p.symbol_sheet_name or ""), str(p.symbol_sheet_filename or ""))
    except Exception:
        return ("", "")


def kipy_ref_label(fp, fp_x: float, fp_y: float,
                   fp_rotation: float) -> Optional['RefLabel']:
    """The Reference field's geometry (#481) off a kipy footprint, in the
    text parser's RefLabel form: the offset from the footprint origin in its
    unrotated frame (pcbnew's GetFPRelativePosition, the inverse of the
    footprint transform), the field's stored -- absolute -- angle, its size
    (file order HEIGHT WIDTH), stroke, layer, justification and visibility.
    None when kipy cannot say. The read this replaces called pcbnew's
    accessors on a kipy footprint and an undefined `to_mm`, so every IPC
    footprint came back with no label. Not probed against a live KiCad."""
    try:
        from kipy.proto.common.types.enums_pb2 import (HorizontalAlignment,
                                                        VerticalAlignment)
        rf = fp.reference_field
        txt = rf.text
        gx, gy = _vec_to_xy_mm(txt.position)
        lx, ly = _global_to_local(fp_x, fp_y, fp_rotation, gx, gy)
        attrs = txt.attributes
        size = attrs.size
        just = []
        ha = _kipy_enum_name(HorizontalAlignment, attrs.horizontal_alignment)
        va = _kipy_enum_name(VerticalAlignment, attrs.vertical_alignment)
        if ha == 'HA_LEFT':
            just.append('left')
        elif ha == 'HA_RIGHT':
            just.append('right')
        if va == 'VA_TOP':
            just.append('top')
        elif va == 'VA_BOTTOM':
            just.append('bottom')
        if attrs.mirrored:
            just.append('mirror')
        return RefLabel(
            at_x=round(lx * 1e6) / 1e6,
            at_y=round(ly * 1e6) / 1e6,
            rotation=float(attrs.angle) % 360,
            size_h=_nm_to_mm(size.y),
            size_w=_nm_to_mm(size.x),
            thickness=_nm_to_mm(attrs.stroke_width),
            layer=_kipy_layer_token(rf.layer) or "F.SilkS",
            justify=' '.join(just),
            hidden=not bool(rf.visible),
            is_property_node=True)
    except Exception:
        return None


def kipy_net_tie_groups(fp) -> List[List[str]]:
    """A kipy footprint's net-tie pad groups, the text parser's
    `(net_tie_pad_groups "1, 2")` read off its definition's `net_ties`. Each
    entry's pad numbers are split on commas and whitespace too, so a group
    KiCad hands over as one "1, 2" string reads the same as one it splits.
    The read this replaces called pcbnew's IsNetTie() on a kipy footprint and
    found no group on any board."""
    out = []
    try:
        ties = list(fp.definition.proto.net_ties)
    except Exception:
        return out
    for t in ties:
        nums = []
        for tok in getattr(t, "pad_number", None) or ():
            for part in re.split(r'[,\s]+', str(tok)):
                if part and part not in nums:
                    nums.append(part)
        if len(nums) >= 2:
            out.append(nums)
    return out


def _kipy_graphic_poly_points(polyline) -> List[Tuple[float, float]]:
    """`poly_points_with_arcs` for a kipy PolyLine: points as held, each arc
    node linearized by `_arc_to_segments` from its own start/mid/end, and --
    only when an arc was present -- consecutive repeats and a closing repeat
    dropped (`_ring_without_repeats`), exactly the text rule."""
    pts, arcs = [], False
    for node in _kipy_nodes(polyline):
        arc = _kipy_node_arc(node)
        if arc is not None:
            arcs = True
            s, m, e = [(x / 1e6, y / 1e6) for x, y in arc]
            segs = _arc_to_segments(s, m, e)
            pts.append(segs[0][0])
            pts.extend(p1 for _p0, p1 in segs)
            continue
        if getattr(node, "has_point", True) is False:
            continue
        try:
            x, y = _kipy_vec_nm(getattr(node, "point", node))
        except Exception:
            continue
        pts.append((x / 1e6, y / 1e6))
    return _ring_without_repeats(pts) if arcs else pts


def kipy_graphic_copper(drawings, footprints, resolve_net) -> List[Segment]:
    """[Segment(graphic=True)] for a kipy board's copper GRAPHICS: board-level
    shapes on a copper layer (#337) and the copper a footprint's land pattern
    draws (#908) -- the twin of main's `pcbnew_graphic_copper` and of the text
    parser's gr_* / fp_* pass, emitted the text parser's way:

    - every copper shape of a pad-bearing footprint (NPTH pads do not count,
      `footprint_copper_is_functional`); a pad-less logo footprint's copper is
      the writer's to relocate to silk and is not modelled;
    - a line or an arc at its stroke, a polygon / rectangle / circle as its
      outline (a 0-stroke filled shape at the fab track width), its filled
      interior as `graphic_ring` (#1181), a zero-length edge skipped;
    - an UNFILLED zero-width shape at UNSTROKED_SHAPE_WIDTH, pcbnew's loader
      rule (6925b417) -- kipy hands over the loaded width, so this is a guard;
    - a polygon's arc vertices linearized by `_arc_to_segments` from their own
      control points (`_kipy_graphic_poly_points`, 6925b417);
    - a bezier is not modelled, as on the text side (graphic_copper_unmeasured).

    `footprints` is (key, kipy footprint) pairs; `board.get_shapes()` returns
    no footprint child, so the footprint half reads each part's own shapes,
    which KiCad holds in board coordinates. `owner_ref` is the footprint key.
    Not probed against a live KiCad."""
    segments: List[Segment] = []

    def feeders():
        for d in (drawings or ()):
            yield d, ""
        for key, fp in (footprints or ()):
            try:
                npads = sum(1 for p in _fp_pads(fp)
                            if kipy_pad_type(p) != 'np_thru_hole')
            except Exception:
                npads = 0
            if not footprint_copper_is_functional(npads):
                continue
            for g in _kipy_fp_shapes(fp):
                yield g, key

    for d, owner in feeders():
        try:
            layer = _kipy_layer_token(getattr(d, "layer", None))
        except Exception:
            continue
        if not layer.endswith('.Cu'):
            continue
        kind = _kipy_shape_kind(d)
        try:
            w = _nm_to_mm(d.attributes.stroke.width)
        except Exception:
            w = 0.0
        try:
            filled = bool(d.attributes.fill.filled)
        except Exception:
            filled = False
        if kind in ("segment", "arc"):
            filled = False
        if w <= 0 and not filled:
            w = UNSTROKED_SHAPE_WIDTH
        try:
            nid, _nn = resolve_net(getattr(d, "net", None))
        except Exception:
            nid = 0
        uuid = _kipy_uuid(d)

        def emit_outline(pts, kind_='poly', circle=None):
            if len(pts) < 2:
                return
            ew = w if w > 0 else defaults.TRACK_WIDTH
            seq = list(pts) + [pts[0]]
            ring = tuple((float(x), float(y)) for x, y in pts) if filled else None
            for a, b in zip(seq, seq[1:]):
                if a == b:
                    continue
                segments.append(Segment(
                    start_x=a[0], start_y=a[1], end_x=b[0], end_y=b[1],
                    width=ew, layer=layer, net_id=nid, uuid=uuid, graphic=True,
                    owner_ref=owner, drawn_width=max(0.0, w), graphic_kind=kind_,
                    graphic_circle=circle, graphic_filled=filled,
                    graphic_ring=ring))

        try:
            if kind == "segment":
                a, b = _vec_to_xy_mm(d.start), _vec_to_xy_mm(d.end)
                segments.append(Segment(
                    start_x=a[0], start_y=a[1], end_x=b[0], end_y=b[1],
                    width=w, layer=layer, net_id=nid, uuid=uuid, graphic=True,
                    owner_ref=owner, drawn_width=w, graphic_kind='line'))
            elif kind == "arc":
                for p0, p1 in _arc_to_segments(_vec_to_xy_mm(d.start),
                                               _vec_to_xy_mm(d.mid),
                                               _vec_to_xy_mm(d.end)):
                    segments.append(Segment(
                        start_x=p0[0], start_y=p0[1], end_x=p1[0], end_y=p1[1],
                        width=w, layer=layer, net_id=nid, uuid=uuid,
                        graphic=True, owner_ref=owner, drawn_width=w,
                        graphic_kind='arc'))
            elif kind == "polygon":
                for pwh in (getattr(d, "polygons", None) or []):
                    emit_outline(_kipy_graphic_poly_points(
                        getattr(pwh, "outline", None)))
            elif kind == "rectangle":
                (x1, y1), (x2, y2) = (_vec_to_xy_mm(d.top_left),
                                      _vec_to_xy_mm(d.bottom_right))
                emit_outline([(x1, y1), (x2, y1), (x2, y2), (x1, y2)],
                             kind_='rect')
            elif kind == "circle":
                c, e = _vec_to_xy_mm(d.center), _vec_to_xy_mm(d.radius_point)
                r = math.hypot(e[0] - c[0], e[1] - c[1])
                emit_outline([(c[0] + r * math.cos(k * math.pi / 8),
                               c[1] + r * math.sin(k * math.pi / 8))
                              for k in range(16)],
                             kind_='circle', circle=(c[0], c[1], r))
        except Exception:
            continue
    return segments


_KIPY_ISLAND_MODES = {'IRM_ALWAYS': 0, 'IRM_NEVER': 1, 'IRM_AREA': 2}


def _kipy_zone_entries(z, resolve_net, in_footprint: bool = False) -> List[Zone]:
    """One Zone per COPPER layer of a kipy zone, as `extract_zones` and main's
    `_extract_zones_from_pcbnew` emit them, or [] when it is not copper.

    - A rule area is a keep-out, not copper: skipped (rule areas reach
      board_info.keepouts from the file, `extract_keepouts`). kipy's
      get_zones() returns them with every pour, and the reader this replaces
      modelled each as a net-0 pour on its layers.
    - A zone on no copper layer (a mask or paste opening drawn as a zone) is
      not copper either (da63ad5c).
    - The outline is pcbnew's (`kipy_outline_points`: arc vertices to the
      nanometre, fe367f3f; a closing repeat merged, 37fb1b93); every further
      ring is a hole (#1279) -- KiCad loads every `(polygon ...)` after the
      first as a hole of the first.
    - `teardrop` is KiCad's generated teardrop area (#1164): copper, never a
      pour. Priority, island removal, clearance, min thickness, uuid and
      `in_footprint` as the text parser reads them.
    """
    try:
        if z.is_rule_area():
            return []
    except Exception:
        pass
    names = []
    for bl in (getattr(z, "layers", None) or ()):
        n = _kipy_layer_token(bl)
        if n.endswith('.Cu') and n not in names:
            names.append(n)
    if not names:
        return []
    pwhs = []
    try:
        from kipy.geometry import PolygonWithHoles
        pwhs = [PolygonWithHoles(proto_ref=p) for p in z.proto.outline.polygons]
    except Exception:
        try:
            pwhs = [z.outline]
        except Exception:
            pwhs = []
    if not pwhs:
        return []
    polygon = kipy_outline_points(getattr(pwhs[0], "outline", None), closed=False)
    if not polygon:
        return []
    rings = list(getattr(pwhs[0], "holes", None) or [])
    for extra in pwhs[1:]:
        rings.append(getattr(extra, "outline", None))
        rings.extend(getattr(extra, "holes", None) or [])
    holes = [h for h in (kipy_outline_points(r, closed=False) for r in rings)
             if len(h) >= 3]
    net_id, net_name = resolve_net(getattr(z, "net", None))
    try:
        priority = int(getattr(z, "priority", 0) or 0)
    except Exception:
        priority = 0
    island_mode = 0
    island_area = 0.0
    clearance = min_thickness = None
    teardrop = False
    # #1329: how the pour joins its pads, and its spoke geometry, as the text
    # parser's zone_connection_settings reads them (None = unknown).
    pad_connection = thermal_gap = thermal_bridge_width = None
    try:
        _conn = z.proto.copper_settings.connection
        pad_connection = kipy_zone_connect_name(_conn.zone_connection)
        if _conn.HasField('thermal_spokes'):
            _sp = _conn.thermal_spokes
            if _sp.HasField('gap'):
                thermal_gap = _nm_to_mm(_sp.gap.value_nm)
            if _sp.HasField('width'):
                thermal_bridge_width = _nm_to_mm(_sp.width.value_nm)
    except Exception:
        pass
    try:
        from kipy.proto.board.board_types_pb2 import (IslandRemovalMode,
                                                       TeardropType)
        island_mode = _KIPY_ISLAND_MODES.get(
            _kipy_enum_name(IslandRemovalMode, z.island_mode), 0)
        island_area = float(z.min_island_area or 0) / 1e12   # nm^2 -> mm^2
        if z.clearance is not None:
            clearance = _nm_to_mm(z.clearance)
        if z.min_thickness is not None:
            min_thickness = _nm_to_mm(z.min_thickness)
        td = z.teardrop
        teardrop = td is not None and _kipy_enum_name(TeardropType, td.type) in (
            'TDT_VIA_PAD', 'TDT_TRACK_END')
    except Exception:
        pass
    uuid = _kipy_uuid(z)
    return [Zone(net_id=net_id, net_name=net_name, layer=n, polygon=polygon,
                 uuid=uuid, priority=priority, island_removal_mode=island_mode,
                 island_area_min=island_area, clearance=clearance,
                 min_thickness=min_thickness, pad_connection=pad_connection,
                 thermal_gap=thermal_gap,
                 thermal_bridge_width=thermal_bridge_width,
                 in_footprint=in_footprint, teardrop=teardrop, holes=holes)
            for n in names]


def _extract_zones_kipy(board, get_layer_name, resolve_net,
                        footprints=()) -> List[Zone]:
    """Every copper zone of a kipy board (`_kipy_zone_entries`): the board's
    own and, as the text parser reads them (#478, `in_footprint`), the zones a
    footprint carries -- which `board.get_zones()` does not return. A
    footprint zone whose uuid the board listing already gave is not read
    twice. `resolve_net` is the builder's name->synthetic-id resolver; without
    it zones would mismatch the rest of PCBData's net ids. `get_layer_name` is
    kept for the call shape; layers are named by `_kipy_layer_token`."""
    zones: List[Zone] = []
    seen = set()
    try:
        zone_iter = list(board.get_zones())
    except Exception:
        zone_iter = []
    for z in zone_iter:
        try:
            got = _kipy_zone_entries(z, resolve_net)
        except Exception:
            continue
        zones.extend(got)
        u = _kipy_uuid(z)
        if u:
            seen.add(u)
    for _key, fp in (footprints or ()):
        try:
            items = list(fp.definition.items)
        except Exception:
            continue
        for it in items:
            if type(it).__name__ != "Zone":
                continue
            u = _kipy_uuid(it)
            if u and u in seen:
                continue
            try:
                zones.extend(_kipy_zone_entries(it, resolve_net, in_footprint=True))
            except Exception:
                continue
    return zones


def _global_to_local(fp_x, fp_y, fp_rotation_deg, global_x, global_y):
    """Reverse transform: global board coordinates to local footprint coordinates.

    The exact inverse of `local_to_global` (whose forward negates the angle,
    the KiCad rotation convention): local = R(+rotation) . (global - origin).
    The previous body applied the TRANSPOSED rotation (both sin signs
    flipped), so round-tripping a point through local_to_global came back
    reflected for any rotated footprint -- measured 12.15mm off on a -90 part
    (#481 label placement). That was latent until KiCad 7+ removed
    pad.GetPos0(), which makes the pad-local fallback below run for EVERY pad
    under a modern pcbnew.
    """
    rad = math.radians(fp_rotation_deg)  # POSITIVE angle reverses the negated forward
    cos_r = math.cos(rad)
    sin_r = math.sin(rad)

    dx = global_x - fp_x
    dy = global_y - fp_y

    local_x = dx * cos_r - dy * sin_r
    local_y = dx * sin_r + dy * cos_r

    return local_x, local_y


def _polygon_union_excess(polys_a, polys_b):
    """How far the copper `polys_a` describes reaches OUTSIDE the copper
    `polys_b` describes: (distance mm, (x, y)) at the worst sample.

    Each argument is a pad's `polygons`, read the way every consumer reads it
    -- the UNION of its rings, each ring even-odd. The two parse paths spell
    one pad differently (pcbnew merges a custom pad into one outline, the text
    parser keeps one ring per primitive plus the anchor), so the rings cannot
    be compared one to one; their unions can. Samples are every vertex of
    `polys_a` and the midpoint of every edge, which sees a protruding cap or
    corner and a missing primitive (its vertices are outside); a notch cut
    into the middle of a long edge between two samples is the one shape this
    can miss."""
    from check_drc import _point_in_poly as _pip
    from geometry_utils import point_to_segment_distance as _psd
    rings_b = [r for r in polys_b if len(r) >= 3]
    boxes_b = [(min(p[0] for p in r), min(p[1] for p in r),
                max(p[0] for p in r), max(p[1] for p in r)) for r in rings_b]
    worst, where = 0.0, None
    for ring in polys_a:
        n = len(ring)
        if n < 2:
            continue
        samples = list(ring) + [((ring[i][0] + ring[(i + 1) % n][0]) / 2.0,
                                 (ring[i][1] + ring[(i + 1) % n][1]) / 2.0)
                                for i in range(n)]
        for x, y in samples:
            if any(bx0 <= x <= bx1 and by0 <= y <= by1 and _pip(x, y, r)
                   for r, (bx0, by0, bx1, by1) in zip(rings_b, boxes_b)):
                continue
            d = min((_psd(x, y, r[i][0], r[i][1], r[(i + 1) % len(r)][0],
                          r[(i + 1) % len(r)][1])
                     for r in rings_b for i in range(len(r))), default=float('inf'))
            if d > worst:
                worst, where = d, (x, y)
    return worst, where


def _pair_nearest_bounds(items_a, items_b):
    """Pair two equal-length lists of objects carrying `.bounds` by GEOMETRY:
    an exact match on the bounds rounded to 0.1 um first (the common case,
    and linear), then each leftover with its nearest remaining partner by the
    largest per-coordinate difference. Returns the list of (a, b) pairs.

    A comparator that sorts both lists and zips them pairs by RANK, and two
    float spellings of one coordinate (119.950866 vs 119.9508655) are enough
    to re-rank a row of identical-size openings."""
    def key(o):
        return tuple(round(v, 4) for v in o.bounds)
    pool = {}
    for b in items_b:
        pool.setdefault(key(b), []).append(b)
    pairs, left_a = [], []
    for a in items_a:
        hit = pool.get(key(a))
        if hit:
            pairs.append((a, hit.pop()))
        else:
            left_a.append(a)
    left_b = [b for bs in pool.values() for b in bs]
    for a in left_a:
        if not left_b:
            break
        j = min(range(len(left_b)), key=lambda k: max(
            abs(x - y) for x, y in zip(a.bounds, left_b[k].bounds)))
        pairs.append((a, left_b.pop(j)))
    return pairs


def compare_pcb_data(from_board: 'PCBData', from_file: 'PCBData', tolerance: float = 0.01) -> List[str]:
    """Compare two PCBData objects and return list of differences.

    Useful for validating that build_pcb_data_from_board() produces the same
    results as parse_kicad_pcb().

    Args:
        from_board: PCBData built from the live KiCad board over IPC
        from_file: PCBData parsed from .kicad_pcb file
        tolerance: Position tolerance in mm for coordinate comparisons

    Returns:
        List of difference description strings (empty if identical)
    """
    diffs = []

    def close(a, b):
        return abs(a - b) < tolerance

    # --- Compare board info ---
    bi_b = from_board.board_info
    bi_f = from_file.board_info
    if set(bi_b.copper_layers) != set(bi_f.copper_layers):
        diffs.append(f"Copper layers differ: board={bi_b.copper_layers} file={bi_f.copper_layers}")

    if bi_b.board_bounds and bi_f.board_bounds:
        for i, label in enumerate(['min_x', 'min_y', 'max_x', 'max_y']):
            if not close(bi_b.board_bounds[i], bi_f.board_bounds[i]):
                diffs.append(f"Board bounds {label}: board={bi_b.board_bounds[i]:.3f} file={bi_f.board_bounds[i]:.3f}")
    elif bi_b.board_bounds != bi_f.board_bounds:
        diffs.append(f"Board bounds: board={bi_b.board_bounds} file={bi_f.board_bounds}")

    if len(bi_b.stackup) != len(bi_f.stackup):
        diffs.append(f"Stackup layer count: board={len(bi_b.stackup)} file={len(bi_f.stackup)}")
    # Layer by layer (#1222): the impedance width solver and the via barrel
    # length read thickness, epsilon_r and loss_tangent, so a front that
    # combines a multi-sheet dielectric or reads a locked thickness differently
    # sizes different tracks. Thickness to 1 nm (pcbnew holds it in integer
    # nm); the two dielectric constants to 1e-6, well above float noise and
    # far below any declared value's precision.
    for i, (sb, sf) in enumerate(zip(bi_b.stackup, bi_f.stackup)):
        if (sb.name, sb.layer_type) != (sf.name, sf.layer_type):
            diffs.append(f"Stackup layer {i}: board={sb.name!r} ({sb.layer_type}) "
                         f"file={sf.name!r} ({sf.layer_type})")
            continue
        for _attr, _tol in (('thickness', 1e-6), ('epsilon_r', 1e-6),
                            ('loss_tangent', 1e-6)):
            vb, vf = getattr(sb, _attr), getattr(sf, _attr)
            if abs(vb - vf) > _tol:
                diffs.append(f"Stackup layer {i} {sb.name!r} {_attr}: "
                             f"board={vb!r} file={vf!r}")

    # --- Compare nets (net 0 = unconnected pseudo-net; one path may list it) ---
    board_net_ids = set(from_board.nets.keys()) - {0}
    file_net_ids = set(from_file.nets.keys()) - {0}
    if board_net_ids != file_net_ids:
        only_board = board_net_ids - file_net_ids
        only_file = file_net_ids - board_net_ids
        if only_board:
            diffs.append(f"Nets only in board: {sorted(only_board)[:10]}{'...' if len(only_board) > 10 else ''}")
        if only_file:
            diffs.append(f"Nets only in file: {sorted(only_file)[:10]}{'...' if len(only_file) > 10 else ''}")

    for net_id in board_net_ids & file_net_ids:
        bn = from_board.nets[net_id]
        fn = from_file.nets[net_id]
        if bn.name != fn.name:
            diffs.append(f"Net {net_id} name: board='{bn.name}' file='{fn.name}'")
        if len(bn.pads) != len(fn.pads):
            diffs.append(f"Net {net_id} '{bn.name}' pad count: board={len(bn.pads)} file={len(fn.pads)}")

    # --- Compare footprints ---
    # #726: name the duplicate-reference disagreement DIRECTLY. Both paths
    # derive their keys with `disambiguate_references` over their own ordered
    # footprint list, so a KiCad whose `GetFootprints()` no longer iterates in
    # file order shows up below as a pile of position mismatches -- true, but it
    # reads as a parser bug rather than as the ordering change it is.
    #
    # Compared as a key -> uuid MAP, not as `duplicate_references`. That dict is
    # order-INVARIANT -- a permutation of the twins leaves it byte-identical --
    # so the obvious version of this check cannot fire on the one thing it
    # exists to name. (Measured: swapping two twins produced 8 footprint diffs
    # and 0 duplicate-reference lines.)
    if from_board.duplicate_references != from_file.duplicate_references:
        diffs.append(
            f"Duplicate references disagree: board={from_board.duplicate_references} "
            f"file={from_file.duplicate_references}")
    _dup_names = set(from_board.duplicate_references) | set(from_file.duplicate_references)
    if _dup_names:
        def _twin_uuids(pcb):
            return {k: getattr(f, 'uuid', '') for k, f in pcb.footprints.items()
                    if k.split(DUP_REF_SEP)[0] in _dup_names}
        bu, fu = _twin_uuids(from_board), _twin_uuids(from_file)
        moved = sorted(k for k in set(bu) & set(fu) if bu[k] != fu[k])
        if moved:
            diffs.append(
                f"Duplicate-reference ORDINALS name different blocks: {moved[:6]}. "
                f"The two parse paths enumerated the footprints in different "
                f"orders, so their `{DUP_REF_SEP}n` suffixes landed on different "
                f"parts (see disambiguate_references).")

    board_refs = set(from_board.footprints.keys())
    file_refs = set(from_file.footprints.keys())
    if board_refs != file_refs:
        only_board = board_refs - file_refs
        only_file = file_refs - board_refs
        if only_board:
            diffs.append(f"Footprints only in board: {sorted(only_board)[:10]}")
        if only_file:
            diffs.append(f"Footprints only in file: {sorted(only_file)[:10]}")

    for ref in sorted(board_refs & file_refs):
        bf = from_board.footprints[ref]
        ff = from_file.footprints[ref]

        if not close(bf.x, ff.x) or not close(bf.y, ff.y):
            diffs.append(f"Footprint {ref} position: board=({bf.x:.3f},{bf.y:.3f}) file=({ff.x:.3f},{ff.y:.3f})")
        if not close(bf.rotation % 360, ff.rotation % 360):
            diffs.append(f"Footprint {ref} rotation: board={bf.rotation:.1f} file={ff.rotation:.1f}")
        # #1054: the GUI's placement Apply sets each live lock from the TEXT
        # parse of the result board, and every placement step decides what it
        # may move from this field -- so the two paths must agree on it.
        if bool(getattr(bf, 'locked', False)) != bool(getattr(ff, 'locked', False)):
            diffs.append(f"Footprint {ref} locked: board={bf.locked} file={ff.locked}")
        # Footprint-level clearance override (#326): a divergence here shifts
        # EVERY inheriting pad's resolved local_clearance between the two paths.
        bfc = getattr(bf, 'clearance', 0.0); ffc = getattr(ff, 'clearance', 0.0)
        if not close(bfc, ffc):
            diffs.append(f"Footprint {ref} clearance: board={bfc:.3f} file={ffc:.3f}")
        # Footprint-level zone connection: inherited by every pad without its own.
        bzc = getattr(bf, 'zone_connection', None); fzc = getattr(ff, 'zone_connection', None)
        if bzc != fzc:
            diffs.append(f"Footprint {ref} zone_connection: board={bzc} file={fzc}")
        # #1211: the schematic sheet, which `place_pose replicate` resolves a
        # channel by (name) and maps its parts by (path). A pcbnew accessor
        # that moved would hand the GUI a board with no sheets.
        for _sheet in ('sheet_path', 'sheet_name', 'sheet_file'):
            bs = getattr(bf, _sheet, ''); fs = getattr(ff, _sheet, '')
            if bs != fs:
                diffs.append(f"Footprint {ref} {_sheet}: board={bs!r} file={fs!r}")
        # Net-tie pad groups (#328): the pcbnew path wraps GetNetTiePadGroups() in a
        # broad try/except -> [], so a KiCad build that drops them would silently give
        # the GUI zero net-ties (phantom shorts / refused tie escapes) while the text
        # parser has them. Compare as order-insensitive sets of frozensets.
        bnt = {frozenset(g) for g in (getattr(bf, 'net_tie_groups', None) or [])}
        fnt = {frozenset(g) for g in (getattr(ff, 'net_tie_groups', None) or [])}
        if bnt != fnt:
            diffs.append(f"Footprint {ref} net_tie_groups: board={sorted(map(sorted, bnt))} "
                         f"file={sorted(map(sorted, fnt))}")
        if len(bf.pads) != len(ff.pads):
            diffs.append(f"Footprint {ref} pad count: board={len(bf.pads)} file={len(ff.pads)}")
        else:
            # Pair pads by POSITION, not pad_number: pad numbers are not unique
            # (empty "" thermal/mechanical pads, repeated connector pads), so
            # sorting+zipping on pad_number misaligns a roundrect pad against a
            # circle pad and cascades into spurious diffs. Position is unique per
            # pad and identical (to ~1um) between the board and file parses.
            def _pad_key(p):
                # Include layers: a footprint can stack pads at one position+number
                # (separate copper / paste / mask apertures), so position+number
                # alone is not unique and would pair non-deterministically.
                return (round(p.global_x, 3), round(p.global_y, 3), p.pad_number,
                        tuple(sorted(p.layers)))
            b_pads = sorted(bf.pads, key=_pad_key)
            f_pads = sorted(ff.pads, key=_pad_key)
            # Sub-micron board/file coordinate differences can straddle the
            # rounding above (e.g. -11.7875 -> -11.787 vs -11.788) and scramble
            # the zip pairing into cascade mismatches (crkbd EXSW). Re-pair by
            # nearest same-number pad within a small tolerance first; anything
            # left over keeps the sorted order.
            unmatched_f = list(f_pads)
            pairs = []
            leftovers_b = []
            for bp in b_pads:
                best = None
                best_d = 0.02  # mm; well above float noise, below any pitch
                for fp2 in unmatched_f:
                    if fp2.pad_number != bp.pad_number:
                        continue
                    d = max(abs(fp2.global_x - bp.global_x), abs(fp2.global_y - bp.global_y))
                    if d < best_d:
                        best, best_d = fp2, d
                if best is not None:
                    unmatched_f.remove(best)
                    pairs.append((bp, best))
                else:
                    leftovers_b.append(bp)
            pairs.extend(zip(leftovers_b, unmatched_f))
            for bp, fp in pairs:
                if not close(bp.global_x, fp.global_x) or not close(bp.global_y, fp.global_y):
                    diffs.append(f"Footprint {ref} pad pairing mismatch (position): "
                                 f"board {bp.pad_number}@({bp.global_x:.3f},{bp.global_y:.3f}) "
                                 f"vs file {fp.pad_number}@({fp.global_x:.3f},{fp.global_y:.3f})")
                    continue
                if bp.pad_number != fp.pad_number:
                    diffs.append(f"Pad @({bp.global_x:.3f},{bp.global_y:.3f}) number: board={bp.pad_number} file={fp.pad_number}")
                if bp.net_id != fp.net_id:
                    diffs.append(f"Pad {ref}:{bp.pad_number} net_id: board={bp.net_id} file={fp.net_id}")
                if bp.shape != fp.shape:
                    diffs.append(f"Pad {ref}:{bp.pad_number} shape: board={bp.shape} file={fp.shape}")
                if not close(bp.size_x, fp.size_x) or not close(bp.size_y, fp.size_y):
                    diffs.append(f"Pad {ref}:{bp.pad_number} size: board=({bp.size_x:.3f},{bp.size_y:.3f}) file=({fp.size_x:.3f},{fp.size_y:.3f})")
                # Residual rect tilt - the obstacle/DRC geometry rotates the pad
                # rectangle by this, so a board/file mismatch (e.g. a custom or
                # diagonal pad modelled differently by the two paths) changes the
                # modelled copper footprint.
                br = getattr(bp, 'rect_rotation', 0.0); fr = getattr(fp, 'rect_rotation', 0.0)
                if not close(br, fr):
                    diffs.append(f"Pad {ref}:{bp.pad_number} rect_rotation: board={br:.2f} file={fr:.2f}")
                # A custom pad's real copper (#188): the obstacle map and
                # check_drc read `polygons` before the size rect, and nothing
                # compared them. Compared as the UNION each path describes,
                # both ways (`_polygon_union_excess`), since pcbnew merges the
                # primitives into one ring and the text parser keeps one per
                # primitive.
                bpp = getattr(bp, 'polygons', None) or []
                fpp = getattr(fp, 'polygons', None) or []
                if bool(bpp) != bool(fpp):
                    diffs.append(f"Pad {ref}:{bp.pad_number} polygons: board={len(bpp)} "
                                 f"ring(s) file={len(fpp)} ring(s)")
                elif bpp:
                    for _who, _a, _b in (('board', bpp, fpp), ('file', fpp, bpp)):
                        _d, _at = _polygon_union_excess(_a, _b)
                        if _d >= tolerance:
                            diffs.append(
                                f"Pad {ref}:{bp.pad_number} polygons: {_who} copper reaches "
                                f"{_d:.3f} mm past the other path's at "
                                f"({_at[0]:.3f},{_at[1]:.3f})")
                # Per-pad local clearance override (fiducial keep-clear rings etc.).
                # A pcbnew accessor that silently returns 0 would drop a keep-out the
                # file parse honors - this surfaces that divergence.
                bc = getattr(bp, 'local_clearance', 0.0); fc = getattr(fp, 'local_clearance', 0.0)
                if not close(bc, fc):
                    diffs.append(f"Pad {ref}:{bp.pad_number} local_clearance: board={bc:.3f} file={fc:.3f}")
                # Zone connection override: 'none' keeps the pour off the pad,
                # so a path that dropped it would credit pour contact KiCad
                # never makes.
                bzc = getattr(bp, 'zone_connection', None); fzc = getattr(fp, 'zone_connection', None)
                if bzc != fzc:
                    diffs.append(f"Pad {ref}:{bp.pad_number} zone_connection: board={bzc} file={fzc}")
                # Roundrect corner ratio - only consumed (and only meaningful) for
                # roundrect pads; pcbnew returns a default ratio on every pad shape,
                # so comparing it on circle/rect pads is pure noise.
                if bp.shape == 'roundrect' and fp.shape == 'roundrect':
                    brr = getattr(bp, 'roundrect_rratio', 0.0); frr = getattr(fp, 'roundrect_rratio', 0.0)
                    if not close(brr, frr):
                        diffs.append(f"Pad {ref}:{bp.pad_number} roundrect_rratio: board={brr:.3f} file={frr:.3f}")
                if not close(bp.drill, fp.drill):
                    diffs.append(f"Pad {ref}:{bp.pad_number} drill: board={bp.drill:.3f} file={fp.drill:.3f}")
                # #1248: pad-to-die length (mm) / delay (ps). A getter that read
                # 0 would silently drop the term every length matcher adds.
                for _die in ('die_length', 'die_delay'):
                    bd = getattr(bp, _die, 0.0); fd = getattr(fp, _die, 0.0)
                    if abs(bd - fd) > 1e-6:
                        diffs.append(f"Pad {ref}:{bp.pad_number} {_die}: board={bd:.6f} file={fd:.6f}")
                # Compare layers (as sets since order may differ)
                if set(bp.layers) != set(fp.layers):
                    diffs.append(f"Pad {ref}:{bp.pad_number} layers: board={bp.layers} file={fp.layers}")

    # --- Compare segments (geometry, not just count) ---
    # Existing tracks are routing obstacles, so a position/width/layer/net
    # divergence changes what the router sees. Match as a multiset of canonical
    # signatures (endpoint order normalised, coords quantised to ~1um) since the
    # board and file orderings differ.
    def _q(v):
        return round(v, 3)

    def _net_label(pcb, nid):
        # Compare by NAME, not raw id: for nets carried only by copper
        # GRAPHICS (#337, no pads/tracks anchor them), pcbnew's renumbered
        # code and the file's net-table id legitimately differ (openstint
        # /A-: board 3 vs file 14) while naming the same net.
        # Net 0 is "no net" on both, whether or not a path lists it (a KiCad
        # 9 file without the net-0 entry: apple1_aci_card's 2,363 unnetted
        # tracks read '' on one side and 0 on the other).
        if not nid:
            return ''
        n = pcb.nets.get(nid)
        return n.name if n is not None else nid

    def _seg_sig_for(pcb):
        def _seg_sig(s):
            ends = tuple(sorted([(_q(s.start_x), _q(s.start_y)), (_q(s.end_x), _q(s.end_y))]))
            if getattr(s, 'graphic', False):
                # KiCad RECOMPUTES a copper graphic's net from connectivity on
                # load (openstint: file attribute /A-, pcbnew says GND for the
                # same art). Same copper either way -- compare geometry only.
                # `owner_ref` IS compared (#908): it is not recomputed by
                # KiCad, both paths derive it from the same disambiguator, and
                # it is what scopes the own-pad obstacle lift -- so the two
                # fronts disagreeing about it is exactly the drift this
                # comparator exists to catch.
                # #962: drawn_width / graphic_kind / graphic_filled are
                # compared too. The off-outline grade measures from them, so a
                # front that reads a different stroke or fill grades a
                # different overrun.
                # #1181: and the filled interior, `graphic_ring` -- the
                # obstacle map stamps it and check_drc grades inside it, so a
                # front reading a different ring routes around different
                # copper.
                _dw = getattr(s, 'drawn_width', None)
                _ring = getattr(s, 'graphic_ring', None)
                return (ends, _q(s.width), s.layer, '<graphic>',
                        getattr(s, 'owner_ref', ''),
                        None if _dw is None else _q(_dw),
                        getattr(s, 'graphic_kind', ''),
                        bool(getattr(s, 'graphic_filled', False)),
                        None if _ring is None
                        else tuple((_q(x), _q(y)) for x, y in _ring))
            return (ends, _q(s.width), s.layer, _net_label(pcb, s.net_id))
        return _seg_sig

    def _multiset_diff_2sig(board_items, file_items, sig_b, sig_f, label, fmt):
        from collections import Counter
        cb = Counter(sig_b(x) for x in board_items)
        cf = Counter(sig_f(x) for x in file_items)
        only_board = list((cb - cf).elements())
        only_file = list((cf - cb).elements())
        if only_board or only_file:
            diffs.append(f"{label} count: board={len(board_items)} file={len(file_items)}; "
                         f"{len(only_board)} only in board, {len(only_file)} only in file")
            for s in only_board[:5]:
                diffs.append(f"  {label} only in board: {fmt(s)}")
            for s in only_file[:5]:
                diffs.append(f"  {label} only in file: {fmt(s)}")

    def _multiset_diff(board_items, file_items, sig, label, fmt):
        from collections import Counter
        cb = Counter(sig(x) for x in board_items)
        cf = Counter(sig(x) for x in file_items)
        only_board = list((cb - cf).elements())
        only_file = list((cf - cb).elements())
        if only_board or only_file:
            diffs.append(f"{label} count: board={len(board_items)} file={len(file_items)}; "
                         f"{len(only_board)} only in board, {len(only_file)} only in file")
            for s in only_board[:5]:
                diffs.append(f"  {label} only in board: {fmt(s)}")
            for s in only_file[:5]:
                diffs.append(f"  {label} only in file: {fmt(s)}")

    _multiset_diff_2sig(from_board.segments, from_file.segments,
                        _seg_sig_for(from_board), _seg_sig_for(from_file), "Segment",
                        lambda s: f"ends={s[0]} w={s[1]} layer={s[2]} net={s[3]}")

    # --- Compare vias (geometry, not just count) ---
    # The pair IN ORDER and the type: a consumer keys on both (placement
    # Apply's via key is the ordered pair), so a sorted pair hid the paths
    # disagreeing on a via written bottom first.
    def _via_sig(v):
        return (_q(v.x), _q(v.y), _q(v.size), _q(v.drill), v.net_id, tuple(v.layers),
                getattr(v, 'via_type', 'through'))

    _multiset_diff(from_board.vias, from_file.vias, _via_sig, "Via",
                   lambda v: f"({v[0]},{v[1]}) size={v[2]} drill={v[3]} net={v[4]} "
                             f"layers={list(v[5])} type={v[6]}")

    # --- Compare zones, as a multiset per (net, layer) ---
    # Pairing the two lists by sort order on (net_id, layer) was ambiguous
    # wherever two zones share a net and a layer -- a pour and the teardrops on
    # its tracks (#1164), or two pours -- so a disagreement could land on the
    # wrong partner, or two could cancel. Each (net, layer) group is compared as
    # a multiset of what a consumer reads: the teardrop flag (#1164: it decides
    # `pour_zones` membership -- rip-eligibility, the --force-reroute skip, the
    # plane-finalize hand-off, the GUI's plane suggestions), the vertex count,
    # and the fill semantics (#350: the priority-exact fill-point rejection and
    # the island-patch gating diverge CLI vs GUI without them).
    # island_area_min is only meaningful in mode 2: pcbnew's GetMinIslandArea()
    # reports the C++ default (10mm^2) even when the file carries no token
    # (mode 0/1 zones), so comparing it unconditionally manufactured phantom
    # parity diffs (free_dap).
    if len(from_board.zones) != len(from_file.zones):
        diffs.append(f"Zone count: board={len(from_board.zones)} file={len(from_file.zones)}")

    # The outline itself, on pcbnew's nm grid: both fronts hold the vertices
    # pcbnew loads (an `(arc ...)` vertex is KiCad's own polyline on both,
    # `zone_outline_points`), so they agree to the nanometre or not at all.
    def _zone_sig(z):
        mode = getattr(z, 'island_removal_mode', 0)
        return (bool(getattr(z, 'teardrop', False)), len(z.polygon),
                getattr(z, 'priority', 0), mode,
                round(getattr(z, 'island_area_min', 0.0), 3) if mode == 2 else None,
                tuple((_kiround(x * 1e6), _kiround(y * 1e6)) for x, y in z.polygon),
                # #1279: the cut-outs, the same way.
                tuple(tuple((_kiround(x * 1e6), _kiround(y * 1e6)) for x, y in h)
                      for h in (getattr(z, 'holes', None) or ())))

    def _zone_fmt(s):
        import zlib
        return (f"[{'teardrop' if s[0] else 'pour'} vertices={s[1]} priority={s[2]} "
                f"island_mode={s[3]}" + (f" island_area_min={s[4]}" if s[3] == 2 else '')
                + f" outline=#{zlib.crc32(repr(s[5]).encode()):08x}"
                + (f" holes={len(s[6])}#{zlib.crc32(repr(s[6]).encode()):08x}" if s[6] else '')
                + "]")

    from collections import Counter as _Counter
    _zb, _zf = {}, {}
    for _z in from_board.zones:
        _zb.setdefault((_z.net_id, _z.layer), _Counter())[_zone_sig(_z)] += 1
    for _z in from_file.zones:
        _zf.setdefault((_z.net_id, _z.layer), _Counter())[_zone_sig(_z)] += 1
    for _key in sorted(set(_zb) | set(_zf), key=lambda k: (k[0], str(k[1]))):
        cb, cf = _zb.get(_key, _Counter()), _zf.get(_key, _Counter())
        if cb == cf:
            continue
        net_id, layer = _key
        td_b = sum(n for s, n in cb.items() if s[0])
        td_f = sum(n for s, n in cf.items() if s[0])
        if td_b != td_f:
            diffs.append(f"Zone net={net_id} layer={layer} teardrop count: "
                         f"board={td_b} file={td_f}")
        only_b = sorted((cb - cf).elements())
        only_f = sorted((cf - cb).elements())
        diffs.append(
            f"Zone net={net_id} layer={layer}: {len(only_b)} only in board "
            f"{' '.join(map(_zone_fmt, only_b[:3]))}{' ...' if len(only_b) > 3 else ''}; "
            f"{len(only_f)} only in file "
            f"{' '.join(map(_zone_fmt, only_f[:3]))}{' ...' if len(only_f) > 3 else ''}")

    # --- Compare board outline / cutouts (used for edge & cutout obstacles) ---
    bo_b = bi_b.board_outline or []
    bo_f = bi_f.board_outline or []
    if len(bo_b) != len(bo_f):
        diffs.append(f"Board outline vertex count: board={len(bo_b)} file={len(bo_f)}")
    cut_b = bi_b.board_cutouts or []
    cut_f = bi_f.board_cutouts or []
    if len(cut_b) != len(cut_f):
        diffs.append(f"Board cutout count: board={len(cut_b)} file={len(cut_f)}")
    # Multi-outline boards (#304): both sides must agree on the OUTER ring set
    outs_b = sorted(len(o) for o in (getattr(bi_b, 'board_outlines', None) or []))
    outs_f = sorted(len(o) for o in (getattr(bi_f, 'board_outlines', None) or []))
    if outs_b != outs_f:
        diffs.append(f"Board outer-ring set: board={outs_b} file={outs_f}")
    # Reclassified milled inner contours (#505): they drive edge clearance and
    # the edge keep-out band, so a side that misses one routes copper onto the
    # mill line while the other does not.
    ec_b = sorted(len(o) for o in (getattr(bi_b, 'board_edge_contours', None) or []))
    ec_f = sorted(len(o) for o in (getattr(bi_f, 'board_edge_contours', None) or []))
    if ec_b != ec_f:
        diffs.append(f"Board milled inner-contour set: board={ec_b} file={ec_f}")

    # --- Compare keepout zones (routing obstacles) ---
    kz_b = from_board.keepout_zones or []
    kz_f = from_file.keepout_zones or []
    if len(kz_b) != len(kz_f):
        diffs.append(f"Keepout zone count: board={len(kz_b)} file={len(kz_f)}")

    # --- Compare guide paths (used by guided routing) ---
    gp_b = from_board.guide_paths or []
    gp_f = from_file.guide_paths or []
    if len(gp_b) != len(gp_f):
        diffs.append(f"Guide path count: board={len(gp_b)} file={len(gp_f)}")

    # --- #962: the paste stencil and the via-protection policy ---
    # Both fronts feed one builder (paste_apertures.build_paste_apertures), so
    # a disagreement here is a disagreement in what each READ: the overrides,
    # the setup, or the paste graphics. Compared by identity plus bounds to the
    # comparator's own tolerance; tests/gui_parity/test_962_paste_parity.py is
    # the tighter gate, and it also checks against pcbnew's own margin.
    if not (close(bi_b.pad_to_paste_clearance, bi_f.pad_to_paste_clearance)
            and close(bi_b.pad_to_paste_clearance_ratio,
                      bi_f.pad_to_paste_clearance_ratio)):
        diffs.append(
            f"Board paste setup: board=({bi_b.pad_to_paste_clearance}, "
            f"{bi_b.pad_to_paste_clearance_ratio}) file=("
            f"{bi_f.pad_to_paste_clearance}, {bi_f.pad_to_paste_clearance_ratio})")
    if bi_b.via_protection_setup != bi_f.via_protection_setup:
        diffs.append(f"Via protection setup: board={bi_b.via_protection_setup} "
                     f"file={bi_f.via_protection_setup}")
    # The copper-to-hole floor each front prices NPTH holes at (#1038's via
    # keep-out turns on above 0). The board side reads pcbnew's in-memory
    # rule, the file side the project; on a board pcbnew just loaded from
    # that file they must declare the same floor, including pcbnew's factory
    # stand-in on a project-less board (obstacle_map.declared_hole_clearance).
    # Compared exactly (a rule value, not a geometry), and never skipped in
    # silence: a comparison that could not run is reported as one.
    try:
        from obstacle_map import declared_hole_clearance
        _hc_b = declared_hole_clearance(from_board)
        _hc_f = declared_hole_clearance(from_file)
        if abs(_hc_b[0] - _hc_f[0]) > 1e-9:
            diffs.append(f"Copper-to-hole floor: board={_hc_b[0]} ({_hc_b[1] or 'none'}) "
                         f"file={_hc_f[0]} ({_hc_f[1] or 'none'})")
    except ImportError as _e:
        diffs.append(f"Copper-to-hole floor: not compared ({type(_e).__name__}: {_e})")
    # The kicad_routing_tools records each front reads: the live board's
    # (its project's copy and the copy on the board) against the file's (the
    # project's copy and the board file's). Freshly loaded, they are the same.
    _rb_p = getattr(from_board, 'live_records_provider', None)
    _src = getattr(from_file, 'source_path', '') or ''
    if callable(_rb_p) and _src:
        try:
            import tool_records
            _rb, _rf = _rb_p() or {}, tool_records.effective_namespace(_src)
            if _rb != _rf:
                diffs.append(f"Tool records: board={sorted(_rb)} rev "
                             f"{tool_records.revision(_rb)} file={sorted(_rf)} "
                             f"rev {tool_records.revision(_rf)}")
        except ImportError as _e:
            diffs.append(f"Tool records: not compared ({type(_e).__name__}: {_e})")

    def _ap_sig(pcb, ap):
        nm = pcb.nets.get(ap.net_id)
        return (ap.owner_ref, ap.layer, ap.source, ap.pad_number,
                nm.name if nm else '')
    from collections import Counter as _Counter
    _cb = _Counter(_ap_sig(from_board, a) for a in from_board.paste_apertures or [])
    _cf = _Counter(_ap_sig(from_file, a) for a in from_file.paste_apertures or [])
    if _cb != _cf:
        diffs.append(f"Paste apertures: only board={sorted((_cb - _cf).elements())[:6]} "
                     f"only file={sorted((_cf - _cb).elements())[:6]}")
    else:
        # Pair the openings WITHIN one signature by geometry, never by a sort
        # of the raw bounds: the openings of one QFN windowpane or one
        # connector's paste graphics share every signature field, and their
        # bounds agree between the paths only to float noise (119.950866 vs
        # 119.9508655), which re-orders a sort and paired the board's opening
        # #1 against the file's #4 -- 506 phantom diffs over 31 boards whose
        # two sets were the same rectangles.
        def _by_sig(pcb):
            out = {}
            for a in pcb.paste_apertures or []:
                out.setdefault(_ap_sig(pcb, a), []).append(a)
            return out
        _gb, _gf = _by_sig(from_board), _by_sig(from_file)
        for _sig in sorted(_gb, key=repr):
            for a, b in _pair_nearest_bounds(_gb[_sig], _gf[_sig]):
                if (not all(close(x, y) for x, y in zip(a.bounds, b.bounds))
                        or not all(close(x, y) for x, y in zip(a.margin, b.margin))):
                    diffs.append(f"Paste aperture {a.label()}: bounds/margin "
                                 f"board={a.bounds}/{a.margin} file={b.bounds}/{b.margin}")

    # #962: what the off-outline grade discloses it could NOT measure. A front
    # that misses an entry reports a narrower blind spot than the other.
    def _um(pcb):
        return _Counter((u.get('owner_ref', ''), u.get('kind', ''))
                        for u in (pcb.graphic_copper_unmeasured or []))
    _ub, _uf = _um(from_board), _um(from_file)
    if _ub != _uf:
        diffs.append(f"Unmeasured graphic copper: only board={sorted((_ub - _uf).elements())[:6]} "
                     f"only file={sorted((_uf - _ub).elements())[:6]}")

    # #1279: the protected sense branches -- what each front would keep new
    # copper off, and why a branch it declines is declined.
    from sense_branch import branch_signature
    _sb = sorted(branch_signature(b) for b in (from_board.sense_branches or []))
    _sf = sorted(branch_signature(b) for b in (from_file.sense_branches or []))
    if _sb != _sf:
        diffs.append(f"Sense branches: board={_sb} file={_sf}")

    return diffs


def save_extracted_data(pcb_data: PCBData, output_path: str):
    """Save extracted PCB data to JSON file."""

    def serialize(obj):
        if hasattr(obj, '__dict__'):
            return {k: serialize(v) for k, v in obj.__dict__.items()}
        elif isinstance(obj, dict):
            return {str(k): serialize(v) for k, v in obj.items()}
        elif isinstance(obj, (list, tuple)):
            return [serialize(item) for item in obj]
        elif isinstance(obj, (set, frozenset)):
            # #1436: keepout `layers` is a set; JSON has no set type. Sort
            # (by repr, so mixed element types cannot raise) for stable output.
            return [serialize(item) for item in sorted(obj, key=repr)]
        else:
            return obj

    data = serialize(pcb_data)

    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(data, f, indent=2)


def get_nets_to_route(pcb_data: PCBData,
                      net_patterns: Optional[List[str]] = None,
                      exclude_patterns: Optional[List[str]] = None,
                      component_ref: Optional[str] = None) -> List[Net]:
    """
    Get nets that need routing based on filters.

    Args:
        pcb_data: Parsed PCB data
        net_patterns: List of wildcard patterns to match net names (e.g., ["LVDS_*", "DATA_*"])
        exclude_patterns: Patterns to exclude (default: GND, VCC, unconnected)
        component_ref: Only include nets connected to this component

    Returns:
        List of Net objects with 2+ pads that need routing
    """
    import fnmatch

    if exclude_patterns is None:
        exclude_patterns = ['*GND*', '*VCC*', '*VDD*', '*unconnected*', '*NC*', '']

    routes = []

    for net_id, net in pcb_data.nets.items():
        # Skip nets with < 2 pads (nothing to route)
        if len(net.pads) < 2:
            continue

        # Skip based on exclude patterns
        excluded = False
        for pattern in exclude_patterns:
            if fnmatch.fnmatchcase(net.name.upper(), pattern.upper()):
                excluded = True
                break
        if excluded:
            continue

        # Filter by net patterns if provided
        if net_patterns:
            matched = False
            for pattern in net_patterns:
                if fnmatch.fnmatchcase(net.name, pattern):
                    matched = True
                    break
            if not matched:
                continue

        # Filter by component if provided
        if component_ref:
            has_component = any(p.component_ref == component_ref for p in net.pads)
            if not has_component:
                continue

        routes.append(net)

    return routes


def _is_ball_grid(pads) -> bool:
    """Gate for geometry-based BGA classification: only a genuine ball/pin grid
    qualifies — enough pads, near-uniform pad size, AND a true 2-D matrix.
    Guards against keyswitches, SMD diode pairs, thermal-via exposed pads, and
    connector arrays being classified as BGAs and walled off behind exclusion
    zones (issue #82). BGA/PGA balls are identical; the size-mix false positives
    (switch pins + stabilizer holes, EP via grid + perimeter pads) fail the
    uniform-size test.

    The matrix test additionally rejects wide-pitch through-hole *headers* —
    e.g. the 2-row RPi_Pico_SMD_TH, which is uniform-size but only two columns
    wide (issue #82, set-2 reopen). A real array fills >=3 rows AND >=3 columns
    with several pads each; a 2-row header's columns are deep but it has only
    two of them, and a 1-/2-row header's rows hold just 1-2 pads. Pad-local
    coordinates are used so the test is independent of placement rotation.
    """
    if len(pads) < 16:
        return False
    size_counts = {}
    for p in pads:
        key = (round(p.size_x, 2), round(p.size_y, 2))
        size_counts[key] = size_counts.get(key, 0) + 1
    dominant_size, dominant_n = max(size_counts.items(), key=lambda kv: kv[1])
    if dominant_n < 0.9 * len(pads):
        return False

    # Among the uniform pads, count rows/columns that hold >=3 pads. A genuine
    # ball/pin array populates several of each; a pin header does not.
    uniform = [p for p in pads
               if (round(p.size_x, 2), round(p.size_y, 2)) == dominant_size]
    return pads_form_matrix(uniform)


def pads_form_matrix(pads) -> bool:
    """Do the pads fill a 2-D matrix: >=3 rows AND >=3 columns that each hold
    >=3 of them? Such an array has INTERIOR pads, every neighbour ring of which
    is more pads -- the geometry a via lattice (a pour that must thread between
    drop vias) is about. A perimeter land pattern (LGA-12, a 2-row header) has
    none: every pad is on the periphery. Pad-local coordinates, so the answer
    does not depend on placement rotation."""
    row_counts = {}
    col_counts = {}
    for p in pads:
        ry = round(p.local_y, 2)
        rx = round(p.local_x, 2)
        row_counts[ry] = row_counts.get(ry, 0) + 1
        col_counts[rx] = col_counts.get(rx, 0) + 1
    populated_rows = sum(1 for n in row_counts.values() if n >= 3)
    populated_cols = sum(1 for n in col_counts.values() if n >= 3)
    return populated_rows >= 3 and populated_cols >= 3


def detect_package_type(footprint: Footprint) -> str:
    """
    Detect the package type of a footprint based on its characteristics.

    Returns one of: 'BGA', 'QFN', 'QFP', 'SOIC', 'DIP', 'OTHER'

    Detection is based on:
    - Footprint name patterns
    - Pad arrangement (grid vs perimeter)
    - Pad shapes and sizes
    """
    fp_name = footprint.footprint_name.upper()

    # Check footprint name first. Grid/land-grid/chip-scale arrays all get BGA
    # treatment (bga_fanout escape + BGA exclusion zone): LGA (land grid), CSP/
    # WLCSP/WLP (wafer-level chip-scale = micro-BGA), CGA (column grid). Without
    # this an LGA-12 etc. is too small for the geometric grid gate below and falls
    # through to OTHER, so its interior lands get routed over (issue #144).
    if any(k in fp_name for k in ('BGA', 'FBGA', 'LFBGA', 'LGA', 'CSP', 'WLCSP', 'WLP', 'CGA')):
        return 'BGA'
    if 'QFN' in fp_name or 'DFN' in fp_name or 'MLF' in fp_name:
        return 'QFN'
    if 'QFP' in fp_name or 'LQFP' in fp_name or 'TQFP' in fp_name:
        return 'QFP'
    if 'PLCC' in fp_name:
        # PLCC is a PERIMETER package (J-lead), never a ball array -- but the
        # THT socket variant's staggered double-ring pin field reads as a
        # sparse uniform grid and passed _is_ball_grid, so route.py walled a
        # PLCC-44 socket behind a BGA exclusion zone and nearby nets burned
        # ~1M A* iterations each (#513 item 16, rc2014_82c55_ide U1; the #82
        # class). SMD PLCC gets QFN (perimeter) treatment; the THT socket
        # needs no fanout at all -- its pins are reachable on every layer.
        return 'OTHER' if any((p.drill or 0) > 0 for p in footprint.pads) else 'QFN'
    if 'SOIC' in fp_name or 'SOP' in fp_name or 'SSOP' in fp_name or 'TSSOP' in fp_name:
        return 'SOIC'
    if 'DIP' in fp_name or 'PDIP' in fp_name:
        return 'DIP'

    # Analyze pad arrangement if name doesn't indicate type. The PINS only
    # (#1148): a paste or mask window is not a pad, and reading it made an
    # 0201's split paste windows a four-pad "QFN" (rp2350 C28/R9) and
    # orangecrab U6 a QFN that is OTHER without its apertures.
    pads = non_aperture_pads(footprint)
    if len(pads) < 4:
        return 'OTHER'

    # Get unique X and Y positions
    x_positions = sorted(set(round(p.global_x, POSITION_DECIMALS) for p in pads))
    y_positions = sorted(set(round(p.global_y, POSITION_DECIMALS) for p in pads))

    # BGA: grid arrangement (multiple rows AND columns of pads)
    # QFN/QFP: perimeter arrangement (pads mostly on edges)

    if len(x_positions) >= 4 and len(y_positions) >= 4:
        # Check if pads form a filled grid (BGA) or just perimeter (QFN/QFP)
        # Count pads in interior vs perimeter
        min_x, max_x = min(x_positions), max(x_positions)
        min_y, max_y = min(y_positions), max(y_positions)

        # Define "interior" as not on the outermost positions
        interior_pads = 0
        perimeter_pads = 0
        tolerance = 0.1

        for pad in pads:
            on_edge = (abs(pad.global_x - min_x) < tolerance or
                      abs(pad.global_x - max_x) < tolerance or
                      abs(pad.global_y - min_y) < tolerance or
                      abs(pad.global_y - max_y) < tolerance)
            if on_edge:
                perimeter_pads += 1
            else:
                interior_pads += 1

        # BGA has many interior pads, QFN/QFP has mostly perimeter pads.
        # Either way the geometry path only declares BGA for a genuine
        # uniform ball/pin grid (issue #82).
        if interior_pads > perimeter_pads:
            return 'BGA' if _is_ball_grid(pads) else 'OTHER'
        elif perimeter_pads > 0:
            # Check pad shapes - QFN typically has rectangular pads, BGA has circular
            circular_pads = sum(1 for p in pads if p.shape in ('circle', 'oval'))
            rect_pads = sum(1 for p in pads if p.shape in ('rect', 'roundrect'))
            if rect_pads > circular_pads:
                return 'QFN'
            else:
                return 'BGA' if _is_ball_grid(pads) else 'OTHER'

    return 'OTHER'


def get_footprint_bounds(footprint: Footprint, margin: float = 0.0) -> Tuple[float, float, float, float]:
    """
    Get the bounding box of a footprint based on its pad positions.

    Args:
        footprint: The footprint to analyze
        margin: Extra margin to add around the bounds (in mm)

    Returns:
        (min_x, min_y, max_x, max_y) tuple
    """
    if not footprint.pads:
        # Fall back to footprint position if no pads
        return (footprint.x - margin, footprint.y - margin,
                footprint.x + margin, footprint.y + margin)

    min_x = min(p.global_x - p.size_x/2 for p in footprint.pads)
    max_x = max(p.global_x + p.size_x/2 for p in footprint.pads)
    min_y = min(p.global_y - p.size_y/2 for p in footprint.pads)
    max_y = max(p.global_y + p.size_y/2 for p in footprint.pads)

    return (min_x - margin, min_y - margin, max_x + margin, max_y + margin)


def find_components_by_type(pcb_data: 'PCBData', package_type: str) -> List[Footprint]:
    """
    Find all components of a specific package type.

    Args:
        pcb_data: Parsed PCB data
        package_type: One of 'BGA', 'QFN', 'QFP', 'SOIC', 'DIP', 'OTHER'

    Returns:
        List of matching Footprint objects
    """
    matches = []
    for ref, fp in pcb_data.footprints.items():
        if detect_package_type(fp) == package_type:
            matches.append(fp)
    return matches


#: An adjacent-coordinate gap below this is not spacing -- it is float noise, or
#: two pads that sit at the same point on that axis. 10um is far under any real
#: pad pitch (the finest in the corpus is a 0.2mm QFN).
_PITCH_NOISE_MM = 0.01


def detect_bga_pitch(footprint: Footprint) -> float:
    """
    Detect the pitch (pad spacing) of a BGA footprint.

    The MEDIAN adjacent gap per axis, then the smaller of the two axes. On a
    regular array that is exactly the pitch, and unlike a minimum it is not
    moved by a few odd pads.

    It used to return `min()` over the adjacent gaps of the unique x and y
    coordinates, which let a SINGLE anomalous pad pair speak for the whole
    package. Measured over corpus sets 1-5, that read a pitch of ~1e-6mm for
    cparti_fpga's 256-ball U1 and zynq_ad9364's 400-ball U1 and U2 (all real
    1.0mm/0.8mm arrays), 0.0125mm for watchy's U6, and 0.006mm for a 2.54mm
    header -- 78 of 348 footprints under 0.05mm, which is not a pitch any part
    has. Two consumers acted on those numbers:

      * `auto_detect_bga_exclusion_zones` (and routing_common's --no-bga-zones
        branch) set `edge_tolerance = margin + pitch * 1.1`, which feeds
        `connectivity.is_edge_stub`. That compares a pad CENTRE against a
        bounding box drawn at pad EDGES, so a collapsed tolerance can never
        match: is_edge_stub returned False for every pad of those four parts,
        silently disabling the outer-row test that gates ~10
        layer_swap_optimization branches -- on the largest arrays on the board.
      * `route_planes._resolve_zone_clearance_impl` computes
        `gap = pitch - field_via` and takes a min ACROSS fields, so one bad
        reading poisons the whole board: it returns early warning "pour cannot
        thread the densest BGA lattice even at the fab floor" and skips the
        tightening every other field might have needed. Its `if not pitch`
        guard does not catch this -- the bad values are ~1e-6, not 0. Seen in
        the recorded corpus on quickfeather, whose 10-pad U5 read 0.095mm and
        produced a NEGATIVE requirement (`needs -0.253mm < floor 0.1`).

    `diff_pair_routing._field_at` reports the value in diagnostics and says so
    ("reported, never acted on"); it is unaffected either way.

    Returns:
        Pitch in mm, or 1.0 as default if cannot be detected
    """
    # The pins only (#1148): a thermal pad's split paste windows sit between
    # the balls, so they read as a pitch no part has (glasgow U36/U8: 0.1 for
    # a 0.325 array; rp2350's 0402 caps: 0.025).
    pads = non_aperture_pads(footprint)
    if len(pads) < 2:
        return 1.0

    axis_pitches = []
    for _coord in (lambda q: q.global_x, lambda q: q.global_y):
        positions = sorted({_coord(p) for p in pads})
        gaps = [b - a for a, b in zip(positions, positions[1:])
                if (b - a) >= _PITCH_NOISE_MM]
        if gaps:
            axis_pitches.append(_median(gaps))

    if axis_pitches:
        return min(axis_pitches)
    return 1.0


def _median(values):
    """Median without importing statistics into this hot module."""
    ordered = sorted(values)
    n = len(ordered)
    mid = n // 2
    if n % 2:
        return ordered[mid]
    return (ordered[mid - 1] + ordered[mid]) / 2.0


def auto_detect_bga_exclusion_zones(pcb_data: 'PCBData', margin: float = 0.0) -> List[Tuple[float, float, float, float, float]]:
    """
    Auto-detect BGA exclusion zones from all BGA components in the PCB.

    Via placement should be avoided inside BGA packages to prevent shorts
    with the BGA balls.

    The zone is anchored to the BGA's PAD bounding box (get_footprint_bounds is
    pad-edge based), not the courtyard, and defaults to margin=0 so the zone ends
    exactly at the edge of the outermost pads. The fanout escapes every pad to a
    stub that exits PAST the pad edge (>= (pitch - pad_size)/2, and ideally
    BGA_EXIT_MARGIN), so a pad-edge zone leaves those stub tips in open copper --
    a coupled/single-ended leg can then attach to a tip outside the keep-out
    instead of being marooned inside it. The old margin=0.5 pushed the zone ~0.3mm
    PAST where the escapes exit, burying the tips (issue #243).

    Returns zones as 5-tuples: (min_x, min_y, max_x, max_y, edge_tolerance)
    where edge_tolerance = margin + pitch * 1.1 (pitch + 10%)

    Args:
        pcb_data: Parsed PCB data
        margin: Extra margin around the pad bounding box (in mm); 0 = end at pad edge

    Returns:
        List of (min_x, min_y, max_x, max_y, edge_tolerance) tuples for each BGA
    """
    zones = []
    bga_components = find_components_by_type(pcb_data, 'BGA')

    for fp in bga_components:
        bounds = get_footprint_bounds(fp, margin=margin)
        pitch = detect_bga_pitch(fp)
        # Edge tolerance = margin + pitch * 1.1 (pitch + 10% for tolerance)
        edge_tolerance = margin + pitch * 1.1
        zones.append((*bounds, edge_tolerance))

    return zones


if __name__ == "__main__":
    import argparse

    # A REAL parser, not a hand-rolled `Usage:` print. `--help` used to be read
    # as the input FILENAME here, so a capability probe got
    # `FileNotFoundError: '--help'` -- and this module, the one every other
    # tool parses boards with, was invisible to any catalogue that asks a tool
    # what it is (#937). `krt_registry.py` enumerates the catalogue by exactly
    # that question, so a tool that cannot answer it is a tool nobody finds.
    _ap = argparse.ArgumentParser(
        description="Parse a KiCad PCB file and save the extracted data as "
                    "JSON.")
    _ap.add_argument('input_file', help='the .kicad_pcb to parse')
    _ap.add_argument('output_file', nargs='?', default=None,
                     help='where to write the JSON '
                          '(default: <input>_extracted.json)')
    _a = _ap.parse_args()

    input_file = _a.input_file
    output_file = _a.output_file or input_file.replace('.kicad_pcb',
                                                       '_extracted.json')

    print(f"Parsing {input_file}...")
    pcb_data = parse_kicad_pcb(input_file)

    print(f"Found:")
    print(f"  - {len(pcb_data.board_info.copper_layers)} copper layers: {pcb_data.board_info.copper_layers}")
    print(f"  - {len(pcb_data.nets)} nets")
    print(f"  - {len(pcb_data.footprints)} footprints")
    print(f"  - {sum(len(fp.pads) for fp in pcb_data.footprints.values())} pads")
    print(f"  - {len(pcb_data.vias)} vias")
    print(f"  - {len(pcb_data.segments)} track segments")
    if pcb_data.board_info.board_bounds:
        bounds = pcb_data.board_info.board_bounds
        print(f"  - Board bounds: ({bounds[0]:.1f}, {bounds[1]:.1f}) to ({bounds[2]:.1f}, {bounds[3]:.1f}) mm")

    print(f"\nSaving extracted data to {output_file}...")
    save_extracted_data(pcb_data, output_file)
    print("Done!")
