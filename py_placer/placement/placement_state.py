"""Is this board in a state the placement tools can work on? (#431)

Two gates, both report-only here; the CLIs decide what to do with the verdict.

**Unplaced.** A tool that reads anything off the poses has nothing to work
with on a pile of parts at the origin: a grade fires every rule at once,
labels placed around a pile mean nothing, and an intent read off a pile's
poses would record the pile as the spec. This module detects that and lets
the caller refuse (check_floorplan, beautify_labels), warn
(render_placement) or read nothing off the parts' poses (`emit_intent` on a
`pile`, #1103) -- which is how `place_simple.py` treats every unlocked part
anyway.

**Already routed.** Placement models no copper at all: legality is courtyard,
pads and outline, cost is airwires, and `placement.writer.write_placed_output`
rewrites footprint positions only. So placement on a routed board moves parts
and leaves every track behind, detached from its pad.

Deliberately NOT in `legality.py`. That module answers "is this placement legal",
and by that measure `watchy.kicad_pcb` is illegal -- 81 of its 82 parts are in
courtyard violation. Unplaced-ness is a different question and must not borrow
that answer: **density is not unplacedness.**
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional

# Exit code for "the board is not in a state this tool can work on".
# 0 = ok, 1 = crash, 2 = argparse (parser.error), 3 = this.
# A distinct code so a caller branches on the number rather than scraping text.
UNPLACED_EXIT = 3

# --- thresholds -------------------------------------------------------------
# S1: share of footprints sharing a position with another. Two parts at the SAME
# coordinate is physically impossible in a real placement however dense, so this
# is the one signal strong enough to fire alone. Measured across every board in
# kicad_files/: none has more than 2 co-located parts, so the headroom is real.
DUP_FRACTION = 0.5
# S2: bbox diagonal of footprint centers / board diagonal. NEVER sufficient
# alone -- a legitimate 6-part board in the corner of a large outline is
# low-spread. Deliberately not courtyard-area-over-board-area, which needs
# polygon area and breaks on the boards with no courtyards at all.
SPREAD_RATIO = 0.15
# S3: share of footprints whose courtyard leaves the outline. 0.9, not 0.5,
# because castellated and edge-connector boards legitimately overhang.
OUTSIDE_FRACTION = 0.9

DEFAULT_THRESHOLDS = {'dup_fraction': DUP_FRACTION,
                      'spread_ratio': SPREAD_RATIO,
                      'outside_fraction': OUTSIDE_FRACTION}


@dataclass
class PlacementState:
    unplaced: bool = False
    partially_unplaced: bool = False
    has_copper: bool = False
    reasons: List[str] = field(default_factory=list)
    signals: Dict[str, object] = field(default_factory=dict)
    n_footprints: int = 0
    distinct_positions: int = 0
    duplicate_fraction: float = 0.0
    spread_ratio: Optional[float] = None
    # None means UNAVAILABLE (no usable outline), never 0.0. A misparsed outline
    # must not be able to manufacture an "unplaced" verdict.
    outside_fraction: Optional[float] = None
    #: #1165. The same share over the UNLOCKED parts only, measured when the
    #: board has both locked and unlocked pad-bearing parts (None otherwise,
    #: or with no usable outline). A KiCad lock is a placement decision: the
    #: skill places and locks the fixed parts BEFORE it seeds, and those
    #: on-board parts diluted `outside_fraction` below the threshold while
    #: every part still to be placed sat in the pile (ecc83: 4 locked holes
    #: of 15 parts, 0.733).
    movable_outside_fraction: Optional[float] = None
    stacked_refs: List[str] = field(default_factory=list)
    #: The subset of `stacked_refs` that is not explicable as deliberate
    #: co-location -- i.e. neither on a side the others cannot reach (a DRILLED
    #: part reaches both) nor a marker class. Being LOCKED is deliberately not
    #: an excuse; see the NOTE in `stack_suspects`. `partially_unplaced` keys on
    #: THIS, not on `stacked_refs`.
    stacked_suspect_refs: List[str] = field(default_factory=list)
    segments: int = 0
    vias: int = 0
    #: KiCad teardrop areas (#1164): generated where a track meets a pad or
    #: via, so routing output exactly as a track is, and left behind on the
    #: pad's old pose when the part moves.
    teardrops: int = 0

    @property
    def copper_carried(self) -> str:
        """What `has_copper` counted, for a refusal to name."""
        if self.teardrops:
            return (f"{self.segments} segment(s), {self.vias} via(s) and "
                    f"{self.teardrops} teardrop zone(s)")
        return f"{self.segments} segment(s) and {self.vias} via(s)"

    @property
    def off_outline(self) -> bool:
        """The parts still to be placed sit off the outline (#1165):
        `s3_outside` over every part, or `s3_movable_outside` over the
        unlocked ones when some are locked. A staging ring is spread, not
        stacked, so it reads neither unplaced nor partially unplaced; and a
        pile whose fixed parts were placed and LOCKED first -- the order the
        placement skill prescribes -- reads under the whole-board threshold
        once enough of them stand on the board."""
        sig = self.signals or {}
        return bool(sig.get('s3_outside') or sig.get('s3_movable_outside'))

    @property
    def pile(self) -> bool:
        """The poses carry no placement decisions (#1103, #1109).

        `unplaced`, OR a staging ring (`off_outline`: spread, not stacked,
        so it reads neither unplaced nor partially unplaced -- run 36/37's
        StickHub pile, 93% off the board; measured over the UNLOCKED parts
        when some are locked, #1165), OR a heap: at least `DUP_FRACTION` of
        the parts stacked in a way a placed board does not explain
        (`stacked_suspect_refs`, which excuses far-side and marker
        co-locations; the raw `duplicate_fraction` read a placed orangecrab
        as a pile). The ONE pile test: emit_intent, board_brief and
        check_assembly read it, and the placement skill chooses its mode
        from it. A locked part's pose stays a decision on a pile: emit_intent
        keeps its claims.
        """
        n = max(1, self.n_footprints or 0)
        return bool(self.unplaced or self.off_outline
                    or (self.partially_unplaced
                        and len(self.stacked_suspect_refs) / n
                        >= DUP_FRACTION))

    @property
    def blocked(self) -> bool:
        """Either gate says a placement tool should not proceed."""
        return self.unplaced or self.has_copper


def _diag(xs, ys) -> float:
    if not xs:
        return 0.0
    return ((max(xs) - min(xs)) ** 2 + (max(ys) - min(ys)) ** 2) ** 0.5


def assess_placement(pcb_data, pcb_file: Optional[str] = None,
                     thresholds: Optional[Dict] = None) -> PlacementState:
    """Measure the board's placement state. Never raises, never mutates."""
    th = dict(DEFAULT_THRESHOLDS)
    th.update(thresholds or {})
    st = PlacementState()

    from kicad_parser import non_aperture_pads
    # Aperture-only pads are not pads (#1143).
    fps = [fp for fp in (pcb_data.footprints or {}).values()
           if non_aperture_pads(fp)]
    st.n_footprints = len(fps)
    # ROUTED copper only (#908). A footprint's own drawn copper -- a SOT89
    # tab, a PCB antenna -- now parses as `graphic=True` Segments, and it is
    # not something a placement run would strand: it MOVES WITH ITS PART.
    # Counting it made watchy's 48 antenna edges read as a routed board, and
    # every placement CLI refused an unrouted board with "moving a footprint
    # would strand its tracks".
    st.segments = len([s for s in (pcb_data.segments or [])
                       if not getattr(s, 'graphic', False)])
    st.vias = len(pcb_data.vias or [])
    # A TEARDROP zone is routing output too (#1164): KiCad generates one where
    # a track meets a pad or via and drops it on refill once the track is
    # gone, and each sits on its pad's pose. A board stripped of its tracks
    # but not its teardrops still records where every teardropped part was,
    # and a part moved off it strands the zone. One per zone block: a
    # multi-layer zone parses as one Zone per layer, but a teardrop has one.
    st.teardrops = len({(z.uuid or id(z)) for z in (pcb_data.zones or [])
                        if getattr(z, 'teardrop', False)})
    st.has_copper = (st.segments + st.vias + st.teardrops) > 0
    if not fps:
        st.reasons.append("board has no footprints with pads")
        return st

    # --- S1: coincident positions
    pos: Dict[tuple, List[str]] = {}
    for fp in fps:
        pos.setdefault((round(fp.x, 3), round(fp.y, 3)), []).append(fp.reference)
    st.distinct_positions = len(pos)
    dup_refs = sorted(r for refs in pos.values() if len(refs) > 1 for r in refs)

    # A CO-LOCATED PAIR IS NOT AUTOMATICALLY A PILE-UP. This was bare origin
    # equality over every pad-bearing footprint, so six fiducials sitting on
    # four mounting holes -- placed there deliberately, and stamped
    # `(locked yes)` -- reported "N footprint(s) are stacked ... typically a
    # netlist re-import dropping new parts at one spot", on a board where
    # nothing had moved. TWO filters, both from data already in hand and both
    # already trusted elsewhere in this codebase:
    #
    #   * opposite SIDES cannot pile up -- unless a part is DRILLED, which puts
    #     it on both. legality._sides_interact draws exactly that line
    #     ("None = through (both sides)"), and this filter now follows it.
    #   * a MARKER class (fiducial / mount_hole / testpoint) is co-located by
    #     design. legality._MARKER waives exactly this set for assembly.
    #
    # "Every part in the group is LOCKED" is deliberately NOT a filter: see
    # the NOTE in `stack_suspects`.
    #
    # The strong signal (s1) keeps every ref, because a genuine pile-up buries
    # markers and locks along with everything else; only the weak
    # `partially_unplaced` signal is filtered.
    _by_ref = {fp.reference: fp for fp in fps}
    #: Matches legality._MARKER, which waives exactly this set for assembly.
    _MARKER_CLASSES = ('fiducial', 'mount_hole', 'testpoint')

    def _marker(ref):
        try:
            from placement.part_class import classify_part
            # `.name`, not `.kind`. PartClass has fields (name, confidence,
            # evidence); reading `.kind` raised AttributeError straight into
            # the except below, so this whole filter was dead code while the
            # warning text advertised it.
            return classify_part(_by_ref[ref], ref).name in _MARKER_CLASSES
        except Exception:                                   # noqa: BLE001
            return False

    def _sides_of(ref):
        return footprint_sides(_by_ref[ref])

    def _suspect_in(refs_at_one_spot):
        # The NOTE on what is deliberately not a filter ("all locked") is
        # on `stack_suspects`.
        return stack_suspects(refs_at_one_spot, _sides_of, _marker)

    suspect = sorted(r for refs in pos.values() if len(refs) > 1
                     for r in _suspect_in(refs))
    st.stacked_refs = dup_refs
    st.stacked_suspect_refs = suspect
    st.duplicate_fraction = len(dup_refs) / len(fps)
    s1 = (st.duplicate_fraction >= th['dup_fraction']
          and st.distinct_positions <= max(3, 0.1 * len(fps)))

    # --- S2: spread
    xs = [fp.x for fp in fps]
    ys = [fp.y for fp in fps]
    bb = getattr(pcb_data.board_info, 'board_bounds', None)
    if bb:
        board_diag = ((bb[2] - bb[0]) ** 2 + (bb[3] - bb[1]) ** 2) ** 0.5
        if board_diag > 1e-6:
            st.spread_ratio = _diag(xs, ys) / board_diag
    s2 = st.spread_ratio is not None and st.spread_ratio < th['spread_ratio']

    # --- S3: outside the outline
    _out_flags: Dict[str, bool] = {}
    st.outside_fraction = _outside_fraction(pcb_data, pcb_file, fps,
                                            flags_out=_out_flags)
    s3 = (st.outside_fraction is not None
          and st.outside_fraction >= th['outside_fraction'])
    # #1165: the same test over the UNLOCKED parts. Only a REPORTED signal
    # (`off_outline`, so `pile`): `unplaced` and
    # `partially_unplaced` are unchanged, so every `gate_or_exit` caller
    # refuses and warns exactly as before.
    _locked = [fp for fp in fps if getattr(fp, 'locked', False)]
    _movable = [fp for fp in fps if not getattr(fp, 'locked', False)]
    if _locked and _movable and st.outside_fraction is not None:
        st.movable_outside_fraction = (
            sum(1 for fp in _movable if _out_flags.get(fp.reference))
            / len(_movable))
    s3m = (st.movable_outside_fraction is not None
           and st.movable_outside_fraction >= th['outside_fraction'])

    st.signals = {'s1_coincident': s1, 's2_low_spread': s2, 's3_outside': s3,
                  's3_movable_outside': s3m,
                  'duplicate_fraction': round(st.duplicate_fraction, 3),
                  'distinct_positions': st.distinct_positions,
                  'spread_ratio': (None if st.spread_ratio is None
                                   else round(st.spread_ratio, 4)),
                  'outside_fraction': (None if st.outside_fraction is None
                                       else round(st.outside_fraction, 3)),
                  'locked': len(_locked),
                  'movable_outside_fraction': (
                      None if st.movable_outside_fraction is None
                      else round(st.movable_outside_fraction, 3))}

    st.unplaced = bool(s1 or (s2 and s3) or (s3 and st.distinct_positions <= 2))
    st.partially_unplaced = (not st.unplaced) and bool(suspect)

    if s1:
        st.reasons.append(
            f"{len(dup_refs)} of {len(fps)} footprints ({st.duplicate_fraction:.0%}) "
            f"share a position with another, at only {st.distinct_positions} "
            f"distinct coordinate(s) -- parts have not been placed")
    if s2:
        st.reasons.append(
            f"footprints span {st.spread_ratio:.1%} of the board diagonal")
    if s3:
        st.reasons.append(
            f"{st.outside_fraction:.0%} of footprints lie outside the board outline")
    elif s3m:
        st.reasons.append(
            f"{st.movable_outside_fraction:.0%} of the unlocked footprints lie "
            f"outside the board outline ({len(_locked)} locked part(s) not "
            f"counted: a lock is a placement decision)")
    if st.partially_unplaced:
        _benign_n = len(dup_refs) - len(suspect)
        st.reasons.append(
            f"{len(suspect)} footprint(s) are stacked on another "
            f"({', '.join(suspect[:8])}) -- typically a netlist re-import "
            f"dropping new parts at one spot on an otherwise-placed board"
            + (f" [{_benign_n} further co-located ref(s) look deliberate "
               f"(the far side of the board, or a marker class) and are not "
               f"counted -- note being LOCKED is NOT one of the excuses, "
               f"because this toolchain stamps its own locks]"
               if _benign_n else ""))
    return st


def footprints_outside(pcb_data, pcb_file, refs) -> Optional[Dict[str, bool]]:
    """`{ref: its courtyard leaves the outline}` for `refs`, measured as
    `assess_placement`'s S3 measures it; None when the board has no usable
    outline. On a pile this is what tells a STAGED pose (off the board)
    from one a stager kept (#1353)."""
    fps = [pcb_data.footprints[r] for r in refs
           if r in (pcb_data.footprints or {})]
    flags: Dict[str, bool] = {}
    if _outside_fraction(pcb_data, pcb_file, fps, flags) is None:
        return None
    return flags


def _outside_fraction(pcb_data, pcb_file, fps,
                      flags_out: Optional[Dict[str, bool]] = None
                      ) -> Optional[float]:
    """Share of footprints whose courtyard leaves the outline, or None when
    there is no usable outline to measure against. `flags_out`, when given,
    receives `{ref: outside}` per measured footprint, so a caller can take
    the share over a subset (#1165) without reading the courtyards twice."""
    bi = pcb_data.board_info
    if getattr(bi, 'board_bounds', None) is None:
        return None
    try:
        from placement.legality import (BoardOutlineGate, rotate_local_bounds)
    except Exception:
        return None
    try:
        gate = BoardOutlineGate(bi, 0.0)
    except Exception:
        return None
    if not gate.rings and gate.usable is None:
        return None

    sides = {}
    if pcb_file and os.path.isfile(pcb_file):
        try:
            from placement.parser import extract_courtyard_sides
            sides = extract_courtyard_sides(pcb_file)
        except Exception:
            sides = {}

    outside = 0
    for fp in fps:
        rect = _global_rect(fp, sides.get(fp.reference), rotate_local_bounds)
        if rect is None:
            continue
        try:
            hit = gate.rect_outside_amount(rect) > 0
        except Exception:
            return None
        if hit:
            outside += 1
        if flags_out is not None:
            flags_out[fp.reference] = hit
    return outside / len(fps) if fps else None


def _global_rect(fp, side_boxes, rotate_local_bounds):
    """Absolute courtyard rect for a footprint, falling back to its pad bbox --
    the same fallback the placement state uses when a footprint draws no
    courtyard."""
    local = None
    if side_boxes:
        from placement.legality import footprint_side
        from placement.parser import courtyard_for_side
        local = courtyard_for_side(side_boxes, footprint_side(fp))
    if local is None:
        try:
            from placement.utility import compute_footprint_bbox_local
            local = compute_footprint_bbox_local(fp)
        except Exception:
            return None
    if local is None:
        return None
    lx0, ly0, lx1, ly1 = rotate_local_bounds(*local, fp.rotation or 0.0)
    return (fp.x + lx0, fp.y + ly0, fp.x + lx1, fp.y + ly1)


def format_report(state: PlacementState, tool: str = 'placement', *,
                  allow_unplaced: bool = False, allow_routed: bool = False,
                  warn_only: bool = False) -> str:
    """Human text for a refusal or a warning. A condition the caller already
    overrides gets no "Override with" advice (#1202: beautify_labels and
    check_floorplan always pass allow_routed=True, ran and exited 0, and still
    printed the refusal and its override).

    `warn_only` is the renderer's mode, which refuses nothing: it gets no
    override advice at all, and no advice to run the renderer (#1354:
    render_placement told the user to "Override with --allow-unplaced", a
    flag it does not have and exits 2 on, then rendered anyway)."""
    allow_unplaced = allow_unplaced or warn_only
    allow_routed = allow_routed or warn_only
    lines = []
    if state.unplaced:
        lines.append(
            f"{tool}: this board does not look PLACED "
            f"({state.n_footprints} footprints at {state.distinct_positions} "
            f"distinct position(s)).")
        for r in state.reasons:
            lines.append(f"  - {r}")
        lines.append(
            "  Its poses are not a placement yet: py_placer/place_simple.py "
            "places a board from a pile.")
        if not warn_only:
            lines.append(
                "  To see what the file currently contains:  "
                "python3 -X utf8 py_tools/render_placement.py <board> "
                "-o state.png")
        if not allow_unplaced:
            lines.append("  Override with --allow-unplaced.")
    elif state.partially_unplaced:
        lines.append(f"{tool}: WARNING - {state.reasons[-1]}")
    if state.has_copper and allow_routed:
        lines.append(
            f"{tool}: this board carries {state.copper_carried}; proceeding "
            f"with the copper left as it is.")
    elif state.has_copper:
        lines.append(
            f"{tool}: this board carries {state.copper_carried}. Placement "
            f"moves FOOTPRINTS and does not move "
            f"copper -- every track would be left behind, detached from its pad. "
            f"Run placement on the UNROUTED board and re-route from there. "
            f"Override with --allow-routed.")
    return "\n".join(lines)


def gate_or_exit(pcb_data, pcb_file, tool: str, *, allow_unplaced: bool = False,
                 allow_routed: bool = False, warn_only: bool = False):
    """Assess, print, and exit `UNPLACED_EXIT` unless overridden.

    `warn_only=True` is the renderer's mode: it must still render an unplaced
    board (that is the whole point of being able to SEE the state), so it
    reports and returns.
    """
    import sys
    st = assess_placement(pcb_data, pcb_file)
    msg = format_report(st, tool, allow_unplaced=allow_unplaced,
                        allow_routed=allow_routed, warn_only=warn_only)
    blocking = (st.unplaced and not allow_unplaced) or (st.has_copper and not allow_routed)
    if msg:
        print(msg, file=sys.stderr if (blocking and not warn_only) else sys.stdout)
    if blocking and not warn_only:
        sys.exit(UNPLACED_EXIT)
    return st


#: Part classes whose co-location is by design: fiducials, mounting holes and
#: testpoints are stacked on purpose, and one real part on a fiducial is not
#: a stack of parts (check_assembly's COINCIDENT ORIGINS, run 19).
ASSEMBLY_MARKER_CLASSES = ('fiducial', 'mount_hole', 'testpoint')


def is_assembly_marker(pcb_data, ref: str) -> bool:
    """True when `ref` is an `ASSEMBLY_MARKER_CLASSES` part."""
    from placement.part_class import classify_part
    try:
        return classify_part(pcb_data.footprints[ref],
                             ref).name in ASSEMBLY_MARKER_CLASSES
    except Exception:                                          # noqa: BLE001
        return False


def footprint_sides(fp) -> tuple:
    """Which physical sides this footprint occupies: ('F',), ('B',) or both.

    A DRILLED part is on both, and that is not a nicety -- it is the case
    this partition got wrong. `legality.footprint_side` alone put tigard's
    J2 (F.Cu, nine 1.0mm drilled pads) and JP1 (B.Cu) in different groups,
    each alone on its side, so both were exonerated. Moved to one
    coordinate they produce SIX pad conflicts by `grade_pad_legality` --
    this function was the only thing in the repo saying they were fine, and
    it said so because it modelled a through-hole part as living on one
    side. `legality._sides_interact` has always drawn the other line
    ("None = through (both sides)"); `footprint_side` /
    `footprint_has_through_pads` are the canonical readers, so use them
    rather than comparing `fp.layer` strings.
    """
    try:
        from placement.legality import (footprint_has_through_pads,
                                        footprint_side)
        if footprint_has_through_pads(fp):
            return ('F', 'B')
        return (footprint_side(fp),)
    except Exception:                                       # noqa: BLE001
        # Unknown side is the SUSPICIOUS answer, not the safe one: put it
        # on both so it groups with everything at this coordinate.
        return ('F', 'B')


def stack_suspects(refs_at_one_spot, sides_of, is_marker,
                   touch=None) -> List[str]:
    """The refs at ONE coordinate that are not explicable, PER SIDE
    (`sides_of(ref)` -> its sides, `is_marker(ref)` -> a marker class).
    THE stack rule: `assess_placement` calls it about the board, and
    `coincident_stack_suspects` about the parts at one origin.

    `touch(a, b, side)`, when given (#1204), keeps in a side's group only
    the parts that share something physical there with another member
    (`coincident_touch`): a shared origin is a library's choice of datum,
    and two halves of one module footprint share theirs with 34 mm between
    their pads. None is the origin-only rule, which `assess_placement`
    keeps -- it reads a pile, where every part at the spot is suspect.

    Partitioned by side rather than tested group-wide. A group-wide test
    let ONE back-side part exonerate an arbitrarily large front-side pile:
    measured, 105 parts at a single coordinate with one on the far side
    reported zero suspects and an empty reason list -- silence about a
    board in a far worse state than the one this filter was written for.
    Parts on opposite sides cannot collide; parts on the SAME side still
    can, however many neighbours the far side has -- and a drilled part is
    on both sides, so it collides with either.
    """
    out = set()
    by_side = {'F': [], 'B': []}
    for r in refs_at_one_spot:
        for s in sides_of(r):
            by_side[s].append(r)
    for _side, group in by_side.items():
        if touch is not None:
            group = [r for r in group
                     if any(o != r and touch(r, o, _side) for o in group)]
        if len(group) < 2:
            continue                    # alone on its side: nothing to hit
        if all(is_marker(r) for r in group):
            continue                    # co-located by design
        # NOTE what is deliberately NOT here: "all locked". A lock records
        # a decision about WHERE a part goes; it does not make two parts
        # able to occupy one space. And this toolchain stamps its own locks
        # -- `place_pose lock`, through seeder.stamp_locked -- so a
        # pile-up the tools CREATED would have been invisible to the check
        # meant to catch pile-ups. Markers are the real discriminator: a
        # fiducial and a mounting hole are different physical things and
        # share a coordinate by design.
        out.update(group)
    return sorted(out)      # a through part appears in both side-groups


def coincident_stack_suspects(refs, sides, markers, touch=None) -> List[str]:
    """check_assembly's verdict on ONE origin holding `refs`, given each
    ref's `sides` and `markers` maps: the suspect refs when the origin GATES
    (>= 2 suspect non-markers -- `coincident_stack_groups`' rule), else [].
    For a caller deciding which parts will be present there (#1151).
    `touch` is `coincident_touch`'s predicate (#1204); the grader passes it,
    and so must any caller that means to agree with the grader."""
    sus = stack_suspects(refs, sides.__getitem__, markers.__getitem__,
                         touch=touch)
    if sum(1 for r in sus if not markers[r]) >= 2:
        return sus
    return []


def _copper_presence(fp):
    """`({side: copper pad bbox}, [NPTH hole bbox])` of one footprint at
    its pose (#1204). A pad's copper is on the faces its layers expand to
    (`check_drc.pad_copper_layers`, the router's and the grader's one
    expansion), so a plated barrel is on both; an NPTH has no copper and
    is returned as its HOLE, which no part's copper may cover on either
    face. Aperture-only pads are not pads (#1143). A pad tilted off the
    axes is boxed by its half-diagonal: a larger box, never a smaller one."""
    import math
    from kicad_parser import pad_is_aperture_only
    from check_drc import pad_copper_layers
    boxes: Dict[str, List[float]] = {}
    holes = []
    for pad in getattr(fp, 'pads', None) or ():
        if pad_is_aperture_only(pad):
            continue
        if getattr(pad, 'pad_type', '') == 'np_thru_hole':
            r = (getattr(pad, 'drill', 0.0) or 0.0) / 2.0
            hx = getattr(pad, 'hole_x', None)
            hy = getattr(pad, 'hole_y', None)
            cx = pad.global_x if hx is None else hx
            cy = pad.global_y if hy is None else hy
            if r > 0:
                holes.append((cx - r, cy - r, cx + r, cy + r))
            continue
        sx, sy = (pad.size_x or 0.0) / 2.0, (pad.size_y or 0.0) / 2.0
        if abs(getattr(pad, 'rect_rotation', 0.0) or 0.0) > 1e-6:
            sx = sy = math.hypot(sx, sy)
        box = (pad.global_x - sx, pad.global_y - sy,
               pad.global_x + sx, pad.global_y + sy)
        layers = pad_copper_layers(pad, ('F.Cu', 'B.Cu'))
        for side, layer in (('F', 'F.Cu'), ('B', 'B.Cu')):
            if layer not in layers:
                continue
            b = boxes.get(side)
            boxes[side] = (list(box) if b is None else
                           [min(b[0], box[0]), min(b[1], box[1]),
                            max(b[2], box[2]), max(b[3], box[3])])
    return {s: tuple(b) for s, b in boxes.items()}, holes


def body_contact_pairs(grade) -> Dict[frozenset, set]:
    """`{frozenset({a, b}): {side}}` -- the pairs a `grade_body_overlap`
    result says share physical space on a face (#1204): pad copper
    intersecting (`pad_intersection`), pad copper under a drawn body
    (`pads_under_body`), and two DRAWN bodies overlapping (`fab`) -- unless
    either body is silk-drawn, which may be an assembly marking and never
    gates (#896). Courtyards are not here: a courtyard is a keep-out margin,
    not a body, and the courtyard channel has its own gate."""
    from placement.body import SOURCE_SILK
    from placement.legality import HEIGHT_WAIVER
    srcs = (grade or {}).get('body_sources') or {}
    out: Dict[frozenset, set] = {}
    for q in (grade or {}).get('pairs') or ():
        if q.kind not in ('pad_intersection', 'pads_under_body', 'fab'):
            continue
        if q.waiver == HEIGHT_WAIVER:
            continue        # #1443: one stands under the other's body
        if q.kind == 'fab' and SOURCE_SILK in (srcs.get(q.a), srcs.get(q.b)):
            continue
        out.setdefault(frozenset((q.a, q.b)), set()).add(q.side)
    return out


def coincident_touch(pcb_data, contacts=None, waivers=(), hits=None):
    """`touch(a, b, side)` for `stack_suspects` (#1204): do two parts that
    share an origin share anything PHYSICAL on `side`?

    * their copper pad boxes overlap there (`_copper_presence`; a plated
      barrel puts a part's copper box on both faces) -- run 19's
      SW17/SW34/REF_PUCK_R had rotated pads that interleave, so no pad
      intersects and the boxes still overlap;
    * an NPTH hole of one lies on the other's copper box there; or
    * `contacts` (`body_contact_pairs` of the board's grade) names the pair
      on that side: pad copper intersecting, pads under a body, or two
      drawn bodies overlapping.

    Not an NPTH hole on an NPTH hole: CM5 IO's Module301 and Module302
    each draw the module's two mounting holes, at the same spots.

    A pair in `waivers` (an intent's `overlap_waivers`) never touches: the
    author accepted it, as they accept its courtyard pair. `hits`, when
    given, receives each waived pair that would otherwise have touched."""
    from placement.legality import EPS, rect_overlap_area
    waived = {frozenset(p) for p in (waivers or ()) if len(p) == 2}
    fps = getattr(pcb_data, 'footprints', None) or {}
    cache: Dict[str, tuple] = {}

    def _geom(ref):
        if ref not in cache:
            fp = fps.get(ref)
            cache[ref] = (_copper_presence(fp) if fp is not None
                          else ({}, []))
        return cache[ref]

    def _physical(a, b, side):
        if side in ((contacts or {}).get(frozenset((a, b))) or ()):
            return True
        (ba, ha), (bb, hb) = _geom(a), _geom(b)
        xa, xb = ba.get(side), bb.get(side)
        if xa and xb and rect_overlap_area(xa, xb) > EPS:
            return True
        return (bool(xb) and any(rect_overlap_area(h, xb) > EPS for h in ha)
                or bool(xa) and any(rect_overlap_area(h, xa) > EPS
                                    for h in hb))

    def touch(a, b, side):
        if frozenset((a, b)) in waived:
            if hits is not None and _physical(a, b, side):
                hits.add(tuple(sorted((a, b))))
            return False
        return _physical(a, b, side)
    return touch


def coincident_stack_groups(pcb_data, pcb_file: Optional[str] = None, *,
                            contacts=None, waivers=(),
                            exempt_out: Optional[List[Dict]] = None
                            ) -> List[Dict]:
    """check_assembly's COINCIDENT ORIGINS channel: `[{point, refs}]`, one
    entry per origin (rounded to 1 um) holding >= 2 suspect NON-marker
    parts. The pad-bearing parts are bucketed by origin, each bucket is
    partitioned by physical side, and a side's group keeps only the parts
    that share something physical there (#1204, `coincident_touch`: copper
    pad boxes, an NPTH hole on copper, or `contacts` -- the board grade's
    `body_contact_pairs`); all-marker groups are exonerated. `refs` keeps
    every suspect part at the point, markers included. A pair the intent's
    `overlap_waivers` names (`waivers`) is accepted here as it is in the
    courtyard channel. One function, so every reader and the grader cannot
    disagree about what a stack is.

    `exempt_out`, when given, receives `{point, refs, why}` for each origin
    the origin-only rule would have gated and this one does not: `why` is
    'apart' (nothing physical shared on a common face -- CM5 IO's two
    module halves, a Tag-Connect's alignment holes around a part on the far
    face) or 'waived' (with the `waived_pairs` that did it)."""
    from kicad_parser import non_aperture_pads
    fps = pcb_data.footprints or {}
    buckets: Dict = {}
    for ref, fp in fps.items():
        if non_aperture_pads(fp):
            buckets.setdefault((round(fp.x, 3), round(fp.y, 3)),
                               []).append(ref)
    hits: set = set()
    touch = coincident_touch(pcb_data, contacts, waivers, hits=hits)
    bare = coincident_touch(pcb_data, contacts)
    out = []
    for pt, refs in sorted(buckets.items()):
        if len(refs) < 2:
            continue
        refs = sorted(refs)
        sides = {r: footprint_sides(fps[r]) for r in refs}
        markers = {r: is_assembly_marker(pcb_data, r) for r in refs}
        sus = coincident_stack_suspects(refs, sides, markers, touch=touch)
        if sus:
            out.append({'point': [pt[0], pt[1]], 'refs': sus})
            continue
        if exempt_out is None or not coincident_stack_suspects(
                refs, sides, markers):
            continue
        if coincident_stack_suspects(refs, sides, markers, touch=bare):
            exempt_out.append({
                'point': [pt[0], pt[1]], 'refs': refs, 'why': 'waived',
                'waived_pairs': sorted(list(p) for p in hits
                                       if set(p) <= set(refs))})
        else:
            exempt_out.append({'point': [pt[0], pt[1]], 'refs': refs,
                               'why': 'apart'})
    return out

