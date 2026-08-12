"""Is this board in a state the placement tools can work on? (#431)

Two gates, both report-only here; the CLIs decide what to do with the verdict.

**Unplaced.** The quench is *perturbative* -- `docs/placement-optimization.md`
argues at length that a human seed is what lets it inherit every unmodeled
constraint for free, and `place_optimize.py`'s own docstring promises output that
is "your placement, nudged". Handed a pile of parts at the origin it has nothing
to refine: every candidate pose is illegal, so the run prints "0 parts moved"
plus a legality block that *looks like a result*. Worse, a large
`--max-displacement` produces a tiny scatter around the origin that looks like
progress and is not a placement. This module detects that and lets the caller
refuse instead. Placing a board from scratch is deliberately out of scope.

**Already routed.** Verified in code, not assumed: the quench models no copper at
all -- `place_optimize.py` contains zero references to segments or vias, legality
is courtyard + outline, cost is pad-to-pad airwires, and
`placement.writer.write_placed_output` rewrites footprint positions only. So
placement on a routed board moves every part and leaves every track behind,
detached from its pad. Nothing warned about this before.

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
    stacked_refs: List[str] = field(default_factory=list)
    #: The subset of `stacked_refs` that is not explicable as deliberate
    #: co-location -- i.e. neither on a side the others cannot reach (a DRILLED
    #: part reaches both) nor a marker class. Being LOCKED is deliberately not
    #: an excuse; see the NOTE in `_suspect_in`. `partially_unplaced` keys on
    #: THIS, not on `stacked_refs`.
    stacked_suspect_refs: List[str] = field(default_factory=list)
    segments: int = 0
    vias: int = 0

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

    fps = [fp for fp in (pcb_data.footprints or {}).values() if fp.pads]
    st.n_footprints = len(fps)
    st.segments = len(pcb_data.segments or [])
    st.vias = len(pcb_data.vias or [])
    st.has_copper = (st.segments + st.vias) > 0
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
    # A third filter, "every part in the group is LOCKED", was tried and
    # REMOVED -- see the NOTE in _suspect_in. It is named here because the
    # field docstring and the operator-facing reason string both used to
    # advertise it, which is worse than not having it.
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
        fp = _by_ref[ref]
        try:
            from placement.legality import (footprint_has_through_pads,
                                            footprint_side)
            if footprint_has_through_pads(fp):
                return ('F', 'B')
            return (footprint_side(fp),)
        except Exception:                                   # noqa: BLE001
            # Unknown side is the SUSPICIOUS answer, not the safe one: put it
            # on both so it groups with everything at this coordinate.
            return ('F', 'B')

    def _suspect_in(refs_at_one_spot):
        """The refs at ONE coordinate that are not explicable, PER SIDE.

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
            for s in _sides_of(r):
                by_side[s].append(r)
        for _side, group in by_side.items():
            if len(group) < 2:
                continue                    # alone on its side: nothing to hit
            if all(_marker(r) for r in group):
                continue                    # co-located by design
            # NOTE what is deliberately NOT here: "all locked". A lock records
            # a decision about WHERE a part goes; it does not make two parts
            # able to occupy one space. And this toolchain stamps its own locks
            # -- seeder.stamp_locked, called on place_seed's output -- so a
            # pile-up the tools CREATED would have been invisible to the check
            # meant to catch pile-ups. Markers are the real discriminator: a
            # fiducial and a mounting hole are different physical things and
            # share a coordinate by design. Costs nothing on the corpus: every
            # board that changed was already suppressed by side or by class.
            out.update(group)
        return sorted(out)      # a through part appears in both side-groups

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
    st.outside_fraction = _outside_fraction(pcb_data, pcb_file, fps)
    s3 = (st.outside_fraction is not None
          and st.outside_fraction >= th['outside_fraction'])

    st.signals = {'s1_coincident': s1, 's2_low_spread': s2, 's3_outside': s3,
                  'duplicate_fraction': round(st.duplicate_fraction, 3),
                  'distinct_positions': st.distinct_positions,
                  'spread_ratio': (None if st.spread_ratio is None
                                   else round(st.spread_ratio, 4)),
                  'outside_fraction': (None if st.outside_fraction is None
                                       else round(st.outside_fraction, 3))}

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


def _outside_fraction(pcb_data, pcb_file, fps) -> Optional[float]:
    """Share of footprints whose courtyard leaves the outline, or None when
    there is no usable outline to measure against."""
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
            if gate.rect_outside_amount(rect) > 0:
                outside += 1
        except Exception:
            return None
    return outside / len(fps) if fps else None


def _global_rect(fp, side_boxes, rotate_local_bounds):
    """Absolute courtyard rect for a footprint, falling back to its pad bbox --
    the same fallback the quench uses when a footprint draws no courtyard."""
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


def format_report(state: PlacementState, tool: str = 'placement') -> str:
    """Human text for a refusal or a warning."""
    lines = []
    if state.unplaced:
        lines.append(
            f"{tool}: this board does not look PLACED "
            f"({state.n_footprints} footprints at {state.distinct_positions} "
            f"distinct position(s)).")
        for r in state.reasons:
            lines.append(f"  - {r}")
        lines.append(
            "  This toolchain refines an existing placement; it does not place a "
            "board from scratch. Place the parts in KiCad first.")
        lines.append(
            "  To see what the file currently contains:  "
            "python3 render_placement.py <board> -o state.png")
        lines.append("  Override with --allow-unplaced.")
    elif state.partially_unplaced:
        lines.append(f"{tool}: WARNING - {state.reasons[-1]}")
    if state.has_copper:
        lines.append(
            f"{tool}: this board carries {state.segments} segment(s) and "
            f"{state.vias} via(s). Placement moves FOOTPRINTS and does not move "
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
    msg = format_report(st, tool)
    blocking = (st.unplaced and not allow_unplaced) or (st.has_copper and not allow_routed)
    if msg:
        print(msg, file=sys.stderr if (blocking and not warn_only) else sys.stdout)
    if blocking and not warn_only:
        sys.exit(UNPLACED_EXIT)
    return st
