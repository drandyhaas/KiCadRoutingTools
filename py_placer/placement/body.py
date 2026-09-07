"""One body model for every placement instrument (#896).

WHAT THIS IS FOR
----------------
A footprint's "body" is not one thing a board file states; it is whatever the
library happened to draw. Before this module the answer was decided in five
places with three different ladders, and the ladder that most consumers used --
`legality.part_local_bounds` -- went straight from courtyard to the PAD BOUNDING
BOX. On a library that draws no courtyard, every placement instrument therefore
graded pad boxes and could not see a body at all.

Measured on `esp_prog` (OLIMEX), the board that produced #896: **0 of 21**
footprints draw a courtyard, and six of them -- CON1, CON2, U1, U2, Q1, Q2 --
draw no `.Fab` outline either. Those six are the connector housings, the SSOP
and the SOT89 whose collisions cost run 25 two laps; each was found only by an
adversarial reviewer writing its own geometry, because no checker in the chain
could see a housing at all.

THE LADDER
----------
    courtyard  ->  fab  ->  silk U pad_bbox  ->  pad_bbox  ->  synthetic

`source` names the rung that answered, always, so a consumer reports "body from
silk" rather than "no body", and a pad-bbox answer is disclosed rather than
passing as geometry somebody drew.

Two rules the ladder encodes, both measured rather than assumed:

1. **Courtyard and fab are taken AS DRAWN; only the silk rung unions with the
   pads.** Silk is not an outline -- on a stock KiCad footprint it is a pair of
   clipped side ticks. Over the 10 esp_prog footprints that draw both fab and
   silk, the silk bbox is narrower than the fab body along the pad axis on 10 of
   10 (Y1: 0.508mm against a 3.200mm body, an 84% loss) and wider across it on
   10 of 10, so no offset reconciles them -- the sign of the error differs per
   axis. Taken bare, silk would SHRINK parts, which is the unsafe direction
   (`legality.LocalBounds`' own comment: shrinking can flip `zone_is_anchor`
   True->False, whose non-anchor branch has a strictly smaller admissible origin
   box). esp_prog Q1/Q2 (SOT23) would go from a 3.610 x 2.902 pad box to a
   0.838 x 2.845 silk box. Unioning the silk rung with the pad bbox removes that
   hazard entirely and -- verified -- changes none of the three numbers #896
   asks for.

   Unioning the courtyard and fab rungs too would be a DIFFERENT measurement:
   it moves lap-3 U1<->Y1 from -0.1330 to -0.2830, because Y1's pads overhang
   its fab body by 0.15mm on the facing side. That is the occupancy question,
   not the body question; see `occupancy_local` below.

2. **A pad-less footprint gets no silk body.** A logo, an OSHW mark or a
   decorative silk drawing is not a part to collide with. Measured: allowing
   them produced 5 corpus pairs above the run-23 blocking floors and ALL FIVE
   were logos (`logo`, `oshw:oshw`, `Glasgow:nono_hana_lines`). Refusing them
   leaves 4 new pairs corpus-wide and 0 above the floors.

NO OFFSET CONSTANT, DELIBERATELY. #896 proposes expanding silk by "the
silk-to-body offset the library uses, ~0.2 mm on OLIMEX". That is refuted by the
issue's own acceptance numbers: a 0.2mm expansion on both parts turns
-0.12 / -0.09 / -0.133 into -0.52 / -0.49 / -0.53. Silk at its stroke
centreline, unioned with the pads, reproduces all three exactly.

WHAT THIS MODULE DOES NOT DECIDE
--------------------------------
It reports geometry and its provenance. It does not waive, gate or price
anything, and a body never relaxes a pad-pair clearance -- pads remain the DRC
truth (`legality.grade_pad_legality`, `check_drc.check_pad_pad_overlap`).
"""
from __future__ import annotations

from typing import Dict, NamedTuple, Optional, Tuple

Bbox = Tuple[float, float, float, float]

# The rungs, in order, as they appear in `BodyGeometry.source`. `pad_bbox` and
# `none` keep the spellings `board_brief.parts_section` already ships so a
# reader meets one vocabulary; `courtyard`, `fab` and `silk` name the drawn
# geometry that answered.
SOURCE_COURTYARD = 'courtyard'
SOURCE_FAB = 'fab'
SOURCE_SILK = 'silk'
SOURCE_PADS = 'pad_bbox'
SOURCE_NONE = 'none'

#: Ladder order, for reporting a source mix in a stable sequence.
SOURCES = (SOURCE_COURTYARD, SOURCE_FAB, SOURCE_SILK, SOURCE_PADS,
           SOURCE_NONE)


class BodyGeometry(NamedTuple):
    """One footprint's body, in the footprint's own UNROTATED local frame.

    `body_local` answers *what is this part's body* -- the assembly question, and
    the one #896's numbers are measured against. `occupancy_local` answers *what
    does this part occupy*, which is body united with the pad bbox: a part
    occupies its copper as well as its plastic, and a consumer that decides
    whether something may be SEATED somewhere must never be handed the smaller
    of the two. They are stored as two values rather than one value and a flag
    because they are two questions with two right answers.

    Both are `None` only when the footprint has neither drawn geometry nor pads,
    which is `SOURCE_NONE`; `legality.LocalBounds.synthetic` is the population
    that then falls back to the +-0.5mm fiction.

    `silk_rejected` records that silk was drawn but did not survive -- either
    the footprint has no pads (a logo) or its silk bbox lies inside the pad bbox
    (a pin-1 tick, a polarity bar), so the union is the pad bbox and the source
    reads `pad_bbox`. It is a disclosure, not an error: a fragment must not be
    LABELLED a body.
    """
    ref: str
    body_local: Optional[Bbox]
    occupancy_local: Optional[Bbox]
    source: str
    silk_rejected: bool = False


def _union(a: Bbox, b: Bbox) -> Bbox:
    return (min(a[0], b[0]), min(a[1], b[1]),
            max(a[2], b[2]), max(a[3], b[3]))


def _contained(inner: Bbox, outer: Bbox, eps: float = 1e-9) -> bool:
    return (inner[0] >= outer[0] - eps and inner[1] >= outer[1] - eps
            and inner[2] <= outer[2] + eps and inner[3] <= outer[3] + eps)


def _for_side(sides: Optional[Dict[str, Bbox]],
              side: str) -> Optional[Bbox]:
    """The bbox a part on `side` should use: its own side's if drawn, else the
    other side's, else None.

    Deliberately the same rule as `parser.courtyard_for_side`, and it calls it,
    so the three rungs cannot drift apart on the "library drew only the F side"
    case -- which is common enough that the courtyard reader documents it.
    """
    from placement.parser import courtyard_for_side
    return courtyard_for_side(sides, side)


def body_geometry(fp, side: str,
                  courtyard_sides: Optional[Dict[str, Bbox]] = None,
                  fab_sides: Optional[Dict[str, Bbox]] = None,
                  silk_sides: Optional[Dict[str, Bbox]] = None,
                  ref: str = '') -> BodyGeometry:
    """THE ladder, for one footprint. Per-side bboxes come from the caller.

    Split from `board_bodies` so a consumer that already holds the three
    per-side maps (the quench holds one, `grade_body_overlap` holds two) pays
    the file read once, and so the rung logic has exactly one home.
    """
    from placement.utility import compute_footprint_bbox_local

    ref = ref or getattr(fp, 'reference', '') or ''
    pads: Optional[Bbox] = None
    if getattr(fp, 'pads', None):
        try:
            pads = compute_footprint_bbox_local(fp)
        except Exception:                                    # noqa: BLE001
            pads = None

    drawn = _for_side(courtyard_sides, side)
    if drawn is not None:
        return BodyGeometry(ref, drawn,
                            drawn if pads is None else _union(drawn, pads),
                            SOURCE_COURTYARD)

    drawn = _for_side(fab_sides, side)
    if drawn is not None:
        return BodyGeometry(ref, drawn,
                            drawn if pads is None else _union(drawn, pads),
                            SOURCE_FAB)

    silk = _for_side(silk_sides, side)
    if silk is not None and pads is not None:
        # Rule 1: the silk rung unions. Rule 2 (pad-less refusal) is the
        # `pads is not None` guard above -- a logo reaches the branch below and
        # is recorded as rejected.
        body = _union(silk, pads)
        if _contained(silk, pads):
            # A tick mark: the union IS the pad bbox, so say `pad_bbox`.
            return BodyGeometry(ref, body, body, SOURCE_PADS,
                                silk_rejected=True)
        return BodyGeometry(ref, body, body, SOURCE_SILK)

    if pads is not None:
        return BodyGeometry(ref, pads, pads, SOURCE_PADS,
                            silk_rejected=silk is not None)
    return BodyGeometry(ref, None, None, SOURCE_NONE,
                        silk_rejected=silk is not None)


def board_bodies(pcb_data, pcb_file: Optional[str] = None
                 ) -> Dict[str, BodyGeometry]:
    """`{ref: BodyGeometry}` for a whole board, one file read.

    Needs the board FILE, because footprint graphics reach neither parse path:
    `kicad_parser.Footprint` carries no courtyard, fab or silk field on the text
    side or the pcbnew side. Falls back to `pcb_data.source_path` -- which
    `build_pcb_data_from_board` fills from `board.GetFileName()`, so the GUI
    reaches the same geometry -- and to pad bboxes everywhere when there is no
    readable path at all.

    A ref is ABSENT only when even the pad fallback raised, matching
    `legality.part_local_bounds` so both see one universe.
    """
    from placement.legality import footprint_side
    from placement.parser import (extract_courtyard_sides, extract_fab_sides,
                                  extract_silk_sides)

    path = pcb_file or getattr(pcb_data, 'source_path', None)
    crt: Dict[str, Dict[str, Bbox]] = {}
    fab: Dict[str, Dict[str, Bbox]] = {}
    silk: Dict[str, Dict[str, Bbox]] = {}
    if path:
        for reader, target in ((extract_courtyard_sides, 'crt'),
                               (extract_fab_sides, 'fab'),
                               (extract_silk_sides, 'silk')):
            try:
                got = reader(path)
            except Exception:                                # noqa: BLE001
                got = {}
            if target == 'crt':
                crt = got
            elif target == 'fab':
                fab = got
            else:
                silk = got

    out: Dict[str, BodyGeometry] = {}
    for ref, fp in sorted((pcb_data.footprints or {}).items()):
        geom = body_geometry(fp, footprint_side(fp),
                             courtyard_sides=crt.get(ref),
                             fab_sides=fab.get(ref),
                             silk_sides=silk.get(ref), ref=ref)
        if geom.body_local is None and geom.source != SOURCE_NONE:
            continue
        out[ref] = geom
    return out


def source_mix(bodies) -> Dict[str, int]:
    """`{source: count}` over `BodyGeometry` records, in ladder order.

    The number `check_assembly`'s BODY COVERAGE line reports, so the coverage
    claim and the geometry cannot drift apart.
    """
    counts = {s: 0 for s in SOURCES}
    for g in (bodies.values() if isinstance(bodies, dict) else bodies):
        counts[g.source] = counts.get(g.source, 0) + 1
    return counts


def format_source_mix(bodies) -> str:
    """One line naming every rung that answered, zeros included.

    Zeros are kept deliberately: "0 courtyard" on a board is the finding, and a
    mix that only lists what it found cannot say that.
    """
    counts = source_mix(bodies)
    return ', '.join(f'{counts[s]} {s}' for s in SOURCES)
