"""When a board cannot hold its parts, say so WITH THE NUMBER.

`grow_board` measures the board's area utilisation per face and what outline
would hold the parts; `floorplan`'s overfull and crowded rules read it. A board
that is genuinely too small is reported with the measured number, so the
person reading it knows what to change.

`out_of_board_area` is not that number: a part sitting inside a cutout scores
0.0 area, which is why `floorplan` disqualifies it as a budget key.

THIS REPORTS; IT NEVER REFUSES AND NEVER ACTS: `ran` is False only when the
evidence is missing, and the numbers themselves never refuse. A wrong
auto-action fails invisibly.

Nothing here writes Edge.Cuts or a stackup. The only two things in the tree
that write an outline stay `kicad_writer.strip_zero_length_edge_cuts` and the
corpus tool `fix_outline_gaps.py`. This proposes with a number, exactly as
`recommend-stackup/SKILL.md:44` already does: "propose the nearest workable
option... Do not modify the board file directly -- stackup is a fab-facing
decision the user must own."

`grow_board` returns the house shape:

    ran        bool -- False means NOT MEASURED, never "fine"
    reason     why it did not run
    measured   what this board is
    expected   what it would have to be
    action     the sentence or argv a human would act on, never run here
"""
from __future__ import annotations

import math
from typing import Dict, Mapping, Optional

# A board packed tighter than this by courtyard area is worth remarking on even
# when every part currently fits: the parts have to be ROUTED between, and the
# corpus's own comfortable boards sit far below it. Deliberately advisory --
# see the module docstring on why nothing here refuses.
CROWDED_UTILISATION = 0.55

# A declared ZONE whose members' courtyards cover this much of it gets an
# advisory (`plan_zone_utilisation`, #1468). Read off measured zones of
# 0603-class parts: every zone seen failing to seat all its members was at
# 57% or more (57% in 2 of 12 seeds, 60% and 77% in every seed, 69% in every
# seed of two rounds), and zones at about 41% and 50% seated every member.
# 0.50 flags each failure seen; a zone at 72% that fitted is flagged too.
# A heuristic, never an error: area is necessary, not sufficient.
ZONE_FIT_UTILISATION = 0.50


def _skip(reason: str) -> Dict:
    return {'ran': False, 'reason': reason}


#: The two deficit fields `deficit_totals` will sum. An ALLOWLIST rather than
#: a getattr with a default, for the reason in that function's docstring: a
#: silent 0 from a mistyped field name is precisely the failure it exists
#: about, and adding a parameter would have reopened it.
_DEFICIT_FIELDS = ('deficit', 'deficit_floor')


def deficit_totals(ledgers, *, field: str = 'deficit') -> Dict[str, int]:
    """(lanes short in total, parts with any short face, parts examined).

    Read off `FaceLedger.deficit` directly. `PartEscape` exposes `worst` (a
    FaceLedger or None) and NO `worst_deficit` attribute -- that name exists
    only as a key in its `to_dict()`. A `getattr(p, 'worst_deficit', 0)`
    therefore always returns the default, silently, and turns "38 faces are
    short" into "0", which is the exact shape of failure this repo names
    elsewhere: a component nothing examined reported as clean.

    `field='deficit_floor'` sums #700's layer-aware LOWER bound instead --
    lanes short even using every other signal layer. `parts` is counted as
    "any face short" rather than through `worst`, which is provably the same
    answer for `deficit` (`worst` is None exactly when no face is short), so
    no existing caller's number moves.
    """
    if field not in _DEFICIT_FIELDS:
        raise ValueError(f"unknown deficit field {field!r}; "
                         f"known: {_DEFICIT_FIELDS}")
    lanes = parts = 0
    for p in ledgers:
        vals = [getattr(f, field) for f in (getattr(p, 'faces', ()) or ())]
        if any(v > 0 for v in vals):
            parts += 1
        lanes += sum(vals)
    return {'lanes': lanes, 'parts': parts, 'examined': len(ledgers)}


# A container must HOST this fraction of the other parts to count as one.
HOSTS_FRACTION = 0.30


def hosts_the_design(ref, gx0, gy0, gx1, gy1, fp, footprints) -> bool:
    """Is this big footprint a frame around the design, or just a big part?

    Area alone is not enough, and getting that wrong is worse than not
    excluding at all. `CONTAINER_RATIO` is relative to the BOARD, so on a
    genuinely too-small board ordinary connectors cross it -- measured on a
    synthetically shrunk splitflap, 12 plain connectors were classed as
    containers, which would have suppressed the very parts causing the
    shortfall and understated the answer this option exists to give.

    A real container hosts the design: other parts sit INSIDE it. rp2350's U8
    contains most of the board; a big connector on a small board contains
    nothing.

    Two conditions beyond that, each added because the first version of this
    guard failed the same way it was written to prevent:

    * **SAME SIDE ONLY.** Utilisation is per-side, so a part on F.Cu cannot
      host one on B.Cu. Layer-blind, an 18x25.5mm module on F.Cu was
      "hosting" twenty 0402s underneath it on B.Cu and got excluded, and a
      board whose true F.Cu utilisation was 1.10 reported `fits` at 0.55.
    * **NOT A PILE.** On an unplaced board every part shares one coordinate,
      so ANY part covering that point hosts 100% of the design. Measured on a
      shrunk, piled haasoscope: a BGA-256 and a BGA-529 were classed as
      containers and a real 524.86mm2 shortfall was reported as no shortfall
      at all. Two BGAs are not "a frame, not a body". Require the hosted
      parts to occupy several DISTINCT positions, which a pile cannot.
    """
    x0 = fp.x + gx0
    y0 = fp.y + gy0
    x1 = fp.x + gx1
    y1 = fp.y + gy1
    side = getattr(fp, 'layer', None) or 'F.Cu'
    others = [o for r, o in footprints.items()
              if r != ref and (getattr(o, 'layer', None) or 'F.Cu') == side]
    if len(others) < 4:
        return False
    hosted = [o for o in others if x0 <= o.x <= x1 and y0 <= o.y <= y1]
    if len(hosted) < HOSTS_FRACTION * len(others):
        return False
    # A frame hosts a LAYOUT, not a stack. Distinct positions is what
    # separates the two, and it is exactly what a pile does not have.
    spots = {(round(o.x, 3), round(o.y, 3)) for o in hosted}
    return len(spots) >= max(4, 0.5 * len(hosted))


#: Public since #709: the pocket census (`check_pockets`, since retired)
#: excluded the same frames from its area-weighted centroid, and
#: `board_brief` records the repo's rule that an artifact must not sit behind
#: a private cross-module name. The old spelling
#: stays so nothing in-tree has to change in the same commit.
_hosts_the_design = hosts_the_design


#: WHICH rect a through-hole part presents on the face it is not mounted on
#: (#878). Named once, so the arithmetic below and the disclosure string cannot
#: drift apart -- they are the two halves of one claim.
#:
#: Decided by measurement, not preference: `tests/measure_878_far_face_area.py`
#: swept three candidate currencies over the tracked corpus and its
#: pre-registered rule selected this one. `legality.rect_on` is the only place
#: in the tree where the far-face rect is DECIDED rather than sidestepped, and
#: it answers the drilled-pad rect; `check_pockets.courtyard_cover` (since
#: retired) charged the whole courtyard, but it answered a per-window COVER
#: question where summing never arises, so it was not a competing answer.
#: The measurement's falsifier: charging the whole courtyard makes
#: flat_hierarchy's far charge equal its near charge part-for-part (64 of 64
#: pad-bearing parts drilled), which is the same area counted twice.
FAR_FACE_BASIS = ('the drilled-pad rect, the same rect legality.rect_on '
                  'presents on the far side')


def _far_face_area(fp, clearance: float) -> float:
    """mm2 a drilled part obstructs on the face it is NOT mounted on.

    0.0 when the part reaches no other face. The SAME `+clearance` on each axis
    as the near charge at `grow_board`'s `a = (w + clearance) * (h + clearance)`
    -- charged differently, the two would not be commensurate and the per-side
    sums could not be compared with each other or with `usable`.
    """
    from placement.legality import (footprint_has_through_pads,
                                    rotate_local_bounds,
                                    through_pad_bounds_local)
    if not footprint_has_through_pads(fp):
        return 0.0
    tb = through_pad_bounds_local(fp)
    if tb is None:      # same drill>0 predicate as the guard above; belt only
        return 0.0
    tx0, ty0, tx1, ty1 = rotate_local_bounds(*tb, fp.rotation or 0.0)
    return (tx1 - tx0 + clearance) * (ty1 - ty0 + clearance)


def grow_board(pcb_data, pcb_file: str, *, clearance: float,
               board_edge_clearance: float,
               assembly_sides: Optional[str] = None,
               side_overrides: Optional[Mapping[str, str]] = None) -> Dict:
    """Does the total part area fit inside the outline, and by how much not?

    This is the number the prohibition asks for and nobody computes. It is
    deliberately a NECESSARY-condition test, not a sufficient one: parts whose
    courtyards sum to more than the usable area certainly do not fit, while
    parts that sum to less may still not fit for shape reasons. Reporting it
    as "at least this much more" rather than "this much more" is the honest
    reading, and the field is named so.

    Courtyard area is used where a footprint has one and the pad bounding box
    where it does not, and the count of each is reported: a pad bbox carries
    no courtyard margin, so a board made of them reads smaller than it builds.

    `assembly_sides` (#837) is the board's declared assembly policy -- 'F',
    'B', 'both', or None for "nobody declared". It decides WHICH SIDE'S AREA
    the parts are charged against, and that is the whole of its effect:

      * None or 'both' -> `max(per_side)`, bit-identical to every number this
        function reported before the key existed. A part on B.Cu does not
        compete for F.Cu area.
      * 'F' or 'B'     -> `sum(per_side)`. Every part has to fit on the one
        face the fab will populate, so the demand on it is the total.

    The two agree exactly when the far face is empty, which is why declaring
    the OBSERVED policy is inert: 15 of the 22 tracked boards have zero
    back-side area. It moves the 7 that do not, and flips the verdict on the
    two whose sum exceeds their usable area (orangecrab_ext_pll 0.92 -> 1.25,
    ulx3s 0.90 -> 1.34).

    `side_overrides` is the plan-check projection of the declared faces
    (`floorplan.declared_side_claims`). A plan is checked before its
    placement exists,
    so its necessary area bound must charge a member to the face the plan
    requires rather than to the generator pile's incidental current layer.
    Ordinary option callers omit it and continue to measure the board as-is.
    """
    from placement.parser import extract_courtyard_bboxes
    bi = pcb_data.board_info
    if bi.board_bounds is None:
        return _skip('the board has no outline to measure against')
    x0, y0, x1, y1 = bi.board_bounds
    usable_w = max(0.0, (x1 - x0) - 2 * board_edge_clearance)
    usable_h = max(0.0, (y1 - y0) - 2 * board_edge_clearance)
    usable = usable_w * usable_h

    try:
        cy = extract_courtyard_bboxes(pcb_file) or {}
    except Exception as e:                       # noqa: BLE001
        return _skip(f'courtyards unreadable: {type(e).__name__}: {e}')

    total = 0.0
    from_courtyard = from_pads = no_geometry = 0
    biggest = []
    from placement.legality import (CONTAINER_RATIO, footprint_side,
                                    rotate_local_bounds)
    from placement.utility import compute_footprint_bbox_local
    from kicad_parser import non_aperture_pads
    per_side = {'F.Cu': 0.0, 'B.Cu': 0.0}
    # #878. TWO dicts, because they answer two different questions and the
    # tree needed both. `per_side` is the POPULATION charge -- each part once,
    # on the face a reflow pass places it on -- and it is what the one-face
    # sum below reads. `obstructed` is the OBSTRUCTION charge: a through-hole
    # part's leads come out on the far face and block it there too, and that
    # area was charged to NOBODY. `busiest` reads this one, because what a
    # face can still hold is what is not already blocked on it.
    obstructed = {'F.Cu': 0.0, 'B.Cu': 0.0}
    far_area = 0.0
    far_parts = 0
    containers = []
    outline_area = (x1 - x0) * (y1 - y0)
    for ref, fp in sorted(pcb_data.footprints.items()):
        box = cy.get(ref)
        if box:
            from_courtyard += 1
        elif non_aperture_pads(fp):     # aperture-only pads are not pads (#1143)
            # NOT max(global_x) - min(global_x): the pad-CENTRE span omits the
            # pads' own size, so a two-pad part measures ZERO width along its
            # pad axis (esp_prog C1: [0.0, 1.778] against a real 2.794x1.016)
            # and the sum understated total part area by 51% on that board.
            box = compute_footprint_bbox_local(fp)
            from_pads += 1
        else:
            no_geometry += 1
            continue
        # Both sources are footprint-LOCAL; rotate into board space or every
        # 90/270 part is transposed.
        gx0, gy0, gx1, gy1 = rotate_local_bounds(*box, fp.rotation or 0.0)
        w, h = gx1 - gx0, gy1 - gy0
        # A CONTAINER is a module-outline footprint hosting the design -- a
        # frame, not a body -- and charging its area against the board is how
        # this option told a SHIPPING board it does not fit. Measured on
        # rp2350_fpga_eensy: U8's bbox is 1.15x the whole board and 66% of the
        # busiest side, giving utilisation 1.91 and a demand for 526.67 mm2
        # more area; without it the board reads 0.64 and fits. Same ratio and
        # the same calibration part as legality.CONTAINER_RATIO, which exists
        # for exactly this and which this function was not consulting.
        if (outline_area > 0 and (w * h) >= CONTAINER_RATIO * outline_area
                and _hosts_the_design(ref, gx0, gy0, gx1, gy1, fp,
                                      pcb_data.footprints)):
            containers.append((round(w * h, 2), ref))
            continue
        # Each part also needs clearance around it; charging half the
        # clearance per side is what makes the sum comparable to the usable
        # area rather than to the parts' bare footprints.
        a = (w + clearance) * (h + clearance)
        total += a
        # `footprint_side` rather than the raw `fp.layer` string, so this
        # partition cannot grow a third key that `max()` would then silently
        # rank against the other two. Inert on the corpus -- every tracked
        # board's footprints are on F.Cu or B.Cu -- and the point is that it
        # stays inert. Same helper the census, the rule and every other
        # per-side instrument in the tree read.
        planned_side = ((side_overrides or {}).get(ref)
                        or footprint_side(fp))
        layer = 'B.Cu' if planned_side == 'B' else 'F.Cu'
        per_side[layer] = per_side.get(layer, 0.0) + a
        # #878. `layer` again rather than a second `footprint_side` call: ONE
        # side decision per part, so the near and far charges cannot land on
        # inconsistent faces.
        obstructed[layer] = obstructed.get(layer, 0.0) + a
        far = _far_face_area(fp, clearance)
        if far > 0.0:
            other = 'F.Cu' if layer == 'B.Cu' else 'B.Cu'
            obstructed[other] = obstructed.get(other, 0.0) + far
            far_area += far
            far_parts += 1
        # NOT the far charge: `biggest` feeds `largest_parts_mm2`, which
        # answers "which part is big enough that its SIZE is the constraint".
        # That is a question about a part's own body, so adding its leads'
        # far-face area would charge the same part twice.
        biggest.append((round(a, 2), ref))
    biggest.sort(reverse=True)

    # Utilisation is PER SIDE. Summing both sides against one side's usable
    # area is why this told the shipping OrangeCrab it does not fit and
    # recommended shrinking a working board: F.Cu 999.94mm2 and B.Cu 364.17
    # against 1088.58 usable -- each side fits, their sum (1364.11, util
    # 1.2531) does not. A part on B.Cu does not compete for F.Cu area.
    # (ulx3s flips the same way, 1.3373 -> 0.9018.)
    # #878: the busiest side is the most OBSTRUCTED face, not the most
    # POPULATED one. A face's remaining capacity is what is not already blocked
    # on it, and a through-hole part blocks both. On the one-face basis below
    # nothing changes, and that is not an oversight -- see `charged`.
    busiest = max(obstructed.values()) if obstructed else 0.0
    # #837. WHICH area the utilisation is computed from. `busiest` keeps its
    # own key on both bases and stops being the number the verdict rests on
    # rather than becoming a lie. #878 changed WHAT it is the busiest of: the
    # busiest OBSTRUCTED face, not the busiest populated one. It is still "the
    # busiest side's area" only if that phrase means obstruction, which is why
    # it is spelled out here rather than left to the reader.
    one_face = assembly_sides in ('F', 'B')
    # #878: `per_side`, deliberately, and it is the half of this that is easy
    # to get wrong. Under a one-face policy `sum(per_side)` is each part
    # EXACTLY ONCE -- the demand on the single face the fab populates. A
    # through-hole part's leads come out on the face nobody populates, so they
    # compete with nothing there, and adding them here would be the same area
    # counted twice. Measured on the tracked corpus: charging the far face
    # into this sum double-charges 10 of the 15 one-face boards, worst
    # flat_hierarchy 5927.41 -> 9404.40 mm2. So the far charge lives in
    # `busiest` alone. `tests/878_far_face_currency.json` records it as
    # `post_hoc.one_face_charged_once`, which is True only when nothing is
    # charged far.
    charged = sum(per_side.values()) if one_face else busiest
    util = (charged / usable) if usable > 0 else float('inf')
    fits = charged <= usable
    out = {
        'ran': True,
        'measured': {
            'part_area_mm2': round(total, 2),
            # Both sides, as POPULATED. Reported because `part_area_mm2` alone
            # cannot be checked against `utilisation` on a two-sided board,
            # and a reader who tries will conclude the number is wrong.
            #
            # #878: on the busiest basis `utilisation` is NO LONGER computed
            # from this dict -- it comes from `obstructed_area_by_side_mm2`
            # below, and the two differ on any board with a drilled part. This
            # comment used to say "and the one utilisation is computed from",
            # which after #878 sent a reader to the wrong key and produced
            # exactly the disagreement the key exists to prevent
            # (rp2350_fpga_eensy_prePlane: max(this)/usable is 0.6220 against a
            # reported 0.6566). Reconcile against `obstructed_...` on the
            # busiest basis and against this one under a declared `F`/`B`.
            'part_area_by_side_mm2': {k: round(v, 2)
                                      for k, v in sorted(per_side.items())},
            # #878. What each face is OBSTRUCTED by, which is not what each
            # face is POPULATED with: a drilled part's leads block the far
            # side. `part_area_by_side_mm2` above stays the population charge
            # and still sums to `part_area_mm2`; this one does not, and the
            # difference is exactly `far_face_area_mm2`.
            'obstructed_area_by_side_mm2': {
                k: round(v, 2) for k, v in sorted(obstructed.items())},
            'busiest_side_area_mm2': round(busiest, 2),
            'far_face_area_mm2': round(far_area, 2),
            'far_face_parts': far_parts,
            # The human channel for the basis is `not_modelled`, at option
            # level -- the same split #837 made between the
            # `charged_area_is_sum` bool and the prose.
            'far_face_basis': FAR_FACE_BASIS,
            # #837: the area `utilisation` is ACTUALLY computed from, and a
            # BOOL saying which rule produced it.
            'charged_area_mm2': round(charged, 2),
            'charged_area_is_sum': one_face,
            'assembly_sides_declared': assembly_sides,
            'usable_area_mm2': round(usable, 2),
            'outline_area_mm2': round((x1 - x0) * (y1 - y0), 2),
            'utilisation': round(util, 4),
            'parts': len(pcb_data.footprints),
            # Named, never silently dropped: a reader must be able to see
            # WHY a 731mm2 part is absent from the sum.
            'containers_excluded': [r for _a, r in sorted(containers,
                                                          reverse=True)],
            'container_area_mm2': round(sum(a for a, _r in containers), 2),
            'extent_from_courtyard': from_courtyard,
            'extent_from_pad_bbox': from_pads,
            'parts_without_geometry': no_geometry,
            'largest_parts_mm2': biggest[:5],
        },
        # AT OPTION LEVEL, beside `action`: inside `measured` it would read
        # as one more number.
        #
        # #878, and what is left after it. A through-hole part's leads ARE now
        # charged on the far face, at `FAR_FACE_BASIS` -- the drilled-pad rect,
        # the model `legality.rect_on` already enforces for the placement
        # search. The other per-face instrument #878 named,
        # `check_pockets.courtyard_cover`, charged the whole courtyard on both
        # faces instead; it has since been retired, so that disagreement is
        # gone. (The issue's own table also named
        # `floorplan.rule_keepout` as a third model; it is not one -- it uses
        # `part.sides` only to BIND which keep-outs apply, then hands
        # `keepout_hit` both rects and takes a max, consulting no side.)
        #
        # 17 of the 22 tracked boards carry through-hole parts. Measured on
        # the busiest-side basis, this moves `busiest` on ONE of them
        # (rp2350_fpga_eensy_prePlane, utilisation 0.6220 -> 0.6566) and flips
        # no verdict; the whole-courtyard alternative moves two and flips none
        # either. The one-face basis is deliberately untouched -- see
        # `charged` above. `tests/878_far_face_currency.json` is the sweep.
        'not_modelled': '; '.join(
            # BASIS-SPECIFIC, because the far charge does not reach this
            # board's number on every basis, and a blanket sentence would
            # claim it does. #878's own pre-registered rule said the
            # disclosure had to become basis-specific if a basis kept
            # `none`; the one-face basis does, by the post-hoc finding above.
            # And it is dropped entirely when the board has no drilled part:
            # on qfn_interior_pads or routed_output the old wording claimed
            # two models "differ by that much" when they differ by nothing.
            ([] if not far_parts else
             [f'a through-hole part is charged {FAR_FACE_BASIS}, not its '
              f'whole courtyard (#878: the model the placement search '
              f'itself enforces)'
              + (' -- and NONE of it reaches `utilisation` here, because a '
                 'declared one-face policy charges each part exactly once on '
                 'the face the fab populates, where the leads compete with '
                 'nothing' if one_face else '')])
            # The container blind spot, named rather than left to be found.
            # A container is exempted BEFORE the far charge, and the same
            # exemption is in the sweep that chose the currency -- so no arm
            # of that measurement could see it. On rp2350_fpga_eensy_prePlane
            # the excluded U8 is a through-hole module whose drilled-pad box
            # equals its courtyard: 728.3 mm2 on the far face, 7x the whole
            # board's reported far charge. Charging it would take that board
            # to utilisation 1.60 and `fits_by_area` False, on hardware that
            # ships. The near-face exemption's argument -- a frame hosts the
            # parts inside it -- does NOT transfer to the far face, where
            # nothing is hosted and the pins are real copper. So this is an
            # open question, disclosed, not a settled one.
            + ([] if not containers else
               [f'the far-face area of {len(containers)} excluded container(s)'
                f' ({", ".join(r for _a, r in sorted(containers, reverse=True))})'
                f' -- they are exempted before the charge, and the #878 sweep'
                f' shares that exemption, so it did not measure them'])
            + ([] if assembly_sides else [
                'whether the back side will be POPULATED -- no assembly.sides '
                'was declared, so the busier face is charged and back-side '
                'area is credited free (#837)'])),
        'expected': {'utilisation': f'<= 1.0 to fit at all, and typically '
                                    f'<= {CROWDED_UTILISATION} to route'},
        'fits_by_area': fits,
    }
    if not fits:
        need = charged - usable
        # Square-ish growth is the cheapest way to state it; a real outline
        # change is a mechanical decision and this does not pretend otherwise.
        #
        # The side must be solved for USABLE area, not outline area. It was
        # sqrt(outline_area + need), where `need` is a usable-area shortfall --
        # so the proposed board's usable area, (side - 2*edge)^2, was still
        # short. Measured: it proposed 55.1mm square for parts needing
        # 2971.0mm2 usable, which yields 2920.3 -- still 50.7mm2 under. A
        # square whose USABLE area holds the parts has side sqrt(busiest)
        # plus the edge clearance it loses on each side.
        # ROUND UP to the published precision, then keep a hair of margin.
        # sqrt(busiest) + 2*edge makes (side - 2*edge)^2 == busiest EXACTLY,
        # and the action string publishes `{side:.1f}` -- so any downward
        # rounding ships a proposal that is short. Measured across the corpus:
        # 8 of 22 boards were short by 0.9-3.1 mm2, and the test that checks
        # this passed only because splitflap's 55.1897 happens to round up.
        exact = math.sqrt(max(0.0, charged)) + 2.0 * board_edge_clearance
        side = math.ceil(exact * 10.0) / 10.0
        out['measured']['shortfall_mm2_at_least'] = round(need, 2)
        # The number, not just the prose. The proposal was reachable only by
        # parsing the action string, which is also what the test audited -- so
        # the audit read the same rounded text it was meant to check.
        out['measured']['proposed_square_side_mm'] = side
        _basis = (f"on {assembly_sides}.Cu, which must carry every part"
                  if one_face else "on the busiest side")
        out['action'] = (
            f"the parts need AT LEAST {need:.1f} mm2 more usable area than "
            f"this outline has ({charged:.1f} vs {usable:.1f} mm2 "
            f"{_basis}). A square "
            f"board holding them would be about {side:.1f} x {side:.1f} mm "
            f"against today's {x1 - x0:.1f} x {y1 - y0:.1f}. The outline is a "
            f"mechanical decision: this reports it and changes nothing.")
    elif util >= CROWDED_UTILISATION:
        out['action'] = (
            f"the parts fit by area ({util:.0%} of usable) but leave little "
            f"room to route between them. Consider a larger outline or a "
            f"denser layer stack before blaming the placement.")
    return out
