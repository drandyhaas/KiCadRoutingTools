"""When a board cannot hold its parts, say so WITH THE NUMBER and the options.

The prohibition this serves is written down in six places -- docs/floorplan-
intent.md:29-40, floorplan.py:14-19, check_floorplan.py:19-24,
rule_envelope:692, and both placement skills -- and every one of them says the
same sentence:

    the honest response to a board that is genuinely too small is to say so
    with the measured number and stop

The number was never computed. `out_of_board_area` is the nearest quantity and
`floorplan.py:347-360` explicitly DISQUALIFIES it as a budget key, because a
part sitting inside a cutout scores 0.0 area. Nothing sums a required area,
nothing re-runs a lane ledger at another layer count, and nothing converts a
face deficit into the millimetres of span that would clear it. So a run that
hits a genuinely too-small board can only stop, and the person reading it has
to go and measure by hand what to change.

THIS REPORTS; IT NEVER REFUSES AND NEVER ACTS. That is the house pattern from
`placement_driver._guard_congestion:779-823` -- "ok is False ONLY when the
evidence is missing; the numbers themselves never refuse... The executor
decides" -- and from `lock_advisor`, which prints a paste-ready `--lock` and
locks nothing, because a wrong auto-action fails invisibly.

Nothing here writes Edge.Cuts or a stackup. The only two things in the tree
that write an outline stay `kicad_writer.strip_zero_length_edge_cuts` and the
corpus tool `fix_outline_gaps.py`. This proposes with a number, exactly as
`recommend-stackup/SKILL.md:44` already does: "propose the nearest workable
option... Do not modify the board file directly -- stackup is a fab-facing
decision the user must own."

Every option carries the house shape:

    ran        bool -- False means NOT MEASURED, never "fine"
    reason     why it did not run
    measured   what this board is
    expected   what it would have to be
    action     the sentence or argv a human would act on, never run here
"""
from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence

# A board packed tighter than this by courtyard area is worth remarking on even
# when every part currently fits: the parts have to be ROUTED between, and the
# corpus's own comfortable boards sit far below it. Deliberately advisory --
# see the module docstring on why nothing here refuses.
CROWDED_UTILISATION = 0.55


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


def _hosts_the_design(ref, gx0, gy0, gx1, gy1, fp, footprints) -> bool:
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


def grow_board(pcb_data, pcb_file: str, *, clearance: float,
               board_edge_clearance: float) -> Dict:
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
    from placement.legality import CONTAINER_RATIO, rotate_local_bounds
    from placement.utility import compute_footprint_bbox_local
    per_side = {'F.Cu': 0.0, 'B.Cu': 0.0}
    containers = []
    outline_area = (x1 - x0) * (y1 - y0)
    for ref, fp in sorted(pcb_data.footprints.items()):
        box = cy.get(ref)
        if box:
            from_courtyard += 1
        elif fp.pads:
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
        layer = getattr(fp, 'layer', None) or 'F.Cu'
        per_side[layer] = per_side.get(layer, 0.0) + a
        biggest.append((round(a, 2), ref))
    biggest.sort(reverse=True)

    # Utilisation is PER SIDE. Summing both sides against one side's usable
    # area is why this told the shipping OrangeCrab it does not fit and
    # recommended shrinking a working board: F.Cu 999.94mm2 and B.Cu 364.17
    # against 1088.58 usable -- each side fits, their sum (1364.11, util
    # 1.2531) does not. A part on B.Cu does not compete for F.Cu area.
    # (ulx3s flips the same way, 1.3373 -> 0.9018.)
    busiest = max(per_side.values()) if per_side else 0.0
    util = (busiest / usable) if usable > 0 else float('inf')
    fits = busiest <= usable
    out = {
        'ran': True,
        'measured': {
            'part_area_mm2': round(total, 2),
            # Both sides, and the one utilisation is computed from. Reported
            # because `part_area_mm2` alone cannot be checked against
            # `utilisation` on a two-sided board, and a reader who tries will
            # conclude the number is wrong.
            'part_area_by_side_mm2': {k: round(v, 2)
                                      for k, v in sorted(per_side.items())},
            'busiest_side_area_mm2': round(busiest, 2),
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
        'expected': {'utilisation': f'<= 1.0 to fit at all, and typically '
                                    f'<= {CROWDED_UTILISATION} to route'},
        'fits_by_area': fits,
    }
    if not fits:
        need = busiest - usable
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
        exact = math.sqrt(max(0.0, busiest)) + 2.0 * board_edge_clearance
        side = math.ceil(exact * 10.0) / 10.0
        out['measured']['shortfall_mm2_at_least'] = round(need, 2)
        # The number, not just the prose. The proposal was reachable only by
        # parsing the action string, which is also what the test audited -- so
        # the audit read the same rounded text it was meant to check.
        out['measured']['proposed_square_side_mm'] = side
        out['action'] = (
            f"the parts need AT LEAST {need:.1f} mm2 more usable area than "
            f"this outline has ({busiest:.1f} vs {usable:.1f} mm2 on the "
            f"busiest side). A square "
            f"board holding them would be about {side:.1f} x {side:.1f} mm "
            f"against today's {x1 - x0:.1f} x {y1 - y0:.1f}. The outline is a "
            f"mechanical decision: this reports it and changes nothing.")
    elif util >= CROWDED_UTILISATION:
        out['action'] = (
            f"the parts fit by area ({util:.0%} of usable) but leave little "
            f"room to route between them. Consider a larger outline or a "
            f"denser layer stack before blaming the placement.")
    return out


def add_layers(pcb_data, pcb_file: str, *, clearance: float,
               track_width: Optional[float] = None,
               step: int = 2) -> Dict:
    """Would more copper layers clear the escape-lane deficits?

    Two effects, and only one of them is modelled here honestly:

    * a finer FAB FLOOR at a higher layer count narrows the lane pitch, so a
      face supplies more lanes. That is computable, and it is what this
      reports -- `fab_tiers.fab_floor_min` already indexes on layer count and
      `check_channels.py:160-170` ran exactly this experiment by hand at three
      clearances (supply 29 -> 120 -> 242).
    * nets escaping on OTHER layers relieve a face entirely. That needs a
      routing attempt, so it is named as not modelled rather than guessed.
    """
    from placement.escape import escape_ledger, lane_pitch  # noqa: F401
    try:
        from fab_tiers import (count_copper_layers_in_data,
                               count_copper_layers_in_file, fab_floor_bucket,
                               fab_floor_min)
    except ImportError as e:
        return _skip(f'fab_tiers unavailable: {e}')

    # File first (it is what a saved board is), then the parsed board. The
    # file counter returns 0 for an unsaved / in-memory board whose
    # `board_info.copper_layers` is perfectly good, and this option SKIPPED
    # on every one of those -- reporting "could not count" about a board whose
    # layer list was in the argument it was handed.
    n = count_copper_layers_in_file(pcb_file) or count_copper_layers_in_data(
        pcb_data)
    if not n:
        return _skip('could not count this board\'s copper layers')
    try:
        now = fab_floor_min(n)
        more = fab_floor_min(n + step)
    except Exception as e:                       # noqa: BLE001
        return _skip(f'no fab floor for {n} or {n + step} layers: {e}')

    # WHY the two floors are the same, when they are. `same_floor` below says
    # they match; only the bucket says whether that is a measurement or a
    # modelling limit, and this option printed the first while meaning the
    # second on every board above 2 copper layers (#700).
    b_now, b_more = fab_floor_bucket(n), fab_floor_bucket(n + step)
    layer_blind = (b_now.bucket == b_more.bucket)

    def _deficit(clr, tw, layers=None, field='deficit'):
        return deficit_totals(
            escape_ledger(pcb_data, pcb_file=pcb_file, clearance=clr,
                          track_width=tw, signal_layers=layers), field=field)

    # BOTH sides must be measured at their own fab floor, or the comparison
    # is not about layers at all. Measuring "now" at the board's clearance
    # and "more" at the n+2 fab floor reported 43 -> 12 lanes on a board
    # where the two floors are IDENTICAL -- the whole apparent gain was the
    # clearance change, credited to the layer count.
    same_floor = (now.get('clearance') == more.get('clearance')
                  and now.get('track_width') == more.get('track_width'))
    try:
        d_now = _deficit(now.get('clearance', clearance),
                         now.get('track_width', track_width))
        d_more = _deficit(more.get('clearance', clearance),
                          more.get('track_width', track_width))
        d_board = _deficit(clearance, track_width)
        # #700 item 2: the effect this option used to name as NOT MODELLED --
        # nets escaping on the new layers instead of through a face -- now has
        # a bound. Both sides at the board's own clearance, so the only thing
        # that moves between them is the layer count.
        f_now = _deficit(clearance, track_width, layers=n,
                         field='deficit_floor')
        f_more = _deficit(clearance, track_width, layers=n + step,
                          field='deficit_floor')
    except Exception as e:                       # noqa: BLE001
        return _skip(f'the escape ledger did not run: {type(e).__name__}: {e}')

    if d_now['examined'] == 0:
        return _skip('this board has no fine-pitch part, so there is no '
                     'escape-lane question to answer')

    out = {
        'ran': True,
        'measured': {
            'copper_layers': n,
            'floor_now': {k: now.get(k) for k in ('track_width', 'clearance')},
            'floor_at_more': {k: more.get(k)
                              for k in ('track_width', 'clearance')},
            # `_now` means AT THIS BOARD'S CLEARANCE -- what the board is
            # actually short of today. It used to mean "at the fab floor for
            # this layer count", which reported tigard as 0 while
            # move_blocker, in the same document, named 41 deficit lanes.
            # Two different questions cannot share the word "now".
            'deficit_lanes_now': d_board['lanes'],
            'deficit_parts_now': d_board['parts'],
            'deficit_lanes_at_fab_floor': d_now['lanes'],
            'deficit_parts_at_fab_floor': d_now['parts'],
            'deficit_lanes_at_more': d_more['lanes'],
            'deficit_parts_at_more': d_more['parts'],
            'deficit_lanes_at_board_clearance': d_board['lanes'],
            'floors_differ': not same_floor,
            'fine_pitch_parts': d_now['examined'],
            # WHY the floors are the same, when they are. `floors_differ` is
            # the observation; these say whether it is a fact about fabs or
            # about this table. `fab_floor_layer_blind` is the headline and is
            # forced into the text digest.
            'fab_bucket_now': b_now.bucket,
            'fab_bucket_at_more': b_more.bucket,
            'fab_floor_layer_blind': layer_blind,
            'fab_bucket_saturated': b_now.saturated,
            'fab_buckets_modelled': list(b_now.buckets),
            # #700 item 2. A LOWER bound on the deficit: lanes short even
            # when every other signal layer is counted. Both at the board's
            # own clearance, so the layer count is the only thing that moves.
            'deficit_floor_lanes_now': f_now['lanes'],
            'deficit_floor_lanes_at_more': f_more['lanes'],
            # COPPER layers, and named so. `add_layers` asks about a stackup
            # this board does not have, so it hands the ledger the copper
            # count as the signal count -- which OVER-states on a poured
            # board (kit-dev-coldfire: 4 copper layers, 1 signal). Safe for a
            # LOWER bound, since more assumed layers can only shrink
            # `deficit_floor`, but it is not the ledger's own `signal_layers`
            # and must not borrow its name.
            'copper_layers_assumed_signal_now': n,
            'copper_layers_assumed_signal_at_more': n + step,
        },
        'expected': {'deficit_lanes': 0},
        'not_modelled': (
            # The escape-onto-other-layers effect is now BOUNDED, not
            # unmodelled -- so this says what is left, which is whether the
            # router would actually take that escape.
            'whether a net that COULD escape on another layer actually does '
            '-- that needs a routing attempt; the bound is '
            'deficit_floor_lanes_*'
            + ('' if not layer_blind else
               f". AND, above {b_now.buckets[-1]} copper layers, any fab-floor "
               f"difference at all -- fab_tiers models buckets "
               f"{list(b_now.buckets)} and nothing finer")),
    }
    # The BRANCHES moved to the board clearance; the prose must move with
    # them. Quoting d_now (the fab floor) inside a branch chosen by d_board
    # reintroduces the same "two questions, one word" defect one level down:
    # measured on esp_prog at clearance 0.6, the else-branch printed "0
    # deficit lane(s) remain" while d_board['lanes'] was 1.
    gain = d_now['lanes'] - d_more['lanes']
    if layer_blind:
        # NOT `same_floor`. The two coincide on today's table, but they are
        # different claims: `same_floor` compares two dicts, while this asks
        # whether the comparison was CAPABLE of differing. Saying "more layers
        # buy NO extra lanes" off the first is how a 6-layer board 121 escape
        # lanes short was told to stop asking (#700).
        out['action'] = (
            f"this comparison is STRUCTURALLY BLIND above {b_now.bucket} "
            f"copper layers. fab_tiers models {len(b_now.buckets)} layer "
            f"bucket(s) ({', '.join(str(x) for x in b_now.buckets)}); both {n} "
            f"and {n + step} land in bucket {b_now.bucket}, so fab_floor_min "
            f"returns the SAME dict "
            f"({now.get('track_width')} / {now.get('clearance')} mm) for both "
            f"and this option cannot see a difference between them. That is a "
            f"limit of the table, NOT a measurement that more layers buy "
            f"nothing: JLC publishes one multilayer capability column and this "
            f"repo does not invent the rest. What more layers would actually "
            f"buy is nets escaping on the new layers, and THAT is now "
            f"bounded: {f_now['lanes']} lane(s) are short even if all {n} "
            f"copper layers carried signal, against {f_more['lanes']} at "
            f"{n + step}. Those are LOWER bounds -- a drop to 0 does not mean "
            f"the board routes, and a board whose inner layers are poured has "
            f"fewer signal layers than this assumes.")
    elif same_floor:
        # Reachable only if a future table gains a bucket whose floors happen
        # to match its neighbour's. Then "identical floors" IS the measurement.
        out['action'] = (
            f"the fab floor is identical at {n} and {n + step} copper layers "
            f"({now.get('track_width')} / {now.get('clearance')} mm), so more "
            f"layers buy NO extra lanes on a face. They would only help by "
            f"letting nets escape on the new layers instead, which needs a "
            f"routing attempt to measure.")
    elif d_board['lanes'] == 0:
        # Judged at the BOARD's clearance, not the fab floor. At the floor
        # this read 0 on boards with 41 deficit lanes and printed the
        # false-clean below.
        out['action'] = ("no face is short of lanes at this board's "
                         "clearance, so more layers would not fix an escape "
                         "problem")
    elif gain > 0:
        out['action'] = (
            f"going from {n} to {n + step} copper layers takes the fab floor "
            f"finer and removes {gain} of {d_now['lanes']} deficit lane(s) at the fab floor (this board is short {d_board['lanes']} at its own clearance) on its "
            f"own, before counting nets that would escape on the new layers. "
            f"Stackup is a fab-facing decision: this reports it and changes "
            f"nothing.")
    else:
        out['action'] = (
            f"{d_board['lanes']} deficit lane(s) at this board's clearance, and "
            f"{d_more['lanes']} still remain at the fab floor for {n + step} "
            f"layers -- the finer floor does not supply them, so this is a "
            f"geometry problem (move what ate the span) rather than a stackup "
            f"one.")
    return out


def move_blocker(pcb_data, pcb_file: str, *, clearance: float,
                 track_width: Optional[float] = None,
                 limit: int = 5) -> Dict:
    """Which neighbour to move, and by how many millimetres.

    `escape.FaceLedger` already carries `deficit` and `blockers` -- its own
    dataclass calls blockers "WHO ate it -- the actionable field". What it has
    never carried is the CONVERSION: a face short by k lanes needs
    k * lane_pitch mm of span freed, and that one multiplication is the
    difference between "this face is short 3 lanes" and "move C14 1.4 mm".
    """
    from placement.escape import escape_ledger
    try:
        led = escape_ledger(pcb_data, pcb_file=pcb_file, clearance=clearance,
                            track_width=track_width)
    except Exception as e:                       # noqa: BLE001
        return _skip(f'the escape ledger did not run: {type(e).__name__}: {e}')
    if not led:
        return _skip('this board has no fine-pitch part, so no face has a '
                     'lane budget to be short of')

    rows = []
    for part in led:
        for face in (getattr(part, 'faces', ()) or ()):
            d = getattr(face, 'deficit', 0)
            if d <= 0:
                continue
            pitch = getattr(face, 'lane_pitch_mm', None) or 0.0
            rows.append({
                'ref': getattr(part, 'ref', None),
                'face': getattr(face, 'face', None),
                'deficit_lanes': d,
                'lane_pitch_mm': round(pitch, 4),
                'span_needed_mm': round(d * pitch, 3),
                'blocked_mm': round(getattr(face, 'blocked_mm', 0.0) or 0.0,
                                    3),
                'blockers': list(getattr(face, 'blockers', ()) or ()),
            })
    rows.sort(key=lambda r: (-r['span_needed_mm'], str(r['ref'])))
    if not rows:
        return {'ran': True, 'measured': {'faces_in_deficit': 0},
                'expected': {'faces_in_deficit': 0},
                'action': 'no face is short of lanes; nothing to move'}
    worst = rows[0]
    who = worst['blockers'][0] if worst['blockers'] else None
    out = {'ran': True,
           'measured': {'faces_in_deficit': len(rows), 'worst': rows[:limit]},
           'expected': {'faces_in_deficit': 0}}
    out['action'] = (
        f"{worst['ref']}'s {worst['face']} face is short {worst['deficit_lanes']} "
        f"lane(s); freeing {worst['span_needed_mm']:.2f} mm of span there "
        + (f"means moving {who}, which ate {worst['blocked_mm']:.2f} mm of it."
           if who else
           "would clear it, though nothing was identified as eating the span.")
        + " Net ordering cannot fix a face deficit -- it only chooses which "
          "nets strand there.")
    return out


def relax_clearance(pcb_data, pcb_file: str, *, clearance: float,
                    track_width: Optional[float] = None,
                    factors: Sequence[float] = (0.75, 0.5)) -> Dict:
    """What a tighter clearance would buy, and what the fab allows.

    Reported as a LADDER rather than a single answer, and floored at the fab
    minimum for this board's layer count, because a lane pitch below what any
    fab can etch predicts capacity nobody can build -- `check_channels.py:
    152-174` measured that trap directly (declared 0.02/0.02 reported supply
    242 against 29 at 0.2/0.2).
    """
    from placement.escape import escape_ledger
    floor_clr = None
    try:
        from fab_tiers import (count_copper_layers_in_data,
                               count_copper_layers_in_file, fab_floor_min)
        # Same file-then-memory order as add_layers: an unsaved board has a
        # perfectly good layer list and used to get no fab floor at all here,
        # so its ladder reported every rung as measurable.
        n = (count_copper_layers_in_file(pcb_file)
             or count_copper_layers_in_data(pcb_data))
        if n:
            floor_clr = fab_floor_min(n).get('clearance')
    except Exception:                            # noqa: BLE001
        floor_clr = None

    def _deficit(clr):
        led = escape_ledger(pcb_data, pcb_file=pcb_file, clearance=clr,
                            track_width=track_width)
        # Via deficit_totals, NOT `getattr(p, 'worst_deficit', 0)`. That name
        # is a to_dict() key, never an attribute, so the getattr returned 0 on
        # every board and this whole option reported "no face is short of
        # lanes" while move_blocker, in the same report, named 23 faces in
        # deficit on tigard. Same lane definition as move_blocker (all faces,
        # not the worst per part) so the two cannot disagree again.
        return deficit_totals(led)['lanes']

    try:
        base = _deficit(clearance)
    except Exception as e:                       # noqa: BLE001
        return _skip(f'the escape ledger did not run: {type(e).__name__}: {e}')
    if base == 0:
        return _skip('no face is short of lanes at the current clearance, so '
                     'relaxing it buys nothing measurable here')

    ladder = []
    for f in factors:
        clr = round(clearance * f, 4)
        below = floor_clr is not None and clr < floor_clr
        row = {'clearance': clr, 'below_fab_floor': bool(below),
               'fab_floor': floor_clr}
        if below:
            row['deficit_lanes'] = None
            row['note'] = ('below the fab floor for this layer count -- not '
                           'measured, because capacity nobody can etch is '
                           'not capacity')
        else:
            row['deficit_lanes'] = _deficit(clr)
        ladder.append(row)
    out = {'ran': True,
           'measured': {'clearance': clearance, 'deficit_lanes': base,
                        'ladder': ladder, 'fab_floor_clearance': floor_clr},
           'expected': {'deficit_lanes': 0}}
    usable = [r for r in ladder if r.get('deficit_lanes') is not None]
    best = min(usable, key=lambda r: r['deficit_lanes']) if usable else None
    if best and best['deficit_lanes'] < base:
        out['action'] = (
            f"tightening clearance from {clearance} to {best['clearance']} mm "
            f"takes the deficit from {base} to {best['deficit_lanes']} lane(s) "
            f"and stays at or above this board's fab floor. That is a DRC "
            f"decision the board owner makes, not a placement one.")
    else:
        out['action'] = (
            f"tightening clearance to the fab floor does not clear the "
            f"{base} deficit lane(s) -- the span is taken by geometry, so "
            f"move what ate it or add layers.")
    return out


def smaller_footprint(pcb_data, pcb_file: str, *, clearance: float,
                      limit: int = 5) -> Dict:
    """Which parts are big enough that their SIZE is the constraint.

    Ranked by courtyard area against the board's usable area, because that is
    the honest signal available without a parts database: this cannot know
    that a 0805 has a 0402 equivalent, and says so rather than pretending.
    """
    g = grow_board(pcb_data, pcb_file, clearance=clearance,
                   board_edge_clearance=0.0)
    if not g.get('ran'):
        return _skip(g.get('reason', 'the area measurement did not run'))
    biggest = g['measured'].get('largest_parts_mm2') or []
    usable = g['measured'].get('usable_area_mm2') or 0.0
    if not biggest or usable <= 0:
        return _skip('no part geometry to rank')
    rows = [{'ref': ref, 'area_mm2': a,
             'share_of_usable': round(a / usable, 4)}
            for a, ref in biggest[:limit]]
    out = {'ran': True,
           'measured': {'largest': rows, 'usable_area_mm2': usable},
           'expected': {'share_of_usable': 'no single part dominating'},
           'not_modelled': 'whether a smaller equivalent part EXISTS -- that '
                           'needs a parts database this toolchain does not '
                           'have'}
    top = rows[0]
    out['action'] = (
        f"{top['ref']} alone occupies {top['share_of_usable']:.1%} of the "
        f"usable area ({top['area_mm2']} mm2). If the board is short of room, "
        f"a smaller package for it buys the most; whether one exists is a "
        f"parts decision this tool cannot make.")
    return out


OPTIONS = {
    'grow_board': grow_board,
    'add_layers': add_layers,
    'move_blocker': move_blocker,
    'relax_clearance': relax_clearance,
    'smaller_footprint': smaller_footprint,
}


def capacity_options(pcb_data, pcb_file: str, *, clearance: float,
                     board_edge_clearance: float,
                     track_width: Optional[float] = None,
                     only: Optional[Sequence[str]] = None) -> Dict:
    """Every option, each either measured or explaining why it was not."""
    out: Dict[str, Dict] = {}
    for name, fn in OPTIONS.items():
        if only and name not in only:
            continue
        kw = {'clearance': clearance}
        if name == 'grow_board':
            kw['board_edge_clearance'] = board_edge_clearance
        if name in ('add_layers', 'move_blocker', 'relax_clearance'):
            kw['track_width'] = track_width
        try:
            out[name] = fn(pcb_data, pcb_file, **kw)
        except Exception as e:                   # noqa: BLE001 - disclosed
            # A crash is NOT "this option does not apply here", and the two
            # must not read alike: a leftover tuple index in add_layers once
            # raised KeyError and arrived looking exactly like the honest
            # "this board has no fine-pitch part" skip, on a board with 43
            # deficit lanes. `error` is what tells them apart.
            out[name] = dict(_skip(f'INTERNAL ERROR: {type(e).__name__}: {e}'),
                             error=True)
    return out


def format_text(opts: Dict) -> str:
    L = ["CAPACITY OPTIONS -- measured, never applied. Nothing here writes an "
         "outline or a stackup."]
    for name in OPTIONS:
        o = opts.get(name)
        if o is None:
            continue
        if not o.get('ran'):
            tag = 'FAILED' if o.get('error') else 'not measured'
            L.append(f"  {name}: {tag} -- {o.get('reason')}")
            continue
        # Always print what was MEASURED, not only what to do about it. An
        # option whose numbers are comfortable still has numbers, and a line
        # carrying only the option's name reads as "nothing found" when it
        # actually means "measured, and fine".
        L.append(f"  {name}: {_digest(o.get('measured') or {})}")
        if o.get('action'):
            for chunk in _wrap(o['action'], 74):
                L.append(f"    {chunk}")
        if o.get('not_modelled'):
            L.append(f"    NOT MODELLED: {o['not_modelled']}")
    ran = sum(1 for o in opts.values() if o.get('ran'))
    L.append(f"  {ran} of {len(opts)} option(s) measured; "
             f"{len(opts) - ran} could not be.")
    return "\n".join(L)


# Keys the text channel must never drop, whatever else is added beside them.
# `_digest`'s positional `limit` silently truncated: adding the fab-floor pair
# pushed `deficit_lanes_at_more` -- the "would more layers help" number the
# whole option exists to answer -- off the end, and `parts=92` off grow_board.
# A digest that drops the headline is not a digest.
#
# `fab_floor_layer_blind` is here and `fab_buckets_modelled` deliberately is
# NOT: the second is a non-empty list, so forcing it would spend a slot
# rendering the useless `fab_buckets_modelled[2]` and evict a real number --
# the same eviction the `containers_excluded` comment in `_digest` records.
_DIGEST_ALWAYS = ('deficit_lanes_now', 'deficit_lanes_at_more',
                  'deficit_lanes_at_fab_floor', 'utilisation',
                  'busiest_side_area_mm2', 'usable_area_mm2',
                  'parts', 'shortfall_mm2_at_least',
                  'proposed_square_side_mm', 'faces_in_deficit',
                  'deficit_lanes', 'containers_excluded',
                  # #700. `copper_layers` joins the forced set rather than
                  # riding in on the unforced remainder: `_digest` returns
                  # `out[:max(limit, len(forced))]`, so once add_layers' forced
                  # count passed the limit of 5 NO unforced key could appear at
                  # all, and the layer count -- the subject of the whole
                  # option -- silently left the text channel. The forced list
                  # IS add_layers' digest now, so it has to be the curated
                  # headline rather than a supplement to one.
                  'fab_floor_layer_blind', 'copper_layers',
                  'deficit_floor_lanes_now', 'deficit_floor_lanes_at_more')


def _digest(measured: Dict, limit: int = 5) -> str:
    """The scalar measurements, compactly. Nested rows are named, not dumped:
    a text digest that inlined a findings list is how a summary stops being
    one (the same trap board_brief's locks tally fell into).

    `_DIGEST_ALWAYS` keys are emitted whether or not they fall inside `limit`,
    and a non-empty dict is named with its size rather than skipped entirely --
    `part_area_by_side_mm2` was added so a reader could check `utilisation`
    against a number, and being a dict it never appeared in the text channel
    it was added for.
    """
    bits, forced = [], []
    for k, v in measured.items():
        empty = isinstance(v, (list, tuple, dict)) and not v
        if isinstance(v, bool):
            bit = f"{k}={'yes' if v else 'no'}"
        elif isinstance(v, (int, float)):
            bit = f"{k}={v}"
        elif isinstance(v, (list, tuple)):
            bit = f"{k}[{len(v)}]"
        elif isinstance(v, dict) and v:
            bit = f"{k}{{{len(v)}}}"
        else:
            continue
        # An EMPTY collection is never forced. Forcing `containers_excluded`
        # unconditionally spent a slot on `[0]` and evicted
        # usable_area_mm2/outline_area_mm2 -- the denominator `utilisation` is
        # computed from -- on the 32 of 33 corpus boards that have no
        # container. A digest that drops the denominator to report an empty
        # list is worse than the truncation it was fixing.
        (forced if (k in _DIGEST_ALWAYS and not empty) else bits).append(bit)
    out = forced + [b for b in bits if b not in forced]
    return ', '.join(out[:max(limit, len(forced))]) if out else 'measured'


def _wrap(text: str, width: int) -> List[str]:
    out, line = [], ''
    for word in text.split():
        if len(line) + len(word) + 1 > width:
            out.append(line)
            line = word
        else:
            line = f"{line} {word}".strip()
    if line:
        out.append(line)
    return out
