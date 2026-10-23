"""Apply a model-chosen pose, and let the engine grade it.

`pose_score.rank_poses` lists legal (x, y, rot) candidates nearest first, and
`grade_pad_legality` / `check_assembly` / `render_placement --review-sheet`
grade a pose; this module APPLIES one, so a decision such as "U1 belongs
here, facing that way" reaches the board through a registered tool
(`place_pose.py`) rather than a hand script around
`placement.writer.write_placed_output`.

This module is the ENGINE half, so both fronts get the same behaviour: the CLI
parses argv into `ops` and calls `apply_poses`; anything in-process (the GUI's
own apply path, a future control) calls the same function rather than
re-assembling writer + sibling copy + grade for itself. That ordering is
CLAUDE.md's CLI/GUI rule applied before a divergence rather than after one.

Two rules worth stating because getting either wrong is silent:

* LEGALITY IS RELATIVE. `QuenchState.candidate_valid` is an absolute gate and
  on real boards it is already False for a large fraction of parts before
  anything moves, so "refuse any pose candidate_valid dislikes" would refuse
  poses no worse than where the part already sits, and refuse to touch a board
  mid-repair at all. The verdict here is `grade_pad_legality` on the candidate
  board against the same grade on the INPUT board: a request is refused when it
  makes a category WORSE, never for damage it inherited.
* NO RULE IS RE-DERIVED. The pad geometry is `grade_pad_legality`'s, the pad
  stacks are check_assembly's own `legality.pad_intersection_pairs` (#1064),
  the face
  rule is `escape.assign_faces` (#850), the ranking is `pose_score.rank_poses`,
  the writer is `placement.writer.write_placed_output`, the siblings are
  `portfolio.copy_siblings` (#441). This module is plumbing and arithmetic on
  their answers. `part_faces` is the one place that repeats a CALLING PATTERN
  rather than a rule -- see its docstring for why it does not simply call
  `board_context.pads_by_face`.
"""
from __future__ import annotations

import contextlib
import io
import os
import math
import shutil
import stat
import tempfile
from typing import Dict, List, Optional, Sequence, Tuple

#: The cardinal faces, in the order a POSITIVE rotation walks them.
#:
#: Measured from the parser's own transform rather than reasoned about
#: (`kicad_parser.py:706`): a pad's global offset is `R(-rot) . local`, so
#: raising a footprint's rotation by 90 degrees sends a pad at local (+1, 0)
#: to global offset (0, -1) -- east to north, since `escape.face_of` calls the
#: smaller-y side north on a y-grows-down board.
#: A cycle written the wrong way round would be a plausible-looking rotation
#: that faces the part backwards.
FACE_CYCLE = ('east', 'north', 'west', 'south')

#: What a caller may spell a face as. The engine's own vocabulary
#: (`escape.FACES`) is the canonical form; the single letters exist because
#: the issue's worked example uses `W` and a compass letter is what a reader
#: writes. Nothing else is guessed.
FACE_ALIASES = {'n': 'north', 'north': 'north',
                's': 'south', 'south': 'south',
                'e': 'east', 'east': 'east',
                'w': 'west', 'west': 'west'}

#: The legality categories a request may not WORSEN. Board-level counts from
#: `grade_pad_legality`. #962 added `oob_graphic_copper_count`, the parts whose
#: footprint GRAPHIC copper (a drawn SOT-89 tab, an antenna) reaches past the
#: outline. `place_pose set U2 115.34 93.6 --rot 90` put esp_prog's tab 1.11 mm
#: off the board with every arm here reading 0.
LEGALITY_KEYS = ('pad_conflicts', 'hole_conflicts', 'oob_pad_count',
                 'pad_edge_conflicts', 'pad_edge_unmeasured',
                 'oob_keepout_copper_count',
                 'oob_graphic_copper_count',
                 'mating_keepout_count',
                 # #1266: pads against ANOTHER part's graphic copper -- short
                 # of clearance on a net it is not, and touching on any net.
                 'graphic_pad_conflicts', 'graphic_pad_stack_count',
                 'pad_stack_count',
                 # #1306: one footprint's own pads touching on two nets.
                 'own_pad_short_count')

#: The MAGNITUDES, and they are not a nicety: a count arm alone accepts a
#: request that keeps the tally and deepens the damage. Measured on the
#: count-plus-shortfall version, on flat_hierarchy: a part already 2.0 mm off
#: the board was moved to 204.66 mm off it, `oob_pad_count 1 -> 1`, exit 0,
#: `legal: true`, and no field in the summary said so. (esp_prog reproduces
#: the same mechanism at 2.008 -> 101.008 mm.) CLAUDE.md calls copper outside
#: the
#: outline the top-priority placement defect, so its AMOUNT is an arm too.
#: The graphic-copper overrun is one for the same reason (#962).
#: #1100: conflicting PAIRS, compared as sets -- a new pair is refused even
#: when the counts tie.
#: #1064: PAD STACKS -- two parts' pad copper overlapping, ANY net, which is
#: check_assembly's never-waivable pad_intersection. `pad_conflict_pairs`
#: skips same-net pads before it measures, so esp_prog's C4 on Y1 (one net)
#: was no pair at all and this verb wrote a board check_assembly calls NOT
#: BUILDABLE. The pairs are measured by check_assembly's own function.
#: #1266: a `[pad part, copper owner]` pair against footprint graphic copper.
#: #1306: a `["U1.1", "U1.2"]` pair of one footprint's pads shorted.
PAIR_KEYS = ('pad_conflict_pairs', 'hole_conflict_pairs',
             'graphic_pad_conflict_pairs', 'graphic_pad_stack_pairs',
             'pad_stack_pairs', 'own_pad_short_pairs')

MAGNITUDE_KEYS = ('pad_shortfall', 'oob_pad_amount', 'pad_edge_shortfall',
                  'oob_keepout_copper_amount',
                  'oob_graphic_copper_amount',
                  'mating_keepout_amount',
                  'graphic_pad_shortfall',
                  'pad_stack_area')

#: What `legal` does and does NOT cover, published with every summary (#962
#: follow-up item 4). A partial claim must read as partial.
LEGAL_SCOPE = ('pad-pad clearance', 'hole-hole clearance', 'pad copper vs the '
               'outline', 'pad copper vs the edge-clearance floor',
               'footprint graphic copper vs the outline',
               'pad copper vs a board rule-area keep-out band (#1031)',
               "part bodies vs a PCB-edge plug's mating region (#1098)",
               "pad copper vs ANOTHER part's footprint graphic copper (a drawn "
               "SOT-89 tab, an antenna), its filled interior included: short "
               "of clearance on a net it is not, or touching on any net "
               "(#1266)",
               "two parts' pad copper overlapping on a shared side, ANY net "
               "-- a pad stack, check_assembly's pad_intersection (#1064)",
               "one footprint's own pads touching on two different real nets "
               "(KiCad's shorting_items; #1306)")
LEGAL_UNMEASURED = ('footprint graphic copper vs the edge-clearance floor '
                    '(disclosed as graphic_edge_shortfall_refs, not gated)',
                    'footprint copper the parser does not model: pad-less '
                    'logos, bezier curves, copper text (named per part in '
                    'oob_graphic_copper_unmeasured)',
                    'footprint graphic copper on a board with no outline, and '
                    'on a part that changed side in memory (listed as '
                    'no-outline / moved-side in oob_graphic_copper_unmeasured)',
                    'rule areas a footprint owns (listed in '
                    'keepout_copper_unmeasured)',
                    'solder paste and mask openings', 'component bodies / '
                    'courtyards', 'routing', 'zone fill',
                    "parts sharing an origin whose pads do not intersect "
                    "(check_assembly's coincident_origins)")
MAGNITUDE_EPS = 1e-6

#: How many entries of each per-pad / per-pair list the stdout JSON_SUMMARY
#: carries (#1352) -- board_score's cap (#1194). On a 97-part pile
#: (pico_ice_rev3) the summary line was 291 KB, 245 KB of it the pad-edge
#: findings and unmeasured rows before and after, and a refusal's one-line
#: reason was buried in it. `place_pose --json PATH` writes every entry.
STDOUT_LIST_CAP = 20

#: The summary's per-pad and per-pair lists (`_legality_row`), capped on
#: stdout. The request's own lists (`ops`, `moved`, `locked`) are not.
SUMMARY_LISTS = ('pad_clearance_required', 'worst',
                 'oob_pad_copper_refs_after',
                 'oob_graphic_copper_refs_after',
                 'oob_graphic_copper_waived_after',
                 'oob_graphic_copper_unmeasured_after',
                 'graphic_edge_shortfall_refs_after',
                 'keepout_copper_pads_after', 'keepout_copper_tht_refs_after',
                 'keepout_copper_unmeasured_after', 'pad_stack_pairs_after',
                 'graphic_pad_conflict_pairs_after',
                 'graphic_pad_stack_pairs_after', 'graphic_pad_rows_after',
                 'graphic_pad_unmeasured_after', 'own_pad_short_pairs_after')

#: ...and the lists inside each pad-edge grade (`pad_edge_before/_after`).
PAD_EDGE_LISTS = ('findings', 'unmeasured', 'rules_unmeasured')


def _cap_lists(doc: Dict, keys, cap: int) -> Dict:
    """`doc` with each list in `keys` longer than `cap` cut to `cap`, and
    `<key>_count` / `<key>_truncated` beside it; `doc` itself is untouched."""
    long_ = [k for k in keys if isinstance(doc.get(k), list) and len(doc[k]) > cap]
    if not long_:
        return doc
    out = dict(doc)
    for k in long_:
        out[k] = doc[k][:cap]
        out[k + '_count'] = len(doc[k])
        out[k + '_truncated'] = len(doc[k]) - cap
    return out


def stdout_summary(summary: Dict, cap: int = STDOUT_LIST_CAP) -> Dict:
    """The JSON_SUMMARY copy of a place_pose summary (#1352): every list in
    SUMMARY_LISTS, and each pad-edge grade's PAD_EDGE_LISTS, capped at `cap`
    entries, `<list>_count` the full length and `<list>_truncated` how many
    were left out. The counts and magnitudes the verdict reads are scalars
    and are never cut. `summary` itself -- what `--json` writes -- is not
    touched. Pure."""
    out = _cap_lists(summary, SUMMARY_LISTS, cap)
    for ch in ('pad_edge_before', 'pad_edge_after'):
        g = out.get(ch)
        if isinstance(g, dict):
            capped = _cap_lists(g, PAD_EDGE_LISTS, cap)
            if capped is not g:
                if out is summary:
                    out = dict(summary)
                out[ch] = capped
    return out

#: `replicate --search`'s default lattice pitch, mm (#1211). Coarser than
#: the snap's 0.25: every candidate is a written, re-parsed, re-graded board
#: (about 0.27 s on the 114-part multichannel_mixer), and a 3 mm radius at
#: 0.5 is 113 of them per target.
SEARCH_STEP = 0.5


class PoseRefusal(Exception):
    """A request refused for a stated reason, with the numbers behind it.

    `code` is the exit code the CLI should use, and the two values mean
    different things to a caller: 2 is "the request does not name a thing on
    this board" (an unknown ref, a face with no row) -- a typo the caller
    fixes by rewriting the command -- and
    4 is "the request is well-formed and the board says no" (it grades worse,
    or the part is locked), which is a MEASUREMENT the caller acts on. Folding
    both into one code would make a typo indistinguishable from a finding.
    """

    def __init__(self, reason: str, code: int = 4, **extra):
        super().__init__(reason)
        self.reason = reason
        self.code = code
        self.extra = extra


# ---------------------------------------------------------------------------
# knobs
# ---------------------------------------------------------------------------

def resolve_knobs(board_path: str, clearance=None, board_edge_clearance=None,
                  track_width=None):
    """(clearance, board_edge_clearance, track_width, knobs) from the BOARD.

    Same resolution `place_simple` and `board_context` use
    (`list_nets.board_floor_knobs`): explicit value > the board's own Default
    netclass / constraint > the fixed default. A fixed 0.25/0.55 on a board
    routed to 0.15/0.3 vetoes poses the router would happily route, which is
    the run-7 S4 finding this exists to avoid.

    The clearance is read as KiCad GRADES it (#1338): the class floored at
    Board Setup `min_clearance`. This verb's verdict is a grade, and on
    multichannel_mixer (class 0.2, min_clearance 0.3) the class alone called
    `legal` a pose KiCad reports two pad clearance errors on.
    """
    import list_nets
    import routing_defaults as defaults
    clr, edge, knobs = list_nets.board_floor_knobs(
        board_path, clearance, board_edge_clearance)
    if not math.isfinite(edge) or edge < 0:
        raise PoseRefusal('board-edge-clearance must be finite and nonnegative (mm); '
                          'got %r' % edge, code=2)
    tw = track_width
    if tw is None:
        tw = (list_nets.board_default_netclass_param(board_path, 'track_width')
              or defaults.TRACK_WIDTH)
    return clr, edge, float(tw), knobs


# ---------------------------------------------------------------------------
# faces
# ---------------------------------------------------------------------------

def normalize_face(token: str) -> str:
    """A face token in the engine's own spelling, or a refusal naming both."""
    key = str(token or '').strip().lower()
    if key not in FACE_ALIASES:
        raise PoseRefusal(
            "%r is not a face: use north/south/east/west (or N/S/E/W)"
            % (token,), code=2)
    return FACE_ALIASES[key]


def rotate_face(face: str, delta_deg: float) -> str:
    """Where `face` points after the part is turned by `delta_deg`.

    Only multiples of 90 are meaningful here; a non-orthogonal delta is
    refused rather than rounded, because rounding it would silently answer a
    different question than the caller asked. That refusal is an INTERNAL
    guard, not a CLI path: the only production caller passes `face_delta`'s
    output, which is a multiple of 90 by construction. `rotate R4 45` reaches
    the writer and is graded like any other pose.
    """
    if abs(delta_deg / 90.0 - round(delta_deg / 90.0)) > 1e-6:
        raise PoseRefusal(
            "a face can only be carried by an orthogonal rotation; %g degrees "
            "is not a multiple of 90" % (delta_deg,), code=2)
    steps = int(round(delta_deg / 90.0)) % 4
    return FACE_CYCLE[(FACE_CYCLE.index(face) + steps) % 4]


def face_delta(from_face: str, to_face: str) -> float:
    """The smallest non-negative multiple of 90 carrying one face to another."""
    steps = (FACE_CYCLE.index(to_face) - FACE_CYCLE.index(from_face)) % 4
    return float(steps * 90)


def part_faces(pcb_data, ref: str, *, clearance: float, track_width: float):
    """`{face: [pad, ...]}` for one part at its CURRENT pose.

    Calls `escape.assign_faces` -- THE face rule (#850) -- with the same
    pairing `board_context.pads_by_face` uses. The RULE is not repeated; the
    six lines that feed it are, and deliberately:

    * `pads_by_face` is whole-board and returns `{ref: {face: [pad]}}`, so
      using it here would build the copper geometry for every part to answer
      about one;
    * it lives in `py_tools/board_context.py`, which imports
      `render_placement`; an engine module under `py_placer/placement/`
      importing that inverts the layering the rest of this package keeps;
    * their ERROR behaviour differs on purpose. `pads_by_face` skips a
      pad-less block and swallows an `assign_faces` exception, because it is
      building a sheet about every part. This verb was ASKED about one part,
      so both of those are refusals with a reason.

    If the pairing ever has to change, it changes in `escape.assign_faces`,
    which is the thing #850 made single.
    """
    from placement.escape import assign_faces, board_copper_geometry
    from placement.escape import _part_rect        # noqa: PLC2701
    fp = pcb_data.footprints.get(ref)
    if fp is None:
        raise PoseRefusal("%s is not a footprint on this board" % (ref,), code=2)
    from kicad_parser import non_aperture_pads
    if not non_aperture_pads(fp):       # apertures are not pads (#1143)
        raise PoseRefusal("%s has no pads, so it has no face to aim" % (ref,),
                          code=2)
    try:
        obstruction = board_copper_geometry(pcb_data, clearance)
    except Exception:                                        # noqa: BLE001
        obstruction = {}
    own = obstruction.get(ref)
    rect = own.rect if own is not None else _part_rect(fp)
    asg = assign_faces(fp, own, lane_mm=track_width + clearance,
                       fallback_rect=rect, clearance=clearance,
                       track_width=track_width)
    by: Dict[str, List] = {}
    for pad, face in asg.faces:
        by.setdefault(face or 'interior', []).append(pad)
    return by


def part_centre(pcb_data, ref: str):
    """The part's pad-copper centre, or its origin when it has no pads.

    Pad centre rather than `(at x y)`: a connector's origin can sit well off
    its body, and "which way is USB1 from here" is a question about copper.
    """
    fp = pcb_data.footprints.get(ref)
    if fp is None:
        raise PoseRefusal("%s is not a footprint on this board" % (ref,),
                          code=2)
    from kicad_parser import non_aperture_pads
    pads = non_aperture_pads(fp)        # apertures are not copper (#1143)
    if not pads:
        return float(fp.x), float(fp.y)
    xs = [p.global_x for p in pads]
    ys = [p.global_y for p in pads]
    return (min(xs) + max(xs)) / 2.0, (min(ys) + max(ys)) / 2.0


def bearing_face(from_xy, to_xy) -> str:
    """The cardinal direction of `to_xy` seen from `from_xy`.

    A tie -- a partner exactly on the diagonal -- resolves to the X axis,
    deterministically. Stated as it IS rather than as a rule about the
    dominant axis (there is no dominant axis in a tie) or about `FACE_CYCLE`
    order (which would answer `north` for the up-left diagonal, where this
    answers `west`). Deterministic is the property that matters; which way it
    breaks is arbitrary, and a caller aiming across a diagonal should name the
    face it wants instead.
    """
    dx = to_xy[0] - from_xy[0]
    dy = to_xy[1] - from_xy[1]
    if abs(dx) >= abs(dy):
        return 'east' if dx >= 0 else 'west'
    return 'south' if dy >= 0 else 'north'      # y grows down (#850's north)


# ---------------------------------------------------------------------------
# op resolution -- every op is resolved against the INPUT board
# ---------------------------------------------------------------------------

def _replicate_note(pcb_data, row: Dict) -> Tuple[Dict, Dict]:
    """`(note, placement)` for one replicate row: the summary's per-part note
    and the writer's placement (used only when `row['moved']`)."""
    fp = pcb_data.footprints[row['ref']]
    rot0 = float(fp.rotation or 0.0) % 360.0
    note = {'ref': row['ref'], 'kind': 'replicate',
            'source': row['source'],
            'from': [round(float(fp.x), 4), round(float(fp.y), 4), rot0],
            'to': [round(row['x'], 4), round(row['y'], 4), row['rot']],
            'moved': bool(row['moved'])}
    place = {'reference': row['ref'], 'new_x': row['x'],
             'new_y': row['y'], 'new_rotation': row['rot']}
    if row['flip']:
        # Only on a side CHANGE: a `new_side` key the writer does not need
        # would still be a different code path (#714).
        from placement.legality import footprint_side
        note['side'] = [footprint_side(fp), row['side']]
        place['new_side'] = row['side']
    return note, place


def resolve_ops(pcb_data, ops: Sequence[Dict], *, clearance: float,
                track_width: float, reports: Optional[List[Dict]] = None,
                replicate_rows: Optional[List] = None):
    """Turn caller ops into writer placements + a per-op note.

    Every op reads the INPUT board, so a call carrying several of them
    describes ONE arrangement rather than a sequence whose later members see
    the earlier ones' effects. That is what makes "a whole arrangement is one
    board state" true rather than merely intended.

    A `replicate` op (#1211) expands into one note per TARGET part (kind
    `replicate`, with its `source`) and a placement for each part that
    actually moves -- so a KiCad-locked part already at its copied pose is
    not refused for a move nobody made. Sheets are named by
    `Footprint.sheet_name`, which both parse paths fill; `reports` collects
    each op's mapping for the summary, and `replicate_rows` each op's
    `(op, report, rows)` at full precision, which `--search` shifts.
    """
    placements: List[Dict] = []
    notes: List[Dict] = []
    seen = {}
    sources = {}
    for op in ops:
        kind = op['kind']
        if kind == 'replicate':
            from placement import replicate
            rows, report = replicate.plan(
                pcb_data, op['from'], op['to'],
                anchor=op.get('anchor'), offset=op.get('offset'),
                allow_net_mismatch=op.get('allow_net_mismatch') or (),
                only=op.get('only') or (), skip=op.get('skip') or ())
            for r in report['source_refs']:
                sources.setdefault(r, report['from']['as_given'])
            if reports is not None:
                reports.append(report)
            if replicate_rows is not None:
                replicate_rows.append((op, report, rows))
            for row in rows:
                ref = row['ref']
                if ref in seen:
                    raise PoseRefusal(
                        "%s is named by two ops in one call (%s then "
                        "replicate); one call describes one arrangement, so "
                        "a part gets one pose" % (ref, seen[ref]), code=2)
                seen[ref] = 'replicate'
                note, place = _replicate_note(pcb_data, row)
                notes.append(note)
                if row['moved']:
                    placements.append(place)
            continue
        ref = op['ref']
        fp = pcb_data.footprints.get(ref)
        if fp is None:
            raise PoseRefusal(
                "%s is not a footprint on this board (the parser names "
                "duplicate references TP4 / TP4~2, #726)" % (ref,), code=2)
        if ref in seen:
            raise PoseRefusal(
                "%s is named by two ops in one call (%s then %s); one call "
                "describes one arrangement, so a part gets one pose"
                % (ref, seen[ref], kind), code=2)
        seen[ref] = kind
        # The baseline rotation is normalised to [0, 360) because the FILE's
        # spelling is not canonical: KiCad writes -90 where this tool writes
        # 270, and comparing the two raw would report a part as MOVED by a
        # rotation it already has.
        x, y = float(fp.x), float(fp.y)
        rot = float(fp.rotation or 0.0) % 360.0
        note = {'ref': ref, 'kind': kind,
                'from': [round(x, 4), round(y, 4), rot]}

        if kind == 'set':
            if op.get('x') is not None:
                x = float(op['x'])
            if op.get('y') is not None:
                y = float(op['y'])
            if op.get('rot') is not None:
                rot = float(op['rot']) % 360.0
        elif kind == 'rotate':
            deg = float(op['rot'])
            rot = ((rot + deg) if op.get('relative') else deg) % 360.0
        elif kind == 'face':
            face = normalize_face(op['face'])
            by = part_faces(pcb_data, ref, clearance=clearance,
                            track_width=track_width)
            if not by.get(face):
                raise PoseRefusal(
                    # code=2: the docstring and PoseRefusal both file this as
                    # "the request does not name a thing on this board", and
                    # it was exiting 4 -- the code that means the board said
                    # no about a well-formed request.
                    "%s has no pads on its %s face right now, so there is no "
                    "row to aim; it has %s" % (
                        ref, face,
                        ', '.join('%s: %d' % (f, len(p))
                                  for f, p in sorted(by.items())) or 'none'),
                    code=2, faces={f: len(p) for f, p in by.items()})
            if op['partner'] == ref:
                raise PoseRefusal(
                    "%s cannot face itself; name the part its %s row should "
                    "point at" % (ref, face), code=2)
            here = part_centre(pcb_data, ref)
            there = part_centre(pcb_data, op['partner'])
            if abs(here[0] - there[0]) < 1e-9 and abs(here[1] - there[1]) < 1e-9:
                # Two parts on one coordinate is a PILE, not a direction. The
                # bearing would fall out of a tie-break and read as an answer.
                raise PoseRefusal(
                    "%s and %s share a centre (%.4f, %.4f), so there is no "
                    "direction to aim -- place one of them first"
                    % (ref, op['partner'], here[0], here[1]), code=2)
            target = bearing_face(here, there)
            delta = face_delta(face, target)
            rot = (rot + delta) % 360.0
            # The row is identified by each pad's INDEX in `fp.pads`, never by
            # its number. Pad numbers are NOT unique -- a shield, a split
            # thermal paddle and every unnumbered mechanical pad share one --
            # and a number-keyed map collapses them to whichever pad the dict
            # saw last. Measured: esp_prog's USB1 carries SIX pads numbered
            # `0`, spread across its faces, so the verification below scored a
            # row that had provably rotated as 0/2 and then called it
            # SYMMETRIC. Over the 22 tracked boards in `kicad_files/`, 196
            # parts on 16 boards repeat a pad number at all (57 on 11 boards
            # if the empty number is excluded, 49 on 8 if `0` is too) -- the
            # definition matters, so it is written down rather than summarised
            # as one figure. The index is stable across the write (the
            # writer reorders nothing), and `row_pads` keeps the numbers for a
            # human to read.
            ix = {id(p): i for i, p in enumerate(fp.pads or ())}
            note.update({'face': face, 'partner': op['partner'],
                         'target_face': target, 'rotation_delta': delta,
                         'row_pad_ix': [ix[id(p)] for p in by[face]],
                         'row_pads': [p.pad_number for p in by[face]]})
        else:
            raise PoseRefusal("unknown op %r" % (kind,), code=2)

        note['to'] = [round(x, 4), round(y, 4), rot]
        note['moved'] = note['to'] != note['from']
        if not note['moved']:
            # #1322: say WHY nothing moved. `rotate R2 180` on a part already
            # at 180 reads as a turn to anyone who meant one.
            note['unmoved_why'] = (
                "already at %g degrees: `rotate` sets an ABSOLUTE angle; "
                "`rotate %s %g --relative` turns it by %g about its origin"
                % (rot, ref, float(op['rot']), float(op['rot']))
                if kind == 'rotate' and not op.get('relative')
                else 'already at this pose')
        notes.append(note)
        placements.append({'reference': ref, 'new_x': x, 'new_y': y,
                           'new_rotation': rot})
    # A replicate copies the INPUT poses of its source sheet, so a source part
    # that another op in this call moves would leave the copy describing an
    # arrangement the output does not have.
    clash = sorted(r for r in {p['reference'] for p in placements}
                   if r in sources)
    if clash:
        raise PoseRefusal(
            "%s %s in the --from sheet of a replicate (%s) and %s moved by "
            "this same call; the copy reads the INPUT board's poses, so the "
            "two would disagree. Place the source sheet in one call, then "
            "replicate it in the next" % (
                ', '.join(clash), 'is' if len(clash) == 1 else 'are',
                sources[clash[0]], 'is' if len(clash) == 1 else 'are'),
            code=2, source_moved=clash)
    return placements, notes


# ---------------------------------------------------------------------------
# legality
# ---------------------------------------------------------------------------

def grade(pcb_data, board_path: str, clearance: float,
          board_edge_clearance=None, declared_keepouts=()) -> Dict:
    """`grade_pad_legality` at this board's own poses -- never a re-derivation.

    `pcb_file` is passed so `PadClearanceModel` can read the netclasses, the
    `.kicad_dru` layer rules and any pad `local_clearance` override (#697):
    a board that declares none of the three grades exactly as a flat scalar
    would, and one that declares them is graded the way check_drc will.
    `declared_keepouts` is the intent's keep-outs (#1098): a declared
    `mating:<ref>` replaces the derived plug region, as in the seeder.
    #1064: plus `pad_stack_census`, check_assembly's own pad_intersection
    channel, because `grade_pad_legality` skips a same-net pad pair before it
    measures anything and so never sees two parts' pads stacked on one net.
    """
    from placement.legality import grade_pad_legality, pad_stack_census
    g = grade_pad_legality(pcb_data, clearance, pcb_file=board_path,
                           edge_margin=board_edge_clearance,
                           declared_keepouts=declared_keepouts)
    g.update(pad_stack_census(pcb_data, clearance))
    return g


def worsened(before: Dict, after: Dict) -> List[str]:
    """Which legality categories the request made worse. [] is the good case."""
    out = [k for k in LEGALITY_KEYS
           if (after.get(k) or 0) > (before.get(k) or 0)]
    # #1098: an unmeasured plug mating region is not a clean one --
    # check_assembly fails closed on it, and so does this verb.
    if after.get('mating_keepout_error'):
        out.append('mating_keepout_error')
    out += [k for k in MAGNITUDE_KEYS
            if (after.get(k) or 0.0) > ((before.get(k) or 0.0) + MAGNITUDE_EPS)]
    # #962: a SWAP can hold the graphic-copper count and summed amount level
    # while moving the overrun onto a part that was clean. A part newly past
    # the outline is new damage when the totals merely TIE; a request that
    # strictly lowers the count or the amount is an improvement and is not
    # refused for where the remainder landed. A `before` that carries no refs
    # (an older report) cannot say which parts are new, so the arm is off.
    if 'oob_graphic_copper_refs' in before:
        was = {r[0] for r in (before.get('oob_graphic_copper_refs') or ())}
        new = [r[0] for r in (after.get('oob_graphic_copper_refs') or ())
               if r[0] not in was]
        improved = ((after.get('oob_graphic_copper_count') or 0)
                    < (before.get('oob_graphic_copper_count') or 0)
                    or (after.get('oob_graphic_copper_amount') or 0.0)
                    < (before.get('oob_graphic_copper_amount') or 0.0) - MAGNITUDE_EPS)
        if new and not improved:
            out.append('oob_graphic_copper_refs')
    # #1100: the same question for pad and hole conflicts, with NO
    # improvement exemption. Run 37 seated C18 so that it left a conflict it
    # had in the off-board pile and made one on the board with D11: the
    # count tied (2 -> 2) and the board was written with a short. A pair
    # that did not exist before is new damage whatever else went away; a
    # report without the pair list (an older one) leaves the arm off.
    for pair_key in PAIR_KEYS:
        if pair_key in before:
            had = {tuple(p[:2]) for p in (before.get(pair_key) or ())}
            fresh = [p for p in (after.get(pair_key) or ())
                     if tuple(p[:2]) not in had]
            if fresh:
                out.append(pair_key)
    return out


def new_pairs(before: Dict, after: Dict, pair_key: str) -> List[str]:
    """`["A/B", ...]`: the pairs in `after[pair_key]` that `before` lacks."""
    had = {tuple(p[:2]) for p in (before.get(pair_key) or ())}
    return ['%s/%s' % (p[0], p[1]) for p in (after.get(pair_key) or ())
            if tuple(p[:2]) not in had]


#: The pair list that names the parts behind a count arm, so a refused snap
#: candidate is reported by WHO it hit, not only by which tally moved (#1209).
PAIR_OF = {'pad_conflicts': 'pad_conflict_pairs',
           'hole_conflicts': 'hole_conflict_pairs',
           'graphic_pad_conflicts': 'graphic_pad_conflict_pairs',
           'graphic_pad_stack_count': 'graphic_pad_stack_pairs',
           'pad_stack_count': 'pad_stack_pairs',
           'own_pad_short_count': 'own_pad_short_pairs'}


def regrade_refusal(before: Dict, after: Dict, bad: List[str],
                    strict: bool) -> Dict[str, List[str]]:
    """`{category: ["A/B", ...]}`: why ONE re-graded snap candidate was refused.

    `bad` is `worsened(before, after)`. A pair arm whose count arm also moved
    is folded into the count arm's entry, so a hole conflict reads once, as
    `hole_conflicts: [Module301/SW501]`, not twice. With `bad` empty the
    candidate was refused by `--strict-legal` alone, and the categories are
    the ones still dirty at that pose (inherited damage included: the strict
    arm is absolute), marked as such.
    """
    out: Dict[str, List[str]] = {}
    if bad:
        for k in bad:
            if k in PAIR_KEYS and any(PAIR_OF.get(c) == k for c in bad):
                continue
            pk = PAIR_OF.get(k) or (k if k in PAIR_KEYS else None)
            out[k] = new_pairs(before, after, pk) if pk else []
        return out
    if not strict:
        return out
    for k in LEGALITY_KEYS:
        if after.get(k):
            pk = PAIR_OF.get(k)
            out[k + ' (--strict-legal)'] = (
                ['%s/%s' % (p[0], p[1]) for p in (after.get(pk) or ())]
                if pk else [])
    for k in MAGNITUDE_KEYS:
        if (after.get(k) or 0.0) > MAGNITUDE_EPS:
            out[k + ' (--strict-legal)'] = []
    # The two non-numeric arms of `is_clean`.
    for dirty, label in (
            (after.get('pad_edge', {}).get('complete') is False,
             'pad_edge incomplete'),
            (bool(after.get('mating_keepout_error')), 'mating_keepout_error')):
        if dirty:
            out[label + ' (--strict-legal)'] = []
    return out


def is_clean(report: Dict) -> bool:
    """Is this board legal in the ABSOLUTE sense, not merely no worse?"""
    return not (report.get('pad_edge', {}).get('complete') is False
                or report.get('mating_keepout_error')
                or any(report.get(k) for k in LEGALITY_KEYS)
                or any((report.get(k) or 0.0) > MAGNITUDE_EPS
                       for k in MAGNITUDE_KEYS))


def _legality_row(before: Dict, after: Dict) -> Dict:
    row = {}
    for key in LEGALITY_KEYS + MAGNITUDE_KEYS:
        row[key + '_before'] = before.get(key)
        row[key + '_after'] = after.get(key)
    row['pad_clearance_required'] = after.get('required')
    row['worst'] = after.get('worst')
    row['pad_edge_before'] = before.get('pad_edge')
    row['pad_edge_after'] = after.get('pad_edge')
    row['oob_pad_copper_count_before'] = before.get('oob_pad_copper_count')
    row['oob_pad_copper_count_after'] = after.get('oob_pad_copper_count')
    row['oob_pad_copper_refs_after'] = after.get('oob_pad_copper_refs')
    row['oob_pad_basis'] = after.get('oob_pad_basis')
    # #962: which parts' graphic copper, what is waived and why, what is not
    # measured, and the edge-floor shortfall that is disclosed but not gated.
    row['oob_graphic_copper_refs_after'] = after.get('oob_graphic_copper_refs')
    row['oob_graphic_copper_waived_after'] = after.get('oob_graphic_copper_waived')
    row['oob_graphic_copper_unmeasured_after'] = after.get('oob_graphic_copper_unmeasured')
    row['graphic_edge_shortfall_refs_after'] = after.get('graphic_edge_shortfall_refs')
    # #1031: which pads sit in a rule-area keep-out band, and which are only
    # reported (through-hole, reachable on an uncovered layer).
    row['keepout_copper_pads_after'] = after.get('keepout_copper_pads')
    row['keepout_copper_tht_refs_after'] = after.get('keepout_copper_tht_refs')
    row['keepout_copper_unmeasured_after'] = after.get('keepout_copper_unmeasured')
    # #1064: which parts' pads are stacked, and what measured them.
    row['pad_stack_pairs_after'] = after.get('pad_stack_pairs')
    row['pad_stack_basis'] = after.get('pad_stack_basis')
    # #1266: which pads sit on (or near) another part's graphic copper.
    for key in ('graphic_pad_conflict_pairs', 'graphic_pad_stack_pairs',
                'graphic_pad_rows', 'graphic_pad_unmeasured'):
        row[key + '_after'] = after.get(key)
    row['graphic_pad_basis'] = after.get('graphic_pad_basis')
    # #1306: which of one footprint's own pads are shorted together.
    row['own_pad_short_pairs_after'] = after.get('own_pad_short_pairs')
    row['own_pad_basis'] = after.get('own_pad_basis')
    return row


# ---------------------------------------------------------------------------
# snapping
# ---------------------------------------------------------------------------

def _ranker_state(pcb, board_path: str, ref: str, at, *, clearance: float,
                  board_edge_clearance: float):
    """The ranker's state for `ref`, judged from the INPUT board.

    `pcb` / `board_path` are the board the caller's re-grade compares
    against, so every part's SEED -- the baseline `LegalityContext.pads_ok`
    and `keepout_ok` refuse a pose relative to -- is its pose there. `at`
    (x, y, rot), when given, is where the caller asked the part to go: it is
    moved there AFTER the state is built, so the sweep centres on the request
    while the part is still judged against where it started. A state built
    from the staged board would take the request as its own baseline and
    admit nearby poses carrying the same new conflict, which the re-grade,
    against the input, then refuses.

    The INCUMBENT of `candidate_valid`'s off-board escape branch is pinned
    to the input pose too (`QuenchState.incumbent_at`), before the move, so
    that branch admits only poses less off the board than the INPUT, not
    than the request.
    """
    import pose_score
    st = pose_score.make_state(pcb, board_path, clearance=clearance,
                               board_edge_clearance=board_edge_clearance)
    part = st.parts.get(ref) if (at is not None
                                 and hasattr(st, 'parts')) else None
    if part is not None:
        rot = float(at[2]) % 360.0
        st.incumbent_at[ref] = (part.x, part.y, part.rot)
        # The rotation caches carry the input pose's 90-degree lattice only;
        # a rect asked for a rotation outside it falls back to 0 degrees.
        part.ensure_rotation(rot)
        st.apply_move(ref, float(at[0]), float(at[1]), rot)
    return st


def _is_pose(p: Dict, at) -> bool:
    """Is ranked pose `p` the pose `at` (x, y, rot)?"""
    if at is None:
        return False
    import pose_score
    return (abs(p['x'] - float(at[0])) <= 1e-4
            and abs(p['y'] - float(at[1])) <= 1e-4
            and pose_score._same_rot(p['rot'], at[2]))    # noqa: PLC2701


def _rank(pcb, board_path: str, ref: str, st, **kw):
    """`pose_score.rank_poses` on the ranker's state `st`: `(on_board,
    ranked, diagnostics)`.

    `ranked` and `diagnostics` are the plain ranking. When the part's input
    pose is off the board -- `candidate_valid`'s escape branch is open for it
    (`QuenchState.escape_board_term`) -- that ranking also admits poses
    merely LESS off the board than the input, and those can come first. So
    `on_board` is the same sweep with that branch CLOSED -- only poses the
    ordinary gate admits -- for a caller to try FIRST. It is None when the
    branch is closed anyway (the input is on the board, or overlaps a part),
    where `ranked` already is that ranking."""
    import pose_score
    diag: Dict = {}
    ranked = pose_score.rank_poses(pcb, board_path, ref, state=st,
                                   diagnostics=diag, **kw)
    on_board = None
    if (hasattr(st, 'escape_board_term')
            and st.escape_board_term(ref) is not None):
        st.escape_closed.add(ref)
        try:
            on_board = pose_score.rank_poses(pcb, board_path, ref, state=st,
                                             **kw)
        finally:
            st.escape_closed.discard(ref)
    return on_board, ranked, diag


def _pose_key(p: Dict) -> Tuple[float, float, float]:
    """One candidate pose as a set key, so a pose on two rungs is one."""
    return (round(p['x'], 4), round(p['y'], 4),
            round(float(p['rot']) % 360.0, 4))


def nearest_legal(board_path: str, ref: str, *, clearance: float,
                  board_edge_clearance: float, radius: float, step: float,
                  rotations=None, limit: int = 12, pcb_data=None, at=None):
    """The ranked legal poses for `ref` on the board at `board_path`.

    `pose_score.rank_poses`, unchanged: this only resolves the knobs from the
    board first and carries the dropped-pose census back, so an empty ranking
    can say "the knobs veto even staying put" instead of "no legal pose"
    (run-7 S4).

    `at` (#1256): the pose the caller asked for and its own grade refused.
    The sweep centres there, the part is judged against its pose on THIS
    board (the input, see `_ranker_state`), and `at` itself is never
    returned -- the census counts it as `requested_dropped`.

    From an input pose off the board, the poses on the board come first
    (#1277, `_rank`), in the order the snap tries them; the census counts
    them as `ranked_on_board` (None when the input is on the board).
    """
    from kicad_parser import parse_kicad_pcb
    pcb = pcb_data if pcb_data is not None else parse_kicad_pcb(board_path)
    st = _ranker_state(pcb, board_path, ref, at, clearance=clearance,
                       board_edge_clearance=board_edge_clearance)
    kw = {} if rotations is None else {'rotations': tuple(rotations)}
    on_board, ranked, diag = _rank(pcb, board_path, ref, st, radius=radius,
                                   step=step, limit=limit + 1, **kw)
    poses = ranked
    if on_board is not None:
        seen = {_pose_key(p) for p in on_board}
        poses = list(on_board) + [p for p in ranked
                                  if _pose_key(p) not in seen]
    kept = [p for p in poses if not _is_pose(p, at)]
    diag['requested_dropped'] = len(poses) - len(kept)
    diag['ranked_on_board'] = (None if on_board is None else min(limit, sum(
        1 for p in on_board if not _is_pose(p, at))))
    return kept[:limit], diag


def snap_candidates(board_path: str, ref: str, *, rot: float, clearance: float,
                    board_edge_clearance: float, radius: float, step: float,
                    pcb_data=None, at=None):
    """Poses to TRY for a snap, nearest-first, from TWO rungs. (list, census)

    `board_path` / `pcb_data` are the INPUT board -- the one the caller's
    re-grade compares against -- and `at` is the (x, y) the caller asked
    for, refused by that grade (#1256). Both rungs centre on `at` (default:
    where the part sits on the board), rung 1 judges the part against its
    pose on the board (`_ranker_state`), and the requested pose itself is
    never returned (`requested_dropped` in the census).

    Rung 1 is `rank_poses` -- the poses the seat gate admits, nearest first,
    and worth trying first because they already pass the gate.

    Rung 2 is the bare lattice around where the part now sits, and it exists
    because rung 1 alone leaves `--snap` DEAD on the boards it is aimed at.
    `rank_poses` filters through `QuenchState.candidate_valid`, an absolute
    gate, while this verb's verdict is relative: measured on flat_hierarchy,
    `set C4 --near 128.0 49.53 --radius 3` had rung 1 return ZERO candidates
    (625 dropped, including the part's own spot) while **236** poses on the
    same lattice inside the same radius graded no worse by `worsened` -- the
    nearest 0.354 mm away. Refusing there, with "no legal pose was found
    nearby", asserts something the tool's own grade contradicts.

    A ladder rather than a replacement (the repo's own rule: prefer, then fall
    back, so a rung cannot lose a repair). Rung 2 candidates are NOT claimed
    legal -- every candidate from either rung is re-graded by the caller
    before it is written.

    From an input pose OFF the board, rung 1 is preceded by `ranked_on_board`
    (#1277, `_rank`): the same ranking with the escape branch closed, so a
    pose on the board is tried before the poses merely less off the board
    than the input, which rung 1 also admits and can rank first. The
    census counts it as `ranked_on_board`, None when the input is on the
    board (rung 1 then already is that ranking).
    """
    import pose_score
    from kicad_parser import parse_kicad_pcb
    pcb = pcb_data if pcb_data is not None else parse_kicad_pcb(board_path)
    request = None if at is None else (float(at[0]), float(at[1]), rot)
    st = _ranker_state(pcb, board_path, ref, request, clearance=clearance,
                       board_edge_clearance=board_edge_clearance)
    # "The knobs veto even staying put" (run-7 S4) is a fact about the part's
    # pose on the INPUT board, where the re-grade is no-worse by definition:
    # a veto THERE means the ranker's gates are stricter than the board. The
    # ranking's own in-place spot is the request, which the caller's grade
    # has just refused, so a veto there only agrees with it (#1256).
    _home = st.parts.get(ref) if hasattr(st, 'parts') else None
    home_veto = None
    if _home is not None and hasattr(st, 'candidate_veto'):
        _v = st.candidate_veto(ref, _home.seed_x, _home.seed_y,
                               _home.orig_rot)
        if _v is not None:
            home_veto = {'check': _v[0], 'blocker': _v[1]}
    try:
        on_board, ranked, diag = _rank(pcb, board_path, ref, st,
                                       radius=radius, step=step, limit=24,
                                       rotations=(rot,))
    except pose_score.PoseUnrankable as exc:
        # #959 (#999): `place_pose` catches PoseRefusal only, so a pad-less
        # block reached a traceback here. The code carries over unchanged;
        # the advice does not -- this caller IS `place_pose set`, so it is
        # told what to do instead of the snap, not to run itself.
        raise PoseRefusal(
            f"{ref} cannot be snapped: {exc.why}. A snap searches the poses "
            f"the ranking can score, and there are none for this block -- "
            f"give it an exact pose instead (drop --near / --snap / "
            f"--strict-legal) and lock it", code=exc.code, ref=ref) from exc
    # #1256: the request was refused by the caller's grade before this ran,
    # so it is not a candidate -- the lattice below already skips offset 0.
    requested_dropped = int(any(_is_pose(p, request)
                                for p in list(on_board or ()) + ranked))
    ranked = [p for p in ranked if not _is_pose(p, request)]
    # The Euclidean bound, because `_offsets` walks SQUARE rings: a corner of
    # the r=4 ring sits 5.66 mm out, and a caller who typed `--radius 4` read
    # it as a distance (measured: a snap moved a part 5.0 mm under 4).
    out = [dict(p, rung='ranked') for p in ranked
           if (p.get('dist_mm') or 0.0) <= radius + 1e-9]
    seen = {(round(p['x'], 4), round(p['y'], 4)) for p in out}
    # #1277: from an input pose off the board, the ranking with the escape
    # branch closed comes FIRST, as its own rung -- only adding candidates
    # ahead of the ranked rung, which stays as it was, as does the lattice
    # (`seen` is the ranked rung's alone, so no lattice pose is lost to it).
    # A pose on two rungs is re-graded once (`apply_poses`).
    first = [] if on_board is None else [
        dict(p, rung='ranked_on_board') for p in on_board
        if not _is_pose(p, request)
        and (p.get('dist_mm') or 0.0) <= radius + 1e-9]

    part = st.parts.get(ref) if hasattr(st, 'parts') else None
    lattice = []
    if part is not None:
        for dx, dy in pose_score._offsets(radius, step):   # noqa: PLC2701
            dist = (dx * dx + dy * dy) ** 0.5
            if dist > radius + 1e-9 or (dx == 0.0 and dy == 0.0):
                continue
            x, y = round(part.x + dx, 4), round(part.y + dy, 4)
            if (x, y) in seen:
                continue
            seen.add((x, y))
            lattice.append({'x': x, 'y': y, 'rot': rot,
                            'dist_mm': round(dist, 4), 'rung': 'lattice'})
        lattice.sort(key=lambda p: (p['dist_mm'], p['x'], p['y']))
    # Each rung stays in ITS OWN list -- the gate-admitted poses, then the
    # bare lattice, both nearest first -- and `apply_poses` walks them in two
    # phases with a budget each. It does not merge them into one nearest-first
    # list: measured on esp_prog, the 24 lattice candidates nearest a refused
    # point ALL failed (they sit beside the thing that refused it) while a
    # ranked pose 4.78 mm away passed, so a single nearest-first cap turns a
    # snap that worked into a refusal. The caller's "stay near" is honoured in
    # the second phase, which only considers lattice poses NEARER than the
    # ranked answer.
    ordered = first + out + lattice
    # The ranker's nearest pose the radius bound dropped (#1209), so a snap
    # that refuses can say "--radius N would reach a pose" -- the honest
    # advice to a call that already snapped -- instead of offering the pose
    # back as something to "try with --snap". Never re-graded here.
    beyond = min((p for p in list(on_board or ()) + ranked
                  if (p.get('dist_mm') or 0.0) > radius + 1e-9),
                 key=lambda p: (p.get('dist_mm') or 0.0), default=None)
    census = {'ranked_on_board': None if on_board is None else len(first),
              'ranked': len(out),
              'ranked_before_radius': len(ranked),
              'requested_dropped': requested_dropped,
              'nearest_beyond_radius': (
                  None if beyond is None else
                  {'x': beyond['x'], 'y': beyond['y'], 'rot': beyond['rot'],
                   'dist_mm': beyond.get('dist_mm')}),
              'lattice': len(lattice),
              'radius_mm': radius,
              'step_mm': step,
              'rotations': [rot],
              'dropped_total': diag.get('dropped_total', 0),
              # The rotations vetoed at offset 0 -- the REQUEST's spot.
              'dropped_in_place': diag.get('dropped_in_place', []),
              # The ranker's verdict on the part's pose on the input board:
              # {check, blocker} when vetoed there, else None (#1256).
              'input_pose_vetoed': home_veto,
              # #1113: which check vetoed them, and against whom.
              'dropped_by': diag.get('dropped_by', {})}
    return ordered, census


# ---------------------------------------------------------------------------
# replicate --search: a copy that does not fit where its anchor puts it
# ---------------------------------------------------------------------------

def search_offsets(radius: float, step: float) -> List[Tuple[float, float, float]]:
    """`[(dx, dy, dist), ...]`: every point of a square lattice of pitch
    `step` within the DISTANCE `radius` of the origin, nearest first, ties
    broken by (dy, dx) so the order is the same on every machine. (0, 0)
    leads: the copy as asked is always the first candidate."""
    n = int(math.floor(radius / step + 1e-9))
    out = []
    for i in range(-n, n + 1):
        for j in range(-n, n + 1):
            dx, dy = round(i * step, 9), round(j * step, 9)
            dist = math.hypot(dx, dy)
            if dist <= radius + 1e-9:
                out.append((round(dist, 9), dy, dx))
    out.sort()
    return [(dx, dy, dist) for dist, dy, dx in out]


def shift_rows(pcb_data, rows: Sequence[Dict], dx: float, dy: float) -> List[Dict]:
    """`rows` translated by (dx, dy), each `moved` re-judged against the
    part's pose on the board (the anchor too: off its default offset it
    moves like every other member)."""
    from placement import replicate
    out = []
    for row in rows:
        ft = pcb_data.footprints[row['ref']]
        r = dict(row, x=round(row['x'] + dx, 6), y=round(row['y'] + dy, 6))
        if dx == 0.0 and dy == 0.0:
            r['x'], r['y'] = row['x'], row['y']
        r['moved'] = (abs(r['x'] - float(ft.x)) > replicate.POSE_EPS
                      or abs(r['y'] - float(ft.y)) > replicate.POSE_EPS
                      or replicate._rot_differs(            # noqa: SLF001
                          r['rot'], float(ft.rotation or 0.0) % 360.0)
                      or r['flip'])
        out.append(r)
    return out


def search_replicate(board_path: str, pcb, cand: str, before: Dict,
                     placements: Sequence[Dict], targets: Sequence[Dict], *,
                     clearance: float, board_edge_clearance, declared_keepouts,
                     strict: bool, guarded) -> Tuple[Optional[List[Dict]], List[Dict]]:
    """`replicate --search MM` (#1211): fit each searched target's copy
    nearest its default offset. `(placements, census)`; `placements` is None
    when some target found no accepted offset.

    `targets`: `[{'prefix', 'rows', 'radius', 'step'}]` in call order (each
    replicate op's --to order). They are fitted ONE AT A TIME: while target k
    is tried, the targets fitted before it sit where they were fitted, every
    other op of the call where it asked, and the targets AFTER it are left
    out of the grade on both sides (their parts are deleted from the parsed
    input and from each parsed candidate alike): they are about to move, so
    where they sit on the input board is no obstacle, and where their own
    copy will land is decided when their turn comes -- against this one. A
    candidate is accepted by the call's own verdict -- `worsened` empty, and
    `is_clean` under `strict` -- on a staged, written, re-parsed board,
    never on a model of one; the first accepted offset in `search_offsets`
    order (nearest the default, then dy, dx) is kept. The LAST target is
    graded with nothing left out against the real input, so a search that
    succeeds hands back the very board the caller's verdict passes. A
    candidate that would move a guarded part (KiCad-locked or a seated
    plug, not unlocked in the call) is skipped and counted."""
    from kicad_parser import parse_kicad_pcb
    from placement.portfolio import copy_siblings
    from placement.writer import (OutlineOwnerMove, SideFlipUnsupported,
                                  write_placed_output)
    searched = {r['ref'] for t in targets for r in t['rows']}
    fixed = [p for p in placements if p['reference'] not in searched]
    chosen: List[Dict] = []
    census: List[Dict] = []

    def _without(parsed, refs):
        for r in refs:
            parsed.footprints.pop(r, None)
        return parsed

    for k, t in enumerate(targets):
        later = {r['ref'] for u in targets[k + 1:] for r in u['rows']}
        if later:
            with contextlib.redirect_stdout(io.StringIO()):
                base = grade(_without(parse_kicad_pcb(board_path), later),
                             board_path, clearance, board_edge_clearance,
                             declared_keepouts)
        else:
            base = before
        lattice = search_offsets(t['radius'], t['step'])
        row = {'sheet': '/' + t['prefix'],
               'name': (t.get('report') or {}).get('sheet', {}).get('name'),
               'radius_mm': t['radius'],
               'step_mm': t['step'], 'candidates': len(lattice), 'tried': 0,
               'skipped_guarded': 0, 'found': False, 'delta': None,
               'dist_mm': None, 'later_left_out': len(later),
               'refused_by': {}}
        census.append(row)
        found = None
        for dx, dy, dist in lattice:
            shifted = shift_rows(pcb, t['rows'], dx, dy)
            moves = [r for r in shifted if r['moved']]
            if guarded & {r['ref'] for r in moves}:
                row['skipped_guarded'] += 1
                continue
            row['tried'] += 1
            trial = fixed + [_replicate_note(pcb, r)[1]
                             for r in chosen + moves]
            # The writer and the grade narrate every call; a hundred
            # candidates of that would bury the one line that matters.
            with contextlib.redirect_stdout(io.StringIO()):
                try:
                    write_placed_output(board_path, cand, trial)
                except (OutlineOwnerMove, SideFlipUnsupported):
                    # Off its default offset a member the default left
                    # alone (the anchor) moves too -- maybe one the writer
                    # cannot.
                    row['refused_by']['writer'] = (
                        row['refused_by'].get('writer', 0) + 1)
                    continue
                copy_siblings(board_path, cand)
                g = grade(_without(parse_kicad_pcb(cand), later), cand,
                          clearance, board_edge_clearance, declared_keepouts)
            b = worsened(base, g)
            if not b and (not strict or is_clean(g)):
                found = shifted
                row.update(found=True, delta=[dx, dy], dist_mm=round(dist, 6))
                break
            for cat in regrade_refusal(base, g, b, strict):
                row['refused_by'][cat] = row['refused_by'].get(cat, 0) + 1
        if found is None:
            return None, census
        chosen.extend(r for r in found if r['moved'])
        t['chosen_rows'] = found
    return fixed + [_replicate_note(pcb, r)[1] for r in chosen], census


# ---------------------------------------------------------------------------
# locks
# ---------------------------------------------------------------------------

def check_lock_refs(pcb_data, lock_refs=(), unlock_refs=()) -> None:
    """Refuse a lock/unlock request the board cannot honour.

    Two ways it could not, both of which used to exit 0 with a summary that
    said otherwise:

    * a ref that is not on the board. `set NOSUCHREF` refuses with code 2 and
      the #726 hint, while `lock NOSUCHREF` wrote the board, exited 0, and
      listed the ref under `locked` -- telling a consumer a decision had been
      recorded that had not.
    * the same ref in BOTH lists. `apply_locks` stamps then strips, so unlock
      always won whatever the caller wrote: `unlock C4 set C4 ... lock C4` --
      the re-lock workflow this tool's own refusal message recommends -- moved
      the part, dropped the lock, and reported `locked: ["C4"]`.
    """
    known = set((pcb_data.footprints or {}))
    missing = sorted((set(lock_refs) | set(unlock_refs)) - known)
    if missing:
        raise PoseRefusal(
            "%s %s not on this board, so there is nothing to lock or unlock "
            "(the parser names duplicate references TP4 / TP4~2, #726)"
            % (', '.join(missing), 'is' if len(missing) == 1 else 'are'),
            code=2, missing=missing)
    both = sorted(set(lock_refs) & set(unlock_refs))
    if both:
        raise PoseRefusal(
            "%s is named by both lock and unlock in one call; the stamping "
            "order would decide it silently (unlock wins), and one call "
            "describes one arrangement. Say which you mean."
            % ', '.join(both), code=2, conflicting=both)


def apply_locks(board_file: str, lock_refs=(), unlock_refs=()) -> Dict:
    """Stamp / strip `(locked yes)` in place. Returns what actually changed."""
    from placement.seeder import stamp_locked, stamp_unlocked
    out = {'locked': [], 'unlocked': [], 'locked_count': 0, 'unlocked_count': 0}
    if lock_refs:
        out['locked'] = sorted(lock_refs)
        out['locked_count'] = stamp_locked(board_file, list(lock_refs))
    if unlock_refs:
        out['unlocked'] = sorted(unlock_refs)
        out['unlocked_count'] = stamp_unlocked(board_file, list(unlock_refs))
    return out


# ---------------------------------------------------------------------------
# the verb
# ---------------------------------------------------------------------------

def apply_poses(board_path: str, out_path: Optional[str], ops: Sequence[Dict],
                *, pcb_data=None, clearance=None, board_edge_clearance=None,
                track_width=None, lock_refs=(), unlock_refs=(),
                snap: bool = False, snap_radius: float = 2.0,
                snap_step: float = 0.25, snap_tries: int = 6,
                strict: bool = False,
                force: bool = False, dry_run: bool = False,
                intent=None, group_sources: Sequence[str] = ()) -> Dict:
    """Apply `ops` to `board_path`, grade the result, write it to `out_path`.

    `intent` (a loaded floorplan `Intent`, #959): a moved part that ends
    further outside its block's zone than it started, by an ERROR-severity
    `zone_containment`, is refused like a legality regression (`force`
    writes anyway). Without it no zone is read.

    Returns the summary a caller prints as `JSON_SUMMARY`. Raises `PoseRefusal`
    when the request cannot be honoured -- an unknown ref, a face with no pads,
    or a pose that makes the board's pad legality worse (unless `force`).

    The candidate is STAGED and graded before anything reaches `out_path`, so a
    refused request leaves no half-written board behind and `dry_run` grades
    exactly what a real run would have written.

    `pcb_data` is an OPTIMISATION, not a second input: it must be the parse of
    `board_path`. Ops and both grades read it, while the writer re-reads the
    file, so handing over a `PCBData` that differs from what is on disk would
    describe one board and write another. An in-process caller with a live
    pcbnew board must write it out first and pass that path.
    """
    from kicad_parser import parse_kicad_pcb
    from placement.portfolio import copy_siblings
    from placement.writer import write_placed_output

    if not dry_run and not out_path:
        raise PoseRefusal("a write needs an output path; pass one, or "
                          "--dry-run to grade without writing", code=2)
    requested = {'clearance': clearance, 'board_edge_clearance': board_edge_clearance}
    clearance, board_edge_clearance, track_width, knobs = resolve_knobs(
        board_path, clearance, board_edge_clearance, track_width)
    for key, value in requested.items():
        knobs[key].update(requested=value, units='mm')
    pcb = pcb_data if pcb_data is not None else parse_kicad_pcb(board_path)

    replicated: List[Dict] = []
    replicate_rows: List = []
    placements, notes = resolve_ops(pcb, ops, clearance=clearance,
                                    track_width=track_width,
                                    reports=replicated,
                                    replicate_rows=replicate_rows)
    check_lock_refs(pcb, lock_refs, unlock_refs)
    # #1098: the intent's keep-outs reach every grade below, so a declared
    # `mating:<ref>` is the region this verb holds a move to.
    declared_keepouts = tuple(getattr(intent, 'keepouts', None) or ())

    # A KiCad lock is a DECISION someone recorded in the file -- the seeder
    # stamps the intent's must_lock refs there, and run 25 stamped its rotation
    # decisions there so no later repair step would lift them. So a direct
    # order to move a locked part is refused, and the escape hatch is the
    # deliberate one this tool already has: name the ref in `unlock` in the
    # SAME call, which is honoured because every op reads the input board.
    # `--force` deliberately does NOT open this: it is about legality, and a
    # waiver flag that also silently lifts locks would make every lock in the
    # chain conditional on a flag nobody re-reads.
    if placements:
        # #1098: a PCB-edge plug seated at its edge carries the keep-out that
        # keeps parts off its tongue; moving it inland would take the region
        # with it and read as an improvement. Treated as a lock: `unlock`
        # the ref in the same call if the move is meant. The placement
        # state's own set (`seated_plugs`): a plug named by a declared `mating:` keep-out
        # counts too, and a plug that is NOT seated (in the pile, hanging
        # across an edge) is free to move.
        from placement.floorplan import seated_plugs, with_derived_keepouts
        try:
            _plugs = seated_plugs(
                with_derived_keepouts(declared_keepouts, pcb, board_path),
                pcb, board_path)
        except Exception as exc:                             # noqa: BLE001
            raise PoseRefusal(
                "cannot tell whether a moved part is a seated PCB-edge plug "
                "(%s: %s); nothing was written" % (type(exc).__name__, exc))
        _moved_plugs = sorted({p['reference'] for p in placements}
                              & (_plugs - set(unlock_refs)))
        if _moved_plugs:
            raise PoseRefusal(
                "%s %s a PCB-edge plug seated at its edge: its tongue's "
                "keep-out (mating:%s) belongs to that pose, and moving it "
                "would move the region too or leave it behind. Name it in "
                "`unlock` in the same call if you mean it."
                % (', '.join(_moved_plugs),
                   'is' if len(_moved_plugs) == 1 else 'are',
                   _moved_plugs[0]),
                locked=_moved_plugs)
        from placement.parser import extract_locked_refs
        locked_now = extract_locked_refs(board_path)
        blocked = sorted({p['reference'] for p in placements}
                         & (locked_now - set(unlock_refs)))
        if blocked:
            raise PoseRefusal(
                "%s %s locked in the board (KiCad `(locked yes)`), and a lock "
                "is a decision, not a preference. Unlock in the same call if "
                "you mean it: place_pose.py IN OUT unlock %s set %s ..."
                % (', '.join(blocked), 'is' if len(blocked) == 1 else 'are',
                   ' '.join(blocked), blocked[0]),
                locked=blocked)
    summary: Dict = {
        'input': board_path,
        'output': None if dry_run else out_path,
        'dry_run': bool(dry_run),
        'ops': notes,
        'moved': [n for n in notes if n['moved']],
        'knobs': knobs,
        'clearance': clearance,
        'board_edge_clearance': board_edge_clearance,
        'track_width': track_width,
        'snapped': None,
        'nearest_legal': None,
        'refused': None,
        'forced': False,
        # The lock INTENT rides in every summary, refusals included: it used
        # to be added only on the two paths that reached the stamping, so a
        # refused dry run had no `locked` key at all and a reader could not
        # tell "asked for nothing" from "never got that far".
        'locked': sorted(lock_refs),
        'unlocked': sorted(unlock_refs),
        'locked_count': None,
        'unlocked_count': None,
        # What a dry run WOULD have written, since the flag's own help says
        # it reports the path.
        'would_write': out_path,
    }
    if replicated:
        summary['replicate'] = replicated

    before = grade(pcb, board_path, clearance, board_edge_clearance,
                   declared_keepouts)
    stage = tempfile.TemporaryDirectory(prefix='place_pose_')
    try:
        cand = os.path.join(stage.name, 'candidate.kicad_pcb')
        if placements:
            from placement.writer import OutlineOwnerMove, SideFlipUnsupported
            try:
                write_placed_output(board_path, cand, placements)
            except (OutlineOwnerMove, SideFlipUnsupported) as exc:
                # The writer REFUSES what it cannot write faithfully (#829 a
                # part that draws the outline, #714 a side flip it would have
                # to guess at -- `replicate` copies the source's side). Said
                # with a summary like every other refusal, not a traceback.
                summary.update(output=None, refused=(
                    'the writer cannot write this arrangement: %s' % exc))
                raise PoseRefusal(summary['refused'], summary=summary) from exc
        else:
            shutil.copyfile(board_path, cand)
        copy_siblings(board_path, cand)
        cand_pcb = parse_kicad_pcb(cand)
        after = grade(cand_pcb, cand, clearance, board_edge_clearance,
                      declared_keepouts)
        bad = worsened(before, after)
        snap_ran = False
        search_finding = None

        search_targets = [
            {'prefix': t['sheet']['path'].lstrip('/'),
             'rows': [r for r in rows
                      if r['target'] == t['sheet']['path'].lstrip('/')],
             'radius': float(op['search']),
             'step': float(op.get('search_step') or SEARCH_STEP),
             'report': t}
            for op, report, rows in replicate_rows if op.get('search')
            for t in report['to']]
        if search_targets and (bad or (strict and not is_clean(after))):
            # #1211: the copy as asked does not fit; try each searched
            # target's offset nearest its default (`search_replicate`).
            from placement.floorplan import seated_plugs, with_derived_keepouts
            from placement.parser import extract_locked_refs
            guarded = ((extract_locked_refs(board_path)
                        | seated_plugs(with_derived_keepouts(
                            declared_keepouts, pcb, board_path),
                            pcb, board_path))
                       - set(unlock_refs))
            found, census = search_replicate(
                board_path, pcb, cand, before, placements, search_targets,
                clearance=clearance, board_edge_clearance=board_edge_clearance,
                declared_keepouts=declared_keepouts, strict=strict,
                guarded=guarded)
            for t, c in zip(search_targets, census):
                t['report']['search'] = c
            for t in search_targets[len(census):]:
                t['report']['search'] = {'sheet': '/' + t['prefix'],
                                         'not_tried': 'an earlier target '
                                                      'found no offset'}
            if found is not None:
                placements = found
                by_ref = {}
                for t in search_targets:
                    rep = t['report']
                    chosen = t['chosen_rows']
                    rep['offset_default'] = list(rep['offset'])
                    rep['offset'] = [round(rep['offset'][0]
                                           + rep['search']['delta'][0], 6),
                                     round(rep['offset'][1]
                                           + rep['search']['delta'][1], 6)]
                    rep['moved'] = sum(1 for r in chosen if r['moved'])
                    for r in chosen:
                        by_ref[r['ref']] = _replicate_note(pcb, r)[0]
                notes[:] = [by_ref.get(n['ref'], n)
                            if n['kind'] == 'replicate' else n for n in notes]
                summary['moved'] = [n for n in notes if n['moved']]
                write_placed_output(board_path, cand, placements)
                copy_siblings(board_path, cand)
                cand_pcb = parse_kicad_pcb(cand)
                after = grade(cand_pcb, cand, clearance, board_edge_clearance,
                              declared_keepouts)
                bad = worsened(before, after)
            else:
                miss = census[-1]
                search_finding = (
                    '--search found no offset within %g mm of %s at which the '
                    'copy grades no worse (%d of %d lattice offsets tried at a '
                    '%g mm step, %d skipped for moving a locked part; refused '
                    'by %s)' % (
                        miss['radius_mm'], miss['name'] or miss['sheet'],
                        miss['tried'],
                        miss['candidates'], miss['step_mm'],
                        miss['skipped_guarded'],
                        ', '.join('%s x%d' % kv for kv in sorted(
                            miss['refused_by'].items(),
                            key=lambda kv: (-kv[1], kv[0]))) or 'nothing'))
                # Back to the copy as asked: the refusal below reports THAT.
                write_placed_output(board_path, cand, placements)
                copy_siblings(board_path, cand)

        if snap and replicated:
            # A replicate is ONE rigid copy: snapping a member would break
            # the arrangement it was asked for, even when only one part moved.
            summary['snap_census'] = {
                'skipped': 'snap relocates one part, and a replicate is a '
                           'rigid copy of a whole sheet; it is graded as '
                           'given'}
        elif snap and len(placements) != 1:
            # SAID, not silently dropped: --snap/--near relocates ONE part,
            # and a call carrying several ops has no single point to sweep
            # around. A flag that quietly does nothing is how a caller
            # concludes the engine considered an alternative and found none.
            summary['snap_census'] = {
                'skipped': 'snap applies to exactly one pose op; this call '
                           'carries %d' % len(placements)}
        if ((bad or (strict and not is_clean(after))) and snap
                and len(placements) == 1 and not replicated):
            # Rank around the REQUESTED point, not the part's old one, and
            # judge the part against its pose on the INPUT board (#1256):
            # the state is built from `board_path` -- every seed, the
            # baseline the ranker's pad check is relative to, is the pose the
            # re-grade below compares with -- and the part is then moved to
            # the request, where the sweep centres. Built from the staged
            # board, the request was its own baseline, and the ranker passed
            # poses carrying the very conflict that refused it. The rotation
            # stays as asked -- a snap relocates a decision, it does not
            # overrule it.
            #
            # Each candidate is RE-GRADED rather than trusted, because the
            # ranking's own legality (`QuenchState.candidate_valid`, an AABB
            # gate) and this verb's verdict (`grade_pad_legality`, exact
            # geometry) are different currencies: the ranker can hand back the
            # very pose that was refused. Walking the list until one grades
            # clean is what makes --snap mean "a pose that passes", not "the
            # first pose the other instrument liked".
            ref = placements[0]['reference']
            want_rot = placements[0]['new_rotation']
            try:
                poses, census = snap_candidates(
                    board_path, ref, rot=want_rot, clearance=clearance,
                    board_edge_clearance=board_edge_clearance,
                    radius=snap_radius, step=snap_step, pcb_data=pcb,
                    at=(placements[0]['new_x'], placements[0]['new_y']))
            except PoseRefusal as exc:
                # The refusal rides in the SAME summary every other refusal
                # here carries -- ops, knobs and lock intent included -- so a
                # snap that cannot rank does not print an empty document. And
                # like every other refusal, it names no output: nothing was
                # written.
                summary['refused'] = exc.reason
                summary['output'] = None
                raise PoseRefusal(exc.reason, code=exc.code, ref=ref,
                                  summary=summary) from exc
            snap_ran = True
            # #1209: nothing is named here yet. This used to store the first
            # RANKED candidate before any re-grade, and when every candidate
            # then failed, the refusal offered that very pose back -- the
            # requested one, 0.000 mm away -- as something to "try with
            # --snap", to a call that had just snapped. A snap names only
            # what its own re-grade passed (set below), or nothing.
            summary['snap_census'] = census
            # WHY each re-graded candidate was refused: {category: candidates}
            # and {category: {pair: candidates}}, so a refusal can say what
            # the census found instead of only "no".
            refused_by: Dict[str, int] = {}
            refused_pairs: Dict[str, Dict[str, int]] = {}

            def _grade_pose(cp):
                """Write one candidate onto the staging board and grade it."""
                trial = [{'reference': ref, 'new_x': cp['x'],
                          'new_y': cp['y'], 'new_rotation': cp['rot']}]
                write_placed_output(board_path, cand, trial)
                copy_siblings(board_path, cand)
                pcb_c = parse_kicad_pcb(cand)
                g = grade(pcb_c, cand, clearance, board_edge_clearance,
                          declared_keepouts)
                b = worsened(before, g)
                if b or (strict and not is_clean(g)):
                    for k, pairs in regrade_refusal(before, g, b,
                                                    strict).items():
                        refused_by[k] = refused_by.get(k, 0) + 1
                        bucket = refused_pairs.setdefault(k, {})
                        for pr in pairs:
                            bucket[pr] = bucket.get(pr, 0) + 1
                return trial, pcb_c, g, b

            # TWO PHASES, each with its OWN budget:
            #
            # * the RANKED rung is pre-filtered by `candidate_valid`, so its
            #   candidates usually pass this verb's grade too. It is nearest
            #   first among the poses the gate admits, so when the nearer
            #   poses are vetoed its first answer can still be several mm out.
            # * the LATTICE rung is unfiltered, and its nearest members sit
            #   right beside a point that was just refused, so they mostly
            #   fail.
            #
            # So: take the ranked rung's first passing pose, then spend a
            # SECOND budget on lattice poses that are NEARER than it, and
            # prefer one of those if it passes. A single shared cap in either
            # order loses one of the two -- lattice-first starves the ranked
            # rung, ranked-first starves the lattice.
            #
            # From an input pose off the board, a THIRD budget comes first --
            # the ranking with the escape branch closed (`ranked_on_board`).
            # It only adds candidates ahead of the ranked rung, whose own
            # budget and order are unchanged. A pose already re-graded is not
            # graded twice, nor counted again: every such pose was refused
            # (the walk stops at the first that passes), so skipping it
            # cannot lose a later rung's answer.
            #
            # And when that rung answers, a nearer lattice pose must grade no
            # worse than ITS answer too, not only no worse than the input:
            # from an input off the board, "no worse than the input" admits
            # pad copper still past the outline.
            chosen = None
            graded = set()

            def _walk(cands, floor=None):
                for cp in [p for p in cands
                           if _pose_key(p) not in graded][:snap_tries]:
                    graded.add(_pose_key(cp))
                    trial, pcb_c, g, b = _grade_pose(cp)
                    if (not b and (not strict or is_clean(g))
                            and (floor is None or not worsened(floor, g))):
                        return (cp, trial, pcb_c, g, b)
                return None

            for rung in ('ranked_on_board', 'ranked'):
                chosen = _walk([p for p in poses if p['rung'] == rung])
                if chosen is not None:
                    break
            limit = chosen[0]['dist_mm'] if chosen else float('inf')
            nearer = _walk([p for p in poses
                            if p['rung'] == 'lattice'
                            and (p['dist_mm'] or 0.0) < limit],
                           floor=chosen[3] if chosen and chosen[0]['rung']
                           == 'ranked_on_board' else None)
            chosen = nearer or chosen                # strictly nearer
            tried = len(graded)
            summary['snap_census']['candidates_tried'] = tried

            if chosen is not None:
                cand_pose, placements, cand_pcb, after, bad = chosen
                # The one pose this snap may name: its own re-grade passed it.
                summary['nearest_legal'] = cand_pose
                summary['nearest_legal_basis'] = (
                    'the snapped pose: ranked by pose_score.rank_poses '
                    '(rung ranked, or ranked_on_board from an input pose off '
                    'the board) or taken from the lattice (rung lattice), '
                    'then RE-GRADED here and passed before it was written')
                # RE-STAGE the winner. Phase 2 may have graded other poses
                # after it, so the staging file holds the LAST candidate
                # tried, not the chosen one -- and the staging file is what
                # gets promoted. (Caught by the test that compares the written
                # board against `snapped.to`, which is the whole point of
                # asserting on the FILE rather than on the summary.)
                write_placed_output(board_path, cand, placements)
                copy_siblings(board_path, cand)
                summary['snapped'] = {
                    'ref': ref,
                    'to': [cand_pose['x'], cand_pose['y'], cand_pose['rot']],
                    'dist_mm': cand_pose.get('dist_mm'),
                    # WHICH rung answered: 'ranked' means the pose scorer and
                    # this verb agreed, 'lattice' means only this verb's own
                    # grade accepted it; 'ranked_on_board' is 'ranked' with
                    # the scorer's off-board escape licence closed (#1277).
                    'rung': cand_pose.get('rung'),
                    'candidates_tried': tried}
                for n in notes:
                    if n['ref'] == ref:
                        n['to'] = [round(cand_pose['x'], 4),
                                   round(cand_pose['y'], 4), cand_pose['rot']]
                        n['moved'] = n['to'] != n['from']
                        n['snapped'] = True
                summary['moved'] = [n for n in notes if n['moved']]
            else:
                # Nothing TRIED graded clean: put the board back to the pose
                # the caller actually asked for, so the refusal below reports
                # THEIR request rather than the last thing tried.
                # `candidates_tried` against `ranked + lattice` is how a reader
                # tells "there was nothing" from "we stopped looking".
                # `untried` counts distinct poses: a ranked_on_board pose is
                # often on the ranked rung too (#1277).
                summary['nearest_legal_basis'] = (
                    'none: --snap re-graded every candidate it tried and '
                    'refused each one (snap_census.regrade_refused_by), and '
                    'a pose its own re-grade refused is not offered back '
                    '(#1209)')
                summary['snap_census']['regrade_refused_by'] = dict(
                    sorted(refused_by.items(), key=lambda kv: (-kv[1], kv[0])))
                summary['snap_census']['regrade_refused_pairs'] = {
                    k: dict(sorted(v.items(), key=lambda kv: (-kv[1], kv[0])))
                    for k, v in sorted(refused_pairs.items()) if v}
                summary['snap_census']['untried'] = max(
                    0, len({_pose_key(p) for p in poses}) - tried)
                summary['snap_census']['snap_tries'] = snap_tries
                write_placed_output(board_path, cand, placements)
                copy_siblings(board_path, cand)
                cand_pcb = parse_kicad_pcb(cand)
                after = grade(cand_pcb, cand, clearance, board_edge_clearance,
                              declared_keepouts)
                bad = worsened(before, after)

        # A `face` op's rotation is PREDICTED (FACE_CYCLE) and then MEASURED on
        # the board that was actually written, because the prediction is a
        # property of the rigid body and the engine's face rule is not exactly
        # rotation-invariant: `face_of` takes an argmin against a box that is
        # not square, so a CORNER pad can change sides under a rotation that
        # carries the row. Measured on esp_prog U1 -- ONE part, rows of 9 and
        # 10 pads, all three deltas: every multi-pad row keeps its predicted
        # face for 8-9 of its members, and the only complete miss is a one-pad
        # "row" at 180 degrees. That is the evidence for the prediction being
        # usable; it is NOT evidence about small rows, and a 2-pad row measured
        # on flat_hierarchy is often 180-degree symmetric, so its face cannot
        # be changed by any rotation at all. Hence: predict, MEASURE on the
        # board actually written, and say which happened.
        findings: List[str] = []
        face_miss = []
        for n in [n for n in notes if n['kind'] == 'face']:
            by = part_faces(cand_pcb, n['ref'], clearance=clearance,
                            track_width=track_width)
            # By INDEX, for the reason `resolve_ops` records the indices: a
            # `{pad_number: face}` map silently answers about a DIFFERENT pad
            # whenever a footprint repeats a number, which 196 parts across 16
            # of this repo's 22 tracked boards do.
            after_pads = (cand_pcb.footprints[n['ref']].pads or ())
            face_by_ix = {}
            for f, pads in by.items():
                for p in pads:
                    face_by_ix[id(p)] = f
            counts: Dict[str, int] = {}
            for i in n['row_pad_ix']:
                key = ('gone' if i >= len(after_pads)
                       else face_by_ix.get(id(after_pads[i]), 'gone'))
                counts[key] = counts.get(key, 0) + 1
            hit = counts.get(n['target_face'], 0)
            n['row_landed'] = counts
            n['row_on_target'] = [hit, len(n['row_pad_ix'])]
            n['row_predicted_face'] = rotate_face(n['face'],
                                                  n['rotation_delta'])
            # A row that did not MOVE under a rotation that should have
            # carried it is symmetric under that rotation -- common on a
            # 2-pad passive, where `assign_faces` answers the same at 0 and
            # 180 degrees. "landed north x2" is true and unhelpful; "this row
            # cannot be aimed by rotating" is the fact the caller needs.
            n['row_symmetric'] = (len(counts) == 1
                                  and n['face'] in counts
                                  and n['face'] != n['target_face'])
            if hit * 2 <= len(n['row_pad_ix']):
                face_miss.append(n)

        summary.update(_legality_row(before, after))
        if replicated:
            # Per part, so a refused copy says WHICH members collide and with
            # whom, not only which tally moved: the NEW pairs (and stacks)
            # each moved replicate member is in.
            fresh: Dict[str, List[str]] = {}
            for pk in PAIR_KEYS:
                had = {tuple(p[:2]) for p in (before.get(pk) or ())}
                for p in after.get(pk) or ():
                    if tuple(p[:2]) in had:
                        continue
                    # An own-pad pair names two PADS of one part (#1306).
                    for r in ({p[0].partition('.')[0]}
                              if pk == 'own_pad_short_pairs' else p[:2]):
                        fresh.setdefault(r, []).append(
                            '%s %s/%s' % (pk, p[0], p[1]))
            # ...and the per-PART channels, so a member the copy put past the
            # outline, into the edge band or a keep-out is named too: those
            # carry no pair, and a refusal that lists only pairs leaves the
            # caller unable to say which member to leave out (#1211).
            for rk in ('oob_pad_copper_refs', 'oob_keepout_copper_refs',
                       'oob_graphic_copper_refs'):
                had = {r[0]: r[1] for r in (before.get(rk) or ())}
                for r in after.get(rk) or ():
                    if r[1] > had.get(r[0], 0.0) + MAGNITUDE_EPS:
                        fresh.setdefault(r[0], []).append(
                            '%s %s' % (rk[:-len('_refs')], r[0]))
            had = {f['pad_ref']: f.get('shortfall_mm') or 0.0
                   for f in (before.get('pad_edge') or {}).get('findings') or ()}
            for f in (after.get('pad_edge') or {}).get('findings') or ():
                if (f.get('shortfall_mm') or 0.0) > (
                        had.get(f['pad_ref'], 0.0) + MAGNITUDE_EPS):
                    fresh.setdefault(f['pad_ref'].partition('.')[0], []).append(
                        'pad_edge %s' % f['pad_ref'])
            for n in notes:
                if n['kind'] == 'replicate' and n['moved']:
                    n['new_conflicts'] = sorted(fresh.get(n['ref'], ()))
        for channel in ('pad_edge_before', 'pad_edge_after'):
            summary[channel]['source'] = knobs['board_edge_clearance']['source']
            summary[channel]['requested_mm'] = requested['board_edge_clearance']
        # TWO keys, because one word cannot carry both facts and the wrong one
        # was being published: `legal` used to mean "no worse than the input",
        # so a board still carrying a pad conflict reported `legal: true`.
        # `legal` is now the ABSOLUTE fact a reader takes it for, and
        # `no_worse` is the relative verdict this verb ACTS on.
        summary['no_worse'] = not bad
        summary['legal'] = is_clean(after)
        summary['legal_basis'] = (
            'legal = measured pad/hole/outline channels (pad copper AND '
            'footprint graphic copper) are clean, no two parts\' pads are '
            'stacked (any net), and edge coverage is complete; no_worse = no '
            'measured category worsened relative to the input board. '
            'Neither verifies what legal_unmeasured lists.')
        summary['legal_scope'] = list(LEGAL_SCOPE)
        summary['legal_unmeasured'] = list(LEGAL_UNMEASURED)

        if face_miss:
            reason = '; '.join(
                ("%s: the %s row cannot be aimed by rotating -- it reads %s "
                 "at every rotation this measured (a 2-pad row is often "
                 "180-degree symmetric). Move the part instead."
                 % (n['ref'], n['face'], n['face'])) if n.get('row_symmetric')
                else ("%s: the %s row was aimed at %s but landed %s"
                      % (n['ref'], n['face'], n['target_face'],
                         ', '.join('%s x%d' % (f, c)
                                   for f, c in sorted(
                                       n['row_landed'].items()))))
                for n in face_miss)
            # ACCUMULATED, not assigned: the legality block below used to
            # overwrite this, so a forced run that missed its aim AND made the
            # board worse shipped with only the legality finding in `refused`
            # -- the face facts surviving nowhere a reader looks.
            findings.append(reason)
            summary['refused'] = '; '.join(findings)
            if not force:
                summary['output'] = None      # nothing was written
                raise PoseRefusal(summary['refused'], summary=summary)
            summary['forced'] = True

        if intent is not None and not placements:
            summary['zone_check'] = {
                'intent': getattr(intent, 'source_path', '') or None,
                'refs': [], 'rows': [], 'worse': [],
                'skipped': 'no op in this call moves a part, so no zone '
                           'can get worse'}
        if intent is not None and placements:
            zc = zone_check(intent, pcb, board_path, cand_pcb, cand,
                            [p_['reference'] for p_ in placements],
                            group_sources=group_sources, clearance=clearance,
                            board_edge_clearance=board_edge_clearance)
            summary['zone_check'] = zc
            if zc.get('worse'):
                findings.append(zc['reason'])
                summary['refused'] = '; '.join(findings)
                if not force:
                    summary['output'] = None      # nothing was written
                    raise PoseRefusal(summary['refused'], summary=summary)
                summary['forced'] = True

        if bad or (strict and not summary['legal']):
            if (summary['nearest_legal'] is None and len(placements) == 1
                    and not snap_ran and not replicated):
                # Name the alternative even when the caller did not ask to
                # snap: a refusal that only says no costs the caller another
                # round trip to find out what WOULD have worked. NOT on a call
                # that snapped (#1209): its own census is the answer there,
                # and an unverified ranker pose "to try with --snap" is advice
                # it has already taken.
                try:
                    # #1256: from the INPUT board, centred on the refused
                    # request, which is never named back.
                    poses, _d = nearest_legal(
                        board_path, placements[0]['reference'],
                        clearance=clearance,
                        board_edge_clearance=board_edge_clearance,
                        radius=max(snap_radius, 2.0), step=snap_step,
                        rotations=(placements[0]['new_rotation'],),
                        pcb_data=pcb,
                        at=(placements[0]['new_x'], placements[0]['new_y'],
                            placements[0]['new_rotation']))
                    summary['nearest_legal'] = poses[0] if poses else None
                    summary['nearest_legal_basis'] = (
                        'pose_score.rank_poses / QuenchState.candidate_valid '
                        '-- an AABB gate, NOT re-graded: a suggestion to try, '
                        'not a pose this verb has verified')
                    # An empty ranking has two opposite meanings and they are
                    # reported apart (run-7 S4): "nowhere to go" and "the
                    # knobs veto even staying put". A swallowed exception is
                    # a THIRD, so it is named rather than folded into null.
                    summary['nearest_legal_census'] = {
                        'ranked': len(poses),
                        # #1277: how many of them are on the board, ranked
                        # first; None when the input pose is on the board.
                        'ranked_on_board': _d.get('ranked_on_board'),
                        'dropped_total': _d.get('dropped_total', 0),
                        'dropped_in_place': _d.get('dropped_in_place', []),
                        'dropped_by': _d.get('dropped_by', {}),
                        # The SWEEP's radius, and it is a Chebyshev box half
                        # width, not the Euclidean bound `--radius` applies to
                        # a snap: this list is information about what exists,
                        # so a pose further out than the caller would accept
                        # is still worth naming.
                        'sweep_radius_mm': max(snap_radius, 2.0),
                        'rotations': [placements[0]['new_rotation']]}
                except Exception as exc:                     # noqa: BLE001
                    summary['nearest_legal'] = None
                    # `PoseUnrankable.why` is the bare cause: its full reason
                    # tells the reader to run `place_pose set`, which is this
                    # very command (#959).
                    summary['nearest_legal_census'] = {
                        'error': '%s: %s' % (type(exc).__name__,
                                             getattr(exc, 'why', None)
                                             or exc)}
            findings.append(_refusal_reason(bad, strict, before, after,
                                            summary))
            if search_finding:
                findings.append(search_finding)
            summary['refused'] = '; '.join(findings)
            if not force:
                # A refusal writes nothing, so the summary must not name a
                # path: the two refusal channels disagreed about this, and a
                # machine caller cannot key on a field that means "written"
                # on one path and "would have been" on the other.
                summary['output'] = None
                raise PoseRefusal(summary['refused'], summary=summary)
            summary['forced'] = True

        if dry_run:
            # The lock keys are already in the summary with `None` counts:
            # "0 stamped" and "not attempted" must not read alike, and every
            # summary carries the intent whether or not this path is reached.
            return summary

        # Locks are stamped on the STAGED board and VERIFIED before anything is
        # promoted. Two reasons, both measured:
        #
        # * the move guard above is lifted by the REQUEST (`locked_now -
        #   unlock_refs`), so an unlock that silently fails would move a part
        #   whose lock survives into the output -- exit 0, `unlocked 0`, and a
        #   locked part at a new pose. Verifying here means a failed unlock
        #   refuses instead, with nothing written.
        # * promoting a finished file also removes the in-place special case:
        #   the candidate always lives in the staging dir, so writing the
        #   output can never be a file copying onto itself (a case-only path
        #   difference used to raise `shutil.SameFileError` and exit 1).
        if lock_refs or unlock_refs:
            summary.update(apply_locks(cand, lock_refs, unlock_refs))
            from placement.parser import extract_locked_refs
            still = sorted(set(unlock_refs) & extract_locked_refs(cand))
            if still:
                reason = (
                    "unlock did not take on %s: the board still carries "
                    "`(locked yes)` for it after stamping, so a move guarded "
                    "by that lock would have been written with the lock "
                    "intact." % ', '.join(still))
                summary['refused'] = reason
                summary['output'] = None
                raise PoseRefusal(reason, summary=summary, unlock_failed=still)

        _promote(cand, out_path, summary)
        return summary
    finally:
        stage.cleanup()


def zone_check(intent, pcb, board_path, cand_pcb, cand_path, refs, *,
               group_sources: Sequence[str] = (), clearance=None,
               board_edge_clearance=None) -> Dict:
    """`zone_containment` for the moved refs, on the input and on the
    candidate (#959, #998). Run 29 wrote `Ref*` out of its own zone plan at
    lap 10 and learned it only from the next grade.

    WORSE, never "outside": this verb's legality verdict is relative for the
    reason its module docstring gives, and a part on an unplaced pile starts
    outside every zone -- an absolute gate would refuse the moves that bring
    it home. So a row is worse when the candidate has an ERROR-severity
    finding and it sits further out than the input did (0 when the input was
    inside tolerance). A WARN row is reported, never refused: an author who
    demoted `zone_containment` said the zone is advisory. A zone smaller than
    the courtyard is graded on the part centre on both sides, as the rule
    grades it; a rotation that changes which way a part is graded compares
    the two readings as they come."""
    from placement import floorplan as _fp
    kw = dict(group_sources=group_sources, clearance=clearance,
              board_edge_clearance=board_edge_clearance)
    doc: Dict = {'intent': getattr(intent, 'source_path', '') or None,
                 'refs': sorted(set(refs)), 'rows': [], 'worse': []}
    try:
        was = _fp.zone_containment_of(intent, pcb, board_path, refs, **kw)
        now = _fp.zone_containment_of(intent, cand_pcb, cand_path, refs, **kw)
    except _fp.UntrustworthyOutline as exc:
        doc['unmeasured'] = ('the board outline is not one the grader may '
                             'rely on (%s), so zone_containment cannot run '
                             'here either' % exc)
        return doc
    for key in sorted(set(was) | set(now)):
        a, b = now.get(key), was.get(key)
        v = a or b
        row = {'ref': key[0], 'block': key[1],
               'zone': v.expected.get('zone'),
               'tolerance_mm': v.expected.get('tolerance_mm'),
               'outside_mm_before': b.measured['outside_mm'] if b else 0.0,
               'outside_mm_after': a.measured['outside_mm'] if a else 0.0,
               'axis': a.measured.get('axis') if a else None,
               'severity': a.severity if a else None}
        doc['rows'].append(row)
        if (a is not None and a.severity == _fp.ERROR
                and row['outside_mm_after'] > row['outside_mm_before'] + 1e-9):
            doc['worse'].append(row)
    if doc['worse']:
        doc['reason'] = '; '.join(
            "%s would sit %.2fmm past the %s edge of block %r (zone %s, "
            "tolerance %gmm; it was %s) -- the intent's zone_containment is "
            "an ERROR. Move it inside the zone, change the plan, or pass "
            "--force" % (
                r['ref'], r['outside_mm_after'], r['axis'], r['block'],
                r['zone'], r['tolerance_mm'],
                ('%.2fmm past it' % r['outside_mm_before'])
                if r['outside_mm_before'] else 'inside it')
            for r in doc['worse'])
    return doc


def _promote(staged: str, out_path: str, summary: Optional[Dict] = None) -> None:
    """Move a finished staged board and its siblings onto the output path.

    Stage all files and back up prior destinations before replacing anything.
    Roll back completed replacements on failure. If restoration itself fails,
    report the affected paths and retain backups instead of claiming atomicity.
    Destination-only requirements cannot join a board graded without them.
    """
    from copy_board import SIBLING_EXTS
    src_base = os.path.splitext(staged)[0]
    dst_base = os.path.splitext(out_path)[0]
    pairs = [(staged, out_path)]
    extra_requirements = [dst_base + ext for ext in SIBLING_EXTS
                          if ext != '.kicad_prl' and os.path.exists(dst_base + ext)
                          and not os.path.isfile(src_base + ext)]
    if extra_requirements:
        doc = dict(summary or {})
        reason = (
            'output has requirement siblings absent from the graded input: %s. '
            'Use a fresh output path or reconcile these declarations with the input.'
            % ', '.join(extra_requirements))
        doc.update(output=None, refused='; '.join(x for x in (doc.get('refused'), reason) if x))
        raise PoseRefusal(doc['refused'], code=2, summary=doc)
    for ext in SIBLING_EXTS:
        if os.path.isfile(src_base + ext):
            pairs.append((src_base + ext, dst_base + ext))
    staged_tmps = []
    backups = {}
    replaced = []
    recovery_paths = []
    try:
        for _src, dst in pairs:
            if os.path.lexists(dst) and (not os.path.isfile(dst) or os.path.islink(dst)):
                raise OSError('destination is not a regular file: %s' % dst)
        for src, dst in pairs:
            tmp = dst + '.krt-tmp'
            shutil.copyfile(src, tmp)
            staged_tmps.append((tmp, dst))
        for _src, dst in pairs:
            backups[dst] = None
            if os.path.exists(dst):
                fd, backup = tempfile.mkstemp(prefix='.krt-backup-', dir=os.path.dirname(
                    os.path.abspath(dst)))
                os.close(fd)
                backups[dst] = backup
                shutil.copy2(dst, backup)
        # NOT `pop()` before the replace: a failing `os.replace` would then
        # have already removed its own tmp from the cleanup list, and the file
        # it could not move was left beside the output. Remove only on
        # success, so `finally` still owns everything that did not land.
        for entry in list(reversed(staged_tmps)):
            os.replace(entry[0], entry[1])
            replaced.append(entry[1])
            staged_tmps.remove(entry)
    except OSError as exc:
        rollback_errors = []
        for dst in reversed(replaced):
            try:
                backup = backups[dst]
                if backup is None:
                    os.remove(dst)
                else:
                    os.replace(backup, dst)
                    backups[dst] = None
            except OSError as restore_error:
                rollback_errors.append({'path': dst, 'error': str(restore_error),
                                        'backup': backups[dst]})
                if backups[dst]:
                    recovery_paths.append(backups[dst])
        reason = 'cannot write %s: %s. ' % (out_path, exc)
        if rollback_errors:
            reason += ('Output partially changed; restoration failed for %s. '
                       'Retained backup paths are listed in rollback_errors.'
                       % ', '.join(row['path'] for row in rollback_errors))
        else:
            reason += 'Nothing was written: previous output files were preserved or restored.'
        doc = dict(summary or {})
        # The grade this run already did is kept, and any finding it was
        # forced past is kept WITH the write error rather than replaced by it:
        # a summary that reports only the last thing to go wrong is how a
        # waived finding disappears.
        doc['refused'] = '; '.join(x for x in (doc.get('refused'), reason) if x)
        doc['output'] = None
        doc['output_state'] = 'partial' if rollback_errors else 'unchanged'
        doc['rollback_errors'] = rollback_errors
        raise PoseRefusal(reason, code=2, summary=doc)
    finally:
        for backup in backups.values():
            if backup and backup not in recovery_paths:
                try:
                    os.chmod(backup, os.stat(backup).st_mode | stat.S_IWUSR)
                    os.remove(backup)
                except OSError:
                    pass
        for tmp, _dst in staged_tmps:
            try:
                os.remove(tmp)
            except OSError:
                pass


def _refusal_reason(bad, strict, before, after, summary) -> str:
    if bad:
        # A pair list is named by its NEW pairs, not dumped (a pile has
        # dozens of inherited ones).
        parts = ', '.join(
            ('%s new: %s' % (k, ', '.join(new_pairs(before, after, k)))
             if k in PAIR_KEYS else
             '%s %s -> %s' % (k, before.get(k), after.get(k)))
            for k in bad)
        # NO verdict verb in the sentence. It used to end "Refused rather
        # than written", and `--force` reprints this text on a run that WROTE
        # -- so the finding contradicted the outcome in its own last clause.
        # What happened is the caller's line to print; this is the finding.
        reason = ("this pose makes the board's placement legality WORSE (%s); "
                  "the board's inherited violations are not counted against you."
                  % parts)
        if any(k.startswith('pad_stack_') for k in bad):
            reason += (" A pad stack is two parts' pad copper overlapping on "
                       "a shared side, whatever the nets (check_assembly's "
                       "pad_intersection, never waivable); check_assembly "
                       "grades a board with one NOT BUILDABLE.")
    else:
        reason = ("--strict-legal was asked for and the board is not clean at "
                  "this pose (%s)" % ', '.join(
                      '%s %s' % (k, after.get(k)) for k in LEGALITY_KEYS
                      if after.get(k)))
    sc = summary.get('snap_census') or {}
    if 'candidates_tried' in sc and not summary.get('snapped'):
        # #1209: this call SNAPPED and nothing passed. Its own census is the
        # finding; a pose it refused is never offered back, and `--snap` is
        # never advised to a call that already carries it.
        return reason + _snap_census_sentence(sc)
    nl = summary.get('nearest_legal')
    # A ranked pose the escape branch alone admitted is only LESS off the
    # board than the part's input pose (`rank_poses` labels it `legal:
    # False, escape: True`), and the sentence must not read as a legal one.
    off = (", only less off the board than the part's input pose"
           if nl and nl.get('escape') else '')
    if nl and not bad:
        # --strict-legal is a WHOLE-BOARD condition, and this candidate is
        # about ONE part: moving there can leave every other conflict in
        # place, so it is offered as a next step and not as a fix.
        reason += (" The pose ranker's nearest candidate for this part is "
                   "x=%g y=%g rot=%g (%.3f mm away%s), but --strict-legal is "
                   "a condition on the WHOLE board, so that pose need not "
                   "satisfy it."
                   % (nl['x'], nl['y'], nl['rot'], nl.get('dist_mm') or 0.0,
                      off))
    elif nl:
        reason += (" The pose ranker's nearest candidate is x=%g y=%g "
                   "rot=%g (%.3f mm away%s) -- try it with --snap, which "
                   "re-grades before writing."
                   % (nl['x'], nl['y'], nl['rot'], nl.get('dist_mm') or 0.0,
                      off))
    elif summary.get('snap_census', {}).get('dropped_in_place'):
        import pose_score
        reason += (" No legal pose was found nearby, and the census shows the "
                   "knobs veto the part's own spot too -- check the resolved "
                   "clearance against the board's floor before reading this "
                   "as 'the part is stuck'. Vetoed by "
                   + pose_score.veto_phrase(
                       summary['snap_census'].get('dropped_by') or {})
                   + ".")
    return reason


def _snap_census_sentence(sc: Dict) -> str:
    """What a snap that refused EVERY candidate found, for the refusal (#1209).

    The categories and pairs that refused the re-graded candidates, then the
    two levers that could still change the answer -- untried candidates
    (`--snap-tries`) and a ranked pose past the bound (`--radius`) -- each
    named only when it exists.
    """
    tried = sc.get('candidates_tried') or 0
    radius = sc.get('radius_mm') or 0.0
    by = sc.get('regrade_refused_by') or {}
    pairs = sc.get('regrade_refused_pairs') or {}

    def _named(k):
        names = list(pairs.get(k) or ())
        if not names:
            return ''
        more = len(names) - 4
        return ' (%s%s)' % (', '.join(names[:4]),
                            ', +%d more' % more if more > 0 else '')

    # #1277: the on-board rung exists only from an input pose off the board.
    on_board = ('%d ranked on the board + ' % sc['ranked_on_board']
                if sc.get('ranked_on_board') is not None else '')
    if tried:
        out = (" --snap re-graded %d candidate(s) within %g mm (of %s%d ranked "
               "+ %d lattice poses in reach) and refused every one: %s."
               % (tried, radius, on_board, sc.get('ranked') or 0,
                  sc.get('lattice') or 0,
                  ', '.join('%s x%d%s' % (k, n, _named(k))
                            for k, n in list(by.items())[:6])
                  or 'no category recorded'))
    else:
        out = (" --snap had no candidate to re-grade within %g mm (%s%d ranked "
               "+ %d lattice poses in reach)."
               % (radius, on_board, sc.get('ranked') or 0,
                  sc.get('lattice') or 0))
    untried = sc.get('untried') or 0
    if untried:
        out += (" %d pose(s) in reach were NOT re-graded (--snap-tries %s per "
                "rung): raise --snap-tries before reading this as 'nothing "
                "fits'." % (untried, sc.get('snap_tries')))
    nb = sc.get('nearest_beyond_radius')
    if nb:
        reach = math.ceil((nb.get('dist_mm') or 0.0) * 100.0 - 1e-6) / 100.0
        out += (" The pose ranker's nearest pose past the bound is x=%g y=%g "
                "rot=%g (%.3f mm away), not re-graded: --radius %g would let "
                "the snap try it." % (nb['x'], nb['y'], nb['rot'],
                                      nb.get('dist_mm') or 0.0, reach))
    home = sc.get('input_pose_vetoed')
    if home and not sc.get('ranked_before_radius'):
        # #1256: about the part's pose on the INPUT board -- staying put --
        # not the request's spot, which the grade had already refused.
        import pose_score
        out += (" The ranker kept no pose at all, and it vetoes the part's "
                "pose on the input board too (%s%s), where the re-grade "
                "cannot be worse -- its knobs are stricter than the board, "
                "so check the resolved clearance against the board's floor "
                "before reading this as 'the part is stuck'. Vetoed by "
                % (home.get('check'),
                   ' vs %s' % home['blocker'] if home.get('blocker') else '')
                + pose_score.veto_phrase(sc.get('dropped_by') or {}) + ".")
    return out
