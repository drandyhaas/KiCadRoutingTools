#!/usr/bin/env python3
"""What a board looks like to something that has to place it, in one artifact.

The placement toolchain already emits every number an author needs -- escape
lane deficits and who ate them, block displacement, net affinity, part classes,
groups, lock advice, the render checklist and the crop that frames each
finding. What it has never had is one place to READ them from, so an author
(human or model) reassembles the picture from a dozen commands every time, or
does not, and places from a guess.

THIS MODULE ASSEMBLES; IT DOES NOT COMPUTE. Every number comes from the
function that already owns it, and `sources` names that function per section,
so there is one source of truth per fact and the brief cannot drift away from
the graders. The one exception is `fit`, which is opt-in and says so: it
answers "where does a W x H part fit at all", the question
`wk/run19/urchin/probe_space.py` was hand-written to answer and whose output
became that run's X0 = 46.0.

Two rules inherited from the evidence map
(.claude/skills/plan-pcb-routing/references/evidence-map.md):

    never read a picture on its own -- every render is paired with a number
    that either confirms or contradicts it, and the number wins

    keys are literal; if a key is not in the output you are looking at, you
    are looking at the wrong artifact

so `renders` is folded in only as a reference to a real `render_placement
--json-out` document, never re-derived here.

A section that cannot run says WHY, in `skipped`, and is absent from the body.
A brief that silently omitted a measurement would read as "nothing to report".

    python3 board_brief.py board.kicad_pcb
    python3 board_brief.py board.kicad_pcb --json brief.json --fit 15.5x15.5
    python3 board_brief.py board.kicad_pcb --requirements "USB on the north
        edge; the two thumb clusters must stay reachable" --render-json r.json

Exit codes: 0 = a brief was written, 2 = usage/load error, 3 = no outline
(there is nothing to place against).
"""
import _path  # noqa: F401  (py_tools -> py_router/py_placer on sys.path)

import argparse
import json
import math
import os
import sys

SCHEMA = 1
DEFAULT_FIT_STEP_MM = 2.0
WORST_N = 10


def _safe(section, fn, skipped, *a, **kw):
    """Run an emitter; record WHY it did not run rather than dropping it."""
    try:
        return fn(*a, **kw)
    except Exception as e:                       # noqa: BLE001 - reported
        skipped[section] = f"{type(e).__name__}: {e}"
        return None


def _rings(raw):
    """Ring vertices as JSON, at the parser's own precision."""
    return [[[round(float(x), 6), round(float(y), 6)] for x, y in ring]
            for ring in (raw or ()) if ring]


def _shoelace(ring):
    """The tree's own shoelace, with a local fallback.

    `board_section` is the one section `build_brief` calls outside `_safe`
    (its keys are structurally required, down to the JSON_SUMMARY line), so a
    hard dependency on a PRIVATE cross-module name would put the whole
    artifact behind someone else's rename.
    """
    try:
        from kicad_writer import _polygon_area
        return _polygon_area(ring)
    except Exception:                            # noqa: BLE001
        a = 0.0
        for i in range(len(ring)):
            x1, y1 = ring[i][0], ring[i][1]
            x2, y2 = ring[(i + 1) % len(ring)][0], ring[(i + 1) % len(ring)][1]
            a += x1 * y2 - x2 * y1
        return abs(a) / 2.0


def board_section(pcb, pcb_file, skipped):
    from kicad_parser import parse_kicad_pcb  # noqa: F401  (documents origin)
    bi = pcb.board_info
    out = {
        'path': os.path.abspath(pcb_file),
        'bounds': list(bi.board_bounds) if bi.board_bounds else None,
        'copper_layers': list(getattr(bi, 'copper_layers', []) or []),
        'cutouts': len(getattr(bi, 'board_cutouts', []) or []),
        'outlines': len(getattr(bi, 'board_outlines', []) or []),
        # #505: an interior contour that is a milled edge, not a hole. A part
        # whose pads land inside one is unroutable, and it is invisible to a
        # bounding-box view of the board.
        'milled_contours': len(getattr(bi, 'board_edge_contours', []) or []),
        'stackup': [{'name': l.name, 'type': l.layer_type,
                     'thickness': l.thickness}
                    for l in (getattr(bi, 'stackup', []) or [])],
        'footprints': len(pcb.footprints),
        'nets': len(pcb.nets),
    }
    # WHERE, not just how many. These are already-parsed vertex lists and the
    # brief used to call len() on all three, so an author was told a board has
    # two cutouts and never where -- on watchy, two strap slots spanning
    # nearly the full width at the north and south edges, which is exactly
    # where `place_edge north|south` aims. They are also the only source in
    # the brief for `place_keepout`'s required `rect`: the rings ARE the rects.
    out['outline_rings'] = _rings(getattr(bi, 'board_outlines', None))
    out['cutout_rings'] = _rings(getattr(bi, 'board_cutouts', None))
    out['milled_contour_rings'] = _rings(getattr(bi, 'board_edge_contours',
                                                 None))
    out['outline_source'] = 'edge_cuts_contours'
    if out['bounds']:
        x0, y0, x1, y1 = out['bounds']
        out['size_mm'] = [round(x1 - x0, 3), round(y1 - y0, 3)]
        out['area_mm2'] = round((x1 - x0) * (y1 - y0), 2)
        if not out['outline_rings']:
            # `extract_board_contours` returns NOTHING for a simple
            # axis-aligned rectangle -- "Simple rectangle, use bounding box"
            # (kicad_parser.py:2228) -- which is 24 of the 33 corpus boards.
            # Publishing only what it returns meant the rings, and the areas
            # guarded on them, were absent from most briefs with nothing in
            # `skipped` to say why, while this section's stated purpose is to
            # be the source for `place_keepout`'s rect. On those boards the
            # outline IS the bounding box, so it is stated rather than
            # withheld -- and `outline_source` says which it came from.
            out['outline_rings'] = [[[round(x0, 6), round(y0, 6)],
                                     [round(x1, 6), round(y0, 6)],
                                     [round(x1, 6), round(y1, 6)],
                                     [round(x0, 6), round(y1, 6)]]]
            out['outline_source'] = 'bounding_box'
            # ...and the COUNT follows the geometry. A rectangular board has
            # one outline; reporting 0 because the contour extractor
            # short-circuits was the same under-report in scalar form.
            out['outlines'] = 1
    # Real copper, by shoelace. `area_mm2` stays the bounding box so nothing
    # reading it changes meaning; this is the number a cutout can reduce
    # (watchy: 1553.06 bbox against 1434.09 of copper, 8.3% of it not there).
    # DELIBERATELY NOT called `usable_area_mm2`: `options.grow_board` already
    # emits that name for the bbox minus the edge-clearance band, which is
    # cutout-blind and not comparable -- one name, two quantities, across two
    # artifacts an author reads together, is the drift this module exists to
    # prevent.
    if out['outline_rings']:
        o_area = sum(_shoelace(r) for r in out['outline_rings'])
        c_area = sum(_shoelace(r) for r in out['cutout_rings'])
        out['outline_area_mm2'] = round(o_area, 2)
        out['cutout_area_mm2'] = round(c_area, 2)
        out['copper_area_mm2'] = round(max(0.0, o_area - c_area), 2)
    floors = _safe('board.floors', _floors, skipped, pcb_file)
    if floors is not None:
        out['floors'] = floors
    return out


def _floors(pcb_file):
    """The board's own floors, with each value's SOURCE.

    `board_floor_knobs` returns `(clearance, board_edge_clearance, knobs)` --
    a triple, not a mapping. Only `knobs` belongs in the brief: it is the
    part that says whether a number came from the board's netclass, a board
    constraint, or a fixed default, and a reader who cannot tell those apart
    cannot tell a real floor from a guess.
    """
    from list_nets import board_floor_knobs
    return board_floor_knobs(pcb_file)[2]


# Brief keys a PLACEMENT produces and an unplaced board cannot. Each is nulled
# rather than dropped, and named in `position_dependent.invalid`, because the
# shape of the answer is the problem: on a pile these come back in the exact
# schema a placed board yields, so a reader has no way to tell them apart.
# Measured placed -> piled, same brief, no marking anywhere: ulx3s escape
# 19 -> 109 deficit lanes and locks.high 10 -> 0; kit-dev-coldfire 0 -> 82
# lanes on a board with no escape problem at all; splitflap_driver
# locks.high 16 -> 2; watchy 25 -> 69 lanes, locks.high 7 -> 1, tethers 6 -> 4.
#
# Mechanisms, one per group:
#   escape   `_blocked_span` charges every co-located neighbour against the
#            band, so supply -> 0 and deficit == demand for every part but
#            the largest. `deficit_lanes` becomes a netlist count wearing an
#            escape verdict's clothes, and `blockers` names parts to move
#            that are not anywhere.
#   locks    the geometric rules (off_board, near_board_edge,
#            family_orbit_seat) cannot fire at a centre pile, so the
#            `geometric and lexical -> high` promotion is unreachable and the
#            pad-conflict demotion fires on nearly everything.
#   decap    every IC-to-cap distance is 0.0, so the strict `<` in the
#            nearest-IC search is decided by FILE ORDER and every cap on a
#            rail re-parents onto one chip.
#   netprefix  fails the OTHER way: the 20mm spread gate is disarmed at
#            spread 0, so it MANUFACTURES blocks the real board lacks.
POSITION_DEPENDENT = (
    'measurements.escape.deficit_parts',
    'measurements.escape.deficit_lanes',
    'measurements.escape.worst[].worst_face',
    'measurements.escape.worst[].worst_deficit',
    'measurements.escape.worst[].faces[].supply',
    'measurements.escape.worst[].faces[].deficit',
    'measurements.escape.worst[].faces[].blocked_mm',
    'measurements.escape.worst[].faces[].blockers',
    # #700's layer-aware pair. Both derive from `supply`, already listed, so
    # they inherit its position dependence exactly -- and `worst_face_floor`
    # is None on a board with no floor-level deficit, the same way
    # `worst_face` is on a board with no deficit at all.
    'measurements.escape.worst[].worst_face_floor',
    'measurements.escape.worst[].worst_deficit_floor',
    'measurements.escape.worst[].faces[].deficit_floor',
    'measurements.locks.high',
    'measurements.locks.lock_argv',
    'measurements.locks.tally.findings_high',
    'measurements.locks.tally.findings_medium',
    'measurements.locks.tally.findings_low',
    'measurements.locks.tally.findings_covered',
    'measurements.locks.tally.unlocked_high',
    'structure.decap_tethers',
    'structure.groups.decap',
    'structure.groups.netprefix',
)

# Fires the disclosure even when `assess_placement` stops short of `unplaced`.
# Matches `placement_state.DUP_FRACTION`, the same threshold that module uses
# for its own strong signal -- this is that signal WITHOUT the
# distinct-positions conjunct the mechanical exemptions break.
PILE_FRACTION = 0.5

# KEPT, but derived from the part's pose rather than from the netlist -- so
# they are correct for the board AS IT IS and not for the board as it will
# be. Named rather than nulled because each is the best answer available and
# an author needs it: `extent_mm` is what sizes a part, per-face `demand` is
# what an escape budget is read from. What makes them treacherous on a
# staged board is that `stage_unaided` writes ROTATION 0 for every part it
# moves, and all of these follow rotation.
#
# Measured on boards staged by this repo's own stager: 130 of 234 ulx3s parts
# have `extent_mm` with width and height SWAPPED against the placed board
# (BAT1 [20.3, 14.1] at rot 90 -> [14.1, 20.3] at rot 0); watchy 43 of 84,
# tigard 30 of 92, every one an exact swap. And U2's escape demand moves
# north 21 -> 2 / east 1 -> 21 as the part turns, so the per-face table is
# right about a rotation the author has not chosen yet.
POSE_DERIVED_KEPT = (
    ('parts[].rotation',
     'on a staged board this is a generator default, not a decision'),
    ('parts[].extent_mm',
     'follows `rotation`: swap width and height when you rotate the part'),
    ('parts[].rect / parts[].at',
     'the pile coordinate for any part `mechanical.pose_shared_with` lists'),
    ('measurements.escape.worst[].faces[].face / .demand / .span_mm',
     'which face a net leaves through follows rotation, so the table is '
     'right about the CURRENT angle only. Per-part totals are invariant. '
     '#700\'s via_slots / other_layer_lanes / supply_other_max / '
     'supply_bound derive from span_mm alone, so they follow rotation the '
     'same way and are kept for the same reason -- unlike deficit_floor, '
     'which also reads `supply` and is therefore nulled with it'),
    ('measurements.escape.worst[] membership and order',
     'the ledger is sorted by a deficit that is now null and truncated to '
     'the first 10, so on a board with more than 10 fine-pitch parts this '
     'is a pile-selected SET, not just a pile-ordered one'),
    ('renders.*',
     'a render draws the poses in the file, so a render of this board shows '
     'the pile'),
)


def _null_path(node, parts):
    """Set one dotted path to None wherever it EXISTS; `a[]` walks a list.

    Returns whether the path was present, so `position_dependent.invalid`
    lists the paths this board actually had rather than the table's wish
    list. Present-and-already-None counts: otherwise a key that is null for
    its own reasons ships null with nothing in `invalid` explaining it, which
    is the exact silence this block exists to break.
    """
    key, listy = parts[0], parts[0].endswith('[]')
    if listy:
        key = key[:-2]
    if not isinstance(node, dict) or key not in node:
        return False
    if len(parts) == 1:
        node[key] = None
        return True
    child = node[key]
    if listy:
        # A LIST COMPREHENSION, NOT A GENERATOR. `any()` short-circuits on
        # the first True, so a generator would null element 0 and leave
        # 1..N intact while still reporting the path as nulled.
        if isinstance(child, (str, bytes)) or not isinstance(child, (list,
                                                                    tuple)):
            return False        # `a[]` over a non-list: nothing to walk
        return any([_null_path(it, parts[1:]) for it in child])
    return _null_path(child, parts[1:])


def state_section(pcb, pcb_file, skipped):
    from placement.placement_state import assess_placement
    st = assess_placement(pcb, pcb_file)
    return {'unplaced': st.unplaced,
            # WHY, not just the verdict. Dropping these left a JSON reader
            # with a bare boolean and no way to see what fired -- and they
            # are what `position_dependent.because` quotes.
            'reasons': list(st.reasons),
            'signals': dict(st.signals),
            'partially_unplaced': st.partially_unplaced,
            'has_copper': st.has_copper,
            'duplicate_fraction': round(st.duplicate_fraction, 4),
            'spread_ratio': round(st.spread_ratio, 4),
            'outside_fraction': round(st.outside_fraction, 4),
            'stacked_suspect_refs': list(st.stacked_suspect_refs),
            'segments': st.segments, 'vias': st.vias}


def parts_section(pcb, pcb_file, skipped):
    """Per part: what an author needs to decide where it goes.

    Size comes from the courtyard where the footprint has one and the pad
    bounding box where it does not -- and `courtyard` says which, because a
    pad bbox carries NO courtyard margin and treating the two alike is how a
    part gets seated tighter than its own footprint allows (the quench warns
    about this on every board that needs it).
    """
    from placement.legality import rotate_local_bounds
    from placement.parser import extract_courtyard_bboxes
    from placement.utility import compute_footprint_bbox_local
    from placement.part_class import classify_part
    from placement.escape import lane_pitch  # noqa: F401 (documents origin)

    cy = _safe('parts.courtyards', extract_courtyard_bboxes, skipped,
               pcb_file) or {}
    out = {}
    for ref, fp in sorted(pcb.footprints.items()):
        pads = fp.pads or []
        box = cy.get(ref)
        if box:
            src = 'courtyard'
        elif pads:
            # compute_footprint_bbox_local, NOT max(global_x) - min(global_x).
            # The pad-CENTRE span omits the pads' own size, so a two-pad part
            # measures ZERO width along its pad axis: esp_prog C1 (an 0603)
            # reported [0.0, 1.778] where its real extent is 2.794 x 1.016,
            # and the understatement reached 51% of total part area on that
            # board -- a number grow_board's utilisation is computed from.
            box = compute_footprint_bbox_local(fp)
            src = 'pad_bbox'
        else:
            box = None
            src = 'none'
        if box is None:
            w = h = 0.0
            rect = None
        else:
            # extract_courtyard_bboxes and compute_footprint_bbox_local BOTH
            # return the footprint-LOCAL frame; the parser's own docstring
            # says so ("must be transformed to global coordinates using the
            # footprint's position and rotation"). Reporting them raw
            # transposes every part on a 90/270 rotation -- 133 of 266 on
            # glasgow_revC -- so `extent_mm` could not be fed to --fit WxH,
            # whose `fit` section IS board-frame.
            gx0, gy0, gx1, gy1 = rotate_local_bounds(*box, fp.rotation or 0.0)
            w, h = gx1 - gx0, gy1 - gy0
            # The same rect `legality.graded_parts_from_file` publishes
            # (`rect = (fp.x + lx0, fp.y + ly0, ...)`). The rotation is
            # applied one line up and the translation was thrown away, so
            # this brief emitted `rotation` and no position while standing
            # one addition away from the absolute pose.
            rect = [round(fp.x + gx0, 6), round(fp.y + gy0, 6),
                    round(fp.x + gx1, 6), round(fp.y + gy1, 6)]
        cls = classify_part(fp, ref)
        row = {'extent_mm': [round(w, 3), round(h, 3)],
               'extent_source': src,
               'pads': len(pads),
               'side': getattr(fp, 'layer', None),
               # WHERE THE PART IS, which this table has never carried. Six
               # of the thirteen plan ops take a literal board coordinate
               # (place_keepout.rect, place_fixed.at, place_at.at,
               # place_array.origin, place_slots.slots, place_pack.zone), and
               # `place_fixed` -- the op whose whole job is asserting a pose
               # the mechanical drawing already fixed -- REQUIRES an `at` no
               # section of this brief could supply. Reported as a fact: on
               # an unplaced board it is the pile coordinate, which `state`
               # already says is meaningless, and the two must be read
               # together.
               'at': [round(fp.x, 6), round(fp.y, 6)],
               'rect': rect,
               'rotation': getattr(fp, 'rotation', None),
               'locked': bool(getattr(fp, 'locked', False)),
               'dnp': bool(getattr(fp, 'dnp', False)),
               'footprint': getattr(fp, 'footprint_name', None),
               'value': getattr(fp, 'value', None),
               'nets': sorted({p.net_name for p in pads
                               if p.net_id and p.net_name})}
        if cls is not None and getattr(cls, 'name', None):
            row['class'] = cls.name
            row['class_confidence'] = getattr(cls, 'confidence', None)
            row['class_evidence'] = list(getattr(cls, 'evidence', ()) or ())
        out[ref] = row
    return out


def mechanical_section(pcb, pcb_file, skipped):
    """Which parts' positions are a mechanical fact, on what evidence.

    INFERENCE, NEVER AUTHORITY -- `--requirements` is the channel for fact,
    carried verbatim, because the constraints that fix a connector or a
    mounting hole live in an enclosure drawing this tool cannot see. What
    this section adds is the evidence an author would otherwise have to
    reconstruct: the class, its confidence, what the classifier saw, and
    whether the board's own author already pinned the part.

    It exists because the brief's only other channel for the same question --
    `measurements.locks.high` -- is GEOMETRIC and collapses on exactly the
    input an unaided author has. Measured placed -> piled: watchy 7 -> 1,
    ulx3s 10 -> 0, splitflap_driver 16 -> 2. `part_class.classify_part` reads
    footprint name, reference prefix and pin function and never a coordinate,
    so it answers the same on both.

    `pose_shared_with` is the per-part honesty check, and it is per PART
    rather than a whole-board verdict for the reason `plan_resolve` gives:
    a part alone at its coordinate is where someone put it, whatever state
    the rest of the board is in. A staged unaided board carries the real
    poses of exactly these parts, so an empty list here is the difference
    between `place_fixed` having a coordinate to assert and not.
    """
    from placement.part_class import mechanical_parts
    found = mechanical_parts(pcb)
    at_coord = {}
    for ref, fp in (pcb.footprints or {}).items():
        at_coord.setdefault((round(fp.x, 4), round(fp.y, 4)), []).append(ref)
    out = {}
    for ref, rec in found.items():
        fp = pcb.footprints.get(ref)
        shared = []
        if fp is not None:
            shared = sorted(r for r in at_coord.get(
                (round(fp.x, 4), round(fp.y, 4)), ()) if r != ref)
        row = dict(rec)
        row['pose_shared_with'] = shared
        out[ref] = row
    return {'refs': out,
            'note': 'INFERENCE from part class, not a mechanical drawing. '
                    'These are the parts whose position a real new board '
                    'would already know; `at` is [x, y, rotation] AS THE FILE '
                    'HAS IT, so read `pose_shared_with` before treating one '
                    'as a fact to assert with place_fixed. State real '
                    'mechanical constraints with --requirements.'}


def structure_section(pcb, pcb_file, skipped):
    """The electrical structure a floorplan thinks in."""
    from placement.groups import AUTO_SOURCES, SOURCES, derive_groups, \
        decap_tethers
    out = {}
    by_source = {}
    for src in SOURCES:
        g = _safe(f'structure.groups.{src}', derive_groups, skipped, pcb,
                  (src,))
        if g:
            by_source[src] = {k: sorted(v) for k, v in sorted(g.items())}
    out['groups'] = by_source
    out['groups_auto_sources'] = list(AUTO_SOURCES)
    teth = _safe('structure.decap_tethers', decap_tethers, skipped, pcb)
    if teth is not None:
        out['decap_tethers'] = {a: [r for r, _d in sorted(v)]
                                for a, v in sorted(teth.items())}

    from list_nets import (find_differential_pairs, find_high_connection_nets,
                           find_power_nets)
    dp = _safe('structure.diff_pairs', find_differential_pairs, skipped, pcb)
    if dp is not None:
        out['diff_pairs'] = _jsonable(dp)
    pw = _safe('structure.power_nets', find_power_nets, skipped, pcb)
    if pw is not None:
        gnd, vcc, cand = pw
        out['power_nets'] = {'gnd': _jsonable(gnd), 'vcc': _jsonable(vcc),
                             'candidates': _jsonable(cand)}
    hi = _safe('structure.top_nets', find_high_connection_nets, skipped, pcb,
               top_n=15)
    if hi is not None:
        out['top_nets'] = _jsonable(hi)
    return out


def _jsonable(v):
    """Emitters return tuples/dataclasses; a brief has to be JSON."""
    if isinstance(v, dict):
        return {str(k): _jsonable(x) for k, x in v.items()}
    if isinstance(v, (list, tuple, set)):
        return [_jsonable(x) for x in (sorted(v) if isinstance(v, set) else v)]
    if hasattr(v, 'to_dict'):
        return _jsonable(v.to_dict())
    if hasattr(v, '_asdict'):
        return _jsonable(v._asdict())
    if isinstance(v, (int, float, str, bool)) or v is None:
        return v
    return str(v)


def measurements_section(pcb, pcb_file, skipped, clearance, edge):
    """The numbers that say a placement will fight the router.

    `escape` is the one that is a BINDING constraint rather than a
    preference: a face short of lanes cannot pass its own nets at any net
    ordering, and `blockers` names which neighbour ate the span -- that field
    is the difference between a signal and an action.
    """
    out = {}
    from placement.escape import escape_ledger
    led = _safe('measurements.escape', escape_ledger, skipped, pcb,
                pcb_file=pcb_file, clearance=clearance)
    if led is not None:
        # `PartEscape` exposes `worst` (a FaceLedger or None) and NO
        # `worst_deficit` attribute -- that name exists only as a key in its
        # to_dict(). A getattr(..., 'worst_deficit', 0) therefore always
        # returns the default, silently reporting every board as having no
        # face in deficit.
        from placement.options import deficit_totals
        tot = deficit_totals(led)
        out['escape'] = {'parts': len(led),
                         'worst': [_jsonable(p) for p in led[:WORST_N]],
                         'deficit_parts': tot['parts'],
                         'deficit_lanes': tot['lanes']}

    from placement.lock_advisor import advise_locks, to_json as lock_json
    adv = _safe('measurements.locks', advise_locks, skipped, pcb,
                pcb_file=pcb_file, conflict_clearance=clearance)
    if adv is not None:
        doc = lock_json(adv)
        # The advisor's own tally() keys, named explicitly. A
        # startswith('findings') filter also matches `findings` itself and
        # embeds the whole per-part findings list -- reasons, evidence and
        # all -- under a key called "tally".
        out['locks'] = {
            'tally': {k: doc[k] for k in
                      ('findings_high', 'findings_medium', 'findings_low',
                       'findings_covered', 'unlocked_high', 'locked_in_file',
                       'advisories') if k in doc},
            'lock_argv': doc.get('lock_argv', []),
            'high': [f['ref'] for f in doc.get('findings', [])
                     if f.get('confidence') == 'high'],
        }
    return out


def fit_section(pcb, pcb_file, extents, step, clearance, edge, skipped):
    """Where does a W x H part fit at all?

    Opt-in and COMPUTED here rather than assembled, which is why it is the
    one section that names its own method. It is the question
    `wk/run19/urchin/probe_space.py` was hand-written to answer, and whose
    answer became that run's X0 = 46.0 and "U1 at (28, 60)" -- authored as
    constants because nothing in the toolchain would say it.

    Reports the legal-centre BOUNDS and count per extent, not a bitmap: an
    author needs to know whether a 15.5mm part has anywhere to go and roughly
    where, and a 60x40 grid of booleans in a JSON brief is unreadable.
    """
    from placement.legality import BoardOutlineGate
    bi = pcb.board_info
    if bi.board_bounds is None:
        skipped['fit'] = 'the board has no outline to fit against'
        return None
    gate = BoardOutlineGate(bi, edge)
    x0, y0, x1, y1 = bi.board_bounds
    out = []
    for (w, h) in extents:
        hw, hh = w / 2.0, h / 2.0
        pts = []
        nx = max(1, int((x1 - x0) / step))
        ny = max(1, int((y1 - y0) / step))
        for i in range(nx + 1):
            for j in range(ny + 1):
                cx = round(x0 + i * step, 3)
                cy = round(y0 + j * step, 3)
                if gate.rect_outside_amount(
                        (cx - hw, cy - hh, cx + hw, cy + hh)) <= 1e-9:
                    pts.append((cx, cy))
        row = {'extent_mm': [w, h], 'step_mm': step, 'legal_cells': len(pts),
               'fits': bool(pts)}
        if pts:
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            row['centre_bounds'] = [min(xs), min(ys), max(xs), max(ys)]
        out.append(row)
    return out


# Each entry names the function that owns the section's numbers AND what the
# section requires of the input. The second half is not decoration: a reader
# who cannot tell "needs a placement" from "needs only the netlist" cannot
# tell which half of a brief for an unplaced board to believe -- and every
# section here used to read alike.
SOURCES_NOTE = {
    'board': 'kicad_parser.parse_kicad_pcb + list_nets.board_floor_knobs + '
             'board_brief._shoelace '
             '[requires: an outline. Independent of part positions]',
    'state': 'placement.placement_state.assess_placement '
             '[requires: nothing. This section IS the placement verdict '
             'the others are read against]',
    'parts': 'placement.parser.extract_courtyard_bboxes + '
             'placement.utility.compute_footprint_bbox_local + '
             'placement.legality.rotate_local_bounds + '
             'placement.part_class.classify_part '
             '[requires: nothing for extent/class/nets; `at` and `rect` are '
             'reported from the file, so on an unplaced board they are the '
             'pile coordinate]',
    'mechanical': 'placement.part_class.mechanical_parts '
                  '(classify_part; pose-independent, so it survives an '
                  'unplaced board where measurements.locks does not) '
                  '[requires: nothing. Read `pose_shared_with` before '
                  'trusting an `at`]',
    'structure': 'placement.groups.derive_groups / decap_tethers + '
                 'list_nets.find_differential_pairs / find_power_nets / '
                 'find_high_connection_nets '
                 '[requires: a PLACEMENT for groups.decap, groups.netprefix '
                 'and decap_tethers, which are distance-derived; nothing for '
                 'groups.sheet/kicad, diff_pairs, power_nets, top_nets]',
    'measurements': 'placement.escape.escape_ledger + '
                    'placement.lock_advisor.advise_locks '
                    '[requires: a PLACEMENT. escape supply and lock '
                    'confidence are both measured between parts, so on an '
                    'unplaced board most of this section is null -- see '
                    'position_dependent]',
    'fit': 'placement.legality.BoardOutlineGate.rect_outside_amount '
           '(COMPUTED here, not assembled) '
           '[requires: an outline only. It never reads another part, so it '
           'is valid on an unplaced board]',
    'renders': 'py_tools/render_placement.py --json-out (folded in verbatim) '
               '[requires: whatever the render was made from -- it draws the '
               'poses in the file]',
}


def build_brief(pcb, pcb_file, *, clearance=None, board_edge_clearance=None,
                requirements=None, fit=(), fit_step=DEFAULT_FIT_STEP_MM,
                render_doc=None):
    skipped = {}
    # Board-first: an unset knob resolves from this board's own Default
    # netclass / min_copper_edge_clearance, and the returned `knobs` records
    # which source each came from. `board_floor_knobs` returns a TRIPLE.
    try:
        from list_nets import board_floor_knobs
        clr, edge, _knobs = board_floor_knobs(
            pcb_file, clearance=clearance,
            board_edge_clearance=board_edge_clearance)
    except Exception as e:                       # noqa: BLE001 - disclosed
        clr = 0.25 if clearance is None else clearance
        edge = 0.55 if board_edge_clearance is None else board_edge_clearance
        skipped['floors'] = f"{type(e).__name__}: {e}"

    brief = {'schema': SCHEMA, 'kind': 'board-brief',
             'clearance': clr, 'board_edge_clearance': edge}
    brief['board'] = board_section(pcb, pcb_file, skipped)
    brief['state'] = _safe('state', state_section, skipped, pcb, pcb_file,
                           skipped)
    brief['parts'] = _safe('parts', parts_section, skipped, pcb, pcb_file,
                           skipped)
    brief['mechanical'] = _safe('mechanical', mechanical_section, skipped, pcb,
                                pcb_file, skipped)
    brief['structure'] = _safe('structure', structure_section, skipped, pcb,
                               pcb_file, skipped)
    brief['measurements'] = _safe('measurements', measurements_section,
                                  skipped, pcb, pcb_file, skipped, clr, edge)
    if fit:
        f = _safe('fit', fit_section, skipped, pcb, pcb_file, fit, fit_step,
                  clr, edge, skipped)
        if f is not None:
            brief['fit'] = f
    if render_doc is not None:
        # Verbatim, and only the keys the evidence map names -- a brief that
        # paraphrased a checklist would be a second source of truth for the
        # same finding.
        brief['renders'] = {
            'instrument': render_doc.get('instrument'),
            'checklist': render_doc.get('checklist'),
            'panels': render_doc.get('panels'),
            'describe': render_doc.get('describe'),
        }
    if requirements:
        # Verbatim. These are the constraints that live nowhere in the board
        # file -- enclosure fit, connector positions, thermal and EMI zoning,
        # datasheet layout intent -- and paraphrasing them here would be this
        # tool inventing mechanical geometry.
        brief['requirements'] = requirements
    # The board already told us it is a pile; act on it. This runs AFTER
    # every section so there is one table of position-dependent paths rather
    # than a scattering of `if unplaced` branches, and so `invalid` is
    # derived from what was actually nulled instead of restated by hand.
    st = brief.get('state') or {}
    # NOT `st['unplaced']` alone. `assess_placement`'s verdict needs BOTH a
    # high duplicate fraction AND `distinct_positions <= max(3, 0.1n)`, and
    # this repo's own stager breaks the second conjunct with the very
    # exemptions that make an unaided board usable: `stage_unaided` on
    # splitflap_driver leaves 58 of 65 parts on one coordinate but 8 distinct
    # positions (7 mechanical + the pile), so `unplaced` is False,
    # `partially_unplaced` is True -- and this whole disclosure did not fire
    # on the tool's own output. The brief's question is narrower than the
    # stager's: "are these coordinates meaningful", and half the parts
    # sharing one is enough to answer no. A netlist re-import dropping three
    # new parts on a placed board is 3/65 = 0.046 and stays unmarked, which
    # is the case `partially_unplaced` exists for.
    dup = st.get('duplicate_fraction') or 0.0
    piled = bool(st.get('unplaced')) or dup >= PILE_FRACTION
    if piled:
        nulled = [p for p in POSITION_DEPENDENT
                  if _null_path(brief, p.split('.'))]
        brief['position_dependent'] = {
            'invalid': nulled,
            'because': list(st.get('reasons') or ())
            or [f"{dup:.0%} of footprints share a position with another"],
            'also_pose_derived': [{'key': k, 'caveat': why}
                                  for k, why in POSE_DERIVED_KEPT],
            'note': (
                'Keys in `invalid` are null because they are MEASURED FROM '
                'PART POSITIONS and this board has none yet -- not because '
                'they could not be computed. Keys in `also_pose_derived` are '
                'KEPT: each is the best answer available and an author needs '
                'it, but every one follows the part\'s pose, and a staged '
                'board carries rotation 0 for everything it moved. '
                'Everything named in NEITHER list is a netlist, footprint or '
                'outline fact and holds. `fit` is unaffected -- it probes the '
                'outline and never another part, so on this board it is the '
                'section that still answers "where could this go".'),
            'gate': ('state.unplaced' if st.get('unplaced')
                     else f'duplicate_fraction {dup:.3f} >= {PILE_FRACTION}'),
            'not_applied_to': (
                'a board where only a few refs share a coordinate -- a '
                'netlist re-import dropping new parts at one spot leaves the '
                'rest of the numbers meaningful, and state.'
                'stacked_suspect_refs names the affected refs'),
        }
    brief['sources'] = SOURCES_NOTE
    brief['skipped'] = skipped
    return brief


def format_text(b):
    L = []
    bd = b.get('board') or {}
    L.append(f"BOARD {os.path.basename(bd.get('path', '?'))}: "
             f"{bd.get('footprints')} parts, {bd.get('nets')} nets, "
             f"{len(bd.get('copper_layers') or [])} copper layers")
    if bd.get('size_mm'):
        L.append(f"  outline {bd['size_mm'][0]} x {bd['size_mm'][1]} mm "
                 f"({bd.get('area_mm2')} mm2), cutouts {bd.get('cutouts')}, "
                 f"milled contours {bd.get('milled_contours')}")
        if bd.get('cutout_area_mm2'):
            L.append(f"    copper {bd.get('copper_area_mm2')} mm2 -- "
                     f"{bd['cutout_area_mm2']} mm2 of the outline is cut out")
    st = b.get('state') or {}
    if st:
        what = ('UNPLACED' if st.get('unplaced') else
                'partially unplaced' if st.get('partially_unplaced')
                else 'placed')
        copper = 'yes' if st.get('has_copper') else 'no'
        L.append(f"  state: {what}; copper {copper} "
                 f"({st.get('segments')} segs, {st.get('vias')} vias)")
        for r in (st.get('reasons') or ())[:3]:
            L.append(f"    - {r}")
    # BEFORE the numbers, not after them and not only in the JSON. This
    # printed `state: UNPLACED` and then, eleven lines later, an escape
    # deficit and a lock tally measured on the pile, in the same words a
    # placed board gets.
    pd = b.get('position_dependent') or {}
    if pd.get('invalid'):
        L.append(f"  !! {len(pd['invalid'])} measurement(s) below are NULL: "
                 f"they are measured from part positions and this board has "
                 f"none yet")
        L.append("     every part's input coordinate is meaningless, so an "
                 "op that does not state a target has nothing to work from. "
                 "State mechanical")
        L.append("     poses with place_fixed and the rest with "
                 "place_at/place_pack. `--fit WxH` still answers where a "
                 "part can go at all.")
    mech = ((b.get('mechanical') or {}).get('refs')) or {}
    if mech:
        _fixed = sorted(r for r, v in mech.items()
                        if not v.get('pose_shared_with'))
        L.append(f"  mechanical: {len(mech)} part(s) whose position is a "
                 f"mechanical fact {sorted(mech)[:6]}"
                 f"{'...' if len(mech) > 6 else ''}")
        L.append(f"    {len(_fixed)} of them sit alone at their coordinate, "
                 f"so `at` is assertable with place_fixed: {_fixed[:6]}"
                 f"{'...' if len(_fixed) > 6 else ''}")
    stx = b.get('structure') or {}
    for src, groups in (stx.get('groups') or {}).items():
        # A nulled source is named by position_dependent, not counted here --
        # `len(None)` is how the caveat first shipped.
        if groups is None:
            L.append(f"  groups[{src}]: null (needs a placement)")
            continue
        L.append(f"  groups[{src}]: {len(groups)} block(s) "
                 f"{sorted(groups)[:4]}{'...' if len(groups) > 4 else ''}")
    pw = (stx.get('power_nets') or {})
    if pw:
        L.append(f"  power: gnd {pw.get('gnd')}, vcc {pw.get('vcc')}")
    if stx.get('diff_pairs'):
        L.append(f"  diff pairs: {len(stx['diff_pairs'])}")
    ms = b.get('measurements') or {}
    esc = ms.get('escape') or {}
    if esc:
        _dp = esc.get('deficit_parts')
        L.append(f"  escape: {esc.get('parts')} fine-pitch part(s), "
                 + (f"{_dp} in deficit" if _dp is not None
                    else "deficit NOT MEASURED (needs a placement)"))
        for row in (esc.get('worst') or [])[:3]:
            if row.get('worst_deficit') is None:
                continue
            L.append(f"    {row.get('ref')}: worst face "
                     f"{row.get('worst_face')} deficit "
                     f"{row.get('worst_deficit')}")
    lk = ms.get('locks') or {}
    if lk:
        t = lk.get('tally') or {}
        _uh = t.get('unlocked_high')
        L.append(f"  locks: "
                 + (f"{_uh} high-confidence part(s) NOT locked" if _uh
                    is not None else
                    "confidence NOT MEASURED (needs a placement) -- see the "
                    "`mechanical` section, which does not")
                 + f"; {t.get('locked_in_file', 0)} already locked in the "
                   f"file")
        if lk.get('lock_argv'):
            L.append(f"    {' '.join(lk['lock_argv'])}")
    for row in (b.get('fit') or []):
        w, h = row['extent_mm']
        if row['fits']:
            cb = row['centre_bounds']
            L.append(f"  fit {w}x{h}mm: {row['legal_cells']} legal centre(s) "
                     f"at {row['step_mm']}mm, x {cb[0]}..{cb[2]}, "
                     f"y {cb[1]}..{cb[3]}")
        else:
            L.append(f"  fit {w}x{h}mm: NOWHERE on this board at "
                     f"{row['step_mm']}mm")
    if b.get('requirements'):
        L.append(f"  requirements (verbatim): {b['requirements']}")
    if b.get('skipped'):
        L.append(f"  {len(b['skipped'])} section(s) did not run:")
        for k, why in sorted(b['skipped'].items()):
            L.append(f"    - {k}: {why}")
    return "\n".join(L)


def _parse_fit(values):
    out = []
    for v in values or ():
        try:
            w, h = v.lower().replace('*', 'x').split('x')
            out.append((float(w), float(h)))
        except ValueError:
            raise SystemExit(f"board_brief: --fit wants WxH in mm, got {v!r}")
    return out


def main(argv=None):
    p = argparse.ArgumentParser(
        description="Assemble what a placement author needs to read.",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("board")
    p.add_argument("--json", metavar="PATH", help="Write the full brief here")
    p.add_argument("--clearance", type=float, default=None)
    p.add_argument("--board-edge-clearance", type=float, default=None)
    p.add_argument("--requirements", default=None,
                   help="The constraints that live nowhere in the board file "
                        "(enclosure, connector edges, thermal, EMI). Carried "
                        "VERBATIM")
    p.add_argument("--requirements-file", default=None)
    p.add_argument("--fit", action="append", metavar="WxH",
                   help="Also report where a part of this extent fits at all "
                        "(repeatable)")
    p.add_argument("--fit-step", type=float, default=DEFAULT_FIT_STEP_MM,
                   metavar="MM")
    p.add_argument("--render-json", default=None, metavar="PATH",
                   help="A render_placement --json-out document to fold in")
    p.add_argument("-q", "--quiet", action="store_true")
    a = p.parse_args(argv)

    from kicad_parser import parse_kicad_pcb
    try:
        pcb = parse_kicad_pcb(a.board)
    except Exception as e:                       # noqa: BLE001
        print(f"board_brief: cannot read {a.board}: {e}", file=sys.stderr)
        return 2
    if pcb.board_info.board_bounds is None:
        print("board_brief: this board has no Edge.Cuts outline, so there is "
              "nothing to place against. The outline is spec-owned.",
              file=sys.stderr)
        return 3

    req = a.requirements
    if a.requirements_file:
        try:
            with open(a.requirements_file, encoding='utf-8') as f:
                req = f.read().strip()
        except OSError as e:
            print(f"board_brief: cannot read requirements: {e}",
                  file=sys.stderr)
            return 2

    render_doc = None
    if a.render_json:
        try:
            with open(a.render_json, encoding='utf-8') as f:
                render_doc = json.load(f)
        except (OSError, ValueError) as e:
            print(f"board_brief: cannot read the render doc: {e}",
                  file=sys.stderr)
            return 2

    brief = build_brief(pcb, a.board, clearance=a.clearance,
                        board_edge_clearance=a.board_edge_clearance,
                        requirements=req, fit=_parse_fit(a.fit),
                        fit_step=a.fit_step, render_doc=render_doc)
    if not a.quiet:
        print(format_text(brief))
    if a.json:
        with open(a.json, 'w', encoding='utf-8') as f:
            json.dump(brief, f, indent=1, sort_keys=True, default=str)
        print(f"Wrote {a.json}")
    print("JSON_SUMMARY: " + json.dumps(
        {'board': a.board, 'parts': brief['board'].get('footprints'),
         'nets': brief['board'].get('nets'),
         'unplaced': (brief.get('state') or {}).get('unplaced'),
         'sections': sorted(k for k, v in brief.items()
                            if isinstance(v, (dict, list))
                            and k not in ('sources', 'skipped')),
         'skipped': sorted(brief.get('skipped') or {})}, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
