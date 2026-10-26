#!/usr/bin/env python3
"""#891: the component-context sheet -- what a model needs to reason about a
layout, printed once, for a reader.

WHY THIS EXISTS
---------------
The model is the only thing in the chain that can reason about a layout: which
pin row faces which connector, where a regulator's input and output sit
relative to their sources, whether a pair's pin order crosses. It cannot do
that from what the tools emit. Nothing printed, per part, the package and body
size, each pad with its net and pin function, which other parts each net
reaches, and which SIDE of the part those partners are on.

Run 25 paid for it exactly once and completely. USB1 pad 2 = /D_N is north of
pad 3 = /D_P, while U1 pad 6 = /D_P is north of pad 7 = /D_N: the pair is
polarity-crossed at every rotation of U1, because parity cannot be flipped by
rotation -- only by a mirror or a via hop. That single fact decided the board's
only open clause. It was visible in a throwaway probe the teammate wrote by
hand, read by nobody, and rediscovered after routing.

WHAT IT IS NOT
--------------
It is an ASSEMBLER, not a new engine. Every number here is computed by the
function that already owns it, and the ones that matter say so on the sheet:

    body extent + source   placement.body            (#896's one model)
    pads by board face     placement.escape.assign_faces  (#850's one rule)
    pin-order agreement    placement.pair_order.pair_metrics
    pairs through series   diff_xnet.find_ac_coupled_xnets (#1251)
    part class             placement.part_class.classify_part
    decap partner          placement.groups (_elect_tethers / decap_populations)
    emc roles + legs       placement.emc_roles (#1254)
    diff pairs             list_nets.find_differential_pairs (resonator-safe)
    sheets, drawn blocks   placement.schematic (the project's .kicad_sch)
    net classes            list_nets.net_class_memberships
    floors                 list_nets.board_floor_knobs

Two rules it obeys, both from #711's design brief:

* **Every derived fact carries its source.** A role read off a footprint name
  and a role declared in a brief are different claims and are printed as such.
* **"unknown" is a first-class answer.** Looked-and-could-not-tell is printed
  apart from nobody-looked, and neither is ever guessed.

THE ROLE COLUMN, SCOPED HONESTLY
--------------------------------
The parser reads a footprint's `Value` on both paths, but there is NO
`Datasheet` field, no `Description`, and no `(model ...)` path anywhere in
`PCBData`. So a role here cannot cite a datasheet, and this tool does not
pretend to: it infers from what the board actually carries -- footprint name,
reference prefix, `pinfunction`/`pintype`, copper pad count and geometry, net
names -- and prints `unknown` where that runs out. The datasheet pass stays
where it already lives, in the skill layer (`/analyze-power-nets`,
`/identify-diff-pairs`), and a `--brief` declaration outranks any inference and
is flagged when the two disagree.
"""
from __future__ import annotations

#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['placement', 'combined'], 'kind': 'instrument'}

import _path  # noqa: F401
import argparse
import contextlib
import json
import os
import sys
from typing import Dict, List

SCHEMA = 1


# ---------------------------------------------------------------------------
# Roles. The table and `infer_role` live in `placement.roles` (#1249): the
# decap owner predicate (`placement.groups`) asks whether a part is a
# regulator, and the placement engine must not import py_tools -- so the one
# table lives where both can read it. `infer_role` is re-exported here for
# this tool's callers.
# ---------------------------------------------------------------------------

from placement.roles import infer_role  # noqa: E402,F401


# ---------------------------------------------------------------------------
# Faces
# ---------------------------------------------------------------------------

def pads_by_face(pcb_data, clearance: float, track_width: float):
    """`{ref: {face: [pad, ...]}}` at each part's CURRENT pose.

    Calls `escape.assign_faces`, the one face rule (#850), rather than
    re-deriving one: three copies of that loop existed and each PAIRED the pad
    box and the part box differently, which on one corpus board made all 244
    pads read interior. `interior` collects the pads that escape through no
    face -- they need a via, not a lane.
    """
    from placement.escape import assign_faces, board_copper_geometry
    from placement.escape import _part_rect        # noqa: PLC2701
    try:
        obstruction = board_copper_geometry(pcb_data, clearance)
    except Exception:                                        # noqa: BLE001
        obstruction = {}
    lane = track_width + clearance
    out: Dict[str, Dict[str, List]] = {}
    for ref, fp in (pcb_data.footprints or {}).items():
        own = obstruction.get(ref)
        if own is None and not (fp.pads or ()):
            # A pad-less block (a logo, a panel tab): `_part_rect` is a
            # min/max over pad centres and raises on an empty set. It has no
            # face to escape through either, so skipping is the answer rather
            # than a fabricated box.
            continue
        rect = own.rect if own is not None else _part_rect(fp)
        try:
            asg = assign_faces(fp, own, lane_mm=lane, fallback_rect=rect,
                               clearance=clearance, track_width=track_width)
        except Exception:                                    # noqa: BLE001
            continue
        by: Dict[str, List] = {}
        for pad, face in asg.faces:
            by.setdefault(face or 'interior', []).append(pad)
        out[ref] = by
    return out


# ---------------------------------------------------------------------------
# The sheet
# ---------------------------------------------------------------------------

_LETTER = {'west': 'W', 'north': 'N', 'east': 'E', 'south': 'S'}
_OPPOSITE = {'W': 'E', 'E': 'W', 'N': 'S', 'S': 'N'}


def mating_faces(pcb_data, pcb_file: str, clearance: float):
    """`{ref: {edge, dist_mm, interior, overhang_mm}}` for connector-family
    parts.

    The edge and distance come from `render_placement.connector_edge_facts`,
    which is the list the review sheet already prints -- deliberately broader
    than `part_class`'s gating classes, because run 23 seated four generic
    connectors mid-board and no instrument said so.

    `overhang_mm` is measured here because that function CLAMPS its distance at
    zero (`max(0.0, dist)`), which is right for "how far from the edge" and
    loses the sign for "how far PAST it" -- and a receptacle's overhang is the
    fact a mating face is about. USB1 on esp_prog overhangs; a header does not.
    """
    import render_placement as RP
    from placement import mating
    from placement.body import board_bodies
    from placement.legality import rotate_local_bounds
    from placement.part_class import INTERIOR_AFFINITY_MM
    model = RP.PlacementModel(pcb_data, pcb_file,
                              quench_kwargs={'clearance': clearance})
    if model.state is None:
        return {}
    bounds = getattr(pcb_data.board_info, 'board_bounds', None)
    if not bounds:
        return {}
    bodies = board_bodies(pcb_data, pcb_file)
    out = {}
    # WHICH parts are connector-family is `connector_edge_facts`' question and
    # it keeps it -- a heuristic broader than part_class's gating classes,
    # because run 23 seated four generic connectors mid-board and nothing said
    # so. WHERE the part's edge is, this measures from the BODY model rather
    # than reusing that function's number, because it reads `model.rect` --
    # the placement state's pad-box ladder, which this PR deliberately does not
    # convert. Reusing it would put two different geometries on one sheet,
    # which is what #896 exists to stop: measured on esp_prog, USB1's pad box
    # sits 0.49mm inside the west edge while its drawn .Fab body overhangs it,
    # and the overhang is the fact a mating face is about.
    for ref, _cls, _edge, _dist, _interior in RP.connector_edge_facts(model):
        geom = bodies.get(ref)
        fp = pcb_data.footprints.get(ref)
        if geom is None or geom.occupancy_local is None or fp is None:
            continue
        x0, y0, x1, y1 = rotate_local_bounds(*geom.occupancy_local,
                                             fp.rotation or 0.0)
        rect = (fp.x + x0, fp.y + y0, fp.x + x1, fp.y + y1)
        dists = {'W': rect[0] - bounds[0], 'N': rect[1] - bounds[1],
                 'E': bounds[2] - rect[2], 'S': bounds[3] - rect[3]}
        near, dist = min(dists.items(), key=lambda kv: kv[1])
        # #1155: the MATING face is the way the body leaves its pad row
        # (`placement.mating`), which #891 asked this sheet for; the nearest
        # edge stands in only when the body names no face. They differ on a
        # connector seated backwards -- sonde_xilinx J2's shell over the
        # board read "Mating face E" because its back grazed the east edge.
        lf = mating.mating_face_local(fp, geom)
        facing = mating.facing_edge(lf.face, fp.rotation or 0.0)
        edge = _LETTER.get(facing, near)
        out[ref] = {'edge': edge,
                    'edge_basis': 'facing' if facing else 'nearest',
                    'dist_mm': round(max(0.0, dists[edge]), 3),
                    'interior': bool(dist > INTERIOR_AFFINITY_MM),
                    'overhang_mm': round(max(0.0, -dists[edge]), 3),
                    'nearest_edge': near,
                    'nearest_dist_mm': round(max(0.0, dist), 3),
                    'nearest_overhang_mm': round(max(0.0, -dist), 3),
                    'facing_basis': lf.basis,
                    # #1155: strong only with an edge it points at, as
                    # `floorplan.edge_facing` reports it.
                    'facing_strong': bool(lf.strong and facing),
                    'facing_reason': lf.reason or (
                        '' if facing else
                        'the pose is more than 30 degrees off a quarter '
                        'turn, so the face names no edge'),
                    'class': _cls, 'basis': geom.source}
    return out


def serves_map(pcb_data) -> dict:
    """`{cap_ref: (ic_ref, distance_mm)}` -- what each passive SERVES.

    The decap tether election RESTRICTED to the module's own radius
    (`groups.DECAP_RADIUS_MM`), which is what `parts[].serves` publishes and
    what makes the column honest. The raw election is radius-free and always
    names SOMETHING: on the run-25 fixture it puts a bulk capacitor 17.24mm
    from the USB socket, which is not a fact about what that capacitor serves.

    Extracted from `build_context` (#894) so a scoring pass can have this one
    column without paying for the whole sheet -- `build_context` builds a
    QuenchState TWICE and reads the board file three more times for bodies.
    One implementation, two callers.

    KNOWN INCOMPLETE, and the incompleteness is the reason #894's
    `cluster_to_pin` does not rest on it alone: the election is syntactic (a
    ref starting `C` bridging exactly two nets), so an inductor, a crystal's
    load relation or an ESD array is never a subject here. Its target is a
    cap OWNER (`groups.chip_refs`): an IC by shape, or since #1249 a supply
    owner (`groups.supply_owners` -- a `power_out` pad, the `regulator`
    role, or three copper pads with one ground and two capped rails), so a
    3-pad SOT89 regulator's own caps now serve it (esp_prog C1/C3 -> U2;
    before, C3 -> U1 and C1 -> USB1). The DECLARED channel (#902's
    `proximity` claims) reaches what the election cannot; this is what
    covers the parts nobody declared.

    Never raises: an unreadable board yields `{}`, which the caller must treat
    as "not measured" and not as "nothing serves anything".
    """
    tethers = {}
    try:
        from placement.groups import DECAP_RADIUS_MM
        for cap, ic, dist in _elect(pcb_data):
            if ic and dist is not None and dist <= DECAP_RADIUS_MM:
                tethers[cap] = (ic, dist)
    except Exception:                                        # noqa: BLE001
        tethers = {}
    return tethers


def build_context(pcb_data, pcb_file: str, *, clearance: float,
                  track_width: float, brief=None,
                  schematic=None) -> Dict[str, object]:
    """The whole document, as data. `format_md` renders it.

    `schematic`: None finds the project's root `.kicad_sch` beside the board
    (`placement.schematic.discover`), a path reads that one, and False reads
    none."""
    from placement.body import board_bodies
    from placement.legality import footprint_side, rotate_local_bounds
    from placement.part_class import classify_part
    from kicad_parser import non_aperture_pads
    import list_nets

    bodies = board_bodies(pcb_data, pcb_file)
    faces = pads_by_face(pcb_data, clearance, track_width)

    # net_id -> the refs it reaches, and net_id -> name.
    net_refs: Dict[int, set] = {}
    net_name: Dict[int, str] = {}
    for ref, fp in (pcb_data.footprints or {}).items():
        for pad in (fp.pads or ()):
            nid = getattr(pad, 'net_id', 0) or 0
            if not nid:
                continue
            net_refs.setdefault(nid, set()).add(ref)
            net_name.setdefault(nid, getattr(pad, 'net_name', '') or '')

    try:
        pairs = list_nets.find_differential_pairs(pcb_data)
    except Exception:                                        # noqa: BLE001
        pairs = {}
    tethers = serves_map(pcb_data)

    declared_roles = {}
    if brief is not None:
        for row in (getattr(brief, 'raw', None) or {}).get('interfaces', []):
            if isinstance(row, dict) and row.get('ref') and row.get('role'):
                declared_roles[row['ref']] = row['role']

    try:
        faces_out = mating_faces(pcb_data, pcb_file, clearance)
    except Exception:                                        # noqa: BLE001
        faces_out = {}

    # #1353: on a PILE the poses are staging, not decisions (#1103's rule,
    # through the one pile predicate #1109 shares with emit_intent and
    # board_brief). Every claim read off a pose -- a pin-order verdict, a
    # leg distance, a nearest edge, a nearest-cap election -- is withheld
    # unless every part it reads is KiCad-LOCKED, whose pose IS a decision.
    # gp14 StickHub's pile printed 653 pin-order verdicts (124 CROSSED) and
    # EMC leg distances with no pile notice.
    pile = None
    try:
        from placement.placement_state import assess_placement
        _st = assess_placement(pcb_data, pcb_file)
    except Exception:                                        # noqa: BLE001
        _st = None
    _locked = {r for r, fp in (pcb_data.footprints or {}).items()
               if getattr(fp, 'locked', False)}
    if _st is not None and _st.pile:
        pile = {'reasons': list(_st.reasons[:3]), 'locked': sorted(_locked),
                'withheld': {}}

    def _claim(*refs):
        """A pose-derived fact about `refs` is a claim: not a pile, or
        every one of them is locked."""
        return pile is None or all(r in _locked for r in refs if r)

    # The drawing is pose-independent, so a pile withholds nothing here.
    if schematic is False:
        sch = {'found': False, 'skipped': 'by --no-schematic'}
    else:
        from placement.schematic import section as schematic_section
        try:
            sch = schematic_section(pcb_data, pcb_file, schematic or None)
        except Exception as exc:                             # noqa: BLE001
            sch = {'found': True, 'error': f'{type(exc).__name__}: {exc}'}
    sch_parts = sch.get('parts') or {}

    parts = []
    for ref, fp in sorted((pcb_data.footprints or {}).items()):
        geom = bodies.get(ref)
        cls = classify_part(fp, ref)
        role = infer_role(fp, ref, getattr(cls, 'name', None))
        if ref in declared_roles:
            inferred = role['role']
            role = {'role': declared_roles[ref], 'source': 'brief',
                    'evidence': ['declared in the design brief']}
            if inferred not in ('unknown', declared_roles[ref]):
                role['contradicts_inference'] = inferred

        extent = None
        if geom is not None and geom.body_local is not None:
            x0, y0, x1, y1 = rotate_local_bounds(*geom.body_local,
                                                 fp.rotation or 0.0)
            extent = [round(x1 - x0, 3), round(y1 - y0, 3)]

        side_pads = {}
        for face, pads in sorted((faces.get(ref) or {}).items()):
            side_pads[face] = [{
                'pad': p.pad_number,
                'net': getattr(p, 'net_name', '') or '',
                'pinfunction': getattr(p, 'pinfunction', '') or '',
                'pintype': getattr(p, 'pintype', '') or '',
                'reaches': sorted(net_refs.get(
                    getattr(p, 'net_id', 0) or 0, set()) - {ref}),
            } for p in pads]

        weights: Dict[str, set] = {}
        for pad in (fp.pads or ()):
            nid = getattr(pad, 'net_id', 0) or 0
            for other in net_refs.get(nid, set()):
                if other != ref:
                    weights.setdefault(other, set()).add(nid)
        partners = sorted(((len(v), k) for k, v in weights.items()),
                          reverse=True)

        parts.append({
            'ref': ref,
            'footprint': getattr(fp, 'footprint_name', '') or '',
            'value': getattr(fp, 'value', '') or '',
            'role': role,
            'part_class': getattr(cls, 'name', None),
            'part_class_confidence': getattr(cls, 'confidence', None),
            'body_mm': extent,
            'body_source': (geom.source if geom is not None else 'none'),
            'drawn_body_source': (geom.drawn_source if geom is not None
                                  else 'none'),
            # Apertures (paste/mask windows) are not pads (#1143).
            'pads': len(non_aperture_pads(fp)),
            'side': footprint_side(fp),
            'at': [round(fp.x, 3), round(fp.y, 3),
                   round(fp.rotation or 0.0, 3)],
            'locked': bool(getattr(fp, 'locked', False)),
            'dnp': bool(getattr(fp, 'dnp', False)),
            'pads_by_face': side_pads,
            'partners': [{'ref': r, 'shared_nets': n} for n, r in partners],
            'schematic': sch_parts.get(ref),
            'mating': faces_out.get(ref) if _claim(ref) else None,
            'serves': ({'ref': tethers[ref][0],
                        'distance_mm': round(tethers[ref][1], 3)}
                       if ref in tethers and tethers[ref][0]
                       and _claim(ref, tethers[ref][0]) else None),
        })
        if pile is not None:
            if faces_out.get(ref) and not _claim(ref):
                pile['withheld'].setdefault('parts[].mating', []).append(ref)
            if (ref in tethers and tethers[ref][0]
                    and not _claim(ref, tethers[ref][0])):
                pile['withheld'].setdefault('parts[].serves', []).append(ref)

    po = pin_order_rows(pcb_data, pcb_file, clearance)
    emc = emc_roles_section(pcb_data)
    if pile is not None:
        _withhold_staged(po, emc, _claim, pile)

    return {
        'schema': SCHEMA,
        'board': os.path.basename(pcb_file),
        'bounds': list(pcb_data.board_info.board_bounds or ()),
        'copper_layers': list(pcb_data.board_info.copper_layers or ()),
        # #1197: a pour decides the routing plan, and nothing here named one.
        'pours': _pours(pcb_data, pcb_file),
        'floors': {'clearance': clearance, 'track_width': track_width},
        # `find_differential_pairs` returns (positive, negative) tuples.
        'diff_pairs': [list(t) for t in sorted(pairs or ())],
        'parts': parts,
        'pin_order': po,
        # The project's schematic: sheets, drawn blocks and their notes, and
        # the designer's layout notes. `parts[].schematic` is each part's
        # sheet and block; the per-part map is not repeated here.
        'schematic': {k: v for k, v in sch.items() if k != 'parts'},
        # #1254: converters, crystals, oscillators and ESD arrays, each leg
        # measured and a ready `proximity[]` row beside it. Report only.
        'emc_roles': emc,
        # #1353: None on a placed board; on a pile, why, and every
        # pose-derived claim withheld (by section, with the refs or count).
        'pile': pile,
        'sources': {
            'emc_roles': 'placement.emc_roles (#1254): roles read off PIN '
                         'FUNCTIONS (SW/LX, VIN, PGND/GND; I/O or CH lines) '
                         'and the role table (crystals); every leg pad edge '
                         'to pad edge (legality.pad_rect + rect_gap), never '
                         'a loop area; `unknown` where the pins carry no '
                         'function; rows carry max_mm null -- the limit is '
                         'the author\'s. `keep_away`: PROPOSED design-brief '
                         'rules (#1253), each clock part\'s nets against the '
                         'switch nodes, in route.py\'s --keep-away syntax '
                         '(parsed and resolved by keep_away), at a derived '
                         'default gap, measured by check_floorplan\'s own '
                         'keep_away grade',
            'body_mm / body_source': 'placement.body.board_bodies (#896)',
            'schematic': 'placement.schematic: the root .kicad_sch beside '
                         'the board (or --schematic), its sheet tree, and '
                         'each footprint matched to its symbol by the uuids '
                         'of its (path ...). A block is the parts drawn '
                         'together on one sheet: average linkage on the gap '
                         'between their symbols\' drawn boxes, cut at '
                         'BLOCK_CUT_MM. A note is a short text beside a '
                         'block. A layout note\'s rows name both parts in '
                         'the note\'s own words and are checked against '
                         'the netlist (STALE when the named part is not on '
                         'the pin\'s net); max_mm is the author\'s',
            'pads_by_face': 'placement.escape.assign_faces (#850)',
            'pin_order': 'placement.pair_order.pair_metrics; a pair through '
                         'per-leg series parts (#1251): the junctions of '
                         'diff_xnet.find_ac_coupled_xnets, judged by '
                         'pair_order.composed_pair_metrics',
            'part_class': 'placement.part_class.classify_part',
            'serves': 'placement.groups tether election: the nearest cap '
                      'owner on the cap\'s rail -- an IC (4+ copper pads, '
                      'not a row) or a supply owner such as a 3-pad '
                      'regulator (#1249)',
            'diff_pairs': 'list_nets.find_differential_pairs '
                          '(rejects 2-terminal resonators)',
            'pours': 'board_brief.pour_census (the brief\'s own `pours`)',
            'role': 'placement.roles.infer_role, from footprint name, pins '
                    '(pintype / pin function) and reference prefix; the '
                    'board carries no datasheet or 3D-model field to cite',
            'mating': 'which parts count as connectors: '
                      'render_placement.connector_edge_facts. `edge` is the '
                      'edge the mating face points at when the body names '
                      'one (placement.mating, #1155), else the nearest, and '
                      '`dist_mm` / `overhang_mm` measure to THAT edge; '
                      '`nearest_*` always to the nearest. Both from the '
                      'placement.body occupancy rect, because '
                      'connector_edge_facts reads the placement state pad-box ladder '
                      'and would put two geometries on one sheet',
        },
    }


def _elect(pcb_data):
    from placement.groups import _elect_tethers      # noqa: PLC2701
    return _elect_tethers(pcb_data)


def emc_roles_section(pcb_data) -> dict:
    """`placement.emc_roles.emc_roles` (#1254), or `{'error': ...}` -- named,
    never an empty section a reader would take for "no hot loop here"."""
    try:
        from placement.emc_roles import emc_roles
        return emc_roles(pcb_data)
    except Exception as exc:                                 # noqa: BLE001
        return {'error': f'{type(exc).__name__}: {exc}', 'roles': [],
                'counts': {}}


def _emc_md(sec) -> List[str]:
    """The markdown for `emc_roles`: one block per role, its legs as a
    table, its rows ready to paste."""
    L = ['## EMC roles (report only)', '']
    if sec.get('error'):
        L += [f"NOT MEASURED ({sec['error']}).", '']
        return L
    if not sec.get('roles'):
        L += ['No switching converter, crystal, oscillator, ESD array or '
              'ESD diode is named by the pins or the role table here.', '']
        return L
    L += [sec.get('note', ''), '', f"Legs: {sec.get('basis', '')}.", '']
    for r in sec['roles']:
        what = r['role'] + (f" ({r['topology']})" if r.get('topology')
                            else '')
        if r['role'] == 'unknown':
            what = f"unknown -- a {r.get('candidate')}?"
        L.append(f"### {r['part']}  --  {what}"
                 + (f"  value `{r['value']}`" if r.get('value') else ''))
        L.append('')
        for e in r.get('evidence') or ():
            L.append(f"- {e}")
        if r.get('why'):
            L.append(f"- **Not read:** {r['why']}")
        for n in r.get('notes') or ():
            L.append(f"- note: {n}")
        for u in r.get('unmeasured_legs') or ():
            L.append(f"- pad {u['pad']} ({u['net']}) not measured: "
                     f"{u['why']}")
        rows = r.get('proximity') or []
        if rows:
            L.append('')
            L.append('| leg | from | to | net | gap mm | sides |')
            L.append('|---|---|---|---|---|---|')
            for pr in rows:
                for g in pr['legs']:
                    a, b = g['from'], g['to']
                    L.append(f"| {g['leg']} | {a['ref']}.{a['pad']} | "
                             f"{b['ref']}.{b['pad']}"
                             + (f" ({b['pin_function']})"
                                if b.get('pin_function') else '')
                             + f" | {a['net']} | "
                             f"{'-' if g['gap_mm'] is None else g['gap_mm']}"
                             f" | {a['side']} / {b['side']} |")
            L.append('')
            for pr in rows:
                _w = pr['worst_leg_mm']
                L.append(f"- row, worst leg "
                         + ('withheld (staging poses)' if _w is None
                            else f"{_w} mm")
                         + f": `{json.dumps(pr['row'], sort_keys=True)}`")
        L.append('')
    L += _keep_away_md(sec)
    return L


def _keep_away_md(sec) -> List[str]:
    """The keep-away PROPOSALS (#1253): each rule, what it protects from
    what, check_floorplan's reading of it now, and the brief rows ready to
    paste."""
    props = sec.get('keep_away') or []
    skipped = sec.get('keep_away_skipped') or []
    if not props and not skipped:
        return []
    gap = sec.get('keep_away_gap') or {}
    L = ['### Keep-away proposals (derived default gap -- NOT declared)', '',
         sec.get('keep_away_note', ''), '',
         f"Gap {gap.get('gap_mm')} mm: {gap.get('basis', '')}.", '']
    if props:
        L += ['| rule | clock | switch nodes | closest now (pad edge) |',
              '|---|---|---|---|']
        for p in props:
            c, m = p['clock'], p.get('measured') or {}
            sw = '; '.join(f"{s['net']} ({s['part']}, {s['kind']}, "
                           f"{s['read']})" for s in p['switch_nodes'])
            cl = m.get('closest') or {}
            v, a = cl.get('victim') or {}, cl.get('aggressor') or {}
            now = (f"{m['min_gap_mm']} mm, {v.get('ref')}.{v.get('pad')} "
                   f"({v.get('side')}) to {a.get('ref')}.{a.get('pad')} "
                   f"({a.get('side')})"
                   + (', opposite faces' if cl.get('cross_face') else '')
                   + (' -- INSIDE the proposed gap' if p.get('inside_now')
                      else '')
                   if m.get('min_gap_mm') is not None
                   else f"not measured ({m.get('state')})")
            L.append(f"| `{p['rule']}` | {c['part']} {c['role']} "
                     f"({c['read']}) | {sw} | {now} |")
        L += ['', "Ready to paste into the design brief's `keep_away`:", '',
              '```json',
              json.dumps({'keep_away': [p['brief_row'] for p in props]},
                         indent=1),
              '```', '']
    for n in skipped:
        L.append(f"- not proposed: {n}")
    if skipped:
        L.append('')
    return L


def _verdict(m):
    """AGREES / CROSSED / UNDETERMINED.

    A tie means two pads project to one point on the channel axis, so their
    order is not a fact about the board -- `pair_metrics`' deterministic
    tie-break invents one. On a two-net pair a single tie makes the verdict
    meaningless, which is why UNDETERMINED wins over the inversion count
    rather than being a footnote to it: the tracked esp_prog reports exactly
    this for the USB pair at U1's current rotation, and calling it AGREES
    would tell a reader the polarity is fine when nothing measured it.
    """
    if m.get('ties'):
        return 'UNDETERMINED (pads tie on the channel axis)'
    return 'AGREES' if not m.get('inversions') else 'CROSSED'


def _pair_span_mm(pcb_data, a: str, b: str, only_nets=None):
    """Worst straight-line pad-to-pad span between two parts, over the nets
    they SHARE. `None` when they share none.

    #895's criterion 1 asks for the length a pair or bus is forced to run, and
    nothing in this toolchain produced it: `pair_metrics` computes inversions
    and discards the distance, `render_placement --json-out` gives a
    board-total `hpwl`, and `net_affinity` needs declared zoned blocks. So it
    is measured here, where the pin-order rows already stand.

    The WORST net rather than the mean: a bus is as long as its longest
    member, and averaging hides the one that will not fit. Pad CENTRES, not
    edges, because this is a routing-length question rather than a clearance
    one -- and it is deliberately a straight line, not a route: the criterion
    compares it against what the two footprints would allow side by side,
    which is a judgement the reader makes with the two `body_mm` extents
    printed beside it. A tool that guessed that denominator would be inventing
    the threshold the criterion exists to leave to a human.
    """
    import math
    fa = (pcb_data.footprints or {}).get(a)
    fb = (pcb_data.footprints or {}).get(b)
    if fa is None or fb is None:
        return None
    want = set(only_nets or ())
    by_a = {}
    for pad in (fa.pads or ()):
        nid = getattr(pad, 'net_id', 0) or 0
        if nid > 0 and (not want or nid in want):
            by_a.setdefault(nid, []).append(pad)
    worst = None
    for pad in (fb.pads or ()):
        nid = getattr(pad, 'net_id', 0) or 0
        if nid <= 0 or nid not in by_a:
            continue
        near = min(math.dist((q.global_x, q.global_y),
                             (pad.global_x, pad.global_y))
                   for q in by_a[nid])
        if worst is None or near > worst:
            worst = near
    return None if worst is None else round(worst, 3)


def pin_order_rows(pcb_data, pcb_file: str, clearance: float):
    """Pin-order agreement for every connected part pair sharing >= 2 nets.

    `pair_order.pair_inversions` is the implementation and the argument
    (Supowit 1987; Leiserson & Pinter 1983): `inversions` is a LOWER BOUND on
    the crossings any router must pay, and `lis` is the largest subset routable
    with none. This prints its verdict; it does not re-derive it.
    """
    try:
        from placement.pair_order import pair_inversions
        # `PlacementModel` builds the QuenchState, resolving the board's own
        # floors and handling the no-outline fallback. Constructing one here
        # would be a second copy of a ten-argument block that has already
        # drifted once (before #431 fixed it).
        from render_placement import PlacementModel
        model = PlacementModel(pcb_data, pcb_file,
                               quench_kwargs={'clearance': clearance})
        if model.state is None:
            return {'error': 'the placement state could not be built', 'rows': []}
        got = pair_inversions(model.state)
    except Exception as exc:                                 # noqa: BLE001
        return {'error': f'{type(exc).__name__}: {exc}', 'rows': []}
    rows = []
    for (a, b), m in sorted(got.items()):
        rows.append({
            'a': a, 'b': b, 'nets': m.get('nets'), 'scope': 'interface',
            'inversions': m.get('inversions'), 'lis': m.get('lis'),
            'ties': m.get('ties', 0), 'verdict': _verdict(m),
            'span_mm': _pair_span_mm(pcb_data, a, b),
        })

    # PAIR-SCOPED rows, and they are the point. A differential pair's polarity
    # parity is a two-net question, and asking it as part of the whole
    # interface hides it: on esp_prog U1<->USB1 shares three nets and AGREES
    # overall while /D_P and /D_N alone are CROSSED. That fact decided run
    # 25's only open clause and was found after routing.
    #
    # `pairs` is the CENSUS behind those rows (#1251): every pair
    # `list_nets.find_differential_pairs` found, which ones a row measures,
    # and why each other one has none. Without it a reader of the rows cannot
    # tell "no pair on this board" from "14 pairs, none carried whole by two
    # parts" -- ulx3s is the second and `pair_length` used to say the first.
    census = {'found': [], 'direct': [], 'composed': [], 'not_measured': [],
              'xnet_warnings': [], 'xnets_not_chained': []}
    found, name_to_id, direct = [], {}, set()
    try:
        from placement.pair_order import pair_metrics
        import list_nets
        for ref, fp in (pcb_data.footprints or {}).items():
            for pad in (fp.pads or ()):
                nm = getattr(pad, 'net_name', '') or ''
                if nm:
                    name_to_id.setdefault(nm, getattr(pad, 'net_id', 0) or 0)
        found = list(list_nets.find_differential_pairs(pcb_data) or ())
        census['found'] = [[pos, neg] for pos, neg in found]
        for pos, neg in found:
            ids = [name_to_id.get(pos), name_to_id.get(neg)]
            if None in ids or 0 in ids:
                continue
            carriers = sorted({r for r, fp in (pcb_data.footprints or {}).items()
                               if any((getattr(q, 'net_id', 0) or 0) in ids
                                      for q in (fp.pads or ()))})
            for i, a in enumerate(carriers):
                for b in carriers[i + 1:]:
                    m = pair_metrics(model.state, a, b, only_nets=ids)
                    if m is None:
                        continue
                    direct.add(f'{pos}/{neg}')
                    rows.append({
                        'a': a, 'b': b, 'nets': m['nets'],
                        'scope': f'pair {pos}/{neg}',
                        'inversions': m['inversions'], 'lis': m['lis'],
                        'ties': m.get('ties', 0), 'verdict': _verdict(m),
                        'span_mm': _pair_span_mm(pcb_data, a, b,
                                                 only_nets=ids),
                    })
    except Exception as exc:                                 # noqa: BLE001
        rows.append(_pair_error_row('the pair rows', exc))
        census['error'] = f'{type(exc).__name__}: {exc}'
        found = []
    composed = set()
    if found:
        try:
            got, composed, census['xnet_warnings'], \
                census['xnets_not_chained'] = composed_pair_rows(
                    pcb_data, model.state, found, name_to_id)
            rows.extend(got)
        except Exception as exc:                             # noqa: BLE001
            rows.append(_pair_error_row('the composed pairs', exc))
            census['error'] = f'{type(exc).__name__}: {exc}'
    census['direct'] = sorted(direct)
    census['composed'] = sorted(composed)
    if 'error' not in census:
        census['not_measured'] = _unmeasured_pairs(
            pcb_data, found, name_to_id, direct | composed,
            census['xnets_not_chained'])

    # Pair rows first: they are the narrow, unfixable-by-rotation claim.
    rows.sort(key=lambda r: (r['scope'] == 'interface',
                             -(r['inversions'] or 0), r['a'], r['b']))
    return {'error': None, 'rows': rows, 'pairs': census}


STAGED = 'withheld: the board is a pile, so this pose is staging (#1353)'


def _withhold_staged(po, emc, claim, pile):
    """On a pile, take every pose-derived claim out of the pin-order and
    EMC sections unless all the parts it reads are locked (#1353), and
    count what went into `pile['withheld']`. The netlist half stays: which
    parts share which nets, which pins a role names, the proposed rules."""
    rows = po.get('rows') or []
    kept = [r for r in rows if r.get('error') or claim(r['a'], r['b'])]
    if len(kept) != len(rows):
        pile['withheld']['pin_order.rows'] = len(rows) - len(kept)
        po['rows'] = kept
    n_legs = 0
    for role in (emc or {}).get('roles') or ():
        for pr in role.get('proximity') or ():
            for g in pr.get('legs') or ():
                if not claim(g['from']['ref'], g['to']['ref']):
                    g['gap_mm'] = None
                    g['withheld'] = STAGED
                    n_legs += 1
            if any(g.get('withheld') for g in pr.get('legs') or ()):
                pr['worst_leg_mm'] = None
    if n_legs:
        pile['withheld']['emc_roles legs'] = n_legs
    n_ka = 0
    for p in (emc or {}).get('keep_away') or ():
        refs = [p['clock']['part']] + [s['part'] for s in p['switch_nodes']]
        if not claim(*refs):
            p['measured'] = {'state': STAGED}
            p['inside_now'] = None
            n_ka += 1
    if n_ka:
        pile['withheld']['emc_roles keep_away measured'] = n_ka


def _pair_error_row(what, exc):
    """The row a raised pair branch leaves. Its scope is the bare `pair`: no
    pair is named because none was measured, and `pair_length` reads that
    scope as "the pair population is incomplete", never as "no pair"."""
    return {'a': '-', 'b': '-', 'nets': 0, 'scope': 'pair',
            'inversions': None, 'lis': None, 'span_mm': None,
            'verdict': f'NOT MEASURED ({type(exc).__name__})',
            'error': f'{what}: {type(exc).__name__}: {exc}'}


def _copper_nets(fp):
    """The net ids on a part's pads, aperture-only windows excluded (#1143)."""
    from kicad_parser import non_aperture_pads
    return {getattr(q, 'net_id', 0) or 0 for q in non_aperture_pads(fp)}


def _carried_whole_by(pcb_data, net_ids, exclude=()):
    """The parts whose copper pads carry EVERY net in `net_ids`."""
    want = set(net_ids)
    return sorted(r for r, fp in (pcb_data.footprints or {}).items()
                  if r not in exclude and want <= _copper_nets(fp))


#: What a composed row's span is, printed beside it.
COMPOSED_SPAN_BASIS = ('the longer leg: end pad -> series part pads -> end '
                       'pad, straight between pad centres')


def composed_pair_rows(pcb_data, state, found, name_to_id):
    """Rows for pairs that run END TO END through per-leg series parts (#1251).

    A pair through AC caps or USB series resistors is two name-detected pairs:
    U1 carries FPDI_D0+/-, GPDI1 carries /gpdi/GPDI_D0+/-, and C42 / C43 each
    carry one net of each. No two parts carry both nets of either pair, so the
    direct rows above find none of it -- ulx3s has 14 pairs and no row.

    The junctions are the ROUTER's: `diff_xnet.find_ac_coupled_xnets` (#196),
    which joins two pairs only when BOTH polarities are bridged, each by its
    own non-DNP part with exactly two copper pads, on the same polarity. Its
    chain is walked from one end to the other, and the row carries:

      * `a` / `b`: the end parts, the one part (series parts aside) carrying
        each end pair whole. None or several is said, not chosen;
      * `series`: the series part on each leg, in chain order;
      * `legs_mm`: each leg's pad-centre polyline, end pad -> series pads ->
        end pad, taking the end pad nearest its series pad; `leg_skew_mm` is
        |P - N|. Numbers, not a verdict: a straight-line floor says what the
        placement forces, and meanders can still add copper;
      * `span_mm`: the longer leg (`COMPOSED_SPAN_BASIS`) -- worst member, as
        `_pair_span_mm` takes it for a direct pair;
      * the polarity verdict between the two END parts
        (`pair_order.composed_pair_metrics`).

    The chain follows name-detected pairs only, so it stops at the last part
    carrying one whole -- which need not be where the signal ends (rp2350's
    stops at U3, a flow-through ESD part whose J1-side nets are not a named
    pair). `stops_at` names each end with its part class and copper pad count
    and the row's `note` says so; nothing past an end is followed or guessed.

    Returns (rows, composed pair keys, the router's warnings, XNets that are
    not a simple chain). A key is "POS/NEG", as in a direct row's scope.
    """
    import math
    from diff_xnet import find_ac_coupled_xnets
    from kicad_parser import non_aperture_pads
    from routing_config import DiffPairNet
    from placement.pair_order import composed_pair_metrics
    from placement.part_class import classify_part

    fps = pcb_data.footprints or {}
    dp, member = {}, {}
    for pos, neg in found:
        ids = (name_to_id.get(pos), name_to_id.get(neg))
        if None in ids or 0 in ids:
            continue
        key = f'{pos}/{neg}'
        dp[key] = DiffPairNet(base_name=key, p_net_id=ids[0], n_net_id=ids[1],
                              p_net_name=pos, n_net_name=neg)
        member[ids[0]] = (key, 'P')
        member[ids[1]] = (key, 'N')
    xnets, warnings = find_ac_coupled_xnets(pcb_data, dp)

    def dist(p, q):
        return math.dist((p.global_x, p.global_y), (q.global_x, q.global_y))

    def end_note(ref):
        fp = fps[ref]
        try:
            cls = classify_part(fp, ref).name
        except Exception:                                    # noqa: BLE001
            cls = None
        return {'ref': ref, 'class': cls,
                'copper_pads': len(non_aperture_pads(fp))}

    rows, composed, unchained = [], set(), []
    for xn in xnets:
        keys = sorted(m.base_name for m in xn.members)
        bridges = set(xn.bridge_refs)
        # (key, key) -> polarity -> {'ref': series part, key: its pad there}
        junction = {}
        for ref in sorted(bridges):
            p0, p1 = non_aperture_pads(fps[ref])
            # One polarity per part: the junction rule refuses a part that
            # bridges P to N, so both pads name the same one.
            k0, pol = member[p0.net_id]
            k1 = member[p1.net_id][0]
            junction.setdefault(frozenset((k0, k1)), {})[pol] = {
                'ref': ref, k0: p0, k1: p1}
        nbrs = {k: set() for k in keys}
        for edge in junction:
            a_k, b_k = sorted(edge)
            nbrs[a_k].add(b_k)
            nbrs[b_k].add(a_k)
        tips = [k for k in keys if len(nbrs[k]) == 1]
        if (len(junction) != len(keys) - 1 or len(tips) != 2
                or any(len(v) > 2 for v in nbrs.values())):
            unchained.append({'pairs': keys, 'series': sorted(bridges),
                              'why': 'its series parts join the pairs in a '
                                     'branch, not a chain, so there is no '
                                     'single end-to-end leg'})
            continue
        order, prev = [tips[0]], None
        while len(order) < len(keys):
            nxt = sorted(nbrs[order[-1]] - {prev})[0]
            prev = order[-1]
            order.append(nxt)
        ends = [_carried_whole_by(pcb_data, (dp[k].p_net_id, dp[k].n_net_id),
                                  exclude=bridges)
                for k in (order[0], order[-1])]
        if all(len(e) == 1 for e in ends) and ends[0][0] > ends[1][0]:
            order.reverse()
            ends.reverse()
        composed.update(order)
        scope = 'pair ' + ' + '.join(order)
        series = {pol: [junction[frozenset((order[i], order[i + 1]))][pol]
                        ['ref'] for i in range(len(order) - 1)]
                  for pol in ('P', 'N')}
        row = {'a': ','.join(ends[0]) or '-', 'b': ','.join(ends[1]) or '-',
               'nets': 2, 'scope': scope, 'composed': True,
               'series': series, 'inversions': None, 'lis': None, 'ties': 0,
               'span_mm': None, 'span_basis': COMPOSED_SPAN_BASIS,
               'legs_mm': None, 'leg_skew_mm': None}
        if not all(len(e) == 1 for e in ends):
            bad = [(k, e) for k, e in zip((order[0], order[-1]), ends)
                   if len(e) != 1]
            row['verdict'] = 'NOT MEASURED (an end part is not determined)'
            row['note'] = '; '.join(
                f'{k} is carried whole by ' + (', '.join(e) if e else
                                               'no part')
                + (' -- which of them ends the signal is not something the '
                   'netlist says' if e else '')
                for k, e in bad)
            rows.append(row)
            continue
        a, b = ends[0][0], ends[1][0]
        legs = {}
        for pol in ('P', 'N'):
            pts = []
            for i in range(len(order) - 1):
                j = junction[frozenset((order[i], order[i + 1]))][pol]
                pts += [j[order[i]], j[order[i + 1]]]
            nid_a = getattr(dp[order[0]], f'{pol.lower()}_net_id')
            nid_b = getattr(dp[order[-1]], f'{pol.lower()}_net_id')
            first = min((q for q in non_aperture_pads(fps[a])
                         if q.net_id == nid_a),
                        key=lambda q: (dist(q, pts[0]), q.global_x,
                                       q.global_y))
            last = min((q for q in non_aperture_pads(fps[b])
                        if q.net_id == nid_b),
                       key=lambda q: (dist(q, pts[-1]), q.global_x,
                                      q.global_y))
            path = [first] + pts + [last]
            legs[pol] = sum(dist(path[i], path[i + 1])
                            for i in range(len(path) - 1))
        m = composed_pair_metrics(
            state, a, b, (dp[order[0]].p_net_id, dp[order[0]].n_net_id),
            (dp[order[-1]].p_net_id, dp[order[-1]].n_net_id))
        stops = [end_note(a), end_note(b)]
        row.update({
            'a': a, 'b': b,
            'legs_mm': {pol: round(v, 3) for pol, v in legs.items()},
            # From the unrounded legs: rounding each first can move the
            # difference by a unit in the last place.
            'leg_skew_mm': round(abs(legs['P'] - legs['N']), 3),
            'span_mm': round(max(legs.values()), 3),
            'stops_at': stops,
            'note': ('composed through name-detected pairs only: it stops at '
                     + ' and '.join(f"{s['ref']} ({s['class'] or 'no part class'}"
                                    f", {s['copper_pads']} copper pads)"
                                    for s in stops)
                     + ', where no name-detected pair continues. A part '
                       'that is not a connector may still pass the signal on '
                       'through nets the name-based detector cannot pair; '
                       'those are not followed.')})
        if m is None:
            row['verdict'] = ('NOT MEASURED (the end parts have no channel '
                              'axis or lack an escape pad per leg)')
        else:
            row.update({'nets': m['nets'], 'inversions': m['inversions'],
                        'lis': m['lis'], 'ties': m.get('ties', 0),
                        'verdict': _verdict(m)})
        rows.append(row)
    return rows, composed, warnings, unchained


def _unmeasured_pairs(pcb_data, found, name_to_id, measured, unchained):
    """Each found pair no row measures, with the reason it has none."""
    in_branch = {k for u in unchained for k in u['pairs']}
    out = []
    for pos, neg in found:
        key = f'{pos}/{neg}'
        if key in measured:
            continue
        ids = (name_to_id.get(pos), name_to_id.get(neg))
        if None in ids or 0 in ids:
            out.append({'pair': key, 'carried_whole_by': [],
                        'why': 'a net of the pair is on no pad'})
            continue
        whole = _carried_whole_by(pcb_data, ids)
        if key in in_branch:
            why = ('its series parts join it to other pairs in a branch, not '
                   'a chain, so there is no single end-to-end leg')
        elif len(whole) >= 2:
            why = (f"carried whole by {', '.join(whole)}, but pair_metrics "
                   f"gave no row (the parts coincide, or one lacks an escape "
                   f"pad per net)")
        else:
            why = ((f'carried whole by {whole[0]} only' if whole
                    else 'no part carries both nets')
                   + ', and no series part joins it end to end to another '
                     'detected pair')
        out.append({'pair': key, 'carried_whole_by': whole, 'why': why})
    return out


def _pours(pcb_data, pcb_file):
    """board_brief's pour census, the same call (#1197); None if it fails."""
    try:
        from board_brief import pour_census
        return pour_census(pcb_data, pcb_file)
    except Exception:                            # noqa: BLE001 - disclosed
        return None


def mating_line(m) -> str:
    """The sheet's "Mating face" sentence for one `mating_faces` row.

    #1155: when the body names a face (`edge_basis` 'facing') that is not the
    edge the part sits nearest, the sentence says both. OPPOSITE is a
    reversed pose (sonde_xilinx J2's shell over the board), said only of a
    strong face -- the kind the edge rule ERRORs; on a weak face it is a
    reviewer's question. A perpendicular face is a part opening ALONG its
    edge, which a corner connector does by design (StickHub J5)."""
    near = m.get('nearest_edge', m['edge'])
    if m.get('edge_basis') == 'facing' and near != m['edge']:
        if _OPPOSITE.get(near) == m['edge'] and m.get('facing_strong'):
            tail = "  --  the body extends INTO the board; a reversed pose"
        elif _OPPOSITE.get(near) == m['edge']:
            tail = ("  --  it points into the board on a weak face "
                    "(under 2 mm), which a reviewer should check")
        else:
            tail = (f"  --  it opens toward {m['edge']}, along the {near} "
                    f"edge rather than out of it")
        # A weak face says so: three stock receptacles read a confident
        # wrong face under 2 mm (`placement.mating`).
        return (f"Mating face **{m['edge']}** (the body leaves its pad row "
                f"toward {m['edge']}, {m['facing_basis']}"
                + ('' if m.get('facing_strong') else ', a weak face')
                + f"), but it sits "
                f"nearest the **{near}** edge, {m['nearest_dist_mm']}mm from "
                f"it"
                + (f", overhanging it by {m['nearest_overhang_mm']}mm"
                   if m['nearest_overhang_mm'] > 0 else '')
                + tail
                + ('  --  INTERIOR for a connector, which a reviewer should '
                   'explain' if m['interior'] else ''))
    return (f"Mating face **{m['edge']}**"
            + ('' if m.get('edge_basis') == 'facing' else
               f" (nearest edge; "
               f"{m.get('facing_reason') or 'the body names no face'})")
            + f", {m['dist_mm']}mm from that edge"
            + (f", overhanging it by {m['overhang_mm']}mm"
               if m['overhang_mm'] > 0 else '')
            + ('  --  INTERIOR for a connector, which a reviewer should '
               'explain' if m['interior'] else ''))


def _schematic_md(sec) -> List[str]:
    """The `## Schematic` section: each sheet's drawn blocks with the notes
    beside them, then the designer's layout notes."""
    L = ['## Schematic', '']
    if sec.get('skipped'):
        return L + [f"Not read ({sec['skipped']}).", '']
    if sec.get('error'):
        return L + [f"NOT READ ({sec['error']}) -- this sheet cannot say how "
                    f"the parts are drawn.", '']
    if not sec.get('found'):
        return L + ['No schematic beside the board (looked for '
                    + ', '.join(f"`{os.path.basename(p)}`"
                                for p in sec.get('looked_for') or ())
                    + '; `--schematic PATH` names one).', '']
    L.append(f"Read `{sec['root']}` ({sec['files']} file(s), "
             f"{sec['sheet_instances']} sheet instance(s)): "
             f"{sec['matched']} of {sec['board_parts']} board parts matched "
             f"to their symbols by the uuids of their `(path ...)`.")
    if sec.get('missing_files'):
        L.append('Sheet files NOT FOUND: '
                 + ', '.join(f"`{m}`" for m in sec['missing_files'])
                 + ' -- their parts read as not on the schematic.')
    if sec.get('not_on_schematic'):
        miss = sec['not_on_schematic']
        L.append(f"Not on the schematic ({len(miss)}): "
                 + ', '.join(miss[:30]) + (' ...' if len(miss) > 30 else '')
                 + ('. Fewer than 80% matched: the schematic and the board '
                    'disagree, so read the blocks as a stale drawing.'
                    if sec['matched'] < 0.8 * max(1, sec['board_parts'])
                    else '.'))
    L.append('')
    L.append('A **block** is the parts drawn together on one sheet (average '
             f"drawn gap within {sec['block_cut_mm']} mm). On the human "
             'boards a drawn block sits as tight as a functional group: '
             'blocks are evidence for which group a part joins, sheets for '
             'the groups themselves. Quoted text is a note drawn beside the '
             'block.')
    L.append('')
    for sh in sec.get('sheets') or ():
        L.append(f"### Sheet `{sh['sheet'] or '<root>'}` -- {sh['file']}, "
                 f"{sh['parts']} part(s)")
        L.append('')
        for b in sh['blocks']:
            notes = ''.join(f' "{n}"' for n in b['notes'][:4])
            L.append(f"- **{b['id']}** {' '.join(b['refs'])}"
                     + (f" --{notes}" if notes else ''))
        if sh['single']:
            L.append(f"- drawn on their own: {' '.join(sh['single'])}")
        if sh['notes']:
            L.append('- other notes: '
                     + '; '.join(f'"{n}"' for n in sh['notes'][:8]))
        L.append('')
    notes = sec.get('layout_notes') or ()
    if notes:
        L.append('### Layout notes')
        L.append('')
        L.append('The designer\'s own placement instructions. A row names '
                 'both parts in the note\'s words and was checked against '
                 'the netlist; `max_mm` is null because the limit is the '
                 'author\'s. A STALE row names a part that is not on the '
                 'named pin\'s net (the note predates a renumbering): never '
                 'use it. Where the note does not name its subject, the '
                 'parts drawn nearest it are listed: read the note, and '
                 'decide.')
        L.append('')
        for n in notes:
            L.append(f"- \"{n['text']}\" (sheet `{n['sheet'] or '<root>'}`)"
                     + (f"; names {', '.join(n['named'])}" if n['named']
                        else '')
                     + (f"; pin {', '.join(n['pins'])} read as "
                        f"{', '.join(n['pin_owner'])}'s (the part drawn "
                        f"nearest with that pad)" if n.get('pin_owner')
                        else '')
                     + (f"; drawn nearest {', '.join(n['drawn_nearest'])}"
                        if n['drawn_nearest'] else ''))
            for r in n['rows']:
                L.append('  - row `' + json.dumps(
                    {k: v for k, v in r.items() if k != 'why'},
                    sort_keys=True) + '`')
            for r in n.get('stale') or ():
                L.append(f"  - STALE: {r['stale']}")
        L.append('')
    return L


def format_md(doc) -> str:
    """The sheet, for a reader. Markdown because the audience is a model and a
    human reading the same page, and a table is how a pin row reads."""
    L = []
    b = doc['bounds']
    L.append(f"# Component context -- {doc['board']}")
    L.append('')
    if b:
        L.append(f"Board {round(b[2] - b[0], 2)} x {round(b[3] - b[1], 2)} mm, "
                 f"layers {', '.join(doc['copper_layers'])}, floors "
                 f"clearance {doc['floors']['clearance']}mm / track "
                 f"{doc['floors']['track_width']}mm.")
    if doc.get('panels_error'):
        L.append(f"PANELS NOT WRITTEN ({doc['panels_error']}).")
        L.append('')
    pile = doc.get('pile')
    if pile:
        # #1353: BEFORE any number, as board_brief says it.
        L.append('')
        L.append('**This board is a PILE** ('
                 + '; '.join(pile.get('reasons') or ()) + '): its poses are '
                 'staging, not decisions. Every claim read off a pose -- a '
                 'pin-order verdict, a leg distance, a mating edge, a '
                 'nearest-cap election -- is WITHHELD unless every part it '
                 'reads is KiCad-locked'
                 + (f" ({', '.join(pile['locked'][:8])}"
                    + ('...' if len(pile['locked']) > 8 else '') + ')'
                    if pile.get('locked') else ' (none is)')
                 + '. A part\'s printed `pose` is where the file has it. '
                 'Withheld: '
                 + (', '.join(f"{k} {v if isinstance(v, int) else len(v)}"
                              for k, v in sorted(pile['withheld'].items()))
                    or 'nothing') + '.')
        L.append('')
    pc = doc.get('pours')
    if pc is None:
        L.append('Copper pours: NOT MEASURED.')
    else:
        from board_brief import format_pours
        L.append('Copper pours: ' + (format_pours(pc) or 'none') + '.'
                 + (f" Keep-out rule areas: {pc['keepout_areas']}."
                    if pc.get('keepout_areas') else ''))
    if doc['diff_pairs']:
        L.append('Differential pairs: '
                 + ', '.join('/'.join(t) for t in doc['diff_pairs']) + '.')
    L.append('')
    L.append('Every derived column names its source; `unknown` means the tool '
             'looked and could not tell, which is not the same as absent. '
             'See the SOURCES section at the end.')
    L.append('')

    if doc.get('schematic') is not None:
        L.extend(_schematic_md(doc['schematic']))

    po = doc['pin_order']
    L.append('## Pin-order agreement')
    L.append('')
    _po_withheld = ((pile or {}).get('withheld') or {}).get('pin_order.rows')
    if _po_withheld:
        L.append(f"{_po_withheld} pin-order row(s) WITHHELD: the board is a "
                 f"pile, and a verdict measured at staging poses is not a "
                 f"claim. Place the parts, then read this section.")
        L.append('')
    if po.get('error'):
        L.append(f"NOT MEASURED ({po['error']}) -- this sheet cannot say "
                 f"whether any pair's order crosses.")
    elif not po['rows'] and _po_withheld:
        pass
    elif not po['rows']:
        L.append('No two parts share 2 or more nets, so there is no pin order '
                 'to agree about.')
    else:
        L.append('A CROSSED pair forces at least `inversions` crossings on '
                 'ANY router: on a 2-layer board that is a via per net or '
                 'back-side copper. Rotation cannot fix it -- parity flips '
                 'only under a mirror.')
        L.append('')
        L.append('| A | B | scope | nets | span mm | inversions | '
                 'max planar | verdict |')
        L.append('|---|---|---|---|---|---|---|---|')
        for r in po['rows']:
            _sp = r.get('span_mm')
            L.append(f"| {r['a']} | {r['b']} | {r['scope']} | {r['nets']} | "
                     f"{'-' if _sp is None else _sp} | "
                     f"{r['inversions']} | {r['lis']} | {r['verdict']} |")
        L.append('')
        L.append('`span mm` is the WORST straight-line pad-to-pad distance '
                 'over the nets the two parts share -- the length this pair '
                 'or bus is forced to run. Compare it against the shortest '
                 'the two bodies allow side by side (their `body_mm` are '
                 'below); a ratio much above 1.5 is a finding to explain. '
                 'The threshold is a judgement, which is why the tool '
                 'reports the measurement and not a verdict.')
        composed = [r for r in po['rows'] if r.get('composed')]
        if composed:
            L.append('')
            L.append('Pairs composed END TO END through per-leg series parts '
                     '(#1251): `a`/`b` are the end parts, the verdict is '
                     'between them, and the span is '
                     f'{COMPOSED_SPAN_BASIS}. Each leg is printed; |P-N| is '
                     'a straight-line floor difference, not a verdict.')
            L.append('')
            L.append('| A | B | series P / N | P mm | N mm | abs(P-N) mm | '
                     'note |')
            L.append('|---|---|---|---|---|---|---|')
            for r in composed:
                legs = r.get('legs_mm') or {}
                ser = r.get('series') or {}
                L.append(f"| {r['a']} | {r['b']} | "
                         f"{','.join(ser.get('P') or [])} / "
                         f"{','.join(ser.get('N') or [])} | "
                         f"{legs.get('P', '-')} | {legs.get('N', '-')} | "
                         f"{r.get('leg_skew_mm', '-')} | "
                         f"{r.get('note') or ''} |")
    unmeasured = (po.get('pairs') or {}).get('not_measured') or []
    if unmeasured:
        L.append('')
        L.append('Differential pairs no row measures: '
                 + '; '.join(f"{p['pair']} ({p['why']})" for p in unmeasured)
                 + '.')
    L.append('')

    if doc.get('emc_roles') is not None:
        L.extend(_emc_md(doc['emc_roles']))

    L.append('## Parts')
    L.append('')
    for p in doc['parts']:
        body = (f"{p['body_mm'][0]} x {p['body_mm'][1]} mm"
                if p['body_mm'] else 'no body geometry')
        L.append(f"### {p['ref']}  --  {p['role']['role']}"
                 f"  ({p['role']['source']})")
        L.append('')
        L.append(f"`{p['footprint']}`"
                 + (f"  value `{p['value']}`" if p['value'] else '')
                 + f"  --  body {body} from **{p['body_source']}**, "
                 f"{p['pads']} pad(s), side {p['side']}, "
                 f"pose ({p['at'][0]}, {p['at'][1]}, {p['at'][2]}deg)"
                 + ('  LOCKED' if p['locked'] else '')
                 + ('  DNP' if p['dnp'] else ''))
        if p.get('schematic'):
            ps = p['schematic']
            L.append('')
            L.append(f"Schematic: sheet `{ps['sheet'] or '<root>'}`, "
                     + (f"block {ps['block']}." if ps['block']
                        else 'drawn on its own.'))
        if p.get('panel'):
            L.append('')
            L.append(f"![{p['ref']}]({os.path.basename(p['panel'])})")
        if p['role'].get('contradicts_inference'):
            L.append('')
            L.append(f"> The brief declares this a "
                     f"`{p['role']['role']}`; inference off the board says "
                     f"`{p['role']['contradicts_inference']}`. The brief "
                     f"wins, and the disagreement is printed rather than "
                     f"resolved silently.")
        if p.get('mating'):
            m = p['mating']
            L.append('')
            L.append(mating_line(m))
        if p['serves']:
            L.append('')
            L.append(f"Serves **{p['serves']['ref']}** "
                     f"({p['serves']['distance_mm']}mm away).")
        if p['pads_by_face']:
            L.append('')
            for face, pads in p['pads_by_face'].items():
                bits = []
                for q in pads:
                    t = f"{q['pad']} {q['net'] or '-'}"
                    if q['pinfunction']:
                        t += f" ({q['pinfunction']})"
                    if q['reaches']:
                        t += ' -> ' + ' '.join(q['reaches'])
                    bits.append(t)
                L.append(f"- **{face}**: " + ' | '.join(bits))
        if p['partners']:
            L.append('')
            L.append('- partners by shared nets: '
                     + ', '.join(f"{q['ref']} ({q['shared_nets']})"
                                 for q in p['partners'][:8]))
        L.append('')

    L.append('## Sources')
    L.append('')
    for k, v in sorted(doc['sources'].items()):
        L.append(f"- **{k}** -- {v}")
    L.append('')
    return '\n'.join(L)


def write_panels(pcb_data, pcb_file: str, out_dir: str, refs,
                 clearance: float):
    """One cropped PNG per part, through `render_placement`'s own machinery.

    `PanelSpec` + `render_panel` + `union_view` are the same three calls the
    renderer makes for a `--zoom-group` crop; nothing about the drawing is
    re-implemented here, so a panel on this sheet and a panel on a review
    sheet cannot disagree about what the board looks like.

    Returns `{ref: path}` for the panels actually written.
    """
    import render_placement as RP
    os.makedirs(out_dir, exist_ok=True)
    model = RP.PlacementModel(pcb_data, pcb_file,
                              quench_kwargs={'clearance': clearance})
    if model.state is None:
        return {}
    out = {}
    for ref in refs:
        rect = model.rect(ref)
        if rect is None:
            continue
        spec = RP.PanelSpec(model, view=RP.union_view([rect], 2.0),
                            side=model.side(ref), prominent={ref},
                            moves=(), hot_nets=(), blocker_nets=(),
                            pick_nets=(), label=ref,
                            opts=dict(ratsnest=True), defects=())
        img = RP.render_panel(spec, size=700, supersample=2)
        # A ref can carry characters a filesystem will not (`Ref*~2`,
        # `#uuid`), so the name is sanitised and the MAP is what the sheet
        # references -- never a name reconstructed from the ref.
        safe = ''.join(c if (c.isalnum() or c in '-_.') else '_'
                       for c in ref) or 'part'
        path = os.path.join(out_dir, f'{safe}.png')
        img.save(path)
        out[ref] = path
    return out


def build_parser():
    p = argparse.ArgumentParser(
        description='#891: a per-part context sheet a model can reason from.')
    p.add_argument('board', help='the .kicad_pcb to read')
    g = p.add_mutually_exclusive_group()
    g.add_argument('--md', action='store_true',
                   help='markdown to stdout (the default)')
    g.add_argument('--json', action='store_true',
                   help='the same document as JSON on stdout')
    p.add_argument('-o', '--output', help='write to this file instead')
    p.add_argument('--clearance', type=float, default=None,
                   help="override the board's own clearance floor")
    p.add_argument('--track-width', type=float, default=None,
                   help="override the board's own track width")
    sch = p.add_mutually_exclusive_group()
    sch.add_argument('--schematic', metavar='PATH',
                     help="the project's root .kicad_sch (default: "
                          "<board>.kicad_sch beside the board, else the one "
                          ".kicad_pro's)")
    sch.add_argument('--no-schematic', action='store_true',
                     help='read no schematic')
    p.add_argument('--panels', metavar='DIR',
                   help='write one cropped PNG per part into DIR and '
                        'reference them from the sheet')
    try:
        from placement.cli_gates import add_brief_arg
        add_brief_arg(p)
    except Exception:                                        # noqa: BLE001
        pass
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    from kicad_parser import parse_kicad_pcb
    import list_nets
    import routing_defaults as defaults

    pcb = parse_kicad_pcb(args.board)
    clr, _edge, _knobs = list_nets.board_floor_knobs(
        args.board, clearance=args.clearance)
    tw = args.track_width
    if tw is None:
        tw = (list_nets.board_default_netclass_param(
            args.board, 'track_width') or defaults.TRACK_WIDTH)

    brief = None
    try:
        from placement.cli_gates import load_brief_or_exit
        brief, _bpath, code = load_brief_or_exit(args, args.board)
        if code:
            return code
    except Exception:                                        # noqa: BLE001
        brief = None

    # Everything the LIBRARIES print goes to stderr while the document is
    # built. `cli_banner` is not the only thing that can poison a bare-JSON
    # stdout: `parser.warn_missing_courtyards` prints to stdout from inside
    # the placement state, and on esp_prog -- 18 courtyard-less parts -- that alone
    # made `--json` a JSONDecodeError at char 2. The warning is worth keeping;
    # it just is not part of the document.
    with contextlib.redirect_stdout(sys.stderr):
        doc = build_context(pcb, args.board, clearance=clr, track_width=tw,
                            brief=brief,
                            schematic=(False if args.no_schematic
                                       else args.schematic))
    if args.panels:
        try:
            with contextlib.redirect_stdout(sys.stderr):
                panels = write_panels(pcb, args.board, args.panels,
                                      [p['ref'] for p in doc['parts']], clr)
        except Exception as exc:                             # noqa: BLE001
            # Named, never silent: a sheet whose pictures failed must say so,
            # or a reader concludes the parts have nothing worth showing.
            doc['panels_error'] = f'{type(exc).__name__}: {exc}'
            panels = {}
        for part in doc['parts']:
            part['panel'] = panels.get(part['ref'])
        doc['panels_written'] = len(panels)
    text = (json.dumps(doc, indent=1, sort_keys=True) if args.json
            else format_md(doc))
    if args.output:
        with open(args.output, 'w', encoding='utf-8') as fh:
            fh.write(text + '\n')
        print(f'wrote {args.output}', file=sys.stderr)
    else:
        print(text)
    return 0


# NO cli_banner.install() here, deliberately -- `--json` writes a bare
# document to stdout and a `CMD:` line ahead of it is a JSONDecodeError at
# char 0. `board_brief.py` and `converge.py` make the same choice for the same
# reason; converge's comment is the canonical statement of it.
if __name__ == '__main__':
    sys.exit(main())
