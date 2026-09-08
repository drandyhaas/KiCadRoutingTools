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
    part class             placement.part_class.classify_part
    decap partner          placement.groups (_elect_tethers / decap_populations)
    diff pairs             list_nets.find_differential_pairs (resonator-safe)
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

import _path  # noqa: F401
import argparse
import contextlib
import json
import os
import sys
from typing import Dict, List, Optional

SCHEMA = 1


# ---------------------------------------------------------------------------
# Roles. Deliberately NOT part of `part_class.classify_part`.
#
# `grade_body_overlap._waiver_for` branches on that classifier's `.name`
# against its marker and edge tables, so adding a role vocabulary there would
# silently change waivers on the body-overlap channel. These live beside it and
# read it, never inside it.
# ---------------------------------------------------------------------------

#: (role, evidence-kind) by footprint-name substring. Lower-cased match.
_ROLE_FP = (
    ('crystal', ('xtal', 'crystal', 'resonator', 'oscillator')),
    ('usb receptacle', ('usb_micro', 'usb-micro', 'usb_c', 'usb_b', 'usb_a',
                        'uicro')),
    ('pin header', ('pinheader', 'hn1x', 'socket_strip', 'header')),
    ('npn/pnp transistor', ('sot23', 'sot-23', 'sot323', 'to92')),
    ('regulator', ('sot89', 'sot-89', 'sot223', 'to252', 'dpak')),
    ('mounting hole', ('mountinghole',)),
)

#: role by reference prefix, used only when nothing stronger fired.
_ROLE_PREFIX = {
    'C': 'capacitor', 'R': 'resistor', 'L': 'inductor', 'D': 'diode',
    'Q': 'transistor', 'U': 'integrated circuit', 'Y': 'crystal',
    'X': 'crystal', 'J': 'connector', 'SW': 'switch', 'FB': 'ferrite bead',
    'TP': 'test point', 'FID': 'fiducial', 'LED': 'led',
}


def _prefix(ref: str) -> str:
    out = ''
    for ch in ref:
        if ch.isalpha() or ch in '#*':
            out += ch
        else:
            break
    return out.upper()


def infer_role(fp, ref: str, cls_name: Optional[str]) -> Dict[str, object]:
    """`{'role', 'source', 'evidence'}` -- or role `'unknown'`.

    Ordered strongest-first, and each rung names itself. `unknown` is returned
    rather than a prefix guess when the reference carries no convention this
    table knows: "I looked and could not tell" is a different statement from a
    low-confidence guess, and a reader who cannot tell them apart will trust
    the guess.
    """
    name = (getattr(fp, 'footprint_name', '') or '').lower()
    value = (getattr(fp, 'value', '') or '').strip()
    pref = _prefix(ref)

    if cls_name in ('mount_hole', 'fiducial', 'testpoint', 'edge_receptacle',
                    'edge_actuator'):
        return {'role': cls_name.replace('_', ' '), 'source': 'part_class',
                'evidence': [f'placement.part_class says {cls_name}']}
    for role, keys in _ROLE_FP:
        hit = next((k for k in keys if k in name), None)
        if hit:
            return {'role': role, 'source': 'footprint',
                    'evidence': [f'footprint name contains {hit!r}']}
    if pref in _ROLE_PREFIX:
        ev = [f'reference prefix {pref!r} (convention only)']
        if value:
            ev.append(f'value {value!r}')
        return {'role': _ROLE_PREFIX[pref], 'source': 'ref_prefix',
                'evidence': ev}
    return {'role': 'unknown', 'source': 'looked',
            'evidence': [f'footprint {name!r}, value {value!r}, prefix '
                         f'{pref!r} match no rule here']}


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
    # the quench's pad-box ladder, which this PR deliberately does not
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
        edge, dist = min(dists.items(), key=lambda kv: kv[1])
        out[ref] = {'edge': edge,
                    'dist_mm': round(max(0.0, dist), 3),
                    'interior': bool(dist > INTERIOR_AFFINITY_MM),
                    'overhang_mm': round(max(0.0, -dist), 3),
                    'class': _cls, 'basis': geom.source}
    return out


def build_context(pcb_data, pcb_file: str, *, clearance: float,
                  track_width: float, brief=None) -> Dict[str, object]:
    """The whole document, as data. `format_md` renders it."""
    from placement.body import board_bodies
    from placement.legality import footprint_side, rotate_local_bounds
    from placement.part_class import classify_part
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
    # `serves` is the tether election RESTRICTED to the near population.
    # The raw election is radius-free and always names SOMETHING: on esp_prog
    # it puts C1 17.24mm from the USB socket, which is not a fact about what
    # C1 serves. #902 is the issue for the election itself (a 3-pad regulator
    # can never be a target, `DECAP_MIN_IC_PADS = 4`); until then this column
    # reports only what is within the module's own radius and says nothing
    # otherwise.
    tethers = {}
    try:
        from placement.groups import DECAP_RADIUS_MM
        for cap, ic, dist in _elect(pcb_data):
            if ic and dist is not None and dist <= DECAP_RADIUS_MM:
                tethers[cap] = (ic, dist)
    except Exception:                                        # noqa: BLE001
        tethers = {}

    declared_roles = {}
    if brief is not None:
        for row in (getattr(brief, 'raw', None) or {}).get('interfaces', []):
            if isinstance(row, dict) and row.get('ref') and row.get('role'):
                declared_roles[row['ref']] = row['role']

    try:
        faces_out = mating_faces(pcb_data, pcb_file, clearance)
    except Exception:                                        # noqa: BLE001
        faces_out = {}

    parts = []
    for ref, fp in sorted((pcb_data.footprints or {}).items()):
        geom = bodies.get(ref)
        cls = classify_part(fp, ref)
        role = infer_role(fp, ref, getattr(cls, 'name', None))
        if ref in declared_roles:
            inferred = role['role']
            role = {'role': declared_roles[ref], 'source': 'brief',
                    'evidence': [f'declared in the design brief']}
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
            'pads': len(fp.pads or ()),
            'side': footprint_side(fp),
            'at': [round(fp.x, 3), round(fp.y, 3),
                   round(fp.rotation or 0.0, 3)],
            'locked': bool(getattr(fp, 'locked', False)),
            'dnp': bool(getattr(fp, 'dnp', False)),
            'pads_by_face': side_pads,
            'partners': [{'ref': r, 'shared_nets': n} for n, r in partners],
            'mating': faces_out.get(ref),
            'serves': ({'ref': tethers[ref][0],
                        'distance_mm': round(tethers[ref][1], 3)}
                       if ref in tethers and tethers[ref][0] else None),
        })

    return {
        'schema': SCHEMA,
        'board': os.path.basename(pcb_file),
        'bounds': list(pcb_data.board_info.board_bounds or ()),
        'copper_layers': list(pcb_data.board_info.copper_layers or ()),
        'floors': {'clearance': clearance, 'track_width': track_width},
        # `find_differential_pairs` returns (positive, negative) tuples.
        'diff_pairs': [list(t) for t in sorted(pairs or ())],
        'parts': parts,
        'pin_order': pin_order_rows(pcb_data, pcb_file, clearance),
        'sources': {
            'body_mm / body_source': 'placement.body.board_bodies (#896)',
            'pads_by_face': 'placement.escape.assign_faces (#850)',
            'pin_order': 'placement.pair_order.pair_metrics',
            'part_class': 'placement.part_class.classify_part',
            'serves': 'placement.groups tether election '
                      '(DECAP_MIN_IC_PADS = 4, so a 3-pad regulator can '
                      'never be a target -- see #902)',
            'diff_pairs': 'list_nets.find_differential_pairs '
                          '(rejects 2-terminal resonators)',
            'role': 'inferred here from footprint / prefix / value; the '
                    'board carries no datasheet or 3D-model field to cite',
            'mating': 'which parts count as connectors: '
                      'render_placement.connector_edge_facts. Where the edge '
                      'is: measured from the placement.body occupancy rect, '
                      'because that function reads the quench pad-box ladder '
                      'and would put two geometries on one sheet',
        },
    }


def _elect(pcb_data):
    from placement.groups import _elect_tethers      # noqa: PLC2701
    return _elect_tethers(pcb_data)


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
        # drifted once (render vs place_optimize, before #431 fixed it).
        from render_placement import PlacementModel
        model = PlacementModel(pcb_data, pcb_file,
                               quench_kwargs={'clearance': clearance})
        if model.state is None:
            return {'error': 'the quench state could not be built', 'rows': []}
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
    try:
        from placement.pair_order import pair_metrics
        import list_nets
        name_to_id = {}
        for ref, fp in (pcb_data.footprints or {}).items():
            for pad in (fp.pads or ()):
                nm = getattr(pad, 'net_name', '') or ''
                if nm:
                    name_to_id.setdefault(nm, getattr(pad, 'net_id', 0) or 0)
        for pos, neg in (list_nets.find_differential_pairs(pcb_data) or ()):
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
                    rows.append({
                        'a': a, 'b': b, 'nets': m['nets'],
                        'scope': f'pair {pos}/{neg}',
                        'inversions': m['inversions'], 'lis': m['lis'],
                        'ties': m.get('ties', 0), 'verdict': _verdict(m),
                        'span_mm': _pair_span_mm(pcb_data, a, b,
                                                 only_nets=ids),
                    })
    except Exception as exc:                                 # noqa: BLE001
        rows.append({'a': '-', 'b': '-', 'nets': 0, 'scope': 'pair',
                     'inversions': None, 'lis': None, 'span_mm': None,
                     'verdict': f'NOT MEASURED ({type(exc).__name__})'})

    # Pair rows first: they are the narrow, unfixable-by-rotation claim.
    rows.sort(key=lambda r: (r['scope'] == 'interface',
                             -(r['inversions'] or 0), r['a'], r['b']))
    return {'error': None, 'rows': rows}


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
    if doc['diff_pairs']:
        L.append('Differential pairs: '
                 + ', '.join('/'.join(t) for t in doc['diff_pairs']) + '.')
    L.append('')
    L.append('Every derived column names its source; `unknown` means the tool '
             'looked and could not tell, which is not the same as absent. '
             'See the SOURCES section at the end.')
    L.append('')

    po = doc['pin_order']
    L.append('## Pin-order agreement')
    L.append('')
    if po.get('error'):
        L.append(f"NOT MEASURED ({po['error']}) -- this sheet cannot say "
                 f"whether any pair's order crosses.")
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
    L.append('')

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
            L.append(f"Mating face **{m['edge']}**, {m['dist_mm']}mm from that "
                     f"edge"
                     + (f", overhanging it by {m['overhang_mm']}mm"
                        if m['overhang_mm'] > 0 else '')
                     + ('  --  INTERIOR for a connector, which a reviewer '
                        'should explain' if m['interior'] else ''))
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
    # the quench, and on esp_prog -- 18 courtyard-less parts -- that alone
    # made `--json` a JSONDecodeError at char 2. The warning is worth keeping;
    # it just is not part of the document.
    with contextlib.redirect_stdout(sys.stderr):
        doc = build_context(pcb, args.board, clearance=clr, track_width=tw,
                            brief=brief)
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
