"""Reconcile every ref two channels declare, and read `mechanical.json` (#959, #1001).

Before this, three input channels could say where a part belongs and nothing
compared them. Run 29's `mechanical.json` declared USB1 at (117.5, 100.0, 180)
-- the WEST edge -- while its design brief declared USB1 EAST, and
`context.brief.contradictions` reported `[]`: the contradiction check compared
only the brief's edge with the emitter's, the emitter writes `edge: None` for a
classed connector, and nothing read `mechanical.json` at all. The #959
comment's `mechanical_probe` measured the consequence: an absent and a
deliberately contradictory `mechanical.json` produced byte-identical intents.

This module is the channel's reader and its referee. It never decides who is
right on its own authority; it assigns each VALUE an authority from where it
came from, and says which one wins by default:

  declared       the value matches the compiled design brief -- a human wrote it
  recorded_fact  it existed BEFORE the run: the board outline, and a
                 `mechanical.json` the unaided-regime manifest vouches for (or
                 one found with no manifest at all, disclosed as unverified)
  hypothesis     anything the run wrote: a zone plan, an `--intent` file, a
                 `mechanical.json` whose bytes changed after staging. Being
                 serialized gives a guess no authority (#959 comment 3.4)
  inferred       the board re-derives it from a part's current pose
  assumption     a floor `stage_unaided` wrote from the fixed default -- the
                 run chooses its own floors, so this is never a contradiction

A disagreement between two `declared` / `recorded_fact` values is a
CONTRADICTION: P1 refuses until the plan writes why in
`dispositions.contradictions`, because the author decides and the losing
channel may well be the physically right one (it was, in the issue's own
example). A disagreement with anything weaker is DRIFT: the stronger value
wins without anyone having to say so.

Self-labels are never trusted. A zone plan carrying `"source": "brief"` is
still a zone plan.
"""
from __future__ import annotations

import glob
import hashlib
import json
import math
import os
from types import SimpleNamespace
from typing import Dict, List, Optional, Sequence

MECHANICAL_NAME = 'mechanical.json'
_EDGES = ('north', 'south', 'east', 'west')

#: Strongest first. `winner` is the first channel in this order; a tie keeps
#: the channel that was listed first.
AUTHORITY_ORDER = ('declared', 'recorded_fact', 'hypothesis', 'inferred',
                   'assumption')
#: The two authorities whose disagreement is a CONTRADICTION rather than drift.
REFUSAL_AUTHORITIES = ('declared', 'recorded_fact')

#: How far a part may sit from its declared mechanical pose before the pose
#: row reads as drift. Coordinates are stored to the nanometre; stage_unaided
#: rounds to 1e-6 mm. Well above both and well below any placement move.
POSE_TOL_MM = 0.01
ROT_TOL_DEG = 0.01
#: The drift threshold an anchor block grades at (`tolerance_mm`). Explicit,
#: because the intent's 0.5 mm default would hide a move smaller than that.
ANCHOR_TOL_MM = 0.05


class MechanicalError(ValueError):
    """A `mechanical.json` this module cannot read as either known shape."""


def discover_mechanical(board_path: str) -> str:
    """`mechanical.json` in the board's own directory, or ''.

    `stage_unaided` writes it next to `board.kicad_pcb` (not as a stem
    sibling), and the #959 comment's `mechanical_probe` puts it there too."""
    p = os.path.join(os.path.dirname(os.path.abspath(board_path)),
                     MECHANICAL_NAME)
    return p if os.path.isfile(p) else ''


def _sha256(path: str) -> str:
    h = hashlib.sha256()
    with open(path, 'rb') as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def _num(v, where):
    if isinstance(v, bool) or not isinstance(v, (int, float)) \
            or not math.isfinite(float(v)):
        raise MechanicalError(f"{where}: expected a number, got {v!r}")
    return float(v)


def _floors(raw) -> Dict[str, object]:
    """`{'knobs': {knob: {value, source}}, 'unavailable': str|None}`.

    Two spellings: stage_unaided writes `board_floor_knobs`'s dict
    (`{knob: {value, source}}`) or the string `'unavailable: ...'`; the
    declaration form writes `{knob: number}`."""
    if raw is None:
        return {'knobs': {}, 'unavailable': None}
    if isinstance(raw, str):
        return {'knobs': {}, 'unavailable': raw}
    if not isinstance(raw, dict):
        raise MechanicalError(f"project.floors: expected an object or a "
                              f"string, got {type(raw).__name__}")
    knobs = {}
    for k, v in sorted(raw.items()):
        if isinstance(v, dict):
            knobs[str(k)] = {'value': _num(v.get('value'),
                                           f'project.floors.{k}.value'),
                             'source': str(v.get('source') or 'unstated')}
        else:
            knobs[str(k)] = {'value': _num(v, f'project.floors.{k}'),
                             'source': 'declared in mechanical.json'}
    return {'knobs': knobs, 'unavailable': None}


def load_mechanical(path: str) -> Dict[str, object]:
    """Read one `mechanical.json` into one normalised shape.

    Two shapes are accepted, because two exist:

      * stage_unaided's `{schema, kind: 'mechanical-declaration', refs:
        {ref: [x, y, rot]}, reasons, project: {floors}}`;
      * the declaration form the #959 comment's probe wrote,
        `{interfaces: [{ref, edge}], fixed: [{ref, x, y, rot?, reason}],
        project: {floors}}`.

    Anything else raises MechanicalError -- a file named `mechanical.json`
    that says nothing this module can read must not grade as "no mechanical
    facts".
    """
    try:
        with open(path, encoding='utf-8') as fh:
            raw = json.load(fh)
    except (OSError, ValueError) as exc:
        raise MechanicalError(f"{path}: {exc}") from exc
    if not isinstance(raw, dict):
        raise MechanicalError(f"{path}: expected a JSON object")
    poses: Dict[str, Dict[str, object]] = {}
    edges: Dict[str, str] = {}
    if raw.get('kind') == 'mechanical-declaration' or 'refs' in raw:
        shape = 'stage_unaided'
        refs = raw.get('refs')
        if not isinstance(refs, dict):
            raise MechanicalError(f"{path}: `refs` must map ref -> [x, y, rot]")
        reasons = raw.get('reasons') or {}
        for ref, v in sorted(refs.items()):
            if not isinstance(v, (list, tuple)) or len(v) != 3:
                raise MechanicalError(
                    f"{path}: refs.{ref}: expected [x, y, rot], got {v!r}")
            poses[str(ref)] = {
                'x': _num(v[0], f'refs.{ref}[0]'),
                'y': _num(v[1], f'refs.{ref}[1]'),
                'rot': _num(v[2], f'refs.{ref}[2]') % 360.0,
                'reason': str(reasons.get(ref) or '')}
    elif ('interfaces' in raw or 'fixed' in raw) and set(raw) <= {
            'interfaces', 'fixed', 'project', 'note', 'context', 'schema'}:
        shape = 'declaration'
        for i, row in enumerate(raw.get('interfaces') or []):
            if not isinstance(row, dict) or not row.get('ref'):
                raise MechanicalError(
                    f"{path}: interfaces[{i}]: expected {{ref, edge}}")
            edge = row.get('edge')
            if edge not in _EDGES:
                raise MechanicalError(
                    f"{path}: interfaces[{i}] ({row['ref']}): edge {edge!r}, "
                    f"expected one of {', '.join(_EDGES)}")
            edges[str(row['ref'])] = edge
        for i, row in enumerate(raw.get('fixed') or []):
            if not isinstance(row, dict) or not row.get('ref'):
                raise MechanicalError(
                    f"{path}: fixed[{i}]: expected {{ref, x, y, rot?}}")
            rot = row.get('rot')
            poses[str(row['ref'])] = {
                'x': _num(row.get('x'), f'fixed[{i}].x'),
                'y': _num(row.get('y'), f'fixed[{i}].y'),
                # No `rot` leaves the rotation unconstrained, not 0.
                'rot': None if rot is None else _num(
                    rot, f'fixed[{i}].rot') % 360.0,
                'reason': str(row.get('reason') or '')}
    else:
        raise MechanicalError(
            f"{path}: not a mechanical declaration this build reads -- "
            f"expected stage_unaided's `refs` map or `interfaces` / `fixed` "
            f"lists (got keys {sorted(raw)})")
    floors = _floors((raw.get('project') or {}).get('floors')
                     if isinstance(raw.get('project'), dict) else None)
    return {'path': os.path.abspath(path), 'sha256': _sha256(path),
            'shape': shape, 'poses': poses, 'edges': edges,
            'floors': floors}


def mechanical_provenance(mech: Dict, board_path: str):
    """`(status, why, staged_locks)` for one loaded `mechanical.json`.

    status: `verified` (the unaided-regime manifest names this file and its
    sha), `unverified` (no manifest, or one staged before the sha was
    recorded), or `mismatch` (the manifest recorded different bytes -- the
    file changed after staging, so it is the run's own writing and reads as a
    hypothesis). `staged_locks` is the lock set of the staged board the
    manifest names, the only locks that existed before the run, or None."""
    from . import provenance as PV
    wd = PV.regime_for(board_path)
    if wd is None:
        return ('unverified', 'no unaided-regime manifest governs this '
                'board, so nothing vouches for when this file was written',
                None)
    try:
        with open(os.path.join(wd, PV.REGIME_NAME), encoding='utf-8') as fh:
            man = json.load(fh)
    except (OSError, ValueError) as exc:
        return ('unverified', f'the regime manifest is unreadable ({exc})',
                None)
    staged_locks = None
    sb = man.get('staged_board')
    if sb and os.path.isfile(sb):
        try:
            from .parser import extract_locked_refs
            staged_locks = set(extract_locked_refs(sb))
        except (OSError, ValueError):
            staged_locks = None
    mp = man.get('mechanical')
    if not mp or os.path.abspath(mp) != os.path.abspath(mech['path']):
        return ('unverified', 'the regime manifest names a different '
                'mechanical file, or none', staged_locks)
    msha = man.get('mechanical_sha256')
    if not msha:
        return ('unverified', 'the regime manifest records no '
                'mechanical_sha256 (staged before #959)', staged_locks)
    if msha != mech['sha256']:
        return ('mismatch', 'mechanical.json changed after staging: its '
                'sha no longer matches the regime manifest, so it is the '
                "run's own writing", staged_locks)
    return 'verified', 'its sha matches the regime manifest', staged_locks


def _rank(authority: str) -> int:
    try:
        return AUTHORITY_ORDER.index(authority)
    except ValueError:
        return len(AUTHORITY_ORDER)


def _kind(values: Dict[str, Dict[str, object]], same) -> str:
    vals = [v for v in values.values() if v['value'] is not None]
    if len(vals) < 2:
        return 'single'
    first = vals[0]['value']
    if all(same(first, v['value']) for v in vals[1:]):
        return 'agree'
    strong = [v for v in vals if v['authority'] in REFUSAL_AUTHORITIES]
    if len(strong) >= 2 and not all(same(strong[0]['value'], v['value'])
                                    for v in strong[1:]):
        return 'contradiction'
    return 'drift'


def _winner(values: Dict[str, Dict[str, object]]) -> Optional[str]:
    ranked = sorted((ch for ch, v in values.items()
                     if v['value'] is not None),
                    key=lambda ch: _rank(values[ch]['authority']))
    return ranked[0] if ranked else None


def _body_rect_at(pcb, bodies, ref, x=None, y=None, rot=None):
    from .floorplan import drawn_body_rect
    fp = pcb.footprints.get(ref)
    if fp is None:
        return None
    proxy = SimpleNamespace(x=fp.x if x is None else x,
                            y=fp.y if y is None else y,
                            rotation=fp.rotation if rot is None else rot)
    rect, _src = drawn_body_rect(bodies.get(ref), proxy)
    return rect


def _edge_of(rect, bounds) -> Optional[str]:
    if rect is None or bounds is None:
        return None
    from .floorplan import _nearest_edge
    return _nearest_edge(rect, bounds)


def reconcile(pcb, board_path: str, *, brief_fragment: Optional[Dict] = None,
              brief_source: Optional[str] = None,
              mechanical: Optional[Dict] = None,
              intent_doc: Optional[Dict] = None,
              intent_source: Optional[str] = None,
              floors_used: Optional[Dict] = None) -> List[Dict[str, object]]:
    """One row per (ref, field) that TWO OR MORE channels speak to.

    Fields: `edge` (for every ref some channel declares an edge for; the
    mechanical and board values are read off the DRAWN BODY at the declared
    and current pose -- a courtyard or pad bbox reads esp_prog's USB1 as
    south, its body as west), `pose` (mechanical vs board), `on_board` (a
    declared pose whose body is DISJOINT from the outline contradicts the
    outline), and `floors` (mechanical vs the floors the grade used; a
    report, never a contradiction).
    """
    from .body import board_bodies
    try:
        bodies = board_bodies(pcb, board_path)
    except Exception:                                       # noqa: BLE001
        bodies = {}
    bounds = pcb.board_info.board_bounds if pcb.board_info else None
    brief_edges = {str(c.get('ref')): c.get('edge')
                   for c in (brief_fragment or {}).get('edge_connectors') or []
                   if c.get('ref') and c.get('edge')}
    intent_edges = {str(c.get('ref')): c.get('edge')
                    for c in (intent_doc or {}).get('edge_connectors') or []
                    if c.get('ref') and c.get('edge')}
    mech = mechanical or {}
    prov, prov_why, staged_locks = ((None, None, None) if not mech else
                                    mechanical_provenance(mech, board_path))
    mech_auth = ('hypothesis' if prov == 'mismatch' else 'recorded_fact')
    mech_src = (f"{mech.get('path')} ({prov}: {prov_why})"
                if mech else None)
    rows: List[Dict[str, object]] = []

    def _row(ref, field, values, same, why=''):
        kind = _kind(values, same)
        if kind == 'single':
            return
        rows.append({'id': f'{ref}:{field}', 'ref': ref, 'field': field,
                     'values': values, 'kind': kind,
                     'winner': _winner(values), 'why': why})

    declared_edge_refs = sorted(set(brief_edges) | set(intent_edges)
                                | set(mech.get('edges') or {}))
    for ref in declared_edge_refs:
        if ref not in pcb.footprints:
            continue
        values: Dict[str, Dict[str, object]] = {}
        if ref in brief_edges:
            values['brief'] = {'value': brief_edges[ref],
                               'authority': 'declared',
                               'source': brief_source}
        if ref in (mech.get('edges') or {}):
            values['mechanical'] = {'value': mech['edges'][ref],
                                    'authority': mech_auth,
                                    'source': mech_src}
        elif ref in (mech.get('poses') or {}):
            p = mech['poses'][ref]
            values['mechanical'] = {
                'value': _edge_of(_body_rect_at(pcb, bodies, ref, p['x'],
                                                p['y'], p['rot']), bounds),
                'authority': mech_auth, 'source': mech_src}
        if ref in intent_edges:
            ie = intent_edges[ref]
            values['intent'] = {
                'value': ie,
                # Per value: an intent carrying the brief's own value is
                # carrying a declaration; anything else is the run's guess.
                'authority': ('declared' if brief_edges.get(ref) == ie
                              else 'hypothesis'),
                'source': intent_source}
        values['board'] = {'value': _edge_of(_body_rect_at(pcb, bodies, ref),
                                             bounds),
                           'authority': 'inferred', 'source': board_path}
        _row(ref, 'edge', values, lambda a, b: a == b,
             'the edge each channel puts this part on, read off the drawn '
             'body')

    def _same_pose(a, b):
        return (abs(a[0] - b[0]) <= POSE_TOL_MM
                and abs(a[1] - b[1]) <= POSE_TOL_MM
                and (a[2] is None or b[2] is None
                     or min(abs(a[2] - b[2]) % 360.0,
                            360.0 - abs(a[2] - b[2]) % 360.0) <= ROT_TOL_DEG))

    for ref, p in sorted((mech.get('poses') or {}).items()):
        fp = pcb.footprints.get(ref)
        if fp is None:
            continue
        board_auth = ('recorded_fact' if staged_locks and ref in staged_locks
                      else 'hypothesis' if getattr(fp, 'locked', False)
                      else 'inferred')
        values = {
            'mechanical': {'value': (p['x'], p['y'], p['rot']),
                           'authority': mech_auth, 'source': mech_src},
            'board': {'value': (fp.x, fp.y, (fp.rotation or 0.0) % 360.0),
                      'authority': board_auth, 'source': board_path}}
        _row(ref, 'pose', values, _same_pose,
             p.get('reason') or 'a declared mechanical pose')
        rect = _body_rect_at(pcb, bodies, ref, p['x'], p['y'], p['rot'])
        if rect is not None and bounds is not None and (
                rect[2] < bounds[0] or rect[0] > bounds[2]
                or rect[3] < bounds[1] or rect[1] > bounds[3]):
            rows.append({
                'id': f'{ref}:on_board', 'ref': ref, 'field': 'on_board',
                'values': {
                    'mechanical': {'value': [round(v, 4) for v in rect],
                                   'authority': mech_auth,
                                   'source': mech_src},
                    'outline': {'value': [round(v, 4) for v in bounds],
                                'authority': 'recorded_fact',
                                'source': board_path}},
                'kind': ('contradiction' if mech_auth == 'recorded_fact'
                         else 'drift'),
                'winner': 'outline',
                'why': 'the declared pose puts the body entirely off the '
                       'board outline'})

    knobs = ((mech.get('floors') or {}).get('knobs') or {})
    unavailable = (mech.get('floors') or {}).get('unavailable')
    if (knobs or unavailable) and floors_used:
        for knob in sorted(set(knobs) | set(floors_used)):
            m = knobs.get(knob)
            u = floors_used.get(knob)
            if m is None and u is None:
                continue
            values = {}
            if m is not None or unavailable:
                values['mechanical'] = {
                    'value': None if m is None else m['value'],
                    'authority': 'assumption',
                    'source': (f"{mech_src}; {m['source']}" if m
                               else f"{mech_src}; {unavailable}")}
            if u is not None:
                values['graded'] = {
                    'value': u.get('value'),
                    'authority': ('recorded_fact'
                                  if u.get('source') in ('board netclass',
                                                         'board constraint')
                                  else 'hypothesis'
                                  if u.get('source') == 'cli'
                                  else 'assumption'),
                    'source': u.get('source')}
            vals = [v['value'] for v in values.values()
                    if v['value'] is not None]
            rows.append({
                'id': f'floors:{knob}', 'ref': None, 'field': f'floors.{knob}',
                'values': values,
                'kind': ('agree' if len(vals) > 1 and len(set(vals)) == 1
                         else 'report'),
                'winner': 'graded' if u is not None else None,
                'why': ('staging recorded this floor as an assumption; the '
                        'grade used the value named `graded`')})
    return rows


def contradictions(rows) -> List[Dict[str, object]]:
    return [r for r in rows if r['kind'] == 'contradiction']


def lost_mechanical_refs(rows) -> List[str]:
    """Mechanical refs whose declared value LOST a contradiction: compiling
    them into anchors would make the intent say both things at once (brief
    east plus an anchor west), which no placement can satisfy."""
    return sorted({r['ref'] for r in rows
                   if r['kind'] == 'contradiction' and r['ref']
                   and 'mechanical' in r['values']
                   and r['winner'] != 'mechanical'})


def format_rows(rows) -> List[str]:
    lines = []
    for r in rows:
        if r['kind'] in ('agree', 'single'):
            continue
        vals = '; '.join(
            f"{ch} {v['value']!r} [{v['authority']}]"
            for ch, v in r['values'].items() if v['value'] is not None)
        lines.append(f"  {r['kind'].upper()} {r['id']}: {vals} -> "
                     f"{r['winner'] or 'nobody'} wins")
    return lines


def anchor_blocks(pcb, board_path: str, mechanical: Dict, *,
                  lost: Sequence[str] = (), state=None,
                  tolerance_mm: float = ANCHOR_TOL_MM):
    """`(blocks, skipped)`: one grade-only anchor block per mechanical ref.

    The zone is the GRADER's own rect for the part at its declared pose --
    `_Part.rect`, which is what `rule_zone_containment` tests -- unioned with
    the exactly rotated rect (the grader falls back to its 0-degree bounds
    off the 90-degree lattice), rounded outward. So a part sitting at its
    declared pose grades clean and one that moved does not.

    Grade-only: P1 requires each anchored ref to be FILE-locked, so the
    seeder treats it as placed and never seats it here -- measured before
    this was built, the seeder cannot seat a part at an exact pose (a part
    overhanging the outline has no admissible pose at all; edge claims are
    seated before zones; a seat lands anywhere within the tolerance).
    No `rotation` key: rotation drift is graded by `mechanical_drift`, and a
    rotation DECISION on a block would collide with a plan's own claims.

    Skipped, with the reason: pad-less refs (the seeder never places them;
    they are reconciled only), refs the board does not have, and refs whose
    mechanical value lost a contradiction.
    """
    from .legality import rotate_local_bounds
    blocks, skipped = [], {}
    if state is None:
        import pose_score
        state = pose_score.make_state(pcb, board_path)
    # `state.parts` holds the `_Part` objects -- the ones whose `.rect(x, y,
    # rot)` the grader's own rects come from -- not the frozen-pose records
    # `graded_parts()` returns.
    parts = state.parts
    for ref, p in sorted((mechanical.get('poses') or {}).items()):
        fp = pcb.footprints.get(ref)
        if fp is None:
            skipped[ref] = 'not on this board'
            continue
        if not fp.pads:
            skipped[ref] = ('pad-less: the seeder never places it, so it is '
                            'reconciled rather than anchored')
            continue
        if ref in lost:
            skipped[ref] = ('its mechanical value lost a contradiction; an '
                            'anchor would contradict the winning declaration')
            continue
        part = parts.get(ref)
        if part is None:
            skipped[ref] = 'the placement state carries no geometry for it'
            continue
        rot = p['rot'] if p['rot'] is not None else (fp.rotation or 0.0)
        g = part.rect(p['x'], p['y'], rot)
        b0 = part.bounds_by_rot[0.0]
        e = rotate_local_bounds(*b0, rot)
        exact = (p['x'] + e[0], p['y'] + e[1], p['x'] + e[2], p['y'] + e[3])
        zone = [math.floor(min(g[0], exact[0]) * 1e4) / 1e4,
                math.floor(min(g[1], exact[1]) * 1e4) / 1e4,
                math.ceil(max(g[2], exact[2]) * 1e4) / 1e4,
                math.ceil(max(g[3], exact[3]) * 1e4) / 1e4]
        blocks.append({
            'name': f'mech:{ref}', 'refs': [glob.escape(ref)], 'zone': zone,
            'tolerance_mm': tolerance_mm,
            'note': ('mechanical.json: '
                     + (p.get('reason') or 'a declared mechanical pose')),
            'context': {'basis': 'mechanical',
                        'mechanical_pose': [p['x'], p['y'], p['rot']],
                        'source': mechanical.get('path')}})
    return blocks, skipped
