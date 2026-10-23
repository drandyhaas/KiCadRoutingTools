"""Reconcile every ref two channels declare, and read `mechanical.json`.

Three input channels can say where a part belongs -- the design brief, the
intent, `mechanical.json` -- and the board says where it is. This module reads
`mechanical.json` and compares them. It never decides who is right on its own
authority; it assigns each VALUE an authority from where it came from, and
says which one wins by default:

  declared       the value matches the compiled design brief -- a human wrote it
  recorded_fact  it existed before the run: the board outline, and a
                 `mechanical.json` beside the board
  hypothesis     anything the run wrote: a zone plan, an `--intent` file, a
                 lock on the board (nothing records whether it predates the
                 run). Being serialized gives a guess no authority
  inferred       the board re-derives it from a part's current pose
  assumption     a floor `mechanical.json` records -- the run chooses its own
                 floors, so this is never a contradiction

A disagreement between two `declared` / `recorded_fact` values is a
CONTRADICTION: `check_floorplan` reports it until the plan acknowledges it in
`dispositions.contradictions`. A disposition ACCEPTS the winner the row names
-- it does not flip it. A plan is not a declaration and cannot overrule one;
to make the other value hold, correct the source that is wrong (the brief,
`mechanical.json`, or the board). A disagreement with anything weaker is
DRIFT: the stronger value wins without anyone having to say so.

Mechanical ANCHORS are compiled at GRADE time from the file itself
(`floorplan.mechanical_anchor_violations`), never read out of a plan: a plan
block that could label itself an anchor could exempt itself from the envelope
and overlap checks.

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
#: Every top-level key the declaration form carries: `interfaces`, `fixed`
#: and `project`, plus free-form `schema`, `note` and `context`.
_MECH_KEYS = frozenset({'schema', 'interfaces', 'fixed', 'project', 'note',
                        'context'})

#: Strongest first. `winner` is the first channel in this order; a tie keeps
#: the channel that was listed first.
AUTHORITY_ORDER = ('declared', 'recorded_fact', 'hypothesis', 'inferred',
                   'assumption')
#: The two authorities whose disagreement is a CONTRADICTION rather than drift.
REFUSAL_AUTHORITIES = ('declared', 'recorded_fact')

#: How far a part may sit from its declared mechanical pose before the pose
#: row reads as drift. Coordinates are stored to the nanometre. Well above
#: that and well below any placement move.
POSE_TOL_MM = 0.01
ROT_TOL_DEG = 0.01
#: The drift threshold an anchor block grades at (`tolerance_mm`). Explicit,
#: because the intent's 0.5 mm default would hide a move smaller than that.
ANCHOR_TOL_MM = 0.05


class MechanicalError(ValueError):
    """A `mechanical.json` this module cannot read."""


def discover_mechanical(board_path: str) -> str:
    """`mechanical.json` in the board's own directory (not a stem sibling),
    or ''."""
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


def _floors(raw) -> Dict[str, float]:
    """`{knob: value}` from `project.floors` (`{knob: number}`)."""
    if raw is None:
        return {}
    if not isinstance(raw, dict):
        raise MechanicalError(f"project.floors: expected an object, got "
                              f"{type(raw).__name__}")
    return {str(k): _num(v, f'project.floors.{k}')
            for k, v in sorted(raw.items())}


def load_mechanical(path: str) -> Dict[str, object]:
    """Read one `mechanical.json`: `{interfaces: [{ref, edge}], fixed: [{ref,
    x, y, rot?, reason}], project: {floors: {knob: number}}}`.

    Anything else raises MechanicalError -- a file named `mechanical.json`
    that says nothing this module can read must not grade as "no mechanical
    facts". That includes a file whose sections are all empty, an unknown key,
    and a malformed entry in ANY section: an `interfaces` row with
    `edge: "up"` is refused, never skipped.
    """
    try:
        with open(path, encoding='utf-8') as fh:
            raw = json.load(fh)
    except (OSError, ValueError) as exc:
        raise MechanicalError(f"{path}: {exc}") from exc
    if not isinstance(raw, dict):
        raise MechanicalError(f"{path}: expected a JSON object")
    if not ({'interfaces', 'fixed'} & set(raw)):
        raise MechanicalError(
            f"{path}: not a mechanical declaration this build reads -- "
            f"expected `interfaces` / `fixed` lists (got keys {sorted(raw)})")
    unknown = sorted(set(raw) - _MECH_KEYS)
    if unknown:
        raise MechanicalError(
            f"{path}: unknown key(s) {unknown}; a mechanical declaration "
            f"carries only {sorted(_MECH_KEYS)}")
    poses: Dict[str, Dict[str, object]] = {}
    edges: Dict[str, str] = {}
    for key in ('interfaces', 'fixed'):
        if key in raw and not isinstance(raw[key], list):
            raise MechanicalError(f"{path}: `{key}` must be a list")
    for i, row in enumerate(raw.get('interfaces') or []):
        if not isinstance(row, dict) or not row.get('ref'):
            raise MechanicalError(
                f"{path}: interfaces[{i}]: expected {{ref, edge}}")
        edge = row.get('edge')
        if edge not in _EDGES:
            raise MechanicalError(
                f"{path}: interfaces[{i}] ({row['ref']}): edge {edge!r}, "
                f"expected one of {', '.join(_EDGES)}")
        if str(row['ref']) in edges:
            raise MechanicalError(
                f"{path}: interfaces[{i}]: {row['ref']} is declared twice")
        edges[str(row['ref'])] = edge
    for i, row in enumerate(raw.get('fixed') or []):
        if not isinstance(row, dict) or not row.get('ref'):
            raise MechanicalError(
                f"{path}: fixed[{i}]: expected {{ref, x, y, rot?}}")
        if str(row['ref']) in poses:
            raise MechanicalError(
                f"{path}: fixed[{i}]: {row['ref']} already has a declared "
                f"pose -- one ref, one pose")
        rot = row.get('rot')
        poses[str(row['ref'])] = {
            'x': _num(row.get('x'), f'fixed[{i}].x'),
            'y': _num(row.get('y'), f'fixed[{i}].y'),
            # No `rot` leaves the rotation unconstrained, not 0.
            'rot': None if rot is None else _num(
                rot, f'fixed[{i}].rot') % 360.0,
            'reason': str(row.get('reason') or '')}
    if not poses and not edges:
        raise MechanicalError(
            f"{path}: declares no pose and no edge -- a mechanical "
            f"declaration that says nothing would grade as 'no mechanical "
            f"facts'. Delete it, or pass --no-mechanical")
    floors = _floors((raw.get('project') or {}).get('floors')
                     if isinstance(raw.get('project'), dict) else None)
    return {'path': os.path.abspath(path), 'sha256': _sha256(path),
            'poses': poses, 'edges': edges,
            'floors': floors}


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


class _Geometry:
    """The board's bodies and placement state, built on first use: a
    reconciliation with nothing to compare must not pay for either (the
    Phase-3 verifier measured 0.55 s on ulx3s for an empty one)."""

    def __init__(self, pcb, board_path, floors_used=None):
        self.pcb, self.board_path = pcb, board_path
        self._bodies = self._state = None
        # the floors the grade beside this read at (check_floorplan's
        # `knobs`), so an edge read here is the grade's; else the board's
        knobs = floors_used or {}
        self._floors = tuple((knobs.get(k) or {}).get('value')
                             for k in ('clearance', 'board_edge_clearance'))

    @property
    def bodies(self):
        if self._bodies is None:
            from .body import board_bodies
            try:
                self._bodies = board_bodies(self.pcb, self.board_path)
            except Exception:                               # noqa: BLE001
                self._bodies = {}
        return self._bodies

    @property
    def state(self):
        if self._state is None:
            import pose_score
            try:
                self._state = pose_score.make_state(
                    self.pcb, self.board_path, clearance=self._floors[0],
                    board_edge_clearance=self._floors[1])
            except ValueError as exc:
                # No outline: there is no edge to read a part against. The
                # grade refuses such a board itself, at exit 3 --
                # reconciliation, which runs first, must not turn that into
                # a traceback (pre-push review: exit 1 where the base exited
                # 3). Only THAT error: any other would silently drop the
                # edge rows (and a contradiction on them).
                if 'board boundary' not in str(exc):
                    raise
                self._state = False
        return self._state or None

    def _proxy(self, ref, x, y, rot):
        fp = self.pcb.footprints.get(ref)
        if fp is None:
            return None
        return SimpleNamespace(x=fp.x if x is None else x,
                               y=fp.y if y is None else y,
                               rotation=((fp.rotation or 0.0) if rot is None
                                         else rot))

    def body_rect(self, ref, x=None, y=None, rot=None):
        """The DRAWN body (fab/silk, else courtyard) at a pose."""
        from .floorplan import drawn_body_rect
        proxy = self._proxy(ref, x, y, rot)
        if proxy is None:
            return None
        return drawn_body_rect(self.bodies.get(ref), proxy)[0]

    def seat_rect(self, ref, entry, x=None, y=None, rot=None):
        """The rect `rule_edge_connector` reads a part's edge off, at a pose:
        `edge_seat_rect` itself -- the drawn body for an edge receptacle or an
        `edge_mount` entry, the courtyard otherwise. Reading the drawn body
        for every part disagreed with the grader on three corpus refs, and
        manufactured a contradiction on rp2350's SW1 that the grade passed."""
        from .floorplan import drawn_body_rect, edge_seat_rect
        proxy = self._proxy(ref, x, y, rot)
        st = self.state
        part = st.parts.get(ref) if st is not None else None
        if proxy is None or part is None:
            return None
        rect, _basis = edge_seat_rect(
            entry or {}, part.rect(proxy.x, proxy.y, proxy.rotation),
            lambda: drawn_body_rect(self.bodies.get(ref), proxy))
        return rect


def _edge_of(rect, bounds, geo=None, ref=None, rot=None) -> Optional[str]:
    """The edge the grade reads `rect` on (`floorplan._nearest_edge`, at
    the state's `edge_reach_mm`, `ref`'s mating face at `rot` -- its board
    turn when None -- breaking a tie between two sides it reaches)."""
    if rect is None or bounds is None:
        return None
    from .floorplan import _nearest_edge, edge_facing, edge_reach_mm
    st = geo.state if geo is not None else None

    def facing():
        fp = geo.pcb.footprints.get(ref) if geo is not None and ref else None
        if fp is None:
            return None
        turn = rot if isinstance(rot, (int, float)) else None
        return edge_facing({}, fp, geo.bodies.get(ref), rotation=turn)['facing']
    return _nearest_edge(rect, bounds, edge_reach_mm(st) if st else 0.0,
                         facing=facing)


def same_pose(a, b) -> bool:
    """Two `(x, y, rot)` poses within `POSE_TOL_MM` / `ROT_TOL_DEG`; a None
    rotation constrains nothing. The one comparison the pose row and the
    fixed-pose grade share."""
    return (abs(a[0] - b[0]) <= POSE_TOL_MM
            and abs(a[1] - b[1]) <= POSE_TOL_MM
            and (a[2] is None or b[2] is None
                 or min(abs(a[2] - b[2]) % 360.0,
                        360.0 - abs(a[2] - b[2]) % 360.0) <= ROT_TOL_DEG))


def fixed_pose_tuple(entry: Dict):
    """An intent `fixed_poses[]` entry as `(x, y, rot)`, rot None when absent
    or "unknown"."""
    rot = entry.get('rot')
    return (float(entry['x']), float(entry['y']),
            None if rot is None or rot == 'unknown' else float(rot) % 360.0)


def reconcile(pcb, board_path: str, *, brief_fragment: Optional[Dict] = None,
              brief_source: Optional[str] = None,
              mechanical: Optional[Dict] = None,
              intent_doc: Optional[Dict] = None,
              intent_source: Optional[str] = None,
              floors_used: Optional[Dict] = None) -> List[Dict[str, object]]:
    """One row per (ref, field) that TWO OR MORE channels speak to.

    Fields: `edge` (for every ref some channel declares an edge for; the
    mechanical and board values are read off `edge_seat_rect` at the declared
    and current pose -- the rect the grader itself reads, so reconciliation
    and the grade cannot disagree about which edge a part is on), `pose`
    (mechanical vs board), `on_board` (a declared pose whose drawn body is
    DISJOINT from the outline's bounding box contradicts the outline), and
    `floors` (mechanical vs the floors the grade used; a report, never a
    contradiction). A row whose mechanical value lost ANOTHER row's
    contradiction does not name mechanical its winner.
    """
    geo = _Geometry(pcb, board_path, floors_used)
    bounds = pcb.board_info.board_bounds if pcb.board_info else None
    brief_entries = {str(c.get('ref')): c
                     for c in (brief_fragment or {}).get('edge_connectors')
                     or [] if c.get('ref') and c.get('edge')}
    brief_edges = {r: c.get('edge') for r, c in brief_entries.items()}
    intent_entries = {str(c.get('ref')): c
                      for c in (intent_doc or {}).get('edge_connectors')
                      or [] if c.get('ref') and c.get('edge')}
    intent_edges = {r: c.get('edge') for r, c in intent_entries.items()}
    brief_auth = 'declared'
    if brief_source and os.path.isfile(brief_source):
        brief_source = f"{brief_source} (sha256 {_sha256(brief_source)})"
    mech = mechanical or {}
    mech_auth = 'recorded_fact'
    mech_src = (f"{mech.get('path')} (sha256 {mech.get('sha256')})"
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
        # The entry the GRADER would read this ref's edge by: the brief's
        # when it declares one, else the plan's, else none (courtyard).
        entry = brief_entries.get(ref) or intent_entries.get(ref)
        if ref in brief_edges:
            values['brief'] = {'value': brief_edges[ref],
                               'authority': brief_auth,
                               'source': brief_source}
        if ref in (mech.get('edges') or {}):
            values['mechanical'] = {'value': mech['edges'][ref],
                                    'authority': mech_auth,
                                    'source': mech_src}
        elif ref in (mech.get('poses') or {}):
            p = mech['poses'][ref]
            values['mechanical'] = {
                'value': _edge_of(geo.seat_rect(ref, entry, p['x'], p['y'],
                                                p['rot']), bounds, geo, ref,
                                  p['rot']),
                'authority': mech_auth, 'source': mech_src}
        if ref in intent_edges:
            ie = intent_edges[ref]
            values['intent'] = {
                'value': ie,
                # Per value: an intent carrying the brief's own value is
                # carrying a declaration; anything else is the run's guess.
                'authority': (brief_auth if brief_edges.get(ref) == ie
                              else 'hypothesis'),
                'source': intent_source}
        values['board'] = {'value': _edge_of(geo.seat_rect(ref, entry),
                                             bounds, geo, ref),
                           'authority': 'inferred', 'source': board_path}
        _row(ref, 'edge', values, lambda a, b: a == b,
             'the edge each channel puts this part on, read off the rect the '
             'edge-connector rule reads (`edge_seat_rect`)')

    _same_pose = same_pose
    # #1054: a brief `fixed[].pose` is a THIRD channel on the pose row. A
    # brief pose that disagrees with mechanical.json is a contradiction,
    # rather than a second anchor nobody reconciled.
    brief_poses = {str(f.get('ref')): f for f in
                   (brief_fragment or {}).get('fixed_poses') or ()}

    for ref, p in sorted((mech.get('poses') or {}).items()):
        fp = pcb.footprints.get(ref)
        if fp is None:
            continue
        # A lock on the board may be the run's own writing -- nothing records
        # whether it predates the run -- so a locked pose is a hypothesis and
        # an unlocked one is inferred.
        cur = (fp.x, fp.y, (fp.rotation or 0.0) % 360.0)
        locked = getattr(fp, 'locked', False)
        board_auth = 'hypothesis' if locked else 'inferred'
        values = {
            'mechanical': {'value': (p['x'], p['y'], p['rot']),
                           'authority': mech_auth, 'source': mech_src},
            'board': {'value': cur, 'authority': board_auth,
                      'source': board_path}}
        if ref in brief_poses:
            values['brief'] = {'value': fixed_pose_tuple(brief_poses[ref]),
                               'authority': brief_auth,
                               'source': brief_source}
        _row(ref, 'pose', values, _same_pose,
             p.get('reason') or 'a declared mechanical pose')
        # A brief pose that AGREES with mechanical.json corroborates it; it
        # does not replace it. The pose is a physical fact recorded in the
        # file, so the row keeps `mechanical` as its winner (and the ledger
        # its basis) and names the brief as corroborating -- rather than
        # letting the brief's higher authority rank relabel a recorded fact
        # as a declaration (Phase-1 re-verifier). Only a DISAGREEING brief
        # wins, as a contradiction.
        if ('brief' in values and rows and rows[-1]['id'] == f'{ref}:pose'
                and _same_pose(values['brief']['value'],
                               values['mechanical']['value'])
                and rows[-1]['winner'] == 'brief'):
            rows[-1]['winner'] = 'mechanical'
            rows[-1]['why'] = (f"{rows[-1]['why']} -- the design brief "
                               f"declares the same pose, corroborating it")
        rect = geo.body_rect(ref, p['x'], p['y'], p['rot'])
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
                'kind': 'contradiction',
                'winner': 'outline',
                'why': 'the declared pose puts the drawn body entirely '
                       'outside the board outline\'s bounding box'})

    # #1051/#1054: a mechanical POSE for a part the brief declares an ARRAY
    # member. The file pins the part; the brief says it moves with its row --
    # and no plan can do both (the loader refuses a fixed pose on a member,
    # and a FILE lock on a member is an array_conflict). So it is the same
    # kind of disagreement as a brief edge against a mechanical edge: two
    # declared/recorded values that disagree are a CONTRADICTION, reported
    # until answered, and the stronger source wins -- a declared brief over
    # a recorded file, so the mechanical value LOSES, is not anchored
    # (`lost_mechanical_refs`) and owes no lock. To keep the pose instead,
    # take the part out of the array.
    members = {}
    for a in (brief_fragment or {}).get('arrays') or ():
        for m in a.get('members') or ():
            members.setdefault(str(m), str(a.get('name')))
    for ref, p in sorted((mech.get('poses') or {}).items()):
        if ref not in members or ref not in pcb.footprints:
            continue
        values = {
            'mechanical': {'value': f"fixed at ({p['x']:g}, {p['y']:g}"
                                    + (f", {p['rot']:g}deg)"
                                       if p.get('rot') is not None else ')'),
                           'authority': mech_auth, 'source': mech_src},
            'brief': {'value': f"moves with array {members[ref]!r}",
                      'authority': brief_auth, 'source': brief_source}}
        _row(ref, 'array', values, lambda a, b: a == b,
             'the mechanical file pins this part at a pose, and the brief '
             'declares it a member of a row that moves as one piece -- one '
             'of the two has to give')

    knobs = mech.get('floors') or {}
    if knobs and floors_used:
        for knob in sorted(set(knobs) | set(floors_used)):
            m = knobs.get(knob)
            u = floors_used.get(knob)
            if m is None and u is None:
                continue
            values = {}
            if m is not None:
                values['mechanical'] = {'value': m, 'authority': 'assumption',
                                        'source': mech_src}
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
                'why': ('mechanical.json records this floor; the grade '
                        'used the value named `graded`')})
    # A mechanical value that LOST one row's contradiction is not the winner
    # of that ref's other rows either: a USB1 that lost `USB1:edge` to the
    # brief must not print "mechanical wins" on `USB1:pose`.
    lost = set(lost_mechanical_refs(rows))
    for r in rows:
        if (r.get('ref') in lost and r['winner'] == 'mechanical'
                and r['kind'] != 'contradiction'):
            rest = {ch: v for ch, v in r['values'].items()
                    if ch != 'mechanical'}
            r['winner'] = _winner(rest)
            r['why'] = (f"{r['why']} -- the mechanical value lost "
                        f"{r['ref']}'s contradiction elsewhere, so it wins "
                        f"nothing here")
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
                  tolerance_mm: float = ANCHOR_TOL_MM,
                  prefix: str = 'mech:', basis: str = 'mechanical'):
    """`(blocks, skipped)`: one grade-only anchor block per mechanical ref.

    Compiled by the GRADE from the file (`floorplan.mechanical_anchor_
    violations`), and never written into a plan: `block` names starting
    `mech:` are refused in an intent, so no plan can leave an anchor out or
    label a zone of its own as one.

    The zone is the GRADER's own rect for the part at its declared pose --
    `_Part.rect`, which is what `rule_zone_containment` tests -- unioned with
    the exactly rotated rect (the grader falls back to its 0-degree bounds
    off the 90-degree lattice), rounded outward; over every rotation the part
    may take when the declaration gives none. So a part sitting at its
    declared pose grades clean and one that moved does not.

    Grade-only: the anchor never seats anything. A part reaches its pose
    either FILE-locked there or through an intent
    `fixed_poses[]` entry, which place_simple seats exactly and holds
    (#1054) -- a zone is no way to seat one: measured before this was
    built, a part overhanging the outline has no admissible zone pose at
    all, edge claims are seated before zones, and a seat lands anywhere
    within the tolerance.
    No `rotation` key: rotation drift is graded by `mechanical_drift`, and a
    rotation DECISION on a block would collide with a plan's own claims.

    Skipped, with the reason: pad-less refs (never placed; they are
    reconciled only), refs the board does not have, and refs whose
    mechanical value lost a contradiction.

    `prefix`/`basis` (#1054): the same anchor, compiled from an intent's
    `fixed_poses[]` instead of the file -- `floorplan.fixed_pose_violations`
    passes `fixed:` and the entry's own basis, so a brief-declared pose is
    graded by exactly this geometry and never by a second copy of it.
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
        # `fp.pads` on purpose (#1143): the placement state keeps a part
        # whose only pads are paste/mask apertures movable -- so such a part
        # is anchored.
        if not fp.pads:
            skipped[ref] = ('pad-less: it is never placed, so it is '
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
        # A declaration with no `rot` constrains the POSITION only. The
        # anchor is compiled at GRADE time from the board being graded, so
        # the part's CURRENT rotation is the one to hold it at: that pins no
        # rotation (the part is always at its own) and stays as tight as a
        # declared one. The union over all rotations this replaced admitted
        # a 16 mm move of kit-dev's SW_ONOFF201 (round-2 verifier).
        rots = ([p['rot']] if p['rot'] is not None else
                [(fp.rotation or 0.0) % 360.0])
        b0 = part.bounds_by_rot[0.0]
        rects = []
        for rot in rots:
            rects.append(part.rect(p['x'], p['y'], rot))
            e = rotate_local_bounds(*b0, rot)
            rects.append((p['x'] + e[0], p['y'] + e[1],
                          p['x'] + e[2], p['y'] + e[3]))
        zone = [math.floor(min(r[0] for r in rects) * 1e4) / 1e4,
                math.floor(min(r[1] for r in rects) * 1e4) / 1e4,
                math.ceil(max(r[2] for r in rects) * 1e4) / 1e4,
                math.ceil(max(r[3] for r in rects) * 1e4) / 1e4]
        blocks.append({
            'name': f'{prefix}{ref}', 'refs': [glob.escape(ref)], 'zone': zone,
            'tolerance_mm': tolerance_mm,
            'note': (('mechanical.json: ' if basis == 'mechanical'
                      else 'fixed_poses: ')
                     + (p.get('reason') or 'a declared mechanical pose')),
            'context': {'basis': basis,
                        'mechanical_pose': [p['x'], p['y'], p['rot']],
                        'source': mechanical.get('path')}})
    return blocks, skipped


def mechanical_fixed_poses(mechanical: Dict, anchored: Sequence[str], *,
                           claimed: Dict[str, str] = None):
    """`(entries, skipped)`: mechanical.json poses as intent `fixed_poses[]`
    rows (#1054), so place_simple can SEAT what the grade anchors.

    Only the refs `anchor_blocks` anchored: its skips (off the board,
    pad-less, a lost contradiction) are the same reasons not to seat one,
    and a second list of reasons would drift from the first. `claimed` is
    `{ref: why}` for refs another claim of the same intent already places
    (an edge connector, a must_lock pattern, a brief-declared pose): each is
    skipped WITH that reason, because the intent loader refuses a ref under
    two such claims and the grade anchors it from the file either way.
    A declaration with no `rot` compiles with no `rot` -- position only.
    """
    claimed = claimed or {}
    entries, skipped = [], {}
    for ref in sorted(anchored):
        p = (mechanical.get('poses') or {}).get(ref)
        if p is None:
            continue
        if ref in claimed:
            skipped[ref] = claimed[ref]
            continue
        row = {'ref': ref, 'x': p['x'], 'y': p['y'], 'basis': 'mechanical',
               'why': ('mechanical.json: '
                       + (p.get('reason') or 'a declared mechanical pose'))}
        if p.get('rot') is not None:
            row['rot'] = p['rot']
        entries.append(row)
    return entries, skipped
