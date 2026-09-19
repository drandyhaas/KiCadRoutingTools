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
CONTRADICTION: P1 refuses until the plan acknowledges it in
`dispositions.contradictions`. A disposition ACCEPTS the winner the row names
-- it does not flip it. A plan is not a declaration and cannot overrule one;
to make the other value hold, correct the source that is wrong (the brief,
`mechanical.json`, or the board). A disagreement with anything weaker is
DRIFT: the stronger value wins without anyone having to say so.

Mechanical ANCHORS are compiled at GRADE time from the file itself
(`floorplan.mechanical_anchor_violations`), never read out of a plan: a plan
that must carry them can leave them out (run 29's plans did), and a plan block
that could label itself an anchor could exempt itself from the envelope and
overlap checks.

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
#: Every top-level key either shape carries. stage_unaided writes schema,
#: kind, refs, reasons, project and note; the declaration form interfaces,
#: fixed and project.
_MECH_KEYS = frozenset({'schema', 'kind', 'refs', 'reasons', 'interfaces',
                        'fixed', 'project', 'note', 'context'})

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
    facts". That includes a file whose sections are all empty, a key neither
    shape has, and a malformed entry in ANY section: an `interfaces` row with
    `edge: "up"` is refused whichever shape carries it, never skipped.
    """
    try:
        with open(path, encoding='utf-8') as fh:
            raw = json.load(fh)
    except (OSError, ValueError) as exc:
        raise MechanicalError(f"{path}: {exc}") from exc
    if not isinstance(raw, dict):
        raise MechanicalError(f"{path}: expected a JSON object")
    if not ({'refs', 'interfaces', 'fixed'} & set(raw)):
        raise MechanicalError(
            f"{path}: not a mechanical declaration this build reads -- "
            f"expected stage_unaided's `refs` map or `interfaces` / `fixed` "
            f"lists (got keys {sorted(raw)})")
    unknown = sorted(set(raw) - _MECH_KEYS)
    if unknown:
        raise MechanicalError(
            f"{path}: unknown key(s) {unknown}; a mechanical declaration "
            f"carries only {sorted(_MECH_KEYS)}")
    if 'kind' in raw and raw['kind'] != 'mechanical-declaration':
        raise MechanicalError(f"{path}: kind {raw['kind']!r}, expected "
                              f"'mechanical-declaration'")
    poses: Dict[str, Dict[str, object]] = {}
    edges: Dict[str, str] = {}
    shape = 'stage_unaided' if 'refs' in raw else 'declaration'
    if 'refs' in raw:
        refs = raw['refs']
        if not isinstance(refs, dict):
            raise MechanicalError(f"{path}: `refs` must map ref -> [x, y, rot]")
        reasons = raw.get('reasons') or {}
        if not isinstance(reasons, dict):
            raise MechanicalError(f"{path}: `reasons` must map ref -> text, "
                                  f"got {type(reasons).__name__}")
        for ref, v in sorted(refs.items()):
            if not isinstance(v, (list, tuple)) or len(v) != 3:
                raise MechanicalError(
                    f"{path}: refs.{ref}: expected [x, y, rot], got {v!r}")
            poses[str(ref)] = {
                'x': _num(v[0], f'refs.{ref}[0]'),
                'y': _num(v[1], f'refs.{ref}[1]'),
                'rot': _num(v[2], f'refs.{ref}[2]') % 360.0,
                'reason': str(reasons.get(ref) or '')}
    elif 'reasons' in raw:
        raise MechanicalError(f"{path}: `reasons` annotates `refs`, and this "
                              f"file has none")
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
    if not poses and not edges and not (
            raw.get('kind') == 'mechanical-declaration'
            and isinstance(raw.get('refs'), dict)):
        # The stager's own `kind` + an empty `refs` map is a DECLARATION
        # that the board has no mechanical parts, and it is written on 9 of
        # the 22 corpus boards; refusing it dead-ended every unaided run on
        # them (round-2 verifier). A hand-written file whose sections are
        # all empty says nothing, and that stays refused.
        raise MechanicalError(
            f"{path}: declares no pose and no edge -- a mechanical "
            f"declaration that says nothing would grade as 'no mechanical "
            f"facts'. Delete it, or pass --no-mechanical")
    floors = _floors((raw.get('project') or {}).get('floors')
                     if isinstance(raw.get('project'), dict) else None)
    return {'path': os.path.abspath(path), 'sha256': _sha256(path),
            'shape': shape, 'poses': poses, 'edges': edges,
            'floors': floors}


def brief_authority(brief_path: Optional[str], board_path: str):
    """`(authority, why)` for the design brief's values.

    `declared` -- a human's -- except under an unaided regime that did not
    record the brief at staging: there the placement skill's P-brief stage
    has the RUN write it, so it is the run's reading of the requirements, a
    hypothesis. The round-2 verifier laundered a moved mechanical ref past P1
    and the grade with one such row (`Ref*` declared on the north edge beat
    the recorded pose, which then anchored nothing). A manifest that records
    `brief_sha256` vouches for the brief it names, byte for byte."""
    man = regime_manifest(board_path)
    if not isinstance(man, dict):
        return 'declared', 'no unaided regime governs this board'
    rec = man.get('brief_sha256')
    if rec and brief_path and os.path.isfile(brief_path) \
            and _sha256(brief_path) == rec:
        return 'declared', 'recorded at staging by the unaided regime'
    return 'hypothesis', ('written during an unaided run -- the regime '
                          'recorded no brief at staging, so it is the run\'s '
                          'reading of the requirements, not a declaration')


def regime_manifest(board_path: str):
    """The unaided-regime manifest governing `board_path` as a dict, None
    when no regime governs it, or a string saying why it could not be read."""
    from . import provenance as PV
    wd = PV.regime_for(board_path)
    if wd is None:
        return None
    try:
        with open(os.path.join(wd, PV.REGIME_NAME), encoding='utf-8') as fh:
            return json.load(fh)
    except (OSError, ValueError) as exc:
        return f'the regime manifest is unreadable ({exc})'


def staged_lock_poses(man: Dict):
    """`{ref: (x, y, rot)}` for every part the staged board carried
    `(locked yes)` for AT STAGING, or None when nothing vouches for them.

    The manifest's own `staged_lock_poses` when `start_regime` recorded it
    (#959). Otherwise the staged board's CURRENT locks, only while its sha
    still matches `staged_sha256`: the skill's own commands edit that file in
    place (`place_pose BOARD BOARD ... lock`), so reading it later would
    promote the run's lock to a pre-run fact -- which the Phase-3 verifier
    measured turning the run's own move into a "contradiction between
    declared sources" that the run could then disposition away."""
    rec = man.get('staged_lock_poses')
    if isinstance(rec, dict):
        return {str(r): tuple(float(v) for v in xyz)
                for r, xyz in rec.items()}
    sb = man.get('staged_board')
    if not sb or not os.path.isfile(sb):
        return None
    if not man.get('staged_sha256') or _sha256(sb) != man['staged_sha256']:
        return None
    try:
        from .provenance import locked_poses
        return {r: tuple(xyz) for r, xyz in locked_poses(sb).items()}
    except (OSError, ValueError):
        return None


def mechanical_provenance(mech: Dict, board_path: str):
    """`(status, why, staged_locks)` for one loaded `mechanical.json`.

    status: `verified` (the unaided-regime manifest names this file and its
    sha), `unverified` (no manifest, or one staged before the sha was
    recorded), or `mismatch` (the manifest recorded different bytes -- the
    file changed after staging, so it is the run's own writing and reads as a
    hypothesis). `staged_locks` is `{ref: pose}` for the locks the staged
    board carried WHEN IT WAS STAGED -- the only locks that existed before
    the run -- or None when nothing can vouch for it (`staged_lock_poses`)."""
    man = regime_manifest(board_path)
    if man is None:
        return ('unverified', 'no unaided-regime manifest governs this '
                'board, so nothing vouches for when this file was written',
                None)
    if isinstance(man, str):
        return ('unverified', man, None)
    staged_locks = staged_lock_poses(man)
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


class _Geometry:
    """The board's bodies and placement state, built on first use: a
    reconciliation with nothing to compare must not pay for either (the
    Phase-3 verifier measured 0.55 s on ulx3s for an empty one)."""

    def __init__(self, pcb, board_path):
        self.pcb, self.board_path = pcb, board_path
        self._bodies = self._state = None

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
                self._state = pose_score.make_state(self.pcb,
                                                    self.board_path)
            except ValueError:
                # No outline (or none the placement state trusts): there is
                # no edge to read a part against. The grade refuses such a
                # board itself, at exit 3 -- reconciliation, which runs
                # first, must not turn that into a traceback (pre-push
                # review: exit 1 where the base exited 3).
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
    mechanical and board values are read off `edge_seat_rect` at the declared
    and current pose -- the rect the grader itself reads, so reconciliation
    and the grade cannot disagree about which edge a part is on), `pose`
    (mechanical vs board), `on_board` (a declared pose whose drawn body is
    DISJOINT from the outline's bounding box contradicts the outline), and
    `floors` (mechanical vs the floors the grade used; a report, never a
    contradiction). A row whose mechanical value lost ANOTHER row's
    contradiction does not name mechanical its winner.
    """
    geo = _Geometry(pcb, board_path)
    bounds = pcb.board_info.board_bounds if pcb.board_info else None
    brief_entries = {str(c.get('ref')): c
                     for c in (brief_fragment or {}).get('edge_connectors')
                     or [] if c.get('ref') and c.get('edge')}
    brief_edges = {r: c.get('edge') for r, c in brief_entries.items()}
    intent_entries = {str(c.get('ref')): c
                      for c in (intent_doc or {}).get('edge_connectors')
                      or [] if c.get('ref') and c.get('edge')}
    intent_edges = {r: c.get('edge') for r, c in intent_entries.items()}
    brief_auth, brief_auth_why = brief_authority(brief_source, board_path)
    if brief_source and os.path.isfile(brief_source):
        brief_source = (f"{brief_source} (sha256 {_sha256(brief_source)}; "
                        f"{brief_auth_why})")
    mech = mechanical or {}
    prov, prov_why, staged_locks = ((None, None, None) if not mech else
                                    mechanical_provenance(mech, board_path))
    mech_auth = ('hypothesis' if prov == 'mismatch' else 'recorded_fact')
    mech_src = (f"{mech.get('path')} (sha256 {mech.get('sha256')}; "
                f"{prov}: {prov_why})" if mech else None)
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
                                                p['rot']), bounds),
                'authority': mech_auth, 'source': mech_src}
        if ref in intent_edges:
            ie = intent_edges[ref]
            values['intent'] = {
                'value': ie,
                # Per value: an intent carrying the brief's own value is
                # carrying what the brief is -- a declaration, or under an
                # unaided regime the run's own reading; anything else is the
                # run's guess.
                'authority': (brief_auth if brief_edges.get(ref) == ie
                              else 'hypothesis'),
                'source': intent_source}
        values['board'] = {'value': _edge_of(geo.seat_rect(ref, entry),
                                             bounds),
                           'authority': 'inferred', 'source': board_path}
        _row(ref, 'edge', values, lambda a, b: a == b,
             'the edge each channel puts this part on, read off the rect the '
             'edge-connector rule reads (`edge_seat_rect`)')

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
        # A board pose is a pre-run fact only while the part is LOCKED where
        # it was locked at staging. The run's own lock, or a staged lock the
        # run has since moved, is the run's writing.
        cur = (fp.x, fp.y, (fp.rotation or 0.0) % 360.0)
        locked = getattr(fp, 'locked', False)
        sp = (staged_locks or {}).get(ref)
        board_auth = ('recorded_fact' if locked and sp is not None
                      and _same_pose(sp, cur)
                      else 'hypothesis' if locked else 'inferred')
        values = {
            'mechanical': {'value': (p['x'], p['y'], p['rot']),
                           'authority': mech_auth, 'source': mech_src},
            'board': {'value': cur, 'authority': board_auth,
                      'source': board_path}}
        _row(ref, 'pose', values, _same_pose,
             p.get('reason') or 'a declared mechanical pose')
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
                'kind': ('contradiction' if mech_auth == 'recorded_fact'
                         else 'drift'),
                'winner': 'outline',
                'why': 'the declared pose puts the drawn body entirely '
                       'outside the board outline\'s bounding box'})

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
    # A mechanical value that LOST one row's contradiction is not the winner
    # of that ref's other rows either: run 29's USB1 lost `USB1:edge` to the
    # brief, and `USB1:pose` still printed "mechanical wins".
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
                  tolerance_mm: float = ANCHOR_TOL_MM):
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
            'name': f'mech:{ref}', 'refs': [glob.escape(ref)], 'zone': zone,
            'tolerance_mm': tolerance_mm,
            'note': ('mechanical.json: '
                     + (p.get('reason') or 'a declared mechanical pose')),
            'context': {'basis': 'mechanical',
                        'mechanical_pose': [p['x'], p['y'], p['rot']],
                        'source': mechanical.get('path')}})
    return blocks, skipped
