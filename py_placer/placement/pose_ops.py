"""Apply a MODEL-CHOSEN pose, and let the engine grade it (#892).

The ranking half of "where should this part go" already shipped: `pose_score.
rank_poses` returns legal (x, y, rot) candidates cheapest-first, `converge.py
poses` is its CLI, and `grade_pad_legality` / `check_assembly` / `render_
placement --review-sheet` all grade a pose. What did not exist is the verb that
APPLIES one: no placement CLI in the tree accepted a pose at all, so a model
that had decided "U1 belongs here, facing that way" had to write a hand script
around `placement.writer.write_placed_output` (run 25's `pose_assist.py`;
run 24 reused `freeze_stamp.py` for the lock half). A hand script is exactly
what the provenance regime refuses and the cheats watcher flags, so the
sanctioned tool has to be a registered lever -- see `place_pose.py`.

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
* NO RULE IS RE-DERIVED. The pad geometry is `grade_pad_legality`'s, the face
  rule is `escape.assign_faces` (#850), the ranking is `pose_score.rank_poses`,
  the writer is `placement.writer.write_placed_output`, the siblings are
  `portfolio.copy_siblings` (#441). This module is plumbing and arithmetic on
  their answers. `part_faces` is the one place that repeats a CALLING PATTERN
  rather than a rule -- see its docstring for why it does not simply call
  `board_context.pads_by_face`.
"""
from __future__ import annotations

import os
import shutil
import tempfile
from typing import Dict, List, Optional, Sequence

#: The cardinal faces, in the order a POSITIVE rotation walks them.
#:
#: Measured from the parser's own transform rather than reasoned about
#: (`kicad_parser.py:706`): a pad's global offset is `R(-rot) . local`, so
#: raising a footprint's rotation by 90 degrees sends a pad at local (+1, 0)
#: to global offset (0, -1) -- east to north, since `escape.face_of` calls the
#: smaller-y side north on a y-grows-down board.
#: `tests/test_892_place_pose.py` pins this against a real write + re-parse,
#: because a cycle written the wrong way round is a plausible-looking rotation
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
#: `grade_pad_legality`.
LEGALITY_KEYS = ('pad_conflicts', 'hole_conflicts', 'oob_pad_count')

#: The MAGNITUDES, and they are not a nicety: a count arm alone accepts a
#: request that keeps the tally and deepens the damage. Measured on the
#: count-plus-shortfall version -- a part already 2.0 mm off the board was
#: moved to 204.66 mm off it, `oob_pad_count 1 -> 1`, exit 0, `legal: true`,
#: and no field in the summary said so. CLAUDE.md calls pad copper outside the
#: outline the top-priority placement defect, so its AMOUNT is an arm too.
MAGNITUDE_KEYS = ('pad_shortfall', 'oob_pad_amount')
MAGNITUDE_EPS = 1e-6


class PoseRefusal(Exception):
    """A request refused for a stated reason, with the numbers behind it.

    `code` is the exit code the CLI should use, and the two values mean
    different things to a caller: 2 is "the request does not name a thing on
    this board" (an unknown ref, a face with no row, a rotation that is not a
    multiple of 90) -- a typo the caller fixes by rewriting the command -- and
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

    Same resolution `place_seed`, `converge poses` and `board_context` use
    (`list_nets.board_floor_knobs`): explicit value > the board's own Default
    netclass / constraint > the fixed default. A fixed 0.25/0.55 on a board
    routed to 0.15/0.3 vetoes poses the router would happily route, which is
    the run-7 S4 finding this exists to avoid.
    """
    import list_nets
    import routing_defaults as defaults
    clr, edge, knobs = list_nets.board_floor_knobs(
        board_path, clearance, board_edge_clearance)
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
    different question than the caller asked.
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
    if not (fp.pads or ()):
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
    pads = list(fp.pads or ())
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

def resolve_ops(pcb_data, ops: Sequence[Dict], *, clearance: float,
                track_width: float):
    """Turn caller ops into writer placements + a per-op note.

    Every op reads the INPUT board, so a call carrying several of them
    describes ONE arrangement rather than a sequence whose later members see
    the earlier ones' effects. That is what makes "a whole arrangement is one
    board state" true rather than merely intended.
    """
    placements: List[Dict] = []
    notes: List[Dict] = []
    seen = {}
    for op in ops:
        kind = op['kind']
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
                    "%s has no pads on its %s face right now, so there is no "
                    "row to aim; it has %s" % (
                        ref, face,
                        ', '.join('%s: %d' % (f, len(p))
                                  for f, p in sorted(by.items())) or 'none'),
                    faces={f: len(p) for f, p in by.items()})
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
            note.update({'face': face, 'partner': op['partner'],
                         'target_face': target, 'rotation_delta': delta,
                         'row_pads': [p.pad_number for p in by[face]]})
        else:
            raise PoseRefusal("unknown op %r" % (kind,), code=2)

        note['to'] = [round(x, 4), round(y, 4), rot]
        note['moved'] = note['to'] != note['from']
        notes.append(note)
        placements.append({'reference': ref, 'new_x': x, 'new_y': y,
                           'new_rotation': rot})
    return placements, notes


# ---------------------------------------------------------------------------
# legality
# ---------------------------------------------------------------------------

def grade(pcb_data, board_path: str, clearance: float) -> Dict:
    """`grade_pad_legality` at this board's own poses -- never a re-derivation.

    `pcb_file` is passed so `PadClearanceModel` can read the netclasses, the
    `.kicad_dru` layer rules and any pad `local_clearance` override (#697):
    a board that declares none of the three grades exactly as a flat scalar
    would, and one that declares them is graded the way check_drc will.
    """
    from placement.legality import grade_pad_legality
    return grade_pad_legality(pcb_data, clearance, pcb_file=board_path)


def worsened(before: Dict, after: Dict) -> List[str]:
    """Which legality categories the request made worse. [] is the good case."""
    out = [k for k in LEGALITY_KEYS
           if (after.get(k) or 0) > (before.get(k) or 0)]
    out += [k for k in MAGNITUDE_KEYS
            if (after.get(k) or 0.0) > ((before.get(k) or 0.0) + MAGNITUDE_EPS)]
    return out


def is_clean(report: Dict) -> bool:
    """Is this board legal in the ABSOLUTE sense, not merely no worse?"""
    return not (any(report.get(k) for k in LEGALITY_KEYS)
                or any((report.get(k) or 0.0) > MAGNITUDE_EPS
                       for k in MAGNITUDE_KEYS))


def _legality_row(before: Dict, after: Dict) -> Dict:
    row = {}
    for key in LEGALITY_KEYS + MAGNITUDE_KEYS:
        row[key + '_before'] = before.get(key)
        row[key + '_after'] = after.get(key)
    row['pad_clearance_required'] = after.get('required')
    row['worst'] = after.get('worst')
    return row


# ---------------------------------------------------------------------------
# snapping
# ---------------------------------------------------------------------------

def nearest_legal(board_path: str, ref: str, *, clearance: float,
                  board_edge_clearance: float, radius: float, step: float,
                  rotations=None, limit: int = 12, pcb_data=None):
    """The ranked legal poses for `ref` on the board at `board_path`.

    `pose_score.rank_poses`, unchanged: this only resolves the knobs from the
    board first and carries the dropped-pose census back, so an empty ranking
    can say "the knobs veto even staying put" instead of "no legal pose"
    (run-7 S4).
    """
    import pose_score
    from kicad_parser import parse_kicad_pcb
    pcb = pcb_data if pcb_data is not None else parse_kicad_pcb(board_path)
    st = pose_score.make_state(pcb, board_path, clearance=clearance,
                               board_edge_clearance=board_edge_clearance)
    diag: Dict = {}
    kw = {} if rotations is None else {'rotations': tuple(rotations)}
    poses = pose_score.rank_poses(pcb, board_path, ref, radius=radius,
                                  step=step, limit=limit, state=st,
                                  diagnostics=diag, **kw)
    return poses, diag


def snap_candidates(board_path: str, ref: str, *, rot: float, clearance: float,
                    board_edge_clearance: float, radius: float, step: float,
                    pcb_data=None):
    """Poses to TRY for a snap, nearest-first, from TWO rungs. (list, census)

    Rung 1 is `rank_poses` -- cost-ordered, and worth trying first because it
    knows about wirelength and crossings, which this verb does not.

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
    """
    import pose_score
    from kicad_parser import parse_kicad_pcb
    pcb = pcb_data if pcb_data is not None else parse_kicad_pcb(board_path)
    st = pose_score.make_state(pcb, board_path, clearance=clearance,
                               board_edge_clearance=board_edge_clearance)
    diag: Dict = {}
    ranked = pose_score.rank_poses(pcb, board_path, ref, radius=radius,
                                   step=step, limit=24, state=st,
                                   rotations=(rot,), diagnostics=diag)
    # The Euclidean bound, because `_offsets` walks SQUARE rings: a corner of
    # the r=4 ring sits 5.66 mm out, and a caller who typed `--radius 4` read
    # it as a distance (measured: a snap moved a part 5.0 mm under 4).
    out = [dict(p, rung='ranked') for p in ranked
           if (p.get('dist_mm') or 0.0) <= radius + 1e-9]
    seen = {(round(p['x'], 4), round(p['y'], 4)) for p in out}

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
    census = {'ranked': len(out),
              'ranked_before_radius': len(ranked),
              'lattice': len(lattice),
              'radius_mm': radius,
              'step_mm': step,
              'rotations': [rot],
              'dropped_total': diag.get('dropped_total', 0),
              'dropped_in_place': diag.get('dropped_in_place', []),
              'stopped_early': bool(diag.get('stopped_early'))}
    return out + lattice, census


# ---------------------------------------------------------------------------
# locks
# ---------------------------------------------------------------------------

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
                force: bool = False, dry_run: bool = False) -> Dict:
    """Apply `ops` to `board_path`, grade the result, write it to `out_path`.

    Returns the summary a caller prints as `JSON_SUMMARY`. Raises `PoseRefusal`
    when the request cannot be honoured -- an unknown ref, a face with no pads,
    or a pose that makes the board's pad legality worse (unless `force`).

    The candidate is STAGED and graded before anything reaches `out_path`, so a
    refused request leaves no half-written board behind and `dry_run` grades
    exactly what a real run would have written.
    """
    from kicad_parser import parse_kicad_pcb
    from placement.portfolio import copy_siblings
    from placement.writer import write_placed_output

    if not dry_run and not out_path:
        raise PoseRefusal("a write needs an output path; pass one, or "
                          "--dry-run to grade without writing", code=2)
    clearance, board_edge_clearance, track_width, knobs = resolve_knobs(
        board_path, clearance, board_edge_clearance, track_width)
    pcb = pcb_data if pcb_data is not None else parse_kicad_pcb(board_path)

    placements, notes = resolve_ops(pcb, ops, clearance=clearance,
                                    track_width=track_width)

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
    }

    before = grade(pcb, board_path, clearance)
    stage = tempfile.TemporaryDirectory(prefix='place_pose_')
    try:
        cand = os.path.join(stage.name, 'candidate.kicad_pcb')
        if placements:
            write_placed_output(board_path, cand, placements)
        else:
            shutil.copyfile(board_path, cand)
        copy_siblings(board_path, cand)
        cand_pcb = parse_kicad_pcb(cand)
        after = grade(cand_pcb, cand, clearance)
        bad = worsened(before, after)

        if snap and len(placements) != 1:
            # SAID, not silently dropped: --snap/--near relocates ONE part,
            # and a call carrying several ops has no single point to sweep
            # around. A flag that quietly does nothing is how a caller
            # concludes the engine considered an alternative and found none.
            summary['snap_census'] = {
                'skipped': 'snap applies to exactly one pose op; this call '
                           'carries %d' % len(placements)}
        if bad and snap and len(placements) == 1:
            # Rank around the REQUESTED point, not the part's old one: the
            # sweep in `rank_poses` is centred on where the part sits in the
            # board it is handed, and on the staged board that is exactly
            # where the caller aimed. The rotation stays as asked -- a snap
            # relocates a decision, it does not overrule it.
            #
            # Each candidate is RE-GRADED rather than trusted, because the
            # ranking's own legality (`QuenchState.candidate_valid`, an AABB
            # gate) and this verb's verdict (`grade_pad_legality`, exact
            # geometry) are different currencies: the ranker can hand back the
            # very pose that was refused. Walking the list until one grades
            # clean is what makes --snap mean "a pose that passes", not "the
            # cheapest pose the other instrument liked".
            ref = placements[0]['reference']
            want_rot = placements[0]['new_rotation']
            poses, census = snap_candidates(
                cand, ref, rot=want_rot, clearance=clearance,
                board_edge_clearance=board_edge_clearance,
                radius=snap_radius, step=snap_step, pcb_data=cand_pcb)
            summary['nearest_legal'] = next(
                (p for p in poses if p['rung'] == 'ranked'), None)
            summary['nearest_legal_basis'] = (
                'pose_score.rank_poses / QuenchState.candidate_valid -- an '
                'AABB gate, RE-GRADED here before it is written')
            summary['snap_census'] = census
            tried = 0
            for cand_pose in poses[:snap_tries]:
                tried += 1
                trial = [{'reference': ref, 'new_x': cand_pose['x'],
                          'new_y': cand_pose['y'],
                          'new_rotation': cand_pose['rot']}]
                write_placed_output(board_path, cand, trial)
                copy_siblings(board_path, cand)
                cand_pcb = parse_kicad_pcb(cand)
                after = grade(cand_pcb, cand, clearance)
                bad = worsened(before, after)
                if not bad:
                    placements = trial
                    summary['snapped'] = {
                        'ref': ref,
                        'to': [cand_pose['x'], cand_pose['y'],
                               cand_pose['rot']],
                        'dist_mm': cand_pose.get('dist_mm'),
                        # WHICH rung answered: 'ranked' means the pose scorer
                        # and this verb agreed, 'lattice' means only this
                        # verb's own grade accepted it.
                        'rung': cand_pose.get('rung'),
                        'candidates_tried': tried}
                    summary['snap_census']['candidates_tried'] = tried
                    for n in notes:
                        if n['ref'] == ref:
                            n['to'] = [round(cand_pose['x'], 4),
                                       round(cand_pose['y'], 4),
                                       cand_pose['rot']]
                            n['moved'] = n['to'] != n['from']
                            n['snapped'] = True
                    summary['moved'] = [n for n in notes if n['moved']]
                    break
            else:
                # Nothing TRIED graded clean: put the board back to the pose
                # the caller actually asked for, so the refusal below reports
                # THEIR request rather than the last thing tried. Note the cap
                # -- `candidates_tried` against `ranked + lattice` is how a
                # reader tells "there was nothing" from "we stopped looking".
                summary['snap_census']['candidates_tried'] = tried
                write_placed_output(board_path, cand, placements)
                copy_siblings(board_path, cand)
                cand_pcb = parse_kicad_pcb(cand)
                after = grade(cand_pcb, cand, clearance)
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
        face_miss = []
        for n in [n for n in notes if n['kind'] == 'face']:
            by = part_faces(cand_pcb, n['ref'], clearance=clearance,
                            track_width=track_width)
            landed = {p.pad_number: f for f, pads in by.items() for p in pads}
            counts: Dict[str, int] = {}
            for num in n['row_pads']:
                key = landed.get(num, 'gone')
                counts[key] = counts.get(key, 0) + 1
            hit = counts.get(n['target_face'], 0)
            n['row_landed'] = counts
            n['row_on_target'] = [hit, len(n['row_pads'])]
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
            if hit * 2 <= len(n['row_pads']):
                face_miss.append(n)

        summary.update(_legality_row(before, after))
        # TWO keys, because one word cannot carry both facts and the wrong one
        # was being published: `legal` used to mean "no worse than the input",
        # so a board still carrying a pad conflict reported `legal: true`.
        # `legal` is now the ABSOLUTE fact a reader takes it for, and
        # `no_worse` is the relative verdict this verb ACTS on.
        summary['no_worse'] = not bad
        summary['legal'] = is_clean(after)
        summary['legal_basis'] = (
            'legal = the board is clean at this pose; no_worse = the verdict '
            'this verb refuses on (relative to the input board)')

        if face_miss and not bad:
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
            summary['refused'] = reason
            if not force:
                summary['output'] = None      # nothing was written
                raise PoseRefusal(reason, summary=summary)
            summary['forced'] = True

        if bad or (strict and not summary['legal']):
            if summary['nearest_legal'] is None and len(placements) == 1:
                # Name the alternative even when the caller did not ask to
                # snap: a refusal that only says no costs the caller another
                # round trip to find out what WOULD have worked.
                try:
                    poses, _d = nearest_legal(
                        cand, placements[0]['reference'], clearance=clearance,
                        board_edge_clearance=board_edge_clearance,
                        radius=max(snap_radius, 2.0), step=snap_step,
                        rotations=(placements[0]['new_rotation'],),
                        pcb_data=cand_pcb)
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
                        'dropped_total': _d.get('dropped_total', 0),
                        'dropped_in_place': _d.get('dropped_in_place', []),
                        'stopped_early': bool(_d.get('stopped_early')),
                        # The SWEEP's radius, and it is a Chebyshev box half
                        # width, not the Euclidean bound `--radius` applies to
                        # a snap: this list is information about what exists,
                        # so a pose further out than the caller would accept
                        # is still worth naming.
                        'sweep_radius_mm': max(snap_radius, 2.0),
                        'rotations': [placements[0]['new_rotation']]}
                except Exception as exc:                     # noqa: BLE001
                    summary['nearest_legal'] = None
                    summary['nearest_legal_census'] = {
                        'error': '%s: %s' % (type(exc).__name__, exc)}
            reason = _refusal_reason(bad, strict, before, after, summary)
            summary['refused'] = reason
            if not force:
                # A refusal writes nothing, so the summary must not name a
                # path: the two refusal channels disagreed about this, and a
                # machine caller cannot key on a field that means "written"
                # on one path and "would have been" on the other.
                summary['output'] = None
                raise PoseRefusal(reason, summary=summary)
            summary['forced'] = True

        if dry_run:
            # A dry run still says what the locks WOULD do, with the counts
            # left None: "0 stamped" and "not attempted" must not read alike.
            if lock_refs or unlock_refs:
                summary.update({'locked': sorted(lock_refs),
                                'unlocked': sorted(unlock_refs),
                                'locked_count': None, 'unlocked_count': None})
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
                    "intact. Nothing was written." % ', '.join(still))
                summary['refused'] = reason
                summary['output'] = None
                raise PoseRefusal(reason, summary=summary, unlock_failed=still)

        _promote(cand, out_path)
        return summary
    finally:
        stage.cleanup()


def _promote(staged: str, out_path: str) -> None:
    """Move a finished staged board (and its siblings) onto the output path."""
    from placement.portfolio import copy_siblings
    try:
        shutil.copyfile(staged, out_path)
        copy_siblings(staged, out_path)
    except OSError as exc:
        # A missing output directory used to surface as a FileNotFoundError
        # traceback and an exit 1 that the CLI's own table does not list --
        # after the whole grade had printed, so the run looked successful
        # until the last line.
        raise PoseRefusal(
            "cannot write %s: %s. The grade above is real; nothing was "
            "written." % (out_path, exc), code=2)


def _refusal_reason(bad, strict, before, after, summary) -> str:
    if bad:
        parts = ', '.join('%s %s -> %s' % (k, before.get(k), after.get(k))
                          for k in bad)
        reason = ("this pose makes the board's pad legality WORSE (%s). "
                  "Refused rather than written; the board's inherited "
                  "violations are not counted against you." % parts)
    else:
        reason = ("--strict-legal was asked for and the board is not clean at "
                  "this pose (%s)" % ', '.join(
                      '%s %s' % (k, after.get(k)) for k in LEGALITY_KEYS
                      if after.get(k)))
    nl = summary.get('nearest_legal')
    if nl and not bad:
        # --strict-legal is a WHOLE-BOARD condition, and this candidate is
        # about ONE part: moving there can leave every other conflict in
        # place, so it is offered as a next step and not as a fix.
        reason += (" The pose ranker's nearest candidate for this part is "
                   "x=%g y=%g rot=%g (%.3f mm away), but --strict-legal is a "
                   "condition on the WHOLE board, so that pose need not "
                   "satisfy it."
                   % (nl['x'], nl['y'], nl['rot'], nl.get('dist_mm') or 0.0))
    elif nl:
        reason += (" The pose ranker's nearest candidate is x=%g y=%g "
                   "rot=%g (%.3f mm away) -- try it with --snap, which "
                   "re-grades before writing."
                   % (nl['x'], nl['y'], nl['rot'], nl.get('dist_mm') or 0.0))
    elif summary.get('snap_census', {}).get('dropped_in_place'):
        reason += (" No legal pose was found nearby, and the census shows the "
                   "knobs veto the part's own spot too -- check the resolved "
                   "clearance against the board's floor before reading this "
                   "as 'the part is stuck'.")
    return reason
