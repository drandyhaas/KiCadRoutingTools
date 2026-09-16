"""Was every pose in this board produced by a registered engine lever?

`fence_audit` asks a different question, correctly, and answers it every time:
*does any file in this work dir carry the control's poses?* That is the BLIND
question. Run 19 passed it -- `VERDICT: CLEAN, exit 0` -- on a run whose
placement came from a 221-line hand script, because a hand-arranged board
matches the control at ~0.0, nowhere near `MATCH_FRAC = 0.98`. There was
nothing for it to find.

The claim being made was a different one: *the engine placed this board*.
Nothing in the repo measured that. `FENCE_CLAUSE` gestures at it behaviourally
and concedes the limit in its own text ("nothing downstream can detect that it
happened"), and it says DISCLOSE, not refrain.

So this is an orthogonal sibling, not a replacement. Two questions, two
instruments, two manifests.

THIS IS AN ACCOUNTING BOUNDARY, NOT A SECURITY BOUNDARY, and saying so is the
honest register (`fence_audit.py:84-113` does the same for its own allow-list).
A determined author can call `declare_lever` from a hand script. What changes
is that doing so is an affirmative falsification rather than an omission -- and
because `provenance_audit` reconciles the DELIVERED BOARD's moved poses against
the ledger rather than reading the log alone, a forger must fabricate a
consistent `refs_moved` chain, which is a much larger act than skipping a
disclosure.

Registration is by explicit call, never by sniffing `sys.argv[0]`: sniffing is
defeated by one assignment, and a boundary that looks stronger than it is, is
worse than one that states its own limit.
"""
from __future__ import annotations

import contextlib
import hashlib
import inspect
import json
import os
import time
from typing import Dict, List, Optional, Sequence

SCHEMA = 1
LEDGER_NAME = '.pose-provenance.jsonl'
REGIME_NAME = '.unaided-manifest.json'

# The CLI entry points allowed to author poses. A tool absent from this list
# is not forbidden -- it simply cannot claim the run was engine-authored.
LEVER_REGISTRY = (
    'place_seed.py', 'place_optimize.py',
    'place_reconstruct.py', 'place_portfolio.py', 'place_route_loop.py',
    'place_fanout_clearance.py', 'converge.py',
    # #892: the sanctioned pose SETTER. It exists so that applying a model's
    # own layout decision is a registered lever rather than the hand script
    # this regime is built to refuse (run 25's `pose_assist.py`).
    'place_pose.py',
    # The ROUTER is a deliberate pose author: #666's scoped cap move calls
    # `write_placed_output` to relocate a cap off a rescue via. Absent from
    # this list it raised `UnaidedViolation` inside the funnel, route.py's
    # own `except Exception` swallowed it, and the router SILENTLY SKIPPED a
    # repair it performs everywhere else -- so an unaided work dir got a
    # different router. It lives in py_router/, not py_placer/.
    'route.py',
    # Staging tools author poses BY DESIGN -- that is what staging is.
    # `perturb.py` is a LIBRARY with no __main__; it is reached through
    # stage_blind, whose declaration covers it by the innermost-wins rule.
    #
    # `stage_unaided.py` was MISSING here while its own __main__ carried the
    # comment "In LEVER_REGISTRY, so it must DECLARE" -- the file asserted a
    # membership this tuple did not grant. That was harmless only while
    # NOTHING armed a regime (#903): now that both stagers arm their own work
    # dir, a RESTAGE declares `stage_unaided.py`, and without this entry the
    # stager is refused by the guard it installed one line earlier.
    'perturb.py', 'stage_blind.py', 'stage_unaided.py',
)

# Registered but NOT pose writers. `beautify_labels.py` moves reference-
# designator silkscreen through `write_label_output`, which is not the pose
# funnel -- it never records and never raises, so listing it here would be a
# decorative entry that implies a coverage this instrument does not have.
# Named rather than silently omitted, because "why is it missing" is a
# question someone will ask.
NOT_POSE_WRITERS = ('beautify_labels.py',)

#: Levers whose ledger row must carry NO inputs and NO poses.
#:
#: The ledger lives in the WORK DIR, which is inside the fence. That was
#: harmless while nothing armed a regime; since #903 both stagers do, and a
#: re-stage records itself. Measured on splitflap_driver, the unredacted row
#: held all three things the fence exists to keep out: `lever_argv` naming
#: `kicad_files/splitflap_driver.kicad_pcb` AND the truth directory,
#: `refs_moved` naming the perturbed block exactly (J19, U1..U9), and 65
#: `poses_written` of which 56 were the CONTROL pose to six decimals. A run
#: that read its own ledger could reconstruct the answer key, and
#: `fence_audit` could not see it because `.jsonl` was not a scanned
#: extension.
#:
#: `perturb.py` is here for the innermost-wins case where it declares itself.
#: What survives redaction is what the watcher actually reads -- the lever
#: name -- so the restage counter is unaffected.
FENCE_SENSITIVE_LEVERS = ('stage_blind.py', 'stage_unaided.py', 'perturb.py')

_active: List[Dict] = []


class UnaidedViolation(RuntimeError):
    """A pose write with no registered lever, under an unaided regime."""


@contextlib.contextmanager
def declare_lever(file: str, argv: Optional[Sequence[str]] = None):
    """Declare that the poses written inside this block come from `file`.

    Called explicitly by each CLI. The innermost declaration wins, so a tool
    that shells out to another still attributes to the one doing the writing.
    """
    _active.append({'lever': os.path.basename(file),
                    'lever_argv': list(argv) if argv else None})
    try:
        yield
    finally:
        _active.pop()


def active_lever() -> Optional[Dict]:
    return dict(_active[-1]) if _active else None


def _caller() -> str:
    """The outermost frame outside the placement PACKAGE -- the run-19 detector.

    A hand script that imports `placement.writer` and writes poses records
    ITSELF here, whatever it does or does not declare.

    It used to skip every frame under `/py_placer/`, which is where all the
    lever CLIs live (`place_seed.py`, `place_optimize.py`, ...) -- so the field
    documented as the detector recorded `<unknown>` for exactly the tools it
    exists to identify, and `<frozen runpy>` under `python -m`. Only the
    package internals are uninteresting; the CLI that called them is the
    answer.
    """
    try:
        for fr in inspect.stack()[1:]:
            fn = fr.filename.replace('\\', '/')
            if '/py_placer/placement/' in fn or fn.endswith('provenance.py'):
                continue
            # `recorded_delivery` is a context manager, so the frame between
            # it and the lever is contextlib's `__enter__`; without this every
            # delivery row names `contextlib.py` as its author.
            if fn.endswith('/contextlib.py'):
                continue
            if fn.startswith('<'):               # <frozen runpy>, <string>
                continue
            return f"{os.path.basename(fr.filename)}:{fr.lineno} in {fr.function}"
    except Exception:                            # noqa: BLE001
        pass
    return '<unknown>'


def regime_for(path: str) -> Optional[str]:
    """The work dir governing `path`, or None. Walks up for the manifest."""
    d = os.path.dirname(os.path.abspath(path)) or os.getcwd()
    seen = 0
    while d and seen < 24:
        if os.path.isfile(os.path.join(d, REGIME_NAME)):
            return d
        parent = os.path.dirname(d)
        if parent == d:
            break
        d, seen = parent, seen + 1
    return None


def sha256_file(path: str) -> str:
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


# THE POSE DIGEST (#972). The byte hashes above answer "is this the same FILE",
# and a chain of rows cannot be linked on that: legitimate steps rewrite a board
# without moving a part -- `seeder.stamp_locked` adds `(locked yes)` after the
# seed row, route.py writes copper, `beautify_labels` moves silkscreen, a fill
# or a pcbnew re-save rewrites the text -- and every one of them would read as
# an unrecorded change. The digest answers "is this the same ARRANGEMENT": a
# canonical hash of every footprint's (x, y, rotation, side), so a row's
# `parent_pose_sha256` links to an earlier row's `board_pose_sha256` across any
# rewrite that moved nothing, and a copy or `os.replace` delivery links by
# content rather than by path.
#
# The scheme id is part of the VALUE, not a separate key: a digest written
# under a different canonical form must never be compared with this one, and
# a prefix cannot be separated from the hash it qualifies.
POSE_DIGEST_SCHEME = 'p1'


def pose_footprints(path: str) -> Dict:
    """{ref: Footprint} through the parser's own footprint extractor.

    `parse_kicad_pcb` takes its footprints from exactly this call and changes
    no x/y/rotation/layer afterwards, so the keys are the ones the audit and
    every lever use -- including #726's `TP4~2` ordinals and a reference-less
    block's `#<uuid>` -- at about a third of the cost, because nets, zones and
    outline contours are never built.
    """
    from kicad_parser import extract_footprints_and_pads
    with open(path, 'r', encoding='utf-8') as f:
        content = f.read()
    return extract_footprints_and_pads(content, {}, {})[0]


def pose_table_of(footprints: Dict) -> Dict:
    """{ref: (x, y, rotation, side)} -- the audit's pose shape since #714."""
    from placement.legality import footprint_side
    return {ref: (fp.x, fp.y, fp.rotation or 0.0, footprint_side(fp))
            for ref, fp in footprints.items()}


def pose_table(path: str) -> Dict:
    return pose_table_of(pose_footprints(path))


def pose_digest(table: Dict) -> str:
    """Canonical digest of a pose table: `"p1:<sha256 hex>"`.

    Positions are integer NANOMETRES: the writer emits `:.6f` millimetres and
    KiCad stores integer nm, so any rewrite that keeps the decimal recovers the
    same integer. Rotation is quantised to 1e-4 degree BEFORE the modulo, so
    -90 and 270, 360 and 0, and a `-1e-17` that `% 360` turns into 360.0 all
    land on one integer. Integers and side letters only: a float in the JSON
    would spell -0.0 differently from 0.0.

    Exact on purpose. Two poses that differ by less than a nanometre link;
    anything else does not, and the audit then compares poses at its own drift
    tolerance -- so a missed link can hide nothing, it only stops a shortcut.
    """
    rows = [[ref, round(x * 1e6), round(y * 1e6),
             round(((rot or 0.0) % 360.0) * 1e4) % 3600000, side]
            for ref, (x, y, rot, side) in sorted(table.items())]
    blob = json.dumps(rows, separators=(',', ':'), ensure_ascii=True)
    return (POSE_DIGEST_SCHEME + ':'
            + hashlib.sha256(blob.encode('ascii')).hexdigest())


def file_pose_digest(path: str) -> Optional[str]:
    """The pose digest of a board file, or None. NEVER raises.

    Called from inside `commit_write`, which runs after the board is already
    on disk: an exception there would ship a board with no ledger row, which
    is the defect #960 closed. A digest that cannot be computed is recorded as
    None and the audit treats that link as unknown.
    """
    try:
        if not os.path.isfile(path):
            return None
        return pose_digest(pose_table(path))
    except Exception:                            # noqa: BLE001
        return None


def _stamp_board_pose(row: Dict, output_file: str) -> None:
    # A staging row carries no pose content of any kind (FENCE_SENSITIVE_LEVERS).
    if 'redacted' not in row:
        row['board_pose_sha256'] = file_pose_digest(output_file)


_PENDING: Dict[str, Dict] = {}


def commit_write(output_file: str) -> Optional[Dict]:
    """Finish the row `record_write(pending=True)` started, now the file exists.

    Split in two so the REFUSAL can happen before the write. The gate used to
    run after it, which made refusing decorative -- the poses were already on
    disk and the exception only described a file it had helped produce.
    """
    row = _PENDING.pop(os.path.abspath(output_file), None)
    if row is None:
        return None
    root = row.pop('_root')
    row['board_sha256'] = (sha256_file(output_file)
                           if os.path.isfile(output_file) else None)
    _stamp_board_pose(row, output_file)
    with open(os.path.join(root, LEDGER_NAME), 'a', encoding='utf-8') as f:
        f.write(json.dumps(row, sort_keys=True) + '\n')
    return row


def _side_changed(fp, placement) -> bool:
    """Did this placement ask to put `fp` on the other face (#714)?

    Reads the side through `legality.footprint_side` rather than re-deriving
    the first-character test here: this repo already carries that rule in one
    place and a second copy is a second thing to get wrong. Imported lazily,
    as this module already does for `kicad_parser`; `legality` imports nothing
    from here, so there is no cycle.
    """
    want = placement.get('new_side')
    if want is None:
        return False
    try:
        from placement.legality import footprint_side
    except ImportError:                                          # noqa: BLE001
        return True     # cannot tell -> report the change, never hide it
    return footprint_side(fp) != want


def record_write(input_file: str, output_file: str,
                 placements: Sequence[Dict],
                 pending: bool = False) -> Optional[Dict]:
    """Append one row for a pose write. Returns it, or None outside a regime.

    Raises `UnaidedViolation` when a regime is in force and no lever is
    declared. With `pending=True` the row is held until `commit_write`, so a
    refusal can precede the write rather than follow it.
    """
    root = regime_for(output_file)
    lever = active_lever()
    if root is None:
        return None
    if lever is None:
        raise UnaidedViolation(
            f"{os.path.basename(output_file)}: poses were written with no "
            f"registered lever, under the unaided regime at {root}. The "
            f"caller was {_caller()}. A run that claims the engine placed "
            f"this board cannot contain a pose this tool did not author -- "
            f"see placement/provenance.py.")
    if lever['lever'] not in LEVER_REGISTRY:
        raise UnaidedViolation(
            f"{os.path.basename(output_file)}: {lever['lever']!r} is not in "
            f"LEVER_REGISTRY, so it cannot author poses under the unaided "
            f"regime at {root}. Register it deliberately or run outside the "
            f"regime.")

    # refs_moved is SEPARATE from refs_written on purpose. `perturb.
    # _all_at_current` hands the writer EVERY part so that six-decimal `(at)`
    # reformatting cannot fingerprint the moved block; without the split every
    # row would claim the whole board and the audit would be vacuous.
    moved = []
    before = None
    try:
        before = pose_footprints(input_file)
        for p in placements:
            ref = p.get('reference')
            fp = before.get(ref)
            if fp is None:
                moved.append(ref)
                continue
            if (abs(fp.x - p.get('new_x', fp.x)) > 1e-6
                    or abs(fp.y - p.get('new_y', fp.y)) > 1e-6
                    or abs(((fp.rotation or 0.0)
                            - (p.get('new_rotation') or 0.0) + 180.0) % 360.0
                           - 180.0) > 1e-6):
                moved.append(ref)
            elif _side_changed(fp, p):
                # #714. A flip with identical x/y/rot is a real change to the
                # delivered board, and without this term it lands in
                # `refs_written` but NOT in `refs_moved`, with a
                # `poses_written` row byte-identical to the incumbent -- so
                # the ledger records the flip nowhere and `fence_audit` grades
                # a flipped board as untouched. That is a silent wrongness of
                # exactly the class #714 exists to remove, and it would have
                # been introduced by the fix for it.
                moved.append(ref)
    except Exception:                            # noqa: BLE001
        moved = [p.get('reference') for p in placements]

    # The POSE, not only the ref. A claim keyed on the ref alone says "the
    # engine touched C1", which a later hand edit of C1 then inherits: the
    # audit compares ref sets, finds C1 claimed, and grades CLEAN. Since a
    # real run legitimately moves most of the board, ref-keyed claims make
    # most of the board launderable, and the instrument is weakest exactly
    # where the run is most active. Recording where each ref was PUT lets the
    # audit compare the delivered geometry against the claim.
    _written = {}
    for p_ in placements:
        _ref = p_.get('reference')
        if not _ref:
            continue
        _fp = (before or {}).get(_ref) if isinstance(before, dict) else None
        _written[_ref] = [
            round(float(p_.get('new_x', getattr(_fp, 'x', 0.0) or 0.0)), 6),
            round(float(p_.get('new_y', getattr(_fp, 'y', 0.0) or 0.0)), 6),
            round(float(p_.get('new_rotation')
                        if p_.get('new_rotation') is not None
                        else (getattr(_fp, 'rotation', 0.0) or 0.0)) % 360.0,
                  4)]

    # #714. A SEPARATE key, deliberately not a fourth element of
    # `poses_written`: that list's shape is [x, y, rot] in every ledger ever
    # written, and only refs that actually carried a `new_side` appear here, so
    # an old ledger stays readable and `SCHEMA` does not move (`read_ledger` is
    # shape-agnostic and no consumer indexes past [2] -- `git grep
    # poses_written` finds only its own definition).
    _sides = {p_.get('reference'): p_.get('new_side')
              for p_ in placements
              if p_.get('reference') and p_.get('new_side') is not None}

    # The INPUT's arrangement, from the same parse `refs_moved` just used and
    # therefore from before the write -- an in-place write hashes the board it
    # replaces. None when the input could not be read; never an exception.
    _parent_pose = None
    if isinstance(before, dict):
        try:
            _parent_pose = pose_digest(pose_table_of(before))
        except Exception:                        # noqa: BLE001
            _parent_pose = None

    row = {'t': time.time(), 'schema': SCHEMA,
           'path': os.path.abspath(output_file),
           'parent_sha256': (sha256_file(input_file)
                             if os.path.isfile(input_file) else None),
           'parent_pose_sha256': _parent_pose,
           'lever': lever['lever'], 'lever_argv': lever['lever_argv'],
           'declared': True, 'caller': _caller(),
           'refs_written': sorted(p.get('reference') for p in placements),
           'poses_written': _written,
           'sides_written': _sides,
           'refs_moved': sorted(r for r in moved if r)}
    if lever['lever'] in FENCE_SENSITIVE_LEVERS:
        # A STAGING row states that a staging happened and nothing else. Its
        # argv names the source board and the truth dir, its `refs_moved` is
        # the perturbed block by name, and its poses are the control's -- all
        # of it inside the fence, in a file the run can read (see
        # FENCE_SENSITIVE_LEVERS). `parent_sha256` goes too: a hash is not a
        # path, but it turns "which board is this?" into a test the run can
        # run against every candidate on disk. `parent_pose_sha256` goes for
        # the same reason, and `_stamp_board_pose` adds no board digest to a
        # redacted row: the audit's root is the hash-verified staged FILE, so
        # nothing would read one.
        #
        # Dropping the pose keys is not a loss to the audit, it is more
        # correct: the staged board is the BASELINE the audit compares
        # against, never a claim about a delivered one.
        row = {'t': row['t'], 'schema': SCHEMA, 'path': row['path'],
               'lever': row['lever'], 'declared': True,
               'caller': row['caller'],
               'redacted': 'staging row: argv, parent hash and poses withheld '
                           '-- this ledger is inside the fence'}
    if pending:
        row['_root'] = root
        _PENDING[os.path.abspath(output_file)] = row
        return row
    row['board_sha256'] = (sha256_file(output_file)
                           if os.path.isfile(output_file) else None)
    _stamp_board_pose(row, output_file)
    with open(os.path.join(root, LEDGER_NAME), 'a', encoding='utf-8') as f:
        f.write(json.dumps(row, sort_keys=True) + '\n')
    return row


def read_ledger(root: str) -> List[Dict]:
    """Every row, tolerating a torn last line (board_store.Ledger's rule)."""
    path = os.path.join(root, LEDGER_NAME)
    out: List[Dict] = []
    if not os.path.isfile(path):
        return out
    with open(path, encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                out.append(json.loads(line))
            except ValueError:
                pass
    return out


def start_regime(workdir: str, staged_board: str, **extra) -> str:
    """Mark a work dir unaided and seed the chain with the staged board."""
    os.makedirs(workdir, exist_ok=True)
    doc = {'schema': SCHEMA, 'kind': 'unaided-regime',
           'staged_board': os.path.abspath(staged_board),
           'staged_sha256': sha256_file(staged_board),
           'lever_registry': list(LEVER_REGISTRY), **extra}
    path = os.path.join(workdir, REGIME_NAME)
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(doc, f, indent=1, sort_keys=True)
    return path


@contextlib.contextmanager
def recorded_delivery(input_file: str, output_file: str,
                      placements: Sequence[Dict]):
    """Record a delivery that is not a writer call -- a copy or a rename (#973).

    A lever that builds its board somewhere else and then `shutil.copy`s or
    `os.replace`s it onto the output leaves either no row at all or a row
    naming the intermediate: place_seed's `--repair`/`--reseat` staged in a
    temp dir and delivered with an EMPTY writer call, its polish wrote
    `<out>.polish` and renamed it, and place_route_loop copied its last round.
    Wrap the copy:

        with provenance.recorded_delivery(real_input, out, moves):
            shutil.copyfile(staged_board, out)

    `placements` are the moves the LEVER made, relative to `input_file` --
    never a diff of the two files, which would record whatever else the staged
    board carries as the lever's own work. If the delivered file disagrees
    with them, the audit's replay names the difference.

    The row is recorded BEFORE the body, so an undeclared caller is refused
    while the output is untouched; it is committed after the body, so its
    board digest is the delivered file's. A body that raises leaves no row.
    Outside a regime this does nothing and parses nothing.
    """
    row = record_write(input_file, output_file, placements, pending=True)
    if row is None:
        yield None
        return
    key = os.path.abspath(output_file)
    try:
        yield row
        # A writer call to the same path inside the body keys its own pending
        # row on this path and commits it; put this one back before committing.
        _PENDING[key] = row
        commit_write(output_file)
    finally:
        if _PENDING.get(key) is row:
            del _PENDING[key]
