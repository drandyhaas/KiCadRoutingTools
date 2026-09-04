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
    'perturb.py', 'stage_blind.py',
)

# Registered but NOT pose writers. `beautify_labels.py` moves reference-
# designator silkscreen through `write_label_output`, which is not the pose
# funnel -- it never records and never raises, so listing it here would be a
# decorative entry that implies a coverage this instrument does not have.
# Named rather than silently omitted, because "why is it missing" is a
# question someone will ask.
NOT_POSE_WRITERS = ('beautify_labels.py',)

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
        from kicad_parser import parse_kicad_pcb
        before = parse_kicad_pcb(input_file).footprints
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

    row = {'t': time.time(), 'schema': SCHEMA,
           'path': os.path.abspath(output_file),
           'parent_sha256': (sha256_file(input_file)
                             if os.path.isfile(input_file) else None),
           'lever': lever['lever'], 'lever_argv': lever['lever_argv'],
           'declared': True, 'caller': _caller(),
           'refs_written': sorted(p.get('reference') for p in placements),
           'poses_written': _written,
           'sides_written': _sides,
           'refs_moved': sorted(r for r in moved if r)}
    if pending:
        row['_root'] = root
        _PENDING[os.path.abspath(output_file)] = row
        return row
    row['board_sha256'] = (sha256_file(output_file)
                           if os.path.isfile(output_file) else None)
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
