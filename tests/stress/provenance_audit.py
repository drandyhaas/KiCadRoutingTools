#!/usr/bin/env python3
"""Was every pose in this board produced by a registered engine lever?

    python3 -X utf8 tests/stress/provenance_audit.py --workdir DIR

The orthogonal sibling of `fence_audit`, which asks a DIFFERENT question --
"does any file in this work dir carry the control's poses?" -- and answers it
correctly every time. Run 19 passed it (`VERDICT: CLEAN, exit 0`) on a run
whose placement came from a 221-line hand script, because a hand-arranged
board matches the control at ~0.0. There was nothing for it to find. Nothing
in the repo asked the other question.

THE AUDIT IS A POSE RECONCILIATION, NOT A LOG READ, and that is what makes it
worth having. It computes which refs actually MOVED between the staged board
and the delivered one, then requires every one of them to be claimed by a row
with a registered lever. A hand script that edits `(at ...)` as raw text
appears in no row, so bypassing the instrument does not bypass the check --
the board's own geometry is the anchor.

AND THE CLAIMS MUST FORM A LINEAGE (#972). Every engine row carries the pose
digest of the board it read and of the board it wrote. Starting from the
staged board, a row whose input arrangement is already accounted for accounts
for its output, replayed from the poses it recorded; the delivered board must
be one of those arrangements, and every pose in it must be where that replay
puts it. The link is the ARRANGEMENT, not the file bytes or the path, so a
lock stamp, routed copper, a copy or an `os.replace` delivery all keep it. A
board no recorded write produced is compared with the nearest recorded one,
and the parts that differ are named. Before this, pose claims came only from
rows naming the delivered file, so a declared write of a hand-edited board to
a NEW path left the edit `unverifiable` and graded CLEAN.

Exit codes:

    0  CLEAN     every moved pose traces to a registered lever, and the board
                 is one the ledger's lineage produced (or matches the nearest
                 one it produced, pose for pose)
    2  usage / IO
    4  VIOLATION a moved pose has no lever, or a pose is not where the
                 lineage put it
    5  UNPROVEN  nothing can be concluded -- no manifest, a stale staged
                 board, no delivered board, no ledger and nothing moved, or a
                 recorded write read a board no recorded write produced and
                 re-moved every part that differed, so the change cannot be
                 named

5 is load-bearing. `fence_audit` collapses "no manifest" into LEAK and warns
about it in its own text; doing that here would retroactively accuse every run
that predates this instrument, which did nothing wrong. "I cannot prove it"
and "I proved it false" must be different numbers, and only an affirmative
finding produces 4.
"""
import argparse
import json
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(_HERE))
for _p in (ROOT, os.path.join(ROOT, 'py_router'),
           os.path.join(ROOT, 'py_tools'), os.path.join(ROOT, 'py_placer')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

CLEAN, USAGE, VIOLATION, UNPROVEN = 0, 2, 4, 5
POSE_TOL_MM = 1e-6
POSE_TOL_DEG = 1e-3
#: How far a delivered pose may sit from the pose a lever recorded. Looser
#: than POSE_TOL_* because a row stores rotation to 4 decimals and the board
#: holds the writer's `.6g`.
DRIFT_TOL_MM, DRIFT_TOL_DEG = 1e-3, 1e-2


def poses(path):
    """{ref: (x, y, rotation, side)}.

    The SIDE is the fourth element since #714 gave `write_placed_output` a
    real layer flip. A flip that holds its pose changes the delivered board
    and moves none of x/y/rot, so a three-element pose made it invisible to
    this audit -- and this audit's whole job is to reconcile what changed
    against what a declared lever claimed. `perturb`'s `layer_flip` produces
    exactly that board.

    CALLS the table the ledger's pose digests are computed from rather than
    re-deriving it: a second copy is a second thing to disagree about which
    footprint a key names.
    """
    from placement import provenance as PV
    return PV.pose_table(path)


def _pose_differs(got, want):
    """Beyond DRIFT_TOL? `want[3]` None says nothing about the side."""
    return (abs(got[0] - want[0]) > DRIFT_TOL_MM
            or abs(got[1] - want[1]) > DRIFT_TOL_MM
            or abs(((got[2] or 0.0) - (want[2] or 0.0) + 180.0) % 360.0
                   - 180.0) > DRIFT_TOL_DEG
            or (want[3] is not None and got[3] != want[3]))


def _well_formed(row):
    """A row the audit can read without guessing. Anything else is COUNTED
    and skipped: one malformed line used to raise a TypeError deep in the
    claims loop, and the CLI turned that into UNPROVEN -- so appending
    garbage to a ledger demoted a VIOLATION to "cannot conclude"."""
    if not isinstance(row, dict):
        return False

    def _names(v):
        return v is None or (isinstance(v, (list, tuple))
                             and all(isinstance(x, str) for x in v))

    if not (_names(row.get('refs_moved')) and _names(row.get('refs_written'))):
        return False
    pw = row.get('poses_written')
    if pw is not None:
        if not isinstance(pw, dict):
            return False
        for v in pw.values():
            if not (isinstance(v, (list, tuple)) and len(v) >= 3
                    and all(isinstance(x, (int, float))
                            and not isinstance(x, bool) for x in v[:3])):
                return False
    sw = row.get('sides_written')
    if sw is not None and not (isinstance(sw, dict) and all(
            x is None or isinstance(x, str) for x in sw.values())):
        return False
    return all(row.get(k) is None or isinstance(row.get(k), str)
               for k in ('path', 'lever', 'parent_pose_sha256',
                         'board_pose_sha256'))


def _usable(row, PV):
    """A row that can CLAIM: declared, registered, and not a staging row.
    A staging row is never a link -- it is redacted, and the baseline is the
    hash-verified staged FILE, not anything a row says about it."""
    return (bool(row.get('declared'))
            and row.get('lever') in PV.LEVER_REGISTRY
            and row.get('lever') not in PV.FENCE_SENSITIVE_LEVERS
            and 'redacted' not in row)


def _replay(table, row):
    """`table` with this row's MOVES applied. `refs_moved` only, never every
    `poses_written` entry: place_seed and perturb hand the writer every part,
    so replaying all of them would record whatever pose a part HAD -- a hand
    edit passed through a write-all lever would become a recorded pose."""
    out = dict(table)
    _poses = row.get('poses_written') or {}
    _sides = row.get('sides_written') or {}
    for ref in row.get('refs_moved') or ():
        p = _poses.get(ref)
        if p is None:
            continue
        old = out.get(ref)
        side = _sides.get(ref) or (old[3] if old is not None else None)
        out[ref] = (float(p[0]), float(p[1]), float(p[2]), side)
    return out


def _differing(delivered, expected):
    return sorted(r for r, got in delivered.items()
                  if r in expected and _pose_differs(got, expected[r]))


def lineage(rows, staged_table, delivered_table):
    """Which recorded arrangement is the delivered board, and what differs.

    Returns a dict: `status` is `verified` (the lineage produced exactly this
    arrangement), `broken` (a recorded write produced it from an input nothing
    recorded), `unrecorded` (no recorded write produced it), `legacy` (some
    claiming row predates the pose digests), or `unlinkable` (a digest is None
    or of an unknown scheme). `drift` names refs present in both the delivered
    board and the expected one whose poses differ beyond DRIFT_TOL.
    """
    from placement import provenance as PV
    usable = [(i, r) for i, r in enumerate(rows) if _usable(r, PV)]
    root = PV.pose_digest(staged_table)
    dg = PV.pose_digest(delivered_table)
    _pfx = PV.POSE_DIGEST_SCHEME + ':'

    def _linkable(d):
        return isinstance(d, str) and d.startswith(_pfx)

    # FORWARD, from the root only. "Some row produced my parent" is not
    # enough: two no-op writes of a hand-edited board would vouch for each
    # other. Repeated to a fixed point because ledger order is append-at-
    # COMMIT order, which need not be the order the boards were read in.
    known, made_by = {root: staged_table}, {root: None}
    grew = True
    while grew:
        grew = False
        for i, r in usable:
            b, p = r.get('board_pose_sha256'), r.get('parent_pose_sha256')
            if _linkable(b) and _linkable(p) and p in known and b not in known:
                known[b] = _replay(known[p], r)
                made_by[b] = i
                grew = True

    def _who(i):
        if i is None:
            return 'staged'
        return {'lever': rows[i].get('lever'),
                'path': os.path.basename(rows[i].get('path') or '')}

    detail = {'staged_pose_sha256': root, 'delivered_pose_sha256': dg,
              'states': len(known), 'tip': None, 'break': None,
              'compared_to': None, 'missing_refs': []}

    def _done(status, expected, compared_to):
        detail['compared_to'] = compared_to
        if expected is None:
            return {'status': status, 'drift': [], 'missing': [],
                    'expected': None, 'detail': detail}
        missing = sorted(r for r in expected if r not in delivered_table)
        detail['missing_refs'] = missing[:40]
        return {'status': status, 'drift': _differing(delivered_table, expected),
                'missing': missing, 'expected': expected, 'detail': detail}

    if dg in known:
        detail['tip'] = None if made_by[dg] is None else _who(made_by[dg])
        # Verified is not the end: the replay is what the rows CLAIM, and a
        # row whose file disagrees with its own claims is caught here.
        return _done('verified', known[dg], _who(made_by[dg]))

    if any('board_pose_sha256' not in r or 'parent_pose_sha256' not in r
           for _i, r in usable):
        return _done('legacy', None, None)
    if any(not (_linkable(r.get('board_pose_sha256'))
                and _linkable(r.get('parent_pose_sha256')))
           for _i, r in usable):
        return _done('unlinkable', None, None)

    # Walk BACK from the delivered arrangement through the newest row that
    # produced each digest, to the input nothing accounts for.
    producer = {}
    for i, r in usable:
        producer[r['board_pose_sha256']] = i
    chain, cur, seen = [], dg, set()
    while cur in producer and cur not in known and cur not in seen:
        seen.add(cur)
        i = producer[cur]
        chain.append(i)
        cur = rows[i].get('parent_pose_sha256')
    chain.reverse()                                    # oldest first
    if chain:
        detail['tip'] = _who(chain[-1])
        detail['break'] = dict(_who(chain[0]), parent_pose_sha256=rows[
            chain[0]].get('parent_pose_sha256'))
    moved_by_chain = {ref for i in chain for ref in rows[i].get('refs_moved') or ()}

    # The NEAREST recorded arrangement: the fewest parts that must have been
    # changed outside the ledger. Ties go to the most recent state, the staged
    # board last, so the choice never depends on dict order.
    def _rank(d):
        diff = [r for r in _differing(delivered_table, known[d])
                if r not in moved_by_chain]
        return (len(diff), -(made_by[d] if made_by[d] is not None else -1))

    nearest = min(known, key=_rank)
    expected = known[nearest]
    for i in chain:
        expected = _replay(expected, rows[i])
    return _done('broken' if chain else 'unrecorded', expected,
                 _who(made_by[nearest]))


def added_refs(a, b):
    """Refs present in the delivered board and absent from the staged one."""
    return sorted(r for r in b if r not in a)


def moved_refs(a, b):
    """Refs whose pose differs. Rotation compared MODULO 360, because the
    writer normalises -90 to 270 and a raw float compare would report an
    untouched part as moved."""
    out = []
    for ref, pb in sorted(b.items()):
        pa = a.get(ref)
        if pa is None:
            # ADDED, not moved. A ref absent from the staged board has no
            # pose to differ from, and counting it as moved made "someone
            # dropped a test point into the work dir" an unaided VIOLATION.
            # Adding a part is a real thing to disclose, so it is reported --
            # under its own name, by the caller.
            continue
        drot = abs(((pa[2] - pb[2]) + 180.0) % 360.0 - 180.0)
        # The side term is an OR, not a tolerance: a flip in place moves
        # nothing else, so without it the part reads as untouched (#714).
        if (abs(pa[0] - pb[0]) > POSE_TOL_MM or abs(pa[1] - pb[1]) > POSE_TOL_MM
                or drot > POSE_TOL_DEG or pa[3] != pb[3]):
            out.append(ref)
    return out


def audit(workdir, delivered=None):
    from placement import provenance as PV
    manifest = os.path.join(workdir, PV.REGIME_NAME)
    if not os.path.isfile(manifest):
        return UNPROVEN, {'verdict': 'UNPROVEN',
                          'reason': f'no {PV.REGIME_NAME}: this work dir was '
                                    f'not staged for an unaided run, so there '
                                    f'is no claim to check'}
    with open(manifest, encoding='utf-8') as f:
        regime = json.load(f)
    staged = regime.get('staged_board')
    if not staged or not os.path.isfile(staged):
        return UNPROVEN, {'verdict': 'UNPROVEN',
                          'reason': f'the staged board named by the manifest '
                                    f'is not readable: {staged!r}'}
    # ...AND IS THE BOARD THE MANIFEST DESCRIBES. `staged_sha256` was written
    # and read by nobody, so "readable" was the whole check -- and the whole
    # audit is a comparison AGAINST this file, so a stale one silently moves
    # the baseline and every verdict computed from it is about the wrong
    # question.
    #
    # Reachable without anyone acting in bad faith: `stage()` writes the board
    # first and arms last, and the steps between (sibling copy, project
    # sanitise, mechanical.json) can raise OSError, which the CLI turns into
    # exit 2 AFTER the board has already been replaced. Measured: staging a
    # different source into a dir whose `mechanical.json` could not be written
    # left the old manifest describing bytes that were no longer there, and
    # nothing reported it.
    _sha = regime.get('staged_sha256')
    if _sha and _sha != PV.sha256_file(staged):
        return UNPROVEN, {'verdict': 'UNPROVEN', 'staged': staged,
                          'reason': f'the manifest describes a DIFFERENT '
                                    f'board than the one at {staged}: it '
                                    f'records {_sha[:16]}..., the file hashes '
                                    f'{PV.sha256_file(staged)[:16]}.... The '
                                    f'baseline every verdict here is measured '
                                    f'against is stale, so nothing can be '
                                    f'concluded -- re-stage the work dir'}

    if delivered is None:
        # Newest .kicad_pcb, EXCLUDING intermediates. The chain routinely
        # drops `*.staging.kicad_pcb` and `*.polish` next to a board, and
        # picking one used to yield a soft UNPROVEN -- harmless. Now that a
        # ledger-less moved pose is a VIOLATION, a mis-picked artifact is an
        # affirmative accusation, so the guess has to be narrower.
        ARTIFACTS = ('.staging.kicad_pcb', '.polish.kicad_pcb',
                     '_before.kicad_pcb', '_control.kicad_pcb')
        cands = [os.path.join(workdir, n) for n in sorted(os.listdir(workdir))
                 if n.endswith('.kicad_pcb')
                 and not any(n.endswith(a) for a in ARTIFACTS)
                 and os.path.abspath(os.path.join(workdir, n))
                 != os.path.abspath(staged)]
        if not cands:
            return UNPROVEN, {'verdict': 'UNPROVEN',
                              'reason': 'no delivered board in the work dir '
                                        '(staging artifacts are not one)'}
        # THE LEDGER NAMES IT. Every row carries the `path` its lever wrote,
        # so the newest row is a STATEMENT about which board the chain
        # produced, where mtime is a guess -- and the comment above already
        # says the guess has to be narrow now that a mis-pick is an
        # affirmative accusation. mtime remains the fallback for a work dir
        # whose ledger names no board that is still present.
        # A STAGING row is never a delivered board -- it is a baseline, the
        # same thing `staged` already is, and the exclusion above says so for
        # this dir's own staged board.
        #
        # Without this a NESTED staging launders a violation into CLEAN. A
        # stage into `<workdir>/inner` writes its board before its own
        # manifest exists, so `regime_for` binds that write to the OUTER
        # regime and appends a row whose `path` points into `inner`. Being
        # the newest row it became the outer dir's "delivered board", and the
        # real one was never audited: measured, a hand-edited delivered board
        # went from `UNAIDED VIOLATION, 8 poses not where their lever put
        # them` (exit 4) to `CLEAN` (exit 0) purely by staging a
        # sub-experiment underneath it. The nested stage still prints its
        # stderr NOTE; a note hours earlier is not a defence against the
        # audit reading the wrong file.
        by_ledger = None
        for r in reversed([x for x in PV.read_ledger(workdir)
                           if _well_formed(x)]):
            p = r.get('path')
            if r.get('lever') in PV.FENCE_SENSITIVE_LEVERS:
                continue
            if p and os.path.isfile(p) \
                    and os.path.abspath(p) != os.path.abspath(staged):
                by_ledger = p
                break
        delivered = by_ledger or max(cands, key=os.path.getmtime)

    _read = PV.read_ledger(workdir)
    rows = [r for r in _read if _well_formed(r)]
    malformed = len(_read) - len(rows)
    _sp, _dp = poses(staged), poses(delivered)
    moved = moved_refs(_sp, _dp)
    added = added_refs(_sp, _dp)
    if not rows:
        # COMPUTE `moved` FIRST. This returned UNPROVEN before looking at the
        # board, which swallowed the exact case the instrument was built for:
        # a purely hand-placed board has no ledger BECAUSE nothing engine-side
        # ran, and it came back 5 ("I cannot prove it") instead of 4 ("I
        # proved it false"). Those two must be different numbers -- it is the
        # reason this file has four exit codes -- and the board itself
        # distinguishes them. No ledger AND no movement is genuinely
        # unproven; no ledger and 65 moved parts is a violation with a
        # witness.
        if moved:
            return VIOLATION, {
                'verdict': 'UNAIDED VIOLATION', 'delivered': delivered,
                'staged': staged, 'ledger_rows': 0, 'moved': len(moved),
                'claimed': 0, 'unclaimed_refs': moved[:40],
                'added_refs': added[:40],
                'undeclared_refs': {}, 'levers': [], 'callers': [],
                'reason': (
                    f"{len(moved)} pose(s) differ from the staged board and "
                    f"there is NO ledger at all -- nothing engine-side wrote "
                    f"them. This is the hand-placed case, not an unmeasured "
                    f"one: the board is the witness.")}
        return UNPROVEN, {
            'verdict': 'UNPROVEN', 'delivered': delivered,
            'moved': 0,
            'reason': 'no pose-provenance ledger AND no pose differs from the '
                      'staged board: this run predates the instrument, or '
                      'nothing wrote a pose. Not a violation -- nothing was '
                      'measured and nothing moved.'}
    # NEWEST claim wins, and it carries the POSE the lever wrote. Keyed on
    # the ref alone, a claim is inheritable: the engine legitimately moves C1,
    # and a later hand edit of C1 then rides that claim to CLEAN. Since a real
    # run moves most of the board, ref-keyed claims leave most of the board
    # launderable -- the instrument would be weakest exactly where the run is
    # most active, which is the opposite of what it is for.
    #
    # THE POSE CLAIM IS PER FILE. `claimed` spans the whole ledger -- "some
    # registered lever touched this ref" is a statement about the run -- but
    # `claim_pose` may only come from rows that wrote THE BOARD BEING
    # AUDITED. One lever writing several candidates and keeping one is the
    # loop's NORMAL shape (`place_route_loop`, `place_seed`, `place_portfolio`
    # all do it), and taking the newest row for a ref regardless of its target
    # convicted the kept board of being "not where the lever put it" -- the
    # lever put it there, in a different file, which was then discarded.
    # Measured: a two-candidate lap graded exit 4 on all three moved refs with
    # nothing hand-edited. An instrument that cries wolf on the normal path is
    # worse than no instrument.
    # ... but scoping ALONE goes blind on a board the ledger never names.
    # `place_route_loop` delivered by `shutil.copy(cur_file,
    # args.output_file)`, so no row's `path` was the delivered file and EVERY
    # ref fell to `unverifiable_claims` -- measured: a hand-move of C1 by
    # +37/+21 mm in that board graded CLEAN, which is the laundering this
    # check exists to catch, on the rig's own main output. So: scope only
    # when the ledger DOES name the delivered file; when it names it nowhere,
    # fall back to the whole ledger and check the poses anyway.
    #
    # AND SCOPING BY PATH WAS ITSELF A LAUNDERING CHANNEL (#972): once any row
    # named the delivered file, an earlier hand edit carried into it by a
    # declared write to a NEW path had no pose to compare, fell to
    # `unverifiable`, and graded CLEAN. Since the rows carry pose digests the
    # LINEAGE below decides instead -- by arrangement, not by path -- and this
    # per-file claim is read only for a ledger whose claiming rows predate the
    # digests (`legacy`).
    delivered_abs = os.path.normcase(os.path.abspath(delivered))

    def _same_file(rp):
        # normcase, not bare abspath: on Windows `C:\...\Work` and
        # `c:\...\work` are the same file, and a case difference silently
        # disabled the pose check.
        return os.path.normcase(os.path.abspath(rp or '')) == delivered_abs

    _names_delivered = any(_same_file(r.get('path')) for r in rows)
    claimed, undeclared, claim_pose = {}, {}, {}
    # Ledger order, not the recorded clock: append order is chronological by
    # construction, and a backward clock step between two writes would
    # otherwise reverse which claim wins.
    for row in rows:
        lever = row.get('lever')
        ok = bool(row.get('declared')) and lever in PV.LEVER_REGISTRY
        _poses = row.get('poses_written') or {}
        # #714's separate key, read with a bare `.get`: it is absent on every
        # row written before that change, and a claim carrying no side is
        # `unverifiable` for the side rather than a mismatch.
        _sides = row.get('sides_written') or {}
        _wrote_this = (not _names_delivered) or _same_file(row.get('path'))
        # A digest-era row whose INPUT could not be parsed: `record_write`
        # then counts every placement it was handed as moved, at whatever
        # pose it was handed, so its poses vouch for nothing -- a write-all
        # lever would record a hand edit as its own claim.
        _blind = ('parent_pose_sha256' in row
                  and row.get('parent_pose_sha256') is None)
        for ref in row.get('refs_moved') or ():
            if ok:
                claimed[ref] = lever or row.get('caller', '<unknown>')
                if _wrote_this and ref in _poses and not _blind:
                    claim_pose[ref] = tuple(_poses[ref]) + (_sides.get(ref),)
            else:
                undeclared.setdefault(ref, lever or row.get(
                    'caller', '<unknown>'))

    # A claim is only good for the pose it claimed. Two kinds of ref have no
    # pose to compare and both stay ref-keyed rather than being failed for it:
    # a row written before `poses_written` existed, and a ref whose only
    # claiming rows targeted a DIFFERENT file (a board copied into place, or
    # a delivered board this ledger never names). Both are named in the doc as
    # `unverifiable_claims`, so "not checked" cannot be mistaken for "checked".
    unclaimed = sorted(r for r in moved if r not in claimed)
    lin = lineage(rows, _sp, _dp)
    drifted, unverifiable = [], []
    if lin['status'] in ('legacy', 'unlinkable'):
        for ref in moved:
            if ref not in claimed:
                continue
            want = claim_pose.get(ref)
            if want is None:
                unverifiable.append(ref)
                continue
            got = _dp.get(ref)        # poses() -> (x, y, rotation, side)
            if got is None:
                continue
            # `want[3]` is None on a claim that named no side -- every
            # pre-#714 row, and every write that did not flip. A None claim is
            # not a mismatch; it simply says nothing about the side, which is
            # the same thing `unverifiable_claims` already says about a
            # missing pose.
            if _pose_differs(got, want):
                drifted.append(ref)
        # #972 for a ledger with no lineage to walk: a claim with no pose to
        # compare is still WRONG when the delivered pose is one no row ever
        # recorded for that ref, and not the staged pose either.
        # MOVES only, as `_replay` reads them: a write-all lever records every
        # pose it was handed, a hand edit included, and counting those as
        # "recorded" let one pre-digest row reopen #972.
        # A row whose input could not be parsed says nothing either way: its
        # refs stay unverifiable rather than being accused on its account.
        _recorded, _unreadable = {}, set()
        for row in rows:
            if not _usable(row, PV):
                continue
            if ('parent_pose_sha256' in row
                    and row.get('parent_pose_sha256') is None):
                _unreadable.update(row.get('refs_moved') or ())
                continue
            _sides = row.get('sides_written') or {}
            _poses = row.get('poses_written') or {}
            for ref in row.get('refs_moved') or ():
                p = _poses.get(ref)
                if p is not None:
                    _recorded.setdefault(ref, []).append(
                        (p[0], p[1], p[2], _sides.get(ref)))
        for ref in list(unverifiable):
            got = _dp.get(ref)
            if got is None or ref in _unreadable:
                continue
            if all(_pose_differs(got, w) for w in
                   _recorded.get(ref, []) + [_sp.get(ref)] if w is not None):
                unverifiable.remove(ref)
                drifted.append(ref)
    else:
        # The lineage names every part whose pose is not where the recorded
        # writes put it, moved relative to the staged board or not: a hand
        # REVERT of an engine move is a hand placement too. A ref with no
        # claim at all is already `unclaimed`, which says more.
        drifted = [r for r in lin['drift'] if r not in unclaimed]
        # A RENAME is invisible to a per-ref comparison: the old ref is
        # missing, the new one is "added", and neither is compared -- so
        # renaming a part and moving it graded CLEAN. An added part that is
        # not where a missing part was expected is a pose no lever wrote.
        if lin['missing'] and added:
            _gone = [lin['expected'][m] for m in lin['missing']]
            unclaimed = sorted(set(unclaimed) | {
                a for a in added
                if all(_pose_differs(_dp[a], w) for w in _gone)})

    drifted = sorted(drifted)
    unverifiable = sorted(unverifiable)
    bad = sorted(r for r in moved if r in undeclared and r not in claimed)
    doc = {'workdir': os.path.abspath(workdir), 'staged': staged,
           'delivered': delivered, 'ledger_rows': len(rows),
           'moved': len(moved), 'added_refs': added[:40],
           'claimed': len(claimed),
           'unclaimed_refs': unclaimed[:40],
           'drifted_refs': drifted[:40],
           'unverifiable_claims': unverifiable[:40],
           'undeclared_refs': {r: undeclared[r] for r in bad[:40]},
           'levers': sorted({r.get('lever') for r in rows if r.get('lever')}),
           'callers': sorted({r.get('caller') for r in rows
                              if r.get('caller')})[:10],
           'lineage': lin['status'],
           'lineage_detail': dict(lin['detail'], malformed_rows=malformed)}
    _det = lin['detail']
    _brk = _det.get('break')
    _cmp = _det.get('compared_to')
    _cmp_s = (_cmp if isinstance(_cmp, str)
              else f"the {_cmp['lever']} write to {_cmp['path']}" if _cmp
              else 'nothing')
    if lin['status'] == 'broken':
        _how = (f" The board descends from a {_brk['lever']} write to "
                f"{_brk['path']} whose input matches no board the ledger "
                f"recorded; compared with {_cmp_s}.")
    elif lin['status'] == 'unrecorded':
        _how = (f" No recorded write produced this board; compared with the "
                f"nearest one the ledger did, {_cmp_s}.")
    else:
        _how = ''
    if unclaimed:
        doc.update(verdict='UNAIDED VIOLATION', reason=(
            f"{len(unclaimed)} moved pose(s) trace to no registered lever. "
            f"A hand-authored pose reaches the board without a ledger row "
            f"whatever tool it bypassed, because this compares the BOARD, "
            f"not the log.{_how}"))
        return VIOLATION, doc
    if drifted:
        doc.update(verdict='UNAIDED VIOLATION', reason=(
            f"{len(drifted)} pose(s) are NOT where the lever that claims them "
            f"put them ({', '.join(drifted[:6])}). The ref is claimed, the "
            f"POSE is not: something moved it after the engine wrote it. A "
            f"claim keyed on the ref alone would have graded this CLEAN, "
            f"which is how a hand edit of a part the engine legitimately "
            f"touched becomes invisible.{_how}"))
        return VIOLATION, doc
    if lin['missing']:
        doc.update(verdict='UNPROVEN', reason=(
            f"{len(lin['missing'])} part(s) the recorded writes placed are "
            f"not on this board ({', '.join(lin['missing'][:6])}): a deleted "
            f"or renamed part has no pose left to compare. Not a violation, "
            f"and not provably clean.{_how}"))
        return UNPROVEN, doc
    if lin['status'] == 'broken':
        # Something moved outside the ledger -- the write's input is no board
        # any recorded write produced -- but that write moved again every
        # part that differs, so there is no pose left to name. Not a finding
        # (4) and not a pass (0).
        doc.update(verdict='UNPROVEN', reason=(
            f"a {_brk['lever']} write to {_brk['path']} read a board no "
            f"recorded write produced, and re-moved every part that differs "
            f"from {_cmp_s}, so what changed outside the ledger cannot be "
            f"named. Not a violation, and not provably clean."))
        return UNPROVEN, doc
    if lin['status'] == 'unlinkable' and unverifiable:
        doc.update(verdict='UNPROVEN', reason=(
            f"the ledger's pose digests do not link (a digest is missing or "
            f"of an unknown scheme), and {len(unverifiable)} claim(s) have no "
            f"pose to compare ({', '.join(unverifiable[:6])})."))
        return UNPROVEN, doc
    # Name BOTH causes. Saying "predate `poses_written`" about a ledger this
    # run wrote seconds ago sends the reader looking for an old ledger that
    # does not exist; the usual cause now is a claiming row that wrote a
    # different file.
    _unv = (f" ({len(unverifiable)} claim(s) matched by ref only: the "
            f"claiming row wrote a different file, or predates "
            f"`poses_written`)" if unverifiable else '')
    _clean_how = (f" No recorded write produced exactly this board, and every "
                  f"shared part matches {_cmp_s} pose for pose."
                  if lin['status'] == 'unrecorded' else '')
    doc.update(verdict='CLEAN', reason=(
        f"all {len(moved)} moved pose(s) trace to "
        f"{', '.join(doc['levers']) or 'no lever (nothing moved)'}"
        f", and each is where its lever put it{_unv}.{_clean_how}"))
    return CLEAN, doc


def main(argv=None):
    p = argparse.ArgumentParser(
        description="Audit that every pose in a delivered board came from a "
                    "registered engine lever.")
    p.add_argument("--workdir", required=True)
    p.add_argument("--delivered", default=None,
                   help="The board to audit (default: the newest board a "
                        "ledger row NAMES, else the newest .kicad_pcb in the "
                        "work dir that is not the staged one)")
    p.add_argument("--json", metavar="PATH")
    a = p.parse_args(argv)

    if not os.path.isdir(a.workdir):
        print(f"provenance_audit: no such work dir: {a.workdir}",
              file=sys.stderr)
        return USAGE
    try:
        code, doc = audit(a.workdir, a.delivered)
    except Exception as e:                       # noqa: BLE001
        print(f"provenance_audit: {type(e).__name__}: {e}", file=sys.stderr)
        return UNPROVEN

    print(f"VERDICT: {doc['verdict']}")
    print(f"  {doc['reason']}")
    if doc.get('unclaimed_refs'):
        print(f"  unclaimed: {', '.join(doc['unclaimed_refs'][:12])}")
    for ref, who in (doc.get('undeclared_refs') or {}).items():
        print(f"    {ref}: written by {who}, undeclared")
    if doc.get('lineage') not in (None, 'verified'):
        _d = doc.get('lineage_detail') or {}
        print(f"  lineage: {doc['lineage']} (compared with "
              f"{_d.get('compared_to')}; break at {_d.get('break')})")
    if a.json:
        with open(a.json, 'w', encoding='utf-8') as f:
            json.dump(doc, f, indent=1, sort_keys=True)
    print("JSON_SUMMARY: " + json.dumps(
        {k: doc.get(k) for k in
         # `unverifiable_claims` rides in the summary because run_watch
         # scrapes only VERDICT/unclaimed lines: without it a board whose
         # poses were never checked prints an unqualified CLEAN.
         ('verdict', 'moved', 'claimed', 'ledger_rows', 'levers',
          'unverifiable_claims', 'lineage')},
        sort_keys=True))
    return code


if __name__ == "__main__":
    sys.exit(main())
