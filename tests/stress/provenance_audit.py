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
in the ledger's parent-chain with a registered lever. A hand script that edits
`(at ...)` as raw text appears in no row, so bypassing the instrument does not
bypass the check -- the board's own geometry is the anchor.

Exit codes:

    0  CLEAN     every moved pose traces to a registered lever
    2  usage / IO
    4  VIOLATION a moved pose has no lever, or a row says declared: false
    5  UNPROVEN  the chain is broken, not violated -- no ledger, unreadable,
                 or a sha appearing nowhere

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


def poses(path):
    """{ref: (x, y, rotation, side)}.

    The SIDE is the fourth element since #714 gave `write_placed_output` a
    real layer flip. A flip that holds its pose changes the delivered board
    and moves none of x/y/rot, so a three-element pose made it invisible to
    this audit -- and this audit's whole job is to reconcile what changed
    against what a declared lever claimed. `perturb`'s `layer_flip` produces
    exactly that board.
    """
    from kicad_parser import parse_kicad_pcb
    from placement.legality import footprint_side
    pcb = parse_kicad_pcb(path)
    return {r: (f.x, f.y, f.rotation or 0.0, footprint_side(f))
            for r, f in pcb.footprints.items()}


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
        by_ledger = None
        for r in reversed(PV.read_ledger(workdir)):
            p = r.get('path')
            if p and os.path.isfile(p) \
                    and os.path.abspath(p) != os.path.abspath(staged):
                by_ledger = p
                break
        delivered = by_ledger or max(cands, key=os.path.getmtime)

    rows = PV.read_ledger(workdir)
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
    # `place_route_loop.py:737` delivers by `shutil.copy(cur_file,
    # args.output_file)`, so no row's `path` is the delivered file and EVERY
    # ref falls to `unverifiable_claims` -- measured: a hand-move of C1 by
    # +37/+21 mm in that board graded CLEAN, which is the laundering this
    # check exists to catch, on the rig's own main output. So: scope only
    # when the ledger DOES name the delivered file; when it names it nowhere,
    # fall back to the whole ledger and check the poses anyway. The two
    # failure modes are asymmetric -- crying wolf on the normal path costs a
    # false accusation, going blind costs the finding.
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
        for ref in row.get('refs_moved') or ():
            if ok:
                claimed[ref] = lever or row.get('caller', '<unknown>')
                if _wrote_this and ref in _poses:
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
    _TOL_MM, _TOL_DEG = 1e-3, 1e-2
    drifted, unverifiable = [], []
    for ref in moved:
        if ref not in claimed:
            continue
        want = claim_pose.get(ref)
        if want is None:
            unverifiable.append(ref)
            continue
        got = _dp.get(ref)            # poses() -> (x, y, rotation, side)
        if got is None:
            continue
        # `want[3]` is None on a claim that named no side -- every pre-#714
        # row, and every write that did not flip. A None claim is not a
        # mismatch; it simply says nothing about the side, which is the same
        # thing `unverifiable_claims` already says about a missing pose.
        if (abs(got[0] - want[0]) > _TOL_MM
                or abs(got[1] - want[1]) > _TOL_MM
                or abs(((got[2] or 0.0) - want[2] + 180.0) % 360.0 - 180.0)
                > _TOL_DEG
                or (want[3] is not None and got[3] != want[3])):
            drifted.append(ref)

    unclaimed = sorted(r for r in moved if r not in claimed)
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
                              if r.get('caller')})[:10]}
    if unclaimed:
        doc.update(verdict='UNAIDED VIOLATION', reason=(
            f"{len(unclaimed)} moved pose(s) trace to no registered lever. "
            f"A hand-authored pose reaches the board without a ledger row "
            f"whatever tool it bypassed, because this compares the BOARD, "
            f"not the log."))
        return VIOLATION, doc
    if drifted:
        doc.update(verdict='UNAIDED VIOLATION', reason=(
            f"{len(drifted)} pose(s) are NOT where the lever that claims them "
            f"put them ({', '.join(drifted[:6])}). The ref is claimed, the "
            f"POSE is not: something moved it after the engine wrote it. A "
            f"claim keyed on the ref alone would have graded this CLEAN, "
            f"which is how a hand edit of a part the engine legitimately "
            f"touched becomes invisible."))
        return VIOLATION, doc
    # Name BOTH causes. Saying "predate `poses_written`" about a ledger this
    # run wrote seconds ago sends the reader looking for an old ledger that
    # does not exist; the usual cause now is a claiming row that wrote a
    # different file.
    _unv = (f" ({len(unverifiable)} claim(s) matched by ref only: the "
            f"claiming row wrote a different file, or predates "
            f"`poses_written`)" if unverifiable else '')
    doc.update(verdict='CLEAN', reason=(
        f"all {len(moved)} moved pose(s) trace to "
        f"{', '.join(doc['levers']) or 'no lever (nothing moved)'}"
        f", and each is where its lever put it{_unv}"))
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
    if a.json:
        with open(a.json, 'w', encoding='utf-8') as f:
            json.dump(doc, f, indent=1, sort_keys=True)
    print("JSON_SUMMARY: " + json.dumps(
        {k: doc.get(k) for k in
         # `unverifiable_claims` rides in the summary because run_watch
         # scrapes only VERDICT/unclaimed lines: without it a board whose
         # poses were never checked prints an unqualified CLEAN.
         ('verdict', 'moved', 'claimed', 'ledger_rows', 'levers',
          'unverifiable_claims')},
        sort_keys=True))
    return code


if __name__ == "__main__":
    sys.exit(main())
