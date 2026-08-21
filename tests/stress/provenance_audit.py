#!/usr/bin/env python3
"""Was every pose in this board produced by a registered engine lever?

    python3 -X utf8 tests/stress/provenance_audit.py --workdir DIR --mode audit

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
    from kicad_parser import parse_kicad_pcb
    pcb = parse_kicad_pcb(path)
    return {r: (f.x, f.y, f.rotation or 0.0)
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
        if (abs(pa[0] - pb[0]) > POSE_TOL_MM or abs(pa[1] - pb[1]) > POSE_TOL_MM
                or drot > POSE_TOL_DEG):
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
        delivered = max(cands, key=os.path.getmtime)

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
    claimed, undeclared = {}, {}
    for row in rows:
        lever = row.get('lever')
        ok = bool(row.get('declared')) and lever in PV.LEVER_REGISTRY
        for ref in row.get('refs_moved') or ():
            (claimed if ok else undeclared).setdefault(ref, lever or row.get(
                'caller', '<unknown>'))

    unclaimed = sorted(r for r in moved if r not in claimed)
    bad = sorted(r for r in moved if r in undeclared and r not in claimed)
    doc = {'workdir': os.path.abspath(workdir), 'staged': staged,
           'delivered': delivered, 'ledger_rows': len(rows),
           'moved': len(moved), 'added_refs': added[:40],
           'claimed': len(claimed),
           'unclaimed_refs': unclaimed[:40],
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
    doc.update(verdict='CLEAN', reason=(
        f"all {len(moved)} moved pose(s) trace to "
        f"{', '.join(doc['levers']) or 'no lever (nothing moved)'}"))
    return CLEAN, doc


def main(argv=None):
    p = argparse.ArgumentParser(
        description="Audit that every pose in a delivered board came from a "
                    "registered engine lever.")
    p.add_argument("--workdir", required=True)
    p.add_argument("--delivered", default=None,
                   help="The board to audit (default: newest .kicad_pcb in "
                        "the work dir that is not the staged one)")
    p.add_argument("--mode", choices=('audit',), default='audit')
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
         ('verdict', 'moved', 'claimed', 'ledger_rows', 'levers')},
        sort_keys=True))
    return code


if __name__ == "__main__":
    sys.exit(main())
