#!/usr/bin/env python3
"""#714 consumer: the `layer_flip` damage kind, and what it must NOT disturb.

`placement/perturb.py`'s module docstring listed a layer flip as one of two
mutations deliberately NOT offered, because the writer could not emit one. It
can now, so the kind exists -- and this gate holds it to the three properties
that make it a damage kind rather than a broad rewrite.

THE ONE THAT IS EASY TO GET WRONG is the third. `perturb._all_at_current` hands
the writer EVERY part, deliberately: it exists so the moved set cannot be read
off six-decimal `(at)` formatting, which is how run 14's perturbed block leaked
(24 of 235 blocks, recoverable with no tooling and no truth). If the flip's
side key reached those pass-through dicts, the kind would flip the whole board;
if it failed to reach the members, it would flip nothing. Both look plausible in
a summary. So the flipped set is compared against the member set EXACTLY, not
counted.

And the CONTROL board -- the dose-0 pass that is this rig's ground truth -- must
be byte-identical to what it was before this kind existed, because a control
formatted differently from its subject is itself a comparison anyone could make.

    python3 tests/test_714_perturb_layer_flip.py
"""
from __future__ import annotations

import hashlib
import os
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, os.path.join(REPO, 'py_placer'))
sys.path.insert(0, TESTS)

RUN_ALL_TIMEOUT = 600
BOARD = os.path.join(REPO, 'kicad_files', 'tigard.kicad_pcb')

# tigard flips 33 of its 92 blocks through the default unit. Floored, not
# pinned: a board edit may move the number, but a kind that damages one part
# or the whole board is not the kind this is.
MIN_FLIPPED = 5


def _sha(path):
    with open(path, 'rb') as fh:
        return hashlib.sha256(fh.read()).hexdigest()


def main():
    from kicad_parser import parse_kicad_pcb
    from placement.legality import PartPads, footprint_side
    from placement.perturb import KINDS, perturb, _all_at_current
    from placement.writer import write_placed_output

    problems = []
    if 'layer_flip' not in KINDS:
        print("FAIL: perturb.KINDS has no 'layer_flip'")
        return 1
    # `wrong_side` is a point reflection through the board centre and carries
    # published numbers in docs/placement-predictors.md and two committed
    # baselines. It must still be there, and it must still be a different kind.
    if 'wrong_side' not in KINDS:
        problems.append("'wrong_side' was removed or renamed -- its measured "
                        "numbers are recorded in two committed baselines")

    tmp = tempfile.mkdtemp(prefix='krt714perturb')
    try:
        out = os.path.join(tmp, 'flipped.kicad_pcb')
        ctrl = os.path.join(tmp, '_truth', 'control.kicad_pcb')
        rec = perturb(BOARD, out, kind='layer_flip', dose_mm=0.0,
                      control_out=ctrl, write_record=False)
        if rec.get('status') != 'ok':
            print(f"FAIL: perturb returned {rec.get('status')}: "
                  f"{rec.get('reason')}")
            return 1

        members = set(rec['block']['members'])
        a = parse_kicad_pcb(BOARD).footprints
        b = parse_kicad_pcb(out).footprints

        flipped = {r for r in b
                   if r in a and footprint_side(a[r]) != footprint_side(b[r])}

        # EXACT, not a count. A side key leaking into `_all_at_current`'s
        # pass-through dicts flips the whole board; one that never reaches the
        # members flips nothing. Both read as plausible in a summary.
        if flipped != {r for r in members if r in b}:
            problems.append(
                f"the flipped set is not the member set: "
                f"{len(flipped)} flipped, {len(members)} members, "
                f"{len(flipped - members)} flipped that were not asked for, "
                f"{len(members - flipped)} asked for that did not flip")
        if len(flipped) < MIN_FLIPPED:
            problems.append(f"only {len(flipped)} part(s) flipped -- a kind "
                            f"that damages one part is not a damage class")
        if len(flipped) == len(b):
            problems.append("EVERY block flipped -- the side key reached the "
                            "pass-through dicts in _all_at_current")

        # The damaged board must not contradict itself: this is the whole
        # point of the capability, checked on the rig that consumes it.
        for ref in sorted(flipped)[:40]:
            pp = PartPads(b[ref], clearance=0.2)
            if pp.pad_sides != {'F', 'B'} and pp.pad_sides != {pp.side}:
                problems.append(
                    f"{ref}: fp.layer says {pp.side}, its pads say "
                    f"{sorted(pp.pad_sides)}")

        # The CONTROL is this rig's ground truth, and it must be
        # KIND-INDEPENDENT: `_all_at_current(st, [], pcb)` is the same call
        # whatever kind was asked for, so a control that differs between kinds
        # means the side key leaked into the control path. Compared against
        # another kind's control rather than against a hand-built identity
        # write, because `_all_at_current` walks `state.parts` -- which
        # EXCLUDES the zero-pad blocks `pcb.footprints` carries -- and the
        # writer reformats `(at)` for every ref it is handed, so a hand-built
        # list is a different call and differs for a reason that is not a
        # regression. (Measured: it does differ, which is what this comment is
        # here to stop the next reader re-discovering.)
        other = os.path.join(tmp, 'other.kicad_pcb')
        other_ctrl = os.path.join(tmp, '_truth2', 'control.kicad_pcb')
        perturb(BOARD, other, kind='translate', dose_mm=0.0,
                control_out=other_ctrl, write_record=False)
        if _sha(ctrl) != _sha(other_ctrl):
            problems.append(
                "the dose-0 CONTROL board differs between `layer_flip` and "
                "`translate` -- ground truth is supposed to be the same board "
                "whatever kind was asked for, so the side key has leaked into "
                "the control path")
        # The LEDGER must see the flip. Before #714 `record_write` derived
        # `moved` from x/y/rot only and wrote `poses_written` as [x, y, rot],
        # so a flip holding its pose landed in `refs_written` and NOT in
        # `refs_moved`, with a pose claim byte-identical to the incumbent --
        # `fence_audit` and every provenance consumer would grade a flipped
        # board as untouched. That would have been a new silent wrongness of
        # the very class this issue removes, shipped by the fix for it.
        # `record_write` answers None outside a regime, so the regime is armed
        # here the way `tests/test_provenance_audit.py` arms one -- otherwise
        # this arm would "pass" by never running.
        from placement import provenance
        wd = os.path.join(tmp, 'wk')
        os.makedirs(wd, exist_ok=True)
        staged = os.path.join(wd, 'board.kicad_pcb')
        shutil.copyfile(BOARD, staged)
        provenance.start_regime(wd, staged)
        ledger_out = os.path.join(wd, 'flipped.kicad_pcb')
        shutil.copyfile(out, ledger_out)

        ref = sorted(flipped)[0]
        with provenance.declare_lever('place_optimize.py',
                                      ['place_optimize.py', staged]):
            row = provenance.record_write(
                staged, ledger_out,
                [{'reference': ref, 'new_x': round(a[ref].x, 6),
                  'new_y': round(a[ref].y, 6),
                  'new_rotation': a[ref].rotation,
                  'new_side': footprint_side(b[ref])}])
            row2 = provenance.record_write(
                staged, ledger_out,
                [{'reference': ref, 'new_x': round(a[ref].x, 6),
                  'new_y': round(a[ref].y, 6),
                  'new_rotation': a[ref].rotation}])
        if row is None:
            problems.append("record_write returned no row -- the regime was "
                            "not armed, so this arm proves nothing")
        else:
            if ref not in (row.get('refs_moved') or ()):
                problems.append(
                    f"{ref} flipped but the ledger does not call it moved: "
                    f"refs_moved={row.get('refs_moved')}")
            if (row.get('sides_written') or {}).get(ref) is None:
                problems.append(f"{ref} flipped but `sides_written` has no "
                                f"entry for it: {row.get('sides_written')}")
            # ...and a write with NO side key must not gain a side row.
            if row2 is not None and row2.get('sides_written'):
                problems.append("a write with no side key recorded a side: "
                                f"{row2.get('sides_written')}")
    except Exception as exc:                                     # noqa: BLE001
        import traceback
        traceback.print_exc()
        print(f"FAIL (BROKEN TEST): {type(exc).__name__}: {exc}")
        return 1
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    if problems:
        print(f"FAIL: {len(problems)} problem(s)")
        for p in problems[:25]:
            print("  " + p)
        return 1
    print(f"PASS: layer_flip flipped exactly its {len(flipped)} members, "
          f"left {len(b) - len(flipped)} alone, and the control is unmoved")
    return 0


if __name__ == '__main__':
    sys.exit(main())
