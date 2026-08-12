#!/usr/bin/env python3
"""Stage a BLIND perturbed subject: damaged board here, truth over there.

    python3 -X utf8 tests/stress/stage_blind.py SRC.kicad_pcb WORKDIR TRUTHDIR

The run may open WORKDIR/board.kicad_pcb and nothing else. TRUTHDIR gets the
control, the perturbation record and the draw, and is not read until the board
is frozen. Pair it with `fence_audit.py --mode create` immediately afterwards.

Disclosure discipline, tightened against the leaks earlier runs found:

  * The writer's own "Modified N footprint positions" line is a dose/block
    tell, so every call runs with stdout captured and discarded.
  * Run 10 disclosed `PERTURBED BLOCK N` because its report's `home /N`
    denominator needed it, and a watcher correctly called that a leak: a
    `swap` draw unions two disjoint blocks and so reports a distinctly larger
    N than the other kinds. This script discloses NOTHING about the block. The
    denominator is read from the record at AUDIT time, when the fence is
    already down; the run itself is scored on the routed outcome, which needs
    no truth at all.
  * There is deliberately NO redraw gate. Run 10 used `home_frac <= 0.20`,
    an asymmetric filter that bites mainly on `scatter` and is therefore
    itself informative about the kind.

THE DOSE IS SCALED TO THE BOARD, and this file used to only claim that.
The previous version called `placement.perturb.max_feasible_dose(pcb)` with one
positional argument against a signature of `(state, members, u, *, grid_step,
dose_cap, ...)`. Binding failed on ARITY, before any board data was touched --
so the `except` was not a guard, it was the ONLY path, and a hard-coded 30.0 mm
was the scale for every board on earth. The docstring's "a band of the board's
own max feasible dose" was not unverifiable; it was always false.

It cannot be fixed by correcting the arity either: `max_feasible_dose` answers
"how far can THIS block travel along THIS direction, up to dose_cap" -- it is
bounded by the cap you pass, so it cannot be used to DERIVE a dose, and it
needs a block and a direction that do not exist until after the draw. There is
no board-maximum mode to call.

So the scale comes from a real board property instead: the outline's smaller
span, which is what actually bounds how far a part can move and still be on
the board. `perturb()` independently clips whatever is asked for
(`on_overflow='clip'`), and the APPLIED dose is recorded in the record as
`max_feasible_dose_mm` / `clipped` -- so the truth dir holds what landed, not
what was requested.

AND THE APPLIED DOSE IS NOW CHECKED, which is the other half of that sentence
and was missing. Recording what landed in the truth dir is no use to a run that
is told "damage applied" either way. Run 14 staged castor_pollux with a
requested dose of >= 4.248 mm and an applied dose of 0.100 mm -- the mutation's
own `grid_step` -- because the block was hard against the outline and
`on_overflow='clip'` ate 98% of it. Nothing in this file read `rec['clipped']`,
`rec['status']` or `rec['dose_mm_applied']`, so it exited 0. The run then spent
an hour proving, correctly, that an undamaged board was undamaged.

A draw is now accepted only if it MEASURABLY landed (see MIN_MATERIAL_MM /
MATERIAL_FRAC below), and otherwise redrawn up to MAX_DRAWS times before the
script refuses outright. The disclosure stays fence-safe: one line saying a
redraw happened, with no count, no kind, no block and no dose.
"""
import contextlib
import io
import json
import os
import random
import shutil
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # #522/py_placer layout

from kicad_parser import parse_kicad_pcb  # noqa: E402
import placement.perturb as P  # noqa: E402

#: Fraction of the board's smaller span a single draw may ask for. The low end
#: has to be material (a 1 mm nudge on an 80 mm board is not a recovery test)
#: and the high end has to leave the board recognisable.
DOSE_BAND = (0.06, 0.22)

#: A draw is only a subject if the damage ACTUALLY LANDED. `perturb()` clips to
#: the outline (`on_overflow='clip'`), so a block hard against an edge can be
#: asked for 4 mm and moved 0.1 mm -- and the old code printed "damage applied"
#: and exited 0 regardless, because it never read `rec['clipped']`,
#: `rec['status']` or `rec['dose_mm_applied']`. Run 14 (castor_pollux) was
#: staged that way: 24 parts displaced by 0.100 mm, which is exactly the
#: mutation's own `grid_step`. Every placement instrument reported the board
#: clean, correctly, and the recovery half of the run was void before it began.
#:
#: So the draw is now CHECKED and, if it did not land, REDRAWN. The bar is
#: measured off `dose_mm_applied` -- the grader's own perturbed-pad RMS, which
#: is what `recovery` later divides by -- and not off the requested dose, which
#: is the number that lied.
MIN_MATERIAL_MM = 1.0      #: absolute floor; 10x the 0.1 mm legalize grid step
MATERIAL_FRAC = 0.35       #: ...and at least this much of what was asked for
MAX_DRAWS = 12             #: redraw budget before refusing outright


def board_scale_mm(pcb):
    """The board's smaller outline span, or None when it declares no outline.

    This is a REAL board property, unlike the constant it replaces. It is the
    dimension that actually bounds a displacement: a part pushed further than
    this along the short axis is off the board whatever the dose says.
    """
    b = getattr(pcb.board_info, 'board_bounds', None)
    if not b:
        return None
    w, h = abs(b[2] - b[0]), abs(b[3] - b[1])
    return min(w, h) if w > 0 and h > 0 else None


def main(src, workdir, truthdir, kinds=None):
    """`kinds` narrows the draw to a subset of KINDS, or None for all of them.

    NARROWING IS A DISCLOSURE, and the run must say so. The kinds displace very
    different fractions of a board -- measured on neo6502, `pile` moves 67% of
    the parts and `translate` 27% -- so a run that wants the placement half
    exercised has a legitimate reason to ask for one. What it costs is exactly
    one bit of the fence: the run is still blind to the block, the dose and the
    seed, but it knows the kind. Report it that way rather than implying a full
    blind draw.
    """
    os.makedirs(workdir, exist_ok=True)
    os.makedirs(truthdir, exist_ok=True)

    pool = tuple(kinds) if kinds else P.KINDS
    bad = [k for k in pool if k not in P.KINDS]
    if bad:
        raise SystemExit('stage_blind: unknown kind(s) %s. Valid: %s'
                         % (', '.join(bad), ' '.join(P.KINDS)))

    rng = random.Random(int.from_bytes(os.urandom(16), 'big'))

    # The AUDIT reference: a plain copy of the source. The perturbation writes
    # its own dose-0 control below, straight into the truth dir -- it must
    # never be derived from `out`, which is what put the human placement inside
    # the work dir in the first place (run-12 Tier 0).
    control = os.path.join(truthdir, 'control.kicad_pcb')
    shutil.copyfile(src, control)
    for ext in ('.kicad_pro', '.kicad_prl'):
        sib = os.path.splitext(src)[0] + ext
        if os.path.exists(sib):
            shutil.copyfile(sib, os.path.join(truthdir, 'control' + ext))

    out = os.path.join(workdir, 'board.kicad_pcb')

    # Computed under capture -- the scale is itself a size tell, and so is
    # every rejected draw.
    buf = io.StringIO()
    attempts = []
    rec = kind = seed = dose = None
    with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(buf):
        pcb = parse_kicad_pcb(src)
        scale = board_scale_mm(pcb)
        if scale is None:
            raise SystemExit(
                'stage_blind: this board declares no usable outline, so a '
                'dose cannot be scaled to it. Refusing rather than falling '
                'back to a constant -- a silent constant is what this script '
                'shipped for three runs.')
        for _ in range(MAX_DRAWS):
            k = rng.choice(pool)
            s = int.from_bytes(os.urandom(4), 'big')
            d = max(3.0, scale * rng.uniform(*DOSE_BAND))
            # control_out FENCES BOTH HALVES OF TRUTH IN ONE CALL. Without it
            # the tool writes `board.control.kicad_pcb` -- the human placement,
            # pose for pose -- and `board.perturb.json` (it embeds
            # `original_poses`) beside the damaged board, INSIDE the work dir
            # the run then reads.
            r = P.perturb(src, out, kind=k, dose_mm=d, seed=s,
                          write_record=True,
                          control_out=os.path.join(
                              truthdir, 'perturbed.control.kicad_pcb'))
            applied = float(r.get('dose_mm_applied') or 0.0)
            need = max(MIN_MATERIAL_MM, MATERIAL_FRAC * d)
            landed = r.get('status') == 'ok' and applied >= need
            attempts.append({'kind': k, 'seed': s, 'dose_mm_requested': d,
                             'dose_mm_applied': applied, 'required': need,
                             'clipped': bool(r.get('clipped')),
                             'status': r.get('status'), 'accepted': landed})
            if landed:
                rec, kind, seed, dose = r, k, s, d
                break

    if rec is None:
        # Same shape as the outline guard: refuse rather than hand back a cell
        # that looks like a result and is not one. The message names no kind,
        # no block and no dose -- only that this BOARD would not take one.
        raise SystemExit(
            'stage_blind: no draw landed a material dose on this board after '
            '%d attempts, so there is no subject to stage. Refusing rather '
            'than emitting a board whose damage is at the grid step -- that '
            'is what voided run 14.' % MAX_DRAWS)

    with open(os.path.join(truthdir, 'draw.json'), 'w', encoding='utf-8') as f:
        json.dump({'kind': kind, 'dose_mm': dose, 'seed': seed,
                   'dose_mm_applied': rec.get('dose_mm_applied'),
                   'clipped': bool(rec.get('clipped')),
                   'board_scale_mm': scale, 'dose_band': list(DOSE_BAND),
                   'material_floor_mm': MIN_MATERIAL_MM,
                   'material_frac': MATERIAL_FRAC,
                   'draws': attempts, 'source': src}, f, indent=1)
    written = os.path.join(truthdir, 'board.perturb.json')
    want = os.path.join(truthdir, 'perturbed.perturb.json')
    if os.path.exists(written):
        shutil.move(written, want)
    for cand in (os.path.splitext(out)[0] + '.perturb.json',):
        if os.path.exists(cand):
            shutil.move(cand, want)
    if not os.path.exists(want):
        with open(want, 'w', encoding='utf-8') as f:
            json.dump(rec, f, indent=1)

    # The ONLY disclosure: board shape, which is visible on the board anyway.
    # NOT the scale, the band, the dose, the kind, the seed or the block.
    p = parse_kicad_pcb(out)
    b = p.board_info.board_bounds
    print(f"parts {len(p.footprints)} | nets "
          f"{len([n for n in p.nets.values() if n.name])} | copper layers "
          f"{len(p.board_info.copper_layers)} "
          f"({', '.join(p.board_info.copper_layers)}) | bbox "
          f"{b[2]-b[0]:.1f} x {b[3]-b[1]:.1f}")
    print(f"segments {len(p.segments)} | vias {len(p.vias)}")
    if len(attempts) > 1:
        # Fence-safe: says a redraw happened and why, and nothing about the
        # kind, the block, the seed or the dose. Deliberately carries no count
        # -- how MANY draws a board rejects is a property of the board, and the
        # run does not need it. The full attempt list is in the truth dir.
        print("stage_blind: redrew (dose infeasible for this board)")
    print("damage applied; kind/dose/seed/block written to the truth dir "
          "and disclosed nowhere")


if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser(
        description=__doc__.strip().splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('src'); ap.add_argument('workdir'); ap.add_argument('truthdir')
    ap.add_argument('--kind', action='append', metavar='KIND',
                    help='narrow the draw to this perturbation kind '
                         '(repeatable). Valid: ' + ' '.join(P.KINDS) + '. The '
                         'kinds displace very different fractions of a board '
                         '(measured: pile 67%%, translate 27%%), so a run that '
                         'wants the PLACEMENT half exercised has a reason to '
                         'ask. It costs one bit of the fence -- still blind to '
                         'block, dose and seed, but not to kind -- so SAY SO '
                         'in the report.')
    _a = ap.parse_args()
    main(_a.src, _a.workdir, _a.truthdir, _a.kind)
