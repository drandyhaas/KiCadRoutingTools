#!/usr/bin/env python3
"""Is this board a usable subject for a perturbed-corpus run? Answer before staging.

    python3 -X utf8 tests/stress/qualify_subject.py BOARD [BOARD ...] [--draws 5]

Run 14 chose castor_pollux because it was the largest set-1 board no run had
used. That is not a qualification. The draw it got was clipped to 0.100 mm, the
board stayed perfectly legal, and an hour of chain time proved that an
undamaged board was undamaged. Nothing in the rig could have told anyone in
advance, because nobody asks the question this script asks.

Three outcomes are possible and only one of them is a subject:

  REJECT  the rig cannot damage this board. Draws clip to nothing, so
          `recovery` and `home /N` will read near-perfect no matter what any
          placement search does or does not do.
  WEAK    the dose lands, but the copper-free gates stay clean. The damage is
          real and invisible to every instrument the placement half owns, so a
          repair loop has nothing to aim at and nothing to verify against.
  GOOD    the dose lands AND the gates fire. Now a placement run can fail, and
          a placement run that fails is the only kind that can succeed.

This is deliberately CHEAP: it perturbs to a temp dir and grades copper-free,
so it costs seconds per draw instead of the hour a full chain costs. It routes
nothing. The routability question ("does the damaged board actually fail to
route, and does the original succeed") is the expensive half and stays a manual
step; see tests/stress/RUNBOOK.md.

It prints AGGREGATES ONLY -- rates and medians over the draws -- and never the
kind, block, seed or direction of any individual draw. So it is safe to run on
a board you intend to stage blind afterwards, which is the whole point.
"""
import argparse
import contextlib
import io
import json
import os
import random
import re
import shutil
import statistics
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # #522/py_placer layout

from kicad_parser import parse_kicad_pcb  # noqa: E402
import placement.perturb as P  # noqa: E402
from stage_blind import (board_scale_mm, DOSE_BAND, MIN_MATERIAL_MM,  # noqa: E402
                         MATERIAL_FRAC)

#: Verdict bands. A board must land a material dose ALMOST every time to be a
#: subject, not merely more often than not: a run stakes an hour of chain time
#: on a single draw, so a 50% clip rate is a coin flip on whether the run has a
#: subject at all. That is what happened on run 14.
LAND_REJECT = 0.5      #: below this, the rig cannot damage the board
LAND_GOOD = 0.8        #: at or above this, the draw is dependable
FIRE_GOOD = 0.5        #: fraction of landed draws that must trip a gate


def _board_clearance(board):
    """The board's own Default netclass clearance, else the routing default."""
    try:
        from list_nets import board_default_netclass_clearance
        v = board_default_netclass_clearance(board)
        if v:
            return float(v)
    except Exception:                                   # noqa: BLE001
        pass
    return 0.25


def _gates(board, clearance):
    """(drc_violations, assembly_blocking) on the COPPER-FREE board."""
    drc = subprocess.run(
        [sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_router', 'check_drc.py'), board,
         '--clearance', str(clearance), '--check-pad-edge'],
        capture_output=True, text=True)
    # check_drc emits exactly two summary shapes -- `FOUND {n} DRC VIOLATIONS:`
    # and `NO DRC VIOLATIONS FOUND!`. It has never printed "Total violations",
    # so the old scan for that string was dead code that ALWAYS fell through to
    # a fallback counting every line containing "VIOLATION" -- the FOUND banner
    # plus one per type header, i.e. `1 + n_types`, never the real count (a
    # 40-violation board read as 3). It only ever fed a `> 0` boolean, so the
    # verdicts were right by luck rather than by measurement.
    nv = 0
    if 'NO DRC VIOLATIONS' not in drc.stdout:
        m = re.search(r'^FOUND (\d+) DRC VIOLATIONS', drc.stdout, re.M)
        if m:
            nv = int(m.group(1))
        else:
            # No recognisable summary: do not invent a number. -1 is "unknown",
            # which is still truthy for the `> 0` gate but cannot be mistaken
            # for a measured count if this is ever read quantitatively.
            nv = -1
    asm = subprocess.run(
        [sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_tools', 'check_assembly.py'),
         board, '--clearance', str(clearance)],
        capture_output=True, text=True)
    blocking = 0
    for line in asm.stdout.splitlines():
        s = line.strip()
        if s.startswith('blocking '):
            blocking = int(s.split()[1])
            break
    return nv, blocking


def qualify(board, draws=5, seed=None):
    pcb = parse_kicad_pcb(board)
    scale = board_scale_mm(pcb)
    if scale is None:
        return {'board': board, 'verdict': 'REJECT',
                'reason': 'no usable outline, so a dose cannot be scaled to it'}
    if pcb.segments or pcb.vias:
        return {'board': board, 'verdict': 'REJECT',
                'reason': 'board carries copper; strip it first (the placement '
                          'gate refuses a routed board, and the copper encodes '
                          'the original poses)'}

    rng = random.Random(seed if seed is not None
                        else int.from_bytes(os.urandom(8), 'big'))
    clearance = _board_clearance(board)
    tmp = tempfile.mkdtemp(prefix='qualify_')
    landed, fired, applied, blocked = 0, 0, [], []
    try:
        for _ in range(draws):
            kind = rng.choice(P.KINDS)
            dose = max(3.0, scale * rng.uniform(*DOSE_BAND))
            out = os.path.join(tmp, 'p.kicad_pcb')
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(buf):
                rec = P.perturb(board, out, kind=kind, dose_mm=dose,
                                seed=int.from_bytes(os.urandom(4), 'big'),
                                write_record=False,
                                control_out=os.path.join(tmp, 'c.kicad_pcb'))
            a = float(rec.get('dose_mm_applied') or 0.0)
            applied.append(a)
            if rec.get('status') == 'ok' and \
                    a >= max(MIN_MATERIAL_MM, MATERIAL_FRAC * dose):
                landed += 1
                nv, blk = _gates(out, clearance)
                blocked.append((nv, blk))
                if nv > 0 or blk > 0:
                    fired += 1
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    land_rate = landed / float(draws)
    fire_rate = (fired / float(landed)) if landed else 0.0
    # Bands chosen against run 14: castor_pollux lands about half its draws,
    # which is not REJECT (the board CAN be damaged; at 2.0 mm it goes NOT
    # BUILDABLE) but is emphatically not GOOD either, because a coin flip
    # decides whether the run has a subject at all. That case has to have a
    # name, and WEAK is it.
    if land_rate < LAND_REJECT:
        verdict, reason = 'REJECT', (
            'only %d of %d draws landed a material dose; recovery here would '
            'measure the clip, not the run' % (landed, draws))
    elif land_rate < LAND_GOOD:
        verdict, reason = 'WEAK', (
            'only %d of %d draws landed a material dose; usable, but expect '
            'to redraw and never assume the dose landed' % (landed, draws))
    elif fire_rate < FIRE_GOOD:
        verdict, reason = 'WEAK', (
            'doses land (%d/%d) but only %d made a copper-free gate fire; the '
            'damage is largely invisible to the placement half'
            % (landed, draws, fired))
    else:
        verdict, reason = 'GOOD', (
            '%d of %d draws landed, and %d of those made a gate fire'
            % (landed, draws, fired))
    return {'board': board, 'verdict': verdict, 'reason': reason,
            'draws': draws, 'landed': landed, 'gates_fired': fired,
            'land_rate': round(land_rate, 3), 'fire_rate': round(fire_rate, 3),
            'applied_mm_median': round(statistics.median(applied), 3) if applied else None,
            'applied_mm_min': round(min(applied), 3) if applied else None,
            'applied_mm_max': round(max(applied), 3) if applied else None,
            'graded_at_clearance': clearance, 'board_scale_mm': round(scale, 1)}


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('boards', nargs='+')
    ap.add_argument('--draws', type=int, default=5,
                    help='draws per board (default 5). More is better and '
                         'costs seconds; the variance between draws is the '
                         'thing run 14 got caught by.')
    ap.add_argument('--seed', type=int, default=None,
                    help='make the QUALIFICATION reproducible. Does not seed '
                         'the run you stage afterwards.')
    ap.add_argument('--json', dest='json_out', default=None)
    a = ap.parse_args()

    rows = []
    for b in a.boards:
        r = qualify(b, draws=a.draws, seed=a.seed)
        rows.append(r)
        print('%-9s %-34s %s' % (r['verdict'], os.path.basename(b), r['reason']))
        if r.get('applied_mm_median') is not None:
            print('          applied dose mm: median %.3f, range %.3f to %.3f'
                  ' | graded at clearance %.3f'
                  % (r['applied_mm_median'], r['applied_mm_min'],
                     r['applied_mm_max'], r['graded_at_clearance']))
    if a.json_out:
        # run 19 lost a qualification JSON to a not-yet-created directory
        # AFTER all the draws had been paid for. Make the parent, then write.
        os.makedirs(os.path.dirname(a.json_out) or '.', exist_ok=True)
        with open(a.json_out, 'w', encoding='utf-8') as f:
            json.dump(rows, f, indent=1)
        print('  JSON -> %s' % a.json_out)
    good = [r for r in rows if r['verdict'] == 'GOOD']
    print('\n%d of %d qualify as subjects' % (len(good), len(rows)))
    return 0 if good else 1


if __name__ == '__main__':
    import cli_banner
    cli_banner.install()
    sys.exit(main())
