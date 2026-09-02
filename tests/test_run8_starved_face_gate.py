#!/usr/bin/env python3
"""A face that carries demand and has no lane left (E8) -- as a DELTA, not an absolute.

The third wrong-basin counter-gate, and the one whose first design failed.

The proposal was absolute: flag any escape face with zero supply at the finest
grid while carrying >= N nets, because such a face is a placement the router
cannot rescue. It reads well, and it does discriminate on the board it came
from. Calibrated against the preregistered promotion rule ("promote only if no
healthy board fires"), it fails:

    demand >= 5    6 of 33 healthy in-repo boards fire
    demand >= 7    4 of 33
    demand >= 9    2 of 33
    demand >= 11   2 of 33

and worse, on one run the HUMAN control board fires exactly the same face as
the tool's output. A starved face is often a property of the DESIGN -- a dense
part hard against an edge -- not of the placement decision under test.

#847 -- THAT DENOMINATOR IS NOT RECONSTRUCTABLE, and the re-run at the
pad-copper rect. `git ls-files 'kicad_files/*.kicad_pcb'` gives 22, while a
plain `ls` on a working copy that has run the suite gives 33: the extra 11 are
GITIGNORED generated outputs (`interf_u_*` x4, `fanout_output*`,
`sonde_u_routed_routed`, ...), four of them derivatives of one source board.
That is the hazard `run_utils.corpus_boards()`'s own docstring describes, and
the unreproducible number was cited in production code. Re-run over the tracked
set, at the pad-copper rect, by `tests/measure_847_calibration.py`:

    demand >= 5    2 of 16 boards with a ledger   orangecrab, rp2350
    demand >= 7    2 of 16                        the same two
    demand >= 9    2 of 16                        the same two
    demand >= 11   1 of 16                        rp2350 alone

Both denominators are given because a board that auto-detects no fine-pitch
part can never fire, and counting it as evidence of quiet is how 33 happened.
GATE_MIN_DEMAND stays 7 -- but note that 5, 7 and 9 fire on the SAME two
boards, so nothing in that range discriminates on this corpus and the value is
left alone rather than re-pinned on a measurement that cannot tell them apart.

Taking the delta against the board the placement was derived from removes the
design term, because both boards carry it:

    a wrong-basin placement    1 starved face on its input -> 2, NEW: 2
    three legitimate runs      unchanged, NEW: 0

So the gate is `--baseline` + `--gate`, mirroring check_assembly, and the
absolute count stays a report. The refuted absolute form is recorded here with
its numbers so the finding stays a change detector rather than folklore.

#841: THE WRONG-BASIN ARM NO LONGER FIRES, and that is recorded rather than
repaired, because repairing it here would mean tuning a constant until one
fixture agreed with a conclusion already reached.

`face_lane_ledger` now charges a neighbour its pad COPPER instead of its
COURTYARD. The detection turned entirely on the difference. glasgow's U30 is
an 8.35 x 8.35mm pad field inside an 11.0 x 11.0mm courtyard -- a 1.325mm
skirt on every side. U1's east face looks `max(1.0, 4 * pitch_routed)` deep
for neighbours, which at --clearance 0.09 is the 1.0mm FLOOR. U30's courtyard
reached into that floor; its copper sits 1.775mm out and does not. So:

    band 1.0 (shipped)  U1 E supply 25, eaten by C14/C76      exit 0, 0 NEW
    band 2.0            U1 E supply  1, eaten by U30 (30.3)   exit 4, 2 NEW
    band 3.0            U1 E supply  1, eaten by U30          exit 0, 0 NEW
    band 4.0            U1 E supply  1, eaten by U30          exit 4, 1 NEW

...all at --clearance 0.09, track 0.127, grid 0.05, which is the basis
`_u1_east` below uses. The CLI resolves the board's OWN track width and
reports 28 / 2 / 2 / 2 with U30 at 37.88 lanes; same conclusion, different
instrument, so the table is one basis rather than two halves.

The detection is BAND-GATED, not gone -- and the gate is NON-MONOTONE on the
only fixture there is: it fires at 2.0, does NOT fire at 3.0, and fires again
at 4.0. That is why no value of it was adopted. Filed as #847.

#847: WHAT THAT TABLE ACTUALLY MEASURED, and the loss REPAIRED
---------------------------------------------------------------
The table above is right and its reading was wrong, in a way that matters
because it sent the fix at the wrong parameter. `tests/measure_847_band_gate.py`
decomposes the verdict; three corrections follow from it.

  1. THE TWO COLUMNS ARE TWO DIFFERENT PHENOMENA. "U1 E supply 1, eaten by
     U30" and "exit 4, 2 NEW" are presented above as one story. They are not.
     `_starved_faces` is EMPTY at every band on BOTH boards -- the absolute
     predicate never fires on this fixture at all -- so every exit-4 came from
     `lost_last_lane`, and the faces that produced it are the diodes D22/D23
     (at 2.0) and D21 (at 4.0), carrying demand 3 and 1. U1 never appeared in
     the exit code.

  2. SO RE-PINNING GATE_MIN_DEMAND COULD NEVER HAVE FIXED IT. `lost_last_lane`
     is deliberately unfiltered by --min-demand, and it was the only channel
     firing. #847's own "shape of a fix" would have moved nothing here.

  3. THE NON-MONOTONICITY IS `lost_last_lane`'s `before > 0` CLAUSE. Both
     supplies fall as the band deepens, so each face contributes an INTERVAL:

         D22 N   1.5: 16/16   2.0*: 0/16   2.5*: 0/16   3.0: 0/0
         D21 W   3.0:  3/4    3.5*: 0/4    4.0*: 0/4    5.0: 0/0

     (now/base; * = fired.) Nothing fires at 3.0 because D22/D23's BASELINE
     has itself reached zero -- the damage is masked because the baseline
     became equally bad. That is an artifact, not a property.

The repair is therefore in the PREDICATE, not the band, and the band is left
where it is: `tests/measure_847_calibration.py` measures that deepening it
raises the false-positive rate (at 2.0mm the truth-restore control reaches a
0.435 drop). `check_channels.lost_escape_share` adds the magnitude form --
a face carrying real demand that lost >= 20% of its escape supply -- and with
it the wrong-basin board fails the gate again AT THE SHIPPED BAND, on the face
#847 actually names:

    U1 E   supply 43 -> 28   demand 12   drop 0.349    -> exit 4
    truth restore control    worst drop  0.093         -> exit 0

So the sentence that stood here -- "the gate currently has NO known true
positive on the copper instrument" -- is now false, and it was a real loss
while it stood. It is kept in the history rather than deleted, because the
refuted form is the change detector.

`wk/` is gitignored, so this block still SKIPs on a clean clone. What changed
is that the skip is now LOUD (exit 77 with a `SKIP:` line the runner buckets
separately) instead of returning 0 and landing in `passed` -- which is how an
assertion CI has never executed came to look green.

Run: python3 -X utf8 tests/test_run8_starved_face_gate.py
"""
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'tests'))
os.environ.setdefault('KRT_NO_BANNER', '1')

import check_channels                                          # noqa: E402
import run_utils                                              # noqa: E402
from run_utils import evidence as run_check_evidence          # noqa: E402

SKIP_EXIT = 77
from kicad_parser import parse_kicad_pcb                       # noqa: E402
from placement import routability as _R                        # noqa: E402


def _u1_east(board, band):
    """U1's east face on the wrong-basin board, at one escape band."""
    pcb = parse_kicad_pcb(board)
    kw = dict(clearance=0.09, track_width=0.127, grid_step=0.05,
              pcb_file=board)
    if band is not None:
        kw['escape_band_mm'] = band
    rows = _R.face_lane_ledger(pcb, 'U1', **kw)
    return next(r for r in rows if r['face'] == 'E')


def _check_the_band_is_what_moved(workdir):
    """#841: the detection is band-gated, and that is asserted rather than
    described.

    Without this the docstring above is the only record that the wrong-basin
    board still HAS a starved face at a deeper band -- and a claim that lives
    only in prose is the one that goes stale. `escape_band_mm` is a public
    keyword of `face_lane_ledger` that no production caller passes, which is
    itself part of the finding.
    """
    board = os.path.join(workdir, 'rL_repair.kicad_pcb')
    shipped = _u1_east(board, None)
    deep = _u1_east(board, 2.0)
    check('at the shipped band U1 east is not starved',
          shipped['supply_finest_grid'] >= 20, shipped)
    check('...and U30 is not even charged there',
          'U30' not in {r for r, _v in shipped['eaten_by']}, shipped)
    check('at a 2.0mm band the same face collapses',
          deep['supply_finest_grid'] <= 2, deep)
    check('...and U30 is the blocker that took it',
          deep['eaten_by'] and deep['eaten_by'][0][0] == 'U30', deep)

FAILURES = []


def check(name, cond, detail=''):
    print(f'  {"PASS" if cond else "FAIL"}  {name}'
          + (f'\n        {detail}' if not cond and detail else ''))
    if not cond:
        FAILURES.append(name)


def run(args):
    proc = subprocess.run(
        [sys.executable, '-X', 'utf8',
         os.path.join(ROOT, 'py_tools', 'check_channels.py')] + args,
        capture_output=True, text=True, encoding='utf-8', errors='replace',
        cwd=ROOT)
    return proc.returncode, (proc.stdout or '') + (proc.stderr or '')


def main():
    print('the rule itself')
    led = {'U1': [{'face': 'S', 'demand_nets': 10, 'supply_finest_grid': 0},
                  {'face': 'N', 'demand_nets': 12, 'supply_finest_grid': 4},
                  {'face': 'W', 'demand_nets': 2, 'supply_finest_grid': 0}]}
    starved = check_channels._starved_faces(led, check_channels.GATE_MIN_DEMAND)
    check('a face with demand and no supply is starved',
          ('U1', 'S', 10) in starved, str(starved))
    check('a face with supply is not', not any(f == 'N' for _, f, _ in starved))
    check('a face with almost no demand is not (that is noise, not news)',
          not any(f == 'W' for _, f, _ in starved), str(starved))
    check('the demand floor is a named constant, not a literal',
          isinstance(check_channels.GATE_MIN_DEMAND, int)
          and check_channels.GATE_MIN_DEMAND >= 5)

    print('the gate needs a baseline (the absolute form was refuted)')
    board = os.path.join(ROOT, 'kicad_files', 'tigard.kicad_pcb')
    code, out = run([board, '--gate'])
    check('a --gate run with no baseline cannot fail the board',
          code == 0, out[-300:])

    code, out = run([board, '--baseline', board, '--gate'])
    check('a board against ITSELF is never a new starvation',
          code == 0 and 'NEW' in out, out[-300:])
    check('...and says so with counts', '0 NEW' in out, out[-300:])

    print('recorded runs, when the work dir is present (wk/ is gitignored)')
    wrong = os.path.join(ROOT, 'wk', 'run7', 'glasgow_revC')
    boards = [os.path.join(wrong, n) for n in ('rL_repair.kicad_pcb',
                                               'perturbed.kicad_pcb')]
    missing = [b for b in boards
               if not (os.path.isfile(b) and os.path.getsize(b) > 0)]
    if missing:
        # LOUD, and exit 77. This block used to print two spaces and no colon
        # and return 0, so on a clean clone it landed in `passed` -- an
        # assertion CI has never executed, reading as green. `run_all.py:200`
        # requires the colon form, and 77 is the bucket that is not a pass.
        print(f'SKIP: the recorded run-7 boards are absent, so the only '
              f'wrong-basin fixture the starved-face gate has could not be '
              f'measured: {", ".join(os.path.relpath(m, ROOT) for m in missing)}')
        print('      wk/ is gitignored. The arms above ran; these did not, '
              'and the measured values are in the docstring.')
        return SKIP_EXIT

    for b in boards:
        run_check_evidence(b, 'the wrong-basin fixture')
    code, out = run([boards[0], '--clearance', '0.09',
                     '--baseline', boards[1], '--gate'])
    # #841 took this from 4 to 0 and that loss was recorded here rather than
    # repaired by tuning the band. #847 repaired it in the PREDICATE, so it is
    # 4 again -- and on the face the issue actually names, at the band the
    # tool actually uses.
    check('the wrong-basin placement fails the gate again at the SHIPPED '
          'escape band (#847 repaired the loss #841 recorded)', code == 4,
          f'exit {code} :: {out[-400:]}')
    check('...and it is U1 east that fails it, not a diode',
          'U1 E' in out, out[-500:])
    check('...named by the SHARE form, because supply never reached zero',
          'lost 35% of its escape' in out and '43 -> 28' in out, out[-500:])
    # The correction that made #847 findable: the exit code was NEVER the
    # absolute predicate on this fixture, at any band.
    check('...while the absolute starvation form still finds nothing',
          '0 now, 0 on the baseline' in out, out[-500:])

    # THE DEMAND CONJUNCT, which only this fixture can discriminate. The share
    # form is filtered by --min-demand and `lost_last_lane` deliberately is
    # not, and that difference is load-bearing rather than cosmetic: without
    # it the form fires on demand-1 diodes whose supply merely halved, and on
    # the truth-restore control. The tracked tigard pair CANNOT pin this --
    # measured, no face on it has both demand < 7 and a >= 20% drop -- so the
    # arm lives here, where D21 W (demand 1, supply 8 -> 3, drop 0.625) does.
    # The mutation battery records that removing the conjunct therefore
    # survives on a clean clone.
    # Scoped to the NEW lines. Every ref's per-face ledger is printed too, so
    # a bare substring search finds `D21 W` in the routine listing whatever
    # the gate decided -- an arm that would have passed either way.
    def _new_lines(text):
        return [ln for ln in text.splitlines() if ln.lstrip().startswith('NEW')]

    code, out = run([boards[0], '--clearance', '0.09',
                     '--baseline', boards[1], '--gate', '--min-demand', '1'])
    loose = '\n'.join(_new_lines(out))
    check('at --min-demand 1 the share form reaches the demand-1 diodes',
          'D21 W' in loose, loose or out[-400:])
    code, out = run([boards[0], '--clearance', '0.09',
                     '--baseline', boards[1], '--gate'])
    tight = '\n'.join(_new_lines(out))
    check('...and at the default it does NOT -- the conjunct is what keeps '
          'the form off noise', 'D21 W' not in tight, tight or '(no NEW)')
    check('...while the real finding survives the conjunct', 'U1 E' in tight,
          tight or '(no NEW)')

    control = os.path.join(wrong, 'perturbed.control.kicad_pcb')
    if os.path.isfile(control):
        code, out = run([control, '--clearance', '0.09',
                         '--track-width', '0.0889', '--baseline', boards[1],
                         '--gate'])
        # ONE BASIS. --track-width is passed explicitly because this board
        # declares a different netclass, and grading the two halves of a delta
        # differently manufactured two false positives while #847 was being
        # measured. A control graded at another basis is not a control.
        check('the legitimate restore does NOT fail the gate (the control)',
              code == 0, f'exit {code} :: {out[-400:]}')

    _check_the_band_is_what_moved(wrong)

    print()
    if FAILURES:
        print(f'FAIL: {len(FAILURES)} check(s): {", ".join(FAILURES)}')
        return 1
    print('OK')
    return 0


if __name__ == '__main__':
    sys.exit(main())
