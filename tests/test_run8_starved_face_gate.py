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
at 4.0. That is why no value of it is adopted here. The band
floor and the neighbour rectangle are one model (a courtyard IS a body plus a
skirt, so shrinking to copper while holding the band silently narrows the
search by that skirt), and re-deriving it needs the 33-healthy-board
preregistered calibration above re-run at the copper rect, not a fixture fit.
Filed as its own issue.

Two things follow that a reader must not have to infer:

  * The gate currently has NO known true positive on the copper instrument.
    That is a real loss, not a bookkeeping item.
  * `wk/` is gitignored, so a clean clone SKIPs this block and CI has never
    executed the assertion. A green suite is not evidence about it either way.

The arm below therefore asserts what is MEASURED at the shipped band, and a
second arm pins the band dependence through `face_lane_ledger`'s own
`escape_band_mm` keyword, so the mechanism stays executable instead of
becoming prose in this docstring.

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
os.environ.setdefault('KRT_NO_BANNER', '1')

import check_channels                                          # noqa: E402
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
    if os.path.isdir(wrong):
        code, out = run([os.path.join(wrong, 'rL_repair.kicad_pcb'),
                         '--clearance', '0.09', '--baseline',
                         os.path.join(wrong, 'perturbed.kicad_pcb'), '--gate'])
        # #841: 4 -> 0. Recorded, with the mechanism pinned below, NOT
        # repaired by moving the band until this fixture agrees again.
        check('the wrong-basin placement no longer fails the gate at the '
              'shipped escape band (#841, a LOSS -- see the docstring)',
              code == 0, out[-400:])
        check('...and it names no new starved face', 'NEW:' not in out,
              out[-400:])
        _check_the_band_is_what_moved(wrong)
    else:
        print('  SKIP  recorded boards not present; the measured values are '
              'in the docstring')

    print()
    if FAILURES:
        print(f'FAIL: {len(FAILURES)} check(s): {", ".join(FAILURES)}')
        return 1
    print('OK')
    return 0


if __name__ == '__main__':
    sys.exit(main())
