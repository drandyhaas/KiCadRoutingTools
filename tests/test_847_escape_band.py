#!/usr/bin/env python3
"""The escape band: one resolver, and the band a row reports is the band it used.

#847. Both lane ledgers look a fixed depth off a face for neighbours, and both
open-coded the same arithmetic on a DIFFERENT pitch:

    escape.part_escape           max(lane * 4.0, 1.0)    lane = track + clearance
    routability.face_lane_ledger max(1.0, 4 * pitch)     pitch = GRID-QUANTIZED

Same shape, two bases, and nothing in either file said so. Measured over the 22
tracked boards they disagree on 19 of them -- 2.2mm against 2.4mm wherever the
board falls back to `routing_defaults` 0.3/0.25 -- and agree only on
`glasgow_revC`, `flat_hierarchy` and `routed_output`.

This file pins the resolver that replaced both, and three facts about it that a
reader should not have to take on trust:

  * WHICH TERM DECIDES. `4 * lane` is the model; the 1.0mm floor is the term
    #847 is about, and it was chosen while a neighbour contributed its
    COURTYARD rather than the pad copper both ledgers charge since #841.
    Measured here: on the tracked corpus that floor decides on **exactly one
    board**, `routed_output` -- 16 of 388 routability face-rows and 3 of 97
    escape parts. Everywhere else `4 * lane` is already larger. So a fix that
    re-derives only the FLOOR would be very nearly a no-op, and that is worth
    knowing before anyone spends a calibration on it.

  * THE TIE IS NOT THE FLOOR. At the basis
    `tests/test_run8_starved_face_gate.py` uses (track 0.127, clearance 0.09,
    grid 0.05) the quantized pitch is 0.25 and `4 * pitch` is EXACTLY 1.0, so
    the floor ties and decides nothing. At the basis the shipped CLI resolves
    for that same board (track 0.0889, grid 0.1) the pitch is 0.20, `4 * pitch`
    is 0.80, and the floor DOES bind. Same band, two different reasons; the
    test file's docstring blends the two bases and this arm keeps them apart.

  * THE REPORTED BAND IS THE BAND THAT WAS USED. A field naming the band would
    be worse than no field if it could drift from the depth the obstruction
    kernel actually searched. The arms below capture the band interval
    `span_eaten` is handed -- the UPSTREAM producer -- and compare its depth
    against what the row reports. Asserting the row against the resolver that
    filled the row would be true by construction and would see nothing; that
    exact mistake passed for six commits on this branch's parent (#841).

Run: python3 -X utf8 tests/test_847_escape_band.py
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
for _p in ('py_router', 'py_placer', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _p))
os.environ.setdefault('KRT_NO_BANNER', '1')

from kicad_parser import parse_kicad_pcb                       # noqa: E402
from placement import escape as E                              # noqa: E402
from placement import routability as R                         # noqa: E402

RUN_ALL_FAST_OK = True
RUN_ALL_TIMEOUT = 300

BOARDS = os.path.join(ROOT, 'kicad_files')
TIGARD = os.path.join(BOARDS, 'tigard.kicad_pcb')
GLASGOW = os.path.join(BOARDS, 'glasgow_revC.kicad_pcb')
ROUTED = os.path.join(BOARDS, 'routed_output.kicad_pcb')

FAILURES = []


def check(name, cond, detail=''):
    print(f'  {"PASS" if cond else "FAIL"}  {name}'
          + (f'\n        {detail}' if not cond and detail else ''))
    if not cond:
        FAILURES.append(name)


# --------------------------------------------------------------- the resolver

def the_arithmetic():
    print('the resolver, and the term that decided')
    b = E.escape_band(0.55, basis='raw_lane')
    check('4 x lane wins when it exceeds the floor',
          abs(b.mm - 2.2) < 1e-9 and b.source == 'lanes', b)
    b = E.escape_band(0.20, basis='quantized_lane')
    check('the floor wins when 4 x lane is under it',
          abs(b.mm - 1.0) < 1e-9 and b.source == 'floor', b)

    # The distinction #847's own framing turns on. `4 * 0.25 == 1.0` exactly,
    # so removing the floor would change NOTHING at this basis -- reporting it
    # as 'floor' would say the opposite of what is true.
    b = E.escape_band(0.25, basis='quantized_lane')
    check('an exact tie is reported as lanes, not as the floor',
          abs(b.mm - 1.0) < 1e-9 and b.source == 'lanes', b)

    b = E.escape_band(0.55, basis='raw_lane', override=2.0)
    check('an explicit band overrides both terms and says so',
          abs(b.mm - 2.0) < 1e-9 and b.source == 'caller', b)
    check('...and the override is honoured even below the floor',
          abs(E.escape_band(0.55, basis='raw_lane', override=0.3).mm
              - 0.3) < 1e-9)

    check('the two terms are named constants, not literals',
          E.ESCAPE_BAND_LANES == 4.0 and E.ESCAPE_BAND_FLOOR_MM == 1.0,
          (E.ESCAPE_BAND_LANES, E.ESCAPE_BAND_FLOOR_MM))
    check('the basis is carried through, not invented',
          E.escape_band(0.4, basis='quantized_lane').basis == 'quantized_lane')


# ------------------------------------------- the reported band is the real one

def _captured_band_depth(fn):
    """Run `fn` with `span_eaten` wrapped; return the set of band DEPTHS seen.

    The band arrives as an interval `(near, far)` in board coordinates, so its
    depth is the interval's width. This reads the value the obstruction kernel
    was actually handed -- not the field the row carries -- which is the whole
    point of the arm.
    """
    seen = set()
    orig = E.span_eaten

    def probe(lo, hi, band, horiz, obstacles):
        seen.add(round(abs(band[1] - band[0]), 6))
        return orig(lo, hi, band, horiz, obstacles)

    E.span_eaten = probe
    try:
        fn()
    finally:
        E.span_eaten = orig
    return seen


def the_band_reported_is_the_band_searched():
    print('the band a row reports is the depth the kernel searched')
    pcb = parse_kicad_pcb(TIGARD)

    rows = {}

    def run_routability():
        rows['r'] = R.face_lane_ledger(pcb, 'U3', clearance=0.09,
                                       track_width=0.127, grid_step=0.05,
                                       pcb_file=TIGARD)

    depths = _captured_band_depth(run_routability)
    reported = {r['escape_band_mm'] for r in rows['r']}
    check('face_lane_ledger searched exactly one depth', len(depths) == 1,
          depths)
    check('...and every row reports that same depth',
          len(reported) == 1 and abs(next(iter(reported))
                                     - next(iter(depths))) < 1e-6,
          (reported, depths))
    check('...reported with the term that set it',
          {r['escape_band_source'] for r in rows['r']} == {'lanes'}
          and {r['escape_band_basis'] for r in rows['r']} == {'quantized_lane'},
          rows['r'][0])

    # An OVERRIDE must reach the kernel too, or the keyword the fixture's band
    # ladder rides on is decoration. This is the one production-reachable knob
    # #847 names, so it gets its own arm rather than sharing the one above.
    def run_override():
        rows['o'] = R.face_lane_ledger(pcb, 'U3', clearance=0.09,
                                       track_width=0.127, grid_step=0.05,
                                       escape_band_mm=2.0, pcb_file=TIGARD)

    depths = _captured_band_depth(run_override)
    check('an explicit escape_band_mm reaches the obstruction kernel',
          depths == {2.0}, depths)
    check('...and the rows say a caller set it',
          {r['escape_band_source'] for r in rows['o']} == {'caller'}
          and {r['escape_band_mm'] for r in rows['o']} == {2.0},
          rows['o'][0])

    def run_escape():
        rows['e'] = E.part_escape(pcb, 'U3', pcb_file=TIGARD)

    depths = _captured_band_depth(run_escape)
    band_mm = {f.escape_band_mm for f in rows['e'].faces}
    check('part_escape searched exactly one depth', len(depths) == 1, depths)
    check('...and its FaceLedgers report that same depth',
          len(band_mm) == 1 and abs(next(iter(band_mm))
                                    - next(iter(depths))) < 1e-6,
          (band_mm, depths))
    check('...on the raw-lane basis',
          {f.escape_band_basis for f in rows['e'].faces} == {'raw_lane'},
          rows['e'].faces[0])

    # A SECOND BASIS, because the first one is degenerate and an independent
    # verifier caught it: at clearance 0.09 / track 0.127 / grid 0.05 the
    # quantized pitch is 0.25, so `4 * pitch` is EXACTLY 1.0 -- and a row that
    # hard-coded 1.0 while searching something else would satisfy every
    # assertion above. Measured: it did. This basis resolves to 2.4, where a
    # hard-coded constant and the real band cannot coincide.
    def run_nondegenerate():
        rows['nd'] = R.face_lane_ledger(pcb, 'U3', clearance=0.25,
                                        track_width=0.3, grid_step=0.1,
                                        pcb_file=TIGARD)

    depths = _captured_band_depth(run_nondegenerate)
    reported = {r['escape_band_mm'] for r in rows['nd']}
    check('at a NON-degenerate basis the searched depth is not 1.0',
          depths and abs(next(iter(depths)) - 1.0) > 0.5, depths)
    check('...and the row still reports the depth that was searched',
          len(reported) == 1 and len(depths) == 1
          and abs(next(iter(reported)) - next(iter(depths))) < 1e-6,
          (reported, depths))

    def run_escape_reach():
        rows['er'] = E.part_escape(pcb, 'U3', reach_mm=3.0, pcb_file=TIGARD)

    depths = _captured_band_depth(run_escape_reach)
    check('part_escape honours reach_mm and reports it as caller',
          depths == {3.0}
          and {f.escape_band_source for f in rows['er'].faces} == {'caller'},
          depths)


# ------------------------------------------------- the two bases still differ

def the_two_ledgers_still_differ():
    print('the two ledgers share the arithmetic and NOT the basis')
    # tigard falls back to routing_defaults 0.3 / 0.25, so the raw lane is 0.55
    # (escape -> 2.2) and the pitch quantized to a 0.1 grid is 0.6
    # (routability -> 2.4). This is a DISAGREEMENT, pinned as one: #847 unified
    # the arithmetic, not the basis, and a later commit that unifies the basis
    # should have to come here and say so.
    raw = E.escape_band(0.55, basis='raw_lane')
    quant = E.escape_band(0.60, basis='quantized_lane')
    check('the same face resolves to two different depths today',
          abs(raw.mm - 2.2) < 1e-9 and abs(quant.mm - 2.4) < 1e-9
          and raw.mm != quant.mm, (raw, quant))
    check('...and each names which pitch it came from',
          raw.basis != quant.basis, (raw.basis, quant.basis))


# --------------------------------------------------- where the floor DECIDES

def where_the_floor_decides():
    print('the 1.0mm floor decides on one tracked board, not on the corpus')
    # Named boards rather than a corpus sweep, so this stays a fast unit test;
    # the corpus census is `tests/measure_847_calibration.py`, which is
    # where the 16-of-388 / 3-of-97 figures in the docstring are re-derived.
    pcb = parse_kicad_pcb(ROUTED)
    ref = next(r for r in sorted(pcb.footprints) if pcb.footprints[r].pads)
    rows = R.face_lane_ledger(pcb, ref, clearance=0.09, track_width=0.09,
                              grid_step=0.1, pcb_file=ROUTED)
    check('routed_output is the board where the floor binds',
          rows and {r['escape_band_source'] for r in rows} == {'floor'},
          rows[0] if rows else None)

    pcb = parse_kicad_pcb(GLASGOW)
    rows = R.face_lane_ledger(pcb, 'U1', clearance=0.2, track_width=0.2,
                              grid_step=0.1, pcb_file=GLASGOW)
    check('glasgow at its own netclass is set by 4 x lane, not the floor',
          rows and {r['escape_band_source'] for r in rows} == {'lanes'}
          and {r['escape_band_mm'] for r in rows} == {1.6},
          rows[0] if rows else None)


# ------------------------------------------------------- the CLI can reach it

def the_cli_can_reach_the_band():
    """#847's first complaint: the band was reachable from no shipped entry.

    `run_utils.check` rather than a bare returncode, per CLAUDE.md: an
    ImportError and a satisfied guard both exit non-zero, and only one of them
    is evidence. Imported under a different name because this file already
    defines a `check` of its own with a different signature.
    """
    print('the band is reachable from the shipped CLI')
    import json
    import tempfile
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import run_utils
    from run_utils import check as run_check

    tool = run_utils.tool('check_channels.py')
    with tempfile.TemporaryDirectory() as tmp:
        out = {}
        for label, extra in (('default', []), ('flagged', ['--escape-band',
                                                           '2.0'])):
            path = os.path.join(tmp, f'{label}.json')
            r = run_check([sys.executable, '-X', 'utf8', tool, TIGARD,
                           '--refs', 'U3', '--json', path] + extra,
                          accept=True)
            out[label] = json.load(open(run_utils.evidence(path),
                                       encoding='utf-8'))
            check(f'the header names the band and its source ({label})',
                  'escape-band' in r.stdout,
                  r.stdout.splitlines()[0] if r.stdout else '')

        a, b = out['default'], out['flagged']
        check('the JSON records the band, its source and its basis',
              a['escape_band']['source'] == 'lanes'
              and a['escape_band']['basis'] == 'quantized_lane',
              a['escape_band'])
        check('--escape-band actually changes the band',
              b['escape_band']['value'] == 2.0
              and b['escape_band']['source'] == 'caller',
              b['escape_band'])
        # A flag that is reported but does not reach the ledger would satisfy
        # every assertion above and change nothing. Measured on tigard U3: the
        # south and east faces go 2 -> 17 and 6 -> 17 lanes at a 2.0mm band.
        sa = {r['face']: r['supply_finest_grid'] for r in a['ledgers']['U3']}
        sb = {r['face']: r['supply_finest_grid'] for r in b['ledgers']['U3']}
        check('...and the supplies it produces move with it', sa != sb,
              (sa, sb))
        check('the JSON records the demand threshold it graded at',
              a.get('min_demand') == 7, a.get('min_demand'))


# --------------------------------------- a true positive a clean clone can run

def the_gate_has_a_tracked_true_positive():
    """#847's headline loss, closed on TRACKED boards.

    The issue's complaint is that `check_channels --gate` has no known true
    positive on the copper instrument, and that the one fixture it had lives
    in gitignored `wk/` -- so a clean clone skips it and CI has never executed
    the assertion either way.

    `tests/fixtures/run23/tigard_{damaged,placed}.kicad_pcb` are TRACKED, and
    under the share predicate the damaged board fails the gate at the SHIPPED
    band while the undamaged one against itself does not. That is the whole
    property, on boards anyone gets from a checkout.

    Worth knowing why this was not available before: under the shipped
    predicate this pair reports `0 now, 0 on the baseline, 0 NEW` and exit 0.
    Nothing about it is new geometry -- U3's east face was already falling
    27 -> 13 lanes against a demand of 9. No predicate could see it.
    """
    print('the gate has a true positive a clean clone can run')
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import run_utils
    from run_utils import check as run_check

    tool = run_utils.tool('check_channels.py')
    fix = os.path.join(ROOT, 'tests', 'fixtures', 'run23')
    dmg = run_utils.evidence(os.path.join(fix, 'tigard_damaged.kicad_pcb'),
                             'the damaged fixture')
    ok = run_utils.evidence(os.path.join(fix, 'tigard_placed.kicad_pcb'),
                            'the undamaged fixture')

    # code=4 AND the reason. A bare non-zero cannot tell "the gate fired" from
    # "the CLI died before parsing its arguments", and this repo has shipped
    # exactly that mistake.
    r = run_check([sys.executable, '-X', 'utf8', tool, dmg,
                   '--baseline', ok, '--gate'], code=4, refuse='NEW')
    check('a tracked damaged board fails the gate at the shipped band',
          True)
    check('...on U3 east, the face that lost the escape',
          'U3 E' in r.stdout and '27 -> 13' in r.stdout, r.stdout[-400:])
    check('...via the SHARE form, since supply never reached zero',
          'of its escape' in r.stdout
          and '0 now, 0 on the baseline' in r.stdout, r.stdout[-400:])

    run_check([sys.executable, '-X', 'utf8', tool, ok,
               '--baseline', ok, '--gate'], accept=True)
    check('...while the undamaged board against itself does not', True)

    # And the negative control that matters most: turning the predicate off
    # must restore the old verdict, which is what says the SHARE form is what
    # catches this rather than something else that changed along the way.
    run_check([sys.executable, '-X', 'utf8', tool, dmg, '--baseline', ok,
               '--gate', '--min-supply-drop', '0'], accept=True)
    check('...and with --min-supply-drop 0 the gate goes quiet again, which '
          'is what makes the share form the cause', True)


def main():
    the_arithmetic()
    print()
    the_band_reported_is_the_band_searched()
    print()
    the_two_ledgers_still_differ()
    print()
    where_the_floor_decides()
    print()
    the_cli_can_reach_the_band()
    print()
    the_gate_has_a_tracked_true_positive()
    print()
    if FAILURES:
        print(f'FAIL: {len(FAILURES)} check(s): {", ".join(FAILURES)}')
        return 1
    print('OK')
    return 0


if __name__ == '__main__':
    sys.exit(main())
