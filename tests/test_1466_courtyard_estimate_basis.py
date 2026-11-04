"""#1466: a part that draws no courtyard is measured on ONE basis.

Since #1300 place_simple seats such a part on an ESTIMATED courtyard (its
.Fab body, else silk, else pads, united with its pads). Before this fix the
up-front warning every pre-placement tool printed still said such parts "fall
back to their pad bounding box", and check_floorplan's zone rules and
`--plan-only` budgets really did measure the pad box -- a smaller part than
the placer seats. A user read the warning and drew courtyards of their own,
larger than the estimate, and two zones stopped fitting.

Pinned here, on a tracked fixture whose 18 pad-bearing parts draw no
courtyard:
  1. the disclosure names the estimate per basis, and keeps the pad-box
     WARNING for a part with no estimate only;
  2. a state that does not estimate (a pose tool's) still names the
     estimates placement will use;
  3. the zone rules and the plan check's zone sums read the estimate
     place_simple seats the part on, while the legality budget keeps the
     board's own basis.

    python3 tests/test_1466_courtyard_estimate_basis.py
"""
import contextlib
import io
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from kicad_parser import parse_kicad_pcb                      # noqa: E402
import pose_score                                             # noqa: E402
from placement import floorplan                               # noqa: E402
from placement.body import CourtyardEstimate                  # noqa: E402
from placement.parser import warn_missing_courtyards          # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
FAILS = []


def check(name, ok, detail=''):
    print(f"  {'PASS' if ok else 'FAIL'}  {name}"
          + (f"  ({detail})" if detail and not ok else ''))
    if not ok:
        FAILS.append(name)


def _quiet(fn, *a, **kw):
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        out = fn(*a, **kw)
    return out, buf.getvalue()


def test_the_disclosure():
    est = {'C1': CourtyardEstimate('C1', 'F', (-1, -1, 1, 1), 'fab'),
           'U1': CourtyardEstimate('U1', 'F', (-3, -4, 3, 4), 'silk')}
    _, text = _quiet(warn_missing_courtyards, ['C1', 'U1'], 'placement', est)
    check('an estimated part is named per basis, as an ESTIMATE',
          'ESTIMATED' in text and 'fab: C1' in text and 'silk: U1' in text,
          text)
    check('...and never called a pad bounding box',
          'pad bounding box' not in text, text)
    _, text = _quiet(warn_missing_courtyards, ['C1', 'X9'], 'placement', est)
    check('a part with no estimate keeps the pad-box WARNING, alone',
          'WARNING' in text and 'X9' in text
          and text.count('pad bounding box') == 1
          and 'C1' not in text.split('WARNING')[1], text)


def test_a_state_that_does_not_estimate_names_the_estimates():
    pcb = parse_kicad_pcb(BOARD)
    _, text = _quiet(pose_score.make_state, pcb, BOARD)
    check('a pose tool\'s state names the estimates placement uses',
          'ESTIMATED' in text and 'silk: ' in text and 'fab: ' in text, text)
    check('...with no pad-box warning for a part that has an estimate',
          'pad bounding box' not in text, text)


def test_one_basis():
    pcb = parse_kicad_pcb(BOARD)
    placer, _ = _quiet(pose_score.make_state, pcb, BOARD,
                       estimate_courtyards=True)
    pads, _ = _quiet(pose_score.make_state, pcb, BOARD)
    ref = next(r for r, e in sorted(placer.courtyard_estimates.items())
               if e.basis in ('fab', 'silk')
               and placer.parts[r].rect()[2] > pads.parts[r].rect()[2] + 0.1)
    want = placer.parts[ref].rect()
    it = floorplan.intent_from_dict({
        'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm',
        'blocks': [{'name': 'z', 'refs': [ref],
                    'zone': [want[0] - 5, want[1] - 5,
                             want[2] + 5, want[3] + 5]}]}, '')
    (ctx, _o, state, _b, _p), _ = _quiet(floorplan._grade_ctx, it, pcb,
                                         BOARD)
    got = floorplan._ctx_zone_parts(ctx)[ref].rect
    check(f'the zone rules measure {ref} on the estimate place_simple '
          f'seats it on', got == want, (got, want))
    check('...while the legality budget keeps the board\'s own basis',
          ctx.parts[ref].rect == pads.parts[ref].rect(),
          (ctx.parts[ref].rect, pads.parts[ref].rect()))
    # moved so its estimate leaves the zone while its pad box stays inside:
    # graded on the estimate, the zone rule names it
    shift = (want[2] - pads.parts[ref].rect()[2]) + 4.99
    it2 = floorplan.intent_from_dict({
        'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm',
        'blocks': [{'name': 'z', 'refs': [ref], 'tolerance_mm': 0,
                    'zone': [want[0] - 5, want[1] - 5,
                             want[2] + 5 - shift, want[3] + 5]}]}, '')
    (ctx2, *_r), _ = _quiet(floorplan._grade_ctx, it2, pcb, BOARD)
    hits = [v for v in floorplan.rule_zone_containment(ctx2)
            if v.ref == ref]
    check('a member whose estimate leaves its zone is a zone_containment '
          'finding', len(hits) == 1, [v.message for v in hits])
    (found, measured), _ = _quiet(floorplan.plan_check, it, pcb, BOARD)
    w, h = want[2] - want[0], want[3] - want[1]
    row = measured['zones'][0]
    check('the plan check charges the zone the estimated area',
          abs(row['members_area_mm2'] - round(w * h, 3)) < 1e-3, row)


def main():
    test_the_disclosure()
    test_a_state_that_does_not_estimate_names_the_estimates()
    test_one_basis()
    if FAILS:
        print(f"FAILED: {len(FAILS)} check(s)")
        return 1
    print("all checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
