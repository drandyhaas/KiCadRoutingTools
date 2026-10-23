#!/usr/bin/env python3
"""#1201 census: can a fab rung be rejected by size, and may silk containment gate?

Three measurements, all through the production readers:

1. **Fab/pad area ratio.** For every part that draws a .Fab outline AND has
   copper pads: `area(fab bbox) / area(pad bbox)`, both in the footprint's own
   frame and on its own side (`courtyard_for_side`, the rule `body_geometry`
   uses). A pin-1 dot is a fab rung far smaller than the copper it marks; a
   body is not. The guard `body.fab_cannot_be_body` rejects a ratio below
   `FAB_MIN_PAD_AREA_FRAC`, and this is the census that threshold rests on.
2. **Silk containment.** With the guard ON, every containment pair
   (`grade_body_overlap`'s `containment_pairs`) whose only exemption is the
   #896 silk rule -- a silk-drawn member, no `_GATE_EXEMPT` waiver. These are
   the pairs that WOULD gate if a silk body were allowed to gate containment
   (#1201's comment). On a designer's own board each one is a false positive
   of that rule.
3. **What the guard moves.** `courtyard_blocking`, `containment_blocking` and
   the drawn-source mix with the guard OFF and ON, per board.

The guard arms are produced by patching, so the script measures the same thing
before and after #1201 lands: when `body.fab_cannot_be_body` exists it is the
ON arm and is stubbed to False for the OFF arm; before it exists the ON arm is
simulated by withholding the fab rung below the threshold.

    python3 -X utf8 tests/measure_1201_fab_and_silk.py [--no-fa10] [--json OUT] [board ...]

The fa10 human boards are read in place when present (`FA10_HUMAN`); they are
not tracked, so nothing here is asserted against them -- `test_1201_fab_guard`
pinned the tracked numbers until it was deleted with the old placement flows
(ecd673eb). A measurement tool; `run_all` does not collect `measure_*`.
"""
import contextlib
import json
import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
for _p in (ROOT_DIR, os.path.join(ROOT_DIR, 'py_router'),
           os.path.join(ROOT_DIR, 'py_placer'), TESTS_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from kicad_parser import parse_kicad_pcb, non_aperture_pads  # noqa: E402
from run_utils import corpus_boards  # noqa: E402
import placement.body as body  # noqa: E402
from placement.legality import (footprint_side,  # noqa: E402
                                grade_body_overlap)
from placement.parser import courtyard_for_side, extract_fab_sides  # noqa: E402
from placement.utility import compute_footprint_bbox_local  # noqa: E402

#: The designer's own boards from the fa10 campaign (the staged `<board>.kicad_pcb`
#: beside them is the PILE, not the human board).
FA10_TRUTH = os.path.expanduser(
    '~/Documents/prive/git/krt-fa10/wk/fa10/_truth')
FA10_HUMAN = [os.path.join(FA10_TRUTH, b, 'human.kicad_pcb') for b in (
    'CM5_MINIMA_3', 'One-Air-Max', 'StickHub', 'complex_hierarchy',
    'ecc83-pp_v2', 'glasgow_revC', 'interf_u', 'multichannel_mixer',
    'rp2350_fpga_eensy_prePlane', 'sonde_xilinx')]

#: Before the guard exists, the ON arm simulates it at this threshold
#: (body.FAB_MIN_PAD_AREA_FRAC's value; the live constant is used once it
#: exists).
SIM_FRAC = 0.02
#: The threshold the counts below are reported against.
FRAC = getattr(body, 'FAB_MIN_PAD_AREA_FRAC', SIM_FRAC)

#: The waivers that already exempt a containment from gating
#: (legality.grade_body_overlap's `_GATE_EXEMPT`).
GATE_EXEMPT = ('marker_class', 'container_class', 'intent_declared')


def _area(b):
    return max(0.0, b[2] - b[0]) * max(0.0, b[3] - b[1])


def _pads_bbox(fp):
    if not non_aperture_pads(fp):
        return None
    try:
        return compute_footprint_bbox_local(fp)
    except Exception:                                        # noqa: BLE001
        return None


def ratio_rows(pcb, path):
    """[(ref, ratio, fab, pads)] for every part with a fab rung and pads."""
    try:
        fab_all = extract_fab_sides(path)
    except Exception:                                        # noqa: BLE001
        fab_all = {}
    rows = []
    for ref, fp in sorted(pcb.footprints.items()):
        fab = courtyard_for_side(fab_all.get(ref), footprint_side(fp))
        pads = _pads_bbox(fp)
        if fab is None or pads is None or _area(pads) <= 0:
            continue
        rows.append((ref, _area(fab) / _area(pads), fab, pads))
    return rows


@contextlib.contextmanager
def guard(on):
    """Run with the fab guard ON or OFF, whether or not #1201 has landed."""
    live = getattr(body, 'fab_cannot_be_body', None)
    if live is not None:
        saved = live
        if not on:
            body.fab_cannot_be_body = lambda fab, pads: False
        try:
            yield 'live'
        finally:
            body.fab_cannot_be_body = saved
        return
    if not on:
        yield 'absent'
        return
    orig = body.body_geometry

    def _sim(fp, side, courtyard_sides=None, fab_sides=None, silk_sides=None,
             ref='', courtyard_shapes=None, fab_shapes=None):
        fab = courtyard_for_side(fab_sides, side)
        pads = _pads_bbox(fp)
        if fab is not None and pads is not None \
                and _area(fab) < SIM_FRAC * _area(pads):
            fab_sides, fab_shapes = None, None
        return orig(fp, side, courtyard_sides=courtyard_sides,
                    fab_sides=fab_sides, silk_sides=silk_sides, ref=ref,
                    courtyard_shapes=courtyard_shapes, fab_shapes=fab_shapes)
    body.body_geometry = _sim
    try:
        yield 'simulated@%g' % SIM_FRAC
    finally:
        body.body_geometry = orig


def grade(pcb, path):
    import routing_defaults as defaults
    from list_nets import board_floor
    clearance, _src = board_floor(path, 'clearance', None, defaults.CLEARANCE)
    return grade_body_overlap(pcb, clearance, pcb_file=path)


def silk_only_containments(g):
    """Containment pairs whose ONLY exemption is a silk-drawn member."""
    silk = {r for r, s in (g.get('body_sources') or {}).items()
            if s == body.SOURCE_SILK}
    return [p for p in g['containment_pairs']
            if p.waiver not in GATE_EXEMPT and (p.a in silk or p.b in silk)]


def main(argv):
    out_json = None
    if '--json' in argv:
        i = argv.index('--json')
        out_json = argv[i + 1]
        argv = argv[:i] + argv[i + 2:]
    fa10 = '--no-fa10' not in argv
    argv = [a for a in argv if a != '--no-fa10']
    tracked = list(corpus_boards())
    if not tracked:
        print('NOT RUN: git cannot list the tracked boards')
        return 2
    human = [b for b in FA10_HUMAN if fa10 and os.path.isfile(b)]
    boards = ([('tracked', b) for b in tracked] + [('fa10', b) for b in human]
              + [('extra', b) for b in argv])
    out = {'frac': FRAC, 'boards': []}
    tracked_min = None
    dots = []
    silk_trips = []
    for kind, path in boards:
        name = (os.path.basename(os.path.dirname(path)) + '/human'
                if kind == 'fa10' else os.path.basename(path))
        pcb = parse_kicad_pcb(path)
        rows = ratio_rows(pcb, path)
        for ref, r, _f, _p in rows:
            if kind == 'tracked' and (tracked_min is None or r < tracked_min[0]):
                tracked_min = (r, name, ref)
            if r < FRAC:
                dots.append((kind, name, ref, r))
        with guard(False) as how_off:
            g_off = grade(pcb, path)
        with guard(True) as how_on:
            g_on = grade(pcb, path)
        trips = silk_only_containments(g_on)
        silk_trips += [(kind, name, p.a, p.b, p.contained_frac, p.waiver)
                       for p in trips]
        src_off = g_off.get('body_sources') or {}
        src_on = g_on.get('body_sources') or {}
        moved = sorted(r for r in set(src_off) | set(src_on)
                       if src_off.get(r) != src_on.get(r))
        row = {'board': name, 'kind': kind,
               'fab_parts': len(rows),
               'below_frac': sum(1 for _r, r, _f, _p in rows if r < FRAC),
               'guard_off': how_off, 'guard_on': how_on,
               'courtyard_blocking': [g_off['courtyard_blocking'],
                                      g_on['courtyard_blocking']],
               'containment_blocking': [g_off['containment_blocking'],
                                        g_on['containment_blocking']],
               'drawn_source_moved': len(moved),
               'silk_only_containments': [
                   [p.a, p.b, p.contained_frac, p.waiver] for p in trips]}
        out['boards'].append(row)
        print('%-34s %-7s fab=%4d <%.2f=%3d  courtyard_blocking %s  '
              'containment_blocking %s  drawn-source moved %d  '
              'silk-only containments %d' % (
                  name[:34], kind, row['fab_parts'], FRAC,
                  row['below_frac'], row['courtyard_blocking'],
                  row['containment_blocking'], len(moved), len(trips)))
    print()
    if tracked_min:
        print('tracked minimum fab/pad area ratio: %.4f (%s %s)' % tracked_min)
    tr_dots = [d for d in dots if d[0] == 'tracked']
    print('below %.2f: %d tracked, %d fa10' % (
        FRAC, len(tr_dots), len(dots) - len(tr_dots)))
    if dots:
        print('  largest rejected ratio: %.5f (%s %s)' % max(
            ((d[3], d[1], d[2]) for d in dots)))
    print('silk-only containment pairs with the guard ON (would gate if silk '
          'gated containment): %d' % len(silk_trips))
    for t in silk_trips:
        print('  %-7s %-30s %s / %s  frac %.3f  waiver %r' % t)
    out['tracked_min'] = list(tracked_min) if tracked_min else None
    out['below_frac'] = [list(d) for d in dots]
    out['silk_only_containments'] = [list(t) for t in silk_trips]
    if out_json:
        with open(out_json, 'w', encoding='utf-8') as fh:
            json.dump(out, fh, indent=1)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
