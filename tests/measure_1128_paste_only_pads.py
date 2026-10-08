#!/usr/bin/env python3
"""#1128: what reading a paste-only pad as copper moves, per board.

`legality.occupancy_shape` (the courtyard census's per-part occupancy) and
`legality.pad_copper_overrun_mm` (THE gating measure for pad copper past the
outline) skip NPTH pads but read `pad.layers` nowhere, so a pad on no copper
layer -- a paste-only aperture -- is measured as if it were copper.

Two arms on ONE tree, so the difference is that population and nothing else:

* `as-is`   -- the board as parsed;
* `copper`  -- a deep copy with every non-NPTH pad that carries no copper
  (`not legality._pad_carries_copper(pad)`) removed from its footprint.

Removing them changes nothing the measures do not read: `PartPads` and
`_pad_with_copper` already skip those pads, and the paste apertures are built
at parse time. Before the #1128 fix the arm difference is the fix's impact;
after it the difference must be 0 on every board. Measured at the fix's
parent it was ALREADY 0 on all 16 boards that carry such pads (their
apertures sit inside copper or courtyard), so this script shows the fix moves
no real board; that the fix landed is pinned by
tests/test_1128_paste_only_not_copper.py's synthetic cases, not here.

#1143 extended the same rule to the other placement measures its sweep found,
through `kicad_parser.pad_is_aperture_only` (which keeps NPTH and drilled
pads); its own per-site measurement, with the readers left on purpose, is
tests/measure_1143_paste_only_sites.py.

Per board: the paste-only pad population, the courtyard census (every
courtyard pair from `grade_body_overlap`: count and area), the census's
blocking count, and `grade_pad_legality`'s per-part
`oob_pad_copper_overrun_mm`.

Not collected by run_all (no `test_` prefix).

    python3 -X utf8 tests/measure_1128_paste_only_pads.py [--demos DIR]
        [--board PATH ...]
"""
import argparse
import contextlib
import copy
import io
import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for _d in ('py_router', 'py_placer', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _d))
sys.path.insert(0, TESTS_DIR)

CLEARANCE = 0.2


def _paste_only(fp):
    from placement.legality import _pad_carries_copper
    return [p for p in (fp.pads or ())
            if getattr(p, 'pad_type', '') != 'np_thru_hole'
            and not _pad_carries_copper(p)]


def _strip(pcb):
    out = copy.deepcopy(pcb)
    for fp in out.footprints.values():
        drop = {id(p) for p in _paste_only(fp)}
        if drop:
            fp.pads = [p for p in fp.pads if id(p) not in drop]
    return out


def _measure(pcb, path):
    from placement import legality
    with contextlib.redirect_stdout(io.StringIO()):
        g = legality.grade_body_overlap(pcb, CLEARANCE, pcb_file=path)
        pl = legality.grade_pad_legality(pcb, CLEARANCE, pcb_file=path)
    court = [q for q in g['pairs'] if q.kind == 'courtyard']
    over = {r: v for r, v in (pl.get('oob_pad_copper_overrun_mm') or {}).items()
            if v}
    return {'courtyard_pairs': len(court),
            'courtyard_mm2': round(sum(q.area_mm2 for q in court), 3),
            'blocking': int(g.get('blocking') or 0),
            'overrun': over}


def measure(path):
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(path)
    pop = {r: len(_paste_only(fp)) for r, fp in pcb.footprints.items()}
    pop = {r: n for r, n in pop.items() if n}
    if not pop:
        return {'paste_only_pads': 0, 'parts': 0}
    a = _measure(pcb, path)
    b = _measure(_strip(pcb), path)
    moved_over = {r: (a['overrun'].get(r, 0.0), b['overrun'].get(r, 0.0))
                  for r in set(a['overrun']) | set(b['overrun'])
                  if abs(a['overrun'].get(r, 0.0)
                         - b['overrun'].get(r, 0.0)) > 1e-6}
    return {'paste_only_pads': sum(pop.values()), 'parts': len(pop),
            'as_is': a, 'copper': b, 'overrun_moved': moved_over,
            'moved': (a['courtyard_pairs'] != b['courtyard_pairs']
                      or abs(a['courtyard_mm2'] - b['courtyard_mm2']) > 1e-6
                      or a['blocking'] != b['blocking'] or bool(moved_over))}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--demos', default=None,
                    help='Also measure every .kicad_pcb under this directory '
                         '(e.g. the KiCad install\'s share/kicad/demos)')
    ap.add_argument('--board', action='append', default=None)
    args = ap.parse_args(argv)
    if args.board:
        boards = list(args.board)
    else:
        from run_utils import corpus_boards
        boards = list(corpus_boards())
    if args.demos:
        for dp, _dn, fns in os.walk(args.demos):
            boards += [os.path.join(dp, f) for f in sorted(fns)
                       if f.endswith('.kicad_pcb')]
    n_moved = 0
    for b in boards:
        try:
            r = measure(b)
        except Exception as exc:                          # noqa: BLE001
            print(f"{os.path.basename(b)}: UNMEASURABLE {type(exc).__name__}: "
                  f"{str(exc)[:120]}", flush=True)
            continue
        name = os.path.basename(b)
        if not r['paste_only_pads']:
            print(f"{name}: no paste-only pad", flush=True)
            continue
        a, c = r['as_is'], r['copper']
        n_moved += r['moved']
        print(f"{name}: {r['paste_only_pads']} paste-only pad(s) on "
              f"{r['parts']} part(s) -- census {a['courtyard_pairs']} -> "
              f"{c['courtyard_pairs']} pairs, {a['courtyard_mm2']} -> "
              f"{c['courtyard_mm2']} mm2, blocking {a['blocking']} -> "
              f"{c['blocking']}; overrun moved on "
              f"{len(r['overrun_moved'])} part(s)"
              + (f" {dict(sorted(r['overrun_moved'].items())[:4])}"
                 if r['overrun_moved'] else '')
              + ('   MOVED' if r['moved'] else ''), flush=True)
    print(f"\n{n_moved} board(s) where the paste-only pads move a measure")
    return 0


if __name__ == '__main__':
    sys.exit(main())
