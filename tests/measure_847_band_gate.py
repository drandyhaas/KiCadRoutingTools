#!/usr/bin/env python3
"""What decides `check_channels --gate`, band by band, attributed to faces.

#847 records that the starved-face gate is NON-MONOTONE in the escape band --
it fires at 2.0, does not fire at 3.0, and fires again at 4.0 on the only
wrong-basin fixture there is -- and concludes, rightly, that no band can be
adopted by picking the value that makes one fixture agree.

This is the instrument that says WHY, because "non-monotone" is an observation
and a mechanism is what you can fix. It decomposes the verdict at each band
into the two channels that produce it and attributes each one to named faces on
a named SIDE of the delta.

WHAT IT FOUND, and the reason this file exists rather than a paragraph:

  1. `_starved_faces` is EMPTY at every band, on BOTH boards. The absolute
     starvation predicate (`supply == 0` AND `demand >= --min-demand`) never
     fires on this fixture at all. Every exit-4 verdict comes from
     `lost_last_lane`, which is deliberately NOT filtered by --min-demand.
     So #847's own proposed fix -- re-run the calibration, re-pin
     GATE_MIN_DEMAND -- cannot move this fixture in either direction.

  2. It is NOT U1 and NOT U30. The faces that decide the exit code are the
     diodes D21/D22/D23, carrying demand 1-3. The U1-east/U30 collapse the
     issue describes is real and is a SUPPLY fact, but it is a different
     phenomenon from the exit code, and the test file's docstring presents the
     two as one story.

  3. The non-monotonicity is `lost_last_lane`'s `before > 0` clause. Its
     predicate is `supply_now == 0 AND supply_base > 0`, and BOTH supplies fall
     as the band deepens, so each face contributes an INTERVAL of bands and the
     gate is a union of intervals. Nothing fires at 3.0 because D22/D23's
     BASELINE has itself saturated to 0 -- the damage is masked because the
     baseline became equally bad -- and D21 has not yet crossed. That is an
     ARTIFACT, not a property: "the baseline got worse too" is not evidence
     that the placement under test is fine.

THE RULE THIS FILE JUDGES BY, stated before the run rather than after: a
band-dependence is an ARTIFACT if the statistic is non-monotone while both
boards' underlying supplies are monotone -- the predicate manufactured it. It
is a GENUINE property if it survives a magnitude-valued statistic. Measured
below: the `supply == 0` flicker is an artifact; the fall-off at band 6.0,
where both boards saturate, is genuine and bounds the usable band from above.

    python3 -X utf8 tests/measure_847_band_gate.py --dir wk/run7/glasgow_revC
    python3 -X utf8 tests/measure_847_band_gate.py --dir DIR --out before.json
    python3 -X utf8 tests/measure_847_band_gate.py --diff before.json after.json

The default pair lives under `wk/`, which is GITIGNORED, so on a clean clone
this SKIPs loudly (exit 77 with a `SKIP:` line naming the path) rather than
quietly measuring nothing. `--now` / `--base` take any two boards, and the
tracked `tests/fixtures/run23/tigard_{damaged,placed}.kicad_pcb` pair is a
legitimate-reconstruction control that runs anywhere.
"""
import argparse
import json
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_router', 'py_placer', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _p))
os.environ.setdefault('KRT_NO_BANNER', '1')

import routing_defaults as D                                   # noqa: E402
import check_channels as CC                                    # noqa: E402
from kicad_parser import parse_kicad_pcb, detect_package_type  # noqa: E402
from placement import routability as R                         # noqa: E402
from list_nets import board_floor                              # noqa: E402
from fab_tiers import (fab_floor_min,                          # noqa: E402
                       count_copper_layers_in_file)

SKIP_EXIT = 77

#: The ladder #847 tabulates, plus the two values on either side of each flip
#: it reports, plus one deep enough to show BOTH boards saturating. `None` is
#: the shipped band, which is not the same as any literal -- it is whatever the
#: board's own floors resolve to, and naming it as a literal is how the two
#: bases in that issue's table got blended in the first place.
BANDS = (None, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 5.0, 6.0)


def engine_sha():
    try:
        return subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=ROOT,
                              capture_output=True, text=True).stdout.strip()
    except Exception:                                          # noqa: BLE001
        return 'unknown'


def floors(path, clearance=None, track=None):
    """check_channels' OWN resolution, board-first then fab-floored.

    Called rather than re-implemented: a measurement graded at a basis the
    shipped tool would not use is not a measurement of the shipped tool.
    """
    clr, clr_src = board_floor(path, 'clearance', clearance, D.CLEARANCE)
    trk, trk_src = board_floor(path, 'track_width', track, D.TRACK_WIDTH)
    try:
        fab = fab_floor_min(count_copper_layers_in_file(path))
    except Exception:                                          # noqa: BLE001
        fab = {}
    if fab.get('clearance') is not None:
        clr = max(clr, fab['clearance'])
    if fab.get('track_width') is not None:
        trk = max(trk, fab['track_width'])
    return trk, clr, trk_src, clr_src


def fine_refs(pcb, trk, clr):
    """check_channels' own auto-detection, so the ref set is the shipped one."""
    pitch_floor = 2 * (trk + clr)
    out = []
    for ref, fp in sorted((pcb.footprints or {}).items()):
        try:
            kind = detect_package_type(fp)
        except Exception:                                      # noqa: BLE001
            kind = None
        if kind in ('QFN', 'QFP', 'BGA'):
            out.append(ref)
            continue
        xs = sorted({round(q.global_x, 3) for q in (fp.pads or [])})
        gaps = [b - a for a, b in zip(xs, xs[1:]) if b - a > 1e-3]
        if gaps and min(gaps) < pitch_floor and len(fp.pads or []) >= 8:
            out.append(ref)
    return out


def ledgers(pcb, path, refs, trk, clr, grid, band):
    kw = dict(clearance=clr, track_width=trk, grid_step=grid, pcb_file=path)
    if band is not None:
        kw['escape_band_mm'] = band
    out = {}
    for ref in refs:
        rows = R.face_lane_ledger(pcb, ref, **kw)
        if rows:
            out[ref] = rows
    return out


def new_deficit_lanes(now, base):
    """Lanes of deficit the board under test ADDED, per (ref, face).

    The magnitude form of the same question `lost_last_lane` asks as a boolean.
    It strictly generalises it: a face going `supply k -> 0` at demand `d`
    contributes `d - max(0, d - k)` = `min(d, k)`, which is >= 1 exactly when
    `d >= 1 and k >= 1` -- `lost_last_lane`'s condition. What it does NOT do is
    saturate: once the baseline has also reached zero the boolean goes quiet
    while this keeps reporting the deficit that remains.
    """
    was = {(ref, r['face']): r['deficit_finest_grid']
           for ref, rows in (base or {}).items() for r in rows}
    total = 0
    rows_out = []
    for ref, rows in sorted((now or {}).items()):
        for r in rows:
            delta = r['deficit_finest_grid'] - was.get((ref, r['face']), 0)
            if delta > 0:
                total += delta
                rows_out.append({'ref': ref, 'face': r['face'],
                                 'lanes': delta,
                                 'demand': r['demand_nets'],
                                 'supply': r['supply_finest_grid']})
    rows_out.sort(key=lambda e: (-e['lanes'], e['ref'], e['face']))
    return total, rows_out


def gate_verdict(now, base, min_demand):
    """Reproduce `check_channels.main`'s gate, with the channels kept APART.

    The shipped code merges `lost_last_lane`'s hits into `new_starved` before
    deciding, so its output cannot tell you which channel fired. That merge is
    the whole reason the fixture's table reads as one phenomenon when it is
    two, so this keeps them separate and reports both.
    """
    starved_now = CC._starved_faces(now, min_demand)
    starved_base = CC._starved_faces(base, min_demand)
    was = {(r, f) for r, f, _d in starved_base}
    from_starved = [t for t in starved_now if (t[0], t[1]) not in was]
    seen = {(t[0], t[1]) for t in from_starved}
    from_lost = [t for t in CC.lost_last_lane(now, base)
                 if (t[0], t[1]) not in seen]
    return {
        'starved_now': [[r, f, d] for r, f, d in starved_now],
        'starved_base': [[r, f, d] for r, f, d in starved_base],
        'new_from_starvation': [[r, f, d] for r, f, d in from_starved],
        'new_from_lost_last_lane': [[r, f, d, b] for r, f, d, b in from_lost],
        'new_total': len(from_starved) + len(from_lost),
        'exit': 4 if (from_starved or from_lost) else 0,
    }


def sweep(now_path, base_path, bands, min_demand, clearance=None, track=None,
          grid=None):
    trk, clr, trk_src, clr_src = floors(now_path, clearance, track)
    grid = grid if grid is not None else D.GRID_STEP
    now_pcb = parse_kicad_pcb(now_path)
    base_pcb = parse_kicad_pcb(base_path)
    refs = fine_refs(now_pcb, trk, clr)
    doc = {'engine_sha': engine_sha(),
           'now': os.path.basename(now_path),
           'base': os.path.basename(base_path),
           'basis': {'track': trk, 'track_source': trk_src,
                     'clearance': clr, 'clearance_source': clr_src,
                     'grid': grid, 'min_demand': min_demand},
           'refs': refs, 'bands': {}}
    for band in bands:
        now = ledgers(now_pcb, now_path, refs, trk, clr, grid, band)
        base = ledgers(base_pcb, base_path, refs, trk, clr, grid, band)
        resolved = (now[refs[0]][0]['escape_band_mm']
                    if refs and refs[0] in now else None)
        verdict = gate_verdict(now, base, min_demand)
        total, contrib = new_deficit_lanes(now, base)
        # Supplies on BOTH sides for every face, which is what makes the
        # interval visible: a `lost_last_lane` hit is a face whose `now` has
        # crossed to 0 while its `base` has not, and the flip at 3.0 is the
        # base crossing too.
        supplies = {}
        for ref, rows in now.items():
            for r in rows:
                b = next((x for x in base.get(ref, [])
                          if x['face'] == r['face']), None)
                supplies[f'{ref} {r["face"]}'] = [
                    r['supply_finest_grid'],
                    b['supply_finest_grid'] if b else None,
                    r['demand_nets']]
        doc['bands'][str(band)] = {
            'resolved_mm': resolved, 'verdict': verdict,
            'new_deficit_lanes': total, 'contributors': contrib[:8],
            'supplies': supplies}
    return doc


def report(doc):
    b = doc['basis']
    print(f"{doc['now']} against {doc['base']}  "
          f"(track {b['track']} [{b['track_source']}] "
          f"clearance {b['clearance']} [{b['clearance_source']}] "
          f"grid {b['grid']} min-demand {b['min_demand']})")
    print(f"  refs: {', '.join(doc['refs'])}")
    print()
    print(f"  {'band':>6s} {'mm':>6s} {'exit':>5s} {'NEW':>4s} "
          f"{'starv':>6s} {'lost':>5s} {'newdef':>7s}  contributors")
    for key in doc['bands']:
        e = doc['bands'][key]
        v = e['verdict']
        contrib = ' '.join(f"{c['ref']}.{c['face']}({c['lanes']})"
                           for c in e['contributors'][:4])
        mm = e['resolved_mm']
        print(f"  {key:>6s} {mm if mm is None else f'{mm:.2f}':>6} "
              f"{v['exit']:>5d} {v['new_total']:>4d} "
              f"{len(v['new_from_starvation']):>6d} "
              f"{len(v['new_from_lost_last_lane']):>5d} "
              f"{e['new_deficit_lanes']:>7d}  {contrib}")
    print()
    print("  starv = NEW faces from the ABSOLUTE starvation predicate")
    print("  lost  = NEW faces from lost_last_lane (supply crossed to zero)")
    print("  newdef= sum of max(0, deficit_now - deficit_base), the MAGNITUDE")
    print()
    _intervals(doc)


def _intervals(doc):
    """Per face, the bands at which each channel fires -- the union of windows.

    This is the picture the word "non-monotone" stands in for. A face that
    fires over a CONTIGUOUS RANGE and then stops has not stopped being damaged;
    its baseline has caught up with it.
    """
    keys = list(doc['bands'])
    fired = {}
    for k in keys:
        for row in doc['bands'][k]['verdict']['new_from_lost_last_lane']:
            fired.setdefault(f'{row[0]} {row[1]}', []).append(k)
    if not fired:
        print('  no face ever fired lost_last_lane in this sweep')
        return
    print('  lost_last_lane, per face, with both sides supply at each band:')
    for face in sorted(fired):
        bands_fired = fired[face]
        line = []
        for k in keys:
            s = doc['bands'][k]['supplies'].get(face)
            mark = '*' if k in bands_fired else ' '
            line.append(f"{k}{mark}{s[0]}/{s[1] if s else '-'}"
                        if s else f'{k}{mark}-')
        print(f"    {face:14s} " + '  '.join(line))
    print('    (band*now/base;  * = fired.  A face stops firing when its '
          'BASE reaches 0 too.)')


def diff(a, b):
    print(f"engine {a['engine_sha'][:12]} -> {b['engine_sha'][:12]}")
    keys = [k for k in a['bands'] if k in b['bands']]
    moved = 0
    for k in keys:
        va, vb = a['bands'][k], b['bands'][k]
        if (va['verdict']['exit'] != vb['verdict']['exit']
                or va['new_deficit_lanes'] != vb['new_deficit_lanes']):
            moved += 1
            print(f"  band {k}: exit {va['verdict']['exit']} -> "
                  f"{vb['verdict']['exit']}, newdef "
                  f"{va['new_deficit_lanes']} -> {vb['new_deficit_lanes']}")
    gone = [k for k in a['bands'] if k not in b['bands']]
    if gone:
        print(f"  bands not in the AFTER run, not compared: {gone}")
    print(f"  {moved} of {len(keys)} compared bands moved")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--dir', default=os.path.join('wk', 'run7',
                                                  'glasgow_revC'),
                    help='directory holding rL_repair/perturbed (default: the '
                         'wk/ fixture, which is gitignored)')
    ap.add_argument('--now', default=None, help='the board under test')
    ap.add_argument('--base', default=None, help='the board it derives from')
    ap.add_argument('--clearance', type=float, default=None)
    ap.add_argument('--track-width', type=float, default=None)
    ap.add_argument('--grid-step', type=float, default=None)
    ap.add_argument('--min-demand', type=int, default=CC.GATE_MIN_DEMAND)
    ap.add_argument('--bands', nargs='*', type=float, default=None,
                    help='override the ladder (the shipped band is always '
                         'measured first)')
    ap.add_argument('--out', help='write the measurement to this JSON file')
    ap.add_argument('--diff', nargs=2, metavar=('BEFORE', 'AFTER'),
                    help='print the transition between two --out files')
    args = ap.parse_args()

    if args.diff:
        import run_utils
        a = json.load(open(run_utils.evidence(args.diff[0], 'BEFORE'),
                           encoding='utf-8'))
        b = json.load(open(run_utils.evidence(args.diff[1], 'AFTER'),
                           encoding='utf-8'))
        return diff(a, b)

    now = args.now or os.path.join(ROOT, args.dir, 'rL_repair.kicad_pcb')
    base = args.base or os.path.join(ROOT, args.dir, 'perturbed.kicad_pcb')
    for label, path in (('now', now), ('base', base)):
        if not (os.path.isfile(path) and os.path.getsize(path) > 0):
            # LOUD, with the path and the reason -- a check whose input is
            # missing tests nothing, and a silent skip reads as a pass.
            print(f'SKIP: the {label} board is not a readable non-empty file: '
                  f'{path}\n'
                  f'      The default pair lives under wk/, which is '
                  f'gitignored, so a clean clone has neither. Pass --now/--base '
                  f'(the tracked pair tests/fixtures/run23/tigard_damaged'
                  f'.kicad_pcb and tigard_placed.kicad_pcb runs anywhere), or '
                  f'--dir at a recorded run.')
            return SKIP_EXIT

    bands = tuple([None] + list(args.bands)) if args.bands else BANDS
    doc = sweep(now, base, bands, args.min_demand, args.clearance,
                args.track_width, args.grid_step)
    report(doc)
    if args.out:
        with open(args.out, 'w', encoding='utf-8') as fh:
            json.dump(doc, fh, indent=1, sort_keys=True)
        print(f'  JSON -> {args.out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
