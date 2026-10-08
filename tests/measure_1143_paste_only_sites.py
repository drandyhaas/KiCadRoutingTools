#!/usr/bin/env python3
"""#1143: what reading an APERTURE-ONLY pad as a pad moves, per site, per board.

An aperture-only pad is a pad that puts nothing on a copper layer: not NPTH,
no drill, and no `*.Cu` entry in `pad.layers` (a paste or mask aperture, e.g.
the split paste windows of a thermal pad, or a jetson spacer's paste ring).
#1128 made `legality.occupancy_shape` and `pad_copper_overrun_mm` skip them;
#1143 names three more placement measures that do not, and the research sweep
for it found about twenty more sites of the same class, some of them in
routing code.

Two arms on ONE parse, so the difference is that population and nothing else:

* `as-is`     -- the board as parsed;
* `stripped`  -- a deep copy with every aperture-only pad removed.

The predicate below is this script's OWN (`_aperture_only`), written from the
definition and independent of the code under test, so the script can grade a
fix rather than repeat it. NPTH and drilled pads are KEPT: a mounting hole is
physical extent, and dropping it moves splitflap H6/H7 and test_837's census.

Before the #1143 fix the arm difference is the fix's impact. After it, every
PLACEMENT site must agree in both arms on every board (the as-parsed arm then
equals the stripped one), and `--check-predicate` asserts the code's predicate
matches this script's on every pad. The three ROUTING readers (`ROUTING_SITES`:
the package detector, the BGA pitch and the pin count the QFN auto-pick ranks
by) were left by #1143 and read the pins only since #1148, so after it they
must agree in both arms too.

KiCad 10.0.0's RoyalBlue54L-Feather demo carries 349 copies of
`(curved_edges no)filter_ratio 0.9)`; since #1149 the parser reads them the way
KiCad does, so it is ordinary evidence.

Not collected by run_all (no `test_` prefix).

    python3 -X utf8 tests/measure_1143_paste_only_sites.py [--demos DIR]
        [--board PATH ...] [--json-out PATH] [--check-predicate]
"""
import argparse
import contextlib
import copy
import io
import json
import math
import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for _d in ('py_router', 'py_placer', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _d))
sys.path.insert(0, TESTS_DIR)

CLEARANCE = 0.2
TOL = 1e-6
#: The ROUTING side's readers (py_router), pins-only since #1148.
ROUTING_SITES = ('package_type', 'bga_pitch', 'pad_count')
#: A board whose file is known not to parse as KiCad reads it. Empty since
#: #1149 taught the parser RoyalBlue54L-Feather's unbracketed teardrop tokens.
SUSPECT = {}


def _aperture_only(p):
    """This script's own predicate: not NPTH, no drill, no `*.Cu` layer."""
    if getattr(p, 'pad_type', '') == 'np_thru_hole':
        return False
    if (getattr(p, 'drill', 0) or 0) > 0:
        return False
    return not any(str(l).endswith('.Cu') for l in (p.layers or ()))


def _strip(pcb):
    out = copy.deepcopy(pcb)
    for fp in out.footprints.values():
        if any(_aperture_only(p) for p in (fp.pads or ())):
            fp.pads = [p for p in fp.pads if not _aperture_only(p)]
    return out


def _quiet(fn, *a, **k):
    with contextlib.redirect_stdout(io.StringIO()), \
            contextlib.redirect_stderr(io.StringIO()):
        return fn(*a, **k)


def _r(v, nd=4):
    if isinstance(v, float):
        return round(v, nd) if math.isfinite(v) else v
    if isinstance(v, (list, tuple)):
        return type(v)(_r(x, nd) for x in v)
    if isinstance(v, dict):
        return {k: _r(x, nd) for k, x in v.items()}
    return v


def _site_values(pcb, path, refs):
    """{site: value} on one arm. Every site is guarded on its own, so one
    raising site is reported as such instead of hiding the others."""
    from placement import (legality, escape, groups, part_class, pose_ops,
                           body, floorplan)
    from placement.utility import compute_footprint_bbox_local
    import placement_score as PS
    import kicad_parser as KP
    out = {}

    def site(name, fn):
        try:
            out[name] = _r(_quiet(fn))
        except Exception as exc:                          # noqa: BLE001
            out[name] = f'RAISED {type(exc).__name__}: {str(exc)[:80]}'

    fps = pcb.footprints
    site('bbox_local', lambda: {r: tuple(compute_footprint_bbox_local(fps[r]))
                                for r in refs})

    def _bodies():
        bb = body.board_bodies(pcb, path)
        return {r: (bb[r].occupancy_local, bb[r].body_local,
                    getattr(bb[r], 'drawn_local', None), bb[r].source)
                for r in refs if r in bb}
    site('board_bodies', _bodies)
    site('graded_rect', lambda: {g.ref: g.rect for g in
                                 legality.graded_parts_from_file(pcb, path)
                                 if g.ref in refs})

    def _overlap():
        g = legality.grade_body_overlap(pcb, CLEARANCE, pcb_file=path)
        return {'blocking': int(g.get('blocking') or 0),
                'advisory': int(g.get('advisory') or 0),
                'pairs': len(g.get('pairs') or ())}
    site('grade_body_overlap', _overlap)
    site('assembly_census', lambda: {k: v for k, v in
                                     legality.assembly_census(pcb).items()
                                     if k != 'basis'})
    site('pad_area_balance', lambda: {k: v for k, v in
                                      PS.pad_area_balance(pcb).items()
                                      if k in ('value', 'abstained',
                                               'npth_pads_excluded')})
    site('pad_pitch', lambda: {r: escape.pad_pitch(fps[r]) for r in refs})
    site('fine_pitch_parts', lambda: sorted(escape.fine_pitch_parts(pcb)))

    def _ledger():
        led = escape.escape_ledger(pcb, pcb_file=path)
        return {e.ref: (e.pitch_mm, e.interior_pads, len(e.faces))
                for e in led}
    site('escape_ledger', _ledger)
    site('part_copper_geometry', lambda: {
        r: g.rect for r, g in legality.part_copper_geometry(
            fps, CLEARANCE).items() if r in refs})
    site('edge_facing', lambda: {k: v for k, v in
                                 PS.edge_facing(pcb, path).items()
                                 if k in ('value', 'pads', 'parts')})

    def _cls():
        res = {}
        for r in refs:
            c = part_class.classify_part(fps[r], r)
            band = (part_class.default_band(c.name, fps[r])
                    if c.name else None)
            res[r] = (c.name, c.confidence, band)
        return res
    site('classify_part', _cls)
    site('elect_tethers', lambda: sorted(
        (c, ic, d) for c, ic, d in groups._elect_tethers(pcb)
        if (c in refs or ic in refs)))
    site('decap_census', lambda: {
        k: v for k, v in floorplan.decap_census(pcb).items()
        if k in ('tethers', 'beyond_radius', 'beyond_radius_refs',
                 'worst_beyond_mm', 'no_rail_chip')})
    site('chip_refs', lambda: sorted(groups.chip_refs(pcb)))
    site('part_centre', lambda: {r: pose_ops.part_centre(pcb, r)
                                 for r in refs})

    def _bodyless():
        res = {}
        for r in refs:
            s = legality.bodyless_pad_shape(fps[r])
            res[r] = None if s is None else round(
                float(getattr(s, 'area', 0.0)), 4)
        return res
    site('bodyless_pad_shape', _bodyless)
    site('derive_groups', lambda: {
        k: v for k, v in groups.derive_groups(
            pcb, ('kicad', 'sheet', 'netprefix', 'decap')).items()
        if set(v) & set(refs)})
    # routing reach (py_router): the package detector, the BGA pitch and the
    # QFN auto-pick's ranking key, each read through the code that uses it.
    from qfn_fanout import autopick_rank
    site('package_type', lambda: {r: KP.detect_package_type(fps[r])
                                  for r in refs})
    site('bga_pitch', lambda: {r: KP.detect_bga_pitch(fps[r]) for r in refs})
    site('pad_count', lambda: {r: autopick_rank(fps[r])[1] for r in refs})
    return out


def measure(path):
    from kicad_parser import parse_kicad_pcb
    pcb = _quiet(parse_kicad_pcb, path)
    pop = {}
    netted = 0
    only = []
    for r, fp in pcb.footprints.items():
        ap = [p for p in (fp.pads or ()) if _aperture_only(p)]
        if ap:
            pop[r] = len(ap)
            netted += sum(1 for p in ap if getattr(p, 'net_id', 0))
            if len(ap) == len(fp.pads or ()):
                only.append(r)
    res = {'aperture_pads': sum(pop.values()), 'parts': len(pop),
           'aperture_only_parts': sorted(only), 'netted_aperture_pads': netted}
    if not pop:
        return res
    refs = sorted(pop)
    a = _site_values(pcb, path, refs)
    b = _site_values(_strip(pcb), path, refs)
    moved = {}
    for k in a:
        if a[k] != b.get(k):
            moved[k] = {'as_is': a[k], 'stripped': b.get(k)}
    res['moved'] = moved
    res['sites'] = sorted(a)
    return res


def _diff_line(site, d):
    a, b = d['as_is'], d['stripped']
    if isinstance(a, dict) and isinstance(b, dict):
        keys = [k for k in sorted(set(a) | set(b), key=str)
                if a.get(k) != b.get(k)]
        shown = ', '.join(f"{k}: {a.get(k)} -> {b.get(k)}" for k in keys[:4])
        more = f' (+{len(keys) - 4} more)' if len(keys) > 4 else ''
        return f"    {site}: {len(keys)} moved -- {shown}{more}"
    return f"    {site}: {a} -> {b}"


def check_predicate(boards):
    """The code's predicate must agree with this script's on every pad."""
    from kicad_parser import parse_kicad_pcb
    import kicad_parser as KP
    fn = getattr(KP, 'pad_is_aperture_only', None)
    if fn is None:
        print('check-predicate: kicad_parser.pad_is_aperture_only not found')
        return 2
    bad = 0
    n = 0
    for b in boards:
        pcb = _quiet(parse_kicad_pcb, b)
        for r, fp in pcb.footprints.items():
            for p in fp.pads or ():
                n += 1
                if bool(fn(p)) != _aperture_only(p):
                    bad += 1
                    if bad <= 5:
                        print(f'  DISAGREE {os.path.basename(b)} {r}.'
                              f'{p.pad_number} {p.pad_type} {p.layers}')
    print(f'check-predicate: {n} pads, {bad} disagreement(s)')
    return 1 if bad else 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--demos', default=None,
                    help="Also measure every .kicad_pcb under this directory "
                         "(e.g. the KiCad install's share/kicad/demos)")
    ap.add_argument('--board', action='append', default=None)
    ap.add_argument('--json-out', default=None)
    ap.add_argument('--check-predicate', action='store_true',
                    help='After measuring, assert the code predicate equals '
                         "this script's on every pad (exit 1 on a mismatch)")
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
    rows = {}
    n_moved = n_routing = 0
    for b in boards:
        name = os.path.basename(b)
        try:
            r = measure(b)
        except Exception as exc:                          # noqa: BLE001
            print(f"{name}: UNMEASURABLE {type(exc).__name__}: "
                  f"{str(exc)[:120]}", flush=True)
            continue
        rows[b] = r
        if not r['aperture_pads']:
            print(f"{name}: no aperture-only pad", flush=True)
            continue
        flag = f"   [SUSPECT FILE: {SUSPECT[name]}]" if name in SUSPECT else ''
        moved = r.get('moved') or {}
        placement = [k for k in moved if k not in ROUTING_SITES]
        if name not in SUSPECT:
            n_moved += bool(placement)
            n_routing += any(k in ROUTING_SITES for k in moved)
        print(f"{name}: {r['aperture_pads']} aperture-only pad(s) on "
              f"{r['parts']} part(s), {r['netted_aperture_pads']} netted; "
              f"aperture-only parts {r['aperture_only_parts'] or 'none'}; "
              f"{len(placement)} placement site(s) moved, "
              f"{len(moved) - len(placement)} routing{flag}", flush=True)
        for k in sorted(moved):
            print(_diff_line(k, moved[k])
                  + ('   [routing reader]' if k in ROUTING_SITES else ''),
                  flush=True)
    print(f"\n{n_moved} board(s) where aperture-only pads move a PLACEMENT "
          f"site; {n_routing} where they move a routing reader "
          f"({', '.join(ROUTING_SITES)}) (suspect files excluded)")
    if args.json_out:
        with open(args.json_out, 'w', encoding='utf-8') as f:
            json.dump(rows, f, indent=1, sort_keys=True, default=str)
    if args.check_predicate:
        return check_predicate([b for b in boards
                                if os.path.basename(b) not in SUSPECT])
    return 0


if __name__ == '__main__':
    sys.exit(main())
