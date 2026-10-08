#!/usr/bin/env python3
"""#1142: what a per-cap `decap_ungraded` promotion changes, measured.

`decap_distance` grades only the caps within `groups.DECAP_RADIUS_MM` (5 mm)
of the chip they tether to; a cap beyond it is `decap_ungraded`, a WARN. #1102
made `emit_intent(decaps_from=REF)` promote that rule to ERROR board-wide, but
only when REF keeps EVERY rail cap within the radius -- and none of the seven
#1105 references does, so a decoupler the reference keeps 1 mm from its IC and
a seed strands 15 mm away reads as a WARN everywhere.

#1142's fix is per cap: a cap REF keeps within the radius of its chip (and
that the graded board carries with the same footprint) is held to it -- ERROR
when the graded board leaves it beyond -- and a cap REF itself keeps beyond
stays a WARN.

This script measures that change BEFORE it exists, through an ORACLE that
applies the per-cap rule to the grade's own findings:

    held  = REF's near caps (`groups.decap_populations` at the radius) whose
            (ref, footprint) pair the graded board shares
    NEW   = every non-`decap_ungraded` error, plus every `decap_ungraded`
            finding (warn or error) whose ref is held

and, once the change exists (the emitted intent carries
`decaps.within_radius_refs`), grades with the real code too and checks the
code equals the oracle and the held list equals the oracle's.

Two parts:

(a) FIXED POINT -- every corpus board and KiCad 10 demo, emitted with
    `--decaps-from` ITSELF through the CLI and graded on itself. A board must
    never fail its own reference: NEW has 0 `decap_ungraded` errors on every
    board (exit 1 otherwise).
(b) THE ISSUE'S PILES -- the seven #1105 boards staged as unaided piles
    (`test_placement_ab._pile_inputs`, the pilot's own basis), seeded once
    (place_seed's scope, stage 3.5 off) and graded: decap_ungraded per
    severity, total errors, and which exit-code consumers flip (check_floorplan
    and place_seed exit 4 on an error; the free agent's DONE test needs 0).
    With the change in place, `--intent-arms` also seeds each pile under the
    intent WITHOUT the held list, so the seed's own decisions under the two
    intents can be compared (the seeder's repair and no-worse checks read
    ERRORs only).

Not collected by run_all (no `test_` prefix).

    python3 -X utf8 tests/measure_1142_ungraded_per_cap.py [--demos DIR]
        [--no-piles] [--no-fixed-point] [--pile NAME ...] [--intent-arms]
        [--json-out PATH]
"""
import argparse
import contextlib
import copy
import io
import json
import os
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for _d in ('py_router', 'py_placer', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _d))
sys.path.insert(0, TESTS_DIR)

PILES = ('esp_prog', 'splitflap_driver', 'tigard', 'watchy', 'glasgow_revC',
         'ulx3s', 'orangecrab_ext_pll')
SUSPECT = {'RoyalBlue54L-Feather.kicad_pcb'}   # see measure_1143's docstring


def _quiet(fn, *a, **k):
    with contextlib.redirect_stdout(io.StringIO()), \
            contextlib.redirect_stderr(io.StringIO()):
        return fn(*a, **k)


def _parse(path):
    from kicad_parser import parse_kicad_pcb
    return _quiet(parse_kicad_pcb, path)


def held_by(ref_pcb, graded_pcb, radius=None):
    """The oracle's held set: REF's near caps the graded board carries with
    the same footprint."""
    from placement import groups
    near, _beyond, _orph = groups.decap_populations(
        ref_pcb, radius=radius or groups.DECAP_RADIUS_MM)
    theirs = {r: f.footprint_name for r, f in ref_pcb.footprints.items()}
    ours = {r: f.footprint_name for r, f in graded_pcb.footprints.items()}
    return sorted(c for caps in near.values() for c, _d in caps
                  if c in ours and ours[c] == theirs.get(c))


def _grade(intent, pcb, path):
    from placement import floorplan as fp
    return _quiet(fp.grade, intent, pcb, path)


def _count(g, held):
    errs = list(g.errors)
    warns = list(g.warnings)
    du = [v for v in errs + warns if v.rule == 'decap_ungraded']
    old = {'errors': len(errs),
           'du_error': sum(1 for v in du if v.severity == 'error'),
           'du_warn': sum(1 for v in du if v.severity != 'error')}
    new_errs = ([v for v in errs if v.rule != 'decap_ungraded']
                + [v for v in du if v.ref in held])
    new = {'errors': len(new_errs),
           'du_error': sum(1 for v in du if v.ref in held),
           'du_warn': sum(1 for v in du if v.ref not in held)}
    return old, new, sorted(v.ref for v in du if v.ref in held)


def _emit(board, ref, ipath, allow_unplaced=False):
    argv = [sys.executable, '-X', 'utf8',
            os.path.join(ROOT, 'py_tools', 'check_floorplan.py'), board,
            '--emit-intent', ipath, '--decaps-from', ref]
    if allow_unplaced:
        argv.insert(4, '--allow-unplaced')
    r = subprocess.run(argv, capture_output=True, text=True,
                       encoding='utf-8', errors='replace', cwd=ROOT)
    if r.returncode != 0 or not os.path.isfile(ipath):
        raise RuntimeError(f"emit exited {r.returncode}: "
                           f"{(r.stdout + r.stderr)[-300:]}")
    with open(ipath, encoding='utf-8') as fh:
        return json.load(fh), r.stdout


def _code_view(doc, g):
    """What the CODE says, once the change exists: its held list and its
    per-severity `decap_ungraded` count."""
    lst = (doc.get('decaps') or {}).get('within_radius_refs')
    du = [v for v in list(g.errors) + list(g.warnings)
          if v.rule == 'decap_ungraded']
    return {'has_list': lst is not None,
            'held': sorted(lst or ()),
            'severity_key': 'decap_ungraded' in (doc.get('severity') or {}),
            'du_error': sum(1 for v in du if v.severity == 'error'),
            'errors': len(g.errors)}


def fixed_point(boards):
    from placement import floorplan as fp
    rows = {}
    bad = 0
    for b in boards:
        name = os.path.basename(b)
        td = tempfile.mkdtemp(prefix='m1142fp_')
        try:
            doc, _out = _emit(b, b, os.path.join(td, 'i.json'))
        except Exception as exc:                          # noqa: BLE001
            print(f"  {name}: not emitted ({str(exc)[:120]})", flush=True)
            continue
        if (doc.get('decaps') or {}).get('max_distance_mm') is None:
            print(f"  {name}: no decap limit armed (no tethers), skipped",
                  flush=True)
            continue
        pcb = _parse(b)
        intent = fp.load_intent(os.path.join(td, 'i.json'))
        g = _grade(intent, pcb, b)
        held = held_by(pcb, pcb)
        old, new, stranded = _count(g, set(held))
        code = _code_view(doc, g)
        ok = new['du_error'] == 0 and (not code['has_list']
                                       or code['du_error'] == 0)
        bad += (not ok) and name not in SUSPECT
        rows[name] = {'held': len(held), 'old': old, 'new': new,
                      'code': code, 'ok': ok}
        cen = (doc.get('context') or {}).get('decap_census') or {}
        print(f"  {name}: held {len(held)}, reference beyond "
              f"{cen.get('beyond_radius')}; decap_ungraded errors old "
              f"{old['du_error']} -> new {new['du_error']}"
              + (f" (code {code['du_error']})" if code['has_list'] else '')
              + ('' if ok else '   FIXED POINT BROKEN'), flush=True)
    return rows, bad


def piles(names, intent_arms=False):
    import test_placement_ab as AB
    from placement import floorplan as fp
    rows = {}
    for name in names:
        board = os.path.join(ROOT, 'kicad_files', name + '.kicad_pcb')
        d = tempfile.mkdtemp(prefix='m1142pile_')
        pile, intent, doc, seed_refs = _quiet(AB._pile_inputs, board, d)
        out = os.path.join(d, 'seed', name + '.kicad_pcb')
        os.makedirs(os.path.dirname(out))
        _quiet(AB._run_seed, pile, out, intent, {'seed_refs': seed_refs},
               ignore_nets=['GND'])
        seeded = _parse(out)
        g = _grade(intent, seeded, out)
        held = held_by(_parse(board), seeded)
        old, new, stranded = _count(g, set(held))
        code = _code_view(doc, g)
        row = {'held': len(held), 'old': old, 'new': new,
               'stranded_held': stranded, 'code': code,
               'exit4_flips': old['errors'] == 0 and new['errors'] > 0}
        if code['has_list']:
            row['code_matches_oracle'] = (code['held'] == held
                                          and code['du_error']
                                          == new['du_error'])
        if intent_arms and code['has_list']:
            # The same pile seeded under the intent WITHOUT the held list:
            # what the seeder's own error-reading checks decided differently.
            doc2 = copy.deepcopy(doc)
            doc2['decaps'].pop('within_radius_refs', None)
            doc2['decaps'].pop('within_radius_mm', None)
            ip2 = os.path.join(d, 'pile_intent_nolist.json')
            with open(ip2, 'w', encoding='utf-8') as fh:
                json.dump(doc2, fh, indent=1)
            out2 = os.path.join(d, 'seed_nolist', name + '.kicad_pcb')
            os.makedirs(os.path.dirname(out2))
            _quiet(AB._run_seed, pile, out2, fp.load_intent(ip2),
                   {'seed_refs': seed_refs}, ignore_nets=['GND'])
            a = {r: (round(f.x, 4), round(f.y, 4), round(f.rotation or 0, 3))
                 for r, f in _parse(out2).footprints.items()}
            b = {r: (round(f.x, 4), round(f.y, 4), round(f.rotation or 0, 3))
                 for r, f in seeded.footprints.items()}
            moved = sorted(r for r in b if a.get(r) != b[r])
            g2 = _grade(intent, _parse(out2), out2)
            row['intent_arms'] = {'parts_placed_differently': len(moved),
                                  'refs': moved[:12],
                                  'errors_with_list': len(g.errors),
                                  'errors_without_list': len(g2.errors)}
        rows[name] = row
        print(f"  {name}: held {len(held)}; seed strands {len(stranded)} "
              f"held cap(s) {stranded[:6]}; decap_ungraded error/warn "
              f"{old['du_error']}/{old['du_warn']} -> "
              f"{new['du_error']}/{new['du_warn']}; errors {old['errors']} -> "
              f"{new['errors']}"
              + ('   exit-4 FLIPS' if row['exit4_flips'] else '')
              + (f"; code matches oracle: {row['code_matches_oracle']}"
                 if 'code_matches_oracle' in row else '')
              + (f"; intent arms: {row['intent_arms']['parts_placed_differently']}"
                 f" part(s) placed differently, errors "
                 f"{row['intent_arms']['errors_without_list']} (no list) vs "
                 f"{row['intent_arms']['errors_with_list']} (list)"
                 if 'intent_arms' in row else ''), flush=True)
    return rows


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--demos', default=None)
    ap.add_argument('--no-piles', action='store_true')
    ap.add_argument('--no-fixed-point', action='store_true')
    ap.add_argument('--pile', action='append', default=None)
    ap.add_argument('--intent-arms', action='store_true')
    ap.add_argument('--json-out', default=None)
    args = ap.parse_args(argv)
    out = {}
    rc = 0
    if not args.no_fixed_point:
        from run_utils import corpus_boards
        boards = list(corpus_boards())
        if args.demos:
            for dp, _dn, fns in os.walk(args.demos):
                boards += [os.path.join(dp, f) for f in sorted(fns)
                           if f.endswith('.kicad_pcb')]
        print('(a) fixed point: each board graded on its own --decaps-from '
              'intent', flush=True)
        out['fixed_point'], bad = fixed_point(boards)
        print(f"  -> {bad} board(s) fail their own reference", flush=True)
        rc = 1 if bad else 0
    if not args.no_piles:
        print('(b) the #1105 piles, seeded once and graded', flush=True)
        out['piles'] = piles(args.pile or PILES, args.intent_arms)
        mism = [n for n, r in out['piles'].items()
                if r.get('code_matches_oracle') is False]
        if mism:
            print(f"  -> code disagrees with the oracle on {mism}")
            rc = 1
    if args.json_out:
        with open(args.json_out, 'w', encoding='utf-8') as fh:
            json.dump(out, fh, indent=1, sort_keys=True, default=str)
    return rc


if __name__ == '__main__':
    sys.exit(main())
