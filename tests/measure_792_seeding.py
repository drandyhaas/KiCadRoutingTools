#!/usr/bin/env python3
"""What #792's seeder change does, measured per board and paired.

NOT named `test_*`, so `run_all.py` never collects it: it runs a full
`seed_from_intent` per arm per board (minutes), and it asserts nothing. It
WRITES `tests/792_seeding_rows.jsonl`, and `tests/test_792_seeding_claims.py`
is the committed change detector that re-derives the aggregates from it.

WHY THIS AND NOT A `test_placement_ab.py` ROW. That harness's `_run` calls
`quench(...)`; nothing in the table ever calls `seed_from_intent`, and
`_intent_for` emits without `derive_decaps`, so every existing row's intent
carries `decaps: {}` and no decap rule can arm. Hosting a seed row there means a
new arm kind inside a gate seven pinned rows depend on -- a change to the gate
made in the same PR whose evidence the gate would be producing. So the >= 3
distinct boards and the recorded-rather-than-argued discipline are reproduced
HERE, and the harness is left alone. That is stated in the PR body rather than
left for a reviewer to notice.

WHAT THESE ROWS DO AND DO NOT MEASURE. The `off` arm carries NO `decaps` key,
so stage 2.5 does not run in it at all. `off` vs `on` therefore measures what
DECLARING `decaps.max_distance_mm` costs -- a property of #704's feature -- and
NOT what #792 changed. That distinction was got wrong in the first draft of the
claims test, which gated on "pin geometry does not get worse" and read a 32%
rp2350 regression as a defect in this change; rp2350 has ZERO orphans, so the
narrowing cannot touch it, and the whole delta is the pin stage claiming 15 caps
that nothing claimed before.

The base-vs-head comparison -- the one that measures #792 itself -- is a
separate paired run over both worktrees, quoted in the PR body: every CONTROL
arm byte-identical, no part newly unseated, ulx3s 181 parts moved with 19
put-backs. It is not committed here because it needs two trees.

THREE ARMS per board, all from the same rng seed:

  off    no `decaps` key            -- the stage never runs
  on     `decaps` as emitted        -- the stage runs, with the narrowing and
                                      the put-back
  chips  `+ decap_owner_chips=True` -- also the grouper's answer to "what is an
                                      IC", which is flagged OFF pending this

WHAT IS MEASURED, and why it is not the seeder's own notes. `pin_gap_sum` and
`pin_gap_max` come from `floorplan.supply_pins` + `_pin_gap` on the SEEDED
board -- an independent grader, not the stage's commentary. Counting "decap
for ..." notes would be the seeder grading itself, which is the circularity
`test_placement_ab`'s own header warns about; the notes are recorded as
evidence and never compared.
"""
import json
import os
import random
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for _d in ('', 'py_placer', 'py_router', 'py_tools'):
    _p = os.path.join(ROOT, _d)
    if _p not in sys.path:
        sys.path.insert(0, _p)

import run_utils                                            # noqa: E402
from kicad_parser import parse_kicad_pcb                    # noqa: E402
from placement import floorplan as fp                       # noqa: E402
from placement import groups as groups_mod                  # noqa: E402
from placement import seeder                                # noqa: E402

ROWS_PATH = os.path.join(TESTS_DIR, '792_seeding_rows.jsonl')
SEED = 11


def _pin_gaps(pcb, poses):
    """Sum and max of every supply pin's gap to its nearest same-rail cap, at
    the SEEDED poses. Independent of the seeder: this is the grader's own
    `supply_pins` + `_pin_gap`, over a board whose parts have been moved."""
    for ref, (x, y, rot) in poses.items():
        fpp = pcb.footprints.get(ref)
        if fpp is None:
            continue
        dx, dy = x - fpp.x, y - fpp.y
        if dx or dy:
            for pad in fpp.pads:
                pad.global_x += dx
                pad.global_y += dy
            fpp.x, fpp.y = x, y
    recs = fp.supply_pins(pcb)
    by_net = fp._decap_caps_by_net(pcb)
    gaps = []
    for ref, rec in recs.items():
        for pad, _net in rec['pins']:
            caps = [c for c in by_net.get(pad.net_id, ())
                    if c.reference != ref]
            gs = [fp._pin_gap(pad, c, pad.net_id) for c in caps]
            gs = [g for g in gs if g is not None]
            if gs:
                gaps.append(min(gs))
    return (round(sum(gaps), 4) if gaps else None,
            round(max(gaps), 4) if gaps else None, len(gaps))


def _arm(path, doc, **kw):
    pcb = parse_kicad_pcb(path)
    res = seeder.seed_from_intent(pcb, path, fp.intent_from_dict(doc, path),
                                  random.Random(SEED),
                                  group_sources=('kicad', 'sheet'), **kw)
    poses = {p['reference']: (p['new_x'], p['new_y'], p['new_rotation'])
             for p in res['placements']}
    s, m, n = _pin_gaps(parse_kicad_pcb(path), poses)
    return {
        'poses': {k: [round(v[0], 4), round(v[1], 4), round(v[2], 3)]
                  for k, v in sorted(poses.items())},
        'unseated': sorted(res.get('unseated') or []),
        'pin_gap_sum': s, 'pin_gap_max': m, 'pins_measured': n,
        # Evidence, never compared: the stage grading itself.
        'claimed': len([x for x in res.get('notes') or []
                        if 'decap for' in x]),
        'put_back': len([x for x in res.get('notes') or []
                         if 'zone-packed into' in x]),
        'declined': len([x for x in res.get('notes') or []
                         if 'falls through' in x]),
    }


def main(argv=None):
    import argparse
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--boards', nargs='*', default=None)
    ap.add_argument('--out', default=ROWS_PATH)
    a = ap.parse_args(argv)

    paths = run_utils.corpus_boards()
    if not paths:
        print("REFUSED: corpus_boards() returned nothing (git could not "
              "answer). This measures a FIXED set; it must not measure an "
              "unidentifiable one.", file=sys.stderr)
        return 2
    if a.boards:
        want = set(a.boards)
        paths = [p for p in paths
                 if os.path.basename(p).replace('.kicad_pcb', '') in want]

    rows = []
    for path in paths:
        name = os.path.basename(path).replace('.kicad_pcb', '')
        pcb = parse_kicad_pcb(path)
        doc = fp.emit_intent(pcb, path, derive_decaps=True)
        if (doc.get('decaps') or {}).get('max_distance_mm') is None:
            # No limit derivable -> the stage never arms -> nothing to pair.
            # Recorded rather than dropped, so a reader can tell "measured and
            # inert" from "not measured".
            rows.append({'board': name, 'armed': False,
                         'reason': 'no decaps.max_distance_mm derivable'})
            continue
        off_doc = dict(doc)
        off_doc['decaps'] = {}
        row = {'board': name, 'armed': True,
               'scope': len({c for caps in
                             groups_mod.decap_populations(pcb)[0].values()
                             for c, _d in caps})
                        + len(groups_mod.decap_populations(pcb)[1]),
               'orphans': len(groups_mod.decap_populations(pcb)[2])}
        try:
            row['off'] = _arm(path, off_doc)
            row['on'] = _arm(path, doc)
            row['chips'] = _arm(path, doc, decap_owner_chips=True)
        except Exception as exc:                            # noqa: BLE001
            row['error'] = '%s: %s' % (type(exc).__name__, exc)
        rows.append(row)
        print('%-26s %s' % (name, row.get('error') or
                            'sum %s -> %s -> %s' % (
                                row['off']['pin_gap_sum'],
                                row['on']['pin_gap_sum'],
                                row['chips']['pin_gap_sum'])),
              file=sys.stderr)

    with open(a.out, 'w', encoding='utf-8') as fh:
        for r in rows:
            fh.write(json.dumps(r, sort_keys=True) + '\n')
    print('wrote %d row(s) to %s' % (len(rows), a.out), file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())
