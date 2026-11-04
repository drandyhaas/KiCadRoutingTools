"""#1476: place_simple prices a declared `keep_away[]` row.

The intent's `keep_away[]` -- two net groups no closer than GAP mm, pad edge
to pad edge -- was graded by check_floorplan and acted on by nothing, so a
placement learned only afterwards that it broke the rule. The anneal now
prices each part pair's shortfall at `KEEP_AWAY_WEIGHT` per mm, with the
grade's own measurement (`floorplan._keep_away_gaps` on `legality.pad_rect`
rects, two pads of one net or of one part never measured), whenever the
intent declares a row.

Checked here: the rule reaches the model; a move is priced locally exactly
as `total()` prices the board from scratch (the anneal adds the local
changes up, so a drift would steer it); and the model's shortfall is the
grade's, pair for pair, on a board written at the moved poses.
"""

import json
import os
import random
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

import place_simple as ps                                     # noqa: E402
from kicad_parser import parse_kicad_pcb                      # noqa: E402
from placement import floorplan as fp                         # noqa: E402
from placement.grouping import load_groups                    # noqa: E402
from placement.groups import parse_sources                    # noqa: E402
from placement.simple_anneal import _place                    # noqa: E402
from placement.writer import write_placed_output              # noqa: E402

BOARD = os.path.join(ROOT, 'tests', 'fixtures', '959', 'run29_pile.kicad_pcb')
RULE = '/+3.3V,Net-(C1-Pad1):/D_P,/D_N,Net-(C2-Pad1):3'
FAILS = []


def ok(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}"
          + (f"  ({detail})" if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


def _model(td):
    pcb = parse_kicad_pcb(BOARD)
    doc = fp.emit_intent(pcb, BOARD)
    doc['keep_away'] = [{'rule': RULE, 'why': 't'}]
    doc['min_reader'] = max(10, doc.get('min_reader') or 0)
    intent_path = os.path.join(td, 'intent.json')
    json.dump(doc, open(intent_path, 'w'))
    groups = os.path.join(td, 'groups.json')
    json.dump({'schema': 1, 'groups': [
        {'name': 'all', 'refs': sorted(r for r, f in pcb.footprints.items() if f.pads)}]},
        open(groups, 'w'))
    args = ps.build_parser().parse_args(
        [BOARD, os.path.join(td, 'o.kicad_pcb'), '--groups', groups, '--intent', intent_path])
    intent = fp.load_intent(intent_path)
    blocks, _ = fp.resolve_blocks(intent, pcb, parse_sources('auto'))
    state, model, movable, notes = ps.build(
        pcb, BOARD, load_groups(groups), doc, args, intent, blocks, set())
    return state, model, sorted(movable), notes, intent


def _grade_shortfall(state, intent, td):
    """The grade's keep_away findings on a board written at `state`'s poses:
    the summed shortfall and the part pairs, each pair once."""
    out = os.path.join(td, 'moved.kicad_pcb')
    write_placed_output(BOARD, out, [
        {'reference': r, 'new_x': p.x, 'new_y': p.y, 'new_rotation': p.rot % 360.0}
        for r, p in state.parts.items() if not p.locked])
    r = fp.grade(intent, parse_kicad_pcb(out), out)
    rows = [v for v in r.errors + r.warnings if v.rule == 'keep_away']
    return (sum(v.expected['gap_mm'] - v.measured['gap_mm'] for v in rows),
            {tuple(sorted((v.ref, v.measured['aggressor']['ref']))) for v in rows})


def test_priced_locally_as_from_scratch_and_as_graded(td):
    state, model, movable, notes, intent = _model(td)
    ok('the rule reaches the model', len(model.ka) == 1 and model._ka_parts,
       (len(model.ka), sorted(model._ka_parts)))
    ok('and is named once', any(n.startswith('keep_away priced: 1 rule') for n in notes),
       notes)
    rng = random.Random(1476)
    x0, y0, x1, y1 = model.pcb.board_info.board_bounds
    ka = sorted(model._ka_parts & set(movable))
    model.reprice_all()
    cur = model.total()['cost']
    worst, priced = 0.0, 0
    for k in range(300):
        # mostly keep-away parts, crowded into one corner so pairs come
        # inside the gap and leave it again; now and then two at once
        refs = rng.sample(ka, 2 if k % 5 == 0 else 1)
        new = {r: (rng.uniform(x0 + 3, x0 + 14), rng.uniform(y0 + 3, y0 + 14),
                   rng.choice((0.0, 90.0, 180.0, 270.0))) for r in refs}
        for r, (_x, _y, rot) in new.items():
            state.parts[r].ensure_rotation(rot)
        t = model.touched(refs)
        base = model.local(refs, t, reprice=False)
        snap = model.snapshot(t, refs)
        was = {r: (state.parts[r].x, state.parts[r].y, state.parts[r].rot) for r in refs}
        for r in refs:
            _place(state, r, *new[r])
        dc = model.local(refs, t) - base
        if rng.random() < 0.7:
            model.commit(refs, t[0])
            cur += dc
        else:
            for r in refs:
                _place(state, r, *was[r])
            model.restore(snap)
        if k % 50 == 49:
            tot = model.total()
            worst = max(worst, abs(tot['cost'] - cur))
            priced += tot['keep_away'] > 0
            cur = tot['cost']
    tot = model.total()
    ok('the moves came inside the gap', priced >= 3 and tot['keep_away'] > 0,
       (priced, tot['keep_away']))
    ok('a move is priced locally as total() prices it', worst < 1e-6, worst)
    short, pairs = _grade_shortfall(state, intent, td)
    # the grade reports each gap to 4 decimals
    ok('the shortfall is the grade\'s',
       abs(short - tot['keep_away']) < 1e-4 * max(1, len(pairs)),
       (short, tot['keep_away']))
    ok('pair for pair', pairs == set(model._kp), (sorted(pairs), sorted(model._kp)))


if __name__ == '__main__':
    with tempfile.TemporaryDirectory() as td:
        test_priced_locally_as_from_scratch_and_as_graded(td)
    print('ALL PASS' if not FAILS else f'FAILED: {FAILS}')
    sys.exit(1 if FAILS else 0)
