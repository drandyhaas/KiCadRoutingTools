"""#1382, #1423: the intent's decap limits are a soft term, so a placement can
end past one with the objective calm about it and nothing naming the miss.

`placement.decap_repair` reads the model's own limit terms (the grade's
pairings, measured as the grade measures them):

  R. RESIDUAL  `residual(model)` names every term still past its limit -- the
               rail, the cap, the pin (or chip), the distance against the limit.
  M. REPAIR    `repair(...)` moves the cap to the nearest legal pose inside the
               limit, leaves a cap no legal pose frees where it is, and says
               how close the best legal pose got.
  C. COMMAND   `place_simple --repair-decaps` (also through --seeds, where a
               switch must reach each seed's process as its bare name) prints
               and reports both, in the JSON as `decap_residual` /
               `decap_repair`; without the flag the placement is unchanged and
               the residual is still WARNED.

    python3 tests/test_1382_decap_repair.py
"""
import json
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

import place_simple as ps                                     # noqa: E402
from kicad_parser import parse_kicad_pcb                      # noqa: E402
from placement import decap_repair                            # noqa: E402
from placement import floorplan as fp                         # noqa: E402
from placement.grouping import load_groups                    # noqa: E402
from placement.groups import parse_sources                    # noqa: E402
from placement.simple import legal, nearest_legal             # noqa: E402
from run_utils import check, evidence, tool                   # noqa: E402

FAILS = []
BOARD = os.path.join(ROOT, 'kicad_files', 'lvds_converter_dualclk.kicad_pcb')
LIMIT = 2.5


def ok(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}"
          + (f"  ({detail})" if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


def _files(td):
    pcb = parse_kicad_pcb(BOARD)
    doc = fp.emit_intent(pcb, BOARD)
    doc['decaps'] = {'max_pin_distance_mm': LIMIT}
    intent = os.path.join(td, 'intent.json')
    json.dump(doc, open(intent, 'w'))
    groups = os.path.join(td, 'groups.json')
    json.dump({'schema': 1, 'groups': [
        {'name': 'all', 'refs': sorted(r for r, f in pcb.footprints.items() if f.pads)}]},
        open(groups, 'w'))
    return intent, groups


def _model(td):
    intent_path, groups = _files(td)
    pcb = parse_kicad_pcb(BOARD)
    args = ps.build_parser().parse_args(
        [BOARD, os.path.join(td, 'o.kicad_pcb'), '--groups', groups, '--intent', intent_path])
    intent = fp.load_intent(intent_path)
    blocks, _ = fp.resolve_blocks(intent, pcb, parse_sources('auto'))
    state, model, movable, _notes = ps.build(
        pcb, BOARD, load_groups(groups), json.load(open(intent_path)), args, intent, blocks, set())
    return state, model, movable


def _legal_at(state):
    def legal_at(r, x, y, rot):
        return (legal(state, r, x, y, rot)
                and state.intent_clear(r, state.parts[r].grade_rects(x, y, rot)))
    return legal_at


def _put(state, model, ref, x, y):
    pose = nearest_legal(state, ref, x, y, 0.0, frozenset())
    state.parts[ref].ensure_rotation(pose[2])
    state.apply_move(ref, *pose)
    model.reprice_all()
    model.total()
    return pose


def test_residual_and_repair(td):
    state, model, movable = _model(td)
    ok('the limit is priced as a term', len(model._lterms) == 3, len(model._lterms))
    ok('no residual where the caps are at their pins', decap_repair.residual(model) == [])
    _put(state, model, 'C2', 121.0, 63.0)       # the cap 8 mm off IC2's supply pin
    rows = decap_repair.residual(model)
    ok('the miss is named: rail, cap, pin, distance, limit',
       len(rows) == 1 and rows[0]['ic'] == 'IC2' and rows[0]['pin'] == '14'
       and rows[0]['cap'] == 'C2' and rows[0]['rail'] == '/VCC'
       and rows[0]['limit'] == LIMIT and rows[0]['value'] > 8.0, rows)
    text = decap_repair.residual_text(rows[0])
    ok('... in words', 'IC2 pin 14 (/VCC)' in text and 'from C2' in text
       and f'limit {LIMIT:.2f} mm' in text, text)

    got = decap_repair.repair(model, movable, _legal_at(state))
    p = state.parts['C2']
    ok('the repair moves the cap home', got['before'] == 1 and got['after'] == 0
       and [m['ref'] for m in got['moved']] == ['C2'], got)
    ok('... to a pose inside the limit, measured from scratch',
       decap_repair.residual(model) == [], decap_repair.residual(model))
    ok('... not far past it: the nearest legal pose, not the pin itself',
       got['moved'] and 3.0 < got['moved'][0]['mm'] < 8.0, got['moved'])
    ok('the moved cap stands where the report says',
       [round(p.x, 4), round(p.y, 4), p.rot] == got['moved'][0]['to'], (p.x, p.y, p.rot))


def test_unreachable_is_named(td):
    """A cap whose every legal pose is outside the limit stays put and the
    row says how close the best legal pose got."""
    state, model, movable = _model(td)
    _put(state, model, 'C2', 121.0, 63.0)
    before = (state.parts['C2'].x, state.parts['C2'].y)
    got = decap_repair.repair(model, movable, lambda r, x, y, rot: False)
    ok('no legal pose: nothing moves, the term is stuck',
       got['moved'] == [] and got['after'] == 1 and len(got['stuck']) == 1
       and (state.parts['C2'].x, state.parts['C2'].y) == before, got)
    ok('... and has no pose to report a best distance from',
       got['stuck'][0]['reach'] is None, got['stuck'])
    # a pose that is legal but outside the limit: its distance is the 'reach'
    # (the rim of the disc the pin's lower bound allows: a pose there is
    # measured, and its gap is past the limit)
    (cx, cy), reach = decap_repair._target(model, 0, 'C2')
    got = decap_repair.repair(
        model, movable,
        lambda r, x, y, rot: (cx + reach - 0.25 < x < cx + reach and abs(y - cy) < 0.3))
    # only another chip's cap may move: the pose that frees IC2's pin would
    # put IC3's own pin past its limit, and the row says so
    got2 = decap_repair.repair(model, movable, lambda r, x, y, rot: r != 'C2')
    wb = got2['stuck'][0].get('would_break') if got2['stuck'] else None
    ok('a pose that frees the pin by breaking another limit is refused and named',
       got2['moved'] == [] and wb is not None and wb['kind'] == 'pin'
       and wb['ic'] != 'IC2', got2)
    ok('... in words', 'would put' in decap_repair.stuck_text(got2['stuck'][0]),
       decap_repair.stuck_text(got2['stuck'][0]))
    ok('a legal pose outside the limit is measured, kept out, and its distance named',
       got['moved'] == [] and got['stuck'][0]['reach'] is not None
       and got['stuck'][0]['reach'] > LIMIT, got['stuck'])


def test_command(td):
    intent, groups = _files(td)
    for name, extra in (('plain', []), ('repair', ['--repair-decaps'])):
        out = os.path.join(td, name + '_{seed}.kicad_pcb')
        check([sys.executable, '-X', 'utf8', tool('place_simple.py'), BOARD, out,
                   '--groups', groups, '--intent', intent, '--seeds', '0-1', '--moves', '150',
                   '--jobs', '1', '--json', os.path.join(td, name + '_{seed}.json')] + extra,
                  accept=True)
    for s in (0, 1):
        pj = os.path.join(td, f'plain_{s}.json')
        rj = os.path.join(td, f'repair_{s}.json')
        evidence(pj, 'the plain report')
        evidence(rj, 'the repair report')
        plain, rep = json.load(open(pj)), json.load(open(rj))
        ok(f'seed {s}: without the flag no repair runs, the residual is reported',
           plain['decap_repair'] is None and isinstance(plain['decap_residual'], list))
        ok(f'seed {s}: with it the repair runs and never leaves more broken',
           rep['decap_repair'] is not None
           and rep['decap_repair']['after'] <= rep['decap_repair']['before']
           and len(rep['decap_residual']) == rep['decap_repair']['after'], rep['decap_repair'])
        ok(f'seed {s}: and the grade agrees with the residual',
           (rep['floorplan']['errors'] or 0) == len(rep['decap_residual']),
           (rep['floorplan'], rep['decap_residual']))
    logs = {n: open(os.path.join(td, f'{n}_0.log'), encoding='utf-8').read()
            for n in ('plain', 'repair')}
    ok('the switch reaches each seed process: only the repair run prints a tally',
       'decap repair:' in logs['repair'] and 'decap repair:' not in logs['plain'])


def main():
    with tempfile.TemporaryDirectory(prefix='t1382_') as td:
        test_residual_and_repair(td)
        test_unreachable_is_named(td)
        test_command(td)
    if FAILS:
        print(f"FAILED: {len(FAILS)} check(s)")
        return 1
    print("all checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
