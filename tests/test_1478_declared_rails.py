"""#1478: an intent's `rails[]` prices a supply as a rail whatever its fan-out.

place_simple prices a net as a rail only when its name says ground or it
reaches more than a quarter of the parts (and more than six), so on a large
board every supply pulls at a signal's full weight. `rails[]` declares the
supplies that are rails: the model prices them at `--rail-weight`, the
fan-out rule's own rails stay, and a pattern matching no net is warned.
"""

import contextlib
import io
import json
import os
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

BOARD = os.path.join(ROOT, 'tests', 'fixtures', '959', 'run29_pile.kicad_pcb')
FAILS = []


def ok(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}"
          + (f"  ({detail})" if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


def _model(td, rails):
    pcb = parse_kicad_pcb(BOARD)
    doc = fp.emit_intent(pcb, BOARD)
    if rails is not None:
        doc['rails'] = rails
        doc['min_reader'] = max(fp.RAILS_READER, doc.get('min_reader') or 0)
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
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        _state, model, _movable, notes = ps.build(
            pcb, BOARD, load_groups(groups), doc, args, intent, blocks, set())
    nid = {n.name: i for i, n in pcb.nets.items() if n.name}
    return model, notes, out.getvalue(), nid, args


def main():
    with tempfile.TemporaryDirectory() as td:
        base, _n, _o, nid, args = _model(td, None)
    v33 = nid['/+3.3V']
    ok('a 6-part supply on a 21-part board is priced as a signal',
       v33 in base.signal and v33 not in base.rnets)
    with tempfile.TemporaryDirectory() as td:
        model, notes, printed, nid, args = _model(
            td, [{'nets': ['/+3.3V', '/NOPE*'], 'why': 't'}])
    ok('declared, it is priced as a rail', v33 in model.rnets and v33 not in model.signal)
    ok('at --rail-weight', model.wire_weight(v33) == args.rail_weight, model.wire_weight(v33))
    gnd = nid['GND']
    ok('the fan-out rule keeps its own rails', gnd in model.rnets and gnd in base.rnets)
    ok('the declaration is named', any(n.startswith('rails declared: /+3.3V') for n in notes),
       notes)
    ok('a pattern matching no net is warned',
       "rails[0]: '/NOPE*' matches no net" in printed, printed[-300:])
    try:
        fp.intent_from_dict({'schema': 1, 'kind': fp.KIND, 'rails': [{'nets': []}]})
        ok('empty nets refused', False)
    except fp.IntentError as e:
        ok('empty nets refused', 'non-empty' in str(e), str(e))
    print('ALL PASS' if not FAILS else f'FAILED: {FAILS}')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
