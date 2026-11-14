#!/usr/bin/env python3
"""#595, the cost: a paying net is routed round a gap in its reference plane.

`reference_void.py` prices signal-layer cells that have no pour copper on any
adjacent poured layer, for the nets that pay (impedance-declared nets, the
pairs of a diff-pair step, the nets KICAD_REFERENCE_VOID_NETS names). It is
off unless KICAD_REFERENCE_VOID_COST is above 0.

The fixture (tests/fixtures/595/slot) is synthetic: a 4-layer board with a
GND pour on In1.Cu in three zones that leave a slot 1 mm wide from y 1 to
y 7.5 at x 15, and two F.Cu nets with a pad either side of it: SIG at y 4
and SLOW at y 2.5. The straight route of each crosses the slot.

Rows:
  1. the void map: the slot and the band past the pours' edge are void, the
     pour is not, and a hole narrower than the minimum is closed;
  2. who pays: the nets the environment names and a step's extra payers, and
     the field is left out of the cost sources of a net that does not pay;
  3. the smoothing guard: a connector over more void than the copper it
     replaces is refused for a payer and for no other net;
  4. routed with the cost off, both nets cross the slot (the measurement of
     #595's first half reads 2 crossings) and nothing names the field;
  5. routed with the cost on for SIG, SIG does not cross and SLOW still
     does, both nets are routed, check_drc is clean, and the summary carries
     the field's record;
  6. the diff-pair step registers it too.
"""
import json
import os
import shutil
import sys
import tempfile
from types import SimpleNamespace

import numpy as np

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)
from run_utils import check, evidence  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
FIX = os.path.join(TESTS, 'fixtures', '595')
FAILS = []


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def state(layer, gx0, gy0, mask):
    h, w = mask.shape
    return SimpleNamespace(layer=layer, gx0=gx0, gy0=gy0, W=w, H=h, mask=mask)


def the_map():
    print("1. the void map")
    import reference_void as rv
    left = np.ones((100, 135), dtype=bool)          # x 1.0..14.4, y 1.0..10.9
    left[40:42, 50:52] = False                      # a 0.2 mm hole
    right = np.ones((100, 135), dtype=bool)         # x 15.5..28.9
    sts = [state('In1.Cu', 10, 10, left), state('In1.Cu', 155, 10, right)]
    masks = rv.void_masks(sts, ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu'], ['F.Cu', 'B.Cu'],
                          0.1, 0.5, bounds=(0.0, 0.0, 30.0, 12.0))
    ok(sorted(masks) == ['F.Cu'], f"F.Cu has a poured neighbour, B.Cu has none: {sorted(masks)}")
    gx0, gy0, void = masks['F.Cu']

    def at(x, y):
        return bool(void[int(round(y / 0.1)) - gy0, int(round(x / 0.1)) - gx0])
    ok(at(15.0, 4.0), "the slot between the two pours is void")
    ok(not at(8.0, 4.0) and not at(20.0, 4.0), "the pours are not")
    ok(at(15.0, 0.4) and at(0.3, 6.0), "the band past the pours' edge, to the board's, is void")
    ok(not at(6.05, 5.05), "a 0.2 mm hole in a pour is closed (under the 0.5 mm minimum)")
    open_ = rv.void_masks(sts, ['F.Cu', 'In1.Cu'], ['F.Cu'], 0.1, 0.0)['F.Cu']
    ok(bool(open_[2][int(round(5.05 / 0.1)) - open_[1], int(round(6.05 / 0.1)) - open_[0]]),
       "and stays a void with the minimum at 0")
    rows = rv.void_rows(masks, ['F.Cu', 'B.Cu'], 777)
    ok(len(rows) == int(void.sum()) and set(rows[:, 0]) == {0} and set(rows[:, 3]) == {777},
       "the rows are those cells on the layer's index at the cell cost")


def the_payers():
    print("2. who pays")
    import reference_void as rv
    from routing_context import _per_net_cost_sources
    pcb = SimpleNamespace(nets={1: SimpleNamespace(name='SIG'), 2: SimpleNamespace(name='SLOW'),
                                3: SimpleNamespace(name='/usb/D_P')}, source_path='')
    cfg = SimpleNamespace(net_layer_widths={})
    os.environ['KICAD_REFERENCE_VOID_NETS'] = 'SIG, /usb/*'
    try:
        got = rv.payer_net_ids(pcb, cfg, extra=[9])
    finally:
        del os.environ['KICAD_REFERENCE_VOID_NETS']
    ok(got == {1, 3, 9}, f"the nets the globs name and the step's extra payers: {sorted(got)}")
    ok(rv.payer_net_ids(pcb, SimpleNamespace(net_layer_widths={2: {}})) == {2},
       "a net with an impedance width of this step")
    rows = np.array([[0, 1, 1, 5]], dtype=np.int32)
    cache = {rv.REFVOID_CACHE_KEY: rows}
    rv._PAYERS[id(cache)] = (cache, frozenset({1}))
    ok(rv.without_reference_void(cache, cache, (1,)) is cache, "a payer keeps the field")
    ok(rv.REFVOID_CACHE_KEY not in rv.without_reference_void(cache, cache, (2,)),
       "a net that does not pay is priced without it")
    ok(rv.REFVOID_CACHE_KEY in _per_net_cost_sources(cache, (1,))
       and rv.REFVOID_CACHE_KEY not in _per_net_cost_sources(cache, (2,)),
       "and the router's own cost sources are filtered so")
    plain = {5: rows}
    ok(rv.without_reference_void(plain, plain, (2,)) is plain, "a run without the field is untouched")


def the_guard():
    print("3. the smoothing guard")
    import reference_void as rv
    void = np.zeros((120, 300), dtype=bool)
    void[10:75, 145:155] = True                     # the slot, x 14.5..15.4, y 1.0..7.4
    vm = rv.VoidMap({'F.Cu': (0, 0, void)}, 0.1, {1})
    round_ = [(10.0, 4.0), (13.0, 8.5), (17.0, 8.5), (20.0, 4.0)]
    straight = [(10.0, 4.0), (20.0, 4.0)]
    ok(abs(vm.void_mm('F.Cu', straight) - 1.0) < 0.06 and vm.void_mm('F.Cu', round_) == 0.0,
       "the straight path lies 1 mm over the slot, the path round it none")
    ok(vm.worse(1, 'F.Cu', round_, straight), "a payer's detour is not straightened across it")
    ok(not vm.worse(2, 'F.Cu', round_, straight), "another net's is")
    ok(not vm.worse(1, 'F.Cu', straight, straight) and not vm.worse(1, 'B.Cu', round_, straight),
       "a connector over no more void than it replaces, or on a layer with no map, passes")
    with open(os.path.join(REPO, 'py_router', 'pcb_modification.py'), encoding='utf-8') as fh:
        ok('_rv595.worse(' in fh.read(), "smooth_octolinear_chains asks it")


def route(td, tag, env):
    sub = os.path.join(td, tag)
    os.makedirs(sub)
    for ext in ('.kicad_pcb', '.kicad_pro'):
        shutil.copy(evidence(os.path.join(FIX, 'slot' + ext), 'the fixture'),
                    os.path.join(sub, 'slot' + ext))
    out, summary = os.path.join(sub, 'out.kicad_pcb'), os.path.join(sub, 'out.json')
    e = {k: v for k, v in os.environ.items() if not k.startswith('KICAD_REFERENCE_VOID')}
    e.update(env)
    r = check(PY + [os.path.join(REPO, 'py_router', 'route.py'),
                    os.path.join(sub, 'slot.kicad_pcb'), out, '--nets', 'SIG', 'SLOW',
                    '--layers', 'F.Cu', '--json-out', summary], accept=True, timeout=900, env=e)
    with open(evidence(summary, 'the summary'), encoding='utf-8') as fh:
        doc = json.load(fh)
    rep = os.path.join(sub, 'ref.json')
    check(PY + [os.path.join(REPO, 'py_tools', 'check_reference_plane.py'),
                evidence(out, 'the routed board'), '--json', rep], accept=True)
    with open(evidence(rep, 'the reference-plane report'), encoding='utf-8') as fh:
        ref = json.load(fh)
    return out, doc, ref, (r.stdout or '')


def the_routes(td):
    print("4. routed with the cost off")
    _o, doc, ref, log = route(td, 'off', {})
    by = ref['by_net']
    ok(sorted(doc.get('routed_single') or []) == ['SIG', 'SLOW'], "both nets are routed")
    ok(by['SIG']['crossings'] == 1 and by['SLOW']['crossings'] == 1,
       f"each crosses the slot once: {by['SIG']['crossings']}, {by['SLOW']['crossings']}")
    ok('Reference-void field' not in log and 'reference_void' not in doc,
       "and the run names no such field")
    print("5. routed with the cost on for SIG")
    out, doc, ref, log = route(td, 'on', {'KICAD_REFERENCE_VOID_COST': '3',
                                          'KICAD_REFERENCE_VOID_NETS': 'SIG'})
    by = ref['by_net']
    ok(sorted(doc.get('routed_single') or []) == ['SIG', 'SLOW']
       and not (doc.get('failed_single') or []), "both nets are still routed")
    ok(by['SIG']['crossings'] == 0 and by['SIG']['void_mm'] == 0.0,
       f"SIG is over copper all the way: {by['SIG']}")
    ok(by['SLOW']['crossings'] == 1, "SLOW, which does not pay, crosses as before")
    rv = doc.get('reference_void') or {}
    ok(rv.get('cost_mm') == 3.0 and rv.get('cells', 0) > 0 and rv.get('payers') == 1
       and 'Reference-void field:' in log and '1 net(s) pay it' in log,
       f"the run says what it priced: {rv}")
    g = check(PY + [os.path.join(REPO, 'py_router', 'check_drc.py'), out], accept=True)
    ok('NO DRC VIOLATIONS FOUND' in (g.stdout or ''), "check_drc is clean")


def the_pair_step():
    print("6. the diff-pair step")
    with open(os.path.join(REPO, 'py_router', 'route_diff.py'), encoding='utf-8') as fh:
        src = fh.read()
    ok('register_reference_void(' in src and 'extra_payers=' in src
       and "summary['reference_void']" in src,
       "route_diff registers the field with its pairs as payers and discloses it")


def main():
    the_map()
    the_payers()
    the_guard()
    with tempfile.TemporaryDirectory() as td:
        the_routes(td)
    the_pair_step()
    if FAILS:
        print(f"\nFAILED ({len(FAILS)}):")
        for f in FAILS:
            print(f"  - {f}")
        return 1
    print("\nALL PASSED")
    return 0


if __name__ == '__main__':
    sys.exit(main())
