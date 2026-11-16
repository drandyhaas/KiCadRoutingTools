#!/usr/bin/env python3
"""#1842: bga_fanout.py --via-span escapes a BGA on one sub-stack.

The fan-out laid through vias only, and its engines measure each via site
off the board with the barrel taken to span the stack: a pad on the far side
of the board under a ball blocked that ball's via. `--via-span TOP BOTTOM`
has route.py's meaning: the part escapes on that run of neighbouring copper
layers, its vias join exactly it, and for the call the board carries only
the copper the span has, so every engine's site test is the span's.

The fixture (tests/fixtures/1842/bga_under) is synthetic: the 8-layer board
of the route step's fixture with the bottom-side part moved so its pads sit
directly under the BGA's balls. 36 balls carry nets; the 12 inner ones need a
via.

Checks:
  1. with through vias the fan-out escapes 24 of 36 (no via site is legal
     over a bottom-side pad of another net);
  2. with `--via-span F.Cu In1.Cu` it escapes all 36, with 12 micro vias
     F.Cu-In1.Cu at the Default class's micro via size, copper on F.Cu and
     In1.Cu only, check_drc clean, and the project's via, drilled-hole,
     annular and copper-to-hole minimums left alone (the micro vias get a
     rule of their own: tests/test_1842_micro_via_rule.py). kicad-cli 10.0.8
     grades that board at 0 violations;
  3. the board is as it was after the call: `only_span_copper` puts back
     every pad, track, via and pour it set aside, in place;
  4. a part that is on neither end of the span is refused by name;
  5. `only_span_copper` on the board with tracks, vias and pours of every
     layer interleaved: each list comes back in its original order, with
     what the call removed gone and what it added after it, also when the
     call raises; a net whose KiCad-locked copper is all off the span stays
     locked inside the call (`locked_net_names`, `cached_protection_map`,
     `effective_protection_map`, `stub_layer_switching._locked_net_ids`); a
     memo built inside (the ground domains, from the pads) does not answer
     after it, and the copper epoch moves on the way in and out.
"""
import json
import os
import re
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)
from run_utils import check, evidence  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
FIX = os.path.join(TESTS, 'fixtures', '1842', 'bga_under')
KEYS = ('min_via_diameter', 'min_through_hole_diameter', 'min_microvia_diameter',
        'min_microvia_drill', 'min_via_annular_width', 'min_hole_clearance')
FAILS = []


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def fan(td, tag, *extra):
    board = os.path.join(td, tag + '_in.kicad_pcb')
    for ext in ('.kicad_pcb', '.kicad_pro'):
        shutil.copy(evidence(FIX + ext, 'the fixture'), os.path.splitext(board)[0] + ext)
    out = os.path.join(td, tag + '.kicad_pcb')
    r = check(PY + [os.path.join(REPO, 'py_router', 'bga_fanout.py'), board, '-o', out,
                    '--component', 'U1', '--clearance', '0.1', '--track-width', '0.1',
                    *extra], accept=True, timeout=1800)
    log = (r.stdout or '') + (r.stderr or '')
    m = re.findall(r'"escaped": (\d+), "failed": (\d+)', log)
    return out, log, (int(m[-1][0]), int(m[-1][1])) if m else None


def restored():
    print("3. the board after the call")
    from via_span import only_span_copper
    pcb = parse_kicad_pcb(FIX + '.kicad_pcb')

    def state():
        return ([id(s) for s in pcb.segments], [id(v) for v in pcb.vias],
                {r: [id(p) for p in fp.pads] for r, fp in pcb.footprints.items()},
                {n: [id(p) for p in pads] for n, pads in pcb.pads_by_net.items()})
    before = state()
    lists = (pcb.segments, pcb.vias, pcb.footprints['U2'].pads)
    with only_span_copper(pcb, ['F.Cu', 'In1.Cu']):
        ok(len(pcb.footprints['U2'].pads) == 0 and len(pcb.footprints['U1'].pads) == 100,
           "inside, the bottom-side part has no pads and the BGA has all of its own")
    ok(state() == before, "every pad, track and via is back, in its place")
    ok(lists[0] is pcb.segments and lists[1] is pcb.vias
       and lists[2] is pcb.footprints['U2'].pads, "and the lists are the same objects")


def set_aside_rows():
    print("5. the set-aside on a board with copper on every layer")
    from types import SimpleNamespace
    from kicad_parser import Segment, Via
    from via_span import only_span_copper
    from protected_nets import (cached_protection_map, effective_protection_map,
                                locked_net_names)
    from stub_layer_switching import _locked_net_ids
    from net_queries import resolve_ground_domains
    span = ['F.Cu', 'In1.Cu']
    pcb = parse_kicad_pcb(FIX + '.kicad_pcb')
    ids = {n.name: nid for nid, n in pcb.nets.items()}
    layers = ['F.Cu', 'B.Cu', 'In1.Cu', 'In3.Cu']
    for i in range(8):
        pcb.segments.append(Segment(start_x=1 + i, start_y=1, end_x=1 + i, end_y=2, width=0.1,
                                    layer=layers[i % 4], net_id=ids['S%02d' % (i + 2)]))
    for i, (pair, kind) in enumerate([(['F.Cu', 'B.Cu'], 'through'),
                                      (['In3.Cu', 'In4.Cu'], 'buried'),
                                      (['F.Cu', 'In1.Cu'], 'micro'),
                                      (['In6.Cu', 'B.Cu'], 'micro')]):
        pcb.vias.append(Via(x=1 + i, y=3, size=0.4, drill=0.2, layers=pair,
                            net_id=ids['S%02d' % (i + 2)], via_type=kind))
    pcb.zones[:] = [SimpleNamespace(layer=l) for l in ('B.Cu', 'F.Cu', 'In2.Cu', 'In1.Cu')]
    # S01: an unlocked track on the span and a locked one off it
    pcb.segments.append(Segment(start_x=20, start_y=1, end_x=21, end_y=1, width=0.1,
                                layer='F.Cu', net_id=ids['S01']))
    pcb.segments.append(Segment(start_x=20, start_y=2, end_x=21, end_y=2, width=0.1,
                                layer='B.Cu', net_id=ids['S01'], locked=True))
    lists = {'segments': pcb.segments, 'vias': pcb.vias, 'zones': pcb.zones}
    before = {k: list(v) for k, v in lists.items()}
    pads_before = {r: list(fp.pads) for r, fp in pcb.footprints.items()}
    ok(resolve_ground_domains(pcb).get(ids['GND']) == {'U2'},
       "outside, U2 is the ground domain")
    epoch0 = getattr(pcb, '_copper_epoch', 0)
    added = Segment(start_x=5, start_y=5, end_x=6, end_y=5, width=0.1, layer='In1.Cu',
                    net_id=ids['S03'])
    with only_span_copper(pcb, span, 'micro'):
        epoch1 = getattr(pcb, '_copper_epoch', 0)
        visible = [s for s in pcb.segments if s.layer in span]
        ok(pcb.segments == visible and all(z.layer in span for z in pcb.zones)
           and [v.via_type for v in pcb.vias] == ['through', 'micro'],
           "inside: only the span's tracks and pours, and the vias with copper on it")
        name = 'S01'
        ok(name in locked_net_names(pcb)
           and cached_protection_map(pcb).get(name) == 'locked'
           and effective_protection_map(pcb).get(name) == 'locked'
           and ids[name] in _locked_net_ids(pcb),
           "inside: S01, whose locked copper is all off the span, is locked")
        ok(resolve_ground_domains(pcb).get(ids['GND']) == set(),
           "inside: U2's pads are set aside, so no part returns to GND")
        removed = pcb.segments[0]
        pcb.segments.remove(removed)
        pcb.segments.append(added)
    epoch2 = getattr(pcb, '_copper_epoch', 0)
    want = [x for x in before['segments'] if x is not removed] + [added]
    ok([id(x) for x in pcb.segments] == [id(x) for x in want],
       "after: the tracks in their original order, the removed one gone, the added one last")
    ok(all([id(x) for x in lists[k]] == [id(x) for x in before[k]] for k in ('vias', 'zones'))
       and {r: [id(p) for p in fp.pads] for r, fp in pcb.footprints.items()}
       == {r: [id(p) for p in v] for r, v in pads_before.items()},
       "and the vias, pours and pads in their original order")
    ok(all(lists[k] is getattr(pcb, k) for k in lists), "the lists are the same objects")
    ok(resolve_ground_domains(pcb).get(ids['GND']) == {'U2'},
       "after: the ground domains are the board's again (the inside memo is dropped)")
    ok(epoch0 < epoch1 < epoch2, f"the copper epoch moves in and out: {epoch0, epoch1, epoch2}")
    ok('_span_set_aside' not in vars(pcb), "and nothing is left set aside")
    state = [id(x) for x in pcb.segments], [id(x) for x in pcb.vias]
    try:
        with only_span_copper(pcb, span, 'micro'):
            pcb.vias.clear()
            raise RuntimeError('boom')
    except RuntimeError:
        pass
    ok(([id(x) for x in pcb.segments], [id(x) for x in pcb.vias])
       == (state[0], [id(v) for v in before['vias'] if v.layers not in (['F.Cu', 'B.Cu'],
                                                                       ['F.Cu', 'In1.Cu'])]),
       "a call that raises: the tracks are back, the set-aside vias are back, and "
       "the ones it removed stay removed")


def main():
    with open(FIX + '.kicad_pro', encoding='utf-8') as fh:
        own = json.load(fh)['board']['design_settings']['rules']
    with tempfile.TemporaryDirectory() as td:
        print("1. through vias")
        out, _log, got = fan(td, 'through', '--layers', 'F.Cu', 'In1.Cu')
        pcb = parse_kicad_pcb(evidence(out, 'the fanned board'))
        ok(got == (24, 12) and not pcb.vias,
           f"24 of 36 balls escape and no via is laid: {got}, {len(pcb.vias)} vias")

        print("2. a micro via span")
        out, log, got = fan(td, 'span', '--via-span', 'F.Cu', 'In1.Cu')
        pcb = parse_kicad_pcb(evidence(out, 'the fanned board'))
        ok(got == (36, 0), f"all 36 balls escape: {got}")
        kinds = sorted({(v.via_type, tuple(v.layers), round(v.size, 4), round(v.drill, 4))
                        for v in pcb.vias})
        ok(kinds == [('micro', ('F.Cu', 'In1.Cu'), 0.25, 0.1)] and len(pcb.vias) == 12,
           f"with 12 micro vias at the class's micro via size: {kinds} x{len(pcb.vias)}")
        ok(sorted({s.layer for s in pcb.segments}) == ['F.Cu', 'In1.Cu'],
           f"copper on the span only: {sorted({s.layer for s in pcb.segments})}")
        d = check(PY + [os.path.join(REPO, 'py_router', 'check_drc.py'), out,
                        '--clearance-margin-mm', '0.0005'], accept=True)
        ok('NO DRC VIOLATIONS FOUND' in (d.stdout or ''), "check_drc is clean")
        with open(os.path.splitext(out)[0] + '.kicad_pro', encoding='utf-8') as fh:
            rules = json.load(fh)['board']['design_settings']['rules']
        ok(all(rules.get(k) == own.get(k) for k in KEYS),
           f"the project's via and hole minimums are the board's own: "
           f"{ {k: rules.get(k) for k in KEYS} }")
        ok('Via span: 12 fan-out via(s) written as micro F.Cu-In1.Cu' in log,
           "and the run says what it wrote")

        restored()
        set_aside_rows()

        print("4. a part on neither end of the span")
        board = os.path.join(td, 'bad_in.kicad_pcb')
        for ext in ('.kicad_pcb', '.kicad_pro'):
            shutil.copy(FIX + ext, os.path.splitext(board)[0] + ext)
        check(PY + [os.path.join(REPO, 'py_router', 'bga_fanout.py'), board,
                    '-o', os.path.join(td, 'bad.kicad_pcb'), '--component', 'U1',
                    '--via-span', 'In1.Cu', 'In6.Cu'],
              refuse='which is not an end of the span')
        ok(True, "is refused by name")
    if FAILS:
        print(f"\nFAILED: {len(FAILS)}")
        return 1
    print("\nALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
