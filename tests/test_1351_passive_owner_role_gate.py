#!/usr/bin/env python3
"""#1351: a resistor array, an RGB LED or a diode is not "the IC a cap
decouples".

`groups._ic_list` admitted any part with four or more copper pads not in a
row, so on rp2350 the 10k pull-up arrays RN1/RN2 (`R_Array_Convex_4x0402`)
and the RGB LED D1 owned the +3V3 decaps, and the grade reported "C25 is
2.54mm from RN1, the IC it decouples" while C22 graded clean at 0.97 mm from
RN1 with the nearest real IC 4.85 mm away. `groups.passive_by_role` now
drops a part `infer_role` calls a resistor, diode or LED, or a resistor
network, unless a pin of it is a supply pin by its symbol or its name says
it is an addressable LED.

Cases (pose-free: owner SETS, never an election, which reads poses):
* rp2350: RN1, RN2 and D1 leave `chip_refs` and every +3V3 cap's
  `rail_chips`; U2 stays; `arrays.pose_free_chip_refs` agrees;
* the exceptions, on edited copies of rp2350's D1: a pad on +3V3 typed
  `power_in`, a pin named `VDD`, a value naming an addressable LED
  (`WS2812B`) each keep it an owner; a `power_in` pad on GND does not;
* tigard and orangecrab's RN arrays and orangecrab's RGB LEDs drop, and no
  `U` part of any tracked board does.

    python3 -X utf8 tests/test_1351_passive_owner_role_gate.py [case ...]
"""
import contextlib
import copy
import io
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import run_utils  # noqa: E402

ROOT = run_utils.ROOT_DIR
for _sub in ('py_router', 'py_tools', 'py_placer'):
    _p = os.path.join(ROOT, _sub)
    if _p not in sys.path:
        sys.path.insert(0, _p)

RUN_ALL_TIMEOUT = 600

RP = os.path.join(ROOT, 'kicad_files', 'rp2350_fpga_eensy_prePlane.kicad_pcb')

_CACHE = {}


def _parse(path):
    if path not in _CACHE:
        from kicad_parser import parse_kicad_pcb
        run_utils.evidence(path, 'tracked board')
        with contextlib.redirect_stdout(io.StringIO()), \
                contextlib.redirect_stderr(io.StringIO()):
            _CACHE[path] = parse_kicad_pcb(path)
    return _CACHE[path]


def _net_id(pcb, name):
    return next(nid for nid, n in pcb.nets.items() if n.name == name)


def test_rp2350_arrays_and_led_own_nothing():
    from placement import groups, arrays
    pcb = _parse(RP)
    chips = groups.chip_refs(pcb)
    for ref in ('RN1', 'RN2', 'D1'):
        assert ref not in chips, (ref, sorted(chips))
        assert groups.passive_by_role(pcb, ref), ref
    assert 'U2' in chips and groups.passive_by_role(pcb, 'U2') is None
    assert not {'RN1', 'RN2', 'D1'} & arrays.pose_free_chip_refs(pcb)
    v33 = _net_id(pcb, '+3V3')
    caps = [r for r, fp in pcb.footprints.items()
            if groups.is_decoupling_cap(fp, r)
            and v33 in {p.net_id for p in fp.pads}]
    assert caps, 'no +3V3 decoupling cap on rp2350'
    for c in caps:
        own = set(groups.rail_chips(pcb, c))
        assert not own & {'RN1', 'RN2', 'D1'}, (c, own)
    print(f"  PASS: rp2350 RN1/RN2/D1 own none of {len(caps)} +3V3 caps "
          f"({groups.passive_by_role(pcb, 'RN1')}; "
          f"{groups.passive_by_role(pcb, 'D1')})")


def _d1_edited(mutate):
    pcb = copy.deepcopy(_parse(RP))
    mutate(pcb, pcb.footprints['D1'])
    return pcb


def test_powered_led_stays_an_owner():
    from placement import groups
    base = _parse(RP)
    v33, gnd = _net_id(base, '+3V3'), _net_id(base, 'GND')

    def typed_rail(pcb, d1):
        next(p for p in d1.pads if p.net_id == v33).pintype = 'power_in'

    def named_rail(pcb, d1):
        next(p for p in d1.pads if p.net_id == v33).pinfunction = 'VDD'

    def family(pcb, d1):
        d1.value = 'WS2812B'

    def typed_ground(pcb, d1):
        p = next(p for p in d1.pads if p.net_id != v33)
        p.net_id, p.pintype = gnd, 'power_in'

    for name, fn in (('typed_rail', typed_rail), ('named_rail', named_rail),
                     ('family', family)):
        pcb = _d1_edited(fn)
        assert groups.passive_by_role(pcb, 'D1') is None, name
        assert 'D1' in groups.chip_refs(pcb), name
    pcb = _d1_edited(typed_ground)
    assert groups.passive_by_role(pcb, 'D1'), 'a GND power_in pin admitted D1'
    assert 'D1' not in groups.chip_refs(pcb)
    print("  PASS: D1 stays an owner with a power_in / VDD pin on +3V3 or "
          "a WS2812B value; a power_in pin on GND does not admit it")


def test_tracked_boards():
    from placement import groups
    from placement.roles import infer_role
    boards = run_utils.corpus_boards()
    if not boards:
        print("  SKIP: git cannot list the tracked boards")
        return
    dropped = {}
    for b in boards:
        pcb = _parse(b)
        for ref, fp in pcb.footprints.items():
            if groups._copper_pads(fp) < groups.DECAP_MIN_IC_PADS:
                continue
            why = groups.passive_by_role(pcb, ref)
            if why:
                dropped.setdefault(os.path.basename(b), set()).add(ref)
                assert infer_role(fp, ref, None)['role'] != 'integrated circuit', \
                    (b, ref, why)
    tig = dropped.get('tigard.kicad_pcb', set())
    oc = dropped.get('orangecrab_ext_pll.kicad_pcb', set())
    assert {'RN1', 'RN2'} <= tig, tig
    assert {'RN1', 'D1', 'D2'} <= oc, oc
    print(f"  PASS: {sum(map(len, dropped.values()))} passive parts dropped on "
          f"{len(dropped)} of {len(boards)} tracked boards, no IC among them")


CASES = [test_rp2350_arrays_and_led_own_nothing, test_powered_led_stays_an_owner,
         test_tracked_boards]

if __name__ == '__main__':
    want = set(sys.argv[1:])
    for t in CASES:
        if not want or t.__name__ in want:
            print(t.__name__)
            t()
    print('ALL PASS')
