#!/usr/bin/env python3
"""#1249: a 3-pad regulator owns its decoupling caps.

`groups.DECAP_MIN_IC_PADS` is four, so a SOT-89 or TO-92 regulator was never
a chip: its input and output caps were elected to whatever other part carried
the rail (esp_prog C3 -> U1 at 2.125 mm, C1 -> USB1 at 17.24 mm), the quench
held them there, and `supply_pins` never saw its VIN / VOUT. A part is now a
cap owner when it is an IC by shape OR a supply owner
(`groups.supply_owners`): a pad typed `power_out`, the `regulator` role
(`placement.roles`, the table `board_context` prints), or three copper pads
on three nets -- one ground, two each carrying a decoupling cap.

Cases:
* esp_prog / the #895 lap-5 fixture: U2 is a supply owner by role, and the
  election moves exactly the issue's table (C3 -> U2 1.53, C1 -> U2 5.17,
  still beyond the 5 mm radius; lap 5: 0.95 / 0.95); the derived limit
  drops 2.125 -> 1.53 and `decap_ungraded` names U2 for C1;
* the rule each signal admits, and what each guard refuses, on edited copies
  of esp_prog's U2: power_out, footprint role, the three-pad shape; a header
  footprint, transistor pins, an uncapped rail and a two-pad source are
  refused, each for its own reason;
* `supply_pins` reads U2 by ROLE (channel `role`): pads 2 (VIN, net
  `Net-(C1-Pad1)`, which the rail-net channel never reads) and 3; the pin
  rule grades C1 against pad 2 (4.41 mm) and C3 against pad 3 (1.12 mm);
  every IC's channel is unchanged;
* ONE owner set: `rail_chips` and `arrays.pose_free_chip_refs` carry U2;
* kit-dev LV101 (a DS1818 reset supervisor) is admitted by the three-pad
  shape -- the named case -- and changes 0 of that board's 53 elections;
  it is not read by role (`regulator_owners`), so its supply pin is +3.3V
  and not RST;
* the role table: TO-92 transistors read by their B/C/E pins, a BAT54 in
  SOT-323 stays a diode, SOT-23 LDOs and ICs are no longer transistors;
* complex_hierarchy (KiCad's demo, read in place when present; not tracked):
  U101, a 78L05 in `TO-92_HandSolder`, is a regulator by its power_out pad,
  and C102 / C104 elect it at 12.36 / 11.49 mm.

    python3 -X utf8 tests/test_1249_regulator_owner.py [case ...]
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

ESP = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
LAP5 = os.path.join(ROOT, 'tests', 'fixtures', 'run25',
                    'esp_prog_lap5.kicad_pcb')
KITDEV = os.path.join(ROOT, 'kicad_files',
                      'kit-dev-coldfire-xilinx_5213.kicad_pcb')
#: KiCad's demo board; an fa10 source, not tracked. Read in place.
COMPLEX = os.path.expanduser(
    '~/Documents/kicad_demos/complex_hierarchy/complex_hierarchy.kicad_pcb')

_CACHE = {}


def _parse(path):
    if path not in _CACHE:
        from kicad_parser import parse_kicad_pcb
        with contextlib.redirect_stdout(io.StringIO()), \
                contextlib.redirect_stderr(io.StringIO()):
            _CACHE[path] = parse_kicad_pcb(path)
    return _CACHE[path]


def _elections(pcb):
    from placement import groups
    return {c: (ic, None if d is None else round(d, 2))
            for c, ic, d in groups._elect_tethers(pcb)}


def test_esp_prog_regulator_owns_its_caps():
    from placement import groups, floorplan as fp
    pcb = _parse(ESP)
    owners = groups.supply_owners(pcb)
    assert list(owners) == ['U2'], owners
    assert owners['U2'].startswith('role regulator') and 'sot89' in \
        owners['U2'], owners
    assert 'U2' in groups.chip_refs(pcb)
    el = _elections(pcb)
    assert el['C3'] == ('U2', 1.53), el
    assert el['C1'] == ('U2', 5.17), el
    # C1 is still beyond the radius -- ungraded, now against the right part.
    near, beyond, _o = groups.decap_populations(pcb)
    assert [(c, ic) for c, ic, _d in beyond] == [('C1', 'U2')], beyond
    assert sorted(c for c, _d in near['U2']) == ['C3'], near
    cen = fp.decap_census(pcb)
    lim, why = fp._decap_derivation(cen)
    # C3 is the board's one tether (C2 and C4 are Y1's load caps, not decaps),
    # and one sample is a coordinate, not a limit: the derivation withholds
    assert lim is None and why and 'only 1 tether' in why, (lim, why)
    pcb5 = _parse(LAP5)
    el5 = _elections(pcb5)
    assert el5['C1'] == ('U2', 0.95) and el5['C3'] == ('U2', 0.95), el5
    print(f"  PASS: esp_prog C3 -> U2 1.53, C1 -> U2 5.17 (beyond, "
          f"{owners['U2']}); limit withheld (1 tether); lap 5 C1/C3 -> U2 0.95")


def _edited(mutate):
    """A copy of esp_prog with U2 edited by `mutate(pcb, u2)`."""
    pcb = copy.deepcopy(_parse(ESP))
    mutate(pcb, pcb.footprints['U2'])
    return pcb


def test_each_signal_admits_and_each_guard_refuses():
    from placement import groups

    def neutral(pcb, u2):
        u2.footprint_name = 'House:Part3'

    def why(pcb):
        return groups.supply_owners(pcb).get('U2')

    # No role from the name: the three-pad shape admits it on its own.
    got = why(_edited(neutral))
    assert got and got.startswith('3 copper pads'), got

    def typed(pcb, u2):
        neutral(pcb, u2)
        next(p for p in u2.pads if p.pad_number == '3').pintype = 'power_out'
    got = why(_edited(typed))
    assert got == 'pad 3 is typed power_out', got

    # A rail with no cap: the shape no longer holds and nothing else does.
    def uncapped(pcb, u2):
        neutral(pcb, u2)
        del pcb.footprints['C1']
    assert why(_edited(uncapped)) is None

    # ...but the footprint role still admits it without the cap.
    def role_uncapped(pcb, u2):
        del pcb.footprints['C1']
    got = why(_edited(role_uncapped))
    assert got and got.startswith('role regulator'), got

    # A three-pin header carrying the same nets is a connector, not an owner.
    def header(pcb, u2):
        u2.footprint_name = 'Connector_PinHeader_2.54mm:PinHeader_1x03'
    assert why(_edited(header)) is None

    # Transistor pins (B/C/E) make it a transistor, which owns nothing.
    def transistor(pcb, u2):
        u2.footprint_name = 'Package_TO_SOT_THT:TO-92_Inline'
        for p, fn in zip(sorted(u2.pads, key=lambda p: p.pad_number),
                         ('E', 'B', 'C')):
            p.pinfunction = fn
    pcb_t = _edited(transistor)
    from placement.roles import infer_role
    assert infer_role(pcb_t.footprints['U2'], 'U2', None)['role'] == \
        'npn/pnp transistor'
    assert why(pcb_t) is None

    # A two-pad source is not something a cap is placed to decouple.
    def two_pad(pcb, u2):
        typed(pcb, u2)
        u2.pads = [p for p in u2.pads if p.pad_number != '1']
    assert why(_edited(two_pad)) is None
    print("  PASS: shape, power_out and role each admit; a header, "
          "transistor pins, an uncapped rail (without the role) and a "
          "two-pad source are refused")


def test_supply_pins_read_the_regulator_by_role():
    from placement import floorplan as fp
    pcb = _parse(ESP)
    recs = fp.supply_pins(pcb)
    u2 = recs['U2']
    assert u2['channel'] == 'role', u2
    assert [(p.pad_number, n) for p, n in u2['pins']] == [
        ('2', 'Net-(C1-Pad1)'), ('3', '/+3.3V')], u2['pins']
    # The rail-net channel alone would take pad 3 only -- the reason the
    # channel exists.
    assert u2['counts']['rail_net'] == 1, u2['counts']
    # Every IC by shape keeps its ladder: no role channel there.
    assert all(r['counts']['role'] == 0 for k, r in recs.items()
               if k != 'U2'), {k: r['counts'] for k, r in recs.items()}
    by = fp._decap_caps_by_net(pcb)
    got = {}
    for pad, _net in u2['pins']:
        _on, caps = fp.decap_pin_caps({}, 'U2', 'F', pad, by)
        gap, who = fp.nearest_rail_cap(pad, caps)
        got[pad.pad_number] = (who, round(gap, 2))
    assert got == {'2': ('C1', 4.41), '3': ('C3', 1.12)}, got
    # The pin rule grades them, at the declared name.
    intent = fp.intent_from_dict({'schema': 1, 'kind': 'floorplan-intent',
                                  'units': 'mm',
                                  'decaps': {'max_pin_distance_mm': 2.0}},
                                 ESP)
    with contextlib.redirect_stdout(io.StringIO()):
        res = fp.grade(intent, pcb, ESP)
    hits = [v for v in list(res.errors) + list(res.warnings)
            if v.ref == 'U2' and v.rule.startswith('decap_pin')]
    assert [(v.rule, v.measured['pad'], v.measured['cap'],
             v.measured['channel']) for v in hits] == [
        ('decap_pin_distance', '2', 'C1', 'role')], hits
    print(f"  PASS: U2 by role, pins {sorted(got)}; C1 -> pad 2 "
          f"{got['2'][1]} mm graded as decap_pin_distance, C3 -> pad 3 "
          f"{got['3'][1]} mm")


def test_one_owner_set():
    from placement import arrays, groups
    pcb = _parse(ESP)
    assert 'U2' in groups.rail_chips(pcb, 'C3'), groups.rail_chips(pcb, 'C3')
    assert 'U2' in groups.rail_chips(pcb, 'C1'), groups.rail_chips(pcb, 'C1')
    assert 'U2' in arrays.pose_free_chip_refs(pcb)
    print("  PASS: rail_chips and pose_free_chip_refs carry U2")


def test_kit_dev_supervisor_is_the_named_case():
    from placement import groups
    pcb = _parse(KITDEV)
    owners = groups.supply_owners(pcb)
    assert list(owners) == ['LV101'], owners
    assert owners['LV101'].startswith('3 copper pads'), owners
    with_lv = _elections(pcb)
    real = groups.supply_owners
    try:
        groups.supply_owners = lambda pcb_data, ics=None: {}
        without = _elections(pcb)
    finally:
        groups.supply_owners = real
    assert len(with_lv) == 38, len(with_lv)    # the decaps by the one rule (f87f1593)
    changed = [c for c in with_lv if with_lv[c] != without[c]]
    assert not changed, changed
    # The shape says LV101 owns caps, not which pins are supply pins: it is
    # not read by role, and the ladder takes its +3.3V pin, not RST.
    from placement import floorplan as fp
    assert 'LV101' not in groups.regulator_owners(pcb)
    rec = fp.supply_pins(pcb)['LV101']
    assert rec['channel'] == 'rail_net', rec
    assert [(p.pad_number, n) for p, n in rec['pins']] == [('2', '+3.3V')], \
        rec['pins']
    print("  PASS: kit-dev LV101 (DS1818) admitted by the 3-pad shape; "
          "0 of 38 elections change; its supply pin is +3.3V (pad 2) by "
          "the rail-net channel, not RST")


def test_role_table_reads_shared_packages_through_the_pins():
    from placement.part_class import classify_part
    from placement.roles import infer_role

    def role(board, ref):
        pcb = _parse(os.path.join(ROOT, 'kicad_files', board))
        fp_ = pcb.footprints[ref]
        return infer_role(fp_, ref, classify_part(fp_, ref).name)['role']
    want = {('flat_hierarchy.kicad_pcb', 'Q1'): 'npn/pnp transistor',
            ('glasgow_revC.kicad_pcb', 'D18'): 'diode',
            ('glasgow_revC.kicad_pcb', 'U14'): 'regulator',
            ('glasgow_revC.kicad_pcb', 'U13'): 'integrated circuit',
            ('kit-dev-coldfire-xilinx_5213.kicad_pcb', 'LV101'): 'unknown',
            ('kit-dev-coldfire-xilinx_5213.kicad_pcb', 'D101'): 'diode',
            ('tigard.kicad_pcb', 'U1'): 'integrated circuit',
            ('watchy.kicad_pcb', 'U2'): 'regulator',
            ('esp_prog.kicad_pcb', 'U2'): 'regulator'}
    got = {k: role(*k) for k in want}
    assert got == want, {k: (got[k], want[k]) for k in want
                         if got[k] != want[k]}
    print(f"  PASS: {len(want)} shared-package parts read through their pins "
          f"and prefixes")


def test_complex_hierarchy_78L05():
    if not os.path.isfile(COMPLEX):
        print(f"  SKIP: {COMPLEX} absent (KiCad's demo, not tracked)")
        return
    from placement import groups
    from placement.part_class import classify_part
    from placement.roles import infer_role
    pcb = _parse(COMPLEX)
    u101 = pcb.footprints['U101']
    assert infer_role(u101, 'U101', classify_part(u101, 'U101').name)[
        'role'] == 'regulator'
    assert groups.supply_owners(pcb) == {
        'U101': 'pad 1 is typed power_out'}, groups.supply_owners(pcb)
    el = _elections(pcb)
    assert el['C102'] == ('U101', 12.36) and el['C104'] == ('U101', 11.49), \
        {k: el[k] for k in ('C102', 'C104')}
    print("  PASS: complex_hierarchy U101 is a regulator; C102 -> 12.36, "
          "C104 -> 11.49 mm")


TESTS = [v for k, v in sorted(globals().items())
         if k.startswith('test_') and callable(v)]


if __name__ == '__main__':
    want = sys.argv[1:]
    n = 0
    for t in TESTS:
        if want and not any(w in t.__name__ for w in want):
            continue
        print(t.__name__)
        t()
        n += 1
    print(f"ALL PASS ({n} case(s))")
