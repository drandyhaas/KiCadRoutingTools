#!/usr/bin/env python3
"""#1605: an active oscillator whose pads carry no pin metadata is not a
passive crystal, so a supply rail's decaps never become its load caps.

`emc_roles._crystals` read a crystal-role part as a PASSIVE crystal when it
had two signal pads on two nets and no output, and `_signal_pads` set a
supply pin aside only by its pin type or function. fomu's U7 (4-pin
oscillator: pad 1 dangling, 2 GND, 3 /OSC_IN, 4 +3V3, no pin metadata) kept
`+3V3` as a "load net", and `crystals.crystal_claims` then claimed every
two-pad cap from `+3V3` to ground as its load cap -- 12 `proximity[]` rows.

The fix reads a supply pin off its NET as well (`emc_roles._rail`: the
decap pin rule's rail-shaped name AND a decoupling cap to ground, a digit-led
name counting only when voltage-shaped), and a part with a supply pin is
never passive. fomu is not in the repo; ulx3s Y1 is the same part (4-pin
oscillator, no pin metadata, pads on +3V3 / GND / CLK_25MHz / +3V3), so
leaving its pad 1 dangling, as fomu's is, makes the fomu shape on a tracked
board.

Cases:
* the fomu shape (ulx3s Y1, pad 1 dangling): not a crystal, no claim row, no
  +3V3 cap claimed; read as a crystal-or-oscillator candidate whose evidence
  names pad 4 a supply pin, and the keep-away proposal still protects
  CLK_25MHz;
* the same with pin metadata that says nothing about supply (pad 4 typed
  `passive`, function `4`): a load net is never a rail, so still no claim;
* the trap stays shut: watchy Y1 on `/32K_P` / `/32K_N`, rail-shaped to
  `is_power_net_name` and decoupled by its load caps, is still a crystal
  with load caps C17/C18 -- with its pin metadata stripped too;
* tigard Y1 (no pin metadata, a real passive crystal) keeps its claims.

    python3 -X utf8 tests/test_1605_oscillator_supply_pin.py [case ...]
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

BOARDS = os.path.join(ROOT, 'kicad_files')
ULX = os.path.join(BOARDS, 'ulx3s.kicad_pcb')
WATCHY = os.path.join(BOARDS, 'watchy.kicad_pcb')
TIGARD = os.path.join(BOARDS, 'tigard.kicad_pcb')
_CACHE = {}


def _parse(path):
    if path not in _CACHE:
        run_utils.evidence(path, 'the board')
        from kicad_parser import parse_kicad_pcb
        with contextlib.redirect_stdout(io.StringIO()), \
                contextlib.redirect_stderr(io.StringIO()):
            _CACHE[path] = parse_kicad_pcb(path)
    return _CACHE[path]


def _quiet(fn, *a):
    with contextlib.redirect_stdout(io.StringIO()), \
            contextlib.redirect_stderr(io.StringIO()):
        return fn(*a)


def _roles(pcb):
    from placement.emc_roles import emc_roles
    return _quiet(emc_roles, pcb)


def _claims(pcb):
    from placement.crystals import crystal_claims
    return _quiet(crystal_claims, pcb)


def _net_id(pcb, name):
    return next(nid for nid, n in pcb.nets.items() if n.name == name)


def _fomu_shape(pin_metadata=False):
    """ulx3s with Y1's pad 1 (+3V3) on a net of its own, as fomu U7's pad 1
    reaches no other part: two signal pads were left, CLK_25MHz and +3V3."""
    from kicad_parser import Net
    pcb = copy.deepcopy(_parse(ULX))
    y1 = pcb.footprints['Y1']
    nid = max(pcb.nets) + 1
    pcb.nets[nid] = Net(net_id=nid, name='Net-(Y1-Pad1)')
    p1 = next(p for p in y1.pads if p.pad_number == '1')
    p1.net_id, p1.net_name = nid, 'Net-(Y1-Pad1)'
    if pin_metadata:
        p4 = next(p for p in y1.pads if p.pad_number == '4')
        p4.pintype, p4.pinfunction = 'passive', '4'
    return pcb


def _rail_caps(pcb, rail):
    """The two-pad caps from `rail` to ground: what the bug claimed."""
    from net_queries import is_ground_net_name
    rid = _net_id(pcb, rail)
    out = []
    for r, fp in pcb.footprints.items():
        nets = {p.net_id for p in fp.pads if p.net_id}
        if r.startswith('C') and len(nets) == 2 and rid in nets and any(
                is_ground_net_name(pcb.nets[n].name) for n in nets - {rid}):
            out.append(r)
    return out


def test_the_fomu_shape_is_not_a_crystal():
    pcb = _fomu_shape()
    caps = _rail_caps(pcb, '+3V3')
    assert len(caps) >= 10, ('the case is not armed: +3V3 has too few '
                             'decaps to be claimed', caps)
    rows, notes = _claims(pcb)
    assert not [r for r in rows if 'Y1' in (r['ref'], r['near'])], rows
    assert not [n for n in notes if n.startswith('crystal Y1 ')], notes
    claimed = {r['ref'] for r in rows}
    assert not claimed & set(caps), sorted(claimed & set(caps))
    y1 = [r for r in _roles(pcb)['roles'] if r['part'] == 'Y1']
    assert [(r['role'], r.get('candidate')) for r in y1] == [
        ('unknown', 'crystal or oscillator')], y1
    ev = ' | '.join(y1[0]['evidence'])
    assert 'Y1 pad 4 is a supply pin: on +3V3, a rail' in ev, ev
    assert '1 signal pad(s) on 1 non-ground net(s) and a supply pin' in ev, ev
    sec = _roles(pcb)
    clocks = [k for k in sec.get('keep_away') or ()
              if 'CLK_25MHz' in str(k.get('rule', ''))]
    assert clocks, [k.get('rule') for k in sec.get('keep_away') or ()]
    print(f"  PASS: Y1 (pad 1 dangling, as fomu U7) is a crystal-or-"
          f"oscillator candidate, none of the {len(caps)} +3V3 decaps is "
          f"claimed, and CLK_25MHz is still kept away")


def test_metadata_that_says_nothing_still_keeps_a_rail_off_the_load():
    pcb = _fomu_shape(pin_metadata=True)
    rows, notes = _claims(pcb)
    assert not [r for r in rows if 'Y1' in (r['ref'], r['near'])], rows
    y1 = [r for r in _roles(pcb)['roles'] if r['part'] == 'Y1']
    assert y1 and y1[0]['role'] == 'unknown', y1
    print("  PASS: pad 4 typed passive, function '4', on +3V3: still no "
          "crystal claim (a load net is never a rail)")


def _watchy_y1(pcb):
    rows, notes = _claims(pcb)
    note = next((n for n in notes if n.startswith('crystal Y1 ')), None)
    assert note is not None, notes
    caps = {r['ref'] for r in rows if r['near'] == 'Y1'}
    return note, caps


def test_digit_led_crystal_nets_stay_load_nets():
    from net_queries import is_power_net_name
    assert is_power_net_name('/32K_P'), 'the trap is not armed'
    pcb = _parse(WATCHY)
    note, caps = _watchy_y1(pcb)
    assert caps == {'C17', 'C18'}, (note, caps)
    bare = copy.deepcopy(pcb)
    for p in bare.footprints['Y1'].pads:
        p.pintype, p.pinfunction = '', ''
    note2, caps2 = _watchy_y1(bare)
    assert (note2, caps2) == (note, caps), (note2, caps2)
    print(f"  PASS: watchy Y1 on /32K_P, /32K_N is a crystal with load caps "
          f"{sorted(caps)}, with or without its pin metadata")


def test_a_passive_crystal_without_metadata_keeps_its_claims():
    rows, notes = _claims(_parse(TIGARD))
    note = next((n for n in notes if n.startswith('crystal Y1 ')), None)
    assert note and note.endswith('load caps C23, C24'), notes
    print(f"  PASS: tigard {note}")


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
