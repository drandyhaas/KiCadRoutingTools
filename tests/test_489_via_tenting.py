#!/usr/bin/env python3
"""#489 §8: via tenting/plugging/filling must round-trip, not be re-stamped.

Via had no field for the protection family, the parser explicitly skipped those
tokens, and generate_via_sexpr hardcoded `(tenting (front yes) (back yes))` on
every via it emitted. So any via that was ripped and RE-PLACED (rip-up/reroute,
sub-grid via nudge, tap relocation) silently lost its real spec -- which matters
most for via-in-pad, where IPC-4761 Type VII filled+capped+plated is what keeps
solder out of the barrel.

Covers:
  * the parser reads all five tokens off a real multi-line via block
  * the writer re-emits a via's own spec verbatim (whitespace normalised)
  * a via with NO spec emits NO protection token in EITHER dialect, so it
    inherits the board's `(setup ...)` policy -- what pcbnew does for a via the
    GUI adds and KiCad for one the user places. (Inverted from what #489 s8
    first shipped; see the comment at that check.)
  * prevailing_via_protection is a deterministic majority. It is no longer the
    default for vias the tool adds -- over 886 corpus boards a prevailing spec
    never once disagreed with the board's own setup -- but it stays correct and
    is still the honest answer to "what do this board's vias say".
  * the via-in-pad fab note fires on a same-net pad hit and not otherwise

Run with:  python3 tests/test_489_via_tenting.py
"""
import os
import re
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_router'))  # #522
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_tools'))  # #522
sys.path.insert(0, os.path.join(ROOT_DIR, "rust_router"))

import kicad_parser as kp  # noqa: E402
from kicad_parser import Via, Pad  # noqa: E402
from kicad_writer import (generate_via_sexpr, prevailing_via_protection,  # noqa: E402
                          prevailing_via_protection_in_text)
from fab_notes import via_in_pad_sites, via_in_pad_summary  # noqa: E402

# A real hackrf_one via block: full IPC-4761 declaration, multi-line.
REAL_VIA = '''	(via
		(at 154.4118 141.5736)
		(size 0.6858)
		(drill 0.3302)
		(layers "F.Cu" "B.Cu")
		(capping no)
		(covering
			(front no)
			(back no)
		)
		(plugging
			(front no)
			(back no)
		)
		(filling no)
		(net "!MIX_BYPASS")
		(uuid "4d55cf6a-37ab-475e-af81-21fc5c01c815")
	)
'''

TENTED_VIA = '''	(via
		(at 10.0 20.0)
		(size 0.6)
		(drill 0.3)
		(layers "F.Cu" "B.Cu")
		(tenting (front yes) (back yes))
		(net "GND")
		(uuid "11111111-2222-3333-4444-555555555555")
	)
'''

PLAIN_VIA = '''	(via
		(at 30.0 40.0)
		(size 0.6)
		(drill 0.3)
		(layers "F.Cu" "B.Cu")
		(net "GND")
		(uuid "99999999-8888-7777-6666-555555555555")
	)
'''


def run():
    fails = []

    def check(cond, msg):
        if not cond:
            fails.append(msg)

    # ------------------------------------------------ parser reads all 5 tokens
    specs = kp._extract_via_protection_attrs(REAL_VIA)
    uuid = "4d55cf6a-37ab-475e-af81-21fc5c01c815"
    check(uuid in specs, f"no spec parsed for the real via; got {specs}")
    spec = specs.get(uuid, {})
    check(set(spec) == {'capping', 'covering', 'plugging', 'filling'},
          f"expected capping/covering/plugging/filling, got {sorted(spec)}")
    check(spec.get('capping') == 'no' and spec.get('filling') == 'no',
          f"whole-via tokens should read 'no', got {spec}")
    check('(front no)' in spec.get('covering', '') and '(back no)' in spec.get('covering', ''),
          f"per-side covering should keep both sides, got {spec.get('covering')!r}")
    check(kp._extract_via_protection_attrs(PLAIN_VIA) == {},
          "a via with no protection tokens must yield no spec")

    # ------------------------------------------------- writer re-emits verbatim
    out = generate_via_sexpr(154.4118, 141.5736, 0.6858, 0.3302,
                             ['F.Cu', 'B.Cu'], 3, net_name='!MIX_BYPASS',
                             tenting_attrs=spec)
    check('(covering (front no) (back no))' in out,
          f"per-side covering must round-trip on one line; got:\n{out}")
    check('(capping no)' in out and '(filling no)' in out,
          f"capping/filling must round-trip; got:\n{out}")
    check('(tenting' not in out,
          f"a via that specifies no tenting must NOT gain one; got:\n{out}")
    # Token order is the file's canonical order, not dict order.
    order = [m.group(1) for m in re.finditer(r'\((tenting|covering|plugging|capping|filling)[\s)]', out)]
    check(order == ['covering', 'plugging', 'capping', 'filling'],
          f"tokens should emit in canonical order, got {order}")

    # -------------------------------- nothing to say -> say NOTHING, either way
    # This assertion is INVERTED from the one #489 s8 originally shipped, which
    # required KiCad-10 output to carry `(tenting (front yes) (back yes))` when
    # the caller passed no spec. Probed against pcbnew 10.0.0: a via left at
    # TENTING_MODE_FROM_BOARD serialises with NO token, and a token appears only
    # when the via OVERRIDES the board -- so the old default silently converted
    # every via this tool added into an override.
    #
    # Measured over 886 corpus boards: three (nanovoltmeter_marge,
    # hexberry_fpga, pedal_404) declare `(tenting (front no) (back no))`
    # board-wide, and every via the tool added to them came back stamped
    # `(front yes) (back yes)` -- tenting a via the designer said to leave
    # exposed, which is a fab error. It hid because KiCad's FACTORY policy is
    # tented front+back, so on an ordinary board the stamp and the inheritance
    # agree.
    for label, out in (
            ('KiCad 10 (named net)',
             generate_via_sexpr(1, 2, 0.6, 0.3, ['F.Cu', 'B.Cu'], 5, net_name='GND')),
            ('KiCad 9 (numeric net)',
             generate_via_sexpr(1, 2, 0.6, 0.3, ['F.Cu', 'B.Cu'], 5))):
        for token in ('tenting', 'covering', 'plugging', 'capping', 'filling'):
            check(f'({token}' not in out,
                  f"{label}: a via with no spec must emit NO {token} token and "
                  f"inherit the board's (setup ...); got:\n{out}")

    # ------------------------------------------- prevailing convention, majority
    def _v(spec):
        return Via(x=0, y=0, size=0.6, drill=0.3, layers=['F.Cu', 'B.Cu'],
                   net_id=1, tenting_attrs=spec)

    a = {'capping': 'no'}
    b = {'tenting': '(front yes) (back yes)'}
    check(prevailing_via_protection([_v(a), _v(a), _v(b)]) == a,
          "the majority spec must win")
    check(prevailing_via_protection([_v(b), _v(a), _v(a)]) == a,
          "the majority must not depend on input order")
    check(prevailing_via_protection([]) is None,
          "no vias -> no prevailing spec")
    check(prevailing_via_protection([_v({}), _v({})]) is None,
          "vias with no spec -> no prevailing spec")
    # Deterministic tie-break: same answer whichever order a tie arrives in.
    t1 = prevailing_via_protection([_v(a), _v(b)])
    t2 = prevailing_via_protection([_v(b), _v(a)])
    check(t1 == t2, f"a tie must break deterministically, got {t1} vs {t2}")

    # Passed EXPLICITLY, a prevailing spec still round-trips: the function is
    # unchanged, only its use as the added-via default is retired.
    board_text = REAL_VIA + REAL_VIA.replace(uuid, "0000ffff-1111-2222-3333-444444444444")
    prevailing = prevailing_via_protection_in_text(board_text)
    check(prevailing is not None and prevailing.get('capping') == 'no',
          f"text-based prevailing spec should match the board, got {prevailing}")
    new_out = generate_via_sexpr(5, 5, 0.6, 0.3, ['F.Cu', 'B.Cu'], 1,
                                 net_name='GND', tenting_attrs=prevailing)
    check('(tenting (front yes) (back yes))' not in new_out,
          f"a new via must not be stamped tented on a board that declines it; got:\n{new_out}")
    check(prevailing_via_protection_in_text(PLAIN_VIA) is None,
          "a board with no protection tokens has no prevailing spec")

    # --------------------- a board that says DO NOT TENT must be obeyed (e2e)
    # The regression this whole inversion exists for, through the real writer
    # rather than the sexpr helper. nanovoltmeter_marge, hexberry_fpga and
    # pedal_404 in the stress corpus all carry exactly this setup, and every via
    # the tool added to them came back tented.
    import shutil as _sh, tempfile as _tf
    from kicad_writer import add_tracks_and_vias_to_pcb
    # version >= KICAD_10_MIN_VERSION (20250000) ON PURPOSE. A `(setup
    # (tenting ...))` block is a KiCad-10 feature, and below that threshold
    # is_kicad_10() is False, add_tracks_and_vias_to_pcb drops net_id_to_name,
    # the via emits `(net N)` -- and the OLD fallback emitted no tenting for the
    # numeric dialect anyway. The check would then pass against the very bug it
    # exists to catch. Verified by reverting the fix: this now FAILS.
    NO_TENT_BOARD = """(kicad_pcb
\t(version 20260206)
\t(generator "pcbnew")
\t(generator_version "10.0")
\t(general (thickness 1.6))
\t(paper "A4")
\t(layers
\t\t(0 "F.Cu" signal)
\t\t(2 "B.Cu" signal)
\t\t(44 "Edge.Cuts" user)
\t)
\t(setup
\t\t(tenting
\t\t\t(front no)
\t\t\t(back no)
\t\t)
\t)
\t(net 0 "")
\t(net 1 "/SIG")
\t(gr_line (start 0 0) (end 40 0) (stroke (width 0.1) (type solid)) (layer "Edge.Cuts") (uuid "e0000000-0000-0000-0000-000000000001"))
)
"""
    _tmp = _tf.mkdtemp(prefix='no_tent_')
    try:
        _src = os.path.join(_tmp, 'in.kicad_pcb')
        _dst = os.path.join(_tmp, 'out.kicad_pcb')
        with open(_src, 'w', encoding='utf-8') as _f:
            _f.write(NO_TENT_BOARD)
        add_tracks_and_vias_to_pcb(
            _src, _dst, [],
            [{'x': 10.0, 'y': 10.0, 'size': 0.6, 'drill': 0.3,
              'layers': ['F.Cu', 'B.Cu'], 'net_id': 1}],
            net_id_to_name={0: '', 1: '/SIG'})
        with open(_dst, encoding='utf-8') as _f:
            _out = _f.read()
        _via = _out[_out.index('\t(via'):]
        check('(tenting' not in _via,
              "a board declaring `(tenting (front no) (back no))` must not have "
              "the tool stamp tenting on the vias it adds -- that tents a via "
              "the designer said to leave exposed. Got:\n" + _via[:400])
        _parsed = kp.extract_vias(_out, {'': 0, '/SIG': 1})
        check(len(_parsed) == 1 and _parsed[0].tenting_attrs == {},
              f"the added via must carry no spec at all, got "
              f"{[v.tenting_attrs for v in _parsed]}")
    finally:
        _sh.rmtree(_tmp, ignore_errors=True)

    # ------------------------------------------------- via-in-pad fab note
    pad = Pad(component_ref='U1', pad_number='A1', global_x=10.0, global_y=10.0,
              local_x=0, local_y=0, size_x=0.5, size_y=0.5, shape='rect',
              layers=['F.Cu'], net_id=7, net_name='N', drill=0.0, pad_type='smd')
    far_pad = Pad(component_ref='U1', pad_number='A2', global_x=20.0, global_y=20.0,
                  local_x=0, local_y=0, size_x=0.5, size_y=0.5, shape='rect',
                  layers=['F.Cu'], net_id=7, net_name='N', drill=0.0, pad_type='smd')
    in_pad = {'x': 10.05, 'y': 9.95, 'size': 0.45, 'drill': 0.25, 'net_id': 7}
    outside = {'x': 15.0, 'y': 15.0, 'size': 0.45, 'drill': 0.25, 'net_id': 7}
    foreign = {'x': 10.0, 'y': 10.0, 'size': 0.45, 'drill': 0.25, 'net_id': 99}
    pads_by_net = {7: [pad, far_pad]}
    check(len(via_in_pad_sites([in_pad], pads_by_net)) == 1,
          "a via inside a same-net pad is via-in-pad")
    check(via_in_pad_sites([outside], pads_by_net) == [],
          "a via clear of every pad is not via-in-pad")
    check(via_in_pad_sites([foreign], pads_by_net) == [],
          "a via in a FOREIGN pad is a short, not via-in-pad -- not this report")
    # #695: an OFF-CENTRE via-in-pad has its centre outside the pad outline
    # while the barrel still overlaps the copper. Solder wicks through copper
    # continuity, not through the centre point, so that joint needs Type VII
    # too -- and the router places exactly this geometry on purpose (QFN
    # allow_via_in_pad, plane taps clamped to the pad edge, BGA underpad).
    # The pad is 0.5 wide (edge at x=10.25) and the barrel radius is 0.225.
    grazing = {'x': 10.40, 'y': 10.0, 'size': 0.45, 'drill': 0.25, 'net_id': 7}
    clear = {'x': 10.60, 'y': 10.0, 'size': 0.45, 'drill': 0.25, 'net_id': 7}
    check(abs(grazing['x'] - 10.0) > pad.size_x / 2,
          "the grazing via's CENTRE must be outside the pad, or this row "
          "passes on the centre test it is meant to retire")
    check(len(via_in_pad_sites([grazing], pads_by_net)) == 1,
          "a via whose BARREL overlaps a same-net pad is via-in-pad (#695)")
    check(via_in_pad_sites([clear], pads_by_net) == [],
          "a via whose barrel clears the pad is NOT via-in-pad -- without "
          "this control the row above passes on a check that credits all")
    # The credit must follow the pad's SHAPE, not a box around it. bga_fanout
    # hands this function its dog-bone vias, and those sit on the half-pitch
    # DIAGONAL from a round BGA pad -- precisely where a box inflated by the
    # barrel radius over-credits by sqrt(2). Measured on ulx3s's BGA-381 while
    # fixing #695: the box named all 379 of them and the exact test named none,
    # the nearest copper being 0.37mm from a 0.225mm barrel. A false site here
    # is a fab process requirement (filled/capped/plated), not a stray line.
    round_pad = Pad(component_ref='U2', pad_number='A2', global_x=0.0, global_y=0.0,
                    local_x=0, local_y=0, size_x=0.4, size_y=0.4, shape='circle',
                    layers=['F.Cu'], net_id=8, net_name='M', drill=0.0,
                    pad_type='smd')
    dogbone = {'x': 0.4, 'y': 0.4, 'size': 0.45, 'drill': 0.25, 'net_id': 8}
    # On the branch: inside the inflated BOX (0.4 <= 0.2+0.225 on both axes),
    # outside the real copper (radially 0.566 - 0.2 = 0.366 > 0.225).
    check(abs(dogbone['x']) <= round_pad.size_x / 2 + dogbone['size'] / 2
          and abs(dogbone['y']) <= round_pad.size_y / 2 + dogbone['size'] / 2,
          "the dog-bone via must be inside the pad's inflated BOX, or this "
          "row does not exercise the corner over-credit it names")
    check(via_in_pad_sites([dogbone], {8: [round_pad]}) == [],
          "a dog-bone via on the half-pitch diagonal of a ROUND pad is not "
          "via-in-pad -- the credit follows the pad shape, not its box (#695)")
    # The cheap reject must be the DIAGONAL one, not an axis-aligned box: a
    # rotated pad keeps size_x/size_y in its OWN frame and carries the tilt in
    # rect_rotation, so a board-space box clips its copper. This via's centre
    # is literally INSIDE the copper and a box gate drops it -- a MISSED Type
    # VII callout, the unsafe direction.
    tilted = Pad(component_ref='U3', pad_number='1', global_x=0.0, global_y=0.0,
                 local_x=0, local_y=0, size_x=1.0, size_y=0.2, shape='rect',
                 layers=['F.Cu'], net_id=9, net_name='R', drill=0.0,
                 pad_type='smd', rotation=45.0)
    tilted.rect_rotation = 45.0
    on_tilt = {'x': 0.35, 'y': 0.35, 'size': 0.45, 'drill': 0.25, 'net_id': 9}
    check(abs(on_tilt['x']) > tilted.size_y / 2 + on_tilt['size'] / 2,
          "the tilted-pad via must fall outside the axis-aligned box, or "
          "this row does not exercise the gate it names")
    check(len(via_in_pad_sites([on_tilt], {9: [tilted]})) == 1,
          "a via inside a ROTATED pad's copper is via-in-pad -- the reject "
          "must be the diagonal prefilter, not a board-space box (#695)")
    # The no-checker fallback is a real branch and had no coverage: with
    # check_drc unavailable _pad_holds falls back to the pad's circumscribed
    # reach, which must still refuse the dog-bone corner artifact above.
    import builtins
    _real_import = builtins.__import__

    def _no_check_drc(name, *a, **kw):
        if name == 'check_drc':
            raise ImportError('forced: exercising the fallback branch')
        return _real_import(name, *a, **kw)

    builtins.__import__ = _no_check_drc
    try:
        check(len(via_in_pad_sites([grazing], pads_by_net)) == 1,
              "with no check_drc, the fallback still names a real overlap")
        check(via_in_pad_sites([dogbone], {8: [round_pad]}) == [],
              "with no check_drc, the fallback's circumscribed-reach guard "
              "still refuses the dog-bone corner artifact")
    finally:
        builtins.__import__ = _real_import
    summary = via_in_pad_summary([in_pad, outside], pads_by_net)
    check(summary and summary['count'] == 1 and summary['pads'] == ['U1.A1'],
          f"summary should name the one hit pad, got {summary}")
    check('IPC-4761' in (summary or {}).get('note', ''),
          "the fab note must name the IPC-4761 requirement it creates")
    check(via_in_pad_summary([outside], pads_by_net) is None,
          "no via-in-pad -> no record (so nothing is printed)")

    if fails:
        for f in fails:
            print(f"  FAIL  {f}")
        print(f"\n{len(fails)} check(s) FAILED")
        return False
    print("PASS  #489 §8 via protection round-trip + prevailing convention + fab note")
    return True


if __name__ == "__main__":
    sys.exit(0 if run() else 1)
