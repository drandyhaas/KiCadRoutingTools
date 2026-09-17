#!/usr/bin/env python3
"""The staged BOARD MINIMUM never outranks a pad's own declared clearance.

    python3 tests/test_staged_min_clearance_respects_pad_override.py

`kicad_drc_compare._staged_copy` stages a board so kicad-cli grades it at the
routed floor. It did that two ways: it equalised the Default net class, AND it
forced `rules.min_clearance` up to the same value. The second one is not the
same rule as the first.

In KiCad a per-pad `local_clearance` REPLACES the net class for that pad
(#326) -- but the BOARD MINIMUM outranks the pad override. So forcing
`min_clearance` up grades the board's own declaration away and reports copper
the router legitimately laid at that override. The DRC writeback caps this key
DOWN to a pad override for exactly that reason (#900/#530); staging it back up
undid that.

MEASURED on a20_can, whose U1 (8 pads), C1, C2, CAN_T1 and the 3.3V/5.0V1
jumper all declare `local_clearance` 0.0508 (2 mil):

    project                                        kicad-cli clearance items
    board's own (min_clearance 0.0, Default 0.254)            0
    staged (min_clearance 0.254, Default 0.254)             48
    Default raised to 0.60, min_clearance 0.0              255

The 48 are pad-to-track pairs at 0.070..0.234mm -- every one legal at the
pad's own 0.0508, none >= 0.254. The third row is the control: the NET CLASS
arm still grades, so the routed floor is still enforced wherever a pad has
not declared otherwise. This was a20_can's entire kicad_only=49, and with
mod_bme280 (37->2) and bitaxe_ultra (11->0) most of the corpus `kicad_drc`
totals that made HEAD look far worse than v0.22.0.

Whole-arm effect, HEAD `d96877f1`, kicad items before -> after:
  with a pad override BELOW the floor: a20_can 49->2, mod_bme280 37->2,
    esp_prog 6->1, fomu 3->0, bitaxe_ultra 11->0
  at or above it, or none: watchy 3->3, upduino 2->2, cynthion 6->6,
    lna3030 6->6, hackrf_one 1->1, and every override-free board unchanged
  drc_real: UNCHANGED on every board (this touches the kicad arm only).
"""
import io
import json
import os
import sys
import contextlib
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('', 'py_router', 'py_tools', 'tests/stress'):
    _q = os.path.join(ROOT, _p)
    if _q not in sys.path:
        sys.path.insert(0, _q)

import kicad_drc_compare as KD  # noqa: E402

STRESS = os.path.expanduser('~/Documents/kicad_stress_test')
A20 = f'{STRESS}/cloud_head-kc_d96877f1/set3/a20_can/step2_route.kicad_pcb'
LNA = f'{STRESS}/cloud_head-kc_d96877f1/set3/lna3030/step2_route.kicad_pcb'

fails = []


def check(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}: {name}" + (f"   [{detail}]" if detail else ''))
    if not cond:
        fails.append(name)


# Every probe answers rather than raising, so the UNFIXED code fails each row on
# its own terms instead of dying on the first missing attribute -- a battery
# that crashes reports nothing about the rows it never reached, and reads as one
# tidy failure. (It did exactly that on the first draft.)
def pad_override(board):
    fn = getattr(KD, '_smallest_pad_override', None)
    if fn is None:
        return '<no _smallest_pad_override: the staging does not consult pad overrides>'
    try:
        KD._PAD_OVERRIDE_CACHE.pop(board, None)
    except AttributeError:
        pass
    try:
        return fn(board)
    except Exception as e:
        return f'<{type(e).__name__}: {e}>'


def _mk(tmp, pad_clearance, min_clearance=0.0, default_class=0.254):
    """A board + project pair: one footprint, one pad, optional override."""
    pc = f' (clearance {pad_clearance})' if pad_clearance is not None else ''
    pcb = f"""(kicad_pcb (version 20240108) (generator test)
  (net 0 "") (net 1 "/A")
  (footprint "L:P" (layer "F.Cu") (at 10 10)
    (property "Reference" "U1" (at 0 0) (layer "F.SilkS"))
    (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu"){pc} (net 1 "/A"))
  )
)
"""
    b = os.path.join(tmp, 'bd.kicad_pcb')
    open(b, 'w').write(pcb)
    pro = {"board": {"design_settings": {"rules": {"min_clearance": min_clearance},
                                         "rule_severities": {}}},
           "net_settings": {"classes": [{"name": "Default", "clearance": default_class}]}}
    json.dump(pro, open(os.path.join(tmp, 'bd.kicad_pro'), 'w'))
    return b


def staged_min_clearance(board, clearance):
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        d, b2 = KD._staged_copy(board, clearance)
    cfg = json.load(open(os.path.splitext(b2)[0] + '.kicad_pro'))
    return cfg['board']['design_settings']['rules']['min_clearance'], cfg


def main():
    print("1. the staged board minimum is capped at the pad's own declaration")
    with tempfile.TemporaryDirectory() as t:
        b = _mk(t, 0.0508)
        po = pad_override(b)
        check("_smallest_pad_override reads the pad's value", po == 0.0508, f"{po}")
        mc, cfg = staged_min_clearance(b, 0.254)
        check("staged min_clearance does NOT exceed the pad override",
              mc <= 0.0508 + 1e-12, f"min_clearance={mc} pad override=0.0508")
        check("the NET CLASS arm is untouched (the routed floor still grades)",
              abs(cfg['net_settings']['classes'][0]['clearance'] - 0.254) < 1e-12,
              f"Default={cfg['net_settings']['classes'][0]['clearance']}")

    print("2. a pad override ABOVE the routed floor does not lower the minimum")
    with tempfile.TemporaryDirectory() as t:
        b = _mk(t, 0.5)
        pad_override(b)
        mc, _ = staged_min_clearance(b, 0.254)
        check("staged min_clearance is still the routed floor",
              abs(mc - 0.254) < 1e-12, f"min_clearance={mc}")

    print("3. a board with NO pad override is unaffected")
    with tempfile.TemporaryDirectory() as t:
        b = _mk(t, None)
        po = pad_override(b)
        check("_smallest_pad_override is None", po is None, f"{po}")
        mc, _ = staged_min_clearance(b, 0.254)
        check("staged min_clearance is the routed floor",
              abs(mc - 0.254) < 1e-12, f"min_clearance={mc}")

    print("4. the real boards")
    if os.path.isfile(A20):
        po = pad_override(A20)
        check("a20_can declares a 2 mil pad override",
              isinstance(po, float) and abs(po - 0.0508) < 1e-9, f"{po}")
        mc, _ = staged_min_clearance(A20, 0.254)
        check("...and its staged minimum respects it", mc <= 0.0508 + 1e-12, f"min_clearance={mc}")
    else:
        print(f"  SKIP: {A20} not present -- rows 1-3 still discriminate")
    if os.path.isfile(LNA):
        po = pad_override(LNA)
        check("lna3030 declares none, so nothing changes for it", po is None, f"{po}")
    else:
        print(f"  SKIP: {LNA} not present")

    print(f"\n{len(fails)} failed")
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
