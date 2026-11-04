#!/usr/bin/env python3
"""A pad whose number is written bare, `(pad A5 smd rect ...)`, is a pad to
the text readers that still asked for a quoted number.

KiCad writes every pad number quoted, but it reads the bare spelling, and
footprints imported from other EDA tools keep it in the board file (a USB-C
receptacle numbered A5, B12 ... is the common one). #1290 taught the pad
parser and the placement writer the bare spelling; two text readers kept
the quoted-only pattern:

  * model_export._assembled: a footprint whose pads are all bare is "not an
    assembled part", so its 3D model is neither staged nor reported missing;
  * route_bus._pad_block: the bus step cannot find a bare pad's block.

    python3 tests/test_bare_pad_number_readers.py
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for sub in ('py_router', 'py_placer', 'awx', ''):
    sys.path.insert(0, os.path.join(ROOT, sub))

import model_export                                              # noqa: E402

QUOTED = """(footprint "Connector_USB:USB_C_Receptacle"
  (layer "F.Cu") (at 10 20)
  (pad "A5" smd rect (at 0 0) (size 0.7 1.2) (layers "F.Cu") (net 2 "CC1"))
  (pad "7" thru_hole oval (at 4 2) (size 1.1 1.9) (layers "*.Cu") (net 1 "GND"))
)"""
BARE = QUOTED.replace('(pad "A5"', '(pad A5').replace('(pad "7"', '(pad 7')


def test_assembled_reads_a_bare_pad_number():
    assert model_export._assembled(QUOTED)
    assert model_export._assembled(BARE), 'bare pad numbers: part not assembled'
    # the other exits still hold
    assert not model_export._assembled(BARE.replace('(layer "F.Cu")', '(layer "F.Cu") (attr dnp)'))
    assert not model_export._assembled('(footprint "x" (pad "" np_thru_hole circle (at 0 0)))')
    print("  PASS: _assembled sees bare-numbered copper pads")


def test_pad_block_finds_a_bare_pad():
    from route_bus import _pad_block
    for text in (QUOTED, BARE):
        for num in ('A5', '7'):
            span = _pad_block(text, num, True)
            assert span is not None, (num, text[:40])
            assert text[span[0]:span[1]].startswith('(pad') and num in text[span[0]:span[1]]
    # a number that is only a prefix of another does not match it
    assert _pad_block(BARE, 'A', True) is None
    assert _pad_block(BARE, 'A5', False) is None
    print("  PASS: _pad_block finds bare pads and not prefixes")


def main():
    fns = [v for k, v in sorted(globals().items()) if k.startswith('test_')]
    for fn in fns:
        fn()
    print(f"{len(fns)} test(s) passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
