#!/usr/bin/env python3
"""#962: the board's via-protection POLICY is read from `(setup ...)` in one spelling.

A via with no protection spec of its own inherits the board's `(setup ...)`.
The via-in-paste grade has to know what that inherited process IS, and until
#962 nothing read it. esp_prog declares `(capping no) (filling no)`, the
opposite of the IPC-4761 Type VII a via under a paste opening needs.

Invariants:
1. The KiCad 10 multi-line form (esp_prog) reads token for token.
2. The legacy KiCad 9 form `(tenting front back)` (sonde_u) is canonicalised
   to `(front yes) (back yes)`. The tokens it does not write take KiCad's
   factory defaults. Values probed on pcbnew 10.0.0; the pcbnew path is gated
   by tests/gui_parity/test_962_paste_parity.py.
3. `front` alone and `none` canonicalise per side.
4. A board with no `(setup ...)` at all gets the defaults, all five tokens.
5. A via's OWN `(capping yes)` never leaks into the board policy. The setup is
   read from the balanced `(setup ...)` block only.
6. `pad_to_paste_clearance` does not swallow `pad_to_paste_clearance_ratio`,
   and both are read.

Run:
    python3 tests/test_962_setup_via_protection.py
"""

import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_router'))

from kicad_parser import (parse_kicad_pcb, extract_board_setup_paste_and_protection,  # noqa: E402
                          canonical_via_protection_setup, VIA_PROTECTION_SETUP_DEFAULTS)

RUN_ALL_FAST_OK = True
FAILS = []


def check(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}: {name}"
          + (f"   [{detail}]" if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


def main():
    esp = parse_kicad_pcb(os.path.join(ROOT_DIR, 'kicad_files', 'esp_prog.kicad_pcb'))
    check('1. esp_prog: KiCad 10 setup reads token for token',
          esp.board_info.via_protection_setup == {
              'tenting': '(front yes) (back yes)', 'covering': '(front no) (back no)',
              'plugging': '(front no) (back no)', 'capping': 'no', 'filling': 'no'},
          str(esp.board_info.via_protection_setup))
    son = parse_kicad_pcb(os.path.join(ROOT_DIR, 'kicad_files', 'sonde_u.kicad_pcb'))
    check('2. sonde_u: legacy (tenting front back) canonicalises, rest default',
          son.board_info.via_protection_setup == VIA_PROTECTION_SETUP_DEFAULTS,
          str(son.board_info.via_protection_setup))

    c = canonical_via_protection_setup({'tenting': 'front'})
    check('3. (tenting front) -> front yes, back no',
          c['tenting'] == '(front yes) (back no)', c['tenting'])
    c = canonical_via_protection_setup({'tenting': 'none', 'capping': 'yes'})
    check('3. (tenting none) -> both no; capping carried',
          c['tenting'] == '(front no) (back no)' and c['capping'] == 'yes', str(c))
    c = canonical_via_protection_setup({'covering': '(front  yes)\n (back no)'})
    check('3. an already per-side form is whitespace-normalised, not rewritten',
          c['covering'] == '(front yes) (back no)', c['covering'])

    pc, pr, vp = extract_board_setup_paste_and_protection('(kicad_pcb (version 1))')
    check('4. no (setup ...) -> 0/0 and all five defaults',
          pc == 0.0 and pr == 0.0 and vp == VIA_PROTECTION_SETUP_DEFAULTS, str(vp))

    txt = ('(kicad_pcb (setup (pad_to_paste_clearance -0.05) '
           '(pad_to_paste_clearance_ratio -0.1) (tenting front back))\n'
           ' (via (at 1 1) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") '
           '(capping yes) (filling yes) (net 0)))')
    pc, pr, vp = extract_board_setup_paste_and_protection(txt)
    check('5. a via\'s own (capping yes)(filling yes) does not leak into the policy',
          vp['capping'] == 'no' and vp['filling'] == 'no', str(vp))
    check('6. both paste setup numbers are read, neither swallowing the other',
          pc == -0.05 and pr == -0.1, f'{pc} {pr}')
    check('6. esp_prog pad_to_paste_clearance is -0.0508',
          esp.board_info.pad_to_paste_clearance == -0.0508
          and esp.board_info.pad_to_paste_clearance_ratio == 0.0)

    print(f"\n{'ALL PASS' if not FAILS else f'{len(FAILS)} FAILED'}")
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
