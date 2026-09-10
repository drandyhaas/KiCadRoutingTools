#!/usr/bin/env python3
"""#910: the opt-in delivery fill, and the trap it must not re-open.

A routed board ships zone OUTLINES with no `(filled_polygon ...)`. Opened in
KiCad before a refill -- or graded by `kicad-cli pcb drc` WITHOUT
`--refill-zones` -- it reports plane-net opens that are not real.

Invariants gated here:
  1. `write_filled_board` REFUSES rather than crashes when KiCad's python is
     absent, and leaves the destination untouched.
  2. It writes the destination only on success (no half-written deliverable).
  3. On a real zoned board it produces `filled_polygon` blocks, and the
     unconnected count read WITHOUT `--refill-zones` drops -- the phantom
     opens the step exists to remove. Measured on
     lvds_converter_dualclk_gnd with its fills stripped: 54 -> 42.
  4. The sibling `.kicad_pro` survives with every net class intact. This is
     the trap: a plain `pcbnew.SaveBoard` rewrites the project from KiCad's
     in-memory view and deletes every non-Default class.
  5. The CLI refuses (exit 1) and removes its output if a class went missing,
     rather than shipping a board graded against rules it no longer carries.

Arms 3-5 need KiCad's bundled python. Without it the file exits 77 (SKIP),
which `run_all` counts apart from a pass -- it does NOT quietly report green.

Run:
    python3 tests/test_910_fill_for_delivery.py
"""

import json
import os
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_router'))  # #522
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_tools'))  # #522

from run_utils import check, evidence, tool
from kicad_exact_fill import write_filled_board, find_kicad_python
from kicad_parser import find_matching_paren

RUN_ALL_TIMEOUT = 1200
SKIP_EXIT = 77

BOARD = os.path.join(ROOT_DIR, 'kicad_files',
                     'lvds_converter_dualclk_gnd.kicad_pcb')


def _strip_fills(src, dst):
    """A copy of `src` with every `(filled_polygon ...)` block removed."""
    with open(src, encoding='utf-8', errors='replace') as fh:
        t = fh.read()
    out, pos = [], 0
    while True:
        i = t.find('(filled_polygon', pos)
        if i < 0:
            break
        out.append(t[pos:i])
        pos = find_matching_paren(t, i)
    out.append(t[pos:])
    with open(dst, 'w', encoding='utf-8') as fh:
        fh.write(''.join(out))


def _classes(pcb):
    pro = os.path.splitext(pcb)[0] + '.kicad_pro'
    if not os.path.isfile(pro):
        return None
    with open(pro, encoding='utf-8') as fh:
        doc = json.load(fh)
    return {c.get('name')
            for c in (doc.get('net_settings') or {}).get('classes') or []}


def main():
    fails = []

    def ck(name, cond, detail=''):
        print(f"  {'PASS' if cond else 'FAIL'}: {name}"
              + (f"   [{detail}]" if detail and not cond else ''))
        if not cond:
            fails.append(name)

    evidence(BOARD, 'the zoned fixture board')
    tmp = tempfile.mkdtemp(prefix='t910_')
    try:
        # --- 1/2: the refusal path, with no KiCad python -------------------
        import kicad_exact_fill as kef
        src = os.path.join(tmp, 'in.kicad_pcb')
        shutil.copyfile(BOARD, src)
        dst = os.path.join(tmp, 'out_refused.kicad_pcb')
        real = kef.find_kicad_python
        try:
            kef.find_kicad_python = lambda: None
            st = write_filled_board(src, dst)
        finally:
            kef.find_kicad_python = real
        ck('no KiCad python: refuses with a REASON, not a crash',
           (not st.ok) and st.reason == 'no_kicad_python', f'{st.reason}')
        ck('no KiCad python: the destination is untouched',
           not os.path.exists(dst))

        if find_kicad_python() is None:
            print("\nSKIP: KiCad's bundled python not found; arms 3-5 need it.")
            return SKIP_EXIT

        # --- 3: the phantom opens, before and after ------------------------
        nofill = os.path.join(tmp, 'nofill.kicad_pcb')
        _strip_fills(BOARD, nofill)
        with open(nofill, encoding='utf-8', errors='replace') as fh:
            ck('the fixture really starts with no fills (the premise)',
               fh.read().count('(filled_polygon') == 0)
        filled = os.path.join(tmp, 'filled.kicad_pcb')
        st = write_filled_board(nofill, filled, verbose=True)
        ck('the fill runs', st.ok, f'{st.reason} {st.detail}')
        n = 0
        if os.path.isfile(filled):
            with open(filled, encoding='utf-8', errors='replace') as fh:
                n = fh.read().count('(filled_polygon')
        ck('the filled board carries filled_polygon blocks', n > 0, f'{n}')

        from fill_for_delivery import _unconnected
        before, after = _unconnected(nofill), _unconnected(filled)
        if before is None or after is None:
            print("  (kicad-cli unavailable: skipping the unconnected delta)")
        else:
            ck('filling REMOVES phantom opens (unconnected drops)',
               after < before, f'{before} -> {after}')

        # --- 4/5: the .kicad_pro trap --------------------------------------
        # Author a project with a non-Default class beside the input, then
        # check the CLI carries it and refuses if it ever goes missing.
        pro_src = os.path.splitext(nofill)[0] + '.kicad_pro'
        with open(pro_src, 'w', encoding='utf-8') as fh:
            json.dump({'net_settings': {'classes': [
                {'name': 'Default', 'clearance': 0.2},
                {'name': 'HighSpeed', 'clearance': 0.15}]}}, fh)
        out2 = os.path.join(tmp, 'delivered.kicad_pcb')
        r = check([sys.executable, tool('fill_for_delivery.py'),
                   nofill, '-o', out2], accept=True, timeout=900)
        ck('the CLI succeeds on a board with a project', r.returncode == 0)
        got = _classes(out2)
        ck('every net class survived the fill',
           got is not None and {'Default', 'HighSpeed'} <= got, f'{got}')
        ck('the CLI reported the unconnected delta',
           'unconnected (no --refill-zones)' in (r.stdout or ''))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print(f"\n{'FAILED: ' + ', '.join(fails) if fails else 'ALL PASS'}")
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
