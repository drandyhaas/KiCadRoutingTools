#!/usr/bin/env python3
"""#908: the writer must not delete a component's own copper.

`move_copper_graphics_to_silkscreen` relocates NET-LESS copper graphics off
copper (#146: an unmodelled copper logo shorted ~30 orangecrab plane
traces/vias). Its "net-tied copper is functional, leave it" guard (#337/#369)
is a NO-OP for footprint shapes, because a footprint shape cannot carry a
`(net ...)` in KiCad at all -- so EVERY `fp_*` copper shape was relocated,
including watchy's PCB antenna and the SOT89 tab under esp_prog's U2.

The owner decides instead: a footprint WITH pads owns functional land-pattern
copper (kept on copper, and modelled by the parser); a footprint with NO pads
is a logo (moved, exactly as #146 does).

Invariants gated here:
  1. `fp_poly` on F.Cu in a 2-pad footprint  -> UNCHANGED.
  2. The same `fp_poly` in a 0-pad footprint -> moved to F.SilkS.
  3. B.Cu behaves the same way on both arms (the layer is not the decision).
  4. Board-level net-less `gr_poly`          -> still moved (#146 intact).
  5. Board-level net-tied `gr_poly`          -> still left  (#337 intact).
  6. A pad-bearing footprint's SILKSCREEN art is untouched by either arm
     (the gate must not become "never touch a footprint").
  7. Real corpus boards: esp_prog / tigard / ulx3s / watchy move 0 and keep
     1 / 1 / 6 / 12; orangecrab_ext_pll still moves its 4 logo shapes;
     splitflap_driver (no footprint copper) is unchanged either way.

Run:
    python3 tests/test_908_writer_owner_gate.py
"""

import io
import contextlib
import os
import re
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_router'))  # #522
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_tools'))  # #522

from run_utils import evidence
from kicad_writer import move_copper_graphics_to_silkscreen

#: This test imports run_utils (for `evidence`) but never shells out and never
#: routes; it is pure text in and text out. Measured well under a second.
RUN_ALL_FAST_OK = True


def _fp(pads, shape_layer='F.Cu', ref='U1'):
    """One footprint block with `pads` pads and one fp_poly on `shape_layer`."""
    pad_txt = '\n'.join(
        '   (pad "%d" smd rect (at %d 0) (size 0.6 0.6) (layers "F.Cu") '
        '(net %d "/N%d"))' % (i + 1, i, i + 1, i + 1) for i in range(pads))
    return ('(footprint "L:P" (layer "F.Cu") (at 10 10)\n'
            '   (property "Reference" "%s")\n'
            '%s\n'
            '   (fp_poly (pts (xy 0 0) (xy 1 0) (xy 1 1) (xy 0 1))\n'
            '     (stroke (width 0) (type solid)) (fill yes)\n'
            '     (layer "%s") (uuid "aaa"))\n'
            '   (fp_poly (pts (xy 2 0) (xy 3 0) (xy 3 1))\n'
            '     (stroke (width 0.1) (type solid))\n'
            '     (layer "F.SilkS") (uuid "bbb")))'
            % (ref, pad_txt, shape_layer))


def _board(body):
    return ('(kicad_pcb (version 20221018)\n (net 0 "")\n (net 1 "/N1")\n'
            ' (net 2 "/N2")\n%s\n)' % body)


def _move(text):
    """Run the pass, returning (new_text, moved_count, kept_count)."""
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        out = move_copper_graphics_to_silkscreen(text)
    log = buf.getvalue()
    m = re.search(r'Moved (\d+) copper graphic', log)
    k = re.search(r'Kept (\d+) footprint copper shape', log)
    return out, int(m.group(1)) if m else 0, int(k.group(1)) if k else 0


def main():
    fails = []

    def check(name, cond, detail=''):
        print(f"  {'PASS' if cond else 'FAIL'}: {name}"
              + (f"   [{detail}]" if detail and not cond else ''))
        if not cond:
            fails.append(name)

    # --- 1/2/3: the owner decides, on both copper sides -------------------
    for layer, silk in (('F.Cu', 'F.SilkS'), ('B.Cu', 'B.SilkS')):
        src = _board(_fp(2, layer))
        out, moved, kept = _move(src)
        check(f'2-pad footprint keeps its {layer} copper',
              out == src and moved == 0 and kept == 1,
              f'moved={moved} kept={kept} changed={out != src}')

        src0 = _board(_fp(0, layer))
        out0, moved0, kept0 = _move(src0)
        check(f'0-pad logo footprint still moves off {layer}',
              moved0 == 1 and kept0 == 0
              and f'(layer "{silk}") (uuid "aaa")' in out0,
              f'moved={moved0} kept={kept0}')

        # 6: silkscreen art inside the kept footprint is untouched either way
        check(f'{layer} arm leaves the footprint\'s own silk art alone',
              out.count('(layer "F.SilkS") (uuid "bbb")') == 1
              and out0.count('(layer "F.SilkS") (uuid "bbb")') == 1)

    # --- 4/5: board-level gr_* behaviour is unchanged ---------------------
    gr_free = _board('(gr_poly (pts (xy 0 0) (xy 1 0) (xy 1 1))'
                     ' (stroke (width 0.1) (type solid)) (fill yes)'
                     ' (layer "F.Cu") (uuid "ccc"))')
    out, moved, kept = _move(gr_free)
    check('board-level net-less gr_poly is still moved (#146)',
          moved == 1 and kept == 0 and '(layer "F.SilkS") (uuid "ccc")' in out,
          f'moved={moved}')

    gr_tied = _board('(gr_poly (pts (xy 0 0) (xy 1 0) (xy 1 1))'
                     ' (stroke (width 0.1) (type solid)) (fill yes)'
                     ' (layer "F.Cu") (net 1 "/N1") (uuid "ddd"))')
    out, moved, kept = _move(gr_tied)
    check('board-level net-TIED gr_poly is still left (#337)',
          out == gr_tied and moved == 0, f'moved={moved}')

    # --- 7: the real corpus ----------------------------------------------
    # (moved, kept) per board. The `kept` figures are the shapes that used to
    # be deleted from copper on every write.
    expect = {
        'esp_prog': (0, 1),
        'tigard': (0, 1),
        'ulx3s': (0, 6),
        'watchy': (0, 12),
        'orangecrab_ext_pll': (4, 0),
        'splitflap_driver': (0, 0),
    }
    seen = 0
    for board, (want_moved, want_kept) in sorted(expect.items()):
        path = os.path.join(ROOT_DIR, 'kicad_files', f'{board}.kicad_pcb')
        if not os.path.isfile(path):
            check(f'{board}: corpus board present', False, path)
            continue
        evidence(path, f'{board} corpus board')
        with open(path, encoding='utf-8', errors='replace') as fh:
            src = fh.read()
        out, moved, kept = _move(src)
        seen += 1
        check(f'{board}: moved={want_moved} kept={want_kept}',
              (moved, kept) == (want_moved, want_kept),
              f'got moved={moved} kept={kept}')
        check(f'{board}: text changes only when something moved',
              (out != src) == (want_moved > 0))
    # Anti-vacuity: an empty corpus must not pass this section silently.
    check('the corpus section actually ran', seen == len(expect),
          f'{seen}/{len(expect)}')

    print(f"\n{'FAILED: ' + ', '.join(fails) if fails else 'ALL PASS'}")
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
