#!/usr/bin/env python3
"""Remove ONLY copper from a .kicad_pcb, preserving everything else.

Makes the unrouted twin of a routed corpus board, which is what a
perturbed-corpus run needs as a subject: `placement_driver --stage P0`
refuses a board carrying copper, and on a ROUTED board that copper also
encodes the original poses (run 14 measured 301 of 569 pads within 5 um of
their own track endpoint, which `fence_audit` cannot see because it
compares poses and never opens copper).

    python3 -X utf8 tests/stress/strip_copper_only.py SRC.kicad_pcb DST.kicad_pcb

Why this exists instead of `tests/stress/strip_routing.py` / `prep_set2.py`:
those are corpus NORMALIZERS and they rewrite `Edge.Cuts` (RDP-simplify the
outline, rebuild it as chained segments, drop graphics). The routing skill
forbids them for exactly that reason -- the outline is a mechanical decision.
For a recovery run the outline must stay bit-identical, because the final board
is compared against a human reference that has the ORIGINAL outline.

So this removes, and removes nothing else:
  * top-level `(segment ...)`  -- straight track copper
  * top-level `(arc ...)`      -- curved track copper (this board is arc-heavy)
  * top-level `(via ...)`
  * `(filled_polygon ...)` inside zones -- the pour result, not the pour intent

Zone DEFINITIONS survive: a zone is design intent (which net pours where), not
routing output, and dropping it would silently delete part of the board's
design. Footprints, pads, graphics, `Edge.Cuts`, `setup`, stackup, netlist and
every uuid are untouched.

The scanner is quote-aware: KiCad s-expr strings may contain parentheses, and a
naive depth counter desyncs on the first `(gr_text "foo (bar)")`.
"""
import os
import re
import shutil
import sys

DROP_TOP = {'segment', 'arc', 'via'}
DROP_NESTED = {'filled_polygon'}
_HEAD = re.compile(r'\(\s*([A-Za-z_0-9]+)')


def _forms(text):
    """Yield (start, end, depth, head) for every s-expression form."""
    stack = []
    in_str = False
    esc = False
    for i, ch in enumerate(text):
        if in_str:
            if esc:
                esc = False
            elif ch == '\\':
                esc = True
            elif ch == '"':
                in_str = False
            continue
        if ch == '"':
            in_str = True
        elif ch == '(':
            stack.append(i)
        elif ch == ')':
            if not stack:
                raise SystemExit('unbalanced ")" at offset %d' % i)
            s = stack.pop()
            m = _HEAD.match(text, s)
            yield s, i + 1, len(stack), (m.group(1) if m else '')
    if stack:
        raise SystemExit('unbalanced "(" -- %d unclosed' % len(stack))


def strip(text):
    cuts = []
    for s, e, depth, head in _forms(text):
        if depth == 1 and head in DROP_TOP:
            cuts.append((s, e))
        elif depth >= 2 and head in DROP_NESTED:
            cuts.append((s, e))
    # Forms are yielded innermost-first; keep only outermost non-overlapping.
    cuts.sort()
    merged = []
    for s, e in cuts:
        if merged and s < merged[-1][1]:
            merged[-1] = (merged[-1][0], max(merged[-1][1], e))
        else:
            merged.append((s, e))
    out = []
    prev = 0
    for s, e in merged:
        # also swallow the run of whitespace that indented the dropped form
        b = s
        while b > prev and text[b - 1] in ' \t':
            b -= 1
        if b > prev and text[b - 1] == '\n':
            b -= 1
        out.append(text[prev:b])
        prev = e
    out.append(text[prev:])
    return ''.join(out), len(merged)


def main(src, dst):
    text = open(src, encoding='utf-8').read()
    new, n = strip(text)
    os.makedirs(os.path.dirname(os.path.abspath(dst)), exist_ok=True)
    with open(dst, 'w', encoding='utf-8', newline='') as f:
        f.write(new)
    for ext in ('.kicad_pro', '.kicad_prl', '.kicad_dru'):
        sib = os.path.splitext(src)[0] + ext
        if os.path.exists(sib):
            shutil.copyfile(sib, os.path.splitext(dst)[0] + ext)
    print('dropped %d copper form(s); %d -> %d chars' % (n, len(text), len(new)))


if __name__ == '__main__':
    if len(sys.argv) != 3:
        raise SystemExit(__doc__.strip().splitlines()[2].strip())
    main(sys.argv[1], sys.argv[2])
