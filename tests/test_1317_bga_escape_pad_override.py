#!/usr/bin/env python3
"""#1317: the BGA dog-bone / under-pad escape honours a footprint clearance
override above the run clearance.

imx6ulz's U1 declares a footprint clearance of 0.12 over a 0.1 run. The
dog-bone engine (bga_fanout/underpad.py) priced every ball at the scalar
--clearance -- its occupancy stamps, its exact stub / via-site checks -- so
its escapes slid to within 0.115 of neighbouring balls: legal at 0.1, 5-7 um
inside the 0.12 KiCad grades (`check_drc --clearance-margin 0`: 20
PAD-SEGMENT items at U1 after the fanout alone; KiCad itself reports them).
The default 5 % grading margin hid it; the grader is right to have one, the
generator was wrong. Overrides now price the balls raise-only, as qfn_fanout
already does.

Checks (glasgow_revC's U30, 0.8 mm / 0.35 mm balls, a temp copy carrying
`(clearance 0.12)` on U30):
  1. a dog-bone fanout at --clearance 0.1 --track-width 0.1 lays no track
     or via within 0.12 of a U30 ball of another net, and still escapes.
  2. via_clears_pad_rects prices a pad's override (raise-only).

    python3 tests/test_1317_bga_escape_pad_override.py
"""
import math
import os
import sys
import tempfile
from types import SimpleNamespace

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from copy_board import copy_board                              # noqa: E402
from kicad_parser import parse_kicad_pcb                       # noqa: E402
from run_utils import check as run_check, evidence             # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'glasgow_revC.kicad_pcb')
REF, OVERRIDE, RUN_CLR = 'U30', 0.12, 0.1
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def inject_override(pcb):
    """Give REF a footprint-level (clearance OVERRIDE) in its header."""
    with open(pcb, encoding='utf-8') as f:
        s = f.read()
    i = s.find(f'(property "Reference" "{REF}"')
    j = s.rfind('(footprint ', 0, i)
    k = s.find('(at ', j)
    end = s.find('\n', k)
    assert 0 <= j < k < end < i, 'U30 header not found'
    s = s[:end] + f'\n\t\t(clearance {OVERRIDE})' + s[end:]
    with open(pcb, 'w', encoding='utf-8') as f:
        f.write(s)


def _seg_gap(px, py, r, s):
    ax, ay, bx, by = s.start_x, s.start_y, s.end_x, s.end_y
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - ax - t * dx, py - ay - t * dy) - r - s.width / 2.0


def ball_grazes(before, after):
    """New copper within OVERRIDE of a REF ball of another net, edge to edge."""
    a, b = parse_kicad_pcb(before), parse_kicad_pcb(after)
    old_s = {(s.start_x, s.start_y, s.end_x, s.end_y, s.layer) for s in a.segments}
    old_v = {(v.x, v.y) for v in a.vias}
    balls = [p for p in b.footprints[REF].pads if 'F.Cu' in (p.layers or [])]
    check('the override reached the parsed balls',
          all(abs((p.local_clearance or 0) - OVERRIDE) < 1e-9 for p in balls),
          str({p.local_clearance for p in balls}))
    new_s = [s for s in b.segments if s.layer == 'F.Cu'
             and (s.start_x, s.start_y, s.end_x, s.end_y, s.layer) not in old_s]
    new_v = [v for v in b.vias if (v.x, v.y) not in old_v]
    bad = []
    for p in balls:
        r = max(p.size_x, p.size_y) / 2.0
        for s in new_s:
            if s.net_id != p.net_id:
                g = _seg_gap(p.global_x, p.global_y, r, s)
                if g < OVERRIDE - 1e-4:
                    bad.append(('track', p.pad_number, round(g, 4)))
        for v in new_v:
            if v.net_id != p.net_id:
                g = math.hypot(v.x - p.global_x, v.y - p.global_y) - r - v.size / 2.0
                if g < OVERRIDE - 1e-4:
                    bad.append(('via', p.pad_number, round(g, 4)))
    return bad, len(new_s), len(new_v)


def fanout(tmp):
    print(f'1. a dog-bone fanout of {REF} under a {OVERRIDE} footprint override:')
    evidence(BOARD, 'the glasgow_revC fixture')
    src = os.path.join(tmp, 'in.kicad_pcb')
    copy_board(BOARD, src)
    inject_override(src)
    out = os.path.join(tmp, 'out.kicad_pcb')
    r = run_check([sys.executable, '-X', 'utf8', 'py_router/bga_fanout.py', src,
                   '--component', REF, '--escape-method', 'dogbone',
                   '--clearance', str(RUN_CLR), '--track-width', '0.1',
                   '--via-size', '0.45', '--via-drill', '0.25',
                   '--output', out], accept=True)
    evidence(out, 'the fanned board')
    bad, ns, nv = ball_grazes(src, out)
    log = r.stdout + r.stderr
    esc = [l for l in log.splitlines() if l.startswith('JSON_SUMMARY')]
    check('it drew escapes', ns > 0 and nv > 0, f"{ns} segment(s), {nv} via(s)")
    check(f'no new copper within {OVERRIDE} of a foreign {REF} ball',
          not bad, f"{len(bad)}: {bad[:6]}")
    import json
    summ = json.loads(esc[-1].split(':', 1)[1]) if esc else {}
    check('the fanout still escapes most balls',
          summ.get('escaped', 0) >= 0.9 * max(1, summ.get('requested', 0)),
          f"{summ.get('escaped')}/{summ.get('requested')}")


def via_rect():
    print('2. via_clears_pad_rects prices the override, raise-only:')
    from bga_fanout.geometry import via_clears_pad_rects
    pad = SimpleNamespace(global_x=0.0, global_y=0.0, size_x=0.4, size_y=0.4,
                          rect_rotation=0.0, shape='rect', local_clearance=0.12)
    # ring half 0.15 at x = 0.2 + 0.15 + 0.11: clears 0.1, not 0.12
    x = 0.2 + 0.15 + 0.11
    check('a via 0.11 from a pad overriding 0.12 is refused',
          via_clears_pad_rects(x, 0.0, 0.15, RUN_CLR, [pad]) is False)
    pad.local_clearance = 0.05
    check('an override below the run clearance leaves the run clearance',
          via_clears_pad_rects(0.2 + 0.15 + 0.09, 0.0, 0.15, RUN_CLR, [pad]) is False
          and via_clears_pad_rects(x, 0.0, 0.15, RUN_CLR, [pad]) is True)


def main():
    with tempfile.TemporaryDirectory() as tmp:
        fanout(tmp)
    via_rect()
    if failures:
        print(f"\nFAILED ({len(failures)}): {failures}")
        return 1
    print('\nPASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
