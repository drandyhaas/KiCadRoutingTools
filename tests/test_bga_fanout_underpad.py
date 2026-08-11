#!/usr/bin/env python3
"""Regression test for issue #122: the under-pad grid escape method.

  python3 tests/test_bga_fanout_underpad.py

The under-pad method routes each BGA signal ball with a via-in-pad and a path
UNDER the pad field on inner layers, escaping dense, fully-populated arrays the
channel router cannot. This test runs it through the engine entry point
(generate_bga_fanout, escape_method='underpad') on the dense ulx3s 22x22 BGA and
checks:
  - all signal balls escape (0 failed),
  - power/plane nets are skipped (not fanned),
  - every signal ball gets exactly one via-in-pad,
  - a bottom-side (B.Cu) BGA escapes too (layer roles swap correctly).

Uses kicad_files/ulx3s.kicad_pcb; skips cleanly if absent.
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # #522
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # #522

from kicad_parser import parse_kicad_pcb
from bga_fanout import generate_bga_fanout

BOARD = os.path.join(ROOT, "kicad_files", "ulx3s.kicad_pcb")
LAYERS = ["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"]
# plane_drop off: this test asserts SIGNAL-escape geometry (plane balls stay
# bare); the #424 plane-ball drop pass is covered by test_bga_fanout_plane_drop.
PARAMS = dict(track_width=0.12, clearance=0.1, via_size=0.35, via_drill=0.2,
              escape_method='underpad', plane_drop='off')


def main():
    print("=" * 60)
    print("Issue #122 under-pad escape regression test")
    print("=" * 60)
    if not os.path.exists(BOARD):
        print(f"  [SKIP] board not present: {BOARD}")
        return 0

    checks = []
    pcb = parse_kicad_pcb(BOARD)
    fp = pcb.footprints["U1"]
    tracks, vias, _vr, failed = generate_bga_fanout(fp, pcb, layers=LAYERS, **PARAMS)

    checks.append(("tracks were generated", len(tracks) > 0))
    checks.append(("no balls failed to escape", len(failed) == 0))
    # ulx3s U1 has ~204 real signal nets; all should escape. Outer balls go on
    # the BGA layer with NO via, so the via count is < the escaped count.
    escaped_nets = {t['net_id'] for t in tracks}
    checks.append(("escaped most signal nets (>180)", len(escaped_nets) > 180))
    checks.append(("some balls escape via-less on the BGA layer (outer ring)",
                   0 < len(vias) < len(escaped_nets)))
    via_nets = {v['net_id'] for v in vias}
    checks.append(("power/plane nets not fanned (GND absent)",
                   all(pcb.nets.get(nid) is None or pcb.nets[nid].name != 'GND'
                       for nid in via_nets)))
    # Every via is via-in-pad (on a copper-spanning via) at a BGA ball.
    checks.append(("all vias span the stack", all(
        v.get('layers') and v['layers'][0] == 'F.Cu' for v in vias)))

    # Bottom-side BGA: flip U1 to B.Cu, escape should still fully succeed.
    pcb2 = parse_kicad_pcb(BOARD)
    fp2 = pcb2.footprints["U1"]
    fp2.layer = 'B.Cu'
    t2, v2, _v2r, f2 = generate_bga_fanout(fp2, pcb2, layers=LAYERS, **PARAMS)
    checks.append(("bottom-side BGA escapes (0 failed)", len(f2) == 0 and len(t2) > 0))
    # The via-less edge escape must now live on B.Cu, not F.Cu.
    bcu_edge = any(t['layer'] == 'B.Cu' for t in t2)
    checks.append(("bottom-side edge escape uses B.Cu", bcu_edge))

    checks += _q7_drill_floor_reads_the_board()

    for name, ok in checks:
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}")
    n_pass = sum(1 for _, ok in checks if ok)
    print(f"\n{n_pass}/{len(checks)} checks passed")
    print("=" * 60)
    return 0 if n_pass == len(checks) else 1


def _q7_drill_floor_reads_the_board():
    """The underpad escape's drill floor was a constant, and the board unread.

    `_via_site_conflict` spaced every drill at the flat
    routing_defaults.HOLE_TO_HOLE_CLEARANCE, and grepping bga_fanout/underpad.py
    for board_floor / board_constraint / min_hole_to_hole / resolve_hole_clearance
    returned NOTHING -- the module never read the board at all. So a board
    declaring min_hole_to_hole 0.4 got its BGA underpad drills spaced at 0.2:
    the D9/D11 substitute-a-constant class, in the direction that ignores a
    board asking for MORE. qfn_fanout is the sibling engine and already reads
    it; check_join.py:118-122 is the board-first AND raise-only model.

    Measured on ulx3s U1 (plane_drop at its 'auto' DEFAULT -- the rest of this
    file pins it 'off', and with it off `_via_site_conflict` never governs a
    site here, so the floor is inert and this gap is invisible):

        declared        before          after
        (nothing)   224 vias 0.3657   224 vias 0.3657   <- control, identical
        0.4         224 vias 0.3657   224 vias 0.6000   <- board ignored/obeyed
        0.10        224 vias 0.3657   224 vias 0.6000 + fab-pin disclosure

    A declared 0.4 costs NO escapes; only the spacing moves.
    """
    import io
    import json
    import math
    import shutil
    import contextlib
    import tempfile
    out = []
    if not os.path.exists(BOARD):
        return out

    # plane_drop left at its 'auto' default on purpose (see the docstring).
    params = {k: v for k, v in PARAMS.items() if k != 'plane_drop'}

    def run(h2h):
        with tempfile.TemporaryDirectory() as tmp:
            b = os.path.join(tmp, 'u.kicad_pcb')
            shutil.copyfile(BOARD, b)
            if h2h is not None:
                with open(os.path.splitext(b)[0] + '.kicad_pro', 'w',
                          encoding='utf-8') as f:
                    json.dump({'board': {'design_settings': {
                        'rules': {'min_hole_to_hole': h2h}}}}, f)
            pcb = parse_kicad_pcb(b)
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                _t, vias, _vr, failed = generate_bga_fanout(
                    pcb.footprints["U1"], pcb, layers=LAYERS, **params)
            gap = min((math.hypot(a['x'] - c['x'], a['y'] - c['y'])
                       - (a.get('drill', 0.2) + c.get('drill', 0.2)) / 2
                       for i, a in enumerate(vias)
                       for c in vias[i + 1:]), default=None)
            return len(vias), len(failed), gap, buf.getvalue()

    base_n, _bf, base_gap, base_said = run(None)
    decl_n, decl_f, decl_gap, decl_said = run(0.4)
    sub_n, _sf, sub_gap, sub_said = run(0.10)

    # The defect, in one line: the board's own rule was not honoured.
    out.append(("#Q7 a declared min_hole_to_hole 0.4 spaces the drills",
                decl_gap is not None and decl_gap >= 0.4 - 1e-9))
    out.append(("#Q7 ...and it says the board is where that came from",
                "from the board's own" in decl_said))
    # It must not buy the spacing by dropping escapes.
    out.append(("#Q7 ...at no cost in escaped balls",
                decl_n == base_n and decl_f == 0))
    # Raise-only: a sub-fab declaration is pinned UP, and disclosed.
    out.append(("#Q7 a sub-fab 0.10 is pinned to the fab floor, not obeyed",
                sub_gap is not None and sub_gap >= 0.2 - 1e-9
                and "below the" in sub_said))
    # CONTROL: a board declaring nothing must be byte-identical to before.
    out.append(("#Q7 a board declaring nothing is unchanged",
                base_gap is not None and abs(base_gap - 0.3657) < 0.01
                and base_n == 224 and 'ole-to-hole' not in base_said))
    return out


if __name__ == "__main__":
    sys.exit(main())
