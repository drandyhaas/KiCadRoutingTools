#!/usr/bin/env python3
"""#1804: the QFN via-drop gives no second via to a pad that has its escape,
and its summary counts PADS beside the per-net counts.

`qfn_fanout.py --escape-method underpad` run again on its own output used to
drill again for pads it had escaped: its #479 reuse test reached only as far
as the ladder's first rung, so a via one grid step further out (or moved by
the snap) was not reused and a second via was laid one rung further out,
and the reuse branch wrote its bridging track again on every lap. GND vias
on the tigard board went 28, 36, 44, 48 over four laps, every lap's
JSON_SUMMARY reading the same four per-net numbers. Now a pad whose copper a
same-net via overlaps, or that one same-net track joins to such a via
(`pad_has_via_escape`), is left out before the escape runs; a pad stacked
with another whose copper a via of the same run overlaps (an exposed pad's
rectangle and the thermal hole at its centre) is served by it, not dropped.

  1. tigard U3 GND, lap 1 on the tracked board: every perimeter GND pad gets
     a via, nothing is dropped, and the per-pad keys add up.
  2. lap 2 on lap 1's output: no via and no track ("No fanout tracks
     generated"), every pad named served, the net still counted escaped, and
     the board has the same GND copper with no track written twice.
  3. control, the skip never hides an unserved pad: lap 1's board with one
     perimeter pad's via and stub taken off -- the engine re-escapes exactly
     that pad and skips the rest.
  4. glasgow U1 GND, lap 1: the exposed pad's rectangles, stacked on the
     thermal hole at its centre, are served by that hole's via -- no pad is
     dropped and GND is not failed (it was, with one pad dropped at every
     rung of the clearance ladder, which therefore ran to its last rung).
  5. pad_has_via_escape's negative controls: a track from the pad to a point
     with no via, and a blind via that does not reach the escape layer.

    python3 tests/test_1804_qfn_underpad_rerun.py
"""
import contextlib
import io
import json
import os
import re
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from run_utils import check as run_check, evidence             # noqa: E402

TIGARD = os.path.join(ROOT, 'kicad_files', 'tigard.kicad_pcb')
GLASGOW = os.path.join(ROOT, 'kicad_files', 'glasgow_revC.kicad_pcb')
QFN = os.path.join(ROOT, 'py_router', 'qfn_fanout.py')
COPY = os.path.join(ROOT, 'py_router', 'copy_board.py')
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def summary(out):
    m = re.search(r'^JSON_SUMMARY: (.*)$', out, re.M)
    return json.loads(m.group(1)) if m else {}


def load(path):
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        return parse_kicad_pcb(evidence(path))


def gnd_copper(path, ref):
    pcb = load(path)
    gid = next(p.net_id for p in pcb.footprints[ref].pads if p.net_name == 'GND')
    segs = [(s.layer,) + tuple(sorted([(round(s.start_x, 4), round(s.start_y, 4)),
                                       (round(s.end_x, 4), round(s.end_y, 4))]))
            for s in pcb.segments if s.net_id == gid]
    vias = [(round(v.x, 4), round(v.y, 4)) for v in pcb.vias if v.net_id == gid]
    return pcb, gid, segs, vias


def lap(src, dst, ref):
    r = run_check([sys.executable, '-X', 'utf8', QFN, evidence(src), '--output', dst,
                   '--component', ref, '--nets', 'GND', '--escape-method', 'underpad',
                   '--allow-via-in-pad'], accept=True)
    return summary(r.stdout), r.stdout


def tigard(tmp):
    b0 = os.path.join(tmp, 't0.kicad_pcb')
    run_check([sys.executable, '-X', 'utf8', COPY, TIGARD, b0], accept=True)
    b1, b2 = os.path.join(tmp, 't1.kicad_pcb'), os.path.join(tmp, 't2.kicad_pcb')

    print("1. tigard U3 GND, lap 1")
    s1, _log = lap(b0, b1, 'U3')
    pcb1, gid, segs1, vias1 = gnd_copper(b1, 'U3')
    from kicad_parser import pad_is_plated_through
    perim = [p for p in pcb1.footprints['U3'].pads
             if p.net_id == gid and not pad_is_plated_through(p)
             and max(p.size_x, p.size_y) < 1.0]
    stub_ends = {e for s in segs1 for e in s[1:]}
    via_set = set(vias1)
    served = [p for p in perim
              if any((round(p.global_x, 4), round(p.global_y, 4)) in s[1:]
                     and (s[1] in via_set or s[2] in via_set) for s in segs1)
              or (round(p.global_x, 4), round(p.global_y, 4)) in via_set]
    check("each of the 10 perimeter GND pads has its own via",
          len(perim) == 10 and len(served) == 10, f"{len(served)} of {len(perim)}")
    check("nothing dropped, GND escaped",
          s1.get('failed') == 0 and s1.get('dropped_pads') == [], repr(s1.get('dropped_pads')))
    check("pads add up: requested = escaped + served + dropped",
          s1.get('pads_requested') == 28 and s1.get('pads_escaped') == 28
          and s1.get('pads_already_served') == 0
          and s1.get('vias_placed') == len(vias1),
          f"{ {k: s1.get(k) for k in ('pads_requested', 'pads_escaped', 'pads_already_served', 'vias_placed')} }")
    check("no track of lap 1 is written twice", len(segs1) == len(set(segs1)),
          f"{len(segs1) - len(set(segs1))} copies")

    print("2. lap 2 on lap 1's output")
    s2, log2 = lap(b1, b2, 'U3')
    _pcb2, _g, segs2, vias2 = gnd_copper(b2, 'U3')
    check("no via and no track", 'No fanout tracks generated' in log2
          and s2.get('vias_placed') == 0, f"vias_placed={s2.get('vias_placed')}")
    check("every pad is named served",
          s2.get('pads_already_served') == s2.get('pads_requested') == 28
          and s2.get('pads_escaped') == 0, f"{s2.get('pads_already_served')} of {s2.get('pads_requested')}")
    check("the net is still counted escaped",
          s2.get('requested') == 1 and s2.get('escaped') == 1 and s2.get('failed') == 0,
          f"{s2.get('requested')}/{s2.get('escaped')}/{s2.get('failed')}")
    check("the board's GND copper is lap 1's, no track twice",
          sorted(segs2) == sorted(segs1) and sorted(vias2) == sorted(vias1)
          and len(segs2) == len(set(segs2)),
          f"{len(vias2)} vias / {len(segs2)} tracks vs {len(vias1)} / {len(segs1)}")

    print("3. control: one perimeter pad's via taken off -- that pad alone is re-escaped")
    import qfn_fanout
    pcb = load(b1)
    fp = pcb.footprints['U3']
    vk = (round(perim[0].global_x, 4), round(perim[0].global_y, 4))
    stub = [s for s in pcb.segments if s.net_id == gid and vk in (
        (round(s.start_x, 4), round(s.start_y, 4)), (round(s.end_x, 4), round(s.end_y, 4)))]
    far = {(round(x, 4), round(y, 4)) for s in stub
           for x, y in ((s.start_x, s.start_y), (s.end_x, s.end_y))} - {vk}
    gone = [v for v in pcb.vias if v.net_id == gid
            and ((round(v.x, 4), round(v.y, 4)) in far or (round(v.x, 4), round(v.y, 4)) == vk)]
    check("the control removes one stub and one via", len(gone) == 1 and len(stub) <= 1,
          f"{len(stub)} stub(s), {len(gone)} via(s)")
    pcb.segments = [s for s in pcb.segments if s not in stub]
    pcb.vias = [v for v in pcb.vias if v not in gone]
    with contextlib.redirect_stdout(io.StringIO()):
        _t, vias, failed = qfn_fanout.generate_qfn_fanout(
            fp, pcb, net_filter=['GND'], layer='F.Cu', escape_method='underpad',
            allow_via_in_pad=True, track_width=0.2, clearance=0.2, grid_step=0.05)
    c = qfn_fanout.underpad_pad_counts('U3')
    check("that pad gets a via again and the other 27 are skipped",
          len(vias) == 1 and failed == [] and c.get('pads_already_served') == 27
          and c.get('pads_escaped') == 1,
          f"{len(vias)} via(s), failed {failed}, {c}")


def glasgow(tmp):
    print("4. glasgow U1 GND, lap 1: stacked exposed-pad rectangles")
    b0 = os.path.join(tmp, 'g0.kicad_pcb')
    run_check([sys.executable, '-X', 'utf8', COPY, GLASGOW, b0], accept=True)
    s, log = lap(b0, os.path.join(tmp, 'g1.kicad_pcb'), 'U1')
    check("no pad dropped and GND not failed",
          s.get('dropped_pads') == [] and s.get('failed') == 0
          and s.get('unescaped_nets') == [],
          f"dropped {s.get('dropped_pads')}, failed {s.get('failed')}")
    check("the clearance ladder stops at the first rung that drops nothing",
          re.search(r'escapes \d+ of them \(0 still dropped\)', log) is not None)
    check("every pad is accounted",
          s.get('pads_requested') == s.get('pads_escaped') == 36, repr(
              {k: s.get(k) for k in ('pads_requested', 'pads_escaped', 'vias_placed')}))


def negatives():
    print("5. pad_has_via_escape's negative controls")
    from types import SimpleNamespace as NS
    from qfn_fanout import pad_has_via_escape
    pad = NS(global_x=0.0, global_y=0.0, size_x=0.3, size_y=0.8, shape='rect',
             rect_rotation=0.0, layers=['F.Cu'], net_id=5, pad_type='smd', drill=0.0)
    cu = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']
    via = NS(x=1.0, y=0.0, size=0.45, drill=0.2, layers=['F.Cu', 'B.Cu'],
             via_type='through', net_id=5)
    stub = NS(start_x=0.0, start_y=0.0, end_x=1.0, end_y=0.0, layer='F.Cu', net_id=5)
    check("a stub to a via is served", pad_has_via_escape(pad, [stub], [via], 'B.Cu', cu))
    loose = NS(start_x=0.0, start_y=0.0, end_x=2.0, end_y=0.0, layer='F.Cu', net_id=5)
    check("a stub to no via is not", not pad_has_via_escape(pad, [loose], [via], 'B.Cu', cu))
    inner = NS(start_x=0.0, start_y=0.0, end_x=1.0, end_y=0.0, layer='In1.Cu', net_id=5)
    check("a track on a layer the pad has no copper on is not",
          not pad_has_via_escape(pad, [inner], [via], 'B.Cu', cu))
    blind = NS(x=1.0, y=0.0, size=0.45, drill=0.2, layers=['F.Cu', 'In1.Cu'],
               via_type='blind', net_id=5)
    check("a blind via that stops above the escape layer is not",
          not pad_has_via_escape(pad, [stub], [blind], 'B.Cu', cu))
    inpad = NS(x=0.0, y=0.2, size=0.3, drill=0.15, layers=['F.Cu', 'B.Cu'],
               via_type='through', net_id=5)
    check("a via in the pad is served", pad_has_via_escape(pad, [], [inpad], 'B.Cu', cu))


def main():
    tmp = tempfile.mkdtemp(prefix='t1804_')
    try:
        tigard(tmp)
        glasgow(tmp)
        negatives()
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    if failures:
        print(f"\nFAIL: {len(failures)} check(s): {failures}")
        return 1
    print("\nPASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
