#!/usr/bin/env python3
"""#1834: the QFN stub escape lays no second stub on a pad that already has
its surface escape, and its summary counts those pads.

`qfn_fanout.py` (stub method) run again on its own output laid every stub
again -- tigard U3's 96 segments became 192, then 288, all exact same-net
copies -- and every lap's JSON_SUMMARY read `"escaped": 32` with nothing
saying the pads already were. A re-run with another parameter forked a few
pads into two stubs. Now a pad with same-net track copper that starts on the
pad and reaches outside the part's pad hull or a same-net via
(`pad_has_stub_escape`) is left out before the fan is planned, and the
per-pad keys #1804 added count it as `pads_already_served`.

  1. tigard U3, lap 1 on the tracked board, lap 2 on lap 1's output with the
     same arguments: lap 2 writes nothing, names all 48 pads served, still
     counts the 32 nets escaped, and the board is lap 1's copper exactly.
  2. lap 2 with another --clearance: still nothing laid (no forked stub).
  3. controls, the skip never hides an unserved pad (in-process, on the
     tracked board plus the engine's own first-run stubs): one mid-side pad
     with (a) no stub, (b) a track running inward from it, (c) its straight
     leg cut back to the pad's edge is escaped again -- with exactly the
     stub the first run gave it (the fan angles are ramped over every
     selected pad, served or not) -- and the other 47 are skipped.
  4. pad_has_stub_escape on synthetic copper: a chain that leaves the hull
     through a second track (end to end, or a T-junction) or reaches a via
     inside the hull is served; a track that does not start on the pad, or
     ends on the hull's edge, is not.

    python3 tests/test_1834_qfn_stub_rerun.py
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
QFN = os.path.join(ROOT, 'py_router', 'qfn_fanout.py')
COPY = os.path.join(ROOT, 'py_router', 'copy_board.py')
REF = 'U3'
ARGS = ['--component', REF, '--width', '0.1', '--clearance', '0.1']
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


def seg_key(layer, a, b, net):
    return (layer, net) + tuple(sorted([(round(a[0], 4), round(a[1], 4)),
                                        (round(b[0], 4), round(b[1], 4))]))


def part_copper(path):
    pcb = load(path)
    nets = {p.net_id for p in pcb.footprints[REF].pads if p.net_id}
    return sorted(seg_key(s.layer, (s.start_x, s.start_y), (s.end_x, s.end_y), s.net_id)
                  for s in pcb.segments if s.net_id in nets)


def lap(src, dst, extra=()):
    r = run_check([sys.executable, '-X', 'utf8', QFN, evidence(src), '--output', dst]
                  + ARGS + list(extra), accept=True)
    return summary(r.stdout), r.stdout


def laps(tmp):
    b0 = os.path.join(tmp, 't0.kicad_pcb')
    run_check([sys.executable, '-X', 'utf8', COPY, TIGARD, b0], accept=True)
    b1, b2, b3 = (os.path.join(tmp, f't{i}.kicad_pcb') for i in (1, 2, 3))

    print("1. tigard U3, lap 1 then lap 2 on its output")
    s1, _ = lap(b0, b1)
    c1 = part_copper(b1)
    check("lap 1 escapes every pad", s1.get('failed') == 0
          and s1.get('pads_requested') == s1.get('pads_escaped') == 48
          and s1.get('pads_already_served') == 0 and len(c1) == 96,
          f"{len(c1)} tracks, " + repr({k: s1.get(k) for k in (
              'failed', 'pads_requested', 'pads_escaped', 'pads_already_served')}))
    s2, log2 = lap(b1, b2)
    check("lap 2 lays nothing", 'No fanout tracks generated' in log2
          and 'Writing' not in log2)
    check("lap 2 names every pad served",
          s2.get('pads_already_served') == s2.get('pads_requested') == 48
          and s2.get('pads_escaped') == 0 and s2.get('dropped_pads') == [],
          repr({k: s2.get(k) for k in ('pads_requested', 'pads_escaped',
                                       'pads_already_served', 'dropped_pads')}))
    check("lap 2 still counts the nets escaped",
          s2.get('requested') == s2.get('escaped') == s1.get('requested') == 32
          and s2.get('failed') == 0,
          f"{s2.get('requested')}/{s2.get('escaped')}/{s2.get('failed')}")
    c2 = part_copper(b2)
    check("lap 2's board is lap 1's copper, no track twice",
          c2 == c1 and len(set(c2)) == len(c2), f"{len(c2)} vs {len(c1)} tracks")

    print("2. lap 2 with --clearance 0.15: no forked stub")
    s3, log3 = lap(b1, b3, ['--clearance', '0.15'])
    check("nothing laid", 'No fanout tracks generated' in log3
          and part_copper(b3) == c1 and s3.get('pads_already_served') == 48,
          f"served {s3.get('pads_already_served')}")


def controls():
    print("3. controls: a mid-side pad with no, an inward, or a cut-back stub")
    import qfn_fanout
    from kicad_parser import Segment
    if not hasattr(qfn_fanout, 'stub_pad_counts'):
        check("qfn_fanout.stub_pad_counts exists", False)
        return
    kw = dict(layer='F.Cu', track_width=0.1, clearance=0.1, grid_step=0.1,
              extension=0.05)

    def run(extra_segments):
        pcb = load(TIGARD)
        pcb.segments = list(pcb.segments) + extra_segments
        with contextlib.redirect_stdout(io.StringIO()):
            tracks, _v, failed = qfn_fanout.generate_qfn_fanout(
                pcb.footprints[REF], pcb, **kw)
        return tracks, failed, qfn_fanout.stub_pad_counts(REF), pcb

    first, _f, c0, pcb = run([])
    check("the first run escapes all 48 pads", c0.get('pads_escaped') == 48
          and len(first) == 96, repr(c0))

    def segs(ts):
        return [Segment(start_x=t['start'][0], start_y=t['start'][1],
                        end_x=t['end'][0], end_y=t['end'][1], width=t['width'],
                        layer=t['layer'], net_id=t['net_id']) for t in ts]

    def key(t):
        return seg_key(t['layer'], t['start'], t['end'], t['net_id'])

    # a single-pad-on-part net, on a side, not the outermost pad of it (the
    # outermost one sets the side's fan ramp)
    fp = pcb.footprints[REF]
    by_net = {}
    for p in fp.pads:
        by_net.setdefault(p.net_id, []).append(p)
    cx = sum(p.global_x for p in fp.pads) / len(fp.pads)
    cy = sum(p.global_y for p in fp.pads) / len(fp.pads)
    cands = [ps[0] for nid, ps in by_net.items() if len(ps) == 1 and nid
             and sum(1 for t in first if t['net_id'] == nid) == 2]
    cands.sort(key=lambda p: min(abs(p.global_x - cx), abs(p.global_y - cy)))
    pad = cands[0]
    mine = [t for t in first if t['net_id'] == pad.net_id]
    rest = [t for t in first if t['net_id'] != pad.net_id]
    leg = next(t for t in mine if abs(t['start'][0] - pad.global_x) < 1e-4
               and abs(t['start'][1] - pad.global_y) < 1e-4)
    ux, uy = cx - pad.global_x, cy - pad.global_y
    n = (ux * ux + uy * uy) ** 0.5
    inward = dict(leg, end=(pad.global_x + ux / n * 0.4, pad.global_y + uy / n * 0.4))
    # the straight leg cut back to the pad's own copper edge
    ex, ey = leg['end'][0] - pad.global_x, leg['end'][1] - pad.global_y
    half = (pad.size_x if abs(ex) > abs(ey) else pad.size_y) / 2
    m = (ex * ex + ey * ey) ** 0.5
    cut = dict(leg, end=(pad.global_x + ex / m * half, pad.global_y + ey / m * half))
    for label, extra in (("(a) no stub", []), ("(b) a track inward", [inward]),
                         ("(c) the straight leg cut back to the pad edge", [cut])):
        tracks, failed, c, _pcb = run(segs(rest + extra))
        check(f"{label}: that pad alone is escaped, with the first run's stub",
              sorted(key(t) for t in tracks) == sorted(key(t) for t in mine)
              and failed == [] and c.get('pads_already_served') == 47
              and c.get('pads_escaped') == 1,
              f"{len(tracks)} track(s), {c}")


def synthetic():
    print("4. pad_has_stub_escape on synthetic copper")
    from types import SimpleNamespace as NS
    import qfn_fanout
    if not hasattr(qfn_fanout, 'pad_has_stub_escape'):
        check("pad_has_stub_escape exists", False)
        return
    cu = ['F.Cu', 'B.Cu']

    def mk_pad(x, y):
        return NS(global_x=x, global_y=y, size_x=0.8, size_y=0.3, shape='rect',
                  rect_rotation=0.0, layers=['F.Cu'], net_id=5, pad_type='smd',
                  drill=0.0)
    # a 4x4 mm square of pads; the pad under test on the right edge
    pads = [mk_pad(x, y) for x in (-2.0, 2.0) for y in (-1.0, 0.0, 1.0)]
    pad = pads[4]                                    # (2.0, 0.0)
    hull = qfn_fanout.part_pad_hull(NS(pads=pads))

    def seg(a, b, w=0.1):
        return NS(start_x=a[0], start_y=a[1], end_x=b[0], end_y=b[1], width=w,
                  layer='F.Cu', net_id=5)
    has = qfn_fanout.pad_has_stub_escape
    check("a stub out of the hull is served",
          has(pad, [seg((2.0, 0.0), (2.6, 0.0))], [], hull, 'F.Cu', cu))
    check("a stub ending on the hull's edge is not",
          not has(pad, [seg((2.0, 0.0), (2.4, 0.0))], [], hull, 'F.Cu', cu))
    check("a track inward is not",
          not has(pad, [seg((2.0, 0.0), (0.5, 0.0))], [], hull, 'F.Cu', cu))
    check("a chain that leaves the hull through its second track is served",
          has(pad, [seg((2.0, 0.0), (1.0, 0.0)), seg((1.0, 0.0), (1.0, -3.0))],
              [], hull, 'F.Cu', cu))
    check("so is one joined at a T",
          has(pad, [seg((2.0, 0.0), (1.0, 0.0)), seg((1.5, 0.0), (1.5, -3.0))],
              [], hull, 'F.Cu', cu))
    check("a closed outline of tracks round the pad that crosses the hull is not",
          not has(pad, [seg((2.0, 0.0), (2.6, -0.1)), seg((2.6, -0.1), (2.6, 0.2)),
                        seg((2.6, 0.2), (2.0, 0.1)), seg((2.0, 0.1), (2.0, 0.0))],
                  [], hull, 'F.Cu', cu))
    check("a track that does not start on the pad is not",
          not has(pad, [seg((1.0, 0.5), (1.0, -3.0))], [], hull, 'F.Cu', cu))
    via = NS(x=1.0, y=0.0, size=0.45, drill=0.2, layers=['F.Cu', 'B.Cu'],
             via_type='through', net_id=5)
    check("a track to a via inside the hull is served",
          has(pad, [seg((2.0, 0.0), (1.0, 0.0))], [via], hull, 'F.Cu', cu))
    check("a track on another layer is not",
          not has(pad, [NS(start_x=2.0, start_y=0.0, end_x=3.0, end_y=0.0,
                           width=0.1, layer='B.Cu', net_id=5)], [], hull, 'F.Cu', cu))


def main():
    tmp = tempfile.mkdtemp(prefix='t1834_')
    try:
        laps(tmp)
        controls()
        synthetic()
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    if failures:
        print(f"\nFAIL: {len(failures)} check(s): {failures}")
        return 1
    print("\nPASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
