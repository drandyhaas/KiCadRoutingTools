#!/usr/bin/env python3
"""#1332: a qfn_fanout stub that reaches a same-net pad ends on it.

Every clearance test in the stub fan skips same-net pads, so a 45-degree fan
could clip a same-net pad -- a pin's own decoupling cap beside it -- and run
on past it. On steppenprobe (gp14 M17) U1.64's +1V8 fan clipped C11.1's
roundrect corner and ended 0.27 mm beyond it; route.py keeps input copper,
so after routing check_weird reported a 1.2 mm dangling tail and
check_complete refused DONE. `_end_stubs_at_same_net_pads` now ends each
stub at its deepest contact with the first same-net pad its copper
overlaps, and closes a sub-floor corner web into the pad with the connector
close_soft_joints would add (#416).

Checks:
  1. Unit, on steppenprobe's geometry (a 0.15 mm fan grazing a 0.875 x 0.95
     roundrect corner): the stub ends in contact with the pad and no longer
     runs past it; at a 0.05 mm floor the web is fine and no closing segment
     is laid, at a 0.15 mm floor it is a neck and one is laid, ending inside
     the pad. The same fan past a FOREIGN-net pad, and past a same-net pad it
     never touches, is left alone.
  2. End to end on kit-dev-coldfire U102 (LQFP-100, whose +3.3V and VDDPLL
     fans clip their decoupling caps): the CLI says which stubs it ended,
     every one of them ends in contact with that pad, check_weird finds no
     dangling end at any of their tips and no narrow-pad-joint, and the run
     still escapes every pad, DRC-clean.

    python3 tests/test_1332_qfn_stub_ends_at_same_net_pad.py
"""
import contextlib
import io
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
from types import SimpleNamespace

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from kicad_parser import parse_kicad_pcb                       # noqa: E402
from check_drc import point_to_pad_distance, segment_pad_gap   # noqa: E402
from pcb_modification import terminal_web_neck_exact           # noqa: E402
from qfn_fanout import _end_stubs_at_same_net_pads             # noqa: E402
from qfn_fanout.types import FanoutStub                        # noqa: E402
from run_utils import check as run_check, evidence             # noqa: E402
from synth import make_pad                                     # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'kit-dev-coldfire-xilinx_5213.kicad_pcb')
TW = 0.15
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def stub_and_pads(net_cap):
    """steppenprobe U1.64 -> C11.1, with C11.1 on net `net_cap`."""
    own = make_pad(1, 119.0, 77.9925, ref='U1', num='64', net_name='+1V8',
                   size_x=1.55, size_y=0.3)
    cap = make_pad(net_cap, 117.0103, 79.9175, ref='C11', num='1',
                   net_name='+1V8' if net_cap == 1 else 'OTHER',
                   size_x=0.875, size_y=0.95, shape='roundrect',
                   roundrect_rratio=0.25)
    stub = FanoutStub(pad=own, pad_pos=(119.0, 77.9925),
                      corner_pos=(118.0538, 77.9925), stub_end=(116.3, 79.8),
                      side='left')
    return stub, own, cap


def run_unit(net_cap, floor, cap_shift=(0.0, 0.0)):
    stub, own, cap = stub_and_pads(net_cap)
    cap.global_x += cap_shift[0]
    cap.global_y += cap_shift[1]
    pcb = SimpleNamespace(segments=[], pads_by_net={1: [own] + ([cap] if net_cap == 1 else [])})
    n = _end_stubs_at_same_net_pads([stub], [own, cap], TW,
                                    lambda a, b, nid: False, pcb, floor)
    return n, stub, cap


def main():
    # 1. Unit.
    n, stub, cap = run_unit(1, 0.05)
    tip_d = point_to_pad_distance(stub.stub_end[0], stub.stub_end[1], cap)
    check('the grazing fan is ended on the same-net pad', n == 1
          and stub.joined_pad == 'C11.1', f'n={n} joined={stub.joined_pad}')
    check('its tip is in contact with the pad', tip_d < TW / 2,
          f'tip {stub.stub_end} is {tip_d:.4f} mm from the copper')
    check('and it no longer runs past the pad (old tip x=116.30)',
          stub.stub_end[0] > 116.573, f'tip x {stub.stub_end[0]:.4f}')
    n, stub, cap = run_unit(1, TW)
    check('this corner contact is a full web even at a 0.15 mm floor '
          '(KiCad-exact erosion): no closing segment', n == 1
          and stub.join_end is None, str(stub.join_end))
    # The same fan with the pad moved 0.03 mm away from it: the overlap is
    # ~0.008 mm, a sub-floor web at 0.1 mm.
    away = (0.7176 * 0.03, 0.6964 * 0.03)
    n, stub, cap = run_unit(1, 0.1, cap_shift=away)
    tip_d = point_to_pad_distance(stub.stub_end[0], stub.stub_end[1], cap)
    check('a shallow graze is a neck at 0.1 mm: a closing segment is laid',
          n == 1 and tip_d < TW / 2 and stub.join_end is not None,
          f'tip {tip_d:.4f} mm off the copper, join_end={stub.join_end}')
    if stub.join_end is not None:
        legs = [(stub.pad_pos, stub.corner_pos), (stub.corner_pos, stub.stub_end),
                (stub.stub_end, stub.join_end)]
        view = SimpleNamespace(
            segments=[SimpleNamespace(start_x=a[0], start_y=a[1], end_x=b[0],
                                      end_y=b[1], width=TW, layer='F.Cu', net_id=1)
                      for a, b in legs],
            pads_by_net={1: [stub.pad, cap]})
        neck = [terminal_web_neck_exact(view, 1, 'F.Cu', x, y, 0.1)
                for x, y in (stub.stub_end, stub.join_end)]
        check('...after which the joint is a full web (KiCad-exact erosion)',
              neck == [False, False], str(neck))
    n, stub, _cap = run_unit(1, 0.05, cap_shift=away)
    check('...and at a 0.05 mm floor the same graze needs none',
          n == 1 and stub.join_end is None, str(stub.join_end))
    n, stub, _cap = run_unit(2, TW)
    check('a FOREIGN-net pad in the same place is not joined (clearance owns it)',
          n == 0 and stub.stub_end == (116.3, 79.8) and stub.joined_pad is None)
    n, stub, _cap = run_unit(1, TW, cap_shift=(0.3, 0.3))
    check('a same-net pad the fan never touches is left alone',
          n == 0 and stub.stub_end == (116.3, 79.8))

    # 2. End to end.
    tmp = tempfile.mkdtemp(prefix='t1332_')
    try:
        out = os.path.join(tmp, 'u102.kicad_pcb')
        r = run_check([sys.executable, '-X', 'utf8', 'py_router/qfn_fanout.py',
                       evidence(BOARD, 'coldfire board'), '--component', 'U102',
                       '--nets', '*', '!GND*', '--width', str(TW),
                       '--clearance', '0.15', '-o', out], accept=True)
        evidence(out, 'qfn_fanout output')
        line = next((l for l in r.stdout.splitlines() if 'JSON_SUMMARY' in l), '')
        js = json.loads(line.split('JSON_SUMMARY:', 1)[1]) if line else {}
        m = re.search(r'Same-net pads: (\d+) stub\(s\) end on a same-net pad', r.stdout)
        joined = int(m.group(1)) if m else 0
        check('precondition: the run ends stubs on same-net pads', joined >= 3,
              f'{joined} stub(s)')
        check('every pad still escapes',
              js.get('failed') == 0 and js.get('escaped') == js.get('requested'),
              f"{js.get('escaped')}/{js.get('requested')}")
        with contextlib.redirect_stdout(io.StringIO()):
            src, pcb = parse_kicad_pcb(BOARD), parse_kicad_pcb(out)
        before = {(s.start_x, s.start_y, s.end_x, s.end_y) for s in src.segments}
        new = [s for s in pcb.segments
               if (s.start_x, s.start_y, s.end_x, s.end_y) not in before]
        names = re.findall(r'(\S+) at (\w+)\.(\w+)', r.stdout.split('Same-net pads:', 1)[-1]
                           .splitlines()[0]) if m else []
        tips_ok, tips = True, []
        for net, ref, num in names:
            pad = next(p for p in pcb.footprints[ref].pads if p.pad_number == num)
            mine = [s for s in new if s.net_id == pad.net_id]
            ends = [(s.end_x, s.end_y) for s in mine]
            near = min((point_to_pad_distance(x, y, pad) for x, y in ends),
                       default=math.inf)
            tips.append(min(ends, key=lambda e: point_to_pad_distance(e[0], e[1], pad)))
            tips_ok &= near < TW / 2
        check('each ended stub finishes in contact with its pad', tips_ok and names,
              f'{len(names)} stub(s)')
        # The same property read off the copper alone, whatever the run says:
        # a new segment whose copper reaches a same-net pad of ANOTHER part
        # ends in contact with that pad (nothing runs on past it).
        past = []
        for s in new:
            for p in pcb.pads_by_net.get(s.net_id, []):
                if p.component_ref == 'U102' or (
                        not (p.drill and p.drill > 0) and s.layer not in p.layers):
                    continue
                if segment_pad_gap(s.start_x, s.start_y, s.end_x, s.end_y, p) \
                        < s.width / 2 - 1e-6 and \
                        point_to_pad_distance(s.end_x, s.end_y, p) >= s.width / 2:
                    past.append(f'{pcb.nets[s.net_id].name} past '
                                f'{p.component_ref}.{p.pad_number}')
        check('no new segment runs on past a same-net pad it reaches', not past,
              '; '.join(past[:3]))
        w = subprocess.run([sys.executable, '-X', 'utf8', 'py_router/check_weird.py',
                            out, '--max-print', '0'], cwd=ROOT, capture_output=True,
                           text=True, encoding='utf-8', errors='replace')
        graded = 'WEIRD' in w.stdout
        dangling_at_tip = [l.strip() for l in w.stdout.splitlines()
                           if 'dangling' in l and any(
                               f'({x:.3f}, {y:.3f})' in l for x, y in tips)]
        check('check_weird ran and finds no dangling end at an ended stub',
              w.returncode in (0, 1) and graded and not dangling_at_tip,
              '; '.join(dangling_at_tip[:2]))
        check('check_weird finds no narrow-pad-joint',
              'narrow-pad-joint: 0' in w.stdout or 'narrow-pad-joint:' not in w.stdout)
        d = subprocess.run([sys.executable, '-X', 'utf8', 'py_router/check_drc.py',
                            out, '--clearance-margin', '0.1'], cwd=ROOT,
                           capture_output=True, text=True, encoding='utf-8',
                           errors='replace')
        check('the output is DRC-clean', 'NO DRC VIOLATIONS FOUND' in d.stdout,
              d.stdout.strip().splitlines()[-1] if d.stdout.strip() else '')
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
