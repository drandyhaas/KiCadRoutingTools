#!/usr/bin/env python3
"""#1703: the intra-pair matcher prices the P/N partner at the pair's own
copper and lets a bump sit at the pair's clearance.

`route_diff --diff-pair-intra-match` meanders the shorter member. A bump away
from the partner starts its entry chamfer on the member's routed centreline,
exactly one pitch (width + gap) from the partner. The matcher priced the
partner at `own half + config.track_width / 2 + P/N clearance` with a strict
`<`, so two things refused every such bump at its root:

  A. a gap EQUAL to the P/N clearance (what the gap floors put a pair at):
     the root measures 0.3499999999999943 against a 0.35 keep-out at width
     0.15 / gap 0.2 / clearance 0.2, and float noise said no;
  B. a pair in a net class narrower than the Default class: with
     --track-width omitted the run's base width is the Default class pair
     width, and the matcher priced both the bump and its partner at it
     while the pair was routed (diff_pair_loop) at its class geometry.

Every coupled run then read `(no-bumps)` and the pair shipped unmatched.
Now the matcher runs each pair at the width and gap it was routed at
(`length_matching.intra_pair_config`, route_diff's `pair_diff_geom`), prices
the partner at its segment's own width, and treats a distance within one
nanometre of the clearance as at it.

Checks:
  1. A (unit): a coupled run at gap == clearance, P 1 mm short, is matched
     with bumps, and no bump comes closer to N than the clearance;
  2. the bump search's own partner check allows a root AT the pitch and
     still refuses one two nanometres inside the clearance (the tolerance
     is one nanometre, not a looser keep-out);
  3. B (route_diff, the in-repo LVDS board, the /CLK pair alone): the pair's
     net class 0.2 / 0.25 under a Default class of width 0.3, flags omitted,
     gets bumps and is matched, as it is under a Default class of 0.2.

    python3 tests/test_1703_intra_pair_partner_pricing.py
"""
import contextlib
import io
import json
import math
import os
import re
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

import length_matching as lm                                   # noqa: E402
from geometry_utils import segment_to_segment_distance         # noqa: E402
from kicad_parser import BoardInfo, Net, PCBData, Segment      # noqa: E402
from routing_config import GridRouteConfig                     # noqa: E402
from run_utils import evidence, tool                           # noqa: E402

failures = []
P, N = 1, 2
W, GAP, CLR = 0.15, 0.2, 0.2
P_Y, N_Y = 105.0, 105.35        # pitch 0.35, which float subtraction reads 0.3499999999999943


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def _seg(x1, y1, x2, y2, net):
    return Segment(start_x=x1, start_y=y1, end_x=x2, end_y=y2, width=W,
                   layer='F.Cu', net_id=net)


def unit_case():
    p = [_seg(105.0, P_Y, 125.0, P_Y, P)]
    n = [_seg(105.0, N_Y, 125.0, N_Y, N)]
    bi = BoardInfo(layers={}, board_bounds=(100.0, 100.0, 130.0, 115.0),
                   copper_layers=['F.Cu', 'B.Cu'])
    pcb = PCBData(board_info=bi, footprints={}, vias=[], pads_by_net={},
                  nets={P: Net(P, '/CK_P'), N: Net(N, '/CK_N')}, segments=p + n)
    cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'], track_width=W, clearance=CLR,
                          diff_pair_gap=GAP)
    cfg.length_match_tolerance = 0.1
    cfg.diff_pair_intra_match = True
    # no pads on the board: the matcher reads the result's own lengths, and
    # N's stub makes it 1 mm longer
    result = {'new_segments': list(p) + list(n), 'new_vias': [], 'n_src_stub_length': 1.0}
    status = {}
    with contextlib.redirect_stdout(io.StringIO()):
        lm.apply_intra_pair_length_matching(result, cfg, pcb, p_net_id=P, n_net_id=N,
                                            status=status)
    check('A: a run at gap == clearance is matched with bumps',
          status.get('status') == 'matched', str(status))
    bumps = [s for s in result['new_segments'] if s.net_id == P]
    gap = min(segment_to_segment_distance(a.start_x, a.start_y, a.end_x, a.end_y,
                                          b.start_x, b.start_y, b.end_x, b.end_y)
              for a in bumps for b in n) - W
    check('A: no bump comes closer to N than the clearance',
          gap >= CLR - 1e-6, f'closest copper gap {gap:.9f}')

    # the partner check itself: a bump root one nanometre inside the
    # clearance is still refused, at the root and at the pitch it is allowed
    def amp_at(dy):
        return lm.get_safe_amplitude_at_point(
            110.0, P_Y + dy, 1.0, 0.0, 0.0, -1.0, 1, 1.0, lm.MIN_AMPLITUDE, 'F.Cu',
            pcb, P, cfg, paired_net_id=N, own_width=W)
    check('the root at the pitch is allowed', amp_at(0.0) > 0, f'{amp_at(0.0)}')
    check('a root 2 nm inside the clearance is refused', amp_at(2e-6) == 0,
          f'{amp_at(2e-6)}')


def route_case(tmp, default_width):
    """The /CLK pair of the in-repo LVDS board alone, in its own net class
    0.2 / 0.25, flags omitted; only the Default class width differs."""
    d = os.path.join(tmp, f'dw{default_width}')
    os.makedirs(d)
    inp, out = os.path.join(d, 'in.kicad_pcb'), os.path.join(d, 'out.kicad_pcb')
    subprocess.run([sys.executable, '-B', tool('copy_board.py'),
                    os.path.join(ROOT, 'kicad_files', 'lvds_converter_dualclk.kicad_pcb'),
                    inp], capture_output=True, check=True, cwd=ROOT)
    evidence(inp, 'the copied LVDS board')

    def cls(name, w):
        return {"name": name, "clearance": 0.2, "track_width": w, "diff_pair_width": w,
                "diff_pair_gap": 0.25, "via_diameter": 0.6, "via_drill": 0.3}
    pro = {"meta": {"filename": "in.kicad_pro", "version": 3},
           "board": {"design_settings": {"rules": {"min_clearance": 0.1,
                                                    "min_track_width": 0.1}}},
           "net_settings": {"classes": [cls("Default", default_width), cls("LVDS", 0.2)],
                            "netclass_patterns": [{"netclass": "LVDS", "pattern": "/CLK*"},
                                                  {"netclass": "LVDS", "pattern": "/DATA*"}]}}
    with open(os.path.join(d, 'in.kicad_pro'), 'w') as fh:
        json.dump(pro, fh, indent=2)
    r = subprocess.run([sys.executable, '-B', tool('route_diff.py'), inp, out,
                        '--nets', '/CLK*', '--diff-pair-intra-match', '-v'],
                       capture_output=True, text=True, cwd=ROOT, timeout=900)
    log = r.stdout + r.stderr
    m = re.search(r'JSON_SUMMARY: (\{.*\})', log)
    summary = json.loads(m.group(1)) if m else {}
    rec = next((x for x in summary.get('intra_pair_matching') or []
                if x.get('pair') == '/CLK'), {})
    runs = next((ln.strip() for ln in log.splitlines() if 'Straight runs' in ln), '')
    added = next((ln.strip() for ln in log.splitlines() if 'bumps added' in ln), '')
    return r.returncode, rec, runs, added, log


def main():
    unit_case()
    with tempfile.TemporaryDirectory() as tmp:
        for dw in (0.3, 0.2):
            code, rec, runs, added, log = route_case(tmp, dw)
            if code != 0 or not rec:
                check(f'B: route_diff ran (Default width {dw})', False,
                      f'exit {code}: ' + log[-400:])
                continue
            check(f'B: Default class width {dw}: the pair gets bumps and is matched',
                  rec.get('matcher_status') == 'matched' and bool(added)
                  and 'no-bumps' not in runs,
                  f"{rec.get('matcher_status')} {rec.get('matcher_delta_mm')}; "
                  f"{added or runs}")
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
