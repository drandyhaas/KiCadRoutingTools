#!/usr/bin/env python3
"""smooth_board.py ROUTED FANOUT NETS OUT -- the octolinear smoother (#536)
once, over a routed board's LANES.

The probe braids of a descent run with the smoother off (`BRAID_SMOOTH=0`,
replan.braid_run probe=True): it never moves a via and it was ~3.7 s of a
~6 s two-lane local braid at K51, because it validates every shortcut
against the whole board's copper. So a board assembled from probes carries
unsmoothed lanes -- legal, graded the same (open, drc, vias), just more
segments. This runs the same smoother once over every lane of the run's
nets (the copper the routed board carries beyond the fanout board's), with
the fanout copper protected as input copper, and writes the board.

usage: smooth_board.py ROUTED.kicad_pcb FANOUT.kicad_pcb NET,NET,... OUT.kicad_pcb
"""
import contextlib
import io
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
from kicad_parser import parse_kicad_pcb  # noqa: E402
from pcb_modification import smooth_octolinear_chains  # noqa: E402
import source_realize as sr  # noqa: E402
import replan  # noqa: E402  lane_items, copy_board
import rules as _rules  # noqa: E402


def main(argv):
    if len(argv) != 5:
        sys.exit(__doc__)
    routed, fanout, nets_csv, out = argv[1:]
    _rules.install_defaults()
    names = [n for n in nets_csv.split(',') if n]
    pcb_r = parse_kicad_pcb(routed)
    pcb_f = parse_kicad_pcb(fanout)
    byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb_r.nets.items()}
    names = [nm for nm in names if nm in byname]
    lanes = replan.lane_items(pcb_r, pcb_f, names, byname)
    kids = {byname[nm][0] for nm in names}
    res = [{'new_segments': list(lanes[nm][0])} for nm in names]
    before = sum(len(r['new_segments']) for r in res)
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(buf):
        _n, _nets, _rm, _addl, stt = smooth_octolinear_chains(
            res, pcb_r, kids, clearance=_rules.active().clearance, keep_input_copper=True)
    after = sum(len(r['new_segments']) for r in res)
    # the board text: every lane segment of the run's nets out, the smoothed ones in
    n2n = {i: n.name for i, n in pcb_r.nets.items()}
    txt = open(routed, encoding='utf-8').read()
    old = [s for nm in names for s in lanes[nm][0]]
    txt, n_s = sr.remove_segments_from_content(txt, old, n2n)
    if n_s != len(old):
        sys.exit(f'smooth_board: stripped {n_s} of {len(old)} lane segments -- refusing to write')
    add = []
    for k, nm in enumerate(names):
        nid = byname[nm][0]
        for s in res[k]['new_segments']:
            add.append(f'  (segment (start {s.start_x:.4f} {s.start_y:.4f}) (end {s.end_x:.4f} {s.end_y:.4f}) '
                       f'(width {s.width}) (layer "{s.layer}") (net {nid}))\n')
    i = txt.rstrip().rfind(')')
    txt = txt[:i] + ''.join(add) + txt[i:]
    with open(out, 'w', encoding='utf-8') as f:
        f.write(txt)
    replan.fp.copy_pro(routed, out)
    replan.ship_vias.stamp(out, 'smooth')
    print(f'smooth_board: {len(names)} nets, lane segments {before} -> {after} '
          f'({stt.get("spans", 0)} spans, -{stt.get("saved_mm", 0):.2f} mm) -> {os.path.basename(out)}')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
