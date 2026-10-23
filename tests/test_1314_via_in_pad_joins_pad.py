#!/usr/bin/env python3
"""#1314: the net_queries track graph joins a via-in-pad to its pad.

`_net_track_graph` gave each pad and each via per-layer hub nodes that met
only TRACK nodes inside them. A via centred in an SMD pad whose tracks leave
on an inner layer therefore never reached its pad: on imx6ulz (gp14 H22)
`board_score --length-groups` read DRAM_DQ3 and DRAM_DQ7 as "not joined by
track copper" and left the whole DRAM_LDQ group ungraded, though
check_connected and KiCad both measured the nets. A via whose centre lies in
a pad (the landing test a track end gets) now joins it on every layer both
carry.

Checks:
  1. Synthetic, 4 layers: two F.Cu pads, a via centred in each, the track
     on In2.Cu only. The net is joined and the pin-pair path is the track
     plus each via's barrel from F.Cu to In2.Cu. A via 1 mm away from the
     pad, touching nothing, does not join it.
  2. routed_output.kicad_pcb, three nets whose only track leaves a via-in-pad
     on another layer (witnessed): check_connected calls them connected,
     net_pads_joined and both match-length modes measure them, and
     `board_score --length-groups` grades their group (`length` ran, the
     group passes) instead of listing it ungraded.

    python3 tests/test_1314_via_in_pad_joins_pad.py
"""
import contextlib
import io
import json
import os
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from kicad_parser import (BoardInfo, Net, Pad, PCBData, Segment,  # noqa: E402
                          StackupLayer, Via, parse_kicad_pcb)
from net_queries import (net_match_length, net_pads_joined,  # noqa: E402
                         pin_pair_path_length, _pad_holds_point)
from run_utils import evidence  # noqa: E402

NET = 7
LAYERS = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']
BOARD = os.path.join(ROOT, 'kicad_files', 'routed_output.kicad_pcb')
NETS = ['Net-(U2A-DATA_0)', 'Net-(U2A-DATA_2)', 'Net-(U2A-CLK)']
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def pad(x, ref):
    return Pad(pad_number='1', net_id=NET, net_name='N', shape='rect',
               global_x=x, global_y=0.0, local_x=0.0, local_y=0.0,
               size_x=0.6, size_y=0.6, layers=['F.Cu'], drill=0.0,
               pad_type='smd', component_ref=ref)


def board(vias):
    stack = []
    for i, name in enumerate(LAYERS):
        stack.append(StackupLayer(name=name, layer_type='copper', thickness=0.035))
        if i < len(LAYERS) - 1:
            stack.append(StackupLayer(name=f'd{i}', layer_type='core',
                                      thickness=0.5))
    info = BoardInfo(layers={i: n for i, n in enumerate(LAYERS)},
                     copper_layers=list(LAYERS))
    info.stackup = stack
    pa, pb = pad(0.0, 'U1'), pad(10.0, 'U2')
    seg = Segment(start_x=0.0, start_y=0.0, end_x=10.0, end_y=0.0, width=0.2,
                  layer='In2.Cu', net_id=NET)
    pcb = PCBData(board_info=info, nets={NET: Net(net_id=NET, name='N')},
                  footprints={}, segments=[seg],
                  vias=[Via(x=x, y=0.0, size=0.3, drill=0.15,
                            layers=['F.Cu', 'B.Cu'], net_id=NET) for x in vias],
                  pads_by_net={NET: [pa, pb]})
    return pcb, pa, pb


def quiet(fn, *a):
    with contextlib.redirect_stdout(io.StringIO()), \
            contextlib.redirect_stderr(io.StringIO()):
        return fn(*a)


def main():
    # 1. Synthetic.
    pcb, pa, pb = board([0.0, 10.0])
    per_step = pcb.get_via_barrel_length('F.Cu', 'B.Cu') / 3.0
    want = 10.0 + 2 * 2 * per_step          # F.Cu -> In2.Cu at each end
    path = pin_pair_path_length(pcb, NET, pa, pb)
    check('fixture: the stackup gives the via a barrel', per_step > 0,
          f'{per_step:.4f} mm per layer step')
    check('two vias-in-pad with the track on In2.Cu: the pads are joined',
          net_pads_joined(pcb, NET))
    check('the pin-pair path is the track plus each barrel F.Cu -> In2.Cu',
          path is not None and abs(path - want) < 1e-6,
          f'{path} vs {want:.4f}')
    check('both match-length modes measure it',
          net_match_length(pcb, NET, 'total') is not None
          and net_match_length(pcb, NET, 'pin_pair') is not None)
    pcb, pa, pb = board([0.0, 11.0])
    check('a via 1 mm off the pad, touching nothing, does not join it',
          not net_pads_joined(pcb, NET)
          and pin_pair_path_length(pcb, NET, pa, pb) is None)

    # 2. A real board.
    src = quiet(parse_kicad_pcb, evidence(BOARD, 'routed_output board'))
    ids = {n.name: i for i, n in src.nets.items()}
    witnessed = []
    for name in NETS:
        nid = ids[name]
        segs = [s for s in src.segments if s.net_id == nid]
        # a pad holding a via, and no track END of the net inside it on the
        # pad's own layers: the via is that pad's only link to the net
        vip = [p for p in src.pads_by_net[nid]
               if any(v.net_id == nid and _pad_holds_point(p, v.x, v.y)
                      for v in src.vias)
               and not any(s.layer in p.layers and (
                   _pad_holds_point(p, s.start_x, s.start_y)
                   or _pad_holds_point(p, s.end_x, s.end_y)) for s in segs)]
        witnessed.append(bool(vip))
    check('witness: each net has a pad whose only link is a via-in-pad',
          all(witnessed), str(dict(zip(NETS, witnessed))))
    r = subprocess.run([sys.executable, '-X', 'utf8', 'py_router/check_connected.py',
                        BOARD, '--nets'] + NETS, cwd=ROOT, capture_output=True,
                       text=True, encoding='utf-8', errors='replace')
    check('check_connected calls the three nets connected',
          'ALL NETS FULLY CONNECTED' in r.stdout, r.stdout.strip()[-200:])
    measured = {name: (net_pads_joined(src, ids[name]),
                       net_match_length(src, ids[name], 'total'),
                       net_match_length(src, ids[name], 'pin_pair'))
                for name in NETS}
    check('net_pads_joined and both modes measure them',
          all(j and t is not None and p is not None
              for j, t, p in measured.values()),
          '; '.join(f'{n}: {v}' for n, v in measured.items()))

    with tempfile.TemporaryDirectory() as tmp:
        groups = os.path.join(tmp, 'groups.json')
        with open(groups, 'w') as fh:
            json.dump({'G': {'nets': NETS, 'tolerance_mm': 1000.0,
                             'mode': 'pin_pair'}}, fh)
        out = os.path.join(tmp, 'score.json')
        # board_score's exit code is the board's blocking verdict (this board
        # has unrouted nets), so the run is judged by its output instead.
        s = subprocess.run([sys.executable, '-X', 'utf8', 'py_tools/board_score.py',
                            BOARD, '--length-groups', groups, '--json', out],
                           cwd=ROOT, capture_output=True, text=True,
                           encoding='utf-8', errors='replace')
        check('board_score ran to a verdict',
              'Traceback' not in s.stdout + s.stderr and 'SCORE_JSON' in s.stdout,
              (s.stdout + s.stderr)[-300:])
        doc = json.load(open(evidence(out, 'board_score json')))
        ln = doc['components']['length']
        check('board_score grades the group instead of leaving it ungraded',
              ln.get('ran') is True and ln['groups']['G'].get('pass') is True
              and 'length' not in doc.get('ungraded', []), json.dumps(ln)[:300])

    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
