#!/usr/bin/env python3
"""#1391 follow-up: a named-pattern length-match group takes members from the
board's own copper, but only members that copper actually ROUTES.

A pattern group pulls in every board net with copper and no result in this
run, measured by `_seed_group_members_from_board` and recorded protected
('length-matched', #521). A member that is not routed -- it failed and kept
residual copper (an escape stub, a kept spur) -- used to be seeded at its stub
length and recorded protected too, so a later step's rip ladders and
--force-reroute refused the one net that still needed routing. Seeding is now
gated on the authoritative zone/fill-aware connectivity grade.

An 8-net bus, group /BIT[0-7], the call routing /BIT[1-7]:

  1. Precondition: /BIT0 carries a stub off one pad and its pads are NOT
     connected (graded by check_net_connectivity), so the stub is the
     residual-copper case and not a route.
  2. Seeding, unit level: the stubbed /BIT0 is not seeded; the same board with
     /BIT0 routed by an earlier step (control) seeds it.
  3. End to end, stub: /BIT0 is not recorded protected and not in the group's
     verdicts, while the members this call routed ARE recorded (so the record
     itself works).
  4. End to end, control: /BIT0 routed by an earlier step joins the group,
     measured from the board, and is recorded protected.

    python3 tests/test_1391_incomplete_member_not_protected.py   (about 1 min)
"""
import contextlib
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS)

from run_utils import evidence                                   # noqa: E402
from kicad_parser import parse_kicad_pcb                         # noqa: E402
from check_connected import check_net_connectivity               # noqa: E402
from length_matching import _seed_group_members_from_board       # noqa: E402
from protected_nets import read_protected_nets                   # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'interf_u_unrouted_placed.kicad_pcb')
NET = '/BIT0'
GROUP = ['--length-match-group', '/BIT[0-7]', '--length-match-tolerance', '0.2',
         '--no-bga-zones']
FAILS = []


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def parse(path):
    with contextlib.redirect_stdout(io.StringIO()), \
            contextlib.redirect_stderr(io.StringIO()):
        return parse_kicad_pcb(path)


def route(src, out, nets, *extra):
    r = subprocess.run([sys.executable, '-X', 'utf8', 'py_router/route.py', src, out,
                        '--nets', nets] + list(extra), cwd=ROOT,
                       capture_output=True, text=True, encoding='utf-8', errors='replace')
    return r.returncode, r.stdout + r.stderr


def with_stub(src, dst, pad):
    """`src` plus a 1 mm same-net spur leaving `pad` toward nothing."""
    text = open(src, encoding='utf-8').read()
    x, y = pad.global_x, pad.global_y
    layer = next((l for l in pad.layers if l.endswith('.Cu') and '*' not in l),
                 'F.Cu')
    seg = (f'\t(segment\n\t\t(start {x} {y})\n\t\t(end {x + 1.0} {y})\n'
           f'\t\t(width 0.25)\n\t\t(layer "{layer}")\n'
           f'\t\t(net "{NET}")\n'
           f'\t\t(uuid "13911391-0000-4000-8000-000000001391")\n\t)\n')
    i = text.rstrip().rfind(')')
    open(dst, 'w', encoding='utf-8').write(text[:i] + seg + text[i:])
    pro = src[:-len('.kicad_pcb')] + '.kicad_pro'
    if os.path.isfile(pro):
        shutil.copyfile(pro, dst[:-len('.kicad_pcb')] + '.kicad_pro')
    return dst


def net_state(pcb):
    nid = next(i for i, n in pcb.nets.items() if n.name == NET)
    segs = [s for s in pcb.segments if s.net_id == nid]
    vias = [v for v in pcb.vias if v.net_id == nid]
    pads = pcb.pads_by_net.get(nid, [])
    zones = [z for z in pcb.zones if z.net_id == nid]
    res = check_net_connectivity(nid, segs, vias, pads, zones, tolerance=0.02,
                                 pcb_data=pcb)
    return nid, segs, pads, res['connected']


def group_record(js_path):
    try:
        doc = json.load(open(js_path, encoding='utf-8'))
    except (OSError, ValueError):
        return {}
    return (doc.get('length_match_groups') or [{}])[0]


def main():
    tmp = tempfile.mkdtemp(prefix='t1391i_')
    try:
        b = os.path.join(tmp, 'b.kicad_pcb')
        subprocess.run([sys.executable, '-X', 'utf8', 'py_router/copy_board.py',
                        BOARD, b], cwd=ROOT, capture_output=True)
        evidence(b, 'the copied board')
        pads = parse(b).pads_by_net[next(i for i, n in parse(b).nets.items()
                                         if n.name == NET)]
        stub = with_stub(b, os.path.join(tmp, 'stub.kicad_pcb'), pads[0])

        # 1. Precondition.
        pcb_stub = parse(stub)
        _, segs, npads, conn = net_state(pcb_stub)
        check('1. the stubbed net has copper and >= 2 pads, and is NOT connected',
              len(segs) == 1 and len(npads) >= 2 and not conn,
              f'segs={len(segs)} pads={len(npads)} connected={conn}')

        # Control board: /BIT0 routed by an earlier step.
        early = os.path.join(tmp, 'early.kicad_pcb')
        rc, _ = route(b, early, NET)
        pcb_early = parse(early)
        _, esegs, _, econn = net_state(pcb_early)
        check('1. control precondition: the earlier step connected the net',
              rc == 0 and esegs and econn, f'rc={rc} connected={econn}')

        # 2. Seeding, unit level.
        names = [NET, '/BIT1']
        with contextlib.redirect_stdout(io.StringIO()):
            s_stub = _seed_group_members_from_board(names, {'/BIT1'}, pcb_stub)
            s_early = _seed_group_members_from_board(names, {'/BIT1'}, pcb_early)
        check('2. a stub that does not connect the pads is not seeded',
              NET not in s_stub, str(sorted(s_stub)))
        check('2. control: a routed earlier-step member is seeded',
              NET in s_early and s_early[NET]['route_length'] > 0,
              str(sorted(s_early)))

        # 3. End to end over the stub.
        o3 = os.path.join(tmp, 'o3.kicad_pcb')
        j3 = os.path.join(tmp, 'o3.json')
        rc, log = route(stub, o3, '/BIT[1-7]', *GROUP, '--json-out', j3)
        evidence(j3, 'the stub run\'s JSON summary')
        prot = read_protected_nets(o3[:-len('.kicad_pcb')] + '.kicad_pro')
        grp = group_record(j3)
        check('3. the stubbed net is not recorded protected',
              rc == 0 and NET not in prot, f'rc={rc} {NET}={prot.get(NET)}')
        check('3. the stubbed net has no member verdict',
              NET not in (grp.get('member_status') or {}),
              str(sorted(grp.get('member_status') or {})))
        check('3. the run says why the stubbed net was left out',
              'left out' in log and NET in log.split('left out', 1)[1][:400])
        routed_prot = sorted(n for n, v in prot.items() if v == 'length-matched')
        check('3. the members this call routed ARE recorded protected',
              len(routed_prot) >= 2, str(routed_prot))

        # 4. End to end, control.
        o4 = os.path.join(tmp, 'o4.kicad_pcb')
        j4 = os.path.join(tmp, 'o4.json')
        rc, log = route(early, o4, '/BIT[1-7]', *GROUP, '--json-out', j4)
        evidence(j4, 'the control run\'s JSON summary')
        prot = read_protected_nets(o4[:-len('.kicad_pcb')] + '.kicad_pro')
        check('4. control: an earlier-step routed member is recorded protected',
              rc == 0 and prot.get(NET) == 'length-matched',
              f'rc={rc} {NET}={prot.get(NET)}')
        check('4. control: it is measured from the board',
              'measured from existing board copper' in log and NET in log)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print('FAILED: ' + ', '.join(FAILS) if FAILS else 'ALL OK')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
