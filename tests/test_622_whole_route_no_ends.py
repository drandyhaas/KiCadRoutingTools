#!/usr/bin/env python3
"""A round of the whole route solves the lanes its fanout gave ends, and leaves the rest open (whole_route.no_ends,
with_ends).

  python3 tests/test_622_whole_route_no_ends.py

A fanout can choose a lane no escape: sdi_mipi_conv's SDI bus after the DDR3 bus and its terminations left SDI_D18's
ball a via in pad with no stub. The round's solve asserted on it in braid.endpoints ('no free stub end'), and so did
every last resort after it, each on the run's whole net list: the bus laid nothing. Pinned on a generated bus (synth_bus,
SU1 to SD1, nets SYN00.. and pairs SYP0/SYP1):

1. no_ends names a net with no copper, and not one with a stub at each part, nor one with no copper that the fanout's
   plan sidecar gives ends (the solve reads those off the plan, not the board) -- but it does name a net the sidecar
   ran (its 'nets') and laid no end for, stubbed or not: that copper is the field's dog-bone, not a tooth (SDI_D13 and
   SDI_D19, inside U11 on its far face, which the solve's frame refuses for the whole round);
2. with_ends leaves a lane out whole: a pair whose one leg has no end leaves with its other leg, and the rest are
   kept in their order.
"""
import contextlib
import io
import json
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
AWX = os.path.join(ROOT, 'awx')
sys.path.insert(0, AWX)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
with contextlib.redirect_stdout(io.StringIO()):
    import whole_route as wr  # noqa: E402
    from kicad_parser import parse_kicad_pcb  # noqa: E402
    from kicad_writer import add_tracks_and_vias_to_pcb  # noqa: E402

BAD = []


def check(ok, what):
    print(f'  {"ok" if ok else "FAIL"}: {what}')
    if not ok:
        BAD.append(what)


with tempfile.TemporaryDirectory() as tmp:
    raw = os.path.join(tmp, 'bus.kicad_pcb')
    r = subprocess.run([sys.executable, 'synth_bus.py', raw, '--k', '8', '--pairs', '2', '--pattern', 'shuffle',
                        '--seed', '1'], cwd=AWX, capture_output=True, text=True)
    if r.returncode != 0 or not os.path.isfile(raw):
        raise SystemExit(f'BROKEN TEST: synth_bus wrote no board -- {(r.stderr or r.stdout)[-300:]}')
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(raw)
    nid = {n.name: i for i, n in pcb.nets.items()}
    want = ['SYN00', 'SYN01', 'SYN02', 'SYP0_P', 'SYP0_N', 'SYP1_P', 'SYP1_N']
    if not all(n in nid for n in want):
        raise SystemExit(f'BROKEN TEST: the generated bus lacks {[n for n in want if n not in nid]}')
    # a stub 1 mm out of each ball toward the other part, on F: SYN00, SYP0_P, SYP1_P and SYP1_N; SYP0_N and SYN01 and
    # SYN02 bare
    tracks = []
    for n in ('SYN00', 'SYP0_P', 'SYP1_P', 'SYP1_N'):
        for p in pcb.nets[nid[n]].pads:
            dx = 1.0 if p.component_ref == 'SU1' else -1.0
            tracks.append({'start': (p.global_x, p.global_y), 'end': (p.global_x + dx, p.global_y), 'width': 0.12,
                           'layer': 'F.Cu', 'net_id': nid[n]})
    bench = os.path.join(tmp, 'fo.kicad_pcb')
    with contextlib.redirect_stdout(io.StringIO()):
        add_tracks_and_vias_to_pcb(raw, bench, tracks, [], [], net_id_to_name={i: n for n, i in nid.items()})
    # the fanout's sidecar gives SYN02 ends (its plan's), none to SYN01
    json.dump({'ends': {'SYN02': [[0, 0], [1, 1]]}}, open(os.path.join(tmp, 'fo.plan.json'), 'w'))

    print('1. no_ends: a bare net with no plan, not a stubbed one nor a bare one the plan gives ends')
    got = wr.no_ends(bench, want, 'SD1')
    check(got == ['SYN01', 'SYP0_N'], f'the nets with no end: {got} (want SYN01, SYP0_N)')
    # (the case reaches what it pins: with no sidecar, SYN02 has none either)
    os.remove(os.path.join(tmp, 'fo.plan.json'))
    got2 = wr.no_ends(bench, want, 'SD1')
    check(got2 == ['SYN01', 'SYN02', 'SYP0_N'], f'with no plan sidecar, SYN02 has no end either: {got2}')
    # a sidecar that RAN every net and laid ends for SYN02 and SYP1 only: SYN00 and SYP0_P have stubs, but no tooth
    # its fanout laid -- the field's copper, not an end
    ran = {'nets': want, 'ends': {n: [[0, 0], [1, 1]] for n in ('SYN02', 'SYP1_P', 'SYP1_N')}}
    json.dump(ran, open(os.path.join(tmp, 'fo.plan.json'), 'w'))
    got3 = wr.no_ends(bench, want, 'SD1')
    check(got3 == ['SYN00', 'SYN01', 'SYP0_P', 'SYP0_N'],
          f'a net the sidecar ran and laid no end for has none, stubbed or not: {got3}')
    json.dump({'ends': {'SYN02': [[0, 0], [1, 1]]}}, open(os.path.join(tmp, 'fo.plan.json'), 'w'))

    print('2. with_ends: a lane left out whole, the rest kept in order')
    lanes = {'SYN00': ('SYN00',), 'SYN01': ('SYN01',), 'SYN02': ('SYN02',), 'SYP0': ('SYP0_P', 'SYP0_N'),
             'SYP1': ('SYP1_P', 'SYP1_N')}
    keep, bad, mates = wr.with_ends(bench, want, 'SD1', lanes)
    check(keep == ['SYN00', 'SYN02', 'SYP1_P', 'SYP1_N'], f'the lanes kept, in order: {keep}')
    check(bad == ['SYN01', 'SYP0_N'], f'named with no end: {bad}')
    check(mates == ['SYP0_P'], f'SYP0_P left out with its leg SYP0_N: {mates}')
    keep_, bad_, mates_ = wr.with_ends(bench, ['SYN00', 'SYP1_P', 'SYP1_N'], 'SD1', lanes)
    check((keep_, bad_, mates_) == (['SYN00', 'SYP1_P', 'SYP1_N'], [], []), 'every lane with ends: all kept')

if BAD:
    print('FAIL: ' + '; '.join(BAD))
    sys.exit(1)
print('PASS: a round solves the lanes with ends, the plan\'s or the board\'s, and leaves the rest open, a pair whole')
