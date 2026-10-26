#!/usr/bin/env python3
"""#1432: the length measure reproduces the KiCad release that grades the board.

check_drc's `.kicad_dru` length window (#1313) was measured as kicad-cli 10.0.0
measures a net, and later 10.0 releases measure differently -- by up to about
one via barrel per via, so a net near a window limit graded on the other side
of the one kicad-cli applied. Each change is now gated on the release that made
it, and the release is the installed kicad-cli's (newest modelled without one):

  10.0.2  a via is charged half of each inner copper layer that ends its span;
  10.0.4  a via joins every track that ends inside its pad, not only at its
          centre, a via no track ends at is charged nothing, and a run is cut
          at the via pad;
  10.0.5  a via's span takes in the side layer of the pad connected to it,
          and arcs merge into runs;
  10.0.7  same-layer tracks merge where tracks of other layers also end;
  every   a run that ends at a pad's anchor is cut where it leaves the pad's
          outline polygon (a circle pad is at least 16 inscribed sides).

The issue's repro on kicad_files/routed_output.kicad_pcb: DATA_13 measures
15.1494 mm in 10.0.0 and 15.3782 mm in 10.0.7 (a 15.25 max passes one, fails
the other), BE_0 18.8847 and 18.8453 (an 18.86 max the other way round).

Checks:
  * the release selection: forced, installed, newer or older than modelled,
    none installed;
  * synthetic nets, each isolating one change, with the length worked out by
    hand for every release that measures it differently;
  * every net kicad-cli 10.0.0, 10.0.2, 10.0.4, 10.0.5, 10.0.6 and 10.0.7
    reported for three routed boards and for tests/fixtures/1432/
    length_cases.kicad_pcb, one net per case above
    (tests/fixtures/1432/kicad_length_actuals.json), equal at its 4 decimals
    with the measure forced to that release -- an arc net within 5 um, since
    an arc is measured on the parser's chords;
  * the issue's two verdict flips, through check_drc, for 10.0.0 and 10.0.7;
  * when a kicad-cli in the modelled range is installed: every net of
    routed_output against its live `length_out_of_range` report, measured as
    the installed release (no release forced).

    python3 tests/test_1432_kicad_release_length.py
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

from check_drc import run_drc                                   # noqa: E402
from copy_board import copy_board                               # noqa: E402
from kicad_parser import (BoardInfo, Footprint, Net, Pad, PCBData,  # noqa: E402
                          Segment, StackupLayer, Via, parse_kicad_pcb)
import net_queries                                              # noqa: E402
from net_queries import net_copper_length, net_pad_to_die_length  # noqa: E402
from run_utils import evidence                                  # noqa: E402

RO = os.path.join(ROOT, 'kicad_files', 'routed_output.kicad_pcb')
ACTUALS = os.path.join(TESTS_DIR, 'fixtures', '1432', 'kicad_length_actuals.json')
RELEASES = ('10.0.0', '10.0.2', '10.0.4', '10.0.5', '10.0.6', '10.0.7')
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def quiet(fn, *a, **k):
    out = io.StringIO()
    with contextlib.redirect_stdout(out), contextlib.redirect_stderr(out):
        return fn(*a, **k), out.getvalue()


def same4(got, want):
    """Equal at four decimals as KiCad prints them (printf: the double's exact
    value, so an exact half-nm tie rounds to even, not up)."""
    return f'{got:.4f}' == f'{float(want):.4f}'


# --- the release selection ----------------------------------------------------
def selection():
    real = net_queries.installed_kicad_release
    try:
        for have, want, why in (('10.0.0', '10.0.0', 'the installed release'),
                                ('10.0.6', '10.0.6', 'one between two changes'),
                                ('10.1.0', net_queries.LENGTH_MEASURE_NEWEST,
                                 'newer: the newest modelled'),
                                ('9.0.4', net_queries.LENGTH_MEASURE_OLDEST,
                                 'older: the oldest modelled'),
                                (None, net_queries.LENGTH_MEASURE_NEWEST,
                                 'no kicad-cli: the newest modelled')):
            net_queries.installed_kicad_release = lambda have=have: have
            got = net_queries.length_measure_release()
            note = net_queries.length_measure_note()
            check(f'installed {have}: measured as {want} ({why})',
                  got == want and f'KiCad {want} measures' in note
                  and ((have in (None, want)) == ('installed' not in note)), note)
        net_queries.installed_kicad_release = lambda: '10.0.0'
        net_queries.use_length_measure_release('10.0.7')
        check('a forced release wins over the installed one, and the note '
              'names the installed one',
              net_queries.length_measure_release() == '10.0.7'
              and 'installed kicad-cli is 10.0.0' in net_queries.length_measure_note())
        net_queries.use_length_measure_release(None)
        check('forcing None goes back to the installed release',
              net_queries.length_measure_release() == '10.0.0')
        check('an explicit release argument wins over everything',
              net_queries.length_measure_release('10.0.4') == '10.0.4')
    finally:
        net_queries.installed_kicad_release = real
        net_queries.use_length_measure_release(None)


# --- synthetic nets -----------------------------------------------------------
CU, CORE = 0.035, 0.5
LAYERS = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']
F_IN1 = CU + CORE + CU                  # F.Cu .. In1.Cu, both copper layers whole
F_IN1_HALF = CU + CORE + CU / 2         # ... with half of In1.Cu (10.0.2)
I1_I2 = CU + CORE + CU
I1_I2_HALF = CU / 2 + CORE + CU / 2
STACK = 4 * CU + 3 * CORE


def board(footprint_layer=None):
    stack = []
    for i, n in enumerate(LAYERS):
        stack.append(StackupLayer(name=n, layer_type='copper', thickness=CU))
        if i < 3:
            stack.append(StackupLayer(name=f'd{i}', layer_type='core', thickness=CORE))
    info = BoardInfo(layers=dict(enumerate(LAYERS)), copper_layers=list(LAYERS))
    info.stackup = stack
    fps = {}
    if footprint_layer:
        fps['U1'] = Footprint(reference='U1', footprint_name='X', x=0.0, y=0.0,
                              rotation=0.0, layer=footprint_layer)
    return PCBData(board_info=info, nets={1: Net(net_id=1, name='N')},
                   footprints=fps, segments=[], vias=[], pads_by_net={})


def seg(layer, x0, y0, x1, y1, uuid=''):
    return Segment(start_x=x0, start_y=y0, end_x=x1, end_y=y1, width=0.1,
                   layer=layer, net_id=1, uuid=uuid)


def via(x=0.0, y=0.0):
    return Via(x=x, y=y, size=0.3, drill=0.15, layers=['F.Cu', 'B.Cu'], net_id=1)


def pad(shape='rect', kind='smd', layers=('F.Cu',), sx=0.5, sy=0.5, x=0.0, y=0.0):
    return Pad(pad_number='1', net_id=1, net_name='N', shape=shape, global_x=x,
               global_y=y, local_x=x, local_y=y, size_x=sx, size_y=sy,
               layers=list(layers), drill=0.0, pad_type=kind, component_ref='U1')


def net_length(release, segs, vias=(), pads=(), fp_layer=None):
    pcb = board(fp_layer)
    pcb.segments, pcb.vias = list(segs), list(vias)
    pcb.pads_by_net = {1: list(pads)}
    return net_copper_length(pcb, 1, release=release)


def per_release(label, segs, vias, pads, wants, fp_layer=None, tol=1e-6):
    """`wants`: {first release of a value: length}; every release in RELEASES
    takes the value of the newest key at or below it."""
    keys = sorted(wants, key=net_queries._release_key)
    for rel in RELEASES:
        k = [x for x in keys
             if net_queries._release_key(x) <= net_queries._release_key(rel)][-1]
        got = net_length(rel, segs, vias, pads, fp_layer)
        check(f'{rel}: {label}', abs(got - wants[k]) < tol,
              f'{got:.6f} vs {wants[k]:.6f}')


def synthetic():
    per_release('via-in-pad, track on In1.Cu: F.Cu to In1.Cu, half of In1.Cu from 10.0.2',
                [seg('In1.Cu', 0, 0, 5, 0)], [via()], [pad()],
                {'10.0.0': 5.0 + F_IN1, '10.0.2': 5.0 + F_IN1_HALF})
    per_release('tracks on In1.Cu and In2.Cu: half of each inner layer from 10.0.2',
                [seg('In1.Cu', 0, 0, 5, 0), seg('In2.Cu', 0, 0, 5, 0)], [via()], [],
                {'10.0.0': 10.0 + I1_I2, '10.0.2': 10.0 + I1_I2_HALF})
    per_release('a track ending inside the via pad but off its centre joins its '
                'layer and is cut at the pad from 10.0.4',
                [seg('In1.Cu', 0, 0, 5, 0), seg('In2.Cu', 0.04, 0, 5, 0)], [via()], [],
                {'10.0.0': 5.0 + 4.96, '10.0.4': 10.0 + I1_I2_HALF})
    per_release('a track ending outside the via pad adds no layer',
                [seg('In1.Cu', 0, 0, 5, 0), seg('In2.Cu', 0.2, 0, 5, 0)], [via()], [],
                {'10.0.0': 5.0 + 4.8})
    per_release('a via no track ends at: the span its tracks touch (one layer: '
                'to the far end) before 10.0.4, nothing and a cut run after',
                [seg('F.Cu', 0.05, 0, 5, 0)], [via()], [],
                {'10.0.0': 4.95 + STACK, '10.0.4': 5.0})
    th = pad('circle', 'thru_hole', ('*.Cu',), 0.6, 0.6)
    per_release("a through-hole pad joins its footprint's side (B.Cu) from 10.0.5",
                [seg('In1.Cu', 0, 0, 5, 0)], [via()], [th],
                {'10.0.0': 5.0, '10.0.5': 5.0 + CU / 2 + CORE + CU + CORE + CU},
                fp_layer='B.Cu')
    # an arc (chords sharing one uuid) after a straight track that ends inside
    # the pad: merged into one run from 10.0.5, so the run is cut at the pad
    # outline; before, the straight piece lies wholly inside the pad and stays
    p1 = pad('rect', 'smd', ('F.Cu',), 1.0, 1.0)
    arc = [seg('F.Cu', 0.3, 0, 0.6, 0.1, 'arc1'), seg('F.Cu', 0.6, 0.1, 0.9, 0, 'arc1')]
    y = 0.1 * 0.2 / 0.3
    per_release('an arc merges into a run from 10.0.5',
                [seg('F.Cu', 0, 0, 0.3, 0, 'st1')] + arc, [], [p1],
                {'10.0.0': 0.3 + 2 * math.hypot(0.3, 0.1),
                 '10.0.5': (math.hypot(0.5, y) + math.hypot(0.1, 0.1 - y)
                            + math.hypot(0.3, 0.1))}, tol=2e-6)
    # two F.Cu tracks meeting at a via that an In1.Cu track also leaves: one run
    # from 10.0.7, then cut where it leaves the pad its first track ends at
    j = [seg('F.Cu', 0, 0, 0.3, 0.3), seg('F.Cu', 0.3, 0.3, 3, 0.3),
         seg('In1.Cu', 0.3, 0.3, 0.3, 5)]
    apart = math.hypot(0.3, 0.3) + 2.7 + 4.7
    per_release('same-layer tracks merge where another layer also ends from 10.0.7',
                j, [via(0.3, 0.3)], [p1],
                {'10.0.0': apart + F_IN1, '10.0.2': apart + F_IN1_HALF,
                 '10.0.7': math.hypot(0.5, 0.3) + 2.5 + 4.7 + F_IN1_HALF})

    # the pad-outline cut, every release
    p = pad('rect', 'smd', ('F.Cu',), 0.64, 0.54)
    y = 0.2 * (3 - 0.32) / (3 - 0.2)
    per_release('a bend inside the pad it ends at is cut at the pad outline',
                [seg('F.Cu', 3, 0, 0.2, 0.2), seg('F.Cu', 0.2, 0.2, 0, 0)], [], [p],
                {'10.0.0': math.hypot(3 - 0.32, y) + math.hypot(0.32, y)}, tol=2e-6)
    c = pad('circle', 'smd', ('F.Cu',), 0.41, 0.41)
    r = 0.205
    ring = [(r * math.cos(math.radians(11.25 + 22.5 * k)),
             -r * math.sin(math.radians(11.25 + 22.5 * k))) for k in range(16)]
    p0, q1 = (3.0, 0.0), (0.1, 0.1)

    def cut(hit):
        return math.hypot(p0[0] - hit[0], p0[1] - hit[1]) + math.hypot(*hit)
    poly = None
    for i in range(16):
        (ax, ay), (bx, by) = ring[i], ring[(i + 1) % 16]
        d = (q1[0] - p0[0]) * (by - ay) - (q1[1] - p0[1]) * (bx - ax)
        t = ((ax - p0[0]) * (by - ay) - (ay - p0[1]) * (bx - ax)) / d
        u = ((ax - p0[0]) * (q1[1] - p0[1]) - (ay - p0[1]) * (q1[0] - p0[0])) / d
        if 0 <= t <= 1 and 0 <= u <= 1:
            poly = cut((p0[0] + t * (q1[0] - p0[0]), p0[1] + t * (q1[1] - p0[1])))
            break
    dx, dy = q1[0] - p0[0], q1[1] - p0[1]
    qa, qb, qc = dx * dx + dy * dy, 2 * p0[0] * dx, p0[0] ** 2 - r * r
    tc = (-qb - math.sqrt(qb * qb - 4 * qa * qc)) / (2 * qa)
    circle = cut((p0[0] + tc * dx, p0[1] + tc * dy))
    got = net_length('10.0.0', [seg('F.Cu', 3, 0, 0.1, 0.1), seg('F.Cu', 0.1, 0.1, 0, 0)],
                     [], [c])
    check('a circle pad outline is the inscribed 16-gon, not the circle',
          poly is not None and abs(got - poly) < 2e-6 and abs(poly - circle) > 1e-6,
          f'{got:.6f}; 16-gon {poly:.6f}, true circle {circle:.6f}')
    per_release('a run that starts inside the pad it ends at is cut from its start',
                [seg('F.Cu', 0.2, 0, 3, 0), seg('F.Cu', 3, 0, 3, 2),
                 seg('F.Cu', 3, 2, 0, 2), seg('F.Cu', 0, 2, 0, 0)], [],
                [pad('rect', 'smd', ('F.Cu',), 1.0, 1.0)], {'10.0.0': 10.0})


def via_pad_search_cost():
    """The connected-pad search of a net with N vias and N pads rejects a far
    pad before it builds its layer set: expand_pad_layers runs O(N) times, not
    N x N (deterministic call count, not timing)."""
    n = 300
    pcb = board()
    pcb.pads_by_net = {1: [pad('rect', 'smd', ('F.Cu',), 0.5, 0.5, x=2.0 * k)
                           for k in range(n)]}
    pcb.vias = [via(2.0 * k + 0.6, 0.0) for k in range(n)]
    pcb.segments = [s for k in range(n)
                    for s in (seg('F.Cu', 2.0 * k, 0, 2.0 * k + 0.6, 0),
                              seg('In2.Cu', 2.0 * k + 0.6, 0, 2.0 * k + 0.6, 1.0))]
    calls = [0]
    real = net_queries.expand_pad_layers

    def counting(*a, **k):
        calls[0] += 1
        return real(*a, **k)
    net_queries.expand_pad_layers = counting
    try:
        net_copper_length(pcb, 1, release='10.0.7')
    finally:
        net_queries.expand_pad_layers = real
    check('the via-pad search is not vias x pads', calls[0] <= 10 * n,
          f'{calls[0]} expand_pad_layers calls for {n} vias and {n} pads')


# --- every net kicad-cli reported, per release --------------------------------
def pinned():
    doc = json.load(open(evidence(ACTUALS, 'kicad-cli actuals')))
    for stem, rel_path in doc['boards'].items():
        pcb, _ = quiet(parse_kicad_pcb, evidence(os.path.join(ROOT, rel_path), stem))
        by = {n.name.replace('{slash}', '/'): i for i, n in pcb.nets.items()}
        with open(os.path.join(ROOT, rel_path), encoding='utf-8') as fh:
            text = fh.read()
        arc_nets = {int(m) for m in re.findall(r'\(arc \(start[^\n]*?\(net (\d+)\)', text)}
        for rel in RELEASES:
            want = doc['actuals'][rel][stem]
            off = {}
            for name, w in want.items():
                nid = by.get(name)
                got = (net_copper_length(pcb, nid, release=rel)
                       + net_pad_to_die_length(pcb, nid)) if nid is not None else None
                ok = got is not None and (abs(got - float(w)) <= 0.005
                                          if nid in arc_nets else same4(got, w))
                if not ok:
                    off[name] = (w, None if got is None else round(got, 4))
            check(f'{stem}: all {len(want)} nets kicad-cli {rel} reported equal '
                  f'at its 4 decimals', len(want) >= 10 and not off,
                  str(dict(list(off.items())[:5])))


def verdict_flips():
    with tempfile.TemporaryDirectory() as tmp:
        b = os.path.join(tmp, 'ro.kicad_pcb')
        quiet(copy_board, RO, b)
        with open(os.path.join(tmp, 'ro.kicad_dru'), 'w', encoding='utf-8') as fh:
            fh.write("(version 1)\n"
                     "(rule \"d13\"\n  (condition \"A.NetName == 'Net-(U2A-DATA_13)'\")\n"
                     "  (constraint length (max 15.25mm)))\n"
                     "(rule \"be0\"\n  (condition \"A.NetName == 'Net-(U2A-BE_0)'\")\n"
                     "  (constraint length (max 18.86mm)))\n")
        for rel, want in (('10.0.0', {'Net-(U2A-BE_0)': 18.8847}),
                          ('10.0.7', {'Net-(U2A-DATA_13)': 15.3782})):
            net_queries.use_length_measure_release(rel)
            try:
                rows, log = quiet(run_drc, b, clearance=0.1, max_print=0,
                                  check_sizes=False)
            finally:
                net_queries.use_length_measure_release(None)
            got = {v['net1']: round(v['length_mm'], 4) for v in rows
                   if v.get('type') == 'length-out-of-range'}
            check(f"{rel}: the issue's windows grade as kicad-cli {rel} grades them",
                  got == want and f'measured as KiCad {rel} measures' in log, str(got))


def live():
    from kicad_oracle import find_kicad_cli, kicad_cli_version
    cli = find_kicad_cli(warn=False)
    have = kicad_cli_version(cli) if cli else None
    if not have or net_queries.length_measure_release(have) != have:
        print(f'  [--] live cross-check not run: installed kicad-cli is {have}, '
              f'not a modelled release')
        return
    with tempfile.TemporaryDirectory() as tmp:
        b = os.path.join(tmp, 'ro.kicad_pcb')
        quiet(copy_board, RO, b)
        with open(os.path.join(tmp, 'ro.kicad_dru'), 'w', encoding='utf-8') as fh:
            fh.write("(version 1)\n(rule \"all\"\n  (condition \"A.NetName == '*'\")\n"
                     "  (constraint length (max 0.001mm)))\n")
        out = os.path.join(tmp, 'drc.json')
        subprocess.run([cli, 'pcb', 'drc', '--severity-all', '--format', 'json',
                        '-o', out, b], capture_output=True, text=True)
        want = {}
        for v in json.load(open(evidence(out, 'kicad-cli drc json')))['violations']:
            if v['type'] != 'length_out_of_range':
                continue
            m = re.search(r'actual ([\d.]+) mm', v['description'])
            names = {re.findall(r'\[([^\]]*)\]', i['description'])[0] for i in v['items']
                     if re.findall(r'\[([^\]]*)\]', i['description'])}
            if m and len(names) == 1:
                want[names.pop()] = m.group(1)
        pcb, _ = quiet(parse_kicad_pcb, b)
        by = {n.name.replace('{slash}', '/'): i for i, n in pcb.nets.items()}
        off = {}
        for name, w in want.items():
            nid = by.get(name)
            got = (net_copper_length(pcb, nid) + net_pad_to_die_length(pcb, nid)
                   if nid is not None else None)
            if got is None or not same4(got, w):
                off[name] = (w, got)
        check(f'live kicad-cli {have}: all {len(want)} nets of routed_output equal, '
              f'measured as the installed release',
              len(want) > 150 and not off and net_queries.length_measure_release() == have,
              str(dict(list(off.items())[:5])))


def main():
    selection()
    synthetic()
    via_pad_search_cost()
    pinned()
    verdict_flips()
    live()
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
