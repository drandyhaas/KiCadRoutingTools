#!/usr/bin/env python3
"""A BGA fanout gives every plane ball a via or names it (plane-drop pass).

The drop pass (#424) took a net left out of `--nets` for a plane only with
>= 6 balls on the part or a zone of its own. A small rail -- two to five
balls, its pour not drawn yet -- was then neither escaped nor dropped, and
nothing said so: JSON_SUMMARY counted it nowhere. A drop that found no via
site was counted (`failed`) but never named. Both are now covered:

* an excluded net is also a plane when it is declared in
  `--plane-net-layers` or every one of its balls is a power pin;
* every plane ball the call leaves with no copper is named, with why, in
  JSON_SUMMARY `plane_drop.unserved` and on the console.

The fixture is generated here: a 4-layer board with an 8 x 8, 0.8 mm BGA.
The outer two rings are signals to a connector; the middle 4 x 4 holds GND
(10 balls) and three 2-ball rails: VRAIL (power pins), VAUX (no pin type,
declared in --plane-net-layers) and VNONE (no pin type, not declared). The
blocked variant adds four vias of another net on the gap sites around one
GND ball, and the run forbids via-in-pad.

  1. VRAIL and VAUX balls get a via; VNONE balls are named, with the reason;
  2. the signal escape is the same copper with the pin types and without
     (the pass runs after the signals and only adds);
  3. the blocked GND ball is named with "no free inter-ball via site, and
     via-in-pad is off";
  4. every plane ball with no copper on the written board is named, and
     no named ball has copper (the CLI's JSON_SUMMARY, read off the board);
  5. an unfiltered run escapes the small rails as signals and names none.

    python3 tests/test_bga_plane_balls_named.py
"""
import json
import math
import os
import re
import shutil
import sys
import tempfile
import uuid

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

import run_utils                                                 # noqa: E402

failures = []
N, PITCH, BALL = 8, 0.8, 0.4
STACK = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']
INNER = {}            # (i, j) -> net, the middle 4 x 4
for _k, (_i, _j) in enumerate((i, j) for j in range(2, 6) for i in range(2, 6)):
    INNER[(_i, _j)] = 'GND' if (_i + _j) % 2 else None
_RAILS = iter(['VRAIL', 'VRAIL', 'VAUX', 'VAUX', 'VNONE', 'VNONE'])
for _ij in sorted(INNER, key=lambda ij: (ij[1], ij[0])):
    if INNER[_ij] is None:
        INNER[_ij] = next(_RAILS, 'GND')
BLOCKED = (3, 2)      # a GND ball whose four gap sites the blocked variant takes
FILTER = ['*', '!GND', '!V*']


def ok(label, cond, detail=''):
    print(f"  [{'ok' if cond else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not cond:
        failures.append(label)


def _n(v):
    return f"{round(v, 4):.4f}".rstrip('0').rstrip('.')


def board(pintypes=True, blocked=False):
    """The fixture board's text."""
    c = 10.0
    org = c - (N - 1) * PITCH / 2
    xy = {(i, j): (org + i * PITCH, org + j * PITCH) for i in range(N) for j in range(N)}
    net = {}
    sig = 0
    for j in range(N):
        for i in range(N):
            if (i, j) in INNER:
                net[(i, j)] = INNER[(i, j)]
            else:
                sig += 1
                net[(i, j)] = f'/S{sig:02d}'

    def pad(ref, num, x, y, w, h, layer, nm, shape='rect', pt=None):
        mask = 'F.Mask' if layer == 'F.Cu' else 'B.Mask'
        ptx = f'\t\t\t(pintype "{pt}")\n' if pt else ''
        return (f'\t\t(pad "{num}" smd {shape} (at {_n(x)} {_n(y)}) (size {_n(w)} {_n(h)})'
                f' (layers "{layer}" "{mask}") (net "{nm}")\n{ptx}'
                f'\t\t\t(uuid "{uuid.uuid5(uuid.NAMESPACE_URL, f"{ref}.{num}")}"))\n')

    def fp(ref, pads):
        return (f'\t(footprint "fixture:{ref}" (layer "F.Cu") (uuid "{uuid.uuid5(uuid.NAMESPACE_URL, ref)}")'
                f' (at 0 0)\n\t\t(property "Reference" "{ref}" (at 0 0 0) (layer "F.SilkS") (hide yes)'
                f' (uuid "{uuid.uuid5(uuid.NAMESPACE_URL, ref + ".r")}")'
                f' (effects (font (size 1 1) (thickness 0.15))))\n\t\t(attr smd)\n{"".join(pads)}\t)\n')

    def ptype(nm):
        if not pintypes:
            return None
        return 'power_in' if nm in ('GND', 'VRAIL') else 'bidirectional'

    u1 = [pad('U1', f'{"ABCDEFGH"[j]}{i + 1}', x, y, BALL, BALL, 'F.Cu', net[(i, j)],
              'circle', ptype(net[(i, j)])) for (i, j), (x, y) in sorted(xy.items())]
    sigs = sorted(n for n in net.values() if n.startswith('/'))
    j1 = [pad('J1', k + 1, 2.0 + 0.5 * k, 2.0, 0.3, 0.8, 'F.Cu', n) for k, n in enumerate(sigs)]
    # the rails' other end, and the net of the blocking vias
    r1 = [pad('R1', 1, 18.0, 6.0, 0.5, 0.5, 'F.Cu', 'VRAIL'),
          pad('R1', 2, 18.0, 7.0, 0.5, 0.5, 'F.Cu', 'GND'),
          pad('R1', 3, 18.0, 8.0, 0.5, 0.5, 'F.Cu', 'VAUX'),
          pad('R1', 4, 18.0, 9.0, 0.5, 0.5, 'F.Cu', 'VNONE'),
          pad('R1', 5, 18.0, 10.0, 0.5, 0.5, 'F.Cu', 'BLOCK'),
          pad('R1', 6, 18.0, 11.0, 0.5, 0.5, 'F.Cu', 'BLOCK')]
    vias = ''
    if blocked:
        bx, by = xy[BLOCKED]
        for k, (dx, dy) in enumerate(((-1, -1), (1, -1), (-1, 1), (1, 1))):
            vias += (f'\t(via (at {_n(bx + dx * PITCH / 2)} {_n(by + dy * PITCH / 2)}) (size 0.45)'
                     f' (drill 0.2) (layers "F.Cu" "B.Cu") (net "BLOCK")'
                     f' (uuid "{uuid.uuid5(uuid.NAMESPACE_URL, f"blk{k}")}"))\n')
    layers = ''.join(f'\t\t({i} "{n}" signal)\n' for i, n in zip((0, 4, 6, 2), STACK))
    return ('(kicad_pcb (version 20260206) (generator "pcbnew") (generator_version "10.0")\n'
            '\t(general (thickness 1.6))\n\t(paper "A4")\n'
            f'\t(layers\n{layers}\t\t(1 "F.Mask" user) (3 "B.Mask" user) (5 "F.SilkS" user)'
            ' (25 "Edge.Cuts" user)\n\t)\n\t(setup (pad_to_mask_clearance 0))\n'
            + fp('U1', u1) + fp('J1', j1) + fp('R1', r1) + vias
            + '\t(gr_rect (start 0 0) (end 20 20) (stroke (width 0.1) (type default))'
              ' (fill no) (layer "Edge.Cuts") (uuid "6b1d8f0e-0000-4000-8000-000000000001"))\n)\n')


PROJECT = {"board": {"design_settings": {"rules": {
    "min_clearance": 0.1, "min_track_width": 0.1, "min_via_diameter": 0.4,
    "min_via_annular_width": 0.1, "min_through_hole_diameter": 0.2,
    "min_hole_to_hole": 0.25, "min_copper_edge_clearance": 0.3}}},
    "net_settings": {"classes": [{"name": "Default", "clearance": 0.1,
                                  "track_width": 0.1, "via_diameter": 0.45,
                                  "via_drill": 0.2}],
                     "meta": {"version": 3}},
    "meta": {"version": 3}}


def write(tmp, name, **kw):
    path = os.path.join(tmp, name + '.kicad_pcb')
    with open(path, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write(board(**kw))
    with open(os.path.join(tmp, name + '.kicad_pro'), 'w', encoding='utf-8') as fh:
        json.dump(PROJECT, fh)
    return path


ENGINE = dict(layers=STACK, track_width=0.1, clearance=0.1, via_size=0.45,
              via_drill=0.2)


def engine(path, **kw):
    """generate_bga_fanout on `path`'s U1: (pcb, tracks, vias, report)."""
    import bga_fanout as bf
    from kicad_parser import parse_kicad_pcb
    pcb = parse_kicad_pcb(run_utils.evidence(path))
    tracks, vias, _rm, _failed = bf.generate_bga_fanout(
        pcb.footprints['U1'], pcb, **dict(ENGINE, **kw))
    return pcb, tracks, vias, dict(bf.LAST_PLANE_DROP_REPORT)


def balls_with_via(pcb, vias, net):
    """U1 pads of `net` that a via of this call reaches or a track ties to a via."""
    fp = pcb.footprints['U1']
    nid = next(p.net_id for p in fp.pads if p.net_name == net)
    out = set()
    for p in fp.pads:
        if p.net_name != net:
            continue
        for v in vias:
            if v['net_id'] == nid and math.hypot(v['x'] - p.global_x, v['y'] - p.global_y) < 1.2:
                out.add(p.pad_number)
    return out


def bare_plane_balls(path):
    """'U1.PAD' of every non-signal ball on the WRITTEN board with no copper
    of its net touching it."""
    from kicad_parser import parse_kicad_pcb
    pcb = parse_kicad_pcb(run_utils.evidence(path))
    out = set()
    for p in pcb.footprints['U1'].pads:
        if p.net_name.startswith('/'):
            continue
        r = BALL / 2
        hit = any(v.net_id == p.net_id and math.hypot(v.x - p.global_x, v.y - p.global_y)
                  <= r + v.size / 2 for v in pcb.vias)
        hit = hit or any(s.net_id == p.net_id and s.layer == 'F.Cu' and any(
            math.hypot(x - p.global_x, y - p.global_y) <= r
            for x, y in ((s.start_x, s.start_y), (s.end_x, s.end_y))) for s in pcb.segments)
        if not hit:
            out.add(f'U1.{p.pad_number}')
    return out


def main():
    tmp = tempfile.mkdtemp(prefix='t_pbn_')
    try:
        typed = write(tmp, 'typed', pintypes=True)
        plain = write(tmp, 'plain', pintypes=False)
        pnl = {'VAUX': ['In2.Cu']}

        print("1. small rails: power pins and a declared plane get vias, the rest is named")
        pcb, tr, vi, rep = engine(typed, net_filter=FILTER, plane_net_layers=pnl)
        n_vrail = len(balls_with_via(pcb, vi, 'VRAIL'))
        n_vaux = len(balls_with_via(pcb, vi, 'VAUX'))
        ok("VRAIL (both balls power pins) dropped", n_vrail == 2, f"{n_vrail}/2 balls with a via")
        ok("VAUX (declared in plane_net_layers) dropped", n_vaux == 2, f"{n_vaux}/2")
        uns = rep.get('unserved') or []
        named = {u['net'] for u in uns}
        ok("VNONE named unserved, with the reason",
           named == {'VNONE'} and len(uns) == 2
           and all('not taken for a plane' in u['reason'] for u in uns), json.dumps(uns))
        ok("the drop report counts the new rails",
           {'VRAIL', 'VAUX'} <= set(rep.get('nets') or {}), str(sorted(rep.get('nets') or {})))

        print("2. the signal escape is unchanged by the new drops")
        pcb2, tr2, vi2, rep2 = engine(plain, net_filter=FILTER)
        sig_ids = {p.net_id for p in pcb.footprints['U1'].pads if p.net_name.startswith('/')}

        def sig(ts):
            return sorted((t['net_id'], t['layer'], tuple(round(c, 4) for c in t['start']),
                           tuple(round(c, 4) for c in t['end'])) for t in ts if t['net_id'] in sig_ids)
        ok("non-vacuous: the signals escaped", len(sig(tr)) > 20, f"{len(sig(tr))} signal tracks")
        ok("signal copper identical with and without the rail drops", sig(tr) == sig(tr2))
        ok("without pin types or a declaration, the three rails are named",
           {u['net'] for u in rep2.get('unserved') or []} == {'VRAIL', 'VAUX', 'VNONE'},
           str(sorted({u['net'] for u in rep2.get('unserved') or []})))

        print("3. a drop with no via site is named with why")
        blk = write(tmp, 'blocked', pintypes=True, blocked=True)
        _p3, _t3, _v3, rep3 = engine(blk, net_filter=FILTER, plane_net_layers=pnl,
                                     same_net_pad_clearance=0.1)
        bpad = f'U1.{"ABCDEFGH"[BLOCKED[1]]}{BLOCKED[0] + 1}'
        hit = [u for u in rep3.get('unserved') or [] if u['ball'] == bpad]
        ok(f"{bpad} (gap sites taken, via-in-pad off) named",
           len(hit) == 1 and 'no free inter-ball via site, and via-in-pad is off' in hit[0]['reason'],
           json.dumps(hit))
        ok("its reason is the drop pass's own record",
           bpad in (rep3.get('failed_balls') or {}), str(sorted(rep3.get('failed_balls') or {})))

        print("4. the CLI names exactly the bare plane balls of the board it writes")
        out = os.path.join(tmp, 'out.kicad_pcb')
        r = run_utils.check([sys.executable, '-X', 'utf8', 'py_router/bga_fanout.py',
                             run_utils.evidence(blk), '-c', 'U1', '-o', out,
                             '--layers', *STACK, '--clearance', '0.1', '--track-width', '0.1',
                             '--same-net-pad-clearance', '0.1', '--nets', *FILTER,
                             '--plane-net-layers', 'VAUX:In2.Cu'], accept=True)
        text = r.stdout + r.stderr
        m = re.search(r'^JSON_SUMMARY: (.*)$', text, re.M)
        summ = json.JSONDecoder().raw_decode(m.group(1))[0] if m else {}
        named4 = {u['ball'] for u in (summ.get('plane_drop') or {}).get('unserved') or []}
        bare = bare_plane_balls(out)
        ok("non-vacuous: some plane ball is bare", bool(bare), str(sorted(bare)))
        ok("named == bare on the written board", named4 == bare,
           f"named {sorted(named4)}, bare {sorted(bare)}")
        ok("the console names them too", 'Plane balls left with no via on U1' in text
           and bpad in text)

        print("5. unfiltered: small rails escape as signals, none named")
        _p5, _t5, _v5, rep5 = engine(plain)
        ok("nothing unserved", (rep5.get('unserved') or []) == [], json.dumps(rep5.get('unserved')))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print()
    if failures:
        print(f"FAILED: {len(failures)}: {failures}")
        return 1
    print("ALL CHECKS PASSED")
    return 0


if __name__ == '__main__':
    sys.exit(main())
