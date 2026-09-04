#!/usr/bin/env python3
"""#678: a pour-served ball is a COMMITMENT the route step's plane finalize
must defend.

The BGA fanout's pour-direct serves a plane-net ball by fill contact instead
of a drop via (measured: 38 rail balls, 18 -> 0 gap vias). Routing later in
the same chain can carve that ball's fill island off the sourced region, and
the refill then ships it split. Before #678 the fanout recorded only a per-net
COUNT ('pour': n) in a process-local report, so no later step could tell WHICH
balls were promised, and the composition-time invariant (#662 3b) runs before
routing exists. This gate pins the post-route half of the contract:

  1. the fanout RECORDS every promised ball (REF.PAD -> net, layer) in its
     JSON summary and in the sibling .kicad_pro, the channel every later
     chain step inherits (negative control: absent on a pre-#678 tree);
  2. the audit answers "does this promised ball reach the sourced region?"
     for a carved-off ball (detached, with a weld link anchored AT THE BALL)
     and for an intact one (kept) -- against the raster model always, and
     against KiCad's exact fill when pcbnew is available;
  3. a route step reading the promise reports the audit's POPULATIONS in
     its summary at both the finalize and ship-time sites (negative
     control: no `pour_served` key on a pre-#678 tree), and the promised
     ball is connected on the shipped board;
  4. the GUI-shaped front (return_results=True, no file) gets the same
     audit through results_data, disclosed as model-sourced.

Populations are printed for every step; a step that checks nothing fails.
Needs neither wx nor pcbnew (pcbnew only ADDS the exact-source arm).

    python3 tests/test_678_pour_served_defence.py
"""
import io
import contextlib
import json
import os
import re
import shutil
import sys
import tempfile

# T678_ROOT points the gate at ANOTHER checkout's engine -- the negative
# control: on a pre-#678 tree the fanout records no promise and the route
# step reports no audit, and the gate must FAIL there for those reasons.
ROOT = os.environ.get('T678_ROOT') or os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from run_utils import check, evidence          # noqa: E402

PY = sys.executable
FANOUT = os.path.join(ROOT, 'py_router', 'bga_fanout.py')
ROUTE = os.path.join(ROOT, 'py_router', 'route.py')

CHECKS = []


def ok(name, cond, detail=''):
    CHECKS.append((name, bool(cond)))
    print(('  PASS: ' if cond else '  FAIL: ') + name + (' -- ' + detail if detail else ''))


# --------------------------------------------------------------------------
# Fixture: a 20x20 two-layer board, GND poured on BOTH layers, a 3x3 "BGA"
# U1 at (10,10) on a 1.0 mm pitch whose CENTRE ball B2 is GND (the 8 ring
# balls are signals S1..S8 with partner pads on J1), a plated GND mounting
# hole MH1 as the sourced region's anchor. `carve=True` adds a closed loop of
# CUT-net copper on F.Cu around the whole package (pads J2.1/J2.2 at its
# corners), which cuts B2's F.Cu fill island off the sourced region -- the
# route-stage damage #678 is about, laid by hand so it is deterministic.
# --------------------------------------------------------------------------
HEADER = '''(kicad_pcb
\t(version 20260206)
\t(generator "pcbnew")
\t(generator_version "10.0")
\t(general
\t\t(thickness 1.6)
\t\t(legacy_teardrops no)
\t)
\t(paper "A4")
\t(layers
\t\t(0 "F.Cu" signal)
\t\t(2 "B.Cu" signal)
\t\t(9 "F.Adhes" user "F.Adhesive")
\t\t(11 "B.Adhes" user "B.Adhesive")
\t\t(13 "F.Paste" user)
\t\t(15 "B.Paste" user)
\t\t(5 "F.SilkS" user "F.Silkscreen")
\t\t(7 "B.SilkS" user "B.Silkscreen")
\t\t(1 "F.Mask" user)
\t\t(3 "B.Mask" user)
\t\t(25 "Edge.Cuts" user)
\t\t(27 "Margin" user)
\t\t(31 "F.CrtYd" user "F.Courtyard")
\t\t(29 "B.CrtYd" user "B.Courtyard")
\t\t(35 "F.Fab" user)
\t\t(33 "B.Fab" user)
\t)
\t(setup
\t\t(pad_to_mask_clearance 0)
\t\t(allow_soldermask_bridges_in_footprints no)
\t\t(tenting (front yes) (back yes))
\t\t(pcbplotparams (layerselection 0x00000000_00000000_55555555_5755f5ff) (outputdirectory ""))
\t)
'''

NETS = ['GND', 'S1', 'S2', 'S3', 'S4', 'S5', 'S6', 'S7', 'S8', 'CUT', 'CUT2']


def _fp(ref, name, x, y, pads, layer='F.Cu'):
    body = [f'\t(footprint "{name}"\n\t\t(layer "{layer}")\n\t\t(uuid "uuid-{ref}")\n'
            f'\t\t(at {x} {y})\n'
            f'\t\t(property "Reference" "{ref}"\n\t\t\t(at 0 -2 0)\n\t\t\t(layer "F.SilkS")\n'
            f'\t\t\t(uuid "uuid-{ref}-ref")\n\t\t\t(effects (font (size 1 1)))\n\t\t)\n'
            f'\t\t(property "Value" ""\n\t\t\t(at 0 2 0)\n\t\t\t(layer "F.Fab")\n'
            f'\t\t\t(uuid "uuid-{ref}-val")\n\t\t\t(effects (font (size 1 1)))\n\t\t)\n']
    for num, (px, py, net, kind) in pads.items():
        if kind == 'th':
            body.append(f'\t\t(pad "{num}" thru_hole circle\n\t\t\t(at {px} {py})\n'
                        f'\t\t\t(size 1.6 1.6)\n\t\t\t(drill 0.8)\n'
                        f'\t\t\t(layers "*.Cu" "*.Mask")\n\t\t\t(remove_unused_layers no)\n'
                        f'\t\t\t(net "{net}")\n\t\t\t(uuid "uuid-{ref}-{num}")\n\t\t)\n')
        else:
            sz = 0.5 if kind == 'ball' else 0.6
            shape = 'circle' if kind == 'ball' else 'rect'
            body.append(f'\t\t(pad "{num}" smd {shape}\n\t\t\t(at {px} {py})\n'
                        f'\t\t\t(size {sz} {sz})\n\t\t\t(layers "F.Cu" "F.Mask" "F.Paste")\n'
                        f'\t\t\t(net "{net}")\n\t\t\t(uuid "uuid-{ref}-{num}")\n\t\t)\n')
    body.append('\t)\n')
    return ''.join(body)


def _zone(net, layer, uid):
    return (f'\t(zone\n\t\t(net "{net}")\n\t\t(layer "{layer}")\n'
            f'\t\t(uuid "{uid}")\n\t\t(hatch edge 0.5)\n'
            f'\t\t(connect_pads yes\n\t\t\t(clearance 0.1)\n\t\t)\n\t\t(min_thickness 0.1)\n'
            f'\t\t(fill yes\n\t\t\t(thermal_gap 0.3)\n\t\t\t(thermal_bridge_width 0.3)\n'
            f'\t\t\t(island_removal_mode 0)\n\t\t)\n'
            f'\t\t(polygon\n\t\t\t(pts\n\t\t\t\t(xy 1 1) (xy 19 1) (xy 19 19) (xy 1 19)\n\t\t\t)\n\t\t)\n\t)\n')


def _seg(x1, y1, x2, y2, net, layer, uid, width=0.15):
    return (f'\t(segment\n\t\t(start {x1} {y1})\n\t\t(end {x2} {y2})\n\t\t(width {width})\n'
            f'\t\t(layer "{layer}")\n\t\t(net "{net}")\n\t\t(uuid "{uid}")\n\t)\n')


def build_fixture(path, carve=False):
    parts = [HEADER]
    parts.append('\t(gr_rect\n\t\t(start 0 0)\n\t\t(end 20 20)\n\t\t(stroke (width 0.1) (type default))\n'
                 '\t\t(fill no)\n\t\t(layer "Edge.Cuts")\n\t\t(uuid "edge")\n\t)\n')
    # U1: 3x3 balls, rows A..C (y), cols 1..3 (x); centre B2 = GND.
    rows, cols = 'ABC', '123'
    pads = {}
    k = 0
    for ri, r in enumerate(rows):
        for ci, c in enumerate(cols):
            num = f'{r}{c}'
            px, py = (ci - 1) * 1.0, (ri - 1) * 1.0
            if num == 'B2':
                pads[num] = (px, py, 'GND', 'ball')
            else:
                k += 1
                pads[num] = (px, py, f'S{k}', 'ball')
    parts.append(_fp('U1', 'Package_BGA:BGA-9_3x3_P1.0mm', 10, 10, pads))
    parts.append(_fp('MH1', 'MountingHole:MountingHole_0.8mm', 3, 3,
                     {'1': (0, 0, 'GND', 'th')}))
    parts.append(_fp('J1', 'Connector:J1', 17, 10,
                     {str(i + 1): (0, (i - 3.5) * 1.2, f'S{i + 1}', 'smd')
                      for i in range(8)}))
    if carve:
        # Two TREE-shaped nets, not one closed loop: the route step's
        # cleanup prunes a same-net cycle as redundant copper (measured: a
        # 4-segment CUT square lost 2 segments to "Cycle prune" and the
        # island reconnected by itself). CUT is a U open at the top; CUT2 is
        # a bar across the opening, 0.15 mm off the U's end pads -- legal at
        # the 0.1 clearance, and no fill can pass (0.1 clearance each side
        # leaves nothing at the 0.1 min thickness).
        a, b = 7.4, 12.6
        parts.append(_fp('J2', 'Connector:J2', 10, 10,
                         {'1': (a - 10, 7.7 - 10, 'CUT', 'smd'),
                          '2': (b - 10, 7.7 - 10, 'CUT', 'smd')}))
        parts.append(_seg(a, 7.7, a, b, 'CUT', 'F.Cu', 'cut1'))
        parts.append(_seg(a, b, b, b, 'CUT', 'F.Cu', 'cut2'))
        parts.append(_seg(b, b, b, 7.7, 'CUT', 'F.Cu', 'cut3'))
        parts.append(_fp('J3', 'Connector:J3', 10, 10,
                         {'1': (6.9 - 10, 7.275 - 10, 'CUT2', 'smd'),
                          '2': (13.1 - 10, 7.275 - 10, 'CUT2', 'smd')}))
        parts.append(_seg(6.9, 7.275, 13.1, 7.275, 'CUT2', 'F.Cu', 'cut4'))
    parts.append(_zone('GND', 'F.Cu', 'zone-f'))
    parts.append(_zone('GND', 'B.Cu', 'zone-b'))
    parts.append(')\n')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(''.join(parts))
    return path


def _json_summary(out):
    m = re.findall(r'^JSON_SUMMARY: (.*)$', out, re.M)
    return json.loads(m[-1]) if m else None


def main():
    work = tempfile.mkdtemp(prefix='t678_')
    print(f"work dir: {work}")

    # ---- 1. the fanout records the promise ---------------------------------
    base = build_fixture(os.path.join(work, 'base.kicad_pcb'))
    fan = os.path.join(work, 'fan.kicad_pcb')
    r = check([PY, FANOUT, base, '--output', fan, '--component', 'U1',
               '--nets', '*', '!GND', '--layers', 'F.Cu', 'B.Cu',
               '--clearance', '0.1', '--track-width', '0.1',
               '--via-size', '0.3', '--via-drill', '0.15',
               '--plane-net-layers', 'GND:F.Cu'], accept=True)
    out = (r.stdout or '') + (r.stderr or '')
    summ = _json_summary(out)
    ok('fanout printed a JSON_SUMMARY', summ is not None)
    drop = (summ or {}).get('plane_drop') or {}
    gnd = (drop.get('nets') or {}).get('GND') or {}
    print(f"    plane_drop GND counters: {gnd}")
    ok('fanout served the GND centre ball by pour contact (pour >= 1)',
       gnd.get('pour', 0) >= 1, f"pour={gnd.get('pour')} gap={gnd.get('gap')} "
       f"in_pad={gnd.get('in_pad')} failed={gnd.get('failed')}")
    served = drop.get('pour_served_pads') or {}
    ok('fanout summary names WHICH ball was promised (pour_served_pads has U1.B2)',
       'U1.B2' in served, f"pour_served_pads={sorted(served)}")
    # A pre-#678 engine has none of this API. Say so as a FAIL with its
    # reason and stop -- a traceback here would be the BROKEN-TEST shape
    # (a check that dies before it checks anything).
    try:
        from protected_nets import read_pour_served_pads, pro_path_for_board
        from pour_promise import audit_pour_promises
    except ImportError as e:
        ok('engine carries the #678 promise API (protected_nets.read_pour_served_pads, '
           'pour_promise.audit_pour_promises)', False, f"{e}")
        shutil.rmtree(work, ignore_errors=True)
        failed = [n for n, c in CHECKS if not c]
        print(f"\n{len(CHECKS) - len(failed)}/{len(CHECKS)} checks passed")
        print("FAIL (pre-#678 engine):\n  " + "\n  ".join(failed))
        return 1
    pro = pro_path_for_board(fan)
    evidence(pro, 'fanout output .kicad_pro')
    promised = read_pour_served_pads(pro)
    print(f"    .kicad_pro pour_served_pads: {promised}")
    ok('promise persisted in the sibling .kicad_pro (negative control: absent pre-#678)',
       promised.get('U1.B2', {}).get('net') == 'GND'
       and promised.get('U1.B2', {}).get('layer') == 'F.Cu')
    ok('pour-served ball got NO via (the promise displaced it)',
       gnd.get('in_pad', 0) == 0 and gnd.get('gap', 0) == 0)

    # ---- 2. the audit: carved -> detached, intact -> kept -----------------
    from kicad_parser import parse_kicad_pcb
    carved = build_fixture(os.path.join(work, 'carved.kicad_pcb'), carve=True)
    intact = build_fixture(os.path.join(work, 'intact.kicad_pcb'))
    for src in (carved, intact):
        shutil.copy(pro, pro_path_for_board(src))
    pcb_c = parse_kicad_pcb(carved)
    pcb_i = parse_kicad_pcb(intact)
    a_model = audit_pour_promises(pcb_c, promised)          # no file -> model
    print(f"    model audit (carved): source={a_model['source']} promised={a_model['promised']} "
          f"checked={a_model['checked']} kept={a_model['kept']} "
          f"detached={[d['key'] for d in a_model['detached']]} stale={a_model['stale']}")
    ok('model audit checks the one promised ball', a_model['checked'] == 1 and a_model['promised'] == 1)
    ok('model audit: carved ball is DETACHED',
       [d['key'] for d in a_model['detached']] == ['U1.B2'])
    link = (a_model['detached'] or [{}])[0].get('link')
    print(f"    weld link: {link}")
    ok('weld link is anchored AT THE BALL (10,10 F.Cu, kind pad)',
       bool(link) and abs(link[1][0] - 10) < 1e-6 and abs(link[1][1] - 10) < 1e-6
       and link[1][2] == 'F.Cu' and link[1][3] == 'pad')
    ok('weld link target is on sourced copper (not the ball itself)',
       bool(link) and (abs(link[2][0] - 10) > 0.3 or abs(link[2][1] - 10) > 0.3))
    a_int = audit_pour_promises(pcb_i, promised)
    print(f"    model audit (intact): kept={a_int['kept']} detached={[d['key'] for d in a_int['detached']]}")
    ok('model audit: intact ball is KEPT', a_int['kept'] == ['U1.B2'] and not a_int['detached'])
    # stale bookkeeping: a promise for a pad that is not on the board, one
    # whose net was renamed, and one whose net owns no zone are reported
    # apart, never guessed.
    a_stale = audit_pour_promises(pcb_i, {
        'U9.Z9': {'net': 'GND', 'layer': 'F.Cu'},
        'U1.A1': {'net': 'GND', 'layer': 'F.Cu'},
        'U1.A2': {'net': 'S2', 'layer': 'F.Cu'}})
    reasons = {s['key']: s['reason'] for s in a_stale['stale']}
    print(f"    stale reasons: {reasons}")
    ok('stale promises are classified (missing / renamed / no_zone)',
       reasons == {'U9.Z9': 'missing', 'U1.A1': 'renamed', 'U1.A2': 'no_zone'}
       and a_stale['checked'] == 0)
    from kicad_exact_fill import find_kicad_python
    if find_kicad_python() is not None:
        a_exact = audit_pour_promises(pcb_c, promised, board_file=carved)
        print(f"    exact audit (carved): source={a_exact['source']} kept={a_exact['kept']} "
              f"detached={[d['key'] for d in a_exact['detached']]} why={a_exact['why']}")
        ok('exact-fill audit: carved ball is DETACHED', a_exact['source'] == 'exact'
           and [d['key'] for d in a_exact['detached']] == ['U1.B2'])
        a_exact_i = audit_pour_promises(pcb_i, promised, board_file=intact)
        ok('exact-fill audit: intact ball is KEPT', a_exact_i['source'] == 'exact'
           and a_exact_i['kept'] == ['U1.B2'])
    else:
        print("    (pcbnew unavailable: exact-source arm not run; model arm covers the verdict)")

    # ---- 3. the route step defends and DISCLOSES --------------------------
    routed = os.path.join(work, 'routed.kicad_pcb')
    jout = os.path.join(work, 'routed.json')
    r = check([PY, ROUTE, carved, routed, '--nets', 'GND',
               '--layers', 'F.Cu', 'B.Cu', '--track-width', '0.15',
               '--clearance', '0.1', '--via-size', '0.4', '--via-drill', '0.2',
               '--json-out', jout], accept=True, timeout=600)
    out = (r.stdout or '') + (r.stderr or '')
    lines = [l for l in out.splitlines() if 'Pour-served balls (#678' in l]
    for l in lines:
        print('    ' + l.strip())
    ok('route step printed the promise audit populations', bool(lines))
    evidence(jout, 'route --json-out')
    js = json.load(open(jout))
    ps = js.get('pour_served') or {}
    print(f"    summary pour_served: {json.dumps(ps)}")
    ok('route summary carries pour_served.finalize (negative control: absent pre-#678)',
       isinstance(ps.get('finalize'), dict) and ps['finalize'].get('promised') == 1
       and ps['finalize'].get('checked') == 1)
    ok('route summary carries pour_served.ship with the same population',
       isinstance(ps.get('ship'), dict) and ps['ship'].get('checked') == 1)
    ok('ship-time audit reports the ball KEPT (0 detached)',
       ps.get('ship', {}).get('kept') == 1 and not ps.get('ship', {}).get('detached'))
    from check_connected import check_net_connectivity
    pcb_r = parse_kicad_pcb(routed)
    gid = next(i for i, n in pcb_r.nets.items() if n.name == 'GND')
    res = check_net_connectivity(gid, [s for s in pcb_r.segments if s.net_id == gid],
                                 [v for v in pcb_r.vias if v.net_id == gid],
                                 pcb_r.pads_by_net.get(gid, []),
                                 [z for z in pcb_r.zones if z.net_id == gid],
                                 pcb_data=pcb_r)
    print(f"    shipped GND: connected={res.get('connected')} components={res.get('num_components')} "
          f"disconnected_pads={res.get('disconnected_pads')}; GND vias on board: "
          f"{sum(1 for v in pcb_r.vias if v.net_id == gid)}")
    ok('promised ball is connected on the shipped board (fill-aware)', res.get('connected'))

    # ---- 4. GUI-shaped front: return_results, no file to refill -----------
    from route import batch_route
    buf = io.StringIO()
    pcb_g = parse_kicad_pcb(carved)
    with contextlib.redirect_stdout(buf):
        outg = batch_route(input_file=carved, output_file='', net_names=['GND'],
                           layers=['F.Cu', 'B.Cu'], track_width=0.15, clearance=0.1,
                           via_size=0.4, via_drill=0.2, grid_step=0.1,
                           return_results=True, pcb_data=pcb_g)
    rd = outg[3] if len(outg) > 3 else {}
    psg = rd.get('pour_served') or {}
    print(f"    GUI-front results_data pour_served: {json.dumps(psg)}")
    ok('GUI-shaped front gets the audit through results_data',
       isinstance(psg.get('finalize'), dict) and psg['finalize'].get('checked') == 1)
    ok('GUI-shaped front discloses its source (model or exact), never a bare verdict',
       psg.get('finalize', {}).get('source') in ('model', 'exact'))

    shutil.rmtree(work, ignore_errors=True)
    failed = [n for n, c in CHECKS if not c]
    print(f"\n{len(CHECKS) - len(failed)}/{len(CHECKS)} checks passed")
    if failed:
        print("FAIL:\n  " + "\n  ".join(failed))
        return 1
    print("PASS: #678 pour-served balls are recorded, audited and defended")
    return 0


if __name__ == '__main__':
    sys.exit(main())
