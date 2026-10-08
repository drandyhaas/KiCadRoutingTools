#!/usr/bin/env python3
"""A part's own net-less copper against its own pad's net is disclosed, not hidden (#995).

#908 lets a net route onto net-0 footprint copper drawn around its own pad:
esp_prog U2's SOT-89 tab around pad 2 (`Net-(C1-Pad1)`). Physically that is
the pad's own copper. KiCad gives footprint graphic copper no net, so its DRC
reports every such contact as `shorting_items` against `<no net>` -- one on the
unrouted board (pad 2 against its own tab) and one per track routed onto it.
check_drc waived them silently, so kicad_drc_compare called each a check_drc
false negative (4 KICAD-ONLY on the routed board).

Now check_drc publishes each waived contact as an accepted
`footprint-own-copper` row and prints it under WARNINGS, and kicad_drc_compare
drops the KiCad item matching the part KiCad names and that net. Only the ONE
net the part's own pads give its copper qualifies:

  * a GND track touching the same tab is a real GND short to `Net-(C1-Pad1)`,
    which KiCad reports in the same `<no net>` form. It must not become an
    accepted row: check_drc COUNTS it (#1181 -- a touching track used to
    grant the tab its own net and waive itself), and the compare tool pairs it
    with KiCad's item;
  * a net-less PAD ("Pad 3 [<no net>] of U2") is not graphic copper.

The KiCad leg runs only where kicad-cli is found and says so when it does not.

    python3 tests/test_995_footprint_own_copper.py
"""
import contextlib
import io
import math
import os
import shutil
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for p in (os.path.join(ROOT, 'py_router'), os.path.join(ROOT, 'tests', 'stress')):
    if p not in sys.path:
        sys.path.insert(0, p)

from kicad_parser import parse_kicad_pcb
from check_drc import run_drc, footprint_own_copper_nets
import kicad_drc_compare as kdc

FAILS = []
FIXTURE = os.path.join(ROOT, 'tests', 'fixtures', 'run25', 'esp_prog_placed.kicad_pcb')
OWN_NET = 'Net-(C1-Pad1)'


def check(name, cond, detail=""):
    if not cond:
        FAILS.append(name)
    print(("  PASS " if cond else "  FAIL ") + name + (f"  {detail}" if detail else ""))


def _parse(path):
    with contextlib.redirect_stdout(io.StringIO()):
        return parse_kicad_pcb(path)


def _drc(path):
    with contextlib.redirect_stdout(io.StringIO()):
        return run_drc(path, quiet=True, print_summary=False)


def _with_track(base, out, net_name):
    """`base` plus a 0.2 mm track ending on the midpoint of U2's tab edge
    farthest from pad 2, coming from 1 mm outside: contact, not a crossing."""
    pcb = _parse(base)
    tab = [s for s in pcb.segments if getattr(s, 'graphic', False) and s.owner_ref == 'U2']
    pad2 = [p for p in pcb.footprints['U2'].pads if p.pad_number == '2'][0]
    cx = sum((s.start_x + s.end_x) / 2 for s in tab) / len(tab)
    cy = sum((s.start_y + s.end_y) / 2 for s in tab) / len(tab)
    far = max(tab, key=lambda s: math.hypot((s.start_x + s.end_x) / 2 - pad2.global_x,
                                            (s.start_y + s.end_y) / 2 - pad2.global_y))
    mx, my = (far.start_x + far.end_x) / 2, (far.start_y + far.end_y) / 2
    d = math.hypot(mx - cx, my - cy)
    x0, y0 = mx + (mx - cx) / d, my + (my - cy) / d
    net = [n for n in pcb.nets.values() if n.name == net_name][0]
    seg = (f'\t(segment (start {x0:.4f} {y0:.4f}) (end {mx:.4f} {my:.4f}) (width 0.2) '
           f'(layer "{far.layer}") (net {net.net_id}) '
           f'(uuid "99500000-0000-4000-8000-00000000000{net.net_id % 10}"))\n')
    with open(base, encoding='utf-8') as f:
        txt = f.read()
    i = txt.rstrip().rfind(')')
    with open(out, 'w', encoding='utf-8') as f:
        f.write(txt[:i] + seg + txt[i:])
    shutil.copy(os.path.splitext(base)[0] + '.kicad_pro', os.path.splitext(out)[0] + '.kicad_pro')


def _own(rows):
    return [r for r in rows if r.get('accepted') == 'footprint-own-copper']


def t_check_drc(base, own, gnd):
    pcb = _parse(base)
    c1 = [n.net_id for n in pcb.nets.values() if n.name == OWN_NET][0]
    check("U2's copper belongs to pad 2's net alone",
          footprint_own_copper_nets(pcb).get('U2') == frozenset((c1,)),
          str(footprint_own_copper_nets(pcb).get('U2')))

    rows = {k: _drc(p) for k, p in (('base', base), ('own', own), ('gnd', gnd))}
    for k in ('base', 'own'):
        r = rows[k]
        check(f'{k}: no counted violation', not [v for v in r if not v.get('accepted')],
              str([v['type'] for v in r if not v.get('accepted')][:5]))
    counted = [v for v in rows['gnd'] if not v.get('accepted')]
    check('gnd: the GND short is counted, against the tab (#1181)',
          len(counted) == 1 and {counted[0]['net1'], counted[0]['net2']} == {'GND', 'net_0'}
          and counted[0].get('item2') == 'Polygon(U2)',
          str([(v['type'], v['net1'], v['net2']) for v in counted]))
    b, o, g = (_own(rows[k]) for k in ('base', 'own', 'gnd'))
    check('base: pad 2 against its own tab is published',
          b and all((r['owner'], r['net1'], r['net2']) == ('U2', OWN_NET, '<no net>') for r in b),
          f'{len(b)} row(s)')
    check('base: the rows name the class KiCad raises',
          all(r['kicad_class'] == 'shorting_items' for r in b))
    check('an own-net track on the tab adds one published contact',
          len(o) == len(b) + 1 and all(r['net1'] == OWN_NET for r in o),
          f'{len(b)} -> {len(o)}')
    check('a GND track on the tab is NOT published as own copper',
          len(g) == len(b) and not [r for r in g if r['net1'] == 'GND'],
          f'{len(b)} -> {len(g)}: {sorted({r["net1"] for r in g})}')


def t_check_drc_warns(own):
    r = subprocess.run([sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_router', 'check_drc.py'), own],
                       capture_output=True, text=True, cwd=ROOT)
    check('check_drc exits 0 on own-copper contacts', r.returncode == 0, f'exit {r.returncode}')
    check('check_drc prints the footprint own copper warning',
          'footprint own copper:' in r.stdout and f'U2 {OWN_NET} x' in r.stdout,
          [ln for ln in r.stdout.splitlines() if 'own copper' in ln][:1])


def t_compare_matching():
    check('a KiCad graphic item names its part',
          kdc._graphic_owners(['Polygon [<no net>] of U2 on F.Cu',
                               'Track [GND] on F.Cu, length 1.0000 mm']) == ('U2',))
    check('a net-less PAD is not graphic copper',
          kdc._graphic_owners(['Pad 3 [<no net>] of U2 on F.Cu']) == ())

    def kv(nets, owners, typ='shorting_items'):
        return {'type': typ, 'nets': frozenset(nets), 'pos': (0.0, 0.0),
                'graphic_owners': tuple(owners)}
    cd_own = [{'type': 'footprint-own-copper', 'nets': frozenset((OWN_NET, '<no net>')),
               'owner': 'U2', 'accepted': 'footprint-own-copper'}]
    items = [kv(('<no net>', OWN_NET), ['U2']),
             kv(('<no net>', OWN_NET), ['U2'], 'clearance'),
             kv(('<no net>', 'GND'), ['U2']),
             kv(('<no net>', OWN_NET), ['U3']),
             kv(('<no net>', OWN_NET), ['U2'], 'tracks_crossing')]
    kept, dropped = kdc._drop_kicad_own_copper(items, cd_own)
    check('own-net shorting_items and clearance on U2 are dropped', dropped == 2, f'dropped {dropped}')
    check('GND on U2, the same net on another part, and another class are kept',
          kept == items[2:], str([(sorted(k['nets']), k['graphic_owners'], k['type']) for k in kept]))
    check('nothing is dropped without a published contact',
          kdc._drop_kicad_own_copper(items, []) == (items, 0))


def t_kicad_leg(base, own, gnd):
    if not (os.path.isfile(kdc.KICAD_CLI) or shutil.which(kdc.KICAD_CLI)):
        print(f'  SKIP the KiCad leg: kicad-cli not found ({kdc.KICAD_CLI})')
        return
    with contextlib.redirect_stdout(io.StringIO()):
        d_own = kdc.compare_board_data(own, baseline=base)
        d_gnd = kdc.compare_board_data(gnd, baseline=base)
    check('KiCad: the own-net contact is on the #995 channel, not kicad_only',
          d_own['kicad_only'] == 0 and d_own['kicad_own_copper'] >= 1,
          f"kicad_only={d_own['kicad_only']} own={d_own['kicad_own_copper']}")
    check('KiCad: the GND short is matched by check_drc, not on the #995 channel',
          d_gnd['kicad_only'] == 0 and d_gnd['checkdrc_only'] == 0
          and d_gnd['matched'] == 1 and d_gnd['kicad_own_copper'] == 0,
          f"matched={d_gnd['matched']} kicad_only={d_gnd['kicad_only']} "
          f"checkdrc_only={d_gnd['checkdrc_only']} own={d_gnd['kicad_own_copper']}")


def main():
    tmp = tempfile.mkdtemp(prefix='t995_')
    try:
        base = os.path.join(tmp, 'base.kicad_pcb')
        shutil.copy(FIXTURE, base)
        shutil.copy(os.path.splitext(FIXTURE)[0] + '.kicad_pro', os.path.join(tmp, 'base.kicad_pro'))
        own, gnd = os.path.join(tmp, 'own.kicad_pcb'), os.path.join(tmp, 'gnd.kicad_pcb')
        _with_track(base, own, OWN_NET)
        _with_track(base, gnd, 'GND')
        t_check_drc(base, own, gnd)
        t_check_drc_warns(own)
        t_compare_matching()
        t_kicad_leg(base, own, gnd)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print()
    if FAILS:
        print(f"{len(FAILS)} FAILURE(S): {', '.join(FAILS)}")
        return 1
    print("ALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
