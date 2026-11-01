"""#1587: a chip named for the interface it serves is not an edge receptacle.

#1534 made an interface stem (`usb`, `sim`) connector evidence only beside a
second sign of a connector, but kept the stem standing as a WORD of the item
as that sign on its own. So a soldered eSIM named `Security:SIM_MFF2-8`
(reference IC1, eight pads, no mechanical pad) was an edge receptacle: the
heights rule read it as reached from above and refused it under an M.2 card
with 1.5 mm of room for its 1.0 mm body, and the emitter's roster owed it an
edge. `USB-ESD-6` and `USB_Switch` chips went the same way.

Pinned here:
  1. on a chip's reference (U, IC) the word alone is not connector evidence:
     `SIM_MFF2-8`, `USB-ESD-6` and `USB_Switch` are no edge receptacles, on
     the CLI's `lib:item` and the GUI's bare item alike;
  2. the eSIM's machine form factor (`MFF2`) is a chip package, whatever the
     reference;
  3. an item that SPELLS a connector keeps its class on a chip's reference
     (IQRF's `KON-SIM-01`, `USB_Micro-B`, `USB-TYPE-C`, `MICRO-USB`,
     `simple-usb-c`), and the word still counts on any other reference
     (`US1`);
  4. end to end, the issue's board: the eSIM under the M.2 card's cover is
     mountable, and the emitted roster owes it no edge.

    python3 tests/test_1587_chip_interface_word.py
"""
import json
import os
import shutil
import sys
import tempfile
from types import SimpleNamespace

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from placement.part_class import classify_part                # noqa: E402
from run_utils import check as run_check, evidence, tool      # noqa: E402

FAILS = []


def check(name, ok, detail=''):
    print(f"  {'PASS' if ok else 'FAIL'}  {name}"
          + (f"  ({detail})" if detail and not ok else ''))
    if not ok:
        FAILS.append(name)


def _pad(pf=None):
    return SimpleNamespace(pad_type='smd', pinfunction=pf, layers=['F.Cu'],
                           global_x=0.0, global_y=0.0)


def _both(name, ref, pins=('IO1', 'GND')):
    """The class on the CLI's `lib:item` and the GUI's bare item."""
    out = []
    for n in (name, name.rsplit(':', 1)[-1]):
        fp = SimpleNamespace(footprint_name=n, pads=[_pad(p) for p in pins])
        out.append(classify_part(fp, ref).name)
    return tuple(out)


def _part(ref, x, y, name, crt, pads):
    t = (f'  (footprint "{name}" (layer "F.Cu") (at {x} {y})\n'
         f'    (property "Reference" "{ref}" (at 0 0) (layer "F.SilkS") '
         f'(hide yes))\n'
         f'    (fp_rect (start {-crt[0]} {-crt[1]}) (end {crt[0]} {crt[1]}) '
         f'(stroke (width 0.05) (type default)) (layer "F.CrtYd"))\n')
    for n, (px, py) in enumerate(pads, 1):
        t += (f'    (pad "{n}" smd rect (at {px} {py}) (size 0.8 0.5) '
              f'(layers "F.Cu" "F.Paste" "F.Mask"))\n')
    return t + '  )\n'


def _board(path):
    """The issue's board: an M.2 socket J1 and an MFF2 eSIM IC1 (two rows of
    four pads, no mechanical pad) in an 80 x 60 mm outline."""
    mff2 = ([(-2.8, -1.905 + 1.27 * k) for k in range(4)]
            + [(2.8, 1.905 - 1.27 * k) for k in range(4)])
    socket = [(0.0, -10 + k) for k in range(21)]
    out = ['(kicad_pcb (version 20241229) (generator "t1587")\n'
           '  (net 0 "")\n']
    for a, b in (((0, 0), (80, 0)), ((80, 0), (80, 60)), ((80, 60), (0, 60)),
                 ((0, 60), (0, 0))):
        out.append(f'  (gr_line (start {a[0]} {a[1]}) (end {b[0]} {b[1]}) '
                   f'(stroke (width 0.05) (type default)) '
                   f'(layer "Edge.Cuts"))\n')
    out.append(_part('J1', 20, 30, 'Connector:M.2_Socket_KeyE', (2.5, 12),
                     socket))
    out.append(_part('IC1', 40, 30, 'Security:SIM_MFF2-8', (3.2, 3.15), mff2))
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(''.join(out) + ')\n')


def main():
    # 1. a chip named for its interface
    for name, ref in (('Security:SIM_MFF2-8', 'IC1'),
                      ('house:USB-ESD-6', 'U1'),
                      ('house:USB_Switch', 'U1'),
                      ('house:SIM_Switch_4P', 'IC2')):
        got = _both(name, ref)
        check(f'{name} ({ref}) is no edge receptacle on either front',
              'edge_receptacle' not in got, got)
    for name, ref in (('Security:SIM_MFF2-8', 'IC1'),
                      ('house:USB-ESD-6', 'U1')):
        got = _both(name, ref)
        check(f'{name} ({ref}) has no class at all', got == (None, None), got)

    # 2. MFF2 is a chip package, whatever the reference
    for name, ref in (('Security:SIM_MFF2-8', 'J9'),
                      ('house:eSIM_MFF2', 'X1'),
                      ('house:SIM-MFF-2_8P', 'SIM1')):
        got = _both(name, ref)
        check(f'{name} ({ref}) is a chip package', got == (None, None), got)

    # 3. a connector spelling keeps the class on a chip's reference; the word
    #    still counts on any other one
    for name, ref in (
            ('RF_Module:IQRF_TRx2DA_KON-SIM-01', 'U4'),
            ('RF_Module:IQRF_TRx2D_KON-SIM-01', 'IC3'),
            ('Connector_USB:USB_Micro-B_Molex_47346-0001', 'U9'),
            ('Connector_USB:USB_A_Molex_67643_Horizontal', 'U2'),
            ('Connector_USB:USB_B_Lumberg_2411_02_Horizontal', 'U2'),
            ('Connector_USB:USB_Mini-B_Lumberg_2486_01_Horizontal', 'U2'),
            ('Connector_USB:USB_C_Plug_Molex_105444', 'U2'),
            ('easyeda2kicad:USB-TYPE-C-SMD_C9900008089', 'U1'),
            ('footprints:simple-usb-c', 'U2'),
            ('Viktor:MICRO-USB-SMD_HRS-ZX62D-B-5P830', 'U1'),
            ('house:Nano_SIM_6P', 'U7'),
            ('usb_otg:USB-MICRO-B-FCI-10118192-0001LF', 'US1')):
        got = _both(name, ref, (None, None))
        check(f'{name} ({ref}) stays an edge receptacle on both fronts',
              got == ('edge_receptacle', 'edge_receptacle'), got)

    # 4. end to end on the issue's board
    tmp = tempfile.mkdtemp(prefix='t1587_')
    try:
        board = os.path.join(tmp, 'esim.kicad_pcb')
        _board(board)
        intent = os.path.join(tmp, 'intent.json')
        with open(intent, 'w', encoding='utf-8') as fh:
            json.dump({'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm',
                       'min_reader': 15,
                       'covers': [{'connector': 'J1', 'side': 'F',
                                   'axis_deg': 0, 'width_mm': 22,
                                   'length_mm': 30, 'underside_mm': 2.5,
                                   'removable': True,
                                   'source': 'M.2 2230 card'}],
                       'heights': [{'ref': 'IC1', 'height_mm': 1.0,
                                    'source': 'its datasheet'}]}, fh)
        r = run_check([sys.executable, tool('check_floorplan.py'),
                       evidence(board, 'board'), '--intent',
                       evidence(intent, 'intent')], accept=True)
        check('the grade ran the mountable rule',
              'rule(s) ran' in r.stdout, r.stdout[-600:])
        check('the eSIM is not refused under the M.2 card',
              'IC1 is refused' not in r.stdout
              and 'mountable: covers' not in r.stdout, r.stdout[-600:])
        r = run_check([sys.executable, tool('check_assembly.py'), board,
                       '--intent', intent], accept=True)
        line = next((l for l in r.stdout.splitlines()
                     if 'mountable (#1504)' in l), '')
        check('check_assembly: J1 YES with the eSIM the least room',
              'J1 YES' in line and 'IC1' in line, line or r.stdout[-600:])
        emitted = os.path.join(tmp, 'emitted.json')
        r = run_check([sys.executable, tool('check_floorplan.py'), board,
                       '--emit-intent', emitted, '--allow-unplaced'],
                      accept=True)
        with open(evidence(emitted, 'emitted intent'),
                  encoding='utf-8') as fh:
            doc = json.load(fh)
        rows = {c.get('ref') for c in doc.get('edge_connectors') or ()}
        owed = [l for l in r.stdout.splitlines()
                if 'OWED: edge_connector' in l and 'IC1' in l]
        check('the emitter neither declares nor owes the eSIM an edge',
              'IC1' not in rows and not owed, (rows, owed))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    if FAILS:
        print(f"FAILED: {len(FAILS)} check(s)")
        return 1
    print("all checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
