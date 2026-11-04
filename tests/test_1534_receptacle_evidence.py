"""#1534: an interface word in a footprint name is not a receptacle on its own.

`classify_part` made any footprint whose name contained `usb` or `sim` an
edge receptacle: a USB ESD array named by its part number, a USB common-mode
choke, an LED, a cellular or GNSS module. The emitter then declared it on an
edge, the OWED roster asked for an edge for it, the grade failed the human
layout, and the heights rule treated it as reached from above. Its pin
channel, meanwhile, matched receptacle pins by exact name only, so a USB-C
receptacle whose pins read `CC1_A5` / `CC2_B5` / `SHIELD_SH` (a symbol's
stacked-pin names) had no class once its footprint was named by part number.

Pinned here:
  1. a name that only says what INTERFACE a part serves (`usb`, `sim`) makes
     it a receptacle -- or a connector at all -- only beside connector
     evidence: a connector reference prefix, a reference naming the
     interface, a connector word in the name, the stem standing as a word of
     the item (`USB-MICRO-B`, not a part number like `USBLC6`), or
     receptacle pins;
  2. stock connector names keep their class, on the CLI's `lib:item` and
     the GUI's bare item alike;
  3. receptacle pins are read before KiCad's `_<suffix>`: `CC1_A5` is CC1;
  4. a SON package (USON/WSON/...) is an IC package, whatever its pins;
  5. end to end on a tracked board with two footprints renamed to their
     part numbers: the emitter does not declare the ESD array a receptacle,
     and does declare the USB-C receptacle one by its pins.

    python3 tests/test_1534_receptacle_evidence.py
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

BOARD = os.path.join(ROOT, 'kicad_files',
                     'rp2350_fpga_eensy_prePlane.kicad_pcb')
FAILS = []


def check(name, ok, detail=''):
    print(f"  {'PASS' if ok else 'FAIL'}  {name}"
          + (f"  ({detail})" if detail and not ok else ''))
    if not ok:
        FAILS.append(name)


def _pad(pf=None):
    return SimpleNamespace(pad_type='smd', pinfunction=pf, layers=['F.Cu'],
                           global_x=0.0, global_y=0.0)


def _cls(name, ref, pins=(None, None)):
    fp = SimpleNamespace(footprint_name=name, pads=[_pad(p) for p in pins])
    return classify_part(fp, ref).name


def _both(name, ref, pins=(None, None)):
    """The class on the CLI's `lib:item` and the GUI's bare item."""
    return _cls(name, ref, pins), _cls(name.rsplit(':', 1)[-1], ref, pins)


def main():
    # 1. an interface word alone
    for name, ref in (('Power_Protection:USBLC6-2P6', 'U3'),
                      ('Inductor_SMD:L_CommonModeChoke_Coilcraft_0603USB',
                       'L1'),
                      ('LED_SMD:LED_Lumex_SML-LX0404SIUPGUSB', 'D1'),
                      ('RF_GSM:SIMCom_SIM800C', 'U1'),
                      ('RF_GPS:SIM28ML', 'U2'),
                      ('house:ESIM_MFF2', 'U5')):
        got = _both(name, ref, ('IO1', 'GND'))
        check(f'{name} ({ref}) is no edge part and no connector',
              got == (None, None), got)

    # 2. connector evidence keeps the class
    for name, ref in (
            ('Connector_USB:USB_C_Receptacle_HRO_TYPE-C-31-M-12', 'J1'),
            ('Connector_USB:USB_Micro-B_Molex_47346-0001', 'J2'),
            ('Connector_USB:USB_Micro-B_Molex_47346-0001', 'USB1'),
            ('Connector_USB:USB_A_Molex_67643_Horizontal', 'P3'),
            ('Connector_Card:Micro_SIM_Card_Holder', 'U7'),
            ('house:USB_C_Receptacle_16P', 'CN2'),
            # the interface as a WORD of the item, whatever the reference
            ('usb_otg:USB-MICRO-B-FCI-10118192-0001LF', 'US1'),
            ('RF_Module:IQRF_TRx2DA_KON-SIM-01', 'U4')):
        got = _both(name, ref)
        check(f'{name} ({ref}) stays an edge receptacle on both fronts',
              got == ('edge_receptacle', 'edge_receptacle'), got)
    got = _both('Connector_USB:USB_Micro-B_Molex_47346-0001', 'U9')
    check('a stock USB connector library still classes its part on the CLI',
          got[0] == 'edge_receptacle', got)
    for name in ('Connector_HDMI:HDMI_A_Molex_208658-1001_Horizontal',
                 'Connector_RJ:RJ45_Amphenol_54602-x08_Horizontal'):
        got = _both(name, 'U1')
        check(f'{name} keeps its class without a connector reference',
              got == ('edge_receptacle', 'edge_receptacle'), got)

    # 3. receptacle pins before the `_<suffix>`
    pins = ('CC1_A5', 'CC2_B5', 'SHIELD_SH', 'VBUS_A4')
    got = _both('HRO:TYPE-C-31-M-12', 'J1', pins)
    check('a USB-C named by part number classes by its CC1_A5 / CC2_B5 pins',
          got == ('edge_receptacle', 'edge_receptacle'), got)
    fp = SimpleNamespace(footprint_name='HRO:TYPE-C-31-M-12',
                         pads=[_pad(p) for p in pins])
    check('...at high confidence',
          classify_part(fp, 'J1').confidence == 'high',
          classify_part(fp, 'J1'))
    got = _both('HRO:TYPE-C-31-M-12', 'J1', ('VBUS_A4', 'GND'))
    check('...and without receptacle pins it has no class', got[0] is None,
          got)

    # 4. a SON package is an IC package
    for name in ('Package_SON:USON-10_2.5x1.0mm_P0.5mm',
                 'Package_SON:WSON-8-1EP_2x2mm_P0.5mm_EP0.9x1.6mm',
                 'Package_SON:Texas_S-PVSON-N8'):
        got = _both(name, 'U6', ('CC1', 'CC2'))
        check(f'{name} with CC pins is no receptacle', got == (None, None),
              got)

    # 5. end to end
    evidence(BOARD, 'tracked board')
    with open(BOARD, encoding='utf-8') as fh:
        text = fh.read()
    esd = '(footprint "Package_TO_SOT_SMD:SOT-666"'
    usbc = ('(footprint "amemb-Connector_USB:USB_C_Receptacle_HRO_'
            'TYPE-C-31-M-12-Edge_Silkscreen_Removed"')
    check('the tracked board carries the two footprints',
          text.count(esd) == 1 and text.count(usbc) == 1
          and 'pinfunction "CC1_A5"' in text)
    text = (text.replace(esd, '(footprint "Power_Protection:USBLC6-2P6"')
            .replace(usbc, '(footprint "HRO:TYPE-C-31-M-12"'))
    tmp = tempfile.mkdtemp(prefix='t1534_')
    try:
        board = os.path.join(tmp, 'renamed.kicad_pcb')
        with open(board, 'w', encoding='utf-8') as fh:
            fh.write(text)
        emitted = os.path.join(tmp, 'emitted.json')
        run_check([sys.executable, tool('check_floorplan.py'), board,
                   '--emit-intent', emitted, '--declare-classes'],
                  accept=True)
        with open(evidence(emitted, 'emitted intent')) as fh:
            rows = {c['ref']: c for c in json.load(fh).get('edge_connectors')
                    or ()}
        renamed = [r for r, c in rows.items()
                   if c.get('class') == 'edge_receptacle']
        esd_ref = next((r for r in ('U3',) if r in rows), None)
        check('the ESD array is not declared an edge receptacle',
              esd_ref is None
              or rows[esd_ref].get('class') != 'edge_receptacle',
              rows.get('U3'))
        check('the USB-C receptacle is, by its pins',
              rows.get('J1', {}).get('class') == 'edge_receptacle',
              renamed)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    if FAILS:
        print(f"FAILED: {len(FAILS)} check(s)")
        return 1
    print("all checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
