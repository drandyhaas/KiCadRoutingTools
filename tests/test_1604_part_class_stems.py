"""#1604: four part classes a bare-substring stem got wrong.

`classify_part` read several stems as bare substrings of the whole
lower-case `lib:item`. Each wrong class was a mechanical claim (the edge seat
and the floorplan's edge claims read `edge_actuator` and `edge_receptacle`):

  1. `tact` inside `contact`: `PhoenixContact_MC_1,5_...` was an
     `edge_actuator`; it is a terminal block, `connector_affinity`;
  2. a chip package named for a switch IC (`USB_Switch_TSSOP-10`,
     `SOT-23-5_Switching`) was an `edge_actuator`; it is no class;
  3. the chip-package veto read the LIBRARY too, so every part of a house
     library named for a chip (`tomu-fpga:`) was vetoed: its `USB-B` is an
     `edge_receptacle`, its pin headers `connector_affinity`;
  4. a logo (`Symbol:USB_Logo`, `USB_Logo_Copper`) or any pad-less
     footprint was an `edge_receptacle`; it is no class.

Guards for what must NOT change: real switches and buttons (`SW_SPST`,
`Button_Switch_*`, a tactile `Tact` item), real USB connectors, Phoenix
terminal blocks, and the parts whose LIBRARY is the only sign of what they
are (`Jack:`, `ScrewTerminals:`, a `switches` library): the connector and
actuator stems still read the whole `lib:item`, which both fronts carry.

    python3 tests/test_1604_part_class_stems.py
"""
import os
import sys
from types import SimpleNamespace

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from placement.part_class import classify_part                # noqa: E402

FAILS = []


def check(name, ok, detail=''):
    print(f"  {'PASS' if ok else 'FAIL'}  {name}"
          + (f"  ({detail})" if detail and not ok else ''))
    if not ok:
        FAILS.append(name)


def _cls(name, ref, pad_type='smd', n=2):
    pads = [SimpleNamespace(pad_type=pad_type, pinfunction='',
                            drill=0.8 if pad_type == 'thru_hole' else 0.0,
                            layers=['F.Cu'], global_x=float(k), global_y=0.0)
            for k in range(n)]
    return classify_part(SimpleNamespace(footprint_name=name, pads=pads),
                         ref).name


def expect(label, name, ref, want, pad_type='smd', n=2, item_too=True):
    """The class on `lib:item` and, where the item alone carries the
    evidence, on the bare item too."""
    got = _cls(name, ref, pad_type, n)
    check(f'{label}: {ref} {name} -> {want}', got == want, f'got {got}')
    if item_too and ':' in name:
        item = name.rsplit(':', 1)[-1]
        got = _cls(item, ref, pad_type, n)
        check(f'{label}: {ref} {item} -> {want}', got == want, f'got {got}')


print('#1604 the issue\'s parts')
expect('tact in contact', 'Connector_Phoenix_MC:PhoenixContact_MC_1,5_2-G-'
       '3.81_1x02_P3.81mm_Horizontal', 'J2', 'connector_affinity',
       'thru_hole')
expect('tact in contact (corpus)', 'Connector_Phoenix_MSTB:PhoenixContact_'
       'MSTBA_2,5_2-G-5,08_1x02_P5.08mm_Horizontal', 'J4',
       'connector_affinity', 'thru_hole')
expect('switch IC', 'Package_SO:USB_Switch_TSSOP-10', 'U3', None)
expect('switching regulator', 'Package_TO_SOT_SMD:SOT-23-5_Switching', 'U4',
       None)
expect('chip-named library', 'tomu-fpga:USB-B', 'J5', 'edge_receptacle')
expect('chip-named library, U ref', 'tomu-fpga:USB-B', 'U9',
       'edge_receptacle', n=8)
expect('chip-named library header', 'tomu-fpga:PinHeader_2x20_P2.54mm_'
       'Vertical', 'J1', 'connector_affinity', 'thru_hole', 40)
expect('logo with pads', 'Symbol:USB_Logo', 'G1', None)
expect('logo, pad-less', 'Symbol:USB_Logo', 'G1', None, n=0)
expect('copper-art logo', 'Symbol:USB_Logo_Copper', 'LOGO1', None, n=0)
expect('pad-less art named for a jack', 'art:Audio_Jack_Outline', 'G2', None,
       n=0)

print('guards: what must not change')
expect('SPST switch', 'Button_Switch_SMD:SW_SPST_PTS645', 'SW1',
       'edge_actuator')
expect('push button', 'Button_Switch_THT:SW_PUSH_6mm', 'SW2',
       'edge_actuator', 'thru_hole')
expect('tactile item', 'house:KMR2_Tact', 'S1', 'edge_actuator')
expect('tactile word', 'house:TACTILE_6x6', 'S2', 'edge_actuator')
expect('slide switch', 'Button_Switch_SMD:SW_SPDT_PCM12', 'SW3',
       'edge_actuator')
expect('DIP switch', 'Button_Switch_THT:SW_DIP_SPSTx04_Slide_9.78x12.34mm_'
       'W7.62mm_P2.54mm', 'SW4', 'edge_actuator', 'thru_hole', 8)
expect('USB-C receptacle', 'Connector_USB:USB_C_Receptacle_HRO_TYPE-C-31-M-12',
       'J1', 'edge_receptacle', n=16)
expect('micro-USB', 'Connector_USB:USB_Micro-B_Molex-105017-0001', 'J3',
       'edge_receptacle', n=6)
expect('Phoenix terminal block', 'TerminalBlock_Phoenix:TerminalBlock_'
       'Phoenix_MKDS-1,5-2_1x02_P5.00mm_Horizontal', 'J6',
       'connector_affinity', 'thru_hole')
expect('a library the only evidence: jack', 'Jack:C18185602', 'J1',
       'edge_receptacle', item_too=False)
expect('a library the only evidence: terminal', 'ScrewTerminals:Generic-'
       '5.08-2P', 'J16', 'connector_affinity', 'thru_hole', item_too=False)
expect('a library the only evidence: switch', 'switches:Kailh_PG1232',
       'SW1', 'edge_actuator', item_too=False)
expect('ESD array named for USB', 'Package_TO_SOT_SMD:SOT-23-6_USBLC6-2SC6',
       'U5', None, n=6)
expect('QFN chip', 'Package_DFN_QFN:QFN-24-1EP_4x4mm_P0.5mm', 'U1', None,
       n=25)
expect('mounting hole, pad-less', 'MountingHole:MountingHole_3.2mm_M3', 'H1',
       'mount_hole', n=0)
expect('fiducial', 'Fiducial:Fiducial_1mm_Mask2mm', 'FID1', 'fiducial', n=1)
expect('analog-out header is no logo', 'house:ANALOGOUT_Conn_1x03', 'J8',
       'connector_affinity', 'thru_hole', 3)

if FAILS:
    print(f'\nFAIL: {len(FAILS)} check(s): {FAILS}')
    sys.exit(1)
print('\nPASS')
