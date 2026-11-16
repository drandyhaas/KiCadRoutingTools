#!/usr/bin/env python3
"""A net reads as a rail or ground by NAME only on its leaf, by chunk, and not
when it also names a signal about the supply (#1835).

find_power_nets matched VIN/VBUS/VCC/VDD/VBAT/GND as substrings of the whole
hierarchical name, so `/DRIVING_LED` (DRI-VIN-G) was a rail, every `VBUS_EN`,
`VBAT_SENSE`, `PG_VDD` too, `NN_GND_SCL` was ground, and a net under a
`/VBUS_SWITCH/` sheet was a rail whatever its own name. Those reached
power_net_evidence (return-via coverage, --add-gnd-vias, the Planes tab, the
qfn power-stub note) and autoroute's --power-nets (routed at 0.3 mm).

  1. find_power_nets over a table of names: real rails and grounds in every
     spelling stay, enable / power-good / sense / detect / feedback / bus
     lines and path-only matches leave;
  2. the in-repo sonde_u board: `/VCC_SENSE-ERROR*` is no longer power by
     name, GND and VCC still are;
  3. autoroute on a generated board: an enable line with 3 pads stays out of
     `power_nets`, the rail beside it stays in.

    python3 tests/test_1835_power_name_tokens.py
"""
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
from types import SimpleNamespace as NS

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from kicad_parser import parse_kicad_pcb                         # noqa: E402
from list_nets import find_power_nets, power_net_evidence         # noqa: E402
import run_utils                                                 # noqa: E402

failures = []


def ok(label, cond, detail=''):
    print(f"  [{'ok' if cond else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not cond:
        failures.append(label)


GROUND = ['GND', '/GND', 'AGND', 'GNDA', 'DGND', 'PGND', 'GND_A', 'GNDPLL1', 'CPVSS',
          'VSS', 'VSSA', '/Power/GND_ISO', 'Net-(U5-GND)', 'MotorGND', 'AIN_GND[3]']
RAIL = ['+3V3', '/Power/+5V_USB', '-12V', '3V3', '1V8', 'VCC', 'VCC_3V3', 'VCC1V8',
        'VDD_IO', 'VDDIO', 'VDDQ', 'AVDD', 'DVDD_1V8', 'VCCINT', 'VCCAUX', 'MGTAVCC',
        'INTVCC', 'VBUS', 'USB_VBUS', 'VBUS_IN', 'VBATT', 'VBAT', 'VIN', 'VIN1',
        'VINT', 'AVIN', 'Net-(U1-VCC)', 'HDMI_+5V', 'BMS.+5V', '/SEN1_VCC',
        '+3V3_SENS', 'VBus_Slave', 'eMMC_VDDI', '+5V_AON']
NEITHER = ['/DRIVING_LED', '/LED_DRIVE', '/VBUS_EN', '~{VBUS_EN}', '/VBAT_SENSE',
           '/PG_VDD', 'NN_GND_SCL', '/DDR5/VSS_DET1', '/Debug/GNDDetect',
           'USB_VBUSDET', 'PMU-VBUSEN', '/Power Supply/VCCOEN_PS_M2C',
           '/MCU/PG_+3.3VA', '/Supply/DC-DC VCCINT/SET', '/Power/VCC_1V8_en',
           '/Supply/VDDQ_SEN_+', '/VBUS_ALERT_L', '/Power/DVDD_FB',
           'VDDR_PGOOD', 'Net-(IC4-VBAT_OV)', 'SPI_VCC_OK', '/USB/VBUS_MON_P1',
           '/Supply/VCC_IO_EN', '/Power/VCCD_PG', '/USB PD/VBUS_DISCH',
           '/USB PHY/USB_HOST_VBUS_FLG', '/SoM_Power/VIN_PWR_ON']


def table_checks():
    print('1. find_power_nets over a table of names')
    names = GROUND + RAIL + NEITHER
    pad = lambda ref: NS(component_ref=ref, pintype='passive')
    pcb = NS(nets={i + 1: NS(name=n, pads=[pad('U1'), pad('R1'), pad('Q1')])
                   for i, n in enumerate(names)}, zones=[])
    gnd, vcc, _ = find_power_nets(pcb)
    gnd, vcc = {n for n, _ in gnd}, {n for n, _ in vcc}
    ok('every ground reads as ground', set(GROUND) <= gnd, repr(sorted(set(GROUND) - gnd)))
    ok('every rail reads as a rail', set(RAIL) <= vcc, repr(sorted(set(RAIL) - vcc)))
    wrong = sorted((gnd | vcc) & set(NEITHER))
    ok('no signal about a supply reads as power', not wrong, repr(wrong))


def board_checks():
    print('2. sonde_u: the sense line is not power by name')
    board = run_utils.evidence(os.path.join(ROOT, 'kicad_files', 'sonde_u.kicad_pcb'),
                               'in-repo board')
    pcb = parse_kicad_pcb(board)
    why = {pcb.nets[nid].name: w for nid, w in power_net_evidence(pcb).items()}
    ok('GND and VCC still power', 'GND' in why and 'VCC' in why, repr(why))
    ok('/VCC_SENSE-ERROR* is not', '/VCC_SENSE-ERROR*' not in why,
       repr(why.get('/VCC_SENSE-ERROR*')))


def gen(path, a, b):
    """Three 2-pin parts, each with one pad on `a` and one on `b`: 3 pads a net,
    autoroute's cut, and no power pins, so only the names decide."""
    def fp(i):
        pad = lambda n, dx, net: (f'(pad "{n}" smd rect (at {dx} 0) (size 1 1) '
                                  f'(layers "F.Cu" "F.Mask" "F.Paste") (net "{net}"))')
        return (f'(footprint "R" (layer "F.Cu") (at {10 + 5 * i} 10) '
                f'(property "Reference" "R{i}" (at 0 0) (layer "F.SilkS")) '
                f'{pad(1, -1, a)} {pad(2, 1, b)})')
    with open(path, 'w', encoding='utf-8') as f:
        f.write('(kicad_pcb (version 20260206) (generator "pcbnew") (generator_version "10.0") '
                '(general (thickness 1.6)) '
                '(layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user)) '
                + ' '.join(fp(i) for i in range(3)) +
                ' (gr_rect (start 0 0) (end 40 20) (layer "Edge.Cuts") '
                '(stroke (width 0.1) (type default)) (fill no)))\n')
    return run_utils.evidence(path, 'generated board')


def autoroute_checks(tmp):
    print('3. autoroute: an enable line is not a --power-nets rail')
    board = gen(os.path.join(tmp, 'en.kicad_pcb'), '/Supply/VCC_IO_EN', '/Supply/+3V3')
    r = subprocess.run([sys.executable, '-B', '-X', 'utf8', 'py_router/autoroute.py', board,
                        os.path.join(tmp, 'work'), '--dry-run'], cwd=ROOT,
                       capture_output=True, text=True, encoding='utf-8', errors='replace')
    out = r.stdout + r.stderr
    if 'Traceback' in out:
        raise AssertionError(f'BROKEN TEST: autoroute crashed:\n{out[-1500:]}')
    m = re.search(r'^AUTOROUTE_ANALYSIS: (.*)$', out, re.M)
    if not m:
        raise AssertionError(f'BROKEN TEST: no AUTOROUTE_ANALYSIS line:\n{out[-1500:]}')
    power = json.loads(m.group(1))['power_nets']
    ok('the rail is a power net', '/Supply/+3V3' in power, repr(power))
    ok('the enable line is not', '/Supply/VCC_IO_EN' not in power, repr(power))


def main():
    tmp = tempfile.mkdtemp(prefix='t1835_')
    try:
        table_checks()
        board_checks()
        autoroute_checks(tmp)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    if failures:
        print(f'FAILED: {len(failures)}: {failures}')
        return 1
    print('PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
