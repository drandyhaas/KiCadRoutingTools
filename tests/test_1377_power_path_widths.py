#!/usr/bin/env python3
"""#1377 #1378 #1380 #1381: power-path analysis sizes a net from the loads on it.

The /analyze-power-nets tool turns classified parts into per-net track widths.
What must hold:

  * #1377 a sink's `current_rating_ma` is its TOTAL supply current and is
    charged once per supply net, however many pads it has there; a source's
    rating is a limit and is never added; a ground net carries the sinks that
    have a pad on it, once each (an isolated ground carries nothing);
  * #1378 a classified part reaches the nets whatever its pintype (rail-named
    nets, or the nets the agent names), a 3-terminal pass-through conducts, and
    a classified part that reaches no net WARNS;
  * #1380 every sink on a rail is traced, not one per net, and two sources on
    one net are both path ends;
  * #1381 a regulator is entered from its output to its input and never the
    other way, the search stops at the first root source, and a source on the
    sink's own net is a one-net path.

The unit rows build the parts by hand; the last rows run the tool on tracked
boards with a classification as the skill asks for it.

    python3 -X utf8 tests/test_1377_power_path_widths.py
"""
import os
import sys
import types
import warnings

RUN_ALL_TIMEOUT = 300

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, HERE)

import analyze_power_paths as app  # noqa: E402
from analyze_power_paths import (ComponentInfo, ComponentRole,  # noqa: E402
                                 classify_component, trace_power_paths,
                                 get_power_net_recommendations)

fails = []


def check(name, cond, detail=''):
    print(('PASS: ' if cond else 'FAIL: ') + name
          + (f'  {detail}' if detail else ''))
    if not cond:
        fails.append(name)


def part(ref, pads, role=ComponentRole.UNKNOWN, ma=None):
    """pads: [(number, net, pinfunction, pintype)]"""
    c = ComponentInfo(ref=ref, value=ref, footprint_name='X', pad_count=len(pads),
                      net_connections={n: net for n, net, _, _ in pads},
                      pin_functions={n: f for n, _, f, _ in pads},
                      pin_types={n: t for n, _, _, t in pads})
    c.role, c.current_rating_ma, c.classified = role, ma, True
    return c


def run(parts, **kw):
    comps = {p.ref: p for p in parts}
    names = sorted({n for p in parts for n in p.net_connections.values()})
    pcb = types.SimpleNamespace(nets={i + 1: types.SimpleNamespace(name=n)
                                      for i, n in enumerate(names)})
    paths = trace_power_paths(pcb, comps)
    with warnings.catch_warnings(record=True) as w:
        warnings.simplefilter('always')
        rec = get_power_net_recommendations(pcb, comps, paths, **kw)
    return paths, rec, [str(x.message) for x in w]


SINK, SRC, PASS = (ComponentRole.CURRENT_SINK, ComponentRole.POWER_SOURCE,
                   ComponentRole.PASS_THROUGH)


def mcu(ref, ma, rail='+3V3', n=5, gnd='GND'):
    pads = [(f'v{i}', rail, 'VDD', 'power_in') for i in range(n)]
    pads += [(f'g{i}', gnd, 'VSS', 'power_in') for i in range(4)]
    return part(ref, pads, SINK, ma)


def ldo(ref, vin, vout, ma=300):
    return part(ref, [('1', vin, 'IN', 'power_in'), ('2', 'GND', 'GND', 'power_in'),
                      ('3', vout, 'OUT', 'power_out')], SRC, ma)


# ---------------------------------------------------------------- #1377
def t_charge_once():
    # 40 mA IC with five VDD pads: 40 mA, not 200 -> 0.3 mm
    _, rec, _ = run([mcu('U1', 40)], min_current_ma=10)
    check('#1377 a multi-pad IC is charged once per net',
          rec.get('+3V3') == 0.30 and rec.get('GND') == 0.30, str(rec))

    # a 300 mA regulator feeding a 40 mA load: the rating is a limit
    _, rec, _ = run([mcu('U1', 40), ldo('U2', '+5V', '+3V3', 300)], min_current_ma=10)
    check('#1377 a source rating is never added to the sinks it feeds',
          rec.get('+3V3') == 0.30, str(rec))

    # two loads on one rail add; each only once
    _, rec, _ = run([mcu('U1', 40), mcu('U2', 40)], min_current_ma=10)
    check('#1377 two loads add (80 mA -> 0.5 mm)', rec.get('+3V3') == 0.50, str(rec))

    # a ground with no load on it is not sized from the board's total
    iso = part('J1', [('1', '/ISO_GND', 'GND', 'passive')], SRC, 500)
    _, rec, _ = run([mcu('U1', 400), iso], min_current_ma=10)
    check('#1377 an isolated ground gets no load',
          '/ISO_GND' not in rec and rec.get('GND') == 0.70, str(rec))

    # a ground is sized by the sinks on it
    _, rec, _ = run([mcu('U1', 400, gnd='GND'), mcu('U2', 40, rail='+1V8', gnd='GNDA')],
                    min_current_ma=10)
    check('#1377 each ground carries the sinks on it',
          rec.get('GND') == 0.70 and rec.get('GNDA') == 0.30, str(rec))


# ---------------------------------------------------------------- #1378
def t_classified_reach_nets():
    # no pintype and no pinfunction: rail-named nets still count
    bare = part('U1', [('1', '+3V3', '', ''), ('2', 'GND', '', ''),
                       ('3', '/SIG', '', '')], SINK, 150)
    _, rec, w = run([bare], min_current_ma=10)
    check('#1378 untyped pads on rail-named nets are charged',
          rec.get('+3V3') == 0.70 and rec.get('GND') == 0.70
          and '/SIG' not in rec and not w, f'{rec} {w}')

    # nets that look like nothing: the agent names them
    odd = part('P4', [('1', '/HEAT_A', '', 'input'), ('2', '/HEAT_B', '', 'input'),
                      ('3', '/SIG', '', 'input')], SRC, 300)
    tube = part('V1', [('1', '/HEAT_A', '', 'input'), ('2', '/HEAT_B', '', 'input')],
                SINK, 300)
    _, rec, w = run([odd, tube])
    check('#1378 an unreadable classified part warns',
          len(w) == 1 and 'V1' in w[0], f'{rec} {w}')
    classify_component({'V1': tube}, 'V1', SINK, current_rating_ma=300,
                       supply_nets=['/HEAT_A', '/HEAT_B'])
    classify_component({'P4': odd}, 'P4', SRC, current_rating_ma=300,
                       supply_nets=['/HEAT_A', '/HEAT_B'])
    paths, rec, w = run([odd, tube])
    check('#1378 supply_nets= names the nets: path found, widths given, no warning',
          rec.get('/HEAT_A') == 0.70 and rec.get('/HEAT_B') == 0.70 and not w
          and {x.source_component for x in paths} == {'P4'}, f'{rec} {w} {paths}')
    try:
        classify_component({'V1': tube}, 'V1', SINK, supply_nets=['/NOPE'])
        check('#1378 supply_nets= on a net the part is not on is refused', False)
    except ValueError:
        check('#1378 supply_nets= on a net the part is not on is refused', True)

    # a 3-terminal FET (drain, source, gate) conducts between its power nets
    fet = part('Q1', [('1', '+BATT', '', 'passive'), ('2', '/VSW', '', 'passive'),
                      ('3', '/GATE', '', 'passive')], PASS)
    bat = part('J1', [('1', '+BATT', '', 'passive'), ('2', 'GND', '', 'passive')], SRC, 500)
    load = mcu('U1', 200, rail='/VSW')
    paths, rec, w = run([fet, bat, load])
    check('#1378 a 3-terminal pass-through conducts',
          [(p.source_component, p.nets_in_path) for p in paths]
          == [('J1', ['+BATT', '/VSW'])] and rec.get('+BATT') == 0.70, f'{paths} {rec}')

    # a pass-through that joins fewer than two nets warns when classified
    dead = part('FB1', [('1', '+3V3', '', 'passive'), ('2', 'GND', '', 'passive')], PASS)
    _, _, w = run([dead])
    check('#1378 a pass-through reaching one net warns', len(w) == 1 and 'FB1' in w[0], str(w))


# ---------------------------------------------------------------- #1380
def t_every_sink_traced():
    src = part('J1', [('1', '/VIN', '', 'passive'), ('2', 'GND', '', 'passive')], SRC, 1000)
    dio = part('D1', [('1', '/VIN', '', 'passive'), ('2', '+5V', '', 'passive')], PASS)
    paths, rec, _ = run([src, dio, mcu('U1', 22, rail='+5V'), mcu('U2', 22, rail='+5V')],
                        min_current_ma=10)
    check('#1380 both loads behind a diode are traced',
          sorted(p.sink_component for p in paths) == ['U1', 'U2'], str(paths))
    check('#1380 the input net carries both (44 mA -> 0.3 mm, 44 not 22)',
          rec.get('/VIN') == 0.30 and rec.get('+5V') == 0.30, str(rec))
    _, rec, _ = run([src, dio, mcu('U1', 45, rail='+5V'), mcu('U2', 45, rail='+5V')],
                    min_current_ma=10)
    check('#1380 ...and 90 mA on the input net is 0.5 mm', rec.get('/VIN') == 0.50, str(rec))

    # two sources on one net are both path ends
    tb = part('TB1', [('1', '/VIN', '', 'passive')], SRC, 1000)
    paths, _, _ = run([src, tb, dio, mcu('U1', 22, rail='+5V')])
    check('#1380 two sources on one net are both path ends',
          {p.source_component for p in paths} == {'J1', 'TB1'}, str(paths))


# ---------------------------------------------------------------- #1381
def t_directed_regulators():
    j1 = part('J1', [('1', '+5V', 'VBUS', 'passive'), ('2', 'GND', 'GND', 'passive')], SRC, 500)
    reg33 = ldo('U8', '+5V', '+3V3')
    reg12 = ldo('U36', '+5V', '+1V2')
    reg_b = ldo('U14', '+5V', '/VBANK')
    load12 = mcu('U30', 100, rail='+1V2')
    paths, rec, _ = run([j1, reg33, reg12, reg_b, load12, mcu('U13', 100, rail='+3V3')],
                        min_current_ma=10)
    got = sorted((p.sink_component, p.source_component, tuple(p.nets_in_path)) for p in paths)
    check('#1381 each load traces to its own regulator chain, nothing else',
          got == [('U13', 'J1', ('+5V', '+3V3')), ('U30', 'J1', ('+5V', '+1V2'))], str(got))
    check('#1381 the shared input carries the two loads once (200 mA)',
          rec.get('+5V') == 0.70, str(rec))
    check('#1381 a regulator output nobody draws on gets no width',
          '/VBANK' not in rec, str(rec))

    # a source on the sink's own net is a one-net path
    paths, _, _ = run([reg12, load12])
    check('#1381 the sink-rail regulator is a path end',
          [(p.source_component, p.nets_in_path) for p in paths]
          == [('U36', ['+5V', '+1V2'])], str(paths))

    # the search stops at the first root: a second source behind it is not a path
    far = part('J9', [('1', '+3V3', '', 'passive')], SRC, 1000)
    paths, _, _ = run([far, ldo('U8', '+5V', '+3V3'), j1, mcu('U13', 100, rail='+3V3')])
    check('#1381 stop at the first root source',
          {p.source_component for p in paths} == {'J9'}, str(paths))

    # connector data nets are not source terminals when a rail is recognised
    usb = part('J2', [('1', '/xVBUS', 'VBUS', 'passive'), ('2', '/USB_N', 'D-', 'passive'),
                      ('3', 'GND', 'GND', 'passive')], SRC, 500)
    sw = part('U15', [('1', '/xVBUS', '', 'passive'), ('2', '+5V', '', 'passive'),
                      ('3', '/USB_N', '', 'passive')], PASS)
    paths, rec, _ = run([usb, sw, mcu('U7', 100, rail='+5V')], min_current_ma=10)
    check('#1381 a data net is not a source terminal',
          [(p.source_component, p.nets_in_path) for p in paths]
          == [('J2', ['/xVBUS', '+5V'])] and '/USB_N' not in rec, f'{paths} {rec}')


# ------------------------------------------------------------ tracked boards
def t_boards():
    import contextlib
    import io

    def load(name, classes):
        with contextlib.redirect_stdout(io.StringIO()):
            comps, pcb = app.analyze_pcb(os.path.join(ROOT, 'kicad_files', name))
        for ref, role, ma in classes:
            classify_component(comps, ref, role, current_rating_ma=ma)
        paths = trace_power_paths(pcb, comps)
        with warnings.catch_warnings():
            warnings.simplefilter('ignore')
            return comps, paths, get_power_net_recommendations(pcb, comps, paths)

    # the skill's own worked example: a barrel jack -> switch -> fuse -> LDO
    _, paths, rec = load('kit-dev-coldfire-xilinx_5213.kicad_pcb', [
        ('J201', SRC, 1000), ('TB201', SRC, 1000), ('U102', SINK, 200), ('U301', SINK, 150)])
    chain = [p for p in paths if p.sink_component == 'U301']
    check('skill example: U301 is traced back to the jack and the terminal block '
          '(one net) through switch, fuse, LDO',
          {p.source_component for p in chain} == {'J201', 'TB201'}
          and all(p.components_in_path[-1] == 'VR201'
                  and {'SW_ONOFF201', 'F201'} <= set(p.components_in_path)
                  for p in chain), str(chain))
    check('skill example: +3.3V is sized from the loads, not 4850 mA',
          rec['+3.3V'] <= 1.0 and rec['GND'] <= 1.2, str(rec.get('+3.3V')))

    # a USB-C input behind a power switch, a 3-pin FET and a regulator
    comps, paths, rec = load('watchy.kicad_pcb', [
        ('J1', SRC, 500), ('J2', SRC, 500), ('Q1', PASS, 500), ('D1', PASS, 500),
        ('U2', SRC, 500), ('U4', SINK, 350)])
    check('3-pin FET conducts: the battery connector is a path end',
          any(p.source_component == 'J1' for p in paths), str([(p.source_component,
                                                                 p.nets_in_path) for p in paths]))

    # untyped pads: classified parts reach the rails by net name
    _, paths, rec = load('tigard.kicad_pcb', [
        ('U1', SRC, 300), ('U2', SRC, 300), ('U3', SINK, 150)])
    check('untyped board: rails other than GND are sized',
          {'+3V3', '+5V', 'GND'} <= set(rec), str(rec))

    # a regulator chain on a board with typed pins
    _, paths, rec = load('glasgow_revC.kicad_pcb', [
        ('J1', SRC, 500), ('U15', PASS, 500), ('U8', SRC, 500), ('U36', SRC, 300),
        ('U14', SRC, 150), ('U31', SRC, 150), ('U1', SINK, 85), ('U30', SINK, 100)])
    u30 = sorted(tuple(p.nets_in_path) for p in paths if p.sink_component == 'U30')
    check('regulator chain: the 1.2 V load reaches its own LDO, never a sibling',
          any(n[-1] == '+1V2' for n in u30) and all(
              'Net-(U14-OUT)' not in n and 'Net-(U31-OUT)' not in n for n in u30), str(u30))
    check('regulator chain: a load on +1V2 is not charged 1200 mA there',
          rec.get('+1V2', 0) <= 0.7, str(rec.get('+1V2')))


for fn in (t_charge_once, t_classified_reach_nets, t_every_sink_traced,
           t_directed_regulators, t_boards):
    fn()

print('\nFAILED: ' + ', '.join(fails) if fails else '\nall passed')
sys.exit(1 if fails else 0)
