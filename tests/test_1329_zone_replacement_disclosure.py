#!/usr/bin/env python3
"""#1329: replacing a designer's same-net pour says what it changes, and why.

route_planes replaces an existing same-net pour on a layer it pours (that is
documented, and `--skip-existing-zones` keeps the pour). It reported only
"Replacing existing zone ... with new parameters". On sonde_xilinx the B.Cu
GND pour went from thermal relief to solid, clearance 0.508 -> 0.254, minimum
width 0.254 -> 0.1 and a new outline, with no word about any of them. The run
now prints each setting that changes, with the old and new values and where
the new one comes from, and records the same in JSON_SUMMARY.zone_replacements.

Checks:
  1. the parser reads a pour's pad connection and spoke geometry
     (`zone_connection_settings`, `Zone.pad_connection`), with absent tokens
     read as KiCad's default connection and an unknown spoke.
  2. zone_replacement_changes names every changed setting with a reason, and
     nothing when the settings are the same.
  3. route_planes on sonde_u's thermal-relief GND pour (the issue's board):
     the console and JSON_SUMMARY list the changes, and every "new" value is
     the one the written zone carries. --thermal-relief drops the pad
     connection row; --skip-existing-zones keeps the pour and lists nothing.
  4. a multi-layer pour replaced on both its layers is listed once.

    python3 tests/test_1329_zone_replacement_disclosure.py
"""
import json
import os
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from run_utils import check as run_check, evidence             # noqa: E402

SONDE = os.path.join(ROOT, 'kicad_files', 'sonde_u.kicad_pcb')
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def parser():
    print('1. the parser reads the pad connection and spokes:')
    from kicad_parser import zone_connection_settings
    cases = {
        '(connect_pads\n\t\t(clearance 0.5)\n\t)': 'thermal',
        '(connect_pads yes (clearance 0.5))': 'solid',
        '(connect_pads no (clearance 0.5))': 'none',
        '(connect_pads thru_hole_only (clearance 0.5))': 'thru_hole_only',
        '': 'thermal',
    }
    for hdr, want in cases.items():
        got = zone_connection_settings('(zone (net 1) ' + hdr)[0]
        check(f'{hdr.split(chr(10))[0] or "no connect_pads"} -> {want}', got == want, got)
    got = zone_connection_settings(
        '(zone (fill yes (thermal_gap 0.508) (thermal_bridge_width 0.635))')
    check('fill spokes read', got[1:] == (0.508, 0.635), str(got))
    got = zone_connection_settings('(zone (fill yes))')
    check('absent spokes read as unknown', got[1:] == (None, None), str(got))
    evidence(SONDE, 'the sonde_u fixture')
    from kicad_parser import parse_kicad_pcb
    z = [z for z in parse_kicad_pcb(SONDE).zones if z.net_name == 'GND']
    check("sonde_u's GND pour reads thermal, 0.508 / 0.508 spokes",
          len(z) == 1 and (z[0].pad_connection, z[0].thermal_gap,
                           z[0].thermal_bridge_width) == ('thermal', 0.508, 0.508),
          str([(x.pad_connection, x.thermal_gap, x.thermal_bridge_width) for x in z]))


class _Old:
    """A parser Zone's settings, without a board."""
    def __init__(self, **kw):
        self.__dict__.update(dict(pad_connection='thermal', clearance=0.508,
                                  min_thickness=0.254, thermal_gap=0.508,
                                  thermal_bridge_width=0.508, island_removal_mode=1,
                                  priority=2, polygon=[(1, 1), (9, 1), (9, 9), (1, 9)],
                                  holes=[]))
        self.__dict__.update(kw)


def changes():
    print('2. zone_replacement_changes:')
    from plane_io import zone_replacement_changes, format_zone_replacement
    new = dict(thermal_relief=False, clearance=0.25, min_thickness=0.1, priority=2,
               polygon_points=[(0.5, 0.5), (19.5, 0.5), (19.5, 9.5), (0.5, 9.5)])
    rows = {r['setting']: r for r in zone_replacement_changes(_Old(), [new], 'why')}
    check('the changed settings, each once',
          sorted(rows) == sorted(['pad connection', 'clearance', 'minimum width',
                                  'thermal gap / spoke width', 'outline',
                                  'isolated islands']), str(sorted(rows)))
    pc = rows.get('pad connection', {})
    check('thermal relief -> solid, because --thermal-relief was not given',
          (pc.get('old'), pc.get('new')) == ('thermal relief', 'solid')
          and '--thermal-relief' in pc.get('why', ''), str(pc))
    check('every row has a reason', all(r['why'] for r in rows.values()))
    check('the priority kept is not listed', 'fill priority' not in rows)
    rows = zone_replacement_changes(_Old(), [dict(new, thermal_relief=True)], 'why')
    check('with --thermal-relief the connection is not listed',
          'pad connection' not in {r['setting'] for r in rows})
    same = _Old(pad_connection='solid', clearance=0.25, min_thickness=0.1,
                thermal_gap=0.2, thermal_bridge_width=0.2, island_removal_mode=0,
                polygon=list(new['polygon_points']))
    rows = zone_replacement_changes(same, [new], 'why')
    check('the same settings: nothing listed', rows == [], str(rows))
    line = format_zone_replacement('GND', ['B.Cu'], rows)
    check('...and the line says so', 'unchanged' in line[0], line[0])


def summary(log):
    for line in log.splitlines():
        if line.startswith('JSON_SUMMARY: '):
            return json.loads(line[len('JSON_SUMMARY: '):])
    return {}


def end_to_end(tmp):
    print('3. route_planes on sonde_u (a thermal-relief GND pour on B.Cu):')
    from kicad_parser import parse_kicad_pcb
    out = os.path.join(tmp, 'pour.kicad_pcb')
    r = run_check([sys.executable, '-X', 'utf8', 'py_router/route_planes.py', SONDE, out,
                   '--nets', 'GND', '--plane-layers', 'B.Cu'], accept=True)
    log = r.stdout + r.stderr
    evidence(out, 'the poured board')
    check('the console lists the replacement',
          "Replaced 'GND' pour on B.Cu" in log, '' if 'Replaced' in log else log[-500:])
    check('...the connection change first among them',
          'pad connection: thermal relief -> solid' in log)
    rep = summary(log).get('zone_replacements') or []
    check('JSON_SUMMARY.zone_replacements has the one pour',
          len(rep) == 1 and rep[0].get('layers') == ['B.Cu'], str(rep)[:300])
    rows = {c['setting']: c for c in (rep[0]['changes'] if rep else [])}
    for s in ('pad connection', 'clearance', 'minimum width', 'outline'):
        check(f'  ... lists {s}', s in rows)
    z = [z for z in parse_kicad_pcb(out).zones if z.net_name == 'GND']
    written = z[0] if len(z) == 1 else None
    check('one GND pour written', written is not None, str(len(z)))
    if written is not None and rows:
        check('the listed new connection is the written one',
              rows['pad connection']['new'] == 'solid'
              and written.pad_connection == 'solid', written.pad_connection)
        check('the listed new clearance is the written one',
              rows['clearance']['new'] == f"{written.clearance:g} mm",
              f"{rows['clearance']['new']} vs {written.clearance}")
        check('the listed new width is the written one',
              rows['minimum width']['new'] == f"{written.min_thickness:g} mm",
              f"{rows['minimum width']['new']} vs {written.min_thickness}")

    out = os.path.join(tmp, 'thermal.kicad_pcb')
    r = run_check([sys.executable, '-X', 'utf8', 'py_router/route_planes.py', SONDE, out,
                   '--nets', 'GND', '--plane-layers', 'B.Cu', '--thermal-relief'],
                  accept=True)
    rep = summary(r.stdout + r.stderr).get('zone_replacements') or []
    rows = {c['setting'] for c in (rep[0]['changes'] if rep else [])}
    check('--thermal-relief: still replaced, the connection not listed',
          len(rep) == 1 and 'pad connection' not in rows
          and 'thermal gap / spoke width' in rows, str(sorted(rows)))

    out = os.path.join(tmp, 'kept.kicad_pcb')
    r = run_check([sys.executable, '-X', 'utf8', 'py_router/route_planes.py', SONDE, out,
                   '--nets', 'GND', '--plane-layers', 'B.Cu', '--skip-existing-zones'],
                  accept=True)
    log = r.stdout + r.stderr
    check('--skip-existing-zones: nothing replaced, nothing listed',
          'zone_replacements' not in summary(log) and 'Replaced ' not in log)
    z = [z for z in parse_kicad_pcb(out).zones if z.net_name == 'GND']
    check("...and the designer's pour ships as it was",
          len(z) == 1 and (z[0].pad_connection, z[0].clearance, z[0].min_thickness)
          == ('thermal', 0.508, 0.254), str([(x.pad_connection, x.clearance) for x in z]))


MULTI = '''(kicad_pcb
	(version 20240108)
	(generator "pcbnew")
	(generator_version "8.0")
	(general
		(thickness 1.6)
	)
	(layers
		(0 "F.Cu" signal)
		(31 "B.Cu" signal)
		(44 "Edge.Cuts" user)
	)
	(setup
		(pad_to_mask_clearance 0)
	)
	(net 0 "")
	(net 1 "GND")
	(gr_rect
		(start 0 0)
		(end 20 10)
		(stroke
			(width 0.1)
			(type default)
		)
		(fill none)
		(layer "Edge.Cuts")
		(uuid "edge")
	)
	(zone
		(net 1)
		(net_name "GND")
		(layers "F.Cu" "B.Cu")
		(uuid "designer-pour")
		(hatch edge 0.5)
		(connect_pads
			(clearance 0.5)
		)
		(min_thickness 0.25)
		(filled_areas_thickness no)
		(fill yes
			(thermal_gap 0.5)
			(thermal_bridge_width 0.5)
		)
		(polygon
			(pts
				(xy 1 1) (xy 19 1) (xy 19 9) (xy 1 9)
			)
		)
	)
)
'''


def multi_layer(tmp):
    print('4. a multi-layer pour replaced on both layers is listed once:')
    src = os.path.join(tmp, 'multi.kicad_pcb')
    with open(src, 'w', encoding='utf-8') as f:
        f.write(MULTI)
    out = os.path.join(tmp, 'multi_out.kicad_pcb')
    r = run_check([sys.executable, '-X', 'utf8', 'py_router/route_planes.py', src, out,
                   '--nets', 'GND', 'GND', '--plane-layers', 'F.Cu', 'B.Cu'],
                  accept=True)
    rep = summary(r.stdout + r.stderr).get('zone_replacements') or []
    check('one entry, both layers',
          len(rep) == 1 and sorted(rep[0].get('layers', [])) == ['B.Cu', 'F.Cu']
          and rep[0].get('replaced_uuid') == 'designer-pour', str(rep)[:300])


def section(fn, *args):
    """Run one numbered check group; an exception fails THAT group, named,
    and the others still run."""
    try:
        fn(*args)
    except Exception as exc:                                # noqa: BLE001
        check(f'{fn.__name__} ran to the end', False, f"{type(exc).__name__}: {exc}")


def main():
    with tempfile.TemporaryDirectory() as tmp:
        section(parser)
        section(changes)
        section(end_to_end, tmp)
        section(multi_layer, tmp)
    if failures:
        print(f"\nFAILED ({len(failures)}): {failures}")
        return 1
    print('\nPASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
