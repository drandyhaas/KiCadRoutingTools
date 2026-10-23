#!/usr/bin/env python3
"""#1299: a run that pours a net on EVERY layer of its multi-layer pour
replaces that pour, and two overlapping same-net pours are one reference.

route_planes decided "replace the same-net pour?" one layer at a time
(`plane_io.check_existing_zones`), and a pour spanning other layers was always
kept -- even when the same call poured the net on all of them. The writer's
"remove a pour only when every layer it covers is replaced" path could never
fire, so on CM5_MINIMA_3 `--nets GND x5 --plane-layers F In1 In2 In4 B` kept
the designer's 5-layer GND pour and stacked a new GND zone on each layer. The
check is now given every layer the call pours the net on.

check_impedance then keyed reference copper by (zone, fill component), so a
trace passing from one GND pour's fill to the other's counted as a reference
SPLIT: 28 nets / 114 crossings (64 split) on that board, 16 / 53 / 0 split once
the stale pour was deleted. Overlapping same-net fills are now one key.

Checks:
  1. check_existing_zones: a 2-layer pour is replaced by a call that pours
     both layers, kept (and out-ranked) by one that pours one.
  2. the writer's span reader agrees with the parser on KiCad 5 / 6
     spellings: a bare `(layer B.Cu)`, `(layers F&B.Cu)`, `(layers *.Cu)`.
  3. route_planes end to end on a 2-layer board with ONE (layers F.Cu B.Cu)
     GND pour: both layers -> exactly two single-layer GND pours, no WARNING;
     one layer -> the pour is kept and the WARNING names the other layer.
  4. check_impedance: a trace over two overlapping GND pours grades no
     crossing (it graded a split); a gap between two GND pours, and a seam
     between GND and VCC pours, still grade a crossing.

    python3 tests/test_1299_multilayer_zone_replace.py
"""
import os
import re
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))
sys.path.insert(0, TESTS_DIR)

from run_utils import check as run_check, evidence             # noqa: E402

failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


# A 20 x 10 mm two-layer board; the pours and tracks are appended per case.
HEAD = '''(kicad_pcb
	(version 20240108)
	(generator "pcbnew")
	(generator_version "8.0")
	(general
		(thickness 1.6)
	)
	(paper "A4")
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
	(net 2 "SIG")
	(net 3 "VCC")
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
'''

SIG_TRACK = '''	(segment
		(start 2 5)
		(end 18 5)
		(width 0.2)
		(layer "F.Cu")
		(net 2)
		(uuid "sig")
	)
'''


def pour(uuid, layers, x1, x2, net=1, name='GND'):
    return f'''	(zone
		(net {net})
		(net_name "{name}")
		{layers}
		(uuid "{uuid}")
		(hatch edge 0.5)
		(connect_pads
			(clearance 0.2)
		)
		(min_thickness 0.2)
		(filled_areas_thickness no)
		(fill yes
			(thermal_gap 0.5)
			(thermal_bridge_width 0.5)
		)
		(polygon
			(pts
				(xy {x1} 1) (xy {x2} 1) (xy {x2} 9) (xy {x1} 9)
			)
		)
	)
'''


def write(path, *parts):
    with open(path, 'w', encoding='utf-8') as f:
        f.write(HEAD + ''.join(parts) + ')\n')
    return path


def pours_in(path):
    """[(net name, sorted copper layers)] of every pour block in the file."""
    with open(path, encoding='utf-8') as f:
        text = f.read()
    out = []
    for m in re.finditer(r'\(zone\b', text):
        block = text[m.start():text.find('(polygon', m.start())]
        name = re.search(r'\(net_name\s+"([^"]*)"\)', block)
        lay = re.search(r'\(layers?\s+([^()]*)\)', block)
        out.append((name.group(1) if name else '',
                    sorted(t.strip('"') for t in lay.group(1).split())))
    return out


def decision():
    print('1. check_existing_zones decides with the whole call:')
    from plane_io import ZoneInfo, check_existing_zones
    multi = ZoneInfo(net_id=1, net_name='GND', layer='B.Cu', uuid='m',
                     span=('F.Cu', 'B.Cu'))
    _c, _k, replace, coexist = check_existing_zones(
        [multi], 'B.Cu', 'GND', 1, call_layers=['F.Cu', 'B.Cu'])
    check('a call pouring F.Cu and B.Cu replaces the (F.Cu, B.Cu) pour',
          replace is multi and coexist == [], f"replace={replace} coexist={coexist}")
    _c, _k, replace, coexist = check_existing_zones(
        [multi], 'B.Cu', 'GND', 1, call_layers=['B.Cu'])
    check('a call pouring only B.Cu keeps it and out-ranks it',
          replace is None and coexist == [multi], f"replace={replace}")
    _c, _k, replace, _co = check_existing_zones([multi], 'B.Cu', 'GND', 1)
    check('no call_layers: the target layer alone, as before', replace is None)
    three = ZoneInfo(net_id=1, net_name='GND', layer='B.Cu', uuid='t',
                     span=('F.Cu', 'In1.Cu', 'B.Cu'))
    _c, _k, replace, _co = check_existing_zones(
        [three], 'B.Cu', 'GND', 1, call_layers=['F.Cu', 'B.Cu'])
    check('a pour over a layer the call does not pour is kept', replace is None)


def writer_spans():
    print('2. the writer reads a pour\'s span as the parser does:')
    from plane_io import filter_zones_from_content
    k5 = HEAD + ('\t(zone (net 1) (net_name "GND") (layer B.Cu) (tstamp k5)\n'
                 '\t\t(polygon (pts (xy 1 1) (xy 19 1) (xy 19 9)))\n\t)\n)\n')
    out = filter_zones_from_content(k5, [(1, 'B.Cu')])
    check('KiCad 5 bare (layer B.Cu) is removable', 'tstamp k5' not in out)
    k6 = HEAD + ('\t(zone (net 1) (net_name "GND") (layers F&B.Cu) (tstamp k6)\n'
                 '\t\t(polygon (pts (xy 1 1) (xy 19 1) (xy 19 9)))\n\t)\n)\n')
    out = filter_zones_from_content(k6, [(1, 'B.Cu')])
    check('(layers F&B.Cu) is kept when only B.Cu is replaced', 'tstamp k6' in out)
    out = filter_zones_from_content(k6, [(1, 'B.Cu'), (1, 'F.Cu')])
    check('(layers F&B.Cu) goes when F.Cu and B.Cu are replaced',
          'tstamp k6' not in out)
    star = k6.replace('F&B.Cu', '*.Cu').replace('tstamp k6', 'tstamp st')
    out = filter_zones_from_content(star, [(1, 'B.Cu'), (1, 'F.Cu')])
    check('(layers *.Cu) goes when every board copper layer is replaced',
          'tstamp st' not in out)
    out = filter_zones_from_content(star, [(1, 'B.Cu')])
    check('(layers *.Cu) is kept when one is not', 'tstamp st' in out)


def end_to_end(tmp):
    print('3. route_planes on ONE (layers F.Cu B.Cu) GND pour:')
    src = write(os.path.join(tmp, 'multi.kicad_pcb'),
                pour('designer', '(layers "F.Cu" "B.Cu")', 1, 19))
    evidence(src, 'the synthetic board')
    out = os.path.join(tmp, 'both.kicad_pcb')
    r = run_check([sys.executable, '-X', 'utf8', 'py_router/route_planes.py', src, out,
                   '--nets', 'GND', 'GND', '--plane-layers', 'F.Cu', 'B.Cu'],
                  accept=True)
    log = r.stdout + r.stderr
    evidence(out, 'the both-layers output')
    got = sorted(pours_in(out))
    check('both layers -> two single-layer GND pours, the 2-layer one gone',
          got == [('GND', ['B.Cu']), ('GND', ['F.Cu'])], str(got))
    check('...and no "cannot be replaced" warning',
          'cannot be replaced' not in log,
          '' if 'cannot be replaced' not in log else log[-400:])
    check('...and it says the pour goes as a whole', 'as a whole' in log)
    out = os.path.join(tmp, 'one.kicad_pcb')
    r = run_check([sys.executable, '-X', 'utf8', 'py_router/route_planes.py', src, out,
                   '--nets', 'GND', '--plane-layers', 'B.Cu'], accept=True)
    log = r.stdout + r.stderr
    evidence(out, 'the one-layer output')
    got = sorted(pours_in(out))
    check('one layer -> the 2-layer pour is kept beside the new B.Cu pour',
          got == [('GND', ['B.Cu']), ('GND', ['B.Cu', 'F.Cu'])], str(got))
    named = "pour 'GND' on F.Cu in the same run" in log
    check('...and the WARNING names the layer the run does not pour',
          named, '' if named else log[-600:])


def impedance(tmp):
    print('4. check_impedance: overlapping same-net fills are one reference:')
    from kicad_parser import parse_kicad_pcb
    from check_impedance import analyze_impedance

    def crossings(name, *pours):
        path = write(os.path.join(tmp, name), SIG_TRACK, *pours)
        report = analyze_impedance(parse_kicad_pcb(path), net_patterns=['SIG'])
        check(f'{name}: the SIG trace was analysed',
              [n.net_name for n in report.nets] == ['SIG'],
              str([n.net_name for n in report.nets]))
        return [(c.kind, round(c.x, 2)) for n in report.nets for c in n.crossings]

    one = crossings('one.kicad_pcb', pour('b', '(layer "B.Cu")', 1, 19))
    check('control: one GND pour under the trace, no crossing', one == [], str(one))
    two = crossings('two.kicad_pcb', pour('a', '(layer "B.Cu")', 1, 10),
                    pour('b', '(layer "B.Cu")', 1, 19))
    check('two overlapping GND pours: no crossing (was a split at x=10)',
          two == [], str(two))
    gap = crossings('gap.kicad_pcb', pour('a', '(layer "B.Cu")', 1, 9),
                    pour('b', '(layer "B.Cu")', 11, 19))
    check('two GND pours 2 mm apart: still a crossing', bool(gap), str(gap))
    seam = crossings('seam.kicad_pcb', pour('a', '(layer "B.Cu")', 1, 10, 3, 'VCC'),
                     pour('b', '(layer "B.Cu")', 10, 19))
    check('a VCC pour beside a GND pour: still a crossing', bool(seam), str(seam))


def section(fn, *args):
    """Run one numbered check group; an exception fails THAT group, named,
    and the others still run."""
    try:
        fn(*args)
    except Exception as exc:                                # noqa: BLE001
        check(f'{fn.__name__} ran to the end', False, f"{type(exc).__name__}: {exc}")


def main():
    with tempfile.TemporaryDirectory() as tmp:
        section(decision)
        section(writer_spans)
        section(end_to_end, tmp)
        section(impedance, tmp)
    if failures:
        print(f"\nFAILED ({len(failures)}): {failures}")
        return 1
    print('\nPASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
