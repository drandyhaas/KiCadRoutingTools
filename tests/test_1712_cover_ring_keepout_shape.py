"""#1712: a cover hole site's screw-head ring reads a part on the shapes a
circle keep-out over the site reads.

`mountable` measured the ring against ONE box per part, the bounding box of
its courtyard and its pads (`occupancy_local`), while a keep-out -- in the
seat and in the grade -- reads the courtyard and each pad copper box on their
own (#1588). For a part whose pads reach past the ends of its courtyard the
union's bounding box has empty corners, so a part clear of a keep-out of the
ring's radius on the site was still named in the ring, and the cover refused.

A synthetic board: J1 a socket whose module's hole site is at (28, 27.5),
ring 2.43 mm (a 2.7 mm hole); U1 a part with a 3 x 3 mm courtyard whose four
pads reach 0.7 mm past its left and right ends, so its union box has empty
corners. The intent carries the cover and a circle keep-out of the ring's own
radius on the site. For each case the ring (`mount_report`) and the keep-out
(check_floorplan's `keepout` rule) must name U1 alike:

  A. the site 2.26 mm off the union box's empty corner (courtyard 2.80 mm,
     nearest pad 2.88 mm): neither names U1 -- the ring did on the base code;
  B. the same with U1 turned a quarter: neither;
  C. the site 2.0 mm past a pad's end (courtyard 2.7 mm): both name U1, so the
     ring still reads pad copper beside the courtyard.

    python3 tests/test_1712_cover_ring_keepout_shape.py
"""
import contextlib
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, os.path.join(REPO, 'py_placer'))
sys.path.insert(0, TESTS)
import run_utils  # noqa: E402

from placement import floorplan as fp  # noqa: E402
from placement.heights import to_board  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
failures = []
SITE = (28.0, 27.5)
RING = 2.43
#: U1's courtyard and its pads (local), the pads reaching past the courtyard's x ends
CRT = (-1.5, -1.5, 1.5, 1.5)
PADS = [('1', -1.9, -0.5), ('2', -1.9, 0.5), ('3', 1.9, 0.5), ('4', 1.9, -0.5)]


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def board(td, name, at, rot):
    sock = ''.join(f'    (pad "{k + 1}" smd rect (at 0 {-4.5 + k}) (size 0.6 0.3) (layers "F.Cu")'
                   f' (net 0 ""))\n' for k in range(10))
    pads = ''.join(f'    (pad "{n}" smd rect (at {x} {y}{" " + str(rot) if rot else ""}) '
                   f'(size 0.6 0.4) (layers "F.Cu") (net {1 + int(n) % 2} "{("GND", "SIG")[int(n) % 2]}"))\n'
                   for n, x, y in PADS)
    text = ('(kicad_pcb (version 20240108) (generator pcbnew)\n'
            '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
            '  (net 0 "")\n  (net 1 "GND")\n  (net 2 "SIG")\n'
            '  (gr_rect (start 0 0) (end 60 60) (stroke (width 0.1) (type default))'
            ' (layer "Edge.Cuts"))\n'
            '  (footprint "t:SOCKET" (layer "F.Cu") (at 10 30)\n'
            '    (property "Reference" "J1" (at 0 0) (layer "F.SilkS") (hide yes))\n'
            '    (fp_rect (start -1 -5) (end 1 5) (stroke (width 0.05) (type default))'
            ' (layer "F.CrtYd"))\n' + sock + '  )\n'
            f'  (footprint "t:GULL" (layer "F.Cu") (at {at[0]:.4f} {at[1]:.4f}'
            f'{" " + str(rot) if rot else ""})\n'
            '    (property "Reference" "U1" (at 0 0) (layer "F.SilkS") (hide yes))\n'
            f'    (fp_rect (start {CRT[0]} {CRT[1]}) (end {CRT[2]} {CRT[3]}) (stroke (width 0.05)'
            ' (type default)) (layer "F.CrtYd"))\n' + pads + '  )\n)\n')
    path = os.path.join(td, name + '.kicad_pcb')
    with open(path, 'w') as fh:
        fh.write(text)
    return path


def intent():
    return {'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm', 'min_reader': fp.HOLES_READER,
            'covers': [{'connector': 'J1', 'side': 'F', 'axis_deg': 0, 'width_mm': 8,
                        'length_mm': 20, 'underside_mm': 5.0, 'removable': True,
                        'source': 'synthetic: a module',
                        'holes': [{'length_mm': 18, 'off_mm': -2.5, 'diameter_mm': 2.7}]}],
            'heights': [{'ref': 'U1', 'height_mm': 0.5, 'source': 'synthetic'}],
            'keepouts': [{'name': 'site', 'circle': [SITE[0], SITE[1], RING],
                          'why': "the cover's hole, at its ring's radius"}]}


def ring_names(path, doc):
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(path)
    m = fp.mount_report(fp.intent_from_dict(doc, ''), pcb, path)[0]
    site = m['holes'][0]
    check('the ring is the 2.7 mm hole\'s', abs(site['reach_mm'] - RING) < 1e-9, str(site))
    return [p['ref'] for p in site['parts']], m


def keepout_names(td, path, doc):
    ip = os.path.join(td, 'intent.json')
    with open(ip, 'w') as fh:
        json.dump(doc, fh)
    out = os.path.join(td, 'fp.json')
    if os.path.exists(out):
        os.remove(out)
    # exit 0 clean or 4 graded with an error finding; the JSON is the evidence either way
    r = subprocess.run(PY + [os.path.join(REPO, 'py_tools', 'check_floorplan.py'), path,
                             '--intent', ip, '--json', out], capture_output=True, text=True)
    check('check_floorplan graded the board', r.returncode in (0, 4)
          and 'Traceback' not in r.stdout + r.stderr, (r.stdout + r.stderr)[-600:])
    run_utils.evidence(out, 'the floorplan grade')
    return sorted({v['ref'] for v in json.load(open(out))['violations'] if v['rule'] == 'keepout'})


def case(td, label, local_site, rot):
    """U1 posed so the site lies at `local_site` in its own frame."""
    off = to_board((*local_site,) * 2, (0.0, 0.0, float(rot)))[:2]
    path = board(td, label, (SITE[0] - off[0], SITE[1] - off[1]), rot)
    ring, m = ring_names(path, intent())
    keep = keepout_names(td, path, intent())
    return ring, keep, m


def main():
    td = tempfile.mkdtemp(prefix='krt1712_')
    try:
        corner = (2.2 + 1.6, 1.5 + 1.6)     # 2.26 mm off the union box's empty corner
        for label, rot in (('A', 0), ('B', 90)):
            ring, keep, m = case(td, label, corner, rot)
            print(f'{label}. the site off the empty corner, U1 at rot {rot}')
            check(f'{label}: the keep-out of the ring\'s radius does not hold U1', keep == [],
                  str(keep))
            check(f'{label}: ...and the ring does not name it, the cover mountable',
                  ring == [] and m['mountable'] is True, json.dumps(m.get('why')))
        print('C. the site 2.0 mm past a pad\'s end')
        ring, keep, m = case(td, 'C', (2.2 + 2.0, 0.5), 0)
        check('C: the keep-out holds U1', keep == ['U1'], str(keep))
        check('C: ...and the ring names it, by its pad copper',
              ring == ['U1'] and m['mountable'] is False
              and abs(m['holes'][0]['parts'][0]['gap_mm'] - (2.0 - 1.35)) < 1e-3,
              json.dumps(m['holes'][0]))
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if failures:
        print(f'FAIL: {len(failures)}: ' + '; '.join(failures))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
