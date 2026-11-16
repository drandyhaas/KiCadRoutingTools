#!/usr/bin/env python3
"""The KiCad-oracle recheck keeps its copper out of keep-outs (#1778).

The oracle lays copper outside the router's stamped maps -- the #649b
stitching via, the #648 sliver weld, the rescue escalation and the exact-fill
strap -- and none of it asked whether a keep-out rule area forbade it, so
route.py shipped a GND via inside a `(vias not_allowed)` area (KiCad:
items_not_allowed). Every emission site now asks `check_drc.
copper_keepout_gate`; the #649b position search skips a refused spot and
tries the next, and a link with no legal copper is left honestly open.

  1. the gate takes the run's net-filtered keep-outs by argument (the oracle
     re-parses the board, so nothing is attached to its parse), and the
     #649b site search passes over a refused spot to the next ring;
  2. end to end on the issue's board (needs kicad-cli, else SKIP): the
     stitching via is REFUSED, the shipped board has no copper in the area
     (check_drc's census and kicad-cli's items_not_allowed), and U1 is left
     unconnected rather than joined illegally;
  3. controls: the same board with vias ALLOWED in the area joins U1
     (connected, 0 items_not_allowed), and the board without the S1 pad,
     which never reached the #649b branch, still ends as an honest open.

    python3 tests/test_1778_oracle_stitch_keepout.py
"""
import contextlib
import io
import os
import re
import shutil
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from kicad_parser import parse_kicad_pcb, Segment, Via          # noqa: E402
from check_drc import copper_keepout_gate, rule_area_audit       # noqa: E402
import net_keepouts                                              # noqa: E402
import run_utils                                                 # noqa: E402

FIX = os.path.join(TESTS_DIR, 'fixtures', '1778')
failures = []


def ok(label, cond, detail=''):
    print(f"  [{'ok' if cond else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not cond:
        failures.append(label)


def py(*argv):
    r = subprocess.run([sys.executable, '-B', '-X', 'utf8', *map(str, argv)], cwd=ROOT,
                       capture_output=True, text=True, encoding='utf-8', errors='replace')
    out = r.stdout + r.stderr
    if 'Traceback' in out:
        raise AssertionError(f'BROKEN TEST: {argv[0]} crashed:\n{out[-1500:]}')
    return r.returncode, out


def parse(path):
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        return parse_kicad_pcb(path)


def gate_argument():
    print('1. copper_keepout_gate(net_keepouts=...)')
    pcb = parse(os.path.join(FIX, 'via-ko-nos1.kicad_pcb'))
    gnd = next(i for i, n in pcb.nets.items() if n.name == 'GND')
    ko = {'name': 'NK', 'nets': frozenset({'GND'}), 'layers': ['F.Cu', 'B.Cu'],
          'polygon': [(30, 5), (34, 5), (34, 9), (30, 9)], 'holes': [],
          'tracks_allowed': False, 'vias_allowed': False}
    via = Via(x=32, y=7, size=0.6, drill=0.3, layers=['F.Cu', 'B.Cu'], net_id=gnd)
    far = Via(x=20, y=20, size=0.6, drill=0.3, layers=['F.Cu', 'B.Cu'], net_id=gnd)
    g = copper_keepout_gate(pcb, net_keepouts=[ko])
    ok('given list, nothing attached -> a gate', g is not None)
    ok('bound net via inside refused', bool(g and g(vias=[via])))
    ok('far via passes', g is not None and g(vias=[far]) == '')
    net_keepouts.attach(pcb, [ko])
    g2 = copper_keepout_gate(pcb)
    ok('None falls back to the attached list', bool(g2 and g2(vias=[via])))
    seg = Segment(start_x=31, start_y=7, end_x=33, end_y=7, width=0.2,
                  layer='F.Cu', net_id=gnd)
    ok('track of the bound net refused', bool(g2 and g2(segments=[seg])))


def site_search():
    print('1b. stitch_via_site: a refused spot hands the search to the next')
    from kicad_oracle import stitch_via_site
    everywhere = lambda x, y: True                               # noqa: E731
    ok('free board: the anchor itself', stitch_via_site(10, 10, None, 0, everywhere,
                                                        everywhere) == (10, 10))
    # a keep-out covering everything within 0.7 mm of the anchor: the 0.0,
    # 0.3 and 0.6 rings are refused and the 1.0 ring answers
    out_of_area = lambda x, y: (x - 10) ** 2 + (y - 10) ** 2 > 0.7 ** 2  # noqa: E731
    got = stitch_via_site(10, 10, None, 0, everywhere, out_of_area)
    ok('refused rings skipped, next ring taken',
       got is not None and abs(((got[0] - 10) ** 2 + (got[1] - 10) ** 2) ** 0.5 - 1.0) < 1e-9,
       repr(got))
    ok('nothing admissible -> None',
       stitch_via_site(10, 10, None, 0, everywhere, lambda x, y: False) is None)
    ok('the fills still decide',
       stitch_via_site(10, 10, None, 0, lambda x, y: x > 11.2, everywhere)[0] > 11.2)


def stage(tmp, name, src, edit=None):
    dst = os.path.join(tmp, name + '.kicad_pcb')
    rc, o = py('py_router/copy_board.py', src, dst)
    run_utils.evidence(dst, f'staged {name}')
    if edit:
        t = open(dst, encoding='utf-8', newline='').read()
        t2 = edit(t)
        assert t2 != t, f'BROKEN TEST: the {name} edit changed nothing'
        open(dst, 'w', encoding='utf-8', newline='').write(t2)
    return dst


def route_and_grade(tmp, name, board, cli):
    out = os.path.join(tmp, name + '.routed.kicad_pcb')
    rc, log = py('py_router/route.py', board, out, '--nets', '*', '--layers', 'F.Cu', 'B.Cu',
                 '--escalation', 'off', '--no-fix-drc-settings')
    ok(f'{name}: route.py exit 0', rc == 0, f'exit {rc}')
    run_utils.evidence(out, f'routed {name}')
    pcb = parse(out)
    census = rule_area_audit(pcb, pcb.segments, pcb.vias)
    rpt = os.path.join(tmp, name + '.drc.rpt')
    subprocess.run([cli, 'pcb', 'drc', '--refill-zones', '--format', 'report', '-o', rpt, out],
                   capture_output=True)
    txt = open(run_utils.evidence(rpt, f'{name} kicad-cli DRC report'),
               encoding='utf-8', errors='replace').read()
    m = re.search(r'Found (\d+) unconnected', txt)
    unconnected = int(m.group(1)) if m else None
    return log, census, txt.count('[items_not_allowed]'), unconnected


def end_to_end(cli):
    tmp = tempfile.mkdtemp(prefix='t1778_')
    try:
        src = os.path.join(FIX, 'via-ko.kicad_pcb')
        cases = {
            'via-ko': stage(tmp, 'via-ko', src),
            'via-ko-v': stage(tmp, 'via-ko-v', src,
                              lambda t: t.replace('(vias not_allowed)', '(vias allowed)')),
            'via-ko-nos1': stage(tmp, 'via-ko-nos1', os.path.join(FIX, 'via-ko-nos1.kicad_pcb')),
        }
        res = {n: route_and_grade(tmp, n, b, cli) for n, b in cases.items()}

        print('2. the issue board: the stitching via is refused')
        log, census, ina, unc = res['via-ko']
        ok('the #649b branch was reached and refused the via',
           'stitching via REFUSED' in log and '#1778' in log)
        ok('no #649b via placed', 'stitching via placed' not in log)
        ok('check_drc census: no copper in the area', census == [], repr(census))
        ok('kicad-cli: 0 items_not_allowed', ina == 0, f'{ina}')
        ok('kicad-cli: U1 reported unconnected (honest open)', unc == 1, f'{unc}')

        print('3. controls')
        log, census, ina, unc = res['via-ko-v']
        ok('vias allowed: 0 items_not_allowed', ina == 0, f'{ina}')
        ok('vias allowed: U1 joined (0 unconnected)', unc == 0, f'{unc}')
        ok('vias allowed: nothing refused', '#1778' not in log)
        log, census, ina, unc = res['via-ko-nos1']
        ok('no S1: 0 items_not_allowed', ina == 0, f'{ina}')
        ok('no S1: honest open', unc == 1, f'{unc}')
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def main():
    gate_argument()
    site_search()
    from kicad_oracle import find_kicad_cli
    cli = find_kicad_cli(warn=False)
    if not cli:
        if failures:
            print(f'FAILED: {len(failures)}: {failures}')
            return 1
        print('SKIP: kicad-cli not found -- the oracle recheck (sections 2-3) needs KiCad')
        return 77
    end_to_end(cli)
    if failures:
        print(f'FAILED: {len(failures)}: {failures}')
        return 1
    print('PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
