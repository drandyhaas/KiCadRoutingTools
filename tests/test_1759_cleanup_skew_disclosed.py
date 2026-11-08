#!/usr/bin/env python3
"""#1759: a later route.py step whose cleanup moves a skew-graded pair out of
its tolerance says so.

The chain (esp_prog, as the issue's): route_diff matches the USB pair /D_P
/D_N to 0.127 mm and records it protected; one foreign GND via is dropped
inside the matcher's bump's clearance; the all-nets `route.py --nets '*'`
then runs its post-route cleanup over every net the glob selected, pair
included. The octolinear graze re-bend replaced the bump with the shortest
jog (skew 0.0006 -> 0.299 mm) and route.py said nothing.

Rows:
  1. unit: skew_moved_out_of_range reports a group moved from within to out,
     and one already out that the step pushed further; not one that stayed
     within, improved, or could not be measured.
  2. unit: skew_group_snapshot grades the matched pair on the recorded
     route_diff tolerance, within.
  3. the chain: route.py's console and JSON_SUMMARY name a pair whose skew the
     cleanup moved out of range, or the pair ships within its tolerance --
     never out of range and unnamed.

    python3 tests/test_1759_cleanup_skew_disclosed.py
"""
import json
import os
import re
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from run_utils import check as run_check, evidence, tool        # noqa: E402

FAILS = []
G = ['--clearance', '0.1778', '--track-width', '0.2', '--via-size', '0.45',
     '--via-drill', '0.3']
VIA = ('\t(via\n\t\t(at 125.65 102.15)\n\t\t(size 0.45)\n\t\t(drill 0.3)\n'
       '\t\t(layers "F.Cu" "B.Cu")\n\t\t(net "GND")\n'
       '\t\t(uuid "00000000-0000-4000-8000-000000001234")\n\t)\n')


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def py(*argv):
    return run_check([sys.executable, '-B', '-X', 'utf8'] + [str(a) for a in argv],
                     accept=True)


def unit_rows():
    from check_drc import skew_moved_out_of_range as moved

    def row(skew, excess):
        return {'nets': ['/P', '/N'], 'rule': 'r', 'skew_mm': skew,
                'min_mm': None, 'max_mm': 0.1, 'excess_mm': excess}
    k = ('r', (1, 2))
    got = moved({k: row(0.01, 0.0)}, {k: row(0.3, 0.2)})
    check("within -> out is reported", len(got) == 1 and got[0]['was_within']
          and got[0]['skew_after_mm'] == 0.3, str(got))
    got = moved({k: row(0.2, 0.1)}, {k: row(0.3, 0.2)})
    check("out -> further out is reported, not as was_within",
          len(got) == 1 and not got[0]['was_within'], str(got))
    check("within -> within is not", moved({k: row(0.01, 0.0)}, {k: row(0.09, 0.0)}) == [])
    check("out -> less out is not", moved({k: row(0.3, 0.2)}, {k: row(0.2, 0.1)}) == [])
    check("an unmeasured side is not compared",
          moved({k: None}, {k: row(0.3, 0.2)}) == []
          and moved({k: row(0.01, 0.0)}, {k: None}) == [])


def main():
    print("1. skew_moved_out_of_range")
    unit_rows()
    tmp = tempfile.mkdtemp(prefix='t1759_')
    try:
        a, b, p, c = (os.path.join(tmp, f'{t}.kicad_pcb') for t in 'abpc')
        py(tool('copy_board.py'), os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb'), a)
        r = py(tool('route_diff.py'), evidence(a, 'esp_prog copy'), b,
               '--nets', '/D_P', '/D_N', *G, '--diff-pair-gap', '0.25', '--no-gnd-vias',
               '--diff-pair-intra-match', '--length-match-tolerance', '0.127')
        if 'bumps added' not in r.stdout:
            print("  [BROKEN TEST] route_diff added no bump: the pair this test "
                  "needs matched is not")
            return 1
        py(tool('copy_board.py'), evidence(b, 'matched board'), p)
        with open(p, encoding='utf-8') as fh:
            t = fh.read()
        i = t.rstrip().rfind(')')
        with open(p, 'w', encoding='utf-8') as fh:
            fh.write(t[:i] + VIA + t[i:])

        print("2. skew_group_snapshot on the matched board")
        from kicad_parser import parse_kicad_pcb
        from check_drc import skew_group_snapshot
        snap = skew_group_snapshot(parse_kicad_pcb(p), p)
        rows = [v for v in snap.values() if v and set(v['nets']) == {'/D_P', '/D_N'}]
        check("the pair is graded on the recorded tolerance, within",
              len(rows) == 1 and rows[0]['max_mm'] == 0.127
              and rows[0]['excess_mm'] == 0.0, str(rows))

        print("3. the all-nets route.py step")
        r = py(tool('route.py'), p, c, '--nets', '*', *G)
        log = r.stdout + r.stderr
        m = next((l for l in log.splitlines() if l.startswith('JSON_SUMMARY:')), None)
        summ = json.loads(m.split(':', 1)[1]) if m else {}
        named = [x for x in summ.get('skew_moved_by_cleanup') or []
                 if set(x['nets']) == {'/D_P', '/D_N'}]
        import subprocess
        r = subprocess.run([sys.executable, '-B', '-X', 'utf8', tool('check_drc.py'),
                            evidence(c, 'routed board')], capture_output=True,
                           text=True, encoding='utf-8', errors='replace', cwd=ROOT)
        if 'Traceback' in r.stdout + r.stderr or 'Diff-pair rules' not in r.stdout:
            print("  [BROKEN TEST] check_drc did not grade the pair rules: "
                  + (r.stdout + r.stderr)[-800:])
            return 1
        out_of_range = bool(re.search(r'/D_[PN]: [0-9.]+mm (shorter|longer) than',
                                      r.stdout))
        print(f"  shipped out of range: {out_of_range}; named: {bool(named)}")
        check("an out-of-range pair is named in JSON_SUMMARY",
              named or not out_of_range,
              f"out of range {out_of_range}, named {bool(named)}")
        check("...and on the console",
              not named or 'the post-route cleanup moved /D_P / /D_N out of its '
              'skew rule' in log)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(FAILS) if FAILS else 'ALL OK')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
