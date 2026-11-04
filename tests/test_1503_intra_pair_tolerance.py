"""#1503: the P/N skew a diff pair is matched to, and graded at.

* route_diff matches a pair a .kicad_dru P/N skew rule (`within_diff_pairs`)
  binds to that rule's max, with or without --diff-pair-intra-match; every
  other matched pair to the default (0.1 mm). An explicit
  --length-match-tolerance overrides both.
* It records each matched pair and its tolerance on the board
  (`intra_pair_tolerance`), and the graders (check_drc, board_score) hold a
  pair to its recorded tolerance where no .kicad_dru skew rule binds it --
  a rule that does bind it wins, as KiCad's last match does.

On the in-repo qfn_csi_underpad_diff board (CSI_D0 routes coupled).

    python3 tests/test_1503_intra_pair_tolerance.py
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)
import run_utils  # noqa: E402

RUN_ALL_TIMEOUT = 600
PY = [sys.executable, '-X', 'utf8']
BOARD = os.path.join('kicad_files', 'qfn_csi_underpad_diff.kicad_pcb')
RULE = ('(version 1)\n(rule "csi pair skew"\n  (condition "A.inDiffPair(\'*CSI*\')")\n'
        '  (constraint skew (max {mm}mm) (within_diff_pairs)))\n')
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def run(argv):
    return subprocess.run(PY + argv, capture_output=True, text=True, cwd=REPO)


def route(td, name, dru_mm=None, flags=()):
    d = os.path.join(td, name)
    os.makedirs(d)
    src = os.path.join(d, 'in.kicad_pcb')
    run([os.path.join('py_router', 'copy_board.py'), BOARD, src])
    run_utils.evidence(src)
    if dru_mm is not None:
        with open(os.path.join(d, 'in.kicad_dru'), 'w') as fh:
            fh.write(RULE.format(mm=dru_mm))
    out = os.path.join(d, 'out.kicad_pcb')
    r = run([os.path.join('py_router', 'route_diff.py'), src, out, '--nets', 'CSI_*', *flags])
    run_utils.evidence(out)
    return r, out


def record(out):
    with open(os.path.splitext(out)[0] + '.kicad_pro') as fh:
        return (json.load(fh).get('kicad_routing_tools') or {}).get('intra_pair_tolerance') or {}


def set_record(out, tol):
    pro = os.path.splitext(out)[0] + '.kicad_pro'
    with open(pro) as fh:
        d = json.load(fh)
    for v in d['kicad_routing_tools']['intra_pair_tolerance'].values():
        v['tolerance_mm'] = tol
    d['kicad_routing_tools']['records_rev'] = d['kicad_routing_tools'].get('records_rev', 0) + 100
    with open(pro, 'w') as fh:
        json.dump(d, fh)


def diff_pairs(out):
    r = run([os.path.join('py_tools', 'board_score.py'), out])
    line = next((x for x in r.stdout.splitlines() if x.startswith('SCORE_JSON=')), None)
    if line is None:
        return None, r.stdout[-400:]
    return json.loads(line[len('SCORE_JSON='):])['components']['diff_pairs'], ''


def main():
    td = tempfile.mkdtemp(prefix='krt1503_')
    try:
        print('1. the tolerance each pair is matched to, and recorded')
        r, a = route(td, 'flag', flags=('--diff-pair-intra-match',))
        rec = record(a).get('CSI_D0_P') or {}
        check('--diff-pair-intra-match, no rule: 0.1 mm, source default',
              (rec.get('tolerance_mm'), rec.get('source')) == (0.1, 'default'), json.dumps(rec))
        r, b = route(td, 'rule', dru_mm=0.127)
        rec = record(b).get('CSI_D0_P') or {}
        check('a .kicad_dru P/N rule, no flag: matching on, at the rule\'s 0.127 mm',
              'Intra-pair matching on for' in r.stdout
              and (rec.get('tolerance_mm'), rec.get('source')) == (0.127, 'dru'),
              json.dumps(rec) + r.stdout[-300:])
        r, c = route(td, 'cli', dru_mm=0.127, flags=('--length-match-tolerance', '0.3'))
        rec = record(c).get('CSI_D0_P') or {}
        check('--length-match-tolerance 0.3 overrides the rule',
              (rec.get('tolerance_mm'), rec.get('source')) == (0.3, 'cli'), json.dumps(rec))
        r, n = route(td, 'none')
        check('no flag, no rule: nothing matched, nothing recorded', record(n) == {},
              json.dumps(record(n)))

        print('2. the graders hold a pair to its recorded tolerance')
        dp, why = diff_pairs(a)
        check('no .kicad_dru: diff pairs graded on the record, clean at 0.1 mm',
              dp is not None and dp.get('ran') and dp.get('count') == 0, json.dumps(dp) + why)
        set_record(a, 0.01)
        dp, why = diff_pairs(a)
        check('the record tightened to 0.01 mm: one skew violation',
              dp is not None and dp.get('count') == 1, json.dumps(dp) + why)
        set_record(b, 0.01)
        dp, why = diff_pairs(b)
        check('a .kicad_dru rule over the pair wins over its record (0.127 mm): clean',
              dp is not None and dp.get('ran') and dp.get('count') == 0, json.dumps(dp) + why)
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if failures:
        print(f'FAIL: {len(failures)}: ' + '; '.join(failures))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
