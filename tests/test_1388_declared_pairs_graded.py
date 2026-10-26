#!/usr/bin/env python3
"""#1388: declared diff pairs are measured, never graded, and check_complete
said nothing about them.

board_score only grades a pair against a `.kicad_dru` skew / diff_pair_uncoupled
rule (it will not invent a threshold), so a board whose project DECLARES a
diff-pair net class but ships no such rule had its pairs measured and nothing
more -- while the placement verifier FAILs a declared class left ungraded.
And a rule whose condition matches no net counted as "graded, 0 violations".

  1. A project netclass whose diff_pair_gap/width differs from Default's,
     with pair nets in it, is a DECLARATION: with no rule, `diff_pairs` stays
     ungraded but carries `declared`, and check_complete DISCLOSES it in its
     reason (not counted: the verdict is whatever the rest of the board earns).
  2. A class that merely repeats Default's pair settings declares nothing.
  3. A rule that binds no net is not a rule that grades: board_score lists it
     in `unbound_rules` and treats the component as having no rule; check_drc
     prints a NOT GRADING note (pair and length rules alike).
  4. A rule that binds the declared class makes the component graded, and the
     board is no longer held by the declaration.

    python3 tests/test_1388_declared_pairs_graded.py
"""
import contextlib
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT)
sys.path.insert(0, TESTS_DIR)
for _p in ('py_router', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _p))

import run_utils                                               # noqa: E402
import board_score                                             # noqa: E402
from copy_board import copy_board                              # noqa: E402

SRC = os.path.join(ROOT, 'kicad_files', 'routed_output.kicad_pcb')
PAIR = ('/fpga_adc/lvds_rx1_1_P', '/fpga_adc/lvds_rx1_1_N')
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def stage(tmp, name, gap, dru=None):
    """routed_output with a netclass 'DP' on PAIR whose diff_pair_gap is
    `gap` (Default's own when equal), and an optional .kicad_dru."""
    dst = os.path.join(tmp, name)
    with contextlib.redirect_stdout(io.StringIO()):
        copy_board(SRC, dst)
    pro = dst[:-len('.kicad_pcb')] + '.kicad_pro'
    with open(pro, encoding='utf-8') as fh:
        doc = json.load(fh)
    ns = doc.setdefault('net_settings', {})
    default = next(c for c in ns['classes'] if c['name'] == 'Default')
    cls = dict(default, name='DP', priority=0)
    cls['diff_pair_gap'] = default['diff_pair_gap'] if gap is None else gap
    ns['classes'] = [c for c in ns['classes'] if c['name'] != 'DP'] + [cls]
    ns['netclass_patterns'] = [{'netclass': 'DP', 'pattern': n} for n in PAIR]
    with open(pro, 'w', encoding='utf-8') as fh:
        json.dump(doc, fh)
    dru_path = dst[:-len('.kicad_pcb')] + '.kicad_dru'
    if dru:
        with open(dru_path, 'w', encoding='utf-8') as fh:
            fh.write(dru)
    elif os.path.isfile(dru_path):
        os.remove(dru_path)
    return dst


def dp(board):
    return board_score.score_diff_pairs(board, {})


RULE = '''(version 1)
(rule "skew"
  (condition "A.NetClass == '%s'")
  (constraint skew (max 0.15mm) (within_diff_pairs)))
'''


def main():
    tmp = tempfile.mkdtemp(prefix='t1388_')
    try:
        # 1. declared class, no rule
        b = stage(tmp, 'decl.kicad_pcb', 0.123)
        r = dp(b)
        check('declared class, no rule: ungraded but `declared`',
              r.get('ran') is False and list(r.get('declared', {})) == ['DP']
              and r['declared']['DP'] == [' / '.join(PAIR)], str(r)[:300])
        res = subprocess.run([sys.executable, '-X', 'utf8',
                              os.path.join(ROOT, 'check_complete.py'), b,
                              '--skip-slow', '--json', b + '.cc.json'],
                             cwd=ROOT, capture_output=True, text=True,
                             encoding='utf-8', errors='replace')
        run_utils.evidence(b + '.cc.json', 'check_complete json')
        with open(b + '.cc.json', encoding='utf-8') as fh:
            doc = json.load(fh)
        _disc = doc['reason'].split('DISCLOSED, not counted: ', 1)
        check('check_complete: discloses the declaration, not as a reason',
              res.returncode in (0, 4) and len(_disc) == 2
              and 'diff_pairs: the board DECLARES it (DP)' in _disc[1]
              and 'DECLARES' not in _disc[0], doc['reason'][-400:])

        # 2. a class that repeats Default's pair settings declares nothing
        b2 = stage(tmp, 'same.kicad_pcb', None)
        r = dp(b2)
        check('a class equal to Default declares nothing',
              r.get('ran') is False and 'declared' not in r, str(r)[:200])

        # 3. a rule that binds no net grades nothing
        b3 = stage(tmp, 'unbound.kicad_pcb', 0.123, RULE % 'NoSuchClass')
        r = dp(b3)
        check('an unbound rule is not a rule: ungraded, listed, still declared',
              r.get('ran') is False and r.get('unbound_rules') == ['skew']
              and 'DP' in r.get('declared', {}), str(r)[:300])
        out = subprocess.run([sys.executable, '-X', 'utf8',
                              os.path.join(ROOT, 'py_router', 'check_drc.py'),
                              b3, '--nets', PAIR[0]], cwd=ROOT,
                             capture_output=True, text=True, encoding='utf-8',
                             errors='replace')
        text = out.stdout + out.stderr
        check('check_drc says the rule grades nothing',
              "NOT GRADING: rule 'skew' binds no net" in text, text[-400:])
        lr = RULE.replace('constraint skew (max 0.15mm) (within_diff_pairs)',
                          'constraint length (max 100mm)') % 'NoSuchClass'
        b3b = stage(tmp, 'unbound_len.kicad_pcb', 0.123, lr)
        out = subprocess.run([sys.executable, '-X', 'utf8',
                              os.path.join(ROOT, 'py_router', 'check_drc.py'),
                              b3b, '--nets', PAIR[0]], cwd=ROOT,
                             capture_output=True, text=True, encoding='utf-8',
                             errors='replace')
        text = out.stdout + out.stderr
        check('...and for a length rule',
              "NOT GRADING: rule 'skew' binds no net" in text, text[-400:])

        # 4. a rule that binds the declared class grades the component
        b4 = stage(tmp, 'bound.kicad_pcb', 0.123, RULE % 'DP')
        r = dp(b4)
        check('a rule that binds the class: graded, not held by the declaration',
              r.get('ran') is True and 'declared' not in r
              and 'unbound_rules' not in r, str(r)[:300])
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
