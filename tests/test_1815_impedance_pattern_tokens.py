#!/usr/bin/env python3
"""#1815: every --impedance-nets pattern is accounted on its own.

  1. "A, B" (comma then space, one argument) grades A and B. The comma token
     was not stripped, so the second pattern was " B", matched nothing, and the
     board read one blocker better than it is.
  2. A pattern naming no net of the board makes impedance UNKNOWN and is named
     in the reason, on the console and in `patterns_matching_no_net`, also when
     another pattern graded nets. It used to vanish beside a pattern that matched.
  3. "A B" in one argument (the only spaced form grade.py --spec can carry) is
     one pattern holding a space; the reason names it and the two accepted
     spellings rather than "no routed nets matched".
Controls: separate arguments and "A,B" grade both nets, as they did.
"""
import json
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
from run_utils import check, evidence  # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'cap_chain.kicad_pcb')
SCORE = os.path.join(ROOT, 'py_tools', 'board_score.py')
ROUTE = os.path.join(ROOT, 'py_router', 'route.py')


def _score(board, td, label, imp_args):
    js = os.path.join(td, f'{label}.json')
    # exit 4 in every case here (blocking non-zero or unknown): assert JSON
    r = check([sys.executable, '-X', 'utf8', SCORE, board, '--json', js,
               '--quiet', '--impedance-nets', *imp_args], code=4)
    with open(evidence(js, f'{label} score'), encoding='utf-8') as f:
        s = json.load(f)
    return s, s['components']['impedance'], r.stdout


def main():
    evidence(BOARD, 'fixture board')
    with tempfile.TemporaryDirectory() as td:
        routed = os.path.join(td, 'r.kicad_pcb')
        check([sys.executable, '-X', 'utf8', ROUTE, BOARD, routed,
               '--nets', 'DPA_P', 'DPB_P', 'DPA_N', 'DPB_N'], accept=True)
        evidence(routed, 'routed board')

        ref = None
        for label, args in (('separate', ['DPA_P', 'DPB_P']),
                            ('comma', ['DPA_P,DPB_P']),
                            ('comma_space', ['DPA_P, DPB_P'])):
            s, c, _ = _score(routed, td, label, args)
            assert c.get('nets_analyzed') == 2, (label, c)
            assert 'impedance' not in (s.get('unknown') or []), (label, s.get('unknown'))
            assert not c.get('patterns_matching_no_net'), (label, c)
            ref = s['blocking'] if ref is None else ref
            assert s['blocking'] == ref, (label, s['blocking'], ref)
        print(f"  PASS: separate, 'A,B' and 'A, B' each grade 2 nets, blocking {ref}")

        s, c, out = _score(routed, td, 'nonet', ['DPA_P', 'NO_SUCH_NET'])
        assert s['blocking'] is None, s['blocking']
        assert 'impedance' in (s.get('unknown') or []), s.get('unknown')
        assert c.get('patterns_matching_no_net') == ['NO_SUCH_NET'], c
        assert 'NO_SUCH_NET' in c['reason'], c
        assert any(ln.startswith('IMPEDANCE PATTERNS') and 'NO_SUCH_NET' in ln
                   for ln in out.splitlines()), out[-1500:]
        print("  PASS: a pattern naming no net is named and makes impedance unknown")

        s, c, out = _score(routed, td, 'spaced', ['DPA_P DPB_P'])
        assert s['blocking'] is None, s['blocking']
        assert c.get('patterns_matching_no_net') == ['DPA_P DPB_P'], c
        assert "'DPA_P DPB_P'" in c['reason'] and 'commas' in c['reason'], c
        print("  PASS: 'A B' in one argument is named as one spaced pattern")


if __name__ == '__main__':
    main()
