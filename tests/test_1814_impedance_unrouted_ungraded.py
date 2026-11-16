#!/usr/bin/env python3
"""#1814: board_score on an UNROUTED board with --impedance-nets has a number.

check_impedance analyses only a net with a track segment, so on a board whose
named nets have no copper yet it returns no row. score_impedance used to read
that as "asked for and could not run": `blocking` null, the component in
`unknown`, and converge answered NO-SCORE for every placement row of a board
that declares impedance nets. "Not routed yet" is what `unrouted` and `broken`
already count, so the component is now UNGRADED (as `length` is, #1246) and
`blocking` is the sum of the rest. A pattern that names no net of the board is
still a finding about the spec and stays unknown.
"""
import json
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
from run_utils import check, evidence  # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'lvds_converter_dualclk.kicad_pcb')
SCORE = os.path.join(ROOT, 'py_tools', 'board_score.py')


def _score(td, label, extra):
    js = os.path.join(td, f'{label}.json')
    # board_score exits 4 whenever blocking is non-zero or unknown, so the exit
    # code tells these cases apart from nothing; the JSON fields do.
    r = check([sys.executable, '-X', 'utf8', SCORE, BOARD, '--json', js,
               '--quiet', *extra], code=4)
    with open(evidence(js, f'{label} score'), encoding='utf-8') as f:
        return json.load(f), r.stdout


def main():
    evidence(BOARD, 'fixture board')
    with tempfile.TemporaryDirectory() as td:
        base, _ = _score(td, 'nospec', [])
        n_unrouted = base['blocking']
        assert isinstance(n_unrouted, int) and n_unrouted > 0, base['blocking']

        spec, out = _score(td, 'spec', ['--impedance-nets', '/CLK+', '/CLK-'])
        comp = spec['components']['impedance']
        assert spec['blocking'] == n_unrouted, (
            f"blocking {spec['blocking']}, wanted {n_unrouted} (the same board "
            f"without the flag): {comp}")
        assert 'impedance' in (spec.get('ungraded') or []), spec.get('ungraded')
        assert 'impedance' not in (spec.get('unknown') or []), spec.get('unknown')
        assert comp.get('ran') is False, comp
        assert '2 net(s)' in comp['reason'] and 'no track copper' in comp['reason'], comp
        print(f"  PASS: unrouted named nets -> ungraded, blocking {spec['blocking']}")

        nonet, out = _score(td, 'nonet', ['--impedance-nets', 'NO_SUCH_NET'])
        comp = nonet['components']['impedance']
        assert nonet['blocking'] is None, nonet['blocking']
        assert 'impedance' in (nonet.get('unknown') or []), nonet.get('unknown')
        assert 'NO_SUCH_NET' in comp['reason'], comp
        print("  PASS: a pattern naming no net stays unknown, blocking null")


if __name__ == '__main__':
    main()
