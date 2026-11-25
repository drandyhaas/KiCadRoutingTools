"""A clearance rule written `A.NetClass == 'HV' && A.Net != B.Net` is honoured.

KiCad tests clearance only between items on different nets, so `A.Net !=
B.Net` is true for every pair a clearance rule is asked about, and KiCad's own
custom-rule examples carry it. The scoped channel (#1491) admitted only
comparisons of one item's property against a literal, so this two-item term
pushed the whole rule out: the router routed the class at its net-class value
and check_drc printed NOT GRADED, while KiCad enforced the rule. Measured on a
4-layer board with a 0.6 mm high-voltage rule: check_drc found 37 violations
with the term and 518 without it.

Invariants:
1. With the term, in either order and either operand order, the rule is
   taken by the scoped channel and resolves to the same pair values as the
   rule without it.
2. A term under `||` or `!` is not dropped (it changes what the rule binds),
   and a rule whose only condition is the term stays out of the channel.
3. check_drc flags two HV tracks closer than the rule with the term, and does
   not print NOT GRADED.

    python3 tests/test_dru_distinct_nets_clearance.py
"""
import os
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)
sys.path.insert(0, os.path.join(TESTS, 'oracle'))

from constraint_agreement import write_board, pcb_data_for  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


HV = [{'name': 'HV', 'clearance': 0.2, 'priority': 0}]
PATTERNS = [('A', 'HV')]           # net 1 'A' is HV; nets 2 and 3 are Default


def _rule(cond):
    return f'(rule "hv" (constraint clearance (min 0.6mm)) (condition "{cond}"))'


def _values(td, name, cond):
    from kicad_dru import ScopedClearanceRules
    b = os.path.join(td, name + '.kicad_pcb')
    write_board(b, classes=HV, patterns=PATTERNS, dru=_rule(cond))
    sc = ScopedClearanceRules.for_board(pcb_data_for(b), b)
    if sc is None:
        return None
    return (sc.pair_value(1, 'track', 2, 'track'), sc.pair_value(2, 'track', 3, 'track'),
            sc.pair_value(2, 'track', 1, 'pad'))


def same_values(td):
    print('1. the term does not change what the rule binds')
    plain = _values(td, 'plain', "A.NetClass == 'HV'")
    check('the plain class rule is in the channel (baseline)',
          plain is not None and plain[0] and plain[0][0] == 0.6, f'{plain}')
    for i, cond in enumerate(("A.NetClass == 'HV' && A.Net != B.Net",
                              "A.Net != B.Net && A.NetClass == 'HV'",
                              "A.hasNetclass('HV') && B.Net != A.Net")):
        got = _values(td, f'term{i}', cond)
        check(f'{cond!r}: taken, same pair values as the plain rule',
              got is not None and [g and g[0] for g in got] == [p and p[0] for p in plain],
              f'{got} vs {plain}')


def kept_terms(td):
    print('2. only && conjuncts are dropped')
    from design_rules import parse_condition
    try:
        from kicad_dru import strip_distinct_nets
    except ImportError as e:
        check('kicad_dru.strip_distinct_nets exists', False, str(e))
        return
    for cond in ("A.NetClass == 'HV' || A.Net != B.Net",
                 "A.NetClass == 'HV' && !(A.Net != B.Net)"):
        node = parse_condition(cond)
        check(f'{cond!r}: left as it is', strip_distinct_nets(node) == node)
    check("a rule whose only condition is the term stays out of the channel",
          _values(td, 'only', 'A.Net != B.Net') is None)


def grader(td):
    print('3. check_drc grades the rule with the term')
    b = os.path.join(td, 'g.kicad_pcb')
    write_board(b, classes=HV, patterns=PATTERNS,
                dru=_rule("A.NetClass == 'HV' && A.Net != B.Net"),
                segments=[(5, 10, 15, 10, 0.2, 'F.Cu', 1), (5, 10.6, 15, 10.6, 0.2, 'F.Cu', 2)])
    r = subprocess.run(PY + [os.path.join('py_router', 'check_drc.py'), b, '--clearance', '0.1'],
                       capture_output=True, text=True, cwd=REPO)
    check('an HV track 0.4 mm from another net under the 0.6 mm rule: flagged',
          r.returncode != 0 and 'NOT GRADED' not in r.stdout, r.stdout[-400:])


def main():
    with tempfile.TemporaryDirectory() as td:
        same_values(td)
        kept_terms(td)
        grader(td)
    if failures:
        print(f'FAIL: {len(failures)}: ' + '; '.join(failures))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
