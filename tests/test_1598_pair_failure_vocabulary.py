#!/usr/bin/env python3
"""#1598: the skills read route_diff's pair results, in route_diff's own words.

`diagnose-routing-failures` said in its description that it diagnoses
route_diff runs, and its body read none of route_diff's pair keys:
`failed_diff_pairs`, `single_ended_diff_pairs`, `member_incomplete_pairs` and
the rest occurred 0 times, and its only retry was a route.py command.
`plan-pcb-routing` gated the all-nets step on fanout failures only, so a run
spent hours of all-nets routes while pair steps had failed.

Both skills now read the pair results. This file holds what they cite to what
the code emits, derived from the code rather than copied into this file:

  1. every route_diff summary key the two skills cite exists in route_diff's
     `summary` dict, and the gate's keys are cited by both;
  2. every pair_reports field and `outcome` value diagnose-routing-failures
     cites is one build_pair_reports writes;
  3. every `failure_reason` code route_diff can emit -- classify_diff_pair_
     failure's returns, each `reason=` handed to record_pair_diag, each
     `fail_reason = '<code>'` refinement, each literal `failure_reason` --
     has a row in diagnose-routing-failures' pair table, and every code the
     table names is one of them;
  4. every `intra_pair_matching` status route_diff can ship (the matcher's
     regrade and its two literal records) is defined by both skills, in
     diagnose-routing-failures' "route_diff runs" section and in the routing
     skill's all-nets gate, with the AC-coupled pass's number named (#1669);
  5. negative controls: a table missing a code, a cited key route_diff does not
     emit and a status left undefined are all reported.

Static: reads sources and skills, spawns nothing.

    python3 -X utf8 tests/test_1598_pair_failure_vocabulary.py
"""
import ast
import os
import re
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
PYR = os.path.join(ROOT, 'py_router')
SKILLS = os.path.join(ROOT, '.claude', 'skills')
DIAGNOSE = os.path.join(SKILLS, 'diagnose-routing-failures', 'SKILL.md')
ROUTING = os.path.join(SKILLS, 'plan-pcb-routing', 'SKILL.md')

#: The pair keys the all-nets gate reads (#1598's Expected): both skills
#: must cite each of them.
GATE_KEYS = ('failed_diff_pairs', 'single_ended_diff_pairs',
             'member_incomplete_pairs', 'pair_reports')


def _read(path):
    with open(path, encoding='utf-8') as fh:
        return fh.read()


def _tree(name):
    return ast.parse(_read(os.path.join(PYR, name)))


def _func(tree, name):
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    raise AssertionError(f'{name} not found')


def _strs(node):
    return {n.value for n in ast.walk(node)
            if isinstance(n, ast.Constant) and isinstance(n.value, str)}


def summary_keys():
    """The keys of route_diff's `summary = {...}` (the JSON_SUMMARY dict)."""
    keys = set()
    for node in ast.walk(_tree('route_diff.py')):
        if (isinstance(node, ast.Assign) and isinstance(node.value, ast.Dict)
                and any(getattr(t, 'id', None) == 'summary' for t in node.targets)):
            keys |= {k.value for k in node.value.keys
                     if isinstance(k, ast.Constant)}
    assert 'failed_diff_pairs' in keys, sorted(keys)
    return keys


def pair_report_vocabulary():
    """(fields, outcomes): the keys a pair_reports record can carry, and the
    `outcome` values build_pair_reports assigns."""
    fn = _func(_tree('diff_pair_custody.py'), 'build_pair_reports')
    fields, outcomes = set(), set()
    for node in ast.walk(fn):
        if isinstance(node, ast.Dict):
            fields |= {k.value for k in node.keys if isinstance(k, ast.Constant)}
            for k, v in zip(node.keys, node.values):
                if isinstance(k, ast.Constant) and k.value == 'outcome' \
                        and isinstance(v, ast.Constant):
                    outcomes.add(v.value)
        elif isinstance(node, ast.Subscript) and isinstance(node.ctx, ast.Store) \
                and isinstance(node.slice, ast.Constant):
            fields.add(node.slice.value)
        elif isinstance(node, ast.Assign) and any(
                getattr(t, 'id', None) == 'outcome' for t in node.targets):
            outcomes |= _strs(node.value)
    assert {'failure_reason', 'outcome'} <= fields, sorted(fields)
    return fields, outcomes


def _returned(expr):
    """The string values `expr` can evaluate to: a constant, or either arm
    of a conditional -- never the strings its tests read."""
    if isinstance(expr, ast.Constant) and isinstance(expr.value, str):
        return {expr.value}
    if isinstance(expr, ast.IfExp):
        return _returned(expr.body) | _returned(expr.orelse)
    return set()


def failure_reason_codes():
    """Every failure_reason value route_diff's pair machinery can write."""
    codes = set()
    custody = _tree('diff_pair_custody.py')
    for node in ast.walk(_func(custody, 'classify_diff_pair_failure')):
        if isinstance(node, ast.Return) and node.value is not None:
            codes |= _returned(node.value)
    for name in sorted(os.listdir(PYR)):
        if not name.endswith('.py'):
            continue
        for node in ast.walk(_tree(name)):
            if isinstance(node, ast.Call) and getattr(
                    node.func, 'id', None) == 'record_pair_diag':
                for kw in node.keywords:
                    if kw.arg == 'reason' and isinstance(kw.value, ast.Constant):
                        codes.add(kw.value.value)
            elif isinstance(node, ast.Assign) and isinstance(node.value, ast.Constant) \
                    and isinstance(node.value.value, str) and any(
                        getattr(t, 'id', '').endswith('fail_reason')
                        for t in node.targets):
                codes.add(node.value.value)
            elif isinstance(node, ast.Dict):
                for k, v in zip(node.keys, node.values):
                    if isinstance(k, ast.Constant) and k.value == 'failure_reason' \
                            and isinstance(v, ast.Constant) and isinstance(v.value, str):
                        codes.add(v.value)
    codes.discard('unknown')        # "nothing was recorded", not a mode
    assert {'congestion', 'connector-graze', 'electrically-short'} <= codes, \
        sorted(codes)
    return codes


def table_codes(text):
    """The codes in diagnose-routing-failures' pair table: backticked tokens
    in the first cell of each row after the `failure_reason` header, outside
    the cell's parentheses (which say where else the pair shows)."""
    rows = text.split('| `failure_reason` |', 1)
    assert len(rows) == 2, 'the pair table header is gone'
    out = set()
    for line in rows[1].splitlines()[2:]:
        if not line.startswith('|'):
            break
        cell = re.sub(r'\([^)]*\)', '', line.split('|')[1])
        out |= set(re.findall(r'`([a-z][a-z-]+)`', cell))
    return out


def cited_pair_keys(text):
    """Backticked summary-looking keys: `*_pairs`, `*_followup_nets`, and the
    two route_diff record names."""
    return set(re.findall(
        r'`((?:[a-z_]+_pairs)|single_ended_followup_nets|single_ended_fallback|'
        r'casualty_reconcile)`', text))


def _status_values(target, value):
    """The strings `value` assigns to a `rec['status']` in `target`, tuple
    assignments included."""
    if isinstance(target, ast.Tuple) and isinstance(value, ast.Tuple):
        out = set()
        for t, v in zip(target.elts, value.elts):
            out |= _status_values(t, v)
        return out
    if (isinstance(target, ast.Subscript) and isinstance(target.slice, ast.Constant)
            and target.slice.value == 'status'):
        return _returned(value)
    return set()


def intra_pair_statuses():
    """Every status an `intra_pair_matching` record can ship with: what
    regrade_intra_pair_reports assigns, and the `rec.update(status='...')`
    literals in run_intra_pair_matching. The regrade rewrites every other
    status, so the matcher's own words (`improved`) never ship."""
    tree = _tree('length_matching.py')
    out = set()
    for node in ast.walk(_func(tree, 'regrade_intra_pair_reports')):
        if isinstance(node, ast.Assign):
            for t in node.targets:
                out |= _status_values(t, node.value)
    for node in ast.walk(_func(tree, 'run_intra_pair_matching')):
        if isinstance(node, ast.Call) and getattr(node.func, 'attr', '') == 'update':
            for kw in node.keywords:
                if kw.arg == 'status' and isinstance(kw.value, ast.Constant):
                    out.add(kw.value.value)
    assert {'matched', 'within-tolerance', 'unmatched', 'not-matchable', 'skipped'} == out, \
        sorted(out)
    return out


def status_problems(diag, routing, statuses):
    out = []
    sect = diag.split('### route_diff runs', 1)
    body = sect[1].split('\n## ', 1)[0] if len(sect) == 2 else ''
    gate = routing.split('A pair is ready for the all-nets step', 1)
    gate = gate[1].split('### Stop conditions', 1)[0] if len(gate) == 2 else ''
    for where, text in (('diagnose-routing-failures', body), ('plan-pcb-routing', gate)):
        for st in sorted(statuses):
            if f'`{st}`' not in text:
                out.append(f'{where} does not define `intra_pair_matching` status `{st}`')
        if 'ac_coupled_xnets' not in text:
            out.append(f'{where} does not name `ac_coupled_xnets` for a `skipped` pair')
    return out


def problems(diag, routing, keys, fields, outcomes, codes):
    out = []
    for where, text in (('diagnose-routing-failures', diag),
                        ('plan-pcb-routing', routing)):
        for k in sorted(cited_pair_keys(text) - keys):
            out.append(f'{where} cites `{k}`, which route_diff\'s summary '
                       f'does not emit')
        for k in GATE_KEYS:
            if f'`{k}' not in text:
                out.append(f'{where} does not read `{k}`')
    tab = table_codes(diag)
    for c in sorted(codes - tab):
        out.append(f'failure_reason `{c}` has no row in the pair table')
    for c in sorted(tab - codes):
        out.append(f'the pair table names `{c}`, which route_diff never emits')
    sect = diag.split('### route_diff runs', 1)
    if len(sect) != 2:
        out.append('diagnose-routing-failures has no "route_diff runs" section')
    else:
        body = sect[1].split('\n## ', 1)[0]
        for f in ('outcome', 'failure_reason', 'failure_stage',
                  'incomplete_members', 'blocking_nets', 'casualty',
                  'se_fallback', 'copper', 'no_coupled_trunk'):
            if f'`{f}`' in body and f not in fields:
                out.append(f'cites pair_reports field `{f}`, which '
                           f'build_pair_reports does not write')
        m = re.search(r'`outcome`\s*\(([^)]*)\)', body)
        cited = set(re.findall(r'`([a-z]+)`', m.group(1))) if m else set()
        if not cited:
            out.append('the section lists no `outcome` values')
        for o in sorted(cited - outcomes):
            out.append(f'cites outcome `{o}`, which build_pair_reports never '
                       f'assigns')
        for o in sorted(outcomes - cited):
            out.append(f'outcome `{o}` is not listed')
    return out


def test_the_skills_read_route_diffs_pair_results_in_its_words():
    keys = summary_keys()
    fields, outcomes = pair_report_vocabulary()
    codes = failure_reason_codes()
    bad = problems(_read(DIAGNOSE), _read(ROUTING), keys, fields, outcomes,
                   codes)
    assert not bad, '\n  ' + '\n  '.join(bad)
    print(f'  PASS: {len(codes)} failure_reason codes, {len(outcomes)} '
          f'outcomes and the gate keys {", ".join(GATE_KEYS)} are read in '
          f'route_diff\'s own words')


def test_both_skills_define_every_intra_pair_status():
    statuses = intra_pair_statuses()
    bad = status_problems(_read(DIAGNOSE), _read(ROUTING), statuses)
    assert not bad, '\n  ' + '\n  '.join(bad)
    print(f'  PASS: both skills define {", ".join(sorted(statuses))}')


def test_the_status_check_trips_on_an_undefined_status():
    statuses = intra_pair_statuses()
    diag, routing = _read(DIAGNOSE), _read(ROUTING)
    bad = status_problems(diag.replace('`not-matchable`', 'unmatchable'), routing, statuses)
    assert any('diagnose-routing-failures does not define' in b and 'not-matchable' in b
               for b in bad), bad
    bad = status_problems(diag, routing.replace('`skipped`', 'passed over'), statuses)
    assert any('plan-pcb-routing does not define' in b and 'skipped' in b for b in bad), bad
    print('  PASS: an undefined status is reported, per skill')


def test_the_check_trips_on_a_missing_code_and_an_invented_key():
    keys = summary_keys()
    fields, outcomes = pair_report_vocabulary()
    codes = failure_reason_codes()
    diag, routing = _read(DIAGNOSE), _read(ROUTING)
    dropped = '\n'.join(ln for ln in diag.splitlines()
                        if not ln.startswith('| `needs-fanout`'))
    bad = problems(dropped, routing, keys, fields, outcomes, codes)
    assert any('`needs-fanout` has no row' in b for b in bad), bad
    invented = routing + '\nread `stranded_diff_pairs` too\n'
    bad = problems(diag, invented, keys, fields, outcomes, codes)
    assert any('`stranded_diff_pairs`' in b for b in bad), bad
    ungated = diag.replace('`member_incomplete_pairs`', '`member_incomplete`')
    bad = problems(ungated, routing, keys, fields, outcomes, codes)
    assert any('does not read `member_incomplete_pairs`' in b for b in bad), bad
    print('  PASS: a missing code, an invented key and an unread gate key '
          'are all reported')


if __name__ == '__main__':
    for _name, _fn in sorted(globals().items()):
        if _name.startswith('test_') and callable(_fn):
            print(f'--- {_name}')
            _fn()
    print('ALL PASS')
