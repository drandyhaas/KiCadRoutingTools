"""#1528: a NetName-scoped .kicad_dru clearance rule is graded at its value.

    (rule "gnd_hs" (constraint clearance (min 0.2mm))
      (condition "A.NetName == '/GND' && B.NetClass == 'HS'"))

lets /GND come within 0.2 mm of a 0.5 mm class. check_drc named it NOT
GRADED and graded every /GND<->HS pair at 0.5, so board_score counted copper
KiCad accepts as blocking DRC. Now:

1. KiCad's string compare, measured on kicad-cli 10.0.6: `A.NetName ==` is
   case-insensitive and a literal with '*'/'?' is a wildcard; a via's
   `A.Layer` is undefined (both `==` and `!=` false);
2. the scoped channel (#1491) takes NetName rules, for the grader
   (pair-exact) and the router's stamps alike;
3. check_drc grades the pairs at the rule: the /GND<->HS items leave the
   count (KiCad: 0 with the rule), the rest stay, and the rule is no longer
   listed ungraded;
4. a rule that stays unmodelled (insideCourtyard) is still NOT GRADED, and
   the violations it may make legal are TAGGED `unverified_rule` and counted
   apart (`UNVERIFIED (clearance): N`, JSON `unverified`, board_score
   `drc.unverified`) -- still counted, since a courtyard term nothing here
   reads makes nearly every HS pair "maybe".

    python3 tests/test_1528_netname_scoped_clearance.py
"""
import fnmatch
import json
import os
import shutil
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)
import run_utils  # noqa: E402

RUN_ALL_TIMEOUT = 600
PY = [sys.executable, '-X', 'utf8']
BOARD = os.path.join(REPO, 'kicad_files', 'lvds_converter_dualclk.kicad_pcb')
PATS = ['/CLK*', '/DATA*']
GND_HS = ('(rule "gnd_hs" (constraint clearance (min 0.2mm)) '
          '(condition "A.NetName == \'/GND\' && B.NetClass == \'HS\'"))')
COURT = ('(rule "court_hs" (constraint clearance (min 0.2mm)) '
         '(condition "A.insideCourtyard(\'IC4\') && A.NetClass == \'HS\'"))')
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def hs(name):
    return any(fnmatch.fnmatchcase(name or '', p) for p in PATS)


def write_project(board, hs_mm):
    def cls(name, c, prio):
        return {'name': name, 'clearance': c, 'track_width': 0.2, 'via_diameter': 0.6,
                'via_drill': 0.3, 'diff_pair_gap': c, 'diff_pair_width': 0.2,
                'priority': prio}
    stem = os.path.splitext(board)[0]
    pro = {'board': {'design_settings': {'rules': {'min_clearance': 0.0}}},
           'net_settings': {'meta': {'version': 4},
                            'classes': [cls('Default', 0.2, 2147483647),
                                        cls('HS', hs_mm, 0)],
                            'netclass_patterns': [{'netclass': 'HS', 'pattern': p}
                                                  for p in PATS]},
           'meta': {'filename': os.path.basename(stem) + '.kicad_pro', 'version': 3}}
    with open(stem + '.kicad_pro', 'w', encoding='utf-8') as fh:
        json.dump(pro, fh, indent=2)


def write_dru(board, *rules):
    with open(os.path.splitext(board)[0] + '.kicad_dru', 'w', encoding='utf-8') as fh:
        fh.write('(version 1)\n' + '\n'.join(rules) + '\n')


def copy_board(src, dst):
    run_utils.check(PY + [os.path.join(REPO, 'py_router', 'copy_board.py'), src, dst],
                    accept=True)
    return run_utils.evidence(dst, 'copied board')


def drc(board, td, tag):
    """(check_drc stdout, its --json document)."""
    out = os.path.join(td, tag + '.json')
    r = subprocess.run(PY + [os.path.join(REPO, 'py_router', 'check_drc.py'), board,
                             '--json', out], capture_output=True, text=True, cwd=REPO)
    if 'Traceback' in r.stdout + r.stderr or not os.path.isfile(out):
        raise AssertionError(f'BROKEN TEST: check_drc crashed\n{(r.stdout + r.stderr)[-800:]}')
    with open(out, encoding='utf-8') as fh:
        return r.stdout, json.load(fh)


def gnd_hs(doc):
    return [it for it in doc['items'] if not it.get('accepted')
            and '/GND' in (it.get('net1'), it.get('net2'))
            and (hs(it.get('net1')) or hs(it.get('net2')))]


def score(board):
    r = subprocess.run(PY + [os.path.join(REPO, 'py_tools', 'board_score.py'), board,
                             '--quiet'], capture_output=True, text=True, cwd=REPO)
    line = next((x for x in r.stdout.splitlines() if x.startswith('SCORE_JSON=')), '')
    if 'Traceback' in r.stdout + r.stderr or not line:
        raise AssertionError(f'BROKEN TEST: board_score\n{(r.stdout + r.stderr)[-800:]}')
    return json.loads(line[len('SCORE_JSON='):])


def evaluator():
    print("1. KiCad's compare semantics")
    import design_rules as d
    nm = getattr(d, 'netname_matches', None)
    check('NetName == is case-insensitive and wildcards a */? literal',
          nm is not None and nm('/CLK', '/clk') and nm('/CLK_N', '/CLK*')
          and nm('/CLK+', '/clk?') and not nm('/DATA', '/CLK*'))
    rule = d.parse_dru(GND_HS)[0][0]
    a = d.RuleItem(type='via', net_name='/gnd')
    b = d.RuleItem(type='track', netclasses=frozenset({'HS'}), effective_class='HS')
    check('the rule binds a /GND via (lower-case net) against an HS track, commutatively',
          d._rule_matches(rule, b, a, 'F.Cu', ['F.Cu', 'B.Cu']))
    via = d.RuleItem(type='via', layers=frozenset({'F.Cu', 'B.Cu'}))
    for op in ('==', '!='):
        r = d.parse_dru(f'(rule v (constraint disallow via) '
                        f'(condition "A.Layer {op} \'F.Cu\'"))')[0][0]
        check(f"a via's A.Layer is undefined: `A.Layer {op} 'F.Cu'` is false",
              not d._rule_matches(r, via, None, 'B.Cu', ['F.Cu', 'B.Cu']))
    maybe = getattr(d, 'rule_may_match', None)
    if maybe is None:
        check('three-valued rule reading exists', False)
        return
    cr = d.parse_dru(COURT)[0][0]
    ast = d.parse_condition_maybe(cr.condition_src)
    dflt = d.RuleItem(type='track', effective_class='Default')
    check('insideCourtyard && NetClass: "maybe" for an HS item, "no" for two Default ones',
          maybe(cr, ast, b, dflt, 'F.Cu', ['F.Cu']) is None
          and maybe(cr, ast, dflt, dflt, 'F.Cu', ['F.Cu']) is False)


def channel(board):
    print('2. the scoped channel takes the NetName rule')
    from kicad_parser import parse_kicad_pcb
    from kicad_dru import ScopedClearanceRules
    pcb = parse_kicad_pcb(board)
    sc = ScopedClearanceRules.for_board(pcb, board)
    check('the scoped channel selects gnd_hs', sc is not None and sc.names == ['gnd_hs'],
          f'{sc and sc.names}')
    if sc is None:
        return
    ids = {n.name: i for i, n in pcb.nets.items() if n.name}
    got = sc.pair_value(ids['/CLK'], 'track', ids['/GND'], 'via')
    check('an HS track vs a /GND via -> 0.2 (commutative)', got and got[0] == 0.2, f'{got}')
    check('two HS nets -> no rule', sc.pair_value(ids['/CLK'], 'track', ids['/DATA'],
                                                   'track') is None)
    from routing_config import GridRouteConfig
    from kicad_dru import install_track_clearances
    cfg = GridRouteConfig()
    cfg.clearance = 0.2
    hs_ids = [i for n, i in ids.items() if hs(n)]
    cfg.set_net_clearances({i: 0.5 for i in hs_ids}, hs_ids)
    install_track_clearances(cfg, None, board, pcb, routed_net_ids=hs_ids)
    check('routing the HS class, the router stamps /GND at the rule (0.2), not 0.5',
          abs(cfg.obstacle_clearance(ids['/GND']) - 0.2) < 1e-9,
          f"{cfg.obstacle_clearance(ids['/GND'])}")


def main():
    if not os.path.isfile(BOARD):
        print(f'SKIP: {BOARD} missing')
        return 77
    td = tempfile.mkdtemp(prefix='krt1528_')
    try:
        evaluator()

        # the whole board routed at 0.2 (HS = Default), then graded with HS at 0.5
        src = copy_board(BOARD, os.path.join(td, 'in.kicad_pcb'))
        write_project(src, 0.2)
        routed = os.path.join(td, 'r.kicad_pcb')
        run_utils.check(PY + [os.path.join(REPO, 'py_router', 'route.py'), src, routed],
                        accept=True)
        run_utils.evidence(routed, 'routed board')
        boards = {}
        for tag, rules in (('norule', ()), ('rule', (GND_HS,)), ('court', (COURT,))):
            boards[tag] = copy_board(routed, os.path.join(td, tag + '.kicad_pcb'))
            write_project(boards[tag], 0.5)
            if rules:
                write_dru(boards[tag], *rules)

        channel(boards['rule'])

        print('3. check_drc grades the NetName rule pair by pair')
        out_n, doc_n = drc(boards['norule'], td, 'norule')
        out_r, doc_r = drc(boards['rule'], td, 'rule')
        n_pairs = len(gnd_hs(doc_n))
        check('non-vacuity: without the rule /GND<->HS pairs violate the 0.5 class',
              n_pairs > 0, f'{n_pairs}')
        check('with the rule no /GND<->HS pair is a violation (KiCad: 0)',
              len(gnd_hs(doc_r)) == 0, f'{len(gnd_hs(doc_r))}')
        check('...and every other violation stays',
              doc_r['violations'] == doc_n['violations'] - n_pairs,
              f"{doc_r['violations']} vs {doc_n['violations']} - {n_pairs}")
        check('the rule is no longer NOT GRADED',
              "NOT GRADED (clearance): rule 'gnd_hs'" not in out_r)
        s = score(boards['rule'])
        d = (s.get('components') or {}).get('drc') or {}
        check('board_score: drc counts the graded value, gnd_hs not ungraded',
              d.get('count') == doc_r['violations'] and not d.get('ungraded_rules'),
              json.dumps(d)[:300])

        print('4. an unmodelled rule stays NOT GRADED; what it may relax is counted apart')
        out_c, doc_c = drc(boards['court'], td, 'court')
        check('court_hs is NOT GRADED', "NOT GRADED (clearance): rule 'court_hs'" in out_c)
        tagged = [it for it in doc_c['items'] if it.get('unverified_rule') == 'court_hs']
        check('the violations it may relax are tagged with it',
              tagged and doc_c.get('unverified') == len(tagged), f"{doc_c.get('unverified')}")
        check('they involve an HS net, the only class the rule can bind',
              all(hs(it.get('net1')) or hs(it.get('net2')) for it in tagged))
        check('...and stay counted', doc_c['violations'] == doc_n['violations'],
              f"{doc_c['violations']} vs {doc_n['violations']}")
        s = score(boards['court'])
        d = (s.get('components') or {}).get('drc') or {}
        check('board_score: drc.unverified counts them apart, the rule ungraded',
              d.get('unverified') == len(tagged) and d.get('ungraded_rules') == ['court_hs'],
              json.dumps(d)[:300])
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if failures:
        print(f'FAIL: {len(failures)}: ' + '; '.join(failures))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
