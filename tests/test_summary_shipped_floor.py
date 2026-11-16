#!/usr/bin/env python3
"""route.py's summaries report the clearance floor the run WROTE BACK.

`min_clearance_used` and `design_rules` were measured when a JSON_SUMMARY was
BUILT, and pass 1's is built before the in-run plane finalize, whose
fine-pitch plane taps descend below `--clearance`. main()'s .kicad_pro
writeback reads the live ledger afterwards. Measured on kit-dev-coldfire
(`--nets GND +3.3V`, clearance 0.2): the project was written at 0.15 ("DEFAULT
CLASS CLEARANCE LOWERED BY A DESCENT"), while JSON_SUMMARY_MIN, --json-out and
the log merge all said `min_clearance_used: 0.2`, and `design_rules` carried
no `clearance` row. A harness grading "at min_clearance_used" then graded the
board stricter than it was routed -- phantom violations (CLAUDE.md: grade at
the floor the step wrote back).

  A. route_summary: the end-of-run re-stamp reaches every summary, keeps each
     summary's own `unsupported_rules`, prints a JSON_SHIPPED_RULES record only
     when a printed summary said something else, and the log merge applies it,
     so --json-out and merge_route_summaries(log) stay one document (#830).
  A2. the grades of a step's own copper (`STEP_RULE_KEYS`: hole_rules and
     the rest) survive a ledger re-stamp, and the outermost run's grade of
     the copper it ships replaces them in --json-out and the log merge.
  B. end to end on the kit board: the pour, then route.py `--nets GND`. The
     run must descend (else this fixture tests nothing -- BROKEN), and every
     channel must then report the floor the .kicad_pro says.

    python3 -X utf8 tests/test_summary_shipped_floor.py
"""
import json
import os
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from run_utils import check as run_check, evidence  # noqa: E402

FAILS = []


def check(name, cond, detail=''):
    print(('  PASS ' if cond else '  FAIL ') + name
          + (f'\n        {detail}' if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


def row_a_restamp_and_merge():
    print('A. the re-stamp, its printed record, and the log merge')
    try:
        import route_summary as rs
        from route_summary import (restamp_shipped_rules, sink_summary,
                                   merge_route_summaries, merge_summaries)
    except ImportError as e:
        check('route_summary carries the shipped-floor re-stamp', False, str(e))
        return
    ledger0 = {'count': 0, 'narrowed': [], 'min_delivered': {}}
    ledger1 = {'count': 1, 'min_delivered': {'clearance': 0.15},
               'narrowed': [{'net': 7, 'net_name': 'GND', 'kind': 'clearance',
                             'requested': 0.2, 'delivered': 0.15,
                             'site': 'plane tap', 'count': 1}]}
    pass1 = {'successful': 1, 'failed': 0, 'min_clearance_used': 0.2,
             'design_rules': dict(ledger0, unsupported_rules=['rule_x'])}
    sub = {'successful': 0, 'failed': 0, 'min_clearance_used': 0.2,
           'design_rules': dict(ledger0, unsupported_rules=[])}
    rs.reset_run_state()
    log = ''
    for s in (pass1, sub):
        log += 'JSON_SUMMARY: ' + json.dumps(s) + '\n'
        sink_summary(s)

    rec = restamp_shipped_rules(0.15, ledger1)
    check('every summary now says the shipped floor',
          [s['min_clearance_used'] for s in rs.SUMMARY_SINK] == [0.15, 0.15])
    check("...and the shipped ledger, keeping each summary's unsupported_rules",
          pass1['design_rules'] == dict(ledger1, unsupported_rules=['rule_x'])
          and sub['design_rules'] == dict(ledger1, unsupported_rules=[]),
          f"{pass1['design_rules']} / {sub['design_rules']}")
    check('a record is returned because the printed summaries said 0.2',
          rec == {'min_clearance_used': 0.15, 'design_rules': ledger1}, str(rec))

    stale = merge_route_summaries(log)
    check('control: a log WITHOUT the record still merges to the stale 0.2',
          stale['min_clearance_used'] == 0.2, str(stale.get('min_clearance_used')))
    log += 'JSON_SHIPPED_RULES: ' + json.dumps(rec) + '\n'
    from_log = merge_route_summaries(log)
    from_file = json.loads(json.dumps(merge_summaries(list(rs.SUMMARY_SINK))))
    check('the log merge applies the record (0.15, clearance row present)',
          from_log['min_clearance_used'] == 0.15
          and from_log['design_rules']['min_delivered'] == {'clearance': 0.15},
          str(from_log.get('design_rules')))
    check('--json-out (merge of the sink) == merge of the log (#830)',
          from_log == from_file,
          f'log {from_log}\n        file {from_file}')

    # A record printed BEFORE the run's last summary describes an earlier run.
    early = ('JSON_SHIPPED_RULES: ' + json.dumps(rec) + '\n'
             + 'JSON_SUMMARY: ' + json.dumps(dict(pass1, min_clearance_used=0.2))
             + '\n')
    check('a record that precedes the last summary is not applied',
          merge_route_summaries(early)['min_clearance_used'] == 0.2)

    # Nothing moved after printing: no record, so an ordinary log is unchanged.
    rs.reset_run_state()
    same = {'min_clearance_used': 0.2,
            'design_rules': dict(ledger0, unsupported_rules=[])}
    sink_summary(same)
    check('the sink and its printed record grow together',
          len(rs.SUMMARY_SINK) == len(rs.PRINTED_RULES) == 1)
    check('no record when every printed summary already said it',
          restamp_shipped_rules(0.2, ledger0) is None)
    rs.reset_run_state()


def row_a2_step_rule_grades():
    print('A2. the grades of the step\'s own copper survive the re-stamp')
    import route_summary as rs
    from route_summary import (restamp_shipped_rules, sink_summary,
                               merge_route_summaries, merge_summaries,
                               stamp_design_rules, STEP_RULE_KEYS)
    check('STEP_RULE_KEYS are the keys disclose_step_rules returns',
          set(STEP_RULE_KEYS) == {'annular_rules', 'via_rules', 'edge_rules',
                                  'hole_rules', 'board_setup_vias'}, str(STEP_RULE_KEYS))
    ledger = {'count': 0, 'narrowed': [], 'min_delivered': {}}
    lap_rows = [{'rule': 'h', 'min': 0.6, 'rows': 6, 'worst_mm': 0.34, 'nets': ['A', 'B']}]
    ship_rows = [{'rule': 'h', 'min': 0.6, 'rows': 4, 'worst_mm': 0.34, 'nets': ['A', 'B']}]
    pass1 = {'min_clearance_used': 0.2,
             'design_rules': dict(ledger, unsupported_rules=[])}
    lap = {'min_clearance_used': 0.2,
           'design_rules': dict(ledger, unsupported_rules=[], hole_rules=lap_rows)}

    # a ledger re-stamp (the shipped width / via rows) keeps each summary's grade
    doc = {'design_rules': dict(lap['design_rules'])}
    stamp_design_rules(doc, dict(ledger, count=1))
    check('a ledger re-stamp keeps hole_rules and takes the new ledger',
          doc['design_rules'].get('hole_rules') == lap_rows
          and doc['design_rules']['count'] == 1, str(doc))
    stamp_design_rules(doc, ledger, {})
    check('a shipped grade that breaks nothing removes the stale rows',
          'hole_rules' not in doc['design_rules'], str(doc))

    rs.reset_run_state()
    log = ''
    for s in (pass1, lap):
        log += 'JSON_SUMMARY: ' + json.dumps(s) + '\n'
        sink_summary(s)
    # control: with no shipped grade the ledger re-stamp changes nothing
    check('control: an unchanged ledger and no shipped grade print no record',
          restamp_shipped_rules(0.2, ledger) is None)
    check('...and the merge keeps the last pass\'s rows (the old --json-out lost them)',
          merge_summaries(list(rs.SUMMARY_SINK))['design_rules'].get('hole_rules')
          == lap_rows)
    rec = restamp_shipped_rules(0.2, ledger, {'hole_rules': ship_rows})
    check('the shipped grade is a record of its own',
          rec is not None and rec.get('step_rules') == {'hole_rules': ship_rows},
          str(rec))
    from_file = json.loads(json.dumps(merge_summaries(list(rs.SUMMARY_SINK))))
    check('--json-out carries the grade of the shipped copper',
          from_file['design_rules'].get('hole_rules') == ship_rows,
          str(from_file['design_rules']))
    log += 'JSON_SHIPPED_RULES: ' + json.dumps(rec) + '\n'
    from_log = merge_route_summaries(log)
    check('the log merge applies it too, one document (#830)',
          from_log['design_rules'] == from_file['design_rules'],
          f"log {from_log['design_rules']}\n        file {from_file['design_rules']}")
    rs.reset_run_state()


BASE = ['--track-width', '0.2', '--clearance', '0.2', '--via-size', '0.5',
        '--via-drill', '0.4', '--hole-to-hole-clearance', '0.3',
        '--layers', 'F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']


def row_b_kit_board():
    print('B. kit-dev-coldfire: the plane finalize descends, every channel says so')
    src = evidence(os.path.join(ROOT, 'kicad_files',
                                'kit-dev-coldfire-xilinx_5213.kicad_pcb'), 'fixture')
    py = [sys.executable, '-X', 'utf8']
    with tempfile.TemporaryDirectory(prefix='shipped_floor_') as td:
        poured = os.path.join(td, 'poured.kicad_pcb')
        out = os.path.join(td, 'routed.kicad_pcb')
        js = os.path.join(td, 'routed.json')
        run_check(py + [os.path.join(ROOT, 'py_router', 'route_planes.py'), src,
                        poured, '--nets', '+3.3V', 'GND', '+3.3V', 'GND',
                        '--plane-layers', 'F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']
                  + BASE, accept=True)
        r = run_check(py + [os.path.join(ROOT, 'py_router', 'route.py'),
                            evidence(poured, 'poured board'), out, '--nets', 'GND',
                            '--json-out', js] + BASE, accept=True)
        log = r.stdout
        with open(os.path.splitext(evidence(out, 'routed board'))[0]
                  + '.kicad_pro', encoding='utf-8') as fh:
            pro = json.load(fh)
        floor = pro['board']['design_settings']['rules']['min_clearance']
        if not floor < 0.2 - 1e-9:
            raise AssertionError(
                f'BROKEN TEST: the run wrote back min_clearance {floor}, not a '
                f'descent below 0.2 -- this fixture no longer exercises a '
                f'finalize descent, so it tests nothing')
        print(f'  (the writeback stored rules.min_clearance {floor})')

        mins = [ln for ln in log.splitlines()
                if ln.startswith('JSON_SUMMARY_MIN: ')]
        check('exactly one JSON_SUMMARY_MIN', len(mins) == 1, str(len(mins)))
        got_min = json.loads(mins[-1].split(': ', 1)[1]).get('min_clearance_used')
        check('JSON_SUMMARY_MIN min_clearance_used == the written floor',
              got_min == floor, f'MIN {got_min} vs .kicad_pro {floor}')
        with open(evidence(js, '--json-out'), encoding='utf-8') as fh:
            doc = json.load(fh)
        check('--json-out min_clearance_used == the written floor',
              doc.get('min_clearance_used') == floor,
              f"--json-out {doc.get('min_clearance_used')} vs {floor}")
        dr = doc.get('design_rules') or {}
        check('--json-out design_rules carries the descent (min_delivered)',
              (dr.get('min_delivered') or {}).get('clearance') == floor,
              str(dr.get('min_delivered')))
        from route_summary import merge_route_summaries
        merged = merge_route_summaries(log)
        check('the log merge says the written floor too',
              merged and merged.get('min_clearance_used') == floor,
              str(merged and merged.get('min_clearance_used')))
        # Only the two keys this file is about: keys the finalize stamps after
        # the printed summary (finalize_excluded_nets, oracle_reconnect) are
        # a separate #830 gap on a run like this one.
        check('--json-out and the log merge agree on the floor and the ledger',
              all((merged or {}).get(k) == doc.get(k)
                  for k in ('min_clearance_used', 'design_rules')),
              str([k for k in ('min_clearance_used', 'design_rules')
                   if (merged or {}).get(k) != doc.get(k)]))


if __name__ == '__main__':
    row_a_restamp_and_merge()
    row_a2_step_rule_grades()
    row_b_kit_board()
    print()
    if FAILS:
        print(f'FAILED: {len(FAILS)} check(s): {FAILS}')
        sys.exit(1)
    print('All checks passed.')
