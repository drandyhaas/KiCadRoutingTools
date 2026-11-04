"""#1467: a cross-group `proximity` row moves its subject into its partner's
group, and an ANCHOR subject carries its whole group with it. A group so
emptied is dropped, with its own `near` / `apart` / `edge` hints. That fold is
by design; it used to be said only in each seed's own log, as "in their IC's
group" whatever the partner was.

Now the one fold (`grouping.fold_groups`) is disclosed by every reader:
check_groups WARNs once per dissolved group (the group that absorbs it, the
hints it drops), carries `moved` / `dissolved` in its JSON_SUMMARY and its
--json, and lists the groups as they will be placed; place_simple's seed
NOTE says "partner's group", its report carries `fold`, and the parent of a
several-seed run prints the fold once and writes it to judge.json.

    python3 tests/test_1467_fold_disclosed.py
"""
import argparse
import contextlib
import io
import json
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from run_utils import check, evidence, tool                   # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
FAILS = []


def ok(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}"
          + (f"  ({detail})" if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


# a group whose anchor follows a claim into another group, one of its parts
# following its own claim elsewhere: the group empties and is dropped
GROUPS = {'schema': 1, 'groups': [
    {'name': 'usb_in', 'refs': ['USB1'], 'anchor': 'USB1', 'near': ['power']},
    {'name': 'power', 'refs': ['U2', 'C1', 'C3'], 'anchor': 'U2', 'near': ['usb_in', 'core']},
    {'name': 'core', 'refs': ['U1', 'Y1', 'C2', 'C4', 'R3', 'R4'], 'anchor': 'U1',
     'near': ['autoreset']},
    {'name': 'autoreset', 'refs': ['Q1', 'Q2', 'R1', 'R2'], 'anchor': 'Q1',
     'near': ['prog'], 'apart': ['usb_in'], 'edge': 'W'},
    {'name': 'prog', 'refs': ['CON1', 'CON2', 'Ref*', 'Ref*~2'], 'anchor': 'CON1',
     'near': ['autoreset']}]}
ROWS = [{'ref': 'R1', 'near': 'U1', 'pads': {'R1': ['2'], 'U1': ['16']}, 'max_mm': 3.0,
         'basis': 'pad_edge', 'why': 'at its pin'},
        {'ref': 'Q1', 'near': 'CON1', 'pads': {'Q1': ['3'], 'CON1': ['2']}, 'max_mm': 10.0,
         'basis': 'pad_edge', 'why': 'by the header'}]


def unit():
    from placement.grouping import fold_groups, fold_text
    fold = fold_groups(GROUPS, ROWS)
    ok('the anchor carries its group: every part of it moves',
       fold['moved'] == {'Q1': ('autoreset', 'prog'), 'Q2': ('autoreset', 'prog'),
                         'R1': ('autoreset', 'core'), 'R2': ('autoreset', 'prog')},
       fold['moved'])
    d = fold['dissolved']
    ok('the emptied group is one dissolved record', [x['group'] for x in d] == ['autoreset'], d)
    if d:
        ok('it names the group that absorbs it, and where each part went',
           d[0]['into'] == 'prog' and d[0]['parts'] == {'prog': ['Q1', 'Q2', 'R2'],
                                                        'core': ['R1']}, d[0])
        ok('its own hints that act no more are named: apart and edge, not a near '
           'its parts now sit in',
           d[0]['lost'] == {'apart': ['usb_in'], 'edge': 'W'}, d[0]['lost'])
    names = [g['name'] for g in fold['doc']['groups']]
    core = next(g for g in fold['doc']['groups'] if g['name'] == 'core')
    ok('the placed groups drop it, and a hint naming it names the absorber',
       'autoreset' not in names and core.get('near') == ['prog'], (names, core))
    ok('the input is not modified',
       [g['name'] for g in GROUPS['groups']][3] == 'autoreset')
    moved_line, lines = fold_text(fold)
    ok("the moved line says partner's group, not IC's",
       moved_line.startswith("parts that follow a claim into their partner's group:")
       and "IC's" not in moved_line, moved_line)
    ok('one line per dissolved group, naming absorber and dropped hints',
       len(lines) == 1 and 'autoreset is dissolved into prog' in lines[0]
       and 'apart usb_in' in lines[0] and 'edge W' in lines[0], lines)
    none = fold_groups(GROUPS, [])
    ok('no claim, no fold', not none['moved'] and not none['dissolved']
       and none['doc'] is GROUPS)
    ok('a template member is a claim at its chip',
       fold_groups(GROUPS, [], [{'anchor': 'U1', 'members': [{'ref': 'C1'}]}])['moved']
       == {'C1': ('power', 'core')})


def check_groups_run(td):
    groups = os.path.join(td, 'groups.json')
    with open(groups, 'w', encoding='utf-8') as fh:
        json.dump(GROUPS, fh)
    intent = os.path.join(td, 'intent.json')
    with open(intent, 'w', encoding='utf-8') as fh:
        json.dump({'kind': 'floorplan-intent', 'schema': 1, 'proximity': ROWS}, fh)
    js = os.path.join(td, 'cg.json')
    r = check([sys.executable, '-X', 'utf8', tool('check_groups.py'), BOARD,
               '--groups', groups, '--intent', intent, '--json', js], accept=True)
    evidence(js, 'the check_groups report')
    out = r.stdout
    warns = [ln for ln in out.splitlines() if ln.startswith('  WARN  group autoreset')]
    ok('check_groups WARNs once for the dissolved group', len(warns) == 1, out)
    ok('the WARN names the absorber and the dropped hints',
       bool(warns) and 'dissolved into prog' in warns[0] and 'apart usb_in' in warns[0])
    line = next((ln for ln in out.splitlines() if ln.startswith('JSON_SUMMARY: ')), None)
    s = json.loads(line[len('JSON_SUMMARY: '):]) if line else {}
    ok('JSON_SUMMARY carries moved and dissolved',
       s.get('dissolved') == ['autoreset'] and s.get('moved', {}).get('Q2')
       == ['autoreset', 'prog'] and s.get('warnings', 0) >= 1, s)
    rep = json.load(open(js, encoding='utf-8'))
    ok('--json carries the dissolved record',
       [d['group'] for d in rep.get('dissolved') or []] == ['autoreset']
       and 'Q1' in (rep.get('moved') or {}), rep.get('dissolved'))
    rows = {ln.split()[0]: ln for ln in out.splitlines()
            if ln[:2] == '  ' and ln[2] != ' ' and len(ln.split()) > 2 and ln.split()[0] in
            ('core', 'autoreset', 'prog')}
    ok('the table lists the groups as they will be placed',
       'dissolved into prog' in rows.get('autoreset', '')
       and 'R1 (from autoreset)' in rows.get('core', '')
       and 'Q1 (from autoreset)' in rows.get('prog', ''), rows)


def parent(td):
    """The several-seed parent says the fold once and writes it to judge.json."""
    import place_simple
    fold = {'moved': {'Q1': ['autoreset', 'prog']},
            'dissolved': [{'group': 'autoreset', 'into': 'prog',
                           'parts': {'prog': ['Q1']}, 'lost': {'apart': ['usb_in']}}]}
    summaries = {s: {'final': {'cost': 1.0 + s}, 'illegal': [], 'fold': fold} for s in (0, 1)}
    a = argparse.Namespace(out=os.path.join(td, 's{seed}.kicad_pcb'), judge=2, board=BOARD)
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        place_simple._name_judged(a, [0, 1], summaries)
    out = buf.getvalue()
    ok('the parent prints the dissolved group once',
       out.count('autoreset is dissolved into prog') == 1, out)
    ok("the parent's moved line says partner's group",
       "partner's group: Q1 (autoreset -> prog)" in out, out)
    judge = json.load(open(os.path.join(td, 'judge.json'), encoding='utf-8'))
    ok('judge.json carries the fold', judge.get('fold') == fold, judge.get('fold'))


def seed_run(td):
    """One seed: the NOTE says partner's group, and the report carries `fold`."""
    groups, intent = os.path.join(td, 'groups.json'), os.path.join(td, 'intent.json')
    out, js = os.path.join(td, 'one.kicad_pcb'), os.path.join(td, 'one.json')
    r = check([sys.executable, '-X', 'utf8', tool('place_simple.py'), BOARD, out,
               '--groups', groups, '--intent', intent, '--seed', '0', '--moves', '300',
               '--json', js], accept=True)
    evidence(js, 'the place_simple report')
    ok("the seed's NOTE says partner's group",
       "NOTE: parts that follow a claim into their partner's group" in r.stdout
       and "IC's group:" not in r.stdout, r.stdout[-2000:])
    ok("the seed's NOTE names the dissolved group",
       'NOTE: group autoreset is dissolved into prog' in r.stdout)
    line = next((ln for ln in r.stdout.splitlines() if ln.startswith('JSON_SUMMARY: ')), '')
    s = json.loads(line[len('JSON_SUMMARY: '):]) if line else {}
    ok('JSON_SUMMARY carries fold',
       [d['group'] for d in (s.get('fold') or {}).get('dissolved') or []] == ['autoreset'],
       s.get('fold'))


def main():
    evidence(BOARD, 'the fixture board')
    unit()
    with tempfile.TemporaryDirectory(prefix='t1467_') as td:
        check_groups_run(td)
        parent(td)
        seed_run(td)
    if FAILS:
        print(f"FAIL: {len(FAILS)} check(s): {FAILS}")
        return 1
    print('PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
