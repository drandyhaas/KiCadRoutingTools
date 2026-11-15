#!/usr/bin/env python3
"""#1852: whether KiCad loads a `.kicad_dru` is asked of KiCad, on a stub board.

KiCad drops a whole rules file it cannot load and says nothing. The refusal
model (`design_rules.dru_refusals`, #1690) is hand-written, and KiCad 10
drops at least 31 forms it does not name. With kicad-cli installed, every
reader of the refusal finding -- `DesignRules.kicad_refused` (each routing
step's console line and `JSON_SUMMARY.design_rules.kicad_refuses_file`),
`read_board_kicad_refusals` (check_drc's line and `--json`) and
check_complete's grade of a board with no footprint to probe -- now asks
KiCad: one `kicad-cli pcb drc` of a stub board with the board's layer table,
the rules file and one probe rule (`dru_load`), cached on disk by the rules
text, the layers and the kicad-cli version. The model stays the fallback
without kicad-cli and the source of WHY.

DISCLOSURE ONLY: what the tools read and route is the same with and without
kicad-cli; only what they say about the file differs.

Part 1 needs no KiCad: KiCad's answer is injected. Part 2 asks kicad-cli:
the 31 forms (`test_1810`'s 13 and the 18 measured in its follow-up), the
model's own forms (`test_1690`'s), files that load, a layer named by its
user name, the cache, check_drc, check_complete, and two route.py runs with
and without kicad-cli. Without kicad-cli part 2 is skipped and the test exits
77 after part 1 passes.

    python3 tests/test_1852_dru_load_check.py
"""
import concurrent.futures
import contextlib
import io
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
sys.path.insert(0, REPO)

RUN_ALL_TIMEOUT = 900

# The cache is this test's own, so part 2 asks KiCad rather than reading an
# answer an earlier run left, and a child process shares it.
_CACHE = tempfile.mkdtemp(prefix='krt1852_cache_')
os.environ['XDG_CACHE_HOME'] = _CACHE

import run_utils  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402
import design_rules  # noqa: E402
from design_rules import DesignRules  # noqa: E402
from test_1810_check_complete_kicad_rules import DROPPED as DROPPED_1810  # noqa: E402
from test_1690_rule_readings import FILES_REFUSED, REFUSED  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
FIXTURE = os.path.join(TESTS, 'fixtures', '1643', 'edge_gap.kicad_pcb')
ROUTE_FIXTURE = os.path.join(TESTS, 'fixtures', '1603', 'edge_sma.kicad_pcb')
HEAD = '.kicad_dru: KiCad ENFORCES NO RULE OF THIS FILE'
WORKERS = 3

_OK = ('(rule "court gap" (constraint clearance (min 0.3mm))\n'
       "  (condition \"A.insideCourtyard('J1') && A.getField('x') == ''\"))\n")


def _before(rule):
    return '(version 1)\n' + rule + '\n' + _OK


#: Forms KiCad 10 drops a whole file for that the model did not name when
#: #1840 measured them (kicad-cli 10.0.6), beyond `test_1810`'s 13
DROPPED_1840 = (
    ('disallow twice in one rule',
     _before('(rule "w" (constraint disallow track) (constraint disallow via))')),
    ('clearance twice in one rule',
     _before('(rule "w" (constraint clearance (min 0.1mm)) (constraint clearance (max 2mm)))')),
    ('a lower-case layer name',
     _before('(rule "w" (layer "f.cu") (constraint track_width (min 0.1mm)))')),
    ('a layer named Top, unquoted',
     _before('(rule "w" (layer Top) (constraint track_width (min 0.1mm)))')),
    ('(layer) with no name', _before('(rule "w" (layer) (constraint track_width (min 0.1mm)))')),
    ('(layer ...) with two names',
     _before('(rule "w" (layer "F.Cu" "B.Cu") (constraint track_width (min 0.1mm)))')),
    ('(severity) with no value',
     _before('(rule "w" (severity) (constraint track_width (min 0.1mm)))')),
    ('(constraint) with no kind', _before('(rule "w" (constraint))')),
    ('an assertion with no string', _before('(rule "w" (constraint assertion))')),
    ('track_angle (min 10deg)', _before('(rule "w" (constraint track_angle (min 10deg)))')),
    ('a unitless skew', _before('(rule "w" (constraint skew (max 1)))')),
    ('a unitless diff_pair_uncoupled',
     _before('(rule "w" (constraint diff_pair_uncoupled (max 1)))')),
    ('a unitless text_height', _before('(rule "w" (constraint text_height (min 1)))')),
    ('a unitless thermal_relief_gap',
     _before('(rule "w" (constraint thermal_relief_gap (min 1)))')),
    ('min_resolved_spokes (min 2mm)',
     _before('(rule "w" (constraint min_resolved_spokes (min 2mm)))')),
    ('min_resolved_spokes (min 2)', _before('(rule "w" (constraint min_resolved_spokes (min 2)))')),
    ('zone_connection thermal', _before('(rule "w" (constraint zone_connection thermal))')),
    ('zone_connection with no style', _before('(rule "w" (constraint zone_connection))')),
)
DROPPED = tuple(DROPPED_1810) + DROPPED_1840
assert len(DROPPED) == 31, len(DROPPED)

#: Files KiCad loads, the neighbours of some dropped forms
LOADS = (
    ('the rule alone', '(version 1)\n' + _OK),
    ('min_resolved_spokes 2', _before('(rule "w" (constraint min_resolved_spokes 2))')),
    ('zone_connection solid', _before('(rule "w" (constraint zone_connection solid))')),
    ('track_angle (min 10)', _before('(rule "w" (constraint track_angle (min 10)))')),
    ('skew (max 1mm)', _before('(rule "w" (constraint skew (max 1mm)))')),
    ('a layer by a canonical name the board has not', _before(
        '(rule "w" (layer "In9.Cu") (constraint track_width (min 0.1mm)))')),
)

#: A rule on a layer by the user name `Top`: KiCad loads it on a board whose F.Cu
#: is called that, and drops it on one whose is not
TOP_RULE = _before('(rule "w" (layer "Top") (constraint track_width (min 0.1mm)))')

failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def stage(work, name, dru, board=FIXTURE, rename_top=False):
    d = os.path.join(work, name)
    os.makedirs(d)
    text = open(run_utils.evidence(board), encoding='utf-8', newline='').read()
    if rename_top:
        assert text.count('(0 "F.Cu" signal)') == 1, 'the fixture changed'
        text = text.replace('(0 "F.Cu" signal)', '(0 "F.Cu" signal "Top")')
    path = os.path.join(d, 'b.kicad_pcb')
    with open(path, 'w', encoding='utf-8', newline='') as fh:
        fh.write(text)
    if dru is not None:
        with open(os.path.join(d, 'b.kicad_dru'), 'w', encoding='utf-8') as fh:
            fh.write(dru)
    return run_utils.evidence(path)


def rules_of(path):
    return DesignRules.from_project(parse_kicad_pcb(path), path)


def model_of(path):
    """The refusal model's answer for the rules file beside ``path``."""
    with open(os.path.splitext(path)[0] + '.kicad_dru', encoding='utf-8') as fh:
        text = fh.read()
    return design_rules.dru_refusals(text, design_rules.parse_dru(text)[0])


@contextlib.contextmanager
def kicad_answer(answer):
    """`dru_load.kicad_loads_dru` answering ``answer`` (a dict), counted."""
    import dru_load
    real, calls = dru_load.kicad_loads_dru, []

    def fake(*a, **k):
        calls.append(a)
        return dict(answer)
    dru_load.kicad_loads_dru = fake
    try:
        yield calls
    finally:
        dru_load.kicad_loads_dru = real


def read_state(dr):
    """What a step reads of the rules: the whole table but the finding."""
    t = dr.table()
    t.pop('kicad_refused')
    return json.dumps(t, sort_keys=True, default=str)


# ---------------------------------------------------------------- part 1

def part1(work):
    print('1. KiCad\'s answer injected (no KiCad needed)')
    missed = DROPPED_1840[1][1]               # a form the model does not name
    known = '(version 1)\n(rule "typo" (condition "A.onLayer(\'F.Cu\')") ' \
            '(constraint track_width (min 0.7mm)))\n' + _OK
    b_missed = stage(work, 'p1_missed', missed)
    b_known = stage(work, 'p1_known', known)

    with kicad_answer({'loaded': None, 'why': 'kicad-cli not found'}) as calls:
        dr = rules_of(b_missed)
        check('building the table asks KiCad nothing (a table built to resolve sizes '
              'starts no kicad-cli)', calls == [], str(calls))
        dr.resolve('clearance', dr.item_for_net(1, 'track', 'F.Cu'), None, 'F.Cu')
        check('... nor does resolving a rule', calls == [], str(calls))
        got = dr.kicad_refused
        check('without kicad-cli: the model\'s answer, which misses this form',
              got == [] and model_of(b_missed) == [] and len(calls) == 1, f'{got} {calls}')
        dr.kicad_refused
        check('... asked once per table', len(calls) == 1, str(calls))
        dr = rules_of(b_known)
        check('without kicad-cli: the model\'s reasons stand',
              [r['rule'] for r in dr.kicad_refused] == ['typo']
              and 'kicad_version' not in dr.kicad_refused[0], str(dr.kicad_refused))

    with kicad_answer({'loaded': False, 'kicad_version': '10.9.9'}):
        dr = rules_of(b_missed)
        rows = dr.kicad_refused
        check('KiCad drops a file the model names nothing in: one row says so',
              len(rows) == 1 and rows[0]['rule'] == '' and 'KiCad refuses this file' in
              rows[0]['reason'] and 'stub board' in rows[0]['reason']
              and rows[0].get('kicad_version') == '10.9.9', str(rows))
        lines = dr.report_lines()
        check('... and the step\'s first line names KiCad\'s version',
              lines and lines[0].startswith(HEAD + ' (kicad-cli 10.9.9 does not load it): '),
              str(lines[:1]))
        dr = rules_of(b_known)
        check('KiCad drops a file the model names a cause in: the model says WHY',
              [r['rule'] for r in dr.kicad_refused] == ['typo']
              and 'A.onLayer() is not a function it knows' in dr.kicad_refused[0]['reason']
              and dr.kicad_refused[0].get('kicad_version') == '10.9.9', str(dr.kicad_refused))
        refused_state = {p: read_state(rules_of(p)) for p in (b_missed, b_known)}
        sizes = rules_of(b_known).resolve(
            'track_width', rules_of(b_known).item_for_net(1, 'track', 'F.Cu'), None, 'F.Cu').min

    with kicad_answer({'loaded': True, 'kicad_version': '10.9.9'}):
        dr = rules_of(b_known)
        check('KiCad loads a file the model refuses: no finding (KiCad decides)',
              dr.kicad_refused == [] and model_of(b_known), str(dr.kicad_refused))
        loaded_state = {p: read_state(rules_of(p)) for p in (b_missed, b_known)}
        sizes_loaded = rules_of(b_known).resolve(
            'track_width', rules_of(b_known).item_for_net(1, 'track', 'F.Cu'), None, 'F.Cu').min

    with kicad_answer({'loaded': None, 'why': 'kicad-cli not found'}):
        none_state = {p: read_state(rules_of(p)) for p in (b_missed, b_known)}
    check('disclosure only: the rules read are the same whatever KiCad answers '
          '(the whole table, the onLayer rule a refused file holds included)',
          refused_state == loaded_state == none_state and sizes == sizes_loaded == 0.7,
          f'{sizes} {sizes_loaded}')

    import dru_load
    top = stage(work, 'p1_top', TOP_RULE, rename_top=True)
    layers = dru_load.stub_layers(top)
    check('the stub carries the board\'s layer table, user names included',
          (0, 'F.Cu', 'signal', 'Top') in layers and (2, 'B.Cu', 'signal', None) in layers
          and (5, 'F.SilkS', 'user', 'F.Silkscreen') in layers, str(layers[:6]))
    text = dru_load.stub_board(layers)
    check('... written in KiCad 9/10\'s format with the probe\'s one footprint',
          '(0 "F.Cu" signal "Top")' in text and f'"{dru_load.STUB_REF}"' in text
          and text.count('(footprint ') == 1, text[:300])
    four = dru_load.stub_layers(None, ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu'])
    check('no board file: the copper layers, renumbered',
          [(o, n) for o, n, _t, _u in four][:4] == [(0, 'F.Cu'), (4, 'In1.Cu'),
                                                    (6, 'In2.Cu'), (2, 'B.Cu')], str(four))
    name, rule = dru_load.probe_rule('X1', [dru_load.LOAD_PROBE])
    check('the probe takes a name the file does not declare',
          name == dru_load.LOAD_PROBE + ' 2' and f'(rule "{name}" (severity warning)' in rule
          and "A.Reference != 'X1'" in rule, rule)
    check('the cache lives outside the repository',
          not os.path.abspath(dru_load.cache_dir()).startswith(REPO + os.sep)
          and dru_load.cache_dir().startswith(_CACHE), dru_load.cache_dir())


# ---------------------------------------------------------------- part 2

def part2(work):
    print('2. kicad-cli asked on a stub board')
    import dru_load
    jobs = ([('dropped', w, t, False) for w, t in DROPPED]
            + [('model', b, '(version 1)\n' + b + '\n' + _OK, False) for b, _w in FILES_REFUSED]
            + [('model', c, f'(version 1)\n(rule "c" (condition "{c}") '
                            f'(constraint clearance (min 0.1mm)))\n' + _OK, False)
               for c in REFUSED[:8]]
            + [('loads', w, t, False) for w, t in LOADS]
            + [('loads', 'a layer by its user name', TOP_RULE, True),
               ('dropped', 'a user name the board does not give', TOP_RULE, False)])

    def run(job):
        i, (kind, what, text, top) = job
        path = stage(work, f'p2_{i}', text, rename_top=top)
        dr = rules_of(path)
        return kind, what, model_of(path), dr.kicad_refused, dr.report_lines()

    with concurrent.futures.ThreadPoolExecutor(WORKERS) as pool:
        results = list(pool.map(run, enumerate(jobs)))
    misses = 0
    for kind, what, model, got, lines in results:
        if kind == 'loads':
            check(f'loads: {what}: no finding', got == [] and not any(
                HEAD in ln for ln in lines), str(got)[:300])
            continue
        said = bool(got) and all(r.get('kicad_version') for r in got) \
            and all('KiCad refuses' in r['reason'] for r in got) \
            and lines and lines[0].startswith(HEAD + ' (kicad-cli ')
        if kind == 'model':
            check(f'the model\'s form {what!r}: KiCad drops it, and the model says why',
                  said and [r['reason'] for r in got] == [r['reason'] for r in model],
                  str(got)[:300])
        else:
            misses += not model and what in dict(DROPPED)
            check(f'dropped: {what}: said not to be loaded', said, str(got)[:300])
    print(f'     ({misses} of the {len(DROPPED)} measured forms are ones the model '
          f'names nothing in)')

    # the cache: a second ask starts no kicad-cli, in this process or the next
    # (a text no row above asked about)
    text = DROPPED[0][1].replace('"court gap"', '"court gap, cached"')
    board = stage(work, 'p2_cache', text)
    first = dru_load.kicad_loads_dru(text, board)
    check('a first ask runs kicad-cli', first.get('loaded') is False
          and first.get('cached') is False, str(first))
    real_run = dru_load.subprocess.run

    def no_run(*a, **k):
        raise AssertionError('kicad-cli started on a cached answer')
    dru_load.subprocess.run = no_run
    try:
        again = dru_load.kicad_loads_dru(text, board)
        dru_load._MEMO.clear()
        disk = dru_load.kicad_loads_dru(text, board)
    finally:
        dru_load.subprocess.run = real_run
    check('the cache hits on a second call (in process, then from disk), with no '
          'kicad-cli started, not even for its version',
          again.get('cached') is True and disk.get('cached') is True
          and again['loaded'] is disk['loaded'] is False, f'{again} {disk}')
    files = os.listdir(dru_load.cache_dir())
    check('... one answer file per question, one for the kicad-cli version',
          sum(f.startswith('cli-') for f in files) == 1 and not any(
              f.startswith('.tmp-') for f in files), str(files)[:300])

    # check_drc says it, and writes it to --json
    out = board + '.json'
    r = subprocess.run(PY + [os.path.join(REPO, 'py_router', 'check_drc.py'), board,
                             '--json', out], cwd=REPO, capture_output=True, text=True,
                       encoding='utf-8', timeout=300)
    check('check_drc ran', 'Traceback' not in r.stdout + r.stderr, r.stderr[-600:])
    with open(run_utils.evidence(out), encoding='utf-8') as fh:
        doc = json.load(fh)
    check('check_drc says KiCad does not load the file, a form the model misses',
          HEAD + ' (kicad-cli ' in r.stdout and doc.get('kicad_refuses_file')
          and doc['kicad_refuses_file'][0].get('kicad_version'), r.stdout[-600:])

    # check_complete: a board with no footprint the probe can name
    import check_complete
    bare_dir = os.path.join(work, 'bare')
    os.makedirs(bare_dir)
    bare = os.path.join(bare_dir, 'b.kicad_pcb')
    with open(bare, 'w', encoding='utf-8') as fh:
        fh.write('(kicad_pcb (version 20241229) (generator "pcbnew")\n'
                 '  (layers (0 "F.Cu" signal) (2 "B.Cu" signal)))\n')
    for what, text, want in (('drops', DROPPED_1840[0][1], 'not'), ('loads', LOADS[0][1], 'ok')):
        with open(os.path.join(bare_dir, 'b.kicad_dru'), 'w', encoding='utf-8') as fh:
            fh.write(text)
        g = check_complete.kicad_rule_grade(run_utils.evidence(bare))
        if want == 'not':
            check('check_complete, no footprint to probe with, a file KiCad drops: '
                  'not loaded, asked on the stub', 'KiCad did not load' in g.get('error', '')
                  and 'stub board' in g['error'], str(g)[:300])
        else:
            check('check_complete, no footprint to probe with, a file KiCad loads: graded',
                  g.get('errors_by_rule') == {} and g.get('kicad_version'), str(g)[:300])


_HIDE = '''
import sys
sys.path.insert(0, {py!r})
import dru_load
dru_load._find_cli = lambda: None
'''


def routed(work, name, dru, hide):
    d = os.path.join(work, name)
    os.makedirs(d)
    board = os.path.join(d, 'b.kicad_pcb')
    shutil.copy(run_utils.evidence(ROUTE_FIXTURE), board)
    if dru is not None:
        with open(os.path.join(d, 'b.kicad_dru'), 'w', encoding='utf-8') as fh:
            fh.write(dru)
    env = dict(os.environ)
    if hide:
        inj = os.path.join(d, 'inj')
        os.makedirs(inj)
        with open(os.path.join(inj, 'sitecustomize.py'), 'w', encoding='utf-8') as fh:
            fh.write(_HIDE.format(py=os.path.join(REPO, 'py_router')))
        env['PYTHONPATH'] = inj + os.pathsep + env.get('PYTHONPATH', '')
    out = os.path.join(d, 'out.kicad_pcb')
    r = run_utils.check(PY + [os.path.join(REPO, 'py_router', 'route.py'), board, out],
                        accept=True, env=env, timeout=600)
    pcb = parse_kicad_pcb(run_utils.evidence(out))
    names = {i: n.name for i, n in pcb.nets.items()}
    copper = sorted([('seg', names.get(s.net_id), s.layer, round(s.width, 4))
                     + tuple(sorted([(round(s.start_x, 4), round(s.start_y, 4)),
                                     (round(s.end_x, 4), round(s.end_y, 4))]))
                     for s in pcb.segments]
                    + [('via', names.get(v.net_id), round(v.x, 4), round(v.y, 4),
                        round(v.size, 4), round(v.drill, 4)) for v in pcb.vias])
    js = [ln for ln in r.stdout.splitlines() if ln.startswith('JSON_SUMMARY:')]
    block = json.loads(js[-1].split(':', 1)[1]).get('design_rules', {}) if js else None
    rule_lines = [ln.strip() for ln in r.stdout.splitlines() if '.kicad_dru' in ln]
    return copper, block, rule_lines


def part3(work):
    print('3. a routing step reads the same rules with and without kicad-cli')
    # a layer clearance rule the router reads (#498), in a file KiCad drops
    # for a form the model does not name
    dru = ('(version 1)\n(rule "wide gap" (layer "F.Cu") (constraint clearance (min 0.6mm)))\n'
           '(rule "w" (layer "F.Cu") (layer "B.Cu") (constraint track_width (min 0.1mm)))\n')
    with_cli, b_with, l_with = routed(work, 'r_cli', dru, hide=False)
    without, b_without, l_without = routed(work, 'r_nocli', dru, hide=True)
    plain, _b, _l = routed(work, 'r_plain', None, hide=False)
    check('the rule is read: the copper differs from the board routed with no rules file',
          with_cli != plain, f'{len(with_cli)} items both')
    check('the same copper with and without kicad-cli', with_cli == without and with_cli,
          f'{len(with_cli)} vs {len(without)} items')
    check('with kicad-cli the step says KiCad does not load the file, in the log and '
          'JSON_SUMMARY', any(ln.startswith(HEAD + ' (kicad-cli ') for ln in l_with)
          and (b_with or {}).get('kicad_refuses_file'), f'{l_with} {b_with}')
    check('without it, the model\'s answer (it names nothing): no finding',
          not any(HEAD in ln for ln in l_without)
          and not (b_without or {}).get('kicad_refuses_file'), f'{l_without}')
    check('... and every other rule line is the same',
          [ln for ln in l_with if HEAD not in ln] == [ln for ln in l_without if HEAD not in ln],
          f'{l_with} {l_without}')


def main():
    work = tempfile.mkdtemp(prefix='krt1852_')
    skipped = False
    try:
        part1(work)
        with contextlib.redirect_stdout(io.StringIO()):
            from kicad_oracle import find_kicad_cli
            cli = find_kicad_cli(warn=False)
        if cli:
            part2(work)
            part3(work)
        else:
            print('2. SKIPPED: no kicad-cli (set KICAD_CLI)')
            skipped = True
    except AssertionError as exc:
        check('a run_utils assertion held', False, str(exc)[:1500])
    finally:
        shutil.rmtree(work, ignore_errors=True)
        shutil.rmtree(_CACHE, ignore_errors=True)
    if failures:
        print(f'FAIL: {len(failures)}: ' + '; '.join(failures))
        return 1
    if skipped:
        print('PART 1 PASS')
        print('SKIP: part 2 needs kicad-cli (none found; set KICAD_CLI)')
        return 77
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
