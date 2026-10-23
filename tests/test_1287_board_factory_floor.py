#!/usr/bin/env python3
"""--escalation board with the DRC-settings writeback OFF floors a rule the
project leaves out at KiCad's factory value (#1287).

``board_floors_from_rules`` reads an absent or 0 ``rules.min_*`` as unset, and
under ``board`` an unset key falls through to the fab tier. That is right when
the writeback runs: it sets the rule to the routed floor. Without it
(--no-fix-drc-settings, the GUI's "Fix DRC settings after routing" unticked)
the input project ships unchanged, and KiCad grades an absent rule at the
factory value it loads for it -- min_track_width 0.2 on KiCad 10. Reported:
a 4-layer board rescued a net at the 0.0889 fab floor and kicad-cli flagged 14
track_width violations, "min width 0.2000 mm; actual 0.0889 mm".

The board is #1262's (tests/test_1262_board_class_floor.py builds it): FB's
only way out is a 0.64 mm strip along U2's pad tips. Here its project leaves
min_track_width out and sets the clearance to 0.235 at both the class and
min_clearance, so clearance cannot descend. FB's 0.2 track does not fit the
strip (0.67 mm); the rescue's 0.127 fab-floor track does (0.597 mm).

Checks:
  A. graded_board_floors: a declared rule at its value, a declared 0 as no
     floor, an absent or non-numeric rule at KICAD_FACTORY_RULES
  B. board_policy_floors: writeback off floors an unset rule at the factory
     value, a request below a floor is that floor, a request of 0 moves
     nothing; writeback on and the other policies are unchanged
  C. both fronts: set_policy_from_args (a project leaving rules out, and no
     project at all) and set_fab_tier_from_config (live floors) agree
  D. KICAD_FACTORY_RULES against pcbnew's own DEFAULT_* constants, under
     every KiCad python found (skipped, and said, when there is none)
  E. END TO END, writeback off: no track descent, FB's copper is never under
     0.2 mm, and the run says why
  F. END TO END, writeback on (control): the 0.127 rescue still happens and
     the output project's min_track_width is lowered to it

    python3 tests/test_1287_board_factory_floor.py
"""
import contextlib
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile
from types import SimpleNamespace as NS

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

import fab_tiers                                       # noqa: E402
import test_1262_board_class_floor as board1262        # noqa: E402
from run_utils import evidence                         # noqa: E402

FAILS = []
CLR = 0.235
FACTORY_TRACK = 0.2


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def quiet(fn, *a, **kw):
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        return fn(*a, **kw), buf.getvalue()


def project():
    p = board1262.project()
    rules = p['board']['design_settings']['rules']
    rules.pop('min_track_width')
    rules['min_clearance'] = CLR
    cls = p['net_settings']['classes'][0]
    cls['clearance'] = CLR
    cls['track_width'] = FACTORY_TRACK
    return p


def stage(d, name, proj=True):
    pcb = os.path.join(d, name + '.kicad_pcb')
    with open(pcb, 'w', encoding='utf-8') as f:
        f.write(board1262.board_text())
    if proj:
        with open(os.path.splitext(pcb)[0] + '.kicad_pro', 'w', encoding='utf-8') as f:
            json.dump(project(), f, indent=2)
    return pcb


# ----------------------------------------------------------------- A. graded
def t_graded():
    print('A. graded_board_floors')
    F = fab_tiers.KICAD_FACTORY_RULES
    g = fab_tiers.graded_board_floors({})
    check('no rules at all: every floor is the factory value',
          g == {fk: F[rk] for rk, fk in fab_tiers.BOARD_RULE_TO_FLOOR_KEY.items()
                if F[rk] > 0}, str(g))
    check('...track 0.2, via 0.5 / drill 0.3, hole-to-hole 0.25, edge 0.5',
          (g.get('track_width'), g.get('via_diameter'), g.get('via_drill'),
           g.get('hole_to_hole'), g.get('board_edge'))
          == (0.2, 0.5, 0.3, 0.25, 0.5), str(g))
    check('...and no clearance floor (min_clearance factory value is 0)',
          'clearance' not in g, str(g))
    g = fab_tiers.graded_board_floors({'min_track_width': 0.1,
                                       'min_via_diameter': 0,
                                       'min_through_hole_diameter': 'x'})
    check('a declared rule is graded at its value', g.get('track_width') == 0.1, str(g))
    check('a declared 0 is KiCad\'s "no check": no floor',
          'via_diameter' not in g, str(g))
    check('a non-numeric value is not a declaration: the factory value',
          g.get('via_drill') == 0.3, str(g))
    _, said = quiet(fab_tiers.graded_board_floors, {'min_via_diameter': 0.4},
                    announce=True)
    check('announce names the unset rules', '#1287' in said
          and 'min_track_width at 0.2' in said and 'min_via_diameter' not in said,
          said.strip())
    rules = project()['board']['design_settings']['rules']
    check('control: the writeback-on reading leaves the absent rule unset',
          'track_width' not in fab_tiers.board_floors_from_rules(rules, CLR))


# --------------------------------------------------------- B. policy floors
def t_policy():
    print('B. board_policy_floors')
    rules = project()['board']['design_settings']['rules']
    declared = fab_tiers.board_floors_from_rules(rules, CLR)
    graded = fab_tiers.graded_board_floors(rules)
    req = {'clearance': CLR, 'track_width': 0.2, 'via_size': 0.6, 'via_drill': 0.3,
           'hole_to_hole_clearance': 0.25, 'board_edge_clearance': 0.0}
    f, _ = quiet(fab_tiers.board_policy_floors, 'board', declared, req, CLR,
                 writeback=False, graded_floors=graded)
    check('writeback off: the unset track rule floors at 0.2',
          f.get('track_width') == FACTORY_TRACK, str(f))
    check('...declared rules keep their value',
          f.get('via_diameter') == 0.5 and f.get('board_edge') == 0.3, str(f))
    check('...and clearance is still the graded class', f.get('clearance') == CLR, str(f))
    f, said = quiet(fab_tiers.board_policy_floors, 'board', declared,
                    dict(req, track_width=0.15, via_size=0.45), CLR,
                    writeback=False, graded_floors=graded)
    check('writeback off, a request below a floor: the request is the floor '
          '(no descent), never dropped to the fab floor',
          f.get('track_width') == 0.15 and f.get('via_diameter') == 0.45, str(f))
    check('...and it says so', said.count('#1287') == 2, said.strip())
    f, _ = quiet(fab_tiers.board_policy_floors, 'board', {'board_edge': 0.5},
                 {'board_edge_clearance': 0.0}, None, writeback=False)
    check('a request of 0 (the edge "use the copper clearance" sentinel) '
          'moves nothing', f.get('board_edge') == 0.5, str(f))
    f, said = quiet(fab_tiers.board_policy_floors, 'board', declared,
                    dict(req, via_size=0.45), CLR, writeback=True,
                    graded_floors=graded)
    check('control: writeback on is unchanged -- no track floor, a request '
          'below a declared floor marks it stale',
          'track_width' not in f and 'via_diameter' not in f and 'stale' in said,
          str(f))
    for pol in ('fab', 'off'):
        f, _ = quiet(fab_tiers.board_policy_floors, pol, declared, req, CLR,
                     writeback=False, graded_floors=graded)
        check(f'{pol}: the floors pass through', f == declared, str(f))


# ----------------------------------------------------------- C. both fronts
def t_fronts(tmp):
    print('C. both fronts')
    pcb = stage(tmp, 'fronts')
    bare = stage(tmp, 'bare', proj=False)
    prev = fab_tiers.get_escalation_policy()
    args = dict(escalation='board', clearance=CLR, track_width=0.2,
                via_size=0.6, via_drill=0.3)
    try:
        f, said = quiet(fab_tiers.set_policy_from_args,
                        NS(no_fix_drc_settings=True, **args), pcb)
        check('CLI, --no-fix-drc-settings: the track floor is the factory 0.2',
              f.get('track_width') == FACTORY_TRACK
              and fab_tiers.get_escalation_policy()[1].get('track_width')
              == FACTORY_TRACK, str(f))
        check('...the run says so', '#1287' in said, said.strip())
        f, _ = quiet(fab_tiers.set_policy_from_args,
                     NS(no_fix_drc_settings=False, **args), pcb)
        check('CLI, writeback on: no track floor', 'track_width' not in f, str(f))
        f, _ = quiet(fab_tiers.set_policy_from_args, NS(**args), pcb)
        check('CLI, a caller without the flag keeps the writeback-on floors',
              'track_width' not in f, str(f))
        f, _ = quiet(fab_tiers.set_policy_from_args,
                     NS(no_fix_drc_settings=True, **args), bare)
        check('CLI, no project at all: every rule at its factory value',
              (f.get('track_width'), f.get('via_diameter'), f.get('board_edge'))
              == (0.2, 0.5, 0.5), str(f))
        # The GUI reads the live board's design settings, which hold the
        # factory value for a rule the project leaves out.
        live = {'clearance': CLR, 'track_width': FACTORY_TRACK,
                'via_diameter': 0.5, 'via_drill': 0.3}
        gui = {'escalation': 'board', 'clearance': CLR, 'track_width': 0.15,
               'via_size': 0.6, 'via_drill': 0.3, 'board_floors': live,
               'default_class_clearance': CLR}
        quiet(fab_tiers.set_fab_tier_from_config, dict(gui, fix_drc_settings=False))
        g = fab_tiers.get_escalation_policy()[1]
        f, _ = quiet(fab_tiers.set_policy_from_args,
                     NS(no_fix_drc_settings=True, **dict(args, track_width=0.15)), pcb)
        check('GUI, "Fix DRC settings" unticked: the same track floor as the CLI',
              g.get('track_width') == f.get('track_width') == 0.15, f'{g} vs {f}')
    finally:
        fab_tiers.set_escalation_policy(*prev)


# ------------------------------------------------------- D. against pcbnew
_PROBE = r"""
import json, pcbnew
names = {'min_clearance': 'DEFAULT_MINCLEARANCE',
         'min_track_width': 'DEFAULT_TRACKMINWIDTH',
         'min_via_diameter': 'DEFAULT_VIASMINSIZE',
         'min_through_hole_diameter': 'DEFAULT_MINTHROUGHDRILL',
         'min_hole_to_hole': 'DEFAULT_HOLETOHOLEMIN',
         'min_copper_edge_clearance': 'DEFAULT_COPPEREDGECLEARANCE'}
out = {k: float(getattr(pcbnew, n)) for k, n in names.items()}
out['min_via_annular_width'] = round((out['min_via_diameter']
                                      - out['min_through_hole_diameter']) / 2, 9)
print('PROBE ' + json.dumps({'version': pcbnew.GetBuildVersion(), 'rules': out}))
"""


def t_pcbnew():
    print('D. KICAD_FACTORY_RULES against pcbnew')
    try:
        from kicad_exact_fill import kicad_python_candidates
        cands = [c for c in dict.fromkeys(kicad_python_candidates())
                 if c and os.path.isfile(c)]
    except Exception as e:                                     # noqa: BLE001
        cands = []
        print(f'  (no candidate list: {e!r})')
    seen = 0
    for cand in cands:
        try:
            r = subprocess.run([cand, '-c', _PROBE], capture_output=True,
                               text=True, timeout=120)
        except (OSError, subprocess.TimeoutExpired):
            continue
        line = next((l for l in r.stdout.splitlines() if l.startswith('PROBE ')), '')
        if not line:
            continue
        got = json.loads(line[len('PROBE '):])
        seen += 1
        ver, rules = got['version'], got['rules']
        table = fab_tiers.KICAD_FACTORY_RULES
        check(f'pcbnew {ver}: the table covers every rule it reads',
              set(rules) == set(table), str(sorted(set(rules) ^ set(table))))
        check(f'pcbnew {ver}: no table value is looser than KiCad grades',
              all(table[k] >= rules[k] - 1e-9 for k in rules if k in table),
              str({k: (table.get(k), v) for k, v in rules.items()}))
        if ver.startswith('10.'):
            check(f'pcbnew {ver}: the table IS KiCad 10\'s factory values',
                  all(abs(table[k] - rules[k]) < 1e-9 for k in rules if k in table),
                  str({k: (table.get(k), v) for k, v in rules.items()}))
    if not seen:
        print('  SKIP: no KiCad python found; the table is unchecked here')


# ------------------------------------------------------------- E/F. routing
def fb_widths(board):
    """The widths of FB's segments on `board`, read off the written file."""
    evidence(board, 'routed board')
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(board)
    fb = next(n for n, net in pcb.nets.items() if net.name == '/FB')
    return sorted({round(s.width, 4) for s in pcb.segments if s.net_id == fb})


def track_rows(summ):
    return [r for r in (summ.get('design_rules') or {}).get('narrowed') or []
            if r.get('kind') == 'track_width']


def t_route(tmp):
    print('E. writeback off, end to end')
    inp = stage(tmp, 'in')
    out = os.path.join(tmp, 'off.kicad_pcb')
    log, summ = board1262.route(inp, out, ['--no-fix-drc-settings',
                                           '--track-width', str(FACTORY_TRACK)])
    if not summ:
        raise AssertionError('BROKEN TEST: route.py printed no JSON_SUMMARY\n'
                             + log[-1500:])
    rows = track_rows(summ)
    check('no track descent recorded', not rows, str(rows))
    w = fb_widths(out)
    check('no FB copper under the factory 0.2 mm',
          all(x >= FACTORY_TRACK - 1e-6 for x in w), str(w))
    check('the board floors record the track floor',
          (summ.get('design_rules') or {}).get('board_floors', {})
          .get('track_width') == FACTORY_TRACK,
          str((summ.get('design_rules') or {}).get('board_floors')))
    check('the run says why', '#1287' in log)

    print('F. writeback on (control)')
    out2 = os.path.join(tmp, 'on.kicad_pcb')
    log2, summ2 = board1262.route(inp, out2, ['--track-width', str(FACTORY_TRACK)])
    if not summ2:
        raise AssertionError('BROKEN TEST: route.py printed no JSON_SUMMARY\n'
                             + log2[-1500:])
    rows2 = track_rows(summ2)
    check('the rescue still narrows FB to the 0.127 fab floor, recorded',
          any(r['delivered'] == 0.127 and r['site'] == 'net rescue'
              and r.get('net_name') == '/FB' for r in rows2), str(rows2))
    check('...the copper is there', 0.127 in fb_widths(out2), str(fb_widths(out2)))
    with open(os.path.splitext(out2)[0] + '.kicad_pro', encoding='utf-8') as f:
        rules2 = json.load(f)['board']['design_settings']['rules']
    check('the output project\'s min_track_width is lowered to it',
          rules2.get('min_track_width') == 0.127, str(rules2.get('min_track_width')))


def main():
    tmp = tempfile.mkdtemp(prefix='t1287_')
    try:
        # Each section on its own, so a missing helper is a FAIL of that
        # section and the end-to-end evidence still runs. A broken TEST
        # (route.py crashing, no summary) still raises.
        for name, fn in (('A', t_graded), ('B', t_policy),
                         ('C', lambda: t_fronts(tmp)), ('D', t_pcbnew)):
            try:
                fn()
            except AttributeError as e:
                check(f'{name}: the rule exists', False, repr(e))
        t_route(tmp)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print()
    if FAILS:
        print(f'{len(FAILS)} FAILURE(S): {", ".join(FAILS)}')
        sys.exit(1)
    print('ALL PASS')


if __name__ == '__main__':
    main()
