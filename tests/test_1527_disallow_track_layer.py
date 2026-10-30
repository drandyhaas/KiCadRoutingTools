"""#1527: a class-scoped layer `disallow track` rule is honoured, graded and named.

    (rule "hs_layers" (constraint disallow track)
      (condition "A.NetClass == 'HS' && A.Type == 'Track' && A.Layer != 'F.Cu'"))

keeps the HS class off every layer but F.Cu. Its condition parses, so the
rule passed as "consumed": route.py laid HS tracks on B.Cu without naming
it, check_drc and board_score graded nothing, and KiCad reported one
items_not_allowed per track. Now:

1. the parse no longer passes it as consumed: `unsupported()` names it until
   a step honours it, and a `via` token stays named after (no step keeps a
   net's vias off);
2. route.py honours the track ban (`net_keepouts.derive_layer_bans` ->
   forbidden layers per bound net): no HS track on B.Cu, the console says
   so, and the stub-switch validators refuse B.Cu for an HS net;
3. check_drc grades it per track and via as KiCad does (measured on
   kicad-cli 10.0.6: a track ban counts each HS track on B.Cu; a via's
   A.Layer is undefined, so `A.Layer != 'F.Cu'` flags no via while a
   `(layer B.Cu)` clause flags every HS through via), board_score counts the
   rows in `drc`, and a token nothing grades is `NOT GRADED (disallow)`,
   listed in board_score's `drc.ungraded_rules`.

    python3 tests/test_1527_disallow_track_layer.py
"""
import fnmatch
import json
import os
import shutil
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
HS_LAYERS = ('(rule "hs_layers" (constraint disallow track) (condition '
             '"A.NetClass == \'HS\' && A.Type == \'Track\' && A.Layer != \'F.Cu\'"))')
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def write_project(board):
    """Default 0.2 mm and a class HS that copies it (so the class itself
    changes nothing) holding /CLK* and /DATA*."""
    def cls(name, prio):
        return {'name': name, 'clearance': 0.2, 'track_width': 0.2, 'via_diameter': 0.6,
                'via_drill': 0.3, 'diff_pair_gap': 0.2, 'diff_pair_width': 0.2,
                'priority': prio}
    stem = os.path.splitext(board)[0]
    pro = {'board': {'design_settings': {'rules': {'min_clearance': 0.0}}},
           'net_settings': {'meta': {'version': 4},
                            'classes': [cls('Default', 2147483647), cls('HS', 0)],
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


def hs_copper(board):
    """(HS tracks on B.Cu, HS vias) of a routed board."""
    from kicad_parser import parse_kicad_pcb
    pcb = parse_kicad_pcb(board)
    hs = {i for i, n in pcb.nets.items()
          if n.name and any(fnmatch.fnmatchcase(n.name, p) for p in PATS)}
    return (sum(1 for s in pcb.segments if s.net_id in hs and s.layer == 'B.Cu'),
            sum(1 for v in pcb.vias if v.net_id in hs))


def score(board):
    """board_score's document (it exits 4 whenever something blocks or is
    ungraded, so the document, not the exit code, is the evidence)."""
    import subprocess
    r = subprocess.run(PY + [os.path.join(REPO, 'py_tools', 'board_score.py'), board,
                             '--quiet'], capture_output=True, text=True, cwd=REPO)
    if 'Traceback' in r.stdout + r.stderr:
        raise AssertionError(f'BROKEN TEST: board_score crashed\n{r.stderr[-800:]}')
    line = next((x for x in r.stdout.splitlines() if x.startswith('SCORE_JSON=')), '')
    if not line:
        raise AssertionError(f'BROKEN TEST: board_score printed no SCORE_JSON\n'
                             f'{(r.stdout + r.stderr)[-800:]}')
    return json.loads(line[len('SCORE_JSON='):])


def parse_contract():
    print('1. the parse names a scope-evaluable disallow rule until a step honours it')
    from design_rules import parse_dru, DesignRules
    rules, _ = parse_dru(HS_LAYERS + '\n(rule "hs_vias" (constraint disallow track via) '
                         '(condition "A.NetClass == \'HS\'"))\n'
                         '(rule "no_blind" (constraint disallow blind_via micro_via))')
    by = {r.name: r for r in rules}
    check('the track ban is evaluated (not unsupported) ...',
          by['hs_layers'].unsupported is None and by['hs_layers'].condition is not None)
    check('... and NOT passed as consumed: router_unhonoured names it',
          bool(getattr(by['hs_layers'], 'router_unhonoured', None)), f'{by["hs_layers"]}')
    check('a ban of vias no router lays asks nothing of the router',
          getattr(by['no_blind'], 'router_unhonoured', None) is None)
    dr = DesignRules()
    dr.rules = rules
    names = [n for n, _w in dr.unsupported()]
    check("unsupported() lists 'hs_layers' and 'hs_vias', not 'no_blind'",
          'hs_layers' in names and 'hs_vias' in names and 'no_blind' not in names,
          f'{names}')


def stub_switch(ruled):
    from kicad_parser import parse_kicad_pcb
    from routing_config import GridRouteConfig
    from design_rules import install_design_rules
    from net_keepouts import attach
    import net_keepouts
    from stub_layer_switching import forbidden_destination
    import inspect
    if not hasattr(net_keepouts, 'banned_layers') or \
            'net_ids' not in inspect.signature(forbidden_destination).parameters:
        check('the stub-switch validator reads the track bans', False,
              'forbidden_destination takes no nets')
        return
    pcb = parse_kicad_pcb(ruled)
    cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'])
    rules = install_design_rules(cfg, ruled, pcb, net_keepouts=True)
    attach(pcb, rules.net_keepouts)
    ids = {n.name: i for i, n in pcb.nets.items() if n.name}
    got = net_keepouts.banned_layers(pcb, (ids['/CLK'],))
    check('the HS net /CLK is banned from B.Cu only', got == {'B.Cu'}, f'{got}')
    check('B.Cu is a forbidden destination for /CLK, not for /GND',
          forbidden_destination(cfg, 'B.Cu', pcb, (ids['/CLK'],))
          and not forbidden_destination(cfg, 'B.Cu', pcb, (ids['/GND'],)))


def main():
    if not os.path.isfile(BOARD):
        print(f'SKIP: {BOARD} missing')
        return 77
    td = tempfile.mkdtemp(prefix='krt1527_')
    try:
        parse_contract()

        print('2. route.py keeps the HS tracks off B.Cu')
        free = copy_board(BOARD, os.path.join(td, 'free.kicad_pcb'))
        write_project(free)
        free_out = os.path.join(td, 'free_r.kicad_pcb')
        run_utils.check(PY + [os.path.join(REPO, 'py_router', 'route.py'), free, free_out],
                        accept=True)
        n_free, _ = hs_copper(run_utils.evidence(free_out, 'routed board'))
        check('non-vacuity: without the rule HS routes on B.Cu', n_free > 0, f'{n_free}')

        ruled = copy_board(BOARD, os.path.join(td, 'ruled.kicad_pcb'))
        write_project(ruled)
        write_dru(ruled, HS_LAYERS)
        ruled_out = os.path.join(td, 'ruled_r.kicad_pcb')
        r = run_utils.check(PY + [os.path.join(REPO, 'py_router', 'route.py'), ruled,
                                  ruled_out], accept=True)
        n_ruled, _ = hs_copper(run_utils.evidence(ruled_out, 'routed board'))
        check('with the rule no HS track lies on B.Cu', n_ruled == 0, f'{n_ruled}')
        check('the console says the rule is honoured',
              "rule 'hs_layers': honoured as forbidden layers" in r.stdout, r.stdout[-400:])
        check('...and lists it nowhere as NOT honoured',
              "'hs_layers': NOT honoured" not in r.stdout
              and '"unsupported_rules": ["hs_layers"' not in r.stdout)

        print('3. a stub switch refuses a banned layer for a bound net')
        stub_switch(ruled)

        print('4. check_drc and board_score grade it as KiCad does')
        graded = copy_board(free_out, os.path.join(td, 'graded.kicad_pcb'))
        write_project(graded)
        write_dru(graded, HS_LAYERS)
        r = run_utils.check(PY + [os.path.join(REPO, 'py_router', 'check_drc.py'), graded],
                            refuse='ITEMS-NOT-ALLOWED violations', code=1)
        check(f'one items-not-allowed per HS track on B.Cu ({n_free})',
              f'ITEMS-NOT-ALLOWED violations ({n_free})' in r.stdout
              and "rule 'hs_layers'" in r.stdout, r.stdout[-600:])
        s = score(graded)
        drc = (s.get('components') or {}).get('drc') or {}
        check('board_score counts them in drc (blocking)',
              (drc.get('by_type') or {}).get('items-not-allowed') == n_free
              and (s.get('blocking') or 0) >= n_free, json.dumps(drc)[:300])

        _n_tracks, n_vias = hs_copper(graded)
        vias = copy_board(free_out, os.path.join(td, 'vias.kicad_pcb'))
        write_project(vias)
        write_dru(vias,
                  '(rule "via_layer_prop" (constraint disallow via) '
                  '(condition "A.NetClass == \'HS\' && A.Layer != \'F.Cu\'"))',
                  '(rule "no_pads" (constraint disallow pad) '
                  '(condition "A.NetClass == \'HS\'"))')
        r = run_utils.check(PY + [os.path.join(REPO, 'py_router', 'check_drc.py'), vias],
                            accept=True, allow=())
        check("a via's A.Layer is undefined: `A.Layer != 'F.Cu'` flags no via",
              '0 track/via item(s) not allowed' in r.stdout, r.stdout[-400:])
        check("a token nothing grades is NOT GRADED (disallow)",
              "NOT GRADED (disallow): rule 'no_pads'" in r.stdout, r.stdout[-400:])
        s = score(vias)
        drc = (s.get('components') or {}).get('drc') or {}
        check("board_score lists it ungraded", drc.get('ungraded_rules') == ['no_pads']
              and 'drc.dru_rules' in (s.get('ungraded') or []), json.dumps(drc)[:300])

        lay = copy_board(free_out, os.path.join(td, 'lay.kicad_pcb'))
        write_project(lay)
        write_dru(lay, '(rule "via_layer_clause" (layer "B.Cu") (constraint disallow via) '
                       '(condition "A.NetClass == \'HS\'"))')
        if n_vias:
            r = run_utils.check(PY + [os.path.join(REPO, 'py_router', 'check_drc.py'), lay],
                                refuse='ITEMS-NOT-ALLOWED violations', code=1)
            check(f'a (layer B.Cu) clause flags every HS through via ({n_vias})',
                  f'ITEMS-NOT-ALLOWED violations ({n_vias})' in r.stdout, r.stdout[-400:])
        else:
            check('non-vacuity: the free route has HS vias', False, '0 vias')
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if failures:
        print(f'FAIL: {len(failures)}: ' + '; '.join(failures))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
