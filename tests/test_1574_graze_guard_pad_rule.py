"""#1574: route_diff's connector-graze guard (#165) prices a foreign PAD as
check_drc grades the track against it.

`_connector_grazes_foreign_copper` re-checks a finished pair route against
every foreign pad, promising "the SAME geometry and tolerance as check_drc".
It priced a pad with `config.obstacle_clearance(net)`, the STAMP value, which
asks the class/type-scoped .kicad_dru channel (#1491) about tracks and vias
only -- so a class-to-pad rule (`A.NetClass == 'HS' && B.Type == 'Pad'`)
never reached it. With class HS at 0.5 and that rule at 0.2, the guard
rejected every pair route of lvds_converter_dualclk at gaps the rule allows
(18 rejections, `Diff pairs: 0/2 routed`, both pairs single-ended), and the
tightening direction (class 0.2, a 0.4 pad rule) passed copper check_drc
flags. The guard now prices a pad at `config.pad_pair_clearance(pad, net,
layer, other_type='track')`, the pair's own value; a via-to-pad rule does not
price a track.

    1. the guard against one pad, under each rule, agreeing with check_drc
    2. lvds_converter_dualclk, class HS 0.5 + the 0.2 pad rule: 2/2 coupled

    python3 tests/test_1574_graze_guard_pad_rule.py
"""
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
sys.path.insert(0, os.path.join(TESTS, 'oracle'))
import run_utils  # noqa: E402

from constraint_agreement import write_board  # noqa: E402

RUN_ALL_TIMEOUT = 600
PY = [sys.executable, '-X', 'utf8', '-B']
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


HS_PAD = ('(rule "hs pad" (constraint clearance (min {v}mm)) '
          "(condition \"A.NetClass == 'HS' && B.Type == 'Pad'\"))")
VIA_PAD = ('(rule "via pad" (constraint clearance (min {v}mm)) '
           "(condition \"A.Type == 'Via' && B.Type == 'Pad'\"))")

# write_board's nets: 1 = A (P), 2 = B (N), 3 = C (the foreign pad's, Default 0.2)
P, N, C = 1, 2, 3
PAD_X, PAD_Y, PAD = 20.0, 20.0, 1.0
WIDTH = 0.2

# (label, HS class clearance, .kicad_dru rule, pad-edge to track-edge gap, grazes)
CASES = [
    ('relaxing: class 0.5, pad rule 0.2 -- a 0.3 mm gap is legal',
     0.5, HS_PAD.format(v=0.2), 0.3, False),
    ('relaxing: ...a 0.15 mm gap still grazes the rule',
     0.5, HS_PAD.format(v=0.2), 0.15, True),
    ('tightening: class 0.2, pad rule 0.4 -- a 0.3 mm gap grazes',
     0.2, HS_PAD.format(v=0.4), 0.3, True),
    ('tightening: ...a 0.45 mm gap clears',
     0.2, HS_PAD.format(v=0.4), 0.45, False),
    ('a via-to-pad rule 0.4 does not price a track: a 0.3 mm gap is legal',
     0.2, VIA_PAD.format(v=0.4), 0.3, False),
]


def _board(td, i, hs, dru, gap):
    b = os.path.join(td, f'g{i}.kicad_pcb')
    y = PAD_Y - (PAD / 2 + WIDTH / 2 + gap)
    write_board(b, classes=[{'name': 'HS', 'clearance': hs, 'priority': 0}],
                patterns=[('A', 'HS'), ('B', 'HS')], dru=dru,
                footprints=[dict(ref='U1', x=PAD_X, y=PAD_Y, net_id=C, net_name='C',
                                 size=PAD)],
                segments=[(PAD_X - 2, y, PAD_X + 2, y, WIDTH, 'F.Cu', P)])
    return b


def _guard(b, hs):
    """The guard on the board's own segment, configured as the engine does."""
    from kicad_parser import parse_kicad_pcb
    from routing_config import GridRouteConfig
    from kicad_dru import install_track_clearances
    from diff_pair_routing import _connector_grazes_foreign_copper
    pcb = parse_kicad_pcb(b)
    cfg = GridRouteConfig()
    cfg.clearance = 0.2
    cfg.layers = ['F.Cu', 'B.Cu']
    cfg.set_net_clearances({P: hs, N: hs}, [P, N])
    install_track_clearances(cfg, None, b, pcb, routed_net_ids=[P, N])
    segs = [s for s in pcb.segments if s.net_id == P]
    pcb.segments = []
    return _connector_grazes_foreign_copper(segs, pcb, P, N, cfg)


def _check_drc_flags(b, kind=None):
    """True when check_drc flags the board (only `kind` violations, e.g.
    'PAD-SEGMENT', when given); refuses an output with no verdict at all."""
    r = subprocess.run(PY + [os.path.join('py_router', 'check_drc.py'), b],
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=REPO)
    out = r.stdout + r.stderr
    if 'NO DRC VIOLATIONS FOUND' in out:
        return False
    if 'DRC VIOLATIONS:' in out:
        return f'{kind} violations' in out if kind else True
    raise AssertionError(f'BROKEN TEST: check_drc gave no verdict on {b}\n{out[-1200:]}')


def guard_cases(td):
    print('1. the guard against one foreign pad, and check_drc on the same copper')
    for i, (label, hs, dru, gap, grazes) in enumerate(CASES):
        b = _board(td, i, hs, dru, gap)
        run_utils.evidence(b)
        g = _guard(b, hs)
        check(f'{label}: the guard {"rejects" if grazes else "accepts"}',
              (g is not None and g[0] == 'pad') if grazes else g is None,
              f'got {g and (g[0], round(g[3], 4))}')
        d = _check_drc_flags(b, 'PAD-SEGMENT')
        check(f'...and check_drc {"flags" if grazes else "passes"} it too', d == grazes)


NETS = ['/CLK+', '/CLK-', '/DATA+', '/DATA-']


def _project(stem, hs):
    def cls(n, c, prio):
        return {'name': n, 'clearance': c, 'track_width': 0.15, 'via_diameter': 0.6,
                'via_drill': 0.3, 'diff_pair_gap': 0.15, 'diff_pair_width': 0.15,
                'diff_pair_via_gap': 0.25, 'microvia_diameter': 0.3, 'microvia_drill': 0.1,
                'priority': prio, 'bus_width': 12, 'wire_width': 6, 'line_style': 0,
                'pcb_color': 'rgba(0, 0, 0, 0.000)', 'schematic_color': 'rgba(0, 0, 0, 0.000)'}
    json.dump({'board': {'design_settings': {'rules': {'min_clearance': 0.0,
                                                       'min_track_width': 0.0}}},
               'net_settings': {'meta': {'version': 4},
                                'classes': [cls('Default', 0.15, 2147483647), cls('HS', hs, 0)],
                                'netclass_patterns': [{'netclass': 'HS', 'pattern': p}
                                                      for p in NETS]},
               'meta': {'filename': os.path.basename(stem) + '.kicad_pro', 'version': 3}},
              open(stem + '.kicad_pro', 'w', encoding='utf-8'), indent=2)


def routed(td):
    print('2. lvds_converter_dualclk: class HS 0.5 and a 0.2 class-to-pad rule')
    b = os.path.join(td, 'lvds.kicad_pcb')
    run_utils.check(PY + [os.path.join('py_router', 'copy_board.py'),
                          os.path.join('kicad_files', 'lvds_converter_dualclk.kicad_pcb'), b],
                    accept=True)
    run_utils.evidence(b)
    stem = os.path.splitext(b)[0]
    _project(stem, 0.5)
    open(stem + '.kicad_dru', 'w', encoding='utf-8').write(
        '(version 1)\n' + HS_PAD.format(v=0.2) + '\n')
    out_b = os.path.join(td, 'lvds_o.kicad_pcb')
    r = run_utils.check(PY + [os.path.join('py_router', 'route_diff.py'), b, out_b,
                              '--nets'] + NETS, accept=True)
    line = next((ln for ln in r.stdout.splitlines() if ln.startswith('JSON_SUMMARY:')), None)
    if line is None:
        raise AssertionError(f'BROKEN TEST: route_diff printed no JSON_SUMMARY\n{r.stdout[-1500:]}')
    doc = json.loads(line[len('JSON_SUMMARY:'):])
    check('the rule reached the scoped channel',
          (doc.get('design_rules', {}).get('scoped_clearance') or {}).get('rules') == ['hs pad'],
          json.dumps(doc.get('design_rules', {}).get('scoped_clearance')))
    check('both pairs route coupled (was 0/2, every member single-ended)',
          sorted(doc.get('routed_diff_pairs', [])) == ['/CLK', '/DATA']
          and not (doc.get('single_ended_fallback') or {}).get('attempted')
          and all(p.get('outcome') == 'coupled' for p in doc.get('pair_reports', [])),
          f"routed {doc.get('routed_diff_pairs')}, fallback "
          f"{doc.get('single_ended_fallback')}")
    graze = [ln.strip() for ln in r.stdout.splitlines()
             if 'Connector grazes foreign pad IC4.12' in ln]
    check('the guard no longer rejects the route at IC4.12 (0.364 mm, legal at 0.2)',
          not graze, graze[0] if graze else '')
    run_utils.evidence(out_b)
    check('check_drc grades the routed board clean (it reads the rule)',
          not _check_drc_flags(out_b))
    c = run_utils.check(PY + [os.path.join('py_router', 'check_connected.py'), out_b,
                              '--nets'] + NETS, accept=True)
    check('...and all four nets connect', 'ALL NETS FULLY CONNECTED' in c.stdout,
          c.stdout[-800:])


def main():
    td = tempfile.mkdtemp(prefix='t1574_')
    try:
        guard_cases(td)
        routed(td)
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if failures:
        print(f'\nFAILED {len(failures)}: ' + '; '.join(failures))
        return 1
    print('\nPASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
