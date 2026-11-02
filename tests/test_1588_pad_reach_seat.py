"""#1588: place_simple seated a part on its DRAWN courtyard while check_assembly
gates courtyard PLUS pad copper (`legality.occupancy_shape`), so on a project
with `courtyards_overlap: error` a pad reaching 0.2 mm past its courtyard made
every seed `illegal []` and NOT BUILDABLE; a keep-out read clear with a pad in
it, and fit_template's "off the chip" measured courtyards only.

The placement state (`estimate_courtyards`, what place_simple and fit_template
build) now seats such a part on the box of what check_assembly grades it on
(`body.pad_reach_courtyards`), but only where the pads reach as deep as
check_assembly gates a pair from on that project (`legality.
courtyard_gate_depth_mm`: 0.01 mm at `error`, 0.3 mm otherwise, never at
`ignore`); a part whose pads lie inside its courtyard is untouched. The grade's
keep-out rule and the seat's keep-out test read pad copper beside the
courtyard. Built from the issue's repro: ten 3 x 2 mm parts whose pad 2 reaches
OVER mm past the drawn courtyard.

    python3 tests/test_1588_pad_reach_seat.py
"""
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

FAILS = []


def ok(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}"
          + (f"  ({detail})" if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


def _part(i, over, x, y):
    a, b = f'N{i}', f'N{(i + 1) % 10}'
    return (f'  (footprint "t:OVER" (layer "F.Cu") (at {x} {y})\n'
            f'    (property "Reference" "U{i + 1}" (at 0 0) (layer "F.SilkS") (hide yes))\n'
            '    (fp_rect (start -1.5 -1) (end 1.5 1) (stroke (width 0.05) (type default))'
            ' (layer "F.CrtYd"))\n'
            f'    (pad "1" smd rect (at -0.9 -0.3) (size 0.8 0.6) (layers "F.Cu") (net {i + 2} "{a}"))\n'
            f'    (pad "2" smd rect (at {1.1 + over:.3f} 0) (size 0.8 0.8) (layers "F.Cu")'
            f' (net {(i + 1) % 10 + 2} "{b}"))\n'
            '    (pad "3" smd rect (at -0.9 0.6) (size 0.4 0.4) (layers "F.Cu") (net 1 "GND")))\n')


def board(d, over, severity='error', n=10, poses=None):
    """The repro's board (`n` parts) in `d`, with a project at `severity`
    (None: no project). Returns its path."""
    os.makedirs(d, exist_ok=True)
    poses = poses or {i: (4 + (i % 4) * 5, 4 + (i // 4) * 5) for i in range(n)}
    nets = ['GND'] + [f'N{i}' for i in range(10)]
    path = os.path.join(d, 'b.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write('(kicad_pcb (version 20240108) (generator pcbnew)\n'
                 '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
                 '  (net 0 "")\n'
                 + ''.join(f'  (net {k + 1} "{m}")\n' for k, m in enumerate(nets))
                 + '  (gr_rect (start 0 0) (end 22 18) (stroke (width 0.1) (type default))'
                   ' (layer "Edge.Cuts"))\n'
                 + ''.join(_part(i, over, *poses[i]) for i in range(n)) + ')\n')
    if severity is not None:
        json.dump({'board': {'design_settings': {
                       'rule_severities': {'courtyards_overlap': severity},
                       'rules': {'min_clearance': 0.1}}},
                   'net_settings': {'classes': [{'name': 'Default', 'clearance': 0.1,
                                                 'track_width': 0.15, 'via_diameter': 0.5,
                                                 'via_drill': 0.3}]}},
                  open(os.path.join(d, 'b.kicad_pro'), 'w'))
    return path


def state(path, estimate=True, keepouts=()):
    from kicad_parser import parse_kicad_pcb
    import pose_score
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(path)
        return pose_score.make_state(pcb, path, clearance=0.1, board_edge_clearance=0.3,
                                     estimate_courtyards=estimate, keepouts=list(keepouts))


def reach(st):
    """The state's #1588 map ({} on a build that has none)."""
    return getattr(st, 'pad_reach', {})


def main():
    from placement import legality
    from placement.seeder import pose_ok
    with tempfile.TemporaryDirectory(prefix='t1588_') as td:
        # -- 1. who is seated on pad copper, and only them --------------------
        st = state(board(os.path.join(td, 'e02'), 0.2))
        u1 = st.parts['U1']
        ok('an error project: every part reaching 0.2 mm is seated on courtyard + pads',
           sorted(reach(st)) == sorted(f'U{i + 1}' for i in range(10))
           and getattr(st, 'pad_reach_floor', None) == legality.COURTYARD_ERROR_MIN_DEPTH_MM
           and abs(reach(st)['U1'].reach_mm - 0.2) < 1e-6, reach(st))
        ok('... and its rect reaches the pad (x + 1.7), not the drawn courtyard (x + 1.5)',
           abs(u1.rect()[2] - (u1.x + 1.7)) < 1e-6 and abs(u1.rect()[0] - (u1.x - 1.5)) < 1e-6,
           u1.rect())
        st0 = state(board(os.path.join(td, 'e00'), 0.0))
        ok('control: pads within the courtyard are untouched',
           reach(st0) == {} and abs(st0.parts['U1'].rect()[2] - (st0.parts['U1'].x + 1.5)) < 1e-6,
           reach(st0))
        stw = state(board(os.path.join(td, 'w02'), 0.2, severity=None))
        stw6 = state(board(os.path.join(td, 'w06'), 0.6, severity=None))
        ok('no project: 0.2 mm is under the 0.3 mm blocking floor (inert), 0.6 mm is not',
           reach(stw) == {} and len(reach(stw6)) == 10
           and getattr(stw6, 'pad_reach_floor', None) == legality.COURTYARD_BLOCKING_MIN_DEPTH_MM,
           (reach(stw), getattr(stw6, 'pad_reach_floor', None)))
        sti = state(board(os.path.join(td, 'i06'), 0.6, severity='ignore'))
        ok('an ignore project gates no courtyard pair, so nothing is seated on pads',
           reach(sti) == {}, reach(sti))
        stg = state(os.path.join(td, 'e02', 'b.kicad_pcb'), estimate=False)
        ok('a grading state (no estimates) keeps the drawn courtyard',
           reach(stg) == {} and abs(stg.parts['U1'].rect()[2] - (stg.parts['U1'].x + 1.5)) < 1e-6)

        # -- 2. the seat refuses what check_assembly gates --------------------
        # U2 beside U1 (at 60, 60): courtyards 0.15 mm apart (clear at 0.1),
        # U1's pad 2 0.05 mm inside U2's courtyard
        two = board(os.path.join(td, 'pair'), 0.2, n=2, poses={0: (6, 9), 1: (15, 9)})
        sp = state(two)
        sp_old = state(two, estimate=False)
        at = (6 + 3.15, 9.0, 0.0)
        from kicad_parser import parse_kicad_pcb
        with contextlib.redirect_stdout(io.StringIO()):
            pcb = parse_kicad_pcb(two)
            census = legality.CourtyardCensus(pcb, two)
            grade = census.grade({'U2': at})
        gating = [(q.a, q.b) for q in grade.error_pairs]
        ok('the pose is one check_assembly gates (an error pair U1/U2)',
           sorted(map(sorted, gating)) == [['U1', 'U2']], gating)
        ok('... which the drawn courtyard alone admits (the bug)',
           pose_ok(sp_old, 'U2', *at, set()))
        ok('... and the seat now refuses it',
           not pose_ok(sp, 'U2', *at, set()))
        clear = (6 + 3.45, 9.0, 0.0)
        ok('0.3 mm further out, past the pad, the seat admits it',
           pose_ok(sp, 'U2', *clear, set()))

        # -- 3. keep-outs read the pad copper, grade and seat ------------------
        # U1's pad 2 spans x 7.3-7.7 (courtyard to 7.5): a circle reaching
        # 7.6 is in the pad, not in the courtyard
        ko = {'name': 'hole', 'circle': [8.2, 9.0, 0.6], 'why': 'test'}
        ip = os.path.join(td, 'pair', 'intent.json')
        json.dump({'kind': 'floorplan-intent', 'schema': 1, 'units': 'mm',
                   'keepouts': [ko]}, open(ip, 'w'))
        from placement import floorplan
        intent = floorplan.load_intent(ip)
        with contextlib.redirect_stdout(io.StringIO()):
            res = floorplan.grade(intent, pcb, two)
        hits = [v.ref for v in res.violations if v.rule == 'keepout']
        ok('check_floorplan flags the pad in the keep-out', hits == ['U1'], hits)
        sk = state(two, estimate=False, keepouts=intent.keepouts)
        ok('... and the seat refuses that pose, on the drawn courtyard too',
           not pose_ok(sk, 'U1', 6.0, 9.0, 0.0, set()))
        ok('... while a pose 0.2 mm left of it is clear',
           pose_ok(sk, 'U1', 5.8, 9.0, 0.0, set()))

        # -- 4. fit_template's clearance reads the same copper ------------------
        from placement.templates import fit
        b6 = board(os.path.join(td, 'e06'), 0.6, n=2, poses={0: (6, 9), 1: (15, 9)})
        with contextlib.redirect_stdout(io.StringIO()):
            pcb6 = parse_kicad_pcb(b6)
            row = fit(pcb6, state(b6), {'anchor': 'U1', 'members': [
                {'ref': 'U2', 'pads': {'U2': ['1'], 'U1': ['2']}}]})
            c6 = legality.CourtyardCensus(pcb6, b6)
        m = row['members'][0]
        g1 = c6.graded_part('U1', (0.0, 0.0, 0.0))
        g2 = c6.graded_part('U2', (m['at'][0], m['at'][1], m['rot']))
        gap = g1.poly.distance(g2.poly)
        ok('the fitted member clears the chip\'s courtyard + pad copper',
           gap >= 0.1 - 1e-6, (m, gap))
        ok('... and its "off the chip" is measured to that copper',
           row['fitted']['off_chip_mm']['U2'] <= gap + 1e-6,
           (row['fitted']['off_chip_mm'], gap))

        # -- 5. place_simple: every seed buildable; the pre-flight said once ---
        d = os.path.join(td, 'run')
        b = board(d, 0.2)
        groups = os.path.join(d, 'groups.json')
        json.dump({'schema': 1, 'groups': [{'name': 'all',
                                            'refs': [f'U{i + 1}' for i in range(10)]}]},
                  open(groups, 'w'))
        it = os.path.join(d, 'intent.json')
        json.dump({'kind': 'floorplan-intent', 'schema': 1}, open(it, 'w'))
        r = check([sys.executable, '-X', 'utf8', tool('place_simple.py'), b,
                   os.path.join(d, 'p{seed}.kicad_pcb'), '--groups', groups, '--intent', it,
                   '--seeds', '0,1', '--jobs', '2', '--moves', '3000',
                   '--json', os.path.join(d, 'r{seed}.json')], accept=True)
        judge = os.path.join(d, 'judge.json')
        evidence(judge, 'the run\'s judge.json')
        jd = json.load(open(judge))
        reps = []
        for s in (0, 1):
            evidence(os.path.join(d, f'r{s}.json'), f'seed {s} report')
            reps.append(json.load(open(os.path.join(d, f'r{s}.json'))))
        ok('every seed is legal and check_assembly-buildable',
           all(not q['illegal'] and (q['assembly'] or {}).get('blocking') == 0 for q in reps),
           [(q['illegal'], q['assembly']) for q in reps])
        ok('the pre-flight reaches the run\'s own output, once',
           r.stdout.count('NOTE: (every seed) pad copper reaches past the drawn courtyard '
                          'by 0.01 mm or more') == 1, r.stdout[-1500:])
        ok('... and judge.json', sorted(jd.get('pad_overhang') or {})
           == sorted(f'U{i + 1}' for i in range(10)), jd.get('pad_overhang'))
    if FAILS:
        print(f"FAILED: {len(FAILS)} check(s)")
        return 1
    print("all checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
