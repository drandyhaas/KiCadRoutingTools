"""#1588: a courtyard pair is graded courtyard on courtyard, as KiCad grades it.

check_assembly measured a courtyard pair on the drawn courtyard UNITED with
the part's pad copper (`legality.occupancy_shape`, #1094), so a pad reaching
past its own courtyard into a neighbour's was a pair -- one kicad-cli's
`courtyards_overlap` does not report. On a project with `courtyards_overlap:
error` (#1385) every seed of the issue's repro graded NOT BUILDABLE on such
pairs while KiCad reported none. A drawn courtyard is now graded as drawn
(a part that draws none keeps its occupancy box, as before), so the gate
agrees with KiCad and the placer, which seats on the drawn courtyard, agrees
with the gate. Pad copper stays graded where it is a rule -- and a keep-out
(a screw head, a hole) is physical, so `rule_keepout` and the seat's keep-out
test read pad copper beside the courtyard. place_simple still names a part
whose pads reach past its courtyard (#1386), as information, once per run and
in judge.json. Built from the issue's repro: 3 x 2 mm parts whose pad 2
reaches OVER mm past the drawn courtyard.

    python3 tests/test_1588_pad_reach_seat.py
"""
import contextlib
import io
import json
import os
import re
import shutil
import subprocess
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


def board(d, over, n=10, poses=None):
    """The repro's board (`n` parts) in `d`, its project at
    courtyards_overlap=error. Returns its path."""
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
    json.dump({'board': {'design_settings': {
                   'rule_severities': {'courtyards_overlap': 'error'},
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


def census_pairs(path):
    """check_assembly's courtyard pairs on `path` (KiCad's error currency)."""
    from kicad_parser import parse_kicad_pcb
    from placement import legality
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(path)
        g = legality.CourtyardCensus(pcb, path).grade()
    return {tuple(sorted((q.a, q.b))) for q in g.error_pairs}


def kicad_pairs(path):
    """kicad-cli's courtyards_overlap pairs, or None without kicad-cli."""
    from kicad_oracle import find_kicad_cli
    cli = find_kicad_cli(warn=False)
    if not cli:
        return None
    out = os.path.join(os.path.dirname(path), 'drc.json')
    subprocess.run([cli, 'pcb', 'drc', '--format', 'json', '--severity-all', '-o', out, path],
                   capture_output=True, timeout=300)
    evidence(out, "kicad-cli's DRC report")
    got = set()
    for v in json.load(open(out, encoding='utf-8'))['violations']:
        if v['type'] == 'courtyards_overlap':
            refs = [m.group(1) for i in v['items']
                    for m in [re.search(r'Footprint (\S+)', i['description'])] if m]
            got.add(tuple(sorted(refs)))
    return got


def main():
    from placement.seeder import pose_ok
    with tempfile.TemporaryDirectory(prefix='t1588_') as td:
        # -- 1. the gate is KiCad's: courtyard on courtyard ---------------------
        # U1 at (6, 9); U2 0.15 mm clear of its courtyard with U1's pad 2
        # 0.05 mm inside U2's courtyard (KiCad: clean), and U3 overlapping
        # U4's courtyard by 0.05 mm (KiCad: an error), on one board
        pads_in = (6 + 3.15, 9.0)
        poses = {0: (6, 9), 1: pads_in, 2: (6, 14), 3: (6 + 2.95, 14)}
        b = board(os.path.join(td, 'gate'), 0.2, n=4, poses=poses)
        got = census_pairs(b)
        ok('a pad over a neighbour\'s courtyard is no courtyard pair; '
           'courtyards that meet are', got == {('U3', 'U4')}, got)
        kp = kicad_pairs(b)
        if kp is None:
            print('  SKIP  kicad-cli agreement (no kicad-cli)')
        else:
            ok('... exactly the pairs kicad-cli reports', kp == got, (kp, got))

        # -- 2. the placer seats on the drawn courtyard -------------------------
        two = board(os.path.join(td, 'pair'), 0.2, n=2, poses={0: (6, 9), 1: (15, 9)})
        st = state(two)
        u1 = st.parts['U1']
        ok('the seat\'s rect is the drawn courtyard (x + 1.5), not the pad (x + 1.7)',
           abs(u1.rect()[2] - (u1.x + 1.5)) < 1e-6, u1.rect())
        ok('... so it admits the pose KiCad passes (a pad over a courtyard)',
           pose_ok(st, 'U2', *pads_in, 0.0, set()))

        # -- 3. keep-outs read the pad copper, grade and seat ------------------
        # U1's pad 2 spans x 7.3-7.7 (courtyard to 7.5): a circle reaching
        # 7.6 is in the pad, not in the courtyard
        ko = {'name': 'hole', 'circle': [8.2, 9.0, 0.6], 'why': 'test'}
        ip = os.path.join(td, 'pair', 'intent.json')
        json.dump({'kind': 'floorplan-intent', 'schema': 1, 'units': 'mm',
                   'keepouts': [ko]}, open(ip, 'w'))
        from kicad_parser import parse_kicad_pcb
        from placement import floorplan
        intent = floorplan.load_intent(ip)
        with contextlib.redirect_stdout(io.StringIO()):
            res = floorplan.grade(intent, parse_kicad_pcb(two), two)
        hits = [v.ref for v in res.violations if v.rule == 'keepout']
        ok('check_floorplan flags the pad in the keep-out', hits == ['U1'], hits)
        sk = state(two, keepouts=intent.keepouts)
        ok('... and the seat refuses that pose',
           not pose_ok(sk, 'U1', 6.0, 9.0, 0.0, set()))
        ok('... while a pose 0.2 mm left of it is clear',
           pose_ok(sk, 'U1', 5.8, 9.0, 0.0, set()))

        # -- 4. fit_template measures "off the chip" on the courtyard ----------
        from placement.legality import rect_gap
        from placement.templates import fit
        b6 = board(os.path.join(td, 'e06'), 0.6, n=2, poses={0: (6, 9), 1: (15, 9)})
        s6 = state(b6)
        with contextlib.redirect_stdout(io.StringIO()):
            row = fit(parse_kicad_pcb(b6), s6, {'anchor': 'U1', 'members': [
                {'ref': 'U2', 'pads': {'U2': ['1'], 'U1': ['2']}}]})
        m = row['members'][0]
        gap = rect_gap(s6.parts['U1'].rect(0.0, 0.0, 0.0),
                       s6.parts['U2'].rect(m['at'][0], m['at'][1], m['rot']))
        ok('"off the chip" is the gap between the drawn courtyards',
           abs(row['fitted']['off_chip_mm']['U2'] - round(max(0.0, gap), 3)) < 1e-6
           and gap >= 0.1 - 1e-6, (row['fitted']['off_chip_mm'], gap))

        # -- 5. place_simple: buildable seeds; the pre-flight note said once ---
        d = os.path.join(td, 'run')
        b = board(d, 0.6)
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
        ok('the pre-flight note reaches the run\'s own output, once',
           r.stdout.count('NOTE: (every seed) pad copper reaches more than 0.3 mm past '
                          'the drawn courtyard') == 1, r.stdout[-1500:])
        ok('... and judge.json', sorted(jd.get('pad_overhang') or {})
           == sorted(f'U{i + 1}' for i in range(10)), jd.get('pad_overhang'))
        placed = kicad_pairs(os.path.join(d, 'p0.kicad_pcb'))
        if placed is not None:
            ok('... and kicad-cli reports no courtyard pair on a placed seed',
               placed == set(), placed)
        shutil.rmtree(d, ignore_errors=True)
    if FAILS:
        print(f"FAILED: {len(FAILS)} check(s)")
        return 1
    print("all checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
