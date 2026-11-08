"""#1756: `place_simple --legalise-only` keeps the board's legal poses.

place_simple started every anneal from a random scatter, so a board whose
parts were already placed could not be given to it as a start: the
arrangement was discarded. `--legalise-only` skips the scatter and the
anneal and moves only the parts that are not legal by KiCad's standard
(the checkers', not place_simple's spacing), each to its nearest
legal pose (the anneal's own post step), mending one no such pose frees with
the anneal's repair, shoves only.

1. A legal board comes back with ZERO moves, its poses as they were: the
   in-repo cap_chain as drawn, and a seed place_simple annealed on it.
2. The same board with two parts made illegal -- C2 dropped onto C1, J2's
   pads pushed past the outline -- comes back legal (check_assembly
   buildable with no baseline, place_simple's own `illegal` empty), only
   J2 and one of the overlapping pair moved, each by a short step.
3. The mode is deterministic: two runs give the same poses.
4. The combinations that make no sense are refused, with the reason.
5. A part that was legal is never broken to mend another: C2 left half in
   a keep-out beside C1, in a pocket with room for one of them, is left
   illegal and named (exit 4) while C1 stays exactly where it stood -- the
   repair may shove a legal part only to another legal pose, and never
   further than LEGALISE_REACH from where the board put it.
6. The standard is KiCad's, not place_simple's spacing: C2 set 0.1 mm from
   C1's courtyard -- closer than the clearance place_simple keeps between
   courtyards, so its own seat test calls it illegal, while check_assembly
   and the pad clearance accept it -- is NOT moved.

    python3 tests/test_1756_legalise_only.py
"""
import json
import math
import os
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, os.path.join(REPO, 'py_placer'))
import run_utils  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
BOARD = os.path.join(REPO, 'kicad_files', 'cap_chain.kicad_pcb')
PLACE = os.path.join(REPO, 'py_placer', 'place_simple.py')
# C2 onto C1 (a courtyard overlap), J2's pads 0.45 mm past the east edge (x = 31)
NUDGES = {'C2': (18.3, 13.4, 0.0), 'J2': (30.6, 15.0, 0.0)}
# the one column the case-5 keep-outs leave open (x0, y0, x1, y1): room for
# one 0402 (C1, rows 12.54-13.46), not for two
POCKET = [16.8, 12.4, 19.2, 14.2]
#: case 6: C2's centre 0.1 mm past where its courtyard would touch C1's
SNUG_Y = 13.46 + 0.1 + 0.46
#: mm the nudged parts may travel back: a nearest legal pose, not a re-placement
SHORT_MM = 3.0
failures = []


def ok(label, cond, detail=''):
    print(f"  [{'ok' if cond else 'FAIL'}] {label}"
          f"{(': ' + str(detail)[-900:]) if detail and not cond else ''}")
    if not cond:
        failures.append(label)


def run(*args):
    return subprocess.run(PY + list(args), capture_output=True, text=True, cwd=REPO,
                          encoding='utf-8', errors='replace')


def poses(path):
    from kicad_parser import parse_kicad_pcb
    return {r: (f.x, f.y, f.rotation % 360.0)
            for r, f in parse_kicad_pcb(run_utils.evidence(path)).footprints.items()}


def legalise(board, out, groups, intent, *extra):
    rep = out[:-len('.kicad_pcb')] + '.json'
    r = run_utils.check(PY + [PLACE, board, out, '--groups', groups, '--intent', intent,
                              '--legalise-only', '--json', rep, *extra], accept=True)
    return json.load(open(run_utils.evidence(rep, 'the legalise report'))), r.stdout


def assembly(board, td, tag):
    """check_assembly with NO baseline: a pair the input already had is not
    excused, so buildable means legal."""
    js = os.path.join(td, f'asm_{tag}.json')
    run(run_utils.tool('check_assembly.py'), board, '--json', js)
    return json.load(open(run_utils.evidence(js, f'the {tag} assembly grade')))


def main():
    from placement.writer import write_placed_output
    from placement.portfolio import copy_siblings
    with tempfile.TemporaryDirectory(prefix='krt1756_') as td:
        board = os.path.join(td, 'b.kicad_pcb')
        run(os.path.join(REPO, 'py_router', 'copy_board.py'), BOARD, board)
        run_utils.evidence(board, 'the fixture board')
        intent = os.path.join(td, 'intent.json')
        run(os.path.join(REPO, 'py_tools', 'check_floorplan.py'), board,
            '--emit-intent', intent)
        run_utils.evidence(intent, 'the emitted intent')
        groups = os.path.join(td, 'groups.json')
        json.dump({'schema': 1, 'groups': [{'name': 'caps', 'refs': ['C1', 'C2']}]},
                  open(groups, 'w'))

        print('1. a legal board comes back with zero moves')
        seeded = os.path.join(td, 'seed0.kicad_pcb')
        run_utils.check(PY + [PLACE, board, seeded, '--groups', groups, '--intent', intent,
                              '--seed', '0'], accept=True)
        for tag, src in (('as drawn', board), ('annealed seed 0', seeded)):
            out = os.path.join(td, f"legal_{tag.replace(' ', '_')}.kicad_pcb")
            rep, _log = legalise(src, out, groups, intent)
            ok(f'{tag}: the report names the mode', rep.get('mode') == 'legalise-only'
               and rep.get('start') == 'board', (rep.get('mode'), rep.get('start')))
            lz = rep.get('legalised') or {}
            ok(f'{tag}: nothing illegal, nothing moved',
               lz.get('count') == 0 and not lz.get('illegal_before') and not rep['illegal'], lz)
            before, after = poses(src), poses(out)
            ok(f'{tag}: every pose identical', before == after,
               {r: (before[r], after.get(r)) for r in before if before[r] != after.get(r)})

        print('2. a board with illegal parts comes back legal, only they moved, a short way')
        nudged = os.path.join(td, 'nudged.kicad_pcb')
        write_placed_output(board, nudged, [
            {'reference': r, 'new_x': x, 'new_y': y, 'new_rotation': rot}
            for r, (x, y, rot) in NUDGES.items()])
        copy_siblings(board, nudged)
        asm0 = assembly(nudged, td, 'nudged')
        ok('the nudged board is NOT buildable (the case is real)',
           asm0.get('buildable') is False, asm0.get('verdict'))
        fixed = os.path.join(td, 'fixed.kicad_pcb')
        rep, log = legalise(nudged, fixed, groups, intent)
        lz = rep['legalised']
        ok('the nudged parts were illegal as the board had them',
           set(NUDGES) <= set(lz['illegal_before']), lz['illegal_before'])
        moved = set(lz['moved'])
        ok('J2 moved, and of the overlapping C1/C2 exactly one, nothing else',
           'J2' in moved and len(moved & {'C1', 'C2'}) == 1 and moved <= {'C1', 'C2', 'J2'},
           sorted(moved))
        ok(f'each moved under {SHORT_MM:g} mm', 0 < lz['max_mm'] < SHORT_MM, lz['max_mm'])
        ok('the log says what it moved', 'legalise only' in log and
           f"{lz['count']} moved" in log and 'moved J2' in log, log[-800:])
        ok("place_simple's own grade: nothing left illegal", rep['illegal'] == [],
           rep['illegal'])
        asm = assembly(fixed, td, 'fixed')
        ok('check_assembly (no baseline): buildable',
           asm.get('buildable') is True, asm.get('verdict'))
        before, after = poses(nudged), poses(fixed)
        ok('every other part exactly where it stood',
           all(before[r] == after[r] for r in before if r not in moved))
        for r in sorted(moved):
            d = math.hypot(after[r][0] - before[r][0], after[r][1] - before[r][1])
            ok(f'{r} written where the report says ({d:.3f} mm)',
               abs(d - lz['moved'][r]['mm']) < 1e-3, (d, lz['moved'][r]))

        print('3. deterministic')
        again = os.path.join(td, 'fixed_again.kicad_pcb')
        rep2, _log = legalise(nudged, again, groups, intent)
        ok('the same moves', rep2['legalised']['moved'] == lz['moved'],
           (rep2['legalised']['moved'], lz['moved']))
        ok('the same poses', poses(again) == after)

        print('4. the combinations that make no sense are refused')
        base = [PLACE, nudged, os.path.join(td, 'x.kicad_pcb'), '--groups', groups,
                '--intent', intent, '--legalise-only']
        for extra, why in ((['--seeds', '0,1'], 'name one --seed, not --seeds'),
                           (['--moves', '100'], '--moves does not apply'),
                           (['--clustered-seeds', '0'], '--clustered-seeds (a start)'),
                           (['--preview'], '--preview draws an annealed seed')):
            run_utils.check(PY + base + extra, refuse=why, code=2)
            ok(f"{' '.join(extra)}: refused, naming why", True)
        run_utils.check(PY + [PLACE, nudged, os.path.join(td, 's{seed}.kicad_pcb'),
                              '--groups', groups, '--intent', intent, '--legalise-only'],
                        refuse='OUT carries {seed} with no --seed', code=2)
        ok('OUT with {seed} and no --seed: refused, naming why', True)

        print('5. a legal neighbour is never broken to mend a part')
        # keep-outs leave one column, room for C1 (legal, where it stands)
        # and not for C2 too: C2 half in the south keep-out can only be
        # mended by pushing C1 into the north one
        pocket = os.path.join(td, 'pocket_intent.json')
        doc = json.load(open(intent))
        x0, y0, x1, y1 = POCKET
        doc['keepouts'] = [{'name': 'west', 'rect': [0, 0, x0, 30], 'allow': ['J1']},
                           {'name': 'east', 'rect': [x1, 0, 40, 30], 'allow': ['J2']},
                           {'name': 'north', 'rect': [x0, 0, x1, y0]},
                           {'name': 'south', 'rect': [x0, y1, x1, 30]}]
        json.dump(doc, open(pocket, 'w'))
        jam = os.path.join(td, 'jam.kicad_pcb')
        write_placed_output(board, jam, [{'reference': 'C2', 'new_x': 18.0, 'new_y': 14.3,
                                          'new_rotation': 0.0}])
        copy_siblings(board, jam)
        out = os.path.join(td, 'jam_out.kicad_pcb')
        r = run_utils.check(PY + [PLACE, jam, out, '--groups', groups, '--intent', pocket,
                                  '--legalise-only', '--json', out[:-10] + '.json'],
                            refuse='C2 is left illegal', code=4)
        rep = json.load(open(run_utils.evidence(out[:-10] + '.json', 'the pocket report')))
        lz = rep['legalised']
        ok('C1 was legal, C2 was not', lz['illegal_before'] == ['C2'], lz['illegal_before'])
        ok('C2 is left illegal and named, inside the keep-out',
           rep['illegal'] == ['C2'] and 'south' in rep['illegal_why'].get('C2', ''),
           (rep['illegal'], rep['illegal_why']))
        ok('no part that was legal is broken', lz['broken'] == [], lz['broken'])
        before, after = poses(jam), poses(out)
        ok('C1 exactly where it stood', before['C1'] == after['C1'], (before['C1'], after['C1']))
        ok('every part that was legal unmoved',
           all(before[q] == after[q] for q in before if q not in lz['illegal_before']))
        ok('the log names it', 'C2 is left illegal' in r.stdout, r.stdout[-600:])

        print("6. closer than place_simple's spacing, legal to KiCad: not moved")
        import pose_score
        from kicad_parser import parse_kicad_pcb
        from placement.simple import legal
        snug = os.path.join(td, 'snug.kicad_pcb')
        # C1's courtyard ends at y 13.46; C2's (0.46 half height) starts 0.1 mm on
        write_placed_output(board, snug, [{'reference': 'C2', 'new_x': 18.0,
                                           'new_y': SNUG_Y, 'new_rotation': 0.0}])
        copy_siblings(board, snug)
        stt = pose_score.make_state(parse_kicad_pcb(snug), snug)
        p2 = stt.parts['C2']
        ok("place_simple's own seat test calls C2 illegal (the case is real)",
           not legal(stt, 'C2', p2.x, p2.y, p2.rot))
        ok('check_assembly (no baseline) calls the board buildable',
           assembly(snug, td, 'snug').get('buildable') is True)
        out = os.path.join(td, 'snug_out.kicad_pcb')
        rep, _log = legalise(snug, out, groups, intent)
        lz = rep['legalised']
        ok('nothing illegal by the standard, nothing moved',
           lz['illegal_before'] == [] and lz['count'] == 0 and rep['illegal'] == [], lz)
        ok('every pose identical', poses(snug) == poses(out))
    print(f"\n{'FAILED: ' + ', '.join(failures) if failures else 'all passed'}")
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
