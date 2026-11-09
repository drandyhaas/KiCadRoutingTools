#!/usr/bin/env python3
"""#1735: five follow-ups to route_plan_check and the replayer's hole readers.

Checks:
  1. R01 asks for `set -e` before the first command, in the one spelling the
     plan's readers skip as a directive: refused without it, accepted before
     or after the `cd`, refused as `set -eu`;
  2. `compute_prune_keep`'s chain_holes seeds from the boards read before any
     output, so a manifest whose first command is a `--help` does not report
     its own input board as a hole -- and a real hole is still one;
  3. `compute_contamination` grades a board first read mid-chain the same
     whether the manifest spelled its path absolute or relative (the
     absolute-path exemption it carried could not fire: boards are keyed by
     basename). Behaviour-neutral, so this check also passes on base;
  4. R12 asks for the cap pass after the bus step (`route_bus.py`, which fans
     out its ball arrays), but not after `route_bus.py --find`;
  5. R15 places the GND-via floor where route_planes places its vias: at the
     board's Default class as LOWERED by an upstream step's written-back
     clearance (not by one that writes nothing, nor by a step off the GND
     pass's lineage), and pinned UP to the physical fab floor (the clearance,
     and a via whose drill leaves no annular ring).

    python3 tests/test_1735_plan_checker_followups.py
"""
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [os.path.join(ROOT, 'tests'), os.path.join(ROOT, 'tests', 'stress'),
                os.path.join(ROOT, 'py_router')]
from run_utils import evidence  # noqa: E402

CHECKER = os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-routing',
                       'scripts', 'route_plan_check.py')
BOARD = os.path.join(ROOT, 'tests', 'fixtures', 'run25', 'esp_prog_placed.kicad_pcb')

VERIFY = ('# python3 -X utf8 py_router/check_drc.py final.kicad_pcb\n'
          '# python3 -X utf8 py_router/check_connected.py final.kicad_pcb\n'
          '# python3 -X utf8 py_tools/check_orphan_stubs.py final.kicad_pcb\n')
POUR = 'python3 -X utf8 py_router/route_planes.py {a} {b} --nets GND --plane-layers B.Cu'
ROUTE = "python3 -X utf8 py_router/route.py {a} {b} --nets '*' --power-nets GND"

failures = []


def ok(label, cond, detail=''):
    print(f"  [{'ok' if cond else 'FAIL'}] {label}{(': ' + detail) if detail and not cond else ''}")
    if not cond:
        failures.append(label)


def run(td, name, text, board=BOARD):
    """(exit, failed rule ids, {rule: (status word, lines under it)})."""
    path = os.path.join(td, name)
    with open(path, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write(text)
    evidence(path, name)
    r = subprocess.run([sys.executable, '-X', 'utf8', CHECKER, path]
                       + (['--board', board] if board else []),
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=ROOT, timeout=120)
    assert 'Traceback' not in r.stdout + r.stderr, r.stdout + r.stderr
    rules, cur = {}, None
    for ln in r.stdout.splitlines():
        w = ln.split()
        if len(w) > 1 and w[0] in ('ok', 'FAIL', 'SKIP', 'NOTE') and w[1][:1] == 'R':
            cur = w[1]
            rules[cur] = (w[0], [])
        elif cur and ln.startswith('          '):
            rules[cur][1].append(ln.strip())
        else:
            cur = None
    failed = {k for k, (st, _) in rules.items() if st == 'FAIL'}
    return r.returncode, failed, rules


def chain(*steps, head='set -e\ncd /repo\n'):
    """`steps` as a plan: board k is s<k>.kicad_pcb, the last one final."""
    lines = []
    for i, s in enumerate(steps):
        b = 'final.kicad_pcb' if i == len(steps) - 1 else f's{i + 1}.kicad_pcb'
        lines.append(s.format(a=f's{i}.kicad_pcb', b=b))
    return head + '\n'.join(lines) + '\n' + VERIFY


def r01(td):
    print('1. R01 asks for set -e')
    body = chain(POUR, ROUTE, head='')
    rc, failed, rules = run(td, 'nosete.sh', 'cd /repo\n' + body)
    ok('no set -e: refused by R01, saying why',
       failed == {'R01'} and any('set -e' in t for t in rules['R01'][1]),
       f'{sorted(failed)} {rules.get("R01")}')
    rc, failed, _ = run(td, 'after.sh', 'cd /repo\nset -e\n' + body)
    ok('set -e after the cd: accepted', rc == 0, f'rc={rc} {sorted(failed)}')
    rc, failed, _ = run(td, 'before.sh', 'set -e\ncd /repo\n' + body)
    ok('set -e before the cd: accepted', rc == 0, f'rc={rc} {sorted(failed)}')
    rc, failed, _ = run(td, 'eu.sh', 'cd /repo\nset -eu\n' + body)
    ok('set -eu (a spelling the readers take as a command): refused',
       'R01' in failed, f'{sorted(failed)}')


def holes(td):
    print("2-3. the replayer's hole readers")
    import redo_stress_test as R
    d = os.path.join(td, 'm')
    os.makedirs(d)
    src = os.path.join(d, 'src.kicad_pcb')
    man = os.path.join(d, 'redo_commands.sh')
    with open(man, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write('set -e\n'
                 'python3 -X utf8 py_router/route_planes.py --help\n'
                 f'python3 -X utf8 py_router/route_planes.py {src} s1.kicad_pcb --nets GND\n'
                 "python3 -X utf8 py_router/route.py s1.kicad_pcb final.kicad_pcb --nets '*'\n")
    cmds = R.parse_manifest(evidence(man, 'manifest'))
    ok('a leading --help: the input board is a seed, not a chain hole',
       R.compute_prune_keep(cmds)[1].get('chain_holes') == [],
       str(R.compute_prune_keep(cmds)[1].get('chain_holes')))
    with open(man, 'a', encoding='utf-8', newline='\n') as fh:
        fh.write("python3 -X utf8 py_router/route.py hand.kicad_pcb final2.kicad_pcb --nets '*'\n")
    cmds = R.parse_manifest(man)
    ok('...a board first read mid-chain is still a hole',
       R.compute_prune_keep(cmds)[1].get('chain_holes') == ['hand.kicad_pcb'],
       str(R.compute_prune_keep(cmds)[1].get('chain_holes')))
    for spelled in ('mid.kicad_pcb', os.path.join(d, 'mid.kicad_pcb')):
        cm = [(None, ['python3', 'py_router/route_planes.py', src, 's1.kicad_pcb']),
              (None, ['python3', 'py_router/route.py', spelled, 'x.kicad_pcb'])]
        ok(f'contamination: a mid-chain {"absolute" if os.path.isabs(spelled) else "relative"} '
           f'unproduced board is a hole', R.compute_contamination(cm)[0] == ['mid.kicad_pcb'],
           str(R.compute_contamination(cm)[0]))


def r12(td):
    print('4. R12 and the bus step')
    BUS = 'python3 -X utf8 awx/route_bus.py {a} {b} --src U1 --dest U2'
    CAP = 'python3 -X utf8 py_placer/place_fanout_clearance.py {a} {b}'
    rc, failed, rules = run(td, 'bus.sh', chain(POUR, BUS, ROUTE), board=None)
    ok('a bus step and no cap pass: refused by R12, naming route_bus.py',
       failed == {'R12'} and any('route_bus.py' in t for t in rules['R12'][1]),
       f'{sorted(failed)} {rules.get("R12")}')
    rc, failed, _ = run(td, 'buscap.sh', chain(POUR, BUS, CAP, ROUTE), board=None)
    ok('...with the cap pass after it: accepted', rc == 0, f'rc={rc} {sorted(failed)}')
    find = ('set -e\ncd /repo\npython3 -X utf8 awx/route_bus.py s0.kicad_pcb --find\n'
            + chain(POUR, ROUTE, head=''))
    rc, failed, _ = run(td, 'find.sh', find, board=None)
    ok('route_bus.py --find (writes nothing) asks for no cap pass',
       'R12' not in failed, f'{sorted(failed)}')


def r15(td):
    print('5. R15 at the clearance and via the step places its vias at')
    OWN = '--no-fix-drc-settings'
    GND = ('python3 -X utf8 py_router/route_planes.py {a} {b} --nets GND GND '
           '--plane-layers F.Cu B.Cu --add-gnd-vias ' + OWN + ' %s')

    def r15_of(name, *steps, board=BOARD):
        rc, failed, rules = run(td, name, chain(*steps), board=board)
        return rules['R15']

    up = ROUTE + ' --clearance-ceiling 0.1'
    st, why = r15_of('wb_ok.sh', POUR + ' ' + OWN, up, GND % '--via-size 0.45 --gnd-via-distance 1.7')
    ok('an upstream ceiling written back lowers the class: 1.7 clears 3x(0.45+0.1)',
       st == 'ok', f'{st} {why}')
    st, why = r15_of('wb_low.sh', POUR + ' ' + OWN, up, GND % '--via-size 0.45 --gnd-via-distance 1.6')
    ok('...and 1.6 is refused at 1.650, naming the step that lowered it',
       st == 'FAIL' and why and 'lowered upstream by route.py' in why[0]
       and '= 1.650' in why[0], f'{st} {why}')
    st, why = r15_of('wb_off.sh', POUR + ' ' + OWN, up + ' ' + OWN,
                     GND % '--via-size 0.45 --gnd-via-distance 1.7')
    ok('an upstream step that writes nothing back (--no-fix-drc-settings) does not',
       st == 'FAIL' and why and '[board Default class]' in why[0] and '= 1.800' in why[0],
       f'{st} {why}')
    side = ("set -e\ncd /repo\n"
            "python3 -X utf8 py_router/route.py s0.kicad_pcb side.kicad_pcb --nets '*' "
            "--clearance-ceiling 0.1\n"
            + chain(POUR + ' ' + OWN, GND % '--via-size 0.45 --gnd-via-distance 1.7', ROUTE,
                    head=''))
    rc, failed, rules = run(td, 'side.sh', side)
    st, why = rules['R15']
    ok('...nor does a step off the GND pass\'s lineage',
       st == 'FAIL' and why and '= 1.800' in why[0], f'{st} {why}')
    st, why = r15_of('pin.sh', POUR + ' ' + OWN,
                     GND % '--via-size 0.45 --clearance-ceiling 0.05 --gnd-via-distance 1.6',
                     ROUTE)
    ok('a ceiling below the 2-layer fab floor is pinned up to 0.1: 1.6 refused at 1.650',
       st == 'FAIL' and why and 'pinned to the fab floor' in why[0] and '= 1.650' in why[0],
       f'{st} {why}')
    st, why = r15_of('ring.sh', POUR + ' ' + OWN,
                     GND % '--via-size 0.3 --via-drill 0.3 --gnd-via-distance 1.5', ROUTE)
    ok('a via with no annular ring grows to drill + 2 x 0.05: 1.5 refused at 1.650',
       st == 'FAIL' and why and 'via 0.4 [this step, pinned' in why[0] and '= 1.650' in why[0],
       f'{st} {why}')


def main():
    with tempfile.TemporaryDirectory() as td:
        r01(td)
        holes(td)
        r12(td)
        r15(td)
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
