"""#1486: a placement run records itself, for a replay with no AI.

With PLACEMENT_MANIFEST set, each placement tool appends its invocation to
that JSON record, with every file it reads by CONTENT -- so a file the
agent edits between two steps (an emitted intent tightened by hand) is the
edited text at the step that read it. A place_simple run records once, not
once per seed. Unset, nothing is recorded.
"""

import json
import os
import shutil
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))
from placement import run_record                              # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
FAILS = []


def ok(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}"
          + (f"  ({detail})" if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


def main():
    with tempfile.TemporaryDirectory() as td:
        man = os.path.join(td, 'placement_manifest.json')
        env = dict(os.environ, PLACEMENT_MANIFEST=man)

        def run(*args):
            return subprocess.run([sys.executable, '-X', 'utf8'] + list(args), env=env,
                                  capture_output=True, text=True, cwd=td)
        pile, intent = os.path.join(td, 'pile.kicad_pcb'), os.path.join(td, 'intent.json')
        run(os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-placement', 'scripts',
                         'make_unplaced.py'), BOARD, pile)
        run(os.path.join(ROOT, 'py_tools', 'check_floorplan.py'), pile, '--allow-unplaced',
            '--emit-intent', intent)
        emitted = open(intent).read()
        doc = json.load(open(intent))           # the agent's own edit, between steps
        doc['proximity'] = doc.get('proximity', [])
        doc.setdefault('context', {})['note'] = 'edited by hand'
        json.dump(doc, open(intent, 'w'))
        edited = open(intent).read()
        groups = os.path.join(td, 'groups.json')
        json.dump({'schema': 1, 'groups': [{'name': 'all', 'refs': ['U1', 'U2', 'C1', 'C2']}]},
                  open(groups, 'w'))
        # the agent places a plain copy of the pile, which no step records
        fixed = os.path.join(td, 'fixed.kicad_pcb')
        with open(fixed[:-len('.kicad_pcb')] + '.kicad_pro', 'w') as fh:
            fh.write('{"board": {"design_settings": {}}}')
        shutil.copy(pile, fixed)
        r = run(os.path.join(ROOT, 'py_placer', 'place_simple.py'), fixed,
                os.path.join(td, 's{seed}.kicad_pcb'), '--groups', groups, '--intent', intent,
                '--seeds', '0-1', '--jobs', '1')
        ok('the run placed', os.path.exists(os.path.join(td, 's1.kicad_pcb')), r.stderr[-300:])
        rec = run_record.load(man)
        tools = [s['tool'] for s in rec['steps']]
        ok('each step once, in order (no per-seed records)',
           tools == ['make_unplaced', 'check_floorplan', 'place_simple'], tools)
        ps = rec['steps'][-1]
        ok('the step names the files it read', set(ps['inputs']) >= {groups, intent},
           ps['inputs'])
        ok('the intent as place_simple read it: the edited text',
           rec['files'][ps['inputs'][intent]] == edited != emitted)
        dst = os.path.join(td, 'restored')
        run_record.restore_inputs(rec, ps, lambda p: os.path.join(dst, os.path.basename(p)))
        ok('a replay restores it exactly',
           open(os.path.join(dst, 'intent.json')).read() == edited)
        ok('the board it placed, and its project, restored as it read them',
           all(open(os.path.join(dst, 'fixed' + ext)).read()
               == open(fixed[:-len('.kicad_pcb')] + ext).read()
               for ext in ('.kicad_pcb', '.kicad_pro')))
        env.pop('PLACEMENT_MANIFEST')
        os.remove(man)
        run(os.path.join(ROOT, 'py_tools', 'check_floorplan.py'), pile, '--allow-unplaced',
            '--emit-intent', intent)
        ok('unset, nothing is recorded', not os.path.exists(man))
    print('ALL PASS' if not FAILS else f'FAILED: {FAILS}')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
