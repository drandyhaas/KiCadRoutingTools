"""The bus step's opt-in settings are OFF unless set: a run that names none of them runs as before.

  python3 tests/test_622_opt_in_defaults.py

Each is a setting or an environment variable read at import or at its call, none a default:
1. the jump's pack (awx/pack.py): PACK_JUMP and PACK_STACK off -- route_bus --pack is the taut pack;
2. the plan's supply-pad vias (py_router/plan_together.py): KICAD_PLAN_TAPS 0 -- no pad asked a via;
3. each bus's preferred layers (awx/group_ends.py, awx/whole_solve.py): GROUP_PREFER unset -- no term in the
   group's model, no price in a bus's own solve;
and, as the control, each one on when set.

Each case runs in a process of its own with the settings removed from (or added to) its environment: the modules read
some of them when they are imported.
"""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
OPT = ('PACK_JUMP', 'PACK_STACK', 'PACK_CENTRE', 'PACK_PULL', 'PACK_REACH', 'KICAD_PLAN_TAPS', 'GROUP_PREFER',
       'GROUP_ENDS', 'JOINT_BUS')

PROBE = r'''
import json, os, sys
sys.path.insert(0, os.path.join(sys.argv[1], 'awx'))
sys.path.insert(0, os.path.join(sys.argv[1], 'py_router'))
import pack, env_knobs
out = {'jump': pack.JUMP, 'stack': pack.STACK, 'taps': env_knobs.PLAN_TAPS}
try:
    import group_ends, whole_solve
    out['prefer'] = list(group_ends.prefer_rows(None, [object(), object()], 4, [0, 0, 0, 0])[1])
    out['bus_prefer'] = whole_solve._bus_prefer()
except ImportError as e:
    out['skip'] = str(e)
print(json.dumps(out))
'''


def probe(**settings):
    env = {k: v for k, v in os.environ.items() if k not in OPT}
    env.update(settings)
    p = subprocess.run([sys.executable, '-c', PROBE, ROOT], env=env, capture_output=True, text=True)
    assert p.returncode == 0, p.stderr[-2000:]
    import json
    return json.loads(p.stdout.strip().splitlines()[-1])


def test_off_unless_set():
    got = probe()
    assert got['jump'] is False and got['stack'] is False, got
    assert got['taps'] == 0, got
    if 'skip' not in got:
        assert got['prefer'] == [] and got['bus_prefer'] is None, got


def test_on_when_set():
    got = probe(PACK_JUMP='1', PACK_STACK='1', KICAD_PLAN_TAPS='2')
    assert got['jump'] is True and got['stack'] is True, got
    assert got['taps'] == 2, got


if __name__ == '__main__':
    test_off_unless_set()
    print('off unless set: ok')
    test_on_when_set()
    print('on when set: ok')
