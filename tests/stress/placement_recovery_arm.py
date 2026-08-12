"""One placement-recovery arm, end to end, with no LLM in the loop.

Damage a board, run the reconstruct ladder on it, grade the result, and only
THEN open the control to compute recovery. Everything before that line is
board-only, which is what makes the recovery number mean anything.

    python3 -X utf8 tests/stress/placement_recovery_arm.py <name> <clearance>

expects wk/run8/<name>/ to hold perturbed.kicad_pcb, its control and its
perturb.json (see the workdir prep in the run-8 notes), and prints one JSON
object: the gate numbers, the graded output, how many vectors each source
produced, the rigid-consistency count, and the recovery.

THAT LAYOUT IS NOW A LEAK, and this reader is kept on it only to stay usable
against the run-8 dirs that already exist. Truth living in the work dir is what
run-12 Tier 0 closed: `fence_audit --mode create` reports a control inside the
work dir whatever it is named, and `perturb(control_out=...)` puts the control
AND the record in a sibling `_truth/`. Stage anything NEW that way
(tests/stress/RUNBOOK.md, "Staging a perturbed subject") and point `ctrl` at
the fenced path; do not copy this dir shape forward.

Two rules this encodes, both learned the expensive way:

  * the applied dose must exceed the home tolerance, or the arm cannot measure
    recovery at all -- one run shipped an arm clipped to 1.2mm against a 2mm
    tolerance and reported a recovery number for it;
  * truth is opened at ONE point, after the board is frozen. Everything above
    that comment runs on the damaged board alone.
"""
import json
import math
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
os.chdir(ROOT)
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # #522/py_placer layout
os.environ['KRT_NO_BANNER'] = '1'

from kicad_parser import parse_kicad_pcb
from placement import recovery

PY = [sys.executable, '-X', 'utf8']


def run(args, timeout=1800):
    p = subprocess.run(PY + args, capture_output=True, text=True,
                       encoding='utf-8', errors='replace', timeout=timeout)
    return p.returncode, (p.stdout or '') + (p.stderr or '')


def summary(out):
    for line in reversed(out.splitlines()):
        if line.startswith('JSON_SUMMARY: '):
            try:
                return json.loads(line[len('JSON_SUMMARY: '):])
            except Exception:
                return {}
    return {}


def count_drc(board, clr):
    code, out = run(['check_drc.py', board, '--clearance', str(clr),
                     '--clearance-margin', '0'])
    if 'NO DRC VIOLATIONS' in out:
        return 0
    for line in out.splitlines():
        if line.startswith('LISTING:'):
            try:
                return int(line.split()[3])
            except Exception:
                pass
    return sum(1 for ln in out.splitlines() if ln.startswith('  Pad:'))


def assembly(board, baseline=None):
    args = ['check_assembly.py', board]
    if baseline:
        args += ['--baseline', baseline]
    code, out = run(args)
    blocking = advisory = locked = 0
    for ln in out.splitlines():
        if ln.strip().startswith('blocking '):
            parts = ln.split()
            blocking = int(parts[1])
            advisory = int(parts[3])
        if 'LOCKED-PART CONTACT' in ln:
            locked = int(ln.split('(')[1].split(')')[0])
    return blocking, advisory, locked, code


def arm(name, clearance):
    wd = f'wk/run8/{name}'
    dmg = f'{wd}/perturbed.kicad_pcb'
    ctrl = f'{wd}/perturbed.control.kicad_pcb'
    res = {'board': name}

    res['gate_drc'] = count_drc(dmg, clearance)
    b, a, l, _ = assembly(dmg)
    res['gate_blocking'], res['gate_locked'] = b, l

    out_board = f'{wd}/r1.kicad_pcb'
    code, out = run(['place_reconstruct.py', dmg, out_board,
                     '--clearance', str(clearance)])
    rep = summary(out)
    res['recon_exit'] = code
    res['gate_before'] = rep.get('gate_before')
    res['gate_after'] = rep.get('gate_after_exchange') or rep.get(
        'gate_after_assign')
    res['vectors'] = len(rep.get('vectors') or [])
    res['vectors_derived'] = len(rep.get('vectors_derived') or [])
    if not os.path.exists(out_board):
        res['error'] = 'no output board'
        return res

    res['out_drc'] = count_drc(out_board, clearance)
    b, a, l, code = assembly(out_board, baseline=dmg)
    res['out_blocking'], res['out_locked'] = b, l

    code, out = run(['check_rigid_consistency.py', dmg, out_board,
                     '--clearance', str(clearance)])
    res['rigid_inconsistent'] = out.count('moved [')

    # --- truth opens HERE, and only here -------------------------------
    rec = json.load(open(f'{wd}/perturbed.perturb.json', encoding='utf-8'))
    members = list((rec.get('at_emit', {}).get('displacement', {})
                    .get('per_part') or {}).keys())
    c, d, f = (parse_kicad_pcb(p) for p in (ctrl, dmg, out_board))
    pc, pd, pf = (recovery.board_poses(x) for x in (c, d, f))
    d0 = recovery.displacement_to_original(pd, pc, d.footprints, subset=members)
    d1 = recovery.displacement_to_original(pf, pc, f.footprints, subset=members)
    res['members'] = len(members)
    res['d0'] = d0['perturbed_pad_rms']
    res['d1'] = d1['perturbed_pad_rms']
    res['recovery'] = recovery.recovery_fraction(d0['perturbed_pad_rms'],
                                                 d1['perturbed_pad_rms'])
    res['home_before'] = sum(
        1 for r in members
        if r in pd and r in pc
        and math.hypot(pd[r][0] - pc[r][0], pd[r][1] - pc[r][1]) <= 2.0)
    res['home_after'] = sum(
        1 for r in members
        if r in pf and r in pc
        and math.hypot(pf[r][0] - pc[r][0], pf[r][1] - pc[r][1]) <= 2.0)
    return res


if __name__ == '__main__':
    name, clr = sys.argv[1], float(sys.argv[2])
    print(json.dumps(arm(name, clr), indent=1, sort_keys=True))
