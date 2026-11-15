"""#1936: a net is routed at its tuning profile's width on each layer.

KiCad 10's tuning profile stores, per signal layer, the track width that meets
its target impedance there (`layer_entries[].width`, nanometres). The reader
took the delays (#1846) and ignored the widths, so a net of a class that names
a profile was drawn at the class's one `track_width` on every layer.

The board is #1846's fixture (nets P and Q of class TIMED, profile PROF6, class
track width 0.1 mm) with Q's track removed and the profile's widths edited.

  1. `profile_layer_widths` gives each profiled net its width per layer, and
     nothing for a layer whose entry stores no width;
  2. a floor pins a narrower width up and says which;
  3. route.py with no --track-width draws Q on F.Cu at the profile's F.Cu
     width and says so;
  4. an explicit --track-width wins;
  5. a class that names no profile is drawn at the class width, as before.
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

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, 'py_router'))

import run_utils  # noqa: E402
import test_1846_time_domain_rules as t1846  # noqa: E402

PY = [sys.executable]
FAILS = []
F_W, B_W = 0.2, 0.13          # the widths the edited profile gives F.Cu and B.Cu


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def _widths(proj):
    prof = t1846._profile(proj)
    prof['layer_entries'][0]['width'] = int(F_W * 1e6)
    prof['layer_entries'].append(dict(t1846.BCU, width=int(B_W * 1e6)))
    prof['layer_entries'].append(dict(t1846.BCU, signal_layer='In1.Cu', width=0))


def _no_profile(proj):
    _widths(proj)
    t1846._no_profile(proj)


def stage(td, name, edit):
    """The fixture with Q's track removed and the project edited."""
    board = t1846.stage(td, name, t1846.FLOOR, False, edit)
    with open(board, encoding='utf-8') as fh:
        text = fh.read()
    text, n = re.subn(r'\t\(segment\s*\(start 3 6\).*?\n\t\)\n', '', text, flags=re.S)
    assert n == 1, "BROKEN TEST: the fixture has Q's one track"
    with open(board, 'w', encoding='utf-8') as fh:
        fh.write(text)
    return run_utils.evidence(board)


def q_widths(board):
    """The widths of Q's routed segments, by layer."""
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        pcb = parse_kicad_pcb(board)
    nid = next(i for i, n in pcb.nets.items() if n.name == 'Q')
    out = {}
    for s in pcb.segments:
        if s.net_id == nid:
            out.setdefault(s.layer, set()).add(round(s.width, 4))
    return out


def route(td, board, tag, extra=()):
    out = os.path.join(td, tag + '.kicad_pcb')
    r = subprocess.run(PY + [os.path.join('py_router', 'route.py'), board, out,
                             '--nets', 'Q', '--layers', 'F.Cu'] + list(extra),
                       capture_output=True, text=True, cwd=REPO)
    line = next((ln for ln in r.stdout.splitlines()
                 if ln.startswith('JSON_SUMMARY_MIN:')), None)
    doc = json.loads(line[len('JSON_SUMMARY_MIN:'):]) if line else {}
    assert doc.get('routed') == 1, (f"BROKEN TEST: Q did not route ({tag})\n"
                                    f"{(r.stdout + r.stderr)[-800:]}")
    return run_utils.evidence(out), r.stdout


def reader(td):
    print('1-2. the reader')
    from kicad_parser import parse_kicad_pcb
    from tuning_profiles import profile_layer_widths, read_tuning_profiles
    board = stage(td, 'rd', _widths)
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        pcb = parse_kicad_pcb(board)
    ids = {n.name: i for i, n in pcb.nets.items() if i}
    prof = read_tuning_profiles(board)[0]['PROF6']
    ok(prof.layer_width_mm == {'F.Cu': F_W, 'B.Cu': B_W} and not prof.differential,
       f"the profile carries its widths, and none for the layer that stores 0: "
       f"{prof.layer_width_mm}")
    got, used, raised = profile_layer_widths(pcb, board)
    ok(got.get(ids['Q']) == {'F.Cu': F_W, 'B.Cu': B_W}
       and got.get(ids['P']) == {'F.Cu': F_W, 'B.Cu': B_W}
       and used == {'PROF6': len(got)} and not raised,
       f"each net of the class gets them: {got.get(ids['Q'])}, {used}")
    got, _used, raised = profile_layer_widths(pcb, board, floor=0.15)
    ok(got[ids['Q']] == {'F.Cu': F_W, 'B.Cu': 0.15}
       and raised == [('PROF6', 'B.Cu', B_W, 0.15)],
       f"a floor pins the narrower width up and names it: {got[ids['Q']]}, {raised}")
    got, _u, _r = profile_layer_widths(pcb, board, net_ids=[ids['Q']])
    ok(list(got) == [ids['Q']], 'and it answers for the nets asked')


def routed(td):
    print('3-5. routed')
    board = stage(td, 'rt', _widths)
    out, log = route(td, board, 'rt_o')
    ok(q_widths(out) == {'F.Cu': {F_W}},
       f"with no --track-width Q is drawn at the profile's F.Cu width: {q_widths(out)}")
    ok('Tuning-profile widths:' in log and 'PROF6' in log,
       'and the run says which profile gave the widths')
    out, log = route(td, board, 'rt_tw', ['--track-width', '0.15'])
    ok(q_widths(out) == {'F.Cu': {0.15}} and 'Tuning-profile widths:' not in log,
       f"an explicit --track-width wins: {q_widths(out)}")
    board = stage(td, 'np', _no_profile)
    out, log = route(td, board, 'np_o')
    ok(q_widths(out) == {'F.Cu': {0.1}} and 'Tuning-profile widths:' not in log,
       f"a class that names no profile is drawn at the class width: {q_widths(out)}")


def main():
    td = tempfile.mkdtemp(prefix='krt1936_')
    try:
        reader(td)
        routed(td)
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if FAILS:
        print(f'FAIL: {len(FAILS)}: ' + '; '.join(FAILS))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
