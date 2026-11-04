"""#1430: the bus step names the balls of its arrays it walls in.

A plain bus between two array parts can leave an array's OTHER ball (a net that
is not a bus net) reachable on the board as it came and reachable from nowhere
once the locked lanes are laid: its net then ships open, and no check of the
step saw it (exit 0, connected=1 drc=1). `route_bus.walled_grade` names such
balls: the BUS line's `walled=`, a log line, the summary's `walled`.

End to end (about 3 minutes, one routing process) on a bus that walls one ball
in: the BUS line carries `walled=1`, the log names the ball and its net, and the
summary lists it. The control row: a board graded against itself walls nothing.
"""
import json
import os
import re
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
from run_utils import evidence

BOARD = os.path.join(ROOT, 'kicad_files', 'orangecrab_ext_pll.kicad_pcb')


def test_walled_ball_is_named():
    with tempfile.TemporaryDirectory() as tmp:
        evidence(BOARD, 'the input board')
        oc = os.path.join(tmp, 'oc.kicad_pcb')
        subprocess.run([sys.executable, '-B', '-X', 'utf8',
                        os.path.join(ROOT, 'tests', 'stress', 'strip_copper_only.py'), BOARD, oc],
                       check=True, capture_output=True, cwd=ROOT)
        evidence(oc, 'the stripped board')
        out = os.path.join(tmp, 'bus', 'oc_bus.kicad_pcb')
        r = subprocess.run([sys.executable, '-B', '-X', 'utf8', os.path.join(ROOT, 'awx', 'route_bus.py'),
                            oc, out, '--rounds', '1', '--inproc', '--bus', 'U3:U4',
                            '--clearance-ceiling', '0.1', '--track-width', '0.1',
                            '--via-size', '0.4', '--via-drill', '0.2'],
                           capture_output=True, text=True, encoding='utf-8', errors='replace', cwd=ROOT)
        text = r.stdout + r.stderr
        assert r.returncode == 0, (r.returncode, text[-1500:])
        bus_line = [ln for ln in text.splitlines() if ln.startswith('BUS ')]
        assert bus_line, text[-1500:]
        m = re.search(r'walled=(\d+)', bus_line[-1])
        assert m, f'no walled= on the BUS line: {bus_line[-1]}'
        assert int(m.group(1)) == 1, bus_line[-1]
        assert 'WALLED IN by the step' in text and 'U4.H9' in text, text[-1500:]
        summary = os.path.join(tmp, 'bus', 'oc_bus.bus', 'U3-U4.summary.json')
        evidence(summary, 'the bus summary')
        with open(summary, encoding='utf-8') as f:
            walled = json.load(f)['walled']
        assert walled['walled'] == ['U4.H9'], walled


def test_unchanged_board_walls_nothing():
    """a board graded against itself walls nothing: a ball is named only when the step made it unreachable"""
    sys.path.insert(0, os.path.join(ROOT, 'awx'))
    sys.path.insert(0, os.path.join(ROOT, 'py_router'))
    import route_bus
    from kicad_parser import parse_kicad_pcb
    pcb = parse_kicad_pcb(BOARD)
    nets3 = {p.net_name for p in pcb.footprints['U3'].pads if p.net_name}
    nets4 = {p.net_name for p in pcb.footprints['U4'].pads if p.net_name}
    names = sorted(nets3 & nets4)
    assert names, 'the arrays share no net'
    got = route_bus.walled_grade(BOARD, BOARD, ['U3', 'U4'], names, log=lambda *_: None)
    assert got['balls'] > 0 and got['walled'] == [], got


if __name__ == '__main__':
    test_unchanged_board_walls_nothing()
    print('control ok')
    test_walled_ball_is_named()
    print('ok')
