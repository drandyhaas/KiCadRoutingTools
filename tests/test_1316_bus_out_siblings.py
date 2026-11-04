"""#1316: the bus step's OUT carries the board's own siblings.

route_bus handed OUT on with its .kicad_pro alone: the work boards on the way
(flow_frame's turn, the whole route's best round) copied only the project and
the ladder, so OUT lost the board's .kicad_dru -- KiCad and check_drc then
graded it without the length windows its rules declare -- and its design brief.

Two rows:
  * flow_frame.turn_file carries every copy_board.SIBLING_EXTS file (fast).
  * the step end to end (about 3 minutes, one routing process): OUT.kicad_dru is
    beside OUT, and check_drc on OUT reads the length rule's rows.
"""
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
for d in ('', 'py_router', 'py_tools', 'awx'):
    sys.path.insert(0, os.path.join(ROOT, d) if d else ROOT)
from run_utils import evidence

BOARD = os.path.join(ROOT, 'kicad_files', 'orangecrab_ext_pll.kicad_pcb')
DRU = '(version 1)\n(rule "ram"\n  (condition "A.NetName == \'RAM_*\'")\n  (constraint length (max 1mm)))\n'


def test_turn_file_carries_every_sibling():
    import flow_frame
    from copy_board import SIBLING_EXTS
    with tempfile.TemporaryDirectory() as tmp:
        src = os.path.join(tmp, 'a.kicad_pcb')
        shutil.copyfile(os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb'), src)
        for ext in tuple(SIBLING_EXTS) + ('.ladder.txt',):
            with open(os.path.join(tmp, 'a' + ext), 'w', encoding='utf-8') as f:
                f.write('x\n')
        dst = os.path.join(tmp, 'b.kicad_pcb')
        flow_frame.turn_file(src, dst, 1, 100.0, 100.0)
        missing = [e for e in tuple(SIBLING_EXTS) + ('.ladder.txt',)
                   if not os.path.isfile(os.path.join(tmp, 'b' + e))]
        assert not missing, f'turn_file dropped siblings {missing}'


def test_route_bus_out_keeps_the_rules_file():
    with tempfile.TemporaryDirectory() as tmp:
        oc = os.path.join(tmp, 'oc.kicad_pcb')
        evidence(BOARD, 'the input board')
        subprocess.run([sys.executable, '-B', '-X', 'utf8',
                        os.path.join(ROOT, 'tests', 'stress', 'strip_copper_only.py'), BOARD, oc],
                       check=True, capture_output=True, cwd=ROOT)
        evidence(oc, 'the stripped board')
        with open(os.path.join(tmp, 'oc.kicad_dru'), 'w', encoding='utf-8') as f:
            f.write(DRU)
        out = os.path.join(tmp, 'bus', 'oc_bus.kicad_pcb')
        r = subprocess.run([sys.executable, '-B', '-X', 'utf8', os.path.join(ROOT, 'awx', 'route_bus.py'),
                            oc, out, '--rounds', '1', '--inproc', '--bus', 'U3:U4',
                            '--clearance-ceiling', '0.1', '--track-width', '0.1',
                            '--via-size', '0.4', '--via-drill', '0.2'],
                           capture_output=True, text=True, encoding='utf-8', errors='replace', cwd=ROOT)
        assert r.returncode == 0, (r.returncode, (r.stdout + r.stderr)[-1500:])
        evidence(out, 'the bus step output')
        dru = out[:-len('.kicad_pcb')] + '.kicad_dru'
        assert os.path.isfile(dru), 'OUT.kicad_dru is not beside OUT'
        with open(dru, encoding='utf-8') as f:
            assert f.read() == DRU
        g = subprocess.run([sys.executable, '-B', '-X', 'utf8',
                            os.path.join(ROOT, 'py_router', 'check_drc.py'), out, '--clearance-margin', '0.1'],
                           capture_output=True, text=True, encoding='utf-8', errors='replace', cwd=ROOT)
        assert 'Length rules' in g.stdout, g.stdout[-800:]


if __name__ == '__main__':
    test_turn_file_carries_every_sibling()
    print('turn_file ok')
    test_route_bus_out_keeps_the_rules_file()
    print('ok')
