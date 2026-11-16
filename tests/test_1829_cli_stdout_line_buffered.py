#!/usr/bin/env python3
"""#1829: chain and long-running CLIs deliver each line to a pipe when it is printed.

#1825 line-buffered place_simple and judge_seeds. place_fanout_clearance, a
chain step, still printed through Python's 8 KB block buffer when stdout was
a pipe or a log: every line reached the reader when the run exited, and a run
killed part way left none of its stdout behind. run_plan, place_movie and the
two fanout fronts (bga_fanout, qfn_fanout, which print no CMD/EXIT banner)
did the same.

Checks:
  1. place_fanout_clearance on a five-BGA board, stdout a pipe: the
     `Loading` line is read before the output board exists (the pass writes
     its board last);
  2. a run of the same command killed once that line has been read leaves
     the line in its log file;
  3. each tool, started as its own script with stdout a pipe and `--help`,
     has stdout line-buffered by the time argparse exits -- so the streams
     are set before the tool prints anything.
"""
import os
import subprocess
import sys
import tempfile
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
from run_utils import evidence, tool                           # noqa: E402
from copy_board import copy_board                              # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'orangecrab_ext_pll.kicad_pcb')
WATCH = 'Loading'

# Started as a script, each must line-buffer before argparse runs.
TOOLS = ('place_fanout_clearance.py', 'run_plan.py', 'place_movie.py',
         'bga_fanout.py', 'qfn_fanout.py')

PROBE = """import os, runpy, sys
tool = sys.argv[1]
sys.path.insert(0, os.path.dirname(os.path.abspath(tool)))   # as a script run gets
sys.argv = [tool, '--help']
try:
    runpy.run_path(tool, run_name='__main__')
except SystemExit:
    pass
sys.stderr.write('LINE_BUFFERED=%s\\n' % sys.stdout.line_buffering)
"""


def cap_pass_argv(board, out):
    return [sys.executable, '-X', 'utf8', tool('place_fanout_clearance.py'), board, out]


def test_live_pipe():
    evidence(BOARD, 'the fixture board')
    with tempfile.TemporaryDirectory() as td:
        board = os.path.join(td, 'in.kicad_pcb')
        copy_board(BOARD, board)
        evidence(board, 'the copied board')
        out = os.path.join(td, 'out.kicad_pcb')
        proc = subprocess.Popen(cap_pass_argv(board, out), cwd=ROOT,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                text=True, encoding='utf-8', errors='replace')
        lines, seen = [], {}

        def reader():
            for line in proc.stdout:
                if WATCH in line and 'existed' not in seen:
                    seen['existed'] = os.path.exists(out)
                lines.append(line.rstrip('\n'))
        t = threading.Thread(target=reader, daemon=True)
        t.start()
        try:
            proc.wait(timeout=600)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
        t.join(timeout=30)
        text = '\n'.join(lines)
        assert proc.returncode == 0 and 'Traceback' not in text, (proc.returncode, text[-1500:])
        evidence(out, 'the cap pass output')
        assert 'existed' in seen, f'no {WATCH!r} line arrived:\n{text[-1500:]}'
        assert seen['existed'] is False, (f'the {WATCH!r} line reached the pipe only after '
                                          'the pass had written its board')
    print(f'  PASS: the {WATCH!r} line reached the pipe before the output board was written')


def test_killed_run_keeps_its_lines():
    with tempfile.TemporaryDirectory() as td:
        board = os.path.join(td, 'in.kicad_pcb')
        copy_board(BOARD, board)
        evidence(board, 'the copied board')
        out = os.path.join(td, 'out.kicad_pcb')
        log = os.path.join(td, 'run.log')
        with open(log, 'w') as fh:
            proc = subprocess.Popen(cap_pass_argv(board, out), cwd=ROOT,
                                    stdout=fh, stderr=subprocess.STDOUT)
            # kill as soon as the line is in the log, or after 60 s: with a
            # block-buffered stdout the line never reaches the file before exit
            t0 = time.monotonic()
            while proc.poll() is None and time.monotonic() - t0 < 60:
                with open(log, encoding='utf-8', errors='replace') as rd:
                    if WATCH in rd.read():
                        break
                time.sleep(0.05)
            finished = proc.poll() is not None
            proc.terminate()
            proc.wait()
        evidence(log, 'the run log')
        text = open(log, encoding='utf-8', errors='replace').read()
        assert not finished, (f'the {WATCH!r} line reached the log only when the run '
                              f'exited, so a run killed earlier keeps none of it:\n{text[-1500:]}')
        assert WATCH in text, f'the killed run left no {WATCH!r} line in its log:\n{text[-1500:]}'
        assert not os.path.exists(out), 'the run had already written its board when killed'
    print(f'  PASS: a run killed after printing {WATCH!r} keeps that line in its log')


def test_streams_set_before_argparse():
    with tempfile.TemporaryDirectory() as td:
        probe = os.path.join(td, 'probe.py')
        with open(probe, 'w') as f:
            f.write(PROBE)
        evidence(probe, 'the probe script')
        bad = []
        for name in TOOLS:
            path = tool(name)
            r = subprocess.run([sys.executable, '-X', 'utf8', probe, path], cwd=ROOT,
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                               text=True, encoding='utf-8', errors='replace', timeout=300)
            got = [ln for ln in r.stderr.splitlines() if ln.startswith('LINE_BUFFERED=')]
            assert got, f'{name}: the probe never reported (a broken test):\n{r.stderr[-1500:]}'
            assert 'usage' in r.stdout.lower(), f'{name}: --help printed no usage:\n{r.stdout[-800:]}'
            if got[-1] != 'LINE_BUFFERED=True':
                bad.append(name)
        assert not bad, f'stdout to a pipe is not line-buffered in: {", ".join(bad)}'
    print(f'  PASS: {len(TOOLS)} tools line-buffer stdout to a pipe before argparse runs')


if __name__ == '__main__':
    test_live_pipe()
    test_killed_run_keeps_its_lines()
    test_streams_set_before_argparse()
    print('ALL PASS')
