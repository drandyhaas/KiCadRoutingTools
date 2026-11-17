"""
KiCad Routing Tools - a CLI script run as a child of KiCad's process.

The Autorouter and Bus tabs run a CLI (autoroute.py, awx/route_bus.py) on a
SNAPSHOT of the open board, in a work folder of their own, and read what it
prints while it runs. The pieces both use:

  snapshot_board   the live board saved to WORKDIR/input.kicad_pcb, with the
                   board file's project siblings (.kicad_pro, .kicad_dru, ...)
  signal_group     a signal to the child's whole process group (every stage
                   of the CLI runs under it); taskkill /T on Windows
  ChildProcess     Popen in its own process group, a reader thread queueing
                   its output (kept whole in a log file), drained by the
                   tab's wx timer; Stop is SIGTERM, then SIGKILL

Nothing here imports wx: the tabs own the UI.
"""

import os
import queue
import shutil
import signal
import subprocess
import threading
import time


def snapshot_board(workdir, board_filename, name='input.kicad_pcb'):
    """Write the open board to WORKDIR/NAME with its project siblings;
    returns the path, or raises. The live board (pcbnew.GetBoard) when there
    is one, else the board file."""
    staged = os.path.join(workdir, name)
    try:
        from copy_board import SIBLING_EXTS
    except Exception:                                       # noqa: BLE001
        SIBLING_EXTS = ('.kicad_pro', '.kicad_prl', '.kicad_dru',
                        '.design-brief.json')
    board = None
    try:
        import pcbnew
        board = pcbnew.GetBoard()
    except Exception:                                       # noqa: BLE001
        board = None
    if board is not None:
        # aSkipSettings: a snapshot; writing a .kicad_pro beside it crashes
        # on pre-KiCad-10 projects (see swig_gui._stage_live_board). The
        # siblings come from the board's own files, as the AI tabs stage.
        import pcbnew
        # tool_records.save_board keeps the record on the file (#1906)
        import tool_records
        tool_records.save_board(pcbnew.SaveBoard, staged, board,
                                aSkipSettings=True)
    elif board_filename and os.path.isfile(board_filename):
        shutil.copyfile(board_filename, staged)
    else:
        raise RuntimeError("no open board and no board file to route")
    if board_filename:
        stem = os.path.splitext(os.path.abspath(board_filename))[0]
        base = os.path.splitext(staged)[0]
        for ext in SIBLING_EXTS:
            if os.path.isfile(stem + ext):
                shutil.copyfile(stem + ext, base + ext)
    return staged


def signal_group(proc, sig):
    """SIG to PROC's whole process group (None on Windows: taskkill /T /F)."""
    if proc is None or proc.poll() is not None:
        return
    try:
        if os.name == 'nt':
            subprocess.run(['taskkill', '/T', '/F', '/PID', str(proc.pid)],
                           capture_output=True)
        else:
            os.killpg(os.getpgid(proc.pid), sig)
    except (OSError, ProcessLookupError):
        pass


class ChildProcess:
    """One CLI run: ARGV in CWD with ENV, its stdout and stderr together,
    line by line, kept whole in LOG_PATH. `drain()` from the UI thread;
    `stop()` then `tick()` from its timer."""

    def __init__(self, argv, cwd, env=None, log_path=None):
        self.argv = list(argv)
        self.cwd = cwd
        self.env = env
        self.log_path = log_path
        self.proc = None
        self.returncode = None
        self.stopped = False
        self._queue = queue.Queue()
        self._kill_at = None
        self._done = False

    def start(self):
        kw = {}
        if os.name == 'nt':
            kw['creationflags'] = subprocess.CREATE_NEW_PROCESS_GROUP
        else:
            kw['start_new_session'] = True
        self.proc = subprocess.Popen(
            self.argv, cwd=self.cwd, env=self.env, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True, encoding='utf-8',
            errors='replace', bufsize=1, **kw)
        threading.Thread(target=self._read, daemon=True).start()
        return self

    def _read(self):
        log = None
        try:
            if self.log_path:
                log = open(self.log_path, 'w', encoding='utf-8')
        except OSError:
            log = None
        try:
            for line in self.proc.stdout:
                if log:
                    log.write(line)
                    log.flush()
                self._queue.put(line.rstrip('\n'))
        finally:
            if log:
                log.close()
            self.proc.wait()
            self._queue.put(None)

    def drain(self):
        """([new output lines], whether the child has ended and its output
        is all read)."""
        out = []
        while True:
            try:
                line = self._queue.get_nowait()
            except queue.Empty:
                break
            if line is None:
                self._done = True
                self.returncode = self.proc.returncode
                break
            out.append(line)
        return out, self._done

    def running(self):
        return self.proc is not None and not self._done

    def stop(self):
        """SIGTERM to the group now, SIGKILL three seconds on (tick)."""
        if self.proc is None or self._done:
            return
        self.stopped = True
        signal_group(self.proc, signal.SIGTERM if os.name != 'nt' else None)
        self._kill_at = time.time() + 3.0

    def tick(self):
        if self._kill_at is not None and time.time() > self._kill_at:
            self._kill_at = None
            if os.name != 'nt':
                signal_group(self.proc, signal.SIGKILL)

    def kill(self):
        signal_group(self.proc, signal.SIGKILL if os.name != 'nt' else None)
