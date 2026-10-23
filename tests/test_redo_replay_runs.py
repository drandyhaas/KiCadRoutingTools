"""redo_stress_test.py actually REPLAYS a manifest, on every platform.

Its run loop imported `resource` for per-command CPU, which exists only on
POSIX, so on Windows every real replay died on ModuleNotFoundError before the
first command -- only `--dry-run` worked. Its timeout path called `pgrep`,
which Windows lacks too, and killing only the wrapper there would leave the
command's own children running.

Three things are asserted on a tiny two-command manifest and a sleeping one:
  1. a real replay runs both commands in order and exits 0;
  2. --timings-out records them, with `cpu_seconds` exactly where getrusage
     exists (absent, not a made-up 0, where it does not);
  3. a command past --timeout is killed WITH its child: the child's delayed
     write never lands.
"""
import json
import os
import shlex
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from run_utils import check, evidence  # noqa: E402

REDO = os.path.join(HERE, "stress", "redo_stress_test.py")
try:
    import resource  # noqa: F401
    HAVE_RUSAGE = True
except ImportError:
    HAVE_RUSAGE = False


def _manifest(d, lines):
    path = os.path.join(d, "redo_commands.sh")
    with open(path, "w", encoding="utf-8") as f:
        f.write("set -e\n" + "\n".join(lines) + "\n")
    return path


def test_replay_runs_and_times():
    with tempfile.TemporaryDirectory() as d:
        man = _manifest(d, [
            "python3 -X utf8 -c \"open('a.txt', 'w').write('replayed')\"",
            "python3 -X utf8 -c \"open('b.txt', 'w').write(open('a.txt').read())\"",
        ])
        work = os.path.join(d, "work")
        timings = os.path.join(d, "timings.json")
        check([sys.executable, REDO, man, "--verbatim", "--skip-validate",
               "--workdir", work, "--timings-out", timings], accept=True)
        evidence(os.path.join(work, "b.txt"), "the second command's output")
        assert open(os.path.join(work, "b.txt")).read() == "replayed"
        evidence(timings, "the timings file")
        cmds = json.load(open(timings))["commands"]
        assert [c["returncode"] for c in cmds] == [0, 0], cmds
        for c in cmds:
            assert ("cpu_seconds" in c) == HAVE_RUSAGE, (HAVE_RUSAGE, c)
    print(f"PASS: replay ran both commands; cpu_seconds "
          f"{'recorded' if HAVE_RUSAGE else 'absent (no getrusage)'}")


def test_timeout_kills_the_tree():
    with tempfile.TemporaryDirectory() as d:
        late = os.path.join(d, "work", "late.txt").replace("\\", "/")
        # The wrapper starts a child that writes `late` after 6 s, then waits.
        child = f"import time; time.sleep(6); open('{late}', 'w').write('alive')"
        parent = ("import subprocess, sys; "
                  f"subprocess.Popen([sys.executable, '-c', {child!r}]).wait()")
        man = _manifest(d, ["python3 -X utf8 -c " + shlex.quote(parent)])
        t0 = time.time()
        check([sys.executable, REDO, man, "--verbatim", "--skip-validate",
               "--workdir", os.path.join(d, "work"), "--timeout", "2"],
              refuse="TIMED OUT", code=2)
        took = time.time() - t0
        assert took < 6, f"the replay waited out the command ({took:.1f}s)"
        time.sleep(8)  # past the child's write, had it survived
        assert not os.path.exists(late), "the timed-out command's child survived"
    print(f"PASS: timeout killed the command and its child ({took:.1f}s)")


if __name__ == "__main__":
    test_replay_runs_and_times()
    test_timeout_kills_the_tree()
