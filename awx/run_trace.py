"""run_trace -- what a run SAVES for a person to read afterwards: which code ran, and the stages' own records.

provenance(OUT): OUT/run.json -- the code's commit, a hash of its uncommitted change (the change itself in
OUT/code.patch), the command, the awx settings in force, the host and the start -- so that two runs compared are
known to be the code they are said to be (a run of a scratch copy of the tree, deleted since, could not be).

Trace(PATH): a JSON-lines record of a stage's own steps, one dict a line, each with `v` (the record's schema) and `t`
(seconds since the trace was opened) -- read by bus_trace.py. Writing never fails a run: a record that cannot be
written is dropped with a line on stderr."""
import hashlib
import json
import os
import socket
import subprocess
import sys
import time

V = 1
HERE = os.path.dirname(os.path.abspath(__file__))


def _git(*a):
    """git in the tree's root (the pathspecs given are the root's: awx, py_router)"""
    p = subprocess.run(['git', '-C', os.path.dirname(HERE)] + list(a), capture_output=True, text=True)
    return p.stdout if p.returncode == 0 else None


def provenance(out, extra=None):
    """OUT/run.json and OUT/code.patch (see the module's docstring); the record written, or None"""
    try:
        import awx_settings
        os.makedirs(out, exist_ok=True)
        head = (_git('rev-parse', 'HEAD') or '').strip() or None
        diff = _git('diff', 'HEAD', '--', 'awx', 'py_router') or ''
        untracked = [f for f in (_git('ls-files', '--others', '--exclude-standard', '--', 'awx', 'py_router') or '')
                     .splitlines() if f.endswith('.py')]
        with open(os.path.join(out, 'code.patch'), 'w', encoding='utf-8') as f:
            f.write(diff)
            for u_ in untracked:            # (a new file is in no diff: written whole, as git would add it)
                try:
                    txt_ = open(os.path.join(os.path.dirname(HERE), u_), encoding='utf-8').read()
                except OSError:
                    continue
                f.write(f'diff --git a/{u_} b/{u_}\nnew file mode 100644\n--- /dev/null\n+++ b/{u_}\n'
                        f'@@ -0,0 +1,{txt_.count(chr(10))} @@\n' + ''.join('+' + ln for ln in txt_.splitlines(True)))
                diff += txt_
        rec = {'v': V, 'head': head, 'tree': os.path.dirname(HERE),
               'diff_sha1': hashlib.sha1(diff.encode()).hexdigest() if diff else None,
               'diff_lines': diff.count('\n'), 'untracked_py': untracked, 'argv': list(sys.argv),
               'settings': {k: v for k, v in sorted(awx_settings.environ().items()) if k.isupper()
                            and k not in ('PATH', 'HOME', 'PWD', 'OLDPWD', 'SHELL', 'TMPDIR', 'USER', 'LOGNAME')
                            and not k.startswith(('LC_', 'XPC_', 'TERM', '__CF', 'SSH_', 'CLAUDE', 'NODE_'))
                            and not any(s_ in k for s_ in ('KEY', 'TOKEN', 'SECRET', 'PASSWORD', 'AUTH', 'CRED'))},
               'host': socket.gethostname(), 'start': time.strftime('%Y-%m-%dT%H:%M:%S'), **(extra or {})}
        with open(os.path.join(out, 'run.json'), 'w', encoding='utf-8') as f:
            json.dump(rec, f, indent=1)
        return rec
    except Exception as e:                  # (a record, never a reason to stop the run)
        print(f'run_trace: no run.json in {out}: {e!r}', file=sys.stderr)
        return None


def jsonable(x):
    """`x` with sets, tuples and frozensets as sorted lists, dict keys as strings: what json writes"""
    if isinstance(x, dict):
        return {(k if isinstance(k, str) else '|'.join(map(str, k)) if isinstance(k, (tuple, frozenset)) else str(k)):
                jsonable(v) for k, v in x.items()}
    if isinstance(x, (set, frozenset)):
        return sorted((jsonable(v) for v in x), key=repr)
    if isinstance(x, (list, tuple)):
        return [jsonable(v) for v in x]
    if isinstance(x, float):
        return round(x, 4)
    if isinstance(x, (str, int, bool)) or x is None:
        return x
    try:
        return float(x) if not float(x).is_integer() else int(x)
    except (TypeError, ValueError):
        return str(x)


class Trace:
    """a JSON-lines record at `path` (see the module's docstring)"""

    def __init__(self, path, fresh=True):
        self.path = path
        self.t0 = time.time()
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        if fresh and os.path.exists(path):
            os.remove(path)

    def rec(self, rec, /, **fields):
        """a record of type `rec` (its 'rec' key) with `fields`"""
        try:
            line = json.dumps({'v': V, 't': round(time.time() - self.t0, 2), 'rec': rec, **jsonable(fields)})
            with open(self.path, 'a', encoding='utf-8') as f:
                f.write(line + '\n')
        except Exception as e:
            print(f'run_trace: a {rec} record not written to {self.path}: {e!r}', file=sys.stderr)
