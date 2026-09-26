#!/usr/bin/env python3
"""stage_cache.py --out FILE [--out FILE ...] -- SCRIPT ARG... -- run one whole-route stage, or restore its outputs.

A stage's outputs are a function of its script, its arguments, its environment, and every file it reads -- the code it
imports and the data it opens. The first run records all of that, by content, beside its outputs and its console log:
the script, the arguments (a named file by its content; a declared output by its place in the list, so the same stage
into another directory is the same stage), the environment (a file a variable names by its content, the session's own
variables aside), every module file the stage loaded, every other file it opened for reading, and every path it looked
for and did not find. A later run whose script, arguments and environment match restores the outputs and replays the
log when every recorded file is unchanged and every missing one still missing, and runs the stage otherwise. Nothing
is guessed: a change to anything the stage read, or a file appearing where it looked, runs it again. A stage that fails
is not recorded, nor one a file it read changed under while it ran (its content by then is not what the stage read);
an entry's meta is written last and whole, so an interrupted recording is never restored. What it cannot see: a
directory listing, a subprocess's reads.

    python3 stage_cache.py --out g1.json -- whole_geo.py solve.json g1.json

OFF BY DEFAULT: STAGE_CACHE=1 turns it on (a test harness redoing the same bench -- whole_loop.sh sets it); without it
the stage runs and nothing is recorded. STAGE_CACHE_DIR is where entries live (default awx/tmp/stage_cache).
"""
import atexit
import contextlib
import hashlib
import io
import json
import os
import runpy
import shutil
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
CACHE = os.environ.get('STAGE_CACHE_DIR') or os.path.join(HERE, 'tmp', 'stage_cache')
# variables that change from one shell (or agent session, or terminal) to the next and name nothing a stage reads --
# with them in the key a new session restored nothing, and one named the agent's 217 MB binary, hashed every stage
VOLATILE = {'_', 'OLDPWD', 'PWD', 'SHLVL', 'TERM_SESSION_ID', 'SECURITYSESSIONID', 'COLUMNS', 'LINES',
            'STAGE_CACHE', 'STAGE_CACHE_DIR', 'SSH_AUTH_SOCK', 'LaunchInstanceID', 'AI_AGENT', 'COLORTERM',
            'OSLogRateLimit', 'GIT_EDITOR'}
VOLATILE_PREFIX = ('CLAUDE', '__CF', 'XPC_', 'TERM_PROGRAM')


def enabled():
    """the cache is ON only when asked (STAGE_CACHE=1): it serves a harness that runs the same stages on the same inputs
    again and again, and would otherwise fill a user's disk beside the code"""
    return os.environ.get('STAGE_CACHE', '0') not in ('', '0')


def file_sha(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


# the Python installation's own files (the interpreter's library, NumPy, SciPy, OR-tools) are recorded by size and
# time, as a build tool records them: hashing their binaries on every check would cost seconds; the repository's
# files and the data are recorded by content
INSTALL = tuple(sorted({os.path.realpath(p) + os.sep for p in (sys.prefix, sys.base_prefix, sys.exec_prefix)}))


def signature(path):
    rp = os.path.realpath(path)
    if rp.startswith(INSTALL) and not rp.startswith(REPO + os.sep):
        st = os.stat(rp)
        return f'stat:{st.st_size}:{st.st_mtime_ns}'
    return file_sha(path)


def named(value):
    """a value as the key sees it: the files it names (itself, @itself, or a comma list of them) by their content"""
    parts = []
    for piece in value.split(','):
        p = piece[1:] if piece.startswith('@') else piece
        parts.append(['file', file_sha(p)] if p and os.path.isfile(p) else ['text', piece])
    return parts


def env_key(leave_out=()):
    """the environment as a key sees it: every variable but the session's own (and leave_out), a file it names by
    its content"""
    return {k: named(v) for k, v in sorted(os.environ.items())
            if k not in VOLATILE and k not in leave_out and not k.startswith(VOLATILE_PREFIX)}


def modules():
    """the files of every module this process has loaded"""
    return {os.path.abspath(m.__file__) for m in list(sys.modules.values())
            if isinstance(getattr(m, '__file__', None), str)}


# ---- what a run READ: every file it opened for reading, every path it looked for and did not find (a file that
# appears later changes what it does as surely as one that changes: a board's plan sidecar, its rules file), and
# every module it loaded (its source: an edited module outside the repository keeps its old .pyc until it is imported)
_RECORDERS = []


def _record_open(event, a):
    if not _RECORDERS or event != 'open' or not a or not isinstance(a[0], (str, bytes, os.PathLike)):
        return
    mode = a[1] if len(a) > 1 and isinstance(a[1], str) else 'r'
    if 'r' in mode or ('+' in mode and 'w' not in mode):
        p = os.path.abspath(os.fsdecode(a[0]))
        for r_ in _RECORDERS:
            r_['read'].add(p)


sys.addaudithook(_record_open)


@contextlib.contextmanager
def recording():
    """{'read', 'absent', 't0'} of what runs inside: files opened for reading, paths probed and not found (os.stat,
    which os.path.exists / isfile and pathlib ask)"""
    rec = {'read': set(), 'absent': set(), 't0': time.time_ns()}
    _RECORDERS.append(rec)
    real_stat = os.stat

    def stat(path, *a, **k):
        try:
            return real_stat(path, *a, **k)
        except FileNotFoundError:
            if isinstance(path, (str, bytes, os.PathLike)):
                p = os.path.abspath(os.fsdecode(path))
                for r_ in _RECORDERS:
                    r_['absent'].add(p)
            raise
    os.stat = stat
    try:
        yield rec
    finally:
        os.stat = real_stat
        _RECORDERS.remove(rec)


def evidence(rec, skip=()):
    """a recording as an entry keeps it -- {'read': {path: signature}, 'absent': [path]}: every module loaded and
    every file read (less skip, a stage's own outputs, and this cache) by content, every path looked for and not
    found; None when a file it read changed while it ran (its content NOW is not what it read)"""
    skip = {os.path.abspath(p) for p in skip}
    keep = lambda p: p not in skip and not p.startswith(CACHE + os.sep)
    files = {p for p in (modules() | rec['read']) if keep(p) and os.path.isfile(p)}
    if any(os.stat(p).st_mtime_ns >= rec['t0'] for p in files):
        return None
    absent = sorted(p for p in (rec['absent'] | rec['read']) if keep(p) and not os.path.exists(p))
    return {'read': {p: signature(p) for p in sorted(files)}, 'absent': absent}


def still(ev):
    """every file of an entry's evidence as it was, and every path it did not find still absent"""
    return (all(os.path.isfile(p) and signature(p) == h for p, h in ev['read'].items())
            and not any(os.path.exists(p) for p in ev.get('absent', ())))


def stage_key(script, sargs, outs):
    outs_ = [os.path.abspath(o) for o in outs]
    args = [['out', outs_.index(os.path.abspath(a))] if os.path.abspath(a) in outs_ else named(a) for a in sargs]
    env = env_key()
    blob = json.dumps({'script': file_sha(script), 'name': os.path.basename(script), 'args': args, 'env': env},
                      sort_keys=True)
    return hashlib.sha256(blob.encode()).hexdigest()


class Tee(io.TextIOBase):
    def __init__(self, real, copy):
        self.real, self.copy = real, copy

    def write(self, s):
        self.copy.write(s)
        return self.real.write(s)

    def flush(self):
        self.real.flush()


def run(script, sargs):
    """the stage in this process, as `python3 SCRIPT ARG...` runs it: its exit code"""
    sys.argv = [script] + list(sargs)
    sys.path.insert(0, os.path.dirname(os.path.abspath(script)))
    try:
        runpy.run_path(script, run_name='__main__')
        return 0
    except SystemExit as e:
        return e.code if isinstance(e.code, int) else (0 if e.code is None else 1)


def main():
    args = sys.argv[1:]
    outs = []
    while args and args[0] == '--out':
        outs.append(args[1])
        args = args[2:]
    if not args or args[0] != '--' or len(args) < 2:
        sys.exit(__doc__)
    script, sargs = args[1], args[2:]
    if not enabled():
        sys.exit(run(script, sargs))
    key = stage_key(script, sargs, outs)
    entry = os.path.join(CACHE, key[:24])
    meta_p = os.path.join(entry, 'meta.json')
    if os.path.isfile(meta_p):
        meta = json.load(open(meta_p))
        if still(meta):
            for i, o in enumerate(outs):
                shutil.copyfile(os.path.join(entry, f'out{i}'), o)
            sys.stdout.write(open(os.path.join(entry, 'log')).read())
            sys.stdout.write(f'stage_cache: {os.path.basename(script)} restored ({len(meta["read"])} files unchanged)\n')
            sys.exit(0)
    log = io.StringIO()
    real, real_err = sys.stdout, sys.stderr
    sys.stdout, sys.stderr = Tee(real, log), Tee(real_err, log)
    try:
        with recording() as rec:
            rc = run(script, sargs)
            atexit._run_exitfuncs()             # what the stage prints on its way out belongs to its log too
    finally:
        sys.stdout.flush(); sys.stderr.flush()
        sys.stdout, sys.stderr = real, real_err
    if rc != 0 or not all(os.path.isfile(o) for o in outs):
        sys.exit(rc or 1)
    ev = evidence(rec, skip=outs)
    if ev is None:              # a file it read changed while it ran: the outputs stand, the next run runs it again
        print(f'stage_cache: {os.path.basename(script)} not recorded -- a file it read changed while it ran',
              file=sys.stderr)
        sys.exit(0)
    # the entry: its old meta out first, so a half-written entry is never restored; the outputs and the log; the meta
    # last, whole
    os.makedirs(entry, exist_ok=True)
    if os.path.exists(meta_p):
        os.remove(meta_p)
    for i, o in enumerate(outs):
        shutil.copyfile(o, os.path.join(entry, f'out{i}'))
    open(os.path.join(entry, 'log'), 'w').write(log.getvalue())
    json.dump({'script': script, 'args': sargs, **ev}, open(meta_p + '.tmp', 'w'), indent=0)
    os.replace(meta_p + '.tmp', meta_p)
    sys.exit(0)


if __name__ == '__main__':
    main()
