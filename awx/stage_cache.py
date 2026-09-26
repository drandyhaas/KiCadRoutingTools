#!/usr/bin/env python3
"""stage_cache.py --out FILE [--out FILE ...] -- SCRIPT ARG... -- run one whole-route stage, or restore its outputs.

A stage's outputs are a function of its script, its arguments, its environment, and every file it reads -- the code it
imports and the data it opens. The first run records all of that, by content, beside its outputs and its console log:
the script, the arguments (a named file by its content; a declared output by its place in the list, so the same stage
into another directory is the same stage), the environment (a file a variable names by its content), every module
file of this repository the stage loaded, and every other file it opened for reading. A later run whose script,
arguments and environment match restores the outputs and replays the log when every recorded file is unchanged, and
runs the stage otherwise. Nothing is guessed: a change to anything the stage read runs it again. A stage that fails is
not recorded, nor one a file it read changed under while it ran (its content by then is not what the stage read).

    python3 stage_cache.py --out g1.json -- whole_geo.py solve.json g1.json

STAGE_CACHE=0 runs the stage and records nothing; STAGE_CACHE_DIR is where entries live (default awx/tmp/stage_cache).
"""
import atexit
import hashlib
import io
import json
import os
import runpy
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
CACHE = os.environ.get('STAGE_CACHE_DIR') or os.path.join(HERE, 'tmp', 'stage_cache')
# variables that change from one shell (or agent session, or terminal) to the next and name nothing a stage reads --
# with them in the key a new session restored nothing, and one named the agent's 217 MB binary, hashed every stage
VOLATILE = {'_', 'OLDPWD', 'PWD', 'SHLVL', 'TERM_SESSION_ID', 'SECURITYSESSIONID', 'COLUMNS', 'LINES',
            'STAGE_CACHE', 'STAGE_CACHE_DIR', 'SSH_AUTH_SOCK', 'LaunchInstanceID', 'AI_AGENT', 'COLORTERM',
            'OSLogRateLimit', 'GIT_EDITOR'}
VOLATILE_PREFIX = ('CLAUDE', '__CF', 'XPC_', 'TERM_PROGRAM')


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


def repo_modules():
    """the files of every module of this repository this process has loaded"""
    return {os.path.abspath(m.__file__) for m in list(sys.modules.values())
            if getattr(m, '__file__', None) and os.path.abspath(m.__file__).startswith(REPO + os.sep)}


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


def run(script, sargs, record=None):
    """the stage in this process, as `python3 SCRIPT ARG...` runs it: its exit code, and (record) the files it read"""
    sys.argv = [script] + list(sargs)
    sys.path.insert(0, os.path.dirname(os.path.abspath(script)))
    if record is not None:
        def hook(event, a):
            if event == 'open' and a and isinstance(a[0], (str, bytes, os.PathLike)):
                mode = a[1] if len(a) > 1 and isinstance(a[1], str) else 'r'
                if not any(c in mode for c in 'wax+'):
                    record.add(os.path.abspath(os.fsdecode(a[0])))
        sys.addaudithook(hook)
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
    if os.environ.get('STAGE_CACHE', '1') == '0':
        sys.exit(run(script, sargs))
    key = stage_key(script, sargs, outs)
    entry = os.path.join(CACHE, key[:24])
    meta_p = os.path.join(entry, 'meta.json')
    if os.path.isfile(meta_p):
        meta = json.load(open(meta_p))
        if all(os.path.isfile(p) and signature(p) == h for p, h in meta['read'].items()):
            for i, o in enumerate(outs):
                shutil.copyfile(os.path.join(entry, f'out{i}'), o)
            sys.stdout.write(open(os.path.join(entry, 'log')).read())
            sys.stdout.write(f'stage_cache: {os.path.basename(script)} restored ({len(meta["read"])} files unchanged)\n')
            sys.exit(0)
    read = set()
    t_start = __import__('time').time_ns()
    log = io.StringIO()
    real, real_err = sys.stdout, sys.stderr
    sys.stdout, sys.stderr = Tee(real, log), Tee(real_err, log)
    try:
        rc = run(script, sargs, record=read)
        atexit._run_exitfuncs()                 # what the stage prints on its way out belongs to its log too
    finally:
        sys.stdout.flush(); sys.stderr.flush()
        sys.stdout, sys.stderr = real, real_err
    if rc != 0 or not all(os.path.isfile(o) for o in outs):
        sys.exit(rc or 1)
    # what the stage read: every module of this repository it loaded, and every other file it opened -- less its
    # own outputs and this cache
    mods = repo_modules()
    outs_ = {os.path.abspath(o) for o in outs}
    files = {p for p in (mods | read) if os.path.isfile(p) and p not in outs_ and not p.startswith(CACHE + os.sep)}
    # a file changed while the stage ran is recorded by its content NOW, which is not what the stage read: record
    # nothing (the outputs stand; the next run runs it again)
    moved = sorted(p for p in files if os.stat(p).st_mtime_ns >= t_start)
    if moved:
        print(f'stage_cache: {os.path.basename(script)} not recorded -- {len(moved)} file(s) it read changed while it '
              f'ran: {", ".join(os.path.relpath(p, REPO) for p in moved[:3])}', file=sys.stderr)
        sys.exit(0)
    os.makedirs(entry, exist_ok=True)
    for i, o in enumerate(outs):
        shutil.copyfile(o, os.path.join(entry, f'out{i}'))
    open(os.path.join(entry, 'log'), 'w').write(log.getvalue())
    json.dump({'script': script, 'args': sargs, 'read': {p: signature(p) for p in sorted(files)}},
              open(meta_p, 'w'), indent=0)
    sys.exit(0)


if __name__ == '__main__':
    main()
