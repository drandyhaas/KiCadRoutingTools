#!/usr/bin/env python3
"""Instrumentation ONLY: run a command, tee it, record start/end/exit.

This exists because per-command timings are NOT recoverable after the fact.
`loop_driver.log` writes its `t` AFTER the stage function returns, so it
timestamps completions rather than start/end pairs -- and it logs only the
driver's own advisory invocations, never the placement/routing tool
subprocesses run in response to that advice. Without a record kept at call
time, "the three slowest individual tool invocations with their commands" has
no honest answer but "not recoverable". Run 21 hit exactly that and wrote this
wrapper into its work dir mid-run; it is promoted here so the next run can
just use it.

The clock here is DESCRIPTIVE and nothing else: `wall_s` is written to the
row and printed, and no code path in this repo compares it to a limit. There
is no budget, no cap and no timeout -- a command runs until it is done.

It writes NO poses and NO copper, reads no board, and takes no decisions. Say
so in the journal and to the cheat watcher anyway: a wrapper around the tools
is the shape a hand-rolled pose writer would take, so what it does is bounded
here and every argv it ever ran is in `cmd_timing.jsonl`.

    python3 -X utf8 tests/stress/tee_cmd.py [--workdir DIR] <label> -- <cmd> [args ...]

Appends one JSON row per invocation to `<workdir>/cmd_timing.jsonl` and tees
combined output to `<workdir>/logs/<label>.log` (also echoed to this process's
stdout so the caller sees it live). `--workdir` defaults to this file's own
directory, which is what a copy dropped into the work dir wants.
"""
import json
import os
import subprocess
import sys
import time


def main(argv):
    if '--' not in argv:
        raise SystemExit(
            'usage: tee_cmd.py [--workdir DIR] <label> -- <command> [args ...]')
    cut = argv.index('--')
    head, cmd = argv[:cut], argv[cut + 1:]
    if not cmd:
        raise SystemExit('tee_cmd: no command given after --')

    here = os.path.dirname(os.path.abspath(__file__))
    # BOTH spellings. `--workdir=DIR` is the form a shell writes without
    # thinking about it, and it was not recognised: the whole token fell
    # through into the label, so the rows landed in this file's own directory
    # under a label like `--workdir=wk/run24_p0`.
    for i, tok in enumerate(head):
        if tok == '--workdir':
            if i + 1 >= len(head):
                raise SystemExit('tee_cmd: --workdir needs a value')
            here, head = os.path.abspath(head[i + 1]), head[:i] + head[i + 2:]
            break
        if tok.startswith('--workdir='):
            val = tok.split('=', 1)[1]
            if not val:
                raise SystemExit('tee_cmd: --workdir needs a value')
            here, head = os.path.abspath(val), head[:i] + head[i + 1:]
            break
    label = '_'.join(head) or 'unlabelled'

    ledger = os.path.join(here, 'cmd_timing.jsonl')
    logdir = os.path.join(here, 'logs')
    os.makedirs(logdir, exist_ok=True)
    # A label reused across laps must not overwrite the earlier lap's log --
    # the timing table needs both rows, and an overwritten log is a stage that
    # silently cannot be timed.
    path = os.path.join(logdir, label + '.log')
    n = 1
    while os.path.exists(path):
        n += 1
        path = os.path.join(logdir, '%s.%d.log' % (label, n))

    # Run 24 (finding A-10): on a reused label the unsuffixed `<label>.done`
    # kept the OLDEST attempt's exit forever, while the stage advice says
    # "wait on logs/<label>.done" -- so a waiter armed during a retry
    # returned instantly with a stale verdict. The canonical marker now
    # DISAPPEARS while a retry runs and reappears with the newest code:
    # absent == in flight, present == the latest attempt's exit. The
    # numbered `<label>.N.done` files still pair 1:1 with their logs for
    # the timing table.
    canonical_done = os.path.join(logdir, label + '.done')
    if n > 1:
        try:
            os.remove(canonical_done)
        except OSError:
            pass

    def _mark_done(rc):
        """Write both completion markers. The only writer, and it always runs.

        `<label>.done` appearing IS the waiting protocol, and the canonical
        marker is removed up front on a retry -- so any path that fails to
        write it leaves "absent means still in flight" true forever, which is
        exactly the hang this file exists to prevent. A crash, or a command
        that never starts, must still produce a marker; the code inside it
        says what happened.

        On the first attempt the numbered and canonical paths are the same
        file, so this is one write; on a retry they differ and the canonical
        one carries the newest attempt's exit (last-attempt-wins).
        """
        done = path[:-4] + '.done' if path.endswith('.log') else path + '.done'
        for target in (done, canonical_done):
            try:
                with open(target, 'w', encoding='utf-8') as fh:
                    fh.write('%d\n' % rc)
            except OSError:
                pass

    t0 = time.time()
    with open(path, 'w', encoding='utf-8', errors='replace') as log:
        log.write('CMD: %s\n' % ' '.join(cmd))
        log.flush()
        try:
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT)
        except OSError as exc:
            # The command never started (not found, not executable). Without a
            # marker here a waiter blocks forever on a process that will never
            # exist.
            log.write('tee_cmd: could not start: %s\n' % exc)
            log.write('EXIT=127' + chr(10))
            log.flush()
            _mark_done(127)
            print('[tee_cmd] %s could not start: %s' % (label, exc))
            return 127
        # THE STREAMING LOOP IS INSIDE THE GUARD, not just `wait()`. A long
        # command spends almost all of its life in this loop, so a Ctrl-C
        # while the child is printing is the LIKELY interrupt, not the rare
        # one -- and unguarded it propagated out with no marker written,
        # leaving "absent means still in flight" permanently true. That is the
        # hang this file exists to prevent, in the path that runs 99% of the
        # time.
        try:
            for raw in proc.stdout:
                line = raw.decode('utf-8', 'replace')
                log.write(line)
                log.flush()
                sys.stdout.write(line)
                sys.stdout.flush()
            code = proc.wait()
        except BaseException:
            # A signal, a KeyboardInterrupt, an unexpected read error: write
            # the marker on the way out so the waiter is released rather than
            # left waiting on a marker that can no longer arrive. The child is
            # killed first, or it keeps writing into a log nobody reads.
            try:
                proc.kill()
            except Exception:                              # noqa: BLE001
                pass
            log.write('EXIT=-1' + chr(10))
            log.flush()
            _mark_done(-1)
            raise
        # The anchor contract run_watch documents is "one CMD: and one EXIT=
        # per log". We write the CMD: line above because the two skill drivers
        # install no cli_banner and would otherwise be unanchored -- but we
        # never wrote the EXIT= half, so a teed log of a banner-less tool
        # satisfied only one side of it. `[tee_cmd] ... exit=N` below is for
        # humans and does not carry the EXIT= prefix any parser looks for.
        log.write('EXIT=%d' % code + chr(10))
        log.flush()
    # run-23: ONE canonical completion signal. The exit line above lives in
    # the log; the `[tee_cmd] ... exit=` line goes to stdout only -- and a
    # waiter grepping the log for the stdout line cost 25 measured minutes.
    # `<label>.done` holds the exit code and appears exactly when the child
    # has exited; wait on os.path.exists of THIS, nothing else.
    _mark_done(code)
    t1 = time.time()

    with open(ledger, 'a', encoding='utf-8') as f:
        f.write(json.dumps({
            'label': label, 'log': os.path.relpath(path, here),
            't_start': round(t0, 3), 't_end': round(t1, 3),
            'iso_start': time.strftime('%Y-%m-%dT%H:%M:%S',
                                       time.localtime(t0)),
            'wall_s': round(t1 - t0, 3), 'exit': code,
            'argv': cmd, 'cmdline': ' '.join(cmd),
        }) + '\n')
    print('[tee_cmd] %s exit=%d wall=%.1fs -> %s'
          % (label, code, t1 - t0, os.path.relpath(path, here)))
    return code


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
