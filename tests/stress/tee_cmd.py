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
    if '--workdir' in head:
        i = head.index('--workdir')
        if i + 1 >= len(head):
            raise SystemExit('tee_cmd: --workdir needs a value')
        here, head = os.path.abspath(head[i + 1]), head[:i] + head[i + 2:]
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

    t0 = time.time()
    with open(path, 'w', encoding='utf-8', errors='replace') as log:
        log.write('CMD: %s\n' % ' '.join(cmd))
        log.flush()
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT)
        for raw in proc.stdout:
            line = raw.decode('utf-8', 'replace')
            log.write(line)
            log.flush()
            sys.stdout.write(line)
            sys.stdout.flush()
        code = proc.wait()
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
    with open(path[:-4] + '.done' if path.endswith('.log')
              else path + '.done', 'w', encoding='utf-8') as f:
        f.write('%d\n' % code)
    if n > 1:
        # last-attempt-wins: the canonical marker always carries the NEWEST
        # attempt's exit, so "wait on logs/<label>.done" is correct across
        # retries (it was removed above while this attempt ran).
        with open(canonical_done, 'w', encoding='utf-8') as f:
            f.write('%d\n' % code)
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
