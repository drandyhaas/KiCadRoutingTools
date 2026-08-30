#!/usr/bin/env python3
"""A refused rip must be MACHINE-readable, not just printed.

When `--rip-existing-nets` matches a protected or KiCad-locked net, the router
refuses it and prints why. That print is for a human reading a log. A program
driving the router cannot see it -- and the router's own failure hint tells that
program to retry with `--rip-existing-nets` naming exactly the net it just
refused. A caller following the router's advice therefore loops forever.

Worse, the two refusal reasons need different responses and the print does not
separate them for a machine:

    reason != 'locked'  ->  name the net EXACTLY (no glob) and it will be ripped
    reason == 'locked'  ->  there is NO override, ever; stop asking

So `JSON_SUMMARY['protected_skipped']` carries `{context: {net: reason}}`.
"""
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')


def _run(board, out, extra, td):
    js = os.path.join(td, f'{os.path.basename(out)}.json')
    r = subprocess.run([sys.executable, '-X', 'utf8',
                        os.path.join(ROOT, 'py_router', 'route.py'), board, out,
                        '--json-out', js] + extra,
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=ROOT)
    assert r.returncode == 0, r.stdout[-2500:]
    return json.load(open(js, encoding='utf-8')), r.stdout


def _lock_all_copper_of(path, net):
    """Mark every segment of `net` KiCad-locked, the way a user pinning copper
    in the GUI does. kicad_parser accepts (locked yes) only directly after
    (width ...) or (layer ...) -- both before (net ...)."""
    txt = open(path, encoding='utf-8').read()

    def fix(m):
        blk = m.group(0)
        if f'(net "{net}")' not in blk or '(locked yes)' in blk:
            return blk
        return re.sub(r'(\(layer\s+"[^"]+"\)\s*\n)', r'\1\t\t(locked yes)\n',
                      blk, count=1)

    txt = re.sub(r'\(segment\b(?:[^()]|\((?:[^()]|\([^()]*\))*\))*\)', fix, txt)
    open(path, 'w', encoding='utf-8').write(txt)


def test_locked_net_refusal_reaches_the_summary():
    if not os.path.isfile(BOARD):
        print("  SKIP: fixture missing")
        return
    with tempfile.TemporaryDirectory() as td:
        staged = os.path.join(td, 'b.kicad_pcb')
        shutil.copyfile(BOARD, staged)

        # 1. lay some copper down, so there is something to lock and to block with
        first = os.path.join(td, 'r1.kicad_pcb')
        _run(staged, first, ['--nets', 'GND'], td)

        # 2. pin it, then ask a LATER run to rip it via a glob
        _lock_all_copper_of(first, 'GND')
        summary, out = _run(first, os.path.join(td, 'r2.kicad_pcb'),
                            ['--nets', '*', '--rip-existing-nets', '*'], td)

        skipped = summary.get('protected_skipped')
        assert skipped, (
            "a refused rip left no machine-readable trace; the log said:\n"
            + '\n'.join(l for l in out.splitlines() if 'PROTECTED' in l))
        flat = {n: r for ctx in skipped.values() for n, r in ctx.items()}
        assert 'GND' in flat, f"GND was locked but is not in {skipped}"
        assert flat['GND'] == 'locked', (
            f"reason must distinguish the no-override case, got {flat['GND']!r}")
        # And the human-readable print is still there -- this adds a channel,
        # it does not replace one.
        assert 'PROTECTED' in out
    print("  PASS: a locked net's refusal is in JSON_SUMMARY with reason 'locked'")


def test_clean_run_carries_no_key():
    """The key must be absent, not empty, when nothing was refused -- a caller
    testing `if summary.get('protected_skipped')` should see nothing."""
    if not os.path.isfile(BOARD):
        print("  SKIP: fixture missing")
        return
    with tempfile.TemporaryDirectory() as td:
        summary, _ = _run(BOARD, os.path.join(td, 'r.kicad_pcb'),
                          ['--nets', 'GND'], td)
        assert 'protected_skipped' not in summary
    print("  PASS: absent on a run that refused nothing")


if __name__ == '__main__':
    for k, v in sorted(globals().items()):
        if k.startswith('test_'):
            print(f"--- {k}")
            v()
    print("ALL PASS")
