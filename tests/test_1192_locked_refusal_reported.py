#!/usr/bin/env python3
"""#1192: a KiCad-locked refusal says it has no override, and an emptied
scope still reports it in --json-out.

One `(locked yes)` segment protects its whole net with no override (#521),
yet the exclusion print told the user to "name a net exactly (no glob) to
override" -- to a caller who had. And when that refusal emptied the scope,
route.py took the "nothing to route" early return, whose --json-out had no
`protected_skipped`: a program read "nothing to do", not "refused".

Checks:
  1. filter_rippable_names: a 'locked' refusal prints no override hint and
     says to unlock it; a mixed refusal names the override for the rest; a
     non-locked refusal keeps the hint.
  2. route.py `--nets N --force-reroute` on a board whose net N carries one
     locked segment: the console line says no override, and --json-out
     carries protected_skipped {'--force-reroute': {N: 'locked'}}.

    python3 tests/test_1192_locked_refusal_reported.py
"""
import contextlib
import io
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

import protected_nets                                          # noqa: E402
from copy_board import copy_board                              # noqa: E402
from kicad_parser import parse_kicad_pcb                       # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'routed_output.kicad_pcb')
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def refusal_line(names, protected, overrides):
    protected_nets.clear_skipped()
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        kept = protected_nets.filter_rippable_names(
            names, protected, overrides, context='--force-reroute')
    return kept, buf.getvalue().strip()


def main():
    # 1. The print.
    kept, line = refusal_line(['+5V'], {'+5V': 'locked'}, ['+5V'])
    check('a locked net is refused even when named exactly', kept == [], line)
    check('a locked refusal prints no override hint',
          'name a net exactly' not in line and 'no override' in line, line)
    kept, line = refusal_line(['+5V', '/D0'], {'+5V': 'locked', '/D0': 'matched'},
                              [])
    check('a mixed refusal names the override for the rest and the lock',
          "override all but 'locked'" in line and 'no override' in line, line)
    kept, line = refusal_line(['/D0'], {'/D0': 'matched'}, [])
    check('a non-locked refusal keeps the hint',
          'name a net exactly (no glob) to override' in line
          and 'no override' not in line, line)
    protected_nets.clear_skipped()

    # 2. End to end.
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(BOARD)
    from check_connected import check_net_connectivity
    by_net = {}
    for s in pcb.segments:
        by_net.setdefault(s.net_id, []).append(s)

    def connected(n):
        c = check_net_connectivity(
            n, by_net[n], [v for v in pcb.vias if v.net_id == n],
            pcb.pads_by_net.get(n, []),
            [z for z in (pcb.zones or []) if z.net_id == n])
        return c['num_components'] == 1 and not c['disconnected_pads']
    # A net that is already connected, so the refusal empties the scope.
    net_id = next((n for n in sorted(
        (n for n in by_net if n and n in pcb.nets and pcb.nets[n].name
         and len(by_net[n]) >= 2 and not any(z.net_id == n for z in pcb.zones or [])),
        key=lambda n: (-len(by_net[n]), n)) if connected(n)), None)
    check('precondition: a connected routed net exists', net_id is not None)
    if net_id is None:
        return 1
    name = pcb.nets[net_id].name
    tmp = tempfile.mkdtemp(prefix='t1192_')
    try:
        inp = os.path.join(tmp, 'in.kicad_pcb')
        with contextlib.redirect_stdout(io.StringIO()):
            copy_board(BOARD, inp)
        text = open(inp, encoding='utf-8').read()
        # Lock the net's first segment (either net dialect).
        net_pat = r'\(net (?:%d|"%s")\)' % (net_id, re.escape(name))
        m = next((b for b in re.finditer(r'\(segment\n(?:\t\t[^\n]*\n)+?\t\)', text)
                  if re.search(net_pat, b.group(0))), None)
        check('precondition: a segment of the chosen net was found', m is not None,
              name)
        if m is None:
            return 1
        text = text[:m.start()] + '(segment\n\t\t(locked yes)\n' + text[m.start() + len('(segment\n'):]
        open(inp, 'w', encoding='utf-8').write(text)
        with contextlib.redirect_stdout(io.StringIO()):
            locked = protected_nets.locked_net_names(parse_kicad_pcb(inp))
        check('precondition: the net reads as locked', locked == {name},
              str(locked))

        out, js = os.path.join(tmp, 'out.kicad_pcb'), os.path.join(tmp, 'out.json')
        r = subprocess.run([sys.executable, '-X', 'utf8', 'py_router/route.py',
                            inp, out, '--json-out', js, '--nets', name,
                            '--force-reroute'],
                           cwd=ROOT, capture_output=True, text=True,
                           encoding='utf-8', errors='replace')
        con = r.stdout + r.stderr
        check('route.py exits 0', r.returncode == 0, con[-600:] if r.returncode else '')
        line = next((l for l in con.splitlines() if 'PROTECTED net(s)' in l), '')
        check('the console refusal says the lock has no override',
              'no override' in line and 'name a net exactly' not in line, line)
        doc = json.load(open(js, encoding='utf-8')) if os.path.isfile(js) else {}
        check('the run took the nothing-to-route return',
              doc.get('status') == 'already_connected', str(doc.get('status')))
        check('--json-out carries the refusal',
              (doc.get('protected_skipped') or {}).get('--force-reroute')
              == {name: 'locked'}, str(doc.get('protected_skipped')))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
