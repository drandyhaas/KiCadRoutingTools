#!/usr/bin/env python3
"""#1766: a pad contact names its net's protection, and the remedy that fits.

A route step takes the copper of a net in its --nets off another net's pad
(#1680) -- unless the net is protected. KiCad-locked copper and copper a
`sense:` group holds are never ripped and NO flag lifts them, so the
diagnose skill's "name that net in --nets" cannot fix those shorts, and the
`SHORTED on the board this run ships` line and the `pad_contacts` rows did not
say which cause was in play.

Now each shipped row carries `net_protected` (the NET's reason: protection is
per net, so an unlocked track of a net with any locked copper says 'locked'),
the SHORTED line names it, and the audit prints the remedy: unlock / ungroup
in the board, or report the short as the board's copper.

Rig (from the #1766 report): tests/fixtures/sense_branch/sense_star.kicad_pcb
plus a net SIG, two pads joined by two tracks that meet ON VDD pad U2.1.
  locked: only the FIRST track (locked yes) -- the whole net is locked
  sense:  both tracks held by a `sense:J1.1` group
  free:   neither -- the control: route.py --nets SIG takes them off the pad

    python3 -X utf8 tests/test_1766_pad_contact_protection.py
"""
import json
import os
import re
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_router', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))
from run_utils import evidence  # noqa: E402

BOARD = os.path.join(ROOT, 'tests', 'fixtures', 'sense_branch', 'sense_star.kicad_pcb')
FAILS = []


def check(name, cond, detail=''):
    print(('  PASS  ' if cond else '  FAIL  ') + name + (f'  [{detail}]' if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


def make(td, mode):
    p = os.path.join(td, f'in_{mode}.kicad_pcb')
    subprocess.run([sys.executable, '-X', 'utf8', 'py_router/copy_board.py', BOARD, p],
                   cwd=ROOT, capture_output=True)
    evidence(p, 'fixture copy')
    with open(p, encoding='utf-8') as fh:
        t = fh.read()
    assert '(net 1 "VDD")\n' in t and '  (gr_rect' in t, 'fixture changed shape'
    t = t.replace('(net 1 "VDD")\n', '(net 1 "VDD") (net 2 "SIG")\n', 1)
    fp = ('  (footprint "T:P" (layer "F.Cu") (at {x} 12)\n'
          '    (property "Reference" "{r}" (at 0 -2 0) (layer "F.SilkS") (effects (font (size 1 1))))\n'
          '    (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu" "F.Paste" "F.Mask") (net 2 "SIG")))\n')
    seg = ('  (segment (start {a}) (end {b}) (width 0.2){lk} (layer "F.Cu") (net 2) '
           '(uuid "44444444-0000-0000-0000-00000000000{i}"))\n')
    add = (fp.format(x=5, r='J1') + fp.format(x=25, r='J2')
           + seg.format(a='5 12', b='15 8', i=1,
                        lk=' (locked yes)' if mode == 'locked' else '')
           + seg.format(a='15 8', b='25 12', i=2, lk=''))
    if mode == 'sense':
        add += ('  (group "sense:J1.1" (uuid "55555555-0000-0000-0000-000000000001") (members '
                '"44444444-0000-0000-0000-000000000001" "44444444-0000-0000-0000-000000000002"))\n')
    i = t.index('  (gr_rect')
    with open(p, 'w', encoding='utf-8') as fh:
        fh.write(t[:i] + add + t[i:])
    return p


def route(td, mode):
    src = make(td, mode)
    js = os.path.join(td, f'{mode}.json')
    r = subprocess.run([sys.executable, '-X', 'utf8', 'py_router/route.py', src,
                        os.path.join(td, f'out_{mode}.kicad_pcb'), '--nets', 'SIG',
                        '--json-out', js],
                       cwd=ROOT, capture_output=True, text=True,
                       encoding='utf-8', errors='replace')
    log = re.sub(r'\x1b\[[0-9;]*m', '', r.stdout + r.stderr)
    check(f'{mode}: route.py exits 0', r.returncode == 0, log[-400:])
    evidence(js, f'{mode} --json-out')
    with open(js, encoding='utf-8') as fh:
        return log, json.load(fh)


def unit_rows():
    from pad_contact import protection_remedy, format_rows
    check('locked: no flag lifts it, unlock in the board',
          'no flag lifts it' in protection_remedy('locked')
          and 'unlock it in the board' in protection_remedy('locked'))
    check('sense: no flag lifts it, ungroup',
          'no flag lifts it' in protection_remedy('sense:J1.1')
          and 'ungroup' in protection_remedy('sense:J1.1'))
    check('a .kicad_pro protection is lifted by naming the net exactly',
          'name the net exactly' in protection_remedy('matched-group')
          and 'no flag' not in protection_remedy('matched-group'))
    row = {'pad': 'U2.1', 'pad_net': 'VDD', 'net': 'SIG', 'kind': 'track',
           'at': [15, 8], 'overlap_mm': 0.3, 'short': True}
    check('an unprotected row prints as before',
          format_rows([row]) == ['U2.1 (VDD) on a SIG track at (15.00, 8.00), overlap 0.300 mm'],
          str(format_rows([row])))
    check('a protected row names its net\'s reason',
          format_rows([dict(row, net_protected='locked')])[0].endswith('[net SIG: locked]'),
          str(format_rows([dict(row, net_protected='locked')])))


def board_rows():
    with tempfile.TemporaryDirectory() as td:
        for mode, why in (('locked', 'locked'), ('sense', 'sense:J1.1')):
            log, js = route(td, mode)
            rows = [r for r in js.get('pad_contacts') or [] if r.get('short')]
            check(f'{mode}: the short ships (the step cannot rip it)',
                  len(rows) == 2 and all(r['pad'] == 'U2.1' for r in rows), str(rows))
            check(f'{mode}: every row names its NET\'s protection ({why})',
                  rows and all(r.get('net_protected') == why for r in rows),
                  str([r.get('net_protected') for r in rows]))
            check(f'{mode}: the SHORTED line names it',
                  f'[net SIG: {why}]' in log)
            check(f'{mode}: the audit prints the no-flag remedy',
                  f'SIG: {"KiCad-locked copper" if mode == "locked" else "held by the"}' in log
                  and 'no flag lifts it' in log)
        log, js = route(td, 'free')
        check('free (control): the step takes the copper off the pad',
              'Pad contacts: ' in log and not (js.get('pad_contacts') or []),
              str(js.get('pad_contacts')))


def main():
    unit_rows()
    board_rows()
    if FAILS:
        print(f'\nFAIL: {len(FAILS)} check(s): ' + '; '.join(FAILS))
        return 1
    print('\nALL CHECKS PASSED')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
