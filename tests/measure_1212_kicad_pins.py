#!/usr/bin/env python3
"""#1212: does check_assembly's pin_in_courtyard channel find what KiCad's
`pth_inside_courtyard` / `npth_inside_courtyard` finds?

KiCad grades each drilled pad against every OTHER footprint's courtyard. The
repo had no such check: a pin frame (rp2350's Teensy U8) was graded on its
rect, which flagged every part inside it and could not locate the real hit,
SW1's courtyard over U8 pins 16 and 17. The channel now grades a pin frame's
holes (`legality.CourtyardCensus._pin_pairs`). KiCad is the referee:

  * every KiCad (n)pth_inside_courtyard whose pad belongs to a CONTAINER must
    have a matching pin_in_courtyard pair (same two refs, the pin named) --
    a MISS is a defect of the channel;
  * every pin_in_courtyard pair must have a KiCad item behind it -- an EXTRA
    is a phantom.

KiCad items for drilled pads of NON-container parts are counted and listed
but not required: for an ordinary part the courtyard channel stands in for
them, which is what this channel exists to stop doing for a frame only.

A project that sets either rule to `ignore` is graded on a scratch copy
(`py_router/copy_board.py`) with both set to `error`, so KiCad reports them.

    python3 tests/measure_1212_kicad_pins.py board.kicad_pcb [...] [--corpus]

Exit 0 when nothing is MISSED and nothing is EXTRA; 1 otherwise; 77 when
kicad-cli is not installed (a self-skip, never a pass).
"""
from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in ('py_placer', 'py_router', 'py_tools', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

KICAD_CLI = (os.environ.get('KICAD_CLI')
             or shutil.which('kicad-cli')
             or next((p for p in (
                 'C:/Program Files/KiCad/10.0/bin/kicad-cli.exe',
                 'C:/Program Files/KiCad/9.0/bin/kicad-cli.exe',
                 '/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli',
                 '/usr/bin/kicad-cli') if os.path.isfile(p)), None))

RULES = ('pth_inside_courtyard', 'npth_inside_courtyard')
# Locale-independent: kicad-cli prints in the user's KiCad language
# ('PTH pad 16 [+3V3] of U8', 'PTH-pad 16 [+3V3] van U8'), so match
# the SHAPE -- an optional pad number (an NPTH hole often has none: 'NPTH-pad
# van FR'), an optional [net], one word, the reference.
_PAD = re.compile(r'pad\s+(?:(\S+)\s+)?(?:\[[^\]]*\]\s+)?\S+\s+(\S+)',
                  re.I)
_FP = re.compile(r'[Ff]ootprint (\S+)')


def _scratch(board, td):
    """A copy of `board` (siblings included) whose project grades both rules
    at error, or `board` itself when it already does."""
    sys.path.insert(0, os.path.join(ROOT, 'py_router'))
    pro = os.path.splitext(board)[0] + '.kicad_pro'
    doc = {}
    if os.path.exists(pro):
        with open(pro, encoding='utf-8') as fh:
            doc = json.load(fh)
    sev = (((doc.get('board') or {}).get('design_settings') or {})
           .get('rule_severities') or {})
    if all(sev.get(r, 'error') == 'error' for r in RULES):
        return board
    dst = os.path.join(td, os.path.basename(board))
    subprocess.run([sys.executable, '-X', 'utf8',
                    os.path.join(ROOT, 'py_router', 'copy_board.py'),
                    board, dst], check=True, capture_output=True, cwd=ROOT)
    dpro = os.path.splitext(dst)[0] + '.kicad_pro'
    with open(dpro, encoding='utf-8') as fh:
        doc = json.load(fh)
    rs = doc.setdefault('board', {}).setdefault(
        'design_settings', {}).setdefault('rule_severities', {})
    for r in RULES:
        rs[r] = 'error'
    with open(dpro, 'w', encoding='utf-8') as fh:
        json.dump(doc, fh, indent=2)
    return dst


def kicad_items(board, td):
    """{(pad_owner, other_footprint): {pins}} from kicad-cli's DRC."""
    path = _scratch(board, td)
    out = os.path.join(td, 'drc.json')
    subprocess.run([KICAD_CLI, 'pcb', 'drc', '--format', 'json',
                    '--severity-all', '-o', out, path],
                   capture_output=True, text=True)
    with open(out, encoding='utf-8') as fh:
        doc = json.load(fh)
    found = {}
    for v in doc.get('violations', []):
        if v.get('type') not in RULES:
            continue
        owner = other = pin = None
        for it in v.get('items', []):
            d = it.get('description', '')
            m = _PAD.search(d)
            if m and owner is None:
                pin, owner = m.group(1) or '', m.group(2)
                continue
            m = _FP.search(d)
            if m:
                other = m.group(1)
        if owner and other:
            found.setdefault((owner, other), set()).add(pin)
    return found


def ours(board):
    """{(frame, other): {pins}} from check_assembly's own GATING pin
    pairs at the project's own severities (the board handed in is the copy
    KiCad graded), and the board's containers."""
    from kicad_parser import parse_kicad_pcb
    from placement import legality
    pcb = parse_kicad_pcb(board)
    g = legality.grade_body_overlap(pcb, 0.2, pcb_file=board)
    kinds = g['containers']
    out = {}
    for p in g['pin_in_courtyard_pairs']:
        frame, other = (p.a, p.b) if p.a in kinds else (p.b, p.a)
        out.setdefault((frame, other), set()).update(p.pins)
    return out, kinds


def compare(board):
    with tempfile.TemporaryDirectory() as td:
        # ONE copy for both referees: `ours` grades the project KiCad was
        # handed, both pin rules at error. Grading the original instead made
        # a project that ignores them disagree with KiCad by construction.
        path = _scratch(board, td)
        k = kicad_items(path, td)
        mine, kinds = ours(path)
    missed, extra, ordinary = [], [], []
    for (owner, other), pins in sorted(k.items()):
        if owner not in kinds:
            ordinary.append((owner, other, sorted(pins)))
            continue
        got = mine.get((owner, other), set())
        if not pins <= got:
            missed.append((owner, other, sorted(pins - got)))
    for (frame, other), pins in sorted(mine.items()):
        kp = k.get((frame, other), set())
        if not pins <= kp:
            extra.append((frame, other, sorted(pins - kp)))
    return {'board': board, 'containers': kinds, 'kicad': len(k),
            'ours': len(mine), 'missed': missed, 'extra': extra,
            'ordinary': ordinary}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('boards', nargs='*')
    ap.add_argument('--corpus', action='store_true')
    args = ap.parse_args(argv)
    if not KICAD_CLI:
        print('SKIP: kicad-cli is not installed')
        return 77
    boards = list(args.boards)
    if args.corpus:
        import run_utils
        boards += [os.path.join(ROOT, b) for b in run_utils.corpus_boards()]
    bad = 0
    for b in boards:
        r = compare(b)
        print(f"{os.path.basename(b)[:36]:36s} containers {r['containers']}"
              f"  kicad {r['kicad']}  ours {r['ours']}  missed "
              f"{r['missed']}  extra {r['extra']}  (non-container items: "
              f"{len(r['ordinary'])})")
        bad += bool(r['missed'] or r['extra'])
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
