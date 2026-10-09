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

Every board is graded on a scratch copy (`py_router/copy_board.py`) whose
project sets both rules to `error`, so KiCad reports them -- ALWAYS a copy:
kicad-cli rewrites the board's sibling `.kicad_prl`, and grading in place
wrote into the measured tree (second phase-3 verifier).

`--onset` re-measures `CourtyardCensus.PIN_HOLE_TOLERANCE_MM`: SW1's
courtyard is slid toward U8's pins on the tracked rp2350 board in 1 um steps
and both referees are asked where the pin starts to count. Exit 0 when they
agree on the first step.

    python3 tests/measure_1212_kicad_pins.py board.kicad_pcb [...] [--corpus]
    python3 tests/measure_1212_kicad_pins.py --onset

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
    at error. Always a copy, never the board itself: kicad-cli rewrites the
    graded board's `.kicad_prl`."""
    sys.path.insert(0, os.path.join(ROOT, 'py_router'))
    dst = os.path.join(tempfile.mkdtemp(dir=td), os.path.basename(board))
    subprocess.run([sys.executable, '-X', 'utf8',
                    os.path.join(ROOT, 'py_router', 'copy_board.py'),
                    board, dst], check=True, capture_output=True, cwd=ROOT)
    dpro = os.path.splitext(dst)[0] + '.kicad_pro'
    doc = {}
    if os.path.exists(dpro):
        with open(dpro, encoding='utf-8') as fh:
            doc = json.load(fh)
    rs = doc.setdefault('board', {}).setdefault(
        'design_settings', {}).setdefault('rule_severities', {})
    for r in RULES:
        rs[r] = 'error'
    with open(dpro, 'w', encoding='utf-8') as fh:
        json.dump(doc, fh, indent=2)
    return dst


def kicad_items(path, td):
    """{(pad_owner, other_footprint): {pins}} from kicad-cli's DRC of
    `path` -- a `_scratch` copy."""
    out = os.path.join(os.path.dirname(path), 'drc.json')
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


#: The onset fixture: the tracked rp2350 board, SW1 turned to 0 degrees
#: under U8's pins 17-19 (fa10 seed s04_u6r90's pose, where a 1 um graze
#: was the second phase-3 verifier's phantom). Each step slides SW1 1 um
#: further in; step 0 is that 1 um graze.
ONSET_BOARD = os.path.join(ROOT, 'kicad_files',
                           'rp2350_fpga_eensy_prePlane.kicad_pcb')
ONSET_POSE = (151.15, 119.116, 0.0)
ONSET_STEPS_UM = tuple(range(0, 9))


def onset(steps=ONSET_STEPS_UM):
    """[(step_um, kicad pins, our pins)] for SW1 over U8, and the first
    step each referee reports."""
    from kicad_parser import parse_kicad_pcb
    from placement.writer import write_placed_output
    import contextlib
    import io
    rows = []
    with tempfile.TemporaryDirectory() as td:
        for d in steps:
            path = _scratch(ONSET_BOARD, td)
            x, y, rot = ONSET_POSE
            with contextlib.redirect_stdout(io.StringIO()):
                write_placed_output(path, path, [{
                    'reference': 'SW1', 'new_x': x,
                    'new_y': round(y + d / 1000.0, 6), 'new_rotation': rot}],
                    pcb_data=parse_kicad_pcb(path))
            k = kicad_items(path, td)
            mine, _kinds = ours(path)
            rows.append((d, sorted(k.get(('U8', 'SW1'), ())),
                         sorted(mine.get(('U8', 'SW1'), ()))))
    first = lambda i: next((r[0] for r in rows if r[i]), None)  # noqa: E731
    return rows, first(1), first(2)


CHAINING = os.path.join(ROOT, 'tests', 'fixtures',
                        '1212_courtyard_chaining.json')


def chaining():
    """[(name, recorded, measured)] for every drawing in the chaining
    fixture: each courtyard put on a part P over test_1212's pin frame,
    graded by kicad-cli, `malformed_courtyard` on P meaning not closed."""
    from test_1212_container_pins import _frame_board
    with open(CHAINING, encoding='utf-8') as fh:
        cases = json.load(fh)['cases']
    rows = []
    with tempfile.TemporaryDirectory() as td:
        for name, c in sorted(cases.items()):
            sub = tempfile.mkdtemp(dir=td)
            path = _frame_board(sub, [])
            part = (f'  (footprint "t:P" (layer "{c["side"]}.Cu")'
                    f' (at 3 3.1 {c["rot"]:g})\n'
                    f'    (property "Reference" "P" (at 0 0) (layer'
                    f' "{c["side"]}.SilkS"))\n'
                    + ''.join('    ' + e + '\n' for e in c['courtyard'])
                    + '  )\n')
            with open(path, encoding='utf-8') as fh:
                body = fh.read().rstrip()
            with open(path, 'w', encoding='utf-8') as fh:
                fh.write(body[:-1] + part + ')\n')
            out = os.path.join(sub, 'drc.json')
            subprocess.run([KICAD_CLI, 'pcb', 'drc', '--format', 'json',
                            '--severity-all', '-o', out, path],
                           capture_output=True, text=True)
            with open(out, encoding='utf-8') as fh:
                doc = json.load(fh)
            mal = any(v.get('type') == 'malformed_courtyard' and any(
                (it.get('description') or '').split()[-1:] == ['P']
                for it in v.get('items', []))
                for v in doc.get('violations', []))
            rows.append((name, c['kicad_closed'], not mal))
    return rows


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('boards', nargs='*')
    ap.add_argument('--corpus', action='store_true')
    ap.add_argument('--onset', action='store_true',
                    help='re-measure the hole tolerance on the tracked '
                         'rp2350 board (see the module docstring)')
    ap.add_argument('--chaining', action='store_true',
                    help='re-measure kicad-cli\'s verdict on every drawing '
                         'in tests/fixtures/1212_courtyard_chaining.json')
    args = ap.parse_args(argv)
    if not KICAD_CLI:
        print('SKIP: kicad-cli is not installed')
        return 77
    if args.chaining:
        rows = chaining()
        moved = [r for r in rows if r[1] != r[2]]
        for name, was, now in moved:
            print(f"{name}: recorded closed={was}, kicad-cli now {now}")
        print(f"{len(rows)} drawings, {len(moved)} verdict(s) moved")
        return 1 if moved else 0
    if args.onset:
        rows, k0, o0 = onset()
        for d, k, o in rows:
            print(f"step {d} um past a 1 um graze: kicad {k}  ours {o}")
        print(f"first step reported: kicad {k0}  ours {o0}")
        return 0 if k0 == o0 else 1
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
