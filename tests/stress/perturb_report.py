#!/usr/bin/env python3
"""Movies and a score table for every #411 cell measured so far.

Reads the append-only scoreboard, finds each cell's arm boards across the work
dirs, renders every arm against the SAME reference (the control, i.e. the
dose-0 pass of the writer), and stitches one captioned animation per cell.

    python3 -X utf8 tests/stress/perturb_report.py --scoreboard <jsonl> \\
        --work wk/b2 wk/loop --out wk/report
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import subprocess
import sys

_here = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(_here, '..', '..'))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # #522/py_placer layout

# Frame order tells the story: ground truth, the damage, then each repair.
ARM_ORDER = ['control', 'none', 'quench@3mm', 'quench@dose',
             'loop@shipped', 'loop@targeted', 'loop@allon']
ARM_FILE = {
    'control': 'perturbed.control', 'none': 'perturbed',
    'quench@3mm': 'quench_3mm', 'quench@dose': 'quench_dose',
    'loop@shipped': 'loop_shipped', 'loop@targeted': 'loop_targeted',
    'loop@allon': 'loop_allon',
}


def load(scoreboard):
    rows = [json.loads(l) for l in open(scoreboard, encoding='utf-8')]
    return [r for r in rows if r.get('status') == 'ok']


def find_cells(work_dirs):
    """{(board, kind): {arm: board_path}} across every work dir."""
    cells = {}
    for wd in work_dirs:
        for d in sorted(glob.glob(os.path.join(wd, '*__*', 'd*'))):
            cell = os.path.basename(os.path.dirname(d))
            board, kind = cell.split('__', 1)
            ctl = os.path.join(d, 'perturbed.control.kicad_pcb')
            if not os.path.exists(ctl):
                continue
            slot = cells.setdefault((board, kind), {})
            for arm, stem in ARM_FILE.items():
                p = os.path.join(d, stem + '.kicad_pcb')
                if os.path.exists(p):
                    slot.setdefault(arm, p)
    return cells


def render(board, control, out_png):
    if os.path.exists(out_png):
        return True
    r = subprocess.run(
        [sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_tools', 'render_placement.py'),
         board, '--before', control, '-o', out_png, '--ratsnest-all'],
        capture_output=True, text=True, cwd=ROOT, timeout=900)
    return r.returncode == 0 and os.path.exists(out_png)


def caption(board, kind, arm, row):
    if arm == 'control':
        return f'{board} / {kind}   ORIGINAL human placement (ground truth)'
    d = (row or {}).get('displacement') or {}
    rf = (row or {}).get('recovery_fraction')
    am = (row or {}).get('arm_meta') or {}
    s = am.get('summary') or {}
    bits = [f"NC {row['NC']}", f"OO {row['OO']:.1f}",
            f"dist {d.get('perturbed_pad_rms', 0):.2f}mm"]
    if rf is not None and arm != 'none':
        bits.insert(0, f"recovery {rf:+.3f}")
    if s.get('failures_before') is not None:
        bits.append(f"fail {s['failures_before']}->{s['failures_after']}")
    tag = 'PERTURBED' if arm == 'none' else arm
    return f'{tag}   ' + '  '.join(bits)


def build(cell, arms, rows, out_dir, hold=9, duration=150):
    from PIL import Image, ImageDraw
    board, kind = cell
    by_arm = {r['arm']: r for r in rows
              if r['board'] == board and r['kind'] == kind}
    control = arms['control']
    frames = []
    for arm in ARM_ORDER:
        if arm not in arms:
            continue
        png = os.path.join(out_dir, f'{board}__{kind}__{ARM_FILE[arm]}.png')
        if not render(arms[arm], control, png):
            print(f'  render failed: {board}/{kind}/{arm}')
            continue
        im = Image.open(png).convert('RGB')
        im = im.resize((im.width // 2, im.height // 2))
        d = ImageDraw.Draw(im)
        d.rectangle([0, 0, im.width, 26], fill=(12, 12, 16))
        d.text((8, 7), caption(board, kind, arm, by_arm.get(arm)),
               fill=(240, 230, 140))
        frames.append(im)
    if len(frames) < 2:
        return None
    seq = []
    for f in frames:
        seq += [f] * hold
    out = os.path.join(out_dir, f'{board}__{kind}.gif')
    seq[0].save(out, save_all=True, append_images=seq[1:],
                duration=duration, loop=0, optimize=True)
    return out


def table(rows):
    """Every arm, grouped by cell, with the control as each cell's reference."""
    hdr = (f"{'board':17s} {'kind':10s} {'arm':>14s} {'dose':>7s} {'recov':>7s} "
           f"{'dist':>7s} {'NC':>5s} {'OO':>7s} {'oob':>4s} {'HPWL':>8s} "
           f"{'failures':>9s} {'acc':>4s} {'secs':>7s}")
    out = [hdr, '-' * len(hdr)]
    key = lambda r: (r['board'], r['kind'],
                     ARM_ORDER.index(r['arm']) if r['arm'] in ARM_ORDER else 99)
    last = None
    for r in sorted(rows, key=key):
        cell = (r['board'], r['kind'])
        if last and cell != last:
            out.append('')
        last = cell
        d = r.get('displacement') or {}
        rf = r.get('recovery_fraction')
        am = r.get('arm_meta') or {}
        s = am.get('summary') or {}
        fb, fa = s.get('failures_before'), s.get('failures_after')
        out.append(
            f"{r['board']:17s} {r['kind']:10s} {r['arm']:>14s} "
            f"{r.get('dose_mm_applied') or 0:7.3f} "
            f"{('--' if rf is None else f'{rf:+.3f}'):>7s} "
            f"{d.get('perturbed_pad_rms', 0):7.3f} {r['NC']:5d} {r['OO']:7.2f} "
            f"{r['OoB']['count']:4d} {r['HPWL']:8.1f} "
            f"{(f'{fb}->{fa}' if fb is not None else '--'):>9s} "
            f"{str(s.get('rounds_accepted', '--')):>4s} "
            f"{str(am.get('seconds', '--')):>7s}")
    return '\n'.join(out)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scoreboard', required=True)
    p.add_argument('--work', nargs='+', required=True)
    p.add_argument('--out', required=True)
    p.add_argument('--no-movies', action='store_true')
    a = p.parse_args(argv)

    os.makedirs(a.out, exist_ok=True)
    rows = load(a.scoreboard)
    print(table(rows))
    print(f"\n{len(rows)} rows  |  "
          f"{len({(r['board'], r['kind']) for r in rows})} cells  |  "
          f"code_version {sorted({r.get('code_version') for r in rows})}")
    if a.no_movies:
        return 0
    made = []
    for cell, arms in sorted(find_cells(a.work).items()):
        got = build(cell, arms, rows, a.out)
        if got:
            made.append(got)
            print(f"movie: {got}  ({len(arms)} arm(s))")
    print(f"\n{len(made)} movie(s) in {a.out}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
