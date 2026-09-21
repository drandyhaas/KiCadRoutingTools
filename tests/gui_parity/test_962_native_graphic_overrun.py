#!/usr/bin/env python3
"""#962 native control: our graphic-copper overrun equals pcbnew's own geometry.

`check_drc.footprint_graphic_outline_census` measures footprint graphic copper
against the outline from the repo's own model: outline segments at the DRAWN
stroke, true circles. This compares it with an independent instrument, KiCad's
own. esp_prog's outline is an axis-aligned rectangle, so the exact signed
overrun of a shape is the largest of its four side overhangs, computed from
pcbnew's stroke-inclusive bounding box against the Edge.Cuts centrelines.

The staged poses:
- U2 at 115.34 x rotations 0/90/180/270;
- U2 at 115.34 on the B side (rotation 90);
- 116.70;
- the original.

Every pose's native and modelled overrun must agree within 0.01 mm, and the
set must contain both an off-board pose and an on-board one. The issue
comment's own numbers anchor it: 115.34/90 is 1.11 mm off, 116.70 is 0.25 mm
inside.

Needs pcbnew; re-execs into KiCad's python. Exits 2, not 0, when no pcbnew
python exists.

    python3 -X utf8 tests/gui_parity/test_962_native_graphic_overrun.py
"""
import glob
import os
import shutil
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
KICAD_PYTHONS = [
    "/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3",
    "/usr/bin/python3",
    os.path.expandvars(r"C:\Program Files\KiCad\bin\python.exe"),
    *sorted(glob.glob(r"C:\Program Files\KiCad\*\bin\python.exe"), reverse=True),
]
FAILS = []


def _reexec():
    for cand in KICAD_PYTHONS:
        if cand == sys.executable or not os.path.exists(cand):
            continue
        if subprocess.run([cand, '-c', 'import pcbnew'], capture_output=True).returncode == 0:
            argv = [cand, '-X', 'utf8', os.path.abspath(__file__)] + sys.argv[1:]
            if os.name == 'nt':
                sys.exit(subprocess.run(argv).returncode)
            os.execv(cand, argv)
    print("NOT RUN: no python with pcbnew found (exit 2, not a pass)")
    sys.exit(2)


def check(name, cond, detail=''):
    print(f"  {'ok  ' if cond else 'FAIL'}  {name}" + (f"  -- {detail}" if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


def main():
    try:
        import pcbnew  # noqa: F401
    except ImportError:
        _reexec()
    import pcbnew
    for p in ('py_router', 'py_placer', 'tests'):
        sys.path.insert(0, os.path.join(REPO, p))
    from copy_board import copy_board
    from kicad_parser import parse_kicad_pcb
    from check_drc import footprint_graphic_outline_census
    from placement.writer import write_placed_output

    esp = os.path.join(REPO, 'kicad_files', 'esp_prog.kicad_pcb')
    work = tempfile.mkdtemp(prefix='krt962n_')
    poses = [('orig', None)] + [('115.34 r%d' % r, dict(new_x=115.34, new_rotation=r))
                                for r in (0, 90, 180, 270)]
    poses += [('115.34 r90 B', dict(new_x=115.34, new_rotation=90, new_side='B')),
              ('116.70 r90', dict(new_x=116.70, new_rotation=90))]
    seen = set()
    got = {}
    try:
        for i, (name, pl) in enumerate(poses):
            out = os.path.join(work, 'p%d.kicad_pcb' % i)
            copy_board(esp, out)
            if pl:
                write_placed_output(out, out, [dict(reference='U2', new_y=93.6, **pl)])
            mine = max(r['overrun_mm'] for r in footprint_graphic_outline_census(
                parse_kicad_pcb(out))['rows'] if r['owner_ref'] == 'U2')
            b = pcbnew.LoadBoard(out)
            xs, ys = [], []
            for d in b.GetDrawings():
                if d.GetLayer() == pcbnew.Edge_Cuts:
                    for v in (d.GetStart(), d.GetEnd()):
                        xs.append(v.x / 1e6)
                        ys.append(v.y / 1e6)
            ex0, ex1, ey0, ey1 = min(xs), max(xs), min(ys), max(ys)
            u2 = [f for f in b.GetFootprints() if f.GetReference() == 'U2'][0]
            native = -1e9
            for d in u2.GraphicalItems():
                if d.GetClass() != 'PCB_SHAPE':
                    continue
                if d.GetLayer() not in (pcbnew.F_Cu, pcbnew.B_Cu):
                    continue
                bb = d.GetBoundingBox()
                l, r_, t, bo = bb.GetLeft() / 1e6, bb.GetRight() / 1e6, bb.GetTop() / 1e6, bb.GetBottom() / 1e6
                native = max(native, ex0 - l, r_ - ex1, ey0 - t, bo - ey1)
            got[name] = (mine, native)
            seen.add(native > 1e-6)
            check('%-14s model %+.4f  native %+.4f' % (name, mine, native),
                  abs(mine - native) <= 0.01)
        check('the sweep has an off-board pose AND an on-board one', seen == {True, False})
        check('anchor: 115.34 r90 is 1.11 mm off (the issue comment)',
              abs(got['115.34 r90'][1] - 1.11) <= 0.005, str(got.get('115.34 r90')))
        check('anchor: 116.70 r90 is 0.25 mm inside (the issue comment)',
              abs(got['116.70 r90'][1] + 0.25) <= 0.005, str(got.get('116.70 r90')))
    finally:
        shutil.rmtree(work, ignore_errors=True)
    print('\n%d failure(s)' % len(FAILS))
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
