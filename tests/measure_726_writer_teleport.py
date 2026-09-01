#!/usr/bin/env python3
"""#726 measurement: what `write_placed_output` does to a footprint's twin.

`placement/writer.py:write_placed_output` finds footprint blocks in the
.kicad_pcb TEXT and matches each one's `(property "Reference" "XX")` against a
dict keyed by reference (`writer.py:184`, `:201-208`). Nothing consumes an
entry, so ONE placement rewrites EVERY block sharing that reference.

Three tables:

  E  THE HEADLINE -- the identity write. `placement/perturb.py:_all_at_current`
     hands the writer every footprint at the pose it ALREADY has, and its
     docstring says "Poses are unchanged for non-members -- only their
     formatting is." `perturb.py:572` builds the stress harness's ground-truth
     CONTROL board that way. This table asks how many blocks such a call
     actually relocates. Post-fix it must read 0 on every board, and the
     boards that already read 0 are the built-in negative controls: a gate
     that cannot report zero is measuring nothing.

  F  the single-placement teleport -- one placement for one duplicated
     reference, and how many blocks move. Includes the writer's own
     "Modified N footprint positions" line, which counts BLOCKS, not
     placements, and is the only existing observable of the defect.

  G  twin span -- how far apart the blocks sharing a reference actually sit,
     i.e. how far the loser travels when the winner's pose is stamped on it.

Poses, not bytes. The writer reformats every `(at)` it touches to `:.6f`
(`writer.py:224-237`), so `(at 141.2 95.9)` becomes `(at 141.200000 95.900000)`
with the part in the same place. A byte comparison would report that as a move.

Board set from `run_utils.corpus_boards()` (git ls-files), never a glob.
Not named `test_*`: `tests/run_all.py` must not collect a measurement.

    python3 -X utf8 tests/measure_726_writer_teleport.py
"""
import io
import os
import shutil
import sys
import tempfile
from contextlib import redirect_stdout

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for _p in ('', 'py_router', 'py_placer', 'py_tools'):
    _d = os.path.join(ROOT, _p)
    if _d not in sys.path:
        sys.path.insert(0, _d)
sys.path.insert(0, TESTS_DIR)

from run_utils import corpus_boards                              # noqa: E402
from kicad_parser import parse_kicad_pcb                         # noqa: E402
from placement.writer import write_placed_output                 # noqa: E402
from measure_726_duplicate_census import (blocks, raw_reference,  # noqa: E402
                                          fp_uuid, pose, _read)

SKIP_EXIT = 77
TOL = 1e-6      # mm; both sides are nm-quantized, so a real move is huge


def block_poses(path):
    """[(uuid, raw_reference, (x, y, rot))] in FILE ORDER.

    Keyed on uuid so a block can be followed across the write even though its
    reference is exactly the thing that is ambiguous.
    """
    return [(fp_uuid(t), raw_reference(t), pose(t))
            for _, _, t in blocks(_read(path))]


def _write(src, dst, placements):
    """Run the writer, returning its stdout (the `Modified N` line)."""
    buf = io.StringIO()
    with redirect_stdout(buf):
        write_placed_output(src, dst, placements)
    out = buf.getvalue()
    # The writer prints a success line last; the number this measurement
    # is about is the 'Modified N footprint positions' line (writer.py:385),
    # which counts BLOCKS rewritten, not placements supplied.
    for line in reversed(out.splitlines()):
        if line.startswith('Modified ') and 'footprint positions' in line:
            return line.strip()
    return '(no Modified line)'


def table_e(paths, workdir):
    print("\n== TABLE E -- the IDENTITY write: every part at the pose it "
          "already has ==\n")
    hdr = ('%-34s %7s %12s %10s  %s'
           % ('board', 'blocks', 'placements', 'MOVED', 'max displacement (mm)'))
    print(hdr)
    print('-' * len(hdr))
    total_moved = 0
    zero_rows = 0
    detail = []
    for p in paths:
        before = block_poses(p)
        pcb = parse_kicad_pcb(p)
        placements = [{'reference': r, 'new_x': f.x, 'new_y': f.y,
                       'new_rotation': f.rotation}
                      for r, f in pcb.footprints.items()]
        dst = os.path.join(workdir, os.path.basename(p))
        _write(p, dst, placements)
        after = block_poses(dst)
        assert len(before) == len(after), (
            '%s: block count changed %d -> %d; the writer must never add or '
            'drop a block' % (p, len(before), len(after)))
        moved = []
        for (uu, ref, (x0, y0, r0)), (_, _, (x1, y1, r1)) in zip(before, after):
            d = ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5
            if d > TOL or abs(r1 - r0) > TOL:
                moved.append((uu, ref, (x0, y0), (x1, y1), d))
        total_moved += len(moved)
        if not moved:
            zero_rows += 1
        print('%-34s %7d %12d %10d  %.2f'
              % (os.path.basename(p)[:34], len(before), len(placements),
                 len(moved), max([m[4] for m in moved], default=0.0)))
        for m in moved:
            detail.append((os.path.basename(p), m))
    print('\nTOTAL blocks relocated by a write whose contract is that nothing '
          'moves: %d.  Boards reading 0: %d of %d.'
          % (total_moved, zero_rows, len(paths)))
    if detail:
        print('\nEvery relocation, named:')
        print('  %-24s %-8s %-9s %-22s %-22s %8s'
              % ('board', 'ref', 'uuid', 'from', 'to', 'mm'))
        for b, (uu, ref, a, z, d) in detail:
            print('  %-24s %-8s %-9s (%8.3f,%8.3f) -> (%8.3f,%8.3f) %8.2f'
                  % (b.replace('.kicad_pcb', '')[:24], ref, uu[-8:],
                     a[0], a[1], z[0], z[1], d))
    return total_moved


def table_f(paths, workdir):
    print("\n== TABLE F -- one placement for one reference ==\n")
    hdr = ('%-22s %-8s %7s %7s %7s  %s'
           % ('board', 'ref', 'blocks', 'placed', 'MOVED', "writer's own stdout"))
    print(hdr)
    print('-' * len(hdr))
    detail = []
    for p in paths:
        before = block_poses(p)
        counts = {}
        for _, ref, _ in before:
            counts[ref] = counts.get(ref, 0) + 1
        # The board's first duplicated reference in file order; on a clean
        # board, its first reference -- the one-name-one-block control.
        target = next((r for _, r, _ in before if counts[r] > 1), None)
        control = target is None
        if control:
            target = before[0][1]
        tx, ty = 120.0, 110.0
        dst = os.path.join(workdir, 'f_' + os.path.basename(p))
        out = _write(p, dst, [{'reference': target, 'new_x': tx, 'new_y': ty,
                               'new_rotation': 0.0}])
        after = block_poses(dst)
        moved = []
        for (uu, ref, (x0, y0, r0)), (_, _, (x1, y1, r1)) in zip(before, after):
            d = ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5
            if d > TOL or abs(r1 - r0) > TOL:
                moved.append((uu, ref, (x0, y0), (x1, y1), d))
        print('%-22s %-8s %7d %7d %7d  %s'
              % (os.path.basename(p).replace('.kicad_pcb', '')[:22], target,
                 counts[target], 1, len(moved),
                 out)
              + ('   <- CONTROL: one name, one block' if control else ''))
        for m in moved:
            detail.append((os.path.basename(p), target, m))
    print('\nEvery block moved, named:')
    print('  %-22s %-8s %-9s %-22s %-22s %8s'
          % ('board', 'ref', 'uuid', 'from', 'to', 'mm'))
    for b, tgt, (uu, ref, a, z, d) in detail:
        print('  %-22s %-8s %-9s (%8.3f,%8.3f) -> (%8.3f,%8.3f) %8.2f'
              % (b.replace('.kicad_pcb', '')[:22], ref, uu[-8:],
                 a[0], a[1], z[0], z[1], d))


def table_g(paths):
    print("\n== TABLE G -- twin span: how far the loser travels ==\n")
    hdr = '%-22s %-8s %4s %14s' % ('board', 'ref', 'n', 'max span (mm)')
    print(hdr)
    print('-' * len(hdr))
    rows = []
    for p in paths:
        groups = {}
        for uu, ref, (x, y, _) in block_poses(p):
            groups.setdefault(ref, []).append((x, y))
        for ref, pts in groups.items():
            if len(pts) < 2:
                continue
            span = max(((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5
                       for a in pts for b in pts)
            rows.append((os.path.basename(p).replace('.kicad_pcb', ''),
                         ref, len(pts), span))
    for r in sorted(rows, key=lambda r: -r[3]):
        print('%-22s %-8s %4d %14.2f' % r)
    if not rows:
        print('(no duplicate references in the tracked corpus)')


def main():
    paths = corpus_boards()
    if not paths:
        print('SKIP: run_utils.corpus_boards() returned nothing -- git could '
              'not answer, so there is no identified board set to measure.')
        return SKIP_EXIT
    print('#726 writer teleport over %d git-TRACKED boards.' % len(paths))
    workdir = tempfile.mkdtemp(prefix='m726_')
    try:
        table_e(paths, workdir)
        table_f(paths, workdir)
        table_g(paths)
    finally:
        shutil.rmtree(workdir, ignore_errors=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
