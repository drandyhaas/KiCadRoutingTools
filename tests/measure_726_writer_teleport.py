#!/usr/bin/env python3
"""#726 measurement: what `write_placed_output` does to a footprint's twin.

`placement/writer.py:write_placed_output` finds footprint blocks in the
.kicad_pcb TEXT and matches each one's `(property "Reference" "XX")` against a
dict keyed by reference (`writer.py:184`, `:201-208`). Nothing consumes an
entry, so ONE placement rewrites EVERY block sharing that reference.

Four tables:

  E1 THE HEADLINE -- the identity write, as the shipped code actually performs
     it. `placement/perturb.py:_all_at_current` hands the writer every part at
     the pose it ALREADY has; its docstring says "Poses are unchanged for
     non-members -- only their formatting is", and `perturb.py:572` builds the
     stress harness's ground-truth CONTROL board with it. This table IMPORTS
     and CALLS that function rather than reimplementing it.

  E2 the UPPER BOUND -- one placement per PARSED footprint. No shipped caller
     produces this, and the gap between E2 and E1 is the finding rather than
     the noise: `_all_at_current` iterates `sorted(state.parts)`, and
     `quench.py:815-825` admits a zero-pad footprint to `state.parts` only when
     the board draws it a courtyard, while `portfolio.free_refs` and
     `portfolio.py:318` filter on `fp.pads` outright. An earlier draft of this
     script reported E2 AS E1 and inflated the headline fourfold; an
     independent fact-check caught it. Both numbers are reported, and labelled.

  F  the single-placement teleport -- one placement for one duplicated
     reference, and how many blocks move. Includes the writer's own
     "Modified N footprint positions" line, which counts BLOCKS, not
     placements, and is the only existing observable of the defect. A control
     row must move exactly ONE block or it says so instead of claiming to be a
     control.

  G  twin span -- the widest gap inside a shared name.

Poses, not bytes. The writer reformats every `(at)` it touches to `:.6f`
(`writer.py:224-237`), so `(at 141.2 95.9)` becomes `(at 141.200000 95.900000)`
with the part in the same place. Measured, a byte comparison over-reports by
1317 blocks against 12 real E2 moves.

Board set from `run_utils.corpus_boards()` (git ls-files), never a glob: one
gitignored generated board in `kicad_files/` moves every headline here (E2's 12
relocations become 14, and "17 of 22" becomes "17 of 23").

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

    Blocks are matched across a write POSITIONALLY, by index in this list --
    the writer never reorders, adds or drops one. The uuid rides along as a
    label for the report and as a CHECK (`_paired` asserts the sequence is
    unchanged), never as the key: `cap_chain` shares one footprint uuid across
    C1/C2 and another across J1/J2, so a uuid key would be ambiguous on a real
    board in this very corpus.
    """
    return [(fp_uuid(t), raw_reference(t), pose(t))
            for _, _, t in blocks(_read(path))]


def _paired(before, after, where):
    """Zip two block lists, refusing if the write disturbed the sequence."""
    assert len(before) == len(after), (
        '%s: block count changed %d -> %d; the writer must never add or drop '
        'a block' % (where, len(before), len(after)))
    assert [b[0] for b in before] == [a[0] for a in after], (
        '%s: the uuid SEQUENCE changed across the write, so blocks can no '
        'longer be matched by index and every displacement below is suspect'
        % where)
    return list(zip(before, after))


def _write(src, dst, placements):
    """Run the writer; return its `Modified N footprint positions` line."""
    buf = io.StringIO()
    with redirect_stdout(buf):
        write_placed_output(src, dst, placements)
    out = buf.getvalue()
    # The writer prints a success line last; the number this measurement is
    # about is `Modified N footprint positions` (writer.py:385), which counted
    # BLOCKS rewritten rather than placements supplied.
    for line in reversed(out.splitlines()):
        if line.startswith('Modified ') and 'footprint positions' in line:
            return line.strip()
    return '(no Modified line)'


def _moved(before, after, where):
    out = []
    for (uu, ref, (x0, y0, r0)), (_, _, (x1, y1, r1)) in _paired(
            before, after, where):
        d = ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5
        if d > TOL or abs(r1 - r0) > TOL:
            out.append((uu, ref, (x0, y0), (x1, y1), d))
    return out


def real_all_at_current(pcb, board_path):
    """What `perturb._all_at_current` ACTUALLY produces for this board.

    Imported and called, not reimplemented -- reimplementing it is precisely
    the mistake this table used to make. It iterates `sorted(state.parts)`, and
    `quench.py:815-825` admits a zero-pad footprint to `state.parts` only when
    the board draws it a courtyard. Every one of glasgow's 7 `REF**`,
    orangecrab's 3 `G***` and ulx3s' 2 `EMARD` is zero-pad with NO courtyard
    (Table B's `pad` and `CrtYd` columns say so), so no shipped caller can hand
    the writer a placement for them.
    """
    from placement.perturb import _all_at_current
    from pose_score import make_state
    with redirect_stdout(io.StringIO()):
        st = make_state(pcb, board_path)
        return _all_at_current(st, [], pcb)


def _identity_arm(paths, workdir, build, title, note):
    print()
    print(title)
    print()
    print(note)
    print()
    hdr = ('%-34s %7s %12s %10s  %s'
           % ('board', 'blocks', 'placements', 'MOVED',
              'max displacement (mm)'))
    print(hdr)
    print('-' * len(hdr))
    total_moved = 0
    zero_rows = 0
    detail = []
    for p in paths:
        before = block_poses(p)
        pcb = parse_kicad_pcb(p)
        placements = build(pcb, p)
        dst = os.path.join(workdir, os.path.basename(p))
        _write(p, dst, placements)
        moved = _moved(before, block_poses(dst), os.path.basename(p))
        total_moved += len(moved)
        if not moved:
            zero_rows += 1
        print('%-34s %7d %12d %10d  %.2f'
              % (os.path.basename(p)[:34], len(before), len(placements),
                 len(moved), max([m[4] for m in moved], default=0.0)))
        for m in moved:
            detail.append((os.path.basename(p), m))
    print()
    print('TOTAL blocks relocated by a write whose contract is that nothing '
          'moves: %d.  Boards reading 0: %d of %d.'
          % (total_moved, zero_rows, len(paths)))
    if detail:
        print()
        print('Every relocation, named:')
        print('  %-24s %-8s %-9s %-22s %-22s %8s'
              % ('board', 'ref', 'uuid', 'from', 'to', 'mm'))
        for b, (uu, ref, a, z, d) in detail:
            print('  %-24s %-8s %-9s (%8.3f,%8.3f) -> (%8.3f,%8.3f) %8.2f'
                  % (b.replace('.kicad_pcb', '')[:24], ref, uu[-8:],
                     a[0], a[1], z[0], z[1], d))
    return total_moved


E1_NOTE = ("This is the SHIPPED call. `perturb.py:572` builds the stress\n"
           "harness's ground-truth CONTROL board with it, and its docstring\n"
           'says "Poses are unchanged for non-members -- only their\n'
           'formatting is."')

E2_NOTE = ("NOT reachable by any shipped caller, and the gap from E1 is the\n"
           "point rather than the noise. `_all_at_current` iterates\n"
           "`sorted(state.parts)`; `quench.py:815-825` admits a zero-pad\n"
           "footprint only when the board draws it a courtyard, and\n"
           "`portfolio.free_refs` / `portfolio.py:318` filter on `fp.pads`\n"
           "outright. Every block E2 relocates that E1 does not is zero-pad\n"
           "with no courtyard -- cross-read Table B's `pad` and `CrtYd`\n"
           "columns. Reported as the exposure a future caller inherits if that\n"
           "filter ever moves, NOT as a count of damage done.")


def table_e(paths, workdir):
    e1 = _identity_arm(
        paths, workdir, real_all_at_current,
        '== TABLE E1 -- the REAL identity write (perturb._all_at_current) ==',
        E1_NOTE)
    e2 = _identity_arm(
        paths, workdir,
        lambda pcb, path: [{'reference': r, 'new_x': f.x, 'new_y': f.y,
                            'new_rotation': f.rotation}
                           for r, f in pcb.footprints.items()],
        '== TABLE E2 -- the UPPER BOUND: one placement per PARSED footprint ==',
        E2_NOTE)
    print()
    print('E1 (reachable today) = %d blocks.  E2 (upper bound) = %d blocks.'
          % (e1, e2))
    return e1


def table_f(paths, workdir):
    print()
    print('== TABLE F -- one placement for one reference ==')
    print()
    hdr = ('%-22s %-8s %7s %7s %7s  %s'
           % ('board', 'ref', 'blocks', 'placed', 'MOVED',
              "writer's own stdout"))
    print(hdr)
    print('-' * len(hdr))
    detail = []
    for p in paths:
        before = block_poses(p)
        counts = {}
        for _uu, ref, _pz in before:
            counts[ref] = counts.get(ref, 0) + 1
        # The board's first duplicated reference in file order; on a clean
        # board, its first reference -- the one-name-one-block control.
        target = next((r for _u, r, _z in before if counts[r] > 1), None)
        control = target is None
        if control:
            target = before[0][1]
        dst = os.path.join(workdir, 'f_' + os.path.basename(p))
        out = _write(p, dst, [{'reference': target, 'new_x': 120.0,
                               'new_y': 110.0, 'new_rotation': 0.0}])
        moved = _moved(before, block_poses(dst), os.path.basename(p))
        # A control that cannot fail is not a control. A clean board must move
        # exactly ONE block for one placement, and the label is earned only
        # when it did -- a block whose Reference property is EMPTY parses under
        # a `#uuid` key, so a target read off the file text can name something
        # the writer cannot address, which reads as 0.
        if not control:
            note = ''
        elif len(moved) == 1:
            note = '   <- CONTROL: one name, one block'
        else:
            note = '   <- control BROKEN: expected 1 move, got %d' % len(moved)
        print('%-22s %-8s %7d %7d %7d  %s%s'
              % (os.path.basename(p).replace('.kicad_pcb', '')[:22], target,
                 counts[target], 1, len(moved), out, note))
        for m in moved:
            detail.append((os.path.basename(p), target, m))
    print()
    print('Every block moved, named:')
    print('  %-22s %-8s %-9s %-22s %-22s %8s'
          % ('board', 'ref', 'uuid', 'from', 'to', 'mm'))
    for b, _tgt, (uu, ref, a, z, d) in detail:
        print('  %-22s %-8s %-9s (%8.3f,%8.3f) -> (%8.3f,%8.3f) %8.2f'
              % (b.replace('.kicad_pcb', '')[:22], ref, uu[-8:],
                 a[0], a[1], z[0], z[1], d))


def table_g(paths):
    print()
    print('== TABLE G -- twin span: the widest gap inside a shared name ==')
    print()
    print('For a 2-member group this IS the distance the loser travels when')
    print("the winner's pose is stamped on it. For glasgow's 7-member group it")
    print('is the widest pair among them, which no single block travels --')
    print("E2's 70.66 mm is that board's real maximum.")
    print()
    hdr = '%-22s %-8s %4s %14s' % ('board', 'ref', 'n', 'max span (mm)')
    print(hdr)
    print('-' * len(hdr))
    rows = []
    for p in paths:
        groups = {}
        for _uu, ref, (x, y, _r) in block_poses(p):
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
