#!/usr/bin/env python3
"""#726 parity gate: both parse paths must name a duplicated block the same.

`parse_kicad_pcb` walks the file; `build_pcb_data_from_board` walks a live
pcbnew BOARD. When two footprint blocks claim one reference, the name that
separates them is a FILE-ORDER ORDINAL, which is a property of the ordered
list rather than of any one footprint -- so the two paths agree only if they
enumerate footprints in the same order. This gate is where that is checked.

WHY A KEY-SET COMPARISON IS NOT ENOUGH, and why the old harness was blind.
`compare_pcb_data` is the existing two-path diff, and before #726 it could not
see this defect at all: both paths dropped the SAME block, so their key sets
matched and it reported perfect parity on a board that had lost six
footprints. The residual hazard now is the mirror image -- two paths that
disambiguate in OPPOSITE order still agree on the key SET (`{TP4, TP4~2}`
either way) while `TP4` means a different physical part on each side. So the
load-bearing arm here is per-key identity: same position, same rotation, same
layer, same uuid.

THE SELF-EXPIRING ARM. `board.GetFootprints()` returning KiCad's footprints in
file order is an empirical fact about one release, not a documented contract.
Measured on KiCad 10.0.0 it holds on all 22 tracked corpus boards, for both the
uuid sequence and the reference sequence. The arm named "GetFootprints() still
iterates in FILE ORDER" pins it, and if a future KiCad breaks it the ordinal
scheme is unsound and must move to a different key -- DO NOT relax that arm to
make it pass.

Needs pcbnew; re-execs into KiCad's python automatically. Lives in
`tests/gui_parity/` because `run_all.py`'s glob only collects `tests/test_*.py`
and must never collect a gate that re-execs.

    python3 -X utf8 tests/gui_parity/test_726_parse_path_parity.py
"""
import glob
import os
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

KICAD_PYTHONS = [
    "/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3",
    "/usr/bin/python3",
    os.path.expandvars(r"C:\Program Files\KiCad\bin\python.exe"),
    # KiCad 9/10 install under a VERSIONED directory on Windows. Newest first.
    *sorted(glob.glob(r"C:\Program Files\KiCad\*\bin\python.exe"), reverse=True),
]

#: The five tracked boards that carry a duplicate reference, plus two clean
#: controls. Named rather than globbed: a board that stops being a witness must
#: make this gate FAIL loudly, not quietly reduce its own coverage.
WITNESSES = ('esp_prog', 'glasgow_revC', 'orangecrab_ext_pll', 'ulx3s',
             'watchy')
CONTROLS = ('tigard', 'cap_chain')

TOL = 1e-6      # mm; both sides are nm-quantized, so real agreement is exact


def _reexec_into_kicad():
    for cand in KICAD_PYTHONS:
        if cand == sys.executable or not os.path.exists(cand):
            continue
        if subprocess.run([cand, '-c', 'import pcbnew'],
                          capture_output=True).returncode == 0:
            argv = [cand, '-X', 'utf8', os.path.abspath(__file__)] + sys.argv[1:]
            if os.name == 'nt':
                # os.execv re-splits argv on spaces through the CRT on Windows
                # ("C:\Program Files\..." tears in two); subprocess quotes it.
                sys.exit(subprocess.run(argv).returncode)
            os.execv(cand, argv)
    print("SKIP: no python with pcbnew found")
    sys.exit(0)


FAILURES = []


def check(cond, what, detail=''):
    if cond:
        print('  ok    %s' % what)
    else:
        print('  FAIL  %s%s' % (what, ('  -- ' + detail) if detail else ''))
        FAILURES.append(what)


def main():
    try:
        import pcbnew  # noqa: F401
    except ImportError:
        _reexec_into_kicad()

    import pcbnew
    for _p in ('', 'py_router', 'py_placer', 'py_tools'):
        _d = os.path.join(REPO, _p)
        if _d not in sys.path:
            sys.path.insert(0, _d)
    from kicad_parser import (parse_kicad_pcb, build_pcb_data_from_board,
                              compare_pcb_data, disambiguate_references,
                              iter_footprint_blocks, footprint_uuid)

    print('KiCad build: %s' % pcbnew.GetBuildVersion())
    boards = []
    for name in WITNESSES + CONTROLS:
        p = os.path.join(REPO, 'kicad_files', name + '.kicad_pcb')
        if not os.path.exists(p):
            check(False, 'board %s is present' % name,
                  '%s is gone; this gate covers a NAMED set, so replace the '
                  'witness rather than letting coverage shrink silently' % p)
            continue
        boards.append((name, p))

    for name, p in boards:
        print('\n%s' % name)
        f = parse_kicad_pcb(p)
        board = pcbnew.LoadBoard(p)
        g = build_pcb_data_from_board(board)

        check(set(f.footprints) == set(g.footprints),
              '%s: both paths produce the same key set' % name,
              'only file=%s only board=%s'
              % (sorted(set(f.footprints) - set(g.footprints))[:6],
                 sorted(set(g.footprints) - set(f.footprints))[:6]))

        # The arm that key-set equality cannot give: two paths that
        # disambiguate in opposite order agree on the SET while `TP4` means a
        # different part on each side.
        bad = []
        for k in sorted(set(f.footprints) & set(g.footprints)):
            a, b = f.footprints[k], g.footprints[k]
            if (abs(a.x - b.x) > TOL or abs(a.y - b.y) > TOL
                    or abs((a.rotation or 0.0) - (b.rotation or 0.0)) > TOL
                    or (a.layer or '') != (b.layer or '')
                    or (a.uuid or '') != (b.uuid or '')):
                bad.append((k, (a.x, a.y, a.rotation, a.layer, a.uuid),
                            (b.x, b.y, b.rotation, b.layer, b.uuid)))
        check(not bad,
              '%s: each key lands on the SAME physical footprint' % name,
              str(bad[:3]))

        check(f.duplicate_references == g.duplicate_references,
              '%s: both paths report the same duplicate_references' % name,
              '%s != %s' % (f.duplicate_references, g.duplicate_references))

        # Drive the shipped comparator rather than re-deriving its checks: it
        # is what `py_tools/validate_pcb_data.py` runs, and #726 taught it to
        # name a duplicate-reference disagreement directly.
        diffs = [d for d in compare_pcb_data(g, f)
                 if 'ootprint' in d or 'Duplicate references' in d]
        check(not diffs, '%s: compare_pcb_data reports no footprint diff' % name,
              str(diffs[:3]))

        # --- the self-expiring arm ---
        with open(p, encoding='utf-8', errors='replace') as fh:
            content = fh.read()
        file_blocks = list(iter_footprint_blocks(content))
        file_uuids = [footprint_uuid(b[2]) for b in file_blocks]
        file_raw = [b[3] for b in file_blocks]
        live = list(board.GetFootprints())
        live_uuids = [fp.m_Uuid.AsString() for fp in live]
        live_raw = []
        for fp in live:
            r = fp.GetReference()
            live_raw.append(r if r else '#' + fp.m_Uuid.AsString())
        check(file_uuids == live_uuids and file_raw == live_raw,
              '%s: GetFootprints() still iterates in FILE ORDER' % name,
              'KiCad footprint iteration order is not a documented contract. '
              'It matched file order on 10.0.0 for every tracked board. If '
              'this fails on a newer KiCad the file-order ordinal key is '
              'UNSOUND and must move to a different scheme -- do not relax '
              'this arm. First divergence at index %s'
              % next((i for i, (a, b) in enumerate(zip(file_uuids, live_uuids))
                      if a != b), 'len mismatch %d vs %d'
                     % (len(file_uuids), len(live_uuids))))
        check(disambiguate_references(file_raw)
              == disambiguate_references(live_raw),
              '%s: the shared key function agrees on both orderings' % name)

    print('\n%d failure(s)' % len(FAILURES))
    for f in FAILURES:
        print('  FAILED: %s' % f)
    return 1 if FAILURES else 0


if __name__ == '__main__':
    sys.exit(main())
