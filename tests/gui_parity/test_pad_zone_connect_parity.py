#!/usr/bin/env python3
"""Parity gate: Pad.zone_connection reads the same on both parse paths, and
KiCad's own fill agrees with what it means.

The board is tests/test_pad_zone_connect.py's: a GND pour on B.Cu (solid)
under J1 (pad 1 `(zone_connect 0)`, pad 2 inheriting) and J2 (footprint-level
`(zone_connect 0)`, pad 2 back to solid). Checked:

- the text parse and build_pcb_data_from_board give every pad and footprint
  the same zone_connection, equal to pcbnew's native
  GetZoneConnectionOverrides / GetLocalZoneConnection;
- compare_pcb_data is clean;
- after pcbnew refills the pour, exactly the pads our connectivity grade calls
  open (J1.1, J2.1) are the ones KiCad leaves out of the pour's cluster.

Needs pcbnew; re-execs into KiCad's python automatically. Exits 2 when no
pcbnew python is found, rather than 0.

    python3 -X utf8 tests/gui_parity/test_pad_zone_connect_parity.py
"""
import glob
import os
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, 'py_router'))
from kicad_locate import path_version_key  # noqa: E402
del sys.path[0]
KICAD_PYTHONS = [
    "/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3",
    "/usr/bin/python3",
    os.path.expandvars(r"C:\Program Files\KiCad\bin\python.exe"),
    *sorted(glob.glob(r"C:\Program Files\KiCad\*\bin\python.exe"),
           key=path_version_key, reverse=True),
]


def _reexec_into_kicad():
    for cand in KICAD_PYTHONS:
        if cand == sys.executable or not os.path.exists(cand):
            continue
        if subprocess.run([cand, '-c', 'import pcbnew'],
                          capture_output=True).returncode == 0:
            argv = [cand, '-X', 'utf8', os.path.abspath(__file__)] + sys.argv[1:]
            if os.name == 'nt':
                sys.exit(subprocess.run(argv).returncode)
            os.execv(cand, argv)
    print("NOT RUN: no python with pcbnew found (exit 2, not a pass)")
    sys.exit(2)


FAILURES = []


def check(cond, what, detail=''):
    print(('  ok    ' if cond else '  FAIL  ') + what
          + ('' if cond or not detail else '  -- ' + detail))
    if not cond:
        FAILURES.append(what)


def main():
    try:
        import pcbnew  # noqa: F401
    except ImportError:
        _reexec_into_kicad()
    import pcbnew
    sys.path.insert(0, os.path.join(REPO, 'py_router'))
    sys.path.insert(0, os.path.join(REPO, 'tests'))
    from kicad_parser import (parse_kicad_pcb, build_pcb_data_from_board,
                              compare_pcb_data, ZONE_CONNECT_NAMES)
    from check_connected import check_net_connectivity
    import test_pad_zone_connect as fixture

    print('KiCad build: %s' % pcbnew.GetBuildVersion())
    path = fixture.write_board(tempfile.mkdtemp(prefix='krt_zone_connect_parity_'))
    board = pcbnew.LoadBoard(path)
    check(board is not None, 'pcbnew loads the fixture')
    if board is None:
        return 1
    f = parse_kicad_pcb(path)
    g = build_pcb_data_from_board(board)

    print('\nparse paths')
    native_pad, native_fp = {}, {}
    for fp in board.GetFootprints():
        ref = fp.GetReference()
        native_fp[ref] = ZONE_CONNECT_NAMES.get(int(fp.GetLocalZoneConnection()))
        for p in fp.Pads():
            native_pad[(ref, p.GetNumber())] = ZONE_CONNECT_NAMES.get(
                int(p.GetZoneConnectionOverrides()))
    for label, model in (('text parse', f), ('pcbnew builder', g)):
        got_pad = {(fp.reference, p.pad_number): p.zone_connection
                   for fp in model.footprints.values() for p in fp.pads}
        got_fp = {fp.reference: fp.zone_connection for fp in model.footprints.values()}
        check(got_pad == native_pad,
              '%s: every pad\'s zone_connection is pcbnew\'s GetZoneConnectionOverrides'
              % label, 'got %s native %s' % (got_pad, native_pad))
        check(got_fp == native_fp,
              '%s: every footprint\'s zone_connection is pcbnew\'s GetLocalZoneConnection'
              % label, 'got %s native %s' % (got_fp, native_fp))
    check(native_pad.get(('J1', '1')) == 'none' and native_fp.get('J2') == 'none',
          'witness: the fixture carries a pad-level and a footprint-level NONE',
          repr((native_pad, native_fp)))
    diffs = compare_pcb_data(g, f)
    check(not diffs, 'compare_pcb_data is clean', str(diffs[:4]))

    print('\nKiCad\'s fill as the oracle')
    filler = pcbnew.ZONE_FILLER(board)
    filler.Fill(board.Zones())
    board.BuildConnectivity()
    conn = board.GetConnectivity()
    kicad_open = set()
    for fp in board.GetFootprints():
        for p in fp.Pads():
            joined = [i for i in conn.GetConnectedItems(p) if i.GetClass() == 'ZONE']
            if not joined:
                kicad_open.add((fp.GetReference(), p.GetNumber()))
    gnd = next(n for n, v in g.nets.items() if v.name == 'GND')
    pads = g.pads_by_net[gnd]
    r = check_net_connectivity(gnd, [], [], pads, [z for z in g.zones if z.net_id == gnd],
                               pcb_data=g)
    ours = set()
    for (x, y, _l, ref) in (r.get('disconnected_pads') or []):
        for p in pads:
            if p.component_ref == ref and abs(p.global_x - x) < 1e-6 and abs(p.global_y - y) < 1e-6:
                ours.add((ref, p.pad_number))
    check(kicad_open == {('J1', '1'), ('J2', '1')},
          'KiCad leaves exactly the two NONE pads off the pour', repr(sorted(kicad_open)))
    check(ours == kicad_open, 'our grade of the pcbnew-built board names the same pads',
          'ours %s kicad %s' % (sorted(ours), sorted(kicad_open)))

    print()
    if FAILURES:
        print('FAILED: %d check(s): %s' % (len(FAILURES), FAILURES[:6]))
        return 1
    print('PASS: zone connection overrides read and mean what pcbnew says')
    return 0


if __name__ == '__main__':
    sys.exit(main())
