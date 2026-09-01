#!/usr/bin/env python3
"""RefLabel parity gate (#481): text parser vs pcbnew, plus the PERMANENT
angle-semantics pin.

Both parse paths must fill `Footprint.ref_label` with the SAME normalized
values -- that is the contract that lets the beautify_labels engine make
identical decisions on the CLI (text-parsed PCBData) and in the GUI
(pcbnew-backed PCBData). This gate compares every field on every footprint
of two boards chosen for coverage: splitflap_driver (rotated footprints,
19 hidden labels) and glasgow_revC (94 mirrored B-side labels).

The angle gate is the load-bearing part: the implementation-time probe
settled that the stored Reference angle is the label's ABSOLUTE board
rotation (GetTextAngle() returns the file bytes unchanged; GetDrawRotation()
is the keep-upright fold of the stored value with the footprint rotation
playing no part). `placement.labels.label_world_angle` encodes that verdict,
and this gate pins it against pcbnew's own GetDrawRotation() forever -- a
KiCad release that changes the storage convention fails here, loudly.

Needs pcbnew; re-execs into KiCad's python automatically.

    python3 tests/gui_parity/test_ref_label_pcbnew_parity.py
"""
import glob
import os
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, 'py_router'))  # #522
sys.path.insert(0, os.path.join(REPO, 'py_placer'))  # placement split
sys.path.insert(0, os.path.join(REPO, 'py_tools'))  # #522

KICAD_PYTHONS = [
    "/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3",
    "/usr/bin/python3",
    os.path.expandvars(r"C:\\Program Files\\KiCad\\bin\\python.exe"),
    # KiCad 9/10 install under a VERSIONED directory on Windows
    # (C:\Program Files\KiCad\10.0\bin). Newest first.
    *sorted(glob.glob(r"C:\Program Files\KiCad\*\bin\python.exe"), reverse=True),
]

BOARDS = [
    os.path.join(REPO, 'kicad_files', 'splitflap_driver.kicad_pcb'),
    os.path.join(REPO, 'kicad_files', 'glasgow_revC.kicad_pcb'),
]

TOL = 1e-6  # mm; both sides are nm-quantized so real agreement is exact


def _reexec_into_kicad():
    for cand in KICAD_PYTHONS:
        if cand == sys.executable:
            continue
        if os.path.exists(cand):
            r = subprocess.run([cand, '-c', 'import pcbnew'],
                               capture_output=True)
            if r.returncode == 0:
                argv = [cand, os.path.abspath(__file__)] + sys.argv[1:]
                if os.name == 'nt':
                    # os.execv re-splits argv on spaces through the CRT on
                    # Windows ("C:\Program Files\..." tears in two);
                    # subprocess quotes correctly.
                    sys.exit(subprocess.run(argv).returncode)
                os.execv(cand, argv)
    print("ERROR: no python with pcbnew found")
    sys.exit(2)


def compare_board(board_path):
    import pcbnew
    from kicad_parser import (build_pcb_data_from_board, parse_kicad_pcb,
                              disambiguate_references)
    from placement.labels import (fold_upright, label_world_anchor,
                                  label_world_angle)

    text_pcb = parse_kicad_pcb(board_path)
    board = pcbnew.LoadBoard(board_path)
    gui_pcb = build_pcb_data_from_board(board)

    # PCBData key -> live pcbnew footprint, resolved with the SAME function
    # both parse paths use (#726). This used to reproduce the last-wins
    # collapse deliberately, because both dicts collapsed the same way and it
    # had to agree with them -- so on glasgow, six of the seven `REF**` labels
    # were never compared at all. Every block is a key now, so the gate covers
    # all seven, and it becomes a THIRD independent check that
    # `disambiguate_references` answers the same over the live board as it does
    # over each parse path's own ordering.
    _live = list(board.GetFootprints())
    live_fps = dict(zip(
        disambiguate_references(
            [f.GetReference() or ('#' + f.m_Uuid.AsString()) for f in _live]),
        _live))

    problems = []
    checked = angles = 0
    for ref, tfp in text_pcb.footprints.items():
        if ref.startswith('#'):
            continue  # reference-less drill dots carry no label
        tl = tfp.ref_label
        gfp = gui_pcb.footprints.get(ref)
        gl = gfp.ref_label if gfp is not None else None
        if tl is None or gl is None:
            problems.append(f"{ref}: label {'text' if tl is None else 'gui'} "
                            f"side is None")
            continue
        checked += 1
        for fld in ('at_x', 'at_y', 'rotation', 'size_h', 'size_w',
                    'thickness'):
            tv, gv = getattr(tl, fld), getattr(gl, fld)
            if abs(tv - gv) > TOL:
                problems.append(f"{ref}.{fld}: text {tv} vs pcbnew {gv}")
        for fld in ('layer', 'justify', 'hidden', 'is_property_node'):
            tv, gv = getattr(tl, fld), getattr(gl, fld)
            if tv != gv:
                problems.append(f"{ref}.{fld}: text {tv!r} vs pcbnew {gv!r}")

        # World anchor: local_to_global on the text values must land on
        # pcbnew's own GetPosition() (nm-snapped, so exact agreement).
        pcb_ref = live_fps[ref].Reference()
        pos = pcb_ref.GetPosition()
        wx, wy = label_world_anchor(tfp, tl)
        if abs(wx - pos.x / 1e6) > TOL or abs(wy - pos.y / 1e6) > TOL:
            problems.append(f"{ref}.anchor: text ({wx}, {wy}) vs pcbnew "
                            f"({pos.x / 1e6}, {pos.y / 1e6})")

        # THE angle gate: label_world_angle == GetDrawRotation for
        # keep-upright labels (the default); == the raw stored angle
        # otherwise. Fold both into (-90, 90] before comparing so a 270-vs-
        # -90 spelling difference can't fake a failure.
        try:
            dr = pcb_ref.GetDrawRotation().AsDegrees()
        except AttributeError:
            dr = pcb_ref.GetDrawRotation() / 10.0
        try:
            keep_upright = pcb_ref.IsKeepUpright()
        except Exception:
            keep_upright = True
        angles += 1
        if keep_upright:
            if abs(label_world_angle(tfp, tl) - fold_upright(dr)) > TOL:
                problems.append(
                    f"{ref}.angle: label_world_angle "
                    f"{label_world_angle(tfp, tl)} vs GetDrawRotation {dr}")
        else:
            if abs((tl.rotation - dr) % 360) > TOL:
                problems.append(
                    f"{ref}.angle(no-keep-upright): stored {tl.rotation} "
                    f"vs GetDrawRotation {dr}")
    return checked, angles, problems


def main():
    failed = False
    for board_path in BOARDS:
        checked, angles, problems = compare_board(board_path)
        name = os.path.basename(board_path)
        if problems:
            failed = True
            print(f"FAIL {name}: {len(problems)} mismatch(es) over "
                  f"{checked} label(s)")
            for p in problems[:25]:
                print(f"  {p}")
            if len(problems) > 25:
                print(f"  ... and {len(problems) - 25} more")
        else:
            print(f"PASS {name}: {checked} labels field-identical, "
                  f"{angles} draw-rotation checks")
        # Coverage floors: the boards were chosen FOR these populations, so
        # a parser regression that silently drops them must fail the gate.
        if name == 'splitflap_driver.kicad_pcb' and checked < 65:
            failed = True
            print(f"FAIL {name}: only {checked} labels checked (expected 65)")
        if name == 'glasgow_revC.kicad_pcb' and checked < 90:
            failed = True
            print(f"FAIL {name}: only {checked} labels checked (expected 90+)")
    return 1 if failed else 0


if __name__ == '__main__':
    try:
        import pcbnew  # noqa: F401
    except ImportError:
        _reexec_into_kicad()
    sys.exit(main())
