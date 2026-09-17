"""Native geometry and text-parser coverage on copies of esp_prog (#967).

Run with KiCad Python; re-execs into it automatically. Missing pcbnew exits 77,
never an unmeasured pass.

ONE ARM DROPPED BY THE IPC PORT, AND IT IS NAMED HERE RATHER THAN DELETED.
Main runs each assertion over BOTH parse paths -- `parse_kicad_pcb` (the file)
and `build_pcb_data_from_board` (a live pcbnew BOARD it has just mutated with
`SetChamferPositions` / `Padstack().SetMode`). On this branch
`build_pcb_data_from_board` is the KIPY builder: it cannot be handed a
`pcbnew.BOARD`, and it reaches a running KiCad over a socket, so the live arm
cannot be made in process. It cannot be faked through `fake_ipc_board` either
-- that serves the read path by re-parsing the file with `parse_kicad_pcb`, so
the "two" arms would be one, and the gate would pass whatever the kipy builder
actually did.

WHAT STILL HOLDS: everything below runs on the TEXT parser, which the port does
not touch, against boards pcbnew authored -- so the chamfer / per-layer-padstack
disclosure, the near-cardinal tilt and the unequal-axis circle are all really
graded. WHAT IS NOT COVERED: that the IPC builder discloses the same two
approximations. That is `_kipy_geometry_approximations` in
`py_router/kicad_parser.py`, whose reason strings are deliberately identical to
this parser's so a consumer cannot tell the fronts apart -- and no gate on this
branch grades it against a live KiCad. Measure it there before trusting it.
"""
import copy
import hashlib
import math
import os
import subprocess
from pathlib import Path
import sys
import tempfile

KICAD_PYTHONS = [
    '/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/'
    'Versions/Current/bin/python3',
    '/usr/bin/python3',
    r'C:\Program Files\KiCad\10.0\bin\python.exe',
]

try:
    import pcbnew
except ImportError:
    # Re-exec rather than skip: pcbnew authors the mutated fixtures the TEXT
    # arm reads, so skipping here drops coverage that does run on this branch.
    for _cand in KICAD_PYTHONS:
        if _cand == sys.executable or not os.path.exists(_cand):
            continue
        if subprocess.run([_cand, '-c', 'import pcbnew'],
                          capture_output=True).returncode == 0:
            _argv = [_cand, os.path.abspath(__file__)] + sys.argv[1:]
            if os.name == 'nt':
                raise SystemExit(subprocess.run(_argv).returncode)
            os.execv(_cand, _argv)
    print('SKIP: native pad coverage requires KiCad pcbnew')
    raise SystemExit(77)

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/'py_router'), str(ROOT/'py_placer')]
from copy_board import copy_board
from kicad_parser import parse_kicad_pcb
from placement.legality import grade_pad_edge_clearance


def main():
    source = ROOT/'kicad_files/esp_prog.kicad_pcb'
    original = hashlib.sha256(source.read_bytes()).hexdigest()
    with tempfile.TemporaryDirectory(prefix='krt967_native_') as scratch:
        path = Path(scratch)/'board.kicad_pcb'
        copy_board(str(source), str(path))
        board = pcbnew.LoadBoard(str(path))
        fp = next(f for f in board.GetFootprints() if f.GetReference() == 'Y1')
        pad = list(fp.Pads())[0]
        pad.SetShape(pcbnew.PAD_SHAPE_CHAMFERED_RECT)
        pad.SetSize(pcbnew.VECTOR2I(2000000, 1000000))
        pad.SetRoundRectRadiusRatio(.25)
        pad.SetChamferRectRatio(.05)
        pad.SetChamferPositions(15)
        pad.SetOrientation(pcbnew.EDA_ANGLE(33, pcbnew.DEGREES_T))
        pad.SetPosition(pcbnew.VECTOR2I(130000000, 104040000))
        pcbnew.SaveBoard(str(path), board)
        board = pcbnew.LoadBoard(str(path))
        fp = next(f for f in board.GetFootprints() if f.GetReference() == 'Y1')
        pad = list(fp.Pads())[0]
        # Native effective copper, not the footprint bbox or production formula.
        native_bottom = pcbnew.ToMM(pad.GetEffectivePolygon(pcbnew.F_Cu).BBox().GetBottom())
        edge_y = max(pcbnew.ToMM(point.y) for shape in board.GetDrawings()
                     if shape.GetLayer() == pcbnew.Edge_Cuts
                     for point in (shape.GetStart(), shape.GetEnd()))
        gap = edge_y - native_bottom
        assert abs(gap - .523258) < 1e-6, gap
        for parsed in (parse_kicad_pcb(str(path)),):  # live kipy arm: see the module docstring
            target = copy.deepcopy(parsed.footprints['Y1'])
            target.pads = [target.pads[0]]
            parsed.footprints = {'Y1': target}
            assert target.pads[0].geometry_approximations == ('chamfered pad',)
            result = grade_pad_edge_clearance(parsed, .55, str(path))
            assert not result['complete'], result
            assert 'chamfered pad' in result['unmeasured'][0]['reason'], result
        # Nonuniform padstacks also flatten per-layer geometry in PCBData.
        pad.SetChamferPositions(0)
        pad.Padstack().SetMode(pcbnew.PADSTACK.MODE_FRONT_INNER_BACK)
        pad.SetShape(pcbnew.F_Cu, pcbnew.PAD_SHAPE_OVAL)
        pad.SetShape(pcbnew.B_Cu, pcbnew.PAD_SHAPE_RECT)
        for layer in (pcbnew.F_Cu, pcbnew.B_Cu):
            pad.SetSize(layer, pcbnew.VECTOR2I(2000000, 1000000))
        pad.SetAttribute(pcbnew.PAD_ATTRIB_PTH)
        pad.SetLayerSet(pcbnew.LSET.AllCuMask())
        pad.SetDrillSize(pcbnew.VECTOR2I(300000, 300000))
        pcbnew.SaveBoard(str(path), board)
        board = pcbnew.LoadBoard(str(path))
        fp = next(f for f in board.GetFootprints() if f.GetReference() == 'Y1')
        pad = list(fp.Pads())[0]
        back_gap = edge_y - pcbnew.ToMM(
            pad.GetEffectivePolygon(pcbnew.B_Cu).BBox().GetBottom())
        assert abs(back_gap - .496026) < 1e-6, back_gap
        for parsed in (parse_kicad_pcb(str(path)),):  # live kipy arm: see the module docstring
            target = parsed.footprints['Y1'].pads[0]
            assert 'per-layer padstack' in target.geometry_approximations, target
            result = grade_pad_edge_clearance(parsed, .55, str(path))
            assert not result['complete'], result
            assert any('per-layer padstack' in row['reason']
                       for row in result['unmeasured']), result
        print(f'PASS: native chamfer gap {gap:.6f}, padstack gap {back_gap:.6f} mm < .55 mm; '
              'the TEXT parser discloses chamfer/padstack coverage '
              '(the live kipy arm is NOT covered -- see the module docstring)')
        # The routing broad phase snaps within one degree of cardinal axes.
        # Exact edge grading must recover the actual copper tilt in both parsers.
        for angle in (0, .5, 1, 1.0001, 89.5, 90, 90.5, 179.5, 269.5, 359.5):
            board = pcbnew.LoadBoard(str(source))
            fp = next(f for f in board.GetFootprints() if f.GetReference() == 'Y1')
            pad = list(fp.Pads())[0]
            pad.SetShape(pcbnew.PAD_SHAPE_RECT)
            pad.SetSize(pcbnew.VECTOR2I(2000000, 1000000))
            pad.SetOrientation(pcbnew.EDA_ANGLE(angle, pcbnew.DEGREES_T))
            pad.SetPosition(pcbnew.VECTOR2I(130000000, 104000000))
            pcbnew.SaveBoard(str(path), board)
            board = pcbnew.LoadBoard(str(path))
            expected = 1.5 - (abs(math.sin(math.radians(angle)))
                              + .5 * abs(math.cos(math.radians(angle))))
            for parsed in (parse_kicad_pcb(str(path)),):  # live kipy arm: see the module docstring
                target = parsed.footprints['Y1']
                target.pads = [target.pads[0]]
                parsed.footprints = {'Y1': target}
                result = grade_pad_edge_clearance(parsed, .55, str(path))
                assert result['complete'], result
                assert abs(result['minimum_gap_mm'] - expected) < 1e-6, (angle, result)
        pad = list(next(f for f in board.GetFootprints() if f.GetReference() == 'Y1').Pads())[0]
        pad.SetShape(pcbnew.PAD_SHAPE_CIRCLE)
        pad.SetSize(pcbnew.VECTOR2I(2000000, 1000000))
        pcbnew.SaveBoard(str(path), board)
        board = pcbnew.LoadBoard(str(path))
        for parsed in (parse_kicad_pcb(str(path)),):  # live kipy arm: see the module docstring
            result = grade_pad_edge_clearance(parsed, .55, str(path))
            assert not result['complete'], result
            assert any('unequal-axis circle' in row['reason'] for row in result['unmeasured'])
        print('PASS: near-cardinal exact tilt and unequal-axis circle coverage')
    assert hashlib.sha256(source.read_bytes()).hexdigest() == original


if __name__ == '__main__':
    main()
