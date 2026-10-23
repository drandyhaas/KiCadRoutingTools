#!/usr/bin/env python3
"""#1350: movie3d draws a DNP part as its pads and counts it apart from the
parts whose model did not resolve.

movie3d exports the board's GLB with `--no-dnp`, so a Do-Not-Populate part
never gets its model; it was then drawn as a box and reported as having
"fell back to a box" (on one board, 28 of its 102 boxes were DNP 0402s).
main's stage3d took the same fix; this is the branch's 3D movie.

Checks, on an in-repo board's footprints with one marked DNP:
  1. `part_scene` gives the DNP part no body (pads only) and marks it `dnp`;
     the same part not DNP keeps its body.
  2. `attach_models`, with the GLB export stubbed to match every populated
     part, reports the DNP part under `dnp`, not under `boxes`, counts only
     populated parts, and prints it as not populated.

    python3 tests/test_1350_movie3d_dnp.py
"""
import contextlib
import io
import os
import sys
import types
from types import SimpleNamespace

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'movie3d'))
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

import record                                       # noqa: E402
from kicad_parser import parse_kicad_pcb            # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')


def main():
    pcb = parse_kicad_pcb(BOARD)
    refs = sorted(r for r, fp in pcb.footprints.items()
                  if not r.startswith('#') and fp.pads)[:4]
    assert len(refs) == 4, refs
    dnp_ref = next(r for r in refs if not record._not_assembled(pcb.footprints[r]))
    fp = pcb.footprints[dnp_ref]

    kept = record.part_scene(dnp_ref, fp, None, None, False)
    assert kept['body'] is not None and kept['dnp'] is False, kept
    fp.dnp = True
    gone = record.part_scene(dnp_ref, fp, None, None, False)
    assert gone['body'] is None and gone['dnp'] is True, gone
    assert gone['pads'] == kept['pads']
    print(f"  PASS: DNP {dnp_ref} is drawn as its pads, with no body")

    parts = [record.part_scene(r, pcb.footprints[r], None, None, False) for r in refs]
    populated = [r for r in refs if r != dnp_ref]
    stub = types.ModuleType('models')
    stub.export = lambda *a, **k: ({'matched': populated, 'path': 'x.glb', 'found': [], 'missing': [],
                                    'cached': True, 'note': ''}, None)
    sys.modules['models'] = stub
    rec = SimpleNamespace(scene={'parts': parts}, model_src=(BOARD, None, BOARD), refs=refs)
    a = SimpleNamespace(tint=0.5, no_models=False, kicad_cli=None)
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        record.attach_models(a, rec)
    rep, out = rec.model_report, buf.getvalue()
    assert rep['dnp'] == [dnp_ref] and dnp_ref not in rep['boxes'], rep
    assert rep['parts'] == len(populated) and rep['models'] == len(populated), rep
    assert 'none fell back to a box' in out, out
    assert f'1 DNP part(s) not populated, so drawn as pads: {dnp_ref}' in out, out
    print(f"  PASS: the report counts {len(populated)} populated parts and names "
          f"{dnp_ref} as DNP, not as a box")
    print('test_1350_movie3d_dnp: all passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
