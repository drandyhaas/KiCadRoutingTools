#!/usr/bin/env python3
"""#1368: check_complete without --authored-from read DONE on copper below the
floors the project once declared.

The writeback rewrites the project's floors down to what the chain placed and
records the originals in `kicad_routing_tools.fab_floor_origin`. check_complete
used to answer "nothing left to compare against" and say nothing. Now:

  1. A recorded origin is the reference when no --authored-from is given:
     copper below it is `relaxed`, source `fab_floor_origin`, and DISCLOSED in
     the verdict's reason -- not UNSOUND: the chain's own descent lowered it
     and says so; --authored-from grades it.
  2. An origin the copper honours leaves nothing relaxed.
  3. With no origin and no --authored-from the floor check did not run, and
     the verdict NAMES fab_floors as unexamined with the reason.
  4. --authored-from still takes precedence over the origin.

    python3 tests/test_1368_floor_origin_fallback.py
"""
import contextlib
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT)
sys.path.insert(0, TESTS_DIR)
for _p in ('py_router', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _p))

import run_utils                                               # noqa: E402
from check_complete import fab_floor_integrity                 # noqa: E402
from copy_board import copy_board                              # noqa: E402

SRC = os.path.join(ROOT, 'kicad_files', 'routed_output.kicad_pcb')
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def stage(tmp, name, origin=None):
    dst = os.path.join(tmp, name)
    with contextlib.redirect_stdout(io.StringIO()):
        copy_board(SRC, dst)
    pro = dst[:-len('.kicad_pcb')] + '.kicad_pro'
    doc = {}
    if os.path.isfile(pro):
        with open(pro, encoding='utf-8') as fh:
            doc = json.load(fh)
    doc.pop('kicad_routing_tools', None)
    if origin is not None:
        doc['kicad_routing_tools'] = {'fab_floor_origin': origin}
    with open(pro, 'w', encoding='utf-8') as fh:
        json.dump(doc, fh)
    return dst


def cc(board, *extra):
    js = board + '.cc.json'
    r = subprocess.run([sys.executable, '-X', 'utf8',
                        os.path.join(ROOT, 'check_complete.py'), board,
                        '--json', js, *extra], cwd=ROOT, capture_output=True,
                       text=True, encoding='utf-8', errors='replace')
    run_utils.evidence(js, 'check_complete json')
    with open(js, encoding='utf-8') as fh:
        return r.returncode, json.load(fh)


def main():
    tmp = tempfile.mkdtemp(prefix='t1368_')
    try:
        # 1. an origin the copper breaks
        b1 = stage(tmp, 'broken.kicad_pcb',
                   {'min_via_diameter': 5.0, 'min_track_width': 5.0})
        f = fab_floor_integrity(b1, None)
        check('origin used as the reference', f.get('ran') is True
              and f.get('source') == 'fab_floor_origin', json.dumps(f)[:300])
        keys = sorted(r['key'] for r in f.get('relaxed', ()))
        check('copper below the origin is relaxed',
              keys == ['min_track_width', 'min_via_diameter'], str(keys))
        code, doc = cc(b1)
        check('disclosed, not UNSOUND', doc.get('verdict') != 'UNSOUND'
              and code != 5 and 'fab_floor_origin' in (doc.get('reason') or '')
              and 'track width 5.0 -> 0.1' in (doc.get('reason') or ''),
              f"{doc.get('verdict')} exit {code}: {(doc.get('reason') or '')[-300:]}")

        # 2. an origin the copper honours
        b2 = stage(tmp, 'honoured.kicad_pcb',
                   {'min_via_diameter': 0.01, 'min_track_width': 0.01})
        f = fab_floor_integrity(b2, None)
        check('origin honoured: ran, nothing relaxed',
              f.get('ran') is True and not f.get('relaxed'), json.dumps(f)[:300])

        # 3. no origin, no reference: named, with the reason
        b3 = stage(tmp, 'bare.kicad_pcb')
        f = fab_floor_integrity(b3, None)
        check('no origin: the check did not run',
              f.get('ran') is False and 'fab_floor_origin' in f.get('reason', ''),
              json.dumps(f)[:300])
        code, doc = cc(b3)
        check('the verdict names fab_floors as unexamined',
              any(str(u).startswith('fab_floors') for u in doc.get('ungraded', ()))
              and 'fab_floors (' in doc.get('reason', ''),
              doc.get('reason', '')[-300:])

        # 4. --authored-from takes precedence over the recorded origin
        b4 = stage(tmp, 'ref.kicad_pcb')
        f = fab_floor_integrity(b1, b4)
        check('--authored-from wins over the origin',
              f.get('source') == 'authored_from' and not f.get('relaxed'),
              json.dumps(f)[:300])
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
