#!/usr/bin/env python3
"""#1619 / #1620: the improvement gate grades a pour net as the re-grade does.

The gate compares the shipped board with the input through the raster fill
model. That model pours onto a pad that KiCad's thermal relief or a foreign
class's clearance keeps the fill off. So when a pass removed a GND pad's
only stub, the gate read the pad as still joined by the pour: `lost []`,
accepted. Meanwhile the end-of-run re-grade, which reads the net on KiCad's
fill, printed "Broken by this run ...: GND". Now a pour net that the re-grade
reads open while the raster credits it is graded on KiCad's exact fill of
both boards, whatever the verdict.

The board is #1619's without its .kicad_dru: a GND pad in a pad row of a
0.5 mm class, whose fanout stub (under the class clearance, so a graze-prune
candidate) is its only connection. To make a pass remove the stub, this test puts
back the prune's old fill model, the one carved at the zone's clearance
alone (`trusted_zone_model` ignoring the class map). batch_route then runs
in-process on files, as route.py does.

Checks (need KiCad's python for the exact fill; exit 77 without it):
  1. The old model removes the stub (the fixture reaches the defect), the
     re-grade names GND broken by the run, and the gate now lists GND in
     `lost` too -- the two verdicts agree.
  2. With the fixed prune the stub stays: the gate reads `lost []`, and
     the re-grade names nothing broken by the run.

    python3 tests/test_1619_1620_gate_reads_regrade.py
"""
import contextlib
import io
import json
import os
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from run_utils import evidence                                  # noqa: E402
import test_1619_graze_prune_class_fill as fixture              # noqa: E402

failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def last_json(log, tag):
    rows = [l[len(tag):].strip() for l in log.splitlines() if l.startswith(tag)]
    return json.loads(rows[-1]) if rows else None


def run(tmp, tag, old_model):
    import plane_fill_model
    from route import batch_route
    inp = os.path.join(tmp, f'{tag}.kicad_pcb')
    out = os.path.join(tmp, f'{tag}_out.kicad_pcb')
    fixture.build(inp, dru=False)
    real = plane_fill_model.trusted_zone_model
    if old_model:
        plane_fill_model.trusted_zone_model = (
            lambda pcb, z, cache, net_clearances=None: real(pcb, z, cache))
    buf = io.StringIO()
    try:
        with contextlib.redirect_stdout(buf):
            batch_route(inp, out, ['/X', 'GND'])
    finally:
        plane_fill_model.trusted_zone_model = real
    evidence(out, 'routed output')
    log = buf.getvalue()
    return log, last_json(log, 'JSON_IMPROVEMENT_GATE:'), last_json(log, 'JSON_REGRADE:')


def main():
    from kicad_exact_fill import find_kicad_python
    if find_kicad_python() is None:
        print("SKIP: no KiCad python -- the exact fill this test grades on")
        return 77
    tmp = tempfile.mkdtemp(prefix='t1619g_')
    try:
        print("1. the old prune model removes GND's only stub")
        log, gate, rg = run(tmp, 'old', old_model=True)
        if 'Graze prune: removed' not in log:
            print("  [BROKEN TEST] the old model removed nothing: this board does "
                  "not reach the defect")
            return 1
        if gate is None or rg is None:
            print("  [BROKEN TEST] no JSON_IMPROVEMENT_GATE / JSON_REGRADE line")
            return 1
        check("the re-grade names GND broken by the run",
              'GND' in (rg.get('broken_by_run') or []), str(rg.get('broken_by_run')))
        check("and the gate lists GND as lost", 'GND' in gate.get('lost', []),
              f"lost {gate.get('lost')}, verdict {gate.get('verdict')}")

        print("2. the fixed prune keeps it")
        log, gate, rg = run(tmp, 'new', old_model=False)
        check("nothing pruned", 'Graze prune: removed' not in log)
        check("the gate reads lost []", gate is not None and gate.get('lost') == [],
              str(gate and gate.get('lost')))
        check("the re-grade names nothing broken by the run",
              rg is not None and not rg.get('broken_by_run'))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
