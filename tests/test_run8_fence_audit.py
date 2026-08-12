#!/usr/bin/env python3
"""fence_audit: a ground-truth leak is content, not a filename (run-7 D4).

The hole this closes: the perturbation's INPUT board (the human placement with
its copper stripped) sat in every run-7 work dir. Nothing named it as truth, so
nothing fenced it -- and its footprint poses ARE the human placement.

The four behaviours pinned here are the ones that make the audit usable rather
than merely loud:

  1. a renamed copy of the control is caught (the point);
  2. a work dir with only the perturbed board is clean (no false alarm);
  3. a board PRODUCED BY THE RUN that reaches truth is NOT a leak -- one run-7
     board recovered bit-exactly (d1 = 0.000000) and every downstream board it
     wrote matches the control. Calling that a breach would make the audit fire
     loudest on the experiment's best result;
  4. a board PRESENT AT CREATION that matches truth still is a leak, even
     after the same run.

3 and 4 are the same content and opposite verdicts, so the creation manifest --
not a threshold -- is what separates them.

Run: python3 -X utf8 tests/test_run8_fence_audit.py
"""
import os
import shutil
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # #522/py_placer layout

AUDIT = os.path.join(ROOT, 'tests', 'stress', 'fence_audit.py')
BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')

CLEAN, LEAK = 0, 4


def _sha(path):
    import hashlib
    return hashlib.sha256(open(path, 'rb').read()).hexdigest()


def _reserialise(src, dst):
    """Same poses, different bytes -- the shape a real recovery has.

    A `shutil.copy` fixture cannot distinguish a reconstruction from a copy,
    which is precisely the hole the audit used to have. Writing the board back
    out through the writer reproduces every footprint pose and changes the
    file, so the fixture tests the property the audit now tests.

    THE WRITER IS NOW IDEMPOTENT ON ITS OWN OUTPUT. `perturb` hands it every
    footprint rather than only the moved ones -- the `(at)` formatting used to
    fingerprint the perturbed block -- so a control it produced is already in
    writer form and re-serialising it changes nothing. The byte difference a
    real recovery has (fresh uuids, different copper) is not reproduced by a
    pose rewrite any more, so the fixture states it explicitly: a trailing
    newline is legal s-expression whitespace, invisible to the parser, and
    keeps this test about the AUDIT's sha-vs-pose logic rather than about how
    the writer happens to format.
    """
    from kicad_parser import parse_kicad_pcb
    from placement.writer import write_placed_output
    pcb = parse_kicad_pcb(src)
    write_placed_output(src, dst, [
        {'reference': ref, 'new_x': fp.x, 'new_y': fp.y,
         'new_rotation': fp.rotation}
        for ref, fp in sorted(pcb.footprints.items())])
    with open(dst, 'a', encoding='utf-8') as fh:
        fh.write('\n')


def run_audit(control, workdir, mode):
    proc = subprocess.run(
        [sys.executable, '-X', 'utf8', AUDIT, '--control', control,
         '--workdir', workdir, '--mode', mode],
        capture_output=True, text=True, encoding='utf-8', errors='replace')
    return proc.returncode, (proc.stdout or '') + (proc.stderr or '')


def make_case(tmp, truth_dir=None):
    """control + a perturbed board whose poses differ from it.

    `control_out` is passed on purpose. Without it `perturb` writes the
    perturbation RECORD beside the output board (perturb.py:596-602) -- inside
    the work dir -- and that record embeds `original_poses` verbatim. This
    fixture used to take that default, so case 1 below ("a work dir holding
    only the perturbed board is clean") was staging a full ground-truth carrier
    and passing anyway, because `DEFAULT_ALLOW` exempted `*.perturb.json` by
    name. Staging it the way the RUNBOOK says to stage it makes case 1 test
    what its name claims, and case 1b pins the behaviour that name-exemption
    used to hide.
    """
    from placement.perturb import perturb

    # The default is a SIBLING of the work dir, never `<tmp>/_truth`: a truth
    # dir nested inside the work dir is still inside the fence, and
    # `fence_audit` walks into it and reports both files. Every caller passes
    # an explicit path today, so this default is only ever a trap for the next
    # one.
    truth = truth_dir or os.path.join(os.path.dirname(os.path.abspath(tmp)),
                                      '_truth')
    os.makedirs(truth, exist_ok=True)
    control = os.path.join(truth, 'perturbed.control.kicad_pcb')
    perturbed = os.path.join(tmp, 'perturbed.kicad_pcb')
    perturb(BOARD, perturbed, kind='translate', seed=91, dose_mm=8.0,
            control_out=control)
    return control, perturbed


def main():
    failures = []

    def check(name, cond, detail=''):
        print(f'  {"PASS" if cond else "FAIL"}  {name}'
              + (f'\n        {detail}' if not cond and detail else ''))
        if not cond:
            failures.append(name)

    with tempfile.TemporaryDirectory() as tmp:
        wd = os.path.join(tmp, 'wk')
        os.makedirs(wd)
        control, perturbed = make_case(wd, truth_dir=os.path.join(tmp, '_truth'))

        # 1. clean: the work dir holds the perturbed board and nothing else.
        #    The control lives OUTSIDE, in _truth/, which is where the staging
        #    recipe puts it.
        code, out = run_audit(control, wd, 'audit')
        check('perturbed-only work dir is CLEAN', code == CLEAN, out)

        # 1b. the record is truth in JSON clothing, and it is what the DEFAULT
        #     perturb path drops into the work dir. `DEFAULT_ALLOW` used to
        #     exempt it by name, so this exact file could sit in the fence and
        #     the audit would report CLEAN.
        rec = os.path.join(wd, 'perturbed.perturb.json')
        with open(rec, 'w', encoding='utf-8') as fh:
            import json as _json
            from kicad_parser import parse_kicad_pcb as _pk
            _json.dump({'original_poses': {
                r: [f.x, f.y, f.rotation]
                for r, f in _pk(control).footprints.items()}}, fh)
        code, out = run_audit(control, wd, 'audit')
        check('a pose RECORD inside the fence is a LEAK', code == LEAK, out)
        check('the record leak names the file',
              'perturbed.perturb.json' in out, out)
        check('and says records are never reconstructed',
              'never reconstructed' in out, out)

        # 1c. same truth, saved as UTF-16. The content test must not depend on
        #     how the file was encoded -- reading it as utf-8 text raised
        #     UnicodeDecodeError into a bare except and waved it through.
        with open(rec, encoding='utf-8') as fh:
            _raw = fh.read()
        os.remove(rec)
        rec16 = os.path.join(wd, 'notes.json')
        with open(rec16, 'w', encoding='utf-16') as fh:
            fh.write(_raw)
        code, out = run_audit(control, wd, 'audit')
        check('a UTF-16 pose record is still a LEAK', code == LEAK, out)
        os.remove(rec16)
        code, out = run_audit(control, wd, 'audit')
        check('and the dir is CLEAN again once it is gone', code == CLEAN, out)

        # 2. the leak, under a name nobody would fence
        leaked = os.path.join(wd, 'stripped.kicad_pcb')
        shutil.copy(control, leaked)
        code, out = run_audit(control, wd, 'audit')
        check('a renamed control copy is a LEAK', code == LEAK, out)
        check('the leak names the file', 'stripped.kicad_pcb' in out, out)

        os.remove(leaked)

        # 3. create -> manifest -> a board written LATER that reaches truth
        code, out = run_audit(control, wd, 'create')
        check('create mode on a clean dir exits 0', code == CLEAN, out)
        check('create writes the manifest',
              os.path.isfile(os.path.join(wd, '.fence-manifest.json')), out)

        # A byte-identical COPY of the control is not a reconstruction, and
        # used to be reported as one: `shutil.copy` + a name not in the
        # manifest was classed 'produced by the run' and exited 0. That is the
        # laundering route -- copy the control, and the audit congratulates
        # you. A real recovery differs in copper, UUIDs and netclasses even
        # when every pose matches, which is exactly what `control_sha256` in
        # the manifest is for.
        laundered = os.path.join(wd, 'r4_copied.kicad_pcb')
        shutil.copy(control, laundered)
        code, out = run_audit(control, wd, 'audit')
        check('a byte-identical copy of the control is a LEAK, not a recovery',
              code == LEAK, out)
        check('...and says it was copied rather than rebuilt',
              'copied, not reconstructed' in out, out)
        os.remove(laundered)

        # A GENUINE reconstruction: same poses, different bytes. Re-serialising
        # through the writer reproduces every pose and changes the file, which
        # is the shape a real recovery has.
        recovered = os.path.join(wd, 'r4_final.kicad_pcb')
        _reserialise(control, recovered)
        assert _sha(recovered) != _sha(control), 'fixture must differ in bytes'
        code, out = run_audit(control, wd, 'audit')
        check('a board produced BY the run that reaches truth is not a leak',
              code == CLEAN, out)
        check('...and is reported as recovery, not silence',
              'r4_final.kicad_pcb' in out and 'produced by the run' in out, out)

        # 4. same content, present at creation -> still a leak
        os.remove(recovered)
        os.remove(os.path.join(wd, '.fence-manifest.json'))
        preexisting = os.path.join(wd, 'board_orig.kicad_pcb')
        shutil.copy(control, preexisting)
        code, out = run_audit(control, wd, 'create')
        check('create mode CATCHES a pre-existing carrier', code == LEAK, out)

    print()
    if failures:
        print(f'FAIL: {len(failures)} check(s): {", ".join(failures)}')
        return 1
    print('OK')
    return 0


if __name__ == '__main__':
    sys.exit(main())
