#!/usr/bin/env python3
"""The SHIPPED stagers arm the unaided regime, run as the RUNBOOK runs them.

#903. `placement.provenance` was a complete instrument with no production
caller: `start_regime` was invoked by two tests and by nothing else, so
`.unaided-manifest.json` never existed, `regime_for` answered None,
`record_write` returned None, `.pose-provenance.jsonl` was never written, and
`provenance_audit.py` printed

    VERDICT: UNPROVEN
      no .unaided-manifest.json: this work dir was not staged for an unaided run

on every real run -- by construction, not by accident. The gate that refuses an
undeclared pose writer was installed and never armed, so run 25's hand rotation
(`pose_assist.py rotate`, a script that imports `placement.writer` directly)
reached the board and was reported by a watcher five hours later instead of
being refused at 17:52.

WHY A SUBPROCESS TEST, beside the in-process block in
`tests/test_provenance_audit.py`. The defect was a MISSING CALL in an entry
point. An in-process check calls `stage()` and can never see what `__main__`
does or does not do around it -- and `__main__` is exactly where the CLI's
`declare_lever` lives, and where the RUNBOOK's reader starts. This file runs
the two stagers the way `wk/run25/prompt_esp_prog.md:68` runs them, as
processes, and reads what they left on disk.

Every assertion goes through `run_utils.check(...)`, so a command that dies of
an ImportError is reported as a BROKEN TEST rather than as a guard that held.

Exit 0 all-pass, 1 any failure. NEVER exit 77: a gate that self-skips reports
every row it guards as satisfied, and this one is a mutation-battery killer.
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile

_TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(_TESTS)
for _p in (_TESTS, ROOT, os.path.join(ROOT, 'py_router'),
           os.path.join(ROOT, 'py_placer'), os.path.join(ROOT, 'py_placer', 'placement')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import run_utils  # noqa: E402
from run_utils import check, evidence  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
STAGE_UNAIDED = os.path.join('tests', 'stress', 'stage_unaided.py')
STAGE_BLIND = os.path.join('tests', 'stress', 'stage_blind.py')
AUDIT = os.path.join('tests', 'stress', 'provenance_audit.py')
FENCE = os.path.join('tests', 'stress', 'fence_audit.py')
BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')

REGIME = '.unaided-manifest.json'
LEDGER = '.pose-provenance.jsonl'

passed = failed = 0


def ck(name, ok, detail=''):
    global passed, failed
    if ok:
        passed += 1
        print(f'  OK   {name}' + (f' -- {detail}' if detail else ''))
    else:
        failed += 1
        print(f'  FAIL {name} -- {detail}')


def rows(path):
    out = []
    with open(path, encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if line:
                out.append(json.loads(line))
    return out


# The fixture is REQUIRED, not optional. A missing board here must be a
# failure: "the fixture is gone" and "the stagers do not arm" would otherwise
# produce the same exit code.
evidence(BOARD, 'the splitflap_driver fixture')

_tmp = tempfile.mkdtemp(prefix='t903_')
try:
    wd = os.path.join(_tmp, 'wk')
    td = os.path.join(_tmp, 'truth')

    # ----------------------------------------------------------------- 1
    # The shipped CLI arms. This is the whole issue.
    check(PY + [STAGE_UNAIDED, BOARD, wd, td], accept=True)
    manifest = os.path.join(wd, REGIME)
    evidence(manifest, 'the unaided regime manifest')
    ck('stage_unaided.py writes the regime manifest', True, manifest)

    m = json.load(open(manifest, encoding='utf-8'))
    board = os.path.join(wd, 'board.kicad_pcb')
    ck('the manifest names the STAGED board, not the source',
       os.path.abspath(m['staged_board']) == os.path.abspath(board),
       str(m.get('staged_board')))
    # Armed AFTER the board and its siblings are final. If it were armed
    # first, `staged_sha256` would name bytes that no longer exist and NOTHING
    # would report it: provenance_audit only checks the staged board is
    # readable, never that it is the one the manifest describes.
    import hashlib

    def _sha(p):
        h = hashlib.sha256()
        with open(p, 'rb') as fh:
            for chunk in iter(lambda: fh.read(1 << 20), b''):
                h.update(chunk)
        return h.hexdigest()
    ck('and its hash is the hash of the board ON DISK',
       m['staged_sha256'] == _sha(board), m['staged_sha256'][:16])
    ck('the source board is named NOWHERE in the manifest',
       'splitflap' not in json.dumps(m).lower(), json.dumps(m)[:120])

    # ----------------------------------------------------------------- 2
    # The verdict this issue is about is GONE, and what replaces it is a
    # different, honest UNPROVEN: nothing has been delivered yet. Asserting
    # WHICH 5 is the finding -- a bare "still exits 5" would pass unchanged
    # against the unfixed tree.
    r = check(PY + [AUDIT, '--workdir', wd], code=5,
              refuse='no delivered board in the work dir')
    ck('the "not staged for an unaided run" verdict is GONE',
       'not staged for an unaided run' not in (r.stdout + r.stderr),
       (r.stdout or '').strip().splitlines()[-2:][0][:100])

    # ----------------------------------------------------------------- 3
    # A RESTAGE is permitted, and records itself. This is what makes the
    # LEVER_REGISTRY entry behavioural rather than decorative: without it the
    # stager is refused by the guard it installed one run earlier.
    check(PY + [STAGE_UNAIDED, BOARD, wd, td], accept=True)
    ledger = os.path.join(wd, LEDGER)
    evidence(ledger, 'the pose-provenance ledger a restage writes')
    lr = rows(ledger)
    ck('a restage is permitted and writes exactly one row',
       len(lr) == 1, str(len(lr)))
    ck('...under its own lever', lr[0]['lever'] == 'stage_unaided.py',
       lr[0]['lever'])
    # ...AND THE ROW SAYS NOTHING ELSE. The ledger lives in the work dir,
    # which is inside the fence. Unredacted, this row held `lever_argv`
    # naming the SOURCE BOARD and the truth dir, `refs_moved` naming the
    # perturbed block, and 65 `poses_written` of which 56 were the control
    # pose -- the answer key, in a file the run can read, which `fence_audit`
    # cannot see because `.jsonl` is not a scanned extension.
    ck('a staging row carries no argv, no parent hash and no poses',
       not {'lever_argv', 'parent_sha256', 'poses_written', 'refs_written',
            'refs_moved', 'sides_written'} & set(lr[0]),
       str(sorted(lr[0])))
    ck('...and says so, rather than looking like a row that never had them',
       'fence' in (lr[0].get('redacted') or ''), str(lr[0].get('redacted')))
    blob = open(ledger, encoding='utf-8').read()
    ck('the ledger names neither the source board nor the truth dir',
       'splitflap' not in blob and os.path.basename(td) not in blob,
       blob[:160])
    m2 = json.load(open(manifest, encoding='utf-8'))
    ck('prior_stagings counts rows that PREDATE the restage, not its own',
       m2['prior_stagings'] == 0, str(m2['prior_stagings']))
    check(PY + [STAGE_UNAIDED, BOARD, wd, td], accept=True)
    m3 = json.load(open(manifest, encoding='utf-8'))
    ck('...and rises by one on the next restage',
       m3['prior_stagings'] == 1, str(m3['prior_stagings']))
    # ...and it is a STAGING count, not a row count. An engine write in the
    # same dir must not inflate it, or "restaged over 47 rows" is what a dir
    # restaged once reports.
    lever = os.path.join(_tmp, 'lever_pose.py')
    with open(lever, 'w', encoding='utf-8') as fh:
        fh.write(
            "import os, sys\n"
            "ROOT = sys.argv[3]\n"
            "for p in (os.path.join(ROOT, 'py_router'),\n"
            "          os.path.join(ROOT, 'py_placer')):\n"
            "    sys.path.insert(0, p)\n"
            "from kicad_parser import parse_kicad_pcb\n"
            "from placement.provenance import declare_lever\n"
            "from placement.writer import write_placed_output\n"
            "src, dst = sys.argv[1], sys.argv[2]\n"
            "pcb = parse_kicad_pcb(src)\n"
            "refs = sorted(pcb.footprints)[:3]\n"
            "mv = [{'reference': r, 'new_x': 12.0 + i, 'new_y': 34.0,\n"
            "       'new_rotation': 0.0} for i, r in enumerate(refs)]\n"
            "with declare_lever('place_optimize.py', sys.argv):\n"
            "    print(write_placed_output(src, dst, mv))\n")
    delivered = os.path.join(wd, 'placed.kicad_pcb')
    check(PY + [lever, board, delivered, ROOT], accept=True)
    check(PY + [STAGE_UNAIDED, BOARD, wd, td], accept=True)
    m4 = json.load(open(manifest, encoding='utf-8'))
    ck('an ENGINE write does not inflate prior_stagings',
       m4['prior_stagings'] == 2, str(m4['prior_stagings']))
    ck('...while prior_ledger_rows does count it, which is the laundering '
       'number', m4['prior_ledger_rows'] > m4['prior_stagings'],
       f"rows {m4['prior_ledger_rows']} stagings {m4['prior_stagings']}")

    # ----------------------------------------------------------------- 4
    # THE POINT, end to end, with run 25's OWN hand script shape: a script
    # that imports placement.writer and writes poses, declaring nothing.
    hand = os.path.join(_tmp, 'hand_pose.py')
    with open(hand, 'w', encoding='utf-8') as fh:
        fh.write(
            "import os, sys\n"
            "ROOT = sys.argv[3]\n"
            "for p in (os.path.join(ROOT, 'py_router'),\n"
            "          os.path.join(ROOT, 'py_placer')):\n"
            "    sys.path.insert(0, p)\n"
            "from kicad_parser import parse_kicad_pcb\n"
            "from placement.writer import write_placed_output\n"
            "src, dst = sys.argv[1], sys.argv[2]\n"
            "pcb = parse_kicad_pcb(src)\n"
            "ref = sorted(pcb.footprints)[0]\n"
            "fp = pcb.footprints[ref]\n"
            "print(write_placed_output(src, dst, [\n"
            "    {'reference': ref, 'new_x': fp.x, 'new_y': fp.y,\n"
            "     'new_rotation': 90.0}]))\n")
    victim = os.path.join(wd, 'hand.kicad_pcb')
    # `allow`: an uncaught UnaidedViolation prints a traceback, and
    # run_utils would otherwise class this satisfied guard as a broken test.
    check(PY + [hand, board, victim, ROOT], refuse='UnaidedViolation',
          allow=('Traceback (most recent call last)',))
    ck('an undeclared hand pose write is REFUSED in a staged work dir', True)
    ck('...and nothing landed -- refusing means not writing',
       not os.path.exists(victim), victim)

    # ----------------------------------------------------------------- 5
    # A registered lever in the same dir audits CLEAN. This is M1's exit
    # criterion: PROVENANCE reads something other than UNPROVEN. The lever
    # and its delivered board were written in step 3, above.
    r = check(PY + [AUDIT, '--workdir', wd, '--delivered', delivered],
              accept=True)
    ck('a registered lever in a staged work dir audits CLEAN',
       'VERDICT: CLEAN' in r.stdout, (r.stdout or '').strip()[:120])
    js = json.loads(r.stdout.split('JSON_SUMMARY: ', 1)[1].splitlines()[0])
    # Not vacuous: a CLEAN over zero moved poses would pass against a tree
    # where nothing was ever recorded.
    ck('...over poses that really moved, so CLEAN is not vacuous',
       js['moved'] > 0 and js['claimed'] >= js['moved'],
       f"moved {js['moved']} claimed {js['claimed']}")

    # --------------------------------------------------------------- 5b
    # A NESTED stage must not launder a violation. The inner board is written
    # before the inner manifest exists, so `regime_for` binds that write to
    # the OUTER regime and appends a row whose `path` points into the inner
    # dir. Being the newest row it used to become the outer dir's "delivered
    # board" -- so the real one was never audited, and a hand-edited board
    # went from UNAIDED VIOLATION (exit 4) to CLEAN (exit 0) purely by
    # staging a sub-experiment underneath it.
    hand2 = os.path.join(_tmp, 'edit_pose.py')
    with open(hand2, 'w', encoding='utf-8') as fh:
        fh.write(
            "import os, sys\n"
            "ROOT = sys.argv[3]\n"
            "for p in (os.path.join(ROOT, 'py_router'),\n"
            "          os.path.join(ROOT, 'py_placer')):\n"
            "    sys.path.insert(0, p)\n"
            "from kicad_parser import parse_kicad_pcb\n"
            "from placement.provenance import declare_lever\n"
            "from placement.writer import write_placed_output\n"
            "src, dst = sys.argv[1], sys.argv[2]\n"
            "pcb = parse_kicad_pcb(src)\n"
            "refs = sorted(pcb.footprints)[:3]\n"
            "mv = [{'reference': r, 'new_x': 90.0 + i, 'new_y': 70.0,\n"
            "       'new_rotation': 0.0} for i, r in enumerate(refs)]\n"
            "with declare_lever('place_optimize.py', sys.argv):\n"
            "    print(write_placed_output(src, dst, mv))\n")
    # Drift the delivered board AWAY from where its lever claimed to put it.
    # Written OUTSIDE the work dir -- `regime_for` walks up and finds no
    # manifest there -- then copied in. The copy leaves no ledger row, which
    # is what a hand edit of `(at ...)` looks like to the audit.
    drifted = os.path.join(_tmp, 'drifted.kicad_pcb')
    check(PY + [hand2, delivered, drifted, ROOT], accept=True)
    shutil.copyfile(drifted, delivered)
    r = check(PY + [AUDIT, '--workdir', wd, '--delivered', delivered], code=4,
              refuse='are NOT where the lever')
    ck('a drifted delivered board is an UNAIDED VIOLATION', True)
    inner = os.path.join(wd, 'inner')
    check(PY + [STAGE_UNAIDED, BOARD, inner, os.path.join(_tmp, 'truth2')],
          accept=True)
    r = check(PY + [AUDIT, '--workdir', wd], code=4,
              refuse='are NOT where the lever')
    ck('...and a nested stage underneath it does NOT launder it to CLEAN',
       'inner' not in (r.stdout or ''), (r.stdout or '')[:160])

    # --------------------------------------------------------------- 5c
    # A manifest that describes a board no longer on disk is UNPROVEN, not a
    # baseline. `staged_sha256` was written and read by nobody, and the whole
    # audit is a comparison against this file -- reachable without bad faith,
    # since `stage()` writes the board first and arms last and the steps
    # between can raise.
    stale = os.path.join(_tmp, 'stale')
    check(PY + [STAGE_UNAIDED, BOARD, stale, os.path.join(_tmp, 't_stale')],
          accept=True)
    sm = os.path.join(stale, REGIME)
    doc = json.load(open(sm, encoding='utf-8'))
    doc['staged_sha256'] = '0' * 64
    with open(sm, 'w', encoding='utf-8') as fh:
        json.dump(doc, fh)
    check(PY + [AUDIT, '--workdir', stale], code=5,
          refuse='describes a DIFFERENT board')
    ck('a manifest whose hash no longer matches the board is UNPROVEN', True)

    # ----------------------------------------------------------------- 6
    # The fence is unharmed, in BOTH modes. The manifest is a `.json` inside
    # the work dir, so it is scanned; it must never become a row.
    control = os.path.join(td, 'control.kicad_pcb')
    evidence(control, 'the staged control board')
    check(PY + [FENCE, '--control', control, '--workdir', wd,
                '--mode', 'create'], accept=True)
    ck('fence_audit --mode create is CLEAN on an armed work dir', True)
    fm = os.path.join(wd, '.fence-manifest.json')
    evidence(fm, 'the fence creation manifest')
    fj = json.load(open(fm, encoding='utf-8'))
    # The manifest is a `.json` and IS opened; it becomes no row because it
    # has no `original_poses`. The ledger is `.jsonl` and is not opened at
    # all -- deliberately, see the SCANNED_EXT note in fence_audit.py. Either
    # way neither may be a fence row.
    ck('neither provenance file is a fence row',
       not any(REGIME in f or LEDGER in f for f in fj.get('files') or ()),
       str(fj.get('files'))[:140])
    check(PY + [FENCE, '--control', control, '--workdir', wd,
                '--mode', 'audit'], accept=True)
    ck('fence_audit --mode audit is CLEAN too', True)

    # ----------------------------------------------------------------- 7
    # stage_blind arms the same way, and its manifest carries NOTHING about
    # the draw. `sanitized` names the withheld strings and kind/dose/seed ARE
    # the fence, so an author who helpfully passed **rec would leak the answer
    # key into the work dir. This asserts the exact key set, so that edit
    # cannot pass.
    wdb = os.path.join(_tmp, 'wkb')
    tdb = os.path.join(_tmp, 'truthb')
    rb = subprocess.run(PY + [STAGE_BLIND, BOARD, wdb, tdb],
                        capture_output=True, text=True, encoding='utf-8',
                        errors='replace', cwd=ROOT, timeout=1800)
    # A board that cannot take a material dose is a fixture problem, and it
    # must FAIL rather than skip: a battery reads a skip as every row killed.
    ck('stage_blind.py staged the fixture', rb.returncode == 0,
       (rb.stdout + rb.stderr)[-200:])
    if rb.returncode == 0:
        mb_path = evidence(os.path.join(wdb, REGIME),
                           "stage_blind's regime manifest")
        mb = json.load(open(mb_path, encoding='utf-8'))
        ck('stage_blind arms the regime too', True, mb_path)
        ck('its manifest carries EXACTLY the schema keys, no draw extras',
           set(mb) == {'schema', 'kind', 'staged_board', 'staged_sha256',
                       'lever_registry'}, str(sorted(mb)))
        import placement.perturb as _P
        blob = json.dumps(mb)
        ck('no perturbation kind appears in it',
           not [k for k in _P.KINDS if k in blob],
           str([k for k in _P.KINDS if k in blob]))
        draw = json.load(open(os.path.join(tdb, 'draw.json'), encoding='utf-8'))
        ck('...while the truth dir does carry the draw, so the check is real',
           bool(draw.get('kind')), str(draw.get('kind')))

    # ----------------------------------------------------------------- 8
    # The restage is visible to the watcher. Neither stager installs
    # cli_banner, so the CMD:-line counter can never see the FIRST staging
    # (it creates the work dir; there is nowhere to tee to yet). The ledger
    # row is the source that works.
    sys.path.insert(0, os.path.join(ROOT, 'tests', 'stress'))
    import run_watch as RW  # noqa: E402
    # RE-DERIVED, not a constant: a hardcoded total silently stops meaning
    # anything the next time a step is added above it, and would have to be
    # edited rather than consulted.
    want_n = len([r for r in rows(ledger)
                  if r.get('lever') in ('stage_unaided.py', 'stage_blind.py')])
    ck('run_watch counts every recorded re-stage in the ledger, and only those',
       RW._ledger_stagings(ledger) == want_n and want_n >= 2,
       f'{RW._ledger_stagings(ledger)} vs {want_n} staging rows')
    ck('...which redaction did NOT break -- it reads `lever`, which survives',
       all('lever' in r for r in rows(ledger)), str(sorted(rows(ledger)[0])))
    # BOTH stagers, asserted against a synthetic ledger rather than against a
    # blind dir that happens to hold no restage -- "0 restages here" is true
    # of a matcher that names neither.
    synth = os.path.join(_tmp, 'synth.jsonl')
    with open(synth, 'w', encoding='utf-8') as fh:
        for lev in ('stage_blind.py', 'stage_unaided.py', 'place_seed.py'):
            fh.write(json.dumps({'lever': lev}) + os.linesep)
    ck('...and the matcher names BOTH stagers and nothing else',
       RW._ledger_stagings(synth) == 2, str(RW._ledger_stagings(synth)))
    # ...and DISCRIMINATES: a ledger of ordinary lever rows counts zero, so
    # the two counts above are not just "every row".
    nostage = os.path.join(_tmp, 'nostage.jsonl')
    with open(nostage, 'w', encoding='utf-8') as fh:
        for lev in ('place_seed.py', 'converge.py', 'route.py'):
            fh.write(json.dumps({'lever': lev}) + os.linesep)
    ck('...and a ledger of non-staging levers counts zero',
       RW._ledger_stagings(nostage) == 0, str(RW._ledger_stagings(nostage)))
finally:
    shutil.rmtree(_tmp, ignore_errors=True)

print(f'\n{passed} passed, {failed} failed')
print('903 coverage: unaided=yes blind=yes refusal=yes restage=yes fence=yes')
sys.exit(1 if failed else 0)
