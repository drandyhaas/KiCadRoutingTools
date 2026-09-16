#!/usr/bin/env python3
"""#903 mutation battery: is the ARMING actually covered, or do its tests
merely run?

    python3 tests/mutate_903.py
    python3 tests/mutate_903.py --row the-unaided-stager-never-arms
    python3 tests/mutate_903.py --list

NOT named `test_*`, so `run_all.py` never collects it: it REWRITES engine and
harness files in place and restores them, and a suite running beside it would
grade a mutated tree. One writer per tree.

A row is KILLED when any named test exits non-zero -- a failed assertion and
an ERROR count the same, because a mutation that makes the graders crash is
still a mutation the graders noticed. A row whose anchor does not match
EXACTLY ONCE is BROKEN, not skipped: an anchor that silently matches nothing
reports every mutation as killed and is the most flattering possible bug.

WHY THIS BATTERY MATTERS PARTICULARLY HERE. #903's whole content is a CALL
THAT WAS NOT MADE. The instrument it arms was complete, correct and tested
before this change -- `tests/test_provenance_audit.py` was 47 green rows
against an arming that no production path ever performed. So "the tests pass"
was true of the defect, and every row below is a way of putting the defect
back that a reader could mistake for a tidy-up: dropping a call, arming the
wrong dir, hashing the wrong file, reverting one tuple entry.

Row 1 removes the arming call, which is the DEFECT restated; it is not a
full revert of the PR (the pre-#903 tree also lacked the registry entry,
the declaration, stage_blind's arming and the run_watch changes, each of
which has a row of its own).

BYTECODE. Each row rewrites a file and restores it within the same second,
and the registry row is nearly size-preserving -- exactly the (mtime, size)
pair CPython's `.pyc` check treats as unchanged. The runner drops the target's
`__pycache__` and runs every test with `-B`; without that a later row imports
an earlier row's mutant and the results are fiction. `tests/stress/` needs
this as much as `py_placer/` does, because both stagers are imported as
modules by other tests.
"""
import argparse
import os
import shutil
import subprocess
import sys

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)

SU = os.path.join(ROOT, 'tests', 'stress', 'stage_unaided.py')
SB = os.path.join(ROOT, 'tests', 'stress', 'stage_blind.py')
PV = os.path.join(ROOT, 'py_placer', 'placement', 'provenance.py')
RW = os.path.join(ROOT, 'tests', 'stress', 'run_watch.py')
PA = os.path.join(ROOT, 'tests', 'stress', 'provenance_audit.py')
PS = os.path.join(ROOT, 'py_placer', 'place_seed.py')
RL = os.path.join(ROOT, 'py_placer', 'place_route_loop.py')
TARGETS = {'su': SU, 'sb': SB, 'pv': PV, 'rw': RW, 'pa': PA, 'ps': PS,
           'rl': RL}

T_903 = os.path.join(TESTS, 'test_903_stagers_arm_the_regime.py')
T_PROV = os.path.join(TESTS, 'test_provenance_audit.py')
#: #972's lineage gate. In-process and a few seconds, so rows that it kills
#: name it ALONE rather than paying for T_PROV's real CLI runs.
T_972 = os.path.join(TESTS, 'test_972_pose_lineage.py')
#: #973's delivery gate: the real place_seed CLI and place_route_loop's main().
T_973 = os.path.join(TESTS, 'test_973_delivery_rows.py')

#: The cheap gate, run unmutated first. `T_PROV` is the in-process half and
#: `T_903` the subprocess half; a row is only evidence if both are green
#: before anything is rewritten.
BASELINE = (T_903, T_PROV)

#: (name, target, old, new, tests, expectation)
ROWS = [
    # ---- the defect itself, put back --------------------------------------
    # This is `main`'s tree as of the commit before this PR: `stage()` writes
    # the board and never arms. Every downstream symptom follows -- no
    # manifest, no ledger, no refusal, UNPROVEN forever.
    ('the-unaided-stager-never-arms', 'su',
     "    _PV.start_regime(_wd, out_board, mechanical=os.path.abspath(mech),\n"
     "                     prior_ledger_rows=_prior,\n"
     "                     prior_stagings=_prior_stagings)\n",
     "",
     (T_903, T_PROV), 'KILLED'),

    ('the-blind-stager-never-arms', 'sb',
     "    _PV.start_regime(os.path.dirname(os.path.abspath(out)), out)\n",
     "",
     (T_903,), 'KILLED'),

    # ---- armed, but describing the wrong thing ----------------------------
    # The manifest hashes the SOURCE instead of the staged board. Not caught
    # by provenance_audit, which only checks the staged board is READABLE and
    # the source is a perfectly readable file -- so the only thing between
    # this and a manifest that lies (while naming the source path inside the
    # fence) is T_903's two assertions. That is this row's whole job.
    ('the-manifest-describes-the-source', 'su',
     "    _PV.start_regime(_wd, out_board, mechanical=os.path.abspath(mech),",
     "    _PV.start_regime(_wd, src, mechanical=os.path.abspath(mech),",
     (T_903,), 'KILLED'),

    # Armed over the wrong directory: `regime_for` walks UP from the board,
    # so arming the PARENT still governs the work dir and every refusal still
    # fires -- but `provenance_audit --workdir` looks only in the dir it was
    # given and finds nothing. The whole instrument reads UNPROVEN again
    # while every in-process test still passes.
    ('the-regime-is-armed-one-level-up', 'su',
     "    _wd = os.path.dirname(os.path.abspath(out_board))\n",
     "    _wd = os.path.dirname(os.path.dirname(os.path.abspath(out_board)))\n",
     (T_903, T_PROV), 'KILLED'),

    # ---- the registry entry, reverted alone -------------------------------
    # Killed by BOTH a behavioural gate and a membership one, and the
    # behavioural half is the one that matters: T_903 restages, `record_write`
    # refuses a lever the tuple does not carry, and the stager is refused by
    # the guard it installed one run earlier. (T_PROV also asserts membership
    # directly, so this row would die even without that.)
    ('the-registry-loses-the-unaided-stager', 'pv',
     "    'perturb.py', 'stage_blind.py', 'stage_unaided.py',\n",
     "    'perturb.py', 'stage_blind.py',\n",
     (T_903, T_PROV), 'KILLED'),

    # ---- the declaration, back in __main__ only ---------------------------
    # The shape this PR found by writing its own test: the CLI still works,
    # so a reviewer sees nothing, and a LIBRARY caller of `stage()` into an
    # armed dir raises "no registered lever" -- the stager refusing itself.
    ('the-stager-declares-only-in-main', 'su',
     "    with (contextlib.nullcontext() if _PV.active_lever() is not None\n"
     "          else _PV.declare_lever('stage_unaided.py')):\n"
     "        write_placed_output(src, out_board, placements)\n",
     "    write_placed_output(src, out_board, placements)\n",
     (T_PROV,), 'KILLED'),

    # The inverse: declare UNCONDITIONALLY, which breaks innermost-wins in the
    # one direction that matters. `declare_lever`'s contract is that "a tool
    # that shells out to another still attributes to the one doing the
    # writing" -- an inner declaration that always fires attributes a staging
    # performed INSIDE another lever's scope to the stager instead of to the
    # caller, so the ledger names the wrong tool. Nothing about the board
    # changes, and (since the row is redacted either way) nothing about the
    # fence does either; only the attribution is wrong.
    ('the-inner-declaration-overrides-its-caller', 'su',
     "    with (contextlib.nullcontext() if _PV.active_lever() is not None\n"
     "          else _PV.declare_lever('stage_unaided.py')):\n",
     "    with _PV.declare_lever('stage_unaided.py'):\n",
     (T_PROV,), 'KILLED'),

    # ---- the disclosure that keeps a restage honest -----------------------
    # Read AFTER the staging write instead of before, so the count includes
    # this call's own row and "restaged over 1 row" becomes true of a dir
    # nothing had restaged. A one-line move that reads as a tidy-up.
    ('prior-stagings-counts-its-own-row', 'su',
     "    _rows = _PV.read_ledger(_wd)\n",
     "    _rows = []\n",
     (T_903, T_PROV), 'KILLED'),

    # ...and it is a STAGING count, not a row count. An unfiltered total reads
    # "restaged over 47 rows" on a dir restaged once -- measured, four
    # place_seed candidate writes made it say 4.
    ('prior-stagings-counts-every-engine-write-too', 'su',
     "    _prior_stagings = sum(1 for _r in _rows\n"
     "                          if _r.get('lever') in _PV.FENCE_SENSITIVE_LEVERS)\n",
     "    _prior_stagings = len(_rows)\n",
     (T_903,), 'KILLED'),

    # ---- the fence leak this PR would otherwise have opened ---------------
    # The ledger lives INSIDE the work dir. Unredacted, a staging row holds
    # `lever_argv` naming the source board and the truth dir, `refs_moved`
    # naming the perturbed block, and poses that are the control's -- and
    # fence_audit cannot see it, because `.jsonl` is not a scanned extension.
    ('the-staging-row-is-not-redacted', 'pv',
     "    if lever['lever'] in FENCE_SENSITIVE_LEVERS:\n",
     "    if False:\n",
     (T_903,), 'KILLED'),

    # Redacted, but keeping the argv -- the single most valuable field to a
    # run trying to identify its own source board.
    ('the-redacted-row-keeps-the-argv', 'pv',
     "        row = {'t': row['t'], 'schema': SCHEMA, 'path': row['path'],\n"
     "               'lever': row['lever'], 'declared': True,\n"
     "               'caller': row['caller'],\n",
     "        row = {'t': row['t'], 'schema': SCHEMA, 'path': row['path'],\n"
     "               'lever': row['lever'], 'declared': True,\n"
     "               'caller': row['caller'],\n"
     "               'lever_argv': lever['lever_argv'],\n",
     (T_903,), 'KILLED'),

    # ---- the nested-stage laundering channel ------------------------------
    # A nested staging row's `path` points into the inner dir and became the
    # OUTER dir's delivered board, so a drifted board went from VIOLATION to
    # CLEAN purely by staging a sub-experiment underneath it.
    ('a-nested-staging-row-can-be-the-delivered-board', 'pa',
     "            if r.get('lever') in PV.FENCE_SENSITIVE_LEVERS:\n"
     "                continue\n",
     "",
     (T_903,), 'KILLED'),

    # ---- the baseline nobody checked --------------------------------------
    # `staged_sha256` was written and read by nobody, and the whole audit is a
    # comparison against that file.
    ('the-stale-manifest-is-not-detected', 'pa',
     "    if _sha and _sha != PV.sha256_file(staged):\n",
     "    if False:\n",
     (T_903,), 'KILLED'),

    # ---- the watcher gap this PR opened, and closed -----------------------
    # Was a declared SURVIVOR with "neither stager prints a CMD: line" as the
    # reason. A review pointed out that this explains why the counter is
    # unreliable in the FIELD, not why it cannot be TESTED -- and it is, in
    # about a second, once the predicate is a function instead of an inline
    # `any(...)`. An exclusion needs evidence like a claim does.
    ('the-restage-counter-forgets-the-unaided-stager', 'rw',
     "    return any(str(t).endswith(('stage_blind.py', 'stage_unaided.py'))\n"
     "               for t in toks)\n",
     "    return any(str(t).endswith('stage_blind.py') for t in toks)\n",
     (T_903,), 'KILLED'),

    # The nesting guard the ledger counter needs. A NESTED dir's FIRST
    # staging lands its row in the OUTER ledger, so without this the outer
    # dir reports a re-stage it never had -- measured, two first-stagings
    # reported "1 re-staging(s)".
    ('the-ledger-counter-counts-a-nested-dirs-first-stage', 'rw',
     "                if os.path.dirname(os.path.abspath(p)) != os.path.abspath(\n"
     "                        os.path.dirname(os.path.abspath(path))):\n"
     "                    continue\n",
     "",
     (T_903,), 'KILLED'),

    # A watcher that dies before the DONE block never runs the fence or the
    # provenance audit, and its silence reads as clean. UnicodeDecodeError is
    # a ValueError, so `except OSError` did not hold it.
    # Anchored WITH its preceding line: three readers in this file share the
    # same `open(..., errors='replace')` text, and `replace(..., 1)` would
    # have taken whichever came first -- the pre-flight said so, in one
    # second, before anything ran. Both halves of the guard are reverted
    # together, because either alone leaves the crash unreachable.
    ('the-ledger-reader-dies-on-a-non-utf8-byte', 'rw',
     "    try:\n"
     "        with open(path, encoding='utf-8', errors='replace') as f:\n"
     "            for line in f:\n"
     "                line = line.strip()\n"
     "                if not line:\n"
     "                    continue\n"
     "                try:\n"
     "                    r = json.loads(line)\n",
     "    try:\n"
     "        with open(path, encoding='utf-8') as f:\n"
     "            for line in f:\n"
     "                line = line.strip()\n"
     "                if not line:\n"
     "                    continue\n"
     "                try:\n"
     "                    r = json.loads(line)\n",
     (T_903,), 'KILLED'),

    # The LEDGER counter is the one that works (neither stager prints a
    # `CMD:` line, so the log counter above can miss the first staging
    # entirely). T_903 asserts on it directly, which is why this row dies and
    # the log-counter row above does not.
    ('the-ledger-restage-counter-names-one-stager', 'rw',
     "                if str(r.get('lever') or '') not in ('stage_unaided.py',\n"
     "                                                     'stage_blind.py'):\n",
     "                if str(r.get('lever') or '') not in ('stage_blind.py',):\n",
     (T_903,), 'KILLED'),

    # WAS a declared survivor, on the reasoning that building the nested case
    # means arming a temp dir's PARENT and so contaminating every other test
    # under the same temp root. That is true of the IN-PROCESS file and false
    # of `T_903`, which runs subprocesses inside its own `mkdtemp` -- a review
    # built the fixture in two calls. An exclusion needs evidence like a claim
    # does, and this one had a reason that did not survive checking.
    ('the-nested-regime-note-is-silenced', 'su',
     "    if _outer is not None and os.path.abspath(_outer) != _wd:\n",
     "    if False:\n",
     (T_903,), 'KILLED'),

    # ---- #972: the pose digest the lineage links on -----------------------
    # Every row below keeps the ledger WRITING and the audit RUNNING, so a
    # suite that only checks for rows and verdicts stays green. What breaks is
    # the link: a digest that describes the wrong board, or none.
    ('the-row-records-no-parent-pose', 'pv',
     "           'parent_pose_sha256': _parent_pose,\n",
     "",
     (T_972,), 'KILLED'),

    # Hashing the OUTPUT at record time instead of the parsed input: identical
    # for an in-place write, None for every write to a new path.
    ('the-parent-pose-is-read-from-the-output', 'pv',
     "            _parent_pose = pose_digest(pose_table_of(before))\n",
     "            _parent_pose = file_pose_digest(output_file)\n",
     (T_972,), 'KILLED'),

    # A board digest that restates the parent: every write becomes a no-op
    # link, so any arrangement reaches the root.
    ('the-board-pose-restates-the-parent', 'pv',
     "        row['board_pose_sha256'] = file_pose_digest(output_file)\n",
     "        row['board_pose_sha256'] = row.get('parent_pose_sha256')\n",
     (T_972,), 'KILLED'),

    ('the-pose-digest-ignores-rotation', 'pv',
     "             round(((rot or 0.0) % 360.0) * 1e4) % 3600000, side]\n",
     "             0, side]\n",
     (T_972,), 'KILLED'),

    ('the-pose-digest-ignores-the-side', 'pv',
     "             round(((rot or 0.0) % 360.0) * 1e4) % 3600000, side]\n",
     "             round(((rot or 0.0) % 360.0) * 1e4) % 3600000, 'F']\n",
     (T_972,), 'KILLED'),

    # `% 360` on a float leaves -1e-17 at 360.0; without the integer modulo
    # after quantising, 0 and 360 are two arrangements and a normalising
    # rewrite breaks the chain.
    ('the-rotation-is-not-folded-after-quantising', 'pv',
     "             round(((rot or 0.0) % 360.0) * 1e4) % 3600000, side]\n",
     "             round(((rot or 0.0) % 360.0) * 1e4), side]\n",
     (T_972,), 'KILLED'),

    ('a-staging-row-carries-a-pose-digest', 'pv',
     "    if 'redacted' not in row:\n",
     "    if True:\n",
     (T_972, T_903), 'KILLED'),

    # The digest runs inside `commit_write`, AFTER the board is on disk. A
    # narrowed except lets a parse failure escape there, and the board ships
    # with no row -- #960's defect by another road.
    ('the-board-digest-can-raise-after-the-write', 'pv',
     "    except Exception:                            # noqa: BLE001\n"
     "        return None\n",
     "    except KeyError:\n"
     "        return None\n",
     (T_972,), 'KILLED'),

    # The PARENT digest's own guard. Reached only when the input parsed and
    # digesting it failed; a narrowed except there raises out of
    # `record_write` and the lever's write never happens.
    ('the-parent-digest-can-raise', 'pv',
     "        except Exception:                        # noqa: BLE001\n"
     "            _parent_pose = None\n",
     "        except KeyError:\n"
     "            _parent_pose = None\n",
     (T_972,), 'KILLED'),

    # The non-pending path: no production lever takes it today, which is
    # exactly why nothing else would notice its rows stop linking.
    ('a-direct-row-carries-no-board-pose', 'pv',
     "        return row\n"
     "    row['board_sha256'] = (sha256_file(output_file)\n"
     "                           if os.path.isfile(output_file) else None)\n"
     "    _stamp_board_pose(row, output_file)\n",
     "        return row\n"
     "    row['board_sha256'] = (sha256_file(output_file)\n"
     "                           if os.path.isfile(output_file) else None)\n",
     (T_972,), 'KILLED'),

    # Truncating instead of rounding: 137.253 * 1e4 is 1372529.99..., so an
    # angle the writer emits as `.6g` would digest one step off the same
    # angle read back from another spelling.
    ('the-pose-digest-truncates', 'pv',
     "    rows = [[ref, round(x * 1e6), round(y * 1e6),\n"
     "             round(((rot or 0.0) % 360.0) * 1e4) % 3600000, side]\n",
     "    rows = [[ref, int(x * 1e6), int(y * 1e6),\n"
     "             int(((rot or 0.0) % 360.0) * 1e4) % 3600000, side]\n",
     (T_972,), 'KILLED'),

    # Outside a regime the writer must cost nothing it did not cost before.
    ('the-digest-is-computed-outside-a-regime', 'pv',
     "    root = regime_for(output_file)\n"
     "    lever = active_lever()\n",
     "    file_pose_digest(input_file)\n"
     "    root = regime_for(output_file)\n"
     "    lever = active_lever()\n",
     (T_972,), 'KILLED'),

    # ---- #972: the lineage the audit walks --------------------------------
    # The finding itself, put back: the lineage is computed and then ignored,
    # which is the per-file scoping's blindness by another road.
    ('the-lineage-names-nothing', 'pa',
     "        drifted = [r for r in lin['drift'] if r not in unclaimed]\n",
     "        drifted = []\n",
     (T_972,), 'KILLED'),

    # "Some row produced my parent" instead of "my parent is reachable from
    # the staged board": two no-op writes of a hand-edited board vouch for
    # each other. The drift check still names the part, so what dies is the
    # lineage status the tests pin.
    ('reachability-becomes-membership', 'pa',
     "            if _linkable(b) and _linkable(p) and p in known and b not in known:\n"
     "                known[b] = _replay(known[p], r)\n",
     "            if _linkable(b) and b not in known:\n"
     "                known[b] = _replay(known.get(p, staged_table), r)\n",
     (T_972,), 'KILLED'),

    # A verified arrangement short-circuits to "nothing drifted": a row whose
    # delivered file carries a pose its claims do not is blessed.
    ('a-verified-board-skips-the-replay', 'pa',
     "        return _done('verified', known[dg], _who(made_by[dg]))\n",
     "        return _done('verified', None, _who(made_by[dg]))\n",
     (T_972,), 'KILLED'),

    # Replaying every written pose instead of every MOVE re-blesses a hand
    # edit that a write-all lever (place_seed, perturb) passed through.
    ('the-replay-applies-every-written-pose', 'pa',
     "    _sides = row.get('sides_written') or {}\n"
     "    for ref in row.get('refs_moved') or ():\n",
     "    _sides = row.get('sides_written') or {}\n"
     "    for ref in _poses:\n",
     (T_972,), 'KILLED'),

    # A redacted staging row has no digests by design; counting it as a
    # pre-digest row sends every restaged work dir back to the #972 path.
    ('a-staging-row-makes-the-ledger-legacy', 'pa',
     "    if any('board_pose_sha256' not in r or 'parent_pose_sha256' not in r\n"
     "           for _i, r in usable):\n",
     "    if any('board_pose_sha256' not in r or 'parent_pose_sha256' not in r\n"
     "           for r in rows):\n",
     (T_972,), 'KILLED'),

    ('a-staging-row-can-claim', 'pa',
     "            and row.get('lever') not in PV.FENCE_SENSITIVE_LEVERS\n"
     "            and 'redacted' not in row)\n",
     "            )\n",
     (T_972,), 'KILLED'),

    ('an-unknown-digest-scheme-links', 'pa',
     "        return isinstance(d, str) and d.startswith(_pfx)\n",
     "        return isinstance(d, str)\n",
     (T_972,), 'KILLED'),

    ('a-malformed-row-crashes-the-audit', 'pa',
     "    rows = [r for r in _read if _well_formed(r)]\n",
     "    rows = list(_read)\n",
     (T_972,), 'KILLED'),

    # The newest row is not the nearest arrangement: a hand edit in a copy of
    # the FIRST candidate would name every part the second one moved.
    ('the-nearest-state-is-the-newest', 'pa',
     "        return (len(diff), -(made_by[d] if made_by[d] is not None else -1))\n",
     "        return (0, -(made_by[d] if made_by[d] is not None else -1))\n",
     (T_972,), 'KILLED'),

    ('a-broken-lineage-outranks-unclaimed', 'pa',
     "    if unclaimed:\n",
     "    if unclaimed and lin['status'] != 'broken':\n",
     (T_972,), 'KILLED'),

    ('legacy-skips-the-any-pose-check', 'pa',
     "            if all(_pose_differs(got, w) for w in\n",
     "            if False and all(_pose_differs(got, w) for w in\n",
     (T_972,), 'KILLED'),

    ('a-broken-lineage-with-nothing-named-is-clean', 'pa',
     "    if lin['status'] == 'broken':\n"
     "        # Something moved outside the ledger",
     "    if False:\n"
     "        # Something moved outside the ledger",
     (T_972,), 'KILLED'),

    # Compared with the nearest state WITHOUT the chain's own recorded moves:
    # the write that broke the chain gets its legitimate parts named too.
    ('the-chain-is-not-replayed', 'pa',
     "    for i in chain:\n"
     "        expected = _replay(expected, rows[i])\n",
     "",
     (T_972,), 'KILLED'),

    ('an-unlinkable-ledger-is-clean', 'pa',
     "    if lin['status'] == 'unlinkable' and unverifiable:\n",
     "    if False:\n",
     (T_972,), 'KILLED'),

    # ---- #973: a delivery by copy or rename is recorded -------------------
    # Recorded AFTER the body: an undeclared caller is still refused, but only
    # once the output is already on disk -- the refusal made decorative.
    ('the-delivery-records-after-the-body', 'pv',
     "    row = record_write(input_file, output_file, placements, pending=True)\n"
     "    if row is None:\n"
     "        yield None\n"
     "        return\n"
     "    key = os.path.abspath(output_file)\n"
     "    try:\n"
     "        yield row\n",
     "    yield None\n"
     "    row = record_write(input_file, output_file, placements, pending=True)\n"
     "    if row is None:\n"
     "        return\n"
     "    key = os.path.abspath(output_file)\n"
     "    try:\n"
     "        pass\n",
     (T_973,), 'KILLED'),

    # Committed BEFORE the body: the row's board digest describes whatever
    # the output held before the copy.
    ('the-delivery-commits-before-the-body', 'pv',
     "        yield row\n"
     "        # A writer call to the same path inside the body keys its own pending\n"
     "        # row on this path and commits it; put this one back before committing.\n"
     "        _PENDING[key] = row\n"
     "        commit_write(output_file)\n",
     "        _PENDING[key] = row\n"
     "        commit_write(output_file)\n"
     "        yield row\n",
     (T_973,), 'KILLED'),

    ('a-writer-call-in-the-body-swallows-the-delivery-row', 'pv',
     "        _PENDING[key] = row\n",
     "",
     (T_973,), 'KILLED'),

    ('a-failed-delivery-leaves-its-row-pending', 'pv',
     "    finally:\n"
     "        if _PENDING.get(key) is row:\n"
     "            del _PENDING[key]\n",
     "    finally:\n"
     "        pass\n",
     (T_973,), 'KILLED'),

    # The defect itself, for place_seed --repair/--reseat.
    ('the-repair-delivery-is-an-empty-write-again', 'ps',
     "            write_placed_output(cur, _final, [])\n"
     "            with provenance.recorded_delivery(\n"
     "                    args.input_file, args.output_file,\n"
     "                    list(delivered_moves.values())):\n"
     "                shutil.copyfile(_final, args.output_file)\n",
     "            write_placed_output(cur, args.output_file, [])\n",
     (T_973,), 'KILLED'),

    ('the-repair-delivery-claims-nothing', 'ps',
     "                    list(delivered_moves.values())):\n",
     "                    []):\n",
     (T_973,), 'KILLED'),

    ('the-reseat-pass-moves-are-dropped', 'ps',
     "            for m in moves:\n",
     "            for m in (moves if tag == 'repair' else []):\n",
     (T_973,), 'KILLED'),

    # Recorded against the temp board the passes built: a parent no row
    # produced, so the lineage breaks on every legitimate repair.
    ('the-repair-delivery-reads-the-staged-board', 'ps',
     "                    args.input_file, args.output_file,\n",
     "                    cur, args.output_file,\n",
     (T_973,), 'KILLED'),

    ('siblings-reach-the-output-before-the-refusal', 'ps',
     "            write_placed_output(cur, _final, [])\n",
     "            write_placed_output(cur, _final, [])\n"
     "            copy_siblings(cur, args.output_file)\n",
     (T_973,), 'KILLED'),

    ('the-polish-rename-is-unrecorded-again', 'ps',
     "        with provenance.recorded_delivery(args.output_file, args.output_file,\n"
     "                                          moves):\n"
     "            os.replace(tmp, args.output_file)\n",
     "        os.replace(tmp, args.output_file)\n",
     (T_973,), 'KILLED'),

    # The defect itself, for place_route_loop.
    ('the-loop-delivery-is-a-bare-copy-again', 'rl',
     "    with provenance.recorded_delivery(args.input_file, args.output_file,\n"
     "                                      list(delivered_moves.values())):\n"
     "        shutil.copy(cur_file, args.output_file)\n",
     "    shutil.copy(cur_file, args.output_file)\n",
     (T_973,), 'KILLED'),

    ('the-loop-claims-only-its-last-accepted-round', 'rl',
     "            for m in ([dict(m) for m in reloc.moves]\n",
     "            delivered_moves.clear()\n"
     "            for m in ([dict(m) for m in reloc.moves]\n",
     (T_973,), 'KILLED'),

    ('a-rejected-round-is-claimed', 'rl',
     "            print(f\"  REJECTED - reverting, widening the nudge cap\"\n",
     "            delivered_moves.update({p['reference']: dict(p)\n"
     "                                    for p in (placements or [])})\n"
     "            print(f\"  REJECTED - reverting, widening the nudge cap\"\n",
     (T_973,), 'KILLED'),

    ('the-loop-delivery-reads-the-round-board', 'rl',
     "    with provenance.recorded_delivery(args.input_file, args.output_file,\n",
     "    with provenance.recorded_delivery(cur_file, args.output_file,\n",
     (T_973,), 'KILLED'),

    ('siblings-reach-the-loop-output-before-the-refusal', 'rl',
     "    with provenance.recorded_delivery(args.input_file, args.output_file,\n",
     "    copy_siblings(cur_file, args.output_file)\n"
     "    with provenance.recorded_delivery(args.input_file, args.output_file,\n",
     (T_973,), 'KILLED'),
]

# Every anchor must match its target exactly once BEFORE anything is
# rewritten. A stale anchor otherwise reports BROKEN mid-run, after the
# witnesses have been paid for; this is the one second (#877).
from mutation_anchors import preflight   # noqa: E402
preflight(__file__)


#: `run_all.py`'s self-skip code. A test that exits 77 asserted NOTHING, so
#: reading it as a kill is exactly the "most flattering possible bug" this
#: file's header refuses. It is a THIRD outcome, not a kill.
SKIP_EXIT = 77

#: T_903's end-of-run marker, and the FLOOR its derived row count must clear.
#:
#: The marker used to be a constant string, which proved only that the script
#: reached its last line -- delete half its checks and it still printed. It is
#: `903 coverage: <N> rows (...)` now, and this floor is what makes the guard
#: mean something: a gate that quietly stopped running arms is reported rather
#: than believed, and the later rows do not score KILLED for free.
#:
#: Raise it when the gate genuinely grows; that is the point of a pin.
COVERAGE = '903 coverage:'
COVERAGE_FLOOR = 30


def run(tests, want_coverage=False):
    """(killed, why) -- killed when ANY named test exits non-zero.

    `why` is `SKIP:<name>` when a test SELF-SKIPPED, which the caller turns
    into BROKEN: a mutation is only shown to be caught if the test that
    caught it actually ran. `want_coverage` additionally requires T_903's
    end-of-run marker on the UNMUTATED pass, so a gate that silently stopped
    covering half its arms is reported rather than believed.
    """
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1')
    for t in tests:
        r = subprocess.run([sys.executable, '-B', '-X', 'utf8', t],
                           cwd=ROOT, capture_output=True, text=True, env=env,
                           timeout=3600)
        out = r.stdout or ''
        if r.returncode == SKIP_EXIT:
            return False, 'SKIP:' + os.path.basename(t)
        # TWO SKIP SHAPES, not one. `run_all.py`'s contract is exit 77, and
        # `T_903` honours it -- but `T_PROV` predates that and self-skips a
        # missing fixture with a `SKIP:` line and exit ZERO. Reading only 77
        # let a fixtureless `T_PROV` sail through the baseline, which is the
        # exact "gate that asserted nothing" this function exists to catch.
        if any(ln.startswith('SKIP:') for ln in out.splitlines()):
            return False, 'SKIP:' + os.path.basename(t)
        if r.returncode != 0:
            return True, os.path.basename(t)
        if want_coverage and os.path.basename(t) == os.path.basename(T_903):
            line = next((ln for ln in out.splitlines()
                         if ln.startswith(COVERAGE)), None)
            if line is None:
                return True, 'NO-COVERAGE:' + os.path.basename(t)
            try:
                rows_run = int(line.split(COVERAGE, 1)[1].split()[0])
            except (IndexError, ValueError):
                return True, 'NO-COVERAGE:' + os.path.basename(t)
            if rows_run < COVERAGE_FLOOR:
                return True, (f'THIN-COVERAGE:{os.path.basename(t)} ran '
                              f'{rows_run} rows, floor is {COVERAGE_FLOOR}')
    return False, ''


def _drop_pycache(path):
    """A size-preserving rewrite inside one second is invisible to CPython.

    The `.pyc` validity check is (source mtime, source size). Rows here change
    a line's content without changing its length, and the whole battery runs
    inside a few seconds -- so a stale `.pyc` would serve the PREVIOUS row's
    mutant to every later import. Both stagers ARE imported as modules by
    other tests, so `tests/stress/__pycache__` matters as much as
    `py_placer/placement/__pycache__`.
    """
    cache = os.path.join(os.path.dirname(path), '__pycache__')
    if os.path.isdir(cache):
        shutil.rmtree(cache, ignore_errors=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--row')
    ap.add_argument('--list', action='store_true')
    a = ap.parse_args()
    if a.list:
        for n, t, _o, _w, tests, exp in ROWS:
            print(f'  {n:46s} {os.path.basename(TARGETS[t]):20s} {exp}')
        return 0

    rows = [r for r in ROWS if not a.row or r[0] == a.row]
    if not rows:
        print(f'no row named {a.row!r}')
        return 2

    # Uncommitted work in the files this rewrites is unrecoverable if the
    # process dies mid-row: a crash between the write and the restore leaves
    # the MUTANT on disk with nothing to put back. `git status --porcelain`
    # rather than `git diff --quiet`, so a STAGED change counts as dirty too.
    dirty = subprocess.run(['git', 'status', '--porcelain', '--']
                           + list(TARGETS.values()), cwd=ROOT,
                           capture_output=True, text=True).stdout.strip()
    if dirty:
        print('REFUSED: the files this battery rewrites have uncommitted '
              'changes:\n' + dirty + '\nA row that dies between the write and '
              'the restore leaves a MUTANT in your tree. Commit first.')
        return 2

    # The battery is only evidence if the gate passes UNMUTATED first.
    # Without this, a red tree -- or a test that SELF-SKIPS because a fixture
    # is missing -- scores every row KILLED and this file reports full
    # coverage for a gate that asserted nothing.
    every = tuple(dict.fromkeys(t for r in rows for t in r[4]))
    killed0, why0 = run(every, want_coverage=True)
    if killed0 or why0.startswith('SKIP:'):
        print('BROKEN: the gate does not pass on the UNMUTATED tree ({}). '
              'Every row would score KILLED against it, so nothing here would '
              'be evidence. Fix the tree first.'.format(why0 or 'unknown'))
        return 2
    originals = {k: open(v, encoding='utf-8').read() for k, v in TARGETS.items()}
    killed = survived = broken = disagree = 0
    try:
        for name, tgt, old, new, tests, exp in rows:
            src = originals[tgt]
            if src.count(old) != 1:
                print(f'  {name:46s} BROKEN (anchor matched {src.count(old)}x)')
                broken += 1
                continue
            with open(TARGETS[tgt], 'w', encoding='utf-8', newline='') as fh:
                fh.write(src.replace(old, new, 1))
            _drop_pycache(TARGETS[tgt])
            try:
                died, by = run(tests)
            finally:
                with open(TARGETS[tgt], 'w', encoding='utf-8',
                          newline='') as fh:
                    fh.write(src)
                _drop_pycache(TARGETS[tgt])
            if by.startswith('SKIP:'):
                print(f'  {name:46s} BROKEN ({by} -- it asserted nothing)')
                broken += 1
                continue
            got = 'KILLED' if died else 'SURVIVED'
            mark = '' if got == exp else '   *** DISAGREES with ' + exp
            if got != exp:
                disagree += 1
            killed += died
            survived += not died
            print(f'  {name:46s} {got:9s} {by}{mark}')
    finally:
        for k, v in TARGETS.items():
            with open(v, 'w', encoding='utf-8', newline='') as fh:
                fh.write(originals[k])
            _drop_pycache(v)
    print(f'\n{len(rows)} row(s): {killed} killed, {survived} survived, '
          f'{broken} broken, {disagree} disagreeing with expectation')
    return 1 if (broken or disagree) else 0


if __name__ == '__main__':
    sys.exit(main())
