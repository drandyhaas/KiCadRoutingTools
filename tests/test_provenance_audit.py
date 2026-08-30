#!/usr/bin/env python3
"""The provenance audit, attacked the way a hand script would attack it.

The case that matters is the one run 19 produced: a board whose poses are
genuinely BLIND -- nothing read the control -- and genuinely NOT the engine's.
`fence_audit` says CLEAN on that, correctly, because it asks a different
question. This must not.

Five cases, and the third is the load-bearing one:

  1. a run through a registered lever            -> CLEAN (0)
  2. a write through the funnel with no lever    -> refused BEFORE the write
  3. a hand edit of `(at ...)` as RAW TEXT       -> VIOLATION (4)
  4. no ledger, and poses MOVED                  -> VIOLATION (4)
  5. no ledger, and nothing moved                -> UNPROVEN (5)

Case 3 is why the audit reconciles the BOARD rather than reading the log: a
script that never imports the writer leaves no row to be missing, and an audit
that trusted the ledger would report CLEAN on a board it never authored.

Cases 4 and 5 are the same check split on the board's own evidence, and an
audit found them merged the wrong way: `audit()` returned UNPROVEN whenever
there were no rows, WITHOUT looking at the poses, which swallowed exactly the
run-19 case the instrument was built for -- a purely hand-placed board has no
ledger BECAUSE nothing engine-side ran, and it came back "I cannot prove it"
instead of "I proved it false". Runs predating the instrument are still
protected, by the earlier manifest check: a work dir nobody staged returns
UNPROVEN before the ledger is consulted at all.

The last block is the one that would have caught the whole thing being inert:
it ARMS a regime and runs a real CLI. Nothing called `declare_lever` outside
this file, so every real run was UNPROVEN, and arming a regime by hand made
`place_optimize.py` raise -- it is in LEVER_REGISTRY but declared nothing, so
the gate refused the engine itself.
"""
import io
import os
import re
import shutil
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for p in (REPO,):
    if p not in sys.path:
        sys.path.insert(0, p)
        sys.path.insert(0, os.path.join(p, 'py_router'))
        sys.path.insert(0, os.path.join(p, 'py_tools'))
        sys.path.insert(0, os.path.join(p, 'py_placer'))
        sys.path.insert(0, os.path.join(p, 'tests', 'stress'))

BOARD = os.path.join(REPO, 'kicad_files', 'splitflap_driver.kicad_pcb')

passed = failed = 0


def check(name, ok, detail=""):
    global passed, failed
    passed += bool(ok)
    failed += not ok
    print(f"  {'OK  ' if ok else 'FAIL'} {name}{(' -- ' + detail) if detail else ''}")


if not os.path.isfile(BOARD):
    print("SKIP: fixture missing")
    sys.exit(0)

from kicad_parser import parse_kicad_pcb
from placement import provenance as PV
from placement.writer import write_placed_output
import provenance_audit as PA


def _stage(src, dst):
    """Copy the board and its siblings into the work dir (#441).

    The audit needs exactly two things: a staged board and a regime
    manifest naming it. Nothing here perturbs or piles the board.
    """
    shutil.copy2(src, dst)
    for ext in ('.kicad_pro', '.kicad_dru'):
        sib = os.path.splitext(src)[0] + ext
        if os.path.isfile(sib):
            shutil.copy2(sib, os.path.splitext(dst)[0] + ext)


def fresh():
    d = tempfile.mkdtemp()
    wd = os.path.join(d, 'wk')
    os.makedirs(wd)
    staged = os.path.join(wd, 'board.kicad_pcb')
    _stage(BOARD, staged)
    PV.start_regime(wd, staged)
    return wd, staged


def some_moves(staged, n=3):
    pcb = parse_kicad_pcb(staged)
    refs = sorted(pcb.footprints)[:n]
    return [{'reference': r, 'new_x': 12.0 + i, 'new_y': 34.0,
             'new_rotation': 0.0} for i, r in enumerate(refs)]


# --------------------------------------------------------------------------
# 1. a registered lever
# --------------------------------------------------------------------------
wd, staged = fresh()
out = os.path.join(wd, 'seeded.kicad_pcb')
with PV.declare_lever('place_optimize.py', ['place_optimize.py', staged]):
    write_placed_output(staged, out, some_moves(staged))
code, doc = PA.audit(wd, out)
check("a run through a registered lever is CLEAN",
      code == PA.CLEAN, f"{code} {doc.get('reason')}")
check("and it says which lever", doc.get('levers') == ['place_optimize.py'],
      str(doc.get('levers')))
check("it reconciles the moved refs, not merely the row count",
      doc['moved'] == 3 and doc['claimed'] >= 3, str(doc))

# --------------------------------------------------------------------------
# 2. the funnel with no lever declared
# --------------------------------------------------------------------------
wd, staged = fresh()
try:
    write_placed_output(staged, os.path.join(wd, 'x.kicad_pcb'),
                        some_moves(staged))
    check("an undeclared write through the funnel is refused", False,
          "it wrote anyway")
except PV.UnaidedViolation as e:
    check("an undeclared write through the funnel is refused at write time",
          'no registered lever' in str(e), str(e)[:120])
    check("and the refusal names the CALLER, which is the run-19 detector",
          'test_provenance_audit.py' in str(e), str(e)[:200])

# an unregistered lever is refused too
wd, staged = fresh()
try:
    with PV.declare_lever('arrange.py', ['arrange.py']):
        write_placed_output(staged, os.path.join(wd, 'y.kicad_pcb'),
                            some_moves(staged))
    check("an UNREGISTERED lever is refused", False, "it wrote anyway")
except PV.UnaidedViolation as e:
    check("an unregistered lever is refused, naming itself",
          'arrange.py' in str(e) and 'LEVER_REGISTRY' in str(e), str(e)[:140])

# --------------------------------------------------------------------------
# 3. THE ONE THAT MATTERS: a raw-text hand edit, bypassing the funnel entirely
# --------------------------------------------------------------------------
wd, staged = fresh()
out = os.path.join(wd, 'seeded.kicad_pcb')
with PV.declare_lever('place_optimize.py', ['place_optimize.py']):
    write_placed_output(staged, out, some_moves(staged))
# Now a hand script edits the file directly -- no import, no funnel, no row.
src = open(out, encoding='utf-8').read()
hand = os.path.join(wd, 'hand.kicad_pcb')
pcb = parse_kicad_pcb(out)
moved_already = {m['reference'] for m in some_moves(staged)}
victim = next(r for r in sorted(pcb.footprints) if r not in moved_already)

# Edit the VICTIM'S OWN footprint block. A non-greedy regex spanning from
# `(footprint` to the Reference property silently matches the FIRST
# footprint's `(at` and the victim's name, so it moves the wrong part -- and
# then the audit is being asked about a ref that was already moved legally,
# which is not the case this test exists for.
from kicad_parser import find_matching_paren
starts = [m.start() for m in re.finditer(r'\(footprint\s+"', src)]
edited, n = src, 0
for st in reversed(starts):
    end = find_matching_paren(src, st)
    block = src[st:end]
    if not re.search(r'\(property\s+"Reference"\s+"' + re.escape(victim)
                     + r'"', block):
        continue
    new_block, k = re.subn(r'\(at\s+[\d.-]+\s+[\d.-]+',
                           '(at 77.5 88.5', block, count=1)
    if k:
        edited = src[:st] + new_block + src[end:]
        n = 1
    break
open(hand, 'w', encoding='utf-8').write(edited)
hand_moved = PA.moved_refs(PA.poses(out), PA.poses(hand))
check("the hand edit moved exactly the victim (else the case proves nothing)",
      n == 1 and hand_moved == [victim],
      f"substitutions {n}, moved {hand_moved}, victim {victim}")

code, doc = PA.audit(wd, hand)
check("a raw-text hand edit is a VIOLATION, not CLEAN",
      code == PA.VIOLATION, f"{code} {doc.get('verdict')}")
check("and it names the ref nothing authored",
      victim in (doc.get('unclaimed_refs') or []),
      f"{victim} vs {doc.get('unclaimed_refs')}")
check("the reason says why the log could not have caught it",
      'compares the BOARD' in doc.get('reason', ''), doc.get('reason'))

# --------------------------------------------------------------------------
# 4. no ledger -> UNPROVEN, never VIOLATION
# --------------------------------------------------------------------------
wd, staged = fresh()
out = os.path.join(wd, 'seeded.kicad_pcb')
with PV.declare_lever('place_optimize.py', ['place_optimize.py']):
    write_placed_output(staged, out, some_moves(staged))
os.unlink(os.path.join(wd, PV.LEDGER_NAME))
code, doc = PA.audit(wd, out)
# Deleting the ledger of a run that MOVED parts is indistinguishable from
# never having had one, and the board still shows the movement -- so this is
# a violation with a witness, not an unmeasured run. The audit used to return
# UNPROVEN here because it checked `if not rows` BEFORE computing `moved`,
# which swallowed the purely hand-placed case (run 19's) the whole instrument
# was built for: it came back 5 instead of 4.
check("a missing ledger with MOVED poses is a VIOLATION, not merely unproven",
      code == PA.VIOLATION, f"{code} {doc.get('verdict')}")
check("and it says the board is the witness",
      'the board is the witness' in doc.get('reason', ''), doc.get('reason'))
check("it names the refs nothing accounts for",
      len(doc.get('unclaimed_refs') or []) == 3,
      str(doc.get('unclaimed_refs')))

# ...but a staged run where NOTHING moved is genuinely unproven, not accused.
wd2, staged2 = fresh()
out2 = os.path.join(wd2, 'copy.kicad_pcb')
import shutil as _sh
_sh.copy(staged2, out2)
code2, doc2 = PA.audit(wd2, out2)
check("a staged run with no ledger and NO movement stays UNPROVEN",
      code2 == PA.UNPROVEN, f"{code2} {doc2.get('verdict')}")
check("and that branch says nothing was measured, rather than accusing",
      'Not a violation' in doc2.get('reason', ''), doc2.get('reason'))

# a work dir that was never staged is UNPROVEN too
d = tempfile.mkdtemp()
code, doc = PA.audit(d)
check("an unstaged work dir is UNPROVEN, not a violation",
      code == PA.UNPROVEN, f"{code} {doc.get('verdict')}")

# --------------------------------------------------------------------------
# Outside a regime the gate is INERT -- no ledger, no refusal, no change
# --------------------------------------------------------------------------
d = tempfile.mkdtemp()
plain = os.path.join(d, 'plain.kicad_pcb')
write_placed_output(BOARD, plain, [])
check("outside a regime the writer is unchanged and writes no ledger",
      os.path.isfile(plain)
      and not os.path.exists(os.path.join(d, PV.LEDGER_NAME)))

# --------------------------------------------------------------------------
# The CLI
# --------------------------------------------------------------------------
wd, staged = fresh()
out = os.path.join(wd, 'seeded.kicad_pcb')
with PV.declare_lever('place_seed.py', ['place_seed.py']):
    write_placed_output(staged, out, some_moves(staged))
r = subprocess.run(
    [sys.executable, '-X', 'utf8',
     os.path.join('tests', 'stress', 'provenance_audit.py'),
     '--workdir', wd],
    capture_output=True, text=True, cwd=REPO, timeout=600,
    env=dict(os.environ, PYTHONIOENCODING='utf-8'))
check("the CLI exits 0 on a clean chain", r.returncode == 0,
      f"rc={r.returncode} {r.stderr[-200:]}")
check("the CLI prints a VERDICT and a JSON_SUMMARY",
      'VERDICT: CLEAN' in r.stdout and 'JSON_SUMMARY:' in r.stdout,
      r.stdout[-200:])

# --------------------------------------------------------------------------
# ARMED, END TO END, through a real CLI
#
# The instrument had no working armed state at all: nothing called
# declare_lever outside this file, so every real run was UNPROVEN, and arming
# a regime by hand made place_optimize.py RAISE -- it is in LEVER_REGISTRY but
# declared nothing, so the gate refused the engine itself. Both halves are
# asserted here, because "it works if you call it right" was already true and
# was useless.
# --------------------------------------------------------------------------
fh = os.path.join(REPO, 'kicad_files', 'flat_hierarchy.kicad_pcb')
if os.path.isfile(fh):
    # A board with a sibling project, staged explicitly so the quench grades
    # at the board's own floor; the lever is the quench CLI, whose __main__
    # declares itself for the whole run.
    _d = tempfile.mkdtemp()
    wd = os.path.join(_d, 'wk')
    os.makedirs(wd)
    staged = os.path.join(wd, 'board.kicad_pcb')
    _stage(fh, staged)
    PV.start_regime(wd, staged)
    out = os.path.join(wd, 'placed.kicad_pcb')
    r = subprocess.run(
        [sys.executable, '-X', 'utf8',
         os.path.join('py_placer', 'place_optimize.py'),
         staged, out, '--max-passes', '2', '--max-displacement', '3'],
        capture_output=True, text=True, cwd=REPO, timeout=900,
        env=dict(os.environ, PYTHONIOENCODING='utf-8'))
    check("a real CLI runs INSIDE an armed regime without raising",
          r.returncode == 0 and os.path.isfile(out),
          f"rc={r.returncode} {(r.stdout + r.stderr)[-300:]}")
    code, doc = PA.audit(wd, out)
    check("and the run audits CLEAN, with the lever named",
          code == PA.CLEAN and doc.get('levers') == ['place_optimize.py'],
          f"{code} {doc.get('verdict')} {doc.get('levers')}")
    check("every moved pose is claimed -- not a subset",
          doc['moved'] > 0 and doc['claimed'] >= doc['moved'],
          f"moved {doc['moved']} claimed {doc['claimed']}")
    check("the ledger records the CALLER as the CLI, not <unknown>",
          any('place_optimize' in (row.get('caller') or '')
              for row in PV.read_ledger(wd)),
          str([row.get('caller') for row in PV.read_ledger(wd)]))
else:
    check("the flat_hierarchy fixture exists", False, fh)

# The refusal must precede the write. It used to raise AFTER f.write(), so the
# poses were already on disk and the "gate" described a file it had helped
# produce.
wd, staged = fresh()
victim = os.path.join(wd, 'never.kicad_pcb')
try:
    write_placed_output(staged, victim, some_moves(staged))
    check("an undeclared write is refused BEFORE the file is written", False,
          "it wrote anyway")
except PV.UnaidedViolation:
    check("an undeclared write is refused BEFORE the file is written",
          not os.path.exists(victim),
          "the board must not exist -- refusing means not writing")

# --------------------------------------------------------------------------
# A CLAIM IS ONLY GOOD FOR THE POSE IT CLAIMED
#
# `claimed` used to be keyed on the REF. A real run legitimately moves most of
# the board, so most refs become claimed -- and a claimed ref is then a
# licence: hand-move it afterwards and the audit still grades CLEAN, because
# it only ever asked "did some lever touch this ref". The instrument was
# weakest exactly where the run was most active.
# --------------------------------------------------------------------------
import re as _re                                                # noqa: E402


def _hand_move(board, ref, dx, dy):
    """Move one footprint by editing the file text -- no lever, no row."""
    txt = open(board, encoding='utf-8').read()
    i = txt.index('"%s"' % ref)
    j = txt.rindex('(at ', 0, i)
    k = txt.index(')', j)
    nums = _re.findall(r'-?\d+\.?\d*', txt[j:k + 1])
    new = '(at %.6f %.6f%s)' % (float(nums[0]) + dx, float(nums[1]) + dy,
                                (' ' + nums[2]) if len(nums) > 2 else '')
    with open(board, 'w', encoding='utf-8') as f:
        f.write(txt[:j] + new + txt[k + 1:])
    return new


wd, staged = fresh()
out = os.path.join(wd, 'seeded.kicad_pcb')
_moves = some_moves(staged)
with PV.declare_lever('place_optimize.py', ['place_optimize.py', staged]):
    write_placed_output(staged, out, _moves)
code, doc = PA.audit(wd, out)
check('the engine run itself is CLEAN', code == PA.CLEAN, doc.get('reason'))
_victim = _moves[0]['reference']
check('...and the ref the hand edit will target IS claimed by it',
      _victim not in (doc.get('unclaimed_refs') or []), _victim)

_hand_move(out, _victim, 37.0, 21.0)
code, doc = PA.audit(wd, out)
check('a hand edit of a CLAIMED ref is a violation, not CLEAN',
      code == PA.VIOLATION,
      f"{doc.get('verdict')} -- a ref-keyed claim is inheritable, so every "
      f"part the engine legitimately moved becomes launderable")
check('and it is named as DRIFTED, not as unclaimed',
      doc.get('drifted_refs') == [_victim],
      f"drifted={doc.get('drifted_refs')} unclaimed={doc.get('unclaimed_refs')}")
check('the reason says the ref is claimed but the POSE is not',
      'POSE' in (doc.get('reason') or ''), doc.get('reason'))

# The pose the lever wrote is what is compared, so an untouched delivery of
# the SAME poses stays clean -- or the check above would fire on every run.
wd2, staged2 = fresh()
out2 = os.path.join(wd2, 'seeded.kicad_pcb')
with PV.declare_lever('place_optimize.py', ['place_optimize.py', staged2]):
    write_placed_output(staged2, out2, some_moves(staged2))
_rows2 = PV.read_ledger(wd2)
check('the ledger row records WHERE each ref was put',
      bool(_rows2) and bool(_rows2[-1].get('poses_written')),
      'poses_written is what makes the claim pose-keyed rather than ref-keyed')
check('a delivery nobody touched afterwards is still CLEAN',
      PA.audit(wd2, out2)[0] == PA.CLEAN)

# --------------------------------------------------------------------------
# A POSE CLAIM IS PER FILE, and the loop's NORMAL shape must stay CLEAN
#
# One declared lever writing several candidates and keeping one is what
# `place_route_loop`, `place_seed` and `place_portfolio` all do. Taking the
# newest row for a ref REGARDLESS of which board it wrote convicted the KEPT
# board of being "not where the lever put it" -- the lever did put it there,
# in a file that was then discarded. That fires on almost every real run, and
# exit 4 is an affirmative accusation, so this is worse than not checking.
# --------------------------------------------------------------------------
wd, staged = fresh()
_kept = os.path.join(wd, 'kept.kicad_pcb')
_cand = os.path.join(wd, 'cand2.kicad_pcb')
_refs = [m['reference'] for m in some_moves(staged)]
with PV.declare_lever('place_route_loop.py', ['place_route_loop.py', staged]):
    write_placed_output(staged, _kept, some_moves(staged))
    # A SECOND candidate from the same lever, at different poses, later
    # rejected and deleted -- which is what a loop lap does.
    _alt = [dict(m, new_x=m['new_x'] + 9.0, new_y=m['new_y'] + 7.0)
            for m in some_moves(staged)]
    write_placed_output(staged, _cand, _alt)
os.remove(_cand)
code, doc = PA.audit(wd, _kept)
check('a lever that wrote two candidates does not convict the kept one',
      code == PA.CLEAN,
      f"{doc.get('verdict')} drifted={doc.get('drifted_refs')} -- the pose "
      f"claim was read off a row that wrote a DIFFERENT board")
check('...and the refs are still CLAIMED, not merely unchecked',
      doc.get('claimed') == len(_refs) and not doc.get('unclaimed_refs'),
      str(doc.get('claimed')) + ' ' + str(doc.get('unclaimed_refs')))

# The same ledger, audited against the OTHER candidate, is also clean: each
# board is compared to the row that wrote IT.
wd2, staged2 = fresh()
_a = os.path.join(wd2, 'a.kicad_pcb')
_b = os.path.join(wd2, 'b.kicad_pcb')
with PV.declare_lever('place_route_loop.py', ['place_route_loop.py', staged2]):
    write_placed_output(staged2, _a, some_moves(staged2))
    write_placed_output(staged2, _b,
                        [dict(m, new_x=m['new_x'] + 9.0)
                         for m in some_moves(staged2)])
check('either candidate audits clean against its own row',
      PA.audit(wd2, _a)[0] == PA.CLEAN and PA.audit(wd2, _b)[0] == PA.CLEAN,
      f"a={PA.audit(wd2, _a)[1].get('verdict')} "
      f"b={PA.audit(wd2, _b)[1].get('verdict')}")

# ...and a hand edit of the kept board is STILL caught, or the fix above has
# simply turned the check off.
_hand_move(_a, _refs[0], 31.0, 17.0)
code, doc = PA.audit(wd2, _a)
check('but a hand edit of a claimed ref is still a violation',
      code == PA.VIOLATION and doc.get('drifted_refs') == [_refs[0]],
      f"{doc.get('verdict')} drifted={doc.get('drifted_refs')}")

# A board no ledger row names cannot be pose-checked at all. That must be
# UNVERIFIABLE and said out loud, never a silent pass.
wd3, staged3 = fresh()
_orig = os.path.join(wd3, 'seeded.kicad_pcb')
with PV.declare_lever('place_optimize.py', ['place_optimize.py', staged3]):
    write_placed_output(staged3, _orig, some_moves(staged3))
_copy = os.path.join(wd3, 'shipped.kicad_pcb')
_sh.copyfile(_orig, _copy)
code, doc = PA.audit(wd3, _copy)
# THE STRONGER PROPERTY. `place_route_loop.py:737` delivers by
# `shutil.copy(cur_file, args.output_file)`, so the ledger names the delivered
# board NOWHERE -- and scoping the pose claim to rows that wrote it turned the
# check OFF for the rig's own main output. Measured before the fallback: a
# hand-move of C1 by +37/+21 mm in that board graded CLEAN. So a board the
# ledger does not name must still have its poses checked, against the whole
# ledger.
check('a copied board the ledger names nowhere is CLEAN when honest',
      code == PA.CLEAN, f"{doc.get('verdict')} {doc.get('reason')}")
_hand = os.path.join(wd3, 'shipped_edited.kicad_pcb')
_sh.copyfile(_copy, _hand)
with open(_hand, encoding='utf-8') as _fh:
    _txt = _fh.read()
# Move ONE part by hand in the copied board, without a lever and without a
# ledger row: string surgery on the first footprint's (at x y), which is
# exactly what a teammate 'just nudging something' would leave behind.
_i_fp = _txt.find('(footprint')
_i_at = _txt.find('(at ', _i_fp) if _i_fp >= 0 else -1
_i_end = _txt.find(')', _i_at) if _i_at >= 0 else -1
_parts = _txt[_i_at + 4:_i_end].split() if _i_end > 0 else []
if len(_parts) >= 2:
    _nx = float(_parts[0]) + 37.0
    _ny = float(_parts[1]) + 21.0
    _rest = ' '.join(_parts[2:])
    _new_at = '(at %s %s%s' % (_nx, _ny, (' ' + _rest) if _rest else '')
    with open(_hand, 'w', encoding='utf-8') as _fh:
        _fh.write(_txt[:_i_at] + _new_at + _txt[_i_end:])
    code_h, doc_h = PA.audit(wd3, _hand)
    check('and a HAND EDIT to that same copied board is still caught',
          code_h == PA.VIOLATION,
          f"{doc_h.get('verdict')} {doc_h.get('reason')} -- a board the "
          f'ledger does not name must not turn the pose check off')
else:
    check('and a HAND EDIT to that same copied board is still caught', False,
          'could not locate a footprint (at x y) to move; the fixture is '
          'not exercising the path it claims to')


# --------------------------------------------------------------------------
# THE ROUTER IS A POSE AUTHOR
#
# route.py calls `write_placed_output` for #666's scoped cap move. Absent from
# LEVER_REGISTRY it raised inside the funnel, route.py's own `except
# Exception` swallowed the refusal, and the router silently skipped a repair
# it performs everywhere else -- so arming the fence changed the engine the
# rig was measuring.
# --------------------------------------------------------------------------
check('route.py is a registered lever', 'route.py' in PV.LEVER_REGISTRY,
      str(PV.LEVER_REGISTRY))
_rt = open(os.path.join(REPO, 'py_router', 'route.py'), encoding='utf-8').read()
check('route.py still calls the pose funnel (or the entry is pointless)',
      'write_placed_output' in _rt)

# RUN the __main__ block and ask provenance what is declared. A source-text
# check for 'declare_lever' passes on a route.py whose IMPORT was removed --
# the name still appears at the call site, and route.py wraps the whole
# declaration in try/except so the failure is silent by construction. That is
# the exact shape of the bug, so the test has to be a runtime one.
import contextlib as _ctx                                       # noqa: E402
import runpy as _runpy                                          # noqa: E402
_argv0 = sys.argv
sys.argv = ['route.py', '--help']
try:
    with _ctx.redirect_stdout(io.StringIO()):
        _runpy.run_path(os.path.join(REPO, 'py_router', 'route.py'),
                        run_name='__main__')
except SystemExit:
    pass                                   # --help, as intended
except Exception as _e:                                        # noqa: BLE001
    print(f'    (route.py __main__ raised {type(_e).__name__}: {_e})')
finally:
    sys.argv = _argv0
_live = PV.active_lever()
check('route.py DECLARES its lever when it actually runs',
      bool(_live) and _live.get('lever') == 'route.py',
      f'active_lever()={_live} -- route.py swallows the declaration failure, '
      f'so nothing but a runtime check can see this')

wd3, staged3 = fresh()
_cap = some_moves(staged3, 1)
try:
    with PV.declare_lever('route.py', ['route.py', staged3]):
        _ok = write_placed_output(staged3, staged3, _cap)
    check("the router's cap move SUCCEEDS inside an armed regime", _ok,
          'the move is skipped inside the fence and performed outside it, '
          'so the rig would be measuring a different router')
except PV.UnaidedViolation as e:
    check("the router's cap move SUCCEEDS inside an armed regime", False,
          f'refused: {e}')


# --------------------------------------------------------------------------
# EVERY registered lever must actually declare one
#
# An audit found 5 of 12 registry entries declaring nothing, three of which
# write poses through the funnel -- so under an armed regime they raised
# `UnaidedViolation` and the gate refused the engine itself. That is the exact
# failure the registry exists to prevent, and it survived the commit that
# claimed to fix it because only seven CLIs were armed.
# --------------------------------------------------------------------------
import glob as _glob

_missing = []
for _lever in PV.LEVER_REGISTRY:
    _hits = (_glob.glob(os.path.join(REPO, 'py_placer', _lever))
             + _glob.glob(os.path.join(REPO, 'py_router', _lever))
             + _glob.glob(os.path.join(REPO, 'tests', 'stress', _lever))
             + _glob.glob(os.path.join(REPO, 'py_placer', 'placement', _lever)))
    if not _hits:
        continue
    _src = open(_hits[0], encoding='utf-8').read()
    # A library with no __main__ is reached through a declaring CLI; the
    # innermost-wins rule covers it.
    if 'declare_lever' not in _src and '__main__' in _src:
        _missing.append(_lever)
check("every registered lever with a __main__ actually declares one",
      not _missing,
      f"{_missing} -- these raise UnaidedViolation under an armed regime, "
      f"refusing the engine")
check("and the registry is not empty (else the check is vacuous)",
      len(PV.LEVER_REGISTRY) >= 8, str(len(PV.LEVER_REGISTRY)))
check("a tool that does NOT write poses is named, not silently registered",
      hasattr(PV, 'NOT_POSE_WRITERS')
      and 'beautify_labels.py' in PV.NOT_POSE_WRITERS,
      "beautify_labels writes silkscreen through write_label_output, not the "
      "pose funnel -- registering it would imply coverage that does not exist")

# The check above reads the SOURCE for `declare_lever`. That is not enough, and
# run 20 measured the gap: `place_fanout_clearance.py` contained the call, so
# the static check passed, while the module raised
#   NameError: name 'sys' is not defined
# on EVERY invocation including --help -- because the `declare_lever(...,
# sys.argv)` line was the file's only use of `sys` and nothing imported it. A
# registered lever that cannot start is a lever that cannot author anything, so
# the registry was describing a capability the repo did not have.
#
# --help is the cheapest possible execution: it exercises import, module-level
# code and the argparse build without touching a board.
_dead, _unresolved = [], []
for _lever in PV.LEVER_REGISTRY:
    # Same three legs as the static check above. Dropping one made the two loops
    # disagree about which levers they cover.
    _hits = (_glob.glob(os.path.join(REPO, 'py_placer', _lever))
             + _glob.glob(os.path.join(REPO, 'py_router', _lever))
             + _glob.glob(os.path.join(REPO, 'tests', 'stress', _lever))
             + _glob.glob(os.path.join(REPO, 'py_placer', 'placement', _lever)))
    if not _hits:
        # NOT a silent pass. A lever renamed without updating LEVER_REGISTRY
        # degrades every check in this file to vacuous, and `len(REGISTRY) >= 8`
        # guards a vacuous registry, not vacuous resolution.
        _unresolved.append(_lever)
        continue
    _src = open(_hits[0], encoding='utf-8').read()
    if '__main__' not in _src:
        continue                       # a library, reached through a CLI
    try:
        _r = subprocess.run([sys.executable, '-X', 'utf8', _hits[0], '--help'],
                            capture_output=True, text=True, timeout=120,
                            cwd=REPO)
    except subprocess.TimeoutExpired:
        # Unwrapped, this escaped the file: no named failure and no
        # "N passed, M failed" summary at all.
        _dead.append(f'{_lever}: hung for 120s on --help')
        continue
    if _r.returncode != 0:
        _tail = ((_r.stderr or _r.stdout or '').strip().splitlines() or [''])[-1]
        _dead.append(f'{_lever}: rc={_r.returncode} {_tail[:110]}')
check("every registered lever actually STARTS (--help exits 0)",
      not _dead,
      ' | '.join(_dead) + "  -- a lever that cannot start cannot author poses, "
      "and the source-text check above cannot see this")
check("every registered lever RESOLVES to a file",
      not _unresolved,
      f"{_unresolved} -- an entry that resolves to nothing makes both the "
      f"static and the runtime check above vacuous for it")

# The --help loop covers LEVER_REGISTRY, which is ~10 files. The same bug class
# -- a module-level name used but never imported -- is repo-wide and kills any
# CLI it touches. Measured while fixing place_fanout_clearance.py:
# tests/stress/fix_mixed_net_refs.py, a step the stress RUNBOOK tells you to
# run, had been dead 129 commits with `NameError: name 'os' is not defined`.
# A static sweep costs ~0.4s and covers every file, not just the registry.
_TOOL_DIRS = ('tests/stress', 'py_placer', 'py_tools')
try:
    _ruff = subprocess.run(['ruff', 'check', '--select', 'F821',
                            '--output-format=concise'] + list(_TOOL_DIRS),
                           capture_output=True, text=True, timeout=300, cwd=REPO)
    _f821 = [ln for ln in (_ruff.stdout or '').splitlines() if 'F821' in ln]
    check("no CLI directory uses a name it never imported (ruff F821)",
          not _f821,
          ' | '.join(_f821[:6]) + f"  ({len(_f821)} total) -- this is the "
          f"place_fanout_clearance / fix_mixed_net_refs bug class; each one is "
          f"a tool that dies on every invocation")
except (OSError, subprocess.TimeoutExpired):
    print('  SKIP no ruff on PATH -- the F821 sweep did not run, so the '
          'undefined-name bug class is UNCHECKED here (the --help loop still '
          'covers LEVER_REGISTRY)')

print(f"\n{passed} passed, {failed} failed")
sys.exit(1 if failed else 0)
