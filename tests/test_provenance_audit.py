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
import json
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
     '--workdir', wd, '--mode', 'audit'],
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
