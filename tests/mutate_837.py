#!/usr/bin/env python3
"""Mutation battery for #837: does the assembly-sides work have real gates?

Each row breaks ONE decision and names the gate that must catch it. A row that
SURVIVES is a hole, and every survivor here carries its measured reason in the
comment above it rather than being deleted.

    python3 -X utf8 tests/mutate_837.py --list
    python3 -X utf8 tests/mutate_837.py --verify-anchors
    python3 -X utf8 tests/mutate_837.py --selftest
    python3 -X utf8 tests/mutate_837.py

The runner contract, copied from `tests/mutate_714.py` and `mutate_829.py`
because these traps are not hypothetical:

* **The gates must pass UNMUTATED first.** A battery whose gates are already
  red scores every row KILLED and exits 0 -- the most flattering possible bug.
* **An anchor must match EXACTLY ONCE.** A stale anchor that matches nothing
  reports its row as killed; matching twice mutates something else. Both are
  BROKEN, never a silent skip, and `--verify-anchors` finds them in one second
  rather than fifty minutes into a run.
* **`.pyc` caches are dropped around every row.** CPython validates bytecode
  on mtime with one-second granularity plus size, and several rows here are
  size-preserving one-token edits, so two rows inside one second would leave
  the second import reading the FIRST mutant. `--selftest` PROVES that defence
  rather than asserting it.
* **`str.replace(old, new, 1)` on an in-memory copy, restored in a `finally`.**
  Never `sed`, never `git checkout --` (which would eat concurrent work).
* **It refuses to start on a dirty tree.**
"""
from __future__ import annotations

import argparse
import io
import os
import shutil
import subprocess
import sys

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)

LEG = os.path.join(_ROOT, 'py_placer', 'placement', 'legality.py')
FLOOR = os.path.join(_ROOT, 'py_placer', 'placement', 'floorplan.py')
OPTS = os.path.join(_ROOT, 'py_placer', 'placement', 'options.py')
CA = os.path.join(_ROOT, 'py_tools', 'check_assembly.py')
CC = os.path.join(_ROOT, 'py_tools', 'check_capacity.py')
TARGETS = {'le': LEG, 'fp': FLOOR, 'op': OPTS, 'ca': CA, 'cc': CC}

CEN = os.path.join(_TESTS, 'test_837_assembly_sides.py')
CAP = os.path.join(_TESTS, 'test_capacity_options.py')
SCH = os.path.join(_TESTS, 'test_549_floorplan_schema.py')
CLI = os.path.join(_TESTS, 'test_549_floorplan_cli.py')

BASELINE = (CEN, CAP, SCH, CLI)

ROWS = [
    # ------------------------------------------------- the census's two rules
    # THE reason the census exists. esp_prog and watchy are the witnesses: 3
    # and 1 zero-pad blocks on the back, and counting them makes a
    # single-sided board report a second reflow pass for silkscreen logos.
    ('zero-pad-blocks-count-as-parts', 'le',
     "        if not (fp.pads or ()):\n            zero_pad[side].append(ref)\n"
     "            continue\n",
     "        if not (fp.pads or ()):\n            zero_pad[side].append(ref)\n",
     (CEN,), 'KILLED'),

    # The other half: the verdict must come from the PAD-BEARING census. On 20
    # of 22 tracked boards the two agree, so only esp_prog and watchy can
    # catch this -- which is why they are both in the fixture table.
    ('the-verdict-is-taken-from-the-block-census', 'le',
     "    populated = tuple(s for s in ('F', 'B') if pad_bearing[s])\n",
     "    populated = tuple(s for s in ('F', 'B') if blocks[s])\n",
     (CEN,), 'KILLED'),

    # `drill > 0` is `footprint_has_through_pads`, which answers the
    # OBSTRUCTION question. watchy: 5 vs 1, because SW1-SW4 are SMD switches
    # with unplated alignment posts, and the text then tells the reader they
    # need wave soldering.
    ('the-through-hole-rule-is-bare-drill', 'le',
     "        if any(pad_is_plated_through(p) for p in fp.pads):\n",
     "        if any((getattr(p, 'drill', 0) or 0) > 0 for p in fp.pads):\n",
     (CEN,), 'KILLED'),

    # flat_hierarchy: 58 through-hole parts and 6 NPTH mounting holes. Folding
    # the holes into SMD reports 1 reflow pass for a board that gets none.
    ('npth-only-parts-are-counted-as-smd', 'le',
     "        elif any(getattr(p, 'pad_type', '') != 'np_thru_hole' "
     "for p in fp.pads):\n",
     "        elif True:\n",
     (CEN,), 'KILLED'),

    # Reflow passes counted from POPULATED faces rather than from the SMD
    # population. Same witness, opposite direction.
    ('reflow-passes-count-populated-faces', 'le',
     "        'reflow_passes': sum(1 for s in ('F', 'B') if smd[s]),\n",
     "        'reflow_passes': len(populated),\n",
     (CEN,), 'KILLED'),

    # Listing only the BACK leaves the printed arithmetic short on 7 of 22
    # boards -- interf_u reports 24 pad-bearing of 25 blocks and explains
    # nothing.
    ('only-back-side-zero-pad-blocks-are-listed', 'le',
     "            zero_pad[side].append(ref)\n",
     "            zero_pad[side].append(ref) if side == 'B' else None\n",
     (CEN,), 'KILLED'),

    # ------------------------------------------------------------- the rule
    # `ctx.sev` hard-defaults to ERROR. Nothing in the engine can move a part
    # between faces, so an error here is a red mark no run can clear -- the
    # defect `zone_side` already carries.
    ('the-rule-defaults-to-error', 'fp',
     "    sev = ctx.intent.severity_of('assembly_side', default=WARN)\n",
     "    sev = ctx.sev('assembly_side')\n",
     (CEN,), 'KILLED'),

    # `both` must RUN. Skipping it lands a DECLARED key in the abstention
    # channel: measured, `rules_skipped_arm 0 -> 7` and
    # `boards_clean_but_ungraded 0 -> 2` (kit-dev, sonde_u).
    ('a-declared-both-skips-the-rule', 'fp',
     "        return (intent.assembly or {}).get('sides') in _ASSEMBLY_SIDES\n",
     "        return (intent.assembly or {}).get('sides') in ('F', 'B')\n",
     (CEN,), 'KILLED'),

    # The face has to be named. `single` is under-specified and ulx3s is the
    # witness: 163 of its 226 pad-bearing parts are on the back.
    ('single-is-an-accepted-spelling', 'fp',
     "_ASSEMBLY_SIDES = ('F', 'B', 'both')\n",
     "_ASSEMBLY_SIDES = ('F', 'B', 'both', 'single')\n",
     (CEN,), 'KILLED'),

    # An assembly block that declares nothing grades nothing -- the #710
    # failure one level down.
    ('an-assembly-block-with-no-sides-loads', 'fp',
     "    elif assembly:\n",
     "    elif False:\n",
     (CEN,), 'KILLED'),

    # The emitter must declare the OBSERVED policy. A hardcoded `both` also
    # grades clean on every board, which is why the test compares the emitted
    # value against the census per board rather than only grading.
    ('the-emitter-declares-a-constant', 'fp',
     "    _assembly = {} if _cen['sides'] is None else {\n        'sides': "
     "_cen['sides'],\n",
     "    _assembly = {} if _cen['sides'] is None else {\n        'sides': "
     "'both',\n",
     (CEN,), 'KILLED'),

    ('the-reader-version-does-not-move', 'fp',
     "READER_VERSION = 3\n",
     "READER_VERSION = 2\n",
     (CEN,), 'KILLED'),

    # ------------------------------------------------------- the arithmetic
    ('the-charge-is-always-the-busiest-side', 'op',
     "    charged = sum(per_side.values()) if one_face else busiest\n",
     "    charged = busiest\n",
     (CAP, CEN), 'KILLED'),

    ('the-charge-is-always-the-sum', 'op',
     "    charged = sum(per_side.values()) if one_face else busiest\n",
     "    charged = sum(per_side.values())\n",
     (CAP, CEN), 'KILLED'),

    # `_digest` skips STRING values before the forced-key list, so a string
    # basis label never reaches the text channel -- the channel a human reads.
    ('the-basis-label-is-a-string', 'op',
     "            'charged_area_is_sum': one_face,\n",
     "            'charged_area_is_sum': 'sum' if one_face else 'max',\n",
     (CAP, CEN), 'KILLED'),

    # The threading, not the arithmetic: a policy that never reaches
    # `grow_board` leaves every board on the default basis, silently.
    ('the-policy-never-reaches-grow-board', 'op',
     "            kw['assembly_sides'] = assembly_sides\n",
     "            kw['assembly_sides'] = None\n",
     (CEN,), 'KILLED'),

    # ------------------------------------------------------------- the CLI
    ('the-census-reports-blocks-as-pad-bearing', 'ca',
     "            'parts_by_side': _cen['pad_bearing'],\n",
     "            'parts_by_side': _cen['blocks'],\n",
     (CEN,), 'KILLED'),

    ('the-observed-policy-is-hardcoded', 'ca',
     "            'assembly_sides': _cen['sides'],\n",
     "            'assembly_sides': 'F',\n",
     (CEN,), 'KILLED'),

    # ------------------------------------------------- disclosed SURVIVORS
    # Every tracked board's footprints are on F.Cu or B.Cu, so reading the raw
    # layer string and calling `footprint_side` agree on all 23. The helper is
    # used because the partition must not be ABLE to grow a third key that
    # `max()` would rank against the other two -- an invariant no corpus board
    # can exercise, which is exactly why it is stated here rather than left as
    # an untested preference.
    ('the-side-partition-reads-the-raw-layer-string', 'op',
     "        layer = 'B.Cu' if footprint_side(fp) == 'B' else 'F.Cu'\n",
     "        layer = getattr(fp, 'layer', None) or 'F.Cu'\n",
     (CAP, CEN), 'SURVIVED'),

    # `ctx.parts` (the graded-part population) and the census's pad-bearing
    # refs agree on every tracked board -- measured, 0 disagreements over 23.
    # They are NOT the same set by construction: `QuenchState` admits a
    # zero-pad footprint that draws a courtyard, and the census excludes it.
    # No corpus board carries one, so this row cannot be killed without a
    # synthesised fixture; it is kept as the disclosure that the two
    # populations are only empirically equal.
    ('the-rule-grades-the-quench-population', 'fp',
     "    for ref in ctx.assembly_census()['pad_bearing_refs'][other]:\n",
     "    for ref in [r for r, p in sorted(ctx.parts.items())\n"
     "               if p.side == other]:\n",
     (CEN,), 'SURVIVED'),

    # ------------------------- rows an adversarial review found ALIVE
    # The proposal has to hold the parts it is proposed for. The
    # pre-existing ceil audit runs UNDECLARED, where charged == busiest, so
    # it could not tell these apart: under `--assembly-sides F` on ulx3s the
    # proposal would be 65.7mm for parts needing 79.7mm -- 2007mm2 short,
    # 32%, with nothing failing.
    ('the-proposal-is-solved-from-the-busiest-side', 'op',
     "        exact = math.sqrt(max(0.0, charged)) + 2.0 * board_edge_clearance\n",
     "        exact = math.sqrt(max(0.0, busiest)) + 2.0 * board_edge_clearance\n",
     (CAP,), 'KILLED'),

    # `not_modelled` inside `measured` reaches neither `format_text` nor
    # `_digest`, so both disclosures ship only in --json -- the channel the
    # bool type was chosen precisely so as not to depend on.
    ('the-disclosure-goes-back-inside-measured', 'op',
     "        'not_modelled': '; '.join(\n",
     "        'measured_not_modelled': '; '.join(\n",
     (CEN,), 'KILLED'),

    # Forcing the bool into the digest evicts a real number: `_digest`
    # returns `out[:max(limit, len(forced))]` and the forced list is already
    # at the limit, so `part_area_mm2` left the text channel on 21 of 22
    # boards.
    ('the-basis-bool-is-forced-into-the-digest', 'op',
     "                  'charged_area_mm2',\n",
     "                  'charged_area_mm2', 'charged_area_is_sum',\n",
     (CEN,), 'KILLED'),

    # An intent that declares nothing is not one that declares `both`.
    ('an-undeclared-intent-reads-as-both', 'cc',
     "            declared = (load_intent(a.intent).assembly or {}).get('sides')\n",
     "            declared = load_intent(a.intent).assembly_sides()\n",
     (CEN,), 'KILLED'),

    ('the-policy-source-is-not-forwarded', 'cc',
     "        'assembly_sides_source': sides_src,\n",
     "",
     (CEN,), 'KILLED'),

    ('the-declared-sides-are-not-forwarded', 'cc',
     "        'assembly_sides': sides,\n",
     "",
     (CEN,), 'KILLED'),
]


def _git_clean(paths):
    r = subprocess.run(['git', 'diff', '--quiet', '--'] + list(paths), cwd=_ROOT)
    return r.returncode == 0


def _drop_pyc():
    for base in (os.path.join(_ROOT, 'py_placer'),
                 os.path.join(_ROOT, 'py_router')):
        for dirpath, dirnames, _files in os.walk(base):
            if os.path.basename(dirpath) == '__pycache__':
                shutil.rmtree(dirpath, ignore_errors=True)
                dirnames[:] = []


def _run(tests):
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', KRT_NO_BANNER='1')
    for t in tests:
        r = subprocess.run([sys.executable, '-B', '-X', 'utf8', t],
                           cwd=_ROOT, capture_output=True, text=True,
                           encoding='utf-8', errors='replace', env=env)
        if r.returncode != 0:
            return True, f"{os.path.basename(t)} exit {r.returncode}"
    return False, "all named tests passed"


def _verify_anchors():
    bad = 0
    for name, target, old, _new, _tests, _exp in ROWS:
        src = io.open(TARGETS[target], encoding='utf-8').read()
        n = src.count(old)
        if n != 1:
            print(f"BROKEN {name}: anchor matched {n} times in "
                  f"{os.path.basename(TARGETS[target])}")
            bad += 1
    print(f"{len(ROWS) - bad}/{len(ROWS)} anchors match exactly once")
    return 1 if bad else 0


def _selftest():
    """Prove the .pyc defence, do not assert it.

    Two size-preserving mutations inside one second; the SECOND probe must
    observe the SECOND mutant. The probe prints a per-mutant value, so a probe
    that printed only "it ran" could not report OK against a stale cache.
    """
    if not _git_clean([LEG]):
        print("REFUSED: legality.py is dirty", file=sys.stderr)
        return 2
    src = io.open(LEG, encoding='utf-8').read()
    anchor = "        'reflow_passes': sum(1 for s in ('F', 'B') if smd[s]),\n"
    if src.count(anchor) != 1:
        print(f"BROKEN selftest: anchor matched {src.count(anchor)} times",
              file=sys.stderr)
        return 2
    probe = [sys.executable, '-B', '-X', 'utf8', '-c',
             "import sys;"
             "sys.path[:0]=[r'%s',r'%s'];"
             "from kicad_parser import parse_kicad_pcb as P;"
             "from placement.legality import assembly_census as A;"
             "print(A(P(r'%s'))['reflow_passes'])"
             % (os.path.join(_ROOT, 'py_router'),
                os.path.join(_ROOT, 'py_placer'),
                os.path.join(_ROOT, 'kicad_files', 'ulx3s.kicad_pcb'))]
    seen = []
    try:
        for value in ('7', '9'):
            _drop_pyc()
            io.open(LEG, 'w', encoding='utf-8').write(
                src.replace(anchor, f"        'reflow_passes': {value},\n", 1))
            r = subprocess.run(probe, cwd=_ROOT, capture_output=True,
                               text=True,
                               env=dict(os.environ,
                                        PYTHONDONTWRITEBYTECODE='1'))
            seen.append((r.stdout or '').strip())
    finally:
        io.open(LEG, 'w', encoding='utf-8').write(src)
        _drop_pyc()
    ok = seen == ['7', '9']
    print(f"two same-second size-preserving mutations, probe read {seen} -- "
          f"{'OK: the second import saw the SECOND mutant' if ok else 'FAILED'}")
    return 0 if ok else 1


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--list', action='store_true')
    ap.add_argument('--row', action='append', default=None)
    ap.add_argument('--verify-anchors', action='store_true')
    ap.add_argument('--selftest', action='store_true')
    a = ap.parse_args(argv)

    if a.list:
        for name, target, _o, _n, tests, exp in ROWS:
            print(f"{exp:9} {target}  {name}  "
                  f"[{', '.join(os.path.basename(t) for t in tests)}]")
        killed = sum(1 for r in ROWS if r[5] == 'KILLED')
        print(f"{len(ROWS)} rows: {killed} KILLED, {len(ROWS) - killed} "
              f"SURVIVED (each disclosed with its reason)")
        return 0
    if a.verify_anchors:
        return _verify_anchors()
    if a.selftest:
        return _selftest()

    if not _git_clean(list(TARGETS.values())):
        print("REFUSED: an engine file this battery mutates is dirty. Commit "
              "or stash first -- the restore path would eat the change.",
              file=sys.stderr)
        return 2
    if _verify_anchors():
        return 2

    # The gates must pass UNMUTATED, or every row reports KILLED and this
    # run exits 0 having measured nothing.
    _drop_pyc()
    failed, why = _run(BASELINE)
    if failed:
        print(f"BROKEN: the gates do not pass on the UNMUTATED tree ({why}). "
              f"Every row would report KILLED and this run would exit 0.",
              file=sys.stderr)
        return 2
    print(f"baseline: {why}")

    rows = [r for r in ROWS if not a.row or r[0] in set(a.row)]
    originals = {k: io.open(v, encoding='utf-8').read()
                 for k, v in TARGETS.items()}
    wrong = broken = 0
    try:
        for name, target, old, new, tests, expect in rows:
            src = originals[target]
            _drop_pyc()
            io.open(TARGETS[target], 'w', encoding='utf-8').write(
                src.replace(old, new, 1))
            killed, why = _run(tests)
            io.open(TARGETS[target], 'w', encoding='utf-8').write(src)
            _drop_pyc()
            got = 'KILLED' if killed else 'SURVIVED'
            mark = 'ok ' if got == expect else 'BAD'
            if got != expect:
                wrong += 1
            print(f"  {mark} {got:9} {name}  ({why})")
    finally:
        for k, v in TARGETS.items():
            io.open(v, 'w', encoding='utf-8').write(originals[k])
        _drop_pyc()

    killed = sum(1 for r in rows if r[5] == 'KILLED')
    print(f"\n{len(rows)} rows, {killed} expected KILLED, "
          f"{len(rows) - killed} expected SURVIVED, "
          f"{wrong} disagreeing with expectation, {broken} broken")
    return 1 if (wrong or broken) else 0


if __name__ == '__main__':
    sys.exit(main())
