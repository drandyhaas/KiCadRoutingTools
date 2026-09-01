#!/usr/bin/env python3
"""The #829 mutation battery, shipped so its numbers can be re-derived.

Same contract as `tests/mutate_830.py`, which this copies: a row is KILLED by a
failure OR an error; an anchor that does not match EXACTLY ONCE is BROKEN,
never silently skipped; `str.replace(old, new, 1)`, never `sed`; originals
restored in a `finally`; it REFUSES to start on a dirty engine; and the gate
must pass UNMUTATED first, because a battery whose gate was already failing
reports every row as killed.

Every row reverts or weakens one decision. The three that matter most are the
ones that do NOT simply undo the fix:

  * `the-lock-is-on-owns_edge_cuts` -- the rule the issue itself proposed, and
    the one this fix deliberately does not implement. It still refuses the
    structural owner, so it passes the headline check; it must be KILLED by the
    carried-relief checks, which is what stops "lock every owner" from creeping
    back in and freezing crkbd's 184 per-LED windows.
  * `the-tripwire-arms-off-the-per-ref-cache` -- restores a defect this
    commit's own test found: arming off a cache the per-ref gate populates as a
    side effect, so the tripwire silently disarms whenever the moved footprints
    are not the ones carrying Edge.Cuts.
  * `the-fingerprint-is-the-whole-outline` -- drops the structural-only masking.
    A whole-outline fingerprint refuses a CARRIED relief's legitimate move,
    which is the failure the naive version actually had.

NOT named `test_*.py`, so `tests/run_all.py` never collects it: it rewrites
engine files in place. One writer per tree.

    python3 tests/mutate_829.py
    python3 tests/mutate_829.py --row the-pad-gate-is-the-only-gate
    python3 tests/mutate_829.py --list
    python3 tests/mutate_829.py --selftest
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

PARSER = os.path.join(_ROOT, 'py_router', 'kicad_parser.py')
QUENCH = os.path.join(_ROOT, 'py_placer', 'placement', 'quench.py')
PORTF = os.path.join(_ROOT, 'py_placer', 'placement', 'portfolio.py')
WRITER = os.path.join(_ROOT, 'py_placer', 'placement', 'writer.py')
TARGETS = {'pa': PARSER, 'qu': QUENCH, 'po': PORTF, 'wr': WRITER}

T829 = os.path.join(_TESTS, 'test_829_edge_cuts_owner.py')
T550 = os.path.join(_TESTS, 'test_550_circle_curve_bounds.py')
TREF = os.path.join(_TESTS, 'test_ref_label_parser.py')

ROWS = [
    # --- the classifier ---------------------------------------------------
    ('the-owner-scan-forgets-the-reference', 'pa',
     "        if pts:\n"
     "            owners.setdefault(_footprint_reference(fp_text)[0], "
     "[]).extend(pts)\n",
     "        if False:\n"
     "            owners.setdefault(_footprint_reference(fp_text)[0], "
     "[]).extend(pts)\n",
     (T829,), 'KILLED'),

    ('every-owner-is-structural', 'pa',
     "    return {ref: not all(inside(px, py) for px, py in pts)\n"
     "            for ref, pts in owners.items()}\n",
     "    return {ref: True for ref, pts in owners.items()}\n",
     (T829,), 'KILLED'),

    ('no-owner-is-structural', 'pa',
     "    if board_only_bounds is None:\n",
     "    if True:\n",
     (T829,), 'KILLED'),

    # The ladder step I got wrong first: bounds-but-no-closed-ring is NOT
    # "the footprints are the outline". splitflap_driver is exactly that
    # board, and treating it as no-outline marked an under-body window
    # structural.
    ('bounds-without-rings-means-no-outline', 'pa',
     "    rings = [r for r in (board_only_outers or []) if len(r) >= 3]\n",
     "    rings = []\n    board_only_bounds = None\n",
     (T829,), 'KILLED'),

    # EXPECTED SURVIVOR, kept rather than deleted. The fingerprint masks
    # CARRIED owners before comparing, so the only geometry left is the
    # board-level outline plus the structural owners -- and every gate above
    # already stops a structural owner moving. The rings/cutouts terms are
    # therefore hardening for a case the gates make unreachable: a structural
    # owner moving in a way that changes a ring without changing the bounding
    # box. It stays because it starts failing the day someone removes the
    # masking (see `the-fingerprint-is-the-whole-outline`, which IS killed) or
    # widens what may move.
    ('the-fingerprint-ignores-rings-and-cutouts', 'pa',
     "        tuple(sorted(ring(r) for r in (board_info.board_outlines or []))),\n"
     "        tuple(sorted(ring(r) for r in (board_info.board_cutouts or []))),\n",
     "        None,\n        None,\n",
     (T829,), 'SURVIVED'),

    ('the-fingerprint-is-the-whole-outline', 'pa',
     "    if carried:\n        content = _mask_footprint_blocks(content, "
     "only_refs=carried)\n",
     "    if False:\n        content = _mask_footprint_blocks(content, "
     "only_refs=carried)\n",
     (T829,), 'KILLED'),

    # --- the movable-set gates --------------------------------------------
    ('the-quench-lock-is-reverted', 'qu',
     "            locked = (ref in locked_refs\n"
     "                      or owns_outline\n",
     "            locked = (ref in locked_refs\n",
     (T829,), 'KILLED'),

    ('the-quench-lock-is-on-owns_edge_cuts', 'qu',
     "            owns_outline = getattr(fp, 'owns_board_outline', False)\n",
     "            owns_outline = getattr(fp, 'owns_edge_cuts', False)\n",
     (T829,), 'KILLED'),

    ('the-quench-lock-is-silent', 'qu',
     "            if owns_outline:\n                outline_locked.append(ref)\n",
     "            if False:\n                outline_locked.append(ref)\n",
     (T829,), 'KILLED'),

    ('free_refs-is-reverted', 'po',
     "        if getattr(fp, 'owns_board_outline', False):\n",
     "        if False:\n",
     (T829,), 'KILLED'),

    ('free_refs-locks-on-owns_edge_cuts', 'po',
     "        if getattr(fp, 'owns_board_outline', False):\n",
     "        if getattr(fp, 'owns_edge_cuts', False):\n",
     (T829,), 'KILLED'),

    ('free_refs-refuses-without-a-reason', 'po',
     '                refused[ref] = ("draws the board outline -- moving it '
     'would "\n'
     '                                "resize the board, which is not this '
     'tool\'s "\n'
     '                                "to change (#829)")\n',
     '                refused[ref] = "refused"\n',
     (T829,), 'KILLED'),

    # --- the writer -------------------------------------------------------
    ('the-writer-skips-instead-of-raising', 'wr',
     "            raise OutlineOwnerMove(\n"
     "                f\"{ref} draws the board outline (Edge.Cuts geometry outside \"\n",
     "            continue\n            raise OutlineOwnerMove(\n"
     "                f\"{ref} draws the board outline (Edge.Cuts geometry outside \"\n",
     (T829,), 'KILLED'),

    ('the-writer-backstop-is-gone', 'wr',
     "        if _OWNS_OUTLINE_RE.search(fp_text) and _draws_board_outline(\n"
     "                input_file, ref):\n",
     "        if False:\n",
     (T829,), 'KILLED'),

    ('the-tripwire-is-gone', 'wr',
     "    if _outline_owner_map(input_file):\n",
     "    if False:\n",
     (T829,), 'KILLED'),

    ('the-tripwire-arms-off-the-per-ref-cache', 'wr',
     "    if _outline_owner_map(input_file):\n",
     "    if _OUTLINE_OWNER_CACHE.get(input_file):\n",
     (T829,), 'KILLED'),

    # --- the parser refactor must not break what it touched ---------------
    ('the-reference-helper-drops-the-6-7-spelling', 'pa',
     "        ref_match = re.search(r'\\(fp_text\\s+reference\\s+\"([^\"]+)\"', "
     "fp_text)\n",
     "        ref_match = None\n",
     (TREF, T550), 'KILLED'),
]


def _git_clean(paths):
    r = subprocess.run(['git', 'diff', '--quiet', '--'] + list(paths),
                       cwd=_ROOT)
    return r.returncode == 0


def _drop_pyc():
    """Remove every cached bytecode file under the trees we mutate.

    CPython validates a `.pyc` on the source's mtime with ONE-SECOND
    granularity and its size. Two size-preserving rows applied inside the same
    second can therefore leave the second import reading the FIRST mutant --
    reported as a survivor for a row that was never really applied. Several
    rows here are size-preserving one-token edits (`owns_board_outline` ->
    `owns_edge_cuts` is not, but `if carried:` -> `if False:` families are
    close), so this is not hypothetical.
    """
    for base in (os.path.join(_ROOT, 'py_placer'),
                 os.path.join(_ROOT, 'py_router')):
        for dirpath, dirnames, _files in os.walk(base):
            if os.path.basename(dirpath) == '__pycache__':
                shutil.rmtree(dirpath, ignore_errors=True)
                dirnames[:] = []


def _run(tests):
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1')
    for t in tests:
        r = subprocess.run([sys.executable, '-B', '-X', 'utf8', t],
                           cwd=_ROOT, capture_output=True, text=True,
                           encoding='utf-8', errors='replace', env=env)
        if r.returncode != 0:
            return True, f"{os.path.basename(t)} exit {r.returncode}"
    return False, "all named tests passed"


def _selftest():
    """Prove the .pyc defence instead of asserting it.

    Applies a size-preserving mutation to kicad_parser.py twice within one
    second and requires the SECOND probe to observe the SECOND mutant. The
    probe prints the classification, which differs per mutant -- a probe that
    printed only "it ran" would report OK against a stale cache.
    """
    if not _git_clean([PARSER]):
        print("REFUSED: kicad_parser.py is dirty", file=sys.stderr)
        return 2
    src = io.open(PARSER, encoding='utf-8').read()
    anchor = "_OUTLINE_EPS = 1e-6\n"
    if src.count(anchor) != 1:
        print(f"BROKEN selftest: anchor matched {src.count(anchor)} times",
              file=sys.stderr)
        return 2
    probe = [sys.executable, '-B', '-X', 'utf8', '-c',
             "import sys; sys.path.insert(0, 'py_router');"
             "import kicad_parser as k; print(repr(k._OUTLINE_EPS))"]
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1')
    seen = []
    try:
        for repl in ("_OUTLINE_EPS = 2e-6\n", "_OUTLINE_EPS = 3e-6\n"):
            _drop_pyc()
            io.open(PARSER, 'w', encoding='utf-8', newline='').write(
                src.replace(anchor, repl, 1))
            r = subprocess.run(probe, cwd=_ROOT, capture_output=True,
                               text=True, encoding='utf-8', env=env)
            seen.append(r.stdout.strip())
    finally:
        _drop_pyc()
        io.open(PARSER, 'w', encoding='utf-8', newline='').write(src)
    ok = len(seen) == 2 and all(seen) and seen[0] != seen[1]
    print(f"  selftest: two same-second size-preserving mutations, probe read "
          f"{seen} -- "
          + ('OK: the second import saw the SECOND mutant' if ok else
             'the probe cannot tell the mutants apart, or the second read a '
             'STALE .pyc'))
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--row', action='append', default=None)
    ap.add_argument('--list', action='store_true')
    ap.add_argument('--selftest', action='store_true')
    a = ap.parse_args()

    if a.list:
        for name, tgt, _o, _n, tests, exp in ROWS:
            print(f"  {exp:9} {name}  [{tgt}] "
                  f"-> {', '.join(os.path.basename(t) for t in tests)}")
        return 0
    if a.selftest:
        return _selftest()

    rows = ROWS
    if a.row:
        unknown = [n for n in a.row if n not in {r[0] for r in ROWS}]
        if unknown:
            print(f"no such row: {', '.join(unknown)}; try --list",
                  file=sys.stderr)
            return 2
        rows = [r for r in ROWS if r[0] in set(a.row)]

    if not _git_clean(TARGETS.values()):
        print("REFUSED: the target files are dirty. Restoring would write the "
              "COMMITTED text back over uncommitted work.", file=sys.stderr)
        return 2

    # The battery is only evidence if the gate passes UNMUTATED first.
    _drop_pyc()
    baseline_killed, why = _run((T829,))
    if baseline_killed:
        print(f"BROKEN: the gate does not pass on the UNMUTATED tree ({why}).",
              file=sys.stderr)
        return 2

    originals = {k: io.open(p, encoding='utf-8').read()
                 for k, p in TARGETS.items()}
    verdicts = []
    try:
        for name, tgt, old, new, tests, expect in rows:
            src = originals[tgt]
            n = src.count(old)
            if n != 1:
                verdicts.append((name, 'BROKEN', f"anchor matched {n} times"))
                print(f"  BROKEN   {name} -- anchor matched {n} times")
                continue
            _drop_pyc()
            io.open(TARGETS[tgt], 'w', encoding='utf-8', newline='').write(
                src.replace(old, new, 1))
            killed, why = _run(tests)
            io.open(TARGETS[tgt], 'w', encoding='utf-8', newline='').write(src)
            _drop_pyc()
            got = 'KILLED' if killed else 'SURVIVED'
            mark = 'ok' if got == expect else 'WRONG'
            verdicts.append((name, got, why))
            print(f"  {got:9}{'' if mark == 'ok' else ' WRONG'} "
                  f"{name} -- {why}")
    finally:
        for k, p in TARGETS.items():
            io.open(p, 'w', encoding='utf-8', newline='').write(originals[k])
        _drop_pyc()

    killed = sum(1 for _n, g, _w in verdicts if g == 'KILLED')
    survived = sum(1 for _n, g, _w in verdicts if g == 'SURVIVED')
    broken = [n for n, g, _w in verdicts if g == 'BROKEN']
    wrong = [r[0] for r, (_n, g, _w) in zip(rows, verdicts)
             if g != r[5] and g != 'BROKEN']
    print(f"\n{len(verdicts)} row(s): {killed} killed, {survived} survived, "
          f"{len(broken)} broken, {len(wrong)} disagreeing with expectation")
    if wrong:
        print("  WRONG: " + ', '.join(wrong))
    if broken:
        print("  BROKEN: " + ', '.join(broken))
    return 1 if (wrong or broken) else 0


if __name__ == '__main__':
    sys.exit(main())
