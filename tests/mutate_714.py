#!/usr/bin/env python3
"""The #714 mutation battery, shipped so its numbers can be re-derived.

Same contract as `tests/mutate_829.py`, which this copies: a row is KILLED by a
failure OR an error; an anchor that does not match EXACTLY ONCE is BROKEN, never
silently skipped; `str.replace(old, new, 1)`, never `sed`; originals restored in
a `finally`; it REFUSES to start on a dirty engine; `.pyc` caches are dropped
around every row because several of these are size-preserving one-token edits;
and the gates must pass UNMUTATED first, because a battery whose gate was
already failing reports every row as killed.

ONE ADDITION TO THE CONTRACT, forced by this feature. The acceptance gate needs
pcbnew and lives under `tests/gui_parity/`, and a gate that SELF-SKIPS at exit 0
would report every row it guards as SURVIVED. So the baseline run asserts that
`test_714_mirror_pcbnew_parity.py` printed its `coverage:` line -- it exits 2
rather than skipping when pcbnew is absent, and this checks that it really ran.

WHY BOTH THE VALUE GATES AND THE ROUND TRIP SHIP. They are not redundant, and
the rows prove it in both directions:

  * `the-negation-goes-through-float` is killed by RT ALONE. Every value test
    passes -- the numbers are right, only their spelling changed.
  * `fp_arc-mirrors-without-the-start-end-swap` is killed by PAR ALONE. The
    round trip survives it (it is still an involution) and so does every
    geometric check, because mirroring three points without exchanging start
    and end gives the same curve through the same three points.

Four more survive the round trip for the same structural reason -- a wrong
transform that is still an involution round-trips perfectly -- and that is why
"flip and flip back" was never going to be sufficient on its own.

    python3 tests/mutate_714.py
    python3 tests/mutate_714.py --row the-local-y-mirror-is-omitted
    python3 tests/mutate_714.py --list
    python3 tests/mutate_714.py --selftest

NOT named `test_*.py`, so `tests/run_all.py` never collects it: it rewrites
engine files in place. One writer per tree.
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

WRITER = os.path.join(_ROOT, 'py_placer', 'placement', 'writer.py')
PARSER = os.path.join(_ROOT, 'py_router', 'kicad_parser.py')
PROV = os.path.join(_ROOT, 'py_placer', 'placement', 'provenance.py')
TARGETS = {'wr': WRITER, 'pa': PARSER, 'pv': PROV}

VAC = os.path.join(_TESTS, 'test_714_mirror_discriminates.py')
SELF = os.path.join(_TESTS, 'test_714_side_self_consistency.py')
RT = os.path.join(_TESTS, 'test_714_flip_roundtrip.py')
REF = os.path.join(_TESTS, 'test_714_refusals.py')
IDENT = os.path.join(_TESTS, 'test_714_identity_write_unchanged.py')
PERT = os.path.join(_TESTS, 'test_714_perturb_layer_flip.py')
PAR = os.path.join(_TESTS, 'gui_parity', 'test_714_mirror_pcbnew_parity.py')

# The cheap gates, run unmutated first. PAR is checked separately because it
# re-execs into KiCad's python and costs a couple of minutes.
BASELINE = (VAC, SELF, RT, REF, IDENT)

ROWS = [
    # ---------------------------------------------------- the mirror itself
    # THE reason this battery exists. Measured on the 22 tracked boards, 1198
    # of 1319 pad-bearing footprints (90.8%) have a pad POSITION multiset
    # closed under y -> -y, so if this row SURVIVES the fixture set has fallen
    # into that trap and every geometric assertion in the suite is decoration.
    ('the-local-y-mirror-is-omitted', 'wr',
     "                f\"{_negate_coord(m.group(5), ref, m.group(1))})\")\n",
     "                f\"{m.group(5)})\")\n",
     (VAC, PAR), 'KILLED'),

    # Killed ONLY by the two x-asymmetric fixtures (glasgow_revC U7,
    # orangecrab_ext_pll J4). Drop either and this goes green for free.
    ('the-mirror-is-on-x-instead-of-y', 'wr',
     "        return (f\"({m.group(1)}{m.group(2)}{m.group(3)}{m.group(4)}\"\n"
     "                f\"{_negate_coord(m.group(5), ref, m.group(1))})\")\n",
     "        return (f\"({m.group(1)}{m.group(2)}\"\n"
     "                f\"{_negate_coord(m.group(3), ref, m.group(1))}\"\n"
     "                f\"{m.group(4)}{m.group(5)})\")\n",
     (VAC, PAR), 'KILLED'),

    # Killed by SELF with no pcbnew at all -- the cheapest kill in the set,
    # and the one that proves the two independent side derivations disagree.
    ('pad-layers-are-not-toggled', 'wr',
     "        node = node[:lm.start()] + new_block + node[lend:]\n",
     "        node = node\n",
     (SELF, PAR), 'KILLED'),

    # `*.Cu` is KiCad's ALL-copper set and is already its own mirror.
    # Narrowing it to `B.Cu` silently drops 66 pads on rp2350 U8 alone.
    ('wildcards-are-toggled-too', 'pa',
     "    if name.startswith('F.'):\n        return 'B.' + name[2:]\n",
     "    if name.startswith('*'):\n        return 'B' + name[1:]\n"
     "    if name.startswith('F.'):\n        return 'B.' + name[2:]\n",
     (VAC, PAR), 'KILLED'),

    # ------------------------------------------------------------- the texts
    ('text-angle-negates-instead-of-180-minus', 'wr',
     "            new = _flip_at_angle(node, ref, lambda a: 180.0 - a)\n",
     "            new = _flip_at_angle(node, ref, lambda a: -a)\n",
     (PAR,), 'KILLED'),

    # The B->F direction in one line. Dropping only the token would leave an
    # empty `(justify)`; never deleting the node leaves a doubled mirror.
    ('justify-mirror-is-added-never-removed', 'wr',
     "        if 'mirror' in toks:\n",
     "        if False:\n",
     (RT, PAR), 'KILLED'),

    # The bug an adversarial verification actually found. pcbnew adds
    # `(justify mirror)` only to a text on a SIDED layer; 28 flip-eligible
    # texts on the corpus sit on a User layer, and none of the first thirteen
    # parity fixtures carried one.
    ('the-justify-toggle-is-unconditional', 'wr',
     "            if sided:\n                new = _flip_justify(new)\n",
     "            if True:\n                new = _flip_justify(new)\n",
     (PAR,), 'KILLED'),

    # ---------------------------------------------------------- the graphics
    # Killed by PAR ALONE. RT survives it -- reversing a sweep twice restores
    # it -- and so does every geometric check, because the mirrored arc passes
    # through the same three points either way.
    ('fp_arc-mirrors-without-the-start-end-swap', 'wr',
     "    if head == 'fp_arc':\n",
     "    if False:\n",
     (PAR,), 'KILLED'),

    # The 3D viewer applies the flip at render time; mirroring the offset
    # double-applies it. orangecrab J4 carries a large non-zero one.
    ('the-model-offset-is-mirrored', 'wr',
     "    'model',\n})\n",
     "})\n",
     (PAR,), 'KILLED'),

    # ------------------------------------------------- the shape of the bug
    # The EXACT silent failure #714 describes: the layer says B, the geometry
    # is still F, and `legality.footprint_side` reports B for the whole run.
    ('only-the-footprint-layer-changes', 'wr',
     "        elif head == 'pad':\n            new = _flip_pad(node, ref, old_rot, new_rot)\n",
     "        elif head == 'pad':\n            new = node\n",
     (SELF, VAC, PAR), 'KILLED'),

    # a = R + p, and a flip sends p -> -p, so a' = R_new - (a - R_old). The
    # translate formula a + delta_rot equals that only when p is 0 or 180.
    ('pad-angle-uses-the-translate-delta', 'wr',
     "    node = _flip_at_angle(node, ref, lambda a: new_rot - (a - old_rot))\n",
     "    node = _flip_at_angle(node, ref, lambda a: a + (new_rot - old_rot))\n",
     (VAC, PAR), 'KILLED'),

    # ---------------------------------------------------------- the refusals
    ('an-unknown-node-passes-through', 'wr',
     "        if head not in _FLIP_HANDLED:\n            raise SideFlipUnsupported(\n",
     "        if head not in _FLIP_HANDLED:\n            continue\n        if False:\n            raise SideFlipUnsupported(\n",
     (REF,), 'KILLED'),

    # EXPECTED TO SURVIVE, and correctly -- measured, not assumed. Disabling
    # the named table does NOT let a `(zone ...)` through: the head is not in
    # the whitelist either, so the generic arm refuses it AND names it, which
    # is what REF checks. Two independent guards cover the same construct.
    # Kept with its real expectation rather than deleted: a row that documents
    # overlapping defences is worth more than a missing one.
    ('the-zone-refusal-is-dropped', 'wr',
     "        if head in _FLIP_NAMED_REFUSALS:\n",
     "        if False:\n",
     (REF,), 'SURVIVED'),

    ('the-pad-construct-refusals-are-dropped', 'wr',
     "        if head in _PAD_REFUSALS:\n",
     "        if False:\n",
     (REF,), 'KILLED'),

    # -------------------------------------------------- format and the ledger
    # Killed by RT ALONE: every value test passes, because the numbers are
    # right and only their SPELLING changed (`0.500` -> `0.5`). The exact
    # mirror image of the fp_arc row above, and the reason both gates ship.
    ('the-negation-goes-through-float', 'wr',
     "    if float(tok) == 0.0:\n        return tok                      # '0' and '-0.000' both stay put\n"
     "    return tok[1:] if tok.startswith('-') else '-' + tok\n",
     "    return _fmt_mm(-float(tok))\n",
     (RT,), 'KILLED'),

    # The #829 outline gate must count a side-only change as a change, or a
    # footprint that draws the board outline can be mirrored silently.
    ('the-pose-eps-gate-ignores-side', 'wr',
     "                     or side_change)\n",
     "                     )\n",
     (REF,), 'SURVIVED'),

    # Proves the IDENT baseline is load-bearing rather than decorative: a
    # baseline committed in its own PR is self-certifying, so something has to
    # be able to redden it.
    ('the-coordinate-format-loses-precision', 'wr',
     "            new_at = f\"(at {new_x:.6f} {new_y:.6f} {new_rot:.6g})\"\n",
     "            new_at = f\"(at {new_x:.4g} {new_y:.4g} {new_rot:.6g})\"\n",
     (IDENT,), 'KILLED'),

    ('provenance-does-not-record-the-side', 'pv',
     "            elif _side_changed(fp, p):\n",
     "            elif False:\n",
     (PERT,), 'KILLED'),

    ('provenance-drops-the-sides-written-key', 'pv',
     "           'sides_written': _sides,\n",
     "",
     (PERT,), 'KILLED'),
]


def _git_clean(paths):
    r = subprocess.run(['git', 'diff', '--quiet', '--'] + list(paths), cwd=_ROOT)
    return r.returncode == 0


def _drop_pyc():
    """Remove cached bytecode under the trees we mutate.

    CPython validates a `.pyc` on the source's mtime with ONE-SECOND
    granularity and its size. Several rows here are size-preserving one-token
    edits (`if sided:` -> `if True:`, `elif _side_changed(...)` -> `elif
    False:`), so two rows applied inside the same second would otherwise leave
    the second import reading the FIRST mutant -- reported as a survivor for a
    row that was never really applied.
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
        if t == PAR and 'coverage:' not in r.stdout:
            # A killer gate that did not run reports every row as SURVIVED.
            return True, "PARITY GATE DID NOT RUN (no coverage line)"
    return False, "all named tests passed"


def _selftest():
    """Prove the .pyc defence instead of asserting it.

    Applies a size-preserving mutation twice within one second and requires
    the SECOND probe to observe the SECOND mutant. The probe prints a value
    that differs per mutant -- one that printed only "it ran" would report OK
    against a stale cache.
    """
    if not _git_clean([WRITER]):
        print("REFUSED: writer.py is dirty", file=sys.stderr)
        return 2
    src = io.open(WRITER, encoding='utf-8').read()
    anchor = "_POSE_EPS = 1e-6\n"
    if src.count(anchor) != 1:
        print(f"BROKEN selftest: anchor matched {src.count(anchor)} times",
              file=sys.stderr)
        return 2
    probe = [sys.executable, '-B', '-X', 'utf8', '-c',
             "import sys; sys.path[:0]=['py_router','py_placer'];"
             "import placement.writer as w; print(repr(w._POSE_EPS))"]
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1')
    seen = []
    try:
        for repl in ("_POSE_EPS = 2e-6\n", "_POSE_EPS = 3e-6\n"):
            _drop_pyc()
            io.open(WRITER, 'w', encoding='utf-8', newline='').write(
                src.replace(anchor, repl, 1))
            r = subprocess.run(probe, cwd=_ROOT, capture_output=True,
                               text=True, encoding='utf-8', env=env)
            seen.append(r.stdout.strip())
    finally:
        _drop_pyc()
        io.open(WRITER, 'w', encoding='utf-8', newline='').write(src)
    ok = len(seen) == 2 and all(seen) and seen[0] != seen[1]
    print(f"  selftest: two same-second size-preserving mutations, probe read "
          f"{seen} -- "
          + ('OK: the second import saw the SECOND mutant' if ok else
             'the probe cannot tell the mutants apart, or the second read a '
             'STALE .pyc'))
    return 0 if ok else 1


def _verify_anchors():
    originals = {k: io.open(p, encoding='utf-8').read()
                 for k, p in TARGETS.items()}
    bad = []
    for name, tgt, old, _new, _t, _e in ROWS:
        n = originals[tgt].count(old)
        if n != 1:
            bad.append(f"{name}: matched {n} times in {tgt}")
    for b in bad:
        print("  BROKEN ANCHOR " + b)
    print(f"  {len(ROWS) - len(bad)}/{len(ROWS)} anchors match exactly once")
    return 1 if bad else 0


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--row', action='append', default=None)
    ap.add_argument('--list', action='store_true')
    ap.add_argument('--selftest', action='store_true')
    ap.add_argument('--verify-anchors', action='store_true')
    ap.add_argument('--no-parity', action='store_true',
                    help="skip rows whose only gate is the pcbnew parity test "
                         "(they will report BROKEN-BY-OMISSION, never a pass)")
    a = ap.parse_args()

    if a.list:
        for name, tgt, _o, _n, tests, exp in ROWS:
            print(f"  {exp:9} {name}  [{tgt}] "
                  f"-> {', '.join(os.path.basename(t) for t in tests)}")
        return 0
    if a.selftest:
        return _selftest()
    if a.verify_anchors:
        return _verify_anchors()

    rows = ROWS
    if a.row:
        unknown = [n for n in a.row if n not in {r[0] for r in ROWS}]
        if unknown:
            print(f"no such row: {', '.join(unknown)}; try --list",
                  file=sys.stderr)
            return 2
        rows = [r for r in ROWS if r[0] in set(a.row)]
    if a.no_parity:
        rows = [r for r in rows if tuple(r[4]) != (PAR,)]

    if not _git_clean(TARGETS.values()):
        print("REFUSED: the target files are dirty. Restoring would write the "
              "COMMITTED text back over uncommitted work.", file=sys.stderr)
        return 2

    # The battery is only evidence if the gates pass UNMUTATED first.
    _drop_pyc()
    gates = list(BASELINE)
    if any(PAR in r[4] for r in rows):
        gates.append(PAR)
    baseline_killed, why = _run(tuple(gates))
    if baseline_killed:
        print(f"BROKEN: the gates do not pass on the UNMUTATED tree ({why}). "
              f"Every row would report KILLED and this run would exit 0.",
              file=sys.stderr)
        return 2
    print(f"  baseline: {len(gates)} gate(s) pass unmutated")

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
