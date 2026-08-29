#!/usr/bin/env python3
"""The #698 mutation battery, shipped so its numbers can be re-derived.

`tests/test_698_reseat_acceptance.py` records what each arm kills. A count is
only checkable if the exact source edit is written down, so the edits live here
as data, next to the numbers they produced.

Every row carries an EXPECTATION. An inert row recorded as an expected survivor
is a finding; an inert row quietly deleted is a hole. A row whose verdict does
not match its expectation is reported as WRONG.

WHY THIS BATTERY EXISTS FOR THIS CHANGE. #698 is a change to an ACCEPTANCE
rule, and an acceptance rule is the easiest thing in this repo to test
vacuously: every arm can be satisfied by a board that would have come out right
under the old rule too. Two of this change's own arms were caught that way
before the battery existed -- arm G asserted the auto path on a healthy board,
where the auto census is empty by construction and the gate is never reached at
all, so every assertion in it passed while testing nothing.

NOT named `test_*.py`, so `tests/run_all.py` does not collect it: it REWRITES
the engine in place. One writer per tree -- do not run it while a suite, an A/B
replay or a review is reading the same checkout. It refuses to start on a dirty
engine, because restoring would write the COMMITTED text back over uncommitted
work.

    python3 tests/mutate_698.py
    python3 tests/mutate_698.py --row hpwl-is-hard-too
    python3 tests/mutate_698.py --list

A row is KILLED by a FAILURE **or an ERROR**: several of these make an arm raise
rather than fail, and a battery that counted only failures would call that a
survivor.

An anchor that does not match EXACTLY ONCE is reported as BROKEN rather than
skipped -- a battery that silently applies nothing reports every row as a
survivor, which reads as a catastrophic test failure and is really a stale
anchor.

Python `str.replace`, never `sed`.

THE MEASURED TABLE IS IN THE HEADER OF `test_698_reseat_acceptance.py`, FROM THE
RUN -- never predicted here and never edited afterwards to match.
"""
from __future__ import annotations

import argparse
import io
import os
import subprocess
import sys

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)

SEEDER = os.path.join(_ROOT, 'py_placer', 'placement', 'seeder.py')
RECON = os.path.join(_ROOT, 'py_placer', 'placement', 'reconstruct.py')
QUENCH = os.path.join(_ROOT, 'py_placer', 'placement', 'quench.py')
TARGETS = {'s': SEEDER, 'r': RECON, 'q': QUENCH}

T698 = os.path.join(_TESTS, 'test_698_reseat_acceptance.py')
T630 = os.path.join(_TESTS, 'test_630_seeder_eviction.py')
T702 = os.path.join(_TESTS, 'test_702_quench_intent_gate.py')
TPS = os.path.join(_TESTS, 'test_place_seed.py')

ROWS = [
    # ---- the acceptance rule ----------------------------------------------
    ('the-explicit-branch-never-fires', 's',
     "    if scope_source != 'explicit':\n",
     "    if True:\n",
     (T698,), 'KILLED'),

    ('the-explicit-branch-always-fires', 's',
     "    if scope_source != 'explicit':\n",
     "    if False:\n",
     (T698, TPS), 'KILLED'),

    ('accept-anything-that-is-safe', 's',
     "    accepted = bool(witness_ok and safe and not risen "
     "and fired is not None)\n",
     "    accepted = bool(witness_ok and safe and not risen)\n",
     (T698,), 'KILLED'),

    ('drop-the-safety-half', 's',
     "    accepted = bool(witness_ok and safe and not risen "
     "and fired is not None)\n",
     "    accepted = bool(witness_ok and not risen and fired is not None)\n",
     (T698,), 'KILLED'),

    ('drop-the-intent-licence', 's',
     "    accepted = bool(witness_ok and safe and not risen "
     "and fired is not None)\n",
     "    accepted = bool(witness_ok and safe and fired is not None)\n",
     (T698,), 'KILLED'),

    ('drop-the-witness-conjunct', 's',
     "    accepted = bool(witness_ok and safe and not risen "
     "and fired is not None)\n",
     "    accepted = bool(safe and not risen and fired is not None)\n",
     (T698,), 'KILLED'),

    # ---- the term-wise licence --------------------------------------------
    ('hpwl-is-hard-too', 's',
     "    for n in ('hole', 'oob', 'overlap'):\n",
     "    for n in ('hole', 'oob', 'overlap', 'hpwl'):\n",
     (T698,), 'KILLED'),

    ('oob-must-still-improve', 's',
     "    for n in ('hole', 'oob', 'overlap'):\n"
     "        # 1e-9, the SAME epsilon `eviction_licence_ok` uses. Two licences"
     " on\n"
     "        # one pass must not carry two tolerances, and `measure` rounds "
     "these\n"
     "        # to 4 decimals anyway.\n"
     "        if after[idx(n)] > before[idx(n)] + 1e-9:\n"
     "            rose.append(n)\n",
     "    for n in ('hole', 'oob', 'overlap'):\n"
     "        if after[idx(n)] >= before[idx(n)] - 1e-9:\n"
     "            rose.append(n)\n",
     (T698,), 'KILLED'),

    ('stacks-may-worsen', 's',
     "    for n in ('locked_contacts', 'pad_pairs', 'stacks'):\n",
     "    for n in ('locked_contacts', 'pad_pairs'):\n",
     (T698,), 'KILLED'),

    ('the-safety-half-never-objects', 's',
     "    return (not rose), rose\n",
     "    return True, rose\n",
     (T698,), 'KILLED'),

    # ---- the bases ---------------------------------------------------------
    ('the-intent-basis-is-dropped', 's',
     "    out['intent'] = intent_count\n",
     "    out['intent'] = 0\n",
     (T698,), 'KILLED'),

    ('scope_hpwl-is-the-whole-board', 's',
     "    return round(state.hpwl(nets), 3)\n",
     "    return round(state.hpwl(), 3)\n",
     (T698,), 'KILLED'),

    ('hpwl-becomes-a-basis', 's',
     "RESEAT_BASES = ('locked_contacts', 'pad_pairs', 'hole', 'oob', "
     "'intent',\n"
     "                'stacks', 'overlap', 'scope_hpwl')\n",
     "RESEAT_BASES = ('locked_contacts', 'pad_pairs', 'hole', 'oob', "
     "'intent',\n"
     "                'stacks', 'overlap', 'scope_hpwl', 'hpwl')\n",
     (T698,), 'KILLED'),

    # EXPECTED SURVIVOR, recorded rather than deleted. Every count basis is an
    # integer, so `> 0` and `>= 1` are the same predicate; the row is here
    # because `>= 1` is the form that stays correct if a basis ever becomes a
    # float, and it change-detects that. It is NOT evidence of a test hole.
    ('a-count-basis-fires-on-any-gain', 's',
     "        if units == 'count':\n"
     "            ok = gain >= 1\n",
     "        if units == 'count':\n"
     "            ok = gain > 0\n",
     (T698,), 'SURVIVED'),

    # ---- min_gain ----------------------------------------------------------
    ('min_gain-is-ignored', 's',
     "            ok = gain > max(abs(float(min_gain)), 1e-9)\n",
     "            ok = gain > 1e-9\n",
     (T698,), 'KILLED'),

    ('min_gain-gates-the-count-bases-too', 's',
     "        if units == 'count':\n"
     "            ok = gain >= 1\n",
     "        if units == 'count':\n"
     "            ok = gain >= max(1, float(min_gain))\n",
     (T698,), 'KILLED'),

    # ---- the intent licence (the VECTOR, not the count) --------------------
    ('the-licence-sums-the-vector', 'q',
     "            for t, b, a in zip(self.spec[ref], bv, av):\n"
     "                if a > b + legality.EPS:\n"
     "                    risen.append((ref, t.rule, t.name, b, a))\n",
     "            if sum(av) > sum(bv) + legality.EPS:\n"
     "                risen.append((ref, 'sum', 'summed', sum(bv), sum(av)))\n",
     (T698,), 'KILLED'),

    ('the-licence-never-objects', 'q',
     "        return (not risen), risen\n",
     "        return True, risen\n",
     (T698,), 'KILLED'),

    ('a-breach-is-at-or-above-threshold', 'q',
     "                if v > t.threshold:\n"
     "                    count += 1\n",
     "                if v >= t.threshold:\n"
     "                    count += 1\n",
     (T698,), 'KILLED'),

    # ---- the prune probe ---------------------------------------------------
    ('prune-ignores-the-probe', 'r',
     "        undoes_intent = any(a > b + 1e-9\n"
     "                            for a, b in zip(after_intent, base_intent))\n",
     "        undoes_intent = False\n",
     (T698,), 'KILLED'),

    ('prune-refuses-every-revert', 'r',
     "        undoes_intent = any(a > b + 1e-9\n"
     "                            for a, b in zip(after_intent, base_intent))\n",
     "        undoes_intent = bool(after_intent)\n",
     (T698, T630), 'KILLED'),

    ('the-probe-is-never-handed-over', 's',
     "                                     intent_probe=(probe.terms if probe\n"
     "                                                   is not None "
     "else None))\n",
     "                                     intent_probe=None)\n",
     (T698,), 'KILLED'),

    ('prune-samples-the-probe-after-twice', 'r',
     "        base_intent = intent_probe(ref) if intent_probe is not None "
     "else ()\n",
     "        base_intent = ()\n",
     (T698,), 'KILLED'),

    # ---- the probe must not ARM the seat gate ------------------------------
    ('the-probe-arms-the-state', 'q',
     "        self.refs: Tuple[str, ...] = tuple(rs)\n",
     "        self.refs: Tuple[str, ...] = tuple(rs)\n"
     "        state._intent_spec = dict(self.spec)\n"
     "        state._intent_active = True\n",
     (T698,), 'KILLED'),

    # ---- the extraction was supposed to be behaviour-free ------------------
    # The first version of this row mutated `not any(` to `False or not any(`,
    # which is the SAME expression -- a row that cannot fail, recorded as an
    # expected survivor. It looked like a finding and was a tautology.
    ('the-zone-spec-forgets-the-anchor-branch', 'q',
     "                _anchor = not any(\n"
     "                    _fp.zone_fits_courtyard(\n"
     "                        _z['rect'], _p.rect(0.0, 0.0, _r), _tol)\n"
     "                    for _r in (_p.rot % 360, (_p.rot + 90) % 360))\n",
     "                _anchor = False\n",
     (T702,), 'KILLED'),

    ('the-zone-spec-drops-the-exclusive-branch', 'q',
     "            elif _z['exclusive'] and (not _z['side']\n",
     "            elif False and _z['exclusive'] and (not _z['side']\n",
     (T702,), 'KILLED'),

    # EXPECTED SURVIVOR, recorded rather than deleted, and the reason matters:
    # `refs` is a work-SAVING pre-filter, not a correctness boundary.
    # `IntentProbe.__init__` walks the same ref set again when it builds
    # `self.spec`, so a wider `build_zone_spec` produces terms that are then
    # discarded. Removing the parameter would be a defensible simplification;
    # what would NOT be defensible is reading this survivor as a test hole and
    # writing an arm that asserts the pre-filter through the probe, which
    # cannot see it.
    ('build_zone_spec-ignores-its-refs-filter', 'q',
     "    items = (parts.items() if refs is None\n"
     "             else ((r, parts[r]) for r in refs if r in parts))\n",
     "    items = parts.items()\n",
     (T698, T702), 'SURVIVED'),

    ('hpwl-ignores-its-net-subset', 'q',
     "        items = (self.net_refs.items() if nets is None else\n"
     "                 [(n, self.net_refs[n]) for n in sorted(nets)\n"
     "                  if n in self.net_refs])\n",
     "        items = self.net_refs.items()\n",
     (T698,), 'KILLED'),

    # ---- the pre-push review's findings ------------------------------------
    ('the-auto-branch-outranks-the-eviction-licence', 's',
     "    if basis.get('eviction_licence') is False:\n"
     "        return (f\"{head}: the eviction licence -- moving parts outside "
     "the \"\n"
     "                f\"scope raised the stack count or the overlap area. See "
     "the \"\n"
     "                f\"note above for the figures.\")\n",
     "",
     (T698,), 'KILLED'),

    ('the-empty-basis-goes-back-to-a-literal', 's',
     "                'accept_basis': basis_skeleton(\n"
     "                    scope_source, policy='empty',\n"
     "                    witnesses_before=witnesses_before,\n"
     "                    witnesses_after=witnesses_before,\n"
     "                    hpwl_before=_empty_gate[_recon.GATE_TERMS.index("
     "'hpwl')],\n"
     "                    hpwl_after=_empty_gate[_recon.GATE_TERMS.index("
     "'hpwl')]),\n",
     "                'accept_basis': {'scope_source': scope_source,\n"
     "                                 'policy': 'empty', 'fired': None,\n"
     "                                 'terms': [], 'safety': None,\n"
     "                                 'intent_licence': None},\n",
     (T698,), 'KILLED'),

    ('min_gain-refuses-a-gain-of-exactly-min_gain', 's',
     "            ok = gain >= float(min_gain) - 1e-9\n",
     "            ok = gain > float(min_gain)\n",
     (T698,), 'KILLED'),

    ('a-continuous-basis-fires-on-rounding-noise', 's',
     "            ok = gain > MEASURE_QUANTUM\n",
     "            ok = gain > 1e-9\n",
     (T698,), 'KILLED'),

    ('the-KEPT-note-credits-the-probe-for-every-held-revert', 'r',
     "            if undoes_intent and wanted:\n",
     "            if undoes_intent:\n",
     (T698,), 'KILLED'),

    ('the-probe-goes-back-to-the-named-scope-only', 's',
     "        probe = _q.IntentProbe(state, zones=_bundle['zones'])\n",
     "        probe = _q.IntentProbe(state, zones=_bundle['zones'], "
     "refs=scope)\n",
     (T698,), 'KILLED'),

    # ---- reporting ---------------------------------------------------------
    ('accept_basis-never-names-the-winner', 's',
     "    basis['fired'] = fired if accepted else None\n",
     "    basis['fired'] = None\n",
     (T698,), 'KILLED'),

    ('the-early-out-drops-accept_basis', 's',
     "                'accept_basis': {'scope_source': scope_source,\n"
     "                                 'policy': 'empty', 'fired': None,\n"
     "                                 'terms': [], 'safety': None,\n"
     "                                 'intent_licence': None},\n",
     "",
     (T698,), 'KILLED'),

    ('the-auto-path-reports-a-safety-half-it-never-measured', 's',
     "        'safety': None, 'intent_licence': None,\n",
     "        'safety': {'ok': True, 'worsened': []},\n"
     "        'intent_licence': {'ok': True, 'risen': []},\n",
     (T698,), 'KILLED'),

    ('the-refusal-note-goes-back-to-blaming-oob', 's',
     "        notes.append(reseat_refusal_note(len(scope), accept_basis))\n",
     "        notes.append(f\"REVERTED: re-seating {len(scope)} part(s) did \"\n"
     "                     f\"not strictly improve the off-board amount\")\n",
     (T698,), 'KILLED'),
]


def _git_clean(paths):
    r = subprocess.run(['git', 'diff', '--quiet', '--'] + list(paths),
                       cwd=_ROOT)
    return r.returncode == 0


def _run(tests):
    for t in tests:
        r = subprocess.run([sys.executable, '-X', 'utf8', t],
                           cwd=_ROOT, capture_output=True, text=True,
                           encoding='utf-8', errors='replace')
        if r.returncode != 0:
            return True, f"{os.path.basename(t)} exit {r.returncode}"
    return False, "all named tests passed"


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--row', action='append', default=None)
    ap.add_argument('--list', action='store_true')
    a = ap.parse_args()

    if a.list:
        for name, tgt, _o, _n, tests, exp in ROWS:
            print(f"  {exp:9} {name}  [{tgt}] "
                  f"-> {', '.join(os.path.basename(t) for t in tests)}")
        return 0

    rows = ROWS
    if a.row:
        unknown = [n for n in a.row if n not in {r[0] for r in ROWS}]
        if unknown:
            print(f"no such row: {', '.join(unknown)}; try --list",
                  file=sys.stderr)
            return 2
        rows = [r for r in ROWS if r[0] in set(a.row)]

    if not _git_clean(TARGETS.values()):
        print("REFUSED: the engine files are dirty. Restoring would write the "
              "COMMITTED text back over uncommitted work.", file=sys.stderr)
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
            io.open(TARGETS[tgt], 'w', encoding='utf-8', newline='').write(
                src.replace(old, new, 1))
            killed, why = _run(tests)
            io.open(TARGETS[tgt], 'w', encoding='utf-8',
                    newline='').write(src)
            got = 'KILLED' if killed else 'SURVIVED'
            mark = 'ok' if got == expect else 'WRONG'
            verdicts.append((name, got, why))
            print(f"  {got:9}{'' if mark == 'ok' else ' WRONG'} {name} -- {why}")
    finally:
        for k, p in TARGETS.items():
            io.open(p, 'w', encoding='utf-8', newline='').write(originals[k])

    wrong = [v for v, (name, got, _w) in zip(rows, verdicts)
             if got != v[5]]
    broken = [n for n, g, _w in verdicts if g == 'BROKEN']
    print(f"\n{len(verdicts)} row(s): "
          f"{sum(1 for _n, g, _w in verdicts if g == 'KILLED')} killed, "
          f"{sum(1 for _n, g, _w in verdicts if g == 'SURVIVED')} survived, "
          f"{len(broken)} broken, {len(wrong)} disagreeing with expectation")
    return 1 if (wrong or broken) else 0


if __name__ == '__main__':
    sys.exit(main())
