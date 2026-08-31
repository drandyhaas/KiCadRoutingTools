#!/usr/bin/env python3
"""Mutation battery for #711 / #712 / #706.

Green tests are not evidence of coverage. Every row below breaks one thing the
three test files CLAIM to hold down; a row that survives is a hole in the
tests, and a row recorded as an expected survivor is a finding rather than a
convenience.

    python3 tests/mutate_711.py            # every row
    python3 tests/mutate_711.py --list
    python3 tests/mutate_711.py --row conjunct-never-fires

A row is KILLED by a FAILURE or by an ERROR. An anchor that does not match
EXACTLY ONCE is reported BROKEN, never skipped: a mutation that silently
edited nothing would be recorded as a surviving row, which is the opposite of
what it means.

Refuses to start on a dirty target tree, because it restores the ORIGINAL text
from disk and would write committed text over uncommitted work.

THE MEASURED TABLE GOES IN THE HEADER OF THE TEST FILE IT DEFENDS, FROM THE
RUN -- never predicted here and never edited afterwards to match.
"""
import argparse
import os
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

KILLED, SURVIVED, BROKEN = 'KILLED', 'SURVIVED', 'BROKEN'

TARGETS = {
    'fp': os.path.join(REPO, 'py_placer', 'placement', 'floorplan.py'),
    'sd': os.path.join(REPO, 'py_placer', 'placement', 'seeder.py'),
    'db': os.path.join(REPO, 'py_placer', 'placement', 'design_brief.py'),
    'cb': os.path.join(REPO, 'py_router', 'copy_board.py'),
}

T712 = 'tests/test_712_edge_centering.py'
T711 = 'tests/test_711_design_brief.py'
T706 = 'tests/test_706_seat_edge_target.py'
TSIB = 'tests/test_711_sibling_lists.py'
TSCH = 'tests/test_549_floorplan_schema.py'

#: (name, target, old, new, tests, expected)
ROWS = [
    # ---- #712: the grade -------------------------------------------------
    ('conjunct-never-fires', 'fp',
     "    yield from _grade_along_edge(ctx, c, ref, part, _sev)",
     "    return\n        yield from _grade_along_edge(ctx, c, ref, part, _sev)",
     (T712,), KILLED),
    ('measure-only-when-declared', 'fp',
     "    ctx.edge_seating.append(row)\n    if not declared:",
     "    if declared:\n        ctx.edge_seating.append(row)\n    if not declared:",
     (T712,), KILLED),
    ('centring-has-a-default-threshold', 'fp',
     "    declared = centre_claim is not None or band_claim is not None",
     "    declared = True\n    centre_claim = centre_claim or {'tolerance_mm': 1.0}",
     (T712,), KILLED),
    ('offset-loses-its-sign', 'fp',
     "    offset = centre - (lo + span / 2.0)",
     "    offset = abs(centre - (lo + span / 2.0))",
     (T712,), KILLED),
    ('no-outside_mm-for-the-repair-census', 'fp',
     "                          'outside_mm': round(abs(offset) - tol, 4)},",
     "                          },",
     (T712,), KILLED),

    # ---- #712: the span basis -------------------------------------------
    ('span-always-bbox', 'fp',
     "    rings = list(getattr(gate, 'rings', None) or [])\n    if rings:",
     "    rings = []\n    if rings:",
     (T712,), KILLED),
    ('abstain-never-fires', 'fp',
     "    if outline.get('simple_rectangle') and not outline.get('cutouts'):",
     "    if True:",
     (T712,), KILLED),
    ('split-runs-accepted-as-one', 'fp',
     "        if len(merged) != 1:",
     "        if False:",
     (T712,), KILLED),
    ('claim-with-no-edge-infers-one', 'fp',
     "    edge = c.get('edge')\n    if not edge:",
     "    edge = c.get('edge') or _nearest_edge(part.rect, ctx.outline_bounds)\n    if not edge:",
     (T712,), KILLED),

    # ---- #712: the schema ------------------------------------------------
    ('both-along-edge-forms-allowed', 'fp',
     "    if centre is not None and band is not None:",
     "    if False:",
     (TSCH, T711), KILLED),
    ('tolerance_mm-gets-a-default', 'fp',
     "        if 'tolerance_mm' not in centre:",
     "        centre.setdefault('tolerance_mm', 1.0)\n        if False:",
     (TSCH,), KILLED),
    ('inverted-band-accepted', 'fp',
     "        if not f0 < f1:",
     "        if False:",
     (TSCH, T711), KILLED),
    ('reader-version-not-bumped', 'fp',
     "READER_VERSION = 2",
     "READER_VERSION = 1",
     (TSCH,), KILLED),

    # ---- #711: the brief -------------------------------------------------
    ('board-brief-accepted-as-a-design-brief', 'db',
     "    if kind != KIND:",
     "    if False:",
     (T711,), KILLED),
    ('height-silently-accepted', 'db',
     "    for bad, why in sorted(_REFUSED_TOP_LEVEL.items()):",
     "    for bad, why in sorted({}.items()):",
     (T711,), KILLED),
    ('duplicate-interface-allowed', 'db',
     "        if ref in seen_refs:",
     "        if False:",
     (T711,), KILLED),
    ('unknown-edge-becomes-a-claim', 'db',
     "        elif edge == UNKNOWN:",
     "        elif edge == UNKNOWN and False:",
     (T711,), KILLED),
    ('unknown-and-absent-collapse', 'db',
     "    absent = [k for k in _TIER0",
     "    absent = [] or [k for k in ()",
     (T711,), KILLED),
    ('inferred-outranks-declared', 'db',
     "        base.update({k: v for k, v in c.items() if k != 'context'})",
     "        base.update({k: v for k, v in c.items()\n                     if k != 'context' and k not in base})",
     (T711,), KILLED),
    ('evidence-dropped-on-merge', 'db',
     "        base = by_ref.get(ref)\n        if base is None:",
     "        base = dict(c) if by_ref.get(ref) else None\n        if base is None:",
     (T711,), KILLED),
    ('absent-ref-silently-dropped', 'db',
     "            unmatched.append(ref)\n        conns.append(entry)",
     "            unmatched.append(ref)\n            continue\n        conns.append(entry)",
     (T711,), KILLED),
    ('fixed-becomes-must_lock', 'db',
     "    if wrote_along_edge:",
     "    fragment['must_lock'] = [f['ref'] for f in brief.fixed]\n    if wrote_along_edge:",
     (T711,), KILLED),
    ('min_reader-written-always', 'db',
     "    if wrote_along_edge:\n        # The first real use",
     "    if True:\n        # The first real use",
     (T711,), KILLED),
    ('a-corrupt-sibling-is-swallowed', 'db',
     "    except (OSError, ValueError) as exc:\n        raise BriefError(f\"cannot read design brief {path}: {exc}\") from None",
     "    except (OSError, ValueError):\n        return empty_brief()",
     (T711,), KILLED),

    # ---- #711: the sibling list -----------------------------------------
    ('brief-not-carried-by-siblings', 'cb',
     '                ".design-brief.json")',
     '                )',
     (TSIB, T711), KILLED),

    # ---- #706: the seat --------------------------------------------------
    ('seat-ignores-the-declaration', 'sd',
     "    if declared is not None:\n        cur = declared - _c_off",
     "    if False:\n        cur = declared - _c_off",
     (T706,), KILLED),
    ('declared-window-not-clamped', 'sd',
     "    win = _declared_frac_window(entry, span)\n    if win is not None:",
     "    win = _declared_frac_window(entry, span)\n    if False:",
     (T706,), KILLED),
    ('origin-centre-currency-error', 'sd',
     "    _c_off = _centre_offset_frac(part, state.board, edge)",
     "    _c_off = 0.0",
     (T706,), KILLED),
    ('ladder-not-scaled-to-the-window', 'sd',
     "    _step = ((f_hi - f_lo) / 0.8) if win is not None else 1.0",
     "    _step = 1.0",
     (T706,), KILLED),
    ('rotation-gate-open-to-undeclared', 'sd',
     "    if declared is not None and not _already_on_its_edge(state, part):",
     "    if not _already_on_its_edge(state, part):",
     (T706,), KILLED),
    ('rotation-guard-deleted', 'sd',
     "    if declared is not None and not _already_on_its_edge(state, part):",
     "    if declared is not None:",
     (T706,), KILLED),
    ('rotation-note-reads-the-new-angle', 'sd',
     "        was_rot = part.rot",
     "        was_rot = part.rot\n        _ = was_rot",
     (T706,), SURVIVED),   # see the note in the header this feeds
    ('stage-one-ignores-the-declaration', 'sd',
     "            _dec = _declared_frac(c)\n            frac = _dec if _dec is not None else (k + 1) / (len(specs) + 1)",
     "            _dec = None\n            frac = (k + 1) / (len(specs) + 1)",
     (T706,), KILLED),
]


def _dirty(paths):
    r = subprocess.run(['git', 'diff', '--quiet', '--'] + list(paths),
                       cwd=REPO)
    return r.returncode != 0


def _run(tests):
    for t in tests:
        r = subprocess.run([sys.executable, '-X', 'utf8', t], cwd=REPO,
                           capture_output=True, encoding='utf-8',
                           errors='replace', timeout=1800)
        if r.returncode != 0:
            return KILLED, f"{t} exit {r.returncode}"
    return SURVIVED, ''


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--row', action='append')
    ap.add_argument('--list', action='store_true')
    a = ap.parse_args(argv)

    rows = ROWS
    if a.row:
        want = set(a.row)
        rows = [r for r in ROWS if r[0] in want]
        missing = want - {r[0] for r in rows}
        if missing:
            print(f"unknown row(s): {sorted(missing)}", file=sys.stderr)
            return 2
    if a.list:
        for name, tgt, _, _, tests, exp in ROWS:
            print(f"  {name:42s} {tgt}  expect {exp}  ({', '.join(tests)})")
        return 0

    used = sorted({TARGETS[r[1]] for r in rows})
    if _dirty(used):
        print("REFUSING: the target tree is dirty. This restores the ORIGINAL "
              "text from disk and would write committed text over uncommitted "
              "work.", file=sys.stderr)
        return 2

    results = {}
    for name, tgt, old, new, tests, expected in rows:
        path = TARGETS[tgt]
        src = open(path, encoding='utf-8').read()
        n = src.count(old)
        if n != 1:
            results[name] = (BROKEN, f"anchor matched {n} times, expected 1")
            print(f"  {name:42s} BROKEN  (anchor matched {n} times)")
            continue
        try:
            open(path, 'w', encoding='utf-8').write(src.replace(old, new, 1))
            verdict, why = _run(tests)
        finally:
            open(path, 'w', encoding='utf-8').write(src)
        results[name] = (verdict, why)
        mark = 'ok ' if verdict == expected else 'DISAGREES'
        print(f"  {name:42s} {verdict:9s} {mark}  {why}")

    killed = sum(1 for v, _ in results.values() if v == KILLED)
    surv = sum(1 for v, _ in results.values() if v == SURVIVED)
    broken = sum(1 for v, _ in results.values() if v == BROKEN)
    disagree = [n for n, (v, _) in results.items()
                if v != dict((r[0], r[5]) for r in rows)[n]]
    print(f"\n{len(results)} rows: {killed} killed, {surv} survived, "
          f"{broken} broken, {len(disagree)} disagreeing with expectation")
    if disagree:
        print(f"  disagreeing: {sorted(disagree)}")
    return 1 if (broken or disagree) else 0


if __name__ == '__main__':
    sys.exit(main())
