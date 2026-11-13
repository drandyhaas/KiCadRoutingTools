#!/usr/bin/env python3
"""A corpus replay leaves out its slowest boards, ranked by their newest replay.

The three slowest boards set a corpus A/B's wall clock, hours past every other
board, so `cloud_replay_sets.py` drops them by default (`--limit -5`; a
negative limit drops the |N| slowest, a positive one keeps the N cheapest).
"Slowest" is read from `sweep_lib.load_cost_table`, which used to hold only
each board's ORIGINAL stress run timed as a whole agent session: a board
that replays in minutes could rank above one that now takes an hour.

Rows:
  - the cost table takes a board's NEWEST harvested cloud replay row, not its
    slowest and not an older wave's;
  - a cloud row outranks an ab_* summary and the agent-session fallback, and
    an empty (half-harvested) row is skipped, not fatal;
  - drop_slowest drops the N slowest, never a board with no recorded cost,
    and nothing at N = 0;
  - cloud_replay_sets defaults --limit to -5 and hands it to modal_app as ONE
    token, and hands --exclude through as --exclude (an explicit --boards
    list would switch the default drop off).

    python3 tests/test_cloud_drop_slowest.py
"""
import json
import os
import re
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'tests' / 'stress' / 'modal_sweep'))
import sweep_lib as sl   # noqa: E402

FAILS = []


def check(name, cond, detail=''):
    print(('  PASS  ' if cond else '  FAIL  ') + name + (f'  [{detail}]' if detail else ''))
    if not cond:
        FAILS.append(name)


def row(path, board, seconds, mtime):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({'board': board, 'total_seconds': seconds}))
    os.utime(path, (mtime, mtime))


def main():
    now = time.time()
    with tempfile.TemporaryDirectory() as td:
        st = Path(td)
        # 'quick' was slow in an old wave and fast in the newest one
        row(st / 'cloud_old-kc_1/_raw/old-kc_1/set1__quick.json', 'quick', 9000, now - 900)
        row(st / 'cloud_new-kc_2/_raw/new-kc_2/set1__quick.json', 'quick', 100, now - 100)
        row(st / 'cloud_new-kc_2/_raw/new-kc_2/set1__slow.json', 'slow', 5000, now - 100)
        (st / 'cloud_new-kc_2/_raw/new-kc_2/set1__empty.json').write_text('')
        # an ab_* summary and an agent session that a cloud row must outrank
        ab = st / 'ab_x' / 'set1'
        ab.mkdir(parents=True)
        (ab / 'summary.json').write_text(json.dumps(
            [{'board': 'slow', 'total_seconds': 1}, {'board': 'abonly', 'total_seconds': 300}]))
        res = st / 'results_set1'
        res.mkdir()
        (res / 'slow.json').write_text(json.dumps({'board': 'slow', 'timing': {'tool_exec_s': 40000}}))
        (res / 'agent.json').write_text(json.dumps({'board': 'agent', 'timing': {'tool_exec_s': 2000}}))

        c = sl.load_cost_table(st)
        check("a board's newest cloud replay row sets its cost",
              (c.get('quick') or {}).get('seconds') == 100, f"{c.get('quick')}")
        check("a cloud row outranks the ab_* summary and the agent session",
              (c.get('slow') or {}).get('seconds') == 5000, f"{c.get('slow')}")
        check("ab_* and the agent session still fill boards with no cloud row",
              (c.get('abonly') or {}).get('seconds') == 300
              and (c.get('agent') or {}).get('seconds') == 2000)

        bl = [('set1', b) for b in ('quick', 'slow', 'abonly', 'agent', 'nocost')]
        if not hasattr(sl, 'drop_slowest'):
            check("sweep_lib has drop_slowest", False)
            print(f"\nFAIL: {len(FAILS)} row(s): {FAILS}")
            return 1
        kept, dropped = sl.drop_slowest(bl, c, 2)
        check("the two slowest are dropped, slowest first",
              [b for _s, _set, b in dropped] == ['slow', 'agent'], f"{dropped}")
        check("the rest are kept, a board with no cost among them",
              [b for _s, b in kept] == ['quick', 'abonly', 'nocost'], f"{kept}")
        kept, dropped = sl.drop_slowest([('set1', 'nocost')], c, 3)
        check("a board with no recorded cost is never dropped", kept == [('set1', 'nocost')])
        kept, dropped = sl.drop_slowest(bl, c, 0)
        check("N = 0 drops nothing", kept == bl and not dropped)

    src = (ROOT / 'tests' / 'stress' / 'cloud_replay_sets.py').read_text()
    check("cloud_replay_sets defaults --limit to -5",
          re.search(r'"--limit", type=int, default=-5', src) is not None)
    check("the limit reaches modal_app as one token",
          'cmd += [f"--limit={args.limit}"]' in src)
    check("--exclude reaches modal_app as --exclude, not as a --boards list",
          'cmd += ["--exclude", ",".join(sorted(drop))]' in src)
    app = (ROOT / 'tests' / 'stress' / 'modal_sweep' / 'modal_app.py').read_text()
    check("modal_app drops the slowest through sweep_lib, never with --boards",
          'elif limit < 0 and not boards:' in app and 'sl.drop_slowest(' in app)

    if FAILS:
        print(f"\nFAIL: {len(FAILS)} row(s): {FAILS}")
        return 1
    print("\nthe slowest boards are dropped by their newest replay time. OK")
    return 0


if __name__ == '__main__':
    sys.exit(main())
