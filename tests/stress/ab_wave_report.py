#!/usr/bin/env python3
"""Aggregate release report: one candidate wave vs one or more baseline waves.

`ab_replay_grade.py --compare OLD NEW` diffs ONE set's `summary.json` against
another and prints a per-board table. A release decision needs the roll-up: every
set at once, against more than one baseline arm, with the regressions ranked. That
is this tool.

    python3 tests/stress/ab_wave_report.py \
        --new  ~/Documents/kicad_stress_test/ab_main_0728a \
        --base 0726a=~/Documents/kicad_stress_test/ab_main_0726a \
        --base dp250=~/Documents/kicad_stress_test/ab_dp250 \
        --sets 1,2,3,4,5,6,7,8,9,10,11

`--base` is repeatable and takes `NAME=WAVEDIR`. A baseline that only covers some
sets (ab_dp250 is sets 6-11) simply reports on the sets it has -- the per-set
table shows which. Use `--top` to lengthen the per-board lists, `--all` for the
full listing.

===============================================================================
GRADING SEMANTICS -- identical to ab_replay_grade.compare(); do not diverge
===============================================================================
  * drc_real   -- ROUTER-ATTRIBUTABLE DRC: raw check_drc minus the #405 unrouted-
                  input baseline and the #408 accepted-by-design edge items. Never
                  grade on raw `drc`: it counts pre-existing input copper (edge
                  connector pads, chassis-ground pours) the router never touched.
                  Falls back to raw drc only for pre-#408 summaries.
  * nets_incomplete -- unrouted + connectivity-issue nets. Never grade on `conn`
                  alone: a net that loses its copper ENTIRELY leaves the conn
                  bucket for the unrouted bucket, so conn can DROP while the board
                  got strictly worse.
  * kicad_connection_width -- #406 min copper web (graded when a floor is recorded).
  * diff_pairs_coupled -- fewer coupled pairs = quality regression (a pair that
                  falls back to single-ended still "routes").
Sign convention: drc / incomplete / connection_width NEGATIVE = better;
coupled diff-pairs POSITIVE = better.

A board is compared only when its chain is COMPLETE IN BOTH arms -- a broken
chain has no final board to grade, and silently scoring it 0 would read as a
perfect board. Chains that broke or newly completed are reported separately,
because a chain that stops completing is a release blocker that never shows up
as a DRC delta.

IMPORTANT: both arms must be graded by the SAME checker build. Every grader here
is under active development, so a stored `summary.json` is a snapshot of code
that no longer exists -- re-grade the baseline first
(`ab_wave_driver.py regrade --out <baseline_wave>`) or the report will attribute
grader drift to the engine change.
"""
import argparse
import json
from pathlib import Path


def load(wave, sets):
    """{set: {board: row}} for one wave dir."""
    out = {}
    for s in sets:
        f = Path(wave).expanduser() / f"set{s}" / "summary.json"
        if f.exists():
            out[s] = {r["board"]: r for r in json.loads(f.read_text())}
    return out


def drc(r):
    v = r.get("drc_real")
    return v if v is not None else r.get("drc")


def incompl(r):
    v = r.get("nets_incomplete")
    return v if v is not None else r.get("conn")


def cw(r):
    return r.get("kicad_connection_width")


def dpc(r):
    return r.get("diff_pairs_coupled")


def delta(o, n, fn):
    """Paired delta; a metric missing on either side contributes nothing."""
    a, b = fn(o), fn(n)
    return (b - a) if (a is not None and b is not None) else 0


def is_regression(dd, di, dc, dp):
    return dd > 0 or di > 0 or dc > 0 or dp < 0


def arm_report(name, base, new, sets, top):
    print("=" * 104)
    print(f"CANDIDATE vs {name}")
    print("=" * 104)
    tot = {"drc": 0, "inc": 0, "cw": 0, "dp": 0}
    per_set, rows = [], []
    broke, fixed, missing = [], [], []
    for s in sets:
        b_set, n_set = base.get(s, {}), new.get(s, {})
        if not b_set or not n_set:
            continue
        d = {"drc": 0, "inc": 0, "cw": 0, "dp": 0}
        n_cmp = 0
        for board in sorted(set(b_set) & set(n_set)):
            o, nn = b_set[board], n_set[board]
            oc, nc = o.get("chain_complete"), nn.get("chain_complete")
            if not (oc and nc):
                if oc and not nc:
                    broke.append(f"set{s}/{board}")
                elif nc and not oc:
                    fixed.append(f"set{s}/{board}")
                continue
            n_cmp += 1
            dd, di = delta(o, nn, drc), delta(o, nn, incompl)
            dc, dp = delta(o, nn, cw), delta(o, nn, dpc)
            for k, v in (("drc", dd), ("inc", di), ("cw", dc), ("dp", dp)):
                d[k] += v
                tot[k] += v
            if dd or di or dc or dp:
                # rank by impact: an incomplete net matters more than a DRC item
                rows.append((abs(dd) + abs(di) * 3 + abs(dc) + abs(dp), s, board,
                             dd, di, dc, dp, drc(o), drc(nn), incompl(o), incompl(nn)))
        missing += [f"set{s}/{b}" for b in sorted(set(b_set) - set(n_set))]
        per_set.append((s, n_cmp, d))

    print(f"{'set':>6} {'boards':>7} {'drc_real':>10} {'incomplete':>12} "
          f"{'conn_width':>11} {'dpair':>7}")
    print("-" * 60)
    for s, n_cmp, d in per_set:
        print(f"{'set'+s:>6} {n_cmp:>7} {d['drc']:>+10d} {d['inc']:>+12d} "
              f"{d['cw']:>+11d} {d['dp']:>+7d}")
    print("-" * 60)
    nb = sum(x[1] for x in per_set)
    print(f"{'TOTAL':>6} {nb:>7} {tot['drc']:>+10d} {tot['inc']:>+12d} "
          f"{tot['cw']:>+11d} {tot['dp']:>+7d}")
    verdict = "REGRESSION" if is_regression(tot["drc"], tot["inc"], tot["cw"], tot["dp"]) \
        else "no regression"
    print(f"\nVERDICT vs {name}: {verdict}   "
          f"(neg drc/incomplete/conn_width = better; pos dpair = better)")

    if broke:
        print(f"\n!! chain BROKE in candidate ({len(broke)}) -- RELEASE BLOCKER: {', '.join(broke)}")
    if fixed:
        print(f"\n++ chain NEWLY COMPLETE in candidate ({len(fixed)}): {', '.join(fixed)}")
    if missing:
        print(f"\n?? in baseline but not candidate ({len(missing)}): {', '.join(missing)}")

    rows.sort(reverse=True)
    regs = [r for r in rows if is_regression(r[3], r[4], r[5], r[6])]
    imps = [r for r in rows if not is_regression(r[3], r[4], r[5], r[6])]
    for title, rr in (("REGRESSED boards", regs), ("IMPROVED boards", imps)):
        print(f"\n{title} ({len(rr)}):")
        print(f"  {'set/board':34} {'drc_real':>17} {'incomplete':>17} {'cw':>7} {'dp':>5}")
        for _, s, b, dd, di, dc, dp, od, nd, oi, ni in (rr if top < 0 else rr[:top]):
            print(f"  {'set'+s+'/'+b:34} {str(od)+'->'+str(nd):>11} {dd:>+5d} "
                  f"{str(oi)+'->'+str(ni):>11} {di:>+5d} {dc:>+7d} {dp:>+5d}")
        if top >= 0 and len(rr) > top:
            print(f"  ... and {len(rr)-top} more (use --all)")
    return tot, nb


def main():
    ap = argparse.ArgumentParser(description="Roll a corpus wave up against baseline wave(s)")
    ap.add_argument("--new", required=True, help="candidate wave dir")
    ap.add_argument("--base", action="append", required=True, metavar="NAME=WAVEDIR",
                    help="baseline arm, repeatable")
    ap.add_argument("--sets", default="1,2,3,4,5,6,7,8,9,10,11")
    ap.add_argument("--top", type=int, default=25, help="per-board rows to list per section")
    ap.add_argument("--all", action="store_true", help="list every changed board")
    args = ap.parse_args()

    sets = [s.strip() for s in args.sets.split(",") if s.strip()]
    top = -1 if args.all else args.top
    new = load(args.new, sets)
    ncomplete = sum(1 for s in new.values() for r in s.values() if r.get("chain_complete"))
    ntot = sum(len(s) for s in new.values())
    print(f"CANDIDATE: {args.new}")
    print(f"  chains complete: {ncomplete}/{ntot}\n")
    for spec in args.base:
        name, _, path = spec.partition("=")
        arm_report(name or path, load(path or name, sets), new, sets, top)
        print()


if __name__ == "__main__":
    main()
