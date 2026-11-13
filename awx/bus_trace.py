"""bus_trace.py GROUP_OUT [ROUTE_WORK ..] [--lane L ..] [--json OUT.json] -- READ what a run saved (run_trace): the
group model's every step (GROUP_OUT/checks.jsonl, group_ends.py) and, given a bus step's work directory, what its
feedback credited (run/*.trace.jsonl, whole_feedback.py). Read-only.

Prints, in order:
  RUN        which code ran (run.json): its commit and the hash of its uncommitted change
  CHECKS     a line a check: how the model got its state (a repair, or the start and the search), its top tiers
             (walled balls, legs changing layer, plane-layer runs), the rest by family in vias, the ends that moved
             since the state before, the lanes the repair freed and why, and the check's verdict -- its status, the
             lanes and rules it named, and the field's DRC when laid
  LANES      the lanes a check named, most often first: how often named, by which rules, how often their ends moved
  REASONS    every banned via-end layer the checks met, with the copper behind it (vban_why)
  LANE L     with --lane: that lane check by check -- its ends (layer, face, surface or via, the layers a via end may
             run on), its change, the lanes it crosses, what it paid, whether the check named it and why, what the
             lessons held it to, and each finding a route credited to it
--json OUT.json writes the per-lane table."""
import collections
import json
import os
import sys

args = [a for a in sys.argv[1:] if not a.startswith('--')]
opt = {}
it = iter(sys.argv[1:])
for a in it:
    if a in ('--lane', '--json'):
        opt.setdefault(a, []).append(next(it))
group, routes = args[0], args[1:]
want = set(x for x in opt.get('--lane', []) if x not in routes)
routes = [r for r in routes if r not in want]


def jl(path):
    return [json.loads(ln) for ln in open(path, encoding='utf-8')] if os.path.isfile(path) else []


R = jl(os.path.join(group, 'checks.jsonl'))
if not R:
    raise SystemExit(f'bus_trace: no {os.path.join(group, "checks.jsonl")} (a run before run_trace saved none)')
run = json.load(open(os.path.join(group, 'run.json'))) if os.path.isfile(os.path.join(group, 'run.json')) else {}
print(f'RUN  {group}: head {str(run.get("head"))[:10]}, uncommitted {run.get("diff_lines")} line(s) of diff '
      f'(sha1 {str(run.get("diff_sha1"))[:10]}), {run.get("host")}, started {run.get("start")}')
fr = next((r for r in R if r['rec'] == 'from'), None)
if fr:
    print(f'FROM {fr["run"]}: feedback {fr["feedback"]}, ruled out {fr["hard"]}, lanes freed by it {fr["fed"]}')
for r in R:
    if r['rec'] == 'feedback':
        print(f'FEEDBACK check {r["check"]} {r["bus"]}: ruled out ' + ', '.join(
            f'{k} ({len(v)} option(s))' for k, v in r['ruled_out'].items()) + f'; {r["ruled_pairs"]} option pair(s); '
            f'priced {sum(v["options"] for v in r["priced"].values())} option(s) at {len(r["priced"])} end(s)')

# ---- CHECKS
print('\nCHECKS')
by_check = collections.defaultdict(list)
for r in R:
    if 'check' in r:
        by_check[r['check']].append(r)
named = collections.Counter()
rules_of = collections.defaultdict(collections.Counter)
moved_n = collections.Counter()
why_ban = collections.Counter()
lane_rows = collections.defaultdict(list)
for c in sorted(by_check):
    for r in by_check[c]:
        if r['rec'] == 'model':
            o = r['objective']
            rest = ', '.join(f'{k} {v:.1f}' for k, v in sorted(o['rest_vias'].items(), key=lambda kv: -abs(kv[1]))
                             if abs(v) >= 0.05)
            free = r.get('free') or {}
            fw = collections.Counter(w.split(' ')[0].split(':')[0] for v in free.values() for w in v.values())
            print(f'  check {c} model ({r["how"]}, {r["status"]}, {r["secs"]:.0f} s): walled {o["walls"]}, fanned '
                  f'{o["fans"]}, legs changing {o["changes"]}, plane runs {o["plane_runs_vias"]:.0f} vias; rest: {rest}'
                  + (f'; the repair freed {sum(len(v) for v in free.values())} lane(s): '
                     + ', '.join(f'{n} {k}' for k, n in sorted(fw.items())) if free else ''))
        elif r['rec'] == 'state':
            mv = r.get('moved') or {}
            for l_ in mv:
                moved_n[l_] += 1
            print(f'  check {c} state {r["bus"]}: {r["crossings"]} crossings, {r["changes"]} changing, {r["vias"]} fanout '
                  f'vias, cut {r["cut"]}; {r["n_moved"]} lane(s) moved' + (': ' + ', '.join(
                      f'{l_}({"+".join(e[0] for e in v)})' for l_, v in sorted(mv.items())) if mv else ''))
            for l_, x in r['lanes'].items():
                lane_rows[l_].append(('state', c, x, l_ in mv))
        elif r['rec'] == 'verdict':
            v = r['verdict'] or {}
            if v.get('plan'):
                print(f'  check {c} verdict {r["bus"]} ({r["mode"]}): a plan ({r["secs"]:.0f} s)')
                continue
            for l_ in v.get('lanes') or ():
                named[l_] += 1
            for l_, f_ in v.get('rules') or ():
                rules_of[l_][f_] += 1
            for l_, k_, L_, w_ in v.get('vban_why') or ():
                if l_ in (v.get('lanes') or ()):
                    why_ban[(w_.split(' beside ')[0] if ' beside ' in w_ else w_, w_)] += 1
            print(f'  check {c} verdict {r["bus"]} ({r["mode"]}): NO plan [{v.get("status")}] '
                  f'({r["secs"]:.0f} s) -- lanes {v.get("lanes")}, core {v.get("core")}, rules '
                  + ', '.join(f'{l_} {f_}' for l_, f_ in v.get('rules') or ()) + f'; no room for a change '
                  f'{v.get("nochange")}; log {r.get("solve_log")}')
            for l_ in v.get('lanes') or ():
                lane_rows[l_].append(('verdict', c, v, False))
        elif r['rec'] == 'lessons':
            print(f'  check {c} lessons {r["bus"]}: new ' + ', '.join(
                f'{k} {len(v) if hasattr(v, "__len__") else v}' for k, v in r['new'].items())
                + f'; the repair to free {len(r["free_next"])}: {", ".join(r["free_next"])}')
        elif r['rec'] == 'field':
            print(f'  check {c} field: DRC {r["drc_field"]} (bare {r["drc_bare"]}), {r["lessons"]} lesson(s), '
                  f'{len(r.get("moved") or [])} part(s) moved')
done = next((r for r in R if r['rec'] == 'done'), None)
if done:
    print(f'  DONE after {done["checks"]} check(s), {done["secs"]:.0f} s' + (f', NO PLAN for {done["no_plan"]}'
                                                                               if done['no_plan'] else ''))

# ---- LANES the checks named
print('\nLANES named by the checks (most first): named / rules / ends moved')
for l_, n in named.most_common():
    print(f'  {l_:16s} {n} / ' + ', '.join(f'{f} {k}' for f, k in rules_of[l_].most_common()) + f' / {moved_n[l_]}')
if why_ban:
    print('\nREASONS a named lane\'s via end was kept off a layer (the copper behind each ban)')
    for (kind, w_), n in why_ban.most_common(30):
        print(f'  {n:3d}  {w_}')

# ---- the route's feedback traces
for wd in routes:
    for root, _d, files in os.walk(wd):
        for f in sorted(files):
            if f.endswith('.trace.jsonl'):
                T = jl(os.path.join(root, f))
                print(f'\nFEEDBACK TRACE {os.path.relpath(os.path.join(root, f), wd)}: {len(T)} record(s)')
                for t in T:
                    if t['rec'] == 'finding' and (not want or set(t['lanes']) & want):
                        cr = '; '.join(f'{x["as"]} ({x["act"]}): ' + ' + '.join(
                            f'{e["lane"]} {e["end"]} {e["layer"]} {e["dist_mm"]} mm'
                            for e in x['ends']) for x in t['credited']) or 'nothing'
                        print(f'  {t["kind"]:6s} {"/".join(t["lanes"]):24s} near {t["near"]} -> {cr}')
                    elif t['rec'] == 'name':
                        print(f'  NAMED ({t["why"]}): {t["lanes"]}')

# ---- LANE L
for l_ in sorted(want):
    print(f'\nLANE {l_}')
    for kind, c, x, mv in lane_rows.get(l_, ()):
        if kind == 'state':
            e = lambda k: (f'{x[k]["layer"]} {x[k]["dir"]} {"via" if x[k]["via"] else "surface"}'   # noqa: E731
                           + (f' runs {"/".join(x[k]["runs_on"])}' if x[k]['runs_on'] else '')
                           + f' at ({x[k]["xy"][0][0]:.2f}, {x[k]["xy"][0][1]:.2f})')
            print(f'  check {c} {"MOVED " if mv else ""}tooth {e("tooth")}; berth {e("berth")}; change {x["change"]}; '
                  f'crosses {", ".join(x["crosses"]) or "none"}')
        else:
            rs = [f for ll, f in x.get('rules') or () if ll == l_]
            vb = [f'{("tooth", "berth")[k]} {L}: {w}' for ll, k, L, w in x.get('vban_why') or () if ll == l_]
            print(f'  check {c} NAMED: rules {rs}' + ('; no room for a change' if l_ in (x.get('nochange') or ()) else '')
                  + (f'; core with {[p for p in x.get("core") or () if l_ in p]}' if any(l_ in p for p in x.get('core') or ()) else '')
                  + (f'; banned: {"; ".join(vb)}' if vb else ''))
    for r in R:
        if r['rec'] == 'model':
            p = ((r['objective'].get('lanes') or {}).get(next(iter(r['objective'].get('lanes') or {'': 0}), '')) or {}).get(l_)
            fw = [w for v in (r.get('free') or {}).values() for ll, w in v.items() if ll == l_]
            if p or fw:
                print(f'  check {r["check"]} paid {p or {}}' + (f'; freed in the repair: {fw[0]}' if fw else ''))
        elif r['rec'] == 'lessons':
            hit = {k: [x for x in (v if isinstance(v, list) else list(v)) if l_ in json.dumps(x)]
                   for k, v in r['new'].items() if isinstance(v, (list, dict))}
            hit = {k: v for k, v in hit.items() if v}
            if hit:
                print(f'  check {r["check"]} taught: {hit}')

if opt.get('--json'):
    json.dump({l_: [{'what': k, 'check': c, 'moved': mv, **({'ends': x} if k == 'state' else {})}
                    for k, c, x, mv in rows] for l_, rows in lane_rows.items()},
              open(opt['--json'][0], 'w'), indent=1)
    print(f'\nwrote {opt["--json"][0]}')
