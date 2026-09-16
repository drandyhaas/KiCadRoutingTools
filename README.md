# metrics-data

The reach-snapshot **archive** for KiCadRoutingTools. Data only — no code.

This branch is orphaned (it shares no history with `main`) and is written by
`.github/workflows/metrics.yml`, which appends one snapshot per run.

## Why it is not on `main`

GitHub's traffic API is a **rolling 14-day window that is never backfilled**, so
the snapshots have to accumulate somewhere durable or those days are lost for
everyone, permanently. Actions artifacts expire and the Actions cache is
evicted, so neither can hold it. It does have to be committed — it just does not
have to be committed to `main`, where a daily bookkeeping commit buries the
project's actual history.

Every snapshot taken before 2026-09-16 is still in `main`'s history; this branch
was seeded from the last of them.

## Files

| file | what it holds |
|---|---|
| `traffic_daily.json` | per-day views and clones, merged MAX per date |
| `releases.json`      | per-release asset download counts |
| `paths.json`         | popular paths, most recent snapshot |
| `referrers.json`     | referring sites, most recent snapshot |
| `meta.json`          | run timestamps and which endpoints failed |

`traffic_daily.json` merges by taking the **maximum** per date: a part-elapsed
day observed by one run must never be frozen at its partial value by a later
run, and a re-run inside the 14-day window must never reduce a banked day.

## Reading it

    git fetch origin metrics-data
    git show origin/metrics-data:traffic_daily.json

The rendered page is published to GitHub Pages from `main`'s workflow; nothing
here is served directly.
