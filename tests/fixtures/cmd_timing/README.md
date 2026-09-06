# `cmd_timing` fixtures (#887)

Two small ledgers in `tests/stress/tee_cmd.py`'s row format, standing in for the
real thing. They exist because **`wk/` is gitignored** — `git ls-files | grep -c
'^wk/'` returns 0 — so the 153-row, 104 KB run-24 ledger that
`py_router/cmd_timing.py` was written against cannot be committed, and the
regression cannot depend on it.

`tests/test_887_cmd_timing_reader.py` reads only these files and **carries the
whole regression on a fresh clone**. `tests/test_887_run24_regression.py`
corroborates the same reader against the real ledger when a machine happens to
have one, and self-skips (exit 77) when it does not.

Every row's `t_start` is `1787200000.0 + <offset>`; `iso_start` is that instant
in UTC. Nothing reads the absolute epoch — only the differences matter.

## `synthetic_run.jsonl` — 20 rows, one per rule

Each row is here for a named reason. **If you delete or change one, the test it
serves stops testing anything**, so this table is the map.

| # | label | wall s | exit | bucket | why this row exists |
|---:|---|---:|---:|---|---|
| 1 | `fence-audit-start` | 0.5 | 0 | close-out | Ran **first** and buckets **last**. Deliberately row 1 so a bucketer keyed on position ("everything after the last route step", which is how the original mandate worded it) fails too, not only a prefix bug. |
| 2 | `staging-assembly0` | 1.0 | **4** | staging | A non-zero exit that is still counted — run 24's own start anchor exited 4 and is row 1 of the published table. |
| 3 | `P0-driver` | 0.5 | 0 | P* | An ordinary placement step. |
| 4 | `P3-reconstruct` | 4.0 | 0 | P* | Names `r1.kicad_pcb` in `argv`, for the board→command mapping. |
| 5 | `Pclose-q-render` | 1.5 | 0 | **P\*** | `Pclose-*` counts under `P*`, because `close` is not a prefix **of** `Pclose-*`. |
| 6 | `Pclose-film` | 2.0 | 0 | **P\*** | A second one, so the count is not 1-vs-0. |
| 7 | `L2-driver` | 0.5 | **4** | L* | Second non-zero exit. |
| 8 | `L5-final-record` | 0.25 | **2** | L* | Repeated label, 1st entry. |
| 9 | `L5-final-record` | 0.25 | 0 | L* | Repeated label, 2nd — "counted once per entry" means two steps, not one. |
| 10 | `R1-pour` | 1.5 | 0 | R* | Names `r1_pour.kicad_pcb`. |
| 11 | `R3-route` | **60.0** | 0 | R* | Longest step. Names `r3_route.kicad_pcb`. |
| 12-14 | `R5-prune` ×3 | 0.5 each | 2, 0, 0 | R* | A label repeated **three** times, one of them failing. |
| 15 | `R7-layercosts` | **24.0** | 0 | R* | 2nd longest. |
| 16 | `V-complete` | **8.2** | 0 | V* | 3rd longest. |
| 17 | `V-drc` | 0.5 | 0 | V* | |
| 18 | `close-drc-final` | 0.5 | 0 | close-out | The `close*` prefix, distinct from `fence*`. |
| 19 | **`route4`** | 1.0 | 0 | **other** | `tests/stress/RUNBOOK.md`'s own worked example label. Lowercase, so it is **not** `R*` — this is what makes the `other` bucket real and the case-sensitivity decision visible. |
| 20 | `fence-audit-end` | 2.0 | 0 | close-out | |

Hand-computed expectations, asserted as literals in the test:

```
staging    1 /   1.0      R*         6 /  87.0
P*         4 /   8.0      V*         2 /   8.7
L*         3 /   1.0      close-out  3 /   3.0
                          other      1 /   1.0
n = 20   tool 109.7 s   run 412.0 s   outside 302.3 s
3 longest: R3-route 60.0, R7-layercosts 24.0, V-complete 8.2
```

Gaps between every `t_end` and the next `t_start` make "time outside the tools"
**302.3 s** by construction. No total lands on an exact `.5`, so no assertion is
hostage to a rounding tie-break.

## `out_of_order.jsonl` — 4 rows, the defensive pathologies

Kept **separate** so the file above stays an honest picture of what a serial
`tee_cmd` ledger actually looks like (disjoint rows, already in order), while
the defensive code is still pinned.

| label | t_start | t_end | why |
|---|---:|---:|---|
| `B-second` | +100 | +105 | Written **second** in the file though it is not first in time. |
| `D-longest` | +300 | **+700** | Starts before `E-last` and **outlives** it, so `max(t_end)` is not the last row's `t_end`. |
| `A-first` | +0 | +1 | Written **third**, but is first in time — so `load_rows` must sort. |
| `E-last` | +600 | +602 | Last by `t_start`, **not** last by `t_end`. |

Span is therefore **700 s**, not 602. These rows also overlap, which is why the
disjointness assertion lives in the run-24 arm and not here.

## Extending them

Append a line and add its expectation to `WANT_SUBTOTALS` (and a row above).
Note that `argv`/`cmdline` are only meaningful on the four rows that name a
board; the rest carry a plausible command so the shape is right, and no
assertion reads them.
