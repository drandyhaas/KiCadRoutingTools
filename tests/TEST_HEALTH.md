# Test health

What was red, why, and what was done about it. Written so the next reader can
tell a genuinely-skipped test from an abandoned one — and can disagree with a
judgement call without re-deriving it.

**The rule every entry here was held to:** a test that stops failing must stop
failing because the thing it tests got fixed, or because the test was asserting
something untrue. Deleting it, loosening an assertion, or blanket-skipping a
class are all ways of making the suite lie.

---

## Prerequisites this suite needs

| what | why | if missing |
|---|---|---|
| `rust_router/grid_router.pyd` | every routing test imports it via `py_router/route.py` | `python3 build_router.py` — downloads the published binary, no cargo needed. **It is `.gitignore`d**, so a fresh clone has none and ~110 tests fail until it is built. This is the single biggest difference between a fresh clone and a working tree. |
| `pcbnew`, `wx` | `tests/gui_parity/` only | those gates cannot run; they are the legitimate `SKIP_EXIT` candidates. Nothing outside `gui_parity/` needs them. |
| `scipy`, `numpy`, `shapely`, `PIL`, `matplotlib` | assorted | present in a normal dev env |

Untracked artifacts also matter: `.gitignore` excludes `wk/` and most generated
`kicad_files/*`, so a fresh worktree fails tests a working tree passes. That is
environment, not regression — check it before diagnosing anything.

## How to skip honestly

`tests/run_all.py` defines `SKIP_EXIT = 77`. A test that **cannot** run prints
`SKIP: <reason>` and exits 77. It then lands in its own bucket, is reported
under *"SELF-SKIPPED (n) — these asserted NOTHING; they are not passes"*, and
**still forces a non-zero exit**. Use it only when a prerequisite is genuinely
absent.

Do **not** use `--fast` exclusion as a skip: those land in the `skipped` bucket,
which prints no reason and never appears under that banner.

## How to declare a long runtime

`RUN_ALL_TIMEOUT = <seconds>` at module scope. Read from the source (never by
importing — importing a test runs it), and it can only raise the budget, never
lower it below the global `--timeout`. Put the measurement in the comment beside
it.

---

## The 2026-08-19 sweep: 22 failures + 3 timeouts, all pre-existing

None was introduced by `run22-honesty-fixes`. Established by running four at
base `798b11a3` in the same working tree and getting identical exit codes *and*
identical failure text, and by diffing the product module each test exercises
against base.

### Class 1 — the #522 reorg (18 tests, one bug)

`ee860796` (2026-08-04) moved the CLIs out of the repo root into `py_router/`,
`py_placer/`, `py_tools/`. These tests kept spawning
`os.path.join(ROOT, 'route.py')`. Python exits **2** with `can't open file …:
[Errno 2]` on **stderr**, and they all assert on **stdout** — so the class
surfaced as a bare `AssertionError` with an empty message that reads exactly
like a product failure.

**Fixed** by `run_utils.tool()` / `tool_env()`, delegating to the resolver the
repo already ships (`krt_capabilities._tool_path`). `tool()` **raises**, naming
what it looked for and where — a moved tool must fail at the call site, not
three frames away inside a subprocess. A static lint cannot do this job:
`test_431_board_gates` builds `os.path.join(ROOT, script)` with `script` a
variable, so the bad path is only knowable at runtime. **The raising resolver is
the gate.**

**What it uncovered — the actual value:**

- `test_soft_joint` — four checks expecting `n == 0` passed only because the
  subprocess produced no output at all. Five of ten were vacuous; all ten now
  run and pass.
- `test_457_startup_checks` — asserted `returncode == 1`, which a
  `ModuleNotFoundError` also satisfies. Exactly the false-clean that
  `run_utils._ACCIDENTS` exists to catch, in a file that never used it.
- `test_protect_nets_flag`'s surviving unit test was **doubly stale underneath
  the dead one**: its fake pcb lacked `segments`/`vias` (added by #521) and set
  `_protection_map_cache` after that memo was renamed `_protection_map_memo`, so
  it silently supplied *no* protection and every net read as rippable. It could
  not have passed for a long time and nobody could see it, because the
  `--protect-nets` function beside it crashed first.

**Latent, not churned:** eight files still use `PYTHONPATH=ROOT`
(`test_place_reconstruct`, `test_run4_anchors_first`, `test_run4_reconstruct`,
`test_run5_emit_guard` and others). All currently green, so they are recorded
here rather than changed while they pass.

### Class 2 — stale fixtures

`11f6d48f` (2026-08-16) gave tigard a `.kicad_pro`; `ada4ca87` corrected
`list_nets`' docstring but not the two tests still using tigard as their
"declares nothing" example — the 8-check failure in `test_run12_tools` and one
in `test_board_floors`.

Repointed to `splitflap_driver` (no `.kicad_pro` tracked **or** on disk; only
four are tracked at all: `flat_hierarchy`, `routed_output`, `tigard`, `watchy`,
so a fresh clone agrees).

**The first repoint was too broad and the tests caught it.** `NO_FLOOR_BOARD`
was doing double duty and the channel/stack cases depend on tigard's actual refs
(`KeyError: 'J2'`). Split into `NO_FLOOR_BOARD` (declares nothing) and
`GEOM_BOARD` (real geometry). Changing a fixture used for two purposes to fix
one of them is a goalpost move; the split is not.

`assert_still_undeclared` is the anti-rot guard, and it is **mutation-tested**:
pointed at tigard it fails with *"FIXTURE ROT … now DECLARES (1 class(es), 3
constraint(s))"*, and passes on the replacement. A guard nobody has seen fail is
a guard nobody knows works.

### Class 3 — the product improved and the test didn't hear

`test_549_floorplan_grade` asserted a `gr_circle`-only board is refused with
`bounds is None`. `outline_state` now tessellates a circular outline into 64
segments and derives correct bounds — the defect it guarded is **fixed**.

The refusal property still matters, so it keeps a test, on the fixture that
still cannot yield bounds. I probed four candidates (circle-only, no geometry,
one stray segment, two disjoint segments); only **no Edge.Cuts geometry at all**
still refuses. And the capability that retired the old fixture gets its own
test — 64 segments, bounds `(30,30,70,70)`, `emit_intent` does not raise.
Without that second test the first starts passing for the *wrong* reason if
tessellation regresses, which is how the stale version survived.

### Class 4 — doc lint

`test_run8_skills_generic`: three `wk/runNN` strings genericised to
`<workdir>/<board>`. Aligned with the `--workdir` work rather than incidental to
it: the run directory is a parameter now, so a hardcoded `wk/run17` in the
driver's own emitted text is actively wrong.

Its "the combined skill is thin (< 200 lines)" check was **re-baselined to 230,
not satisfied by trimming**, with the reason at the assertion. `ee6c3ad1`
(2026-08-12) took that skill 103 → 204 lines in one commit, so it had been red
for a week — and the content added is the L2/L3/L5 board-binding gates and the
delegation rule, which the combined skill alone owns. The sibling check that
*directly* measures restatement passes; line count is the proxy, and the proxy
was stale.

Moving a threshold to make a test pass is normally the failure mode this work
removes. What makes this one legitimate: the direct measure is green, the growth
is auditable to one commit, and the reason is recorded where the next reader can
disagree with it.

### Class 5 — timeouts

Three tests exceeded the runner's 600 s cap while carrying **larger internal
budgets**. Measured: `test_obstacle_map_balance` passes **alone in 681 s** with
all 18 checks green, including its two self-checks that it did real work.

Fixed by `RUN_ALL_TIMEOUT` (above), declared at 1200 / 3600 / 1800 with the
measurement beside each.

**Still worth doing:** `test_obstacle_map_balance` is four independent stages
(`route.py`, `route_diff`, `route_planes`, `repair`) and would be better as four
files — each inside the default budget, and a future leak attributed to one
front-end instead of one opaque timeout. The budget makes it honest; the split
would make it diagnosable.

---

## Corrections I had to make during this sweep

Recorded because the method looked rigorous in both cases and was not.

1. **"The router is missing."** It is not. `rust_router/grid_router.pyd` is
   built and version-matched; my import test put `py_router` on `sys.path`
   instead of `rust_router`. Building it would have been wasted work, and the
   claim would have sent the next reader after the wrong prerequisite.

2. **"`test_run5_exchange` is the one genuine failure left."** It was the #522
   path class, in a file I had left out of the batch. I "verified" it against
   the base engine *with the path fix already in place* — except the fix was
   never applied to that file, so both arms of the comparison carried the same
   broken invocation. Identical failures on both sides proved only that the bug
   predated the branch: true, and useless. The `2 != 0` was the tell — exit 2 is
   a usage error, not a placement verdict.

3. **`test_457_determinism` listed as green without being run.** The patch
   script had silently no-opped (its pattern did not match this file's
   `PYTHONHASHSEED=seed, PYTHONPATH=ROOT`) and reported success anyway. I took
   the claim from the script's output instead of a test run. Verified means run.

---

## A third of the corpus no-op baseline is untracked (2026-08-19)

`tests/stress/corpus_noop_sweep.py` is the standing gate for placement-engine
changes, and its own docstring says *"A baseline nobody can reproduce is not a
baseline."* **11 of the 33 boards it pins are not tracked by git** — they are
generated tool outputs (`fanout_output1/2`, `fanout_starting_point`,
`flat_hierarchy_routed`, `interf_u_connected/fanout/plane/routed`,
`qfn_fanned_out`, `sonde_u_routed`, `sonde_u_routed_routed`), matched by
`.gitignore` patterns such as `/kicad_files/interf_u_*.kicad_pcb`.

So a row's recorded value depends on which generation of the file the developer
happens to have on disk. Measured while landing the containment census:

```
CHANGED interf_u_plane:reconstruct: refused:carries copper  ->  quiet
CHANGED interf_u_plane:repair:      refused:board state     ->  quiet
```

Nothing moved on either arm — only the *reason* changed, because the copy on
disk had been regenerated copper-free by an earlier sweep. **Reproduced with the
engine change stashed out**, so it is fixture state, not a regression. Both rows
were left as they were rather than re-recorded: pinning them to `quiet` would
freeze a state that only exists on a machine that regenerated the file, and
would hide this finding.

A fresh clone does not see it. Those boards are absent, so they land in
`skipped` ("outside this run's scope") rather than `gone` — a deliberate branch
with its reasoning in the source. That is why the gate can be green on CI and
red on a working tree, which is the confusing part.

**Not fixed here**, because the fix changes what the gate covers and that is a
decision, not a cleanup. The options are to drop the untracked boards from the
baseline (22 boards, all reproducible), or to regenerate them deterministically
as part of the sweep. Until then, read a changed row against `git
check-ignore` before believing it.

---

## The bodyless-footprint hole is DISCLOSED, not fixed (2026-08-19)

The body-containment channel cannot judge a part that draws no `.Fab` outline,
and `grade_body_overlap` reports that as `fab_unjudged`. It looks like a gap to
close. It is not, and the measurement is recorded here so nobody spends a day
rediscovering it:

- **144 of 1583 footprints across 33 boards are unjudged, and 144 of 144 draw
  ZERO `.Fab` geometric primitives.** The classes "primitive type not handled"
  and "degenerate bbox" are both empty; every kind used on that layer
  (`fp_line`, `fp_arc`, `fp_circle`, `fp_poly`, `fp_rect`) is already read.
- **313 footprints DO have non-closing `.Fab` chains** — tigard's `Q1` stops
  0.02 mm short on its left edge — and every one is judged correctly, because a
  bbox is a min/max over points and the gap is interior to it. A closure or
  tolerance fix therefore moves **0 footprints and 0 pairs**.
- The unjudged parts are mounting holes, testpoints, logos, fiducials and panel
  tabs — parts that genuinely draw no body.
- **The one change that would close the hole is measured dangerous.** Giving a
  bodyless part a fallback body (courtyard, else pad bbox) adds **77 new fab
  pairs, 65 above the containment threshold**, dominated by rp2350's `Teensy40`
  — a bodyless module whose courtyard swallows 56 neighbours at frac 1.0. It
  also breaks the 4-pair calibration gate outright.

Two tests hold this. `test_the_bodyless_hole_is_exactly_what_was_measured` pins
the count at 144 so a "helpful" parser change has to argue with the number, and
`test_no_unjudged_part_has_fab_geometry_to_read` pins the **invariant** — no
unjudged part has readable `.Fab` geometry — which is the claim that actually
licenses "no parser fix helps", and which survives the corpus-membership drift
described in the section above.
