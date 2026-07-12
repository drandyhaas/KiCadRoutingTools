# Contributing to KiCadRoutingTools

Thanks for helping make the router better! There are two contributions we
value above everything else:

1. **Route a board we haven't seen and tell us what broke.** Every new real
   board finds real bugs. You don't need to know the codebase — you need a
   `.kicad_pcb` file and an hour.
2. **Fix an open issue and send a pull request.**

Both are described below. For either one, [Claude
Code](https://claude.ai/claude-code) does most of the heavy lifting — the
repository ships skills that plan, run, and grade a full routing workflow.

---

## 1. Route a new board (the best way to contribute)

The router is stress-tested against a corpus of open-hardware boards
(`tests/stress/`), but the corpus can't cover everything. Boards **not** in
the corpus — your own designs, or any open-hardware KiCad project — are where
new bugs live: DRC violations, disconnected pads, parser errors, crashes,
pathological slowness.

### Setup

```bash
git clone https://github.com/drandyhaas/KiCadRoutingTools.git
cd KiCadRoutingTools
python3 build_router.py        # fetches the prebuilt Rust router for your platform
```

### Route it with Claude

Open Claude Code in the repository and point the planning skill at your board:

```
> /plan-pcb-routing path/to/your_board.kicad_pcb
```

The skill analyzes the board (fanout needs, differential pairs, power nets,
plane candidates), proposes a step-by-step plan, runs the commands, and
verifies the result. The same flow is available inside KiCad via the plugin's
**Claude tab** (*Plan Routing* → *Run Selected Steps*) — see
[docs/claude-skills.md](docs/claude-skills.md).

Prefer driving it by hand? The full CLI workflow is in the
[README](README.md) (`bga_fanout.py`, `route.py`, `route_diff.py`,
`route_planes.py`, …).

### Grade the result honestly

This is where contributions become valuable. Whatever routed, verify it —
the two checks are independent and **both** are required:

```bash
python3 check_drc.py your_board_routed.kicad_pcb        # clearances / shorts
python3 check_connected.py your_board_routed.kicad_pcb  # every pad actually reached
```

- A DRC-clean board can still be disconnected (floating copper has no
  clearance conflicts), so never skip `check_connected.py`.
- `check_drc.py` auto-grades at the clearance the board was actually routed
  to (recorded in the output's `.kicad_pro`). Don't grade at a stricter
  value than the route used — that manufactures phantom violations.
- For a one-shot full QA (DRC + connectivity + length matching + diff pairs
  + GND return vias), use `> /review-routed-board your_board_routed.kicad_pcb`.
- If you have KiCad installed, `kicad-cli pcb drc` is the reference oracle
  for cross-checking anything borderline.

### What counts as a finding

Anything in this list is worth an issue, even if you're not sure it's a bug:

- **DRC violations** in the output (shorts, clearance grazes, hole-to-hole)
- **Connectivity problems** — pads left unconnected, especially when the
  router's own summary claimed success
- **Crashes / tracebacks / hangs** in any script or the plugin
- **Parser errors** on a board KiCad itself opens fine
- **Pathological slowness** (a step taking minutes where similar boards take
  seconds)
- **GUI vs CLI divergence** — the plugin producing a different result than
  the equivalent command line (we treat this as a bug class of its own)

---

## 2. Filing a great issue

A reproducible issue gets fixed ten times faster. Include:

1. **A pointer to the board file.** Best: a link to the board's source
   repository (and commit) if it's open hardware, or attach a zip with the
   `.kicad_pcb` + `.kicad_pro`. If the board is private, say so — a link to
   a similar open board that reproduces the problem, or a stripped-down
   repro, is the next best thing. Issues without a board are still welcome,
   just slower to act on.
2. **The exact commands you ran**, in order, with all flags (copy-paste from
   your shell history), or the plan the Claude tab executed. Routing output
   depends heavily on the parameter chain, and re-running the same board at
   different clearances is a different experiment.
3. **The full output** — the router's summary plus `check_drc.py` /
   `check_connected.py` results, and the complete traceback for crashes.
4. **Versions**: KiCadRoutingTools version (or commit), KiCad version, OS,
   Python version.

One issue per distinct failure mode, please — a short with a repair step and
a crash in the parser are two issues even if the same board produced both.

---

## 3. Fixing issues and sending pull requests

Issues labeled with concrete checker output and a board pointer are the most
approachable. Before you start:

- **Read [CLAUDE.md](CLAUDE.md).** It's written for AI-assisted development
  but it is the house rulebook for humans too: how to grade results, the
  CLI/GUI parity rules, and the testing conventions. If you work with Claude
  Code in this repository, it's loaded automatically.
- **Prefer Python-only changes.** The Rust router (`rust_router/`) requires a
  version bump, local rebuilds, and redistributing per-platform binaries via
  GitHub Releases — heavy overhead. If a fix seems to need a Rust change,
  raise that in the issue first; there is usually a Python-side approach.
- **Mind CLI/GUI parity.** The CLI scripts and the KiCad plugin call the same
  shared engine functions, but fixes at the edges (argparse defaults, GUI
  panels, post-passes in a `main()`) reach only one front. The rule of thumb
  and the full checklist live in CLAUDE.md ("Keep CLI and GUI routing in
  sync"). When you touch routing behavior, run the parity gates:

  ```bash
  python3 tests/gui_parity/test_manifest_plan_parity.py
  python3 tests/gui_parity/test_cli_postpass_coverage.py
  ```

### Verifying a fix

- Reproduce the reported failure **before** the fix, then show it gone
  **after**, on the same board and command chain.
- Grade with `check_drc.py` (at the routed clearance) **and**
  `check_connected.py` — a fix that trades a DRC violation for a
  disconnected pad is not a fix.
- Routing is deterministic, but outputs carry per-run random UUIDs — compare
  checker counts, never file hashes.
- Run the test scripts near your change (they're standalone:
  `python3 tests/test_<name>.py`), and `python3 tests/run_doc_examples.py`
  if you touched documented APIs.

### The pull request

- Small, focused PRs against `main`. One failure mode per PR, matching the
  issue it closes.
- In the description: the issue number, the board + command chain used to
  reproduce, and before/after checker output.
- If your change adds an engine parameter or flag, follow the
  "Claude-settable end to end" checklist in CLAUDE.md (GUI control, plan
  executor, manifest converter, parity test) — a flag only the CLI knows
  about silently does nothing in the plugin.

Claude Code is genuinely good at this loop: point it at the issue
(`gh issue view <n>`), let it reproduce, fix, and verify — CLAUDE.md keeps it
honest about grading.

---

## 4. Growing the stress corpus

If your board is open hardware and exposed a bug, it's a candidate for the
permanent regression corpus. `tests/stress/validate_candidate.py` checks
whether a board meets the corpus bar (real outline, enough footprints and
nets, parseable by pcbnew); the full pipeline is documented in
`tests/stress/RUNBOOK.md`. Mention in your issue that the board is
corpus-eligible and we'll take it from there.

---

## License

By contributing, you agree that your contributions will be licensed under the
project's [MIT License](LICENSE).
