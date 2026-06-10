---
name: sync-ipc-migration
description: Brings recent main-branch fixes onto the ipc-migration branch via a guided git merge, resolving conflicts so the code really works under the kipy/IPC plugin (KiCad 10) instead of the old SWIG interface. Use whenever ipc-migration needs to catch up with main.
---

# Sync ipc-migration with main

`ipc-migration` is a structural port of the plugin from the SWIG `pcbnew`
interface to the IPC API (kipy / KiCad 10). It is **not** a simple feature
branch — files were renamed, deleted, and added during the port, so main's
commits often touch code that looks different (or no longer exists) on
ipc-migration. The goal of this skill: apply main's recent fixes to
ipc-migration **without reintroducing SWIG-era code paths**, and verify the
result actually works under IPC.

**Never alter `main`.** All work happens on `ipc-migration`. Do not push
unless the user explicitly asks.

## Know the port's shape (what differs structurally)

These are the load-bearing differences between the branches. When a conflict
or auto-merge touches one of them, think IPC, not SWIG:

- **Renamed:** `kicad_routing_plugin/swig_gui.py` → `routing_dialog.py`.
- **Deleted on ipc-migration:** `kicad_routing_plugin/action_plugin.py`,
  `board_swaps.py`, `deps_check.py`, and root `__init__.py`'s SWIG entry.
- **Added on ipc-migration:** `kicad_ipc_adapter.py`, `kicad_routing_plugin/ipc_entry.py`,
  `ipc_settings_store.py`, `plugin.json`, `requirements.txt`.
- **Rewritten for IPC:** board mutations go through the IPC adapter
  (`apply_routing_results`, kipy commits) instead of `import pcbnew` /
  `from .board_swaps import apply_swaps_to_board`.
- **metadata.json** on ipc-migration carries `"runtime": "ipc"`,
  `"kicad_version": "10.0"` (main carries `swig` / `9.0`).

Run this first to re-orient (the set of renamed/deleted files may grow):

```bash
git fetch --all
git checkout ipc-migration
git log --oneline ipc-migration..main          # main fixes not yet ported
git diff --stat main ipc-migration             # structural divergence
```

## Step 1: Guided merge (preferred over cherry-pick)

A 3-way merge handles the renames/deletions far better than cherry-picking,
and matches this branch's history ("Merge main into ipc-migration: ...").

```bash
git merge --no-commit --no-ff main
```

Inspect every conflict AND spot-check the auto-merges (auto-merge can take
main's SWIG version of a file wholesale). `git diff --name-only --diff-filter=U`
lists conflicts; `git diff main:<file> <file>` shows what an auto-merge produced
vs main.

## Step 2: Resolve per file-class

**Conflict resolution rules — apply the _intent_ of main's change, keep IPC plumbing:**

- **metadata.json** — keep ipc-migration's runtime fields (`runtime: ipc`,
  `kicad_version: 10.0`, `status`). The version number stays in lock-step with
  `main` and `VERSION` (do not invent a new version — historically both branches
  share the same number, e.g. 0.15.9). Update the `download_url` to that same version.
- **VERSION** — keep it equal to main's value (shared across both branches).
- **package_pcm.py / install_plugin.py** — take main's packaging improvements.
  Main's denylist packaging (ship the whole repo tree minus `ROOT_EXCLUDE`) is a
  *strict improvement* for IPC: the old allowlist would drop IPC-only files like
  `plugin.json`; the denylist ships them automatically. Before accepting, confirm
  ipc-migration had no bespoke packaging logic worth preserving
  (`git diff ipc-migration:package_pcm.py main:package_pcm.py`). Keep the
  `.claude/` skills and `CLAUDE.md` in installs.
- **GUI files** (`differential_gui.py`, `fanout_gui.py`, `planes_gui.py`,
  `routing_dialog.py`) — port main's *UI intent* (button labels, new buttons,
  layout tweaks) but **never** reintroduce `import pcbnew`,
  `from .swig_gui ...`, or `from .board_swaps ...`. Keep the IPC handlers
  (`_apply_results_to_board` via the IPC adapter, `_on_cancel_or_close`).
  Close buttons use `self.GetTopLevelParent().EndModal(wx.ID_CANCEL)` — that
  wx pattern is identical on both branches, so it ports as-is.
- **docs/, README.md, .claude/skills/** — take main's content additions
  (branch-agnostic). New skills on main are simply added.

## Step 3: Verify it really works under IPC

```bash
# No leftover conflict markers (ignore ==== separator lines inside docs):
git grep -nE '^(<<<<<<<|>>>>>>>)' -- .

# No SWIG code paths reintroduced into the merged plugin files:
git grep -n 'import pcbnew\|from .swig_gui\|from .board_swaps' -- kicad_routing_plugin/

# Changed Python still compiles (syntax-only; wx/kipy need not be installed):
python3 -m py_compile install_plugin.py package_pcm.py kicad_routing_plugin/*.py

# metadata.json is valid and carries IPC runtime fields:
python3 -c "import json; m=json.load(open('metadata.json')); print(m['versions'][0])"

# main was NOT altered, and is now fully contained in ipc-migration:
git rev-parse --short main origin/main        # must be equal
git log --oneline ipc-migration..main         # must be empty
```

If anything regressed, look at the IPC adapter usage in `routing_dialog.py`
for the correct pattern to mirror.

## Step 4: Commit (do not push)

Commit the merge with a message summarizing which fixes were ported and how IPC
plumbing was preserved. Leave pushing to the user unless they ask. End the
commit message with:

```
Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
```
