#!/usr/bin/env python3
"""Migrate recorded `redo_commands.sh` manifests to the current CLI.

Manifests are the no-LLM replay corpus (see RUNBOOK "Replaying & A/B"). They
are plain recorded command lines, they are NOT under version control, and
`ab_replay_grade.py` / `redo_diff_stage.py` run them verbatim -- so a flag the
CLI has dropped makes the chain die in argparse instead of replaying.

This rewrites them in place. DRY RUN BY DEFAULT; pass --apply to write.
Idempotent: a migrated manifest is left untouched on a second run.

WHAT IT DOES TODAY
  route_planes.py lines: drop flags the pours-first architecture removed
  (#562 -- the pour places no taps and cannot rip, so these fed nothing).
  Only route_planes.py lines are touched. route_disconnected_planes.py
  lines are left exactly as recorded -- NOTE that module is now
  repair_planes.py (standalone utility, not a chain step), so manifests
  carrying the old name no longer replay past that step; the decision
  (2026-08-04) is that old manifests are historical records, not runnable
  chains. The pours-first STEP-ORDER rewrite below is what would make a
  manifest runnable again.

WHAT IT DOES NOT DO YET
  The pours-first STEP-ORDER rewrite (pour first, drop the now-no-op repair
  and reconnect steps, add --power-nets to the route step). That is a
  different transformation: it changes what the chain MEASURES, not just
  whether it parses, so it belongs in its own reviewed pass.

    python3 tests/stress/migrate_manifests.py [ROOT] [--apply]

ROOT defaults to ~/Documents/kicad_stress_test.
"""
import os
import re
import sys

# flag -> does it consume a following value? (store_true flags are False)
REMOVED_FROM_ROUTE_PLANES = {
    '--max-search-radius': True,
    '--max-via-reuse-radius': True,
    '--close-via-radius': True,
    '--rip-blocker-nets': False,
    '--max-rip-nets': True,
    '--reroute-ripped-nets': False,
}

# Only rewrite lines invoking THIS script; the repair engine keeps its flags.
TARGET = 'route_planes.py'


def strip_flags(line: str):
    """Return (new_line, [flags dropped]).

    SURGICAL, not tokenized: excise exactly the flag (and its value, if it
    takes one) and leave every other byte of the line alone. Tokenizing and
    re-joining would silently renormalize whitespace inside quoted arguments
    -- unacceptable here, because these manifests are the replay corpus, are
    not under version control, and are rewritten IN PLACE.
    """
    if TARGET not in line:
        return line, []
    out, dropped = line, []
    for flag, takes_value in REMOVED_FROM_ROUTE_PLANES.items():
        # \b so --max-rip-nets never matches inside a longer flag name.
        pat = (re.escape(flag) + r'\b(?:=\S+|\s+\S+)?' if takes_value
               else re.escape(flag) + r'\b')
        while True:
            m = re.search(r'[ \t]+' + pat, out)
            if not m:
                break
            dropped.append(flag)
            out = out[:m.start()] + out[m.end():]
    return out, dropped


def main():
    args = [a for a in sys.argv[1:]]
    apply = '--apply' in args
    args = [a for a in args if a != '--apply']
    root = os.path.expanduser(args[0] if args
                              else '~/Documents/kicad_stress_test')

    manifests = []
    for dirpath, _dirs, files in os.walk(root):
        if 'redo_commands.sh' in files:
            manifests.append(os.path.join(dirpath, 'redo_commands.sh'))
    manifests.sort()

    changed, total_drops = 0, 0
    per_flag = {}
    for path in manifests:
        try:
            with open(path, 'r', errors='replace') as fh:
                lines = fh.readlines()
        except OSError as e:
            print(f"  SKIP {path}: {e}")
            continue
        new_lines, file_drops = [], []
        for ln in lines:
            nl, dropped = strip_flags(ln)
            new_lines.append(nl)
            file_drops.extend(dropped)
        if not file_drops:
            continue
        changed += 1
        total_drops += len(file_drops)
        for f in file_drops:
            per_flag[f] = per_flag.get(f, 0) + 1
        rel = os.path.relpath(path, root)
        print(f"  {rel}: -{len(file_drops)} flag(s) "
              f"({', '.join(sorted(set(file_drops)))})")
        if apply:
            with open(path, 'w') as fh:
                fh.writelines(new_lines)

    print()
    print(f"{'APPLIED' if apply else 'DRY RUN'}: "
          f"{changed}/{len(manifests)} manifest(s) need changes, "
          f"{total_drops} flag occurrence(s)")
    for f, n in sorted(per_flag.items()):
        print(f"    {f}: {n}")
    if not apply and changed:
        print("\nRe-run with --apply to write.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
