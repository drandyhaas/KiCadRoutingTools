"""CLI/GUI parity for placement-block routing scope (#459).

`route.py --group BLOCK` narrows a run to one placement block. A recorded plan
step carries that block, and `ai_plan.apply_step_selection` has to resolve it to
the SAME net list the CLI would route -- otherwise a replayed plan routes a
different set of nets than the chain it replays.

This is the gap that made the check worth writing: all three of `--group`,
`--undo` and `--preview` originally fell through `manifest_to_plan` into the
generic `params` bucket, where `ai_plan` drops any param with no matching dialog
control. `nets` then defaulted to `['*']`, so a recorded

    route.py in.kicad_pcb out.kicad_pcb --undo --group decap:U3

replayed as "route every net on the board" -- the inverse operation, at ~100x
the scope, silently. `test_manifest_plan_parity.check_group_flags` guards the
converter half (no wx needed); this guards the RESOLUTION half, which needs
KiCad's python because `ai_plan` imports wx at module scope.

Run it the same way as its siblings -- it re-execs into KiCad's python itself:

    python3 tests/gui_parity/test_group_scope_parity.py
"""

import glob
import importlib.util
import os
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REPO)
# py_router, or `main()`'s very first import dies: `group_routing` and
# `kicad_parser` both live there since the #522 reorg, and REPO alone does not
# reach them. Every sibling gate in this directory inserts it; this one did
# not, so it exited 1 on EVERY machine -- cloud and local alike -- before it
# checked anything. `run_all.py`'s flat glob does not collect this directory,
# which is why a permanently red gate went unnoticed.
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, os.path.join(REPO, 'py_placer'))  # placement split
KICAD_PYTHONS = [
    "/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3",
    "/usr/bin/python3",
    os.path.expandvars(r"C:\\Program Files\\KiCad\\bin\\python.exe"),
    *sorted(glob.glob(r"C:\Program Files\KiCad\*\bin\python.exe"), reverse=True),
]

BOARD = os.path.join(REPO, "kicad_files", "rp2350_fpga_eensy_prePlane.kicad_pcb")
BLOCK = "sheet:558c3023"


def _reexec_into_kicad():
    """ai_plan imports wx at module scope, so this gate needs KiCad's python.
    Re-exec rather than skip -- a gate that quietly skips is a gate that never
    runs (CLAUDE.md records six consecutive sessions of exactly that)."""
    try:
        import wx  # noqa: F401
        return
    except ImportError:
        pass
    for cand in KICAD_PYTHONS:
        if cand == sys.executable or not os.path.exists(cand):
            continue
        if subprocess.run([cand, '-c', 'import wx, pcbnew'],
                          capture_output=True).returncode == 0:
            sys.exit(subprocess.run([cand, os.path.abspath(__file__)]
                                    + sys.argv[1:]).returncode)
    print("ERROR: no python with wx+pcbnew found -- cannot check GUI resolution")
    sys.exit(2)


def _load_ai_plan():
    sys.path.insert(0, os.path.join(REPO, 'kicad_routing_plugin'))
    sys.path.insert(0, os.path.join(REPO, 'py_router'))  # placement split
    sys.path.insert(0, os.path.join(REPO, 'py_tools'))  # placement split
    spec = importlib.util.spec_from_file_location(
        'ai_plan_under_test', os.path.join(REPO, 'kicad_routing_plugin', 'ai_plan.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def main():
    _reexec_into_kicad()
    from group_routing import block_net_names, block_refs
    from kicad_parser import parse_kicad_pcb
    from placement.groups import parse_sources

    ai_plan = _load_ai_plan()
    pcb = parse_kicad_pcb(BOARD)
    all_nets = sorted(n.name for n in pcb.nets.values() if n.net_id > 0 and n.name)
    refs = block_refs(pcb, BLOCK, parse_sources('sheet'))

    bad = []
    for scope in ('internal', 'touching'):
        cli = sorted(block_net_names(pcb, refs, scope))
        notes = []
        gui = sorted(ai_plan._group_net_names(
            pcb, {'group': BLOCK, 'group_by': 'sheet', 'group_scope': scope},
            all_nets, notes))
        status = "OK" if cli == gui else "MISMATCH"
        print(f"  {scope:9s} CLI={len(cli):3d} GUI={len(gui):3d}  {status}")
        if cli != gui:
            bad.append((scope, f"CLI {cli} != GUI {gui}"))
        if notes:
            bad.append((scope, f"unexpected notes: {notes}"))
        if len(gui) >= len(all_nets):
            bad.append((scope, f"did not narrow at all ({len(gui)} of "
                               f"{len(all_nets)} board nets) -- this is the "
                               f"whole-board blow-up the gate exists for"))

    # An unresolvable block must NOT silently fall back to every net.
    notes = []
    got = ai_plan._group_net_names(pcb, {'group': 'no-such-block'}, all_nets, notes)
    if not notes:
        print("  unknown    MISMATCH (resolved silently)")
        bad.append(('unknown-block', "no note recorded; a plan would widen to "
                                     "the whole board with nothing printed"))
    else:
        print(f"  unknown    OK (noted, {len(got)} nets left unscoped)")

    print(f"\nGroup scope parity: {len(bad)} problem(s).")
    for scope, why in bad:
        print(f"    {scope}: {why}")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
