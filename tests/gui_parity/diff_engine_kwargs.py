#!/usr/bin/env python3
"""Diff two KICAD_DUMP_BATCH_KWARGS captures (CLI reference vs GUI run) and report
the non-benign per-engine parameter divergences.

This is the diagnostic that localized every GUI/CLI plane-param divergence in the
#362 sweep (same_net_pad_clearance, board_edge_clearance, min_track_width,
all_layers, no_bga_zone, max_iterations). It complements the regression GATES
(test_gui_livechain_rp2350.py, test_plane_*_parity.py): the gates lock in a fixed
divergence; this tool FINDS new ones.

## How to produce the two captures

Both fronts write one JSONL line per engine call (batch_route + the plane engines
create_plane / repair route_planes) when BOTH env vars are set:
    KICAD_DUMP_BATCH_KWARGS=<file> KICAD_DUMP_BATCH_KWARGS_CONTINUE=1

1. CLI reference -- run the chain FRESH AT HEAD, never reuse recorded stress boards
   (they were routed at an older commit; diffing against them manufactures phantom
   divergences). Replay the board's redo_commands.sh (or the exact tool commands)
   with the env vars set, from the unrouted board:
       KICAD_DUMP_BATCH_KWARGS=/tmp/cli.jsonl KICAD_DUMP_BATCH_KWARGS_CONTINUE=1 \
         bash <replay of the chain>

2. GUI capture -- launch pcbnew FROM A TERMINAL with the env vars (so the in-process
   plugin inherits them; launching from the Dock does NOT), open the unrouted board,
   and run the plan from the Claude tab:
       KICAD_DUMP_BATCH_KWARGS=/tmp/gui.jsonl KICAD_DUMP_BATCH_KWARGS_CONTINUE=1 \
         "/Applications/KiCad/KiCad.app/Contents/Applications/pcbnew.app/Contents/MacOS/pcbnew" board.kicad_pcb

3. Diff:  python3 diff_engine_kwargs.py /tmp/cli.jsonl /tmp/gui.jsonl
   Exit code is the number of engine calls with non-benign divergences (0 = parity).

Benign-by-design differences are filtered: dry_run (the GUI applies via pcbnew,
the CLI writes a file), return_results (same), layer_costs [] vs None (equivalent),
routing_layers None vs the full copper stack (None auto-detects to it), and
skip_existing_zones (moot on a board with no pre-existing pours). Everything else
that differs is a real divergence to reconcile in the GUI call site.

CAVEAT -- route calls pair by index, planes pair reliably. The plane engines fire
a fixed number of times (create_plane once, repair route_planes once per repair
step), so they pair 1:1 and their MATCH/diff verdict is authoritative. batch_route,
by contrast, fires a nondeterministic number of times (rip-up retries, reconnect
sub-routes), so CLI vs GUI often have DIFFERENT route-call counts and this tool's
by-index pairing compares mismatched steps -- treat batch_route diffs as noise
unless the counts match. The plane rows are what this tool is for.

No KiCad/pcbnew needed -- pure JSONL processing.
"""
import json
import sys

# Differences that are correct-by-design, not divergences (see module docstring).
BENIGN_KEYS = {
    '_engine', 'input_file', 'output_file',
    'dry_run', 'return_results',
    'layer_costs',        # [] (CLI) vs None (GUI) -- get_layer_costs treats equal
    'routing_layers',     # None (CLI auto-detect) vs full copper stack (GUI)
    'skip_existing_zones',
    'net_names',          # order/expansion differences; connectivity-equivalent
}


def _load(path):
    """engine-name -> list of kwargs dicts, in call order."""
    from collections import defaultdict
    by_engine = defaultdict(list)
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            d = json.loads(line)
            eng = d.get('_engine') or f"batch_route[{len(d.get('net_names', []))}nets]"
            by_engine[eng].append(d)
    return by_engine


def main(argv):
    if len(argv) != 3:
        print("usage: diff_engine_kwargs.py <cli_kwargs.jsonl> <gui_kwargs.jsonl>")
        return 2
    cli = _load(argv[1])
    gui = _load(argv[2])
    engines = sorted(set(cli) | set(gui))
    total_divergent = 0
    for eng in engines:
        cs, gs = cli.get(eng, []), gui.get(eng, [])
        n = min(len(cs), len(gs))
        if len(cs) != len(gs):
            print(f"  {eng}: call-count differs (CLI {len(cs)}, GUI {len(gs)}) "
                  f"-- comparing the first {n}")
        for i in range(n):
            c, g = cs[i], gs[i]
            keys = (set(c) | set(g)) - BENIGN_KEYS
            diffs = [(k, c.get(k, '<absent>'), g.get(k, '<absent>'))
                     for k in sorted(keys) if c.get(k) != g.get(k)]
            if diffs:
                total_divergent += 1
                print(f"  {eng}#{i}: {len(diffs)} non-benign diff(s)")
                for k, a, b in diffs:
                    print(f"      {k}: CLI={a!r}  GUI={b!r}")
            else:
                print(f"  {eng}#{i}: MATCH")
    print(f"\n{total_divergent} engine call(s) with non-benign divergences "
          f"({'PARITY' if total_divergent == 0 else 'DIVERGENCES FOUND'})")
    return total_divergent


if __name__ == '__main__':
    sys.exit(main(sys.argv))
