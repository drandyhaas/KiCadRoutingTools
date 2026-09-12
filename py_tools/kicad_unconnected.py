#!/usr/bin/env python3
"""Authoritative unconnected-item count via kicad-cli DRC (--refill-zones).

The oracle cross-check for `check_connected.py`: our fill validator credits
plane connections through pour islands that KiCad's fill (min-width pinch +
isolated-island pruning) never actually joins -- ottercast k_seam graded 1
unconnected by check_connected and 8 by KiCad, the 7 extras being dogbone
vias the pour physically cannot reach (barrel-to-barrel 0.06mm vs the ~0.3mm
two-clearances + min-fill-width the pour needs). Wave/replay grading should
report BOTH numbers; a gap between them is a grader-parity bug or a real
plane gap, never noise.

Usage:
    python3 kicad_unconnected.py board.kicad_pcb [--items] [--json out.json]
                                                 [--pairs-json pairs.json]

Prints ``KICAD_UNCONNECTED: <n>`` (or ``KICAD_UNCONNECTED: ERR <why>`` when
kicad-cli is unavailable/fails -- graders treat that as "no oracle", never 0).
Exit codes follow the repo's gate convention: 0 clean, 4 when items remain,
3 when the oracle could not run (no kicad-cli / DRC failure -- NOT clean).

``--pairs-json`` writes the PARSED endpoint pairs (kicad_oracle's tuples:
net + both endpoints with x/y/layer/kind) -- a machine-readable work list
naming each remaining join, so an endgame step can target exact pad<->copper
pairs instead of re-deriving them from the human-readable --items text.

Two caller contracts on --pairs-json worth stating (both verified):
  * on exit 3 (no oracle) NO pairs file is written -- the failure return
    happens before the writer runs, so a caller reading the path
    unconditionally will see a stale or missing file. Check the exit first.
  * the pairs count can be BELOW the item count (items whose endpoints fail
    to parse or straddle nets are dropped), so ``KICAD_UNCONNECTED_PAIRS: 0``
    beside exit 4 is a legitimate state -- it must never be read as clean;
    the exit code carries the verdict, the pairs file only the targetable
    subset.
"""

from __future__ import annotations

#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['routing', 'combined'], 'kind': 'instrument'}

import _path  # noqa: F401  (#522: makes ../py_router importable)

import argparse
import json
import os
import subprocess
import sys
import tempfile

KICAD_CLI_CANDIDATES = [
    os.environ.get('KICAD_CLI', ''),
    '/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli',
    '/usr/bin/kicad-cli',
    '/usr/local/bin/kicad-cli',
    'kicad-cli',
]


def find_kicad_cli():
    for c in KICAD_CLI_CANDIDATES:
        if not c:
            continue
        if os.path.sep in c:
            if os.path.exists(c):
                return c
        else:
            from shutil import which
            w = which(c)
            if w:
                return w
    # Windows installs are versioned and not on PATH; newest version wins.
    if sys.platform == 'win32':
        import glob
        hits = sorted(glob.glob(r'C:\Program Files\KiCad\*\bin\kicad-cli.exe'))
        if hits:
            return hits[-1]
    return None


def kicad_unconnected(board_path: str, keep_json: str = None):
    """Run kicad-cli DRC with zone refill; return (count, items, err).

    count is None on failure (err says why). items are the raw
    ``unconnected_items`` dicts from the JSON report."""
    cli = find_kicad_cli()
    if cli is None:
        return None, [], 'kicad-cli not found (set KICAD_CLI)'
    out_path = keep_json
    tmp = None
    if out_path is None:
        tmp = tempfile.NamedTemporaryFile(suffix='.json', delete=False)
        out_path = tmp.name
        tmp.close()
    try:
        r = subprocess.run(
            [cli, 'pcb', 'drc', '--refill-zones', '--format', 'json',
             '-o', out_path, board_path],
            capture_output=True, text=True, timeout=600)
        if not os.path.exists(out_path) or os.path.getsize(out_path) == 0:
            return None, [], f'kicad-cli produced no report (rc={r.returncode}: ' \
                             f'{(r.stderr or r.stdout).strip()[:120]})'
        with open(out_path, encoding='utf-8') as f:
            data = json.load(f)
        items = data.get('unconnected_items', []) or []
        return len(items), items, None
    except subprocess.TimeoutExpired:
        return None, [], 'kicad-cli timed out'
    except Exception as e:
        return None, [], f'{type(e).__name__}: {e}'
    finally:
        if tmp is not None:
            try:
                os.unlink(out_path)
            except OSError:
                pass


def parse_pairs(items: list) -> list:
    """Parse raw DRC unconnected items into join specs [{net, a, b}] with
    kicad_oracle's own item parser (net + endpoint x/y/layer/kind) -- the
    same pairs the router's oracle recheck acts on. Pairs whose endpoints
    fail to parse or straddle nets are dropped, exactly as the oracle drops
    them."""
    from kicad_oracle import _parse_item
    pairs = []
    for u in items:
        its = u.get('items', [])
        if len(its) < 2:
            continue
        a = _parse_item(its[0])
        b = _parse_item(its[1])
        if a and b and a[0] == b[0]:
            pairs.append({'net': a[0],
                          'a': {'x': a[1], 'y': a[2], 'layer': a[3], 'kind': a[4]},
                          'b': {'x': b[1], 'y': b[2], 'layer': b[3], 'kind': b[4]}})
    return pairs


def write_pairs_json(board_path: str, items: list, out_path: str) -> int:
    """Write the parsed join work list to out_path; returns the pair count.
    Consumes the report THIS run already fetched -- the flag costs no second
    kicad-cli DRC."""
    pairs = parse_pairs(items)
    with open(out_path, 'w', encoding='utf-8') as f:
        json.dump({'board': board_path, 'count': len(pairs), 'pairs': pairs},
                  f, indent=2)
    return len(pairs)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('board')
    ap.add_argument('--items', action='store_true',
                    help='print each unconnected item pair')
    ap.add_argument('--json', default=None,
                    help='keep the full kicad-cli DRC report here')
    ap.add_argument('--pairs-json', default=None, metavar='PATH',
                    help='write the PARSED join pairs (net + both endpoints '
                         'with x/y/layer/kind) as a machine-readable work list')
    args = ap.parse_args()
    n, items, err = kicad_unconnected(args.board, keep_json=args.json)
    if n is None:
        print(f'KICAD_UNCONNECTED: ERR {err}')
        return 3
    print(f'KICAD_UNCONNECTED: {n}')
    if args.items:
        for u in items:
            parts = []
            for it in u.get('items', []):
                pos = it.get('pos', {})
                parts.append(f"{it.get('description', '?')[:60]} "
                             f"@({pos.get('x', 0):.2f},{pos.get('y', 0):.2f})")
            print('  ' + ' <-> '.join(parts))
    if args.pairs_json:
        np = write_pairs_json(args.board, items, args.pairs_json)
        print(f'KICAD_UNCONNECTED_PAIRS: {np} -> {args.pairs_json}')
    return 4 if n > 0 else 0


if __name__ == '__main__':
    import cli_banner; cli_banner.install()  # CMD/EXIT self-echo (run-5 c1)
    sys.exit(main())
