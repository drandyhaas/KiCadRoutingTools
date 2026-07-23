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

Prints ``KICAD_UNCONNECTED: <n>`` (or ``KICAD_UNCONNECTED: ERR <why>`` when
kicad-cli is unavailable/fails -- graders treat that as "no oracle", never 0).
Exit code is always 0; this is a grading tool, not a gate.
"""
from __future__ import annotations

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


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('board')
    ap.add_argument('--items', action='store_true',
                    help='print each unconnected item pair')
    ap.add_argument('--json', default=None,
                    help='keep the full kicad-cli DRC report here')
    args = ap.parse_args()
    n, items, err = kicad_unconnected(args.board, keep_json=args.json)
    if n is None:
        print(f'KICAD_UNCONNECTED: ERR {err}')
        return 0
    print(f'KICAD_UNCONNECTED: {n}')
    if args.items:
        for u in items:
            parts = []
            for it in u.get('items', []):
                pos = it.get('pos', {})
                parts.append(f"{it.get('description', '?')[:60]} "
                             f"@({pos.get('x', 0):.2f},{pos.get('y', 0):.2f})")
            print('  ' + ' <-> '.join(parts))
    return 0


if __name__ == '__main__':
    sys.exit(main())
