#!/usr/bin/env python3
"""Animate a routing run into a movie (issue #482).

Two modes:

* **Single trace** -- ``animate_route.py TRACE.json --board BOARD.kicad_pcb``:
  replay one ``*_routetrace.json`` (from ``KICAD_ROUTE_TRACE=1``) over the
  board substrate, drawing each segment/via as it is laid, ripped, restored.

* **Whole run** -- ``animate_route.py --run-dir RUNDIR [-o OUT.gif]``: build one
  movie spanning a whole stress run's step chain. The ``stepN_*.kicad_pcb``
  boards are cumulative (each contains all prior routing), so each step's new
  copper is revealed as the delta from the previous step -- which covers EVERY
  front-end (fanout, diff pairs, planes, signal, repair), not just the ones
  that emit a fine trace. Where a step DOES have a ``<step>_routetrace.json``
  (the route.py / route_diff.py steps), its fine per-copper events are spliced
  in so rips and restores animate; other steps reveal their delta in chunks.
  The substrate and end state are the final board.

New copper flashes white, reroutes/restores green, rips flash red on the frame
before they vanish. Output is a Pillow GIF (no ffmpeg); ``--png-dir`` also
dumps raw frames for external encoding.
"""
from __future__ import annotations

import argparse
import glob
import os
import re
import sys
from typing import Dict, List, Optional, Tuple

from route_trace import (load_trace, _Seg, _Via, seg_key_row, via_key_row)

_RIP = (255, 66, 66)        # ripped copper
_NEW = (250, 250, 250)      # freshly routed copper
_RESTORE = (86, 224, 96)    # rerouted / restored copper


def _add_color(event: str) -> Tuple[int, int, int]:
    e = (event or '').lower()
    if 'reroute' in e or 'restore' in e or 'rescue' in e:
        return _RESTORE
    return _NEW


def _board_rows(pcb, layers) -> Tuple[List[List], List[List]]:
    """Serialize a board's copper into trace-style rows (so it diffs against
    trace events and other boards by the same keys)."""
    li = {n: i for i, n in enumerate(layers)}
    lset = set(layers)
    seg_rows = [[round(s.start_x, 4), round(s.start_y, 4),
                 round(s.end_x, 4), round(s.end_y, 4),
                 round(s.width, 4), li.get(s.layer, 0)]
                for s in pcb.segments if s.layer in lset]
    via_rows = []
    for v in pcb.vias:
        vl = getattr(v, 'layers', None) or []
        a = li.get(vl[0], 0) if vl else 0
        b = li.get(vl[-1], len(layers) - 1) if vl else len(layers) - 1
        via_rows.append([round(v.x, 4), round(v.y, 4), round(v.size, 4),
                         round(v.drill, 4), a, b])
    return seg_rows, via_rows


class Movie:
    """Accumulates animation frames over a shared, growing copper state.

    ``live_s`` / ``live_v`` map a geometry key -> a drawable adapter; each
    frame renders the current live copper with an optional highlight for what
    just changed. Re-adding an already-present key is a no-op (so a step's
    finalize re-adding prior copper neither duplicates nor, with
    ``only_new``, flashes it)."""

    def __init__(self, renderer, layers, rip_hold: int = 2):
        self.r = renderer
        self.layers = layers
        self.rip_hold = rip_hold
        self.live_s: Dict[Tuple, _Seg] = {}
        self.live_v: Dict[Tuple, _Via] = {}
        self.frames: List = []

    def _frame(self, hl_s, hl_v, color, label):
        self.frames.append(self.r.frame(
            segments=list(self.live_s.values()), vias=list(self.live_v.values()),
            highlight_segments=hl_s, highlight_vias=hl_v,
            highlight_color=color, label=label))

    def snapshot(self, label):
        """A plain frame of the current state (no highlight)."""
        self.frames.append(self.r.frame(
            segments=list(self.live_s.values()), vias=list(self.live_v.values()),
            label=label))

    def add(self, seg_rows, via_rows, event, label, only_new=False):
        """Add copper and emit a frame highlighting what landed."""
        new_s, new_v = [], []
        for row in seg_rows:
            k = seg_key_row(row)
            fresh = k not in self.live_s
            self.live_s[k] = _Seg(row, self.layers)
            if fresh or not only_new:
                new_s.append(self.live_s[k])
        for row in via_rows:
            k = via_key_row(row)
            fresh = k not in self.live_v
            self.live_v[k] = _Via(row, self.layers)
            if fresh or not only_new:
                new_v.append(self.live_v[k])
        if new_s or new_v:
            self._frame(new_s, new_v, _add_color(event), label)

    def remove(self, seg_keys, via_keys, label, by=None):
        """Flash the doomed copper red (still present), then drop it."""
        hl_s = [self.live_s[k] for k in seg_keys if k in self.live_s]
        hl_v = [self.live_v[k] for k in via_keys if k in self.live_v]
        if not hl_s and not hl_v:
            return
        rlabel = label + (f"  (rip by {by})" if by else '  (rip)')
        for _ in range(max(1, self.rip_hold)):
            self._frame(hl_s, hl_v, _RIP, rlabel)
        for k in seg_keys:
            self.live_s.pop(k, None)
        for k in via_keys:
            self.live_v.pop(k, None)

    def play_trace(self, trace, label_prefix='', only_new=False):
        """Replay a fine per-copper trace's events."""
        events = trace.get('events') or []
        total = len(events)
        for i, ev in enumerate(events, 1):
            name = ev.get('net_name') or (f"net {ev['net']}" if 'net' in ev else '')
            event = ev.get('event', '')
            lbl = f"{label_prefix}{i}/{total} {event}" + (f"  {name}" if name else '')
            dks = [seg_key_row(r) for r in ev.get('del_s', ())]
            dkv = [via_key_row(r) for r in ev.get('del_v', ())]
            if dks or dkv:
                self.remove(dks, dkv, lbl, by=ev.get('by'))
            if ev.get('add_s') or ev.get('add_v'):
                self.add(ev.get('add_s', ()), ev.get('add_v', ()), event, lbl,
                         only_new=only_new)

    def reveal_delta(self, seg_rows, via_rows, label, chunks=6):
        """Coarse reveal: bring live state to exactly (seg_rows, via_rows),
        adding new copper in ``chunks`` batches and ripping vanished copper.
        Used for steps without a fine trace (fanout / planes / repair)."""
        want_s = {seg_key_row(r): r for r in seg_rows}
        want_v = {via_key_row(r): r for r in via_rows}
        gone_s = [k for k in self.live_s if k not in want_s]
        gone_v = [k for k in self.live_v if k not in want_v]
        if gone_s or gone_v:
            self.remove(gone_s, gone_v, label)
        add_s = [r for k, r in want_s.items() if k not in self.live_s]
        add_v = [r for k, r in want_v.items() if k not in self.live_v]
        if not add_s and not add_v:
            return
        n = max(1, chunks)
        per = max(1, (len(add_s) + n - 1) // n)
        vper = max(1, (len(add_v) + n - 1) // n)
        i = j = 0
        while i < len(add_s) or j < len(add_v):
            self.add(add_s[i:i + per], add_v[j:j + vper], 'route', label)
            i += per
            j += vper

    def reconcile_to(self, seg_rows, via_rows, label):
        """Force live state to exactly the given copper (silent trueup)."""
        want_s = {seg_key_row(r): r for r in seg_rows}
        want_v = {via_key_row(r): r for r in via_rows}
        changed = False
        for k in [k for k in self.live_s if k not in want_s]:
            self.live_s.pop(k, None); changed = True
        for k in [k for k in self.live_v if k not in want_v]:
            self.live_v.pop(k, None); changed = True
        for k, r in want_s.items():
            if k not in self.live_s:
                self.live_s[k] = _Seg(r, self.layers); changed = True
        for k, r in want_v.items():
            if k not in self.live_v:
                self.live_v[k] = _Via(r, self.layers); changed = True
        if changed:
            self.snapshot(label)


# ---------------------------------------------------------------------------
# Drivers
# ---------------------------------------------------------------------------
def _renderer(board_path, layers, size, ss, alpha):
    from kicad_parser import parse_kicad_pcb
    from route_render import BoardRenderer
    pcb = parse_kicad_pcb(board_path)
    lyrs = layers or list(pcb.board_info.copper_layers)
    return BoardRenderer(pcb, size=size, supersample=ss, layers=lyrs,
                         layer_alpha=alpha), lyrs


def build_single(trace, board_path, size, ss, alpha, rip_hold):
    r, layers = _renderer(board_path, trace.get('layers'), size, ss, alpha)
    m = Movie(r, layers, rip_hold=rip_hold)
    m.snapshot(f"0/{len(trace.get('events') or [])}  (input)")
    m.play_trace(trace)
    m.snapshot("(routed)")
    return m.frames


def _step_sort_key(path: str):
    b = os.path.basename(path)
    m = re.match(r'step(\d+)', b)
    return (int(m.group(1)) if m else 9999, b)


def discover_steps(run_dir: str) -> Tuple[List[Tuple[str, str, Optional[str]]], Optional[str]]:
    """Return [(label, board_path, trace_path|None), ...] in chain order, and
    the final board path. A step's trace is ``<board_basename>_routetrace.json``
    when present."""
    boards = [p for p in glob.glob(os.path.join(run_dir, 'step*.kicad_pcb'))]
    boards.sort(key=_step_sort_key)
    steps = []
    for b in boards:
        base = os.path.splitext(b)[0]
        tr = base + '_routetrace.json'
        label = os.path.splitext(os.path.basename(b))[0]
        steps.append((label, b, tr if os.path.exists(tr) else None))
    final = boards[-1] if boards else None
    return steps, final


def build_run(run_dir, size, ss, alpha, rip_hold, chunks):
    from kicad_parser import parse_kicad_pcb
    steps, final = discover_steps(run_dir)
    if not final:
        return []
    r, layers = _renderer(final, None, size, ss, alpha)
    m = Movie(r, layers, rip_hold=rip_hold)
    m.snapshot("input")
    for label, board, trace_path in steps:
        pcb = parse_kicad_pcb(board)
        seg_rows, via_rows = _board_rows(pcb, layers)
        if trace_path:
            try:
                m.play_trace(load_trace(trace_path), label_prefix=f"{label}: ",
                             only_new=True)
                m.reconcile_to(seg_rows, via_rows, label)   # trueup to step board
                continue
            except Exception as e:
                print(f"animate_route: trace {trace_path} failed ({e}); "
                      f"revealing delta", file=sys.stderr)
        m.reveal_delta(seg_rows, via_rows, label, chunks=chunks)
    # final trueup (in case the graded final differs from the last step board)
    fpcb = parse_kicad_pcb(final)
    m.reconcile_to(*_board_rows(fpcb, layers), "routed")
    return m.frames


def save_gif(frames, out, fps, end_hold, png_dir=None):
    if not frames:
        print("animate_route: no frames", file=sys.stderr)
        return False
    dur = max(20, int(1000 / max(0.1, fps)))
    hold = [frames[-1]] * max(1, int(end_hold * fps))
    frames[0].save(out, save_all=True, append_images=frames[1:] + hold,
                   duration=dur, loop=0, optimize=False)
    print(f"animate_route: wrote {out} ({len(frames)} frames, {dur}ms each, "
          f"{frames[0].size[0]}x{frames[0].size[1]})")
    if png_dir:
        os.makedirs(png_dir, exist_ok=True)
        for i, fr in enumerate(frames):
            fr.save(os.path.join(png_dir, f'frame_{i:05d}.png'))
        print(f"animate_route: dumped {len(frames)} PNG frames to {png_dir}")
    return True


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('trace', nargs='?', help='*_routetrace.json (single-trace mode)')
    ap.add_argument('--run-dir', default=None, help='stress run dir (whole-run mode)')
    ap.add_argument('--board', default=None, help='board .kicad_pcb substrate (single-trace mode)')
    ap.add_argument('-o', '--output', default=None, help='output .gif')
    ap.add_argument('--size', type=int, default=1000)
    ap.add_argument('--supersample', type=int, default=1)
    ap.add_argument('--layer-alpha', type=int, default=150)
    ap.add_argument('--fps', type=float, default=6.0)
    ap.add_argument('--rip-hold', type=int, default=2)
    ap.add_argument('--end-hold', type=float, default=1.5)
    ap.add_argument('--chunks', type=int, default=6, help='reveal batches per untraced step')
    ap.add_argument('--png-dir', default=None)
    args = ap.parse_args()

    if args.run_dir:
        frames = build_run(args.run_dir, args.size, args.supersample,
                           args.layer_alpha, args.rip_hold, args.chunks)
        out = args.output or os.path.join(args.run_dir, 'routing.gif')
    else:
        if not args.trace:
            print("animate_route: give a TRACE.json or --run-dir", file=sys.stderr)
            return 1
        trace = load_trace(args.trace)
        board = args.board
        if board is None:
            base = args.trace
            for suf in ('_routetrace.json', '.json'):
                if base.endswith(suf):
                    base = base[:-len(suf)]
                    break
            board = base + '.kicad_pcb'
        if not os.path.exists(board):
            print(f"animate_route: board not found: {board} (pass --board)", file=sys.stderr)
            return 1
        frames = build_single(trace, board, args.size, args.supersample,
                              args.layer_alpha, args.rip_hold)
        out = args.output or (os.path.splitext(args.trace)[0] + '.gif')

    return 0 if save_gif(frames, out, args.fps, args.end_hold, args.png_dir) else 1


if __name__ == '__main__':
    sys.exit(main())
