#!/usr/bin/env python3
"""Animate a routing run from a route trace (issue #482).

Replays a ``*_routetrace.json`` (produced by running the router with
``KICAD_ROUTE_TRACE=1``) over the board's static substrate, drawing each
segment and via as it is laid down, ripped up, and restored -- in routing
order. New copper flashes bright, rips flash red (shown on the frame before
they vanish), reroutes/restores flash green. Output is an animated GIF
assembled natively by Pillow (no ffmpeg needed); ``--png-dir`` also dumps the
raw frames for external encoding.

Usage:
    # 1. record a trace (default-off flag)
    KICAD_ROUTE_TRACE=1 python3 route.py IN.kicad_pcb OUT.kicad_pcb ...
    # 2. render the movie (needs the ORIGINAL/unrouted or routed board for the
    #    substrate -- pads, outline, zones)
    python3 animate_route.py OUT_routetrace.json --board IN.kicad_pcb \
        -o routing.gif [--size 1000] [--fps 6] [--supersample 1]

If ``--board`` is omitted it defaults to the trace basename + ``.kicad_pcb``.
"""
from __future__ import annotations

import argparse
import os
import sys
from typing import Dict, List, Tuple

from route_trace import (load_trace, _Seg, _Via, seg_key_row, via_key_row)

_RIP = (255, 66, 66)        # ripped copper
_NEW = (250, 250, 250)      # freshly routed copper
_RESTORE = (86, 224, 96)    # rerouted / restored copper


def _add_color(event: str) -> Tuple[int, int, int]:
    e = (event or '').lower()
    if 'reroute' in e or 'restore' in e or 'rescue' in e:
        return _RESTORE
    return _NEW


def build_frames(trace: Dict, board_path: str, size: int, supersample: int,
                 layer_alpha: int, rip_hold: int) -> List:
    from kicad_parser import parse_kicad_pcb
    from route_render import BoardRenderer

    pcb = parse_kicad_pcb(board_path)
    layers = trace.get('layers') or list(pcb.board_info.copper_layers)
    r = BoardRenderer(pcb, size=size, supersample=supersample,
                      layers=layers, layer_alpha=layer_alpha)

    live_s: Dict[Tuple, _Seg] = {}
    live_v: Dict[Tuple, _Via] = {}
    frames: List = []
    events = trace.get('events') or []
    total = len(events)

    # opening frame: bare substrate
    frames.append(r.frame(segments=[], vias=[], label='0 / %d  (input)' % total))

    for i, ev in enumerate(events, 1):
        seq = ev.get('seq', i)
        name = ev.get('net_name') or (f"net {ev['net']}" if 'net' in ev else '')
        event = ev.get('event', '')
        add_s = [_Seg(row, layers) for row in ev.get('add_s', ())]
        add_v = [_Via(row, layers) for row in ev.get('add_v', ())]
        del_s_keys = [seg_key_row(row) for row in ev.get('del_s', ())]
        del_v_keys = [via_key_row(row) for row in ev.get('del_v', ())]

        label = f"{i}/{total}  [{seq}] {event}" + (f"  {name}" if name else '')

        # 1) rip frame: show doomed copper red BEFORE removing it
        if del_s_keys or del_v_keys:
            hl_s = [live_s[k] for k in del_s_keys if k in live_s]
            hl_v = [live_v[k] for k in del_v_keys if k in live_v]
            rip_label = label + (f"  (rip by {ev['by']})" if ev.get('by') else '  (rip)')
            rip_frame = r.frame(segments=list(live_s.values()),
                                vias=list(live_v.values()),
                                highlight_segments=hl_s, highlight_vias=hl_v,
                                highlight_color=_RIP, label=rip_label)
            for _ in range(max(1, rip_hold)):
                frames.append(rip_frame)
            for k in del_s_keys:
                live_s.pop(k, None)
            for k in del_v_keys:
                live_v.pop(k, None)

        # 2) apply additions
        for row, s in zip(ev.get('add_s', ()), add_s):
            live_s[seg_key_row(row)] = s
        for row, v in zip(ev.get('add_v', ()), add_v):
            live_v[via_key_row(row)] = v

        # 3) state frame: new copper highlighted (skip if this event only ripped)
        if add_s or add_v:
            frames.append(r.frame(segments=list(live_s.values()),
                                  vias=list(live_v.values()),
                                  highlight_segments=add_s, highlight_vias=add_v,
                                  highlight_color=_add_color(event), label=label))
        elif not (del_s_keys or del_v_keys):
            frames.append(r.frame(segments=list(live_s.values()),
                                  vias=list(live_v.values()), label=label))

    # closing frame: settled board, no highlight
    frames.append(r.frame(segments=list(live_s.values()),
                          vias=list(live_v.values()),
                          label=f'{total}/{total}  (routed)'))
    return frames


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('trace', help='*_routetrace.json from KICAD_ROUTE_TRACE=1')
    ap.add_argument('--board', default=None,
                    help='board .kicad_pcb for the substrate (default: trace '
                         'name minus _routetrace + .kicad_pcb)')
    ap.add_argument('-o', '--output', default=None, help='output .gif (default: alongside trace)')
    ap.add_argument('--size', type=int, default=1000, help='longest dimension px (default 1000)')
    ap.add_argument('--supersample', type=int, default=1, help='anti-alias factor (default 1 for speed)')
    ap.add_argument('--layer-alpha', type=int, default=150)
    ap.add_argument('--fps', type=float, default=6.0, help='frames per second (default 6)')
    ap.add_argument('--rip-hold', type=int, default=2, help='frames to hold a rip red (default 2)')
    ap.add_argument('--end-hold', type=float, default=1.5, help='seconds to hold the final frame (default 1.5)')
    ap.add_argument('--png-dir', default=None, help='also dump raw PNG frames here (for ffmpeg)')
    args = ap.parse_args()

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

    frames = build_frames(trace, board, args.size, args.supersample,
                          args.layer_alpha, args.rip_hold)
    if not frames:
        print("animate_route: no frames (empty trace)", file=sys.stderr)
        return 1

    out = args.output or (os.path.splitext(args.trace)[0] + '.gif')
    dur = max(20, int(1000 / max(0.1, args.fps)))
    hold = [frames[-1]] * max(1, int(args.end_hold * args.fps))
    frames[0].save(out, save_all=True, append_images=frames[1:] + hold,
                   duration=dur, loop=0, optimize=False)
    print(f"animate_route: wrote {out} ({len(frames)} frames, {dur}ms each, "
          f"{frames[0].size[0]}x{frames[0].size[1]})")

    if args.png_dir:
        os.makedirs(args.png_dir, exist_ok=True)
        for i, fr in enumerate(frames):
            fr.save(os.path.join(args.png_dir, f'frame_{i:05d}.png'))
        print(f"animate_route: dumped {len(frames)} PNG frames to {args.png_dir}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
