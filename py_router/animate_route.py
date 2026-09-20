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

#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['routing'], 'kind': 'instrument'}

import argparse
import glob
import os
import re
import sys
from typing import Dict, List, Optional, Tuple

from route_trace import (load_trace, _Seg, _Via, seg_key_row, via_key_row)

from render_theme import DARK as _THEME_DARK

#: Aliases onto the dark theme, kept as names for out-of-repo callers. The
#: EVENT colours are the two #946 opened on: `_RIP` and `_RESTORE` differ
#: almost entirely in hue, along the one axis red-green colour blindness
#: removes -- 233 apart in RGB and 76 under a Vienot transform. #1013 moves
#: `_RESTORE` to cyan and gives the rip a dash; #1011 changes no value.
_RIP = _THEME_DARK.rgb('event_ripped')
_NEW = _THEME_DARK.rgb('event_new')
_RESTORE = _THEME_DARK.rgb('event_restored')


def _add_color(event: str, theme=None) -> Tuple[int, int, int]:
    """Which event colour an 'add' carries. Resolves a ROLE, not an RGB, so a
    themed movie flashes in its own palette."""
    th = theme or _THEME_DARK
    e = (event or '').lower()
    if 'reroute' in e or 'restore' in e or 'rescue' in e:
        return th.rgb('event_restored')
    return th.rgb('event_new')


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

    def __init__(self, renderer, layers, rip_hold: int = 2, theme=None):
        self.r = renderer
        # Off the renderer by default, so no call site has to learn about it.
        self.theme = theme or getattr(renderer, 'theme', _THEME_DARK)
        # #1014: which event roles this run has ACTUALLY produced, so far. The
        # key draws only these -- #896's rule, ported from
        # `render_placement.draw_legend`: a legend listing a mark the picture
        # does not carry teaches the reader to look for something that is not
        # there. A movie of a clean run rips nothing and must not advertise a
        # rip colour.
        #
        # It grows FRAME BY FRAME rather than being computed once over the
        # whole film, and that is the honest reading: at frame 40 the key says
        # what has happened by frame 40, never what is coming.
        self.seen_events = []
        self.layers = layers
        self.rip_hold = rip_hold
        self.live_s: Dict[Tuple, _Seg] = {}
        self.live_v: Dict[Tuple, _Via] = {}
        self.frames: List = []
        # Plane fills revealed so far (dynamic_zones): a plane pours in on the
        # frame its taps first land, rather than being an always-on backdrop.
        self.zone_avail = renderer.zone_net_ids() if getattr(renderer, 'dynamic_zones', False) else set()
        self.revealed_zones: set = set()

    def reveal_zone(self, net_id) -> None:
        if net_id in self.zone_avail:
            self.revealed_zones.add(net_id)

    def _note_event(self, role):
        if role not in self.seen_events:
            self.seen_events.append(role)

    def _key_overlay(self):
        """The in-frame key, drawn through `frame(overlays=...)`.

        That seam draws at supersampled resolution above copper and below the
        label, and -- the reason this is cheap -- it costs NO FRAME GEOMETRY.
        #946 ranked the key as step 8, after several layout changes, because it
        ranked by where a key APPEARS rather than by how it is DRAWN.
        """
        if not self.seen_events:
            return None
        from render_chrome import event_rows, draw_key
        rows = event_rows(self.theme, seen=self.seen_events)

        def _draw(d, r):
            ss = max(1, int(getattr(r, 'ss', 1)))
            draw_key(d, rows, width=r.W * ss, height=r.H * ss,
                     theme=self.theme, corner='bl', pad_scale=ss)
        return _draw

    def _frame(self, hl_s, hl_v, color, label, mark='solid'):
        ov = self._key_overlay()
        self.frames.append(self.r.frame(
            segments=list(self.live_s.values()), vias=list(self.live_v.values()),
            highlight_segments=hl_s, highlight_vias=hl_v,
            highlight_color=color, highlight_mark=mark, label=label,
            zone_net_ids=self.revealed_zones,
            overlays=[ov] if ov else None))

    def snapshot(self, label):
        """A plain frame of the current state (no highlight)."""
        self.frames.append(self.r.frame(
            segments=list(self.live_s.values()), vias=list(self.live_v.values()),
            label=label, zone_net_ids=self.revealed_zones))

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
            role = ('event_restored'
                    if _add_color(event, self.theme)
                    == self.theme.rgb('event_restored') else 'event_new')
            self._note_event(role)
            self._frame(new_s, new_v, _add_color(event, self.theme), label)

    def remove(self, seg_keys, via_keys, label, by=None):
        """Flash the doomed copper red (still present), then drop it."""
        hl_s = [self.live_s[k] for k in seg_keys if k in self.live_s]
        hl_v = [self.live_v[k] for k in via_keys if k in self.live_v]
        if not hl_s and not hl_v:
            return
        rlabel = label + (f"  (rip by {by})" if by else '  (rip)')
        for _ in range(max(1, self.rip_hold)):
            self._note_event('event_ripped')
            self._frame(hl_s, hl_v, self.theme.rgb('event_ripped'), rlabel,
                        mark=self.theme.mark('event_ripped'))
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
            # reveal this net's plane fill on the plane-copper events that add it
            if 'net' in ev and event in ('plane-tap', 'plane-join', 'plane-fill'):
                self.reveal_zone(ev['net'])
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
def _renderer(board_path, layers, size, ss, alpha, dynamic_zones=False,
              theme=None):
    from kicad_parser import parse_kicad_pcb
    from route_render import BoardRenderer
    pcb = parse_kicad_pcb(board_path)
    lyrs = layers or list(pcb.board_info.copper_layers)
    return BoardRenderer(pcb, size=size, supersample=ss, layers=lyrs,
                         layer_alpha=alpha, dynamic_zones=dynamic_zones,
                         theme=theme), lyrs


def build_single(trace, board_path, size, ss, alpha, rip_hold):
    r, layers = _renderer(board_path, trace.get('layers'), size, ss, alpha)
    m = Movie(r, layers, rip_hold=rip_hold)
    m.snapshot(f"0/{len(trace.get('events') or [])}  (input)")
    m.play_trace(trace)
    m.snapshot("(routed)")
    return m.frames


def _step_sort_key(path: str):
    # Match the step number anywhere in the name: chains use both `step6_route`
    # and `board_step6_route` conventions; sub-steps (step2a/2b) sort by number
    # then name. A sub-letter breaks ties within a step.
    b = os.path.basename(path)
    m = re.search(r'step\s*(\d+)([a-z]*)', b, re.I)
    return (int(m.group(1)) if m else 9999, m.group(2) if m else '', b)


def discover_steps(run_dir: str) -> Tuple[List[Tuple[str, str, Optional[str]]], Optional[str]]:
    """Return [(label, board_path, trace_path|None), ...] in chain order, and
    the final board path. A step's trace is ``<board_basename>_routetrace.json``
    when present.

    Ordering: prefer ``stepN`` in the filename (``step6_route`` or
    ``board_step6_route``); when no board is step-numbered (chains that name
    outputs semantically -- ``fanout`` / ``diff_groupA`` / ``planes`` /
    ``final_board``) fall back to write-time (mtime) order, which is the order
    the chain produced them. The final board is an explicit ``final*`` if
    present, else the last in order."""
    all_pcb = glob.glob(os.path.join(run_dir, '*.kicad_pcb'))
    stepped = [p for p in all_pcb if re.search(r'step\s*\d', os.path.basename(p), re.I)]
    if stepped:
        boards = sorted(stepped, key=_step_sort_key)
        # A semantically-named final board (``final_board.kicad_pcb``) must not
        # be dropped just because step-numbered boards exist -- the "explicit
        # final* wins" rule below can only pick from ``boards`` (#513 item 10:
        # a bare render_run.py rendered the second-to-last board as final).
        finals = [p for p in all_pcb if p not in stepped
                  and 'final' in os.path.basename(p).lower()]
        boards += sorted(finals, key=lambda p: (os.path.getmtime(p), os.path.basename(p)))
    else:
        boards = sorted(all_pcb, key=lambda p: (os.path.getmtime(p), os.path.basename(p)))
    steps = []
    for b in boards:
        base = os.path.splitext(b)[0]
        tr = base + '_routetrace.json'
        label = os.path.splitext(os.path.basename(b))[0]
        steps.append((label, b, tr if os.path.exists(tr) else None))
    final = boards[-1] if boards else None
    for b in boards:                      # an explicit final_board wins
        if 'final' in os.path.basename(b).lower():
            final = b
    return steps, final


def steps_for_boards(boards: List[str]) -> List[Tuple[str, str, Optional[str]]]:
    """[(label, board, trace|None), ...] for an EXPLICIT, already-ordered board
    list (make_movie.py's board-sequence mode, GUI per-step snapshots). Same
    shape discover_steps returns, minus the discovery/ordering."""
    steps = []
    for b in boards:
        tr = os.path.splitext(b)[0] + '_routetrace.json'
        steps.append((os.path.splitext(os.path.basename(b))[0], b,
                      tr if os.path.exists(tr) else None))
    return steps


def build_run(run_dir, size, ss, alpha, rip_hold, chunks):
    steps, final = discover_steps(run_dir)
    if not final:
        return []
    return build_boards(steps, final, size, ss, alpha, rip_hold, chunks)


def build_boards(steps, final, size, ss, alpha, rip_hold, chunks, stage=None,
                 marks=None, theme=None, layout=None, aspect=None,
                 geom_out=None):
    """Frames for a chain given as [(label, board, trace|None), ...] plus the
    final board. ``build_run`` is this with the chain discovered from a run dir.

    ``stage`` (movie_camera.Stage, #431) adds a camera and animates FOOTPRINT
    motion for placement rounds. With ``stage=None`` -- every existing caller --
    the three hooks below are falsy branches and the routing movie is unchanged.

    ``marks``, when a list is passed, collects ``(label, board, first, last)``
    frame indices per step -- what a composer needs to caption, badge or splice
    a beat without re-deriving where it landed. Rendering one pass and reading
    the boundaries back is the only way to keep ONE scale across the whole
    film; a per-segment call restarts from an empty board and re-reveals all
    the copper, which reads as the board redrawing itself between beats.
    """
    from kicad_parser import parse_kicad_pcb
    if not final:
        return []
    # dynamic_zones: plane pours reveal as each plane is created, rather than
    # sitting under every frame from the start. It is ALSO what lets a Stage
    # animate part motion: with it, frame() draws pads per frame from
    # renderer.pcb, so re-pointing that attribute moves the parts.
    r, layers = _renderer(final, None, size, ss, alpha, dynamic_zones=True,
                          theme=theme)
    # #1018. The frame shape is decided ONCE, here, before any frame exists --
    # and only when a layout was actually asked for. 'legacy' (the default) is
    # left completely alone so every existing movie stays bit-for-bit what it
    # was, which is the same posture KICAD_MOVIE_CAMERA takes.
    #
    # `geom_out` follows the idiom `marks` already established in this
    # signature: when a list is passed, it collects what a composer needs,
    # without changing the return type.
    # ALWAYS planned, including 'legacy'. The even-forcing is a FIX, not a
    # layout feature: `_write_mp4` crops `a.shape[0] & ~1` AND
    # `a.shape[1] & ~1`, so an odd frame has always been losing that row or
    # column -- silently, in every movie this repo has written. Planning
    # legacy too means the frame is even BEFORE the encoder, so nothing is
    # cropped away.
    #
    # DISCLOSED: on a board whose aspect gives an odd dimension (most of them:
    # routed_output at size 500 is 500x309) the legacy frame is now 1 px
    # shorter or narrower than it used to be. That pixel was being thrown away
    # by the encoder anyway; the difference is that now the picture knows.
    if True:
        import frame_layout
        _g = frame_layout.plan_frame(
            r.pcb.board_info.board_bounds, layout=layout or 'legacy',
            ratio=frame_layout.parse_ratio(aspect), size=size,
            panel=False, legacy_size=(r.W, r.H))
        # Only when the layout genuinely MOVES the board box. On 'legacy' the
        # box is the renderer's own size evened, and the evening is applied by
        # cropping the composed frame instead -- because
        # `tests/test_431_placement_movie.py:92-121` pins exactly ONE
        # `set_view` on the no-stage path, and that assertion is this phase's
        # own falsifier: if the layout work needs a second aim, the layout work
        # is wrong.
        moved = (_g.board.w, _g.board.h) != (r.W, r.H)
        if _g.layout != 'legacy' and moved:
            r.set_canvas(_g.board.w, _g.board.h)
        if geom_out is not None:
            geom_out.append(_g)
    m = Movie(r, layers, rip_hold=rip_hold)
    if stage is not None:
        stage.attach(m, r, layers)
    m.snapshot("input")
    for _step in steps:
        # 4th element (optional, back-compatible): 'revert' undoes a beat with
        # the SILENT trueup instead of reveal_delta -- which would flash the
        # copper red and label it "(rip)". Nothing was ripped; an attempt was
        # not kept, and the two read completely differently in a film.
        label, board, trace_path = _step[0], _step[1], _step[2]
        mode = _step[3] if len(_step) > 3 else None
        _first = len(m.frames)
        pcb = parse_kicad_pcb(board)
        seg_rows, via_rows = _board_rows(pcb, layers)
        if mode == 'revert':
            if stage is not None:
                r.pcb = pcb
            m.reconcile_to(seg_rows, via_rows, label)
            if len(m.frames) == _first:
                # An attempt that only MOVED parts changes no copper, so the
                # silent trueup stays silent and the undo is invisible. The
                # parts still went back; show that they did.
                m.snapshot(label)
            if marks is not None:
                marks.append((label, board, _first, len(m.frames)))
            continue
        if stage is not None:
            # The renderer draws footprints from THIS board from here on.
            r.pcb = pcb
            if stage.enter_step(label, board, pcb, seg_rows, via_rows):
                if marks is not None:
                    marks.append((label, board, _first, len(m.frames)))
                continue        # a placement round; the stage emitted its frames
        # Reveal any plane whose pour exists on this step board but had no fine
        # plane-tap event (untraced plane step) so its fill still appears here.
        step_zone_nets = {z.net_id for z in (getattr(pcb, 'zones', None) or [])
                          if z.net_id and len(z.polygon) >= 3}
        if trace_path:
            try:
                m.play_trace(load_trace(trace_path), label_prefix=f"{label}: ",
                             only_new=True)
                for _nid in step_zone_nets:
                    m.reveal_zone(_nid)
                m.reconcile_to(seg_rows, via_rows, label)   # trueup to step board
                if marks is not None:
                    marks.append((label, board, _first, len(m.frames)))
                continue
            except Exception as e:
                print(f"animate_route: trace {trace_path} failed ({e}); "
                      f"revealing delta", file=sys.stderr)
        for _nid in step_zone_nets:      # untraced plane step: reveal its pours
            m.reveal_zone(_nid)
        m.reveal_delta(seg_rows, via_rows, label, chunks=chunks)
        if marks is not None:
            marks.append((label, board, _first, len(m.frames)))
    # final trueup (in case the graded final differs from the last step board)
    fpcb = parse_kicad_pcb(final)
    if stage is not None:
        r.pcb = fpcb
    for _z in (getattr(fpcb, 'zones', None) or []):   # ensure every pour shows
        m.reveal_zone(_z.net_id)
    m.reconcile_to(*_board_rows(fpcb, layers), "routed")
    if stage is not None:
        stage.outro()
    # #1018: the board was rendered into its PLANNED BOX; the frame is the
    # planned FRAME. Composing here rather than leaving the box as the frame is
    # what makes the size claim real -- the rail, the panel and the foot exist
    # as reserved ground from this commit, and #1019/#1020/#1021 fill them.
    #
    # In place, so peak memory stays about two frames rather than twice the
    # movie: the same reason `movie_panels.compose_two_panel` does it that way.
    if geom_out:
        _compose_into_frame(m.frames, geom_out[0], r)
    return m.frames


def _write_mp4(frames, out, fps) -> bool:
    """H.264 mp4 via imageio-ffmpeg (much smaller than GIF, full color, plays
    everywhere). Returns False if imageio/ffmpeg isn't available so the caller
    can fall back to GIF."""
    try:
        import numpy as np
        import imageio.v2 as imageio
    except Exception as e:
        # SAY SO. The encode-failure branch below prints and this one did not,
        # so a missing imageio-ffmpeg silently produced a .gif where the caller
        # asked for .mp4 -- the only trace being a `wrote ...` line with a
        # different extension than the one requested. Two branches, one
        # consequence, and only one of them was audible.
        print(f"animate_route: mp4 unavailable ({e}); falling back to GIF. "
              f"`pip install imageio imageio-ffmpeg` for mp4.", file=sys.stderr)
        return False
    try:
        # yuv420p (broadly playable: browsers, QuickTime, Slack, social) needs
        # even dimensions; macro_block_size=1 stops imageio from padding to 16,
        # and we crop each frame to even W/H ourselves (drops at most 1 px).
        w = imageio.get_writer(out, fps=max(1, round(fps)), codec='libx264',
                               quality=8, macro_block_size=1, pixelformat='yuv420p')
        for fr in frames:
            a = np.asarray(fr.convert('RGB'))
            h, wd = a.shape[0] & ~1, a.shape[1] & ~1
            w.append_data(a[:h, :wd])
        w.close()
        return True
    except Exception as e:
        print(f"animate_route: mp4 encode failed ({e}); falling back to GIF",
              file=sys.stderr)
        return False


def _png_info(meta):
    """A Pillow PngInfo for one frame's metadata block (#887)."""
    from PIL import PngImagePlugin
    info = PngImagePlugin.PngInfo()
    for k, v in (meta or {}).items():
        info.add_text(str(k), '' if v is None else str(v))
    return info


def _compose_into_frame(frames, geom, r):
    """Fit each board-box frame into its planned frame, IN PLACE.

    Two cases, and the first is the common one:

    * the frame IS the board box, to within the even-forcing -- so the frame is
      CROPPED to the planned size. That is exactly what `_write_mp4` already
      did with `& ~1`, made explicit and applied to the GIF path too, where it
      was not happening at all;
    * the frame is larger, because the layout reserved a rail, a panel or a
      foot -- so the board is pasted into its box on a frame-sized canvas, and
      the reserved regions are ground until #1019/#1020/#1021 fill them.
    """
    from PIL import Image
    th = getattr(r, 'theme', None)
    bg = th.rgb('ground') if th is not None else (14, 16, 18)
    W, H = geom.frame.w, geom.frame.h
    for i in range(len(frames)):
        f = frames[i]
        if f.size == (W, H):
            continue
        if f.width >= W and f.height >= H and geom.board.x == 0                 and geom.board.y == 0 and geom.panel is None:
            frames[i] = f.crop((0, 0, W, H))
            continue
        canvas = Image.new('RGB', (W, H), bg)
        canvas.paste(f, (geom.board.x, geom.board.y))
        frames[i] = canvas


def _uniform_or_pad(frames):
    """Every frame at the first frame's size, letterboxed rather than squashed.

    Returns `frames` unchanged when they already agree, so the common path
    allocates nothing.
    """
    try:
        import frame_layout
        frame_layout.assert_frames_uniform([f.size for f in frames])
        return frames
    except Exception as exc:                                    # noqa: BLE001
        if 'frame_layout' not in str(type(exc)) and not isinstance(exc, ValueError):
            return frames
    from PIL import Image
    W, H = frames[0].size
    print('animate_route: MIXED FRAME SIZES -- %s' % (
        [f.size for f in frames if f.size != (W, H)][:3],), file=sys.stderr)
    print('animate_route: padding every frame to %dx%d. Pillow would NOT have '
          'raised: it writes a valid GIF in which every later frame has been '
          'silently resized to the first.' % (W, H), file=sys.stderr)
    out = []
    for f in frames:
        if f.size == (W, H):
            out.append(f)
            continue
        pad = Image.new(f.mode, (W, H), (0, 0, 0))
        pad.paste(f, ((W - f.width) // 2, (H - f.height) // 2))
        out.append(pad)
    return out


def save_movie(frames, out, fps, end_hold, png_dir=None, frame_meta=None):
    """Write the frames to ``out``. Format follows the extension: `.mp4`
    (imageio-ffmpeg; falls back to a sibling `.gif` if unavailable) or `.gif`
    (native Pillow, no dependency).

    ``frame_meta`` (#887), when given, is one dict per frame written into the
    dumped PNGs' text chunks. It is indexed against ``frames``, NOT against
    the ``seq`` built below: ``seq`` appends the end-hold repeats, while the
    PNG loop iterates ``frames``. A short list raises rather than silently
    misattributing every frame after the gap.

    Only the PNG dump carries it. The .mp4 hands numpy arrays to imageio and
    Pillow's GIF writer has no per-frame text channel, so there is nowhere
    else for it to go."""
    if frame_meta is not None and len(frame_meta) != len(frames):
        raise ValueError('save_movie: frame_meta has %d entries for %d '
                         'frames' % (len(frame_meta), len(frames)))
    if not frames:
        print("animate_route: no frames", file=sys.stderr)
        return False
    # #946/#1018. THE choke point: make_movie, make_film.build_film,
    # animate_fanout_clearance.render_gif and tests/stress/render_run.py all
    # arrive here, so this is the one place a mixed-size film can be caught.
    #
    # It REPORTS AND PADS; it does not raise. Aborting a routing run for a
    # cosmetic reason is something this repo refuses elsewhere too
    # (`movie_panels._finite` coerces a mistyped tuning value rather than
    # taking the movie down), and all three of today's outcomes are worse than
    # a pad: Pillow silently resizes every later frame to the first, _write_mp4
    # fails loudly and falls back to the GIF that then absorbs it, and nothing
    # anywhere says a word. After this the film is produced, the defect is
    # AUDIBLE, and the distortion is a letterbox rather than a squash.
    frames = _uniform_or_pad(frames)
    hold = [frames[-1]] * max(1, int(end_hold * fps))
    seq = frames + hold
    ext = os.path.splitext(out)[1].lower()
    if ext == '.mp4' and _write_mp4(seq, out, fps):
        print(f"animate_route: wrote {out} ({len(frames)} frames @ {fps:g}fps, "
              f"{frames[0].size[0]}x{frames[0].size[1]}, h264)")
    else:
        if ext == '.mp4':
            out = os.path.splitext(out)[0] + '.gif'
        dur = max(20, int(1000 / max(0.1, fps)))
        frames[0].save(out, save_all=True, append_images=seq[1:],
                       duration=dur, loop=0, optimize=False)
        # Count what LANDED, not what was handed to the encoder. Pillow's GIF
        # writer collapses runs of byte-identical frames into one frame with an
        # accumulated duration, and this film is full of such runs by
        # construction -- card holds and the end hold are literal repeats of a
        # single Image. So `len(frames)` overstates the file: one run printed
        # 377 and wrote 348, and the gap was only found by counting the
        # delivered GIF by hand. A number nobody can reconcile against the
        # artifact is worse than no number.
        _n = len(frames)
        try:
            from PIL import Image as _PILImage, ImageSequence
            with _PILImage.open(out) as _chk:
                _n = sum(1 for _ in ImageSequence.Iterator(_chk))
        except Exception:                                       # noqa: BLE001
            pass
        print(f"animate_route: wrote {out} ({_n} frames, {dur}ms each, "
              f"{frames[0].size[0]}x{frames[0].size[1]})")
    if png_dir:
        os.makedirs(png_dir, exist_ok=True)
        for i, fr in enumerate(frames):
            # pnginfo=None is Pillow's own PNG default, so the
            # no-metadata path is byte-for-byte what it always was.
            fr.save(os.path.join(png_dir, f'frame_{i:05d}.png'),
                    pnginfo=(_png_info(frame_meta[i]) if frame_meta
                             else None))
        print(f"animate_route: dumped {len(frames)} PNG frames to {png_dir}")
    return True


# Back-compat alias (render_run.py and older callers).
save_gif = save_movie


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('trace', nargs='?', help='*_routetrace.json (single-trace mode)')
    ap.add_argument('--run-dir', default=None, help='stress run dir (whole-run mode)')
    ap.add_argument('--board', default=None, help='board .kicad_pcb substrate (single-trace mode)')
    ap.add_argument('-o', '--output', default=None,
                    help='output path; extension picks the format: .mp4 (smaller, '
                         'full-color, plays everywhere; needs imageio-ffmpeg) or '
                         '.gif (native, autoplays inline). Default: .gif')
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
