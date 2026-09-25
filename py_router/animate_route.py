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


class _OpLog(object):
    """An append-only log of edits to one live-copper dict (#1036).

    The lower box's layer strip needs the copper AS IT STOOD on each frame.
    Keeping `tuple(live_s.values())` per frame cost O(frames x segments):
    measured on run 32's 9:16 full-trace film, a 523 MB Python heap at 5797
    frames. The log costs O(edits) instead, and a frame keeps only its
    position in it (`_LiveRef`).
    """
    __slots__ = ('ops', '_state', '_at')

    def __init__(self):
        self.ops = []           # (key, value) = set;  (key, _GONE) = removed
        self._state = {}
        self._at = 0

    def state_at(self, n):
        """The dict's values after the first `n` edits, in insertion order.

        Fast forward when frames are asked in order (the encoder streams them
        that way); a backwards ask replays from the start, which is slower
        but never wrong.
        """
        if n < self._at:
            self._state, self._at = {}, 0
        st = self._state
        for k, v in self.ops[self._at:n]:
            if v is _GONE:
                st.pop(k, None)
            else:
                st[k] = v
        self._at = n
        return tuple(st.values())


_GONE = object()


class _LoggedDict(dict):
    """A dict whose every edit is recorded into an `_OpLog`."""

    def __init__(self, log):
        super().__init__()
        self._log = log

    def __setitem__(self, k, v):
        super().__setitem__(k, v)
        self._log.ops.append((k, v))

    def __delitem__(self, k):
        super().__delitem__(k)
        self._log.ops.append((k, _GONE))

    def pop(self, k, *default):
        had = k in self
        out = super().pop(k, *default)
        if had:
            self._log.ops.append((k, _GONE))
        return out

    def clear(self):
        for k in list(self):
            self._log.ops.append((k, _GONE))
        super().clear()

    def update(self, *a, **kw):
        for k, v in dict(*a, **kw).items():
            self[k] = v

    def setdefault(self, k, v=None):
        if k not in self:
            self[k] = v
        return self[k]

    def popitem(self):
        k, v = super().popitem()
        self._log.ops.append((k, _GONE))
        return k, v


class _LiveRef(object):
    """A frame's copper: a position in an `_OpLog`, resolved at draw time."""
    __slots__ = ('log', 'n')

    def __init__(self, log, n):
        self.log, self.n = log, n

    def resolve(self):
        return self.log.state_at(self.n)


def _live(value):
    """A chrome record's copper as a tuple, whether stored as a `_LiveRef`
    or (from an older caller) as the tuple itself."""
    if isinstance(value, _LiveRef):
        return value.resolve()
    return tuple(value or ())


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
        #: #1019. One record per frame: what the RAIL says (stable), what the
        #: EVENT line says (per frame), what the TOTALS block says. Kept beside
        #: the frames rather than baked into them, so the composer can size
        #: each region for its own content. One strip doing four jobs is why
        #: the caption overflowed and dropped `hole-conflict 0.60mm` and
        #: `oob 7` off a strip that still looked complete.
        self.chrome = []
        self.rail_left = ''
        self.rail_right = ''
        self.totals = ''
        #: Set by `build_boards` when the layout reserved a rail.
        self.split_caption = False
        #: Set by `build_boards` when the layout reserved a LOWER BOX. It
        #: gates the per-frame copper snapshot below: retaining
        #: `tuple(self.live_s.values())` on every frame costs
        #: O(frames x segments) references (measured: 5961 refs / 48 KB on a
        #: 7-frame 1701-segment reveal, and it grows with both), and 'legacy'
        #: -- the default -- never draws a panel at all.
        self.want_panel = False
        #: True when the board's parts are still stacked in a pile, from
        #: `assess_placement`. It picks the box's SEEDING content, and it is
        #: the only thing that can: a label cannot say whether the parts have
        #: been seated yet.
        self.unplaced = False
        #: The layer the current event is on, so the strip can light it.
        self.active_layer = None
        #: An extra `fn(draw, renderer)` drawn through `frame(overlays=...)`
        #: for as long as a caller keeps it set -- the seam a Stage needs to
        #: draw a ghost and an arrow over a placement tween. It costs NO frame
        #: geometry, which is why the overlay seam is the right place for it.
        self.overlay = None
        self.layers = layers
        self.rip_hold = rip_hold
        #: Layer name -> trace-row index, so a live `_Seg` can be turned back
        #: into the row shape `copper_motion` works in.
        self._li = {n: i for i, n in enumerate(layers)}
        #: #1022. Retract a rip and grow its replacement instead of flashing
        #: red for two frames. Tied to `rip_hold` deliberately rather than
        #: given a knob of its own: `--rip-hold 0` already means "no rip
        #: animation, just cut", and that is exactly the degradation this
        #: needs. It changes frame COUNT, never frame SIZE.
        self.motion = rip_hold > 0
        #: `{class: (seated, total)}` for the box's inventory content,
        #: computed ONCE over the board rather than per frame.
        self.inventory = {}
        #: Edit logs behind the live copper (#1036): see `_OpLog`.
        self._log_s, self._log_v = _OpLog(), _OpLog()
        self.live_s: Dict[Tuple, _Seg] = _LoggedDict(self._log_s)
        self.live_v: Dict[Tuple, _Via] = _LoggedDict(self._log_v)
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

    def _overlays(self):
        """Everything to draw above the copper this frame, in draw order."""
        return [o for o in (self.overlay, self._key_overlay()) if o]

    def _key_overlay(self):
        """The in-frame key, drawn through `frame(overlays=...)`.

        That seam draws at supersampled resolution above copper and below the
        label, and -- the reason this is cheap -- it costs NO FRAME GEOMETRY.
        #946 ranked the key as step 8, after several layout changes, because it
        ranked by where a key APPEARS rather than by how it is DRAWN.
        """
        if not self.seen_events:
            return None
        if self.split_caption:
            # A layout with a RAIL carries the key there (#946 review): as
            # a corner overlay it floated over the board's bottom-left.
            return None
        from render_chrome import event_rows, draw_key
        rows = event_rows(self.theme, seen=self.seen_events)

        def _draw(d, r):
            ss = max(1, int(getattr(r, 'ss', 1)))
            draw_key(d, rows, width=r.W * ss, height=r.H * ss,
                     theme=self.theme, corner='bl', pad_scale=ss)
        return _draw

    def _note_chrome(self, label):
        self.chrome.append({'rail': self.rail_left,
                            'rail_right': self.rail_right,
                            'event': label or '', 'totals': self.totals,
                            # #1020: the copper as it stands on THIS frame, so
                            # the per-layer strip grows with the film instead
                            # of showing the finished board from frame one.
                            # ONLY when a box exists to draw it in -- see
                            # `want_panel`.
                            # Stored as a POSITION in the edit log (#1036),
                            # not a copy: `_live()` resolves it at draw time.
                            'live': (_LiveRef(self._log_s,
                                              len(self._log_s.ops))
                                     if self.want_panel else ()),
                            'live_v': (_LiveRef(self._log_v,
                                                len(self._log_v.ops))
                                       if self.want_panel else ()),
                            'unplaced': self.unplaced,
                            'inventory': self.inventory,
                            'active': self.active_layer,
                            # the key's rows as of THIS frame, for the rail
                            'seen': tuple(self.seen_events)})

    def _frame(self, hl_s, hl_v, color, label, mark='solid', base_s=None):
        """One frame. `base_s` overrides the live copper drawn under the
        highlight.

        #1022 needs it for ONE case, and the docstring used to claim two: a
        GROWTH stage must not have its finished self already drawn underneath
        it, because `add` inserts into `live_s` before it draws. The
        retraction half was vacuous -- `remove` pops the doomed keys BEFORE it
        animates, so the live state is already correct there and passing it
        explicitly is pixel-identical (verified). `base_v` is gone for the same
        reason: it never had a caller."""
        ov = self._overlays()
        self._note_chrome(label)
        # #1019: when a rail is going to carry this, the over-board strip is a
        # DUPLICATE, and a duplicate that sits on the copper is worse than no
        # strip at all. `_label` stays for the legacy frame, which has no rail.
        if self.split_caption:
            label = None
        self.frames.append(self.r.frame(
            segments=(list(self.live_s.values()) if base_s is None
                      else list(base_s)),
            vias=list(self.live_v.values()),
            highlight_segments=hl_s, highlight_vias=hl_v,
            highlight_color=color, highlight_mark=mark, label=label,
            zone_net_ids=self.revealed_zones,
            overlays=ov or None))

    def refresh_placement(self, pcb, path=None):
        """Re-read the lower box's non-routing data from THIS board.

        Three defects the round-2 verifier measured, all in one place:

        * `assess_placement` lives in `py_placer/placement/`, which `py_router`
          does not put on `sys.path`, so the import raised `ModuleNotFoundError`
          into the swallow and `unplaced` was ALWAYS False -- the 'seeding'
          content could not occur in a CLI film at all. Proven with a genuinely
          piled board: the CLI arm reported `{'bookend': 1}` and the GUI arm,
          with `py_placer` already on the path, reported `{'seeding': 1}`. The
          path is added here rather than at module scope, because a movie must
          not pay for a placement import it may never need.
        * it read the CHAIN'S FINAL board, so a film OF a seeding run asked a
          board that is by then placed. Every step re-reads its own.
        * the inventory was computed once, so the bars never emptied.

        Never raises: without `py_placer` the inventory still counts parts, it
        just cannot tell a seated one from a piled one, and that is a strictly
        better answer than no box.
        """
        if not self.want_panel or pcb is None:
            return
        import render_panels as _rp
        unseated = ()
        try:
            import os as _os
            import sys as _sys
            _pp = _os.path.join(_os.path.dirname(_os.path.dirname(
                _os.path.abspath(__file__))), 'py_placer')
            if _os.path.isdir(_pp) and _pp not in _sys.path:
                _sys.path.insert(0, _pp)
            from placement.placement_state import assess_placement
            st = assess_placement(pcb, path)
            self.unplaced = bool(st.unplaced)
            unseated = st.stacked_suspect_refs
        except Exception:                                      # noqa: BLE001
            pass
        # A part ENTIRELY off the board is not placed either (#1036).
        # `assess_placement` finds STACKED parts, and run 32's pile is laid out
        # in rows beside the outline, not stacked: 247 of its 272 parts sit
        # outside it, and the box read "272 of 272 placed" over the pile.
        # ENTIRELY: the test is the part's pad extent against the outline, not
        # its origin -- an edge connector whose origin overhangs the outline
        # is placed, and read "N-1 of N" when the origin decided.
        try:
            bb = pcb.board_info.board_bounds
            if bb:
                x0, y0, x1, y1 = bb

                def _off(fp):
                    pads = fp.pads or ()
                    if not pads:
                        return not (x0 <= fp.x <= x1 and y0 <= fp.y <= y1)
                    px0 = min(p.global_x - p.size_x / 2.0 for p in pads)
                    px1 = max(p.global_x + p.size_x / 2.0 for p in pads)
                    py0 = min(p.global_y - p.size_y / 2.0 for p in pads)
                    py1 = max(p.global_y + p.size_y / 2.0 for p in pads)
                    return px1 < x0 or px0 > x1 or py1 < y0 or py0 > y1
                off = {ref for ref, fp in pcb.footprints.items()
                       if _off(fp)}
                if off:
                    unseated = set(unseated or ()) | off
        except Exception:                                      # noqa: BLE001
            pass
        try:
            self.inventory = _rp.inventory_counts(pcb, unseated)
        except Exception:                                      # noqa: BLE001
            pass

    def _row(self, sg):
        """A live `_Seg` back as a trace row, for `copper_motion`."""
        return [sg.start_x, sg.start_y, sg.end_x, sg.end_y, sg.width,
                self._li.get(sg.layer, 0)]

    def _near_live(self, rows, exclude=()):
        """Live copper near `rows`, as rows -- the anchor search's haystack.

        BOUNDED on purpose: `anchor_for` is O(|moving| x |live|), and a rip of
        20 segments against a 1701-segment board would be 68k distance
        computations per rip. Copper far from the doomed set cannot be the end
        it is pulled back to, so a neighbourhood filter keeps the cost
        proportional to the neighbourhood.

        **THE TEST IS SEGMENT-OVERLAP, NOT ENDPOINT-CONTAINMENT**, and the
        phase-12 verifier measured why the first version was wrong. It admitted
        a live segment only when one of its two ENDPOINTS fell in the box,
        while the justification is about DISTANCE -- so a long trunk passing
        THROUGH the neighbourhood with both ends far outside it was dropped.
        Constructed as a T-junction (a 60 mm trunk, a stub branching mid-span)
        and driven through `Movie.remove`: the filter returned nothing, the
        anchor fell back to the centroid rule and landed on the stub's own
        MIDDLE, and the stub retracted from both ends at once leaving a
        DETACHED FLOATING piece -- the exact artefact `order_from` exists to
        prevent. Comparing bounding boxes costs the same and cannot miss it.
        """
        if not rows:
            return []
        xs = [r[0] for r in rows] + [r[2] for r in rows]
        ys = [r[1] for r in rows] + [r[3] for r in rows]
        pad = max(max(xs) - min(xs), max(ys) - min(ys), 1.0)
        x0, x1 = min(xs) - pad, max(xs) + pad
        y0, y1 = min(ys) - pad, max(ys) + pad
        skip = set(exclude)
        out = []
        for k, sg in self.live_s.items():
            if k in skip:
                continue
            # the segment's own bbox against the neighbourhood's -- true for a
            # segment with an end inside, AND for one that merely crosses it.
            if (min(sg.start_x, sg.end_x) <= x1
                    and max(sg.start_x, sg.end_x) >= x0
                    and min(sg.start_y, sg.end_y) <= y1
                    and max(sg.start_y, sg.end_y) >= y0):
                out.append(self._row(sg))
        return out

    def _motion_frames(self, rows, live_rows, base_s, color, label, mark,
                       grow):
        """Emit the retraction / growth stages. Returns how many it drew."""
        import copper_motion
        try:
            plan = copper_motion.stages(rows, live=live_rows, grow=grow)
        except Exception:                                      # noqa: BLE001
            return 0            # motion is decoration; never lose the film
        n = 0
        for stage in plan:
            if not stage and not grow:
                # The empty last retraction stage IS the event -- the copper is
                # gone -- so it is drawn: one frame of the board without it,
                # still captioned as the rip.
                self._frame([], [], color, label, mark=mark, base_s=base_s)
                n += 1
                continue
            if not stage:
                continue
            self._frame([_Seg(r, self.layers) for r in stage], [], color,
                        label, mark=mark, base_s=base_s)
            n += 1
        return n

    def snapshot(self, label):
        """A plain frame of the current state (no highlight).

        Carries `overlay` too: a placement tween is made of SNAPSHOTS, so a
        ghost hooked only into `_frame` would never appear on the frames it
        exists for.
        """
        ov = self._overlays()
        self._note_chrome(label)
        if self.split_caption:
            label = None
        self.frames.append(self.r.frame(
            segments=list(self.live_s.values()), vias=list(self.live_v.values()),
            label=label, zone_net_ids=self.revealed_zones,
            overlays=ov or None))

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
            # `new_s` holds `_Seg` objects, which carry `.layer` already --
            # the strip lights whichever layer the event touched.
            self.active_layer = next(
                (sg.layer for sg in new_s if getattr(sg, 'layer', None)),
                self.active_layer)
            col = _add_color(event, self.theme)
            # #1022. A RESTORE grows out of its anchor, which is the opposite
            # motion to the rip that preceded it -- and direction survives
            # every colour deficiency there is. Only restores move: a plain
            # `new` add happens thousands of times in a film and animating
            # each one would make every movie four times longer for an event
            # that has no counterpart to be confused with.
            if self.motion and role == 'event_restored' and new_s:
                fresh = {seg_key_row(self._row(sg)) for sg in new_s}
                rows = [self._row(sg) for sg in new_s]
                near = self._near_live(rows, exclude=fresh)
                base = [sg for k, sg in self.live_s.items() if k not in fresh]
                # Every stage but the last: the last one is the full copper,
                # and that frame is the ordinary one below, so the live state
                # and the final frame cannot disagree.
                import copper_motion
                try:
                    plan = copper_motion.stages(rows, live=near, grow=True)
                except Exception:                              # noqa: BLE001
                    plan = []
                for stage in plan[:-1]:
                    if stage:
                        self._frame([_Seg(r, self.layers) for r in stage], [],
                                    col, label, base_s=base)
            self._frame(new_s, new_v, col, label)

    def remove(self, seg_keys, via_keys, label, by=None):
        """Flash the doomed copper red (still present), then drop it."""
        hl_s = [self.live_s[k] for k in seg_keys if k in self.live_s]
        hl_v = [self.live_v[k] for k in via_keys if k in self.live_v]
        if not hl_s and not hl_v:
            return
        rlabel = label + (f"  (rip by {by})" if by else '  (rip)')
        rip = self.theme.rgb('event_ripped')
        dash = self.theme.mark('event_ripped')
        for _ in range(max(1, self.rip_hold)):
            self._note_event('event_ripped')
            self._frame(hl_s, hl_v, rip, rlabel, mark=dash)
        # #1022. The rows BEFORE they leave, and the neighbourhood they are
        # pulled back toward -- both read while the copper is still live.
        rows = [self._row(sg) for sg in hl_s] if self.motion else []
        near = self._near_live(rows, exclude=set(seg_keys)) if rows else []
        for k in seg_keys:
            self.live_s.pop(k, None)
        for k in via_keys:
            self.live_v.pop(k, None)
        if rows:
            # A via cannot retract -- it is a hole, not a length -- so it
            # leaves with the flash and the tracks pull back after it.
            self._motion_frames(rows, near, list(self.live_s.values()), rip,
                                rlabel, dash, grow=False)

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


def render_chrome_lap(n, laps, label):
    """`lap 3 of 5 - route` for a loop chain, the step label otherwise."""
    from render_chrome import lap_text
    try:
        lap = laps.index(n) + 1
    except ValueError:
        return label
    phase = 'route' if 'routed' in label else 'place'
    return lap_text(label, lap=lap, laps=len(laps), phase=phase)


def board_title(final, steps=(), hint=None):
    """What the rail's STABLE left should say: the board, not a step.

    `hint` wins -- `make_movie` passes the run directory's name when the chain
    came from one, which is the closest thing a multi-step run has to a board
    name. Otherwise the step prefix is stripped off the final board's stem,
    because the rail's left is the one field that does NOT change frame to
    frame and naming it after the LAST step contradicts that: a film of
    `step1 -> step4` read `step4_restored` on frame 1.

    Falls back to the stem unchanged, which is what a single-board film has
    always shown.
    """
    if hint:
        return str(hint)
    stem = os.path.splitext(os.path.basename(final or ''))[0]
    if len(steps) < 2:
        return stem
    # The two shapes a chain's boards actually take, neither of which contains
    # a board name: `stepN_<what>` from a routing chain, `loop_roundN` /
    # `roundN` from a placement loop. Strip the run prefix; if every step
    # leaves the SAME tail that tail is the board, and if they leave nothing
    # (a loop's boards are numbered and nothing else) the run directory is the
    # only name there is.
    pre = re.compile(r'^(?:step|loop_round|round|iter)\d*[_-]?')
    if not pre.match(stem):
        # NEVER A LATER BOARD'S NAME (#1036 review). A hand-named chain --
        # glasgow_unplaced -> placed_v2 -> ... -> K3C_route -- used to title
        # every frame with the FINAL stem, so the placement beats read
        # "K3C_route" on the left beside "placed_v2" on the right. The film's
        # name is the directory the chain lives in; boards spread over
        # several directories fall back to the FIRST board, which is at worst
        # a name from the past, never from the future.
        stems = [os.path.splitext(os.path.basename(str(st[1])))[0]
                 for st in steps if len(st) > 1]
        if len(set(stems)) <= 1:
            return stem
        dirs = {os.path.dirname(os.path.abspath(str(st[1])))
                for st in steps if len(st) > 1}
        if len(dirs) == 1:
            name = os.path.basename(dirs.pop())
            if name:
                return name
        return stems[0] if stems else stem
    stems = [os.path.splitext(os.path.basename(str(st[1])))[0]
             for st in steps if len(st) > 1]
    tails = {pre.sub('', x) for x in stems}
    if len(tails) == 1:
        only = tails.pop()
        if only:
            return only
    return os.path.basename(os.path.dirname(os.path.abspath(final))) or stem


def trace_frame_estimate(trace, rip_hold=2):
    """How many frames `Movie.play_trace` will emit for ``trace``, read off the
    events alone -- before a single frame is drawn.

    One frame per add event, `max(1, rip_hold)` per rip, plus the retract/grow
    motion stages #1022 draws (bounded by `copper_motion`'s stage count, taken
    here as 3 per rip and per restore). An ESTIMATE, and it is used only to
    decide whether a trace fits the film's frame budget; it errs high, so a
    trace that is played is never longer than it was allowed to be by much.
    """
    n = 0
    motion = 3 if rip_hold > 0 else 0
    for ev in (trace.get('events') or ()):
        if ev.get('del_s') or ev.get('del_v'):
            n += max(1, rip_hold) + (motion if ev.get('del_s') else 0)
        if ev.get('add_s') or ev.get('add_v'):
            n += 1
            e = str(ev.get('event', '')).lower()
            if 'reroute' in e or 'restore' in e or 'rescue' in e:
                n += motion
    return n


class _OverBudget(Exception):
    """A trace that does not fit the film's frame budget (#1036)."""


def build_boards(steps, final, size, ss, alpha, rip_hold, chunks, stage=None,
                 marks=None, theme=None, layout=None, aspect=None,
                 geom_out=None, title=None, frames_sink=None,
                 max_frames=None, notes=None, attempts_band=False,
                 iso_panel=False, lands_out=None):
    """Frames for a chain given as [(label, board, trace|None), ...] plus the
    final board. ``build_run`` is this with the chain discovered from a run dir.

    ``lands_out`` (#1042), a dict when passed, collects ``{normcased abs
    board path: frame}`` -- the frame a GLIDE lands on, which is where the
    placement panels change beat (a glide shows the SOURCE board until then).

    ``stage`` (movie_camera.Stage, #431) adds a camera and animates FOOTPRINT
    motion for placement rounds. With ``stage=None`` -- every existing caller --
    the three hooks below are falsy branches and the routing movie is unchanged.

    ``marks``, when a list is passed, collects ``(label, board, first, last)``
    frame indices per step -- what a composer needs to caption, badge or splice
    a beat without re-deriving where it landed. Rendering one pass and reading
    the boundaries back is the only way to keep ONE scale across the whole
    film; a per-segment call restarts from an empty board and re-reveals all
    the copper, which reads as the board redrawing itself between beats.

    ``frames_sink`` (#1036), when given, replaces the in-memory frame list --
    `make_movie` passes a `frame_spool.FrameSpool`, so a 6000-frame film costs
    one frame of RAM instead of 29.5 GB. Every caller that passes nothing gets
    the list it always got.

    ``max_frames`` (#1036) is the film's FRAME BUDGET. A per-segment trace
    whose estimated length does not fit its fair share of what is left
    (`trace_frame_estimate`) is not played: its step falls back to the
    board-to-board `reveal_delta(..., chunks)` reveal, and the fallback is
    printed LOUDLY and appended to ``notes`` when a list is passed. ``None``
    or 0 = no budget, today's behaviour.

    ``attempts_band`` (#946/C4) reserves the attempts band INSIDE the planned
    frame (`plan_frame(track_px=)`), so a declared ratio keeps its size. A
    CALLABLE ``(frame_w, frame_h) -> px`` sizes it instead (#1042: the
    placement panels' `movie_placement.band_px`); the
    band's box is `geom_out[0].track` and `movie_attempts.attach(box=)` draws
    into it. ``iso_panel`` asks the layout to split its panel so the 3D view
    has a region of its own (`geom.panel_split[0]`); the layer strip then
    draws into the other half.
    """
    from kicad_parser import parse_kicad_pcb
    if not final:
        return [] if frames_sink is None else frames_sink
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
    _geom = None
    if True:
        import frame_layout
        _plan_kw = dict(
            layout=layout or 'legacy',
            ratio=frame_layout.parse_ratio(aspect), size=size,
            # A panel is reserved for every layout that declares one, EXCEPT
            # 'legacy' -- which has no chrome at all, because legacy means
            # today's frame and today's frame has no lower box.
            panel=(str(layout or 'legacy').lower() != 'legacy'),
            legacy_size=(r.W, r.H), iso=bool(iso_panel))
        _g = frame_layout.plan_frame(r.pcb.board_info.board_bounds,
                                     **_plan_kw)
        if attempts_band:
            # The band's height is a fraction of the FRAME it sits in, so the
            # frame is planned once to learn its size and once more with the
            # band reserved. Two calls of pure arithmetic, no pixels.
            try:
                import movie_attempts
                if callable(attempts_band):
                    # #1042: the caller SIZES the band for this frame -- the
                    # placement panels' `band_px`, which reserves room for
                    # readable panels (plot >= PLOT_MIN_PX) beside or above
                    # the verdict graph, or 0 when the frame cannot hold them.
                    _bh = int(attempts_band(_g.frame.w, _g.frame.h) or 0)
                    _bh -= _bh % 2
                else:
                    _bh = movie_attempts.band_height(_g.frame.w, _g.frame.h)
            except Exception:                                  # noqa: BLE001
                _bh = 0
            if _bh:
                _g = frame_layout.plan_frame(r.pcb.board_info.board_bounds,
                                             track_px=_bh, **_plan_kw)
        # Only when the layout genuinely MOVES the board box. On 'legacy' the
        # box is the renderer's own size evened, and the evening is applied by
        # cropping the composed frame instead -- because
        # `tests/test_431_placement_movie.py:92-121` pins exactly ONE
        # `set_view` on the no-stage path, and that assertion is this phase's
        # own falsifier: if the layout work needs a second aim, the layout work
        # is wrong.
        moved = (_g.board.w, _g.board.h) != (r.W, r.H)
        # A legacy frame with a DECLARED ratio or a reserved band is not
        # today's frame any more, so its board box is honoured too; the plain
        # legacy frame keeps the one-set_view path the test pins.
        if moved and (_g.layout != 'legacy' or aspect or _g.track):
            r.set_canvas(_g.board.w, _g.board.h)
        # `geom_out` is an OUTPUT collector, never the switch. It was both
        # until a full film was rendered twice: `build_boards(layout='split')`
        # WITHOUT a `geom_out` list silently produced today's frame -- no rail,
        # no lower box, no composition -- so the layout took effect only for a
        # caller that happened to ask for the geometry back. `make_film` is
        # exactly such a caller.
        _geom = _g
        if geom_out is not None:
            geom_out.append(_g)
    m = Movie(r, layers, rip_hold=rip_hold)
    if frames_sink is not None:
        m.frames = frames_sink
    # #1036: the frame budget, shared FAIRLY among the traced steps -- a
    # greedy budget would let the first long trace eat the film and starve
    # every later one into chunks, which is the opposite of what a viewer
    # wants from the end of a run.
    _budget = int(max_frames or 0)
    _traced_left = sum(1 for _s in steps if len(_s) > 2 and _s[2])
    # #1020: the lower box's non-routing contents, decided ONCE. `want_panel`
    # also gates the per-frame copper snapshot, so a legacy film -- which has
    # no box -- retains nothing.
    m.want_panel = bool(_geom is not None and _geom.panel is not None
                        and _geom.panel.h > 0)
    #: #946/C4: the iso view takes `panel_split[0]`; the strip draws into
    #: `panel_split[1]`, and the iso half is left as panel ground for
    #: `movie_panels.compose_two_panel(box=)` to fill.
    m.iso_in_panel = bool(iso_panel and _geom is not None
                          and _geom.panel_split)
    if m.want_panel:
        # Seeded from the chain's FIRST board, not its last: the opening
        # snapshot is of the board as it arrived, and a film of a seeding run
        # asked the final board -- which is by then placed.
        _seed = steps[0][1] if steps else final
        try:
            m.refresh_placement(parse_kicad_pcb(_seed), _seed)
        except Exception:                                      # noqa: BLE001
            pass
    # #1019. THE RAIL COUNTS LAPS, NOT STEPS. A loop revisits the same step, so
    # `step 2 - route` cannot say whether this is the first attempt or the
    # fourth. `placement_chain` labels its steps `round N` / `round N routed`,
    # and those rounds ARE the laps -- so when the chain is a loop the rail can
    # count them, and when it is not there are no laps to count and the step
    # label is the honest thing to show.
    _laps = sorted({int(mm.group(1)) for mm in
                    (re.match(r'round (\d+)', str(st[0])) for st in steps)
                    if mm})
    m.rail_left = board_title(final, steps, title)
    m.split_caption = bool(_geom is not None and _geom.rail.h > 0)
    # #1036: the SUBSTRATE is each step's own board. The renderer is built
    # from the FINAL board (it fixes the canvas and the scale), and until this
    # change only a Stage re-pointed `r.pcb` per step -- so without one, the
    # opening frame of a chain that starts from an unplaced pile showed the
    # FINAL placement's pads under the first board's copper. `frame()` draws
    # pads and zones per frame from `r.pcb` (dynamic_zones), so re-pointing it
    # is the whole fix; it changes no frame geometry and builds no renderer.
    if steps and os.path.abspath(steps[0][1]) != os.path.abspath(final):
        try:
            r.pcb = parse_kicad_pcb(steps[0][1])
        except Exception:                                      # noqa: BLE001
            pass
    if stage is not None:
        stage.attach(m, r, layers)
    m.snapshot("input")
    #: The board the previous step left, for a glide's source inventory.
    _prev_board = steps[0][1] if steps else None
    for _step in steps:
        _lbl = str(_step[0])
        _mm = re.match(r'round (\d+)', _lbl)
        if _mm and _laps:
            m.rail_right = render_chrome_lap(int(_mm.group(1)), _laps, _lbl)
        else:
            m.rail_right = _lbl
        # 4th element (optional, back-compatible): 'revert' undoes a beat with
        # the SILENT trueup instead of reveal_delta -- which would flash the
        # copper red and label it "(rip)". Nothing was ripped; an attempt was
        # not kept, and the two read completely differently in a film.
        label, board, trace_path = _step[0], _step[1], _step[2]
        mode = _step[3] if len(_step) > 3 else None
        _first = len(m.frames)
        pcb = parse_kicad_pcb(board)
        seg_rows, via_rows = _board_rows(pcb, layers)
        # #1020: this step's OWN board answers the box, so the inventory
        # empties as the board fills and a seeding beat is a seeding beat.
        _gliding = (stage is not None and mode != 'revert'
                    and stage.handles(board))
        if _gliding and _prev_board:
            # #1036: a glide is drawn from the board it LEAVES. The box's
            # inventory follows the parts, so it reads the source board until
            # the glide lands -- it read "272 of 272 placed" mid-glide, the
            # destination's count, over parts still in the pile. The stage
            # calls `on_arrive` right before its landing frame.
            m.refresh_placement(r.pcb, _prev_board)

            def _on_arrive(_p=pcb, _b=board):
                m.refresh_placement(_p, _b)
                # #1042: the placement panels read the SAME landing frame,
                # so their beat changes with the inventory, not before it.
                if lands_out is not None:
                    lands_out.setdefault(
                        os.path.normcase(os.path.abspath(_b)), len(m.frames))
            stage.on_arrive = _on_arrive
        else:
            m.refresh_placement(pcb, board)
        _prev_board = board
        # Every step draws its OWN board's pads (#1036), stage or not. The
        # board it REPLACES is handed to the stage, whose camera shots before
        # a placement beat must show the parts where they WERE -- otherwise
        # the establishing shot shows the finished placement, and the parts
        # then jump back into the pile to glide out of it.
        if stage is not None:
            stage.prev_pcb = r.pcb
        r.pcb = pcb
        if mode == 'revert':
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
            if stage.enter_step(label, board, pcb, seg_rows, via_rows):
                if marks is not None:
                    marks.append((label, board, _first, len(m.frames)))
                continue        # a placement round; the stage emitted its frames
        # Reveal any plane whose pour exists on this step board but had no fine
        # plane-tap event (untraced plane step) so its fill still appears here.
        step_zone_nets = {z.net_id for z in (getattr(pcb, 'zones', None) or [])
                          if z.net_id and len(z.polygon) >= 3}
        if trace_path:
            _traced_left = max(0, _traced_left - 1)
            try:
                _tr = load_trace(trace_path)
                if _budget:
                    _left = max(0, _budget - len(m.frames))
                    _share = _left // (_traced_left + 1)
                    _est = trace_frame_estimate(_tr, rip_hold)
                    if _est > _share:
                        _msg = ('step %r trace needs ~%d frames, its share of '
                                'the --max-frames %d budget is %d; revealing '
                                'its delta in %d chunks instead'
                                % (str(label), _est, _budget, _share, chunks))
                        print('animate_route: TRACE OVER BUDGET -- ' + _msg,
                              file=sys.stderr)
                        if notes is not None:
                            notes.append(_msg)
                        raise _OverBudget(_msg)
                m.play_trace(_tr, label_prefix=f"{label}: ",
                             only_new=True)
                for _nid in step_zone_nets:
                    m.reveal_zone(_nid)
                m.reconcile_to(seg_rows, via_rows, label)   # trueup to step board
                if marks is not None:
                    marks.append((label, board, _first, len(m.frames)))
                continue
            except _OverBudget:
                pass            # already said, loudly; fall through to chunks
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
    m.refresh_placement(fpcb, final)
    r.pcb = fpcb
    for _z in (getattr(fpcb, 'zones', None) or []):   # ensure every pour shows
        m.reveal_zone(_z.net_id)
    _before = len(m.frames)
    m.reconcile_to(*_board_rows(fpcb, layers), "routed")
    # THE CLOSING BOOKEND. `reconcile_to` is silent when nothing changed, so a
    # film whose last step already matched the final board ended on a ROUTING
    # frame and never reached the bookend content at all -- half the "open and
    # close" the lower box is designed around, missing.
    #
    # Only when a box EXISTS to hold it: on 'legacy' this would add a frame to
    # every existing movie, and legacy is the arm that must not move.
    if m.want_panel and len(m.frames) == _before:
        m.snapshot("routed")
    if stage is not None:
        stage.outro()
    # #1018: the board was rendered into its PLANNED BOX; the frame is the
    # planned FRAME. Composing here rather than leaving the box as the frame is
    # what makes the size claim real -- the rail, the panel and the foot exist
    # as reserved ground from this commit, and #1019/#1020/#1021 fill them.
    #
    # In place, so peak memory stays about two frames rather than twice the
    # movie: the same reason `movie_panels.compose_two_panel` does it that way.
    if _geom is not None:
        _compose_into_frame(m.frames, _geom, r, m.chrome,
                            iso_in_panel=getattr(m, 'iso_in_panel', False))
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
    except Exception as e:
        print(f"animate_route: mp4 encode failed ({e}); falling back to GIF",
              file=sys.stderr)
        return False
    # PRODUCING a frame is not ENCODING it (#1036 review). A spooled film's
    # frames are composed lazily as this loop pulls them, so an exception
    # raised while pulling one is the film's own defect: re-raised as itself,
    # never reported as "mp4 encode failed" and retried as a GIF that fails
    # the same way. Only the encoder's own calls fall back.
    it = iter(frames)
    while True:
        try:
            fr = next(it)
        except StopIteration:
            break
        except BaseException:
            try:
                w.close()
            except Exception:                                   # noqa: BLE001
                pass
            try:
                os.remove(out)          # a truncated film is not a film
            except OSError:
                pass
            raise
        try:
            a = np.asarray(fr.convert('RGB'))
            h, wd = a.shape[0] & ~1, a.shape[1] & ~1
            w.append_data(a[:h, :wd])
        except Exception as e:
            try:
                w.close()
            except Exception:                                   # noqa: BLE001
                pass
            print(f"animate_route: mp4 encode failed ({e}); falling back to "
                  f"GIF", file=sys.stderr)
            return False
    try:
        w.close()
    except Exception as e:
        print(f"animate_route: mp4 encode failed ({e}); falling back to GIF",
              file=sys.stderr)
        return False
    return True


def _png_info(meta):
    """A Pillow PngInfo for one frame's metadata block (#887)."""
    from PIL import PngImagePlugin
    info = PngImagePlugin.PngInfo()
    for k, v in (meta or {}).items():
        info.add_text(str(k), '' if v is None else str(v))
    return info


def _compose_into_frame(frames, geom, r, chrome=None, iso_in_panel=False):
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
    import frame_spool
    th = getattr(r, 'theme', None)
    bg = th.rgb('ground') if th is not None else (14, 16, 18)
    W, H = geom.frame.w, geom.frame.h
    # #1036: one per-frame transform. On a spool it is applied lazily while
    # the encoder streams; on a list it rewrites in place, as it always did.
    chrome_fn = (_chrome_drawer(len(frames), geom, r, chrome,
                                iso_in_panel=iso_in_panel)
                 if chrome and geom.rail.h > 0 else None)

    def _one(i, f):
        if f.size != (W, H):
            if (f.width >= W and f.height >= H and geom.board.x == 0
                    and geom.board.y == 0 and geom.panel is None):
                f = f.crop((0, 0, W, H))
            else:
                canvas = Image.new('RGB', (W, H), bg)
                canvas.paste(f, (geom.board.x, geom.board.y))
                f = canvas
        if chrome_fn is not None:
            f = chrome_fn(i, f)
        return f
    frame_spool.transform(frames, _one, out_size=(W, H))


#: The least spare height under the layer grid worth filling with the board's
#: numbers; below it the grid is centred instead.
STRIP_SUMMARY_MIN_PX = 90


def _draw_panel(d, geom, r, c, iso_in_panel=False):
    """The lower box, whichever of its four contents this phase asks for.

    Four REAL contents, not one content and three captions: the phase-1
    verifier measured that `draw_inventory` had no caller anywhere in the repo
    and that `phase_for(unplaced=...)` was never called from production, so
    'seeding' could not occur in a film at all and the other two branches drew
    a literal string. Each branch now draws data the film already holds.
    """
    if geom.panel is None or geom.panel.h <= 0 or geom.panel.w <= 0:
        return
    try:
        import render_panels
        th = getattr(r, 'theme', None)
        phase = render_panels.phase_for(c.get('event', ''),
                                        unplaced=bool(c.get('unplaced')))
        box = geom.panel
        d.rectangle([box.x, box.y, box.x + box.w - 1, box.y + box.h - 1],
                    fill=th.rgb('chrome_panel') if th else (14, 14, 18))
        if iso_in_panel and geom.panel_split:
            # the iso half is filled later by compose_two_panel(box=)
            box = geom.panel_split[1]
        # THE GUTTER (#946 review): every content keeps the design system's
        # inner margin from its box -- the 4:3 inventory's counts touched
        # the frame's right edge.
        import render_chrome
        g = render_chrome.gutter_px(geom.frame.w)
        box = box._replace(x=box.x + g, y=box.y + g,
                           w=max(2, box.w - 2 * g), h=max(2, box.h - 2 * g))
        if phase == 'routing':
            # Cells shaped like the BOARD, in a grid when the box is tall
            # (the 1:1 sidebar gave 85x500 cells). The grid is centred; when
            # there is room under it, the board's numbers go there.
            _x0, _y0, _x1, _y1 = r.bounds
            _asp = max(_x1 - _x0, 1e-6) / max(_y1 - _y0, 1e-6)
            _b, _n, gh = render_panels.grid_boxes(
                box, len(r.copper_layers), _asp)
            spare = box.h - gh
            if _n and spare >= STRIP_SUMMARY_MIN_PX:
                strip = box._replace(h=gh)
                render_panels.draw_summary(
                    d, box._replace(y=box.y + gh, h=spare), theme=th,
                    lines=render_panels.board_summary(
                        r.pcb, _live(c.get('live')), _live(c.get('live_v'))))
            else:
                strip = box._replace(y=box.y + max(0, spare) // 2,
                                     h=min(box.h, gh) if _n else box.h)
            render_panels.draw_layer_strip(
                d, strip, bounds=r.bounds, segments=_live(c.get('live')),
                layers=list(r.copper_layers), palette=r.palette, theme=th,
                active=c.get('active'), grid=True)
        elif phase == 'bookend':
            render_panels.draw_summary(
                d, box, theme=th,
                lines=render_panels.board_summary(
                    r.pcb, _live(c.get('live')), _live(c.get('live_v'))))
        else:
            inv = c.get('inventory') or {}
            done = sum(a for a, _b in inv.values())
            tot = sum(b for _a, b in inv.values())
            render_panels.draw_inventory(d, box, counts=inv, placed=done,
                                         total=tot, theme=th)
    except Exception:                                          # noqa: BLE001
        pass


def _draw_chrome(frames, geom, r, chrome):
    """Fill the reserved rail and foot (#1019), on a list or a spool."""
    import frame_spool
    frame_spool.transform(frames, _chrome_drawer(len(frames), geom, r, chrome))


def _chrome_drawer(n_frames, geom, r, chrome, iso_in_panel=False):
    """``fn(i, frame) -> frame`` drawing the rail and foot for frame ``i``.

    Each region is sized for ITS OWN content and ellipsises inside itself, so a
    long event line can no longer push the totals off the edge of a strip that
    still looks complete. It needs only the frame COUNT, never the pixels of
    another frame, which is what lets a spool apply it while streaming (#1036).
    """
    th = getattr(r, 'theme', None)
    n = max(1, n_frames - 1)
    ticks = tuple(sorted({c.get('lap_at') for c in chrome
                          if c.get('lap_at') is not None}))

    def _fn(i, f):
        _draw_chrome_one(f, i, n, geom, r, chrome, th, ticks, iso_in_panel)
        return f
    return _fn


def _draw_chrome_one(f, i, n, geom, r, chrome, th, ticks,
                     iso_in_panel=False):
    from PIL import ImageDraw
    import render_chrome
    c = chrome[i] if i < len(chrome) else (chrome[-1] if chrome else {})
    d = ImageDraw.Draw(f)
    _draw_panel(d, geom, r, c, iso_in_panel=iso_in_panel)
    _seen = c.get('seen') or ()
    render_chrome.draw_rail(d, geom.rail, c.get('rail', ''),
                            c.get('rail_right', ''), theme=th,
                            progress=i / float(n), ticks=ticks,
                            key_rows=(render_chrome.event_rows(th, seen=_seen)
                                      if _seen and th is not None else None))
    if geom.foot.h > 0:
        d.rectangle([geom.foot.x, geom.foot.y,
                     geom.foot.x + geom.foot.w - 1,
                     geom.foot.y + geom.foot.h - 1],
                    fill=th.rgb('chrome_panel') if th else (14, 14, 18))
        render_chrome.draw_totals(d, geom.foot, c.get('totals', ''),
                                  theme=th)
        ev = c.get('event', '')
        if ev:
            from route_render import load_font
            font = load_font(max(9, int(geom.foot.h * 0.34)))
            d.text((geom.foot.x + 6, geom.foot.y + 3),
                   render_chrome._fit(d, ev, font, geom.foot.w * 0.55),
                   font=font,
                   fill=th.rgb('chrome_text') if th else (240, 240, 240))


def _pad_rgb(theme=None):
    try:
        import render_theme
        return render_theme.theme(theme, strict=False).rgb('ground')
    except Exception:                                          # noqa: BLE001
        return (14, 16, 18)


def _uniform_or_pad(frames, theme=None):
    """Every frame at the first frame's size, letterboxed rather than squashed.

    Returns `frames` unchanged when they already agree, so the common path
    allocates nothing.

    The pad is the THEME's ground, not black: a black letterbox on a light
    film is the one place the whole theme system would have leaked, and it
    would look like a defect in the frame rather than in the pad. `theme` is
    optional because `save_movie` is called with a bare frame list from
    several places; without one the pad falls back to the dark ground, which
    is what it always was.

    A `frame_spool.FrameSpool` is checked from its recorded sizes and padded
    by a lazy transform, so the check decodes no frame it does not have to.
    """
    import frame_spool
    sizes = frame_spool.frame_sizes(frames)
    try:
        import frame_layout
        frame_layout.assert_frames_uniform(sorted(sizes))
        return frames
    except Exception as exc:                                    # noqa: BLE001
        if 'frame_layout' not in str(type(exc)) and not isinstance(exc, ValueError):
            return frames
    from PIL import Image
    W, H = frames[0].size
    print('animate_route: MIXED FRAME SIZES -- %s' % (
        sorted(sz for sz in sizes if sz != (W, H))[:3],), file=sys.stderr)
    print('animate_route: padding every frame to %dx%d. Pillow would NOT have '
          'raised: it writes a valid GIF in which every later frame has been '
          'silently resized to the first.' % (W, H), file=sys.stderr)
    ground = _pad_rgb(theme)

    def _pad(_i, f):
        if f.size == (W, H):
            return f
        pad = Image.new(f.mode, (W, H), ground)
        pad.paste(f, ((W - f.width) // 2, (H - f.height) // 2))
        return pad
    if frame_spool.is_spool(frames):
        frames.map(_pad, out_size=(W, H))
        return frames
    return [_pad(i, f) for i, f in enumerate(frames)]


#: #1036: the most frames a GIF is handed. Pillow's GIF writer keeps every
#: frame it is given (it diffs each against the last), so a 6000-frame film
#: would be held whole no matter how it was streamed in. Above this the GIF
#: STRIDES, as the owner's `awx/evolve_movie.py` does (`n_frames // 260`),
#: and each kept frame lasts `stride` frame-times so the film keeps its
#: length. The .mp4 is never strided: imageio streams it frame by frame.
GIF_MAX_FRAMES = 260


def save_movie(frames, out, fps, end_hold, png_dir=None, frame_meta=None,
               theme=None):
    """Write the frames to ``out``. Format follows the extension: `.mp4`
    (imageio-ffmpeg; falls back to a sibling `.gif` if unavailable) or `.gif`
    (native Pillow, no dependency).

    ``frames`` is a list of Pillow images or a `frame_spool.FrameSpool`
    (#1036). A spool is STREAMED: the .mp4 and the PNG dump read one frame at
    a time, so memory does not grow with the frame count. Pillow's GIF
    writer holds every frame it is handed, so a GIF over `GIF_MAX_FRAMES` is
    strided, and says so: its memory is bounded by the cap, not flat.

    ``frame_meta`` (#887), when given, is one dict per frame written into the
    dumped PNGs' text chunks. It is indexed against ``frames``, NOT against
    the end-hold repeats appended after them, while the PNG loop iterates
    ``frames``. A short list raises rather than silently misattributing every
    frame after the gap.

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
    frames = _uniform_or_pad(frames, theme)
    n = len(frames)
    n_hold = max(1, int(end_hold * fps))
    W, H = frames[0].size

    def _seq():
        last = None
        for f in frames:
            last = f
            yield f
        for _ in range(n_hold):
            yield last
    ext = os.path.splitext(out)[1].lower()
    if ext == '.mp4' and _write_mp4(_seq(), out, fps):
        print(f"animate_route: wrote {out} ({n} frames @ {fps:g}fps, "
              f"{W}x{H}, h264)")
    else:
        if ext == '.mp4':
            out = os.path.splitext(out)[0] + '.gif'
        dur = max(20, int(1000 / max(0.1, fps)))
        if n > GIF_MAX_FRAMES:
            # EXACTLY the cap (#1036 review: it kept 261): the first and the
            # last frame and evenly spaced ones between, and the end hold is
            # folded into the LAST frame's duration instead of appended as
            # frames of its own.
            cap = GIF_MAX_FRAMES
            keep = sorted({int(round(i * (n - 1) / float(cap - 1)))
                           for i in range(cap)})
            step = (n - 1) / float(cap - 1)
            fdur = max(20, int(round(dur * step)))
            print(f"animate_route: GIF STRIDED -- {n} frames is over the "
                  f"{cap}-frame GIF cap; keeping {len(keep)} evenly spaced "
                  f"frames, {fdur}ms each. The .mp4 is never strided.",
                  file=sys.stderr)
            seq = [frames[i] for i in keep]
            durs = [fdur] * len(seq)
            durs[-1] += dur * n_hold
            seq[0].save(out, save_all=True, append_images=seq[1:],
                        duration=durs, loop=0, optimize=False)
            dur = fdur
        else:
            seq = list(_seq())
            seq[0].save(out, save_all=True, append_images=seq[1:],
                        duration=dur, loop=0, optimize=False)
        del seq
        # Count what LANDED, not what was handed to the encoder. Pillow's GIF
        # writer collapses runs of byte-identical frames into one frame with an
        # accumulated duration, and this film is full of such runs by
        # construction -- card holds and the end hold are literal repeats of a
        # single Image. So `len(frames)` overstates the file: one run printed
        # 377 and wrote 348, and the gap was only found by counting the
        # delivered GIF by hand. A number nobody can reconcile against the
        # artifact is worse than no number.
        _n = n
        try:
            from PIL import Image as _PILImage, ImageSequence
            with _PILImage.open(out) as _chk:
                _n = sum(1 for _ in ImageSequence.Iterator(_chk))
        except Exception:                                       # noqa: BLE001
            pass
        print(f"animate_route: wrote {out} ({_n} frames, {dur}ms each, "
              f"{W}x{H})")
    if png_dir:
        os.makedirs(png_dir, exist_ok=True)
        for i, fr in enumerate(frames):
            # pnginfo=None is Pillow's own PNG default, so the
            # no-metadata path is byte-for-byte what it always was.
            fr.save(os.path.join(png_dir, f'frame_{i:05d}.png'),
                    pnginfo=(_png_info(frame_meta[i]) if frame_meta
                             else None))
        print(f"animate_route: dumped {n} PNG frames to {png_dir}")
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
    ap.add_argument('--layer-alpha', type=int, default=None,
                    help="per-layer copper opacity 1-255. Default: the "
                         "theme's own measured alpha (dark 150, light 205)")
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
