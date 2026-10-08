#!/usr/bin/env python3
"""The film's per-frame stage record -> a timeline a 3D page can draw (#1081).

`animate_route.build_boards(stage_out=)` records, for EVERY frame, where the
film is in the copper edit logs, what is highlighted, what a growth stage
hides under itself, which parts are mid-glide and where, whether the board is
seen from the back, and whether it is mid-flip. This module turns that record
into data a renderer can replay with no knowledge of how the film was made:

  * **copper as lifetimes.** Each edit-log entry becomes one item with the op
    index it was born at and the op index it died at. A frame at log position
    `n` shows exactly the items with `born < n <= died` -- the same set
    `_OpLog.state_at(n)` returns, which `test_1081_timeline` checks frame by
    frame against the X-ray's own draw calls;
  * **frames as small states**: that position, the items a growth stage hides,
    the highlight rows, the moving parts' poses, the board side and flip.
    Frames with identical states are coalesced (`unique`), because a rip hold
    or a caption-only change repeats a picture the renderer need not redraw.

**The board faces the work** (`activity_sides`). A glide faces the side
its moving parts are on, copper faces the layer it lands on, and the board
turns over about a second BEFORE the work begins, so the viewer sees it land.
Work shorter than the dwell does not turn the board (no flip for a stray
event), and an inner layer keeps whatever face is showing. This is the
issue's own rule -- flip for bottom work, flip BACK for top work -- and it is
deliberately not the 2D Stage's: the Stage flips for placement only and never
flips back, so a film whose last copper landed on F.Cu ended face-down with
every one of those segments on the far side (the phase-7 verification). The
3D board is the stage3d film's only board view, so its side is chosen for the
viewer; `side_rule` says which rule ran.
"""
from __future__ import annotations

import hashlib
import json
import math
from typing import List, Optional

#: A glide on the other face turns the board once it lasts this long...
AUTO_DWELL_S = 1.0
#: ...but COPPER must hold a face for longer: a routing film lands chunk
#: after chunk on alternating layers, and at 1 s the board turned 27 times in
#: an 853-frame film (the final review) -- a flip every 5 s, which reads as a
#: fault, not as "the work moved to the back".
COPPER_DWELL_S = 4.0
#: ...and between two turns COPPER decided, the board stays put at least
#: this long. A glide is exempt: it is the placement's own event, and the
#: board must be allowed to turn back for the copper that follows it.
#: (Measured on that 853-frame film: 1 s / no hold 27 turns, 2 s / 2 s 13,
#: 4 s / 6 s 6 -- one turn every ~24 s.)
HOLD_S = 6.0
#: ...and turns over this long.
AUTO_FLIP_S = 1.0


def _gone_marker():
    try:
        from animate_route import _GONE
    except Exception:                                          # noqa: BLE001
        return object()
    return _GONE


_GONE = _gone_marker()


def _is_gone(val) -> bool:
    return val is _GONE


def _row(val, width):
    """A `_Seg`/`_Via` adapter back to its numeric row."""
    if width == 'seg':
        return (float(val.start_x), float(val.start_y), float(val.end_x),
                float(val.end_y), float(val.width), val.layer)
    ls = getattr(val, 'layers', None) or ['F.Cu', 'B.Cu']
    return (float(val.x), float(val.y), float(val.size), float(val.drill),
            ls[0], ls[-1])


def _ops_items(ops, kind):
    items = []
    live = {}
    for i, (key, val) in enumerate(ops):
        j = live.pop(key, None)
        if j is not None:
            items[j][-1] = i          # replaced or removed at op i
        if _is_gone(val):
            continue
        items.append(list(_row(val, kind)) + [i, -1])
        live[key] = len(items) - 1
    return items


class _Replay(object):
    """`{key: item index}` for the items alive after `n` ops, advanced
    INCREMENTALLY: frames ask in log order and the log only grows, so one
    pass over the ops serves every frame. Rebuilding it per hidden frame
    was quadratic -- 3.8 s of a 1099-frame build, and ~2 minutes projected
    for a 5000-frame one (the phase-5 verifier)."""

    def __init__(self, ops):
        self.ops, self.at, self.count, self.live = ops, 0, 0, {}

    def upto(self, n):
        if n < self.at:                 # never in log order; stay correct
            self.at, self.count, self.live = 0, 0, {}
        for key, val in self.ops[self.at:n]:
            self.live.pop(key, None)
            if not _is_gone(val):
                self.live[key] = self.count
                self.count += 1
        self.at = max(self.at, n)
        return self.live


def _key_to_item(ops, n):
    """`{key: item index}` for the items alive after `n` ops."""
    live = {}
    count = 0
    for i, (key, val) in enumerate(ops):
        if i >= n:
            break
        live.pop(key, None)
        if not _is_gone(val):
            live[key] = count
        if not _is_gone(val):
            count += 1
    return live


def _smooth(k):
    return k * k * (3 - 2 * k)


def activity_sides(log, epochs, fps, dwell_s=AUTO_DWELL_S,
                   flip_s=AUTO_FLIP_S, copper_dwell_s=COPPER_DWELL_S,
                   hold_s=HOLD_S):
    """Per-frame flip angle: face the side the film is working on.

    Each frame WANTS a side: the side of the parts gliding on it (their
    resting layer), else the copper layer its event touched (`B.Cu` back,
    `F.Cu` front), else no preference. Runs of one wanted side shorter than
    `dwell_s` are absorbed into the side already showing -- hysteresis, so a
    stray event never flips the board -- and each remaining change turns the
    board over the `flip_s` BEFORE its run starts (never overlapping the
    previous turn), so the work is seen landing face-on. The whole log is
    known before a frame is drawn, which is what makes looking ahead
    deterministic. Returns (angles, turns)."""
    dwell = max(1, int(round(dwell_s * fps)))
    cdwell = max(dwell, int(round(copper_dwell_s * fps)))
    hold = max(1, int(round(hold_s * fps)))
    turn = max(1, int(round(flip_s * fps)))
    want = []
    glide = []
    for r in log:
        glide.append(bool(r.get('moving')))
        w = None
        mv = r.get('moving') or {}
        if mv:
            tab = epochs[r['epoch']] if 0 <= r['epoch'] < len(epochs) else {}
            back = sum(1 for ref in mv
                       if str((tab.get(ref) or [0, 0, 0, 'F.Cu'])[3])
                       .startswith('B'))
            w = 'B' if back * 2 > len(mv) else 'F'
        else:
            a = r.get('active') or ''
            w = 'B' if a == 'B.Cu' else ('F' if a == 'F.Cu' else None)
        want.append(w)
    # runs of a preference: [side, first, end, end of its LAST real work];
    # no-preference frames extend the run before them but are not work
    runs = []
    for i, w in enumerate(want):
        if w is None:
            if runs:
                runs[-1][2] = i + 1
            continue
        if runs and runs[-1][0] == w:
            runs[-1][2] = runs[-1][3] = i + 1
        else:
            runs.append([w, i, i + 1, i + 1])
    # hysteresis: a run shorter than its dwell keeps the side showing, and
    # no turn comes sooner than `hold` after the last
    side = 'F'
    work_end = 0                        # end of the last GLIDE on `side`
    changes = []                        # (start, side, work_end before it)
    last_copper_turn = -hold
    for w, a, b, last in runs:
        by_glide = any(glide[a:b])
        need = dwell if by_glide else cdwell
        if (w != side and b - a >= need
                and (by_glide or a - last_copper_turn >= hold)):
            changes.append((a, w, work_end))
            side = w
            if not by_glide:
                last_copper_turn = a
        if w == side and glide[last - 1]:
            work_end = last
    angles = [0.0] * len(log)
    cur, last_end = 0.0, 0
    turns = 0
    ci = 0
    target = {'F': 0.0, 'B': math.pi}
    # the turn: `turn` frames ending where the new work starts -- but never
    # before the previous side's last GLIDE ends, so a part is never seen
    # turning away mid-move (copper still landing may be seen mid-turn);
    # squeezed into what is left when the gap is short, at worst one frame
    windows = []
    for c, w, prev_end in changes:
        start = max(last_end, prev_end, c - turn)
        end = max(start + 1, c)
        windows.append((start, end, w))
        last_end = end
    for i in range(len(log)):
        while ci < len(windows) and i >= windows[ci][1]:
            cur = target[windows[ci][2]]
            ci += 1
        angles[i] = cur
    for start, end, w in windows:
        frm = target['B' if w == 'F' else 'F']
        span = float(end - start)
        for i in range(start, min(end, len(log))):
            k = _smooth((i - start + 1) / span)
            angles[i] = frm + (target[w] - frm) * k
        turns += 1
    return angles, turns


def build(stage_out, *, fps=6.0, stage_present=True) -> dict:
    """The JSON timeline for one film.

    `stage_out` is what `build_boards(stage_out=)` filled. The result:
    `layers`, `segs` / `vias` (rows with `born`, `died` op indices),
    `epochs` (part poses per board), `frames` (one per film frame, each an
    index into `states`), `states` (the distinct per-frame states) and
    `side_rule` (the activity rule and its turn count)."""
    log = stage_out['log']
    segs = _ops_items(stage_out['ops_s'], 'seg')
    vias = _ops_items(stage_out['ops_v'], 'via')
    layers = list(stage_out['layers'])
    li = {n: i for i, n in enumerate(layers)}
    for s in segs:
        s[5] = li.get(s[5], 0)
    for v in vias:
        v[4], v[5] = li.get(v[4], 0), li.get(v[5], len(layers) - 1)
    angles, nf = activity_sides(log, stage_out.get('epochs') or [], fps)
    rule = ('activity: faces the side being worked on, turning %.1f s '
            'ahead, ignoring a glide under %.1f s and copper under %.1f s, '
            'copper turning at most once per %.1f s -- %d turn(s)'
            % (AUTO_FLIP_S, AUTO_DWELL_S, COPPER_DWELL_S, HOLD_S, nf))
    ops_s = stage_out['ops_s']
    replay = _Replay(ops_s)
    states = []
    index = {}
    frames = []
    for i, r in enumerate(log):
        hide = []
        if r.get('hide'):
            k2i = replay.upto(r['ns'])
            hide = sorted(k2i[k] for k in r['hide'] if k in k2i)
        st = {'ns': r['ns'], 'nv': r['nv'], 'hide': hide,
              'hl_s': [list(x) for x in r.get('hl_s') or ()],
              'hl_v': [list(x) for x in r.get('hl_v') or ()],
              'color': list(r['color']) if r.get('color') else None,
              'epoch': r['epoch'],
              'moving': {k: list(v) for k, v in
                         sorted((r.get('moving') or {}).items())},
              'angle': round(angles[i], 6),
              # the pours revealed so far (#1090), as the 2D film draws them
              'zones': sorted(int(z) for z in (r.get('zones') or ())),
              'active': r.get('active')}
        h = hashlib.sha1(json.dumps(st, sort_keys=True).encode()).hexdigest()
        if h not in index:
            index[h] = len(states)
            states.append(st)
        frames.append(index[h])
    return {'layers': layers, 'segs': segs, 'vias': vias,
            'epochs': [{k: list(v) for k, v in e.items()}
                       for e in stage_out['epochs']],
            'frames': frames, 'states': states, 'side_rule': rule}


def visible(items, n, hide=()) -> List[int]:
    """Indices of the copper items shown at log position `n`: born before
    it, not dead by it, and not hidden by a growth stage."""
    h = set(hide)
    return [i for i, it in enumerate(items)
            if it[-2] < n and (it[-1] < 0 or it[-1] >= n) and i not in h]


def state_for(timeline, frame) -> Optional[dict]:
    fr = timeline['frames']
    if not 0 <= frame < len(fr):
        return None
    return timeline['states'][fr[frame]]
