#!/usr/bin/env python3
"""The whole-route film's sound track (awx/whole_movie_sound.py), the parts of it that need no voice and no ffmpeg:
the film's timeline fitted to its narration, the cues put on that timeline and thinned, and the report that no line
runs into the next.

  python3 tests/test_622_whole_movie_sound.py

On made-up scenes whose lines are given as voiced (samples of a known length, no voice spoken), this pins:

1. the default voice is the neural one, DEFAULT_VOICE, and narrate's own default is that constant;
2. a scene whose lines fit in it keeps its own length, and its moments map to film seconds one for one;
3. a line longer than the stretch up to the next line slows the scene over that stretch only: the next line starts
   no earlier than LEAD + the line + GAP after the line's own start;
4. a line with a t_end is done GAP before the scene reaches t_end, and the rest of the scene after it keeps its pace;
5. a last line longer than the scene's rest holds the last frame: the scene grows by what the line needs;
6. report() checks every line against the next, its t_end and the scene's end (and passes on these);
7. a cue lands where its scene's moment is on the fitted timeline; cues of one kind closer than THIN are thinned to
   the earlier; a scene change adds a 'stage' where a new stage lights up and a 'turn' between two scenes of one stage.
"""
import contextlib
import io
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))

import whole_movie_sound as wms  # noqa: E402

fails = []


def check(ok, what):
    print(('ok   ' if ok else 'FAIL ') + what)
    if not ok:
        fails.append(what)


class Sc:
    """a scene as the sound reads it: key, stage, secs, says, cues -- and, once 'voiced', each line's samples"""

    def __init__(self, key, stage, secs, lines, cues=()):
        self.key, self.stage, self.secs, self.cues = key, stage, secs, list(cues)
        self.knots = None
        # (each line: (t, seconds it lasts[, t_end]) -- voiced as samples i0..i1 of that length)
        self.voiced = [(t, f'line {j}', '-', 0, int(round(d * wms.SR)), (u[0] if u else None))
                       for j, (t, d, *u) in enumerate(lines)]

    @property
    def span(self):
        return self.knots[-1][0] if self.knots else self.secs


L, G = wms.LEAD, wms.GAP
# 1. the default voice
import inspect  # noqa: E402
check(wms.DEFAULT_VOICE == 'kokoro:am_puck', f'1: the default voice is the neural one: {wms.DEFAULT_VOICE}')
check(inspect.signature(wms.narrate).parameters['voice'].default == wms.DEFAULT_VOICE,
      '1: narrate defaults to DEFAULT_VOICE')

a = Sc('a', 'one', 10.0, [(0.0, 2.0), (0.5, 2.0)])                       # fits: 10 s gives each line 5 s
b = Sc('b', 'one', 10.0, [(0.0, 7.0), (0.3, 1.0)])                       # the first line needs 7.8 s, has 3 s
c = Sc('c', 'two', 10.0, [(0.0, 4.0, 0.2), (0.6, 1.0)])                  # done by t 0.2 (2 s), needs 4.8 s
d = Sc('d', 'two', 4.0, [(0.5, 5.0)])                                    # a last line longer than the rest of it
scenes = [a, b, c, d]
wms.fit(scenes)

# 2. a scene that fits
check(abs(a.span - 10.0) < 1e-9, f'2: a scene whose lines fit keeps its length: {a.span:.3f}')
check(all(abs(wms.moment(a, t) - 10.0 * t) < 1e-9 for t in (0.0, 0.25, 0.5, 0.9)), '2: its moments one for one')

# 3. a long line slows its stretch only
need = L + 7.0 + G
check(abs(wms.moment(b, 0.3) - need) < 1e-6, f'3: the next line\'s moment after the long line: {wms.moment(b, 0.3):.3f}'
      f' (needs {need:.3f})')
check(abs(b.span - (need + 0.7 * 10.0)) < 1e-6, f'3: the rest of the scene at its own pace: {b.span:.3f}')

# 4. a line with a t_end
check(wms.moment(c, 0.2) >= L + 4.0 + G - 1e-6, f'4: the line is done GAP before its t_end: {wms.moment(c, 0.2):.3f}')
check(abs((wms.moment(c, 0.6) - wms.moment(c, 0.2)) - 4.0) < 1e-6, '4: from t_end to the next line, the scene\'s own pace')

# 5. the last frame held
check(abs(d.span - (0.5 * 4.0 + L + 5.0 + G)) < 1e-6, f'5: the last frame held for the last line: {d.span:.3f}')
check(abs(wms.moment(d, 1.0) - 4.0) < 1e-6, '5: the scene\'s last moment where its own length ends')

# 6. the report
buf = io.StringIO()
try:
    with contextlib.redirect_stdout(buf):
        wms.report(scenes)
    ok = True
except AssertionError as e:
    ok = False
    print(e)
nl_ = sum(1 for ln in buf.getvalue().splitlines() if ln.lstrip().startswith('line ') and ' s + ' in ln)
check(ok and nl_ == 7, f'6: report() passes, every line listed: {nl_} of 7')
bad = Sc('bad', 'one', 4.0, [(0.0, 1.0), (0.5, 1.0)])
bad.knots = [(0.0, 0.0), (4.0, 1.0)]              # (not fitted)
bad.said = [(L, bad.voiced[0]), (L + 0.5, bad.voiced[1])]               # (the second line put on top of the first)
try:
    with contextlib.redirect_stdout(io.StringIO()):
        wms.report([bad])
    caught = False
except AssertionError:
    caught = True
check(caught, '6: control: a line that runs into the next is refused by report()')

# 7. the cues on the timeline
e = Sc('e', 'one', 2.0, [])
f = Sc('f', 'one', 2.0, [], cues=[(0.5, 'tick', 0.5), (0.51, 'tick', 0.5), (0.9, 'tick', 0.5)])
g = Sc('g', 'two', 2.0, [], cues=[(0.25, 'ping', 0.5)])
for s_ in (e, f, g):
    s_.knots = None
    s_.said = []
ev = wms.timeline([e, f, g])
ticks = [x for x in ev if x[1] == 'tick']
check(len(ticks) == 2, f'7: two ticks 0.02 s apart thinned to one: {len(ticks)} of 3 left')
check(abs(ticks[0][0] - 3.0) < 1e-9 and abs(ticks[1][0] - 3.8) < 1e-9, f'7: the ticks at their moments: '
      f'{[round(x[0], 3) for x in ticks]}')
check([(round(x[0], 3), x[1]) for x in ev if x[1] in ('turn', 'stage')] == [(2.0, 'turn'), (4.0, 'stage')],
      '7: a turn between two scenes of one stage, a stage bell where the next lights up')
ping = [x for x in ev if x[1] == 'ping']
check(len(ping) == 1 and abs(ping[0][0] - 4.5) < 1e-9, '7: a cue of a later scene after the scenes before it')
# (and on a fitted scene, a cue at a line's moment lands where the line starts, less LEAD)
h = Sc('h', 'one', 10.0, [(0.0, 7.0), (0.3, 1.0)], cues=[(0.3, 'mark', 0.5)])
wms.fit([h])
mk = [x for x in wms.timeline([h]) if x[1] == 'mark']
check(len(mk) == 1 and abs(mk[0][0] - (h.said[1][0] - L)) < 1e-6, '7: a cue on a fitted scene with its line')
check(np.isfinite([x[0] for x in ev]).all(), '7: every effect has a time')

print('ALL PASS' if not fails else f'{len(fails)} FAILED')
sys.exit(1 if fails else 0)
