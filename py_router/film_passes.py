#!/usr/bin/env python3
"""The film's bands and panels, ONE implementation for both front ends (#1087).

`make_movie` and `make_film.build_film` each re-implemented the same
post-pass pipeline -- discover the search behind the film, measure the
placement panels, reserve the band, then compose the benchmark band or the
placement panels -- so every film-level feature had to be threaded twice,
and #1081's stage3d band was the latest to pay that. This module is that
pipeline, in two halves around `animate_route.build_boards`:

  * `plan(...)` runs BEFORE the frame is planned and answers what
    `build_boards` must reserve (`attempts_band`);
  * `compose(...)` runs AFTER, on the frames.

The frame is always stage3d's (the only film layout): ONE band. With a
converge ledger or loop rounds behind the film it is the benchmark band,
which folds the old verdict band and the placement panels into one curve;
with none -- a placement chain made from boards alone -- it is the #1042
placement panels, so such a film keeps its placement numbers.

What stays in each front end is what is genuinely its own: make_movie's run
clock, make_film's badges and cards. Every status line prints with the
caller's `who` prefix, and -- as before -- even when quiet, because a band
with no dialog control has no other way to say whether it ran.
"""
from __future__ import annotations

import os
import sys
from typing import Any, NamedTuple


class Bands(NamedTuple):
    btrack: Any                 # movie_benchmark.BenchTrack, or None
    ptrack: Any                 # movie_placement track, or None
    pwhy: str
    pfn: Any                    # movie_placement.band_px(...)
    band: Any                   # what build_boards reserves (attempts_band)
    placement_asked: Any


def _say(who, msg):
    print(('%s: %s' % (who, msg)) if who else msg, file=sys.stderr)


def plan(steps, final, *, attempts=None, attempts_ledger=None,
         attempts_from=None, placement=None, quiet=False,
         who='make_movie') -> Bands:
    """Everything the frame must reserve, decided before it is planned.

    `attempts`: False is the OFF arm (no benchmark band); anything else
    discovers the search -- `attempts_ledger` (or `placement['ledger']`) if
    named, else beside `attempts_from` or the final board;
    `attempts_from=''` means do not look. `placement`: `{'off', 'ledger',
    'benchmark', 'benchmark_score', 'intent'}`."""
    placement = dict(placement or {})
    here = os.path.dirname(os.path.abspath(final)) if final else ''
    btrack = None
    # #1081. ONE band, the benchmark band, which folds the verdict band and
    # the placement panels into one curve -- neither is measured or
    # reserved beside it.
    try:
        import movie_benchmark
        if attempts is not False and attempts_from != '':
            led = attempts_ledger or placement.get('ledger')
            btrack = (movie_benchmark.from_converge_ledger(led) if led
                      else movie_benchmark.discover(attempts_from or here))
        if btrack is not None and placement.get('benchmark'):
            btrack = movie_benchmark.with_benchmark(
                btrack, movie_benchmark.grade_benchmark(
                    placement['benchmark'],
                    placement.get('benchmark_score')))
    except Exception as exc:                                   # noqa: BLE001
        _say(who or 'make_movie', 'no benchmark band (%s)' % exc)
        btrack = None
    if btrack is not None and len(btrack.points) >= 2:
        return Bands(btrack, None, 'folded into the benchmark band '
                     '(stage3d)', None, True, False)
    # NO ledger behind the film -- a placement chain made from boards alone:
    # the placement panels stay (the final review: such a film used to lose
    # every placement number it had)
    ptrack, pwhy = _placement(steps, placement, attempts_ledger, here, quiet)
    pfn, band = None, False
    if ptrack is not None:
        import movie_placement
        # the band is SIZED for readable panels (`plan_band`), not scaled
        pfn = movie_placement.band_px(ptrack)
        band = pfn
    return Bands(None, ptrack, pwhy, pfn, band, placement.get('asked'))


def _placement(steps, placement, attempts_ledger, here, quiet):
    """`(ptrack, why)`: the #1042 placement panels, measured."""
    if placement.get('off'):
        return None, 'off (--no-placement-panel)'
    try:
        import movie_placement
        led = placement.get('ledger') or attempts_ledger
        if not led and here:
            cand = os.path.join(here, 'ledger.jsonl')
            led = cand if os.path.isfile(cand) else None
        return movie_placement.build_track(
            steps, [], ledger=led, benchmark=placement.get('benchmark'),
            intent=placement.get('intent'), quiet=quiet)
    except Exception as exc:                                   # noqa: BLE001
        return None, 'could not measure (%s)' % exc


def compose(frames, bands, geom, marks, lands, theme, *, quiet=False,
            who='make_movie'):
    """The band, onto frames `build_boards` planned with `bands`: the
    benchmark band, or the placement panels -- before the run clock, so a
    band sits next to the board it annotates."""
    box = geom.track if geom is not None else None
    if bands.btrack is not None:
        try:
            import movie_benchmark
            frames, rep = movie_benchmark.attach(frames, bands.btrack,
                                                 box=box, theme=theme,
                                                 marks=marks)
            _say(who, movie_benchmark.status_line(rep))
        except Exception as exc:                               # noqa: BLE001
            _say(who or 'make_movie', 'no benchmark band (%s)' % exc)
        return frames
    ptrack, pwhy, plan_ = bands.ptrack, bands.pwhy, None
    if ptrack is not None:
        import movie_placement
        pfn = bands.pfn
        plan_ = pfn.plans[-1] if pfn is not None and pfn.plans else None
        if plan_ is not None and plan_.mode == 'declined':
            ptrack, pwhy = None, 'declined: %s' % plan_.why
        elif box is not None:
            ptrack = movie_placement.with_firsts(ptrack, marks, lands)
            frames = movie_placement.compose(frames, box, ptrack, marks,
                                             theme, geom.frame.h)
        else:
            ptrack, pwhy = None, 'no band could be reserved in this frame'
    # SAID whenever a placement was found, drawn or declined
    if bands.ptrack is not None or bands.placement_asked or plan_ is not None:
        import movie_placement
        _say(who, movie_placement.status_line(ptrack, pwhy, plan_))
    return frames
