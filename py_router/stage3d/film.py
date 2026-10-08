#!/usr/bin/env python3
"""Put the 3D board into a stage3d film's board box (#1081).

`animate_route.build_boards` renders every frame through the 2D X-ray as it
always has, and records the per-frame stage state. When the layout is
`stage3d`, `apply()` then renders the 3D board -- EAGERLY, every distinct
state to disk, before a single frame is composed -- and only when that
succeeded in full does it map each board-box frame onto its 3D picture.

Eager, not a lazy per-frame pass, and on purpose: a lazy pass that fails
mid-stream either takes the whole film down (`_write_mp4` deletes a film when
a frame raises) or, as an optional pass, silently switches from 3D to 2D half
way through. Rendered first, the answer is all-or-nothing: the 3D board for
every frame, or the X-ray for every frame and a sentence saying why.
"""
from __future__ import annotations

import atexit
import os
import shutil
import sys
import tempfile

#: `$KICAD_STAGE3D_MODELS=0` skips kicad-cli's real part models (boxes only).
MODELS_ENV = 'KICAD_STAGE3D_MODELS'
#: The state frames live until the film is WRITTEN (the board box maps onto
#: them lazily), so they cannot be removed here. `cleanup()` removes them;
#: the front ends call it after `save_movie`, and `atexit` stays a backstop
#: -- in a long-lived process (KiCad, for the GUI recorder) it would be the
#: only removal, and every film left hundreds of PNGs behind.
_LIVE = []
#: The last `apply` report in this process (#1109): a caller that does not
#: hold the return value (make_film goes through animate_route) can still say
#: which board box the film got. Empty until `apply` runs.
LAST_REPORT = {}


def cleanup():
    """Remove every state-frame directory this process made so far."""
    while _LIVE:
        shutil.rmtree(_LIVE.pop(), ignore_errors=True)


def _say(msg, notes):
    print(msg, file=sys.stderr)
    if notes is not None:
        notes.append(msg)


def apply(frames, stage_out, final, geom, theme, *, stage_present,
          mode='auto', notes=None, models=None, fps=6.0):
    """`_apply`, with its report kept in `LAST_REPORT` (#1109)."""
    LAST_REPORT.clear()
    frames, report = _apply(frames, stage_out, final, geom, theme,
                            stage_present=stage_present, mode=mode,
                            notes=notes, models=models, fps=fps)
    LAST_REPORT.clear()
    LAST_REPORT.update(report, asked=mode)
    return frames, report


def _apply(frames, stage_out, final, geom, theme, *, stage_present,
           mode='auto', notes=None, models=None, fps=6.0):
    """Map `frames` (board-box images, a list or a FrameSpool) onto the 3D
    board. Returns `(frames, report)`; on ANY failure the frames are the
    X-ray's, untouched, and `report['why']` says why. `mode` 'off' asks for
    the X-ray."""
    report = {'mode': '2d', 'why': ''}
    if mode in ('2d', 'off'):
        report['why'] = 'the 2D X-ray was asked for'
        _say('stage3d: 2D X-ray in the board box -- %s' % report['why'],
             notes)
        return frames, report
    try:
        from stage3d import render3d, scene as SC, timeline as TL
        from kicad_parser import parse_kicad_pcb
        import frame_spool
    except Exception as exc:                                   # noqa: BLE001
        report['why'] = 'could not load the 3D renderer (%s)' % exc
        _say('stage3d: 2D X-ray in the board box -- %s' % report['why'],
             notes)
        return frames, report
    # #1089: 'blender' is the hi-fi backend (Cycles on the CPU), 'auto'
    # the three.js one
    backend = 'blender' if mode == 'blender' else 'three'
    ok, why = render3d.available(backend)
    if not ok:
        report['why'] = why
        _say('stage3d: 2D X-ray in the board box -- %s' % why, notes)
        return frames, report
    tmp = tempfile.mkdtemp(prefix='krt_stage3d_')
    _LIVE.append(tmp)
    atexit.register(shutil.rmtree, tmp, True)
    try:
        tl = TL.build(stage_out, fps=fps or 6.0,
                      stage_present=stage_present)
        pcb = parse_kicad_pcb(final)
        sc = SC.build_scene(pcb)
        glb, gwhy = None, 'part models off ($%s=0)' % MODELS_ENV
        want_models = (models if models is not None
                       else os.environ.get(MODELS_ENV, '1') != '0')
        if want_models:
            glb, gwhy = SC.export_glb(final, pcb, os.path.join(tmp, 'glb'))
        if glb:
            sc['glb'] = {k: v for k, v in glb.items() if k != 'path'}
        pngs, info, rwhy = render3d.render(
            sc, tl, width=geom.board.w, height=geom.board.h,
            out_dir=tmp, theme=getattr(theme, 'name', theme) or None,
            glb=glb, backend=backend)
    except Exception as exc:                                   # noqa: BLE001
        pngs, info, rwhy = None, {}, 'the 3D board failed (%s)' % (
            str(exc).splitlines()[0][:160] if str(exc) else
            type(exc).__name__)
        gwhy = ''
    if not pngs:
        report['why'] = rwhy
        _say('stage3d: 2D X-ray in the board box -- %s' % rwhy, notes)
        return frames, report
    order = tl['frames']
    if len(order) != len(frames):
        report['why'] = ('the stage record has %d frames and the film %d'
                         % (len(order), len(frames)))
        _say('stage3d: 2D X-ray in the board box -- %s' % report['why'],
             notes)
        return frames, report
    from PIL import Image
    W, H = geom.board.w, geom.board.h

    def _to_3d(i, _f):
        with Image.open(pngs[order[i]]) as im:
            out = im.convert('RGB')
        return out if out.size == (W, H) else out.resize((W, H))
    frames = frame_spool.transform(frames, _to_3d, out_size=(W, H))
    report.update(mode='3d', why=rwhy, models=gwhy, side=tl['side_rule'],
                  states=len(tl['states']), renderer=info.get('renderer'),
                  tools=info.get('tools'))
    _say('stage3d: 3D board -- %s; %s; side: %s' % (
        rwhy, gwhy or 'no part models', tl['side_rule']), notes)
    return frames, report
