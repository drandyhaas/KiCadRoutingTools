#!/usr/bin/env python3
"""The 3D board replays the X-ray's frames, event for event (#1081).

`stage3d.timeline` rebuilds each frame's copper from the stage record alone
(a log position, the keys a growth stage hides, the highlight rows). The
claim that the 3D board shows "exactly the events the 2D X-ray shows" is only
as good as that rebuild, so this file checks it against the X-ray's OWN draw
calls: `BoardRenderer.frame` is wrapped, every argument it receives on a
Movie frame is recorded, and for every frame of a real film the timeline must
rebuild the same segments, the same vias, the same highlights -- on a film
with a flip, a glide, a rip-and-retract and a growth stage.

It also pins the side rules: with a Stage, the 3D board turns on the Stage's
own flip frames and nowhere else; without one, `activity_sides` turns only after
the back-side work has held for the dwell, never for a stray event.
"""
import json
import math
import os
import sys

_TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(_TESTS)
for _p in (ROOT, _TESTS, os.path.join(ROOT, 'py_router')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import importlib.util                                           # noqa: E402
if importlib.util.find_spec('PIL') is None:
    print('SKIP: needs Pillow')
    sys.exit(77)

import film_chain_1081 as FC                                    # noqa: E402
import animate_route as A                                       # noqa: E402
import route_render as RR                                       # noqa: E402
from stage3d import timeline as TL                              # noqa: E402

_FAIL = []


def _check(ok, msg):
    print('  %s %s' % ('ok  ' if ok else 'FAIL', msg))
    if not ok:
        _FAIL.append(msg)


def _r(x):
    return round(float(x), 4)


def _seg_row(s):
    return (_r(s.start_x), _r(s.start_y), _r(s.end_x), _r(s.end_y),
            _r(s.width), s.layer)


def _via_row(v):
    ls = getattr(v, 'layers', None) or ['F.Cu', 'B.Cu']
    return (_r(v.x), _r(v.y), _r(v.size), _r(v.drill), ls[0], ls[-1])


#: The parts the fixture moves (C26 on the front, U1 -- turning -- on the
#: back): their pose on every frame is checked against the X-ray's own.
POSED = ('C26', 'U1')


def _film_with_calls(boards, **kw):
    """Film, capturing what `BoardRenderer.frame` drew for each Movie frame."""
    calls = []
    last = {}
    orig_frame = RR.BoardRenderer.frame
    orig_push = A.Movie._push_frame

    def _frame(self, segments=None, vias=None, highlight_segments=None,
               highlight_vias=None, **k):
        fps = getattr(self.pcb, 'footprints', {}) or {}
        last['pose'] = {ref: (fp.x, fp.y, fp.rotation or 0.0)
                        for ref, fp in fps.items() if ref in POSED}
        last['c'] = (sorted(_seg_row(s) for s in (segments or ())),
                     sorted(_via_row(v) for v in (vias or ())),
                     sorted(_seg_row(s) for s in (highlight_segments or ())),
                     sorted(_via_row(v) for v in (highlight_vias or ())))
        return orig_frame(self, segments=segments, vias=vias,
                          highlight_segments=highlight_segments,
                          highlight_vias=highlight_vias, **k)

    def _push(self, img, label, kind, **k):
        calls.append((kind, last.get('c'), last.get('pose')))
        return orig_push(self, img, label, kind, **k)
    RR.BoardRenderer.frame, A.Movie._push_frame = _frame, _push
    try:
        out = {}
        frames, _m, st, _g = FC.film(boards, stage_out=out, **kw)
    finally:
        RR.BoardRenderer.frame, A.Movie._push_frame = orig_frame, orig_push
    return frames, st, out, calls


def _rebuilt(tl, st):
    names = tl['layers']
    segs = sorted((_r(s[0]), _r(s[1]), _r(s[2]), _r(s[3]), _r(s[4]),
                   names[s[5]])
                  for i, s in enumerate(tl['segs'])
                  if i in set(TL.visible(tl['segs'], st['ns'], st['hide'])))
    vias = sorted((_r(v[0]), _r(v[1]), _r(v[2]), _r(v[3]), names[v[4]],
                   names[v[5]])
                  for v in (tl['vias'][i]
                            for i in TL.visible(tl['vias'], st['nv'])))
    hl_s = sorted((_r(h[0]), _r(h[1]), _r(h[2]), _r(h[3]), _r(h[4]),
                   names[int(h[5])]) for h in st['hl_s'])
    hl_v = sorted((_r(h[0]), _r(h[1]), _r(h[2]), _r(h[3]), names[int(h[4])],
                   names[int(h[5])]) for h in st['hl_v'])
    return segs, vias, hl_s, hl_v


def test_every_frame_is_rebuilt_from_the_record_alone():
    with FC.Chain(rot_u1=90.0) as c:
        # a rip-and-retract and a regrowth, after the flip: re-reveal the
        # copper board through a board with a chunk of it removed
        tr = FC.rip_trace(c.boards[-1], os.path.join(c.dir, 'trace.json'))
        frames, st, out, calls = _film_with_calls(c.boards, traces={3: tr})
        tl = TL.build(out)
        # the record must survive JSON: the page receives it that way
        tl = json.loads(json.dumps(tl))
        _check(len(tl['frames']) == len(frames) == len(calls),
               'one timeline frame per film frame (%d / %d / %d)'
               % (len(tl['frames']), len(frames), len(calls)))
        bad = []
        checked = 0
        pose_bad = []
        for i, (kind, call, pose) in enumerate(calls):
            if kind == 'flip' or call is None:
                continue          # a flip frame is the Stage's own picture
            # the PARTS, from the record alone: the epoch's resting pose,
            # overridden by `moving` mid-glide -- the phase-5 verifier found
            # every pose mutation survived a check that read only copper
            stt = TL.state_for(tl, i)
            ep = tl['epochs'][stt['epoch']]
            for ref in POSED:
                want = (pose or {}).get(ref)
                got = stt['moving'].get(ref) or (ep.get(ref) or [None])[:3]
                if want is None or got is None or any(
                        abs(float(a) - float(b)) > 1e-9
                        for a, b in zip(got, want)):
                    pose_bad.append((i, ref, got, want))
            checked += 1
            got = _rebuilt(tl, TL.state_for(tl, i))
            for what, a, b in zip(('segments', 'vias', 'highlights',
                                   'via highlights'), got, call):
                if a != b:
                    bad.append((i, what, len(a), len(b)))
        _check(checked > 40, 'checked %d Movie frames' % checked)
        _check(not pose_bad, 'every frame\'s part poses rebuilt from the '
               'record equal the X-ray\'s, turning glide included (first '
               'mismatches: %s)' % pose_bad[:3])
        _check(not bad, 'every frame\'s copper rebuilt from the record '
               'alone equals what the X-ray drew (first mismatches: %s)'
               % bad[:4])
        grown = [i for i, s in enumerate(tl['frames'])
                 if tl['states'][s]['hide']]
        _check(grown, '%d frames hide a growth stage\'s finished copper '
               '(the fixture must exercise it)' % len(grown))
        died = sum(1 for s in tl['segs'] if s[-1] >= 0)
        _check(died >= 24, '%d copper items die (the rip)' % died)


def test_the_board_faces_the_work_and_turns_back():
    """#1081: the board flips for bottom-side work and flips BACK for top
    work. The Stage alone never flips back -- a film whose last copper
    landed on F.Cu ended face-down (the phase-7 verification) -- so the 3D
    board follows the activity: U1's back-side glide is seen from the back,
    the F.Cu copper that follows from the front, the B.Cu after it from the
    back, and each turn completes BEFORE its work starts."""
    with FC.Chain() as c:
        out = {}
        tr = FC.rip_trace(c.boards[-1], os.path.join(c.dir, 'tr.json'))
        _f, _m, st, _g = FC.film(c.boards, stage_out=out, traces={3: tr})
        tl = TL.build(out)
        ang = [tl['states'][s]['angle'] for s in tl['frames']]
        log = out['log']
        glide_b = [i for i in range(len(log))
                   if 'U1' in (log[i].get('moving') or {})]
        f_cu = [i for i in range(len(log)) if log[i].get('kind') == 'frame'
                and log[i].get('active') == 'F.Cu']
        b_cu = [i for i in range(len(log)) if log[i].get('kind') == 'frame'
                and log[i].get('active') == 'B.Cu']
        _check(glide_b and all(abs(ang[i] - math.pi) < 1e-5
                               for i in glide_b),
               'U1\'s back-side glide is seen from the back, every frame '
               '(%s)' % [round(ang[i], 2) for i in glide_b])
        first_f = f_cu[:6]
        _check(first_f and all(abs(ang[i]) < 1e-5 for i in first_f),
               'the F.Cu copper that follows lands face-on, the board turned '
               'back (%s)' % [round(ang[i], 2) for i in first_f])
        # the LONGEST run of B.Cu work (the rule ignores copper that lands
        # for under COPPER_DWELL_S): the board faces the back when it starts
        runs, cur = [], []
        for i in b_cu:
            if cur and i != cur[-1] + 1:
                runs.append(cur)
                cur = []
            cur.append(i)
        if cur:
            runs.append(cur)
        longest = max(runs, key=len) if runs else []
        need = int(TL.COPPER_DWELL_S * 6.0)
        # this fixture's B.Cu copper lands in runs SHORTER than the dwell,
        # so the board must not turn for it (a sustained run that does turn
        # it is test_the_side_rule_waits_out_a_stray_event's)
        _check(longest and len(longest) < need
               and all(abs(ang[i]) < 1e-5 for i in longest),
               'and a short burst of B.Cu copper (%d frames, under %d) does '
               'NOT turn the board over' % (len(longest), need))
        _check(tl['side_rule'].startswith('activity:'),
               'the rule is named (%r)' % tl['side_rule'])


def _rec(active=None, moving=None, epoch=0):
    return {'active': active, 'moving': moving or {}, 'epoch': epoch}


def test_the_side_rule_waits_out_a_stray_event():
    fps = 6.0
    ep = [{'U1': [0, 0, 0, 'B.Cu'], 'C1': [0, 0, 0, 'F.Cu']}]
    rec = ([_rec('F.Cu')] * 20 + [_rec('B.Cu')] * 2
           + [_rec('F.Cu')] * 20 + [_rec('B.Cu')] * 30
           + [_rec('In1.Cu')] * 10)
    ang, turns = TL.activity_sides(rec, ep, fps)
    _check(len(ang) == len(rec), 'one angle per frame')
    turn = int(TL.AUTO_FLIP_S * fps)
    _check(all(v == 0.0 for v in ang[:42 - turn]),
           'a 2-frame stray back-side event does not flip the board')
    _check(turns == 1, 'one turn for the sustained back-side work (%d)'
           % turns)
    _check(all(abs(v - math.pi) < 1e-9 for v in ang[42:]),
           'the board already faces the back when the work starts, and an '
           'inner-layer event keeps it there')
    _check(0 < ang[42 - turn] < math.pi, 'the turn happens BEFORE the work')
    # a glide faces its parts' side, whatever the copper last touched
    rec = [_rec('F.Cu')] * 12 + [_rec(moving={'U1': [1, 1, 0]})] * 10
    ang, _t = TL.activity_sides(rec, ep, fps)
    _check(all(abs(v - math.pi) < 1e-9 for v in ang[12:]),
           'a back-side glide is watched from the back')


def test_the_revealed_pours_ride_the_timeline():
    """#1090: the film's revealed plane nets reach the 3D board per frame."""
    rec = {'kind': 'frame', 'ns': 0, 'nv': 0, 'hide': (), 'hl_s': (),
           'hl_v': (), 'color': None, 'mark': 'solid', 'zones': (3, 1),
           'epoch': 0, 'moving': {}, 'mirror': False, 'flip': None,
           'view': None, 'active': None}
    tl = TL.build({'log': [dict(rec, zones=()), rec], 'epochs': [{}],
                   'layers': ['F.Cu', 'B.Cu'], 'ops_s': [], 'ops_v': []})
    _check([TL.state_for(tl, i)['zones'] for i in (0, 1)] == [[], [1, 3]],
           'zones per frame: %s' % [TL.state_for(tl, i)['zones']
                                    for i in (0, 1)])


def test_a_flat_model_path_is_found_by_its_name():
    """esp_prog references `${KISYS3DMOD}/R_0402_1005Metric.wrl` FLAT, while
    KiCad 10 keeps `Resistor_SMD.3dshapes/R_0402_1005Metric.step`: its 16
    references found 0 models, so every part was a box (run 35). A model the
    path cannot find is looked up by name in the LIBRARY dirs -- never in
    the board's own folder -- and one found nowhere is left as written."""
    import shutil
    import tempfile
    from stage3d import scene as SC
    tmp = tempfile.mkdtemp(prefix='t1081m_')
    try:
        lib = os.path.join(tmp, 'lib')
        prj = os.path.join(tmp, 'prj')
        os.makedirs(os.path.join(lib, 'Resistor_SMD.3dshapes'))
        os.makedirs(prj)
        want = os.path.join(lib, 'Resistor_SMD.3dshapes',
                            'R_0402_1005Metric.step')
        open(want, 'w').close()
        open(os.path.join(prj, 'QSG5032.step'), 'w').close()
        text = ('(model "${KISYS3DMOD}/R_0402_1005Metric.wrl")\n'
                '(model "${KISYS3DMOD}/QSG5032.step")\n')
        out, twins, named = SC.stage_models(
            text, {'KISYS3DMOD': lib, 'KIPRJMOD': prj}, prj)
        _check(named == 1 and want.replace('\\', '/') in out,
               'the flat .wrl reference is found as its library .step (%d, '
               '%r)' % (named, out.splitlines()[0]))
        _check('"${KISYS3DMOD}/QSG5032.step"' in out,
               "a model only the board's own folder has is not found by "
               'name there, and stays as written')
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_a_part_nothing_is_placed_on_has_no_body():
    """A part with no 3D model that KiCad excludes from pick-and-place (or
    marks board-only) is copper, not a component: a USB-A plug made of board
    traces, a PCB antenna, a Tag-Connect footprint. A box drawn on it read as
    a part that is not there (StickHub's J1, run 36). An assembled part with
    no model still gets its box."""
    from kicad_parser import parse_kicad_pcb
    from stage3d import scene as SC
    sc = SC.build_scene(parse_kicad_pcb(os.path.join(
        FC.ROOT, 'kicad_files', 'ulx3s.kicad_pcb')))
    _check(sc['parts']['AE1']['body'] is None,
           "ulx3s AE1 (a PCB antenna, not placed) has no body")
    sc = SC.build_scene(parse_kicad_pcb(os.path.join(
        FC.ROOT, 'kicad_files', 'rp2350_fpga_eensy_prePlane.kicad_pcb')))
    _check(sc['parts']['J2']['body'] is None
           and sc['parts']['U1']['body'] is not None,
           'rp2350 J2 (a Tag-Connect footprint) has no body; U1 keeps one')
    sc = SC.build_scene(parse_kicad_pcb(os.path.join(
        FC.ROOT, 'kicad_files', 'esp_prog.kicad_pcb')))
    _check(sc['parts']['Ref*']['body'] is not None,
           'esp_prog\'s fiducial (no model, but assembled) keeps its box')


TESTS = (
    test_a_part_nothing_is_placed_on_has_no_body,
    test_every_frame_is_rebuilt_from_the_record_alone,
    test_the_board_faces_the_work_and_turns_back,
    test_the_side_rule_waits_out_a_stray_event,
    test_the_revealed_pours_ride_the_timeline,
    test_a_flat_model_path_is_found_by_its_name,
)


def main():
    for fn in TESTS:
        print('%s:' % fn.__name__)
        fn()
    if _FAIL:
        print('')
        print('%d FAILURE(S)' % len(_FAIL))
        for msg in _FAIL:
            print('  - %s' % msg)
        return 1
    print('')
    print('all %d checks passed' % len(TESTS))
    return 0


if __name__ == '__main__':
    sys.exit(main())
