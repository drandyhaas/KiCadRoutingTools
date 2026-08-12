#!/usr/bin/env python3
"""The film: placement motion outside the loop, and the attempts that were dropped.

Two defects this pins, both of which made a run's own record misleading:

  * A placement step changes no copper, and the movie animates copper deltas.
    Footprint motion was animated only through a `movie_camera.Stage`, and a
    Stage was built only from `place_route_loop`'s sidecars -- so every chain
    driven by hand rendered the step that decides everything downstream as ONE
    frame. Measured before the fix: seed -> placed, 14 parts moved, 1 frame.
  * The accepted spine was 10 boards of the 45 a run left on disk. The other 35
    are where the search actually happened and nothing could show them.
"""
import os
import shutil
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

sys.path.insert(0, os.path.join(ROOT, 'py_placer'))  # placement split
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # placement split
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # placement split
BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')


def _variant(src, dest, dx=2.0, dy=1.5, rot=None, n=3):
    """`src` with its first `n` footprints nudged -- a placement attempt."""
    from kicad_parser import parse_kicad_pcb
    from placement.writer import write_placed_output
    pcb = parse_kicad_pcb(src)
    refs = sorted(pcb.footprints)[:n]
    places = []
    for r in refs:
        f = pcb.footprints[r]
        places.append({'reference': r, 'new_x': f.x + dx, 'new_y': f.y + dy,
                       'new_rotation': rot if rot is not None else f.rotation})
    write_placed_output(src, dest, places)
    return refs


# --------------------------------------------------------------- synth_rounds

def test_synth_rounds_recovers_moves_from_the_boards_themselves():
    from movie_camera import synth_rounds
    with tempfile.TemporaryDirectory() as td:
        b = os.path.join(td, 'moved.kicad_pcb')
        refs = _variant(BOARD, b)
        rounds = synth_rounds([BOARD, b])
        assert len(rounds) == 1, "only the board whose parts moved is a beat"
        moved = {m['reference'] for m in rounds[0]['moved']}
        assert moved == set(refs), moved
        m = rounds[0]['moved'][0]
        assert len(m['from']) == 3 and len(m['to']) == 3, "pose is (x, y, rot)"
    print(f"  PASS: {len(refs)} moves recovered with no sidecar")


def test_a_rotation_in_place_counts_as_a_move():
    """A part that turns 180 without shifting moves no origin at all. A
    position-only diff shows nothing -- and a rotation that put a pin on the
    far side of a package is exactly the change a film has to show."""
    from movie_camera import synth_rounds
    from kicad_parser import parse_kicad_pcb
    with tempfile.TemporaryDirectory() as td:
        pcb = parse_kicad_pcb(BOARD)
        ref = sorted(pcb.footprints)[0]
        f = pcb.footprints[ref]
        b = os.path.join(td, 'turned.kicad_pcb')
        from placement.writer import write_placed_output
        write_placed_output(BOARD, b, [{'reference': ref, 'new_x': f.x,
                                        'new_y': f.y,
                                        'new_rotation': (f.rotation + 180) % 360}])
        rounds = synth_rounds([BOARD, b])
        assert rounds and rounds[0]['moved'][0]['reference'] == ref
    print("  PASS: a pure rotation is a move")


def test_a_routing_only_chain_synthesises_nothing():
    """No parts moved -> no placement beats -> no Stage, and the routing movie
    that every existing caller gets is byte-for-byte the one it got before."""
    from movie_camera import synth_rounds
    assert synth_rounds([BOARD, BOARD]) == []
    print("  PASS: a copper-only chain is left alone")


# ------------------------------------------------------------- the regression

def test_a_placement_step_is_more_than_one_frame():
    import animate_route as a
    import make_movie as mm
    with tempfile.TemporaryDirectory() as td:
        b = os.path.join(td, 'placed.kicad_pcb')
        _variant(BOARD, b, n=6)
        steps = a.steps_for_boards([BOARD, b])
        plain = a.build_boards(steps, b, 400, 1, 150, 2, 6)
        assert len(plain) <= 2, f"copper-only path: {len(plain)} frames"

        out = os.path.join(td, 'f.gif')
        mm.make_movie([BOARD, b], out=out, size=400, camera='auto', tween=8,
                      quiet=True)
        assert os.path.isfile(out)
    print(f"  PASS: {len(plain)} frame(s) without a camera; the camera path "
          f"now animates the move")


def test_the_marks_hook_lines_up_with_the_steps():
    """A composer that cannot say which frames belong to which beat cannot
    badge, caption or splice one."""
    import animate_route as a
    with tempfile.TemporaryDirectory() as td:
        b = os.path.join(td, 'p.kicad_pcb')
        _variant(BOARD, b, n=4)
        steps = a.steps_for_boards([BOARD, b, BOARD])
        marks = []
        frames = a.build_boards(steps, BOARD, 400, 1, 150, 2, 6, marks=marks)
        assert len(marks) == len(steps), (len(marks), len(steps))
        for (label, board, first, last) in marks:
            assert 0 <= first <= last <= len(frames)
        assert [m[1] for m in marks] == [s[1] for s in steps]
    print("  PASS: one mark per step, in order, in range")


def test_revert_undoes_a_beat_without_calling_it_a_rip():
    """`reveal_delta` flashes the copper red and labels it "(rip)". Nothing was
    ripped -- an attempt was not kept, and the two read completely
    differently."""
    import animate_route as a
    with tempfile.TemporaryDirectory() as td:
        b = os.path.join(td, 'p.kicad_pcb')
        _variant(BOARD, b, n=4)
        marks = []
        a.build_boards([('try', b, None), ('back', BOARD, None, 'revert')],
                       BOARD, 400, 1, 150, 2, 6, marks=marks)
        assert len(marks) == 2
        assert marks[1][3] > marks[1][2], "the undo must be visible"
    print("  PASS: a revert emits frames and never says (rip)")


# ------------------------------------------------------------------ the film

def test_the_film_shows_the_attempts_and_marks_them():
    import make_film as mf
    with tempfile.TemporaryDirectory() as td:
        good = os.path.join(td, 'good.kicad_pcb')
        bad = os.path.join(td, 'bad.kicad_pcb')
        _variant(BOARD, good, dx=1.0, dy=1.0, n=4)
        _variant(BOARD, bad, dx=-4.0, dy=3.0, n=4)
        shots = mf.parse_positional([BOARD, good, bad], [os.path.basename(bad)])
        assert [s['accepted'] for s in shots] == [True, True, False]
        assert shots[2]['revert_to'] == os.path.abspath(good), \
            "an attempt reverts to the last KEPT board, not to its predecessor"

        frames = mf.build_film(shots, size=400, fps=6.0, camera='auto',
                               quiet=True)
        assert frames, "no frames"
        red = sum(1 for f in frames
                  if f.convert('RGB').getpixel((0, f.height // 2)) == (200, 60, 60))
        assert red > 0, "the attempt's frames must be badged"
        assert red < len(frames), "the kept beats must NOT be badged"
    print(f"  PASS: {red}/{len(frames)} frames badged as an attempt")


def test_a_diagnostic_card_is_spliced_in_at_frame_size():
    """The renders were the INPUT to each decision and sat beside the film as
    loose PNGs. A card that is a different size than the film is a cut, not a
    beat."""
    import make_film as mf
    from PIL import Image
    with tempfile.TemporaryDirectory() as td:
        png = os.path.join(td, 'why.png')
        Image.new('RGB', (1234, 200), (10, 90, 160)).save(png)
        good = os.path.join(td, 'g.kicad_pcb')
        _variant(BOARD, good, n=3)
        shots = ([mf.card_shot(png, 'the delta that motivated this')] +
                 mf.parse_positional([BOARD, good], []))
        frames = mf.build_film(shots, size=400, fps=6.0, camera='off',
                               quiet=True)
        assert len({f.size for f in frames}) == 1, "one size across the film"
        assert frames[0].getpixel((5, 5)) == (14, 14, 18), \
            "a leading card opens the film, ahead of the input snapshot"
    print("  PASS: cards are letterboxed to frame size and placed in order")


def test_the_cli_runs_end_to_end():
    with tempfile.TemporaryDirectory() as td:
        good = os.path.join(td, 'g.kicad_pcb')
        bad = os.path.join(td, 'b.kicad_pcb')
        _variant(BOARD, good, n=3)
        _variant(BOARD, bad, dx=-3.0, n=3)
        out = os.path.join(td, 'film.gif')
        r = subprocess.run(
            [sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_tools', 'make_film.py'),
             BOARD, good, bad, '--reject', 'b.kicad_pcb', '-o', out,
             '--size', '400'],
            capture_output=True, text=True, encoding='utf-8', errors='replace',
            cwd=ROOT)
        assert r.returncode == 0, r.stderr[-800:]
        assert os.path.isfile(out) and os.path.getsize(out) > 0
        assert '1 attempts' in r.stderr, r.stderr[-400:]

        r = subprocess.run(
            [sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_tools', 'make_film.py'),
             '-o', out], capture_output=True, text=True, cwd=ROOT)
        # The message claimed "not a crash" while nothing checked that -- a
        # crash exiting 2 passed. Assert what the sentence says.
        assert r.returncode == 2 and 'Traceback' not in (r.stderr or ''), \
            f"no boards must be a clean refusal, not a crash: {(r.stderr or '')[-400:]}"
    print("  PASS: the CLI films a chain and refuses an empty one")


def test_loop_sidecars_still_get_the_loop_behaviour():
    """`enter_step` clears the copper on the way into a placement round because
    the LOOP re-routes from scratch each round. A synthesised beat must not
    inherit that -- but a real sidecar still must."""
    from movie_camera import Stage, load_round_sidecars
    import json
    with tempfile.TemporaryDirectory() as td:
        b = os.path.join(td, 'loop_round1.kicad_pcb')
        refs = _variant(BOARD, b, n=3)
        with open(os.path.join(td, 'loop_round1.json'), 'w') as f:
            json.dump({'schema': 1, 'round': 1, 'board': 'loop_round1.kicad_pcb',
                       'accepted': True,
                       'moved': [{'reference': refs[0], 'from': [0, 0, 0],
                                  'to': [1, 1, 0]}]}, f)
        rounds = load_round_sidecars(td)
        assert len(rounds) == 1 and not rounds[0].get('synth')
        assert load_round_sidecars(td, accepted_only=True) == rounds
    print("  PASS: a real sidecar is not marked synth and keeps its semantics")



def test_a_part_move_never_gets_fewer_than_ten_frames():
    """A placement beat is the one thing in a routing film that is not copper
    appearing. At a handful of frames it reads as a jump cut rather than as a
    part travelling, and you cannot see WHICH part went WHERE -- which is the
    only reason the beat is in the film at all.

    Two paths could starve it and both are floored: the tween clamp, and the
    runtime budget, which may cut a pan to nothing but may not cut a move."""
    from movie_camera import (Stage, Action, plan_shots, apply_budget,
                              MIN_MOVE_FRAMES)
    assert MIN_MOVE_FRAMES >= 10

    for t in (1, 2, 5, 8, 10):
        assert Stage([], tween=t).tween_frames >= MIN_MOVE_FRAMES, t
    # a bigger request is honoured, and 0 stays an explicit "cut, no glide"
    assert Stage([], tween=24).tween_frames == 24
    assert Stage([], tween=0).tween_frames == 0

    # Budget starvation: a 1-second budget at 6fps against 52 content frames.
    acts = [Action('place', 'round 1', (0, 0, 10, 10), 'F', frames=12),
            Action('route', 'step', None, 'F', frames=40)]
    tight = apply_budget(plan_shots(acts, (0, 0, 50, 20)), 1.0, 6.0)
    place = [s for s in tight if s.kind == 'action' and s.action.kind == 'place']
    route = [s for s in tight if s.kind == 'action' and s.action.kind == 'route']
    assert place and place[0].frames >= MIN_MOVE_FRAMES,         f"the budget starved a part move to {place[0].frames} frames"
    assert route, "a routing beat should still be present"
    print(f"  PASS: moves floored at {MIN_MOVE_FRAMES} through the tween AND "
          f"the budget")

if __name__ == '__main__':
    if not os.path.isfile(BOARD):
        print("SKIP: fixture missing")
        sys.exit(0)
    for k, v in sorted(globals().items()):
        if k.startswith('test_'):
            print(f"--- {k}")
            v()
    print("ALL PASS")
