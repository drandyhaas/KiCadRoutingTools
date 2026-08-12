#!/usr/bin/env python3
"""make_film.py -- one film of the WHOLE search, not just the part that worked.

`make_movie.py` animates a chain of boards, and the convergence movie is fed
the ACCEPTED boards deliberately: a reverted iteration spliced into that
sequence animates a change that was undone, which reads as the router thrashing
when it was doing the opposite. That rule is right, and this does not change it.

But it leaves three things off the film, and they are most of what a run
actually consisted of:

  * **the first placements** -- the parts arriving on the board. A placement
    step changes no copper, and the routing movie animates copper deltas, so
    the step that decides everything downstream rendered as a single frame.
  * **the attempts that were not kept** -- on one run the accepted spine was 10
    boards out of 45 on disk. The 35 that were tried and dropped are where the
    search actually happened, and nothing could see them.
  * **the diagnostics** -- the delta renders, focus panels and score plots that
    were the INPUT to each decision. They sat beside the film as loose PNGs, so
    reading "why did it do that" meant opening files in another window.

This composes all three into one artifact. An attempt is shown and then
explicitly undone, badged and captioned as such, so it reads as a search that
considered something and moved on -- which is what happened -- rather than as
copper thrashing. Diagnostics are spliced in as cards where they were produced.

Everything is rendered in ONE pass so a single scale holds across the film;
per-segment renders restart from an empty board and re-reveal the copper, which
looks like the board redrawing itself between beats.

    # the whole search from a place_route_loop work dir, attempts included
    python3 make_film.py --from-loop-dir wk/ -o film.gif

    # a hand-driven chain, marking which boards were dead ends
    python3 make_film.py seed.kicad_pcb placed.kicad_pcb 'try_*.kicad_pcb' \\
        final.kicad_pcb --reject 'try_*' --cards-from renders/ -o film.gif

    # explicit, with captions
    python3 make_film.py seed.kicad_pcb \\
        'delta.png=what moved, and by how much' placed.kicad_pcb -o film.gif
"""
import _path  # noqa: F401  (py_tools -> py_router/py_placer on sys.path)
import argparse
import fnmatch
import glob
import json
import os
import sys

CARD_EXT = ('.png', '.jpg', '.jpeg', '.gif', '.bmp', '.webp')
BOARD_EXT = ('.kicad_pcb',)

DEFAULT_SIZE = 900
DEFAULT_FPS = 6.0
DEFAULT_HOLD = 1.6          # seconds a diagnostic card stays up


# ---------------------------------------------------------------------------
# Shots
# ---------------------------------------------------------------------------
def board_shot(path, label=None, accepted=True, revert_to=None):
    return {'kind': 'board', 'path': os.path.abspath(path),
            'label': label or os.path.splitext(os.path.basename(path))[0],
            'accepted': bool(accepted), 'revert_to': revert_to}


def card_shot(path, caption=None, hold=DEFAULT_HOLD):
    return {'kind': 'card', 'path': os.path.abspath(path),
            'caption': caption or os.path.splitext(os.path.basename(path))[0]
            .replace('_', ' '), 'hold': hold}


def shots_from_loop_dir(work_dir, include_rejected=True, cards=True):
    """Every round a `place_route_loop` run wrote -- kept AND dropped.

    `make_movie.placement_chain` reads the same sidecars and keeps only the
    accepted ones, because it is building the convergence movie. This is the
    other half of that file: the rounds that were tried and reverted, which are
    on disk and were, until now, invisible.
    """
    from movie_camera import load_round_sidecars
    rounds = load_round_sidecars(work_dir, accepted_only=False)
    shots, last_accepted = [], None
    for rd in rounds:
        b = rd.get('board') and os.path.join(work_dir, rd['board'])
        if not b or not os.path.exists(b):
            continue
        n = rd.get('round')
        acc = bool(rd.get('accepted'))
        if not acc and not include_rejected:
            continue
        moved = len(rd.get('moved') or [])
        what = f"{moved} part{'s' if moved != 1 else ''} moved" if moved else 'no move'
        if acc:
            shots.append(board_shot(b, f"round {n}  ({what})", True))
            r = rd.get('routed') and os.path.join(work_dir, rd['routed'])
            if r and os.path.exists(r):
                shots.append(board_shot(r, f"round {n} routed", True))
            last_accepted = r if (r and os.path.exists(r)) else b
        else:
            why = 'screened' if rd.get('screened') else 'rejected'
            shots.append(board_shot(b, f"round {n}  ({what}) -- {why}", False,
                                    revert_to=last_accepted))
    if cards:
        shots = _interleave_cards(shots, _cards_in(work_dir))
    return shots


def shots_from_ledger(ledger_path, store_root=None, work=None,
                      include_rejected=True):
    """The film of a `converge.py` run, straight off its ledger.

    The ledger already records what this needs -- parent_sha, result_sha,
    accepted, kind, lever_argv -- so the film is the ledger read out loud, and
    each beat's caption is the actual lever that produced it rather than a
    description of it.
    """
    from board_store import BoardStore, Ledger
    entries = Ledger(ledger_path).entries()
    # `boards`, not `store`: `converge.py record` is the only writer of a ledger
    # and it puts the content-addressed boards in `<ledger dir>/boards`
    # (converge.py:218). Defaulting to a different name here made
    # `--from-ledger` return ZERO shots against a real ledger, silently -- the
    # per-sha `store.has()` check just skips every entry.
    root = store_root or os.path.join(os.path.dirname(
        os.path.abspath(ledger_path)), 'boards')
    if not os.path.isdir(root):                 # tolerate the older name
        alt = os.path.join(os.path.dirname(os.path.abspath(ledger_path)), 'store')
        if os.path.isdir(alt):
            root = alt
    store = BoardStore(root)
    work = work or os.path.join(os.path.dirname(os.path.abspath(ledger_path)),
                                '_film')
    os.makedirs(work, exist_ok=True)
    shots, last_accepted = [], None
    for e in entries:
        sha = e.get('result_sha')
        if not sha or not store.has(sha):
            continue
        acc = bool(e.get('accepted'))
        if not acc and not include_rejected:
            continue
        dest = os.path.join(work, f"{e.get('iteration', len(shots))}_{sha[:8]}.kicad_pcb")
        try:
            store.get(sha, dest)
        except Exception:
            continue
        argv = e.get('lever_argv') or []
        lever = ' '.join(str(a) for a in argv[:6]) if argv else (e.get('lever') or '')
        head = f"i{e.get('iteration', '?')} {e.get('kind', 'completion')}"
        if acc:
            shots.append(board_shot(dest, f"{head}  {lever}".strip(), True))
            last_accepted = dest
        else:
            shots.append(board_shot(dest, f"{head}  {lever}  -- not kept".strip(),
                                    False, revert_to=last_accepted))
    return shots


def _cards_in(d):
    out = []
    for p in sorted(glob.glob(os.path.join(d, '*'))):
        # isfile: a DIRECTORY named like an image (render_placement's old
        # --per-side `-o x.png` layout) passed the extension filter and died
        # at Image.open with PermissionError (run-3, the film build).
        if os.path.isfile(p) and os.path.splitext(p)[1].lower() in CARD_EXT:
            out.append(p)
    return out


def _interleave_cards(shots, card_paths):
    """Put each diagnostic next to the beat it describes.

    A render is named after the board it was made from far more often than not
    (`round3_delta.png`, `r3_focus1.png`), so matching on the board stem puts it
    where it belongs. Anything that matches nothing goes up front, which is
    where an overview or a spec sheet wants to be anyway.
    """
    stems = {}
    for i, s in enumerate(shots):
        if s['kind'] == 'board':
            stems.setdefault(os.path.splitext(os.path.basename(s['path']))[0], i)
    placed, out = {}, []
    for c in card_paths:
        cstem = os.path.splitext(os.path.basename(c))[0].lower()
        hit = None
        for stem, i in stems.items():
            if stem.lower() in cstem:
                if hit is None or len(stem) > hit[1]:
                    hit = (i, len(stem))
        placed.setdefault(hit[0] if hit else -1, []).append(c)
    for c in placed.get(-1, []):
        out.append(card_shot(c))
    for i, s in enumerate(shots):
        out.append(s)
        for c in placed.get(i, []):
            out.append(card_shot(c))
    return out


def parse_positional(items, reject_globs):
    """`path`, `path=caption`, or a glob of either. Extension picks the kind."""
    shots, last_accepted = [], None
    for raw in items:
        spec, _, caption = raw.partition('=')
        matches = sorted(glob.glob(spec)) or [spec]
        for p in matches:
            ext = os.path.splitext(p)[1].lower()
            if ext in CARD_EXT and os.path.isdir(p):
                continue  # a directory named like an image is not a card
            if ext in CARD_EXT:
                shots.append(card_shot(p, caption or None))
            elif ext in BOARD_EXT:
                base = os.path.basename(p)
                rejected = any(fnmatch.fnmatch(base, g) or fnmatch.fnmatch(p, g)
                               for g in reject_globs)
                s = board_shot(p, caption or None, not rejected,
                               revert_to=None if not rejected else last_accepted)
                if not rejected:
                    last_accepted = s['path']
                shots.append(s)
            else:
                raise SystemExit(f"make_film: don't know what {p} is "
                                 f"(expected a board or an image)")
    return shots


# ---------------------------------------------------------------------------
# Rendering
# ---------------------------------------------------------------------------
def _card_frame(size_wh, image_path, caption):
    """A diagnostic held at frame size: letterboxed, captioned, not cropped.

    Cropping to fill would cut the legend off a delta render, and the legend is
    the half that carries the numbers.
    """
    from PIL import Image, ImageDraw
    from route_render import load_font
    W, H = size_wh
    canvas = Image.new('RGB', (W, H), (14, 14, 18))
    strip = max(22, H // 9)
    if image_path and os.path.exists(image_path):
        try:
            im = Image.open(image_path).convert('RGB')
            aw, ah = max(1, W - 16), max(1, H - strip - 16)
            sc = min(aw / im.width, ah / im.height)
            im = im.resize((max(1, int(im.width * sc)), max(1, int(im.height * sc))),
                           Image.LANCZOS)
            canvas.paste(im, ((W - im.width) // 2, (H - strip - im.height) // 2))
        except Exception as exc:
            print(f"make_film: could not read {image_path} ({exc})", file=sys.stderr)
    d = ImageDraw.Draw(canvas)
    d.rectangle([0, H - strip, W, H], fill=(28, 28, 34))
    font = load_font(max(11, strip // 2))
    txt = caption or ''
    try:
        tw = d.textlength(txt, font=font)
    except Exception:
        tw = len(txt) * strip // 4
    d.text((max(6, (W - tw) // 2), H - strip + strip // 5), txt,
           fill=(228, 228, 236), font=font)
    return canvas


def _badge(frame, text, rgb=(200, 60, 60)):
    """Mark a frame as an attempt: a border and a tag, drawn in place.

    Without it a rejected beat is indistinguishable from a kept one, and a film
    that shows an undone change without saying so is worse than one that omits
    it -- which is exactly why the convergence movie omits it.
    """
    from PIL import ImageDraw
    from route_render import load_font
    W, H = frame.size
    d = ImageDraw.Draw(frame)
    w = max(2, H // 160)
    for i in range(w):
        d.rectangle([i, i, W - 1 - i, H - 1 - i], outline=rgb)
    font = load_font(max(11, H // 44))
    try:
        tw = d.textlength(text, font=font)
    except Exception:
        tw = len(text) * H // 80
    pad = max(4, H // 100)
    x0, y0 = W - tw - 3 * pad, pad + w
    d.rectangle([x0, y0, W - pad - w, y0 + font.size + 2 * pad // 2], fill=rgb)
    d.text((x0 + pad, y0 + pad // 2), text, fill=(255, 255, 255), font=font)
    return frame


def build_film(shots, size=DEFAULT_SIZE, fps=DEFAULT_FPS, supersample=1,
               layer_alpha=150, rip_hold=2, chunks=6, camera='auto',
               camera_budget=0.0, tween=10, quiet=False):
    """Frames for the whole shot list. One render pass, one scale."""
    import animate_route as a
    boards = [s for s in shots if s['kind'] == 'board']
    if not boards:
        return []

    # A rejected attempt is followed by an explicit undo back to the last
    # accepted board, so the NEXT beat's delta is measured against what was
    # actually kept -- not against a board that was thrown away.
    steps, owner = [], []
    for s in shots:
        if s['kind'] != 'board':
            continue
        steps.append((s['label'], s['path'], None))
        owner.append(s)
        if not s['accepted'] and s.get('revert_to'):
            steps.append(('back to the kept board', s['revert_to'], None, 'revert'))
            owner.append({'kind': 'board', 'accepted': True, 'revert': True,
                          'label': 'reverted'})
    final = steps[-1][1]

    stage = None
    if str(camera).lower() not in ('off', '', 'none', '0'):
        try:
            from movie_camera import Stage, synth_rounds
            rounds = synth_rounds([st[1] for st in steps])
            if any(rd['moved'] for rd in rounds):
                stage = Stage(rounds, '', fps=fps, budget=camera_budget,
                              tween=tween, quiet=quiet)
        except Exception as exc:
            if not quiet:
                print(f"make_film: no camera ({exc})", file=sys.stderr)

    marks = []
    frames = a.build_boards(steps, final, size, supersample, layer_alpha,
                            rip_hold, chunks, stage=stage, marks=marks)
    if not frames:
        return []

    # Badge every frame that belongs to an attempt.
    by_step = {}
    for i, (label, board, first, last) in enumerate(marks):
        if i < len(owner):
            by_step[i] = (owner[i], first, last)
    n_att = 0
    for i, (s, first, last) in by_step.items():
        if s.get('accepted'):
            continue
        n_att += 1
        for f in frames[first:last]:
            _badge(f, 'TRIED')

    # Splice the cards in where they sit in the shot order. Walk the shot list
    # and the marks together: the Nth board shot is the Nth mark, and a card
    # goes in front of whatever board shot follows it.
    size_wh = frames[0].size
    hold_frames = lambda h: max(1, int(round(h * fps)))
    inserts = {}
    mark_i, pending = 0, []
    for s in shots:
        if s['kind'] == 'card':
            pending.append(s)
            continue
        # A card in front of the FIRST beat opens the film, so it goes before
        # frame 0 -- build_boards' own "input" snapshot, which is otherwise the
        # first thing on screen and explains nothing.
        at = 0 if mark_i == 0 else (marks[mark_i][2] if mark_i < len(marks)
                                    else len(frames))
        for c in pending:
            inserts.setdefault(at, []).extend(
                [_card_frame(size_wh, c['path'], c['caption'])] * hold_frames(c['hold']))
        pending = []
        # skip the auto-inserted revert step so the pairing stays aligned
        mark_i += 1
        if not s['accepted'] and s.get('revert_to'):
            mark_i += 1
    for c in pending:
        inserts.setdefault(len(frames), []).extend(
            [_card_frame(size_wh, c['path'], c['caption'])] * hold_frames(c['hold']))

    out = []
    for i, f in enumerate(frames):
        out.extend(inserts.get(i, []))
        out.append(f)
    out.extend(inserts.get(len(frames), []))

    if not quiet:
        n_cards = sum(1 for s in shots if s['kind'] == 'card')
        # "assembled", not "frames": this is the list handed to the writer, and
        # the GIF encoder collapses runs of identical frames (card holds, the
        # end hold), so the file that lands holds FEWER. Printing both as
        # "frames" invited exactly one reconciliation failure -- 377 here, 348
        # in the delivered GIF, and nothing said which was the film. The
        # `animate_route: wrote ...` line below counts the artifact itself; that
        # is the number to quote.
        print(f"make_film: {len(boards)} beats ({n_att} attempts), "
              f"{n_cards} cards, {len(out)} frames assembled "
              f"(the writer reports what the file actually holds)",
              file=sys.stderr)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__.split('\n')[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="A shot is a board, an image, or either with '=caption'. "
               "Globs are expanded in place.")
    ap.add_argument('shots', nargs='*',
                    help="boards and diagnostic images, in film order")
    ap.add_argument('--from-loop-dir', metavar='DIR',
                    help="expand every round a place_route_loop run wrote, "
                         "kept AND dropped, from its loop_round*.json sidecars")
    ap.add_argument('--from-ledger', metavar='FILE',
                    help="expand a converge.py ledger; boards come out of the "
                         "content-addressed store")
    ap.add_argument('--store', help="board store root (default: <ledger dir>/store)")
    ap.add_argument('--reject', action='append', default=[], metavar='GLOB',
                    help="positional boards matching this were dead ends "
                         "(repeatable)")
    ap.add_argument('--accepted-only', action='store_true',
                    help="drop the attempts -- the convergence movie's cut")
    ap.add_argument('--cards-from', metavar='DIR', action='append', default=[],
                    help="splice every image in DIR next to the beat its name "
                         "matches (repeatable)")
    ap.add_argument('--no-cards', action='store_true',
                    help="boards only, even if a source dir has images in it")
    ap.add_argument('--hold', type=float, default=DEFAULT_HOLD, metavar='SEC',
                    help=f"how long a card stays up (default {DEFAULT_HOLD})")
    ap.add_argument('-o', '--out', default='film.gif')
    ap.add_argument('--size', type=int, default=DEFAULT_SIZE)
    ap.add_argument('--fps', type=float, default=DEFAULT_FPS)
    ap.add_argument('--supersample', type=int, default=1)
    ap.add_argument('--layer-alpha', type=int, default=150)
    ap.add_argument('--rip-hold', type=int, default=2)
    ap.add_argument('--chunks', type=int, default=6)
    ap.add_argument('--end-hold', type=float, default=1.5)
    ap.add_argument('--camera', default='auto', choices=['off', 'auto'],
                    help="animate footprint motion (default: on -- the parts "
                         "arriving is most of what a film is for)")
    ap.add_argument('--camera-budget', type=float, default=0.0,
                    help="cap the camera runtime in seconds (0 = unlimited)")
    ap.add_argument('--tween', type=int, default=10,
                    help="frames per part move (0 snaps)")
    ap.add_argument('--png-dir', help="also dump every frame as a PNG")
    ap.add_argument('--shots-json', help="write the resolved shot list here")
    ap.add_argument('--quiet', action='store_true')
    a = ap.parse_args(argv)

    shots = []
    if a.from_loop_dir:
        shots += shots_from_loop_dir(a.from_loop_dir,
                                     include_rejected=not a.accepted_only,
                                     cards=not a.no_cards)
    if a.from_ledger:
        shots += shots_from_ledger(a.from_ledger, a.store,
                                   include_rejected=not a.accepted_only)
    if a.shots:
        shots += parse_positional(a.shots, a.reject)
    for d in a.cards_from:
        shots = _interleave_cards(shots, _cards_in(d))
    if a.no_cards:
        shots = [s for s in shots if s['kind'] != 'card']
    for s in shots:
        if s['kind'] == 'card':
            s['hold'] = a.hold

    if not any(s['kind'] == 'board' for s in shots):
        print("make_film: no boards to film. Give board paths, --from-loop-dir "
              "or --from-ledger.", file=sys.stderr)
        return 2
    if a.shots_json:
        with open(a.shots_json, 'w', encoding='utf-8') as f:
            json.dump(shots, f, indent=2)

    frames = build_film(shots, size=a.size, fps=a.fps, supersample=a.supersample,
                        layer_alpha=a.layer_alpha, rip_hold=a.rip_hold,
                        chunks=a.chunks, camera=a.camera,
                        camera_budget=a.camera_budget, tween=a.tween,
                        quiet=a.quiet)
    if not frames:
        print("make_film: nothing to animate", file=sys.stderr)
        return 1
    import animate_route as ar
    ar.save_movie(frames, a.out, a.fps, a.end_hold, png_dir=a.png_dir)
    return 0


if __name__ == '__main__':
    sys.exit(main())
