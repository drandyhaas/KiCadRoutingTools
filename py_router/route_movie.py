#!/usr/bin/env python3
"""Make a movie of a routing run: each step's copper, in the order it was laid
(#482).

    python3 py_router/route_movie.py RUNDIR [--final KEPT.kicad_pcb] [-o RUNDIR/routing.mp4]
    python3 py_router/route_movie.py a.kicad_pcb b.kicad_pcb ... -o out.mp4

A run dir plays the LINEAGE of its final board, in the order its
redo_commands.sh writes it (#1638): the boards a replay of the chain would
make, without the tries the run went back from or the branches nothing went
on from. The final board is `--final` when given, else the last board the
chain writes whose name says `final`, else the last board it writes -- the
manifest cannot say which board the run kept, so a run whose last try was
rejected names its kept board. Without a manifest, a run dir plays `stepN` in
the name, else the order the boards were written, a `final*` board last. Play
starts from the chain's input board -- the
first board `redo_commands.sh` reads -- when it can be found. Where a step left
a `<board>_routetrace.json` (route.py / route_diff.py run with
KICAD_ROUTE_TRACE=1), its copper plays one router event per frame: new copper
flashes white, restored copper cyan, ripped copper red and dashed before it
goes. Whatever else the step changed (a plane pour, the plane finalize, a step
with no trace) then lands net by net, until the frame is the board the step
wrote. The caption names the step, the event and the net.

Frames are drawn by `route_render`. The movie is an .mp4 through `ffmpeg` when
it is on PATH, else a GIF beside it.
"""
#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['routing'], 'kind': 'instrument'}

import argparse
import glob
import json
import math
import os
import re
import shlex
import shutil
import subprocess
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

SIZE = 1000         # longest frame side, px
FPS = 12.0          # twice the August movie's 6: the same frames, played at double speed
RIP_HOLD = 2        # frames a rip shows red before it goes
CHUNKS = 6          # frames for a step's untraced copper
END_HOLD = 0.75     # seconds on the last frame
MAX_FRAMES = 2400   # traced events past this share frames (200 s at 12 fps)
ZONE_TINT = 0.2     # a pour's share of its layer's colour (the stills' 0.45 hides a track on it)


class MovieWriter:
    """Frames in, a movie out: streamed to `ffmpeg` (H.264 .mp4) when the
    output is .mp4 and ffmpeg is on PATH, else kept and saved as a GIF
    (`<out>.gif`). Every frame is resized to the first one's size."""

    def __init__(self, out, fps=FPS):
        self.out, self.fps, self.size, self.n = out, fps, None, 0
        self.proc, self.frames, self.last = None, [], None
        if out.lower().endswith('.mp4') and shutil.which('ffmpeg'):
            self.proc = 'pending'
        else:
            self.out = os.path.splitext(out)[0] + '.gif'

    def add(self, img, times=1):
        img = img.convert('RGB')
        if self.size is None:
            self.size = img.size
            if self.proc == 'pending':
                w, h = self.size
                self.proc = subprocess.Popen(
                    ['ffmpeg', '-v', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
                     '-s', f'{w}x{h}', '-r', f'{self.fps:g}', '-i', '-',
                     '-vf', 'pad=ceil(iw/2)*2:ceil(ih/2)*2', '-c:v', 'libx264',
                     '-pix_fmt', 'yuv420p', '-crf', '23', '-movflags', '+faststart',
                     self.out], stdin=subprocess.PIPE)
        elif img.size != self.size:
            img = img.resize(self.size)
        self.last = img
        for _ in range(times):
            self.n += 1
            if self.proc:
                self.proc.stdin.write(img.tobytes())
            else:
                self.frames.append(img.quantize(colors=255))

    def close(self, hold=END_HOLD):
        """Hold the last frame `hold` seconds, finish the file and return its
        path (None when no frame came)."""
        if self.last is None:
            return None
        self.add(self.last, max(1, int(round(hold * self.fps))))
        if self.proc:
            self.proc.stdin.close()
            if self.proc.wait() != 0:
                raise RuntimeError(f'ffmpeg failed writing {self.out}')
        else:
            self.frames[0].save(self.out, save_all=True, append_images=self.frames[1:],
                                duration=int(round(1000 / self.fps)), loop=0)
        return self.out


class _S:
    __slots__ = ('start_x', 'start_y', 'end_x', 'end_y', 'width', 'layer', 'net_id')

    def __init__(self, x1, y1, x2, y2, w, layer, net=0):
        (self.start_x, self.start_y, self.end_x, self.end_y, self.width,
         self.layer, self.net_id) = x1, y1, x2, y2, w, layer, net


class _V:
    __slots__ = ('x', 'y', 'size', 'drill', 'layers', 'net_id')

    def __init__(self, x, y, size, drill, layers, net=0):
        self.x, self.y, self.size, self.drill, self.layers, self.net_id = x, y, size, drill, layers, net


def _skey(s):
    a, b = (round(s.start_x, 3), round(s.start_y, 3)), (round(s.end_x, 3), round(s.end_y, 3))
    return (min(a, b), max(a, b), round(s.width, 3), s.layer)


def _vkey(v):
    return (round(v.x, 3), round(v.y, 3), round(v.size, 3), round(v.drill, 3))


def _board_copper(pcb):
    """{key: drawable} for the board's tracks and vias (footprint and board
    graphic copper is the substrate's, not the router's)."""
    segs, vias = {}, {}
    for s in pcb.segments:
        if not getattr(s, 'graphic', False):
            d = _S(s.start_x, s.start_y, s.end_x, s.end_y, s.width, s.layer, s.net_id)
            segs[_skey(d)] = d
    for v in pcb.vias:
        d = _V(v.x, v.y, v.size, v.drill, list(v.layers or []), v.net_id)
        vias[_vkey(d)] = d
    return segs, vias


def _trace_rows(ev, key, layers):
    segs = [_S(r[0], r[1], r[2], r[3], r[4],
               layers[int(r[5])] if 0 <= int(r[5]) < len(layers) else layers[0], ev.get('net', 0))
            for r in ev.get(key + '_s', ())]
    vias = [_V(r[0], r[1], r[2], r[3], [layers[int(r[4])], layers[int(r[5])]], ev.get('net', 0))
            for r in ev.get(key + '_v', ())
            if 0 <= int(r[4]) < len(layers) and 0 <= int(r[5]) < len(layers)]
    return segs, vias


def _step_key(path):
    b = os.path.basename(path)
    m = re.search(r'step\s*(\d+)([a-z]*)', b, re.I)
    return (int(m.group(1)) if m else 9999, m.group(2) if m else '', b)


def _manifest_reader():
    """redo_stress_test's manifest reading (parse, check test, prune), the
    one every replay runs; it lives with the replay harness."""
    stress = os.path.join(os.path.dirname(_HERE), 'tests', 'stress')
    if stress not in sys.path:
        sys.path.insert(0, stress)
    import redo_stress_test
    return redo_stress_test


def chain_order(manifest, run_dir, final=None):
    """The boards of the final board's LINEAGE, in the order `manifest` (a
    redo_commands.sh) writes them, that are in `run_dir` (matched by name: a
    manifest may name the recording machine's paths). [] when it names none.

    The lineage is the one a replay runs (`redo_stress_test.
    compute_prune_keep`): a try the run went back from, and a branch nothing
    went on from, are not in it (#1638). The final board is `final` when
    given, else the last board the chain writes whose name says `final`,
    else the last board it writes -- a manifest records which boards were
    written, not which one the run KEPT, so a run whose last try was
    rejected names its kept board with `final` (`--final`, or a `final*`
    copy at the end)."""
    R = _manifest_reader()
    cmds = R.parse_manifest(manifest)
    outs = [R.board_io(argv)[1] for _cwd, argv in cmds
            if not R.is_check_cmd(argv)]
    outs = [o for o in outs if o]
    if final is None:
        final = next((o for o in reversed(outs) if 'final' in o.lower()), None)
    keep, info = R.compute_prune_keep(
        cmds, final_board=os.path.basename(final) if final else None)
    if final and info.get('final_board') != os.path.basename(final):
        raise ValueError(f"no command in {manifest} writes {os.path.basename(final)}")
    out = []
    for i in sorted(keep):
        if R.is_check_cmd(cmds[i][1]):
            continue
        board = R.board_io(cmds[i][1])[1]
        if board:
            path = os.path.join(run_dir, board)
            if os.path.isfile(path) and path not in out:
                out.append(path)
    return out


def run_boards(run_dir, manifest=None, final=None):
    """The run dir's boards in chain order: the boards of the final board's
    lineage in the order the chain's manifest (`manifest`, default the dir's
    redo_commands.sh) writes them (`chain_order`; `final` names the kept
    board) -- a name cannot say it: tigard's chain writes step2a_diffpairs
    before step2_route. Without a manifest, step-numbered boards by step,
    then `final*` boards by write time; with no step numbers, write-time
    order."""
    manifest = manifest or os.path.join(run_dir, 'redo_commands.sh')
    if os.path.isfile(manifest):
        got = chain_order(manifest, run_dir, final)
        if got:
            return got
    all_pcb = glob.glob(os.path.join(run_dir, '*.kicad_pcb'))
    stepped = [p for p in all_pcb if re.search(r'step\s*\d', os.path.basename(p), re.I)]
    by_time = lambda p: (os.path.getmtime(p), os.path.basename(p))  # noqa: E731
    if not stepped:
        return sorted(all_pcb, key=by_time)
    finals = [p for p in all_pcb if p not in stepped and 'final' in os.path.basename(p).lower()]
    return sorted(stepped, key=_step_key) + sorted(finals, key=by_time)


def chain_input(run_dir):
    """The first board `redo_commands.sh` reads that lives outside the run
    dir (the chain's unrouted input), or None."""
    path = os.path.join(run_dir, 'redo_commands.sh')
    if not os.path.isfile(path):
        return None
    here = os.path.realpath(run_dir)
    for line in open(path, encoding='utf-8', errors='replace'):
        if line.lstrip().startswith('#') or '.kicad_pcb' not in line:
            continue
        try:
            words = shlex.split(line)
        except ValueError:
            continue
        for w in words:
            if w.endswith('.kicad_pcb') and os.path.isabs(w) and os.path.isfile(w) \
                    and os.path.dirname(os.path.realpath(w)) != here:
                return w
    return None


def shoot(movie, r, live_s, live_v, label, hl_s=(), hl_v=(), role='event_new', rips=(),
          times=1):
    """One frame: `live_*` copper on `r`'s board, `hl_*` lit in the theme's
    `role` colour, `rips` red and dashed over it, `label` top left."""
    ov = None
    if rips:
        def ov(d, rr):
            rr._draw_segments(d, rips, color=rr.theme.rgb('event_ripped'), mark='dashed')
    movie.add(r.frame(segments=list(live_s.values()), vias=list(live_v.values()),
                      highlight_segments=list(hl_s), highlight_vias=list(hl_v),
                      highlight_color=r.theme.rgb(role), label=label,
                      overlays=[ov] if ov else None), times)


def play_chain(movie, boards, first=None, *, size=SIZE, theme='dark', max_frames=MAX_FRAMES,
               prefix='', first_label='input', zone_tint=ZONE_TINT):
    """Play `boards` (chain order) into `movie` from `first`, the board the
    chain started from (None: its first board's pads and no copper). Each
    caption starts with `prefix`. Returns the last step's (renderer, tracks,
    vias) so a caller can go on from the board it ends on."""
    from kicad_parser import parse_kicad_pcb
    from route_render import BoardRenderer
    traces = {b: os.path.splitext(b)[0] + '_routetrace.json' for b in boards}
    traces = {b: json.load(open(t)) for b, t in traces.items() if os.path.isfile(t)}
    n_events = sum(len(t.get('events', ())) for t in traces.values())
    per = max(1, math.ceil(n_events / max_frames)) if max_frames else 1

    def draw(*a, **kw):
        shoot(movie, *a, **kw)

    pcb0 = parse_kicad_pcb(first or boards[0])
    live_s, live_v = _board_copper(pcb0) if first else ({}, {})
    r = BoardRenderer(pcb0, size=size, supersample=1, theme=theme, show_zones=bool(first),
                      zone_tint=zone_tint)
    draw(r, live_s, live_v, prefix + first_label)
    for board in boards:
        step = prefix + os.path.splitext(os.path.basename(board))[0]
        pcb = parse_kicad_pcb(board)
        r = BoardRenderer(pcb, size=size, supersample=1, theme=theme, zone_tint=zone_tint)
        layers = list(pcb.board_info.copper_layers)
        events = traces.get(board, {}).get('events', [])
        tlayers = traces.get(board, {}).get('layers') or layers
        for i in range(0, len(events), per):
            batch = events[i:i + per]
            span = f"{i + 1}" if len(batch) == 1 else f"{i + 1}-{i + len(batch)}"
            ev = batch[-1]
            label = f"{step}: {span}/{len(events)} {ev.get('event', '')} {ev.get('net_name', '')}"
            added, ripped = {}, {}
            for e in batch:
                ds, dv = _trace_rows(e, 'del', tlayers)
                if len(batch) == 1 and (ds or dv):      # the rip shows, then goes
                    draw(r, live_s, live_v, label, rips=ds, hl_v=dv, role='event_ripped',
                         times=RIP_HOLD)
                for s in ds:
                    live_s.pop(_skey(s), None)
                    ripped[_skey(s)] = s
                for v in dv:
                    live_v.pop(_vkey(v), None)
                a_s, a_v = _trace_rows(e, 'add', tlayers)
                for s in a_s:
                    live_s[_skey(s)] = added[_skey(s)] = s
                for v in a_v:
                    live_v[_vkey(v)] = added[_vkey(v)] = v
            hl_s = [o for k, o in added.items() if isinstance(o, _S) and k in live_s]
            hl_v = [o for k, o in added.items() if isinstance(o, _V) and k in live_v]
            if hl_s or hl_v or len(batch) > 1:
                role = 'event_restored' if all(e.get('event') == 'restore' for e in batch) \
                    else 'event_new'
                rips = [s for k, s in ripped.items() if k not in live_s] if len(batch) > 1 else ()
                draw(r, live_s, live_v, label, hl_s, hl_v, role, rips=rips)
        # the rest of what the step changed, net by net, until the frame is
        # the board it wrote
        want_s, want_v = _board_copper(pcb)
        gone = [s for k, s in live_s.items() if k not in want_s]
        by_net = {}
        for k, s in want_s.items():
            if k not in live_s:
                by_net.setdefault(s.net_id, [[], []])[0].append(s)
        for k, v in want_v.items():
            if k not in live_v:
                by_net.setdefault(v.net_id, [[], []])[1].append(v)
        live_s = {k: s for k, s in live_s.items() if k in want_s}
        live_v = {k: v for k, v in live_v.items() if k in want_v}
        nets = list(by_net)
        if not nets:
            if gone or not events:
                draw(r, live_s, live_v, step, rips=gone)
            continue
        chunk = max(1, math.ceil(len(nets) / CHUNKS))
        for j in range(0, len(nets), chunk):
            part = nets[j:j + chunk]
            hs = [s for n in part for s in by_net[n][0]]
            hv = [v for n in part for v in by_net[n][1]]
            for s in hs:
                live_s[_skey(s)] = s
            for v in hv:
                live_v[_vkey(v)] = v
            names = [pcb.nets[n].name for n in part if n in pcb.nets and pcb.nets[n].name]
            label = f"{step}: {j // chunk + 1}/{math.ceil(len(nets) / chunk)} " + (
                ', '.join(names[:3]) + (f" +{len(names) - 3}" if len(names) > 3 else ''))
            draw(r, live_s, live_v, label, hs, hv, rips=gone if j == 0 else ())
    return r, live_s, live_v


def make_movie(inputs, out=None, size=SIZE, fps=FPS, theme='dark', max_frames=MAX_FRAMES,
               quiet=False, final=None):
    """Write the movie of a run dir (one entry) or of a board sequence; returns
    its path, or None when there is no board. `final` names a run dir's kept
    board (see `chain_order`)."""
    say = (lambda *a: None) if quiet else print
    if len(inputs) == 1 and os.path.isdir(inputs[0]):
        run_dir = inputs[0]
        boards, first = run_boards(run_dir, final=final), chain_input(run_dir)
        out = out or os.path.join(run_dir, 'routing.mp4')
    else:
        boards, first = [os.path.abspath(b) for b in inputs], None
        out = out or os.path.splitext(boards[-1])[0] + '_routing.mp4'
    if not boards:
        say(f"route_movie: no boards in {inputs[0]}")
        return None
    movie = MovieWriter(out, fps)
    r, live_s, live_v = play_chain(movie, boards, first, size=size, theme=theme,
                                   max_frames=max_frames)
    shoot(movie, r, live_s, live_v, os.path.splitext(os.path.basename(boards[-1]))[0])
    path = movie.close()
    say(f"route_movie: {path} ({movie.n} frames, {movie.n / fps:.0f} s)")
    return path


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('inputs', nargs='+', help='a run dir, or boards in chain order')
    ap.add_argument('-o', '--output', help='the movie (default RUNDIR/routing.mp4, or '
                                           '<last board>_routing.mp4); .gif writes a GIF')
    ap.add_argument('--size', type=int, default=SIZE, help=f'longest frame side, px (default {SIZE})')
    ap.add_argument('--fps', type=float, default=FPS, help=f'frames per second (default {FPS:g})')
    ap.add_argument('--theme', default='dark', help="render theme: dark (default) or light")
    ap.add_argument('--max-frames', type=int, default=MAX_FRAMES,
                    help=f'past this many router events, several share a frame (default '
                         f'{MAX_FRAMES}; 0 = one frame each)')
    ap.add_argument('--final', metavar='BOARD',
                    help='a run dir\'s KEPT board (a name or path): the movie plays its '
                         'lineage and ends on it. Default: the last board the chain '
                         'writes whose name says final, else the last board it writes')
    a = ap.parse_args(argv)
    if a.final and not (len(a.inputs) == 1 and os.path.isdir(a.inputs[0])):
        ap.error('--final names a board of a run dir; give the run dir')
    try:
        path = make_movie(a.inputs, out=a.output, size=a.size, fps=a.fps, theme=a.theme,
                          max_frames=a.max_frames, final=a.final)
    except ValueError as e:
        ap.error(str(e))
    return 0 if path else 1


if __name__ == '__main__':
    sys.exit(main())
