#!/usr/bin/env python3
"""#1870: the cleanup trims, and check_orphan_stubs names, the tails
check_weird and KiCad report.

`trim_dangles_past_body_anchor` left a dead-end tail of up to 3 track widths
past the anchor on its body ("not worth churn"), and check_orphan_stubs
excused such an end as a reverse T. With check_weird naming a tail from
KiCad's touch distance on, the cleanup would ship what the checker reports.
The trim now reads the same test, `connectivity.end_touches`: a tail is left
only while an anchor's copper still touches its end. check_orphan_stubs
keeps its end-cap rule alone, which is that test plus the 0.05 mm it always
credited.

Rows are the boards of test_1870_tail_past_tee.py (a track that runs `tail`
past a tee, a leaning tee or a via on its body):
  1. the cleanup cuts a named tail back to its anchor and leaves a nib, and
     the net stays connected;
  2. after it check_weird reports no dangling end on any board;
  3. check_orphan_stubs calls the named tails orphans and the nibs not (it
     credits an end cap 0.05 mm past the touch distance, its own `tol`, so a
     tail in that band is not an orphan there);
  4. kicad-cli reports no `track_dangling` on any cleaned board (exit 77
     without kicad-cli, after 1 to 3 have passed).

    python3 tests/test_1870_trim_tail_past_tee.py
"""
import contextlib
import io
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, os.path.join(REPO, 'py_tools'))
import run_utils  # noqa: E402
from test_1870_tail_past_tee import ROWS, board, weird  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402
from routing_config import GridRouteConfig  # noqa: E402
from cleanup_pipeline import run_post_route_cleanup  # noqa: E402
from check_connected import check_net_connectivity  # noqa: E402
from check_orphan_stubs import find_orphan_stubs  # noqa: E402
from kicad_writer import generate_segment_sexpr  # noqa: E402

failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def cleaned(path, w):
    """route_diff's cleanup over the board's one net -> (pcb, x of the long
    track's far end, connected?)."""
    with contextlib.redirect_stdout(io.StringIO()), \
            contextlib.redirect_stderr(io.StringIO()):
        pcb = parse_kicad_pcb(path)
        cfg = GridRouteConfig(track_width=w, clearance=0.1, layers=['F.Cu', 'B.Cu'])
        results = [{'new_segments': list(pcb.segments), 'new_vias': []}]
        run_post_route_cleanup(results, pcb, {1}, cfg, snap=False, phantom=False,
                               neck=False)
    conn = check_net_connectivity(1, list(pcb.segments), list(pcb.vias),
                                  pcb.pads_by_net[1])
    far = max(max(s.start_x, s.end_x) for s in pcb.segments
              if s.layer == 'F.Cu' and abs(s.start_y - 10) < 1e-6
              and abs(s.end_y - 10) < 1e-6)
    return pcb, far, conn.get('connected') is True


def written(pcb, src, dst):
    """`src` with its tracks replaced by `pcb`'s -> dst (for the checkers
    that read a file)."""
    with open(src, encoding='utf-8') as fh:
        text = fh.read()
    keep = [ln for ln in text.splitlines() if not ln.startswith('  (segment ')]
    assert keep[-1] == ')', keep[-1]
    segs = [generate_segment_sexpr((s.start_x, s.start_y), (s.end_x, s.end_y),
                                   s.width, s.layer, s.net_id)
            for s in pcb.segments]
    with open(dst, 'w', encoding='utf-8') as fh:
        fh.write('\n'.join(keep[:-1] + segs + [')']) + '\n')
    return run_utils.evidence(dst) or dst


def main():
    td = tempfile.mkdtemp(prefix='krt1870b_')
    skipped = False
    try:
        print('1. the cleanup cuts a named tail back to its anchor, and leaves a nib')
        after = {}
        for kind, w, ow, tail, named in ROWS:
            key = (kind, w, ow, tail)
            src = board(os.path.join(td, '%s_%s_%s_%s.kicad_pcb' % key), kind, w, ow, tail)
            pcb, far, connected = cleaned(src, w)
            want = 14.0 if named else round(14 + tail, 4)
            check(f'{kind}, track {w}, anchor {ow}, tail {tail}: '
                  f'{"cut back to x=14" if named else "left as it is"}, connected',
                  abs(far - want) < 1e-6 and connected, f'ends at x={far:.4f}, '
                  f'connected {connected}')
            after[key] = (src, written(pcb, src, src.replace('.kicad_pcb', '_clean.kicad_pcb')))

        print('2. check_weird reports no dangling end after the cleanup')
        left = {k: weird(dst)[0] for k, (_src, dst) in after.items()}
        check('0 on all %d boards' % len(left), not any(left.values()),
              str({k: n for k, n in left.items() if n}))

        print('3. check_orphan_stubs: a named tail is an orphan, a nib is not')
        for kind, w, ow, tail, named in ROWS:
            src = after[(kind, w, ow, tail)][0]
            with contextlib.redirect_stdout(io.StringIO()):
                found = sorted(pt for layers in find_orphan_stubs(src).values()
                               for pts in layers.values() for pt in pts)
            # the end's distance to the anchor's copper: along the track to a
            # tee or a via, across to a tee leaning 45 degrees toward it
            dist = tail * (math.sqrt(0.5) if kind == 'lean' else 1.0)
            orphan = dist > (w + ow) / 2 + 0.05
            want = [(round(14 + tail, 4), 10.0)] if orphan else []
            check(f'{kind}, track {w}, anchor {ow}, tail {tail}: '
                  + ('an orphan at its free end' if orphan else
                     'not an orphan' + (' (inside its 0.05 mm credit)' if named else '')),
                  found == want, str(found))

        print('4. kicad-cli reports no track_dangling after the cleanup')
        with contextlib.redirect_stdout(io.StringIO()):
            from kicad_oracle import find_kicad_cli
            cli = find_kicad_cli(warn=False)
        if not cli:
            print('  SKIPPED: no kicad-cli (set KICAD_CLI)')
            skipped = True
        else:
            bad = {}
            for key, (_src, dst) in after.items():
                rep = dst + '.drc.json'
                subprocess.run([cli, 'pcb', 'drc', '--severity-all', '--format', 'json',
                                '-o', rep, dst], capture_output=True)
                with open(run_utils.evidence(rep), encoding='utf-8') as fh:
                    n = sum(v['type'] == 'track_dangling'
                            for v in json.load(fh)['violations'])
                if n:
                    bad[key] = n
            check('0 on all %d boards' % len(after), not bad, str(bad))
    except AssertionError as exc:
        check('no tool died', False, str(exc)[:1500])
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if failures:
        print(f'FAIL: {len(failures)}: ' + '; '.join(failures))
        return 1
    if skipped:
        print('PARTS 1-3 PASS')
        print('SKIP: part 4 needs kicad-cli (none found; set KICAD_CLI) -- that '
              'KiCad reports no dangling track after the cleanup was not checked')
        return 77
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
