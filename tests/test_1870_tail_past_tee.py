#!/usr/bin/env python3
"""#1870: a track end that runs past a tee is named once KiCad would report it.

check_weird allowed a tail past a mid-body anchor of up to 3 track widths
("sub-visible nib") and said nothing. KiCad's DRC reports the end as
`track_dangling` as soon as no connected item's copper lies within half the
track's width of it, which for a tee of the same width is one track width.
So a 0.127 mm track running 0.20 to 0.38 mm past a tee was a KiCad warning
on a board check_weird called `NO WEIRD THINGS FOUND!`.

The tail is now a nib only while an anchor's copper still touches its end
(`connectivity.end_touches`), which is KiCad's test: half the track's width
plus half the anchor's (a via: its size).

Part 1 (no KiCad): the table below, through check_weird.
Part 2 (kicad-cli; exit 77 without it): KiCad's `track_dangling` count on
every board of the table is check_weird's `dangling-end` count.

    python3 tests/test_1870_tail_past_tee.py
"""
import contextlib
import io
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
import run_utils  # noqa: E402
from synth import RoutingBoard  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
failures = []

#: (anchor, track width, anchor width, tail, named?) -- the anchor is a tee
#: of that width, a tee leaning 45 degrees toward the free end, or a via of
#: that size on the body. Named = at least (track + anchor) / 2 from the
#: anchor's copper; each pair of rows stands on either side of that. The
#: comparison is strict, as KiCad's: a tail of exactly that distance is
#: reported (the 0.127 and 0.2 rows), one a tenth of a micron short is not.
ROWS = (
    ('tee', 0.127, 0.127, 0.1269, False), ('tee', 0.127, 0.127, 0.127, True),
    ('tee', 0.2, 0.2, 0.1999, False), ('tee', 0.2, 0.2, 0.2, True),
    ('tee', 0.127, 0.127, 0.10, False), ('tee', 0.127, 0.127, 0.20, True),
    ('tee', 0.127, 0.127, 0.30, True), ('tee', 0.127, 0.127, 0.38, True),
    ('tee', 0.127, 0.127, 0.50, True),
    ('tee', 0.3, 0.3, 0.20, False), ('tee', 0.3, 0.3, 0.50, True),
    ('tee', 0.3, 0.3, 0.80, True), ('tee', 0.3, 0.3, 0.95, True),
    ('tee', 0.127, 0.5, 0.30, False), ('tee', 0.127, 0.5, 0.33, True),
    ('tee', 0.5, 0.127, 0.30, False), ('tee', 0.5, 0.127, 0.33, True),
    ('via', 0.127, 0.6, 0.34, False), ('via', 0.127, 0.6, 0.38, True),
    ('lean', 0.2, 0.2, 0.26, False), ('lean', 0.2, 0.2, 0.30, True),
)


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def board(path, kind, w, ow, tail):
    """Pads A (10, 10) and C (14, 12) of one net; a track from A along y = 10
    that ends `tail` past x = 14, where the rest of the net leaves it."""
    rb = RoutingBoard(w=30, h=20)
    rb.pad('A', 10, 10, 'N')
    rb.pad('C', 14, 12, 'N')
    rb.seg(10, 10, round(14 + tail, 4), 10, 'N', w=w)
    if kind == 'tee':
        rb.seg(14, 10, 14, 12, 'N', w=ow)
    elif kind == 'lean':
        rb.seg(14, 10, 16, 12, 'N', w=ow)
        rb.seg(16, 12, 14, 12, 'N', w=ow)
    else:
        rb.via(14, 10, 'N', size=ow, drill=0.3)
        rb.seg(14, 10, 14, 12, 'N', w=w, layer='B.Cu')
        rb.via(14, 12, 'N', size=0.6, drill=0.3)
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(rb.text())
    return run_utils.evidence(path) or path


def weird(path):
    """check_weird's console report of `path` -> (dangling-end count, text)."""
    r = subprocess.run(PY + [os.path.join(REPO, 'py_router', 'check_weird.py'), path],
                       capture_output=True, text=True, encoding='utf-8', errors='replace')
    out = r.stdout + r.stderr
    assert 'Traceback' not in out, out[-800:]
    m = re.search(r'^\s*dangling-end: (\d+)', out, re.M)
    return (int(m.group(1)) if m else 0), out


def main():
    td = tempfile.mkdtemp(prefix='krt1870_')
    skipped = False
    try:
        print('1. check_weird names a tail once no anchor touches its end')
        paths, counts = {}, {}
        for kind, w, ow, tail, named in ROWS:
            key = (kind, w, ow, tail)
            paths[key] = board(os.path.join(td, '%s_%s_%s_%s.kicad_pcb' % key),
                               kind, w, ow, tail)
            n, out = weird(paths[key])
            counts[key] = n
            reach = (w + ow) / 2
            check(f'{kind}, track {w} mm, anchor {ow} mm (touch {reach:.4f} mm), '
                  f'tail {tail} mm: {"named" if named else "a nib, not named"}',
                  n == (1 if named else 0)
                  and not (named and 'NO WEIRD THINGS FOUND!' in out), out[-300:])
            if key == ('tee', 0.127, 0.127, 0.30):
                check('... as a half-segment tail, with its length and the tee',
                      'half-segment tail dangling 0.300mm past body anchor at '
                      '(14.000, 10.000)' in out, out[-400:])

        print('2. kicad-cli reports the same ends')
        with contextlib.redirect_stdout(io.StringIO()):
            from kicad_oracle import find_kicad_cli
            cli = find_kicad_cli(warn=False)
        if not cli:
            print('  SKIPPED: no kicad-cli (set KICAD_CLI)')
            skipped = True
        else:
            for key, path in paths.items():
                rep = path + '.drc.json'
                subprocess.run([cli, 'pcb', 'drc', '--severity-all', '--format', 'json',
                                '-o', rep, path], capture_output=True)
                with open(run_utils.evidence(rep), encoding='utf-8') as fh:
                    k = sum(v['type'] == 'track_dangling'
                            for v in json.load(fh)['violations'])
                check('%s, track %s, anchor %s, tail %s: KiCad track_dangling %d'
                      % (key + (k,)), k == counts[key], f'check_weird {counts[key]}')
    except AssertionError as exc:
        check('no tool died', False, str(exc)[:1500])
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if failures:
        print(f'FAIL: {len(failures)}: ' + '; '.join(failures))
        return 1
    if skipped:
        print('PART 1 PASS')
        print('SKIP: part 2 needs kicad-cli (none found; set KICAD_CLI) -- that '
              'KiCad reports the same ends was not checked')
        return 77
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
