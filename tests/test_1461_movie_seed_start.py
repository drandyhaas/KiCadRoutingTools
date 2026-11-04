#!/usr/bin/env python3
"""#1461: without --route, make_movie and place_movie took --scattered,
--clustered and --anchored and dropped them, so place_simple placed `--seed S`
from a scattered start: a kept clustered or anchored seed was filmed as
another placement, and nothing said so.

The counts (place_simple's defaults when none is given) now become the
--clustered-seeds / --anchored-seeds lists a run of them numbers its seeds by
(`place_movie.seed_starts`, over `place_simple.start_lists`, the numbering
place_simple's own default seeds and the --route flow use), a line names the
seed and its start, and a list beside a count that starts the seed otherwise
is refused (exit 2).

    python3 tests/test_1461_movie_seed_start.py
"""
import contextlib
import io
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for _d in ('', 'py_router', 'py_tools', 'py_placer', 'movie3d'):
    _p = os.path.join(ROOT, _d) if _d else ROOT
    if _p not in sys.path:
        sys.path.insert(0, _p)
sys.path.insert(0, HERE)

import place_movie                                            # noqa: E402
from place_simple import (CLUSTERED_SEEDS, SCATTERED_SEEDS,  # noqa: E402
                          start_lists)
from run_utils import check, tool                             # noqa: E402

NONE = (None, None, None)


def starts(given, *opts, out='o.kicad_pcb'):
    return place_movie.seed_starts('t', given, ['--groups', 'g.json', *opts], out)


def test_start_lists():
    assert start_lists(2, 2, 1) == ('2-3', '4-4')
    assert start_lists(6, 6, 0) == ('6-11', None)
    assert start_lists(3, 0, 2) == (None, '3-4')


def test_counts_become_lists():
    opts, line = starts((2, 2, None), '--seed', '2')
    assert opts[-2:] == ['--clustered-seeds', '2-3'], opts
    assert 'filming seed 2 from a clustered start' in line, line
    opts, line = starts((2, 2, 1), '--seed', '4')
    assert opts[-4:] == ['--clustered-seeds', '2-3', '--anchored-seeds', '4-4'], opts
    assert 'from an anchored start' in line, line
    # no count given: a default run's numbering, so a default run's clustered seed
    s = SCATTERED_SEEDS
    opts, line = starts(NONE, '--seed', str(s))
    assert opts[-2:] == ['--clustered-seeds', f'{s}-{s + CLUSTERED_SEEDS - 1}'], opts
    assert f'filming seed {s} from a clustered start, as a default run' in line, line
    _, line = starts(NONE)
    assert 'filming seed 0 from a scattered start' in line, line


def test_lists_kept_and_conflicts_refused():
    opts, line = starts(NONE, '--seed', '2', '--clustered-seeds', '2')
    assert opts.count('--clustered-seeds') == 1 and 'from a clustered start' in line, (opts, line)
    # a list agreeing with the counts is kept as given
    opts, line = starts((2, 2, None), '--seed', '2', '--clustered-seeds', '2')
    assert opts.count('--clustered-seeds') == 1, opts
    opts, why = starts((2, 2, None), '--seed', '1', '--clustered-seeds', '1')
    assert opts is None and 'give the lists or the counts' in why, why
    opts, why = starts((2, 2, None), '--seed', '9')
    assert opts is None and 'is not one of the 4 seeds' in why, why
    # several seeds are place_simple's business: the counts go on to it
    opts, line = starts((2, 2, None), out='s{seed}.kicad_pcb')
    assert opts[-4:] == ['--scattered', '2', '--clustered', '2'] and line is None, opts


def test_cli_refuses():
    check([sys.executable, '-X', 'utf8', tool('place_movie.py'), 'b.kicad_pcb', 'o.kicad_pcb',
           '--seed', '1', '--scattered', '2', '--clustered', '2', '--clustered-seeds', '1'],
          refuse='give the lists or the counts, not both', code=2)
    # make_movie refuses the same before it records (its node / ffmpeg checks stubbed)
    import make_movie
    which, isdir = make_movie.shutil.which, make_movie.os.path.isdir
    make_movie.shutil.which = lambda name: '/usr/bin/' + name
    make_movie.os.path.isdir = lambda p: True
    err = io.StringIO()
    try:
        with contextlib.redirect_stderr(err):
            rc = make_movie.main(['b.kicad_pcb', 'o.kicad_pcb', '--seed', '1', '--scattered', '2',
                                  '--clustered', '2', '--clustered-seeds', '1'])
    finally:
        make_movie.shutil.which, make_movie.os.path.isdir = which, isdir
    assert rc == 2 and 'make_movie: seed 1 starts clustered by --clustered-seeds 1' in err.getvalue(), \
        (rc, err.getvalue())


def main():
    test_start_lists()
    test_counts_become_lists()
    test_lists_kept_and_conflicts_refused()
    test_cli_refuses()
    print('test_1461_movie_seed_start: all passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
