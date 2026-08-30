"""Generated fixture boards, built on demand from TRACKED inputs.

Several tests consume boards that another test produces as a side effect —
`fanout_starting_point.kicad_pcb`, `fanout_output*.kicad_pcb`,
`interf_u_plane.kicad_pcb`. Those files are gitignored (correctly: they are
outputs), so on a fresh clone they do not exist, and `tests/run_all.py` globs
alphabetically, which puts every consumer BEFORE its producer:

    test_493_interpreter_independence.py   27  <  test_fanout_and_route.py  91
    test_506_507_plan_movie_tools.py       37  <  test_fanout_and_route.py  91
    test_dru_layer_clearance_e2e.py        83  <  test_fanout_and_route.py  91

So on a clean tree those tests could never pass, in any order. They had each
grown their own `if not os.path.exists(...): print("SKIP"); return`, which turns
a missing fixture into silent non-coverage — the fresh-clone hazard #457 item 3
names (#190/#245/#432/#433).

The fix is that every one of these boards is REPRODUCIBLE: the chains root in
boards that ARE tracked.

    haasoscope_pro_max_test.kicad_pcb   (tracked)
      -> qfn_fanned_out.kicad_pcb
        -> fanout_starting_point.kicad_pcb
          -> fanout_output1.kicad_pcb
            -> fanout_output2.kicad_pcb

    interf_u_unrouted.kicad_pcb         (tracked)
      -> interf_u_plane.kicad_pcb

`ensure(name)` returns the path, building the whole chain if needed. Each step
costs 2-5s and the result persists, so only the first consumer in a session pays,
and a run that already executed test_fanout_and_route.py pays nothing.

Deliberately NOT covered: `interf_u_connected.kicad_pcb` and friends, which need
a full board route (minutes). A test wanting those should use a tracked board
instead.

Producers keep their own explicit chains — test_fanout_and_route.py and
test_interf_u.py are testing those pipelines, not merely using them. Consumers
here use the boards purely as substrate (parse them, fan out from them, diff
them), so the two recipes drifting apart cannot invalidate an assertion.
"""

import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BOARDS = os.path.join(ROOT, 'kicad_files')

if os.path.join(ROOT, 'py_router') not in sys.path:
    sys.path.insert(0, os.path.join(ROOT, 'py_router'))
# One list, not a hand-written copy -- this file's said
# ('.kicad_pcb', '.kicad_pro', '.kicad_prl') and dropped `.kicad_dru`.
from copy_board import SIBLING_EXTS  # noqa: E402

# Mirrors tests/test_fanout_and_route.py's parameter groups.
_L4 = ['--layers', 'F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']
_L5 = ['--layers', 'F.Cu', 'In1.Cu', 'In2.Cu', 'In3.Cu', 'B.Cu']
_GEOM = ['--track-width', '0.1', '--clearance', '0.1',
         '--via-size', '0.3', '--via-drill', '0.2']

# output -> (input or None when the input is a tracked root, argv after python3)
_RECIPES = {
    'qfn_fanned_out.kicad_pcb': (
        'haasoscope_pro_max_test.kicad_pcb',
        lambda src, dst: ['py_router/qfn_fanout.py', src, '--output', dst,
                          '--component', 'U2', '--nets', 'Net-(U2*)']),
    'fanout_starting_point.kicad_pcb': (
        'qfn_fanned_out.kicad_pcb',
        lambda src, dst: ['py_router/bga_fanout.py', src, '--component', 'U3',
                          '--output', dst, '--nets', '*U2A*DATA*',
                          '--primary-escape', 'horizontal',
                          '--force-escape-direction'] + _L4 + _GEOM),
    'fanout_output1.kicad_pcb': (
        'fanout_starting_point.kicad_pcb',
        lambda src, dst: ['py_router/bga_fanout.py', src, '--component', 'U3',
                          '--output', dst, '--nets', '*U2A*',
                          '--primary-escape', 'horizontal',
                          '--check-for-previous',
                          '--force-escape-direction'] + _L4 + _GEOM),
    'fanout_output2.kicad_pcb': (
        'fanout_output1.kicad_pcb',
        lambda src, dst: ['py_router/bga_fanout.py', src, '--component', 'IC1',
                          '--output', dst, '--nets', '*lvds_rx*',
                          '--diff-pairs', '*lvds_rx*',
                          '--primary-escape', 'vertical']
                         + _L5 + ['--no-inner-top-layer'] + _GEOM),
    'interf_u_plane.kicad_pcb': (
        'interf_u_unrouted.kicad_pcb',
        lambda src, dst: ['py_router/route_planes.py', src, dst, '--nets', 'VCC', 'GND',
                          '--plane-layers', 'F.Cu', 'B.Cu']),
}


class FixtureBuildError(RuntimeError):
    """A fixture board could not be produced from its tracked root."""


# #522 moved the CLI scripts out of the repo root (bga_fanout / qfn_fanout /
# route_planes -> py_router/). The recipes above name them bare, so resolve
# here rather than in every lambda: a recipe added later gets this for free,
# and a script that moves again is one list entry instead of N.
_SCRIPT_DIRS = ('py_router', 'py_placer', 'py_tools', '')


def _script_path(name):
    """Absolute path to a fixture recipe's CLI script, by bare filename."""
    for d in _SCRIPT_DIRS:
        p = os.path.join(ROOT, d, name) if d else os.path.join(ROOT, name)
        if os.path.isfile(p):
            return p
    raise FixtureBuildError(
        f"fixture recipe wants {name}, which is in none of "
        f"{', '.join(repr(d or '<root>') for d in _SCRIPT_DIRS)}")


def _build(name, path):
    src_name, argv_for = _RECIPES[name]
    src = ensure(src_name)
    # Build under a temporary name that still ends in .kicad_pcb, so the engine's
    # own sibling handling (the .kicad_pro DRC floor, per CLAUDE.md #441) works
    # normally, then move the board AND its siblings into place together. The
    # rename is what stops an interrupted run from leaving a half-written board
    # that the next run mistakes for a finished fixture.
    stem = path[:-len('.kicad_pcb')]
    tmp_stem = f"{stem}.building{os.getpid()}"
    tmp = tmp_stem + '.kicad_pcb'
    argv = argv_for(src, tmp)
    argv = [sys.executable, _script_path(argv[0])] + list(argv[1:])
    print(f"  [fixture] building {name} from {os.path.basename(src)} ...")
    try:
        r = subprocess.run(argv, capture_output=True, text=True, cwd=ROOT)
        if r.returncode != 0 or not os.path.exists(tmp):
            raise FixtureBuildError(
                f"could not build {name}:\n  {' '.join(argv[1:])}\n"
                f"{r.stdout[-1200:]}\n{r.stderr[-800:]}")
        for ext in ('.kicad_pcb',) + SIBLING_EXTS:
            if os.path.exists(tmp_stem + ext):
                os.replace(tmp_stem + ext, stem + ext)
    finally:
        for ext in ('.kicad_pcb',) + SIBLING_EXTS:
            if os.path.exists(tmp_stem + ext):
                os.unlink(tmp_stem + ext)
    return path


def ensure(name):
    """Absolute path to fixture board `name`, building it if it is absent.

    `name` may be a tracked root, in which case its presence is simply asserted —
    a missing tracked board is a real repository problem, not something to work
    around.
    """
    path = os.path.join(BOARDS, name)
    if os.path.exists(path):
        return path
    if name not in _RECIPES:
        raise FixtureBuildError(
            f"kicad_files/{name} is missing and has no build recipe. It is "
            f"supposed to be tracked in git -- check your checkout.")
    return _build(name, path)


def ensure_many(*names):
    return [ensure(n) for n in names]


if __name__ == '__main__':
    # `python3 tests/fixture_boards.py` pre-builds everything, e.g. before a
    # parallel or offline run.
    for _n in _RECIPES:
        print(f"{_n}: {ensure(_n)}")
    print("ALL FIXTURES PRESENT")
