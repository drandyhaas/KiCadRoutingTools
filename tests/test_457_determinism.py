"""
Quench output must not depend on PYTHONHASHSEED (issue #457 item 1).

`QuenchState.net_refs` was a `Dict[int, Set[str]]`. Its iteration order becomes the
point order handed to `compute_mst_edges`, and Prim's tie-break there is
first-index-wins (seed node 0, `argmin`, and a strict `<` on the frontier update).
Equidistant pads are the norm on a real board — uniform-pitch GND arrays, identical
decaps on a grid, symmetric connectors — so a different order builds a different
tree of the same total length. Total length is a sum and so does not notice; the
CROSSING count does, and crossings are the second term of the objective, so
different seeds accepted different moves and produced different boards.

Measured on unmodified boards before the fix, before a single move was made:

    interf_u_unrouted   447 / 457 / 450 crossings   at seeds 0 / 1 / 12345
    glasgow_revC       1344 / 1346 / 1347

with airwire length bit-identical (4015.669376 / 3808.133871) across all three.
That asymmetry is why HPWL is worth reporting: it is order-invariant by
construction, so it tells you whether two runs differ in tie-breaks or in real
placement quality.

The subprocesses are the point — PYTHONHASHSEED is fixed at interpreter start, so
this cannot be tested in-process.
"""

import json
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_router'))  # #522
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_placer'))  # placement split
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_tools'))  # #522
from run_utils import tool_env as _tool_env  # #522 PYTHONPATH

# #522 reorg + skill merge: engine -> py_router/, placer -> py_placer/,
# board_score.py -> the placement-and-routing skill. Without these roots the
# imports below raise and the test never runs (it reports as a failure while
# asserting nothing).
_R522 = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p522 in ('py_router', 'py_placer',
              os.path.join('.claude', 'skills',
                           'plan-pcb-placement-and-routing', 'scripts')):
    _d522 = os.path.join(_R522, _p522)
    if _d522 not in sys.path:
        sys.path.insert(0, _d522)


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SEEDS = ('0', '1', '12345')

# Two boards, deliberately: interf_u is single-sided-ish and modest, glasgow is
# large and two-sided (172 front / 92 back), which exercises far more tied pads.
BOARDS = ('interf_u_unrouted.kicad_pcb', 'glasgow_revC.kicad_pcb')

_PROBE = r'''
import hashlib, json, sys
from kicad_parser import parse_kicad_pcb
from placement.quench import QuenchState

KN = dict(clearance=0.25, board_edge_clearance=0.55, crossing_penalty=10.0,
          halo_base=0.3, halo_coef=0.15, halo_weight=1.0,
          edge_halo=1.0, edge_weight=1.0, grid_step=0.05)
f = sys.argv[1]
st = QuenchState(parse_kicad_pcb(f), f, **KN)
aw = [tuple(round(v, 9) for v in a)
      for lst in st.net_airwires.values() for a in lst]
cost = st.total_cost()
sys.stdout.write("RESULT" + json.dumps({
    'airwires': hashlib.sha1(repr(aw).encode()).hexdigest(),
    'n': len(aw),
    'length': round(cost['length'], 9),
    'crossings': cost['crossings'],
    'total': round(cost['total'], 9),
    'hpwl': round(cost['hpwl'], 9),
    'net_refs': hashlib.sha1(repr(sorted(
        (k, list(v)) for k, v in st.net_refs.items())).encode()).hexdigest(),
    'net_refs_order': hashlib.sha1(repr(
        [(k, list(v)) for k, v in st.net_refs.items()]).encode()).hexdigest(),
}))
'''


def _run(board, seed):
    # PYTHONPATH must reach py_router/ py_tools/ py_placer/ (and rust_router
    # for grid_router) -- the probe imports kicad_parser, which #522 moved
    # out of the repo root. run_utils.tool_env covers them all.
    env = dict(_tool_env(), PYTHONHASHSEED=seed)
    r = subprocess.run([sys.executable, '-c', _PROBE,
                        os.path.join(ROOT, 'kicad_files', board)],
                       capture_output=True, text=True, cwd=ROOT, env=env)
    if r.returncode != 0:
        raise AssertionError(f"probe failed on {board} seed={seed}:\n"
                             f"{r.stdout[-1500:]}\n{r.stderr[-1500:]}")
    marker = r.stdout.rindex('RESULT') + len('RESULT')
    return json.loads(r.stdout[marker:])


def test_quench_state_is_identical_across_hash_seeds():
    for board in BOARDS:
        results = {s: _run(board, s) for s in SEEDS}
        base = results[SEEDS[0]]
        for s in SEEDS[1:]:
            got = results[s]
            for key in ('airwires', 'n', 'length', 'crossings', 'total',
                        'hpwl', 'net_refs_order'):
                assert got[key] == base[key], (
                    f"{board}: {key} differs between PYTHONHASHSEED="
                    f"{SEEDS[0]} and {s}: {base[key]!r} vs {got[key]!r}. "
                    "Something in quench iterates a set of strings again.")
        print(f"  PASS: {board} identical across seeds "
              f"{'/'.join(SEEDS)} (crossings={base['crossings']}, "
              f"hpwl={base['hpwl']:.1f})")


def test_hpwl_is_order_invariant_even_if_something_else_is_not():
    """HPWL reads only each net's pad-position extremes, so it cannot move with
    airwire order. It is the witness that separates 'differs in tie-breaks' from
    'differs in placement quality' — keep it that way."""
    board = BOARDS[0]
    hpwls = {s: _run(board, s)['hpwl'] for s in SEEDS}
    assert len(set(hpwls.values())) == 1, f"HPWL is not order-invariant: {hpwls}"
    print(f"  PASS: HPWL stable at {list(hpwls.values())[0]:.3f}mm")


def test_net_refs_is_not_a_set():
    """The fix is at the source: net_refs holds sorted LISTS, so a future read
    site cannot reintroduce the bug by iterating it."""
    from kicad_parser import parse_kicad_pcb
    from placement.quench import QuenchState
    f = os.path.join(ROOT, 'kicad_files', 'interf_u_unrouted.kicad_pcb')
    st = QuenchState(parse_kicad_pcb(f), f, clearance=0.25,
                     board_edge_clearance=0.55, crossing_penalty=10.0,
                     halo_base=0.3, halo_coef=0.15, halo_weight=1.0,
                     edge_halo=1.0, edge_weight=1.0, grid_step=0.05)
    for net_id, refs in st.net_refs.items():
        assert isinstance(refs, list), \
            f"net {net_id}: net_refs holds {type(refs).__name__}, not list"
        assert refs == sorted(refs), f"net {net_id}: refs not sorted: {refs}"
        assert len(refs) == len(set(refs)), f"net {net_id}: duplicate refs"


TESTS = [
    test_net_refs_is_not_a_set,
    test_hpwl_is_order_invariant_even_if_something_else_is_not,
    test_quench_state_is_identical_across_hash_seeds,
]


if __name__ == '__main__':
    for t in TESTS:
        print(f"--- {t.__name__}")
        t()
    print("ALL PASS")
