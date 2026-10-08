#!/usr/bin/env python3
"""#1129: place_portfolio never moves a part the intent locks.

`portfolio.free_refs` decides which parts a strategy may perturb. It read the
board's `(locked yes)`, `--lock` globs and #829's outline owners, but not the
intent gate's `lock_refs` (`must_lock` and the edge claims). The quench does
read them -- and freezes the part WHERE IT STANDS, so a candidate shipped a
`must_lock` part already moved by its strategy:

    lock.json: {"schema": 1, "kind": "floorplan-intent", "units": "mm",
                "must_lock": ["U1"]}
    place_portfolio.py splitflap_driver.kicad_pcb --out-dir out
        --intent lock.json --only 2 --no-render
    U1: input (181.61, 36.83, 270) -> cand_02.seed and cand_02 at rotation 90

`free_refs` now drops `intent_locks` and names each in `refused`. What each
case pins:

* the issue's command keeps U1 at its input pose in the seed AND the
  candidate, and prints why U1 was not perturbed;
* control: the same command without the intent turns U1 (the defect is the
  intent's, not the board's);
* `free_refs` drops an edge-claimed ref the gate locks, records the reason,
  and with no intent locks returns exactly the list it always did.

    python3 tests/test_1129_portfolio_intent_locks.py [name-substring ...]
"""
import json
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import run_utils  # noqa: E402

ROOT = run_utils.ROOT_DIR
for _sub in ('py_router', 'py_tools', 'py_placer'):
    _p = os.path.join(ROOT, _sub)
    if _p not in sys.path:
        sys.path.insert(0, _p)

RUN_ALL_TIMEOUT = 600

BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')
PORTFOLIO = os.path.join(ROOT, 'py_placer', 'place_portfolio.py')
INPUT_U1 = (181.61, 36.83, 270.0)


def _pose(path, ref):
    from kicad_parser import parse_kicad_pcb
    fp = parse_kicad_pcb(path).footprints[ref]
    return (round(fp.x, 3), round(fp.y, 3), round((fp.rotation or 0) % 360, 3))


def _portfolio(td, intent_doc):
    out = os.path.join(td, 'out')
    argv = [sys.executable, '-X', 'utf8', PORTFOLIO, BOARD, '--out-dir', out,
            '--only', '2', '--no-render']
    if intent_doc is not None:
        ipath = os.path.join(td, 'lock.json')
        with open(ipath, 'w', encoding='utf-8') as fh:
            json.dump(intent_doc, fh)
        argv += ['--intent', ipath]
    r = run_utils.check(argv, accept=True, timeout=1200)
    seed = run_utils.evidence(os.path.join(out, 'cand_02.seed.kicad_pcb'))
    cand = run_utils.evidence(os.path.join(out, 'cand_02.kicad_pcb'))
    return r.stdout + r.stderr, _pose(seed, 'U1'), _pose(cand, 'U1')


def test_a_must_lock_part_keeps_its_input_pose():
    assert _pose(BOARD, 'U1') == INPUT_U1, _pose(BOARD, 'U1')
    with tempfile.TemporaryDirectory() as td:
        out, seed, cand = _portfolio(td, {'schema': 1,
                                          'kind': 'floorplan-intent',
                                          'units': 'mm', 'must_lock': ['U1']})
    assert seed == INPUT_U1 and cand == INPUT_U1, (seed, cand)
    assert 'U1 not perturbed: locked by the intent' in out, out[-1500:]
    print(f"  PASS: must_lock U1 stays at {cand} in cand_02.seed and cand_02, "
          f"and the run says why")


def test_without_the_intent_the_strategy_turns_it():
    """Control: the `poses` strategy of candidate 2 does turn U1 when nothing
    locks it -- so the case above measures the lock, not a strategy that
    leaves U1 alone anyway."""
    with tempfile.TemporaryDirectory() as td:
        _out, seed, _cand = _portfolio(td, None)
    assert seed != INPUT_U1, seed
    print(f"  PASS: without the intent cand_02.seed has U1 at {seed}")


def test_free_refs_drops_the_gate_locks_and_says_why():
    from kicad_parser import parse_kicad_pcb
    from placement import floorplan
    from placement.portfolio import free_refs
    pcb = parse_kicad_pcb(BOARD)
    base = free_refs(pcb, BOARD)
    assert free_refs(pcb, BOARD, intent_locks=None) == base
    assert free_refs(pcb, BOARD, intent_locks=()) == base
    edge = sorted(r for r in base if r.startswith('J'))[0]
    doc = {'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm',
           'must_lock': ['U1'],
           'edge_connectors': [{'ref': edge, 'class': 'edge_receptacle',
                                'edge': 'north'}]}
    with tempfile.TemporaryDirectory() as td:
        ipath = os.path.join(td, 'i.json')
        with open(ipath, 'w', encoding='utf-8') as fh:
            json.dump(doc, fh)
        gate, _probs = floorplan.resolve_intent_gate(
            floorplan.load_intent(ipath), pcb, ('kicad', 'sheet'))
    locks = gate['lock_refs']
    assert {'U1', edge} <= set(locks), locks
    refused = {}
    got = free_refs(pcb, BOARD, refused=refused, intent_locks=locks)
    assert got == [r for r in base if r not in set(locks)], (got, locks)
    for r in ('U1', edge):
        assert '(#1129)' in refused.get(r, ''), refused
    print(f"  PASS: free_refs drops {sorted(set(locks) & set(base))} and names "
          f"each; with no intent locks the list is unchanged ({len(base)})")


TESTS = [
    test_a_must_lock_part_keeps_its_input_pose,
    test_without_the_intent_the_strategy_turns_it,
    test_free_refs_drops_the_gate_locks_and_says_why,
]


if __name__ == '__main__':
    want = sys.argv[1:]
    for t in TESTS:
        if want and not any(w in t.__name__ for w in want):
            continue
        print(f"--- {t.__name__}")
        t()
    print("ALL PASS")
