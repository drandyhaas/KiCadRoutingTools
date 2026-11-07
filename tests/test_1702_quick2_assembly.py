#!/usr/bin/env python3
"""#1702: `route_placed.py --quick 2`, the seed judge's level, grades the assembly
as its other levels do, so a score can see a body through the board.

Its score came from `quick_score.py`, which grades no assembly without
`--baseline`, and route_placed passed none: a seed with a body through the laminate
scored `blocking 0`, on an image with every model.

Replays a one-step recorded chain, in a throwaway corpus, over the #1603 fixture:

  1. the designer's pose: the assembly is graded, with `blocking` still the
     connectivity plus that component, and nothing else in the document changes;
  2. the same part stood wholly on the board: `body_in_board_count` is measured at
     1 and `blocking` carries it. Needs kicad-cli to draw the model; without it
     this part exits 77 with a `SKIP:` line.

    python3 tests/test_1702_quick2_assembly.py
"""
import json
import os
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))

import run_utils  # noqa: E402

RUN_ALL_TIMEOUT = 600
SKIP_EXIT = 77
FIXTURE = os.path.join(TESTS, 'fixtures', '1603', 'edge_sma.kicad_pcb')
PY = [sys.executable, '-X', 'utf8', '-B']
ROUTE_PLACED = os.path.join(TESTS, 'stress', 'placement_ab', 'route_placed.py')


def corpus(root):
    """A corpus holding one recorded chain, as route_placed reads one."""
    run = os.path.join(root, 'corpus', 'runs_set900', 'edge')
    os.makedirs(run)
    with open(os.path.join(run, 'redo_commands.sh'), 'w', encoding='utf-8') as f:
        f.write(f"# cwd={run}\n"
                f"python3 -X utf8 {REPO}/py_router/route.py "
                f"{root}/corpus/boards_unrouted_set900/edge.kicad_pcb step1.kicad_pcb --nets '*'\n")
    return os.path.join(root, 'corpus')


def score(root, name, board_text, level):
    """route_placed at `level` over `board_text` -> its score.json as a dict."""
    placed = os.path.join(root, f'{name}.kicad_pcb')
    with open(placed, 'w', encoding='utf-8') as f:
        f.write(board_text)
    out = os.path.join(root, name)
    subprocess.run(PY + [ROUTE_PLACED, placed, 'edge', '900', out, '--quick', str(level),
                         '--corpus', os.path.join(root, 'corpus'), '--repo', REPO],
                   cwd=REPO, capture_output=True, text=True, timeout=300, check=True)
    with open(run_utils.evidence(os.path.join(out, 'score.json'), f'{name} score'),
              encoding='utf-8') as f:
        return json.load(f)


def main():
    run_utils.evidence(FIXTURE, 'the #1603 fixture')
    with open(FIXTURE, encoding='utf-8') as f:
        text = f.read()
    stood = text.replace('(at 28.46 15)', '(at 16.53 10.2634)', 1)
    assert stood != text, "the fixture's jack pose moved"
    with tempfile.TemporaryDirectory(prefix='t1702q_') as root:
        corpus(root)
        doc = score(root, 'designer', text, 2)
        assert 'assembly' not in doc['ungraded'], doc['ungraded']
        asm = doc['blocking_by'].get('assembly')
        assert isinstance(asm, int), doc['blocking_by']
        assert doc['blocking'] == doc['blocking_by']['unrouted'] + doc['blocking_by']['broken'] + asm, doc
        assert 'quality' in doc and doc['components']['assembly']['ran'], doc
        print(f"  ok: --quick 2 grades the assembly on the designer's pose "
              f"(assembly {asm}, blocking {doc['blocking']})")

        from model_export import resolve_cli
        cli, why = resolve_cli()
        if not cli:
            print(f'SKIP: no kicad-cli ({why}): the body gate is not read here')
            return SKIP_EXIT
        doc = score(root, 'stood', stood, 2)
        conj = doc['components']['assembly'].get('conjuncts') or {}
        if 'body_in_board_count' not in conj:
            print(f"SKIP: kicad-cli resolved no model of the jack "
                  f"({doc['components']['assembly'].get('conjuncts_unmeasured')})")
            return SKIP_EXIT
        assert conj['body_in_board_count'] == 1, conj
        assert doc['blocking_by']['assembly'] >= 1, doc['blocking_by']
        assert doc['blocking'] >= doc['blocking_by']['assembly'], doc
        print(f"  ok: a body through the board counts: body_in_board_count "
              f"{conj['body_in_board_count']}, assembly {doc['blocking_by']['assembly']}, "
              f"blocking {doc['blocking']}")
    print('test_1702_quick2_assembly: all passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
