#!/usr/bin/env python3
"""Refuse an image whose placements could not read check_assembly's body gate.

    prove_models.py REPO [--min-models N]

modal_app.py runs this at image build, so an image short of KiCad's stock 3D
models fails its build. It would otherwise run and grade: the heights code
reports the gate unmeasured, not failed, so place_simple seats a part without
it and board_score lists the conjunct as unmeasured.

Proves, through the code that reads them on a run (`model_export`,
`placement.heights`):
  1. the library `${KICADn_3DMODEL_DIR}` resolves to holds at least N models;
  2. tests/fixtures/1603's stock edge-mount jack, stood wholly on the board,
     reads as a body through it, which it only does when kicad-cli drew the
     model at its own depth.

A tree from before that gate existed (`--src` of an old base) reads no gate, so
nothing is asked of it, and it says so.
"""
import argparse
import contextlib
import io
import os
import sys

#: well under what any KiCad 10 release ships (7236 in 10.0.0) and well over a
#: partial copy
MIN_MODELS = 7000
#: the fixture's jack and a pose that stands it wholly on that board (#1603)
FIXTURE = os.path.join('tests', 'fixtures', '1603', 'edge_sma.kicad_pcb')
REF, POSE = 'J3', (16.53, 10.2634, 0.0)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('repo')
    ap.add_argument('--min-models', type=int, default=MIN_MODELS)
    a = ap.parse_args(argv)
    for sub in ('py_router', 'py_placer'):
        sys.path.insert(0, os.path.join(a.repo, sub))
    try:
        from kicad_parser import parse_kicad_pcb
        from model_export import model_dirs, resolve_cli
        from placement.heights import body_in_board, for_board
    except ImportError as e:
        print(f'this tree has no body gate to read ({e}): nothing to prove')
        return 0
    cli, why = resolve_cli()
    if not cli:
        sys.exit(f'NO MODELS PROVED: {why}')
    root = model_dirs(cli, a.repo).get('KICAD10_3DMODEL_DIR')
    if not root:
        sys.exit(f'SHORT IMAGE: no 3D model library beside {cli}')
    count = sum(fn.lower().endswith(('.step', '.stp')) for _d, _s, names in os.walk(root) for fn in names)
    if count < a.min_models:
        sys.exit(f'SHORT IMAGE: {root} holds {count} models, fewer than {a.min_models}')
    print(f'{count} models in {root}')
    path = os.path.join(a.repo, FIXTURE)
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(path)
        heights = for_board(pcb, path)
    read = body_in_board(heights, pcb, {REF: POSE})
    if read['unmeasured'] or REF not in read['gating_refs']:
        sys.exit(f"SHORT IMAGE: the body gate read {read['gating_refs']} on {FIXTURE}'s {REF} at {POSE}, "
                 f"unmeasured: {read['unmeasured']}")
    row = next(r for r in read['rows'] if r['ref'] == REF)
    print(f"body gate read: {REF} {row['inside_mm']} mm inside at {POSE}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
