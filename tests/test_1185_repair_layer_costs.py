#!/usr/bin/env python3
"""#1185: the plane repair's region joins get the chain's layer costs for
EVERY layer, a forbidden -1 included.

repair_planes built its engine config without `layers=`, so `config.layers`
was the 2-layer default and get_layer_costs() -- which iterates
config.layers -- turned a 4-layer chain's `--layer-costs 1.0 3.0 -1 1.0`
into `[1000, 3000]`. The join router indexed that against the 4-layer
layer_map, and Rust folds a missing entry to 1.0x, so One-Air-Max's finalize
laid 215 +3V3 straps on the GND plane layer the run had forbidden.

Checks:
  1. GridRouteConfig.layer_costs_for aligns costs by layer NAME: a reordered
     or partial list gets each layer's own cost, -1 stays -1, an unlisted
     layer is 1.0x.
  2. The repair engine's config carries the routing layers, so its costs
     come out one per layer with In2's -1 intact: auto-detected (all copper
     layers) and an explicit subset.

    python3 tests/test_1185_repair_layer_costs.py
"""
import contextlib
import io
import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

import kicad_dru                                                # noqa: E402
import repair_planes                                            # noqa: E402
from routing_config import GridRouteConfig, FORBIDDEN_LAYER_COST  # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'watchy.kicad_pcb')   # 4 copper layers
FOUR = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


class _Stop(Exception):
    pass


def engine_config(routing_layers):
    """The config repair_planes builds, captured at its first use."""
    seen = {}
    real = kicad_dru.install_layer_clearances

    def spy(config, *a, **k):
        seen['config'] = config
        raise _Stop()
    kicad_dru.install_layer_clearances = spy
    try:
        with contextlib.redirect_stdout(io.StringIO()), \
                contextlib.redirect_stderr(io.StringIO()):
            repair_planes.repair_planes(
                BOARD, '', ['GND'], ['In2.Cu'], dry_run=True,
                routing_layers=routing_layers,
                layer_costs=[1.0, 3.0, -1, 1.0][:len(routing_layers or FOUR)])
    except _Stop:
        pass
    finally:
        kicad_dru.install_layer_clearances = real
    return seen.get('config')


def main():
    # 1. By-name alignment.
    c = GridRouteConfig(layers=list(FOUR), layer_costs=[1.0, 1.5, -1, 1.0])
    F = FORBIDDEN_LAYER_COST
    check('aligned list: each layer its own cost',
          c.layer_costs_for(FOUR) == [1000, 1500, F, 1000],
          str(c.layer_costs_for(FOUR)))
    check('reordered list: costs follow the names',
          c.layer_costs_for(['B.Cu', 'In2.Cu', 'F.Cu']) == [1000, F, 1000])
    check('a layer the config does not list is 1.0x',
          c.layer_costs_for(['F.Cu', 'In9.Cu']) == [1000, 1000])
    two = GridRouteConfig(layer_costs=[1.0, 1.5, -1, 1.0])
    check('precondition: the 2-layer default truncates a 4-layer cost list',
          two.layers == ['F.Cu', 'B.Cu'] and two.get_layer_costs() == [1000, 1500])

    # 2. The engine's config.
    cfg = engine_config(None)
    check('the engine reached its config', cfg is not None)
    if cfg is not None:
        check('auto-detected: config.layers is every copper layer',
              list(cfg.layers) == FOUR, str(cfg.layers))
        check('auto-detected: In2 stays forbidden for the joins',
              cfg.layer_costs_for(FOUR) == [1000, 3000, F, 1000],
              str(cfg.layer_costs_for(FOUR)))
    sub = ['F.Cu', 'In2.Cu', 'B.Cu']
    cfg = engine_config(sub)
    if cfg is not None:
        check('explicit subset: config.layers is the subset',
              list(cfg.layers) == sub, str(cfg.layers))
        check('explicit subset: costs one per routing layer',
              len(cfg.get_layer_costs()) == len(sub))

    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
