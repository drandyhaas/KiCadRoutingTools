#!/usr/bin/env python3
"""#382 E5: engine result-shape contracts.

- batch_route._empty_results_data() must carry EXACTLY the keys the full
  return_results path builds (all empty), so a GUI caller that iterates a key
  can never KeyError on an early-exit path.
- create_plane must return a 3-tuple (CLI, return_results=False) or an 8-tuple
  (GUI, return_results=True) from EVERY path, including validation-error early
  exits -- the GUI unpacks exactly 8 values.
"""
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import route
import route_planes


def _ok(name, cond):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}")
    return bool(cond)


def _full_path_keys():
    """Keys of the dict batch_route builds under `if return_results:` (full path),
    scraped from source so this test fails if the two drift apart."""
    src = open(os.path.join(os.path.dirname(__file__), '..', 'route.py'),
               encoding='utf-8').read()
    # The full-path dict is the one assigned to `results_data = {` with the
    # long comment about "Return results data for direct application".
    m = re.search(r"results_data = \{(.*?)\n        \}", src, re.DOTALL)
    assert m, "could not locate full-path results_data dict"
    return set(re.findall(r"'([a-z_]+)':", m.group(1)))


def main():
    r = []

    empty = route._empty_results_data()
    full_keys = _full_path_keys()
    r.append(_ok(f"_empty_results_data keys == full-path keys ({len(full_keys)})",
                 set(empty.keys()) == full_keys))
    r.append(_ok("_empty_results_data values all empty lists",
                 all(v == [] for v in empty.values())))

    # create_plane shape from a validation-error path (net/layer count mismatch),
    # which is the first early return, in both modes.
    cli = route_planes.create_plane('x', 'out.kicad_pcb', ['GND'],
                                    ['In1.Cu', 'In2.Cu'], return_results=False)
    gui = route_planes.create_plane('x', '', ['GND'],
                                    ['In1.Cu', 'In2.Cu'], return_results=True)
    r.append(_ok("create_plane CLI error return is a 3-tuple", len(cli) == 3))
    r.append(_ok("create_plane GUI error return is an 8-tuple", len(gui) == 8))
    try:
        a, b, c, d, e, f, g, h = gui  # exactly how planes_gui unpacks it
        r.append(_ok("create_plane GUI 8-unpack succeeds", True))
    except ValueError:
        r.append(_ok("create_plane GUI 8-unpack succeeds", False))

    passed = sum(r)
    print(f"\n{passed}/{len(r)} result-shape tests passed")
    return 0 if passed == len(r) else 1


if __name__ == "__main__":
    sys.exit(main())
