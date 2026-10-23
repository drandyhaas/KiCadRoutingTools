#!/usr/bin/env python3
"""BGA fanout's surface rescue draws on a concrete copper layer, never '*.Cu'.

The #652 surface rescue picked the first layer of a ball that ends in `.Cu`,
and '*.Cu' does. Over open_weather_station U8's grid of through-hole thermal
pads it wrote 17 GND tracks on layer "*.Cu", a file KiCad refuses to load,
so the whole board went ungraded. `bga_fanout.ball_surface_layer` is the one
choice of layer: a named copper layer as before, the footprint's mount layer
for a through-hole ball, None for a ball with no copper.

    python3 tests/test_bga_surface_rescue_layer.py
"""
import os
import sys
from types import SimpleNamespace

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from bga_fanout import ball_surface_layer        # noqa: E402


def main():
    smd = SimpleNamespace(layers=['F.Cu', 'F.Paste', 'F.Mask'])
    tht = SimpleNamespace(layers=['*.Cu', '*.Mask'])
    back = SimpleNamespace(layers=['B.Cu', 'B.Mask'])
    none = SimpleNamespace(layers=['F.Mask'])
    assert ball_surface_layer(smd) == 'F.Cu'
    assert ball_surface_layer(back, SimpleNamespace(layer='F.Cu')) == 'B.Cu'
    assert ball_surface_layer(tht, SimpleNamespace(layer='B.Cu')) == 'B.Cu'
    assert ball_surface_layer(tht) == 'F.Cu'
    assert ball_surface_layer(none) is None
    for p in (smd, tht, back):
        assert not ball_surface_layer(p, SimpleNamespace(layer='F.Cu')).startswith('*')
    print('  PASS: a named copper layer as given; a through-hole ball on its part\'s '
          'mount layer, never "*.Cu"; no copper, no layer')
    print('test_bga_surface_rescue_layer: all passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
