"""One route_planes `--nets` argument names the nets one plane layer pours.

A split layer is ONE argument with its nets joined by `|`, quoted on the
command line (`--nets GND '+3V3|+5V' --plane-layers In1.Cu In2.Cu`); the
repeated-layer form (`--nets GND +3V3 +5V --plane-layers In1.Cu In2.Cu
In2.Cu`) means the same. Every reader of a route_planes command splits the
group here, so none of them reads `+3V3|+5V` as one net name (#1495): the
tool itself, route_plan_check's coverage rule, manifest_to_plan's GUI plan
and render_power's chain reader.
"""
from typing import Iterable, List


def split_plane_group(arg) -> List[str]:
    """The nets one `--nets` argument pours: `'+3V3|+5V'` -> `['+3V3', '+5V']`."""
    return [n.strip() for n in str(arg).split('|') if n.strip()]


def plane_net_names(args: Iterable) -> List[str]:
    """Every net a route_planes `--nets` list pours, in order, groups split."""
    return [n for a in args for n in split_plane_group(a)]
