"""The plane's web through a ball-grid array's via lattice.

A plane on an inner layer reaches the drops under a ball-grid array only
through the copper left between the vias round them. Two vias one pitch
apart, each cleared by the pour's clearance, leave

    pitch - via - 2 * clearance

of plane between them, and the fill drops any neck narrower than the zone's
minimum width. Below that the plane under the array is islands: a drop inside
it sits on a piece of plane that reaches nothing, and the route step's plane
repair taps each such ball with a trace, ripping the signals in its way.
Measured on zynq_ad9364 (0.8 mm arrays, 0.45 mm vias, a 0.2 mm pour, 0.1 mm
minimum width): no web at all, and 100+ pads tapped with 26-69 signal rips
per run.

Two levers, in this order:

* the pour's clearance (`route_planes`), lowered to what threads the array's
  vias, never below the fab floor -- free;
* the fanout's via (`bga_fanout`), stepped down the fab ladder to one that
  threads the pour as it stands -- even below an explicit ``--via-size``,
  disclosed as every other escalation is.
"""
import math

#: mm the web keeps above the zone's minimum width: a neck exactly AT the
#: minimum is the fill's own rounding to keep or drop.
WEB_MARGIN = 0.01

#: KiCad's default zone minimum width, for a zone that does not say.
DEFAULT_MIN_WIDTH = 0.25


def _down(x, q=1e-4):
    """x rounded DOWN to q (0.1 um): a clearance or via read off this module
    never exceeds what threads."""
    return math.floor(x / q + 1e-6) * q


def web(pitch, via, clearance, min_width):
    """The copper between two vias one pitch apart, past the zone's minimum
    width (negative: no web)."""
    return pitch - via - 2.0 * clearance - min_width


def threads(pitch, via, clearance, min_width):
    """True when the pour passes between two vias one pitch apart."""
    return web(pitch, via, clearance, min_width) >= WEB_MARGIN - 1e-9


def clearance_to_thread(pitch, via, min_width):
    """The largest pour clearance that passes between vias of this size."""
    return _down((pitch - via - min_width - WEB_MARGIN) / 2.0)


def via_to_thread(pitch, clearance, min_width):
    """The largest via the pour passes between at this clearance."""
    return _down(pitch - 2.0 * clearance - min_width - WEB_MARGIN)


def _inside(poly, x, y):
    """Point in polygon (even-odd ray cast) over a zone outline's vertices."""
    hit = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
            hit = not hit
    return hit


def array_plane_zones(footprint, pcb_data):
    """The board's pours that must reach drops under this array: a zone whose
    outline covers a ball of its own net (a split plane's region may cover
    only some of them; a footprint's own zone aside)."""
    balls = {}
    for p in footprint.pads:
        if p.net_id:
            balls.setdefault(p.net_id, []).append((p.global_x, p.global_y))
    return [z for z in (pcb_data.zones or [])
            if z.net_id in balls and not getattr(z, 'in_footprint', False)
            and len(z.polygon or []) >= 3
            and any(_inside(z.polygon, x, y) for x, y in balls[z.net_id])]


def plane_web_via(footprint, pcb_data, via_size, via_drill, clearance,
                  rungs, pitch=None):
    """The via the fanout lays under ``footprint`` so its plane pours thread
    the lattice, or None when there is nothing to change.

    Each pour over the array is held at max(its own clearance, ``clearance``)
    -- KiCad fills at the larger of the zone's and the class's -- and its own
    minimum width. ``rungs`` is the fab ladder the site may step down
    (`fab_tiers.escalation_rungs`; empty under ``--escalation off``): the
    largest rung via below ``via_size`` that threads every pour is taken, with
    min(``via_drill``, the rung's drill). Returns a dict: ``via`` / ``drill``
    (unchanged when no rung threads), ``threads`` (False: no rung does),
    ``pitch``, ``clearance`` / ``min_width`` (the binding pour's), ``nets``
    (the pours' net names) and ``asked`` (the via it was given)."""
    if pitch is None:
        from kicad_parser import detect_bga_pitch
        pitch = detect_bga_pitch(footprint)
    if not pitch:
        return None
    zones = array_plane_zones(footprint, pcb_data)
    if not zones:
        return None
    worst = None
    for z in zones:
        c = max(z.clearance if z.clearance is not None else clearance, clearance)
        t = z.min_thickness if z.min_thickness is not None else DEFAULT_MIN_WIDTH
        cap = via_to_thread(pitch, c, t)
        if worst is None or cap < worst[0]:
            worst = (cap, c, t)
    cap, c, t = worst
    nets = sorted({z.net_name for z in zones if z.net_name})
    out = {'pitch': pitch, 'clearance': c, 'min_width': t, 'nets': nets,
           'asked': via_size, 'via': via_size, 'drill': via_drill, 'threads': True}
    if via_size <= cap + 1e-9:
        return None
    for d, dr in sorted({(r['via_diameter'], r['via_drill']) for r in rungs},
                        reverse=True):
        if d < via_size - 1e-9 and d <= cap + 1e-9:
            out.update(via=d, drill=min(via_drill, dr))
            return out
    out['threads'] = False
    return out
