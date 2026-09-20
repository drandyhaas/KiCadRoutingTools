#!/usr/bin/env python3
"""Measure a render palette: contrast, colour-vision separation, compositing.

`render_theme` (once it lands, #1011) declares WHAT COLOUR things are; this
measures whether that declaration actually works. It is the instrument behind
`tests/test_946_palette_measures.py`, and it is deliberately a separate file so
that the gate can re-derive every number from the shipped palette instead of
comparing against a constant somebody typed.

**STDLIB ONLY, AND NEVER PIL.** The palette must be measurable without the
raster stack: `py_tools/render_placement.py` imports the palette at module scope
and must still import with ``sys.modules['PIL'] = None``
(``tests/test_943_optional_render_dependency.py:153-178`` asserts exactly that,
because ``board_context.py`` and the stress predictors import it for
``PlacementModel`` / ``legality_findings`` and draw nothing).

**NO ``set()`` ANYWHERE IN A PALETTE PATH.**
``tests/test_431_render_placement.py:175-195`` requires byte-identical PNGs
across two subprocesses with different ``PYTHONHASHSEED``. A palette or a report
assembled from set iteration order flaps that test intermittently, which is the
worst possible failure mode for a colour change. Ordered tuples and dict
literals only, and ``json.dump(..., sort_keys=True)``.

**THE INSTRUMENT IS PINNED SEPARATELY FROM THE PALETTE.** ``--self-test``
asserts the transforms against three published fixtures in milliseconds. Without
it a broken deuteranope transform reads as a broken palette, and the two have
very different fixes. Same posture as ``awx/evolve_movie.py``'s ``self_test()``,
which runs on every invocation for the same reason.

Method: WCAG 2.x relative luminance and contrast ratio; the Vienot, Brettel &
Mollon (1999) deuteranope projection applied in linear sRGB; and the same linear
blend ``route_render.BoardRenderer.frame()`` performs at ``layer_alpha``.
"""
from __future__ import annotations

KRT_TOOL = {'scope': ['routing', 'placement'], 'kind': 'instrument'}

import argparse
import json
import math
import os
import sys
from typing import Dict, List, Sequence, Tuple

RGB = Tuple[int, int, int]

# --------------------------------------------------------------------------
# colour maths
# --------------------------------------------------------------------------


def srgb_to_linear(v: float) -> float:
    """One 0..255 channel to linear light."""
    v /= 255.0
    return v / 12.92 if v <= 0.04045 else ((v + 0.055) / 1.055) ** 2.4


def linear_to_srgb(v: float) -> int:
    v = max(0.0, min(1.0, v))
    out = 12.92 * v if v <= 0.0031308 else 1.055 * v ** (1 / 2.4) - 0.055
    return int(round(255 * out))


def relative_luminance(c: Sequence[int]) -> float:
    """WCAG 2.x relative luminance."""
    r, g, b = (srgb_to_linear(x) for x in c)
    return 0.2126 * r + 0.7152 * g + 0.0722 * b


def contrast_ratio(a: Sequence[int], b: Sequence[int]) -> float:
    """WCAG 2.x contrast ratio, always >= 1.0."""
    la, lb = relative_luminance(a), relative_luminance(b)
    hi, lo = max(la, lb), min(la, lb)
    return (hi + 0.05) / (lo + 0.05)


def deuteranope(c: Sequence[int]) -> RGB:
    """Vienot, Brettel & Mollon (1999), applied in linear sRGB.

    Roughly 8% of men and 0.5% of women see this transform of the frame. It is
    the harsher of the two common red-green deficiencies for this palette
    because it collapses toward the L axis, which is where `_RIP` and the old
    green `_RESTORE` both sit.
    """
    r, g, b = (srgb_to_linear(x) for x in c)
    lms_l = 17.8824 * r + 43.5161 * g + 4.11935 * b
    lms_s = 0.0299566 * r + 0.184309 * g + 1.46709 * b
    lms_m = 0.494207 * lms_l + 1.24827 * lms_s
    return (
        linear_to_srgb(0.080944 * lms_l - 0.130504 * lms_m + 0.116721 * lms_s),
        linear_to_srgb(-0.0102485 * lms_l + 0.0540194 * lms_m - 0.113615 * lms_s),
        linear_to_srgb(-0.000365294 * lms_l - 0.00412163 * lms_m + 0.693513 * lms_s),
    )


def rgb_distance(a: Sequence[int], b: Sequence[int]) -> float:
    """Euclidean distance in sRGB. Crude as a perceptual metric and the right
    one here anyway: it is the space the constants are written in, and every
    number in #946 is quoted in it."""
    return math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a, b)))


def composite(fg: Sequence[int], bg: Sequence[int], alpha: int) -> RGB:
    """The linear blend `frame()` performs at `layer_alpha`.

    This is why the declared layer palette is not what reaches the screen:
    every layer contracts toward the board body by `1 - alpha/255`.
    """
    f = alpha / 255.0
    return tuple(int(round(fg[i] * f + bg[i] * (1 - f))) for i in range(3))


# --------------------------------------------------------------------------
# the palette under measurement
# --------------------------------------------------------------------------

#: Today's constants, harvested from the modules that own them. #1011 replaces
#: this with `render_theme.THEMES`; the maths above does not change, which is
#: the whole point of landing the instrument first.
_TODAY = {
    'name': 'dark (as shipped)',
    'ground': (14, 16, 18),          # route_render._BG
    'board_body': (26, 34, 28),      # route_render._BOARD_FILL
    'edge': (225, 225, 210),         # route_render._EDGE
    'pad': (192, 168, 96),           # route_render._PAD
    'pad_hole': (10, 10, 10),        # route_render._PAD_HOLE
    'via': (176, 176, 184),          # route_render._VIA
    'hilite': (255, 60, 60),         # route_render._HILITE
    'event_new': (250, 250, 250),    # animate_route._NEW
    'event_restored': (86, 224, 96), # animate_route._RESTORE
    'event_ripped': (255, 66, 66),   # animate_route._RIP
    'defect_conflict': (255, 64, 64),    # render_placement.C_CONFLICT
    'defect_net_fail': (232, 72, 72),    # render_placement.C_AIR_FAIL
    'status_tried': (200, 60, 60),       # make_film._badge default
    'layer_alpha': 150,
    'layers': (
        (208, 64, 58), (70, 130, 210), (96, 190, 96), (214, 190, 78),
        (196, 110, 206), (94, 200, 200), (224, 150, 70), (150, 150, 224),
        (170, 210, 90), (210, 120, 150),
    ),
}

#: Names in stack order, matching `route_render.layer_palette`'s assignment.
LAYER_NAMES = ('F.Cu', 'B.Cu', 'In1', 'In2', 'In3',
               'In4', 'In5', 'In6', 'In7', 'In8')

#: The three roles a viewer must never confuse, in report order.
EVENT_ROLES = ('event_new', 'event_restored', 'event_ripped')

#: Roles that are red today and mean four different things (#946).
RED_FAMILY = ('event_ripped', 'defect_conflict', 'defect_net_fail',
              'status_tried')


def current_palette() -> Dict:
    """Today's palette, as a plain dict. Returns a copy: a caller proposing a
    variant must not mutate the shipped one underneath the gate."""
    out = dict(_TODAY)
    out['layers'] = tuple(_TODAY['layers'])
    return out


# --------------------------------------------------------------------------
# measurements
# --------------------------------------------------------------------------


def measure_events(pal: Dict) -> Dict:
    """Each event against the board body, and each pair against each other --
    in normal vision and under a deuteranope transform."""
    body = pal['board_body']
    contrast = {}
    for role in EVENT_ROLES:
        contrast[role] = round(contrast_ratio(pal[role], body), 4)
    pairs = {}
    for i in range(len(EVENT_ROLES)):
        for j in range(i + 1, len(EVENT_ROLES)):
            a, b = EVENT_ROLES[i], EVENT_ROLES[j]
            pairs['%s|%s' % (a, b)] = {
                'normal': round(rgb_distance(pal[a], pal[b]), 4),
                'deuteranope': round(
                    rgb_distance(deuteranope(pal[a]), deuteranope(pal[b])), 4),
            }
    rip, res = pal['event_ripped'], pal['event_restored']
    return {
        'contrast_vs_board': contrast,
        'contrast_min': round(min(contrast.values()), 4),
        'pairs': pairs,
        'rip_restore_deuteranope': pairs['event_restored|event_ripped']['deuteranope'],
        'rip_restore_luminance_ratio': round(contrast_ratio(rip, res), 4),
        'pair_deuteranope_min': round(
            min(p['deuteranope'] for p in pairs.values()), 4),
    }


def measure_structure(pal: Dict) -> Dict:
    """The tokens that draw the board itself, against the board body.

    #946 measured the events and the layers and never these. `edge` is the one
    that decides whether a frame has a board in it at all.
    """
    body = pal['board_body']
    out = {}
    for role in ('edge', 'pad', 'via', 'pad_hole'):
        if role in pal:
            out[role] = round(contrast_ratio(pal[role], body), 4)
    out['board_vs_ground'] = round(contrast_ratio(body, pal['ground']), 4)
    return out


def measure_red_family(pal: Dict) -> Dict:
    """Every pair among the roles that are red today.

    The worst pair is `event_ripped` vs `defect_conflict` at 2.8 -- one is
    something the router DID, the other something WRONG WITH THE BOARD, and
    they are the same colour.
    """
    present = tuple(r for r in RED_FAMILY if r in pal)
    pairs = {}
    for i in range(len(present)):
        for j in range(i + 1, len(present)):
            a, b = present[i], present[j]
            pairs['%s|%s' % (a, b)] = round(rgb_distance(pal[a], pal[b]), 4)
    return {'pairs': pairs,
            'min': round(min(pairs.values()), 4) if pairs else None}


def measure_layers(pal: Dict, alpha: int = None) -> Dict:
    """The layer palette AS RENDERED -- composited at `layer_alpha` over the
    board body, which is not what the constants say.

    Two different quantities, and #946 measured only the first:
      * separation BETWEEN layers, which compositing preserves;
      * contrast AGAINST the board, which it does not.
    """
    alpha = pal['layer_alpha'] if alpha is None else alpha
    body = pal['board_body']
    rendered = tuple(composite(c, body, alpha) for c in pal['layers'])
    n = len(rendered)
    dists: List[Tuple[float, str, str]] = []
    for i in range(n):
        for j in range(i + 1, n):
            dists.append((rgb_distance(rendered[i], rendered[j]),
                          LAYER_NAMES[i], LAYER_NAMES[j]))
    dists.sort()
    contrasts = [contrast_ratio(c, body) for c in rendered]
    return {
        'alpha': alpha,
        'rendered': {LAYER_NAMES[i]: list(rendered[i]) for i in range(n)},
        'closest_pair': round(dists[0][0], 4),
        'closest_pair_names': '%s|%s' % (dists[0][1], dists[0][2]),
        'mean_pair': round(sum(d[0] for d in dists) / len(dists), 4),
        'contrast_min': round(min(contrasts), 4),
        'contrast_min_layer': LAYER_NAMES[contrasts.index(min(contrasts))],
        'contrast_mean': round(sum(contrasts) / len(contrasts), 4),
    }


def measure_crossings(pal: Dict, threshold: float = 34.0,
                      alpha: int = None) -> Dict:
    """Two-layer crossings that impersonate a THIRD layer.

    Each layer is alpha-composited onto the accumulated image, so an overlap is
    a BLEND -- and the blend can land on a real layer's solo appearance. This
    is the part that affects every viewer, not only colour-deficient ones.
    """
    alpha = pal['layer_alpha'] if alpha is None else alpha
    body = pal['board_body']
    solo = tuple(composite(c, body, alpha) for c in pal['layers'])
    n = len(solo)
    hits = []
    for i in range(n):
        for j in range(n):
            if i == j:
                continue
            blend = composite(pal['layers'][j], solo[i], alpha)
            for k in range(n):
                if k in (i, j):
                    continue
                dist = rgb_distance(blend, solo[k])
                if dist < threshold:
                    hits.append({
                        'over': LAYER_NAMES[j], 'under': LAYER_NAMES[i],
                        'reads_as': LAYER_NAMES[k],
                        'renders': list(blend), 'distance': round(dist, 4),
                    })
    hits.sort(key=lambda h: (h['distance'], h['over'], h['under'], h['reads_as']))
    return {'threshold': threshold, 'count': len(hits),
            'worst': hits[0] if hits else None, 'hits': hits}


def audit(pal: Dict = None) -> Dict:
    """Every measurement, as one JSON-shaped document."""
    pal = current_palette() if pal is None else pal
    return {
        'schema': 1,
        'kind': 'palette-audit',
        'palette': pal['name'],
        'events': measure_events(pal),
        'structure': measure_structure(pal),
        'red_family': measure_red_family(pal),
        'layers': measure_layers(pal),
        'crossings': measure_crossings(pal),
    }


# --------------------------------------------------------------------------
# the instrument's own pins
# --------------------------------------------------------------------------

#: Published values the TRANSFORMS must reproduce, so a broken transform can
#: never be read as a broken palette. Each is checkable by hand.
SELF_TEST_FIXTURES = (
    ('contrast black/white == 21.00',
     lambda: contrast_ratio((0, 0, 0), (255, 255, 255)), 21.0, 0.001),
    ('relative_luminance(white) == 1.0',
     lambda: relative_luminance((255, 255, 255)), 1.0, 1e-9),
    ('relative_luminance(black) == 0.0',
     lambda: relative_luminance((0, 0, 0)), 0.0, 1e-9),
    # #946's own two headline numbers, which the note quotes as 76 and 187.
    ('deuteranope rip|green == 76 (the collapse #946 opened on)',
     lambda: rgb_distance(deuteranope((255, 66, 66)),
                          deuteranope((86, 224, 96))), 76.0, 1.5),
    ('deuteranope rip|cyan == 187 (the fix)',
     lambda: rgb_distance(deuteranope((255, 66, 66)),
                          deuteranope((80, 215, 230))), 187.0, 1.5),
    ('composite is the identity at alpha 255',
     lambda: rgb_distance(composite((208, 64, 58), (26, 34, 28), 255),
                          (208, 64, 58)), 0.0, 1e-9),
)


def self_test(quiet: bool = False) -> int:
    """Pin the TRANSFORMS, not the palette. Milliseconds, no I/O."""
    bad = 0
    for label, fn, want, tol in SELF_TEST_FIXTURES:
        got = fn()
        ok = abs(got - want) <= tol
        if not ok:
            bad += 1
        if not quiet or not ok:
            print('  %-4s %-56s got %.4f want %.4f (+-%g)'
                  % ('ok' if ok else 'FAIL', label, got, want, tol))
    if bad:
        print('palette_audit --self-test: %d of %d fixtures FAILED. The '
              'instrument is wrong; do not trust any palette number it '
              'produced.' % (bad, len(SELF_TEST_FIXTURES)), file=sys.stderr)
    elif not quiet:
        print('palette_audit --self-test: %d/%d transforms pinned'
              % (len(SELF_TEST_FIXTURES), len(SELF_TEST_FIXTURES)))
    return 1 if bad else 0


# --------------------------------------------------------------------------
# report
# --------------------------------------------------------------------------


def format_report(doc: Dict) -> str:
    ev, st, rf = doc['events'], doc['structure'], doc['red_family']
    ly, cr = doc['layers'], doc['crossings']
    L = []
    L.append('palette: %s' % doc['palette'])
    L.append('')
    L.append('EVENTS -- contrast vs the board body')
    for role in EVENT_ROLES:
        L.append('  %-16s %7.2fx' % (role, ev['contrast_vs_board'][role]))
    L.append('EVENTS -- separation')
    for key, p in sorted(ev['pairs'].items()):
        L.append('  %-34s normal %7.1f   deuteranope %7.1f'
                 % (key, p['normal'], p['deuteranope']))
    L.append('  rip:restore luminance ratio %.2fx'
             % ev['rip_restore_luminance_ratio'])
    L.append('')
    L.append('STRUCTURE -- contrast vs the board body')
    for k in sorted(st):
        L.append('  %-16s %7.2fx' % (k, st[k]))
    L.append('')
    L.append('THE RED FAMILY -- how far apart four different meanings are')
    for key, d in sorted(rf['pairs'].items(), key=lambda kv: kv[1]):
        L.append('  %-46s %7.1f' % (key, d))
    L.append('')
    L.append('LAYERS -- as rendered at alpha %d' % ly['alpha'])
    L.append('  closest pair %.1f (%s)   mean pair %.1f'
             % (ly['closest_pair'], ly['closest_pair_names'], ly['mean_pair']))
    L.append('  contrast vs board: min %.2fx (%s)   mean %.2fx'
             % (ly['contrast_min'], ly['contrast_min_layer'],
                ly['contrast_mean']))
    L.append('')
    L.append('CROSSINGS -- two layers blending into a third, within %.0f'
             % cr['threshold'])
    L.append('  %d collisions' % cr['count'])
    for h in cr['hits'][:5]:
        L.append('    %-5s over %-5s renders %-17s reads as %-5s (%.1f)'
                 % (h['over'], h['under'], str(tuple(h['renders'])),
                    h['reads_as'], h['distance']))
    return '\n'.join(L)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description='Measure a render palette: contrast, colour-vision '
                    'separation, compositing collisions (#946).')
    ap.add_argument('--json', metavar='PATH',
                    help='write the measurement as JSON')
    ap.add_argument('--write-baseline', action='store_true',
                    help='re-record tests/946_theme_contrast_baseline.json')
    ap.add_argument('--self-test', action='store_true',
                    help='pin the TRANSFORMS against published fixtures and '
                         'exit; a broken transform must not read as a broken '
                         'palette')
    ap.add_argument('--quiet', action='store_true')
    a = ap.parse_args(argv)

    if a.self_test:
        return self_test(quiet=a.quiet)

    # The instrument checks itself before it measures anything, always.
    if self_test(quiet=True):
        return 1

    doc = audit()
    if not a.quiet:
        print(format_report(doc))

    if a.json:
        os.makedirs(os.path.dirname(os.path.abspath(a.json)) or '.',
                    exist_ok=True)
        with open(a.json, 'w', encoding='utf-8') as fh:
            json.dump(doc, fh, indent=1, sort_keys=True)
            fh.write('\n')
        if not a.quiet:
            print('\nwrote %s' % a.json)

    if a.write_baseline:
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        path = os.path.join(root, 'tests', '946_theme_contrast_baseline.json')
        with open(path, 'w', encoding='utf-8') as fh:
            json.dump(doc, fh, indent=1, sort_keys=True)
            fh.write('\n')
        print('wrote baseline %s' % path)

    return 0


if __name__ == '__main__':
    sys.exit(main())
