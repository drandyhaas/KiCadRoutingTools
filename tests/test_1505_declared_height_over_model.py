"""#1505: a declared height is the part's height, over its 3D model too.

Under a declared `covers[]` body the declared `height_mm` used to decide
only the quick "fits" test; past it the part's MODEL decided cell by cell,
so a part declared 9.0 mm with a 0.35 mm model was admitted under an 8.0 mm
underside -- while the same declaration was refused on a part with no
model, and a height-limited keep-out honoured it. `with_declared` now gives
a declared part's model cells the declared height (every cell when the
declaration is the taller, clipped when it is the lower), so a declared
body, a measured raised body and a keep-out read one height.

Synthetic, no kicad-cli: J carries the bodies, R is a 1 x 0.5 mm part.

    python3 tests/test_1505_declared_height_over_model.py
"""
import contextlib
import io
import os
import sys
from types import SimpleNamespace

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, os.path.join(REPO, 'py_placer'))

import numpy as np  # noqa: E402

from placement import heights  # noqa: E402

failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


C = heights.CELL_MM
SMALL = (-0.5, -0.25, 0.5, 0.25)            # R's occupancy, local
OCC_J = (-1.0, -1.0, 1.0, 1.0)              # J's occupancy, local
J_AT = (100.0, 50.0, 0.0)
PCB = SimpleNamespace(footprints={r: SimpleNamespace(pads=[object()]) for r in ('J', 'R', 'Y')})
#: J's declared body: 10 mm along local +x, 4 mm wide, 8.0 mm up
COVER = {'connector': 'J', 'side': 'F', 'axis_deg': 0, 'width_mm': 4.0, 'offset_mm': 0.0,
         'length_mm': 10.0, 'underside_mm': 8.0, 'source': 'synthetic'}


def model(h):
    """R's model: its 1 x 0.5 mm footprint `h` mm tall, cell by cell."""
    pts = np.array([[x + C / 2, y + C / 2] for x in np.arange(-0.5, 0.5, C)
                    for y in np.arange(-0.25, 0.25, C)])
    return pts, np.full(len(pts), h)


def measured(bodies, covers=None):
    tops = {r: (float(b[1].max()), 'model') for r, b in bodies.items()}
    return heights.Heights(top=dict(tops, J=(1.0, 'model')), covers=covers or {},
                           bodies=bodies, reached={'ZZ': 'none'})


def declared(h, rows, covers=(COVER,)):
    with contextlib.redirect_stdout(io.StringIO()):
        return heights.with_declared(h, {'covers': list(covers), 'heights': rows}, PCB)


def under_decl(h, ref='R'):
    """The pair J / `ref`, `ref` 5 mm along J's declared body: mm^2 refused."""
    return h.pair('J', J_AT, OCC_J, 'F', ref, (105.0, 50.0, 0.0), SMALL, 'F', 0.1) or 0.0


def declared_body():
    print('1. under a declared body (8.0 mm underside)')
    low = measured({'R': model(0.35)})
    check('a 0.35 mm model with nothing declared fits', under_decl(declared(low, [])) == 0.0)
    v = under_decl(declared(low, [{'ref': 'R', 'height_mm': 9.0, 'source': 's'}]))
    check('...declared 9.0 mm it is refused, as its declaration says', v > 0.0, f'{v}')
    v = under_decl(declared(measured({}), [{'ref': 'Y', 'height_mm': 9.0, 'source': 's'}]), 'Y')
    check('...as a part with no model declared 9.0 mm always was', v > 0.0, f'{v}')
    check('a 0.35 mm model declared 0.4 mm fits',
          under_decl(declared(low, [{'ref': 'R', 'height_mm': 0.4, 'source': 's'}])) == 0.0)
    tall = measured({'R': model(9.0)})
    check('a 9.0 mm model is refused', under_decl(declared(tall, [])) > 0.0)
    check('...and declared 0.5 mm, the declaration wins: it fits',
          under_decl(declared(tall, [{'ref': 'R', 'height_mm': 0.5, 'source': 's'}])) == 0.0)


def raised_body():
    print('2. under a measured raised body (2.0 mm up, past the occupancy)')
    n = int(2 / C)
    lo = np.full((n + 24, n), np.inf)        # J's raster from local (-1, -1)
    lo[:n, :] = 5.0                          # over its occupancy
    lo[n:, :] = 2.0                          # raised 2.0 mm for 6 mm past +x
    cov = heights.Cover('J', -1.0, -1.0, lo, np.zeros(lo.shape, bool))
    low = measured({'R': model(0.35)}, covers={'J': cov})
    at = (103.0, 50.0, 0.0)

    def pair(h):
        return h.pair('J', J_AT, OCC_J, 'F', 'R', at, SMALL, 'F', 0.1, inside=False) or 0.0
    check('a 0.35 mm model with nothing declared fits', pair(declared(low, [], ())) == 0.0)
    v = pair(declared(low, [{'ref': 'R', 'height_mm': 3.0, 'source': 's'}], ()))
    check('...declared 3.0 mm it meets the raised body', v > 0.0, f'{v}')
    tall = measured({'R': model(3.0)}, covers={'J': cov})
    check('a 3.0 mm model declared 1.0 mm fits under it',
          pair(declared(tall, [{'ref': 'R', 'height_mm': 1.0, 'source': 's'}], ())) == 0.0)


def main():
    declared_body()
    raised_body()
    if failures:
        print(f'FAIL: {len(failures)}: ' + '; '.join(failures))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
