#!/usr/bin/env python3
"""#908: check_drc must see FOREIGN copper against a footprint's own copper.

The point of modelling `fp_*` copper is that a foreign track laid across a
SOT89 tab is caught here instead of on the fab. The mirror-image requirement
is that the footprint's OWN pad under its OWN copper is NOT reported: that is
fixed library geometry, never pipeline-introduced, and the repo already holds
that doctrine for pad-vs-pad (`tests/test_drc_pad_pad.py:7-11`). KiCad's own
connectivity unification says the same thing -- it derives a graphic's net
from what touches it (#337) -- so the exemption is KiCad's rule, not a waiver
invented here.

Invariants gated here:
  1. A foreign-net track crossing a footprint's F.Cu poly IS reported, and
     the report NAMES the polygon (`Polygon(U1)`), the way KiCad does.
  2. The same track on B.Cu is NOT reported (the layer is respected).
  3. The footprint's own pad under its own poly is NOT reported.
  4. A track on the same net as the touched pad is NOT reported.
  5. `no_net` is set on a violation involving netless copper: a pad with no
     net cannot electrically SHORT a net (the existing pad-pad idiom).

Run:
    python3 tests/test_908_drc_report.py
"""

import os
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_router'))  # #522
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_tools'))  # #522

from check_drc import run_drc

RUN_ALL_FAST_OK = True

#: U1 sits at (10,10) with pad 1 at its origin and a 3x1 mm F.Cu poly running
#: east from the pad. A track at y=10 crosses that poly.
BOARD = '''(kicad_pcb
 (version 20221018)
 (net 0 "")
 (net 1 "/A")
 (net 2 "/B")
 (footprint "L:P" (layer "F.Cu") (at 10 10)
   (property "Reference" "U1")
   (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 1 "/A"))
   (fp_poly (pts (xy 0.4 -0.5) (xy 3.4 -0.5) (xy 3.4 0.5) (xy 0.4 0.5))
     (stroke (width 0) (type solid)) (fill yes)
     (layer "F.Cu") (uuid "poly1")))
%s
)'''

#: crosses the poly's northern edge at x=12
TRACK = ('(segment (start 12 8) (end 12 12) (width 0.15) (layer "%s")'
         ' (net %d) (uuid "t1"))')


def _run(body):
    with tempfile.NamedTemporaryFile('w', suffix='.kicad_pcb',
                                     delete=False, encoding='utf-8') as fh:
        fh.write(BOARD % body)
        path = fh.name
    try:
        return run_drc(path, clearance=0.2, quiet=True)
    finally:
        os.unlink(path)


def main():
    fails = []

    def check(name, cond, detail=''):
        print(f"  {'PASS' if cond else 'FAIL'}: {name}"
              + (f"   [{detail}]" if detail and not cond else ''))
        if not cond:
            fails.append(name)

    def involving_poly(vs):
        return [v for v in vs
                if 'Polygon' in str(v.get('item1', ''))
                or 'Polygon' in str(v.get('item2', ''))]

    # --- 1: foreign copper across the tab IS caught, and NAMED -------------
    vs = _run(TRACK % ('F.Cu', 2))
    hits = involving_poly(vs)
    check('a foreign track across a footprint poly is reported',
          len(hits) >= 1, f'{len(hits)} of {len(vs)} violations')
    check('the report names the polygon by its owner',
          bool(hits) and any('Polygon(U1)' in str(h.get('item1', ''))
                             + str(h.get('item2', '')) for h in hits),
          f'{[ (h.get("item1"), h.get("item2")) for h in hits ]}')
    check('the netless side is marked no_net (a clearance issue, not a short)',
          bool(hits) and all(h.get('no_net') for h in hits
                             if h['type'] in ('segment-segment',
                                              'pad-segment')))

    # --- 2: the same track on the other side is NOT caught -----------------
    vs_b = _run(TRACK % ('B.Cu', 2))
    check('the same track on B.Cu is not reported against an F.Cu poly',
          involving_poly(vs_b) == [],
          f'{involving_poly(vs_b)}')

    # --- 3: the footprint's own pad under its own copper -------------------
    vs_none = _run('')
    check('the footprint\'s own pad under its own poly is not reported',
          involving_poly(vs_none) == [], f'{involving_poly(vs_none)}')

    # --- 4: same-net copper is not reported --------------------------------
    vs_same = _run(TRACK % ('F.Cu', 1))
    check('a track on the touched pad\'s own net is not reported',
          involving_poly(vs_same) == [], f'{involving_poly(vs_same)}')

    # Anti-vacuity: arm 1 must genuinely differ from arms 2-4, otherwise the
    # whole file would pass with a check_drc that reports nothing at all.
    check('the positive arm really differs from the negative arms',
          len(involving_poly(vs)) > 0
          and len(involving_poly(vs_b)) == 0
          and len(involving_poly(vs_same)) == 0)

    print(f"\n{'FAILED: ' + ', '.join(fails) if fails else 'ALL PASS'}")
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
