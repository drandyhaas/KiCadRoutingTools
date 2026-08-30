#!/usr/bin/env python3
"""
Gate for the PLURAL (layers ...) form of net-tagged copper GRAPHICS.

KiCad writes the singular ``(layer "F.Cu")`` for a graphic on one layer and the
PLURAL ``(layers "F.Cu" "F.Mask")`` for the same item when it also carries mask
or paste. ``extract_segments``'s #337 copper-graphics pass matched only the
singular token, so the plural form was dropped ENTIRELY -- not connectivity,
and (the dangerous half) **not an obstacle**.

Measured on stress board ``endgame_trackball`` (set 19): 8 filled 3.2 mm copper
``gr_circle``s written with ``(layers "F.Cu" "F.Mask")`` were invisible. The
unrouted input had 0 foreign-net tracks crossing them; the routed output had
**13**, across 6 different nets -- real shorts that ``check_drc`` could not see
because it reads this same parser. Found while measuring issue #659.

Invariants gated here (both directions -- a gate that only proves the new form
is now parsed would still pass if the pass started swallowing everything):

  1. PLURAL copper form is parsed, on the copper layer, with its net and width.
  2. SINGULAR copper form still parses (no regression).
  3. A plural form naming NO copper layer emits nothing (negative control).
  4. A two-sided graphic emits copper on BOTH copper layers.
  5. Only the ``.Cu`` member of a plural list becomes copper -- F.Mask does not.
  6. Every gr_* tag honours the plural form, not just gr_circle.

Run:
    python3 tests/test_copper_graphics_layers_plural.py
"""

import os
import sys

_TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS_DIR)
for _p in (_ROOT, os.path.join(_ROOT, 'py_router')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from kicad_parser import extract_segments

NAME_TO_ID = {'GNDS': 7, 'SIG': 3}

FAILURES = []


def check(cond, label):
    print(f"  {'PASS' if cond else 'FAIL'}  {label}")
    if not cond:
        FAILURES.append(label)


def graphics(content):
    return [s for s in extract_segments(content, NAME_TO_ID)
            if getattr(s, 'graphic', False)]


def test_plural_circle_parsed():
    """1 + 5: the endgame_trackball shape -- a filled copper disc on
    (layers "F.Cu" "F.Mask")."""
    content = '''
	(gr_circle
		(center 35.9791 40.2336)
		(end 37.5791 40.2336)
		(stroke (width 0.1) (type default))
		(fill yes)
		(layers "F.Cu" "F.Mask")
		(net "GNDS")
		(uuid "ff2a8d57-60f6-4e06-9041-61dcc8816475")
	)
'''
    segs = graphics(content)
    check(len(segs) > 0, "plural (layers ...) copper gr_circle is parsed at all")
    # `segs and ...` on every one of these: `all()` over an EMPTY list is
    # True, so without the conjunct each of these would report PASS against
    # the very parser that drops the block -- a vacuous gate that reads green
    # exactly when the bug is present.
    check(bool(segs) and all(s.layer == 'F.Cu' for s in segs),
          "only the .Cu member becomes copper (F.Mask does not)")
    check(bool(segs) and all(s.net_id == 7 for s in segs),
          "net name resolves through the plural form")
    check(bool(segs) and all(abs(s.width - 0.1) < 1e-9 for s in segs),
          "stroke width survives")
    # the disc is closed: every emitted vertex is within r of the centre
    import math
    ok = bool(segs) and all(
        abs(math.hypot(s.start_x - 35.9791, s.start_y - 40.2336) - 1.6) < 1e-6
        for s in segs)
    check(ok, "emitted outline sits on the disc's true radius (1.6 mm)")


def test_singular_still_parsed():
    """2: negative control on the OTHER side -- the singular form must not
    have been broken by teaching the parser the plural one."""
    content = '''
	(gr_line
		(start 10.0 10.0)
		(end 20.0 10.0)
		(stroke (width 0.25) (type default))
		(layer "B.Cu")
		(net "SIG")
	)
'''
    segs = graphics(content)
    check(len(segs) == 1, "singular (layer ...) copper gr_line still parses (1 seg)")
    check(bool(segs) and segs[0].layer == 'B.Cu', "singular form keeps its layer")


def test_non_copper_plural_emits_nothing():
    """3: the guard must still REFUSE a plural list with no copper in it --
    otherwise 'now parsed' would just mean 'parses everything'."""
    content = '''
	(gr_circle
		(center 5.0 5.0)
		(end 6.0 5.0)
		(stroke (width 0.1) (type default))
		(fill yes)
		(layers "F.SilkS" "F.Mask")
		(net "GNDS")
	)
'''
    check(graphics(content) == [], "silkscreen/mask-only plural list emits no copper")


def test_two_sided_emits_both_layers():
    """4: a graphic on two copper layers is copper on both."""
    content = '''
	(gr_rect
		(start 0.0 0.0)
		(end 4.0 2.0)
		(stroke (width 0.2) (type default))
		(fill yes)
		(layers "F.Cu" "B.Cu")
		(net "SIG")
	)
'''
    layers = {s.layer for s in graphics(content)}
    check(layers == {'F.Cu', 'B.Cu'},
          f"two-sided plural graphic emits on both copper layers (got {sorted(layers)})")


def test_every_tag_honours_plural():
    """6: the fix lives in the shared field reader, so every gr_* tag must
    benefit -- not only the gr_circle that exposed it."""
    blocks = {
        'gr_line': '(gr_line (start 0 0) (end 1 0) (stroke (width 0.2)) '
                   '(layers "F.Cu" "F.Mask") (net "SIG"))',
        'gr_arc': '(gr_arc (start 0 0) (mid 0.5 0.5) (end 1 0) (stroke (width 0.2)) '
                  '(layers "F.Cu" "F.Mask") (net "SIG"))',
        'gr_poly': '(gr_poly (pts (xy 0 0) (xy 1 0) (xy 1 1)) (width 0.2) (fill yes) '
                   '(layers "F.Cu" "F.Mask") (net "SIG"))',
        'gr_rect': '(gr_rect (start 0 0) (end 1 1) (stroke (width 0.2)) (fill yes) '
                   '(layers "F.Cu" "F.Mask") (net "SIG"))',
        'gr_circle': '(gr_circle (center 0 0) (end 1 0) (stroke (width 0.2)) (fill yes) '
                     '(layers "F.Cu" "F.Mask") (net "SIG"))',
    }
    for tag, blk in blocks.items():
        segs = graphics('\n\t' + blk + '\n')
        check(len(segs) > 0 and all(s.layer == 'F.Cu' for s in segs),
              f"{tag} honours the plural (layers ...) form")


def main():
    print(__doc__.strip().splitlines()[0])
    for fn in (test_plural_circle_parsed, test_singular_still_parsed,
               test_non_copper_plural_emits_nothing,
               test_two_sided_emits_both_layers, test_every_tag_honours_plural):
        print(f"\n{fn.__name__}:")
        fn()
    print()
    if FAILURES:
        print(f"FAILED ({len(FAILURES)}): " + "; ".join(FAILURES))
        return 1
    print("All copper-graphics plural-layers invariants hold.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
