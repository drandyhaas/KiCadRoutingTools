#!/usr/bin/env python3
"""The IPC adapter must ROUND mm->nm, never truncate.

kipy's own `Vector2.from_xy_mm` / `PolyLineNode.from_xy_mm` go through
`kipy.util.from_mm`, which is a bare `int(value_mm * 1_000_000)`. 66.1mm is
66099999.99999999 in binary float, so it truncates to 66099999 nm -- every
coordinate whose mm value is not exactly representable lands 1 nm short, on
every track, via and zone outline the plugin emits.

This is the SWIG side's #493 `FromMM` bug in kipy's clothing. It was found by
`tests/gui_parity/replay_plan_vs_run.py`, which graded nano_eeprom_prog
EQUIVALENT rather than IDENTICAL on 6 segments 1 nm off; with the adapter
rounding, that board replays IDENTICAL (487/487 segments, 63/63 vias).

Why this file exists as well as that harness: the harness needs KiCad's python,
a stress corpus and ~4 minutes, so nothing cheap would notice someone
"simplifying" vec_mm back to the convenience constructor -- and a 1 nm error is
far below every DRC and fab threshold, so no grader would flag it either.

Run under KiCad's python (kipy is bundled there); SKIPs without kipy, since
there is then no adapter to test rather than a passing one.
"""
import os
import sys

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _ROOT)
sys.path.insert(0, os.path.join(_ROOT, 'py_router'))

# Values float CANNOT hold exactly -- x.1 and x.6 land just below the integer
# nanometre, which is what makes truncation visible. A test built only on 0.25
# or 1.7 would pass against the truncating constructor and prove nothing.
CASES = (66.1, 65.6, 64.6, 175.9, 173.6, 0.1, 12.3)
# ...and values that ARE exact, so a "fix" that perturbs clean coordinates
# (e.g. adding an epsilon) fails too.
EXACT = (0.25, 1.7, 0.5, 100.0, 0.0)


def main():
    try:
        import kipy  # noqa: F401
    except ImportError:
        print("SKIP: kipy not importable -- run under KiCad's python "
              "(there is no adapter to test here, so this is not a pass)")
        return 0

    from kicad_ipc_adapter import vec_mm, _mm_to_nm, make_zone

    fails = []

    def check(name, ok, detail=''):
        if not ok:
            fails.append(name)
        print(('  PASS ' if ok else '  FAIL ') + name
              + (('  ' + detail) if detail and not ok else ''))

    print('-- vec_mm rounds, on values float cannot hold exactly --')
    for mm in CASES:
        want = round(mm * 1_000_000)
        v = vec_mm(mm, mm)
        check('vec_mm(%s) -> %d nm' % (mm, want),
              v.x == want and v.y == want,
              'got x=%d y=%d (truncation would give %d)'
              % (v.x, v.y, int(mm * 1_000_000)))

    print('-- ...and leaves exactly-representable values alone --')
    for mm in EXACT:
        want = round(mm * 1_000_000)
        v = vec_mm(mm, mm)
        check('vec_mm(%s) -> %d nm' % (mm, want),
              v.x == want and v.y == want, 'got x=%d' % v.x)

    print('-- the truncating constructor is NOT used (positive control) --')
    # Proves the CASES above can actually discriminate: kipy's own helper must
    # disagree with us on at least one of them, or this file is vacuous.
    from kipy.geometry import Vector2
    disagree = [mm for mm in CASES
                if Vector2.from_xy_mm(mm, mm).x != round(mm * 1_000_000)]
    check('kipy.Vector2.from_xy_mm still truncates at least one case',
          bool(disagree),
          'it no longer truncates ANY case -- kipy may have fixed from_mm; '
          'if so this guard is inert and the CASES need harder values')
    print('       (kipy truncates: %s)'
          % ', '.join(str(m) for m in disagree))

    print('-- _mm_to_nm itself --')
    for mm in CASES:
        check('_mm_to_nm(%s)' % mm, _mm_to_nm(mm) == round(mm * 1_000_000),
              'got %d' % _mm_to_nm(mm))

    print('-- make_zone outlines take the same rounding path --')
    # Source-level: building a real Zone needs a live board's net map, so
    # assert the call shape instead. from_xy_mm anywhere in this function is
    # the bug coming back.
    # Read the CODE, not the source text: this function's comment NAMES
    # from_xy_mm to explain why it is avoided, so a substring scan fails on the
    # explanation. (First cut of this file did exactly that -- the same trap
    # that made a docstring-scanning gate pass with its guard deleted.)
    import ast
    import inspect
    import textwrap
    tree = ast.parse(textwrap.dedent(inspect.getsource(make_zone)))
    called = {n.func.attr for n in ast.walk(tree)
              if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)}
    named = {n.id for n in ast.walk(tree) if isinstance(n, ast.Name)}
    check('make_zone does not CALL from_xy_mm',
          'from_xy_mm' not in called,
          'make_zone still builds outline nodes with the truncating helper')
    check('make_zone rounds through _mm_to_nm',
          '_mm_to_nm' in named)

    print()
    if fails:
        print('FAILED (%d): %s' % (len(fails), ', '.join(fails)))
        return 1
    print('ALL PASS -- the IPC adapter rounds mm->nm at every emit site')
    return 0


if __name__ == '__main__':
    sys.exit(main())
