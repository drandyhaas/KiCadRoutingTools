#!/usr/bin/env python3
"""A theme is COMPLETE, DETERMINISTIC and PIL-FREE (#946, #1011).

Three properties, each with a specific way of failing that nothing else in the
suite would catch.

**COMPLETE.** A role that resolves to `None` paints black at draw time and
looks like a rendering bug rather than a missing declaration. `Theme.__init__`
refuses both directions -- a missing role, and a role that is not in `ROLES` --
so this file mostly proves the refusal is armed on every shipped theme.

**VALUE-PRESERVING at #1011; three values moved at #1012.** `DARK` was
byte-for-byte the constants the repo shipped before the design system, and
#1012 moved exactly three of them -- `defect_conflict`, `defect_net_fail` and
`status_tried` -- out of the red family, each justified by a measurement in
`tests/test_946_palette_measures.py`. The table below is the whole mapping as
literals, because that is the one form a reviewer can check against
`git show upstream/main:<file>` without running anything. **A phase that
changes a value edits this table in the same commit**, and
`tests/test_946_palette_measures.py` is what says whether the new value is
allowed.

The table is not decoration: writing it caught **four real colour changes** the
byte-identity harness could not see, because that harness renders
`route_render` and never renders the fanout animation, `make_film`'s cards or
`evolve_movie`. Four of `animate_fanout_clearance`'s colours had been folded
onto `render_placement`'s near-identical ones -- 4 to 13 apart -- and would have
shipped silently, since `tests/test_431_animator_port.py` asserts two renders
match EACH OTHER, not that they match what they used to be.

**DETERMINISTIC.** `tests/test_431_render_placement.py:175-195` requires
byte-identical PNGs across two `PYTHONHASHSEED` values. A palette built from set
iteration order flaps that test intermittently -- the worst possible failure
mode for a colour change -- so this guards it at the data level, ~200x faster,
and names the cause when it fires.

**PIL-FREE.** `py_tools/render_placement.py` imports `render_theme` at module
scope and must keep importing with `sys.modules['PIL'] = None`.
"""
import os
import subprocess
import sys

RUN_ALL_FAST_OK = True

_TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(_TESTS)
for _p in (ROOT, _TESTS, os.path.join(ROOT, 'py_router'),
           os.path.join(ROOT, 'py_tools')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import render_theme as RT  # noqa: E402

#: The whole DARK mapping, as literals. See the docstring: this is the form a
#: reviewer can check against the merge-base by eye.
DARK_AT_1012 = {
    'ground': (14, 16, 18),
    'board_body': (26, 34, 28),
    'board_edge': (225, 225, 210),
    'zone_tint': (120, 120, 120),
    'pad': (192, 168, 96),
    'pad_hole': (10, 10, 10),
    'via': (176, 176, 184),
    'via_hole': (10, 10, 10),
    'hilite': (255, 60, 60),
    'event_new': (250, 250, 250),
    'event_restored': (80, 215, 230),
    'event_ripped': (255, 66, 66),
    'defect_conflict': (255, 140, 0),
    'defect_hole': (255, 160, 64),
    'defect_courtyard': (255, 120, 40),
    'defect_required_gap': (255, 200, 64),
    'defect_net_fail': (214, 96, 24),
    'defect_net_block': (236, 158, 60),
    'place_court_front': (150, 152, 168),
    'place_court_back': (108, 132, 160),
    'place_court_dim': (58, 60, 70),
    'place_locked': (92, 88, 74),
    'place_ghost': (76, 76, 92),
    'place_arrow': (236, 214, 110),
    'place_label': (226, 228, 238),
    'place_airwire': (86, 96, 112),
    'place_net_pick': (96, 214, 170),
    'pad_tht': (196, 150, 74),
    'pad_front': (198, 172, 96),
    'pad_back': (104, 150, 196),
    'fanout_ground': (18, 20, 26),
    'fanout_field_edge': (70, 78, 96),
    'fanout_court': (150, 150, 165),
    'fanout_court_seed': (52, 52, 60),
    'fanout_label': (235, 235, 245),
    'chrome_panel': (14, 14, 18),
    'chrome_panel_edge': (44, 50, 58),
    'chrome_strip': (28, 28, 34),
    'chrome_band': (0, 0, 0),
    'chrome_text': (240, 240, 240),
    'chrome_strip_text': (228, 228, 236),
    'chrome_text_dim': (138, 146, 158),
    'chrome_text_faint': (78, 84, 94),
    'chrome_rule': (42, 50, 44),
    'chrome_error': (196, 128, 128),
    'status_tried': (160, 78, 20),
    'status_best': (255, 214, 88),
    'status_kept': (86, 206, 130),
    'status_dropped': (206, 78, 92),
    'film_ground': (10, 11, 13),
    'film_panel': (20, 23, 28),
    'film_text': (228, 232, 238),
    'event_added': (255, 248, 150),
    'event_removed': (255, 70, 120),
    'op_seed': (150, 162, 176),
    'op_descend': (86, 206, 130),
    'op_jump': (242, 162, 58),
    'op_cross': (190, 130, 236),
}

_FAIL = []


def fail(msg):
    _FAIL.append(msg)
    print('  FAIL: %s' % msg)


def test_dark_is_value_preserving():
    """#1011 changes no colour. Every value here came from a shipped constant,
    and the comment beside it in render_theme.py names which."""
    missing = [r for r in DARK_AT_1012 if r not in RT.ROLES]
    if missing:
        fail('table names %d role(s) ROLES does not: %s'
             % (len(missing), ', '.join(sorted(missing))))
    untabled = [r for r in RT.ROLES if r not in DARK_AT_1012]
    if untabled:
        fail('%d role(s) have no expected value -- an untabled role is an '
             'unreviewed one: %s' % (len(untabled), ', '.join(untabled)))
    for role, want in DARK_AT_1012.items():
        if role not in RT.ROLES:
            continue
        got = RT.DARK.rgb(role)
        if tuple(got) != tuple(want):
            fail('VALUE MOVED: %s %s -> %s' % (role, want, got))
    if not _FAIL:
        print('  PASS: all %d DARK roles hold the value they replaced'
              % len(RT.ROLES))


def test_every_theme_is_complete_in_both_directions():
    for name, th in sorted(RT.THEMES.items()):
        for role in RT.ROLES:
            try:
                rgb = th.rgb(role)
            except KeyError as exc:
                fail('theme %r cannot resolve %r (%s)' % (name, role, exc))
                continue
            if (not isinstance(rgb, tuple) or len(rgb) != 3
                    or not all(isinstance(c, int) and 0 <= c <= 255
                               for c in rgb)):
                fail('theme %r role %r is not an RGB triple: %r'
                     % (name, role, rgb))
            if th.mark(role) not in RT.MARKS:
                fail('theme %r role %r has mark %r, outside %s'
                     % (name, role, th.mark(role), RT.MARKS))
        if len(th.layers) != 10:
            fail('theme %r has %d layers, expected 10'
                 % (name, len(th.layers)))
        if not 1 <= th.layer_alpha <= 255:
            fail('theme %r layer_alpha %r out of range'
                 % (name, th.layer_alpha))
    if len(set(RT.ROLES)) != len(RT.ROLES):
        dupes = sorted(r for r in set(RT.ROLES) if RT.ROLES.count(r) > 1)
        fail('ROLES has duplicates: %s' % ', '.join(dupes))
    if not _FAIL:
        print('  PASS: %d theme(s) x %d roles, all resolvable and well-formed'
              % (len(RT.THEMES), len(RT.ROLES)))


def test_the_refusals_are_armed():
    """A gate that cannot refuse is not a gate. Build three malformed themes
    and require each to be rejected FOR ITS OWN REASON."""
    good = dict((r, (1, 2, 3)) for r in RT.ROLES)
    cases = (
        ('a missing role',
         lambda: RT.Theme('x', dict((k, v) for k, v in good.items()
                                    if k != RT.ROLES[0]), {},
                          RT.DARK.layers, 150), KeyError),
        ('an unknown role',
         lambda: RT.Theme('x', dict(good, not_a_role=(1, 2, 3)), {},
                          RT.DARK.layers, 150), KeyError),
        ('a mark outside the vocabulary',
         lambda: RT.Theme('x', good, {RT.ROLES[0]: 'sparkle'},
                          RT.DARK.layers, 150), ValueError),
    )
    for label, build, want in cases:
        try:
            build()
        except want:
            continue
        except Exception as exc:                                # noqa: BLE001
            fail('%s raised %s, not %s'
                 % (label, type(exc).__name__, want.__name__))
            continue
        fail('%s was ACCEPTED -- the refusal is not armed' % label)
    try:
        RT.theme('chartreuse')
        fail('theme("chartreuse") was accepted in strict mode')
    except ValueError:
        pass
    if RT.theme('chartreuse', strict=False) is not RT.THEMES['dark']:
        fail('lenient resolution did not fall back to dark')
    if not _FAIL:
        print('  PASS: 5 refusals armed (3 malformed themes, 2 name modes)')


def _subprocess_palette(seed):
    env = dict(os.environ)
    env['PYTHONHASHSEED'] = seed
    env['PYTHONPATH'] = os.pathsep.join(
        [os.path.join(ROOT, 'py_router'), env.get('PYTHONPATH', '')])
    code = ('import json,sys,render_theme as t;'
            'json.dump([[n,[list(h.rgb(x)) for x in t.ROLES],'
            '[list(c) for c in h.layers],h.layer_alpha]'
            ' for n,h in sorted(t.THEMES.items())], sys.stdout)')
    return subprocess.run([sys.executable, '-X', 'utf8', '-c', code],
                          capture_output=True, text=True, encoding='utf-8',
                          errors='replace', env=env, cwd=ROOT)


def test_the_palette_is_deterministic_across_hash_seeds():
    out = []
    for seed in ('0', '12345'):
        r = _subprocess_palette(seed)
        if r.returncode != 0:
            fail('render_theme at PYTHONHASHSEED=%s exited %d: %s'
                 % (seed, r.returncode, (r.stderr or '')[-400:]))
            return
        out.append(r.stdout)
    if not out[0]:
        fail('BROKEN: the subprocess produced nothing to compare')
        return
    if out[0] != out[1]:
        fail('the palette differs by PYTHONHASHSEED -- a set() or an '
             'unordered comprehension is in a palette path')
        return
    print('  PASS: identical across PYTHONHASHSEED 0 and 12345')


def test_it_imports_without_pillow():
    """render_placement imports this at module scope and must keep importing
    with PIL absent -- test_943 asserts that, and this is the half of the rule
    that lives with the module it constrains."""
    env = dict(os.environ)
    env['PYTHONPATH'] = os.pathsep.join(
        [os.path.join(ROOT, 'py_router'), os.path.join(ROOT, 'py_tools'),
         env.get('PYTHONPATH', '')])
    code = ("import sys; sys.modules['PIL'] = None;"
            "import render_theme, render_placement;"
            "assert render_placement.PlacementModel is not None;"
            "assert render_placement.legality_findings is not None;"
            "print(render_theme.DARK.name)")
    r = subprocess.run([sys.executable, '-X', 'utf8', '-c', code],
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', env=env, cwd=ROOT)
    if r.returncode != 0 or 'dark' not in (r.stdout or ''):
        fail('import with PIL blocked failed (%d): %s'
             % (r.returncode, ((r.stderr or '') + (r.stdout or ''))[-500:]))
        return
    print('  PASS: render_theme and render_placement import with PIL blocked')


def test_layer_palette_matches_route_renders_assignment():
    """The mapping moved modules; it must not have moved behaviour."""
    import route_render as RR
    stacks = (['F.Cu', 'B.Cu'],
              ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu'],
              ['F.Cu'] + ['In%d.Cu' % i for i in range(1, 13)] + ['B.Cu'])
    for st in stacks:
        a, b = RR.layer_palette(st), RT.layer_palette(st)
        if a != b:
            fail('layer_palette differs on %d layers: %s vs %s'
                 % (len(st), a, b))
            return
        if a['F.Cu'] != RT.DARK.layers[0] or a.get('B.Cu') != RT.DARK.layers[1]:
            fail('F.Cu/B.Cu are not slots 0 and 1 on a %d-layer stack'
                 % len(st))
            return
    print('  PASS: layer assignment identical on 3 stackups, F.Cu/B.Cu pinned')


def test_the_film_chrome_reads_the_active_theme():
    """#946/C4: the last DARK literals in the film path are gone.

    `make_film._card_frame` and `_badge`, the iso panel (`iso_panel`,
    `stack`) and the run clock (`cmd_timing.add_clock_band`) each drew in
    DARK whatever `--theme` said, so a light film carried dark cards, a dark
    iso slab and a dark clock band. Asserted by PIXEL on each, under LIGHT --
    and by source for `make_film`, whose module must not bind DARK at all."""
    _mark = len(_FAIL)
    try:
        from PIL import Image
    except ImportError:
        print('  SKIP: needs Pillow for the pixel half')
        return
    import cmd_timing
    import make_film
    import movie_panels
    L, D = RT.theme('light'), RT.theme('dark')
    card = make_film._card_frame((200, 120), None, 'x', theme='light')
    if card.getpixel((5, 5)) != L.rgb('chrome_panel'):
        fail('a light card is %r, not LIGHT chrome_panel %r'
             % (card.getpixel((5, 5)), L.rgb('chrome_panel')))
    fr = Image.new('RGB', (80, 60), (0, 0, 0))
    make_film._badge(fr, 'TRIED', theme='light')
    if fr.getpixel((0, 0)) != L.rgb('status_tried'):
        fail('a light badge is %r, not LIGHT status_tried %r'
             % (fr.getpixel((0, 0)), L.rgb('status_tried')))
    pan, _err = movie_panels.iso_panel((160, 100), None, 'cap',
                                       theme='light')
    if pan.getpixel((5, 5)) != L.rgb('chrome_panel'):
        fail('a light iso panel ground is %r' % (pan.getpixel((5, 5)),))
    if pan.getpixel((5, 98)) != L.rgb('chrome_strip'):
        fail('a light iso caption strip is %r' % (pan.getpixel((5, 98)),))
    st = movie_panels.stack(Image.new('RGB', (40, 10)),
                            Image.new('RGB', (40, 10)), theme='light')
    if st.size != (40, 20):
        fail('stack changed shape: %r' % (st.size,))
    band = cmd_timing.add_clock_band(Image.new('RGB', (120, 40)),
                                     ['t 0:01'], 30, theme='light')
    if band.getpixel((119, 69)) != L.rgb('chrome_band'):
        fail('a light clock band is %r, not LIGHT chrome_band %r'
             % (band.getpixel((119, 69)), L.rgb('chrome_band')))
    # the dark default is unchanged
    if make_film._card_frame((50, 40), None, '').getpixel((2, 2)) \
            != D.rgb('chrome_panel') and RT.default_theme().name == 'dark':
        fail('the default card is no longer DARK chrome_panel')
    src = open(make_film.__file__, encoding='utf-8').read()
    if 'DARK as _TH' in src or 'import DARK' in src:
        fail('make_film still binds DARK')
    if len(_FAIL) == _mark:
        print('  PASS: cards, badges, iso panel and clock band draw in the '
              'active theme')


TESTS = (
    test_the_film_chrome_reads_the_active_theme,
    test_dark_is_value_preserving,
    test_every_theme_is_complete_in_both_directions,
    test_the_refusals_are_armed,
    test_the_palette_is_deterministic_across_hash_seeds,
    test_it_imports_without_pillow,
    test_layer_palette_matches_route_renders_assignment,
)


def main():
    for fn in TESTS:
        print('%s:' % fn.__name__)
        fn()
    if _FAIL:
        print('')
        print('%d FAILURE(S)' % len(_FAIL))
        for m in _FAIL:
            print('  - %s' % m)
        return 1
    print('')
    print('all %d checks passed' % len(TESTS))
    return 0


if __name__ == '__main__':
    sys.exit(main())
