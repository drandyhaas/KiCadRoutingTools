#!/usr/bin/env python3
"""The colour language every render in this repo speaks (#946, #1011).

``frame_layout`` plans WHERE things go; this plans WHAT COLOUR THEY ARE. Same
posture as its neighbour: pure data over stdlib, so a palette can be measured --
contrast, colour-vision separation, compositing collisions -- before anything is
drawn. ``palette_audit`` does that measuring; ``tests/test_946_palette_measures``
re-derives its floors from what is declared here.

**THIS MODULE MUST NEVER IMPORT PIL, AT ANY SCOPE.**
``py_tools/render_placement.py`` imports it at module scope and must still
import with ``sys.modules['PIL'] = None`` --
``tests/test_943_optional_render_dependency.py`` asserts exactly that, because
``board_context.py`` and the stress predictors import that module for
``PlacementModel`` / ``legality_findings`` and draw nothing.

**AND IT MUST CONTAIN NO ``set()``.**
``tests/test_431_render_placement.py:175-195`` requires byte-identical PNGs
across two subprocesses with different ``PYTHONHASHSEED``. A palette assembled
from set iteration order flaps that test intermittently, which is the worst
possible failure mode for a colour change. Ordered tuples and dict literals
only.

WHAT A ROLE IS. A role is A QUESTION THE PICTURE ANSWERS, not a swatch. The
families are load-bearing, because the accident they replace is that **red meant
four things at once**: ripped copper ``(255,66,66)``
(``animate_route.py:39``), a pad/hole conflict ``(255,64,64)``
(``render_placement.py:806`` -- **2.8 apart**, and opposite in kind), a failed
net ``(232,72,72)`` and the ``TRIED`` badge ``(200,60,60)``
(``make_film.py``). And ``place_route_loop``'s default artifact is ONE film over
both halves, so red changed meaning at every phase boundary, several times a
run.

    structure  what the board IS          (body, edge, pad, via)
    event      what JUST HAPPENED to copper on this frame
    defect     what is WRONG and will not fix itself
    place      what MOVED, and from where
    chrome     the frame around the picture -- never the picture
    status     an editorial mark ON a frame (a badge, a record line)
    op         which operator produced an attempt (the awx research vocabulary,
               shared so that a colour cannot mean two things across two films)

EVERY ROLE CARRIES A MARK BESIDE ITS RGB. The mark vocabulary is exactly the one
``render_placement.draw_legend`` already speaks -- ``solid``, ``ring``,
``dashed``, ``hatch``, ``arrow``, ``line`` -- so the key and the draw site agree
BY CONSTRUCTION rather than by a human keeping two lists in step. A second,
non-colour channel is also what makes a distinction survive a greyscale print, a
projector, a compressed GIF, and the next person who adds a fourth event colour;
``render_placement.py:802`` already hatches locked parts for that reason.

THE LAYER PALETTE BELONGS TO A THEME, and that took measuring to establish.
#946 concluded it carries over to a light ground unchanged, because alpha
compositing is a linear blend and therefore preserves the DIFFERENCES between
layers -- closest rendered pair 24.8 dark vs 24.3 light, mean 78.3 vs 78.4, and
that much is right. But it never measured contrast AGAINST the board, which
falls from 1.96x/3.09x to **1.19x/1.55x**: ``_LAYER_PALETTE`` is a light-on-dark
palette, so over a bright board every entry is nearly the board. So a theme owns
its layer palette AND its ``layer_alpha``.

#1011 is VALUE-PRESERVING: ``DARK`` below is byte-for-byte today's constants, so
every existing render is unchanged and the refactor proves itself. The palette
changes are #1012.
"""
from __future__ import annotations

import sys
from typing import Dict, Sequence, Tuple

RGB = Tuple[int, int, int]

#: The non-colour channels a role can ask for. Exactly the vocabulary
#: `render_placement.draw_legend` already draws, so that function ports with no
#: vocabulary change at all.
MARKS: Tuple[str, ...] = ('solid', 'ring', 'dashed', 'hatch', 'arrow', 'line')

#: Every role, in report order. A tuple, never a set: see the docstring.
ROLES: Tuple[str, ...] = (
    # --- structure: what the board IS -------------------------------------
    'ground', 'board_body', 'board_edge', 'zone_tint',
    'pad', 'pad_hole', 'via', 'via_hole', 'hilite',
    # --- event: what JUST HAPPENED to copper on this frame ----------------
    'event_new', 'event_restored', 'event_ripped',
    # --- defect: what is WRONG and will not fix itself --------------------
    'defect_conflict', 'defect_hole', 'defect_courtyard',
    'defect_required_gap', 'defect_net_fail', 'defect_net_block',
    # --- place: what MOVED, and from where --------------------------------
    'place_court_front', 'place_court_back', 'place_court_dim',
    'place_locked', 'place_ghost', 'place_arrow', 'place_label',
    'place_airwire', 'place_net_pick',
    'pad_tht', 'pad_front', 'pad_back',
    # --- the fanout animator's NEAR-COPIES of the four above. Listed apart
    #     rather than merged, because #1011 is value-preserving and the
    #     mapping audit caught the merge: they are 6 to 12 apart, which is a
    #     sixth instance of the drift #946 is about, and #1012 gets to
    #     collapse them ON A MEASUREMENT rather than by assumption.
    'fanout_ground', 'fanout_field_edge', 'fanout_court', 'fanout_court_seed',
    'fanout_label',
    # --- chrome: the frame around the picture, never the picture ----------
    'chrome_panel', 'chrome_panel_edge', 'chrome_strip', 'chrome_band',
    'chrome_text', 'chrome_strip_text', 'chrome_text_dim',
    'chrome_text_faint', 'chrome_rule',
    'chrome_error',
    # --- status: an editorial mark ON a frame ------------------------------
    'status_tried', 'status_best', 'status_kept', 'status_dropped',
    # --- the evolution film's own ground, panel and text, and its own two
    #     event colours. `event_added`/`event_removed` mean exactly what
    #     `event_new`/`event_ripped` mean and are DIFFERENT VALUES -- the
    #     seventh instance. #1013 collapses them; #1011 records them.
    'film_ground', 'film_panel', 'film_text',
    'event_added', 'event_removed',
    # --- op: which operator produced an attempt ----------------------------
    'op_seed', 'op_descend', 'op_jump', 'op_cross',
)


class Theme(object):
    """One complete assignment of ROLES -> (rgb, mark), plus the layers.

    ``__slots__`` and a raising ``rgb()`` on purpose: a bare dict would let a
    typo'd role resolve to ``None`` and paint black at draw time, which is the
    one failure a palette module must not permit.
    """

    __slots__ = ('name', '_rgb', '_mark', 'layers', 'layer_alpha')

    def __init__(self, name: str, colours: Dict[str, RGB],
                 marks: Dict[str, str], layers: Sequence[RGB],
                 layer_alpha: int):
        missing = tuple(r for r in ROLES if r not in colours)
        if missing:
            raise KeyError('theme %r is missing %d role(s): %s'
                           % (name, len(missing), ', '.join(missing)))
        extra = tuple(r for r in colours if r not in ROLES)
        if extra:
            raise KeyError('theme %r declares %d role(s) that are not in '
                           'ROLES: %s' % (name, len(extra), ', '.join(extra)))
        bad = tuple('%s=%s' % (k, v) for k, v in marks.items()
                    if v not in MARKS)
        if bad:
            raise ValueError('theme %r uses marks outside the vocabulary: %s'
                             % (name, ', '.join(bad)))
        self.name = name
        self._rgb = dict(colours)
        self._mark = dict(marks)
        self.layers = tuple(tuple(c) for c in layers)
        self.layer_alpha = int(layer_alpha)

    def rgb(self, role: str) -> RGB:
        try:
            return self._rgb[role]
        except KeyError:
            raise KeyError('no such role %r in theme %r (roles: %s)'
                           % (role, self.name, ', '.join(ROLES)))

    def mark(self, role: str) -> str:
        """The non-colour channel this role draws in. 'solid' when unstated."""
        return self._mark.get(role, 'solid')

    def get(self, role: str, default=None):
        return self._rgb.get(role, default)

    def __repr__(self):
        return '<Theme %r, %d roles, %d layers at alpha %d>' % (
            self.name, len(self._rgb), len(self.layers), self.layer_alpha)


# ---------------------------------------------------------------------------
# DARK -- byte-for-byte what the repo shipped before #946. Every comment names
# the module and constant the value came from, so the port is checkable by
# grep rather than by trust.
# ---------------------------------------------------------------------------

_DARK_LAYERS = (
    (208, 64, 58),    # 0  F.Cu   red
    (70, 130, 210),   # 1  B.Cu   blue
    (96, 190, 96),    # 2  In1    green
    (214, 190, 78),   # 3  In2    yellow
    (196, 110, 206),  # 4  In3    magenta
    (94, 200, 200),   # 5  In4    cyan
    (224, 150, 70),   # 6  In5    orange
    (150, 150, 224),  # 7  In6    periwinkle
    (170, 210, 90),   # 8  In7    lime
    (210, 120, 150),  # 9  In8    pink
)

_DARK = {
    # route_render
    'ground':             (14, 16, 18),      # _BG
    'board_body':         (26, 34, 28),      # _BOARD_FILL
    'board_edge':         (225, 225, 210),   # _EDGE
    'zone_tint':          (120, 120, 120),   # route_render.py:294, inline
    'pad':                (192, 168, 96),    # _PAD
    'pad_hole':           (10, 10, 10),      # _PAD_HOLE
    'via':                (176, 176, 184),   # _VIA
    'via_hole':           (10, 10, 10),      # _VIA_HOLE
    'hilite':             (255, 60, 60),     # _HILITE
    # animate_route
    'event_new':          (250, 250, 250),   # _NEW
    'event_restored':     (86, 224, 96),     # _RESTORE  -- #1013 moves to cyan
    'event_ripped':       (255, 66, 66),     # _RIP
    # render_placement
    'defect_conflict':    (255, 140, 0),     # #1012: was (255,64,64),
                                             # 2.8 from event_ripped
    'defect_hole':        (255, 160, 64),    # C_HOLE
    'defect_courtyard':   (255, 120, 40),    # C_COURT_OVL
    'defect_required_gap': (255, 200, 64),   # a literal in FIVE places, unnamed
    'defect_net_fail':    (214, 96, 24),     # #1012: was (232,72,72)
    'defect_net_block':   (236, 158, 60),    # C_AIR_BLOCK
    'place_court_front':  (150, 152, 168),   # C_COURT_F
    'place_court_back':   (108, 132, 160),   # C_COURT_B
    'place_court_dim':    (58, 60, 70),      # C_COURT_DIM
    'place_locked':       (92, 88, 74),      # C_LOCKED
    'place_ghost':        (76, 76, 92),      # C_GHOST
    'place_arrow':        (236, 214, 110),   # C_ARROW
    'place_label':        (226, 228, 238),   # C_LABEL
    'place_airwire':      (86, 96, 112),     # C_AIR
    'place_net_pick':     (96, 214, 170),    # C_AIR_PICK
    'pad_tht':            (196, 150, 74),    # C_PAD_THT
    'pad_front':          (198, 172, 96),    # C_PAD_F
    'pad_back':           (104, 150, 196),   # C_PAD_B
    # animate_fanout_clearance's own set, 6-12 away from the four above
    'fanout_ground':      (18, 20, 26),      # _PlainCanvas._bg
    'fanout_field_edge':  (70, 78, 96),      # the BGA field outline
    'fanout_court':       (150, 150, 165),   # cf. place_court_front, 4.1 away
    'fanout_court_seed':  (52, 52, 60),      # cf. place_court_dim, 12.8 away
    'fanout_label':       (235, 235, 245),   # cf. place_label, 12.4 away
    # movie_panels / make_film / cmd_timing -- the four near-identical greys
    'chrome_panel':       (14, 14, 18),      # _PANEL_BG, make_film card
    'chrome_panel_edge':  (44, 50, 58),      # evolve_movie.PANEL_EDGE
    'chrome_strip':       (28, 28, 34),      # _STRIP_BG, make_film strip
    'chrome_band':        (0, 0, 0),         # cmd_timing clock band
    # TWO text greys, 17.4 apart, and keeping them apart is a finding rather
    # than an oversight: `route_render._label` draws its HUD ON the picture at
    # (240,240,240) while `movie_panels._STRIP_FG` and `make_film`'s caption
    # draw in a panel strip at (228,228,236). Merging them is a judgement #1012
    # gets to make ON A MEASUREMENT -- #1011 is value-preserving, and the
    # byte-identity gate caught the merge the moment it was attempted.
    'chrome_text':        (240, 240, 240),   # route_render._label, cmd_timing
    'chrome_strip_text':  (228, 228, 236),   # _STRIP_FG, make_film caption
    'chrome_text_dim':    (138, 146, 158),   # evolve_movie.DIM
    'chrome_text_faint':  (78, 84, 94),      # evolve_movie.FAINT
    'chrome_rule':        (42, 50, 44),
    'chrome_error':       (196, 128, 128),   # movie_panels error text
    # editorial marks
    'status_tried':       (160, 78, 20),     # #1012: was (200,60,60) --
                                             # a red badge on a frame whose
                                             # copper also flashes red is
                                             # the same collision
    'status_best':        (255, 214, 88),    # evolve_movie.BEST
    'status_kept':        (86, 206, 130),    # evolve_movie.KEPT
    'status_dropped':     (206, 78, 92),     # evolve_movie.DROPPED
    # the awx operator vocabulary
    # the evolution film's chrome and its own event pair
    'film_ground':        (10, 11, 13),      # evolve_movie.BG, 6.4 from ground
    'film_panel':         (20, 23, 28),      # evolve_movie.PANEL
    'film_text':          (228, 232, 238),   # evolve_movie.TEXT
    'event_added':        (255, 248, 150),   # evolve ADDED  = event_new
    'event_removed':      (255, 70, 120),    # evolve REMOVED = event_ripped
    'op_seed':            (150, 162, 176),
    'op_descend':         (86, 206, 130),
    'op_jump':            (242, 162, 58),
    'op_cross':           (190, 130, 236),
}

#: The second channel, per role. Only the roles that HAVE one are listed;
#: everything else is 'solid'. `place_locked` hatching is the repo's own
#: precedent (`render_placement.py:802`, "hatch so 'locked' reads without a
#: legend"), and it is what #1013 extends to the rip.
_DARK_MARKS = {
    'place_locked': 'hatch',
    'place_ghost': 'dashed',
    'place_arrow': 'arrow',
    'place_airwire': 'line',
    'place_net_pick': 'line',
    'defect_conflict': 'ring',
    'defect_hole': 'ring',
    'defect_required_gap': 'dashed',
    'defect_net_fail': 'line',
    'defect_net_block': 'line',
}

DARK = Theme('dark', _DARK, _DARK_MARKS, _DARK_LAYERS, 150)

# ---------------------------------------------------------------------------
# LIGHT (#1012). NOT a background swap, and not a transform of DARK -- the
# obvious light theme (darken the dark events) lands at 88.6 deuteranope
# separation between ripped and restored, WORSE than the neighbourhood of the
# red/green collision this whole issue is about. At contrast >= 4.5 over a
# light body the usable gamut is the dark half of the cube, where dark-red and
# dark-teal both lose their blue separation.
#
# Three things #946 concluded or never measured, and what measuring found:
#
#   * "the layer palette carries over unchanged" -- TRUE for separation
#     BETWEEN layers (closest rendered pair 24.8 dark vs 24.3 light), FALSE
#     for contrast against the board, which falls 1.96x/3.09x -> 1.19x/1.55x.
#     `_LAYER_PALETTE` is a light-on-dark palette; over a bright board every
#     entry is nearly the board. k = 0.74 at alpha 205 beats dark on all three
#     measures at once, and keeps alpha < 255 so the crossing blend survives.
#   * the STRUCTURE tokens were never measured at all. `board_edge` scores
#     12.33x on the dark body and 1.01x on a light one -- the outline vanishes,
#     and with the body only 1.17x off the ground a light frame has no board.
#   * `pad_hole` is the one genuinely THEME-INVARIANT token, 1.22x -> 15.21x.
#     A hole is a hole. Recorded so nobody later "fixes" it.
#
# The 27 decorative roles were derived by compressing each family's own
# lightness ORDERING into the band that clears the floor -- three simpler rules
# failed first, each destroying separation in its own way; the derivation
# script records all three.
# ---------------------------------------------------------------------------
_LIGHT_LAYERS = (
    (154, 47, 43),     # F.Cu  dark x 0.74
    (52, 96, 155),     # B.Cu  dark x 0.74
    (71, 141, 71),     # In1   dark x 0.74
    (158, 141, 58),    # In2   dark x 0.74
    (145, 81, 152),    # In3   dark x 0.74
    (70, 148, 148),    # In4   dark x 0.74
    (166, 111, 52),    # In5   dark x 0.74
    (111, 111, 166),   # In6   dark x 0.74
    (126, 155, 67),    # In7   dark x 0.74
    (155, 89, 111),    # In8   dark x 0.74
)

_LIGHT = {
    # --- structure
    'ground':                (236, 238, 232),
    'board_body':            (223, 227, 218),
    'board_edge':            (78, 86, 72),
    'zone_tint':             (110, 112, 106),
    'pad':                   (150, 118, 24),
    'pad_hole':              (10, 10, 10),
    'via':                   (96, 100, 104),
    'via_hole':              (10, 10, 10),
    'hilite':                (190, 24, 38),
    # --- event
    'event_new':             (24, 26, 20),
    'event_restored':        (0, 60, 150),
    'event_ripped':          (190, 24, 38),
    # --- defect
    'defect_conflict':       (204, 90, 0),
    'defect_hole':           (199, 100, 0),
    'defect_courtyard':      (204, 76, 0),
    'defect_required_gap':   (168, 120, 0),
    'defect_net_fail':       (170, 70, 12),
    'defect_net_block':      (189, 107, 5),
    # --- place
    'place_court_front':     (62, 64, 80),
    'place_court_back':      (61, 80, 101),
    'place_court_dim':       (113, 115, 125),
    'place_locked':          (115, 111, 97),
    'place_ghost':           (102, 102, 118),
    'place_arrow':           (115, 96, 7),
    'place_label':           (26, 28, 38),
    'place_airwire':         (87, 97, 113),
    'place_net_pick':        (23, 118, 82),
    'pad_tht':               (162, 117, 42),
    'pad_front':             (153, 126, 45),
    'pad_back':              (51, 102, 153),
    # --- fanout
    'fanout_ground':         (238, 240, 234),
    'fanout_field_edge':     (93, 101, 119),
    'fanout_court':          (64, 64, 79),
    'fanout_court_seed':     (115, 115, 123),
    'fanout_label':          (27, 27, 37),
    # --- chrome
    'chrome_panel':          (245, 246, 241),
    'chrome_panel_edge':     (178, 185, 170),
    'chrome_strip':          (232, 234, 227),
    'chrome_band':           (245, 246, 241),
    'chrome_text':           (20, 24, 15),
    'chrome_strip_text':     (30, 34, 25),
    'chrome_text_dim':       (78, 86, 72),
    'chrome_text_faint':     (125, 133, 118),
    'chrome_rule':           (205, 210, 198),
    'chrome_error':          (168, 40, 40),
    # --- status
    'status_tried':          (128, 58, 8),
    'status_best':           (158, 119, 0),
    'status_kept':           (33, 146, 74),
    'status_dropped':        (168, 36, 50),
    # --- film
    'film_ground':           (241, 243, 237),
    'film_panel':            (232, 234, 228),
    'film_text':             (24, 28, 20),
    'event_added':           (120, 104, 0),
    'event_removed':         (176, 0, 60),
    # --- op
    'op_seed':               (89, 101, 115),
    'op_descend':            (33, 146, 74),
    'op_jump':               (189, 107, 0),
    'op_cross':              (113, 16, 188),
}


#: Marks are a property of the ROLE, not of the theme: a rip is dashed in both.
LIGHT = Theme('light', _LIGHT, _DARK_MARKS, _LIGHT_LAYERS, 205)



#: The themes this repo ships. A CLOSED mapping, deliberately: the contrast
#: gate's whole value is that it can iterate EVERY theme, and an unmeasured
#: theme is worse than no theme.
THEMES: Dict[str, Theme] = {'dark': DARK, 'light': LIGHT}

DEFAULT_THEME_NAME = 'dark'


def theme(name=None, *, strict=True) -> Theme:
    """Resolve a theme by name, or a Theme straight through.

    `strict` is the asymmetry `make_movie._panels_wanted` already established
    and this copies deliberately: an unknown value passed IN CODE raises,
    naming the accepted set, because a typo in code is a bug and silently
    rendering the wrong movie hides it. An unknown value arriving from the
    ENVIRONMENT warns and falls back, because a typo in a shell must not abort
    a routing run that happened to ask for a movie.
    """
    if name is None:
        return default_theme()
    if isinstance(name, Theme):
        return name
    key = str(name).strip().lower()
    if key in THEMES:
        return THEMES[key]
    if strict:
        raise ValueError('render_theme: %r is not one of %s'
                         % (name, ', '.join(sorted(THEMES))))
    print('render_theme: %r is not one of %s; using %s'
          % (name, ', '.join(sorted(THEMES)), DEFAULT_THEME_NAME),
          file=sys.stderr)
    return THEMES[DEFAULT_THEME_NAME]


def default_theme() -> Theme:
    """The theme the environment asks for.

    Re-read on every call rather than frozen at import: a test that renders
    both themes in one process would otherwise have to mutate global state.
    It is a dict lookup.
    """
    name = DEFAULT_THEME_NAME
    try:
        import env_knobs
        name = getattr(env_knobs, 'RENDER_THEME', DEFAULT_THEME_NAME)
    except Exception:                                          # noqa: BLE001
        pass
    return theme(name, strict=False)


def layer_palette(copper_layers: Sequence[str],
                  th: Theme = None) -> Dict[str, RGB]:
    """Map copper layer names to colours: F.Cu first, B.Cu second, inners in
    order, wrapping modulo the palette.

    Moved here from `route_render` unchanged, so that a theme can supply a
    different set of ten without every caller learning about it.
    """
    th = DARK if th is None else th
    pal = th.layers
    out: Dict[str, RGB] = {}
    inner = 2
    for name in copper_layers:
        if name == 'F.Cu':
            out[name] = pal[0]
        elif name == 'B.Cu':
            out[name] = pal[1]
        else:
            out[name] = pal[inner % len(pal)]
            inner += 1
    return out


def audit_lines(th: Theme = None) -> list:
    """The theme as a table, for a console or a commit message. Deliberately
    not a `__main__`: this is a leaf library, and a runnable module would owe a
    `KRT_TOOL` declaration. `palette_card.py` draws it instead."""
    th = DARK if th is None else th
    out = ['theme %s -- %d roles, %d layers at alpha %d'
           % (th.name, len(ROLES), len(th.layers), th.layer_alpha)]
    for role in ROLES:
        out.append('  %-22s %-16s %s'
                   % (role, str(th.rgb(role)), th.mark(role)))
    return out
