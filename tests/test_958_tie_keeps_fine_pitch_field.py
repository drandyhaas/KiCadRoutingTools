"""#958's equal-length tie-break does not rearrange copper in a fine-pitch field.

`smooth_octolinear_chains` phase 2 (#958) accepts a connector of EQUAL length
when it emits strictly fewer legs. Straightening does not ADD copper -- it MOVES
it: a staircase hugs its own corner, and the diagonal that replaces it cuts
across, sweeping through the space the steps left open. Between 0.5mm-pitch pads
that space is the escape corridor for a pad that still needs one, and BOTH
variants are DRC-legal, so `clears()` cannot choose between them. A tie buys one
fewer leg; it must not buy it there.

MEASURED on ft2232h_jtag, `+1V8` under U4 -- 3.8728mm either way, 3 legs vs 2:

    3 legs   (129.000,103.500)-(128.400,102.900)
             (128.400,102.900)-(125.800,102.900)
             (125.800,102.900)-(125.500,102.600)
    2 legs   (129.000,103.500)-(128.100,102.600)
             (128.100,102.600)-(125.500,102.600)

The run drops from y=102.900 to y=102.600 -- 0.15mm from where /OSCI's #666
bare-ball escape drops its via. The dogbone failed, the rescue failed, and /OSCI
shipped in two pieces.

WHY PITCH AND NOT THE PACKAGE NAME. `find_components_by_type('BGA')` is the
obvious reuse and is the WRONG set: ft2232h_jtag's U4 -- the part this fix
exists for -- is `Package_QFP:LQFP-64_10x10mm_P0.5mm`, so `detect_package_type`
calls it QFP and the BGA filter returns nothing. (Its log line "BGA Grid
Analysis for U4" is the FANOUT's grid analyser, which runs on any candidate and
finds a pitch in a QFP's peripheral rows.) What makes the space a corridor is
the pitch, so a 0.5mm QFP, a 0.65mm BGA and a 0.2mm QFN all qualify. The region
is built from `detect_bga_pitch` + `get_footprint_bounds`, the two helpers
`auto_detect_bga_exclusion_zones` is itself made of.

WHY A STATIC REGION AND NOT "PADS THAT LACK COPPER". That set changes on every
route pass, so a guard keyed on it fires differently each lap. An earlier cut
did exactly that and was REVERTED (549f16b5): it fixed ft2232h_jtag and took
ottercast_audio from 2 to 4 nets incomplete across its five-pass chain. A
package does not move between passes.

    board          v0.22.0   HEAD   waiting-pad cut   THIS
    ft2232h_jtag      0/34   1/34              0/34   0/34
    ottercast        0/158  2/158             4/158  2/158

One improved, one unchanged, none regressed. The cost is real and recorded:
ft2232h_jtag's smoothing goes 46 -> 39 spans (pre-#958 was 27), so #958 still
nets +12 spans there.
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'py_router'))

FAILS = []


def check(name, cond, detail=''):
    print(f"--- {name}")
    if cond:
        print(f"  PASS{': ' + detail if detail else ''}")
    else:
        print(f"  FAIL: {detail}")
        FAILS.append(name)


def _channels(row):
    """The guard's threshold, in ROUTING CHANNELS.

    It used to be a fixed `pitch <= 0.8mm`. That number was only ever a round
    value above the four parts below (0.2/0.31/0.5/0.65) -- no 0.8mm part
    appears in its rationale -- and it is the commonest BGA pitch in the
    corpus, so cparti_fpga U3, icepi_zero U11 and zynq_ad9364 U1/U5 computed
    0.8000000000000114 / 0.8001 / 0.800092 and a bare `>` put every one of them
    on the COARSE side by float noise.
    """
    try:
        from pcb_modification import _SMOOTH_DENSE_CHANNELS
        return _SMOOTH_DENSE_CHANNELS
    except ImportError:
        check(row, False,
              'pcb_modification exports no _SMOOTH_DENSE_CHANNELS -- the '
              'equal-length tie-break is not protecting fine-pitch fields, '
              'because there is no guard to do it')
        return None


class _FakePad:
    def __init__(self, x, y, size):
        self.global_x, self.global_y = x, y
        self.size_x = self.size_y = size
        self.pad_type = 'smd'


class _FakeFp:
    """A square lattice at `pitch` with square pads of `pad` -- the only two
    numbers the decision reads, plus the >=16-pad population it requires."""
    def __init__(self, pitch, pad, n=5):
        self.pads = [_FakePad(i * pitch, j * pitch, pad)
                     for i in range(n) for j in range(n)]


def _is_dense(pitch, pad, track_width, clearance, k=None):
    """Ask the REAL decision, not a copy of its arithmetic.

    This used to reimplement the formula, which made every behavioural row
    below self-confirming: swapping the engine back to the old fixed-pitch rule
    left them all green and only the source-text row failed. A gate that
    restates the code it is checking cannot detect the code changing.
    """
    from pcb_modification import pad_field_is_corridor
    fp = _FakeFp(pitch, pad)
    if k is None:
        return pad_field_is_corridor(fp, track_width, clearance)
    return pad_field_is_corridor(fp, track_width, clearance, channels=k)


def _fn_body(fname):
    import ast
    src = open(os.path.join(os.path.dirname(__file__), '..',
                            'py_router', 'pcb_modification.py')).read()
    lines = src.splitlines()
    for fn in ast.walk(ast.parse(src)):
        if isinstance(fn, ast.FunctionDef) and fn.name == fname:
            return "\n".join(lines[fn.lineno - 1:getattr(fn, 'end_lineno', fn.lineno)])
    return None


#: MEASURED (pitch, median pad, board track_width, board clearance) for the
#: parts this guard must decide, read off the corpus boards. Pitch alone cannot
#: separate them -- bitaxe_ultra's 1.27mm U12 leaves 0.370mm between its pads
#: while zynq_ad9364's 0.8mm U1 leaves 0.450mm -- which is why the rule is
#: stated in channels the board's own geometry defines.
MEASURED = [
    # Each part at ITS OWN board's channel. None of these boards declares a
    # netclass, so every one falls back to routing_defaults (0.3 track /
    # 0.25 clearance) -> chan 0.800. An earlier cut of this table carried
    # per-board channels that were GUESSED (0.35 / 0.404 / 0.55), which put
    # the rows in a different order entirely and made a threshold look
    # derivable that was not -- read the board, do not assume the floors.
    # board/part                  pitch    pad  track  clr   must be dense?
    ('ft2232h U4 (0.5 QFP)',      0.500, 0.300, 0.30, 0.250, True),
    ('ottercast U3 (0.4 QFN)',    0.400, 0.200, 0.30, 0.250, True),
    ('cparti U3 (0.8)',           0.800, 0.550, 0.30, 0.250, True),
    ('icepi U11 (0.8)',           0.800, 0.458, 0.30, 0.250, True),
    ('ottercast U1 (0.65 BGA)',   0.650, 0.300, 0.30, 0.250, True),
    ('zynq U1 (0.8, 400-ball)',   0.800, 0.350, 0.30, 0.250, True),
    # --- and the coarse side, which a wider threshold would pull in ---
    ('glasgow J5 (1.27)',         1.270, 0.760, 0.30, 0.250, False),
    ('cparti U1 (1.0 BGA)',       1.000, 0.400, 0.30, 0.250, False),
    ('splitflap U1 (1.27)',       1.270, 0.600, 0.30, 0.250, False),
]


def t_the_threshold_admits_the_packages_that_matter():
    """Every measured fine-pitch part is a corridor; a coarse one is not.

    The 0.8mm rows are the ones the fixed threshold got wrong: they are the
    commonest BGA pitch in the corpus and they computed a hair ABOVE 0.8, so a
    bare `>` skipped exactly the packages the guard exists for.
    """
    k = _channels('t_the_threshold_admits_the_packages_that_matter')
    if k is None:
        return
    wrong = [n for (n, p, pad, tw, clr, want) in MEASURED
             if _is_dense(p, pad, tw, clr) != want]
    check('t_the_threshold_admits_the_packages_that_matter',
          not wrong, f'all {len(MEASURED)} measured parts classify as '
                     f'measured (wrong: {wrong})')
    check('t_the_threshold_is_not_unbounded',
          k <= 2.0,
          f'{k} channels keeps ordinary coarse-pitch parts out of the guard')


def t_the_rule_is_derived_not_a_fixed_pitch():
    """The guard must read the board's geometry, not a magic millimetre.

    A fixed pitch cannot express "the gap admits one track": the same pitch is
    a corridor on a 0.2mm-clearance board and open room on a 0.1mm one. Asked
    of the SOURCE, because the old constant's name is what a reader greps for.
    """
    src = open(os.path.join(os.path.dirname(__file__), '..',
                            'py_router', 'pcb_modification.py')).read()
    check('t_the_rule_is_derived_not_a_fixed_pitch',
          '_SMOOTH_DENSE_PITCH_MM' not in src,
          'the fixed-pitch constant is gone')
    body = _fn_body('smooth_octolinear_chains')
    check('t_the_rule_reads_track_and_clearance',
          body is not None and 'track_width' in body and 'clearance' in body,
          'the dense-field test is built from track_width + clearance')


def t_a_tight_coarse_pitch_part_is_still_a_corridor():
    """The point of measuring the GAP: bitaxe_ultra's U12 is a 1.27mm part, but
    0.9mm pads leave only 0.370mm between them -- narrower than the 0.8mm BGAs
    this guard protects. Under the old pitch rule it was coarse; under the gap
    rule it is a corridor, which is what the space actually is."""
    k = _channels('t_a_tight_coarse_pitch_part_is_still_a_corridor')
    if k is None:
        return
    check('t_a_tight_coarse_pitch_part_is_still_a_corridor',
          _is_dense(1.270, 0.900, 0.30, 0.250),
          'a 1.27mm part with 0.370mm between pads counts as a corridor')


def t_the_region_comes_from_the_boards_own_helpers():
    """Reuse, not a re-derivation: pitch and bounds are the shared helpers."""
    body = _fn_body('smooth_octolinear_chains')
    _corridor = _fn_body('pad_field_is_corridor') or ''
    check('t_the_region_comes_from_the_boards_own_helpers',
          body is not None and 'get_footprint_bounds' in body
          and 'pad_field_is_corridor' in body
          and 'detect_bga_pitch' in _corridor,
          'built from detect_bga_pitch + get_footprint_bounds')
    # ...and NOT from the BGA-only component filter, which misses U4. Asked of
    # the AST: the name appears in a comment explaining exactly this, so a
    # substring test would read that comment as the defect it warns about.
    import ast
    src = open(os.path.join(os.path.dirname(__file__), '..',
                            'py_router', 'pcb_modification.py')).read()
    called = set()
    for fn in ast.walk(ast.parse(src)):
        if isinstance(fn, ast.FunctionDef) and fn.name == 'smooth_octolinear_chains':
            for node in ast.walk(fn):
                if isinstance(node, ast.Call) and isinstance(node.func, ast.Name):
                    called.add(node.func.id)
    check('t_it_does_not_key_on_the_bga_only_filter',
          'find_components_by_type' not in called,
          "not keyed on find_components_by_type('BGA') -- ft2232h_jtag's U4 is "
          "a QFP and would be missed")
    # The pitch helper now sits one level down, inside pad_field_is_corridor
    # -- the decision was extracted so the gate can call it directly instead of
    # restating its arithmetic. Follow it there rather than loosening the row.
    for _fn2 in ast.walk(ast.parse(src)):
        if (isinstance(_fn2, ast.FunctionDef)
                and _fn2.name == 'pad_field_is_corridor'):
            for _n2 in ast.walk(_fn2):
                if isinstance(_n2, ast.Call) and isinstance(_n2.func, ast.Name):
                    called.add(_n2.func.id)
                if isinstance(_n2, ast.ImportFrom):
                    called.update(a.name for a in _n2.names)
    check('t_it_calls_the_pitch_and_bounds_helpers',
          {'detect_bga_pitch', 'get_footprint_bounds'} <= called,
          f'calls {sorted({"detect_bga_pitch", "get_footprint_bounds"} & called)}')


def t_the_measured_geometry_is_decided_correctly():
    """The ft2232h_jtag case as numbers: same length, and the loser goes low."""
    old = [(129.000, 103.500), (128.400, 102.900),
           (125.800, 102.900), (125.500, 102.600)]
    bad = [(129.000, 103.500), (128.100, 102.600), (125.500, 102.600)]

    def length(pts):
        return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))
    check('t_the_variants_are_the_same_length',
          abs(length(bad) - length(old)) < 1e-9,
          f'{length(old):.4f}mm both -- a TIE, the only case this guard touches')
    # U4's pad-bounds box, measured: X[118.05,130.95] Y[100.55,113.45].
    bx0, by0, bx1, by1 = 118.05, 100.55, 130.95, 113.45

    def inside(pts):
        return any(min(a[0], b[0]) <= bx1 and max(a[0], b[0]) >= bx0
                   and min(a[1], b[1]) <= by1 and max(a[1], b[1]) >= by0
                   for a, b in zip(pts, pts[1:]))
    check('t_the_losing_collapse_lies_in_the_field',
          inside(bad), 'the 2-leg variant crosses U4\'s pad field, so the '
                       'guard refuses it')


def t_the_guard_is_scoped_to_ties_and_cheap():
    """Wiring and cost: ties only, after the memoised clearance test."""
    body = _fn_body('smooth_octolinear_chains')
    if body is None:
        check('t_the_guard_is_scoped_to_ties_and_cheap', False,
              'smooth_octolinear_chains not found')
        return
    check('t_the_guard_only_touches_ties',
          '_is_tie and _dense_boxes' in body,
          'gated on _is_tie, so strictly-shorter collapses are untouched '
          'inside the field as well')
    # COST: the guard must be a bbox test, not a geometry query. That is what
    # makes its position in the ladder irrelevant -- unlike the reverted
    # waiting-pad cut, whose per-candidate distance query had to sit behind the
    # memoised clears_m to stay off the hot path.
    import ast
    src = open(os.path.join(os.path.dirname(__file__), '..',
                            'py_router', 'pcb_modification.py')).read()
    guard_fn = None
    for fn in ast.walk(ast.parse(src)):
        if isinstance(fn, ast.FunctionDef) and fn.name == '_in_dense_field':
            guard_fn = fn
    names = set()
    if guard_fn is not None:
        for node in ast.walk(guard_fn):
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Name):
                names.add(node.func.id)
    check('t_the_query_does_no_geometry',
          guard_fn is not None and not (names - {'min', 'max'}),
          f'_in_dense_field calls only {sorted(names)} -- bbox compares, so it '
          f'is cheap wherever it sits in the ladder')
    # And the region itself is built ONCE per call, not per candidate: its
    # construction must live outside the collapse loop.
    outer = [fn for fn in ast.walk(ast.parse(src))
             if isinstance(fn, ast.FunctionDef) and fn.name == 'collapse']
    inner = "\n".join(
        src.splitlines()[outer[0].lineno - 1:outer[0].end_lineno]) if outer else ''
    check('t_the_region_is_built_once',
          'get_footprint_bounds' not in inner,
          'the box list is built outside the greedy collapse loop')


def main():
    t_the_threshold_admits_the_packages_that_matter()
    t_the_rule_is_derived_not_a_fixed_pitch()
    t_a_tight_coarse_pitch_part_is_still_a_corridor()
    t_the_region_comes_from_the_boards_own_helpers()
    t_the_measured_geometry_is_decided_correctly()
    t_the_guard_is_scoped_to_ties_and_cheap()
    print()
    if FAILS:
        print(f"{len(FAILS)} FAILURE(S): {', '.join(FAILS)}")
        return 1
    print("ALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
