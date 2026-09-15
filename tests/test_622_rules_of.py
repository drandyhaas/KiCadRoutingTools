#!/usr/bin/env python3
"""#622 `awx/rules.py`: the ONE resolver for the chain's design rules.

Three things are asserted, and the first is the one that matters:

1. THE BENCH RESOLVES TO TODAY'S LITERALS, BIT FOR BIT. Every number the
   chain used to carry as a literal (0.127 / 0.105 / 0.1 / 0.25 / 0.15 /
   0.9 / 0.35 / 0.38 / 0.232 and the four expressions derived from them)
   must come back from `rules_of(bench)` as the SAME DOUBLE -- compared
   with `==` AND with `.hex()`, never `approx`. This is not pedantry:
   `0.1 + 0.005` is one ULP above the double `0.105` and `0.127 + 0.105`
   one ULP below `0.232`, and a 1-ULP clearance moves a routing-grid cell,
   which moves a lane, which changes the via count. The flag-off chain is
   byte-identical only while this holds.

2. A board that asks for MORE gets more. A project whose Default net class
   declares 0.15 must raise the spec clearance, the hug, the lane slice and
   the fanout clearance -- and must NOT touch the track or the via, which
   no clearance rule binds.

3. A Default class clearance of 0 is UNSET, not a floor of zero (#966), so
   it resolves to exactly the same numbers as the bench.

Plus the two structural guards that make the install model safe: installing
the default Rules must leave every module constant bit-identical (so a
formula that drifts from the literal it replaced is caught here and not on
a board), and no design-rule constant may be captured as a DEFAULT ARGUMENT
(a default argument binds at def time, where install cannot reach it).
"""
import ast
import json
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
AWX = os.path.join(ROOT, 'awx')
BENCH = os.path.join(AWX, 'fb_t2q_fresh.kicad_pcb')

sys.path.insert(0, AWX)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

failures = []


def eq(name, got, want):
    """Float equality that also pins the BITS -- see the docstring."""
    ok = (got == want and float(got).hex() == float(want).hex())
    if not ok:
        failures.append(f'{name}: {got!r} [{float(got).hex()}] != '
                        f'{want!r} [{float(want).hex()}]')
    return ok


def truthy(name, cond, detail=''):
    if not cond:
        failures.append(f'{name}: {detail or "false"}')
    return cond


# ---------------------------------------------------------------- 1. bench
import rules as R  # noqa: E402

# The literals as the chain carried them before rules.py existed. These are
# the RECORDED VALUES: do not "simplify" one into an expression over another,
# that is the drift this file exists to catch.
LITERALS = {
    'clearance': 0.1,
    'track': 0.127,
    'via_size': 0.25,
    'via_drill': 0.15,
    'fan_track': 0.1,
    'lane_pitch': 0.35,
    'exit_pitch': 0.38,
    'band_tip': 0.9,
}
DERIVED = {
    'hug': 0.105,
    'lane_slice': 0.232,
    'fan_clear': 0.1,
    'band_gap': 0.127 + 0.105 + 0.07,
    'half_sep': (0.127 + 0.1) / 2,
    'via_need': 0.25 / 2 + 0.105 + 0.127 / 2 + 0.03,
    'end_keep': 0.127 + 0.105 + 0.05,
    'margin_out': 0.1 + 0.127 / 2,
}

if not os.path.isfile(BENCH):
    print(f'SKIP: no bench board at {BENCH}')
    sys.exit(0)

bench = R.rules_of(BENCH, dest_ref='DU1')
for k, v in LITERALS.items():
    eq(f'bench.{k}', getattr(bench, k), v)
for k, v in DERIVED.items():
    eq(f'bench.{k}', getattr(bench, k), v)

# the two the braid already read off the board, unchanged
eq('bench.hole_to_hole', bench.hole_to_hole, 0.127)
eq('bench.edge_clearance', bench.edge_clearance, 0.2)

# and DEFAULT -- what every module constant is initialized from -- agrees
for k, v in list(LITERALS.items()) + list(DERIVED.items()):
    eq(f'DEFAULT.{k}', getattr(R.DEFAULT, k), v)

# The bench's Default net class declares clearance 0.0. That must be read as
# UNSET (#966) -- if it were read as a floor of 0, or skipped for the wrong
# reason, the test above would still pass, so assert the REASON.
truthy('bench clearance provenance',
       bench.sources.get('clearance') == 'chain preference',
       f'expected the chain preference to win, got '
       f'{bench.sources.get("clearance")!r}')
with open(os.path.splitext(BENCH)[0] + '.kicad_pro') as fh:
    _pro = json.load(fh)
_dflt = next((c for c in _pro['net_settings']['classes']
              if c.get('name') == 'Default'), {})
truthy('bench Default class really is 0 (the premise of the test above)',
       _dflt.get('clearance') == 0.0,
       f'bench Default class clearance is {_dflt.get("clearance")!r} -- this '
       f'test no longer tests the UNSET rule')

# band_tip's formula, verified against the real array rather than asserted:
# DU1's pitch is 0.8 and the fanout exit margin 0.5, so 0.4 + 0.5 = 0.9.
truthy('band_tip provenance', 'DU1 pitch 0.8' in bench.sources.get('band_tip', ''),
       f'got {bench.sources.get("band_tip")!r}')


# ------------------------------------------------- 2. a board that asks 0.15
def _staged(tmp, mutate):
    """The bench copied with copy_board (#441: never cp a board without its
    project) and its project mutated."""
    dst = os.path.join(tmp, 'b.kicad_pcb')
    r = subprocess.run([sys.executable,
                        os.path.join(ROOT, 'py_router', 'copy_board.py'),
                        BENCH, dst], capture_output=True, text=True)
    if r.returncode != 0 or not os.path.isfile(dst):
        raise RuntimeError('copy_board failed: ' + (r.stdout + r.stderr)[-400:])
    pro = os.path.splitext(dst)[0] + '.kicad_pro'
    with open(pro) as fh:
        p = json.load(fh)
    mutate(p)
    with open(pro, 'w') as fh:
        json.dump(p, fh, indent=2)
    return dst


def _set_default_clearance(p, v):
    for c in p['net_settings']['classes']:
        if c.get('name') == 'Default':
            c['clearance'] = v


with tempfile.TemporaryDirectory() as tmp:
    wide = R.rules_of(_staged(tmp, lambda p: _set_default_clearance(p, 0.15)))
    eq('wide.clearance', wide.clearance, 0.15)
    eq('wide.hug', wide.hug, 0.155)
    eq('wide.fan_clear', wide.fan_clear, 0.15)
    eq('wide.lane_slice', wide.lane_slice, 0.282)
    eq('wide.half_sep', wide.half_sep, (0.127 + 0.15) / 2)
    # a clearance rule binds no width: track and via must NOT move
    eq('wide.track', wide.track, 0.127)
    eq('wide.fan_track', wide.fan_track, 0.1)
    eq('wide.via_size', wide.via_size, 0.25)
    eq('wide.via_drill', wide.via_drill, 0.15)
    truthy('wide clearance provenance',
           'Default net class' in wide.sources.get('clearance', ''),
           f'got {wide.sources.get("clearance")!r}')
    # the lane pitch is floored at one lane's slice, and 0.282 < 0.35, so it
    # does not bind yet -- the floor is real but inert here
    eq('wide.lane_pitch', wide.lane_pitch, 0.35)

    # ...and at a clearance where the slice DOES exceed the preferred pitch
    huge = R.rules_of(_staged(tmp, lambda p: _set_default_clearance(p, 0.3)))
    eq('huge.clearance', huge.clearance, 0.3)
    eq('huge.lane_slice', huge.lane_slice, 0.432)
    eq('huge.lane_pitch', huge.lane_pitch, 0.432)
    eq('huge.exit_pitch', huge.exit_pitch, 0.432)

    # A class TRACK WIDTH is a preference, not a constraint, so it must NOT
    # raise the braid's lane track: the board carries two track widths on
    # purpose (the fanout's stubs and the braid's wider lanes), and a 2-layer
    # board whose Default class says 0.3 would otherwise make the bus
    # unroutable. Only rules.min_track_width (what KiCad grades a width
    # against) may raise it. Without this arm the resolver can start reading
    # the class width and every other assertion here still passes -- measured
    # with awx/tmp/mutate_rules.py, where that mutation SURVIVED.
    def _wide_track(p):
        for c in p['net_settings']['classes']:
            if c.get('name') == 'Default':
                c['track_width'] = 0.3
    wt = R.rules_of(_staged(tmp, _wide_track))
    eq('class track 0.3 does not move the lane track', wt.track, 0.127)
    eq('class track 0.3 does not move the fanout track', wt.fan_track, 0.1)

    # ...while the board MINIMUM does, on both, because KiCad grades it
    def _min_track(p):
        p['board']['design_settings']['rules']['min_track_width'] = 0.2
    mt = R.rules_of(_staged(tmp, _min_track))
    eq('min_track_width 0.2 raises the lane track', mt.track, 0.2)
    eq('min_track_width 0.2 raises the fanout track', mt.fan_track, 0.2)
    truthy('min_track provenance',
           'min_track_width' in mt.sources.get('track', ''),
           f'got {mt.sources.get("track")!r}')

    # a bigger board minimum via, and the annular ring closing over the pair
    def _big_via(p):
        r_ = p['board']['design_settings']['rules']
        r_['min_via_drill'] = 0.3
        r_['min_via_annular_width'] = 0.1
    bv = R.rules_of(_staged(tmp, _big_via))
    eq('min_via_drill raises the drill', bv.via_drill, 0.3)
    eq('annular ring raises the barrel', bv.via_size, 0.5)   # 0.3 + 2*0.1

    # a .kicad_dru layer rule on an outer layer, tighten-only (#498)
    dru = _staged(tmp, lambda p: None)
    with open(os.path.splitext(dru)[0] + '.kicad_dru', 'w') as fh:
        fh.write('(version 1)\n(rule outer_wide (layer F.Cu)\n'
                 '  (constraint clearance (min 0.2mm)))\n')
    ruled = R.rules_of(dru)
    eq('ruled.clearance', ruled.clearance, 0.2)
    truthy('ruled clearance provenance',
           '.kicad_dru' in ruled.sources.get('clearance', ''),
           f'got {ruled.sources.get("clearance")!r}')
    # a RELAXING rule must not lower the chain's number (scalar clearance:
    # taking one layer's relaxation would under-space the other)
    with open(os.path.splitext(dru)[0] + '.kicad_dru', 'w') as fh:
        fh.write('(version 1)\n(rule outer_tight (layer F.Cu)\n'
                 '  (constraint clearance (min 0.05mm)))\n')
    relaxed = R.rules_of(dru)
    eq('relaxed.clearance (tighten-only)', relaxed.clearance, 0.1)


# ------------------------------------------- 3. unset / declares nothing
with tempfile.TemporaryDirectory() as tmp:
    unset = R.rules_of(_staged(tmp, lambda p: _set_default_clearance(p, 0.0)))
    for k, v in LITERALS.items():
        if k == 'band_tip':
            continue                      # needs dest_ref; covered above
        eq(f'unset.{k}', getattr(unset, k), v)
    for k, v in DERIVED.items():
        eq(f'unset.{k}', getattr(unset, k), v)

    # ...and a board with NO project at all: the chain's defaults, plus the
    # fab floor for its layer count. Written by hand (NOT copy_board) exactly
    # because the point is a board with no siblings.
    bare = os.path.join(tmp, 'bare.kicad_pcb')
    shutil.copyfile(BENCH, bare)
    none = R.rules_of(bare)
    for k, v in LITERALS.items():
        if k == 'band_tip':
            continue
        eq(f'no-project.{k}', getattr(none, k), v)
    truthy('no-project hole_to_hole is None', none.hole_to_hole is None,
           f'got {none.hole_to_hole!r}')


# ------------------------------------- structural guard A: install is inert
# Installing the DEFAULT rules must leave every constant exactly as the
# module defined it. This catches a derived formula in rules.py drifting from
# the literal it replaced -- on the bench, in milliseconds, instead of on a
# routed board.
import topo_strings as ts      # noqa: E402
import braid as br             # noqa: E402
import select_moves as sm      # noqa: E402
import source_realize as sr    # noqa: E402

BEFORE = [
    ('topo_strings.TRACK', lambda: ts.TRACK), ('topo_strings.SPEC_CLEAR', lambda: ts.SPEC_CLEAR),
    ('topo_strings.MARGIN_OUT', lambda: ts.MARGIN_OUT),
    ('braid.TRACK', lambda: br.TRACK), ('braid.CLEAR', lambda: br.CLEAR),
    ('braid.SPEC_CLEARANCE', lambda: br.SPEC_CLEARANCE),
    ('braid.VIA_SIZE', lambda: br.VIA_SIZE), ('braid.VIA_DRILL', lambda: br.VIA_DRILL),
    ('braid.MINP', lambda: br.MINP), ('braid.LPITCH', lambda: br.LPITCH),
    ('braid.BAND_GAP', lambda: br.BAND_GAP), ('braid.HALF_SEP', lambda: br.HALF_SEP),
    ('braid.VIA_NEED', lambda: br.VIA_NEED), ('braid.END_KEEP', lambda: br.END_KEEP),
    ('select_moves.BAND_TIP', lambda: sm.BAND_TIP),
    ('select_moves.BAND_LPITCH', lambda: sm.BAND_LPITCH),
    ('select_moves.NEST_IN', lambda: sm.NEST_IN),
    ('source_realize.FAN_TRACK', lambda: sr.FAN_TRACK),
    ('source_realize.FAN_CLEAR', lambda: sr.FAN_CLEAR),
]
snapshot = {n: g() for n, g in BEFORE}
installed = R.install(R.DEFAULT)
for n, g in BEFORE:
    eq(f'install(DEFAULT) leaves {n}', g(), snapshot[n])
for n, _ in BEFORE:
    truthy(f'install reaches {n}', n in installed,
           f'{n} is not in the installed list -- a stage would keep the '
           f'0.1 mm-process default for it')

# and installing a WIDER rule really moves them (a wiring fix can be inert)
R.install(R.Rules(clearance=0.15, track=0.2))
truthy('install(wide) moves braid.CLEAR', br.CLEAR == 0.155, f'got {br.CLEAR}')
truthy('install(wide) moves topo_strings.TRACK', ts.TRACK == 0.2, f'got {ts.TRACK}')
truthy('install(wide) moves braid.VIA_NEED',
       br.VIA_NEED == 0.25 / 2 + 0.155 + 0.2 / 2 + 0.03, f'got {br.VIA_NEED}')
R.install(R.DEFAULT)                       # leave the modules as we found them


# ------------------------ structural guard C: the `__main__` trap, directly
# A chain stage runs as `python3 braid.py`, so the router's own module is
# named '__main__' and `sys.modules['braid']` does not exist. The first
# version of install() used a plain `sys.modules.get(name)` and therefore
# wrote NOTHING into the router while the stage printed the resolved rules
# it was not using -- a board routed at the 0.1 mm defaults with a 0.15 line
# in its log. This runs a stand-in stage as a real subprocess: a file named
# braid.py whose constants install must reach.
with tempfile.TemporaryDirectory() as tmp:
    stage = os.path.join(tmp, 'braid.py')
    with open(stage, 'w') as fh:
        fh.write(
            'import sys\n'
            f'sys.path.insert(0, {AWX!r})\n'
            'import rules\n'
            'CLEAR = 0.105\n'
            'SPEC_CLEARANCE = 0.1\n'
            'TRACK = 0.127\n'
            'VIA_SIZE = 0.25\n'
            'VIA_DRILL = 0.15\n'
            'rules.install(rules.Rules(clearance=0.15, track=0.2))\n'
            'print(CLEAR, SPEC_CLEARANCE, TRACK)\n')
    out = subprocess.run([sys.executable, stage], capture_output=True, text=True)
    got = out.stdout.strip()
    truthy('install reaches a stage running as __main__',
           got == '0.155 0.15 0.2',
           f'a file named braid.py run as a script printed {got!r} '
           f'(expected "0.155 0.15 0.2") -- install did not reach it; '
           f'stderr: {out.stderr.strip()[-200:]}')


# --------------------------- structural guard B: no default-argument capture
# A module constant used as a DEFAULT ARGUMENT is bound at def time, so
# rules.install (which writes module attributes) cannot reach it: the site
# would silently keep the bench's number on every other board. There was
# exactly one (braid.clip_round_ends' keep_r) and it was fixed; this stops
# the next one.
CONSTS = {'TRACK', 'CLEAR', 'SPEC_CLEARANCE', 'SPEC_CLEAR', 'VIA_SIZE',
          'VIA_DRILL', 'END_KEEP', 'VIA_NEED', 'BAND_GAP', 'HALF_SEP', 'MINP',
          'LPITCH', 'NEST_IN', 'BAND_TIP', 'BAND_LPITCH', 'MARGIN_OUT',
          'FAN_TRACK', 'FAN_CLEAR'}
scanned = 0
for fn in sorted(os.listdir(AWX)):
    if not fn.endswith('.py'):
        continue
    try:
        tree = ast.parse(open(os.path.join(AWX, fn), encoding='utf-8').read())
    except SyntaxError as e:
        failures.append(f'{fn} does not parse: {e}')
        continue
    scanned += 1
    for node in ast.walk(tree):
        if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            continue
        for d in list(node.args.defaults) + [k for k in node.args.kw_defaults if k]:
            for s in ast.walk(d):
                nm = (s.id if isinstance(s, ast.Name) else
                      s.attr if isinstance(s, ast.Attribute) else None)
                if nm in CONSTS:
                    failures.append(
                        f'{fn}:{node.lineno} def {node.name}(...={nm}): a design-rule '
                        f'constant as a DEFAULT ARGUMENT is bound at def time and '
                        f'rules.install cannot reach it -- read it in the body')
truthy('the AST scan actually scanned something', scanned > 10,
       f'only {scanned} file(s) parsed')


# ------------------------------------------------------------------ verdict
if failures:
    print(f'FAIL test_622_rules_of: {len(failures)} problem(s)')
    for f in failures:
        print('  ' + f)
    sys.exit(1)
print('PASS test_622_rules_of: bench resolves to the literals bit-for-bit; '
      '0.15 / dru / unset / no-project arms correct; install inert on DEFAULT, '
      f'live on a wider rule; {scanned} awx files clean of default-arg capture')
