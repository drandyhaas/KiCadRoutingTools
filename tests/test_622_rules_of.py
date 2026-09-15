#!/usr/bin/env python3
"""#622 `awx/rules.py`: the ONE definition of the topo chain's design constants.

The module does NOT read boards -- py_router already resolves net classes,
`.kicad_dru` rules and fab tiers, and the topo chain is not a second front
for that. It holds the chain's constants, threads them into every module
that used to carry its own literal, and offers ONE seam
(`Rules.from_router_config`) for the day the main router supplies the
geometry instead.

What this asserts:

1. THE CONSTANTS ARE TODAY'S LITERALS, BIT FOR BIT -- 0.127 / 0.105 / 0.1 /
   0.25 / 0.15 / 0.9 / 0.35 / 0.38 / 0.232 and the five expressions derived
   from them, compared with `==` AND with `.hex()`, never `approx`. This is
   not pedantry: `0.1 + 0.005` is one ULP above the double `0.105` and
   `0.127 + 0.105` one ULP below `0.232`, and a 1-ULP clearance moves a
   routing-grid cell, which moves a lane, which changes the via count. The
   chain is byte-identical only while this holds.

2. `Rules.from_router_config` derives the chain's quantities from a
   supplied routing geometry by the documented formulas, and does NOT
   invent the two things a router config cannot say (the chain's two track
   widths, and band_tip's array geometry).

3. The install model is safe: installing DEFAULT leaves every module
   constant bit-identical (so a formula that drifts from the literal it
   replaced is caught here, not on a board), installing a DIFFERENT Rules
   really moves them, it reaches a stage running as `__main__`, and no
   design constant is captured as a DEFAULT ARGUMENT (bound at def time,
   where install cannot reach it).
"""
import ast
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
AWX = os.path.join(ROOT, 'awx')

sys.path.insert(0, AWX)

failures = []


def eq(name, got, want):
    """Float equality that also pins the BITS -- see the docstring."""
    if not (got == want and float(got).hex() == float(want).hex()):
        failures.append(f'{name}: {got!r} [{float(got).hex()}] != '
                        f'{want!r} [{float(want).hex()}]')


def truthy(name, cond, detail=''):
    if not cond:
        failures.append(f'{name}: {detail or "false"}')


# ------------------------------------------------------- 1. the constants
import rules as R  # noqa: E402

# The literals as the chain carried them before rules.py existed. These are
# the RECORDED VALUES: do not "simplify" one into an expression over
# another, that is the drift this file exists to catch.
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

for k, v in LITERALS.items():
    eq(f'DEFAULT.{k}', getattr(R.DEFAULT, k), v)
for k, v in DERIVED.items():
    eq(f'DEFAULT.{k}', getattr(R.DEFAULT, k), v)

# the two the braid reads off the board itself are NOT invented here
truthy('DEFAULT.hole_to_hole is None', R.DEFAULT.hole_to_hole is None,
       f'got {R.DEFAULT.hole_to_hole!r}')
truthy('DEFAULT.edge_clearance is None', R.DEFAULT.edge_clearance is None,
       f'got {R.DEFAULT.edge_clearance!r}')

# and the module does not read boards. Checked on the AST, not on the text:
# the docstring NAMES those modules to explain why it does not use them, and
# a substring guard would fire on the explanation -- which is how a guard
# ends up being "fixed" by deleting the paragraph that says what it means.
_tree = ast.parse(open(os.path.join(AWX, 'rules.py'), encoding='utf-8').read())
BANNED_IMPORTS = {'list_nets', 'kicad_dru', 'fab_tiers', 'kicad_parser',
                  'routing_defaults', 'fix_kicad_drc_settings', 'json'}
_imported = set()
for _n in ast.walk(_tree):
    if isinstance(_n, ast.Import):
        _imported |= {a.name.split('.')[0] for a in _n.names}
    elif isinstance(_n, ast.ImportFrom) and _n.module:
        _imported.add(_n.module.split('.')[0])
_bad = sorted(_imported & BANNED_IMPORTS)
truthy('rules.py imports nothing that reads a board', not _bad,
       f'imports {_bad} -- the board-derived resolution was removed '
       f'deliberately: py_router owns that, and two resolvers would be two '
       f'chances to disagree about one board')
_fns = {n.name for n in ast.walk(_tree)
        if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))}
truthy('rules.py defines no rules_of(board)', 'rules_of' not in _fns,
       f'functions: {sorted(_fns)}')
truthy('...and no chain module calls one',
       not any('rules_of(' in open(os.path.join(AWX, f), encoding='utf-8').read()
               for f in os.listdir(AWX) if f.endswith('.py')),
       'a caller still resolves rules from a board')


# ----------------------------------------------- 2. the handover seam
class _Cfg:
    """A stand-in for a py_router GridRouteConfig."""
    def __init__(self, **kw):
        self.__dict__.update(kw)


# the identity case: a config carrying the chain's own numbers reproduces
# the chain's own rules, bit for bit
same = R.Rules.from_router_config(_Cfg(clearance=0.1, track_width=0.127,
                                       via_size=0.25, via_drill=0.15))
for k, v in LITERALS.items():
    if k in ('fan_track', 'band_tip'):
        continue                      # not derivable from a router config
    eq(f'from_router_config(identity).{k}', getattr(same, k), v)
for k, v in DERIVED.items():
    if k in ('fan_clear',):
        continue                      # == clearance, checked below
    eq(f'from_router_config(identity).{k}', getattr(same, k), v)
eq('from_router_config(identity).fan_clear', same.fan_clear, 0.1)

# a WIDER supplied geometry: every derived quantity follows the formulas
wide = R.Rules.from_router_config(
    _Cfg(clearance=0.15, track_width=0.2, via_size=0.45, via_drill=0.3,
         hole_to_hole_clearance=0.25, board_edge_clearance=0.3))
eq('wide.clearance', wide.clearance, 0.15)
eq('wide.track', wide.track, 0.2)
eq('wide.hug', wide.hug, 0.155)
eq('wide.fan_clear', wide.fan_clear, 0.15)
eq('wide.lane_slice', wide.lane_slice, 0.355)
eq('wide.via_size', wide.via_size, 0.45)
eq('wide.via_drill', wide.via_drill, 0.3)
eq('wide.half_sep', wide.half_sep, (0.2 + 0.15) / 2)
eq('wide.via_need', wide.via_need, 0.45 / 2 + 0.155 + 0.2 / 2 + 0.03)
eq('wide.band_gap', wide.band_gap, 0.2 + 0.155 + 0.07)
eq('wide.end_keep', wide.end_keep, 0.2 + 0.155 + 0.05)
eq('wide.margin_out', wide.margin_out, 0.15 + 0.2 / 2)
eq('wide.hole_to_hole', wide.hole_to_hole, 0.25)
eq('wide.edge_clearance', wide.edge_clearance, 0.3)
# the pitches keep their FLOOR semantics: the chain's own value, never
# below one lane's slice at the supplied geometry (0.355 > 0.35 > 0.38? no:
# 0.355 exceeds lane_pitch 0.35 and not exit_pitch 0.38)
eq('wide.lane_pitch (floored at the slice)', wide.lane_pitch, 0.355)
eq('wide.exit_pitch (chain value still larger)', wide.exit_pitch, 0.38)

# the two things it must NOT invent
eq('fan_track defaults to the supplied width', wide.fan_track, 0.2)
truthy('...and says so in a note',
       any('fan_track' in n for n in wide.notes), f'notes={wide.notes}')
eq('band_tip keeps the chain default', wide.band_tip, 0.9)
split = R.Rules.from_router_config(
    _Cfg(clearance=0.15, track_width=0.2), fan_track=0.1, band_tip=1.2)
eq('an explicit fan_track is honoured', split.fan_track, 0.1)
eq('an explicit band_tip is honoured', split.band_tip, 1.2)
truthy('...and the note is gone when the caller split it',
       not any('fan_track' in n for n in split.notes), f'notes={split.notes}')

# a config missing the two required names is REFUSED, not silently defaulted
try:
    R.Rules.from_router_config(_Cfg(via_size=0.25))
    failures.append('from_router_config accepted a config with no clearance '
                    'or track_width -- it must refuse, not fall back to the '
                    'chain defaults, or a mis-wired handover looks like a '
                    'working one')
except ValueError as e:
    truthy('the refusal names what was missing',
           'clearance' in str(e) and 'track_width' in str(e), str(e))

# provenance is carried, so a stage's printed line says where its numbers
# came from
truthy('DEFAULT says it is the module constants',
       R.DEFAULT.source == 'awx/rules.py constants', R.DEFAULT.source)
truthy('a supplied geometry says so', 'router config' in wide.source,
       wide.source)


# --------------------------------------- 3a. install is inert on DEFAULT
import topo_strings as ts      # noqa: E402
import braid as br             # noqa: E402
import select_moves as sm      # noqa: E402
import source_realize as sr    # noqa: E402

TARGETS = [
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
# every module's constant must already BE the literal, before any install
LIT = dict(LITERALS, **DERIVED)
BEFORE_LIT = {
    'topo_strings.TRACK': 0.127, 'topo_strings.SPEC_CLEAR': 0.1,
    'topo_strings.MARGIN_OUT': LIT['margin_out'],
    'braid.TRACK': 0.127, 'braid.CLEAR': 0.105, 'braid.SPEC_CLEARANCE': 0.1,
    'braid.VIA_SIZE': 0.25, 'braid.VIA_DRILL': 0.15,
    'braid.MINP': 0.38, 'braid.LPITCH': 0.35,
    'braid.BAND_GAP': LIT['band_gap'], 'braid.HALF_SEP': LIT['half_sep'],
    'braid.VIA_NEED': LIT['via_need'], 'braid.END_KEEP': LIT['end_keep'],
    'select_moves.BAND_TIP': 0.9, 'select_moves.BAND_LPITCH': 0.35,
    'select_moves.NEST_IN': 0.232,
    'source_realize.FAN_TRACK': 0.1, 'source_realize.FAN_CLEAR': 0.1,
}
for n, g in TARGETS:
    eq(f'{n} as imported', g(), BEFORE_LIT[n])

installed = R.install(R.DEFAULT)
for n, g in TARGETS:
    eq(f'install(DEFAULT) leaves {n}', g(), BEFORE_LIT[n])
    truthy(f'install reaches {n}', n in installed,
           f'{n} is not in the installed list -- a supplied geometry would '
           f'never reach it')

# ...and a DIFFERENT Rules really moves them (a wiring fix can be inert)
R.install(R.Rules(clearance=0.15, track=0.2))
truthy('install(wide) moves braid.CLEAR', br.CLEAR == 0.155, f'got {br.CLEAR}')
truthy('install(wide) moves topo_strings.TRACK', ts.TRACK == 0.2, f'got {ts.TRACK}')
truthy('install(wide) moves braid.VIA_NEED',
       br.VIA_NEED == 0.25 / 2 + 0.155 + 0.2 / 2 + 0.03, f'got {br.VIA_NEED}')
truthy('install(wide) moves select_moves.NEST_IN', sm.NEST_IN == 0.355,
       f'got {sm.NEST_IN}')
R.install(R.DEFAULT)                       # leave the modules as we found them
for n, g in TARGETS:
    eq(f'restored {n}', g(), BEFORE_LIT[n])


# ------------------------------------------- 3b. the `__main__` trap
# A chain stage runs as `python3 braid.py`, so the router's own module is
# named '__main__' and `sys.modules['braid']` does not exist. An early
# version of install() used a plain `sys.modules.get(name)` and therefore
# wrote NOTHING into the router while the stage printed numbers it was not
# using. This runs a stand-in stage as a real subprocess.
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


# ------------------------------- 3c. no design constant as a default argument
# A module constant used as a DEFAULT ARGUMENT is bound at def time, so
# install (which writes module attributes) cannot reach it: the site would
# silently keep the chain's default under a supplied geometry. There was
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
                        f'{fn}:{node.lineno} def {node.name}(...={nm}): a design '
                        f'constant as a DEFAULT ARGUMENT is bound at def time and '
                        f'install cannot reach it -- read it in the body')
truthy('the AST scan actually scanned something', scanned > 10,
       f'only {scanned} file(s) parsed')

# every stage's entry point installs (the threading is the deliverable)
for fn in ('braid.py', 'fanout_from_plan.py', 'make_bench.py', 'pack_board.py',
           'replan.py', 'cut_ledger.py', 'collapse_dives.py'):
    body = open(os.path.join(AWX, fn), encoding='utf-8').read()
    truthy(f'{fn} installs the chain constants',
           'install_defaults()' in body,
           'a stage that does not install keeps its own module defaults, so '
           'a supplied geometry would reach every stage but this one')


# ------------------------------------------------------------------ verdict
if failures:
    print(f'FAIL test_622_rules_of: {len(failures)} problem(s)')
    for f in failures:
        print('  ' + f)
    sys.exit(1)
print('PASS test_622_rules_of: the constants are the literals bit-for-bit; '
      'from_router_config derives and refuses correctly; install inert on '
      'DEFAULT, live on a supplied geometry, reaches a __main__ stage; '
      f'{scanned} awx files clean of default-arg capture')
