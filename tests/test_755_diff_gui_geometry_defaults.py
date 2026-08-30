#!/usr/bin/env python3
"""#755: the differential tab's geometry fallbacks come from routing_defaults.

Three `config.get(...)` fallbacks in the diff tab's `batch_route_diff_pairs`
call were bare literals that DISAGREED with the one place the repo declares
these knobs, sitting immediately above a line that did it correctly:

    clearance=config.get('clearance', 0.1),      # routing_defaults.CLEARANCE 0.25
    via_size=config.get('via_size', 0.3),        # routing_defaults.VIA_SIZE  0.5
    via_drill=config.get('via_drill', 0.2),      # routing_defaults.VIA_DRILL 0.3
    hole_to_hole_clearance=config.get(..., defaults.HOLE_TO_HOLE_CLEARANCE)

A FOURTH site the issue does not name carries the same key and the same
literal, ~45 lines above: `_diff_clearance = config.get('clearance', 0.1)`,
which seeds `net_clearances` for EVERY net on the board and the Min-Clearance
cap applied to them. It is the one with the widest blast radius, and it is
fixed here too -- a one-site fix would have shipped a false negative.

WHICH DEFAULT A DIFF-PAIR RUN INHERITS, since the issue says correctly that
"match the siblings" is not well formed:

    planes_gui.py:958-961,1080-1083   defaults.CLEARANCE / VIA_SIZE / VIA_DRILL
    swig_gui.py                       reads the dialog controls, no literals
    fanout_gui.py:1930-1932           defaults.BGA_CLEARANCE / BGA_VIA_SIZE

The answer is the SIGNAL set (CLEARANCE / VIA_SIZE / VIA_DRILL), not the
BGA_* set, and the deciding argument is not which sibling looks closest --
it is what the CLI does, because the two fronts must agree. `route_diff.py`
defaults `--clearance` / `--via-size` / `--via-drill` to None and resolves
each from the board's Default net class, falling back to exactly these three
constants. The BGA_* knobs are the fanout tab's escape-via geometry and have
no business pricing a differential pair.

REACHABILITY, which the issue asks for first and not last: these are `.get()`
fallbacks, and on the shipping GUI path they never fire. `_on_route` merges
`get_routing_config()` (which is `swig_gui._build_routing_config`, and that
sets 'clearance', 'via_size' and 'via_drill' unconditionally from the dialog
controls) with the diff tab's own `get_config()`. The AI plan executor does
not build a config either -- it drives `differential_tab._on_route(None)`, the
same path. So this is a LATENT TRAP, like the `cap_prefix` one in #742, and
not a live bug: a partially built config (a future caller, a test stand-in)
would have routed diff pairs at a geometry no CLI invocation can produce.
`test_the_shipping_path_populates_all_three` pins that reading, so if a
future refactor stops populating a key the trap becomes live and this file
says so.

WHY THIS IS A SOURCE GATE and not a routing test: the value under test is a
fallback that the shipping path never reaches, so there is no behaviour to
observe without constructing the very partial config that does not occur.
Importing the module needs wx. So the gate reads the source -- and is run in
BOTH directions: `_audit` must REJECT the pre-fix text, or it is a gate that
cannot see the thing it was written for.

    python3 tests/test_755_diff_gui_geometry_defaults.py
"""
import ast
import os
import sys

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _ROOT)
sys.path.insert(0, os.path.join(_ROOT, 'py_router'))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import routing_defaults as defaults

DIFF_GUI = os.path.join(_ROOT, 'kicad_routing_plugin', 'differential_gui.py')
SWIG_GUI = os.path.join(_ROOT, 'kicad_routing_plugin', 'routing_dialog.py')

# key -> the routing_defaults constant a diff-pair run must inherit.
EXPECTED = {
    'clearance': 'CLEARANCE',
    'via_size': 'VIA_SIZE',
    'via_drill': 'VIA_DRILL',
    'hole_to_hole_clearance': 'HOLE_TO_HOLE_CLEARANCE',
    'board_edge_clearance': 'BOARD_EDGE_CLEARANCE',
    'grid_step': 'GRID_STEP',
    'diff_pair_gap': 'DIFF_PAIR_GAP',
    'max_iterations': 'MAX_ITERATIONS',
    'diff_pair_width': 'DIFF_PAIR_WIDTH',
}
# Keys whose fallback routing_defaults does not declare: a literal is the only
# channel there, so they are not offences. Listed rather than skipped silently.
NO_CONSTANT = {'min_turning_radius', 'max_setback_angle', 'max_turn_angle',
               'diff_chamfer_extra', 'same_net_pad_clearance', 'coplanar_gap'}


def _audit(src):
    """Every `config.get('<key>', <literal>)` in `src` whose key names a knob
    routing_defaults declares. Returns [(lineno, key, literal, constant)]."""
    out = []
    for node in ast.walk(ast.parse(src)):
        if not (isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == 'get'
                and len(node.args) == 2
                and isinstance(node.args[0], ast.Constant)
                and isinstance(node.args[0].value, str)):
            continue
        key, fb = node.args[0].value, node.args[1]
        const = EXPECTED.get(key)
        if const is None or not isinstance(fb, ast.Constant):
            continue
        if not isinstance(fb.value, (int, float)) or isinstance(fb.value, bool):
            continue
        out.append((node.lineno, key, fb.value, const))
    return out


def run():
    fails = []

    def check(name, cond, note=''):
        print(('PASS' if cond else 'FAIL') + f': {name}' +
              (f'  [{note}]' if note else ''))
        if not cond:
            fails.append(name)

    _defaults_are_what_we_think(check)
    _the_file_is_clean(check)
    _the_gate_is_not_vacuous(check)
    _the_shipping_path_populates_them(check)

    print()
    if fails:
        print(f'{len(fails)} FAILURE(S): {fails}')
        return 1
    print('all checks passed')
    return 0


def _defaults_are_what_we_think(check):
    """The three constants, printed. If routing_defaults moves, the numbers in
    this file's docstring are stale and the run says so."""
    print('routing_defaults, the one place the repo declares these')
    for key, const in (('clearance', 'CLEARANCE'), ('via_size', 'VIA_SIZE'),
                       ('via_drill', 'VIA_DRILL')):
        print(f'        {key:<10} -> defaults.{const} = '
              f'{getattr(defaults, const)}')
    check('the three constants the diff tab now inherits still exist',
          all(hasattr(defaults, c) for c in EXPECTED.values()),
          'a renamed constant makes the fallbacks silently wrong again')
    # The measured divergence the issue is about, kept as a number so a future
    # reader can see what was actually at stake.
    print(f'        the divergence that was shipped: clearance 0.1 vs '
          f'{defaults.CLEARANCE}, via_size 0.3 vs {defaults.VIA_SIZE} '
          f'(a {defaults.VIA_SIZE - 0.3:.1f}mm barrel), via_drill 0.2 vs '
          f'{defaults.VIA_DRILL}')
    print()


def _the_file_is_clean(check):
    print('differential_gui.py: no bare-literal fallback for a declared knob')
    offences = _audit(open(DIFF_GUI, encoding='utf-8').read())
    for ln, key, lit, const in offences:
        print(f'        line {ln}: {key}={lit!r}, should be defaults.{const} '
              f'= {getattr(defaults, const)}')
    check('differential_gui.py is clean', not offences)

    # ALL FOUR sites, not just the three the issue names.
    src = open(DIFF_GUI, encoding='utf-8').read()
    for key, const in (('clearance', 'CLEARANCE'), ('via_size', 'VIA_SIZE'),
                       ('via_drill', 'VIA_DRILL')):
        n = src.count(f"config.get('{key}', defaults.{const})")
        print(f'        config.get({key!r}, defaults.{const}) x{n}')
    check('BOTH clearance sites are fixed -- the call site AND the '
          'net_clearances seed the issue does not name',
          src.count("config.get('clearance', defaults.CLEARANCE)") == 2,
          'a one-site fix here is a false negative')
    print()


def _the_gate_is_not_vacuous(check):
    """NEGATIVE CONTROL. Run the same checker over the pre-fix text; it must
    report all four sites. Without this the section above is a checker that
    could be blind to the very pattern it exists for."""
    print('negative control: the checker over the PRE-FIX source')
    pre_fix = (
        "def f(config, batch_route_diff_pairs, defaults):\n"
        "    _diff_clearance = config.get('clearance', 0.1)\n"
        "    return batch_route_diff_pairs(\n"
        "        clearance=config.get('clearance', 0.1),\n"
        "        via_size=config.get('via_size', 0.3),\n"
        "        via_drill=config.get('via_drill', 0.2),\n"
        "        hole_to_hole_clearance=config.get('hole_to_hole_clearance',\n"
        "                                          defaults.HOLE_TO_HOLE_CLEARANCE),\n"
        "        min_turning_radius=config.get('min_turning_radius', 0.2),\n"
        "    )\n")
    got = _audit(pre_fix)
    for ln, key, lit, const in got:
        print(f'        line {ln}: {key}={lit!r} (wants defaults.{const})')
    keys = sorted(k for _l, k, _v, _c in got)
    check('the checker finds all FOUR pre-fix sites',
          keys == ['clearance', 'clearance', 'via_drill', 'via_size'],
          f'got {keys}')
    check('and does not flag the line that was already right',
          not any(k == 'hole_to_hole_clearance' for _l, k, _v, _c in got))
    check('nor a knob routing_defaults does not declare '
          '(min_turning_radius: a literal is the only channel)',
          not any(k in NO_CONSTANT for _l, k, _v, _c in got))
    print()


def _the_shipping_path_populates_them(check):
    """The reachability claim, pinned rather than asserted in prose: the diff
    tab's config is `_build_routing_config` merged with the tab's own, and
    that builder sets all three keys unconditionally. If a refactor drops one,
    the latent trap becomes live and this arm goes red."""
    print('reachability: the shipping path populates all three keys')
    src = open(SWIG_GUI, encoding='utf-8').read()
    fn = next((n for n in ast.walk(ast.parse(src))
               if isinstance(n, ast.FunctionDef)
               and n.name == '_build_routing_config'), None)
    check('swig_gui._build_routing_config still exists (the diff tab merges '
          'it via get_routing_config)', fn is not None)
    if fn is None:
        return
    keys = {k.value for node in ast.walk(fn)
            if isinstance(node, ast.Dict)
            for k in node.keys
            if isinstance(k, ast.Constant) and isinstance(k.value, str)}
    for key in ('clearance', 'via_size', 'via_drill'):
        print(f'        {key!r} built unconditionally: {key in keys}')
    check('all three keys are set, so the fallbacks are a LATENT trap and '
          'this change is inert on the shipping path',
          {'clearance', 'via_size', 'via_drill'} <= keys,
          'if this fails, the fallbacks are LIVE and the change is not inert')
    print()


if __name__ == '__main__':
    sys.exit(run())
