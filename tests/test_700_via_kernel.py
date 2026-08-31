#!/usr/bin/env python3
"""#700 phase 1: ONE via centre-to-centre rule, shared by three callers.

`fab_tiers.min_via_center_distance` is the #491 arithmetic lifted out of
`diff_pair_routing._min_via_center_distance` so a stdlib-only consumer -- the
placement escape ledger -- can price a via slot without importing the routing
stack. Three sites open-coded it before this: the diff-pair launch, and
`add_gnd_vias` twice.

The move is only worth anything if it is behaviour-preserving AND if the
callers actually go through it, so both are asserted. A sweep that only ever
exercised the copper rule would pass on a kernel that dropped the drill term
entirely, which is the #491 bug -- so the sweep counts how often each rule
binds and refuses to be vacuous.
"""
import ast
import io
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from fab_tiers import min_via_center_distance          # noqa: E402


def t_the_kernel_is_the_max_of_the_copper_and_drill_rules():
    """Both rules, over a grid wide enough that each one binds many times."""
    copper = drill = tie = 0
    for vs in (0.25, 0.3, 0.45, 0.6, 0.8):
        for cl in (0.09, 0.1, 0.2, 0.25, 0.3):
            for vd in (0.15, 0.2, 0.3, 0.4):
                for hh in (0.0, 0.2, 0.25, 0.3, 0.5):
                    got = min_via_center_distance(vs, cl, vd, hh)
                    want = max(vs + cl, vd + hh)
                    assert abs(got - want) < 1e-12, (vs, cl, vd, hh, got, want)
                    if vs + cl > vd + hh:
                        copper += 1
                    elif vd + hh > vs + cl:
                        drill += 1
                    else:
                        tie += 1
    # Not decoration: a sweep in which the drill rule never binds cannot tell
    # this kernel from `via_diameter + clearance`, which is exactly the code
    # #491 replaced.
    assert copper > 50 and drill > 50, (copper, drill, tie)
    print(f"  PASS: copper binds {copper}, drill binds {drill}, {tie} ties")


def t_the_491_case_still_answers_the_drill_rule():
    """lvds_rx1_10, the board that produced the issue.

    0.3 via / 0.2 drill / 0.09 clearance, board min_hole_to_hole 0.3: the
    copper rule asks 0.39mm, the drill rule 0.5mm. Answering 0.39 ships legal
    copper the fab cannot drill.
    """
    assert abs(min_via_center_distance(0.3, 0.09, 0.2, 0.3) - 0.5) < 1e-12
    assert 0.3 + 0.09 < 0.5, 'the fixture stopped being a drill-bound case'
    print("  PASS: 0.50mm (drill), not 0.39mm (copper)")


def t_a_missing_hole_to_hole_is_zero_not_a_crash():
    """`config.hole_to_hole_clearance` is absent on older configs and the
    adapter passes `getattr(..., 0.0)`. None must read as 'no drill rule
    declared', never as a TypeError halfway through a route."""
    assert min_via_center_distance(0.5, 0.25, 0.3, None) == 0.75
    assert min_via_center_distance(0.5, 0.25, 0.3) == 0.75
    print("  PASS: None and omitted both mean 0.0")


def t_the_adapter_agrees_with_the_kernel():
    """`diff_pair_routing._min_via_center_distance` is now a config-shaped
    adapter. Compare it against the kernel on the same values, including a
    config with NO `hole_to_hole_clearance` attribute at all."""
    import diff_pair_routing as dpr

    class _Cfg:
        def __init__(self, vs, cl, vd, hh=None):
            self.via_size, self.clearance, self.via_drill = vs, cl, vd
            if hh is not None:
                self.hole_to_hole_clearance = hh

    for vs, cl, vd, hh in ((0.3, 0.09, 0.2, 0.3), (0.6, 0.2, 0.3, 0.25),
                           (0.45, 0.1, 0.2, None), (0.25, 0.3, 0.15, 0.0)):
        want = min_via_center_distance(vs, cl, vd, hh or 0.0)
        assert abs(dpr._min_via_center_distance(_Cfg(vs, cl, vd, hh))
                   - want) < 1e-12, (vs, cl, vd, hh)
    print("  PASS: the adapter and the kernel agree, missing attribute included")


def t_no_caller_still_open_codes_the_rule():
    """The point of a shared kernel is that the copies are GONE.

    Asserted on the AST rather than by grepping for the expression text: a
    comment quoting the old expression satisfies a grep and proves nothing,
    and this repo has shipped that mistake.

    Matched by SHAPE, not by names. The rule is `max(<via + something>,
    <drill + something>)` -- two ADDITIONS. Keying on "this max() mentions
    both via_size and via_drill" is too broad and this test caught itself
    doing it: `add_gnd_vias.py:148-149` is `max(via_drill / 2, via_size / 2 -
    track_width / 2)`, a comparison of RADII for a clear-check scan, which is
    a different quantity that must not be routed through this kernel.
    """
    def _addends(node):
        """Attribute names appearing inside a top-level `+` operand."""
        if not isinstance(node, ast.BinOp) or not isinstance(node.op, ast.Add):
            return set()
        return {n.attr for n in ast.walk(node) if isinstance(n, ast.Attribute)}

    for rel in ('py_router/add_gnd_vias.py', 'py_router/diff_pair_routing.py'):
        path = os.path.join(ROOT, rel)
        tree = ast.parse(io.open(path, encoding='utf-8').read())
        for node in ast.walk(tree):
            if not (isinstance(node, ast.Call)
                    and getattr(node.func, 'id', None) == 'max'):
                continue
            sums = [_addends(a) for a in node.args]
            copper = any({'via_size', 'via_diameter'} & s for s in sums)
            drill = any('via_drill' in s for s in sums)
            assert not (copper and drill), (
                f'{rel} still open-codes the via pitch rule: {sums}')
    print("  PASS: no open-coded max(via+clr, drill+h2h) left in either module")


TESTS = (t_the_kernel_is_the_max_of_the_copper_and_drill_rules,
         t_the_491_case_still_answers_the_drill_rule,
         t_a_missing_hole_to_hole_is_zero_not_a_crash,
         t_the_adapter_agrees_with_the_kernel,
         t_no_caller_still_open_codes_the_rule)


def _every_case_is_registered():
    """A test defined but not listed in TESTS never runs, and a green suite
    then reports coverage it does not have."""
    defined = {k for k in globals() if k.startswith('t_')}
    listed = {f.__name__ for f in TESTS}
    assert defined == listed, f'not registered: {sorted(defined - listed)}'


if __name__ == '__main__':
    _every_case_is_registered()
    for fn in TESTS:
        print(fn.__name__)
        fn()
    print('\nALL PASS')
