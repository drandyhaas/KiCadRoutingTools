#!/usr/bin/env python3
"""#1136: a static gate over the admit/refuse checks #1136 converted -- they
price another net's copper through the pair helpers and never regain a flat
clearance term unnoticed.

Each of these functions decides whether copper may sit next to ANOTHER
net's copper, and each used to price that pair at the flat
`config.clearance` (or a flat `clearance` argument). The behavioural test is
tests/test_1136_admission_pairwise.py; this one reads the code SHAPE (an
AST, never comments or prose), so a later edit that re-adds a flat term to
one of them fails here even where no geometry in that test reaches it.

A. Every READ of a flat clearance in a swept function -- `config.clearance`
   on a config-like name (or a local alias of one), or
   `getattr(x, 'clearance')` -- and every use of a flat VALUE (a bare
   `clearance` / `clr`, or a local alias of it or of a read) as an operand
   of arithmetic, a comparison, an augmented assignment or a `max` / `min`
   argument, must be on ALLOWED, keyed by the expression's code with its
   count, and each entry says why it is not another net's copper. Passing
   `clearance` on as the `base=` of a pricing call, or returning it from the
   no-config branch, is neither: those are the old floor, handed to the
   helper that prices the pair from it.
B. Every swept function prices through a pair helper: `pair_clearance`,
   `pad_pair_clearance(_before_override)`, `single_ended_routing._pair_floor`
   or `length_matching._meander_pair_pricing`.
C. The callers that thread the pricing in pass it: every call of
   `_restored_piece_collides` in repair_planes passes `config=`,
   `piece_net=` and `plane_net=`; every call of `_leg_clear` /
   `_find_cap_relocation` in net_rescue passes `config=`.

A negative control runs first: rule A must flag each flat spelling below,
and rule B a function that prices nothing.

What it cannot see: a function not in SITES; an allowlisted expression
reused for a different purpose (the key is the code and its count); exotic
spellings (`vars(config)['clearance']`, `operator.attrgetter`, a parameter
default bound to the flat value).

    python3 tests/test_1136_no_flat_clearance_gate.py
"""
import ast
import os
import sys
from collections import Counter

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

#: file -> the functions #1136 converted
SITES = {
    'py_router/stub_layer_switching.py': [
        'via_barrel_clear_of_foreign_copper', 'stub_clear_of_foreign_pads',
        'stub_clear_of_foreign_tracks'],
    'py_router/net_rescue.py': ['_leg_clear', '_via_site_clear',
                                '_find_cap_relocation', '_cap_conflicts'],
    'py_router/layer_swap_optimization.py': ['_bare_pad_pair_vias_fit'],
    'py_router/diff_pair_multipoint.py': ['_fans_fit'],
    'py_router/length_matching.py': ['get_safe_amplitude_at_point',
                                     'get_safe_amplitude_for_diff_pair',
                                     '_meander_pair_pricing'],
    'py_router/plane_blocker_detection.py': ['_restored_piece_collides'],
    'py_router/single_ended_routing.py': ['_unblock_via_refit',
                                          '_merge_terminal_to_exact'],
    'py_router/diff_pair_routing.py': ['_collapse_leg_attach_join',
                                       '_gnd_via_offsets', '_create_gnd_vias',
                                       '_settle_gnd_via',
                                       '_min_via_center_distance',
                                       '_pair_via_offset', '_try_route_direction',
                                       '_route_direct_coupled_middle',
                                       '_process_via_positions'],
}

_SAME_NET = ('a SAME-net item (check_drc grades no clearance between them): '
             'it keeps the value it was always tested at')
_FLAT_SCALAR = ('the flat scalar a board that declares no class and no rule '
                'reads, kept in its term order so such a board is unchanged; '
                'another net\'s item takes the pair value')
_EDGE = 'the board-edge keep-out, not a pair of nets'
_LM = 'py_router/length_matching.py'

#: (file, function, expression code) -> (count, why)
ALLOWED = {
    ('py_router/layer_swap_optimization.py', '_bare_pad_pair_vias_fit',
     'clearance = config.clearance'): (1, _SAME_NET + ' (two new vias of '
                                          'one net); also the pads\' inert '
                                          'fast path'),
    ('py_router/net_rescue.py', '_via_site_clear',
     'clr = config.clearance'): (1, 'the pads\' inert fast path: the value '
                                    'pad_pair_clearance_before_override '
                                    'returns when nothing is declared'),
    ('py_router/diff_pair_multipoint.py', '_fans_fit',
     'clearance = config.clearance'): (1, _SAME_NET),
    (_LM, 'get_safe_amplitude_at_point',
     'net_half + config.track_width / 2 + config.clearance + '
     'meander_clearance_margin + corner_margin'):
        (1, _FLAT_SCALAR + ' (required_clearance)'),
    (_LM, 'get_safe_amplitude_at_point',
     'config.via_size / 2 + net_half + config.clearance + '
     'meander_clearance_margin + corner_margin'):
        (1, _FLAT_SCALAR + ' (via_clearance)'),
    (_LM, 'get_safe_amplitude_at_point',
     'net_half + config.track_width / 2 + config.clearance'):
        (1, 'paired_clearance when there is NO partner (it is unused then); '
            'with one it is priced at the pair'),
    (_LM, 'get_safe_amplitude_at_point',
     'config.board_edge_clearance if config.board_edge_clearance > 0 else '
     'config.clearance'): (1, _EDGE),
    (_LM, 'get_safe_amplitude_for_diff_pair',
     'net_half + config.track_width / 2 + config.clearance + '
     'meander_clearance_margin + diff_pair_extra + corner_margin'):
        (1, _FLAT_SCALAR + ' (required_clearance)'),
    (_LM, 'get_safe_amplitude_for_diff_pair',
     'config.via_size / 2 + net_half + config.clearance + '
     'meander_clearance_margin + diff_pair_extra + corner_margin'):
        (1, _FLAT_SCALAR + ' (via_clearance)'),
    (_LM, 'get_safe_amplitude_for_diff_pair',
     'config.board_edge_clearance if config.board_edge_clearance > 0 else '
     'config.clearance'): (1, _EDGE),
    (_LM, 'get_safe_amplitude_at_point',
     'net_half + config.clearance + corner_margin'):
        (1, _FLAT_SCALAR + ' (pad_clearance; also an OWN-net pad)'),
    (_LM, 'get_safe_amplitude_for_diff_pair',
     'net_half + config.clearance + corner_margin + diff_pair_extra'):
        (1, _FLAT_SCALAR + ' (pad_clearance)'),
    ('py_router/diff_pair_routing.py', '_collapse_leg_attach_join',
     'config.pair_clearance(net_id, partner_segs[0].net_id, pen.layer) if '
     'partner_segs else config.clearance'):
        (1, 'the INTRA-pair floor (#1134): priced at the pair when the '
            'partner has copper; with none there is nothing to graze'),
    # #1218: the P/N transition vias and their partner tracks.
    ('py_router/diff_pair_routing.py', '_min_via_center_distance',
     "config.pair_clearance(net_a, net_b, kind='stack') if net_a is not None "
     "and net_b is not None else config.clearance"):
        (1, 'no pair given: the values-in adapter test_700 drives with a bare '
            'config, at the run\'s base; every router caller passes the pair'),
    ('py_router/diff_pair_routing.py', '_pair_via_offset',
     "config.pair_clearance(p_net, n_net, kind='stack') if p_net is not None "
     "and n_net is not None else config.clearance"):
        (1, 'no pair given: the run\'s base; every caller passes P and N'),
    ('py_router/diff_pair_routing.py', '_process_via_positions',
     "config.pair_clearance(p_net_id, n_net_id, kind='stack') if p_net_id is "
     "not None and n_net_id is not None else config.clearance"):
        (1, 'no pair given: the run\'s base; both callers pass P and N'),
    ('py_router/diff_pair_routing.py', '_try_route_direction',
     'config.track_width / 2 + config.clearance'):
        (1, 'the centerline setback\'s floor off the pad edge the pair '
            'launched from -- its OWN pad, no other net\'s copper'),
    ('py_router/diff_pair_routing.py', '_route_direct_coupled_middle',
     'config.track_width + config.diff_pair_gap - config.clearance'):
        (1, 'a lane-width COST margin for the partner leg (P and N couple at '
            'the gap; every obstacle stamp already carries its own pair '
            'clearance): no copper is admitted or refused on it'),
}

_CFG_NAMES = ('config', 'tap_config', 'cfg', 'c')
_CFG_ATTRS = ('config', 'cfg', '_config', '_cfg')
_FOLDS = ('max', 'min')
_PRICERS = ('pair_clearance', 'pad_pair_clearance',
            'pad_pair_clearance_before_override')
_PRICER_FUNCS = ('_pair_floor', '_meander_pair_pricing', '_gnd_via_offsets')


def _cfgish(v, aliases=()):
    return ((isinstance(v, ast.Name)
             and (v.id in _CFG_NAMES or v.id in aliases
                  or v.id.endswith('_cfg') or v.id.endswith('config')))
            or (isinstance(v, ast.Attribute) and v.attr in _CFG_ATTRS))


def _flat_read(n, aliases=()):
    if (isinstance(n, ast.Attribute) and n.attr == 'clearance'
            and isinstance(n.ctx, ast.Load) and _cfgish(n.value, aliases)):
        return True
    return (isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
            and n.func.id == 'getattr' and len(n.args) >= 2
            and isinstance(n.args[1], ast.Constant)
            and n.args[1].value == 'clearance')


def _aliases(fn):
    """(config aliases, flat-value names) assigned inside `fn`."""
    cfg, flat = set(), {'clearance', 'clr'}
    for _ in range(3):
        for n in ast.walk(fn):
            if not (isinstance(n, ast.Assign) and len(n.targets) == 1
                    and isinstance(n.targets[0], ast.Name)):
                continue
            t, v = n.targets[0].id, n.value
            if _cfgish(v, cfg):
                cfg.add(t)
            elif _flat_read(v, cfg) or (isinstance(v, ast.Name)
                                         and v.id in flat):
                flat.add(t)
    return cfg, flat


def census(fn):
    """{expression code: count} of every flat read and flat-value use."""
    cfg, flat = _aliases(fn)
    parents = {c: p for p in ast.walk(fn) for c in ast.iter_child_nodes(p)}

    def bare(n):
        return (isinstance(n, ast.Name) and n.id in flat
                and isinstance(n.ctx, ast.Load))
    out = Counter()
    for n in ast.walk(fn):
        if _flat_read(n, cfg):
            p = parents[n]
            # the smallest enclosing statement-level expression worth naming
            while isinstance(p, (ast.BinOp, ast.BoolOp)) and \
                    isinstance(parents.get(p), (ast.BinOp, ast.BoolOp)):
                p = parents[p]
            out[ast.unparse(p)] += 1
        elif isinstance(n, ast.BinOp) and (bare(n.left) or bare(n.right)):
            out[ast.unparse(n)] += 1
        elif isinstance(n, ast.Compare) and (
                bare(n.left) or any(bare(c) for c in n.comparators)):
            out[ast.unparse(n)] += 1
        elif isinstance(n, ast.AugAssign) and bare(n.value):
            out[ast.unparse(n)] += 1
        elif (isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
              and n.func.id in _FOLDS and any(bare(a) for a in n.args)):
            out[ast.unparse(n)] += 1
    return out


def prices_pairwise(fn):
    for n in ast.walk(fn):
        if isinstance(n, ast.Call):
            f = n.func
            if isinstance(f, ast.Attribute) and f.attr in _PRICERS:
                return True
            if isinstance(f, ast.Name) and f.id in _PRICER_FUNCS:
                return True
    return False


def _functions(tree):
    return {n.name: n for n in ast.walk(tree)
            if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))}


def _calls(tree, name):
    for n in ast.walk(tree):
        if isinstance(n, ast.Call):
            f = n.func
            fname = f.id if isinstance(f, ast.Name) else (
                f.attr if isinstance(f, ast.Attribute) else None)
            if fname == name:
                yield n


def negative_control():
    """Rule A must flag every flat spelling, and rule B a function that
    prices nothing; a gate that cannot is not a gate."""
    bad = {
        'attr': 'def f(config, d, r):\n    return d < r + config.clearance',
        'alias of config':
            'def f(config, d, r):\n    c2 = config\n'
            '    return d < r + c2.clearance',
        'getattr': "def f(config, d):\n"
                   "    return d < getattr(config, 'clearance', 0.2)",
        'bare argument': 'def f(clearance, d, r):\n    return d < r + clearance',
        'alias of a read':
            'def f(config, d):\n    flat = config.clearance\n'
            '    need = 0.3 + flat\n    return d < need',
        'folded': 'def f(clearance, lc):\n    return max(clearance, lc)',
    }
    for label, src in bad.items():
        fn = ast.parse(src).body[0]
        assert census(fn), f'negative control: rule A missed {label!r}'
        assert not prices_pairwise(fn), label
    good = ast.parse('def f(config, a, b, d, r):\n'
                     '    return d < r + config.pair_clearance(a, b)\n'
                     ).body[0]
    assert not census(good) and prices_pairwise(good)
    print(f"  negative control: rule A flags all {len(bad)} flat spellings, "
          f"rule B a function that prices nothing")


def main():
    negative_control()
    failures = []
    seen = set()
    for rel, names in SITES.items():
        with open(os.path.join(ROOT, rel), encoding='utf-8') as f:
            tree = ast.parse(f.read())
        fns = _functions(tree)
        for name in names:
            fn = fns.get(name)
            if fn is None:
                failures.append(f"{rel}: {name} not found (renamed? update "
                                f"SITES, or the gate covers nothing there)")
                continue
            if not prices_pairwise(fn):
                failures.append(f"{rel}:{name}: prices no pair (rule B)")
            for expr, count in census(fn).items():
                key = (rel, name, expr)
                seen.add(key)
                allowed = ALLOWED.get(key)
                if allowed is None:
                    failures.append(f"{rel}:{name}: flat clearance term "
                                    f"{expr!r} (x{count}) -- price the pair, "
                                    f"or allow it with the reason (rule A)")
                elif count != allowed[0]:
                    failures.append(f"{rel}:{name}: {expr!r} appears "
                                    f"{count}x, allowed {allowed[0]}x")
    for key in ALLOWED:
        if key not in seen:
            failures.append(f"stale ALLOWED entry (the code no longer has "
                            f"it): {key}")
    # rule C: the callers thread the pricing in
    for rel, callee, kws in (
            ('py_router/repair_planes.py', '_restored_piece_collides',
             ('config', 'piece_net', 'plane_net')),
            ('py_router/net_rescue.py', '_leg_clear', ('config',)),
            ('py_router/net_rescue.py', '_find_cap_relocation', ('config',))):
        with open(os.path.join(ROOT, rel), encoding='utf-8') as f:
            tree = ast.parse(f.read())
        calls = [c for c in _calls(tree, callee)]
        # net_rescue aliases _leg_clear as _lc666 before calling it
        if callee == '_leg_clear':
            calls += list(_calls(tree, '_lc666'))
        if not calls:
            failures.append(f"{rel}: no call of {callee} found (rule C has "
                            f"nothing to check)")
        for c in calls:
            have = {k.arg for k in c.keywords}
            missing = [k for k in kws if k not in have]
            if missing:
                failures.append(f"{rel}:{c.lineno}: {callee}(...) without "
                                f"{', '.join(k + '=' for k in missing)} "
                                f"(rule C)")
    n_fns = sum(len(v) for v in SITES.values())
    if failures:
        for f in failures:
            print(f"  FAIL: {f}")
        print(f"FAILED: {len(failures)} finding(s) over {n_fns} functions")
        return 1
    print(f"PASS: {n_fns} functions price the pair; {len(ALLOWED)} allowed "
          f"flat terms, each with its reason; the callers pass the pricing")
    return 0


if __name__ == '__main__':
    sys.exit(main())
