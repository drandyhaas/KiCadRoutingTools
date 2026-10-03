#!/usr/bin/env python3
"""#980: a static gate over the checks that admit or refuse copper against a
FOREIGN object -- they price the pair through the check_drc-parity helpers
and never regain a flat clearance term.

#980 and its two sweeps found 15 restore call sites and ~20 admission checks
pricing a foreign track, via or pad at one flat `config.clearance`. Each was
found by accident; this gate is what keeps a new one from shipping
unnoticed. It reads the code SHAPE (an AST), never comments or prose:

A. In every swept function, a `+`/`-` term that is a bare flat clearance
   (`config.clearance`, `tap_config.clearance`, `cfg.clearance`, or a
   `clearance` / `clr` name) must be on the ALLOWED list below, each entry
   with the reason it is not a pair: the meander search's inert-path scalars
   (the value a board with nothing declared reads, byte-identical) and an
   NPTH hole (a hole has no net).
B. Every swept function prices through `pair_clearance` /
   `pad_pair_clearance` (directly, through a `getattr(config,
   'pair_clearance')` handle, or through `_pair_floor`, which hands the
   foreign-distance helpers the base and class map they fold it from).
C. Every call of the restore predicate (and its aliases), of
   `partition_force_restores`, of the plane twin, and of the rescue leg /
   cap relocation passes `config=` -- the plane twin also both nets.

A negative control runs rules A and B over the pre-#980 restore predicate
and the pre-#980 sliver weld, and requires both to fail.

    python3 tests/test_980_no_flat_clearance_gate.py
"""
import ast
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

#: file -> the functions that decide admit/refuse against foreign copper
SITES = {
    'py_router/rip_up_reroute.py': ['_saved_route_colliders'],
    'py_router/plane_blocker_detection.py': ['_restored_piece_collides'],
    'py_router/kicad_oracle.py': ['_direct_sliver_weld', '_stitch_via_clear'],
    'py_router/layer_swap_optimization.py': ['_bare_pad_pair_vias_fit'],
    'py_router/diff_pair_multipoint.py': ['_fans_fit'],
    'py_router/length_matching.py': ['get_safe_amplitude_at_point',
                                     'get_safe_amplitude_for_diff_pair'],
    'py_router/stub_layer_switching.py': [
        'via_barrel_clear_of_foreign_copper', 'stub_clear_of_foreign_pads',
        'stub_clear_of_foreign_tracks'],
    'py_router/net_rescue.py': ['_leg_clear', '_via_site_clear',
                                '_find_cap_relocation', '_cap_conflicts'],
    'py_router/single_ended_routing.py': ['_unblock_via_refit',
                                          '_merge_terminal_to_exact'],
    'py_router/diff_pair_routing.py': ['_collapse_leg_attach_join'],
}

_INERT = ('the inert-path scalar: what the search reads when nothing is '
          'declared (pair_clearance_inert), kept in its term order so such a '
          'board is byte-identical; foreign items take the pair value')
#: (file, function, the term's code) -> why a flat clearance is right there
ALLOWED = {
    ('py_router/kicad_oracle.py', '_direct_sliver_weld',
     'reach + pad.drill / 2.0 + clr'):
        'an NPTH hole has no net: copper keeps the flat clearance from it',
    ('py_router/length_matching.py', 'get_safe_amplitude_at_point',
     'net_half + config.track_width / 2 + config.clearance'): _INERT,
    ('py_router/length_matching.py', 'get_safe_amplitude_at_point',
     'config.via_size / 2 + net_half + config.clearance'): _INERT,
    ('py_router/length_matching.py', 'get_safe_amplitude_at_point',
     'net_half + config.clearance'): _INERT + ' (and an OWN-net pad)',
    ('py_router/length_matching.py', 'get_safe_amplitude_for_diff_pair',
     'net_half + config.track_width / 2 + config.clearance'): _INERT,
    ('py_router/length_matching.py', 'get_safe_amplitude_for_diff_pair',
     'config.via_size / 2 + net_half + config.clearance'): _INERT,
    ('py_router/length_matching.py', 'get_safe_amplitude_for_diff_pair',
     'net_half + config.clearance'): _INERT,
}

#: Rule C: function name -> keywords every call must pass
CALLS = {
    '_saved_route_collides': ('config',),
    '_saved_route_colliders': ('config',),
    'partition_force_restores': ('config',),
    '_restored_piece_collides': ('config', 'piece_net', 'plane_net'),
    '_leg_clear': ('config',),
    '_find_cap_relocation': ('config',),
}


def _flat(n):
    return ((isinstance(n, ast.Attribute) and n.attr == 'clearance'
             and isinstance(n.value, ast.Name)
             and n.value.id in ('config', 'tap_config', 'cfg'))
            or (isinstance(n, ast.Name) and n.id in ('clearance', 'clr')))


def _flat_terms(fn):
    """Every BinOp(+/-) in `fn` with a bare flat clearance operand, as the
    smallest such expression's code (an outer sum that merely CONTAINS it is
    not repeated)."""
    out = set()
    for b in ast.walk(fn):
        if isinstance(b, ast.BinOp) and isinstance(b.op, (ast.Add, ast.Sub)) \
                and (_flat(b.left) or _flat(b.right)):
            out.add((b.lineno, ast.unparse(b)))
    return out


def _prices_pairwise(fn):
    for c in ast.walk(fn):
        if isinstance(c, ast.Call):
            f = c.func
            if isinstance(f, ast.Attribute) and f.attr in (
                    'pair_clearance', 'pad_pair_clearance'):
                return True
            if (isinstance(f, ast.Name) and f.id == 'getattr'
                    and len(c.args) >= 2
                    and isinstance(c.args[1], ast.Constant)
                    and c.args[1].value == 'pair_clearance'):
                return True
            # single_ended_routing._pair_floor: the (base, class map) pair
            # its foreign-distance helpers fold the pair value from
            if isinstance(f, ast.Name) and f.id == '_pair_floor':
                return True
    return False


def _rules_ab(rel, src, fn_name):
    tree = ast.parse(src)
    fn = next((n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)
               and n.name == fn_name), None)
    if fn is None:
        return [f'{rel}: {fn_name} is gone -- update SITES']
    probs = []
    for lineno, code in sorted(_flat_terms(fn)):
        if (rel, fn_name, code) not in ALLOWED:
            probs.append(f'{rel}:{lineno} {fn_name}: flat clearance term '
                         f'`{code}`')
    if not _prices_pairwise(fn):
        probs.append(f'{rel}: {fn_name} never calls pair_clearance / '
                     f'pad_pair_clearance')
    return probs


def test_rules_a_and_b():
    probs = []
    seen = set()
    for rel, fns in SITES.items():
        with open(os.path.join(ROOT, rel), encoding='utf-8') as fh:
            src = fh.read()
        for fn in fns:
            probs += _rules_ab(rel, src, fn)
            tree = ast.parse(src)
            node = next(n for n in ast.walk(tree)
                        if isinstance(n, ast.FunctionDef) and n.name == fn)
            seen |= {(rel, fn, code) for _l, code in _flat_terms(node)}
    assert not probs, '\n'.join(probs)
    stale = sorted(set(ALLOWED) - seen)
    assert not stale, f'ALLOWED entries that match nothing: {stale}'
    n = sum(len(v) for v in SITES.values())
    print(f"  PASS: {n} functions price pairwise; {len(ALLOWED)} flat "
          f"term(s), each allowed by name")


def _aliases(tree):
    """{local name: real name} for `import ... as` of the CALLS functions."""
    out = {n: n for n in CALLS}
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom):
            for a in node.names:
                if a.name in CALLS and a.asname:
                    out[a.asname] = a.name
        elif isinstance(node, ast.Assign) and isinstance(node.value, ast.Tuple):
            # `_lc666, _vc666 = _leg_clear, _via_site_clear`
            for t, v in zip(getattr(node.targets[0], 'elts', []),
                            node.value.elts):
                if isinstance(t, ast.Name) and isinstance(v, ast.Name) \
                        and v.id in CALLS:
                    out[t.id] = v.id
    return out


def test_rule_c_every_call_passes_config():
    probs, n = [], 0
    for sub in ('py_router', 'kicad_routing_plugin', 'py_placer', 'py_tools'):
        base = os.path.join(ROOT, sub)
        for dirpath, _dirs, files in os.walk(base):
            for f in files:
                if not f.endswith('.py'):
                    continue
                path = os.path.join(dirpath, f)
                rel = os.path.relpath(path, ROOT).replace(os.sep, '/')
                with open(path, encoding='utf-8') as fh:
                    tree = ast.parse(fh.read())
                names = _aliases(tree)
                for c in ast.walk(tree):
                    if not isinstance(c, ast.Call):
                        continue
                    f_ = c.func
                    nm = (f_.id if isinstance(f_, ast.Name)
                          else f_.attr if isinstance(f_, ast.Attribute)
                          else None)
                    real = names.get(nm)
                    if real is None:
                        continue
                    n += 1
                    have = {k.arg for k in c.keywords}
                    missing = [k for k in CALLS[real] if k not in have]
                    if missing:
                        probs.append(f'{rel}:{c.lineno} {nm}(...) lacks '
                                     f'{missing}')
    # 15 restore sites + the partition call + the predicate's own inner
    # call + 2 plane-twin calls + the rescue leg and cap relocation = 21
    assert n >= 21, f'only {n} calls found -- the walk lost some'
    assert not probs, '\n'.join(probs)
    print(f"  PASS: {n} restore / rescue call(s), every one passes config")


_PRE_980_RESTORE = '''
def _saved_route_colliders(saved_result, pcb_data, own_net_ids, clearance,
                           first_only=False):
    for s in segs:
        hw = s.width / 2.0
        for o in o_segs:
            thr = hw + o.width / 2.0 + clearance
    return hits
'''

_PRE_980_WELD = '''
def _direct_sliver_weld(pcb_data, net_id, ax, ay, bx, by, layer, config):
    clr = config.clearance
    for s in pcb_data.segments:
        need = reach + s.width / 2.0 + clr
    return None
'''


def test_negative_control():
    p1 = _rules_ab('py_router/rip_up_reroute.py', _PRE_980_RESTORE,
                   '_saved_route_colliders')
    p2 = _rules_ab('py_router/kicad_oracle.py', _PRE_980_WELD,
                   '_direct_sliver_weld')
    assert any('flat clearance term' in p for p in p1), p1
    assert any('never calls' in p for p in p1), p1
    assert any('flat clearance term' in p for p in p2), p2
    print("  PASS: the pre-#980 restore predicate and sliver weld both fail "
          "rules A and B")


TESTS = [test_rules_a_and_b, test_rule_c_every_call_passes_config,
         test_negative_control]


if __name__ == '__main__':
    for t in TESTS:
        print(f"--- {t.__name__}")
        t()
    print('ALL PASS')
