#!/usr/bin/env python3
"""#900: the #530 pad-override cap belongs to `rules.min_clearance` ALONE.

`compute_targets` caps the recorded floor at the smallest copper-pad
`local_clearance` override on the board, because KiCad floors an override at
`rules.min_clearance` -- a floor above it flags copper routed correctly at the
override (#530, `tests/test_530_pad_override_replaces.py`). The cap's own
comment said this "costs nothing" because "the class clearances carry the real
requirement".

That was false: `apply_targets_to_project` read the same capped key for the net
classes, so one part carrying a 2 mil library override turned a requested
`--clearance 0.15` into a 0.0508 mm board -- Default AND every other class,
since the standalone writeback clamps them all. The next chain step then READS
that class back as the board's own floor (`route.py`'s
`board_default_netclass_clearance`), so the whole board routes at 0.0508. Run 25
(esp_prog) restored the floor by hand eight times and missed once.

WHAT THIS FILE CAN AND CANNOT PROVE. The CLI arms drive the real writeback end
to end. The live-board twin `apply_targets_to_board` cannot run here (it opens
with `import pcbnew`), so its half is held by a SOURCE guard plus the fake-board
arms for `clamp_nondefault_netclasses_on_board`, which has no pcbnew import.
A source guard is the weaker instrument -- `tests/test_782_nondefault_netclass_clamp.py`
says so about its own -- and the real one is
`tests/gui_parity/test_900_live_class_clearance.py`.
"""
import ast
import json
import os
import sys
import tempfile

RUN_ALL_TIMEOUT = 300

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
sys.path.insert(0, os.path.join(ROOT, 'tests', 'oracle'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

import run_utils                                              # noqa: E402
from constraint_agreement import write_board, DEFAULT_CLASS    # noqa: E402
from fix_kicad_drc_settings import (                           # noqa: E402
    _RULE_KEYS, _class_clearance, apply_targets_to_project,
    clamp_nondefault_netclasses_on_board, compute_targets, scan_board_minima)
from test_782_nondefault_netclass_clamp import board_with      # noqa: E402
import list_nets                                               # noqa: E402

SCRIPT = os.path.join(ROOT, 'py_router', 'fix_kicad_drc_settings.py')
OVERRIDE = 0.0508        # the 2 mil OLIMEX library override from run 25
ROUTED = 0.15            # what the brief asked for
STOCK = 0.2              # the class / rule the board ships with
WIDE = dict(DEFAULT_CLASS, name='Wide', clearance=0.4)

fails = []


def check(label, ok, detail=''):
    print(f"  {'PASS' if ok else 'FAIL'}  {label}"
          + (f"   [got {detail}]" if not ok and detail != '' else ''))
    if not ok:
        fails.append(label)


def near(a, b, eps=1e-9):
    return a is not None and abs(float(a) - float(b)) < eps


def _write(tmp, *, pad_override=None, dru=None, drop_classes=False):
    """A 2-pad board. `rules.min_*` start at STOCK so they have room to FALL --
    `write_board` seeds every rule at 0.0, and the writeback only lowers, so
    without this every rule assertion below would be vacuously 0.0."""
    pcb = os.path.join(tmp, 'b.kicad_pcb')
    fp = {'ref': 'U1', 'x': 10, 'y': 10, 'net_id': 1, 'net_name': 'A'}
    if pad_override is not None:
        fp['pad_clearance'] = pad_override
    write_board(pcb, footprints=[fp, {'ref': 'U2', 'x': 20, 'y': 10,
                                      'net_id': 2, 'net_name': 'B'}],
                rules={'min_clearance': STOCK, 'min_hole_clearance': 0.25},
                classes=[WIDE], dru=dru)
    if drop_classes:
        pro = os.path.splitext(pcb)[0] + '.kicad_pro'
        doc = json.load(open(pro, encoding='utf-8'))
        doc['net_settings']['classes'] = []
        json.dump(doc, open(pro, 'w', encoding='utf-8'), indent=2)
    run_utils.evidence(pcb, 'the synthetic board')
    return pcb


def _read(pcb):
    doc = json.load(open(os.path.splitext(pcb)[0] + '.kicad_pro', encoding='utf-8'))
    return (doc['board']['design_settings']['rules'],
            {c['name']: c for c in doc['net_settings']['classes']})


def _fix(pcb, clearance=ROUTED):
    run_utils.check([sys.executable, '-X', 'utf8', SCRIPT, pcb,
                     '--clearance', str(clearance)], accept=True)
    return _read(pcb)


# --------------------------------------------------------------------------
# 0. ON THE BRANCH: the fixture must actually carry what the arms below assume.
# --------------------------------------------------------------------------
def test_the_fixture_is_on_the_branch():
    print('\n-- 0. the board really carries a pad override below the request --')
    with tempfile.TemporaryDirectory() as tmp:
        pcb = _write(tmp, pad_override=OVERRIDE)
        m = scan_board_minima(pcb)
        check('scan_board_minima sees the pad override',
              near(m.get('min_pad_clearance_override'), OVERRIDE),
              m.get('min_pad_clearance_override'))
        rules, classes = _read(pcb)
        check('the staged project starts at the stock floor/classes',
              near(rules.get('min_clearance'), STOCK)
              and near(classes['Default']['clearance'], STOCK)
              and near(classes['Wide']['clearance'], 0.4))
        # A guard on this file's own constants, not on product code: it can
        # only fail if someone edits them, and it is here so that editing them
        # into a non-discriminating order is loud. Not evidence of anything.
        check('OVERRIDE < ROUTED < STOCK (this file s own constants)',
              OVERRIDE < ROUTED < STOCK)


# --------------------------------------------------------------------------
# 1. THE BUG.
# --------------------------------------------------------------------------
def test_the_cap_stops_at_the_rule_floor():
    print('\n-- 1. pad override 0.0508, --clearance 0.15 --')
    with tempfile.TemporaryDirectory() as tmp:
        pcb = _write(tmp, pad_override=OVERRIDE)
        _staged, _ = _read(pcb)
        rules, classes = _fix(pcb)
        check('rules.min_clearance capped at the pad override (#530 kept)',
              near(rules.get('min_clearance'), OVERRIDE), rules.get('min_clearance'))
        check('net_class[Default].clearance is the ROUTED 0.15, not the cap',
              near(classes['Default'].get('clearance'), ROUTED),
              classes['Default'].get('clearance'))
        check('net_class[Wide].clearance clamps to the routed 0.15, not the cap',
              near(classes['Wide'].get('clearance'), ROUTED),
              classes['Wide'].get('clearance'))
        # The CONSEQUENCE, not just the field: this is the value the next
        # routing step reads as "the board's own Default clearance".
        check('board_default_netclass_clearance now reads 0.15',
              near(list_nets.board_default_netclass_clearance(pcb), ROUTED),
              list_nets.board_default_netclass_clearance(pcb))
        check('min_hole_clearance keeps the routed value (never capped)',
              near(rules.get('min_hole_clearance'), ROUTED),
              rules.get('min_hole_clearance'))
        # The writeback may only ADD rule names. A project legitimately
        # carries rules this tool never writes (microvia_*), so compare the
        # DELTA, not the whole key set.
        _added = set(rules) - set(_staged)
        check('the writeback added no non-rule key',
              _added <= set(_RULE_KEYS), sorted(_added - set(_RULE_KEYS)))
        check('class_clearance is not in the shipped project at all',
              'class_clearance' not in rules)


def test_the_cap_is_a_min_not_an_assignment():
    print('\n-- 2. --clearance 0.03, BELOW the override --')
    with tempfile.TemporaryDirectory() as tmp:
        rules, classes = _fix(_write(tmp, pad_override=OVERRIDE), clearance=0.03)
        check('rules.min_clearance is 0.03, not lifted to the override',
              near(rules.get('min_clearance'), 0.03), rules.get('min_clearance'))
        check('net_class[Default].clearance is 0.03',
              near(classes['Default'].get('clearance'), 0.03),
              classes['Default'].get('clearance'))


def test_control_without_an_override():
    print('\n-- 3. control: the same board with no pad override --')
    with tempfile.TemporaryDirectory() as tmp:
        rules, classes = _fix(_write(tmp, pad_override=None))
        check('rules.min_clearance is the routed value (nothing to cap at)',
              near(rules.get('min_clearance'), ROUTED), rules.get('min_clearance'))
        check('net_class[Default].clearance is the routed value',
              near(classes['Default'].get('clearance'), ROUTED),
              classes['Default'].get('clearance'))
        check('the two agree when no cap fires -- the fix did not simply '
              'divorce them on every board',
              near(rules.get('min_clearance'), classes['Default'].get('clearance')))


def test_a_dru_rule_caps_the_floor_and_not_the_class():
    """Driven through `fix_project_for_output`, not the standalone CLI: the #498
    dru cap lives in that function (and in `apply_targets_to_board`), which is
    the path EVERY routing step takes. `main()` has never applied it -- that is
    pre-existing and out of scope here."""
    print('\n-- 4. a .kicad_dru rule below the class (#498) --')
    from fix_kicad_drc_settings import fix_project_for_output
    rule = '(rule relax (layer "B.Cu") (constraint clearance (min 0.05mm)))'
    with tempfile.TemporaryDirectory() as tmp:
        pcb = _write(tmp, dru=rule)
        fix_project_for_output(pcb, clearance=ROUTED, verbose=False)
        rules, classes = _read(pcb)
        check('rules.min_clearance capped at the dru rule',
              near(rules.get('min_clearance'), 0.05), rules.get('min_clearance'))
        check('net_class[Default].clearance keeps the routed value',
              near(classes['Default'].get('clearance'), ROUTED),
              classes['Default'].get('clearance'))
        check('net_class[Wide].clearance keeps the routed value too',
              near(classes['Wide'].get('clearance'), ROUTED),
              classes['Wide'].get('clearance'))


def test_a_created_default_class_is_born_at_the_routed_value():
    print('\n-- 5. no Default class, routed LOOSER than the 0.2 template --')
    with tempfile.TemporaryDirectory() as tmp:
        rules, classes = _fix(_write(tmp, pad_override=OVERRIDE, drop_classes=True),
                              clearance=0.3)
        check('a Default class was created', 'Default' in classes, sorted(classes))
        check('created at the routed 0.3, not the template 0.2 only-lower can '
              'never raise',
              near(classes.get('Default', {}).get('clearance'), 0.3),
              classes.get('Default', {}).get('clearance'))
        check('rules.min_clearance still capped at the pad override',
              near(rules.get('min_clearance'), OVERRIDE), rules.get('min_clearance'))


# --------------------------------------------------------------------------
# Unit arms: the arithmetic, and the two shapes the CLI arms cannot see.
# --------------------------------------------------------------------------
def test_compute_targets_emits_both_values():
    print('\n-- 6. compute_targets, directly --')
    t = compute_targets(clearance=ROUTED,
                        minima={'min_pad_clearance_override': OVERRIDE})
    check('min_clearance capped', near(t.get('min_clearance'), OVERRIDE),
          t.get('min_clearance'))
    check('class_clearance uncapped', near(t.get('class_clearance'), ROUTED),
          t.get('class_clearance'))
    # UNCONDITIONAL emission. If the key were emitted only when the cap fires,
    # every behaviour arm above would still pass -- the fallback makes the
    # no-cap case byte-identical. This is the only arm that sees it.
    t2 = compute_targets(clearance=ROUTED)
    check('class_clearance emitted even when no cap fires',
          near(t2.get('class_clearance'), ROUTED), sorted(t2))
    check('_class_clearance falls back for a hand-built dict '
          '(gui_utils / fanout_gui pass {min_clearance: ceiling})',
          near(_class_clearance({'min_clearance': 0.3}), 0.3))
    check('_class_clearance prefers the class key over the capped floor',
          near(_class_clearance({'min_clearance': OVERRIDE,
                                 'class_clearance': ROUTED}), ROUTED))


def test_rule_keys_is_derived_not_guessed():
    """The allow-list must equal what compute_targets can actually emit, or a
    newly added RULE is silently never written -- the cost of an allow-list,
    paid here rather than in a shipped project.

    Derived BY SOURCE, from every `targets["..."] =` assignment in the
    function. A first draft derived it from one CALL, which is blind to any key
    gated on a `minima` entry that call does not supply -- the shape
    `min_via_annular_width` already has, so the hole was one argument wide.
    """
    print('\n-- 7. _RULE_KEYS vs compute_targets --')
    src = open(os.path.join(ROOT, 'py_router', 'fix_kicad_drc_settings.py'),
               encoding='utf-8').read()
    fn = ''
    for node in ast.walk(ast.parse(src)):
        if isinstance(node, ast.FunctionDef) and node.name == 'compute_targets':
            fn = ast.get_source_segment(src, node) or ''
    check('compute_targets was found (guard not stale)', bool(fn))
    assigned = set()
    for node in ast.walk(ast.parse(fn.strip())):
        if (isinstance(node, ast.Subscript) and isinstance(node.value, ast.Name)
                and node.value.id == 'targets'
                and isinstance(node.slice, ast.Constant)
                and isinstance(node.slice.value, str)):
            assigned.add(node.slice.value)
    check('the source walk found the keys at all (non-vacuity)',
          len(assigned) >= 8, sorted(assigned))
    check('every key compute_targets can write is registered',
          assigned - {'class_clearance'} <= set(_RULE_KEYS),
          sorted(assigned - {'class_clearance'} - set(_RULE_KEYS)))
    check('every registered key is one compute_targets can write',
          set(_RULE_KEYS) <= assigned, sorted(set(_RULE_KEYS) - assigned))


def test_the_created_class_unit_arm():
    print('\n-- 8. apply_targets_to_project on a project with no classes --')
    proj = {'board': {'design_settings': {'rules': {'min_clearance': STOCK}}},
            'net_settings': {'meta': {'version': 0}, 'classes': []}}
    apply_targets_to_project(proj, compute_targets(
        clearance=ROUTED, minima={'min_pad_clearance_override': OVERRIDE}), {})
    d = next(c for c in proj['net_settings']['classes'] if c['name'] == 'Default')
    check('created Default is the routed 0.15, not the 0.0508 cap',
          near(d.get('clearance'), ROUTED), d.get('clearance'))
    check('the created class is COMPLETE (KiCad drops a sparse one)',
          set(d) >= set(DEFAULT_CLASS), sorted(set(DEFAULT_CLASS) - set(d)))


def test_the_live_board_non_default_clamp():
    """`clamp_nondefault_netclasses_on_board` has no pcbnew import, so the fake
    net classes reach it. This is the ONLY arm that kills a revert of its
    `nd_map` line -- the CLI arm above exercises a different function."""
    print('\n-- 9. the live-board non-Default clamp (fake classes) --')
    b, _d, o = board_with(STOCK, {'Wide': 0.4})
    clamp_nondefault_netclasses_on_board(b, compute_targets(
        clearance=ROUTED, minima={'min_pad_clearance_override': OVERRIDE}))
    check('Wide clamps to the routed 0.15, not the 0.0508 cap',
          near(o['Wide'].clearance_mm, ROUTED), o['Wide'].clearance_mm)
    b2, _d2, o2 = board_with(STOCK, {'Wide': 0.4})
    clamp_nondefault_netclasses_on_board(b2, {'min_clearance': 0.3})
    check('the hand-built {min_clearance: ceiling} caller still works',
          near(o2['Wide'].clearance_mm, 0.3), o2['Wide'].clearance_mm)


def test_apply_targets_to_board_reads_the_class_value():
    """SOURCE guard for the one writer no wx-free arm can drive.
    `apply_targets_to_board` opens with `import pcbnew`. The real instrument is
    tests/gui_parity/test_900_live_class_clearance.py."""
    print('\n-- 10. source guard: apply_targets_to_board --')
    src = open(os.path.join(ROOT, 'py_router', 'fix_kicad_drc_settings.py'),
               encoding='utf-8').read()
    fn = ''
    for node in ast.walk(ast.parse(src)):
        if isinstance(node, ast.FunctionDef) and node.name == 'apply_targets_to_board':
            fn = ast.get_source_segment(src, node) or ''
    check('apply_targets_to_board was found (guard not stale)', bool(fn))
    check('its Default-class map reads _class_clearance',
          '_class_clearance(targets)' in fn)
    check('and no longer reads the capped rule floor for a class',
          'nc_map = {"SetClearance": targets.get("min_clearance")}' not in fn)


def main():
    test_the_fixture_is_on_the_branch()
    test_the_cap_stops_at_the_rule_floor()
    test_the_cap_is_a_min_not_an_assignment()
    test_control_without_an_override()
    test_a_dru_rule_caps_the_floor_and_not_the_class()
    test_a_created_default_class_is_born_at_the_routed_value()
    test_compute_targets_emits_both_values()
    test_rule_keys_is_derived_not_guessed()
    test_the_created_class_unit_arm()
    test_the_live_board_non_default_clamp()
    test_apply_targets_to_board_reads_the_class_value()
    print()
    if fails:
        print(f"FAIL: {len(fails)} check(s) failed: {fails}")
        return 1
    print('PASS: the pad-override and .kicad_dru caps stop at '
          'rules.min_clearance; the net classes carry the clearance the board '
          'was routed to (#900)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
