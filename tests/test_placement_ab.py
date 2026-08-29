#!/usr/bin/env python3
"""Paired A/B for placement objective terms, graded by an INDEPENDENT check.

Every term in the quench objective is a claim about routability, and a term can
always be made to improve the number it is itself computed from. This runs the
same board through the optimizer twice -- flag off, flag on -- writes both, and
then grades both with `floorplan.grade(..., with_health=True)`, which
re-derives its corridors from the FINAL poses. The optimizer minimises against
rectangles frozen at construction; the grader does not use them. So a term that
only games its own model shows up here as "improved nothing".

Table-driven on purpose (`ROWS`): the next term under test adds a row, not a
file. Adding a row does not change how any existing row is judged.

The verdict is a PAIRED, DIRECTIONAL, NON-REGRESSION rule, never a per-board
absolute:

  * the claimed signal must improve on at least N-1 BOARDS,
  * and must regress on NONE,
  * while `crossings` and `hpwl` -- the terms that already shipped -- do not
    get worse anywhere.

Unchanged boards count as neutral AND ARE PRINTED. A silently-dropped neutral
board is how a term with no effect on 3 of 4 boards reads as a clean sweep.

THE MEASURED NUMBERS LIVE IN `tests/placement_ab_baseline.json`, NOT IN PROSE.
A row's `why` records the MECHANISM; every number it once carried is in the
committed baseline, which this script re-measures and compares on every run.

That is not decoration. `corridor-ulx3s` sat rejected on a recorded measurement
whose claimed signal had reversed, and this gate printed PASS the whole time
(#694). The reason is worth stating exactly, because the obvious reading is
wrong: the gate did NOT compare the signal. `_verdict` collapses three
criteria -- the signal, the guards, and intent errors -- into ONE categorical
mark, and only that mark is checked against `expect`. So a reversal in the
signal was MASKED by a different criterion turning the mark `regress` for its
own reasons. An aggregate verdict cannot report which of its inputs moved;
only per-key evidence can, and prose cannot be re-run at all.

Usage:
    python3 -X utf8 tests/test_placement_ab.py            # the default table
    python3 -X utf8 tests/test_placement_ab.py --row corridor-ulx3s
    python3 -X utf8 tests/test_placement_ab.py --list
    python3 -X utf8 tests/test_placement_ab.py --self-test   # gate logic only
    python3 -X utf8 tests/test_placement_ab.py --write-baseline
"""
import argparse
import json
import os
import shutil
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_placer'))  # placement split
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_router'))  # placement split
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_tools'))  # placement split

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BOARDS = os.path.join(ROOT, 'kicad_files')

# Fourteen full quenches (7 rows x off/on) on five boards. Measured 233-340 s
# of row time for the first three rows; the #702 rows add roughly as much
# again. Declared with headroom so a slower box reports FAIL, not TIME.
RUN_ALL_TIMEOUT = 1800

DEFAULT_BASELINE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                'placement_ab_baseline.json')

# The sources `resolve_blocks` derives from when the grade resolves an emitted
# intent's blocks. `emit_intent` stamps EVERY block with a `group` key, so a
# grade given no sources resolves every one of them to nothing -- see `_run`.
# `('kicad', 'sheet')` is what `groups.parse_sources('auto')` returns, and it is
# what `check_floorplan.py`, `place_seed.py` and `place_portfolio.py` all
# default to; spelled out rather than imported so the fixture cannot drift with
# a change to `auto`'s membership.
GROUP_SOURCES = ('kicad', 'sheet')

# The keys compared against the committed baseline.
#
# `seconds` is wall-clock and is never compared. `corridor_cut` is never
# compared either, and that is deliberate: the OFF arm is not given
# `corridor_specs` (see `run_row`), so it builds no corridor and reports 0.0.
# "0.0 -> 806.84" is not a regression, it is a comparison that was never made,
# and recording it as evidence would pin a fiction.
BASELINE_INT_KEYS = ('crossings', 'health_bus_foreign_crossings',
                     'intent_errors', 'intent_errors_enforced',
                     'intent_errors_other')
BASELINE_FLOAT_KEYS = ('hpwl', 'health_block_displacement_max_mm')
BASELINE_DICT_KEYS = ('intent_errors_by_rule',)

# Recorded as EVIDENCE, deliberately not graded: `health_block_displacement_max_mm`
# is neither a row's `signal` nor a `guard`, and it currently worsens in the ON
# arm on two of the three boards. Promoting it to a guard would fail the corridor
# rows for a second reason without anyone having decided that displacing a block
# further is a cost here -- so it is recorded, visible, and left to whoever makes
# that call. Recording it is not the same as agreeing with it.

# CLAUDE.md's rule, now enforced here instead of only stated there. Three is
# also about the smallest table at which "improve on N-1, regress on none" says
# anything: a term whose per-board direction is a coin flip passes 1 run in 2^N,
# so 1 in 8 at N=3. (That is an upper bound on the null rate, not the rate: a
# term with no effect at all marks `neutral`, which fails the rule outright.)
MIN_TRIAL_BOARDS = 3


#: Sentinel a row puts in `quench_on` to mean "the gate resolved from THIS
#: row's intent". Not the intent itself: the table is read by `--list` and by
#: the baseline comparator, and an Intent object in it would print as a repr.
ROW_INTENT = '<resolved from this row>'

# --- the table ------------------------------------------------------------
#
# `quench_on` is merged into the OFF kwargs to make the ON run, so a row states
# exactly one difference. `corridors` are the health.bus_corridors the intent
# declares; the grader reads the same declaration and re-derives the geometry.
#
# `signal` is the JSON_SUMMARY-style key the row claims to improve, and
# `guard` the keys that must not get worse. Lower is better for all of them.
# `expect` pins the mark that was MEASURED. Omit it for a term still on trial;
# a trial row must improve on >= N-1 BOARDS and regress on none.
#
# `rejected` marks a term that was tried, measured, and NOT adopted. Its rows
# stay in the table as the evidence and as a change detector -- they are judged
# against their recorded marks rather than against "the term must help", so a
# rejected term does not leave a permanent red mark that someone eventually
# deletes along with the finding.
#
# `why` states the MECHANISM only. Numbers belong in the baseline, which is
# re-measured and compared; a number in a comment is re-read and believed.
#
# The corridor globs name sub-buses SEPARATELY. Merging them halves the
# corridor's `cover` (address and data leave the part on different faces, so
# the endpoint average lands between them) and the resulting rectangle is a
# fiction: ulx3s SDRAM_A* scores cover 0.81, merged SDRAM_* scores 0.46. This
# is not a footnote -- the first run of this harness used the merged glob and
# reported the term INERT, because it had been pointed at a phantom.
ROWS = [
    {
        'name': 'corridor-ulx3s',
        'board': 'ulx3s.kicad_pcb',
        'corridors': [{'name': 'sdram_a', 'nets': ['SDRAM_A*'],
                       'width_mm': 8.0},
                      {'name': 'sdram_d', 'nets': ['SDRAM_D*'],
                       'width_mm': 8.0}],
        'ignore_nets': ['GND', '+3V3', '+5V'],
        'quench_on': {'corridor_weight': 20.0},
        'signal': 'health_bus_foreign_crossings',
        'guard': ('crossings', 'hpwl'),
        'expect': 'regress',
        'rejected': True,
        'why': ('MECHANISM: the term buys its signal with intent-zone '
                'containment. Both guards improve and the re-derived '
                'bus_foreign_crossings improves too, but the ON arm leaves '
                'more members outside their block zone, so the row marks '
                'REGRESS on intent errors rather than on its signal. AT HEAD '
                'that direction is decided by the hard pad+drill legality '
                'layer: quench(pad_legality=False) on this board sends the '
                'signal the other way -- the direction recorded at 82dbf662 '
                '(2026-08-03) -- and leaves the OFF arm byte-identical. That '
                'makes the layer sufficient to decide the direction TODAY; it '
                'does not establish what changed historically, and this method '
                'cannot, because corridor_weight does not exist on the branch '
                'that introduced the layer. See docs/placement-optimization.md. '
                'Numbers: tests/placement_ab_baseline.json.'),
    },
    {
        'name': 'corridor-orangecrab',
        'board': 'orangecrab_ext_pll.kicad_pcb',
        'corridors': [{'name': 'ram_d', 'nets': ['RAM_D*'], 'width_mm': 6.0},
                      {'name': 'ram_a', 'nets': ['RAM_A*'], 'width_mm': 6.0}],
        'ignore_nets': ['GND', '+3V3', '+1V1', 'VCC*'],
        'quench_on': {'corridor_weight': 20.0},
        'signal': 'health_bus_foreign_crossings',
        'guard': ('crossings', 'hpwl'),
        'expect': 'improve',
        'rejected': True,
        'why': ('MECHANISM: the signal and both guards improve here. This row '
                'has flipped its mark once already, which is the change '
                'detector doing its job -- the flip was blamed on the '
                'pad+drill legality layer, but measured at HEAD that layer is '
                'INERT on this board (toggling quench(pad_legality=...) moves '
                'no number), so the cause is not established. It is '
                'ulx3s where that layer decides the direction. '
                'Numbers: tests/placement_ab_baseline.json.'),
    },
    {
        'name': 'corridor-coldfire',
        'board': 'kit-dev-coldfire-xilinx_5213.kicad_pcb',
        'corridors': [{'name': 'an', 'nets': ['AN*'], 'width_mm': 8.0},
                      {'name': 'bdm', 'nets': ['DDAT*', 'PST*'],
                       'width_mm': 8.0}],
        'ignore_nets': ['GND', 'VCC*', '+3.3V', '+5V'],
        'quench_on': {'corridor_weight': 20.0},
        'signal': 'health_bus_foreign_crossings',
        'guard': ('crossings', 'hpwl'),
        'expect': 'improve',
        'rejected': True,
        'why': ('MECHANISM: signal and both guards improve. This row was '
                'kept BECAUSE it was once the only board where the term '
                'helped -- and it is no longer the dissenter, which is the '
                'point: a term that helps on one board of three is not a term, '
                'WHICH board disagrees is not stable, and deleting whichever '
                'row currently disagrees is how a one-in-three result becomes '
                'folklore. Numbers: tests/placement_ab_baseline.json.'),
    },

    # --- #702: the declared-intent gate ------------------------------------
    #
    # ON TRIAL on purpose: no `expect`, so `gate()`'s >=3-DISTINCT-BOARDS rule
    # actually runs on this term instead of being skipped the way a pinned row
    # skips it. The marks get pinned from the run that measures them, in the
    # same commit, never from this comment.
    #
    # ON THE CIRCULARITY, STATED RATHER THAN HIDDEN. #701 deliberately made the
    # grader and the enforcer ONE implementation, so for these three rules the
    # file's usual independence premise does not hold. It is still not a
    # tautology, and the reason is measurable: the gate is MONOTONE and never
    # repairs, so it can LOSE to an unconstrained quench that happened to
    # improve the same count. The `zone_exclusive` row is exactly that case.
    # What makes these rows worth running is the GUARDS -- a constraint that
    # buys containment by wrecking crossings or hpwl is a bad trade, and that
    # is what `crossings`, `hpwl` and `intent_errors_other` are here to catch.
    {
        'name': 'intent-ulx3s',
        'board': 'ulx3s.kicad_pcb',
        'corridors': [],
        'ignore_nets': ['GND', '+3V3', '+5V'],
        'quench_on': {'intent_gate': ROW_INTENT},
        'signal': 'intent_errors_enforced',
        'guard': ('crossings', 'hpwl', 'intent_errors_other'),
        'expect': 'regress',
        'why': ('MECHANISM: the seed grades clean and the quench manufactures '
                'every zone_containment error, so the gate has something real '
                'to prevent -- and it prevents all of them. The mark is '
                'REGRESS on the GUARDS, not on the signal: a hard constraint '
                'removes poses from the search, so the objective it is '
                'overruling gets worse. That is the trade being bought, and '
                'pinning it here is what makes the PRICE a change detector '
                'rather than a footnote. ulx3s emits 4 disjoint zones, one '
                'B-side, so the side path is exercised too.'),
    },
    {
        'name': 'intent-orangecrab',
        'board': 'orangecrab_ext_pll.kicad_pcb',
        'corridors': [],
        'ignore_nets': ['GND', '+3V3', '+5V'],
        'quench_on': {'intent_gate': ROW_INTENT},
        'signal': 'intent_errors_enforced',
        'guard': ('crossings', 'hpwl', 'intent_errors_other'),
        'expect': 'improve',
        'why': ('MECHANISM: same as intent-ulx3s but thinner, and a SECOND '
                'board -- which is what the >=3-board rule is about. Here the '
                'constraint costs nothing: both guards improve as well, so a '
                'gated search is not inherently worse, it depends on whether '
                'the poses it removes were load-bearing.'),
    },
    {
        'name': 'intent-rp2350',
        'board': 'rp2350_fpga_eensy_prePlane.kicad_pcb',
        'corridors': [],
        'quench_base': {'max_displacement': 10.0},
        'quench_on': {'intent_gate': ROW_INTENT},
        'signal': 'intent_errors_enforced',
        'guard': ('crossings', 'hpwl', 'intent_errors_other'),
        'expect': 'regress',
        'why': ('MECHANISM: the CHEAP row (61 parts), and the corpus-scale '
                'test of the monotone FREEZE. This board SEEDS with a '
                'containment error, and the ON arm holds that count instead '
                'of driving it to zero -- which is the whole contract: the '
                'gate prevents a walk-out, it does not repair one. A run that '
                'showed this signal reaching 0 would mean the gate had '
                'started repairing and the contract had changed. It also '
                'carries the known CONTAINER footprint, so the gate is '
                'exercised beside that exemption.'),
    },
    {
        'name': 'intent-exclusive-coldfire',
        'board': 'kit-dev-coldfire-xilinx_5213.kicad_pcb',
        'corridors': [],
        'ignore_nets': ['GND', 'VCC*', '+3.3V', '+5V'],
        'zone_flags': {'exclusive': True},
        'quench_on': {'intent_gate': ROW_INTENT},
        'signal': 'intent_errors_enforced',
        'guard': ('crossings', 'hpwl', 'intent_errors_other'),
        'expect': 'regress',
        'why': ('MECHANISM: the row that DISAGREES, and the only corpus-scale '
                'zone_exclusive coverage. Its signal is NEUTRAL -- the gate '
                'holds the seed count and the unconstrained arm reaches the '
                'same number -- so the mark is decided entirely by the '
                'crossings guard. That is the honest shape of a constraint '
                'that costs something and buys nothing HERE, and the row is '
                'kept for exactly that: a term that helps on one board of '
                'four is not a term, and deleting the dissenting row is how '
                'that becomes folklore.'),
    },
]

QUENCH_BASE = dict(
    max_displacement=3.0, step=1.0, grid_step=0.1, clearance=0.2,
    board_edge_clearance=0.55, crossing_penalty=30.0, length_weight=0.3,
    halo_base=0.5, halo_coef=0.15, halo_weight=2.0, edge_halo=2.0,
    edge_weight=2.0, max_passes=4, verbose=False)


def _intent_for(board_path, corridors, workdir, zone_flags=None):
    """An intent for `board_path` with `corridors` declared.

    Emitted from the board itself rather than hand-written, so the blocks the
    health signals need are the ones that board actually has, and the row only
    has to state the bus.

    `zone_flags` is applied BY RULE to every block that has a zone, never to a
    hand-picked list: a row that named its own members would be fitted to the
    arrangement it is measuring.

    Keep-outs are deliberately NOT injectable here. `emit_intent` writes
    `keepouts: []` by design (a keep-out is a mechanical fact and cannot be
    read off a board), so any corpus keep-out is an invention -- and one that
    BITES is one placed where the OFF arm happens to walk a part, i.e. fitted
    to the OFF arm and invalidated the next time it moves. The keep-out half
    is measured in tests/test_702_quench_intent_gate.py on hand-written boards
    where the answer is a theorem.
    """
    from kicad_parser import parse_kicad_pcb
    from placement import floorplan
    path = os.path.join(workdir, 'intent.json')
    doc = floorplan.emit_intent(parse_kicad_pcb(board_path), board_path)
    if corridors:
        # Guarded: an unconditional assignment plants an empty `bus_corridors`
        # on a row that declares none, which makes
        # `health_bus_foreign_crossings` a measurement of nothing rather than
        # an absent key.
        doc['health'] = dict(doc.get('health') or {})
        doc['health']['bus_corridors'] = corridors
    if zone_flags:
        for b in doc['blocks']:
            if b.get('zone'):
                b.update(zone_flags)
    with open(path, 'w') as fh:
        json.dump(doc, fh, indent=2)
    return floorplan.load_intent(path)


def _run(board_path, out_path, intent, quench_kw, group_sources=GROUP_SOURCES):
    """One quench + write + independent grade. Returns the measured row.

    `group_sources` is NOT optional in spirit, only in signature. Every block
    `_intent_for` emits carries a `group` key (floorplan.emit_intent), and
    `resolve_blocks` only derives groups when it is given sources -- so a grade
    at the `()` default resolves EVERY block to nothing and reports it as
    `block_unresolved`. Measured before this was passed (#702): ulx3s 10
    errors, orangecrab 6, coldfire 2, all of them `block_unresolved`, all of
    them ZERO at ('kicad', 'sheet'). That is the instrument misreading its own
    fixture, not a finding about the board, and it made `intent_errors` a
    column that could not see the rule this file exists to measure.
    """
    from kicad_parser import parse_kicad_pcb
    from placement.quench import quench
    from placement.writer import write_placed_output
    from placement import floorplan

    pcb = parse_kicad_pcb(board_path)
    metrics = {}
    t0 = time.time()
    placements = quench(pcb, pcb_file=board_path, metrics_out=metrics,
                        **quench_kw)
    write_placed_output(board_path, out_path, placements)
    for ext in ('.kicad_pro', '.kicad_dru'):
        src = os.path.splitext(board_path)[0] + ext
        if os.path.exists(src):
            shutil.copy2(src, os.path.splitext(out_path)[0] + ext)

    # The independent half: parse what was WRITTEN and grade it. The corridors
    # here come from the final poses, not from the frozen model the optimizer
    # minimised against.
    graded = parse_kicad_pcb(out_path)
    result = floorplan.grade(intent, graded, out_path, with_health=True,
                             group_sources=group_sources)
    summary = floorplan.summary(result)
    after = metrics.get('after') or {}
    # ERRORS only, by rule. `summary()['violations_by_rule']` counts warnings
    # too, and the mark turns on the ERROR count -- a breakdown that did not
    # sum to `intent_errors` would explain the wrong number.
    by_rule = {}
    for v in result.errors:
        by_rule[v.rule] = by_rule.get(v.rule, 0) + 1
    # #702: split the error count by whether the quench gate can ENFORCE the
    # rule. Raw `intent_errors` mixes the rules a term can move with the ones
    # it cannot, so a row signalling on the total is diluted by whatever else
    # the emitted intent happens to find. The enforced set is imported from
    # the engine, never re-typed here: a rule the engine starts enforcing
    # enters this signal automatically, and one removed to flatter a row trips
    # `test_702_quench_intent_gate.py`'s arm M rather than passing quietly.
    from placement.quench import INTENT_ENFORCED_RULES
    enforced = sum(n for r, n in by_rule.items()
                   if r in INTENT_ENFORCED_RULES)
    gate = metrics.get('intent_gate')
    return {
        'seconds': round(time.time() - t0, 1),
        'crossings': after.get('crossings'),
        # NOT `or 0.0`: a key the optimizer stopped reporting would be recorded
        # as a real measurement of zero, which defeats record_for's refusal and
        # reads as an enormous improvement.
        'hpwl': None if after.get('hpwl') is None
                else round(float(after['hpwl']), 2),
        'corridor_cut': None if after.get('corridor_cut') is None
                        else round(float(after['corridor_cut']), 2),
        'health_bus_foreign_crossings':
            summary.get('health_bus_foreign_crossings'),
        # NOT `health_blocks_displaced`: routability.health only computes
        # that when the intent declares `health.block_displacement_mm`, which
        # `_intent_for` does not, so it was None on every board and every run
        # -- a column of evidence that was never measured. The max is
        # threshold-free and always live.
        'health_block_displacement_max_mm':
            summary.get('health_block_displacement_max_mm'),
        'intent_errors': summary.get('errors'),
        'intent_errors_by_rule': by_rule,
        'intent_errors_enforced': enforced,
        'intent_errors_other': (summary.get('errors') or 0) - enforced,
        # 0 when a gate was built and refused nothing; None when NO gate was
        # built at all. The distinction is the point -- see quench.py.
        'intent_gate_rejected': None if gate is None else gate['rejected'],
    }


def _moved_rules(off, on):
    """`zone_containment 4 -> 7` for every error rule whose count changed."""
    a = off.get('intent_errors_by_rule') or {}
    b = on.get('intent_errors_by_rule') or {}
    return [f"{k} {a.get(k, 0)} -> {b.get(k, 0)}"
            for k in sorted(set(a) | set(b)) if a.get(k, 0) != b.get(k, 0)]


def _verdict(off, on, row):
    """Direction on one board. Returns (mark, notes)."""
    key = row['signal']
    a, b = off.get(key), on.get(key)
    notes = []
    if a is None or b is None:
        return 'skip', [f"{key} not measured on this board"]
    for g in row['guard']:
        ga, gb = off.get(g), on.get(g)
        if ga is not None and gb is not None and gb > ga + 1e-9:
            notes.append(f"GUARD {g} worsened {ga} -> {gb}")
    # PAIRED, not absolute. Both runs quench the board, so both walk parts out
    # of the zones the emitted intent recorded; grading the ON run against zero
    # errors would mark every row a regression for a reason the flag did not
    # cause. Only errors the flag ADDS are its fault.
    ea, eb = off.get('intent_errors') or 0, on.get('intent_errors') or 0
    if eb > ea:
        # NAME the rules that moved. A mark resting on an unattributed error
        # count is what let #694's inverted row keep reading as an intact
        # finding: the verdict came from a criterion nothing printed.
        moved = _moved_rules(off, on)
        detail = f" ({'; '.join(moved)})" if moved else ""
        notes.append(f"intent errors {ea} -> {eb}{detail}")
    if notes:
        return 'regress', notes
    if b < a:
        return 'improve', [f"{key} {a} -> {b}"]
    if b > a:
        return 'regress', [f"{key} {a} -> {b}"]
    return 'neutral', [f"{key} unchanged at {a}"]


def run_row(row, workdir):
    board = os.path.join(BOARDS, row['board'])
    if not os.path.exists(board):
        print(f"  SKIP {row['name']}: no {row['board']} in kicad_files/")
        return 'skip', [], None, None
    d = os.path.join(workdir, row['name'])
    os.makedirs(d, exist_ok=True)
    intent = _intent_for(board, row['corridors'], d, row.get('zone_flags'))

    kw_off = dict(QUENCH_BASE)
    kw_off.update(row.get('quench_base') or {})
    kw_off['ignore_nets'] = list(row.get('ignore_nets') or ())
    kw_on = dict(kw_off)
    kw_on.update(row['quench_on'])
    # #702: the row declares the intent it wants gated with a sentinel, so the
    # table stays plain data and `_intent_for` stays the only place an intent
    # is built. The OFF arm must NOT get one, or "off" would mean "resolved
    # and then ignored" rather than "the engine that shipped".
    if kw_on.get('intent_gate') is ROW_INTENT:
        from kicad_parser import parse_kicad_pcb
        from placement import floorplan
        kw_on['intent_gate'], _probs = floorplan.resolve_intent_gate(
            intent, parse_kicad_pcb(board), GROUP_SOURCES)
    # The ON run needs the corridors the flag prices; the OFF run must NOT get
    # them, or "off" would mean "built and multiplied by zero" rather than
    # "the objective that shipped".
    if 'corridor_weight' in row['quench_on']:
        kw_on['corridor_specs'] = row['corridors']
    # A row that states no difference measures nothing, and reads exactly like
    # a flag that never reached the engine.
    if kw_on == kw_off:
        raise AssertionError(
            f"{row['name']}: quench_on {row['quench_on']} leaves the ON kwargs "
            f"identical to OFF -- the row would measure the same run twice")

    off = _run(board, os.path.join(d, 'off.kicad_pcb'), intent, kw_off)
    on = _run(board, os.path.join(d, 'on.kicad_pcb'), intent, kw_on)
    mark, notes = _verdict(off, on, row)
    expected = row.get('expect')
    tag = mark.upper()
    if expected and mark == expected:
        tag = f"{mark.upper()} (as measured)"
    elif expected:
        tag = f"{mark.upper()} != expected {expected.upper()}"
    print(f"  {row['name']:<24} {tag:<32} "
          f"({off['seconds']}s / {on['seconds']}s)")
    for k in ('crossings', 'hpwl', row['signal']):
        print(f"      {k:<32} {off.get(k)!s:>12} -> {on.get(k)!s:>12}")
    # Printed apart from the real deltas: OFF is deliberately given no
    # corridor, so this pair is not a before/after of the same quantity.
    print(f"      {'corridor_cut':<32} {off.get('corridor_cut')!s:>12} -> "
          f"{on.get('corridor_cut')!s:>12}   [OFF builds no corridor -- not a "
          f"comparison]")
    for n in notes:
        print(f"      {n}")
    if expected and mark != expected and row.get('why'):
        print(f"      recorded reason: {row['why']}")
    return mark, notes, off, on


# --- the committed baseline ------------------------------------------------
#
# Mirrors tests/stress/corpus_noop_sweep.py: `--baseline` reads the committed
# expectation, `--baseline ""` skips, `--write-baseline` re-records and returns
# WITHOUT comparing, so recording can never fail -- the burden is on the human
# to read the table first.

def _sign(a, b, tol=0.0):
    """-1 / 0 / +1 for the move from `a` to `b` (lower is better everywhere)."""
    if a is None or b is None:
        return None
    d = b - a
    if abs(d) <= tol:
        return 0
    return 1 if d > 0 else -1


BASELINE_KEYS = BASELINE_INT_KEYS + BASELINE_FLOAT_KEYS + BASELINE_DICT_KEYS


def record_for(row, mark, off, on):
    """The serializable evidence for one row.

    Refuses a measurement that is MISSING a compared key rather than writing a
    baseline with a hole in it. A key that quietly stops being produced is the
    same failure as a number that quietly inverts: the evidence still looks
    complete. (`health_blocks_displaced` was None on every board for months
    because nothing ever asked whether it had a value.) A key present and None
    is fine -- that is a measurement, and the comparator handles it.
    """
    def keep(arm, d):
        missing = [k for k in BASELINE_KEYS if k not in (d or {})]
        if missing:
            raise AssertionError(
                f"{row['name']}: the {arm} measurement is missing "
                f"{', '.join(missing)} -- _run no longer produces "
                f"{'it' if len(missing) == 1 else 'them'}, or the compared-key "
                f"lists and _run have drifted apart")
        return {k: v for k, v in d.items() if k in BASELINE_KEYS}
    return {'board': row['board'], 'mark': mark,
            'off': keep('OFF', off), 'on': keep('ON', on)}


def _num(v):
    """`v` as a float, or None if it is not a number this can compare."""
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        return None
    return float(v)


def compare_baseline(current, expected, float_tol=1e-6, scope=None,
                     known=None):
    """Compare measured records against the committed baseline.

    Returns a list of problem strings. Every class is fatal; they are kept
    distinct because they mean different things:

      INVERTED -- a key's direction REVERSED (both moves non-zero, opposite
                  ways), or the row's mark changed. This is #694: the finding
                  the row records is no longer the finding the code produces.
      DRIFT    -- the value moved without reversing. Integers compare exactly;
                  floats at a relative tolerance, per arm.
      ORPHAN   -- the baseline carries a row `ROWS` no longer declares. Never
                  scoped away: deleting the row that disagrees is the failure
                  this whole file exists to prevent, and a silent ORPHAN would
                  let it happen while printing "the baseline matches".
      MISSING  -- a row this run was asked for but did not measure.
      NEW ROW  -- a row measured that the baseline has never seen.
      MALFORMED -- the baseline is not shaped like a baseline. Reported, not
                  raised: crashing after a nine-minute table is not a verdict.

    `scope` is the set of row names this run was asked to run (`--row` narrows
    it), so a deliberately narrow run does not report the rest as MISSING.
    `known` is every name `ROWS` declares; a baseline row outside it is an
    ORPHAN regardless of scope.

    Direction is classified with NO tolerance, and `float_tol` decides only
    whether a same-direction value MOVED. Folding the tolerance into the
    direction test made a reversal inside the band read as DRIFT, and made a
    flat baseline that started moving read as INVERTED.
    """
    problems = []
    if not isinstance(expected, dict):
        return [f"MALFORMED baseline: expected an object of rows, got "
                f"{type(expected).__name__}"]
    for name in sorted(current):
        cur = current[name]
        exp = expected.get(name)
        if exp is None:
            problems.append(f"NEW ROW {name}: not in the baseline")
            continue
        if not isinstance(exp, dict):
            problems.append(f"MALFORMED baseline row {name}: expected an "
                            f"object, got {type(exp).__name__}")
            continue
        if cur.get('mark') != exp.get('mark'):
            problems.append(
                f"INVERTED {name}: mark {exp.get('mark')} -> {cur.get('mark')}")
        c_off, c_on = cur.get('off') or {}, cur.get('on') or {}
        e_off, e_on = exp.get('off') or {}, exp.get('on') or {}
        for key in BASELINE_INT_KEYS + BASELINE_FLOAT_KEYS:
            tol = 0.0 if key in BASELINE_INT_KEYS else float_tol
            raw = ((c_off.get(key), e_off.get(key)),
                   (c_on.get(key), e_on.get(key)))
            pairs = [(_num(c), _num(e)) for c, e in raw]
            # Each ARM is compared on its own. Bailing on the whole key the
            # moment one arm was None left the other arm's value permanently
            # uncompared -- a dead evidence column, which is the bug that put
            # `health_blocks_displaced` at None for months.
            for arm, (c, e), (rc, re_) in zip(('off', 'on'), pairs, raw):
                if (c is None) != (e is None):
                    problems.append(
                        f"DRIFT {name}.{arm}.{key}: baseline {re_!r}, "
                        f"measured {rc!r}")
                elif c is not None and abs(c - e) > max(abs(e), 1.0) * tol:
                    problems.append(
                        f"DRIFT {name}.{arm}.{key}: baseline {re_}, "
                        f"measured {rc}")
            (ca, ea), (cb, eb) = pairs
            if None in (ca, cb, ea, eb):
                continue
            cd, ed = _sign(ea, eb), _sign(ca, cb)
            if cd and ed and cd != ed:
                problems.append(
                    f"INVERTED {name}.{key}: baseline {e_off.get(key)} -> "
                    f"{e_on.get(key)}, measured {c_off.get(key)} -> "
                    f"{c_on.get(key)}")
        for key in BASELINE_DICT_KEYS:
            for arm, c_arm, e_arm in (('off', c_off, e_off),
                                      ('on', c_on, e_on)):
                c = c_arm.get(key) or {}
                e = e_arm.get(key) or {}
                if not isinstance(c, dict) or not isinstance(e, dict):
                    problems.append(f"MALFORMED {name}.{arm}.{key}: expected "
                                    f"an object")
                    continue
                for rule in sorted(set(c) | set(e)):
                    if c.get(rule, 0) != e.get(rule, 0):
                        problems.append(
                            f"DRIFT {name}.{arm}.{key}[{rule}]: baseline "
                            f"{e.get(rule, 0)}, measured {c.get(rule, 0)}")
    for name in sorted(expected):
        if name in current:
            continue
        if known is not None and name not in known:
            problems.append(
                f"ORPHAN {name}: the baseline records this row and ROWS no "
                f"longer declares it. Deleting a row deletes its evidence -- "
                f"re-record deliberately, do not let it lapse.")
        elif scope is None or name in scope:
            problems.append(
                f"MISSING {name}: in the baseline, not measured in this run")
    return problems


# --- the gate --------------------------------------------------------------

def gate(rows, marks):
    """The pass rule. Pure: no I/O, so `_self_test` can reach every branch."""
    lines = []
    pinned = {r['name']: r['expect'] for r in rows if r.get('expect')}
    # A `rejected` row is NOT on trial, whatever else it carries. The banner
    # below says so out loud; leaving such a row in `trial` made the code
    # contradict its own printed sentence.
    trial = [r for r in rows if not r.get('expect') and not r.get('rejected')]
    mismatched = [n for n, e in pinned.items() if marks.get(n) != e]
    rejected = [r['name'] for r in rows if r.get('rejected')]

    ok = not mismatched
    if rejected:
        lines.append(
            f"rejected: {len(rejected)} row(s) record a term that was tried "
            f"and NOT adopted ({', '.join(rejected)}). They are judged "
            f"against their recorded marks, not against 'the term helps'.")
    if trial:
        # Judged per BOARD, not per row. Counting rows lets three rows on one
        # board satisfy a rule that says three boards -- and `--row` makes
        # running exactly one of them the convenient path.
        by_board = {}
        for r in trial:
            m = marks.get(r['name'])
            if m == 'skip' or m is None:
                continue
            by_board.setdefault(r['board'], []).append(m)
        boards = sorted(by_board)
        b_reg = [b for b in boards if 'regress' in by_board[b]]
        b_imp = [b for b in boards
                 if 'improve' in by_board[b] and b not in b_reg]
        n = len(boards)
        if n < MIN_TRIAL_BOARDS:
            ok = False
            lines.append(
                f"REFUSED: a term on trial is judged on >= {MIN_TRIAL_BOARDS} "
                f"DISTINCT boards; this run judged {n} "
                f"({', '.join(boards) if boards else 'none'}). Add rows on "
                f"other boards, or run the whole table.")
        ok = ok and not b_reg and len(b_imp) >= max(1, n - 1)
        lines.append(f"on trial: improved {len(b_imp)}/{n} board(s), "
                     f"regressed {len(b_reg)} "
                     f"(rule: improve >= N-1, regress == 0)")
        lines.append(f"          N={n}: a term whose per-board direction is a "
                     f"coin flip passes that rule 1 run in {2 ** n}")
    if pinned:
        lines.append(
            f"pinned:   {len(pinned) - len(mismatched)}/{len(pinned)} rows "
            f"match their measured verdict"
            + (f"; MISMATCH: {', '.join(mismatched)}" if mismatched else ""))
    return ok, lines


def _self_test():
    """Every branch of the gate and the comparator, without quenching.

    Runs at the top of EVERY invocation: it costs milliseconds, and the live
    table cannot reach any of it (all three rows are pinned, so the trial
    branch never executes on a real run). A gate whose own logic is untested
    is how a documented rule stays documentation -- which is exactly what
    #694 found: CLAUDE.md's ">= 3 boards" rule was never in the code.
    """
    def row(name, board, **kw):
        r = {'name': name, 'board': board, 'signal': 's', 'guard': (),
             'quench_on': {'x': 1}}
        r.update(kw)
        return r

    # 1. one improving trial row on one board is REFUSED on board count --
    #    the exact hole #694 names (`max(1, judged - 1) == 1` passed it).
    rows = [row('a', 'b1.kicad_pcb')]
    ok, lines = gate(rows, {'a': 'improve'})
    assert not ok, "one board must not pass"
    assert any('REFUSED' in x for x in lines), lines

    # 2. three trial rows on the SAME board is still one board.
    rows = [row(n, 'b1.kicad_pcb') for n in ('a', 'b', 'c')]
    ok, lines = gate(rows, {'a': 'improve', 'b': 'improve', 'c': 'improve'})
    assert not ok, "three rows on one board must not pass"
    assert any('REFUSED' in x for x in lines), lines

    # 3. three distinct boards, 2 improve + 1 neutral, none regress -> passes.
    rows = [row(n, f'b{i}.kicad_pcb')
            for i, n in enumerate(('a', 'b', 'c'))]
    ok, _ = gate(rows, {'a': 'improve', 'b': 'improve', 'c': 'neutral'})
    assert ok, "improve on N-1 boards with no regress must pass"

    # 3b. two neutral boards is only N-2 improved -- the N-1 rule must FAIL.
    #     Without this case, deleting the improve count entirely still passes
    #     every other assertion here.
    ok, _ = gate(rows, {'a': 'improve', 'b': 'neutral', 'c': 'neutral'})
    assert not ok, "improve on N-2 boards must fail"
    ok, _ = gate(rows, {'a': 'neutral', 'b': 'neutral', 'c': 'neutral'})
    assert not ok, "an inert term must fail"

    # 4. any regress fails, however many improve.
    ok, _ = gate(rows, {'a': 'improve', 'b': 'improve', 'c': 'regress'})
    assert not ok, "a regressing board must fail"

    # 5. a skipped board does not count toward N (and must not pass at 2).
    ok, lines = gate(rows, {'a': 'improve', 'b': 'improve', 'c': 'skip'})
    assert not ok and any('REFUSED' in x for x in lines), lines

    # 6. pinned rows are judged against their recorded mark.
    rows = [row('p', 'b1.kicad_pcb', expect='regress', rejected=True)]
    assert gate(rows, {'p': 'regress'})[0]
    assert not gate(rows, {'p': 'improve'})[0]

    # --- the comparator ---
    base = {'r': {'board': 'b.kicad_pcb', 'mark': 'regress',
                  'off': {'crossings': 100, 'hpwl': 10.0,
                          'intent_errors': 5, 'intent_errors_enforced': 5,
                          'intent_errors_other': 0,
                          'intent_errors_by_rule': {'zone_containment': 5}},
                  'on': {'crossings': 90, 'hpwl': 9.0,
                         'intent_errors': 7, 'intent_errors_enforced': 7,
                         'intent_errors_other': 0,
                         'intent_errors_by_rule': {'zone_containment': 7}}}}
    same = json.loads(json.dumps(base))
    assert compare_baseline(same, base) == [], compare_baseline(same, base)

    # 7. a reversed direction is INVERTED, not DRIFT. This is #694 itself.
    inv = json.loads(json.dumps(base))
    inv['r']['on']['crossings'] = 110
    probs = compare_baseline(inv, base)
    assert any(p.startswith('INVERTED r.crossings') for p in probs), probs

    # 8. a changed mark is INVERTED.
    m = json.loads(json.dumps(base))
    m['r']['mark'] = 'improve'
    assert any('INVERTED r: mark' in p for p in compare_baseline(m, base))

    # 9. integers compare exactly; direction unchanged, so DRIFT not INVERTED.
    d = json.loads(json.dumps(base))
    d['r']['on']['crossings'] = 91
    probs = compare_baseline(d, base)
    assert any(p.startswith('DRIFT r.on.crossings') for p in probs), probs
    assert not any('INVERTED r.crossings' in p for p in probs), probs

    # 10. floats: inside the tolerance is silence, outside it is DRIFT.
    f = json.loads(json.dumps(base))
    f['r']['on']['hpwl'] = 9.0 + 1e-9
    assert compare_baseline(f, base, float_tol=1e-6) == []
    f['r']['on']['hpwl'] = 9.05
    assert any(p.startswith('DRIFT r.on.hpwl')
               for p in compare_baseline(f, base, float_tol=1e-6))

    # 10b. --float-tol is for floats ONLY. A loose tolerance must not start
    #      waving integer counts through: "crossings 90 -> 94" is a real move
    #      at every tolerance, and the --help promises exactly that.
    t = json.loads(json.dumps(base))
    t['r']['on']['hpwl'] = 9.0001
    t['r']['on']['crossings'] = 94
    probs = compare_baseline(t, base, float_tol=0.05)
    assert not any('r.hpwl' in p for p in probs), probs
    assert any(p.startswith('DRIFT r.on.crossings') for p in probs), probs

    # 11. an error rule whose count moved is named BY RULE.
    r = json.loads(json.dumps(base))
    r['r']['on']['intent_errors_by_rule'] = {'zone_containment': 7,
                                             'keepout': 1}
    probs = compare_baseline(r, base)
    assert any('intent_errors_by_rule[keepout]' in p for p in probs), probs

    # 12. scope: a baseline row outside this run is not reported missing.
    assert compare_baseline({}, base, scope=set()) == []
    assert any(p.startswith('MISSING r')
               for p in compare_baseline({}, base, scope={'r'}))

    # 13. a row the baseline has never seen is named, not silently accepted.
    assert any(p.startswith('NEW ROW r') for p in compare_baseline(base, {}))

    # --- _verdict and the record it produces ---
    vrow = {'name': 'v', 'board': 'b.kicad_pcb', 'quench_on': {'x': 1},
            'signal': 'health_bus_foreign_crossings',
            'guard': ('crossings', 'hpwl')}
    voff = {'crossings': 100, 'hpwl': 10.0, 'corridor_cut': 0.0, 'seconds': 1,
            'health_bus_foreign_crossings': 62,
            'health_block_displacement_max_mm': 17.95,
            'intent_errors': 14,
            'intent_errors_enforced': 4, 'intent_errors_other': 10,
            'intent_gate_rejected': None,
            'intent_errors_by_rule': {'block_unresolved': 10,
                                      'zone_containment': 4}}
    von = {'crossings': 90, 'hpwl': 9.0, 'corridor_cut': 800.0, 'seconds': 1,
           'health_bus_foreign_crossings': 55,
           'health_block_displacement_max_mm': 18.09,
           'intent_errors': 17,
           'intent_errors_enforced': 7, 'intent_errors_other': 10,
           'intent_gate_rejected': None,
           'intent_errors_by_rule': {'block_unresolved': 10,
                                     'zone_containment': 7}}

    # 14. #694 in miniature: signal and both guards improve, yet the mark is
    #     REGRESS -- and the note must NAME the rule that decided it, or the
    #     verdict is again resting on a criterion nobody printed.
    mark, notes = _verdict(voff, von, vrow)
    assert mark == 'regress', (mark, notes)
    assert any('zone_containment 4 -> 7' in n for n in notes), notes

    # 15. a guard that worsens is named as a guard, not as the signal.
    g = dict(von, crossings=101)
    mark, notes = _verdict(voff, g, vrow)
    assert mark == 'regress' and any('GUARD crossings' in n for n in notes)

    # 16. with the errors equal, the signal decides.
    e = dict(von, intent_errors=14,
             intent_errors_by_rule={'block_unresolved': 10,
                                    'zone_containment': 4})
    assert _verdict(voff, e, vrow)[0] == 'improve'
    assert _verdict(voff, dict(e, health_bus_foreign_crossings=62),
                    vrow)[0] == 'neutral'
    assert _verdict(voff, dict(e, health_bus_foreign_crossings=70),
                    vrow)[0] == 'regress'

    # 17. the record carries every compared key and NOTHING that is not
    #     comparable -- `seconds` is wall-clock, and `corridor_cut` has no OFF
    #     arm to compare against.
    rec = record_for(vrow, 'regress', voff, von)
    for arm in ('off', 'on'):
        assert 'seconds' not in rec[arm] and 'corridor_cut' not in rec[arm], rec
        for k in BASELINE_INT_KEYS + BASELINE_FLOAT_KEYS + BASELINE_DICT_KEYS:
            assert k in rec[arm], (k, rec[arm])
    assert compare_baseline({'v': rec}, {'v': rec}) == []

    # 18. a measurement that has LOST a compared key refuses, instead of
    #     recording a baseline with a hole in it. Both an int key and a dict
    #     key, and both arms.
    for drop, arm in (('intent_errors', voff), ('intent_errors_by_rule', voff),
                      ('hpwl', von)):
        thin = {k: v for k, v in arm.items() if k != drop}
        a, b = (thin, von) if arm is voff else (voff, thin)
        try:
            record_for(vrow, 'regress', a, b)
        except AssertionError as exc:
            assert drop in str(exc), (drop, exc)
        else:
            raise AssertionError(f'a missing {drop} must refuse')

    # 19. the record must say what it was GIVEN. Comparing a record against
    #     itself (case 17) passes for any consistent corruption -- swapped
    #     arms, a hardcoded mark -- which is #694 in its purest form.
    assert rec['mark'] == 'regress' and rec['board'] == vrow['board'], rec
    assert rec['off']['crossings'] == voff['crossings'], rec
    assert rec['on']['crossings'] == von['crossings'], rec
    assert rec['off'] != rec['on'], 'the arms must not collapse'

    # 20. INVERTED must be pinned on a FLOAT key too, not only on ints. hpwl is
    #     the key whose inversion #694 recorded.
    fi = json.loads(json.dumps(base))
    fi['r']['on']['hpwl'] = 11.0                       # baseline fell, this rose
    probs = compare_baseline(fi, base)
    assert any(p.startswith('INVERTED r.hpwl') for p in probs), probs

    # 21. direction is classified with NO tolerance, so a reversal INSIDE the
    #     float band is still INVERTED, and a FLAT baseline that starts moving
    #     is DRIFT -- not "inverted", because nothing reversed. corridor-
    #     coldfire's recorded intent_errors is flat today, so this is live.
    flat = {'r': {'board': 'b', 'mark': 'improve',
                  'off': {'intent_errors': 2}, 'on': {'intent_errors': 2}}}
    moved = json.loads(json.dumps(flat))
    moved['r']['on']['intent_errors'] = 3
    probs = compare_baseline(moved, flat)
    assert any(p.startswith('DRIFT r.on.intent_errors') for p in probs), probs
    assert not any('INVERTED r.intent_errors' in p for p in probs), probs

    # 22. one arm None must NOT stop the other arm being compared. Bailing on
    #     the whole key is how a dead evidence column stays invisible.
    half = {'r': {'board': 'b', 'mark': 'improve',
                  'off': {'hpwl': None}, 'on': {'hpwl': 18.09}}}
    moved = json.loads(json.dumps(half))
    moved['r']['on']['hpwl'] = 999.0
    probs = compare_baseline(moved, half)
    assert any(p.startswith('DRIFT r.on.hpwl') for p in probs), probs
    # and a key that appears or disappears is named, per arm
    gone = json.loads(json.dumps(half))
    gone['r']['on']['hpwl'] = None
    assert any(p.startswith('DRIFT r.on.hpwl')
               for p in compare_baseline(gone, half))
    assert compare_baseline(half, half) == []

    # 23. a rule the baseline HAD and the run lost is a problem, not only a
    #     rule the run added.
    lost = json.loads(json.dumps(base))
    lost['r']['on']['intent_errors_by_rule'] = {}
    probs = compare_baseline(lost, base)
    assert any('intent_errors_by_rule[zone_containment]' in p
               for p in probs), probs

    # 24. ORPHAN: a baseline row that ROWS no longer declares is reported even
    #     when the run was narrowed, because deleting the row that disagrees is
    #     the failure this file exists to prevent.
    assert any(p.startswith('ORPHAN r')
               for p in compare_baseline({}, base, scope=set(), known=set()))
    assert compare_baseline({}, base, scope=set(), known={'r'}) == []

    # 25. a malformed baseline is REPORTED, not raised: crashing after a
    #     nine-minute table is not a verdict.
    for bad in ([], {'r': []}, {'r': {'mark': 'regress', 'off': None,
                                      'on': None}},
                {'r': {'mark': 'regress', 'off': {'crossings': 'x'},
                       'on': {'crossings': 'y'}}}):
        compare_baseline(json.loads(json.dumps(base)), bad)

    # 26. an UNCHANGED guard is not a regression (>= vs >).
    same_guard = dict(von, intent_errors=14,
                      intent_errors_by_rule=dict(voff['intent_errors_by_rule']),
                      crossings=voff['crossings'], hpwl=voff['hpwl'])
    assert _verdict(voff, same_guard, vrow)[0] == 'improve'

    # 27. a signal the board did not measure SKIPs; it must not raise.
    assert _verdict(voff, dict(von, health_bus_foreign_crossings=None),
                    vrow)[0] == 'skip'
    assert _verdict(dict(voff, health_bus_foreign_crossings=None), von,
                    vrow)[0] == 'skip'

    # 28. a rule present only in the ON arm is named -- the case that matters
    #     most for "NAME the rules that moved".
    only_on = dict(von, intent_errors=15,
                   intent_errors_by_rule={'block_unresolved': 10,
                                          'zone_containment': 4, 'keepout': 1})
    assert any('keepout 0 -> 1' in n
               for n in _verdict(voff, only_on, vrow)[1]), only_on

    # 29. a `rejected` row is never on trial, whatever else it carries -- the
    #     banner says so, and the code must agree.
    rej = [row(n, f'b{i}.kicad_pcb', rejected=True)
           for i, n in enumerate(('a', 'b', 'c'))]
    ok, lines = gate(rej, {'a': 'regress', 'b': 'regress', 'c': 'regress'})
    assert ok, lines
    assert not any('on trial' in x for x in lines), lines


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--row', action='append',
                   help='Run only these table rows (repeatable)')
    p.add_argument('--list', action='store_true', help='List rows and exit')
    p.add_argument('--workdir', default=None,
                   help='Where to write boards (default: a temp dir)')
    p.add_argument('--json', '--json-out', dest='json', default=None,
                   help='Write the full per-row report here (not committed)')
    p.add_argument('--baseline', default=DEFAULT_BASELINE,
                   help='Committed measurements to compare against '
                        '(default: %(default)s); "" to skip the comparison')
    p.add_argument('--write-baseline', action='store_true',
                   help='Record CURRENT measurements as the expectation. Only '
                        'after reading the table and agreeing with every row.')
    p.add_argument('--float-tol', type=float, default=1e-6,
                   help='Relative tolerance for float keys (default: '
                        '%(default)s). Integers always compare exactly.')
    p.add_argument('--self-test', action='store_true',
                   help='Run the gate/comparator logic checks and exit')
    args = p.parse_args(argv)

    # Always, and first: a broken gate now fails in a second instead of after
    # nine minutes of quenching.
    _self_test()
    if args.self_test:
        print('self-test OK')
        return 0

    if args.list:
        for r in ROWS:
            print(f"{r['name']:<24} {r['board']:<28} "
                  f"{r['quench_on']} -> {r['signal']}")
        return 0

    names = [r['name'] for r in ROWS]
    dupes = sorted({n for n in names if names.count(n) > 1})
    if dupes:
        # marks/records/pinned are all keyed by name, so a duplicate silently
        # overwrites the other row's evidence.
        print(f"duplicate row name(s) in ROWS: {', '.join(dupes)}",
              file=sys.stderr)
        return 2
    # A typo'd --row used to be dropped in silence, and the run then reported
    # PASS over whatever survived the filter -- with --write-baseline, over a
    # baseline it had just truncated.
    unknown = [n for n in (args.row or ()) if n not in names]
    if unknown:
        print(f"no such row: {', '.join(unknown)}; try --list", file=sys.stderr)
        return 2
    rows = [r for r in ROWS if not args.row or r['name'] in args.row]
    if not rows:
        print("no such row; try --list", file=sys.stderr)
        return 2
    if not 0.0 <= args.float_tol < 1.0:
        # Negative flagged identical values as DRIFT; large silenced real
        # reversals. Neither is a tolerance.
        print(f"--float-tol must be in [0, 1); got {args.float_tol}",
              file=sys.stderr)
        return 2

    workdir = args.workdir or tempfile.mkdtemp(prefix='placement_ab_')
    os.makedirs(workdir, exist_ok=True)
    print(f"A/B in {workdir}\n")

    marks, records = {}, {}
    for r in rows:
        mark, _notes, off, on = run_row(r, workdir)
        marks[r['name']] = mark
        if off is not None and on is not None:
            records[r['name']] = record_for(r, mark, off, on)

    print()
    tally = {m: [n for n, v in marks.items() if v == m]
             for m in ('improve', 'neutral', 'regress', 'skip')}
    # Printed, not dropped: a term with no effect on 3 of 4 boards must not
    # read as a clean sweep.
    for m in ('improve', 'neutral', 'regress', 'skip'):
        if tally[m]:
            print(f"{m:<9} {len(tally[m])}: {', '.join(tally[m])}")

    if args.write_baseline:
        target = args.baseline or DEFAULT_BASELINE
        # The baseline is written WHOLE, so a partial run would delete the rows
        # it did not measure -- and `--row` is the cheap path, which makes that
        # a live footgun rather than a theoretical one. Refuse instead: mixing
        # numbers from two engine states is not a measurement of either.
        unmeasured = [n for n in names if n not in records]
        if unmeasured:
            print(f"\nREFUSED to write {target}: this run measured "
                  f"{len(records)} of {len(names)} row(s); "
                  f"{', '.join(unmeasured)} did not run. Writing now would "
                  f"drop their evidence. Re-record from a full run.",
                  file=sys.stderr)
            return 2
        with open(target, 'w') as fh:
            json.dump(records, fh, indent=1, sort_keys=True)
        print(f"\nwrote baseline: {target} ({len(records)} row(s)). Read the "
              f"table above and agree with every row before committing it.")
        return 0

    ok, lines = gate(rows, marks)
    for ln in lines:
        print(ln)

    problems, compared = [], False
    if args.baseline:
        expected = None
        try:
            with open(args.baseline) as fh:
                expected = json.load(fh)
        except Exception as exc:                       # noqa: BLE001
            print(f"baseline unreadable: {exc}")
        if expected is None:
            # NOT a pass. Deleting, renaming or truncating the baseline would
            # otherwise remove the entire #694 protection behind exit 0.
            # `--baseline ""` is the deliberate way to run without it.
            ok = False
            print(f"no usable baseline at {args.baseline}. Read the table "
                  f"above, then --write-baseline -- or --baseline \"\" to run "
                  f"without the comparison on purpose.")
        else:
            compared = True
            problems = compare_baseline(records, expected,
                                        float_tol=args.float_tol,
                                        scope={r['name'] for r in rows},
                                        known=set(names))
    else:
        print('baseline comparison SKIPPED (--baseline "")')

    if problems:
        ok = False
        print(f"\nbaseline: {len(problems)} problem(s) vs "
              f"{os.path.basename(args.baseline)}")
        for pr in problems:
            print(f"  {pr}")
        print("  INVERTED means the recorded finding no longer holds -- read "
              "it before re-recording.\n"
              "  An intended change re-records with --write-baseline, in the "
              "same commit as the change.")
    elif compared:
        print(f"baseline: {len(records)} row(s) match "
              f"{os.path.basename(args.baseline)}")

    # Written LAST, so the report carries the verdict rather than only the
    # numbers that led to it.
    if args.json:
        with open(args.json, 'w') as fh:
            json.dump({'rows': records, 'marks': marks, 'problems': problems,
                       'pass': bool(ok)}, fh, indent=1, sort_keys=True)
        print(f"wrote report: {args.json}")

    print(f"\n{'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
