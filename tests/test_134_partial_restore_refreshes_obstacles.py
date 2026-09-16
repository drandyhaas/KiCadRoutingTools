"""A #134 piece-level restore must tell the obstacle map about the copper.

`refresh_net_obstacles` is the codebase's ONE contract for this, and says so:
"the single-ended loop's contract after every commit (and rip_up_net's /
restore_net's after every rip and restore), spelled once." Its #806 note records
what skipping it costs -- a stale cache entry left 30862 of 47448 cells the
board's copper owned unblocked in the map a ripped victim's reroute searched.

There are TWO implementations of the same piece-level settle:

  * `route.py`'s "Issue #134 last resort", and
  * `diff_pair_custody.run_casualty_reconcile`'s branch D,

and route.py's comment claims parity with the other ("route.py #134 parity" /
"parity with the plane tools' piece-level settle"). The casualty one refreshed;
route.py's did not, so every net it restored was INVISIBLE to the later #134
recovery laps, the casualty reconcile and net_rescue -- all of which run after
it and route against that map.

MEASURED on cparti_fpga's retry step (runs_set2, the final `--nets '*'` pass),
which takes this branch NINE times in one run:

    2867: last resort: SPIs_MISO restored 16 seg + 2 via    map not updated
    3928: recovery: re-routing SPIs_SCK, DAC_D0             searches a stale map
    4233: last resort: SPIs_SCK  restored 58 seg + 3 via    map not updated
    6056: recovery: re-routing GND, IO_0, Stage2, ...        map missing 9 nets

SPIs_MISO and SPIs_SCK shipped COLLINEAR on F.Cu at y=78.70 for ~11mm -- a dead
short KiCad reports as "Items shorting two nets", not a graze:

    SPIs_MISO  (167.60,78.70)-(154.60,78.70)
    SPIs_SCK   (165.40,78.70)-(154.40,78.70)   overlap 0.250mm

The board graded drc_real 0 at the step's INPUT and 30 at its output.

This gate is structural because the behaviour needs a full congested retry
ladder to reproduce (~20 minutes on one board). It asks the CALL GRAPH, not the
text: `refresh_net_obstacles` is named in the explanatory comment at both sites,
so a substring test would pass on the broken code it is meant to catch.
"""
import ast
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'py_router'))

FAILS = []

REFRESH = 'refresh_net_obstacles'
MARKER = 'partial_restore_134'

SITES = [
    ('py_router/route.py', "Issue #134 last resort"),
    ('py_router/diff_pair_custody.py', "casualty reconcile branch D"),
]


def check(name, cond, detail=''):
    print(f"--- {name}")
    if cond:
        print(f"  PASS{': ' + detail if detail else ''}")
    else:
        print(f"  FAIL: {detail}")
        FAILS.append(name)


def _marker_blocks(tree):
    """Every statement list that assigns `pruned['partial_restore_134']`,
    paired with the index of that assignment -- i.e. the piece-level settle's
    own block, wherever it sits."""
    out = []

    def _is_marker(stmt):
        for n in ast.walk(stmt):
            if isinstance(n, ast.Constant) and n.value == MARKER:
                return True
        return False

    for node in ast.walk(tree):
        for field in ('body', 'orelse', 'finalbody'):
            body = getattr(node, field, None)
            if not isinstance(body, list):
                continue
            for i, stmt in enumerate(body):
                if isinstance(stmt, ast.Assign) and _is_marker(stmt):
                    out.append((body, i))
    return out


def _calls_after(body, idx):
    names = set()
    for stmt in body[idx:]:
        for n in ast.walk(stmt):
            if isinstance(n, ast.Call) and isinstance(n.func, ast.Name):
                names.add(n.func.id)
    return names


def _names_after(body, idx):
    """Every identifier/attribute mentioned from `idx` on -- used to ask
    whether the restored net is REGISTERED, which is a list mutation and so has
    no call name of its own."""
    out = set()
    for stmt in body[idx:]:
        for n in ast.walk(stmt):
            if isinstance(n, ast.Attribute):
                out.add(n.attr)
            if isinstance(n, ast.Name):
                out.add(n.id)
    return out


def t_both_settles_refresh_the_map():
    root = os.path.join(os.path.dirname(__file__), '..')
    for rel, label in SITES:
        path = os.path.join(root, rel)
        if not os.path.exists(path):
            check(f't_site_exists[{rel}]', False, f'{rel} is missing')
            continue
        try:
            tree = ast.parse(open(path).read())
        except SyntaxError as exc:
            check(f't_site_parses[{rel}]', False, f'{rel} does not parse: {exc}')
            continue
        blocks = _marker_blocks(tree)
        if not blocks:
            check(f't_site_has_the_settle[{rel}]', False,
                  f"no `{MARKER}` marker in {rel} -- the piece-level settle is "
                  f"gone or renamed, so this gate is guarding nothing")
            continue
        ok = all(REFRESH in _calls_after(body, i) for body, i in blocks)
        check(f't_the_settle_refreshes_the_map[{label}]', ok,
              f'{len(blocks)} settle block(s) in {rel}, each followed by '
              f'{REFRESH}()' if ok else
              f'a settle block in {rel} commits copper without calling '
              f'{REFRESH}() -- every pass after it routes against a map that '
              f'does not know the restored copper is there')


def t_both_settles_register_the_net():
    """The copper must also be REGISTERED, or it is never stamped at all.

    `build_single_ended_obstacles` stamps foreign copper for nets in
    `routed_net_ids` (recomputed from pcb_data) or `remaining_net_ids` (from the
    cache). A net in NEITHER list is never stamped, however much copper it owns
    -- and a rip removes the net from routed_net_ids while a failed reroute
    leaves it out of both. So a settle that only sets `routed_results[nid]`
    leaves its copper structurally invisible, and refreshing the cache entry
    does NOT cover it: the cache is consulted only for remaining_net_ids.

    This is why the refresh alone was measured INERT on cparti_fpga: SPIs_MISO
    was restored at trace seq 237 and never ripped again, yet SPIs_SCK was laid
    over it at seq 284, 295 and 307 -- three passes, each blind to copper that
    had been on the board the whole time.
    """
    root = os.path.join(os.path.dirname(__file__), '..')
    for rel, label in SITES:
        path = os.path.join(root, rel)
        if not os.path.exists(path):
            continue
        try:
            tree = ast.parse(open(path).read())
        except SyntaxError:
            continue
        blocks = _marker_blocks(tree)
        if not blocks:
            continue
        ok = all({'routed_net_ids', 'remaining_net_ids'} <= _names_after(body, i)
                 for body, i in blocks)
        check(f't_the_settle_registers_the_net[{label}]', ok,
              'the restored net is put into routed_net_ids and taken out of '
              'remaining_net_ids' if ok else
              'a settle block commits copper without registering the net in '
              'routed_net_ids / remaining_net_ids -- build_single_ended_'
              'obstacles stamps neither list, so the copper is invisible to '
              'every map built afterwards')


def t_the_contract_function_still_exists():
    """Non-vacuity: if the helper were renamed, every row above would fail for
    the wrong reason, so say which failure this is."""
    try:
        from obstacle_cache import refresh_net_obstacles  # noqa: F401
        check('t_the_contract_function_still_exists', True,
              'obstacle_cache.refresh_net_obstacles is importable')
    except ImportError as exc:
        check('t_the_contract_function_still_exists', False,
              f'obstacle_cache exports no {REFRESH} ({exc}) -- the rows above '
              f'are failing because the CONTRACT moved, not because a caller '
              f'dropped it')


def t_the_gate_can_fail():
    """The detector itself, on a synthetic broken site: a settle block with no
    refresh must be reported. Without this the rows above could be passing
    because `_marker_blocks` finds nothing."""
    broken = ast.parse(
        "def f():\n"
        "    if x:\n"
        "        pruned['partial_restore_134'] = True\n"
        "        add_route_to_pcb_data(pcb, pruned)\n"
        "        results.append(pruned)\n")
    blocks = _marker_blocks(broken)
    detected = bool(blocks) and not all(
        REFRESH in _calls_after(b, i) for b, i in blocks)
    check('t_the_gate_can_fail', detected,
          'a synthetic settle block with no refresh IS reported -- the '
          'detector is not vacuous')
    fixed = ast.parse(
        "def f():\n"
        "    if x:\n"
        "        pruned['partial_restore_134'] = True\n"
        "        add_route_to_pcb_data(pcb, pruned)\n"
        "        refresh_net_obstacles(w, c, pcb, cfg, [nid])\n")
    fb = _marker_blocks(fixed)
    check('t_the_gate_accepts_a_fixed_site',
          bool(fb) and all(REFRESH in _calls_after(b, i) for b, i in fb),
          'and the same block WITH the refresh passes')


def main():
    t_the_contract_function_still_exists()
    t_the_gate_can_fail()
    t_both_settles_refresh_the_map()
    t_both_settles_register_the_net()
    print()
    if FAILS:
        print(f"{len(FAILS)} FAILURE(S): {', '.join(FAILS)}")
        return 1
    print("ALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
