"""One net name, one net id -- at every site that resolves a `(net ...)` token.

neo6502 declares `/GPIO22\\I2C1_SDA`. Its eleven segments parsed onto net 4 and
its four vias parsed onto net 0, because `extract_vias` was the ONE site that
ran `_unescape_kicad_string` before the `name_to_id` lookup while the table is
keyed on the RAW file text. The four netless barrels then sat on that net's own
copper and graded as **8 phantom DRC violations** on a board whose file was
correct -- `check_drc` and the corpus A/B both counted them.

The writer half of this was fixed as #312 on this same board (`_escape_net_name`,
so a routed segment is not written with a single backslash). The parser half was
missed, and two more sites carried the same defect in different disguises:

  * `extract_zones` unescaped before its lookup too, under a comment asserting
    the opposite contract from `extract_nets`' own docstring. Measured: a zone
    on this name resolved to net 0 while the identical segment token resolved
    to 4. (#369 A12, which that comment cited, is about the DISPLAY name -- a
    raw `net_name` evaded `--nets` filters -- and that half is unchanged.)
  * `extract_nets` matched net names with `[^"]*`, which ENDS at an escaped
    quote, so a net named with one never entered the table at all.

And the reason none of this was caught: `tests/test_748_via_dialect_protection.py`
hand-keyed its `name_to_id` fixture UNESCAPED, matching the buggy lookup rather
than the real producer. A fixture and a bug can confirm each other indefinitely.
That file now derives its map from `extract_nets`; this file holds the contract
itself, across every site, so a new site cannot pick the wrong dialect quietly.

Run with:  python3 tests/test_backslash_net_resolves_everywhere.py
"""
import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_router'))

import kicad_parser as K  # noqa: E402

FAILS = []

# The two names a quoted S-expr token can carry that `[^"]*` gets wrong, plus a
# plain one so a rule that resolves NOTHING cannot pass this file.
BACKSLASH = '/GPIO22\\I2C1_SDA'      # as neo6502 spells it; raw text `\\`
QUOTED = '/A"B'                      # raw text `\"`
PLAIN = '/SIG'
RAW = {BACKSLASH: '/GPIO22\\\\I2C1_SDA', QUOTED: '/A\\"B', PLAIN: '/SIG'}
IDS = {PLAIN: 5, BACKSLASH: 4, QUOTED: 7}

HDR = '(kicad_pcb (version 20241229) (generator "pcbnew")\n'


def check(name, cond, detail=''):
    print(f"--- {name}")
    if cond:
        print(f"  PASS{': ' + detail if detail else ''}")
    else:
        print(f"  FAIL: {detail}")
        FAILS.append(name)


def _board(dialect):
    """A board carrying, for each of the three names, one segment, one via and
    one zone -- in `dialect` ('v9' numeric refs, 'v10' name refs, 'mixed' as
    this repo's own fanout step really emits)."""
    body = ['\t(net 0 "")']
    for nm, nid in sorted(IDS.items(), key=lambda kv: kv[1]):
        body.append(f'\t(net {nid} "{RAW[nm]}")')
    for nm, nid in sorted(IDS.items(), key=lambda kv: kv[1]):
        seg_ref = f'(net {nid})' if dialect == 'v9' else f'(net "{RAW[nm]}")'
        via_ref = f'(net {nid})' if dialect == 'v9' else f'(net "{RAW[nm]}")'
        if dialect == 'mixed':          # segments numeric, vias by name
            seg_ref = f'(net {nid})'
        y = float(nid)
        body.append(
            f'\t(segment (start 1.0 {y}) (end 2.0 {y}) (width 0.2) '
            f'(layer "F.Cu") {seg_ref} (uuid "aaaaaaaa-0000-0000-0000-'
            f'00000000000{nid}"))')
        body.append(
            f'\t(via (at 3.0 {y}) (size 0.6) (drill 0.3) '
            f'(layers "F.Cu" "B.Cu") {via_ref} (uuid "bbbbbbbb-0000-0000-'
            f'0000-00000000000{nid}"))')
        body.append(
            f'\t(zone {via_ref} (net_name "{RAW[nm]}") (layer "F.Cu") '
            f'(polygon (pts (xy 5.0 {y}) (xy 6.0 {y}) (xy 6.0 {y + 0.5}))))')
    return HDR + '\n'.join(body) + '\n)\n'


def _sites(dialect):
    content = _board(dialect)
    nets, n2i = K.extract_nets(content, kicad_version=9)
    return nets, n2i, {
        'segment': K.extract_segments(content, n2i),
        'via': K.extract_vias(content, n2i),
        'zone': K.extract_zones(content, n2i),
    }


def t_the_net_table_reads_every_name():
    """Non-vacuity, and the `[^"]*` hole: a name is IN the table or no later
    site can possibly resolve it."""
    for dialect in ('v9', 'v10', 'mixed'):
        nets, n2i, _ = _sites(dialect)
        missing = [nm for nm in IDS if RAW[nm] not in n2i]
        check(f't_the_net_table_reads_every_name[{dialect}]', not missing,
              f'name_to_id keys the RAW text of all three names '
              f'(missing: {missing})')
        wrong = {nm: nets[IDS[nm]].name for nm in IDS
                 if nets.get(IDS[nm]) and nets[IDS[nm]].name != nm}
        check(f't_the_display_name_is_unescaped[{dialect}]', not wrong,
              f'Net.name is the unescaped name (wrong: {wrong})')


def t_every_site_agrees_on_the_id():
    """The contract. A via, a segment and a zone carrying ONE net token must
    land on ONE net id -- in every dialect, including the mixed one this repo's
    own fanout step produces."""
    for dialect in ('v9', 'v10', 'mixed'):
        _, _, sites = _sites(dialect)
        for nm, nid in IDS.items():
            got = {}
            got['segment'] = [s.net_id for s in sites['segment']
                              if abs(s.start_y - nid) < 1e-9]
            got['via'] = [v.net_id for v in sites['via']
                          if abs(v.y - nid) < 1e-9]
            got['zone'] = [z.net_id for z in sites['zone']
                           if z.net_name == nm]
            bad = {k: v for k, v in got.items() if v != [nid]}
            check(f't_every_site_agrees_on_the_id[{dialect}][{nm}]',
                  not bad,
                  f'segment/via/zone all resolve to net {nid} '
                  f'(disagreeing: {bad})')


def t_no_copper_lands_on_net_zero():
    """The shape the bug took on a real board: copper the file assigns to a
    real net modelled as net 0, where it is foreign to everything and grades as
    a short against its own net."""
    for dialect in ('v9', 'v10', 'mixed'):
        _, _, sites = _sites(dialect)
        orphans = ([('segment', s.start_y) for s in sites['segment'] if not s.net_id]
                   + [('via', v.y) for v in sites['via'] if not v.net_id]
                   + [('zone', z.net_name) for z in sites['zone'] if not z.net_id])
        check(f't_no_copper_lands_on_net_zero[{dialect}]', not orphans,
              f'every parsed object carries its file net (net-0: {orphans})')


def t_the_writer_round_trips_the_name():
    """The #312 half, asserted here so the pair cannot drift apart again: what
    the writer emits for a name must be what the parser reads back."""
    from kicad_writer import _escape_net_name
    for nm in IDS:
        check(f't_the_writer_round_trips_the_name[{nm}]',
              _escape_net_name(nm) == RAW[nm]
              and K._unescape_kicad_string(RAW[nm]) == nm,
              f'escape/unescape are inverses for {nm!r}')


def t_the_748_fixture_is_derived_not_hand_keyed():
    """The gap that let all of this live: a fixture that agreed with the bug.

    Asked of the SOURCE, because a hand-written map that happens to be right
    today is still free to drift tomorrow -- the point is that the map comes
    from the only producer there is.
    """
    import ast
    src = open(os.path.join(TESTS_DIR,
                            'test_748_via_dialect_protection.py')).read()
    derived = False
    for node in ast.walk(ast.parse(src)):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                and node.func.id == 'extract_nets'):
            derived = True
    check('t_the_748_fixture_is_derived_not_hand_keyed', derived,
          'test_748 builds its name_to_id with extract_nets rather than '
          'hand-keying it')


def main():
    t_the_net_table_reads_every_name()
    t_every_site_agrees_on_the_id()
    t_no_copper_lands_on_net_zero()
    t_the_writer_round_trips_the_name()
    t_the_748_fixture_is_derived_not_hand_keyed()
    print()
    if FAILS:
        print(f"{len(FAILS)} FAILURE(S): {', '.join(FAILS)}")
        return 1
    print("ALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
