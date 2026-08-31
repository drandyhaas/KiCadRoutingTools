#!/usr/bin/env python3
"""#700 item 1: the fab-floor bucketing is VISIBLE, so a comparison stops
claiming a result it could not have measured.

`_FAB_FLOORS` has two rungs, keyed 2 and 4, so `fab_floor_min(4)`, `(6)` and
`(8)` return byte-identical dicts. That is a modelling limit -- JLC publishes
one multilayer capability column -- and `placement.options.add_layers` reported
it as a routing fact: on rp2350_fpga_eensy_prePlane, 6 copper layers and 121
escape lanes short, it said "more layers buy NO extra lanes on a face".

The issue offers two fixes. Extending `_FAB_FLOORS` with 6- and 8-layer rungs
is declined and the reason is checkable: `fab_tiers.py`'s module docstring
cites jlcpcb.com/capabilities for every number in the table, a 6-layer rung has
no such source, and `tests/test_fab_tiers.py` pins the ladder PER BUCKET -- so
inventing the rung means inventing its expected via pair in the test too. This
is the other fix: report the bucket.
"""
import io
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))

import fab_tiers as ft                                      # noqa: E402


def t_the_reported_bucket_is_the_one_the_floor_came_from():
    """The whole point of `_layer_bucket` being ONE function.

    A provenance field that can disagree with the value it describes is worse
    than none: it is a second source of truth that reads as a first. Checked
    over a range wider than the table, so a future rung cannot silently
    desynchronise the two.
    """
    for n in range(0, 13):
        b = ft.fab_floor_bucket(n)
        assert b.bucket in ft._FAB_FLOORS, (n, b.bucket)
        for tier in ft.TIERS:
            assert (ft.fab_floor_ladder(n, tier)[-1]
                    == ft.fab_floor_ladder(b.bucket, tier)[-1]), (n, tier)

    # The loop above routes BOTH sides through `_layer_bucket`, so any
    # idempotent function satisfies it -- `_layer_bucket = lambda n: 4` passes
    # every line of it. So also pin the RULE, against the expression this
    # replaced, over the same range: `_FAB_FLOORS[2] if (n or 2) <= 2 else
    # _FAB_FLOORS[4]`. That is what makes this a regression test rather than a
    # restatement.
    for n in list(range(0, 13)) + [None, 1000]:
        want = 2 if (n or 2) <= 2 else 4
        assert ft.fab_floor_bucket(n).bucket == want, (n, want)
        assert ft._layer_floors(n) is ft._FAB_FLOORS[want], n
    print("  PASS: bucket and floor agree for 0..12 copper layers, both "
          "tiers, and match the rule this replaced")


def t_bucketed_and_saturated_say_which_answers_are_proxies():
    """2 and 4 are answered exactly; 3, 5, 6, 8 are answered by a proxy."""
    exact = {n: ft.fab_floor_bucket(n) for n in (2, 4)}
    proxy = {n: ft.fab_floor_bucket(n) for n in (1, 3, 5, 6, 8, 10)}
    for n, b in exact.items():
        assert b.bucketed is False, (n, b)
        assert b.requested == b.bucket == n, (n, b)
    for n, b in proxy.items():
        assert b.bucketed is True, (n, b)
    # Saturation is the sharper claim: above the last rung, EVERY higher count
    # returns this identical floor, so asking again buys nothing.
    assert ft.fab_floor_bucket(2).saturated is False
    for n in (4, 6, 8, 10):
        assert ft.fab_floor_bucket(n).saturated is True, n
    assert ft.fab_floor_min(6) == ft.fab_floor_min(8) == ft.fab_floor_min(10), \
        'saturated claims these are identical; they must be'
    print("  PASS: 2 and 4 exact, 1/3/5/6/8/10 proxied, 4+ saturated")


def t_an_unreadable_layer_count_takes_the_conservative_bucket():
    """0 means "I could not look", and must never become bucket 0.

    Matches `check_drc`'s own `len(copper_layers) if copper_layers else 2`.
    The conservative direction is the 2-layer bucket, whose clearance floor is
    LOOSER (0.10 vs 0.09), so an unreadable board is never told it can etch
    finer than it can.
    """
    assert ft.fab_floor_bucket(0).bucket == 2
    assert ft.fab_floor_bucket(None).bucket == 2
    assert ft.fab_floor_min(0)['clearance'] > ft.fab_floor_min(4)['clearance']
    print("  PASS: an unreadable count takes the looser bucket")


def t_the_floor_dict_did_not_grow_a_provenance_key():
    """Provenance is a SEPARATE object, and this is why.

    The floor dict is written out: `list_nets.effective_floors` returns one
    under 'fab' and `read_design_rules` embeds that whole structure as a
    documented public return. A key added here silently joins that contract,
    and several consumers index the dict directly.
    """
    for n in (2, 4, 6):
        for tier in ft.TIERS:
            for floor in ft.fab_floor_ladder(n, tier):
                assert tuple(sorted(floor)) == tuple(sorted(ft.FLOOR_KEYS)), \
                    f'{n}L {tier} floor keys drifted: {sorted(floor)}'
    print("  PASS: every floor dict is still exactly FLOOR_KEYS")


def t_an_override_file_cannot_forge_provenance():
    """Enforced by the SIGNATURE, not by a promise.

    `fab_floor_bucket` takes one argument, so a tier and an override file have
    no channel to it at all. `fab_floor_ladder` can today only REPLACE keys
    already present -- but "cannot today" and "cannot by construction" are
    different guarantees, and only the second survives a future edit.
    """
    import inspect
    sig = inspect.signature(ft.fab_floor_bucket)
    assert list(sig.parameters) == ['copper_layer_count'], sig
    # ...and the process-wide tier really is inert on it.
    prev = ft.get_default_fab_tier()
    try:
        before = ft.fab_floor_bucket(6)
        ft.set_default_fab_tier('advanced', {'clearance': 0.01,
                                             'via_diameter': 0.01})
        assert ft.fab_floor_bucket(6) == before, (before, ft.fab_floor_bucket(6))
    finally:
        ft.set_default_fab_tier(*prev)
    print("  PASS: one argument, and the process tier cannot reach it")


def t_fine_via_rung_is_asked_not_restated():
    """The module's one OTHER layer-count branch, derived from the ladder.

    Restating `n > 2 and adv < 0.30 < std` here would be a second copy of a
    condition that has already moved once. Asking `fab_floor_ladder` means the
    field cannot drift from the ladder it describes.
    """
    for n in (1, 2):
        assert ft.fab_floor_bucket(n).fine_via_rung is False, n
        assert len(ft.fab_floor_ladder(n, 'standard')) == 2, n
    for n in (3, 4, 6, 8):
        assert ft.fab_floor_bucket(n).fine_via_rung is True, n
        assert len(ft.fab_floor_ladder(n, 'standard')) == 3, n

    # Value agreement at those six counts is satisfied by a RESTATEMENT
    # (`n > 2` passes every line above), so also check the field tracks the
    # ladder when the ladder MOVES -- which a restatement cannot do. Removing
    # the fine-via rung from the standard ladder must take the field with it.
    _real = ft.fab_floor_ladder
    try:
        ft.fab_floor_ladder = lambda n, tier=None, overrides=None: [
            {}, {}] if (n or 2) > 2 else [{}, {}]
        assert ft.fab_floor_bucket(6).fine_via_rung is False, (
            'fine_via_rung restates `n > 2` instead of asking the ladder')
    finally:
        ft.fab_floor_ladder = _real
    assert ft.fab_floor_bucket(6).fine_via_rung is True, 'restore failed'
    print("  PASS: fine-via rung tracks the standard ladder at 1,2 vs 3,4,6,8, "
          "and follows it when the ladder moves")


def t_add_layers_stops_claiming_a_result_it_could_not_measure():
    """The behaviour #700 filed, on the board it is worst on.

    rp2350_fpga_eensy_prePlane: 6 copper layers, 121 deficit lanes at its own
    clearance, and the tool answered "more layers buy NO extra lanes on a
    face" -- a routing claim, from a comparison of two identical dicts.
    """
    from kicad_parser import parse_kicad_pcb
    from placement.options import add_layers

    board = os.path.join(ROOT, 'kicad_files',
                         'rp2350_fpga_eensy_prePlane.kicad_pcb')
    if not os.path.isfile(board):
        print("  SKIP: rp2350_fpga_eensy_prePlane not in kicad_files/")
        return
    out = add_layers(parse_kicad_pcb(board), board, clearance=0.2)
    assert out.get('ran'), out.get('reason')
    m = out['measured']
    assert m['copper_layers'] == 6, m['copper_layers']
    assert m['fab_bucket_now'] == m['fab_bucket_at_more'] == 4, m
    assert m['fab_floor_layer_blind'] is True, m
    assert m['fab_bucket_saturated'] is True, m
    assert 'STRUCTURALLY BLIND' in out['action'], out['action']
    assert 'buy NO extra lanes' not in out['action'], out['action']
    # The board really is short, so this is not a vacuous pass on a clean board
    assert m['deficit_lanes_now'] > 0, m
    # ...and the admission is machine-readable, not only prose.
    assert 'fab_tiers models buckets' in out['not_modelled'], out['not_modelled']
    print(f"  PASS: 6 layers, {m['deficit_lanes_now']} deficit lanes, "
          f"answered 'structurally blind' instead of 'buy nothing'")


def t_a_two_layer_board_still_gets_the_real_comparison():
    """The 2 -> 4 step crosses a bucket, so it is a genuine measurement and
    must NOT be reported as blind. Without this, "always say blind" passes the
    case above."""
    from kicad_parser import parse_kicad_pcb
    from placement.options import add_layers

    board = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
    if not os.path.isfile(board):
        print("  SKIP: esp_prog not in kicad_files/")
        return
    out = add_layers(parse_kicad_pcb(board), board, clearance=0.2)
    assert out.get('ran'), out.get('reason')
    m = out['measured']
    assert m['copper_layers'] == 2, m['copper_layers']
    assert m['fab_bucket_now'] == 2 and m['fab_bucket_at_more'] == 4, m
    assert m['fab_floor_layer_blind'] is False, m
    assert m['floors_differ'] is True, m
    assert 'STRUCTURALLY BLIND' not in out['action'], out['action']
    assert 'fab_tiers models buckets' not in out['not_modelled'], \
        'a board whose comparison CAN differ must not carry the blindness note'
    print("  PASS: a 2->4 step is measured, not declared blind")


def t_the_headline_key_survives_the_text_digest():
    """`_digest` truncates at five, and this file has already lost a headline
    that way -- the comment above `_DIGEST_ALWAYS` records it. A disclosure
    nobody can see is not a disclosure."""
    from placement import options
    assert 'fab_floor_layer_blind' in options._DIGEST_ALWAYS
    # ...and the non-empty LIST is deliberately not forced: it would spend a
    # slot rendering `fab_buckets_modelled[2]` and evict a real number.
    assert 'fab_buckets_modelled' not in options._DIGEST_ALWAYS
    text = options._digest({'a': 1, 'b': 2, 'c': 3, 'd': 4, 'e': 5, 'f': 6,
                            'fab_floor_layer_blind': True})
    assert 'fab_floor_layer_blind=yes' in text, text
    print("  PASS: the blindness flag is forced past the digest limit")


def t_the_disclosure_is_not_a_string_only_channel():
    """`_digest` skips `str` values outright, so a string-valued `measured`
    key is invisible in the text channel. Every key this phase adds is a bool,
    an int or a list -- pinned, because the next one will be tempting to add
    as a string."""
    from placement import options
    text = options._digest({'a_string': 'invisible', 'a_bool': True})
    assert 'a_string' not in text, text
    assert 'a_bool=yes' in text, text
    print("  PASS: _digest drops strings -- new keys must not be strings")


TESTS = (t_the_reported_bucket_is_the_one_the_floor_came_from,
         t_bucketed_and_saturated_say_which_answers_are_proxies,
         t_an_unreadable_layer_count_takes_the_conservative_bucket,
         t_the_floor_dict_did_not_grow_a_provenance_key,
         t_an_override_file_cannot_forge_provenance,
         t_fine_via_rung_is_asked_not_restated,
         t_add_layers_stops_claiming_a_result_it_could_not_measure,
         t_a_two_layer_board_still_gets_the_real_comparison,
         t_the_headline_key_survives_the_text_digest,
         t_the_disclosure_is_not_a_string_only_channel)


def _every_case_is_registered():
    defined = {n for n in globals() if n.startswith('t_')}
    listed = {f.__name__ for f in TESTS}
    assert defined == listed, f'not registered: {sorted(defined - listed)}'


if __name__ == '__main__':
    _every_case_is_registered()
    for fn in TESTS:
        print(fn.__name__)
        fn()
    print('\nALL PASS')
