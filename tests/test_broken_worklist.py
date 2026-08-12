#!/usr/bin/env python3
"""`broken` must ship a WORK LIST, not just a count (Step 9, 9.1a-ii).

`unrouted` is actionable from names alone -- the net has no copper, route it.
A `broken` net is not: acting on it needs WHICH net, HOW MANY pieces, and WHERE
the stranded pads are. All three were already parsed out of check_connected's
output and then dropped on the floor, so `blocking_by.broken: N` was a number
with nothing behind it.

Measured consequence on a real run: the loop drove `unrouted` 4 -> 0 using
`net_widths`' per-net detail as its model, and left `broken` at 14 across two
iterations because it had no equivalent to work from.
"""
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-routing', 'scripts'))
# board_score's own main() puts the clone root on sys.path before it calls
# anything; a test calling those functions directly has to do the same, or
# `kicad_parser` is missing and the net-name audit reports UNGRADED forever.
sys.path.insert(0, ROOT)

import board_score as bs                                        # noqa: E402

# A check_connected report with one clean net, one unrouted net and two broken
# nets of different severity -- including a stranded pad carrying a REF, which is
# what lets a caller tell a do-not-fit part from a real defect.
SAMPLE = """Found 1330 segments, 108 vias, 210 pads, 1 zones
Checking 41 routed nets
FOUND 3 ISSUES:
  Unrouted nets (1):
    QSPI_SD1 (2 pads)

  Connectivity issues (2):

  FLASH_CS (net 2):
    Segments: 9, Vias: 0, Pads: 3
    Disconnected components: 2
    Disconnected pads:
      (120.49, 78.50) on F.Cu [R1]

  GND (net 4):
    Segments: 618, Vias: 38, Pads: 55
    Disconnected components: 5
    Disconnected pads:
      (137.72, 66.05) on F.Cu [SW1]
      (146.00, 70.50) on F.Cu [J2]
      ... and 15 more
"""


def _parse(monkey_output):
    """Drive score_connectivity against a canned check_connected report."""
    real = bs.run_tool
    bs.run_tool = lambda root, tool, board, *a, **k: (1, monkey_output)
    try:
        return bs.score_connectivity('.', 'board.kicad_pcb')
    finally:
        bs.run_tool = real


def test_broken_carries_names_pieces_and_stranded_pads():
    conn = _parse(SAMPLE)
    d = conn['broken_detail']
    assert set(d) == {'FLASH_CS', 'GND'}, \
        f'broken nets must be named, got {sorted(d)}'
    assert d['GND']['components'] == 5 and d['GND']['joins_needed'] == 4
    assert d['FLASH_CS']['components'] == 2 and d['FLASH_CS']['joins_needed'] == 1
    print('  PASS: broken nets carry names and piece counts')


def test_joins_needed_sums_to_the_blocking_count():
    """The list must be COMPLETE -- otherwise sorting by it hides work."""
    conn = _parse(SAMPLE)
    total = sum(v['joins_needed'] for v in conn['broken_detail'].values())
    assert total == conn['broken'], \
        f'joins_needed sums to {total} but blocking_by.broken is {conn["broken"]}'
    print(f'  PASS: joins_needed sums to blocking_by.broken ({total})')


def test_stranded_pads_carry_the_ref_that_classifies_the_break():
    """A break on a DNF part and a break on the MCU are the same NUMBER and
    completely different work. The ref is the only thing that separates them."""
    conn = _parse(SAMPLE)
    fc = conn['broken_detail']['FLASH_CS']['stranded_pads']
    assert fc and fc[0]['ref'] == 'R1', f'stranded pad ref missing: {fc}'
    assert fc[0]['layer'] == 'F.Cu' and abs(fc[0]['x'] - 120.49) < 1e-9
    gnd = {p['ref'] for p in conn['broken_detail']['GND']['stranded_pads']}
    assert gnd == {'SW1', 'J2'}, f'expected SW1/J2, got {gnd}'
    print('  PASS: stranded pads carry ref, layer and coordinates')


def test_unrouted_names_are_separate_from_broken_names():
    """connectivity_nets is the UNION, so it cannot drive either lever."""
    conn = _parse(SAMPLE)
    assert conn['unrouted_net_names'] == ['QSPI_SD1']
    assert 'QSPI_SD1' not in conn['broken_detail']
    assert set(conn['nets']) == {'QSPI_SD1', 'FLASH_CS', 'GND'}
    print('  PASS: unrouted and broken names are separable')


ZONED_BOARD = '''(kicad_pcb
	(zone
		(net "GND")
		(layer "B.Cu")
		(hatch edge 0.5)
	)
	(footprint "x"
		(pad "1" smd rect
			(zone_connect 2)
		)
	)
)
'''


def test_a_poured_net_is_routed_to_the_plane_repair(tmpdir=None):
    """WHICH STEP fixes a break is decided by one fact: is the net poured?

    route.py cannot tap a pour, so a stranded plane pad handed to it is work that
    cannot succeed -- measured, `broken` sat at 14 across two iterations of
    route.py calls and fell to 11 in ONE route_disconnected_planes call. The
    classification must therefore be read off the board's zones, not guessed.
    """
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        b = os.path.join(d, 'b.kicad_pcb')
        with open(b, 'w', encoding='utf-8') as f:
            f.write(ZONED_BOARD)
        real = bs.run_tool
        bs.run_tool = lambda root, tool, board, *a, **k: (1, SAMPLE)
        try:
            conn = bs.score_connectivity('.', b)
        finally:
            bs.run_tool = real

    assert conn['poured_nets'] == ['GND'], \
        f"a zone naming its net as (net \"GND\") must be seen, got {conn['poured_nets']}"
    assert conn['broken_detail']['GND']['handler'] == 'route_disconnected_planes'
    assert conn['broken_detail']['FLASH_CS']['handler'] == 'route'
    print('  PASS: poured nets go to the plane repair, the rest to route.py')


def test_zone_connect_pad_property_is_not_mistaken_for_a_zone():
    """`(zone_connect 2)` appears hundreds of times inside footprints. Matching
    it would classify half the board as poured."""
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        b = os.path.join(d, 'b.kicad_pcb')
        with open(b, 'w', encoding='utf-8') as f:
            f.write('(kicad_pcb (footprint "x" (pad "1" (zone_connect 2) '
                    '(net 4 "GND"))))\n')
        real = bs.run_tool
        bs.run_tool = lambda root, tool, board, *a, **k: (1, SAMPLE)
        try:
            conn = bs.score_connectivity('.', b)
        finally:
            bs.run_tool = real
    assert conn['poured_nets'] == [], \
        f'zone_connect is not a zone, got {conn["poured_nets"]}'
    print('  PASS: (zone_connect N) is not read as a pour')


def test_a_fully_connected_board_reports_empty_lists_not_missing_keys():
    conn = _parse('ALL NETS FULLY CONNECTED\n')
    assert conn['broken'] == 0 and conn['unrouted'] == 0
    print('  PASS: a clean board is clean')


# --- D6: one payload, one spelling of every net -------------------------------
#
# `poured_nets` is the ONLY net-name field in the score read out of the raw file
# text instead of through the parser, and the file escapes a backslash. Measured
# on neo6502, from a single json.dump:
#     poured_nets   : '/GPIO10\\OE3#'   <- two backslashes
#     unrouted.nets : '/GPIO10\OE3#'    <- one, and the board agrees
# A consumer built --ignore-nets from the first, 10 of 61 names matched nothing,
# and the render came back hpwl +45.6% / crossings +129.5% on an unchanged board.

# A net name with a backslash, as the FILE stores it (escaped). This is the
# neo6502 shape exactly: the file says `\\`, the net is one `\`.
ESCAPED_BOARD = (
    '(kicad_pcb (version 20240108)\n'
    '\t(net 0 "")\n'
    '\t(net 1 "/GPIO10\\\\OE3#")\n'
    '\t(net 2 "GND")\n'
    '\t(zone (net 1) (net_name "/GPIO10\\\\OE3#") (layer "B.Cu") (hatch edge 0.5))\n'
    '\t(zone (net 2) (net_name "GND") (layer "B.Cu") (hatch edge 0.5))\n'
    ')\n')

# What the parser -- and therefore the board, check_connected, and every other
# field of the score -- calls those two nets.
TRUE_NAMES = ['/GPIO10\\OE3#', 'GND']


def _write(d, name, text):
    p = os.path.join(d, name)
    with open(p, 'w', encoding='utf-8') as f:
        f.write(text)
    return p


def test_poured_nets_is_spelled_the_way_the_board_spells_it():
    """The field must survive a round trip through the board's own parser."""
    import tempfile
    from kicad_parser import parse_kicad_pcb
    with tempfile.TemporaryDirectory() as d:
        b = _write(d, 'b.kicad_pcb', ESCAPED_BOARD)
        real = bs.run_tool
        bs.run_tool = lambda root, tool, board, *a, **k: (1, SAMPLE)
        try:
            conn = bs.score_connectivity('.', b)
        finally:
            bs.run_tool = real
        truth = sorted(n.name for n in parse_kicad_pcb(b).nets.values() if n.name)

    assert conn['poured_nets'] == TRUE_NAMES, (
        f'poured_nets must spell nets as the board does.\n'
        f'  got   : {conn["poured_nets"]}\n  want  : {TRUE_NAMES}')
    assert conn['poured_nets'] == truth, (
        f'poured_nets disagrees with the parser on the same file: '
        f'{conn["poured_nets"]} vs {truth}')
    print('  PASS: poured_nets is unescaped, and matches the parser exactly')


def test_poured_nets_carries_its_meaning_in_the_payload():
    """It means "the handler is route_disconnected_planes", NOT "this is a
    plane". A consumer read it the second way and removed 332 of 545 pads
    (72%) from its own analysis. The sentence has to travel WITH the list --
    a comment in board_score.py is invisible to whoever reads the JSON."""
    conn = _parse(SAMPLE)
    meaning = conn.get('poured_nets_meaning', '')
    assert meaning and 'NOT' in meaning and 'ignore-nets' in meaning, \
        f'poured_nets must publish what it means, got {meaning!r}'
    print('  PASS: poured_nets ships its own definition')


def test_every_published_net_name_exists_on_the_board():
    """The self-check that makes a future D6 impossible to publish silently.

    A mangled name and an absent net are INDISTINGUISHABLE downstream -- both
    match nothing, neither complains. Only the tool that emitted the name can
    tell them apart, so it has to be the one that looks.
    """
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        b = _write(d, 'b.kicad_pcb', ESCAPED_BOARD)

        good = {'connectivity_nets': ['GND'],
                'components': {'broken': {'poured_nets': TRUE_NAMES},
                               'unrouted': {'nets': ['/GPIO10\\OE3#']}}}
        a = bs.audit_net_names(b, good)
        assert a['ran'] and a['unknown_count'] == 0, \
            f'a correct payload must audit clean, got {a}'
        assert a['checked'] == 4, f'all four names must be checked, got {a}'

        # The exact defect, reconstructed: poured_nets double-escaped while
        # unrouted.nets is right -- one payload, two spellings.
        bad = {'connectivity_nets': ['GND'],
               'components': {'broken': {'poured_nets': ['/GPIO10\\\\OE3#']},
                              'unrouted': {'nets': ['/GPIO10\\OE3#']}}}
        a = bs.audit_net_names(b, bad)
        assert a['unknown_count'] == 1, \
            f'the double-escaped name must be caught, got {a}'
        assert a['unknown'] == {'components.broken.poured_nets':
                                ['/GPIO10\\\\OE3#']}, \
            f'the audit must name the FIELD that is wrong, got {a["unknown"]}'

        # Fields whose job is naming things that do NOT exist are excluded on
        # purpose; auditing them would report every correct run as broken.
        spec = {'components': {
            'net_widths': {'nets': {'GND': {}},
                           'patterns_matching_no_routed_net': ['NOPE*']},
            'length': {'groups': {'B': {'missing_nets': ['ALSO_NOT_HERE'],
                                        'worst': 'GND'}}}}}
        a = bs.audit_net_names(b, spec)
        assert a['unknown_count'] == 0, \
            f'spec-finding fields must not be audited as net names, got {a}'
    print('  PASS: every published net name is checked against the board')


def test_the_audit_refuses_to_pass_a_board_it_could_not_read():
    """`0 unknown` from a board with no net table is the vacuity trap this
    file guards everywhere else -- it must say `ran: false` instead."""
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        b = _write(d, 'b.kicad_pcb', '(kicad_pcb (version 20240108))\n')
        a = bs.audit_net_names(b, {'connectivity_nets': ['GND']})
    assert a['ran'] is False and 'unknown_count' not in a, \
        f'no net table is not a pass, got {a}'
    print('  PASS: an unreadable board audits UNGRADED, not clean')


if __name__ == '__main__':
    for k, v in sorted(globals().items()):
        if k.startswith('test_'):
            print(f'--- {k}')
            v()
    print('ALL PASS')
