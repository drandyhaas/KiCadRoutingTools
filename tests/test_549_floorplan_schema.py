"""
Floorplan intent: the schema, and what makes one invalid (issue #549).

Board-independent half only -- everything here runs without parsing a
`.kicad_pcb`, which is the point of keeping `validate_intent` separate from
`grade`. An intent that contradicts ITSELF (a zone outside the envelope, two
zones fighting over the same rectangle) should be reported as a broken intent,
not as a pile of board violations that sends the reader looking at footprints.

The load-bearing assertions here are the REJECTIONS. A grader whose intent file
silently accepts a typo is worse than no grader: a block that resolves to
nothing grades clean, and "0 violations" then means "nothing was checked". So
every malformed shape has to raise rather than degrade, and the error has to
name the thing that was wrong.
"""

import json
import os
import re
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_placer'))  # placement split
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_router'))  # placement split
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_tools'))  # placement split

from placement import floorplan as fp
from placement.floorplan import (ERROR, KIND, READER_VERSION, RULES,
                                 SCHEMA_VERSION, WARN, Intent, IntentError,
                                 _SEVERITY_KEYS, intent_from_dict, load_intent,
                                 validate_intent)


def _base(**over):
    d = {
        'schema': SCHEMA_VERSION,
        'kind': KIND,
        'units': 'mm',
        'envelope': {'rect': [0, 0, 100, 80]},
        'blocks': [
            {'name': 'power', 'refs': ['U3', 'C1*'], 'zone': [2, 2, 40, 30]},
            {'name': 'mcu', 'refs': ['U1'], 'zone': [50, 2, 90, 30]},
        ],
    }
    d.update(over)
    return d


def _rejects(raw, why):
    try:
        intent_from_dict(raw)
    except IntentError as exc:
        return str(exc)
    raise AssertionError(f"NOT rejected: {why}")


def test_a_wellformed_intent_loads():
    i = intent_from_dict(_base())
    assert isinstance(i, Intent)
    assert [z.name for z in i.blocks] == ['power', 'mcu']
    assert i.blocks[0].refs == ('U3', 'C1*')
    assert i.blocks[0].rect == (2.0, 2.0, 40.0, 30.0)
    assert i.envelope['rect'] == (0.0, 0.0, 100.0, 80.0)
    assert validate_intent(i) == []
    print(f"  PASS: {len(i.blocks)} blocks, envelope "
          f"{i.envelope['rect']}, no self-contradiction")


def test_rects_are_normalised_not_trusted():
    """A rect given corner-swapped is still a rect. Silently comparing against
    an inverted rect would make every containment test vacuously true."""
    i = intent_from_dict(_base(envelope={'rect': [100, 80, 0, 0]}))
    assert i.envelope['rect'] == (0.0, 0.0, 100.0, 80.0)
    print("  PASS: [100,80,0,0] normalised to (0,0,100,80)")


def test_the_wrong_file_is_refused_by_kind():
    """A round sidecar or a lock-advisor dump is valid JSON with a `schema` key.
    Without the `kind` discriminator it would load as an intent with no blocks
    and no rules -- i.e. grade perfectly clean."""
    sidecar = {'schema': 1, 'round': 3, 'accepted': True, 'moved': []}
    msg = _rejects(sidecar, "a loop round sidecar")
    assert 'kind' in msg, msg
    msg = _rejects(_base(kind='lock-advice'), "kind=lock-advice")
    assert KIND in msg, msg
    print("  PASS: a round sidecar cannot masquerade as an intent")


def test_schema_version_is_enforced():
    msg = _rejects(_base(schema=SCHEMA_VERSION + 1), "a future schema")
    assert str(SCHEMA_VERSION) in msg, msg
    _rejects(_base(schema=None), "a missing schema")
    print("  PASS: only schema %d loads" % SCHEMA_VERSION)


def test_unknown_keys_are_refused_not_ignored():
    """A typo'd key that is silently dropped is a constraint the author thinks
    they set and the grader never checks."""
    msg = _rejects(_base(keepout=[]), "keepout (singular) instead of keepouts")
    assert 'keepout' in msg and 'keepouts' in msg, msg
    msg = _rejects(_base(blocks=[{'name': 'x', 'refs': ['U1'], 'zones': [1]}]),
                   "zones instead of zone")
    assert 'zones' in msg, msg
    print("  PASS: unknown top-level and per-block keys both refused")


#: Matches `rule='x'` and `rule="x"` alike. Built here rather than inline so
#: the pattern's own quoting stays readable.
RULE_LITERAL = "rule=[" + chr(39) + chr(34) + "](" + chr(92) + "w+)[" \
    + chr(39) + chr(34) + "]"

#: The intent vocabulary, stated here rather than derived from the module, so
#: that GROWING a key set is as loud as shrinking one. Both directions are
#: defects: a shrunk set refuses a key real intent files carry, and a grown one
#: re-admits a key nothing reads -- which is #710 itself. The emitter-driven
#: detector in test_549_floorplan_grade.py cannot see either: it only harvests
#: `edge_connectors[]` / `blocks[]` entries, and of the remaining objects
#: `emit_intent` writes only `decaps` (a bare `max_distance_mm`, under
#: --declare-decaps since #704) and never a `health` or `keepouts` one.
#: #704's provenance deliberately went to `context`, which has NO key set --
#: growing `_DECAP_KEYS` would make an older build REFUSE the whole file,
#: since `_reject_unknown` raises.
KEY_SETS = {
    '_TOP_LEVEL_KEYS': {
        'schema', 'kind', 'board', 'units', 'min_reader', 'envelope',
        'defaults', 'blocks', 'keepouts', 'edge_connectors', 'decaps',
        'must_lock', 'legality_budget', 'health', 'severity', 'context',
        'overlap_waivers', 'assembly'},
    '_ENVELOPE_KEYS': {'rect', 'tolerance_mm'},
    '_DEFAULTS_KEYS': {'zone_tolerance_mm'},
    '_BLOCK_KEYS': {'name', 'group', 'refs', 'zone', 'side', 'exclusive',
                    'tolerance_mm', 'note', 'context'},
    '_KEEPOUT_KEYS': {'name', 'rect', 'circle', 'sides', 'allow', 'note',
                      'context'},
    '_EDGE_CONNECTOR_KEYS': {
        'ref', 'edge', 'overhang_mm', 'max_setback_mm', 'class', 'source',
        'note', 'suspect', 'suspect_reason', 'overhang_capped',
        'observed_overhang_mm', 'context',
        # #712: WHERE ALONG the edge. Mutually exclusive, absent by default,
        # and never written by `emit_intent`.
        'center_on_edge', 'along_edge_band'},
    '_OVERHANG_KEYS': {'min', 'max'},
    '_CENTER_ON_EDGE_KEYS': {'tolerance_mm'},
    '_ALONG_EDGE_BAND_KEYS': {'from', 'to'},
    '_DECAP_KEYS': {'max_distance_mm', 'exempt', 'search_radius_mm',
                    'max_pin_distance_mm', 'pin_functions', 'same_side'},
    '_HEALTH_KEYS': {'bus_corridors', 'classes', 'zoned_blocks',
                     'affinity_exempt_nets', 'affinity_exempt_net_ids',
                     'ignore_net_ids', 'max_fanout', 'block_displacement_mm',
                     'plane_layers'},
    '_CORRIDOR_KEYS': {'name', 'nets', 'width_mm'},
    '_BUDGET_KEYS': {'overlap_area', 'oob_count', 'oob_amount'},
    '_WAIVER_KEYS': {'pair', 'reason', 'context'},
    # #837: the board-level assembly policy. `sides` is the claim; `why`
    # records how it was reached, and `emit_intent` fills it with the
    # observation rather than a reason nobody gave.
    '_ASSEMBLY_KEYS': {'sides', 'why', 'context'},
}


def test_the_key_sets_are_exactly_what_is_documented():
    """A change detector on the vocabulary itself, in BOTH directions.

    Shrinking a set is the regression this whole change risks -- it refuses a
    key that real intent files carry. Growing one silently re-admits a key
    nothing reads, which is #710 over again. Neither is visible to the
    emitter-driven detector in the grade test, which only sees the
    `edge_connectors[]` / `blocks[]` entry keys `emit_intent` writes.
    """
    for name, expected in sorted(KEY_SETS.items()):
        actual = set(getattr(fp, name))
        assert actual == expected, (name, sorted(actual ^ expected))
    doc = os.path.join(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))), 'docs', 'floorplan-intent.md')
    with open(doc, encoding='utf-8') as fh:
        text = fh.read()
    undocumented = sorted({k for keys in KEY_SETS.values() for k in keys
                           if f'`{k}`' not in text})
    assert not undocumented, undocumented

    # ...and the ACCEPTED-KEYS TABLE, not merely a mention somewhere in the
    # file. `plane_layers` (#700) went into _HEALTH_KEYS, into this test and
    # into a new prose section, while the table a reader actually consults
    # still listed eight keys where the set had nine -- because "is the name
    # backticked anywhere in this document" cannot tell a table row from a
    # paragraph, and nothing else kept the two in sync.
    rows = {}
    for line in text.splitlines():
        cells = [c.strip() for c in line.split('|')]
        if len(cells) < 4 or not cells[1].startswith('`'):
            continue
        rows[cells[1].strip('`')] = set(re.findall(r'`([a-z_]+)`', cells[2]))
    # Named explicitly rather than derived from the constant's name: a
    # derivation matched `_EDGE_CONNECTOR_KEYS` against an unrelated
    # `edge_connector` row and failed on twelve keys that are documented
    # perfectly well one row further down. A short honest map beats a clever
    # rule that is wrong in a way the failure message hides.
    TABLE_ROWS = {
        '_HEALTH_KEYS': 'health',
        '_CORRIDOR_KEYS': 'health.bus_corridors[]',
        '_DECAP_KEYS': 'decaps',
        '_BUDGET_KEYS': 'legality_budget',
        '_WAIVER_KEYS': 'overlap_waivers[]',
        '_OVERHANG_KEYS': 'edge_connectors[].overhang_mm',
        '_EDGE_CONNECTOR_KEYS': 'edge_connectors[]',
        '_ASSEMBLY_KEYS': 'assembly',
    }
    checked = 0
    for name, row in sorted(TABLE_ROWS.items()):
        listed = rows.get(row)
        assert listed is not None, f'no `{row}` row in the accepted-keys table'
        missing = sorted(KEY_SETS[name] - listed)
        assert not missing, (
            f'{name} and its `{row}` table row have drifted; the row is '
            f'missing {missing}')
        checked += 1
    print(f"  PASS: {len(KEY_SETS)} key sets pinned exactly; "
          f"{sum(len(v) for v in KEY_SETS.values())} names all documented, "
          f"{checked} of them against their table row")


def test_an_intent_using_every_known_key_loads():
    """The other half: every key in every set is ACCEPTED.

    Pinning the sets catches a set that changed; this catches a set that was
    right while the code around it refused the key anyway -- and it is the
    only test that exercises the `health` and `keepouts` keys through the
    loader at all, since `emit_intent` never writes those objects. `decaps`
    it does write since #704, but only `max_distance_mm`, so `exempt` and
    `search_radius_mm` are still covered here alone.
    """
    raw = {
        'schema': SCHEMA_VERSION, 'kind': KIND, 'units': 'mm',
        'board': 'b.kicad_pcb', 'min_reader': READER_VERSION,
        'envelope': {'rect': [0, 0, 100, 80], 'tolerance_mm': 0.4},
        'defaults': {'zone_tolerance_mm': 0.6},
        'blocks': [{'name': 'power', 'group': 'sheet:1', 'refs': ['U3'],
                    'zone': [2, 2, 40, 30], 'side': 'F', 'exclusive': True,
                    'tolerance_mm': 0.7, 'note': 'n', 'context': {'why': 'w'}}],
        'keepouts': [{'name': 'k', 'rect': [0, 0, 6, 6], 'sides': ['F'],
                      'allow': ['MH1'], 'note': 'n', 'context': {'why': 'w'}},
                     {'name': 'k2', 'circle': [50, 5, 8], 'sides': ['F', 'B'],
                      'allow': [], 'note': '', 'context': {}}],
        'edge_connectors': [{
            'ref': 'J1', 'edge': 'east',
            'overhang_mm': {'min': 0.0, 'max': 1.5}, 'max_setback_mm': 2.0,
            'class': 'edge_receptacle', 'source': 'auto-class', 'note': 'n',
            'suspect': True, 'suspect_reason': 'r', 'overhang_capped': True,
            'observed_overhang_mm': 9.0, 'context': {'why': 'w'},
            'center_on_edge': {'tolerance_mm': 1.0}},
            # #712's two fields are MUTUALLY EXCLUSIVE, so covering the
            # vocabulary needs a second entry -- one entry carrying both is
            # refused at load, deliberately. Without this the "every known key
            # is accepted" claim would silently degrade to "every key but one".
            {'ref': 'J2', 'edge': 'west',
             'along_edge_band': {'from': 0.10, 'to': 0.35}}],
        'decaps': {'max_distance_mm': 2.5, 'exempt': ['C99'],
                   'search_radius_mm': 6.0, 'max_pin_distance_mm': 1.5,
                   'pin_functions': ['VCC', 'VDD'], 'same_side': False},
        'must_lock': ['MH*'],
        'legality_budget': {'overlap_area': 1.0, 'oob_count': 2,
                            'oob_amount': 3.0},
        'health': {'bus_corridors': [{'name': 'c', 'nets': ['/D*'],
                                      'width_mm': 8.0}],
                   'classes': {'critical': ['/D*']}, 'zoned_blocks': ['power'],
                   'affinity_exempt_nets': ['/GND'],
                   'affinity_exempt_net_ids': [3], 'ignore_net_ids': [1, 2],
                   'max_fanout': 30, 'block_displacement_mm': 4.0,
                   'plane_layers': ['In1.Cu', 'In2.Cu']},
        'severity': {name: WARN for name in sorted(_SEVERITY_KEYS)},
        'context': {'note': 'read-only'},
        'overlap_waivers': [{'pair': ['U1', 'U2'], 'reason': 'net tie',
                             'context': {'why': 'w'}}],
        'assembly': {'sides': 'F', 'why': 'one reflow pass',
                     'context': {'quoted': 'the fab'}},
    }
    # Every key of every set must appear above, or this proves less than it
    # claims -- the point is coverage of the vocabulary, not of a sample.
    seen = set(raw) | set(raw['envelope']) | set(raw['defaults'])
    seen |= set(raw['blocks'][0]) | set(raw['decaps']) | set(raw['health'])
    seen |= set(raw['legality_budget'])
    # UNION over every entry, not entry [0]: the two #712 fields cannot share
    # one entry, so reading only the first would under-report the vocabulary.
    for c in raw['edge_connectors']:
        seen |= set(c)
    seen |= set(raw['edge_connectors'][0]['overhang_mm'])
    seen |= set(raw['edge_connectors'][0]['center_on_edge'])
    seen |= set(raw['edge_connectors'][1]['along_edge_band'])
    seen |= set(raw['health']['bus_corridors'][0])
    seen |= set(raw['overlap_waivers'][0])
    seen |= set(raw['assembly'])
    for k in raw['keepouts']:
        seen |= set(k)
    missing = sorted({k for keys in KEY_SETS.values() for k in keys} - seen)
    assert not missing, missing

    i = intent_from_dict(raw)
    assert i.blocks[0].tolerance_mm == 0.7 and i.blocks[0].exclusive
    assert i.decaps['search_radius_mm'] == 6.0
    assert i.decaps['max_pin_distance_mm'] == 1.5
    assert i.decaps['pin_functions'] == ['VCC', 'VDD']
    assert i.decaps['same_side'] is False
    assert i.health['max_fanout'] == 30
    assert i.legality_budget['oob_amount'] == 3.0
    assert i.keepouts[0]['allow'] == ('MH1',)
    assert i.edge_connectors[0]['max_setback_mm'] == 2.0
    print(f"  PASS: an intent using all {len(seen)} known keys loads")


def test_min_reader_refuses_a_build_that_would_ignore_the_claim():
    """#710, the compatibility half.

    Refusing unknown nested keys makes an older build fail on a field it does
    not know -- but only if the field is NEW. `min_reader` covers the other
    case: a field whose spelling an old build accepts while its MEANING
    changed, and any file whose author wants "refuse" rather than "grade it
    without this". `schema` cannot do that job: it is matched exactly, so
    bumping it invalidates every existing intent at once.
    """
    assert intent_from_dict(_base(min_reader=READER_VERSION))
    assert intent_from_dict(_base(min_reader=1))
    msg = _rejects(_base(min_reader=READER_VERSION + 1), "a future reader")
    # Phrases, not digits. `str(RV+1) in msg and str(RV) in msg` is
    # `'2' in msg and '1' in msg` at RV=1, which a message with the two
    # numbers SWAPPED satisfies exactly as well. (It did.)
    assert f"min_reader {READER_VERSION + 1}" in msg, msg
    assert f"reader {READER_VERSION}" in msg, msg
    # Refused for its TYPE, distinctly from being too new: a gate that leaked
    # floats still refuses 2.5 as a future reader, and a test that only looks
    # for 'min_reader' cannot tell the two apart. (It could not.)
    for bad in ('2', 2.5, True, [2], 0.5):
        assert 'expected an integer' in _rejects(
            _base(min_reader=bad), f"min_reader={bad!r}")
    # It outranks EVERY other check, including the unknown-key and schema
    # ones. This is the case that matters and the one that was wrong first: a
    # build declaring a new field almost always declares a new TOP-LEVEL one,
    # so if the key set is checked first the author gets "unknown key(s)
    # zones" and goes hunting for a typo instead of upgrading.
    for extra, why in (
            ({'blocks': [{'name': 'x', 'zone': [0, 0, 1, 1]}]},
             "a malformed block"),
            ({'zones': []}, "a future top-level key"),
            ({'schema': SCHEMA_VERSION + 1}, "a future schema"),
            ({'kind': 'lock-advice'}, "the wrong kind")):
        raw = {'schema': SCHEMA_VERSION, 'kind': KIND,
               'min_reader': READER_VERSION + 1}
        raw.update(extra)
        msg = _rejects(raw, f"a future reader with {why}")
        # The upgrade PHRASE, not the word `min_reader`: that word is in the
        # Known: list of every top-level unknown-key message, so asserting it
        # passed even when the key gate ran first and won. (It did.)
        assert f"min_reader {READER_VERSION + 1}" in msg, (why, msg)
        assert 'upgrade' in msg, (why, msg)
    print(f"  PASS: reader {READER_VERSION} loads min_reader<={READER_VERSION}"
          f", refuses {READER_VERSION + 1} before reading further")


def test_a_non_string_key_is_an_intent_error_not_a_TypeError():
    """`intent_from_dict` is public and takes a dict, not JSON.

    The place_* mains catch ValueError (IntentError is one) and turn it into
    exit 2; a TypeError from sorting or joining a non-string key would
    traceback straight past them.
    """
    for raw, where in (({'severity': {1: WARN}}, 'severity'),
                       ({'decaps': {2: 3}}, 'decaps'),
                       ({'envelope': {None: 1}}, 'envelope')):
        msg = _rejects(_base(**raw), f"a non-string key in {where}")
        assert where in msg, (where, msg)
    print("  PASS: a non-string key refuses as an IntentError, not TypeError")


def test_unknown_nested_keys_are_refused_not_ignored():
    """#710: the same principle, one level down.

    The loader used to be strict at exactly two levels -- top-level and
    `blocks[]` -- and permissive everywhere else, so a typo'd key inside a
    keepout, an edge connector or `health` loaded clean and constrained
    nothing. Every case below is a REAL near-miss: `max_setback` for
    `max_setback_mm`, `bus_corridor` for `bus_corridors`, `exempts` for
    `exempt`. Each message must name the key that was wrong AND the one that
    was meant -- a bare "unknown key" leaves the author reading their own
    typo back.
    """
    cases = [
        (_base(keepouts=[{'name': 'k', 'rect': [0, 0, 1, 1], 'sids': ['F']}]),
         'sids', 'sides', 'keepouts[0]'),
        (_base(edge_connectors=[{'ref': 'J1', 'max_setback': 2.0}]),
         'max_setback', 'max_setback_mm', 'edge_connectors[0]'),
        (_base(edge_connectors=[{'ref': 'J1',
                                 'overhang_mm': {'min': 0.0, 'maximum': 1.0}}]),
         'maximum', 'max', 'overhang_mm'),
        (_base(decaps={'max_distance_mm': 2.5, 'exempts': ['C9']}),
         'exempts', 'exempt', 'decaps'),
        (_base(defaults={'zone_tolerance': 0.5}),
         'zone_tolerance', 'zone_tolerance_mm', 'defaults'),
        (_base(health={'bus_corridor': []}),
         'bus_corridor', 'bus_corridors', 'health'),
        (_base(health={'bus_corridors': [{'nets': ['/D*'], 'width': 8.0}]}),
         'width', 'width_mm', 'health.bus_corridors[0]'),
        (_base(envelope={'rect': [0, 0, 100, 80], 'tolerance': 0.5}),
         'tolerance', 'tolerance_mm', 'envelope'),
        (_base(overlap_waivers=[{'pair': ['U1', 'U2'], 'because': 'by design'}]),
         'because', 'reason', 'overlap_waivers[0]'),
        (_base(blocks=[{'name': 'b', 'refs': ['U1'], 'exclusiv': True}]),
         'exclusiv', 'exclusive', 'blocks[0]'),
        (_base(legality_budget={'oob_counts': 1}),
         'oob_counts', 'oob_count', 'legality_budget'),
    ]
    # Split on `Known:` before asserting. Checking both halves of the raw
    # message would pass vacuously on every pair here -- `width` is a
    # substring of `width_mm`, `ref` of `reff` -- so the typo has to appear
    # in the REFUSAL half and the correction in the KNOWN half.
    for raw, typo, meant, where in cases:
        msg = _rejects(raw, f"{typo} instead of {meant}")
        refused, _, known = msg.partition('Known:')
        assert known, f"message carries no `Known:` list: {msg}"
        assert typo in refused, (typo, msg)
        assert meant in known.replace(',', ' ').split(), (meant, msg)
        # WHICH object. Without this the path can name a different object
        # entirely and every case still passes -- which defeats the point of
        # the message. (It did: relabelling the corridor check `blocks[9]`
        # survived all of them.)
        assert where in refused, (where, msg)
        # ...and ONLY the typo is refused, so a set that had lost a
        # legitimate key would not hide behind the same assertions.
        assert refused.count(',') == 0, ("more than one key refused", msg)
    # A typo'd `ref` reports the unknown key, not a missing `ref` -- the
    # author is looking at the key they DID write.
    msg = _rejects(_base(edge_connectors=[{'reff': 'J1'}]), "reff")
    refused, _, known = msg.partition('Known:')
    assert 'reff' in refused, msg
    assert 'ref' in known.replace(',', ' ').split(), msg
    print(f"  PASS: {len(cases) + 1} nested typos refused, each naming the "
          f"key that was meant")


def test_a_nested_object_must_be_an_object():
    """`raw.get(k) or {}` handed a list straight on, so the schema error
    surfaced as an AttributeError inside a rule three frames later -- or, for
    a key nothing reads yet, not at all."""
    for key, value in (('envelope', []), ('defaults', []), ('decaps', 'none'),
                       ('health', []), ('context', 3), ('severity', []),
                       ('legality_budget', [])):
        msg = _rejects(_base(**{key: value}), f"{key}={value!r}")
        assert key in msg, (key, msg)
        assert 'object' in msg, (key, msg)
    # Entries in a LIST get the same treatment, named by index. These guards
    # are separate code from `_obj` -- the bus_corridors one is new here and
    # had no coverage at all.
    for raw, where in (
            (_base(blocks=['U1']), 'blocks[0]'),
            (_base(keepouts=['k']), 'keepouts[0]'),
            (_base(edge_connectors=['J1']), 'edge_connectors[0]'),
            (_base(health={'bus_corridors': ['/D*']}),
             'health.bus_corridors[0]')):
        msg = _rejects(raw, f"{where} as a non-object")
        assert where in msg and 'object' in msg, (where, msg)
    # And the one nested object inside `context`, which IS read back.
    msg = _rejects(_base(context={'budget_withheld': []}),
                   'context.budget_withheld as a list')
    assert 'budget_withheld' in msg and 'object' in msg, msg
    print("  PASS: 7 objects + 4 list entries + context.budget_withheld all "
          "refuse a list/scalar instead of crashing later")


def test_context_is_deliberately_open():
    """The read-only slot. `emit_intent` writes `cutouts` / `file_locked` /
    `budget_withheld` there, and run artifacts add their own provenance prose
    (17 distinct keys across the recorded runs). Nothing grades it, so
    refusing a key here would only push provenance into a key that IS
    graded -- the worse failure."""
    i = intent_from_dict(_base(context={
        'note': 'read-only', 'authored_by': 'run-20',
        'j7_evidence': 'six of seven pins interior',
        'budget_withheld': {'overlap_area': 'a blocking body pair'}}))
    assert i.budget_withheld == {'overlap_area': 'a blocking body pair'}
    print("  PASS: context accepts arbitrary keys; budget_withheld still read")


def test_an_entry_carries_its_own_context_slot():
    """Provenance needs somewhere to go that is not a constraint.

    Without a slot the reasoning drifts into the graded keys -- the recorded
    runs show one intent whose edge_connectors entries grew `band_basis`,
    `why`, `why_not_repaired` and `rejected_alternative`, which every consumer
    then ignored. `note` is not the answer either: it is grepped for the
    substring SUSPECT, so appending prose to it can change behaviour.
    """
    ctx = {'why': 'the mating face is on the east wall of the enclosure',
           'rejected_alternative': 'north, blocked by the display window'}
    i = intent_from_dict(_base(
        blocks=[{'name': 'p', 'refs': ['U1'], 'context': ctx}],
        keepouts=[{'name': 'k', 'rect': [0, 0, 6, 6], 'context': ctx}],
        edge_connectors=[{'ref': 'J1', 'edge': 'east', 'context': ctx}],
        overlap_waivers=[{'pair': ['U1', 'U2'], 'reason': 'net tie',
                          'context': ctx}]))
    assert i.keepouts[0]['context'] == ctx
    assert i.edge_connectors[0]['context'] == ctx
    # blocks[] too. It is the one entry kind rebuilt into a dataclass rather
    # than kept as its raw dict, so accepting the key and dropping it was
    # exactly the #710 defect one level down -- and the earlier version of
    # this test passed a block context and never looked at it.
    assert i.blocks[0].context == ctx
    assert i.overlap_waivers[0]['context'] == ctx
    # Free-form, but still an object -- a list means the author meant
    # something else, while an unknown key inside means nothing at all.
    for raw, where in (
            (_base(blocks=[{'name': 'p', 'refs': ['U1'], 'context': []}]),
             'blocks[0].context'),
            (_base(edge_connectors=[{'ref': 'J1', 'context': 'why'}]),
             'edge_connectors[0].context'),
            (_base(keepouts=[{'name': 'k', 'rect': [0, 0, 6, 6],
                              'context': 1}]), 'keepouts[0].context'),
            (_base(overlap_waivers=[{'pair': ['U1', 'U2'],
                                     'context': []}]),
             'overlap_waivers[0].context')):
        msg = _rejects(raw, f"{where} as a non-object")
        assert where in msg, (where, msg)
    print("  PASS: 4 entry kinds carry a free-form context; a non-object is "
          "refused by path")


def test_severity_keys_are_checked_against_the_rule_names():
    """#710: `severity` validated its VALUES and never its keys.

    The vocabulary is 18 names, not the 11 in `RULES`: five findings are
    raised outside the rules loop (`validate_intent` raises two,
    `resolve_blocks` one, `resolve_intent_gate` raises `intent_zone_in_keepout`
    since #702, and BOTH `grade` and `resolve_intent_gate` raise
    `keepout_allow_unresolved` since #793), and TWO more are raised by a rule
    that is itself in `RULES`: `rule_decap_pin_distance` yields
    `decap_pin_distance_inferred` and `decap_pin_uncovered` beside its own
    name (#705), because one measurement can support several claims whose
    severities an author must be able to set apart. An intent has always been
    allowed to set these. Validating against `RULES` alone would refuse
    `{"block_unresolved": "warn"}`, which is legal today -- so the test
    asserts every name in the real vocabulary still loads, not just that a
    typo is refused.
    """
    # A non-PREFIX typo. `decap_distanc` is a prefix of `decap_distance`, so
    # asserting it appears in the refused half also matched a message that
    # only ever printed the suggestion. (It did.)
    msg = _rejects(_base(severity={'decap_distanse': WARN}), "a typo'd rule")
    refused, _, known = msg.partition('Known:')
    assert 'decap_distanse' in refused, msg
    assert 'decap_distance' in known.replace(',', ' ').split(), msg
    # Stated here as a literal, NOT iterated off _SEVERITY_KEYS: a test that
    # loops over the set under test just tests fewer names when the set
    # shrinks, and passes. (It did.)
    expected = {n for n, _ in RULES} | {'intent_zone_outside_envelope',
                                        'intent_zone_overlap',
                                        'block_unresolved',
                                        'intent_zone_in_keepout',
                                        'keepout_allow_unresolved',
                                        'decap_pin_distance_inferred',
                                        'decap_pin_uncovered'}
    assert _SEVERITY_KEYS == expected, sorted(_SEVERITY_KEYS ^ expected)
    for name in sorted(expected):
        i = intent_from_dict(_base(severity={name: WARN}))
        assert i.severity_of(name) == WARN, name
    print(f"  PASS: {len(expected)} settable rule names accepted, "
          f"a typo refused (RULES has {len(RULES)})")


def test_every_violation_rule_name_is_settable():
    """A change detector for the next rule.

    A rule whose name is not in `_SEVERITY_KEYS` cannot be demoted to warn --
    and, now that severity keys are refused, an intent trying to would be
    told the name does not exist. Read off the SOURCE so a new
    `Violation(rule='...')` is caught wherever it is raised, including the
    paths that never reach the RULES loop.
    """
    import re
    src = os.path.join(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))), 'py_placer', 'placement', 'floorplan.py')
    with open(src, encoding='utf-8') as fh:
        # BOTH quote styles, and any identifier: scanning only single-quoted
        # `rule=` missed a rule written with double quotes, and the
        # `len >= 10` floor let two of the twelve go missing without a word.
        names = set(re.findall(RULE_LITERAL, fh.read()))
    # Exact equality, not a floor and not a subset: every settable name must
    # be raised somewhere, and every name raised must be settable.
    assert names == set(_SEVERITY_KEYS), sorted(names ^ set(_SEVERITY_KEYS))
    print(f"  PASS: all {len(names)} violation rule names are settable")


def test_a_block_needs_refs_or_group():
    msg = _rejects(_base(blocks=[{'name': 'x', 'zone': [0, 0, 1, 1]}]),
                   "a block naming no members")
    assert 'refs' in msg and 'group' in msg, msg
    # refs alone is fine -- it is the PRIMARY form, since sheet group keys are
    # uuid paths nothing can author blind.
    assert intent_from_dict(_base(blocks=[{'name': 'x', 'refs': ['U1', 'U2']}]))
    assert intent_from_dict(_base(blocks=[{'name': 'x', 'group': 'decap:U1'}]))
    print("  PASS: refs or group required; either alone suffices")


def test_refs_as_a_bare_string_is_refused():
    """`"refs": "U1"` iterates as characters and would match nothing. Python
    would accept it silently; the schema must not."""
    msg = _rejects(_base(blocks=[{'name': 'x', 'refs': 'U1'}]), "refs as a str")
    assert 'list' in msg, msg
    print("  PASS: refs must be a list, not a string")


def test_enumerations_are_checked():
    assert 'side' in _rejects(
        _base(blocks=[{'name': 'x', 'refs': ['U1'], 'side': 'top'}]), "side=top")
    assert 'edge' in _rejects(
        _base(edge_connectors=[{'ref': 'J1', 'edge': 'left'}]), "edge=left")
    assert 'units' in _rejects(_base(units='mil'), "units=mil")
    assert 'severity' in _rejects(_base(severity={'envelope': 'maybe'}),
                                  "severity=maybe")
    print("  PASS: side / edge / units / severity enumerations enforced")


def test_malformed_rects_are_refused():
    for bad in ([0, 0, 100], [0, 0, 100, 80, 5], 'big', [0, 0, 100, 'x'],
                [0, 0, 100, True]):
        _rejects(_base(envelope={'rect': bad}), f"envelope.rect={bad!r}")
    print("  PASS: 5 malformed rect shapes refused (incl. bool-as-number)")


def test_duplicate_block_names_are_refused():
    msg = _rejects(_base(blocks=[{'name': 'a', 'refs': ['U1']},
                                 {'name': 'a', 'refs': ['U2']}]),
                   "two blocks named 'a'")
    assert 'duplicate' in msg, msg
    print("  PASS: a duplicate block name cannot silently shadow the first")


def test_keepout_needs_a_shape():
    msg = _rejects(_base(keepouts=[{'name': 'mount'}]), "keepout with no shape")
    assert 'rect' in msg and 'circle' in msg, msg
    i = intent_from_dict(_base(keepouts=[
        {'name': 'mount-NW', 'rect': [0, 0, 6, 6], 'allow': ['MH1']},
        {'name': 'antenna', 'circle': [50, 5, 8]}]))
    assert i.keepouts[0]['allow'] == ('MH1',)
    assert i.keepouts[0]['sides'] == ('F', 'B')          # defaulted, both faces
    assert i.keepouts[1]['circle'] == (50.0, 5.0, 8.0)
    print("  PASS: rect and circle keepouts load; sides default to both faces")


def test_zone_outside_the_envelope_is_a_self_contradiction():
    i = intent_from_dict(_base(blocks=[
        {'name': 'oops', 'refs': ['U9'], 'zone': [90, 70, 140, 120]}]))
    v = validate_intent(i)
    assert [x.rule for x in v] == ['intent_zone_outside_envelope'], v
    assert v[0].block == 'oops'
    assert v[0].measured['zone'] == [90.0, 70.0, 140.0, 120.0]
    print(f"  PASS: {v[0].message}")


def test_two_zones_fighting_over_one_rectangle():
    i = intent_from_dict(_base(blocks=[
        {'name': 'a', 'refs': ['U1'], 'zone': [0, 0, 40, 30]},
        {'name': 'b', 'refs': ['U2'], 'zone': [30, 0, 70, 30]}]))
    v = [x for x in validate_intent(i) if x.rule == 'intent_zone_overlap']
    assert len(v) == 1, v
    assert abs(v[0].measured['overlap_area_mm2'] - 300.0) < 1e-6, v[0].measured
    print(f"  PASS: {v[0].message}")


def test_zones_on_opposite_sides_may_share_a_rectangle():
    """F and B are different real estate. Flagging that pair would make a
    two-sided floorplan unexpressible."""
    i = intent_from_dict(_base(blocks=[
        {'name': 'front', 'refs': ['U1'], 'zone': [0, 0, 40, 30], 'side': 'F'},
        {'name': 'back', 'refs': ['U2'], 'zone': [0, 0, 40, 30], 'side': 'B'}]))
    assert [x.rule for x in validate_intent(i)] == []
    print("  PASS: same rect on F and B is legal")


def test_severity_override_reaches_the_violation():
    i = intent_from_dict(_base(
        severity={'intent_zone_overlap': WARN},
        blocks=[{'name': 'a', 'refs': ['U1'], 'zone': [0, 0, 40, 30]},
                {'name': 'b', 'refs': ['U2'], 'zone': [30, 0, 70, 30]}]))
    v = [x for x in validate_intent(i) if x.rule == 'intent_zone_overlap']
    assert v[0].severity == WARN, v[0]
    assert i.severity_of('envelope') == ERROR         # untouched rules default
    print("  PASS: per-rule severity downgrades one rule and no other")


def test_violation_order_is_a_property_of_the_finding():
    """#457: anything whose order can reach a report must not depend on dict
    iteration. sort_key is what the emitters sort on."""
    i = intent_from_dict(_base(blocks=[
        {'name': 'c', 'refs': ['U3'], 'zone': [0, 0, 40, 30]},
        {'name': 'a', 'refs': ['U1'], 'zone': [30, 0, 70, 30]},
        {'name': 'b', 'refs': ['U2'], 'zone': [200, 200, 240, 230]}]))
    keys = [v.sort_key() for v in sorted(validate_intent(i),
                                         key=lambda v: v.sort_key())]
    assert keys == sorted(keys)
    assert len(set(keys)) == len(keys), "sort_key collides between findings"
    print(f"  PASS: {len(keys)} findings totally ordered by (rule, ref, block)")


def test_load_intent_reports_the_path_and_the_parse_error():
    d = tempfile.mkdtemp()
    good = os.path.join(d, 'intent.json')
    with open(good, 'w', encoding='utf-8') as fh:
        json.dump(_base(), fh)
    i = load_intent(good)
    assert i.source_path == good

    bad = os.path.join(d, 'broken.json')
    with open(bad, 'w', encoding='utf-8') as fh:
        fh.write('{"schema": 1, "kind": "floorplan-intent",')
    try:
        load_intent(bad)
        raise AssertionError("truncated JSON not rejected")
    except IntentError as exc:
        assert 'broken.json' in str(exc), exc

    try:
        load_intent(os.path.join(d, 'nope.json'))
        raise AssertionError("missing file not rejected")
    except IntentError as exc:
        assert 'nope.json' in str(exc), exc
    print("  PASS: truncated and missing files both name the path")


TESTS = [
    test_a_wellformed_intent_loads,
    test_rects_are_normalised_not_trusted,
    test_the_wrong_file_is_refused_by_kind,
    test_schema_version_is_enforced,
    test_unknown_keys_are_refused_not_ignored,
    test_the_key_sets_are_exactly_what_is_documented,
    test_an_intent_using_every_known_key_loads,
    test_min_reader_refuses_a_build_that_would_ignore_the_claim,
    test_a_non_string_key_is_an_intent_error_not_a_TypeError,
    test_unknown_nested_keys_are_refused_not_ignored,
    test_a_nested_object_must_be_an_object,
    test_context_is_deliberately_open,
    test_an_entry_carries_its_own_context_slot,
    test_severity_keys_are_checked_against_the_rule_names,
    test_every_violation_rule_name_is_settable,
    test_a_block_needs_refs_or_group,
    test_refs_as_a_bare_string_is_refused,
    test_enumerations_are_checked,
    test_malformed_rects_are_refused,
    test_duplicate_block_names_are_refused,
    test_keepout_needs_a_shape,
    test_zone_outside_the_envelope_is_a_self_contradiction,
    test_two_zones_fighting_over_one_rectangle,
    test_zones_on_opposite_sides_may_share_a_rectangle,
    test_severity_override_reaches_the_violation,
    test_violation_order_is_a_property_of_the_finding,
    test_load_intent_reports_the_path_and_the_parse_error,
]


if __name__ == '__main__':
    for t in TESTS:
        print(f"--- {t.__name__}")
        t()
    print("ALL PASS")
