"""
Placement blocks: parsing and derivation (issue #459).

Nothing in this repo grouped footprints before. `Footprint` carried neither its
own uuid nor a sheet path, no parse path read KiCad's `(group ...)` blocks, and
no move translated more than one part. #459 needs blocks to exist before it can
move one.

Corpus reality, measured rather than assumed, and the reason the sources are
ordered the way they are:

  * 0 of the 22 boards git tracks under `kicad_files/` carry a `(group ...)`
    block, so that path is verified
    against a synthetic fixture only.
  * 12 of 22 boards with `(path ...)` have more than one sheet. ulx3s: 11 sheets
    sized 83/34/23/20/20/12. That makes sheet the workhorse.
  * A 2-pad cap sits 0.0-2.6mm (median) from the nearest IC and shares a net with
    it in 93-100% of cases. The net-sharing check is what separates a decoupling
    cap from a cap that merely sits nearby -- on ulx3s it rejects 13 of 70.
  * Raw net-name prefixes are dominated by KiCad's auto-generated `Net-(U1-Pad)`
    names: one bogus "NET" bucket of up to 92 refs spread over 75mm. Filtered,
    what remains is small but real.
"""

import math
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_router'))  # #522
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_placer'))  # placement split
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_tools'))  # #522

from kicad_parser import parse_kicad_pcb
from placement.groups import (GroupError, derive_groups, describe,
                              parse_sources, NETPREFIX_MAX_SPREAD_MM)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
KF = os.path.join(ROOT, 'kicad_files')


def _board(name):
    return os.path.join(KF, name + '.kicad_pcb')


# --- parsing -----------------------------------------------------------------

def test_footprints_carry_uuid_and_sheet_path():
    pcb = parse_kicad_pcb(_board('ulx3s'))
    fps = list(pcb.footprints.values())
    assert all(f.uuid for f in fps), "some footprints have no uuid"
    with_path = [f for f in fps if f.sheet_path]
    assert len(with_path) >= 220, f"only {len(with_path)} footprints carry a path"
    # The LAST component is the symbol; the prefix is the sheet.
    sheets = {'/'.join(p for p in f.sheet_path.split('/') if p)
              .rsplit('/', 1)[0] for f in with_path if f.sheet_path.count('/') > 1}
    assert len(sheets) >= 5, f"expected a multi-sheet board, got {len(sheets)}"


def test_group_members_resolve_by_uuid():
    """(group ...) lists members BY UUID, and its own (uuid ...) is its identity,
    not a member. No corpus board has one, so this is a synthetic fixture."""
    src = open(_board('tigard'), encoding='utf-8').read()
    pcb0 = parse_kicad_pcb(_board('tigard'))
    refs = sorted(pcb0.footprints)[:3]
    uu = [pcb0.footprints[r].uuid for r in refs]
    assert all(uu)
    blk = ('\t(group "PowerBlock"\n'
           '\t\t(uuid "aaaaaaaa-bbbb-cccc-dddd-eeeeeeeeeeee")\n\t\t(members\n'
           + ''.join(f'\t\t\t"{u}"\n' for u in uu) + '\t\t)\n\t)\n')
    close = src.rindex(')')
    fd, path = tempfile.mkstemp(suffix='.kicad_pcb')
    os.close(fd)
    open(path, 'w', encoding='utf-8').write(src[:close] + blk + src[close:])
    try:
        got = parse_kicad_pcb(path).groups
        assert got == {'PowerBlock': sorted(refs)}, got
    finally:
        os.unlink(path)


def test_ambiguous_uuid_pulls_in_every_claimant():
    """A uuid is supposed to be unique and in real exports is -- but hand-made
    boards repeat them (cap_chain shares one across two footprints). A uuid->ref
    dict would silently resolve a member to the WRONG part."""
    pcb0 = parse_kicad_pcb(_board('cap_chain'))
    dup = [r for r, f in pcb0.footprints.items()
           if f.uuid and sum(1 for g in pcb0.footprints.values()
                             if g.uuid == f.uuid) > 1]
    assert dup, "fixture assumption: cap_chain has duplicate footprint uuids"
    src = open(_board('cap_chain'), encoding='utf-8').read()
    u = pcb0.footprints[dup[0]].uuid
    blk = ('\t(group "Dup"\n\t\t(uuid "aaaaaaaa-bbbb-cccc-dddd-eeeeeeeeeeee")\n'
           f'\t\t(members\n\t\t\t"{u}"\n\t\t)\n\t)\n')
    close = src.rindex(')')
    fd, path = tempfile.mkstemp(suffix='.kicad_pcb')
    os.close(fd)
    open(path, 'w', encoding='utf-8').write(src[:close] + blk + src[close:])
    try:
        got = parse_kicad_pcb(path).groups['Dup']
        assert len(got) == sum(1 for f in pcb0.footprints.values() if f.uuid == u), got
        assert got == sorted(got), "membership must be sorted (#457)"
    finally:
        os.unlink(path)


def test_boards_without_groups_parse_to_an_empty_dict():
    """0 of the 22 tracked boards have one; absence is normal, not an error."""
    for b in ('tigard', 'watchy', 'ulx3s'):
        assert parse_kicad_pcb(_board(b)).groups == {}


# --- source selection --------------------------------------------------------

def test_parse_sources():
    assert parse_sources(None) == ()
    assert parse_sources('none') == ()
    assert parse_sources('auto') == ('kicad', 'sheet')
    assert parse_sources('sheet') == ('sheet',)
    assert parse_sources('sheet, decap') == ('sheet', 'decap')
    assert parse_sources('sheet,sheet') == ('sheet',), "duplicates collapse"
    try:
        parse_sources('bogus')
    except GroupError as e:
        assert 'bogus' in str(e) and 'kicad' in str(e)
        return
    raise AssertionError("an unknown source must be rejected")


# --- derivation --------------------------------------------------------------

def test_sheet_blocks_on_a_real_board():
    pcb = parse_kicad_pcb(_board('ulx3s'))
    g = derive_groups(pcb, ('sheet',))
    assert len(g) >= 8, f"expected ~10 sheet blocks on ulx3s, got {len(g)}"
    biggest = max(len(v) for v in g.values())
    assert biggest > 50, f"largest sheet block is only {biggest} parts"
    assert all(k.startswith('sheet:') for k in g)
    assert all(v == sorted(v) for v in g.values()), "membership must be sorted"
    print(f"  PASS: ulx3s -> {len(g)} sheet blocks, "
          f"sizes {sorted((len(v) for v in g.values()), reverse=True)[:6]}")


def test_top_level_parts_do_not_collapse_into_one_block():
    """A bare single-uuid path is top level, i.e. an EMPTY sheet prefix. Bucketing
    on that would invent a block out of unrelated parts."""
    pcb = parse_kicad_pcb(_board('rp2350_fpga_eensy_prePlane'))
    g = derive_groups(pcb, ('sheet',))
    assert 'sheet:' not in g and '' not in g
    top = [r for r, f in pcb.footprints.items()
           if f.sheet_path and f.sheet_path.count('/') <= 1]
    grouped = {r for v in g.values() for r in v}
    assert not (set(top) & grouped), f"top-level parts were grouped: {set(top) & grouped}"


def test_a_ref_belongs_to_at_most_one_block():
    pcb = parse_kicad_pcb(_board('glasgow_revC'))
    g = derive_groups(pcb, ('sheet', 'netprefix', 'decap'))
    seen = {}
    for name, refs in g.items():
        for r in refs:
            assert r not in seen, f"{r} in both {seen[r]} and {name}"
            seen[r] = name


def test_precedence_kicad_outranks_sheet():
    """An explicit KiCad group must not be torn apart by a later rule."""
    src = open(_board('ulx3s'), encoding='utf-8').read()
    pcb0 = parse_kicad_pcb(_board('ulx3s'))
    # Pick three refs that sheet grouping would otherwise split apart.
    by_sheet = {}
    for r, f in pcb0.footprints.items():
        parts = [q for q in (f.sheet_path or '').split('/') if q]
        if len(parts) > 1:
            by_sheet.setdefault('/'.join(parts[:-1]), []).append(r)
    sheets = [s for s, v in sorted(by_sheet.items()) if len(v) >= 2][:2]
    picked = sorted(by_sheet[sheets[0]])[:2] + sorted(by_sheet[sheets[1]])[:1]
    uu = [pcb0.footprints[r].uuid for r in picked]
    blk = ('\t(group "Cross"\n\t\t(uuid "aaaaaaaa-bbbb-cccc-dddd-eeeeeeeeeeee")\n'
           '\t\t(members\n' + ''.join(f'\t\t\t"{u}"\n' for u in uu) + '\t\t)\n\t)\n')
    close = src.rindex(')')
    fd, path = tempfile.mkstemp(suffix='.kicad_pcb')
    os.close(fd)
    open(path, 'w', encoding='utf-8').write(src[:close] + blk + src[close:])
    try:
        g = derive_groups(parse_kicad_pcb(path), ('kicad', 'sheet'))
        assert g.get('kicad:Cross') == sorted(picked), g.get('kicad:Cross')
        for name, refs in g.items():
            if name != 'kicad:Cross':
                assert not (set(refs) & set(picked)), f"{name} stole a member"
    finally:
        os.unlink(path)


def test_decap_requires_a_shared_net_not_just_proximity():
    """Proximity alone adopts caps that have no electrical relationship with the
    IC -- 13 of ulx3s' 70. Every decap block member must share a net with its IC."""
    pcb = parse_kicad_pcb(_board('tigard'))
    g = derive_groups(pcb, ('decap',))
    assert g, "expected decap blocks on tigard (a flat board with no sheets)"
    for name, refs in g.items():
        ic = name.split(':', 1)[1]
        assert ic in refs, f"{name} must contain its anchor IC"
        ic_nets = {p.net_id for p in pcb.footprints[ic].pads if p.net_id > 0}
        for r in refs:
            if r == ic:
                continue
            cap_nets = {p.net_id for p in pcb.footprints[r].pads if p.net_id > 0}
            assert cap_nets & ic_nets, f"{r} shares no net with {ic}"
    print(f"  PASS: tigard -> {len(g)} decap blocks, "
          f"{sum(len(v) for v in g.values())} parts")



def test_a_collinear_edge_row_is_not_an_ic_a_cap_can_decouple():
    """`build_chip_list` qualifies a footprint on PAD COUNT alone, so a 1xN
    castellated breakout row (10 pads) is an "IC". Such a row spans a whole board
    edge and carries a rail, so it is nearer to half the decaps than their real
    IC and it passes the shared-net test -- it captures them.

    The false positive is noise. The FALSE NEGATIVE is why the filter exists: a
    cap 2mm from the row and 8mm from the part it actually decouples grades
    CLEAN, because the distance was measured to the row.
    """
    from placement.groups import _pads_are_collinear, decap_tethers
    pcb = parse_kicad_pcb(_board('tigard'))
    collinear = {r for r, fp in pcb.footprints.items()
                 if len(fp.pads) >= 4 and _pads_are_collinear(fp)}
    # whatever they are on this board, none of them may anchor a decap block
    tethers = decap_tethers(pcb)
    assert not (collinear & set(tethers)),         f"a collinear row anchored decaps: {sorted(collinear & set(tethers))}"

    # The corpus may hold no 1xN row at all, which would make the assert above
    # vacuous, so discriminate on constructed geometry too. A stub is enough:
    # the predicate reads nothing but pad coordinates.
    class _P:
        def __init__(self, x, y):
            self.global_x, self.global_y = x, y

    class _F:
        def __init__(self, pts):
            self.pads = [_P(*q) for q in pts]

    row = _F([(i * 2.54, 0.0) for i in range(10)])      # 1x10 header / castellation
    col = _F([(0.0, i * 2.54) for i in range(10)])      # the same, rotated 90
    soic = _F([(-3.5, y) for y in (-1.9, -0.6, 0.6, 1.9)] +
              [(3.5, y) for y in (-1.9, -0.6, 0.6, 1.9)])
    assert _pads_are_collinear(row), "a 1x10 row must read as collinear"
    assert _pads_are_collinear(col), "rotation must not change the verdict"
    assert not _pads_are_collinear(soic), "a SOIC-8 is 2-D and must be kept"
    assert not _pads_are_collinear(_F([]))              # no pads: not a row

    two_d = [r for r, fp in pcb.footprints.items()
             if len(fp.pads) >= 4 and not _pads_are_collinear(fp)]
    assert two_d, "no 2-D multi-pad part on tigard -- fixture assumption broke"
    print(f"  PASS: row/column excluded, SOIC-8 kept; "
          f"{len(collinear)} collinear part(s) on tigard, {len(two_d)} chip(s)")


def test_a_decap_tethers_on_its_rail_not_on_ground():
    """GND is shared with nearly every part, so matching on "shares a net" lets
    the nearest 4-pad neighbour win on ground alone -- measured, that put a
    VCC3V3 decap on the CRYSTAL and the flash's own cap on the USB connector.

    Also pins the second half: the search is nearest-chip-CARRYING-THE-RAIL, not
    nearest-then-reject. Rejecting after the fact dropped the cap entirely when
    an unrelated part was closer, so a genuinely distant decap went ungraded
    instead of flagged.
    """
    from net_queries import is_ground_net_name
    from placement.groups import decap_tethers
    pcb = parse_kicad_pcb(_board('tigard'))
    tethers = decap_tethers(pcb)
    assert tethers, "expected decap tethers on tigard"
    checked = 0
    for ic, caps in tethers.items():
        ic_nets = {p.net_id for p in pcb.footprints[ic].pads if p.net_id > 0}
        for cap, _d in caps:
            nets = {p.net_id for p in pcb.footprints[cap].pads if p.net_id > 0}
            power = {n for n in nets
                     if not is_ground_net_name(
                         getattr(pcb.nets.get(n), 'name', '') or '')}
            if not power:          # a cap between two grounds: nothing to check
                continue
            checked += 1
            assert power & ic_nets,                 f"{cap} tethered to {ic} on ground alone (rails {power})"
    assert checked >= 5, f"only {checked} rail-bearing caps -- fixture too thin"
    print(f"  PASS: {checked} cap(s), every one sharing a RAIL with its IC")


def test_netprefix_rejects_autogenerated_names():
    """KiCad's Net-(U1-Pad3) / unconnected-(...) names carry no functional
    meaning; bucketing on them builds one huge bogus block per board."""
    pcb = parse_kicad_pcb(_board('glasgow_revC'))
    g = derive_groups(pcb, ('netprefix',))
    for name, refs in g.items():
        assert not name.lower().startswith(('net:net', 'net:unconnected')), name
        assert len(refs) < 60, f"{name} has {len(refs)} refs -- looks auto-generated"


def test_netprefix_requires_spatial_coherence():
    pcb = parse_kicad_pcb(_board('ulx3s'))
    g = derive_groups(pcb, ('netprefix',))
    for name, refs in g.items():
        pts = []
        for r in refs:
            fp = pcb.footprints[r]
            xs = [p.global_x for p in fp.pads]
            ys = [p.global_y for p in fp.pads]
            pts.append((sum(xs) / len(xs), sum(ys) / len(ys)) if xs else (fp.x, fp.y))
        cx = sum(p[0] for p in pts) / len(pts)
        cy = sum(p[1] for p in pts) / len(pts)
        spread = max(math.hypot(p[0] - cx, p[1] - cy) for p in pts)
        assert spread <= NETPREFIX_MAX_SPREAD_MM + 1e-6, \
            f"{name} spans {spread:.1f}mm -- a naming convention, not a block"


def test_no_sources_means_no_groups():
    pcb = parse_kicad_pcb(_board('ulx3s'))
    assert derive_groups(pcb, ()) == {}
    assert derive_groups(pcb, parse_sources('none')) == {}


def test_movable_restricts_membership():
    pcb = parse_kicad_pcb(_board('ulx3s'))
    full = derive_groups(pcb, ('sheet',))
    some = sorted({r for v in full.values() for r in v})[:20]
    limited = derive_groups(pcb, ('sheet',), movable=some)
    assert {r for v in limited.values() for r in v} <= set(some)


def test_describe_is_readable():
    pcb = parse_kicad_pcb(_board('ulx3s'))
    txt = describe(derive_groups(pcb, ('sheet',)), limit=2)
    assert 'block(s)' in txt and 'sheet:' in txt
    assert len([l for l in txt.splitlines()]) <= 5
    assert describe({}) .endswith('none')


TESTS = [
    test_footprints_carry_uuid_and_sheet_path,
    test_group_members_resolve_by_uuid,
    test_ambiguous_uuid_pulls_in_every_claimant,
    test_boards_without_groups_parse_to_an_empty_dict,
    test_parse_sources,
    test_sheet_blocks_on_a_real_board,
    test_top_level_parts_do_not_collapse_into_one_block,
    test_a_ref_belongs_to_at_most_one_block,
    test_precedence_kicad_outranks_sheet,
    test_decap_requires_a_shared_net_not_just_proximity,
    test_a_collinear_edge_row_is_not_an_ic_a_cap_can_decouple,
    test_a_decap_tethers_on_its_rail_not_on_ground,
    test_netprefix_rejects_autogenerated_names,
    test_netprefix_requires_spatial_coherence,
    test_no_sources_means_no_groups,
    test_movable_restricts_membership,
    test_describe_is_readable,
]


if __name__ == '__main__':
    for t in TESTS:
        print(f"--- {t.__name__}")
        t()
    print("ALL PASS")
