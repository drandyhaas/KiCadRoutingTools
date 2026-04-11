"""
Regression test — L45: honor (locked yes) on segments and vias.

Before this fix, the parser's segment/via regexes did not allow the
optional (locked yes) field, so any track/via flagged by the user as
locked was either silently stripped from the parsed representation
(when KiCad wrote it in a position the regex tolerated) or caused the
whole segment/via to be dropped (when it broke the strict field order).

These tests pin the new behavior:
  1. Parsing a locked segment sets Segment.locked == True.
  2. Parsing a locked via     sets Via.locked     == True.
  3. Writer-emitted (locked yes) round-trips through the parser.

Fork backlog ref: Entry #1 (L45 locked tracks).
"""
from pathlib import Path

import kicad_parser
import kicad_writer


FIXTURE = Path(__file__).parent / "fixtures" / "with_locked.kicad_pcb"


def test_parse_locked_segment():
    """Locked segment in fixture parses with locked=True; unlocked remains False."""
    assert FIXTURE.exists(), f"fixture missing: {FIXTURE}"
    pcb = kicad_parser.parse_kicad_pcb(str(FIXTURE))

    assert len(pcb.segments) == 2, f"expected 2 segments, got {len(pcb.segments)}"

    by_uuid = {s.uuid: s for s in pcb.segments}
    locked_seg = by_uuid["aaaaaaaa-aaaa-aaaa-aaaa-aaaaaaaaaaaa"]
    plain_seg = by_uuid["bbbbbbbb-bbbb-bbbb-bbbb-bbbbbbbbbbbb"]

    assert locked_seg.locked is True, "segment with (locked yes) must parse locked=True"
    assert plain_seg.locked is False, "segment without (locked ...) must parse locked=False"


def test_parse_locked_via():
    """Locked via in fixture parses with locked=True."""
    pcb = kicad_parser.parse_kicad_pcb(str(FIXTURE))

    assert len(pcb.vias) == 1, f"expected 1 via, got {len(pcb.vias)}"
    via = pcb.vias[0]

    assert via.uuid == "cccccccc-cccc-cccc-cccc-cccccccccccc"
    assert via.locked is True, "via with (locked yes) must parse locked=True"


def test_roundtrip_locked():
    """Writer emits (locked yes) and the parser picks it up again.

    This guards the writer+parser contract: if either side silently drops
    the locked flag, the round-trip breaks.
    """
    pcb = kicad_parser.parse_kicad_pcb(str(FIXTURE))

    # Locate the locked segment from the fixture, then re-emit it via the
    # writer and verify the emitted s-expr contains (locked yes).
    locked_seg = next(
        s for s in pcb.segments if s.uuid == "aaaaaaaa-aaaa-aaaa-aaaa-aaaaaaaaaaaa"
    )
    seg_sexpr = kicad_writer.generate_segment_sexpr(
        start=(locked_seg.start_x, locked_seg.start_y),
        end=(locked_seg.end_x, locked_seg.end_y),
        width=locked_seg.width,
        layer=locked_seg.layer,
        net_id=locked_seg.net_id,
        locked=locked_seg.locked,
    )
    assert "(locked yes)" in seg_sexpr, "writer must emit (locked yes) when locked=True"

    # Same check for via.
    locked_via = pcb.vias[0]
    via_sexpr = kicad_writer.generate_via_sexpr(
        x=locked_via.x,
        y=locked_via.y,
        size=locked_via.size,
        drill=locked_via.drill,
        layers=locked_via.layers,
        net_id=locked_via.net_id,
        locked=locked_via.locked,
    )
    assert "(locked yes)" in via_sexpr

    # Now embed both emitted s-exprs into a minimal PCB wrapper and
    # re-parse to confirm the parser honors what the writer produced.
    roundtrip_pcb = f"""(kicad_pcb
\t(version 20241229)
\t(generator "tubeforge_l45_roundtrip")
\t(generator_version "9.0")
\t(general (thickness 1.6) (legacy_teardrops no))
\t(paper "A4")
\t(layers
\t\t(0 "F.Cu" signal)
\t\t(31 "B.Cu" signal)
\t\t(44 "Edge.Cuts" user)
\t)
\t(net 0 "")
\t(net 1 "GND")
{seg_sexpr}
{via_sexpr}
)
"""
    tmp = FIXTURE.parent / "_l45_roundtrip_tmp.kicad_pcb"
    tmp.write_text(roundtrip_pcb)
    try:
        reparsed = kicad_parser.parse_kicad_pcb(str(tmp))
        assert len(reparsed.segments) == 1
        assert reparsed.segments[0].locked is True, \
            "round-trip lost locked on segment"
        assert len(reparsed.vias) == 1
        assert reparsed.vias[0].locked is True, \
            "round-trip lost locked on via"
    finally:
        tmp.unlink(missing_ok=True)
