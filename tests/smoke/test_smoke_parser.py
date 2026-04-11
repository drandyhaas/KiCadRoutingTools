"""
Smoke test — kicad_parser.

Verifies the module imports cleanly and that parse_kicad_pcb() runs
on a minimal synthetic fixture without crashing. This is NOT a
correctness test — it only guards against import-time and
catastrophic-parse regressions on the fork.
"""
from pathlib import Path

import kicad_parser


FIXTURE = Path(__file__).parent / "fixtures" / "minimal.kicad_pcb"


def test_import_kicad_parser():
    """Module imports without side effects."""
    assert hasattr(kicad_parser, "parse_kicad_pcb")
    assert hasattr(kicad_parser, "PCBData")
    assert hasattr(kicad_parser, "detect_kicad_version")


def test_detect_kicad_version_regex():
    """Version detection on a synthetic header string works."""
    assert kicad_parser.detect_kicad_version("(version 20241229)") == 20241229
    assert kicad_parser.detect_kicad_version("no version here") == 0


def test_parse_minimal_fixture():
    """parse_kicad_pcb() loads the minimal fixture without raising."""
    assert FIXTURE.exists(), f"fixture missing: {FIXTURE}"
    pcb = kicad_parser.parse_kicad_pcb(str(FIXTURE))
    assert pcb is not None
    # Minimal smoke assertions — do not assert on counts beyond what
    # the fixture guarantees, to keep this test stable across parser
    # refactors.
    assert pcb.kicad_version == 20241229
    assert isinstance(pcb.segments, list)
    assert isinstance(pcb.vias, list)
    # Fixture has 2 segments + 1 via — presence check only.
    assert len(pcb.segments) >= 1
    assert len(pcb.vias) >= 1
