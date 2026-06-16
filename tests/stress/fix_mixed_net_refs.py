#!/usr/bin/env python3
"""Rewrite numeric `(net N)` refs to name-style `(net "name")` in a KiCad 10 board.

Stress-test workaround for two paired findings:
- bga_fanout/qfn_fanout call add_tracks_and_vias_to_pcb() without
  net_id_to_name, writing KiCad-9-style numeric net refs into KiCad 10 boards
  (qfn_fanout/__init__.py:272, bga_fanout/__init__.py:1618).
- kicad_parser extract_segments()/extract_vias() try the numeric regex first
  and use the name-style regex only `if not segments` — a mixed file silently
  loses every name-style element, blinding all downstream tools.

Run this on the OUTPUT of any fanout step before continuing the pipeline:
    python3 fix_mixed_net_refs.py <board.kicad_pcb>

The numeric ids are resolved with the same synthetic id assignment the parser
used when the fanout tool read the board (name discovery order is stable:
names come from pads, which fanout does not change).
"""
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from kicad_parser import parse_kicad_pcb, detect_kicad_version, KICAD_10_MIN_VERSION  # noqa: E402


def main(path: str) -> None:
    p = Path(path)
    content = p.read_text()
    if detect_kicad_version(content) < KICAD_10_MIN_VERSION:
        print(f"{path}: not a KiCad 10 file, nothing to do")
        return
    pcb = parse_kicad_pcb(path)
    id2name = {n.net_id: n.name for n in pcb.nets.values()}
    id2name.setdefault(0, "")
    unresolved = set()

    def repl(m):
        nid = int(m.group(1))
        name = id2name.get(nid)
        if name is None:
            unresolved.add(nid)
            return m.group(0)
        return f'(net "{name}")'

    new, n = re.subn(r"\(net\s+(\d+)\)", repl, content)
    if n:
        p.write_text(new)
    print(f"{path}: rewrote {n} numeric net refs"
          + (f", UNRESOLVED ids left as-is: {sorted(unresolved)}" if unresolved else ""))


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    main(sys.argv[1])
