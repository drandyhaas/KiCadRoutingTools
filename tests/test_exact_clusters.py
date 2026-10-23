"""exact_clusters / exact_unconnected: the deterministic oracle (#490).

kicad-cli DRC's threaded connectivity is nondeterministic (rp2040 gave
three different unconnected-item reports on one unchanged board; welds
chasing them made orangecrab's repair grade 103/65/92 across identical
runs). pcbnew's ZONE_FILLER is measured-deterministic, so KiCad fill truth
+ a deterministic union-find replaces the kicad-cli link source.

Asserts against the island-semantics fixtures whose KiCad link demands
were proven with kicad-cli directly (each board: exactly 1 link = 2
clusters), including the thermal-relief subtlety (a THT pad's fill
EXCLUDES the pad area -- spokes land on the ring, so containment at the
pad center must not be the join test). Skips cleanly without pcbnew.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.join(os.path.dirname(__file__), '..'), 'py_router'))  # #522
sys.path.insert(0, os.path.join(os.path.join(os.path.dirname(__file__), '..'), 'py_tools'))  # #522

from kicad_parser import parse_kicad_pcb  # noqa: E402
from kicad_exact_fill import (find_kicad_python, refill_islands,  # noqa: E402
                              exact_clusters, exact_unconnected)

FIX = os.path.join(os.path.dirname(__file__), 'fixtures', 'island_semantics')


def main():
    if find_kicad_python() is None:
        print("SKIP: pcbnew python unavailable")
        sys.exit(77)   # a skip, not a pass (run_all counts exit 77 apart)
    for name in ('two_islands_tht', 'two_islands_smd_stub_via',
                 'two_islands_via_only'):
        b = os.path.join(FIX, name + '.kicad_pcb')
        pcb = parse_kicad_pcb(b)
        gid = next(n for n, net in pcb.nets.items() if net.name == 'GND')
        m = refill_islands(b)
        assert m is not None, name
        islands = [(k[1], poly) for k, v in m.items() if k[0] == 'GND'
                   for poly in v]
        cl = exact_clusters(pcb, gid, islands)
        assert len(cl) == 2, (name, len(cl))
        # Determinism: same verdict and structure twice.
        cl2 = exact_clusters(pcb, gid, islands)
        assert [sorted(c['islands']) for c in cl] == \
            [sorted(c['islands']) for c in cl2], name

        links = exact_unconnected(b, ['GND'])
        assert links is not None and len(links) == 1, (name, links)
        assert links[0][0] == 'GND'
        ax, ay = links[0][1][0], links[0][1][1]
        bx, by = links[0][2][0], links[0][2][1]
        assert (ax, ay) != (bx, by), \
            f"{name}: link endpoints degenerate (same point)"
        print(f"  {name}: 2 clusters, 1 link "
              f"({ax:.1f},{ay:.1f})<->({bx:.1f},{by:.1f})")
    # Debris semantics (XTAL_O class): a net WITH pads whose only track
    # component is pad-less must emit a debris link; a pad-less net with a
    # single component demands nothing (one cluster, no ratsnest).
    from types import SimpleNamespace
    import kicad_exact_fill as kef
    seg = SimpleNamespace(start_x=50.0, start_y=50.0, end_x=52.0,
                          end_y=50.0, width=0.15, layer='F.Cu', net_id=9)
    pad = SimpleNamespace(global_x=10.0, global_y=10.0, size_x=1.0,
                          size_y=1.0, net_id=9, layers=['F.Cu'], drill=0,
                          pad_type='smd')
    pcb = SimpleNamespace(segments=[seg], vias=[], pads_by_net={9: [pad]},
                          nets={9: SimpleNamespace(name='XT')})
    _orig = kef.refill_islands
    try:
        kef.refill_islands = lambda *a, **k: {}
        out = kef.exact_unconnected('/dev/null', [], pcb_data=pcb)
        assert out and out[0][0] == 'XT', out
        pcb.pads_by_net = {}
        pcb.nets[9] = SimpleNamespace(name='XT')
        out = kef.exact_unconnected('/dev/null', [], pcb_data=pcb)
        assert out == [], out
    finally:
        kef.refill_islands = _orig

    print("PASS: exact clusters/links match proven KiCad demands, "
          "deterministically (+ debris semantics)")


if __name__ == '__main__':
    main()
