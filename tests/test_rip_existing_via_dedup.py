#!/usr/bin/env python3
"""Issue #284: cross-invocation --rip-existing-nets stacked-via de-dup.

When route.py rips an existing (already-routed) net and re-routes it, the
reroute frequently lands a via-in-pad via at the EXACT position of the net's
original via, but shrunk to fit clearance -> a DIFFERENT size/drill. The output
writer copies the input file verbatim, so the original via ships unless it is
listed for removal. The stale-input-via strip decides "keep vs strip" by
comparing each original via against the final-board vias of the same net.

The bug: that comparison keyed on POSITION ALONE. A reroute via at the original
position made the original look "kept" (its (x,y) was present in the final set),
so the original survived verbatim next to the reroute's replacement -> two
same-net vias stacked in one hole (a net-independent drill hole-to-hole
violation). sechzig went 7 -> 555 DRC on exactly this class; the residual after
the #301 fix was 3 such stacked-via drill violations.

The fix keys the signature on (x, y, size, drill): a reroute-superseded original
no longer matches (different size) and is stripped, while a via route.py kept
UNCHANGED still matches its identical final-board via and is left alone.

    python3 tests/test_rip_existing_via_dedup.py
"""
import os
import sys
from types import SimpleNamespace

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_writer import remove_vias_from_content


def _via(x, y, size, drill, net=67, layers=('F.Cu', 'B.Cu')):
    return SimpleNamespace(x=x, y=y, size=size, drill=drill, net_id=net,
                           layers=list(layers))


# Mirror of route.py's #284 signature and the position-only signature it replaced.
def _sig_pos(v):
    return (round(v.x, 3), round(v.y, 3))


def _sig_284(v):
    return (round(v.x, 3), round(v.y, 3), round(v.size, 3), round(v.drill, 3))


def _stale(orig_by_net, final_by_net, sig):
    final_sig = {nid: {sig(v) for v in vs} for nid, vs in final_by_net.items()}
    return [v for nid, vs in orig_by_net.items()
            for v in vs if sig(v) not in final_sig.get(nid, ())]


def run():
    fails = []

    def check(name, cond):
        if not cond:
            fails.append(name)

    # Original input via (0.35/0.2) at a via-in-pad location on net 67.
    orig_via = _via(147.60, 120.81, 0.35, 0.2, net=67)
    # A kept via elsewhere on the same net that route.py did NOT touch.
    kept_via = _via(150.00, 125.00, 0.35, 0.2, net=67)
    orig_by_net = {67: [orig_via, kept_via]}

    # Final board: the reroute placed a SHRUNK via (0.3/0.15) at the same spot as
    # orig_via, and left kept_via unchanged.
    reroute_via = _via(147.60, 120.81, 0.3, 0.15, net=67)
    final_by_net = {67: [reroute_via, _via(150.00, 125.00, 0.35, 0.2, net=67)]}

    # Position-only sig (the bug): the superseded original looks "kept" -> NOT stale.
    stale_pos = _stale(orig_by_net, final_by_net, _sig_pos)
    check("position-only sig MISSES the superseded via (the bug)",
          orig_via not in stale_pos)

    # Size/drill-aware sig (the fix): superseded original is stale; kept via is not.
    stale_284 = _stale(orig_by_net, final_by_net, _sig_284)
    check("size-aware sig flags the superseded via stale", orig_via in stale_284)
    check("size-aware sig leaves the genuinely-kept via alone",
          kept_via not in stale_284)
    check("exactly one via flagged stale", len(stale_284) == 1)

    # End-to-end: feeding the stale list to the writer strips the ORIGINAL via
    # block from the verbatim input copy, leaving one hole at that position.
    content = '''(kicad_pcb
\t(version 20250101)
\t(via (at 147.6 120.81) (size 0.35) (drill 0.2) (layers "F.Cu" "B.Cu") (net 67) (uuid "orig"))
\t(via (at 150 125) (size 0.35) (drill 0.2) (layers "F.Cu" "B.Cu") (net 67) (uuid "kept"))
)'''
    out, n = remove_vias_from_content(content, stale_284)
    check("writer removes exactly the superseded original", n == 1)
    check("original via block gone", '"orig"' not in out)
    check("kept via block survives", '"kept"' in out)

    print("=" * 60)
    if fails:
        for f in fails:
            print(f"  FAIL  {f}")
        print(f"\n{len(fails)} failure(s)")
        return 1
    print("  PASS  #284 stacked-via de-dup: size/drill-aware stale-via signature")
    print(f"\n{6}/{6} checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(run())
