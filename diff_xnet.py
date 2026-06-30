"""
AC-coupled differential-pair (XNet) detection -- issue #196.

A high-speed differential pair (PCIe / USB3 / SATA / ...) is commonly split into
two separately-named pairs by series DC-blocking capacitors:

    DPA_P --[C1]-- DPB_P        (the full P conductor: driver -> caps -> receiver)
    DPA_N --[C2]-- DPB_N        (the full N conductor)

``route_diff`` sees ``DPA_*`` and ``DPB_*`` as two independent pairs and
length-matches each side's P/N on its own, so the end-to-end skew the receiver
actually sees (the whole P path vs the whole N path) is never matched. This
module detects that topology -- the industry "extended net" / XNet -- so the
matcher can treat the concatenated P and N conductors as one length budget.

Detection is a pure, side-effect-free read of the parsed board.
"""
from dataclasses import dataclass, field
from typing import Dict, List, Tuple

from kicad_parser import PCBData
from routing_config import DiffPairNet


@dataclass
class AcCoupledXNet:
    """A differential signal whose P and N conductors are each split across one
    or more series 2-pad parts (DC-blocking caps), joining 2+ ``DiffPairNet``s
    into one end-to-end pair.

    Attributes:
        members: the per-segment pairs that make up the chain (2 or more).
        bridge_refs: component references of the series parts joining them.
    """
    members: List[DiffPairNet]
    bridge_refs: List[str] = field(default_factory=list)

    @property
    def p_net_ids(self) -> List[int]:
        return [m.p_net_id for m in self.members]

    @property
    def n_net_ids(self) -> List[int]:
        return [m.n_net_id for m in self.members]

    @property
    def base_names(self) -> List[str]:
        return [m.base_name for m in self.members]


def find_ac_coupled_xnets(
    pcb_data: PCBData,
    diff_pairs: Dict[str, DiffPairNet],
) -> Tuple[List[AcCoupledXNet], List[str]]:
    """Detect AC-coupling XNets among already-detected ``diff_pairs``.

    A junction between two pairs A and B is recognized only when BOTH polarities
    are bridged by a distinct 2-pad, non-DNP part (``A.P<->B.P`` AND
    ``A.N<->B.N``) -- the conservative, symmetric case that unambiguously
    identifies one electrical signal split by series DC-blocking caps. A 2-pad
    part with a pad on GND/power (a decoupling cap) is ignored automatically
    because that pad's net is not a differential-pair member, and a no-pop (DNP)
    part is ignored because it is an open circuit. Single-sided / asymmetric
    junctions are skipped and reported as warnings, so the caller keeps today's
    per-side behavior for them.

    ``diff_pairs`` is the dict returned by ``find_differential_pairs``; its keys
    are used as stable pair identifiers.

    Returns ``(xnets, warnings)``. Pure read of ``pcb_data``; mutates nothing.
    """
    # Map every complete diff-pair member net -> (pair_key, 'P'|'N').
    net_to_member: Dict[int, Tuple[str, str]] = {}
    for pair_key, pair in diff_pairs.items():
        if not pair.is_complete:
            continue
        net_to_member[pair.p_net_id] = (pair_key, 'P')
        net_to_member[pair.n_net_id] = (pair_key, 'N')

    # Per unordered pair-of-pairs, record which polarities are bridged by a 2-pad
    # part: junctions[(keyX, keyY)] -> {'P': ref, 'N': ref}.
    junctions: Dict[Tuple[str, str], Dict[str, str]] = {}
    for ref, fp in pcb_data.footprints.items():
        if fp.dnp or len(fp.pads) != 2:
            continue
        n1, n2 = fp.pads[0].net_id, fp.pads[1].net_id
        if not n1 or not n2 or n1 == n2:
            continue
        m1, m2 = net_to_member.get(n1), net_to_member.get(n2)
        if m1 is None or m2 is None:
            continue  # a side is not a diff-pair net (e.g. GND/VCC decoupling cap)
        (key1, pol1), (key2, pol2) = m1, m2
        if key1 == key2 or pol1 != pol2:
            continue  # within one pair, or a cross-polarity (swapped) bridge
        jkey = (key1, key2) if key1 < key2 else (key2, key1)
        junctions.setdefault(jkey, {})[pol1] = ref

    # A coupling edge requires BOTH polarities bridged (symmetric); warn otherwise.
    edges: List[Tuple[str, str, List[str]]] = []
    warnings: List[str] = []
    for (kx, ky), pols in junctions.items():
        if 'P' in pols and 'N' in pols:
            edges.append((kx, ky, [pols['P'], pols['N']]))
        else:
            only = next(iter(pols))
            warnings.append(
                f"AC-couple: pairs '{kx}' and '{ky}' are bridged on the {only} side "
                f"only (via {pols[only]}); not treating as one signal "
                f"(asymmetric coupling -- per-side matching kept)."
            )

    if not edges:
        return [], warnings

    # Union-find over pair keys connected by coupling edges -> XNet groups
    # (handles chains of 2+ caps, e.g. A--B--C, as one group).
    parent: Dict[str, str] = {}

    def find(x: str) -> str:
        parent.setdefault(x, x)
        root = x
        while parent[root] != root:
            root = parent[root]
        while parent[x] != root:
            parent[x], x = root, parent[x]
        return root

    for kx, ky, _refs in edges:
        parent[find(kx)] = find(ky)

    group_keys: Dict[str, List[str]] = {}
    group_refs: Dict[str, List[str]] = {}
    for key in {k for e in edges for k in e[:2]}:
        group_keys.setdefault(find(key), []).append(key)
    for kx, ky, refs in edges:
        group_refs.setdefault(find(kx), []).extend(refs)

    xnets: List[AcCoupledXNet] = []
    for root, keys in group_keys.items():
        members = [diff_pairs[k] for k in sorted(keys) if k in diff_pairs]
        if len(members) < 2:
            continue
        refs = sorted(set(group_refs.get(root, [])))
        xnets.append(AcCoupledXNet(members=members, bridge_refs=refs))
    return xnets, warnings
