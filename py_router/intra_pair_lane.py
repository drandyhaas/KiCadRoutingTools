"""#1869: the lane a differential pair is length-matched, recorded and graded
on -- the copper path between the two pads that end it -- instead of each
member's total copper.

KiCad's skew DRC sums all of a net's copper, and that stays the default. A
pair with a branch to a third pad (a protection part, a termination) cannot
be matched in both senses: matched pad to pad its total copper differs by the
branches, matched in total copper its signal paths do. With a lane, the pair
step matches the path between the lane's pads, records them with the
tolerance, and check_drc grades the recorded tolerance on that path and
reports each member's branch as a stub.

A spec is `auto` (each member's lane ends, read off its copper by
`net_queries.pin_pair_pads`) or one or more `P_NET=REF.PAD:REF.PAD`, the P
member's two pads; the N member's are its pads on the same two parts.
"""

from typing import Dict, List, Optional, Sequence, Tuple

AUTO = 'auto'


def resolve(spec: Optional[Sequence[str]], diff_pair_ids, pcb_data
            ) -> Optional[Dict[int, object]]:
    """{net id: AUTO or (pad, pad)} for both members of every pair a lane is
    asked for, or None with no spec. Raises ValueError, with the reason, for
    a spec that names a net no pair of the run has, or pads its nets lack."""
    if not spec:
        return None
    if isinstance(spec, str):
        spec = [spec]
    spec = [str(s).strip() for s in spec if str(s).strip()]
    if not spec:
        return None
    pairs = list(diff_pair_ids or ())
    out: Dict[int, object] = {}
    if any(s.lower() == AUTO for s in spec):
        if len(spec) > 1:
            raise ValueError("--diff-pair-intra-lane: 'auto' stands alone; name "
                             "lanes as P_NET=REF.PAD:REF.PAD or give auto")
        for _name, pair in pairs:
            out[pair.p_net_id] = AUTO
            out[pair.n_net_id] = AUTO
        return out
    by_p = {pair.p_net_name: pair for _n, pair in pairs}
    by_n = {pair.n_net_name: pair for _n, pair in pairs}
    for s in spec:
        net, sep, pads = s.partition('=')
        a, sep2, b = pads.partition(':')
        if not sep or not sep2 or not a or not b:
            raise ValueError(f"--diff-pair-intra-lane {s!r}: expected "
                             f"P_NET=REF.PAD:REF.PAD or auto")
        pair = by_p.get(net) or by_n.get(net)
        if pair is None:
            raise ValueError(f"--diff-pair-intra-lane {s!r}: {net} is not a member "
                             f"of a pair this step routes")
        given, other = ((pair.p_net_id, pair.n_net_id) if net == pair.p_net_name
                        else (pair.n_net_id, pair.p_net_id))
        from net_queries import pin_pair_pads, pad_name, _copper_pads
        try:
            first = pin_pair_pads(pcb_data, given, [a, b])
        except ValueError as e:
            raise ValueError(f"--diff-pair-intra-lane {s!r}: {e}") from None
        twin = []
        for p in first:
            same = sorted(pad_name(q) for q in _copper_pads(pcb_data, other)
                          if q.component_ref == p.component_ref)
            if len(same) != 1:
                oname = pcb_data.nets[other].name
                raise ValueError(
                    f"--diff-pair-intra-lane {s!r}: {oname} has "
                    f"{len(same)} pad(s) on {p.component_ref} "
                    f"({', '.join(same) or 'none'}), so its end of the lane is "
                    f"not decided; name that member's lane too")
            twin.append(same[0])
        out[given] = (a, b)
        out.setdefault(other, tuple(twin))
    return out


def lane_pins(config, pcb_data, net_id: int) -> Optional[List[str]]:
    """The two pad names the net's lane runs between, or None with no lane.
    An `auto` lane is read off the copper as it now stands."""
    lanes = getattr(config, 'intra_pair_lanes', None)
    if not lanes:
        return None
    entry = lanes.get(net_id)
    if entry is None:
        return None
    if entry == AUTO:
        from net_queries import pin_pair_pads, pad_name
        pair = pin_pair_pads(pcb_data, net_id)
        return None if pair is None else [pad_name(pair[0]), pad_name(pair[1])]
    return list(entry)


def member_lengths(config, pcb_data, p_net_id: int, n_net_id: int
                   ) -> Optional[Tuple[Optional[float], Optional[float], list, list]]:
    """(P length, N length, P pads, N pads) on the pair's lane, or None when
    the pair has no lane. A length is None while its two pads are not joined."""
    lp, ln = lane_pins(config, pcb_data, p_net_id), lane_pins(config, pcb_data, n_net_id)
    if not lp or not ln:
        return None
    from net_queries import net_match_length
    return (net_match_length(pcb_data, p_net_id, 'pin_pair', pins=lp),
            net_match_length(pcb_data, n_net_id, 'pin_pair', pins=ln), lp, ln)


def pair_skew(config, pcb_data, p_net_id: int, n_net_id: int) -> Optional[float]:
    """The pair's skew as this run grades it: on its lane when it has one,
    else KiCad's (`net_queries.diff_pair_skew`, total copper)."""
    got = member_lengths(config, pcb_data, p_net_id, n_net_id)
    if got is None:
        from net_queries import diff_pair_skew
        return diff_pair_skew(pcb_data, p_net_id, n_net_id)['skew_mm']
    return None if got[0] is None or got[1] is None else abs(got[0] - got[1])


def stub_lengths(pcb_data, net_id: int, pins) -> Optional[float]:
    """The net's copper off its lane, mm: total copper minus the lane path
    (both with the same pad-to-die left out). None when either is unmeasured."""
    from net_queries import net_match_length
    total = net_match_length(pcb_data, net_id, 'total', include_pad_to_die=False)
    lane = net_match_length(pcb_data, net_id, 'pin_pair', include_pad_to_die=False,
                            pins=list(pins))
    if total is None or lane is None:
        return None
    return max(0.0, total - lane)
