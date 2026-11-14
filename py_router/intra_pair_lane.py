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

A spec is `auto` (the two terminals the pair step itself routed the pair
between: `resolve_auto`, once the pair is routed) or one or more
`P_NET=REF.PAD:REF.PAD`, the P member's two pads; the N member's are its pads
on the same two parts. Either way N's lane ends on the same two parts as P's.

A lane applies to a pair the step intra-matches and no `.kicad_dru` skew rule
binds (`settle`): KiCad grades such a rule on total copper, so a pair it binds
is matched on total copper, and the step says so.
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
        from net_queries import pin_pair_pads, pad_label, _copper_pads
        try:
            first = pin_pair_pads(pcb_data, given, [a, b])
        except ValueError as e:
            raise ValueError(f"--diff-pair-intra-lane {s!r}: {e}") from None
        twin = []
        for p in first:
            same = sorted(pad_label(q) for q in _copper_pads(pcb_data, other)
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


def settle(lanes, diff_pair_ids, tolerances, sources):
    """The lanes that apply, and why the others do not: `(lanes or None,
    {P net id: reason} for each pair whose lane is IGNORED, [pair names the
    step does not intra-match])`. A lane applies to a pair the step matches
    P to N (`tolerances`, resolve_intra_pair_tolerances) that no `.kicad_dru`
    skew rule binds (`sources`' rule): KiCad grades the rule on all of each
    member's copper, so that pair is matched on total copper, as it is
    graded (#1869 review)."""
    if not lanes:
        return None, {}, []
    out, ignored, unmatched = {}, {}, []
    for name, pair in diff_pair_ids or ():
        if pair.p_net_id not in lanes:
            continue
        if pair.p_net_id not in (tolerances or {}):
            unmatched.append(name)
            continue
        rule = (sources or {}).get(pair.p_net_id, (None, None))[1]
        if rule:
            ignored[pair.p_net_id] = (
                f".kicad_dru skew rule '{rule}' binds the pair, and KiCad grades "
                f"it on total copper")
            continue
        out[pair.p_net_id] = lanes[pair.p_net_id]
        out[pair.n_net_id] = lanes[pair.n_net_id]
    return (out or None), ignored, unmatched


def _routed_terminals(result, pcb_data, pair):
    """The pair step's own two end terminals for a routed pair, or (None,
    why). A pair of two terminals is routed between them; a multi-point pair
    between the ends of its coupled legs (`_leg_terms`), which must form one
    path -- a leg deferred to single-ended between two routed legs leaves
    none."""
    if not result or result.get('failed'):
        return None, "the pair step did not route the pair"
    legs = result.get('leg_results')
    if legs:
        terms = [lr.get('_leg_terms') for lr in legs]
        if any(x is None for x in terms):
            return None, "a coupled leg carries no terminals"

        def key(term):
            return frozenset((q.component_ref, q.pad_number) for q in term)
        for x, y in zip(terms, terms[1:]):
            if key(x[1]) != key(y[0]):
                return None, ("its coupled legs do not form one path (a leg "
                              "between them was left single-ended)")
        return (terms[0][0], terms[-1][1]), None
    from diff_pair_multipoint import get_diff_pair_terminals
    terms = get_diff_pair_terminals(pcb_data, pair.p_net_id, pair.n_net_id)
    if len(terms) != 2:
        return None, (f"it has {len(terms)} terminals and no coupled legs "
                      f"recorded")
    return (terms[0], terms[1]), None


def resolve_auto(lanes, diff_pair_ids, routed_results, pcb_data):
    """Settle each `auto` lane on the terminals the pair step routed:
    `(lanes or None, {P net id: reason} for each pair left without one)`.
    P's lane is its pads of the two end terminals, N's the partner pads of
    the same two terminals -- so on the same two parts, or no lane."""
    if not lanes or AUTO not in lanes.values():
        return lanes, {}
    from net_queries import pad_label
    out, ignored = dict(lanes), {}
    for _name, pair in diff_pair_ids or ():
        if out.get(pair.p_net_id) != AUTO:
            continue
        ends, why = _routed_terminals((routed_results or {}).get(pair.p_net_id),
                                      pcb_data, pair)
        lp = ln = None
        if ends is not None:
            got = []
            for term in ends:
                pp = [q for q in term if q.net_id == pair.p_net_id]
                nn = [q for q in term if q.net_id == pair.n_net_id]
                if len(pp) != 1 or len(nn) != 1:
                    why = "a terminal is not one P pad and one N pad"
                    break
                got.append((pp[0], nn[0]))
            else:
                if any(a.component_ref != b.component_ref for a, b in got):
                    why = ("P and N end on different parts ("
                           + ', '.join(f"{pad_label(a)} / {pad_label(b)}"
                                       for a, b in got) + ")")
                else:
                    lp = (pad_label(got[0][0]), pad_label(got[1][0]))
                    ln = (pad_label(got[0][1]), pad_label(got[1][1]))
        if lp is None:
            ignored[pair.p_net_id] = f"no auto lane: {why}"
            out.pop(pair.p_net_id, None)
            out.pop(pair.n_net_id, None)
        else:
            out[pair.p_net_id], out[pair.n_net_id] = lp, ln
    return (out or None), ignored


def lane_pins(config, pcb_data, net_id: int) -> Optional[List[str]]:
    """The two pad names the net's lane runs between, or None with no lane
    (an `auto` lane is None until `resolve_auto` settles it)."""
    lanes = getattr(config, 'intra_pair_lanes', None)
    if not lanes:
        return None
    entry = lanes.get(net_id)
    if entry is None or entry == AUTO:
        return None
    return list(entry)


def member_lengths(config, pcb_data, p_net_id: int, n_net_id: int
                   ) -> Optional[Tuple[Optional[float], Optional[float], list, list]]:
    """(P length, N length, P pads, N pads) on the pair's lane, or None when
    the pair has no lane. A length is None while its two pads are not joined."""
    lp, ln = lane_pins(config, pcb_data, p_net_id), lane_pins(config, pcb_data, n_net_id)
    if not lp or not ln:
        return None
    from net_queries import net_match_length
    return (net_match_length(pcb_data, p_net_id, 'pin_pair', pads=lp),
            net_match_length(pcb_data, n_net_id, 'pin_pair', pads=ln), lp, ln)


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
                            pads=list(pins))
    if total is None or lane is None:
        return None
    return max(0.0, total - lane)
