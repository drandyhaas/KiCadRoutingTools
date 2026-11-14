"""#865: route toward a `.kicad_dru` `via_count (max N)` rule.

KiCad's too_many_vias counts every via of a net against the last `via_count`
rule binding it (`check_drc._via_count_rule_pass` grades it, #1707). The
route step routed a net with the vias its cheapest path took and never
looked at the rule.

A net with a budget is still routed whatever it takes -- completion first --
but a route that would put it over its max is searched again at a higher via
cost, and the result with the fewest vias is kept. A net that still ends
over its max is named (`report`); check_drc flags it as before.

`max 0` is a via ban and is honoured elsewhere (`net_keepouts.planar_nets`,
`DesignRules.via_disallow_rule`); only a positive max is a budget here.
"""

from dataclasses import replace
from typing import Dict, List, Optional, Tuple

#: the via-cost multipliers a route over its budget is searched again at
RETRY_COSTS = (4, 16)


def budgets(rules, pcb_data) -> Dict[int, Tuple[int, str]]:
    """{net_id: (max vias, the rule's source text)} for the nets a
    `via_count` rule gives a positive max."""
    if rules is None or pcb_data is None or not any(
            'via_count' in r.constraints for r in rules.rules):
        return {}
    out: Dict[int, Tuple[int, str]] = {}
    for nid, net in (getattr(pcb_data, 'nets', None) or {}).items():
        if not nid or not getattr(net, 'name', None):
            continue
        c = rules.resolve('via_count', rules.item_for_net(nid, 'via', via_type='through'),
                          None, None)
        if c.max is not None and c.max > 0:
            out[int(nid)] = (int(c.max), str(c.source or ''))
    return out


def install(config, pcb_data) -> None:
    """Read the board's via budgets onto ``config.via_budgets`` (None with
    none). Never raises into an engine."""
    config.via_budgets = None
    try:
        got = budgets(getattr(config, 'rules', None), pcb_data)
    except Exception as e:                                      # noqa: BLE001
        print(f"  ERROR: the board's via_count rules could not be read "
              f"({type(e).__name__}: {e}); this run routes WITHOUT them")
        return
    if got:
        config.via_budgets = got


def mark_honoured(rules, pcb_data) -> None:
    """Name, for the rules report, the via_count rules that set a budget."""
    try:
        got = budgets(rules, pcb_data)
    except Exception:                                           # noqa: BLE001
        return
    if not got:
        return
    by_rule: Dict[str, int] = {}
    for _mx, src in got.values():
        by_rule[src] = by_rule.get(src, 0) + 1
    for r in rules.rules:
        spec = r.constraints.get('via_count')
        if spec is None or r.honoured or not (spec.get('max') or 0) > 0:
            continue
        n = sum(v for s, v in by_rule.items() if r.name in s)
        if not n:
            continue
        r.honoured = (f"via_count max {int(spec['max'])}: {n} net(s) are routed again "
                      f"at a higher via cost when a route would exceed it, and the "
                      f"route with the fewest vias is kept (a net still over it is "
                      f"listed when the step ends)")
        r.router_unhonoured = None


def _new_vias(result) -> int:
    return len((result or {}).get('new_vias') or [])


def _usable(result) -> bool:
    return bool(result) and not result.get('failed')


def route_within_budget(impl, pcb_data, net_id: int, config, obstacles, *args, **kwargs):
    """``impl(pcb_data, net_id, config, obstacles, ...)``, searched again at
    a higher via cost while its result would put the net over its budget."""
    result = impl(pcb_data, net_id, config, obstacles, *args, **kwargs)
    budget = (getattr(config, 'via_budgets', None) or {}).get(int(net_id or 0))
    if budget is None or not _usable(result) or not _new_vias(result):
        return result
    have = sum(1 for v in pcb_data.vias if v.net_id == net_id)
    if have + _new_vias(result) <= budget[0]:
        return result
    best = result
    for mult in RETRY_COSTS:
        again = impl(pcb_data, net_id, replace(config, via_cost=config.via_cost * mult),
                     obstacles, *args, **kwargs)
        if _usable(again) and _new_vias(again) < _new_vias(best):
            best = again
        if have + _new_vias(best) <= budget[0]:
            break
    if best is not result:
        print(f"  via_count: this route took {_new_vias(result)} via(s) with {have} "
              f"on the net already (max {budget[0]}, {budget[1]}); kept the route "
              f"with {_new_vias(best)}")
    return best


def report(config, pcb_data) -> List[dict]:
    """The nets with a budget that the board now holds over it, for the run's
    JSON_SUMMARY.design_rules: [{'net', 'vias', 'max', 'rule'}]."""
    out = []
    got = getattr(config, 'via_budgets', None) or {}
    if not got:
        return out
    counts: Dict[int, int] = {}
    for v in pcb_data.vias:
        if v.net_id in got:
            counts[v.net_id] = counts.get(v.net_id, 0) + 1
    for nid, n in sorted(counts.items()):
        mx, src = got[nid]
        if n > mx:
            net = pcb_data.nets.get(nid)
            out.append({'net': getattr(net, 'name', None) or f'net_{nid}',
                        'vias': n, 'max': mx, 'rule': src})
    return out
