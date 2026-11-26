"""Classify a KiCad DRC report against what this router's rule resolver knows.

KiCad's DRC engine is the authority: each violation it reports names the rule
that governs it and the value that rule demands ("Track width (rule 'X' min
width 0.1800 mm; actual 0.1016 mm)"). For every such violation this asks
`DesignRules` -- the one resolver every routing step reads -- for the same
constraint on the same item(s) and layer, and sorts the violation into:

  unknown   the resolver's answer does not forbid what KiCad forbids: a rule
            (or rule source) the router does not read, so no routing step
            could have honoured it
  bypassed  the resolver knows the limit, but the size the router itself chose
            (track width, via diameter, drill, annular ring) breaks it: a
            routing step sized copper without asking the resolver
  other     the resolver knows the limit and the violation is spacing or
            connectivity (a routing outcome, not a rule-reading defect), or
            the report names no value to compare

`unknown` + `bypassed` is "DRC from rules unknown to the router": both mean
the router did not apply a rule KiCad enforces. The target is 0.

    python py_router/drc_rule_audit.py BOARD.kicad_pcb DRC.json [--json OUT]
"""
from __future__ import annotations

import argparse
import collections
import json
import os
import re
import sys
from typing import Dict, List, Optional, Tuple

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

EPS = 0.0005  # mm; KiCad prints four decimals

#: DRC violation type -> (resolver kind, router-chosen size?)
TYPE_KIND = {
    'track_width': ('track_width', True),
    'via_diameter': ('via_diameter', True),
    'hole_size': ('hole_size', True),         # resolver kind for drill
    'annular_width': ('annular_width', True),
    'clearance': ('clearance', False),
    'hole_clearance': ('hole_clearance', False),
    'hole_to_hole': ('hole_to_hole', False),
    'copper_edge_clearance': ('edge_clearance', False),
    'diff_pair_gap_out_of_range': ('diff_pair_gap', False),
    'diff_pair_uncoupled_length_too_long': ('diff_pair_uncoupled', False),
    'length_out_of_range': ('length', False),
    'skew_out_of_range': ('skew', False),
    'too_many_vias': ('via_count', False),
    'connection_width': ('connection_width', False),
}

# "(rule 'NAME' min width 0.1800 mm; actual 0.1016 mm)"
# "(netclass 'Default' clearance 0.2000 mm; actual 0.1500 mm)"
# "(board setup constraints min width 0.1000 mm; actual ...)"
_SRC = re.compile(r"\((?:rule '(?P<rule>[^']*)'|netclass '(?P<nc>[^']*)'|"
                  r"(?P<other>[a-z][a-z ]*?(?:constraints|override|clearance override)))"
                  r"\s+(?P<what>[a-z _]*?)\s*(?P<val>-?[0-9.]+) mm;\s*actual\s+(?P<act>-?[0-9.]+) mm\)")
_NET = re.compile(r"\[([^\]]*)\]")
_LAYER = re.compile(r"\bon ([A-Za-z0-9_.]+\.Cu)\b")
_PAD = re.compile(r"\bpad (\S+) \[[^\]]*\] of (\S+)")


def parse_violation(v: dict) -> Optional[dict]:
    m = _SRC.search(v.get('description', ''))
    if not m:
        return None
    what = m.group('what').strip()
    bound = 'max' if what.startswith('max') else 'min'
    return {'rule': m.group('rule'), 'netclass': m.group('nc'),
            'source': (f"rule '{m.group('rule')}'" if m.group('rule') is not None else
                       f"netclass '{m.group('nc')}'" if m.group('nc') is not None
                       else m.group('other')),
            'bound': bound, 'value': float(m.group('val')),
            'actual': float(m.group('act'))}


class Auditor:
    def __init__(self, board_path: str):
        from kicad_parser import parse_kicad_pcb
        from design_rules import DesignRules
        self.pcb = parse_kicad_pcb(board_path)
        self.rules = DesignRules.from_project(self.pcb, board_path)
        self.net_ids: Dict[str, int] = {}
        for nid, n in self.pcb.nets.items():
            if getattr(n, 'name', None):
                self.net_ids.setdefault(n.name, nid)
        self.pads = {(fp.reference, str(p.pad_number)): p
                     for fp in self.pcb.footprints.values() for p in fp.pads}

    def item(self, desc: str, layer: Optional[str]):
        """RuleItem for one DRC item description, or None (board edge etc.)."""
        low = desc.lower()
        nm = _NET.search(desc)
        net = nm.group(1) if nm else ''
        if net.startswith('<no net>'):
            net = ''
        nid = self.net_ids.get(net, 0)
        if 'pad ' in low or low.startswith('pad'):
            pm = _PAD.search(desc) or re.search(r"\bPad (\S+) \[[^\]]*\] of (\S+)", desc)
            if pm:
                pad = self.pads.get((pm.group(2), pm.group(1)))
                if pad is not None:
                    return self.rules.item_for_pad(pad, layer)
            return self.rules.item_for_net(nid, 'pad', layer)
        for word, typ in (('via', 'via'), ('track', 'track'), ('arc', 'track'),
                          ('zone', 'zone')):
            if low.startswith(word):
                return self.rules.item_for_net(nid, typ, layer)
        if nm:
            return self.rules.item_for_net(nid, 'track', layer)
        return None

    def classify(self, v: dict) -> Tuple[str, str]:
        typ = v.get('type', '')
        if typ not in TYPE_KIND:
            return 'other', f'{typ}: not a rule-valued check'
        kind, chosen = TYPE_KIND[typ]
        p = parse_violation(v)
        if p is None:
            return 'other', f'{typ}: no value in the report'
        items = v.get('items') or []
        layer = None
        for it in items:
            lm = _LAYER.search(it.get('description', ''))
            if lm:
                layer = lm.group(1)
                break
        rule_items = [self.item(it.get('description', ''), layer) for it in items]
        a = rule_items[0] if rule_items else None
        b = rule_items[1] if len(rule_items) > 1 else None
        if a is None:
            if kind == 'edge_clearance' and b is not None:
                a, b = b, None
            else:
                return 'unknown', f"{typ}: item not resolvable ({p['source']})"
        try:
            c = self.rules.resolve_for_copper(kind, a, b, layer)
        except Exception as e:                                 # noqa: BLE001
            return 'unknown', f"{typ}: resolver raised {e!r} ({p['source']})"
        if p['bound'] == 'max':
            known = c.max is not None and c.max <= p['value'] + EPS
            have = c.max
        else:
            have = c.kicad_min if c.kicad_min is not None else c.min
            known = have is not None and have >= p['value'] - EPS
        tag = f"{typ} {p['bound']} {p['value']:.4f} by {p['source']}; resolver {have}"
        if not known:
            return 'unknown', tag
        return ('bypassed' if chosen else 'other'), tag


def audit(board_path: str, drc_path: str) -> dict:
    drc = json.load(open(drc_path, encoding='utf-8'))
    au = Auditor(board_path)
    counts = collections.Counter()
    why = collections.Counter()
    for v in drc.get('violations', []):
        if v.get('severity') not in (None, 'error'):
            continue
        cls, tag = au.classify(v)
        counts[cls] += 1
        why[(cls, re.sub(r'actual [-0-9.]+', '', tag))] += 1
    for v in drc.get('unconnected_items', []):
        counts['other'] += 1
    return {'board': board_path, 'drc': drc_path,
            'counts': {k: counts.get(k, 0) for k in ('unknown', 'bypassed', 'other')},
            'ruleUnknown': counts.get('unknown', 0) + counts.get('bypassed', 0),
            'top': [{'class': k[0], 'what': k[1], 'n': n} for k, n in why.most_common(20)]}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('board')
    ap.add_argument('drc')
    ap.add_argument('--json')
    a = ap.parse_args(argv)
    r = audit(a.board, a.drc)
    c = r['counts']
    print(f"drc_rule_audit: unknown {c['unknown']}, bypassed {c['bypassed']}, "
          f"other {c['other']} (rule unknown to router: {r['ruleUnknown']})")
    for t in r['top']:
        if t['class'] != 'other':
            print(f"  {t['n']:5d} {t['class']:8s} {t['what']}")
    if a.json:
        with open(a.json, 'w', encoding='utf-8') as f:
            json.dump(r, f, indent=1)
    return 0


if __name__ == '__main__':
    sys.exit(main())
