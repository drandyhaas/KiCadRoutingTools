"""Functional groups for grouped placement: the `groups.json` an agent writes,
and what the board says about it.

`place_simple` keeps each FUNCTIONAL GROUP together through its group term
(`placement/simple.py`). The groups are a judgement -- power
input and its conditioning, the core and its clock, an interface and its
protection -- made by an agent reading the parts, their pin functions and their
datasheets. This module does not make that judgement. It loads the file
(`analyze` reports a part in no group or in two as an error, and place_simple
refuses a part in two), and measures what the judgement implies so the agent
can check it:

  * cohesion: per part, its SIGNAL links to its own group and to every other
    group. Ground and board-wide rails are left out (`rail_net_ids`): on a
    small board GND reaches most parts and would make every part look like it
    belongs everywhere;
  * decaps whose IC sits in another group (`decap_conflicts`): place_simple
    leaves such a cap out of its decap term and places it with its own group;
  * bridges: 2-pad parts with signal links into another group (series
    resistors, jumpers) -- they belong to one side and border the other;
  * the inter-group link matrix.

The file (schema 1):

    {"schema": 1,
     "groups": [
       {"name": "core", "role": "core", "refs": ["U3", "C3", "R1"],
        "anchor": "U3", "why": "MCU and its decoupling",
        "edge": null, "side": "F", "near": ["clock"], "apart": []}
     ]}

`refs` may name file-LOCKED parts (a connector placed and locked first): they
are fixed members, the rest of the group is placed around them, and nothing
moves them. `edge` is one of N/S/E/W (the board side a group belongs on),
`near` / `apart` name other groups.

`flippable` names the group's parts that may stand on the other face of the
board -- the agent's judgement of the parts, like the groups (`placement.
faces`). A part keeps the face it has unless the front has no room for it (no
legal pose, or past a decap or claim limit), when place_simple sends it, or
its group's other flippable parts, to the back. A part
not listed never changes face, and a group's `side` puts all of it on one
face whatever this says.

`templates` names the group's TEMPLATE CANDIDATES: chips whose datasheet
layout the agent will copy into an intent `templates[]` row (the
plan-pcb-placement skill's §3.5):

    "templates": [{"chip": "U2", "kind": "switching stage",
                   "why": "buck converter; its datasheet's layout figure"}]

`kind` is one of TEMPLATE_KINDS; `none` records a chip the hints point at
that is no stage, and a `skip` reason a candidate that gets no template (no
layout in its datasheet), so neither is asked about again. The candidates
are the agent's judgement of the parts, like the groups; `template_hints`
lists what the board's pins and nets suggest, as a check on it.
"""
from __future__ import annotations

import json
import re
from typing import Dict, Iterable, List, Optional, Set, Tuple

SCHEMA = 1
EDGES = ('N', 'S', 'E', 'W')
# The intent spells edges out (floorplan._EDGES); a groups file written beside
# one may too, and the full name is unambiguous.
_EDGE_NAMES = {'north': 'N', 'south': 'S', 'east': 'E', 'west': 'W'}
GROUP_KEYS = ('name', 'role', 'refs', 'anchor', 'why', 'edge', 'side', 'near',
              'apart', 'note', 'templates', 'flippable')
#: What a template candidate is: a stage that switches current through an
#: inductor, transformer, flying or bootstrap cap, or a motor; or the parts
#: between a radio chip's RF pins and its antenna.
TEMPLATE_KINDS = ('switching stage', 'rf front end', 'none')
TEMPLATE_KEYS = ('chip', 'kind', 'why', 'skip')
DOC_KEYS = ('schema', 'board', 'groups', 'note')

#: A net is a RAIL -- left out of every connectivity vote here -- when its
#: name says ground or it reaches more
#: than RAIL_SHARE of the board's parts (and more than RAIL_MIN_OWNERS).
RAIL_SHARE = 0.25
RAIL_MIN_OWNERS = 6
#: Parts with this many copper pads are hubs for the cohesion check.
HUB_PADS = 6


class GroupsError(ValueError):
    """A groups file that cannot be read as one (bad JSON, bad keys)."""


def rail_net_ids(owners: Dict[int, Set[str]], name_of, n_parts: int) -> Set[int]:
    """Net ids that are rails (see RAIL_SHARE). `owners` maps a net id to the
    refs with a copper pad on it, `name_of(net_id)` gives its name."""
    from net_queries import is_ground_net_name
    cap = max(RAIL_MIN_OWNERS, RAIL_SHARE * max(1, n_parts))
    out: Set[int] = set()
    for nid, refs in owners.items():
        name = name_of(nid) or ''
        if (name and is_ground_net_name(name)) or len(refs) > cap:
            out.add(nid)
    return out


def load_groups(path: str) -> Dict:
    """The groups file at `path`, its keys checked. Raises GroupsError."""
    try:
        with open(path, encoding='utf-8') as f:
            doc = json.load(f)
    except (OSError, ValueError) as ex:
        raise GroupsError(f"{path}: not a readable JSON file ({ex})")
    return validate_doc(doc, path)


def validate_doc(doc, where: str = 'groups') -> Dict:
    if not isinstance(doc, dict):
        raise GroupsError(f"{where}: the top level must be an object")
    bad = sorted(set(doc) - set(DOC_KEYS))
    if bad:
        raise GroupsError(f"{where}: unknown key(s) {bad} (known: {list(DOC_KEYS)})")
    if doc.get('schema', SCHEMA) != SCHEMA:
        raise GroupsError(f"{where}: schema {doc.get('schema')!r}, this reader knows {SCHEMA}")
    groups = doc.get('groups')
    if not isinstance(groups, list) or not groups:
        raise GroupsError(f"{where}: `groups` must be a non-empty list")
    names, chips = [], set()
    for i, g in enumerate(groups):
        if not isinstance(g, dict):
            raise GroupsError(f"{where}: groups[{i}] is not an object")
        bad = sorted(set(g) - set(GROUP_KEYS))
        if bad:
            raise GroupsError(f"{where}: groups[{i}] unknown key(s) {bad} "
                              f"(known: {list(GROUP_KEYS)})")
        name = g.get('name')
        if not isinstance(name, str) or not name:
            raise GroupsError(f"{where}: groups[{i}] needs a `name`")
        if name in names:
            raise GroupsError(f"{where}: group name {name!r} is used twice")
        names.append(name)
        refs = g.get('refs')
        if not isinstance(refs, list) or not refs or not all(isinstance(r, str) for r in refs):
            raise GroupsError(f"{where}: group {name!r} needs a non-empty `refs` list of strings")
        if isinstance(g.get('edge'), str) and g['edge'] in _EDGE_NAMES:
            g['edge'] = _EDGE_NAMES[g['edge']]
        if g.get('edge') not in (None,) + EDGES:
            raise GroupsError(f"{where}: group {name!r} edge {g.get('edge')!r} is not one of {EDGES}")
        if g.get('side', 'F') not in ('F', 'B'):
            raise GroupsError(f"{where}: group {name!r} side {g.get('side')!r} is not F or B")
        if g.get('anchor') is not None and g['anchor'] not in refs:
            raise GroupsError(f"{where}: group {name!r} anchor {g['anchor']!r} is not one of its refs")
        for key in ('near', 'apart'):
            v = g.get(key) or []
            if not isinstance(v, list):
                raise GroupsError(f"{where}: group {name!r} `{key}` must be a list of group names")
        fl = g.get('flippable')
        if fl is not None:
            if not isinstance(fl, list) or not all(isinstance(r, str) for r in fl):
                raise GroupsError(f"{where}: group {name!r} `flippable` must be a list of refs")
            stray = sorted(set(fl) - set(refs))
            if stray:
                raise GroupsError(f"{where}: group {name!r} `flippable` names {stray}, "
                                  f"which are not among its refs")
        _check_templates(g, name, refs, where, chips)
    for i, g in enumerate(groups):
        for key in ('near', 'apart'):
            for other in g.get(key) or []:
                if other not in names:
                    raise GroupsError(f"{where}: group {g['name']!r} `{key}` names "
                                      f"{other!r}, which is not a group")
    return doc


def _check_templates(g, name, refs, where, chips) -> None:
    """A group's `templates` list: each a chip of the group, a known kind and
    a reason, no chip named twice in the file."""
    tl = g.get('templates')
    if tl is None:
        return
    if not isinstance(tl, list):
        raise GroupsError(f"{where}: group {name!r} `templates` must be a list")
    for k, t in enumerate(tl):
        at = f"{where}: group {name!r} templates[{k}]"
        if not isinstance(t, dict):
            raise GroupsError(f"{at} is not an object")
        bad = sorted(set(t) - set(TEMPLATE_KEYS))
        if bad:
            raise GroupsError(f"{at} unknown key(s) {bad} (known: {list(TEMPLATE_KEYS)})")
        if t.get('chip') not in refs:
            raise GroupsError(f"{at} chip {t.get('chip')!r} is not one of the group's refs")
        if t['chip'] in chips:
            raise GroupsError(f"{at} chip {t['chip']!r} is named a candidate twice")
        chips.add(t['chip'])
        if t.get('kind') not in TEMPLATE_KINDS:
            raise GroupsError(f"{at} kind {t.get('kind')!r} is not one of {TEMPLATE_KINDS}")
        if not isinstance(t.get('why'), str) or not t['why'].strip():
            raise GroupsError(f"{at} needs a `why`: the part, and the datasheet layout to copy")
        if 'skip' in t and (not isinstance(t['skip'], str) or not t['skip'].strip()):
            raise GroupsError(f"{at} `skip` must say why the candidate gets no template")


def template_candidates(doc: Dict, *, all_kinds: bool = False) -> List[Dict]:
    """[{group, chip, kind, why, skip?}]: every template candidate the file
    names -- without `all_kinds`, not the chips it records as `none`."""
    return [dict(t, group=g['name']) for g in doc.get('groups') or ()
            for t in g.get('templates') or () if all_kinds or t.get('kind') != 'none']


#: An RF port's pin function: ANT, RFO, RFI_P, RF_IN, RFIO, LNA_IN, PA_OUT.
_RF_FN = re.compile(r'^(ANT\w*|RFO\w*|RFI\w*|RF_?IN\w*|RF_?OUT\w*|RFIO\w*|LNA_?IN\w*|PA_?OUT\w*)$', re.I)
#: A bootstrap or flying-cap pin: BOOT, BST, VBST, BOOTSTRAP1, CB, C1+, CP, FLY.
_CAP_FN = re.compile(r'(^|[_/])(V?BOOT\w*|V?BST\w*|BS|CB\d?|C\d[+-]|CAP[+-]|CP[+-]?\w*|FLY\w*)$', re.I)
#: A chip pin's net with more parts than this is a rail, not a switch node
#: or an RF match.
_SWITCH_NET_PARTS = 4


def _is_inductor(fp, ref) -> bool:
    """A two-pad inductor (L, value in henries), not a ferrite bead."""
    v = (getattr(fp, 'value', '') or '').replace(' ', '').lower()
    pads = [p for p in fp.pads if p.net_id]
    return (len(pads) == 2 and re.match(r'^L\d', ref) is not None
            and re.search(r'\d(\.\d+)?[pnum\u00b5]?h', v) is not None)


def _is_cap(fp, ref) -> bool:
    return re.match(r'^C\d', ref) is not None and len([p for p in fp.pads if p.net_id]) == 2


def template_hints(doc: Dict, pcb) -> List[str]:
    """What the board's pins and nets suggest and the file does not name:
    a chip that `emc_roles` reads as a switching converter; a chip pin whose
    net carries an inductor and few other parts (a switch node, a built-in
    DC-DC's coil, an RF match); a cap across two of a chip's pins, one of
    them named a bootstrap or flying-cap pin; a pin named as an RF port. A hint
    for the agent to weigh, never a rule: a stage can show none of these, and
    some that show one need nothing -- the file records those as `none`."""
    named = {t['chip'] for t in template_candidates(doc, all_kinds=True)}
    fps = pcb.footprints or {}
    why: Dict[str, List[str]] = {}
    try:
        from placement.emc_roles import emc_roles
        for r in emc_roles(pcb).get('roles') or []:
            kind = r.get('role') if r.get('role') != 'unknown' else r.get('candidate')
            if kind == 'switching converter':
                why.setdefault(r['part'], []).append('emc_roles reads a switching converter')
    except Exception as ex:      # a hint never stops the check
        why.setdefault('?', []).append(f"emc_roles could not read the board ({type(ex).__name__})")
    on: Dict[int, Set[str]] = {}
    for ref, fp in fps.items():
        for p in fp.pads:
            if p.net_id:
                on.setdefault(p.net_id, set()).add(ref)
    gnd = {nid for nid in on if is_ground(_net(pcb, nid))}
    for ref, fp in fps.items():
        pads = [p for p in fp.pads if p.net_id]
        if len(pads) < 5 or not re.match(r'^(U|IC)\d', ref):
            continue
        mine = {p.net_id for p in pads} - gnd
        capped = {p.net_id for p in pads if _CAP_FN.search((p.pinfunction or '').strip())}
        for p in pads:
            fn = (p.pinfunction or '').strip()
            others = on.get(p.net_id, set()) - {ref}
            if p.net_id in gnd or len(others) > _SWITCH_NET_PARTS:
                continue
            ls = sorted(o for o in others if _is_inductor(fps[o], o))
            if ls:
                why.setdefault(ref, []).append(f"pin {p.pad_number}{' ' + fn if fn else ''} "
                                               f"carries inductor {'/'.join(ls)}")
            if fn and _RF_FN.match(fn) and others:
                why.setdefault(ref, []).append(f"pin {p.pad_number} is RF port {fn}")
        for c, cf in fps.items():
            if not _is_cap(cf, c):
                continue
            a, b = [q.net_id for q in cf.pads if q.net_id][:2]
            if a != b and a in mine and b in mine and (a in capped or b in capped):
                why.setdefault(ref, []).append(f"cap {c} across two of its pins")
    out = []
    for ref in sorted(why, key=str):
        if ref in named:
            continue
        seen = list(dict.fromkeys(why[ref]))
        val = getattr(fps.get(ref), 'value', '') or '?'
        out.append(f"{ref} ({val}): {'; '.join(seen[:4])} -- a switching stage or an RF front "
                   f"end? Name it, or record it `none`")
    return out


def _net(pcb, nid) -> str:
    n = (pcb.nets or {}).get(nid)
    return getattr(n, 'name', '') or ''


# ---------------------------------------------------------------------------
# the board's side of the question
# ---------------------------------------------------------------------------

def _cu_pads(fp):
    return [p for p in (fp.pads or ())
            if getattr(p, 'pad_type', '') != 'np_thru_hole'
            and any(str(l).endswith('.Cu') for l in (p.layers or ()))]


def is_ground(name) -> bool:
    """A ground or return net, by name: GND or VSS anywhere in it -- the
    decap rule's ground (`BoardView.decaps`), and place_simple's. Wider than
    `net_queries.is_ground_net_name` on purpose, which leaves out VSS and
    names that only end in GND for the routing tools' sake: here `LV_GND`,
    `VSSA` and `VSS_PA` are grounds too (a decap's other pad, a pour, not a
    channel), and a single-pad `unconnected-(J1-GND-Pad6)` is never priced."""
    u = (name or '').upper()
    return 'GND' in u or 'VSS' in u


#: Roles (`roles.infer_role`) whose supply pins own no decap.
CONNECTOR_ROLES = frozenset({'connector', 'pin header', 'usb receptacle', 'edge receptacle'})


#: A part whose pads span this share of the board's bounds is the board's own
#: contacts (`is_carrier`).
CARRIER_SHARE = 0.5


def is_carrier(pcb, ref) -> bool:
    """Whether a part is the board's own contacts rather than a chip on it:
    its pads, in its own frame, span half the board's bounds or more -- a
    module's castellated or LGA pads (osm_l_bmc_som's OSM-L, core1106_cam),
    a plug-in form factor's pin rows (a Teensy, a DIP-40 adapter), a carrier's
    headers. Every part on such a board lies inside its pad span, so it is
    the nearest "chip" to every cap. Pose-free: the span is read in the
    part's own frame against the outline's bounds."""
    fp = pcb.footprints.get(ref)
    bb = getattr(getattr(pcb, 'board_info', None), 'board_bounds', None)
    pads = _cu_pads(fp) if fp is not None else []
    if not bb or not pads:
        return False
    area = (bb[2] - bb[0]) * (bb[3] - bb[1])
    xs = [p.local_x for p in pads]
    ys = [p.local_y for p in pads]
    return area > 0 and (max(xs) - min(xs)) * (max(ys) - min(ys)) >= CARRIER_SHARE * area


def owns_decaps(pcb, ref) -> bool:
    """Whether a chip's supply pins own decaps: every chip but a connector
    (`CONNECTOR_ROLES`) or the board's own contacts (`is_carrier`). A cap on
    a connector's power pin is a bulk cap where power enters. The decap
    rule's one connector test, read by the placer (`BoardView.supply_pins`)
    and the grade (`floorplan`)."""
    from placement.roles import infer_role
    return (infer_role(pcb.footprints.get(ref), ref, None)['role'] not in CONNECTOR_ROLES
            and not is_carrier(pcb, ref))


#: A supply word in a net or pin name (`reads_as_rail`): a voltage (`+5V`,
#: `3V3`, `V1P8`), a V-name (`VPHY`, `VAA`), or a rail word (`AVDD`,
#: `VCCPLL1`, `REGOUT`, `CPOUT`, `VCAP`).
_RAIL_WORDS = re.compile(
    r'^\+(\d|V|BAT)|^-\d|\d+V\d*|V\d+P\d+|^V[A-Z0-9]|VCC|VDD|VEE|VTT|VBUS|'
    r'VBAT|VSYS|VIN|VOUT|REG|PWR|PLL|CPOUT|VCAP|VIO')
#: A word that makes a rail-like name a SIGNAL: a reference or bias
#: (`VREF`, `VCOM`, `VOCM`), a sense or detect input, a loop's compensation,
#: filter or tuning node (`PLL_VTUNE`), a slew or soft-start pin, a switch
#: node or bootstrap, a reset or enable, a clock or crystal pin (`PLL_XI`).
_NOT_RAIL = re.compile(
    r'REF|VBG|BANDGAP|VCOM|VO?CM\b|SENSE|SNS|DET|COMP|FILT|TUNE|DVDT|BOOT|BST|'
    r'RST|RESET|XTAL|OSC|CLK|SYNC|^EN\b|_EN\b|\bEN$|^FB|_FB\b|^SS\b|_SS\b|'
    r'^SW\b|^LX|^PH\b|^X(I|O|IN|OUT)\b|_X(I|O|IN|OUT)\b')
#: A pin that is a switch node, bootstrap or charge-pump terminal: a cap
#: across two of a chip's pins is a decap only between its supply pins.
_SWITCH_PIN = re.compile(r'^(SW|BST|BOOT|LX|CP|VCP|CB|PH)')
#: Parts a local rail may run through besides its caps (`BoardView._local`).
_RAIL_PATH_ROLES = frozenset({'inductor', 'ferrite bead', 'test point'})


def _name_words(name) -> str:
    return re.sub(r'[~{}]', '', str(name or '').upper())


def reads_as_rail(name) -> Optional[bool]:
    """Whether a net or pin NAME reads as a supply rail: True, False, or None
    for no name. `V_{OUT}` reads as `VOUT`; `VREF`, `VBUS_DET`, `5V_SS`,
    `PLL_VTUNE` read as signals."""
    s = _name_words(name)
    if not s:
        return None
    t = s.replace('_', '')
    return not (_NOT_RAIL.search(s) or _NOT_RAIL.search(t)) and bool(_RAIL_WORDS.search(t))


def decap_view(pcb) -> 'BoardView':
    """`pcb`'s BoardView, built once per parsed board and kept on it. The
    decap rule is electrical, so one answer holds at every pose of the board:
    the grade, the tether gate and the placer ask it per cap and per pin."""
    v = getattr(pcb, '_decap_view', None)
    if v is None or v.pcb is not pcb:
        v = BoardView(pcb)
        try:
            pcb._decap_view = v
        except AttributeError:
            pass
    return v


class BoardView:
    """What grouping needs from a parsed board: who carries copper, who is
    locked, which refs share which nets, and which nets are rails."""

    def __init__(self, pcb):
        self.pcb = pcb
        self.parts: Dict[str, object] = {r: fp for r, fp in (pcb.footprints or {}).items()
                                         if _cu_pads(fp)}
        self.locked: Set[str] = {r for r, fp in self.parts.items()
                                 if getattr(fp, 'locked', False)}
        self.owners: Dict[int, Set[str]] = {}
        self.nets_of: Dict[str, Set[int]] = {}
        for r, fp in self.parts.items():
            for p in _cu_pads(fp):
                if p.net_id > 0:
                    self.owners.setdefault(p.net_id, set()).add(r)
                    self.nets_of.setdefault(r, set()).add(p.net_id)
        nets = pcb.nets or {}
        self.name_of = lambda nid: getattr(nets.get(nid), 'name', '') or ''
        self.rails = rail_net_ids(self.owners, self.name_of, len(self.parts))
        self._supply = None
        self._ladder = None     # the ladder's share of `_supply`
        self._decaps = None
        self._roles: Dict[str, str] = {}
        self._pin_names: Optional[Dict[Tuple[str, str], str]] = None

    def supply_pins(self) -> Dict[int, List[Tuple[str, str]]]:
        """{rail net id: [(chip, pad number)]} -- the chips' supply pins on
        every net but ground, less connectors' (`owns_decaps`): a cap on a
        connector's power pin is a bulk cap where power enters, placed with
        its group, not a decap pulled to that pin. Two sources, joined:

          * `floorplan.supply_pins`, whose ladder takes ONE channel per chip
            (pin type, else pin function, else rail name);
          * the pins on a real rail that the one channel leaves out
            (`rail_pins`): tigard's FT2232H wins on its typed pins and so
            lost VPHY, VPLL and VREG, noahfc's MPU6000 its REGOUT and CPOUT.
        """
        if self._supply is None:
            from placement.floorplan import supply_pins
            self._supply, self._ladder = {}, {}
            seen = set()
            for ic, rec in supply_pins(self.pcb).items():
                if not owns_decaps(self.pcb, ic):
                    continue
                for pad, net in rec.get('pins') or ():
                    nid = getattr(pad, 'net_id', None) or (net if isinstance(net, int) else None)
                    if nid and not is_ground(self.name_of(nid)):
                        self._supply.setdefault(nid, []).append((ic, str(pad.pad_number)))
                        self._ladder.setdefault(nid, []).append((ic, str(pad.pad_number)))
                        seen.add((ic, str(pad.pad_number)))
            for ic, num, nid in self.rail_pins():
                if (ic, num) not in seen:
                    seen.add((ic, num))
                    self._supply.setdefault(nid, []).append((ic, num))
        return self._supply

    def rail_pins(self) -> List[Tuple[str, str, int]]:
        """[(chip, pad number, net id)] -- a chip's pins on a REAL rail, read
        off names and off what else the net reaches. A pin of a chip
        (`groups.chip_refs`, not a connector) that has a pad on ground, on a
        net that is not ground, whose own name does not read as a signal
        (`reads_as_rail` is not False), is one when

          * its net is NAMED as a rail (`/VPHY`, `VREG`, `V1P8`,
            `/DCDC/16V_OUT`), or
          * its own name reads as one (`REGOUT`, `CPOUT`, `5VOUT`, `AVIN`)
            and the net is local to it (`_local`).

        A pin named as a signal stays out on any net (`PD7/NRST` beside its
        RC cap, `VREF`, `VBUS_DET`, `5V_SS`, `PLL_XI`), and so does an
        unnamed pin on a net named `Net-(...)`, which says nothing."""
        from placement.groups import chip_refs
        from placement.roles import pin_function_name
        out = []
        for ic in sorted(chip_refs(self.pcb)):
            fp = self.parts.get(ic)
            if fp is None or not owns_decaps(self.pcb, ic):
                continue
            pads = _cu_pads(fp)
            if not any(p.net_id > 0 and is_ground(self.name_of(p.net_id)) for p in pads):
                continue
            for p in pads:
                name = self.name_of(p.net_id) if p.net_id > 0 else ''
                if not name or is_ground(name) or name.startswith('unconnected-'):
                    continue
                pin = reads_as_rail(pin_function_name(p))
                if pin is False:
                    continue
                leaf = name.split('/')[-1]
                named = not leaf.startswith('Net-(') and reads_as_rail(leaf) is True
                if named or (pin and self._local(p.net_id, ic)):
                    out.append((ic, str(p.pad_number), p.net_id))
        return out

    def _role(self, ref: str) -> str:
        r = self._roles.get(ref)
        if r is None:
            from placement.roles import infer_role
            r = self._roles[ref] = infer_role(self.pcb.footprints.get(ref), ref, None)['role']
        return r

    def _local(self, nid: int, ic: str) -> bool:
        """Whether net `nid` is local to chip `ic`: every other part on it is
        a cap from it to ground, an inductor, ferrite or test point on the
        way in, or a two-pad resistor from a net named as a rail (an RC
        supply filter, glasgow's R59 into U12's Vbus). A reset RC's resistor
        comes from a signal or a pull-up rail the pin is not named for, a
        loop filter's from another cap, so neither reads as a rail here."""
        from placement.groups import is_decoupling_cap
        for r in self.owners.get(nid, ()):
            if r == ic:
                continue
            fp = self.parts[r]
            other = {p.net_id for p in _cu_pads(fp) if p.net_id > 0} - {nid}
            if is_decoupling_cap(fp, r):
                if len(other) == 1 and is_ground(self.name_of(next(iter(other)))):
                    continue
                return False
            role = self._role(r)
            if role in _RAIL_PATH_ROLES:
                continue
            if role == 'resistor' and len(_cu_pads(fp)) == 2 and len(other) == 1:
                o = self.name_of(next(iter(other)))
                leaf = o.split('/')[-1]
                if (not is_ground(o) and not leaf.startswith('Net-(')
                        and reads_as_rail(leaf) is True):
                    continue
            return False
        return True

    def _pin_name(self, ic: str, num: str) -> str:
        if self._pin_names is None:
            from placement.roles import pin_function_name
            self._pin_names = {}
            for r, fp in self.parts.items():
                for p in _cu_pads(fp):
                    self._pin_names.setdefault((r, str(p.pad_number)), pin_function_name(p))
        return self._pin_names.get((ic, num), '')

    def decaps(self) -> Dict[str, int]:
        """{cap: its rail net id} -- every decap by the electrical rule, the
        ONE rule place_simple prices, check_groups reports by and the grade
        measures (`groups.decap_population`). A cap
        (`groups.is_decoupling_cap`) is a decap when it is

          * from ground (`is_ground`) to a rail some other chip has a supply
            pin on (`supply_pins`); its owners are those chips;
          * or between two rails that each carry a supply pin of one chip: a
            STACKED domain, where one die's VSS is the next one's ground
            (bitaxe's BM1366, VDD1 to VDD2 to VDD3), or a chip's own analog
            ground (an ECP5's VCCPLL to GNDPLL). Its owners are the chips
            with a supply pin on both by `floorplan.supply_pins`' ladder --
            not a rail pin (`rail_pins`), which reads a gate driver's VB/VS
            bootstrap pair or a battery monitor's VC cell taps as rails, and
            not a switch-node, bootstrap or charge-pump pin (`_SWITCH_PIN`),
            so a buck's BST-to-SW cap is no decap -- and its rail the first
            of the two by net id.

        A cap from a rail to anything else (a converter's feed-forward cap,
        rail to FB) decouples nothing. Electrical only: it asks nothing of
        where the parts are, so it answers on a pile too."""
        if self._decaps is None:
            from placement.groups import is_decoupling_cap
            supply = self.supply_pins()
            ladder = self._ladder
            self._decaps, self._decap_rails, self._decap_owners = {}, {}, {}
            for ref, fp in sorted(self.parts.items()):
                if not is_decoupling_cap(fp, ref):
                    continue
                nets = sorted({p.net_id for p in fp.pads if p.net_id > 0})
                rails = [n for n in nets if not is_ground(self.name_of(n))]
                if len(nets) != 2 or not rails:
                    continue
                if len(rails) == 1:
                    owners = {ic for ic, _p in supply.get(rails[0], ()) if ic != ref}
                else:
                    a, b = ({ic for ic, num in ladder.get(n, ())
                             if ic != ref and not _SWITCH_PIN.match(self._pin_name(ic, num))}
                            for n in rails)
                    owners = a & b
                if owners:
                    self._decaps[ref] = rails[0]
                    self._decap_rails[ref] = tuple(rails)
                    self._decap_owners[ref] = owners
        return self._decaps

    def decap_rails(self) -> Dict[str, Tuple[int, ...]]:
        """{cap: (rail net id, ...)} -- the rail a decap decouples, or the
        two of a stacked one (`decaps`)."""
        self.decaps()
        return self._decap_rails

    def decap_owners(self) -> Dict[str, Set[str]]:
        """{cap: {owner chip}} -- the chips a decap serves (`decaps`)."""
        self.decaps()
        return self._decap_owners

    def decap_pins(self, cap: str) -> List[Tuple[str, str]]:
        """The supply pins `cap` serves: its owners' pins on its rail."""
        rail = self.decaps().get(cap)
        if rail is None:
            return []
        own = self._decap_owners[cap]
        return [q for q in self.supply_pins().get(rail, ()) if q[0] in own]

    def pad_count(self, ref: str) -> int:
        return len(_cu_pads(self.parts[ref]))


def group_of(doc: Dict) -> Dict[str, str]:
    """{ref: group name} -- the first group naming a ref (duplicates are an
    error `analyze` reports)."""
    out: Dict[str, str] = {}
    for g in doc['groups']:
        for r in g['refs']:
            out.setdefault(r, g['name'])
    return out


def cap_owners(view: BoardView, cap: str):
    """`(net name, [owner refs])` when `cap` is a decap (`BoardView.decaps`):
    its rail (a stacked cap's two, `A|B`) and its owners -- else None."""
    if cap not in view.decaps():
        return None
    return ('|'.join(view.name_of(n) for n in view.decap_rails()[cap]),
            sorted(view.decap_owners()[cap]))


def decap_conflicts(doc: Dict, view: BoardView) -> List[Dict]:
    """Caps the placer would claim at an IC of ANOTHER group: no decap owner
    of the cap's net is in the cap's own group. The groups file is the author
    saying the cap belongs elsewhere (a crystal's load cap, an RC filter), so
    place_simple leaves it out of its decap term."""
    first = group_of(doc)
    out = []
    for cap, g in sorted(first.items()):
        if cap not in view.parts or cap in view.locked:
            continue
        hit = cap_owners(view, cap)
        if hit is None:
            continue
        net, owners = hit
        if any(first.get(o) == g for o in owners):
            continue
        out.append({'cap': cap, 'cap_group': g, 'net': net, 'owners': owners,
                    'owner_groups': sorted({first.get(o, '?') for o in owners})})
    return out


def claim_rows(pcb, intent_doc: Optional[Dict] = None,
               crystals: bool = True) -> Tuple[List[Dict], List[str]]:
    """`(rows, notes)`: the proximity claims place_simple seats -- the
    intent's own `proximity[]` rows (`intent_doc` is the intent JSON) and,
    unless `crystals` is False, each passive crystal's claim at its IC's
    oscillator pins (`crystals.crystal_claims`) that the intent does not
    already make -- and the crystal finder's notes."""
    rows = list((intent_doc or {}).get('proximity') or [])
    if not crystals:
        return rows, []
    from placement.crystals import crystal_claims
    have = {(str(c.get('ref')), str(c.get('near'))) for c in rows}
    xrows, notes = crystal_claims(pcb)
    rows += [r for r in xrows if (r['ref'], r['near']) not in have]
    return rows, notes


def follower_roots(claims: Iterable[Dict]) -> Dict[str, str]:
    """{subject: the part its chain of `proximity[]` claims ends at}: a load
    cap's claim names its crystal and the crystal's names the IC, so both
    end at the IC. A subject with several claims follows its first; a cycle
    stops where it closes."""
    near: Dict[str, str] = {}
    for c in claims:
        near.setdefault(str(c['ref']), str(c['near']))
    roots = {}
    for sub, nxt in near.items():
        seen = {sub}
        while nxt in near and nxt not in seen:
            seen.add(nxt)
            nxt = near[nxt]
        roots[sub] = nxt
    return roots


def fold_followers(doc: Dict, claims: Iterable[Dict]) -> Tuple[Dict, Dict[str, Tuple[str, str]]]:
    """`(doc, moved)`: the groups file with every `proximity[]` subject moved
    into the group of the part its claims end at (`follower_roots`) -- and,
    when a subject is its group's anchor, the rest of that group with it --
    and {ref: (from group, to group)} for each part moved.

    A follower is placed beside its partner, so it must share its
    partner's group: a crystal in a clock group of its own would be held
    apart from the IC whose pins it belongs at. A group the move empties is
    dropped, and the `near` and `apart` entries naming it are pointed at the
    group that took its parts. The input is not modified."""
    roots = follower_roots(claims)
    where = group_of(doc)
    moved = {s: (where[s], where[r]) for s, r in roots.items()
             if s in where and r in where and where[s] != where[r]}
    # A group whose ANCHOR moved follows it whole: a clock group is its
    # crystal, and the jumpers or 0-ohm bridges on the oscillator pins belong
    # at the crystal. A group whose anchor is not a claim subject keeps its
    # other parts.
    for g in doc['groups']:
        a = g.get('anchor')
        if a in moved and moved[a][0] == g['name']:
            for r in g['refs']:
                if r not in moved and where.get(r) == g['name']:
                    moved[r] = (g['name'], moved[a][1])
    if not moved:
        return doc, {}
    out = json.loads(json.dumps(doc))
    for g in out['groups']:
        g['refs'] = [r for r in g['refs'] if r not in moved]
        g['refs'] += sorted(s for s, (_f, t) in moved.items() if t == g['name'])
        if g.get('anchor') is not None and g['anchor'] not in g['refs']:
            del g['anchor']
    gone = {g['name'] for g in out['groups'] if not g['refs']}
    took: Dict[str, str] = {}
    for _s, (f, t) in sorted(moved.items()):
        if f in gone:
            took.setdefault(f, t)
    out['groups'] = [g for g in out['groups'] if g['refs']]
    for g in out['groups']:
        for key in ('near', 'apart'):
            if g.get(key):
                names = [took.get(n, n) for n in g[key]]
                names = [n for i, n in enumerate(names)
                         if n != g['name'] and n not in gone and n not in names[:i]]
                if names:
                    g[key] = names
                else:
                    del g[key]
    return out, moved


def tree_weight(n: int) -> float:
    """The weight of each pair of a net's `n` nodes when the net is priced
    as its spanning tree: the tree has n - 1 edges over n (n - 1) / 2 pairs,
    so each pair carries 2 / n of an edge, and the net sums to n - 1."""
    return 2.0 / n if n > 1 else 0.0


def link_matrix(doc: Dict, view: BoardView) -> Dict[Tuple[str, str], float]:
    """{(group a, group b): signal links between them}, a < b. A signal net
    reaching m groups is a spanning tree over them, m - 1 links, spread
    evenly over its group pairs (`tree_weight`), however many parts of each
    group it touches: a bus on six parts in two groups is one link, as its
    wiring needs one."""
    gof = group_of(doc)
    out: Dict[Tuple[str, str], float] = {}
    for nid in sorted(view.owners):
        if nid in view.rails:
            continue
        gs = sorted({gof[r] for r in view.owners[nid]
                     if r in gof and r in view.parts})
        w = tree_weight(len(gs))
        for i, ga in enumerate(gs):
            for gb in gs[i + 1:]:
                out[(ga, gb)] = out.get((ga, gb), 0.0) + w
    return out


def analyze(doc: Dict, pcb, movable: Optional[Iterable[str]] = None,
            followers: Iterable[str] = (), extra_fixed: Iterable[str] = ()) -> Dict:
    """Everything the checker reports, as data. `movable` defaults to every
    copper-bearing part the file does not lock and `extra_fixed` does not
    name (the parts the intent seats at a declared pose). `followers` are
    the subjects of proximity claims, seated by their claim rather than by
    the decap rule."""
    view = BoardView(pcb)
    gof: Dict[str, List[str]] = {}
    for g in doc['groups']:
        for r in g['refs']:
            gof.setdefault(r, []).append(g['name'])
    movable = (set(movable) if movable is not None
               else set(view.parts) - view.locked - set(extra_fixed))
    # a part that is not movable is FIXED for every question below: locked in
    # the file, or held out by the caller (`movable`)
    fixed = set(view.parts) - movable
    unknown = sorted(r for r in gof if r not in (pcb.footprints or {}))
    padless = sorted(r for r in gof if r in (pcb.footprints or {}) and r not in view.parts)
    duplicates = {r: names for r, names in sorted(gof.items()) if len(names) > 1}
    missing = sorted(r for r in movable if r not in gof)
    fixed_members = sorted(r for r in gof if r in fixed)
    first = group_of(doc)

    # cohesion, per part: signal links to its own group vs the strongest
    # other MOVABLE group. A link to a group of fixed parts (the headers a core
    # talks to) is not a grouping mistake -- it is what the placer puts the
    # group next to -- so it never makes a part "pulled".
    fixed_groups = {g['name'] for g in doc['groups']
                    if all(r in fixed for r in g['refs'] if r in view.parts)}
    anchors = {g.get('anchor') for g in doc['groups'] if g.get('anchor')}
    pulled = []
    bridges = []
    for ref, own in sorted(first.items()):
        if ref not in view.parts or ref in fixed:
            continue
        # a hub -- the group's anchor, or any part of HUB_PADS+ pads -- talks
        # to every group by design; "pulled" is a question about satellites
        hub = ref in anchors or view.pad_count(ref) >= HUB_PADS
        # one link per (signal net, group it reaches): joining a net needs
        # one connection to it, however many of a group's parts it touches
        by_group: Dict[str, int] = {}
        for nid in view.nets_of.get(ref, ()):
            if nid in view.rails:
                continue
            for g in {first[o] for o in view.owners.get(nid, ())
                      if o != ref and o in first}:
                by_group[g] = by_group.get(g, 0) + 1
        own_n = by_group.get(own, 0)
        others = {g: n for g, n in by_group.items()
                  if g != own and g not in fixed_groups}
        if others and not hub:
            best = max(others, key=lambda g: (others[g], g))
            if others[best] > own_n:
                pulled.append({'ref': ref, 'group': own, 'own_links': own_n,
                               'pulled_by': best, 'links': others[best]})
        # a BRIDGE sits in series between groups: a 2-pad part whose two pads
        # are both on signal nets, and those nets reach another group
        sig = [n for n in view.nets_of.get(ref, ()) if n not in view.rails]
        if view.pad_count(ref) == 2 and len(sig) == 2:
            borders = sorted({first[o] for n in sig for o in view.owners.get(n, ())
                              if o != ref and o in first and first[o] != own})
            if borders:
                bridges.append({'ref': ref, 'group': own, 'borders': borders})

    # A cap that follows a proximity claim (a crystal's load cap) is seated
    # by its claim, not by the decap rule, so it is no conflict.
    decap_cross = [d for d in decap_conflicts(doc, view)
                   if d['cap'] not in set(followers)]

    matrix = link_matrix(doc, view)
    errors = []
    if unknown:
        errors.append(f"refs not on this board: {unknown}")
    if duplicates:
        errors.append("refs in more than one group: "
                      + ', '.join(f"{r} ({'/'.join(n)})" for r, n in duplicates.items()))
    if missing:
        errors.append(f"movable parts in no group: {missing}")
    for g in doc['groups']:
        if not any(r in view.parts for r in g['refs']):
            errors.append(f"group {g['name']!r} has no part on this board")
    warnings = []
    if padless:
        warnings.append(f"refs with no copper pads (ignored): {padless}")
    for p in pulled:
        warnings.append(f"{p['ref']} ({p['group']}) has {p['links']} signal link(s) to "
                        f"{p['pulled_by']} and {p['own_links']} to its own group")
    for d in decap_cross:
        warnings.append(f"{d['cap']} ({d['cap_group']}) is a decap of "
                        f"{'/'.join(d['owners'])} ({'/'.join(d['owner_groups'])}) to the "
                        f"decap rule, and no IC in its own group carries its "
                        f"net {d['net']}: place_simple exempts it from the decap rule "
                        f"so it is placed with {d['cap_group']} -- right for a load or "
                        f"RC cap, wrong for a real decap (move it to its IC's group)")
    return {
        'groups': [{'name': g['name'], 'role': g.get('role'),
                    'refs': g['refs'],
                    'movable': [r for r in g['refs'] if r in movable],
                    'fixed': [r for r in g['refs'] if r in fixed]}
                   for g in doc['groups']],
        'links': [{'a': a, 'b': b, 'signal_links': n}
                  for (a, b), n in sorted(matrix.items())],
        'rails': sorted(view.name_of(n) for n in view.rails),
        'pulled': pulled, 'bridges': bridges, 'decap_cross_group': decap_cross,
        'fixed_members': fixed_members, 'errors': errors, 'warnings': warnings,
    }
