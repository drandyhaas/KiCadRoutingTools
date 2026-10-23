"""Crystal claims: each passive crystal at its IC's oscillator pins, and each
of its load caps at the crystal, written as `proximity[]` rows.

A crystal's loop is the one placement fact its netlist cannot show: a 3 mm
loop and a 30 mm one have the same connectivity. `emc_roles` already finds
each passive crystal and the IC its two load nets reach, by pin functions and
nets, never by guessing a pin number. This module writes the claims those
roles imply:

* the crystal: each of its two signal pads within `leg_mm` of the IC pad on
  the same net;
* a leg that reaches the IC through a series part (the resistor many
  reference designs put between XOUT and the crystal: the RP2040's, an
  STM32's): the crystal pad within `leg_mm` of the series part's pad on its
  net, and the series part's other pad within `leg_mm` of the IC pin on ITS
  net -- the two links of that leg;
* each load cap, meaning a two-pad cap with one pad on a load net (either
  leg's) and the other on ground: its load-net pad within `cap_mm` of the
  crystal pad on that net. A cap whose other pad is not ground is not a load
  cap, so it gets no claim.

The rows are ordinary `proximity[]` claims, so `place_simple`'s claim term
pulls each subject to its partner: the crystal to its IC, and each cap to the
crystal. `check_floorplan --emit-intent` writes them into the intent's
`proximity[]` (`floorplan.emit_intent`), where the grade reads them at WARN
(`floorplan.derived_claim`: their limits are the corpus's, not the board's)
and an edit of the intent can loosen or tighten one. place_simple and
`check_groups` also derive them through `grouping.claim_rows`, beside the
intent's own rows, for an intent that does not carry them; a pair the intent
already claims keeps the intent's row.

The limits are derived defaults, not declarations. They are read off the
human-placed stress corpus by `tests/stress/placement_survey.py`
(`leg_gap_mm`, `cap_gap_mm`: the farthest leg and the farthest load cap of
each crystal, pad edge to pad edge, as the claim grades them). Each row says
so in its `context`.
"""
from __future__ import annotations

import contextlib
import io
import re
from typing import Dict, List, Optional, Tuple

#: The farthest crystal leg, pad edge to the IC pad on its net, at the human
#: corpus's 75th percentile, so three human placements in four meet it
#: (`placement_survey.py report`, CRYSTALS: 258 crystals on 437 boards,
#: median 5.37 mm, p75 8.11 mm). It guards against a crystal drifting away;
#: the anneal's claim term pulls the crystal as near its pins as it can.
LEG_MM = 8.1
#: The farthest load cap, pad edge to the crystal pad on its net, likewise
#: (174 crystals with a load cap: median 0.81 mm, p75 2.22 mm).
CAP_MM = 2.2

_CAP_REF = re.compile(r'^C\d')


def _ground(name: str) -> bool:
    from net_queries import is_ground_net_name
    return bool(name) and is_ground_net_name(name)


def _series_part(pcb, x: str, net: int, ic: str, net_name) -> Optional[Tuple]:
    """`(part, its pad on `net`, its other pad, the pins of `ic` on that
    pad's net)` for the two-pad part a crystal leg on `net` reaches `ic`
    through (a series resistor between XOUT and the crystal), else None. A
    part whose other pad is on ground is a load cap, not a series part."""
    for r in sorted(pcb.footprints):
        if r in (x, ic):
            continue
        cu = [p for p in pcb.footprints[r].pads if p.net_id]
        if len(cu) != 2:
            continue
        on = [p for p in cu if p.net_id == net]
        off = [p for p in cu if p.net_id != net]
        if len(on) != 1 or len(off) != 1 or _ground(net_name.get(off[0].net_id, '')):
            continue
        pins = [q for q in pcb.footprints[ic].pads if q.net_id == off[0].net_id]
        if pins:
            return r, on[0], off[0], pins
    return None


def crystal_claims(pcb, leg_mm: float = LEG_MM, cap_mm: float = CAP_MM
                   ) -> Tuple[List[Dict], List[str]]:
    """`(rows, notes)`: one `proximity[]` row per passive crystal, per link of
    a leg through a series part, and per load cap, and a line per crystal
    saying what was found. Reads no pose."""
    from .emc_roles import emc_roles
    with contextlib.redirect_stdout(io.StringIO()):
        roles = emc_roles(pcb).get('roles') or []
    net_name = {nid: (n.name or '') for nid, n in pcb.nets.items()}
    rows: List[Dict] = []
    notes: List[str] = []
    leg_ctx = {'source': 'placement.crystals', 'role': 'crystal',
               'limit': 'derived_default', 'basis': (
                   'the farthest crystal leg on the human corpus, '
                   '75th percentile (placement_survey.py)')}
    for rec in roles:
        if rec.get('role') != 'crystal' or not rec.get('proximity'):
            continue
        x, ic = rec['part'], rec['ic']
        pads = rec['proximity'][0]['row']['pads']
        rows.append({
            'ref': x, 'near': ic, 'max_mm': leg_mm, 'basis': 'pad_edge',
            'pads': {x: list(pads[x]), ic: list(pads[ic])},
            'context': dict(leg_ctx)})
        xfp = pcb.footprints[x]
        load = {p.net_id: str(p.pad_number) for p in xfp.pads
                if str(p.pad_number) in set(pads[x]) and p.net_id}
        # a leg no pin of the IC is on: through a series part, its two links
        series = []
        for u in rec.get('unmeasured_legs') or ():
            xp = next((p for p in xfp.pads if str(p.pad_number) == str(u['pad'])
                       and p.net_id), None)
            got = _series_part(pcb, x, xp.net_id, ic, net_name) if xp else None
            if not got:
                continue
            r, rin, rout, pins = got
            load[xp.net_id] = str(xp.pad_number)
            ipads = sorted({str(q.pad_number) for q in pins})
            series.append(f"pad {xp.pad_number} through {r} to {ic} pad {', '.join(ipads)}")
            rows.append({
                'ref': x, 'near': r, 'max_mm': leg_mm, 'basis': 'pad_edge',
                'pads': {x: [str(xp.pad_number)], r: [str(rin.pad_number)]},
                'context': {**leg_ctx, 'role': 'crystal leg to its series part'}})
            rows.append({
                'ref': r, 'near': ic, 'max_mm': leg_mm, 'basis': 'pad_edge',
                'pads': {r: [str(rout.pad_number)], ic: ipads},
                'context': {**leg_ctx, 'role': 'crystal series part'}})
        caps = []
        for c in sorted(pcb.footprints):
            if c == x or not _CAP_REF.match(c):
                continue
            cu = [p for p in pcb.footprints[c].pads if p.net_id]
            if len(cu) != 2:
                continue
            on = [p for p in cu if p.net_id in load]
            off = [p for p in cu if p.net_id not in load]
            if len(on) != 1 or len(off) != 1 or not _ground(
                    net_name.get(off[0].net_id, '')):
                continue
            caps.append(c)
            rows.append({
                'ref': c, 'near': x, 'max_mm': cap_mm, 'basis': 'pad_edge',
                'pads': {c: [str(on[0].pad_number)],
                         x: [load[on[0].net_id]]},
                'context': {'source': 'placement.crystals',
                            'role': 'crystal load cap',
                            'limit': 'derived_default', 'basis': (
                                'the farthest load cap on the human corpus, '
                                '75th percentile (placement_survey.py)')}})
        notes.append(f"crystal {x} at {ic} pads {', '.join(pads[ic])}"
                     + ''.join(f"; {s}" for s in series)
                     + (f"; load caps {', '.join(caps)}" if caps
                        else "; no load cap (a cap from a load net to ground)"))
    return rows, notes
