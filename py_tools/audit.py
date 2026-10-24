#!/usr/bin/env python3
"""Judge a routed board against a spec and report every check that passed or failed, with reasons (#1237).

    python3 py_tools/audit.py board.kicad_pcb [--spec spec.json]
                              [--baseline INPUT] [--json report.json]

It never infers intent: the spec says what the board should do, and the audit
measures the copper against it. Limits come from the spec, from the board's
own files, or from fab_tiers (JLCPCB); every finding names which in
`limit_from`.

Checks that need no spec always run: check_weird's advisory categories
(#1233-#1235) and check_via_mask's two (#1232). The rest run per spec entry:

  pairs           {"USB": {"nets": ["/USB_DP", "/USB_DM"],
                   "impedance_ohm": 90, "tolerance_pct": 10,
                   "max_skew_mm": 0.15, "max_uncoupled_mm": 3.0,
                   "gap_max_mm": 0.2}}
      pair.impedance   each coupled stretch's differential impedance, from
                       impedance.calculate_impedance_for_layer (IPC-2141
                       edge-coupled forms: approximate), within the tolerance
      pair.skew        net_queries.diff_pair_skew (KiCad's skew measure)
      pair.uncoupled   net_queries.diff_pair_coupling's uncoupled length
                       (KiCad's)
      A stretch is COUPLED when its gap is within the pair's gap window: the
      spec's `gap_max_mm`, else the board's diff_pair_gap rule maximum, else
      its netclass diff pair gap (design_rules.DesignRules). With none of
      those, pair.impedance takes the layer's dielectric height and
      pair.uncoupled counts every parallel facing span, as KiCad does. Each
      record names the window and its source.
      pair.reference   check_impedance's reference-plane crossings under the
                       pair; `reference_nets` limits which pours count
  reference_nets  ["GND"]: the nets whose pours may carry a pair's return
  length_groups   {"BYTE0": {"nets": [...], "tolerance_mm": 0.1,
                   "mode": "total" | "pin_pair"}}, board_score's format
                   (tolerance_mm defaults to 0.1, mode to total)
      length.match     net_queries.net_match_length spread; a net not on the
                       board fails, as board_score counts it
  net_min_widths  {"<net glob>": mm}, board_score's format; first glob wins
      net.min_width    every segment of the net, at the STRICTER of the spec
                       and the board's own min_track_width
  currents        {"VBUS": {"amps": 1.5, "source": "J1.1",
                   "loads": ["U1.3"] | {"U1.3": 0.5, ...},
                   "max_rise_c": 10, "max_drop_mv": 100,
                   "via_plating_mm": 0.02}}
      A list of loads is judged one load at a time, each drawing all `amps`;
      a dict gives each load's own current, solved together. No `loads`
      means every other numbered copper pad of the net, as a list. Every pad
      of a named terminal conducts (a pad number can repeat). The copper is
      dc_network's resistor network of the net's tracks and vias (pours left
      out, so resistances read high); one finding per track or via, at its
      worst current.
      current.capacity each track piece's current against the IPC-2152-form
                       rating (plane_resistance.calculate_max_current_ipc2152)
                       at its width, its layer's copper and `max_rise_c`
      current.via      each via's current against the same rating for its
                       barrel (width pi * drill, thickness the plating)
      current.drop     each load's DC drop from the source (with max_drop_mv)
  decoupling      {"C3": {"ic": "U1", "pins": ["4"], "max_distance_mm": 2.0,
                   "max_vias": 1}}
      decoupling.path  the shortest track path from the cap's pad on the
                       pin's net to the pin (via barrels counted in the
                       length), and the distinct vias on it

A check reports "passed", "failed" (with findings) or "not_judged" (with the
reason it could not be judged, never a pass). Every failure is reported.

Exit codes, as check_floorplan and board_score: 0 every judged check passed,
1 crash, 2 bad arguments, a malformed spec (wrong key, type or range) or an
unreadable baseline, 3 the board is missing or not a board, 4 a check failed.
board_score --spec counts only the spec checks, never the spec-free ones.
"""

#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['routing'], 'kind': 'instrument'}

import _path  # noqa: F401  (#522: makes ../py_router importable)

import argparse
import fnmatch
import glob as _glob
import heapq
import json
import math
import os
import sys
from typing import Dict, List, Optional

DEFAULT_RISE_C = 10.0
#: The checks that run with no spec. Reported, never blocking: board_score's
#: audit component counts only the others.
SPEC_FREE_IDS = ('weird.off_angle_corner', 'weird.short_fragment',
                 'weird.off_centre_via_end', 'via.mask_contact', 'via.mask_web')
#: board_score --length-groups' default, so the two read one format alike.
DEFAULT_LENGTH_TOLERANCE_MM = 0.1

_SPEC_KEYS = {
    'pairs': ({'nets'}, {'impedance_ohm', 'tolerance_pct', 'max_skew_mm',
                         'max_uncoupled_mm', 'gap_max_mm'}),
    'length_groups': ({'nets'}, {'tolerance_mm', 'mode'}),
    'currents': ({'amps', 'source'}, {'loads', 'max_rise_c', 'max_drop_mv',
                                      'via_plating_mm'}),
    'decoupling': ({'ic', 'pins', 'max_distance_mm'}, {'max_vias'}),
}

#: Every value's kind: 'pos' a number > 0, 'nonneg' a number >= 0, 'int' a
#: whole number >= 0, 'str' a non-empty string, 'strs' a non-empty list of
#: them, ('enum', ...) one of the names.
_FIELD_KINDS = {
    'pairs': {'nets': 'strs', 'impedance_ohm': 'pos', 'tolerance_pct': 'pos',
              'max_skew_mm': 'nonneg', 'max_uncoupled_mm': 'nonneg', 'gap_max_mm': 'pos'},
    'length_groups': {'nets': 'strs', 'tolerance_mm': 'nonneg',
                      'mode': ('enum', 'total', 'pin_pair')},
    'currents': {'amps': 'pos', 'source': 'str', 'max_rise_c': 'pos',
                 'max_drop_mv': 'nonneg', 'via_plating_mm': 'pos'},
    'decoupling': {'ic': 'str', 'pins': 'strs', 'max_distance_mm': 'nonneg', 'max_vias': 'int'},
}


class SpecError(Exception):
    pass


def _number(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def _valid(kind, v) -> bool:
    if kind == 'pos':
        return _number(v) and v > 0
    if kind == 'nonneg':
        return _number(v) and v >= 0
    if kind == 'int':
        return isinstance(v, int) and not isinstance(v, bool) and v >= 0
    if kind == 'str':
        return isinstance(v, str) and v != ''
    if kind == 'strs':
        return isinstance(v, list) and v and all(isinstance(x, str) and x for x in v)
    return v in kind[1:]


_KIND_TEXT = {'pos': 'a number > 0', 'nonneg': 'a number >= 0',
              'int': 'a whole number >= 0', 'str': 'a non-empty string',
              'strs': 'a non-empty list of strings'}


def load_spec(path: Optional[str]) -> dict:
    """The spec at `path`, validated; {} without one. Keys starting with '_'
    are comments. Anything else unknown, missing or of the wrong type or range
    is an error (exit 2), so a typo cannot silently leave a check unjudged."""
    if not path:
        return {}
    try:
        with open(path, encoding='utf-8') as fh:
            spec = json.load(fh)
    except (OSError, ValueError) as exc:
        raise SpecError(f'cannot read the spec {path}: {exc}')
    if not isinstance(spec, dict):
        raise SpecError('the spec is not a JSON object')
    spec = {k: v for k, v in spec.items() if not k.startswith('_')}
    known = set(_SPEC_KEYS) | {'reference_nets', 'net_min_widths'}
    bad = sorted(set(spec) - known)
    if bad:
        raise SpecError(f'unknown spec key(s) {bad}; known: {sorted(known)}')
    for section, (required, optional) in _SPEC_KEYS.items():
        entries = spec.get(section, {})
        if not isinstance(entries, dict):
            raise SpecError(f'{section} is not an object')
        for name, entry in entries.items():
            if not isinstance(entry, dict):
                raise SpecError(f'{section}.{name} is not an object')
            keys = {k for k in entry if not k.startswith('_')}
            if required - keys:
                raise SpecError(f'{section}.{name} lacks {sorted(required - keys)}')
            if keys - required - optional:
                raise SpecError(f'{section}.{name} has unknown key(s) '
                                f'{sorted(keys - required - optional)}')
            for key in sorted(keys - {'loads'}):
                kind = _FIELD_KINDS[section][key]
                if not _valid(kind, entry[key]):
                    want = _KIND_TEXT.get(kind) or 'one of %s' % list(kind[1:])
                    raise SpecError(f'{section}.{name}.{key} must be {want}, '
                                    f'not {entry[key]!r}')
            if section == 'currents' and 'loads' in entry:
                loads = entry['loads']
                ok = (_valid('strs', loads) if isinstance(loads, list) else
                      isinstance(loads, dict) and loads and all(
                          isinstance(k, str) and k and _valid('pos', v)
                          for k, v in loads.items()))
                if not ok:
                    raise SpecError(f'currents.{name}.loads must be a non-empty list '
                                    f'of "REF.PAD" or a map of them to amps > 0')
    for name, entry in spec.get('pairs', {}).items():
        if len(entry['nets']) != 2:
            raise SpecError(f'pairs.{name}.nets must name exactly two nets')
        if 'impedance_ohm' in entry and 'tolerance_pct' not in entry:
            raise SpecError(f'pairs.{name} gives impedance_ohm without tolerance_pct')
    if 'reference_nets' in spec and not _valid('strs', spec['reference_nets']):
        raise SpecError('reference_nets must be a non-empty list of net names')
    widths = spec.get('net_min_widths', {})
    if not isinstance(widths, dict) or not all(
            isinstance(k, str) and k and _valid('pos', v) for k, v in widths.items()):
        raise SpecError('net_min_widths must map net globs to widths in mm > 0')
    return spec


# ---------------------------------------------------------------------------
# Report shapes
# ---------------------------------------------------------------------------

def _check(cid, subject=None, findings=None, reason=None, **extra):
    """One check record: failed with findings, not_judged with a reason, else
    passed."""
    if findings:
        status = 'failed'
    elif reason:
        status = 'not_judged'
    else:
        status = 'passed'
    out = {'id': cid, 'status': status}
    if subject is not None:
        out['subject'] = subject
    if findings:
        out['findings'] = findings
    if reason and not findings:
        out['reason'] = reason
    out.update({k: v for k, v in extra.items() if v not in (None, [], {})})
    return out


def _finding(reason, measured=None, limit=None, units=None, where=None,
             items=None, limit_from=None, **extra):
    out = {'reason': reason}
    for k, v in (('measured', measured), ('limit', limit), ('units', units),
                 ('where', where), ('items', items), ('limit_from', limit_from)):
        if v is not None:
            out[k] = v
    out.update(extra)
    return out


def _where(layer, x, y):
    return {'layer': layer, 'x_mm': round(x, 4), 'y_mm': round(y, 4)}


def _r(v, nd=4):
    return None if v is None else round(v, nd)


# ---------------------------------------------------------------------------
# Board lookups
# ---------------------------------------------------------------------------

def net_id_of(pcb, name):
    for nid, n in pcb.nets.items():
        if n.name == name:
            return nid
    return None


def pads_of(pcb, ref_pad: str):
    """The pads 'REF.PAD' names (a pad number can repeat), [] when none."""
    ref, _, num = ref_pad.rpartition('.')
    fp = pcb.footprints.get(ref)
    if fp is None or not num:
        return []
    return [p for p in fp.pads if str(p.pad_number) == num]


def board_min_track_width(pcb_path):
    """(mm, label) the board itself declares as its minimum track width: the
    larger of the project rule and its recorded original (the chain can lower
    the rule), or (None, None)."""
    try:
        from fix_kicad_drc_settings import project_rules, declared_fab_floor
        rules = project_rules(pcb_path) or {}
        vals = [(rules.get('min_track_width'), 'board: rules.min_track_width'),
                (declared_fab_floor(pcb_path, 'min_track_width'),
                 'board: fab_floor_origin.min_track_width')]
        vals = [(float(v), lab) for v, lab in vals if isinstance(v, (int, float)) and v > 0]
        return max(vals) if vals else (None, None)
    except Exception:                                          # noqa: BLE001
        return None, None


# ---------------------------------------------------------------------------
# Spec-free checks (#1232-#1235)
# ---------------------------------------------------------------------------

def spec_free_checks(pcb, baseline_pcb=None):
    from check_weird import (advisory_findings, PERMITTED_TURNS_DEG,
                             TURN_TOLERANCE_DEG, FRAGMENT_FLOOR_MM)
    from check_via_mask import via_mask_findings, split_inherited
    from fab_tiers import JLC_PLUGGED_VIA_MASK_WEB_MM
    out = []
    weird = advisory_findings(pcb)
    limits = {
        'off-angle-corner': ('turn not within %g deg of %s' % (
            TURN_TOLERANCE_DEG, '/'.join('%g' % t for t in PERMITTED_TURNS_DEG)),
            'check_weird.PERMITTED_TURNS_DEG'),
        'short-fragment': ('%g mm' % FRAGMENT_FLOOR_MM, 'check_weird.FRAGMENT_FLOOR_MM'),
        'off-centre-via-end': ('end on the via centre', 'check_weird'),
    }
    for cat, (limit, src) in limits.items():
        found = [_finding(f['detail'], limit=limit, where=_where(f['layer'], f['x'], f['y']),
                          items=[f['net']], limit_from=src)
                 for f in weird if f['category'] == cat]
        out.append(_check('weird.' + cat.replace('-', '_'), findings=found))
    vm = via_mask_findings(pcb)
    inherited = []
    if baseline_pcb is not None:
        vm, inherited = split_inherited(vm, via_mask_findings(baseline_pcb))
    for cat, limit, src in (('via-mask-contact', 'ring outside every pad mask opening',
                             'check_via_mask'),
                            ('via-mask-web', JLC_PLUGGED_VIA_MASK_WEB_MM,
                             'fab: fab_tiers.JLC_PLUGGED_VIA_MASK_WEB_MM')):
        found = []
        for f in vm:
            if f['category'] != cat:
                continue
            extra = {'approximation': f['approximation']} if f.get('approximation') else {}
            found.append(_finding(
                'via ring in the %s opening of pad %s (%s)' % (f['side'], f['pad'], f['pad_net'])
                if cat == 'via-mask-contact' else
                'plugged via %.3f mm from the %s opening of pad %s' % (
                    f['distance_mm'], f['side'], f['pad']),
                measured=None if cat == 'via-mask-contact' else f['distance_mm'],
                limit=limit, units=None if cat == 'via-mask-contact' else 'mm',
                where=_where(f['side'], f['x'], f['y']), items=[f['net'], f['pad']],
                limit_from=src, **extra))
        out.append(_check('via.' + cat.split('-', 1)[1].replace('-', '_'), findings=found,
                          accepted_from_baseline=sum(1 for f in inherited
                                                     if f['category'] == cat) or None))
    return out


# ---------------------------------------------------------------------------
# Pairs
# ---------------------------------------------------------------------------

def coupled_stretches(pcb, p_id, n_id, gap_max=None):
    """[(layer, width_mm, gap_mm, length_mm, x, y)] for every P/N track pair
    that runs parallel and faces each other, as diff_pair_coupling finds them
    (without its obstruction test)."""
    from kicad_seg import iu, intersects, common_parallel_projection, nearest_points, norm, sq_len
    tracks = lambda nid: [s for s in pcb.segments if s.net_id == nid and not s.graphic]
    pts = lambda s: ((iu(s.start_x), iu(s.start_y)), (iu(s.end_x), iu(s.end_y)))
    out = []
    n_by_layer = {}
    for s in tracks(n_id):
        a, b = pts(s)
        if sq_len(a, b) > 2:
            n_by_layer.setdefault(s.layer, []).append((s, a, b))
    for sp in tracks(p_id):
        pa, pb = pts(sp)
        if sq_len(pa, pb) <= 2:
            continue
        for sn, na, nb in n_by_layer.get(sp.layer, ()):
            if intersects(pa, pb, na, nb, lines=True):
                continue
            clip = common_parallel_projection(pa, pb, na, nb)
            if clip is None:
                continue
            ca, cb, da, db = clip
            near_p, _near_n, d2 = nearest_points(ca, cb, da, db)
            gap = (math.sqrt(d2) - (iu(sp.width) + iu(sn.width)) / 2) / 1e6
            if gap <= 0 or (gap_max is not None and gap > gap_max + 5e-4):
                continue
            length = norm(db[0] - da[0], db[1] - da[1]) / 1e6
            if length <= 0:
                continue
            out.append((sp.layer, (sp.width + sn.width) / 2, gap, length,
                        near_p[0] / 1e6, near_p[1] / 1e6))
    return out


def pair_gap_window(pcb, board_path, p_id, entry, name):
    """(gap_min, gap_max, source) bounding a coupled stretch; gap_max None
    when neither the spec nor the board gives one."""
    if 'gap_max_mm' in entry:
        return None, float(entry['gap_max_mm']), f'spec: pairs.{name}.gap_max_mm'
    try:
        from design_rules import DesignRules
        dr = DesignRules.from_project(pcb, board_path)
        c = dr.resolve('diff_pair_gap', dr.item_for_net(p_id, 'track'))
    except Exception:                                          # noqa: BLE001
        return None, None, None
    gmin = c.kicad_min if c.kicad_min is not None else c.min
    if c.max is not None and c.max >= 0:
        return gmin, c.max, f'board: diff_pair_gap rule {c.source}'
    if c.opt is not None and c.opt > 0:
        return gmin, c.opt, 'board: netclass diff_pair_gap'
    return gmin, None, None


def pair_checks(pcb, spec, board_path):
    from net_queries import diff_pair_skew, diff_pair_coupling, net_pads_joined
    out = []
    ref_names = spec.get('reference_nets') or []
    ref_ids = None
    ref_missing = [n for n in ref_names if net_id_of(pcb, n) is None]
    if ref_names:
        ref_ids = {net_id_of(pcb, n) for n in ref_names} - {None}
    for name, entry in spec.get('pairs', {}).items():
        p_name, n_name = entry['nets']
        p_id, n_id = net_id_of(pcb, p_name), net_id_of(pcb, n_name)
        missing = [n for n, i in ((p_name, p_id), (n_name, n_id)) if i is None]
        wanted = ['pair.reference'] + [cid for cid, key in (
            ('pair.impedance', 'impedance_ohm'), ('pair.skew', 'max_skew_mm'),
            ('pair.uncoupled', 'max_uncoupled_mm')) if key in entry]
        if missing:
            out += [_check(cid, name, reason=f'net(s) {missing} are not on the board')
                    for cid in wanted]
            continue
        items = [p_name, n_name]
        gmin, gmax, gsrc = pair_gap_window(pcb, board_path, p_id, entry, name)
        window = {'gap_max_mm': gmax, 'source': gsrc or 'none'}
        if 'impedance_ohm' in entry:
            target, tol = float(entry['impedance_ohm']), float(entry['tolerance_pct'])
            lo, hi = target * (1 - tol / 100.0), target * (1 + tol / 100.0)
            src = f'spec: pairs.{name}.impedance_ohm, tolerance_pct'
            if not pcb.board_info.stackup:
                out.append(_check('pair.impedance', name, reason=(
                    'the board has no stackup: dielectric heights and Er are unknown')))
            else:
                from impedance import calculate_impedance_for_layer, get_layer_impedance_params
                stretches = coupled_stretches(pcb, p_id, n_id, gmax)
                if gmax is None:
                    window = {'gap_max_mm': 'the layer dielectric height',
                              'source': 'no gap in the spec or the board'}
                    stretches = [st for st in stretches
                                 if (get_layer_impedance_params(pcb, st[0]) is not None
                                     and st[2] <= get_layer_impedance_params(
                                         pcb, st[0]).dielectric_height + 5e-4)]
                found = []
                for layer, width, gap, length, x, y in stretches:
                    z = calculate_impedance_for_layer(pcb, layer, width, spacing=gap).get('zdiff')
                    if z is None or not (lo <= z <= hi):
                        found.append(_finding(
                            f'differential impedance {z:.1f} ohm over {length:.3f} mm '
                            f'(width {width:.3f} mm, gap {gap:.3f} mm)' if z is not None
                            else f'no impedance model for {layer}',
                            measured=_r(z, 2), limit=[_r(lo, 2), _r(hi, 2)], units='ohm',
                            where=_where(layer, x, y), items=items, limit_from=src))
                out.append(_check('pair.impedance', name, findings=found,
                                  reason=None if stretches else
                                  'the pair has no stretch inside its gap window',
                                  model='impedance.calculate_impedance_for_layer '
                                        '(IPC-2141 edge-coupled, approximate)',
                                  gap_window=window))
        if 'max_skew_mm' in entry:
            sk = diff_pair_skew(pcb, p_id, n_id)
            limit = float(entry['max_skew_mm'])
            if sk['skew_mm'] is None:
                out.append(_check('pair.skew', name, reason=(
                    'a member\'s pads are not joined by track copper')))
            else:
                out.append(_check('pair.skew', name, findings=[] if sk['skew_mm'] <= limit + 1e-9
                                  else [_finding(f"skew {sk['skew_mm']:.4f} mm",
                                                 measured=_r(sk['skew_mm']), limit=limit,
                                                 units='mm', items=items,
                                                 limit_from=f'spec: pairs.{name}.max_skew_mm')],
                                  measured=_r(sk['skew_mm'])))
        if 'max_uncoupled_mm' in entry and not (net_pads_joined(pcb, p_id)
                                                 and net_pads_joined(pcb, n_id)):
            out.append(_check('pair.uncoupled', name, reason=(
                "a member's pads are not joined by track copper")))
        elif 'max_uncoupled_mm' in entry:
            cp = diff_pair_coupling(pcb, p_id, n_id, gap_min=gmin, gap_max=gmax)
            limit = float(entry['max_uncoupled_mm'])
            u = cp['uncoupled_mm']
            out.append(_check('pair.uncoupled', name,
                              findings=[] if u <= limit + 1e-9 else [_finding(
                                  f'{u:.3f} mm of the pair runs uncoupled', measured=_r(u),
                                  limit=limit, units='mm', items=items,
                                  limit_from=f'spec: pairs.{name}.max_uncoupled_mm')],
                              measured=_r(u), gap_window={'gap_max_mm': gmax,
                                                          'source': gsrc or 'none (KiCad: '
                                                          'every parallel facing span)'}))
        if ref_missing:
            out.append(_check('pair.reference', name, reason=(
                f'reference_nets {ref_missing} are not on the board')))
            continue
        from check_impedance import analyze_impedance
        rep = analyze_impedance(pcb, [_glob.escape(p_name), _glob.escape(n_name)],
                                reference_net_ids=ref_ids, include_plane_nets=True)
        found = [_finding(c.describe(), measured=_r(c.length_mm), units='mm',
                          where=_where(c.layer, c.x, c.y), items=[c.net_name],
                          limit_from=('spec: reference_nets' if ref_ids is not None
                                      else 'any filled pour on the reference layer'),
                          kind=c.kind, ref_layer=c.ref_layer)
                 for net in rep.nets for c in net.crossings]
        out.append(_check('pair.reference', name, findings=found,
                          reason=None if rep.nets else 'no routed copper on the pair'))
    return out


# ---------------------------------------------------------------------------
# Length groups and per-net widths (board_score's formats)
# ---------------------------------------------------------------------------

def length_checks(pcb, spec):
    from net_queries import net_match_length
    out = []
    for name, entry in spec.get('length_groups', {}).items():
        mode = entry.get('mode', 'total')
        lengths, missing, unrouted = {}, [], []
        for n in entry['nets']:
            nid = net_id_of(pcb, n)
            if nid is None:
                missing.append(n)
                continue
            v = net_match_length(pcb, nid, mode)
            (unrouted.append(n) if v is None else lengths.__setitem__(n, v))
        if missing:
            # a finding about the spec, as board_score --length-groups counts it
            out.append(_check('length.match', name, findings=[_finding(
                f'net(s) {missing} named by the group are not on the board',
                items=missing, limit_from=f'spec: length_groups.{name}.nets')]))
            continue
        if unrouted:
            out.append(_check('length.match', name, reason=(
                f'not joined by track copper: {unrouted}')))
            continue
        spread = max(lengths.values()) - min(lengths.values())
        tol = float(entry.get('tolerance_mm', DEFAULT_LENGTH_TOLERANCE_MM))
        out.append(_check('length.match', name, findings=[] if spread <= tol + 1e-9 else [
            _finding(f'length spread {spread:.4f} mm ({mode})', measured=_r(spread),
                     limit=tol, units='mm', items=sorted(lengths),
                     limit_from=(f'spec: length_groups.{name}.tolerance_mm'
                                 if 'tolerance_mm' in entry else
                                 'audit.DEFAULT_LENGTH_TOLERANCE_MM'),
                     lengths={k: _r(v) for k, v in sorted(lengths.items())})],
            measured=_r(spread)))
    return out


def width_checks(pcb, spec, board_path):
    want = spec.get('net_min_widths', {})
    if not want:
        return []
    floor, floor_from = board_min_track_width(board_path)
    names = {n.net_id: n.name for n in pcb.nets.values()}
    by_net = {}
    for s in pcb.segments:
        if not s.graphic and names.get(s.net_id):
            by_net.setdefault(names[s.net_id], []).append(s)
    out, used = [], set()
    for name, segs in sorted(by_net.items()):
        pat = next((p for p in want if fnmatch.fnmatchcase(name, p)), None)
        if pat is None:
            continue
        used.add(pat)
        limit, src = float(want[pat]), f'spec: net_min_widths[{pat!r}]'
        if floor is not None and floor > limit:
            limit, src = floor, floor_from
        found = [_finding(f'track {s.width:.4f} mm wide', measured=_r(s.width), limit=limit,
                          units='mm', where=_where(s.layer, s.start_x, s.start_y),
                          items=[name], limit_from=src)
                 for s in segs if s.width < limit - 1e-9]
        out.append(_check('net.min_width', name, findings=found))
    for pat in want:
        if pat not in used:
            out.append(_check('net.min_width', pat, reason=(
                'the glob matches no net with track copper')))
    return out


# ---------------------------------------------------------------------------
# Currents: capacity, vias and voltage drop over dc_network
# ---------------------------------------------------------------------------

def _default_loads(pcb, nid, source):
    """Every other copper pad of the net with a number: what `loads` means when
    the spec leaves it out."""
    from paste_apertures import pad_has_copper
    return sorted({f'{p.component_ref}.{p.pad_number}' for p in pcb.pads_by_net.get(nid, [])
                   if str(p.pad_number) and pad_has_copper(p)} - {source})


def current_checks(pcb, spec, board_thickness_mm=None):
    import dc_network
    from plane_resistance import calculate_max_current_ipc2152, stackup_copper_oz, OZ_TO_METERS
    out = []
    for name, entry in spec.get('currents', {}).items():
        amps = float(entry['amps'])
        rise = float(entry.get('max_rise_c', DEFAULT_RISE_C))
        plating = float(entry.get('via_plating_mm', dc_network.VIA_PLATING_MM))
        wanted = ['current.capacity', 'current.via'] + (
            ['current.drop'] if 'max_drop_mv' in entry else [])
        nid = net_id_of(pcb, name)
        source = entry['source']
        src_pads = pads_of(pcb, source)
        problem, load_pads = None, {}
        if nid is None:
            problem = f'net {name} is not on the board'
        elif not src_pads:
            problem = f"source pad {source} is not on the board"
        elif any(p.net_id != nid for p in src_pads):
            problem = f"source pad {source} is not on {name}"
        loads = entry.get('loads')
        if problem is None:
            if loads is None:
                loads = _default_loads(pcb, nid, source)
            load_pads = {ld: pads_of(pcb, ld) for ld in loads}
            bad = sorted(ld for ld, ps in load_pads.items()
                         if not ps or any(p.net_id != nid for p in ps))
            if bad:
                problem = f'load pad(s) {bad} are not pads of {name}'
            elif source in load_pads:
                problem = f'the source {source} is also a load'
            elif not load_pads:
                problem = f'{name} has no load pad'
        net = None
        if problem is None:
            terminals = dict(load_pads)
            terminals[source] = src_pads
            net = dc_network.build(pcb, nid, terminals, plating, board_thickness_mm)
            if net is None:
                problem = f'{name} has no track copper'
        if problem:
            out += [_check(cid, name, reason=problem) for cid in wanted]
            continue
        ground = net.terminal_node.get(source)
        if isinstance(loads, dict):
            scenarios = [{ld: float(a) for ld, a in loads.items()}]
        else:
            scenarios = [{ld: amps} for ld in loads]
        worst = {}                      # (kind, id(obj)) -> (amps, obj)
        drops, unreachable = {}, set()
        for sc in scenarios:
            inj = {}
            for ld, a in sc.items():
                node = net.terminal_node.get(ld)
                if node is None or ground is None:
                    unreachable.add(ld)
                    continue
                inj[node] = inj.get(node, 0.0) - a
            if ground is None or not inj:
                continue
            potential, currents, _lost = dc_network.solve(net, inj, ground)
            for ld in sc:
                node = net.terminal_node.get(ld)
                if node is None or potential.get(node) is None:
                    unreachable.add(ld)
                else:
                    drops[ld] = max(drops.get(ld, 0.0), abs(potential[node]))
            for e, c in zip(net.edges, currents):
                if c is None:
                    continue
                key = (e.kind, id(e.obj))
                if abs(c) > worst.get(key, (0.0, None))[0]:
                    worst[key] = (abs(c), e.obj)
        cap, via = [], []
        for (kind, _oid), (i_a, o) in worst.items():
            if kind == 'seg':
                rating = calculate_max_current_ipc2152(o.width, stackup_copper_oz(pcb, o.layer), rise)
                if i_a > rating + 1e-9:
                    cap.append(_finding(
                        f'{i_a:.3f} A through a {o.width:.3f} mm track on {o.layer} '
                        f'rated {rating:.3f} A at {rise:g} C rise',
                        measured=_r(i_a), limit=_r(rating), units='A',
                        where=_where(o.layer, (o.start_x + o.end_x) / 2, (o.start_y + o.end_y) / 2),
                        items=[name],
                        limit_from=f'spec: currents.{name}.amps, max_rise_c; '
                                   f'IPC-2152 form (plane_resistance)'))
            else:
                rating = calculate_max_current_ipc2152(
                    math.pi * (o.drill or 0.0), plating / (OZ_TO_METERS * 1e3), rise)
                if i_a > rating + 1e-9:
                    via.append(_finding(
                        f'{i_a:.3f} A through a via of drill {o.drill:.3f} mm rated '
                        f'{rating:.3f} A at {rise:g} C rise',
                        measured=_r(i_a), limit=_r(rating), units='A',
                        where=_where('via', o.x, o.y), items=[name],
                        limit_from=f'spec: currents.{name}.amps, max_rise_c, via_plating_mm; '
                                   f'IPC-2152 form (plane_resistance)'))
        note = sorted(unreachable)
        reason = f'load(s) {note} not joined to the source by track copper' if note else None
        common = {'assumptions': net.assumptions}
        out.append(_check('current.capacity', name, findings=cap, reason=reason, **common))
        out.append(_check('current.via', name, findings=via, reason=reason, **common))
        if 'max_drop_mv' in entry:
            limit = float(entry['max_drop_mv'])
            found = [_finding(f'{v * 1e3:.1f} mV from {source} to {ld}',
                              measured=_r(v * 1e3, 2), limit=limit, units='mV',
                              items=[name, ld], limit_from=f'spec: currents.{name}.max_drop_mv')
                     for ld, v in sorted(drops.items()) if v * 1e3 > limit + 1e-9]
            out.append(_check('current.drop', name, findings=found, reason=reason,
                              measured={ld: _r(v * 1e3, 2) for ld, v in sorted(drops.items())},
                              **common))
    return out


# ---------------------------------------------------------------------------
# Decoupling
# ---------------------------------------------------------------------------

def _shortest_path(pcb, net_id, pads_a, pads_b):
    """(length mm, via count) of the shortest track path from any of `pads_a`
    to any of `pads_b` (one net), or None when no track path joins them. A via
    counts once however many layer steps of its barrel the path walks."""
    from net_queries import _net_track_graph
    edges = []
    graph = _net_track_graph(pcb, net_id, list(pads_a) + list(pads_b), edges=edges)
    if graph is None:
        return None
    _adj, pad_nodes = graph
    adj = {}
    for n1, n2, w, kind, obj, _span in edges:
        tag = id(obj) if kind == 'via' else None
        adj.setdefault(n1, []).append((n2, w, kind, tag))
        adj.setdefault(n2, []).append((n1, w, kind, tag))
    starts = [h for p in pads_a for h in (pad_nodes.get(id(p)) or [])]
    goals = {h for p in pads_b for h in (pad_nodes.get(id(p)) or [])}
    # state = (node, the via whose barrel the path is in, or None)
    best = {(n, None): (0.0, 0) for n in starts}
    heap = [(0.0, 0, i, n, None) for i, n in enumerate(starts)]
    count = len(heap)
    while heap:
        d, v, _, n, cur = heapq.heappop(heap)
        if (d, v) > best.get((n, cur), (math.inf, 0)):
            continue
        if n in goals:
            return d, v
        for m, w, kind, tag in adj.get(n, ()):
            if kind == 'via':
                nxt, dv = tag, (0 if tag == cur else 1)
            elif kind == 'seg':
                nxt, dv = None, 0
            else:
                nxt, dv = cur, 0        # a join inside a pad or a via keeps the state
            cand = (d + w, v + dv)
            if cand < best.get((m, nxt), (math.inf, 0)):
                best[(m, nxt)] = cand
                count += 1
                heapq.heappush(heap, (cand[0], cand[1], count, m, nxt))
    return None


def decoupling_checks(pcb, spec):
    out = []
    for cap, entry in spec.get('decoupling', {}).items():
        fp = pcb.footprints.get(cap)
        ic = entry['ic']
        limit = float(entry['max_distance_mm'])
        max_vias = entry.get('max_vias')
        found, problems, measured = [], [], {}
        if fp is None:
            out.append(_check('decoupling.path', cap, reason=f'{cap} is not on the board'))
            continue
        for pin in entry['pins']:
            ic_pads = pads_of(pcb, f'{ic}.{pin}')
            if not ic_pads:
                problems.append(f'{ic}.{pin} is not on the board')
                continue
            net_id = ic_pads[0].net_id
            cap_pads = [p for p in fp.pads if p.net_id == net_id and net_id]
            if not cap_pads:
                problems.append(f'{cap} has no pad on the net of {ic}.{pin} '
                                f'({ic_pads[0].net_name})')
                continue
            got = _shortest_path(pcb, net_id, cap_pads, ic_pads)
            if got is None:
                problems.append(f'{cap} and {ic}.{pin} are not joined by track copper')
                continue
            length, vias = got
            measured[f'{ic}.{pin}'] = {'path_mm': _r(length), 'vias': vias}
            items = [f"{cap}.{'/'.join(sorted({str(p.pad_number) for p in cap_pads}))}",
                     f'{ic}.{pin}', ic_pads[0].net_name]
            if length > limit + 1e-9:
                found.append(_finding(f'{length:.3f} mm of track from {items[0]} to {items[1]}',
                                      measured=_r(length), limit=limit, units='mm', items=items,
                                      limit_from=f'spec: decoupling.{cap}.max_distance_mm'))
            if max_vias is not None and vias > int(max_vias):
                found.append(_finding(f'{vias} via(s) on the path from {items[0]} to {items[1]}',
                                      measured=vias, limit=int(max_vias), units='vias',
                                      items=items, limit_from=f'spec: decoupling.{cap}.max_vias'))
        out.append(_check('decoupling.path', cap, findings=found,
                          reason='; '.join(problems) if problems else None,
                          measured=measured,
                          unjudged_pins=problems if (found and problems) else None))
    return out


# ---------------------------------------------------------------------------

class BoardError(Exception):
    pass


def board_thickness_mm(board_path):
    """The board file's `(general (thickness X))`, else impedance's nominal."""
    import re
    from impedance import NOMINAL_BOARD_THICKNESS_MM
    try:
        with open(board_path, encoding='utf-8', errors='replace') as fh:
            head = fh.read(20000)
        m = re.search(r'\(general\b[^()]*(?:\([^()]*\)[^()]*)*?\(thickness\s+([\d.]+)', head)
        if m and float(m.group(1)) > 0:
            return float(m.group(1))
    except (OSError, ValueError):
        pass
    return NOMINAL_BOARD_THICKNESS_MM


def _parse(path, what):
    from kicad_parser import parse_kicad_pcb
    pcb = parse_kicad_pcb(path)
    if not getattr(pcb.board_info, 'copper_layers', None):
        raise BoardError(f'{what} {path} declares no copper layers: it is not a board')
    return pcb


def audit(board_path, spec, baseline_path=None):
    pcb = _parse(board_path, 'board')
    base = _parse(baseline_path, 'baseline') if baseline_path else None
    checks = spec_free_checks(pcb, base)
    checks += pair_checks(pcb, spec, board_path)
    checks += length_checks(pcb, spec)
    checks += width_checks(pcb, spec, board_path)
    checks += current_checks(pcb, spec, board_thickness_mm(board_path))
    checks += decoupling_checks(pcb, spec)
    summary = {s: sum(1 for c in checks if c['status'] == s)
               for s in ('passed', 'failed', 'not_judged')}
    return {'board': os.path.abspath(board_path), 'summary': summary, 'checks': checks}


def main(argv=None):
    ap = argparse.ArgumentParser(
        description='Judge a routed board against a spec; report every check '
                    'that passed or failed, with reasons. Read-only.',
        epilog='Exit 0 = every judged check passed; 4 = a check failed; '
               '3 = board state; 2 = bad arguments or spec; 1 = crash.')
    ap.add_argument('board', help='the .kicad_pcb to audit')
    ap.add_argument('--spec', metavar='JSON', help='what the board should do; '
                    'without it only the spec-free checks run')
    ap.add_argument('--baseline', metavar='BOARD',
                    help='the unrouted input: via-mask findings it already had '
                         'are accepted, not reported')
    ap.add_argument('--json', metavar='PATH', help='write the report here')
    args = ap.parse_args(argv)
    for path, what in ((args.board, 'board'), (args.baseline, 'baseline')):
        if path and not os.path.isfile(path):
            print(f'{what} not found: {path}', file=sys.stderr)
            return 3 if what == 'board' else 2
    try:
        spec = load_spec(args.spec)
    except SpecError as exc:
        print(f'spec error: {exc}', file=sys.stderr)
        return 2
    try:
        report = audit(args.board, spec, args.baseline)
    except BoardError as exc:
        print(str(exc), file=sys.stderr)
        return 3 if str(exc).startswith('board') else 2
    for c in report['checks']:
        subject = f" {c['subject']}" if 'subject' in c else ''
        detail = (f" ({len(c['findings'])} finding(s))" if c['status'] == 'failed'
                  else f" -- {c['reason']}" if c['status'] == 'not_judged' else '')
        print(f"  {c['status'].upper():10} {c['id']}{subject}{detail}")
        for f in (c.get('findings') or [])[:5]:
            print(f"      {f['reason']}")
        if len(c.get('findings') or []) > 5:
            print(f"      ... and {len(c['findings']) - 5} more (see --json)")
    s = report['summary']
    print(f"AUDIT: {s['passed']} passed, {s['failed']} failed, {s['not_judged']} not judged")
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as fh:
            json.dump(report, fh, indent=1)
    return 4 if s['failed'] else 0


if __name__ == '__main__':
    sys.exit(main())
