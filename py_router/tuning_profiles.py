"""#1846: the project's tuning profiles -- KiCad 10's time-domain delays.

KiCad 10 stores, per tuning profile, a propagation delay for each signal layer
and one for a via barrel, and a net class names the profile its nets are timed
with (`.kicad_pro`: `tuning_profiles.tuning_profiles_impedance_geometric[]`
and `net_settings.classes[].tuning_profile`). Its DRC grades a `.kicad_dru`
length or skew bound written in ps/ns against the delay those give.

Measured on kicad-cli 10.0.8 (tests/test_1846_time_domain_rules.py):
  * a layer entry's `delay` and the profile's `via_prop_delay` are attoseconds
    per mm (6000000 is 6.00 ps/mm);
  * a net's delay is each track's length times its layer's delay, plus each
    via's barrel length (as the length DRC charges it) times the via delay,
    plus its pads' pad-to-die delays;
  * a track on a layer the profile has no entry for adds nothing;
  * a net whose class names no profile, or a profile with
    `enable_time_domain_tuning` off, has a delay of 0 ps.
"""
import json
import os
from typing import Dict, List, NamedTuple, Optional, Tuple

#: profile delays are stored in attoseconds per mm
_AS_PER_PS = 1e6
_DEFAULT_PRIORITY = 2147483647


class TuningProfile(NamedTuple):
    name: str
    time_domain: bool                 # enable_time_domain_tuning
    layer_ps_per_mm: Dict[str, float]  # signal layer -> ps/mm
    via_ps_per_mm: float
    via_overrides: int                # entries of `via_overrides` (not modelled)


def read_tuning_profiles(board_path: str
                         ) -> Tuple[Dict[str, TuningProfile], List[Tuple[int, str, str]]]:
    """(profiles by name, [(priority, class name, profile name)]) from the
    board's sibling `.kicad_pro`; ({}, []) when there is no project or it
    declares none."""
    pro = os.path.splitext(board_path)[0] + '.kicad_pro' if board_path else ''
    try:
        with open(pro, encoding='utf-8') as fh:
            proj = json.load(fh)
    except (OSError, ValueError):
        return {}, []
    profiles: Dict[str, TuningProfile] = {}
    block = proj.get('tuning_profiles') or {}
    for p in (block.get('tuning_profiles_impedance_geometric') or []
              if isinstance(block, dict) else []):
        if not isinstance(p, dict) or not p.get('profile_name'):
            continue
        layers = {}
        for e in p.get('layer_entries') or []:
            if isinstance(e, dict) and e.get('signal_layer') and isinstance(
                    e.get('delay'), (int, float)):
                layers[str(e['signal_layer'])] = float(e['delay']) / _AS_PER_PS
        via = p.get('via_prop_delay')
        profiles[str(p['profile_name'])] = TuningProfile(
            str(p['profile_name']), bool(p.get('enable_time_domain_tuning')), layers,
            float(via) / _AS_PER_PS if isinstance(via, (int, float)) else 0.0,
            len(p.get('via_overrides') or []))
    classes = []
    for c in ((proj.get('net_settings') or {}).get('classes') or []):
        if isinstance(c, dict) and c.get('tuning_profile'):
            name = str(c.get('name', 'Default'))
            prio = c.get('priority')
            classes.append((int(prio) if isinstance(prio, int) and name != 'Default'
                            else _DEFAULT_PRIORITY, name, str(c['tuning_profile'])))
    return profiles, sorted(classes)


class DelayModel:
    """The delay KiCad's time-domain DRC gives each net of one board."""

    def __init__(self, board_path: str, pcb_data, memberships=None):
        self.pcb_data = pcb_data
        self.profiles, self._classes = read_tuning_profiles(board_path)
        if memberships is None:
            from list_nets import net_class_memberships
            names = {nid: n.name for nid, n in pcb_data.nets.items() if nid and n.name}
            memberships = net_class_memberships(board_path, names)
        self._member = memberships or {}

    def profile_of(self, net_id: int) -> Optional[TuningProfile]:
        """The profile of the net's class: the highest-priority class of the
        net that names one (Default last), or None."""
        mine = set(self._member.get(net_id) or ()) | {'Default'}
        for _prio, cls, prof in self._classes:
            if cls in mine:
                return self.profiles.get(prof)
        return None

    def net_delay(self, net_id: int) -> Tuple[Optional[float], str]:
        """(delay in ps, note) of one net as the time-domain DRC measures it.

        (None, why) when it cannot be measured: its pads are not joined by
        track copper (as `net_queries.net_match_length`), or its profile
        carries via overrides, which are not modelled. (0.0, why) when KiCad
        itself measures 0 ps: no profile, or time-domain tuning off."""
        from net_queries import (kicad_net_length_parts_nm, net_pads_joined,
                                 net_match_pad_to_die, _NM_PER_MM)
        pcb = self.pcb_data
        if not net_pads_joined(pcb, net_id):
            return None, 'pads not joined by track copper'
        prof = self.profile_of(net_id)
        if prof is None:
            return 0.0, "its net class names no tuning profile"
        if not prof.time_domain:
            return 0.0, f"tuning profile '{prof.name}' has time-domain tuning off"
        if prof.via_overrides:
            return None, f"tuning profile '{prof.name}' has via overrides (not modelled)"
        segs = [s for s in pcb.segments if s.net_id == net_id]
        vias = [v for v in pcb.vias if v.net_id == net_id]
        pads = (getattr(pcb, 'pads_by_net', None) or {}).get(net_id) or []
        by_layer, via_nm = kicad_net_length_parts_nm(pcb, segs, vias, pads)
        ps = sum(nm / _NM_PER_MM * prof.layer_ps_per_mm.get(layer, 0.0)
                 for layer, nm in by_layer.items())
        ps += via_nm / _NM_PER_MM * prof.via_ps_per_mm
        ps += net_match_pad_to_die(pcb, net_id, 'total', attr='die_delay')
        return ps, ''


_MODEL_ATTR = '_tuning_delay_model'


def net_time_profile(pcb_data, net_id) -> Optional[TuningProfile]:
    """The tuning profile KiCad times ``net_id`` with, when the router can
    time the net the same way: its class names a profile, the profile has
    time-domain tuning on, and it carries no via overrides. None otherwise
    (the caller then times the net from the stackup, as before).

    The board's `DelayModel` is built once and kept on ``pcb_data``; a board
    with no file behind it, or no project, has no profiles."""
    model = getattr(pcb_data, _MODEL_ATTR, None)
    if model is None:
        path = getattr(pcb_data, 'source_path', '') or ''
        try:
            model = DelayModel(path, pcb_data) if path else False
        except Exception:                                      # noqa: BLE001
            model = False
        try:
            setattr(pcb_data, _MODEL_ATTR, model)
        except Exception:                                      # noqa: BLE001
            pass
    if not model or not model.profiles or net_id is None:
        return None
    prof = model.profile_of(net_id)
    if prof is None or not prof.time_domain or prof.via_overrides:
        return None
    return prof


def route_delay_ps(pcb_data, prof: TuningProfile, segments, vias) -> float:
    """The delay ``prof`` gives one net's copper: each layer's track length
    times its delay, plus the via barrel length times the via delay, measured
    as the length DRC measures them (`kicad_net_length_parts_nm`, so the
    router's number is the checker's). Pad-to-die delays are the caller's."""
    from net_queries import kicad_net_length_parts_nm, _NM_PER_MM
    by_layer, via_nm = kicad_net_length_parts_nm(pcb_data, list(segments or ()),
                                                 list(vias or ()), [])
    return (sum(nm / _NM_PER_MM * prof.layer_ps_per_mm.get(layer, 0.0)
                for layer, nm in by_layer.items())
            + via_nm / _NM_PER_MM * prof.via_ps_per_mm)
