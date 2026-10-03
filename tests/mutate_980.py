"""The #980 mutation battery: pricing foreign copper at check_drc's pairwise
clearance, at every restore and admission check.

One row per load-bearing line, each reverting it; every row names the test
that must fail. **THE ROWS TO LOOK AT FIRST if this file ever goes red**
restore a defect somebody measured:

  * `restore-reverts-to-flat` -- #980 itself: a restore 0.25mm from a 0.35
    class was admitted at the flat 0.2;
  * `box-back-to-1mm` -- a 2mm power track's collision past the old fixed
    1mm prefilter box was never tested;
  * `board-copper-fallback` -- the phase verifier: a through-hole pad on a
    board routed on a subset of its layers priced at a relax rule (0.15)
    where check_drc grades its class (0.35).

NOT named `test_*.py`, so `tests/run_all.py` does not collect it: it REWRITES
the sources in place. One writer per tree. It refuses to start on a dirty
target, and it runs every witness UNMUTATED first -- a witness that already
fails would score every row as killed.

    python3 tests/mutate_980.py
    python3 tests/mutate_980.py --row restore-reverts-to-flat

A row is KILLED by a failure or an error. An anchor that does not match
EXACTLY ONCE is BROKEN, never skipped; `preflight()` runs right after `ROWS`.
Edits are `str.replace(old, new, 1)`; anchors are LF and translated to the
target's own ending.

Not covered by a row, and why:
  * what the oracle's ESCALATION weld does with the map it is handed: it
    runs only after the main weld ladder fails, which the KiCad-free oracle
    harness does not force. That it is HANDED the map is row
    `escalation-map-dropped` (statically);
  * route_diff's `mark_input_copper` call: route_diff has no sub-run that
    re-parses its output, and its call is the same shape as batch_route's
    (rows `route-mark-*`);
  * the oracle's SECOND `_rekey` call (after the round's re-parse): nothing
    reads `config` after it, so dropping it is an equivalent mutant today;
  * `pair-args-swapped` SURVIVES by design: the pair value is symmetric in
    its two nets, so swapping them is an equivalent mutant (a change
    detector for the day it is not).
"""
from __future__ import annotations

import argparse
import io
import os
import subprocess
import sys

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)
_PR = os.path.join(_ROOT, 'py_router')
_GUI = os.path.join(_ROOT, 'kicad_routing_plugin')

TARGETS = {
    'rc': os.path.join(_PR, 'routing_config.py'),
    'kd': os.path.join(_PR, 'kicad_dru.py'),
    'rr': os.path.join(_PR, 'rip_up_reroute.py'),
    'route': os.path.join(_PR, 'route.py'),
    'pbd': os.path.join(_PR, 'plane_blocker_detection.py'),
    'ko': os.path.join(_PR, 'kicad_oracle.py'),
    'lso': os.path.join(_PR, 'layer_swap_optimization.py'),
    'dpm': os.path.join(_PR, 'diff_pair_multipoint.py'),
    'lm': os.path.join(_PR, 'length_matching.py'),
    'sls': os.path.join(_PR, 'stub_layer_switching.py'),
    'nr': os.path.join(_PR, 'net_rescue.py'),
    'rp': os.path.join(_PR, 'repair_planes.py'),
    'gu': os.path.join(_GUI, 'gui_utils.py'),
    'se': os.path.join(_PR, 'single_ended_routing.py'),
    'dpr': os.path.join(_PR, 'diff_pair_routing.py'),
    'sg': os.path.join(_GUI, 'swig_gui.py'),
}


def _t(name, *cases):
    return (os.path.join(_TESTS, name),) + cases


PAR = _t('test_980_pair_clearance_parity.py')
RES = _t('test_980_restore_pairwise.py')
ORC = _t('test_980_oracle_class_map.py')
OBH = _t('test_980_oracle_behaviour.py')
GATE = _t('test_980_no_flat_clearance_gate.py')
T_ADM = 'test_980_admission_pairwise.py'

# (name, target, old, new, tests, expect)
ROWS = [
    # ---- the helper (routing_config) ------------------------------------
    ('class-term-dropped', 'rc',
     "            if a is not None and a > clr:",
     "            if False:",
     (PAR,), 'KILLED'),
    ('stack-rule-dropped', 'rc',
     "            return self.stack_clearance(clr)",
     "            return clr",
     (PAR,), 'KILLED'),
    ('layer-replacement-dropped', 'rc',
     "            clr = self.layer_clearance(layer, clr)",
     "            pass",
     (PAR,), 'KILLED'),
    ('track-raise-dropped', 'rc',
     "        if kind == 'track' and self.track_clearances:",
     "        if False:",
     (PAR,), 'KILLED'),
    ('nameless-copper-widest-dropped', 'rc',
     "            if not net_a or not net_b:",
     "            if False:",
     (PAR,), 'KILLED'),
    ('pad-override-dropped', 'rc',
     "        return self.pad_override_clearance(eff, pad, other_pad) if override \\",
     "        return eff if override \\",
     (PAR,), 'KILLED'),
    ('pad-shared-layers-dropped', 'rc',
     "            eff = pads_shared_layer_clearance(",
     "            eff = (lambda *a: a[0])(",
     (PAR,), 'KILLED'),
    ('board-copper-fallback', 'rc',
     "            cu = list(board_copper or self.board_copper_layers",
     "            cu = list(board_copper or self.layers or self.board_copper_layers",
     (PAR,), 'KILLED'),
    ('board-copper-not-recorded', 'kd',
     "    config.board_copper_layers = list(copper)",
     "    pass",
     (PAR,), 'KILLED'),
    # ---- the restore family ----------------------------------------------
    ('restore-reverts-to-flat', 'rr',
     "    _pc = getattr(config, 'pair_clearance', None)",
     "    _pc = None",
     (RES,), 'KILLED'),
    ('restore-prices-by-first-own-net', 'rr',
     "                else _clr(s.net_id, o.net_id, s.layer, 'track'))",
     "                else _clr(next(iter(own)), o.net_id, s.layer, 'track'))",
     (RES,), 'KILLED'),
    ('box-back-to-1mm', 'rr',
     "    margin = max(1.0, own_r + reach)",
     "    margin = 1.0",
     (RES,), 'KILLED'),
    ('force-restore-loses-config', 'route',
     "            skip_net_ids=_fr_new_copper, config=config)",
     "            skip_net_ids=_fr_new_copper)",
     (GATE,), 'KILLED'),
    ('tap-piece-restore-flat', 'pbd',
     "    if (_pc is None or piece_net is None or plane_net is None",
     "    if (True or _pc is None or piece_net is None or plane_net is None",
     (RES,), 'KILLED'),
    # ---- the admission sweep ----------------------------------------------
    ('sliver-weld-track-flat', 'ko',
     "        need = reach + s.width / 2.0 + config.pair_clearance(",
     "        need = reach + s.width / 2.0 + clr + 0 * config.pair_clearance(",
     (_t(T_ADM, 'sliver'),), 'KILLED'),
    ('stitch-via-pad-flat', 'ko',
     "                    + config.pad_pair_clearance(pd2, net_id,",
     "                    + config.clearance + 0 * config.pad_pair_clearance(pd2, net_id,",
     (_t(T_ADM, 'stitching'),), 'KILLED'),
    ('swap-via-track-flat', 'lso',
     "            need = vr + sg.width / 2.0 + config.pair_clearance(",
     "            need = vr + sg.width / 2.0 + config.clearance + 0 * config.pair_clearance(",
     (_t(T_ADM, 'swap'),), 'KILLED'),
    ('swap-via-margin-removed', 'lso',
     "                v.net_id, sg.net_id, sg.layer) - margin",
     "                v.net_id, sg.net_id, sg.layer)",
     (_t(T_ADM, 'swap'),), 'KILLED'),
    ('fans-fit-via-flat', 'dpm',
     "        return (clearance if a == b",
     "        return (clearance if True",
     (_t(T_ADM, 'fans'),), 'KILLED'),
    ('meander-track-flat', 'lm',
     "                + config.pair_clearance(net_id, o_net, layer, kind='track')",
     "                + config.clearance",
     (_t(T_ADM, 'meander_amplitude'),), 'KILLED'),
    ('meander-query-radius-narrow', 'lm',
     "                _q_req + _FOREIGN_WIDTH_SLACK",
     "                required_clearance + _FOREIGN_WIDTH_SLACK",
     (_t(T_ADM, 'query_reaches'),), 'KILLED'),
    ('stub-pads-config-width', 'sls',
     "                if best < seg_half + config.pad_pair_clearance(",
     "                if best < config.track_width / 2 + config.pad_pair_clearance(",
     (_t(T_ADM, 'stub_pad_check'),), 'KILLED'),
    ('rescue-leg-flat', 'nr',
     "    _flat = config is None or config.pair_clearance_inert()",
     "    _flat = True",
     (_t(T_ADM, 'rescue_leg'),), 'KILLED'),
    ('cap-conflicts-flat', 'nr',
     "                        + config.pad_pair_clearance(p2, net_id,",
     "                        + config.clearance + 0 * config.pad_pair_clearance(p2, net_id,",
     (_t(T_ADM, 'cap_relocation'),), 'KILLED'),
    # ---- the phase-6 verifier's missing rows ---------------------------------
    ('swap-pad-override-max', 'lso',
     "                pad_clr = config.pad_override_clearance(pad_base, pad)",
     "                pad_clr = max(pad_base, getattr(pad, 'local_clearance', 0.0) or 0.0)",
     (_t(T_ADM, 'overrides_replace'),), 'KILLED'),
    ('swap-pad-margin-always', 'lso',
     "                pad_margin = margin if pad_clr == pad_base else 0.0",
     "                pad_margin = margin",
     (_t(T_ADM, 'overrides_replace'),), 'KILLED'),
    ('fans-pad-override-max', 'dpm',
     "                    pad_clr = config.pad_override_clearance(pad_base, pad)",
     "                    pad_clr = max(pad_base, getattr(pad, 'local_clearance', 0.0) or 0.0)",
     (_t(T_ADM, 'overrides_replace'),), 'KILLED'),
    ('meander-via-flat', 'lm',
     "                + config.pair_clearance(net_id, o_net, layer)",
     "                + config.clearance",
     (_t(T_ADM, 'index_and_extra'),), 'KILLED'),
    ('meander-pad-flat', 'lm',
     "        return (net_half + config.pad_pair_clearance(pad, net_id, layer=layer)",
     "        return (net_half + config.clearance + 0 * config.pad_pair_clearance(pad, net_id, layer=layer)",
     (_t(T_ADM, 'index_and_extra'),), 'KILLED'),
    ('meander-pad-override-ignored-when-inert', 'lm',
     "                _pc_inert and not getattr(pad, 'local_clearance', 0)):",
     "                _pc_inert):",
     (_t(T_ADM, 'pad_override_on_an_inert'),), 'KILLED'),
    ('meander-via-query-narrow', 'lm',
     "                    _q_via + _FOREIGN_WIDTH_SLACK",
     "                    via_clearance + _FOREIGN_WIDTH_SLACK",
     (_t(T_ADM, 'via_query'),), 'KILLED'),
    ('diff-meander-n-half-dropped', 'lm',
     "            pc = max(config.pair_clearance(p_net_id, o_net, layer_name,",
     "            pc = min(config.pair_clearance(p_net_id, o_net, layer_name,",
     (_t(T_ADM, 'either_half'),), 'KILLED'),
    ('diff-meander-via-flat', 'lm',
     "            pc = max(config.pair_clearance(p_net_id, o_net, layer_name),",
     "            pc = config.clearance + 0 * max(config.pair_clearance(p_net_id, o_net, layer_name),",
     (_t(T_ADM, 'either_half'),), 'KILLED'),
    ('diff-meander-pad-flat', 'lm',
     "        pc = max(config.pad_pair_clearance(pad, p_net_id, layer=layer_name),",
     "        pc = config.clearance + 0 * max(config.pad_pair_clearance(pad, p_net_id, layer=layer_name),",
     (_t(T_ADM, 'either_half'),), 'KILLED'),
    ('cap-relocation-via-flat', 'nr',
     "                    half + v.size / 2.0 + _clr(pad, v.net_id):",
     "                    half + v.size / 2.0 + clearance:",
     (_t(T_ADM, 'cap_relocation_via'),), 'KILLED'),
    ('cap-relocation-pad-flat', 'nr',
     "                if _m.hypot(max(_gx, 0.0), max(_gy, 0.0)) < _clr(",
     "                if _m.hypot(max(_gx, 0.0), max(_gy, 0.0)) < clearance + 0 * _clr(",
     (_t(T_ADM, 'cap_relocation_via'),), 'KILLED'),
    ('stitch-via-via-layer-kind', 'ko',
     "                + config.pair_clearance(net_id, v2.net_id, kind='stack')):",
     "                + config.pair_clearance(net_id, v2.net_id, kind='layer')):",
     (_t(T_ADM, 'kind_and_layer'),), 'KILLED'),
    ('stub-pads-origin-layer', 'sls',
     "                        pad, net_id, layer=dest_layer):",
     "                        pad, net_id, layer=seg.layer):",
     (_t(T_ADM, 'kind_and_layer'),), 'KILLED'),
    ('barrel-track-stack-kind', 'sls',
     "        if dist < via_r + config.pair_clearance(net_id, seg.net_id,",
     "        if dist < via_r + config.stack_clearance(config.clearance) + 0 * config.pair_clearance(net_id, seg.net_id,",
     (_t(T_ADM, 'kind_and_layer'),), 'KILLED'),
    ('unblock-refit-flat', 'se',
     "            _base, _lncl = _pair_floor(config, net_id, layer)",
     "            _base, _lncl = config.clearance, None",
     (_t(T_ADM, 'unblock'),), 'KILLED'),
    ('merge-terminal-flat', 'se',
     "    _base, _ncl = _pair_floor(config, net_id, ol)",
     "    _base, _ncl = config.clearance, None",
     (_t(T_ADM, 'merge_terminal'),), 'KILLED'),
    ('collapse-leg-flat', 'dpr',
     "        _base, _ncl = _pair_floor(config, net_id, pen.layer)",
     "        _base, _ncl = config.clearance, None",
     (GATE,), 'KILLED'),
    # ---- the oracle and both fronts ----------------------------------------
    ('oracle-map-not-rekeyed', 'ko',
     "        c = by_name.get(getattr(net, 'name', None))",
     "        c = by_name.get(nid)",
     (ORC,), 'KILLED'),
    ('gui-forward-dropped', 'gu',
     "            net_clearances_by_name=net_clearances_by_name)",
     "            net_clearances_by_name=None)",
     (ORC,), 'KILLED'),
    ('payload-key-dropped', 'route',
     "                    'net_clearances_by_name':",
     "                    'net_clearances_by_name_x':",
     (ORC,), 'KILLED'),
    ('repair-main-map-dropped', 'rp',
     "                                net_clearances_by_name=LAST_NET_CLEARANCES_BY_NAME)",
     "                                net_clearances_by_name=None)",
     (ORC,), 'KILLED'),
    # ---- the phase-5 and phase-7 verifiers' rows -----------------------------
    ('inherited-graze-refused', 'rr',
     "    _inherited = (getattr(pcb_data, '_input_copper_keys', None)",
     "    _inherited = (None and getattr(pcb_data, '_input_copper_keys', None)",
     (_t('test_980_restore_pairwise.py', 'inherited'),), 'KILLED'),
    ('restore-via-via-layer-kind', 'rr',
     "                else _clr(vv.net_id, v.net_id, None, 'stack'))",
     "                else _clr(vv.net_id, v.net_id, None, 'layer'))",
     (_t('test_980_restore_pairwise.py', 'kinds'),), 'KILLED'),
    ('restore-track-via-layer-dropped', 'rr',
     "                else _clr(s.net_id, v.net_id, s.layer, 'layer'))",
     "                else _clr(s.net_id, v.net_id, None, 'layer'))",
     (_t('test_980_restore_pairwise.py', 'kinds'),), 'KILLED'),
    ('restore-track-rule-dropped', 'rr',
     "                else _clr(s.net_id, o.net_id, s.layer, 'track'))",
     "                else _clr(s.net_id, o.net_id, s.layer, 'layer'))",
     (_t('test_980_restore_pairwise.py', 'kinds'),), 'KILLED'),
    ('plane-twin-via-via-layer-kind', 'pbd',
     "                               else _clr(None, 'stack'))",
     "                               else _clr(None, 'layer'))",
     (_t('test_980_restore_pairwise.py', 'kinds'),), 'KILLED'),
    ('oracle-rekey-dropped', 'ko',
     "            config.net_clearances = _oracle_class_map(pcb, _ncl_by_name)",
     "            pass",
     (OBH,), 'KILLED'),
    ('oracle-private-copy-dropped', 'ko',
     "        config = replace(config)",
     "        pass",
     (OBH,), 'KILLED'),
    ('oracle-link-floor-dropped', 'ko',
     "                config.net_clearance_floor = max(",
     "                config.net_clearance_floor = None and max(",
     (OBH,), 'KILLED'),
    ('aliased-oracle-call-map-dropped', 'route',
     "                        net_clearances_by_name=config.net_clearances_by_name(",
     "                        net_clearances_by_name={}, _unused=config.net_clearances_by_name(",
     (ORC,), 'KILLED'),
    # ---- the inherited-graze carve-out and the step's input record ----------
    ('inherited-mine-not-checked', 'rr',
     "                and copper_key(mine) in _inherited",
     "                and True",
     (_t('test_980_restore_pairwise.py', 'every_pair_kind'),), 'KILLED'),
    ('inherited-track-via-flat-dropped', 'rr',
     "                    s, v, _d, hw + v.size / 2.0 + clearance):",
     "                    s, v, _d, hw + v.size / 2.0):",
     (_t('test_980_restore_pairwise.py', 'every_pair_kind'),), 'KILLED'),
    ('inherited-via-via-flat-dropped', 'rr',
     "                    vv, v, _d, vr + v.size / 2.0 + clearance):",
     "                    vv, v, _d, vr + v.size / 2.0):",
     (_t('test_980_restore_pairwise.py', 'every_pair_kind'),), 'KILLED'),
    ('inherited-via-track-flat-dropped', 'rr',
     "                    vv, o, _d, vr + o.width / 2.0 + clearance):",
     "                    vv, o, _d, vr + o.width / 2.0):",
     (_t('test_980_restore_pairwise.py', 'every_pair_kind'),), 'KILLED'),
    ('copper-key-no-net', 'rr',
     "        return ('s', item.net_id, item.layer, round(item.width, 4),",
     "        return ('s', 0, item.layer, round(item.width, 4),",
     (_t('test_980_restore_pairwise.py', 'mark_is_each'),), 'KILLED'),
    ('copper-key-no-width', 'rr',
     "        return ('s', item.net_id, item.layer, round(item.width, 4),",
     "        return ('s', item.net_id, item.layer, 0,",
     (_t('test_980_restore_pairwise.py', 'mark_is_each'),), 'KILLED'),
    ('mark-force-ignored', 'rr',
     "        if (not force",
     "        if (True",
     (_t('test_980_restore_pairwise.py', 'mark_is_each'),), 'KILLED'),
    ('mark-keys-ignored', 'rr',
     "        if keys is not None:",
     "        if False:",
     (_t('test_980_restore_pairwise.py', 'mark_is_each'),), 'KILLED'),
    ('route-mark-dropped', 'route',
     "    mark_input_copper(pcb_data, force=_parsed_here, keys=input_copper_keys)",
     "    pass",
     (_t('test_980_restore_pairwise.py', 'mark_is_each'),), 'KILLED'),
    ('route-mark-not-forced', 'route',
     "    mark_input_copper(pcb_data, force=_parsed_here, keys=input_copper_keys)",
     "    mark_input_copper(pcb_data, force=False, keys=input_copper_keys)",
     (_t('test_980_restore_pairwise.py', 'mark_is_each'),), 'KILLED'),
    ('reconcile-record-not-forwarded', 'route',
     "    _reconcile_kwargs['input_copper_keys'] = getattr(",
     "    _unforwarded_keys = getattr(",
     (_t('test_980_restore_pairwise.py', 'mark_is_each'),), 'KILLED'),
    ('finalize-record-not-forwarded', 'route',
     "                        input_copper_keys=getattr(",
     "                        _unforwarded_keys=getattr(",
     (_t('test_980_restore_pairwise.py', 'mark_is_each'),), 'KILLED'),
    ('repair-record-not-installed', 'rp',
     "    mark_input_copper(pcb_data, keys=input_copper_keys)",
     "    mark_input_copper(pcb_data)",
     (_t('test_980_restore_pairwise.py', 'mark_is_each'),), 'KILLED'),
    ('gui-sync-keeps-a-stale-record', 'sg',
     "        forget_input_copper(self.pcb_data)",
     "        pass",
     (_t('test_980_restore_pairwise.py', 'mark_is_each'),), 'KILLED'),
    ('escalation-map-dropped', 'ko',
     "                        pcb_data, net_id, _gap, config,\n"
     "                        config.net_clearances or None,",
     "                        pcb_data, net_id, _gap, config,\n"
     "                        None,",
     (_t('test_980_oracle_class_map.py', 'escalation'),), 'KILLED'),
    ('cap-config-rules-without-board', 'route',
     "                                                     input_file, pcb_data)",
     "                                                     input_file, None)",
     (_t('test_980_oracle_class_map.py', 'escalation'),), 'KILLED'),
    # ---- a change detector ---------------------------------------------------
    ('pair-args-swapped', 'ko',
     "                + config.pair_clearance(net_id, s2.net_id, s2.layer)):",
     "                + config.pair_clearance(s2.net_id, net_id, s2.layer)):",
     (_t(T_ADM, 'stitching'),), 'SURVIVED'),
]

sys.path.insert(0, _TESTS)
from mutation_anchors import preflight   # noqa: E402
preflight(__file__)


def _dirty(path):
    p = subprocess.run(['git', 'status', '--porcelain', '--', path],
                       capture_output=True, text=True, cwd=_ROOT)
    return bool(p.stdout.strip())


def _run_tests(tests):
    failed = []
    for t in tests:
        p = subprocess.run([sys.executable, '-X', 'utf8', t[0]] + list(t[1:]),
                           capture_output=True, text=True, encoding='utf-8',
                           errors='replace', timeout=2400, cwd=_ROOT)
        if p.returncode != 0:
            failed.append((os.path.basename(t[0]) + ':' + ','.join(t[1:]),
                           p.returncode,
                           [ln.strip()[:90] for ln in
                            ((p.stdout or '') + (p.stderr or '')).splitlines()
                            if 'FAIL' in ln or 'Error' in ln][:2]))
    return failed


def run(only=None):
    rows = [r for r in ROWS if only is None or r[0] == only]
    if not rows:
        print('no row named %r' % only)
        return 1
    for path in TARGETS.values():
        if _dirty(path):
            print('REFUSING: %s has uncommitted changes. Commit or stash '
                  'first -- this battery restores by overwriting.'
                  % os.path.basename(path))
            return 2
    # THE UNMUTATED BASELINE: every witness must pass as the code stands,
    # or a row it "kills" proves nothing.
    witnesses = sorted({t for r in rows for t in r[4]})
    base_fail = _run_tests(witnesses)
    if base_fail:
        print('REFUSING: witnesses fail UNMUTATED -- %s' % base_fail)
        return 2
    print('baseline: %d witnesses pass unmutated' % len(witnesses))
    orig = {k: io.open(v, encoding='utf-8', newline='').read()
            for k, v in TARGETS.items()}
    results = []
    try:
        for name, tgt, old, new, tests, expect in rows:
            path = TARGETS[tgt]
            base = orig[tgt]
            o, n = old, new
            if '\r\n' in base:
                o, n = o.replace('\n', '\r\n'), n.replace('\n', '\r\n')
            if base.count(o) != 1 or o == n:
                results.append((name, 'BROKEN', expect,
                                ['anchor matched %d times' % base.count(o)]))
                continue
            io.open(path, 'w', encoding='utf-8', newline='').write(
                base.replace(o, n, 1))
            try:
                failed = _run_tests(tests)
            finally:
                io.open(path, 'w', encoding='utf-8', newline='').write(base)
            results.append((name, 'KILLED' if failed else 'SURVIVED',
                            expect, [str(f)[:150] for f in failed[:2]]))
            print('%-36s %s' % (name, results[-1][1]), flush=True)
    finally:
        for k, v in TARGETS.items():
            io.open(v, 'w', encoding='utf-8', newline='').write(orig[k])
    wrong = [r for r in results if r[1] != r[2]]
    print('')
    for name, verdict, expect, why in results:
        print('%-36s %-9s%s' % (name, verdict, '' if verdict == expect else
                                '   <-- WRONG, expected %s' % expect))
        for w in why:
            print('      %s' % w)
    print('\n%d rows: %d killed, %d survived, %d broken'
          % (len(results), sum(r[1] == 'KILLED' for r in results),
             sum(r[1] == 'SURVIVED' for r in results),
             sum(r[1] == 'BROKEN' for r in results)))
    return 1 if wrong else 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--row', default=None, help='run only this row')
    a = ap.parse_args()
    return run(a.row)


if __name__ == '__main__':
    sys.exit(main())
