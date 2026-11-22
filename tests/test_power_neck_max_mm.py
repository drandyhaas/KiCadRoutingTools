"""--power-neck-max-mm bounds a power net's neck under every policy (#1872).

Before: the 2 mm PAD_ENTRY_NECK_MM allowance applied only under
--escalation off; under board a 3 mm rail shipped 40.42 mm of 0.25 mm track.
The flag sets the bound under every policy, the shipped necks are listed in
--json-out as power_necks, and the CLI/GUI/plan fronts agree on it.
"""
import json
import math
import os
import sys
import tempfile
from dataclasses import replace
from types import SimpleNamespace

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'rust_router'))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
sys.path.insert(0, os.path.join(ROOT, 'tests', 'oracle'))

import fab_tiers  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402
from routing_config import GridRouteConfig  # noqa: E402
from routing_defaults import PAD_ENTRY_NECK_MM  # noqa: E402
from routing_diagnostics import static_boxin_hint  # noqa: E402
from routing_common import power_neck_report  # noqa: E402
from run_utils import check as run_check, evidence, tool  # noqa: E402
from single_ended_routing import (  # noqa: E402
    _neck_path_ok, _neckdown_allowed, _pad_entry_zone_mm)
import net_rescue  # noqa: E402
from constraint_agreement import write_board  # noqa: E402

FIXTURE = os.path.join(ROOT, 'tests', 'fixtures', '1452', 'neck.kicad_pcb')
TRACK_W, CLEARANCE = 0.25, 0.2
fails = []


def check(name, ok, detail=''):
    print(('PASS' if ok else 'FAIL') + f': {name}' + (f' -- {detail}' if detail else ''))
    if not ok:
        fails.append(name)


def _cfg(flag=None):
    return GridRouteConfig(power_neck_max_mm=flag)


def _with_policy(policy, fn):
    prev = fab_tiers.get_escalation_policy()
    fab_tiers.set_escalation_policy(policy)
    try:
        return fn()
    finally:
        fab_tiers.set_escalation_policy(*prev)


# ---- 1. the allowance and its label -----------------------------------------
print("1. power_neck_allowance_mm / power_neck_bound_label")
for pol, flag, want_a, want_l in (
        ('off', None, PAD_ENTRY_NECK_MM, '--escalation off'),
        ('fab', None, None, None),
        ('board', None, None, None),
        ('off', 2.0, 2.0, '--power-neck-max-mm 2'),
        ('fab', 2.0, 2.0, '--power-neck-max-mm 2'),
        ('board', 2.0, 2.0, '--power-neck-max-mm 2')):
    def _run(pol=pol, flag=flag, want_a=want_a, want_l=want_l):
        cfg = _cfg(flag)
        return (fab_tiers.power_neck_allowance_mm(cfg),
                fab_tiers.power_neck_bound_label(cfg))
    a, l = _with_policy(pol, _run)
    check(f"{pol}, {'no flag' if flag is None else f'flag {flag:g}'}: "
          f"the allowance is {want_a if want_a is not None else 'unbounded'}",
          a == want_a, f"got {a}")
    check(f"{pol}, {'no flag' if flag is None else f'flag {flag:g}'}: "
          f"the bound is {'the policy' if want_l == '--escalation off' else 'the flag' if want_l else 'nothing'}",
          l == want_l, f"got {l}")
check("GUI 0 means no flag: unbounded under board",
      _with_policy('board', lambda: fab_tiers.power_neck_allowance_mm(
          GridRouteConfig(power_neck_max_mm=0.0))) is None)
for bad in (float('nan'), float('inf')):
    try:
        GridRouteConfig(power_neck_max_mm=bad)
        check(f"non-finite {bad} is rejected by the config", False, "accepted")
    except ValueError as e:
        check(f"non-finite {bad} is rejected by the config", True, str(e))
check("a non-finite value that reaches the helper is no flag, never a bound",
      _with_policy('board', lambda: fab_tiers.power_neck_allowance_mm(
          SimpleNamespace(power_neck_max_mm=float('nan')))) is None)


# ---- 2. the ladder gates ----------------------------------------------------
print("2. _neckdown_allowed / _neck_path_ok / _pad_entry_zone_mm")
cfg2 = _cfg(2.0)
cfg0 = _cfg(None)
cfg5 = _cfg(5.0)


def _pol2(pol):
    def run():
        return (_neckdown_allowed(cfg2, 40.0), _neckdown_allowed(cfg2, 1.5),
                _neckdown_allowed(cfg0, 40.0),
                _neck_path_ok([(0, 0, 0), (800, 0, 0)], 0.05, cfg2),
                _neck_path_ok([(0, 0, 0), (30, 0, 0)], 0.05, cfg2),
                _neck_path_ok([(0, 0, 0), (800, 0, 0)], 0.05, cfg0),
                _pad_entry_zone_mm(cfg2), _pad_entry_zone_mm(cfg0))
    return _with_policy(pol, run)


(r40, r15, r40nf, p40, p15, p40nf, z2, z0) = _pol2('board')
check("board, flag 2: a 40 mm span may not neck", r40 is False)
check("board, flag 2: a 1.5 mm span may neck", r15 is True)
check("board, no flag: a 40 mm span may neck (control)", r40nf is True)
check("board, flag 2: a 40 mm necked path is refused", p40 is False)
check("board, flag 2: a 1.5 mm necked path is allowed", p15 is True)
check("board, no flag: a 40 mm necked path is allowed (control)", p40nf is True)
check("board, flag 2: the zone is the allowance less the taper",
      abs(z2 - 1.5) < 1e-9, f"zone {z2}")
check("board, no flag: the zone is neckdown_length (control)",
      z0 == GridRouteConfig().neckdown_length, f"zone {z0}")
check("off, flag 5: the zone is neckdown_length (allowance is wider)",
      _with_policy('off', lambda: _pad_entry_zone_mm(cfg5))
      == GridRouteConfig().neckdown_length)
# Fix 5: a bound at or below the taper leaves no pad-entry zone.
for flag, want in ((0.5, 0.0), (0.4, 0.0), (1.0, 0.5)):
    z = _with_policy('board', lambda f=flag: _pad_entry_zone_mm(_cfg(f)))
    check(f"board, flag {flag:g}: the zone is {want:g} (taper counts inside)",
          abs(z - want) < 1e-9, f"zone {z}")
# Fix 4: the terminal-escalation gate, direct on the synthetic board.
P, F = 1, 2


def _rescue_board(td):
    path = os.path.join(td, 'r.kicad_pcb')
    write_board(path, segments=[(20.0, -1.0, 20.0, 41.0, 0.2, 'F.Cu', F),
                                (19.0, -1.0, 19.0, 41.0, 0.2, 'B.Cu', F)],
                footprints=[
                    {'ref': 'U1', 'x': 5, 'y': 20, 'net_id': P, 'net_name': 'A', 'layer': 'F.Cu'},
                    {'ref': 'U2', 'x': 35, 'y': 20, 'net_id': P, 'net_name': 'A', 'layer': 'B.Cu'},
                    {'ref': 'U3', 'x': 30, 'y': 3, 'net_id': F, 'net_name': 'B', 'layer': 'F.Cu'},
                    {'ref': 'U4', 'x': 30, 'y': 37, 'net_id': F, 'net_name': 'B', 'layer': 'F.Cu'}])
    return path


def _rescue_cfg(width):
    return GridRouteConfig(layers=['F.Cu', 'B.Cu'], track_width=0.25, clearance=0.15,
                           grid_step=0.1, via_size=0.45, via_drill=0.2,
                           net_via_sizes={P: (0.8, 0.4)}, max_iterations=200000,
                           power_net_widths={P: width})


with tempfile.TemporaryDirectory() as _td:
    _pcb = parse_kicad_pcb(_rescue_board(_td))

    def _ladder(width, flag):
        cfg = _rescue_cfg(width)
        if flag is not None:
            cfg = replace(cfg, power_neck_max_mm=flag)
        return [round(r.get_net_track_width(P, 'F.Cu'), 3)
                for r in net_rescue._size_escalation_ladder(cfg, _pcb, P)]

    rungs = _with_policy('fab', lambda: _ladder(0.5, None))
    check("rescue gate: a wide net with no flag still gets size rungs",
          rungs == [0.376, 0.251, 0.127], str(rungs))
    check("rescue gate: flag 2 leaves a 0.5 mm net no size rungs",
          _with_policy('fab', lambda: _ladder(0.5, 2.0)) == [])
    rungs25 = _with_policy('fab', lambda: _ladder(0.25, 2.0))
    check("rescue gate: a net at exactly track width still gets rungs",
          rungs25 == [0.209, 0.168, 0.127], str(rungs25))


# ---- 3. the hint ------------------------------------------------------------
print("3. static_boxin_hint")
cfg = GridRouteConfig(track_width=TRACK_W, clearance=CLEARANCE)
base = {'failed': True, 'iterations_forward': 5000, 'iterations_backward': 1}
refused = dict(base, pad_entry_refused={
    'span_mm': 7.5, 'allowance_mm': PAD_ENTRY_NECK_MM,
    'net_width_mm': 1.2, 'neck_width_mm': TRACK_W,
    'bound': '--power-neck-max-mm 2'})
hint, verdict = static_boxin_hint(refused, cfg, return_verdict=True)
check("a flag-driven refusal names the flag and the allowance",
      '--power-neck-max-mm 2' in hint and 'within 2 mm' in hint, hint)
check("... and tells the user to raise the flag",
      'raise --power-neck-max-mm' in hint, hint)
check("... and does not blame the escalation policy",
      '--escalation off' not in hint and 'allow narrowing' not in hint, hint)
check("... and does not advise a smaller track", '--track-width' not in hint, hint)
check("... and its verdict says why",
      verdict == {'span_mm': 7.5, 'allowance_mm': 2.0, 'net_width_mm': 1.2,
                  'neck_width_mm': 0.25, 'bound': '--power-neck-max-mm 2',
                  'verdict': 'pad_entry_allowance', 'iterations': 5001},
      str(verdict))
refused0 = dict(base, pad_entry_refused={
    'span_mm': 7.5, 'allowance_mm': PAD_ENTRY_NECK_MM,
    'net_width_mm': 0.5, 'neck_width_mm': TRACK_W})
hint0, _ = static_boxin_hint(refused0, cfg, return_verdict=True)
check("control: the off hint is unchanged",
      hint0 == ("Hint: under --escalation off a wide net may narrow only within "
                "2 mm of a pad; this edge spans 7.5 mm and a 0.25 mm track "
                "reaches it, but no route keeps 0.5 mm outside that pad entry. "
                "Open a wider channel to the pad, or allow narrowing "
                "(--escalation fab)."),
      hint0)


# ---- 4. the report ----------------------------------------------------------
print("4. power_neck_report")


def _seg(x0, y0, x1, y1, w, layer='F.Cu', nid=1):
    return SimpleNamespace(net_id=nid, width=w, layer=layer,
                           start_x=x0, start_y=y0, end_x=x1, end_y=y1)


_pad = SimpleNamespace(layers=['F.Cu'], global_x=10.0, global_y=0.0,
                       size_x=1.0, size_y=1.0, component_ref='U1', pad_number='4')
_segs = [_seg(8.0, 0.0, 9.0, 0.0, 0.25), _seg(9.0, 0.0, 10.0, 0.0, 0.25),
         _seg(20.0, 0.0, 20.5, 0.0, 0.4)]
got = power_neck_report(_segs, {1: 1.0}, lambda n: 'VCC',
                        lambda n: [_pad])
check("narrow runs are grouped, longest first, with the pad entered",
      got == {'VCC': [{'layer': 'F.Cu', 'width_mm': 0.25, 'length_mm': 2.0,
                       'pad': 'U1.4'},
                      {'layer': 'F.Cu', 'width_mm': 0.4, 'length_mm': 0.5,
                       'pad': None}]},
      str(got))
check("a net with no narrow copper lists empty",
      power_neck_report([], {2: 1.0}, lambda n: 'OTHER', lambda n: [])
      == {'OTHER': []})


# ---- 5.-8. end to end on the 1452 fixture -----------------------------------
def route(tmp, width, escalation, flag=None):
    out = os.path.join(tmp, f'out_{escalation}_{width:g}_{flag}.kicad_pcb')
    js = os.path.join(tmp, f'out_{escalation}_{width:g}_{flag}.json')
    argv = [sys.executable, '-X', 'utf8', tool('route.py'), FIXTURE, out,
            '--nets', 'VCC', '--layers', 'F.Cu',
            '--track-width', str(TRACK_W), '--clearance', str(CLEARANCE),
            '--via-size', '0.6', '--via-drill', '0.3',
            '--escalation', escalation,
            '--power-nets', 'VCC', '--power-nets-widths', str(width),
            '--json-out', js]
    if flag is not None:
        argv += ['--power-neck-max-mm', str(flag)]
    r = run_check(argv, accept=True)
    return evidence(out, 'routed board'), r.stdout + r.stderr, js


def _rows(out, width):
    pcb = parse_kicad_pcb(out)
    vcc = next(n for n, net in pcb.nets.items() if net.name == 'VCC')
    return power_neck_report(
        [s for s in pcb.segments if s.net_id == vcc], {vcc: width},
        lambda n: 'VCC', lambda n: pcb.nets[n].pads)['VCC']


def _connected(out):
    r = run_check([sys.executable, '-X', 'utf8', tool('check_connected.py'),
                   out, '--nets', 'VCC'], accept=True)
    return r.stdout


EXPECTED_05 = [{'layer': 'F.Cu', 'width_mm': 0.25, 'length_mm': 1.67,
                'pad': 'U1.4'},
               {'layer': 'F.Cu', 'width_mm': 0.25, 'length_mm': 0.14,
                'pad': None}]

tmp = tempfile.mkdtemp(prefix='t1872_')
try:
    print("5. board, net at 0.5 mm, flag 2")
    out, log, js = route(tmp, 0.5, 'board', 2.0)
    check("all VCC pads connected (check_connected)",
          'ALL NETS FULLY CONNECTED' in _connected(out))
    rows = _rows(out, 0.5)
    check("no shipped neck exceeds the flag", rows == EXPECTED_05, str(rows))
    check("the log names the pad-entry neck under the flag",
          'pad-entry neck' in log and '--power-neck-max-mm 2' in log)
    doc = json.load(open(js))
    check("--json-out lists the shipped necks",
          doc.get('power_necks', {}).get('VCC') == EXPECTED_05,
          str(doc.get('power_necks')))

    print("6. board, net at 0.5 mm, NO flag (control)")
    out, log, js = route(tmp, 0.5, 'board')
    check("all VCC pads connected (check_connected)",
          'ALL NETS FULLY CONNECTED' in _connected(out))
    rows = _rows(out, 0.5)
    check("without the flag, necks longer than 2 mm still ship (control)",
          len([r for r in rows if r['length_mm'] > 2.0]) == 2, str(rows))

    print("7. board, net at 1.2 mm, flag 2: refused, and said so")
    out, log, js = route(tmp, 1.2, 'board', 2.0)
    doc = json.load(open(js))
    check("the pin stays open",
          'VCC' in (doc.get('failed_single') or [])
          or 'Disconnected' in _connected(out),
          str(doc.get('failed_single')))
    check("the log names the refusal, the span and the allowance",
          'Neck-down refused (--power-neck-max-mm 2)' in log
          and 'spans' in log and '2mm pad-entry allowance' in log)
    check("no terminal escalation shipped a longer neck instead",
          'Terminal geometry escalation' not in log)
    hints = [ln for ln in log.splitlines() if 'Hint:' in ln]
    check("a hint names the allowance and the flag",
          any('--power-neck-max-mm 2' in h and 'raise --power-neck-max-mm' in h
              for h in hints),
          '\n'.join(hints))
    check("no hint advises a smaller track",
          all('--track-width' not in h for h in hints), '\n'.join(hints))
    check("--json-out lists no shipped neck",
          doc.get('power_necks') == {'VCC': []}, str(doc.get('power_necks')))

    print("8. off, net at 0.5 mm, NO flag (control)")
    out, log, js = route(tmp, 0.5, 'off')
    check("all VCC pads connected (check_connected)",
          'ALL NETS FULLY CONNECTED' in _connected(out))
    rows = _rows(out, 0.5)
    check("the allowance stays 2 mm under off", rows == EXPECTED_05, str(rows))
finally:
    import shutil
    shutil.rmtree(tmp, ignore_errors=True)

if fails:
    print(f"\nFAILED: {len(fails)} check(s): " + '; '.join(fails))
    sys.exit(1)
print("\nALL PASS")
