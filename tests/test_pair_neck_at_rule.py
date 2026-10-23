#!/usr/bin/env python3
"""A diff pair coupled AT its clearance keeps its width (the #318 partner neck).

`_neck_pair_partner_grazes` necks a member's segment that sits inside the pair
clearance of its partner. Its 2e-4 margin sat in the TRIGGER -- what #1159
fixed in `_neck_terminal_grazes`, left in the pair's own copy -- so a coupled
run whose edge gap EQUALS the clearance counted as grazing and lost 0.4 um of
width. route_diff builds exactly that whenever the requested gap is at or
below the clearance, because it raises the gap to the clearance (#441). A USB
pair routed at --track-width 0.2 --diff-pair-gap 0.2 --clearance-ceiling 0.2
shipped six P segments at 0.1996; the output project's min_track_width
followed them down ("FAB FLOOR RELAXED -- track width: 0.2 -> 0.1996 mm"),
and check_complete --authored-from refused the finished board as UNSOUND.
The floor writeback was right to move (KiCad grades a 0.1996 track against a
0.2 minimum as track_width), so the copper is what had to change: a gap at
the rule is legal, and the margin belongs to the neck TARGET only.

Rows:
  1. the reproduction's geometry (P jogs 0.4 mm centre to centre from N's
     coupled middle, 0.2 wide at 0.2): no neck, every width 0.2, no hard;
  2. a diagonal coupled run built at the rule (irrational offsets): no neck;
  3. the 1 nm boundary (NECK_GRAZE_TOL): 0.5 nm inside is the rule, 10 nm
     inside is a graze;
  4. 10 um inside the rule: necked by the shortfall to just inside the rule,
     and the narrowing is in the design-rules ledger ('pair partner neck');
  5. collinear members end to end (no neck can clear them): still hard, and
     no ledger row for copper the caller refuses;
  6. route_diff on cap_chain at gap == clearance: every pair segment ships at
     the requested 0.2 and the written project keeps min_track_width 0.2
     (the parent commit shipped 10 segments at 0.1996 and wrote 0.1996).

    python3 tests/test_pair_neck_at_rule.py
"""
import json
import math
import os
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for _d in ('py_router', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _d))

import fab_tiers                                                    # noqa: E402
from kicad_parser import BoardInfo, parse_kicad_pcb                 # noqa: E402
from routing_config import GridRouteConfig                          # noqa: E402
from synth import make_pcb, make_seg                                # noqa: E402
from diff_pair_routing import _neck_pair_partner_grazes             # noqa: E402
from run_utils import check as run_check, evidence, tool            # noqa: E402

RUN_ALL_TIMEOUT = 300
P, N = 16, 17
W, CLR = 0.2, 0.2
LAYERS = ['F.Cu', 'B.Cu']
SITE = 'pair partner neck'
FAILS = []


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


PCB = make_pcb(board_info=BoardInfo(layers={}, copper_layers=list(LAYERS),
                                    board_bounds=(0, 0, 200, 200)))


def neck(p_segs, n_segs):
    """Run the partner neck on a fresh ledger: (necked, hard, ledger rows)."""
    fab_tiers.reset_ledger()
    cfg = GridRouteConfig(layers=list(LAYERS), clearance=CLR, track_width=W,
                          diff_pair_gap=CLR)
    necked, hard = _neck_pair_partner_grazes(p_segs, n_segs, cfg, PCB)
    rows = [r for r in fab_tiers.escalation_summary()['narrowed']
            if r['site'] == SITE]
    return necked, hard, rows


def widths(segs):
    return sorted({s.width for s in segs})


print("1. the reproduction: P's jogs at the rule beside N's coupled middle")
# /D_P's F.Cu copper around the six segments route_diff necked to 0.1996, and
# /D_N's run beside it (133.5 -> 131.2 at y=92.2), from the USB board.
p = [make_seg(134.6, 94.0, 134.6, 93.9, net_id=P),
     make_seg(134.6, 93.9, 133.3, 92.6, net_id=P),
     make_seg(133.3, 92.6, 133.2, 92.6, net_id=P),
     make_seg(133.2, 92.6, 133.1, 92.7, net_id=P),
     make_seg(133.1, 92.7, 132.1, 92.7, net_id=P),
     make_seg(132.1, 92.7, 132.0, 92.6, net_id=P),
     make_seg(132.0, 92.6, 131.9, 92.6, net_id=P),
     make_seg(131.9, 92.6, 131.5, 93.0, net_id=P),
     make_seg(131.5, 93.0, 131.5, 94.7, net_id=P)]
n = [make_seg(136.2, 94.9, 133.5, 92.2, net_id=N),
     make_seg(133.5, 92.2, 131.2, 92.2, net_id=N),
     make_seg(131.2, 92.2, 129.9, 93.5, net_id=N)]
necked, hard, rows = neck(p, n)
check('no segment is necked', necked == 0, f'necked={necked}')
check('every member keeps the requested width', widths(p + n) == [W],
      f'widths {widths(p + n)}')
check('nothing is hard', not hard, f'{len(hard)} hard')
check('nothing is recorded as narrowed', not rows, f'{rows}')

print('2. a diagonal coupled run built at the rule')
# Centreline (10,10)->(14,14), members offset (W + CLR) / 2 to either side:
# the coordinates are irrational, so the gap is the rule to float noise only.
off = (W + CLR) / 2.0 / math.sqrt(2.0)
p = [make_seg(10 - off, 10 + off, 14 - off, 14 + off, net_id=P)]
n = [make_seg(10 + off, 10 - off, 14 + off, 14 - off, net_id=N)]
necked, hard, rows = neck(p, n)
check('the diagonal run keeps its width', necked == 0 and widths(p + n) == [W]
      and not hard, f'necked={necked} widths {widths(p + n)} hard={len(hard)}')

print('3. the 1 nm boundary')
for inside_nm, want in ((0.5, False), (10.0, True)):
    pitch = W + CLR - inside_nm * 1e-6
    p = [make_seg(20.0, 0.0, 22.0, 0.0, net_id=P)]
    n = [make_seg(20.0, pitch, 22.0, pitch, net_id=N)]
    necked, hard, rows = neck(p, n)
    if want:
        check(f'{inside_nm:g} nm inside the rule: a graze, P necked',
              necked == 1 and p[0].width < W and n[0].width == W and len(rows) == 1,
              f'necked={necked} widths P {p[0].width} N {n[0].width} rows={len(rows)}')
    else:
        check(f'{inside_nm:g} nm inside the rule: the rule, nothing necked',
              necked == 0 and widths(p + n) == [W] and not rows,
              f'necked={necked} widths {widths(p + n)} rows={len(rows)}')

print('4. a real graze: necked by the shortfall, and disclosed')
pitch = W + CLR - 0.01                       # edge gap 0.19 under a 0.2 rule
p = [make_seg(30.0, 0.0, 32.0, 0.0, net_id=P)]
n = [make_seg(30.0, pitch, 32.0, pitch, net_id=N)]
necked, hard, rows = neck(p, n)
gap = pitch - p[0].width / 2.0 - n[0].width / 2.0
check('P is necked and the pair is not refused',
      necked == 1 and not hard and n[0].width == W,
      f'necked={necked} hard={len(hard)} widths P {p[0].width} N {n[0].width}')
check('...by twice the 10 um shortfall (the centreline stays put)',
      abs((W - p[0].width) - 2 * 0.01) < 1e-3, f'P width {p[0].width}')
check('...to just inside the rule (the margin is on the TARGET)',
      CLR - 1e-9 <= gap < CLR + 1e-3, f'edge gap {gap:.5f}')
check('...and the design-rules ledger names it',
      len(rows) == 1 and rows[0]['net'] == P and rows[0]['kind'] == 'track_width'
      and rows[0]['requested'] == W and rows[0]['delivered'] == p[0].width,
      f'{rows}')

print('5. a graze no neck can clear stays hard, with no ledger row')
p = [make_seg(40.0, 0.0, 41.0, 0.0, net_id=P)]
n = [make_seg(41.3, 0.0, 43.0, 0.0, net_id=N)]    # end to end, 0.1 edge gap
necked, hard, rows = neck(p, n)
check('the collinear members are hard', bool(hard), f'{len(hard)} hard')
check('...and nothing is disclosed for copper the caller refuses', not rows,
      f'{rows}')

print('6. route_diff on cap_chain at gap == clearance')
with tempfile.TemporaryDirectory() as td:
    src = os.path.join(td, 'cc.kicad_pcb')
    shutil.copy(os.path.join(ROOT, 'kicad_files', 'cap_chain.kicad_pcb'), src)
    out = os.path.join(td, 'o.kicad_pcb')
    r = run_check([sys.executable, '-X', 'utf8', tool('route_diff.py'), src, out,
                   '--nets', 'DP*', '--track-width', str(W),
                   '--diff-pair-gap', str(CLR), '--clearance', str(CLR)],
                  accept=True)
    summary = next((json.loads(ln.split('JSON_SUMMARY: ', 1)[1])
                    for ln in r.stdout.splitlines()
                    if ln.startswith('JSON_SUMMARY: ')), None)
    check('the run prints its JSON_SUMMARY', summary is not None)
    summary = summary or {}
    check('both pairs route', summary.get('successful') == 2
          and summary.get('failed') == 0,
          f"successful={summary.get('successful')} failed={summary.get('failed')}")
    pcb = parse_kicad_pcb(evidence(out))
    names = {nn.net_id: nn.name for nn in pcb.nets.values()}
    dp = [s for s in pcb.segments if names.get(s.net_id, '').startswith('DP')]
    members = sorted({names[s.net_id] for s in dp})
    check('fixture: all four members carry copper',
          members == ['DPA_N', 'DPA_P', 'DPB_N', 'DPB_P'], f'{members}')
    check('every pair segment ships at the requested width',
          bool(dp) and widths(dp) == [W], f'{len(dp)} segments, widths {widths(dp)}')
    with open(evidence(os.path.join(td, 'o.kicad_pro')), encoding='utf-8') as fh:
        rules = json.load(fh)['board']['design_settings']['rules']
    check('the written project keeps min_track_width at the request',
          rules.get('min_track_width') == W, f'{rules.get("min_track_width")}')
    rows = [row for row in (summary.get('design_rules') or {}).get('narrowed', [])
            if row.get('kind') == 'track_width']
    check('no track narrowing is disclosed, because none shipped', not rows,
          f'{rows}')

print()
if FAILS:
    print(f'{len(FAILS)} FAILURE(S): {", ".join(FAILS)}')
    sys.exit(1)
print('ALL PASS')
