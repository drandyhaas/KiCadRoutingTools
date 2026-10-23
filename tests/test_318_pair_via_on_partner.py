#!/usr/bin/env python3
"""A diff pair whose via sits on its partner's track is refused, not written
(#318's intra-pair gate, which compared tracks only).

`_neck_pair_partner_grazes` necks a member's track that grazes its partner's
just-created copper and reports what necking cannot fix, so the router
refuses the pair instead of shipping P touching N. It compared TRACKS only.
On rp2040_motorctrl (set7) /CAN1 leg 1 J17 -> TP50 is re-routed for polarity
out of the target's opposite side; its centreline loops back over the via
pair it then drops, so each via lands on the partner's F.Cu diagonal. The
tracks necked against each other (0.0889 / 0.1363 mm) and the vias shipped
in contact: two VIA-SEGMENT shorts from route_diff, since #1303's seam
re-ask (ec1cefe3) stopped replacing that leg for losing its coupled run.
The members' vias now count as partner copper on every layer they span.

Checks:
  1. rp2040's geometry: the P via on the N diagonal is a hard violation;
     precondition -- without the vias the gate passes it (what shipped).
  2. Control: a via pair at its designed spacing beside a coupled run, and a
     blind via off the track's layer, are not refused.
  3. A track grazing a partner via by less than a neck can fix is necked and
     not refused (#318's job, now against vias too).
  4. Two member vias closer than the pair clearance are refused.
  5. Negative control: the via 10 um outside the rule passes, 10 um inside
     is refused.

    python3 tests/test_318_pair_via_on_partner.py
"""
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in ('py_router', 'rust_router', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _d))

from kicad_parser import BoardInfo                                  # noqa: E402
from routing_config import GridRouteConfig                          # noqa: E402
from synth import make_pcb, make_seg, make_via                      # noqa: E402
from diff_pair_routing import _neck_pair_partner_grazes             # noqa: E402

RUN_ALL_TIMEOUT = 60
P, N = 174, 175
CLR = 0.09
LAYERS = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']
fails = []


def check(name, ok, detail=''):
    print(('PASS' if ok else 'FAIL') + f': {name}' + (f' -- {detail}' if detail else ''))
    if not ok:
        fails.append(name)


def cfg():
    return GridRouteConfig(layers=list(LAYERS), clearance=CLR, track_width=0.2,
                           diff_pair_gap=0.25, via_size=0.6, via_drill=0.2)


PCB = make_pcb(board_info=BoardInfo(layers={}, copper_layers=list(LAYERS),
                                    board_bounds=(0, 0, 200, 200)))


def run(p_segs, n_segs, p_vias=(), n_vias=()):
    return _neck_pair_partner_grazes(p_segs, n_segs, cfg(), PCB,
                                     p_vias=list(p_vias), n_vias=list(n_vias))


def rp2040():
    p = [make_seg(87.275, 102.493, 88.475, 103.693, net_id=P),
         make_seg(88.445, 102.7, 88.405, 102.7, net_id=P),
         make_seg(88.405, 102.7, 88.325, 102.493, net_id=P)]
    n = [make_seg(87.725, 102.307, 88.925, 103.507, net_id=N),
         make_seg(87.755, 102.7, 87.955, 102.7, net_id=N),
         make_seg(87.955, 102.7, 87.875, 102.307, net_id=N)]
    pv = [make_via(88.445, 102.7, net_id=P, size=0.6, drill=0.2)]
    nv = [make_via(87.755, 102.7, net_id=N, size=0.6, drill=0.2)]
    return p, n, pv, nv


print("1. rp2040_motorctrl /CAN1 leg 1")
p, n, pv, nv = rp2040()
_n0, hard0 = _neck_pair_partner_grazes(p, n, cfg(), PCB)
check("precondition: tracks alone pass the gate (the short shipped)", not hard0,
      f"{len(hard0)} hard")
p, n, pv, nv = rp2040()
_n1, hard1 = run(p, n, pv, nv)
worst = min((h[2] for h in hard1), default=None)
check("with the vias the pair is refused", bool(hard1) and worst < 0,
      f"{len(hard1)} hard, worst gap {worst}")

print("2. controls")
# A via pair at its designed spacing: centres 1.1 mm apart (0.5 mm of copper
# between 0.6 mm vias), each member's track leaving its own via away from
# the partner.
pv = [make_via(10.0, 10.0, net_id=P)]
nv = [make_via(11.1, 10.0, net_id=N)]
for v in pv + nv:
    v.size, v.drill = 0.6, 0.2
pt = [make_seg(10.0, 10.0, 10.0, 15.0, net_id=P)]
nt = [make_seg(11.1, 10.0, 11.1, 15.0, net_id=N)]
_n, hard = run(pt, nt, pv, nv)
check("a via pair at its spacing with tracks leaving it is not refused", not hard,
      f"{len(hard)} hard")
blind = make_via(11.1, 10.0, net_id=N, size=0.6, drill=0.2, layers=('In1.Cu', 'In2.Cu'),
                 via_type='buried')
pt2 = [make_seg(10.0, 10.0, 12.2, 10.0, net_id=P)]          # F.Cu right over it
_n, hard = run(pt2, [], [], [blind])
check("a buried via is not partner copper on F.Cu", not hard, f"{len(hard)} hard")

print("3. a graze a neck can fix")
# Track edge-to-via-edge gap 0.05 at 0.2 mm wide: necked to 0.1 clears 0.09.
via = make_via(20.0, 20.0, net_id=N, size=0.6, drill=0.2)
t = make_seg(19.0, 20.0 + 0.3 + 0.05 + 0.1, 21.0, 20.0 + 0.3 + 0.05 + 0.1, net_id=P)
necked, hard = run([t], [], [], [via])
check("the track is necked against the partner via and not refused",
      necked == 1 and not hard and t.width < 0.2, f"width {t.width}, hard {len(hard)}")

print("4. member vias too close")
_n, hard = run([], [], [make_via(30.0, 30.0, net_id=P, size=0.6, drill=0.2)],
               [make_via(30.65, 30.0, net_id=N, size=0.6, drill=0.2)])
check("vias 0.05 mm apart (0.09 rule) are refused", bool(hard), f"{len(hard)} hard")

print("5. negative control at the rule")
for slack, want in ((+0.01, False), (-0.01, True)):
    via = make_via(40.0, 40.0, net_id=N, size=0.6, drill=0.2)
    # A 0.0889 mm track at the fab floor cannot neck further: the gate decides.
    y = 40.0 + 0.3 + CLR + slack + 0.0889 / 2
    t = make_seg(39.0, y, 41.0, y, net_id=P, width=0.0889)
    _n, hard = run([t], [], [], [via])
    check(f"{slack * 1000:+.0f} um of slack: {'refused' if want else 'passes'}",
          bool(hard) == want, f"{len(hard)} hard")

if fails:
    print(f"\nFAILED: {fails}")
    sys.exit(1)
print("\nALL PASSED")
