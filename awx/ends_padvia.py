"""ends_padvia.py OUT LANE -- a BERTH whose surface layer cannot hold (the solve with no plan, its rules core naming
the lane's 'berthlay') at a ROW PART (escape_moves.row_part: a QFN, an SO): the via IN the lane's pad, at the first
place along the pad's length a via clears every other net's copper on every copper layer, the pad's surface stub taken
off, and the berth that via -- its lane's layer the plan's (whole_solve.via_end: an end standing on a via of its net).
A BACKUP: a row part's menu has no via in its pads -- the pin's way is its stub out of the row, and a via in a pad must
be filled and capped -- so this is laid only where the solve found no plan with the berths it had (whole_route's
repair_berths). On the round's fanout bench (BENCH0, else BENCH) and its plan sidecar; writes OUT and its sidecar.
Exit 0 laid; 2 no place along the pad clears; 3 not a row part's berth."""
import json
import math
import os
import re
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.normpath(os.path.join(HERE, '..', 'py_router')))
import awx_settings  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402
from routing_utils import point_to_pad_rect_dist  # noqa: E402
import braid as te  # noqa: E402
import escape_moves as em  # noqa: E402

MARGIN = 0.05                # past the via's clearance, as ends_reescape
STEP = 0.025                 # along the pad (mm)
NEAR = 2.0                   # the stub's copper taken off: the lane's segments this near the pad (mm)

args = [a for a in sys.argv[1:] if not a.startswith('--')]
out, lane = args[0], args[1]
bench = awx_settings.get('BENCH0') or awx_settings.req('BENCH')
dests = [r for r in (awx_settings.get('DEST') or '').split('+') if r]
pcb = parse_kicad_pcb(bench)
side_f = os.path.splitext(bench)[0] + '.plan.json'
side = json.load(open(side_f))
if lane not in (side.get('ends') or {}):
    print(f'ends_padvia: {lane} has no ends on the sidecar')
    sys.exit(3)
nid = next((i for i, n in pcb.nets.items() if n.name.split('/')[-1] == lane.split('/')[-1]), None)
bx, by = side['ends'][lane][1]
pads = [(r, p) for r in dests if r in pcb.footprints for p in pcb.footprints[r].pads if p.net_id == nid]
if nid is None or not pads:
    print(f'ends_padvia: {lane} has no pad on {"+".join(dests)}')
    sys.exit(3)
ref, pad = min(pads, key=lambda rp: math.hypot(rp[1].global_x - bx, rp[1].global_y - by))
if not em.row_part(pcb.footprints[ref]):
    print(f'ends_padvia: {ref} is no row part -- its menu has its own vias in pads')
    sys.exit(3)
need = te.VIA_SIZE / 2 + te.CLEAR + MARGIN
if te.VIA_SIZE > min(pad.size_x, pad.size_y) + 2 * te.CLEAR:
    print(f'ends_padvia: a {te.VIA_SIZE} via does not fit {ref}.{pad.pad_number} ({pad.size_x:.2f} x {pad.size_y:.2f})')
    sys.exit(2)


def seg_d(p, a, b):
    ax, ay, bx_, by_ = a[0], a[1], b[0], b[1]
    L2 = (bx_ - ax) ** 2 + (by_ - ay) ** 2
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - ax) * (bx_ - ax) + (p[1] - ay) * (by_ - ay)) / L2))
    return math.hypot(ax + t * (bx_ - ax) - p[0], ay + t * (by_ - ay) - p[1])


def clear_at(q):
    """the via's clearance at q to every other net's copper on every layer (tracks, vias, pads as their rectangles)"""
    best = 9e9
    for s in pcb.segments:
        if s.net_id != nid and not getattr(s, 'graphic', False):
            best = min(best, seg_d(q, (s.start_x, s.start_y), (s.end_x, s.end_y)) - s.width / 2)
    for v in pcb.vias:
        if v.net_id != nid:
            best = min(best, math.hypot(v.x - q[0], v.y - q[1]) - v.size / 2)
    for fp in pcb.footprints.values():
        for p in fp.pads:
            if p.net_id != nid and any(str(L).endswith('.Cu') for L in p.layers) and \
                    abs(p.global_x - q[0]) < 3 and abs(p.global_y - q[1]) < 3:
                best = min(best, point_to_pad_rect_dist(q[0], q[1], p))
    return best


# along the pad's long axis, nearest its middle first, within the pad less the via's own reach
ax = (1.0, 0.0) if pad.size_x >= pad.size_y else (0.0, 1.0)
half = max(pad.size_x, pad.size_y) / 2 - te.VIA_SIZE / 2
offs = sorted({round(k * STEP, 4) * sg for k in range(int(half / STEP) + 1) for sg in (1, -1)}, key=abs)
site, room = None, None
import copper_gates  # noqa: E402
_gates = copper_gates.Gates(pcb)
for o in offs:
    q = (pad.global_x + ax[0] * o, pad.global_y + ax[1] * o)
    if not _gates.via_ok(q, te.VIA_SIZE, nid, te.CLEAR):
        continue                 # (off the board's edge clearance, in a keep-out, on foreign filled copper)
    c = clear_at(q)
    if c >= need:
        site, room = q, c
        break
if site is None:
    print(f'ends_padvia: no place along {ref}.{pad.pad_number} clears a via (needs {need:.3f})')
    sys.exit(2)
# the pad's surface stub off: the lane's copper CONNECTED to the pad on its layer, end to end from it, no farther than
# NEAR (the berth's stub -- the lane is not laid on the bench; the source's stub is not joined to it)
own = [s for s in pcb.segments if s.net_id == nid and s.uuid and not getattr(s, 'graphic', False)]


def _on_pad(x, y):
    return point_to_pad_rect_dist(x, y, pad) <= 1e-3


reach_ = {(round(x, 4), round(y, 4)) for s in own for (x, y) in ((s.start_x, s.start_y), (s.end_x, s.end_y))
          if _on_pad(x, y)}
gone_s, grew = set(), True
while grew:
    grew = False
    for s in own:
        a_, b_ = (round(s.start_x, 4), round(s.start_y, 4)), (round(s.end_x, 4), round(s.end_y, 4))
        if s.uuid in gone_s or not (a_ in reach_ or b_ in reach_):
            continue
        if max(math.hypot(s.start_x - pad.global_x, s.start_y - pad.global_y),
               math.hypot(s.end_x - pad.global_x, s.end_y - pad.global_y)) > NEAR:
            continue
        gone_s.add(s.uuid)
        reach_ |= {a_, b_}
        grew = True
gone = sorted(gone_s)
txt = open(bench, encoding='utf-8').read()
for u in gone:
    txt = re.sub(r'\n\s*\(segment\b(?:(?!\n\s*\((?:segment|via|arc)\b).)*?\(uuid\s+"' + re.escape(u) + r'"\)\s*\)',
                 '', txt, count=1, flags=re.S)
tmp = out + '.tmp.kicad_pcb'
open(tmp, 'w', encoding='utf-8').write(txt)
from kicad_writer import add_tracks_and_vias_to_pcb  # noqa: E402
add_tracks_and_vias_to_pcb(tmp, out, [], [{'x': site[0], 'y': site[1], 'size': te.VIA_SIZE, 'drill': te.VIA_DRILL,
                                          'layers': ['F.Cu', 'B.Cu'], 'net_id': nid}],
                           net_id_to_name={i: n.name for i, n in pcb.nets.items()})
os.remove(tmp)
chk = parse_kicad_pcb(out)
left = [s for s in chk.segments if s.net_id == nid and s.uuid in set(gone)]
if left:
    print(f'ends_padvia: {len(left)} of the stub\'s {len(gone)} segment(s) not taken off -- refused')
    sys.exit(2)
side['ends'][lane][1] = [round(site[0], 4), round(site[1], 4)]
json.dump(side, open(os.path.splitext(out)[0] + '.plan.json', 'w'), indent=1)
for ext in ('.kicad_pro', '.kicad_dru'):
    s_ = os.path.splitext(bench)[0] + ext
    if os.path.isfile(s_) and not os.path.isfile(os.path.splitext(out)[0] + ext):
        shutil.copy(s_, os.path.splitext(out)[0] + ext)
print(f'ends_padvia: {lane} berthed by a via in {ref}.{pad.pad_number} at ({site[0]:.3f}, {site[1]:.3f}), '
      f'{math.hypot(site[0] - pad.global_x, site[1] - pad.global_y):.3f} mm off its middle (clear {room - need:+.3f}); '
      f'its stub\'s {len(gone)} segment(s) taken off')
