#!/usr/bin/env python3
"""A plane ball's drop takes a finer rung's via where the rung's does not fit (awx/joint_escape.py: _drops, _sizes).

  python3 tests/test_622_drop_fine_via.py

The joint escape drops a plane ball by a via in a gap beside it (or off the array's edge, or in its pad), held off the
bus's laid lanes by the bar the whole route's audit holds a lane to (_lanes_clear). Between two of the zynq DDR's U2
berths a 0.45 mm via missed that bar by 0.032 mm, a 0.30 mm one cleared it, and three plane balls had no other way
down. On kicad_files/ulx3s.kicad_pcb U1 (four copper layers: the fab ladder's vias 0.45, 0.30 and 0.25), at the
bench's rules (via 0.45/0.3), a GND ball's drops with a laid lane passing one of its gap sites at a chosen distance:

1. LIVENESS: with no lane the site takes the rung's via, and the ladder has finer vias than the rung's (sz['drop_vias'],
   largest first); else the checks below test nothing (BROKEN TEST);
2. a lane the rung's via misses but the next rung's clears: the site takes the next rung's via, at that rung's drill;
3. a lane only the finest rung's via clears: it takes that one;
4. a lane no via of the ladder clears: the site is not offered at all.
"""
import contextlib
import io
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
BOARD = os.path.join(ROOT, 'kicad_files', 'ulx3s.kicad_pcb')
if not os.path.exists(BOARD):
    print(f'SKIP: board not present: {BOARD}')
    sys.exit(77)
with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
    import route_bus as rb  # noqa: E402
    import rules as _rules  # noqa: E402
    _rr = rb.resolve_rules(BOARD, clearance=0.09, track_width=0.15, fanout_track_width=0.12, via_size=0.45,
                           via_drill=0.3, log=lambda *a: None)
    _rules.install(_rr)
    os.environ[_rules.SETTING] = _rules.as_setting(_rr)
    import braid as te  # noqa: E402
    import escape_moves as em  # noqa: E402
    import joint_escape as je  # noqa: E402
    from kicad_parser import parse_kicad_pcb  # noqa: E402
    from list_nets import escalation_rungs  # noqa: E402

BAD = []


def check(ok, what):
    print(('ok    ' if ok else 'FAIL  ') + what)
    if not ok:
        BAD.append(what)


with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
    pcb = parse_kicad_pcb(BOARD)
foot = pcb.footprints['U1']
grid = em.grid_of(foot)
sz = je._sizes(pcb, foot)
cache = {}


def obs(nid, layer, via=False):
    key = (nid, layer, via)
    if key not in cache:
        cache[key] = te.build_obstacles(pcb, nid, {nid}, layer, margin=sz['cl'] + sz['tw'] / 2)
    return cache[key]


def drops(p, rays=()):
    with contextlib.redirect_stdout(io.StringIO()):
        return je._drops(pcb, grid, p, obs, sz, foot, list(rays), None)


bar = te.TRACK / 2 + te.CLEAR + te.GRID / 2           # (_lanes_clear's)
ladder = sorted({(f['via_diameter'] / 2.0, f['via_drill'] / 2.0)
                 for f in escalation_rungs(len(pcb.board_info.copper_layers))
                 if f['via_diameter'] < te.VIA_SIZE - 1e-9}, reverse=True)
# a GND ball with a gap drop at the rung's via and no lane
ball = site = None
for p in sorted(foot.pads, key=lambda q: q.pad_number):
    if (p.net_name or '').split('/')[-1] != 'GND':
        continue
    gap = [d for d in drops(p) if not d.inpad and abs(d.r - sz['vr']) < 1e-9]
    if gap:
        ball, site = p, gap[0].site
        break

# 1. liveness
if ball is None or len(ladder) < 2:
    print(f'BROKEN TEST: no GND ball of U1 with a gap drop at the rung\'s via ({ball}), or the fab ladder below the '
          f'rung is not two vias ({ladder})')
    sys.exit(2)
check(True, f'liveness: U1 {ball.pad_number} drops at {tuple(round(c, 3) for c in site)} with the rung\'s via '
            f'r {sz["vr"]}; the ladder below it {ladder}')
check(sz.get('drop_vias') == ladder, f'the drops\' finer vias are the ladder\'s below the rung, largest first '
                                     f'({sz.get("drop_vias")})')


def at_site(D):
    """the drop at `site` with a lane passing it D mm away (centre to centre), or None"""
    ray = ((site[0] - 1.0, site[1] + D), (site[0] + 1.0, site[1] + D))
    return next((d for d in drops(ball, [ray]) if abs(d.site[0] - site[0]) < 1e-9
                 and abs(d.site[1] - site[1]) < 1e-9), None)


(r1, dr1), (r2, dr2) = ladder[0], ladder[1]
# 2. the rung's via misses the bar, the next rung's clears it
d = at_site(bar + (sz['vr'] + r1) / 2)
check(d is not None and abs(d.r - r1) < 1e-9 and abs(d.dr - dr1) < 1e-9,
      f'a lane the rung\'s via misses and the next rung\'s clears: the site takes that via ({d})')
# 3. only the finest clears it
d = at_site(bar + (r1 + r2) / 2)
check(d is not None and abs(d.r - r2) < 1e-9 and abs(d.dr - dr2) < 1e-9,
      f'a lane only the finest rung\'s via clears: the site takes that one ({d})')
# 4. none does
d = at_site(bar + r2 / 2)
check(d is None, f'a lane no via of the ladder clears: the site is not offered ({d})')

print(f'\n{"PASS" if not BAD else "FAIL"}: {len(BAD)} failure(s)' + (': ' + '; '.join(BAD) if BAD else ''))
sys.exit(1 if BAD else 0)
