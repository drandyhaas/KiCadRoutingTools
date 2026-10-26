#!/usr/bin/env python3
"""#1443: a part may stand under another part's body when both heights are
known and it fits there.

The fixture is a module on headers (a footprint from KiCad's library, its
courtyard removed, as the boards that put parts under a module draw it), an
axial resistor standing under it where a human board has one, a DIP-8 and a
two-pin header.

  1. IPC-7351 names carry a height (`heights.ipc_height`);
  2. `Cover.blocked`, on a synthetic raster: room under the body, a cell the
     model is not over, a drawn courtyard, a raised overhang past the
     occupancy (met by the other part's model cell by cell), a flush
     body past it, the clearance measured as the occupancy test measures it,
     and a part reached from above, which admits nothing inside it;
  3. `Heights.pair`: a part something reaches from above never goes under,
     an unknown height leaves the pair to its occupancy, a turn of both parts
     together changes nothing;
  4. with a synthetic measurement in place of kicad-cli's, the seat gate and
     check_assembly's grade agree: a part that fits is legal and graded
     `height_clear`, a part too tall, a connector, a part on the cover's pin
     and a part under a raised overhang too low for it are refused and
     graded; with no heights every one of them is refused as before;
  5. with KiCad installed, the real measurement of the fixture: the resistor
     where the human put it is legal and check_assembly calls the board
     buildable, and on the cover's pin it is refused.

    python3 tests/test_1443_heights.py
"""
import contextlib
import io
import math
import os
import subprocess
import sys

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, os.path.join(REPO, 'py_placer'))
sys.path.insert(0, TESTS)

import numpy as np  # noqa: E402

import run_utils  # noqa: E402
from placement import heights  # noqa: E402

RUN_ALL_TIMEOUT = 600
FIXTURE = os.path.join(TESTS, 'fixtures', '1443', 'module_cover.kicad_pcb')
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def quiet(fn, *a, **k):
    with contextlib.redirect_stdout(io.StringIO()):
        return fn(*a, **k)


def unit():
    # 1. IPC names
    for name, want in (('CAPC1005X55N', 0.55), ('${X}/QFN50P400X400X80-25N.stp', 0.80),
                       ('BGA324C80P18X18_1500X1500X150', 1.50), ('SOIC127P600X175-8N', 1.75),
                       ('C_0402_1005Metric', None), ('PinHeader_1x10', None)):
        got = heights.ipc_height(name)
        check(f'IPC height of {name}', got == want, f'{got}')

    # 2. Cover.blocked on a 10 x 10 mm raster, cells of 0.25 mm
    c = heights.CELL_MM
    n = int(10 / c)
    lo = np.full((n + 12, n), np.inf)          # 3 mm past the occupancy in +x
    lo[:n, :] = 5.0                            # the body 5 mm over its occupancy
    lo[n:, : n // 2] = 2.0                     # a raised overhang past it
    lo[n:, n // 2:] = 0.2                      # a flush body past it
    lo[8:12, 8:12] = np.inf                    # a hole in the model
    court = np.zeros(lo.shape, bool)
    court[30:34, 30:34] = True                 # a drawn courtyard
    cov = heights.Cover('U', 0.0, 0.0, lo, court)
    occ = (0.0, 0.0, 10.0, 10.0)
    b = cov.blocked
    check('a part with room under the body fits', b(occ, (4, 4, 6, 5), 1.0, 0.1) == 0.0)
    check('...and one within the margin of it does not',
          b(occ, (4, 4, 6, 5), 5.0 - heights.MARGIN_MM + 0.01, 0.1) > 0.0)
    low = heights.Cover('L', 0.0, 0.0, np.where(np.isfinite(lo) & (lo == 5.0), 1.2, lo), court)
    check('an underside under the floor (a model\'s offset, not room) admits nothing',
          low.blocked(occ, (4, 4, 6, 5), 0.2, 0.1) > 0.0)
    check('a cell the model is not over is unknown, so refused',
          b(occ, (2.2, 2.2, 2.8, 2.8), 0.3, 0.1) > 0.0)
    check('a cell of a drawn courtyard is refused', b(occ, (7.6, 7.6, 8.4, 8.4), 0.3, 0.1) > 0.0)
    def body(box, h):
        xs = np.arange(box[0], box[2], c) + c / 2
        ys = np.arange(box[1], box[3], c) + c / 2
        pts = np.array([[x, y] for x in xs for y in ys])
        return pts, np.full(len(pts), h)
    check('a raised overhang past the occupancy refuses a part too tall for it',
          b(occ, (11, 0.5, 12, 1.5), 2.1, 0.1, body=body((11, 0.5, 12, 1.5), 2.1)) > 0.0)
    check('...and admits one under it, however close (the bodies do not meet)',
          b(occ, (11, 0.5, 12, 1.5), 1.95, 0.1, body=body((11, 0.5, 12, 1.5), 1.95)) == 0.0)
    check('...and a part with no model is judged on its occupancy alone',
          b(occ, (11, 0.5, 12, 1.5), 2.1, 0.1) == 0.0)
    check('a flush body past the occupancy is the occupancy test\'s, not this one\'s',
          b(occ, (11, 8, 12, 9), 3.0, 0.1, body=body((11, 8, 12, 9), 3.0)) == 0.0)
    # a box diagonal to the occupancy corner, inside the grown box but 0.14 mm
    # away in Euclidean distance, clear at a 0.12 clearance, as rect_gap says
    check('the clearance to the occupancy is Euclidean, as the occupancy test\'s',
          b(occ, (-1.0, 10.1, -0.1, 11.0), math.inf, 0.12) == 0.0)
    check('...and a box within it is refused', b(occ, (-1.0, 9.95, -0.1, 11.0), math.inf, 0.12) > 0.0)
    # an L-shaped body: tall (2.6 mm) at x 11.0-11.5, low (0.4 mm) at 11.5-12.5
    pts = np.array([[x + 0.125, y + 0.125] for x in np.arange(11.0, 12.5, 0.25)
                    for y in np.arange(0.5, 1.5, 0.25)])
    hv = np.where(pts[:, 0] < 11.5, 2.6, 0.4)
    check('past the occupancy the bodies meet cell by cell: the tall arm is refused',
          b(occ, (10.9, 0.4, 12.6, 1.6), 0.0, 0.1, body=(pts, hv)) > 0.0)
    check('...and the same body with its tall arm past the overhang fits',
          b(occ, (10.9, 0.4, 12.6, 1.6), 0.0, 0.1, body=(pts + [2.0, 0.0], hv)) == 0.0
          and b(occ, (10.9, 0.4, 12.6, 1.6), 0.0, 0.1, body=(pts, np.full(len(hv), 0.4))) == 0.0)
    solid = heights.Cover('J', 0.0, 0.0, lo, court, solid=True)
    check('a part reached from above admits nothing inside its occupancy',
          solid.blocked(occ, (4, 4, 6, 5), 1.0, 0.1) > 0.0)
    check('...and its raised body past the occupancy still binds',
          solid.blocked(occ, (11, 0.5, 12, 1.5), 2.1, 0.1, body=body((11, 0.5, 12, 1.5), 2.1)) > 0.0
          and solid.blocked(occ, (11, 0.5, 12, 1.5), 1.4, 0.1,
                            body=body((11, 0.5, 12, 1.5), 1.4)) == 0.0)
    check('without the occupancy rule only the raised body binds',
          b(occ, (2.2, 2.2, 2.8, 2.8), 0.3, 0.1, inside=False) == 0.0
          and b(occ, (11, 0.5, 12, 1.5), 2.1, 0.1, inside=False,
                body=body((11, 0.5, 12, 1.5), 2.1)) > 0.0)

    # 3. Heights.pair
    small = (-0.5, -0.5, 0.5, 0.5)
    h = heights.Heights(top={'U': (6.0, 'model'), 'C': (0.6, 'model'), 'J': (1.0, 'model')},
                        covers={'U': cov}, reached={'J'})
    at = (100.0, 50.0, 0.0)
    check('Heights.pair: a part with room fits', h.pair('U', at, occ, 'F', 'C', (105, 54.5, 0), small,
                                                         'F', 0.1) == 0.0)
    check('Heights.pair: a part reached from above never goes under, however low',
          (h.pair('U', at, occ, 'F', 'J', (105, 54.5, 0), small, 'F', 0.1) or 0) > 0.0)
    check('Heights.pair: an unknown height leaves the pair to its occupancy',
          h.pair('U', at, occ, 'F', 'X', (105, 54.5, 0), small, 'F', 0.1) is None)
    check('Heights.pair: parts on two faces are not its to judge',
          h.pair('U', at, occ, 'F', 'C', (105, 54.5, 0), small, 'B', 0.1) is None)
    for rot in (90.0, 180.0, 270.0):
        a = math.radians(-rot)
        dx, dy = 5.0, 4.5
        cp = (100 + dx * math.cos(a) - dy * math.sin(a), 50 + dx * math.sin(a) + dy * math.cos(a), rot)
        fits = h.pair('U', (100, 50, rot), occ, 'F', 'C', cp, small, 'F', 0.1) == 0.0
        dx, dy = 11.5, 1.0
        op = (100 + dx * math.cos(a) - dy * math.sin(a), 50 + dx * math.sin(a) + dy * math.cos(a), rot)
        cb = np.array([[x, y] for x in (-0.375, -0.125, 0.125, 0.375) for y in (-0.375, -0.125, 0.125, 0.375)])
        h2 = heights.Heights(top={'U': (6.0, 'model'), 'C': (2.1, 'model')}, covers={'U': cov},
                             bodies={'C': (cb, np.full(len(cb), 2.1))})
        under = (h2.pair('U', (100, 50, rot), occ, 'F', 'C', op, small, 'F', 0.1) or 0) > 0.0
        check(f'turned {rot:.0f} degrees together: same answers', fits and under)


def synthetic(pcb, board, under=9.5, over=None, tops=None):
    """A measurement of the fixture with the module's body `under` mm over its
    occupancy, and `over` mm over a 3 mm band past its +x side."""
    from placement.body import board_bodies
    occ = board_bodies(pcb, board)['U1'].occupancy_local
    c = heights.CELL_MM
    x0 = math.floor(occ[0] / c) * c
    y0 = math.floor(occ[1] / c) * c
    nx = int(math.ceil((occ[2] + 3.0 - x0) / c)) + 1
    ny = int(math.ceil((occ[3] - y0) / c)) + 1
    lo = np.full((nx, ny), np.inf)
    cx = x0 + (np.arange(nx) + 0.5) * c
    cy = y0 + (np.arange(ny) + 0.5) * c
    inside = ((cx >= occ[0]) & (cx <= occ[2]))[:, None] & ((cy >= occ[1]) & (cy <= occ[3]))[None, :]
    lo[inside] = under
    if over is not None:
        band = ((cx > occ[2]) & (cx <= occ[2] + 3.0))[:, None] & np.ones(ny, bool)[None, :]
        lo[band] = over
    cm = np.where(np.isfinite(lo), np.round(lo * 100), 65535).astype('<u2')
    tops = tops or {'U1': 14.6, 'R10': 3.2, 'U3': 3.68, 'J1': 8.54}
    bodies = {}
    for r in ('R10', 'U3', 'J1'):     # each part's body: its occupancy box, its height
        b = board_bodies(pcb, board)[r].occupancy_local
        bx0, by0 = math.floor(b[0] / c) * c, math.floor(b[1] / c) * c
        bnx, bny = int(math.ceil((b[2] - bx0) / c)), int(math.ceil((b[3] - by0) / c))
        bodies[r] = [bx0, by0, bnx, bny,
                     np.full((bnx, bny), round(tops[r] * 100), '<u2').tobytes().hex()]
    raw = {'top': tops, 'covers': {'U1': [x0, y0, nx, ny, cm.tobytes().hex()]}, 'bodies': bodies}
    return heights._load(raw, pcb, board, {}), occ


def wiring():
    from kicad_parser import parse_kicad_pcb
    from placement.legality import HEIGHT_WAIVER, grade_body_overlap
    from placement.quench import QuenchState
    run_utils.evidence(FIXTURE, 'the #1443 fixture')
    pcb = quiet(parse_kicad_pcb, FIXTURE)
    real = heights.for_board
    try:
        def state(h):
            heights.for_board = lambda *_a, **_k: h
            return quiet(QuenchState, pcb, FIXTURE, 0.1, 0.25)
        h, occ = synthetic(pcb, FIXTURE)
        q = state(h)
        u1 = pcb.footprints['U1']
        pads = [p for p in u1.pads]
        mid = (sum(p.global_x for p in pads) / len(pads), sum(p.global_y for p in pads) / len(pads))
        r10 = q.parts['R10']
        human = (r10.x, r10.y, r10.rot)
        check('the cover is U1, the connector J1 is reached from above',
              q.covers == {'U1'} and h.reached == {'J1'}, f'{q.covers}, {set(h.reached)}')
        v = q.candidate_veto('R10', *human)
        check('the resistor under the module where the human put it is legal', v is None, str(v))
        v = q.candidate_veto('U3', mid[0], mid[1], 0.0)
        check('a DIP-8 between the module\'s headers is legal', v is None, str(v))
        v = q.candidate_veto('J1', mid[0], mid[1], 0.0)
        check('a pin header under the module is refused (it is reached from above)',
              v is not None and v[1] == 'U1', str(v))
        p0 = pads[0]
        v = q.candidate_veto('R10', p0.global_x, p0.global_y, 90.0)
        check('the resistor on the module\'s pin is refused', v is not None and v[1] == 'U1', str(v))
        check('...and the violation the anneal\'s escape reads is not zero',
              q.violation_parts('R10', p0.global_x, p0.global_y, 90.0)[1] > 0.0)
        g = quiet(grade_body_overlap, pcb, 0.1, pcb_file=FIXTURE)
        hc = sorted({(p.a, p.b, p.kind) for p in g['pairs'] if p.waiver == HEIGHT_WAIVER})
        check('check_assembly\'s grade clears R10 under U1, every channel',
              ('R10', 'U1', 'courtyard') in hc or ('R10', 'U1', 'fab') in hc, str(hc))
        check('...and nothing about the pair gates',
              not [p for p in g['containment_blocking_pairs'] + g['courtyard_blocking_pairs']
                   if {p.a, p.b} == {'R10', 'U1'}])

        h, _ = synthetic(pcb, FIXTURE, tops={'U1': 14.6, 'R10': 3.2, 'U3': 9.2, 'J1': 8.54})
        q = state(h)
        v = q.candidate_veto('U3', mid[0], mid[1], 0.0)
        check('a part too tall for the room under the module is refused, as before',
              v is not None and v[1] == 'U1', str(v))

        # a raised overhang 1.0 mm high, 3 mm past the occupancy's +x side
        h, occ = synthetic(pcb, FIXTURE, over=1.0)
        q = state(h)
        from placement.heights import to_board
        ob = to_board((occ[2] + 0.5, occ[1] + 10, occ[2] + 2.5, occ[1] + 12), (u1.x, u1.y, u1.rotation))
        spot = ((ob[0] + ob[2]) / 2, (ob[1] + ob[3]) / 2)
        far = q.parts['U3'].rect(spot[0], spot[1], 0.0)
        v = q.candidate_veto('U3', spot[0], spot[1], 0.0)
        check('a part under a raised overhang too low for it is refused, past the occupancy',
              v is not None and v[1] == 'U1', f'{v}, U3 at {far}')

        heights.for_board = lambda *_a, **_k: heights.EMPTY
        q = quiet(QuenchState, pcb, FIXTURE, 0.1, 0.25)
        v = q.candidate_veto('R10', *human)
        check('control: with no heights the resistor under the module is refused, as before',
              v is not None and v[1] == 'U1', str(v))
        g = quiet(grade_body_overlap, pcb, 0.1, pcb_file=FIXTURE)
        check('control: ...and graded as the body it is inside',
              any({p.a, p.b} == {'R10', 'U1'} for p in g['containment_blocking_pairs']))
    finally:
        heights.for_board = real


def kicad():
    from kicad_parser import parse_kicad_pcb
    from model_export import resolve_cli
    from placement.quench import QuenchState
    cli, why = resolve_cli()
    if not cli:
        return f'no kicad-cli ({why})'
    pcb = quiet(parse_kicad_pcb, FIXTURE)
    h = heights.for_board(pcb, FIXTURE)
    if h.height('R10') is None:
        return "kicad-cli resolved no model of the fixture's parts: " + h.note
    check("the real measurement: U1 has room under it, R10's height is its model's",
          'U1' in h.covers and abs(h.height('R10') - 3.2) < 0.05, f'{h.note}; R10 {h.height("R10")}')
    q = quiet(QuenchState, pcb, FIXTURE, 0.1, 0.25)
    r10 = q.parts['R10']
    v = q.candidate_veto('R10', r10.x, r10.y, r10.rot)
    check('...the resistor where the human put it is legal', v is None, str(v))
    p0 = pcb.footprints['U1'].pads[0]
    v = q.candidate_veto('R10', p0.global_x, p0.global_y, 90.0)
    check('...and on the module\'s pin it is refused', v is not None and v[1] == 'U1', str(v))
    r = subprocess.run([sys.executable, '-X', 'utf8', 'py_tools/check_assembly.py', FIXTURE],
                       cwd=REPO, capture_output=True, text=True, encoding='utf-8', errors='replace')
    check('...check_assembly calls the board buildable, naming the heights it read',
          r.returncode == 0 and 'VERDICT: buildable' in r.stdout and 'heights (#1443)' in r.stdout,
          f'rc {r.returncode}: ' + ' | '.join(ln for ln in r.stdout.splitlines()
                                                 if 'VERDICT' in ln or 'heights' in ln))
    return None


def main():
    print('unit:')
    unit()
    print('wiring (a synthetic measurement):')
    wiring()
    print('kicad-cli:')
    skipped = kicad()
    if skipped:
        print(f'  [skip] {skipped}')
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
