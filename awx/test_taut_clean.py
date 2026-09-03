#!/usr/bin/env python3
"""The user's reproduction (0902): a chord seeded across a thin foreign
track capsule is a stable smooth/push cycle -- relax converges with
points INSIDE the capsule and reports success. Asserts:
  1. relax alone returns a polyline that violates (the defect);
  2. relax_clean flags it, reseeds around the capsule's end, and
     returns a clean path in the 'reseeded' status;
  3. negative control: with no obstacle in the way relax_clean is
     bit-identical to relax ('clean', 0 reseeds);
  4. a disc offender (a pad) is routed around too.
"""
import os
import sys
os.environ['TAUT_RESEED'] = '1'   # the reseed policy under test

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import topo_strings as ts   # noqa: E402
import taut_clean as tc     # noqa: E402


def obstacles(caps=(), discs=()):
    obs = ts.Obstacles()
    for (a, b, r, n) in caps:
        obs.add_cap(a, b, r, n)
    for (x, y, r, n) in discs:
        obs.add_disc(x, y, r, n)
    obs.build()
    return obs


def main():
    src, dst = (0.0, 0.0), (10.0, 0.0)
    # a thin track crossing the chord at x=5, from y=-2 to y=+2
    obs = obstacles(caps=[((5.0, -2.0), (5.0, 2.0), 0.15, 'foreign')])

    pts, it = ts.relax(src, dst, obs)
    bad = tc.violations(pts, obs)
    print(f'relax: {it} iters, {ts.polyline_len(pts):.3f} mm, '
          f'{len(bad)} violating sample(s), deepest '
          f'{max((d for _p, d in bad), default=0):.3f} mm')
    assert bad, 'DEFECT NOT REPRODUCED: relax returned a clean path'

    pts2, it2, status, n_re = tc.relax_clean(src, dst, obs)
    bad2 = tc.violations(pts2, obs)
    print(f'relax_clean: status {status}, {n_re} reseed(s), {it2} iters, '
          f'{ts.polyline_len(pts2):.3f} mm, {len(bad2)} violating')
    assert status == 'reseeded' and not bad2, 'reseed did not clean'
    # the clean path goes AROUND an end of the track: its extreme y
    # reaches past +2 or -2
    ys = [p[1] for p in pts2]
    assert max(ys) > 2.0 or min(ys) < -2.0, 'path did not round the end'

    # negative control: obstacle off the chord -> bit-identical to relax
    obs2 = obstacles(caps=[((5.0, 3.0), (5.0, 6.0), 0.15, 'far')])
    p_a, i_a = ts.relax(src, dst, obs2)
    p_b, i_b, st, nr = tc.relax_clean(src, dst, obs2)
    assert st == 'clean' and nr == 0 and p_a == p_b and i_a == i_b, \
        'clean case is not bit-identical to relax'
    print('negative control: bit-identical, status clean')

    # a disc offender (a pad on the chord)
    obs3 = obstacles(discs=[(5.0, 0.0, 0.4, 'pad')])
    p3, i3, st3, nr3 = tc.relax_clean(src, dst, obs3)
    print(f'disc: status {st3}, {nr3} reseed(s), '
          f'{len(tc.violations(p3, obs3))} violating')
    assert not tc.violations(p3, obs3)
    print('PASS')


if __name__ == '__main__':
    main()
