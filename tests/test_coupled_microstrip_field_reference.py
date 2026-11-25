"""differential_microstrip_z0 against a 2-D field solve.

Reference values: odd-mode capacitance of the edge-coupled pair from a
finite-volume Laplace solve on a graded grid (walls 30 h away, odd symmetry
plane, first-order Richardson extrapolation of the 1/32 and 1/64 cell
sizes). The same solver reproduces the Hammerstad-Jensen single line within
0.3% (w 0.1 and 0.2 mm on 0.0994 mm, er 4.1). No solder mask.

The IPC-2141 coupling factor this replaces, Z0 * (1 - 0.48 * exp(-0.96 s/h)),
was off by up to 25% on these rows (a tight gap on a thick core).

Invariants:
1. Each row within 3.5% of the field solve.
2. Zero thickness: the Kirschning-Jansen core within 0.5%.
3. Zdiff == 2 * Zodd, and the width solver round-trips its target.

    python3 tests/test_coupled_microstrip_field_reference.py
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                'py_router'))
import impedance as I  # noqa: E402

failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


# (w, s, h, t, er) -> field-solved Zdiff, ohms
ROWS = [
    ((0.15, 0.10, 0.0994, 0.035, 4.1), 87.74),
    ((0.15, 0.15, 0.0994, 0.035, 4.1), 94.66),
    ((0.15, 0.30, 0.0994, 0.035, 4.1), 102.71),
    ((0.20, 0.20, 0.20, 0.035, 4.3), 109.78),
    ((0.30, 0.50, 0.20, 0.035, 4.3), 103.88),
    ((0.407, 0.25, 0.20, 0.035, 4.5), 80.17),
    ((0.707, 0.325, 1.51, 0.035, 4.5), 107.17),
    ((1.133, 0.325, 1.51, 0.035, 4.5), 92.95),
    ((0.7074, 0.15, 1.51, 0.035, 4.5), 86.35),
    ((1.133, 0.15, 1.51, 0.035, 4.5), 76.47),
]
ZERO_T = ((0.15, 0.15, 0.0994, 0.0001, 4.1), 107.71)


def main():
    print('1. against the field solve')
    worst = 0.0
    for (w, s, h, t, er), ref in ROWS:
        z = I.differential_microstrip_z0(w, s, h, t, er)[0]
        err = z / ref - 1
        worst = max(worst, abs(err))
        check(f'w {w} s {s} h {h}: {z:.2f} vs {ref:.2f} ohm', abs(err) < 0.035,
              f'{100 * err:+.1f}%')
    print(f'  worst {100 * worst:.1f}%')
    print('2. zero thickness')
    (w, s, h, t, er), ref = ZERO_T
    z = I.differential_microstrip_z0(w, s, h, t, er)[0]
    check(f'{z:.2f} vs {ref:.2f} ohm', abs(z / ref - 1) < 0.005, f'{100 * (z / ref - 1):+.2f}%')
    print('3. consistency')
    zd, zo = I.differential_microstrip_z0(0.15, 0.15, 0.0994, 0.035, 4.1)
    check('Zdiff == 2 * Zodd', abs(zd - 2 * zo) < 1e-9)
    w = I.differential_microstrip_width_for_z0(100.0, 0.15, 0.0994, 0.035, 4.1)
    got = I.differential_microstrip_z0(w, 0.15, 0.0994, 0.035, 4.1)[0]
    check('width solver round-trips 100 ohm', abs(got - 100.0) < 0.5, f'{w:.4f} mm -> {got:.2f}')
    check('a zero gap or width gives (0, 0)',
          I.differential_microstrip_z0(0.15, 0.0, 0.1, 0.035, 4.1) == (0.0, 0.0)
          and I.differential_microstrip_z0(0.0, 0.15, 0.1, 0.035, 4.1) == (0.0, 0.0))
    if failures:
        print(f'FAIL: {len(failures)}: ' + '; '.join(failures))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
