#!/usr/bin/env python3
"""`--length-match-group auto` keeps a strobe in its OWN channel's lane when
the channel letter sits before the polarity (`DQS0_A_P`).

    python3 tests/test_ddr_auto_group_channel_strobes.py

The DDR grouper (#1247) reads a channel as the LAST token of a name's tail,
so a two-channel LPDDR4 interface named `DQ0_A`, `DM0_A`, `DQS0_A_P`,
`DQS0_A_N`, ... (the channel after the byte number, the polarity last) read
the strobe's channel as `P` / `N`, i.e. none: DQS0_A_P/N and DQS0_B_P/N were
put in one group of their own, apart from both lanes they strobe, while the
data lanes split by channel correctly. The clock `CK_A_P` lost its channel
the same way, so the command/address bus of two channels with a clock each
was read as sharing one clock and stayed one group.

Pinned here, on synthetic names in three prefix spellings (none, a schematic
sheet `/DDR/`, a part prefix `LP4_`):

  1. non-vacuity: the strobe reads as a DQS of byte 0 with tail [A, P], and
     the DATA lanes split by channel, so a failure below is the strobe's;
  2. each lane of each channel is its 8 DQ, its DM and its own DQS pair (11),
     and there are exactly four lanes;
  3. each channel's CK_x_P/N clocks its own command/address group, as the
     JEDEC spelling `CK_t_A` already does (test_1247);
  4. the `_A_P` spelling partitions exactly as the `_t_A` spelling does;
  5. a polarity with no channel (`DDR_DQS0_P`) still makes no channel.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, 'py_router'))

from length_matching import _ddr_read_name, classify_ddr_nets  # noqa: E402

FAILS = []


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def lpddr4(pre, strobe='{pre}DQS{n}_{ch}_{p}', clock='{pre}CK_{ch}_{p}',
           pol='PN'):
    """Two x16 channels A and B: data, masks, strobes, command, clock."""
    names = []
    for ch in 'AB':
        names += [f'{pre}DQ{i}_{ch}' for i in range(16)]
        names += [f'{pre}DM{n}_{ch}' for n in (0, 1)]
        names += [strobe.format(pre=pre, n=n, ch=ch, p=p)
                  for n in (0, 1) for p in pol]
        names += [f'{pre}CA{i}_{ch}' for i in range(6)]
        names += [f'{pre}CS0_{ch}', f'{pre}CKE0_{ch}']
        names += [clock.format(pre=pre, ch=ch, p=p) for p in pol]
    return names


def grouped(c):
    return {k: set(v) for k, v in c['groups'].items()}


def main():
    for pre in ('', '/DDR/', 'LP4_'):
        print(f"prefix {pre or '(none)'!r}")
        names = lpddr4(pre)
        label = pre or '(no prefix)'
        c = classify_ddr_nets(names)
        g = grouped(c)

        r = _ddr_read_name(f'{pre}DQS0_A_P')
        ok(r is not None and r['kind'] == 'DQS' and r['n'] == 0
           and r['tail'] == ['A', 'P'],
           f"non-vacuity: {pre}DQS0_A_P reads as DQS byte 0, tail [A, P] "
           f"({r and (r['kind'], r['n'], r['tail'])})")
        data_a = {f'{pre}DQ{i}_A' for i in range(8)} | {f'{pre}DM0_A'}
        ok(any(data_a <= v and not any('_B' in n for n in v)
               for v in g.values()),
           "non-vacuity: the data lanes split by channel (channel A lane 0 "
           "holds only channel A data)")
        ok(c['channels'] == {pre: ['A', 'B']},
           f"the report names channels A and B ({c['channels']})")

        lanes = {k for k in g if k.startswith('byte_lane_')}
        want_lanes = {f'byte_lane_{n} @ {label} channel {ch}'
                      for n in (0, 1) for ch in 'AB'}
        ok(lanes == want_lanes, f"exactly four lanes, one per byte per "
           f"channel ({sorted(lanes)})")
        for n in (0, 1):
            for ch in 'AB':
                key = f'byte_lane_{n} @ {label} channel {ch}'
                want = ({f'{pre}DQ{i}_{ch}' for i in range(8 * n, 8 * n + 8)}
                        | {f'{pre}DM{n}_{ch}', f'{pre}DQS{n}_{ch}_P',
                           f'{pre}DQS{n}_{ch}_N'})
                got = g.get(key, set())
                ok(got == want and len(want) == 11,
                   f"{key} = DQ{8 * n}-{8 * n + 7}_{ch} + DM{n}_{ch} + "
                   f"DQS{n}_{ch}_P/N (diff {sorted(got ^ want)})")

        for ch in 'AB':
            key = f'command_address @ {label} channel {ch}'
            want = ({f'{pre}CA{i}_{ch}' for i in range(6)}
                    | {f'{pre}CS0_{ch}', f'{pre}CKE0_{ch}',
                       f'{pre}CK_{ch}_P', f'{pre}CK_{ch}_N'})
            got = g.get(key, set())
            ok(got == want, f"{key} = its CA, CS, CKE and CK_{ch}_P/N "
               f"(diff {sorted(got ^ want)})")
        ok(len(g) == 6, f"six groups in all ({sorted(g)})")

        # The JEDEC ball spelling (channel last) is the reference partition.
        jedec = lpddr4(pre, strobe='{pre}DQS{n}_{p}_{ch}',
                       clock='{pre}CK_{p}_{ch}', pol='tc')
        rename = {f'_{p}_{ch}': f'_{ch}_{q}' for ch in 'AB'
                  for p, q in (('t', 'P'), ('c', 'N'))}

        def as_ap(n):
            for old, new in rename.items():
                if n.endswith(old):
                    return n[:-len(old)] + new
            return n
        ref = {k: {as_ap(n) for n in v}
               for k, v in grouped(classify_ddr_nets(jedec)).items()}
        ok(ref == g, "the _A_P spelling partitions exactly as the _t_A one")

    print("polarity without a channel")
    plain = ([f'DDR_DQ{i}' for i in range(16)] + ['DDR_DM0', 'DDR_DM1']
             + [f'DDR_DQS{n}_{p}' for n in (0, 1) for p in 'PN']
             + ['DDR_A0', 'DDR_BA0', 'DDR_CKE', 'DDR_CK_P', 'DDR_CK_N'])
    c = classify_ddr_nets(plain)
    g = grouped(c)
    ok(not c['channels'] and set(g) == {'byte_lane_0', 'byte_lane_1',
                                        'command_address'}
       and {'DDR_DQS0_P', 'DDR_DQS0_N'} <= g['byte_lane_0'],
       f"DDR_DQS0_P/N: no channel, plain keys, strobe in lane 0 "
       f"({c['channels']}, {sorted(g)})")

    if FAILS:
        print(f'FAIL: {len(FAILS)}: ' + '; '.join(FAILS))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
