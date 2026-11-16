"""#1936 part 3: check_impedance.py reports the copper of a net that is off its
tuning profile's width, and a pair off the profile's gap.

A KiCad 10 tuning profile stores the width (and a differential profile the
P/N gap) that meet its impedance on each layer. KiCad's DRC has no row for a
track drawn at another width, and nothing here read it back off the board.
`check_impedance.py` now reads each net's profile from the project and
reports, per net: the length on layers the profile sizes, how much of it is
at another width, and for a pair the length parallel to its partner and how
much of that is at another gap. Disclosure: the exit code does not change.

Single-ended, #1846's fixture (nets P and Q of class TIMED, profile PROF6,
tracks 0.1 mm on F.Cu):
  1. a profile width of 0.1 mm: both nets sized, none off;
  2. a profile width of 0.2 mm: both named, with the layer, the drawn width,
     the profile's and the length, in the report and the JSON; the exit code
     is the one the board has with the profile met;
  3. Q through two vias onto B.Cu, which the profile gives no width: that
     length is reported as not graded, never as off;
  4. a class that names no profile: no section, no key set.
A pair, cap_chain routed by route_diff.py at its differential profile's
F.Cu width and gap (0.15 mm / 0.3 mm):
  5. against the profile it was routed at: every pair net sized and coupled,
     none off;
  6. the same copper against a profile gap of 0.35 mm: every net off its
     gap over its coupled length, none off its width;
  7. against a profile width of 0.2 mm: every net off its width.
`profile_geometry` is also called on hand-built segments, where each length
is known exactly.
"""
import contextlib
import copy
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile
from types import SimpleNamespace

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, os.path.join(REPO, 'py_tools'))

import run_utils  # noqa: E402
import test_1846_time_domain_rules as t1846  # noqa: E402
import test_1936_tuning_profile_pair_widths as tpair  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
TOOL = os.path.join(REPO, 'py_tools', 'check_impedance.py')
SECTION = '-- Tuning-profile width and pair gap vs the shipped copper'
FAILS = []


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def audit(board):
    """(exit code, report text, JSON document) of check_impedance.py."""
    out = board[:-len('.kicad_pcb')] + '.imp.json'
    r = subprocess.run(PY + [TOOL, run_utils.evidence(board), '--json', out],
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=REPO)
    assert 'Traceback' not in r.stderr and r.returncode in (0, 1), (
        f"BROKEN TEST: check_impedance.py did not run\n{(r.stdout + r.stderr)[-1200:]}")
    with open(run_utils.evidence(out), encoding='utf-8') as fh:
        return r.returncode, r.stdout, json.load(fh)


def rows(doc):
    return {n['net_name']: n for n in doc['nets']}


def _width(mm):
    def edit(proj):
        t1846._profile(proj)['layer_entries'][0]['width'] = int(mm * 1e6)
    return edit


def measured():
    print('0. profile_geometry on known segments')
    from check_impedance import profile_geometry
    from tuning_profiles import TuningProfile

    def seg(x1, y1, x2, y2, w, layer='F.Cu'):
        return SimpleNamespace(start_x=x1, start_y=y1, end_x=x2, end_y=y2,
                               width=w, layer=layer)
    prof = TuningProfile('D', False, {}, 0.0, {}, {'F.Cu': 0.2}, {'F.Cu': 0.3}, True)
    # P: 10 mm at the profile width with N 0.3 mm off, 4 mm at 0.15, then a
    # 3 mm fan-in leg that is not parallel to N, and 2 mm on B.Cu
    p = [seg(0, 0, 10, 0, 0.2), seg(10, 0, 14, 0, 0.15), seg(14, 0, 17, 1, 0.2),
         seg(17, 1, 19, 1, 0.2, 'B.Cu')]
    n = [seg(0, 0.5, 10, 0.5, 0.2), seg(10, 0.55, 14, 0.55, 0.15)]
    got = profile_geometry(p, n, prof, sample_step=0.1)
    fan = (3 ** 2 + 1) ** 0.5
    ok(abs(got['length'] - (14 + fan)) < 1e-6 and abs(got['off_width'] - 4) < 1e-6
       and abs(got['unsized'] - 2) < 1e-6,
       f"sized {got['length']:.3f}, off width {got['off_width']:.3f}, "
       f"unsized {got['unsized']:.3f}")
    ok(abs(got['coupled'] - 14) < 1e-6 and abs(got['off_gap'] - 4) < 1e-6,
       f"coupled over the parallel 14 mm only, 4 mm of it at gap 0.4: "
       f"{got['coupled']:.3f}, {got['off_gap']:.3f}")
    ok(got['rows'] == [
        {'layer': 'F.Cu', 'what': 'gap', 'drawn_mm': 0.4, 'drawn_max_mm': 0.4,
         'profile_mm': 0.3, 'length_mm': 4.0},
        {'layer': 'F.Cu', 'what': 'width', 'drawn_mm': 0.15, 'profile_mm': 0.2,
         'length_mm': 4.0}], f"the rows name both: {got['rows']}")
    single = TuningProfile('S', False, {}, 0.0, {}, {'F.Cu': 0.2}, {'F.Cu': 0.3}, False)
    got = profile_geometry(p, n, single, sample_step=0.1)
    ok(got['coupled'] == 0 and got['off_gap'] == 0 and abs(got['off_width'] - 4) < 1e-6,
       'a single-ended profile grades the width and no gap')


def single_ended(td):
    print('1-4. single-ended')
    board = t1846.stage(td, 'on', t1846.FLOOR, False, _width(0.1))
    rc_on, text, doc = audit(board)
    got = rows(doc)
    ok(SECTION in text and '0 of 2 profiled net(s) off' in text
       and doc['totals']['nets_profiled'] == 2 and doc['totals']['nets_off_profile'] == []
       and got['P']['profile'] == 'PROF6' and got['P']['profile_length_mm'] == 20.0
       and got['Q']['profile_length_mm'] == 21.0,
       f"the profile's width met: both nets sized, none off "
       f"({got['P']['profile_length_mm']}, {got['Q']['profile_length_mm']} mm)")

    board = t1846.stage(td, 'off', t1846.FLOOR, False, _width(0.2))
    rc_off, text, doc = audit(board)
    got = rows(doc)
    ok(doc['totals']['nets_off_profile'] == ['P', 'Q']
       and got['Q']['profile_off'] == [{'layer': 'F.Cu', 'what': 'width', 'drawn_mm': 0.1,
                                        'profile_mm': 0.2, 'length_mm': 21.0}]
       and got['Q']['off_profile'] and got['P']['profile_off_width_mm'] == 20.0,
       f"a wider profile: both named with layer, widths and length: "
       f"{got['Q']['profile_off']}")
    ok('OFF PROFILE' in text and 'F.Cu: 21.00 mm at width 0.1 (profile 0.2)' in text
       and '2 of 2 profiled net(s) off' in text, 'and the report prints the same')
    ok(rc_off == rc_on, f"the exit code is unchanged ({rc_on} -> {rc_off})")

    board = t1846.stage(td, 'vias', t1846.FLOOR, True, _width(0.1))
    _rc, text, doc = audit(board)
    got = rows(doc)
    ok(got['Q']['profile_length_mm'] == 14.0 and got['Q']['profile_unsized_mm'] == 7.0
       and not got['Q']['off_profile']
       and '7.00 mm on layers the profile gives no width (not graded)' in text,
       f"a layer the profile gives no width is not graded: "
       f"{got['Q']['profile_unsized_mm']} mm")

    def no_profile(proj):
        _width(0.2)(proj)
        t1846._no_profile(proj)
    board = t1846.stage(td, 'np', t1846.FLOOR, False, no_profile)
    _rc, text, doc = audit(board)
    got = rows(doc)
    ok(SECTION not in text and doc['totals']['nets_profiled'] == 0
       and got['Q']['profile'] == '' and not got['Q']['off_profile'],
       'a class that names no profile: no section')


def pair(td):
    print('5-7. a routed pair')
    entries = [tpair.entry('F.Cu', 0.15, 0.3)]
    board, out = tpair.stage(td, 'pair', tpair.project(entries))
    r = subprocess.run(PY + [os.path.join('py_router', 'route_diff.py'), board, out,
                             '--nets'] + tpair.NETS + ['--layers', 'F.Cu'],
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=REPO)
    assert r.returncode == 0 and tpair.LINE in r.stdout, (
        f"BROKEN TEST: the pairs were not routed at their profile\n"
        f"{(r.stdout + r.stderr)[-1200:]}")
    nets = ['DPA_N', 'DPA_P', 'DPB_N', 'DPB_P']

    def against(tag, new_entries):
        dst = os.path.join(td, tag)
        os.makedirs(dst)
        b = os.path.join(dst, 'o.kicad_pcb')
        shutil.copy(run_utils.evidence(out), b)
        with open(os.path.join(dst, 'o.kicad_pro'), 'w', encoding='utf-8') as fh:
            json.dump(tpair.project(copy.deepcopy(new_entries)), fh)
        return audit(b)

    _rc, text, doc = against('same', entries)
    got = rows(doc)
    ok(sorted(got) == nets and doc['totals']['nets_off_profile'] == []
       and all(got[n]['profile_differential'] and got[n]['profile_coupled_mm'] > 3.0
               and got[n]['profile_off_gap_mm'] == 0 and got[n]['profile_off_width_mm'] == 0
               for n in nets),
       "routed at its profile: every pair net sized and coupled, none off ("
       + ', '.join(f"{n} {got[n]['profile_coupled_mm']:.1f}" for n in nets) + ' mm coupled)')

    _rc, text, doc = against('gap', [tpair.entry('F.Cu', 0.15, 0.35)])
    got = rows(doc)
    ok(doc['totals']['nets_off_profile'] == nets
       and all(abs(got[n]['profile_off_gap_mm'] - got[n]['profile_coupled_mm']) < 1e-3
               and got[n]['profile_off_width_mm'] == 0
               and got[n]['profile_off'][0]['what'] == 'gap'
               and got[n]['profile_off'][0]['drawn_mm'] == 0.3 for n in nets),
       f"a profile gap of 0.35: every net off its gap over its coupled length, at "
       f"0.3: {got['DPA_P']['profile_off'][:1]}")
    ok('at gap 0.3 (profile 0.35)' in text, 'and the report prints the gap row')

    _rc, _text, doc = against('width', [tpair.entry('F.Cu', 0.2, 0.3)])
    got = rows(doc)
    ok(doc['totals']['nets_off_profile'] == nets
       and all(abs(got[n]['profile_off_width_mm'] - got[n]['profile_length_mm']) < 1e-3
               for n in nets),
       f"a profile width of 0.2: every net off its width over its whole length")


def main():
    td = tempfile.mkdtemp(prefix='krt1936c_')
    try:
        with contextlib.redirect_stderr(io.StringIO()):
            measured()
        single_ended(td)
        pair(td)
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if FAILS:
        print(f'FAIL: {len(FAILS)}: ' + '; '.join(FAILS))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
