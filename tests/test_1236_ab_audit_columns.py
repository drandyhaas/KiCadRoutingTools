#!/usr/bin/env python3
"""#1236 (with #1232-#1235): the corpus A/B grade records the audit columns, and its verdict ignores them.

1. grade() on a small routed board carries every AUDIT_FIELDS key, each the
   final board's count minus the unrouted input's: two off-angle corners, one
   short fragment, one off-centre via end, one via-mask contact, and
   check_impedance's crossing totals.
2. A finding the input already had is not counted again.
3. compare() prints the audit columns beside the verdict, and a pair that
   differs ONLY in them is "no regression"; a DRC regression is still one.

    python3 tests/test_1236_ab_audit_columns.py
"""
import io
import json
import os
import sys
import tempfile
from contextlib import redirect_stdout

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tests', 'stress'))

import ab_replay_grade as abg  # noqa: E402

RUN_ALL_TIMEOUT = 300

FAILS = []


def check(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}: {name}"
          + (f"   [{detail}]" if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


HEAD = ('(kicad_pcb (version 20240202) (generator "t")\n'
        ' (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (35 "F.Paste" user)'
        ' (39 "F.Mask" user) (38 "B.Mask" user) (44 "Edge.Cuts" user))\n'
        ' (setup (pad_to_mask_clearance 0.1))\n'
        ' (net 0 "") (net 1 "/A") (net 2 "/B")\n'
        ' (gr_rect (start 90 90) (end 120 110) (layer "Edge.Cuts") (width 0.1))\n'
        ' (footprint "L:P" (layer "F.Cu") (at 100 100) (property "Reference" "U1")\n'
        '  (pad "1" smd rect (at 0 0) (size 1 0.5) (layers "F.Cu" "F.Paste" "F.Mask")'
        ' (net 1 "/A")))\n'
        ' (footprint "L:P" (layer "F.Cu") (at 112 100) (property "Reference" "U2")\n'
        '  (pad "1" smd rect (at 0 0) (size 1 0.5) (layers "F.Cu" "F.Paste" "F.Mask")'
        ' (net 1 "/A")))\n'
        ' (footprint "L:P" (layer "F.Cu") (at 100 105) (property "Reference" "U3")\n'
        '  (pad "1" smd rect (at 0 0) (size 1 0.5) (layers "F.Cu" "F.Paste" "F.Mask")'
        ' (net 2 "/B")))\n')

# A via whose ring is in U3's mask opening but clear of its copper and paste.
INPUT_VIA = ' (via (at 100.8 105) (size 0.5) (drill 0.3) (layers "F.Cu" "B.Cu") (net 2))\n'

ROUTED = (
    ' (segment (start 100 100) (end 104 100) (width 0.2) (layer "F.Cu") (net 1))\n'
    # a 30 degree jog: a corner at (104, 100) and another at (106.598, 101.5)
    ' (segment (start 104 100) (end 106.598 101.5) (width 0.2) (layer "F.Cu") (net 1))\n'
    ' (segment (start 106.598 101.5) (end 106.648 101.5) (width 0.2) (layer "F.Cu") (net 1))\n'
    ' (segment (start 106.648 101.5) (end 108 101.5) (width 0.2) (layer "F.Cu") (net 1))\n'
    # ends 0.1 mm off the centre of the via at (108.1, 101.5)
    ' (via (at 108.1 101.5) (size 0.5) (drill 0.3) (layers "F.Cu" "B.Cu") (net 1))\n'
    ' (segment (start 108.1 101.5) (end 110 101.5) (width 0.2) (layer "B.Cu") (net 1))\n'
    ' (via (at 110 101.5) (size 0.5) (drill 0.3) (layers "F.Cu" "B.Cu") (net 1))\n'
    ' (segment (start 110 101.5) (end 112 100) (width 0.2) (layer "F.Cu") (net 1))\n'
    # a new via whose ring is in U2's mask opening
    ' (via (at 112.8 100) (size 0.5) (drill 0.3) (layers "F.Cu" "B.Cu") (net 2))\n')


def main():
    d = tempfile.mkdtemp(prefix='t1236_')
    base, final = os.path.join(d, 'board0.kicad_pcb'), os.path.join(d, 'final.kicad_pcb')
    with open(base, 'w') as fh:
        fh.write(HEAD + INPUT_VIA + ')\n')
    with open(final, 'w') as fh:
        fh.write(HEAD + INPUT_VIA + ROUTED + ')\n')

    row = abg.grade(final, '0.15', baseline=base)
    missing = [k for k in abg.AUDIT_FIELDS if k not in row]
    check('1. grade() carries every audit column', not missing, str(missing))
    want = {'audit_off_angle_corner': 2, 'audit_short_fragment': 1,
            'audit_off_centre_via_end': 1, 'audit_via_mask_contact': 1,
            'audit_via_mask_web': 0}
    got = {k: row.get(k) for k in want}
    check('1. the counts are the router-introduced ones', got == want, str(got))
    check('1. check_impedance totals are recorded',
          all(isinstance(row.get(k), (int, float)) for k in
              ('impedance_crossings', 'impedance_nets_with_crossing',
               'impedance_length_over_void_mm')),
          str({k: row.get(k) for k in abg.AUDIT_FIELDS}))
    check('1. no audit error', 'audit_error' not in row and 'impedance_error' not in row,
          str(row.get('audit_error') or row.get('impedance_error')))

    own = abg._audit_grade(final, None)
    check("2. without a baseline the input's via counts too",
          own['audit_via_mask_contact'] == 2, str(own))

    # ---- 3. compare() ----------------------------------------------------------
    core = {'chain_complete': True, 'drc_real': 0, 'nets_incomplete': 0,
            'completion_pct': 100.0}
    old = [dict(core, board='b1', **{k: 0 for k in abg.AUDIT_FIELDS})]
    new = [dict(core, board='b1', **{k: 9 for k in abg.AUDIT_FIELDS})]
    po, pn = os.path.join(d, 'old.json'), os.path.join(d, 'new.json')
    json.dump(old, open(po, 'w'))
    json.dump(new, open(pn, 'w'))
    buf = io.StringIO()
    with redirect_stdout(buf):
        ok = abg.compare(po, pn)
    text = buf.getvalue()
    check('3. a pair differing only in audit columns is no regression', ok is True, text[-600:])
    check('3. the columns are printed beside the verdict',
          'NOT in the verdict' in text and 'audit_via_mask_contact' in text, text[-600:])
    new[0]['drc_real'] = 1
    json.dump(new, open(pn, 'w'))
    with redirect_stdout(io.StringIO()):
        ok = abg.compare(po, pn)
    check('3. a DRC regression is still a regression', ok is False)

    for f in os.listdir(d):
        os.remove(os.path.join(d, f))
    os.rmdir(d)
    print(f"\n{'ALL PASS' if not FAILS else '%d FAIL' % len(FAILS)}")
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
