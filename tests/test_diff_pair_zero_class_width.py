#!/usr/bin/env python3
"""A net class's diff_pair_width / diff_pair_gap of 0 is unset, not a 0 mm
pair (found reviewing #1276).

routed_output.kicad_pro stores `diff_pair_width: 0.0` and `diff_pair_gap: 0.0`
on its Default class (as does the corpus's usb_fm_radio). route_diff.py read
the 0 as a declared width: main() printed "using the board Default net-class
diff-pair-width 0.0mm" and passed track_width=0.0, and the #435 per-pair class
geometry took the 0 as the pair's width, so every pair routed at the fab floor
-- on whichever front, since both reach the engine. KiCad 7 makes no
diff-pair rule for a 0, and KiCad 10 reads it as a set 0 and draws its board
minimum; nobody means a 0 mm pair. list_nets.class_diff_pair_geometry, which
route_diff.py, the per-pair geometry and the GUI's Differential tab all
resolve through, treats a 0 like an absent value: the class's own value, else
Default's, else the class track width / clearance.

Rows:
  - the helper: a 0 and an absent field are unset; a declared one is kept
    (control); a class inherits Default's; nothing declared is None;
  - engine (both fronts): cap_chain under a Default class of track 0.25 /
    clearance 0.2 with diff_pair_width / diff_pair_gap 0, routed with the
    class geometry: the pairs draw at the 0.25 track width, not the 0.127
    fab floor; a class declaring 0.2 / 0.2 still draws 0.2 (control);
  - CLI: route_diff.py without --track-width / --diff-pair-gap on the 0
    class names the track width it fell back to and routes at it; on the
    declared class the message and width are unchanged (control).

    python3 tests/test_diff_pair_zero_class_width.py
"""
import contextlib
import io
import json
import os
import shutil
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in ('py_router', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _d))

from run_utils import check as run_check, evidence, tool  # noqa: E402

FAILS = []
BASE = {"name": "Default", "clearance": 0.2, "track_width": 0.25,
        "via_diameter": 0.6, "via_drill": 0.3}
ZERO = dict(BASE, diff_pair_width=0.0, diff_pair_gap=0.0)
DECLARED = dict(BASE, diff_pair_width=0.2, diff_pair_gap=0.2)


def check(name, cond, detail=''):
    print(('  PASS  ' if cond else '  FAIL  ') + name + (f'  [{detail}]' if detail else ''))
    if not cond:
        FAILS.append(name)


def test_helper():
    try:
        from list_nets import class_diff_pair_geometry as geom
    except ImportError as e:
        check('list_nets.class_diff_pair_geometry exists', False, str(e))
        return
    check('a 0 width and gap fall back to the class track width / clearance',
          geom({'diff_pair_width': 0.0, 'diff_pair_gap': 0.0, 'track_width': 0.09,
                'clearance': 0.09}) == (0.09, 0.09))
    check('absent ones too', geom({'track_width': 0.3, 'clearance': 0.2}) == (0.3, 0.2))
    check('a declared width and gap are kept (control)',
          geom({'diff_pair_width': 0.2, 'diff_pair_gap': 0.15, 'track_width': 0.3,
                'clearance': 0.2}) == (0.2, 0.15))
    check("a class that leaves them unset inherits Default's",
          geom({'diff_pair_width': 0, 'track_width': 0.3},
               {'diff_pair_width': 0.2, 'diff_pair_gap': 0.15}) == (0.2, 0.15))
    check('nothing declared is None', geom({}) == (None, None)
          and geom({'diff_pair_width': None, 'track_width': 'x'}) == (None, None))


def _board(td, label, cls):
    sub = os.path.join(td, label)
    os.makedirs(sub)
    dst = os.path.join(sub, 'cc.kicad_pcb')
    shutil.copy(os.path.join(ROOT, 'kicad_files', 'cap_chain.kicad_pcb'), dst)
    for stem in ('cc', 'o'):
        with open(os.path.join(sub, stem + '.kicad_pro'), 'w', encoding='utf-8') as fh:
            json.dump({"net_settings": {"classes": [cls]}}, fh)
    return dst, os.path.join(sub, 'o.kicad_pcb')


def _pair_widths(out):
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(evidence(out, 'routed output'))
    names = {n.net_id: n.name for n in pcb.nets.values()}
    return sorted({round(s.width, 4) for s in pcb.segments
                   if names.get(s.net_id, '').startswith('DP')})


def test_engine(td):
    import route_diff
    for label, cls, want in (('engine_zero', ZERO, 0.25), ('engine_declared', DECLARED, 0.2)):
        dst, out = _board(td, label, cls)
        with contextlib.redirect_stdout(io.StringIO()):
            route_diff.batch_route_diff_pairs(dst, out, ['DPA_*', 'DPB_*'], track_width=want,
                                              diff_pair_gap=0.2, clearance=0.2,
                                              diff_pair_width_from_class=True,
                                              diff_pair_gap_from_class=True,
                                              layers=['F.Cu', 'B.Cu'])
        w = _pair_widths(out)
        check(f'engine, class pair width {cls["diff_pair_width"]}: coupled legs at {want}',
              bool(w) and abs(max(w) - want) < 1e-3, str(w))


def test_cli(td):
    for label, cls, want, says in (
            ('cli_zero', ZERO, 0.25,
             'using the board Default net-class track width (its diff-pair-width is unset) 0.25mm'),
            ('cli_declared', DECLARED, 0.2,
             'using the board Default net-class diff-pair-width 0.2mm')):
        dst, out = _board(td, label, cls)
        r = run_check([sys.executable, '-X', 'utf8', tool('route_diff.py'), dst, out,
                       '--nets', 'DPA_*', 'DPB_*', '--layers', 'F.Cu', 'B.Cu'], accept=True)
        log = r.stdout + r.stderr
        line = next((l.strip() for l in log.splitlines() if '--track-width not given' in l), '')
        check(f'CLI, class pair width {cls["diff_pair_width"]}: the width it used is named',
              says in line, line)
        w = _pair_widths(out)
        check(f'CLI, class pair width {cls["diff_pair_width"]}: coupled legs at {want}',
              bool(w) and abs(max(w) - want) < 1e-3, str(w))
    gline = next((l.strip() for l in log.splitlines() if '--diff-pair-gap not given' in l), '')
    check('CLI, declared gap: unchanged message (control)',
          'using the board Default net-class diff-pair-gap 0.2mm' in gline, gline)


def main():
    test_helper()
    with tempfile.TemporaryDirectory(prefix='zero_dpw_') as td:
        test_engine(td)
        test_cli(td)
    if FAILS:
        print(f'\nFAIL: {len(FAILS)} check(s): ' + '; '.join(FAILS))
        return 1
    print('\nALL CHECKS PASSED')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
