#!/usr/bin/env python3
"""A fanout reports a net unescaped only while one of its pads has no copper.

  python3 tests/test_fanout_unescaped_means_bare.py

The engines record a failure per ROUTE, and the fronts reported that list as
the unescaped nets: daisho U1's summary listed V1P2 -- one of its 50 balls'
routes given up, every ball still served -- as unescaped, counted it as both
escaped and failed (requested 371 for 370 nets), and printed "1 of 371
requested ball(s) could NOT be escaped". Pinned, on in-repo boards with the
engine's own result doctored the two ways that matter (a net whose given-up
route's ball is still served, S, and a net whose ball is left bare, X):

1. bga_fanout's public return -- what the GUI fanout tab reports as "Failed
   nets" -- names X and not S;
2. bga_fanout.py's JSON_SUMMARY: unescaped_nets == [X], requested counts each
   net once (the clean run's 77), requested == escaped + failed;
3. qfn_fanout.py's JSON_SUMMARY: a served name the engine flagged is not in
   unescaped_nets / failed, and is published as flagged_escaped_nets (the
   engine's "stubs too close" list is not a list of failures).
"""
import contextlib
import io
import json
import os
import runpy
import shutil
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

BGA_BOARD = os.path.join(ROOT, 'kicad_files', 'interf_u_unrouted_placed.kicad_pcb')
QFN_BOARD = os.path.join(ROOT, 'kicad_files', 'haasoscope_pro_max_test.kicad_pcb')
CLEAN_REQUESTED = 77          # U9's clean ledger (tests/test_fanout_cancel.py)


def _doctor_bga(bga_fanout, picked):
    """Wrap the core so its OUTERMOST result carries S (still served) and X
    (copper removed) in its failed list."""
    orig = bga_fanout._generate_bga_fanout_core
    depth = [0]

    def core(*a, **k):
        depth[0] += 1
        try:
            tracks, vias, vr, failed = orig(*a, **k)
        finally:
            depth[0] -= 1
        if depth[0] == 0 and not picked:
            fp, pcb = a[0], a[1]
            names = {p.net_id: p.net_name for p in fp.pads if p.net_id}
            esc = sorted({t['net_id'] for t in tracks if t['net_id'] in names})
            s_id, x_id = esc[0], esc[1]
            picked.update(S=names[s_id], X=names[x_id])
            tracks = [t for t in tracks if t['net_id'] != x_id]
            vias = [v for v in vias if v['net_id'] != x_id]
            failed = list(failed) + [names[s_id], names[x_id]]
        return tracks, vias, vr, failed
    return orig, core


def _quiet(fn):
    with contextlib.redirect_stdout(io.StringIO()) as buf:
        out = fn()
    return out, buf.getvalue()


def _summary(text):
    rows = [l.split('JSON_SUMMARY: ', 1)[1] for l in text.splitlines()
            if l.startswith('JSON_SUMMARY: ')]
    return json.loads(rows[-1]) if rows else None


def check_bga(fails, d):
    from kicad_parser import parse_kicad_pcb
    import bga_fanout
    # 1. the engine's public return
    picked = {}
    orig, core = _doctor_bga(bga_fanout, picked)
    bga_fanout._generate_bga_fanout_core = core
    try:
        pcb = parse_kicad_pcb(BGA_BOARD)
        (_t, _v, _vr, failed), _ = _quiet(
            lambda: bga_fanout.generate_bga_fanout(pcb.footprints['U9'], pcb))
    finally:
        bga_fanout._generate_bga_fanout_core = orig
    if not picked:
        fails.append("fixture: the BGA core was never doctored")
        return
    if picked['S'] in failed:
        fails.append(f"generate_bga_fanout reports {picked['S']} failed, though every "
                     f"one of its balls has copper")
    if picked['X'] not in failed:
        fails.append(f"generate_bga_fanout does not report {picked['X']}, whose ball "
                     f"has no copper")
    # 2. the CLI summary
    picked2 = {}
    orig, core = _doctor_bga(bga_fanout, picked2)
    bga_fanout._generate_bga_fanout_core = core
    argv = sys.argv
    try:
        sys.argv = ['bga_fanout.py', BGA_BOARD, '--component', 'U9',
                    '--output', os.path.join(d, 'bga.kicad_pcb')]
        _, text = _quiet(lambda: bga_fanout.main())
    finally:
        bga_fanout._generate_bga_fanout_core = orig
        sys.argv = argv
    s = _summary(text)
    if s is None:
        fails.append("bga_fanout.py printed no JSON_SUMMARY")
        return
    if s.get('unescaped_nets') != [picked2['X']]:
        fails.append(f"bga_fanout.py unescaped_nets {s.get('unescaped_nets')}, not "
                     f"[{picked2['X']}] (the served {picked2['S']} must not be there)")
    if s.get('requested') != CLEAN_REQUESTED or \
            s.get('requested') != s.get('escaped', 0) + s.get('failed', 0):
        fails.append(f"bga_fanout.py counts a net twice: requested {s.get('requested')} "
                     f"(the clean run's {CLEAN_REQUESTED}), escaped {s.get('escaped')}, "
                     f"failed {s.get('failed')}")


def check_qfn(fails, d):
    import qfn_fanout
    orig = qfn_fanout.generate_qfn_fanout
    seen = {}

    def eng(*a, **k):
        tracks, vias, failed = orig(*a, **k)
        names = {p.net_id: p.net_name for p in a[0].pads if p.net_id}
        s_id = sorted({t['net_id'] for t in tracks if t['net_id'] in names})[0]
        seen['S'] = names[s_id]
        return tracks, vias, list(failed) + [names[s_id]]
    qfn_fanout.generate_qfn_fanout = eng
    argv = sys.argv
    try:
        sys.argv = ['qfn_fanout.py', QFN_BOARD, '--component', 'U2',
                    '--output', os.path.join(d, 'qfn.kicad_pcb')]
        _, text = _quiet(lambda: qfn_fanout.main())
    finally:
        qfn_fanout.generate_qfn_fanout = orig
        sys.argv = argv
    s = _summary(text)
    if s is None or 'S' not in seen:
        fails.append("qfn_fanout.py printed no JSON_SUMMARY, or the engine was not doctored")
        return
    if seen['S'] in (s.get('unescaped_nets') or []) or s.get('failed'):
        fails.append(f"qfn_fanout.py reports {seen['S']} unescaped (failed {s.get('failed')}), "
                     f"though its pads have copper")
    if s.get('flagged_escaped_nets') != [seen['S']]:
        fails.append(f"qfn_fanout.py does not publish the flagged net under its own key "
                     f"(flagged_escaped_nets {s.get('flagged_escaped_nets')})")
    if s.get('requested') != s.get('escaped', 0) + s.get('failed', 0):
        fails.append(f"qfn_fanout.py requested {s.get('requested')} != escaped "
                     f"{s.get('escaped')} + failed {s.get('failed')}")


def main():
    print('=' * 60)
    print("fanout unescaped_nets: a net is unescaped only while a pad has no copper")
    print('=' * 60)
    fails = []
    for b in (BGA_BOARD, QFN_BOARD):
        if not os.path.isfile(b):
            fails.append(f"missing board {b}")
    d = tempfile.mkdtemp(prefix='unescaped_')
    try:
        if not fails:
            check_bga(fails, d)
            check_qfn(fails, d)
    finally:
        shutil.rmtree(d, ignore_errors=True)
    for f in fails:
        print(f'  FAIL: {f}')
    if fails:
        return 1
    print("PASS: a given-up route whose ball is served is not unescaped (engine return, "
          "both CLI summaries); a bare ball is; requested counts each net once")
    return 0


if __name__ == '__main__':
    sys.exit(main())
