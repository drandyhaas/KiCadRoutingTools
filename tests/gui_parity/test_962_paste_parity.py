#!/usr/bin/env python3
"""#962 parity gate for the paste-stencil model: both parse paths, plus native pcbnew as an oracle.

`parse_kicad_pcb` (file text) and `build_pcb_data_from_board` (live pcbnew)
feed the SAME `paste_apertures.build_paste_apertures`. Parity can therefore
only break in what each path READS: the pad and footprint paste overrides, the
board `(setup ...)`, the paste graphics, and the new graphic-copper fields
(`drawn_width` / `graphic_kind` / `graphic_circle`). This gate compares every
one of those on every tracked board, and adds two checks the paths cannot give
each other.

- **A native oracle.** pcbnew's own `PAD.GetSolderPasteMargin(layer)` is
  KiCad's RESOLVED per-axis margin, and every pad opening's margin must equal
  it. Two parse paths that shared a wrong resolver would agree with each other
  and still fail here.
- **Witnesses.**
  - esp_prog U2's F.Paste graphic exists.
  - glasgow J1's pin-in-paste graphics exist.
  - ulx3s carries a pad whose ratio is -0.2.

  A parser that silently produced NO apertures would pass every parity arm.
  The witnesses make that fail instead.

A comparator self-test runs first. It removes one aperture from a copy and
requires the multiset comparison to notice, so the comparison cannot go blind
without this gate failing.

Needs pcbnew; re-execs into KiCad's python automatically. Exits 2 when no
pcbnew python is found, rather than 0, so a mutation battery using it as a
killer cannot score SURVIVED on an environment accident.

    python3 -X utf8 tests/gui_parity/test_962_paste_parity.py
"""
import glob
import math
import os
import subprocess
import sys
from collections import Counter

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

KICAD_PYTHONS = [
    "/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3",
    "/usr/bin/python3",
    os.path.expandvars(r"C:\Program Files\KiCad\bin\python.exe"),
    *sorted(glob.glob(r"C:\Program Files\KiCad\*\bin\python.exe"), reverse=True),
]

TOL = 1e-6


def _reexec_into_kicad():
    for cand in KICAD_PYTHONS:
        if cand == sys.executable or not os.path.exists(cand):
            continue
        if subprocess.run([cand, '-c', 'import pcbnew'],
                          capture_output=True).returncode == 0:
            argv = [cand, '-X', 'utf8', os.path.abspath(__file__)] + sys.argv[1:]
            if os.name == 'nt':
                sys.exit(subprocess.run(argv).returncode)
            os.execv(cand, argv)
    print("NOT RUN: no python with pcbnew found (exit 2, not a pass)")
    sys.exit(2)


FAILURES = []


def check(cond, what, detail=''):
    if cond:
        print('  ok    %s' % what)
    else:
        print('  FAIL  %s%s' % (what, ('  -- ' + detail) if detail else ''))
        FAILURES.append(what)


def _r(v, nd=4):
    return round(float(v), nd) + 0.0


def aperture_key(ap, nets):
    b = ap.bounds
    return (ap.owner_ref, ap.layer, ap.source, ap.pad_number,
            nets.get(ap.net_id).name if ap.net_id in nets else '',
            _r(b[0]), _r(b[1]), _r(b[2]), _r(b[3]),
            _r(ap.margin[0], 6), _r(ap.margin[1], 6), bool(ap.filled),
            _r(ap.width, 6))


#: Aperture BOUNDS are compared with a tolerance, not exactly. Reason, measured:
#: orangecrab U9's custom pads (an anchor rect plus a `gr_circle` primitive,
#: `polygons` None on both paths) already carry a custom-pad EXTENT that
#: differs by 2.7 um between the two parse paths. That predates #962, since
#: the extent builders are separate (`_custom_pad_board_extent` vs pcbnew's
#: bounding box), and the opening inherits it. 5 um admits that residue and
#: still fails on any real difference in margin, size or position (the
#: smallest margin in the corpus is 35 um).
BOUNDS_TOL = 0.005


def identity_key(ap, nets):
    """Everything about an aperture except its geometry."""
    return (ap.owner_ref, ap.layer, ap.source, ap.pad_number,
            nets.get(ap.net_id).name if ap.net_id in nets else '',
            _r(ap.margin[0], 6), _r(ap.margin[1], 6), bool(ap.filled),
            _r(ap.width, 6))


def bounds_delta(f, g):
    """Worst bounds disagreement over identity-matched apertures (mm)."""
    from collections import defaultdict
    ga, gb = defaultdict(list), defaultdict(list)
    for a in f.paste_apertures:
        ga[identity_key(a, f.nets)].append(a.bounds)
    for a in g.paste_apertures:
        gb[identity_key(a, g.nets)].append(a.bounds)
    worst, where = 0.0, ''
    for k, la in ga.items():
        lb = gb.get(k, [])
        for ba, bb in zip(sorted(la), sorted(lb)):
            d = max(abs(x - y) for x, y in zip(ba, bb))
            if d > worst:
                worst, where = d, '%s: %s vs %s' % (k[:4], ba, bb)
    return worst, where


def graphic_key(s):
    c = s.graphic_circle
    return (s.owner_ref, s.layer, s.graphic_kind,
            None if s.drawn_width is None else _r(s.drawn_width, 6),
            None if c is None else (_r(c[0]), _r(c[1]), _r(c[2])),
            bool(s.graphic_filled))


def multiset_diff(a, b):
    ca, cb = Counter(a), Counter(b)
    return sorted((ca - cb).items())[:4], sorted((cb - ca).items())[:4]


def main():
    try:
        import pcbnew  # noqa: F401
    except ImportError:
        _reexec_into_kicad()
    import pcbnew
    for _p in ('', 'py_router', 'py_placer', 'py_tools', 'tests'):
        _d = os.path.join(REPO, _p)
        if _d not in sys.path:
            sys.path.insert(0, _d)
    from kicad_parser import parse_kicad_pcb, build_pcb_data_from_board
    from run_utils import corpus_boards

    print('KiCad build: %s' % pcbnew.GetBuildVersion())
    boards = corpus_boards()
    check(len(boards) >= 20, 'the tracked corpus is visible (%d boards)' % len(boards),
          'run_utils.corpus_boards() returned too few boards; git cannot see '
          'the corpus, so this gate would test nothing')
    # Synthetic boards for what the corpus does not carry. Each goes through
    # the same arms, the native margin oracle included:
    # - a custom pad with a paste RATIO: KiCad sizes that term from the ANCHOR;
    # - an explicit-0 override: unset up to file version 20240201, a real
    #   override after it.
    import tempfile
    syn_dir = tempfile.mkdtemp(prefix='krt962g_')
    for ver in ('20240108', '20241229'):
        p = os.path.join(syn_dir, 'syn_%s.kicad_pcb' % ver)
        with open(p, 'w', encoding='utf-8') as fh:
            fh.write(
                '(kicad_pcb (version %s) (generator "pcbnew") (general (thickness 1.6))\n'
                ' (paper "A4") (layers (0 "F.Cu" signal) (31 "B.Cu" signal) '
                '(35 "F.Paste" user) (44 "Edge.Cuts" user))\n'
                ' (setup (pad_to_paste_clearance -0.02))\n (net 0 "") (net 1 "/A")\n'
                ' (footprint "L:P" (layer "F.Cu") (at 20 20)\n'
                '  (property "Reference" "U1" (at 0 0 0) (layer "F.SilkS") '
                '(effects (font (size 1 1) (thickness 0.15))))\n'
                '  (solder_paste_margin 0.05) (solder_paste_margin_ratio -0.1)\n'
                '  (pad "C" smd custom (at 0 0) (size 0.5 0.5) (layers "F.Cu" "F.Paste") '
                '(net 1 "/A") (solder_paste_margin -0.05) (solder_paste_margin_ratio -0.1)\n'
                '   (options (clearance outline) (anchor rect))\n'
                '   (primitives (gr_poly (pts (xy -1 -0.5) (xy 1 -0.5) (xy 1 0.5) (xy -1 0.5)) '
                '(width 0) (fill yes))))\n'
                '  (pad "Z" smd rect (at 4 0) (size 1 0.5) (layers "F.Cu" "F.Paste") '
                '(net 1 "/A") (solder_paste_margin 0) (solder_paste_margin_ratio 0))))\n'
                % ver)
        boards.append(p)
    # - copper the off-outline grade reads: visible and hidden copper TEXT
    #   (only the visible one is `unmeasured`), and closed shapes filled and
    #   unfilled by token AND by the no-token rule (`Segment.graphic_filled`).
    #   No tracked board carries copper text, so only this board tests the
    #   live path's text reading.
    p = os.path.join(syn_dir, 'syn_copper.kicad_pcb')
    with open(p, 'w', encoding='utf-8') as fh:
        fh.write(
            '(kicad_pcb (version 20240108) (generator "pcbnew") (general (thickness 1.6))\n'
            ' (paper "A4") (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
            ' (setup)\n (net 0 "") (net 1 "/A")\n'
            ' (gr_rect (start 0 0) (end 60 40) (stroke (width 0.1) (type solid)) (fill none) '
            '(layer "Edge.Cuts"))\n'
            ' (footprint "L:T" (layer "F.Cu") (at 10 10)\n'
            '  (property "Reference" "T1" (at 0 -3 0) (layer "F.Cu") (hide yes) '
            '(effects (font (size 1 1) (thickness 0.15))))\n'
            '  (fp_text user "CU" (at 0 3 0) (layer "F.Cu") '
            '(effects (font (size 1 1) (thickness 0.15))))\n'
            '  (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 1 "/A")))\n'
            ' (footprint "L:S" (layer "F.Cu") (at 30 10)\n'
            '  (property "Reference" "T2" (at 0 -3 0) (layer "F.SilkS") '
            '(effects (font (size 1 1) (thickness 0.15))))\n'
            '  (fp_rect (start -3 -2) (end -1 2) (stroke (width 0.2) (type solid)) '
            '(fill none) (layer "F.Cu"))\n'
            '  (fp_rect (start 1 -2) (end 3 2) (stroke (width 0) (type solid)) (layer "F.Cu"))\n'
            '  (fp_circle (center 0 4) (end 1 4) (stroke (width 0.1) (type solid)) '
            '(fill solid) (layer "F.Cu"))\n'
            '  (fp_poly (pts (xy -1 -5) (xy 1 -5) (xy 0 -4)) (stroke (width 0.1) (type solid)) '
            '(layer "F.Cu"))\n'
            '  (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 1 "/A")))\n'
            ')\n')
    boards.append(p)

    # -- comparator self-test: a dropped aperture must be SEEN ---------------
    esp = os.path.join(REPO, 'kicad_files', 'esp_prog.kicad_pcb')
    f0 = parse_kicad_pcb(esp)
    keys = [aperture_key(a, f0.nets) for a in f0.paste_apertures]
    only_a, only_b = multiset_diff(keys, keys[1:])
    check(bool(only_a) and not only_b,
          'self-test: the multiset comparison notices one dropped aperture')

    witnessed = Counter()
    for path in boards:
        name = os.path.splitext(os.path.basename(path))[0]
        print('\n%s' % name)
        f = parse_kicad_pcb(path)
        board = pcbnew.LoadBoard(path)
        g = build_pcb_data_from_board(board)

        bf, bg = f.board_info, g.board_info
        check(abs(bf.pad_to_paste_clearance - bg.pad_to_paste_clearance) < TOL
              and abs(bf.pad_to_paste_clearance_ratio - bg.pad_to_paste_clearance_ratio) < TOL,
              '%s: board paste setup agrees' % name,
              '%r/%r vs %r/%r' % (bf.pad_to_paste_clearance, bf.pad_to_paste_clearance_ratio,
                                  bg.pad_to_paste_clearance, bg.pad_to_paste_clearance_ratio))
        check(bf.via_protection_setup == bg.via_protection_setup
              and len(bf.via_protection_setup) == 5,
              '%s: via-protection setup agrees (all 5 tokens)' % name,
              '%s vs %s' % (bf.via_protection_setup, bg.via_protection_setup))

        bad_fp, bad_pad = [], []
        for k in sorted(set(f.footprints) & set(g.footprints)):
            a, b = f.footprints[k], g.footprints[k]
            if (a.paste_margin, a.paste_margin_ratio) != (
                    None if b.paste_margin is None else _r(b.paste_margin, 6),
                    b.paste_margin_ratio) and not (
                    a.paste_margin is not None and b.paste_margin is not None
                    and abs(a.paste_margin - b.paste_margin) < TOL
                    and a.paste_margin_ratio == b.paste_margin_ratio):
                bad_fp.append((k, (a.paste_margin, a.paste_margin_ratio),
                               (b.paste_margin, b.paste_margin_ratio)))
            for pa, pb in zip(a.pads, b.pads):
                ma, mb = pa.paste_margin, pb.paste_margin
                ra, rb = pa.paste_margin_ratio, pb.paste_margin_ratio
                same_m = (ma is None and mb is None) or (
                    ma is not None and mb is not None and abs(ma - mb) < TOL)
                same_r = (ra is None and rb is None) or (
                    ra is not None and rb is not None and abs(ra - rb) < 1e-9)
                if not (same_m and same_r):
                    bad_pad.append((k, pa.pad_number, (ma, ra), (mb, rb)))
        check(not bad_fp, '%s: footprint paste overrides agree' % name, str(bad_fp[:3]))
        check(not bad_pad, '%s: pad paste overrides agree' % name, str(bad_pad[:3]))

        ka = [identity_key(a, f.nets) for a in f.paste_apertures]
        kb = [identity_key(a, g.nets) for a in g.paste_apertures]
        oa, ob = multiset_diff(ka, kb)
        check(not oa and not ob,
              '%s: paste apertures agree by identity (%d)' % (name, len(ka)),
              'only text=%s only pcbnew=%s' % (oa, ob))
        worst, where = bounds_delta(f, g)
        check(worst <= BOUNDS_TOL,
              '%s: aperture bounds agree within %.3f mm (worst %.4f)'
              % (name, BOUNDS_TOL, worst), where)

        # Which vias each opening concerns: the answer the keep-out and the
        # checker act on. Compared by net NAME, since net ids are per-path.
        import paste_apertures as _pa

        def assoc(P):
            out = []
            for a in P.paste_apertures:
                names = sorted(P.nets[n].name for n in _pa.aperture_nets(P, a)
                               if n in P.nets)
                out.append((identity_key(a, P.nets), tuple(names)))
            return out
        oa, ob = multiset_diff(assoc(f), assoc(g))
        check(not oa and not ob, '%s: every opening concerns the same nets' % name,
              'only text=%s only pcbnew=%s' % (oa[:2], ob[:2]))
        if name.startswith('syn_'):
            mz = [a.margin for a in f.paste_apertures if a.pad_number == 'Z']
            mc = [a.margin for a in f.paste_apertures if a.pad_number == 'C']
            want_z = (0.05 - 0.1 * 1.0, 0.05 - 0.1 * 0.5) if name.endswith('20240108') else (0.0, 0.0)
            if (mz and all(abs(a - b) < 1e-6 for a, b in zip(mz[0], want_z))
                    and mc and all(abs(v + 0.10) < 1e-6 for v in mc[0])):
                witnessed['synthetic custom-anchor + explicit-zero'] += 1
        if name == 'watchy':
            panes = [a for a in f.paste_apertures
                     if a.owner_ref == 'U4' and a.source == 'paste_only_pad']
            if panes and all(_pa.aperture_nets(f, a) for a in panes):
                witnessed['watchy U4 windowpanes concern a net'] += 1

        ga = [graphic_key(s) for s in f.segments if s.graphic]
        gb = [graphic_key(s) for s in g.segments if s.graphic]
        oa, ob = multiset_diff(ga, gb)
        check(not oa and not ob,
              '%s: graphic-copper drawn_width/kind/circle/filled agree (%d)' % (name, len(ga)),
              'only text=%s only pcbnew=%s' % (oa, ob))
        witnessed['a FILLED graphic-copper segment'] += sum(1 for s in f.segments
                                                           if s.graphic and s.graphic_filled)
        ua = [(u['owner_ref'], u['kind']) for u in f.graphic_copper_unmeasured]
        ub = [(u['owner_ref'], u['kind']) for u in g.graphic_copper_unmeasured]
        oa, ob = multiset_diff(ua, ub)
        check(not oa and not ob,
              '%s: unmeasured graphic copper agrees (%d)' % (name, len(ua)),
              'only text=%s only pcbnew=%s' % (oa, ob))
        witnessed['an unmeasured-copper entry'] += len(ua)
        if name == 'syn_copper':
            # the text is unmeasured on BOTH paths, the hidden one on neither
            if ua == [('T1', 'text')] and ub == [('T1', 'text')]:
                witnessed['syn_copper: visible copper text only'] += 1
            fill = {}
            for s in f.segments:
                if s.graphic and s.owner_ref == 'T2':
                    fill.setdefault(s.graphic_kind, set()).add(bool(s.graphic_filled))
            # rect: one unfilled (fill none, 0.2) + one filled (no token, 0);
            # circle filled by token; poly filled by the no-token rule
            if fill == {'rect': {True, False}, 'circle': {True}, 'poly': {True}}:
                witnessed['syn_copper: fill by token and by the no-token rule'] += 1

        # -- native oracle: every pad opening's margin is KiCad's own --------
        live = {}
        for fp, key in zip(board.GetFootprints(), list(g.footprints)):
            live[key] = list(fp.Pads())
        wrong = []
        n_pad_aps = 0
        for ap in f.paste_apertures:
            if ap.source not in ('pad', 'paste_only_pad'):
                continue
            fp = f.footprints.get(ap.owner_ref)
            if fp is None or ap.owner_ref not in live:
                continue
            # match by pad number; numbers repeat on some parts, so every live
            # pad carrying it is a candidate and one must agree
            cands = [lp for lp in live[ap.owner_ref]
                     if lp.GetNumber() == ap.pad_number]
            src = [p for p in fp.pads if p.pad_number == ap.pad_number]
            if not cands or not src:
                continue
            n_pad_aps += 1
            lid = pcbnew.F_Paste if ap.layer == 'F.Paste' else pcbnew.B_Paste
            natives = set()
            for lp in cands:
                if not lp.IsOnLayer(lid):
                    continue
                v = lp.GetSolderPasteMargin(lid)
                nx, ny = v.x / 1e6, v.y / 1e6
                if ap.source == 'paste_only_pad':
                    # KiCad resolves no margin for a copper-less pad (its shape
                    # IS the opening). Read what pcbnew says rather than assume it.
                    pass
                elif src[0].shape != 'custom':
                    # KiCad's margin is in the PAD frame. The parser swapped
                    # size_x/size_y only when it baked a ~90-degree rotation into
                    # them, so detect THAT swap from the sizes. A 45-degree pad
                    # keeps pad-frame sizes plus a residual rect_rotation.
                    ns = lp.GetSize()
                    sx, sy = ns.x / 1e6, ns.y / 1e6
                    p0 = src[0]
                    if (abs(sx - sy) > 1e-6 and abs(p0.size_x - sy) < 1e-6
                            and abs(p0.size_y - sx) < 1e-6):
                        nx, ny = ny, nx
                # A custom pad's margin stays in the PAD frame, as KiCad
                # reports it, so it is compared unswapped.
                natives.add((round(nx, 6), round(ny, 6)))
            mine = (round(ap.margin[0], 6), round(ap.margin[1], 6))
            # KiCad rounds `size * ratio` to whole nanometres (KiROUND); allow 1 nm.
            if not any(abs(mine[0] - n[0]) <= 1.5e-6 and abs(mine[1] - n[1]) <= 1.5e-6
                       for n in natives):
                wrong.append((ap.label(), mine, sorted(natives)))
        check(not wrong,
              '%s: every pad opening margin equals pcbnew GetSolderPasteMargin (%d)'
              % (name, n_pad_aps), str(wrong[:3]))

        if name == 'esp_prog' and any(a.owner_ref == 'U2' and a.source == 'graphic'
                                      for a in f.paste_apertures):
            witnessed['esp_prog U2 F.Paste graphic'] += 1
        if name == 'glasgow_revC' and any(a.owner_ref == 'J1' and a.source == 'graphic'
                                          for a in f.paste_apertures):
            witnessed['glasgow J1 pin-in-paste graphics'] += 1
        if name == 'ulx3s' and any(p.paste_margin_ratio is not None
                                   and abs(p.paste_margin_ratio + 0.2) < 1e-9
                                   for fp in f.footprints.values() for p in fp.pads):
            witnessed['ulx3s pad ratio -0.2'] += 1

    for w in ('esp_prog U2 F.Paste graphic', 'glasgow J1 pin-in-paste graphics',
              'ulx3s pad ratio -0.2', 'watchy U4 windowpanes concern a net',
              'syn_copper: visible copper text only',
              'syn_copper: fill by token and by the no-token rule'):
        check(witnessed[w] == 1, 'witness present: %s' % w,
              'a named witness vanished; the parity arms above could then pass '
              'on an empty model')
    # -- GUI WRITE side (#962): apply_via_protection must reach the saved
    # board. The shipping 10.0.0 SWIG exports no *_MODE_* constants (#751),
    # so it can only get there through the bool flag setters.
    sys.path.insert(0, os.path.join(REPO, 'kicad_routing_plugin'))
    try:
        from gui_utils import apply_via_protection
        from fab_notes import TYPE_VII_STAMP
        import re as _re
        wb = pcbnew.LoadBoard(os.path.join(REPO, 'kicad_files', 'esp_prog.kicad_pcb'))
        for x, spec in ((10, TYPE_VII_STAMP), (12, {})):
            v = pcbnew.PCB_VIA(wb)
            v.SetPosition(pcbnew.VECTOR2I(pcbnew.FromMM(x), pcbnew.FromMM(10)))
            v.SetWidth(pcbnew.FromMM(0.6))
            v.SetDrill(pcbnew.FromMM(0.3))
            apply_via_protection(v, spec)
            wb.Add(v)
        wp = os.path.join(syn_dir, 'write_side.kicad_pcb')
        pcbnew.SaveBoard(wp, wb)
        txt = open(wp, encoding='utf-8').read()

        def blk(x):
            k = txt.find('(at %d 10)' % x)
            return txt[txt.rfind('(via', 0, k):txt.find('(uuid', k)]
        # NOTE this gate's check() takes (cond, what) -- the reverse of the
        # wx-free tests' -- and a swapped call passes on a truthy string.
        check('(capping yes)' in blk(10) and '(filling yes)' in blk(10),
              'GUI write side: a TYPE_VII spec saves (capping yes) (filling yes)', blk(10))
        check(not _re.search(r'\((capping|filling|tenting|covering|plugging)', blk(12)),
              'GUI write side: an EMPTY spec saves no protection token (inherits)',
              blk(12))
    except Exception as e:
        check(False, 'GUI write side ran', '%s: %s' % (type(e).__name__, e))

    check(witnessed['synthetic custom-anchor + explicit-zero'] == 2,
          'witness: both synthetic boards resolve the anchor ratio and the '
          'version-dependent zero as KiCad does',
          str(witnessed['synthetic custom-anchor + explicit-zero']))

    print('\n%d failure(s)' % len(FAILURES))
    for fl in FAILURES:
        print('  FAILED: %s' % fl)
    return 1 if FAILURES else 0


if __name__ == '__main__':
    sys.exit(main())
