#!/usr/bin/env python3
"""replan.py -- the braid's VERDICT re-plans the ends (#622, 2026-09-10).

The plan's model cannot price a swimmer at K51 (rank agreement 0.2-0.4
with the braid over 16 routed boards), so every chooser judged by it
lost at the braid. This driver uses the ROUTE ITSELF as the judge and
the PREVIOUS route as the price. Round r, on a fanout board F (both ends
laid, its plan sidecar beside it) and its routed board R:

  1. VERDICT off R: per net its class in the braid's own schedule (page
     F / page B / swimmer), its real vias, refused and the boxed end
     (`walled_at`), its launch and target rank. The plan model's
     prediction on the plan F carries is recomputed with the residual
     off, and real - predicted per net becomes `plan_ends.RESIDUAL`:
     the learned price every judge below runs with.
  2. WORST: the refused nets, then the swimmers by real vias.
  3. CANDIDATES per worst net at each end the verdict allows (a lane
     boxed at its tooth moves its tooth; a swimmer may move either): the
     destination menu on the bare array (a class not banned and not the
     current one, its lane free of the other berths), the source menu
     with CLIMBS (`SRC_CLIMB`, the honest menu: legs clear of every other
     net's copper); each ranked by the plan's judged cost with every
     other net fixed at what the board carries.
  4. PROBE each candidate with the real router on R: the net stripped
     to its tooth, the asked end re-fanned by the production engine
     against everything else's copper (audited: face / gap / layer /
     kind), the net braided ALONE, the board graded whole. A candidate
     that grades better than R stands (opens first, then vias, no DRC).
  5. APPLY the standing candidates to F: source moves realized
     (`source_realize`, audited), berths re-fanned incrementally
     (`fanout_once`, audited), the UNMOVED teeth and berths checked
     unchanged from the board, the sidecar rewritten from the board it
     sits beside; then the BRAID on the new F, graded, its verdict read.
     Kept iff every move was laid in its asked class AND the board
     grades better; else the moves tried are banned and F stays.

Nothing here reads a face, a ref or a board name: the classes come off
the board, the candidates from the menus, the verdicts from the engines.

usage: replan.py TAG K [--board=BASE] [--dest=REF] [--rounds=4] [--worst=3]
                       [--climb=8] [--probes=1] [--min-vias=3] [--out=TAG]
  reads  tmp/TAG_fo_kK.kicad_pcb (+ .plan.json), tmp/TAG_kK.kicad_pcb
         (+ .log, .pack.json, _refusals.json)
  writes tmp/OUT_rp_kK_r<N>_* per round, tmp/OUT_rp_kK.kicad_pcb (best)
"""
import contextlib
import hashlib
import io
import json
import math
import os
import re
import shutil
import subprocess
import sys
import time
from collections import Counter

ARGV = [a for a in sys.argv[1:] if not a.startswith('--')]
OPTS = dict((a[2:].split('=', 1) + ['1'])[:2] for a in sys.argv[1:] if a.startswith('--'))
# --length=1: a probe that ties the board's vias and shortens the run's
# copper stands (the K28 question: every diver costs exactly 2 vias
# whatever its berth, so a walked berth can only pay in copper).
LENGTH_TIE = OPTS.get('length', '0') == '1'
# --apply=strip (2026-09-10): the round's fanout board is DERIVED from the
# probes' routed board -- every net stripped to the fanout copper of the
# board that last laid its ends (the probe's re-fanned board for a net it
# fanned or re-laid, else the round's fanout board) -- instead of being
# re-fanned from the standing asks. The re-fan lost K35's round: the probes'
# incremental board graded 0 open / 0 DRC / 52 vias (base 61), the re-fanned
# board laid two neighbours differently (SODT0 drifted, SA6's class), so the
# full braid had to decide and came back at 60. Derived, the ends agree by
# construction and the incremental board ships. Destination moves only.
# DEFAULT FLIPPED TO strip (2026-09-12). `refan` was the default and is
# UNFAITHFUL: the probe verifies one move and the apply lays another. Seen
# in tmp/rp1_k51.out, where both applied moves that round disagreed with
# what was probed --
#     SA11 berth: asked surface/right/F exit=(146.33,62.56) v=0,
#          got via_in_pad/right/B ... LAYER F->B, KIND surface->via_in_pad,
#          GAP off 5.50mm
# -- so the thing GRADED is not the thing BUILT, and probing harder buys
# nothing. That matters more than it used to: route-in-the-loop is the one
# approach that makes "more running" monotone by construction, and it is
# worthless on an apply path that does not lay what it promised.
# The evidence for strip was already in the comment above (refan lost
# K35's round, 52 probed -> 60 re-fanned) and every "best measured" board
# in the README came from a strip run; meanwhile all 22 replan runs on
# disk used the refan default. `--apply=refan` remains the opt-out.
APPLY_STRIP = OPTS.get('apply', 'strip') == 'strip'
# --coupled=census: the probe's re-lay set is the end's own conflicts plus
# the braid's blocker census (the recorded behaviour; the only value).
COUPLED = OPTS.get('coupled', 'census')
# --perturb=N (2026-09-18): a NEAR JUMP. N random nets moved to a random
# other class each, through the same probes a descent uses, the probe
# board accepted whatever its grade (a jump does not care where it
# lands; the descent from there is the point). One round; the world it
# writes is a replan stem like any descent's. Measured need: a jump by a
# full chain (re-solve, fan out, braid twice) cost 665 s and landed at
# 84..141 at K51, never descending below 88; a jump of one probe lands
# a move away.
PERTURB = int(OPTS.get('perturb', 0) or 0)
PERTURB_TRIES = int(OPTS.get('perturb-tries', 3))   # candidates probed per perturbed net, at most
PERTURB_SEED = int(OPTS.get('seed', 1))
# --cross=STEM_B (2026-09-19): a CROSSOVER through the probes. The run's
# --from world is parent A; for a random --cross-frac of the nets whose
# ends differ between A and B, B's end(s) are asked for on A's routed
# board through the descent's own probes (both ends together when both
# differ, then each alone), the probe board taken whatever its grade.
# Measured need: population members differ in 2-10 of 41 nets at K41, and
# the chain crossover (re-solve, fan out, braid) cost 214 s and landed at
# 96 from parents at 64 and 66.
CROSS = OPTS.get('cross')
CROSS_FRAC = float(OPTS.get('cross-frac', 0.5))
CROSS_TOL = 0.3     # mm: an end within this of the other parent's, same class, is the same end


def _dban(m):
    """The destination ban key of a move: its class."""
    return (m.direction, m.layer)
# the source menu offers climbs (fanout_from_plan reads this at import)
os.environ.setdefault('SRC_CLIMB', OPTS.get('climb', '14'))

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
from kicad_parser import parse_kicad_pcb  # noqa: E402
import fanout_from_plan as fp  # noqa: E402
import source_realize as sr  # noqa: E402
import plan_ends as pe  # noqa: E402
import braid as te  # noqa: E402
import escape_moves as em  # noqa: E402
from coherent_nets import coherent_nets  # noqa: E402
import rules as _rules  # noqa: E402  ONE source for every design rule
import probe_memo as pm  # noqa: E402  the probe / screen memo (never pay for one twice)

from escape_moves import LAYERS  # noqa: E402,F401  -- ONE source


_ECO = re.compile(r'\s*\(gr_line(?:[^()]|\([^()]*\))*\)')


def strip_eco(txt):
    """The braid's Eco overlay (planned centrelines, end marks) off a board
    text, so successive one-net runs do not stack drawings."""
    return _ECO.sub(lambda m: '' if 'Eco1.User' in m.group(0)
                    or 'Eco2.User' in m.group(0) else m.group(0), txt)


def copy_board(src, dst, eco=False):
    if eco:
        shutil.copy(src, dst)
    else:
        with open(src, encoding='utf-8') as f:
            txt = f.read()
        with open(dst, 'w', encoding='utf-8') as f:
            f.write(strip_eco(txt))
    fp.copy_pro(src, dst)


def source_view(fo, out, names, byname, dref, pad_mm=2.0):
    """The fanout board with the run's DESTINATION copper stripped (a
    window round the array, as fanout_once strips for a re-lay): the board
    the plan's state is read from."""
    pcb = parse_kicad_pcb(fo)
    n2n = {i: n.name for i, n in pcb.nets.items()}
    x0, y0, x1, y1 = em.grid_of(pcb.footprints[dref]).bbox
    x0, y0, x1, y1 = x0 - pad_mm, y0 - pad_mm, x1 + pad_mm, y1 + pad_mm
    nids = {byname[nm][0] for nm in names}
    segs = [s for s in pcb.segments if s.net_id in nids
            and x0 <= min(s.start_x, s.end_x) and max(s.start_x, s.end_x) <= x1
            and y0 <= min(s.start_y, s.end_y) and max(s.start_y, s.end_y) <= y1]
    vias = [v for v in pcb.vias if v.net_id in nids
            and x0 <= v.x <= x1 and y0 <= v.y <= y1]
    content = open(fo, encoding='utf-8').read()
    content, n_s = sr.remove_segments_from_content(content, segs, n2n)
    content, n_v = sr.remove_vias_from_content(content, vias, n2n)
    if n_s != len(segs) or n_v != len(vias):
        print(f'  source view: WARNING strip matched {n_s}/{len(segs)} segments, '
              f'{n_v}/{len(vias)} vias')
    with open(out, 'w', encoding='utf-8') as f:
        f.write(content)
    fp.copy_pro(fo, out)


# --grade=inproc (2026-09-18): the three checks of grade_k run IN THIS
# PROCESS instead of as four python launches. Measured on a K51 probe: the
# subprocess grade is ~3 s of a ~9 s probe, and the checks themselves are
# 1.5 s of that. Same checkers, same clearance, same regexes on the same
# printed verdicts -- only the process boundary is gone. 'sub' (default) is
# the recorded path.
GRADE_MODE = OPTS.get('grade', 'sub')


def _grade_inproc(board, nets):
    import contextlib
    import io
    import runpy
    sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
    import check_drc as _cd
    buf = io.StringIO()
    argv = sys.argv
    try:
        sys.argv = ['check_connected.py', board]
        with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(buf):
            try:
                runpy.run_path(os.path.join(HERE, '..', 'py_router', 'check_connected.py'), run_name='__main__')
            except SystemExit:
                pass
    finally:
        sys.argv = argv
    cc = buf.getvalue()
    if not re.search(r'ALL NETS FULLY CONNECTED|FOUND \d+ ISSUE|\d+ net\(s\) with issues|unconnected', cc, re.I):
        return None, 'BROKEN: check_connected did not report'
    opens = []
    for line in cc.splitlines():
        m = re.search(r'(\S+) \(net \d+\):', line)
        if m and m.group(1).split('/')[-1] in nets:
            opens.append(m.group(1).split('/')[-1])
        m2 = re.match(r'\s+(\S+) \(\d+ pads?\)\s*$', line)
        if m2 and m2.group(1).split('/')[-1] in nets:
            opens.append(m2.group(1).split('/')[-1])
    clr = _rules.active().clearance
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(buf):
        try:
            _cd.run_drc(board, clearance=clr, clearance_margin=0.1, max_print=0)
        except SystemExit:
            pass
    dd = buf.getvalue()
    m = re.search(r'FOUND (\d+) DRC VIOLATIONS', dd)
    if m is None and 'NO DRC VIOLATIONS' not in dd:
        return None, 'BROKEN: check_drc reported no verdict'
    ndrc = int(m.group(1)) if m else 0
    pcb_ = parse_kicad_pcb(board)
    ids = {i for i, n in pcb_.nets.items() if n.name.split('/')[-1] in set(nets)}
    nvias = sum(1 for v in pcb_.vias if v.net_id in ids)
    line = (f'GRADE {os.path.basename(board)} K={len(nets)} clr={clr} open={len(opens)} drc={ndrc} vias={nvias}'
            + (f'  open: {",".join(sorted(opens))}' if opens else ''))
    return [sorted(opens), ndrc, nvias, None], line


def _grade_scoped(board, nets, scope, ref):
    """The grade of a board on which only the `scope` nets changed, from
    a board graded `ref` = (opens, drc, vias, ...) that was DRC-clean:
    every new violation involves changed copper, so the DRC runs over
    the scope's nets against everything; the scope's connectivity is
    checked and the other nets keep the reference's opens; the vias are
    counted off the board. Same answer as the whole-board grade, at a
    fraction of the work (measured: 1.8 s of a 6.6 s probe)."""
    import contextlib
    import io
    sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
    import check_drc as _cd
    import check_connected as _cc
    scope = sorted(set(scope))
    pats = [f'*/{n}' for n in scope] + list(scope)
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(buf):
        issues = _cc.run_connectivity_check(board, pats, quiet=True, pcb_data=parsed(board))
    if any(i.get('scope_error') for i in issues):
        return None, 'BROKEN: scoped connectivity check selected nothing'
    nets_set = set(nets)
    opens = {i['net_name'].split('/')[-1] for i in issues if i.get('net_name')} & nets_set
    opens |= set(ref[0]) - set(scope)
    clr = _rules.active().clearance
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(buf):
        try:
            _cd.run_drc(board, clearance=clr, clearance_margin=0.1, max_print=0, net_patterns=pats,
                        pcb_data=parsed(board))
        except SystemExit:
            pass
    dd = buf.getvalue()
    m = re.search(r'FOUND (\d+) DRC VIOLATIONS', dd)
    if m is None and 'NO DRC VIOLATIONS' not in dd:
        return None, 'BROKEN: scoped check_drc reported no verdict'
    ndrc = int(m.group(1)) if m else 0
    pcb_ = parsed(board)
    ids = {i for i, n in pcb_.nets.items() if n.name.split('/')[-1] in nets_set}
    nvias = sum(1 for v in pcb_.vias if v.net_id in ids)
    line = (f'GRADE {os.path.basename(board)} K={len(nets)} clr={clr} open={len(opens)} drc={ndrc} vias={nvias}'
            f' [scoped to {len(scope)} net(s)]' + (f'  open: {",".join(sorted(opens))}' if opens else ''))
    return [sorted(opens), ndrc, nvias, None], line


def grade(board, K, base, scope=None, ref=None):
    """(opens, drc, vias) of the run's nets on `board`, by grade_k -- or,
    with `scope` (the nets that changed) and `ref` (the DRC-clean grade
    of the board they changed from), by the scoped checks."""
    if GRADE_MODE == 'inproc' and scope and ref is not None and ref[1] == 0:
        g, line = _grade_scoped(board, coherent_nets(K, base), scope, ref)
        if g is not None:
            return g, line
    if GRADE_MODE == 'inproc':
        g, line = _grade_inproc(board, coherent_nets(K, base))
        if g is None:
            return [None, None, None, None], line
        return g, line
    r = subprocess.run([sys.executable, os.path.join(HERE, 'grade_k.py'), board,
                        ','.join(coherent_nets(K, base))], capture_output=True, text=True)
    line = next((l for l in (r.stdout + r.stderr).splitlines() if l.startswith('GRADE')), '')
    m = re.search(r'open=(\d+) drc=(\d+) vias=(\d+)', line)
    opens = sorted(line.split('open: ')[1].split(',')) if 'open: ' in line else []
    mm = None
    if m and LENGTH_TIE:
        pcb_ = parse_kicad_pcb(board)
        ids = {i for i, n in pcb_.nets.items() if n.name.split('/')[-1] in set(coherent_nets(K, base))}
        mm = round(sum(math.hypot(sg.end_x - sg.start_x, sg.end_y - sg.start_y)
                       for sg in pcb_.segments if sg.net_id in ids), 1)
    return ([opens, int(m.group(2)), int(m.group(3)), mm] if m else [None, None, None, None]), line


def count_copper(board, nid):
    pcb = parse_kicad_pcb(board)
    return (sum(1 for v in pcb.vias if v.net_id == nid),
            sum(1 for s in pcb.segments if s.net_id == nid))


PAD_MM = 2.0          # the window round an array (fanout_once's re-lay strip)
TOL = 0.002           # mm: a text block is the same copper as a parsed item
_NUM = r'(-?[\d.]+)'


# ---------------------------------------------------------------- text
def dmenu_full(st):
    """The destination menus of a plan state: the full bare-array menu
    (`dmenu` in plan_state as committed; a `dmenu_full` when a pin or a
    force narrows `dmenu`)."""
    return st.get('dmenu_full', st['dmenu'])


def walk_blocks(txt, token, remove):
    """Every top-level `(token ...)` block: `remove(block)` True drops it.
    Paren-balanced, either net dialect (te.strip_net_items's walk)."""
    out, i = [], 0
    while True:
        j = txt.find('(' + token, i)
        while j >= 0 and txt[j + 1 + len(token)] not in ' \n\t':
            j = txt.find('(' + token, j + 1)
        if j < 0:
            out.append(txt[i:])
            break
        k, depth = j, 0
        while True:
            c = txt[k]
            if c == '(':
                depth += 1
            elif c == ')':
                depth -= 1
                if depth == 0:
                    break
            k += 1
        block = txt[j:k + 1]
        if remove(block):
            out.append(txt[i:j].rstrip(' \t'))
            e = k + 1
            if e < len(txt) and txt[e] == '\n':
                e += 1
            i = e
        else:
            out.append(txt[i:k + 1])
            i = k + 1
    return ''.join(out)


def block_net(block, nid, name):
    m = re.search(r'\(net (\d+)\)', block)
    m2 = re.search(r'\(net "([^"]+)"\)', block)
    return (m and int(m.group(1)) == nid) or (m2 and m2.group(1) == name)


def block_geom(block, token):
    if token == 'segment':
        s = re.search(r'\(start ' + _NUM + ' ' + _NUM + r'\)', block)
        e = re.search(r'\(end ' + _NUM + ' ' + _NUM + r'\)', block)
        L = re.search(r'\(layer "?([^")\s]+)"?\)', block)
        return ((float(s.group(1)), float(s.group(2))), (float(e.group(1)), float(e.group(2))), L.group(1))
    a = re.search(r'\(at ' + _NUM + ' ' + _NUM + r'\)', block)
    return ((float(a.group(1)), float(a.group(2))), None, None)


def _near(p, q):
    return abs(p[0] - q[0]) < TOL and abs(p[1] - q[1]) < TOL


def in_box(p, box):
    return box[0] <= p[0] <= box[2] and box[1] <= p[1] <= box[3]


def strip_to_fanout_copper(txt, nm, nid, name, pcb_f, box):
    """`txt` (a routed board) with net `nm` reduced to the copper the
    fanout board `pcb_f` carries for it inside `box` -- its TOOTH: the
    lane and the berth go, the tooth stays as the fanout laid it (the
    braid keeps its input copper verbatim)."""
    segs = [s for s in pcb_f.segments if s.net_id == nid
            and in_box((s.start_x, s.start_y), box) and in_box((s.end_x, s.end_y), box)]
    vias = [v for v in pcb_f.vias if v.net_id == nid and in_box((v.x, v.y), box)]

    def rm_seg(block):
        if not block_net(block, nid, name):
            return False
        a, b, L = block_geom(block, 'segment')
        return not any(s.layer == L and ((_near(a, (s.start_x, s.start_y)) and _near(b, (s.end_x, s.end_y)))
                                         or (_near(a, (s.end_x, s.end_y)) and _near(b, (s.start_x, s.start_y))))
                       for s in segs)

    def rm_via(block):
        if not block_net(block, nid, name):
            return False
        a, _, _ = block_geom(block, 'via')
        return not any(_near(a, (v.x, v.y)) for v in vias)
    txt = walk_blocks(txt, 'segment', rm_seg)
    return walk_blocks(txt, 'via', rm_via)


def salvage_missing_ends(F1, R, names, byname, sbox, dbox, log=print):
    """A derived fanout board with an END MISSING (2026-09-18): a net the
    braid ripped and re-laid FROM THE PAD -- its stub abandoned -- has
    routed copper that matches no fanout board's, so the strip kept
    nothing and the net vanished from F1 (SDQ2 on the cp lineage's round
    2, the 91-via board; `endpoints` then asserts 'no free stub end' and
    the round dies). Its de facto stub is the lane's pad-exit copper: the
    routed segments and vias of that net inside the array window are
    appended to F1 for every end that has none. Returns the nets touched."""
    pcb1 = parse_kicad_pcb(F1)
    fixed = []
    txt = None
    rtxt = None
    for nm in names:
        nid, net = byname[nm]
        segs = [s for s in pcb1.segments if s.net_id == nid]
        for end, box in (('src', sbox), ('dst', dbox)):
            if any(in_box((s.start_x, s.start_y), box) or in_box((s.end_x, s.end_y), box) for s in segs):
                continue
            if rtxt is None:
                rtxt = strip_eco(open(R, encoding='utf-8').read())
                txt = open(F1, encoding='utf-8').read()
            keep = []

            def _grab(block, _keep=keep, _box=box, _nid=nid, _name=net.name):
                if not block_net(block, _nid, _name):
                    return False
                tok = 'segment' if block.startswith('(segment') else 'via'
                a, b, _L = block_geom(block, tok)
                if tok == 'segment':
                    if in_box(a, _box) and in_box(b, _box):
                        _keep.append(block)
                elif in_box(a, _box):
                    _keep.append(block)
                return False
            walk_blocks(rtxt, 'segment', _grab)
            walk_blocks(rtxt, 'via', _grab)
            if keep:
                i = txt.rstrip().rfind(')')
                txt = txt[:i] + ''.join('  ' + b + '\n' for b in keep) + txt[i:]
                fixed.append(f'{nm}.{end} ({len(keep)} block(s) of pad-exit copper)')
    if txt is not None and fixed:
        with open(F1, 'w', encoding='utf-8') as f:
            f.write(txt)
        log(f'  derived board: ends salvaged from the routed board -- {fixed}')
    return fixed


def strip_window(txt, nets, byname, box):
    """`txt` with those nets' copper inside `box` removed (a segment
    wholly inside, a via inside): fanout_once's re-lay strip, as text."""
    ids = {byname[nm][0]: byname[nm][1].name for nm in nets}

    def rm(token):
        def f(block):
            m = re.search(r'\(net (\d+)\)', block)
            m2 = re.search(r'\(net "([^"]+)"\)', block)
            nid = int(m.group(1)) if m else next((i for i, n in ids.items() if m2 and n == m2.group(1)), None)
            if nid not in ids:
                return False
            a, b, _ = block_geom(block, token)
            return in_box(a, box) and (b is None or in_box(b, box))
        return f
    txt = walk_blocks(txt, 'segment', rm('segment'))
    return walk_blocks(txt, 'via', rm('via'))


def write_board(txt, out, pro_from):
    with open(out, 'w', encoding='utf-8') as f:
        f.write(txt)
    fp.copy_pro(pro_from, out)


# ------------------------------------------------------------- verdict
def _names(s):
    return [x.strip().strip("'") for x in s.split(',') if x.strip()]


def verdict(stem):
    """What the braid said, per net, off its log, pack sidecar and
    refusal record: class, lane vias, refused + boxed end, ranks."""
    V = {}
    log = open(stem + '.log', encoding='utf-8').read()
    for m in re.finditer(r'^  page F: \[(.*?)\]$', log, re.M):
        for n in _names(m.group(1)):
            V.setdefault(n, {})['cls'] = 'F'
    for m in re.finditer(r'^  page B: \[(.*?)\]\s+swimmers: \[(.*?)\]$', log, re.M):
        for n in _names(m.group(1)):
            V.setdefault(n, {})['cls'] = 'B'
        for n in _names(m.group(2)):
            V.setdefault(n, {})['cls'] = 'swim'
    lo = re.findall(r'^  launch order: \[(.*?)\]$', log, re.M)
    to = re.findall(r'^  target order: \[(.*?)\]', log, re.M)
    for l, t in zip(lo, to):
        for i, n in enumerate(_names(l)):
            V.setdefault(n, {})['lrank'] = i
        for i, n in enumerate(_names(t)):
            V.setdefault(n, {})['trank'] = i
    for m in re.finditer(r'last call routed: (\S+) \((\d+) via', log):
        V.setdefault(m.group(1), {})['last_call'] = int(m.group(2))
    for m in re.finditer(r"rip \[(.*?)\]: (\S+) routed \((\d+) via", log):
        V.setdefault(m.group(2), {})['rip'] = _names(m.group(1))
    # THE BRAID'S OWN CENSUS of what walls a lane: the lanes of the run on
    # its blocked frontier (cells each), and the min-cut probe's crossing
    # set -- the good nets in the way of this bad one, by the real router
    for m in re.finditer(r'rip for (\S+): frontier \d+ cells; lanes of this run on it: (.*?)\s+\(', log):
        fr = re.findall(r'([A-Za-z0-9_]+)\((\d+)\)', m.group(2))
        V.setdefault(m.group(1), {})['frontier'] = [(n, int(c)) for n, c in fr]
    for m in re.finditer(r'rip for (\S+): min-cut probe \d+ via\(s\) crosses \[(.*?)\]', log):
        V.setdefault(m.group(1), {})['mincut'] = _names(m.group(2))
    pk = json.load(open(stem + '.pack.json'))
    for n, l in pk['lanes'].items():
        V.setdefault(n, {})['lane_vias'] = len(l['vias'])
        V[n]['lane_mm'] = round(sum(math.hypot(s[2] - s[0], s[3] - s[1]) for s in l['segs']), 1)
    rp = stem + '_refusals.json'
    if os.path.exists(rp):
        for n, info in json.load(open(rp)).items():
            V.setdefault(n, {}).update(refused=True, walled_at=info.get('walled_at'),
                                       pocket=info.get('pocket_mm'))
    # the census carried from earlier rounds and runs (replan writes it
    # beside every board it keeps): an incremental board's own log names
    # only the nets its local braid re-laid
    cp = stem + '.census.json'
    if os.path.exists(cp):
        for n, bl in json.load(open(cp)).items():
            V.setdefault(n, {}).setdefault('census_prev', bl)
    return V


def fmt_v(nm, v, real):
    return (f'{nm:6s} {v.get("cls", "?"):4s} L{v.get("lrank", "?")!s:>3}->T{v.get("trank", "?")!s:<3} '
            f'real {real!s:>2} lane {v.get("lane_vias", 0)}'
            + (f' REFUSED walled at the {v.get("walled_at")}' if v.get('refused') else '')
            + (f' last-call {v["last_call"]}v' if 'last_call' in v else '')
            + (f' rip {v["rip"]}' if 'rip' in v else ''))


# --------------------------------------------------------------- state
class Board:
    """A fanout board F with its plan state read off its source view,
    both ends measured, the menus, the current moves."""

    def __init__(self, F, names, dref, banned=frozenset(), R=None):
        self.F = F
        stem = F[:-len('.kicad_pcb')]
        pcb = parse_kicad_pcb(F)
        self.pcb = pcb
        byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
        self.sv = stem + '_srcview.kicad_pcb'
        source_view(F, self.sv, names, byname, dref)
        self.st = st = fp.plan_state(parse_kicad_pcb(self.sv), names, banned)
        self.byname = st['byname']
        self.names = names
        self.ends = {}
        for nm in names:
            self.ends[nm] = {
                'dst': sr.measure_tooth(pcb, nm, st['dst_pad'][nm], self.byname, dest_ref=st['dref']),
                'src': sr.measure_tooth(pcb, nm, st['src_pad'][nm], self.byname, dest_ref=st['dref'], which='src')}
        self.cur_dst = {nm: fp._menu_match(dmenu_full(st)[nm], self.ends[nm]['dst']) for nm in names}
        self.cur_src = {nm: fp._menu_match(st['smenu'].get(nm, []), self.ends[nm]['src']) for nm in names}
        self.choice = {nm: m for nm, m in self.cur_dst.items() if m is not None}
        self.pads = {nm: (st['dst_pad'][nm].global_x, st['dst_pad'][nm].global_y) for nm in names}
        self.lanes = lane_items(parse_kicad_pcb(R), pcb, names, self.byname) if R else {}
        self.berths = berth_items(pcb, names, self.byname, _pad(st['dgrid'].bbox))

    def cls(self, nm, end):
        g = self.ends[nm][end]
        return (g['direction'], g['layer']) if g else None

    def advance(self, nm, pr):
        """After a standing probe: the moved net's ends (and the co-moved
        berths') as the engine LAID them, so the next net's candidates are
        ranked against the board as it stands -- measured: SBA2's climb
        was asked to the exit SA8's new tooth had just taken."""
        got = {}
        if pr.get('src_got'):
            got[(nm, 'src')] = pr['src_got']
        if pr.get('dst_got'):
            got[(nm, 'dst')] = pr['dst_got']
        for o, g in (pr.get('comove_got') or {}).items():
            if g:
                got[(o, 'dst')] = g
        for (o, end), g in got.items():
            self.ends[o][end] = g
            if end == 'dst':
                self.cur_dst[o] = fp._menu_match(dmenu_full(self.st)[o], g)
                if self.cur_dst[o] is not None:
                    self.choice[o] = self.cur_dst[o]
                else:
                    self.choice.pop(o, None)
            else:
                self.cur_src[o] = fp._menu_match(self.st['smenu'].get(o, []), g)
                self.st['launch'][o] = tuple(g['tooth'])
                self.st['tooth0'][o] = g['layer']
                self.st['tooth_vias'][o] = g['vias']

    def dmenu_honest(self, nm):
        """Every berth move `nm` has on THIS fanout board with the other
        berths as copper (its own excluded): the plan's menu is drawn on a
        bare array, and a site or run the other berths' vias block is one
        the engine will not lay (SA6: 'exact move infeasible even alone'
        for a dogbone the bare menu offered)."""
        import escape_moves as em
        nid = self.byname[nm][0]
        obs = {L: te.build_obstacles(self.pcb, nid, {nid}, L) for L in LAYERS}
        return em.enumerate_moves(
            self.st['dst_pad'][nm], self.st['dgrid'], LAYERS,
            lambda p, q, L: obs[L].seg_clear(p, q),
            lambda p, L: not (obs[L].point_violation(
                p, pad=(te.VIA_SIZE - te.TRACK) / 2) or [0])[0])

    def buses(self):
        return fp.planned_buses(self.st, self.choice)

    def cost(self, choice, launch, tl, tv, buses, cache):
        st = self.st
        return pe.judged_cost(choice, launch, st['dgrid'].bbox, cache, st['sgrid'].bbox,
                              tl, tv, buses, chi=st['chi'])

    def model(self, named):
        """The plan model's prediction per net for the plan this board
        carries (its sidecar's nets), RESIDUAL off: what the sidecar's
        judge said. Returns (pred, bp)."""
        keep, pe.RESIDUAL = pe.RESIDUAL, {}
        try:
            choice = {nm: m for nm, m in self.choice.items() if nm in named}
            achieved = {nm: self.ends[nm]['dst'] for nm in choice}
            _c, pred, bp, _plan = fp.judge_by_braid(self.st, choice, self.F, achieved)
        finally:
            pe.RESIDUAL = keep
        return pred, bp

    def write_sidecar(self, named):
        """The sidecar rewritten from THIS board: the achieved ends and
        layers of every net the previous sidecar named (plus the moved
        ones), as fanout_destination writes it. Returns (pred, bp)."""
        choice = {nm: m for nm, m in self.choice.items() if nm in named}
        achieved = {nm: self.ends[nm]['dst'] for nm in choice}
        keep, pe.RESIDUAL = pe.RESIDUAL, {}
        try:
            _c, pred, bp, plan = fp.judge_by_braid(self.st, choice, self.F, achieved)
        finally:
            pe.RESIDUAL = keep
        side = self.F[:-len('.kicad_pcb')] + '.plan.json'
        with open(side, 'w', encoding='utf-8') as f:
            json.dump(plan, f, indent=1, sort_keys=True)
        return pred, bp, side


# ---------------------------------------------------------- candidates
def rank_dest(B, nm, bans, buses, cache, top):
    """Destination moves for `nm` on the bare array, not banned, not its
    current class, lane free of the other berths as laid; ranked by the
    judged cost with the residual on."""
    st = B.st
    cur = B.cls(nm, 'dst')
    others = {o: m for o, m in B.choice.items() if o != nm}
    pitch = st['dgrid'].pitch_y if True else 0
    pad = st['dst_pad'][nm]

    def depth(m):
        # ball rows (or columns) the move's run crosses on the way out
        ax = 1 if m.direction in ('up', 'down') else 0
        p = st['dgrid'].pitch_y if ax else st['dgrid'].pitch_x
        return abs(m.exit_pt[ax] - (pad.global_y if ax else pad.global_x)) / p
    menu = [m for m in dmenu_full(st)[nm]
            if _dban(m) not in bans and (m.direction, m.layer) != cur
            and not legs_cross(m, others)
            and not (m.kind == 'surface' and depth(m) > MAX_DEPTH + 0.6)]
    out = []
    for m in menu:
        # the berths in the way of this move (the bare menu cannot see
        # them): re-fanned WITH the net, the engine negotiating -- the
        # destination co-move. Ranked after the cost: fewest first.
        N = conflicts(m, B.berths, nm)
        if len(N) > MAX_COMOVE:
            continue
        ch = dict(B.choice)
        ch[nm] = m
        out.append((B.cost(ch, st['launch'], st['tooth0'], st['tooth_vias'], buses, cache), len(N), m, N))
    out.sort(key=lambda t: (t[0], t[1], t[2].vias, t[2].climb))
    # one candidate per class: the cheapest of each (face, layer)
    seen, res = set(), []
    for c, n, m, N in out:
        k = _dban(m)
        if k in seen:
            continue
        seen.add(k)
        m.comove = sorted(N)
        res.append((c, m))
        if len(res) >= top:
            break
    return res, len(menu)


def rank_src(B, nm, buses, cache, top):
    """Source moves for `nm` (the honest menu, climbs included; the
    banned signatures already left it in plan_state), not its current
    tooth; ranked by the judged cost with the residual on."""
    st = B.st
    g = B.ends[nm]['src']
    menu = st['smenu'].get(nm, [])
    if g:
        menu = [m for m in menu if not (m.layer == g['layer']
                                        and abs(m.exit_pt[0] - g['tooth'][0]) < 0.35
                                        and abs(m.exit_pt[1] - g['tooth'][1]) < 0.35)]
    # two teeth cannot share one exit point whatever their layers (the
    # braid orders lanes by their offset at the array): every other
    # tooth's exit as MEASURED, since the menu names only 30 of 47
    others_exit = [B.ends[o]['src']['tooth'] for o in B.names if o != nm and B.ends[o]['src']]
    others_mv = {o: m for o, m in B.cur_src.items() if o != nm and m is not None}
    menu = [m for m in menu
            if all(abs(m.exit_pt[0] - e[0]) >= pe.sm._EXIT_TOL or abs(m.exit_pt[1] - e[1]) >= pe.sm._EXIT_TOL
                   for e in others_exit)]
    out = []
    for m in menu:
        launch = dict(st['launch'])
        launch[nm] = m.exit_pt
        tl = dict(st['tooth0'])
        tl[nm] = m.layer
        tv = dict(st['tooth_vias'])
        tv[nm] = m.vias
        out.append((B.cost(B.choice, launch, tl, tv, buses, cache), m))
    out.sort(key=lambda t: (t[0], t[1].vias, t[1].climb))
    seen, res = set(), []
    for c, m in out:
        k = (m.direction, m.layer, m.kind, m.climb)
        if k in seen:
            continue
        seen.add(k)
        res.append((c, m))
        if len(res) >= top:
            break
    return res, len(menu)


# ------------------------------------------------------- lanes, conflicts
END_AGREE = 0.5       # mm: an end laid to one ask on two boards counts as the same end
MAX_COMOVE = 3        # berths re-fanned with a destination candidate, at most
MAX_DEPTH = 3         # ball rows a SURFACE berth run may cross: deeper ones the
                      # engine's plan-follow does not lay (SA12: a 7-row run on F
                      # asked, 'infeasible even alone', the original re-laid)
TRACK_CLEAR = 0.30    # centre to centre: track + clearance + the engine's grid
VIA_CLEAR = 0.35      # via radius + clearance + half a track + the grid


def _pt_seg_d(p, a, b):
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L2))
    return math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy))


def _seg_seg_d(a, b, c, d):
    if pe.sm._proper_cross(a, b, c, d):
        return 0.0
    return min(_pt_seg_d(a, c, d), _pt_seg_d(b, c, d), _pt_seg_d(c, a, b), _pt_seg_d(d, a, b))


def lane_items(pcb_r, pcb_f, names, byname):
    """Per net, the BRAID's copper on the routed board: its segments and
    vias that the fanout board does not carry (the fanout's own copper
    stays verbatim through the braid)."""
    out = {}
    for nm in names:
        nid = byname[nm][0]
        fs = [s for s in pcb_f.segments if s.net_id == nid]
        fv = [v for v in pcb_f.vias if v.net_id == nid]
        segs = [s for s in pcb_r.segments if s.net_id == nid
                and not any(t.layer == s.layer and
                            ((_near((s.start_x, s.start_y), (t.start_x, t.start_y))
                              and _near((s.end_x, s.end_y), (t.end_x, t.end_y)))
                             or (_near((s.start_x, s.start_y), (t.end_x, t.end_y))
                                 and _near((s.end_x, s.end_y), (t.start_x, t.start_y))))
                            for t in fs)]
        vias = [v for v in pcb_r.vias if v.net_id == nid
                and not any(_near((v.x, v.y), (t.x, t.y)) for t in fv)]
        out[nm] = (segs, vias)
    return out


def conflicts(move, lanes, me):
    """The nets whose LANE copper is in the way of `move`'s legs (same
    layer, within a track's clearance) or of its via site (any layer):
    the lanes that must be re-laid for the engine to lay the move."""
    out = set()
    for nm, (segs, vias) in lanes.items():
        if nm == me:
            continue
        hit = False
        for p, q, L in move.legs:
            for s in segs:
                if s.layer == L and _seg_seg_d(p, q, (s.start_x, s.start_y), (s.end_x, s.end_y)) < TRACK_CLEAR:
                    hit = True
                    break
            if hit:
                break
            for v in vias:
                if _pt_seg_d((v.x, v.y), p, q) < VIA_CLEAR:
                    hit = True
                    break
            if hit:
                break
        if not hit and move.site is not None:
            for s in segs:
                if _pt_seg_d(move.site, (s.start_x, s.start_y), (s.end_x, s.end_y)) < VIA_CLEAR:
                    hit = True
                    break
            if not hit and any(math.hypot(v.x - move.site[0], v.y - move.site[1]) < 2 * VIA_CLEAR for v in vias):
                hit = True
        if hit:
            out.add(nm)
    return out


def berth_items(pcb_f, names, byname, box):
    """Per net, its BERTH copper on the fanout board: segments and vias
    inside the destination window."""
    out = {}
    for nm in names:
        nid = byname[nm][0]
        out[nm] = ([s for s in pcb_f.segments if s.net_id == nid
                    and in_box((s.start_x, s.start_y), box) and in_box((s.end_x, s.end_y), box)],
                   [v for v in pcb_f.vias if v.net_id == nid and in_box((v.x, v.y), box)])
    return out


MEMO_K = None      # set by main: the K whose memo store the probes and screens use


def _store(kind):
    if MEMO_K is None or not pm.ENABLED:
        return None
    st = _store.cache.get(kind)
    if st is None:
        st = _store.cache[kind] = pm.Store(MEMO_K, kind)
    return st


_store.cache = {}


def _board_hash(B):
    """The fanout board's copper as one hash, cached on the Board (its
    copper never changes while the Board lives; engine_lays takes pieces
    off and puts them back)."""
    h = getattr(B, '_copper_hash', None)
    if h is None:
        h = B._copper_hash = pm.copper_hash(B.pcb.segments, B.pcb.vias)
    return h


def engine_lays(B, nm, move, end, others=None):
    """One dry run through engine_lays_many: the screen memo, then the
    pool or here."""
    return engine_lays_many(B, nm, [(move, others)], end)[0]


def _got_tuples(g):
    """A measured end read back from JSON, its points tuples again."""
    if not g:
        return g
    return dict(g, tooth=tuple(g['tooth']), site=(tuple(g['site']) if g.get('site') else None))


def engine_lays_run(B, nm, move, end, others=None):
    """THE ENGINE'S OWN ANSWER to one candidate, in memory: the net's copper
    at that end taken off the parsed fanout board, the production fanout
    asked for the move (with `others` {net: move} laid in the same call,
    a co-move group), nothing written. Returns (achieved dict as
    measure_tooth reports it, exact, in_class, seconds) -- the plan's menu
    is drawn on a model whose clearance is exact geometry and whose array
    is bare; the engine's occupancy grid, keep-outs and deepest-first claim
    order refuse a share of its moves ('OTHER CLASS' in the probes).
    Screening candidates here costs ~1 s each against 10-20 s a probe."""
    from pcb_modification import remove_net_from_pcb_data
    from bga_fanout import generate_bga_fanout
    t0 = time.time()
    st, byname = B.st, B.byname
    pcb = B.pcb
    ref = st['sref'] if end == 'src' else st['dref']
    pads = st['src_pad'] if end == 'src' else st['dst_pad']
    group = {nm: move}
    group.update(others or {})
    box = _pad(st['sgrid'].bbox if end == 'src' else st['dgrid'].bbox)
    removed = {}
    for o in group:
        nid = byname[o][0]
        segs = [s_ for s_ in pcb.segments if s_.net_id == nid
                and in_box((s_.start_x, s_.start_y), box) and in_box((s_.end_x, s_.end_y), box)]
        vias = [v for v in pcb.vias if v.net_id == nid and in_box((v.x, v.y), box)]
        removed[o] = (segs, vias)
        rm_s, rm_v = set(map(id, segs)), set(map(id, vias))
        pcb.segments = [s_ for s_ in pcb.segments if id(s_) not in rm_s]
        pcb.vias = [v for v in pcb.vias if id(v) not in rm_v]
    hints = {}
    for o, m in group.items():
        p = pads[o]
        hints[(round(p.global_x, 3), round(p.global_y, 3))] = sr.full_move(m)
    pcb._fanout_all_foreign_immovable = True
    buf = io.StringIO()
    try:
        with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(buf):
            tracks, vias_add, vias_rm, failed = generate_bga_fanout(
                pcb.footprints[ref], pcb, net_filter=list(group), layers=list(LAYERS),
                track_width=sr.FAN_TRACK, clearance=sr.FAN_CLEAR, via_size=te.VIA_SIZE, via_drill=te.VIA_DRILL,
                exit_margin=0.5, escape_method='underpad', plane_drop='off',
                escape_dir_hints=hints)
    finally:
        for o, (segs, vias) in removed.items():
            pcb.segments.extend(segs)
            pcb.vias.extend(vias)
    nid = byname[nm][0]
    mine = [t for t in tracks if t['net_id'] == nid]
    if not mine:
        return None, False, False, time.time() - t0
    # the laid end: the segment endpoint used once, away from the pad and
    # the vias (te.endpoints' rule), measured as measure_tooth measures
    cnt = Counter()
    for t in mine:
        cnt[(round(t['start'][0], 3), round(t['start'][1], 3))] += 1
        cnt[(round(t['end'][0], 3), round(t['end'][1], 3))] += 1
    p = pads[nm]
    myv = [v for v in vias_add if v.get('net_id') == nid]
    anchors = [(p.global_x, p.global_y, max(p.size_x, p.size_y) / 2)] + \
        [(v['x'], v['y'], te.VIA_SIZE / 2) for v in myv]
    free = [pt for pt, c in cnt.items() if c == 1
            and all(math.hypot(pt[0] - ax, pt[1] - ay) > max(0.02, ar) for ax, ay, ar in anchors)]
    if not free:
        return None, False, False, time.time() - t0
    tip = max(free, key=lambda q: (q[0] - p.global_x) ** 2 + (q[1] - p.global_y) ** 2)
    seg = next(t for t in mine if _near(tip, t['start']) or _near(tip, t['end']))
    g = st['sgrid'] if end == 'src' else st['dgrid']
    x0, y0, x1, y1 = g.bbox
    hx, hy = g.pitch_x / 2, g.pitch_y / 2
    beyond = [f for f, ok in (('right', tip[0] > x1 + hx * 0.5), ('left', tip[0] < x0 - hx * 0.5),
                              ('down', tip[1] > y1 + hy * 0.5), ('up', tip[1] < y0 - hy * 0.5)) if ok]
    face = beyond[0] if beyond else move.direction
    pad_r = max(p.size_x, p.size_y) / 2
    kind = ('via_in_pad' if any(math.hypot(v['x'] - p.global_x, v['y'] - p.global_y) <= pad_r + 0.01 for v in myv)
            else 'dogbone' if myv else 'surface')
    got = {'tooth': (round(tip[0], 3), round(tip[1], 3)), 'layer': seg['layer'], 'vias': len(myv),
           'kind': kind, 'direction': face, 'bearing': face,
           'site': ((round(myv[0]['x'], 3), round(myv[0]['y'], 3)) if myv else None)}
    ax = 0 if move.direction in ('up', 'down') else 1
    gap = abs(got['tooth'][ax] - move.exit_pt[ax])
    in_cls = (face, seg['layer']) == (move.direction, move.layer)
    exact = in_cls and kind == move.kind and gap <= sr.GAP_TOL
    return got, exact, in_cls, time.time() - t0


SCREEN_MAX = 8        # dry runs per net and end, at most


def _end_same(a, b, tol=None):
    """Two measured ends are the same end: both absent, or the same face,
    layer and kind with the tips within `tol` (CROSS_TOL) of each other."""
    if a is None or b is None:
        return a is None and b is None
    if (a['direction'], a['layer'], a['kind']) != (b['direction'], b['layer'], b['kind']):
        return False
    return math.hypot(a['tooth'][0] - b['tooth'][0], a['tooth'][1] - b['tooth'][1]) <= (tol or CROSS_TOL)


def synth_move(nm, got):
    """A Move the engine can be asked for again, from the end it LAID
    (measure_tooth's dict): face, exit, layer, kind, site -- what
    `source_realize.full_move` hands the engine. For an engine substitute
    the plan's menu does not name (SBA2's south-face surface escape, six
    gaps west of any menu move: 109 -> 106 vias)."""
    import escape_moves as em
    if not got or got.get('direction') not in ('left', 'right', 'up', 'down'):
        return None
    return em.Move(nm, got['kind'], got['direction'], got['layer'], tuple(got['tooth']),
                   got.get('vias', 0), [], site=(tuple(got['site']) if got.get('site') else None))


def _others_of(B, m, end):
    return ({o: B.cur_dst[o] for o in (getattr(m, 'comove', []) or []) if B.cur_dst.get(o)}
            if end == 'dst' else None)


def engine_lays_many(B, nm, items, end):
    """The engine's dry runs of several candidates, `items` = [(move,
    others)], each as engine_lays gives it: the screen memo first, the
    rest in the pool side by side when one is up, else here in turn."""
    st = _store('screen')
    out = [None] * len(items)
    miss = []
    for i, (m, others) in enumerate(items):
        key = (pm.key_of({'code': pm.code_hash(), 'knobs': pm.knob_hash(), 'F': _board_hash(B),
                          'net': nm, 'end': end, 'move': sr.move_sig(m),
                          'others': sorted((o, sr.move_sig(om)) for o, om in (others or {}).items())})
               if st is not None else None)
        doc = st.get(key) if st is not None else None
        if doc is not None:
            pm.bump('screen_hit')
            out[i] = (_got_tuples(doc['got']), doc['exact'], doc['in_cls'], 0.0)
            continue
        pm.bump('screen_miss')
        miss.append((i, m, others, key))
    if POOL is not None and miss:
        import concurrent.futures as cf

        def one(t):
            i, m, others, _key = t
            msg = {'op': 'screen', 'net': nm, 'end': end, 'move': move_to_json(m),
                   'others': {o: move_to_json(om) for o, om in (others or {}).items()}}
            try:
                r = POOL.probe(msg)
                return i, (_got_tuples(r['got']), r['exact'], r['in_cls'], r['seconds'])
            except RuntimeError as e:
                sys.stderr.write(f'screen in the pool failed ({e}); run here\n')
                return i, engine_lays_run(B, nm, m, end, others)
        with cf.ThreadPoolExecutor(max_workers=len(POOL.workers)) as ex:
            for i, r in ex.map(one, miss):
                out[i] = r
    else:
        for i, m, others, _key in miss:
            out[i] = engine_lays_run(B, nm, m, end, others)
    if st is not None:
        for i, m, others, key in miss:
            got, exact, in_cls, dt = out[i]
            st.put(key, {'got': got, 'exact': bool(exact), 'in_cls': bool(in_cls), 'seconds': dt})
    return out


def screen(B, nm, ranked, end, top, log=None):
    """The engine's dry run over the model's ranked candidates: the first
    `top` the engine lays as asked (exact or in class) go forward; where it
    lays ANOTHER class, that laid move (menu-matched) goes forward instead
    -- the engine's own realistic version of the ask. Returns [(cost, move)].
    The dry runs are independent: the first SCREEN_MAX candidates run at
    once (engine_lays_many) and are read in rank order with the same
    stopping rule, so the answer is the one-at-a-time answer."""
    out, seen, n_run, n_ok, n_sub, n_no = [], set(), 0, 0, 0, 0
    batch = list(ranked[:SCREEN_MAX])
    lays = engine_lays_many(B, nm, [(m, _others_of(B, m, end)) for c, m in batch], end)
    for (c, m), (got, exact, in_cls, _dt) in zip(batch, lays):
        if len(out) >= top or n_run >= SCREEN_MAX:
            break
        n_run += 1
        if got is None:
            n_no += 1
            continue
        if in_cls:
            n_ok += 1
            key = (m.direction, m.layer, m.kind, round(m.exit_pt[0], 2), round(m.exit_pt[1], 2))
            if key not in seen:
                seen.add(key)
                out.append((c, m))
            continue
        menu_ = B.st['smenu'].get(nm, []) if end == 'src' else dmenu_full(B.st)[nm]
        sub = fp._menu_match(menu_, got) or synth_move(nm, got)
        if sub is None:
            n_no += 1
            continue
        n_sub += 1
        sub.comove = list(getattr(m, 'comove', []) or [])
        key = (sub.direction, sub.layer, sub.kind, round(sub.exit_pt[0], 2), round(sub.exit_pt[1], 2))
        if key not in seen:
            seen.add(key)
            out.append((c, sub))
    if log:
        log(f'    engine screen {nm} {end}: {n_run} dry run(s): {n_ok} as asked, {n_sub} substituted, '
            f'{n_no} not laid -> {len(out)} candidate(s)')
    return out


def legs_cross(m, others):
    """Does a leg of `m` properly cross a leg of another move on the same
    layer? The engine cannot lay that (a surface run across another
    surface run); the selector's non-strict conflict test lets it through
    (measured: SA6's south-face berth asked across the east-face berths'
    row runs, 'exact move infeasible even alone')."""
    for p, q, L in m.legs:
        for om in others.values():
            for r, s_, oL in om.legs:
                if oL == L and pe.sm._proper_cross(p, q, r, s_):
                    return True
    return False


# --------------------------------------------------------------- probe
PROBE_ATTEMPTS = os.environ.get('PROBE_ATTEMPTS', '1')   # the probe braid's attempt ladder
PROBE_BUDGET_X = os.environ.get('PROBE_BUDGET_X', '4')   # its rescue / last-call budget (2 loses nets: measured)
# PROBE_LADDER=open (default, 2026-09-18): a probe braid's rescue ladder starts
# at the rung that opens both layers -- measured over 436 probe braids, lanes
# landed on the two rungs below it in 16 and 16, and every rung costs
# 0.06-0.33 s whether it lands or not; the K41 67 -> 64 descent stands on the
# same two moves (104 -> 95 s) and the K51 null descent gives all 31 verdicts
# identical (63 -> 51 s). PROBE_LADDER=full is the ladder the full braid runs.
PROBE_LADDER = os.environ.get('PROBE_LADDER', 'open')
SCREEN = int(OPTS.get('screen', 1))    # 0: no engine dry runs; the probe's own realize is the screen


def braid_run(board, out_stem, nets, dref, log_to, probe=False):
    t0 = time.time()
    env = dict(os.environ)
    if probe:
        env['BRAID_ATTEMPTS'] = PROBE_ATTEMPTS
        env['BRAID_BUDGET_X'] = PROBE_BUDGET_X
        env['BRAID_SMOOTH'] = os.environ.get('PROBE_SMOOTH', '0')   # the smoother never moves a via
        env['BRAID_LADDER'] = PROBE_LADDER
    r = subprocess.run([sys.executable, '-u', os.path.join(HERE, 'braid.py'),
                        '--board', board, '--dest', dref, '--nets', nets, '--out', out_stem],
                       capture_output=True, text=True, env=env)
    with open(log_to, 'w') as f:
        f.write(r.stdout + r.stderr)
    return os.path.exists(out_stem + '.kicad_pcb'), time.time() - t0


# the board carrying a net's fanout copper (set per round by main under
# --apply=strip: a net re-laid by a later probe is stripped to the copper
# of the board that last laid ITS ends, not the round-start fanout board,
# which no longer holds a berth an earlier stand moved -- measured at K35:
# SA1 lost its new berth to a later probe's strip and was routed to the
# ball, which the derived fanout board then reported as ENDS MISS)
FAN_PCB = None


def coupled_set(B, nm, src_move, dst_move, extra_relay=None):
    """The lanes a probe of this move re-lays with the net: those in the
    way of the new end, the co-moved berths' lanes, the braid's census of
    what walled it, and what the caller names. Returns (C, N)."""
    C = conflicts(src_move if src_move is not None else dst_move, B.lanes, nm)
    N = list(getattr(dst_move, 'comove', []) or [])
    C |= set(N)           # a re-fanned neighbour's lane is re-laid too
    C |= set((getattr(B, 'blockers', {}) or {}).get(nm, []))   # the braid's census: what walled it
    C |= set(extra_relay or [])
    C.discard(nm)
    return C, N


_PARSED = {}


def parsed(path):
    """A parsed board by path, re-read when the file changes."""
    st_ = os.stat(path)
    k = (path, st_.st_mtime_ns, st_.st_size)
    if k not in _PARSED:
        if len(_PARSED) > 24:
            _PARSED.clear()
        _PARSED[k] = parse_kicad_pcb(path)
    return _PARSED[k]


PROBE_FILES = {'rb': '_rb.kicad_pcb', 'rb_pro': '_rb.kicad_pro', 'rb_log': '_rb.log',
               'rb_pack': '_rb.pack.json', 'dst': '_dst.kicad_pcb', 'dst_pro': '_dst.kicad_pro'}
PROBE_FILES_OPT = {'rb_ref': '_rb_refusals.json'}


def probe_key(B, R, nm, src_move, dst_move, K, base, nets_csv, extra_relay=None):
    """What a probe's verdict is a function of: the copper of every net
    outside the coupled set as the routed board carries it, the fanout
    copper the set keeps, the move(s), the co-moves and their asks, the
    set itself, the code and the knobs."""
    C, N = coupled_set(B, nm, src_move, dst_move, extra_relay)
    group = [nm] + sorted(C)
    byname = B.byname
    gids = {byname[c][0] for c in group}
    pcb_r = parsed(R)
    pcb_for = FAN_PCB if FAN_PCB is not None else (lambda _c: B.pcb)
    others = pm.copper_hash([s_ for s_ in pcb_r.segments if s_.net_id not in gids],
                            [v for v in pcb_r.vias if v.net_id not in gids])
    kept = {c: pm.net_copper_hash(pcb_for(c), [byname[c][0]]) for c in group}
    asks = {}
    for o in [nm] + N:
        m = B.cur_dst.get(o)
        asks[o] = sr.move_sig(m) if m is not None else ('face', (B.ends[o]['dst'] or {}).get('direction'))
    parts = {'code': pm.code_hash(), 'knobs': pm.knob_hash(), 'K': K, 'base': os.path.basename(base),
             'nets': hashlib.sha1(nets_csv.encode()).hexdigest()[:12], 'dref': B.st['dref'], 'sref': B.st['sref'],
             'net': nm, 'src': sr.move_sig(src_move) if src_move is not None else None,
             'dst': sr.move_sig(dst_move) if dst_move is not None else None,
             'comove': N, 'asks': asks, 'set': sorted(C), 'others': others, 'kept': kept,
             'coupled': COUPLED, 'grade': GRADE_MODE}
    return pm.key_of(parts)


def _res_doc(res, tag):
    doc = {k: v for k, v in res.items() if k not in ('src', 'dst')}
    files = {}
    if 'board' in res:
        for k, ext in PROBE_FILES.items():
            files[k] = tag + ext
        for k, ext in PROBE_FILES_OPT.items():
            if os.path.exists(tag + ext):
                files[k] = tag + ext
    return {'res': doc, 'files': files, 'tag': tag, 'when': time.time()}


def _res_live(res, src_move, dst_move):
    """A probe result read back from JSON (the memo, a worker) as
    probe_run returns it: the moves attached, the points tuples."""
    res = dict(res)
    res['src'], res['dst'] = src_move, dst_move
    for k in ('src_got', 'dst_got'):
        res[k] = _got_tuples(res.get(k))
    if res.get('comove_got'):
        res['comove_got'] = {o: _got_tuples(g) for o, g in res['comove_got'].items()}
    if res.get('comove'):
        res['comove'] = {o: tuple(v) for o, v in res['comove'].items()}
    return res


def _res_of(doc, src_move, dst_move):
    res = _res_live(doc['res'], src_move, dst_move)
    if 'board' in res:
        res['board'] = doc['files']['rb']
    res['seconds'] = 0.0
    res['memo'] = doc.get('tag')
    return res


def move_to_json(m):
    """A menu move on the wire (to a probe worker): its dataclass fields
    and the co-move list the ranker hangs on it."""
    import dataclasses
    if m is None:
        return None
    d = {f.name: getattr(m, f.name) for f in dataclasses.fields(m)}
    d['replaces'] = move_to_json(d.get('replaces'))
    d['comove'] = list(getattr(m, 'comove', []) or [])
    return d


def move_from_json(d):
    if d is None:
        return None
    d = dict(d)
    comove = d.pop('comove', [])
    d['exit_pt'] = tuple(d['exit_pt'])
    d['legs'] = [(tuple(p), tuple(q), L) for p, q, L in (d.get('legs') or [])]
    d['site'] = tuple(d['site']) if d.get('site') else None
    d['replaces'] = move_from_json(d.get('replaces'))
    m = em.Move(**d)
    m.comove = list(comove)
    return m


def _ends_of(end, m):
    if end == 'both':
        return m[0], m[1]
    return (m, None) if end == 'src' else (None, m)


# ---------------------------------------------------------- the probe pool
# --par=N (2026-09-18): N resident probe workers (probe_worker.py), each
# holding the round's Board and applying the parent's advances; the
# candidates of one net -- independent by construction, the parent
# advances only after the net's menu is judged -- probed N at a time.
POOL = None
WORKER_RECYCLE = int(os.environ.get('PROBE_WORKER_RECYCLE', '100') or 0)
# a worker is 300-450 MB at K41 (measured); one whose PEAK passed this is
# recycled at its next probe (this machine has 8 GB: six workers and two
# parents beside a browser got the population run killed for memory)
WORKER_MAX_MB = float(os.environ.get('PROBE_WORKER_MAX_MB', '1200') or 0)


class _Worker:
    def __init__(self, idx, err_path):
        self.idx, self.err_path, self.p, self.n = idx, err_path, None, 0

    def start(self):
        self.err = open(self.err_path, 'a')
        self.p = subprocess.Popen([sys.executable, '-u', os.path.join(HERE, 'probe_worker.py')],
                                  stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=self.err,
                                  text=True, cwd=HERE, env=dict(os.environ, PROBE_MEMO='0'))
        self.n = 0

    def ask(self, msg):
        self.p.stdin.write(json.dumps(msg, default=str) + '\n')
        self.p.stdin.flush()
        line = self.p.stdout.readline()
        if not line:
            raise RuntimeError(f'probe worker {self.idx} died (rc {self.p.poll()}; see {self.err_path})')
        r = json.loads(line)
        if not r.get('ok'):
            raise RuntimeError(f'probe worker {self.idx}: {r.get("error")}')
        return r

    def stop(self):
        if self.p is None:
            return
        try:
            self.p.stdin.write(json.dumps({'op': 'quit'}) + '\n')
            self.p.stdin.flush()
            self.p.wait(timeout=5)
        except Exception:                                  # noqa: BLE001
            self.p.kill()
        self.p = None


class ProbePool:
    def __init__(self, n, err_dir):
        import queue
        os.makedirs(err_dir, exist_ok=True)
        self.workers = [_Worker(i, os.path.join(err_dir, f'probe_worker_{i}.err')) for i in range(n)]
        for w in self.workers:
            w.start()
        self.state = []              # the messages a fresh worker replays: the round, its advances
        self.idle = queue.Queue()
        for w in self.workers:
            self.idle.put(w)
        self.restarts = 0

    def _replay(self, w):
        w.stop()
        w.start()
        self.restarts += 1
        for m in self.state:
            w.ask(m)

    def _all(self, msg):
        import concurrent.futures as cf

        def one(w):
            if msg['op'] == 'round' and WORKER_RECYCLE and w.n >= WORKER_RECYCLE:
                w.stop()
                w.start()
            try:
                return w.ask(msg)
            except RuntimeError:
                self._replay(w)
                return None
        with cf.ThreadPoolExecutor(max_workers=len(self.workers)) as ex:
            return list(ex.map(one, self.workers))

    def round(self, msg):
        self.state = [msg]
        return self._all(msg)

    def advance(self, msg):
        self.state.append(msg)
        return self._all(msg)

    def probe(self, msg):
        w = self.idle.get()
        try:
            if getattr(w, 'fat', False):
                self._replay(w)          # its peak passed WORKER_MAX_MB: a fresh process
                w.fat = False
            try:
                r = w.ask(msg)
            except RuntimeError as e:
                sys.stderr.write(f'{e}; restarting\n')
                self._replay(w)
                r = w.ask(msg)
            w.n += 1
            if WORKER_MAX_MB and r.get('peak_mb', 0) > WORKER_MAX_MB:
                w.fat = True
            return r
        finally:
            self.idle.put(w)

    def close(self):
        for w in self.workers:
            w.stop()


def probe_many(B, R, nm, items, K, base, nets_csv, log, extra_relay=None, ref=None):
    """The probes of one net's candidates, `items` = [(end, move(s), tag)],
    each as probe_run gives it: a memo hit read back (probe_memo), the
    rest run -- in the pool side by side when one is up, else here one
    after another. Returns the results in the items' order."""
    st = _store('probe')
    out = [None] * len(items)
    miss = []
    for i, (end, m, ptag) in enumerate(items):
        sm, dm = _ends_of(end, m)
        key = probe_key(B, R, nm, sm, dm, K, base, nets_csv, extra_relay) if st is not None else None
        doc = st.get(key) if st is not None else None
        if doc is not None and (not doc.get('files') or pm.files_present(doc)):
            pm.bump('probe_hit')
            out[i] = _res_of(doc, sm, dm)
            continue
        if doc is not None:
            pm.bump('stale_files')
        pm.bump('probe_miss')
        miss.append((i, sm, dm, ptag, key))
    if POOL is not None and miss:
        import concurrent.futures as cf

        def one(t):
            i, sm, dm, ptag, _key = t
            msg = {'op': 'probe', 'R': R, 'net': nm, 'src': move_to_json(sm), 'dst': move_to_json(dm),
                   'tag': ptag, 'extra_relay': list(extra_relay or []), 'ref': ref}
            try:
                r = POOL.probe(msg)
                res = _res_live(r['res'], sm, dm)
            except RuntimeError as e:
                res = {'net': nm, 'src': sm, 'dst': dm, 'fail': f'worker: {e}', 'seconds': 0.0}
            return i, res
        with cf.ThreadPoolExecutor(max_workers=len(POOL.workers)) as ex:
            for i, res in ex.map(one, miss):
                out[i] = res
    else:
        for i, sm, dm, ptag, _key in miss:
            out[i] = probe_run(B, R, nm, sm, dm, ptag, K, base, nets_csv, log, extra_relay, ref=ref)
    if st is not None:
        for i, sm, dm, ptag, key in miss:
            st.put(key, _res_doc(out[i], ptag))
    return out


def probe(B, R, nm, src_move, dst_move, tag, K, base, nets_csv, log, extra_relay=None, ref=None):
    """One probe through probe_many: the memo, then the pool or here."""
    end = 'both' if (src_move is not None and dst_move is not None) else ('src' if src_move is not None else 'dst')
    m = (src_move, dst_move) if end == 'both' else (src_move if end == 'src' else dst_move)
    return probe_many(B, R, nm, [(end, m, tag)], K, base, nets_csv, log, extra_relay, ref=ref)[0]


def probe_run(B, R, nm, src_move, dst_move, tag, K, base, nets_csv, log, extra_relay=None, ref=None):
    """The real router's answer to ONE move on the routed board R: the
    net stripped to its tooth, the asked end(s) re-fanned against the
    frozen copper, braided alone, graded whole. Returns a dict. `ref`:
    R's own grade; when it is DRC-clean the probe's checks are scoped to
    the nets it changed (same answer, a third of the time)."""
    clean = ref is not None and ref[1] == 0
    keep_fast, fp.FAST_PRO = fp.FAST_PRO, True
    try:
        return _probe_run(B, R, nm, src_move, dst_move, tag, K, base, nets_csv, log, extra_relay, ref, clean)
    finally:
        fp.FAST_PRO = keep_fast


def _probe_run(B, R, nm, src_move, dst_move, tag, K, base, nets_csv, log, extra_relay, ref, clean):
    t0 = time.time()
    st, byname = B.st, B.byname
    nid, net = byname[nm]
    res = {'net': nm, 'src': src_move, 'dst': dst_move}
    txt = strip_eco(open(R, encoding='utf-8').read())
    pcb_for = FAN_PCB if FAN_PCB is not None else (lambda _c: B.pcb)
    txt = strip_to_fanout_copper(txt, nm, nid, net.name, pcb_for(nm), _pad(st['sgrid'].bbox))
    # the LOCAL RE-BRAID: the lanes in the way of the new end are stripped
    # (their teeth and berths stay) and re-laid with the moved net
    C, N = coupled_set(B, nm, src_move, dst_move, extra_relay)
    whole = (-1e9, -1e9, 1e9, 1e9)
    for c in sorted(C):
        cid, cnet = byname[c]
        txt = strip_to_fanout_copper(txt, c, cid, cnet.name, pcb_for(c), whole)
    res['relaid'] = sorted(C)
    cur = tag + '_bare.kicad_pcb'
    write_board(txt, cur, R)
    lines = []
    if src_move is not None:
        b1 = tag + '_src.kicad_pcb'
        buf0 = io.StringIO()
        with contextlib.redirect_stdout(buf0), contextlib.redirect_stderr(buf0):
            r = sr.realize(cur, {nm: src_move}, st['src_pad'], byname, st['sref'], b1,
                           log=lines.append, guard_names=(), clean_base=clean)
        with open(tag + '_src.fanout.log', 'w') as f:
            f.write(buf0.getvalue() + '\n'.join(lines))
        a = r['audit'].get(nm, {})
        res['src_laid'] = nm in r['ok']
        res['src_exact'] = bool(a.get('exact'))
        res['src_got'] = a.get('achieved')
        res['src_verdict'] = a.get('verdict')
        if r['rejected'] or nm not in r['ok']:
            res['fail'] = 'source: ' + (r['rejected'] or 'engine refused the tooth')
            res['seconds'] = time.time() - t0
            return res
        cur = b1
    b2 = tag + '_dst.kicad_pcb'
    copy_board(cur, b2)
    move = dst_move if dst_move is not None else B.cur_dst[nm]
    ask = None if move is not None else {nm: B.ends[nm]['dst']['direction']}
    group = [nm] + N
    choice = ({nm: move} if move else {})
    for o in N:
        if B.cur_dst.get(o) is not None:
            choice[o] = B.cur_dst[o]
        else:
            ask = dict(ask or {})
            ask[o] = B.ends[o]['dst']['direction']
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(buf):
        laid, audit_d, ok = fp.fanout_once(b2, group, choice, st['dst_pad'],
                                           st['dref'], byname, cur, relay=group, already=(),
                                           face_asks=ask)
    res['comove'] = {o: (audit_d.get(o, {}).get('verdict'), bool(audit_d.get(o, {}).get('exact')))
                     for o in N}
    # what the engine LAID for the group: the ask the apply step repeats,
    # so a slot exchange the engine negotiated here is asked for outright
    # there (measured: the same group re-fanned on the fanout board in the
    # engine's deepest-first order kept the neighbour exact and dropped
    # the moved net to its fallback)
    res['comove_got'] = {o: (fp.fanout_once.achieved or {}).get(o) for o in N}
    with open(tag + '_dst.fanout.log', 'w') as f:
        f.write(buf.getvalue())
    got = (fp.fanout_once.achieved or {}).get(nm)
    res['dst_laid'] = got is not None
    res['dst_exact'] = bool(audit_d.get(nm, {}).get('exact')) if move else (got is not None)
    res['dst_got'] = got
    res['dst_verdict'] = audit_d.get(nm, {}).get('verdict')
    res['dst_clean'] = ok
    if got is None:
        res['fail'] = 'destination: engine laid no berth'
        res['seconds'] = time.time() - t0
        return res
    if not ok:
        # a re-fanned berth that grazes a LANE routed against the old one:
        # that lane is re-laid with the group (its net named by the DRC pair)
        pairs = sr.drc_pairs(b2, nets=(group if clean else None), pcb_data=(parsed(b2) if clean else None))
        extra = set()
        for ln in pairs:
            for tok in re.findall(r'/([A-Za-z0-9_]+)', ln):
                if tok in byname and tok not in group and tok in B.lanes:
                    extra.add(tok)
        if extra:
            txt2 = open(b2, encoding='utf-8').read()
            for c in sorted(extra):
                cid, cnet = byname[c]
                txt2 = strip_to_fanout_copper(txt2, c, cid, cnet.name, pcb_for(c), whole)
            write_board(txt2, b2, cur)
            C |= extra
            res['relaid'] = sorted(C)
            ok = not sr.drc_pairs(b2, nets=(sorted(set(group) | extra) if clean else None),
                                  pcb_data=(parsed(b2) if clean else None))
        if not ok:
            res['fail'] = 'destination: fanout board not clean/complete (' + '; '.join(pairs[:3]) + ')'
            res['seconds'] = time.time() - t0
            return res
    okb, tb = braid_run(b2, tag + '_rb', ','.join([nm] + sorted(C)), st['dref'], tag + '_rb.log',
                        probe=True)
    res['braid_s'] = tb
    rb = tag + '_rb.kicad_pcb'
    if not okb:
        res['fail'] = 'braid: no board'
        res['seconds'] = time.time() - t0
        return res
    rj = tag + '_rb_refusals.json'
    rf = json.load(open(rj)) if os.path.exists(rj) else {}
    if rf:
        # a refusal with everything else frozen -- the moved net's or a
        # re-laid neighbour's -- is NOT a verdict on the move: the full
        # braid rips and re-lays what this probe cannot (measured: a
        # re-braid of every swimmer on frozen page lanes lost three nets
        # the full braid had closed; SA14's climb routed SA14 at 4 vias
        # and the re-laid SDQ0 refused). Unjudged; the full braid decides.
        res['refused_nets'] = sorted(rf)
        res['unjudged'] = True
        if nm in rf:
            res['refused'] = True
            res['walled_at'] = rf[nm].get('walled_at')
    g, line = grade(rb, K, base, scope=(set([nm]) | C | set(N)) if clean else None, ref=ref)
    res['grade'] = g
    res['board'] = rb
    res['vias_net'] = count_copper(rb, nid)[0]
    res['seconds'] = time.time() - t0
    return res


def _pad(box, m=PAD_MM):
    return (box[0] - m, box[1] - m, box[2] + m, box[3] + m)


def _end_on_routed(pcb_r, B, nm, end):
    """The tooth (end='src') or berth ('dst') of `nm` on a ROUTED board:
    the free end the fanout board's copper would have, found among the
    routed board's segments that match the fanout board's -- the braid
    keeps its input copper verbatim, so the end is the fanout segment
    endpoint the lane continues from."""
    nid = B.byname[nm][0]
    box = _pad(B.st['sgrid'].bbox) if end == 'src' else _pad(B.st['dgrid'].bbox)
    fs = [s_ for s_ in B.pcb.segments if s_.net_id == nid
          and in_box((s_.start_x, s_.start_y), box) and in_box((s_.end_x, s_.end_y), box)]
    rs = [s_ for s_ in pcb_r.segments if s_.net_id == nid
          and any(t.layer == s_.layer and ((_near((s_.start_x, s_.start_y), (t.start_x, t.start_y))
                                           and _near((s_.end_x, s_.end_y), (t.end_x, t.end_y)))
                                          or (_near((s_.start_x, s_.start_y), (t.end_x, t.end_y))
                                              and _near((s_.end_x, s_.end_y), (t.start_x, t.start_y))))
                  for t in fs)]
    if not rs:
        return None
    cnt = Counter()
    for s_ in rs:
        cnt[(round(s_.start_x, 3), round(s_.start_y, 3))] += 1
        cnt[(round(s_.end_x, 3), round(s_.end_y, 3))] += 1
    g = B1_ends_ref = B.ends[nm][end]
    # the fanout copper's free end on the routed board is the one the lane
    # continues from: the endpoint used once that is nearest the fanout
    # board's own measured end
    ends = [pt for pt, c in cnt.items() if c == 1]
    if not ends or g is None:
        return None
    pt = min(ends, key=lambda q: (q[0] - g['tooth'][0]) ** 2 + (q[1] - g['tooth'][1]) ** 2)
    seg = next((s_ for s_ in rs if _near((s_.start_x, s_.start_y), pt) or _near((s_.end_x, s_.end_y), pt)), None)
    return {'tooth': (round(pt[0], 3), round(pt[1], 3)), 'layer': seg.layer if seg else '?',
            'vias': g['vias'], 'kind': g['kind'], 'direction': g['direction']}


def better(g, ref):
    """(opens, drc, vias) lexicographic: fewer opens; no DRC; fewer vias."""
    if g is None or g[0] is None:
        return False
    if g[1] != 0:
        return False
    if len(g[0]) != len(ref[0]):
        return len(g[0]) < len(ref[0])
    if LENGTH_TIE and g[2] == ref[2] and len(g) > 3 and len(ref) > 3 \
            and g[3] is not None and ref[3] is not None:
        return g[3] < ref[3] - 0.3
    return g[2] < ref[2]


def fmt_move(m):
    if m is None:
        return '(engine)'
    if isinstance(m, tuple):
        return f'tooth {fmt_move(m[0])} + berth {fmt_move(m[1])}'
    return sr.fmt_ask(m) + (f' climb {m.climb}' if m.climb else '')


# ---------------------------------------------------------------- main
def main():
    tag, K = ARGV[0], int(ARGV[1])
    if '/' not in tag:
        tag = os.path.join('tmp', tag)
    out_tag = OPTS.get('out', os.path.basename(tag))
    if '/' not in out_tag:
        out_tag = os.path.join('tmp', out_tag)
    base = OPTS.get('board', os.path.join(HERE, 'fb_t2q_fresh.kicad_pcb'))
    dest = OPTS.get('dest', 'DU1')
    # THE DESIGN CONSTANTS (rules.py) -- inert today, the seam tomorrow.
    _r = _rules.install_defaults()
    print(f'rules: clearance {te.SPEC_CLEARANCE} (hug {te.CLEAR}), '
          f'track {te.TRACK}, via {te.VIA_SIZE}/{te.VIA_DRILL}'
          f'  [{_r.source}]')
    ROUNDS = int(OPTS.get('rounds', 4))
    WORST = int(OPTS.get('worst', 3))
    if PERTURB or CROSS:
        ROUNDS, WORST = 1, PERTURB or 10 ** 6
        import random as _random
        prng = _random.Random(PERTURB_SEED)
    PROBES = int(OPTS.get('probes', 1))
    MIN_VIAS = int(OPTS.get('min-vias', 3))
    MODE = OPTS.get('mode', 'incremental')      # incremental | rebraid
    JOINT = OPTS.get('joint', '1') == '1'        # tooth-and-berth pairs probed too
    GATE_MIN = int(OPTS.get('gate-min', 2))      # bad nets a good one must block to be re-planned
    GATES = int(OPTS.get('gates', 3))            # gatekeepers re-planned per round, at most
    CENSUS = int(OPTS.get('census', 4))          # blockers per bad net taken from the braid's census
    stem = f'{out_tag}_rp_k{K}'
    log = print
    t_all = time.time()
    global MEMO_K, POOL
    MEMO_K = K
    PAR = int(OPTS.get('par', 0) or 0)
    if PAR > 0:
        POOL = ProbePool(PAR, os.path.dirname(stem) or '.')
        import atexit
        atexit.register(POOL.close)
    nets_all = coherent_nets(K, base)
    nets_csv = ','.join(nets_all)
    F0 = f'{tag}_fo_k{K}.kicad_pcb'
    R0 = f'{tag}_k{K}.kicad_pcb'
    if OPTS.get('from'):
        # continue from a previous run's boards: STEM_fo.kicad_pcb (+ its
        # plan sidecar) and STEM.kicad_pcb (+ its braid log and pack)
        F0 = OPTS['from'] + '_fo.kicad_pcb'
        R0 = OPTS['from'] + '.kicad_pcb'
    # the nets planned here: those ending on the destination array
    pcb0 = parse_kicad_pcb(base)
    byname0 = {n.name.split('/')[-1]: (i, n) for i, n in pcb0.nets.items()}
    ends0 = te.endpoints(pcb0, nets_all, byname0)
    dref = Counter(ends0[nm][2] for nm in nets_all if nm in ends0).most_common(1)[0][0]
    names = [nm for nm in nets_all if nm in ends0 and ends0[nm][2] == dref]
    named = set(json.load(open(F0[:-len('.kicad_pcb')] + '.plan.json')).get('ends', {}))
    log(f'replan: K{K} {len(names)} nets on {dref} ({len(nets_all) - len(names)} elsewhere), '
        f'from {os.path.basename(F0)} / {os.path.basename(R0)}; {ROUNDS} round(s), '
        f'{WORST} worst, {PROBES} probe(s) per end, SRC_CLIMB={os.environ["SRC_CLIMB"]}')

    # round 0: the recorded boards as they stand
    F, R = f'{stem}_r0_fo.kicad_pcb', f'{stem}_r0.kicad_pcb'
    copy_board(F0, F)
    shutil.copy(F0[:-len('.kicad_pcb')] + '.plan.json', F[:-len('.kicad_pcb')] + '.plan.json')
    copy_board(R0, R, eco=True)
    for ext in ('.log', '.pack.json', '_refusals.json', '.census.json'):
        src = R0[:-len('.kicad_pcb')] + ext
        if os.path.exists(src):
            shutil.copy(src, R[:-len('.kicad_pcb')] + ext)
    best_g, line = grade(R, K, base)
    log(f'  start: open {best_g[0]}, drc {best_g[1]}, vias {best_g[2]}')
    bans_d = {nm: set() for nm in names}
    bans_s = set()             # (net, move sig) at the source
    bans_pair = {nm: set() for nm in names}   # (source sig, berth face, berth layer) tried together
    tried = Counter()
    prevV = None
    census_hist = {}
    fan_src = {}        # net -> the board carrying its fanout copper (--apply=strip)
    _pcb_cache = {}

    def _pcb_of(path):
        if path not in _pcb_cache:
            _pcb_cache[path] = parse_kicad_pcb(path)
        return _pcb_cache[path]
    import gc
    import resource
    for rnd in range(1, ROUNDS + 1):
        # A ROUND THAT DIES MUST NOT LOSE THE BOARDS KEPT SO FAR (2026-09-18):
        # the descent of an open-net seed closed both opens and reached 95 in
        # round 1, then round 2 crashed in Board() on the derived fanout board
        # (SDQ2 'no free stub end') and the run wrote no final -- the world was
        # lost to evolve. The exception is logged, the run finishes with the
        # last kept board, and the derived-board defect stays a TODO.
        try:
            t_r = time.time()
            gc.collect()
            log(f'\n=== round {rnd}: verdict off {os.path.basename(R)}  '
                f'(peak rss {resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1048576:.0f} MB)')
            B = Board(F, names, dref, banned=frozenset(bans_s), R=R)
            if APPLY_STRIP:
                global FAN_PCB
                _F_round = F
                FAN_PCB = lambda c, _F=_F_round: _pcb_of(fan_src.get(c, _F))
            V = verdict(R[:-len('.kicad_pcb')])
            if prevV is not None:
                # an incremental board's log names only the re-laid nets: the
                # others keep the class the last full verdict gave them
                for nm in names:
                    if 'cls' not in V.get(nm, {}) and 'cls' in prevV.get(nm, {}):
                        V.setdefault(nm, {})['cls'] = prevV[nm]['cls']
            pcbR = parse_kicad_pcb(R)
            real = {nm: sum(1 for v in pcbR.vias if v.net_id == B.byname[nm][0]) for nm in names}
            for nm in names:
                # the LANE's vias, off the board: the ends' vias taken off
                ev = sum((B.ends[nm][e] or {}).get('vias', 0) for e in ('src', 'dst'))
                V.setdefault(nm, {})['lane_vias'] = max(0, real[nm] - ev)
            prevV = V
            pred, bp = B.model(named)
            # THE LEARNED PRICE: real - predicted, per net, on the plan routed
            resid = {}
            for nm in names:
                if nm in pred and not V.get(nm, {}).get('refused'):
                    resid[nm] = real[nm] - pred[nm]
            pe.RESIDUAL = resid
            B.swimmers = {nm for nm in names if V.get(nm, {}).get('cls') == 'swim' and not V[nm].get('refused')}
            nsw_plan = sum(1 for nm in names if nm in bp and bp[nm]['page'] is None)
            nsw_braid = sum(1 for nm in names if V.get(nm, {}).get('cls') == 'swim')
            log(f'  plan model on this fanout: {sum(pred.values())} vias over {len(pred)} nets, '
                f'{nsw_plan} swimmers; the braid: {sum(real.values())} vias, {nsw_braid} swimmers, '
                f'refused {[n for n in names if V.get(n, {}).get("refused")]}; '
                # :+d CRASHED HERE. resid is real - pred, and the plan model's
                # prediction is a FLOAT, so the residual always was one -- this
                # line has never changed and raises ValueError on every round 1,
                # which is why no replan round could run at all.
                f'residual real-pred: total {sum(resid.values()):+.1f}, '
                f'|.| {sum(abs(x) for x in resid.values()):.1f} over {len(resid)} nets')
            worst = [nm for nm in names if V.get(nm, {}).get('refused')]
            sw = sorted((nm for nm in names if not V.get(nm, {}).get('refused')
                         and V[nm].get('lane_vias', 0) >= MIN_VIAS),
                        key=lambda n: (-V[n]['lane_vias'], V[n].get('cls') != 'swim'))
            worst += sw[:WORST]
            if PERTURB:
                # the near jump: N random nets, not the worst ones
                worst = prng.sample(names, min(PERTURB, len(names)))
                log(f'  perturb (seed {PERTURB_SEED}): {worst} moved to another class each, '
                    f'{PERTURB_TRIES} candidate(s) probed per net at most')
            cross_cands = {}
            if CROSS:
                # the crossover: the nets whose ends differ from parent B's, a
                # random share of them asked for B's end(s) on this board
                B2 = Board(CROSS + '_fo.kicad_pcb', names, dref)
                differ = {}
                for nm in names:
                    ends_b = {e: (None if _end_same(B.ends[nm][e], B2.ends[nm][e]) else B2.ends[nm][e])
                              for e in ('src', 'dst')}
                    if ends_b['src'] or ends_b['dst']:
                        differ[nm] = ends_b
                picked = sorted(prng.sample(sorted(differ), max(1, round(CROSS_FRAC * len(differ))))) if differ else []
                for nm in picked:
                    ms = synth_move(nm, differ[nm]['src']) if differ[nm]['src'] else None
                    md = synth_move(nm, differ[nm]['dst']) if differ[nm]['dst'] else None
                    cl = []
                    if ms is not None and md is not None:
                        cl.append(('both', (ms, md), 0.0))
                    if md is not None:
                        cl.append(('dst', md, 0.0))
                    if ms is not None:
                        cl.append(('src', ms, 0.0))
                    cross_cands[nm] = cl
                worst = [nm for nm in picked if cross_cands.get(nm)]
                prng.shuffle(worst)
                log(f'  cross (seed {PERTURB_SEED}): {len(differ)} of {len(names)} nets differ from '
                    f'{os.path.basename(CROSS)}; taking B\'s ends for {worst}'
                    + (f' (no askable move for {[n for n in picked if n not in worst]})'
                       if len(worst) < len(picked) else ''))
            bad = set(worst)
            log(f'  worst: ' + '; '.join(fmt_v(nm, V[nm], real[nm]) for nm in worst))
            # THE BLOCKER CENSUS: for each bad net, the lanes the braid found
            # in its way (this round's log, and every earlier round's); a good
            # net in the way of GATE_MIN bad ones is a GATEKEEPER -- not frozen:
            # its lane is re-laid in their probes and it is re-planned itself
            for nm in names:
                v = V.get(nm, {})
                bl = list(v.get('mincut', [])) + [n for n, c in sorted(v.get('frontier', []), key=lambda t: -t[1])]
                seen_b = []
                for b_ in bl:
                    if b_ != nm and b_ in B.byname and b_ not in seen_b:
                        seen_b.append(b_)
                for b_ in v.get('census_prev', []):
                    if b_ not in seen_b and b_ != nm and b_ in B.byname:
                        seen_b.append(b_)
                if seen_b:
                    census_hist.setdefault(nm, [])
                    census_hist[nm] = seen_b[:CENSUS] + [b_ for b_ in census_hist[nm] if b_ not in seen_b][:CENSUS]
            blockers = {nm: [b_ for b_ in census_hist.get(nm, []) if b_ not in bad][:CENSUS] for nm in worst}
            gate_score = Counter(b_ for nm in worst for b_ in blockers[nm])
            gates = [g for g, c in gate_score.most_common() if c >= GATE_MIN][:GATES]
            log('  census (the braid\'s own): ' + '; '.join(f'{nm} <- {blockers[nm]}' for nm in worst if blockers[nm]))
            log(f'  gatekeepers (good nets in the way of >= {GATE_MIN} bad ones): '
                + (', '.join(f'{g} ({gate_score[g]})' for g in gates) or 'none'))
            B.blockers = blockers
            if POOL is not None:
                POOL.round({'op': 'round', 'F': F, 'R': R, 'names': names, 'dref': dref,
                            'banned': [list(x) for x in bans_s], 'blockers': blockers,
                            'swimmers': sorted(B.swimmers), 'resid': resid, 'fan_src': dict(fan_src),
                            'K': K, 'base': base, 'nets_csv': nets_csv, 'apply_strip': APPLY_STRIP,
                            'coupled': COUPLED, 'grade': GRADE_MODE})
            buses = B.buses()
            cache = {}
            stand = {}          # net -> (src_move, dst_move, probe)
            unjudged = {}       # net -> (end, move, probe): laid in class, refused with everything frozen
            R_cur = R           # the routed board the probes run on (advances in incremental mode)
            g_round0 = list(best_g)
            for nm in worst:
                v = V[nm]
                ends_try = []
                if v.get('refused'):
                    w = v.get('walled_at')
                    ends_try = ['src'] if w == 'tooth' else ['dst'] if w == 'berth' else ['dst', 'src']
                else:
                    ends_try = ['dst', 'src']
                ref_g = best_g
                cands = []
                if len(stand) >= WORST + 2:
                    break
                screened = {}
                if CROSS:
                    cands = list(cross_cands.get(nm, []))
                    ends_try = []            # B's ends are the candidates; no ranking, no screen
                    log(f'  {nm}: B\'s end(s) asked: '
                        + '; '.join((f'{fmt_move(m[0])} + {fmt_move(m[1])}' if end == 'both' else fmt_move(m))
                                    for end, m, c in cands))
                for end in ends_try:
                    if PERTURB:
                        # every class the menu offers, shuffled, the current
                        # class out (rank_dest leaves it out already; the
                        # source menu keeps same-class teeth for a descent)
                        if end == 'dst':
                            ranked, n_menu = rank_dest(B, nm, bans_d[nm], buses, cache, 10 ** 6)
                        else:
                            ranked, n_menu = rank_src(B, nm, buses, cache, 10 ** 6)
                            ranked = [(c_, m_) for c_, m_ in ranked if (m_.direction, m_.layer) != B.cls(nm, 'src')]
                        prng.shuffle(ranked)
                        ranked = screen(B, nm, ranked[:SCREEN_MAX], end, PERTURB_TRIES, log)
                        screened[end] = ranked
                        log(f'  {nm} {end}: {n_menu} move(s) in the menu, current '
                            f'{sr.fmt(B.ends[nm][end])}; random other-class candidates: '
                            + ('; '.join(f'{fmt_move(m)}' for c, m in ranked[:PERTURB_TRIES]) or 'none'))
                        for c, m in ranked[:PERTURB_TRIES]:
                            cands.append((end, m, c))
                        continue
                    if end == 'dst':
                        ranked, n_menu = rank_dest(B, nm, bans_d[nm], buses, cache, SCREEN_MAX)
                    else:
                        ranked, n_menu = rank_src(B, nm, buses, cache, SCREEN_MAX)
                    ranked = screen(B, nm, ranked, end, PROBES + 1, log) if SCREEN else ranked[:PROBES + 1]
                    screened[end] = ranked
                    log(f'  {nm} {end}: {n_menu} move(s) in the menu, current '
                        f'{sr.fmt(B.ends[nm][end])}; candidates: '
                        + ('; '.join(f'{fmt_move(m)} judged {c:.1f}' for c, m in ranked[:PROBES]) or 'none'))
                    for c, m in ranked[:PROBES]:
                        cands.append((end, m, c))
                if 'src' in ends_try and 'dst' in ends_try and JOINT:
                    # JOINT candidates: a tooth AND a berth moved together -- a
                    # net whose launch and target ranks are both wrong looks
                    # worse after either end alone (SA12's win took two rounds,
                    # tooth south then berth east, because the first happened to
                    # pay by itself); ranked by the judged cost with both applied
                    rs_ = screened.get('src', [])
                    rd_ = screened.get('dst', [])
                    pairs = []
                    for cs, ms in rs_:
                        for cd, md in rd_:
                            if (sr.move_sig(ms), md.direction, md.layer) in bans_pair[nm]:
                                continue
                            launch = dict(B.st['launch'])
                            launch[nm] = ms.exit_pt
                            tl = dict(B.st['tooth0'])
                            tl[nm] = ms.layer
                            tv = dict(B.st['tooth_vias'])
                            tv[nm] = ms.vias
                            ch = dict(B.choice)
                            ch[nm] = md
                            pairs.append((B.cost(ch, launch, tl, tv, buses, cache), ms, md))
                    pairs.sort(key=lambda t: t[0])
                    if pairs:
                        log(f'  {nm} both: {len(pairs)} pair(s); best: '
                            + '; '.join(f'{fmt_move(ms)} + {fmt_move(md)} judged {c:.1f}' for c, ms, md in pairs[:PROBES]))
                    for c, ms, md in pairs[:PROBES]:
                        cands.append(('both', (ms, md), c))
                results = []
                routed = []         # perturb: every probe that routed, best-first at the end
                if PERTURB:
                    cands = [c_ for c_ in cands if c_[0] != 'both']
                    prng.shuffle(cands)
                    cands = cands[:PERTURB_TRIES]
                items = []
                for end, m, c in cands:
                    tried[nm] += 1
                    items.append((end, m, f'{stem}_r{rnd}_{nm}_{end}{tried[nm]}'))
                prs = probe_many(B, R_cur, nm, items, K, base, nets_csv, log, ref=list(best_g))
                for (end, m, c), pr in zip(cands, prs):
                    if 'fail' in pr:
                        log(f'    probe {end} {fmt_move(m)}: FAILED ({pr["fail"]}; '
                            f'{pr.get("src_verdict") or pr.get("dst_verdict") or ""}) {pr["seconds"]:.0f} s')
                        if end == 'dst':
                            bans_d[nm].add(_dban(m))
                        elif end == 'src':
                            bans_s.add((nm, sr.move_sig(m)))
                        else:
                            bans_pair[nm].add((sr.move_sig(m[0]), m[1].direction, m[1].layer))
                        continue
                    g = pr['grade']
                    if end == 'both':
                        ms, md = m
                        faith = bool(pr.get('src_exact')) and bool(pr.get('dst_exact'))
                        in_cls = ((pr['src_got']['direction'], pr['src_got']['layer']) == (ms.direction, ms.layer)
                                  and (pr['dst_got']['direction'], pr['dst_got']['layer']) == (md.direction, md.layer))
                        verdict_s = f'{pr.get("src_verdict")} / {pr.get("dst_verdict")}'
                        laid_cls = None
                    else:
                        faith = ((pr.get('src_exact', True) if end == 'src' else True)
                                 and (pr.get('dst_exact', True) if end == 'dst' else True))
                        laid_cls = ((pr['src_got']['direction'], pr['src_got']['layer']) if end == 'src'
                                    else (pr['dst_got']['direction'], pr['dst_got']['layer']))
                        in_cls = laid_cls == (m.direction, m.layer)
                        verdict_s = (pr.get('src_verdict') if end == 'src' else pr.get('dst_verdict')) or ''
                    # the candidate's OWN net must have routed: a board that
                    # grades better because the re-laid neighbours came out
                    # cheaper is no verdict on the move (measured: SA11's berth
                    # 'stood' at 107 vias with SA11 still refused)
                    # the engine may have laid another class than asked: if the
                    # board is better all the same, the LAID move (menu-matched)
                    # is what the apply step asks for, and the asked class, which
                    # the engine would not lay, is banned
                    substitute = None
                    if end != 'both' and not in_cls and better(g, ref_g) and not pr.get('refused') and not pr.get('unjudged'):
                        got = pr['src_got'] if end == 'src' else pr['dst_got']
                        menu_ = B.st['smenu'].get(nm, []) if end == 'src' else dmenu_full(B.st)[nm]
                        substitute = fp._menu_match(menu_, got) or synth_move(nm, got)
                        if substitute is not None:
                            substitute.comove = list(getattr(m, 'comove', []) or [])
                    ok = better(g, ref_g) and (in_cls or substitute is not None) and not pr.get('refused')
                    if LENGTH_TIE and pr.get('unjudged') and in_cls and g[0] == [] and g[1] == 0 \
                            and better(g, ref_g):
                        # --length: the local braid refused a lane but the last
                        # call closed it -- the board is complete and clean at
                        # equal vias and shorter copper, which IS the verdict
                        # asked for (K28: 14 of 15 walked probes refused locally
                        # and every board graded 0 open / 0 DRC / 36 vias)
                        ok = True
                        pr['unjudged'] = False
                    if substitute is not None:
                        if end == 'dst':
                            bans_d[nm].add(_dban(m))
                        else:
                            bans_s.add((nm, sr.move_sig(m)))
                        m = substitute
                    if pr.get('unjudged') and in_cls:
                        # the best unjudged per net: its own net routed first,
                        # then the board's opens and vias
                        key = (pr.get('refused', False), len(g[0]), g[2])
                        if nm not in unjudged or key < unjudged[nm][3]:
                            unjudged[nm] = (end, m, pr, key)
                    laid_s = (f'{sr.fmt(pr["src_got"])} + {sr.fmt(pr["dst_got"])}' if end == 'both'
                              else sr.fmt(pr["src_got"] if end == "src" else pr["dst_got"]))
                    log(f'    probe {end} {fmt_move(m)}: laid {laid_s} '
                        f'[{"exact" if faith else ("in class" if in_cls else "OTHER CLASS")}]'
                        + (f' -- {verdict_s}' if not faith else '')
                        + (f'; REFUSED (walled at the {pr.get("walled_at")})' if pr.get('refused') else
                           f'; routed: net {pr["vias_net"]} v (was {real[nm]})')
                        + (f' with {pr["relaid"]} re-laid' if pr.get('relaid') else ' alone')
                        + (f', co-moved {pr["comove"]}' if pr.get('comove') else '')
                        + (f', refused {pr["refused_nets"]}' if pr.get('refused_nets') else '')
                        + f'; board open {g[0]} drc {g[1]} vias {g[2]} (ref {len(ref_g[0])}/{ref_g[2]})'
                        + (f' mm {g[3]} (ref {ref_g[3]})' if LENGTH_TIE and len(g) > 3 and len(ref_g) > 3 else '')
                        + f' -> {"STANDS" if ok else ("unjudged" if pr.get("unjudged") and in_cls else "rejected")}'
                        + (f' (the engine\'s substitute {fmt_move(m)} is the ask)' if substitute is not None else '')
                        + (' [memo]' if pr.get('memo') else f' ({pr["seconds"]:.0f} s)'))
                    if (PERTURB and not pr.get('refused') and (in_cls or substitute is not None)) \
                            or (CROSS and not pr.get('refused') and len(g[0]) <= len(ref_g[0])):
                        # a crossover takes B's end only where it leaves no net
                        # open that was routed (measured: 22 ends taken across
                        # lineages walked the 83 to 110 with two open)
                        routed.append((len(g[0]), g[2], 0, end, m, pr))
                        if not g[0]:
                            break           # a jump lands on the first complete board
                    if ok:
                        results.append((len(g[0]), g[2], (g[3] if LENGTH_TIE and len(g) > 3
                                                           and g[3] is not None else 0), end, m, pr))
                    elif not (pr.get('unjudged') and in_cls) and substitute is None:
                        if end == 'dst':
                            bans_d[nm].add(_dban(m))
                        elif end == 'src':
                            bans_s.add((nm, sr.move_sig(m)))
                        else:
                            bans_pair[nm].add((sr.move_sig(m[0]), m[1].direction, m[1].layer))
                if (PERTURB or CROSS) and routed:
                    results = routed        # the landing: whatever it grades
                    log(f'    {nm}: {"CROSSED" if CROSS else "JUMPED"} (the probe board is the landing, graded '
                        f'open {min(routed)[0]} vias {min(routed)[1]})')
                elif CROSS:
                    log(f'    {nm}: B\'s end NOT taken (no landing without a net left open)')
                if results:
                    results.sort(key=lambda t: (t[0], t[1], t[2]))
                    _o, _v, _mm, end, m, pr = results[0]
                    stand[nm] = ((m[0], m[1], pr) if end == 'both'
                                 else (m if end == 'src' else None, m if end == 'dst' else None, pr))
                    if MODE == 'incremental':
                        # the probe's board IS a routed board (graded whole):
                        # the next net is probed on it, and the round ships it
                        R_cur = pr['board']
                        best_g = list(pr['grade'])
                        B.lanes = lane_items(parse_kicad_pcb(R_cur), B.pcb, names, B.byname)
                        B.advance(nm, pr)
                        fo_b = R_cur[:-len('_rb.kicad_pcb')] + '_dst.kicad_pcb'
                        for o in {nm} | set(pr.get('relaid') or []) | set((pr.get('comove_got') or {})):
                            fan_src[o] = fo_b
                        if POOL is not None:
                            POOL.advance({'op': 'advance', 'net': nm, 'pr': _res_doc(pr, '')['res'],
                                          'R_cur': R_cur, 'fan_src': dict(fan_src)})
                        log(f'    {nm}: {os.path.basename(R_cur)} is the board now '
                            f'(open {best_g[0]}, drc {best_g[1]}, vias {best_g[2]})')
            # PHASE 2: each gatekeeper tried at another class of its own,
            # judged by the local braid with the bad nets it blocks re-laid
            for g in (gates if not (PERTURB or CROSS) else []):
                if g in stand or g in bad:
                    continue
                blocked_by_g = [nm for nm in worst if g in blockers.get(nm, [])]
                ref_g = best_g
                cands = []
                for end in ('dst', 'src'):
                    if end == 'dst':
                        ranked, n_menu = rank_dest(B, g, bans_d[g], buses, cache, SCREEN_MAX)
                    else:
                        ranked, n_menu = rank_src(B, g, buses, cache, SCREEN_MAX)
                    ranked = screen(B, g, ranked, end, PROBES, log) if SCREEN else ranked[:PROBES]
                    log(f'  gatekeeper {g} {end}: {n_menu} move(s), current {sr.fmt(B.ends[g][end])}, blocks '
                        f'{blocked_by_g}; candidates: '
                        + ('; '.join(f'{fmt_move(m)} judged {c:.1f}' for c, m in ranked) or 'none'))
                    for c, m in ranked:
                        cands.append((end, m, c))
                results = []
                items = []
                for end, m, c in cands:
                    tried[g] += 1
                    items.append((end, m, f'{stem}_r{rnd}_{g}_{end}{tried[g]}'))
                prs = probe_many(B, R_cur, g, items, K, base, nets_csv, log, extra_relay=blocked_by_g,
                                 ref=list(best_g))
                for (end, m, c), pr in zip(cands, prs):
                    if 'fail' in pr:
                        log(f'    probe {end} {fmt_move(m)}: FAILED ({pr["fail"]}) {pr["seconds"]:.0f} s')
                        if end == 'dst':
                            bans_d[g].add(_dban(m))
                        else:
                            bans_s.add((g, sr.move_sig(m)))
                        continue
                    gg = pr['grade']
                    laid_cls = ((pr['src_got']['direction'], pr['src_got']['layer']) if end == 'src'
                                else (pr['dst_got']['direction'], pr['dst_got']['layer']))
                    in_cls = laid_cls == (m.direction, m.layer)
                    ok = better(gg, ref_g) and in_cls and not pr.get('refused')
                    log(f'    probe {end} {fmt_move(m)}: laid {sr.fmt(pr["src_got"] if end == "src" else pr["dst_got"])} '
                        f'[{"in class" if in_cls else "OTHER CLASS"}]'
                        + (f'; REFUSED' if pr.get('refused') else f'; routed: net {pr["vias_net"]} v (was {real[g]})')
                        + f' with {pr.get("relaid")} re-laid'
                        + (f', refused {pr["refused_nets"]}' if pr.get('refused_nets') else '')
                        + f'; board open {gg[0]} drc {gg[1]} vias {gg[2]} (ref {len(ref_g[0])}/{ref_g[2]})'
                        f' -> {"STANDS" if ok else ("unjudged" if pr.get("unjudged") and in_cls else "rejected")}'
                        + (' [memo]' if pr.get('memo') else f' ({pr["seconds"]:.0f} s)'))
                    if ok:
                        results.append((len(gg[0]), gg[2], end, m, pr))
                    elif not (pr.get('unjudged') and in_cls):
                        if end == 'dst':
                            bans_d[g].add(_dban(m))
                        else:
                            bans_s.add((g, sr.move_sig(m)))
                if results:
                    results.sort(key=lambda t: (t[0], t[1]))
                    _o, _v, end, m, pr = results[0]
                    stand[g] = (m if end == 'src' else None, m if end == 'dst' else None, pr)
                    if MODE == 'incremental':
                        R_cur = pr['board']
                        best_g = list(pr['grade'])
                        B.lanes = lane_items(parse_kicad_pcb(R_cur), B.pcb, names, B.byname)
                        B.advance(g, pr)
                        if POOL is not None:
                            POOL.advance({'op': 'advance', 'net': g, 'pr': _res_doc(pr, '')['res'],
                                          'R_cur': R_cur, 'fan_src': dict(fan_src)})
                        log(f'    {g}: {os.path.basename(R_cur)} is the board now '
                            f'(open {best_g[0]}, drc {best_g[1]}, vias {best_g[2]})')
            if not stand and unjudged and MODE == 'incremental':
                log(f'  round {rnd}: nothing judged better; {len(unjudged)} unjudged move(s) would need the '
                    f'full braid (--mode=rebraid) -- stopping ({time.time() - t_r:.0f} s)')
                break
            if not stand and unjudged:
                # nothing judged better: the unjudged moves go to the full
                # braid, which is the only judge left for them
                for nm, (end, m, pr, _k) in unjudged.items():
                    stand[nm] = ((m[0], m[1], pr) if end == 'both'
                                 else (m if end == 'src' else None, m if end == 'dst' else None, pr))
                log(f'  round {rnd}: nothing judged better; applying the unjudged moves for the full braid: '
                    + ', '.join(f'{nm} {end} {fmt_move(m)}' for nm, (end, m, pr, _k) in unjudged.items()))
            if not stand:
                log(f'  round {rnd}: no candidate stands -- stopping ({time.time() - t_r:.0f} s)')
                break
            # WHICH APPLY PATH, AND WHY. The strip branch needs four things at
            # once and said nothing when it did not get them, so a whole
            # --apply=strip vs refan A/B ran with the branch never firing in
            # either arm and reported "no difference" off two identical code
            # paths. Name the blocker instead.
            _why = [n for n, ok_ in (('apply!=strip', APPLY_STRIP),
                                     ('mode!=incremental', MODE == 'incremental'),
                                     ('no candidate stands', bool(stand)),
                                     ('no incremental board this round', R_cur != R))
                    if not ok_]
                # A STANDING SOURCE MOVE NO LONGER BLOCKS THE DERIVED PATH (2026-09-18):
                # the probe realized the tooth on its own board, so `fan_src` names
                # copper carrying the new tooth, and salvage_missing_ends covers a net
                # re-laid from its pad. With the block, a round whose source move stood
                # went through the re-fan apply, the engine did not lay the moves as
                # asked, and the round DISCARDED a probe board it had already graded
                # (the jump world: 102 / 1 open -> 98 clean, thrown away).
            log(f'  round {rnd}: apply path = ' + ('DERIVED (strip)' if not _why
                                                   else 'incremental/refan -- blocked by ' + ', '.join(_why)))
            if not _why:
                # --apply=strip: the fanout board IS the routed board without
                # its lanes, net by net, each stripped to the copper of the
                # board that last laid its ends
                F1 = f'{stem}_r{rnd}_fo.kicad_pcb'
                txt = strip_eco(open(R_cur, encoding='utf-8').read())
                whole = (-1e9, -1e9, 1e9, 1e9)
                changed = sorted(o for o in names if fan_src.get(o))
                for nm in names:
                    nid, net = B.byname[nm]
                    txt = strip_to_fanout_copper(txt, nm, nid, net.name, _pcb_of(fan_src.get(nm, F)), whole)
                write_board(txt, F1, F)
                salvage_missing_ends(F1, R_cur, names, B.byname, _pad(B.st['sgrid'].bbox), _pad(B.st['dgrid'].bbox), log)
                B1 = Board(F1, names, dref, banned=frozenset(bans_s))
                named1 = (named | set(changed)) & set(B1.choice)
                pred1, bp1, side = B1.write_sidecar(named1)
                R1 = f'{stem}_r{rnd}.kicad_pcb'
                copy_board(R_cur, R1, eco=True)
                for ext in ('.log', '.pack.json', '_refusals.json'):
                    src_ = R_cur[:-len('.kicad_pcb')] + ext
                    if os.path.exists(src_):
                        shutil.copy(src_, R1[:-len('.kicad_pcb')] + ext)
                with open(R1[:-len('.kicad_pcb')] + '.census.json', 'w') as f:
                    json.dump(census_hist, f, indent=1, sort_keys=True)
                g1, line = grade(R1, K, base)
                # the derived fanout board must carry every changed net's ends
                # where the routed board has them (the same copper, so a
                # mismatch is a stripping error, not the engine's)
                miss, trimmed = [], []
                for nm in changed:
                    a, b = B1.ends[nm]['dst'], B.ends[nm]['dst']
                    if a is None:
                        miss.append(nm)
                    elif b is None or a['layer'] != b['layer'] or a['direction'] != b['direction'] \
                            or math.hypot(a['tooth'][0] - b['tooth'][0], a['tooth'][1] - b['tooth'][1]) > END_AGREE:
                        # the lane joined the berth short of its tip and the
                        # braid trimmed the bypassed tip (the #622 overshoot
                        # trim): the derived board's end is the routed board's
                        trimmed.append(f'{nm}: {sr.fmt(b)} -> {sr.fmt(a)}')
                keep = (better(g1, g_round0) or bool(PERTURB or CROSS)) and not miss
                log(f'  round {rnd}: {"KEPT" if keep else "rejected"} derived -- open {g1[0]}, drc {g1[1]}, '
                    f'vias {g1[2]}' + (f' mm {g1[3]}' if LENGTH_TIE else '')
                    + f' (round start {g_round0[0]}/{g_round0[2]}); fanout board {os.path.basename(F1)} derived '
                    f'from the routed board, {len(changed)} net(s) with new ends, sidecar {len(named1)} nets named'
                    + (f'; NO END on the derived board for {miss}' if miss else '')
                    + (f'; berths trimmed by their lanes: {trimmed}' if trimmed else '')
                    + f' ({time.time() - t_r:.0f} s)')
                if keep:
                    F, R, best_g = F1, R1, g1
                else:
                    best_g = g_round0
                continue
            # standing moves must not CONFLICT with each other (two nets asked
            # for one berth slot: the engine laid neither as asked, measured):
            # the better-graded one keeps its move, the other waits a round
            def _grade_key(t):
                pr = t[2]
                g = pr.get('grade') or [[None] * 99, 0, 10 ** 6]
                return (pr.get('refused', False), len(g[0]), g[2])
            kept = {}
            for nm in sorted(stand, key=lambda n: _grade_key(stand[n])):
                sm_, dm_, pr_ = stand[nm]
                clash = None
                for o, (so, do, _p) in kept.items():
                    if dm_ is not None and do is not None and pe.sm._conflict(dm_, do, strict=True):
                        clash = o
                    if sm_ is not None and so is not None and pe.sm._conflict(sm_, so, strict=True):
                        clash = o
                if clash:
                    log(f'  {nm}: its move conflicts with {clash}\'s -- deferred to a later round')
                    continue
                kept[nm] = stand[nm]
            stand = kept
            # ---- APPLY to the fanout board
            src_moves = {nm: s for nm, (s, d, _) in stand.items() if s is not None}
            dst_moves = {nm: d for nm, (s, d, _) in stand.items() if d is not None}
            log(f'  applying to {os.path.basename(F)}: source {list(src_moves)}, destination {list(dst_moves)}')
            F1 = f'{stem}_r{rnd}_fo.kicad_pcb'
            txt = open(F, encoding='utf-8').read()
            unfaithful = []
            if src_moves:
                # the moved nets' berths go first (one free end for the
                # measure; the berth is re-laid with the others below)
                txt = strip_window(txt, list(src_moves), B.byname, _pad(B.st['dgrid'].bbox))
                f1s = f'{stem}_r{rnd}_fo_srcstrip.kicad_pcb'
                write_board(txt, f1s, F)
                lines = []
                r = sr.realize(f1s, src_moves, B.st['src_pad'], B.byname, B.st['sref'],
                               f'{stem}_r{rnd}_fo_src.kicad_pcb', log=lines.append, guard_names=names)
                for l in lines:
                    if any(nm in l for nm in src_moves) or 'audit' in l or 'unmoved' in l or 'REJECTED' in l:
                        log('    ' + l.strip())
                if r['rejected']:
                    best_g = g_round0
                    log(f'  round {rnd}: source realize REJECTED ({r["rejected"]}) -- moves banned, F stays')
                    for nm, m in src_moves.items():
                        bans_s.add((nm, sr.move_sig(m)))
                    continue
                for nm, m in src_moves.items():
                    a = r['audit'][nm]
                    g = a.get('achieved')
                    if nm not in r['ok'] or g is None or (g['direction'], g['layer']) != (m.direction, m.layer):
                        unfaithful.append(f'{nm} tooth: asked {fmt_move(m)}, got {sr.fmt(g)} -- {a.get("verdict")}')
                        bans_s.add((nm, sr.move_sig(m)))
                    elif not a['exact']:
                        log(f'    {nm} tooth laid in class, not exact: {a.get("verdict")}')
                txt = open(r['board'], encoding='utf-8').read()
            write_board(txt, F1, F)
            # every changed net's berth re-laid against the rest (a source-moved
            # net's berth was stripped above)
            comoved = sorted({o for m in dst_moves.values() for o in (getattr(m, 'comove', []) or [])}
                             - set(dst_moves) - set(src_moves))
            changed = sorted(set(src_moves) | set(dst_moves) | set(comoved))
            if comoved:
                log(f'  co-moved berths (re-fanned with their neighbour, the engine negotiating): {comoved}')
            choice_all = dict(B.choice)
            choice_all.update(dst_moves)
            harvested = []
            for nm, (s_, d_, pr) in stand.items():
                if d_ is None:
                    continue
                for o, got in (pr.get('comove_got') or {}).items():
                    mv = (fp._menu_match(dmenu_full(B.st)[o], got) or synth_move(o, got)) if got else None
                    if mv is not None and sr.move_sig(mv) != sr.move_sig(choice_all.get(o, mv)):
                        choice_all[o] = mv
                        harvested.append(f'{o} -> {fmt_move(mv)}')
                got = pr.get('dst_got')
                if got and not pr.get('dst_exact'):
                    mv = fp._menu_match(dmenu_full(B.st)[nm], got) or synth_move(nm, got)
                    if mv is not None and (mv.direction, mv.layer) == (d_.direction, d_.layer):
                        choice_all[nm] = mv
                        dst_moves[nm] = mv
                        harvested.append(f'{nm} -> {fmt_move(mv)} (the probe\'s own laid gap)')
            if harvested:
                log(f'  asks harvested from the probes\' laid berths: {harvested}')
            ask = {nm: B.ends[nm]['dst']['direction'] for nm in changed if nm not in choice_all}
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(buf):
                laid, audit_d, ok = fp.fanout_once(F1, names, choice_all, B.st['dst_pad'], dref,
                                                   B.byname, F, relay=changed,
                                                   already=[nm for nm in names if nm not in changed],
                                                   face_asks=ask or None)
            with open(f'{stem}_r{rnd}_fo.log', 'w') as f:
                f.write(buf.getvalue())
            achieved = fp.fanout_once.achieved or {}
            for nm in changed:
                a = audit_d.get(nm, {})
                g = achieved.get(nm)
                want = choice_all.get(nm)
                if g is None:
                    unfaithful.append(f'{nm} berth: engine laid none')
                elif nm in comoved and want is not None \
                        and (g['direction'], g['layer']) != (want.direction, want.layer):
                    log(f'    {nm} berth CO-MOVED by the engine: {fmt_move(want)} -> {sr.fmt(g)}')
                elif want is not None and (g['direction'], g['layer']) != (want.direction, want.layer):
                    unfaithful.append(f'{nm} berth: asked {fmt_move(want)}, got {sr.fmt(g)} -- {a.get("verdict")}')
                    if nm in dst_moves:
                        bans_d[nm].add((want.direction, want.layer))
                elif want is not None and not a.get('exact'):
                    log(f'    {nm} berth laid in class, not exact: {a.get("verdict")}')
                else:
                    log(f'    {nm} berth {sr.fmt(g)}' + (' exact' if a.get('exact') else ' (engine\'s own)'))
            log(f'  fanout board {os.path.basename(F1)}: {"clean and complete" if ok else "NOT clean/complete"}'
                + (f'; UNFAITHFUL: ' + ' | '.join(unfaithful) if unfaithful else '; every move laid in its class'))
            if not ok or unfaithful:
                best_g = g_round0
                log(f'  round {rnd}: the fanout board is {"not clean" if not ok else "unfaithful"} -- '
                    f'no braid; the moves not laid as asked are banned, F stays')
                for nm, m in dst_moves.items():
                    if not ok or any(u.startswith(nm + ' ') for u in unfaithful):
                        bans_d[nm].add(_dban(m))
                for nm, m in src_moves.items():
                    if not ok or any(u.startswith(nm + ' ') for u in unfaithful):
                        bans_s.add((nm, sr.move_sig(m)))
                continue
            # the UNMOVED ends must be where they were
            B1 = Board(F1, names, dref, banned=frozenset(bans_s))
            drift = []
            for nm in names:
                if nm in changed:
                    continue
                for end in ('src', 'dst'):
                    a, b = B.ends[nm][end], B1.ends[nm][end]
                    if (a is None) != (b is None) or (a and (a['tooth'] != b['tooth'] or a['layer'] != b['layer']
                                                             or a['vias'] != b['vias'])):
                        drift.append(f'{nm}.{end}')
            log(f'  unmoved ends: {2 * (len(names) - len(changed)) - len(drift)}/{2 * (len(names) - len(changed))} '
                f'unchanged' + (f'; DRIFTED {drift}' if drift else ''))
            named1 = (named | set(changed)) & set(B1.choice)
            pred1, bp1, side = B1.write_sidecar(named1)
            nsw1 = sum(1 for nm in names if nm in bp1 and bp1[nm]['page'] is None)
            log(f'  sidecar {os.path.basename(side)}: {len(named1)} nets named; plan model '
                f'{sum(pred1.values())} vias (residual off), {nsw1} swimmers on paper')
            if MODE == 'incremental' and stand and R_cur != R:
                # the fanout board and the routed board must carry the SAME
                # ends for every changed net (both laid by the engine to the
                # same asks): then the probes' board ships as this round's
                # the routed board's ends for the changed nets are what the
                # probes' engine calls LAID there (measured then); the fanout
                # board's are measured now. Same face and layer, the gap within
                # END_AGREE (two engine runs to one ask land a gap apart)
                laid_on_r = {}
                for nm_, (s_, d_, pr) in stand.items():
                    if pr.get('src_got'):
                        laid_on_r[(nm_, 'src')] = pr['src_got']
                    if pr.get('dst_got'):
                        laid_on_r[(nm_, 'dst')] = pr['dst_got']
                    for o, g in (pr.get('comove_got') or {}).items():
                        if g:
                            laid_on_r[(o, 'dst')] = g
                mismatch = []
                for nm in changed:
                    for e in ('src', 'dst'):
                        a = B1.ends[nm][e]
                        b = laid_on_r.get((nm, e), B.ends[nm][e])   # unchanged at this end: as it was
                        if a is None or b is None or a['layer'] != b['layer'] \
                                or a['direction'] != b['direction'] \
                                or math.hypot(a['tooth'][0] - b['tooth'][0], a['tooth'][1] - b['tooth'][1]) > END_AGREE:
                            mismatch.append(f'{nm}.{e}: fanout {sr.fmt(a)} vs routed {sr.fmt(b)}')
                if mismatch:
                    log(f'  ends DIFFER between the fanout board and the routed board: {mismatch} '
                        f'-> the full braid decides')
                else:
                    R1 = f'{stem}_r{rnd}.kicad_pcb'
                    copy_board(R_cur, R1, eco=True)
                    for ext in ('.log', '.pack.json', '_refusals.json'):
                        src = R_cur[:-len('.kicad_pcb')] + ext
                        if os.path.exists(src):
                            shutil.copy(src, R1[:-len('.kicad_pcb')] + ext)
                    with open(R1[:-len('.kicad_pcb')] + '.census.json', 'w') as f:
                        json.dump(census_hist, f, indent=1, sort_keys=True)
                    g1, line = grade(R1, K, base)
                    keep = better(g1, g_round0) and not drift
                    log(f'  round {rnd}: {"KEPT" if keep else "rejected"} incremental -- open {g1[0]}, drc {g1[1]}, '
                        f'vias {g1[2]} (round start {g_round0[0]}/{g_round0[2]}); ends of {len(changed)} changed '
                        f'net(s) agree on both boards ({time.time() - t_r:.0f} s)')
                    if keep:
                        F, R, best_g = F1, R1, g1
                    else:
                        best_g = g_round0
                    continue
            # ---- BRAID
            R1 = f'{stem}_r{rnd}'
            okb, tb = braid_run(F1, R1, nets_csv, dest, R1 + '.log')
            R1 += '.kicad_pcb'
            if not okb:
                log(f'  round {rnd}: braid produced no board ({tb:.0f} s) -- F stays')
                continue
            g1, line = grade(R1, K, base)
            V1 = verdict(R1[:-len('.kicad_pcb')])
            nsw_b1 = sum(1 for nm in names if V1.get(nm, {}).get('cls') == 'swim')
            pl = re.search(r'^plan from .*$', open(R1[:-len('.kicad_pcb')] + '.log').read(), re.M)
            log(f'  braid {tb:.0f} s: open {g1[0]}, drc {g1[1]}, vias {g1[2]}, {nsw_b1} swimmers '
                f'(before: open {best_g[0]}, drc {best_g[1]}, vias {best_g[2]}, {nsw_braid}); '
                f'{pl.group(0) if pl else "no plan line in the braid log"}')
            for nm in changed:
                log(f'    {fmt_v(nm, V1.get(nm, {}), count_copper(R1, B.byname[nm][0])[0])}')
            best_g = g_round0
            keep = better(g1, best_g) and not unfaithful and not drift
            log(f'  round {rnd}: {"KEPT" if keep else "rejected"} '
                f'({"better" if better(g1, best_g) else "not better"}'
                f'{", unfaithful" if unfaithful else ""}{", drift" if drift else ""}) '
                f'({time.time() - t_r:.0f} s)')
            if keep:
                F, R, best_g = F1, R1, g1
            else:
                for nm, m in dst_moves.items():
                    bans_d[nm].add(_dban(m))
                for nm, m in src_moves.items():
                    bans_s.add((nm, sr.move_sig(m)))
        except Exception as _e:
            import traceback
            log(f'  round {rnd}: ABORTED -- {type(_e).__name__}: {str(_e)[:160]} '
                f'(the boards kept so far stand)')
            log('    ' + traceback.format_exc().strip().splitlines()[-1][:200])
            break
    final = stem + '.kicad_pcb'
    copy_board(R, final, eco=True)
    for ext in ('.log', '.pack.json', '_refusals.json', '.census.json'):
        src = R[:-len('.kicad_pcb')] + ext
        if os.path.exists(src):
            shutil.copy(src, final[:-len('.kicad_pcb')] + ext)
    with open(final[:-len('.kicad_pcb')] + '.census.json', 'w') as f:
        json.dump(census_hist, f, indent=1, sort_keys=True)
    fo_final = stem + '_fo.kicad_pcb'
    copy_board(F, fo_final)
    shutil.copy(F[:-len('.kicad_pcb')] + '.plan.json', fo_final[:-len('.kicad_pcb')] + '.plan.json')
    log(f'\nreplan: best open {best_g[0]} drc {best_g[1]} vias {best_g[2]} -> {os.path.basename(final)} '
        f'(fanout {os.path.basename(fo_final)}) ({time.time() - t_all:.0f} s)')
    log(pm.summary() + (f'; pool {PAR} worker(s), {POOL.restarts} restart(s)' if POOL is not None else ''))
    if POOL is not None:
        POOL.close()
    r = subprocess.run([sys.executable, os.path.join(HERE, 'grade_k.py'), final, nets_csv],
                       capture_output=True, text=True)
    print((r.stdout + r.stderr).strip())
    return 0


if __name__ == '__main__':
    sys.exit(main())
