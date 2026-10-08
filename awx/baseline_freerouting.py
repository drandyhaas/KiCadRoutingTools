#!/usr/bin/env python3
"""baseline_freerouting.py route IN.kicad_pcb NETS SRC OUTDIR [--no-fanout] [--reimport] -- one rung by Freerouting,
for baseline_bench.py: IN (the bench prepared) out as a Specctra DSN with only the rung's NETS (short names, comma
separated) left to route, Freerouting on it with its defaults (its fanout and optimizer; --no-fanout turns the fanout
off), and its session back into IN as OUTDIR/out.kicad_pcb. --reimport takes the session already in OUTDIR.
Needs FREEROUTING_JAR and FREEROUTING_JAVA (2.4.1 wants Java 25): github.com/freerouting/freerouting/releases,
adoptium.net; and KiCad's own python (pcbnew; KICAD_PYTHON, else the usual install paths), under which this file runs
its two KiCad halves itself:

  export IN.kicad_pcb OUT.dsn NETS SRC   every other net keeps only its pads on SRC that its own copper reaches (the
      bench's escape stubs, complete as they stand), its other pads go netless (still obstacles), and its tracks and
      vias are locked (written as fixed). Freerouting 2.4.1 has no working way to leave a net class alone (its
      `ignore_net_classes` is a transient field, dropped when its settings sources merge), so the other nets are
      given nothing to route instead.
  import IN.kicad_pcb IN.ses OUT.kicad_pcb NETS   the board graded: IN's own copper of every OTHER net exactly as it
      stands, plus the rung's copper from the session. The session hands back the fixed wires reshaped and not the
      fixed vias, and KiCad's import replaces every track, so the other nets' copper is taken from IN; a rung via the
      session puts where IN has a via of the same net is IN's own via (the session drops its filled + capped spec).

IN is never written."""
KRT_TOOL = {'scope': [], 'kind': 'actor'}   # #937: a research tool (awx), catalogued, shown at no door

import argparse
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
from kicad_locate import kicad_python_candidates   # noqa: E402
TIMEOUT = 4 * 3600


def short(name):
    return (name or '').split('/')[-1]


def kicad_python():
    """KiCad's own python (pcbnew): KICAD_PYTHON, else every install kicad_locate finds, newest first"""
    for py in kicad_python_candidates():
        if py and os.path.isfile(py) and subprocess.run([py, '-c', 'import pcbnew'], capture_output=True).returncode == 0:
            return py
    sys.exit('baseline_freerouting: no python with pcbnew found; set KICAD_PYTHON')


def route(board, nets, src, out, fanout=True, reimport=False):
    """{step: seconds}: export, Freerouting and import, each logged in OUTDIR/<step>.log"""
    kpy, me = kicad_python(), os.path.abspath(__file__)
    I = lambda f: os.path.join(out, f)
    timing = {}

    def step(name, argv, env=None, timeout=None):
        t = time.time()
        with open(I(name + '.log'), 'w') as f:
            try:
                subprocess.run(argv, cwd=HERE, stdout=f, stderr=subprocess.STDOUT, env=env, timeout=timeout)
            except subprocess.TimeoutExpired:
                f.write('\nTIMEOUT\n')
        timing[name] = round(time.time() - t, 1)

    os.makedirs(out, exist_ok=True)
    if not reimport:
        jar, java = os.environ.get('FREEROUTING_JAR'), os.environ.get('FREEROUTING_JAVA', 'java')
        if not jar or not os.path.isfile(jar):
            sys.exit('baseline_freerouting: set FREEROUTING_JAR (and FREEROUTING_JAVA: 2.4.1 wants Java 25)')
        step('export', [kpy, me, 'export', board, I('in.dsn'), ','.join(nets), src])
        env = dict(os.environ, FREEROUTING__GUI__ENABLED='false')
        if not fanout:
            env['FREEROUTING__ROUTER__FANOUT__ENABLED'] = 'false'
        step('freerouting', [java, '-Djava.awt.headless=true', '-Xmx2g', '-jar', jar, '-de', I('in.dsn'),
                             '-do', I('out.ses'), '-mp', '100'], env=env, timeout=TIMEOUT)
    if os.path.exists(I('out.kicad_pcb')):
        os.remove(I('out.kicad_pcb'))                  # a failed import leaves no board to grade
    step('import', [kpy, me, 'import', board, I('out.ses'), I('out.kicad_pcb'), ','.join(nets)])
    return timing


# ------------------------------------------------------------------ the two KiCad halves (under KiCad's own python)
def canonical_layers(pcbnew, board):
    """the copper layers under their canonical names (F.Cu, B.Cu, In1.Cu ..) for the round trip; {layer id: the
    board's own name}, to put back. KiCad writes a layer's user name into the DSN ("Top Layer" on the zynq bench, an
    Altium import) and its session import then refuses the board (ImportSpecctraSES False, nothing laid)"""
    own = {}
    for lid in board.GetEnabledLayers().CuStack():
        std = pcbnew.LSET.Name(lid) if hasattr(pcbnew.LSET, 'Name') else pcbnew.BOARD.GetStandardLayerName(lid)
        if board.GetLayerName(lid) != std:
            own[lid] = board.GetLayerName(lid)
            board.SetLayerName(lid, std)
    return own


def name_unnamed(board):
    """the footprints with no reference, named NOREF1.. in file order (the same names on export and import): the
    exporter refuses a footprint with no reference (the zynq bench's twelve one-pad parts)"""
    noref = [fp for fp in board.GetFootprints() if not fp.GetReference()]
    for i, fp in enumerate(noref):
        fp.SetReference(f'NOREF{i + 1}')
    return noref


def export(pcbnew, board, out, want, src):
    tracks = list(board.GetTracks())
    reached = set()                                    # (ref, pad) a track or via of the pad's own net touches
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            if pad.GetNetCode() <= 0:
                continue
            for t in tracks:
                if t.GetNetCode() == pad.GetNetCode() and (pad.HitTest(t.GetStart()) or pad.HitTest(t.GetEnd())):
                    reached.add((fp.GetReference(), pad.GetNumber()))
                    break
    kept = cut = 0
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            if pad.GetNetCode() <= 0 or short(pad.GetNetname()) in want:
                continue
            if fp.GetReference() == src and (fp.GetReference(), pad.GetNumber()) in reached:
                kept += 1
                continue
            pad.SetNetCode(0)
            cut += 1
    locked = 0
    for t in tracks:
        if short(t.GetNetname()) not in want:
            t.SetLocked(True)
            locked += 1
    noref = name_unnamed(board)
    renamed = canonical_layers(pcbnew, board)
    # a rule area forbidding only zone fills constrains no route (the zynq bench's nineteen, west of the source)
    fill_only = [z for z in board.Zones()
                 if z.GetIsRuleArea() and not z.GetDoNotAllowTracks() and not z.GetDoNotAllowVias()]
    for z in fill_only:
        board.RemoveNative(z)
    print(f'other nets: {kept} pads kept on {src}, {cut} pads netless, {locked} tracks/vias locked; '
          f'{len(noref)} unnamed footprints named, {len(fill_only)} fill-only rule areas dropped, '
          f'{len(renamed)} copper layers under their canonical names')
    ok = pcbnew.ExportSpecctraDSN(board, out)
    print('export', ok)
    return ok


def without_placement(ses):
    """the session without its placement section, beside it: Freerouting moves no part, and KiCad's import refuses
    the zynq bench's placement outright (ImportSpecctraSES False, nothing laid) while taking its routes"""
    s = open(ses).read()
    i = s.find('(placement')
    if i < 0:
        return ses
    depth, j = 0, i
    while True:
        depth += {'(': 1, ')': -1}.get(s[j], 0)
        if depth == 0:
            break
        j += 1
    out = ses[:-len('.ses')] + '_routes.ses'
    open(out, 'w').write(s[:i] + s[j + 1:])
    return out


def import_(pcbnew, board, in_path, ses, out, want):
    name_unnamed(board)
    own = canonical_layers(pcbnew, board)
    ok = pcbnew.ImportSpecctraSES(board, without_placement(ses))
    print('import', ok)
    if not ok:
        return False                                   # nothing laid: never grade the bench as the router's board
    for lid, name in own.items():
        board.SetLayerName(lid, name)
    dropped = [t for t in board.GetTracks() if short(t.GetNetname()) not in want]
    for t in dropped:
        board.RemoveNative(t)
    orig = pcbnew.LoadBoard(in_path)
    added = 0
    for t in orig.GetTracks():
        if short(t.GetNetname()) in want:
            continue
        d = t.Duplicate()
        board.Add(d)
        d.SetNet(board.FindNet(t.GetNetname()))
        added += 1
    key = lambda v: (v.GetNetname(), round(pcbnew.ToMM(v.GetPosition().x), 3), round(pcbnew.ToMM(v.GetPosition().y), 3))
    bench_vias = {key(v): v for v in orig.GetTracks() if v.Type() == pcbnew.PCB_VIA_T and short(v.GetNetname()) in want}
    kept = 0
    for v in [t for t in board.GetTracks() if t.Type() == pcbnew.PCB_VIA_T and short(t.GetNetname()) in want]:
        b = bench_vias.get(key(v))
        if b is not None:
            board.RemoveNative(v)
            d = b.Duplicate()
            board.Add(d)
            d.SetNet(board.FindNet(b.GetNetname()))
            kept += 1
    print(f'other nets: {len(dropped)} session items dropped, {added} bench items restored; '
          f'rung: {kept} vias at a bench via of their own net restored as the bench\'s')
    pcbnew.SaveBoard(out, board)
    print('saved', out)
    return ok


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    sub = ap.add_subparsers(dest='cmd', required=True)
    r = sub.add_parser('route')
    r.add_argument('board'), r.add_argument('nets'), r.add_argument('src'), r.add_argument('outdir')
    r.add_argument('--no-fanout', action='store_true')
    r.add_argument('--reimport', action='store_true')
    e = sub.add_parser('export')
    e.add_argument('board'), e.add_argument('dsn'), e.add_argument('nets'), e.add_argument('src')
    i = sub.add_parser('import')
    i.add_argument('board'), i.add_argument('ses'), i.add_argument('out'), i.add_argument('nets')
    a = ap.parse_args(argv)
    want = {n for n in a.nets.split(',') if n}
    if a.cmd == 'route':
        print(route(os.path.abspath(a.board), sorted(want), a.src, os.path.abspath(a.outdir), not a.no_fanout,
                    a.reimport))
        return 0
    import pcbnew                                      # KiCad's python only; after argparse, so --help needs none
    board = pcbnew.LoadBoard(a.board)
    if a.cmd == 'export':
        return 0 if export(pcbnew, board, a.dsn, want, a.src) else 1
    return 0 if import_(pcbnew, board, a.board, a.ses, a.out, want) else 1


if __name__ == '__main__':
    sys.exit(main())
