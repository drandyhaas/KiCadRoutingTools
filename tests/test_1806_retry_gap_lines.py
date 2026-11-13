#!/usr/bin/env python3
"""#1806: route.py's end-of-run retry passes say which gap they work on.

The rescue pass and the terminal geometry escalation retry every net a run
would ship open, gap by gap, and printed nothing per gap: a net's rung line,
then only the search's own "No route found after N iterations", then the
net's seconds after its last rung. A lap of tens of minutes could not be told
from a hung one. Each gap attempt now prints one line -- its two ends, its
length, what came of it, the searches and iterations it ran and the seconds
-- and the whole-net retry hands the same step to the GUI's progress
callback.

A. _attempt_net_at_geometry on a two-pad board: the gap line, the progress
   message and the iteration tally.
B. one F.Cu-only wildcard lap of an in-repo board, so nets fail and both
   passes run: every escalation rung line is followed by a gap line, the
   rescue prints gap lines, and an unrecovered net's line carries its
   iterations.
"""
import contextlib
import io
import os
import re
import shutil
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, HERE)

from run_utils import check as run_check, evidence  # noqa: E402

failures = []
GAP_RE = re.compile(r"^    gap \d+/\d+ \(-?[\d.]+, -?[\d.]+\) -> \(-?[\d.]+, -?[\d.]+\), "
                    r"[\d.]+ mm: [^;]+; \d+ search\(es\), \d+ iterations, [\d.]+s$")


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


BOARD = """(kicad_pcb (version 20240108) (generator "test")
  (general (thickness 1.6))
  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))
  (net 0 "")
  (net 1 "SIG")
  (footprint "test:pad" (layer "F.Cu") (at 5 10)
    (property "Reference" "R1" (at 0 -2 0) (layer "F.SilkS") (effects (font (size 1 1) (thickness 0.15))))
    (pad "1" smd rect (at 0 0) (size 0.6 0.6) (layers "F.Cu") (net 1 "SIG")))
  (footprint "test:pad" (layer "F.Cu") (at 15 10)
    (property "Reference" "R2" (at 0 -2 0) (layer "F.SilkS") (effects (font (size 1 1) (thickness 0.15))))
    (pad "1" smd rect (at 0 0) (size 0.6 0.6) (layers "F.Cu") (net 1 "SIG")))
  (gr_rect (start 0 0) (end 20 20) (layer "Edge.Cuts") (stroke (width 0.1) (type solid)))
)
"""

print("A. one whole-net retry on a two-pad board")
import net_rescue  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402
from routing_config import GridRouteConfig  # noqa: E402

tmp = tempfile.mkdtemp(prefix='gap1806_')
try:
    path = os.path.join(tmp, 'b.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(BOARD)
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(path)
    sig = next(i for i, n in pcb.nets.items() if n.name == 'SIG')
    cfg = GridRouteConfig(grid_step=0.1, clearance=0.2, track_width=0.2,
                          via_size=0.6, via_drill=0.3, layers=['F.Cu'])
    stats, msgs, log = {}, [], io.StringIO()
    with contextlib.redirect_stdout(log):
        edges, fully = net_rescue._attempt_net_at_geometry(
            pcb, sig, cfg, None, stats=stats, progress=msgs.append)
    lines = [ln for ln in log.getvalue().splitlines() if GAP_RE.match(ln)]
    check("the gap joins the net", fully and len(edges) == 1)
    check("one gap line naming both ends, the outcome and its cost",
          len(lines) == 1 and '(5.000, 10.000) -> (15.000, 10.000), 10.00 mm'
          in lines[0] and 'joined (2 -> 1 parts)' in lines[0], log.getvalue())
    check("the progress callback hears the gap", msgs == ['gap 1/8'], str(msgs))
    check("the iterations are tallied for the caller",
          stats.get('searches') == 1 and stats.get('iterations', 0) > 0,
          str(stats))
finally:
    shutil.rmtree(tmp, ignore_errors=True)

print("B. one F.Cu-only wildcard lap, so nets fail and both passes run")
SRC = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
evidence(SRC, 'the in-repo board the lap routes')
tmp = tempfile.mkdtemp(prefix='gap1806_')
try:
    inp = os.path.join(tmp, 'in.kicad_pcb')
    run_check([sys.executable, '-X', 'utf8',
               os.path.join(ROOT, 'py_router', 'copy_board.py'), SRC, inp],
              accept=True)
    r = run_check([sys.executable, '-X', 'utf8',
                   os.path.join(ROOT, 'py_router', 'route.py'), inp,
                   os.path.join(tmp, 'out.kicad_pcb'), '--nets', '*',
                   '--layers', 'F.Cu'], accept=True, timeout=1200)
    log = r.stdout.splitlines()
finally:
    shutil.rmtree(tmp, ignore_errors=True)
rung = [i for i, ln in enumerate(log) if re.search(r'\(rung \d+/\d+\)$', ln)]
check("the lap ran the terminal escalation", len(rung) > 0)


def _followed_by_gap(i):
    for ln in log[i + 1:]:
        if GAP_RE.match(ln):
            return True
        if re.search(r'\(rung \d+/\d+\)$', ln) or 'terminal escalation: ' in ln \
                and ('unrecovered' in ln or 'recovered at' in ln):
            return False
    return False


missing = [log[i] for i in rung if not _followed_by_gap(i)]
check("every escalation rung line is followed by a gap line",
      not missing, '; '.join(missing[:3]))
res = []
on = False
for ln in log:
    if ln.startswith('Per-net fine-parameter rescue'):
        on = True
    elif ln.startswith('Rescue pass:'):
        on = False
    elif on and GAP_RE.match(ln):
        res.append(ln)
check("the rescue pass prints its gap attempts", len(res) > 0)
unrec = [ln for ln in log if 'terminal escalation: ' in ln and 'unrecovered' in ln]
check("an unrecovered net's line carries its searches and iterations",
      unrec and all(re.search(r'\d+ search\(es\), \d+ iterations\)', ln)
                    for ln in unrec), '; '.join(unrec[:2]))

print()
if failures:
    print(f"{len(failures)} FAILURE(S): " + "; ".join(failures))
    sys.exit(1)
print("All #1806 retry gap-line checks passed")
