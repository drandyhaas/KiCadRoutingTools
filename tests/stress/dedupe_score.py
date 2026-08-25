#!/usr/bin/env python3
"""Score a candidate .kicad_pcb against the existing corpus for duplication.

Two boards from the same designer (or a fork / a "v2" of the same product) look
new by filename but add nothing to the corpus. The similarity metric calibrated
on the 2026-08-09 Antmicro survey and reused for set28 is

    score = 0.5 * Jaccard(footprint library names) + 0.5 * Jaccard(net names)

with a **~0.30 admit bar**: at or above it, treat the candidate as a duplicate
of its best match and don't admit it. Real distinct boards score well under
0.1 even within one designer's portfolio (set28's intra-candidate worst was
0.046; the worst candidate-vs-corpus was 0.073).

This is a TEXT scan, not a parse: the corpus spans KiCad v5 through v10 and the
pure-Python parser is too slow to run over 400+ boards for a curation decision.
Both spellings of each token are handled, and getting that wrong silently HALVES
every score (an empty net set makes the second term 0):

  - nets     `(net 3 "GND")`  (v6+, id + name)  and  `(net "GND")`  (KiCad 10,
             id-less inside footprint pads)
  - footprints `(footprint "Lib:Name"` (v6+)    and  `(module Lib:Name`  (v5)

Usage:
  python3 dedupe_score.py candidate.kicad_pcb                 # vs $STRESS_DIR corpus
  python3 dedupe_score.py cand.kicad_pcb --corpus DIR [DIR..] # explicit dirs
  python3 dedupe_score.py cand.kicad_pcb --top 10 --bar 0.30
"""
import argparse
import os
import re
import sys
from pathlib import Path

NET_RE = re.compile(r'\(net\s+(?:\d+\s+)?"([^"]*)"')
FP_QUOTED_RE = re.compile(r'\(footprint\s+"([^"]+)"')
FP_BARE_RE = re.compile(r'\((?:footprint|module)\s+([^\s"()]+)')

# Nets present on every board carry no information about what the board IS.
GENERIC_NETS = {"", "gnd", "vcc", "vdd", "+3v3", "+3.3v", "+5v", "gndd", "gnda"}


def tokens(path):
    """(footprint-name set, net-name set) for one board file, case-folded."""
    text = Path(path).read_text(errors="replace")
    fps = set(m.lower() for m in FP_QUOTED_RE.findall(text))
    fps |= set(m.lower() for m in FP_BARE_RE.findall(text))
    nets = set(m.lower() for m in NET_RE.findall(text)) - GENERIC_NETS
    return fps, nets


def jaccard(a, b):
    if not a or not b:
        return 0.0
    return len(a & b) / len(a | b)


def score(cand, other):
    return 0.5 * jaccard(cand[0], other[0]) + 0.5 * jaccard(cand[1], other[1])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("candidate", nargs="+", help="candidate .kicad_pcb file(s)")
    ap.add_argument("--corpus", nargs="*", default=None,
                    help="corpus dirs (default: $STRESS_DIR/boards_unrouted*)")
    ap.add_argument("--top", type=int, default=5, help="matches to print per candidate")
    ap.add_argument("--bar", type=float, default=0.30, help="duplicate threshold")
    args = ap.parse_args()

    if args.corpus:
        dirs = [Path(d) for d in args.corpus]
    else:
        stress = Path(os.environ.get("STRESS_DIR", str(Path.home() / "Documents/kicad_stress_test")))
        dirs = sorted(stress.glob("boards_unrouted*"))
    corpus = sorted({p for d in dirs for p in Path(d).glob("*.kicad_pcb")})
    cand_paths = [Path(c).resolve() for c in args.candidate]
    corpus = [p for p in corpus if p.resolve() not in cand_paths]
    if not corpus:
        print(f"no corpus boards found under {[str(d) for d in dirs]}", file=sys.stderr)
        return 2
    print(f"corpus: {len(corpus)} boards from {len(dirs)} dirs")

    corpus_tok = [(p, tokens(p)) for p in corpus]
    worst = 0.0
    for c in cand_paths:
        ct = tokens(c)
        scored = sorted(((score(ct, t), p) for p, t in corpus_tok), reverse=True)
        # candidates are also compared against EACH OTHER (intra-batch dupes)
        for o in cand_paths:
            if o != c:
                scored.append((score(ct, tokens(o)), o))
        scored.sort(reverse=True)
        best = scored[0][0] if scored else 0.0
        worst = max(worst, best)
        verdict = "DUPLICATE" if best >= args.bar else "distinct"
        print(f"\n{c.name}: {len(ct[0])} fp-names, {len(ct[1])} nets -> "
              f"best {best:.3f} ({verdict}, bar {args.bar:.2f})")
        for s, p in scored[:args.top]:
            print(f"    {s:.3f}  {p.parent.name}/{p.stem}")
    return 1 if worst >= args.bar else 0


if __name__ == "__main__":
    sys.exit(main())
