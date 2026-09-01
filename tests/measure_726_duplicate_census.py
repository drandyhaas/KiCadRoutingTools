#!/usr/bin/env python3
"""#726 measurement: what duplicate footprint references actually cost.

`PCBData.footprints` is a dict keyed by reference, so two footprint blocks
sharing one reference silently overwrite (last wins) -- text path
`kicad_parser.py:2785`, pcbnew path `kicad_parser.py:4644`. This script
measures the consequence on the boards this repo TRACKS, before anything is
built. It is a measurement, not a gate: it is deliberately NOT named `test_*`
so `tests/run_all.py`'s glob never collects it.

Four tables:

  A  census -- blocks vs parsed, and the pad asymmetry. The dropped block's
     pads are appended to `pads_by_net` BEFORE the dict overwrite
     (`kicad_parser.py:2774-2783`), so they survive in the net model with no
     reachable Footprint. That is the sharp number: the two halves of one
     PCBData disagree about how many pads the board has.

  B  what the twins ARE -- pose, side, lock state, courtyard, library. This
     table is what forbids "keep any one of them": on ulx3s, orangecrab and
     glasgow the group members sit on DIFFERENT SIDES, and glasgow's seven
     REF** are 3 locked + 4 unlocked kikit panel tabs.

  C  key-scheme feasibility -- does `board.GetFootprints()` iterate in file
     order? A file-order ordinal is only a viable key if both parse paths can
     compute it independently and agree. Needs KiCad's bundled python; run
     with --pcbnew (or let it auto-detect). Also reports duplicate UUIDs,
     which is why a bare-uuid key is NOT viable.

  D  collision surface -- the proposed key for every duplicate block, and
     whether it survives `part_class`'s `^([A-Za-z]+)` prefix classifier.

The board set comes from `run_utils.corpus_boards()` (git ls-files), never a
directory glob: `kicad_files/` accumulates gitignored GENERATED boards, so a
glob returns 22 entries on a clean clone and 33+ on a worn one.

    python3 -X utf8 tests/measure_726_duplicate_census.py
    python3 -X utf8 tests/measure_726_duplicate_census.py --pcbnew
"""
import argparse
import collections
import glob as _glob
import os
import re
import subprocess
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for _p in ('', 'py_router', 'py_placer', 'py_tools'):
    _d = os.path.join(ROOT, _p)
    if _d not in sys.path:
        sys.path.insert(0, _d)

sys.path.insert(0, TESTS_DIR)
from run_utils import corpus_boards                              # noqa: E402
from kicad_parser import (parse_kicad_pcb,                       # noqa: E402
                          find_matching_paren)

SKIP_EXIT = 77

#: Same candidate list and order as tests/gui_parity/test_ref_label_pcbnew_parity.py.
KICAD_PYTHONS = [
    "/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3",
    "/usr/bin/python3",
    os.path.expandvars(r"C:\Program Files\KiCad\bin\python.exe"),
    *sorted(_glob.glob(r"C:\Program Files\KiCad\*\bin\python.exe"), reverse=True),
]

#: KiCad 8+ property form, then the KiCad 6/7 fallback. Both require at least
#: one character, exactly as extract_footprints_and_pads does -- so an EMPTY
#: `(property "Reference" "")` falls through to the uuid branch here too.
_REF_RE = re.compile(r'\(property\s+"Reference"\s+"([^"]+)"')
_REF_LEGACY_RE = re.compile(r'\(fp_text\s+reference\s+"([^"]+)"')
_UUID_RE = re.compile(r'\(uuid\s+"([^"]+)"')
_AT_RE = re.compile(r'\(at\s+([\d.-]+)\s+([\d.-]+)(?:\s+([\d.-]+))?\)')
_LAYER_RE = re.compile(r'\(layer\s+"([^"]+)"\)')
_FPNAME_RE = re.compile(r'\(footprint\s+"([^"]*)"')


def _header_end(fp_text):
    """First child element, so a PAD's uuid cannot win the footprint's.

    Mirrors kicad_parser.py:2529-2533 exactly. The reference-less branch at
    :2475 uses an UNBOUNDED search; the two agree on every corpus board, and
    the bounded form is the one this measurement (and the fix) uses.
    """
    end = len(fp_text)
    for tok in ('(pad', '(fp_', '(zone', '(model', '(property'):
        i = fp_text.find(tok)
        if i != -1:
            end = min(end, i)
    return end


def blocks(content):
    """(start, end, fp_text) for every footprint block, in FILE ORDER.

    Uses the shipped string-aware `find_matching_paren`, not a depth counter:
    a lone paren inside a property value (an MPN like "TCR2EF115,LM(CT")
    otherwise runs the scan into the next footprint (#113).
    """
    out = []
    for m in re.finditer(r'\(footprint\s+"', content):
        s = m.start()
        e = find_matching_paren(content, s)
        out.append((s, e, content[s:e]))
    return out


def raw_reference(fp_text):
    """The reference the parser would derive, BEFORE any disambiguation."""
    m = _REF_RE.search(fp_text) or _REF_LEGACY_RE.search(fp_text)
    if m:
        return m.group(1)
    u = _UUID_RE.search(fp_text[:_header_end(fp_text)])
    return "#" + u.group(1) if u else "?"


def fp_uuid(fp_text):
    u = _UUID_RE.search(fp_text[:_header_end(fp_text)])
    return u.group(1) if u else ""


def pose(fp_text):
    m = _AT_RE.search(fp_text)
    if not m:
        return (0.0, 0.0, 0.0)
    return (float(m.group(1)), float(m.group(2)),
            float(m.group(3)) if m.group(3) else 0.0)


def _read(path):
    with open(path, encoding='utf-8', errors='replace') as fh:
        return fh.read()


def table_a(paths):
    print("\n== TABLE A -- census (blocks vs parsed, and the pad asymmetry) ==\n")
    hdr = ('%-34s %6s %6s %5s  %-22s %6s %6s %7s'
           % ('board', 'blocks', 'parsed', 'lost', 'duplicate refs',
              'fp_pad', 'netpad', 'orphans'))
    print(hdr)
    print('-' * len(hdr))
    rows = []
    tot_lost = tot_orphan = 0
    for p in paths:
        content = _read(p)
        bs = blocks(content)
        refs = [raw_reference(t) for _, _, t in bs]
        counts = collections.Counter(refs)
        dups = {r: c for r, c in counts.items() if c > 1}
        pcb = parse_kicad_pcb(p)
        n_fp = len(pcb.footprints)
        lost = len(bs) - n_fp
        # A pad reachable from the dict vs a pad the net model holds. The
        # difference is the dropped block's copper: still an obstacle to the
        # router, invisible to every placement instrument.
        fp_pads = sum(len(f.pads) for f in pcb.footprints.values())
        net_pads = sum(len(v) for v in pcb.pads_by_net.values())
        orphans = net_pads - fp_pads
        tot_lost += lost
        tot_orphan += orphans
        rows.append((os.path.basename(p), len(bs), n_fp, lost, dups,
                     fp_pads, net_pads, orphans))
        if dups or orphans:
            print('%-34s %6d %6d %5d  %-22s %6d %6d %7d'
                  % (os.path.basename(p)[:34], len(bs), n_fp, lost,
                     ', '.join('%s x%d' % kv for kv in sorted(dups.items()))[:22],
                     fp_pads, net_pads, orphans))
    clean = [r for r in rows if not r[4] and r[7] == 0]
    print('\n%d board(s) with a duplicate reference or an orphaned pad; '
          '%d clean.' % (len(rows) - len(clean), len(clean)))
    print('TOTAL footprints lost: %d.  TOTAL orphaned pads: %d.'
          % (tot_lost, tot_orphan))
    print('Clean controls: %s'
          % ', '.join(r[0].replace('.kicad_pcb', '') for r in clean[:6]))
    return rows


def table_b(paths):
    print("\n== TABLE B -- what the twins ARE (side, lock, courtyard, library) ==\n")
    hdr = ('%-20s %-7s %4s %9s %9s %6s %-5s %-6s %-5s %-4s %3s  %s'
           % ('board', 'ref', 'idx', 'x', 'y', 'rot', 'layer', 'locked',
              'CrtYd', 'Fab', 'pad', 'footprint_name'))
    print(hdr)
    print('-' * len(hdr))
    any_row = False
    for p in paths:
        content = _read(p)
        bs = blocks(content)
        refs = [raw_reference(t) for _, _, t in bs]
        counts = collections.Counter(refs)
        for idx, ((_, _, t), ref) in enumerate(zip(bs, refs)):
            if counts[ref] < 2:
                continue
            any_row = True
            x, y, rot = pose(t)
            lay = _LAYER_RE.search(t)
            name = _FPNAME_RE.search(t)
            # Footprint-level (locked yes) only: bound to the header so a
            # locked PAD does not read as a locked footprint (parser :2489-2497).
            locked = bool(re.search(r'\(locked\s+yes\)', t[:_header_end(t)]))
            print('%-20s %-7s %4d %9.3f %9.3f %6.1f %-5s %-6s %-5s %-4s %3d  %s'
                  % (os.path.basename(p).replace('.kicad_pcb', '')[:20], ref,
                     idx, x, y, rot,
                     (lay.group(1) if lay else '?')[:5],
                     str(locked), str('.CrtYd"' in t), str('.Fab"' in t),
                     len(re.findall(r'\(pad\s+"', t)),
                     (name.group(1) if name else '?')))
    if not any_row:
        print('(no duplicate references in the tracked corpus)')


class _StubFp:
    """The minimum `classify_part` reads, built per BLOCK.

    Deliberately not `pcb.footprints[...]`: that dict is the thing under
    measurement and holds only the surviving twin, so classifying through it
    would compare a block against itself. `classify_part` reads exactly
    `footprint_name`, `pads[].pad_type` and `pads[].pinfunction` (part_class.py
    :111-170), so a stub carrying those is a faithful input.
    """
    __slots__ = ('footprint_name', 'pads')

    def __init__(self, fp_text):
        m = _FPNAME_RE.search(fp_text)
        self.footprint_name = m.group(1) if m else ''
        self.pads = [_StubPad(t) for t in
                     re.findall(r'\(pad\s+"[^"]*"\s+(\w+)\s', fp_text)]


class _StubPad:
    __slots__ = ('pad_type', 'pinfunction')

    def __init__(self, pad_type):
        self.pad_type = pad_type
        # Not read from the block: a duplicate-reference part in this corpus
        # has at most one pad and no pinfunction. Stated rather than faked --
        # the receptacle rules below are therefore inert for this table.
        self.pinfunction = ''


def table_d(paths):
    """The proposed key, and whether the part classifier survives it."""
    print("\n== TABLE D -- collision surface (proposed key, prefix class) ==\n")
    from placement.part_class import classify_part
    hdr = ('%-20s %-7s %4s  %-10s %-10s %-10s %s'
           % ('board', 'ref', 'idx', 'proposed', 'prefix_in', 'prefix_out',
              'class_changes?'))
    print(hdr)
    print('-' * len(hdr))
    pfx = re.compile(r'^([A-Za-z]+)')
    for p in paths:
        content = _read(p)
        bs = blocks(content)
        refs = [raw_reference(t) for _, _, t in bs]
        keys = disambiguate(refs)
        counts = collections.Counter(refs)
        for idx, ((_, _, t), ref, key) in enumerate(zip(bs, refs, keys)):
            if counts[ref] < 2:
                continue
            a = pfx.match(ref)
            b = pfx.match(key)
            changed = (a.group(1) if a else None) != (b.group(1) if b else None)
            note = 'PREFIX CHANGED' if changed else 'no'
            stub = _StubFp(t)
            ca = classify_part(stub, ref).name
            cb = classify_part(stub, key).name
            if ca != cb:
                note = 'CLASS CHANGED %s -> %s' % (ca, cb)
            elif not changed:
                note = 'no (class %s)' % ca
            print('%-20s %-7s %4d  %-10s %-10s %-10s %s'
                  % (os.path.basename(p).replace('.kicad_pcb', '')[:20], ref,
                     idx, key, a.group(1) if a else '-',
                     b.group(1) if b else '-', note))
    # The other half of the claim: a UNIQUE key must be byte-identical.
    moved = 0
    for p in paths:
        refs = [raw_reference(t) for _, _, t in blocks(_read(p))]
        counts = collections.Counter(refs)
        for ref, key in zip(refs, disambiguate(refs)):
            if counts[ref] == 1 and key != ref:
                moved += 1
    print('\nUnique references whose key would change: %d '
          '(must be 0 -- the scheme must not perturb a non-colliding name).'
          % moved)


def disambiguate(refs):
    """The proposed key scheme, standalone -- file-order ordinal suffix.

    This is a MEASUREMENT copy so the table can be produced before the engine
    change exists. Phase 1 puts the real one in `kicad_parser`; when it lands,
    this function is deleted and the import replaces it (a second permanent
    implementation is exactly the fork the verifier is told to grep for).
    """
    taken = set(refs)
    issued = set()
    seen = collections.Counter()
    out = []
    for r in refs:
        seen[r] += 1
        if seen[r] == 1:
            out.append(r)
            issued.add(r)
            continue
        n = seen[r]
        cand = '%s~%d' % (r, n)
        while cand in taken or cand in issued:
            n += 1
            cand = '%s~%d' % (r, n)
        out.append(cand)
        issued.add(cand)
    return out


_PCBNEW_PROBE = r'''
import os, re, sys
sys.path.insert(0, sys.argv[1])
sys.path.insert(0, os.path.join(sys.argv[1], "py_router"))
sys.path.insert(0, os.path.join(sys.argv[1], "tests"))
import pcbnew
from measure_726_duplicate_census import blocks, raw_reference, fp_uuid, _read
print("KiCad build: %s" % pcbnew.GetBuildVersion())
hdr = ("%-30s %7s %7s %-14s %-12s %s"
       % ("board", "blocks", "live", "order==file", "ref_seq_eq", "duplicate_uuids"))
print(hdr); print("-" * len(hdr))
for p in sys.argv[2:]:
    bs = blocks(_read(p))
    file_uu = [fp_uuid(t) for _, _, t in bs]
    file_rf = [raw_reference(t) for _, _, t in bs]
    bd = pcbnew.LoadBoard(p)
    live = list(bd.GetFootprints())
    live_uu = [f.m_Uuid.AsString() for f in live]
    live_rf = [(f.GetReference() or ("#" + f.m_Uuid.AsString())) for f in live]
    dup_uu = sorted({u for u in file_uu if file_uu.count(u) > 1 and u})
    print("%-30s %7d %7d %-14s %-12s %s"
          % (os.path.basename(p)[:30], len(bs), len(live),
             str(file_uu == live_uu), str(file_rf == live_rf),
             (", ".join(u[:8] for u in dup_uu) or "-")))
'''


def table_c(paths, want):
    print("\n== TABLE C -- key-scheme feasibility, both parse paths ==\n")
    kp = next((c for c in KICAD_PYTHONS
               if os.path.exists(c)
               and subprocess.run([c, '-c', 'import pcbnew'],
                                  capture_output=True).returncode == 0), None)
    if kp is None:
        print('NOT RUN: no python with pcbnew found. Table C is the fact the '
              'ordinal key scheme rests on -- it must be produced before '
              'Phase 1 is trusted. Re-run with KiCad installed.')
        return False
    if not want:
        print('(skipped; pass --pcbnew to run. Found: %s)' % kp)
        return False
    probe = os.path.join(TESTS_DIR, '_726_pcbnew_probe.py')
    with open(probe, 'w', encoding='utf-8') as fh:
        fh.write(_PCBNEW_PROBE)
    try:
        r = subprocess.run([kp, '-X', 'utf8', probe, ROOT] + list(paths),
                           capture_output=True, text=True)
    finally:
        os.remove(probe)
    # pcbnew's wx image-handler chatter is unconditional; drop it.
    for line in (r.stdout or '').splitlines():
        if 'duplicate image handler' in line:
            continue
        print(line)
    if r.returncode != 0:
        print('probe exited %d\n%s' % (r.returncode, (r.stderr or '')[-2000:]))
        return False
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--pcbnew', action='store_true',
                    help='also run Table C under KiCad\'s bundled python')
    args = ap.parse_args()

    paths = corpus_boards()
    if not paths:
        print('SKIP: run_utils.corpus_boards() returned nothing -- git could '
              'not answer, so there is no identified board set to measure. '
              'A census over an unidentified set is not a measurement.')
        return SKIP_EXIT
    print('#726 duplicate-reference census over %d git-TRACKED boards.' % len(paths))

    table_a(paths)
    table_b(paths)
    table_d(paths)
    table_c(paths, args.pcbnew)
    return 0


if __name__ == '__main__':
    sys.exit(main())
