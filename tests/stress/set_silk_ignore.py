#!/usr/bin/env python3
"""Demote the silkscreen DRC checks to 'ignore' in a board's .kicad_pro.

KiCad's silkscreen DRC (silk-over-copper / silk overlap / silk clipped by edge) is
O(n^2) and makes interactive DRC crawl for MINUTES on silk-dense boards -- keyboard
legends (lily58, crkbd, ...) especially. Our router never places silkscreen, so
these checks are pure cosmetic noise for a routed/stress board. Setting them to
'ignore' makes KiCad SKIP the test entirely (not just hide markers), so DRC is fast.

Used by the corpus prep scripts (prep_setN.sh) for silk-heavy boards, and runnable
by hand: set_silk_ignore.py board.kicad_pro [more.kicad_pro ...]. A .kicad_pcb path
is accepted too (its sibling .kicad_pro is edited). Missing/unreadable files are
skipped. Idempotent.
"""
import json
import os
import sys

SILK_KEYS = ('silk_edge_clearance', 'silk_over_copper', 'silk_overlap')


def set_silk_ignore(pro_path):
    """Set the silk DRC severities to 'ignore' in one .kicad_pro; return True if written."""
    if pro_path.endswith('.kicad_pcb'):
        pro_path = pro_path[:-len('.kicad_pcb')] + '.kicad_pro'
    if not os.path.isfile(pro_path):
        return False
    try:
        with open(pro_path, encoding='utf-8') as f:
            proj = json.load(f)
    except (OSError, json.JSONDecodeError):
        return False
    sev = (proj.setdefault('board', {})
               .setdefault('design_settings', {})
               .setdefault('rule_severities', {}))
    for k in SILK_KEYS:
        sev[k] = 'ignore'
    with open(pro_path, 'w', encoding='utf-8') as f:
        json.dump(proj, f, indent=2)
    return True


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 1
    any_written = False
    for path in argv[1:]:
        if set_silk_ignore(path):
            print(f"silk DRC -> ignore: {path}")
            any_written = True
        else:
            print(f"skipped (no .kicad_pro): {path}")
    return 0 if any_written else 1


if __name__ == '__main__':
    raise SystemExit(main(sys.argv))
