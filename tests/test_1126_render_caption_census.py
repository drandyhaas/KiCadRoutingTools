#!/usr/bin/env python3
"""#1126: render_placement's caption prints the courtyard CENSUS.

The caption printed `overlap {metrics.overlap_area}` -- the quench's rect
courtyards, which a #1104 project waiver zeroes -- while the checklist's
`b_courtyard_overlap_mm2`, the panel and (since #1124) the film carry
`grade_body_overlap`'s census. So a film frame and the render of the same
board disagreed: glasgow_revC's caption read 70.05 mm2 against 52.252.

* glasgow_revC: the caption carries the census (52.25), not 70.05 -- and the
  fixture arm refuses if either number drifts, so the case cannot pass by
  the two happening to agree;
* the same board with `courtyards_overlap = ignore` in its project: the
  optimizer's metric drops to 0 under the waiver, the caption still prints
  the census;
* no census -- `grade_body_overlap` raising, or a model with no pcb --
  prints `n/a`, never the empty default's 0.00;
* `describe_pair` reports the census and labels the quench's number as the
  optimizer's, in its text and its JSON.

    python3 -X utf8 tests/test_1126_render_caption_census.py [case ...]
"""
import json
import os
import shutil
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import run_utils  # noqa: E402

ROOT = run_utils.ROOT_DIR
for _sub in ('py_router', 'py_tools', 'py_placer'):
    _p = os.path.join(ROOT, _sub)
    if _p not in sys.path:
        sys.path.insert(0, _p)

import render_placement as RP                      # noqa: E402
from kicad_parser import parse_kicad_pcb           # noqa: E402

RUN_ALL_TIMEOUT = 600
GLASGOW = os.path.join(ROOT, 'kicad_files', 'glasgow_revC.kicad_pcb')


def _model(board=GLASGOW):
    return RP.PlacementModel(parse_kicad_pcb(board), board)


#: glasgow_revC's courtyard census. 52.252 until #1206: 6.2806 mm2 of it was
#: J1 <-> TP13 and J1 <-> TP15 (3.1403 each), two test points between J1's
#: drilled-pad clusters, under the single far-side box drawn over all of them.
#: 45.9714 until J4's F.CrtYd closed (fa10 P1: its ends miss by 8 um at one
#: corner, KiCad chains them; the polygon is 0.0045 mm2 less than the hull
#: that stood in for it).
GLASGOW_CENSUS = 45.9669
#: ...and the optimizer's rect sum on the same board, 70.05 until #1206 (the
#: quench's far side is the clusters too). Still a different number from the
#: census, which is what the caption arms below need.
GLASGOW_RECTS = 62.0504


def test_glasgow_caption_is_the_census():
    m = _model()
    census = RP.legality_findings(m)['courtyard_overlap_mm2']
    assert abs(census - GLASGOW_CENSUS) < 0.0005, census
    assert abs(m.metrics['overlap_area'] - GLASGOW_RECTS) < 0.005, m.metrics
    cap = RP.caption(RP.PanelSpec(m, label='x'))
    assert f'courtyard overlap {GLASGOW_CENSUS:.2f}mm2' in cap, cap
    assert f'{GLASGOW_RECTS:.2f}' not in cap, cap
    print(f"  PASS: {cap}")


def test_a_project_waiver_does_not_zero_the_caption():
    with tempfile.TemporaryDirectory() as td:
        for ext in ('.kicad_pcb', '.kicad_pro'):
            shutil.copy(os.path.splitext(GLASGOW)[0] + ext, td)
        pro = os.path.join(td, 'glasgow_revC.kicad_pro')
        with open(pro, encoding='utf-8') as fh:
            doc = json.load(fh)
        rs = doc.setdefault('board', {}).setdefault(
            'design_settings', {}).setdefault('rule_severities', {})
        rs['courtyards_overlap'] = 'ignore'
        with open(pro, 'w', encoding='utf-8') as fh:
            json.dump(doc, fh, indent=2)
        m = _model(os.path.join(td, 'glasgow_revC.kicad_pcb'))
        census = RP.legality_findings(m)['courtyard_overlap_mm2']
        cap = RP.caption(RP.PanelSpec(m, label='x'))
    assert m.metrics['overlap_area'] < 1e-9, m.metrics['overlap_area']
    assert abs(census - GLASGOW_CENSUS) < 0.0005, census
    assert f'courtyard overlap {GLASGOW_CENSUS:.2f}mm2' in cap, cap
    print(f"  PASS: under courtyards_overlap=ignore the optimizer's metric is "
          f"{m.metrics['overlap_area']:.2f} and the caption still reads the "
          f"census ({census})")


def test_no_census_prints_na():
    from placement import legality
    real = legality.grade_body_overlap

    def boom(*a, **k):
        raise RuntimeError('census unavailable')
    legality.grade_body_overlap = boom
    try:
        m = _model()
        cap = RP.caption(RP.PanelSpec(m, label='x'))
    finally:
        legality.grade_body_overlap = real
    assert RP.legality_findings(m)['courtyard_census_error'], 'no error kept'
    assert 'courtyard overlap n/a (census not built)' in cap, cap
    m2 = _model()
    m2.pcb = None
    cap2 = RP.caption(RP.PanelSpec(m2, label='x'))
    assert 'courtyard overlap n/a (census not built)' in cap2, cap2
    assert '0.00mm2' not in cap and '0.00mm2' not in cap2, (cap, cap2)
    print("  PASS: a raised census and a pcb-less model both print n/a")


def test_the_pair_diff_names_both_numbers():
    # Two DIFFERENT boards, so a census read from the wrong side shows: the
    # pair diff's numbers are per model, whatever the pair is.
    esp = _model(os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb'))
    esp_cen = RP.legality_findings(esp)['courtyard_overlap_mm2']
    assert abs(esp_cen - GLASGOW_CENSUS) > 0.01, esp_cen
    text, J = RP.describe_pair(_model(), esp, None)
    assert J['courtyard_overlap_mm2'] == {'before': GLASGOW_CENSUS,
                                          'after': round(esp_cen, 4)}, J
    assert abs(J['overlap_area']['before'] - GLASGOW_RECTS) < 0.005, J
    assert (f'courtyard overlap mm2 (census): {GLASGOW_CENSUS:.2f} -> '
            f'{esp_cen:.2f}' in text), text
    assert (f'overlap mm2 (optimizer rects): {GLASGOW_RECTS:.2f} ->'
            in text), text
    # a side whose census raised has no census number to diff
    from placement import legality
    before = _model()
    RP.legality_findings(before)
    real = legality.grade_body_overlap

    def boom(*a, **k):
        raise RuntimeError('census unavailable')
    legality.grade_body_overlap = boom
    try:
        after = _model()
        text2, J2 = RP.describe_pair(before, after, None)
    finally:
        legality.grade_body_overlap = real
    assert 'courtyard_overlap_mm2' not in J2, J2
    assert '(census)' not in text2, text2
    print(f"  PASS: the pair diff carries each side's census ({GLASGOW_CENSUS:.2f} -> "
          f"{esp_cen:.2f}) and the optimizer's number, each labelled; a "
          f"side with no census diffs none")


TESTS = [
    test_glasgow_caption_is_the_census,
    test_a_project_waiver_does_not_zero_the_caption,
    test_no_census_prints_na,
    test_the_pair_diff_names_both_numbers,
]


if __name__ == '__main__':
    want = sys.argv[1:]
    for t in TESTS:
        if want and not any(w in t.__name__ for w in want):
            continue
        print(f"--- {t.__name__}")
        t()
    print("ALL PASS")
