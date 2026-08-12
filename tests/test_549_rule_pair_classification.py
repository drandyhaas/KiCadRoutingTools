#!/usr/bin/env python3
"""#549 rule-pair classification: a seg-seg violation that exists ONLY because
a .kicad_dru track rule raised the pair clearance is emitted as its own type
('segment-segment-track-rule', tagged with the binding rule name), and
board_score reports that population as ADVISORY -- outside `blocking`.

Run-6 evidence this pins: 610 floor-governed rule pairs drowned ~17 physical
defects into blocking=627, making the scalar useless as a loop arbiter while
the repo's check_dru floors were the real gate for exactly those pairs.

Fixture: route the e2e's contended nets PLAIN at 0.2 (so copper legally comes
well under 0.45), then attach a CRIT track rule to the ROUTED board. Every
sub-0.45 CRIT-vs-foreign pair is then rule-governed by construction (the
copper passes its base clearance), and there are no physical grazes.
"""
import json
import os
import re
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, "py_router"))  # #522 layout

BOARD = os.path.join(REPO, "kicad_files", "splitflap_driver.kicad_pcb")
SCORE = os.path.join(REPO, '.claude', 'skills', 'plan-pcb-routing',
                     'scripts', 'board_score.py')
RULE = 0.45
DRU = ('(version 1)\n(rule "PCBX: crit space" (condition "A.Type==\'track\' && '
       'B.Type==\'track\' && A.NetClass==\'CRIT\' && B.NetClass!=\'CRIT\'") '
       f'(constraint clearance (min {RULE}mm)))\n')
PRO = {"net_settings": {
    "classes": [{"name": "Default", "clearance": 0.2},
                {"name": "CRIT", "clearance": 0.2}],
    "netclass_patterns": [{"pattern": "/LED_*", "netclass": "CRIT"}]}}
NETS = ["/LED_*", "/LOOPBACK_*", "/MOTOR_C_*", "Net-(D?-Pad2)"]

passed = failed = 0


def check(name, ok, detail=""):
    global passed, failed
    passed += bool(ok)
    failed += not ok
    print(f"  {'OK  ' if ok else 'FAIL'} {name}{(' -- ' + detail) if detail else ''}")


if not os.path.isfile(BOARD):
    print("SKIP: fixture missing")
    sys.exit(0)

with tempfile.TemporaryDirectory() as d:
    src = os.path.join(d, "in.kicad_pcb")
    from copy_board import copy_board
    copy_board(BOARD, src)
    open(os.path.splitext(src)[0] + ".kicad_pro", "w").write(json.dumps(PRO))
    out = os.path.join(d, "routed.kicad_pcb")
    r = subprocess.run(
        [sys.executable, os.path.join(REPO, "py_router", "route.py"), src, out,
         "--nets", *NETS, "--layers", "F.Cu", "B.Cu",
         "--clearance", "0.2", "--track-width", "0.2"],
        capture_output=True, text=True, timeout=1200)
    check("plain route completed", r.returncode == 0, r.stdout[-500:])

    # Attach the rule to the ROUTED board: every sub-0.45 pair is now
    # rule-governed by construction.
    open(os.path.splitext(out)[0] + ".kicad_pro", "w").write(json.dumps(PRO))
    open(os.path.splitext(out)[0] + ".kicad_dru", "w").write(DRU)

    dr = subprocess.run(
        [sys.executable, os.path.join(REPO, "py_router", "check_drc.py"), out,
         "--no-size-checks", "--max-print", "0"],
        capture_output=True, text=True, timeout=600)
    # `[^\n]*` between the count and the colon: check_drc's per-type header
    # carries an OPTIONAL contact suffix -- `SEGMENT-SEGMENT violations (12)
    # -- 3 in CONTACT:` (check_drc.py `_ct`). Pinning `):` matched nothing on
    # any board with a contact-grade violation, and the two failures pointed
    # OPPOSITE ways: n_rule fell to 0 and failed the check below, while n_phys
    # fell to 0 and PASSED it -- silently masking exactly the physical grazes
    # this test exists to catch.
    m_rule = re.search(r'^SEGMENT-SEGMENT-TRACK-RULE violations \((\d+)\)[^\n]*:',
                       dr.stdout, re.M)
    m_phys = re.search(r'^SEGMENT-SEGMENT violations \((\d+)\)[^\n]*:',
                       dr.stdout, re.M)
    n_rule = int(m_rule.group(1)) if m_rule else 0
    n_phys = int(m_phys.group(1)) if m_phys else 0
    check("rule-governed pairs classified as their own type", n_rule > 0,
          dr.stdout[-600:])
    check("no physical seg-seg grazes on the 0.2-clean copper", n_phys == 0,
          f"physical={n_phys}")
    check("record carries the binding rule name",
          "Track rule: 'PCBX: crit space'" in dr.stdout)

    js = os.path.join(d, 'score.json')
    sc = subprocess.run(
        [sys.executable, '-X', 'utf8', SCORE, out, '--json', js],
        capture_output=True, text=True, cwd=REPO, timeout=900)
    data = json.load(open(js, encoding='utf-8'))
    adv = data.get('advisory') or {}
    comp = (data.get('components') or {}).get('drc_rule_pairs') or {}
    drc_comp = (data.get('components') or {}).get('drc') or {}
    check("board_score advisory carries the rule pairs",
          adv.get('drc_rule_pairs') == n_rule, f"advisory={adv}")
    check("rule pairs are NOT in blocking_by",
          'drc_rule_pairs' not in (data.get('blocking_by') or {}),
          str(data.get('blocking_by')))
    check("drc component counts only physical",
          drc_comp.get('count') == 0 or
          'segment-segment-track-rule' not in (drc_comp.get('by_type') or {}),
          str(drc_comp))
    _bb = data.get('blocking_by') or {}
    _expected = sum(c for c in _bb.values() if c)
    check("blocking == sum(blocking_by), rule pairs not added",
          data.get('blocking') == _expected,
          f"blocking={data.get('blocking')} sum(blocking_by)={_expected} "
          f"rule_pairs={n_rule}")
    check("ADVISORY line printed", 'ADVISORY' in sc.stdout, sc.stdout[-300:])
    check("component payload present", comp.get('ran') is True, str(comp))

print(f"\n{passed}/{passed + failed} checks passed")
sys.exit(1 if failed else 0)
