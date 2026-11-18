"""#1936: a net is routed at its tuning profile's width on each layer.

KiCad 10's tuning profile stores, per signal layer, the track width that meets
its target impedance there (`layer_entries[].width`, nanometres). The reader
took the delays (#1846) and ignored the widths, so a net of a class that names
a profile was drawn at the class's one `track_width` on every layer.

The board is #1846's fixture (nets P and Q of class TIMED, profile PROF6, class
track width 0.1 mm) with Q's track removed and the profile's widths edited.

  1. `profile_layer_widths` gives each profiled net its width per layer, and
     nothing for a layer whose entry stores no width;
  2. a floor pins a narrower width up and says which;
  3. route.py with no --track-width draws Q on F.Cu at the profile's F.Cu
     width and says so;
  4. an explicit --track-width wins;
  5. a class that names no profile is drawn at the class width, as before.

A layer's width is, in order: a stored impedance declaration's, a `.kicad_dru`
rule's, the profile's, the class's. Each pair has a row:
  6. --impedance outranks the profile;
  7. a stored impedance declaration outranks it;
  8. a `.kicad_dru` rule that gives one width for every layer outranks it,
     and a rule scoped to another layer leaves the profile its own layers;
  9. the profile's width is clamped into Board Setup's minimum and a rule's
     maximum, and raised to the standard fab floor, each said;
 10. where the board's rules cannot be read the width is held at the fab
     tier's deepest floor and the descent below the standard one is
     disclosed as a class width's is;
 11. the `Tuning-profile widths` line counts the nets the step routes.
"""
import contextlib
import io
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, 'py_router'))

import run_utils  # noqa: E402
import test_1846_time_domain_rules as t1846  # noqa: E402

PY = [sys.executable]
FAILS = []
F_W, B_W = 0.2, 0.13          # the widths the edited profile gives F.Cu and B.Cu


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def _widths(proj):
    prof = t1846._profile(proj)
    prof['layer_entries'][0]['width'] = int(F_W * 1e6)
    prof['layer_entries'].append(dict(t1846.BCU, width=int(B_W * 1e6)))
    prof['layer_entries'].append(dict(t1846.BCU, signal_layer='In1.Cu', width=0))


def _no_profile(proj):
    _widths(proj)
    t1846._no_profile(proj)


def stage(td, name, edit):
    """The fixture with Q's track removed and the project edited."""
    board = t1846.stage(td, name, t1846.FLOOR, False, edit)
    with open(board, encoding='utf-8') as fh:
        text = fh.read()
    text, n = re.subn(r'\t\(segment\s*\(start 3 6\).*?\n\t\)\n', '', text, flags=re.S)
    assert n == 1, "BROKEN TEST: the fixture has Q's one track"
    with open(board, 'w', encoding='utf-8') as fh:
        fh.write(text)
    return run_utils.evidence(board)


def q_widths(board):
    """The widths of Q's routed segments, by layer."""
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        pcb = parse_kicad_pcb(board)
    nid = next(i for i, n in pcb.nets.items() if n.name == 'Q')
    out = {}
    for s in pcb.segments:
        if s.net_id == nid:
            out.setdefault(s.layer, set()).add(round(s.width, 4))
    return out


def route(td, board, tag, extra=(), nets=('Q',)):
    out = os.path.join(td, tag + '.kicad_pcb')
    r = subprocess.run(PY + [os.path.join('py_router', 'route.py'), board, out,
                             '--nets', *nets, '--layers', 'F.Cu'] + list(extra),
                       capture_output=True, text=True, cwd=REPO)
    line = next((ln for ln in r.stdout.splitlines()
                 if ln.startswith('JSON_SUMMARY_MIN:')), None)
    doc = json.loads(line[len('JSON_SUMMARY_MIN:'):]) if line else {}
    assert doc.get('routed') == 1, (f"BROKEN TEST: the net did not route ({tag})\n"
                                    f"{(r.stdout + r.stderr)[-800:]}")
    return run_utils.evidence(out), r.stdout


def reader(td):
    print('1-2. the reader')
    from kicad_parser import parse_kicad_pcb
    from tuning_profiles import profile_layer_widths, read_tuning_profiles
    board = stage(td, 'rd', _widths)
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        pcb = parse_kicad_pcb(board)
    ids = {n.name: i for i, n in pcb.nets.items() if i}
    prof = read_tuning_profiles(board)[0]['PROF6']
    ok(prof.layer_width_mm == {'F.Cu': F_W, 'B.Cu': B_W} and not prof.differential,
       f"the profile carries its widths, and none for the layer that stores 0: "
       f"{prof.layer_width_mm}")
    got, used, raised = profile_layer_widths(pcb, board)
    ok(got.get(ids['Q']) == {'F.Cu': F_W, 'B.Cu': B_W}
       and got.get(ids['P']) == {'F.Cu': F_W, 'B.Cu': B_W}
       and used == {'PROF6': len(got)} and not raised,
       f"each net of the class gets them: {got.get(ids['Q'])}, {used}")
    got, _used, raised = profile_layer_widths(pcb, board, floor=0.15)
    ok(got[ids['Q']] == {'F.Cu': F_W, 'B.Cu': 0.15}
       and raised == [('PROF6', 'B.Cu', B_W, 0.15)],
       f"a floor pins the narrower width up and names it: {got[ids['Q']]}, {raised}")
    got, _u, _r = profile_layer_widths(pcb, board, net_ids=[ids['Q']])
    ok(list(got) == [ids['Q']], 'and it answers for the nets asked')


def routed(td):
    print('3-5. routed')
    board = stage(td, 'rt', _widths)
    out, log = route(td, board, 'rt_o')
    ok(q_widths(out) == {'F.Cu': {F_W}},
       f"with no --track-width Q is drawn at the profile's F.Cu width: {q_widths(out)}")
    ok('Tuning-profile widths:' in log and 'PROF6' in log,
       'and the run says which profile gave the widths')
    out, log = route(td, board, 'rt_tw', ['--track-width', '0.15'])
    ok(q_widths(out) == {'F.Cu': {0.15}} and 'Tuning-profile widths:' not in log,
       f"an explicit --track-width wins: {q_widths(out)}")
    board = stage(td, 'np', _no_profile)
    out, log = route(td, board, 'np_o')
    ok(q_widths(out) == {'F.Cu': {0.1}} and 'Tuning-profile widths:' not in log,
       f"a class that names no profile is drawn at the class width: {q_widths(out)}")


RULE = ('(version 1)\n(rule "W"\n  %s(constraint track_width (%s))\n'
        '  (condition "A.hasNetclass(\'TIMED\')"))\n')


def _edit(*edits):
    def edit(proj):
        _widths(proj)
        for e in edits:
            e(proj)
    return edit


def _f_width(mm):
    def edit(proj):
        t1846._profile(proj)['layer_entries'][0]['width'] = int(round(mm * 1e6))
    return edit


def _board_min(mm):
    def edit(proj):
        proj['board']['design_settings']['rules']['min_track_width'] = mm
    return edit


def _stored(proj):
    proj['kicad_routing_tools'] = {'net_impedance': {
        'Q': {'ohms': 60.0, 'differential': False}}}


def with_dru(board, text):
    with open(os.path.splitext(board)[0] + '.kicad_dru', 'w', encoding='utf-8') as fh:
        fh.write(text)
    return board


def one_width(out):
    got = q_widths(out).get('F.Cu', set())
    return next(iter(got)) if len(got) == 1 else None


def precedence(td):
    print('6-8. what outranks the profile')
    board = stage(td, 'pi', _widths)
    out, log = route(td, board, 'pi_o', ['--impedance', '60'])
    w_imp = one_width(out)
    ok(w_imp is not None and abs(w_imp - F_W) > 0.01
       and 'Tuning-profile widths:' not in log,
       f"--impedance outranks the profile: Q at {w_imp}, not {F_W}")
    board = stage(td, 'ps', _edit(_stored))
    out, log = route(td, board, 'ps_o')
    ok(one_width(out) == w_imp and 'Reapplying stored 60 ohm' in log
       and 'Tuning-profile widths:' not in log,
       f"a stored impedance declaration outranks it: Q at {one_width(out)}, "
       f"the width --impedance 60 gave")
    board = with_dru(stage(td, 'pr', _widths), RULE % ('', 'opt 0.3mm'))
    out, log = route(td, board, 'pr_o')
    ok(one_width(out) == 0.3 and 'Tuning-profile widths:' not in log,
       f"a .kicad_dru rule with one width for every layer outranks it: "
       f"{q_widths(out)}")
    board = with_dru(stage(td, 'pl', _widths),
                     RULE % ('(layer "B.Cu")\n  ', 'opt 0.3mm'))
    out, log = route(td, board, 'pl_o')
    ok(one_width(out) == F_W and "B.Cu: .kicad_dru rule \"W\" sets the width" in log
       and "F.Cu: .kicad_dru rule" not in log,
       f"a rule scoped to B.Cu leaves the profile F.Cu and says which layer "
       f"it sets: {q_widths(out)}")
    from kicad_parser import parse_kicad_pcb
    from design_rules import DesignRules
    from tuning_profiles import profile_layer_widths
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        pcb = parse_kicad_pcb(board)
        rules = DesignRules.from_project(pcb, board)
    q = next(i for i, n in pcb.nets.items() if n.name == 'Q')
    rep = {}
    got, used, _r = profile_layer_widths(pcb, board, rules=rules, report=rep)
    ok(got[q] == {'F.Cu': F_W, 'B.Cu': 0.3} and used == {'PROF6': 2}
       and rep.get('ruled') == {('PROF6', 'B.Cu'): 'W'},
       f"and the reader gives B.Cu the rule's width: {got[q]}, {rep.get('ruled')}")


def clamped(td):
    print('9. the clamp')
    board = stage(td, 'cm', _edit(_f_width(0.11), _board_min(0.127)))
    out, log = route(td, board, 'cm_o')
    ok(one_width(out) == 0.127 and "F.Cu: 0.11 mm is below" in log
       and 'drawn at 0.127 mm' in log,
       f"a profile width under Board Setup's minimum track is drawn at the "
       f"minimum: {q_widths(out)}")
    board = with_dru(stage(td, 'cx', _widths), RULE % ('', 'max 0.15mm'))
    out, log = route(td, board, 'cx_o')
    ok(one_width(out) == 0.15 and "F.Cu: 0.2 mm is above" in log,
       f"and one over a rule's maximum at the maximum: {q_widths(out)}")
    board = stage(td, 'cf', _edit(_f_width(0.08), _board_min(0.05)))
    out, log = route(td, board, 'cf_o')
    ok(one_width(out) == 0.0889 and "F.Cu: 0.08 mm is below" in log,
       f"one between the advanced and the standard fab floor is raised to "
       f"the standard floor, as a class width is: {q_widths(out)}")


def disclosed(td):
    print('10. below the standard floor with the rules unread')
    import route as route_mod
    import design_rules
    import fab_tiers
    board = stage(td, 'ds', _edit(_f_width(0.08), _board_min(0.05)))
    out = os.path.join(td, 'ds_o.kicad_pcb')
    real = design_rules.DesignRules.from_project
    state = {'failed': 0}

    def once(*a, **k):
        # batch_route's own reader is its width resolver
        if (not state['failed'] and os.path.basename(
                sys._getframe(1).f_code.co_filename) == 'route.py'):
            state['failed'] = 1
            raise ValueError('rules unreadable (test)')
        return real(*a, **k)
    design_rules.DesignRules.from_project = once
    buf = io.StringIO()
    try:
        with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(buf):
            route_mod.batch_route(board, out, ['Q'], layers=['F.Cu'],
                                  track_width=0.1, track_width_from_class=True)
            summary = fab_tiers.escalation_summary()
    finally:
        design_rules.DesignRules.from_project = real
    log = buf.getvalue()
    assert state['failed'] and 'design-rule widths unavailable' in log, \
        "BROKEN TEST: the width resolver did not take the fallback"
    ctx = [c for c in summary['fab_tier_contexts'] if 'PROF6' in c and 'F.Cu' in c]
    ok(one_width(run_utils.evidence(out)) == 0.08 and len(ctx) == 1
       and "WARNING: tuning profile 'PROF6' width 0.08 on F.Cu" in log,
       f"the width is drawn and the descent is in design_rules: "
       f"{q_widths(out)}, {ctx}")


def counted(td):
    print('11. the count')
    board = stage(td, 'ct', _widths)
    _out, log = route(td, board, 'ct_o')
    line = next((ln for ln in log.splitlines()
                 if ln.startswith('Tuning-profile widths:')), '')
    ok('Tuning-profile widths: 1 net(s)' in line and 'PROF6: 1' in line,
       f"two nets take the profile and the step routes one: {line!r}")


def main():
    td = tempfile.mkdtemp(prefix='krt1936_')
    try:
        reader(td)
        routed(td)
        precedence(td)
        clamped(td)
        disclosed(td)
        counted(td)
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if FAILS:
        print(f'FAIL: {len(FAILS)}: ' + '; '.join(FAILS))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
