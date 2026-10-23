#!/usr/bin/env python3
"""Independent grade of a plan-pcb-placement board -- never the agent's own claim.

    python3 -X utf8 .claude/skills/plan-pcb-placement/scripts/grade.py BOARD \
        --baseline INPUT.kicad_pcb [--intent INTENT.json] [--mode full|place] \
        [--label NAME] [--out-dir DIR] [--spec NAME=VALUE ...]

Runs the graders the skill's verifier runs, at the board's own .kicad_pro
floor, and writes `<out-dir>/grade_<label>.json` with the headline numbers
and each tool's exit code. `--spec` passes a spec flag the board declares
(e.g. `net-min-widths=widths.json`) to board_score and check_complete.
Grade every arm of a comparison with this same script at the same commit,
or the numbers do not compare.

Exit 0 when the mode's DONE conditions all hold, 4 when they do not, 2 on
usage, 1 when the board or the baseline is missing or empty, or the intent
is missing.
"""
import argparse
import hashlib
import json
import os
import subprocess
import sys

KRT_TOOL = {'scope': ['placement', 'routing', 'combined'], 'kind': 'instrument'}

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__))))))


def _run(argv):
    p = subprocess.run([sys.executable, '-X', 'utf8'] + [str(a) for a in argv],
                       cwd=ROOT, capture_output=True, text=True,
                       encoding='utf-8', errors='replace')
    return p.returncode, (p.stdout or '') + (p.stderr or '')


def _load(path):
    try:
        with open(path, encoding='utf-8') as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return None


def grade(board, baseline, intent=None, mode='full', label=None, out_dir=None,
          spec=()):
    """`spec`: [(flag, value), ...] the board declares (e.g. --net-min-widths
    FILE), passed to board_score and check_complete so DONE is measured
    against the spec, not against the fab floor."""
    # Absolute BEFORE anything runs: every checker runs with cwd=ROOT, so a
    # path relative to the caller's directory would name a missing file.
    board, baseline = os.path.abspath(board), os.path.abspath(baseline)
    intent = os.path.abspath(intent) if intent else None
    if intent and not os.path.isfile(intent):
        raise SystemExit(f'refuse: intent {intent} does not exist')
    for p, what in ((board, 'board'), (baseline, 'baseline')):
        if not os.path.isfile(p) or os.path.getsize(p) == 0:
            raise SystemExit(f'refuse: {what} {p} is not a real non-empty file')
    label = label or os.path.splitext(os.path.basename(board))[0]
    out_dir = os.path.abspath(out_dir or os.path.dirname(board))
    tmp = os.path.join(out_dir, f'_grade_{label}')
    os.makedirs(tmp, exist_ok=True)
    with open(board, 'rb') as fh:
        sha = hashlib.sha256(fh.read()).hexdigest()
    out = {'board': board, 'sha256': sha, 'mode': mode, 'baseline': baseline,
           'intent': intent,
           'has_kicad_pro': os.path.isfile(os.path.splitext(board)[0] + '.kicad_pro')}
    iflag = ['--intent', intent] if intent else []
    spec_args = [x for pair in spec for x in pair]
    out['spec'] = [list(p) for p in spec]

    aj = os.path.join(tmp, 'assembly.json')
    # --baseline ARMS the courtyard gate: a pair a moved part entered gates
    # (#1183). Without it check_assembly reported them and said buildable.
    out['check_assembly_rc'], _ = _run(['py_tools/check_assembly.py', board,
                                        *iflag, '--baseline', baseline,
                                        '--json', aj])
    out['assembly_verdict'] = (_load(aj) or {}).get('verdict')
    out['buildable'] = bool((_load(aj) or {}).get('buildable'))
    out['assembly_gating_basis'] = (_load(aj) or {}).get(
        'courtyard_gating_basis')

    if intent:
        fj = os.path.join(tmp, 'floorplan.json')
        out['check_floorplan_rc'], _ = _run(['py_tools/check_floorplan.py', board,
                                             '--intent', intent, '--allow-routed',
                                             '--json', fj])
        f = _load(fj)
        if isinstance(f, dict) and isinstance(f.get('violations'), list):
            v = f['violations']
            out['floorplan_errors'] = sum(1 for x in v
                                          if x.get('severity') == 'error')
            out['floorplan_warnings'] = sum(1 for x in v
                                            if x.get('severity') == 'warn')
            out['floorplan_pass'] = f.get('pass')
        else:
            out['floorplan_errors'] = None      # unreadable -> not DONE

    rj = os.path.join(tmp, 'render.json')
    out['render_rc'], _ = _run(['py_tools/render_placement.py', board,
                                '--json-out', rj, '-o',
                                os.path.join(tmp, 'render.png'), '--quiet'])
    off = ((_load(rj) or {}).get('checklist') or {}).get('a_off_outline') or {}
    # #1096: the gating subset (castellated edges exempt), the list
    # check_assembly's verdict reads; an older render has only pad_copper.
    out['off_outline_pad_copper'] = (off.get('pad_copper_gating')
                                     if 'pad_copper_gating' in off
                                     else off.get('pad_copper'))
    out['off_outline_graphic_copper'] = off.get('graphic_copper')
    # a pad in a (keepout (tracks not_allowed)) band cannot be routed (#1031)
    out['keepout_copper'] = off.get('keepout_copper')
    metrics = (_load(rj) or {}).get('metrics') or {}
    for k in ('hpwl', 'crossings'):          # render_placement's own keys
        if k in metrics:
            out[k] = metrics[k]

    done = out['buildable'] and not out['off_outline_pad_copper'] \
        and not out['off_outline_graphic_copper'] \
        and not out['keepout_copper'] \
        and (not intent or out.get('floorplan_errors') == 0)

    if mode == 'full':
        sj = os.path.join(tmp, 'score.json')
        out['board_score_rc'], _ = _run(['py_tools/board_score.py', board,
                                         *iflag, *spec_args, '--baseline', baseline,
                                         '--json', sj, '--quiet'])
        s = _load(sj) or {}
        c = s.get('components') or {}
        out.update(blocking=s.get('blocking'),
                   blocking_by=s.get('blocking_by'),
                   ungraded=s.get('ungraded'),
                   unrouted=(c.get('unrouted') or {}).get('count'),
                   broken=(c.get('broken') or {}).get('count'),
                   drc=(c.get('drc') or {}).get('count'),
                   **(s.get('quality') or {}))
        for key, extra in (('check_complete', ['--baseline', baseline]),
                           ('check_complete_authored',
                            ['--baseline', baseline,
                             '--authored-from', baseline])):
            cj = os.path.join(tmp, f'{key}.json')
            out[key + '_rc'], _ = _run(['check_complete.py', board, *iflag,
                                        *spec_args, *extra, '--json', cj])
            out[key] = (_load(cj) or {}).get('verdict')
            # #1183: whether check_complete's score graded the courtyard
            # gate -- the baseline reaching it is what arms it.
            out[key + '_gating_armed'] = (
                ((((_load(cj) or {}).get('score') or {}).get('components')
                  or {}).get('assembly') or {}).get('courtyard_gating_armed'))
        rc, log = _run(['py_router/check_connected.py', board])
        out['check_connected_rc'] = rc
        rc, log = _run(['py_router/check_drc.py', board, '--baseline', baseline,
                        '--clearance-margin', '0.1'])
        out['check_drc_rc'] = rc
        done = done and out['blocking'] == 0 and out['check_complete'] == 'DONE' \
            and out['check_connected_rc'] == 0 and out['check_drc_rc'] == 0

    out['done'] = bool(done)
    dst = os.path.join(out_dir, f'grade_{label}.json')
    with open(dst, 'w', encoding='utf-8') as fh:
        json.dump(out, fh, indent=2, sort_keys=True)
    return out, dst


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('board')
    ap.add_argument('--baseline', required=True,
                    help='the input board the run started from')
    ap.add_argument('--intent', default=None)
    ap.add_argument('--mode', choices=('full', 'place'), default='full')
    ap.add_argument('--label', default=None)
    ap.add_argument('--out-dir', default=None,
                    help="default: the board's own directory")
    ap.add_argument('--spec', action='append', default=[], metavar='NAME=VALUE',
                    help='a spec flag the board declares, without its leading '
                         'dashes, passed to board_score and check_complete; '
                         'repeatable, e.g. --spec net-min-widths=widths.json')
    a = ap.parse_args(argv)
    spec = []
    for s in a.spec:
        name, sep, value = s.partition('=')
        if not sep or not name.strip():
            ap.error(f'--spec {s!r}: expected NAME=VALUE')
        spec.append(('--' + name.strip().lstrip('-'), value))
    out, dst = grade(a.board, a.baseline, a.intent, a.mode, a.label, a.out_dir,
                     spec)
    keys = ('done', 'blocking', 'unrouted', 'broken', 'drc', 'vias',
            'copper_mm', 'segments', 'check_complete', 'check_complete_authored',
            'assembly_verdict', 'floorplan_errors')
    print(json.dumps({k: out.get(k) for k in keys if k in out}))
    print('wrote', dst)
    return 0 if out['done'] else 4


if __name__ == '__main__':
    sys.exit(main())
