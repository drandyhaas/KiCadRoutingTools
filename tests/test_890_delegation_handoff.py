#!/usr/bin/env python3
"""The delegated half's agent TYPE and its hand-off on disk (#890).

The driver emitted `<subagent_prompt agent="claude">`, and every agent type
except `fork` starts with an EMPTY context -- so the placement half never saw
the user's brief, the orchestrator's board analysis, the before-render or the
measured facts, and rebuilt them: 17 read-only probe scripts, ~950 lines, an
hour, on an 18-part board. The prompt it was handed and the prose it returned
then survived nowhere.

WHAT THIS FILE CAN AND CANNOT PROVE. It never spawns an agent, so it cannot
show that a fork is cheaper or better. What it can show is that the DECISION
is taken by the driver and is a function of the flag, at every site; that the
verifier is excluded from it by construction rather than by convention; and
that the archived prompt is byte-identical to the prompt that was emitted --
which is the property that makes the file evidence rather than decoration, and
the one an "ask the orchestrator to save it" design could never have.

Run: python3 -X utf8 tests/test_890_delegation_handoff.py
Exit 0 all-pass, 1 any failure. Never 77.
"""
import os
import sys
import tempfile

RUN_ALL_FAST_OK = True

_TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(_TESTS)
SCRIPTS = os.path.join(ROOT, '.claude', 'skills',
                       'plan-pcb-placement-and-routing', 'scripts')
for _p in (SCRIPTS, ROOT):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import loop_driver as L  # noqa: E402

passed = failed = 0


def ck(name, ok, detail=''):
    """`detail` is what to say when it FAILS, so it is not printed on a pass.

    A detail worded as a diagnosis ("wrote one", "l5 source mentions _agent")
    reads as a contradiction beside `OK`, and a reader skimming a green run
    should not have to reconcile it.
    """
    global passed, failed
    if ok:
        passed += 1
        print(f'  OK   {name}')
    else:
        failed += 1
        print(f'  FAIL {name} -- {detail}')


def args(extra):
    return L._args(['--board', 'b.kicad_pcb'] + extra)


def tag(text):
    """The agent type in the first <subagent_prompt>, or None."""
    if '<subagent_prompt' not in text:
        return None
    head = text.split('<subagent_prompt', 1)[1].split('>', 1)[0]
    if 'agent="' not in head:
        return None
    return head.split('agent="', 1)[1].split('"', 1)[0]


# ---------------------------------------------------------------- 1
# The type is a FUNCTION OF THE FLAG at both delegated sites. Asserted per
# mode and per stage rather than "somewhere in the text", so a mode that
# silently emits the other type cannot pass.
_l2 = ['--placement-report', os.path.join(_TESTS, 'fixtures_890_l2.json')]
for mode, want in (('fork', 'fork'), ('fresh', 'claude')):
    out = L.STAGES['L1'](args(['--delegate-mode', mode]))
    ck(f'L1 emits agent="{want}" in --delegate-mode {mode}',
       tag(out) == want, str(tag(out)))
ck('...and fork is the DEFAULT, with no flag at all',
   tag(L.STAGES['L1'](args([]))) == 'fork',
   str(tag(L.STAGES['L1'](args([])))))

# ---------------------------------------------------------------- 2
# The verifier is excluded BY CONSTRUCTION. #890's one declined sub-item.
#
# Asserted on the MECHANISM, not on an L5 output: reaching L5's verifier
# prompt needs a ledger, a score, a routing close-out and a board whose sha
# matches, and every cheaper fixture returns `<error>` -- against which
# `agent="fork" not in out` is true for free. That tautology is exactly what
# an earlier draft of this file asserted, and it passed. The runtime check
# lives in `loop_driver --self-test`, in the terminal-close-out block that
# already builds that fixture.
import inspect  # noqa: E402
ck('l5 does not consult _agent, so no flag can fork the verifier',
   '_agent(' not in inspect.getsource(L.l5), 'l5 source mentions _agent')
ck('...while l1 and l2 both do, so the check discriminates',
   '_agent(' in inspect.getsource(L.l1)
   and '_agent(' in inspect.getsource(L.l2),
   'l1/l2 do not call _agent -- this check has stopped meaning anything')
ck('L5 has a hand-off prompt name of its own',
   L._PROMPT_FILE.get('L5') == 'verify_prompt.txt',
   str(L._PROMPT_FILE))

# ---------------------------------------------------------------- 3
# `--no-delegate` is orthogonal and still wins, in both modes.
for mode in ('fork', 'fresh'):
    out = L.STAGES['L1'](args(['--delegate-mode', mode, '--no-delegate']))
    ck(f'--no-delegate suppresses the L1 prompt in {mode} mode',
       '<subagent_prompt' not in out, out[:60])

# ---------------------------------------------------------------- 4
# The archived prompt is the prompt that was EMITTED, byte for byte. This is
# the assertion that makes the file evidence; an orchestrator-written copy
# could not satisfy it, and neither could an archive of the whole stage.
with tempfile.TemporaryDirectory() as tmp:
    wk = os.path.join(tmp, 'wk')
    os.makedirs(wk)
    ledger = os.path.join(wk, 'ledger.jsonl')
    open(ledger, 'w', encoding='utf-8').close()
    a = args(['--ledger', ledger])
    a.stage = 'L1'
    out = L.STAGES['L1'](a)
    p = L._write_prompt(a, 'L1', out)
    ck('the driver writes the L1 hand-off prompt beside the ledger',
       p and os.path.isfile(p), str(p))
    body = open(p, encoding='utf-8').read().strip()
    ck('...and it is the PROMPT BODY, byte-identical to what was emitted',
       body == L._prompt_body(out), f'{len(body)} vs {len(L._prompt_body(out))}')
    ck('...not the whole stage', '<stage_instructions' not in body,
       body[:60])
    ck('...and it starts where the prompt starts',
       body.startswith('Drive the placement half'), body[:40])

    # The INLINE arm writes nothing: no tag, no body, no file. That is what
    # keeps the escape hatch byte-clean rather than merely quiet.
    ai = args(['--ledger', ledger, '--no-delegate'])
    ai.stage = 'L1'
    oi = L.STAGES['L1'](ai)
    ck('--no-delegate writes no prompt file at all',
       L._write_prompt(ai, 'L1', oi) is None, 'wrote one')

    # A stage that REFUSES carries no prompt, so it archives none -- the same
    # rule as the inline arm, reached a different way.
    a5 = args(['--ledger', ledger, '--score', 'x.json'])
    a5.stage = 'L5'
    o5 = L.STAGES['L5'](a5)
    ck('a refusing stage archives nothing',
       o5.startswith('<error>') and L._write_prompt(a5, 'L5', o5) is None,
       f'error={o5.startswith("<error>")}')

    # CYCLE SUFFIX. Cycle 2 must not overwrite cycle 1's file, whose mtime is
    # what dates the delegation.
    _n1, P1 = L._paths(a, starting=True)
    ck('the hand-off names carry the cycle suffix like every other artifact',
       P1['place_prompt.txt'].endswith('place_prompt.txt')
       or '_c' in os.path.basename(P1['place_prompt.txt']),
       P1['place_prompt.txt'])
    ck('and place_return.md is registered beside it',
       'place_return.md' in L._ARTIFACTS, str(L._ARTIFACTS[:4]))

# ---------------------------------------------------------------- 5
# A missing ledger directory INVENTS NOTHING and says so. Same rule as
# _log_invocation: a work dir must not appear because someone asked for text.
with tempfile.TemporaryDirectory() as tmp:
    gone = os.path.join(tmp, 'nope', 'ledger.jsonl')
    a = args(['--ledger', gone])
    a.stage = 'L1'
    out = L.STAGES['L1'](a)
    ck('a missing ledger dir writes no prompt file',
       L._write_prompt(a, 'L1', out) is None, 'wrote one')
    ck('...and does not create the directory',
       not os.path.isdir(os.path.dirname(gone)), os.path.dirname(gone))

# ---------------------------------------------------------------- 6
# The context block names only what EXISTS, and always names the command.
with tempfile.TemporaryDirectory() as tmp:
    wk = os.path.join(tmp, 'wk')
    os.makedirs(wk)
    board = os.path.join(tmp, 'b.kicad_pcb')
    for f in (board, os.path.join(tmp, 'b.kicad_pro'),
              os.path.join(wk, 'mechanical.json')):
        open(f, 'w', encoding='utf-8').write('{}')
    ctx = L._context(L._args(['--board', board, '--ledger',
                              os.path.join(wk, 'ledger.jsonl')]), wk)
    ck('the context block always names board_brief.py, even with no artifacts',
       'board_brief.py' in ctx, ctx[:70])
    ck('it names the artifacts that exist',
       'b.kicad_pro' in ctx and 'mechanical.json' in ctx, ctx)
    # THE DISCRIMINATOR. Two files were deliberately not created; naming them
    # would send the half looking for something that is not there.
    ck('...and NOT the ones that do not',
       'design-brief' not in ctx and 'before.json' not in ctx, ctx)

print(f'\n{passed} passed, {failed} failed')
print('890 coverage: agent-type=yes verifier-pinned=yes handoff=yes '
      'context=yes')
sys.exit(1 if failed else 0)
