#!/usr/bin/env python3
"""The placement skill's "Which tool, when" section, gated BOTH WAYS against
the #937 registry.

A hand-written tool table is a second catalogue, and this repo's recorded
failure mode is that a second catalogue drifts and that the names missing from
it are where the bug hides. So the table is allowed to be a DECISION -- which
tool for which situation, which this file does not try to judge -- and its
POPULATION is checked against `krt_registry`:

  * every tool the section names is a real tool at the placement door (no
    phantom, no tool that moved doors);
  * every ACTOR at the placement door is named in the section, so a new one
    cannot arrive with nothing telling a reader when to reach for it;
  * an actor is named in the actors table and an instrument in the
    instruments table, so the section's own actor/instrument split -- the one
    that decides whether a reader may run it freely -- agrees with the
    declaration the tool itself carries.

Reading is AST-only (`krt_registry.declaration`), never the runnable probe:
the probe spends ~86s of subprocess time, and nothing here needs to know
whether a tool runs, only what it declares.

Run:
    python3 tests/test_956_skill_tool_index.py
"""

import os
import re
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT)

import krt_registry as reg  # noqa: E402

RUN_ALL_FAST_OK = True

SKILL = os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-placement',
                     'SKILL.md')
HEADING = '### Which tool, when'
#: The two sub-tables, by the bold line that introduces each.
ACTOR_LEAD = '**Actors'
INSTRUMENT_LEAD = '**Instruments'

#: Kinds a reader may run to ASK rather than to CHANGE. `conditional` is in
#: the instrument half deliberately: check_drc and plane_score answer a
#: question and write no board.
ASK_KINDS = ('instrument', 'conditional')


def _section():
    """(actor_half, instrument_half) of the section's text."""
    text = open(SKILL, encoding='utf-8').read()
    i = text.find(HEADING)
    assert i >= 0, f'{HEADING!r} is not in {SKILL}'
    # To the next heading of the same or higher level.
    m = re.search(r'(?m)^#{1,3} ', text[i + len(HEADING):])
    body = text[i:i + len(HEADING) + (m.start() if m else len(text))]
    a = body.find(ACTOR_LEAD)
    b = body.find(INSTRUMENT_LEAD)
    assert a >= 0 and b > a, 'the section lost one of its two halves'
    return body[a:b], body[b:]


def _tokens(chunk):
    """Every word-ish token inside a backticked span, basename'd.

    The section writes a tool either bare (`place_seed`) or with its
    extension, and sometimes inside a path or beside a flag, so the span is
    split rather than matched whole.
    """
    out = set()
    for raw in re.findall(r'`([^`]+)`', chunk):
        for tok in re.findall(r'[\w./\\-]+', raw):
            out.add(os.path.basename(tok.replace('\\', '/')))
    return out


def _named(chunk, known):
    """The tools of `known` the chunk mentions, bare or with `.py`.

    RESOLVED against the registry, never invented from the text. The first
    draft appended `.py` to any lowercase word of four letters or more, which
    turned the placeholders `<the nets the failure named>` and `<the
    comparator>` into four tools that do not exist and failed its own phantom
    arm.
    """
    toks = _tokens(chunk)
    return {n for n in known if n in toks or n[:-3] in toks}


def _spelled_py(chunk):
    """Tokens the chunk spells WITH `.py` -- the phantom arm's population,
    which must not depend on the registry it is being checked against."""
    return {t for t in _tokens(chunk) if re.fullmatch(r'[a-z_0-9]+\.py', t)}


def main():
    fails = []

    def check(name, cond, detail=''):
        print(f"  {'PASS' if cond else 'FAIL'}: {name}"
              + (f"   [{detail}]" if detail and not cond else ''))
        if not cond:
            fails.append(name)

    # --- the registry, by AST only --------------------------------------
    door = {}
    every = set()
    for rel in reg.tracked_python():
        decl = reg.declaration(reg.ROOT, rel)
        if decl:
            every.add(os.path.basename(rel))
        if decl and 'placement' in (decl.get('scope') or ()):
            # basename: place_fanout_clearance.py is declared twice (the
            # py_placer tool and its py_router shim) and a reader reaches for
            # one name. The KIND is the same on both.
            door.setdefault(os.path.basename(rel), decl.get('kind'))
    check('the registry has a placement door to check against',
          len(door) >= 20, f'{len(door)} tools')
    actors = {n for n, k in door.items() if k == 'actor'}
    asks = {n for n, k in door.items() if k in ASK_KINDS}
    check('...with actors and instruments in it',
          len(actors) >= 8 and len(asks) >= 10,
          f'{len(actors)} actors, {len(asks)} instruments')

    actor_half, instrument_half = _section()
    named_a = _named(actor_half, door)
    named_i = _named(instrument_half, door)
    check('the actors table names tools', len(named_a) >= 8, f'{sorted(named_a)}')
    check('the instruments table names tools', len(named_i) >= 10,
          f'{sorted(named_i)}')

    # --- 1: no phantom ---------------------------------------------------
    # A name the section spells like a tool must BE one somewhere in the
    # registry. Checked against `every` rather than against a directory
    # listing: the first draft asked whether a file of that name existed in
    # py_placer/, so an invented `place_nowhere.py` passed (measured -- the
    # mutation SURVIVED), because a name that exists nowhere at all is
    # exactly the one a directory check cannot see.
    #
    # `every` and not `door`, deliberately: a row may legitimately point at a
    # tool from another door, and arm 3 is what holds the door's own split.
    spelled = _spelled_py(actor_half) | _spelled_py(instrument_half)
    ghosts = sorted(spelled - every)
    check('every tool-shaped name in the section is a tool in the registry',
          not ghosts, f'{ghosts}')

    # --- 2: no un-routed actor (the direction that matters) --------------
    missing = sorted(actors - named_a - named_i)
    check('every ACTOR at the placement door is named in the section',
          not missing,
          f'{missing} -- add a row saying when to reach for it, or move it '
          f'off this door in its KRT_TOOL declaration')

    # --- 3: the halves agree with the declarations -----------------------
    mislabelled_a = sorted(n for n in named_a if door[n] in ASK_KINDS)
    mislabelled_i = sorted(n for n in named_i if door[n] == 'actor')
    check('nothing that only MEASURES is listed as an actor',
          not mislabelled_a, f'{mislabelled_a}')
    check('nothing that CHANGES THE BOARD is listed as an instrument',
          not mislabelled_i, f'{mislabelled_i}')

    # --- 4: anti-vacuity on the extractor --------------------------------
    # A parser that stopped finding names would pass arms 1 and 3 by checking
    # nothing, and arm 2 would then fail loudly -- but only if `actors` is
    # non-empty, which arm 0 pins. This states the remaining assumption: the
    # section really does mention a tool this file can resolve.
    check('the extractor resolves a known tool by name',
          'place_seed.py' in named_a and 'check_assembly.py' in named_i,
          f'{sorted(named_a)[:4]} / {sorted(named_i)[:4]}')

    print(f"\n{'FAILED: ' + ', '.join(fails) if fails else 'ALL PASS'}")
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
