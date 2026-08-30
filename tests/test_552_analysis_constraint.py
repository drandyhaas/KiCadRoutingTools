#!/usr/bin/env python3
"""Every GUI analysis prompt states the constraint, and it binds the child (#552).

The GUI's eight "Ask AI" analysis runs each told the agent *"analysis only, do
not modify any files"* -- eight hand-copied sentences across four modules, none
of which reached a subagent. #552 adds `Task` to `CLAUDE_ALLOWED_TOOLS` so those
runs can dispatch an independent verifier at all, which makes the parent-only
contract a real gap rather than a latent one: a child inherits the tool floor
and nothing else.

What ships is prompt text, stated ONCE in `ai_backend.ANALYSIS_CONSTRAINT`, and
this gate keeps the eight sites using it. That is a CONVENTION, not an
enforcement, and an earlier version of this docstring justified it by claiming
"there is no harness lever for this" -- reasoning from what `build_cmd`
currently emits to what the CLI can do. False: `claude --help` on 2.1.251
offers `--agents <json>` (define a subagent and its tools outright),
`--append-system-prompt`, `--permission-mode` and `--settings`. `--agents` is
the enforceable form of #552 item 2. It is owed, not done, and this gate does
not pretend otherwise.

AST, not grep, on purpose: the constraint text is quoted in comments and in
this file's own docstring, and a source-grep cannot tell a prompt from a
sentence about a prompt. This resolves and EVALUATES the actual argument
expression at each call site -- checking the name `ANALYSIS_CONSTRAINT` appears
is not enough, and five reproduced evasions are why:

  * a ninth prompt in a module the scan's file list did not name;
  * a call passing `instructions=` by keyword instead of positionally;
  * `ANALYSIS_CONSTRAINT.replace("do not modify any files", "you may edit ...")`
    -- the name is present, the meaning inverted;
  * a local `ANALYSIS_CONSTRAINT = "read-only work, please."` shadowing the
    import;
  * a constraint whose text says the opposite while still CONTAINING every
    substring the content checks looked for, including the check that prints
    "it still says the thing it always said".

So: modules are DISCOVERED, keyword arguments are resolved, the value is
evaluated and compared to the real constant, and the constant's own text is
pinned exactly rather than by substring.

    python3 -X utf8 tests/test_552_analysis_constraint.py
"""
import ast
import io
import os
import sys

RUN_ALL_TIMEOUT = 120
RUN_ALL_FAST_OK = True

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PLUGIN = os.path.join(ROOT, 'kicad_routing_plugin')

#: The two ways a GUI tab starts a headless analysis run, and which POSITIONAL
#: index carries the instructions the agent is bound by. The keyword spelling
#: is resolved too -- passing `instructions=` was a reproduced evasion.
#:   AITab._start_run(skill, skill_args, instructions, kind, intro)
#:   run_skill_dialog(parent, title, skill, skill_args, instructions, intro)
ENTRIES = {'_start_run': 2, 'run_skill_dialog': 4}
INSTRUCTIONS_KW = 'instructions'

#: DISCOVERED, not listed. A hardcoded four-file tuple let a ninth prompt in a
#: fifth module pass unseen; the plugin is small enough to scan whole.
def modules():
    return sorted(f for f in os.listdir(PLUGIN)
                  if f.endswith('.py') and f != 'ai_backend.py')

#: Eight today: 3 on the AI tab, 2 on the Basic tab, 2 on Planes, 1 on Diff
#: pairs. A floor, not an equality -- a ninth analysis button is welcome, a
#: scanner that has stopped finding any is not.
MIN_SITES = 8

#: The sentence the eight sites used to carry, verbatim. If it reappears
#: anywhere in the plugin outside ai_backend.py, somebody has hand-written the
#: contract again instead of importing it.
#:
#: This one IS a source-grep, and deliberately so -- it is a "has this string
#: come back" change detector, not a claim about prompts. It therefore fires on
#: a comment quoting the old sentence, which is a false positive it accepts in
#: exchange for catching a copy-paste. The prompt-level checking is done by
#: evaluation, above.
OLD_SENTENCE = 'analysis only, do not modify any files'

#: The reviewed text of ai_backend.ANALYSIS_CONSTRAINT, spelled out here so a
#: change to it is a change to this file too. See t_the_constant_binds_the_child
#: for why this is an equality and not a bag of substrings.
EXPECTED_CONSTRAINT = (
    "analysis and planning only: do not execute any routing commands and do "
    "not modify any files. If you dispatch a subagent, copy this sentence into "
    "its prompt verbatim and give it no tool this run does not have; have it "
    "answer with a line beginning VERDICT= (never RESULT=, which this GUI "
    "reads as the run's own result line)."
)

FAILURES = []


def check(cond, what, detail=''):
    if cond:
        print(f'  ok   {what}')
    else:
        print(f'  FAIL {what}{detail}')
        FAILURES.append(what)


def _constraint():
    sys.path.insert(0, ROOT)
    import types
    sys.modules.setdefault('wx', types.ModuleType('wx'))
    from kicad_routing_plugin import ai_backend
    return ai_backend.ANALYSIS_CONSTRAINT


def _carries_constraint(node, constant):
    """Does this argument expression EVALUATE to text containing the constant?

    Evaluated, not pattern-matched. `ANALYSIS_CONSTRAINT.replace(...)` and a
    locally-rebound `ANALYSIS_CONSTRAINT = "..."` both mention the name and
    both were reproduced as evasions; only the value settles it.

    Unresolvable pieces (f-string variables, other module constants) become a
    placeholder, which cannot manufacture a pass: the constant is 300+
    characters and no placeholder contains it.
    """
    class _Sub(ast.NodeTransformer):
        def visit_Name(self, n):
            if n.id == 'ANALYSIS_CONSTRAINT':
                return ast.copy_location(ast.Constant(value=constant), n)
            return ast.copy_location(ast.Constant(value='<name>'), n)

        def visit_Attribute(self, n):
            if n.attr == 'ANALYSIS_CONSTRAINT':
                return ast.copy_location(ast.Constant(value=constant), n)
            return self.generic_visit(n)

        def visit_JoinedStr(self, n):
            self.generic_visit(n)
            return n

        def visit_FormattedValue(self, n):
            return ast.copy_location(
                ast.Constant(value='<value>'), n)

    try:
        sub = _Sub().visit(ast.parse(ast.unparse(node), mode='eval'))
        ast.fix_missing_locations(sub)
        value = eval(compile(sub, '<site>', 'eval'), {'__builtins__': {}}, {})
    except Exception:
        return False
    return isinstance(value, str) and constant in value


def call_sites(constant):
    """(module, lineno, func, carries) for every analysis-run call."""
    out = []
    for mod in modules():
        path = os.path.join(PLUGIN, mod)
        try:
            tree = ast.parse(io.open(path, encoding='utf-8').read())
        except SyntaxError:
            continue
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            fn = node.func
            name = (fn.attr if isinstance(fn, ast.Attribute)
                    else fn.id if isinstance(fn, ast.Name) else None)
            if name not in ENTRIES:
                continue
            idx = ENTRIES[name]
            arg = None
            if len(node.args) > idx:
                arg = node.args[idx]
            else:
                for kw in node.keywords:
                    if kw.arg == INSTRUCTIONS_KW:
                        arg = kw.value
                        break
            if arg is None:
                continue
            out.append((mod, node.lineno, name,
                        _carries_constraint(arg, constant)))
    return out


# ------------------------------------------------------------- the checks

def t_every_site_uses_the_constant(sites):
    bad = [f'{m}:{ln}  {fn}()' for m, ln, fn, ok in sites if not ok]
    check(not bad,
          'every GUI analysis prompt is built from ANALYSIS_CONSTRAINT',
          ('' if not bad else
           '\n       these call sites spell their own constraint:\n         '
           + '\n         '.join(bad)
           + '\n       Import ANALYSIS_CONSTRAINT from .ai_backend and prefix'
             '\n       it, keeping only this site\'s own RESULT= tail. A'
             '\n       hand-written copy is how seven of eight quietly come to'
             '\n       mean something else.'))


def t_the_scan_is_not_vacuous(sites):
    check(len(sites) >= MIN_SITES,
          f'the scan still finds >= {MIN_SITES} analysis call sites',
          ('' if len(sites) >= MIN_SITES else
           f'\n       found {len(sites)}. Either a tab lost its Ask AI button,'
           '\n       or ENTRIES no longer matches how a run is started -- and'
           '\n       a scanner that matches nothing passes every other check'
           '\n       in this file.'))


def t_nobody_rebinds_the_constant():
    """`ANALYSIS_CONSTRAINT = "..."` outside ai_backend.py is a shadow.

    The evaluation in _carries_constraint cannot see this: a local rebind and
    the real import produce the identical call-site AST, so substituting the
    true value at the Name makes a shadowed site look compliant. This was a
    reproduced evasion. Assignment is the only legitimate-looking spelling of
    it, and outside the defining module there is no legitimate reason for one.
    """
    bad = []
    for mod in modules():
        path = os.path.join(PLUGIN, mod)
        try:
            tree = ast.parse(io.open(path, encoding='utf-8').read())
        except SyntaxError:
            continue
        for node in ast.walk(tree):
            targets = []
            if isinstance(node, ast.Assign):
                targets = node.targets
            elif isinstance(node, (ast.AnnAssign, ast.AugAssign)):
                targets = [node.target]
            elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                targets = [a for a in node.args.args
                           if a.arg == 'ANALYSIS_CONSTRAINT']
                if targets:
                    bad.append(f'{mod}:{node.lineno} (parameter)')
                continue
            for t in targets:
                if isinstance(t, ast.Name) and t.id == 'ANALYSIS_CONSTRAINT':
                    bad.append(f'{mod}:{node.lineno}')
    check(not bad,
          'no module rebinds ANALYSIS_CONSTRAINT over the import',
          ('' if not bad else
           '\n       ' + '\n       '.join(bad)
           + '\n       Import it from .ai_backend; a local binding of the same'
             '\n       name looks compliant to every check in this file.'))


def t_the_old_sentence_is_gone():
    bad = []
    for fn in sorted(os.listdir(PLUGIN)):
        if not fn.endswith('.py') or fn == 'ai_backend.py':
            continue
        src = io.open(os.path.join(PLUGIN, fn), encoding='utf-8').read()
        for i, line in enumerate(src.split('\n'), 1):
            if OLD_SENTENCE in line.lower():
                bad.append(f'{fn}:{i}')
    check(not bad, 'no module hand-writes the old constraint sentence',
          ('' if not bad else '\n       ' + ', '.join(bad)))


def t_the_constant_binds_the_child():
    sys.path.insert(0, ROOT)
    import types
    sys.modules.setdefault('wx', types.ModuleType('wx'))
    from kicad_routing_plugin import ai_backend
    c = ai_backend.ANALYSIS_CONSTRAINT
    # PINNED EXACTLY, not by substring. The four substring checks this
    # replaces were all satisfied by a constraint that said the OPPOSITE
    # ("the old rule ... no longer applies -- you MAY edit the board ... let
    # it write too"), including the one that printed "it still says the thing
    # it always said". A sentence can contain every keyword and mean the
    # reverse; only the whole text settles it. Changing the wording is fine --
    # update this literal in the same commit and the diff shows what moved.
    check(c == EXPECTED_CONSTRAINT,
          'ANALYSIS_CONSTRAINT is exactly the reviewed text',
          ('' if c == EXPECTED_CONSTRAINT else
           f'\n       got:      {c!r}'
           f'\n       expected: {EXPECTED_CONSTRAINT!r}'
           '\n       If the change is intended, update EXPECTED_CONSTRAINT in'
           '\n       the same commit -- the point is that a reviewer sees it.'))
    # VERDICT= matters because extract_result_line takes the LAST RESULT= line
    # of the run's final message. A child's line reaches it only if the parent
    # echoes it, so this is a precaution, not a demonstrated exploit.
    check(ai_backend.extract_result_line('RESULT=parent\nVERDICT=child\n')
          == 'parent',
          'a VERDICT= line is not mistaken for the run result')
    check(ai_backend.extract_result_line('RESULT=parent\nRESULT=child\n')
          == 'child',
          'an echoed RESULT= line WOULD displace it (last wins)')


def t_dispatch_is_granted_under_both_spellings():
    """#552 item 1, and the reason it is TWO names.

    Claude Code renamed the subagent tool: on 2.1.251 the dispatch event
    carries `"name":"Agent"`, while this repo has shipped `Task` since #633.
    A permission rule matches the canonical name only, so one name risks
    granting nothing on whichever CLI the user has. Both cost nothing.

    NOTE what the second check does NOT say. An earlier version called this
    "the floor a child inherits". It is not a floor: `--allowedTools`
    auto-approves rather than restricts (measured -- the run's init event
    reports permissionMode from the USER's settings and lists Write/Edit
    regardless), and `Bash` is on the list, and Bash writes files. All this
    asserts is that no dedicated write TOOL was added by #552.
    """
    sys.path.insert(0, ROOT)
    import types
    sys.modules.setdefault('wx', types.ModuleType('wx'))
    from kicad_routing_plugin import ai_backend, placement_run
    for label, spec in (('analysis', ai_backend.CLAUDE_ALLOWED_TOOLS),
                        ('placement', placement_run.PLACEMENT_ALLOWED_TOOLS)):
        tools = set(spec.split(','))
        check({'Agent', 'Task'} <= tools,
              f'the {label} allowlist grants dispatch under both spellings',
              spec)
    analysis = set(ai_backend.CLAUDE_ALLOWED_TOOLS.split(','))
    check(not ({'Write', 'Edit', 'NotebookEdit'} & analysis),
          'and #552 added no dedicated write tool to the analysis list',
          ai_backend.CLAUDE_ALLOWED_TOOLS)


TESTS = [
    't_every_site_uses_the_constant',
    't_the_scan_is_not_vacuous',
    't_nobody_rebinds_the_constant',
    't_the_old_sentence_is_gone',
    't_the_constant_binds_the_child',
    't_dispatch_is_granted_under_both_spellings',
]


def main():
    print('#552: the GUI analysis contract is stated once and binds the child')
    constant = _constraint()
    sites = call_sites(constant)
    t_every_site_uses_the_constant(sites)
    t_the_scan_is_not_vacuous(sites)
    t_nobody_rebinds_the_constant()
    t_the_old_sentence_is_gone()
    t_the_constant_binds_the_child()
    t_dispatch_is_granted_under_both_spellings()
    print(f'  ({len(sites)} analysis call sites scanned)')
    if FAILURES:
        print(f'FAIL: {len(FAILURES)} check(s)')
        return 1
    print('OK')
    return 0


if __name__ == '__main__':
    sys.exit(main())
