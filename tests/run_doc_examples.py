"""Verify the Python API documentation examples.

Extracts every ```python block from docs/python-api.md and docs/api-*.md,
and runs each block that is a complete example (from the repository root, so
the kicad_files/ paths resolve). Signature listings and intentionally
incomplete fragments are skipped: blocks that don't compile, reference
`pcbnew`, use a `pcb` object without parsing one, or are bare dict/class
literals.

Usage:
    python3 tests/run_doc_examples.py

Exits non-zero if any example fails.
"""
import ast
import os
import re
import subprocess
import sys
import tempfile

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

DOCS = [
    'docs/python-api.md',
    'docs/api-kicad-parser.md',
    'docs/api-kicad-writer.md',
    'docs/api-pcb-modification.md',
    'docs/api-routing-config.md',
    'docs/api-net-analysis.md',
    'docs/api-impedance.md',
]

# Files the examples create in the working directory; removed afterwards.
EXAMPLE_OUTPUTS = ['quickstart_output.kicad_pcb', 'box_output.kicad_pcb']


def is_fragment(block):
    """Blocks that are illustrative fragments rather than complete examples."""
    if re.search(r'\bpcbnew\b|results=results|^\{|@dataclass', block, re.MULTILINE):
        return True
    # Uses a parsed board without parsing one
    if 'pcb.' in block and 'parse_kicad_pcb(' not in block:
        return True
    return False


def gridrouteconfig_undocumented_fields():
    """#382 E11: every public GridRouteConfig field must appear in
    docs/api-routing-config.md, so a newly-added routing knob can't ship
    undocumented. Returns the list of dataclass fields missing from the doc's
    field tables (empty == parity)."""
    src = open(os.path.join(REPO_ROOT, 'py_router', 'routing_config.py'), encoding='utf-8').read()
    fields = [s.target.id
              for node in ast.walk(ast.parse(src))
              if isinstance(node, ast.ClassDef) and node.name == 'GridRouteConfig'
              for s in node.body
              if isinstance(s, ast.AnnAssign) and isinstance(s.target, ast.Name)
              and not s.target.id.startswith('_')]
    doc = open(os.path.join(REPO_ROOT, 'docs/api-routing-config.md'), encoding='utf-8').read()
    documented = set(re.findall(r'\|\s*`([a-z_][a-z0-9_]*)`\s*\|', doc))
    return [f for f in fields if f not in documented]


def gridrouteconfig_stale_values():
    """#923: the VALUES `docs/configuration.md` shows for that same dataclass.

    Its field-parity sibling above holds the NAMES, so a new knob cannot ship
    undocumented -- and the doc went on quoting `heuristic_weight: float = 1.9`
    after #586 made it 2.3, `via_cost: int = 50` after it became 75, and four
    more, because nothing held the numbers. A reader reasons from the number,
    not from the field's presence.

    Returns [(field, documented, real)], empty when they agree.
    """
    src = open(os.path.join(REPO_ROOT, 'py_router', 'routing_config.py'),
               encoding='utf-8').read()
    real = {}
    for node in ast.walk(ast.parse(src)):
        if isinstance(node, ast.ClassDef) and node.name == 'GridRouteConfig':
            for s in node.body:
                if (isinstance(s, ast.AnnAssign) and isinstance(s.target, ast.Name)
                        and s.value is not None):
                    try:
                        real[s.target.id] = ast.literal_eval(s.value)
                    except (ValueError, SyntaxError):
                        pass                 # a field_factory or an expression
    doc = open(os.path.join(REPO_ROOT, 'docs/configuration.md'),
               encoding='utf-8').read()
    out = []
    for m in re.finditer(r'^\s{4}([a-z_]+):\s*(?:int|float|bool|str)\s*='
                         r'\s*([^\s#]+)', doc, re.M):
        name, shown = m.group(1), m.group(2).rstrip(',')
        if name not in real:
            continue
        try:
            value = ast.literal_eval(shown)
        except (ValueError, SyntaxError):
            continue
        if value != real[name]:
            out.append((name, value, real[name]))
    return out


def main():
    failures = []
    ran = skipped = 0

    stale = gridrouteconfig_stale_values()
    if stale:
        failures.append('GridRouteConfig value parity')
        print('FAIL docs/configuration.md quotes stale defaults: '
              + ', '.join(f'{n} = {d!r} (real {r!r})' for n, d, r in stale))
    else:
        print('PASS GridRouteConfig value parity (docs/configuration.md)')

    undoc = gridrouteconfig_undocumented_fields()
    if undoc:
        failures.append('GridRouteConfig field parity')
        print('FAIL GridRouteConfig fields missing from docs/api-routing-config.md: '
              + ', '.join(undoc))
    else:
        print('PASS GridRouteConfig field parity (all documented)')
    for doc in DOCS:
        text = open(os.path.join(REPO_ROOT, doc), encoding='utf-8').read()
        for i, block in enumerate(re.findall(r'```python\n(.*?)```', text, re.DOTALL)):
            try:
                compile(block, f'{doc}:block{i}', 'exec')
            except SyntaxError:
                skipped += 1  # signature listing
                continue
            if is_fragment(block):
                skipped += 1
                continue
            with tempfile.NamedTemporaryFile('w', suffix='.py', dir=REPO_ROOT,
                                             delete=False) as f:
                f.write(block)
                path = f.name
            try:
                # #522: examples import the engine flat; scripts run
                # from a temp file at REPO_ROOT, so hand them the layout.
                _env = dict(os.environ)
                _env['PYTHONPATH'] = os.pathsep.join(
                    [os.path.join(REPO_ROOT, 'py_router'),
                     os.path.join(REPO_ROOT, 'py_tools'),
                     _env.get('PYTHONPATH', '')]).rstrip(os.pathsep)
                r = subprocess.run([sys.executable, '-X', 'utf8', path],
                                   capture_output=True, text=True,
                                   timeout=300, cwd=REPO_ROOT, env=_env)
            finally:
                os.unlink(path)
            ran += 1
            tag = f'{doc} block {i}'
            if r.returncode != 0:
                failures.append(tag)
                print(f'FAIL {tag}\n{r.stderr}')
            else:
                print(f'PASS {tag}')

    for name in EXAMPLE_OUTPUTS:
        path = os.path.join(REPO_ROOT, name)
        if os.path.exists(path):
            os.unlink(path)

    print(f'\n{ran} run, {skipped} skipped (signature listings/fragments), '
          f'{len(failures)} failed')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
