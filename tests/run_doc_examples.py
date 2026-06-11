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


def main():
    failures = []
    ran = skipped = 0
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
                r = subprocess.run([sys.executable, '-X', 'utf8', path],
                                   capture_output=True, text=True,
                                   timeout=300, cwd=REPO_ROOT)
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
