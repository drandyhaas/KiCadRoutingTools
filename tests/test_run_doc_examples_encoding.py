#!/usr/bin/env python3
"""tests/run_doc_examples.py runs a non-ASCII doc example without UTF-8 mode.

Found by a #1272-round reviewer. run_doc_examples writes each ```python block
to a temp script and runs it with `-X utf8`. The script was written in the
LOCALE encoding, while Python reads source as UTF-8: run without `-X utf8` on
Windows (cp1252) the em-dash in docs/api-routing-config.md died as
"SyntaxError: Non-UTF-8 code starting with '\\x97'". And the child's UTF-8
output was decoded in the locale encoding, where a byte cp1252 leaves
undefined (0x81, in U+0101's encoding) raises UnicodeDecodeError.

Each row runs run_doc_examples.main() over ONE synthetic doc in a child with
UTF-8 mode off (`-X utf8=0`, PYTHONUTF8 dropped, LC_ALL=C so a POSIX locale is
ASCII rather than UTF-8):

  locale   precondition: the child's locale encoding is not UTF-8 -- else the
           rows below pass whatever the code does, and say so.
  write    a block with a literal em-dash in its SOURCE runs. bb5943a0:
           "Non-UTF-8 code starting with '\\x97'" (Windows).
  read     an ASCII block PRINTING U+0101 runs. bb5943a0: UnicodeDecodeError
           in the runner (Windows).
  ascii    control: an all-ASCII block runs, as it always did.

    python3 tests/test_run_doc_examples_encoding.py
"""
import io
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))

DOCS = {
    'write': "label = 'plane taps \u2014 the fill model'\n"
             "assert label.split(' \u2014 ')[1] == 'the fill model'\n",
    'read': "print('\\u0101 \\u2014 \\u03a9')\n",
    'ascii': "assert 1 + 1 == 2\nprint('ascii only')\n",
}

DRIVER = '''import sys
sys.path.insert(0, sys.argv[2])
import run_doc_examples
run_doc_examples.DOCS = [sys.argv[1]]
sys.exit(run_doc_examples.main())
'''

FAILS = []


def child_env():
    env = dict(os.environ)
    env.pop('PYTHONUTF8', None)
    env.pop('PYTHONIOENCODING', None)
    env['LC_ALL'] = 'C'
    return env


def run_doc(tmp, name, body):
    doc = os.path.join(tmp, f'{name}.md')
    with io.open(doc, 'w', encoding='utf-8', newline='\n') as f:
        f.write(f'# {name}\n\n```python\n{body}```\n')
    driver = os.path.join(tmp, 'driver.py')
    with io.open(driver, 'w', encoding='utf-8') as f:
        f.write(DRIVER)
    r = subprocess.run([sys.executable, '-X', 'utf8=0', driver, doc, HERE],
                       capture_output=True, env=child_env(), timeout=600)
    out = (r.stdout + r.stderr).decode('utf-8', errors='replace')
    return r.returncode, out


def main():
    probe = subprocess.run(
        [sys.executable, '-X', 'utf8=0', '-c',
         'import locale; print(locale.getpreferredencoding(False))'],
        capture_output=True, text=True, env=child_env())
    enc = probe.stdout.strip()
    utf8_locale = enc.replace('-', '').lower() == 'utf8'
    print(f"child locale encoding: {enc!r}"
          + ("  (UTF-8: the rows cannot tell the fix from the bug here)"
             if utf8_locale else ""))
    with tempfile.TemporaryDirectory(prefix='doc_enc_') as tmp:
        for name, body in DOCS.items():
            rc, out = run_doc(tmp, name, body)
            ok = (rc == 0 and f'{name}.md block 0' in out
                  and '1 run, 0 skipped' in out and '0 failed' in out
                  and 'Traceback' not in out)
            print(f"  {'ok  ' if ok else 'FAIL'}  {name}: exit {rc}")
            if not ok:
                FAILS.append(name)
                print('\n'.join('      ' + ln for ln in out.splitlines()[-12:]))
    print('=' * 60)
    if FAILS:
        print(f"FAILED ({len(FAILS)}): {', '.join(FAILS)}")
        return 1
    print("PASS: run_doc_examples writes and reads its example scripts as UTF-8")
    return 0


if __name__ == '__main__':
    sys.exit(main())
