#!/usr/bin/env python3
"""The --bus progress output must be writable to any console encoding.

Python on Windows encodes stdout with the console code page (cp1252, cp437,
...), not UTF-8, so a non-ASCII character in routing progress text raises
UnicodeEncodeError and aborts the run. The bus-detection block printed its
net order with U+2192 arrows ("sources→targets", "A → B"), so
``route.py <board> --bus`` crashed at the first detected bus group on a stock
Windows console (single_ended_loop.py:192). ASCII is the only intersection of
the console code pages, so this test pins the invariant at the source: every
string constant in the bus-path modules must be pure ASCII.

    python3 tests/test_bus_output_ascii.py
"""
import ast
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

BUS_PATH_MODULES = ["single_ended_loop.py", "bus_detection.py"]


def non_ascii_string_constants(path):
    """(lineno, repr) of every non-ASCII string constant in the module.

    ast.walk reaches plain literals and the Constant parts of f-strings
    (JoinedStr), which is where the crash-causing text lives.
    """
    with open(path, encoding="utf-8") as f:
        tree = ast.parse(f.read(), filename=path)
    offenders = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Constant) and isinstance(node.value, str):
            if not node.value.isascii():
                offenders.append((node.lineno, repr(node.value)))
    return offenders


def run():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    fails = []
    for module in BUS_PATH_MODULES:
        for lineno, text in non_ascii_string_constants(os.path.join(root, module)):
            fails.append(f"{module}:{lineno}: non-ASCII string constant {text}")

    print("=" * 60)
    if fails:
        for f in fails:
            print(f"  FAIL  {f}")
        print(f"\n{len(fails)} failure(s)")
        return 1
    print("  PASS  bus-path modules contain only ASCII string constants")
    print(f"        ({', '.join(BUS_PATH_MODULES)})")
    return 0


if __name__ == "__main__":
    sys.exit(run())
