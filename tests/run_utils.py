"""
Shared utilities for test scripts.
"""

import os
import shlex
import subprocess

# Get the root directory (parent of tests/)
TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)


def run(cmd: str, unbuffered: bool = False) -> None:
    """Run a command string and print output.

    Commands are executed from the project root directory, so paths like
    'python3 route.py' and 'kicad_files/...' work correctly.

    Args:
        cmd: Command string to run (will be parsed using shell-style splitting)
        unbuffered: If True, add -u flag to python commands
    """
    if unbuffered and cmd.startswith('python3 '):
        cmd = 'python3 -u ' + cmd[8:]
    print(f"\n>>> {cmd}")
    args = shlex.split(cmd)
    result = subprocess.run(args, cwd=ROOT_DIR)
    if result.returncode != 0:
        print(f"Command failed with exit code {result.returncode}")
