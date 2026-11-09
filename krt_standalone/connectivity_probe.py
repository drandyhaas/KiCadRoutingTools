"""The isolated connectivity measurement used by autoplan."""

import contextlib
import io
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'py_router'))
from check_connected import run_connectivity_check


if __name__ == '__main__':
    with contextlib.redirect_stdout(io.StringIO()):
        issues = run_connectivity_check(sys.argv[1], quiet=True)
    print('AUTOPLAN_OPEN: ' + json.dumps(sorted({
        issue['net_name'] for issue in issues if issue.get('net_name')
    })))
