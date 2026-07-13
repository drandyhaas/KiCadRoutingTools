"""#382 E7: pytest scaffolding (forward-looking).

The existing 127 test scripts are bare `if __name__ == "__main__": sys.exit(main())`
programs run by tests/run_all.py (and directly). They keep working unchanged --
this file only sets up NEW pytest-style tests written as `def test_*` functions:

- puts the repo root (and rust_router/) on sys.path so a pytest test can
  `import kicad_parser` / `import synth` without the per-file sys.path hack;
- registers the `integration` marker (`@pytest.mark.integration` for tests that
  shell out to a CLI / route a real board);
- adds `--fast`, which deselects integration-marked tests (mirrors
  run_all.py --fast).

Note: pytest is optional and may not be installed; run_all.py is the primary
runner. This file is inert unless pytest is used.
"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
for _p in (_ROOT, os.path.join(_ROOT, 'rust_router')):
    if _p not in sys.path:
        sys.path.insert(0, _p)


def pytest_configure(config):
    config.addinivalue_line(
        "markers",
        "integration: test shells out to a CLI or routes a real board (slow); "
        "deselected by --fast.",
    )


def pytest_addoption(parser):
    parser.addoption(
        "--fast", action="store_true", default=False,
        help="Skip integration-marked (slow) tests.",
    )


def pytest_collection_modifyitems(config, items):
    if not config.getoption("--fast"):
        return
    import pytest
    skip = pytest.mark.skip(reason="--fast: integration test skipped")
    for item in items:
        if "integration" in item.keywords:
            item.add_marker(skip)
