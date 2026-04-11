"""
Smoke test configuration for the diegosaiu-cpu KRT fork.

Adds the KRT project root to sys.path so that smoke tests can import
top-level modules (kicad_parser, obstacle_map, ...) without installing
the package.

Scope: only smoke tests in this directory. Upstream integration scripts
in tests/ are unaffected.
"""
import sys
from pathlib import Path

_PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))
