"""Inputs for the standalone CLI bundle."""

from __future__ import annotations

import ast
import hashlib
import importlib.metadata
import json
import platform
from pathlib import Path
import subprocess
import sys
import tomllib


ROOT = Path(__file__).resolve().parents[1]
RUNTIME_DIRS = ('py_router', 'py_placer', 'py_tools', 'awx', 'krt_standalone')
ROOT_SOURCES = ('krt_capabilities.py',)
RESOURCE_SUFFIXES = frozenset({'.json', '.txt', '.yaml', '.yml', '.csv', '.ttf', '.otf'})
THIRD_PARTY_ROOTS = frozenset({
    'numpy', 'scipy', 'shapely', 'PIL', 'ortools', 'pandas', 'google',
    'absl', 'immutabledict', 'dateutil', 'typing_extensions',
})
RUNTIME_DISTRIBUTIONS = (
    'numpy', 'scipy', 'shapely', 'pillow', 'ortools', 'pandas', 'protobuf',
    'absl-py', 'immutabledict', 'python-dateutil', 'typing-extensions', 'tzdata',
)


def runtime_sources(root: Path = ROOT) -> tuple[Path, ...]:
    files = {root / name for name in ROOT_SOURCES if (root / name).is_file()}
    for directory in RUNTIME_DIRS:
        files.update((root / directory).rglob('*.py'))
    skill_root = root / '.claude' / 'skills'
    for name in ('plan-pcb-placement', 'plan-pcb-routing'):
        files.update((skill_root / name).rglob('*.py'))
    return tuple(sorted(files))


def runtime_resources(root: Path = ROOT) -> tuple[Path, ...]:
    files = {root / name for name in (
        'VERSION', 'requirements.txt', 'LICENSE', 'rust_router/Cargo.toml',
    )}
    files.update((root / 'docs').rglob('*.md'))
    for directory in RUNTIME_DIRS:
        files.update(path for path in (root / directory).rglob('*')
                     if path.is_file() and path.suffix.lower() in RESOURCE_SUFFIXES)
    for name in ('plan-pcb-placement', 'plan-pcb-routing'):
        files.update(path for path in (root / '.claude' / 'skills' / name).rglob('*')
                     if path.is_file() and path.suffix not in {'.py', '.pyc'})
    return tuple(sorted(files))


def source_hidden_imports(sources: tuple[Path, ...]) -> list[str]:
    roots = sys.stdlib_module_names | THIRD_PARTY_ROOTS
    found = set()
    for path in sources:
        tree = ast.parse(path.read_text(encoding='utf-8-sig'), filename=str(path))
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                names = [alias.name for alias in node.names]
            elif isinstance(node, ast.ImportFrom) and not node.level and node.module:
                names = [node.module]
            else:
                continue
            found.update(name for name in names if name.split('.')[0] in roots)
    return sorted(found)


def installed_dependencies() -> dict[str, str]:
    names = (*RUNTIME_DISTRIBUTIONS, 'pyinstaller', 'pyinstaller-hooks-contrib')
    return {name: importlib.metadata.version(name) for name in names}


def check_locked_dependencies(root: Path = ROOT) -> None:
    for line in (root / 'build_tools' / 'requirements-exe.txt').read_text(encoding='utf-8-sig').splitlines():
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        name, expected = line.split('==', 1)
        actual = importlib.metadata.version(name)
        if actual != expected:
            raise RuntimeError(f'EXE_DEPENDENCY_VERSION_MISMATCH: {name}={actual}, expected {expected}')


def native_identity(root: Path = ROOT) -> dict[str, str]:
    native = root / 'rust_router' / 'grid_router.pyd'
    expected = tomllib.loads((native.parent / 'Cargo.toml').read_text())['package']['version']
    probe = subprocess.run([
        sys.executable, '-I', '-c',
        'import sys; sys.path.insert(0, sys.argv[1]); '
        'import grid_router; print(grid_router.__version__)',
        str(native.parent),
    ], check=True, capture_output=True, text=True)
    actual = probe.stdout.strip()
    if actual != expected:
        raise RuntimeError(f'EXE_NATIVE_VERSION_MISMATCH: {actual}, expected {expected}')
    return {'version': actual, 'sha256': hashlib.sha256(native.read_bytes()).hexdigest()}


def source_commit(root: Path = ROOT) -> str | None:
    try:
        result = subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=root,
                                capture_output=True, text=True, check=True)
    except (FileNotFoundError, subprocess.CalledProcessError):
        return None
    return result.stdout.strip()


def prepare_metadata(output: Path, root: Path = ROOT) -> tuple[Path, Path]:
    sources = runtime_sources(root)
    resources = runtime_resources(root)
    missing = [str(path.relative_to(root)) for path in resources if not path.is_file()]
    if missing:
        raise RuntimeError(f'EXE_RESOURCE_MISSING: {", ".join(missing)}')
    scripts = [path.relative_to(root).as_posix() for path in sources]
    metadata = {
        'schema': 1,
        'version': (root / 'VERSION').read_text().strip(),
        'source_commit': source_commit(root),
        'python': platform.python_version(),
        'platform': 'windows-x86_64',
        'native': native_identity(root),
        'dependencies': installed_dependencies(),
        'files': {
            path.relative_to(root).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in (*sources, *resources)
        },
    }
    output.mkdir(parents=True, exist_ok=True)
    script_path = output / 'scripts.json'
    info_path = output / 'build-info.json'
    script_path.write_text(json.dumps({'schema': 1, 'scripts': scripts}, indent=2), encoding='utf-8')
    info_path.write_text(json.dumps(metadata, indent=2), encoding='utf-8')
    return script_path, info_path
