import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'build_tools'))
from bundle_layout import (  # noqa: E402
    check_locked_dependencies,
    native_identity,
    prepare_metadata,
    runtime_resources,
    runtime_sources,
)


def test_standalone_inputs_have_native_and_runtime_contract():
    assert (ROOT / 'VERSION').is_file()
    assert (ROOT / 'rust_router' / 'Cargo.toml').is_file()
    assert (ROOT / 'rust_router' / 'grid_router.pyd').is_file()
    check_locked_dependencies()
    identity = native_identity()
    assert identity['version'] == '0.23.7'
    assert len(identity['sha256']) == 64
    source_names = {path.relative_to(ROOT).as_posix() for path in runtime_sources()}
    assert 'krt_capabilities.py' in source_names
    assert 'py_router/route.py' in source_names
    assert 'py_placer/place_simple.py' in source_names
    resource_names = {path.relative_to(ROOT).as_posix() for path in runtime_resources()}
    assert {'VERSION', 'requirements.txt', 'rust_router/Cargo.toml'} <= resource_names


def test_metadata_manifest_is_reproducible_and_scoped():
    output = ROOT / 'build' / 'test-standalone-inputs'
    output.mkdir(parents=True, exist_ok=True)
    try:
        scripts_path, info_path = prepare_metadata(output)
        scripts = json.loads(scripts_path.read_text(encoding='utf-8'))
        info = json.loads(info_path.read_text(encoding='utf-8'))
        assert scripts['schema'] == 1
        assert 'py_router/route.py' in scripts['scripts']
        assert 'krt_capabilities.py' in scripts['scripts']
        assert info['schema'] == 1
        assert info['platform'] == 'windows-x86_64'
        assert info['native']['version'] == '0.23.7'
        assert all(not name.startswith('..') for name in info['files'])
    finally:
        for path in output.glob('*'):
            path.unlink()
        output.rmdir()
