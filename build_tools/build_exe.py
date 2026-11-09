"""Build the Windows single-file KiCadRoutingTools CLI."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import sys

from bundle_layout import ROOT, check_locked_dependencies, prepare_metadata


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='validate build inputs without freezing')
    args = parser.parse_args(argv)
    if sys.platform != 'win32' or platform.machine().lower() not in {'amd64', 'x86_64'}:
        parser.error('build on 64-bit Windows using 64-bit Python')
    if sys.version_info[:2] != (3, 12):
        parser.error('use Python 3.12 for the pinned standalone dependencies')
    if sys.maxsize <= 2 ** 32:
        parser.error('a 64-bit Python interpreter is required')
    check_locked_dependencies()
    generated = ROOT / 'build' / 'standalone-inputs'
    _, info_path = prepare_metadata(generated)
    print(f'Standalone inputs verified: {info_path}')
    if args.check:
        return 0
    subprocess.run([
        sys.executable, '-m', 'PyInstaller', '--noconfirm',
        '--workpath', str(ROOT / 'build' / 'standalone'),
        '--distpath', str(ROOT / 'dist'),
        str(ROOT / 'build_tools' / 'krt.spec'),
    ], cwd=ROOT, check=True)
    artifact = ROOT / 'dist' / 'krt.exe'
    receipt = {
        'schema': 1,
        'artifact': artifact.name,
        'bytes': artifact.stat().st_size,
        'sha256': hashlib.sha256(artifact.read_bytes()).hexdigest(),
        'build': json.loads(info_path.read_text(encoding='utf-8')),
    }
    receipt_path = artifact.with_suffix('.exe.build.json')
    receipt_path.write_text(json.dumps(receipt, indent=2), encoding='utf-8')
    print(f'Built {artifact} ({receipt["bytes"]} bytes, SHA256 {receipt["sha256"]})')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
