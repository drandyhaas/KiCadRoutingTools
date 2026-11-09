# -*- mode: python ; coding: utf-8 -*-
from pathlib import Path
import sys
import importlib.util

from PyInstaller.utils.hooks import collect_data_files, collect_dynamic_libs, collect_submodules

root = Path(SPECPATH).resolve().parent
sys.path.insert(0, str(root / 'build_tools'))
from bundle_layout import ROOT, prepare_metadata, runtime_resources, runtime_sources, source_hidden_imports

generated = ROOT / 'build' / 'standalone-inputs'
script_manifest, build_info = prepare_metadata(generated)
sources = runtime_sources()
datas = [(str(path), 'krt/' + path.relative_to(ROOT).parent.as_posix())
         for path in (*sources, *runtime_resources())]
datas += [(str(script_manifest), 'krt'), (str(build_info), 'krt')]
datas += collect_data_files('ortools')
datas += collect_data_files('google.protobuf')
hiddenimports = source_hidden_imports(sources)
hiddenimports += [
    'grid_router', 'numpy', 'scipy.optimize', 'scipy.spatial', 'shapely',
    'PIL.Image', 'PIL.ImageFont', 'pandas', 'ortools.sat.python.cp_model',
]
hiddenimports += collect_submodules('ortools', filter=lambda name: '.tests' not in name)
hiddenimports += collect_submodules('google.protobuf', filter=lambda name: '.test' not in name)
binaries = collect_dynamic_libs('ortools')
ortools_origin = Path(importlib.util.find_spec('ortools').origin).parent
binaries += [
    (str(path), 'ortools/.libs')
    for path in (ortools_origin / '.libs').glob('*.dll')
]
binaries += [(str(ROOT / 'rust_router' / 'grid_router.pyd'), 'krt/rust_router')]
a = Analysis(
    [str(ROOT / 'krt_cli.py')],
    pathex=[str(ROOT), str(ROOT / 'rust_router')],
    binaries=binaries,
    datas=datas,
    hiddenimports=sorted(set(hiddenimports)),
    excludes=['tkinter', 'PyQt5', 'PyQt6', 'PySide2', 'PySide6', 'wx', 'modal'],
    noarchive=False,
)
pyz = PYZ(a.pure)
exe = EXE(
    pyz, a.scripts, a.binaries, a.datas,
    [('X utf8', None, 'OPTION'), ('u', None, 'OPTION')],
    name='krt',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=False,
    console=True,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)
