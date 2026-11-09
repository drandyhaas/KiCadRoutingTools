# Standalone CLI executable

The Windows release can include `krt.exe`, a single console executable built
from the KRT source tree. It contains the Python runtime, the Python packages
used by the core routing and placement tools, and the matching Rust extension.
Users do not install Python, Rust, pip, Git, or a compiler.

For developers, create a Windows x64 Python 3.12 environment and install the
locked build dependencies. Build the Rust extension with the repository's
`build_router.py` before freezing the CLI:

```text
python -m pip install -r build_tools/requirements-exe.txt
python build_router.py
python build_tools/build_exe.py
```

The build checks the source version, Rust extension version, native hash, and
dependency lock before invoking PyInstaller. It emits `dist/krt.exe` and a
sidecar build receipt. The executable runs as a console program so callers can
provide normal stdout, stderr, exit-code, and cancellation handling.

Examples:

```text
krt.exe --help
krt.exe --version
krt.exe self-test
krt.exe route input.kicad_pcb output.kicad_pcb
krt.exe place input.kicad_pcb placed.kicad_pcb --groups groups.json --intent intent.json
krt.exe judge-seeds judge.json --chain judge_chain.txt
krt.exe check-drc output.kicad_pcb
krt.exe check-connected output.kicad_pcb
krt.exe render-placement output.kicad_pcb --before input.kicad_pcb -o placement.png
```

The executable preserves the current working directory so board paths remain
relative to the caller. Internal KRT scripts run only from the build manifest;
the standalone entry point does not provide arbitrary Python script, `-m`, or
`-c` execution.

The standalone package provides KRT's algorithm CLI. KiCad's official DRC
engine, KiCad IPC, and optional AI agent integrations remain external tools and
are not silently bundled into this executable. The connectivity checker can
use KiCad to refill boards with zones, so that path still needs a KiCad
installation. Tools that require an external engine report that requirement.

`self-test` runs the bundled Rust, NumPy, SciPy, Shapely, Pillow and OR-Tools
libraries on small calculations. It prints a JSON result and exits nonzero on
failure. It does not download packages or write board files.

Internal subprocesses use `CREATE_NO_WINDOW` on Windows. A GUI host should
also hide the initial `krt.exe` process using its own process-launch options.
Internal tasks belong to a Windows Job and exit when their application worker
closes. This does not make a force kill of the extraction parent graceful.

The PyInstaller one-file bootloader extracts its contents to a temporary
directory during execution and removes them after normal termination. A force
kill can prevent that cleanup, so callers should stop the process gracefully.
