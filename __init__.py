# KiCad Routing Tools - SWIG Action Plugin Registration
# This file must be at the root of the plugin directory for KiCad to find it

import os
import platform
import shutil
import sys

# Add this directory to Python path so kicad_routing_plugin can be found
_plugin_dir = os.path.dirname(os.path.abspath(__file__))
if _plugin_dir not in sys.path:
    sys.path.insert(0, _plugin_dir)


def _resolve_rust_binary():
    """If the Rust router was shipped with platform-suffix names (multi-arch
    macOS PCM package), copy the right one to the canonical filename so
    `import grid_router` works. No-op when grid_router.{so,pyd} already exists.
    """
    rust_dir = os.path.join(_plugin_dir, "rust_router")
    if not os.path.isdir(rust_dir):
        return
    canonical = "grid_router.pyd" if sys.platform == "win32" else "grid_router.so"
    if os.path.exists(os.path.join(rust_dir, canonical)):
        return
    machine = platform.machine().lower()
    candidates = []
    if sys.platform == "darwin":
        if machine == "arm64":
            candidates = ["grid_router-macos-arm64.so"]
        elif machine in ("x86_64", "amd64"):
            candidates = ["grid_router-macos-x86_64.so"]
    elif sys.platform.startswith("linux") and machine in ("x86_64", "amd64"):
        candidates = ["grid_router-linux-x86_64.so"]
    elif sys.platform == "win32" and machine in ("amd64", "x86_64"):
        candidates = ["grid_router-windows-x86_64.pyd"]
    for name in candidates:
        src = os.path.join(rust_dir, name)
        if os.path.exists(src):
            try:
                shutil.copy2(src, os.path.join(rust_dir, canonical))
            except OSError:
                pass
            return


_resolve_rust_binary()

try:
    from kicad_routing_plugin.action_plugin import KiCadRoutingToolsPlugin
    KiCadRoutingToolsPlugin().register()
except Exception as e:
    import logging
    logging.getLogger("KiCadRoutingTools").error(f"Failed to register plugin: {e}")
