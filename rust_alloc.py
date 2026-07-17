"""Tune the Rust router's mimalloc allocator BEFORE ``grid_router`` is imported.

Import this module (``import rust_alloc``) ahead of the first
``from grid_router import ...`` in any process that routes.  It is a no-op on
every import after the first, and safe to import redundantly.

Why (issue #419)
----------------
The Rust router uses ``mimalloc`` as its global allocator (see
``rust_router/Cargo.toml``).  Its large, monotonically-growing structures --
the obstacle-map reference-count HashMaps (``blocked_cells`` per layer,
``blocked_vias``) -- are allocated through mimalloc.  With mimalloc's default
purge delay, freed segments are kept in a per-process cache and NOT returned to
the OS, so a rip-up-heavy route that builds/clones/discards many obstacle maps
accumulates the high-water mark of ALL of them instead of just the live set
(observed: multi-GB on heavy boards, growing 3.2 -> 5.4 -> 6.2 GB over one run).

On macOS these retained mimalloc arenas are mislabelled ``IOAccelerator`` by
``vmmap``/``footprint`` (they look like GPU-driver memory -- they are not; the
router does no GPU work).  On Linux the same retention shows up as RSS that
never drops; on Windows as working-set that never drops.  The cause and the fix
are the same on all platforms.

``MIMALLOC_PURGE_DELAY=0`` makes mimalloc decommit freed pages immediately
(``madvise(MADV_FREE/DONTNEED)`` on POSIX, ``VirtualFree(MEM_DECOMMIT)`` on
Windows).  Retired obstacle maps then return their memory to the OS instead of
piling up.  This is an allocator policy only -- it changes no routing
computation, so results are bit-identical (deterministic, DRC/connectivity
neutral).  Measured cost: within noise on real (compute-bound) routes; ~5-13%
only on a pathological pure-allocation microbenchmark.

Timing matters: mimalloc reads its options once at initialisation (when the
extension loads).  Setting the variable AFTER ``grid_router`` is imported is a
no-op (verified).  Hence this shim, imported first.
"""

import os

# The knob the rest of this file exists to set.  ``setdefault`` so an operator
# who exported a different value (e.g. for A/B measurement) wins.
_PURGE_KEY = "MIMALLOC_PURGE_DELAY"
_PURGE_VAL = "0"

if _PURGE_KEY not in os.environ:
    # POSIX (Linux, macOS on both Apple Silicon and Intel): os.environ write
    # calls setenv(), which mimalloc's getenv() reads.
    os.environ[_PURGE_KEY] = _PURGE_VAL

# Windows: mimalloc reads options via GetEnvironmentVariableA.  CPython's
# os.environ write goes through _wputenv, which the MSVC CRT normally syncs to
# the Win32 environment block -- but the router binary may link a different CRT,
# so make the propagation explicit via the Win32 API.  Guarded and best-effort:
# a failure here just falls back to whatever os.environ already did.
if os.name == "nt":  # pragma: no cover - exercised only on Windows
    try:
        import ctypes

        _kernel32 = ctypes.windll.kernel32
        # Only set the Win32 block if it isn't already visible there (avoid
        # clobbering an operator-provided value that reached Win32 already).
        _needed = _kernel32.GetEnvironmentVariableW(_PURGE_KEY, None, 0) == 0
        if _needed:
            _kernel32.SetEnvironmentVariableW(_PURGE_KEY, os.environ[_PURGE_KEY])
    except Exception:
        pass
