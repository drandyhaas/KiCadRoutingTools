#!/usr/bin/env python3
"""Build script for the Rust grid router module.

By default this downloads a prebuilt binary matching your OS from the project's
GitHub Releases page, so users do NOT need to install a Rust toolchain. If the
download fails (no network, no matching release, etc.) it falls back to a local
`cargo build --release`. Pass --from-source to skip the download and build
locally; pass --tag vX.Y.Z to pin a specific release.
"""

import sys
if sys.version_info[0] < 3:
    print("ERROR: Python 3 is required. You are running Python %d.%d." % sys.version_info[:2])
    print("Try:  python3 build_router.py")
    sys.exit(1)

import argparse
import json
import os
import platform
import shutil
import ssl
import subprocess
import urllib.error
import urllib.request


GITHUB_REPO = "drandyhaas/KiCadRoutingTools"


def _make_ssl_context():
    """Build an SSL context that works even when Python on macOS lacks system CAs.

    Tries the stdlib default first, then certifi if installed.
    """
    try:
        ctx = ssl.create_default_context()
        # Verify the bundle is actually usable by checking it has CAs loaded.
        if ctx.cert_store_stats().get('x509_ca', 0) > 0:
            return ctx
    except Exception:
        pass
    try:
        import certifi
        return ssl.create_default_context(cafile=certifi.where())
    except Exception:
        return ssl.create_default_context()  # last resort; may fail on https


_SSL_CTX = _make_ssl_context()


def _ssl_help_message():
    msg = ("SSL certificate verification failed. ")
    if sys.platform == 'darwin':
        msg += ("On macOS, run the 'Install Certificates.command' shipped with your "
                "Python install (in /Applications/Python <version>/), or "
                "`pip install certifi`, then re-run this script.")
    else:
        msg += "Install the `certifi` package (`pip install certifi`) and try again."
    return msg


def clean(script_dir, rust_dir):
    """Remove all Rust build outputs and compiled library files."""
    print("Cleaning Rust build outputs...")

    # Remove target directory (all Rust build outputs)
    target_dir = os.path.join(rust_dir, 'target')
    if os.path.exists(target_dir):
        print("Removing %s" % target_dir)
        shutil.rmtree(target_dir)

    # Remove compiled library files in rust_router directory
    lib_files = [
        os.path.join(rust_dir, 'grid_router.pyd'),
        os.path.join(rust_dir, 'grid_router.so'),
        os.path.join(rust_dir, 'grid_router.abi3.so'),
    ]
    for lib_file in lib_files:
        if os.path.exists(lib_file):
            print("Removing %s" % lib_file)
            os.remove(lib_file)

    # Remove stale copies in parent directory
    stale_files = [
        os.path.join(script_dir, 'grid_router.pyd'),
        os.path.join(script_dir, 'grid_router.so'),
        os.path.join(script_dir, 'grid_router.abi3.so'),
    ]
    for stale in stale_files:
        if os.path.exists(stale):
            print("Removing stale module: %s" % stale)
            os.remove(stale)

    print("Clean complete.")


def detect_platform_asset():
    """Return (asset_filename, local_dst_filename) for the running platform.

    asset_filename matches the names produced by .github/workflows/release.yml.
    local_dst_filename is what Python expects to import (grid_router.so on
    Mac/Linux, grid_router.pyd on Windows).
    """
    machine = platform.machine().lower()

    if sys.platform == 'win32':
        if machine not in ('amd64', 'x86_64'):
            return None, None
        return 'grid_router-windows-x86_64.pyd', 'grid_router.pyd'

    if sys.platform == 'darwin':
        if machine == 'arm64':
            return 'grid_router-macos-arm64.so', 'grid_router.so'
        if machine in ('x86_64', 'amd64'):
            return 'grid_router-macos-x86_64.so', 'grid_router.so'
        return None, None

    if sys.platform.startswith('linux'):
        if machine not in ('x86_64', 'amd64'):
            return None, None
        return 'grid_router-linux-x86_64.so', 'grid_router.so'

    return None, None


def fetch_release(tag):
    """Fetch a release's JSON metadata from GitHub. tag may be None for latest."""
    if tag:
        url = "https://api.github.com/repos/%s/releases/tags/%s" % (GITHUB_REPO, tag)
    else:
        url = "https://api.github.com/repos/%s/releases/latest" % GITHUB_REPO

    req = urllib.request.Request(url, headers={
        'Accept': 'application/vnd.github+json',
        'User-Agent': 'build_router.py',
    })
    with urllib.request.urlopen(req, timeout=30, context=_SSL_CTX) as resp:
        return json.loads(resp.read().decode('utf-8'))


def download_to(url, dst_path):
    """Stream a URL to disk."""
    req = urllib.request.Request(url, headers={'User-Agent': 'build_router.py'})
    with urllib.request.urlopen(req, timeout=120, context=_SSL_CTX) as resp, open(dst_path, 'wb') as f:
        shutil.copyfileobj(resp, f)


def try_download_prebuilt(script_dir, rust_dir, tag):
    """Attempt to download a prebuilt binary. Returns True on success."""
    asset_name, dst_name = detect_platform_asset()
    if asset_name is None:
        print("No prebuilt binary is published for this platform "
              "(%s %s)." % (sys.platform, platform.machine()))
        return False

    label = tag if tag else "latest"
    print("Looking up %s release on github.com/%s ..." % (label, GITHUB_REPO))

    try:
        release = fetch_release(tag)
    except urllib.error.HTTPError as e:
        print("ERROR: GitHub returned HTTP %s for %s release." % (e.code, label))
        if e.code == 404:
            print("       (No matching release exists yet.)")
        return False
    except ssl.SSLError as e:
        print("ERROR: %s" % e)
        print("       %s" % _ssl_help_message())
        return False
    except (urllib.error.URLError, TimeoutError, OSError) as e:
        if isinstance(getattr(e, 'reason', None), ssl.SSLError):
            print("ERROR: %s" % e.reason)
            print("       %s" % _ssl_help_message())
        else:
            print("ERROR: Could not reach GitHub: %s" % e)
        return False

    assets = release.get('assets', [])
    asset = next((a for a in assets if a.get('name') == asset_name), None)
    if asset is None:
        names = ', '.join(a.get('name', '?') for a in assets) or '(none)'
        print("ERROR: Release %s has no asset named %s. Available: %s"
              % (release.get('tag_name', label), asset_name, names))
        return False

    dst = os.path.join(rust_dir, dst_name)
    download_url = asset['browser_download_url']
    print("Downloading %s (%s bytes) -> %s" % (asset_name, asset.get('size', '?'), dst))
    try:
        download_to(download_url, dst)
    except ssl.SSLError as e:
        print("ERROR: %s" % e)
        print("       %s" % _ssl_help_message())
        return False
    except (urllib.error.URLError, TimeoutError, OSError) as e:
        if isinstance(getattr(e, 'reason', None), ssl.SSLError):
            print("ERROR: %s" % e.reason)
            print("       %s" % _ssl_help_message())
        else:
            print("ERROR: Download failed: %s" % e)
        return False

    # Remove stale copies in parent directory before importing
    _remove_stale_copies(script_dir)

    if not _verify_import(rust_dir):
        print("ERROR: Downloaded binary failed to import. Try --from-source.")
        try:
            os.remove(dst)
        except OSError:
            pass
        return False

    print("Installed prebuilt binary from release %s." % release.get('tag_name', label))
    return True


def build_from_source(script_dir, rust_dir):
    """Build the Rust router module locally with cargo."""
    print("Building Rust router from source...")
    try:
        result = subprocess.run(
            ['cargo', 'build', '--release'],
            cwd=rust_dir,
            capture_output=False
        )
    except FileNotFoundError:
        print("ERROR: Rust is not installed or 'cargo' is not in PATH.")
        print()
        print("Either install Rust, or run this script without --from-source to")
        print("download a prebuilt binary from GitHub Releases.")
        print()
        print("To install Rust, follow these instructions:")
        print()
        if sys.platform == 'win32':
            print("  Windows:")
            print("    1. Download rustup-init.exe from https://rustup.rs/")
            print("    2. Run the installer and follow the prompts")
            print("    3. Restart your terminal/command prompt")
            print()
            print("  Or using winget:")
            print("    winget install Rustlang.Rustup")
        elif sys.platform == 'darwin':
            print("  macOS:")
            print("    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh")
            print()
            print("  Or using Homebrew:")
            print("    brew install rust")
        else:
            print("  Linux:")
            print("    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh")
            print()
            print("  Or using your package manager:")
            print("    Ubuntu/Debian: sudo apt install rustc cargo")
            print("    Fedora: sudo dnf install rust cargo")
            print("    Arch: sudo pacman -S rust")
        print()
        print("After installation, restart your terminal and run this script again.")
        sys.exit(1)

    if result.returncode != 0:
        print("ERROR: Rust build failed!")
        sys.exit(1)

    # Determine source and destination paths
    if sys.platform == 'win32':
        src = os.path.join(rust_dir, 'target', 'release', 'grid_router.dll')
        dst = os.path.join(rust_dir, 'grid_router.pyd')
    elif sys.platform == 'darwin':
        src = os.path.join(rust_dir, 'target', 'release', 'libgrid_router.dylib')
        dst = os.path.join(rust_dir, 'grid_router.so')
    else:
        src = os.path.join(rust_dir, 'target', 'release', 'libgrid_router.so')
        dst = os.path.join(rust_dir, 'grid_router.so')

    print("Copying %s -> %s" % (src, dst))
    shutil.copy2(src, dst)

    _remove_stale_copies(script_dir)

    if not _verify_import(rust_dir):
        print("ERROR: Locally-built binary failed to import.")
        sys.exit(1)


def _remove_stale_copies(script_dir):
    """Remove grid_router copies that may shadow the rust_router/ one on sys.path."""
    stale_files = [
        os.path.join(script_dir, 'grid_router.pyd'),
        os.path.join(script_dir, 'grid_router.so'),
        os.path.join(script_dir, 'grid_router.abi3.so'),
    ]
    for stale in stale_files:
        if os.path.exists(stale):
            print("Removing stale module: %s" % stale)
            os.remove(stale)


def _verify_import(rust_dir):
    """Import grid_router from rust_dir and print its version. Returns True on success."""
    if rust_dir not in sys.path:
        sys.path.insert(0, rust_dir)
    if 'grid_router' in sys.modules:
        del sys.modules['grid_router']
    try:
        import grid_router
    except ImportError as e:
        print("Import failed: %s" % e)
        return False
    print("grid_router v%s ready." % getattr(grid_router, '__version__', 'unknown'))
    return True


def main():
    parser = argparse.ArgumentParser(
        description="Install the Rust grid router module (downloads a prebuilt "
                    "binary by default; falls back to building locally).")
    parser.add_argument('--clean', action='store_true',
                        help="Remove all Rust build outputs and compiled library files")
    parser.add_argument('--from-source', action='store_true',
                        help="Skip the prebuilt download and build locally with cargo")
    parser.add_argument('--tag', default=None,
                        help="Download from a specific release tag (e.g. v0.15.0) "
                             "instead of the latest release")
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    rust_dir = os.path.join(script_dir, 'rust_router')

    if args.clean:
        clean(script_dir, rust_dir)
        return

    if args.from_source:
        build_from_source(script_dir, rust_dir)
        return

    if try_download_prebuilt(script_dir, rust_dir, args.tag):
        return

    print()
    print("Falling back to local build...")
    build_from_source(script_dir, rust_dir)


if __name__ == '__main__':
    main()
