"""Where KiCad is, on any platform, any drive, any version (#763).

ONE place that knows how a KiCad install is laid out. Before this module the
knowledge was duplicated in `kicad_exact_fill.kicad_python_candidates()` and
again in `install_plugin.py`, and BOTH copies hardcoded drive C:, so a KiCad
installed on D: was invisible to the installer *and* to every exact-fill
consumer. The reporter's symptom was the installer announcing "no KiCad
version" and silently falling back to the system python (#763).

Three rules this module exists to keep:

1. **Never hardcode a drive.** Windows installs go wherever the user put them.
   We ask, in order: the REGISTRY (authoritative -- it records the actual
   install), the `ProgramFiles*` environment variables (which follow a
   non-default install root), `PATH`, and only then a scan of every fixed
   drive. The scan is the backstop, not the plan.

2. **Never hardcode a version.** `KiCad\\10.0\\` today, `KiCad\\11.0\\`
   tomorrow. Every lookup GLOBS the version component and sorts numerically
   (see `version_key` -- a string sort puts "9.0" above "10.0" and would hand a
   KiCad 10 user their KiCad 9).

3. **Ask the OS where "Documents" is, never guess.** Windows redirects the
   Documents known folder into OneDrive on some machines and not others, and
   OneDrive creates its own `Documents` folder even when it is NOT the
   redirect target. Guessing by existence picks the decoy: that is exactly how
   the plugin landed in a OneDrive folder KiCad never reads (#763). The
   known-folder API is the same thing KiCad itself calls.

Everything here is stdlib-only and import-safe: `install_plugin.py` runs on a
bare system python BEFORE dependencies are installed.

The filesystem/environment/registry accesses are injectable parameters so the
platform layouts can be simulated in tests on any host -- a Windows drive scan
is otherwise untestable on the macOS/Linux boxes this repo is developed on.
"""
from __future__ import annotations

import glob
import ntpath
import os
import posixpath
import platform
import re
import sys
from typing import Callable, Dict, List, Optional, Sequence, Tuple

# Version directories we know how to name, newest first. This is only a
# PREFERENCE ORDER for reporting -- discovery itself globs, so a version not
# listed here (11.0, a future nightly) is still found. Never gate on it.
KNOWN_VERSIONS = ("10.0", "9.99", "9.0")


def version_key(text: str) -> Tuple[int, ...]:
    """Numeric sort key for a version-ish string ('10.0' -> (10, 0)).

    `()` for something with no digits, which sorts BELOW every real version --
    so an unversioned path is always the last resort, never a winner.
    """
    return tuple(int(p) for p in re.findall(r'\d+', text or ''))


def _basename(path: str) -> str:
    """Last component of a path, splitting on BOTH separators.

    os.path.basename does not split a backslash off-Windows, so it hands back
    the whole `C:\\...\\KiCad\\10.0` string as the "name" -- correct on the one
    platform that matters and silently wrong everywhere it could be tested.
    Same reasoning as path_version_key below.
    """
    return re.split(r'[\\/]', path.rstrip('\\/'))[-1] if path else ''


def path_version_key(path: str) -> Tuple[int, ...]:
    """Numeric version of the `KiCad/<ver>/` component of a path.

    Matches BOTH separators rather than using os.path: off-Windows,
    os.path.dirname does not split a backslash path, so an os.path-based key
    collapses to a constant -- correct on the only platform that matters, and
    untestable everywhere else.
    """
    m = re.search(r'[\\/]KiCad[\\/]([0-9][0-9.]*)[\\/]', path)
    return version_key(m.group(1)) if m else ()


# --------------------------------------------------------------------------
# Windows: drives, program roots, registry
# --------------------------------------------------------------------------

def windows_drives(getlogicaldrives: Optional[Callable[[], int]] = None,
                   exists: Callable[[str], bool] = os.path.exists) -> List[str]:
    """Fixed drive roots present on this machine, e.g. ['C:\\\\', 'D:\\\\'].

    Uses the kernel32 bitmask when available -- probing A:/B: by existence can
    block for seconds on a machine with a floppy/empty card reader mapped, and
    the whole point of this scan is that it is the cheap backstop.
    """
    if getlogicaldrives is None:
        try:                                     # pragma: no cover - Windows
            import ctypes
            getlogicaldrives = ctypes.windll.kernel32.GetLogicalDrives  # type: ignore[attr-defined]
        except Exception:
            getlogicaldrives = None
    if getlogicaldrives is not None:
        try:
            mask = getlogicaldrives()
            return ['{}:\\'.format(chr(ord('A') + i))
                    for i in range(26) if mask >> i & 1]
        except Exception:
            pass
    # Fallback: probe, but skip A:/B: (historically floppies -- slow to fail).
    return ['{}:\\'.format(c) for c in 'CDEFGHIJKLMNOPQRSTUVWXYZ'
            if exists('{}:\\'.format(c))]


def windows_program_roots(environ: Optional[Dict[str, str]] = None,
                          drives: Optional[Sequence[str]] = None) -> List[str]:
    """Directories that may contain a `KiCad` install dir, best first.

    The environment variables come FIRST and are the only ones that follow a
    non-default install root (a user who redirected Program Files, or a 32-bit
    python reading a 64-bit box -- hence ProgramW6432). The per-drive entries
    are the #763 backstop: KiCad on D: matches no environment variable, so
    without them it is invisible.
    """
    environ = os.environ if environ is None else environ
    roots: List[str] = []
    for var in ("ProgramW6432", "ProgramFiles", "ProgramFiles(x86)",
                "ProgramFiles(Arm)"):
        val = environ.get(var)
        if val:
            roots.append(val)
    if drives is None:
        drives = windows_drives()
    for d in drives:
        d = d if d.endswith('\\') else d + '\\'
        roots.append(ntpath.join(d, 'Program Files'))
        roots.append(ntpath.join(d, 'Program Files (x86)'))
        # A portable/extracted install at D:\KiCad. The trailing separator is
        # LOAD-BEARING: ntpath.join('D:', 'KiCad') is 'D:KiCad', a
        # drive-RELATIVE path that resolves against the process's current
        # directory on that drive and matches nothing here.
        roots.append(d)
    seen, out = set(), []
    for r in roots:
        k = r.rstrip('\\').lower()
        if k and k not in seen:
            seen.add(k)
            out.append(r)
    return out


def _winreg_kicad_roots(winreg_mod=None) -> List[str]:
    """KiCad install roots recorded in the registry, newest version first.

    The registry is AUTHORITATIVE -- it records where the installer actually
    put KiCad, on whatever drive -- so it outranks every guess below it. Two
    sources, because either can be absent on a given machine:

      * `KiCad.kicad_pcb.<ver>\\shell\\open\\command` under HKCR, whose payload
        is the pcbnew/kicad command line. Suggested by the #763 reporter.
      * `SOFTWARE\\KiCad\\<ver>` / the Uninstall entries' InstallLocation.

    Returns the KiCad VERSION directory (`...\\KiCad\\10.0`), not the exe path.
    """
    if winreg_mod is None:
        try:
            import winreg as winreg_mod  # type: ignore[no-redef]
        except Exception:
            return []
    found: List[Tuple[Tuple[int, ...], str]] = []

    def _add(path: str) -> None:
        """Reduce an exe/install path to the KiCad version dir and record it."""
        if not path:
            return
        p = path.strip().strip('"')
        # `"C:\...\pcbnew.exe" "%1"` -> the exe; then bin\ -> the version dir.
        m = re.match(r'\s*"([^"]+)"', path)
        if m:
            p = m.group(1)
        else:
            p = p.split(' ')[0]
        low = p.lower()
        if low.endswith('.exe'):
            p = ntpath.dirname(p)                 # ...\10.0\bin
            if ntpath.basename(p).lower() == 'bin':
                p = ntpath.dirname(p)             # ...\10.0
        p = p.rstrip('\\')
        if p:
            found.append((path_version_key(p + '\\'), p))

    hives = [(getattr(winreg_mod, 'HKEY_CLASSES_ROOT', None), ''),
             (getattr(winreg_mod, 'HKEY_LOCAL_MACHINE', None), r'SOFTWARE\Classes')]
    for hive, prefix in hives:
        if hive is None:
            continue
        # Enumerate KiCad.kicad_pcb.<ver> rather than assuming a version.
        try:
            root = winreg_mod.OpenKey(hive, prefix) if prefix else \
                winreg_mod.OpenKey(hive, '')
        except Exception:
            continue
        try:
            i = 0
            while True:
                try:
                    name = winreg_mod.EnumKey(root, i)
                except OSError:
                    break
                i += 1
                if not name.lower().startswith('kicad.kicad_pcb'):
                    continue
                sub = (prefix + '\\' if prefix else '') + name + \
                    r'\shell\open\command'
                try:
                    with winreg_mod.OpenKey(hive, sub) as k:
                        _add(winreg_mod.QueryValueEx(k, '')[0])
                except Exception:
                    continue
        finally:
            try:
                root.Close()
            except Exception:
                pass

    # SOFTWARE\KiCad\<ver>\ InstallLocation-style entries.
    for hive_name in ('HKEY_LOCAL_MACHINE', 'HKEY_CURRENT_USER'):
        hive = getattr(winreg_mod, hive_name, None)
        if hive is None:
            continue
        for sub in (r'SOFTWARE\KiCad', r'SOFTWARE\WOW6432Node\KiCad'):
            try:
                with winreg_mod.OpenKey(hive, sub) as k:
                    j = 0
                    while True:
                        try:
                            ver = winreg_mod.EnumKey(k, j)
                        except OSError:
                            break
                        j += 1
                        try:
                            with winreg_mod.OpenKey(hive, sub + '\\' + ver) as vk:
                                for value in ('InstallLocation', 'InstallDir', ''):
                                    try:
                                        _add(winreg_mod.QueryValueEx(vk, value)[0])
                                        break
                                    except Exception:
                                        continue
                        except Exception:
                            continue
            except Exception:
                continue

    found.sort(key=lambda t: t[0], reverse=True)
    seen, out = set(), []
    for _k, p in found:
        if p.lower() not in seen:
            seen.add(p.lower())
            out.append(p)
    return out


# --------------------------------------------------------------------------
# Install roots (the versioned KiCad directory), all platforms
# --------------------------------------------------------------------------

def darwin_bundle_patterns(home: str) -> List[str]:
    """.app bundle locations searched on macOS, best first.

    `KiCad*.app`, NOT `*.app`: /Applications/KiCad/ also holds the sibling
    launchers (GerbView.app, PCB Calculator.app, "PCB Editor.app", ...). None
    ships a Python.framework, so a bare `*.app` put 12 dead paths in front of
    every caller -- and find_kicad_python() PROBES each candidate in a
    subprocess. A user may also install the bundle loose or keep several
    versions side by side (KiCad 9.app / KiCad 10.app), so glob rather than
    assume. Shared with describe_search, so the diagnostic cannot claim to
    have searched somewhere this does not.
    """
    # posixpath, not os.path: every function here takes an explicit `system`
    # so a caller can reason about a platform other than the host, and macOS
    # paths are POSIX by construction. `os.path.join` on a Windows host emits
    # backslashes into them, which is how a cross-platform caller -- or this
    # module's own test suite -- gets `/Applications/KiCad\KiCad.app`. The
    # Windows branch of `kicad_python_candidates` already uses `ntpath.join`
    # for exactly this reason; these two were simply missed.
    return [
        '/Applications/KiCad/KiCad.app',
        posixpath.join(home, 'Applications', 'KiCad', 'KiCad.app'),
        '/Applications/KiCad/KiCad*.app',
        '/Applications/KiCad*.app',
        posixpath.join(home, 'Applications', 'KiCad', 'KiCad*.app'),
        posixpath.join(home, 'Applications', 'KiCad*.app'),
    ]


def linux_prefixes(home: str) -> List[str]:
    """Prefixes searched on Linux/BSD: distro, flatpak, snap. Shared with
    describe_search for the same reason as darwin_bundle_patterns."""
    return ['/usr', '/usr/local', '/opt/kicad',
            '/var/lib/flatpak/app/org.kicad.KiCad/current/active/files',
            posixpath.join(home, '.local', 'share', 'flatpak', 'app',
                           'org.kicad.KiCad', 'current', 'active', 'files'),
            '/snap/kicad/current/usr']


def kicad_install_roots(system: Optional[str] = None,
                        environ: Optional[Dict[str, str]] = None,
                        globber: Callable[[str], List[str]] = glob.glob,
                        isdir: Callable[[str], bool] = os.path.isdir,
                        drives: Optional[Sequence[str]] = None,
                        winreg_mod=None,
                        home: Optional[str] = None) -> List[str]:
    """Every KiCad install directory found, NEWEST VERSION FIRST.

    Windows entries are the version directory (`D:\\Program Files\\KiCad\\10.0`);
    macOS entries are the `.app` bundle; Linux entries are a prefix such as
    `/usr`. Callers derive `bin/python.exe` etc. themselves -- the layouts
    differ enough that a single "give me the python" is the wrong shape.
    """
    system = platform.system() if system is None else system
    environ = os.environ if environ is None else environ
    home = os.path.expanduser('~') if home is None else home
    out: List[str] = []

    def add(p: str) -> None:
        if p and p not in out:
            out.append(p)

    if system == 'Windows':
        # 1. the registry -- authoritative, drive-agnostic
        for p in _winreg_kicad_roots(winreg_mod):
            if isdir(p):
                add(p)
        # 2. an explicit override, for anything the rest cannot see
        for var in ('KICAD_INSTALL_DIR', 'KICAD_PATH'):
            v = environ.get(var)
            if v and isdir(v):
                add(v.rstrip('\\'))
        # 3. env-var program roots, then every fixed drive (#763)
        prog_roots = windows_program_roots(environ, drives)
        versioned: List[str] = []
        for root in prog_roots:
            versioned.extend(globber(ntpath.join(root, 'KiCad', '*')))
        versioned = [p for p in versioned if isdir(p) and path_version_key(p + '\\')]
        for p in sorted(versioned, key=lambda p: path_version_key(p + '\\'),
                        reverse=True):
            add(p)
        # 4. unversioned last resort (KiCad <= 5, or a hand-extracted tree).
        # ONLY where no versioned install was found beneath it: otherwise
        # `...\KiCad` is the CONTAINER of the real installs, not an install,
        # and listing it hands callers a `...\KiCad\bin\python.exe` that no
        # KiCad >= 6 has -- a guaranteed-dead candidate probed ahead of nothing.
        have = {ntpath.dirname(p).lower() for p in versioned}
        for root in prog_roots:
            p = ntpath.join(root, 'KiCad')
            if isdir(p) and p.lower() not in have:
                add(p)

    elif system == 'Darwin':
        pats = darwin_bundle_patterns(home)
        hits: List[str] = []
        for pat in pats:
            hits.extend(globber(pat) if '*' in pat else
                        ([pat] if isdir(pat) else []))
        # Newest first when the name carries a version; stable otherwise.
        for p in sorted(hits, key=lambda p: version_key(_basename(p)),
                        reverse=True):
            if isdir(p) and p.endswith('.app'):
                add(p)

    else:   # Linux / BSD: distro, flatpak, snap, AppImage-extracted
        for p in linux_prefixes(home):
            if isdir(p):
                add(p)
    return out


def kicad_python_candidates(system: Optional[str] = None,
                            environ: Optional[Dict[str, str]] = None,
                            this_interpreter: Optional[str] = None,
                            **kw) -> List[str]:
    """Plausible pcbnew-capable interpreters, best first (UNVERIFIED).

    Callers verify -- they want different modules (pcbnew here, pcbnew + wx for
    the GUI launchers) -- so this stays a candidate list, not an answer.
    """
    system = platform.system() if system is None else system
    environ = os.environ if environ is None else environ
    cands: List[str] = [environ.get('KICAD_PYTHON') or '']
    if this_interpreter:
        cands.append(this_interpreter)
    for root in kicad_install_roots(system=system, environ=environ, **kw):
        if system == 'Windows':
            cands.append(ntpath.join(root, 'bin', 'python.exe'))
        elif system == 'Darwin':
            fw = posixpath.join(root, 'Contents', 'Frameworks',
                                'Python.framework', 'Versions')
            cands.append(posixpath.join(fw, 'Current', 'bin', 'python3'))
            # Versioned frameworks, newest first, for a bundle whose
            # `Current` symlink is missing or broken.
            globber = kw.get('globber', glob.glob)
            cands.extend(sorted(
                (posixpath.join(v, 'bin', 'python3')
                 for v in globber(posixpath.join(fw, '3.*'))),
                key=lambda p: version_key(p), reverse=True))
        else:
            for name in ('python3', 'python'):
                cands.append(posixpath.join(root, 'bin', name))
    cands.append('/usr/bin/python3')   # distro KiCad ships pcbnew here
    seen, out = set(), []
    for c in cands:
        if c and c not in seen:
            seen.add(c)
            out.append(c)
    return out


# --------------------------------------------------------------------------
# The user-data ("Documents/KiCad") side -- where plugins are installed
# --------------------------------------------------------------------------

def windows_documents_dir(environ: Optional[Dict[str, str]] = None,
                          known_folder: Optional[Callable[[], Optional[str]]] = None,
                          winreg_mod=None,
                          home: Optional[str] = None) -> str:
    """The REAL Documents folder, the way Windows (and KiCad) resolves it.

    #763: the previous code preferred `%OneDrive%\\Documents` whenever it
    existed. OneDrive creates that folder on machines where Documents is NOT
    redirected into it, so the installer wrote the plugin somewhere KiCad never
    looks and the user had to move it by hand. Existence is not evidence of
    redirection -- only the known-folder API knows, and it is what KiCad calls.
    """
    environ = os.environ if environ is None else environ
    home = os.path.expanduser('~') if home is None else home
    if known_folder is None:
        known_folder = _shell_documents_path
    try:
        p = known_folder()
        if p:
            return p
    except Exception:
        pass
    # Registry mirror of the same known folder, for a stripped-down host where
    # the shell API is unavailable.
    if winreg_mod is None:
        try:
            import winreg as winreg_mod  # type: ignore[no-redef]
        except Exception:
            winreg_mod = None
    if winreg_mod is not None:
        for sub in (r'Software\Microsoft\Windows\CurrentVersion\Explorer\User Shell Folders',
                    r'Software\Microsoft\Windows\CurrentVersion\Explorer\Shell Folders'):
            try:
                with winreg_mod.OpenKey(winreg_mod.HKEY_CURRENT_USER, sub) as k:
                    val = winreg_mod.QueryValueEx(k, 'Personal')[0]
                    if val:
                        return os.path.expandvars(val)
            except Exception:
                continue
    return ntpath.join(home, 'Documents')


def _shell_documents_path() -> Optional[str]:      # pragma: no cover - Windows
    """SHGetKnownFolderPath(FOLDERID_Documents), or None off Windows."""
    if sys.platform != 'win32':
        return None
    import ctypes
    from ctypes import wintypes

    class GUID(ctypes.Structure):
        _fields_ = [('Data1', wintypes.DWORD), ('Data2', wintypes.WORD),
                    ('Data3', wintypes.WORD), ('Data4', ctypes.c_ubyte * 8)]

    # FOLDERID_Documents {FDD39AD0-238F-46AF-ADB4-6C85480369C7}
    fid = GUID(0xFDD39AD0, 0x238F, 0x46AF,
               (ctypes.c_ubyte * 8)(0xAD, 0xB4, 0x6C, 0x85, 0x48, 0x03, 0x69, 0xC7))
    out = ctypes.c_wchar_p()
    if ctypes.windll.shell32.SHGetKnownFolderPath(
            ctypes.byref(fid), 0, None, ctypes.byref(out)) != 0:
        return None
    try:
        return out.value
    finally:
        ctypes.windll.ole32.CoTaskMemFree(out)


def kicad_user_bases(system: Optional[str] = None,
                     environ: Optional[Dict[str, str]] = None,
                     isdir: Callable[[str], bool] = os.path.isdir,
                     globber: Callable[[str], List[str]] = glob.glob,
                     home: Optional[str] = None,
                     documents: Optional[str] = None) -> List[str]:
    """Candidate `.../KiCad` user-data directories, best first.

    Windows returns several because the machine may genuinely have more than
    one (a redirected Documents and a stale local one). `kicad_user_base`
    picks between them on EVIDENCE; this function only enumerates.
    """
    system = platform.system() if system is None else system
    environ = os.environ if environ is None else environ
    home = os.path.expanduser('~') if home is None else home
    if system == 'Linux':
        return [os.path.join(home, '.local', 'share', 'kicad')]
    if system == 'Darwin':
        return [os.path.join(home, 'Documents', 'KiCad')]
    if system != 'Windows':
        raise RuntimeError('Unsupported operating system: {}'.format(system))
    docs = windows_documents_dir(environ=environ, home=home) \
        if documents is None else documents
    cands = [ntpath.join(docs, 'KiCad')]
    onedrive = (environ.get('OneDrive') or environ.get('OneDriveConsumer')
                or environ.get('OneDriveCommercial'))
    if onedrive:
        cands.append(ntpath.join(onedrive, 'Documents', 'KiCad'))
    cands.append(ntpath.join(home, 'OneDrive', 'Documents', 'KiCad'))
    cands.append(ntpath.join(home, 'Documents', 'KiCad'))
    seen, out = set(), []
    for c in cands:
        if c.lower() not in seen:
            seen.add(c.lower())
            out.append(c)
    return out


def kicad_user_base(system: Optional[str] = None,
                    environ: Optional[Dict[str, str]] = None,
                    isdir: Callable[[str], bool] = os.path.isdir,
                    globber: Callable[[str], List[str]] = glob.glob,
                    home: Optional[str] = None,
                    documents: Optional[str] = None) -> str:
    """The `.../KiCad` user-data dir to install into.

    Chosen by EVIDENCE, not by mere existence (#763): a candidate that already
    holds a version directory (`9.0/`, `10.0/`) is one KiCad demonstrably uses,
    and beats a candidate that merely exists -- which is how an empty
    OneDrive `Documents` used to win over the real one. Falls back to the
    known-folder path so a first-ever install still lands where KiCad looks.
    """
    cands = kicad_user_bases(system=system, environ=environ, isdir=isdir,
                             globber=globber, home=home, documents=documents)
    for c in cands:                       # 1. evidence: holds a version dir
        for sub in globber(os.path.join(c, '*')):
            if version_key(os.path.basename(sub)) and isdir(sub):
                return c
    for c in cands:                       # 2. exists at all
        if isdir(c):
            return c
    return cands[0]                       # 3. where KiCad WOULD look


def kicad_user_versions(base: str,
                        globber: Callable[[str], List[str]] = glob.glob,
                        isdir: Callable[[str], bool] = os.path.isdir) -> List[str]:
    """Version directories present under a user-data base, newest first.

    Globbed rather than matched against a fixed list so a KiCad 11 install is
    found by a build of this tool that predates it (#763 "robust across
    versions").
    """
    out = []
    for p in globber(os.path.join(base, '*')):
        name = _basename(p)
        if version_key(name) and isdir(p):
            out.append(name)
    return sorted(set(out), key=version_key, reverse=True)


def describe_search(system: Optional[str] = None,
                    environ: Optional[Dict[str, str]] = None,
                    **kw) -> List[str]:
    """Human-readable account of WHERE we looked, for a failed search.

    #763: the installer reported "no KiCad version found" and nothing else, so
    a user whose KiCad was on D: had no way to tell a bad search from a bad
    install. A failed search must say what it tried.
    """
    system = platform.system() if system is None else system
    environ = os.environ if environ is None else environ
    lines = ['Platform: {}'.format(system)]
    if environ.get('KICAD_PYTHON'):
        lines.append('KICAD_PYTHON={}'.format(environ['KICAD_PYTHON']))
    if system == 'Windows':
        try:
            reg = _winreg_kicad_roots(kw.get('winreg_mod'))
        except Exception as e:
            reg = []
            lines.append('  registry lookup failed: {}'.format(e))
        lines.append('  registry install roots: {}'.format(
            ', '.join(reg) if reg else '(none recorded)'))
        roots = windows_program_roots(environ, kw.get('drives'))
        lines.append('  searched {} program root(s): {}'.format(
            len(roots), ', '.join(roots[:8]) + (' ...' if len(roots) > 8 else '')))
        lines.append('  Documents resolves to: {}'.format(
            windows_documents_dir(environ=environ, home=kw.get('home'))))
    else:
        # WHERE we looked, not what we found: an empty "searched: ..." line
        # made from the RESULT tells the operator nothing about the search,
        # which is the one thing a failed search has to explain.
        home = kw.get('home') or os.path.expanduser('~')
        places = (darwin_bundle_patterns(home) if system == 'Darwin'
                  else linux_prefixes(home))
        lines.append('  searched {} location(s):'.format(len(places)))
        for p in places:
            lines.append('    {}'.format(p))
        found = kicad_install_roots(system=system, environ=environ, **kw)
        lines.append('  found: {}'.format(', '.join(found) if found
                                          else '(no install directory)'))
    return lines
